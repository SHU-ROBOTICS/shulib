################################################################################
######################### User configurable parameters #########################
# filename extensions
CEXTS:=c
ASMEXTS:=s S
CXXEXTS:=cpp c++ cc

# probably shouldn't modify these, but you may need them below
ROOT=.
FWDIR:=$(ROOT)/firmware
BINDIR=$(ROOT)/bin
SRCDIR=$(ROOT)/src
INCDIR=$(ROOT)/include

# src/'s OWN INCLUDE ROOT (chunk SRC1, 2026-09-27). src/ is organized by job: robots/ (what each
# robot IS), team/ (the team's code), programs/ (one program each), common/ (what the programs
# share), with main.cpp and build_selection.hpp at the top. Every include of a src/ file is
# written FROM src/ -- `#include "robots/chassis_table.hpp"`, `#include "common/boot_source.hpp"`,
# `#include "build_selection.hpp"` -- the same in every folder, in main.cpp and in the host tests
# (test/CMakeLists.txt gives that build the same root). common.mk's INCLUDE line turns this into
# -iquote"./src", searched AFTER -iquote"./include".
# -iquote and not -I: every src/ include is a quoted one, and -iquote adds src/ to the quoted
# search only, where -I would also answer <...> includes meant for the toolchain and the vendored
# SDK. Search ORDER is not what keeps a same-named header from shadowing a src/ one (or a src/
# header from shadowing a library one): tools/src_build_gate.py's step 0 [include-root] FAILS any
# src/ path that also exists under include/, test/pros_shim/, test/vendor/ or test/, or beside its
# includer in a src/ subfolder, and any src/ include not written from src/ -- so no file can be
# found by two roots in either build, and the order never decides.
EXTRA_INCDIR:=$(SRCDIR)

# src/ compiled with NO warning flags until 2026-08-18 -- common.mk sets only
# -Wno-psabi. The library headers get the full strict set in CI's ARM gate, but
# src/main.cpp and src/programs/bench_r3a.cpp got nothing, and a data abort shipped to the
# robot as a result. Enabled here WITHOUT -Werror; tools/src_build_gate.py is what fails
# a build on a warning: EVERY warning in our code fails it (vendored headers are ignored by
# path), -Wunused-function included since G1 Part 2c's final run (the team lead's ruling,
# 2026-09-26). That category was allowed from GATE1 until then, as the evidence that the
# invented X-drive wiring in src/main.cpp was dead code in the bench and tank builds (robot()
# and portMapString() warned there, and the gate counted them); G1 Part 2c made the X-drive
# data (src/robots/xdrive_config.hpp) built in its own file, NO build emits the warning any more, and
# a NEW dead function in src/ now fails the gate instead of passing it.
# What replaced the warning count as the proof that a build is the variant it claims: the
# identity BEACONS -- the variant beacon, typed in the same branch of src/build_selection.hpp's
# ONE #if chain that sets the program and the robot, and the robot beacon derived from that
# robot (the gate's steps 0 and 6) -- and the PROGRAM read from the linked ELF's symbol table
# (step 8): the variant beacon vouches for the branch, the robot beacon for the robot the branch
# set, step 8 for the program it set and for what each entry point calls.
#
# Honest limit, stated so nobody trusts this further than it goes: -Wformat=2 did
# NOT catch the bug that motivated it. std::string_view is trivially copyable, so
# passing one to a printf %s compiles silently. That hazard is guarded by a comment
# at the call site, not by the compiler.
WARNFLAGS+=-Wall -Wextra -Wformat=2
EXTRA_CFLAGS=
EXTRA_CXXFLAGS=

# ── The §18.5 build hash (chunk R1a — the robot half of what test/CMakeLists.txt
# does for the host suite). The session header must say WHICH BINARY ran; code
# cannot know its own commit, so the BUILD defines it. `--dirty` is load-bearing:
# a clean-looking hash from a modified tree is a WRONG hash, worse than an absent
# one. Evaluated per `make` run (a plain := $(shell ...) at parse time), so the
# robot package can never carry a stale hash the way a configure-time value
# could. If git is unavailable the macro is NOT defined and diag/build_info.hpp's
# loud MISSING path runs — never a silently plausible placeholder.
SHULIB_GIT_HASH:=$(shell git describe --always --dirty --abbrev=7 2>/dev/null)
ifneq ($(SHULIB_GIT_HASH),)
override EXTRA_CXXFLAGS+=-DSHULIB_BUILD_HASH=\"$(SHULIB_GIT_HASH)\"
endif

# ── WHICH ROBOT IS THIS BINARY FOR? (chunk GATE1; third variant at R3b Session 2) ──
# ROBOT selects the robot a build is for -- read, with PROGRAM below, by ONE #if chain in
# src/build_selection.hpp (and nowhere else in the tree), which sets the program, the robot's tables
# and the identity beacon together:
#     make                → ROBOT=bench  — the measured tank BENCH bot: boots the bench tester
#     make ROBOT=tank     → robot two: boots the bench tester too. Since ROB1-BT (2026-11-16) robot two
#                            is robot one's TWIN, the same cascade build (its claws and its signed
#                            chassis table retired): its tables are robot one's design, every number
#                            from src/robots/robot_two_facts.hpp. Add PROGRAM=drive (below) for robot
#                            two's driver program, "shulib Drive". EVERY robot-two build REFUSES,
#                            naming each OWED fact its program needs, until its facts are typed —
#                            exactly as robot one's below.
#     make ROBOT=one      → the 2026 CASCADE robot, robot one (ROB1-A, 2026-11-11): boots the bench
#                            tester over its tables (src/robots/chassis_table.hpp's kRobotOneChassis,
#                            mostly UNSET until Calibrate measures it; src/robots/mechanism_table.hpp's
#                            robot_one region, every number from src/robots/robot_one_facts.hpp). Add
#                            PROGRAM=drive|calibrate|autontest|competition|tune as for robot two. EVERY
#                            robot-one build REFUSES, naming each OWED fact its program needs, until
#                            the builder's facts are typed (docs/robot-facts-form.md). Nothing has
#                            run on robot one.
#     make ROBOT=xdrive   → the invented X-drive (HA-111): src/robots/xdrive_config.hpp, built on
#                            robot::Robot by src/programs/xdrive_program.cpp since G1 Part 2c. Its
#                            ports are invented, so on every robot that exists it refuses at
#                            boot, naming the first port, with nothing powered.
# Any other value is an $(error), NOT a silent default: the previous mechanism was
# "make CXXFLAGS_EXTRA=-DSHULIB_ROBOT_XDRIVE_INVENTED" as documented in src/main.cpp —
# but common.mk consumes EXTRA_CXXFLAGS, the names were transposed, and the documented
# command silently built the BENCH variant (measured 2026-08-19, GATE1 §4.1). A variant
# switch must be a named, validated variable of its own so a typo fails loudly and the
# selection can never collide with the flag list carrying the build hash.
#
# Both `override ... +=` appends (here and the hash above) are deliberate: a plain `+=`
# is silently DISCARDED when EXTRA_CXXFLAGS is set on the make command line — that is
# exactly how `make EXTRA_CXXFLAGS=...` used to ship a binary with NO build hash
# (GATE1 §4.1, second half). With `override`, the hash and the variant define land on
# top of any command-line value instead of vanishing.
ROBOT?=bench
ifeq ($(ROBOT),bench)
# src/build_selection.hpp's chain defaults to the bench tank when neither variant define is set.
else ifeq ($(ROBOT),xdrive)
override EXTRA_CXXFLAGS+=-DSHULIB_ROBOT_XDRIVE_INVENTED
else ifeq ($(ROBOT),tank)
override EXTRA_CXXFLAGS+=-DSHULIB_ROBOT_TANK_2026
else ifeq ($(ROBOT),one)
override EXTRA_CXXFLAGS+=-DSHULIB_ROBOT_ONE_2026
else
$(error unknown ROBOT '$(ROBOT)' — valid values: bench (default), xdrive, tank, one)
endif

# ── THE ROBOTS WITH MECHANISMS (TUNE1-B1, 2026-11-10) ──
# The ROBOT values whose mechanism table (src/robots/mechanism_table.hpp, mechanismTableFor()) is NOT
# empty — the robots "shulib Tune" (PROGRAM=tune, below) can be built for. A robot that registers a
# mechanism table adds its ROBOT value HERE (and its chain branch in src/build_selection.hpp); the tune
# row itself names no robot. tools/src_build_gate.py's step 0 derives the same set from the table file
# and FAILS when this list says otherwise (a robot listed with no mechanisms, or one with mechanisms
# missing), and src/programs/tune_program.cpp's static_assert stops a Tune build of an empty table.
# `tank` is robot two (robot one's twin since ROB1-BT; its claws until then); `one` is robot one (ROB1-A,
# added at its integration onto TUNE1-B1, 2026-11-13). Each robot's Tune build, like its other programs,
# REFUSES while a fact it needs is owed.
MECHANISM_ROBOTS:=tank one

# ── WHICH PROGRAM? (R3b Part 0b, 2026-09-10 — the second build axis; third value at Parts 1–3) ──
# PROGRAM selects which program a build is (what its PROS entry points run) — robot two's, and `tune`
# for any robot in MECHANISM_ROBOTS (above):
#     make ROBOT=tank                   → PROGRAM=tester — "Bench Tests": the read-only tester with its
#                                         gated DRIVE station (slot 3, `pros upload --slot 3`)
#     make ROBOT=tank PROGRAM=drive     → "shulib Drive" (T4 Part 3, 2026-10-01; it REPLACED the
#                                         R3b Part 0b program of that name in place): robot two's
#                                         DRIVER FILE (src/team/robot_two_driver.hpp) on robot::Robot
#                                         over the PROS device provider, built from the CALIBRATION
#                                         RECORD on the SD card (src/common/boot_source.{hpp,cpp}),
#                                         through teleop::runDriverControl with the library's drive
#                                         protections, its log copied to /usd/drive_log.txt
#                                         (src/programs/drive_program.cpp; slot 1,
#                                         `pros upload --slot 1 --name "shulib Drive"`). Refuses at
#                                         boot when no source can describe a drivetrain, or when the
#                                         record does not match the ports. It drives toward the
#                                         RECORD's front — the OTHER end from the old "shulib Drive".
#     make ROBOT=tank PROGRAM=library   → RETIRED at T4 Part 3: "shulib Teleop" was replaced by
#                                         "shulib Drive", whose DETAILS page carries its diagnostics
#                                         (the team lead's ruling R5). An $(error) below says so; the
#                                         define is NOT re-pointed at another program.
#     make ROBOT=tank PROGRAM=calibrate → "shulib Calibrate" (R3d Part 2, 2026-09-18; the library's
#                                         standard Calibrate since CAL1 Part B, 2026-11-05, over
#                                         src/robots/calibrate_facts.hpp): the robot
#                                         MEASURES ITS OWN drivetrain — which ports drive, which
#                                         way each is wired, which side each is on, inches per
#                                         radian per side, the effective track width — and writes
#                                         the answer to /usd/shulib_drivetrain.bin (a verified
#                                         backup, shulib_drivetrain.bak, first) for "shulib
#                                         Drive" to boot from (slot 4,
#                                         `pros upload --slot 4 --name "shulib Calibrate"`).
#                                         It POWERS THE MOTORS. R3d's own Calibrate ran on the
#                                         old robot two from 2026-09-24; the standard one has not
#                                         run on a robot. Since ROB1-BT the record it writes NAMES
#                                         ITS ROBOT, and the boot refuses a record naming another
#                                         robot, or none.
#     make ROBOT=tank PROGRAM=autontest → "shulib Auton Tester" (COMP-t, 2026-09-28): runs ONE of
#                                         the team's autons (src/team/autons.cpp) on the bench from
#                                         RUN on the brain's screen — a 3 s countdown, then
#                                         competition::runAuton, the competition program's own
#                                         call; any controller button or a touch aborts
#                                         (src/programs/auton_tester_program.cpp; slot 2 since
#                                         T4 Part 3 retired Teleop — slot 7 until then, T4's N4).
#                                         It has not run on a robot.
#     make ROBOT=tank PROGRAM=competition → "shulib Competition" (COMP-b, 2026-09-30): THE
#                                         COMPETITION PROGRAM — the auton chosen on the brain's
#                                         menu runs in autonomous() on the program's own task
#                                         (competition::runAuton, the tester's call), a disable or
#                                         the end of autonomous ends it, then driver control runs
#                                         the team's driver file, its log appended to
#                                         /usd/comp_log.txt (src/programs/competition_program.cpp;
#                                         slot 6 — slot 5 is held for the team's old-code fallback,
#                                         T4 Part 0). It has not run on a robot.
#     make ROBOT=<a robot in MECHANISM_ROBOTS> PROGRAM=tune → "shulib Tune" (TUNE1-B1, 2026-11-11):
#                                         LIVE TUNING of the robot's MECHANISMS on the brain screen
#                                         — never the drive (include/shulib/mechanism/tune_program.hpp
#                                         through src/programs/tune_program.cpp; slot 7; its row
#                                         below). It has not run on a robot.
# Six programs in six slots rather than one program with a chooser (team lead's ruling): the
# operator picks a program by NAME from the brain's slot list, and a program called Drive that
# hides the tester behind a timeout is one more thing to explain at a field.
# (ROB1-A, 2026-11-11: robot one — `make ROBOT=one PROGRAM=...` — takes the same four, in the same
# slots, over its own driver file src/team/robot_one_driver.hpp and autons src/team/robot_one_autons.cpp;
# every sentence below that says "robot two" holds for robot one as well. Since its integration onto
# TUNE1-B1 robot one is in MECHANISM_ROBOTS too, so `make ROBOT=one PROGRAM=tune` is its Tune, slot 7.)
# `drive`, `calibrate`, `autontest` and `competition` are only meaningful on robot two — `drive`
# because only robot two has a driver file and a calibration record, `calibrate` because the
# calibrator only supports a tank drivetrain and takes the cartridge from this robot's chassis
# table (the standard Calibrate since CAL1 Part B, src/robots/calibrate_facts.hpp), `autontest` and `competition` because the team's autons (and, for `competition`, its
# driver file) are robot two's and only robot two boots from a calibration record; so any of the
# four with any other ROBOT is an $(error), not a program that refuses at boot. Any other PROGRAM
# value is an $(error) too — the same silent-no-op class GATE1 closed for ROBOT — and so is the
# RETIRED `library`, by name. tools/src_build_gate.py builds this axis as its fourth to eighth
# entries (tank-drive, tank-calibrate, tank-autontest, tank-competition, tank-tune) — and robot one's as
# its tenth to fourteenth (one-drive … one-tune, after `one`) — asserts each define
# lands token-exact and the beacons are in the ELF, and fails the retired define or beacon
# anywhere.
PROGRAM?=tester
ifeq ($(PROGRAM),tester)
# the default: no define; src/build_selection.hpp's chain picks the bench tester on bench and tank
else ifeq ($(PROGRAM),drive)
ifeq ($(filter tank one,$(ROBOT)),)
$(error PROGRAM=drive needs ROBOT=tank or ROBOT=one — only robot two and robot one have a driver file and a calibration record; '$(ROBOT)' cannot drive)
endif
override EXTRA_CXXFLAGS+=-DSHULIB_PROGRAM_DRIVE
else ifeq ($(PROGRAM),library)
$(error PROGRAM=library was retired at T4: "shulib Teleop"'s diagnostics are "shulib Drive"'s DETAILS page -- use make ROBOT=tank PROGRAM=drive)
else ifeq ($(PROGRAM),calibrate)
ifeq ($(filter tank one,$(ROBOT)),)
$(error PROGRAM=calibrate needs ROBOT=tank or ROBOT=one — the calibrator supports only a tank drivetrain and takes the cartridge from this robot's chassis table; '$(ROBOT)' cannot be calibrated by it)
endif
override EXTRA_CXXFLAGS+=-DSHULIB_PROGRAM_CALIBRATE
else ifeq ($(PROGRAM),autontest)
ifeq ($(filter tank one,$(ROBOT)),)
$(error PROGRAM=autontest needs ROBOT=tank or ROBOT=one — the team's autons are robot two's and robot one's; '$(ROBOT)' has none)
endif
override EXTRA_CXXFLAGS+=-DSHULIB_PROGRAM_AUTONTEST
else ifeq ($(PROGRAM),competition)
ifeq ($(filter tank one,$(ROBOT)),)
$(error PROGRAM=competition needs ROBOT=tank or ROBOT=one — the team's autons and driver files are robot two's and robot one's; '$(ROBOT)' has neither)
endif
override EXTRA_CXXFLAGS+=-DSHULIB_PROGRAM_COMPETITION
else ifeq ($(PROGRAM),tune)
# "shulib Tune" (TUNE1-B1, 2026-11-10): LIVE TUNING of the robot's MECHANISMS on the brain screen
# (include/shulib/mechanism/tune_program.hpp, through src/programs/tune_program.cpp; slot 7,
# `pros upload --slot 7 --name "shulib Tune"`). It makes ONLY the mechanisms, never the drive. ROBOT-
# GENERIC (the MECH-S/TUNE1 brief's re-check #10): accepted for ANY robot in MECHANISM_ROBOTS (above),
# never a robot-two-only row — a robot gets Tune by registering its mechanism table. It has not run on
# a robot.
ifeq ($(filter $(ROBOT),$(MECHANISM_ROBOTS)),)
$(error PROGRAM=tune needs a ROBOT whose mechanism table is not empty — '$(ROBOT)' declares no mechanism (src/robots/mechanism_table.hpp), and the Tune program builds ONLY mechanisms; robots with mechanisms: $(MECHANISM_ROBOTS))
endif
override EXTRA_CXXFLAGS+=-DSHULIB_PROGRAM_TUNE
else
$(error unknown PROGRAM '$(PROGRAM)' — valid values: tester (default), drive, calibrate, autontest, competition, tune)
endif

# Pin the language standards to ones arm-none-eabi-gcc 13.2 accepts. The PROS template's common.mk
# defaults to gnu++26 / gnu23 (gcc 14+ spellings); 13.2 wants gnu++20 (matches the host-test build)
# and gnu2x. Set here (before common.mk's `?=`) so it wins and survives kernel-template updates.
CXX_STANDARD:=gnu++20
C_STANDARD:=gnu2x

# Set to 1 to enable hot/cold linking
USE_PACKAGE:=1

# Add libraries you do not wish to include in the cold image here
# EXCLUDE_COLD_LIBRARIES:= $(FWDIR)/your_library.a
EXCLUDE_COLD_LIBRARIES:= 

# Set this to 1 to add additional rules to compile your project as a PROS library template
IS_LIBRARY:=0
# TODO: CHANGE THIS! 
# Be sure that your header files are in the include directory inside of a folder with the
# same name as what you set LIBNAME to below.
LIBNAME:=libbest
VERSION:=1.0.0
# EXCLUDE_SRC_FROM_LIB= $(SRCDIR)/unpublishedfile.c
# this line excludes opcontrol.c and similar files
EXCLUDE_SRC_FROM_LIB+=$(foreach file, $(SRCDIR)/main,$(foreach cext,$(CEXTS),$(file).$(cext)) $(foreach cxxext,$(CXXEXTS),$(file).$(cxxext)))

# files that get distributed to every user (beyond your source archive) - add
# whatever files you want here. This line is configured to add all header files
# that are in the directory include/LIBNAME
TEMPLATE_FILES=$(INCDIR)/$(LIBNAME)/*.h $(INCDIR)/$(LIBNAME)/*.hpp

.DEFAULT_GOAL=quick

################################################################################
################################################################################
########## Nothing below this line should be edited by typical users ###########
# ── THE VARIANT STAMP (ROB1-A, 2026-11-11; the ROB1 brief's re-check R1) ──
# common.mk's objects depend on their sources and headers, NOT on ROBOT, PROGRAM or the flags, so
# until this stamp a `make ROBOT=tank PROGRAM=drive` after a `make ROBOT=tank` compiled NOTHING and
# re-linked the previous program's objects (guide chapter 16 used to say "rm -rf bin .d" for exactly
# that). Robot one's facts refusal (src/robots/robot_one_facts.hpp) fires only when a file compiles,
# so without the stamp a refused robot-one build could leave the LAST variant's package in bin/ to
# upload. So: the build's ROBOT, PROGRAM and EXTRA_CXXFLAGS (the variant defines AND the build hash)
# are written to .d/shulib-variant; when they differ from the last build's, bin/ and .d/ are deleted
# BEFORE anything compiles. Every change of robot, program or commit recompiles everything (~8 s),
# and a refused build leaves NO package. `make clean` is untouched. The stamp is not compiled in, so
# no package changes. Never remove it to "speed up" a build (the ROB1 brief's landmine 20).
SHULIB_VARIANT_STAMP := ROBOT=$(ROBOT) PROGRAM=$(PROGRAM) $(EXTRA_CXXFLAGS)
ifeq ($(filter clean,$(MAKECMDGOALS)),)
ifneq ($(file <.d/shulib-variant),$(SHULIB_VARIANT_STAMP))
$(info shulib: the build changed (ROBOT=$(ROBOT) PROGRAM=$(PROGRAM)) -- removing bin/ and .d/ so every file recompiles)
$(shell rm -rf bin .d && mkdir -p .d)
$(file >.d/shulib-variant,$(SHULIB_VARIANT_STAMP))
endif
endif
-include ./common.mk
