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
#     make ROBOT=tank     → the 2026 tank chassis, robot two: boots the bench tester too, over
#                            its MEASURED, SIGNED chassis table (src/robots/chassis_table.hpp, shared
#                            with the drive program and the library program). Add PROGRAM=drive
#                            (below) for the program that just drives it, or PROGRAM=library for
#                            the program in which THE LIBRARY drives it (R3b Parts 1–3).
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
else
$(error unknown ROBOT '$(ROBOT)' — valid values: bench (default), xdrive, tank)
endif

# ── WHICH PROGRAM? (R3b Part 0b, 2026-09-10 — the second build axis; third value at Parts 1–3) ──
# PROGRAM selects what opcontrol() runs on robot two:
#     make ROBOT=tank                   → PROGRAM=tester — "Bench Tests": the read-only tester with its
#                                         gated DRIVE station (slot 3, `pros upload --slot 3`)
#     make ROBOT=tank PROGRAM=drive     → "shulib Drive": the program that JUST DRIVES robot two from
#                                         the sticks through the hal/pros adapters, with dead-port
#                                         tolerance (src/programs/drive_program.cpp; slot 1,
#                                         `pros upload --slot 1 --name "shulib Drive"`)
#     make ROBOT=tank PROGRAM=library   → "shulib Teleop" (R3b Parts 1–3, 2026-09-14; rebuilt on
#                                         robot::Robot at G1 Part 2b, 2026-09-26): THE LIBRARY
#                                         drives robot two — src/programs/teleop_program.cpp, robot::Robot
#                                         over the PROS device provider, built from the CALIBRATION
#                                         RECORD on the SD card when there is one, else from
#                                         src/robots/chassis_table.hpp (src/common/boot_source.{hpp,cpp}), through
#                                         teleop::runDriverControl and Chassis::drive() (slot 2,
#                                         `pros upload --slot 2 --name "shulib Teleop"`). Refuses
#                                         at boot, on screen, when neither source can describe a
#                                         drivetrain, or when the record does not match the ports.
#     make ROBOT=tank PROGRAM=calibrate → "shulib Calibrate" (R3d Part 2, 2026-09-18): the robot
#                                         MEASURES ITS OWN drivetrain — which ports drive, which
#                                         way each is wired, which side each is on, inches per
#                                         radian per side, the effective track width — and writes
#                                         the answer to /usd/shulib_drivetrain.bin for "shulib
#                                         Teleop" to boot from (slot 4,
#                                         `pros upload --slot 4 --name "shulib Calibrate"`).
#                                         It POWERS THE MOTORS. It has run on robot two since
#                                         2026-09-24 (its recorded reproduction run is still owed).
# Four programs in four slots rather than one program with a chooser (team lead's ruling): the
# operator picks a program by NAME from the brain's slot list, and a program called Drive that
# hides the tester behind a timeout is one more thing to explain at a field.
# `drive`, `library` and `calibrate` are only meaningful on robot two — `drive` and `library`
# because only its chassis table carries measured signs, `calibrate` because the calibrator only
# supports a tank drivetrain and takes the cartridge from that table; so any of the three with any
# other ROBOT is an $(error), not a program that refuses at boot. Any other PROGRAM value is an
# $(error) too — the same silent-no-op class GATE1 closed for ROBOT. tools/src_build_gate.py
# builds this axis as its fourth, fifth and sixth entries (tank-drive, tank-library,
# tank-calibrate) and asserts each define lands token-exact and the beacons are in the ELF.
PROGRAM?=tester
ifeq ($(PROGRAM),tester)
# the default: no define; src/build_selection.hpp's chain picks the bench tester on bench and tank
else ifeq ($(PROGRAM),drive)
ifneq ($(ROBOT),tank)
$(error PROGRAM=drive needs ROBOT=tank — only robot two's chassis table carries measured signs; '$(ROBOT)' cannot drive)
endif
override EXTRA_CXXFLAGS+=-DSHULIB_PROGRAM_DRIVE
else ifeq ($(PROGRAM),library)
ifneq ($(ROBOT),tank)
$(error PROGRAM=library needs ROBOT=tank — only robot two has a calibration record and a signed chassis table; '$(ROBOT)' has no library graph)
endif
override EXTRA_CXXFLAGS+=-DSHULIB_PROGRAM_LIBRARY
else ifeq ($(PROGRAM),calibrate)
ifneq ($(ROBOT),tank)
$(error PROGRAM=calibrate needs ROBOT=tank — the calibrator supports only a tank drivetrain and takes the cartridge from robot two's chassis table; '$(ROBOT)' cannot be calibrated by it)
endif
override EXTRA_CXXFLAGS+=-DSHULIB_PROGRAM_CALIBRATE
else
$(error unknown PROGRAM '$(PROGRAM)' — valid values: tester (default), drive, library, calibrate)
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
-include ./common.mk
