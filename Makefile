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

# src/ compiled with NO warning flags until 2026-08-18 -- common.mk sets only
# -Wno-psabi. The library headers get the full strict set in CI's ARM gate, but
# src/main.cpp and src/bench_r3a.cpp got nothing, and a data abort shipped to the
# robot as a result. Enabled here, deliberately WITHOUT -Werror: in the bench
# and tank builds the X-drive helpers (robot(), portMapString()) are legitimately
# unused, and two -Wunused-function warnings are the correct, informative
# output -- they are how you can see that the invented wiring really is dead code.
# (There were three until R3b Session 2 moved shaped() into the PROS-free
# shulib/teleop/stick_mapping.hpp; tools/src_build_gate.py asserts >= 1, not 3.)
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
# ROBOT selects the variant src/main.cpp builds:
#     make                → ROBOT=bench  — the measured tank BENCH bot: boots the bench tester
#     make ROBOT=tank     → the 2026 tank chassis, robot two: boots the bench tester too, over
#                            its MEASURED, SIGNED chassis table (src/chassis_table.hpp, shared
#                            with the drive program; no library graph is built for it yet).
#                            Add PROGRAM=drive (below) for the program that just drives it.
#     make ROBOT=xdrive   → the invented X-drive wiring (HA-111; cannot boot on the bench bot)
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
# src/main.cpp defaults to the bench tank when neither variant define is set.
else ifeq ($(ROBOT),xdrive)
override EXTRA_CXXFLAGS+=-DSHULIB_ROBOT_XDRIVE_INVENTED
else ifeq ($(ROBOT),tank)
override EXTRA_CXXFLAGS+=-DSHULIB_ROBOT_TANK_2026
else
$(error unknown ROBOT '$(ROBOT)' — valid values: bench (default), xdrive, tank)
endif

# ── WHICH PROGRAM? (R3b Part 0b, 2026-09-10 — the second build axis) ──
# PROGRAM selects what opcontrol() runs on a tester-variant robot:
#     make ROBOT=tank                 → PROGRAM=tester — "Bench Tests": the read-only tester with its
#                                       gated DRIVE station (slot 3, `pros upload --slot 3`)
#     make ROBOT=tank PROGRAM=drive   → "shulib Drive": the program that JUST DRIVES robot two from
#                                       the sticks through the hal/pros adapters, with dead-port
#                                       tolerance (src/drive_program.cpp; slot 1,
#                                       `pros upload --slot 1 --name "shulib Drive"`)
# Two programs in two slots rather than one program with a chooser (team lead's ruling): the
# operator picks a program by NAME from the brain's slot list, and a program called Drive that
# hides the tester behind a timeout is one more thing to explain at a field.
# `drive` is only meaningful with a SIGNED chassis table, and only robot two's is (its signs
# were measured 2026-09-10); so `PROGRAM=drive` with any other ROBOT is an $(error), not a
# program that refuses at boot. Any other PROGRAM value is an $(error) too — the same silent-
# no-op class GATE1 closed for ROBOT. tools/src_build_gate.py builds this axis as its fourth
# entry (tank-drive) and asserts the define lands token-exact and the beacon is in the ELF.
PROGRAM?=tester
ifeq ($(PROGRAM),tester)
# the default: no define; src/main.cpp runs the bench tester on bench and tank
else ifeq ($(PROGRAM),drive)
ifneq ($(ROBOT),tank)
$(error PROGRAM=drive needs ROBOT=tank — only robot two's chassis table carries measured signs; '$(ROBOT)' cannot drive)
endif
override EXTRA_CXXFLAGS+=-DSHULIB_PROGRAM_DRIVE
else
$(error unknown PROGRAM '$(PROGRAM)' — valid values: tester (default), drive)
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
