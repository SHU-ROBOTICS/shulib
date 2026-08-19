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
# build the X-drive helpers (robot(), portMapString(), shaped()) are legitimately
# unused, and three -Wunused-function warnings are the correct, informative
# output -- they are how you can see that the invented wiring really is dead code.
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

# ── WHICH ROBOT IS THIS BINARY FOR? (chunk GATE1) ──────────────────────────────
# ROBOT selects the variant src/main.cpp builds:
#     make                → ROBOT=bench  — the tank bench bot (the only robot that exists)
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
# src/main.cpp defaults to the bench tank when SHULIB_ROBOT_XDRIVE_INVENTED is undefined.
else ifeq ($(ROBOT),xdrive)
override EXTRA_CXXFLAGS+=-DSHULIB_ROBOT_XDRIVE_INVENTED
else
$(error unknown ROBOT '$(ROBOT)' — valid values: bench (default), xdrive)
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
