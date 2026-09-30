#!/usr/bin/env python3
"""src build gate — src/ compiles AND links for the V5, every robot variant (GATE1; third
variant at R3b Session 2; a second axis, the PROGRAM, at R3b Part 0b; its third value, the
LIBRARY program, at R3b Parts 1–3; its fourth, the CALIBRATE program, at R3d Part 2; its fifth,
the AUTON TESTER, at COMP-t).

WHY THIS EXISTS
---------------
CI cross-compiles every header under include/shulib/ for the V5 and holds that line
hard — but until GATE1 it never compiled src/ itself: not main.cpp, not bench_r3a.cpp.
The host build does not either (test/CMakeLists.txt globs test/*_test.cpp). The only
thing standing between a broken src/ and the robot was somebody running `make` by hand,
and the Makefile's own comment records what that cost: src/ compiled with NO warning
flags until 2026-08-18, and a data abort shipped to the robot as a result.

This gate drives the REAL `make` — never a re-implemented compile line, which would be
a second source of truth drifting from common.mk — and it is the real BUILD, link
included: measured 2026-08-19, the old soft-float firmware/liblvgl ABI blocker is gone
and `make` links end to end at arm-none-eabi-g++ 13.2.1 with CXX_STANDARD:=gnu++20.

THE BUILDS (the Makefile's validated ROBOT switch and PROGRAM switch; anything else is an
$(error))
-----------------------------------------------------------------------------------
  bench       ROBOT=bench (default)      the measured tank bench bot — boots the bench tester
  xdrive      ROBOT=xdrive               the invented X-drive (HA-111) — since G1 Part 2c a
                                         config constant (src/robots/xdrive_config.hpp) built on
                                         robot::Robot by src/programs/xdrive_program.cpp
                                                              -DSHULIB_ROBOT_XDRIVE_INVENTED
  tank        ROBOT=tank                 the 2026 tank chassis, robot two — boots the bench
                                         tester over its SIGNED chassis table
                                                              -DSHULIB_ROBOT_TANK_2026
  tank-drive  ROBOT=tank PROGRAM=drive   "shulib Drive" (R3b Part 0b): robot two drives from
                                         the sticks through the hal/pros adapters — its own
                                         program in its own slot
                                                              -DSHULIB_ROBOT_TANK_2026
                                                              -DSHULIB_PROGRAM_DRIVE
  tank-library ROBOT=tank PROGRAM=library "shulib Teleop" (R3b Parts 1–3; on robot::Robot since
                                         G1 Part 2b): THE LIBRARY drives robot two —
                                         src/programs/teleop_program.cpp, robot::Robot over the PROS device
                                         provider (two MotorGroups, drive-encoder odometry,
                                         Chassis), built from the SD card's calibration record
                                         when there is one, else from the chassis table; slot 2
                                                              -DSHULIB_ROBOT_TANK_2026
                                                              -DSHULIB_PROGRAM_LIBRARY
  tank-calibrate ROBOT=tank PROGRAM=calibrate "shulib Calibrate" (R3d Part 2): the robot MEASURES
                                         its own drivetrain (src/programs/calibrate_program.cpp) and
                                         writes /usd/shulib_drivetrain.bin; slot 4
                                                              -DSHULIB_ROBOT_TANK_2026
                                                              -DSHULIB_PROGRAM_CALIBRATE
  tank-autontest ROBOT=tank PROGRAM=autontest "shulib Auton Tester" (COMP-t): runs ONE of the
                                         team's autons (src/team/autons.cpp) from RUN on the
                                         brain's screen through competition::runAuton
                                         (src/programs/auton_tester_program.cpp)
                                                              -DSHULIB_ROBOT_TANK_2026
                                                              -DSHULIB_PROGRAM_AUTONTEST
  (PROGRAM=drive, =library, =calibrate or =autontest with any other ROBOT is a Makefile
   $(error): only robot two has a signed table, only a tank can be calibrated by this
   procedure, and the team's autons are robot two's.)

WHAT `check` DOES, per build (bench, xdrive, tank, tank-drive, tank-library, tank-calibrate,
tank-autontest)
--------------------------------------------------------------
 1. Removes bin/ and .d/ (exactly what `make clean` removes; both gitignored, so the
    tree stays clean). make is INCREMENTAL: a gate that compiles nothing reports green,
    and that stale-artifact class nearly faked a mutation result in R3b. Freshness is
    therefore structural (delete) AND observable (the compiled-TU count is printed and
    checked against the TU list globbed from src/ itself — generated, never enumerated,
    the A4 precedent: an enumerated gate list rots).
 2. Runs `make ROBOT=<variant> V=1`. V=1 makes common.mk's test_output_2 print every
    full compiler command line, which is where the structural assertions read from.
    EXTRA_CXXFLAGS is NEVER passed on the command line — that is precisely how the
    build hash used to get dropped (GATE1 §4.1).
 3. Fails on any compiler/linker error, naming the file.
 4. Applies the warning policy below.
 5. Asserts the variant define and the build hash really landed (structural: the -D
    flags on every C++ compile line; end-to-end: the `git describe` string must appear
    in the bytes of bin/hot.package.elf). This encodes the §4.1 defect permanently:
    the documented variant flag used to be a silent no-op, and the one working
    invocation silently shipped a hash-less binary.
    THE DEFINE CHECK IS TOKEN-EXACT (R3b Session 2): the compile line is split on
    whitespace and every define in the build's SET must be a WHOLE token, and every define
    that belongs to some other build and not to this one must be absent as a token (since
    Part 0b a build carries a SET of defines: tank-drive carries two). A substring check was
    defeated by GATE1's own mutation 5b (a look-alike `-DSHULIB_ROBOT_XDRIVE_INVENTED_BROKEN`),
    which the old warning-count detector caught for xdrive only (see "the variant
    differential, retired" below) — so the structural check has to be exact on its own.
 6. Asserts the IDENTITY BEACONS — two per build.
    THE VARIANT BEACON (R3b Session 2): src/build_selection.hpp defines one string per build —
    `shulib-robot-variant=bench|xdrive|tank|tank-drive|tank-library|tank-calibrate|tank-autontest`
    —
    in THE SAME BRANCH of THE ONE #if chain that sets the program (`kProgram`) and the robot
    (`kRobotId`) — one chain since G1 Part 2c, in that header since 2c's final run, and step 0
    below enforces it — and the program prints it at boot so it is linked in. The expected
    beacon must be in the bytes of bin/hot.package.elf and the other six must NOT be. MATCHED
    WITH THE NUL TERMINATOR (Part 0b): `…=tank` is a prefix of `…=tank-drive`, `…=tank-library`,
    `…=tank-calibrate` and `…=tank-autontest`, so a plain substring search would false-fail every
    one of those; a
    C string literal sits in the ELF with its terminating zero byte, and that is what is
    searched for. This is the end-to-end proof that the `#if` in the SOURCE took the branch the
    Makefile asked for — the one case no command-line check can see is a macro renamed or
    broken in the chain, and a "shulib Teleop" upload that was silently the tester is HA-111's
    defect class again.
    THE ROBOT BEACON (G1 Part 2c's final run, the team lead's ruling "one chain picks both"):
    one string per ROBOT — `shulib-robot=bench-bot|robot-two|invented-xdrive` — DERIVED from
    the branch's `kRobotId` by robotBeacon() in src/build_selection.hpp (never typed in a branch)
    and printed by src/main.cpp's initialize(). The build's robot's beacon (BUILD_ROBOT,
    ROBOT_BEACONS) must be in the ELF and the other robots' must NOT be. The variant beacon is
    typed in the branch, so it cannot see a branch that sets the WRONG ROBOT — a tank build
    carrying the bench bot's chassis table and no claws, with every other check green; this
    half can. (The robot's TABLES follow kRobotId through chassisTableFor() and
    mechanismTableFor(), each host-tested for every robot.)
 7. Asserts bin/hot.package.bin exists — the link is part of the gate.
 8. Asserts THE PROGRAM (added in G1 Part 2c's review). The beacon vouches for the chain's
    BRANCH — not for the enumerator that branch assigns, nor for the function each entry
    point's case calls: `kProgram = Program::BenchTests` under the tank-library beacon, or
    opcontrol()'s BenchTests and Drive calls swapped, pass steps 0-7 untouched (measured), and
    either one is an upload whose name is not its program — a "shulib Teleop" slot that is the
    tester, a "Bench Tests" slot a helper was told is read-only driving the motors. So this step
    reads the linked ELF's symbol table (`arm-none-eabi-nm -C --defined-only`) and requires,
    for the build's program (BUILD_PROGRAM), every shulib::bench:: name src/main.cpp uses for
    it (PROGRAM_ENTRY_SYMBOLS) to be LINKED, and every other program's to be ABSENT. It holds
    because a switch on a `constexpr` enum folds at -Os, a call into another TU is never
    inlined (no LTO), and --gc-sections drops every function nothing calls — measured for all
    six builds at 2c, and for the seventh at COMP-t. If any of that stops being true, this step
    FAILS (a program's own name missing, or another's present); it cannot pass silently. What
    it does NOT see: the
    sentences src/main.cpp paints itself (the DISABLED / FIELD CONTROL reasons and the tester's
    and Calibrate's AUTONOMOUS reasons are main.cpp's own literals, no program's symbol) — a
    swap there shows the wrong sentence and passes.

Once per `check`, before any build — step 0, five source checks:
  THE ONE-CHAIN RULE (G1 Part 2c; widened at 2c's final run, 2026-09-26, the team lead's ruling
  "one chain picks both"): the build switches (`SHULIB_ROBOT_*`, `SHULIB_PROGRAM_*`) are READ IN
  ONE FILE, src/build_selection.hpp (SELECTION_HEADER), and nowhere else — a switch named
  anywhere in the code of any other file under src/, include/shulib/ or test/ (ONE_CHAIN_ROOTS:
  the library headers every src/ file includes see the same -D flags, and a host test that
  defines a switch is an ODR violation across the suite) FAILS. "Any file" is every extension
  make compiles (ALL_EXTS, `.c++` and `.S` included — generated from the Makefile's mirror, never
  typed twice), every header extension, and the host build's own CMake files (test/CMakeLists.txt,
  `*.cmake`: a per-source COMPILE_DEFINITIONS is a switch the host's static_assert cannot see);
  comments and string literals stripped, so an `#if`, `#ifdef`, `#ifndef`, `#elif`, `#elifdef`,
  `#elifndef`, a `#define` that launders one, or plain code is caught, and a switch named in a
  comment or a printed hint is not. A declaration of `kProgram`, `kRobotId` or `kVariantBeacon`
  outside the chain FAILS too, in every initializer form (`= …`, `{…}`, `(…)`), and so does a
  `#define` or `#undef` of ANY selection name, in ANY file, the header and the tables included
  (SELECTION_NAMES and the per-robot names below): `#define kRobotId RobotId::BenchBot`
  after a table's include would re-point every kRobotId below it — kChassis, the tables'
  static_asserts, the mechanism table through its include — while kRobotBeacon, derived above it,
  kept robot two's beacon. The vendored trees are not read (third-party code, ignored by path as
  for warnings). There are NO allowlisted files: src/robots/chassis_table.hpp's own `#if` (2c's named
  residual) and src/robots/mechanism_table.hpp's two (T3's) are gone — the tables select from
  `kRobotId` in C++. In the header, every block that reads a switch is either an
  impossible-combination guard (nothing but `#error` inside) or THE ONE CHAIN — exactly one —
  whose every branch, `#else` included, holds exactly the three constants, each once, and
  nothing else. This is what lets step 6 cover every branch: a beacon can only vouch for the
  selection it shares a branch with, and a second chain — the shape src/main.cpp had until 2c
  (the program in one chain, the beacon in another), or the tables had until 2c's final run
  (the robot in chains of their own) — is exactly how a broken #if picks the wrong program or
  robot while the beacon still reads right.
  A ROBOT'S TABLES BY NAME (the review of 2c's final run; part of the one-chain check, reported
  under [one-chain], because naming a robot's table IS choosing a robot outside the chain):
  every robot's tables are defined in every build under their own names (so the host suite can
  read them), which lets a program name ANOTHER robot's — `tableSideOf(kRobotTwoChassis, port)`
  in the bench tester, `mechanismTableFor(RobotId::BenchBot)` in Calibrate — by a one-token
  edit that no beacon, symbol or static_assert would notice. So the per-robot names — every
  table the two table files define (`constexpr ChassisTable|MechanismTable <name>`), their
  selection functions (`constexpr const …Table& <name>(RobotId …)`) and every name those
  functions return: DERIVED from TABLE_FILES, never listed here, and a table file whose
  selection function step 0 cannot find FAILS rather than checking nothing — may be named in
  code only in the two table files, in test/ (the host suite reads them by name, and no test TU
  is in a package), and inside a `static_assert(…)`'s parentheses anywhere (compile-time only:
  T3 Part 2's checks on robot two's claws). Anywhere else under src/ or include/shulib/ one
  FAILS. A program reads the BUILD's tables, kChassis and kMechanisms, only.
  THE CHAIN'S TABLE (2c's final run): each branch's three constants must be one build's row
  of this gate's own tables — its kVariantBeacon one build's VARIANT_BEACONS entry, its
  kProgram that build's BUILD_PROGRAM, its kRobotId that build's BUILD_ROBOT — every build must
  have exactly one branch, and robotBeacon()'s cases must be ROBOT_BEACONS exactly. So a branch
  that sets the wrong program or the wrong robot is reported from the SOURCE (steps 8 and 6
  report it again from the package), and the source and this gate cannot drift apart. And the
  gate's OWN tables must agree with each other (COMP-t's review, 2026-09-29): VARIANTS — the
  builds the loop runs — must name exactly the builds BUILD_ARGS, VARIANT_DEFINES,
  VARIANT_BEACONS, BUILD_ROBOT and BUILD_PROGRAM describe, and a chain branch's beacon must
  belong to a build in VARIANTS, not merely to VARIANT_BEACONS: a build dropped from VARIANTS
  alone once PASSED "all 6 builds link" with its branch still in the chain (self-test case 33).
  THE PROGRAM TABLE (G1 Part 2c's review): step 8's tables must match the source — the
  `Program` enum's enumerators (src/build_selection.hpp) are exactly PROGRAM_ENTRY_SYMBOLS'
  programs, every build in VARIANTS has one (BUILD_PROGRAM), every shulib::bench:: name
  src/main.cpp uses (comments stripped) belongs to one program, to SHARED_BENCH_NAMES or to
  BUILD_SELECTION_NAMES, and every name in the table is one main.cpp uses. So a program cannot
  be added to the source (the Auton Tester, COMP-t, was the sixth; COMP-b's competition program
  will be the seventh) without telling this gate which functions are its own.
  THE INCLUDE ROOT (SRC1, 2026-09-27, the team lead's layout): src/ is organized by job — robots/,
  team/, programs/, common/, with main.cpp and build_selection.hpp at the top — and BOTH builds
  make src/ itself an include root: the Makefile's EXTRA_INCDIR (common.mk: -iquote"./src", AFTER
  -iquote"./include") and test/CMakeLists.txt (-iquote<repo>/src, BEFORE the host's -I shim,
  library and vendored directories). A quoted include is answered by the first place that has the
  file — beside its includer, then the roots in order — so a path two places could answer is
  decided by search ORDER, and the two builds do not share one: the host test could read one
  header and the robot build another. This check makes the order irrelevant. It FAILS: a source
  or header under src/ whose src-relative path also exists under include/, test/pros_shim/,
  test/vendor/ or test/ (INCLUDE_SHADOW_ROOTS: the other roots, and the host tests' own
  directory), or under a src/ subfolder (found beside an includer there first) — in both
  directions, so a src/shulib/… or src/pros/… that would shadow the library or the SDK fails too;
  a quoted include in a src/ file that climbs with `..` or resolves beside its includer in a
  subfolder (the old sibling form `"chassis_table.hpp"` in src/robots/: it BUILDS, and it reads
  differently in every folder); and a host test that reaches into src/ by `../src/…`. Every
  include of a src/ file is written FROM src/ (`"robots/chassis_table.hpp"`), the same in every
  folder, in main.cpp and in the tests. An old-form include in any other folder resolves nowhere
  and fails the BUILD (self-test case 29). A README (src/team/README.md) is not a source: neither
  this step nor step 1 counts it.
  THE PATHS A PROGRAM PRINTS (SRC1's review, 2026-09-27): a program's text tells its reader which
  file to edit — "fill src/robots/chassis_table.hpp, rebuild, upload", the `[BLD] robot tables:`
  line, the bench tester's census — and nothing else reads those sentences: the host tests pin
  three of them word for word, no step here reads the rest, and the staleness audit reads docs
  only. So a path that names nothing (a typo, or a file moved without its sentence — the class
  SRC1's move made live) printed to a student on the robot with every other check green. It FAILS
  now: every `src/…` path inside a string literal of a file under src/ or include/shulib/
  (PRINTED_PATH_ROOTS: the code a package is built from) must name a file or folder that exists.
  Adjacent literals are joined first, as the compiler joins them, so a path split across two is
  read whole; comments are not read (the staleness audit's class, and a comment reaches no
  robot); test/ is not read (its own tests pin what they print). Self-test case 30.

THE VARIANT DIFFERENTIAL, RETIRED AT G1 PART 2c — what it was, why it went, what replaced it
------------------------------------------------------------------------------------------
It was the brief's warning-count detector: the bench, tank, tank-drive, tank-library and
tank-calibrate builds each had to emit >= 1 -Wunused-function from src/main.cpp (the invented
X-drive wiring — `robot()`, `portMapString()` — was dead code there, and the Makefile
documented those warnings as the evidence) and the xdrive build none (the wiring live). It was
meant to catch a define passed on the command line while the #ifdef in main.cpp that reads it
was renamed or broken. G1 Part 2c moved the X-drive out of main.cpp (it is data now,
src/robots/xdrive_config.hpp, built by robot::Robot in src/programs/xdrive_program.cpp, a file every build
compiles and --gc-sections trims), so no build emits that warning any more and the
differential could only fail. It was also WEAKER than it looked: it could tell only xdrive
from the rest — tank, tank-drive, tank-library and tank-calibrate all looked like bench to
it — and a break in main.cpp's PROGRAM lattice (a separate chain from the beacon's until 2c)
was visible to it only when the break landed on the xdrive line.

What replaced it is the beacon (step 6), made to cover every branch by one chain (step 0): a
broken or renamed #if in the chain changes the program, the robot AND the beacon, so the beacon
check fails for every build, not only xdrive. Below the chain, in C++, all FIVE entry points
switch on the `Program` enum with a case per program and no default, and both tables select on
the `RobotId` enum with a case per robot and no default: a misspelt program or robot is a
compile ERROR, and one added to an enum but not handled everywhere is a -Wswitch warning, which
the policy below FAILS. And what the variant beacon cannot vouch for — which program a branch
assigns and which function a case calls (step 8, from the symbol table), which robot a branch
assigns (step 6's robot beacon) — is read from the package. The self-test plants every one
(cases 16-27; SRC1's folders and include root, cases 28-29; the paths a program prints, case 30;
the seventh build's define and beacon, cases 31-32; a build dropped from VARIANTS alone, case 33);
the G1 Part 2c and SRC1 logs
carry the hand mutations.

THE RESIDUAL G1 PART 2c NAMED, CLOSED AT ITS FINAL RUN (2026-09-26): src/robots/chassis_table.hpp
selected robot two's table over the bench bot's on its own `#if defined(SHULIB_ROBOT_TANK_2026)`,
and a break there made a tank build carry the bench bot's table with the chain — and so the
beacon — still right (measured at 2c: that build PASSED). T3's src/robots/mechanism_table.hpp added two
more such `#if`s, which failed step 0 on the merged tree. All three are gone: the tables select
from `kRobotId`, which the one chain sets; step 0 has no allowlist; and a branch that sets the
wrong robot fails step 0's chain table AND step 6's robot beacon.

THE WARNING POLICY (GATE1 §4.3)
-------------------------------
  errors                                 -> FAIL, naming the file
  warnings from a vendored tree          -> IGNORED BY PATH (include/liblvgl/,
                                            include/pros/ — third-party code we do not
                                            edit; today: 1 x
                                            -Wdeprecated-enum-enum-conversion from
                                            include/liblvgl/core/lv_obj_style.h).
                                            Never by category: killing the category
                                            would silence it in OUR code too.
  any warning, anywhere in ours          -> FAIL. No category is allowed: ALLOWED_CATEGORIES
                                            is EMPTY (the Makefile's WARNFLAGS comment).

  -Wunused-function IS A FAILURE since G1 Part 2c's final run (the team lead's ruling,
  2026-09-26: "-Wunused-function becomes a FAILURE"). It was the one allowed category from
  GATE1 until then: its written justification was the invented X-drive helpers in
  src/main.cpp, dead in five of the six builds, and G1 Part 2c removed them — measured at 2c
  and again at its final run, NO build emits a -Wunused-function. The G1 Part 2 brief had ruled
  that the category stay allowed (§2c item 4); the team lead's ruling supersedes it, and the
  2c-final log records the deviation. What it closes is the hole this header used to name: a
  genuinely NEW dead function in src/ passed the gate, because an allowed category cannot tell
  a new dead function from a documented one. Now it FAILS, naming the file and line; self-test
  case 3 plants one and requires the FAIL, and case 4 still proves the policy is not "ban every
  warning everywhere" (a vendored header's warning is ignored by path).

  Were a category ever allowed again, it would be by CATEGORY, never by file+line: line
  numbers move with every edit, and a gate that needs a re-baseline per commit gets switched
  off. "A gate's exclusion list is where its holes live" (D3): today the only exclusion is the
  vendored paths above.

MISSING TOOLCHAIN = FAIL, NEVER SKIP
------------------------------------
If arm-none-eabi-g++ (or, since step 8, arm-none-eabi-nm — the same toolchain package) is
absent the gate FAILS LOUDLY before touching make. A gate that skip-and-passes when its
compiler is missing is worse than no gate: the green tick lies.
(Likewise: `git describe` yielding nothing fails the hash assertion rather than skipping
it — every environment this gate supports, local checkout and CI checkout alike, has
git; a build_info-style hash-less binary is legal at RUNTIME, but a gate that cannot
verify identity must say so in red, not in green.)

USAGE — one command locally, the identical command in CI (arm-compile-gate job)
-------------------------------------------------------------------------------
  python3 tools/src_build_gate.py check       # the gate (also the no-arg default)
  python3 tools/src_build_gate.py self-test   # prove the detectors fire: plant each
                                              # fault, run the REAL build, confirm the
                                              # verdict, restore byte-exact
"""

import os
import re
import shutil
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ARM_GXX = "arm-none-eabi-g++"

# The builds, in the order the gate runs them, with the make arguments each one takes, the
# SET of defines each one must carry as whole compile-line tokens (every define belonging to
# another build and not to this one must be absent as a token), and the identity beacon each
# one's linked package must contain (and the others must not). The tables mirror the
# Makefile's ROBOT and PROGRAM blocks and src/build_selection.hpp's #if chain (step 0 checks the
# chain against them); a build added to one place and not the others fails here, which is the
# point of having the tables.
VARIANTS = ("bench", "xdrive", "tank", "tank-drive", "tank-library", "tank-calibrate",
            "tank-autontest")
BUILD_ARGS = {
    "bench": ("ROBOT=bench",),
    "xdrive": ("ROBOT=xdrive",),
    "tank": ("ROBOT=tank",),
    "tank-drive": ("ROBOT=tank", "PROGRAM=drive"),
    "tank-library": ("ROBOT=tank", "PROGRAM=library"),
    "tank-calibrate": ("ROBOT=tank", "PROGRAM=calibrate"),
    "tank-autontest": ("ROBOT=tank", "PROGRAM=autontest"),
}
VARIANT_DEFINES = {
    "bench": (),
    "xdrive": ("-DSHULIB_ROBOT_XDRIVE_INVENTED",),
    "tank": ("-DSHULIB_ROBOT_TANK_2026",),
    "tank-drive": ("-DSHULIB_ROBOT_TANK_2026", "-DSHULIB_PROGRAM_DRIVE"),
    "tank-library": ("-DSHULIB_ROBOT_TANK_2026", "-DSHULIB_PROGRAM_LIBRARY"),
    "tank-calibrate": ("-DSHULIB_ROBOT_TANK_2026", "-DSHULIB_PROGRAM_CALIBRATE"),
    "tank-autontest": ("-DSHULIB_ROBOT_TANK_2026", "-DSHULIB_PROGRAM_AUTONTEST"),
}
ALL_DEFINES = sorted({d for ds in VARIANT_DEFINES.values() for d in ds})
VARIANT_BEACONS = {
    "bench": b"shulib-robot-variant=bench",
    "xdrive": b"shulib-robot-variant=xdrive",
    "tank": b"shulib-robot-variant=tank",
    "tank-drive": b"shulib-robot-variant=tank-drive",
    "tank-library": b"shulib-robot-variant=tank-library",
    "tank-calibrate": b"shulib-robot-variant=tank-calibrate",
    "tank-autontest": b"shulib-robot-variant=tank-autontest",
}
# (XDRIVE_WIRING_DEAD_IN — the retired variant differential's table — was removed at G1
#  Part 2c; the header's "the variant differential, retired" says what replaced it and why.)

# Step 6's robot half (header): the ROBOT each build carries the tables of — the `RobotId`
# enumerator its branch of src/build_selection.hpp's chain sets — and each robot's beacon,
# derived from that enumerator by robotBeacon() (step 0 checks this table against its cases).
BUILD_ROBOT = {
    "bench": "BenchBot",
    "xdrive": "InventedXDrive",
    "tank": "RobotTwo",
    "tank-drive": "RobotTwo",
    "tank-library": "RobotTwo",
    "tank-calibrate": "RobotTwo",
    "tank-autontest": "RobotTwo",
}
ROBOT_BEACONS = {
    "BenchBot": b"shulib-robot=bench-bot",
    "RobotTwo": b"shulib-robot=robot-two",
    "InventedXDrive": b"shulib-robot=invented-xdrive",
}

# Step 0, THE ONE-CHAIN RULE (header). The dispatcher; the ONE file that may read a build switch;
# and the three constants every branch of its one chain sets, each exactly once (each matched on
# the whole line, comments stripped).
MAIN_CPP = os.path.join("src", "main.cpp")
SELECTION_HEADER = os.path.join("src", "build_selection.hpp")
# Source extensions, mirroring the Makefile's CEXTS / ASMEXTS / CXXEXTS (make compiles every file
# under src/ with one of these; step 1 checks it compiled exactly them, step 0 reads them all).
CXX_EXTS = (".cpp", ".c++", ".cc")
ALL_EXTS = CXX_EXTS + (".c", ".s", ".S")
# The trees step 0 reads for a switch named outside SELECTION_HEADER (header): src/ (the
# programs); include/shulib/ (the library headers every src/ file includes — they see the same -D
# flags, so a switch read there is a second selection); test/ (a host test that #defines a switch
# before including a table gives kProgram/kRobotId/kChassis two definitions across the suite — an
# ODR violation no compiler reports). NOT read: the vendored trees (include/pros/, include/liblvgl/,
# test/vendor/ — third-party code that never names our switches, ignored by path as for warnings).
# test/build_selection_test.cpp's static_assert catches a switch in the host build's GLOBAL flags
# only; one given to a single source (COMPILE_DEFINITIONS) is seen by neither it nor the compiler,
# so the host build's CMake files are read too (ONE_CHAIN_CMAKE, `#` comments stripped).
ONE_CHAIN_ROOTS = ("src", os.path.join("include", "shulib"), "test")
ONE_CHAIN_SKIPPED = (os.path.join("test", "vendor") + os.sep,)
# Every extension make compiles — GENERATED from ALL_EXTS, so a `.c++` or `.S` file is read the day
# it exists (a .c or .s/.S file never receives a switch — they ride in EXTRA_CXXFLAGS only — so a
# switch named there is a WRONG answer in every build, never a selection) — plus every header.
ONE_CHAIN_EXTS = ALL_EXTS + (".cxx", ".hpp", ".hh", ".hxx", ".h", ".inl", ".ipp", ".tpp")
ONE_CHAIN_CMAKE = ("CMakeLists.txt", ".cmake")
READS_SWITCH = re.compile(r"\bSHULIB_(ROBOT|PROGRAM)_\w*")
DIRECTIVE = re.compile(r"^\s*#\s*(\w+)")
CHAIN_CONSTANTS = (
    ("kProgram", re.compile(r"^inline constexpr Program kProgram = Program::(\w+);$")),
    ("kRobotId", re.compile(r"^inline constexpr RobotId kRobotId = RobotId::(\w+);$")),
    ("kVariantBeacon",
     re.compile(r'^inline constexpr char kVariantBeacon\[\] = "([^"\\]*)";$')),
)
# A declaration (or assignment) of one of the three, anywhere, in every initializer form:
# `kRobotId = …`, `kRobotId{…}`, `kRobotId(…)`, `kVariantBeacon[] = …` — never `kRobotId == …`
# (nothing CALLS a constant, so `kRobotId(` is a declaration; `f(kRobotId)` is not matched).
DECLARES_CHAIN_CONSTANT = re.compile(
    r"\b(kProgram|kRobotId|kVariantBeacon)\b\s*(\[\s*\])?\s*(=(?!=)|\{|\()")
# The names a MACRO may never be (header): the chain's constants, the robot beacon and its function,
# the two enums, and the build's two tables — plus the per-robot names below, derived. A `#define`
# (or `#undef`) of one re-points every use after it, and no compiler warns.
SELECTION_NAMES = frozenset({"kProgram", "kRobotId", "kVariantBeacon", "kRobotBeacon", "robotBeacon",
                             "Program", "RobotId", "kChassis", "kMechanisms"})
DEFINES_MACRO = re.compile(r"^\s*#\s*(define|undef)\s+(\w+)")

# Step 0, A ROBOT'S TABLES BY NAME (header). The two table files; the per-robot tables and the
# selection functions are DERIVED from them (an enumerated list here would miss the next robot's).
TABLE_FILES = (os.path.join("src", "robots", "chassis_table.hpp"),
               os.path.join("src", "robots", "mechanism_table.hpp"))
_TABLE_OBJECT = re.compile(
    r"\bconstexpr\s+(?:[\w:]*::)?(?:ChassisTable|MechanismTable)\s+(\w+)\s*(?:=|\{)")
_TABLE_SELECTOR = re.compile(
    r"\bconstexpr\s+const\s+(?:[\w:]*::)?(?:ChassisTable|MechanismTable)\s*&\s*(\w+)\s*"
    r"\(\s*(?:[\w:]*::)?RobotId\b[^)]*\)\s*(?:noexcept\s*)?\{")
_STATIC_ASSERT = re.compile(r"\bstatic_assert\s*\(")

# Step 8, THE PROGRAM (header), and step 0's program-table check. Which program each build IS —
# the `Program` enumerator its branch of src/build_selection.hpp's chain assigns — and, per
# program, every shulib::bench:: name src/main.cpp's entry points use for that program and for no
# other. A build must link all of its own program's names and none of another's.
BUILD_PROGRAM = {
    "bench": "BenchTests",
    "xdrive": "InventedXDrive",
    "tank": "BenchTests",
    "tank-drive": "Drive",
    "tank-library": "Teleop",
    "tank-calibrate": "Calibrate",
    "tank-autontest": "AutonTester",
}
PROGRAM_ENTRY_SYMBOLS = {
    "BenchTests": ("shulib::bench::testerInitialize", "shulib::bench::runR3a"),
    "Drive": ("shulib::bench::driveInitialize", "shulib::bench::runDrive"),
    "Teleop": ("shulib::bench::teleopInitialize", "shulib::bench::kTeleopAutonomousReason",
               "shulib::bench::teleopAutonomous", "shulib::bench::runTeleop"),
    "Calibrate": ("shulib::bench::calibrateInitialize", "shulib::bench::runCalibrate"),
    "InventedXDrive": ("shulib::bench::xdriveInitialize", "shulib::bench::xdriveAutonomous",
                       "shulib::bench::runXdrive"),
    "AutonTester": ("shulib::bench::autonTesterInitialize",),
}
# The shulib::bench:: names src/main.cpp uses that belong to NO single program: the shared
# state screens, and the fault-latch hook main.cpp defines. Anything else main.cpp names must be
# in PROGRAM_ENTRY_SYMBOLS, or step 0 reports it.
SHARED_BENCH_NAMES = {"benchStateScreen", "competitionBits", "attachFaultLatch"}
# ...and the build selection's own names, which src/main.cpp brings in with `using` (the chain's
# constants, the robot beacon and the enum the entry points switch on).
BUILD_SELECTION_NAMES = {"Program", "kProgram", "kRobotBeacon", "kVariantBeacon"}
ARM_NM = "arm-none-eabi-nm"
ENTRY_POINTS = ("initialize", "disabled", "competition_initialize", "autonomous", "opcontrol")


def _logical_lines(text):
    """Source lines with backslash continuations joined, each paired with its 1-based
    starting line number — a multi-line #if is one directive."""
    out, buf, start = [], "", None
    for n, raw in enumerate(text.splitlines(), 1):
        if start is None:
            start = n
        if raw.endswith("\\"):
            buf += raw[:-1] + " "
            continue
        out.append((start, buf + raw))
        buf, start = "", None
    if start is not None:
        out.append((start, buf))
    return out


# One left-to-right pass: a string or character literal, a // comment, a /* */ comment, or
# anything else. Order matters — `src/*.cpp` inside a // comment is not a block comment's start.
_CXX_TOKENS = re.compile(r'"(?:\\.|[^"\\\n])*"|\'(?:\\.|[^\'\\\n])*\'|//[^\n]*|/\*.*?\*/',
                         flags=re.S)


def _strip_comments(text, literals=False):
    """C++ source with its // and /* */ comments removed (line breaks kept, so line numbers
    survive). String and character literals are kept, or emptied to `""` when `literals` — a
    build switch's name inside a printed string does not READ the switch."""
    def keep(m):
        t = m.group(0)
        if t.startswith("//"):
            return ""
        if t.startswith("/*"):
            return "\n" * t.count("\n")
        if literals:
            return t[0] * 2
        return t
    return _CXX_TOKENS.sub(keep, text)


def _read(repo, rel):
    with open(os.path.join(repo, rel), encoding="utf-8", errors="replace") as fh:
        return fh.read()


def _blank_static_asserts(code):
    """`code` (comments and literals already stripped) with the parenthesised argument of every
    `static_assert(…)` blanked to spaces, line breaks kept: what may name a per-robot table
    (header, "A ROBOT'S TABLES BY NAME") — compile-time only, nothing it names reaches a package.
    Exactly the parentheses: code after the closing one on the same line is still read."""
    out, i = [], 0
    for m in _STATIC_ASSERT.finditer(code):
        if m.start() < i:
            continue  # inside the previous static_assert's argument
        depth, j = 1, m.end()
        while j < len(code) and depth:
            depth += (code[j] == "(") - (code[j] == ")")
            j += 1
        out.append(code[i:m.end()])
        out.append(re.sub(r"[^\n]", " ", code[m.end():j - 1]) + code[j - 1:j])
        i = j
    out.append(code[i:])
    return "".join(out)


def _strip_cmake_comments(text):
    """CMake source with its comments removed, line breaks kept: `#[[ … ]]` bracket comments, and
    a `#` to the end of its line when it is outside a double-quoted argument."""
    text = re.sub(r"#\[(=*)\[.*?\]\1\]", lambda m: "\n" * m.group(0).count("\n"), text, flags=re.S)
    lines = []
    for line in text.split("\n"):
        quoted, cut = False, len(line)
        for k, c in enumerate(line):
            if c == '"' and (k == 0 or line[k - 1] != "\\"):
                quoted = not quoted
            elif c == "#" and not quoted:
                cut = k
                break
        lines.append(line[:cut])
    return "\n".join(lines)


def per_robot_table_names(repo=None):
    """Step 0 (header, "A ROBOT'S TABLES BY NAME"): (names, problems). `names` = every table the two
    table files define, their selection functions, and every name those functions return —
    DERIVED from the files (comments and literals stripped). `problems` names a table file whose
    selection function, or whose tables, step 0 cannot find: fail-closed, because a rule over an
    empty set would check nothing and say so in green."""
    repo = repo or REPO
    names, problems = set(), []
    for rel in TABLE_FILES:
        if not os.path.exists(os.path.join(repo, rel)):
            problems.append(f"{rel} is missing — step 0 cannot derive its per-robot names")
            continue
        code = _strip_comments(_read(repo, rel), literals=True)
        objects = set(_TABLE_OBJECT.findall(code))
        selectors = list(_TABLE_SELECTOR.finditer(code))
        returned = set()
        for m in selectors:
            depth, j = 1, m.end()
            while j < len(code) and depth:
                depth += (code[j] == "{") - (code[j] == "}")
                j += 1
            returned |= set(re.findall(r"\breturn\s+(\w+)\s*;", code[m.end():j]))
        if len(selectors) != 1 or not objects or not returned:
            problems.append(
                f"{rel}: step 0 found {len(selectors)} selection function(s) "
                f"(`constexpr const ChassisTable|MechanismTable& <name>(RobotId …) {{`), "
                f"{len(objects)} table(s) and {len(returned)} returned name(s) — it needs exactly "
                f"one function, and tables it returns, to know which names are a robot's; the rule "
                f"would check nothing (header, step 0)")
        names |= objects | returned | {m.group(1) for m in selectors}
    return names, problems


def _parse_selection_header(text):
    """Step 0's reading of src/build_selection.hpp (comments stripped). Returns (chains,
    problems): every block that reads a build switch and holds anything but `#error` is a CHAIN
    — a list of branches, each [line number, directive, body], the body its (line number, line)
    pairs — and `problems` names every switch read and every chain constant declared OUTSIDE a
    guard or a chain."""
    lines = [(n, l.strip()) for n, l in _logical_lines(_strip_comments(text)) if l.strip()]
    chains, problems = [], []
    i = 0
    while i < len(lines):
        n, line = lines[i]
        d = DIRECTIVE.match(line)
        kind = d.group(1) if d else None
        if kind in ("if", "ifdef", "ifndef") and READS_SWITCH.search(line):
            branches, depth, j = [[n, line, []]], 0, i + 1
            while j < len(lines):
                m, body_line = lines[j]
                d2 = DIRECTIVE.match(body_line)
                k2 = d2.group(1) if d2 else None
                if k2 in ("if", "ifdef", "ifndef"):
                    depth += 1
                elif k2 == "endif" and depth == 0:
                    break
                elif k2 == "endif":
                    depth -= 1
                elif k2 in ("elif", "elifdef", "elifndef", "else") and depth == 0:
                    branches.append([m, body_line, []])
                    j += 1
                    continue
                branches[-1][2].append((m, body_line))
                j += 1
            if j == len(lines):
                problems.append(f"{SELECTION_HEADER}:{n} `{line}` has no #endif")
            guard = all(body and all((DIRECTIVE.match(b) or [None, None])[1] == "error"
                                     for _m, b in body)
                        for _n, _l, body in branches)
            if not guard:
                chains.append(branches)
            i = j + 1
            continue
        if READS_SWITCH.search(line):
            problems.append(
                f"{SELECTION_HEADER}:{n} reads a build switch outside an #error guard and outside "
                f"THE ONE CHAIN (header, step 0): `{line}`")
        elif DECLARES_CHAIN_CONSTANT.search(line):
            problems.append(
                f"{SELECTION_HEADER}:{n} declares one of the chain's constants outside THE ONE "
                f"CHAIN — every branch sets kProgram, kRobotId and kVariantBeacon, and nothing else "
                f"may (header, step 0): `{line}`")
        i += 1
    return chains, problems


def _branch_constants(body):
    """{constant name: [values]} for one branch's body, and the lines that are none of them."""
    found, extra = {}, []
    for m, b in body:
        for name, rx in CHAIN_CONSTANTS:
            hit = rx.match(b)
            if hit:
                found.setdefault(name, []).append(hit.group(1))
                break
        else:
            extra.append((m, b))
    return found, extra


def one_chain_problems(repo=None):
    """Step 0 (header): the build switches are read in src/build_selection.hpp and nowhere
    else — not in any file make compiles, any header or the host build's CMake under src/,
    include/shulib/ or test/ (ONE_CHAIN_ROOTS); no selection name is ever a macro; a per-robot
    table is named only in the tables, in test/ and inside a static_assert; in that header, every
    block reading a switch is an #error guard or THE ONE CHAIN, whose every branch sets kProgram,
    kRobotId and kVariantBeacon exactly once and nothing else. Returns a list of problem
    strings."""
    repo = repo or REPO
    per_robot, problems = per_robot_table_names(repo)
    macro_names = SELECTION_NAMES | per_robot
    for top in ONE_CHAIN_ROOTS:
        for root, dirs, files in os.walk(os.path.join(repo, top)):
            dirs.sort()
            for f in sorted(files):
                rel = os.path.relpath(os.path.join(root, f), repo)
                if rel.startswith(ONE_CHAIN_SKIPPED):
                    continue
                if f == ONE_CHAIN_CMAKE[0] or f.endswith(ONE_CHAIN_CMAKE[1:]):
                    problems += _cmake_selection_problems(repo, rel)
                    continue
                if not f.endswith(ONE_CHAIN_EXTS):
                    continue
                problems += _macro_problems(repo, rel, macro_names)
                if rel != SELECTION_HEADER:
                    problems += _stray_selection_problems(repo, rel, per_robot)
    problems += _selection_header_problems(repo)
    return problems


def _stray_selection_problems(repo, rel, per_robot=frozenset()):
    """One C++ file outside SELECTION_HEADER (header, step 0): a build switch named in its code
    (comments and string literals stripped), one of the chain's constants declared there, or —
    outside the two table files and test/ — a per-robot table named outside a static_assert."""
    problems = []
    code = _strip_comments(_read(repo, rel), literals=True)
    for n, line in _logical_lines(code):
        if READS_SWITCH.search(line):
            problems.append(
                f"{rel}:{n} names a build switch — only {SELECTION_HEADER} may read them "
                f"(header, step 0): `{line.strip()}`")
        elif DECLARES_CHAIN_CONSTANT.search(line):
            problems.append(
                f"{rel}:{n} declares one of THE ONE CHAIN's constants — only "
                f"{SELECTION_HEADER}'s chain may (header, step 0): `{line.strip()}`")
    if per_robot and rel not in TABLE_FILES and not rel.startswith("test" + os.sep):
        named = re.compile(r"\b(" + "|".join(sorted(per_robot)) + r")\b")
        for n, line in _logical_lines(_blank_static_asserts(code)):
            for name in sorted(set(named.findall(line))):
                problems.append(
                    f"{rel}:{n} names {name}, a per-robot table — a program reads the BUILD's "
                    f"tables (kChassis, kMechanisms), chosen by {SELECTION_HEADER}'s one chain; "
                    f"naming a robot's table is choosing a robot outside it (header, step 0, a "
                    f"robot's tables by name; only the tables, test/ and a static_assert may): "
                    f"`{line.strip()}`")
    return problems


def _macro_problems(repo, rel, names):
    """Any C++ file under ONE_CHAIN_ROOTS, SELECTION_HEADER and the tables included (header, step
    0): a `#define` or `#undef` of a selection name — it would re-point every use below it."""
    problems = []
    for n, line in _logical_lines(_strip_comments(_read(repo, rel), literals=True)):
        m = DEFINES_MACRO.match(line)
        if m and m.group(2) in names:
            problems.append(
                f"{rel}:{n} makes {m.group(2)} a macro (`#{m.group(1)}`) — a selection name is "
                f"never a macro: it would re-point every {m.group(2)} after it, and no compiler "
                f"warns (header, step 0): `{line.strip()}`")
    return problems


def _cmake_selection_problems(repo, rel):
    """One of the host build's CMake files (header, step 0): a build switch named in it (comments
    stripped) — a per-source COMPILE_DEFINITIONS gives one test TU another robot's selection, an
    ODR violation across the suite that test/build_selection_test.cpp's static_assert cannot see."""
    problems = []
    for n, line in enumerate(_strip_cmake_comments(_read(repo, rel)).split("\n"), 1):
        if READS_SWITCH.search(line):
            problems.append(
                f"{rel}:{n} names a build switch in the host build — only {SELECTION_HEADER} may "
                f"read them (header, step 0): `{line.strip()}`")
    return problems


def _selection_header_problems(repo):
    """SELECTION_HEADER itself (header, step 0): every block reading a switch is an #error guard
    or THE ONE CHAIN — exactly one — whose every branch, #else included, sets the three constants
    exactly once and nothing else."""
    problems = []
    if not os.path.exists(os.path.join(repo, SELECTION_HEADER)):
        return [f"{SELECTION_HEADER} is missing — no build can say what it is"]
    chains, header_problems = _parse_selection_header(_read(repo, SELECTION_HEADER))
    problems += header_problems
    if not chains:
        problems.append(f"{SELECTION_HEADER} has no chain on the build switches (header, step 0)")
    for c, branches in enumerate(chains):
        if c > 0:  # reported once, as what it is; its branches' shapes are beside the point
            problems.append(
                f"{SELECTION_HEADER}:{branches[0][0]} opens a SECOND chain on the build switches, "
                f"outside THE ONE CHAIN — the beacon cannot vouch for a selection it does not "
                f"share a branch with (header, step 0): `{branches[0][1]}`")
            continue
        if not any(DIRECTIVE.match(d).group(1) == "else" for _n, d, _b in branches):
            problems.append(
                f"{SELECTION_HEADER}:{branches[0][0]} — the chain has no #else: a build that "
                f"matches no branch would have no program and no robot (header, step 0)")
        for n, directive, body in branches:
            found, extra = _branch_constants(body)
            for name, _rx in CHAIN_CONSTANTS:
                k = len(found.get(name, []))
                if k != 1:
                    problems.append(
                        f"{SELECTION_HEADER}:{n} — the branch `{directive}` sets {name} {k} "
                        f"times; every branch of THE ONE CHAIN sets kProgram, kRobotId and "
                        f"kVariantBeacon exactly once (header, step 0)")
            for m, b in extra:
                problems.append(
                    f"{SELECTION_HEADER}:{m} is inside a branch of THE ONE CHAIN but is not one "
                    f"of its three constants (header, step 0): `{b}`")
    return problems


def build_table_problems():
    """The gate's own per-build tables against each other (header, "THE CHAIN'S TABLE"): VARIANTS
    names each build once, and exactly the builds every per-build table describes. Returns a list
    of problem strings."""
    problems = []
    if len(set(VARIANTS)) != len(VARIANTS):
        problems.append(f"VARIANTS names a build twice: {list(VARIANTS)} (header, step 0)")
    for name, table in (("BUILD_ARGS", BUILD_ARGS), ("VARIANT_DEFINES", VARIANT_DEFINES),
                        ("VARIANT_BEACONS", VARIANT_BEACONS), ("BUILD_ROBOT", BUILD_ROBOT),
                        ("BUILD_PROGRAM", BUILD_PROGRAM)):
        if set(table) != set(VARIANTS):
            problems.append(
                f"this gate's {name} describes builds {sorted(table)}, but VARIANTS — the builds "
                f"the gate runs — is {sorted(VARIANTS)}: a build in one and not the other is a build "
                f"the gate never checks (header, step 0)")
    return problems


def chain_table_problems(repo=None):
    """Step 0's second check (header, "THE CHAIN'S TABLE"): the gate's own tables agree
    (build_table_problems), each branch of the chain is one build's row of them, and
    robotBeacon()'s cases are ROBOT_BEACONS. Returns a list of problem strings."""
    repo = repo or REPO
    tables = build_table_problems()
    if not os.path.exists(os.path.join(repo, SELECTION_HEADER)):
        return tables + [f"{SELECTION_HEADER} is missing"]
    text = _read(repo, SELECTION_HEADER)
    chains, _problems = _parse_selection_header(text)
    if not chains:
        return tables + [f"{SELECTION_HEADER} has no chain to check against this gate's tables"]
    # (line, program, robot, beacon) per branch that sets its beacon exactly once; a program or a
    # robot set other than once is None here — the one-chain rule reports that shape, so this
    # check compares only what the branch does set, and does not cascade.
    rows = []
    for n, _directive, body in chains[0]:
        found, _extra = _branch_constants(body)
        one = {name: vals[0] for name, vals in found.items() if len(vals) == 1}
        if "kVariantBeacon" in one:
            rows.append((n, one.get("kProgram"), one.get("kRobotId"), one["kVariantBeacon"]))
    problems = list(tables)
    for v in VARIANTS:
        if v not in VARIANT_BEACONS:
            continue  # reported by build_table_problems
        want = VARIANT_BEACONS[v].decode()
        hits = [r for r in rows if r[3] == want]
        if len(hits) != 1:
            problems.append(f"build {v}: {len(hits)} branches of THE ONE CHAIN carry its beacon "
                            f"{want!r} — exactly one must (header, step 0)")
            continue
        n, program, robot_id, _beacon = hits[0]
        if program is not None and program != BUILD_PROGRAM.get(v):
            problems.append(
                f"build {v}'s branch ({SELECTION_HEADER}:{n}) sets kProgram = Program::{program}, "
                f"but this gate's BUILD_PROGRAM says {BUILD_PROGRAM.get(v)} — the build would run "
                f"another program under its own beacon (header, step 0)")
        if robot_id is not None and robot_id != BUILD_ROBOT.get(v):
            problems.append(
                f"build {v}'s branch ({SELECTION_HEADER}:{n}) sets kRobotId = RobotId::{robot_id}, "
                f"but this gate's BUILD_ROBOT says {BUILD_ROBOT.get(v)} — the build would carry "
                f"another robot's tables under its own beacon (header, step 0)")
    # A branch's beacon must belong to a build the gate RUNS (VARIANTS), not merely to one its
    # beacon table names — else a build dropped from VARIANTS alone leaves its branch unchecked.
    expected = {VARIANT_BEACONS[v].decode() for v in VARIANTS if v in VARIANT_BEACONS}
    for n, _p, _r, beacon in rows:
        if beacon not in expected:
            problems.append(f"{SELECTION_HEADER}:{n} — a branch's beacon {beacon!r} is no build's "
                            f"in this gate's VARIANTS (header, step 0)")
    code = _strip_comments(text)
    enum = re.search(r"enum\s+class\s+RobotId\s*\{(.*?)\}", code, flags=re.S)
    robots = ({e.split("=")[0].strip() for e in enum.group(1).split(",") if e.strip()}
              if enum else set())
    if robots != set(ROBOT_BEACONS):
        problems.append(f"{SELECTION_HEADER}'s RobotId enumerators {sorted(robots)} are not this "
                        f"gate's ROBOT_BEACONS robots {sorted(ROBOT_BEACONS)} (header, step 0)")
    body = re.search(r"robotBeacon\(RobotId \w+\)\s*\{(.*?)\n\}", code, flags=re.S)
    cases = dict(re.findall(r'case RobotId::(\w+):\s*return "([^"]*)";', body.group(1))
                 if body else [])
    if cases != {r: b.decode() for r, b in ROBOT_BEACONS.items()}:
        problems.append(f"{SELECTION_HEADER}'s robotBeacon() returns {cases}, but this gate's "
                        f"ROBOT_BEACONS says "
                        f"{ {r: b.decode() for r, b in ROBOT_BEACONS.items()} } (header, step 0)")
    return problems


def program_table_problems(repo=None):
    """Step 0's second check (header, "THE PROGRAM TABLE"): step 8's tables against the
    source. Returns a list of problem strings."""
    repo = repo or REPO
    problems = []
    code = _strip_comments(_read(repo, MAIN_CPP))
    selection = (_strip_comments(_read(repo, SELECTION_HEADER))
                 if os.path.exists(os.path.join(repo, SELECTION_HEADER)) else "")
    enum = re.search(r"enum\s+class\s+Program\s*\{(.*?)\}", selection, flags=re.S)
    enumerators = ({e.split("=")[0].strip() for e in enum.group(1).split(",") if e.strip()}
                   if enum else set())
    if not enum:
        problems.append(f"{SELECTION_HEADER} has no `enum class Program` — step 8 has nothing to "
                        f"check its tables against")
    for e in sorted(enumerators - set(PROGRAM_ENTRY_SYMBOLS)):
        problems.append(f"{SELECTION_HEADER}'s Program::{e} is not in this gate's "
                        f"PROGRAM_ENTRY_SYMBOLS — name the functions that are its own (header, "
                        f"step 0)")
    for p in sorted(set(PROGRAM_ENTRY_SYMBOLS) - enumerators) if enum else []:
        problems.append(f"this gate's PROGRAM_ENTRY_SYMBOLS names program {p}, which "
                        f"{SELECTION_HEADER}'s Program enum does not have — the table is stale")
    for v in VARIANTS:
        if BUILD_PROGRAM.get(v) not in PROGRAM_ENTRY_SYMBOLS:
            problems.append(f"build {v} has no program in BUILD_PROGRAM — step 8 cannot say "
                            f"what it must link")
    used = set(re.findall(r"shulib::bench::(\w+)", code))
    owned = {sym.rsplit("::", 1)[1] for syms in PROGRAM_ENTRY_SYMBOLS.values() for sym in syms}
    for name in sorted(used - owned - SHARED_BENCH_NAMES - BUILD_SELECTION_NAMES):
        problems.append(f"{MAIN_CPP} uses shulib::bench::{name}, which belongs to no program in "
                        f"this gate's PROGRAM_ENTRY_SYMBOLS and is not in SHARED_BENCH_NAMES or "
                        f"BUILD_SELECTION_NAMES — say whose it is, or step 8 cannot check it "
                        f"(header, step 0)")
    for name in sorted(owned - used):
        problems.append(f"this gate's PROGRAM_ENTRY_SYMBOLS names shulib::bench::{name}, which "
                        f"{MAIN_CPP} never uses — the table is stale")
    return problems


# Step 0, THE INCLUDE ROOT (SRC1, 2026-09-27; header). src/ is itself an include root in both builds;
# these are the OTHER places a quoted include of a src/ path could be answered, in either build: the
# library and SDK root (the robot build searches -iquote ./include BEFORE ./src), the host build's
# PROS shim and vendored headers (-I, searched after the host's -iquote src — so here a src/ path
# would shadow THEM), and test/ itself (a host test's quoted include looks beside the test first).
INCLUDE_ROOT = "src"
INCLUDE_SHADOW_ROOTS = ("include", os.path.join("test", "pros_shim"), os.path.join("test", "vendor"),
                        "test")
INCLUDE_SKIPPED_TEST_DIRS = (os.path.join("test", "pros_shim"), os.path.join("test", "vendor"))
QUOTED_INCLUDE = re.compile(r'^\s*#\s*include\s*"([^"]+)"')


def _src_tree(repo):
    """(files, dirs) under INCLUDE_ROOT, both src-relative with `/` separators: every file with an
    extension a compiler reads (ONE_CHAIN_EXTS — a README is never an include target), and every
    directory below src/ itself."""
    src = os.path.join(repo, INCLUDE_ROOT)
    files, dirs = [], []
    for root, dnames, fnames in os.walk(src):
        dnames.sort()
        here = os.path.relpath(root, src).replace(os.sep, "/")
        if here != ".":
            dirs.append(here)
        for f in sorted(fnames):
            if f.endswith(ONE_CHAIN_EXTS):
                files.append(f if here == "." else f"{here}/{f}")
    return files, dirs


def include_root_problems(repo=None):
    """Step 0 (header, "THE INCLUDE ROOT"): no src/ path is one a second place could answer, and every
    include of a src/ file is written from src/. Returns a list of problem strings."""
    repo = repo or REPO
    problems = []
    files, dirs = _src_tree(repo)
    src_rel = lambda r: f"{INCLUDE_ROOT}/{r}"  # noqa: E731 — a path as the problems print it
    # (a) a SHADOW: the same src-relative path answered by another root, or beside an includer in a
    #     src/ subfolder (a quoted include looks in its includer's own directory FIRST).
    for r in files:
        for other in INCLUDE_SHADOW_ROOTS:
            if os.path.isfile(os.path.join(repo, other, r)):
                problems.append(
                    f"{other}/{r} and {src_rel(r)} are the same path under two include roots — "
                    f"`#include \"{r}\"` would be answered by whichever root a build searches "
                    f"first, and the robot and host builds do not search them in the same order "
                    f"(header, step 0, the include root)")
        for d in dirs:
            if os.path.isfile(os.path.join(repo, INCLUDE_ROOT, d, r)):
                problems.append(
                    f"{src_rel(d + '/' + r)} shadows {src_rel(r)} for every file in "
                    f"{src_rel(d)}/: a quoted include looks beside its includer first, so "
                    f"`#include \"{r}\"` there would not reach {src_rel(r)} (header, step 0, the "
                    f"include root)")
    # (b) a src/ include NOT written from src/: one that climbs, or one a subfolder's file resolves
    #     beside itself — the old sibling form, which builds and reads differently in every folder.
    for r in files:
        here = r.rsplit("/", 1)[0] if "/" in r else ""
        code = _strip_comments(_read(repo, os.path.join(INCLUDE_ROOT, r)))
        for n, line in _logical_lines(code):
            m = QUOTED_INCLUDE.match(line)
            if not m:
                continue
            x = m.group(1)
            if ".." in x.split("/") or x.startswith("./"):
                problems.append(
                    f"{src_rel(r)}:{n} includes \"{x}\" by a relative path — every include of a "
                    f"src/ file is written from src/ (`#include \"robots/chassis_table.hpp\"`) "
                    f"(header, step 0, the include root)")
            elif here and os.path.isfile(os.path.join(repo, INCLUDE_ROOT, here, x)):
                problems.append(
                    f"{src_rel(r)}:{n} includes \"{x}\", which resolves beside it in "
                    f"{src_rel(here)}/ — write it from src/, `#include \"{here}/{x}\"`, the same in "
                    f"every folder (header, step 0, the include root)")
    # (c) a host test that reaches into src/ by a relative path instead of through the root.
    for root, dnames, fnames in os.walk(os.path.join(repo, "test")):
        dnames.sort()
        rel_root = os.path.relpath(root, repo)
        if rel_root.startswith(INCLUDE_SKIPPED_TEST_DIRS):
            dnames[:] = []
            continue
        for f in sorted(fnames):
            if not f.endswith(ONE_CHAIN_EXTS):
                continue
            rel = os.path.join(rel_root, f)
            for n, line in _logical_lines(_strip_comments(_read(repo, rel))):
                m = QUOTED_INCLUDE.match(line)
                if not m or ".." not in m.group(1).split("/"):
                    continue
                target = os.path.relpath(os.path.normpath(os.path.join(root, m.group(1))), repo)
                if target == INCLUDE_ROOT or target.startswith(INCLUDE_ROOT + os.sep):
                    problems.append(
                        f"{rel}:{n} includes \"{m.group(1)}\" — a host test includes a src/ file "
                        f"written from src/, `#include \"{os.path.relpath(target, INCLUDE_ROOT)}\"`"
                        f", through test/CMakeLists.txt's src/ root (header, step 0, the include "
                        f"root)")
    return problems


# Step 0, THE PATHS A PROGRAM PRINTS (SRC1's review, 2026-09-27; header). The code a package is built
# from — src/ and the library headers every src/ file includes — and a path into src/ inside one of
# its string literals. The look-behind keeps `include/src/…`-style tails and `../src/` out; the path
# stops at the first character a file name here does not use, and a sentence's full stop after it is
# not part of it.
PRINTED_PATH_ROOTS = (INCLUDE_ROOT, os.path.join("include", "shulib"))
_PRINTED_PATH = re.compile(r"(?<![\w/.-])src/[A-Za-z0-9_./-]*")
_LITERAL_GAP = re.compile(r"\s*(?:u8|u|U|L)?")  # what may stand between two literals the compiler joins


def _literal_runs(code):
    """(pieces, text) for every run of ADJACENT string literals in `code` (comments already
    stripped): `"fill src/robots/" "chassis_table.hpp"` is one string to the compiler, so it is one
    string here and a path split across the two is read whole. `text` is the literals' contents
    joined, quotes and any encoding prefix dropped; `pieces` is one (offset in `text`, line) per
    literal, so a match in `text` is reported on the line its literal starts. A character literal,
    or any code between two literals, ends a run."""
    runs, text, pieces, end = [], None, None, None
    line, counted = 1, 0
    for m in _CXX_TOKENS.finditer(code):
        t = m.group(0)
        if not t.startswith('"'):
            if text is not None:
                runs.append((pieces, text))
            text = None
            continue
        line += code.count("\n", counted, m.start())
        counted = m.start()
        if text is not None and _LITERAL_GAP.fullmatch(code[end:m.start()]):
            pieces.append((len(text), line))
            text += t[1:-1]
        else:
            if text is not None:
                runs.append((pieces, text))
            text, pieces = t[1:-1], [(0, line)]
        end = m.end()
    if text is not None:
        runs.append((pieces, text))
    return runs


def printed_path_problems(repo=None):
    """Step 0 (header, "THE PATHS A PROGRAM PRINTS"): every `src/…` path inside a string literal of a
    file under PRINTED_PATH_ROOTS names a file or folder that exists. Returns a list of problem
    strings, each on the line of the literal the path starts in."""
    repo = repo or REPO
    problems = []
    for base in PRINTED_PATH_ROOTS:
        for root, dnames, fnames in os.walk(os.path.join(repo, base)):
            dnames.sort()
            for f in sorted(fnames):
                if not f.endswith(ONE_CHAIN_EXTS):
                    continue
                rel = os.path.relpath(os.path.join(root, f), repo)
                for pieces, text in _literal_runs(_strip_comments(_read(repo, rel))):
                    for m in _PRINTED_PATH.finditer(text):
                        path = m.group(0).rstrip(".")
                        if os.path.exists(os.path.join(repo, *path.split("/"))):
                            continue
                        n = [ln for at, ln in pieces if at <= m.start()][-1]
                        problems.append(
                            f"{rel}:{n} prints \"{path}\", which does not exist — text that "
                            f"names a file sends its reader there; after a move it must name "
                            f"the file's new path (header, step 0, the paths a program prints)")
    return problems


_NM_FUNCTION = re.compile(r"(shulib::bench::\w+)\([^()]*\)")
_NM_OBJECT = re.compile(r"shulib::bench::\w+")


def find_nm(path_env=None):
    """Resolve the toolchain's nm (step 8), as find_compiler() resolves g++."""
    return shutil.which(ARM_NM, path=path_env)


def elf_bench_names(elf, nm):
    """The shulib::bench:: names DEFINED in a linked ELF (step 8): each function by its
    qualified name without the parameter list, each object by its qualified name. A local
    static, a lambda or a template instantiation named after a function is NOT that function."""
    p = subprocess.run([nm, "-C", "--defined-only", elf], capture_output=True, text=True,
                       timeout=120)
    if p.returncode != 0:
        raise RuntimeError(f"{ARM_NM} exited {p.returncode}: {p.stderr.strip()}")
    names = set()
    for line in p.stdout.splitlines():
        parts = line.split(" ", 2)
        if len(parts) < 3:
            continue
        name = parts[2].strip()
        m = _NM_FUNCTION.fullmatch(name)
        if m:
            names.add(m.group(1))
        elif _NM_OBJECT.fullmatch(name):
            names.add(name)
    return names


def program_problems(robot, linked):
    """Step 8 for one build, given the shulib::bench:: names its ELF links."""
    prog = BUILD_PROGRAM.get(robot)
    if prog not in PROGRAM_ENTRY_SYMBOLS:
        return [f"build {robot} has no program in BUILD_PROGRAM (step 8)"]
    problems = []
    for sym in PROGRAM_ENTRY_SYMBOLS[prog]:
        if sym not in linked:
            problems.append(
                f"build {robot} is the {prog} program, but bin/hot.package.elf does not link its "
                f"{sym} — an entry point in src/main.cpp calls another program's function where "
                f"{prog}'s belongs, or the chain's branch assigns another program (step 8)")
    for other, syms in PROGRAM_ENTRY_SYMBOLS.items():
        if other == prog:
            continue
        for sym in syms:
            if sym in linked:
                problems.append(
                    f"build {robot} is the {prog} program, but bin/hot.package.elf links the "
                    f"{other} program's {sym} — this build runs {other} code (step 8)")
    return problems


def _beacon_bytes(beacon):
    """A beacon as it sits in the ELF: the literal PLUS its NUL terminator, so that
    `…=tank` cannot match inside `…=tank-drive` (header, step 6)."""
    return beacon + b"\0"

# Vendored third-party trees: warnings here are ignored BY PATH (never by category).
VENDOR_PREFIXES = ("include/liblvgl/", "include/pros/")

# The allowed categories: NONE (the Makefile's WARNFLAGS comment). -Wunused-function was the
# one allowed from GATE1 until G1 Part 2c's final run, when the team lead ruled it a FAILURE
# (the header's "-Wunused-function IS A FAILURE"). Every warning arriving in our code fails;
# self-test case 3 plants a dead function and requires the FAIL.
ALLOWED_CATEGORIES = frozenset()

# (CXX_EXTS / ALL_EXTS, the Makefile's source extensions, are defined with step 0's tables above:
#  step 0 reads every file make compiles.)

ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
DIAG = re.compile(
    r"^(?P<path>[^:\s][^:]*?):(?P<line>\d+):(?:\d+:)?\s*"
    r"(?P<kind>fatal error|error|warning):\s*(?P<msg>.*)$")
CATEGORY = re.compile(r"\[(-W[A-Za-z0-9=+._-]+)\]\s*$")
COMPILED = re.compile(r"^Compiled (\S+) \[(OK|WARNINGS|ERRORS)\]")


def _norm(path):
    while path.startswith("./"):
        path = path[2:]
    return path


def git_describe():
    """The exact incantation the Makefile uses for SHULIB_GIT_HASH."""
    try:
        r = subprocess.run(["git", "describe", "--always", "--dirty", "--abbrev=7"],
                           cwd=REPO, capture_output=True, text=True, timeout=30)
        return r.stdout.strip() if r.returncode == 0 else ""
    except Exception:
        return ""


def expected_tus():
    """Every TU make will compile, globbed from src/ — generated, never enumerated."""
    out = []
    for root, _dirs, files in os.walk(os.path.join(REPO, "src")):
        for f in files:
            if f.endswith(ALL_EXTS):
                out.append(os.path.relpath(os.path.join(root, f), REPO))
    return sorted(out)


def find_compiler(path_env=None):
    """Resolve the cross-compiler. path_env overrides $PATH so the self-test can prove
    the missing-toolchain verdict without uninstalling a compiler."""
    return shutil.which(ARM_GXX, path=path_env)


class BuildResult:
    def __init__(self):
        self.exit = None
        self.text = ""            # ANSI-stripped combined output
        self.errors = []          # (path, line, msg)
        self.vendor = []          # (path, line, category)
        self.allowed = []         # (path, line, category)
        self.disallowed = []      # (path, line, category, msg)
        self.compiled = []        # TU paths make actually compiled this run
        self.cmdline = {}         # TU path -> full compiler command line (V=1)
        self.hash_expected = ""   # git describe at build time


def run_build(robot, path_env=None):
    """One structurally fresh `make <BUILD_ARGS[robot]> V=1`, parsed."""
    for d in ("bin", ".d"):  # what `make clean` removes; both gitignored (landmine 3/8)
        shutil.rmtree(os.path.join(REPO, d), ignore_errors=True)

    b = BuildResult()
    b.hash_expected = git_describe()  # same moment as make's parse-time $(shell ...)

    env = dict(os.environ)
    if path_env is not None:
        env["PATH"] = path_env
    p = subprocess.run(["make", *BUILD_ARGS[robot], "V=1"],
                       cwd=REPO, capture_output=True, text=True, env=env, timeout=600)
    b.exit = p.returncode
    b.text = ANSI.sub("", p.stdout + p.stderr)

    tus = expected_tus()
    for line in b.text.splitlines():
        m = COMPILED.match(line)
        if m:
            b.compiled.append(_norm(m.group(1)))
            continue
        for tu in tus:
            if " -c " in line and line.rstrip().endswith(" " + tu):
                b.cmdline[tu] = line
        m = DIAG.match(line)
        if not m:
            continue
        path, lno, kind, msg = (_norm(m.group("path")), m.group("line"),
                                m.group("kind"), m.group("msg"))
        if kind in ("error", "fatal error"):
            b.errors.append((path, lno, msg))
            continue
        cm = CATEGORY.search(msg)
        cat = cm.group(1) if cm else "(uncategorized)"
        if path.startswith(VENDOR_PREFIXES):
            b.vendor.append((path, lno, cat))
        elif cat in ALLOWED_CATEGORIES:
            b.allowed.append((path, lno, cat))
        else:
            # fail-closed: an uncategorized warning in our code is disallowed too
            b.disallowed.append((path, lno, cat, msg))
    return b


def verdict(robot, b):
    """Apply the policy to one build. Returns (ok, [problem strings])."""
    problems = []

    if b.errors:
        for path, lno, msg in b.errors:
            problems.append(f"ERROR in {path}:{lno} — {msg}")
    if b.exit != 0 and not b.errors:
        tail = "\n      ".join(b.text.splitlines()[-15:])
        problems.append(f"make exited {b.exit} with no parsed compiler error — "
                        f"output tail:\n      {tail}")
    if b.exit != 0 or b.errors:
        return False, problems  # nothing downstream is meaningful on a failed build

    # Freshness, observable: every TU in src/ must have been compiled THIS run.
    exp = expected_tus()
    if sorted(b.compiled) != exp:
        problems.append(
            f"compiled {len(b.compiled)}/{len(exp)} TUs ({sorted(b.compiled)} vs "
            f"expected {exp}) — an incremental no-op or a missed source file")

    for path, lno, cat, msg in b.disallowed:
        problems.append(f"disallowed warning {cat} at {path}:{lno} — {msg}")

    hot_bin = os.path.join(REPO, "bin", "hot.package.bin")
    if not os.path.exists(hot_bin):
        problems.append("bin/hot.package.bin missing — the link is part of this gate")

    # Structural §4.1 assertions, read off the real V=1 compile command lines. TOKEN-EXACT:
    # a look-alike define (GATE1 mutation 5b) is not the define.
    want_defines = VARIANT_DEFINES[robot]
    other_defines = [d for d in ALL_DEFINES if d not in want_defines]
    for tu in exp:
        if not tu.endswith(CXX_EXTS):
            continue
        cmd = b.cmdline.get(tu)
        if cmd is None:
            problems.append(f"no compile command line captured for {tu} "
                            f"(V=1 output changed shape?)")
            continue
        tokens = cmd.split()
        if b.hash_expected and "-DSHULIB_BUILD_HASH=" not in cmd:
            problems.append(f"{tu} compiled WITHOUT -DSHULIB_BUILD_HASH — "
                            f"the §4.1 hash-drop defect is back")
        for want_define in want_defines:
            if want_define not in tokens:
                problems.append(f"build {robot} but {tu} compiled without the whole token "
                                f"{want_define} — the variant switch is a silent no-op "
                                f"again (§4.1)")
        for other in other_defines:
            if other in tokens:
                problems.append(f"build {robot} but {tu} carries another build's "
                                f"define {other}")

    # End-to-end, from the bytes of the linked ELF: the hash string must survive into it
    # (exactly the `strings` measurement that exposed §4.1's second half), and so must
    # THIS variant's identity beacon — and no other variant's (header, step 6).
    elf = os.path.join(REPO, "bin", "hot.package.elf")
    elf_bytes = b""
    if os.path.exists(elf):
        with open(elf, "rb") as f:
            elf_bytes = f.read()
    if b.hash_expected:
        if os.path.exists(elf) and b.hash_expected.encode() not in elf_bytes:
            problems.append(f"build hash '{b.hash_expected}' NOT in "
                            f"bin/hot.package.elf — the binary has no identity")
    else:
        problems.append("git describe yielded nothing — build-hash identity cannot be "
                        "asserted (a skip is not a pass; fix git, do not silence this)")
    if os.path.exists(elf):
        if _beacon_bytes(VARIANT_BEACONS[robot]) not in elf_bytes:
            problems.append(
                f"build {robot} but the beacon {VARIANT_BEACONS[robot].decode()!r} is NOT "
                f"in bin/hot.package.elf — {SELECTION_HEADER}'s #if chain did not take the "
                f"{robot} branch (a renamed macro? the define landed but the source "
                f"never saw it)")
        for v, beacon in VARIANT_BEACONS.items():
            if v != robot and _beacon_bytes(beacon) in elf_bytes:
                problems.append(
                    f"build {robot} but bin/hot.package.elf carries the {v} beacon "
                    f"{beacon.decode()!r} — this is a {v} build wearing a {robot} label")

        # Step 6's robot half (header): the ROBOT the branch set, from the beacon robotBeacon()
        # derives from kRobotId — what the typed variant beacon cannot vouch for.
        want_robot = BUILD_ROBOT.get(robot)
        if want_robot not in ROBOT_BEACONS:
            problems.append(f"build {robot} has no robot in BUILD_ROBOT (step 6)")
        else:
            want = ROBOT_BEACONS[want_robot]
            if _beacon_bytes(want) not in elf_bytes:
                problems.append(
                    f"build {robot} carries robot {want_robot}, but its robot beacon "
                    f"{want.decode()!r} is NOT in bin/hot.package.elf — the chain's branch for "
                    f"this build set another robot (kRobotId), so the build carries another "
                    f"robot's tables; or src/main.cpp no longer prints kRobotBeacon, so nothing "
                    f"links it (step 6, the robot beacon)")
            for other, beacon in ROBOT_BEACONS.items():
                if other != want_robot and _beacon_bytes(beacon) in elf_bytes:
                    problems.append(
                        f"build {robot} but bin/hot.package.elf carries the robot beacon "
                        f"{beacon.decode()!r} — this build carries robot {other}'s tables, not "
                        f"robot {want_robot}'s (step 6, the robot beacon)")

        # Step 8 (header): the PROGRAM, from the symbol table — what the beacon cannot vouch for.
        nm = find_nm()
        if nm is None:
            problems.append(f"{ARM_NM} not found — the program check (step 8) cannot run, and "
                            f"a skip is not a pass")
        else:
            try:
                problems += program_problems(robot, elf_bench_names(elf, nm))
            except (RuntimeError, OSError, subprocess.SubprocessError) as e:
                problems.append(f"the program check (step 8) could not read the symbol table: "
                                f"{e}")
    else:
        problems.append("bin/hot.package.elf missing — the beacon (step 6) and the program "
                        "(step 8) cannot be checked")

    return not problems, problems


def _summary(robot, b):
    allowed = (f"{len(b.allowed)} allowed ({', '.join(sorted(ALLOWED_CATEGORIES))}), "
               if ALLOWED_CATEGORIES else "")
    return (f"  [{robot:10}] exit {b.exit}, {len(b.compiled)}/{len(expected_tus())} TUs "
            f"compiled, warnings: {allowed}{len(b.vendor)} vendor-ignored, "
            f"{len(b.disallowed)} disallowed, hash "
            f"{'asserted (' + b.hash_expected + ')' if b.hash_expected else 'UNAVAILABLE'}"
            f", beacons (variant, robot) and program checked")


def cmd_check(path_env=None, quiet=False):
    def say(msg):
        if not quiet:
            print(msg)

    if find_compiler(path_env) is None or find_nm(path_env) is None:
        if not quiet:  # quiet is only ever the self-test proving this exact verdict
            print(f"src build gate: FAIL — {ARM_GXX} or {ARM_NM} not found on PATH.\n"
                  f"  The gate does NOT skip when its compiler is missing: a green "
                  f"tick that lies is worse than no gate.\n"
                  f"  Install it (CI: apt-get install gcc-arm-none-eabi) and re-run.",
                  file=sys.stderr)
        return 1

    # Step 0, once: five source checks (header) — they need no build.
    all_problems = [f"[one-chain] {p}" for p in one_chain_problems()]
    per_robot, _derive_problems = per_robot_table_names()
    say(f"  [one-chain] the build switches are read in {SELECTION_HEADER} only "
        f"({', '.join(r + '/' for r in ONE_CHAIN_ROOTS)} searched, CMake included), by "
        f"{'ONE chain setting program, robot and beacon in every branch' if not all_problems else 'NOT one chain'}"
        f"; no allowlisted file; no selection name a macro; {len(per_robot)} per-robot names "
        f"named only in the tables, test/ and static_asserts"
        f"{'' if not all_problems else ' — NOT CLEAN, see below'}")
    chain_problems = [f"[chain-table] {p}" for p in chain_table_problems()]
    all_problems += chain_problems
    say(f"  [chain-table] {len(VARIANTS)} builds, {len(ROBOT_BEACONS)} robots, "
        f"{'matching' if not chain_problems else 'NOT matching'} {SELECTION_HEADER}'s chain")
    table_problems = [f"[program-table] {p}" for p in program_table_problems()]
    all_problems += table_problems
    say(f"  [program-table] {len(PROGRAM_ENTRY_SYMBOLS)} programs, "
        f"{sum(len(v) for v in PROGRAM_ENTRY_SYMBOLS.values())} entry names, "
        f"{'matching' if not table_problems else 'NOT matching'} src/main.cpp")
    root_problems = [f"[include-root] {p}" for p in include_root_problems()]
    all_problems += root_problems
    src_files, src_dirs = _src_tree(REPO)
    say(f"  [include-root] {len(src_files)} source files in src/ and {len(src_dirs)} folders "
        f"({', '.join(d + '/' for d in src_dirs)}): "
        f"{'every include of one written from src/, no path a second root could answer' if not root_problems else 'NOT CLEAN, see below'}")
    path_problems = [f"[printed-path] {p}" for p in printed_path_problems()]
    all_problems += path_problems
    say(f"  [printed-path] every src/ path in a string literal under "
        f"{' and '.join(r + '/' for r in PRINTED_PATH_ROOTS)} "
        f"{'names a file or folder that exists' if not path_problems else 'does NOT — see below'}")

    for robot in VARIANTS:
        b = run_build(robot, path_env)
        ok, problems = verdict(robot, b)
        say(_summary(robot, b))
        if not ok:
            all_problems += [f"[{robot}] {p}" for p in problems]

    if all_problems:
        print("\nsrc build gate: FAIL", file=sys.stderr)
        for p in all_problems:
            print(f"  {p}", file=sys.stderr)
        return 1

    allowed = (f"{'/'.join(sorted(ALLOWED_CATEGORIES))} allowed by category, "
               if ALLOWED_CATEGORIES else "no category allowed, ")
    say(f"src build gate: PASS (all {len(VARIANTS)} builds link; policy: errors fail, "
        f"vendor warnings path-ignored, {allowed}every other warning fails; defines "
        f"token-exact; one chain picks program and robot; variant and robot beacons asserted; "
        f"programs asserted; src/'s include root unambiguous; every printed src/ path exists)")
    return 0


# ── self-test ──────────────────────────────────────────────────────────────────────
# Prove each detector CAN fire: plant the fault, run the REAL build against the planted
# tree, confirm the verdict, restore byte-exact. A gate nobody has seen go red is a
# rumour (doc_staleness_audit.py precedent).

SRC_PLANT = os.path.join(REPO, "src", "programs", "bench_r3a.cpp")  # unguarded in every build
MAIN_CPP_PATH = os.path.join(REPO, MAIN_CPP)                      # the dispatcher (cases 18-22)
SELECTION_PATH = os.path.join(REPO, SELECTION_HEADER)             # the one chain (16-19, 21-25)
CHASSIS_TABLE_PATH = os.path.join(REPO, "src", "robots", "chassis_table.hpp")      # (19, 23)
MECHANISM_TABLE_PATH = os.path.join(REPO, "src", "robots", "mechanism_table.hpp")  # (19, 23, 28)
LIBRARY_PLANT = os.path.join(REPO, "include", "shulib", "config", "robot_config.hpp")  # (19)
TEST_PLANT = os.path.join(REPO, "test", "build_selection_test.cpp")                 # (19)
VENDORED_TEST_PLANT = os.path.join(REPO, "test", "vendor", "doctest.h")              # (19)
DRIVE_PLANT = os.path.join(REPO, "src", "programs", "drive_program.cpp")           # (26, 29)
CALIBRATE_PLANT = os.path.join(REPO, "src", "programs", "calibrate_program.cpp")   # (26)
CMAKE_PLANT = os.path.join(REPO, "test", "CMakeLists.txt")                          # (27)
COMMON_PLANT = os.path.join(REPO, "src", "common", "boot_source.hpp")               # (28)
CLAWS_PLANT = os.path.join(REPO, "src", "programs", "teleop_claws.hpp")             # (28)
TEAM_DIR = os.path.join(REPO, "src", "team")                                        # (28)
TEAM_README = os.path.join(TEAM_DIR, "README.md")                                   # (28)
HOST_INCLUDE_PLANT = os.path.join(REPO, "test", "chassis_table_test.cpp")           # (28)
VENDOR_PLANT = os.path.join(REPO, "include", "liblvgl", "core", "lv_obj_style.h")
MAKEFILE = os.path.join(REPO, "Makefile")


class _plant:
    """Append `addition` to `path` for the duration of the block; restore byte-exact."""

    def __init__(self, path, addition):
        self.path, self.addition = path, addition

    def __enter__(self):
        with open(self.path, "rb") as f:
            self.original = f.read()
        with open(self.path, "ab") as f:
            f.write(self.addition.encode())

    def __exit__(self, *exc):
        with open(self.path, "wb") as f:
            f.write(self.original)
        with open(self.path, "rb") as f:
            assert f.read() == self.original, f"restore failed for {self.path}"
        return False


class _transform:
    """Rewrite `path` with `fn(text)` for the duration of the block; restore byte-exact. Asserts
    the rewrite changed something, so a plant that silently matched nothing cannot pass."""

    def __init__(self, path, fn):
        self.path, self.fn = path, fn

    def __enter__(self):
        with open(self.path, "rb") as f:
            self.original = f.read()
        new = self.fn(self.original.decode()).encode()
        assert new != self.original, f"the plant changed nothing in {self.path}"
        with open(self.path, "wb") as f:
            f.write(new)

    def __exit__(self, *exc):
        with open(self.path, "wb") as f:
            f.write(self.original)
        with open(self.path, "rb") as f:
            assert f.read() == self.original, f"restore failed for {self.path}"
        return False


class _replace:
    """Replace the ONE occurrence of `old` in `path` with `new` for the duration of the
    block; restore byte-exact. Asserts exactly one occurrence, so a plant that silently
    matched nothing (the §4.1 class, inside the gate's own self-test) cannot pass."""

    def __init__(self, path, old, new):
        self.path, self.old, self.new = path, old.encode(), new.encode()

    def __enter__(self):
        with open(self.path, "rb") as f:
            self.original = f.read()
        assert self.original.count(self.old) == 1, \
            f"plant target must occur exactly once in {self.path}: {self.old!r}"
        with open(self.path, "wb") as f:
            f.write(self.original.replace(self.old, self.new))

    def __exit__(self, *exc):
        with open(self.path, "wb") as f:
            f.write(self.original)
        with open(self.path, "rb") as f:
            assert f.read() == self.original, f"restore failed for {self.path}"
        return False


class _newfile:
    """Create `path` (which must not exist) holding `content` for the duration of the block — and
    any parent directory it lacks (SRC1: the include-root cases plant `include/robots/…`); remove
    the file and exactly the directories it created, and assert they are gone — a probe file never
    outlives its case."""

    def __init__(self, path, content):
        self.path, self.content = path, content

    def __enter__(self):
        assert not os.path.exists(self.path), f"the probe file already exists: {self.path}"
        missing, d = [], os.path.dirname(self.path)
        while not os.path.exists(d):
            missing.append(d)
            d = os.path.dirname(d)
        self.made = list(reversed(missing))
        for d in self.made:
            os.mkdir(d)
        with open(self.path, "w", encoding="utf-8") as f:
            f.write(self.content)

    def __exit__(self, *exc):
        os.remove(self.path)
        for d in reversed(self.made):
            os.rmdir(d)
        assert not os.path.exists(self.path), f"the probe file was not removed: {self.path}"
        assert not any(os.path.exists(d) for d in self.made), \
            f"a probe directory was not removed: {self.made}"
        return False


def _chain_table_problems_without(build):
    """Self-test case 33: step 0's chain-table check with `build` dropped from VARIANTS ALONE,
    VARIANTS restored before returning."""
    global VARIANTS
    saved = VARIANTS
    try:
        VARIANTS = tuple(v for v in saved if v != build)
        return chain_table_problems()
    finally:
        VARIANTS = saved


def do_self_test():
    if find_compiler() is None or find_nm() is None:
        print(f"src build gate self-test: FAIL — {ARM_GXX} or {ARM_NM} not found; the "
              f"self-test runs real builds and does not skip.", file=sys.stderr)
        return 1

    failures = []
    checks = 0

    def expect(cond, msg):
        nonlocal checks
        checks += 1
        if not cond:
            failures.append(msg)

    # 1: a syntax error in a src/ file -> FAIL, naming the file (the data-abort class).
    with _plant(SRC_PLANT, "\nthis line is not C++\n"):
        b = run_build("bench")
        ok, problems = verdict("bench", b)
        expect(not ok, "1: a syntax error in src/ was not caught")
        expect(any("bench_r3a.cpp" in p for p in problems),
               "1: the failure did not name the broken file")

    # 2: a NEW disallowed warning in src/ -> FAIL (policy is not "ignore all warnings").
    with _plant(SRC_PLANT, "\nstatic int gate1_selftest_unused_variable = 42;\n"):
        b = run_build("bench")
        ok, problems = verdict("bench", b)
        expect(not ok, "2: a disallowed warning (-Wunused-variable) in src/ passed")
        expect(any("-Wunused-variable" in p and "bench_r3a.cpp" in p for p in problems),
               "2: the failure did not name the category and file")

    # 3: a NEW dead function in src/ -> FAIL, naming -Wunused-function and the file (the team
    #    lead's ruling of 2026-09-26, G1 Part 2c's final run: the category that GATE1 allowed is
    #    a FAILURE — header). Case 4 is what still proves the policy is not "ban every warning
    #    everywhere": a vendored header's warning is ignored by path.
    with _plant(SRC_PLANT, "\nnamespace { void gate1_selftest_dead_function() {} }\n"):
        b = run_build("bench")
        ok, problems = verdict("bench", b)
        expect(not ok, "3: a -Wunused-function in src/ PASSED the gate — the category is "
                       "allowed again (the team lead ruled it a FAILURE)")
        expect(any("-Wunused-function" in p and "bench_r3a.cpp" in p for p in problems),
               f"3: the failure did not name -Wunused-function and the file: {problems}")
        expect(not b.allowed, f"3: a warning was ALLOWED by category: {b.allowed}")

    # 4: a vendor-path warning of a DISALLOWED category -> PASS (proves the exclusion
    #    is by PATH — the category alone would fail it).
    with _plant(VENDOR_PLANT, "\nstatic int gate1_selftest_vendor_probe = 7;\n"):
        b = run_build("bench")
        ok, problems = verdict("bench", b)
        expect(ok, f"4: a vendor-path warning FAILED the gate: {problems}")
        expect(any("lv_obj_style.h" in p and c == "-Wunused-variable"
                   for p, _l, c in b.vendor),
               "4: the planted vendor warning never fired (the case proved nothing)")

    # 5: nothing planted (the real tree) -> PASS, via the full every-variant check.
    expect(cmd_check(quiet=True) == 0, "5: the gate FAILS on the real, unplanted tree")

    # 6: missing toolchain -> FAIL, loudly, before any make runs (a skip is not a pass).
    with tempfile.TemporaryDirectory() as empty:
        expect(cmd_check(path_env=empty, quiet=True) == 1,
               "6: a missing toolchain did not FAIL the gate (skip-and-pass is the "
               "worst failure mode this gate has)")

    # 7: the Makefile's tank append DROPPED (brief R3b-S2 test 15's mutation: ROBOT=tank
    #    silently builds bench) -> FAIL, structural detector naming the define, AND the beacon.
    #    (The warning-count detector retired at G1 Part 2c never could see this one: tank
    #    behaved like bench to it.)
    tank_line = "override EXTRA_CXXFLAGS+=-DSHULIB_ROBOT_TANK_2026\n"
    with _replace(MAKEFILE, tank_line, "# (self-test plant: tank append dropped)\n"):
        b = run_build("tank")
        ok, problems = verdict("tank", b)
        expect(not ok, "7: ROBOT=tank with its Makefile append dropped PASSED — the "
                       "silent-no-op class is back for the third variant")
        expect(any("-DSHULIB_ROBOT_TANK_2026" in p and "silent no-op" in p
                   for p in problems),
               "7: the failure did not name the missing tank define")
        expect(any("beacon" in p for p in problems),
               "7: the beacon did not ALSO catch the bench build wearing the tank label")

    # 8: a LOOK-ALIKE tank define (GATE1 mutation 5b's shape) -> FAIL on the token-exact
    #    structural check AND on the beacon — a substring check would have passed this,
    #    and for tank nothing else could have caught it.
    with _replace(MAKEFILE, tank_line,
                  "override EXTRA_CXXFLAGS+=-DSHULIB_ROBOT_TANK_2026_BROKEN\n"):
        b = run_build("tank")
        ok, problems = verdict("tank", b)
        expect(not ok, "8: a look-alike tank define PASSED (the substring hole is open)")
        expect(any("whole token -DSHULIB_ROBOT_TANK_2026" in p for p in problems),
               "8: the token-exact structural check did not fire on the look-alike")
        expect(any("beacon" in p and "NOT in" in p for p in problems),
               "8: the beacon check did not fire on the look-alike")

    # 9: the same drop for xdrive (GATE1's hand-run mutation 5a, now permanent) -> FAIL.
    xdrive_line = "override EXTRA_CXXFLAGS+=-DSHULIB_ROBOT_XDRIVE_INVENTED\n"
    with _replace(MAKEFILE, xdrive_line, "# (self-test plant: xdrive append dropped)\n"):
        b = run_build("xdrive")
        ok, problems = verdict("xdrive", b)
        expect(not ok, "9: ROBOT=xdrive with its Makefile append dropped PASSED")
        expect(any("-DSHULIB_ROBOT_XDRIVE_INVENTED" in p for p in problems),
               "9: the failure did not name the missing xdrive define")

    # 10: the Makefile's PROGRAM=drive append DROPPED (R3b Part 0b: `make ROBOT=tank
    #     PROGRAM=drive` silently builds the tester) -> FAIL on the token-exact structural
    #     check naming the define AND on the beacon (a tank build wearing the tank-drive
    #     label). Both detectors must fire on their own (the retired warning-count detector
    #     never could see this one: tank-drive behaved like tank to it).
    drive_line = "override EXTRA_CXXFLAGS+=-DSHULIB_PROGRAM_DRIVE\n"
    with _replace(MAKEFILE, drive_line, "# (self-test plant: drive append dropped)\n"):
        b = run_build("tank-drive")
        ok, problems = verdict("tank-drive", b)
        expect(not ok, "10: PROGRAM=drive with its Makefile append dropped PASSED — the "
                       "silent-no-op class is back on the second axis")
        expect(any("whole token -DSHULIB_PROGRAM_DRIVE" in p for p in problems),
               "10: the failure did not name the missing PROGRAM define")
        expect(any("beacon" in p and "NOT in" in p for p in problems),
               "10: the tank-drive beacon's absence was not caught")
        expect(any("tank build wearing a tank-drive label" in p for p in problems),
               "10: the tank beacon's presence (NUL-terminated match) was not caught")

    # 11: the real tank-drive build, alone -> PASS, and the tank beacon must NOT be reported
    #     (the prefix hazard: `…=tank` inside `…=tank-drive` would false-fail without the
    #     NUL-terminated match).
    b = run_build("tank-drive")
    ok, problems = verdict("tank-drive", b)
    expect(ok, f"11: the real tank-drive build FAILED the gate: {problems}")

    # 12: the Makefile's PROGRAM=library append DROPPED (R3b Parts 1–3, brief test 19:
    #     `make ROBOT=tank PROGRAM=library` silently builds the tester) -> FAIL on the
    #     token-exact structural check naming the define AND on the beacon (a tank build
    #     wearing the tank-library label). Both detectors must fire (the retired warning-count
    #     detector never could see this one).
    library_line = "override EXTRA_CXXFLAGS+=-DSHULIB_PROGRAM_LIBRARY\n"
    with _replace(MAKEFILE, library_line, "# (self-test plant: library append dropped)\n"):
        b = run_build("tank-library")
        ok, problems = verdict("tank-library", b)
        expect(not ok, "12: PROGRAM=library with its Makefile append dropped PASSED — the "
                       "silent-no-op class is back for the library program")
        expect(any("whole token -DSHULIB_PROGRAM_LIBRARY" in p for p in problems),
               "12: the failure did not name the missing PROGRAM=library define")
        expect(any("beacon" in p and "NOT in" in p for p in problems),
               "12: the tank-library beacon's absence was not caught")
        expect(any("tank build wearing a tank-library label" in p for p in problems),
               "12: the tank beacon's presence (NUL-terminated match) was not caught")

    # 13: the real tank-library build, alone -> PASS (the `…=tank` prefix inside
    #     `…=tank-library` must not false-fail, and the library graph must link).
    b = run_build("tank-library")
    ok, problems = verdict("tank-library", b)
    expect(ok, f"13: the real tank-library build FAILED the gate: {problems}")

    # 14: the Makefile's PROGRAM=calibrate append DROPPED (R3d Part 2: `make ROBOT=tank
    #     PROGRAM=calibrate` silently builds the tester) -> FAIL on the token-exact structural
    #     check naming the define AND on the beacon (a tank build wearing the tank-calibrate
    #     label). Both detectors must fire (the retired warning-count detector never could see
    #     this one). This is the one program that POWERS THE
    #     MOTORS on a robot whose wiring it does not yet know, so an upload that was silently the
    #     tester is the worst confusion on the slot list.
    calibrate_line = "override EXTRA_CXXFLAGS+=-DSHULIB_PROGRAM_CALIBRATE\n"
    with _replace(MAKEFILE, calibrate_line, "# (self-test plant: calibrate append dropped)\n"):
        b = run_build("tank-calibrate")
        ok, problems = verdict("tank-calibrate", b)
        expect(not ok, "14: PROGRAM=calibrate with its Makefile append dropped PASSED — the "
                       "silent-no-op class is back for the calibrate program")
        expect(any("whole token -DSHULIB_PROGRAM_CALIBRATE" in p for p in problems),
               "14: the failure did not name the missing PROGRAM=calibrate define")
        expect(any("beacon" in p and "NOT in" in p for p in problems),
               "14: the tank-calibrate beacon's absence was not caught")
        expect(any("tank build wearing a tank-calibrate label" in p for p in problems),
               "14: the tank beacon's presence (NUL-terminated match) was not caught")

    # 15: the real tank-calibrate build, alone -> PASS (the `…=tank` prefix inside
    #     `…=tank-calibrate` must not false-fail, and the calibrate program must link).
    b = run_build("tank-calibrate")
    ok, problems = verdict("tank-calibrate", b)
    expect(ok, f"15: the real tank-calibrate build FAILED the gate: {problems}")

    # ── G1 Part 2c: what REPLACED the retired variant differential (header) ────────────
    # 16: a branch of THE ONE CHAIN (src/build_selection.hpp since 2c's final run) broken — the
    #     define lands on the command line but the #elif that reads it is misspelt (the case the
    #     retired differential existed for, which it could see only for xdrive) -> the
    #     tank-library build silently becomes the tester, and the VARIANT BEACON must fail it:
    #     the tank-library beacon absent, the tank one present. The structural define check must
    #     NOT be what fires (the define did land), nor the robot beacon (both branches are robot
    #     two's) — each detector is proven on its own.
    library_branch = "defined(SHULIB_PROGRAM_LIBRARY)\ninline constexpr Program kProgram"
    with _replace(SELECTION_PATH, library_branch,
                  "defined(SHULIB_PROGRAM_LIBRARY_BROKEN)\ninline constexpr Program kProgram"):
        b = run_build("tank-library")
        ok, problems = verdict("tank-library", b)
        expect(not ok, "16: a broken #elif in the one chain PASSED the tank-library build — "
                       "the beacon no longer vouches for the program")
        expect(any("beacon 'shulib-robot-variant=tank-library' is NOT in" in p for p in problems),
               "16: the tank-library beacon's absence was not caught")
        expect(any("tank build wearing a tank-library label" in p for p in problems),
               "16: the tank beacon's presence was not caught")
        expect(not any("whole token" in p for p in problems),
               "16: the define check fired — the plant was meant to leave the define landing")
        expect(not any("robot beacon" in p for p in problems),
               "16: the robot beacon fired — both branches set robot two")

    # 17: the same for the xdrive branch (the chain's first line) -> the xdrive build falls
    #     through to the bench tester ON THE BENCH BOT'S TABLES, and both beacons must fail it:
    #     the variant beacon (a bench build wearing the xdrive label) and the robot beacon (the
    #     bench bot's robot beacon present, the X-drive's absent).
    xdrive_branch = "#if defined(SHULIB_ROBOT_XDRIVE_INVENTED)\ninline constexpr Program kProgram"
    with _replace(SELECTION_PATH, xdrive_branch,
                  "#if defined(SHULIB_ROBOT_XDRIVE_INVENTED_BROKEN)\n"
                  "inline constexpr Program kProgram"):
        b = run_build("xdrive")
        ok, problems = verdict("xdrive", b)
        expect(not ok, "17: a broken #if on the chain's xdrive branch PASSED the xdrive build")
        expect(any("bench build wearing a xdrive label" in p for p in problems),
               "17: the bench beacon in an xdrive build was not caught")
        expect(any("robot beacon 'shulib-robot=invented-xdrive' is NOT in" in p
                   for p in problems),
               f"17: the X-drive's missing robot beacon was not caught: {problems}")
        expect(any("carries the robot beacon 'shulib-robot=bench-bot'" in p for p in problems),
               f"17: the bench bot's robot beacon in an xdrive build was not caught: {problems}")

    # 18: a program added to the `Program` enum (src/build_selection.hpp) and not handled by the
    #     entry points' switches -> -Wswitch in src/main.cpp, a disallowed warning -> FAIL,
    #     naming the category. This is the C++ half of the redesign: below the chain, a program
    #     is chosen by name, and the compiler — through this policy — refuses an entry point that
    #     forgot one. EVERY ONE of the five PROS entry points must be in that net (the review of
    #     2c found disabled() and competition_initialize() outside it, as `if`s): each must hold
    #     a `switch (kProgram)`, and the planted enumerator must draw a -Wswitch at each of those
    #     lines (the plant is in the header, so main.cpp's lines do not move).
    with open(MAIN_CPP_PATH, encoding="utf-8") as fh:
        main_src = fh.read()
    entry_switch_lines = {}
    for fn in ENTRY_POINTS:
        m = re.search(r"^void " + fn + r"\(\) \{\n(.*?)^\}", main_src, flags=re.S | re.M)
        body_start = main_src[:m.start(1)].count("\n") + 1 if m else None
        k = m.group(1).find("switch (kProgram)") if m else -1
        expect(k >= 0, f"18: {fn}() in src/main.cpp holds no `switch (kProgram)` — a new "
                       f"program would not be a -Wswitch failure there")
        if k >= 0:
            entry_switch_lines[fn] = str(body_start + m.group(1)[:k].count("\n"))
    enum_tail = "    InventedXDrive,  ///< the invented X-drive (HA-111)\n"
    with _replace(SELECTION_PATH, enum_tail,
                  enum_tail + "    SelfTestUnhandled,  ///< self-test plant (case 18)\n"):
        b = run_build("bench")
        ok, problems = verdict("bench", b)
        expect(not ok, "18: an unhandled Program enumerator PASSED the gate")
        expect(any("-Wswitch" in p and "main.cpp" in p for p in problems),
               "18: the failure did not name -Wswitch in src/main.cpp")
        switch_lines = {lno for path, lno, cat, _m in b.disallowed
                        if cat == "-Wswitch" and path == MAIN_CPP}
        for fn, lno in entry_switch_lines.items():
            expect(lno in switch_lines,
                   f"18: {fn}()'s switch drew no -Wswitch (lines warned: {sorted(switch_lines)})")

    # 19: THE ONE-CHAIN RULE (step 0), a source check. The real tree reports nothing; a build
    #     switch named ANYWHERE outside src/build_selection.hpp is reported — in every directive
    #     form, in a laundering #define and in plain code, in main.cpp, in another src/ file, in
    #     EACH TABLE (the team lead's ruling: the tables choose from kRobotId and have no #if of
    #     their own), in a library header and in a host test (a test that #defines one, too); a
    #     switch named only in a comment or a string literal is not, nor one in a vendored file,
    #     nor the library's documented `kRobot{…}` idiom; in the header, a second block reading a
    #     switch outside the #error guards is a SECOND chain, and a chain constant declared
    #     outside the chain is reported. No build needed; every plant restored byte-exact.
    expect(one_chain_problems() == [],
           f"19: the real tree broke the one-chain rule: {one_chain_problems()}")
    only_header = f"only {SELECTION_HEADER} may read them"
    plants = [
        (MAIN_CPP_PATH, "#if defined(SHULIB_PROGRAM_LIBRARY)"),
        (SRC_PLANT, "#if defined(SHULIB_PROGRAM_LIBRARY)"),
        (SRC_PLANT, "#ifdef SHULIB_ROBOT_TANK_2026"),
        (MAIN_CPP_PATH, "#ifndef SHULIB_PROGRAM_DRIVE"),
        (MAIN_CPP_PATH, "#if 0\n#elifdef SHULIB_ROBOT_XDRIVE_INVENTED"),
        (SRC_PLANT, "#if 0\n#elifndef SHULIB_PROGRAM_LIBRARY"),
    ]
    for table in (CHASSIS_TABLE_PATH, MECHANISM_TABLE_PATH):
        plants += [
            (table, "#if defined(SHULIB_ROBOT_TANK_2026)"),
            (table, "#ifdef SHULIB_ROBOT_TANK_2026"),
            (table, "#if 0\n#elifdef SHULIB_ROBOT_TANK_2026"),
        ]
    # ...and outside src/: a library header every src/ file includes, and a host test TU.
    plants += [
        (LIBRARY_PLANT, "#ifdef SHULIB_ROBOT_TANK_2026"),
        (TEST_PLANT, "#if defined(SHULIB_PROGRAM_LIBRARY)"),
    ]
    for where, form in plants:
        rel = os.path.relpath(where, REPO)
        with _plant(where, f"\n{form}\nstatic int self_test_directive_form;\n#endif\n"):
            found = one_chain_problems()
            expect(any(rel in p and only_header in p and form.split("\n")[-1] in p
                       for p in found),
                   f"19: `{form.splitlines()[-1]}` in {rel} was not reported: {found}")
    for table in (CHASSIS_TABLE_PATH, MECHANISM_TABLE_PATH):
        rel = os.path.relpath(table, REPO)
        for code in ("#define SELF_TEST_IS_TANK defined(SHULIB_ROBOT_TANK_2026)",
                     "inline constexpr bool kSelfTestTank = SHULIB_ROBOT_TANK_2026 + 0;"):
            with _plant(table, f"\n{code}\n"):
                found = one_chain_problems()
                expect(any(rel in p and only_header in p for p in found),
                       f"19: `{code}` in {rel} was not reported: {found}")
        with _plant(table, "\nnamespace shulib::bench::self_test {\n"
                           "inline constexpr RobotId kRobotId = RobotId::BenchBot;\n}\n"):
            found = one_chain_problems()
            expect(any(rel in p and "declares one of THE ONE CHAIN's constants" in p
                       for p in found),
                   f"19: a shadowing kRobotId in {rel} was not reported: {found}")
    with _plant(SRC_PLANT, "\n// SHULIB_ROBOT_TANK_2026, named in a comment\n"
                           "static const char* const kSelfTestHint = "
                           "\"make sets -DSHULIB_ROBOT_TANK_2026\";\n"):
        expect(one_chain_problems() == [],
               f"19: a switch named in a comment or a string was reported as READ: "
               f"{one_chain_problems()}")
    # A host test that DEFINES a switch before including a table (the ODR hazard) is reported;
    # a vendored file is not read at all.
    with _plant(TEST_PLANT, "\n#define SHULIB_ROBOT_TANK_2026 1\n"):
        found = one_chain_problems()
        expect(any(os.path.relpath(TEST_PLANT, REPO) in p and only_header in p for p in found),
               f"19: a test TU defining a build switch was not reported: {found}")
    with _plant(VENDORED_TEST_PLANT, "\n#ifdef SHULIB_ROBOT_TANK_2026\n#endif\n"):
        expect(one_chain_problems() == [],
               f"19: a vendored file was read as ours: {one_chain_problems()}")
    # The library's documented idiom (include/shulib/config/robot_config.hpp: `inline constexpr
    # RobotConfig kRobot{…};`) stays free in src/: the chain's robot constant is kRobotId, so a
    # robot typed by that idiom is not a chain constant.
    with _plant(SRC_PLANT, "\nnamespace shulib::bench::self_test { "
                           "inline constexpr int kRobot{0}; }\n"):
        expect(one_chain_problems() == [],
               f"19: the documented `kRobot{{…}}` idiom was reported as a chain constant: "
               f"{one_chain_problems()}")
    rel = SELECTION_HEADER
    for form, expected in (
            ("#if defined(SHULIB_PROGRAM_LIBRARY)\nstatic int self_test_second_chain;\n#endif",
             "opens a SECOND chain"),
            ("#ifdef SHULIB_ROBOT_TANK_2026\nstatic int self_test_second_chain;\n#endif",
             "opens a SECOND chain"),
            ("#if 0\n#elifndef SHULIB_PROGRAM_LIBRARY\nstatic int self_test_x;\n#endif",
             "reads a build switch outside an #error guard"),
            ("namespace shulib::bench::self_test {\n"
             "inline constexpr Program kProgram = Program::Drive;\n}",
             "declares one of the chain's constants outside THE ONE CHAIN")):
        with _plant(SELECTION_PATH, f"\n{form}\n"):
            found = one_chain_problems()
            expect(any(rel in p and expected in p for p in found),
                   f"19: `{form.splitlines()[0]}` in {rel} was not reported ({expected}): "
                   f"{found}")
    # a branch that does not set all three, or sets something else too
    with _replace(SELECTION_PATH,
                  "inline constexpr RobotId kRobotId = RobotId::RobotTwo;\n"
                  "inline constexpr char kVariantBeacon[] = \"shulib-robot-variant=tank\";\n",
                  "inline constexpr char kVariantBeacon[] = \"shulib-robot-variant=tank\";\n"
                  "inline constexpr int kSelfTestExtra = 1;\n"):
        found = one_chain_problems()
        expect(any("sets kRobotId 0 times" in p for p in found),
               f"19: a branch that sets no robot was not reported: {found}")
        expect(any("is not one of its three constants" in p for p in found),
               f"19: an extra line inside a branch was not reported: {found}")
    expect(one_chain_problems() == [], "19: a plant was not restored")

    # ── G1 Part 2c's review: what the beacon CANNOT vouch for (header, step 8) ──────────────
    # 20: opcontrol()'s Bench Tests and Drive calls SWAPPED. -Wswitch is satisfied, every beacon
    #     is right, steps 0-7 pass — and the "Bench Tests" upload drives the motors from the
    #     sticks. Step 8 must fail it on BOTH halves (Drive's function linked; the tester's
    #     missing), and no beacon may be what fires.
    swapped_from = ("        case Program::BenchTests:\n            shulib::bench::runR3a();\n"
                    "            return;\n        case Program::Drive:\n"
                    "            shulib::bench::runDrive();\n")
    swapped_to = swapped_from.replace("runR3a()", "SWAP_A").replace(
        "runDrive()", "runR3a()").replace("SWAP_A", "runDrive()")
    with _replace(MAIN_CPP_PATH, swapped_from, swapped_to):
        b = run_build("bench")
        ok, problems = verdict("bench", b)
        expect(not ok, "20: opcontrol() running shulib Drive in the Bench Tests build PASSED")
        expect(any("links the Drive program's shulib::bench::runDrive" in p for p in problems),
               f"20: Drive's function linked into the bench build was not reported: {problems}")
        expect(any("does not link its shulib::bench::runR3a" in p for p in problems),
               f"20: the tester's missing opcontrol entry was not reported: {problems}")
        expect(not any("beacon" in p for p in problems),
               "20: a beacon fired — the plant was meant to leave every beacon right")

    # 21: the tank-library branch assigns the WRONG program (`kProgram = Program::BenchTests`)
    #     under the RIGHT beacon: the "shulib Teleop" upload is silently the tester (HA-111's
    #     defect class). The one-chain rule and the program table pass (the branch still sets
    #     all three); step 0's CHAIN TABLE reports it from the source, and step 8 — proven here
    #     on its own, from the build — must fail it on both halves, with no beacon firing.
    with _replace(SELECTION_PATH, "inline constexpr Program kProgram = Program::Teleop;",
                  "inline constexpr Program kProgram = Program::BenchTests;"):
        expect(one_chain_problems() == [] and program_table_problems() == [],
               "21: the plant tripped the one-chain rule or the program table")
        expect(any("sets kProgram = Program::BenchTests, but this gate's BUILD_PROGRAM says "
                   "Teleop" in p for p in chain_table_problems()),
               f"21: the chain table did not report the wrong program: {chain_table_problems()}")
        b = run_build("tank-library")
        ok, problems = verdict("tank-library", b)
        expect(not ok, "21: a tank-library build that is the tester PASSED")
        expect(any("links the BenchTests program's shulib::bench::runR3a" in p
                   for p in problems),
               f"21: the tester's function in the tank-library build was not reported: {problems}")
        expect(any("does not link its shulib::bench::teleopInitialize" in p for p in problems),
               f"21: Teleop's missing boot was not reported: {problems}")
        expect(not any("beacon" in p for p in problems),
               "21: a beacon fired — the plant was meant to leave every beacon right")

    # 22: step 0's program table against the source (header): a Program enumerator the gate's
    #     table does not know, and a shulib::bench:: name main.cpp uses that belongs to no
    #     program, must each be reported; the real tree reports nothing. Source-only.
    expect(program_table_problems() == [],
           f"22: the real tree's program table does not match: {program_table_problems()}")
    with _replace(SELECTION_PATH, enum_tail,
                  enum_tail + "    SelfTestProgram,  ///< self-test plant (case 22)\n"):
        found = program_table_problems()
        expect(any("Program::SelfTestProgram is not in" in p for p in found),
               f"22: an enumerator missing from the gate's table was not reported: {found}")
    with _plant(MAIN_CPP_PATH,
                "\nvoid selfTestUnowned() { shulib::bench::selfTestUnownedEntry(); }\n"):
        found = program_table_problems()
        expect(any("shulib::bench::selfTestUnownedEntry, which belongs to no program" in p
                   for p in found),
               f"22: a name main.cpp uses that no program owns was not reported: {found}")
    with _plant(MAIN_CPP_PATH, "\n// shulib::bench::onlyInAComment(); src/*.cpp\n"):
        expect(program_table_problems() == [],
               "22: a name inside a comment was counted as main.cpp using it")
    expect(program_table_problems() == [], "22: a plant was not restored")

    # ── G1 Part 2c's final run: the ROBOT (the team lead's ruling "one chain picks both") ────
    # 23: a robot added to the `RobotId` enum and handled nowhere -> -Wswitch at robotBeacon()
    #     (src/build_selection.hpp), chassisTableFor() (src/robots/chassis_table.hpp) and
    #     mechanismTableFor() (src/robots/mechanism_table.hpp): a robot cannot be added without saying
    #     what its beacon and its tables are. Step 0's chain table reports it from the source too.
    robot_tail = "    InventedXDrive,  ///< the invented X-drive (A4 register HA-111; no such robot exists)\n"
    with _replace(SELECTION_PATH, robot_tail,
                  robot_tail + "    SelfTestRobot,  ///< self-test plant (case 23)\n"):
        expect(any("RobotId enumerators" in p for p in chain_table_problems()),
               f"23: the chain table did not report an unknown robot: {chain_table_problems()}")
        b = run_build("bench")
        ok, problems = verdict("bench", b)
        expect(not ok, "23: an unhandled RobotId enumerator PASSED the gate")
        warned = {path for path, _lno, cat, _m in b.disallowed if cat == "-Wswitch"}
        for where in (SELECTION_HEADER, os.path.relpath(CHASSIS_TABLE_PATH, REPO),
                      os.path.relpath(MECHANISM_TABLE_PATH, REPO)):
            expect(where in warned,
                   f"23: no -Wswitch in {where} for the unhandled robot (warned: {sorted(warned)})")

    # 24: THE WRONG ROBOT INSIDE A BRANCH (the ruling's case): the tank-library branch sets
    #     `kRobotId = RobotId::BenchBot` under the right program and the right beacon. The build
    #     compiles, links and runs "shulib Teleop" — over the bench bot's chassis table and no
    #     claws. Step 0's chain table reports it from the source; from the package, ONLY the
    #     robot beacon can (the variant beacon is typed in the branch and step 8 checks programs):
    #     robot two's robot beacon absent, the bench bot's present, and nothing else firing.
    with _replace(SELECTION_PATH,
                  "inline constexpr Program kProgram = Program::Teleop;\n"
                  "inline constexpr RobotId kRobotId = RobotId::RobotTwo;\n",
                  "inline constexpr Program kProgram = Program::Teleop;\n"
                  "inline constexpr RobotId kRobotId = RobotId::BenchBot;\n"):
        expect(one_chain_problems() == [] and program_table_problems() == [],
               "24: the plant tripped the one-chain rule or the program table")
        expect(any("sets kRobotId = RobotId::BenchBot, but this gate's BUILD_ROBOT says RobotTwo"
                   in p for p in chain_table_problems()),
               f"24: the chain table did not report the wrong robot: {chain_table_problems()}")
        b = run_build("tank-library")
        ok, problems = verdict("tank-library", b)
        expect(not ok, "24: a tank-library build carrying the bench bot's tables PASSED")
        expect(any("robot beacon 'shulib-robot=robot-two' is NOT in" in p for p in problems),
               f"24: robot two's missing robot beacon was not caught: {problems}")
        expect(any("carries the robot beacon 'shulib-robot=bench-bot'" in p for p in problems),
               f"24: the bench bot's robot beacon in a tank build was not caught: {problems}")
        expect(not any("wearing a" in p or "'shulib-robot-variant=" in p or "(step 8)" in p
                       for p in problems),
               f"24: the variant beacon or step 8 fired — only the robot beacon can see this: "
               f"{problems}")

    # 25: the program and the robot SPLIT across two chains again (the shape the tables had
    #     until the ruling): every kRobotId moved out of the one chain into a second chain of its
    #     own. As written it even picks the right robots — step 0 must FAIL it anyway (a second
    #     chain; branches that set no robot). And the reason it is forbidden, measured: the same
    #     split with the tank define misspelt IN THE ROBOT CHAIN gives a tank build the bench
    #     bot's tables under the right variant beacon, and only the robot beacon sees it.
    def split(text, tank_define="SHULIB_ROBOT_TANK_2026"):
        text = re.sub(r"^inline constexpr RobotId kRobotId = RobotId::\w+;\n", "", text,
                      flags=re.M)
        second = ("#if defined(SHULIB_ROBOT_XDRIVE_INVENTED)\n"
                  "inline constexpr RobotId kRobotId = RobotId::InventedXDrive;\n"
                  f"#elif defined({tank_define})\n"
                  "inline constexpr RobotId kRobotId = RobotId::RobotTwo;\n"
                  "#else\n"
                  "inline constexpr RobotId kRobotId = RobotId::BenchBot;\n"
                  "#endif\n")
        anchor = "\n/// THE ROBOT BEACON of robot"
        assert text.count(anchor) == 1
        return text.replace(anchor, "\n" + second + anchor)
    with _transform(SELECTION_PATH, split):
        found = one_chain_problems()
        expect(any("opens a SECOND chain" in p for p in found),
               f"25: the robot's own chain was not reported: {found}")
        expect(any("sets kRobotId 0 times" in p for p in found),
               f"25: the one chain's branches without a robot were not reported: {found}")
    with _transform(SELECTION_PATH,
                    lambda t: split(t, tank_define="SHULIB_ROBOT_TANK_2026_BROKEN")):
        b = run_build("tank")
        ok, problems = verdict("tank", b)
        expect(not ok, "25: a split chain with a broken robot #if PASSED the tank build")
        expect(any("carries the robot beacon 'shulib-robot=bench-bot'" in p for p in problems),
               f"25: the bench bot's robot beacon in a tank build was not caught: {problems}")
        expect(not any("wearing a" in p or "'shulib-robot-variant=" in p for p in problems),
               f"25: the variant beacon fired — the split left the program's chain right: "
               f"{problems}")
    expect(one_chain_problems() == [] and chain_table_problems() == [],
           "25: a plant was not restored")

    # ── the review of G1 Part 2c's final run ─────────────────────────────────────────────────
    # 26: A ROBOT'S TABLES BY NAME (step 0; header). Every robot's tables are defined in every build,
    #     so a program could name ANOTHER robot's by a one-token edit, and nothing in the package
    #     would notice. The per-robot names are DERIVED from the table files; each one named in a
    #     program is reported; the review's own mutations (the bench tester's IMU from robot two's
    #     table; shulib Drive's ports from the bench bot's; Calibrate dropping the bench bot's
    #     mechanisms, i.e. none — its claws become drive candidates) are reported; a library header
    #     is read too. NOT reported: the tables themselves, test/, a static_assert's argument (T3
    #     Part 2's checks on robot two's claws), a comment, a string. A table added for a new robot
    #     is covered the moment it exists, and a table file whose selection function step 0 cannot
    #     find FAILS (a rule over an empty set would check nothing). Source-only; plants restored.
    per_robot, derive_problems = per_robot_table_names()
    expect(not derive_problems, f"26: the real tables' per-robot names were not derived: "
                                f"{derive_problems}")
    expect({"kBenchBotChassis", "kRobotTwoChassis", "kNoChassis", "chassisTableFor",
            "kRobotTwoMechanisms", "kNoMechanisms", "mechanismTableFor"} <= per_robot,
           f"26: the derived per-robot names miss one: {sorted(per_robot)}")
    by_name = "a per-robot table"
    for name in sorted(per_robot):
        with _plant(SRC_PLANT, f"\nstatic const void* const kSelfTestRobotName = &{name};\n"):
            found = one_chain_problems()
            expect(any("bench_r3a.cpp" in p and f"names {name}, {by_name}" in p for p in found),
                   f"26: {name} named in a program was not reported: {found}")
    for where, old, new in (
            (SRC_PLANT, "ProsImu imu{kChassis.imuPort", "ProsImu imu{kRobotTwoChassis.imuPort"),
            (DRIVE_PLANT, "tableSignedPort(kChassis, p)", "tableSignedPort(kBenchBotChassis, p)"),
            (CALIBRATE_PLANT, "dropDeclaredMechanismPorts(kMechanisms,",
             "dropDeclaredMechanismPorts(mechanismTableFor(RobotId::BenchBot),")):
        rel = os.path.relpath(where, REPO)
        with _replace(where, old, new):
            found = one_chain_problems()
            expect(any(rel in p and by_name in p for p in found),
                   f"26: `{new}` in {rel} was not reported: {found}")
    with _plant(LIBRARY_PLANT, "\ninline constexpr int kSelfTestPort = kRobotTwoChassis.imuPort;\n"):
        found = one_chain_problems()
        expect(any(os.path.relpath(LIBRARY_PLANT, REPO) in p and by_name in p for p in found),
               f"26: a per-robot table named in a library header was not reported: {found}")
    # the legal places: T3 Part 2's static_assert form (multi-line), a comment, a string, test/
    legal = ("\nstatic_assert(mechanismButtonsAvoid(kRobotTwoMechanisms, kTeleopRezeroButton)\n"
             "                  && &chassisTableFor(RobotId::RobotTwo) == &kRobotTwoChassis,\n"
             "              \"self-test: a static_assert may name robot two's tables\");\n"
             "// kBenchBotChassis, named in a comment\n"
             "static const char* const kSelfTestText = \"kNoMechanisms, in a string\";\n")
    with _plant(SRC_PLANT, legal):
        expect(one_chain_problems() == [],
               f"26: a static_assert, a comment or a string was reported: {one_chain_problems()}")
    with _plant(TEST_PLANT, "\nstatic const void* const kSelfTestHost = &kRobotTwoChassis;\n"):
        expect(one_chain_problems() == [],
               f"26: a host test naming a table was reported: {one_chain_problems()}")
    # the exemption is exactly the static_assert's parentheses
    with _plant(SRC_PLANT, "\nstatic_assert(true, \"x\"); static const int kSelfTestAfter = "
                           "kNoChassis.imuPort;\n"):
        found = one_chain_problems()
        expect(any("kNoChassis" in p and by_name in p for p in found),
               f"26: a name AFTER a static_assert on the same line was not reported: {found}")
    # a table added for a new robot is covered the moment it exists (derived, not listed)
    with _plant(CHASSIS_TABLE_PATH, "\nnamespace shulib::bench {\n"
                                    "inline constexpr ChassisTable kSelfTestRobotChassis = {};\n}\n"):
        with _plant(SRC_PLANT, "\nstatic const void* const kSelfTestNew = &kSelfTestRobotChassis;\n"):
            found = one_chain_problems()
            expect(any("names kSelfTestRobotChassis" in p for p in found),
                   f"26: a NEW robot's table named in a program was not reported: {found}")
    # ...in any spelling the selection function returns, not only the one the object pattern reads
    with _replace(CHASSIS_TABLE_PATH, "            return kNoChassis;",
                  "            return kSelfTestAutoChassis;"):
        with _plant(SRC_PLANT, "\nstatic const void* const kSelfTestAuto = &kSelfTestAutoChassis;\n"):
            found = one_chain_problems()
            expect(any("names kSelfTestAutoChassis" in p for p in found),
                   f"26: a table the selection function returns, named in a program, was not "
                   f"reported: {found}")
    # fail-closed: a selection function step 0 cannot recognise is a FAILURE, not an empty rule
    with _replace(CHASSIS_TABLE_PATH,
                  "constexpr const ChassisTable& chassisTableFor(RobotId id) {",
                  "constexpr auto chassisTableFor(RobotId id) -> const ChassisTable& {"):
        found = one_chain_problems()
        expect(any("chassis_table.hpp: step 0 found 0 selection function" in p for p in found),
               f"26: an unrecognised selection function did not FAIL step 0: {found}")
    expect(one_chain_problems() == [], "26: a plant was not restored")

    # 27: THE REVIEW'S STEP-0 GAPS, closed (step 0; header): (a) every extension make compiles is
    #     read — a switch in a `.c++` or a `.S` file is reported; (b) a selection name is never a
    #     macro — `#define kRobotId RobotId::BenchBot` after a table's include (it would give every
    #     tank build the bench bot's table and no claws under robot two's robot beacon), an
    #     `#undef kChassis` in a program, a `#define` of a per-robot name, a `#define kProgram` in
    #     the header itself: each reported; (c) a chain constant declared in the brace and the
    #     parenthesis forms is reported; (d) a switch given to one host test by the host build's
    #     CMake is reported, and one in a CMake comment is not. Source-only; plants restored.
    with _newfile(os.path.join(REPO, "src", "self_test_probe.c++"),
                  "#ifdef SHULIB_ROBOT_TANK_2026\nint self_test_probe;\n#endif\n"):
        found = one_chain_problems()
        expect(any("self_test_probe.c++" in p and only_header in p for p in found),
               f"27: a switch read in a .c++ file (make compiles it) was not reported: {found}")
    with _newfile(os.path.join(REPO, "src", "self_test_probe.S"),
                  "#if defined(SHULIB_PROGRAM_LIBRARY)\n#endif\n"):
        found = one_chain_problems()
        expect(any("self_test_probe.S" in p and only_header in p for p in found),
               f"27: a switch read in a .S file (make assembles it) was not reported: {found}")
    include_line = '#include "build_selection.hpp"\n'
    for table in (CHASSIS_TABLE_PATH, MECHANISM_TABLE_PATH):
        rel = os.path.relpath(table, REPO)
        with _replace(table, include_line, include_line + "#define kRobotId RobotId::BenchBot\n"):
            found = one_chain_problems()
            expect(any(rel in p and "makes kRobotId a macro" in p for p in found),
                   f"27: `#define kRobotId` in {rel} was not reported: {found}")
    for where, line, name in (
            (SRC_PLANT, "#undef kChassis", "kChassis"),
            (MAIN_CPP_PATH, "#define kRobotTwoChassis kBenchBotChassis", "kRobotTwoChassis"),
            (SELECTION_PATH, "#define kProgram Program::Drive", "kProgram"),
            (TEST_PLANT, "#define kMechanisms kNoMechanisms", "kMechanisms")):
        rel = os.path.relpath(where, REPO)
        with _plant(where, f"\n{line}\n"):
            found = one_chain_problems()
            expect(any(rel in p and f"makes {name} a macro" in p for p in found),
                   f"27: `{line}` in {rel} was not reported: {found}")
    for form in ("inline constexpr RobotId kRobotId{RobotId::BenchBot};",
                 "inline constexpr RobotId kRobotId(RobotId::BenchBot);",
                 "inline constexpr Program kProgram{Program::Drive};"):
        with _plant(CHASSIS_TABLE_PATH, f"\nnamespace shulib::bench::self_test {{\n{form}\n}}\n"):
            found = one_chain_problems()
            expect(any("chassis_table.hpp" in p and "declares one of THE ONE CHAIN's constants" in p
                       for p in found),
                   f"27: `{form}` in a table was not reported: {found}")
        with _plant(SELECTION_PATH, f"\nnamespace shulib::bench::self_test {{\n{form}\n}}\n"):
            found = one_chain_problems()
            expect(any(SELECTION_HEADER in p and "declares one of the chain's constants" in p
                       for p in found),
                   f"27: `{form}` in the header outside the chain was not reported: {found}")
    rel = os.path.relpath(CMAKE_PLANT, REPO)
    for line in ("set_source_files_properties(chassis_table_test.cpp PROPERTIES "
                 "COMPILE_DEFINITIONS SHULIB_ROBOT_TANK_2026)",
                 # a quoted '#' starts no comment: the switch after it is still code
                 "target_compile_definitions(shulib_tests PRIVATE \"SELF_TEST=#\" "
                 "SHULIB_PROGRAM_LIBRARY)"):
        with _plant(CMAKE_PLANT, f"\n{line}\n"):
            found = one_chain_problems()
            expect(any(rel in p and only_header in p for p in found),
                   f"27: `{line}` in the host build's CMake was not reported: {found}")
    with _plant(CMAKE_PLANT, "\n# SHULIB_ROBOT_TANK_2026, named in a comment\n"
                             "#[[ SHULIB_PROGRAM_DRIVE,\n in a bracket comment ]]\n"):
        expect(one_chain_problems() == [],
               f"27: a switch in a CMake comment was reported: {one_chain_problems()}")
    expect(one_chain_problems() == [], "27: a plant was not restored")

    # ── SRC1 (2026-09-27): src/ organized into robots/, team/, programs/ and common/ ─────────────
    # 28: STEP 0 READS EVERY FOLDER, AND THE INCLUDE ROOT HOLDS (header). (a) The one-chain rule
    #     walks src/'s subfolders: a build switch in src/programs/ (SRC_PLANT), in src/common/ and in
    #     a new file in src/team/ is reported, and so is a per-robot table named in src/common/ and in
    #     src/team/ (src/robots/'s tables are cases 19, 26 and 27). (b) The include root: the real tree
    #     reports nothing; a header planted where a second place would answer a src/ path — under
    #     include/ (searched BEFORE src/ by the robot build), test/pros_shim/, test/vendor/ or test/
    #     (the host build), or nested in a src/ subfolder (found beside its includer first) — is
    #     reported, and so is a src/ header that would shadow the library's own (src/shulib/…), the
    #     SDK's (src/pros/…) or the vendored test framework's (src/doctest.h); an include written the
    #     old sibling way (it BUILDS — the reason this is a check and not left to the compiler), one
    #     that climbs with `..`, one that starts `./` (it builds too), and a host test reaching
    #     into src/ by `../src/` are each reported, and the old form inside a comment is not.
    #     src/team/README.md is not a source: neither step 0 nor step 1 counts it. Source-only;
    #     every plant restored.
    for where, form in ((SRC_PLANT, "#ifdef SHULIB_ROBOT_TANK_2026"),
                        (COMMON_PLANT, "#if defined(SHULIB_PROGRAM_LIBRARY)")):
        rel = os.path.relpath(where, REPO)
        with _plant(where, f"\n{form}\nstatic int self_test_folder_switch;\n#endif\n"):
            found = one_chain_problems()
            expect(any(rel in p and only_header in p for p in found),
                   f"28: a switch in {rel} was not reported: {found}")
    team_probe = os.path.join(TEAM_DIR, "self_test_probe.hpp")
    rel_team = os.path.relpath(team_probe, REPO)
    with _newfile(team_probe, "#pragma once\n#ifdef SHULIB_ROBOT_TANK_2026\nint self_test_team;\n"
                              "#endif\n"):
        found = one_chain_problems()
        expect(any(rel_team in p and only_header in p for p in found),
               f"28: a switch in a src/team/ file was not reported: {found}")
    with _newfile(team_probe, "#pragma once\n"
                              "static const void* const kSelfTestTeam = &kRobotTwoChassis;\n"):
        found = one_chain_problems()
        expect(any(rel_team in p and f"names kRobotTwoChassis, {by_name}" in p for p in found),
               f"28: a per-robot table named in src/team/ was not reported: {found}")
    with _plant(COMMON_PLANT, "\nstatic const void* const kSelfTestCommon = &kBenchBotChassis;\n"):
        found = one_chain_problems()
        expect(any(os.path.relpath(COMMON_PLANT, REPO) in p
                   and f"names kBenchBotChassis, {by_name}" in p for p in found),
               f"28: a per-robot table named in src/common/ was not reported: {found}")
    expect(include_root_problems() == [],
           f"28: the real tree's include root is not clean: {include_root_problems()}")
    for probe, why in (
            (("include", "robots", "chassis_table.hpp"),
             "include/robots/chassis_table.hpp and src/robots/chassis_table.hpp are the same path"),
            (("test", "pros_shim", "common", "boot_source.hpp"),
             "test/pros_shim/common/boot_source.hpp and src/common/boot_source.hpp are the same"),
            (("test", "robots", "xdrive_config.hpp"),
             "test/robots/xdrive_config.hpp and src/robots/xdrive_config.hpp are the same path"),
            (("test", "build_selection.hpp"),
             "test/build_selection.hpp and src/build_selection.hpp are the same path"),
            (("src", "programs", "robots", "chassis_table.hpp"),
             "src/programs/robots/chassis_table.hpp shadows src/robots/chassis_table.hpp"),
            (("src", "robots", "build_selection.hpp"),
             "src/robots/build_selection.hpp shadows src/build_selection.hpp"),
            (("src", "shulib", "core", "check.hpp"),
             "include/shulib/core/check.hpp and src/shulib/core/check.hpp are the same path"),
            (("src", "pros", "misc.hpp"),
             "test/pros_shim/pros/misc.hpp and src/pros/misc.hpp are the same path"),
            # (SRC1's review) test/vendor/, both directions: a src/doctest.h would be read by all
            # 172 host tests (`#include "doctest.h"`; -iquote src is searched before the vendored
            # directory), and a vendored copy of a src/ path would be read instead of src/'s.
            (("src", "doctest.h"),
             "test/vendor/doctest.h and src/doctest.h are the same path"),
            (("test", "vendor", "robots", "chassis_table.hpp"),
             "test/vendor/robots/chassis_table.hpp and src/robots/chassis_table.hpp are the same")):
        with _newfile(os.path.join(REPO, *probe), "#pragma once\n"):
            found = include_root_problems()
            expect(any(why in p for p in found),
                   f"28: {'/'.join(probe)} was not reported ({why}): {found}")
    for where, old, new, why in (
            (MECHANISM_TABLE_PATH, '#include "robots/chassis_table.hpp"\n',
             '#include "chassis_table.hpp"\n',
             'includes "chassis_table.hpp", which resolves beside it in src/robots/'),
            (CLAWS_PLANT, '#include "common/boot_source.hpp"\n',
             '#include "../common/boot_source.hpp"\n',
             'includes "../common/boot_source.hpp" by a relative path'),
            # (SRC1's review) the `./` form: it BUILDS (not beside its includer, so the -iquote src
            # root answers it), and it is not written from src/.
            (CLAWS_PLANT, '#include "common/boot_source.hpp"\n',
             '#include "./common/boot_source.hpp"\n',
             'includes "./common/boot_source.hpp" by a relative path'),
            (HOST_INCLUDE_PLANT, '#include "robots/chassis_table.hpp"\n',
             '#include "../src/robots/chassis_table.hpp"\n',
             'written from src/, `#include "robots/chassis_table.hpp"`')):
        with _replace(where, old, new):
            found = include_root_problems()
            expect(any(os.path.relpath(where, REPO) in p and why in p for p in found),
                   f"28: `{new.strip()}` in {os.path.relpath(where, REPO)} was not reported: {found}")
    with _plant(MECHANISM_TABLE_PATH, '\n// #include "chassis_table.hpp" -- the old form, in a '
                                      'comment\n'):
        expect(include_root_problems() == [],
               f"28: an include inside a comment was reported: {include_root_problems()}")
    expect(os.path.isfile(TEAM_README), f"28: {TEAM_README} is missing — the README case below "
                                        f"would prove nothing")
    expect(not any(f.endswith(".md") for f in _src_tree(REPO)[0])
           and not any(t.endswith(".md") for t in expected_tus()),
           "28: src/team/README.md was counted as a source")
    expect(include_root_problems() == [] and one_chain_problems() == [],
           "28: a plant was not restored")

    # 29: an include left in the OLD form in src/programs/ (`#include "chassis_table.hpp"`) -> the
    #     BUILD fails, naming the file: nothing answers that spelling from a subfolder — not the
    #     program's own folder, not include/, not src/ — so a half-done move can never compile
    #     against a stale copy. (Step 0 does not report it: the include resolves nowhere, which is
    #     the compiler's to say; case 28 is the form that DOES build.)
    with _replace(DRIVE_PLANT, '#include "robots/chassis_table.hpp"\n',
                  '#include "chassis_table.hpp"\n'):
        b = run_build("bench")
        ok, problems = verdict("bench", b)
        expect(not ok, "29: an old-form include in src/programs/ BUILT")
        expect(any("drive_program.cpp" in p and "chassis_table.hpp" in p for p in problems),
               f"29: the failure did not name the file and the include: {problems}")

    # 30: THE PATHS A PROGRAM PRINTS (SRC1's review, 2026-09-27; header). The real tree reports
    #     nothing — its printed src/ paths all exist, some followed by a sentence's full stop,
    #     which is not part of the path. A path that names nothing is reported: in a program's
    #     refusal (the review's surviving mutation, `src/robot/` for `src/robots/` in shulib Drive's
    #     fix line), split across two adjacent literals (the compiler joins them, so the check
    #     does), and in a library header; the same dead path in a comment is not (a comment
    #     reaches no robot). Source-only; every plant restored.
    expect(printed_path_problems() == [],
           f"30: the real tree's printed paths do not all exist: {printed_path_problems()}")
    rel = os.path.relpath(DRIVE_PLANT, REPO)
    with _replace(DRIVE_PLANT, '"fill src/robots/chassis_table.hpp, rebuild, upload"',
                  '"fill src/robot/chassis_table.hpp, rebuild, upload"'):
        found = printed_path_problems()
        expect(any(rel in p and 'prints "src/robot/chassis_table.hpp"' in p for p in found),
               f"30: a dead path in {rel}'s refusal was not reported: {found}")
    rel = os.path.relpath(COMMON_PLANT, REPO)
    with _plant(COMMON_PLANT, '\ninline constexpr const char* kSelfTestSplit = "fill src/robots/"\n'
                              '    "chasis_table.hpp, rebuild, upload";\n'):
        found = printed_path_problems()
        expect(any(rel in p and 'prints "src/robots/chasis_table.hpp"' in p for p in found),
               f"30: a dead path split across two literals in {rel} was not reported: {found}")
    rel = os.path.relpath(LIBRARY_PLANT, REPO)
    with _plant(LIBRARY_PLANT, '\ninline constexpr const char* kSelfTestLibrary = '
                               '"see src/common/no_such_file.hpp.";\n'):
        found = printed_path_problems()
        expect(any(rel in p and 'prints "src/common/no_such_file.hpp"' in p for p in found),
               f"30: a dead path in a library header ({rel}) was not reported: {found}")
    with _plant(DRIVE_PLANT, '\n// "fill src/robot/chassis_table.hpp" -- a dead path in a comment\n'):
        expect(printed_path_problems() == [],
               f"30: a path inside a comment was reported: {printed_path_problems()}")
    expect(printed_path_problems() == [], "30: a plant was not restored")

    # ── COMP-t (2026-09-28): the seventh build, "shulib Auton Tester" ─────────────────────────
    # 31: the Makefile's PROGRAM=autontest append DROPPED (`make ROBOT=tank PROGRAM=autontest`
    #     silently builds the tester) -> FAIL on the token-exact structural check naming the define
    #     AND on the beacon (a tank build wearing the tank-autontest label) — cases 14's shape. The
    #     program that runs an auton from RUN must never be an upload that is silently Bench Tests
    #     (a helper told "touch RUN and step back" would stand in front of a robot that does
    #     nothing, and learn the wrong lesson about the next one).
    autontest_line = "override EXTRA_CXXFLAGS+=-DSHULIB_PROGRAM_AUTONTEST\n"
    with _replace(MAKEFILE, autontest_line, "# (self-test plant: autontest append dropped)\n"):
        b = run_build("tank-autontest")
        ok, problems = verdict("tank-autontest", b)
        expect(not ok, "31: PROGRAM=autontest with its Makefile append dropped PASSED — the "
                       "silent-no-op class is back for the auton tester")
        expect(any("whole token -DSHULIB_PROGRAM_AUTONTEST" in p for p in problems),
               "31: the failure did not name the missing PROGRAM=autontest define")
        expect(any("beacon" in p and "NOT in" in p for p in problems),
               "31: the tank-autontest beacon's absence was not caught")
        expect(any("tank build wearing a tank-autontest label" in p for p in problems),
               "31: the tank beacon's presence (NUL-terminated match) was not caught")

    # 32: the real tank-autontest build, alone -> PASS (the `…=tank` prefix inside
    #     `…=tank-autontest` must not false-fail, and the tester's program must link — its entry
    #     linked, no other program's).
    b = run_build("tank-autontest")
    ok, problems = verdict("tank-autontest", b)
    expect(ok, f"32: the real tank-autontest build FAILED the gate: {problems}")

    # 33 (COMP-t's review, 2026-09-29): a build dropped from VARIANTS ALONE — its make arguments,
    #     defines, beacon, robot and program still in the tables, its branch still in the chain.
    #     The build loop simply never ran it, and `check` said "PASS (all 6 builds link)" (measured
    #     by the review). Step 0 must FAIL it from the tables AND from the chain; the real tables
    #     report nothing. Source-only (no build).
    expect(chain_table_problems() == [],
           f"33: the real tables report a problem: {chain_table_problems()}")
    found = _chain_table_problems_without("tank-autontest")
    expect(any("VARIANT_BEACONS describes builds" in p and "VARIANTS" in p for p in found),
           f"33: a build dropped from VARIANTS alone was not reported from the tables: {found}")
    expect(any("'shulib-robot-variant=tank-autontest' is no build's in this gate's VARIANTS" in p
               for p in found),
           f"33: its branch, still in the chain, was not reported: {found}")
    expect(chain_table_problems() == [], "33: VARIANTS was not restored")

    for f in failures:
        print(f"  SELF-TEST FAILURE — {f}", file=sys.stderr)
    if failures:
        return 1
    print(f"src build gate self-test: OK ({checks} detector cases, each against a "
          f"real build)")
    return 0


def main():
    arg = sys.argv[1] if len(sys.argv) > 1 else "check"
    if arg == "check":
        return cmd_check()
    if arg == "self-test":
        return do_self_test()
    print(__doc__.split("USAGE")[0], file=sys.stderr)
    print(f"unknown subcommand '{arg}' — use: check | self-test", file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main())
