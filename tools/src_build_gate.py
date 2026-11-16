#!/usr/bin/env python3
"""src build gate — src/ compiles AND links for the V5, every robot variant (GATE1; third
variant at R3b Session 2; a second axis, the PROGRAM, at R3b Part 0b; its third value, the
LIBRARY program, at R3b Parts 1–3 — RETIRED at T4 Part 3; its fourth, the CALIBRATE program, at
R3d Part 2; its fifth, the AUTON TESTER, at COMP-t; its sixth, the COMPETITION program, at COMP-b; its
seventh, "shulib TUNE", at TUNE1-B1 — the first ROBOT-GENERIC row).

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
  tank-drive  ROBOT=tank PROGRAM=drive   "shulib Drive" (T4 Part 3; the R3b Part 0b program of
                                         that name REPLACED in place): robot two's driver file
                                         (src/team/robot_two_driver.hpp) on robot::Robot over the
                                         PROS device provider, built from the SD card's
                                         calibration record (src/programs/drive_program.cpp); slot 1
                                                              -DSHULIB_ROBOT_TANK_2026
                                                              -DSHULIB_PROGRAM_DRIVE
  tank-calibrate ROBOT=tank PROGRAM=calibrate "shulib Calibrate" (R3d Part 2; since CAL1 Part B the
                                         library's standard Calibrate, hal/pros/calibrate.hpp, over
                                         src/robots/calibrate_facts.hpp, glued by
                                         src/programs/calibrate_program.cpp): the robot MEASURES its
                                         own drivetrain and writes /usd/shulib_drivetrain.bin (a
                                         verified backup, .bak, first); slot 4
                                                              -DSHULIB_ROBOT_TANK_2026
                                                              -DSHULIB_PROGRAM_CALIBRATE
  tank-autontest ROBOT=tank PROGRAM=autontest "shulib Auton Tester" (COMP-t): runs ONE of the
                                         team's autons (src/team/autons.cpp) from RUN on the
                                         brain's screen through competition::runAuton
                                         (src/programs/auton_tester_program.cpp); slot 2
                                                              -DSHULIB_ROBOT_TANK_2026
                                                              -DSHULIB_PROGRAM_AUTONTEST
  tank-competition ROBOT=tank PROGRAM=competition "shulib Competition" (COMP-b): THE
                                         COMPETITION PROGRAM — the chosen auton in autonomous()
                                         on the program's own task, then the team's driver file
                                         (src/programs/competition_program.cpp); slot 6
                                                              -DSHULIB_ROBOT_TANK_2026
                                                              -DSHULIB_PROGRAM_COMPETITION
  tank-tune   ROBOT=tank PROGRAM=tune    "shulib Tune" (TUNE1-B1): LIVE TUNING of the robot's
                                         MECHANISMS on the brain screen — the library's program
                                         (include/shulib/mechanism/tune_program.hpp) over the build's
                                         mechanism table (src/programs/tune_program.cpp); slot 7. It
                                         makes ONLY mechanisms, never the drive
                                                              -DSHULIB_ROBOT_TANK_2026
                                                              -DSHULIB_PROGRAM_TUNE
  one, one-drive, one-calibrate, one-autontest, one-competition, one-tune — ROBOT=one [PROGRAM=…]
                                         (chunk ROB1-A, 2026-11-11; one-tune since its integration
                                         onto TUNE1-B1, 2026-11-13): ROBOT ONE, the 2026 cascade
                                         robot — the same six programs in the same slots, over its
                                         own tables (src/robots/chassis_table.hpp, mechanism_table.hpp
                                         from src/robots/robot_one_facts.hpp), driver file and
                                         autons (src/team/robot_one_*), chosen by
                                         src/common/this_robot.hpp
                                                              -DSHULIB_ROBOT_ONE_2026
                                                              [-DSHULIB_PROGRAM_…]
                                         While a fact its program needs is OWED, each of these
                                         FAILS by design — ROBOT ONE'S FACTS PROOF (below) is its
                                         verdict, not the rest of `check`'s.
  (PROGRAM=drive, =calibrate, =autontest or =competition with any other ROBOT is a Makefile
   $(error): only robots two and one have a driver file and a calibration record, only a tank can
   be calibrated by this procedure, and each robot's autons and driver file are its own.
   PROGRAM=tune is the first ROBOT-GENERIC row (the MECH-S/TUNE1 brief's re-check #10): accepted for
   ANY robot in the Makefile's MECHANISM_ROBOTS — the robots whose mechanism table is not empty, robots
   two and one — and an $(error) naming why for any other. Its row names no robot; step 0's [tune-row]
   check holds MECHANISM_ROBOTS to the table file and requires a tune build here per robot in it.)
  RETIRED (T4 Part 3, 2026-10-01): tank-library, ROBOT=tank PROGRAM=library, "shulib Teleop" —
   replaced by "shulib Drive" (the team lead's ruling R5). `make ROBOT=tank PROGRAM=library` is a
   Makefile $(error) naming the retirement, never a different program under the old name; and
   RETIRED (below) keeps its switch, its define and its beacon out of the tree for good: the
   switch named in any code under src/, include/shulib/ or test/ — src/build_selection.hpp
   included — fails step 0, its define on a compile line fails step 5, and its beacon in a
   package fails step 6. Self-test cases 12-13 prove both halves (they stand where the library
   build's own cases did; those retired onto 10-11, which test the same things for tank-drive).

WHAT `check` DOES, per build (bench, xdrive, tank, tank-drive, tank-calibrate, tank-autontest,
tank-competition, tank-tune; robot one's six as ROBOT ONE'S FACTS PROOF says)
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
    `shulib-robot-variant=bench|xdrive|tank|tank-drive|tank-calibrate|tank-autontest|`
    `tank-competition|tank-tune|one|one-drive|one-calibrate|one-autontest|one-competition|one-tune`
    (robot one's six since ROB1-A and its integration onto TUNE1-B1) —
    in THE SAME BRANCH of THE ONE #if chain that sets the program (`kProgram`) and the robot
    (`kRobotId`) — one chain since G1 Part 2c, in that header since 2c's final run, and step 0
    below enforces it — and the program prints it at boot so it is linked in. The expected
    beacon must be in the bytes of bin/hot.package.elf and the other thirteen — the RETIRED
    `…=tank-library` too — must NOT be. MATCHED
    WITH THE NUL TERMINATOR (Part 0b): `…=tank` is a prefix of `…=tank-drive`,
    `…=tank-calibrate`, `…=tank-autontest`, `…=tank-competition` and `…=tank-tune`, and `…=one` of
    robot one's five `…=one-<program>`, so a plain substring search
    would false-fail every one of those; a
    C string literal sits in the ELF with its terminating zero byte, and that is what is
    searched for. This is the end-to-end proof that the `#if` in the SOURCE took the branch the
    Makefile asked for — the one case no command-line check can see is a macro renamed or
    broken in the chain, and a "shulib Drive" upload that was silently the tester is HA-111's
    defect class again.
    THE ROBOT BEACON (G1 Part 2c's final run, the team lead's ruling "one chain picks both"):
    one string per ROBOT — `shulib-robot=bench-bot|robot-two|invented-xdrive|robot-one` — DERIVED from
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
    point's case calls: `kProgram = Program::BenchTests` under the tank-drive beacon, or
    opcontrol()'s BenchTests and Drive calls swapped, pass steps 0-7 untouched (measured), and
    either one is an upload whose name is not its program — a "shulib Drive" slot that is the
    tester, a "Bench Tests" slot a helper was told is read-only driving the motors. So this step
    reads the linked ELF's symbol table (`arm-none-eabi-nm -C --defined-only`) and requires,
    for the build's program (BUILD_PROGRAM), every shulib::bench:: name src/main.cpp uses for
    it (PROGRAM_ENTRY_SYMBOLS) to be LINKED, and every other program's to be ABSENT. It holds
    because a switch on a `constexpr` enum folds at -Os, a call into another TU is never
    inlined (no LTO), and --gc-sections drops every function nothing calls — measured for all
    six builds at 2c, for the seventh at COMP-t, the eighth at COMP-b, and for the new tank-drive
    at T4 Part 3. If any of that stops being true, this step
    FAILS (a program's own name missing, or another's present); it cannot pass silently. What
    LIVE TUNING STAYS OUT OF EVERY OTHER PROGRAM (TUNE1-B1; the MECH-S/TUNE1 brief's D7 (a): "a match
    runs exactly the numbers in the robot file"): every build whose program is not Tune must link NO
    live-tuning symbol (LIVE_TUNING_SYMBOL: shulib::mechanism::LiveTuning's and TuneProgram's
    members, and the bytes of its precondition messages, "LiveTuning::…", which survive the -Os inlining
    that can leave no member symbol at all — LIVE_TUNING_BYTES) — the brief names the three match programs, Competition, Drive and the Auton Tester;
    this checks every program but Tune — and a Tune build must link at least one (else the absence
    above would prove nothing) and NO chassis or Robot symbol (NO_CHASSIS_SYMBOL: the Tune program
    never builds the drive). Each match program (Drive, Competition, the Auton Tester) must link a
    chassis symbol, so the no-chassis pattern is seen to match something. Self-test cases 42-49
    (49: every detector of this half on synthetic ELF contents, no build).
    What it does NOT see: the
    sentences src/main.cpp paints itself (the DISABLED / FIELD CONTROL reasons and the tester's
    AUTONOMOUS reason are main.cpp's own literals, no program's symbol) — a swap there shows the
    wrong sentence and passes. (Calibrate's three state screens are its own symbols since CAL1
    Part B — calibrateDisabled, calibrateConnected, calibrateAutonomous — so their PRESENCE is
    checked; WHICH entry function calls each is not — two swapped between functions keep every
    symbol linked. test/calibrate_facts_test.cpp's F5 pins each Calibrate case inside its own
    function in src/main.cpp's source.)
 9. Asserts ONE-CALL WRITES (SER1, 2026-10-08; A4 register HA-189): no src/ OBJECT prints
    through stdio. Every task on the brain shares ONE stdout FILE, line-buffered, and the
    vendored newlib's per-FILE locks are no-op stubs — so two tasks inside printf/puts/fwrite on
    stdout at once can mix or lose each other's bytes (HA-189, reasoned from the vendored
    libc.a/libpros.a). A line is whole only as ONE write() call, which PROS's serial write mutex
    keeps whole (reasoned, HA-189): include/shulib/hal/pros/char_sink.hpp's
    serialLine()/serialWrapped() and its ProsCharSink. So this step runs `arm-none-eabi-nm -A -u`
    over every object make built (bin/**/*.o — STDIO_EXEMPT_OBJECT, PROS's generated link
    timestamp, aside) and FAILS, naming the object and the symbol, on any of STDIO_BANNED:
      * every printer that writes stdout or stderr WITHOUT naming it — its object references only
        the printer's own name, never stdout: printf, iprintf, vprintf, viprintf, puts, putchar,
        putchar_unlocked, wprintf, vwprintf, putwchar, putwchar_unlocked, perror, psignal, and
        each reentrant `_r` form. That is every such writer the vendored libc.a DEFINES: the
        self-test derives the family from the archive's own symbol table (STDIO_IMPLICIT_WRITER)
        and fails on one the list lacks (the review of 2026-10-15 found viprintf,
        putchar_unlocked and the wide printers PASSING the first list, A1's);
      * the C++ standard streams, narrow and wide: std::cout, cerr, clog, wcout, wcerr, wclog;
      * the FILE-taking printers A1 bans outright: putc, fputc, __swbuf_r, fputs, fprintf,
        vfprintf, fiprintf;
      * and stdio's FILEs themselves: _impure_ptr — what NAMING stdin, stdout or stderr compiles
        to in newlib (`fflush(stdout)`, `fwrite(…, stdout)`, `fputws(…, stdout)`) — and __sf, the
        three FILEs' own array (`&__sf[1]` is stdout by another name). This is how the check sees
        any other stdio call aimed at stdout without banning the call itself.
    ALLOWED: fwrite, fflush, fopen and snprintf/vsnprintf — the SD card's FILEs
    (hal/pros/block_sink.hpp; the bench tester's card probe) and formatting into a buffer — and
    write() itself. It reads the OBJECTS, not the source, because the compiler rewrites
    `printf("x\n")` into `puts("x")` invisibly — and newlib's puts sends the text and the
    newline as TWO writes; and the objects before the link, because --gc-sections would drop an
    unused printer from the package and hide it. Every src/ TU must have its object (a check
    that read nothing would pass). And once per `check`, before any build, the LIST itself is
    checked against the libraries it names ([stdio-list], stdio_list_problems()): every
    STDIO_IMPLICIT_WRITER the vendored libc.a defines must be in it, and every name in it must be
    defined by libc.a or the toolchain's libstdc++.a (a misspelt name bans nothing). A newer libc
    with a new printer FAILS the gate until the list names it. What it does NOT see: a write()
    of a line in two calls (`serialLine` then a second write for the newline) — the host suite
    pins the one-call shape (test/serial_line_test.cpp); a FILE a program opens on the serial's
    descriptor itself (`fdopen(1, …)`, then `fwrite`), or a stdout FILE* handed in from code
    outside src/; stdin's readers (getchar, scanf, gets and their wide forms — they read);
    assert()'s failure message (`__assert_func`) and std::terminate's, both on the way to a
    crash; and the vendored libraries' own stdio (PROS's serial daemon banners, the expander
    ADI's `puts`, crash dumps), which are not src/ objects.

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
  read as C++'s first phases leave the text — line splices joined with nothing (`SHULIB_\\` + a
  next line `ROBOT_…` is one name), comments and string, raw-string and character literals
  stripped, numbers whole (`1'000`'s apostrophe opens no literal; MECH-A5's verification) —
  so an `#if`, `#ifdef`, `#ifndef`, `#elif`, `#elifdef`,
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
  in the bench tester, `mechanismTableFor(RobotId::BenchBot)` in Calibrate's facts — by a one-token
  edit that no beacon, symbol or static_assert would notice. So the per-robot names — every
  table the two table files define (`constexpr ChassisTable|MechanismTable <name>`), their
  selection functions (`constexpr const …Table& <name>(RobotId …)`) and every name those
  functions return: DERIVED from TABLE_FILES, never listed here, and a table file whose
  selection function step 0 cannot find FAILS rather than checking nothing — may be named in
  code only in the two table files, in test/ (the host suite reads them by name, and no test TU
  is in a package), and inside a `static_assert(…)`'s parentheses anywhere (compile-time only:
  T3 Part 2's checks on robot two's claws). Anywhere else under src/ or include/shulib/ one
  FAILS. A program reads the BUILD's tables, kChassis and kMechanisms, only.
  ...AND A ROBOT'S MECHANISM NAMESPACE, PER ROBOT (the mechanism core's MECH-A5, 2026-10-29; the MECH
  brief's re-check T3). Since MECH-A5 a robot's mechanisms are DECLARATIONS inside its own namespace
  in the mechanism table file (`namespace robot_two { … inline constexpr MechanismTable
  kRobotTwoMechanisms = mechanisms(…); }`), and a step names a declaration (`r.set(claw["closed"])`),
  so a program could command ANOTHER robot's same-named mechanism — or end ABSENT — by naming its
  namespace, with no compile error (HA-111's defect class: a build carrying another robot's facts).
  So the robot NAMESPACES are DERIVED too (_robot_namespaces: every namespace block of the mechanism
  table file that declares a `MechanismTable`), and the namespace's NAME, as a whole C++ identifier
  wherever the text writes it — `<ns>::`, `using namespace <ns>`, re-opening `namespace …<ns>`, an
  alias `namespace r2 = <ns>;`, a `#define`, split over lines, or split INSIDE the name by a line
  splice — is refused outside the table files, test/, a static_assert — and THAT ROBOT'S OWN TEAM
  FILES (below). THE LIMIT (MECH-A5's verification): the rule reads text, so a name the PREPROCESSOR
  assembles — `##` token pasting, `CAT(robot_, two)` — is never written out and is not seen; nor is
  a file step 0 does not read (ONE_CHAIN_ROOTS), nor — the one place its reading differs from C++'s
  (_splice) — code after a raw string that a line splice INSIDE its closing `)delim"` ends early
  here. THAT ROBOT'S OWN TEAM FILES: a src/team/ file whose name begins `<ns>_` (src/team/robot_two_driver.hpp), or one
  the reviewed TEAM_FILE_ROBOT map gives to a robot (today the team's auton file, src/team/autons.cpp
  and .hpp, robot two's — until G3a Part 4's per-robot auton files). A robot's own team file may also
  name that robot's own LIST (a driver file checks its controls against its robot's list — its job),
  and nothing of another robot's; a CHASSIS table stays refused in a team file. A broader exception
  re-opens the defect class without a single compile error (the brief's landmine 12).
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
  the seventh) without telling this gate which functions are its own.
  A RETIRED SWITCH (T4 Part 3): RETIRED_SWITCHES' names — SHULIB_PROGRAM_LIBRARY, "shulib
  Teleop"'s — may not be named in code ANYWHERE under ONE_CHAIN_ROOTS, src/build_selection.hpp and
  the host build's CMake included (comments and string literals stripped, as above): a retired
  switch read in an #error guard or in the chain would bring the retired build back under its old
  name. The problem names the retirement.
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
  read whole, and a raw string's text is its own (MECH-A5's verification); comments are not read (the staleness audit's class, and a comment reaches no
  robot); test/ is not read (its own tests pin what they print). Self-test case 30.

ROBOT ONE'S FACTS PROOF (chunk ROB1-A, 2026-11-11; the ROB1 brief's D3 (a), [R1], [R4], [R11])
------------------------------------------------------------------------------------------
src/robots/robot_one_facts.hpp writes every robot-one number nobody has measured ONCE, as OWED, with a
placeholder the host suite runs on; a static_assert under each fact fails robot one's build of every
program that needs it (Stage A: all six; Stage B: Drive, the Auton Tester, Competition and — since the
integration onto TUNE1-B1 — Tune), naming it
("robot one: OWED -- <name> (src/robots/robot_one_facts.hpp)"). `check` proves both halves:
  (i)  THE REAL TREE — robot one's six rows: a row whose program needs no owed fact gets the ordinary
       verdict; every other must FAIL with EXACTLY the OWED lines of the owed facts its program needs,
       compared as a SET (one translation unit prints them all; the expected set is read from the
       ledger's own text — ledger_facts()), with no other error and NO bin/hot.package.bin or .elf left
       behind (the Makefile's variant stamp deletes bin/ and .d/ when ROBOT=, PROGRAM= or the defines
       change, so a refused build cannot leave the last variant's package to upload). Every other
       build printing an OWED line fails.
  (ii) A COPY, NEVER THE REPO — the working tree's files (`git ls-files -co --exclude-standard`, so the
       uncommitted change is what is proven) copied to build/src-gate-facts/ (gitignored); THERE ONLY,
       every `owed(p, stage, "name")` becomes `typed(p, p, stage, "name", "gate copy -- same as the
       placeholder, checked")`, and the gate writes the MARKER FILE build/ROBOT-ONE-GATE-COPY-ONLY at
       the copy's root, which turns the ledger's kRefuseGateCopySource off THERE (the ledger reads it by
       `__has_include`, relative to its own directory); robot one's six builds then run there and must
       pass every check the eight builds pass. The copy is deleted in a `finally` and a leftover one
       swept at the start; the repository's ledger hash before = after and its bin/hot.package.bin
       untouched. A ledger copied back cannot link: the switch is NOT in the ledger's text (the review
       of ROB1-A: a text switch the copy flipped came back with the file, and that ledger linked), so
       the real tree — no marker — refuses the "gate copy" source at make, in every build; step 0
       refuses it too, and refuses the marker in the repository's own build/ (self-test case 57).
STEP 0's ROB1-A RULES: a src/ file outside src/team/ and src/common/this_robot.hpp naming a robot's
driver file or autons (RobotTwoDriver, RobotOneDriver, registerAutons, registerRobotOneAutons —
TEAM_SELECTION_NAMES) chooses a robot outside the one chain and fails; a TYPED fact in the repository's
ledger with an empty source or the gate copy's marker fails, and so does the copy's marker FILE in the
repository's build/ (ledger_source_problems()); and THE STAGES ARE CLASSIFIED (stage_table_problems(),
the review of ROB1-A): every value of src/build_selection.hpp's `Program` has exactly one case in the
ledger's needsStageB() (no `default`), that switch agrees with STAGE_PROGRAMS here program by program,
and every robot-one row's BUILD_PROGRAM is classified — so a program added later cannot build robot
one on Stage A alone while this gate expects exactly that (TUNE1-B1's Tune was the first: at the
integration it was classified as needing Stage B — it homes the cascade with the homing push, jogs it
at the unhomed volts and moves it to its first and last level — so one-tune refuses with all its owed
Stage A and B facts). STEP 8's ROBOT
HALF: no robot's package links another robot's autons registration or carries another robot's
driver-file path or beacon (ROBOT_FOREIGN_SYMBOLS / ROBOT_FOREIGN_STRINGS; the brief's T7).
Self-test cases 50 (robot_one named by a program, robot two's team file and the ledger), 51 (each
TEAM_SELECTION_NAMES in a program), 52 (a typed fact sourced "gate copy" or ""), 53 (the real-tree half
seeing a wrong set — the OWED check's needs() dropped), 54 (the copy's proof interrupted after its
ledger is typed; a leftover copy swept), 55 (THE VARIANT STAMP, make run directly: a PROGRAM switch
recompiles every file, the same variant 0, and a refused robot-one build leaves no package); and, from
the review of ROB1-A, 56 (THE STAGES: a program with no case in needsStageB(), a `default`, the switch
disagreeing with STAGE_PROGRAMS, an unclassified robot-one row) and 57 (a LEDGER COPIED BACK from the copy:
`make ROBOT=one PROGRAM=drive` run directly refuses it by name, no package left; the copy's marker file in
the repository's build/ fails step 0). (ROB1-A numbered them 42-49; TUNE1-B1 landed first with its own
42-49, so the integration onto it renumbered ROB1-A's to 50-57, 2026-11-13.) And, from the review of that
integration (2026-11-13: deleting step 8's robot-half call survived `check` AND the whole self-test), 58
(STEP 8's ROBOT HALF, _self_test_robot_half(): every detector of robot_separation_problems() on synthetic ELF
contents, every robot's ROBOT_FOREIGN_* rows non-empty, and ONE real build through verdict() — robot two's
Auton Tester linking robot one's autons registration — failing step 8 by name on BOTH of the call's inputs:
the linked symbol and the ELF's words, robot one's autons file path — so the call site is proven; the
words half since the verification of that review, when passing the call b"" for the ELF survived the case).

THE VARIANT DIFFERENTIAL, RETIRED AT G1 PART 2c — what it was, why it went, what replaced it
------------------------------------------------------------------------------------------
It was the brief's warning-count detector: the bench, tank, tank-drive, tank-library (a build
itself retired at T4 Part 3) and tank-calibrate builds each had to emit >= 1 -Wunused-function from src/main.cpp (the invented
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
the seventh build's define and beacon, cases 31-32; a build dropped from VARIANTS alone, case 33;
the eighth build's define and beacon, cases 34-35; the retired library build, cases 12-13 since
T4 Part 3; step 9's stdio printers, cases 36-39 since SER1 and 40-41 since its review; the Tune build,
its robot-generic row and step 8's live-tuning half, cases 42-49 since TUNE1-B1; robot one's builds,
its facts proof, its step-0 rules and the variant stamp, cases 50-57 since ROB1-A; step 8's robot half,
case 58 since the review of ROB1-A's integration);
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

import contextlib
import hashlib
import io
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
VARIANTS = ("bench", "xdrive", "tank", "tank-drive", "tank-calibrate", "tank-autontest",
            "tank-competition", "tank-tune", "one", "one-drive", "one-calibrate", "one-autontest",
            "one-competition", "one-tune")
BUILD_ARGS = {
    "bench": ("ROBOT=bench",),
    "xdrive": ("ROBOT=xdrive",),
    "tank": ("ROBOT=tank",),
    "tank-drive": ("ROBOT=tank", "PROGRAM=drive"),
    "tank-calibrate": ("ROBOT=tank", "PROGRAM=calibrate"),
    "tank-autontest": ("ROBOT=tank", "PROGRAM=autontest"),
    "tank-competition": ("ROBOT=tank", "PROGRAM=competition"),
    "tank-tune": ("ROBOT=tank", "PROGRAM=tune"),
    "one": ("ROBOT=one",),
    "one-drive": ("ROBOT=one", "PROGRAM=drive"),
    "one-calibrate": ("ROBOT=one", "PROGRAM=calibrate"),
    "one-autontest": ("ROBOT=one", "PROGRAM=autontest"),
    "one-competition": ("ROBOT=one", "PROGRAM=competition"),
    "one-tune": ("ROBOT=one", "PROGRAM=tune"),
}
VARIANT_DEFINES = {
    "bench": (),
    "xdrive": ("-DSHULIB_ROBOT_XDRIVE_INVENTED",),
    "tank": ("-DSHULIB_ROBOT_TANK_2026",),
    "tank-drive": ("-DSHULIB_ROBOT_TANK_2026", "-DSHULIB_PROGRAM_DRIVE"),
    "tank-calibrate": ("-DSHULIB_ROBOT_TANK_2026", "-DSHULIB_PROGRAM_CALIBRATE"),
    "tank-autontest": ("-DSHULIB_ROBOT_TANK_2026", "-DSHULIB_PROGRAM_AUTONTEST"),
    "tank-competition": ("-DSHULIB_ROBOT_TANK_2026", "-DSHULIB_PROGRAM_COMPETITION"),
    "tank-tune": ("-DSHULIB_ROBOT_TANK_2026", "-DSHULIB_PROGRAM_TUNE"),
    "one": ("-DSHULIB_ROBOT_ONE_2026",),
    "one-drive": ("-DSHULIB_ROBOT_ONE_2026", "-DSHULIB_PROGRAM_DRIVE"),
    "one-calibrate": ("-DSHULIB_ROBOT_ONE_2026", "-DSHULIB_PROGRAM_CALIBRATE"),
    "one-autontest": ("-DSHULIB_ROBOT_ONE_2026", "-DSHULIB_PROGRAM_AUTONTEST"),
    "one-competition": ("-DSHULIB_ROBOT_ONE_2026", "-DSHULIB_PROGRAM_COMPETITION"),
    "one-tune": ("-DSHULIB_ROBOT_ONE_2026", "-DSHULIB_PROGRAM_TUNE"),
}
ALL_DEFINES = sorted({d for ds in VARIANT_DEFINES.values() for d in ds})
VARIANT_BEACONS = {
    "bench": b"shulib-robot-variant=bench",
    "xdrive": b"shulib-robot-variant=xdrive",
    "tank": b"shulib-robot-variant=tank",
    "tank-drive": b"shulib-robot-variant=tank-drive",
    "tank-calibrate": b"shulib-robot-variant=tank-calibrate",
    "tank-autontest": b"shulib-robot-variant=tank-autontest",
    "tank-competition": b"shulib-robot-variant=tank-competition",
    "tank-tune": b"shulib-robot-variant=tank-tune",
    "one": b"shulib-robot-variant=one",
    "one-drive": b"shulib-robot-variant=one-drive",
    "one-calibrate": b"shulib-robot-variant=one-calibrate",
    "one-autontest": b"shulib-robot-variant=one-autontest",
    "one-competition": b"shulib-robot-variant=one-competition",
    "one-tune": b"shulib-robot-variant=one-tune",
}
# (XDRIVE_WIRING_DEAD_IN — the retired variant differential's table — was removed at G1
#  Part 2c; the header's "the variant differential, retired" says what replaced it and why.)

# THE RETIRED BUILDS (header, "RETIRED"; T4 Part 3, 2026-10-01): a build switch, its define and its
# beacon that no build may carry again, each with the sentence a failure names. Kept so a retired
# build cannot come back silently — by an #if that reads its switch (step 0, anywhere, the selection
# header included), by a Makefile that appends its define (step 5, on every compile line), or by a
# branch that types its beacon (step 6, in every package).
RETIRED_WHY = ('RETIRED at T4 Part 3: "shulib Teleop" (PROGRAM=library) was replaced by "shulib '
               'Drive" (PROGRAM=drive) -- the team lead\'s ruling R5')
RETIRED_SWITCHES = {"SHULIB_PROGRAM_LIBRARY": RETIRED_WHY}
RETIRED_DEFINES = {"-DSHULIB_PROGRAM_LIBRARY": RETIRED_WHY}
RETIRED_BEACONS = {b"shulib-robot-variant=tank-library": RETIRED_WHY}

# Step 6's robot half (header): the ROBOT each build carries the tables of — the `RobotId`
# enumerator its branch of src/build_selection.hpp's chain sets — and each robot's beacon,
# derived from that enumerator by robotBeacon() (step 0 checks this table against its cases).
BUILD_ROBOT = {
    "bench": "BenchBot",
    "xdrive": "InventedXDrive",
    "tank": "RobotTwo",
    "tank-drive": "RobotTwo",
    "tank-calibrate": "RobotTwo",
    "tank-autontest": "RobotTwo",
    "tank-competition": "RobotTwo",
    "tank-tune": "RobotTwo",
    "one": "RobotOne",
    "one-drive": "RobotOne",
    "one-calibrate": "RobotOne",
    "one-autontest": "RobotOne",
    "one-competition": "RobotOne",
    "one-tune": "RobotOne",
}
ROBOT_BEACONS = {
    "BenchBot": b"shulib-robot=bench-bot",
    "RobotTwo": b"shulib-robot=robot-two",
    "InventedXDrive": b"shulib-robot=invented-xdrive",
    "RobotOne": b"shulib-robot=robot-one",
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
    "tank-calibrate": "Calibrate",
    "tank-autontest": "AutonTester",
    "tank-competition": "Competition",
    "tank-tune": "Tune",
    "one": "BenchTests",
    "one-drive": "Drive",
    "one-calibrate": "Calibrate",
    "one-autontest": "AutonTester",
    "one-competition": "Competition",
    "one-tune": "Tune",
}
PROGRAM_ENTRY_SYMBOLS = {
    "BenchTests": ("shulib::bench::testerInitialize", "shulib::bench::runR3a"),
    "Drive": ("shulib::bench::driveInitialize", "shulib::bench::kDriveAutonomousReason",
              "shulib::bench::driveAutonomous", "shulib::bench::runDrive"),
    "Calibrate": ("shulib::bench::calibrateInitialize", "shulib::bench::calibrateDisabled",
                  "shulib::bench::calibrateConnected", "shulib::bench::calibrateAutonomous",
                  "shulib::bench::runCalibrate"),
    "InventedXDrive": ("shulib::bench::xdriveInitialize", "shulib::bench::xdriveAutonomous",
                       "shulib::bench::runXdrive"),
    "AutonTester": ("shulib::bench::autonTesterInitialize",),
    "Competition": ("shulib::bench::competitionInitialize", "shulib::bench::competitionDisabled",
                    "shulib::bench::competitionConnected", "shulib::bench::competitionAutonomous",
                    "shulib::bench::runCompetition"),
    "Tune": ("shulib::bench::tuneInitialize", "shulib::bench::runTune"),
}
# Step 8's live-tuning half (header; TUNE1-B1): a symbol of live tuning's — the library's LiveTuning
# and its Tune program — matched anywhere in a demangled name (a template member reads
# "void shulib::mechanism::LiveTuning::positionTuningLines<…>"). No program but Tune may link one, and
# a Tune build must link at least one. The members found in the tank-tune ELF when this was written
# (2026-11-10): LiveTuning::apply / undo / reset / commit / whatChanged / the copy block's lines, and
# TuneProgram's step, make, press, render ...
LIVE_TUNING_SYMBOL = re.compile(r"\bshulib::mechanism::(LiveTuning|TuneProgram)::")
# ...and, because -Os can INLINE every LiveTuning member a program calls (then no member symbol is left —
# the self-test's case 44 measured exactly that, 2026-11-11), the BYTES of live tuning's precondition texts
# ("LiveTuning::reset: attach() the service first"): a precondition's message is a string literal that
# survives inlining, and no header but mechanism/live_tuning.hpp writes "LiveTuning::" inside a string.
LIVE_TUNING_BYTES = b"LiveTuning::"
# ...and a Tune build links no chassis and no Robot (it never builds the drive); every match program
# links a chassis symbol (the pattern's positive control).
NO_CHASSIS_SYMBOL = re.compile(r"\bshulib::(chassis::|robot::Robot::)")
TUNE_PROGRAMS = frozenset({"Tune"})
CHASSIS_PROGRAMS = frozenset({"Drive", "Competition", "AutonTester"})
# The shulib::bench:: names src/main.cpp uses that belong to NO single program: the shared
# state screens, and the fault-latch hook main.cpp defines. Anything else main.cpp names must be
# in PROGRAM_ENTRY_SYMBOLS, or step 0 reports it.
SHARED_BENCH_NAMES = {"benchStateScreen", "competitionBits", "attachFaultLatch"}
# ...and the build selection's own names, which src/main.cpp brings in with `using` (the chain's
# constants, the robot beacon and the enum the entry points switch on).
BUILD_SELECTION_NAMES = {"Program", "kProgram", "kRobotBeacon", "kVariantBeacon"}
ARM_NM = "arm-none-eabi-nm"
ENTRY_POINTS = ("initialize", "disabled", "competition_initialize", "autonomous", "opcontrol")

# Step 9, ONE-CALL WRITES (header; SER1, 2026-10-08, the brief's amendment A1; widened by SER1's
# review, 2026-10-15). The undefined symbols a src/ object may NOT reference. fwrite/fflush/fopen
# and snprintf/vsnprintf are deliberately NOT here: the SD card's FILEs need them, and a buffer is
# not the serial.
STDIO_BANNED = frozenset({
    # A1's list, as the team lead's go carried it.
    "puts", "_puts_r", "putchar", "putc", "fputc", "__swbuf_r", "fputs", "printf", "iprintf",
    "vprintf", "fprintf", "vfprintf", "fiprintf", "perror", "_ZSt4cout", "_ZSt4cerr",
    "_ZSt4clog", "_impure_ptr",
    # The review's (2026-10-15): the rest of the printers that write stdout/stderr WITHOUT naming
    # it — every member of STDIO_IMPLICIT_WRITER the vendored libc.a defines (the self-test derives
    # them) — the wide C++ streams (mangled: nm runs without -C), and the FILE array itself.
    "_printf_r", "_iprintf_r", "_vprintf_r", "viprintf", "_viprintf_r", "_putchar_r",
    "putchar_unlocked", "_putchar_unlocked_r", "_perror_r", "psignal",
    "wprintf", "_wprintf_r", "vwprintf", "_vwprintf_r", "putwchar", "_putwchar_r",
    "putwchar_unlocked", "_putwchar_unlocked_r",
    "_ZSt5wcout", "_ZSt5wcerr", "_ZSt5wclog", "__sf",
})
# The family of printers that write stdout or stderr without naming it (the object then references
# only the printer), as a pattern over a library's own symbol table: the self-test reads the
# vendored libc.a with it and requires every match in STDIO_BANNED, so a printer the list lacks
# FAILS the self-test rather than passing every build (viprintf and putchar_unlocked did, until the
# review of 2026-10-15). sprintf/snprintf/dprintf and the FILE-taking forms do not match: the first
# three format into a buffer or write one descriptor, and the last need a FILE — stdout's is caught
# by _impure_ptr or __sf.
STDIO_IMPLICIT_WRITER = re.compile(
    r"^_?(?:v?i?printf|v?wprintf|puts|putchar|putchar_unlocked|putwchar|putwchar_unlocked"
    r"|perror|psignal)(?:_r)?$")
VENDORED_LIBC = os.path.join(REPO, "firmware", "libc.a")
# PROS's generated link timestamp (common.mk compiles it into bin/ beside src/'s objects); not
# from src/, and it prints nothing.
STDIO_EXEMPT_OBJECT = "_pros_ld_timestamp.o"
_NM_UNDEFINED = re.compile(r"^(?P<obj>.+?):\s+U\s+(?P<sym>\S+)$")


# A line break, or a LINE SPLICE: a backslash ending a physical line, blanks after it allowed (GCC
# splices those too, with a warning; C++23 made it the rule).
_LINE_BREAK_OR_SPLICE = re.compile(r"(\\[ \t\v\f\r]*\n|\n)")


def _splice(text):
    """`text` after C++'s translation phase 2, which runs BEFORE comments and literals exist: every line
    splice is deleted, so the next line joins its line with NOTHING between — `robot_\\` then `two` is the
    identifier `robot_two`, and a `//` comment ending in a backslash runs on over the next line. Each
    deleted line break is put back after the joined line ends, so every line keeps its number and a
    joined line is read on the line it starts. (MECH-A5's verification: the joins were a space, so a name
    split by a splice passed every rule.) One place this differs from C++: C++ undoes the splices inside a
    raw string literal's text (`R"(…)"`); this does not — a raw string whose text has a backslash at a
    line's end can end early here, inside its closing `)delim"`."""
    out, owed = [], 0
    for k, piece in enumerate(_LINE_BREAK_OR_SPLICE.split(text)):
        if k % 2 == 0:
            out.append(piece)  # a line's text
        elif piece == "\n":
            out.append("\n" * (1 + owed))  # a line break, and every break the line's splices deleted
            owed = 0
        else:
            owed += 1  # a splice: deleted, its break owed
    out.append("\n" * owed)
    return "".join(out)


def _logical_lines(text):
    """(1-based line number, logical line) for `text`: its line splices joined as C++ joins them
    (_splice — with nothing between), each logical line at the number of the physical line it starts on
    (the lines it joined follow it, empty) — a multi-line #if is one directive, and `robot_\\` + `two`
    one identifier."""
    return list(enumerate(_splice(text).splitlines(), 1))


# One left-to-right pass over SPLICED text (_splice): a number, a raw string literal, a string or
# character literal, a // comment, a /* */ comment, or anything else. Order matters — `src/*.cpp` inside
# a // comment is not a block comment's start. A NUMBER is C++'s pp-number, so a digit separator's `'`
# (`1'000`) never opens a character literal — before MECH-A5's verification it did, and a name between
# two such numbers on one line was read as a literal's text; a RAW string (`R"(…)"`, `u8R"x(…)x"`) ends
# at its own `)delim"`, never at a `"` inside it. The look-behinds keep both at a token's start (`x1`,
# `FOOR"…"` are an identifier and then whatever follows it).
_CXX_TOKENS = re.compile(r"(?P<number>(?<![\w.])\.?\d(?:[eEpP][+-]|'\w|[\w.])*)"
                         r'|(?P<raw>(?<!\w)(?:u8|[uUL])?R"(?P<delim>[^ ()\\\t\v\f\n]{0,16})\((?P<rawtext>.*?)\)(?P=delim)")'
                         r'|"(?:\\.|[^"\\\n])*"|\'(?:\\.|[^\'\\\n])*\'|//[^\n]*|/\*.*?\*/',
                         flags=re.S)


def _strip_comments(text, literals=False):
    """C++ source SPLICED (_splice: C++'s phase 2 comes first) with its // and /* */ comments removed
    (line breaks kept, so line numbers survive). String, raw-string and character literals are kept, or
    emptied to `""` when `literals` — a build switch's name inside a printed string does not READ the
    switch; numbers are kept as they are."""
    def keep(m):
        t = m.group(0)
        if m.group("number") is not None:
            return t
        if t.startswith("//"):
            return ""
        if t.startswith("/*"):
            return "\n" * t.count("\n")
        if m.group("raw") is not None:
            return '""' + "\n" * t.count("\n") if literals else t
        if literals:
            return t[0] * 2
        return t
    return _CXX_TOKENS.sub(keep, _splice(text))


def _read(repo, rel):
    with open(os.path.join(repo, rel), encoding="utf-8", errors="replace") as fh:
        return fh.read()


# ── MECH-A5: a robot's MECHANISM namespace is a per-robot name too (header, "A ROBOT'S TABLES BY NAME") ──
#: Team files that belong to ONE robot although their name does not say so. REVIEWED BY HAND: a new team
#: file that belongs to one robot but does not begin with its namespace goes here (the MECH brief's
#: landmine 12). G3a Part 4's per-robot auton files (autonsFor(RobotId)) replace these two entries.
TEAM_FILE_ROBOT = {
    os.path.join("src", "team", "autons.cpp"): "robot_two",
    os.path.join("src", "team", "autons.hpp"): "robot_two",
}
TEAM_DIR_REL = os.path.join("src", "team")
_NAMESPACE_OPEN = re.compile(r"\bnamespace\s+((?:\w+::)*)(\w+)\s*\{")
_MECHANISM_TABLE_OBJECT = re.compile(r"\bconstexpr\s+(?:[\w:]*::)?MechanismTable\s+(\w+)\s*(?:=|\{)")


def _robot_namespaces(repo):
    """{namespace: set of the per-robot MECHANISM lists declared inside it}, for the mechanism table
    file (TABLE_FILES[1]): every namespace block that declares a `constexpr MechanismTable <name>` —
    its robot's list. DERIVED, never listed (a robot added is covered the moment its namespace and
    list exist); the file's own enclosing namespaces (`shulib`, `bench`) are never a robot's."""
    rel = TABLE_FILES[1]
    if not os.path.exists(os.path.join(repo, rel)):
        return {}
    code = _strip_comments(_read(repo, rel), literals=True)
    out = {}
    for m in _NAMESPACE_OPEN.finditer(code):
        depth, j = 1, m.end()
        while j < len(code) and depth:
            depth += (code[j] == "{") - (code[j] == "}")
            j += 1
        tables = set(_MECHANISM_TABLE_OBJECT.findall(code[m.end():j]))
        if tables and m.group(2) not in ("shulib", "bench"):
            out.setdefault(m.group(2), set()).update(tables)
    return out


def _team_file_robot(rel, namespaces):
    """The robot namespace a src/team/ file belongs to — TEAM_FILE_ROBOT's entry, or the namespace its
    file name begins with (`robot_two_driver.hpp` -> `robot_two`) — or None (a file under src/team/
    that belongs to no robot, and every file outside src/team/)."""
    if not rel.startswith(TEAM_DIR_REL + os.sep):
        return None
    if rel in TEAM_FILE_ROBOT:
        return TEAM_FILE_ROBOT[rel]
    base = os.path.basename(rel)
    for ns in sorted(namespaces, key=len, reverse=True):  # the longest first: robot_two_x before robot_t
        if base.startswith(ns + "_"):
            return ns
    return None


def _robot_namespace_problems(repo, rel, namespaces=None):
    """Step 0 (header, "…AND A ROBOT'S MECHANISM NAMESPACE, PER ROBOT"): one C++ file — a robot's
    mechanism namespace NAMED outside the table files, test/, a static_assert and that robot's own team
    files. The rule is the namespace's NAME as a whole C++ identifier, not a list of spellings: every
    way to reach a namespace spells its name — `<ns>::` (a line break on either side of the `::`
    included), `using namespace …<ns>` (qualified, split over lines), re-opening `namespace …<ns>` (the
    brace on the next line, `inline`, an attribute), an alias `namespace r2 = <ns>;`, a `#define` — and
    an identifier spans no LOGICAL line, so it is matched per logical line: the text read as C++'s
    first phases leave it (_strip_comments — line splices joined with nothing, so `robot_\\` + `two` is
    the name; comments and literals stripped; numbers whole, so `1'000 … 2'000` hides nothing between).
    THE LIMIT: a name the preprocessor ASSEMBLES (`##` token pasting) is never written out, and no text
    rule sees it; and the one place the reading differs from C++'s (_splice: a line splice inside a raw
    string's closing `)delim"` ends it early here). (MECH-A5's review: the first rule listed three spellings per physical line and missed
    four that compile; its verification: the splices were joined with a space, a digit separator opened
    a character literal, a raw string ended at its first quote — each passed a name that compiles.)
    Returns a list of problem strings."""
    if rel in TABLE_FILES or rel.startswith("test" + os.sep):
        return []
    namespaces = _robot_namespaces(repo) if namespaces is None else namespaces
    if not namespaces:
        return []
    mine = _team_file_robot(rel, namespaces)
    problems = []
    code = _blank_static_asserts(_strip_comments(_read(repo, rel), literals=True))
    for n, line in _logical_lines(code):
        for ns in sorted(namespaces):
            if ns == mine:
                continue
            named = re.search(r"\b" + re.escape(ns) + r"\b", line)
            if named:
                problems.append(
                    f"{rel}:{n} names {ns}, a robot's mechanism namespace — only that robot's own team "
                    f"files (src/team/{ns}_*, or this gate's TEAM_FILE_ROBOT), the table files, test/ and a "
                    f"static_assert may; a step against another robot's declaration would run ITS "
                    f"same-named mechanism, or end ABSENT, with no compile error (header, step 0, a robot's "
                    f"mechanism namespace; HA-111's class): `{line.strip()}`")
    return problems


# ── ROB1-A: a robot's DRIVER FILE and AUTONS by name (the per-robot table rule's sibling) ─────────────────
#: The names a robot's team files define for the programs: each robot's driver and autons registration. A
#: program names none of them — it runs src/common/this_robot.hpp's ThisRobotDriver and
#: registerThisRobotsAutons(), chosen by kRobotId — so a build can never run one robot's buttons or autons
#: over another robot's mechanisms. Allowed in: THIS_ROBOT_FILE, src/team/ (the files that define them), test/.
TEAM_SELECTION_NAMES = ("RobotTwoDriver", "RobotOneDriver", "registerAutons", "registerRobotOneAutons")
THIS_ROBOT_FILE = os.path.join("src", "common", "this_robot.hpp")


def _team_selection_problems(repo, rel):
    """Step 0 (header, step 0's ROB1-A rules): a src/ file outside src/team/ and THIS_ROBOT_FILE naming a
    robot's driver or autons registration (comments and literals stripped) — choosing a robot outside the
    one chain. Returns a list of problem strings."""
    if (not rel.startswith("src" + os.sep) or rel == THIS_ROBOT_FILE
            or rel.startswith(TEAM_DIR_REL + os.sep)):
        return []
    problems = []
    named = re.compile(r"\b(" + "|".join(TEAM_SELECTION_NAMES) + r")\b")
    for n, line in _logical_lines(_strip_comments(_read(repo, rel), literals=True)):
        for name in sorted(set(named.findall(line))):
            problems.append(
                f"{rel}:{n} names {name}, a robot's own driver or autons — a program runs THIS build's "
                f"(ThisRobotDriver, registerThisRobotsAutons(), {THIS_ROBOT_FILE}), chosen by kRobotId; "
                f"naming a robot's is choosing a robot outside the one chain (header, step 0, ROB1-A): "
                f"`{line.strip()}`")
    return problems


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
    namespaces = _robot_namespaces(repo)
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
                    problems += _retired_switch_problems(
                        rel, enumerate(_strip_cmake_comments(_read(repo, rel)).split("\n"), 1))
                    continue
                if not f.endswith(ONE_CHAIN_EXTS):
                    continue
                problems += _macro_problems(repo, rel, macro_names)
                problems += _retired_switch_problems(
                    rel, _logical_lines(_strip_comments(_read(repo, rel), literals=True)))
                if rel != SELECTION_HEADER:
                    problems += _stray_selection_problems(repo, rel, per_robot, namespaces)
                    problems += _robot_namespace_problems(repo, rel, namespaces)
                    problems += _team_selection_problems(repo, rel)
    problems += _selection_header_problems(repo)
    problems += ledger_source_problems(repo)
    return problems


def _live_switches_on(line):
    """The build-switch names `line` reads that are NOT retired (a retired one is
    _retired_switch_problems()' to report, once, naming the retirement)."""
    return {m.group(0) for m in READS_SWITCH.finditer(line)} - set(RETIRED_SWITCHES)


def _retired_switch_problems(rel, lines):
    """Any file step 0 reads, SELECTION_HEADER and the host build's CMake included (header, A
    RETIRED SWITCH): a RETIRED build switch named in its code. `lines` are the file's (line number,
    line) pairs with comments (and, for C++, string literals) already stripped. Returns a list of
    problem strings."""
    problems = []
    for n, line in lines:
        for m in READS_SWITCH.finditer(line):
            why = RETIRED_SWITCHES.get(m.group(0))
            if why is not None:
                problems.append(
                    f"{rel}:{n} names {m.group(0)}, a RETIRED build switch — {why}; it may not "
                    f"come back under its old name (header, step 0, A RETIRED SWITCH): "
                    f"`{line.strip()}`")
    return problems


def _stray_selection_problems(repo, rel, per_robot=frozenset(), namespaces=None):
    """One C++ file outside SELECTION_HEADER (header, step 0): a build switch named in its code
    (comments and string literals stripped), one of the chain's constants declared there, or —
    outside the two table files and test/ — a per-robot table named outside a static_assert. A
    robot's OWN team file (header, "…PER ROBOT"; MECH-A5) may name its own robot's MECHANISM list —
    and nothing else per-robot: another robot's list, any chassis table, any selection function."""
    problems = []
    code = _strip_comments(_read(repo, rel), literals=True)
    for n, line in _logical_lines(code):
        if _live_switches_on(line):
            problems.append(
                f"{rel}:{n} names a build switch — only {SELECTION_HEADER} may read them "
                f"(header, step 0): `{line.strip()}`")
        elif DECLARES_CHAIN_CONSTANT.search(line):
            problems.append(
                f"{rel}:{n} declares one of THE ONE CHAIN's constants — only "
                f"{SELECTION_HEADER}'s chain may (header, step 0): `{line.strip()}`")
    if per_robot and rel not in TABLE_FILES and not rel.startswith("test" + os.sep):
        namespaces = _robot_namespaces(repo) if namespaces is None else namespaces
        mine = _team_file_robot(rel, namespaces)
        if mine is not None:
            per_robot = frozenset(n for n in per_robot if n not in namespaces.get(mine, set()))
        if not per_robot:
            return problems
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
        if _live_switches_on(line):
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


# Step 0's [tune-row] check (TUNE1-B1; header, THE BUILDS): the Makefile's MECHANISM_ROBOTS list — the
# ROBOT values PROGRAM=tune accepts — held to the mechanism table file, the tune row naming no robot
# (no reading of $(ROBOT) or ${ROBOT} in it but the one list filter and its $(error)'s words — a
# comparison, a $(filter) of a name, a quoted ifneq all fail; self-test case 47), and a tune build in
# VARIANTS for every robot the list names.
MECHANISM_TABLE_FILE = os.path.join("src", "robots", "mechanism_table.hpp")
_MECHANISM_ROBOTS_LINE = re.compile(r"^MECHANISM_ROBOTS\s*:=(.*)$", flags=re.M)
_EMPTY_TABLE = re.compile(r"\bconstexpr\s+MechanismTable\s+(\w+)\s*=\s*(?:[\w:]*::)?mechanisms\(\s*\)\s*;")
_TUNE_ROW = re.compile(r"^else ifeq \(\$\(PROGRAM\),tune\)\n(.*?)^override EXTRA_CXXFLAGS\+=-DSHULIB_PROGRAM_TUNE$",
                       flags=re.M | re.S)
_ROBOT_COMPARED = re.compile(r"\(\s*\$\(ROBOT\)\s*,\s*[\w-]+\s*\)|\(\s*[\w-]+\s*,\s*\$\(ROBOT\)\s*\)")


def robots_with_mechanisms(repo=None):
    """The ROBOT values (the Makefile's) whose mechanism table is NOT empty, derived from the table
    file's selection function (`case RobotId::X: return <table>;`) and its empty tables
    (`constexpr MechanismTable <name> = mechanisms();`), and from this gate's own BUILD_ARGS and
    BUILD_ROBOT (which ROBOT value each RobotId is). Returns (set of ROBOT values, problems)."""
    repo = repo or REPO
    problems = []
    code = _strip_comments(_read(repo, MECHANISM_TABLE_FILE))
    body = re.search(r"mechanismTableFor\s*\(\s*RobotId\b[^)]*\)\s*\{(.*?)\n\}", code, flags=re.S)
    if not body:
        return set(), [f"{MECHANISM_TABLE_FILE} has no mechanismTableFor(RobotId) — the [tune-row] check "
                       f"cannot tell which robots have mechanisms"]
    cases = re.findall(r"case\s+RobotId::(\w+)\s*:\s*return\s+([\w:]+)\s*;", body.group(1))
    empty = set(_EMPTY_TABLE.findall(code))
    robot_value = {}
    for build, args in BUILD_ARGS.items():
        robot_args = [a for a in args if a.startswith("ROBOT=")]
        if len(robot_args) == 1 and build in BUILD_ROBOT:
            robot_value.setdefault(BUILD_ROBOT[build], robot_args[0].split("=", 1)[1])
    with_mechanisms = set()
    for robot_id, table in cases:
        name = table.rsplit("::", 1)[-1]
        if name in empty:
            continue
        if robot_id not in robot_value:
            problems.append(f"RobotId::{robot_id} has a mechanism table ({table}) but no build of this "
                            f"gate names its ROBOT value — add its builds (header, [tune-row])")
            continue
        with_mechanisms.add(robot_value[robot_id])
    if not cases:
        problems.append(f"{MECHANISM_TABLE_FILE}'s mechanismTableFor() has no case this check can read")
    return with_mechanisms, problems


def tune_row_problems(repo=None):
    """Step 0's [tune-row] check (header): returns a list of problem strings."""
    repo = repo or REPO
    makefile = _read(repo, "Makefile")
    lines = _MECHANISM_ROBOTS_LINE.findall(makefile)
    problems = []
    if len(lines) != 1:
        return [f"the Makefile sets MECHANISM_ROBOTS {len(lines)} times — exactly once (header, [tune-row])"]
    listed = set(lines[0].split())
    derived, problems = robots_with_mechanisms(repo)
    for r in sorted(listed - derived):
        problems.append(f"the Makefile's MECHANISM_ROBOTS names '{r}', whose mechanism table is EMPTY (or "
                        f"which is no ROBOT value) — PROGRAM=tune would build a Tune with nothing to tune "
                        f"(header, [tune-row])")
    for r in sorted(derived - listed):
        problems.append(f"robot '{r}' has a mechanism table but the Makefile's MECHANISM_ROBOTS does not "
                        f"name it — `make ROBOT={r} PROGRAM=tune` would refuse a robot Tune can tune "
                        f"(header, [tune-row])")
    row = _TUNE_ROW.search(makefile)
    if not row:
        problems.append("the Makefile has no `else ifeq ($(PROGRAM),tune)` row ending in its "
                        "-DSHULIB_PROGRAM_TUNE append (header, [tune-row])")
    else:
        text = "\n".join(l for l in row.group(1).splitlines() if not l.lstrip().startswith("#"))
        # ANY reading of $(ROBOT) in the row but the one list filter (and the $(error)'s own words) is a
        # robot named: `ifneq ($(ROBOT),tank)`, `ifeq ($(filter tank,$(ROBOT)),)`, `ifneq "$(ROBOT)" "tank"`,
        # `${ROBOT}` — the review round's finding: the pattern alone missed the last three forms
        allowed = "$(filter $(ROBOT),$(MECHANISM_ROBOTS))"
        reads = [l for l in text.replace("${ROBOT}", "$(ROBOT)").splitlines()
                 if "$(ROBOT)" in l.replace(allowed, "") and not l.lstrip().startswith("$(error ")]
        if _ROBOT_COMPARED.search(text) or reads:
            problems.append("the Makefile's PROGRAM=tune row compares $(ROBOT) with a robot's name — the "
                            "tune row names NO robot (the brief's re-check #10: a robot gets Tune by "
                            "registering its mechanism table) (header, [tune-row])")
        if "$(filter $(ROBOT),$(MECHANISM_ROBOTS))" not in text:
            problems.append("the Makefile's PROGRAM=tune row does not refuse a ROBOT outside "
                            "MECHANISM_ROBOTS (`$(filter $(ROBOT),$(MECHANISM_ROBOTS))`) (header, [tune-row])")
    for r in sorted(listed):
        builds = [v for v in VARIANTS if BUILD_ARGS.get(v) == (f"ROBOT={r}", "PROGRAM=tune")]
        if len(builds) != 1 or BUILD_PROGRAM.get(builds[0]) not in TUNE_PROGRAMS:
            problems.append(f"robot '{r}' is in MECHANISM_ROBOTS but this gate runs {len(builds)} Tune "
                            f"build(s) for it — exactly one (`ROBOT={r} PROGRAM=tune`, program Tune) "
                            f"(header, [tune-row])")
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
    literal, so a match in `text` is reported on the line its literal starts. A raw string literal's
    text is its own (between `delim(` and `)delim`). A character literal, a number, or any code
    between two literals, ends a run."""
    runs, text, pieces, end = [], None, None, None
    line, counted = 1, 0
    for m in _CXX_TOKENS.finditer(code):
        t = m.group(0)
        if m.group("raw") is not None:
            body = m.group("rawtext")
        elif t.startswith('"'):
            body = t[1:-1]
        else:
            if text is not None:
                runs.append((pieces, text))
            text = None
            continue
        line += code.count("\n", counted, m.start())
        counted = m.start()
        if text is not None and _LITERAL_GAP.fullmatch(code[end:m.start()]):
            pieces.append((len(text), line))
            text += body
        else:
            if text is not None:
                runs.append((pieces, text))
            text, pieces = body, [(0, line)]
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


def elf_defined_names(elf, nm):
    """Every name DEFINED in a linked ELF, demangled (step 8's live-tuning half)."""
    p = subprocess.run([nm, "-C", "--defined-only", elf], capture_output=True, text=True,
                       timeout=120)
    if p.returncode != 0:
        raise RuntimeError(f"{ARM_NM} exited {p.returncode}: {p.stderr.strip()}")
    names = []
    for line in p.stdout.splitlines():
        parts = line.split(" ", 2)
        if len(parts) == 3:
            names.append(parts[2].strip())
    return names


def tuning_problems(robot, names, elf_bytes=b""):
    """Step 8's live-tuning half (header) for one build, given every name its ELF defines and its bytes."""
    prog = BUILD_PROGRAM.get(robot)
    tuning = sorted({n for n in names if LIVE_TUNING_SYMBOL.search(n)})
    if LIVE_TUNING_BYTES in elf_bytes:
        at = elf_bytes.index(LIVE_TUNING_BYTES)
        text = elf_bytes[at:at + 80].split(b"\0", 1)[0].decode("ascii", "replace")
        tuning.append(f"the text {text!r} (a LiveTuning precondition's message — its code is linked, inlined)")
    chassis = sorted({n for n in names if NO_CHASSIS_SYMBOL.search(n)})
    problems = []
    if prog in TUNE_PROGRAMS:
        if not tuning:
            problems.append(f"build {robot} is the Tune program but bin/hot.package.elf links NO "
                            f"live-tuning symbol — the check that every other program links none "
                            f"would prove nothing (step 8)")
        for n in chassis[:3]:
            problems.append(f"build {robot} is the Tune program but bin/hot.package.elf links {n} — "
                            f"the Tune program never builds the drive (step 8)")
    else:
        for n in tuning[:3]:
            problems.append(f"build {robot} is the {prog} program but bin/hot.package.elf links the "
                            f"live-tuning symbol {n} — only \"shulib Tune\" may carry live tuning; a "
                            f"match runs exactly the robot file's numbers (step 8)")
        if prog in CHASSIS_PROGRAMS and not chassis:
            problems.append(f"build {robot} is the {prog} program but bin/hot.package.elf links no "
                            f"chassis symbol — the no-chassis pattern the Tune build is held to matches "
                            f"nothing it should (step 8)")
    return problems


#: Step 8's robot half (ROB1-A; the ROB1 brief's T7): the team code each robot's packages must NEVER link —
#: another robot's autons registration — and the driver-file path no package of the robot may carry.
ROBOT_FOREIGN_SYMBOLS = {
    "RobotOne": ("shulib::bench::registerAutons",),
    "RobotTwo": ("shulib::bench::registerRobotOneAutons",),
    "BenchBot": ("shulib::bench::registerRobotOneAutons",),
    "InventedXDrive": ("shulib::bench::registerRobotOneAutons",),
}
ROBOT_FOREIGN_STRINGS = {
    "RobotOne": (b"src/team/robot_two_driver.hpp\0", b"shulib-robot=robot-two\0"),
    "RobotTwo": (b"src/team/robot_one_driver.hpp\0", b"src/team/robot_one_autons.cpp\0", b"robot one: OWED"),
    "BenchBot": (b"src/team/robot_one_driver.hpp\0", b"src/team/robot_one_autons.cpp\0"),
    "InventedXDrive": (b"src/team/robot_one_driver.hpp\0", b"src/team/robot_one_autons.cpp\0"),
}


def robot_separation_problems(robot, linked, elf_bytes):
    """Step 8's robot half (ROB1-A, T7): build `robot` links none of another robot's team code and carries
    none of its driver-file words. `linked` is elf_bench_names()' set."""
    who = BUILD_ROBOT.get(robot)
    problems = []
    for sym in ROBOT_FOREIGN_SYMBOLS.get(who, ()):
        if sym in linked:
            problems.append(f"build {robot} (robot {who}) links {sym} — another robot's autons are in this "
                            f"robot's package (step 8, ROB1-A)")
    for text in ROBOT_FOREIGN_STRINGS.get(who, ()):
        if text in elf_bytes:
            problems.append(f"build {robot} (robot {who}) carries {text.rstrip(bytes(1)).decode()!r} — another "
                            f"robot's words are in this robot's package (step 8, ROB1-A)")
    return problems


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


def object_of(tu):
    """The object make builds from a src/ TU (common.mk): `src/programs/x.cpp` ->
    `bin/programs/x.cpp.o`."""
    return os.path.join("bin", os.path.relpath(tu, "src")) + ".o"


def built_objects():
    """Every object under bin/ (repo-relative), PROS's link timestamp aside (step 9)."""
    out = []
    for root, _dirs, files in os.walk(os.path.join(REPO, "bin")):
        for f in files:
            if f.endswith(".o") and f != STDIO_EXEMPT_OBJECT:
                out.append(os.path.relpath(os.path.join(root, f), REPO))
    return sorted(out)


def stdio_problems(robot, nm):
    """Step 9 for one build (header): every src/ object's undefined symbols against STDIO_BANNED,
    after the build. Fails, too, when a src/ TU has no object — a check that read nothing passes
    nothing."""
    objects = built_objects()
    problems = []
    missing = [object_of(tu) for tu in expected_tus() if object_of(tu) not in objects]
    if missing:
        problems.append(f"build {robot}: no object for {missing} under bin/ — the stdio check "
                        f"(step 9) cannot read what it cannot find")
    if not objects:
        return problems or [f"build {robot}: no object under bin/ (step 9 read nothing)"]
    p = subprocess.run([nm, "-A", "-u", *objects], cwd=REPO, capture_output=True, text=True,
                       timeout=120)
    if p.returncode != 0:
        raise RuntimeError(f"{ARM_NM} -u exited {p.returncode}: {p.stderr.strip()}")
    for line in p.stdout.splitlines():
        m = _NM_UNDEFINED.match(line.strip())
        if m is None or m.group("sym") not in STDIO_BANNED:
            continue
        sym = m.group("sym")
        what = ("stdin/stdout/stderr (newlib's _impure_ptr: `fflush(stdout)`, "
                "`fwrite(..., stdout)`, ...)" if sym == "_impure_ptr" else
                "stdio's FILEs (`__sf`: `&__sf[1]` is stdout)" if sym == "__sf" else f"`{sym}`")
        problems.append(
            f"build {robot}: {m.group('obj')} references {what} — a write through stdio's "
            f"shared, unguarded FILEs, which two tasks can corrupt (A4 register HA-189). Send each "
            f"line as ONE write(): hal::pros::serialLine() / serialWrapped() or a sink "
            f"(include/shulib/hal/pros/char_sink.hpp) (step 9)")
    return problems


def _defined_symbols(nm, archive):
    """Every global symbol `archive` DEFINES (any section: a function, stdout's FILE pointer, a
    stream object), by name."""
    p = subprocess.run([nm, "-g", "--defined-only", archive], capture_output=True, text=True,
                       timeout=120)
    if p.returncode != 0:
        raise RuntimeError(f"{ARM_NM} on {archive} exited {p.returncode}: {p.stderr.strip()}")
    return {parts[2] for parts in (line.split() for line in p.stdout.splitlines())
            if len(parts) == 3}


def toolchain_libstdcxx(path_env=None):
    """The libstdc++.a the build links (common.mk's `-lstdc++`), for common.mk's own MFLAGS —
    read from common.mk, never typed twice — or None."""
    gxx = find_compiler(path_env)
    if gxx is None:
        return None
    with open(os.path.join(REPO, "common.mk"), encoding="utf-8") as fh:
        m = re.search(r"^MFLAGS=(.*)$", fh.read(), re.M)
    if m is None:
        return None
    p = subprocess.run([gxx, *m.group(1).split(), "-print-file-name=libstdc++.a"],
                       capture_output=True, text=True, timeout=60)
    path = p.stdout.strip()
    return path if p.returncode == 0 and os.path.isabs(path) and os.path.isfile(path) else None


def stdio_list_problems(nm, banned=None, path_env=None):
    """Step 9's LIST against the libraries it names (header), once per check: every printer the
    vendored libc.a defines that writes stdout/stderr without naming it (STDIO_IMPLICIT_WRITER)
    must be in `banned`, and every name in `banned` must be DEFINED by libc.a or the toolchain's
    libstdc++.a — a misspelt name bans nothing. `banned` defaults to STDIO_BANNED as it is AT THE
    CALL (the self-test rebinds it). Returns (the writers found, the problems)."""
    if banned is None:
        banned = STDIO_BANNED
    libc = _defined_symbols(nm, VENDORED_LIBC)
    writers = sorted(s for s in libc if STDIO_IMPLICIT_WRITER.match(s))
    problems = []
    if not writers:
        problems.append(f"no stdout/stderr printer found in {VENDORED_LIBC} — the list check "
                        f"(step 9) read nothing")
    for w in writers:
        if w not in banned:
            problems.append(f"{w}: the vendored libc.a defines it and it writes stdout/stderr "
                            f"without naming it, so an object calling it references only `{w}` "
                            f"— it is not in STDIO_BANNED, and every build calling it PASSES "
                            f"(step 9)")
    cxx = toolchain_libstdcxx(path_env)
    if cxx is None:
        problems.append("the toolchain's libstdc++.a was not found — the C++ stream names in "
                        "STDIO_BANNED cannot be checked (step 9)")
        defined = libc
    else:
        defined = libc | _defined_symbols(nm, cxx)
    for name in sorted(banned - defined):
        problems.append(f"{name}: in STDIO_BANNED, but neither the vendored libc.a nor the "
                        f"toolchain's libstdc++.a defines it — a misspelt name bans nothing "
                        f"(step 9)")
    return writers, problems


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
    if BUILD_ROBOT.get(robot) != "RobotOne" and "robot one: OWED" in b.text:
        problems.append(f"build {robot} printed a `robot one: OWED` line — robot one's facts refusal fires in a "
                        f"build that is not robot one's (header, ROBOT ONE'S FACTS PROOF)")

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
        for retired, why in RETIRED_DEFINES.items():
            if retired in tokens:
                problems.append(f"build {robot} but {tu} carries the RETIRED define {retired} — "
                                f"{why} (header, step 5)")

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
        for beacon, why in RETIRED_BEACONS.items():
            if _beacon_bytes(beacon) in elf_bytes:
                problems.append(
                    f"build {robot} but bin/hot.package.elf carries the RETIRED beacon "
                    f"{beacon.decode()!r} — {why} (header, step 6)")

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
                linked = elf_bench_names(elf, nm)
                problems += program_problems(robot, linked)
                problems += tuning_problems(robot, elf_defined_names(elf, nm), elf_bytes)
                problems += robot_separation_problems(robot, linked, elf_bytes)
            except (RuntimeError, OSError, subprocess.SubprocessError) as e:
                problems.append(f"the program check (step 8) could not read the symbol table: "
                                f"{e}")
    else:
        problems.append("bin/hot.package.elf missing — the beacon (step 6) and the program "
                        "(step 8) cannot be checked")

    # Step 9 (header): ONE-CALL WRITES — no src/ object names stdio's FILEs or calls a stdout/stderr
    # printer. From the OBJECTS, before --gc-sections could hide an unused one.
    nm = find_nm()
    if nm is None:
        problems.append(f"{ARM_NM} not found — the stdio check (step 9) cannot run, and a skip "
                        f"is not a pass")
    else:
        try:
            problems += stdio_problems(robot, nm)
        except (RuntimeError, OSError, subprocess.SubprocessError) as e:
            problems.append(f"the stdio check (step 9) could not read the objects: {e}")

    return not problems, problems


# ── ROBOT ONE'S FACTS PROOF (chunk ROB1-A, 2026-11-11; the ROB1 brief's D3 (a), [R1], [R4], [R11]) ─────────
# src/robots/robot_one_facts.hpp writes every robot-one number nobody has measured as OWED, and a check under
# each fact FAILS robot one's build of every program that needs it, naming it. This gate proves BOTH halves:
#   (i)  THE REAL TREE — robot one's six builds are rows of VARIANTS like every build; each one either links
#        with nothing it needs owed (then every check the eight builds pass), or FAILS with EXACTLY the
#        `robot one: OWED -- <name>` lines of the owed facts its program needs — compared as a SET (one TU
#        repeats them all), the expected set computed here from the ledger's own text — with no other error
#        and NO bin/hot.package.bin left behind (the Makefile's variant stamp, [R1]). Every other build's
#        output must carry no `robot one: OWED` line at all.
#   (ii) A COPY, NEVER THE REPO ([R4]) — the working tree's files (`git ls-files -co --exclude-standard`, so
#        the UNCOMMITTED change is what is proven) copied to build/src-gate-facts/ (gitignored); in the copy
#        ONLY every `= owed(p, stage, "name")` becomes `= typed(p, p, stage, "name", "gate copy -- same as the
#        placeholder, checked")` and the copy's marker file turns the ledger's kRefuseGateCopySource off; robot one's six builds
#        are then built THERE and must pass every check the eight builds pass. The copy is deleted in a
#        `finally` and a leftover one swept at the start; the repository's ledger hash before = after, and the
#        repository's bin/ is never touched by it. A ledger copied back from the copy cannot link: the real
#        ledger refuses the "gate copy" source at make, and step 0 refuses it too.
FACTS_LEDGER = os.path.join("src", "robots", "robot_one_facts.hpp")
FACTS_COPY_DIR = os.path.join("build", "src-gate-facts")
FACTS_COPY_SOURCE = "gate copy -- same as the placeholder, checked"
#: The marker FILE that turns the ledger's gate-copy refusal off — written at the COPY's root only, never
#: in the repository (step 0 refuses it there); the ledger reads it relative to its own directory.
FACTS_COPY_MARKER = os.path.join("build", "ROBOT-ONE-GATE-COPY-ONLY")
#: The ledger's ONE reading of the marker (type_every_fact() requires it exactly once).
FACTS_MARKER_READ = '#if __has_include("../../build/ROBOT-ONE-GATE-COPY-ONLY")'
#: EVERY program, classified — the ledger's needsStageB() restated (step 0's stage_table_problems() reads
#: the C++ switch back and requires the two to agree, and every `Program` value to be in exactly one set).
STAGE_B_PROGRAMS = frozenset({"Drive", "AutonTester", "Competition", "InventedXDrive", "Tune"})
STAGE_A_ONLY_PROGRAMS = frozenset({"BenchTests", "Calibrate"})
#: Which programs need a stage's facts — the ledger's needs() (test/robot_one_facts_test.cpp pins the C++).
STAGE_PROGRAMS = {
    "A": STAGE_B_PROGRAMS | STAGE_A_ONLY_PROGRAMS,
    "B": STAGE_B_PROGRAMS,
    "Recorded": frozenset(),
}
#: A fact's declaration — never matched inside the ledger's banner, whose examples write no declaration.
_FACT_DECL = re.compile(r"\binline\s+constexpr\s+auto\s+k\w+\s*=\s*(owed|typed)\s*\(")
_OWED_MESSAGE = re.compile(r"^static assertion failed: robot one: OWED -- (.*) \(src/robots/robot_one_facts\.hpp\)$")


def _call_args(code, open_at):
    """The top-level arguments of the call whose `(` is at `open_at` in `code` (string-aware), and the
    index just past its `)`."""
    args, cur, depth, quoted, i = [], "", 0, False, open_at
    while i < len(code):
        c = code[i]
        if quoted:
            cur += c
            if c == "\\" and i + 1 < len(code):
                cur += code[i + 1]
                i += 1
            elif c == '"':
                quoted = False
        elif c == '"':
            quoted = True
            cur += c
        elif c in "([{":
            depth += 1
            if depth > 1:
                cur += c
        elif c in ")]}":
            depth -= 1
            if depth == 0:
                args.append(cur)
                return args, i + 1
            cur += c
        elif c == "," and depth == 1:
            args.append(cur)
            cur = ""
        else:
            cur += c
        i += 1
    return args, len(code)


def _literal_text(arg):
    """The text of one string-literal argument (adjacent literals joined), or None."""
    parts = re.findall(r'"((?:\\.|[^"\\])*)"', arg)
    return "".join(parts) if parts else None


def ledger_facts(repo=None):
    """Every fact the ledger declares, from its TEXT (comments stripped, literals kept): a list of dicts
    {kind: owed|typed, stage: A|B|Recorded, name, source, start, end} — `start`/`end` the call's span in
    the comment-stripped text. Raises ValueError on a fact it cannot read (a gate that cannot read the
    ledger proves nothing)."""
    repo = repo or REPO
    code = _strip_comments(_read(repo, FACTS_LEDGER))
    out = []
    for m in _FACT_DECL.finditer(code):
        open_at = m.end() - 1
        args, end = _call_args(code, open_at)
        kind = m.group(1)
        want = 3 if kind == "owed" else 5
        if len(args) != want:
            raise ValueError(f"{FACTS_LEDGER}: a {kind}( with {len(args)} arguments, not {want}: "
                             f"`{code[m.start():end][:120]}`")
        stage = re.search(r"Stage::(\w+)", args[1 if kind == "owed" else 2])
        name = _literal_text(args[2 if kind == "owed" else 3])
        if stage is None or stage.group(1) not in STAGE_PROGRAMS or name is None:
            raise ValueError(f"{FACTS_LEDGER}: a {kind}( whose stage or name this gate cannot read: "
                             f"`{code[m.start():end][:120]}`")
        out.append({"kind": kind, "stage": stage.group(1), "name": name,
                    "source": _literal_text(args[4]) if kind == "typed" else None,
                    "start": m.start(), "end": end, "args": args})
    return out


def expected_owed(build, repo=None):
    """The `robot one: OWED -- <name>` names robot-one build `build` must refuse with: every OWED fact
    whose stage its program needs (STAGE_PROGRAMS). A program STAGE_PROGRAMS does not classify is a
    ValueError — never quietly "Stage A only" (the review of ROB1-A: that is how a new program would
    build robot one on Stage A alone with this gate agreeing)."""
    program = BUILD_PROGRAM[build]
    if program not in STAGE_PROGRAMS["A"]:
        raise ValueError(f"build {build}'s program {program} is not classified in STAGE_PROGRAMS (header, "
                         f"STEP 0's ROB1-A RULES) — say whether it needs Stage B, here AND in the ledger's "
                         f"needsStageB()")
    return {f["name"] for f in ledger_facts(repo) if f["kind"] == "owed" and program in STAGE_PROGRAMS[f["stage"]]}


def _braced_body(code, open_at):
    """The text between the `{` at `open_at` and its matching `}` (comment-stripped code), or None."""
    depth = 0
    for i in range(open_at, len(code)):
        if code[i] == "{":
            depth += 1
        elif code[i] == "}":
            depth -= 1
            if depth == 0:
                return code[open_at + 1:i]
    return None


def stage_table_problems(repo=None):
    """Step 0 (header, STEP 0's ROB1-A RULES — the review of ROB1-A): every value of `Program` has exactly one
    case in the ledger's needsStageB() switch, which has no `default`; the switch agrees with STAGE_PROGRAMS
    program by program; and every robot-one row's BUILD_PROGRAM is classified."""
    repo = repo or REPO
    problems = []
    sel = _strip_comments(_read(repo, SELECTION_HEADER))
    m = re.search(r"enum\s+class\s+Program\s*\{", sel)
    body = _braced_body(sel, m.end() - 1) if m else None
    if body is None:
        return [f"{SELECTION_HEADER}: `enum class Program {{ … }}` not found — the stages cannot be checked"]
    programs = [re.match(r"\s*(\w+)", e).group(1) for e in body.split(",") if e.strip()]
    ledger = _strip_comments(_read(repo, FACTS_LEDGER))
    f = re.search(r"constexpr\s+bool\s+needsStageB\s*\(\s*Program\s+p\s*\)\s*\{", ledger)
    fbody = _braced_body(ledger, f.end() - 1) if f else None
    sw = re.search(r"switch\s*\(\s*p\s*\)\s*\{", fbody) if fbody is not None else None
    swbody = _braced_body(fbody, sw.end() - 1) if sw else None
    if swbody is None:
        return [f"{FACTS_LEDGER}: needsStageB(Program p)'s `switch (p) {{ … }}` not found — the stages cannot "
                f"be checked"]
    if re.search(r"\bdefault\s*:", swbody):
        problems.append(f"{FACTS_LEDGER}: needsStageB()'s switch has a `default` — a program added later would "
                        f"take it silently instead of failing -Wswitch")
    classified, pending = {}, []
    for t in re.finditer(r"case\s+Program::(\w+)\s*:|return\s+(true|false)\s*;", swbody):
        if t.group(1):
            pending.append(t.group(1))
            continue
        for name in pending:
            if name in classified:
                problems.append(f"{FACTS_LEDGER}: needsStageB() has two cases for Program::{name}")
            classified[name] = t.group(2) == "true"
        pending = []
    if pending:
        problems.append(f"{FACTS_LEDGER}: needsStageB()'s cases {pending} reach no `return true;`/`return false;`")
    missing = [n for n in programs if n not in classified]
    extra = sorted(set(classified) - set(programs))
    if missing:
        problems.append(f"{FACTS_LEDGER}: needsStageB() has no case for Program::{', Program::'.join(missing)} "
                        f"— say whether it needs Stage B (it moves the cascade? then true)")
    if extra:
        problems.append(f"{FACTS_LEDGER}: needsStageB() names {extra}, which {SELECTION_HEADER}'s Program lacks")
    if STAGE_B_PROGRAMS & STAGE_A_ONLY_PROGRAMS:
        problems.append(f"STAGE_B_PROGRAMS and STAGE_A_ONLY_PROGRAMS share {sorted(STAGE_B_PROGRAMS & STAGE_A_ONLY_PROGRAMS)}")
    for n in programs:
        if n not in STAGE_PROGRAMS["A"]:
            problems.append(f"this gate's STAGE_PROGRAMS does not classify Program::{n} (header, STEP 0's ROB1-A "
                            f"RULES)")
        elif n in classified and classified[n] != (n in STAGE_B_PROGRAMS):
            problems.append(f"Program::{n}: the ledger's needsStageB() says {str(classified[n]).lower()}, this "
                            f"gate's STAGE_PROGRAMS says {str(n in STAGE_B_PROGRAMS).lower()} — the two must "
                            f"agree")
    for v in VARIANTS:
        if BUILD_ROBOT.get(v) == "RobotOne" and BUILD_PROGRAM.get(v) not in STAGE_PROGRAMS["A"]:
            problems.append(f"robot-one build {v}'s program {BUILD_PROGRAM.get(v)} is not classified in "
                            f"STAGE_PROGRAMS")
    return problems


def ledger_source_problems(repo=None):
    """Step 0 ([R4]): a TYPED fact in the repository's ledger whose source is empty or carries the gate
    copy's marker — the ledger's own check refuses both at make; this names them without a build."""
    repo = repo or REPO
    if not os.path.exists(os.path.join(repo, FACTS_LEDGER)):
        return []
    problems = []
    if os.path.exists(os.path.join(repo, FACTS_COPY_MARKER)):
        problems.append(f"{FACTS_COPY_MARKER} exists in the repository — it is the src build gate's COPY's "
                        f"marker, and here it turns the ledger's gate-copy refusal off for every build; "
                        f"delete it (header, ROBOT ONE'S FACTS PROOF)")
    problems += stage_table_problems(repo)
    try:
        facts = ledger_facts(repo)
    except ValueError as e:
        return problems + [str(e)]
    for f in facts:
        if f["kind"] != "typed":
            continue
        if not f["source"]:
            problems.append(f"{FACTS_LEDGER}: the fact '{f['name']}' is typed with no source — a typed fact "
                            f"says who measured it, how and when (header, ROBOT ONE'S FACTS PROOF)")
        elif "gate copy" in f["source"]:
            problems.append(f"{FACTS_LEDGER}: the fact '{f['name']}' carries the gate copy's source "
                            f"'{f['source']}' — a ledger copied back from build/src-gate-facts/ (header, "
                            f"ROBOT ONE'S FACTS PROOF)")
    return problems


def type_every_fact(text):
    """The ledger `text` with EVERY `= owed(p, stage, "name")` typed as `= typed(p, p, stage, "name",
    FACTS_COPY_SOURCE)` — the copy's edit, never the repo's. The gate-copy refusal is NOT touched here: it
    is turned off by the marker FILE facts_copy_proof() writes at the copy's root. Works on
    the RAW text (comments kept): a declaration is `inline constexpr auto k… = owed(`, which no comment in
    the ledger writes. Raises ValueError when the edit cannot be made exactly."""
    stripped = text
    pieces, last, typed = [], 0, 0
    for m in _FACT_DECL.finditer(stripped):
        if m.group(1) != "owed":
            continue
        args, end = _call_args(stripped, m.end() - 1)
        if len(args) != 3:
            raise ValueError(f"an owed( with {len(args)} arguments")
        raw = [text[a:b] for a, b in _arg_spans(stripped, m.end() - 1)]
        eq = text.index("=", m.start())
        pieces.append(text[last:eq])
        pieces.append(f'= typed({raw[0]}, {raw[0]}, {raw[1]}, {raw[2]}, "{FACTS_COPY_SOURCE}")')
        last = end
        typed += 1
    pieces.append(text[last:])
    out = "".join(pieces)
    if out.count(FACTS_MARKER_READ) != 1:
        raise ValueError(f"the ledger does not read the copy's marker exactly once (`{FACTS_MARKER_READ}`)")
    return out, typed


def _arg_spans(code, open_at):
    """(start, end) of each top-level argument of the call whose `(` is at `open_at` — _call_args' twin."""
    spans, depth, quoted, i, start = [], 0, False, open_at, open_at + 1
    while i < len(code):
        c = code[i]
        if quoted:
            if c == "\\":
                i += 1
            elif c == '"':
                quoted = False
        elif c == '"':
            quoted = True
        elif c in "([{":
            depth += 1
        elif c in ")]}":
            depth -= 1
            if depth == 0:
                spans.append((start, i))
                return spans
        elif c == "," and depth == 1:
            spans.append((start, i))
            start = i + 1
        i += 1
    return spans


def facts_verdict(robot, b, repo=None):
    """The real tree's verdict for robot-one build `robot` (header, ROBOT ONE'S FACTS PROOF (i)): nothing
    owed -> the ordinary verdict; else it must FAIL with exactly the expected OWED set, no other error, and
    no package left in bin/. Returns (ok, problems, refused) — `refused` True when it refused as it must."""
    repo = repo or REPO
    try:
        want = expected_owed(robot, repo)
    except ValueError as e:
        return False, [f"the ledger cannot be read: {e}"], False
    if not want:
        ok, problems = verdict(robot, b)
        return ok, problems, False
    problems = []
    seen, other = set(), []
    for path, lno, msg in b.errors:
        m = _OWED_MESSAGE.match(msg)
        if m:
            seen.add(m.group(1))
        else:
            other.append(f"{path}:{lno} — {msg}")
    if b.exit == 0:
        problems.append(f"build {robot} LINKED although its program needs {len(want)} owed fact(s) — a "
                        f"placeholder can reach the robot (header, ROBOT ONE'S FACTS PROOF)")
    for o in other:
        problems.append(f"build {robot} refused with an error that is not an OWED fact: {o}")
    if seen != want:
        missing = sorted(want - seen)
        extra = sorted(seen - want)
        problems.append(f"build {robot} refused naming the wrong facts — owed and not named: {missing}; named "
                        f"and not owed by this program: {extra} (header, ROBOT ONE'S FACTS PROOF)")
    for leftover in ("hot.package.bin", "hot.package.elf"):
        if os.path.exists(os.path.join(repo, "bin", leftover)):
            problems.append(f"build {robot} refused but bin/{leftover} exists — a package is there to upload "
                            f"(the Makefile's variant stamp, [R1])")
    return not problems, problems, not problems


def _working_tree_files(repo):
    """Every file of the working tree git knows or would add (tracked + untracked, ignored excluded)."""
    r = subprocess.run(["git", "ls-files", "-co", "--exclude-standard", "-z"], cwd=repo,
                       capture_output=True, timeout=60)
    if r.returncode != 0:
        raise RuntimeError(f"git ls-files failed: {r.stderr.decode(errors='replace').strip()}")
    return [f for f in r.stdout.decode().split("\0") if f]


def _sha256(path):
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def facts_copy_proof(path_env=None, say=print, _fail_after_typing=False):
    """THE COPY (header, ROBOT ONE'S FACTS PROOF (ii)). Returns a list of problem strings. `_fail_after_typing`
    (the self-test only) raises once the copy's ledger is typed — an interrupted proof."""
    global REPO
    repo = REPO
    copy = os.path.join(repo, FACTS_COPY_DIR)
    problems = []
    if os.path.exists(copy):
        shutil.rmtree(copy)
        say(f"  [facts-copy] swept a leftover {FACTS_COPY_DIR}/ (an earlier proof was interrupted)")
    ledger = os.path.join(repo, FACTS_LEDGER)
    before = _sha256(ledger)
    package_before = None
    hot = os.path.join(repo, "bin", "hot.package.bin")
    if os.path.exists(hot):
        package_before = _sha256(hot)
    builds = [v for v in VARIANTS if BUILD_ROBOT.get(v) == "RobotOne"]
    try:
        for rel in _working_tree_files(repo):
            src = os.path.join(repo, rel)
            if rel.startswith(FACTS_COPY_DIR + os.sep) or not os.path.isfile(src):
                continue  # a deleted-but-tracked file, or the copy itself
            dst = os.path.join(copy, rel)
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            shutil.copy2(src, dst)
        copied = os.path.join(copy, FACTS_LEDGER)
        with open(copied, encoding="utf-8") as f:
            text = f.read()
        typed_text, n = type_every_fact(text)
        with open(copied, "w", encoding="utf-8") as f:
            f.write(typed_text)
        marker = os.path.join(copy, FACTS_COPY_MARKER)
        os.makedirs(os.path.dirname(marker), exist_ok=True)
        with open(marker, "w", encoding="utf-8") as f:
            f.write("the src build gate's facts-proof COPY (tools/src_build_gate.py) -- never in the repository\n")
        if _fail_after_typing:
            raise RuntimeError("self-test: the proof interrupted after the copy's ledger was typed")
        REPO = copy
        try:
            for v in builds:
                b = run_build(v, path_env)
                ok, p = verdict(v, b)
                say(_summary(v, b).replace(f"[{v:10}]", f"[{v} (copy)]"))
                if not ok:
                    problems += [f"[{v} (copy, every fact typed)] {x}" for x in p]
        finally:
            REPO = repo
        say(f"  [facts-copy] {n} owed facts typed in {FACTS_COPY_DIR}/ only; robot one's {len(builds)} builds "
            f"{'link and pass every check' if not problems else 'do NOT all pass — see below'}")
    finally:
        REPO = repo
        shutil.rmtree(copy, ignore_errors=True)
    if os.path.exists(copy):
        problems.append(f"{FACTS_COPY_DIR}/ is still there after the proof — the `finally` did not delete it")
    if _sha256(ledger) != before:
        problems.append(f"the repository's {FACTS_LEDGER} CHANGED during the copy's proof — the proof must "
                        f"never touch the repo")
    if os.path.exists(os.path.join(repo, FACTS_COPY_MARKER)):
        problems.append(f"the copy's marker {FACTS_COPY_MARKER} is in the REPOSITORY after the proof")
    if os.path.exists(hot):
        if _sha256(hot) != package_before:
            problems.append("the repository's bin/hot.package.bin CHANGED during the copy's proof — the proof "
                            "must never touch the repo's bin/")
        with open(os.path.join(repo, "bin", "hot.package.bin"), "rb") as f:
            if FACTS_COPY_SOURCE.encode() in f.read():
                problems.append("the repository's bin/hot.package.bin carries the gate copy's source")
    elif package_before is not None:
        problems.append("the repository's bin/hot.package.bin VANISHED during the copy's proof")
    return problems


def _summary(robot, b):
    allowed = (f"{len(b.allowed)} allowed ({', '.join(sorted(ALLOWED_CATEGORIES))}), "
               if ALLOWED_CATEGORIES else "")
    return (f"  [{robot:10}] exit {b.exit}, {len(b.compiled)}/{len(expected_tus())} TUs "
            f"compiled, warnings: {allowed}{len(b.vendor)} vendor-ignored, "
            f"{len(b.disallowed)} disallowed, hash "
            f"{'asserted (' + b.hash_expected + ')' if b.hash_expected else 'UNAVAILABLE'}"
            f", beacons (variant, robot), program and stdio printers checked")


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
    tune_problems = [f"[tune-row] {p}" for p in tune_row_problems()]
    all_problems += tune_problems
    say(f"  [tune-row] the Makefile's MECHANISM_ROBOTS "
        f"{'is the robots whose mechanism table is not empty, its tune row names no robot, and each has its tune build' if not tune_problems else 'does NOT hold — see below'}")
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
    # Step 9's list, once (header): against the libraries it names, before any build reads it.
    try:
        writers, list_problems = stdio_list_problems(find_nm(path_env), path_env=path_env)
    except (RuntimeError, OSError, subprocess.SubprocessError) as e:
        writers, list_problems = [], [f"the stdio list (step 9) could not be checked: {e}"]
    all_problems += [f"[stdio-list] {p}" for p in list_problems]
    say(f"  [stdio-list] {len(STDIO_BANNED)} banned names, every one defined by the vendored "
        f"libc.a or libstdc++.a, and all {len(writers)} stdout/stderr printers libc.a defines "
        f"among them" if not list_problems else
        f"  [stdio-list] NOT CLEAN, see below")

    refused_as_owed = 0
    for robot in VARIANTS:
        b = run_build(robot, path_env)
        if BUILD_ROBOT.get(robot) == "RobotOne":
            ok, problems, refused = facts_verdict(robot, b)
        else:
            (ok, problems), refused = verdict(robot, b), False
        if refused:
            refused_as_owed += 1
            say(f"  [{robot:10}] REFUSED as it must: exactly its {len(expected_owed(robot))} owed fact(s), "
                f"each by name, no other error, no package left in bin/ (the facts proof's real-tree half)")
        else:
            say(_summary(robot, b))
        if not ok:
            all_problems += [f"[{robot}] {p}" for p in problems]

    # The facts proof's COPY half (header, ROBOT ONE'S FACTS PROOF (ii)) — never in the repo.
    if any(BUILD_ROBOT.get(v) == "RobotOne" for v in VARIANTS):
        try:
            all_problems += [f"[facts-copy] {p}" for p in facts_copy_proof(path_env, say)]
        except (RuntimeError, OSError, ValueError, subprocess.SubprocessError) as e:
            all_problems.append(f"[facts-copy] the copy's proof could not run: {e}")

    if all_problems:
        print("\nsrc build gate: FAIL", file=sys.stderr)
        for p in all_problems:
            print(f"  {p}", file=sys.stderr)
        return 1

    allowed = (f"{'/'.join(sorted(ALLOWED_CATEGORIES))} allowed by category, "
               if ALLOWED_CATEGORIES else "no category allowed, ")
    say(f"src build gate: PASS (all {len(VARIANTS)} builds link or, robot one's, refuse naming exactly "
        f"their owed facts ({refused_as_owed} refused; every one links in the gate's typed copy); policy: errors fail, "
        f"vendor warnings path-ignored, {allowed}every other warning fails; defines "
        f"token-exact; one chain picks program and robot; variant and robot beacons asserted; "
        f"programs asserted; live tuning only in Tune, Tune with no chassis; src/'s include root unambiguous; every printed src/ path exists; "
        f"no src/ object names stdio's FILEs or calls a stdout/stderr printer)")
    return 0


def _check_with(banned, variants):
    """cmd_check, quiet, with STDIO_BANNED and VARIANTS rebound — the self-test's case 40, which
    empties VARIANTS so no build runs — both restored after; (its exit code, its stderr)."""
    global STDIO_BANNED, VARIANTS
    saved = (STDIO_BANNED, VARIANTS)
    said = io.StringIO()
    try:
        STDIO_BANNED, VARIANTS = banned, variants
        with contextlib.redirect_stderr(said):
            rc = cmd_check(quiet=True)
    finally:
        STDIO_BANNED, VARIANTS = saved
    return rc, said.getvalue()


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
DRIVE_PLANT = os.path.join(REPO, "src", "programs", "drive_program.cpp")     # (26, 28, 29, 30)
CALIBRATE_PLANT = os.path.join(REPO, "src", "robots", "calibrate_facts.hpp")      # (26)
CMAKE_PLANT = os.path.join(REPO, "test", "CMakeLists.txt")                          # (27)
COMMON_PLANT = os.path.join(REPO, "src", "common", "boot_source.hpp")               # (28)
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


def _self_test_robot_half(expect):
    """Self-test case 58 (the review of ROB1-A's integration, 2026-11-13): step 8's ROBOT HALF. Its own
    function so a mutation probe can run exactly this case against a mutated gate."""
    # (a) every detector of robot_separation_problems(), on SYNTHETIC ELF contents (pure: no build)
    for robot, linked, elf_bytes, want in (
            ("one-tune", {"shulib::bench::registerAutons"}, b"", "another robot's autons"),
            ("one", set(), b"x\0shulib-robot=robot-two\0", "another robot's words"),
            ("one-drive", set(), b"src/team/robot_two_driver.hpp\0", "another robot's words"),
            ("tank-tune", {"shulib::bench::registerRobotOneAutons"}, b"", "another robot's autons"),
            ("tank-competition", set(), b"robot one: OWED -- x", "another robot's words"),
            ("tank-drive", set(), b"src/team/robot_one_driver.hpp\0", "another robot's words"),
            ("bench", {"shulib::bench::registerRobotOneAutons"}, b"", "another robot's autons"),
            ("xdrive", set(), b"src/team/robot_one_autons.cpp\0", "another robot's words")):
        found = robot_separation_problems(robot, linked, elf_bytes)
        expect(any(want in x and "(step 8, ROB1-A)" in x for x in found),
               f"58: build {robot} with {sorted(linked)} / {elf_bytes!r} PASSED step 8's robot half: {found}")
    # ...and the clean ones pass (each robot's OWN registration, beacon and driver path)
    for robot, linked, elf_bytes in (
            ("one-tune", {"shulib::bench::registerRobotOneAutons"},
             b"shulib-robot=robot-one\0src/team/robot_one_driver.hpp\0"),
            ("tank-competition", {"shulib::bench::registerAutons"},
             b"shulib-robot=robot-two\0src/team/robot_two_driver.hpp\0")):
        expect(not robot_separation_problems(robot, linked, elf_bytes),
               f"58: build {robot}'s own team code FAILED step 8's robot half: "
               f"{robot_separation_problems(robot, linked, elf_bytes)}")
    # (b) every robot a build names has a forbidden symbol AND a forbidden word (an emptied row is no check)
    for robot in VARIANTS:
        who = BUILD_ROBOT.get(robot)
        expect(ROBOT_FOREIGN_SYMBOLS.get(who) and ROBOT_FOREIGN_STRINGS.get(who),
               f"58: robot {who} (build {robot}) has no ROBOT_FOREIGN_SYMBOLS or ROBOT_FOREIGN_STRINGS row")
    # (c) THE CALL SITE, on one real build: robot two's autons file reaching robot one's registration behind a
    #     condition no compiler can fold -> robot two's Auton Tester links registerRobotOneAutons -> verdict()
    #     FAILS step 8 by name, ON BOTH OF THE CALL'S INPUTS: the linked symbol (`linked`) AND the ELF's words
    #     (`elf_bytes` — the package's DWARF names its compile unit `src/team/robot_one_autons.cpp`, -g in
    #     common.mk's MFLAGS). Deleting the call turns this red; so does passing it b"" for the ELF's bytes,
    #     which checked only the symbol half survived (the verification of the integration's review, 2026-11-13).
    autons_path = os.path.join(TEAM_DIR, "autons.cpp")

    def plant_robot_one(text):
        text = text.replace('#include "team/autons.hpp"\n',
                            '#include "team/autons.hpp"\n'
                            '#include "team/robot_one_autons.hpp"  // (self-test plant)\n', 1)
        return text.replace(
            "void registerAutons(competition::AutonRegistry& registry) {\n",
            "void registerAutons(competition::AutonRegistry& registry) {\n"
            "    static volatile bool robotPlantNever = false;  // (self-test plant)\n"
            "    if (robotPlantNever) { registerRobotOneAutons(registry); }  // (self-test plant)\n", 1)

    with _transform(autons_path, plant_robot_one):
        b = run_build("tank-autontest")
        ok, problems = verdict("tank-autontest", b)
        expect(not ok, "58: robot one's autons registration linked into ROBOT TWO's Auton Tester PASSED")
        expect(not b.disallowed, f"58: the plant raised a warning, so the case proves the warning policy, not "
                                 f"step 8: {b.disallowed[:3]}")
        expect(any("links shulib::bench::registerRobotOneAutons" in p and "(step 8, ROB1-A)" in p
                   for p in problems),
               f"58: the failure did not name robot one's autons registration (step 8's robot half): {problems}")
        expect(any("carries 'src/team/robot_one_autons.cpp'" in p and "another robot's words" in p
                   and "(step 8, ROB1-A)" in p for p in problems),
               f"58: the failure did not name robot one's autons FILE PATH (step 8's robot half, the ELF's words — "
               f"the call site's elf_bytes): {problems}")


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

    # 12-13 (T4 Part 3, 2026-10-01): THE RETIRED LIBRARY BUILD. Until T4 Part 3, cases 12 and 13
    #     were the tank-library build's own — 12 its Makefile append dropped, 13 the real build
    #     alone. They RETIRED with the build, onto cases 10 and 11, which test the same two things
    #     for tank-drive: 12's intent (a dropped PROGRAM append is caught by the token-exact define
    #     check AND the beacon) lives on in 10, and 13's (the `…=tank` prefix inside a longer beacon
    #     must not false-fail, and the library graph must link — tank-drive links robot::Robot,
    #     runDriverControl and the protections since T4 Part 3) in 11. In their place, the
    #     retirement itself (header, RETIRED):
    # 12: SHULIB_PROGRAM_LIBRARY planted (a) in a src/ program, (b) in src/build_selection.hpp as
    #     an #error guard — a form that is LEGAL there for a live switch — and (c) in the host
    #     build's CMake: step 0 must FAIL each, naming the retirement (and (a) once, not also as a
    #     stray switch). Then from the build: (d) the Makefile's drive branch ALSO appending the
    #     retired define — step 5 must name it on the compile lines; (e) the tank-drive branch
    #     typing the retired beacon — step 6 must name it in the package.
    retired = "a RETIRED build switch"
    for where, form in ((SRC_PLANT, "#if defined(SHULIB_PROGRAM_LIBRARY)"),
                        (SELECTION_PATH, "#if defined(SHULIB_PROGRAM_LIBRARY)\n"
                                         "#error \"self-test plant: a retired switch's guard\"")):
        rel = os.path.relpath(where, REPO)
        body = "" if "#error" in form else "static int self_test_retired;\n"
        with _plant(where, f"\n{form}\n{body}#endif\n"):
            found = one_chain_problems()
            expect(any(rel in p and retired in p and "RETIRED at T4 Part 3" in p for p in found),
                   f"12: SHULIB_PROGRAM_LIBRARY in {rel} was not reported as RETIRED: {found}")
            expect(not any(rel in p and "names a build switch" in p for p in found),
                   f"12: the retired switch in {rel} was reported twice: {found}")
    with _plant(CMAKE_PLANT, "\ntarget_compile_definitions(shulib_tests PRIVATE "
                             "SHULIB_PROGRAM_LIBRARY)\n"):
        found = one_chain_problems()
        expect(any(os.path.relpath(CMAKE_PLANT, REPO) in p and retired in p for p in found),
               f"12: SHULIB_PROGRAM_LIBRARY in the host build's CMake was not reported: {found}")
    expect(one_chain_problems() == [], "12: a plant was not restored")
    drive_line = "override EXTRA_CXXFLAGS+=-DSHULIB_PROGRAM_DRIVE\n"
    with _replace(MAKEFILE, drive_line,
                  drive_line + "override EXTRA_CXXFLAGS+=-DSHULIB_PROGRAM_LIBRARY\n"):
        b = run_build("tank-drive")
        ok, problems = verdict("tank-drive", b)
        expect(not ok, "12: a tank-drive build carrying the retired define PASSED")
        expect(any("carries the RETIRED define -DSHULIB_PROGRAM_LIBRARY" in p for p in problems),
               f"12: the retired define on the compile lines was not named: {problems}")
    with _replace(SELECTION_PATH, 'kVariantBeacon[] = "shulib-robot-variant=tank-drive";',
                  'kVariantBeacon[] = "shulib-robot-variant=tank-library";'):
        b = run_build("tank-drive")
        ok, problems = verdict("tank-drive", b)
        expect(not ok, "12: a tank-drive build typing the retired beacon PASSED")
        expect(any("carries the RETIRED beacon 'shulib-robot-variant=tank-library'" in p
                   for p in problems),
               f"12: the retired beacon in the package was not named: {problems}")

    # 13: `make ROBOT=tank PROGRAM=library` -> make FAILS before it compiles anything, with the
    #     Makefile's $(error) naming the retirement and the right command — never a different
    #     program under the retired name (header, RETIRED).
    for d in ("bin", ".d"):
        shutil.rmtree(os.path.join(REPO, d), ignore_errors=True)
    p = subprocess.run(["make", "ROBOT=tank", "PROGRAM=library", "V=1"], cwd=REPO,
                       capture_output=True, text=True, timeout=600)
    said = ANSI.sub("", p.stdout + p.stderr)
    expect(p.returncode != 0, "13: `make ROBOT=tank PROGRAM=library` SUCCEEDED — the retired "
                              "build came back")
    expect("PROGRAM=library was retired at T4" in said and "PROGRAM=drive" in said,
           f"13: the failure did not name the retirement and the right command: {said[-400:]}")
    expect(not re.search(r"^Compiled ", said, flags=re.M)
           and not os.path.exists(os.path.join(REPO, "bin")),
           "13: make compiled something before refusing the retired build")

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
    #     tank-drive build silently becomes the tester, and the VARIANT BEACON must fail it:
    #     the tank-drive beacon absent, the tank one present. The structural define check must
    #     NOT be what fires (the define did land), nor the robot beacon (both branches are robot
    #     two's) — each detector is proven on its own. (On the tank-library branch until T4 Part
    #     3 retired it; re-pointed to tank-drive, its intent unchanged — case 10 drops the
    #     Makefile's append instead, so the define does NOT land there: not a copy.)
    # (ROB1-A: robot one's drive branch also reads SHULIB_PROGRAM_DRIVE, so the target names robot two's.)
    drive_branch = "defined(SHULIB_ROBOT_TANK_2026) && defined(SHULIB_PROGRAM_DRIVE)\ninline constexpr Program kProgram"
    with _replace(SELECTION_PATH, drive_branch,
                  "defined(SHULIB_ROBOT_TANK_2026) && defined(SHULIB_PROGRAM_DRIVE_BROKEN)\n"
                  "inline constexpr Program kProgram"):
        b = run_build("tank-drive")
        ok, problems = verdict("tank-drive", b)
        expect(not ok, "16: a broken #elif in the one chain PASSED the tank-drive build — "
                       "the beacon no longer vouches for the program")
        expect(any("beacon 'shulib-robot-variant=tank-drive' is NOT in" in p for p in problems),
               "16: the tank-drive beacon's absence was not caught")
        expect(any("tank build wearing a tank-drive label" in p for p in problems),
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
    # (T4 Part 3 re-pointed the plants on SHULIB_PROGRAM_LIBRARY, a switch it RETIRED, to live
    # switches — planted retired they would test the RETIRED list, case 12, not the one-chain rule.
    # Each keeps its file and its directive form, so none copies another plant.)
    plants = [
        (MAIN_CPP_PATH, "#if defined(SHULIB_PROGRAM_CALIBRATE)"),
        (SRC_PLANT, "#if defined(SHULIB_PROGRAM_AUTONTEST)"),
        (SRC_PLANT, "#ifdef SHULIB_ROBOT_TANK_2026"),
        (MAIN_CPP_PATH, "#ifndef SHULIB_PROGRAM_DRIVE"),
        (MAIN_CPP_PATH, "#if 0\n#elifdef SHULIB_ROBOT_XDRIVE_INVENTED"),
        (SRC_PLANT, "#if 0\n#elifndef SHULIB_PROGRAM_CALIBRATE"),
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
        (TEST_PLANT, "#if defined(SHULIB_PROGRAM_AUTONTEST)"),
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
            ("#if defined(SHULIB_PROGRAM_CALIBRATE)\nstatic int self_test_second_chain;\n#endif",
             "opens a SECOND chain"),
            ("#ifdef SHULIB_ROBOT_TANK_2026\nstatic int self_test_second_chain;\n#endif",
             "opens a SECOND chain"),
            ("#if 0\n#elifndef SHULIB_PROGRAM_AUTONTEST\nstatic int self_test_x;\n#endif",
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

    # 21: the tank-drive branch assigns the WRONG program (`kProgram = Program::BenchTests`)
    #     under the RIGHT beacon: the "shulib Drive" upload is silently the tester (HA-111's
    #     defect class). The one-chain rule and the program table pass (the branch still sets
    #     all three); step 0's CHAIN TABLE reports it from the source, and step 8 — proven here
    #     on its own, from the build — must fail it on both halves, with no beacon firing. (On the
    #     tank-library branch until T4 Part 3 retired it; re-pointed, its intent unchanged — case
    #     20 swaps main.cpp's CALLS in the bench build, not a branch's program: not a copy.)
    # (ROB1-A: robot one's drive branch sets Program::Drive too, so the target carries robot two's next line.)
    with _replace(SELECTION_PATH, "inline constexpr Program kProgram = Program::Drive;\n"
                                  "inline constexpr RobotId kRobotId = RobotId::RobotTwo;",
                  "inline constexpr Program kProgram = Program::BenchTests;\n"
                  "inline constexpr RobotId kRobotId = RobotId::RobotTwo;"):
        expect(one_chain_problems() == [] and program_table_problems() == [],
               "21: the plant tripped the one-chain rule or the program table")
        expect(any("sets kProgram = Program::BenchTests, but this gate's BUILD_PROGRAM says "
                   "Drive" in p for p in chain_table_problems()),
               f"21: the chain table did not report the wrong program: {chain_table_problems()}")
        b = run_build("tank-drive")
        ok, problems = verdict("tank-drive", b)
        expect(not ok, "21: a tank-drive build that is the tester PASSED")
        expect(any("links the BenchTests program's shulib::bench::runR3a" in p
                   for p in problems),
               f"21: the tester's function in the tank-drive build was not reported: {problems}")
        expect(any("does not link its shulib::bench::driveInitialize" in p for p in problems),
               f"21: Drive's missing boot was not reported: {problems}")
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

    # 24: THE WRONG ROBOT INSIDE A BRANCH (the ruling's case): the tank-drive branch sets
    #     `kRobotId = RobotId::BenchBot` under the right program and the right beacon. The build
    #     compiles, links and runs "shulib Drive" — over the bench bot's tables: no claws, and
    #     the driver file's controls bound to nothing. Step 0's chain table reports it from the
    #     source; from the package, ONLY the robot beacon can (the variant beacon is typed in the
    #     branch and step 8 checks programs): robot two's robot beacon absent, the bench bot's
    #     present, and nothing else firing. (On the tank-library branch until T4 Part 3.)
    with _replace(SELECTION_PATH,
                  "inline constexpr Program kProgram = Program::Drive;\n"
                  "inline constexpr RobotId kRobotId = RobotId::RobotTwo;\n",
                  "inline constexpr Program kProgram = Program::Drive;\n"
                  "inline constexpr RobotId kRobotId = RobotId::BenchBot;\n"):
        expect(one_chain_problems() == [] and program_table_problems() == [],
               "24: the plant tripped the one-chain rule or the program table")
        expect(any("sets kRobotId = RobotId::BenchBot, but this gate's BUILD_ROBOT says RobotTwo"
                   in p for p in chain_table_problems()),
               f"24: the chain table did not report the wrong robot: {chain_table_problems()}")
        b = run_build("tank-drive")
        ok, problems = verdict("tank-drive", b)
        expect(not ok, "24: a tank-drive build carrying the bench bot's tables PASSED")
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
    #     table; shulib Drive's mechanism service bound over the bench bot's mechanisms — re-pointed at
    #     MECH-A5 from its claw boot lines, which left the file (and at T4 Part 3 from the OLD Drive's
    #     ports from the bench bot's chassis table); Calibrate's facts mapping the bench bot's mechanisms, i.e. none — its
    #     claws become drive candidates; re-pointed at CAL1 Part B from the old program's drop call,
    #     deleted with it, to the facts that replaced it) are reported; a library header
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
            (DRIVE_PLANT, "st.mechanisms->bind(kMechanisms, ",
             "st.mechanisms->bind(mechanismTableFor(RobotId::BenchBot), "),
            (CALIBRATE_PLANT, "calibrateFactsFor(kChassis, kMechanisms)",
             "calibrateFactsFor(kChassis, mechanismTableFor(RobotId::BenchBot))")):
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
    legal = ("\nstatic_assert(clearOfItsDrive(kRobotTwoMechanisms, kRobotTwoChassis)\n"
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
                  "#if defined(SHULIB_PROGRAM_CALIBRATE)\n#endif\n"):
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
                 "SHULIB_PROGRAM_AUTONTEST)"):
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
                        (COMMON_PLANT, "#if defined(SHULIB_PROGRAM_CALIBRATE)")):
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
            # (T4 Part 3: on src/programs/teleop_claws.hpp until then, a file Part 3 deleted;
            # re-pointed to the new shulib Drive, a src/programs/ file that includes the same
            # src/common/ header once — the intent unchanged: a relative include in a program.)
            (DRIVE_PLANT, '#include "common/boot_source.hpp"\n',
             '#include "../common/boot_source.hpp"\n',
             'includes "../common/boot_source.hpp" by a relative path'),
            # (SRC1's review) the `./` form: it BUILDS (not beside its includer, so the -iquote src
            # root answers it), and it is not written from src/.
            (DRIVE_PLANT, '#include "common/boot_source.hpp"\n',
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

    # 28b (MECH-A5, the MECH brief's re-check T3): A ROBOT'S MECHANISM NAMESPACE, PER ROBOT (step 0;
    #     header). The robot namespaces are DERIVED from the mechanism table file (robot two's, with its
    #     list); then the brief's nine planted cases (its probe's out5/gate_cases.txt), each alone:
    #     REFUSED — a program stepping robot two's claw by its namespace, a program's `using namespace
    #     robot_two`, src/common re-opening the namespace, robot THREE's driver file naming robot two's
    #     list qualified and unqualified, a chassis table in robot two's own team file (case 28's rule,
    #     unchanged); PASSING — robot two's OWN second team file naming its list and namespace, the
    #     team's auton file (TEAM_FILE_ROBOT) re-stating its `using namespace robot_two`, and a program
    #     naming robot two's list inside a static_assert. A team file of no robot (src/team/<x>.hpp) is
    #     no exception; a robot added to the table file is covered the moment it exists (derived, not
    #     listed) — and its own team file may name its own list, never robot two's. Source-only; every
    #     plant restored.
    #     THE NAME, WHEREVER IT IS WRITTEN (MECH-A5's review): the rule is the namespace's name as a C++
    #     identifier, so seven more plants are REFUSED — an alias (`namespace r2 = robot_two;`), a
    #     `using namespace` split over two lines, one fully qualified at global scope, a re-open with its
    #     brace on the NEXT line, a qualified name split before its `::`, a `#define` of the name, and a
    #     file NAMED like robot two's team files (`robot_two_helpers.hpp`) but outside src/team/ (the
    #     own-team-file exception is src/team/'s only) — and one PASSES: identifiers that merely BEGIN
    #     with the name (`robot_twofold`, `robot_two_count`) are other names. (The first rule matched
    #     three spellings per physical line and passed the first four; mutants G10 — the src/team/
    #     check dropped — and G21 — the qualified `using namespace` dropped — passed every plant above.)
    #     THE TEXT AS THE COMPILER READS IT (MECH-A5's verification): REFUSED — the name split INSIDE it
    #     by a line splice (and with blanks after the backslash), the name between two digit-separated
    #     numbers on one line, the name after a raw string holding a quote, robot two's LIST split by a
    #     splice (the per-robot table rule reads the same text); PASSING — a `//` comment whose backslash
    #     runs it on over the next line, and the name only inside a multi-line raw string. And the reading
    #     directly: a splice joins with nothing on the line it starts, a spliced literal shifts no later
    #     line, a number keeps its digits. (Each REFUSED form compiled and passed the gate before.)
    #     Not planted, the rule's stated limit: a name the preprocessor assembles (`##`).
    namespaces = _robot_namespaces(REPO)
    # (ROB1-A, 2026-11-11: robot one's region is the table file's second robot namespace.)
    expect(namespaces == {"robot_two": {"kRobotTwoMechanisms"}, "robot_one": {"kRobotOneMechanisms"}},
           f"28b: the robot namespaces were not derived from the mechanism table file: {namespaces}")
    by_ns = "a robot's mechanism namespace"
    programs_dir = os.path.join(REPO, "src", "programs")
    common_dir = os.path.join(REPO, "src", "common")
    namespace_cases = (
        ("a program steps robot two's claw by its namespace", os.path.join(programs_dir, "zz_self_test.cpp"),
         '#include "robots/mechanism_table.hpp"\nvoid f() { auto& c = shulib::bench::robot_two::claw; (void)c; }\n',
         by_ns),
        ("a program says using namespace robot_two", os.path.join(programs_dir, "zz_self_test.cpp"),
         '#include "robots/mechanism_table.hpp"\nnamespace shulib::bench { using namespace robot_two; }\n', by_ns),
        ("src/common re-opens robot two's namespace", os.path.join(common_dir, "zz_self_test.hpp"),
         '#pragma once\n#include "robots/mechanism_table.hpp"\nnamespace shulib::bench::robot_two { inline int x = 0; }\n',
         by_ns),
        ("robot THREE's driver file names robot two's list", os.path.join(TEAM_DIR, "robot_three_driver.hpp"),
         '#pragma once\n#include "robots/mechanism_table.hpp"\n'
         'namespace shulib::bench { inline const auto& m = robot_two::kRobotTwoMechanisms; }\n', by_name),
        ("robot three's driver file names kRobotTwoMechanisms unqualified", os.path.join(TEAM_DIR, "robot_three_driver.hpp"),
         '#pragma once\n#include "robots/mechanism_table.hpp"\n'
         'namespace shulib::bench { using MT = decltype(kRobotTwoMechanisms); }\n', by_name),
        ("a chassis table in robot two's own team file", os.path.join(TEAM_DIR, "robot_two_extra.hpp"),
         '#pragma once\n#include "robots/chassis_table.hpp"\nnamespace shulib::bench { inline const auto& c = kRobotTwoChassis; }\n',
         by_name),
        ("a team file of NO robot names robot two's namespace", os.path.join(TEAM_DIR, "shared_helpers.hpp"),
         '#pragma once\n#include "robots/mechanism_table.hpp"\nnamespace shulib::bench { using namespace robot_two; }\n',
         by_ns),
        ("robot two's OWN second team file names its list and namespace", os.path.join(TEAM_DIR, "robot_two_presets.hpp"),
         '#pragma once\n#include "robots/mechanism_table.hpp"\n'
         'namespace shulib::bench::robot_two { inline const auto& m = kRobotTwoMechanisms; }\n', None),
        ("a program names robot two's list inside a static_assert", os.path.join(programs_dir, "zz_self_test.cpp"),
         '#include "robots/mechanism_table.hpp"\n'
         'static_assert(shulib::bench::robot_two::kRobotTwoMechanisms.count == 3);\n', None),
        ("a program ALIASES robot two's namespace", os.path.join(programs_dir, "zz_self_test.cpp"),
         '#include "robots/mechanism_table.hpp"\nnamespace shulib::bench {\nnamespace r2 = robot_two;\n'
         'void f() { (void)r2::clawPivot6; }\n}\n', by_ns),
        ("a program's using namespace split over two lines", os.path.join(programs_dir, "zz_self_test.cpp"),
         '#include "robots/mechanism_table.hpp"\nnamespace shulib::bench {\nusing namespace\n    robot_two;\n}\n',
         by_ns),
        ("a program's using namespace fully qualified at global scope", os.path.join(programs_dir, "zz_self_test.cpp"),
         '#include "robots/mechanism_table.hpp"\nusing namespace shulib::bench::robot_two;\n', by_ns),
        ("src/common re-opens robot two's namespace, the brace on the next line", os.path.join(common_dir, "zz_self_test.hpp"),
         '#pragma once\n#include "robots/mechanism_table.hpp"\nnamespace shulib::bench::robot_two\n{\ninline int x = 0;\n}\n',
         by_ns),
        ("a program's qualified name split before its ::", os.path.join(programs_dir, "zz_self_test.cpp"),
         '#include "robots/mechanism_table.hpp"\nnamespace shulib::bench {\nvoid f() { (void)robot_two\n::claw; }\n}\n',
         by_ns),
        ("a program makes robot two's namespace a macro", os.path.join(programs_dir, "zz_self_test.cpp"),
         '#include "robots/mechanism_table.hpp"\n#define SELF_TEST_R2 robot_two\n', by_ns),
        ("a src/common file NAMED like robot two's team files", os.path.join(common_dir, "robot_two_helpers.hpp"),
         '#pragma once\n#include "robots/mechanism_table.hpp"\n'
         'namespace shulib::bench { inline const auto& c = robot_two::claw; }\n', by_ns),
        ("a program's names that only BEGIN with robot two's namespace", os.path.join(programs_dir, "zz_self_test.cpp"),
         'namespace shulib::bench { inline int robot_twofold = 0; inline int robot_two_count = 0; }\n', None),
        # THE TEXT AS THE COMPILER READS IT (MECH-A5's verification; each REFUSED form compiles, g++ and
        # arm-none-eabi-g++ -std=gnu++20 — the verification's probe):
        ("a program's name split INSIDE it by a line splice", os.path.join(programs_dir, "zz_self_test.cpp"),
         '#include "robots/mechanism_table.hpp"\nnamespace shulib::bench {\nvoid f() { (void)robot_\\\ntwo::claw; }\n}\n',
         by_ns),
        ("the same splice with blanks after its backslash (GCC splices them too)", os.path.join(programs_dir, "zz_self_test.cpp"),
         '#include "robots/mechanism_table.hpp"\nnamespace shulib::bench {\nvoid f() { (void)robot_\\  \ntwo::claw; }\n}\n',
         by_ns),
        ("a program names robot two between two digit-separated numbers on one line", os.path.join(programs_dir, "zz_self_test.cpp"),
         "#include \"robots/mechanism_table.hpp\"\nnamespace shulib::bench {\n"
         "inline int f() { int a = 1'000; (void)robot_two::claw; return a + 2'000; }\n}\n", by_ns),
        ("a program names robot two after a raw string holding a quote", os.path.join(programs_dir, "zz_self_test.cpp"),
         '#include "robots/mechanism_table.hpp"\nnamespace shulib::bench {\n'
         'inline const char* r = R"(")"; inline const auto& c = robot_two::claw; inline const char* s = "";\n}\n', by_ns),
        ("a program's per-robot LIST split inside its name by a line splice", os.path.join(programs_dir, "zz_self_test.cpp"),
         '#include "robots/mechanism_table.hpp"\nnamespace shulib::bench {\ninline const auto& m = robot_two::kRobotTwo\\\n'
         'Mechanisms;\n}\n', by_name),
        ("a // comment ending in a backslash runs on over the next line (a comment to the compiler)",
         os.path.join(programs_dir, "zz_self_test.cpp"),
         '// a note \\\n(void)shulib::bench::robot_two::claw; -- still the comment\nnamespace shulib::bench { inline int x = 0; }\n',
         None),
        ("robot two's name only inside a multi-line raw string", os.path.join(programs_dir, "zz_self_test.cpp"),
         'inline const char* kSelfTestRaw = R"x(\nshulib::bench::robot_two::claw ")"\n)x";\n', None),
    )
    # The reading itself, directly (MECH-A5's verification): a splice joins with NOTHING and its line's
    # number is the line it starts on; _logical_lines splices text it is handed raw; a literal a splice
    # continued keeps every later line's number (before, the stripped literal lost its line break and
    # every later line was reported one short — test/pros_shim/main.h's #error); a number is whole.
    joined = _logical_lines("#if A \\\n  || B\nx robot_\\\ntwo y\nz")
    expect(joined == [(1, "#if A   || B"), (2, ""), (3, "x robot_two y"), (4, ""), (5, "z")],
           f"28b: _logical_lines did not join a splice with nothing, on the line it starts: {joined}")
    stripped = _strip_comments('#error "a \\\nb"\n#endif\nint k = 1\'0; int j = 2\'0;\n', literals=True)
    expect(_logical_lines(stripped)[2:4] == [(3, "#endif"), (4, "int k = 1'0; int j = 2'0;")],
           f"28b: a spliced literal shifted the later lines, or a number lost its digits: {_logical_lines(stripped)}")
    for title, path, content, refused in namespace_cases:
        rel = os.path.relpath(path, REPO)
        with _newfile(path, content):
            found = [p for p in one_chain_problems() if rel in p]
            if refused is None:
                expect(found == [], f"28b: {title} — expected to PASS, reported: {found}")
            else:
                expect(any(refused in p for p in found), f"28b: {title} — expected REFUSED ({refused}): {found}")
    # The team's auton file, its robot by TEAM_FILE_ROBOT: its own `using namespace robot_two` is legal…
    autons = os.path.join(TEAM_DIR, "autons.cpp")
    rel_autons = os.path.relpath(autons, REPO)
    with _plant(autons, "\nnamespace shulib::bench { using namespace robot_two; }\n"):
        expect([p for p in one_chain_problems() if rel_autons in p] == [],
               "28b: the team's auton file (TEAM_FILE_ROBOT: robot two's) was refused its own namespace")
    # …and a robot added to the table file is another robot to the auton file: refused there.
    with _plant(MECHANISM_TABLE_PATH, "\nnamespace shulib::bench::robot_three {\n"
                                      "inline constexpr MechanismTable kRobotThreeMechanisms = mechanism::mechanisms();\n"
                                      "}\n"):
        expect(_robot_namespaces(REPO).get("robot_three") == {"kRobotThreeMechanisms"},
               f"28b: a NEW robot's namespace was not derived: {_robot_namespaces(REPO)}")
        with _plant(autons, "\nnamespace shulib::bench { using namespace robot_three; }\n"):
            found = [p for p in one_chain_problems() if rel_autons in p]
            expect(any("names robot_three" in p and by_ns in p for p in found),
                   f"28b: robot two's auton file naming robot THREE's namespace was not reported: {found}")
        # …and robot THREE's own team file, now that robot three exists: its OWN list passes, robot
        # TWO's list is refused — the exception is the file's robot's lists, never every robot's (the
        # mutation that widened it to every robot's lists survived the nine cases above: MECH-A5's M34).
        three = os.path.join(TEAM_DIR, "robot_three_driver.hpp")
        rel_three = os.path.relpath(three, REPO)
        with _newfile(three, '#pragma once\n#include "robots/mechanism_table.hpp"\n'
                             'namespace shulib::bench::robot_three { inline const auto& m = kRobotThreeMechanisms; }\n'):
            found = [p for p in one_chain_problems() if rel_three in p]
            expect(found == [], f"28b: robot three's own team file was refused its own list: {found}")
        with _newfile(three, '#pragma once\n#include "robots/mechanism_table.hpp"\n'
                             'namespace shulib::bench::robot_three { inline const auto& m = kRobotTwoMechanisms; }\n'):
            found = [p for p in one_chain_problems() if rel_three in p]
            expect(any(by_name in p and "kRobotTwoMechanisms" in p for p in found),
                   f"28b: robot three's team file naming robot TWO's list was not refused: {found}")
    expect(one_chain_problems() == [], "28b: a plant was not restored")

    # 29: an include left in the OLD form in src/programs/ (`#include "chassis_table.hpp"`) -> the
    #     BUILD fails, naming the file: nothing answers that spelling from a subfolder — not the
    #     program's own folder, not include/, not src/ — so a half-done move can never compile
    #     against a stale copy. (Step 0 does not report it: the include resolves nowhere, which is
    #     the compiler's to say; case 28 is the form that DOES build.)
    #     (T4 Part 3: the include re-pointed from robots/chassis_table.hpp, which the new
    #     drive_program.cpp no longer includes, to robots/mechanism_table.hpp, which it does, once.)
    with _replace(DRIVE_PLANT, '#include "robots/mechanism_table.hpp"\n',
                  '#include "mechanism_table.hpp"\n'):
        b = run_build("bench")
        ok, problems = verdict("bench", b)
        expect(not ok, "29: an old-form include in src/programs/ BUILT")
        expect(any("drive_program.cpp" in p and "mechanism_table.hpp" in p for p in problems),
               f"29: the failure did not name the file and the include: {problems}")

    # 30: THE PATHS A PROGRAM PRINTS (SRC1's review, 2026-09-27; header). The real tree reports
    #     nothing — its printed src/ paths all exist, some followed by a sentence's full stop,
    #     which is not part of the path. A path that names nothing is reported: in a program's
    #     boot line (the review's surviving mutation, `src/robot/` for `src/robots/`, was in the OLD
    #     shulib Drive's refusal, a literal T4 Part 3 deleted; re-pointed to the new shulib Drive's
    #     TYPED TABLE boot line), split across two adjacent literals (the compiler joins them, so the check
    #     does), in a library header, and inside a raw string holding a quote (MECH-A5's verification:
    #     the old reading took it for two plain literals with the path between them as code); the same
    #     dead path in a comment is not (a comment reaches no robot). Source-only; every plant restored.
    expect(printed_path_problems() == [],
           f"30: the real tree's printed paths do not all exist: {printed_path_problems()}")
    rel = os.path.relpath(DRIVE_PLANT, REPO)
    with _replace(DRIVE_PLANT, '"TYPED TABLE (src/robots/chassis_table.hpp)"',
                  '"TYPED TABLE (src/robot/chassis_table.hpp)"'):
        found = printed_path_problems()
        expect(any(rel in p and 'prints "src/robot/chassis_table.hpp"' in p for p in found),
               f"30: a dead path in {rel}'s boot line was not reported: {found}")
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
    # A raw string's text is read too, whole (MECH-A5's verification: a raw string holding a quote was
    # read as two plain literals with the path between them as CODE — this plant passed the old gate).
    rel = os.path.relpath(COMMON_PLANT, REPO)
    with _plant(COMMON_PLANT, '\ninline constexpr const char* kSelfTestRaw = R"x(fill "src/robot/chassis_table.hpp")x";\n'):
        found = printed_path_problems()
        expect(any(rel in p and 'prints "src/robot/chassis_table.hpp"' in p for p in found),
               f"30: a dead path inside a raw string in {rel} was not reported: {found}")
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

    # ── COMP-b (2026-09-30): the eighth build, "shulib Competition" ─────────────────────────
    # 34: the Makefile's PROGRAM=competition append DROPPED (`make ROBOT=tank PROGRAM=competition`
    #     silently builds the tester) -> FAIL on the token-exact structural check naming the define
    #     AND on the beacon (a tank build wearing the tank-competition label) — cases 14's and 31's
    #     shape. The program a team uploads for a MATCH must never be an upload that is silently
    #     Bench Tests: at a field it would sit through autonomous and driver control doing nothing.
    competition_line = "override EXTRA_CXXFLAGS+=-DSHULIB_PROGRAM_COMPETITION\n"
    with _replace(MAKEFILE, competition_line, "# (self-test plant: competition append dropped)\n"):
        b = run_build("tank-competition")
        ok, problems = verdict("tank-competition", b)
        expect(not ok, "34: PROGRAM=competition with its Makefile append dropped PASSED — the "
                       "silent-no-op class is back for the competition program")
        expect(any("whole token -DSHULIB_PROGRAM_COMPETITION" in p for p in problems),
               "34: the failure did not name the missing PROGRAM=competition define")
        expect(any("beacon" in p and "NOT in" in p for p in problems),
               "34: the tank-competition beacon's absence was not caught")
        expect(any("tank build wearing a tank-competition label" in p for p in problems),
               "34: the tank beacon's presence (NUL-terminated match) was not caught")

    # 35: the real tank-competition build, alone -> PASS (the `…=tank` prefix inside
    #     `…=tank-competition` must not false-fail, and the competition program must link — its
    #     five entry functions linked, no other program's).
    b = run_build("tank-competition")
    ok, problems = verdict("tank-competition", b)
    expect(ok, f"35: the real tank-competition build FAILED the gate: {problems}")

    # ── SER1 (2026-10-08): step 9, ONE-CALL WRITES — no src/ object writes through stdout ─────
    # Each plant is an EXTERNAL function (declared first, so no warning and no dead code), which
    # its object keeps whatever --gc-sections later does to the package — the reason step 9 reads
    # objects, not the ELF.
    # 36: `printf("x\n")` in a src/ file -> FAIL naming the object AND `puts`: the compiler
    #     rewrites it, invisibly in the source, into puts("x") — two writes in newlib, into
    #     stdout's shared buffer (A4 register HA-189). The bench tester's emit() was exactly this.
    with _plant(SRC_PLANT, "\nvoid ser1SelfTestPrintf();\n"
                           "void ser1SelfTestPrintf() { std::printf(\"x\\n\"); }\n"):
        b = run_build("bench")
        ok, problems = verdict("bench", b)
        expect(not ok, "36: a printf(\"x\\n\") in src/ PASSED the stdio check (step 9)")
        expect(any("bench_r3a.cpp.o" in p and "`puts`" in p and "(step 9)" in p
                   for p in problems),
               f"36: the failure did not name bench_r3a.cpp.o and `puts`: {problems}")
    # 37: `fflush(stdout)` -> FAIL naming _impure_ptr (fflush itself is allowed — the card needs
    #     it; what makes this stdout's is the _impure_ptr reference every use of stdout compiles to).
    with _plant(SRC_PLANT, "\nvoid ser1SelfTestFlush();\n"
                           "void ser1SelfTestFlush() { (void)std::fflush(stdout); }\n"):
        b = run_build("bench")
        ok, problems = verdict("bench", b)
        expect(not ok, "37: a fflush(stdout) in src/ PASSED the stdio check (step 9)")
        expect(any("bench_r3a.cpp.o" in p and "_impure_ptr" in p for p in problems),
               f"37: the failure did not name bench_r3a.cpp.o and _impure_ptr: {problems}")
        expect(not any("`fflush`" in p for p in problems),
               f"37: fflush ITSELF was reported — the card's flush would fail too: {problems}")
    # 38: the NEGATIVE CONTROL — `fwrite` to a FILE* PARAMETER (a card file's shape) -> PASS:
    #     the check bans stdout, not stdio.
    with _plant(SRC_PLANT, "\nvoid ser1SelfTestCard(std::FILE* f);\n"
                           "void ser1SelfTestCard(std::FILE* f) "
                           "{ (void)std::fwrite(\"x\", 1, 1, f); (void)std::fflush(f); }\n"):
        b = run_build("bench")
        ok, problems = verdict("bench", b)
        expect(ok, f"38: an fwrite/fflush to a FILE* parameter FAILED the gate: {problems}")
        # 39: a src/ TU with NO object -> FAIL (a check that read nothing must not pass). Uses
        #     38's bin/ (no extra build): one object removed, step 9 alone re-run.
        os.remove(os.path.join(REPO, "bin", "programs", "bench_r3a.cpp.o"))
        found = stdio_problems("bench", find_nm())
        expect(any("no object for" in p and "bench_r3a.cpp.o" in p for p in found),
               f"39: a src/ TU without its object was not reported: {found}")

    # ── SER1's review (2026-10-15): step 9's LIST, pinned to the libraries and the forms ──────
    # 40: the list against the libraries (source-only, two nm reads). The first list (A1's) lacked
    #     viprintf, putchar_unlocked and the wide printers, and a real bench build calling any of
    #     them PASSED (the review). The real list must be clean; the libc.a read must find the
    #     family (a check that read nothing would pass); a list missing one printer, or naming one
    #     no library defines, must be REPORTED by name.
    nm = find_nm()
    writers, list_problems = stdio_list_problems(nm)
    expect(list_problems == [], f"40: the real STDIO_BANNED is not clean: {list_problems}")
    expect({"printf", "puts", "putchar", "viprintf", "putchar_unlocked", "wprintf", "putwchar",
            "psignal", "_printf_r"} <= set(writers),
           f"40: the libc.a read did not find the printer family: {writers}")
    _w, gap = stdio_list_problems(nm, banned=STDIO_BANNED - {"viprintf"})
    expect(any(p.startswith("viprintf:") and "not in STDIO_BANNED" in p for p in gap),
           f"40: a list without viprintf was not reported: {gap}")
    _w, typo = stdio_list_problems(nm, banned=(STDIO_BANNED - {"_ZSt5wcout"}) | {"_ZSt5wcoutt"})
    expect(any(p.startswith("_ZSt5wcoutt:") and "misspelt" in p for p in typo),
           f"40: a misspelt name was not reported: {typo}")
    #     And `check` RUNS it: cmd_check with a list missing viprintf must FAIL on [stdio-list].
    #     VARIANTS is emptied for the call so no build runs (the chain-table check then reports the
    #     empty build list too — expected; only the [stdio-list] line is asserted).
    saved = (STDIO_BANNED, VARIANTS)
    rc, said = _check_with(STDIO_BANNED - {"viprintf"}, ())
    expect(rc == 1 and "[stdio-list] viprintf:" in said,
           f"40: `check` did not run the list check (rc {rc}): {said[-600:]}")
    expect(STDIO_BANNED is saved[0] and VARIANTS is saved[1], "40: the list was not restored")
    # 41: the forms a student writes, in ONE real build — each must FAIL by its own name: printf
    #     WITH arguments (`printf`, not puts: no _impure_ptr or puts to fall back on, so the name
    #     leaving the list would blind the gate — the review's GM1 cut the list to two names and
    #     cases 36-39 stayed green), `std::cout << …` (`_ZSt4cout`), and the review's four that
    #     passed the first list: viprintf, putchar_unlocked, std::wprintf and std::wcout (plus
    #     putwchar, psignal and `&__sf[1]`). Every plant is an external function (no warning).
    forms = (
        "\n#include <cstdarg>\n#include <csignal>\n#include <cwchar>\n#include <iostream>\n"
        "#include <signal.h>\n"
        "extern \"C\" std::FILE __sf[3];\n"
        "void ser1ReviewPrintf(int n);\n"
        "void ser1ReviewPrintf(int n) { std::printf(\"x %d\\n\", n); }\n"
        "void ser1ReviewCout(int n);\n"
        "void ser1ReviewCout(int n) { std::cout << \"x\" << n; }\n"
        "void ser1ReviewViprintf(int n, ...);\n"
        "void ser1ReviewViprintf(int n, ...) { va_list a; va_start(a, n); "
        "(void)viprintf(\"x %d\\n\", a); va_end(a); }\n"
        "void ser1ReviewPutcharUnlocked(int c);\n"
        "void ser1ReviewPutcharUnlocked(int c) { (void)putchar_unlocked(c); }\n"
        "void ser1ReviewWprintf(int n);\n"
        "void ser1ReviewWprintf(int n) { (void)std::wprintf(L\"x %d\\n\", n); }\n"
        "void ser1ReviewWcout(int n);\n"
        "void ser1ReviewWcout(int n) { std::wcout << L\"x\" << n; }\n"
        "void ser1ReviewPutwchar(wchar_t c);\n"
        "void ser1ReviewPutwchar(wchar_t c) { (void)std::putwchar(c); }\n"
        "void ser1ReviewPsignal(int s);\n"
        "void ser1ReviewPsignal(int s) { psignal(s, \"x\"); }\n"
        "void ser1ReviewSf();\n"
        "void ser1ReviewSf() { (void)std::fflush(&__sf[1]); }\n")
    with _plant(SRC_PLANT, forms):
        b = run_build("bench")
        ok, problems = verdict("bench", b)
        expect(not ok, "41: printf/cout/viprintf/putchar_unlocked/wprintf/wcout/putwchar/psignal/"
                       "__sf in src/ PASSED the stdio check (step 9)")
        expect(not b.disallowed, f"41: the plants raised a warning, so the case proves the "
                                 f"warning policy, not step 9: {b.disallowed[:3]}")
        for sym in ("`printf`", "`_ZSt4cout`", "`viprintf`", "`putchar_unlocked`", "`wprintf`",
                    "`_ZSt5wcout`", "`putwchar`", "`psignal`", "`__sf`"):
            expect(any("bench_r3a.cpp.o" in p and sym in p and "(step 9)" in p
                       for p in problems),
                   f"41: the failure did not name bench_r3a.cpp.o and {sym}: {problems}")

    # ── TUNE1-B1 (2026-11-10): the ninth build, "shulib Tune" — the first ROBOT-GENERIC row ──────────
    # 42: the Makefile's PROGRAM=tune append DROPPED (`make ROBOT=tank PROGRAM=tune` silently builds the
    #     tester) -> FAIL on the token-exact structural check AND on the beacon, and step 0's [tune-row]
    #     check reports the row without its append.
    tune_line = "override EXTRA_CXXFLAGS+=-DSHULIB_PROGRAM_TUNE\n"
    with _replace(MAKEFILE, tune_line, "# (self-test plant: tune append dropped)\n"):
        expect(any("no `else ifeq ($(PROGRAM),tune)` row" in p for p in tune_row_problems()),
               "42: step 0 did not report the tune row without its -DSHULIB_PROGRAM_TUNE append")
        b = run_build("tank-tune")
        ok, problems = verdict("tank-tune", b)
        expect(not ok, "42: PROGRAM=tune with its Makefile append dropped PASSED — the silent-no-op "
                       "class is back for the Tune program")
        expect(any("whole token -DSHULIB_PROGRAM_TUNE" in p for p in problems),
               "42: the failure did not name the missing PROGRAM=tune define")
        expect(any("tank build wearing a tank-tune label" in p for p in problems),
               "42: the tank-tune beacon's absence was not caught")

    # 43: the real tank-tune build, alone -> PASS (the `…=tank` prefix inside `…=tank-tune` must not
    #     false-fail; the Tune ELF links live tuning and no chassis; step 0's [tune-row] is clean).
    b = run_build("tank-tune")
    ok, problems = verdict("tank-tune", b)
    expect(ok, f"43: the real tank-tune build FAILED the gate: {problems}")
    expect(not tune_row_problems(), f"43: step 0's [tune-row] check is not clean on the real tree: "
                                    f"{tune_row_problems()}")

    # 44: LIVE TUNING PLANTED IN A MATCH PROGRAM (the MECH-S/TUNE1 brief's TB6 / MB5: "a match runs exactly
    #     the robot file's numbers") — the competition program links a LiveTuning member, reached from its
    #     initialize() behind a condition no compiler can fold -> step 8 FAILS naming the live-tuning symbol.
    comp_plant = os.path.join(REPO, "src", "programs", "competition_program.cpp")

    def plant_tuning(text):
        text = text.replace('#include "robots/mechanism_table.hpp"\n',
                            '#include "robots/mechanism_table.hpp"\n'
                            '#include "shulib/mechanism/live_tuning.hpp"  // (self-test plant)\n', 1)
        return text.replace(
            "void competitionInitialize(const char* variantBeacon, const char* competitionStatusBits) {\n",
            "void competitionInitialize(const char* variantBeacon, const char* competitionStatusBits) {\n"
            "    static volatile bool tunePlantNever = false;  // (self-test plant)\n"
            "    static shulib::mechanism::LiveTuning tunePlant{kMechanisms};  // (self-test plant)\n"
            "    if (tunePlantNever) { (void)tunePlant.reset(0); }  // (self-test plant)\n", 1)

    with _transform(comp_plant, plant_tuning):
        b = run_build("tank-competition")
        ok, problems = verdict("tank-competition", b)
        expect(not ok, "44: live tuning linked into the COMPETITION program PASSED step 8")
        expect(not b.disallowed, f"44: the plant raised a warning, so the case proves the warning "
                                 f"policy, not step 8: {b.disallowed[:3]}")
        expect(any("live-tuning symbol" in p and "LiveTuning::" in p and "(step 8)" in p
                   for p in problems),
               f"44: the failure did not name a LiveTuning symbol or text (step 8): {problems}")

    # 45: TB12 — `make ROBOT=bench PROGRAM=tune` and `make ROBOT=xdrive PROGRAM=tune` (robots that declare
    #     no mechanism) -> make FAILS before it compiles anything, the Makefile's $(error) naming why.
    for robot_value in ("bench", "xdrive"):
        for d in ("bin", ".d"):
            shutil.rmtree(os.path.join(REPO, d), ignore_errors=True)
        p = subprocess.run(["make", f"ROBOT={robot_value}", "PROGRAM=tune", "V=1"], cwd=REPO,
                           capture_output=True, text=True, timeout=600)
        said = ANSI.sub("", p.stdout + p.stderr)
        expect(p.returncode != 0, f"45: `make ROBOT={robot_value} PROGRAM=tune` SUCCEEDED — a Tune with "
                                  f"nothing to tune")
        expect("PROGRAM=tune needs a ROBOT whose mechanism table is not empty" in said
               and f"'{robot_value}' declares no mechanism" in said,
               f"45: the failure did not name why for {robot_value}: {said[-400:]}")
        expect(not re.search(r"^Compiled ", said, flags=re.M)
               and not os.path.exists(os.path.join(REPO, "bin")),
               f"45: make compiled something before refusing ROBOT={robot_value} PROGRAM=tune")

    # 46: MECHANISM_ROBOTS out of step with the table file -> step 0's [tune-row] FAILS: a robot with
    #     an EMPTY table listed (bench), and the robot with mechanisms left out (tank).
    with _replace(MAKEFILE, "MECHANISM_ROBOTS:=tank one\n", "MECHANISM_ROBOTS:=tank one bench\n"):
        expect(any("names 'bench'" in p for p in tune_row_problems()),
               "46: MECHANISM_ROBOTS naming the bench bot (no mechanisms) PASSED step 0")
    with _replace(MAKEFILE, "MECHANISM_ROBOTS:=tank one\n", "MECHANISM_ROBOTS:=\n"):
        problems = tune_row_problems()
        expect(any("robot 'tank' has a mechanism table" in p for p in problems),
               f"46: MECHANISM_ROBOTS without robot two PASSED step 0: {problems}")

    # 47: the tune row NAMES A ROBOT (a robot-two-only row — the brief's re-check #10 forbids it) ->
    #     step 0's [tune-row] FAILS.
    with _replace(MAKEFILE, "else ifeq ($(PROGRAM),tune)\n",
                  "else ifeq ($(PROGRAM),tune)\nifneq ($(ROBOT),tank)\n"
                  "$(error self-test plant: a robot-two-only tune row)\nendif\n"):
        expect(any("names NO robot" in p for p in tune_row_problems()),
               "47: a tune row comparing $(ROBOT) with a robot's name PASSED step 0")
    #     ... and the same robot-two-only row in the forms the first pattern missed (the review round,
    #     2026-11-11): a $(filter) of the robot, a quoted ifneq, a ${ROBOT} — each FAILS step 0.
    for form in ("ifeq ($(filter tank,$(ROBOT)),)\n", 'ifneq "$(ROBOT)" "tank"\n', "ifneq (${ROBOT},tank)\n"):
        with _replace(MAKEFILE, "else ifeq ($(PROGRAM),tune)\n",
                      "else ifeq ($(PROGRAM),tune)\n" + form +
                      "$(error self-test plant: a robot-two-only tune row)\nendif\n"):
            expect(any("names NO robot" in p for p in tune_row_problems()),
                   f"47: a tune row naming a robot as {form.strip()!r} PASSED step 0")

    # 48: THE BACKSTOP behind the Makefile's list: the list wrongly accepting a robot with no Tune branch
    #     (xdrive) -> the build FAILS at src/build_selection.hpp's guard, naming why — never a Tune upload
    #     that is another program.
    with _replace(MAKEFILE, "MECHANISM_ROBOTS:=tank one\n", "MECHANISM_ROBOTS:=tank one xdrive\n"):
        for d in ("bin", ".d"):
            shutil.rmtree(os.path.join(REPO, d), ignore_errors=True)
        p = subprocess.run(["make", "ROBOT=xdrive", "PROGRAM=tune", "V=1"], cwd=REPO,
                           capture_output=True, text=True, timeout=600)
        said = ANSI.sub("", p.stdout + p.stderr)
        expect(p.returncode != 0, "48: ROBOT=xdrive PROGRAM=tune BUILT with no Tune branch in the chain")
        expect("PROGRAM=tune needs a ROBOT whose mechanism table is not empty -- this ROBOT has no Tune "
               "branch" in said, f"48: the failure was not the selection header's guard: {said[-400:]}")

    # 49: step 8's live-tuning half — EVERY detector of tuning_problems(), on SYNTHETIC ELF contents (the
    #     review round, 2026-11-11: cases 43 and 44 reach two of its five; a detector deleted left every
    #     expectation unchanged). Pure: no build. A Tune ELF linking a chassis or a Robot; a Tune ELF with
    #     no live tuning (and the precondition BYTES counting as live tuning); a match ELF linking a
    #     LiveTuning symbol or its bytes; a match ELF linking no chassis; and the clean ones PASS.
    chassis_name = "shulib::chassis::Chassis::tick()"
    robot_name = "shulib::robot::Robot::Robot()"
    tuning_name = "shulib::mechanism::LiveTuning::apply(unsigned int, shulib::mechanism::Tunable, double)"
    tuning_bytes = b"\0LiveTuning::reset: attach() the service first\0"
    for name, what in ((chassis_name, "shulib::chassis::"), (robot_name, "shulib::robot::Robot::")):
        p49 = tuning_problems("tank-tune", [tuning_name, name])
        expect(any("never builds the drive" in x and what in x and "(step 8)" in x for x in p49),
               f"49: a Tune ELF linking {what} PASSED step 8: {p49}")
    p49 = tuning_problems("tank-tune", ["shulib::bench::runTune()"])
    expect(any("links NO live-tuning symbol" in x for x in p49),
           f"49: a Tune ELF with no live tuning PASSED step 8: {p49}")
    expect(not tuning_problems("tank-tune", ["shulib::bench::runTune()"], tuning_bytes),
           "49: a Tune ELF whose live tuning is inlined (its precondition bytes only) FAILED step 8")
    expect(not tuning_problems("tank-tune", [tuning_name]), "49: a clean Tune ELF FAILED step 8")
    p49 = tuning_problems("tank-competition", [chassis_name, tuning_name])
    expect(any("live-tuning symbol" in x and "LiveTuning::apply" in x for x in p49),
           f"49: a Competition ELF linking a LiveTuning member PASSED step 8: {p49}")
    p49 = tuning_problems("tank-drive", [chassis_name], tuning_bytes)
    expect(any("live-tuning symbol" in x and "a LiveTuning precondition's message" in x for x in p49),
           f"49: a Drive ELF carrying LiveTuning's precondition bytes PASSED step 8: {p49}")
    for match in ("tank-competition", "tank-drive", "tank-autontest"):
        p49 = tuning_problems(match, ["shulib::bench::runCompetition()"])
        expect(any("links no chassis symbol" in x for x in p49),
               f"49: a {match} ELF linking no chassis PASSED step 8: {p49}")
        expect(not tuning_problems(match, [chassis_name]), f"49: a clean {match} ELF FAILED step 8")
    expect(not tuning_problems("tank-calibrate", ["shulib::bench::runCalibrate()"]),
           "49: Calibrate (no chassis program) FAILED step 8's chassis control")
    # ── ROB1-A (2026-11-11): robot one's builds, the facts proof, the step-0 rules and the stamp ──────────
    ledger_path = os.path.join(REPO, FACTS_LEDGER)
    competition_plant = os.path.join(REPO, "src", "programs", "competition_program.cpp")
    autons_plant = os.path.join(TEAM_DIR, "autons.cpp")

    # 50 ([R11]): robot one's NAMESPACE named where only its own files may — a program, robot two's team file,
    #     and the facts LEDGER (which lives in robot_one_facts, a different identifier, on purpose) -> step 0
    #     FAILS each by name; the real tree (the ledger's robot_one_facts; the driver panel's static_assert,
    #     which reaches robot one's table through mechanismTableFor(RobotId::RobotOne)) passes.
    for plant_path, rel in ((DRIVE_PLANT, os.path.join("src", "programs", "drive_program.cpp")),
                            (autons_plant, os.path.join("src", "team", "autons.cpp")),
                            (ledger_path, FACTS_LEDGER)):
        with _plant(plant_path, "\nnamespace gate_rob1_alias = shulib::bench::robot_one;\n"):
            problems = one_chain_problems()
            expect(any(p.startswith(rel + ":") and "names robot_one" in p for p in problems),
                   f"50: {rel} naming robot_one PASSED step 0 (the per-robot namespace rule, [R11]): "
                   f"{problems[:3]}")
    expect(not one_chain_problems(), f"50: the real tree fails step 0: {one_chain_problems()[:3]}")

    # 51 (ROB1-A's step-0 rule): a program naming a robot's OWN driver or autons — RobotTwoDriver,
    #     RobotOneDriver, registerAutons, registerRobotOneAutons — outside src/common/this_robot.hpp ->
    #     FAIL by name; this_robot.hpp itself (it names all four) passes.
    for name in TEAM_SELECTION_NAMES:
        with _plant(competition_plant, f"\nnamespace {{ [[maybe_unused]] constexpr auto gateRob1 = &shulib::bench::{name}; }}\n"
                    if name.startswith("register") else
                    f"\nusing GateRob1Driver = shulib::bench::{name};\n"):
            problems = one_chain_problems()
            expect(any("competition_program.cpp" in p and f"names {name}" in p for p in problems),
                   f"51: a program naming {name} PASSED step 0 (choosing a robot outside the chain): "
                   f"{problems[:3]}")
    expect(not _team_selection_problems(REPO, THIS_ROBOT_FILE),
           "51: src/common/this_robot.hpp — the one file that may — FAILS the rule")

    # 52 ([R4]): a TYPED fact in the repository's ledger with the gate copy's source, or with none -> step 0
    #     FAILS naming the fact (the ledger's own check refuses both at make, too — compile refusals s74/s75).
    camera = 'owed(3, Stage::Recorded, "the AI Vision camera\'s port")'
    for source, word in (('"gate copy -- same as the placeholder, checked"', "gate copy"), ('""', "no source")):
        with _replace(ledger_path, camera,
                      f'typed(3, 4, Stage::Recorded, "the AI Vision camera\'s port", {source})'):
            problems = ledger_source_problems()
            expect(any("the AI Vision camera's port" in p and word in p for p in problems),
                   f"52: a typed fact with {source} as its source PASSED step 0: {problems}")
    expect(not ledger_source_problems(), f"52: the real ledger fails step 0: {ledger_source_problems()}")

    # 53 (T4 — the facts proof's real-tree half sees a WRONG set): the ledger's OWED check with its
    #     `needs(kProgram, …)` dropped (the brief's GM10) — a program then refuses facts that are not its own
    #     -> FAIL naming them; and the real build refuses exactly its own owed set -> PASS. Since 2026-11-16 robot
    #     one's Bench Tests owes nothing (its Stage A all typed), so the case uses Drive: with needs() dropped it
    #     also refuses the RECORDED facts (the camera's), which no program needs. FOLLOW-UP: plant an owed fact so
    #     this case holds once every fact is typed.
    needs_term = "::shulib::bench::robot_one_facts::needs(::shulib::bench::kProgram, (fact).needed)"
    with _replace(ledger_path, needs_term, "true /* (self-test plant: needs dropped) */"):
        b = run_build("one-drive")
        ok, problems, refused = facts_verdict("one-drive", b)
        expect(not ok and not refused, "53: robot one's Drive refusing a RECORDED fact too PASSED")
        expect(any("named and not owed by this program" in p and "the AI Vision camera's port" in p
                   for p in problems), f"53: the failure did not name the extra RECORDED fact: {problems}")
    b = run_build("one-drive")
    ok, problems, refused = facts_verdict("one-drive", b)
    expect(ok and refused, f"53: the real robot-one Drive build does not refuse exactly its owed facts: {problems}")
    expect(not os.path.exists(os.path.join(REPO, "bin", "hot.package.bin")),
           "53: the refused robot-one build left bin/hot.package.bin")

    # 54 ([R4]): THE COPY'S PROOF INTERRUPTED once the copy's ledger is typed (a crash, a kill) — the
    #     repository's ledger hash and bin/ unchanged, the copy deleted by the `finally`; and a LEFTOVER copy
    #     is SWEPT by the next proof's start.
    before = _sha256(ledger_path)
    copy_dir = os.path.join(REPO, FACTS_COPY_DIR)
    os.makedirs(os.path.join(copy_dir, "leftover"), exist_ok=True)
    said = []
    try:
        facts_copy_proof(say=said.append, _fail_after_typing=True)
        expect(False, "54: the interrupted proof did not raise (the case proved nothing)")
    except RuntimeError as e:
        expect("interrupted" in str(e), f"54: the proof raised something else: {e}")
    expect(any("swept a leftover" in x for x in said), f"54: a leftover copy was not swept: {said}")
    expect(not os.path.exists(copy_dir), "54: the interrupted proof left build/src-gate-facts/ behind")
    expect(_sha256(ledger_path) == before, "54: the interrupted proof CHANGED the repository's ledger")
    expect(REPO == os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
           "54: the interrupted proof left REPO pointing at the copy")

    # 55 ([R1], T16 — THE VARIANT STAMP, `make` run DIRECTLY, never through run_build's delete): ROBOT=tank,
    #     then ROBOT=tank PROGRAM=drive with NO rm -> the second recompiles EVERY file and its package is
    #     tank-drive's; the same variant again -> 0 files; then a robot-one variant with owed facts -> refused,
    #     and NO bin/hot.package.bin left (robot two's package gone with it).
    def make_direct(*args):
        p = subprocess.run(["make", *args], cwd=REPO, capture_output=True, text=True, timeout=600)
        text = ANSI.sub("", p.stdout + p.stderr)
        return p.returncode, sum(1 for line in text.splitlines() if COMPILED.match(line)), text
    for d in ("bin", ".d"):
        shutil.rmtree(os.path.join(REPO, d), ignore_errors=True)
    n_tus = len(expected_tus())
    rc1, c1, _t1 = make_direct("ROBOT=tank")
    rc2, c2, t2 = make_direct("ROBOT=tank", "PROGRAM=drive")
    elf_path = os.path.join(REPO, "bin", "hot.package.elf")
    beacon_ok = False
    if os.path.exists(elf_path):
        with open(elf_path, "rb") as f:
            beacon_ok = _beacon_bytes(VARIANT_BEACONS["tank-drive"]) in f.read()
    rc3, c3, _t3 = make_direct("ROBOT=tank", "PROGRAM=drive")
    # A robot-one variant that still OWES facts: Bench Tests links since 2026-11-16 (its Stage A all typed), so
    # the refusal half uses Drive (Stage B owed). FOLLOW-UP: plant an owed fact so this holds once all are typed.
    rc4, _c4, t4 = make_direct("ROBOT=one", "PROGRAM=drive")
    expect(rc1 == 0 and rc2 == 0 and rc3 == 0, f"55: a robot-two build failed ({rc1}, {rc2}, {rc3})")
    expect(c1 == n_tus, f"55: the first build compiled {c1} of {n_tus} files")
    expect(c2 == n_tus, f"55: a PROGRAM switch with no rm compiled {c2} of {n_tus} files — the stamp does "
                        f"not start fresh (the old package's objects re-linked)")
    expect("shulib: the build changed" in t2, "55: the stamp's line was not printed on the switch")
    expect(beacon_ok, "55: after the switch the package is not tank-drive's")
    expect(c3 == 0, f"55: the same variant twice compiled {c3} files (the stamp starts fresh every time)")
    expect(rc4 != 0 and "robot one: OWED" in t4, "55: robot one's Drive with owed facts did not refuse")
    expect(not os.path.exists(os.path.join(REPO, "bin", "hot.package.bin")),
           "55: the refused robot-one build left the previous variant's bin/hot.package.bin to upload")

    # 56 (the review of ROB1-A — THE STAGES ARE CLASSIFIED): a program added to `Program` with no case in the
    #     ledger's needsStageB() (TUNE1-B1's Tune, the reviewers' scenario), a `default` in that switch, and
    #     the switch disagreeing with STAGE_PROGRAMS (Calibrate made to need Stage B) -> step 0 FAILS each,
    #     by name; and a robot-one row whose program STAGE_PROGRAMS does not classify -> expected_owed()
    #     refuses to guess. The real tree passes. Text plants only — no build.
    with _replace(os.path.join(REPO, SELECTION_HEADER), "    Competition,     ///<",
                  "    GateRob1NewProgram,  ///< (self-test plant: a program added)\n    Competition,     ///<"):
        problems = stage_table_problems()
        expect(any("no case for Program::GateRob1NewProgram" in p for p in problems),
               f"56: a program with no case in needsStageB() PASSED step 0: {problems}")
        expect(any("does not classify Program::GateRob1NewProgram" in p for p in problems),
               f"56: a program STAGE_PROGRAMS does not classify PASSED step 0: {problems}")
    with _replace(ledger_path, "        case Program::InventedXDrive:  //",
                  "        default:  // (self-test plant)\n        case Program::InventedXDrive:  //"):
        problems = stage_table_problems()
        expect(any("has a `default`" in p for p in problems), f"56: a `default` PASSED step 0: {problems}")
    with _replace(ledger_path, "        case Program::Calibrate:       // measures the drive; powers no declared mechanism\n",
                  ""), \
         _replace(ledger_path, "        case Program::Drive:\n",
                  "        case Program::Drive:\n        case Program::Calibrate:  // (self-test plant)\n"):
        problems = stage_table_problems()
        expect(any("Program::Calibrate: the ledger's needsStageB() says true, this gate's STAGE_PROGRAMS says "
                   "false" in p for p in problems), f"56: the switch disagreeing with the gate PASSED: {problems}")
    saved_program = BUILD_PROGRAM["one-drive"]
    BUILD_PROGRAM["one-drive"] = "GateRob1NewProgram"
    try:
        expect(any("one-drive's program GateRob1NewProgram is not classified" in p
                   for p in stage_table_problems()), "56: an unclassified robot-one row PASSED step 0")
        try:
            expected_owed("one-drive")
            expect(False, "56: expected_owed() guessed a set for an unclassified program")
        except ValueError as e:
            expect("not classified" in str(e), f"56: expected_owed() raised something else: {e}")
    finally:
        BUILD_PROGRAM["one-drive"] = saved_program
    expect(not stage_table_problems(), f"56: the real tree fails the stage check: {stage_table_problems()}")

    # 57 (the review of ROB1-A — A LEDGER COPIED BACK): the copy's own typed ledger (type_every_fact()'s
    #     output: every fact typed, sourced "gate copy") written over the repository's — the natural "restore"
    #     after an interrupted proof — and `make ROBOT=one PROGRAM=drive` run DIRECTLY (how the team builds,
    #     no gate) -> REFUSED by the ledger's own check, by name, and no package left; step 0 names it too.
    #     And the copy's MARKER FILE in the repository's build/ -> step 0 FAILS naming it.
    with open(ledger_path, encoding="utf-8") as f:
        typed_back, _n = type_every_fact(f.read())
    with _transform(ledger_path, lambda _text: typed_back):
        for d in ("bin", ".d"):
            shutil.rmtree(os.path.join(REPO, d), ignore_errors=True)
        rc, _c, text = make_direct("ROBOT=one", "PROGRAM=drive")
        expect(rc != 0 and "robot one: a typed fact needs a real source" in text,
               f"57: a ledger copied back from the gate's copy LINKED robot one's Drive (rc {rc}) — a "
               f"placeholder package to upload: {text[-400:]}")
        expect(not os.path.exists(os.path.join(REPO, "bin", "hot.package.bin")),
               "57: the copied-back ledger's refused build left bin/hot.package.bin")
        expect(any("gate copy" in p for p in ledger_source_problems()),
               "57: step 0 did not name the copied-back ledger's gate-copy sources")
    with _newfile(os.path.join(REPO, FACTS_COPY_MARKER), "(self-test plant: the copy's marker in the repo)\n"):
        expect(any(FACTS_COPY_MARKER in p and "exists in the repository" in p for p in ledger_source_problems()),
               "57: the copy's marker file in the repository's build/ PASSED step 0")
    expect(not os.path.exists(os.path.join(REPO, FACTS_COPY_MARKER)), "57: the marker plant outlived its case")

    # 58 (the review of ROB1-A's integration): STEP 8's ROBOT HALF — _self_test_robot_half() says how.
    _self_test_robot_half(expect)

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
