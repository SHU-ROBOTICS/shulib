#pragma once
//
// build_selection.hpp — WHICH BUILD THIS IS: the PROGRAM, its identity BEACON and the ROBOT, chosen
// together by ONE #if chain (G1 Part 2c's final run, 2026-09-26 — the team lead's ruling "one chain
// picks both").
//
// ═══ THE RULE ═════════════════════════════════════════════════════════════════════════
// The build switches — SHULIB_ROBOT_XDRIVE_INVENTED, SHULIB_ROBOT_TANK_2026, SHULIB_PROGRAM_DRIVE,
// SHULIB_PROGRAM_LIBRARY, SHULIB_PROGRAM_CALIBRATE, SHULIB_PROGRAM_AUTONTEST and
// SHULIB_PROGRAM_COMPETITION, set by the Makefile's validated ROBOT and PROGRAM variables — are
// READ IN THIS FILE AND NOWHERE ELSE: not in another src/ file, not in a
// library header under include/shulib/ (every src/ file includes them, so they see the same -D
// flags), not in a host test. They are read here twice:
//   * by the impossible-combination guards (an #error, never a program); and
//   * by THE ONE CHAIN, whose every branch sets exactly three constants, together: kProgram (which
//     program this build runs), kRobotId (which robot's tables it carries) and kVariantBeacon (the
//     build's identity string).
// Below the chain everything is C++. src/main.cpp switches on kProgram, a case per program and no
// default; src/robots/chassis_table.hpp and src/robots/mechanism_table.hpp select the robot's tables from kRobotId
// through constexpr functions with a case per robot (chassisTableFor(), mechanismTableFor()); and
// robotBeacon() below turns kRobotId into the robot's beacon. So a misspelt program or robot anywhere
// below the chain is a compile ERROR, and one added to an enum but not handled somewhere is a
// -Wswitch warning, which the src build gate fails. tools/src_build_gate.py's step 0 enforces the
// rule on the source: a build switch named in any other file under src/, include/shulib/ or test/,
// a second chain, or a branch that does not set all three constants FAILS the gate.
//
// ═══ WHY THE ROBOT IS CHOSEN HERE TOO ═════════════════════════════════════════════════
// Until this change the program was chosen here (in src/main.cpp, which held this chain) and the
// ROBOT a second and third time, by #if's of the tables' own on SHULIB_ROBOT_TANK_2026. A break in
// one of those gave a tank build the bench bot's table, or no claws, while main.cpp's chain — and
// so the beacon — still read right, and no detector could see it (G1 Part 2c named that residual
// and measured it: a misspelt #if in chassis_table.hpp PASSED the gate). A table cannot choose on
// its own any more; it can only look at kRobotId.
//
// ═══ WHAT VOUCHES FOR WHAT (tools/src_build_gate.py) ══════════════════════════════════
//   * kVariantBeacon — typed in its branch, printed in the program's boot banner — vouches for
//     the BRANCH the build took (step 6);
//   * kRobotBeacon — DERIVED from kRobotId by robotBeacon(), never typed in a branch, printed by
//     src/main.cpp's initialize() — vouches for the ROBOT that branch set (step 6's robot half): a
//     tank build whose branch says RobotId::BenchBot links "shulib-robot=bench-bot" and not
//     "shulib-robot=robot-two", and FAILS;
//   * the programs' functions in the linked ELF's symbol table vouch for the PROGRAM the branch set
//     and for what each entry point calls (step 8);
//   * step 0 reads this file and checks each branch's three constants against the gate's own
//     per-build tables, so the source and the gate cannot drift apart silently; and it fails a
//     robot chosen OUTSIDE the chain by other means: a program or library header naming a
//     per-robot table (below) outside a static_assert, or a #define of a selection name — this
//     file's, or a table's (`#define kRobotId RobotId::BenchBot` after this file is included
//     would re-point every table's selection while kRobotBeacon, derived here, kept the right
//     robot's beacon).
//
// ═══ THE HOST ═════════════════════════════════════════════════════════════════════════
// The host test build sets no switch, so the chain takes its #else — the bench build's branch,
// exactly what `make` with no ROBOT builds — and the host's kChassis / kMechanisms are the bench
// build's. The suite does not depend on that: every robot's tables are defined in every build under
// their own names (kBenchBotChassis, kRobotTwoChassis, kNoChassis, kRobotTwoMechanisms,
// kNoMechanisms), and the host tests read those and call the selection functions for EVERY RobotId —
// so the tables robot two's builds carry are tested byte for byte on the host. A program never
// names them: it reads kChassis and kMechanisms (the src build gate's step 0 enforces it).
//
// PROS-FREE on purpose, and nothing here but the switches, the chain and the robot beacon: every
// src/ file that reads a table includes it through the table, and the host suite includes it too.

// ── The impossible combinations: an #error, never a program ────────────────────────────
// (The instruction for choosing a build used to say `make CXXFLAGS_EXTRA=-D...` — common.mk
//  consumes EXTRA_CXXFLAGS, the names were transposed, and the documented command silently built
//  the BENCH variant. GATE1 replaced it with the validated ROBOT switch in the Makefile, and
//  tools/src_build_gate.py asserts per build that the defines really land — as whole command-line
//  TOKENS, and end-to-end via the beacons below.)
#if defined(SHULIB_ROBOT_XDRIVE_INVENTED) && defined(SHULIB_ROBOT_TANK_2026)
#error "two robot variant defines are set at once -- select exactly one with make ROBOT=..."
#endif
#if (defined(SHULIB_PROGRAM_DRIVE) + defined(SHULIB_PROGRAM_LIBRARY) \
     + defined(SHULIB_PROGRAM_CALIBRATE) + defined(SHULIB_PROGRAM_AUTONTEST) \
     + defined(SHULIB_PROGRAM_COMPETITION)) > 1
#error "two PROGRAM defines are set at once -- select exactly one with make PROGRAM=..."
#endif
#if defined(SHULIB_PROGRAM_DRIVE) && !defined(SHULIB_ROBOT_TANK_2026)
#error "PROGRAM=drive needs ROBOT=tank -- only robot two's chassis table carries measured signs"
#endif
#if defined(SHULIB_PROGRAM_LIBRARY) && !defined(SHULIB_ROBOT_TANK_2026)
#error "PROGRAM=library needs ROBOT=tank -- only robot two has a calibration record and a signed chassis table"
#endif
#if defined(SHULIB_PROGRAM_CALIBRATE) && !defined(SHULIB_ROBOT_TANK_2026)
#error "PROGRAM=calibrate needs ROBOT=tank -- the calibrator supports only a tank drivetrain and takes the cartridge from robot two's chassis table"
#endif
#if defined(SHULIB_PROGRAM_AUTONTEST) && !defined(SHULIB_ROBOT_TANK_2026)
#error "PROGRAM=autontest needs ROBOT=tank -- the team's autons (src/team/autons.cpp) are robot two's, and only robot two boots from a calibration record"
#endif
#if defined(SHULIB_PROGRAM_COMPETITION) && !defined(SHULIB_ROBOT_TANK_2026)
#error "PROGRAM=competition needs ROBOT=tank -- the team's autons and driver file (src/team/) are robot two's, and only robot two boots from a calibration record"
#endif

namespace shulib::bench {

/// The programs a build can be. src/main.cpp's five entry points switch on it.
enum class Program {
    BenchTests,      ///< "Bench Tests" — the bench bot's, or robot two's (ROBOT=tank)
    Drive,           ///< "shulib Drive"
    Teleop,          ///< "shulib Teleop" — THE LIBRARY drives robot two
    Calibrate,       ///< "shulib Calibrate" — robot two measures itself
    InventedXDrive,  ///< the invented X-drive (HA-111)
    AutonTester,     ///< "shulib Auton Tester" — runs one of the team's autons from RUN (COMP-t)
    Competition,     ///< "shulib Competition" — the competition program (COMP-b)
};

/// The robots a build can carry the tables of. The tables select on it (chassisTableFor(),
/// mechanismTableFor()), and robotBeacon() names it in the linked package.
enum class RobotId {
    BenchBot,        ///< the measured tank bench bot (the team's old competition robot)
    RobotTwo,        ///< the 2026 tank chassis, robot two
    InventedXDrive,  ///< the invented X-drive (A4 register HA-111; no such robot exists)
};

// ═══ THE ONE CHAIN: the program, the robot AND the beacon, chosen in the same branch ════════
// The VARIANT IDENTITY BEACON (R3b Session 2) is one string per BUILD, printed in the program's
// boot banner so it is referenced and survives into the linked package; the gate asserts the
// expected one IS in bin/hot.package.elf and the other seven are NOT (matched WITH the NUL
// terminator, because `...=tank` is a prefix of `...=tank-drive`, `...=tank-library`,
// `...=tank-calibrate`, `...=tank-autontest` and `...=tank-competition`). Every branch sets all
// three constants and nothing else, so a broken or
// renamed #if here changes the program, the robot and the beacon at once, and the gate FAILS. NEVER
// select a program or a robot anywhere else, and never with a second chain. Each beacon is
// referenced only through the selected branch's constant, so no other build's package can carry it
// (the G1 Part 2 brief's landmine 10).
#if defined(SHULIB_ROBOT_XDRIVE_INVENTED)
inline constexpr Program kProgram = Program::InventedXDrive;
inline constexpr RobotId kRobotId = RobotId::InventedXDrive;
inline constexpr char kVariantBeacon[] = "shulib-robot-variant=xdrive";
#elif defined(SHULIB_ROBOT_TANK_2026) && defined(SHULIB_PROGRAM_DRIVE)
inline constexpr Program kProgram = Program::Drive;
inline constexpr RobotId kRobotId = RobotId::RobotTwo;
inline constexpr char kVariantBeacon[] = "shulib-robot-variant=tank-drive";
#elif defined(SHULIB_ROBOT_TANK_2026) && defined(SHULIB_PROGRAM_LIBRARY)
inline constexpr Program kProgram = Program::Teleop;
inline constexpr RobotId kRobotId = RobotId::RobotTwo;
inline constexpr char kVariantBeacon[] = "shulib-robot-variant=tank-library";
#elif defined(SHULIB_ROBOT_TANK_2026) && defined(SHULIB_PROGRAM_CALIBRATE)
inline constexpr Program kProgram = Program::Calibrate;
inline constexpr RobotId kRobotId = RobotId::RobotTwo;
inline constexpr char kVariantBeacon[] = "shulib-robot-variant=tank-calibrate";
#elif defined(SHULIB_ROBOT_TANK_2026) && defined(SHULIB_PROGRAM_AUTONTEST)
inline constexpr Program kProgram = Program::AutonTester;
inline constexpr RobotId kRobotId = RobotId::RobotTwo;
inline constexpr char kVariantBeacon[] = "shulib-robot-variant=tank-autontest";
#elif defined(SHULIB_ROBOT_TANK_2026) && defined(SHULIB_PROGRAM_COMPETITION)
inline constexpr Program kProgram = Program::Competition;
inline constexpr RobotId kRobotId = RobotId::RobotTwo;
inline constexpr char kVariantBeacon[] = "shulib-robot-variant=tank-competition";
#elif defined(SHULIB_ROBOT_TANK_2026)
inline constexpr Program kProgram = Program::BenchTests;
inline constexpr RobotId kRobotId = RobotId::RobotTwo;
inline constexpr char kVariantBeacon[] = "shulib-robot-variant=tank";
#else
inline constexpr Program kProgram = Program::BenchTests;
inline constexpr RobotId kRobotId = RobotId::BenchBot;
inline constexpr char kVariantBeacon[] = "shulib-robot-variant=bench";
#endif

/// THE ROBOT BEACON of robot `id`: one string per ROBOT (the six tank builds share robot two's),
/// DERIVED here from the enum and never typed in a branch — so it vouches for the kRobotId the
/// build's branch set, which the branch's own typed kVariantBeacon cannot. The gate asserts the
/// build's robot beacon IS in its linked package and the other robots' are NOT (step 6), matched
/// with the NUL terminator like the variant beacons. A case per robot and no default: a robot
/// added to RobotId without a beacon is -Wswitch (a gate FAILURE), and reaching the end in a
/// constant expression is a compile error.
constexpr const char* robotBeacon(RobotId id) {
    switch (id) {
        case RobotId::BenchBot:
            return "shulib-robot=bench-bot";
        case RobotId::RobotTwo:
            return "shulib-robot=robot-two";
        case RobotId::InventedXDrive:
            return "shulib-robot=invented-xdrive";
    }
    __builtin_unreachable();
}

/// THIS build's robot beacon. src/main.cpp prints it at boot, which is what links it (and only it)
/// into the package.
inline constexpr const char* kRobotBeacon = robotBeacon(kRobotId);

}  // namespace shulib::bench
