#pragma once
//
// build_selection.hpp — WHICH BUILD THIS IS: the PROGRAM, its identity BEACON and the ROBOT, chosen
// together by ONE #if chain (G1 Part 2c's final run, 2026-09-26 — the team lead's ruling "one chain
// picks both").
//
// ═══ THE RULE ═════════════════════════════════════════════════════════════════════════
// The build switches — SHULIB_ROBOT_XDRIVE_INVENTED, SHULIB_ROBOT_TANK_2026, SHULIB_ROBOT_ONE_2026
// (robot one, ROB1-A 2026-11-11), SHULIB_PROGRAM_DRIVE,
// SHULIB_PROGRAM_CALIBRATE, SHULIB_PROGRAM_AUTONTEST, SHULIB_PROGRAM_COMPETITION and SHULIB_PROGRAM_TUNE, set by the
// Makefile's validated ROBOT and PROGRAM variables — are READ IN THIS FILE AND NOWHERE ELSE: not in
// another src/ file, not in a
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
// ═══ A RETIRED SWITCH (T4 Part 3, 2026-10-01) ═══════════════════════════════════════════
// SHULIB_PROGRAM_LIBRARY — `make ROBOT=tank PROGRAM=library`, "shulib Teleop" — was RETIRED when
// "shulib Drive" (robot two's driver file) replaced it (the team lead's ruling R5, 2026-09-27). It
// is not re-pointed at another program: a stale command in someone's notes would then upload a
// different program under a retired name without a word. So the Makefile's PROGRAM=library is an
// $(error) naming the retirement, and tools/src_build_gate.py keeps it on a RETIRED list: the
// switch named ANYWHERE — here included — fails step 0, and a build carrying its define or its
// beacon fails steps 5 and 6.
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
#if (defined(SHULIB_ROBOT_XDRIVE_INVENTED) + defined(SHULIB_ROBOT_TANK_2026) \
     + defined(SHULIB_ROBOT_ONE_2026)) > 1
#error "two robot variant defines are set at once -- select exactly one with make ROBOT=..."
#endif
#if (defined(SHULIB_PROGRAM_DRIVE) + defined(SHULIB_PROGRAM_CALIBRATE) \
     + defined(SHULIB_PROGRAM_AUTONTEST) + defined(SHULIB_PROGRAM_COMPETITION) \
     + defined(SHULIB_PROGRAM_TUNE)) > 1
#error "two PROGRAM defines are set at once -- select exactly one with make PROGRAM=..."
#endif
// The four programs past the bench tester need a robot with a driver file, autons and a record: robot
// two (ROBOT=tank) or, since ROB1-A (2026-11-11), robot one (ROBOT=one). Robot two's words are kept;
// "or ROBOT=one" is appended.
#if defined(SHULIB_PROGRAM_DRIVE) && !defined(SHULIB_ROBOT_TANK_2026) && !defined(SHULIB_ROBOT_ONE_2026)
#error "PROGRAM=drive needs ROBOT=tank or ROBOT=one -- only robot two and robot one have a driver file and a calibration record"
#endif
#if defined(SHULIB_PROGRAM_CALIBRATE) && !defined(SHULIB_ROBOT_TANK_2026) && !defined(SHULIB_ROBOT_ONE_2026)
#error "PROGRAM=calibrate needs ROBOT=tank or ROBOT=one -- the calibrator supports only a tank drivetrain and takes the cartridge from this robot's chassis table"
#endif
#if defined(SHULIB_PROGRAM_AUTONTEST) && !defined(SHULIB_ROBOT_TANK_2026) && !defined(SHULIB_ROBOT_ONE_2026)
#error "PROGRAM=autontest needs ROBOT=tank or ROBOT=one -- the team's autons (src/team/autons.cpp, src/team/robot_one_autons.cpp) are robot two's and robot one's, and only they boot from a calibration record"
#endif
#if defined(SHULIB_PROGRAM_COMPETITION) && !defined(SHULIB_ROBOT_TANK_2026) && !defined(SHULIB_ROBOT_ONE_2026)
#error "PROGRAM=competition needs ROBOT=tank or ROBOT=one -- the team's autons and driver files (src/team/) are robot two's and robot one's, and only they boot from a calibration record"
#endif
// PROGRAM=tune (TUNE1-B1): a robot with a Tune branch in the chain below — one per robot in the
// Makefile's MECHANISM_ROBOTS (a robot whose mechanism table is not empty). A robot that registers a
// table adds its branch there and its define here; without them the build falls to no Tune branch.
#if defined(SHULIB_PROGRAM_TUNE) && !defined(SHULIB_ROBOT_TANK_2026) && !defined(SHULIB_ROBOT_ONE_2026)
#error "PROGRAM=tune needs a ROBOT whose mechanism table is not empty -- this ROBOT has no Tune branch in src/build_selection.hpp's chain (see the Makefile's MECHANISM_ROBOTS)"
#endif

#include <string_view>  // beaconHasPrefix() (ROB1-BT)

namespace shulib::bench {

/// The programs a build can be. src/main.cpp's five entry points switch on it.
enum class Program {
    BenchTests,      ///< "Bench Tests" — the bench bot's, robot two's (ROBOT=tank) or robot one's (ROBOT=one)
    Drive,           ///< "shulib Drive" — a robot's driver file on the library (T4 Part 3; robot one since ROB1-A)
    Calibrate,       ///< "shulib Calibrate" — the robot measures itself
    InventedXDrive,  ///< the invented X-drive (HA-111)
    AutonTester,     ///< "shulib Auton Tester" — runs one of the team's autons from RUN (COMP-t)
    Competition,     ///< "shulib Competition" — the competition program (COMP-b)
    Tune,            ///< "shulib Tune" — live tuning of the robot's mechanisms (TUNE1-B1)
};

/// The robots a build can carry the tables of. The tables select on it (chassisTableFor(),
/// mechanismTableFor()), and robotBeacon() names it in the linked package.
enum class RobotId {
    BenchBot,        ///< the measured tank bench bot (the team's old competition robot)
    RobotTwo,        ///< robot two — robot one's twin, the same cascade build (ROB1-BT; the claw tank chassis before)
    InventedXDrive,  ///< the invented X-drive (A4 register HA-111; no such robot exists)
    RobotOne,        ///< the 2026 cascade robot, robot one (ROB1-A; built ~2026-11-14 — nothing has run on it)
};

// ═══ THE ONE CHAIN: the program, the robot AND the beacon, chosen in the same branch ════════
// The VARIANT IDENTITY BEACON (R3b Session 2) is one string per BUILD, printed in the program's
// boot banner so it is referenced and survives into the linked package; the gate asserts the
// expected one IS in bin/hot.package.elf and the other thirteen — and the RETIRED `...=tank-library` —
// are NOT (matched WITH the NUL terminator, because `...=tank` is a prefix of `...=tank-drive`,
// `...=tank-calibrate`, `...=tank-autontest`, `...=tank-competition` and `...=tank-tune`, and `...=one`
// of robot one's five `...=one-<program>`). Every branch sets all
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
#elif defined(SHULIB_ROBOT_TANK_2026) && defined(SHULIB_PROGRAM_TUNE)
inline constexpr Program kProgram = Program::Tune;
inline constexpr RobotId kRobotId = RobotId::RobotTwo;
inline constexpr char kVariantBeacon[] = "shulib-robot-variant=tank-tune";
#elif defined(SHULIB_ROBOT_TANK_2026)
inline constexpr Program kProgram = Program::BenchTests;
inline constexpr RobotId kRobotId = RobotId::RobotTwo;
inline constexpr char kVariantBeacon[] = "shulib-robot-variant=tank";
#elif defined(SHULIB_ROBOT_ONE_2026) && defined(SHULIB_PROGRAM_DRIVE)
inline constexpr Program kProgram = Program::Drive;
inline constexpr RobotId kRobotId = RobotId::RobotOne;
inline constexpr char kVariantBeacon[] = "shulib-robot-variant=one-drive";
#elif defined(SHULIB_ROBOT_ONE_2026) && defined(SHULIB_PROGRAM_CALIBRATE)
inline constexpr Program kProgram = Program::Calibrate;
inline constexpr RobotId kRobotId = RobotId::RobotOne;
inline constexpr char kVariantBeacon[] = "shulib-robot-variant=one-calibrate";
#elif defined(SHULIB_ROBOT_ONE_2026) && defined(SHULIB_PROGRAM_AUTONTEST)
inline constexpr Program kProgram = Program::AutonTester;
inline constexpr RobotId kRobotId = RobotId::RobotOne;
inline constexpr char kVariantBeacon[] = "shulib-robot-variant=one-autontest";
#elif defined(SHULIB_ROBOT_ONE_2026) && defined(SHULIB_PROGRAM_COMPETITION)
inline constexpr Program kProgram = Program::Competition;
inline constexpr RobotId kRobotId = RobotId::RobotOne;
inline constexpr char kVariantBeacon[] = "shulib-robot-variant=one-competition";
#elif defined(SHULIB_ROBOT_ONE_2026) && defined(SHULIB_PROGRAM_TUNE)
inline constexpr Program kProgram = Program::Tune;
inline constexpr RobotId kRobotId = RobotId::RobotOne;
inline constexpr char kVariantBeacon[] = "shulib-robot-variant=one-tune";
#elif defined(SHULIB_ROBOT_ONE_2026)
inline constexpr Program kProgram = Program::BenchTests;
inline constexpr RobotId kRobotId = RobotId::RobotOne;
inline constexpr char kVariantBeacon[] = "shulib-robot-variant=one";
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
        case RobotId::RobotOne:
            return "shulib-robot=robot-one";
    }
    __builtin_unreachable();
}

/// THIS build's robot beacon. src/main.cpp prints it at boot, which is what links it (and only it)
/// into the package.
inline constexpr const char* kRobotBeacon = robotBeacon(kRobotId);

/// The word every robot beacon starts with — the robot's name follows it.
inline constexpr char kRobotBeaconPrefix[] = "shulib-robot=";

/// True when `beacon` starts with kRobotBeaconPrefix (a constant expression).
constexpr bool beaconHasPrefix(const char* beacon) { return std::string_view{beacon}.starts_with(kRobotBeaconPrefix); }

/// Robot `id`'s name in its CALIBRATION RECORD (chunk ROB1-BT): its beacon after "shulib-robot=" —
/// `robot-one`, `robot-two` — so the beacon stays the one place a robot's name is typed. Calibrate writes
/// it into the record (`robot=robot-two ...`); the boot refuses a record naming another robot, or none.
constexpr const char* robotRecordName(RobotId id) { return robotBeacon(id) + (sizeof kRobotBeaconPrefix - 1); }
static_assert(beaconHasPrefix(robotBeacon(RobotId::BenchBot)) && beaconHasPrefix(robotBeacon(RobotId::RobotTwo)) &&
                  beaconHasPrefix(robotBeacon(RobotId::InventedXDrive)) &&
                  beaconHasPrefix(robotBeacon(RobotId::RobotOne)),
              "every robot beacon starts with \"shulib-robot=\" -- the record's robot name is the rest of it");

}  // namespace shulib::bench
