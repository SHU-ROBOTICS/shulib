#pragma once
//
// THE MECHANISM TABLE — every robot's MECHANISMS, in the mechanism core's words (chunk MECH-A5,
// 2026-10-29; chunk T3 wrote the first table on 2026-09-26, the pneumatic claw its addendum E2). Each
// robot's mechanisms are DECLARATIONS — what each one IS (mechanism/kinds.hpp: an Arm, a Lift, a
// Flywheel, a Power roller, a Pneumatic) — followed by ONE LIST that says where each is plugged in
// (mechanism/wiring.hpp: `wire(claw, lines('A'))`), all inside that robot's own NAMESPACE
// (`robot_two`), so two robots' `claw`s never collide in this one file every build compiles. The
// PROS-free sibling of src/robots/chassis_table.hpp, which stays "the ONE typed input about a
// drivetrain"; selected, like it, by kRobotId — which src/build_selection.hpp's ONE chain sets in the
// same branch as the program and the beacon — so `make ROBOT=tank` and this file cannot disagree about
// which robot it is. THIS FILE HAS NO #if OF ITS OWN (G1 Part 2c's final run, the team lead's ruling
// "one chain picks both"); the src build gate's step 0 fails any build switch named here.
//
// WHAT IT DECLARES — what the robot IS, never which button does what (that is the robot's DRIVER
// FILE's: src/team/robot_two_driver.hpp, the team lead's ruling R1 of 2026-09-27). Robot two (the
// 2026 tank chassis) has two SEPARATE claw PIVOT motors on ports 6 and 7, RED, each holding its angle
// when let go, and a PNEUMATIC CLAW on the brain's 3-wire port A, CLOSED at the start and when made
// safe, keeping its state through a disable (the team lead, 2026-09-26; T3 and its addendum §0).
// Every other robot declares none. Two pivots are two DECLARATIONS — a mechanism fans one voltage to
// all its motors, so one two-motor declaration would move both claws at once (T3 brief, landmine 1).
//
// A VALUE IS NEVER INVENTED. A field a declaration needs and does not state FAILS THE BUILD, by name
// (mechanism/spec.hpp's refusals: a Pneumatic's `.moveTime`, a cartridge, a safe state, ...), and so do
// a port used twice, a 3-wire letter where a smart port goes, a level count that does not match the
// wiring — mistakes the mechanism core refuses at build, each by name (GCC names your line on the
// error or in its first note — guide chapter 18, "reading a refusal").
// What the build checks no longer needs a check here: until MECH-A5 this file carried five
// consistency checks of its own over hand-typed entry structs; the declarations and mechanisms(...)
// make every one of them now, and a declaration's PROVENANCE is the comment above it (the decided
// rule of the MECH brief: a build check could prove a string non-empty, never that it is true — the
// facts that matter are in these comments AND the A4 register).
//
// SIGNS AND THE CLAW'S LEVEL LIVE HERE AND NOWHERE ELSE. A pivot's `.motors = {Forward}` is its
// motor's own forward; `Reversed` is the flip, written on its declaration and reversed ONCE, by the
// motor adapter (a negative port in the config the boot derives). Which PHYSICAL way either pivot
// turns is UNMEASURED (Station J, A4 register HA-153), and a pivot that moves the wrong way is fixed
// on ITS declaration — never by swapping its buttons in the driver file. The claw's open level is a
// GUESS written ON its `.positions` line (closed = Low, open = High — "line ON = open", the T3
// addendum's starting point, A4 register HA-159): Station J.9 settles it, and the fix for a claw that
// opens on the wrong press is to swap Low and High on that line — its start and safe positions are
// NAMES ("closed"), so the start level and the safe level move with the swap, and nothing else is
// edited. The flip is a measurement: test/mechanism_table_test.cpp pins the levels, and HA-159 and
// Station J.9 quote them — all three change together.
//
// ONE LINE, because one is what was reported (port A). Whether the valve is single or double is NOT
// KNOWN (HA-159); Station J looks at how many 3-wire cables run from the valve into the brain. If it is
// two, the claw does not quietly gain a line: the second line's letter is read off the brain first,
// and each position then states one level PER line (`{"closed", {Low, High}}`) — the build refuses a
// position whose level count differs from its wiring's lines.
//
// THE HOST SUITE TESTS THESE VERY BYTES. Every robot's list is defined in EVERY build under its own
// name (robot_two::kRobotTwoMechanisms), and only the selection — mechanismTableFor(kRobotId), below —
// depends on the build, so test/mechanism_table_test.cpp reads the real declarations field by field
// and calls mechanismTableFor() for EVERY RobotId. A program uses kMechanisms, never a robot's own
// list or namespace: the src build gate's step 0 fails a program that names kRobotTwoMechanisms,
// kNoMechanisms or mechanismTableFor() outside a static_assert, and — PER ROBOT, since MECH-A5 — one
// that names `robot_two` at all (`robot_two::`, `using namespace robot_two`, re-opening it, an alias, a
// macro, split over lines or by a line splice: the name wherever the text writes it — the gate reads the
// text as the compiler does; a name a macro pastes together with `##` is beyond it), outside
// this file, test/, a static_assert and robot two's OWN team files (src/team/robot_two_*, and
// src/team/autons.{hpp,cpp} by the gate's reviewed TEAM_FILE_ROBOT map).
//
// G3a's GENERATOR WRAPS ROBOT TWO'S REGION BYTE FOR BYTE: `// [robot two mechanisms]` ..
// `// [/robot two mechanisms]` is one contiguous block — the caveat comments, the declarations, the
// wiring — with no hand-written line inside (the MECH brief's §6.2 item 2). mechanismTableFor() stays a
// case per robot with no default; G3a Part 2's `[mechanism selection]` markers go inside it.
//
// WHO READS IT, as of MECH-A5:
//   * every program that BOOTS robot two from its record ("shulib Drive", the Auton Tester, the
//     competition program): src/common/boot_source.hpp derives the config::RobotConfig's mechanisms from
//     this list (mechanism::toMechanismConfig, in the list's order), and its declared-port rule refuses
//     a record that drives a declared mechanism port — a motor's or a smart sensor's; each program binds
//     ONE mechanism::MechanismService over the list and the Robot's built devices, prints one boot line
//     per role (mechanism::roleLine) and publishes the list's ROUTE IDS (runner::MechanismRoutes);
//   * robot two's DRIVER FILE (src/team/robot_two_driver.hpp): its controls(...) are checked against
//     kRobotTwoMechanisms at build, and bound to the service by each declaration's name at boot;
//   * the team's autons (src/team/autons.cpp): `using namespace robot_two;`, then a step names a
//     declaration (`r.set(claw["closed"])`);
//   * "shulib Calibrate" (the standard Calibrate since CAL1 Part B, 2026-11-05):
//     src/robots/calibrate_facts.hpp maps this list into its facts — one declared port per motor role,
//     mechanism::declaredMotorPorts(), named by its mechanism (the MECH brief's N9) — and the program
//     drops those ports from its census BEFORE any motor is made, so a declared pivot is never a drive
//     candidate, and WATCHES them during the push without commanding them. (Until CAL1 Part B robot
//     two's own Calibrate dropped them through dropDeclaredMechanismPorts(), deleted with it.)
//
// PROS-FREE ON PURPOSE: plain declarations and pure helpers, so the host test includes it from src/.
// It lives in src/, not include/shulib/, because it is a per-robot composition fact, not library API.
// NOTHING HERE HAS RUN ON A ROBOT.

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "build_selection.hpp"
#include "robots/chassis_table.hpp"
#include "shulib/mechanism/kinds.hpp"
#include "shulib/mechanism/wiring.hpp"

namespace shulib::bench {

/// A robot's list of mechanisms — the LIBRARY's own type (mechanism/wiring.hpp), named here without an
/// alias so the src build gate's step 0 reads `constexpr MechanismTable <name>` as it always has.
using mechanism::MechanismTable;

namespace robot_two {
using namespace mechanism::words;
// [robot two mechanisms]
// The claw pivots (the team lead, 2026-09-26): RED; each holds its angle when let go. Ports 6/7 are
// as REPORTED -- no census has seen them (HA-153); each + is its motor's own forward, and which way
// that turns the claw is unmeasured (the bench page's direction check). No stall signature: held
// against its stop, a pivot is cut only by the driver letting go (HA-154); a loaded hold is
// unmeasured (HA-156).
inline constexpr Power clawPivot6{.name = "claw pivot 6", .motors = {Forward}, .cartridge = Red, .safe = Hold};
inline constexpr Power clawPivot7{.name = "claw pivot 7", .motors = {Forward}, .cartridge = Red, .safe = Hold};

// The pneumatic claw (the team lead, 2026-09-26): CLOSED at the start and when made safe. Which
// level OPENS it, and whether it is single or double, are GUESSES (HA-159; Station J.9): to flip
// it, swap Low and High below.
inline constexpr Pneumatic claw{
    .name = "claw",
    .positions = {{"closed", {Low}, grabs}, {"open", {High}}},
    .start = "closed",
    .safe = "closed",
    .moveTime = 0.5_s,  // PROVISIONAL (HA-185): Station J times it
};

// Where each one is plugged in. Re-cabling changes only these lines. A claw cable swapped with a
// drive cable is invisible to the boot (HA-155): label the cables.
inline constexpr MechanismTable kRobotTwoMechanisms = mechanisms(
    wire(clawPivot6, motors(6)),
    wire(clawPivot7, motors(7)),
    wire(claw, lines('A')));
// [/robot two mechanisms]
}  // namespace robot_two
}  // namespace shulib::bench

// ═══ ROBOT ONE — the 2026 CASCADE robot (chunk ROB1-A, 2026-11-11) ═══════════════════════════
// EVERY LINE FOR ROBOT ONE IS BELOW ROBOT TWO'S REGION, on purpose (the ROB1 brief's re-check R5):
// each declaration and wire(...) above records the LINE it is written on into the list robot two's
// packages carry, so one line added or removed above `// [/robot two mechanisms]` would change all six
// of robot two's tank packages while their machine code and strings stayed the same. This header's
// banner names robot two only; for robot one, every sentence of it holds with these differences:
//   * its numbers come from ITS FACTS LEDGER, src/robots/robot_one_facts.hpp — read as `facts::...`
//     below — where every number nobody has measured yet is OWED: a placeholder the host suite runs
//     on, which REFUSES every robot-one build of a program that needs it, by name;
//   * its driver file is src/team/robot_one_driver.hpp and its autons src/team/robot_one_autons.cpp
//     (`using namespace robot_one;`), the src build gate allowing `robot_one` in src/team/robot_one_*;
//   * its flipper is the FRONT TOGGLE, a pneumatic (the builder's pistons on B, made the autons' flipper by
//     the coordinator, 2026-11-15; ROB1-C2 declared it as the
//     test-proven pneumatic form of test/robot_one_flipper_forms_example_test.cpp — guide chapter 21);
//   * two checks below its list hold ITS facts to ITS list: every non-drive motor the Devices screen
//     lists is declared (or shulib Calibrate would power it as a drive candidate — [R9]), and the
//     flipper's typed kind matches what is declared.
// G3a's generator may take this region over later as it will robot two's: the region between the two
// `[robot one mechanisms]` marker comments below is one contiguous block with no hand-written line inside.
// NOTHING ABOUT ROBOT ONE HAS RUN ON A ROBOT: its facts are typed from its builder's answers (2026-11-15), not
// measured, and no census, reading or photo of it is in this project's records.
#include "robots/robot_one_facts.hpp"

namespace shulib::bench {

namespace robot_one {
using namespace mechanism::words;
namespace facts = robot_one_facts;
// [robot one mechanisms]
// THE CASCADE (the builder, 2026-11-05 and 2026-11-15): a 4-stage STRING cascade (a stationary first stage,
// two intermediate stages, a final stage carrying the claw), strings both ways, NO limit switch, homed by
// stalling gently DOWN into its bottom stop under its current cap -- on its OWN, small homing signature (the
// facts' cascadeHomingStall(): a gentle push never draws the jam signature's amps). Two BLUE motors, DIRECT
// to the spool, facing each other (HA-255; which is Forward is the coordinator's reading, station 13's
// direction check confirms); a third or fourth motor is a DECLARATION edit here (`.motors` and
// `motors(...)`). MANUAL for now: no levels yet (the facts' kCascadeLevels), so every preset slot below is
// empty until levels are typed. Its shape is
// MEASURED -- a row at the bottom stop and the top stop, and at every level once there are levels (HA-254:
// a string winds onto a spool in layers, so the height per turn changes as it climbs; a row AT each level
// makes each level exact). EVERY NUMBER IS A FACT, typed or OWED (src/robots/robot_one_facts.hpp).
inline constexpr Lift cascade{
    .name = "cascade",
    .motors = {facts::kCascadeDirections.value().first, facts::kCascadeDirections.value().second},
    .cartridge = facts::kCascadeCartridge.value(),
    .gearing = gears(facts::kCascadeTeeth.value().motor, facts::kCascadeTeeth.value().spool),
    .shape = measured(facts::kCascadeRows.value().row),
    .safe = Hold,
    .stall = facts::cascadeStall(),
    .currentCap = {.amps = facts::kCascadeCapAmps.value()},
    .range = {facts::cascadeBottom(), facts::cascadeTop()},
    .gravity = constantGravity(units::Voltage{facts::kCascadeHoldVolts.value()}),
    .home = homeByStall(facts::cascadeBottom(), units::Voltage{facts::kCascadeHomingPushVolts.value()},
                        facts::cascadeHomingStall()),
    .presets = {{facts::lv(0).name, facts::lv(0).at}, {facts::lv(1).name, facts::lv(1).at},
                {facts::lv(2).name, facts::lv(2).at}, {facts::lv(3).name, facts::lv(3).at},
                {facts::lv(4).name, facts::lv(4).at}, {facts::lv(5).name, facts::lv(5).at}},
    .unhomedVolts = units::Voltage{facts::kCascadeUnhomedVolts.value()},
};

// THE SIDE TOGGLES (the builder, 2026-11-15): two GREEN motors, DIRECT (a power mechanism states no
// gearing), each its own arm on its own motor and driven SEPARATELY -- held, it RUNS one way; let go, it
// STOPS (Brake: "when I let go it will stop"). POWER mechanisms (run at a voltage), not Arms: no angles,
// no homing. Which way each runs is its `.motors` direction -- OWED until station 13 (the two run mirror-
// image ways). No stall signature: held against its stop, a toggle is cut only by the driver letting go
// (the boot says so, HA-154's words); its current cap (chosen, not measured) bounds how hard it pushes
// there once a program's service binds -- Bench Tests' station 13 binds none, so there it is uncapped.
inline constexpr Power rightToggle{.name = "right toggle", .motors = {facts::kRightToggleDirection.value()},
                                   .cartridge = facts::kSideToggleCartridge.value(), .safe = Brake,
                                   .currentCap = {.amps = facts::kSideToggleCapAmps.value()}};
inline constexpr Power leftToggle{.name = "left toggle", .motors = {facts::kLeftToggleDirection.value()},
                                  .cartridge = facts::kSideToggleCartridge.value(), .safe = Brake,
                                  .currentCap = {.amps = facts::kSideToggleCapAmps.value()}};

// THE PISTONS (the builder, 2026-11-15), each ONE line, two positions. Each STARTS and is made safe in the
// position its solenoid holds UNPOWERED (facts::unpoweredPosition), so nothing moves at power-up; which
// line level is which position is OWED until station 13 fires each (the facts' k...Level).
// The CLAW: one piston on the cascade's top stage -- it picks up Pins and Cups.
inline constexpr Pneumatic claw{
    .name = "claw",
    .positions = {{"closed", {!facts::kClawOpenLevel.value()}, grabs}, {"open", {facts::kClawOpenLevel.value()}}},
    .start = facts::unpoweredPosition(facts::kClawOpenLevel.value(), "closed", "open"),
    .safe = facts::unpoweredPosition(facts::kClawOpenLevel.value(), "closed", "open"),
    .moveTime = facts::kClawStrokeTime.value(),
};
// THE FRONT TOGGLE -- robot one's FLIPPER (the facts' kFlipperKind: a pneumatic): two forward-facing
// pistons on ONE solenoid (one line), pushing forward where the camera looks -- the autons' Toggle step
// pushes with it. Named "flipper" because robot one's flipper check finds it by that name; the team calls
// it the front toggle.
inline constexpr Pneumatic flipper{
    .name = "flipper",
    .positions = {{"closed", {!facts::kFlipperOpenLevel.value()}}, {"open", {facts::kFlipperOpenLevel.value()}}},
    .start = facts::unpoweredPosition(facts::kFlipperOpenLevel.value(), "closed", "open"),
    .safe = facts::unpoweredPosition(facts::kFlipperOpenLevel.value(), "closed", "open"),
    .moveTime = facts::kFlipperStrokeTime.value(),
};
// THE RAMP: one piston on the BACK; it comes DOWN to push other robots (the builder's "open" = down).
inline constexpr Pneumatic ramp{
    .name = "ramp",
    .positions = {{"up", {!facts::kRampDownLevel.value()}}, {"down", {facts::kRampDownLevel.value()}}},
    .start = facts::unpoweredPosition(facts::kRampDownLevel.value(), "up", "down"),
    .safe = facts::unpoweredPosition(facts::kRampDownLevel.value(), "up", "down"),
    .moveTime = facts::kRampStrokeTime.value(),
};

// Where each one is plugged in (the facts form's B6, C1, G21-G24). Re-cabling is a typed edit of the
// ledger's ports and letters.
inline constexpr MechanismTable kRobotOneMechanisms = mechanisms(
    wire(cascade, motors(facts::kCascadePorts.value().first, facts::kCascadePorts.value().second)),
    wire(rightToggle, motors(facts::kRightTogglePort.value())),
    wire(leftToggle, motors(facts::leftTogglePort())),
    wire(claw, lines(facts::kClawLine.value())),
    wire(flipper, lines(facts::kFlipperLine.value())),
    wire(ramp, lines(facts::kRampLine.value())));
// [/robot one mechanisms]
}  // namespace robot_one

/// [R9] True when every port in `listed` (the Devices screen's non-drive motors) is a declared MOTOR
/// port of `m` (mechanism::declaredMotorPorts — exactly the ports shulib Calibrate's facts declare).
constexpr bool everyListedMotorDeclared(const robot_one_facts::PortList& listed, const MechanismTable& m) {
    int declared[mechanism::kMaxMechanisms * mechanism::kMaxMotorRoles]{};
    const std::size_t n = mechanism::declaredMotorPorts(m, std::span<int>{declared});
    for (int i = 0; i < listed.count && i < static_cast<int>(robot_one_facts::kMaxListedPorts); ++i) {
        bool found = false;
        for (std::size_t k = 0; k < n; ++k) {
            found = found || declared[k] == listed.port[i];
        }
        if (!found) {
            return false;
        }
    }
    return true;
}

/// True when `m` declares an ARM (a position mechanism in degrees) named `name` (`arm`), or a PNEUMATIC
/// named `name` (`!arm`).
constexpr bool declaresKindNamed(const MechanismTable& m, bool arm, const char* name) {
    for (std::size_t i = 0; i < m.count && i < mechanism::kMaxMechanisms; ++i) {
        const mechanism::MechanismSpec& s = m.specs[i];
        const bool isArm = s.kind == mechanism::Kind::Position && s.position.units == mechanism::Units::Degrees;
        const bool isPneumatic = s.kind == mechanism::Kind::Pneumatic;
        if ((arm ? isArm : isPneumatic) && robot_one_facts::same(s.name, name)) {
            return true;
        }
    }
    return false;
}

/// True when `m` declares any mechanism named `name`.
constexpr bool declaresNamed(const MechanismTable& m, const char* name) {
    for (std::size_t i = 0; i < m.count && i < mechanism::kMaxMechanisms; ++i) {
        if (robot_one_facts::same(m.specs[i].name, name)) {
            return true;
        }
    }
    return false;
}

/// The flipper's typed kind agrees with the list: a motor arm = an Arm named "flipper", a pneumatic = a
/// Pneumatic named "flipper", a passive bar or "not on the robot yet" = nothing named "flipper".
constexpr bool flipperKindMatches(robot_one_facts::FlipperKind kind, const MechanismTable& m) {
    switch (kind) {
        case robot_one_facts::FlipperKind::MotorArm:
            return declaresKindNamed(m, true, "flipper");
        case robot_one_facts::FlipperKind::Pneumatic:
            return declaresKindNamed(m, false, "flipper");
        case robot_one_facts::FlipperKind::PassiveBar:
        case robot_one_facts::FlipperKind::NotOnTheRobotYet:
            return !declaresNamed(m, "flipper");
    }
    return false;
}

/// Robot one's two fact checks' refusals, by NAME (the mechanism core's idiom: GCC prints the reason as
/// the function's name) — deliberately not constexpr.
namespace refused {
/// [R9] A motor the brain's Devices screen lists (the ledger's kNonDriveMotorPorts) is not a drive motor
/// and robot one's list does not declare it: shulib Calibrate would take it for a drive candidate and
/// POWER it (its census declares only the list's motor ports). Declare its mechanism — the flipper's block
/// included — or correct the ledger.
inline void a_motor_the_devices_screen_lists_is_not_a_drive_motor_and_is_not_declared_in_robot_ones_list() {}
/// The flipper's typed kind (the ledger's kFlipperKind) disagrees with robot one's list: a motor arm needs
/// an Arm named "flipper", a pneumatic a Pneumatic named "flipper", a passive bar or "not on the robot yet"
/// nothing named "flipper".
inline void the_flippers_typed_kind_does_not_match_robot_ones_list() {}
}  // namespace refused

/// ROBOT ONE'S FACTS AGAINST ROBOT ONE'S LIST (the two refusals above). In EVERY build — they are robot
/// one's files, compiled in every build like robot two's — so a typed fact that disagrees with the list
/// fails the host suite too, the day it is typed. test/compile_refusals/ pins both refusals by name.
consteval bool robotOneNonDriveMotorsDeclared(const robot_one_facts::PortList& listed, const MechanismTable& m) {
    if (!everyListedMotorDeclared(listed, m)) {
        refused::a_motor_the_devices_screen_lists_is_not_a_drive_motor_and_is_not_declared_in_robot_ones_list();
    }
    return true;
}
/// The flipper half (above).
consteval bool robotOneFlipperKindMatches(robot_one_facts::FlipperKind kind, const MechanismTable& m) {
    if (!flipperKindMatches(kind, m)) {
        refused::the_flippers_typed_kind_does_not_match_robot_ones_list();
    }
    return true;
}
static_assert(robotOneNonDriveMotorsDeclared(robot_one_facts::kNonDriveMotorPorts.value(),
                                             robot_one::kRobotOneMechanisms),
              "robot one: a motor the Devices screen lists is not a drive motor and is NOT DECLARED in "
              "src/robots/mechanism_table.hpp -- shulib Calibrate would take it for a drive candidate and power it");
static_assert(robotOneFlipperKindMatches(robot_one_facts::kFlipperKind.value(), robot_one::kRobotOneMechanisms),
              "robot one: the flipper's typed kind does not match robot one's list -- a motor arm needs an Arm "
              "named \"flipper\", a pneumatic a Pneumatic named \"flipper\", a passive bar or 'not on the robot yet' none");

/// Every other robot (the bench bot, the invented X-drive): no mechanism declared.
inline constexpr MechanismTable kNoMechanisms = mechanism::mechanisms();

/// Robot `id`'s mechanisms: THE ONE PLACE a robot becomes its mechanisms (the chassis table's
/// chassisTableFor() is the other half). A case per robot and no default, so a robot added to RobotId
/// (src/build_selection.hpp) without a list here is -Wswitch (a src build gate FAILURE, a host compile
/// error), and reaching the end in a constant expression is a compile error. The host suite calls it
/// for EVERY RobotId (test/mechanism_table_test.cpp).
constexpr const MechanismTable& mechanismTableFor(RobotId id) {
    switch (id) {
        case RobotId::RobotTwo:
            return robot_two::kRobotTwoMechanisms;
        case RobotId::BenchBot:
            return kNoMechanisms;
        case RobotId::InventedXDrive:
            return kNoMechanisms;
        case RobotId::RobotOne:
            return robot_one::kRobotOneMechanisms;
    }
    __builtin_unreachable();
}

/// THE BUILD'S LIST — the only name a program uses (the src build gate's step 0 fails one that names a
/// robot's own list): the mechanisms of the robot src/build_selection.hpp's one chain chose for this
/// build. No #if: a misspelt robot here is a compile error. (Never a robot's own list's name: step 0
/// would then read the build's selection as a robot's — the MECH brief's landmine 13.)
inline constexpr const MechanismTable& kMechanisms = mechanismTableFor(kRobotId);

/// THE DRIVE CHECK: no ROLE of `m` — a motor, an angle sensor, a distance or optical piece sensor —
/// on a drive port of `c` (either side, by |port|) or on `c`'s IMU port, and the list passes every
/// wiring rule (a mechanisms(...) list always does). It is the library's one rule,
/// mechanism::clearOfDrive (wiring.hpp — the build-time mirror of config::RobotConfig::valid(), which
/// owns every collision at run time), fed this robot's typed chassis table. A 3-wire role is a
/// different connector the chassis table never names. (Until MECH-A5 this was
/// `mechanismsDisjointFromChassis`, and it checked the MOTOR ports only — the MECH brief's §1.13.)
constexpr bool mechanismsClearOf(const MechanismTable& m, const ChassisTable& c) {
    std::array<std::uint8_t, 2 * static_cast<std::size_t>(kMaxPort)> drive{};
    std::size_t n = 0;
    for (std::size_t i = 0; i < c.leftCount && i < static_cast<std::size_t>(kMaxPort); ++i) {
        drive[n++] = static_cast<std::uint8_t>(absPort(c.left[i]));
    }
    for (std::size_t i = 0; i < c.rightCount && i < static_cast<std::size_t>(kMaxPort); ++i) {
        drive[n++] = static_cast<std::uint8_t>(absPort(c.right[i]));
    }
    return mechanism::clearOfDrive(m, std::span<const std::uint8_t>{drive.data(), n}, c.imuPort);
}

/// The drive check's refusal, by NAME (the mechanism core's idiom: GCC prints the reason as the
/// function's name) — deliberately not constexpr.
namespace refused {
/// A mechanism role is wired to a drive port, or to the IMU's, of its robot's own chassis table.
inline void a_mechanism_is_wired_to_a_drive_port_or_the_imu_port_of_its_robots_chassis_table() {}
}  // namespace refused

/// THE DRIVE CHECK AT BUILD (the static_asserts below): `m` beside its OWN robot's chassis table `c`;
/// a role on a drive port or the IMU's FAILS THE BUILD by the name above (the compile-refusal harness
/// pins it: test/compile_refusals/n31_*). True otherwise.
consteval bool clearOfItsDrive(const MechanismTable& m, const ChassisTable& c) {
    if (!mechanismsClearOf(m, c)) {
        refused::a_mechanism_is_wired_to_a_drive_port_or_the_imu_port_of_its_robots_chassis_table();
    }
    return true;
}

/// True when PHYSICAL smart port `port` (1..21) carries a declared mechanism MOTOR in `t` — the same
/// set of ports the standard Calibrate's facts declare for this list (src/robots/calibrate_facts.hpp,
/// through mechanism::declaredMotorPorts(); until CAL1 Part B robot two's own Calibrate dropped its
/// census ports by this predicate). A Reversed
/// motor is still its physical port (the wiring holds physical ports; the direction is the
/// declaration's); a port outside 1..21 is never one; a 3-wire line is NEVER a smart port (line A is not
/// port 1). MOTOR roles only, as before MECH-A5: the census lists the ports that answer as MOTORS, and a
/// declared smart SENSOR port is not one. (The boot's declared-port rule reads the sensor ports too —
/// src/common/boot_source.hpp.)
constexpr bool isDeclaredMechanismPort(const MechanismTable& t, int port) {
    if (port < 1 || port > kMaxPort) {
        return false;
    }
    for (std::size_t i = 0; i < t.count && i < mechanism::kMaxMechanisms; ++i) {
        for (std::size_t m = 0; m < t.specs[i].motorCount && m < mechanism::kMaxMotorRoles; ++m) {
            if (t.wiring[i].motors[m] == port) {
                return true;
            }
        }
    }
    return false;
}

// ═══ THE COMPILE-TIME CHECKS ═══════════════════════════════════════════════════════════════
// Every declaration and every list checked itself as it was written (mechanisms(...)). What only this
// file can check is each list beside its robot's DRIVE — robot two's real pair in every build, and the
// build's own pair:
static_assert(clearOfItsDrive(robot_two::kRobotTwoMechanisms, kRobotTwoChassis),
              "robot two: a mechanism role is on one of robot two's DRIVE ports (or its IMU port)");
static_assert(clearOfItsDrive(robot_one::kRobotOneMechanisms, kRobotOneChassis),
              "robot one: a mechanism role is on one of robot one's DRIVE ports (or its IMU port)");
static_assert(clearOfItsDrive(kMechanisms, kChassis),
              "a mechanism role is on a DRIVE port (or the IMU port) of this build's chassis table");
// THE SELECTION, restated as a check — by ADDRESS, never by a count (G3a's ask, the MECH brief's §6.2
// item 4: a generated list may hold any number). A wrong robot in mechanismTableFor() (or kMechanisms
// bound to a fixed robot instead of kRobotId) would compile cleanly and hand robot two an EMPTY list —
// Calibrate would then take its claws as drive candidates again, HA-111's defect class (a build carrying
// another robot's facts). A typo would have to be made twice, in two places, to get past these.
static_assert(&kMechanisms == &mechanismTableFor(kRobotId),
              "kMechanisms must be mechanismTableFor(kRobotId): this build's robot, chosen by the one chain");
static_assert(kRobotId != RobotId::RobotTwo || &kMechanisms == &robot_two::kRobotTwoMechanisms,
              "ROBOT=tank must select robot two's mechanisms");
static_assert(kRobotId != RobotId::RobotOne || &kMechanisms == &robot_one::kRobotOneMechanisms,
              "ROBOT=one must select robot one's mechanisms");
static_assert((kRobotId != RobotId::BenchBot && kRobotId != RobotId::InventedXDrive) ||
                  &kMechanisms == &kNoMechanisms,
              "the bench bot and the invented X-drive declare no mechanisms");

}  // namespace shulib::bench
