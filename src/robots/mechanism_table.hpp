#pragma once
//
// THE MECHANISM TABLE — every robot's MECHANISMS, in the mechanism core's words (chunk MECH-A5,
// 2026-10-29; chunk T3 wrote the first table on 2026-09-26). Each robot's mechanisms are DECLARATIONS —
// what each one IS (mechanism/kinds.hpp: an Arm, a Lift, a Flywheel, a Power roller, a Pneumatic) —
// followed by ONE LIST that says where each is plugged in (mechanism/wiring.hpp: `wire(claw, lines('A'))`),
// all inside that robot's own NAMESPACE (`robot_one`, `robot_two`), so two robots' `cascade`s never collide
// in this one file every build compiles. The PROS-free sibling of src/robots/chassis_table.hpp; selected,
// like it, by kRobotId — which src/build_selection.hpp's ONE chain sets with the program and the beacon.
// THIS FILE HAS NO #if OF ITS OWN (the team lead's ruling "one chain picks both"); the src build gate's
// step 0 fails any build switch named here.
//
// TWO ROBOTS, ONE BUILD (the team lead, 2026-11-13: "IT IS LITERALLY THE SAME ROBOT BUILD JUST A MIRRORED
// ROUTINE"). Robot one's region is the design: a string cascade, two side toggles, and three pistons (the
// claw, the front toggle — the flipper — and the ramp). Robot two's region is ITS TWIN, token for token,
// with `robot_one` -> `robot_two` (test/robot_twin_test.cpp fails the host suite on any other difference).
// Each region's NUMBERS come from its own robot's facts file — src/robots/robot_one_facts.hpp,
// src/robots/robot_two_facts.hpp — read as `facts::...`, so the two robots differ ONLY in their facts.
// Robot two's old claw build (two claw pivots on ports 6 and 7, a pneumatic claw on 3-wire A) was RETIRED
// at ROB1-BT (2026-11-16); its region lives on as a host-test fixture (test/mechanism_fixtures.hpp's
// mech_fixture::robot_two).
//
// WHAT IT DECLARES — what the robot IS, never which button does what (that is each robot's DRIVER FILE:
// src/team/robot_one_driver.hpp, src/team/robot_two_driver.hpp). A value is NEVER INVENTED: a field a
// declaration needs and does not state FAILS THE BUILD, by name (mechanism/spec.hpp's refusals), and so do
// a port used twice, a 3-wire letter where a smart port goes, a level count that does not match the
// wiring (guide chapter 18, "reading a refusal"). A motor that turns the wrong way is fixed in its robot's
// facts file (its direction) — never by swapping buttons in a driver file.
//
// THE HOST SUITE TESTS THESE VERY BYTES. Every robot's list is defined in EVERY build under its own name
// (robot_one::kRobotOneMechanisms, robot_two::kRobotTwoMechanisms), and only the selection —
// mechanismTableFor(kRobotId), below — depends on the build. A program uses kMechanisms, never a robot's
// own list or namespace: the src build gate's step 0 fails a program that names a robot's list, its
// namespace or mechanismTableFor() outside a static_assert — the namespace is allowed only here, in
// test/, and in that robot's OWN team files (src/team/robot_<n>_*, and src/team/autons.{hpp,cpp} for
// robot two).
//
// WHO READS IT: every program that BOOTS a robot from its record (src/common/boot_source.hpp derives the
// config's mechanisms from the list, in the list's order, and refuses a record that drives a declared
// port); each robot's driver file (its controls are checked against the list at build); the team's
// autons (`using namespace robot_one;` — a step names a declaration: `r.set(claw["open"])`); and "shulib
// Calibrate" (src/robots/calibrate_facts.hpp: every declared motor port is dropped from its census
// before any motor is made, so a mechanism motor is never a drive candidate).
//
// G3a's GENERATOR may later take each region over: the lines between a region's two marker comments
// (robot one's, robot two's) are one contiguous block with no hand-written line inside.
//
// PROS-FREE ON PURPOSE: the host test includes it from src/. NOTHING HERE HAS RUN ON A ROBOT.

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "build_selection.hpp"
#include "robots/chassis_table.hpp"
#include "robots/robot_one_facts.hpp"
#include "robots/robot_two_facts.hpp"
#include "shulib/mechanism/kinds.hpp"
#include "shulib/mechanism/wiring.hpp"

namespace shulib::bench {

/// A robot's list of mechanisms — the LIBRARY's own type (mechanism/wiring.hpp), named here without an
/// alias so the src build gate's step 0 reads `constexpr MechanismTable <name>` as it always has.
using mechanism::MechanismTable;

// ═══ ROBOT ONE — the 2026 CASCADE robot (chunk ROB1-A, 2026-11-11; its real mechanisms ROB1-C2, 2026-11-15) ═══
// Its numbers come from ITS FACTS FILE, src/robots/robot_one_facts.hpp — read as `facts::...` below — where
// every number nobody has given yet is OWED: a placeholder the host suite runs on, which REFUSES every
// robot-one build of a program that needs it, by name. Its flipper is the FRONT TOGGLE, a pneumatic
// (ROB1-C2). Its facts are typed from its builder's answers (2026-11-15), not measured.
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
// there once a program's service binds, and at Bench Tests' station 13 while it powers it (S13UX).
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

// ═══ ROBOT TWO — robot one's TWIN (chunk ROB1-BT, 2026-11-16) ══════════════════════════════════
// The same build: this region is robot one's, token for token, with `robot_one` -> `robot_two`
// (test/robot_twin_test.cpp). Read robot one's region above for what each mechanism is. Its numbers come
// from src/robots/robot_two_facts.hpp — every one OWED until the builder reads it off robot two, so every
// robot-two build refuses, by name, until then.
namespace robot_two {
using namespace mechanism::words;
namespace facts = robot_two_facts;
// [robot two mechanisms]
// THE CASCADE (robot one's).
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

// THE SIDE TOGGLES (robot one's).
inline constexpr Power rightToggle{.name = "right toggle", .motors = {facts::kRightToggleDirection.value()},
                                   .cartridge = facts::kSideToggleCartridge.value(), .safe = Brake,
                                   .currentCap = {.amps = facts::kSideToggleCapAmps.value()}};
inline constexpr Power leftToggle{.name = "left toggle", .motors = {facts::kLeftToggleDirection.value()},
                                  .cartridge = facts::kSideToggleCartridge.value(), .safe = Brake,
                                  .currentCap = {.amps = facts::kSideToggleCapAmps.value()}};

// THE PISTONS (robot one's): the claw, the front toggle (the flipper), the ramp.
inline constexpr Pneumatic claw{
    .name = "claw",
    .positions = {{"closed", {!facts::kClawOpenLevel.value()}, grabs}, {"open", {facts::kClawOpenLevel.value()}}},
    .start = facts::unpoweredPosition(facts::kClawOpenLevel.value(), "closed", "open"),
    .safe = facts::unpoweredPosition(facts::kClawOpenLevel.value(), "closed", "open"),
    .moveTime = facts::kClawStrokeTime.value(),
};
inline constexpr Pneumatic flipper{
    .name = "flipper",
    .positions = {{"closed", {!facts::kFlipperOpenLevel.value()}}, {"open", {facts::kFlipperOpenLevel.value()}}},
    .start = facts::unpoweredPosition(facts::kFlipperOpenLevel.value(), "closed", "open"),
    .safe = facts::unpoweredPosition(facts::kFlipperOpenLevel.value(), "closed", "open"),
    .moveTime = facts::kFlipperStrokeTime.value(),
};
inline constexpr Pneumatic ramp{
    .name = "ramp",
    .positions = {{"up", {!facts::kRampDownLevel.value()}}, {"down", {facts::kRampDownLevel.value()}}},
    .start = facts::unpoweredPosition(facts::kRampDownLevel.value(), "up", "down"),
    .safe = facts::unpoweredPosition(facts::kRampDownLevel.value(), "up", "down"),
    .moveTime = facts::kRampStrokeTime.value(),
};

// Where each one is plugged in (robot two's facts file).
inline constexpr MechanismTable kRobotTwoMechanisms = mechanisms(
    wire(cascade, motors(facts::kCascadePorts.value().first, facts::kCascadePorts.value().second)),
    wire(rightToggle, motors(facts::kRightTogglePort.value())),
    wire(leftToggle, motors(facts::leftTogglePort())),
    wire(claw, lines(facts::kClawLine.value())),
    wire(flipper, lines(facts::kFlipperLine.value())),
    wire(ramp, lines(facts::kRampLine.value())));
// [/robot two mechanisms]
}  // namespace robot_two

// ═══ THIS BUILD'S ROBOT'S MECHANISMS, BY NAME (chunk ROB1-BT) ═══════════════════════════════════
// The shared skills route (src/team/skills_route.cpp) is compiled into BOTH robots' builds, so the names it
// uses must be the BUILDING robot's: after `using namespace this_robot;`, `cascade` is robot one's cascade in
// robot one's build and robot two's in robot two's, and `facts::lv(i)` reads that robot's facts. (The bench
// bot and the X-drive, which run no auton, get robot two's — as they get its autons. So does the HOST suite,
// a bench build: when it runs robot one's list, a step finds robot one's mechanism by its NAME, and a NAMED
// level only at the same height — so the twin test holds both robots' levels equal.)
// A mirror swaps the robot's SIDES, not these names: robot two runs the route mirrored, so robot one's
// `rightToggle` job is robot two's LEFT toggle — the route picks it with `competition::isMirrored(ctx)`.
namespace this_robot {
using namespace mechanism::words;  // the words a route's steps use: stalls, seesPiece, High, within ...
/// True in robot one's builds.
inline constexpr bool kRobotOneBuild = kRobotId == RobotId::RobotOne;
inline constexpr const mechanism::Lift& cascade = kRobotOneBuild ? robot_one::cascade : robot_two::cascade;
inline constexpr const mechanism::Power& rightToggle = kRobotOneBuild ? robot_one::rightToggle : robot_two::rightToggle;
inline constexpr const mechanism::Power& leftToggle = kRobotOneBuild ? robot_one::leftToggle : robot_two::leftToggle;
inline constexpr const mechanism::Pneumatic& claw = kRobotOneBuild ? robot_one::claw : robot_two::claw;
inline constexpr const mechanism::Pneumatic& flipper = kRobotOneBuild ? robot_one::flipper : robot_two::flipper;
inline constexpr const mechanism::Pneumatic& ramp = kRobotOneBuild ? robot_one::ramp : robot_two::ramp;
/// `facts::lv(i)` — the building robot's level in slot `i` (0..5).
struct facts {
    /// The level in slot `i`.
    static constexpr const robot_one_facts::Level& lv(std::size_t i) {
        return kRobotOneBuild ? robot_one_facts::lv(i) : robot_two_facts::lv(i);
    }
};
}  // namespace this_robot
// Each robot's build names its OWN (the host, a bench build, sees only robot two's side; these see both).
static_assert(kRobotId != RobotId::RobotOne || (&this_robot::cascade == &robot_one::cascade &&
                                                &this_robot::facts::lv(0) == &robot_one_facts::lv(0)),
              "robot one's build must give the shared route robot one's mechanisms (this_robot)");
static_assert(kRobotId != RobotId::RobotTwo || (&this_robot::cascade == &robot_two::cascade &&
                                                &this_robot::facts::lv(0) == &robot_two_facts::lv(0)),
              "robot two's build must give the shared route robot two's mechanisms (this_robot)");

// ═══ EACH ROBOT'S FACTS AGAINST ITS OWN LIST ════════════════════════════════════════════════════

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

/// The two fact checks' refusals, by NAME (the mechanism core's idiom: GCC prints the reason as the
/// function's name) — deliberately not constexpr. The static_assert's line says which robot.
namespace refused {
/// [R9] A motor the brain's Devices screen lists (the facts file's kNonDriveMotorPorts) is not a drive motor
/// and its robot's list does not declare it: shulib Calibrate would take it for a drive candidate and
/// POWER it. Declare its mechanism, or correct the facts file.
inline void a_motor_the_devices_screen_lists_is_not_a_drive_motor_and_is_not_declared_in_its_robots_list() {}
/// The flipper's typed kind (the facts file's kFlipperKind) disagrees with its robot's list: a motor arm
/// needs an Arm named "flipper", a pneumatic a Pneumatic named "flipper", a passive bar or "not on the
/// robot yet" nothing named "flipper".
inline void the_flippers_typed_kind_does_not_match_its_robots_list() {}
}  // namespace refused

/// A ROBOT'S FACTS AGAINST ITS OWN LIST (the two refusals above), checked for both robots in EVERY build,
/// so a typed fact that disagrees with its list fails the host suite the day it is typed.
/// test/compile_refusals/ pins both refusals by name.
consteval bool nonDriveMotorsDeclared(const robot_one_facts::PortList& listed, const MechanismTable& m) {
    if (!everyListedMotorDeclared(listed, m)) {
        refused::a_motor_the_devices_screen_lists_is_not_a_drive_motor_and_is_not_declared_in_its_robots_list();
    }
    return true;
}
/// The flipper half (above).
consteval bool flipperKindAgrees(robot_one_facts::FlipperKind kind, const MechanismTable& m) {
    if (!flipperKindMatches(kind, m)) {
        refused::the_flippers_typed_kind_does_not_match_its_robots_list();
    }
    return true;
}
static_assert(nonDriveMotorsDeclared(robot_one_facts::kNonDriveMotorPorts.value(), robot_one::kRobotOneMechanisms),
              "robot one: a motor the Devices screen lists is not a drive motor and is NOT DECLARED in "
              "src/robots/mechanism_table.hpp -- shulib Calibrate would take it for a drive candidate and power it");
static_assert(flipperKindAgrees(robot_one_facts::kFlipperKind.value(), robot_one::kRobotOneMechanisms),
              "robot one: the flipper's typed kind does not match robot one's list");
static_assert(nonDriveMotorsDeclared(robot_two_facts::kNonDriveMotorPorts.value(), robot_two::kRobotTwoMechanisms),
              "robot two: a motor the Devices screen lists is not a drive motor and is NOT DECLARED in "
              "src/robots/mechanism_table.hpp -- shulib Calibrate would take it for a drive candidate and power it");
static_assert(flipperKindAgrees(robot_two_facts::kFlipperKind.value(), robot_two::kRobotTwoMechanisms),
              "robot two: the flipper's typed kind does not match robot two's list");

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
// file can check is each list beside its robot's DRIVE — both robots' real pairs in every build, and the
// build's own pair:
static_assert(clearOfItsDrive(robot_two::kRobotTwoMechanisms, kRobotTwoChassis),
              "robot two: a mechanism role is on one of robot two's DRIVE ports (or its IMU port)");
static_assert(clearOfItsDrive(robot_one::kRobotOneMechanisms, kRobotOneChassis),
              "robot one: a mechanism role is on one of robot one's DRIVE ports (or its IMU port)");
static_assert(clearOfItsDrive(kMechanisms, kChassis),
              "a mechanism role is on a DRIVE port (or the IMU port) of this build's chassis table");
// THE SELECTION, restated as a check — by ADDRESS, never by a count (G3a's ask, the MECH brief's §6.2
// item 4: a generated list may hold any number). A wrong robot in mechanismTableFor() (or kMechanisms
// bound to a fixed robot instead of kRobotId) would compile cleanly and hand a robot an EMPTY list —
// Calibrate would then take its mechanism motors as drive candidates, HA-111's defect class (a build
// carrying another robot's facts). A typo would have to be made twice, in two places, to get past these.
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
