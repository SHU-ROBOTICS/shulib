#pragma once
//
// mechanism_fixtures.hpp — robots written in the mechanism core's words, for the host tests and the
// compile-refusal harness (chunk MECH-A1, 2026-10-08).
//
// FOUR ROBOTS, each in its own namespace, each declared EXACTLY as a team writes a robot file
// (the MECH brief's §1.1, compiled verbatim):
//   * robot x  — an INVENTED robot with one of each kind: an arm, a two-motor lift in inches, a
//                flywheel, an intake with a distance sensor, a three-position hood on two lines.
//   * robot two — the 2026 tank's claws in the new words, exactly the region MECH-A5 writes into
//                src/robots/mechanism_table.hpp (its caveat comments included). Its `.moveTime`
//                is MECH-A5's provisional 0.5 s (held for A5 as HA-185) — here a fixture value.
//   * robot y  — EVERY optional word (the brief's P5b): an absolute-sensor wrist with a reversed
//                rotation sensor, unhomedVolts and .tuning; a stall-homed arm; a DR4B linkage; a
//                measured, never-homed tilt; an optical roller; a limit-switch catapult; a grip with
//                grabs / releases and a distance sensor.
//   * robot z  — the brief's P4 census robot: robot x's own declarations, plus a wrist with an angle
//                sensor and robot two's first claw pivot (the boot binding check, valid()'s P4b).
// Nothing here is a real robot's table: src/robots/mechanism_table.hpp is untouched until MECH-A5,
// and the `src` build gate's per-robot rule exempts test/. Every `.name` line below is a line the
// tests pin (by the SUFFIX of this file's path — the path is the one the compiler opened): EDIT
// THIS FILE BELOW ITS LAST NAMESPACE, or move the pins with it.
//
// The namespaces sit OUTSIDE shulib on purpose: `using namespace shulib::mechanism::words;` makes
// the words visible at the global scope's level for name lookup, so nothing in shulib::bench (whose
// src/robots/mechanism_table.hpp still declares its own `MechanismTable` until MECH-A5) can be
// found first.

#include "shulib/mechanism/kinds.hpp"
#include "shulib/mechanism/wiring.hpp"

namespace mech_fixture::robot_x {
using namespace shulib::mechanism::words;

// ═══ WHAT EACH MECHANISM IS ═════════════════════════════════════════════════════════════
// An ARM with three presets and gravity compensation (12 lines)
inline constexpr Arm arm{
    .name = "arm",
    .motors = {Forward},
    .cartridge = Green,
    .gearing = gears(12, 60),         // a 12-tooth gear on the motor drives a 60 on the arm
    .safe = Hold,
    .stall = {.amps = 2.0, .belowMotorRpm = 10, .lasting = 0.5_s},
    .range = {0, 160},                // degrees: down on its stop .. just past vertical
    .gravity = cosine(1.6_volt, 45),  // 1.6 V holds it out level; it is level at 45 degrees
    .home = restsAtStop(0),           // it rests on its stop at 0 when the program starts
    .presets = {{"stow", 0}, {"score", 95}, {"high", 140}},
};
// A two-motor LIFT in inches (13 lines)
inline constexpr Lift lift{
    .name = "lift",
    .motors = {Forward, Reversed},    // the right-hand motor is mounted mirrored
    .cartridge = Red,
    .gearing = direct,
    .shape = spool(1.5_in),
    .safe = Hold,
    .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s},
    .range = {0_in, 24_in},
    .gravity = constantGravity(1.0_volt),
    .home = homeBySwitch(0_in, -3_volt),  // down at 3 V until its switch is pressed: that is 0 in
    .presets = {{"floor", 0_in}, {"low", 8_in}, {"high", 22_in}},
};
// A FLYWHEEL with "at speed" (10 lines)
inline constexpr Flywheel flywheel{
    .name = "flywheel",
    .motors = {Forward},
    .cartridge = Blue,
    .gearing = gears(72, 12),         // geared UP 6:1, so 3600 rpm free
    .safe = Coast,
    .stall = {.amps = 2.0, .belowMotorRpm = 30, .lasting = 0.5_s},
    .atSpeed = within(60, 0.1_s),     // "at speed": within 60 rpm for 0.1 s
    .presets = {{"shoot", 3000}},
};
// An INTAKE that can run until its sensor sees a piece (7 lines)
inline constexpr Power intake{
    .name = "intake",
    .motors = {Forward},
    .cartridge = Blue,
    .pieceSensor = distanceSensor(2_in),  // it "sees a piece" closer than 2 in
    .safe = Coast,
};
// A THREE-POSITION pneumatic (7 lines)
inline constexpr Pneumatic hood{
    .name = "hood",
    .positions = {{"down", {Low, Low}}, {"middle", {High, Low}}, {"up", {High, High}}},
    .start = "down",
    .safe = "down",
    .moveTime = 0.3_s,
};

// ═══ WHERE EACH ONE IS PLUGGED IN (re-cabling changes only these lines) ═══════════════════
inline constexpr MechanismTable kRobotXMechanisms = mechanisms(
    wire(arm, motors(1)),
    wire(lift, motors(2, 3), homeSwitch('B')),
    wire(flywheel, motors(4)),
    wire(intake, motors(5), pieceSensor(9)),
    wire(hood, lines('C', 'D')));
}  // namespace mech_fixture::robot_x

namespace mech_fixture::robot_two {
using namespace shulib::mechanism::words;
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
    .moveTime = 0.5_s,  // a fixture value: robot two's is MECH-A5's (held as HA-185; Station J times it)
};

// Where each one is plugged in. Re-cabling changes only these lines. A claw cable swapped with a
// drive cable is invisible to the boot (HA-155): label the cables.
inline constexpr MechanismTable kRobotTwoMechanisms = mechanisms(
    wire(clawPivot6, motors(6)),
    wire(clawPivot7, motors(7)),
    wire(claw, lines('A')));
// [/robot two mechanisms]
}  // namespace mech_fixture::robot_two

namespace mech_fixture::robot_y {
using namespace shulib::mechanism::words;
inline constexpr HeightAt kTilt[] = {{0_in, 0}, {6_in, 30}, {14_in, 70}};
inline constexpr Arm wrist{.name = "wrist", .motors = {Forward}, .cartridge = Green, .gearing = gears(12, 36), .angleSensor = rotationSensor(Reversed), .safe = Hold, .stall = {.amps = 1.6, .belowMotorRpm = 3, .lasting = 0.15_s}, .range = {-90, 90}, .gravity = noGravity, .home = absoluteSensor(0, 180), .presets = {{"flat", 0}}, .unhomedVolts = 4_volt, .tuning = {.kP = 0.3, .within = 2.0}};
// swing's HOMING signature is 0.45 A, not the jam recipe's: a 3 V homing push draws at most about 3/12
// of the 2.5 A limit against its stop on the host test plant (0.62 A), so the 1.5 A it stated until
// MECH-A3's review could never trip — every home() ended TIMED_OUT (the A3 position test homes it)
inline constexpr Arm swing{.name = "swing", .motors = {Forward}, .cartridge = Red, .gearing = gears(12, 84), .safe = Brake, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0, 120}, .gravity = cosine(1.0_volt, 30), .home = homeByStall(0, -3_volt, {.amps = 0.45, .belowMotorRpm = 2, .lasting = 0.2_s}), .presets = {{"down", 0}, {"up", 110}}};
inline constexpr Lift dr4b{.name = "dr4b", .motors = {Forward, Reversed}, .cartridge = Red, .gearing = gears(12, 60), .shape = linkage(14_in, 10_in, 2), .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0_in, 30_in}, .gravity = constantGravity(1.2_volt), .home = restsAtStop(0_in), .presets = {{"low", 4_in}}};
inline constexpr Lift tilt{.name = "tilt", .motors = {Forward}, .cartridge = Red, .gearing = gears(12, 84), .shape = measured(kTilt), .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0_in, 14_in}, .gravity = noGravity, .home = neverHomed, .presets = {{"mid", 6_in}}, .unhomedVolts = 3_volt};
inline constexpr Power roller{.name = "roller", .motors = {Forward}, .cartridge = Blue, .pieceSensor = opticalSensor(0.5), .safe = Coast};
inline constexpr Power catapult{.name = "catapult", .motors = {Forward}, .cartridge = Red, .pieceSensor = limitSwitch(), .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 2, .lasting = 0.2_s}};
inline constexpr Pneumatic grip{.name = "grip", .positions = {{"shut", {High}, grabs}, {"loose", {Low}, releases}},
                                .start = "loose", .safe = "shut", .moveTime = 0.25_s,
                                .pieceSensor = distanceSensor(1.5_in), .confirmWithin = 0.2_s};

inline constexpr MechanismTable kRobotYMechanisms = mechanisms(
    wire(wrist, motors(1), angleSensor(10)),
    wire(swing, motors(2)),
    wire(dr4b, motors(3, 4)),
    wire(tilt, motors(5)),
    wire(roller, motors(6), pieceSensor(9)),
    wire(catapult, motors(7), pieceSensor('E')),
    wire(grip, lines('A'), pieceSensor(8)));
}  // namespace mech_fixture::robot_y

namespace mech_fixture::robot_z {
using namespace shulib::mechanism::words;
using namespace mech_fixture::robot_x;  // arm, lift, flywheel, intake, hood
// P4's robot z: robot x's mechanisms plus an absolute-sensor wrist and robot two's first pivot — the
// boot binding check's census robot (the brief's P4) and valid()'s P4b.
inline constexpr Arm wrist{.name = "wrist", .motors = {Forward}, .cartridge = Green, .gearing = gears(12, 36),
                           .angleSensor = rotationSensor(), .safe = Hold,
                           .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {-90, 90},
                           .gravity = noGravity, .home = absoluteSensor(0, 180)};
inline constexpr MechanismTable kRobotZMechanisms = mechanisms(
    wire(arm, motors(1)),
    wire(lift, motors(2, 3), homeSwitch('B')),
    wire(flywheel, motors(4)),
    wire(intake, motors(5), pieceSensor(9)),
    wire(hood, lines('C', 'D')),
    wire(wrist, motors(6), angleSensor(10)),
    wire(mech_fixture::robot_two::clawPivot6, motors(7)));
}  // namespace mech_fixture::robot_z
