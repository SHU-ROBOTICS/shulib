#pragma once
//
// mechanism_driver_fixtures.hpp — DRIVER FILES' controls in the mechanism core's words, for the host
// tests and the compile-refusal harness (chunk MECH-A4, 2026-10-22).
//
// Each list is written EXACTLY as a team writes a driver file (the MECH brief's §1.1, compiled
// verbatim), checked at build against its robot's list in test/mechanism_fixtures.hpp:
//   * robot x  — the brief's driver file: a held arm with three presets, a held lift homed at driver
//                start, a flywheel toggled at its named speed, a held intake, the hood autonOnly; and
//                its alternative (the hood by presets, the lift homed on a press);
//   * robot two — robot two's claws as its driver file states them since MECH-A5: two held pivots, a
//                toggled claw — the controls statement byte for byte the real file's
//                (test/robot_two_driver_test.cpp pins the two equal);
//   * robot y  — every optional control word at once (the brief's P5b).
// Nothing here is a real robot's driver file (robot two's is src/team/robot_two_driver.hpp).

#include "mechanism_fixtures.hpp"
#include "shulib/mechanism/driver_inputs.hpp"
#include "shulib/teleop/teleop_loop.hpp"

namespace mech_fixture::robot_x {
// The driver file's controls (the brief's §1.1, verbatim).
inline constexpr DriverControls kControls = controls(kRobotXMechanisms,
    whileHeld(arm, Button::R1, Button::R2, 6_volt),  // up / down while held; let go: it holds there
    preset(arm["stow"], Button::Down, 1.5_s),
    preset(arm["score"], Button::Right, 1.5_s),      // the most recent button wins (Q8)
    preset(arm["high"], Button::Up, 1.5_s),
    whileHeld(lift, Button::L1, Button::L2, 12_volt),
    homeAtDriverStart(lift, 1.5_s),                  // after a restart it may not be on its switch
    toggle(flywheel["shoot"], Button::X),            // on at "shoot" / off
    whileHeld(intake, Button::A, Button::B, 12_volt),
    autonOnly(hood));                                // the hood is the autons' alone
// The alternative (the brief's counted steps): the hood by presets, the lift homed on a press.
inline constexpr DriverControls kAltControls = controls(kRobotXMechanisms,
    whileHeld(arm, Button::R1, Button::R2, 6_volt),
    whileHeld(lift, Button::L1, Button::L2, 12_volt),
    homeOnPress(lift, Button::Left, 1.5_s),
    toggle(flywheel["shoot"], Button::X),
    whileHeld(intake, Button::A, Button::B, 12_volt),
    preset(hood["down"], Button::Down),
    preset(hood["middle"], Button::Right),
    preset(hood["up"], Button::Up));
// The loop's numbers (the re-zero on Y, which no control takes).
inline constexpr DriverLoopConfig kLoop{.tickPeriod = 10_ms, .rezeroButton = Button::Y, .releaseBrakeMode = Coast};
}  // namespace mech_fixture::robot_x

namespace mech_fixture::robot_two {
// [driver file two] — the controls (4 lines), as MECH-A5 writes robot two's driver file.
inline constexpr DriverControls kControls = controls(kRobotTwoMechanisms,
    whileHeld(clawPivot6, Button::R1, Button::R2, 12_volt),  // forward / back, full power
    whileHeld(clawPivot7, Button::L1, Button::L2, 12_volt),  // forward / back, full power
    toggle(claw, Button::A));                                // each press opens or closes it
inline constexpr DriverProfile kFeel{.mode = ArcadeBody};
inline constexpr DriverLoopConfig kLoop{.tickPeriod = 10_ms, .rezeroButton = Button::Y, .releaseBrakeMode = Coast};
}  // namespace mech_fixture::robot_two

namespace mech_fixture::robot_y {
// EVERY optional control word (the brief's P5b): a preset by name and a homing on a press beside a
// homing at driver start, a never-homed lift held at its unhomedVolts, an autonOnly roller, a
// pneumatic preset with no time limit.
inline constexpr DriverControls kControls = controls(kRobotYMechanisms,
    whileHeld(wrist, Button::R1, Button::Y, 6_volt),
    preset(wrist["flat"], Button::L1, 1_s),
    homeOnPress(swing, Button::Right, 1_s),
    homeAtDriverStart(swing, 1.5_s),
    preset(swing["up"], Button::Up, 1.5_s),
    whileHeld(dr4b, Button::L2, Button::R2, 12_volt),
    whileHeld(tilt, Button::Down, Button::Left, 3_volt),
    autonOnly(roller),
    whileHeld(catapult, Button::X, Button::B, 12_volt),
    preset(grip["shut"], Button::A));
}  // namespace mech_fixture::robot_y
