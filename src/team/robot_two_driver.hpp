#pragma once
//
// robot_two_driver.hpp — ROBOT TWO'S DRIVER CONTROL: what every stick and button does.
//
// This is the team's driver code (chunk T4; in the mechanism core's words since MECH-A5,
// 2026-10-29). Every program that hands robot two to a driver runs THIS file for it: "shulib Drive"
// on its own, and the competition program's opcontrol(). To change what a button does, change it
// HERE and nowhere else. What the robot IS -- its ports, its cartridges, that a claw pivot is put in
// Hold when let go, that the claw starts CLOSED and which line level closes it -- is in
// src/robots/mechanism_table.hpp, and is not the driver's to choose.
//
// It holds ONLY the team's decisions: which button does what, the feel, and the loop's two numbers.
// How it runs -- binding the buttons to the robot's mechanism service, the controller, the call into
// the library's driver loop -- is written once, in src/common/driver_file.hpp, for every robot's
// driver file. Nothing here draws on a screen or writes a log line: the program around it does both.
//
//   left stick   up / down: drive forward / back. Forward is the end the calibration record
//                calls the front (the camera end, on robot two's record of 2026-09-25).
//                Left / right does nothing: a tank cannot strafe.
//   right stick  left / right: turn
//   R1 / R2      the claw pivot on port 6: forward / back at full power while held. Let go and
//                it is put in Hold (whether Hold keeps a loaded claw up is unmeasured, HA-156)
//   L1 / L2      the claw pivot on port 7: the same
//   A            the claw: each press opens or closes it. The FIRST time driver control starts
//                in a run, the claw is told CLOSED, whatever autonomous left it at (the team
//                lead's ruling, 2026-09-26); a program with no autonomous has left it CLOSED,
//                so there it does not move. After that, a disable and the next start tell it
//                nothing. Which line level closes it is a guess (HA-159)
//   Y            field-centric re-zero. Field-centric is OFF, so Y changes nothing yet
//   let go of the sticks and the drive coasts (Coast is set on every drive motor)
//   (no presets: a preset sends a mechanism to a named position on one press -- `preset(arm["score"],
//   Button::Right, 1.5_s)` -- and robot two's pivots have no positions yet, the team lead's Q6)
//
// The library protects the drive under this file, in every program (teleop/drive_protection.hpp):
// a drive motor that stops answering is dropped, the robot drives on, and the driver is told --
// but once more than two are dead, the drive stays at 0 V for the rest of the run, every later
// start of driver control included, until the program is started again. A drive motor fighting
// its side, or drawing more than 2.4 A while it is driven, for 250 ms stops BOTH sides for 1 s,
// and then driving resumes. None of those numbers has been measured (HA-132, HA-175, HA-176).
//
// NOTHING HERE HAS RUN ON A ROBOT. It is tested on the host, through the PROS shim, against
// robot two's real mechanisms (test/robot_two_driver_test.cpp).

#include "common/driver_file.hpp"
#include "robots/mechanism_table.hpp"

namespace shulib::bench::robot_two {
// [driver file two]
// ═══ WHICH BUTTON DOES WHAT ═══════════════════════════════════════════════════════════════
inline constexpr DriverControls kControls = controls(kRobotTwoMechanisms,
    whileHeld(clawPivot6, Button::R1, Button::R2, 12_volt),  // forward / back, full power
    whileHeld(clawPivot7, Button::L1, Button::L2, 12_volt),  // forward / back, full power
    toggle(claw, Button::A));                                // each press opens or closes it
// [/driver file two]

// ═══ THE FEEL ═════════════════════════════════════════════════════════════════════════════
// The library's default: arcade (left stick drives, right stick turns), a smooth deadband of 0.05,
// no curve, no ramp -- numbers no driver has felt yet (HA-112). Field-centric is OFF.
inline constexpr DriverProfile kFeel{.mode = ArcadeBody};
inline constexpr DriverLoopConfig kLoop{
    .rezeroButton = Button::Y,     // field-centric's "away from me"
    .releaseBrakeMode = Coast,     // let go of the sticks: the drive coasts
};
}  // namespace shulib::bench::robot_two

namespace shulib::bench {
using RobotTwoDriver = DriverFile<robot_two::kControls, robot_two::kFeel, robot_two::kLoop>;
}  // namespace shulib::bench
