#pragma once
//
// robot_one_driver.hpp — ROBOT ONE'S DRIVER CONTROL: what every stick and button does.
//
// ═══ A DEFAULT — THE TEAM'S TO CHANGE ═════════════════════════════════════════════════════
// This file was written by the library's builder (chunk ROB1-A, 2026-11-11; the ROB1 brief's D7), NOT
// by the team, for one reason only: no robot-one program builds without a driver file — the library
// REFUSES a declared mechanism with no control (mechanism::controls(...)). The builder said "buttons can
// come later". So this is a STARTING map, chosen to match robot two's feel, and from the day it lands it
// is the STUDENTS' code (the guardrail G4): change any line, and be able to explain why. "shulib Drive"
// says so on its boot line.
//
// Every program that hands robot one to a driver runs THIS file for it: "shulib Drive" on its own, and
// the competition program's opcontrol(). What the robot IS — the cascade's ports, its motors' directions,
// its levels and their heights, that it holds when let go — is in src/robots/mechanism_table.hpp (robot
// one's region) and src/robots/robot_one_facts.hpp, and is not the driver's to choose. How it runs — the
// controller, the call into the library's driver loop — is src/common/driver_file.hpp, shared with
// robot two's file.
//
//   left stick   up / down: drive forward / back (toward the calibration record's front).
//                Left / right does nothing: a tank cannot strafe.
//   right stick  left / right: turn
//   L1 / L2      the cascade up / down at full power while held. Before it is homed a held button gives
//                it at most its unhomed volts (the robot file's); once homed it is soft-stopped at its
//                bottom and top stops by the library.
//   d-pad Down   the cascade to its FIRST level  (the facts ledger's level slot 0 — "floor" until typed)
//   d-pad Left   the cascade to its SECOND level (slot 1 — "L1" until typed)
//   d-pad Up     the cascade to its THIRD level  (slot 2 — "L2" until typed)
//   d-pad Right  the cascade to its FOURTH level (slot 3 — "L3" until typed)
//                (each bounded at 2 s; a level is named by the LEDGER, so renaming a level is a ledger
//                edit, never an edit here. The map needs at least four levels typed.)
//   B            home the cascade: it stalls gently DOWN into its bottom stop under its current cap
//   (start)      the FIRST time driver control starts in a run, the cascade homes (bounded at 3 s)
//   R1 / R2      FREE until the flipper's kind is known (the brief's D6): a motor arm takes R1 / R2 while
//                held, a pneumatic R1 to toggle — guide chapter 21 has the lines to paste
//   Y            field-centric re-zero. Field-centric is OFF, so Y changes nothing yet
//   let go of the sticks and the drive coasts (Coast is set on every drive motor)
//
// The library protects the drive under this file exactly as it does robot two's (teleop/
// drive_protection.hpp; none of its numbers measured — HA-132, HA-175, HA-176).
//
// NOTHING HERE HAS RUN ON A ROBOT; robot one does not exist in this project's records. It is tested on
// the host, through the PROS shim, against robot one's real declarations on their PLACEHOLDER numbers
// (test/robot_one_driver_test.cpp).

#include "common/driver_file.hpp"
#include "robots/mechanism_table.hpp"

namespace shulib::bench::robot_one {
// [driver file one]
// ═══ WHICH BUTTON DOES WHAT ═══════════════════════════════════════════════════════════════
inline constexpr DriverControls kControls = controls(kRobotOneMechanisms,
    whileHeld(cascade, Button::L1, Button::L2, 12_volt),        // up / down, full power
    preset(cascade[facts::lv(0).name], Button::Down, 2_s),      // the first level
    preset(cascade[facts::lv(1).name], Button::Left, 2_s),      // the second level
    preset(cascade[facts::lv(2).name], Button::Up, 2_s),        // the third level
    preset(cascade[facts::lv(3).name], Button::Right, 2_s),     // the fourth level
    homeOnPress(cascade, Button::B, 3_s),                       // home it now
    homeAtDriverStart(cascade, 3_s));                           // home it at the first driver start
// [/driver file one]

// ═══ THE FEEL ═════════════════════════════════════════════════════════════════════════════
// The library's default, as robot two's: arcade (left stick drives, right stick turns), a smooth
// deadband of 0.05, no curve, no ramp -- numbers no driver has felt yet (HA-112). Field-centric is OFF.
inline constexpr DriverProfile kFeel{.mode = ArcadeBody};
inline constexpr DriverLoopConfig kLoop{
    .rezeroButton = Button::Y,     // field-centric's "away from me"
    .releaseBrakeMode = Coast,     // let go of the sticks: the drive coasts
};
}  // namespace shulib::bench::robot_one

namespace shulib::bench {
using RobotOneDriver = DriverFile<robot_one::kControls, robot_one::kFeel, robot_one::kLoop>;
}  // namespace shulib::bench
