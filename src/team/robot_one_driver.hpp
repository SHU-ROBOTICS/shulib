#pragma once
//
// robot_one_driver.hpp — ROBOT ONE'S DRIVER CONTROL: what every stick and button does.
//
// ═══ THE BUILDER'S MAP (2026-11-15) — THE TEAM'S ═══════════════════════════════════════════
// ROB1-A wrote a DEFAULT here (the brief's D7) only because no robot-one program builds without a driver
// file. On 2026-11-15 the team's builder gave robot one's buttons, and ROB1-C2 typed EXACTLY his map —
// the cascade on R1 / R2, the claw on A, the front toggle on B, the ramp on X, the side toggles on L1 /
// L2 — so this is the TEAM'S map, the STUDENTS' code (the guardrail G4): change any line, and be able to
// explain why. Its three numbers nobody gave (the cascade's volts; the side toggles' volts; the driver-start
// home's limit) are marked "chosen, not measured" where they are written.
//
// Every program that hands robot one to a driver runs THIS file for it: "shulib Drive" on its own, and
// the competition program's opcontrol(). What the robot IS — the cascade's ports, its motors' directions,
// that it holds when let go, the side toggles' ports and which way each runs, which line opens each
// piston — is in src/robots/mechanism_table.hpp (robot one's region) and src/robots/robot_one_facts.hpp,
// and is not the driver's to choose. How it runs — the controller, the call into the library's driver
// loop — is src/common/driver_file.hpp, shared with robot two's file.
//
// [the map] — THE ONE PLACE THE MAP IS LISTED (test/robot_one_driver_test.cpp pins it against the code):
//   left stick   up / down: drive forward / back (toward the calibration record's front).
//                Left / right does nothing: a tank cannot strafe.
//   right stick  left / right: turn
//   R1           HOLD: the cascade goes UP; let go: it stops and HOLDS where it is (a string cascade must
//                not fall). Before it is homed a held button gives it at most its unhomed volts (the
//                robot file's); once homed it is soft-stopped at its bottom and top stops by the library.
//   R2           HOLD: the cascade goes DOWN; let go: it stops and holds, the same.
//   A            the claw: press = open, press again = close (each press flips it)
//   B            the front toggle (the flipper): press = open (out), press again = close (back)
//   X            the ramp: press = down (the builder's "open"), press again = up ("close")
//   L1           HOLD: the RIGHT side toggle runs (one way only); let go: it stops
//   L2           HOLD: the LEFT side toggle runs (one way only); let go: it stops
//   Y            field-centric re-zero (the drive loop's reserved button). Field-centric is OFF, so Y
//                changes nothing yet
//   d-pad        FREE — the cascade's preset heights go here later (the builder: "Later we will have
//                preset heights for the cascade but for now it will be manual"); their levels are typed
//                in the facts file first
//   (start)      the FIRST time driver control starts in a run, the cascade HOMES: it moves DOWN on its
//                own until it stalls gently on its bottom stop (at most 3 s), and R1 / R2 wait until it
//                has — a button held through it does nothing until let go. This is the ONE marked line
//                below the team can delete: without it the cascade is never homed in driver control, so
//                R1 / R2 stay at its unhomed volts with no soft stops for the whole run.
//   let go of the sticks and the drive coasts (Coast is set on every drive motor)
// The pistons start (and are made safe) in the position each holds with its solenoid UNPOWERED, so
// nothing moves at power-up; the first press moves it to the other one.
//
// The library protects the drive under this file exactly as it does robot two's (teleop/
// drive_protection.hpp; none of its numbers measured — HA-132, HA-175, HA-176).
//
// NOTHING HERE HAS RUN ON A ROBOT. It is tested on the host, through the PROS shim, against robot one's
// real declarations on its facts file's values — typed, or PLACEHOLDERS where still owed
// (test/robot_one_driver_test.cpp).

#include "common/driver_file.hpp"
#include "robots/mechanism_table.hpp"

namespace shulib::bench::robot_one {
// [driver file one]
// ═══ WHICH BUTTON DOES WHAT ═══════════════════════════════════════════════════════════════
inline constexpr DriverControls kControls = controls(kRobotOneMechanisms,
    whileHeld(cascade, Button::R1, Button::R2, 12_volt),   // R1 up / R2 down while held; let go: it holds
    toggle(claw, Button::A),                               // each press: open / close
    toggle(flipper, Button::B),                            // the front toggle -- each press: open / close
    toggle(ramp, Button::X),                               // each press: down / up
    whileHeld(rightToggle, Button::L1, 12_volt),           // held: runs (one way); let go: stops
    whileHeld(leftToggle, Button::L2, 12_volt),            // held: runs (one way); let go: stops
    homeAtDriverStart(cascade, 3_s));                      // [delete-able] home it at the first driver start
// The cascade's 12 V is CHOSEN, not measured (ROB1-A's default, kept by the coordinator's ROB1-C2 run,
// 2026-11-15): the fastest jog a blue motor driving the spool directly has — lower it if the cascade is too
// quick to stop where the driver wants (before it is homed a held button gives it at most its unhomed volts).
// The side toggles' 12 V is CHOSEN, not measured (the coordinator's ROB1-C2 run, 2026-11-15): each motor's
// current cap (the facts file's kSideToggleCapAmps, chosen too) bounds how hard it pushes held against a stop
// — whether that is gentle enough on the real arm nobody has measured. The driver-start home's 3 s is the
// library's bound for it (chosen, as ROB1-A's was).
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
