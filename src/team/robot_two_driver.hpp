#pragma once
//
// robot_two_driver.hpp — ROBOT TWO'S DRIVER CONTROL: what every stick and button does.
//
// Robot two is robot one's TWIN (chunk ROB1-BT, 2026-11-16; the team lead: "IT IS LITERALLY THE SAME ROBOT
// BUILD"), so this is robot one's driver file — the builder's map of 2026-11-15, the team's code (G4) —
// with robot two's name: one build, one map, so a driver who learns one robot drives the other. CHANGE A
// BUTTON IN BOTH FILES (src/team/robot_one_driver.hpp has every comment on why each line is what it is):
// test/robot_twin_test.cpp fails the host suite while the two files' controls and feel differ in any code
// but the robot's name, or their header maps in any word. Robot two's old claw map (the pivots on R1/R2 and L1/L2, the claw on A) was
// retired with its claws.
//
// What the robot IS is in src/robots/mechanism_table.hpp (robot two's region) and
// src/robots/robot_two_facts.hpp; how it runs is src/common/driver_file.hpp. NOTHING HERE HAS RUN ON A
// ROBOT: it is tested on the host against robot two's declarations (test/robot_two_driver_test.cpp).
//
// [the map] — the same as robot one's:
// [control words two]
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
// [/control words two]

#include "common/driver_file.hpp"
#include "robots/mechanism_table.hpp"

namespace shulib::bench::robot_two {
// [driver file two]
// ═══ WHICH BUTTON DOES WHAT ═══════════════════════════════════════════════════════════════
inline constexpr DriverControls kControls = controls(kRobotTwoMechanisms,
    whileHeld(cascade, Button::R1, Button::R2, 12_volt),   // R1 up / R2 down while held; let go: it holds
    toggle(claw, Button::A),                               // each press: open / close
    toggle(flipper, Button::B),                            // the front toggle -- each press: open / close
    toggle(ramp, Button::X),                               // each press: down / up
    whileHeld(rightToggle, Button::L1, 12_volt),           // held: runs (one way); let go: stops
    whileHeld(leftToggle, Button::L2, 12_volt),            // held: runs (one way); let go: stops
    homeAtDriverStart(cascade, 3_s));                      // [delete-able] home it at the first driver start
// The 12 V and the 3 s are CHOSEN, not measured — robot one's file says why.
// [/driver file two]

// [feel two]
// ═══ THE FEEL ═════════════════════════════════════════════════════════════════════════════
// The library's default, as robot one's: arcade (left stick drives, right stick turns), a smooth
// deadband of 0.05, no curve, no ramp -- numbers no driver has felt yet (HA-112). Field-centric is OFF.
inline constexpr DriverProfile kFeel{.mode = ArcadeBody};
inline constexpr DriverLoopConfig kLoop{
    .rezeroButton = Button::Y,     // field-centric's "away from me"
    .releaseBrakeMode = Coast,     // let go of the sticks: the drive coasts
};
// [/feel two]
}  // namespace shulib::bench::robot_two

namespace shulib::bench {
using RobotTwoDriver = DriverFile<robot_two::kControls, robot_two::kFeel, robot_two::kLoop>;
}  // namespace shulib::bench
