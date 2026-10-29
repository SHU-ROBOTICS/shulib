// d_home_on_press_on_a_used_button: homeOnPress(lift, Left) beside a preset on Left — a HOMING button doing two jobs, NAMED as Left.
// Written by MECH-A4's review round; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
#include "shulib/teleop/teleop_loop.hpp"
inline constexpr DriverControls c = controls(rx::kRobotXMechanisms, whileHeld(rx::arm, Button::R1, Button::R2, 6_volt), preset(rx::arm["score"], Button::Left, 1.5_s), whileHeld(rx::lift, Button::L1, Button::L2, 12_volt), homeOnPress(rx::lift, Button::Left, 1.5_s), toggle(rx::flywheel["shoot"], Button::X), whileHeld(rx::intake, Button::A, Button::B, 12_volt), autonOnly(rx::hood));
// expect: button_Left_does_two_jobs
// expect-at: 5
