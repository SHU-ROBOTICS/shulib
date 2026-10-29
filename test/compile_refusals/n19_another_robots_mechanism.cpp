// n19_another_robots_mechanism: a control naming ANOTHER robot's mechanism (robot two's claw on robot x).
// Written by MECH-A4; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
#include "shulib/teleop/teleop_loop.hpp"
inline constexpr DriverControls c = controls(rx::kRobotXMechanisms, whileHeld(rx::arm, Button::R1, Button::R2, 6_volt), whileHeld(rx::lift, Button::L1, Button::L2, 12_volt), homeAtDriverStart(rx::lift, 1.5_s), toggle(rx::flywheel["shoot"], Button::X), whileHeld(rx::intake, Button::A, Button::B, 12_volt), autonOnly(rx::hood), toggle(r2::claw, Button::Y));
// expect: a_control_names_a_mechanism_this_robot_does_not_have
// expect-at: 5
