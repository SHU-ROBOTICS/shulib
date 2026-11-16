// d_one_button_hold_button_twice: a ONE-button hold (chunk ROB1-C2) on A beside another control on A — its one
// button IS taken (only the back button it does not have is left free).
// Written by ROB1-C2; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
#include "shulib/teleop/teleop_loop.hpp"
inline constexpr DriverControls c = controls(rx::kRobotXMechanisms, whileHeld(rx::arm, Button::R1, Button::R2, 6_volt), whileHeld(rx::lift, Button::L1, Button::L2, 12_volt), homeAtDriverStart(rx::lift, 1.5_s), toggle(rx::flywheel["shoot"], Button::X), whileHeld(rx::intake, Button::A, 12_volt), preset(rx::hood["down"], Button::A));
// expect: button_A_does_two_jobs
// expect-at: 6
