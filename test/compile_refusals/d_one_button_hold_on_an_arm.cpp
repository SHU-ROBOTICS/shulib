// d_one_button_hold_on_an_arm: the ONE-button hold (`whileHeld(m, button, volts)`, chunk ROB1-C2) on an ARM —
// an arm's jog needs a button each way; the one-button form is a POWER mechanism's only.
// Written by ROB1-C2; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
#include "shulib/teleop/teleop_loop.hpp"
inline constexpr DriverControls c = controls(rx::kRobotXMechanisms, whileHeld(rx::arm, Button::R1, 6_volt), whileHeld(rx::lift, Button::L1, Button::L2, 12_volt), homeAtDriverStart(rx::lift, 1.5_s), toggle(rx::flywheel["shoot"], Button::X), whileHeld(rx::intake, Button::A, Button::B, 12_volt), autonOnly(rx::hood));
// expect: whileHeld_with_one_button_is_for_a_power_mechanism_an_arm_or_a_lift_needs_two
