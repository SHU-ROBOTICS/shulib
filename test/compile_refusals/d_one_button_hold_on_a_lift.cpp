// d_one_button_hold_on_a_lift: the ONE-button hold (`whileHeld(m, button, volts)`, chunk ROB1-C2) on
// a LIFT (a lift's jog needs a button each way): the one-button form is a POWER mechanism's only.
// The review of ROB1-C2: only the arm had a TU, so a requires-clause cut to Arm alone left this call
// refused by an UNNAMED "no matching function" with the harness green.
// Written by ROB1-C2's review round; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
#include "shulib/teleop/teleop_loop.hpp"
inline constexpr mech::Control k = whileHeld(rx::lift, Button::L1, 12_volt);
// expect: whileHeld_with_one_button_is_for_a_power_mechanism_an_arm_or_a_lift_needs_two
