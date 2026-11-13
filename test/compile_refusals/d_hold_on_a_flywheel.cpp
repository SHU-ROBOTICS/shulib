// d_hold_on_a_flywheel: whileHeld on a flywheel — refused by name.
// Written by MECH-A4; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
#include "shulib/teleop/teleop_loop.hpp"
inline constexpr mech::Control k = whileHeld(rx::flywheel, Button::Up, Button::Down, 6_volt);
// expect: whileHeld_is_for_a_power_mechanism_an_arm_or_a_lift_use_toggle_or_preset
// expect-at: 5
