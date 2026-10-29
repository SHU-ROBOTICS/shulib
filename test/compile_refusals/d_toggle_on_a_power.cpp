// d_toggle_on_a_power: toggle(intake, X) — a toggle is for a two-position pneumatic or a flywheel's speed.
// Written by MECH-A4; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
#include "shulib/teleop/teleop_loop.hpp"
inline constexpr mech::Control k = toggle(rx::intake, Button::X);
// expect: a_toggle_is_for_a_two_position_pneumatic_or_a_flywheel_speed
// expect-at: 5
