// n51_toggle_on_a_flywheel: toggle(flywheel, X) — name its speed.
// Written by MECH-A4; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
#include "shulib/teleop/teleop_loop.hpp"
inline constexpr mech::Control k = toggle(rx::flywheel, Button::X);
// expect: a_toggle_on_a_flywheel_names_its_speed_like_toggle_flywheel_shoot
// expect-at: 5
