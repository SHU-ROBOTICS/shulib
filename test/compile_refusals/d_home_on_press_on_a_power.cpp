// d_home_on_press_on_a_power: homeOnPress(intake, ...) — only an arm or a lift homes.
// Written by MECH-A4; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
#include "shulib/teleop/teleop_loop.hpp"
inline constexpr mech::Control k = homeOnPress(rx::intake, Button::X, 1_s);
// expect: homeOnPress_and_homeAtDriverStart_are_for_an_arm_or_a_lift
// expect-at: 5
