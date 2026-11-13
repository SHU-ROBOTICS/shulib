// n03_target_is_outside_the_travel_limits: a constant target past the travel limit (160).
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr auto g = rx::arm.to(175);
// expect: target_is_outside_the_travel_limits
