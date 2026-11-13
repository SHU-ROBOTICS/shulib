// k_arm_below_bottom: an arm target BELOW its bottom limit (0).
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr auto g = rx::arm.to(-1);
// expect: target_is_outside_the_travel_limits
