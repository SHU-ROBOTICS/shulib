// k_lift_below_bottom: a lift height BELOW its bottom (0 in).
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr auto g = rx::lift.to(-1_in);
// expect: target_is_outside_the_travel_limits
