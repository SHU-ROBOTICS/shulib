// k_lift_past_top: a lift height past its top (24 in).
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr auto g = rx::lift.to(25_in);
// expect: target_is_outside_the_travel_limits
