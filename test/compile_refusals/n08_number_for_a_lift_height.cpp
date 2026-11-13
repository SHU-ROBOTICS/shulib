// n08_number_for_a_lift_height: a plain number where a lift needs inches (the error is ON this line).
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr auto g = rx::lift.to(18);
// expect: a_lift_height_needs_inches_like_to_18_in
// expect-at: 4
