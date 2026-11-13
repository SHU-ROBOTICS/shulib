// k_gear_with_no_teeth: a gear with 0 teeth.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr auto g = gears(0, 60);
// expect: a_gear_has_at_least_one_tooth
