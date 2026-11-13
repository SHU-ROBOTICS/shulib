// k_gear_driven_no_teeth: a DRIVEN gear with 0 teeth.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr auto g = gears(12, 0);
// expect: a_gear_has_at_least_one_tooth
