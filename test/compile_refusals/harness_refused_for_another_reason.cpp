// harness_refused_for_another_reason: SELF-TEST (must FAIL): refused, but for a DIFFERENT reason than it states.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr auto g = rx::arm.to(175);
// expect: no_such_preset_on_this_mechanism
