// harness_a_tu_that_compiles: SELF-TEST (must FAIL): a TU that COMPILES prints no reason.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr auto g = rx::arm["score"];
// expect: no_such_preset_on_this_mechanism
