// harness_target_is_outside_the_travel_limits: SELF-TEST (must FAIL): the reason it states appears ONLY in its own file name (every make and GCC line carries it); it is refused for another.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr auto g = rx::arm["scroe"];
// expect: target_is_outside_the_travel_limits
