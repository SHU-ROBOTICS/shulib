// k_past_free_speed_reversed: past the flywheel's free speed in REVERSE (-3600).
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr auto g = rx::flywheel.at(-4000);
// expect: speed_is_past_the_free_speed
