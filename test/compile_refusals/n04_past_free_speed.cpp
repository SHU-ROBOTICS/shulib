// n04_past_free_speed: past the flywheel's free speed (3600).
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr auto g = rx::flywheel.at(4000);
// expect: speed_is_past_the_free_speed
