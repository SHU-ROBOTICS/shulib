// n62_route_id_builtin: a mechanism 'wait' with a position 'ms': the route id wait_ms is the library's own.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Pneumatic bad{.name = "wait", .positions = {{"ms", {Low}}, {"long", {High}}}, .start = "ms", .safe = "ms", .moveTime = 0.3_s};
inline constexpr MechanismTable m = mechanisms(wire(bad, lines('A')));
// expect: a_route_id_is_one_of_the_librarys_own_wait_ms_brake_hold_ms_log
