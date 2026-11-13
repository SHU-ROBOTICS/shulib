// s48_no_move_time: a pneumatic with no .moveTime (P6).
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Pneumatic bad{.name = "bad", .positions = {{"in", {Low}}, {"out", {High}}}, .start = "in", .safe = "in"};
inline constexpr MechanismTable m = mechanisms(wire(bad, lines('A')));
// expect: pneumatic_actuationtime_must_be_more_than_zero
