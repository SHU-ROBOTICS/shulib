// s46_one_position: a pneumatic with one position.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Pneumatic bad{.name = "bad", .positions = {{"a", {Low}}}, .start = "a", .safe = "a", .moveTime = 0.2_s};
inline constexpr MechanismTable m = mechanisms(wire(bad, lines('A')));
// expect: pneumatic_two_or_three_positions
