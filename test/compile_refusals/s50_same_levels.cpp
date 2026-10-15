// s50_same_levels: two positions with the same level.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Pneumatic bad{.name = "bad", .positions = {{"in", {Low}}, {"out", {Low}}}, .start = "in", .safe = "in", .moveTime = 0.2_s};
inline constexpr MechanismTable m = mechanisms(wire(bad, lines('A')));
// expect: pneumatic_two_positions_have_the_same_line_levels
