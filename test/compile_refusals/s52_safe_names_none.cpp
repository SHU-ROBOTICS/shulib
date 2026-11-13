// s52_safe_names_none: a safe that names no position.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Pneumatic bad{.name = "bad", .positions = {{"in", {Low}}, {"out", {High}}}, .start = "in", .safe = "x", .moveTime = 0.2_s};
inline constexpr MechanismTable m = mechanisms(wire(bad, lines('A')));
// expect: pneumatic_its_safe_names_none_of_its_positions
