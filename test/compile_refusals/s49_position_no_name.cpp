// s49_position_no_name: a position named "".
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Pneumatic bad{.name = "bad", .positions = {{"", {Low}}, {"out", {High}}}, .start = "out", .safe = "out", .moveTime = 0.2_s};
inline constexpr MechanismTable m = mechanisms(wire(bad, lines('A')));
// expect: pneumatic_a_position_has_no_name
