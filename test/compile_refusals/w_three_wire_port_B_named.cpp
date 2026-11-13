// w_three_wire_port_B_named: 3-wire port B used twice — NAMED as B.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Pneumatic a{.name = "a", .positions = {{"in", {Low}}, {"out", {High}}}, .start = "in", .safe = "in", .moveTime = 0.2_s};
inline constexpr Pneumatic b{.name = "b", .positions = {{"in", {Low}}, {"out", {High}}}, .start = "in", .safe = "in", .moveTime = 0.2_s};
inline constexpr MechanismTable m = mechanisms(wire(a, lines('B')), wire(b, lines('B')));
// expect: three_wire_port_B_is_used_twice
