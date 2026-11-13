// w_three_wire_port_A_named: 3-wire port A used twice — NAMED as A.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Pneumatic a{.name = "a", .positions = {{"in", {Low}}, {"out", {High}}}, .start = "in", .safe = "in", .moveTime = 0.2_s};
inline constexpr Pneumatic b{.name = "b", .positions = {{"in", {Low}}, {"out", {High}}}, .start = "in", .safe = "in", .moveTime = 0.2_s};
inline constexpr MechanismTable m = mechanisms(wire(a, lines('A')), wire(b, lines('A')));
// expect: three_wire_port_A_is_used_twice
