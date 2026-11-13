// w_three_wire_port_G_named: 3-wire port G used twice — NAMED as G.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Pneumatic a{.name = "a", .positions = {{"in", {Low}}, {"out", {High}}}, .start = "in", .safe = "in", .moveTime = 0.2_s};
inline constexpr Pneumatic b{.name = "b", .positions = {{"in", {Low}}, {"out", {High}}}, .start = "in", .safe = "in", .moveTime = 0.2_s};
inline constexpr MechanismTable m = mechanisms(wire(a, lines('G')), wire(b, lines('G')));
// expect: three_wire_port_G_is_used_twice
