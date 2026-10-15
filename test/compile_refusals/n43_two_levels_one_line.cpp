// n43_two_levels_one_line: two levels per position, wired to ONE line (named as itself) — at its wire(...) line.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Pneumatic h1{.name = "h1", .positions = {{"down", {Low, Low}}, {"middle", {High, Low}}, {"up", {High, High}}}, .start = "down", .safe = "down", .moveTime = 0.3_s};
inline constexpr MechanismTable m = mechanisms(
    wire(h1, lines('C')));
// expect: a_position_states_a_different_number_of_levels_than_the_wiring_has_lines
// expect-at: 6
