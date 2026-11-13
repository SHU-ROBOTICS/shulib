// n42_one_level_two_lines: one level per position, wired to TWO lines (the second cylinder would be dead) — at its wire(...) line.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Pneumatic c2{.name = "c2", .positions = {{"closed", {Low}, grabs}, {"open", {High}}}, .start = "closed", .safe = "closed", .moveTime = 0.5_s};
inline constexpr MechanismTable m = mechanisms(
    wire(c2, lines('A', 'B')));
// expect: a_position_states_a_different_number_of_levels_than_the_wiring_has_lines
// expect-at: 6
