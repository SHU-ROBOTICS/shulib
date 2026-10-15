// m4_pneumatic_spec_at_its_wire_line: MULTI-LINE: a pneumatic's own mistake (two positions at the same level), refused at its wire(...) line — wire(Pneumatic)'s own spec check, which mechanisms()' re-check of every entry would otherwise hide (the resumed review round's dispatch-site sweep).
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Pneumatic bad{.name = "bad", .positions = {{"in", {Low}}, {"out", {Low}}}, .start = "in", .safe = "in", .moveTime = 0.2_s};
inline constexpr MechanismTable m = mechanisms(
    wire(rx::arm, motors(1)),
    wire(bad, lines('A')));
// expect: pneumatic_two_positions_have_the_same_line_levels
// expect-at: 7
