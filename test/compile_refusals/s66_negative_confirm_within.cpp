// s66_negative_confirm_within: a confirmWithin of -0.5 s (the move time, 0.3 s, is correct).
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr Pneumatic bad{.name = "bad", .positions = {{"in", {Low}}, {"out", {High}}}, .start = "in", .safe = "in", .moveTime = 0.3_s, .confirmWithin = -0.5_s};
inline constexpr MechanismTable m = mechanisms(wire(bad, lines('A')));
// expect: pneumatic_confirmwithin_must_not_be_negative
