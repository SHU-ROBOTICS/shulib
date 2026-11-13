// hb_levels_vs_lines: HAND-BUILT (kLevelsVsLines): a position's levels vs the lines (U2) — mechanisms() holds an entry built by hand to wire()'s rules.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(hb::kLevelsVsLines);
// expect: a_position_states_a_different_number_of_levels_than_the_wiring_has_lines
