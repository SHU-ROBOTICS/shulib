// s47_start_index_past: a start index past the positions (by hand).
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(cr::brokenSpec(wire(rx::hood, lines('C', 'D')), [](mech::MechanismSpec& s) { s.pneumatic.start = 3; }));
// expect: pneumatic_start_or_safe_is_not_one_of_its_positions
