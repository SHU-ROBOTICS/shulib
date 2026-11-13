// s09_pneumatic_no_lines: a pneumatic with no line role (by hand).
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(cr::brokenSpec(wire(rx::hood, lines('C', 'D')), [](mech::MechanismSpec& s) { s.lineCount = 0; }));
// expect: a_pneumatic_mechanism_needs_one_or_two_line_roles
