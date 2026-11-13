// w_no_lines: NO line — at its wire(...) line.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(
    wire(rx::hood, lines()));
// expect: a_pneumatic_mechanism_needs_one_or_two_line_roles
// expect-at: 5
