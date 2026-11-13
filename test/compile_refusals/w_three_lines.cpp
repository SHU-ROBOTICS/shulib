// w_three_lines: THREE lines (a pneumatic has at most two line roles).
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(wire(rx::hood, lines('A', 'B', 'C')));
// expect: a_three_wire_port_is_not_a_letter_from_A_to_H_or_more_than_two_lines
