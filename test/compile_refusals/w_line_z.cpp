// w_line_z: a 3-wire letter past H.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(wire(rx::hood, lines('C', 'Z')));
// expect: a_three_wire_port_is_not_a_letter_from_A_to_H_or_more_than_two_lines
