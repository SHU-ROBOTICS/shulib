// n11_lines_numbers: smart-port NUMBERS where 3-wire letters belong.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(wire(rx::hood, lines(6, 7)));
// expect: a_three_wire_port_is_a_letter_in_quotes_like_A_a_number_is_a_smart_port
