// n27_motor_port_letter: a LETTER for a motor port.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(wire(rx::arm, motors('A')));
// expect: a_motor_port_is_a_number_like_6_a_letter_in_quotes_is_a_three_wire_port
