// w_home_switch_z: a home switch on 'Z' (named as a bad 3-wire port, D7).
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(wire(rx::lift, motors(2, 3), homeSwitch('Z')));
// expect: a_three_wire_port_is_not_a_letter_from_A_to_H_or_more_than_two_lines
