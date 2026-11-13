// n26_home_switch_number: a NUMBER for the home switch (66 is the code of B).
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(wire(rx::lift, motors(2, 3), homeSwitch(66)));
// expect: a_three_wire_port_is_a_letter_in_quotes_like_A_a_number_is_a_smart_port
