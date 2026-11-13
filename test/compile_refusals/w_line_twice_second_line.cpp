// w_line_twice_second_line: the hood's SECOND line on the lift's home switch line.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(wire(rx::lift, motors(2, 3), homeSwitch('D')), wire(rx::hood, lines('C', 'D')));
// expect: three_wire_port_D_is_used_twice
