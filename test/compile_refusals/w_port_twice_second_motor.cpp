// w_port_twice_second_motor: a lift's SECOND motor on the arm's port.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(wire(rx::arm, motors(1)), wire(rx::lift, motors(2, 1), homeSwitch('B')));
// expect: smart_port_1_is_used_twice
