// n28_a_three_wire_port_is_used_twice: a 3-wire port used twice (the hood and the lift's switch on C) — NAMED.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(wire(rx::arm, motors(1)), wire(rx::hood, lines('C', 'D')), wire(rx::lift, motors(2, 3), homeSwitch('C')));
// expect: three_wire_port_C_is_used_twice
