// n10_a_smart_port_is_used_twice: a smart port used twice — NAMED.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(wire(rx::arm, motors(1)), wire(rx::intake, motors(1), pieceSensor(9)));
// expect: smart_port_1_is_used_twice
