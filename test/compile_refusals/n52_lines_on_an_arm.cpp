// n52_lines_on_an_arm: an ARM wired with 3-wire lines.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(wire(rx::arm, lines('A')));
// expect: a_motor_mechanism_is_wired_with_motors_like_motors_6_not_lines
