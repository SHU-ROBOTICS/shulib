// n32_motors_on_a_pneumatic: MOTOR ports on a pneumatic mechanism.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(wire(rx::hood, motors(6, 7)));
// expect: a_pneumatic_is_wired_with_lines_like_lines_A_not_motors
