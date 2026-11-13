// s05_motor_kind_with_lines: a motor kind with a 3-wire line role (by hand).
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(cr::brokenSpec(wire(rx::arm, motors(1)), [](mech::MechanismSpec& s) { s.lineCount = 1; }));
// expect: a_motor_mechanism_has_three_wire_line_roles_make_two_mechanisms
