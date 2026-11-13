// s08_pneumatic_with_motors: a pneumatic with a motor role (by hand).
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(cr::brokenSpec(wire(rx::hood, lines('C', 'D')), [](mech::MechanismSpec& s) { s.motorCount = 1; }));
// expect: a_pneumatic_mechanism_has_motor_roles
