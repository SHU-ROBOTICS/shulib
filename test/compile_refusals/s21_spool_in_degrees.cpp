// s21_spool_in_degrees: a spool lift in degrees (by hand).
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(cr::brokenSpec(wire(rx::lift, motors(2, 3), homeSwitch('B')), [](mech::MechanismSpec& s) { s.position.units = mech::Units::Degrees; }));
// expect: position_a_lift_is_in_inches
