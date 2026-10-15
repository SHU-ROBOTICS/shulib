// s12_rotation_as_home_switch: a rotation sensor in the home-switch slot (by hand).
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(cr::brokenSpec(wire(rx::lift, motors(2, 3), homeSwitch('B')), [](mech::MechanismSpec& s) { s.homeSwitch.kind = mech::SensorKind::Rotation; }));
// expect: the_home_switch_role_is_not_a_limit_switch
