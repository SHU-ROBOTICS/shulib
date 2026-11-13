// s33_switch_homing_no_switch: switch homing with its switch role removed (by hand).
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(cr::brokenSpec(wire(rx::lift, motors(2, 3), homeSwitch('B')), [](mech::MechanismSpec& s) { s.homeSwitch.kind = mech::SensorKind::None; }));
// expect: position_homing_by_switch_needs_a_homeswitch_role
