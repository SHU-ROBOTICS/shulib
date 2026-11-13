// s13_home_switch_on_flywheel: a home switch on a flywheel (by hand).
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(cr::brokenSpec(wire(rx::flywheel, motors(4)), [](mech::MechanismSpec& s) { s.homeSwitch.kind = mech::SensorKind::Switch; }));
// expect: a_home_switch_role_on_a_mechanism_that_is_not_position
