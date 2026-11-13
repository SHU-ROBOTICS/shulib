// s11_angle_sensor_on_power: an angle sensor on a power mechanism (by hand).
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(cr::brokenSpec(wire(rx::intake, motors(5), pieceSensor(9)), [](mech::MechanismSpec& s) { s.angleSensor.kind = mech::SensorKind::Rotation; }));
// expect: an_angle_sensor_role_on_a_mechanism_that_is_not_position_or_speed
