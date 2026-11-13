// w_angle_sensor_letter: a LETTER for an angle sensor.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(wire(ry::wrist, motors(1), angleSensor('B')));
// expect: an_angle_sensor_is_a_smart_sensor_give_it_a_port_number
