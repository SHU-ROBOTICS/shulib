// w_sensor_twice_angle: an angle sensor written twice.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(wire(ry::wrist, motors(1), angleSensor(10), angleSensor(11)));
// expect: the_wiring_plugs_in_the_same_sensor_twice
