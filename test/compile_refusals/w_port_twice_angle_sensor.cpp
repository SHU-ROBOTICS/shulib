// w_port_twice_angle_sensor: a wrist's ANGLE SENSOR on the arm's motor port.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(wire(rx::arm, motors(1)), wire(ry::wrist, motors(6), angleSensor(1)));
// expect: smart_port_1_is_used_twice
