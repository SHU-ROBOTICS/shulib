// w_sensor_on_a_motor_port: a piece sensor on another mechanism's motor port — NAMED.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(wire(rx::flywheel, motors(4)), wire(rx::intake, motors(5), pieceSensor(4)));
// expect: smart_port_4_is_used_twice
