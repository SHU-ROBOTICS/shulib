// w_sensor_twice_hides_a_port: a piece sensor written twice, its first port another mechanism's motor (port 4 would never have been counted).
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(wire(rx::flywheel, motors(4)), wire(rx::intake, motors(5), pieceSensor(4), pieceSensor(9)));
// expect: the_wiring_plugs_in_the_same_sensor_twice
