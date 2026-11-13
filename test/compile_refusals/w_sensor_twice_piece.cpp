// w_sensor_twice_piece: a distance piece sensor written twice.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(wire(rx::intake, motors(5), pieceSensor(9), pieceSensor(10)));
// expect: the_wiring_plugs_in_the_same_sensor_twice
