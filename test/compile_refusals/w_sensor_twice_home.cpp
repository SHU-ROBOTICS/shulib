// w_sensor_twice_home: a home switch written twice.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(wire(rx::lift, motors(2, 3), homeSwitch('B'), homeSwitch('C')));
// expect: the_wiring_plugs_in_the_same_sensor_twice
