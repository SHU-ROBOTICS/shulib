// w_sensor_twice_piece_switch: a limit-switch piece sensor written twice.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(wire(ry::catapult, motors(7), pieceSensor('E'), pieceSensor('F')));
// expect: the_wiring_plugs_in_the_same_sensor_twice
