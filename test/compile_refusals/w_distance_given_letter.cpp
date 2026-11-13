// w_distance_given_letter: a LETTER for a distance piece sensor.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(wire(rx::intake, motors(5), pieceSensor('C')));
// expect: a_piece_sensor_is_a_smart_sensor_give_it_a_port_number
