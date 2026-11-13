// w_extra_piece_sensor: a piece sensor plugged in that the arm does not have — at its wire(...) line.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(
    wire(rx::arm, motors(1), pieceSensor(9)));
// expect: the_wiring_plugs_in_a_sensor_the_mechanism_does_not_have
// expect-at: 5
