// w_piece_port_0: piece sensor port 0.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(wire(rx::intake, motors(5), pieceSensor(0)));
// expect: a_smart_port_is_not_a_number_from_1_to_21_or_more_than_four_motors
