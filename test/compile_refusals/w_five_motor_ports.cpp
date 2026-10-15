// w_five_motor_ports: FIVE motor ports (a mechanism has at most four motor roles).
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(wire(rx::arm, motors(1, 2, 3, 4, 5)));
// expect: a_smart_port_is_not_a_number_from_1_to_21_or_more_than_four_motors
