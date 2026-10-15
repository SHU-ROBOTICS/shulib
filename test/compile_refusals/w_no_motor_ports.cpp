// w_no_motor_ports: NO motor port — at its wire(...) line.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(
    wire(rx::arm, motors()));
// expect: the_wiring_gives_a_different_number_of_motors_than_the_mechanism_has
// expect-at: 5
