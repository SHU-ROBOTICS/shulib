// w_hand_built_port_22: a hand-built entry on port 22 (mechanisms() range-checks before counting).
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(cr::brokenWiring(wire(rx::arm, motors(1)), [](mech::MechanismWiring& w) { w.motors[0] = 22; }));
// expect: a_smart_port_is_not_a_number_from_1_to_21_or_more_than_four_motors
