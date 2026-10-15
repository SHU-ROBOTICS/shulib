// hb_motor_unbound: HAND-BUILT (kMotorUnbound): a motor role on port 0 — mechanisms() holds an entry built by hand to wire()'s rules.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(hb::kMotorUnbound);
// expect: the_wiring_gives_a_different_number_of_motors_than_the_mechanism_has
