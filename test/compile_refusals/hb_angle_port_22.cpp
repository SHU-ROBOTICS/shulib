// hb_angle_port_22: HAND-BUILT (kAnglePort22): an angle sensor on port 22 — mechanisms() holds an entry built by hand to wire()'s rules.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(hb::kAnglePort22);
// expect: a_smart_port_is_not_a_number_from_1_to_21_or_more_than_four_motors
