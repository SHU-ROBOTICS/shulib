// hb_angle_extra: HAND-BUILT (kAngleExtra): an angle sensor port on the arm — mechanisms() holds an entry built by hand to wire()'s rules.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(hb::kAngleExtra);
// expect: the_wiring_plugs_in_a_sensor_the_mechanism_does_not_have
