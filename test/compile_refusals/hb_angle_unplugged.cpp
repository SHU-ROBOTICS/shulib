// hb_angle_unplugged: HAND-BUILT (kAngleUnplugged): the wrist's angle sensor unplugged — mechanisms() holds an entry built by hand to wire()'s rules.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(hb::kAngleUnplugged);
// expect: the_mechanism_has_a_sensor_the_wiring_does_not_plug_in
