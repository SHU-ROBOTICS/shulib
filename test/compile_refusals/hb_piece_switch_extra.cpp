// hb_piece_switch_extra: HAND-BUILT (kPieceSwitchExtra): a piece switch line on the arm — mechanisms() holds an entry built by hand to wire()'s rules.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(hb::kPieceSwitchExtra);
// expect: the_wiring_plugs_in_a_sensor_the_mechanism_does_not_have
