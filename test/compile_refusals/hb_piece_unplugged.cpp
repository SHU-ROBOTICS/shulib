// hb_piece_unplugged: HAND-BUILT (kPieceUnplugged): the intake's piece sensor unplugged — mechanisms() holds an entry built by hand to wire()'s rules.
// Generated with the MECH-A1 harness; run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
inline constexpr MechanismTable m = mechanisms(hb::kPieceUnplugged);
// expect: the_mechanism_has_a_sensor_the_wiring_does_not_plug_in
