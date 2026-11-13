// u_stops_seeing_piece_on_a_flywheel: stopsSeeingPiece(flywheel) — a kind with no piece sensor at all (an arm, a lift, a flywheel) — refused by name.
// Written by MECH-A2 (until.hpp); run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
#include "shulib/mechanism/until.hpp"
inline constexpr mech::Until u = stopsSeeingPiece(rx::flywheel);
// expect: that_mechanism_has_no_piece_sensor
// expect-at: 5
