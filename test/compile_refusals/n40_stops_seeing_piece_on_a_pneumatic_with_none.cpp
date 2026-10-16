// n40_stops_seeing_piece_on_a_pneumatic_with_none: stopsSeeingPiece(hood) — a kind that MAY declare a piece sensor, but this one declares none — refused by name.
// Written by MECH-A2 (until.hpp); run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
#include "shulib/mechanism/until.hpp"
inline constexpr mech::Until u = stopsSeeingPiece(rx::hood);
// expect: that_mechanism_has_no_piece_sensor
// expect-at: 5
