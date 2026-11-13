// n39_sees_piece_on_an_arm: a done-condition on ANOTHER mechanism that has no piece sensor at all (an arm) — refused at the auton's line, by name.
// Written by MECH-A2 (until.hpp); run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
#include "shulib/mechanism/until.hpp"
inline constexpr mech::Until u = seesPiece(rx::arm);
// expect: that_mechanism_has_no_piece_sensor
// expect-at: 5
