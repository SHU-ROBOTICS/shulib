// u_sees_piece_on_a_power_with_none: seesPiece(clawPivot6) — a POWER mechanism with no piece sensor (robot two's pivots) — refused by name.
// Written by MECH-A2 (until.hpp); run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
#include "shulib/mechanism/until.hpp"
inline constexpr mech::Until u = seesPiece(r2::clawPivot6);
// expect: that_mechanism_has_no_piece_sensor
// expect-at: 5
