// u_sees_piece_on_a_nameless_declaration: seesPiece(m) on a declaration written WITHOUT .name — on no robot's list, so no step could find its sensor (and a null name would read as "this mechanism") — refused at the auton's line, by A1's own name.
// Written by MECH-A2's review round (until.hpp); run_refusal.py judges it (CMakeLists.txt beside it).
#include "refusal_common.hpp"
#include "shulib/mechanism/until.hpp"
inline constexpr mech::Power nameless{.motors = {Forward}, .cartridge = Blue, .pieceSensor = opticalSensor(0.5), .safe = Coast};
inline constexpr mech::Until u = seesPiece(nameless);
// expect: a_mechanism_has_no_name
// expect-at: 6
