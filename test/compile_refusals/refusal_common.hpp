#pragma once
//
// refusal_common.hpp — what every compile-refusal TU starts from (chunk MECH-A1; README in
// CMakeLists.txt beside it). The fixture robots (test/mechanism_fixtures.hpp), the words a robot
// file uses, and — for the reasons NO declaration can produce (a kind byte, a units byte, a line
// count on a motor kind: the declaration types never write them) — entries built BY HAND from a
// valid wire(...) with ONE spec or wiring field broken. mechanisms() re-checks every entry's spec
// and its wiring beside it (mechanism::entryProblem), so a hand-built entry is refused by the same
// named function a declaration would be; these TUs are how every refusal of spec.hpp has a build-time
// test. The hand-built entries of THE MIRROR'S SECOND DIRECTION (hb_*.cpp) are not written here: they
// are test/mechanism_hand_built.hpp's constants (`hb::`), the very ones the host suite feeds to
// wiringProblem() — one input, both halves.

#include "../mechanism_fixtures.hpp"
#include "../mechanism_hand_built.hpp"
#include "shulib/mechanism/kinds.hpp"
#include "shulib/mechanism/route_ids.hpp"
#include "shulib/mechanism/spec.hpp"
#include "shulib/mechanism/wiring.hpp"

using namespace shulib::mechanism::words;
namespace mech = shulib::mechanism;
namespace rx = mech_fixture::robot_x;
namespace ry = mech_fixture::robot_y;
namespace r2 = mech_fixture::robot_two;
namespace hb = mech_fixture::hand_built;

namespace cr {
/// A valid entry `w` with its spec broken by `edit`.
template <class Edit>
consteval mech::Wired brokenSpec(mech::Wired w, Edit edit) {
    edit(w.spec);
    return w;
}
/// A valid entry `w` with its wiring broken by `edit`.
template <class Edit>
consteval mech::Wired brokenWiring(mech::Wired w, Edit edit) {
    edit(w.wiring);
    return w;
}
}  // namespace cr
