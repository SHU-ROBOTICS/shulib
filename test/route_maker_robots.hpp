#pragma once
//
// route_maker_robots.hpp — WHICH robots the route builder describes, and from which tables (chunk RMP1-A1,
// 2026-11-14; the RMP1 brief §1.1 `describe --robot one|two` [R8] `|x|y|z`). Host only.
//
//   one, two  the team's two robots, from src/robots/mechanism_table.hpp (robot one's levels marked as its
//             facts ledger's, src/robots/robot_one_facts.hpp) — what the published page offers;
//   x, y, z   the fixture robots of test/mechanism_fixtures.hpp, whose numbers the TESTS own (R8): every
//             kind and every homing method, so the page's checks are tested on robots that do not change
//             when the builder types robot one's facts. Their descriptions are COMMITTED
//             (test/goldens/route_maker/robots/) for the page's node tests, and kept equal to this by the
//             suite (route_maker_golden_test.cpp).
// Both 2026 robots are believed tanks ("identical", HA-252); every fixture robot runs on the host test
// plant's 12 in tank.

#include <cstring>
#include <optional>
#include <string_view>

#include "mechanism_fixtures.hpp"
#include "robots/mechanism_table.hpp"
#include "robots/robot_one_facts.hpp"
#include "route_maker_describe.hpp"

namespace route_maker {

/// Robot `id`'s source (one, two, x, y, z), or nothing for another word.
[[nodiscard]] inline std::optional<RobotSource> robotSource(std::string_view id) {
    if (id == "one") {
        return RobotSource{.id = "one", .name = "robot one", .table = &shulib::bench::robot_one::kRobotOneMechanisms,
                           .tank = true,
                           .ledgerLevel = [](std::size_t i) -> const char* {
                               return i < shulib::mechanism::kMaxPresets ? shulib::bench::robot_one_facts::lv(i).name : nullptr;
                           },
                           .ledgerOwed = !shulib::bench::robot_one_facts::kCascadeLevels.typed};
    }
    if (id == "two") {
        // NO ledger on this base: robot two's table is its claws. ROB1-BT gives robot two robot one's build (its
        // cascade, robot_two_facts::lv(i)) — and at THAT merge robot two must get its ledger here, the way robot
        // one has it, or every cascade level the shared route moves to is refused on robot two (RC18 needs the
        // `ledger` mark). The golden test's tripwire ("a robot that builds the shared route and declares a
        // cascade marks its levels as its ledger's") goes red at that merge until it does.
        return RobotSource{.id = "two", .name = "robot two", .table = &shulib::bench::robot_two::kRobotTwoMechanisms,
                           .tank = true, .ledgerLevel = {}};
    }
    if (id == "x") {
        return RobotSource{.id = "x", .name = "robot x (test)", .table = &mech_fixture::robot_x::kRobotXMechanisms,
                           .tank = true, .ledgerLevel = {}};
    }
    if (id == "y") {
        return RobotSource{.id = "y", .name = "robot y (test)", .table = &mech_fixture::robot_y::kRobotYMechanisms,
                           .tank = true, .ledgerLevel = {}};
    }
    if (id == "z") {
        return RobotSource{.id = "z", .name = "robot z (test)", .table = &mech_fixture::robot_z::kRobotZMechanisms,
                           .tank = true, .ledgerLevel = {}};
    }
    return std::nullopt;
}

}  // namespace route_maker
