#pragma once
//
// route_maker_robots.hpp — WHICH robots the route builder describes, and from which tables (chunk RMP1-A1,
// 2026-11-14; the RMP1 brief §1.1 `describe --robot one|two` [R8] `|x|y|z`). Host only.
//
//   one, two  the team's two robots, from src/robots/mechanism_table.hpp (each one's levels marked as its own
//             facts ledger's, src/robots/robot_one_facts.hpp / robot_two_facts.hpp) — what the published page offers;
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
#include "robots/robot_two_facts.hpp"
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
                           .ledgerOwed = !shulib::bench::robot_one_facts::kCascadeLevels.typed,
                           .clawInsetIn = shulib::bench::robot_one_facts::kClawGrabInsetIn.value()};
    }
    if (id == "two") {
        // Robot one's twin (ROB1-BT): its cascade's levels are ITS facts ledger's (robot_two_facts::lv), as robot one's.
        return RobotSource{.id = "two", .name = "robot two", .table = &shulib::bench::robot_two::kRobotTwoMechanisms,
                           .tank = true,
                           .ledgerLevel = [](std::size_t i) -> const char* {
                               return i < shulib::mechanism::kMaxPresets ? shulib::bench::robot_two_facts::lv(i).name : nullptr;
                           },
                           .ledgerOwed = !shulib::bench::robot_two_facts::kCascadeLevels.typed,
                           .clawInsetIn = shulib::bench::robot_two_facts::kClawGrabInsetIn.value()};
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
