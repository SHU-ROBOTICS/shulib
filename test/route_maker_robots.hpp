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

#include <array>
#include <cstring>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "mechanism_fixtures.hpp"
#include "robots/mechanism_table.hpp"
#include "robots/robot_one_facts.hpp"
#include "robots/robot_two_facts.hpp"
#include "route_maker_describe.hpp"

namespace route_maker {

/// RMP1-M: a team robot's wiring facts' `typed` flags, from its facts ledger (robot_one_facts / robot_two_facts).
struct WiringTyped {
    bool cascadePorts, cascadeDirections, rightTogglePort, sideTogglePorts, rightToggleDirection, leftToggleDirection;
    bool clawLine, flipperLine, rampLine, clawLevel, flipperLevel, rampLevel;
};
#define ROUTE_MAKER_WIRING_TYPED(F)                                                                                   \
    WiringTyped{F::kCascadePorts.typed,         F::kCascadeDirections.typed,    F::kRightTogglePort.typed,          \
                F::kSideTogglePorts.typed,      F::kRightToggleDirection.typed, F::kLeftToggleDirection.typed,      \
                F::kClawLine.typed,             F::kFlipperLine.typed,          F::kRampLine.typed,                 \
                F::kClawOpenLevel.typed,        F::kFlipperOpenLevel.typed,     F::kRampDownLevel.typed}

/// Which wiring facts of team mechanism `m` are owed (the left toggle's port is the right's plus the pair); nothing for a
/// name this list does not know.
[[nodiscard]] constexpr std::optional<WiringOwed> wiringOwedOf(std::string_view m, const WiringTyped& t) {
    if (m == "cascade") {
        return WiringOwed{!t.cascadePorts, !t.cascadeDirections};
    }
    if (m == "right toggle") {
        return WiringOwed{!t.rightTogglePort, !t.rightToggleDirection};
    }
    if (m == "left toggle") {
        return WiringOwed{!t.rightTogglePort || !t.sideTogglePorts, !t.leftToggleDirection};
    }
    if (m == "claw") {
        return WiringOwed{!t.clawLine, !t.clawLevel};
    }
    if (m == "flipper") {
        return WiringOwed{!t.flipperLine, !t.flipperLevel};
    }
    if (m == "ramp") {
        return WiringOwed{!t.rampLine, !t.rampLevel};
    }
    return std::nullopt;
}

/// Every mechanism of `table` is one wiringOwedOf() knows.
[[nodiscard]] constexpr bool wiringKnown(const shulib::mechanism::MechanismTable& table) {
    for (std::size_t i = 0; i < table.count; ++i) {
        if (!wiringOwedOf(shulib::mechanism::nameOf(table.specs[i]), WiringTyped{}).has_value()) {
            return false;
        }
    }
    return true;
}
static_assert(wiringKnown(shulib::bench::robot_one::kRobotOneMechanisms) && wiringKnown(shulib::bench::robot_two::kRobotTwoMechanisms),
              "route_maker_robots.hpp: a team mechanism wiringOwedOf() does not know -- add its facts there, or the page shows them unchecked");

/// The facts robot `facts`' Auton Tester needs that are still OWED, by name (its build refuses while any is).
template <std::size_t N>
[[nodiscard]] std::vector<std::string> testerOwed(const shulib::bench::robot_one_facts::FactSummary (&facts)[N]) {
    std::vector<std::string> out;
    for (const auto& f : facts) {
        if (!f.typed && shulib::bench::robot_one_facts::needs(shulib::bench::Program::AutonTester, f.needed)) {
            out.emplace_back(f.name);
        }
    }
    return out;
}

/// Robot `id`'s source (one, two, x, y, z), or nothing for another word.
[[nodiscard]] inline std::optional<RobotSource> robotSource(std::string_view id) {
    if (id == "one") {
        return RobotSource{.id = "one", .name = "robot one", .table = &shulib::bench::robot_one::kRobotOneMechanisms,
                           .tank = true,
                           .ledgerLevel = [](std::size_t i) -> const char* {
                               return i < shulib::mechanism::kMaxPresets ? shulib::bench::robot_one_facts::lv(i).name : nullptr;
                           },
                           .ledgerOwed = !shulib::bench::robot_one_facts::kCascadeLevels.typed,
                           .wiringOwed = [](std::string_view m) {
                               return wiringOwedOf(m, ROUTE_MAKER_WIRING_TYPED(shulib::bench::robot_one_facts)).value_or(WiringOwed{true, true});
                           },
                           .testerOwed = testerOwed(shulib::bench::robot_one_facts::kRobotOneFacts),
                           .clawInsetIn = shulib::bench::robot_one_facts::kClawGrabInsetIn.value(),
                           .frontBayIn = std::array{shulib::bench::robot_one_facts::kFrontBayLipRadiusIn.value(),
                                                    shulib::bench::robot_one_facts::kFrontBayBodyRadiusIn.value()}};
    }
    if (id == "two") {
        // Robot one's twin (ROB1-BT): its cascade's levels are ITS facts ledger's (robot_two_facts::lv), as robot one's.
        return RobotSource{.id = "two", .name = "robot two", .table = &shulib::bench::robot_two::kRobotTwoMechanisms,
                           .tank = true,
                           .ledgerLevel = [](std::size_t i) -> const char* {
                               return i < shulib::mechanism::kMaxPresets ? shulib::bench::robot_two_facts::lv(i).name : nullptr;
                           },
                           .ledgerOwed = !shulib::bench::robot_two_facts::kCascadeLevels.typed,
                           .wiringOwed = [](std::string_view m) {
                               return wiringOwedOf(m, ROUTE_MAKER_WIRING_TYPED(shulib::bench::robot_two_facts)).value_or(WiringOwed{true, true});
                           },
                           .testerOwed = testerOwed(shulib::bench::robot_two_facts::kRobotTwoFacts),
                           .clawInsetIn = shulib::bench::robot_two_facts::kClawGrabInsetIn.value(),
                           .frontBayIn = std::array{shulib::bench::robot_two_facts::kFrontBayLipRadiusIn.value(),
                                                    shulib::bench::robot_two_facts::kFrontBayBodyRadiusIn.value()}};
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
