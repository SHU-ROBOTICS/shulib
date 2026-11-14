#pragma once
//
// this_robot.hpp — THE BUILD'S TEAM FILES, selected by kRobotId (chunk ROB1-A, 2026-11-11; the ROB1
// brief's §1.1). The programs that run a team's code — "shulib Drive", "shulib Auton Tester", "shulib
// Competition" — used to name robot two's directly (RobotTwoDriver, registerAutons()). With two robots,
// each build must run ITS robot's driver file and autons, chosen by the same one chain that chose its
// tables (src/build_selection.hpp) — NEVER by a switch of the program's own. So a program names only:
//   * ThisRobotDriver            — this build's robot's driver file (DriverOf<kRobotId>);
//   * registerThisRobotsAutons() — this build's robot's autons (autonsFor(kRobotId), a case per robot,
//                                   no default: a robot added to RobotId without one is -Wswitch, a gate
//                                   FAILURE);
//   * ThisRobotWords             — the words that are true of this robot only (src/common/robot_words.hpp).
// The src build gate's step 0 fails a program that names RobotTwoDriver or RobotOneDriver (this file
// exempt) — the per-robot table rule's sibling.
//
// THE BENCH BOT AND THE INVENTED X-DRIVE run no driver file and no auton (PROS compiles every program's
// .cpp in every build, and --gc-sections drops what their entry points never reach): they keep ROBOT
// TWO'S driver type and autons, exactly the bytes their builds carried before ROB1-A (landmine 10).
//
// The autons are chosen through a CONSTANT function pointer (kThisRobotsAutons), so only the chosen
// robot's registration is referenced by a build — robot one's never reaches robot two's packages and
// robot two's never reaches robot one's (the brief's T7, the gate's step 8).

#include <cstddef>
#include <cstdio>

#include "build_selection.hpp"
#include "common/robot_words.hpp"
#include "robots/robot_one_facts.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "team/autons.hpp"
#include "team/robot_one_autons.hpp"
#include "team/robot_one_driver.hpp"
#include "team/robot_two_driver.hpp"

namespace shulib::bench {

/// Robot `R`'s driver file. Robot two's for every robot without one of its own (file banner).
template <RobotId R>
struct DriverOf {
    /// The driver file.
    using type = RobotTwoDriver;
};
/// Robot one's driver file.
template <>
struct DriverOf<RobotId::RobotOne> {
    /// The driver file.
    using type = RobotOneDriver;
};

/// THIS build's robot's driver file — the only driver name a program uses.
using ThisRobotDriver = DriverOf<kRobotId>::type;

/// A robot's autons-registering function.
using RegisterAutons = void (*)(competition::AutonRegistry&);

/// Robot `id`'s autons. A case per robot and no default (file banner).
constexpr RegisterAutons autonsFor(RobotId id) {
    switch (id) {
        case RobotId::RobotTwo:
            return &registerAutons;
        case RobotId::RobotOne:
            return &registerRobotOneAutons;
        case RobotId::BenchBot:
            return &registerAutons;
        case RobotId::InventedXDrive:
            return &registerAutons;
    }
    __builtin_unreachable();
}

/// THIS build's robot's autons, as a constant (file banner).
inline constexpr RegisterAutons kThisRobotsAutons = autonsFor(kRobotId);

/// Register THIS build's robot's autons — the only autons call a program makes. Called ONCE, from a
/// program's initialize(), inside its try (src/team/autons.hpp's banner).
inline void registerThisRobotsAutons(competition::AutonRegistry& registry) { kThisRobotsAutons(registry); }

/// The facts line for `facts` facts of which `owed` are still owed, into `out` (at most `size` bytes,
/// NUL-terminated); returns snprintf's count. Its counts are parameters so the host tests can give it a
/// ledger with some facts typed (robot one's has none yet).
inline int robotOneFactsLineFor(char* out, std::size_t size, int facts, int owed) {
    return std::snprintf(out, size,
                         "robot one: %d facts, %d typed (src/robots/robot_one_facts.hpp); %d still owed, none "
                         "needed by this program",
                         facts, facts - owed, owed);
}

/// Robot one's facts line for "shulib Drive"'s boot log: the ledger's counts — provenance, not a
/// refusal (a program that needs an owed fact never links, so "none needed by this program" is true of
/// every build that prints it). Robot one's builds call it; the programs never name robot one's ledger.
inline int robotOneFactsLine(char* out, std::size_t size) {
    return robotOneFactsLineFor(out, size, robot_one_facts::kRobotOneFactCount,
                                robot_one_facts::kRobotOneOwedCount);
}

}  // namespace shulib::bench
