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
// TWO'S driver type and autons (landmine 10) — robot one's twin's since ROB1-BT; they never run them.
//
// The autons are chosen through a CONSTANT function pointer (kThisRobotsAutons), so only the chosen
// robot's registration is referenced by a build — robot one's never reaches robot two's packages and
// robot two's never reaches robot one's (the brief's T7, the gate's step 8).

#include <cstddef>
#include <cstdio>

#include "build_selection.hpp"
#include "common/robot_words.hpp"
#include "robots/robot_one_facts.hpp"
#include "robots/robot_two_facts.hpp"
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

/// The facts line for `robot`'s ledger at `ledgerPath` ("robot one", "src/robots/robot_one_facts.hpp"),
/// `facts` facts of which `owed` are still owed, into `out` (at most `size` bytes, NUL-terminated); returns
/// snprintf's count. The counts are parameters so the host tests can try any.
inline int robotFactsLineFor(char* out, std::size_t size, const char* robot, const char* ledgerPath, int facts,
                             int owed) {
    return std::snprintf(out, size, "%s: %d facts, %d typed (%s); %d still owed, none needed by this program",
                         robot, facts, facts - owed, ledgerPath, owed);
}

/// The facts line for ROBOT ONE's ledger (ROB1-A's form; robotFactsLineFor for robot one).
inline int robotOneFactsLineFor(char* out, std::size_t size, int facts, int owed) {
    return robotFactsLineFor(out, size, "robot one", "src/robots/robot_one_facts.hpp", facts, owed);
}

/// Robot one's facts line (robotFactsLineFor over its ledger's counts).
inline int robotOneFactsLine(char* out, std::size_t size) {
    return robotOneFactsLineFor(out, size, robot_one_facts::kRobotOneFactCount, robot_one_facts::kRobotOneOwedCount);
}

/// Robot two's facts line (ROB1-BT: robot two has a facts file too).
inline int robotTwoFactsLine(char* out, std::size_t size) {
    return robotFactsLineFor(out, size, "robot two", "src/robots/robot_two_facts.hpp",
                             robot_two_facts::kRobotTwoFactCount, robot_two_facts::kRobotTwoOwedCount);
}

/// True when THIS build's robot has a facts ledger: robot one, and robot two since ROB1-BT (its twin).
inline constexpr bool kThisRobotHasLedger = kRobotId == RobotId::RobotOne || kRobotId == RobotId::RobotTwo;

/// THIS build's robot's facts line for "shulib Drive"'s boot log: its ledger's counts — provenance, not a
/// refusal (a program that needs an owed fact never links, so "none needed by this program" is true of
/// every build that prints it). Called only where kThisRobotHasLedger; a robot's package carries only ITS
/// ledger's path (chosen at compile time).
inline int thisRobotFactsLine(char* out, std::size_t size) {
    if constexpr (kRobotId == RobotId::RobotTwo) {
        return robotTwoFactsLine(out, size);
    } else {
        return robotOneFactsLine(out, size);
    }
}

}  // namespace shulib::bench
