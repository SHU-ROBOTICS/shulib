#pragma once
//
// skills_route.hpp — THE TEAM'S ROUTES, shared by both robots (chunk ROB1-BT, 2026-11-16; named routes: RMP1-C,
// 2026-11-17).
//
// Every route is written ONCE in robot one's numbers (src/team/skills_route.cpp, made in the route builder). Both
// autons files register the whole list with one line: robot one each route as written, robot two each route marked
// Mirror::ForRobotTwo MIRRORED across the field's skills diagonal (guide cookbook chapter 9). PROS-free.

#include <cstdint>
#include <span>

#include "shulib/competition/auton_registry.hpp"

namespace shulib::bench {

/// How robot two runs a route.
enum class Mirror : std::uint8_t {
    None,         ///< as written, from robot one's start
    ForRobotTwo,  ///< mirrored across the skills diagonal
};

/// Do both robots run a mirrored route at the SAME time? The route builder asks; the host suite's pair check (the two
/// robots never too close) runs only on Yes.
enum class Together : std::uint8_t {
    NotAnswered,
    No,
    Yes,
};

/// One route of the list (the marked region of skills_route.cpp).
struct TeamRoute {
    competition::AutonEntry (*entry)();  ///< its entry: name, steps, robot one's start, schedule
    Mirror mirror;
    Together together;
};

/// Which robot is registering.
enum class TeamRobot : std::uint8_t { One, Two };

/// Every route, in the brain menu's order.
std::span<const TeamRoute> teamRoutes();

/// Register `routes`, in order: robot one each as written, robot two each marked Mirror::ForRobotTwo mirrored.
void registerRoutes(competition::AutonRegistry& registry, std::span<const TeamRoute> routes, TeamRobot robot);

/// Register every team route (registerRoutes over teamRoutes()) — the one line in each autons file.
void addTeamRoutes(competition::AutonRegistry& registry, TeamRobot robot);

}  // namespace shulib::bench
