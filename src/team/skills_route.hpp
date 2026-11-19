#pragma once
//
// skills_route.hpp — THE TEAM'S SKILLS ROUTE, shared by both robots (chunk ROB1-BT, 2026-11-16).
//
// One route, written ONCE in robot one's numbers (src/team/skills_route.cpp). Robot one registers it as
// written; robot two — robot one's twin — registers it MIRRORED across the field's skills diagonal
// (src/team/autons.cpp, one line; guide cookbook chapter 9). PROS-free: the host suite runs it.

#include "shulib/competition/auton_registry.hpp"

namespace shulib::bench {

/// The skills route's entry — its name, its kind (Skills), its steps, robot one's start and its schedule.
competition::AutonEntry skillsRoute();

}  // namespace shulib::bench
