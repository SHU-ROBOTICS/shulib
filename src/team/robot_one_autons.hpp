#pragma once
//
// robot_one_autons.hpp -- ROBOT ONE'S autons: the one function both auton-running programs call on a
// robot-one build ("shulib Auton Tester" and "shulib Competition", through src/common/this_robot.hpp's
// registerThisRobotsAutons()). The autons themselves are in src/team/robot_one_autons.cpp; the two
// periods every schedule is checked against are src/team/autons.hpp's kAutonPeriods (the game manual's,
// shared by both robots).
//
// PROS-free on purpose: the host suite compiles src/team/robot_one_autons.cpp and runs every auton in it
// on the host test plant (test/robot_one_autons_test.cpp). Chunk ROB1-A, 2026-11-11.

#include "shulib/competition/auton_registry.hpp"

namespace shulib::bench {

/// Register every auton the team has written for ROBOT ONE, in the order the brain's menu lists them.
/// Called ONCE, from a program's initialize(), inside its try, on a registry that drops an entry it
/// refuses (src/team/autons.hpp's banner — the same rules as robot two's registerAutons()).
void registerRobotOneAutons(competition::AutonRegistry& registry);

}  // namespace shulib::bench
