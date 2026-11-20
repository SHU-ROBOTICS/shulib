// autons.cpp -- THE TEAM'S AUTONS for ROBOT TWO, and registerAutons() at the bottom, the one list the
// programs read on a robot-two build ("shulib Auton Tester" and "shulib Competition" -- both hand the
// SAME entry to the SAME library call, so the auton you test on the bench is the auton that runs).
//
// Robot two is robot one's TWIN (chunk ROB1-BT, 2026-11-16): the same build, and the same autons file as
// robot one's (src/team/robot_one_autons.cpp, whose comments explain each line) -- with ONE difference,
// the team lead's ruling: robot two runs each team route marked mirrored MIRRORED across the field's skills
// diagonal (cookbook chapter 9, "One route, two robots"). Robot two's old claw autons file is retired
// with its claws. Every line here is the team's to explain, defend and change (G4).
//
// THE FRAME, THE DISTANCES, THE MECHANISMS: as robot one's file says. `using namespace robot_two;` makes
// robot two's mechanisms nameable by their declarations (src/robots/mechanism_table.hpp).
//
// NOTHING HERE HAS RUN ON A ROBOT. Robot two's numbers are its facts file's
// (src/robots/robot_two_facts.hpp): the Auton Tester and Competition refuse to build while one they need
// is OWED.

#include "team/autons.hpp"

#include "robots/mechanism_table.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/competition/acceptance_run.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/units/literals.hpp"
#include "team/skills_route.hpp"

namespace shulib::bench {

using namespace units::literals;  // 40_in, 0_deg, 57_s
using namespace robot_two;        // robot two's mechanisms, by their declarations (this file is robot two's)

namespace {

// ── THE 40 IN AUTON (robot one's, copied) ──────────────────────────────────────────────────
// Drive straight forward 40 inches and stop, facing the way the robot started.
chassis::RoutineResult fortyInchesForward(competition::AutonContext& ctx) {
    chassis::Routine r{ctx, "40in-forward"};
    r.moveTo(math::Pose2d{40_in, 0_in, 0_deg});
    return r.result();
}

}  // namespace

void registerAutons(competition::AutonRegistry& registry) {
    // 1. THE 40 IN AUTON, as a SKILLS auton (the 60 s autonomous coding skills run).
    competition::AutonEntry forty{};
    forty.name = "40in forward";                      // what the brain's menu shows
    forty.kind = competition::AutonKind::Skills;      // runs in the 60 s skills period
    forty.routine = &fortyInchesForward;              // the function above
    forty.start = competition::StartPose::relative(); // where we place it is (0, 0), facing 0
    forty.endActionAt = 57_s;  // still running at 57 s? it is stopped, and the end action starts
    forty.hardStopAt = 59_s;   // everything stops: 2 s after 57 s, 1 s before the run's 60 s
    // endAction is left out: the end action is chassis.brake() -- 0 V under Brake until still.
    registry.add(forty);

    // 2. THE LIBRARY'S ACCEPTANCE RUN (drive 24 in, turn to 90 degrees, face the start, drive back): the
    //    test that the whole library works on this robot, on the skills schedule.
    registry.add(competition::acceptanceRunEntry(competition::AutonKind::Skills, 57_s, 59_s));

    // 3. THE TEAM'S ROUTES (src/team/skills_route.cpp, named in the route builder): each route marked mirrored for
    //    robot two MIRRORED across the skills diagonal, the others as written.
    addTeamRoutes(registry, TeamRobot::Two);
}

}  // namespace shulib::bench
