// robot_one_autons.cpp -- THE TEAM'S AUTONS for robot one, and registerRobotOneAutons() at the bottom,
// the one list the programs read on a robot-one build.
//
// A STARTER FILE (chunk ROB1-A, 2026-11-11; the ROB1 brief's D8): robot two's "40in forward", copied and
// registered as a SKILLS auton (the only competition on Nov 21 is skills — the team lead's ruling), and
// the library's acceptance run. NOTHING ELSE: THE SKILLS ROUTE IS THE STUDENTS' (the guardrail G4) — it
// goes where the comment below says. Every line here is the team's to explain, defend and change.
//
// Everything src/team/autons.cpp's banner says holds here: the auton you test on the bench ("shulib
// Auton Tester") is the auton that runs in the competition program; the frame is x ahead, y to the left,
// angles counterclockwise; the distances are the robot's own estimate (a disagreement with the tape is
// the calibration record's, never this file's number).
//
// THE MECHANISMS. `using namespace robot_one;` makes robot one's mechanisms nameable here by their
// declarations in src/robots/mechanism_table.hpp (robot one's region) --
//     r.startHome(cascade, 2_s);                  // home the cascade while the first leg drives...
//     r.waitUntilDone(cascade, 2_s);              // ...and wait for the home to FINISH: a move sent while
//                                                 // it is still homing replaces it, and stops NOT_HOMED
//     r.move(cascade.to(18_in), 2_s);             // the cascade to a height, bounded (it has no levels yet --
//                                                 // the builder: manual for now; once levels are typed,
//                                                 // r.move(cascade[facts::lv(0).name], 2_s))
//     r.set(claw["open"]);  r.set(ramp["down"]);  // the pistons, by their positions' names
// -- no auton here steps a mechanism yet: which one when is the team's route. (Until station 13 says which
// line level opens each piston, the Auton Tester and Competition refuse to build, by name.)
//
// NOTHING HERE HAS RUN ON A ROBOT; robot one's facts are typed from its builder's answers (2026-11-15), not
// measured. On a robot-one build the cascade's numbers are its facts ledger's (src/robots/robot_one_facts.hpp)
// — the Auton Tester and Competition REFUSE to build while any it needs is OWED.

#include "team/robot_one_autons.hpp"

#include "robots/mechanism_table.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/competition/acceptance_run.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/units/literals.hpp"

namespace shulib::bench {

using namespace units::literals;  // 40_in, 0_deg, 57_s
using namespace robot_one;        // robot one's mechanisms, by their declarations (this file is robot one's)

namespace {

// ── THE 40 IN AUTON (robot two's, copied) ───────────────────────────────────────────────────
// Drive straight forward 40 inches and stop, facing the way the robot started: ONE moveTo to the pose
// 40 in ahead, 0 in to the side, facing 0 degrees. src/team/autons.cpp's comment on the same auton says
// what the library does when a tank drifts, and how to read a MOTION_TIMEOUT -- all of it holds here.
chassis::RoutineResult fortyInchesForward(competition::AutonContext& ctx) {
    chassis::Routine r{ctx, "40in-forward"};
    r.moveTo(math::Pose2d{40_in, 0_in, 0_deg});
    return r.result();
}

// ── THE SKILLS ROUTE GOES HERE (the students') ──────────────────────────────────────────────
// Write it as a function like the one above, then register it below as a SKILLS auton.

}  // namespace

void registerRobotOneAutons(competition::AutonRegistry& registry) {
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
}

}  // namespace shulib::bench
