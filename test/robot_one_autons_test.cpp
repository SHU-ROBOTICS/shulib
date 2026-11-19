// ROBOT ONE'S STARTER AUTONS (chunk ROB1-A, 2026-11-11; the ROB1 brief's D8 and test T12). Say "host test
// plant", never "simulator". Robot one's facts are typed from its builder's answers (2026-11-15), not
// measured: nothing here has run on it.
//
// WHAT THIS FILE TRIES TO BREAK
//   * THE SKILLS KIND (T12, GM23): the only competition on Nov 21 is skills, so robot one's 40 in is a SKILLS
//     auton — a Match kind could never be picked for a skills run. Its schedule fits the 60 s period with the
//     end action's runway, and so does the acceptance run's.
//   * EXACTLY WHAT THE BRIEF ALLOWS (G4): two entries — the 40 in and the acceptance run — and nothing else
//     (the skills route is the students'; a third entry here would be the builder writing it).
//   * EVERY ENTRY RUNS on the host test plant through the Auton Tester's own sequence (autons_test.cpp's
//     chain) and ends safe — and ENDS WHERE IT SHOULD (the review of ROB1-A, RV1): the 40 in within 0.5 in of
//     (40, 0), the acceptance run back at its start reversed, by the plant's true pose AND the robot's own
//     estimate (robot two's bounds, test/autons_test.cpp autons (3) and (4)). HONEST LIMIT: that stack binds ROBOT TWO'S fake mechanisms (tester_rig's
//     TesterStack); robot one's two entries step no mechanism, so what this proves is the drive path and
//     the registration — robot one's cascade on the host test plant is test/robot_one_tables_test.cpp's.

#include "doctest.h"

#include <cmath>
#include <cstddef>
#include <string>
#include <string_view>

#include "auton_tester_rig.hpp"
#include "programs/auton_tester_run.hpp"
#include "shulib/competition/acceptance_run.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/competition/match_runner.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/units/quantity.hpp"
#include "source_pins.hpp"
#include "team/autons.hpp"
#include "team/robot_one_autons.hpp"

using shulib::bench::kAutonPeriods;
using shulib::bench::registerRobotOneAutons;
using shulib::bench::testerResultWord;
using shulib::bench::TesterRunRecord;
using shulib::competition::AutonKind;
using shulib::competition::AutonRunOutcome;
using shulib::competition::AutonRegistry;
using shulib::competition::StartPose;
using shulib::kinematics::TankKinematics;
using shulib::math::Pose2d;
using shulib::units::Length;
using tester_rig::TesterStack;

namespace {

const TankKinematics& tank() {
    static const TankKinematics k{Length{12.0}};
    return k;
}

AutonRegistry robotOneRegistry() {
    AutonRegistry reg{kAutonPeriods};
    registerRobotOneAutons(reg);
    return reg;
}

}  // namespace

TEST_CASE("robot one autons (T12): exactly the 40 in as a SKILLS auton, the acceptance run and the shared skills route AS WRITTEN, each fitting the 60 s period") {
    // BUG CAUGHT (GM23): the 40 in left a Match auton — it could never be picked for a skills run, the only
    // competition on the 21st; or a schedule that does not leave the end action its runway inside 60 s; or a
    // fourth entry; or robot one's skills route registered MIRRORED (robot two's line, on the wrong robot).
    // (ROB1-BT: the third entry is the team's skills route, src/team/skills_route.cpp — its STEPS are the
    // students' (G4); its registration is plumbing, the route maker's contract.)
    const AutonRegistry reg = robotOneRegistry();
    REQUIRE_MESSAGE(reg.size() == 3, "src/team/robot_one_autons.cpp registers ", reg.size(),
                    " autons and this test pins 3. Added one? Pin it here too (and in robot two's file: the twins "
                    "register the same list -- test/robot_twin_test.cpp)");
    const auto& forty = reg.at(0);
    CHECK(std::string_view{forty.name} == "40in forward");
    CHECK(forty.kind == AutonKind::Skills);
    CHECK(forty.routine != nullptr);
    CHECK(forty.start.mode() == StartPose::Mode::Relative);
    CHECK(forty.endActionAt.value() == 57.0);
    CHECK(forty.hardStopAt.value() == 59.0);
    CHECK(forty.endActionAt.value() < forty.hardStopAt.value());
    CHECK(forty.hardStopAt.value() < 60.0);
    CHECK(reg.periodFor(forty).value() == 60.0);
    CHECK(forty.endAction == nullptr);

    const auto& accept = reg.at(1);
    CHECK(std::string_view{accept.name} == shulib::competition::kAcceptanceRunName);
    CHECK(accept.kind == AutonKind::Skills);
    CHECK(accept.endActionAt.value() == 57.0);
    CHECK(accept.hardStopAt.value() == 59.0);
    CHECK(reg.periodFor(accept).value() == 60.0);
    const auto& skills = reg.at(2);
    CHECK(std::string_view{skills.name} == "skills");
    CHECK(skills.kind == AutonKind::Skills);
    CHECK(skills.mirror == nullptr);  // robot one runs the route AS WRITTEN, in its own numbers
    CHECK(skills.endActionAt.value() == 57.0);
    CHECK(skills.hardStopAt.value() == 59.0);
    // Nothing was refused: a Throw registry would have thrown, and the counts say so.
    CHECK(reg.refusedCount() == 0);
}

TEST_CASE("robot one autons (T12): EVERY registered entry runs through the tester's own sequence on the host test plant and ends safe") {
    // BUG CAUGHT: a robot-one entry that the registry takes but the tester cannot run to a safe end (a
    // schedule, a start, a routine that throws) — found on the bench instead of here.
    const AutonRegistry reg = robotOneRegistry();
    REQUIRE(reg.size() >= 1);
    for (std::size_t i = 0; i < reg.size(); ++i) {
        const std::string name{reg.at(i).name};
        CAPTURE(name);
        TesterStack s{tank()};
        const TesterRunRecord& r = s.run(reg, static_cast<int>(i));
        const std::string word{testerResultWord(r)};
        MESSAGE("robot one auton '", name, "' on the host test plant: ", word);
        CHECK(word != "REFUSED");
        CHECK(word != "PROGRAM ERROR");
        CHECK(word != "NOT STARTED");
        CHECK(word != "HARD STOP");
        CHECK(word != "ABORTED");
        CHECK_FALSE(r.threw);
        CHECK(r.run.started());
        CHECK(r.run.guard.endActionRan);
        CHECK(r.run.guard.endActionSucceeded);
        CHECK(s.driveSafe());
        CHECK_FALSE(s.guard.running());
    }
}

TEST_CASE("robot one autons (T12): the 40 in ENDS 40 in ahead and the acceptance run ENDS back at its start, reversed -- by the plant's true pose AND the robot's own estimate") {
    // BUG CAUGHT (the review of ROB1-A, RV1): the case above checks only that each entry ends SAFE — robot
    // one's 40 in retargeted to 20 in kept the whole suite green. Robot two's tests of the same two autons
    // pin where they end (test/autons_test.cpp, autons (3) and (4)); these are the same bounds.
    const AutonRegistry reg = robotOneRegistry();
    REQUIRE(reg.size() == 3);
    {
        TesterStack s{tank()};
        const TesterRunRecord& r = s.run(reg, 0);
        REQUIRE(r.run.outcome == AutonRunOutcome::RoutineRan);
        CHECK(std::string_view{testerResultWord(r)} == "FINISHED");
        CHECK(r.run.routine.ok);
        CHECK(r.run.routine.steps == 1);
        CHECK(r.run.routine.completed == 1);
        const Pose2d truth = s.rig.h.truePose();
        CHECK(std::abs(truth.x().value() - 40.0) < 0.5);
        CHECK(std::abs(truth.y().value()) < 0.5);
        CHECK(std::abs(truth.heading().degrees()) < 1.2);
        REQUIRE(r.endLive);
        CHECK(std::abs(r.endEstimate.x().value() - 40.0) < 0.5);
        CHECK(std::abs(r.endEstimate.y().value()) < 0.5);
        CHECK(std::abs(r.endEstimate.heading().degrees()) < 1.2);
    }
    {
        TesterStack s{tank()};
        const TesterRunRecord& r = s.run(reg, 1);
        REQUIRE(r.run.outcome == AutonRunOutcome::RoutineRan);
        CHECK(std::string_view{testerResultWord(r)} == "FINISHED");
        CHECK(r.run.routine.steps == 4);
        CHECK(r.run.routine.completed == 4);
        const Pose2d truth = s.rig.h.truePose();
        CHECK(std::hypot(truth.x().value(), truth.y().value()) < 0.6);
        CHECK(std::abs(std::abs(truth.heading().degrees()) - 180.0) < 3.0);
        REQUIRE(r.endLive);
        CHECK(std::hypot(r.endEstimate.x().value(), r.endEstimate.y().value()) < 0.6);
        CHECK(std::abs(std::abs(r.endEstimate.heading().degrees()) - 180.0) < 3.0);
    }
}

TEST_CASE("robot one autons (G4): the file is robot one's own -- it names robot one's list, never robot two's, and registers nothing but its three") {
    // BUG CAUGHT: robot one's autons file reaching into robot two's mechanisms (`robot_two::`), or the
    // builder writing a route the students own (any registry.add beyond the three; the route's STEPS live in
    // src/team/skills_route.cpp, written by the students or their route maker).
    const std::string code = source_pins::stripComments(source_pins::readFile("src/team/robot_one_autons.cpp"));
    CHECK(source_pins::count(code, "using namespace robot_one;") == 1);
    CHECK(source_pins::count(code, "robot_two") == 0);
    CHECK(source_pins::count(code, "registry.add(") == 3);
    CHECK(source_pins::count(code, "registry.add(skillsRoute());") == 1);
    CHECK(source_pins::count(code, "mirrored(") == 0);
    CHECK(source_pins::count(code, "AutonKind::Match") == 0);
}
