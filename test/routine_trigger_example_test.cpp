// STEP1-B — the code the recipe chapter's partway-trigger section and the mechanism cookbook's trigger
// recipe quote (docs/guide/09-the-recipe-api.md, case guide-09f; docs/cookbook/07-mechanisms.md, case
// cookbook-07b): every quoted line is here VERBATIM, compiled, and run through the REAL runAuton on the
// MECH-A4 rig (robot x on the host test plant — test/mechanism_fixtures.hpp's declarations), in the
// context form the STEP1 brief requires (re-check #8). The doc gate's check-examples finds each quoted
// line in this file. A file of its own (as STEP1-A's routine_on_fail_example_test.cpp): it needs the rig
// and robot x's words, which the guide's and cookbook's shared example files do not have. Say "host test
// plant", never "simulator". Nothing here has run on a robot.

#include "doctest.h"

#include <cmath>

#include "mechanism_auton_rig.hpp"
#include "mechanism_fixtures.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/competition/match_runner.hpp"

namespace trigger_examples {
using namespace mech_fixture::robot_x;  // the robot's own namespace: its declarations and the words

// ── guide-09f: partway through a drive ──────────────────────────────────────────────────
RoutineResult partway(AutonContext& ctx) {
    Routine r{ctx, "partway"};
    r.afterTurning(45_deg).startSpinUp(flywheel["shoot"], 2_s).turnTo(90_deg);
    r.atPercent(60).startMove(lift["high"], 5_s)          // at 24 in of the 40, the lift starts up
     .afterDistance(10_in).startRun(intake, 12_volt, seesPiece.orAfter(2_s), 3_s)
     .driveTo(0_in, 40_in);
    r.waitUntilDone(intake, 2_s);
    r.waitUntilDone(lift, 3_s);
    r.waitUntilDone(flywheel, 1_s);
    return r.result();
}

// ── cookbook-07b: start the lift partway through a drive ────────────────────────────────
RoutineResult liftOnTheWay(AutonContext& ctx) {
    Routine r{ctx, "lift-on-the-way"};
    r.atPercent(60).startMove(lift["high"], 5_s).driveTo(40_in, 0_in);  // at 24 in, the lift starts up
    r.waitUntilDone(lift, 3_s);      // up by now (or the routine says why it is not)
    r.run(intake, -12_volt, 0.5_s);  // score
    return r.result();
}

// The same auton written the way it had to be before triggers: lift, THEN drive (the time it saves).
RoutineResult liftThenDrive(AutonContext& ctx) {
    Routine r{ctx, "lift-then-drive"};
    r.move(lift["high"], 5_s);
    r.driveTo(40_in, 0_in);
    r.run(intake, -12_volt, 0.5_s);
    return r.result();
}

// ── guide-10i: one tier down — the leg OBSERVER a trigger rides (Tier 3) ────────────────
struct Halfway {
    bool reached = false;
    static void onTick(void* self, const shulib::motion::LegProgress& p) {  // inside the tick: no blocking
        if (p.live && p.fraction >= 0.5) {
            static_cast<Halfway*>(self)->reached = true;
        }
    }
};
shulib::control::ExitReason watchedDrive(shulib::chassis::Chassis& chassis, Halfway& halfway) {
    const shulib::math::Pose2d target{40_in, 0_in, 0_deg};
    return chassis.moveTo(target, {}, shulib::chassis::LegObserver{&Halfway::onTick, &halfway});
}
}  // namespace trigger_examples

TEST_CASE("guide-09f: the chapter's three marks -- each trigger fires partway, every step SUCCEEDS") {
    // Bug caught: the chapter's listing not compiling, or a trigger it shows not firing (each mechanism's
    // step count is 1 and every wait collects a SUCCEEDED step).
    auton_rig::RobotX x;
    x.world.sensorSeesAt = 2.0;
    const auto res = shulib::competition::runAuton(x.chassis, x.guard,
                                                   auton_rig::entryFor("partway", &trigger_examples::partway), nullptr,
                                                   x.sink, x.svc, auton_rig::kPeriod);
    REQUIRE(res.outcome == shulib::competition::AutonRunOutcome::RoutineRan);
    CHECK(res.routine.ok);
    CHECK(res.routine.steps == 5);  // two legs and three waits: the triggers are no steps
    CHECK(res.routine.completed == 5);
    CHECK(x.svc.slot(2).stepsBegun() == 1);  // the flywheel
    CHECK(x.svc.slot(1).stepsBegun() == 1);  // the lift
    CHECK(x.svc.slot(3).stepsBegun() == 1);  // the intake
}

TEST_CASE("cookbook-07b: the lift starts partway through the drive -- the same auton, over a second sooner") {
    // Bug caught: the recipe's claim that the lift rises WHILE the robot drives (its point): the lift is
    // moving before the drive ends, it is up, and the run is shorter than lift-then-drive's.
    double triggered = 0.0;
    double driveEnd = 0.0;
    double end = 0.0;
    {
        auton_rig::RobotX x;
        x.world.onPace = [&] {
            if (triggered == 0.0 && x.svc.slot(1).running()) {
                triggered = x.now();
            }
        };
        const auto res = shulib::competition::runAuton(
            x.chassis, x.guard, auton_rig::entryFor("lift-on-the-way", &trigger_examples::liftOnTheWay), nullptr,
            x.sink, x.svc, auton_rig::kPeriod);
        CHECK(res.routine.ok);
        CHECK(res.routine.completed == 3);
        driveEnd = 1.80;  // the 40 in leg's settle on this plant (routine_trigger_test.cpp)
        CHECK(triggered > 0.0);
        CHECK(triggered < driveEnd);  // the lift started DURING the drive
        CHECK(std::abs(x.liftInches() - 22.0) < 0.5);
        end = x.now();
    }
    auton_rig::RobotX y;
    const auto seq = shulib::competition::runAuton(
        y.chassis, y.guard, auton_rig::entryFor("lift-then-drive", &trigger_examples::liftThenDrive), nullptr, y.sink,
        y.svc, auton_rig::kPeriod);
    CHECK(seq.routine.ok);
    CHECK(end + 1.0 < y.now());  // the trigger saved more than a second
}

TEST_CASE("guide-10i: the leg observer -- the plain moveTo's leg, watched, one tier below a trigger") {
    // Bug caught: the chapter's Tier-3 listing not compiling, or the observer overload not calling it with
    // the leg's live progress (the flag never set), or running a different leg (not the plain verb's end).
    auton_rig::RobotX x;
    trigger_examples::Halfway halfway;
    const auto exit = trigger_examples::watchedDrive(x.chassis, halfway);
    CHECK(exit == shulib::control::ExitReason::Settled);
    CHECK(halfway.reached);
    CHECK(x.chassis.lastCompleted().endTime.value() == doctest::Approx(1.80));
}
