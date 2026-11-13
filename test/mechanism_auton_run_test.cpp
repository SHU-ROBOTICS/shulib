// MECH-A4 — the mechanism core in an AUTON, through the REAL competition::runAuton (its mechanism-
// service overload) on the host test plant: run while driving (the brief's P1), the students' auton
// as written (P5-run), the CONTROL with the service not in the pacer chain, the deadline cutting a
// blocking mechanism step, the driver -> auton handover (D1; M35 proper), and COMP-a unchanged. Every
// case names the bug it would catch. Say "host test plant", never "simulator": nothing here has run
// on a robot.

#include "doctest.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <type_traits>

#include "mechanism_auton_rig.hpp"
#include "mechanism_fixtures.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/competition/match_runner.hpp"
#include "shulib/core/check.hpp"
#include "shulib/hal/mechanism.hpp"
#include "shulib/manipulation/mechanism_outcome.hpp"
#include "shulib/mechanism/service.hpp"

using auton_rig::RobotX;
using shulib::chassis::RoutineStopCause;
using shulib::competition::AutonContext;
using shulib::competition::AutonRunOutcome;
using shulib::competition::AutonRunResult;
using shulib::competition::runAuton;
using shulib::manipulation::MechanismOutcome;
using namespace mech_fixture::robot_x;  // arm, lift, flywheel, intake, hood — and the words (Routine, ...)

namespace {

// ── P1, instrumented (the probe's auton): the arm moved FIRST, then held through a drive while three
//    other mechanisms run beside it; homing at auton start; the rest of the step forms after ──────
struct P1Probe {
    RobotX* x = nullptr;
    bool sampling = false;
    double worstArm = 0.0;
    bool intakeRan = false;
    double intakeStoppedAt = -1.0;
    double driveStart = 0.0;
    double driveEnd = 0.0;
    long drivePaces = 0;
    long driveTicks = 0;
    double liftAtDriveEnd = 0.0;
    double flyAtDriveEnd = 0.0;
    MechanismOutcome homeVerdict = MechanismOutcome::Running;
    MechanismOutcome intakeVerdict = MechanismOutcome::Running;
    MechanismOutcome liftVerdict = MechanismOutcome::Running;
    MechanismOutcome flyVerdict = MechanismOutcome::Running;
    double liftTrueAtDone = 0.0;
};
P1Probe gP1;

RoutineResult instrumented(AutonContext& ctx) {
    Routine r{ctx, "p1"};
    r.home(lift, 1.5_s);  // AT AUTON START: the lift rests on its switch, done at once
    gP1.homeVerdict = r.lastMechanism().outcome;
    r.move(arm["score"], 1.5_s);
    r.startMove(lift.to(18_in), 6_s).startSpinUp(flywheel["shoot"], 3_s).startRun(intake, 12_volt, seesPiece, 3_s);
    const long paces0 = gP1.x->world.paces;
    const long ticks0 = gP1.x->svc.ticks();
    gP1.driveStart = gP1.x->now();
    gP1.sampling = true;
    r.driveTo(40_in, 0_in);
    gP1.sampling = false;
    gP1.driveEnd = gP1.x->now();
    gP1.drivePaces = gP1.x->world.paces - paces0;
    gP1.driveTicks = gP1.x->svc.ticks() - ticks0;
    gP1.liftAtDriveEnd = gP1.x->liftInches();
    gP1.flyAtDriveEnd = gP1.x->flyPlant.trueRpm();
    r.waitUntilDone(intake, 1_s);
    gP1.intakeVerdict = r.lastMechanism().outcome;
    r.waitUntilDone(lift, 3_s);
    gP1.liftVerdict = r.lastMechanism().outcome;
    gP1.liftTrueAtDone = gP1.x->liftInches();
    r.waitUntilDone(flywheel, 1_s);
    gP1.flyVerdict = r.lastMechanism().outcome;
    r.set(hood["middle"]).set(hood["up"]);
    r.run(intake, 12_volt, 0.5_s).stop(flywheel);
    return r.result();
}

// THE STUDENTS' AUTON, AS WRITTEN (the brief's §1.1, verbatim).
RoutineResult scoreThree(AutonContext& ctx) {
    Routine r{ctx, "score-three"};
    r.startSpinUp(flywheel["shoot"], 2_s);         // the flywheel spins up while the robot drives
    r.startRun(intake, 12_volt, seesPiece.orAfter(2.5_s), 3_s);  // until it sees a piece (or 2.5 s)
    r.driveTo(24_in, 0_in);
    r.waitUntilDone(intake, 1_s);
    r.move(arm["score"], 1.5_s);                    // the arm HOLDS 95 degrees from here on
    r.move(lift.to(18_in), 4_s);
    r.set(hood["up"]);
    r.waitUntilDone(flywheel, 1_s);
    r.run(intake, 12_volt, 0.5_s);                  // feed the piece into the flywheel
    r.stop(flywheel);
    return r.result();
}

// A blocking intake step that waits 5 s for a piece that never comes (the deadline cuts it).
RoutineResult waitsForAPiece(AutonContext& ctx) {
    Routine r{ctx, "waits"};
    r.run(intake, 12_volt, seesPiece, 5_s);
    r.move(arm["high"], 1_s);  // never runs
    return r.result();
}

// Starts the intake for 10 s and returns: the guard's end of run must end it.
RoutineResult startsTheIntake(AutonContext& ctx) {
    Routine r{ctx, "start"};
    r.startRun(intake, 12_volt, 10_s);
    return r.result();
}

// The handover's auton: one drive, nothing of the intake's.
RoutineResult justDrive(AutonContext& ctx) {
    Routine r{ctx, "just-drive"};
    r.driveTo(12_in, 0_in);
    return r.result();
}

}  // namespace

TEST_CASE("MECH-A4 P1: the arm HOLDS through a drive, the intake stops on its sensor MID-drive, one "
          "service tick per pace -- through the REAL runAuton") {
    // Bug caught: a mechanism step that does not keep running while the Routine's blocking drive runs
    // (the service not ticked inside the drive's wait), an arm whose hold sags while the robot drives,
    // an intake that runs to its bound instead of stopping on its sensor, a service ticked twice (or
    // never) per pace, and a run that leaves a mechanism claimed or energized.
    RobotX x;
    gP1 = P1Probe{};
    gP1.x = &x;
    x.world.sensorSeesAt = 2.0;  // a piece reaches the intake 2 s in — mid-drive
    x.world.onPace = [&x] {
        if (gP1.sampling && x.chassis.scheduler().hasActiveMotion()) {
            gP1.worstArm = std::max(gP1.worstArm, std::abs(x.armPlant.trueDeg() - 95.0));
            if (x.intakeM.commandedVoltage().value() > 11.0) {
                gP1.intakeRan = true;
            }
            if (gP1.intakeStoppedAt < 0.0 && gP1.intakeRan && x.intakeM.commandedVoltage().value() == 0.0) {
                gP1.intakeStoppedAt = x.now();
            }
        }
    };
    REQUIRE(x.svc.position(0).homed());  // the arm rests at its stop
    REQUIRE(x.svc.position(1).homed());  // the lift is on its switch at bind
    const AutonRunResult res =
        runAuton(x.chassis, x.guard, auton_rig::entryFor("p1", &instrumented), nullptr, x.sink, x.svc, auton_rig::kPeriod);
    REQUIRE(res.outcome == AutonRunOutcome::RoutineRan);
    CHECK(res.routine.ok);
    CHECK(res.routine.steps == 13);
    CHECK(res.routine.completed == 13);
    CHECK(res.routine.mechanismVerdict == MechanismOutcome::Running);  // no failing mechanism step
    CHECK(gP1.homeVerdict == MechanismOutcome::Succeeded);
    // THE DRIVE: 40 in, about 1.8 s (measured 1.80 s by the brief's probe)
    CHECK(gP1.driveEnd - gP1.driveStart > 1.5);
    // THE HOLD: the probe measured 1.16 deg worst; the bound is the brief's 1.5 deg
    CHECK(gP1.worstArm < 1.5);
    CHECK(gP1.worstArm > 0.0);  // the sampler really ran during the drive
    // THE INTAKE: at 12 V during the drive, stopped by its sensor while the drive still ran
    CHECK(gP1.intakeRan);
    CHECK(gP1.intakeStoppedAt >= 2.0);
    CHECK(gP1.intakeStoppedAt < 2.05);
    CHECK(gP1.intakeStoppedAt < gP1.driveEnd);
    CHECK(gP1.intakeVerdict == MechanismOutcome::Succeeded);
    // THE LIFT and THE FLYWHEEL, started before the drive, worked during it
    CHECK(gP1.liftAtDriveEnd > 5.0);
    CHECK(gP1.flyAtDriveEnd > 2900.0);
    CHECK(gP1.liftVerdict == MechanismOutcome::Succeeded);
    CHECK(std::abs(gP1.liftTrueAtDone - 18.0) < 0.5);
    CHECK(gP1.flyVerdict == MechanismOutcome::Succeeded);
    MESSAGE("P1: drive T+" << gP1.driveStart << "..T+" << gP1.driveEnd << ", arm worst |error| " << gP1.worstArm
                           << " deg, intake 0 V at T+" << gP1.intakeStoppedAt << ", lift " << gP1.liftAtDriveEnd
                           << " in and flywheel " << gP1.flyAtDriveEnd << " rpm at the drive's end, "
                           << gP1.drivePaces << " paces / " << gP1.driveTicks << " service ticks");
    // ONE service tick per pace, during the drive and over the whole run
    CHECK(gP1.drivePaces > 150);
    CHECK(gP1.driveTicks == gP1.drivePaces);
    CHECK(x.svc.ticks() == res.guard.pacesSeen);
    // AFTER the run: every claim released, every safe state applied (the guard's end of run)
    CHECK(x.allSafeAndReleased());
    CHECK(x.count("MECH", "'intake': SUCCEEDED -- sees a piece") == 1);
}

TEST_CASE("MECH-A4 P5-run: the students' scoreThree, AS WRITTEN, runs every step to SUCCEEDED") {
    // Bug caught: a step form the brief's §1.1 auton uses that does not compile, does not run, or ends
    // other than SUCCEEDED on the robot it was written for.
    RobotX x;
    x.world.sensorSeesAt = 1.5;
    const AutonRunResult res = runAuton(x.chassis, x.guard, auton_rig::entryFor("score-three", &scoreThree), nullptr,
                                        x.sink, x.svc, auton_rig::kPeriod);
    REQUIRE(res.outcome == AutonRunOutcome::RoutineRan);
    CHECK(res.routine.ok);
    CHECK(res.routine.steps == 10);
    CHECK(res.routine.completed == 10);
    CHECK(x.count("MECH", "'intake': SUCCEEDED -- sees a piece") == 1);
    CHECK(x.count("MECH", "'arm': SUCCEEDED -- reached") == 1);
    CHECK(x.count("MECH", "'lift': SUCCEEDED -- reached") == 1);
    CHECK(x.count("MECH", "'hood': SUCCEEDED -- time") == 1);
    CHECK(x.count("MECH", "'flywheel': SUCCEEDED -- reached") == 1);
    CHECK(x.allSafeAndReleased());
}

TEST_CASE("MECH-A4 P1 CONTROL: the service NOT in the pacer chain -> the first blocking mechanism wait "
          "stops the chain, saying the service never ticked") {
    // Bug caught: a program that forgot the service in its pacer chain would see every step "succeed"
    // on its start and every wait silently run out — the wiring fault must be said, at the first
    // blocking step, by name.
    RobotX x{/*serviceInChain=*/false};
    x.world.sensorSeesAt = 1.5;
    const AutonRunResult res = runAuton(x.chassis, x.guard, auton_rig::entryFor("score-three", &scoreThree), nullptr,
                                        x.sink, x.svc, auton_rig::kPeriod);
    REQUIRE(res.outcome == AutonRunOutcome::RoutineRan);
    CHECK_FALSE(res.routine.ok);
    CHECK(res.routine.cause == RoutineStopCause::MechanismStepFailed);
    CHECK(res.routine.mechanismVerdict == MechanismOutcome::TimedOut);
    CHECK(res.routine.stoppedAt == 4);  // startSpinUp, startRun, driveTo — then waitUntilDone(intake)
    CHECK(std::string{res.routine.stoppedName} == "intake");
    CHECK(x.count("RTN", "mechanism step FAILED (TIMED_OUT: the mechanism service never ticked -- it is not in "
                         "the chassis pacer's chain)") == 1);
    CHECK(x.svc.ticks() == 0);
    CHECK(x.allSafeAndReleased());  // the guard's cancel-all still ended the started steps
}

TEST_CASE("MECH-A4: the run's DEADLINE cuts a BLOCKING mechanism step -- CANCELLED, the chain stopped, "
          "the end action runs on time") {
    // Bug caught: a blocking mechanism step's frozen wait outliving the deadline (the brief's N1): the
    // Routine must see the cut step's CANCELLED at once and stop, never sit out the step's 5 s.
    RobotX x;
    const AutonRunResult res = runAuton(x.chassis, x.guard,
                                        auton_rig::entryFor("waits", &waitsForAPiece, /*endActionAt=*/1.0, 3.0),
                                        nullptr, x.sink, x.svc, auton_rig::kPeriod);
    REQUIRE(res.outcome == AutonRunOutcome::RoutineRan);
    CHECK_FALSE(res.routine.ok);
    CHECK(res.routine.cause == RoutineStopCause::MechanismStepFailed);
    CHECK(res.routine.mechanismVerdict == MechanismOutcome::Cancelled);
    CHECK(res.routine.stoppedAt == 1);
    CHECK(res.routine.skipped == 1);
    CHECK(res.guard.scoringCut);
    CHECK(res.guard.mechanismCuts >= 1);
    CHECK(res.guard.endActionRan);
    CHECK(res.guard.endActionSucceeded);
    CHECK(res.guard.scoringEnded.value() < 1.05);  // the step ended at the cut, not at its own 5 s
    CHECK(x.allSafeAndReleased());
}

TEST_CASE("MECH-A4 (D1, M35 proper): the driver's intake left at 12 V by a deleted driver task stops "
          "when the auton starts -- runAuton's service overload calls enterAutonomous()") {
    // Bug caught: PROS deletes the driver task on a mode change, so driver control's own exit never
    // ran: a hold-to-run left at 12 V would run through the whole auton (the brief's D1: 269 of 269
    // paces at 12 V). The service overload must stop it BEFORE the guard's run.
    long at12 = 0;
    {
        RobotX x;
        x.svc.enterDriverControl();
        x.svc.power(3).drive(12_volt);  // the driver holds the intake in ...
        x.svc.tickFromDriverLoop();
        REQUIRE(x.intakeM.commandedVoltage().value() == 12.0);  // ... and PROS deletes the task here
        x.world.onPace = [&x, &at12] {
            if (x.guard.running() && x.intakeM.commandedVoltage().value() == 12.0) {
                ++at12;
            }
        };
        const AutonRunResult res = runAuton(x.chassis, x.guard, auton_rig::entryFor("drive", &justDrive), nullptr,
                                            x.sink, x.svc, auton_rig::kPeriod);
        REQUIRE(res.outcome == AutonRunOutcome::RoutineRan);
        CHECK(res.routine.ok);
        CHECK(res.guard.pacesSeen > 50);
        CHECK(at12 == 0);
        CHECK(x.intakeM.commandedVoltage().value() == 0.0);
    }
    // THE CONTROL: COMP-a's span overload never touches the service — the intake runs through the auton.
    {
        RobotX x;
        long ctl12 = 0;
        x.svc.enterDriverControl();
        x.svc.power(3).drive(12_volt);
        x.svc.tickFromDriverLoop();
        x.world.onPace = [&x, &ctl12] {
            if (x.guard.running() && x.intakeM.commandedVoltage().value() == 12.0) {
                ++ctl12;
            }
        };
        const AutonRunResult res = runAuton(x.chassis, x.guard, auton_rig::entryFor("drive", &justDrive), nullptr,
                                            x.sink, x.svc.mechanisms(), auton_rig::kPeriod);
        REQUIRE(res.outcome == AutonRunOutcome::RoutineRan);
        CHECK(ctl12 > 50);  // the D1 failure, reproduced: the control proves the case can fail
    }
}

TEST_CASE("MECH-A4 (D1): a claim ANOTHER owner left on a mechanism is RELEASED at auton start -- said, "
          "and never through its cancel()") {
    // Bug caught: an F1 operation that lived on the deleted driver task keeps its mechanism claimed
    // into the auton, so every step on it is REFUSED; calling the dead operation's cancel() would
    // touch a destroyed stack.
    struct Dead final : shulib::hal::ICancellable {
        int cancels = 0;
        void cancel() override { ++cancels; }
    };
    Dead dead;
    RobotX x;
    REQUIRE(x.mArm.tryClaim(dead));
    const AutonRunResult res = runAuton(x.chassis, x.guard, auton_rig::entryFor("drive", &justDrive), nullptr, x.sink,
                                        x.svc, auton_rig::kPeriod);
    REQUIRE(res.outcome == AutonRunOutcome::RoutineRan);
    CHECK(dead.cancels == 0);
    CHECK_FALSE(x.mArm.claimed());
    CHECK(x.svc.leftoverClaimsReleased() == 1);
    CHECK(x.count("MECH", "'arm': a claim left by another owner RELEASED as autonomous began") == 1);
}

TEST_CASE("MECH-A4: enterAutonomous runs even for a REFUSED entry -- whatever driver control left stops") {
    // Bug caught: the auton-entry stop placed after the refusals, so a refused auton (a schedule past
    // the period) leaves the driver's 12 V intake running into the period it was meant to start.
    RobotX x;
    x.svc.enterDriverControl();
    x.svc.power(3).drive(12_volt);
    x.svc.tickFromDriverLoop();
    REQUIRE(x.intakeM.commandedVoltage().value() == 12.0);
    const AutonRunResult res = runAuton(x.chassis, x.guard,
                                        auton_rig::entryFor("too-long", &justDrive, 40.0, 41.0),  // past the 30 s period
                                        nullptr, x.sink, x.svc, auton_rig::kPeriod);
    CHECK(res.outcome == AutonRunOutcome::Refused);
    CHECK(x.intakeM.commandedVoltage().value() == 0.0);
    CHECK_FALSE(x.mIntake.claimed());
}

TEST_CASE("MECH-A4: the service overload's guard span IS the service's built mechanisms -- a step left "
          "running is ended by the guard's end of run") {
    // Bug caught: the service overload handing the guard an empty (or another) span, so the end of
    // run would never reach a mechanism the service commanded — an intake started by the auton's last
    // line would run on after the auton — or a span holding an ABSENT one.
    RobotX x{true, true, [](shulib::mechanism::Devices (&d)[5]) {
                 d[2] = shulib::mechanism::Devices{.absence = "port 4: nothing answered"};  // the flywheel ABSENT
             }};
    CHECK(x.svc.mechanisms().size() == 4);
    const AutonRunResult res = runAuton(x.chassis, x.guard, auton_rig::entryFor("start", &startsTheIntake), nullptr,
                                        x.sink, x.svc, auton_rig::kPeriod);
    REQUIRE(res.outcome == AutonRunOutcome::RoutineRan);
    CHECK(res.routine.ok);
    CHECK(x.svc.power(3).lastStep().outcome == MechanismOutcome::Cancelled);
    CHECK(x.intakeM.commandedVoltage().value() == 0.0);
    CHECK_FALSE(x.mIntake.claimed());
}

TEST_CASE("MECH-A4: COMP-a unchanged -- the positional AutonContext still compiles and the span overload "
          "still runs a chassis-only routine") {
    // Bug caught: appending `mechanismService` without a default (or before `entry`) breaks every
    // program that builds an AutonContext positionally; a Routine built from such a context must run
    // its chassis steps as ever.
    static_assert(std::is_aggregate_v<AutonContext>);
    RobotX x;
    const shulib::competition::AutonEntry e = auton_rig::entryFor("drive", &justDrive);
    AutonContext ctx{x.chassis, x.guard, x.sink, x.svc.mechanisms(), e};
    CHECK(ctx.mechanismService == nullptr);
    Routine r{ctx, "chassis-only"};
    r.pause(0.05_s);
    CHECK(r.ok());
    CHECK_THROWS_AS(r.move(arm["score"], 1_s), shulib::PreconditionError);
    CHECK(r.ok());  // the throw recorded nothing
    CHECK(r.result().steps == 1);
}

// ═══ MECH-A4's review round (2026-10-27) ═══════════════════════════════════════════════════════

namespace {
RoutineResult drivesThenMovesTheArm(AutonContext& ctx) {
    Routine r{ctx, "drive-then-arm"};
    r.driveTo(12_in, 0_in);
    r.move(arm["score"], 1_s);
    return r.result();
}
}  // namespace

TEST_CASE("MECH-A4 (review, A4-D6): runAuton with an UNBOUND service still runs the auton -- the drive step "
          "completes, the mechanism step ends ABSENT saying the service is not bound") {
    // Bug caught: a bound() precondition in runAuton — a boot that failed to bind the service would then
    // cost the match's WHOLE auton (a throw before step 1) instead of only its mechanism steps.
    auton_rig::TankWorld w;
    shulib::hal::fake::FakeTelemetrySink sink;
    shulib::kinematics::TankKinematics kin{shulib::units::Length{12.0}};
    motion_rig::MotionRig rig{kin, motion_rig::plantConfig(), &sink};
    w.rig = &rig;
    shulib::mechanism::MechanismService svc{w};  // never bound
    shulib::sequence::RunGuard guard{svc};
    shulib::chassis::Chassis chassis{rig.deps, guard, shulib::chassis::ChassisConfig{.motion = motion_rig::motionConfig()}};
    const AutonRunResult res = runAuton(chassis, guard, auton_rig::entryFor("unbound", &drivesThenMovesTheArm), nullptr,
                                        sink, svc, auton_rig::kPeriod);
    REQUIRE(res.outcome == AutonRunOutcome::RoutineRan);
    CHECK(res.routine.steps == 2);
    CHECK(res.routine.completed == 1);  // the drive
    CHECK(res.routine.stoppedAt == 2);
    CHECK(res.routine.cause == RoutineStopCause::MechanismStepFailed);
    CHECK(res.routine.mechanismVerdict == MechanismOutcome::Absent);
    int said = 0;
    for (int i = 0; i < sink.size(); ++i) {
        said += sink.at(i).message.find("'arm': the mechanism service is not bound (the program bound no robot list): "
                                        "the step did nothing") != std::string::npos
                    ? 1
                    : 0;
    }
    CHECK(said == 1);
}
