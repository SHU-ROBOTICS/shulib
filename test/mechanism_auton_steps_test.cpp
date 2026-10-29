// MECH-A4 — the Routine's MECHANISM STEPS and their verdict policy (chassis/routine.hpp's MECH-A4
// block), on the host test plant: every step form; SUCCEEDED and an UNCONFIRMED counted done continue
// the chain; TIMED_OUT, STALLED, NOT_HOMED, ABSENT, REFUSED, CANCELLED and an UNCONFIRMED NOT counted
// each STOP it as RoutineStopCause::MechanismStepFailed — never MechanismFailed, which keeps its F1
// meaning — with the verdict in the stop line, in RoutineResult::mechanismVerdict and in
// lastMechanism(); a START form's failed start; waitUntilDone; the frozen constructor's precondition.
// Every case names the bug it would catch. Nothing here has run on a robot.

#include "doctest.h"

#include <cmath>
#include <string>

#include "mechanism_auton_rig.hpp"
#include "mechanism_fixtures.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/core/check.hpp"
#include "shulib/hal/mechanism.hpp"
#include "shulib/manipulation/mechanism_outcome.hpp"
#include "shulib/mechanism/service.hpp"

using auton_rig::RobotTwo;
using auton_rig::RobotX;
using shulib::chassis::RoutineStopCause;
using shulib::manipulation::MechanismOutcome;
using namespace mech_fixture::robot_x;  // arm, lift, flywheel, intake, hood — and the words

namespace {

/// The two members the Routine's context constructor reads, over a rig (an auton's AutonContext has
/// the same two; a test needs no runAuton to drive a Routine).
struct Ctx {
    shulib::chassis::Chassis& chassis;
    shulib::mechanism::MechanismService* mechanismService;
};

/// How many RTN Warn lines contain `needle`.
int rtn(const RobotX& x, const std::string& needle) { return x.count("RTN", needle); }

}  // namespace

TEST_CASE("MECH-A4 steps: every BLOCKING form reaches SUCCEEDED and the chain goes on") {
    // Bug caught: a step form that commands the wrong behaviour, waits for the wrong slot, or records
    // a SUCCEEDED as a stop — the chain must run every line, in order, each to its verdict.
    RobotX x;
    x.world.sensorSeesAt = 0.4;
    x.world.sensorGoneAt = 2.2;
    Ctx ctx{x.chassis, &x.svc};
    Routine r{ctx, "every-blocking"};
    r.home(lift, 1.5_s);
    CHECK(r.lastMechanism().outcome == MechanismOutcome::Succeeded);
    r.run(intake, 12_volt, seesPiece, 2_s);  // a piece arrives at 0.4 s
    CHECK(r.lastMechanism().doneBy == shulib::mechanism::DoneBy::Sees);
    r.move(arm["score"], 1.5_s);
    CHECK(std::abs(x.armPlant.trueDeg() - 95.0) < 1.5);
    r.move(arm.to(120), 1.5_s);
    r.move(arm.clampedTo(175.0), 2_s);  // a computed target past 160: clamped, logged, SUCCEEDED at 160
    CHECK(x.count("MECH", "'arm': target 175.00 clamped to the travel limit 160.00 (soft stop)") == 1);
    r.move(lift.to(18_in), 4_s);
    CHECK(std::abs(x.liftInches() - 18.0) < 0.5);
    r.move(lift["low"], 4_s);
    r.spinUp(flywheel["shoot"], 2_s);
    CHECK(x.flyPlant.trueRpm() > 2900.0);
    r.set(hood["middle"]).set(hood["up"]);
    CHECK(x.lineC.commanded());
    CHECK(x.lineD.commanded());
    r.run(intake, -6_volt, stopsSeeingPiece, 2_s);  // the piece is gone at 2.2 s
    CHECK(r.lastMechanism().doneBy == shulib::mechanism::DoneBy::StopsSeeing);
    r.run(intake, 12_volt, 0.3_s);
    CHECK(r.lastMechanism().doneBy == shulib::mechanism::DoneBy::Time);
    r.stop(flywheel).stop(intake);
    CHECK(r.ok());
    CHECK(r.result().steps == 14);
    CHECK(r.result().completed == 14);
    CHECK(r.result().mechanismVerdict == MechanismOutcome::Running);
    CHECK(x.flyM.commandedVoltage().value() == 0.0);  // the stop: its safe state (Coast, 0 V)
}

TEST_CASE("MECH-A4 steps: a sensorless grab -- UNCONFIRMED, COUNTED done -- continues the chain (Q2 (c))") {
    // Bug caught: the policy reading UNCONFIRMED as a failure (robot two's every grab would end its
    // auton) — or reading an UNCOUNTED one as done (the next case).
    RobotTwo two;
    Ctx ctx{two.chassis, &two.svc};
    Routine r{ctx, "grab"};
    using namespace mech_fixture::robot_two;
    r.set(claw["open"]).set(claw["closed"]);
    CHECK(r.lastMechanism().outcome == MechanismOutcome::Unconfirmed);
    CHECK(r.lastMechanism().countedDone);
    r.run(clawPivot6, 12_volt, 0.4_s).run(clawPivot7, 12_volt, 0.4_s);
    CHECK(r.ok());
    CHECK(r.result().completed == 4);
    CHECK_FALSE(two.lineA.commanded());  // closed = Low
}

TEST_CASE("MECH-A4 steps: an UNCONFIRMED NOT counted (a piece sensor that stopped answering) STOPS the "
          "chain") {
    // Bug caught: the policy continuing on ANY UNCONFIRMED — a step whose sensor went blind may have
    // missed the very piece it waited for, and the auton must not act as if it had it.
    RobotX x;
    x.world.sensorBlind = true;
    Ctx ctx{x.chassis, &x.svc};
    Routine r{ctx, "blind"};
    r.run(intake, 12_volt, seesPiece, 0.5_s).move(arm["score"], 1_s);
    CHECK_FALSE(r.ok());
    CHECK(r.lastMechanism().outcome == MechanismOutcome::Unconfirmed);
    CHECK_FALSE(r.lastMechanism().countedDone);
    CHECK(r.result().mechanismVerdict == MechanismOutcome::Unconfirmed);
    CHECK(r.result().cause == RoutineStopCause::MechanismStepFailed);
    CHECK(r.result().skipped == 1);
    CHECK(x.count("MECH", "its piece sensor stopped answering") == 1);
}

TEST_CASE("MECH-A4 steps: TIMED_OUT stops the chain as MechanismStepFailed -- the verdict in the stop line, "
          "in mechanismVerdict and in lastMechanism()") {
    // Bug caught: a failing mechanism step reported under F1's MechanismFailed (a then()-action's
    // cause), a verdict missing from any of its three places, or a chain that keeps going.
    RobotX x;  // no piece ever comes
    Ctx ctx{x.chassis, &x.svc};
    Routine r{ctx, "timeout"};
    r.run(intake, 12_volt, seesPiece, 0.5_s).move(arm["score"], 1_s);
    CHECK_FALSE(r.ok());
    CHECK(r.result().cause == RoutineStopCause::MechanismStepFailed);
    CHECK(r.result().cause != RoutineStopCause::MechanismFailed);
    CHECK(r.result().mechanismVerdict == MechanismOutcome::TimedOut);
    CHECK(r.lastMechanism().outcome == MechanismOutcome::TimedOut);
    CHECK(r.result().exit == shulib::control::ExitReason::Running);  // no motion verdict here
    CHECK(r.result().stoppedAt == 1);
    CHECK(std::string{r.result().stoppedName} == "intake");
    CHECK(r.result().skipped == 1);
    CHECK(rtn(x, "routine 'timeout' STOPPED at step 1 (intake): mechanism step FAILED (TIMED_OUT) — skipping the "
                 "rest; drive safed") == 1);
    CHECK(rtn(x, "step 2 (arm) skipped") == 1);
    CHECK(std::abs(x.armPlant.trueDeg()) < 2.5);  // the skipped move never ran
}

TEST_CASE("MECH-A4 steps: STALLED stops the chain (a jammed flywheel)") {
    // Bug caught: a stall mapped to anything the policy continues on.
    RobotX x;
    x.flyPlant.setJammed(true);
    Ctx ctx{x.chassis, &x.svc};
    Routine r{ctx, "jam"};
    r.spinUp(flywheel["shoot"], 3_s).stop(flywheel);
    CHECK_FALSE(r.ok());
    CHECK(r.result().cause == RoutineStopCause::MechanismStepFailed);
    CHECK(r.result().mechanismVerdict == MechanismOutcome::Stalled);
    CHECK(rtn(x, "mechanism step FAILED (STALLED)") == 1);
    CHECK(x.flyM.commandedVoltage().value() == 0.0);  // the stall's safe state
}

TEST_CASE("MECH-A4 steps: NOT_HOMED stops the chain; a START form's NOT_HOMED start stops it too") {
    // Bug caught: a move before homing that "succeeds" (nothing moved), or a start form that hides its
    // failed start until a later waitUntilDone that may never come.
    {
        RobotX x{true, /*liftOnSwitchAtBoot=*/false};
        REQUIRE_FALSE(x.svc.position(1).homed());
        Ctx ctx{x.chassis, &x.svc};
        Routine r{ctx, "unhomed"};
        r.move(lift.to(18_in), 2_s);
        CHECK(r.result().mechanismVerdict == MechanismOutcome::NotHomed);
        CHECK(r.result().cause == RoutineStopCause::MechanismStepFailed);
    }
    {
        RobotX x{true, false};
        Ctx ctx{x.chassis, &x.svc};
        Routine r{ctx, "unhomed-start"};
        const long paces0 = x.world.paces;
        r.startMove(lift.to(18_in), 2_s).driveTo(12_in, 0_in);
        CHECK(x.world.paces == paces0);  // the drive was skipped: nothing paced
        CHECK(r.result().stoppedAt == 1);
        CHECK(r.result().mechanismVerdict == MechanismOutcome::NotHomed);
        CHECK(r.result().skipped == 1);
    }
}

TEST_CASE("MECH-A4 steps: ABSENT -- a mechanism not built this boot, another robot's, an unbound service") {
    // Bug caught: a step on a mechanism the robot lacks that does nothing SILENTLY, or says the wrong
    // reason — a team chasing "not built" when the auton names another robot's claw.
    {
        RobotX x{true, true, [](shulib::mechanism::Devices (&d)[5]) {
                     d[3] = shulib::mechanism::Devices{.absence = "port 5: nothing answered"};
                 }};
        Ctx ctx{x.chassis, &x.svc};
        Routine r{ctx, "absent"};
        r.run(intake, 12_volt, 0.5_s);
        CHECK(r.result().mechanismVerdict == MechanismOutcome::Absent);
        CHECK(x.count("MECH", "'intake': ABSENT in 0.00s (not built at boot: port 5: nothing answered)") == 1);
        CHECK(rtn(x, "mechanism step FAILED (ABSENT)") == 1);
    }
    {
        RobotX x;
        Ctx ctx{x.chassis, &x.svc};
        Routine r{ctx, "another-robot"};
        r.set(mech_fixture::robot_two::claw["closed"]);
        CHECK(r.result().mechanismVerdict == MechanismOutcome::Absent);
        CHECK(x.count("MECH", "'claw' is not on this robot (its wiring list does not have it): the step did nothing") ==
              1);
    }
    {
        // A same-NAMED mechanism of another KIND: robot y's "arm"-free list has none, so build one —
        // robot z holds robot x's arm; a lift named "arm" on another robot would be found by name but
        // not by units. Here: an Arm goal of a preset this robot's arm does not hold at that value.
        RobotX x;
        Ctx ctx{x.chassis, &x.svc};
        Routine r{ctx, "stranger"};
        r.move(mech_fixture::robot_y::swing["up"], 1_s);  // "swing" is not on robot x at all
        CHECK(x.count("MECH", "'swing' is not on this robot") == 1);
    }
    {
        // the service not bound at all
        auton_rig::TankWorld w;
        shulib::hal::fake::FakeTelemetrySink sink;
        shulib::kinematics::TankKinematics kin{shulib::units::Length{12.0}};
        motion_rig::MotionRig rig{kin, motion_rig::plantConfig(), &sink};
        w.rig = &rig;
        shulib::mechanism::MechanismService svc{w};
        shulib::chassis::Chassis chassis{rig.deps, svc, shulib::chassis::ChassisConfig{.motion = motion_rig::motionConfig()}};
        Ctx ctx{chassis, &svc};
        Routine r{ctx, "unbound"};
        r.move(arm["score"], 1_s);
        CHECK(r.result().mechanismVerdict == MechanismOutcome::Absent);
        bool said = false;
        for (int i = 0; i < sink.size(); ++i) {
            said = said || sink.at(i).message.find("'arm': the mechanism service is not bound") != std::string::npos;
        }
        CHECK(said);
    }
}

TEST_CASE("MECH-A4 steps: a goal of ANOTHER robot's same-named mechanism is ABSENT, said so") {
    // Bug caught (landmine 6, A3-D22): an auton compiled into another robot's build moving a
    // same-named mechanism to a stranger's preset — and a sentence that says "not on this robot" when
    // a mechanism of that name IS there.
    using namespace shulib::mechanism::words;
    static constexpr Arm otherArm{.name = "arm", .motors = {Forward}, .cartridge = Green, .gearing = gears(12, 60),
                                  .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 10, .lasting = 0.5_s},
                                  .range = {0, 160}, .gravity = noGravity, .home = restsAtStop(0),
                                  .presets = {{"score", 80}}};  // robot x's arm holds "score" at 95
    RobotX x;
    Ctx ctx{x.chassis, &x.svc};
    Routine r{ctx, "stranger"};
    r.move(otherArm["score"], 1_s);
    CHECK(r.result().mechanismVerdict == MechanismOutcome::Absent);
    CHECK(x.count("MECH", "'arm' on this robot is not the mechanism this step names (another robot's declaration: "
                          "a different kind, units, preset or position): the step did nothing") == 1);
    CHECK(std::abs(x.armPlant.trueDeg()) < 2.5);  // nothing moved
}

TEST_CASE("MECH-A4 steps: REFUSED -- another owner holds the claim; stop() on it is REFUSED too") {
    // Bug caught: a step that takes over a mechanism an F1 operation is ticking (two tickers), or a
    // stop() that "succeeds" while the other owner keeps the mechanism running.
    struct Owner final : shulib::hal::ICancellable {
        void cancel() override {}
    };
    Owner owner;
    {
        RobotX x;
        REQUIRE(x.mIntake.tryClaim(owner));
        Ctx ctx{x.chassis, &x.svc};
        Routine r{ctx, "refused"};
        r.run(intake, 12_volt, 0.5_s);
        CHECK(r.result().mechanismVerdict == MechanismOutcome::Refused);
        CHECK(x.count("MECH", "'intake': REFUSED in 0.00s (another owner holds its claim)") == 1);
        CHECK(x.intakeM.commandedVoltage().value() == 0.0);
        x.mIntake.releaseClaim();
    }
    {
        RobotX x;
        REQUIRE(x.mIntake.tryClaim(owner));
        Ctx ctx{x.chassis, &x.svc};
        Routine r{ctx, "refused-stop"};
        r.stop(intake);
        CHECK(r.result().mechanismVerdict == MechanismOutcome::Refused);
        CHECK(x.count("MECH", "'intake': REFUSED in 0.00s (another owner holds its claim: the stop did nothing)") == 1);
        CHECK(x.mIntake.claimed());
        x.mIntake.releaseClaim();
    }
    {
        RobotX x{true, true, [](shulib::mechanism::Devices (&d)[5]) {
                     d[2] = shulib::mechanism::Devices{.absence = "port 4: nothing answered"};
                 }};
        Ctx ctx{x.chassis, &x.svc};
        Routine r{ctx, "absent-stop"};
        r.stop(flywheel);
        CHECK(r.result().mechanismVerdict == MechanismOutcome::Absent);
        CHECK(x.count("MECH", "'flywheel': ABSENT in 0.00s (not built at boot: port 4: nothing answered)") == 1);
    }
}

TEST_CASE("MECH-A4 steps: the START forms return at once; waitUntilDone collects; a finished step's "
          "waitUntilDone returns at once") {
    // Bug caught: a start form that blocks (no run-while-driving), a waitUntilDone that waits on a
    // finished step's slot anyway, or one that collects the wrong verdict.
    RobotX x;
    Ctx ctx{x.chassis, &x.svc};
    Routine r{ctx, "starts"};
    long p0 = x.world.paces;
    r.startMove(arm["score"], 1.5_s).startSpinUp(flywheel["shoot"], 2_s).startRun(intake, 12_volt, 1_s).startHome(lift, 1_s);
    CHECK(x.world.paces == p0);  // four starts, no pace
    CHECK(r.ok());
    r.waitUntilDone(arm, 2_s);
    CHECK(r.lastMechanism().outcome == MechanismOutcome::Succeeded);
    r.waitUntilDone(intake, 1_s);
    CHECK(r.lastMechanism().outcome == MechanismOutcome::Succeeded);
    CHECK(r.lastMechanism().doneBy == shulib::mechanism::DoneBy::Time);
    r.run(intake, 12_volt, 0.2_s);
    p0 = x.world.paces;
    r.waitUntilDone(intake, 1_s);  // the run above already finished: at once
    CHECK(x.world.paces == p0);
    r.waitUntilDone(flywheel, 1_s).waitUntilDone(lift, 1_s);
    CHECK(r.ok());
    CHECK(r.result().completed == 10);
}

TEST_CASE("MECH-A4 steps: waitUntilDone -- nothing started is a precondition; a step REPLACED from "
          "outside is CANCELLED") {
    // Bug caught: a waitUntilDone that collects a stale verdict (a step of an earlier run, or one a
    // newer command replaced) as this routine's.
    RobotX x;
    Ctx ctx{x.chassis, &x.svc};
    {
        Routine r{ctx, "nothing-started"};
        CHECK_THROWS_AS(r.waitUntilDone(intake, 1_s), shulib::PreconditionError);
        CHECK(r.result().steps == 0);  // the throw recorded nothing
        CHECK_THROWS_AS(r.waitUntilDone(intake, 0_s), shulib::PreconditionError);
    }
    {
        Routine r{ctx, "replaced"};
        r.startRun(intake, 12_volt, 2_s);
        x.svc.power(3).run(-6_volt, shulib::mechanism::Until::time(0.5_s), 0.5_s);  // a newer command, outside
        r.waitUntilDone(intake, 1_s);
        CHECK_FALSE(r.ok());
        CHECK(r.result().mechanismVerdict == MechanismOutcome::Cancelled);
        CHECK(rtn(x, "mechanism step FAILED (CANCELLED: a newer command replaced the step this routine started)") ==
              1);
    }
}

TEST_CASE("MECH-A4 steps: the FROZEN constructor + a mechanism step is a loud precondition naming the fix") {
    // Bug caught: a Routine built from `ctx.chassis` (today's autons.cpp spelling) whose mechanism step
    // silently does nothing.
    RobotX x;
    Routine r{x.chassis, "frozen"};
    try {
        r.move(arm["score"], 1_s);
        FAIL("no precondition");
    } catch (const shulib::PreconditionError& e) {
        CHECK(std::string{e.what()}.find("construct it as Routine r{ctx, \"name\"}") != std::string::npos);
    }
    CHECK(r.result().steps == 0);
    CHECK(r.ok());
}

TEST_CASE("MECH-A4 steps: a then()-action's mechanism failure is STILL MechanismFailed (F1's meaning kept)") {
    // Bug caught: the new cause swallowing the old one — MechanismFailed must keep meaning "a then()
    // action's MechanismOutcome", unchanged.
    RobotX x;
    Ctx ctx{x.chassis, &x.svc};
    Routine r{ctx, "then"};
    r.then([] { return MechanismOutcome::Stalled; }, "op");
    CHECK(r.result().cause == RoutineStopCause::MechanismFailed);
    CHECK(r.result().mechanismVerdict == MechanismOutcome::Running);  // no mechanism STEP failed
}

TEST_CASE("MECH-A4 steps: a step's PRECONDITION throws through, the counters untouched") {
    // Bug caught: a nonsense step converted into a polite stop (the error policy: a programming error
    // stays loud), or a throw that half-recorded a step.
    RobotX x;
    Ctx ctx{x.chassis, &x.svc};
    Routine r{ctx, "pre"};
    CHECK_THROWS_AS(r.move(arm["score"], 0_s), shulib::PreconditionError);
    CHECK_THROWS_AS(r.run(intake, 12_volt, stalls, 1_s), shulib::PreconditionError);  // no stall signature
    CHECK_THROWS_AS(r.run(intake, 12_volt, seesPiece.orAfter(2_s), 1_s), shulib::PreconditionError);
    CHECK(r.result().steps == 0);
    CHECK(r.ok());
    CHECK_FALSE(x.mIntake.claimed());
}

TEST_CASE("MECH-A4 steps: a START form whose START fails -- ABSENT or REFUSED -- stops the chain at once") {
    // Bug caught: a start form that hides a failed start (nothing runs) until a later waitUntilDone that
    // may never come — the robot would drive on as if its intake were running.
    {
        RobotX x{true, true, [](shulib::mechanism::Devices (&d)[5]) {
                     d[3] = shulib::mechanism::Devices{.absence = "port 5: nothing answered"};
                 }};
        Ctx ctx{x.chassis, &x.svc};
        Routine r{ctx, "absent-start"};
        const long paces0 = x.world.paces;
        r.startRun(intake, 12_volt, 1_s).driveTo(12_in, 0_in);
        CHECK(x.world.paces == paces0);  // the drive never ran
        CHECK(r.result().mechanismVerdict == MechanismOutcome::Absent);
        CHECK(r.result().stoppedAt == 1);
    }
    {
        struct Owner final : shulib::hal::ICancellable {
            void cancel() override {}
        };
        Owner owner;
        RobotX x;
        REQUIRE(x.mIntake.tryClaim(owner));
        Ctx ctx{x.chassis, &x.svc};
        Routine r{ctx, "refused-start"};
        r.startRun(intake, 12_volt, 1_s).driveTo(12_in, 0_in);
        CHECK(r.result().mechanismVerdict == MechanismOutcome::Refused);
        CHECK(r.result().stoppedAt == 1);
        CHECK(r.result().skipped == 1);
        x.mIntake.releaseClaim();
    }
}

// ═══ MECH-A4's review round (2026-10-27): each case below is a finding's test ═══════════════════

TEST_CASE("MECH-A4 (review): a waitUntilDone SHORTER than its step stops the chain TIMED_OUT -- the wait ran "
          "out while the service ticked") {
    // Bug caught: a wait that ran out with its step still running recorded as SUCCEEDED (the auton
    // drives on as if the intake had its piece), its verdict missing from lastMechanism(), or the
    // never-ticked sentence where the service DID tick. The brief's own pattern:
    // startRun(intake, …, 3_s) … waitUntilDone(intake, shorter) with no piece coming.
    RobotX x;  // no piece ever comes
    Ctx ctx{x.chassis, &x.svc};
    Routine r{ctx, "short-wait"};
    r.startRun(intake, 12_volt, seesPiece, 3_s).waitUntilDone(intake, 0.5_s).move(arm["score"], 1_s);
    CHECK_FALSE(r.ok());
    CHECK(r.result().stoppedAt == 2);
    CHECK(r.result().cause == RoutineStopCause::MechanismStepFailed);
    CHECK(r.result().mechanismVerdict == MechanismOutcome::TimedOut);
    CHECK(r.lastMechanism().outcome == MechanismOutcome::TimedOut);
    CHECK(rtn(x, "TIMED_OUT: no verdict within the wait's bound; the step runs on to its own") == 1);
    CHECK(rtn(x, "never ticked") == 0);
    CHECK(r.result().skipped == 1);
    CHECK(x.svc.power(3).running());  // the step itself runs on to its own bound
}

TEST_CASE("MECH-A4 (review): after a stop EVERY mechanism step form is SKIPPED and commands nothing") {
    // Bug caught: a step form whose skip guard was lost — an auton that already failed would still spin
    // the flywheel up, home the lift or stop things and count them done, and its later stop would
    // overwrite the stop line's step and cause (the wrong failing step named).
    RobotX x{true, /*liftOnSwitchAtBoot=*/false};  // the lift is not homed: its move is NOT_HOMED
    Ctx ctx{x.chassis, &x.svc};
    Routine r{ctx, "skips"};
    r.move(lift.to(18_in), 1_s);
    REQUIRE(r.result().stoppedAt == 1);
    REQUIRE(r.result().mechanismVerdict == MechanismOutcome::NotHomed);
    std::uint32_t seq[5];
    for (std::uint8_t i = 0; i < 5; ++i) {
        seq[i] = x.svc.slot(i).commandSeq();
    }
    const long paces0 = x.world.paces;
    r.spinUp(flywheel["shoot"], 1_s).startSpinUp(flywheel["shoot"], 1_s).stop(flywheel).stop(intake);
    r.home(arm, 1_s).home(lift, 1_s).startHome(arm, 1_s).startHome(lift, 1_s);
    r.run(intake, 12_volt, 0.3_s).startRun(intake, 12_volt, 0.3_s).set(hood["up"]).waitUntilDone(intake, 1_s);
    CHECK(r.result().stoppedAt == 1);
    CHECK(std::string{r.result().stoppedName} == "lift");
    CHECK(r.result().mechanismVerdict == MechanismOutcome::NotHomed);
    CHECK(r.result().skipped == 12);
    CHECK(r.result().completed == 0);
    CHECK(x.world.paces == paces0);  // nothing waited
    for (std::uint8_t i = 0; i < 5; ++i) {
        CAPTURE(i);
        CHECK(x.svc.slot(i).commandSeq() == seq[i]);
    }
    CHECK(x.flyM.commandedVoltage().value() == 0.0);
}

TEST_CASE("MECH-A4 (review): waitUntilDone's 0 s time limit is refused even on a STARTED step (the EQUALITY "
          "edge, landed on)") {
    // Bug caught: `> 0` loosened to `>= 0` — the old test reached the 0 s limit only with nothing
    // started, where the OTHER precondition (no step begun) threw first and masked it.
    RobotX x;
    Ctx ctx{x.chassis, &x.svc};
    Routine r{ctx, "zero-wait"};
    r.startRun(intake, 12_volt, 1_s);
    try {
        r.waitUntilDone(intake, 0_s);
        FAIL("no precondition");
    } catch (const shulib::PreconditionError& e) {
        CHECK(std::string{e.what()}.find("the time limit must be finite and above 0") != std::string::npos);
    }
    CHECK(r.result().steps == 1);  // the throw recorded nothing
    r.waitUntilDone(intake, 2_s);  // the started step is still collectable
    CHECK(r.ok());
}

TEST_CASE("MECH-A4 (review): spinUp(flywheel.clampedTo(x)) clamps a COMPUTED speed and goes on; set() is "
          "collectable by waitUntilDone") {
    // Bug caught: spinUp passing `computed = false` (a computed speed past the free speed would throw
    // mid-auton instead of being clamped and said), or set() not noting its step as this routine's
    // (`r.set(claw["open"]).waitUntilDone(claw, 1_s)` would then throw).
    RobotX x;
    Ctx ctx{x.chassis, &x.svc};
    {
        Routine r{ctx, "computed"};
        r.spinUp(flywheel.clampedTo(5000.0), 1_s);  // no throw: clamped to the 3600 rpm free speed, said
        CHECK(x.count("MECH", "'flywheel': speed 5000 rpm clamped to its free speed 3600 rpm") == 1);
        CHECK(std::abs(x.svc.speed(2).targetRpm() - 3600.0) < 1e-6);
        // the host test plant runs 5 % slow, so its free speed is out of reach: the step RAN, to its bound
        CHECK(r.result().mechanismVerdict == MechanismOutcome::TimedOut);
        CHECK(r.result().steps == 1);
    }
    {
        Routine r{ctx, "set-then-wait"};
        r.set(hood["up"]).waitUntilDone(hood, 1_s);
        CHECK(r.ok());
        CHECK(r.lastMechanism().outcome == MechanismOutcome::Succeeded);
        CHECK(r.result().completed == 2);
    }
}

TEST_CASE("MECH-A4 (review): the lines that say why a step did nothing are WARN -- ABSENT, not on this robot, "
          "a REFUSED stop") {
    // Bug caught: a "the step did nothing" line logged below Warn — a LevelFilterSink on the brain would
    // drop the one line that says why the auton's mechanism never moved.
    struct Owner final : shulib::hal::ICancellable {
        void cancel() override {}
    };
    using shulib::hal::LogLevel;
    {
        RobotX x;
        Ctx ctx{x.chassis, &x.svc};
        Routine r{ctx, "another-robot"};
        r.set(mech_fixture::robot_two::claw["closed"]);
        CHECK(x.countAt(LogLevel::Warn, "MECH", "'claw' is not on this robot") == 1);
    }
    {
        Owner owner;
        RobotX x;
        REQUIRE(x.mIntake.tryClaim(owner));
        Ctx ctx{x.chassis, &x.svc};
        Routine r{ctx, "refused-stop"};
        r.stop(intake);
        CHECK(x.countAt(LogLevel::Warn, "MECH", "'intake': REFUSED in 0.00s (another owner holds its claim: the stop "
                                                "did nothing)") == 1);
        x.mIntake.releaseClaim();
    }
    {
        RobotX x{true, true, [](shulib::mechanism::Devices (&d)[5]) {
                     d[2] = shulib::mechanism::Devices{.absence = "port 4: nothing answered"};
                 }};
        Ctx ctx{x.chassis, &x.svc};
        Routine r{ctx, "absent-stop"};
        r.stop(flywheel);
        CHECK(x.countAt(LogLevel::Warn, "MECH", "'flywheel': ABSENT in 0.00s (not built at boot") == 1);
    }
}
