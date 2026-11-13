// STEP1-A — THE CUT ALWAYS STOPS (chassis/routine.hpp's STEP1-A block, the STEP1 brief's A5 and A7), on
// the MECH-A4 rig (robot x on the host test plant) through the REAL competition::runAuton: the park at
// T+1.00 s, the hard stop at T+2.50 s — the brief's probe P2, made a test. Each scenario runs the SAME
// chain three ways — no choices (today's rule), onFail(carryOn) on every step, onFail(skipTo(...)) on
// every step — and the invariant the brief's §0.3 item 8 names is the test: FROM THE DEADLINE ON, A
// CHAIN WITH CHOICES DOES EXACTLY WHAT THE SAME CHAIN WITHOUT THEM DOES (every RunGuardReport field
// equal, the same motions issued). P2-B is the failure it prevents: a hand-made carry-on chain ran a
// frozen 2 s pause past the deadline, issued its park at T+3.04 s, the park FAILED and the hard stop
// fired. Also: a CANCELLED step stops a carry-on chain whatever cancelled it (ST16), and nothing throws
// on a run-dependent path — a guard that is not running is read with running() first (ST25, the
// brief's L3). Say "host test plant", never "simulator". Nothing here has run on a robot.

#include "doctest.h"

#include <optional>
#include <string>
#include <string_view>

#include "mechanism_auton_rig.hpp"
#include "mechanism_fixtures.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/competition/match_runner.hpp"
#include "shulib/core/check.hpp"
#include "shulib/manipulation/mechanism_outcome.hpp"
#include "shulib/mechanism/service.hpp"
#include "shulib/motion/move_to_pose.hpp"
#include "shulib/sequence/run_guard.hpp"

using auton_rig::RobotX;
using shulib::chassis::carryOn;
using shulib::chassis::RoutineStopCause;
using shulib::chassis::skipTo;
using shulib::competition::AutonContext;
using shulib::competition::AutonRunOutcome;
using shulib::competition::AutonRunResult;
using shulib::competition::runAuton;
using shulib::control::ExitReason;
using shulib::manipulation::MechanismOutcome;
using shulib::sequence::GuardedWaitResult;
using shulib::sequence::RunGuardReport;
using namespace mech_fixture::robot_x;  // arm, lift, flywheel, intake, hood — and the words

namespace {

/// Which way the scenario's chain is written.
enum class Mode { Default, CarryOn, Skip };
Mode gMode = Mode::Default;

/// Each step's modifier under the current mode (the skip's target never comes: a skip that would
/// pass over the rest).
Routine& choose(Routine& r) {
    switch (gMode) {
        case Mode::CarryOn: return r.onFail(carryOn);
        case Mode::Skip: return r.onFail(skipTo("the end"));
        case Mode::Default: break;
    }
    return r;
}

/// (a) the deadline cuts a MOTION: a 60 in drive still driving at T+1.00 (P2's step 1), then a frozen
/// pause, a drive back and an intake run.
RoutineResult cutMidMotion(AutonContext& ctx) {
    Routine r{ctx, "mid-motion"};
    choose(r).driveTo(60_in, 0_in);
    choose(r).pause(2_s);
    choose(r).driveTo(0_in, 0_in);
    choose(r).run(intake, 12_volt, 1_s);
    return r.result();
}
/// (b) the deadline passes inside a FROZEN pause (the guard cannot cut it): the pause SUCCEEDS after
/// the deadline, the next drive is issued and cut on its first pace, the intake run after it.
RoutineResult cutMidFrozenPause(AutonContext& ctx) {
    Routine r{ctx, "mid-pause"};
    choose(r).pause(0.5_s);
    choose(r).pause(2_s);
    choose(r).driveTo(12_in, 0_in);
    choose(r).run(intake, 12_volt, 1_s);
    return r.result();
}
/// (c) the deadline cuts the GUARD's own pause (a then() action returns its verdict: RunExpired is a
/// failure, not a CANCELLED) — only the guard's expired() can stop a carry-on chain here.
RoutineResult cutMidGuardWait(AutonContext& ctx) {
    Routine r{ctx, "mid-guard-wait"};
    choose(r).then([&ctx] { return ctx.guard.pause(2_s) == GuardedWaitResult::Satisfied; }, "guard pause");
    choose(r).driveTo(12_in, 0_in);
    choose(r).run(intake, 12_volt, 1_s);
    return r.result();
}
/// (d) a skip PENDING across the deadline: a step fails at T+0 and starts skipping; the auton's own
/// Tier-3 frozen wait takes the clock past the deadline; the skip's target comes after it. A skip
/// never resumes scoring after the buzzer.
RoutineResult skipAcrossTheDeadline(AutonContext& ctx) {
    Routine r{ctx, "skip-across"};
    if (gMode == Mode::Skip) {
        r.onFail(skipTo("back"));
    }
    r.then([] { return false; }, "miss");
    ctx.chassis.wait(1.5_s);  // mixed-tier code between steps: the deadline passes here
    r.run(intake, 12_volt, 0.3_s);
    r.label("back").driveTo(12_in, 0_in);
    r.run(intake, 12_volt, 0.3_s);
    return r.result();
}

/// (d), variant 1 (review RM8): the skip's label is the FIRST step after the deadline — the door must
/// read the time BEFORE it reads the label, or the skip lands and its drive is issued after the buzzer.
RoutineResult skipLabelFirstAfterTheDeadline(AutonContext& ctx) {
    Routine r{ctx, "skip-first"};
    if (gMode == Mode::Skip) {
        r.onFail(skipTo("back"));
    }
    r.then([] { return false; }, "miss");
    ctx.chassis.wait(1.5_s);  // the deadline passes here
    r.label("back").driveTo(12_in, 0_in);
    r.run(intake, 12_volt, 0.3_s);
    return r.result();
}
/// (d), variant 2 (review RM3/RM4/RM5): a step CARRIED ON first, then a skip from a step whose verdict is
/// a real exit (TIMED_OUT), cut by the deadline. The stop is the SKIP's origin, with its exit; the
/// earlier carried-on failure stays the first failed step.
RoutineResult carriedOnThenSkipAcross(AutonContext& ctx) {
    Routine r{ctx, "carried-then-skip"};
    if (gMode == Mode::Skip) {
        r.onFail(carryOn);
    }
    r.then([] { return false; }, "first");
    if (gMode == Mode::Skip) {
        r.onFail(skipTo("back"));
    }
    r.then([] { return ExitReason::TimedOut; }, "second");
    ctx.chassis.wait(1.5_s);
    r.run(intake, 12_volt, 0.3_s);
    r.label("back").driveTo(12_in, 0_in);
    return r.result();
}
/// (d), variant 3 (review RM6): the skip starts from a failed MECHANISM step (no piece comes: TIMED_OUT);
/// the stop carries its verdict.
RoutineResult mechanismSkipAcross(AutonContext& ctx) {
    Routine r{ctx, "mech-skip"};
    if (gMode == Mode::Skip) {
        r.onFail(skipTo("back"));
    }
    r.run(intake, 12_volt, seesPiece, 0.3_s);
    ctx.chassis.wait(1.5_s);
    r.run(intake, 12_volt, 0.3_s);
    r.label("back").driveTo(12_in, 0_in);
    return r.result();
}
/// (d), variant 4 (review RM7): the auton's own Tier-3 code starts a motion AFTER the deadline, between
/// the steps; the next step's door ends the pending skip. From the deadline on the skip chain must do
/// what the default chain does — the guard's latch cuts that motion in both.
std::optional<shulib::motion::MoveToPose> gStray;
RoutineResult skipThenStrayMotion(AutonContext& ctx) {
    Routine r{ctx, "skip-stray"};
    if (gMode == Mode::Skip) {
        r.onFail(skipTo("back"));
    }
    r.then([] { return false; }, "miss");
    ctx.chassis.wait(1.5_s);
    gStray.emplace(ctx.chassis.deps(),
                   shulib::math::Pose2d{shulib::units::Length{40.0}, shulib::units::Length{0.0}, shulib::math::Angle{}},
                   motion_rig::motionConfig(), 5.0);
    ctx.chassis.scheduler().async(*gStray);  // mixed-tier code, after the buzzer
    r.run(intake, 12_volt, 0.3_s);           // the door that ends the pending skip
    r.label("back").driveTo(12_in, 0_in);
    ctx.chassis.wait(0.2_s);  // a pace: the guard's latch cuts whatever motion is still live
    return r.result();
}

struct Run {
    AutonRunResult res;
    int motionsStarted = 0;
    int motionsCancelled = 0;
    std::string rtn;  ///< every RTN line, joined
};
Run runScenario(Mode mode, shulib::competition::AutonRoutine fn) {
    gMode = mode;
    RobotX x;
    Run out;
    out.res = runAuton(x.chassis, x.guard, auton_rig::entryFor("cut", fn, 1.0, 2.5), nullptr, x.sink, x.svc,
                       auton_rig::kPeriod);
    out.motionsStarted = x.chassis.scheduler().motionsStarted();
    out.motionsCancelled = x.chassis.scheduler().motionsCancelled();
    for (const std::string& l : x.lines("RTN")) {
        out.rtn += l + "\n";
    }
    gStray.reset();  // variant 4's motion, gone while the rig it drove still exists
    return out;
}
/// Every field of the guard's report, equal.
void checkSameReport(const RunGuardReport& a, const RunGuardReport& b) {
    CHECK(a.scoringCut == b.scoringCut);
    CHECK(a.endActionRan == b.endActionRan);
    CHECK(a.endActionSucceeded == b.endActionSucceeded);
    CHECK(a.floorFired == b.floorFired);
    CHECK(a.abandoned == b.abandoned);
    CHECK(a.postExpiryCancels == b.postExpiryCancels);
    CHECK(a.anonymousClaimsReleased == b.anonymousClaimsReleased);
    CHECK(a.pacesSeen == b.pacesSeen);
    CHECK(a.mechanismCuts == b.mechanismCuts);
    CHECK(a.scoringEnded.value() == b.scoringEnded.value());
    CHECK(a.endActionEnded.value() == b.endActionEnded.value());
}

}  // namespace

TEST_CASE("STEP1-A ST15 (a): the deadline cuts a motion -- a carry-on or skip chain STOPS exactly like today's") {
    // Bug caught: SM9 — carry on past a CANCELLED exit (P2-B: the frozen pause, a park at T+3.04, the
    // hard stop); the stop line not saying the choice was overruled by the CANCEL.
    const Run base = runScenario(Mode::Default, &cutMidMotion);
    REQUIRE(base.res.outcome == AutonRunOutcome::RoutineRan);
    CHECK(base.res.routine.stoppedAt == 1);
    CHECK(base.res.routine.exit == shulib::control::ExitReason::Cancelled);
    CHECK(base.res.guard.scoringCut);
    CHECK(base.res.guard.scoringEnded.value() < 1.02);  // within one tick of T+1.00
    CHECK(base.res.guard.endActionSucceeded);
    CHECK_FALSE(base.res.guard.floorFired);
    for (const Mode m : {Mode::CarryOn, Mode::Skip}) {
        CAPTURE(static_cast<int>(m));
        const Run run = runScenario(m, &cutMidMotion);
        REQUIRE(run.res.outcome == AutonRunOutcome::RoutineRan);
        checkSameReport(run.res.guard, base.res.guard);
        CHECK(run.motionsStarted == base.motionsStarted);
        CHECK(run.res.routine.stoppedAt == 1);
        CHECK(run.res.routine.skipped == 3);
        CHECK(run.res.routine.failedOn == 0);
        CHECK(run.res.routine.jumped == 0);
        CHECK(run.rtn.find("routine 'mid-motion' STOPPED at step 1 (driveTo): motion CANCELLED — the step was "
                           "CANCELLED, so its on-failure choice is ignored — skipping the rest; drive safed") !=
              std::string::npos);
    }
}

TEST_CASE("STEP1-A ST15 (b): the deadline passes inside a FROZEN pause -- every way of writing the chain does "
          "the same") {
    // Bug caught: a carry-on that goes on past the drive the guard cut after the pause (the intake run
    // would be issued after the buzzer, and the latch would have to cut it too).
    const Run base = runScenario(Mode::Default, &cutMidFrozenPause);
    REQUIRE(base.res.outcome == AutonRunOutcome::RoutineRan);
    CHECK(base.res.routine.stoppedAt == 3);  // the drive issued after the deadline, cut on its first pace
    CHECK(base.res.routine.skipped == 1);
    for (const Mode m : {Mode::CarryOn, Mode::Skip}) {
        CAPTURE(static_cast<int>(m));
        const Run run = runScenario(m, &cutMidFrozenPause);
        checkSameReport(run.res.guard, base.res.guard);
        CHECK(run.motionsStarted == base.motionsStarted);
        CHECK(run.res.routine.stoppedAt == 3);
        CHECK(run.res.routine.skipped == 1);
        CHECK(run.res.routine.failedOn == 0);
    }
}

TEST_CASE("STEP1-A ST15 (c): the deadline cuts the GUARD's pause -- no step is issued after the deadline") {
    // Bug caught: SM10 — the guard's expired() not consulted: the then() fails RunExpired (an ordinary
    // failure, not a CANCELLED), a carry-on chain would issue its drive after the buzzer (P2-C).
    const Run base = runScenario(Mode::Default, &cutMidGuardWait);
    REQUIRE(base.res.outcome == AutonRunOutcome::RoutineRan);
    CHECK(base.res.routine.stoppedAt == 1);
    CHECK(base.res.routine.cause == RoutineStopCause::ActionFailed);
    CHECK(base.res.guard.scoringEnded.value() < 1.02);
    CHECK(base.motionsStarted == 1);  // the park (the end action's brake()) — nothing of the chain's
    for (const Mode m : {Mode::CarryOn, Mode::Skip}) {
        CAPTURE(static_cast<int>(m));
        const Run run = runScenario(m, &cutMidGuardWait);
        checkSameReport(run.res.guard, base.res.guard);
        CHECK(run.motionsStarted == 1);  // the park only: no drive issued after the deadline
        CHECK(run.res.routine.stoppedAt == 1);
        CHECK(run.res.routine.failedOn == 0);
        CHECK(run.rtn.find("— the time is up, so its on-failure choice is ignored") != std::string::npos);
    }
}

TEST_CASE("STEP1-A ST15 (d): a skip still pending at the deadline STOPS -- its target never runs after the "
          "buzzer") {
    // Bug caught: SM27 — a pending skip that ignores the deadline: its label lands at T+1.5 and the
    // drive is issued after the buzzer (the default rule stopped long before; a skip must never resume
    // scoring past it).
    const Run base = runScenario(Mode::Default, &skipAcrossTheDeadline);
    REQUIRE(base.res.outcome == AutonRunOutcome::RoutineRan);
    CHECK(base.res.routine.stoppedAt == 1);
    const Run skip = runScenario(Mode::Skip, &skipAcrossTheDeadline);
    REQUIRE(skip.res.outcome == AutonRunOutcome::RoutineRan);
    checkSameReport(skip.res.guard, base.res.guard);
    CHECK(skip.motionsStarted == base.motionsStarted);
    CHECK(skip.motionsStarted == 1);  // the park only: the target's drive was never issued
    // The skip's failure became the chain's STOP, as if it had chosen stop (the cut ignores choices):
    CHECK(skip.res.routine.stoppedAt == 1);
    CHECK(skip.res.routine.cause == RoutineStopCause::ActionFailed);
    CHECK(skip.res.routine.failedOn == 0);
    CHECK(skip.res.routine.firstFailedAt == 0);
    CHECK(std::string_view{skip.res.routine.firstFailedName}.empty());  // review RM3: reset with the count
    CHECK(skip.res.routine.firstFailedCause == RoutineStopCause::None);
    CHECK(std::string_view{skip.res.routine.skipPendingTo}.empty());
    CHECK(skip.res.routine.skipped == 3);  // the jumped-at-the-door step and the two after it
    CHECK(skip.res.routine.jumped == 0);
    CHECK(skip.rtn.find("routine 'skip-across' STOPPED at step 1 (miss): action FAILED — the time ran out while "
                        "skipping to 'back' — skipping the rest; drive safed") != std::string::npos);
}

TEST_CASE("STEP1-A ST15 (d) (review): the skip's LABEL is the first step after the deadline -- it still never "
          "lands") {
    // Bug caught: SM27b (review RM8) — the door reading the label BEFORE the time: the skip lands on the
    // first step after the buzzer and its drive is issued. In (d) above a step that is not the label always
    // comes between the deadline and the label, so that order went unseen.
    const Run base = runScenario(Mode::Default, &skipLabelFirstAfterTheDeadline);
    REQUIRE(base.res.outcome == AutonRunOutcome::RoutineRan);
    const Run skip = runScenario(Mode::Skip, &skipLabelFirstAfterTheDeadline);
    REQUIRE(skip.res.outcome == AutonRunOutcome::RoutineRan);
    checkSameReport(skip.res.guard, base.res.guard);
    CHECK(skip.motionsStarted == base.motionsStarted);
    CHECK(skip.motionsStarted == 1);  // the park only: the label's drive was never issued
    CHECK(skip.res.routine.stoppedAt == 1);
    CHECK(skip.res.routine.skipped == 2);
    CHECK(skip.res.routine.jumped == 0);
    CHECK(skip.res.routine.completed == 0);
    CHECK(skip.rtn.find("routine 'skip-first' STOPPED at step 1 (miss): action FAILED — the time ran out while "
                        "skipping to 'back' — skipping the rest; drive safed") != std::string::npos);
    CHECK(skip.rtn.find("landed") == std::string::npos);
}

TEST_CASE("STEP1-A ST15 (d) (review): a deadline-stopped skip after an EARLIER carried-on failure -- the first "
          "failed step stays, the stop carries the skip origin's exit") {
    // Bug caught: review RM4 — the reset of the first-failed fields at the cut made unconditional (it would
    // wipe step 1's, and both programs' line 3b would read "first failed: step 0"); RM5 — the skip origin's
    // exit not copied into the stop (the result would read exit Running for a step that TIMED OUT).
    const Run base = runScenario(Mode::Default, &carriedOnThenSkipAcross);
    REQUIRE(base.res.outcome == AutonRunOutcome::RoutineRan);
    CHECK(base.res.routine.stoppedAt == 1);
    const Run skip = runScenario(Mode::Skip, &carriedOnThenSkipAcross);
    REQUIRE(skip.res.outcome == AutonRunOutcome::RoutineRan);
    checkSameReport(skip.res.guard, base.res.guard);
    CHECK(skip.motionsStarted == base.motionsStarted);
    const auto& rr = skip.res.routine;
    CHECK(rr.stoppedAt == 2);
    CHECK(std::string_view{rr.stoppedName} == "second");
    CHECK(rr.cause == RoutineStopCause::ActionFailed);
    CHECK(rr.exit == ExitReason::TimedOut);
    CHECK(rr.failedOn == 1);  // step 1, carried on; step 2's skip became the stop
    CHECK(rr.firstFailedAt == 1);
    CHECK(std::string_view{rr.firstFailedName} == "first");
    CHECK(rr.firstFailedCause == RoutineStopCause::ActionFailed);
    CHECK(std::string_view{rr.skipPendingTo}.empty());
    CHECK(rr.skipped == 2);
    CHECK(skip.rtn.find("routine 'carried-then-skip' STOPPED at step 2 (second): action FAILED — the time ran out "
                        "while skipping to 'back' — skipping the rest; drive safed") != std::string::npos);
}

TEST_CASE("STEP1-A ST15 (d) (review): a deadline-stopped skip from a failed MECHANISM step carries its verdict") {
    // Bug caught: review RM6 — the skip origin's mechanism verdict not copied into the stop
    // (mechanismVerdict would read Running for an intake step that TIMED OUT).
    const Run base = runScenario(Mode::Default, &mechanismSkipAcross);
    REQUIRE(base.res.outcome == AutonRunOutcome::RoutineRan);
    REQUIRE(base.res.routine.mechanismVerdict == MechanismOutcome::TimedOut);
    const Run skip = runScenario(Mode::Skip, &mechanismSkipAcross);
    REQUIRE(skip.res.outcome == AutonRunOutcome::RoutineRan);
    checkSameReport(skip.res.guard, base.res.guard);
    CHECK(skip.motionsStarted == base.motionsStarted);
    CHECK(skip.res.routine.stoppedAt == 1);
    CHECK(skip.res.routine.cause == RoutineStopCause::MechanismStepFailed);
    CHECK(skip.res.routine.mechanismVerdict == MechanismOutcome::TimedOut);
    CHECK(skip.res.routine.mechanismVerdict == base.res.routine.mechanismVerdict);
    CHECK(skip.res.routine.failedOn == 0);
}

TEST_CASE("STEP1-A ST15 (d) (review): a stray Tier-3 motion after the deadline -- the skip chain treats it "
          "exactly as the default chain does") {
    // Bug caught: review RM7, turned round — the door that ends a pending skip doing MORE than the default
    // chain does from the deadline on. The drive was already safed at the skip's own failure
    // (carryOnFrom, as a stop safes it); a second cancel there (the build before the review had one)
    // took the stray motion from the guard's latch: postExpiryCancels 0 against the default's 1.
    const Run base = runScenario(Mode::Default, &skipThenStrayMotion);
    REQUIRE(base.res.outcome == AutonRunOutcome::RoutineRan);
    const Run skip = runScenario(Mode::Skip, &skipThenStrayMotion);
    REQUIRE(skip.res.outcome == AutonRunOutcome::RoutineRan);
    checkSameReport(skip.res.guard, base.res.guard);
    CHECK(skip.motionsStarted == base.motionsStarted);
    CHECK(skip.motionsCancelled == base.motionsCancelled);
    CHECK(skip.res.routine.stoppedAt == 1);
    CHECK(base.res.guard.postExpiryCancels == 1);  // the latch cut the stray motion ...
    CHECK(skip.res.guard.postExpiryCancels == 1);  // ... in both chains
}

TEST_CASE("STEP1-A ST16: a mechanism step CANCELLED -- by the deadline's latch, or replaced from outside -- stops "
          "a carry-on chain") {
    // Bug caught: SM9 — a CANCELLED mechanism verdict followed under carryOn. The outside replacement
    // runs with NO guard (a two-member context), so only the CANCELLED rule can stop it.
    {
        RobotX x;
        const auto latch = [](AutonContext& ctx) {
            Routine r{ctx, "latch"};
            r.onFail(carryOn).run(intake, 12_volt, seesPiece, 5_s);  // no piece: the deadline cuts it
            r.onFail(carryOn).move(arm["high"], 1_s);
            return r.result();
        };
        const AutonRunResult res = runAuton(x.chassis, x.guard, auton_rig::entryFor("latch", latch, 1.0, 3.0), nullptr,
                                            x.sink, x.svc, auton_rig::kPeriod);
        REQUIRE(res.outcome == AutonRunOutcome::RoutineRan);
        CHECK(res.routine.stoppedAt == 1);
        CHECK(res.routine.mechanismVerdict == MechanismOutcome::Cancelled);
        CHECK(res.routine.failedOn == 0);
        CHECK(res.guard.scoringEnded.value() < 1.05);
        CHECK(x.count("RTN", "the step was CANCELLED, so its on-failure choice is ignored") == 1);
    }
    {
        RobotX x;
        struct Ctx {
            shulib::chassis::Chassis& chassis;
            shulib::mechanism::MechanismService* mechanismService;
        } ctx{x.chassis, &x.svc};
        Routine r{ctx, "replaced"};
        r.startRun(intake, 12_volt, 2_s);
        x.svc.power(3).run(-6_volt, shulib::mechanism::Until::time(0.5_s), 0.5_s);  // a newer command, outside
        r.onFail(carryOn).waitUntilDone(intake, 1_s).then([] { return true; }, "after");
        CHECK(r.result().stoppedAt == 2);
        CHECK(r.result().mechanismVerdict == MechanismOutcome::Cancelled);
        CHECK(r.result().failedOn == 0);
        CHECK(r.result().skipped == 1);
    }
}

TEST_CASE("STEP1-A ST25: NO THROW on a run-dependent path -- a Routine over a guard that is NOT running fails "
          "steps under every choice") {
    // Bug caught: SM18 — the cut rule reading expired() without running() first: RunGuard::expired()
    // THROWS with no run live, and programs build AutonContexts over idle guards (the tank-steer tests,
    // the COMP-a context): a failing step there would throw out of the auton.
    RobotX x;
    REQUIRE_FALSE(x.guard.running());
    const shulib::competition::AutonEntry e = auton_rig::entryFor("idle", nullptr);
    AutonContext ctx{x.chassis, x.guard, x.sink, x.svc.mechanisms(), e, &x.svc};
    Routine r{ctx, "idle-guard"};
    CHECK_NOTHROW(r.onFail(carryOn).then([] { return false; }, "a"));
    CHECK_NOTHROW(r.onFail(skipTo("L")).then([] { return false; }, "b"));
    CHECK_NOTHROW(r.startRun(intake, 12_volt, 1_s));  // jumped: marks the intake
    CHECK_NOTHROW(r.label("L").onFail(carryOn).waitUntilDone(intake, 1_s));  // the not-started answer
    CHECK_NOTHROW(r.then([] { return false; }, "c"));  // the default: a stop
    CHECK(r.result().failedOn == 3);
    CHECK(r.result().jumped == 1);
    CHECK(r.result().stoppedAt == 5);
    CHECK(r.result().firstFailedAt == 1);
}
