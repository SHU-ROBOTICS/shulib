// COOKBOOK EXAMPLES (chunk D3) — every recipe in docs/cookbook/ compiles and
// RUNS here, against the simulated plant, so a recipe can never become a lie
// with a code block around it. A cookbook whose recipes were only *plausible*
// would be worse than no cookbook: people trust recipes.
//
// Mapping (keep in sync with docs/cookbook/README.md):
//   cookbook-01a/b/c  -> docs/cookbook/01-getting-there.md
//   cookbook-02a/b    -> docs/cookbook/02-when-a-step-fails.md
//   cookbook-03a/b    -> docs/cookbook/03-timing-and-partners.md
//   cookbook-04a/b    -> docs/cookbook/04-drivetrains.md
//   cookbook-05a/b    -> docs/cookbook/05-mixing-tiers.md
//
// The cookbook quotes these bodies VERBATIM — the same anti-rot rule the guide
// uses (docs/internal/guide-maintenance.md rule 1), extended to docs/cookbook/.
// If you change code here, change the matching listing (and vice versa); a
// mismatch is a bug even when both sides work, and the drift scan in
// docs/internal/verify/ turns it red.
//
// WHERE A DURATION MATTERS, THIS FILE ASSERTS AGAINST THE SIMULATED CLOCK, not
// against a sibling literal. That is D2's second green hole (a call site
// spelling `300_s` where `300_ms` was meant passed every outcome assertion
// because nothing watched the clock); every timing claim a recipe's prose makes
// is checked here with a hand-computed bound.

#include "doctest.h"

#include <cmath>
#include <string>
#include <vector>

#include "motion_test_rig.hpp"
#include "shulib/chassis/chassis.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/kinematics/x_drive.hpp"
#include "shulib/units/literals.hpp"

using namespace shulib::units::literals;
using shulib::chassis::Chassis;
using shulib::chassis::Routine;
using shulib::chassis::RoutineResult;
using shulib::chassis::RoutineStopCause;
using shulib::chassis::TrajectoryResult;
using shulib::control::ExitReason;
using shulib::math::Pose2d;
using shulib::units::Length;
using shulib::units::Time;
using shulib::units::Velocity;

namespace {

/// Simulated seconds elapsed on a rig since `t0` — every timing assertion in
/// this file measures the SIMULATED clock, never a wall clock and never a
/// literal copied from the recipe.
double elapsedSince(motion_rig::ChassisRig& c, double t0) {
    return c.rig.h.clock().now().value() - t0;
}

/// Count of Warn-level lines the routine layer emitted (subsystem "RTN").
int routineWarnings(const shulib::hal::fake::FakeTelemetrySink& log) {
    int n = 0;
    for (int i = 0; i < log.size(); ++i) {
        if (log.at(i).subsystem == "RTN"
            && log.at(i).level == shulib::hal::LogLevel::Warn) {
            ++n;
        }
    }
    return n;
}

/// True if any logged line contains `needle`.
bool logged(const shulib::hal::fake::FakeTelemetrySink& log, const std::string& needle) {
    for (int i = 0; i < log.size(); ++i) {
        if (log.at(i).message.find(needle) != std::string::npos) {
            return true;
        }
    }
    return false;
}

// ═══ Cookbook shared pieces (quoted by more than one recipe) ═══════════════════════

// Mechanisms do not exist in shulib yet — F1/F3 build them. Until then a
// mechanism is a struct your team writes, and `then()` takes any callable, so
// the recipes below already have the shape they will keep. `grab()` returning
// false is a step FAILING (the jaws closed on nothing), not a crash.
struct Intake {
    int grabs = 0;
    int releases = 0;
    bool nextGrabSucceeds = true;
    bool grab() {
        ++grabs;
        return nextGrabSucceeds;
    }
    void release() { ++releases; }
};

// The match clock. A Routine has no clock of its own, and the frozen Chassis
// facade has no clock accessor, so reading the time means reaching through the
// Tier-3 seam. That is a real gap, reported rather than hidden.
Time clockNow(Chassis& chassis) { return chassis.deps().ctx->clock().now(); }

// ═══ cookbook-01a — the skeleton ═══════════════════════════════════════════════════

// Every autonomous routine has this shape: say where the robot is, do the work,
// park. Nothing else in this cookbook is more than this with more steps in the
// middle.
RoutineResult skeleton(Chassis& chassis) {
    Routine r{chassis, "skeleton"};
    r.startAt(Pose2d{-48_in, -24_in, 90_deg})
        .moveTo(Pose2d{-24_in, -24_in, 90_deg}, {.timeout = 4_s})
        .brake({.timeout = 2_s});
    return r.result();
}

// ═══ cookbook-01b — a two-goal side run ════════════════════════════════════════════

// One scoring stop, factored out. It takes and returns the Routine, so it
// chains exactly like a built-in step — this is how you add vocabulary to the
// recipe layer without touching the library.
Routine& scoreAt(Routine& r, Length x, Length y, Intake& intake) {
    return r.driveTo(x, y, {.timeout = 5_s, .maxLinearSpeed = Velocity{30.0}})
        .brake({.timeout = 1.5_s})
        .then([&intake] { intake.release(); }, "release")
        .pause(200_ms);
}

// A left-side run: two goals, then park. Read it top to bottom — that is the
// order the robot moves in.
RoutineResult twoGoalSideRun(Chassis& chassis, Intake& intake) {
    Routine r{chassis, "left-two-goal"};
    r.startAt(Pose2d{-48_in, -24_in, 0_deg});
    scoreAt(r, -24_in, -24_in, intake);
    scoreAt(r, -24_in, 24_in, intake);
    r.moveTo(Pose2d{-48_in, -48_in, 0_deg}, {.timeout = 6_s}).brake({.timeout = 1.5_s});
    return r.result();
}

// ═══ cookbook-02a — bail out when a grab fails ═════════════════════════════════════

// The grab is a step like any other: return false and the chain stops, parks
// the drive, and skips the rest. What the chain will NOT do is your fallback
// strategy — that is a second chain, on purpose.
RoutineResult grabOrBailOut(Chassis& chassis, Intake& intake) {
    Routine r{chassis, "grab-or-bail"};
    r.startAt(Pose2d{-48_in, -24_in, 0_deg})
        .driveTo(-24_in, -24_in, {.timeout = 5_s})
        .brake({.timeout = 1.5_s})
        .then([&intake] { return intake.grab(); }, "grab")
        .driveTo(0_in, 0_in, {.timeout = 6_s})
        .then([&intake] { intake.release(); }, "score")
        .brake({.timeout = 1.5_s});
    if (r.ok()) {
        return r.result();
    }

    // Save the verdict BEFORE the fallback: a stopped Routine never runs
    // another step, so the fallback is a new chain with its own counters, and
    // `r` is the only record of what actually went wrong.
    const RoutineResult failure = r.result();
    Routine bail{chassis, "grab-or-bail/fallback"};
    bail.driveTo(-48_in, -48_in, {.timeout = 6_s}).brake({.timeout = 1.5_s});
    return failure;
}

// ═══ cookbook-02b — attempt something, keep going anyway ═══════════════════════════

// A sweep that must NOT stop when one grab misses. `then()` stops the chain on
// a false return, so an attempt that is allowed to fail returns void and puts
// its outcome in a variable you own. The cost is real: the chain's transcript
// will not mention the miss, so log it yourself or nobody will.
Routine& attemptGrab(Routine& r, Intake& intake, std::vector<bool>& outcomes) {
    return r.then([&intake, &outcomes] { outcomes.push_back(intake.grab()); },
                  "attempt-grab");
}

// ═══ cookbook-03a — waiting for a partner ══════════════════════════════════════════

// "Has my partner cleared the lane?" is not a sensor shulib knows about, so the
// condition is yours. This stand-in reports clear once it has been polled
// enough times; on a real robot it reads a distance sensor, a line sensor, or a
// button your driver presses.
struct LaneSensor {
    int polls = 0;
    bool clear() { return ++polls > 30; }
};

// ═══ cookbook-03b — fitting the match window ═══════════════════════════════════════

// A Routine has per-step timeouts but NO whole-chain deadline: if an early leg
// burns six seconds, every later leg still gets its full budget. When the match
// clock is the real constraint, read it yourself between phases and keep the
// parking leg out of the chain that might stop. `secondGoalCost` is YOUR
// measurement of how long the optional goal takes — the library cannot know it.
RoutineResult budgetedAuton(Chassis& chassis, Intake& intake, Time budget,
                            Time secondGoalCost) {
    const Time started = clockNow(chassis);
    Routine r{chassis, "budgeted"};
    r.startAt(Pose2d{-48_in, -24_in, 0_deg})
        .driveTo(-24_in, -24_in, {.timeout = 8_s, .maxLinearSpeed = Velocity{8.0}})
        .then([&intake] { intake.release(); }, "score-1");

    // The optional goal runs only if there is time for it AND for the park.
    const double spent = (clockNow(chassis) - started).value();
    if (r.ok() && spent + secondGoalCost.value() < budget.value()) {
        r.driveTo(-24_in, 24_in, {.timeout = 6_s})
            .then([&intake] { intake.release(); }, "score-2");
    }

    // Parking is unconditional, so it lives outside the chain that might stop.
    const RoutineResult verdict = r.result();
    Routine park{chassis, "budgeted/park"};
    park.moveTo(Pose2d{-48_in, -48_in, 0_deg}, {.timeout = 6_s})
        .brake({.timeout = 1.5_s});
    return verdict;
}

// ═══ cookbook-04b — budgeting a sideways leg ═══════════════════════════════════════

// A drivetrain that can only strafe at a fraction of its forward speed needs a
// bigger time budget for a sideways leg. strafeAuthority() is that fraction
// (1.0 on an X-drive, 0.35 on this H-bot, 0 on a tank) and motionConfig()
// carries the speed the leg will actually run at. `safety` covers acceleration
// and settling, which this arithmetic deliberately does not model.
Time lateralBudget(Chassis& chassis, Length distance, double safety) {
    const double reach =
        chassis.motionConfig().maxLinearSpeed.value() * chassis.strafeAuthority();
    return Time{safety * std::abs(distance.value()) / reach};
}

}  // namespace

// ═══════════════════════════════════════════════════════════════════════════════════
// The cases. Every one runs its recipe against the plant and grades it on the
// simulator's GROUND TRUTH, which the robot's own estimate never sees.
// ═══════════════════════════════════════════════════════════════════════════════════

// Bug caught: the cookbook's opening claim — "this is the shape of every
// routine, and it works" — silently becoming false (a step that no longer
// settles, a park that never happens, or a skeleton that takes longer than an
// auton window). The clock bound is hand-computed: 24 in at the 60 in/s default
// is ~0.4 s of travel plus settling and the brake; anything near 6 s means a
// step is not doing what the prose says.
TEST_CASE("cookbook-01a: the skeleton — seed, move, park") {
    const auto kin = shulib::kinematics::xDrive(7_in);
    auto simCfg = motion_rig::plantConfig();
    simCfg.plant.initialPose = Pose2d{-48_in, -24_in, 90_deg};
    motion_rig::ChassisRig c{kin, simCfg};

    const double t0 = c.rig.h.clock().now().value();
    const RoutineResult res = skeleton(c.chassis);

    CHECK(res.ok);
    CHECK(res.steps == 3);
    CHECK(res.completed == 3);
    CHECK(res.skipped == 0);
    CHECK(c.chassis.scheduler().motionsStarted() == 2);  // startAt is not a motion
    CHECK(motion_rig::posErr(c.rig.h.truePose(), Pose2d{-24_in, -24_in, 90_deg}) < 1.0);
    CHECK(elapsedSince(c, t0) < 4.0);
}

// Bug caught: the two-goal recipe claiming a shape it does not have — a scoring
// sub-sequence that silently stops chaining (scoreAt returning a copy or the
// wrong Routine would break the step count), a release that never fires, or a
// run that does not fit a 15 s auton window. The window bound is the honest
// product claim the chapter makes, so it is asserted rather than asserted-about.
TEST_CASE("cookbook-01b: a two-goal side run, with a composed scoring step") {
    const auto kin = shulib::kinematics::xDrive(7_in);
    auto simCfg = motion_rig::plantConfig();
    simCfg.plant.initialPose = Pose2d{-48_in, -24_in, 0_deg};
    motion_rig::ChassisRig c{kin, simCfg};

    Intake intake;
    const double t0 = c.rig.h.clock().now().value();
    const RoutineResult res = twoGoalSideRun(c.chassis, intake);

    CHECK(res.ok);
    // 1 startAt + 2 x (driveTo, brake, then, pause) + moveTo + brake = 11 steps.
    CHECK(res.steps == 11);
    CHECK(res.completed == 11);
    CHECK(intake.releases == 2);
    CHECK(intake.grabs == 0);
    // 2 driveTo + 2 brake + 1 moveTo + 1 brake; then() and pause() are not motions.
    CHECK(c.chassis.scheduler().motionsStarted() == 6);
    CHECK(motion_rig::posErr(c.rig.h.truePose(), Pose2d{-48_in, -48_in, 0_deg}) < 1.5);
    // The whole run fits a VEX autonomous window with slack (~7.5 s measured).
    CHECK(elapsedSince(c, t0) < 12.0);
}

// Bug caught: the trajectory recipe pretending a failed sweep is recoverable
// blind. `lastTrajectory()` is the ONLY place the chain keeps how far a
// trajectory got; if it were flattened away (or overwritten by the stop) the
// documented "resume from where it broke" advice would be unfollowable, and no
// other assertion in the suite would notice.
TEST_CASE("cookbook-01c: a waypoint sweep, and what a broken sweep leaves you") {
    const auto kin = shulib::kinematics::xDrive(7_in);
    motion_rig::ChassisRig c{kin};

    Routine good{c.chassis, "sweep"};
    good.followTrajectory({Pose2d{12_in, 0_in, 0_deg}, Pose2d{24_in, 12_in, 45_deg},
                           Pose2d{24_in, 24_in, 90_deg}},
                          {.timeout = 8_s})
        .brake({.timeout = 1.5_s});
    CHECK(good.ok());
    CHECK(good.lastTrajectory().succeeded());
    CHECK(good.lastTrajectory().completedLegs == 3);

    // Starve the PER-LEG budget (options are per leg, never per chain): the
    // trajectory stops at the first leg that misses, and so does the routine.
    Routine broken{c.chassis, "sweep-starved"};
    broken.followTrajectory({Pose2d{36_in, 24_in, 0_deg}, Pose2d{-48_in, -24_in, 0_deg}},
                            {.timeout = 0.6_s});
    CHECK_FALSE(broken.ok());
    CHECK(broken.result().cause == RoutineStopCause::MotionFailed);
    CHECK(broken.result().exit == ExitReason::TimedOut);

    // How far it got is still readable — that is what a recovery plan needs.
    const TrajectoryResult partial = broken.lastTrajectory();
    CHECK(partial.completedLegs < partial.totalLegs);
    CHECK(partial.totalLegs == 2);
}

// Bug caught: the bail-out recipe not actually bailing out — the chain failing
// to stop on a false action, the skipped steps running anyway (which would
// drive the robot across the field with nothing in the jaws), the drive left
// energized after the stop, or the fallback chain silently doing nothing. Also
// pins the confusing-but-documented detail the chapter warns about: a
// non-motion stop leaves `exit` at Running.
TEST_CASE("cookbook-02a: bail out when a grab fails — and park somewhere useful") {
    const auto kin = shulib::kinematics::xDrive(7_in);
    auto simCfg = motion_rig::plantConfig();
    simCfg.plant.initialPose = Pose2d{-48_in, -24_in, 0_deg};

    SUBCASE("the grab works: the routine runs to the end") {
        motion_rig::ChassisRig c{kin, simCfg};
        Intake intake;
        const RoutineResult res = grabOrBailOut(c.chassis, intake);
        CHECK(res.ok);
        CHECK(res.steps == 7);
        CHECK(intake.grabs == 1);
        CHECK(intake.releases == 1);
        CHECK(motion_rig::posErr(c.rig.h.truePose(), Pose2d{0_in, 0_in,
                                                            c.rig.h.truePose().heading()})
              < 1.5);
    }

    SUBCASE("the grab misses: the chain stops, and the fallback parks elsewhere") {
        shulib::hal::fake::FakeTelemetrySink log;
        motion_rig::ChassisRig c{kin, simCfg, &log};
        Intake intake;
        intake.nextGrabSucceeds = false;

        const RoutineResult res = grabOrBailOut(c.chassis, intake);

        CHECK_FALSE(res.ok);
        CHECK(res.stoppedAt == 4);
        CHECK(std::string{res.stoppedName} == "grab");
        CHECK(res.cause == RoutineStopCause::ActionFailed);
        // Documented, and genuinely confusing the first time: a stop that was
        // not a motion leaves `exit` at Running ("no motion verdict here").
        CHECK(res.exit == ExitReason::Running);
        CHECK(res.skipped == 3);   // driveTo, score, brake never ran
        CHECK(intake.releases == 0);

        // The stop is loud: exactly one Warn, naming the routine and the step.
        CHECK(routineWarnings(log) == 1);
        CHECK(logged(log, "'grab-or-bail' STOPPED at step 4 (grab)"));

        // …and the fallback chain really drove somewhere else and parked.
        CHECK(motion_rig::posErr(c.rig.h.truePose(), Pose2d{-48_in, -48_in,
                                                            c.rig.h.truePose().heading()})
              < 1.5);
        for (int w = 0; w < c.rig.h.motorCount(); ++w) {
            CHECK(c.rig.h.motor(w).commandedVoltage().value() == 0.0);
        }
    }
}

// Bug caught: the "attempt and continue" idiom silently stopping the chain
// after all (which is what happens the moment someone "improves" the action to
// return the bool), or the chapter's honesty claim going stale — it tells the
// reader that a swallowed failure is INVISIBLE in the transcript, and that
// warning is only true while the routine layer really does stay silent.
TEST_CASE("cookbook-02b: attempt a grab, keep sweeping — and pay for it honestly") {
    const auto kin = shulib::kinematics::xDrive(7_in);
    shulib::hal::fake::FakeTelemetrySink log;
    motion_rig::ChassisRig c{kin, motion_rig::plantConfig(), &log};

    Intake intake;
    std::vector<bool> outcomes;

    Routine r{c.chassis, "sweep-three"};
    r.startAt(Pose2d{0_in, 0_in, 0_deg});
    r.driveTo(24_in, 0_in, {.timeout = 5_s});
    attemptGrab(r, intake, outcomes);
    intake.nextGrabSucceeds = false;  // the middle stop comes up empty
    r.driveTo(24_in, 24_in, {.timeout = 5_s});
    attemptGrab(r, intake, outcomes);
    intake.nextGrabSucceeds = true;
    r.driveTo(0_in, 24_in, {.timeout = 5_s});
    attemptGrab(r, intake, outcomes);
    r.brake({.timeout = 1.5_s});

    // The chain never stopped, even though the middle grab came up empty.
    CHECK(r.ok());
    CHECK(r.result().skipped == 0);
    CHECK(r.result().steps == 8);
    CHECK(intake.grabs == 3);
    REQUIRE(outcomes.size() == 3);
    CHECK(outcomes[0]);
    CHECK_FALSE(outcomes[1]);
    CHECK(outcomes[2]);
    CHECK(motion_rig::posErr(c.rig.h.truePose(), Pose2d{0_in, 24_in,
                                                        c.rig.h.truePose().heading()})
          < 1.5);

    // The price, asserted: the routine layer said NOTHING about the miss.
    CHECK(routineWarnings(log) == 0);
    CHECK_FALSE(logged(log, "attempt-grab"));
}

// Bug caught: a partner-wait that does not actually wait (D2's second green
// hole in miniature — outcomes stay green while the duration is wrong by a
// factor of 1000). Both bounds are checked against the SIMULATED CLOCK, and
// they are hand-computed from the recipe's own durations, not copied from them:
// 750 ms of pause must show up as at least 0.74 s and cannot plausibly exceed
// 1.0 s; the give-up-and-go wait must cost about its 400 ms timeout, not more.
TEST_CASE("cookbook-03a: waiting for an alliance partner, two ways") {
    const auto kin = shulib::kinematics::xDrive(7_in);
    motion_rig::ChassisRig c{kin};
    LaneSensor lane;

    Routine r{c.chassis, "partner"};
    r.startAt(Pose2d{-48_in, -24_in, 0_deg});

    // 1. A fixed beat: sit still, motors idle, while your partner clears out.
    const double beforePause = c.rig.h.clock().now().value();
    r.pause(750_ms);
    const double pauseCost = elapsedSince(c, beforePause);
    CHECK(pauseCost >= 0.74);
    CHECK(pauseCost < 1.0);

    // 2. A condition, with a deadline. If the deadline passes first the chain
    //    STOPS — waitFor() means "the next step assumes this happened".
    r.waitFor([&lane] { return lane.clear(); }, 2_s, "partner-clear");
    CHECK(r.ok());
    CHECK(lane.polls > 30);

    // 3. "Wait, but go anyway": that is NOT waitFor. Drop one tier inside a
    //    then() — waitUntil returns a verdict you deliberately discard.
    const double beforeGiveUp = c.rig.h.clock().now().value();
    r.then([&c] { (void)c.chassis.waitUntil([] { return false; }, 400_ms); },
           "partner-or-not");
    CHECK(r.ok());
    const double giveUpCost = elapsedSince(c, beforeGiveUp);
    CHECK(giveUpCost >= 0.39);
    CHECK(giveUpCost < 0.8);

    r.driveTo(-24_in, -24_in, {.timeout = 5_s}).brake({.timeout = 1.5_s});
    CHECK(r.ok());
}

// Bug caught: the budget recipe not actually budgeting. If the elapsed-time
// check were wrong (or read a wall clock, or compared the wrong units) the
// optional goal would run and the routine would overrun its window — which is
// exactly the failure the recipe exists to prevent, and which NO outcome
// assertion would notice. The deliberately slow first leg (8 in/s over 24 in,
// ~3 s) is what makes the branch fire; the clock bounds are hand-set from
// measured runs (5.3 s dropped / 8.2 s kept), so they bracket the difference
// the branch is supposed to make rather than merely permitting both.
TEST_CASE("cookbook-03b: fitting the match window — the optional goal gets dropped") {
    const auto kin = shulib::kinematics::xDrive(7_in);
    auto simCfg = motion_rig::plantConfig();
    simCfg.plant.initialPose = Pose2d{-48_in, -24_in, 0_deg};

    SUBCASE("a tight budget drops the optional goal, and still parks") {
        motion_rig::ChassisRig c{kin, simCfg};
        Intake intake;
        const double t0 = c.rig.h.clock().now().value();
        // 6 s of budget, and the second goal is measured at 5 s: after the slow
        // first leg there is not enough left, so it is dropped.
        const RoutineResult res = budgetedAuton(c.chassis, intake, 6_s, 5_s);

        CHECK(res.ok);
        CHECK(intake.releases == 1);  // score-2 never ran
        CHECK(res.steps == 3);        // startAt, driveTo, score-1
        CHECK(elapsedSince(c, t0) < 9.0);
        // Parked, unconditionally, on the second chain.
        CHECK(motion_rig::posErr(c.rig.h.truePose(),
                                 Pose2d{-48_in, -48_in, c.rig.h.truePose().heading()})
              < 1.5);
    }

    SUBCASE("a generous budget keeps it") {
        motion_rig::ChassisRig c{kin, simCfg};
        Intake intake;
        const double t0 = c.rig.h.clock().now().value();
        const RoutineResult res = budgetedAuton(c.chassis, intake, 30_s, 5_s);

        CHECK(res.ok);
        CHECK(intake.releases == 2);
        CHECK(res.steps == 5);
        // …and the extra goal genuinely costs time — the branch above is real.
        CHECK(elapsedSince(c, t0) > 6.0);
        CHECK(elapsedSince(c, t0) < 13.0);
    }
}

// Bug caught: the tank recipe quietly depending on sideways motion a tank
// drivetrain does not have. face() computes the bearing when the step RUNS, so
// a recipe that reordered the face and the drive — or that used strafeTo —
// would still "look right" while timing out on a real tank robot. Graded on
// ground truth, with the clock bound making a silently-slow route visible.
TEST_CASE("cookbook-04a: a tank routine — face the point, then drive to it") {
    const shulib::kinematics::TankKinematics kin{12_in};
    auto simCfg = motion_rig::plantConfig();
    simCfg.plant.initialPose = Pose2d{-24_in, -24_in, 0_deg};
    motion_rig::ChassisRig c{kin, simCfg};

    const double t0 = c.rig.h.clock().now().value();

    Routine r{c.chassis, "tank-side"};
    r.startAt(Pose2d{-24_in, -24_in, 0_deg})
        .face(-24_in, 12_in, {.timeout = 3_s})
        .driveTo(-24_in, 12_in, {.timeout = 8_s})
        .face(12_in, 12_in, {.timeout = 3_s})
        .driveTo(12_in, 12_in, {.timeout = 8_s})
        .brake({.timeout = 1.5_s});

    CHECK(r.ok());
    CHECK(r.result().steps == 6);
    CHECK(motion_rig::posErr(c.rig.h.truePose(),
                             Pose2d{12_in, 12_in, c.rig.h.truePose().heading()})
          < 1.5);
    CHECK(elapsedSince(c, t0) < 10.0);
    CHECK(c.chassis.strafeAuthority() == 0.0);  // it never had a sideways option
}

// Bug caught: a lateral leg budgeted as if the drivetrain could strafe at full
// speed. On the H-bot the sideways reach is 35% of the linear budget, so a
// timeout computed from distance/maxLinearSpeed alone is ~3x too small and the
// leg times out honestly — an authoring bug the recipe exists to prevent. This
// pins both halves: the naive budget is genuinely too small, and the
// authority-aware one is genuinely enough.
TEST_CASE("cookbook-04b: budgeting a sideways leg on a limited-strafe drivetrain") {
    const auto kin = motion_rig::hBotKinematics();
    motion_rig::ChassisRig c{kin};

    REQUIRE(c.chassis.strafeAuthority() == doctest::Approx(0.35));

    const Time budget = lateralBudget(c.chassis, 18_in, 3.0);
    // 18 in / (60 in/s * 0.35) = 0.857 s of travel; x3 for accel and settling.
    CHECK(budget.value() == doctest::Approx(2.571).epsilon(0.01));

    // The naive budget — distance / full linear speed — is not enough.
    const Time naive = Time{18.0 / c.chassis.motionConfig().maxLinearSpeed.value()};
    CHECK(naive.value() < budget.value() / 3.0);
    CHECK(c.chassis.strafeTo(0_in, 18_in, {.timeout = naive}) == ExitReason::TimedOut);

    // The authority-aware one is. (Fresh chassis state: re-aim, then strafe.)
    Routine r{c.chassis, "h-lateral"};
    const double t0 = c.rig.h.clock().now().value();
    r.turnTo(0_deg, {.timeout = 3_s}).strafeTo(0_in, 18_in, {.timeout = budget});
    CHECK(r.ok());
    CHECK(elapsedSince(c, t0) < 4.0);
    CHECK(std::abs(c.rig.h.truePose().y().value() - 18.0) < 1.5);
}

// Bug caught: the mixed-tier footgun going undocumented, and then going away.
// A bare chassis.moveTo() inside a routine is NOT a step: if it fails, ok()
// stays true and the chain drives on from the wrong place. The chapter teaches
// then() as the fix — and this case fails if either half changes, so the
// warning cannot become stale in one direction or the advice in the other.
TEST_CASE("cookbook-05a: mixing tiers — a direct call that fails is invisible") {
    const auto kin = shulib::kinematics::xDrive(7_in);
    motion_rig::ChassisRig c{kin};

    SUBCASE("the footgun: the chain never learns the direct call failed") {
        Routine r{c.chassis, "unguarded"};
        r.startAt(Pose2d{0_in, 0_in, 0_deg});
        // Not a step. 0.3 s cannot cross 60 inches.
        const ExitReason direct = c.chassis.moveTo(Pose2d{60_in, 0_in, 0_deg},
                                                   {.timeout = 0.3_s});
        CHECK(direct == ExitReason::TimedOut);
        r.brake({.timeout = 1.5_s});
        CHECK(r.ok());              // the chain is cheerful…
        CHECK(r.result().steps == 2);  // …and never counted the failed move
    }

    SUBCASE("the fix: wrap it in then(), and the verdict is honored") {
        Routine r{c.chassis, "guarded"};
        r.startAt(Pose2d{0_in, 0_in, 0_deg})
            .then([&c] {
                return c.chassis.moveTo(Pose2d{60_in, 0_in, 0_deg}, {.timeout = 0.3_s});
            }, "long-approach")
            .driveTo(60_in, 24_in, {.timeout = 5_s})
            .brake({.timeout = 1.5_s});

        CHECK_FALSE(r.ok());
        CHECK(r.result().cause == RoutineStopCause::ActionFailed);
        CHECK(r.result().exit == ExitReason::TimedOut);  // the facade's verdict, kept
        CHECK(std::string{r.result().stoppedName} == "long-approach");
        CHECK(r.result().skipped == 2);
    }
}

// Bug caught: the re-seeding recipe teaching something that does not happen.
// startAt() mid-chain must move the ESTIMATE and nothing else — if it also
// moved the robot, or if it silently no-op'd once the chain had started, a team
// squaring against a wall would be re-seeding into a lie. Ground truth is the
// control: it must not move at all.
TEST_CASE("cookbook-05b: re-seeding the estimate in the middle of a routine") {
    const auto kin = shulib::kinematics::xDrive(7_in);
    motion_rig::ChassisRig c{kin};

    Routine r{c.chassis, "relocalize"};
    r.startAt(Pose2d{0_in, 0_in, 0_deg}).driveTo(24_in, 0_in, {.timeout = 5_s});
    REQUIRE(r.ok());

    const Pose2d truthBefore = c.rig.h.truePose();

    // Pretend the robot has just squared itself against the wall at x = 36 in:
    // the measurement is better than the estimate, so overwrite the estimate.
    // (The step is called startAt even here — see the chapter's note.)
    r.startAt(Pose2d{36_in, 0_in, c.chassis.pose().heading()});

    CHECK(r.ok());
    CHECK(c.chassis.pose().x().value() == doctest::Approx(36.0));
    // The robot did not move an inch: seeding changes belief, not position.
    CHECK(motion_rig::posErr(c.rig.h.truePose(), truthBefore) == doctest::Approx(0.0));
}
