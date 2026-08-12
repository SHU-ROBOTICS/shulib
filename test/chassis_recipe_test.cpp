// D1 RECIPE LAYER — the Tier-2 chain (shulib::chassis::Routine) over the C4
// facade. Every case names the bug it would catch.
//
// The keystone is the RECIPE TWIN: the same routine (C1's generator, seed 77,
// C2's cadence — the exact auton the C4 twin ran) driven once through Routine
// steps and once through direct facade verbs must agree TO THE BIT, clean and
// hostile. Bit-equality is the proof that the recipe layer delegates and adds
// nothing — with it pinned, C1–C4's accuracy and guarantee baselines carry
// over VERBATIM, and the per-guarantee cases here re-pin the load-bearing ones
// THROUGH the chain (error policy, ODO_STUCK, watchdog + boot, safe state).

#include "doctest.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <type_traits>
#include <vector>

#include "motion_test_rig.hpp"
#include "shulib/chassis/chassis.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/kinematics/x_drive.hpp"
#include "shulib/sim/hostile/composed.hpp"
#include "shulib/sim/hostile/encoder_hostility.hpp"
#include "shulib/sim/hostile/imu_hostility.hpp"
#include "shulib/sim/rng.hpp"

using namespace motion_rig;
using shulib::PreconditionError;
using shulib::chassis::Routine;
using shulib::chassis::RoutineResult;
using shulib::chassis::RoutineStopCause;
using shulib::control::ExitReason;
using shulib::diag::FaultCode;
using shulib::hal::BrakeMode;
using shulib::hal::LogLevel;
using shulib::hal::fake::FakeTelemetrySink;
using shulib::kinematics::TankKinematics;
using shulib::kinematics::xDrive;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::sim::EncoderHostileConfig;
using shulib::sim::EncoderHostileModel;
using shulib::sim::FullHostility;
using shulib::sim::ImuHostileConfig;
using shulib::sim::ImuHostileModel;
using shulib::sim::Rng;
using shulib::units::Length;
using shulib::units::Time;
using shulib::units::Velocity;

namespace {

constexpr double kMoveTimeoutX = 8.0;   // C2's X budget (chassis_routine_test.cpp)
constexpr double kMoveTimeoutH = 14.0;  // C3's strafe-limited budget

/// Every drive motor at exactly 0 V under Brake — the defined safe state.
void checkSafeState(MotionRig& rig) {
    for (int w = 0; w < rig.h.motorCount(); ++w) {
        CHECK(rig.h.motor(w).commandedVoltage().value() == 0.0);
        CHECK(rig.h.motor(w).brakeMode() == BrakeMode::Brake);
    }
}

/// Log-entry counters for asserting on the chain's stop/skip lines.
int countEntries(const FakeTelemetrySink& sink, int from, LogLevel level,
                 const char* subsystem) {
    int n = 0;
    for (int i = from; i < sink.size(); ++i) {
        if (sink.at(i).level == level && sink.at(i).subsystem == subsystem) {
            ++n;
        }
    }
    return n;
}

struct TwinWaypoint {
    double t = 0.0;
    double errTrue = 0.0;
    double errHead = 0.0;
};

struct TwinRun {
    std::vector<TwinWaypoint> waypoints;
    double finalErr = 0.0;
    double totalTime = 0.0;
    double totalDistance = 0.0;
    int moveCount = 0;
};

/// C1's waypoint generator, verbatim (same draws for the same seed — the twin
/// runs LITERALLY the auton C1–C4 measured).
Pose2d nextWaypoint(Rng& rng, const Pose2d& from) {
    const double dx = rng.uniform(-35.0, 35.0);
    const double dy = rng.uniform(-35.0, 35.0);
    const double x = std::clamp(from.x().value() + dx, -55.0, 55.0);
    const double y = std::clamp(from.y().value() + dy, -55.0, 55.0);
    return Pose2d{Length{x}, Length{y}, Angle::radians(rng.uniform(-Angle::kPi, Angle::kPi))};
}

/// C2's cadence (every 3rd waypoint a pure turn, every 7th a strafe), driven
/// EITHER through Routine steps OR through direct facade verbs — the two arms
/// of the RECIPE twin. Identical rig configuration either way.
TwinRun runTwinArm(const shulib::kinematics::IKinematics& kin, int n, std::uint64_t seed,
                   bool viaRecipe, bool hostile, double moveTimeout) {
    auto pcfg = plantConfig();
    pcfg.plant.seed = seed;
    FullHostility world{};
    ChassisRig c{kin, pcfg, nullptr, hostile ? &world.model() : nullptr};
    Routine r{c.chassis, "twin"};

    Rng wp{seed * 2654435761ULL};
    TwinRun out;
    Pose2d target{};
    for (int k = 0; k < n; ++k) {
        target = nextWaypoint(wp, target);
        const bool strafeLeg = (k % 7 == 3);
        if (strafeLeg) {
            if (viaRecipe) {
                r.strafeTo(target.x(), target.y(), {.timeout = Time{moveTimeout}});
                REQUIRE(r.ok());
            } else {
                REQUIRE(c.chassis.strafeTo(target.x(), target.y(),
                                           {.timeout = Time{moveTimeout}})
                        == ExitReason::Settled);
            }
            target = Pose2d{target.x(), target.y(), c.rig.h.truePose().heading()};
        } else {
            if (viaRecipe) {
                r.moveTo(target, {.timeout = Time{moveTimeout}});
                REQUIRE(r.ok());
            } else {
                REQUIRE(c.chassis.moveTo(target, {.timeout = Time{moveTimeout}})
                        == ExitReason::Settled);
            }
        }
        ++out.moveCount;

        TwinWaypoint w;
        w.t = c.rig.h.clock().now().value();
        w.errTrue = posErr(c.rig.h.truePose(), target);
        w.errHead = strafeLeg ? 0.0 : headErr(c.rig.h.truePose(), target);
        out.waypoints.push_back(w);

        if (k % 3 == 2) {
            const Angle newHeading = Angle::radians(wp.uniform(-Angle::kPi, Angle::kPi));
            if (viaRecipe) {
                r.turnTo(newHeading, {.timeout = Time{moveTimeout}});
                REQUIRE(r.ok());
            } else {
                REQUIRE(c.chassis.turnTo(newHeading, {.timeout = Time{moveTimeout}})
                        == ExitReason::Settled);
            }
            target = Pose2d{target.x(), target.y(), newHeading};
            ++out.moveCount;
        }
    }
    out.finalErr = posErr(c.rig.h.truePose(), target);
    out.totalTime = c.rig.h.clock().now().value();
    out.totalDistance = c.pacer.distance;

    // Bookkeeping must agree with physics through BOTH arms — and the chain's
    // own ledger must agree with the scheduler's.
    REQUIRE(c.chassis.scheduler().motionsStarted() == out.moveCount);
    REQUIRE(c.chassis.scheduler().motionsSettled() == out.moveCount);
    if (viaRecipe) {
        const RoutineResult res = r.result();
        REQUIRE(res.ok);
        REQUIRE(res.steps == out.moveCount);
        REQUIRE(res.completed == out.moveCount);
        REQUIRE(res.skipped == 0);
    }
    return out;
}

}  // namespace

// ═══ Compile-time misuse pins (the negative tests ARE the assertions) ══════════════

// Bug caught: a recipe step regressing to accept dimensionless doubles — the
// silent-unit-misuse door §17 orders shut must stay shut one tier up too.
template <typename... Args>
concept RoutineMoveToCallable = requires(Routine& r, Args... a) { r.moveTo(a...); };
template <typename... Args>
concept RoutineStrafeToCallable = requires(Routine& r, Args... a) { r.strafeTo(a...); };
template <typename... Args>
concept RoutineTurnToCallable = requires(Routine& r, Args... a) { r.turnTo(a...); };
template <typename... Args>
concept RoutineFaceCallable = requires(Routine& r, Args... a) { r.face(a...); };
template <typename... Args>
concept RoutineDriveToCallable = requires(Routine& r, Args... a) { r.driveTo(a...); };

static_assert(!RoutineMoveToCallable<double, double, double>);
static_assert(RoutineMoveToCallable<Pose2d>);
static_assert(!RoutineStrafeToCallable<double, double>);
static_assert(RoutineStrafeToCallable<Length, Length>);
static_assert(!RoutineTurnToCallable<double>);
static_assert(RoutineTurnToCallable<Angle>);
static_assert(!RoutineFaceCallable<double, double>);
static_assert(RoutineFaceCallable<Length, Length>);
static_assert(!RoutineDriveToCallable<double, double>);
static_assert(RoutineDriveToCallable<Length, Length>);
// The D2 time retype, one tier up: hold(0.3) / pause(0.5) as bare numbers must
// not compile — a duration is typed (300_ms) at every tier, or the misuse door
// the facade shut would stand open here.
template <typename... Args>
concept RoutineHoldCallable = requires(Routine& r, Args... a) { r.hold(a...); };
template <typename... Args>
concept RoutinePauseCallable = requires(Routine& r, Args... a) { r.pause(a...); };
static_assert(!RoutineHoldCallable<double>);
static_assert(RoutineHoldCallable<shulib::units::Time>);
static_assert(!RoutinePauseCallable<double>);
static_assert(!RoutinePauseCallable<int>);
static_assert(RoutinePauseCallable<shulib::units::Time>);
// Two Routine handles over one chain would race the stop state — forbidden.
static_assert(!std::is_copy_constructible_v<Routine>);
static_assert(!std::is_move_constructible_v<Routine>);

// ═══ The recipe twin (the keystone) ════════════════════════════════════════════════

// Bug caught: ANY deviation of the chain's delegation from the facade path the
// C1–C4 baselines were measured on — an option reshaped, an extra localizer
// read, a swapped argument, a step reordered, hidden motion logic. Bit-equality
// (clean AND hostile) means the recipe layer demonstrably added API, not
// physics, and every measured baseline carries over VERBATIM.
TEST_CASE("D1 twin: Routine steps == direct facade verbs, bit for bit (clean and "
          "hostile)") {
    const auto kin = xDrive(Length{7.0});
    for (const bool hostile : {false, true}) {
        CAPTURE(hostile);
        const TwinRun recipe = runTwinArm(kin, 5, 77, true, hostile, kMoveTimeoutX);
        const TwinRun facade = runTwinArm(kin, 5, 77, false, hostile, kMoveTimeoutX);
        REQUIRE(recipe.moveCount == facade.moveCount);
        REQUIRE(recipe.waypoints.size() == facade.waypoints.size());
        CHECK(recipe.totalTime == facade.totalTime);
        CHECK(recipe.totalDistance == facade.totalDistance);
        CHECK(recipe.finalErr == facade.finalErr);
        for (std::size_t i = 0; i < recipe.waypoints.size(); ++i) {
            CHECK(recipe.waypoints[i].t == facade.waypoints[i].t);
            CHECK(recipe.waypoints[i].errTrue == facade.waypoints[i].errTrue);
            CHECK(recipe.waypoints[i].errHead == facade.waypoints[i].errHead);
        }
        // The hostile arm also re-pins C2's bound THROUGH the chain.
        if (hostile) {
            CHECK(recipe.finalErr < 5.0);
        }
        MESSAGE("recipe twin (hostile=", hostile, "): finalErr=", recipe.finalErr,
                "in time=", recipe.totalTime, "s over ", recipe.moveCount, " motions");
    }
}

// Bug caught: recipe-added per-move error compounding (chain state leaking
// between steps). With clean sensors there is no drift clock, so ANY growth
// with routine length through the chain is chain-added. Bounds are C4's,
// unchanged; X swept at every length, H spot-checked (the twin already proves
// the H path identical to its measured facade baseline).
TEST_CASE("D1 accuracy: CLEAN sweep 5/10/20/40 through Routine steps — error FLAT in "
          "move count") {
    const auto xKin = xDrive(Length{7.0});
    for (const int n : {5, 10, 20, 40}) {
        CAPTURE(n);
        const TwinRun x = runTwinArm(xKin, n, 77, true, false, kMoveTimeoutX);
        MESSAGE("recipe clean n=", n, "  X: finalErr=", x.finalErr,
                "in t=", x.totalTime, "s over ", x.moveCount, " motions");
        CHECK(x.finalErr < 0.8);
    }
    const auto hKin = hBotKinematics();
    const TwinRun h = runTwinArm(hKin, 10, 77, true, false, kMoveTimeoutH);
    CHECK(h.finalErr < 0.8);
}

// ═══ The complete ~10-line routine, all three drivetrains (the DoD item) ═══════════

// Bug caught: the Tier-2 vocabulary failing to express a real auton on any
// drivetrain — X and H use the holonomic verbs; tank uses face()+driveTo()
// (the author-plans-the-turn idiom in field words). Graded on ground truth.
TEST_CASE("D1 routine: a complete recipe auton lands on X, H, and tank") {
    const auto xKin = xDrive(Length{7.0});
    const auto hKin = hBotKinematics();
    const TankKinematics tankKin{Length{12.0}};

    // ── X and H: the same 8-step recipe, holonomic vocabulary ─────────────────────
    for (const bool useH : {false, true}) {
        CAPTURE(useH);
        auto pcfg = plantConfig();
        pcfg.plant.initialPose = Pose2d{Length{-48.0}, Length{-24.0}, Angle::degrees(90.0)};
        ChassisRig c{useH ? static_cast<const shulib::kinematics::IKinematics&>(hKin)
                          : xKin,
                     pcfg};
        const double strafeBudget = useH ? kMoveTimeoutH : kMoveTimeoutX;
        bool actionRan = false;

        Routine r{c.chassis, useH ? "h-auton" : "x-auton"};
        r.startAt(Pose2d{Length{-48.0}, Length{-24.0}, Angle::degrees(90.0)})
            .moveTo(Pose2d{Length{-24.0}, Length{0.0}, Angle::degrees(45.0)},
                    {.timeout = Time{kMoveTimeoutX}})
            .then([&] { actionRan = true; }, "score")  // mechanism seam placeholder
            .strafeTo(Length{-24.0}, Length{20.0}, {.timeout = Time{strafeBudget}})
            .face(Length{0.0}, Length{44.0})
            .driveTo(Length{0.0}, Length{44.0}, {.timeout = Time{kMoveTimeoutX}})
            .hold(Time{0.3})
            .brake();

        REQUIRE(r.ok());
        const RoutineResult res = r.result();
        CHECK(res.steps == 8);
        CHECK(res.completed == 8);
        CHECK(res.skipped == 0);
        CHECK(res.cause == RoutineStopCause::None);
        CHECK(actionRan);
        // 6 motions (startAt and then() are not motions); all settled.
        CHECK(c.chassis.scheduler().motionsStarted() == 6);
        CHECK(c.chassis.scheduler().motionsSettled() == 6);
        // Genuinely at the last target, on truth the estimator cannot see.
        CHECK(posErr(c.rig.h.truePose(), Pose2d{Length{0.0}, Length{44.0}, Angle{}})
              < 1.0);
        CHECK_FALSE(c.rig.latch.hasFault());
    }

    // ── tank: face + driveTo, twice — the drivetrain-honest recipe ────────────────
    {
        auto pcfg = plantConfig();
        pcfg.plant.initialPose = Pose2d{Length{-48.0}, Length{-24.0}, Angle::degrees(0.0)};
        ChassisRig c{tankKin, pcfg};
        bool actionRan = false;

        Routine r{c.chassis, "tank-auton"};
        r.startAt(Pose2d{Length{-48.0}, Length{-24.0}, Angle::degrees(0.0)})
            .face(Length{-12.0}, Length{0.0}, {.timeout = Time{kMoveTimeoutX}})
            .driveTo(Length{-12.0}, Length{0.0}, {.timeout = Time{kMoveTimeoutX}})
            .then([&] { actionRan = true; }, "score")
            .face(Length{24.0}, Length{24.0}, {.timeout = Time{kMoveTimeoutX}})
            .driveTo(Length{24.0}, Length{24.0}, {.timeout = Time{kMoveTimeoutX}})
            .hold(Time{0.3})
            .brake();

        REQUIRE(r.ok());
        CHECK(r.result().steps == 8);
        CHECK(r.result().skipped == 0);
        CHECK(actionRan);
        CHECK(c.chassis.scheduler().motionsStarted() == 6);
        const Pose2d goal{Length{24.0}, Length{24.0},
                          c.rig.h.truePose().heading()};  // heading = arrival bearing
        CHECK(posErr(c.rig.h.truePose(), goal) < 1.0);
        CHECK_FALSE(c.rig.latch.hasFault());
    }
}

// Bug caught: face()/driveTo() drifting from the documented hand idiom they
// abbreviate (hidden motion logic, a swapped atan2 argument, a heading not
// equal to the bearing). The hand arm is written FRESH here from the guide's
// tankGoTo idiom — an independent oracle for the sugar — and both arms must
// agree to the bit AND land at the field target on truth.
TEST_CASE("D1 sugar: face+driveTo == hand-written turnTo(atan2)+moveTo, bit for bit") {
    const TankKinematics kin{Length{12.0}};
    const Length tx{18.0};
    const Length ty{30.0};

    auto runSugar = [&] {
        ChassisRig c{kin};
        Routine r{c.chassis, "sugar"};
        r.face(tx, ty, {.timeout = Time{8.0}}).driveTo(tx, ty, {.timeout = Time{8.0}});
        REQUIRE(r.ok());
        return std::pair{c.rig.h.truePose(), c.rig.h.clock().now().value()};
    };
    auto runHand = [&] {
        ChassisRig c{kin};
        // The guide's tank idiom, by hand: bearing from the ESTIMATE, turn, then
        // move with the bearing as the target heading.
        const Pose2d here = c.chassis.pose();
        const Angle bearing = Angle::radians(
            std::atan2((ty - here.y()).value(), (tx - here.x()).value()));
        REQUIRE(c.chassis.turnTo(bearing, {.timeout = Time{8.0}}) == ExitReason::Settled);
        const Pose2d mid = c.chassis.pose();
        const Angle bearing2 = Angle::radians(
            std::atan2((ty - mid.y()).value(), (tx - mid.x()).value()));
        REQUIRE(c.chassis.moveTo(Pose2d{tx, ty, bearing2}, {.timeout = Time{8.0}})
                == ExitReason::Settled);
        return std::pair{c.rig.h.truePose(), c.rig.h.clock().now().value()};
    };

    const auto [sugarPose, sugarT] = runSugar();
    const auto [handPose, handT] = runHand();
    CHECK(sugarPose.approxEqual(handPose, Length{0.0}, 0.0));  // bit-identical truth
    CHECK(sugarT == handT);
    // And the ground-truth anchor (catches BOTH arms sharing one sign error):
    CHECK(posErr(sugarPose, Pose2d{tx, ty, sugarPose.heading()}) < 1.0);
    CHECK(sugarPose.x().value() > 10.0);  // it went +x…
    CHECK(sugarPose.y().value() > 20.0);  // …and +y, the direction the field says
}

// ═══ The error policy (design constraint 4: errors must not vanish) ════════════════

// Bug caught: the chain continuing after a failed step (driving on from a
// position it is not at), a skipped step executing, the drive left un-safed,
// the failure invisible in the result or the transcript. This is THE policy
// case: stop + safe + skip + report, all four observable.
TEST_CASE("D1 policy: a timed-out step stops the chain — skips the rest, safes the "
          "drive, reports honestly") {
    FakeTelemetrySink sink;
    const auto kin = xDrive(Length{7.0});
    ChassisRig c{kin, plantConfig(), &sink};
    bool lateActionRan = false;

    Routine r{c.chassis, "starved"};
    r.startAt(Pose2d{})
        .moveTo(Pose2d{Length{12.0}, Length{0.0}, Angle{}}, {.timeout = Time{8.0}})
        .moveTo(Pose2d{Length{60.0}, Length{40.0}, Angle{}},
                {.timeout = Time{0.5}})  // starved: cannot cover 55+ in in 0.5 s
        .then([&] { lateActionRan = true; }, "late-action")
        .driveTo(Length{-40.0}, Length{-40.0}, {.timeout = Time{8.0}})
        .brake();

    // The verdict names the step, the cause, and the motion's honest exit.
    CHECK_FALSE(r.ok());
    const RoutineResult res = r.result();
    CHECK(res.steps == 6);
    CHECK(res.completed == 2);
    CHECK(res.stoppedAt == 3);
    CHECK(std::string{res.stoppedName} == "moveTo");
    CHECK(res.cause == RoutineStopCause::MotionFailed);
    CHECK(res.exit == ExitReason::TimedOut);
    CHECK(res.skipped == 3);

    // The skipped steps did not run: the action flag is untouched and the
    // robot is nowhere near the skipped driveTo target.
    CHECK_FALSE(lateActionRan);
    CHECK(posErr(c.rig.h.truePose(), Pose2d{Length{-40.0}, Length{-40.0}, Angle{}})
          > 30.0);
    CHECK(c.chassis.scheduler().motionsStarted() == 2);  // legs 1–2 only

    // The drive was safed by the stop (0 V + Brake — recordStop's cancel).
    checkSafeState(c.rig);

    // The transcript carries the stop and the skips.
    CHECK(countEntries(sink, 0, LogLevel::Warn, "RTN") == 1);
    CHECK(countEntries(sink, 0, LogLevel::Info, "RTN") == 3);
    bool sawStopLine = false;
    for (int i = 0; i < sink.size(); ++i) {
        if (sink.at(i).message.find("'starved' STOPPED at step 3 (moveTo)")
            != std::string::npos) {
            sawStopLine = true;
        }
    }
    CHECK(sawStopLine);
    // And the layer below latched the timeout as usual (nothing masked).
    CHECK(c.rig.latch.raiseCount(FaultCode::MotionTimeout) == 1);
}

// Bug caught: a then()-action's verdict ignored (a failed mechanism action
// letting the routine drive on as if it had scored), or the action running
// BEFORE the preceding motion finished (the seam must be strictly sequential).
TEST_CASE("D1 policy: a failing then()-action stops the chain; actions run in order") {
    const auto kin = xDrive(Length{7.0});
    ChassisRig c{kin};
    const Pose2d a{Length{15.0}, Length{0.0}, Angle{}};
    double actionSawErr = -1.0;
    bool afterRan = false;

    Routine r{c.chassis, "action"};
    r.moveTo(a, {.timeout = Time{8.0}})
        .then(
            [&] {
                // Runs AFTER the move settles: the robot is already at `a`.
                actionSawErr = posErr(c.rig.h.truePose(), a);
                return false;  // the mechanism reports failure
            },
            "grab")
        .then([&] { afterRan = true; }, "after");

    CHECK_FALSE(r.ok());
    CHECK(r.result().stoppedAt == 2);
    CHECK(std::string{r.result().stoppedName} == "grab");
    CHECK(r.result().cause == RoutineStopCause::ActionFailed);
    CHECK(r.result().exit == ExitReason::Running);  // not a motion verdict
    CHECK_FALSE(afterRan);
    CHECK(actionSawErr >= 0.0);
    CHECK(actionSawErr < 1.0);  // sequencing: the action saw the ARRIVED pose
    checkSafeState(c.rig);
}

// Bug caught: an ExitReason-returning action (mixed-tier glue wrapping a
// facade verb) having its verdict dropped — Settled must continue the chain,
// non-Settled must stop it with the reason preserved.
TEST_CASE("D1 policy: an action returning ExitReason has its verdict honored") {
    const auto kin = xDrive(Length{7.0});
    ChassisRig c{kin};

    Routine ok{c.chassis, "act-ok"};
    ok.then([&] { return c.chassis.turnTo(Angle::degrees(90.0),
                                          {.timeout = Time{8.0}}); },
            "turn-glue")
        .hold(Time{0.2});
    CHECK(ok.ok());
    CHECK(ok.result().completed == 2);

    Routine bad{c.chassis, "act-bad"};
    bad.then([&] { return c.chassis.strafeTo(Length{0.0}, Length{40.0},
                                             {.timeout = Time{0.4}}); },
             "doomed-glue")
        .hold(Time{0.2});
    CHECK_FALSE(bad.ok());
    CHECK(bad.result().cause == RoutineStopCause::ActionFailed);
    CHECK(bad.result().exit == ExitReason::TimedOut);  // the verb's reason, kept
    CHECK(bad.result().skipped == 1);
}

// Bug caught: waitFor() treating a timed-out condition as satisfied — the
// routine would act on a field state that never arrived. A timed-out waitFor
// stops the chain (no fault: the layer below is right that a timed-out wait
// is not a pathology — but a RECIPE's later steps assumed the condition).
TEST_CASE("D1 policy: waitFor timeout stops the chain; satisfied-on-entry is free") {
    const auto kin = xDrive(Length{7.0});
    ChassisRig c{kin};

    // Satisfied on entry: no time passes, chain continues.
    Routine r{c.chassis, "waits"};
    const double t0 = c.rig.h.clock().now().value();
    r.waitFor([] { return true; }, Time{5.0}, "already-true");
    CHECK(r.ok());
    CHECK(c.rig.h.clock().now().value() == t0);

    // Never-true: bounded, then the chain stops. The wait itself raises no
    // fault (C2's contract, unchanged through two layers).
    r.waitFor([] { return false; }, Time{0.4}, "ball-seen").hold(Time{0.2});
    CHECK_FALSE(r.ok());
    CHECK(c.rig.h.clock().now().value() >= t0 + 0.4);
    CHECK(c.rig.h.clock().now().value() < t0 + 0.9);
    CHECK(r.result().stoppedAt == 2);
    CHECK(std::string{r.result().stoppedName} == "ball-seen");
    CHECK(r.result().cause == RoutineStopCause::WaitTimedOut);
    CHECK(r.result().exit == ExitReason::Running);
    CHECK(r.result().skipped == 1);
    CHECK_FALSE(c.rig.latch.hasFault());
    checkSafeState(c.rig);
}

// Bug caught: pause() implemented as a bare timed-out wait (it would Warn on
// every deliberate pause and read as a failure), mapping seconds to the wrong
// knob, or moving the robot. A pause is a SUCCESS that takes time.
TEST_CASE("D1 pause: waits the requested time, stays put, stays quiet, continues") {
    FakeTelemetrySink sink;
    const auto kin = xDrive(Length{7.0});
    ChassisRig c{kin, plantConfig(), &sink};

    Routine r{c.chassis, "pausey"};
    r.moveTo(Pose2d{Length{10.0}, Length{0.0}, Angle{}}, {.timeout = Time{8.0}});
    REQUIRE(r.ok());
    const Pose2d before = c.rig.h.truePose();
    const double t0 = c.rig.h.clock().now().value();
    const int logsBefore = sink.size();

    r.pause(Time{0.8});

    const double elapsed = c.rig.h.clock().now().value() - t0;
    CHECK(r.ok());                       // a pause is not a failure
    CHECK(r.result().completed == 2);
    CHECK(elapsed >= 0.8);               // it genuinely waited…
    CHECK(elapsed < 1.3);                // …and not the backstop's worth
    CHECK(posErr(c.rig.h.truePose(), before) < 0.2);  // motors idle: no travel
    // The transcript stayed clean: no Warn from ANY subsystem for a pause.
    for (int i = logsBefore; i < sink.size(); ++i) {
        CHECK(sink.at(i).level != LogLevel::Warn);
    }
    // And the chain continues normally afterwards.
    r.turnTo(Angle::degrees(90.0), {.timeout = Time{8.0}});
    CHECK(r.ok());
    CHECK(r.result().completed == 3);
}

// ═══ Guarantees through the recipe layer ═══════════════════════════════════════════

// Bug caught: the ODO_STUCK abort policy lost behind the chain — the one
// fault that must stop a motion has to surface as a MotionFailed/Cancelled
// stop with the causal fault still named at the facade, promptly, drive safed,
// remaining steps skipped.
TEST_CASE("D1 guarantee: ODO_STUCK aborts the step and stops the chain — cause named, "
          "damage bounded") {
    EncoderHostileConfig enc;
    enc.trackingFreezeAt = Time{1.0};
    enc.trackingFreezeIndex = -1;  // both tracking channels die mid-motion
    EncoderHostileModel model{enc};
    const auto kin = xDrive(Length{7.0});
    ChassisRig c{kin, plantConfig(), nullptr, &model};
    bool afterRan = false;

    Routine r{c.chassis, "stuck"};
    r.moveTo(Pose2d{Length{40.0}, Length{0.0}, Angle{}}, {.timeout = Time{6.0}})
        .then([&] { afterRan = true; }, "after")
        .brake();

    CHECK_FALSE(r.ok());
    CHECK(r.result().stoppedAt == 1);
    CHECK(r.result().cause == RoutineStopCause::MotionFailed);
    CHECK(r.result().exit == ExitReason::Cancelled);
    CHECK(r.result().skipped == 2);
    CHECK_FALSE(afterRan);
    CHECK(c.chassis.lastCompleted().abortFault == FaultCode::OdoStuck);
    CHECK(c.chassis.scheduler().motionsAborted() == 1);
    CHECK(c.rig.h.clock().now().value() < 2.0);  // prompt: the policy, not the watchdog
    checkSafeState(c.rig);
    // Truth-vs-estimate damage stayed bounded (the abort did its job):
    CHECK(posErr(c.rig.h.truePose(), c.rig.loc.pose()) < 8.0);
}

// Bug caught: the watchdog/boot contract lost behind the chain — a recipe
// started during a never-live boot must stop at its FIRST step, bounded in
// time, having never moved, with every later step skipped (not hung, not
// driven blind).
TEST_CASE("D1 guarantee: never-live boot times out the first step, bounded and "
          "motionless; the chain stops there") {
    ImuHostileConfig cfg;
    cfg.calibrationEnd = Time{1e18};  // the IMU never comes up
    ImuHostileModel imu{cfg};
    const auto kin = xDrive(Length{7.0});
    ChassisRig c{kin, plantConfig(), nullptr, &imu};

    Routine r{c.chassis, "boot"};
    const double t0 = c.rig.h.clock().now().value();
    r.moveTo(Pose2d{Length{24.0}, Length{0.0}, Angle{}}, {.timeout = Time{1.5}})
        .turnTo(Angle::degrees(90.0), {.timeout = Time{8.0}});

    CHECK_FALSE(r.ok());
    CHECK(r.result().stoppedAt == 1);
    CHECK(r.result().exit == ExitReason::TimedOut);
    CHECK(r.result().skipped == 1);
    CHECK(c.rig.h.clock().now().value() - t0 >= 1.5);
    CHECK(c.rig.h.clock().now().value() - t0 < 2.0);     // bounded, not hung
    CHECK(posErr(c.rig.h.truePose(), Pose2d{}) < 1e-6);  // never moved
}

// ═══ Tier interop (no cliff) ═══════════════════════════════════════════════════════

// Bug caught: chain state corrupted by direct facade calls between steps, or
// eager execution failing to keep program order = field order — the mixed
// routine (recipe steps + Tier-3 verbs + a pose branch) is the §17 no-cliff
// requirement as a test.
TEST_CASE("D1 interop: recipe steps and direct facade calls interleave in one routine") {
    const auto kin = xDrive(Length{7.0});
    ChassisRig c{kin};

    Routine r{c.chassis, "mixed"};
    r.startAt(Pose2d{}).moveTo(Pose2d{Length{18.0}, Length{0.0}, Angle{}},
                               {.timeout = Time{8.0}});
    REQUIRE(r.ok());

    // Drop a tier mid-routine: a direct verb plus a strategy branch on pose.
    REQUIRE(c.chassis.turnTo(Angle::degrees(90.0), {.timeout = Time{8.0}})
            == ExitReason::Settled);
    const Pose2d mid = c.chassis.pose();
    CHECK(mid.x().value() > 15.0);  // the chain's motion really happened first

    // …and climb back up: the SAME chain object continues, unconfused.
    r.driveTo(Length{18.0}, Length{24.0}, {.timeout = Time{8.0}}).brake();
    CHECK(r.ok());
    const RoutineResult res = r.result();
    CHECK(res.steps == 4);       // the chain counted ONLY its own steps
    CHECK(res.completed == 4);
    // The scheduler counted everything: the chain's 3 motion steps (startAt
    // is not a motion) + the 1 direct verb.
    CHECK(c.chassis.scheduler().motionsStarted() == 4);
    CHECK(posErr(c.rig.h.truePose(), Pose2d{Length{18.0}, Length{24.0}, Angle{}}) < 1.0);
}

// Bug caught: followTrajectory's leg accounting flattened away by the chain —
// completedLegs is strategy-relevant (WHERE the chain broke) and must survive
// the step, in success and in failure.
TEST_CASE("D1 trajectory: the step preserves the full TrajectoryResult") {
    const auto kin = xDrive(Length{7.0});

    {
        ChassisRig c{kin};
        Routine r{c.chassis, "traj-ok"};
        r.followTrajectory({Pose2d{Length{12.0}, Length{0.0}, Angle{}},
                            Pose2d{Length{24.0}, Length{12.0}, Angle::degrees(45.0)},
                            Pose2d{Length{24.0}, Length{24.0}, Angle::degrees(90.0)}},
                           {.timeout = Time{8.0}});
        REQUIRE(r.ok());
        CHECK(r.lastTrajectory().succeeded());
        CHECK(r.lastTrajectory().completedLegs == 3);
        CHECK(r.lastTrajectory().totalLegs == 3);
    }
    {
        ChassisRig c{kin};
        Routine r{c.chassis, "traj-broke"};
        r.followTrajectory({Pose2d{Length{36.0}, Length{24.0}, Angle{}},
                            Pose2d{Length{-48.0}, Length{-24.0}, Angle{}}},
                           {.timeout = Time{0.6}})  // starved per-leg budget
            .hold(Time{0.2});
        CHECK_FALSE(r.ok());
        CHECK(r.result().cause == RoutineStopCause::MotionFailed);
        CHECK(r.result().exit == ExitReason::TimedOut);
        CHECK(r.result().skipped == 1);
        CHECK_FALSE(r.lastTrajectory().succeeded());
        CHECK(r.lastTrajectory().completedLegs < r.lastTrajectory().totalLegs);
        checkSafeState(c.rig);
    }
}

// Bug caught: startAt() silently not seeding the estimate. EVERY shared rig
// auto-seeds the estimate to the plant's pose, so this hole would be GREEN in
// every other case in this file — here the stack is wired BY HAND with the
// plant off-origin and the estimate deliberately unseeded, so startAt is the
// only thing standing between the routine and a 30-inch systematic miss.
// (Found by pre-analysis during the D1 mutation campaign, before running it.)
TEST_CASE("D1 startAt: seeds the estimate — the rigs' auto-seed must not hide it") {
    namespace k = shulib::kinematics;
    namespace loc = shulib::localization;
    namespace m = shulib::motion;
    namespace ch = shulib::chassis;

    const k::MatrixKinematics kin = k::xDrive(Length{7.0});
    auto pcfg = plantConfig();
    pcfg.plant.initialPose = Pose2d{Length{-30.0}, Length{10.0}, Angle{}};  // off-origin
    shulib::sim::SimHarness h{kin, pcfg};
    loc::PilonsOdometry odom{h.imu(), h.makeForwardTrackingWheel(),
                             h.makeLateralTrackingWheel()};
    loc::ComplementaryFusion fusion{};
    loc::Localizer localizer{h.clock(), h.imu(), odom, fusion};
    FakeTelemetrySink faultSink;
    shulib::diag::FaultLatch faults{faultSink, h.clock()};
    shulib::diag::HealthMonitor health{faults};
    const m::MotionDeps deps{.ctx = &h.context(),
                             .localizer = &localizer,
                             .kinematics = &kin,
                             .faults = &faults,
                             .health = &health};
    PlantPacer pacer{h};
    ch::Chassis chassis{deps, pacer, chassisConfig()};
    // NOTE: no setPose anywhere above — the estimate still believes origin.

    const Pose2d target{Length{0.0}, Length{20.0}, Angle::degrees(45.0)};
    Routine r{chassis, "seeded"};
    r.startAt(Pose2d{Length{-30.0}, Length{10.0}, Angle{}})  // load-bearing here
        .moveTo(target, {.timeout = Time{8.0}});
    REQUIRE(r.ok());
    // Without the seed the run lands ~31 in away (the estimate's origin lie
    // becomes a truth offset); with it, on target.
    CHECK(posErr(h.truePose(), target) < 1.0);
}

// Bug caught: the chain forwarding default options instead of the caller's —
// for the SPEED fields specifically. The twin only varies timeouts, and a
// capped leg that loses its cap still settles (faster), so nothing else in
// this file would notice. Time is the observable: the capped leg must take
// genuinely longer than its uncapped twin. (Found by pre-analysis during the
// D1 mutation campaign, before running it.)
TEST_CASE("D1 options: the chain forwards the speed budgets, not just the timeout") {
    const auto kin = xDrive(Length{7.0});
    const Pose2d target{Length{30.0}, Length{0.0}, Angle{}};

    auto legTime = [&](const shulib::chassis::MotionOptions& options) {
        ChassisRig c{kin};
        Routine r{c.chassis, "speed"};
        r.moveTo(target, options);
        REQUIRE(r.ok());
        return c.rig.h.clock().now().value();
    };

    const double uncapped = legTime({.timeout = Time{8.0}});
    const double capped = legTime({.timeout = Time{8.0},
                                   .maxLinearSpeed = Velocity{15.0}});
    CHECK(capped > uncapped * 1.5);  // 30 in at ≤15 in/s vs the 60 in/s default
}

// ═══ Runtime misuse ════════════════════════════════════════════════════════════════

// Bug caught: nonsense accepted quietly, or a precondition throw corrupting
// the chain (a bad call must leave the counters untouched — a programming
// error is not a strategy outcome — and the chain must stay usable).
TEST_CASE("D1 misuse: nonsense throws out of the step; the chain is untouched and "
          "usable") {
    const auto kin = xDrive(Length{7.0});
    ChassisRig c{kin};
    const double nan = std::numeric_limits<double>::quiet_NaN();

    Routine r{c.chassis, "misuse"};
    r.moveTo(Pose2d{Length{8.0}, Length{0.0}, Angle{}}, {.timeout = Time{8.0}});
    REQUIRE(r.ok());
    const RoutineResult before = r.result();

    // The facade's rejections pass through undamped…
    CHECK_THROWS_AS(r.moveTo(Pose2d{Length{nan}, Length{0.0}, Angle{}}), PreconditionError);
    CHECK_THROWS_AS(r.strafeTo(Length{0.0}, Length{nan}), PreconditionError);
    CHECK_THROWS_AS(r.hold(Time{0.0}), PreconditionError);
    CHECK_THROWS_AS(r.moveTo(Pose2d{}, {.timeout = Time{-1.0}}), PreconditionError);
    // …and the layer's own: pause of nonsense, bearing to the point we're on.
    CHECK_THROWS_AS(r.pause(Time{nan}), PreconditionError);
    const Pose2d here = c.chassis.pose();
    CHECK_THROWS_AS(r.face(here.x(), here.y()), PreconditionError);

    // None of that counted as a step, stopped the chain, or moved the robot.
    const RoutineResult after = r.result();
    CHECK(after.ok);
    CHECK(after.steps == before.steps);
    CHECK(after.completed == before.completed);
    CHECK(c.chassis.scheduler().motionsStarted() == 1);

    // Still fully usable.
    r.turnTo(Angle::degrees(45.0), {.timeout = Time{8.0}});
    CHECK(r.ok());
    CHECK(r.result().completed == before.completed + 1);
}
