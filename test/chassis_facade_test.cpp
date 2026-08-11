// C4 CHASSIS FACADE — construction, structural id stamping, options, misuse,
// and the facade-owned safety machinery. Every case names the bug it would
// catch. The routine/accuracy/guarantee suites live in chassis_routine_test.cpp
// and the drive()/frame suites in chassis_drive_test.cpp.

#include "doctest.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <type_traits>

#include "motion_test_rig.hpp"
#include "shulib/chassis/chassis.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/kinematics/x_drive.hpp"
#include "shulib/localization/complementary_fusion.hpp"
#include "shulib/localization/localizer.hpp"
#include "shulib/localization/pilons_odometry.hpp"
#include "shulib/motion/move_to_pose.hpp"
#include "shulib/sim/scenario.hpp"

using namespace motion_rig;
using shulib::PreconditionError;
using shulib::chassis::Chassis;
using shulib::chassis::MotionOptions;
using shulib::control::ExitReason;
using shulib::hal::BrakeMode;
using shulib::hal::fake::FakeTelemetrySink;
using shulib::kinematics::TankKinematics;
using shulib::kinematics::xDrive;
using shulib::math::Angle;
using shulib::math::ChassisSpeeds;
using shulib::math::Frame;
using shulib::math::Pose2d;
using shulib::motion::MoveToPose;
using shulib::motion::WaitResult;
using shulib::units::AngularVelocity;
using shulib::units::Length;
using shulib::units::Time;
using shulib::units::Velocity;

namespace {

/// Every drive motor at exactly 0 V under Brake — the defined safe state.
void checkSafeState(MotionRig& rig) {
    for (int w = 0; w < rig.h.motorCount(); ++w) {
        CHECK(rig.h.motor(w).commandedVoltage().value() == 0.0);
        CHECK(rig.h.motor(w).brakeMode() == BrakeMode::Brake);
    }
}

}  // namespace

// ═══ Compile-time misuse pins (the negative tests ARE the assertions) ══════════════

// Bug caught: a facade signature regressing to accept dimensionless doubles or
// a frame-less drive() — the silent-misuse door §17 Tier 3 orders shut. These
// concepts are templated entities, so an invalid call renders FALSE (testable)
// rather than ill-formed.
template <typename... Args>
concept DriveCallable = requires(Chassis& c, Args... a) { c.drive(a...); };
template <typename... Args>
concept MoveToCallable = requires(Chassis& c, Args... a) { c.moveTo(a...); };
template <typename... Args>
concept StrafeToCallable = requires(Chassis& c, Args... a) { c.strafeTo(a...); };
template <typename... Args>
concept TurnToCallable = requires(Chassis& c, Args... a) { c.turnTo(a...); };
template <typename... Args>
concept FollowCallable = requires(Chassis& c, Args... a) { c.followTrajectory(a...); };

static_assert(!std::is_copy_constructible_v<Chassis>);   // owns the pinned scheduler
static_assert(!std::is_move_constructible_v<Chassis>);
// drive() without a Frame must NOT compile: the caller always says which frame.
static_assert(!DriveCallable<ChassisSpeeds>);
static_assert(DriveCallable<ChassisSpeeds, Frame>);
// Bare doubles cannot reach any verb — units are typed at the API edge.
static_assert(!MoveToCallable<double, double, double>);
static_assert(MoveToCallable<Pose2d>);
static_assert(!StrafeToCallable<double, double>);
static_assert(StrafeToCallable<Length, Length>);
static_assert(!TurnToCallable<double>);
static_assert(TurnToCallable<Angle>);
static_assert(!FollowCallable<>);  // waypoints are required

// ═══ The standalone promise: file-free construction, written out longhand ══════════

// Bug caught: any construction step that silently requires a config file,
// VexBuilder artifact, or helper the library does not ship. This test IS the
// documented plain-C++ recipe — every object a code-fluent team needs, built
// by hand, ending in a Chassis that actually drives to a pose. (The harness
// stands in for hal/pros exactly as it does for every motion test: the fakes
// implement the same F4 interfaces the robot adapters will.)
TEST_CASE("C4 standalone: a working Chassis in plain C++ — no file, no builder, no codegen") {
    namespace k = shulib::kinematics;
    namespace loc = shulib::localization;
    namespace m = shulib::motion;
    namespace ch = shulib::chassis;

    // 1. Drivetrain: value-constructed geometry — drivetrain is config DATA.
    const k::MatrixKinematics kin = k::xDrive(Length{7.0});
    // 2. "Hardware": clock/motors/imu/gps/battery/telemetry (fakes on host).
    shulib::sim::SimHarness h{kin, plantConfig()};
    // 3. Localization: odometry + fusion + the Localizer, wired by hand.
    loc::PilonsOdometry odom{h.imu(), h.makeForwardTrackingWheel(),
                             h.makeLateralTrackingWheel()};
    loc::ComplementaryFusion fusion{};
    loc::Localizer localizer{h.clock(), h.imu(), odom, fusion};
    // 4. Diagnostics: fault latch + health monitor.
    FakeTelemetrySink faultSink;
    shulib::diag::FaultLatch faults{faultSink, h.clock()};
    shulib::diag::HealthMonitor health{faults};
    // 5. One deps bundle; one pacer; ONE Chassis. Nothing else.
    const m::MotionDeps deps{.ctx = &h.context(),
                             .localizer = &localizer,
                             .kinematics = &kin,
                             .faults = &faults,
                             .health = &health};
    PlantPacer pacer{h};
    ch::Chassis chassis{deps, pacer, ch::ChassisConfig{.motion = motionConfig()}};

    // And it WORKS: seed the start pose, drive a leg, graded on ground truth.
    chassis.setPose(Pose2d{});
    const Pose2d target{Length{20.0}, Length{-8.0}, Angle::degrees(45.0)};
    CHECK(chassis.moveTo(target, {.timeout = Time{8.0}}) == ExitReason::Settled);
    CHECK(posErr(h.truePose(), target) < 1.0);
    CHECK(headErr(h.truePose(), target) < 0.03);
}

// ═══ Structural id stamping (C2's convention gap, closed) ══════════════════════════

// Bug caught: the facade constructing a motion from RAW deps instead of
// scheduler.deps() — the exact convention C2 §5 D10 could only document. If
// any facade verb misses the stamped bundle, its records carry id 0 and this
// case goes red. The raw-deps contrast at the end shows the gap is real (the
// Tier-3 path can still miss the stamp BY CHOICE), i.e. the facade's guarantee
// is doing work, not restating a default.
TEST_CASE("C4 stamping: every record of every facade verb carries its command id — "
          "structurally") {
    FakeTelemetrySink sink;
    const auto kin = xDrive(Length{7.0});
    ChassisRig c{kin, plantConfig(), &sink};

    // Verb 1 (moveTo) → id 1 on every motion record.
    REQUIRE(c.chassis.moveTo(Pose2d{Length{15.0}, Length{0.0}, Angle{}},
                             {.timeout = Time{8.0}})
            == ExitReason::Settled);
    const int n1 = sink.recordCount();
    int motionRecords1 = 0;
    for (int i = 0; i < n1; ++i) {
        const auto& r = sink.recordAt(i);
        if (r.activeCommandState != 0) {
            ++motionRecords1;
            CHECK(r.activeCommandId == 1);
        } else {
            CHECK(r.activeCommandId == 0);  // plant-truth / idle records stay unstamped
        }
    }
    CHECK(motionRecords1 > 10);

    // drive() between motions → id 0 (no scheduled motion: honest attribution).
    c.chassis.drive(ChassisSpeeds{Velocity{5.0}, Velocity{0.0}, AngularVelocity{0.0}},
                    Frame::Body);
    const int n2 = sink.recordCount();
    REQUIRE(n2 > n1);
    for (int i = n1; i < n2; ++i) {
        CHECK(sink.recordAt(i).activeCommandId == 0);
    }

    // Verb 2 (turnTo) → id 2: ids advance per motion, never bleed.
    REQUIRE(c.chassis.turnTo(Angle::degrees(90.0), {.timeout = Time{8.0}})
            == ExitReason::Settled);
    int motionRecords2 = 0;
    for (int i = n2; i < sink.recordCount(); ++i) {
        const auto& r = sink.recordAt(i);
        if (r.activeCommandState != 0) {
            ++motionRecords2;
            CHECK(r.activeCommandId == 2);
        }
    }
    CHECK(motionRecords2 > 10);

    // The CONTRAST: a Tier-3 motion built from RAW deps (not chassis.deps())
    // still runs — but its records carry id 0 even while active. This is the
    // hole the facade closes for its own verbs; through them it is unreachable.
    const int n3 = sink.recordCount();
    MoveToPose raw{c.rig.deps, Pose2d{Length{5.0}, Length{0.0}, Angle::degrees(90.0)},
                   motionConfig(), 8.0};
    c.chassis.scheduler().async(raw);
    REQUIRE(c.chassis.scheduler().waitUntilSettled() == ExitReason::Settled);
    int rawActive = 0;
    for (int i = n3; i < sink.recordCount(); ++i) {
        const auto& r = sink.recordAt(i);
        if (r.activeCommandState != 0) {
            ++rawActive;
            CHECK(r.activeCommandId == 0);  // the convention hole, demonstrated
        }
    }
    CHECK(rawActive > 10);
}

// ═══ Per-call options ══════════════════════════════════════════════════════════════

// Bug caught: options.timeoutSeconds silently ignored (every motion riding the
// 5 s config default) — the per-call budget is how routines bound their legs.
TEST_CASE("C4 options: timeoutSeconds bounds the verb — a laterally-impossible tank "
          "target exits TimedOut at the OPTION's budget, not the default") {
    const TankKinematics kin{Length{12.0}};
    ChassisRig c{kin};
    // Laterally offset target on tank: unreachable by physics (authority 0).
    const double t0 = c.rig.h.clock().now().value();
    CHECK(c.chassis.strafeTo(Length{0.0}, Length{24.0}, {.timeout = Time{0.9}})
          == ExitReason::TimedOut);
    const double elapsed = c.rig.h.clock().now().value() - t0;
    CHECK(elapsed >= 0.9);
    CHECK(elapsed < 1.4);  // the 0.9 s option, NOT the 5 s config default
    CHECK(c.rig.latch.raiseCount(shulib::diag::FaultCode::MotionTimeout) == 1);
    // A TIMEOUT stops the motors (0 V) — the Brake mode is the CANCEL safe
    // state's addition, deliberately not asserted here (distinct contracts).
    for (int w = 0; w < c.rig.h.motorCount(); ++w) {
        CHECK(c.rig.h.motor(w).commandedVoltage().value() == 0.0);
    }
}

// Bug caught: options.maxLinearSpeed ignored (the motion runs at the config
// budget) — the slow-approach leg every real auton needs would be a no-op.
// The uncapped twin proves the cap genuinely bound (no vacuity).
TEST_CASE("C4 options: maxLinearSpeed caps the leg's true ground speed") {
    struct SpeedTrackingPacer final : shulib::motion::ITickPacer {
        explicit SpeedTrackingPacer(shulib::sim::SimHarness& harness)
            : h{&harness}, prev{harness.truePose()} {}
        void pace() override {
            h->plant().step(Time{0.01});
            const Pose2d now = h->truePose();
            maxStep = std::max(maxStep, posErr(now, prev));
            prev = now;
        }
        shulib::sim::SimHarness* h;
        double maxStep = 0.0;
        Pose2d prev;
    };

    const auto kin = xDrive(Length{7.0});
    const Pose2d target{Length{30.0}, Length{0.0}, Angle{}};

    auto maxTickStep = [&](const MotionOptions& options) {
        MotionRig rig{kin};
        SpeedTrackingPacer pacer{rig.h};
        Chassis chassis{rig.deps, pacer, chassisConfig()};
        REQUIRE(chassis.moveTo(target, options) == ExitReason::Settled);
        return pacer.maxStep;
    };

    const double capped = maxTickStep({.timeout = Time{8.0}, .maxLinearSpeed = Velocity{20.0}});
    const double uncapped = maxTickStep({.timeout = Time{8.0}});
    CHECK(capped < 20.0 * 0.01 * 1.2);   // ≤ cap × dt, with transient margin
    CHECK(uncapped > 20.0 * 0.01 * 1.5); // the default budget genuinely runs faster
}

// Bug caught: the ω clamp dropped from the shared pipeline, or the
// maxAngularSpeed override ignored — found as mutation M21, which stayed
// GREEN against the original suite: desaturate() keeps big turns CONVERGENT
// without the clamp, so no accuracy test can see it; only the yaw-RATE budget
// itself can. (Without the clamp, a large heading error also steals linear
// authority through desaturation — the commanded mix changes.) This case pins
// the budget observably on ground truth, plus the option override, with an
// uncapped twin proving the pin is not vacuous.
TEST_CASE("C4 options: maxAngularSpeed caps the true yaw rate (closes mutation M21's "
          "green hole)") {
    struct YawTrackingPacer final : shulib::motion::ITickPacer {
        explicit YawTrackingPacer(shulib::sim::SimHarness& harness)
            : h{&harness}, prev{harness.truePose()} {}
        void pace() override {
            h->plant().step(Time{0.01});
            const Pose2d now = h->truePose();
            maxYawStep = std::max(maxYawStep,
                                  std::abs(prev.heading().errorTo(now.heading())));
            prev = now;
        }
        shulib::sim::SimHarness* h;
        double maxYawStep = 0.0;  // rad per 10 ms tick
        Pose2d prev;
    };

    const auto kin = xDrive(Length{7.0});
    auto maxYawStep = [&](const MotionOptions& options) {
        MotionRig rig{kin};
        YawTrackingPacer pacer{rig.h};
        Chassis chassis{rig.deps, pacer, chassisConfig()};
        REQUIRE(chassis.turnTo(Angle::degrees(170.0), options) == ExitReason::Settled);
        return pacer.maxYawStep;
    };

    const double capped = maxYawStep({.timeout = Time{8.0},
                                      .maxAngularSpeed = AngularVelocity{1.5}});
    const double uncapped = maxYawStep({.timeout = Time{8.0}});
    CHECK(capped < 1.5 * 0.01 * 1.25);    // ≤ the option's budget × dt, with margin
    CHECK(uncapped > 1.5 * 0.01 * 1.5);   // the 6 rad/s default genuinely turns faster
    CHECK(uncapped < 6.0 * 0.01 * 1.25);  // …and the CONFIG budget also binds (the clamp)
}

// Bug caught: nonsense options accepted quietly (NaN timeout riding the
// watchdog forever; negative budgets flipping clamps).
TEST_CASE("C4 options: non-finite / negative options are rejected loudly") {
    const auto kin = xDrive(Length{7.0});
    ChassisRig c{kin};
    const double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK_THROWS_AS(c.chassis.moveTo(Pose2d{}, {.timeout = Time{nan}}), PreconditionError);
    CHECK_THROWS_AS(c.chassis.moveTo(Pose2d{}, {.timeout = Time{-1.0}}), PreconditionError);
    CHECK_THROWS_AS(c.chassis.turnTo(Angle{}, {.maxLinearSpeed = Velocity{nan}}),
                    PreconditionError);
    CHECK_THROWS_AS(
        c.chassis.turnTo(Angle{}, {.maxAngularSpeed = AngularVelocity{-2.0}}),
        PreconditionError);
    // And the chassis is still usable after each rejection (no half-armed state):
    CHECK(c.chassis.moveTo(Pose2d{Length{5.0}, Length{0.0}, Angle{}},
                           {.timeout = Time{8.0}})
          == ExitReason::Settled);
}

// Bug caught: a NaN target reaching the PID (NaN volts to hardware for a full
// watchdog window) — rejected at construction, at the source (MoveToPose).
TEST_CASE("C4 misuse: non-finite targets and speeds are rejected before anything moves") {
    const auto kin = xDrive(Length{7.0});
    ChassisRig c{kin};
    const double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK_THROWS_AS(c.chassis.moveTo(Pose2d{Length{nan}, Length{0.0}, Angle{}}),
                    PreconditionError);
    CHECK_THROWS_AS(c.chassis.strafeTo(Length{0.0}, Length{nan}), PreconditionError);
    CHECK_THROWS_AS(c.chassis.drive(ChassisSpeeds{Velocity{nan}, Velocity{0.0},
                                                  AngularVelocity{0.0}},
                                    Frame::Body),
                    PreconditionError);
    CHECK_THROWS_AS(
        c.chassis.followTrajectory({Pose2d{Length{nan}, Length{0.0}, Angle{}}}),
        PreconditionError);
    CHECK_THROWS_AS(c.chassis.followTrajectory(std::span<const Pose2d>{}),
                    PreconditionError);
    // Nothing moved and nothing latched: rejection happened before motion.
    CHECK(posErr(c.rig.h.truePose(), Pose2d{}) < 1e-9);
    CHECK_FALSE(c.rig.latch.hasFault());
}

// Bug caught: a mid-chain NaN waypoint driving legs 1..k-1 and THEN throwing —
// followTrajectory's input validation must be atomic (all-or-nothing).
TEST_CASE("C4 misuse: a bad waypoint ANYWHERE in a trajectory rejects the whole call "
          "before the first leg runs") {
    const auto kin = xDrive(Length{7.0});
    ChassisRig c{kin};
    const double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK_THROWS_AS(
        c.chassis.followTrajectory({Pose2d{Length{10.0}, Length{0.0}, Angle{}},
                                    Pose2d{Length{nan}, Length{5.0}, Angle{}}}),
        PreconditionError);
    CHECK(posErr(c.rig.h.truePose(), Pose2d{}) < 1e-9);         // leg 1 never ran
    CHECK(c.chassis.scheduler().motionsStarted() == 0);
}

// ═══ waitUntil passthrough ═════════════════════════════════════════════════════════

// Bug caught: the facade's waitUntil deviating from C2's pinned semantics
// (entry check, bounded timeout, distinguishable result, no fault on timeout).
TEST_CASE("C4 waitUntil: C2's verb re-exported unchanged — entry check, bounded, "
          "distinguishable, faultless timeout") {
    const auto kin = xDrive(Length{7.0});
    ChassisRig c{kin};
    // True on entry: Satisfied without a single pace.
    const double t0 = c.rig.h.clock().now().value();
    CHECK(c.chassis.waitUntil([] { return true; }, Time{5.0}) == WaitResult::Satisfied);
    CHECK(c.rig.h.clock().now().value() == t0);
    // Never-true: TimedOut at the deadline, no fault raised.
    CHECK(c.chassis.waitUntil([] { return false; }, Time{0.3}) == WaitResult::TimedOut);
    CHECK(c.rig.h.clock().now().value() >= t0 + 0.3);
    CHECK(c.rig.h.clock().now().value() < t0 + 0.5);
    CHECK_FALSE(c.rig.latch.hasFault());
}

// ═══ The candidate verbs: brake / hold ═════════════════════════════════════════════

// Bug caught: brake() not actually wiring DriveBrake (a no-op returning
// Settled while the robot coasts on) — the from-speed rest check is the
// non-vacuous version: the robot is provably MOVING when brake() is called.
TEST_CASE("C4 brake: from speed to certified rest through the facade") {
    const auto kin = xDrive(Length{7.0});
    ChassisRig c{kin};
    // Get genuinely moving via the manual verb (~30 in/s for 0.6 s).
    for (int i = 0; i < 60; ++i) {
        c.chassis.drive(ChassisSpeeds{Velocity{30.0}, Velocity{0.0}, AngularVelocity{0.0}},
                        Frame::Body);
        c.pacer.pace();
    }
    const Pose2d moving1 = c.rig.h.truePose();
    c.pacer.pace();
    REQUIRE(posErr(c.rig.h.truePose(), moving1) > 0.15);  // provably moving NOW

    CHECK(c.chassis.brake({.timeout = Time{5.0}}) == ExitReason::Settled);
    // TRUE rest, not just an estimator claim: the pose stays put afterwards.
    const Pose2d rest = c.rig.h.truePose();
    for (int i = 0; i < 10; ++i) {
        c.pacer.pace();
    }
    CHECK(posErr(c.rig.h.truePose(), rest) < 0.2);
}

// Bug caught: hold() mapping `seconds` to the wrong knob (a watchdog instead
// of holdFor — it would exit TimedOut, or instantly).
TEST_CASE("C4 hold: holds position for the requested window and reports honestly") {
    const auto kin = xDrive(Length{7.0});
    ChassisRig c{kin};
    REQUIRE(c.chassis.moveTo(Pose2d{Length{10.0}, Length{0.0}, Angle{}},
                             {.timeout = Time{8.0}})
            == ExitReason::Settled);
    const Pose2d before = c.rig.h.truePose();
    const double t0 = c.rig.h.clock().now().value();
    CHECK(c.chassis.hold(Time{0.5}) == ExitReason::Settled);
    const double held = c.rig.h.clock().now().value() - t0;
    CHECK(held >= 0.5);
    CHECK(held < 1.0);
    CHECK(posErr(c.rig.h.truePose(), before) < 0.6);  // it held its ground
    CHECK_THROWS_AS(c.chassis.hold(Time{0.0}), PreconditionError);  // nonsense window
}

// ═══ cancel(): the panic stop through the facade ═══════════════════════════════════

// Bug caught: facade cancel() losing the scheduler's no-active-motion panic
// stop (a cancel that can be "too late" to do anything).
TEST_CASE("C4 cancel: panic stop with nothing active still safes the drive") {
    const auto kin = xDrive(Length{7.0});
    ChassisRig c{kin};
    for (int w = 0; w < c.rig.h.motorCount(); ++w) {
        REQUIRE(c.rig.h.motor(w).brakeMode() != BrakeMode::Brake);  // not vacuous
    }
    c.chassis.cancel();
    checkSafeState(c.rig);
}

// ═══ The DetachGuard: a throwing wait must not leave a dangling motion ═════════════

// Bug caught: waitUntilSettled throwing (here: a pacer that stops advancing
// the clock mid-motion) with the verb's STACK-OWNED motion still in the
// scheduler's active slot — the next verb would pre-empt-cancel a destroyed
// object (use-after-free). The guard must cancel on unwind: slot cleared,
// drive safed, chassis still usable.
TEST_CASE("C4 unwind: a mid-verb pacer failure throws loudly, detaches the motion, "
          "and safes the drive") {
    const auto kin = xDrive(Length{7.0});
    MotionRig rig{kin};
    StallingPacer pacer{rig.h, 50};  // healthy for 0.5 s, then the world freezes
    Chassis chassis{rig.deps, pacer, chassisConfig()};

    CHECK_THROWS_AS(chassis.moveTo(Pose2d{Length{40.0}, Length{0.0}, Angle{}},
                                   {.timeout = Time{8.0}}),
                    PreconditionError);
    CHECK_FALSE(chassis.scheduler().hasActiveMotion());  // detached, not dangling
    checkSafeState(rig);                                 // and safed on the way out
    CHECK(chassis.lastExitReason() == ExitReason::Cancelled);
    // Still usable: the panic stop neither throws nor touches the dead motion.
    chassis.cancel();
    checkSafeState(rig);
}
