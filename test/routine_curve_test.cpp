// CURVE1-A — the curve verbs through the FACADE and the ROUTINE, and the auton's run guard: the
// Chassis's curveTo / reverseCurveTo (F6-additive) and the Routine steps of the same names
// (F10-additive, the block "CURVE1: curves"). The motion itself is motion_curve_test.cpp; the pure
// path curve_path_test.cpp. Host test plant only — nothing here has run on a robot.
//
//   CV7   a DEADLINE mid-curve: the cut cancels the curve, safes the drive on that tick, and stops
//         the routine there — under onFail(carryOn) too; a cancel in the END phase cancels the end
//         MoveToPose with it                              [M11 cancel() skips the end MoveToPose]
//   CV8   a per-step heading tolerance reaches the curve's end (STEP1-A, no new code)
//   CV12  the facade's door: 0 / 17 points and a NaN refused BEFORE any motion   [M16 the finiteness
//         check moved past the start of the motion]
//   CV17  a per-step position tolerance reaches the curve's end (STEP1-A, no new code)
//   R     the Routine steps: success, a refusal as a MotionFailed stop, onFail(carryOn); a refused
//         reverseCurveTo names ITSELF in the stop, and its Warn names the BACK
//                                                    [R20 the reverse step named "curveTo"; P3]

#include "doctest.h"

#include <cmath>
#include <limits>
#include <span>
#include <string>
#include <vector>

#include "mechanism_auton_rig.hpp"
#include "motion_test_rig.hpp"
#include "shulib/chassis/chassis.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/competition/match_runner.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/motion/curve_follow.hpp"
#include "shulib/motion/run_reporter.hpp"
#include "shulib/units/literals.hpp"

using namespace motion_rig;
using namespace shulib::units::literals;
using shulib::chassis::carryOn;
using shulib::chassis::CurvePoint;
using shulib::chassis::Routine;
using shulib::chassis::RoutineStopCause;
using shulib::competition::AutonContext;
using shulib::competition::AutonRunOutcome;
using shulib::competition::runAuton;
using shulib::control::ExitReason;
using shulib::hal::BrakeMode;
using shulib::hal::LogLevel;
using shulib::hal::fake::FakeTelemetrySink;
using shulib::kinematics::TankKinematics;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::units::Length;

namespace {

namespace motion = shulib::motion;

constexpr double kDeg = Angle::kPi / 180.0;

Pose2d P(double x, double y, double deg) { return Pose2d{Length{x}, Length{y}, Angle::degrees(deg)}; }

/// A tank on the host test plant behind the real Chassis, starting at `start`.
struct Tank {
    TankKinematics kin{Length{12.0}};
    FakeTelemetrySink sink;
    MotionRig rig;
    PlantPacer pacer;
    shulib::chassis::Chassis chassis;
    explicit Tank(const Pose2d& start = {})
        : rig{kin, cfgFor(start), &sink}, pacer{rig.h}, chassis{rig.deps, pacer, chassisConfig()} {}
    static shulib::sim::SimHarnessConfig cfgFor(const Pose2d& start) {
        shulib::sim::SimHarnessConfig c = plantConfig();
        c.plant.initialPose = start;
        return c;
    }
    [[nodiscard]] std::string warns() const {
        std::string out;
        for (int i = 0; i < sink.size(); ++i) {
            if (sink.at(i).level == LogLevel::Warn) {
                out += sink.at(i).message + "\n";
            }
        }
        return out;
    }
};

// ── CV7's autons (a deadline at T = 1.0 s: the run guard's park) ─────────────────────────
bool gCarryOn = false;
bool gSafeOnReturn = false;
ExitReason gCurveExit = ExitReason::Running;
std::string gCurveName;

/// A long curve the deadline cuts in its PURSUIT, then a step that must never run.
shulib::chassis::RoutineResult curveCutByDeadline(AutonContext& ctx) {
    Routine r{ctx, "curve-cut"};
    if (gCarryOn) {
        r.onFail(carryOn);
    }
    r.curveTo({{36_in, 12_in}, {72_in, 0_in, 0_deg}});
    // Back in the auton right after the cut: the drive must already be safe (0 V + Brake).
    gSafeOnReturn = true;
    for (const shulib::hal::IMotor* m : ctx.chassis.deps().ctx->driveMotors()) {
        gSafeOnReturn = gSafeOnReturn && m->commandedVoltage().value() == 0.0
                        && m->brakeMode() == BrakeMode::Brake;
    }
    gCurveExit = ctx.chassis.lastCompleted().exit;
    gCurveName = ctx.chassis.lastCompleted().name;
    r.driveTo(0_in, 0_in);
    return r.result();
}

}  // namespace

// ═══ CV7 — a deadline mid-curve ═══════════════════════════════════════════════════════════
// Bug caught: a curve the deadline cannot cut (it would keep driving past the buzzer), a cut that
// leaves the drive energised, or a carry-on choice that lets the chain go on past the deadline.
TEST_CASE("CV7 the run guard's deadline cuts a curve: CANCELLED, safe on that tick, the chain stops") {
    for (const bool carry : {false, true}) {
        CAPTURE(carry);
        gCarryOn = carry;
        gSafeOnReturn = false;
        gCurveExit = ExitReason::Running;
        gCurveName.clear();
        auton_rig::RobotX x;
        const auto res = runAuton(x.chassis, x.guard, auton_rig::entryFor("curve", &curveCutByDeadline, 1.0, 2.5),
                                  nullptr, x.sink, x.svc, auton_rig::kPeriod);
        REQUIRE(res.outcome == AutonRunOutcome::RoutineRan);
        CHECK(res.guard.scoringCut);
        CHECK(res.guard.scoringEnded.value() < 1.02);  // within one tick of T + 1.00
        CHECK(gCurveExit == ExitReason::Cancelled);
        CHECK(gSafeOnReturn);
        CHECK(res.routine.stoppedAt == 1);
        CHECK(std::string{res.routine.stoppedName} == "curveTo");
        CHECK(res.routine.cause == RoutineStopCause::MotionFailed);
        CHECK(res.routine.exit == ExitReason::Cancelled);
        CHECK(res.routine.failedOn == 0);  // the deadline STOPS, whatever the choice
        CHECK(res.routine.skipped == 1);
        // The motion that was cut is the curve, by name; the stop line names the step.
        CHECK(gCurveName == "CurveTo");
        bool named = false;
        for (const std::string& l : x.lines("RTN")) {
            named = named || l.find("STOPPED at step 1 (curveTo): motion CANCELLED") != std::string::npos;
        }
        CHECK(named);
    }
}

// Bug caught: cancel() skipping the end MoveToPose (M11) — cut in the END phase, the end leg would be
// left "running" behind a cancelled curve. Through the scheduler, the same cancel the guard's latch
// and every pre-emption call.
TEST_CASE("CV7 a cancel in the curve's END phase cancels its end MoveToPose too (one exit record)") {
    TankKinematics kin{Length{12.0}};
    FakeTelemetrySink sink;
    SchedulerRig s{kin, plantConfig(), &sink};
    const std::vector<CurvePoint> pts = {CurvePoint{36_in, 0_in, 0_deg}};
    motion::CurveFollow m{s.sched.deps(), std::span<const CurvePoint>{pts}, motionConfig(), 6.0};
    s.sched.async(m);
    // Wait until the end MoveToPose has run a few ticks.
    int endTicks = 0;
    REQUIRE(s.sched.waitUntil(
                [&] {
                    endTicks += m.curveStatus().phase == motion::CurvePhase::Ending ? 1 : 0;
                    return endTicks >= 3;
                },
                5.0)
            == motion::WaitResult::Satisfied);
    REQUIRE(m.endMotion() != nullptr);
    const int recordsBefore = sink.recordCount();
    s.sched.cancel();
    CHECK(m.exitReason() == ExitReason::Cancelled);
    CHECK(m.endMotion()->state() == motion::MotionState::Cancelled);
    CHECK(m.endMotion()->exitReason() == ExitReason::Cancelled);
    for (int w = 0; w < s.rig.h.motorCount(); ++w) {
        CHECK(s.rig.h.motor(w).commandedVoltage().value() == 0.0);
        CHECK(s.rig.h.motor(w).brakeMode() == BrakeMode::Brake);
    }
    int exitRecords = 0;
    for (int i = recordsBefore; i < sink.recordCount(); ++i) {
        exitRecords += sink.recordAt(i).activeCommandState
                               == static_cast<std::uint8_t>(motion::MotionState::Cancelled)
                           ? 1
                           : 0;
    }
    CHECK(exitRecords == 1);
    m.cancel();  // idempotent: the verdict stays
    CHECK(m.exitReason() == ExitReason::Cancelled);
}

// ═══ CV8 / CV17 — STEP1-A's tolerances reach the curve's end, with no new code ════════════
// Bug caught: a curve that builds its end from the ROBOT's config instead of the per-call one (the
// tolerance would be silently ignored).
TEST_CASE("CV17 {.positionTolerance = 2_in}: a curve too short for its sideways miss now ends within 2 in") {
    // From 12 in back and 4 in to the side (closer than guide 05's run-up rule): with the robot's own
    // 0.5 in it times out at the budget; with 2 in it settles, within 2 in and slower than 4 in/s.
    {
        Tank t{P(-12, 4, 0)};
        CHECK(t.chassis.curveTo({{0_in, 0_in, 0_deg}}, {.timeout = 3_s}) == ExitReason::TimedOut);
    }
    Tank t{P(-12, 4, 0)};
    const ExitReason e = t.chassis.curveTo({{0_in, 0_in, 0_deg}},
                                           {.timeout = 3_s, .positionTolerance = 2_in});
    CHECK(e == ExitReason::Settled);
    const double err = posErr(t.rig.h.truePose(), P(0, 0, 0));
    CAPTURE(err);
    CHECK(err <= 2.0);
    CHECK(err > 0.5);  // the tolerance, not the default, is what ended it
    CHECK(std::hypot(t.rig.loc.twist().vx().value(), t.rig.loc.twist().vy().value()) <= 4.0);
}

TEST_CASE("CV8 a per-step heading tolerance reaches the curve's end: 3 deg looser, 0.1 deg tighter") {
    // On the lag-free plant the heading is rarely what ends a curve (its end is lined up by the
    // hand-off), so a LOOSER tolerance barely moves the time; a TIGHTER one must slow or fail the end
    // — which only happens if the per-call config reaches the end MoveToPose. Measured: the 12 in bend
    // settles at 2.04 s by default, 2.03 s at 3 deg.
    const std::vector<CurvePoint> pts = {CurvePoint{12_in, 12_in, 90_deg}};
    Tank base{};
    REQUIRE(base.chassis.curveTo(std::span<const CurvePoint>{pts}, {.timeout = 4_s}) == ExitReason::Settled);
    const double tBase = base.rig.h.clock().now().value();
    Tank loose{};
    REQUIRE(loose.chassis.curveTo(std::span<const CurvePoint>{pts},
                                  {.timeout = 4_s, .headingTolerance = 3_deg})
            == ExitReason::Settled);
    const double tLoose = loose.rig.h.clock().now().value();
    CHECK(std::abs(loose.rig.h.truePose().heading().errorTo(Angle::degrees(90))) <= 3.0 * kDeg);
    Tank tight{};
    const ExitReason eTight = tight.chassis.curveTo(std::span<const CurvePoint>{pts},
                                                    {.timeout = 4_s, .headingTolerance = 0.1_deg});
    const double tTight = tight.rig.h.clock().now().value();
    CAPTURE(tBase);
    CAPTURE(tLoose);
    CAPTURE(tTight);
    CHECK(tLoose <= tBase);
    CHECK((eTight == ExitReason::TimedOut || tTight > tBase + 0.05));
    if (eTight == ExitReason::Settled) {
        CHECK(std::abs(tight.rig.h.truePose().heading().errorTo(Angle::degrees(90))) <= 0.1 * kDeg);
    }
}

// ═══ CV12 — the facade's door ════════════════════════════════════════════════════════════
// Bug caught: input checked after the motion starts (M16 — a NaN in point 3 would drive the first
// points, or a garbage path, before anything complains).
TEST_CASE("CV12 0 points, 17 points and a NaN in point 3 are refused BEFORE any motion") {
    Tank t{};
    const double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK_THROWS(t.chassis.curveTo(std::span<const CurvePoint>{}));
    std::vector<CurvePoint> seventeen;
    for (int i = 1; i <= 17; ++i) {
        seventeen.push_back(CurvePoint{Length{6.0 * i}, Length{0.0}});
    }
    CHECK_THROWS(t.chassis.curveTo(std::span<const CurvePoint>{seventeen}));
    const std::vector<CurvePoint> withNan = {CurvePoint{24_in, 0_in}, CurvePoint{36_in, 12_in},
                                             CurvePoint{Length{48.0}, Length{nan}}};
    CHECK_THROWS(t.chassis.curveTo(std::span<const CurvePoint>{withNan}));
    CHECK_THROWS(t.chassis.reverseCurveTo(std::span<const CurvePoint>{withNan}));
    CHECK(t.chassis.scheduler().motionsStarted() == 0);
    CHECK(posErr(t.rig.h.truePose(), P(0, 0, 0)) == 0.0);  // the robot never moved
    // A bad option is the same door.
    CHECK_THROWS(t.chassis.curveTo({{24_in, 0_in}}, {.timeout = shulib::units::Time{-1.0}}));
    CHECK(t.chassis.scheduler().motionsStarted() == 0);
}

// ═══ R — the Routine steps ═══════════════════════════════════════════════════════════════
// Bug caught: a step that delegates to the wrong verb, swallows a refusal, or does not honour onFail.
TEST_CASE("R the Routine's curve steps: success, a refused curve stops the chain, carryOn goes on") {
    {
        Tank t{};
        Routine r{t.chassis, "curves"};
        r.curveTo({{24_in, 12_in}, {48_in, 24_in, 0_deg}}).reverseCurveTo({{24_in, 12_in}, {0_in, 0_in, 0_deg}});
        CHECK(r.ok());
        CHECK(r.result().completed == 2);
        CHECK(posErr(t.rig.h.truePose(), P(0, 0, 0)) <= 0.5);
        CHECK(std::string{t.chassis.lastCompleted().name} == "ReverseCurveTo");
    }
    {
        Tank t{};
        Routine r{t.chassis, "behind"};
        r.curveTo({{-48_in, 0_in}}).driveTo(24_in, 0_in);  // the first point is BEHIND the robot
        CHECK_FALSE(r.ok());
        CHECK(r.result().stoppedAt == 1);
        CHECK(std::string{r.result().stoppedName} == "curveTo");
        CHECK(r.result().cause == RoutineStopCause::MotionFailed);
        CHECK(r.result().exit == ExitReason::TimedOut);
        CHECK(r.result().skipped == 1);
        CHECK(t.warns().find("curve not driven: its first point is 180 deg off the robot's heading") !=
              std::string::npos);
        CHECK(posErr(t.rig.h.truePose(), P(0, 0, 0)) == 0.0);
    }
    {
        Tank t{};
        Routine r{t.chassis, "carry"};
        r.onFail(carryOn).curveTo({{-48_in, 0_in}});
        r.driveTo(24_in, 0_in);
        CHECK(r.result().failedOn == 1);
        CHECK(std::string{r.result().firstFailedName} == "curveTo");
        CHECK(r.result().completed == 1);
        CHECK(posErr(t.rig.h.truePose(), P(24, 0, 0)) <= 0.5);
    }
    {
        Tank t{};
        Routine r{t.chassis, "rev"};
        r.reverseCurveTo({{48_in, 0_in}}).driveTo(24_in, 0_in);  // in FRONT of a backing robot
        CHECK_FALSE(r.ok());
        CHECK(r.result().stoppedAt == 1);
        CHECK(std::string{r.result().stoppedName} == "reverseCurveTo");
        CHECK(r.result().cause == RoutineStopCause::MotionFailed);
        CHECK(t.warns().find("curve not driven: its first point is 180 deg off the robot's back (at "
                             "most 90) -- turn first: faceAway()") != std::string::npos);
        CHECK(t.warns().find("turn first: face()") == std::string::npos);
        CHECK(posErr(t.rig.h.truePose(), P(0, 0, 0)) == 0.0);
    }
}
