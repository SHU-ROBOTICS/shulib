// Tests for diag/plausibility_guard.hpp — D-5, physical-plausibility invariants.
// What each targets:
//  * FIRES: each invariant must catch its injected violation and raise IMPLAUSIBLE
//    (a guard that cannot fire is decoration).
//  * RECOVERS: every violation is log-and-recover — the run continues, values are
//    made safe, and a healthy tick re-arms the episode (no crash, no fault storm).
//  * STAYS QUIET: physical motion inside the envelope must never fire — a guard
//    that cries wolf gets deleted at the first field session.

#include "doctest.h"

#include <cmath>
#include <limits>

#include "motion_test_rig.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/diag/plausibility_guard.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/kinematics/x_drive.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/frame.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/motion/command_pipeline.hpp"
#include "shulib/units/quantity.hpp"

using shulib::PreconditionError;
using shulib::diag::FaultCode;
using shulib::diag::FaultLatch;
using shulib::diag::PlausibilityConfig;
using shulib::diag::PoseDeltaGuard;
using shulib::diag::commandWithinCapability;
using shulib::diag::recoverWheelVoltage;
using shulib::hal::fake::FakeClock;
using shulib::hal::fake::FakeTelemetrySink;
using shulib::math::Angle;
using shulib::math::ChassisSpeeds;
using shulib::math::Pose2d;
namespace units = shulib::units;

namespace {
struct LatchRig {
    FakeTelemetrySink sink;
    FakeClock clock;
    FaultLatch latch{sink, clock};
};

Pose2d poseAt(double x, double y, double headingDeg = 0.0) {
    return Pose2d{units::Length{x}, units::Length{y}, Angle::degrees(headingDeg)};
}
}  // namespace

// Bug caught: the pose-delta invariant not firing on a teleport (the D-5 core), a
// persistent violation spamming one fault per tick (the HealthMonitor anti-spam
// lesson), or the episode never RE-ARMING after recovery.
TEST_CASE("D-5: a pose teleport fires IMPLAUSIBLE once per episode, recovers, and "
          "re-arms for the next episode") {
    LatchRig rig;
    PoseDeltaGuard guard{PlausibilityConfig{.maxSpeed = units::Velocity{100.0},
                                            .maxYawRate = units::AngularVelocity{10.0},
                                            .margin = 1.5}};
    const units::Time dt{0.01};  // allowance: 100*1.5*0.01 = 1.5 in/tick

    CHECK_FALSE(guard.check(poseAt(0.0, 0.0), dt, rig.latch));  // baseline
    CHECK_FALSE(guard.check(poseAt(0.5, 0.0), dt, rig.latch));  // physical: quiet
    CHECK(guard.check(poseAt(8.5, 0.0), dt, rig.latch));        // 8 in teleport: FIRES
    CHECK(rig.latch.raiseCount(FaultCode::Implausible) == 1);
    CHECK(rig.latch.firstFault() == FaultCode::Implausible);
    // The teleport PERSISTS relative to physics for one more tick: same episode,
    // no second raise (the estimate is still moving implausibly).
    CHECK(guard.check(poseAt(16.5, 0.0), dt, rig.latch));
    CHECK(rig.latch.raiseCount(FaultCode::Implausible) == 1);
    // Recovery: a healthy tick returns false and RE-ARMS…
    CHECK_FALSE(guard.check(poseAt(16.6, 0.0), dt, rig.latch));
    // …so the NEXT episode raises again (a dead guard after one episode would
    // miss the second glitch of the match).
    CHECK(guard.check(poseAt(30.0, 0.0), dt, rig.latch));
    CHECK(rig.latch.raiseCount(FaultCode::Implausible) == 2);
    // The detail names the quantity — structured, greppable.
    CHECK(rig.sink.at(0).message.find("pose delta") != std::string::npos);
}

// Bug caught: heading teleports invisible because only position was checked (an
// IMU glitch spins the ESTIMATE 60° in 10 ms while position stays put).
TEST_CASE("D-5: a heading teleport fires; physical rotation stays quiet") {
    LatchRig rig;
    PoseDeltaGuard guard{PlausibilityConfig{.maxSpeed = units::Velocity{100.0},
                                            .maxYawRate = units::AngularVelocity{10.0},
                                            .margin = 1.5}};
    const units::Time dt{0.01};  // heading allowance: 10*1.5*0.01 = 0.15 rad ≈ 8.6°

    CHECK_FALSE(guard.check(poseAt(0.0, 0.0, 0.0), dt, rig.latch));
    CHECK_FALSE(guard.check(poseAt(0.0, 0.0, 5.0), dt, rig.latch));  // brisk but real
    CHECK(guard.check(poseAt(0.0, 0.0, 65.0), dt, rig.latch));       // 60° in a tick
    CHECK(rig.latch.raiseCount(FaultCode::Implausible) == 1);
    CHECK(rig.sink.at(0).message.find("heading delta") != std::string::npos);
}

// Bug caught: the allowance not scaling with dt — a long deliberate pause (pacer
// gap, waitUntil idle) would flag ordinary travel as a teleport.
TEST_CASE("D-5: the envelope scales with dt — the same delta is a fault at 10 ms and "
          "physics at 1 s") {
    LatchRig rig;
    PoseDeltaGuard guard{PlausibilityConfig{.maxSpeed = units::Velocity{100.0},
                                            .maxYawRate = units::AngularVelocity{10.0},
                                            .margin = 1.5}};
    CHECK_FALSE(guard.check(poseAt(0.0, 0.0), units::Time{0.01}, rig.latch));
    // 20 in over a full second = 20 in/s: entirely physical.
    CHECK_FALSE(guard.check(poseAt(20.0, 0.0), units::Time{1.0}, rig.latch));
    // 20 in over 10 ms = 2000 in/s: not on this planet.
    CHECK(guard.check(poseAt(40.0, 0.0), units::Time{0.01}, rig.latch));
}

// Bug caught: reset() not re-baselining — a DELIBERATE teleport (setPose between
// runs) reported as pathology, which is how a correct guard becomes an ignored one.
// Also: a dt<=0 tick must only re-baseline (no interval exists to judge).
TEST_CASE("D-5: reset() forgives a deliberate teleport; dt<=0 only re-baselines") {
    LatchRig rig;
    PoseDeltaGuard guard;
    CHECK_FALSE(guard.check(poseAt(0.0, 0.0), units::Time{0.01}, rig.latch));
    guard.reset();  // the caller re-seeds the pose on purpose
    CHECK_FALSE(guard.check(poseAt(100.0, 50.0), units::Time{0.01}, rig.latch));
    // A zero-dt sample (baseline tick after a monitor reset) judges nothing…
    CHECK_FALSE(guard.check(poseAt(500.0, 50.0), units::Time{0.0}, rig.latch));
    // …but DOES re-baseline: the next real tick is judged from 500, not 100.
    CHECK_FALSE(guard.check(poseAt(500.5, 50.0), units::Time{0.01}, rig.latch));
    CHECK_FALSE(rig.latch.hasFault());
}

// Bug caught: invariant 2 (command within capability) not firing on an
// over-budget or non-finite command — the pipeline-regression detector that
// closed-loop tests structurally cannot see (C4's M21 lesson).
TEST_CASE("D-5: commandWithinCapability fires on over-budget and non-finite commands, "
          "passes honest ones without raising") {
    LatchRig rig;
    const units::Velocity maxLin{60.0};
    const units::AngularVelocity maxAng{6.0};

    const ChassisSpeeds honest{units::Velocity{40.0}, units::Velocity{30.0},
                               units::AngularVelocity{5.0}};  // |v|=50 ≤ 60
    CHECK(commandWithinCapability(honest, maxLin, maxAng, rig.latch, "MOT"));
    CHECK_FALSE(rig.latch.hasFault());

    const ChassisSpeeds tooFast{units::Velocity{80.0}, units::Velocity{0.0},
                                units::AngularVelocity{0.0}};
    CHECK_FALSE(commandWithinCapability(tooFast, maxLin, maxAng, rig.latch, "MOT"));
    const ChassisSpeeds tooSpinny{units::Velocity{0.0}, units::Velocity{0.0},
                                  units::AngularVelocity{9.0}};
    CHECK_FALSE(commandWithinCapability(tooSpinny, maxLin, maxAng, rig.latch, "MOT"));
    const ChassisSpeeds nan{units::Velocity{std::numeric_limits<double>::quiet_NaN()},
                            units::Velocity{0.0}, units::AngularVelocity{0.0}};
    CHECK_FALSE(commandWithinCapability(nan, maxLin, maxAng, rig.latch, "MOT"));
    CHECK(rig.latch.raiseCount(FaultCode::Implausible) == 3);
    CHECK(rig.sink.at(0).message.find("command outside capability") != std::string::npos);
    // At the exact budget (the clamp's own output) the 1% audit margin admits it.
    const ChassisSpeeds exact{units::Velocity{60.0}, units::Velocity{0.0},
                              units::AngularVelocity{6.0}};
    CHECK(commandWithinCapability(exact, maxLin, maxAng, rig.latch, "MOT"));
}

// Bug caught: invariant 3 letting a bad volt REACH a motor — recovery here is
// real (0 V for NaN, clamp for over-ceiling), and a healthy volt must pass
// bit-untouched (the pipeline's bit-identity depends on it).
TEST_CASE("D-5: recoverWheelVoltage — healthy passes untouched; NaN → 0 V; "
          "over-ceiling → clamped; each raised") {
    LatchRig rig;
    const units::Voltage ceiling{12.0};

    CHECK(recoverWheelVoltage(units::Voltage{7.5}, ceiling, rig.latch, "MOT").value()
          == 7.5);
    CHECK(recoverWheelVoltage(units::Voltage{-12.0}, ceiling, rig.latch, "MOT").value()
          == -12.0);
    CHECK_FALSE(rig.latch.hasFault());

    const auto fromNan = recoverWheelVoltage(
        units::Voltage{std::numeric_limits<double>::quiet_NaN()}, ceiling, rig.latch, "MOT");
    CHECK(fromNan.value() == 0.0);
    const auto fromHot =
        recoverWheelVoltage(units::Voltage{45.0}, ceiling, rig.latch, "MOT");
    CHECK(fromHot.value() == doctest::Approx(12.0 * 1.01));
    CHECK(std::isfinite(fromHot.value()));
    const auto fromNegInf = recoverWheelVoltage(
        units::Voltage{-std::numeric_limits<double>::infinity()}, ceiling, rig.latch, "MOT");
    CHECK(fromNegInf.value() == 0.0);
    CHECK(rig.latch.raiseCount(FaultCode::Implausible) == 3);
}

// Bug caught: the PIPELINE's audit wiring severed — found as a GREEN mutation
// during the C5 campaign (M23): with both diag:: audit calls deleted from
// applyCommandPipeline, all 915k assertions stayed green, because the audit is a
// pass-through on healthy input and every D-5 test injected at the free
// functions. This case drives HOSTILE speeds through the pipeline itself (the
// public Tier-3 entry): the clamps propagate the NaN (std::clamp/hypot keep it),
// so invariant 2 must raise at the pipeline's own call site AND invariant 3 must
// stop the NaN volts at the motors. Sever either call and THIS goes red.
TEST_CASE("D-5: the command pipeline's own audit wiring — NaN speeds raise "
          "IMPLAUSIBLE and the motors get 0 V, through applyCommandPipeline itself") {
    using motion_rig::MotionRig;
    const auto kin = shulib::kinematics::xDrive(shulib::units::Length{7.0});
    MotionRig rig{kin};
    const shulib::control::Feedforward ff{motion_rig::motionConfig().wheelFf};

    const ChassisSpeeds hostile{
        units::Velocity{std::numeric_limits<double>::quiet_NaN()},
        units::Velocity{0.0}, units::AngularVelocity{0.0}};
    const auto out = shulib::motion::applyCommandPipeline(
        rig.deps, motion_rig::motionConfig(), ff, hostile, shulib::math::Frame::Body,
        Angle{});
    (void)out;
    // EACH invariant's wiring pinned DISTINCTLY (the first closure attempt used a
    // combined count and invariant 2's severing stayed green behind invariant 3's
    // four raises — a lesson in one mutation hiding behind another): the command
    // audit and the volt recovery write different structured details.
    bool sawCommandAudit = false;
    bool sawVoltRecovery = false;
    for (int i = 0; i < rig.faultSink.size(); ++i) {
        const std::string& msg = rig.faultSink.at(i).message;
        sawCommandAudit = sawCommandAudit
                          || msg.find("command outside capability") != std::string::npos;
        sawVoltRecovery = sawVoltRecovery || msg.find("wheel volt") != std::string::npos;
    }
    CHECK(sawCommandAudit);  // invariant 2 fired AT THE PIPELINE's call site
    CHECK(sawVoltRecovery);  // invariant 3 fired AT THE PIPELINE's call site
    CHECK(rig.latch.raiseCount(FaultCode::Implausible)
          == 1 + kin.wheelCount());  // 1 command + one per NaN wheel volt
    // Invariant 3's recovery is REAL: no motor holds a non-finite command. (The
    // FakeMotor itself precondition-rejects NaN volts, so with the recovery
    // severed this test dies before these lines — red either way.)
    for (int i = 0; i < kin.wheelCount(); ++i) {
        const double v = rig.h.motor(i).commandedVoltage().value();
        CHECK(std::isfinite(v));
        CHECK(v == 0.0);  // NaN volts recover to exactly 0 V, never garbage
    }
}

// Bug caught: a nonsense envelope accepted silently (maxSpeed 0 would flag every
// motion; a NaN margin would flag none — both must be loud at construction).
TEST_CASE("D-5: nonsense PlausibilityConfig is a loud precondition") {
    CHECK_THROWS_AS((PoseDeltaGuard{PlausibilityConfig{.maxSpeed = units::Velocity{0.0}}}),
                    PreconditionError);
    CHECK_THROWS_AS(
        (PoseDeltaGuard{PlausibilityConfig{.maxYawRate = units::AngularVelocity{-1.0}}}),
        PreconditionError);
    CHECK_THROWS_AS((PoseDeltaGuard{PlausibilityConfig{.margin = 0.5}}), PreconditionError);
    CHECK_THROWS_AS(
        (PoseDeltaGuard{
            PlausibilityConfig{.margin = std::numeric_limits<double>::quiet_NaN()}}),
        PreconditionError);
}
