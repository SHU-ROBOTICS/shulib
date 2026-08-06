// C1 UNDER A3 HOSTILITY — the DoD's survival matrix for the motion layer, plus
// the two A3 handoffs (wait-for-live-estimate, the ODO_STUCK cross-check) and
// THE measurement that matters most: SETTLED-vs-TRUTH DIVERGENCE. A motion's
// exit criteria read the ESTIMATE; under hostility the estimate lies by
// construction — so every settled exit here is graded on the gap between "the
// motion believed it arrived" and where the robot ACTUALLY is. The worst gap
// per family is measured, bounded, and reported.
//
// Contract enforced in every case (A3's fault discipline): a fault code where
// a pathology fires, a FINITE pose and finite volts every tick, bounded
// damage, termination — never a hang, never a NaN, never a silent wrong answer.

#include "doctest.h"

#include <cmath>
#include <functional>
#include <string>
#include <memory>

#include "motion_test_rig.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/diag/finite_guard.hpp"
#include "shulib/kinematics/x_drive.hpp"
#include "shulib/motion/drive_brake.hpp"
#include "shulib/motion/hold_pose.hpp"
#include "shulib/motion/move_to_pose.hpp"
#include "shulib/motion/strafe_to.hpp"
#include "shulib/motion/turn_to.hpp"
#include "shulib/sim/hostile/composed.hpp"

using namespace motion_rig;
using shulib::control::ExitReason;
using shulib::diag::FaultCode;
using shulib::diag::isFinitePose;
using shulib::kinematics::xDrive;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::motion::DriveBrake;
using shulib::motion::HoldPose;
using shulib::motion::IMotion;
using shulib::motion::MotionState;
using shulib::motion::MoveToPose;
using shulib::motion::StrafeTo;
using shulib::motion::TurnTo;
using shulib::sim::EncoderHostileConfig;
using shulib::sim::EncoderHostileModel;
using shulib::sim::FullHostility;
using shulib::sim::ImuHostileConfig;
using shulib::sim::ImuHostileModel;
using shulib::sim::JitterSchedule;
using shulib::sim::LatencyHostileConfig;
using shulib::sim::LatencyHostileModel;
using shulib::sim::PowerHostileConfig;
using shulib::sim::PowerHostileModel;
using shulib::sim::SlipHostileConfig;
using shulib::sim::SlipHostileModel;
using shulib::units::AngularVelocity;
using shulib::units::Time;
using shulib::units::Velocity;
using shulib::units::Voltage;

namespace {

/// Divergence bounds for a SETTLED exit under hostility. Provisional-magnitude
/// arithmetic, not wishes: A3 measured the full composed stack at ≤ 0.26 in
/// over 14 s; a C1 motion is shorter, plus the settle tolerance itself (0.5 in
/// / 1.15°). Anything past these means the estimate lied MATERIALLY at the
/// settled instant.
constexpr double kHostilePosGapBound = 1.5;              // inches
constexpr double kHostileHeadGapBound = 0.035;           // rad (~2°)

struct GapStats {
    double posGap = 0.0;   // |truth − estimate| at exit (the settled lie)
    double posMiss = 0.0;  // |truth − target| at exit (what the field sees)
    double headMiss = 0.0;
};

/// Run one motion under one hostile world with per-tick finiteness REQUIREd;
/// requires a SETTLED exit and returns the truth-graded gaps.
GapStats runHostileSettle(shulib::sim::DegradationModel* model, std::uint64_t seed,
                          const std::function<std::unique_ptr<IMotion>(
                              shulib::motion::MotionDeps&)>& makeMotion,
                          const Pose2d& target, int maxTicks = 1200) {
    const auto kin = xDrive(Length{7.0});
    auto pcfg = plantConfig();
    pcfg.plant.seed = seed;
    MotionRig rig{kin, pcfg, nullptr, model};
    auto m = makeMotion(rig.deps);
    m->start();
    auto reason = ExitReason::Running;
    for (int i = 0; i < maxTicks && reason == ExitReason::Running; ++i) {
        rig.loc.update();
        reason = m->tick();
        REQUIRE(isFinitePose(rig.loc.pose()));
        for (int w = 0; w < rig.h.motorCount(); ++w) {
            REQUIRE(std::isfinite(rig.h.motor(w).commandedVoltage().value()));
        }
        if (reason == ExitReason::Running) {
            rig.h.plant().step(Time{0.01});
        }
    }
    REQUIRE(reason == ExitReason::Settled);
    GapStats g;
    g.posGap = posErr(rig.h.truePose(), rig.loc.pose());
    g.posMiss = posErr(rig.h.truePose(), target);
    g.headMiss = headErr(rig.h.truePose(), target);
    return g;
}

}  // namespace

// ═══ A3 HANDOFF #1 — the wait-for-live-estimate contract ═══════════════════════════

// Bug caught: a motion acting on the boot-frozen pose (the exact hole A3's fix
// left open — driving on an estimate that does not exist yet).
TEST_CASE("C1 wait-for-live: a motion issued during IMU calibration waits at ZERO volts, then drives") {
    ImuHostileModel imu{ImuHostileConfig{}};  // 2 s calibration + drift + noise
    const auto kin = xDrive(Length{7.0});
    MotionRig rig{kin, plantConfig(), nullptr, &imu};
    const Pose2d target{Length{24.0}, Length{0.0}, Angle{}};
    MoveToPose m{rig.deps, target, motionConfig(), 6.0};
    m.start();

    int waitTicks = 0;
    bool voltsDuringWait = false;
    auto reason = ExitReason::Running;
    for (int i = 0; i < 900 && reason == ExitReason::Running; ++i) {
        rig.loc.update();
        reason = m.tick();
        if (m.state() == MotionState::WaitingForEstimate) {
            ++waitTicks;
            for (int w = 0; w < rig.h.motorCount(); ++w) {
                voltsDuringWait =
                    voltsDuringWait || rig.h.motor(w).commandedVoltage().value() != 0.0;
            }
            // the whole point: it must not move while it cannot know where it is
            REQUIRE(posErr(rig.h.truePose(), Pose2d{}) < 1e-9);
        }
        if (reason == ExitReason::Running) {
            rig.h.plant().step(Time{0.01});
        }
    }
    REQUIRE(reason == ExitReason::Settled);
    CHECK(waitTicks > 190);            // it genuinely waited out calibration (~2 s + settle)
    CHECK(waitTicks < 260);            // …and no longer than the settle window demands
    CHECK_FALSE(voltsDuringWait);      // ZERO volts on every waiting tick
    CHECK(posErr(rig.h.truePose(), target) < kHostilePosGapBound);
    CHECK_FALSE(rig.latch.hasFault()); // boot is normal: no fault for waiting
}

// Bug caught: an unbounded wait — a dead IMU must not hang the motion (the
// "no motion can hang" clause covers the wait itself).
TEST_CASE("C1 wait-for-live: an estimate that NEVER goes live exits TimedOut, having never moved") {
    ImuHostileConfig cfg;
    cfg.calibrationEnd = Time{1e17};  // never ready
    ImuHostileModel imu{cfg};
    const auto kin = xDrive(Length{7.0});
    MotionRig rig{kin, plantConfig(), nullptr, &imu};
    MoveToPose m{rig.deps, Pose2d{Length{24.0}, Length{0.0}, Angle{}}, motionConfig(), 1.5};
    const ExitReason r = rig.run(m, 400);
    CHECK(r == ExitReason::TimedOut);
    CHECK(m.state() == MotionState::TimedOut);
    CHECK(rig.latch.firstFault() == FaultCode::MotionTimeout);
    CHECK(posErr(rig.h.truePose(), Pose2d{}) < 1e-9);  // never commanded, never moved
    for (int w = 0; w < rig.h.motorCount(); ++w) {
        CHECK(rig.h.motor(w).commandedVoltage().value() == 0.0);
    }
}

// Bug caught: StrafeTo capturing its held heading DURING the boot window —
// it would lock onto calibration garbage (±90° swings) instead of the robot's
// real post-boot heading.
TEST_CASE("C1 wait-for-live: StrafeTo issued during boot captures the POST-boot heading") {
    ImuHostileModel imu{ImuHostileConfig{}};
    const auto kin = xDrive(Length{7.0});
    auto pcfg = plantConfig();
    pcfg.plant.initialPose = Pose2d{Length{0.0}, Length{0.0}, Angle::degrees(40.0)};
    MotionRig rig{kin, pcfg, nullptr, &imu};
    StrafeTo m{rig.deps, Length{0.0}, Length{20.0}, motionConfig(), 6.0};
    const ExitReason r = rig.run(m, 900);
    REQUIRE(r == ExitReason::Settled);
    // The TRUE heading never changed during boot (robot stationary), so a
    // correct capture holds ~the true 40°; a during-boot capture of garbage
    // would have steered the heading loop tens of degrees away.
    CHECK(std::abs(rig.h.truePose().heading().errorTo(Angle::degrees(40.0))) < 0.035);
    CHECK(posErr(rig.h.truePose(),
                 Pose2d{Length{0.0}, Length{20.0}, Angle::degrees(40.0)})
          < kHostilePosGapBound);
}

// Bug caught: gating the SAFE action — DriveBrake must ACT during boot, not
// wait. (Its VERDICT legitimately defers to calibration end: the only rotation
// sensor is serving ±10 rad/s garbage, so "stopped" is unprovable until the
// stream goes live — drive_brake.hpp header. Action now, verdict when provable.)
TEST_CASE("C1 wait-for-live: DriveBrake is exempt — zero volts from tick one during boot") {
    ImuHostileModel imu{ImuHostileConfig{}};
    const auto kin = xDrive(Length{7.0});
    MotionRig rig{kin, plantConfig(), nullptr, &imu};
    DriveBrake m{rig.deps, motionConfig()};
    m.start();
    // Tick a few boot-window steps: the brake COMMAND lands immediately.
    for (int i = 0; i < 10; ++i) {
        rig.loc.update();
        REQUIRE(m.tick() == ExitReason::Running);  // verdict deferred (garbage ω)
        for (int w = 0; w < rig.h.motorCount(); ++w) {
            CHECK(rig.h.motor(w).commandedVoltage().value() == 0.0);
            CHECK(rig.h.motor(w).brakeMode() == shulib::hal::BrakeMode::Brake);
        }
        rig.h.plant().step(Time{0.01});
    }
    // Once calibration ends the verdict lands promptly.
    const ExitReason r = rig.resume(m, 400);
    CHECK(r == ExitReason::Settled);
    CHECK(rig.h.clock().now().value() > 1.9);  // deferred THROUGH the window…
    CHECK(rig.h.clock().now().value() < 3.0);  // …then certified promptly
}

// ═══ A3 HANDOFF #2 — ODO_STUCK: the frozen encoder under the A3 hostile model ══════

// Bug caught: the dead-encoder hole itself. The estimator CANNOT see this
// (A3 §3.4 — zero travel is plausible); only the motion loop's spin-vs-motion
// cross-check stands between a frozen tracking encoder and a silent runaway.
// Mutation #4 (drop the cross-check) must red exactly here.
TEST_CASE("C1 ODO_STUCK: a frozen tracking encoder mid-motion raises the fault via the cross-check") {
    EncoderHostileConfig enc;
    enc.trackingFreezeAt = Time{1.0};
    enc.trackingFreezeIndex = -1;  // both tracking channels die (full odometry freeze)
    EncoderHostileModel model{enc};
    const auto kin = xDrive(Length{7.0});
    MotionRig rig{kin, plantConfig(), nullptr, &model};
    MoveToPose m{rig.deps, Pose2d{Length{40.0}, Length{0.0}, Angle{}}, motionConfig(), 2.5};
    m.start();
    bool estimatorEverFlagged = false;
    auto reason = ExitReason::Running;
    for (int i = 0; i < 600 && reason == ExitReason::Running; ++i) {
        rig.loc.update();
        reason = m.tick();
        REQUIRE(isFinitePose(rig.loc.pose()));
        estimatorEverFlagged = estimatorEverFlagged || rig.loc.lastOdomDeltaImplausible();
        if (reason == ExitReason::Running) {
            rig.h.plant().step(Time{0.01});
        }
    }
    CHECK_FALSE(estimatorEverFlagged);                    // the A3 finding, still true
    CHECK(rig.latch.firstFault() == FaultCode::OdoStuck);  // the loop cross-check caught it
    CHECK(reason == ExitReason::TimedOut);                 // and the watchdog contained it
    CHECK(rig.latch.lastFault() == FaultCode::MotionTimeout);
    // Damage characterization: the robot kept driving while the estimate stood
    // still — bounded by cruise speed × the post-freeze window, and the fault
    // fired within one stall window (~0.3 s) + margin of the freeze.
    CHECK(posErr(rig.h.truePose(), rig.loc.pose()) > 5.0);
    CHECK(posErr(rig.h.truePose(), rig.loc.pose()) < 60.0 * 1.6);
}

// Bug caught: a stall check that false-fires on a pure turn with dead POSITION
// odometry (heading is IMU-owned, so a turn genuinely progresses) — the
// rotation term's reason to exist, from the other side.
TEST_CASE("C1 ODO_STUCK: a frozen tracking encoder does NOT false-fault a pure TurnTo") {
    EncoderHostileConfig enc;
    enc.trackingFreezeAt = Time{0.2};
    enc.trackingFreezeIndex = -1;
    EncoderHostileModel model{enc};
    const auto kin = xDrive(Length{7.0});
    MotionRig rig{kin, plantConfig(), nullptr, &model};
    TurnTo m{rig.deps, Angle::degrees(150.0), motionConfig()};
    const ExitReason r = rig.run(m, 600);
    REQUIRE(r == ExitReason::Settled);  // the turn is genuinely achievable and lands
    CHECK_FALSE(rig.latch.hasFault());  // no ODO_STUCK: real progress was observed
    CHECK(std::abs(rig.h.truePose().heading().errorTo(Angle::degrees(150.0))) < 0.035);
}

// ═══ The settled-vs-truth divergence matrix (per family, then composed) ═══════════

// Bug caught: an estimator/motion pairing whose settled exit is a material lie
// under any single hostility family. The worst gap per family is REPORTED —
// these numbers are the honest answer to "when it says it arrived, where is it?"
TEST_CASE("C1 divergence: settled-vs-truth gap, per hostile family (MoveToPose/TurnTo/StrafeTo)") {
    const Pose2d moveTarget{Length{26.0}, Length{-14.0}, Angle::degrees(75.0)};
    auto mkMove = [&](shulib::motion::MotionDeps& d) -> std::unique_ptr<IMotion> {
        return std::make_unique<MoveToPose>(d, moveTarget, motionConfig(), 6.0);
    };
    auto mkTurn = [&](shulib::motion::MotionDeps& d) -> std::unique_ptr<IMotion> {
        return std::make_unique<TurnTo>(d, Angle::degrees(75.0), motionConfig(), 6.0);
    };
    auto mkStrafe = [&](shulib::motion::MotionDeps& d) -> std::unique_ptr<IMotion> {
        return std::make_unique<StrafeTo>(d, Length{26.0}, Length{-14.0}, motionConfig(), 6.0);
    };

    double worstGap = 0.0;
    double worstMiss = 0.0;
    double worstHead = 0.0;
    using MotionFactory = std::function<std::unique_ptr<IMotion>(shulib::motion::MotionDeps&)>;
    auto grade = [&](const char* family, shulib::sim::DegradationModel* model,
                     std::uint64_t seed) {
        for (const MotionFactory& mk : {MotionFactory{mkMove}, MotionFactory{mkStrafe}}) {
            const GapStats g = runHostileSettle(model, seed, mk, moveTarget);
            CHECK(g.posGap < kHostilePosGapBound);
            CHECK(g.posMiss < kHostilePosGapBound);
            worstGap = std::max(worstGap, g.posGap);
            worstMiss = std::max(worstMiss, g.posMiss);
        }
        const GapStats t = runHostileSettle(
            model, seed, mkTurn, Pose2d{Length{0.0}, Length{0.0}, Angle::degrees(75.0)});
        CHECK(t.headMiss < kHostileHeadGapBound);
        worstHead = std::max(worstHead, t.headMiss);
        MESSAGE("family=", std::string{family}, " worstGap=", worstGap, "in worstMiss=", worstMiss,
                "in worstHeadMiss=", worstHead * 180.0 / Angle::kPi, "deg");
    };

    SUBCASE("imu: calibration + drift + noise") {
        ImuHostileModel m{ImuHostileConfig{}};
        grade("imu", &m, 3);
    }
    SUBCASE("encoders: tick-grid quantization") {
        EncoderHostileModel m{EncoderHostileConfig{}};
        grade("encoders", &m, 4);
    }
    SUBCASE("power: sag + discharge") {
        PowerHostileModel m{PowerHostileConfig{}};
        grade("power", &m, 5);
    }
    SUBCASE("slip: acceleration-triggered traction loss") {
        SlipHostileModel m{SlipHostileConfig{}};
        grade("slip", &m, 6);
    }
    SUBCASE("latency: delayed sensor channels") {
        LatencyHostileModel m{LatencyHostileConfig{}};
        grade("latency", &m, 7);
    }
    SUBCASE("composed: the full A3 hostile world") {
        for (const std::uint64_t seed : {1ULL, 2ULL, 3ULL}) {
            CAPTURE(seed);
            FullHostility world{};
            grade("composed", &world.model(), seed);
        }
    }
}

// ═══ Composed hostility: HoldPose and DriveBrake complete their jobs too ═══════════

TEST_CASE("C1 composed: HoldPose holds and DriveBrake stops under the full hostile world") {
    const auto kin = xDrive(Length{7.0});
    SUBCASE("HoldPose") {
        FullHostility world{};
        MotionRig rig{kin, plantConfig(), nullptr, &world.model()};
        HoldPose m{rig.deps, 2.0, motionConfig()};
        m.start();
        auto reason = ExitReason::Running;
        for (int i = 0; i < 900 && reason == ExitReason::Running; ++i) {
            rig.loc.update();
            reason = m.tick();
            REQUIRE(isFinitePose(rig.loc.pose()));
            if (reason == ExitReason::Running) {
                rig.h.plant().step(Time{0.01});
            }
        }
        REQUIRE(reason == ExitReason::Settled);
        CHECK(posErr(rig.h.truePose(), Pose2d{}) < kHostilePosGapBound);
    }
    SUBCASE("DriveBrake from speed") {
        FullHostility world{};
        MotionRig rig{kin, plantConfig(), nullptr, &world.model()};
        rig.h.commandBodyTwist(shulib::math::ChassisSpeeds{Velocity{40.0}, Velocity{0.0},
                                                           AngularVelocity{0.0}});
        rig.h.runTicks(250, Time{0.01}, [&](int) { rig.loc.update(); });
        DriveBrake m{rig.deps, motionConfig()};
        REQUIRE(rig.run(m, 400) == ExitReason::Settled);
        CHECK(std::abs(rig.h.trueBodyTwist().vx().value()) < 0.5);
        CHECK(std::abs(rig.h.trueBodyTwist().vy().value()) < 0.5);
    }
}

// ═══ Pathology → fault, through the MOTION's own health wiring ═════════════════════

// Bug caught: the motion's HealthMonitor tick missing an observable — each
// pathology below must surface DURING a motion with zero caller wiring.
TEST_CASE("C1 faults: a sentinel breach mid-motion surfaces ODO_STUCK via the motion's wiring") {
    EncoderHostileConfig enc;
    enc.sentinelAt = Time{1.0};
    enc.sentinelFor = Time{0.05};
    EncoderHostileModel model{enc};
    const auto kin = xDrive(Length{7.0});
    MotionRig rig{kin, plantConfig(), nullptr, &model};
    MoveToPose m{rig.deps, Pose2d{Length{30.0}, Length{0.0}, Angle{}}, motionConfig(), 6.0};
    m.start();
    auto reason = ExitReason::Running;
    for (int i = 0; i < 900 && reason == ExitReason::Running; ++i) {
        rig.loc.update();
        reason = m.tick();
        REQUIRE(isFinitePose(rig.loc.pose()));   // the +∞ breach never reaches the pose
        REQUIRE(isFinitePose(rig.h.truePose()));
        if (reason == ExitReason::Running) {
            rig.h.plant().step(Time{0.01});
        }
    }
    REQUIRE(reason == ExitReason::Settled);      // ~1 in of frozen travel, then re-converges
    CHECK(rig.latch.firstFault() == FaultCode::OdoStuck);  // odomImplausible → the monitor
    const double gap = posErr(rig.h.truePose(), rig.loc.pose());
    MESSAGE("sentinel-breach settled gap = ", gap, " in");
    CHECK(gap > 0.3);   // the breach genuinely cost accuracy (bounded lie, visible)
    CHECK(gap < 2.5);   // …and stayed characterized (A3 measured ~1 in + tolerance)
}

TEST_CASE("C1 faults: mid-run IMU dropout does NOT gate motion; IMU_LOST raised; run completes") {
    ImuHostileConfig cfg;
    cfg.calibrationEnd = Time{0.0};
    cfg.rateBiasMax = AngularVelocity{0.0};
    cfg.headingNoiseSigmaRad = 0.0;
    cfg.yawRateNoiseSigmaRadPerS = 0.0;
    cfg.dropoutAt = Time{1.0};
    ImuHostileModel model{cfg};
    const auto kin = xDrive(Length{7.0});
    MotionRig rig{kin, plantConfig(), nullptr, &model};
    // translation-only: the frozen heading equals the true heading, so the
    // Degraded estimate stays usable — exactly why Degraded must not gate
    MoveToPose m{rig.deps, Pose2d{Length{36.0}, Length{0.0}, Angle{}}, motionConfig(), 6.0};
    const ExitReason r = rig.run(m, 900);
    REQUIRE(r == ExitReason::Settled);                       // it kept driving
    CHECK(rig.latch.firstFault() == FaultCode::ImuLost);     // and said why, once
    CHECK(rig.health.imuLost());
    CHECK(rig.loc.qualityClass() == shulib::localization::Localizer::Quality::Degraded);
    CHECK(posErr(rig.h.truePose(), rig.loc.pose()) < kHostilePosGapBound);
}

TEST_CASE("C1 faults: brownout collapse during a motion -> BROWNOUT first, TimedOut, run continues") {
    PowerHostileModel model{PowerHostileConfig{}};
    const auto kin = xDrive(Length{7.0});
    auto pcfg = plantConfig();
    // A nearly-dead pack: a motion-sized load (~8 V/wheel, vs the survival
    // suite's 100 in/s open-loop slam) must still sag it through the cutoff.
    pcfg.plant.batteryVoltage = Voltage{10.8};
    MotionRig rig{kin, pcfg, nullptr, &model};
    MoveToPose m{rig.deps, Pose2d{Length{60.0}, Length{0.0}, Angle{}}, motionConfig(), 2.0};
    const ExitReason r = rig.run(m, 500);
    CHECK(r == ExitReason::TimedOut);                       // it could not move; it did not hang
    CHECK(rig.latch.firstFault() == FaultCode::Brownout);   // the ROOT cause, latched first
    CHECK(rig.health.brownedOut());                         // the run-scoped marker
    CHECK(rig.latch.lastFault() == FaultCode::MotionTimeout);
    CHECK(rig.h.clock().now().value() > 1.9);               // the run reached the watchdog
}

TEST_CASE("C1 faults: thermal droop during a motion raises MOTOR_OVER_TEMP through the wiring") {
    PowerHostileConfig cfg;
    cfg.sagPerCommandedVolt = 0.0;
    cfg.dischargeRatePerS = 0.0;
    cfg.heatRatePerV2 = 0.5;  // strongly accelerated bench thermal mass: the
                              // 55 °C step must arrive INSIDE a ~3 s motion
    cfg.coolRatePerS = 0.0;
    PowerHostileModel model{cfg};
    const auto kin = xDrive(Length{7.0});
    MotionRig rig{kin, plantConfig(), nullptr, &model};
    MoveToPose m{rig.deps, Pose2d{Length{70.0}, Length{0.0}, Angle{}}, motionConfig(), 4.0};
    m.start();
    auto reason = ExitReason::Running;
    for (int i = 0; i < 900 && reason == ExitReason::Running; ++i) {
        // the C1 wiring shape from A3: modeled temperatures → the F4 fakes;
        // the MOTION then reads IMotor::temperature() itself
        for (int w = 0; w < rig.h.motorCount(); ++w) {
            rig.h.motor(w).setTemperature(model.temperatureC(w));
        }
        rig.loc.update();
        reason = m.tick();
        if (reason == ExitReason::Running) {
            rig.h.plant().step(Time{0.01});
        }
    }
    // With cooling disabled the ramp marches through every VEX throttle step
    // (50% → 25% → 12.5%), so the crawl cannot finish 70 in — the watchdog
    // ends it. What this test PINS: the fault surfaced through the motion's
    // own wiring, first, while the run stayed bounded and finite.
    REQUIRE(reason == ExitReason::TimedOut);
    CHECK(rig.latch.firstFault() == FaultCode::MotorOverTemp);
    CHECK(rig.h.motor(0).temperature() >= 55.0);
    CHECK(rig.h.truePose().x().value() > 10.0);  // it drove, then drooped — not dead
}

// ═══ Timing hostility ══════════════════════════════════════════════════════════════

// Bug caught: dt-dependence in the motion's controllers/settle logic — a
// jittered schedule (±20%, 2% spikes ×5) must not break convergence.
TEST_CASE("C1 jitter: a MoveToPose converges under the hostile dt schedule + full hostility") {
    const auto kin = xDrive(Length{7.0});
    FullHostility world{};
    MotionRig rig{kin, plantConfig(), nullptr, &world.model()};
    JitterSchedule schedule{99};
    const Pose2d target{Length{24.0}, Length{10.0}, Angle::degrees(45.0)};
    MoveToPose m{rig.deps, target, motionConfig(), 8.0};
    m.start();
    auto reason = ExitReason::Running;
    for (int i = 0; i < 1200 && reason == ExitReason::Running; ++i) {
        rig.loc.update();
        reason = m.tick();
        REQUIRE(isFinitePose(rig.loc.pose()));
        if (reason == ExitReason::Running) {
            rig.h.plant().step(schedule(i));
        }
    }
    REQUIRE(reason == ExitReason::Settled);
    CHECK(posErr(rig.h.truePose(), target) < kHostilePosGapBound);
}
