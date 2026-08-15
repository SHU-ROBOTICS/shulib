// C1 MOTION PRIMITIVES — core behaviour, graded against GROUND TRUTH.
//
// Every case's comment names the bug class it would catch (the C1 test-bar
// rule: a test that cannot name its bug is not pulling its weight). All grading
// poses come from h.truePose() — the estimate is what the motion READ, truth is
// what the motion DID.

#include "doctest.h"

#include <array>
#include <cmath>
#include <limits>
#include <span>
#include <string>
#include <string_view>

#include "motion_test_rig.hpp"
#include "shulib/control/watchdog.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/diag/finite_guard.hpp"
#include "shulib/diag/term_sink.hpp"
#include "shulib/hal/fake/fake_char_sink.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/kinematics/x_drive.hpp"
#include "shulib/math/frame.hpp"
#include "shulib/motion/odo_stall_check.hpp"
#include "shulib/hal/fake/fake_motor.hpp"
#include "shulib/chassis/robot_context.hpp"
#include "shulib/motion/drive_brake.hpp"
#include "shulib/motion/motion_config.hpp"
#include "shulib/motion/hold_pose.hpp"
#include "shulib/motion/move_to_pose.hpp"
#include "shulib/motion/strafe_to.hpp"
#include "shulib/motion/turn_to.hpp"
#include "shulib/sim/degradation.hpp"

using namespace motion_rig;
using shulib::control::ExitReason;
using shulib::diag::FaultCode;
using shulib::diag::isFinitePose;
using shulib::kinematics::TankKinematics;
using shulib::kinematics::xDrive;
using shulib::math::Angle;
using shulib::math::ChassisSpeeds;
using shulib::math::Pose2d;
using shulib::motion::DriveBrake;
using shulib::motion::HoldPose;
using shulib::motion::MotionState;
using shulib::motion::MoveToPose;
using shulib::motion::StrafeTo;
using shulib::motion::TurnTo;
using shulib::sim::DegradationModel;
using shulib::sim::Rng;
using shulib::units::AngularVelocity;
using shulib::units::Time;
using shulib::units::Velocity;
using shulib::units::Voltage;

namespace {

/// Slack between the settle tolerance (what the ESTIMATE satisfied) and the
/// truth-vs-target bound, for the CLEAN plant where the estimate tracks truth
/// to ~1e-6 in: essentially the tolerance itself plus a hair.
constexpr double kCleanPosSlack = 0.6;    // in  (translationSettle.maxError = 0.5)
constexpr double kCleanHeadSlack = 0.025;  // rad (headingSettle.maxError = 0.02)

/// A test-local disturbance: adds a body-twist worth of wheel motion (post-spin
/// seam — the same physics as being shoved: unpowered tracking wheels SEE it,
/// drive encoders don't).
class PushModel final : public DegradationModel {
public:
    PushModel(const shulib::kinematics::IKinematics& kin, double t0, double t1,
              const ChassisSpeeds& push)
        : wheels_{kin.toWheels(push)}, t0_{t0}, t1_{t1} {}

    [[nodiscard]] shulib::units::Velocity wheelMotionVelocity(int wheel,
                                                              shulib::units::Velocity spin,
                                                              Time now, Rng&) override {
        if (now.value() >= t0_ && now.value() < t1_) {
            return shulib::units::Velocity{spin.value() + wheels_[wheel].value()};
        }
        return spin;
    }

private:
    shulib::kinematics::WheelSpeeds wheels_;
    double t0_;
    double t1_;
};

/// Total traction loss: wheels spin, the robot never moves. The home of the
/// unreachable-target watchdog test AND the ODO_STUCK cross-check's stall case.
class NoTractionModel final : public DegradationModel {
public:
    [[nodiscard]] shulib::units::Velocity wheelMotionVelocity(int, shulib::units::Velocity,
                                                              Time, Rng&) override {
        return shulib::units::Velocity{0.0};
    }
};

}  // namespace

// ── MoveToPose reaches FIELD targets from several starts, graded on truth. ──
// Bug caught: any broken link in the pipeline (PID sign, FF mismatch, frame
// direction, desaturation direction distortion) shows up as a miss or divergence.
TEST_CASE("C1 MoveToPose: X-drive reaches field poses from varied starts (truth-graded)") {
    const auto kin = xDrive(Length{7.0});
    struct Case {
        Pose2d start;
        Pose2d target;
    };
    const Case cases[] = {
        {Pose2d{}, Pose2d{Length{24.0}, Length{0.0}, Angle{}}},                      // straight
        {Pose2d{}, Pose2d{Length{-18.0}, Length{0.0}, Angle{}}},                     // straight back
        {Pose2d{}, Pose2d{Length{20.0}, Length{-14.0}, Angle::degrees(120.0)}},      // diagonal+turn
        {Pose2d{Length{-30.0}, Length{22.0}, Angle::degrees(-135.0)},
         Pose2d{Length{15.0}, Length{-10.0}, Angle::degrees(45.0)}},                 // far, rotated start
        {Pose2d{Length{10.0}, Length{10.0}, Angle::degrees(179.0)},
         Pose2d{Length{10.0}, Length{34.0}, Angle::degrees(-179.0)}},                // across the seam
    };
    for (const Case& c : cases) {
        CAPTURE(c.target.x().value());
        CAPTURE(c.target.y().value());
        auto cfg = plantConfig();
        cfg.plant.initialPose = c.start;
        MotionRig rig{kin, cfg};
        MoveToPose m{rig.deps, c.target, motionConfig()};
        const ExitReason r = rig.run(m, 800);
        REQUIRE(r == ExitReason::Settled);
        CHECK(posErr(rig.h.truePose(), c.target) < kCleanPosSlack);
        CHECK(headErr(rig.h.truePose(), c.target) < kCleanHeadSlack);
        CHECK(m.state() == MotionState::Settled);
    }
}

// ── THE DECOUPLED PROOF: translation and rotation progress SIMULTANEOUSLY. ──
// Bug caught: any turn-then-drive sequencing (the LemLib behaviour this project
// exists to beat). A sequenced implementation has ~zero translation progress by
// the time the heading arrives; here both axes must be mid-flight together.
// This is mutation #2's home.
TEST_CASE("C1 decoupled: diagonal move + 90° heading change happen as ONE motion") {
    const auto kin = xDrive(Length{7.0});
    MotionRig rig{kin};
    const Pose2d target{Length{30.0}, Length{30.0}, Angle::degrees(90.0)};
    MoveToPose m{rig.deps, target, motionConfig()};
    m.start();

    const double totalDist = std::hypot(30.0, 30.0);
    double transFracWhenHeadingArrived = -1.0;
    bool headingProgressWhileTranslating = false;
    double prevTheta = rig.h.plant().truthState().theta;
    auto reason = ExitReason::Running;
    for (int i = 0; i < 800 && reason == ExitReason::Running; ++i) {
        rig.loc.update();
        reason = m.tick();
        if (reason != ExitReason::Running) {
            break;
        }
        rig.h.plant().step(Time{0.01});
        const auto& ts = rig.h.plant().truthState();
        const double transFrac = 1.0 - posErr(rig.h.truePose(), target) / totalDist;
        // first instant the TRUE heading is within 5° of target:
        if (transFracWhenHeadingArrived < 0.0
            && std::abs(Angle::radians(ts.theta).errorTo(target.heading())) < 0.0873) {
            transFracWhenHeadingArrived = transFrac;
        }
        // rotation actively progressing while translation is mid-flight:
        if (transFrac > 0.2 && transFrac < 0.8 && std::abs(ts.theta - prevTheta) > 1e-4) {
            headingProgressWhileTranslating = true;
        }
        prevTheta = ts.theta;
    }
    REQUIRE(reason == ExitReason::Settled);
    // A turn-then-drive would arrive at the heading with ~0% translation done.
    CHECK(transFracWhenHeadingArrived > 0.30);
    CHECK(headingProgressWhileTranslating);
    CHECK(posErr(rig.h.truePose(), target) < kCleanPosSlack);
    CHECK(headErr(rig.h.truePose(), target) < kCleanHeadSlack);
}

// ── The other half of decoupling: a pure strafe HOLDS heading. ──
// Bug caught: cross-coupling from translation into the heading loop (bad frame
// math or a shared controller) shows up as heading wander during a strafe.
TEST_CASE("C1 decoupled: StrafeTo translates laterally while heading holds (truth-graded)") {
    const auto kin = xDrive(Length{7.0});
    auto cfg = plantConfig();
    cfg.plant.initialPose = Pose2d{Length{0.0}, Length{0.0}, Angle::degrees(30.0)};
    MotionRig rig{kin, cfg};
    StrafeTo m{rig.deps, Length{0.0}, Length{24.0}, motionConfig()};
    m.start();

    double worstHeadingDev = 0.0;
    auto reason = ExitReason::Running;
    for (int i = 0; i < 800 && reason == ExitReason::Running; ++i) {
        rig.loc.update();
        reason = m.tick();
        if (reason != ExitReason::Running) {
            break;
        }
        rig.h.plant().step(Time{0.01});
        worstHeadingDev = std::max(
            worstHeadingDev,
            std::abs(rig.h.truePose().heading().errorTo(Angle::degrees(30.0))));
    }
    REQUIRE(reason == ExitReason::Settled);
    CHECK(worstHeadingDev < 0.035);  // ~2°: heading held throughout, not just at the end
    CHECK(posErr(rig.h.truePose(), Pose2d{Length{0.0}, Length{24.0}, Angle::degrees(30.0)})
          < kCleanPosSlack);
    // the held heading is the CAPTURED one (30°), not zero:
    CHECK(std::abs(rig.h.truePose().heading().errorTo(Angle::degrees(30.0))) < kCleanHeadSlack);
}

// ── TurnTo: heading arrives, position stays put. ──
// Bug caught: a turn that translates (kinematics row error, frame leak into the
// linear axes) or misses the heading.
TEST_CASE("C1 TurnTo: rotates in place to a field heading (truth-graded)") {
    for (const bool tank : {false, true}) {
        CAPTURE(tank);
        const auto xk = xDrive(Length{7.0});
        const TankKinematics tk{Length{12.0}};
        const shulib::kinematics::IKinematics& kin =
            tank ? static_cast<const shulib::kinematics::IKinematics&>(tk) : xk;
        MotionRig rig{kin};
        TurnTo m{rig.deps, Angle::degrees(135.0), motionConfig()};
        const ExitReason r = rig.run(m, 600);
        REQUIRE(r == ExitReason::Settled);
        CHECK(std::abs(rig.h.truePose().heading().errorTo(Angle::degrees(135.0))) < kCleanHeadSlack);
        CHECK(posErr(rig.h.truePose(), Pose2d{}) < 0.5);  // stayed in place
    }
}

// ── Tank: reachable targets settle, forward AND reverse; the strafe clamp
// keeps the body command honest every tick. ──
// Bugs caught: tank wheel-order/sign errors (miss); a dropped strafe-authority
// clamp (records would carry body-frame vy on a drive that cannot strafe —
// mutation home for the authority clamp).
TEST_CASE("C1 tank: along-axis targets settle; body vy is clamped to zero (record-audited)") {
    const TankKinematics kin{Length{12.0}};
    for (const double d : {24.0, -18.0}) {
        CAPTURE(d);
        auto cfg = plantConfig();
        cfg.plant.initialPose = Pose2d{Length{5.0}, Length{-4.0}, Angle::degrees(25.0)};
        shulib::hal::fake::FakeTelemetrySink records;
        MotionRig rig{kin, cfg, &records};
        const Pose2d target{Length{5.0 + d * std::cos(Angle::degrees(25.0).radians())},
                            Length{-4.0 + d * std::sin(Angle::degrees(25.0).radians())},
                            Angle::degrees(25.0)};
        MoveToPose m{rig.deps, target, motionConfig()};
        const ExitReason r = rig.run(m, 800);
        REQUIRE(r == ExitReason::Settled);
        CHECK(posErr(rig.h.truePose(), target) < kCleanPosSlack);
        CHECK(headErr(rig.h.truePose(), target) < kCleanHeadSlack);

        // Audit every MOTION record (activeCommandState != 0 — the plant's own
        // records carry 0): the achievable command's BODY vy must be exactly the
        // clamp's output — zero on a strafeAuthority()==0 drive.
        int audited = 0;
        for (int i = 0; i < records.recordCount(); ++i) {
            const auto& rec = records.recordAt(i);
            if (rec.activeCommandState == 0) {
                continue;  // a plant record
            }
            const ChassisSpeeds body =
                shulib::math::fieldToRobot(rec.commanded, rec.measuredPose.heading());
            CHECK(std::abs(body.vy().value()) < 1e-9);
            ++audited;
        }
        CHECK(audited > 50);  // the audit actually ran over a real motion
    }
}

// ── StrafeTo on tank: honestly impossible → TimedOut, never a hang or a lie. ──
// Bug caught: a motion that "settles" on a target it never reached, or spins
// forever on an infeasible demand.
TEST_CASE("C1 tank: StrafeTo to a lateral target exits TimedOut with MOTION_TIMEOUT") {
    const TankKinematics kin{Length{12.0}};
    shulib::hal::fake::FakeTelemetrySink records;
    MotionRig rig{kin, plantConfig(), &records};
    StrafeTo m{rig.deps, Length{0.0}, Length{24.0}, motionConfig(), 1.5};
    const ExitReason r = rig.run(m, 400);
    CHECK(r == ExitReason::TimedOut);
    CHECK(m.state() == MotionState::TimedOut);
    CHECK(rig.latch.firstFault() == FaultCode::MotionTimeout);
    CHECK(posErr(rig.h.truePose(), Pose2d{}) < 1.0);  // it did not wander off sideways
    // Record audit (mutation #5's second detector): the raw demand here is
    // almost PURE lateral, so a dropped authority clamp would put ~maxLinear
    // of body vy into the achievable-command records. Authority 0 ⇒ exactly 0.
    int audited = 0;
    for (int i = 0; i < records.recordCount(); ++i) {
        const auto& rec = records.recordAt(i);
        if (rec.activeCommandState == 0) {
            continue;
        }
        const ChassisSpeeds body =
            shulib::math::fieldToRobot(rec.commanded, rec.measuredPose.heading());
        CHECK(std::abs(body.vy().value()) < 1e-9);
        ++audited;
    }
    CHECK(audited > 50);
}

// ── ExitReason discipline: after exit, tick() is a no-op that repeats the
// verdict and the motors stay stopped. ──
// Bug caught: a finished motion that keeps commanding (would fight the next
// motion C2 schedules), or an exit reason that flaps back to Running.
TEST_CASE("C1 exit discipline: post-exit ticks repeat the verdict; motors stay stopped") {
    const auto kin = xDrive(Length{7.0});
    MotionRig rig{kin};
    MoveToPose m{rig.deps, Pose2d{Length{10.0}, Length{0.0}, Angle{}}, motionConfig()};
    REQUIRE(rig.run(m, 600) == ExitReason::Settled);
    for (int i = 0; i < 5; ++i) {
        rig.loc.update();
        CHECK(m.tick() == ExitReason::Settled);
        for (int w = 0; w < rig.h.motorCount(); ++w) {
            CHECK(rig.h.motor(w).commandedVoltage().value() == 0.0);
        }
        rig.h.plant().step(Time{0.01});
    }
    CHECK(m.exitReason() == ExitReason::Settled);
}

// ── The watchdog: an unreachable target (total traction loss) terminates, and
// the spin-vs-motion cross-check names the cause. ──
// Bugs caught: a defeated watchdog (motion runs forever — mutation #3's home);
// a dropped stall cross-check (no ODO_STUCK — mutation #4's home).
TEST_CASE("C1 watchdog: unreachable target exits TimedOut; stall cross-check raises ODO_STUCK") {
    const auto kin = xDrive(Length{7.0});
    NoTractionModel noTraction;
    MotionRig rig{kin, plantConfig(), nullptr, &noTraction};
    MoveToPose m{rig.deps, Pose2d{Length{40.0}, Length{0.0}, Angle{}}, motionConfig(), 2.0};
    const ExitReason r = rig.run(m, 400);
    CHECK(r == ExitReason::TimedOut);                       // terminated, never hung
    CHECK(rig.h.clock().now().value() < 2.5);               // …at the watchdog, promptly
    CHECK(rig.latch.firstFault() == FaultCode::OdoStuck);   // the cross-check saw it first
    CHECK(rig.latch.lastFault() == FaultCode::MotionTimeout);
    CHECK(posErr(rig.h.truePose(), Pose2d{}) < 1e-9);       // truth: it really never moved
}

// ── DriveBrake stops and stays stopped. ──
// Bug caught: a brake that reports Settled while still rolling, or never
// settles on a stopped robot.
TEST_CASE("C1 DriveBrake: stops from speed and stays stopped (truth-graded)") {
    const auto kin = xDrive(Length{7.0});
    MotionRig rig{kin};
    // get moving first (open loop through the harness)
    rig.h.commandBodyTwist(ChassisSpeeds{Velocity{40.0}, Velocity{10.0}, AngularVelocity{1.0}});
    rig.h.runTicks(50, Time{0.01}, [&](int) { rig.loc.update(); });
    REQUIRE(std::abs(rig.h.trueBodyTwist().vx().value()) > 30.0);

    DriveBrake m{rig.deps, motionConfig()};
    const ExitReason r = rig.run(m, 300);
    REQUIRE(r == ExitReason::Settled);
    CHECK(std::abs(rig.h.trueBodyTwist().vx().value()) < 1e-9);
    CHECK(std::abs(rig.h.trueBodyTwist().vy().value()) < 1e-9);
    CHECK(std::abs(rig.h.trueBodyTwist().omega().value()) < 1e-9);
    // …and STAYS stopped over a further second of ticks:
    const Pose2d atStop = rig.h.truePose();
    rig.h.runTicks(100, Time{0.01}, [&](int) { rig.loc.update(); });
    CHECK(posErr(rig.h.truePose(), atStop) < 1e-9);
    for (int w = 0; w < rig.h.motorCount(); ++w) {
        CHECK(rig.h.motor(w).brakeMode() == shulib::hal::BrakeMode::Brake);
    }
}

// ── HoldPose: pushed off target, it drives back; verdict is honest. ──
// Bugs caught: a hold that doesn't fight back (dead loop), or one that reports
// Settled after being shoved away at the end (the "held the clock out" lie).
TEST_CASE("C1 HoldPose: recovers from a mid-hold shove and reports Settled") {
    const auto kin = xDrive(Length{7.0});
    PushModel push{kin, 1.0, 1.5, ChassisSpeeds{Velocity{25.0}, Velocity{0.0}, AngularVelocity{0.0}}};
    MotionRig rig{kin, plantConfig(), nullptr, &push};
    HoldPose m{rig.deps, 3.5, motionConfig()};
    m.start();

    double worstDev = 0.0;
    auto reason = ExitReason::Running;
    for (int i = 0; i < 600 && reason == ExitReason::Running; ++i) {
        rig.loc.update();
        reason = m.tick();
        if (reason != ExitReason::Running) {
            break;
        }
        rig.h.plant().step(Time{0.01});
        worstDev = std::max(worstDev, posErr(rig.h.truePose(), Pose2d{}));
    }
    REQUIRE(reason == ExitReason::Settled);
    CHECK(worstDev > 2.0);                              // the shove genuinely displaced it
    CHECK(posErr(rig.h.truePose(), Pose2d{}) < 0.6);    // …and it drove back
}

TEST_CASE("C1 HoldPose: still displaced when the window ends -> TimedOut, honestly") {
    const auto kin = xDrive(Length{7.0});
    // push through the END of the hold window so recovery is impossible
    PushModel push{kin, 1.0, 9.9, ChassisSpeeds{Velocity{40.0}, Velocity{0.0}, AngularVelocity{0.0}}};
    MotionRig rig{kin, plantConfig(), nullptr, &push};
    HoldPose m{rig.deps, 2.0, motionConfig()};
    const ExitReason r = rig.run(m, 400);
    CHECK(r == ExitReason::TimedOut);
    // The sustained push settles into a fighting equilibrium: wheels spinning
    // against the shove, robot stationary — which IS the spin-vs-motion stall
    // signature, so ODO_STUCK fires first BY DESIGN (a blocked drivetrain is
    // the same fault family — see odo_stall_check.hpp). The timeout then lands
    // in the cascade.
    CHECK(rig.latch.firstFault() == FaultCode::OdoStuck);
    CHECK(rig.latch.lastFault() == FaultCode::MotionTimeout);
    CHECK(posErr(rig.h.truePose(), Pose2d{}) > 2.0);  // it truly was off target
}

// ── Composition: absolute field targets do not chain error. ──
// Bug caught: relative/odometry-frame targeting (B's landing would inherit A's
// residual) — the defect that makes multi-move routines drift.
TEST_CASE("C1 composition: motion B lands on B's absolute target regardless of A's residual") {
    const auto kin = xDrive(Length{7.0});
    MotionRig rig{kin};
    const Pose2d targetA{Length{24.0}, Length{10.0}, Angle::degrees(45.0)};
    const Pose2d targetB{Length{-10.0}, Length{30.0}, Angle::degrees(-90.0)};
    MoveToPose a{rig.deps, targetA, motionConfig()};
    REQUIRE(rig.run(a, 800) == ExitReason::Settled);
    MoveToPose b{rig.deps, targetB, motionConfig()};
    REQUIRE(rig.run(b, 800) == ExitReason::Settled);
    CHECK(posErr(rig.h.truePose(), targetB) < kCleanPosSlack);
    CHECK(headErr(rig.h.truePose(), targetB) < kCleanHeadSlack);
}

// ── Scaling sanity: time grows like distance/velocity, not quadratically. ──
// Bug caught: a mis-wired integration (e.g. commanding position as velocity)
// makes T(2d) ≈ 4·T(d); the correct velocity-capped loop adds Δd/vmax.
TEST_CASE("C1 scaling: doubling the distance adds ~delta-d/vmax, far from 4x") {
    const auto kin = xDrive(Length{7.0});
    auto timeFor = [&](double dist) {
        MotionRig rig{kin};
        MoveToPose m{rig.deps, Pose2d{Length{dist}, Length{0.0}, Angle{}}, motionConfig()};
        REQUIRE(rig.run(m, 900) == ExitReason::Settled);
        return rig.h.clock().now().value();
    };
    const double t24 = timeFor(24.0);
    const double t48 = timeFor(48.0);
    MESSAGE("t(24in)=", t24, "s  t(48in)=", t48, "s");
    // Δt should be ≈ 24 in / 60 in/s = 0.4 s (the added cruise), with slack:
    CHECK(t48 - t24 > 0.25);
    CHECK(t48 - t24 < 0.60);
    CHECK(t48 < 2.0 * t24);  // and nowhere near 4×
}

// ── Degenerate geometry. ──
// Bugs caught: divide-by-zero at zero distance; a sub-tolerance move that
// overshoots; motion state corruption on dt == 0 (two ticks, no time passing).
TEST_CASE("C1 degenerate: target == current settles promptly and barely moves") {
    const auto kin = xDrive(Length{7.0});
    MotionRig rig{kin};
    MoveToPose m{rig.deps, Pose2d{}, motionConfig()};
    const ExitReason r = rig.run(m, 100);
    REQUIRE(r == ExitReason::Settled);
    CHECK(rig.h.clock().now().value() < 0.5);
    CHECK(posErr(rig.h.truePose(), Pose2d{}) < 0.05);
}

TEST_CASE("C1 degenerate: a sub-tolerance move settles without a round trip") {
    const auto kin = xDrive(Length{7.0});
    MotionRig rig{kin};
    MoveToPose m{rig.deps, Pose2d{Length{0.3}, Length{0.0}, Angle{}}, motionConfig()};
    double worstX = 0.0;
    m.start();
    auto reason = ExitReason::Running;
    for (int i = 0; i < 200 && reason == ExitReason::Running; ++i) {
        rig.loc.update();
        reason = m.tick();
        if (reason == ExitReason::Running) {
            rig.h.plant().step(Time{0.01});
        }
        worstX = std::max(worstX, rig.h.truePose().x().value());
    }
    REQUIRE(reason == ExitReason::Settled);
    CHECK(worstX < 0.8);  // never blew past the tiny target
}

TEST_CASE("C1 degenerate: dt == 0 ticks (no time passing) are safe and stateless") {
    const auto kin = xDrive(Length{7.0});
    MotionRig rig{kin};
    MoveToPose m{rig.deps, Pose2d{Length{20.0}, Length{0.0}, Angle{}}, motionConfig()};
    m.start();
    rig.loc.update();
    (void)m.tick();
    // three more ticks at the SAME clock instant — no NaN, no verdict flap:
    for (int i = 0; i < 3; ++i) {
        rig.loc.update();
        CHECK(m.tick() == ExitReason::Running);
        CHECK(isFinitePose(rig.loc.pose()));
        for (int w = 0; w < rig.h.motorCount(); ++w) {
            CHECK(std::isfinite(rig.h.motor(w).commandedVoltage().value()));
        }
    }
    // and the motion still completes normally afterwards:
    CHECK(rig.resume(m, 800) == ExitReason::Settled);
}

TEST_CASE("C1 degenerate: zero tolerance never settles -> TimedOut; negative tolerance rejected") {
    const auto kin = xDrive(Length{7.0});
    MotionRig rig{kin};
    auto cfg = motionConfig();
    cfg.translationSettle.maxError = 0.0;  // can hold exactly 0.0 error? never, with real dynamics
    MoveToPose m{rig.deps, Pose2d{Length{10.0}, Length{0.0}, Angle{}}, cfg, 1.0};
    CHECK(rig.run(m, 300) == ExitReason::TimedOut);

    auto bad = motionConfig();
    bad.translationSettle.maxError = -1.0;
    CHECK_THROWS_AS((MoveToPose{rig.deps, Pose2d{}, bad}), shulib::PreconditionError);
}

TEST_CASE("C1 preconditions: tick before start, null deps, bad timeout all reject loudly") {
    const auto kin = xDrive(Length{7.0});
    MotionRig rig{kin};
    MoveToPose m{rig.deps, Pose2d{}, motionConfig()};
    CHECK_THROWS_AS((void)m.tick(), shulib::PreconditionError);

    shulib::motion::MotionDeps broken = rig.deps;
    broken.localizer = nullptr;
    CHECK_THROWS_AS((MoveToPose{broken, Pose2d{}, motionConfig()}), shulib::PreconditionError);
    CHECK_THROWS_AS((MoveToPose{rig.deps, Pose2d{}, motionConfig(), -1.0}),
                    shulib::PreconditionError);
    CHECK_THROWS_AS((HoldPose{rig.deps, 0.0, motionConfig()}), shulib::PreconditionError);
}

// ── Reuse: one motion object, run twice (retry semantics C2 will lean on). ──
// Bug caught: stale controller/settle/watchdog state leaking across start().
TEST_CASE("C1 reuse: start() fully re-arms a finished motion") {
    const auto kin = xDrive(Length{7.0});
    MotionRig rig{kin};
    MoveToPose m{rig.deps, Pose2d{Length{15.0}, Length{5.0}, Angle::degrees(30.0)}, motionConfig()};
    REQUIRE(rig.run(m, 800) == ExitReason::Settled);
    // push the robot somewhere else, then run the SAME motion again:
    rig.h.commandBodyTwist(ChassisSpeeds{Velocity{-30.0}, Velocity{0.0}, AngularVelocity{0.0}});
    rig.h.runTicks(60, Time{0.01}, [&](int) { rig.loc.update(); });
    rig.h.stopAllMotors();
    REQUIRE(posErr(rig.h.truePose(), m.target()) > 5.0);
    REQUIRE(rig.run(m, 800) == ExitReason::Settled);
    CHECK(posErr(rig.h.truePose(), m.target()) < kCleanPosSlack);
}

// ── The run is legible through TermSink while it executes (DoD item). ──
// Bug caught: a record stream that bypasses emitRecord or emits records a
// human cannot follow (missing target/error/state content).
// Plumbing note: TermSink's constructor needs the run's clock, which SimHarness
// only exposes AFTER construction — so the live record stream is captured by
// the attached FakeTelemetrySink during the run and each motion record is then
// rendered through TermSink::emit, the identical per-record path a live
// terminal uses.
TEST_CASE("C1 legibility: a motion's record stream renders as framed TermSink lines") {
    const auto kin = xDrive(Length{7.0});
    shulib::hal::fake::FakeTelemetrySink records;
    MotionRig rig{kin, plantConfig(), &records};
    MoveToPose m{rig.deps, Pose2d{Length{12.0}, Length{0.0}, Angle{}}, motionConfig()};
    REQUIRE(rig.run(m, 600) == ExitReason::Settled);

    shulib::hal::fake::FakeCharSink out;
    shulib::diag::TermSink term{rig.h.clock(), out};
    int motionRecords = 0;
    for (int i = 0; i < records.recordCount(); ++i) {
        if (records.recordAt(i).activeCommandState != 0) {  // 0 = a plant record
            term.emit(records.recordAt(i));
            ++motionRecords;
        }
    }
    CHECK(motionRecords > 50);
    CHECK(out.text().find("[t=") != std::string::npos);   // time-stamped
    CHECK(out.text().find("[MOT]") != std::string::npos); // the record channel tag
    std::size_t lines = 0;
    for (const char c : out.text()) {
        if (c == '\n') {
            ++lines;
        }
    }
    CHECK(lines >= static_cast<std::size_t>(motionRecords));  // one framed line each
}

// Bug caught (DEFECTS1 item D13): MotionConfig::validate() guarded its scalars with a bare
// `> 0.0`, which infinity satisfies. An infinite defaultTimeout therefore validated, and
// because `timeout = 0` at a motion's constructor SELECTS config.defaultTimeout, it built a
// Watchdog that can never expire — a motion that runs forever with no TimedOut exit and no
// end-of-run park, defeating the single guarantee watchdog.hpp exists to make. The
// caller-supplied timeout was already screened for finiteness; this field was the way in.
TEST_CASE("MotionConfig: an infinite budget is rejected, at both layers (D13)") {
    const double inf = std::numeric_limits<double>::infinity();

    shulib::motion::MotionConfig cfg;
    cfg.defaultTimeout = inf;
    CHECK_THROWS_AS(cfg.validate(), shulib::PreconditionError);

    for (const char* which : {"lin", "ang", "wheel", "radius"}) {
        shulib::motion::MotionConfig c;
        if (std::string_view{which} == "lin") { c.maxLinearSpeed = shulib::units::Velocity{inf}; }
        if (std::string_view{which} == "ang") {
            c.maxAngularSpeed = shulib::units::AngularVelocity{inf};
        }
        if (std::string_view{which} == "wheel") { c.maxWheelSpeed = shulib::units::Velocity{inf}; }
        if (std::string_view{which} == "radius") { c.rotationRadius = shulib::units::Length{inf}; }
        CHECK_THROWS_AS(c.validate(), shulib::PreconditionError);
    }

    // The second layer: even handed an infinite budget directly, the Watchdog refuses it.
    shulib::hal::fake::FakeClock clk;
    CHECK_THROWS_AS((shulib::control::Watchdog{inf, clk}), shulib::PreconditionError);

    // NEGATIVE CONTROL: the default config still validates and a finite watchdog still
    // constructs — so the throws above are about finiteness, not about a broken validate().
    shulib::motion::MotionConfig good;
    CHECK_NOTHROW(good.validate());
    CHECK_NOTHROW((shulib::control::Watchdog{1.0, clk}));
}

// Bug caught (DEFECTS1 item A1): nothing anywhere compared the drive-motor count to the
// installed kinematics' wheel count, while applyCommandPipeline indexes the motor span by
// WHEEL index with std::span::operator[], which is unchecked. Three motors and an XDrive
// read one past the end of the span on EVERY tick — undefined behaviour, no diagnostic, on
// the hot path. Every RECORD-producing loop already guarded it (`i < motors.size() && ...`);
// only the path that actually drives the robot did not. MotionDeps is the one bundle holding
// both the context and the kinematics, so the cross-check lives there.
TEST_CASE("A1: fewer drive motors than wheels is a loud breach, not silent out-of-bounds") {
    const auto kin = shulib::kinematics::xDrive(shulib::units::Length{7.0});
    REQUIRE(kin.wheelCount() == 4);
    motion_rig::MotionRig rig{kin};

    const auto full = rig.h.context().driveMotors();
    REQUIRE(full.size() == 4);
    const auto three = full.subspan(0, 3);   // one short of the wheel count

    auto makeCtx = [&](std::span<shulib::hal::IMotor* const> motors) {
        return shulib::chassis::RobotContextConfig{.clock = &rig.h.context().clock(),
                                                   .driveMotors = motors,
                                                   .imu = &rig.h.context().imu(),
                                                   .gps = &rig.h.context().gps(),
                                                   .battery = &rig.h.context().battery(),
                                                   .telemetry = &rig.h.context().telemetry(),
                                                   .tags = &rig.h.context().tags(),
                                                   .vision = &rig.h.context().vision()};
    };

    shulib::chassis::RobotContext shortCtx{makeCtx(three)};  // legal on its own
    shulib::motion::MotionDeps shortDeps{.ctx = &shortCtx,
                                         .localizer = &rig.loc,
                                         .kinematics = &kin,
                                         .faults = &rig.latch,
                                         .health = &rig.health};
    CHECK_THROWS_AS(shortDeps.validate(), shulib::PreconditionError);
    // And it must be loud where a caller meets it — at a motion's construction.
    CHECK_THROWS_AS((shulib::motion::TurnTo{shortDeps, shulib::math::Angle::degrees(90.0)}),
                    shulib::PreconditionError);

    // NEGATIVE CONTROL: the identical wiring with a matching count validates, so the throws
    // above are about the count and not about any of the five null checks beside it.
    shulib::chassis::RobotContext fullCtx{makeCtx(full)};
    shulib::motion::MotionDeps okDeps{.ctx = &fullCtx,
                                      .localizer = &rig.loc,
                                      .kinematics = &kin,
                                      .faults = &rig.latch,
                                      .health = &rig.health};
    CHECK_NOTHROW(okDeps.validate());
}

// Bug caught (DEFECTS1 item A30): OdoStallCheck::update() checked only the size CEILING. An
// empty span made the check permanently un-trippable — mean shaft delta 0, so spinTravel 0,
// so `spinTravel >= minSpinTravel` never true — a misconfiguration that silently disables a
// safety cross-check and reports "healthy" forever. A null element was a raw dereference.
// Every sibling fan-out in the tree already checked both.
TEST_CASE("A30: an empty or null-bearing motor span is refused, not silently un-trippable") {
    shulib::motion::OdoStallCheck check;
    const shulib::math::Pose2d origin{};
    const shulib::units::Time t{0.0};

    CHECK_THROWS_AS((void)check.update(t, std::span<shulib::hal::IMotor* const>{}, origin),
                    shulib::PreconditionError);

    shulib::hal::fake::FakeMotor m;
    std::array<shulib::hal::IMotor*, 2> withNull{&m, nullptr};
    CHECK_THROWS_AS((void)check.update(t, std::span<shulib::hal::IMotor* const>{withNull},
                                       origin),
                    shulib::PreconditionError);

    // NEGATIVE CONTROL: a well-formed span is still accepted and still baselines.
    std::array<shulib::hal::IMotor*, 1> ok{&m};
    CHECK_NOTHROW((void)check.update(t, std::span<shulib::hal::IMotor* const>{ok}, origin));
}

// DEFECTS1 items D12 + D14 — FIXED IN THE HEADER, NOT COVERED HERE, and that is recorded
// rather than papered over. HoldPose's only deadline was holdFor + 1.0 s, and that same
// watchdog bounds the wait-for-live boot window, so HoldPose(deps, 0.5) had a 1.5 s total
// budget against a ~2 s V5 IMU calibration and exited TimedOut before its hold began. The
// watchdog is now armed with max(holdFor + slack, the effective timeout).
//
// NO TEST PINS IT, and mutation M21 (reverting to holdFor + kHoldSlack) stayed GREEN. Two
// attempts failed for the same reason: MotionRig's localizer is seeded live in its
// constructor, so neither a long bootSettleTime nor holding the IMU un-ready keeps
// qualityClass() at Uninitialized long enough to reach the old budget. Pinning this needs a
// rig that boots cold, which this chunk did not build. The gap is real and is named in
// DEFECTS1-COMPLETED.md rather than hidden behind a test that passes for the wrong reason —
// an earlier draft of exactly that test is what M21 exposed.
