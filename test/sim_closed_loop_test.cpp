// Closed-loop tests against the A2 plant: the first closed loops in the project.
//
// SHAPE (the honest end-to-end loop): the controller's FEEDBACK comes from the
// synthesized SENSOR path (PilonsOdometry over the fake tracking wheels + IMU) — the
// code under test reads only F4 fakes, exactly as on a robot — while the ASSERTIONS
// grade the result against ground TRUTH. Truth is never an input, only the judge.
//
// The convergence case alone would be theater (the brief: "a plant that always
// converges is not modelling anything"), so this file also proves the plant can
// MISBEHAVE: a sign-flipped gain runs away without bound, and an overdriven gain
// against the first-order lag produces a sustained limit cycle that never settles —
// a genuinely dynamic failure that only exists because the lag is real.

#include "doctest.h"

#include <algorithm>
#include <cmath>

#include "shulib/control/pid.hpp"
#include "shulib/kinematics/x_drive.hpp"
#include "shulib/localization/pilons_odometry.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/sim/scenario.hpp"
#include "shulib/units/quantity.hpp"

using shulib::control::Pid;
using shulib::control::PidConfig;
using shulib::kinematics::xDrive;
using shulib::localization::PilonsOdometry;
using shulib::math::Angle;
using shulib::math::ChassisSpeeds;
using shulib::sim::SimHarness;
using shulib::sim::SimHarnessConfig;
using shulib::units::AngularVelocity;
using shulib::units::Length;
using shulib::units::Time;
using shulib::units::Velocity;

namespace {
/// Lagged plant (τ = 0.3 s): the closed-loop cases NEED momentum — a memoryless
/// plant cannot overshoot, so it could never distinguish good gains from bad.
[[nodiscard]] SimHarnessConfig laggedConfig() {
    SimHarnessConfig cfg;
    cfg.plant.wheelFf = {.kS = 1.2, .kV = 0.17, .kA = 0.051};
    return cfg;
}
constexpr double kDt = 0.01;
}  // namespace

// ── DoD: a closed-loop Pid drives the plant to a position and HOLDS it ──
// kP = 3 on a τ = 0.3 lag gives ζ ≈ 0.53 (underdamped but decaying, poles from
// τs² + s + kP = 0) — chosen so the loop genuinely oscillates before settling;
// an overdamped pick would hide integration errors in the plant's momentum.
TEST_CASE("sim closed loop: Pid + odometry feedback converges to the target and holds") {
    const auto kin = xDrive(Length{7.0});
    SimHarness h{kin, laggedConfig()};
    PilonsOdometry odom{h.imu(), h.makeForwardTrackingWheel(), h.makeLateralTrackingWheel()};
    Pid pid{PidConfig{.kP = 3.0, .outputMin = -60.0, .outputMax = 60.0}, h.clock()};

    const double target = 24.0;
    double worstLateHoldError = 0.0;  // max |truth error| over the final second
    h.runTicks(600, Time{kDt}, [&](int tick) {
        odom.update();  // sensor-path feedback: the controller NEVER sees truth
        const double cmd = pid.update(target, odom.pose().x().value());
        h.commandBodyTwist(ChassisSpeeds{Velocity{cmd}, Velocity{0.0}, AngularVelocity{0.0}});
        if (tick >= 500) {
            worstLateHoldError =
                std::max(worstLateHoldError, std::abs(h.truePose().x().value() - target));
        }
    });
    // Graded against TRUTH: settled within 0.05" and held there for the whole last second.
    CHECK(std::abs(h.truePose().x().value() - target) < 0.05);
    CHECK(worstLateHoldError < 0.05);
    CHECK(std::abs(h.truePose().y().value()) < 1e-6);  // the x-loop must not smear sideways
}

// ── DoD: heading closed loop through the IMU path settles under 0.5° and holds ──
TEST_CASE("sim closed loop: heading Pid through the IMU settles within half a degree") {
    const auto kin = xDrive(Length{7.0});
    SimHarness h{kin, laggedConfig()};
    Pid pid{PidConfig{.kP = 4.0, .outputMin = -6.0, .outputMax = 6.0}, h.clock()};
    const Angle target = Angle::degrees(90.0);
    h.runTicks(600, Time{kDt}, [&](int) {
        // shortest signed error from the SENSOR heading (F3 wrap semantics)
        const double err = h.imu().heading().errorTo(target);
        const double cmd = pid.update(err, 0.0);  // drive the error to zero
        h.commandBodyTwist(ChassisSpeeds{Velocity{0.0}, Velocity{0.0}, AngularVelocity{cmd}});
    });
    CHECK(std::abs(h.truePose().heading().errorTo(target)) < 0.5 * Angle::kPi / 180.0);
    // A pure turn must not translate the robot (the X-drive spins in place).
    CHECK(std::abs(h.truePose().x().value()) < 1e-6);
    CHECK(std::abs(h.truePose().y().value()) < 1e-6);
}

// ── The anti-theater cases: bad gains must demonstrably FAIL ──
TEST_CASE("sim closed loop: a sign-flipped gain runs away without bound") {
    const auto kin = xDrive(Length{7.0});
    SimHarness h{kin, laggedConfig()};
    PilonsOdometry odom{h.imu(), h.makeForwardTrackingWheel(), h.makeLateralTrackingWheel()};
    Pid pid{PidConfig{.kP = -3.0, .outputMin = -60.0, .outputMax = 60.0}, h.clock()};
    const double target = 24.0;
    double errAtHalf = 0.0;
    h.runTicks(600, Time{kDt}, [&](int tick) {
        odom.update();
        const double cmd = pid.update(target, odom.pose().x().value());
        h.commandBodyTwist(ChassisSpeeds{Velocity{cmd}, Velocity{0.0}, AngularVelocity{0.0}});
        if (tick == 300) {
            errAtHalf = std::abs(h.truePose().x().value() - target);
        }
    });
    const double errAtEnd = std::abs(h.truePose().x().value() - target);
    CHECK(errAtEnd > 100.0);       // left the field entirely
    CHECK(errAtEnd > errAtHalf);   // and still getting worse — divergence, not drift
}

TEST_CASE("sim closed loop: an overdriven gain against the lag limit-cycles and never settles") {
    const auto kin = xDrive(Length{7.0});
    SimHarness h{kin, laggedConfig()};
    PilonsOdometry odom{h.imu(), h.makeForwardTrackingWheel(), h.makeLateralTrackingWheel()};
    // WHERE 20000 COMES FROM (derived, not tuned): the discrete loop is
    //     e' = e − dt·v',   v' = q·v + (1−q)·kP·e,   q = e^{−dt/τ},
    // whose eigenvalue product is exactly q < 1 — so a P velocity command on a lag is
    // stable until dt·(1−q)·kP > 2(1+q): kP ≈ 12,000 for τ = 0.3, dt = 0.01. (A first
    // attempt at kP = 500 SETTLED, correctly — logged in A2-PROGRESS.) At kP = 20,000
    // the linear loop has an eigenvalue ≈ −4.4: genuinely unstable, and the ±60 in/s
    // saturation bounds the blow-up into a sustained limit cycle. This failure mode
    // exists ONLY because the lag dynamics are real — the anti-theater proof.
    Pid pid{PidConfig{.kP = 20000.0, .outputMin = -60.0, .outputMax = 60.0}, h.clock()};
    const double target = 24.0;
    double lateAmplitude = 0.0;   // max |truth error| in the final 2 s
    int lateCrossings = 0;        // sign changes of the error in the final 2 s
    bool lateSaturated = false;   // the command still slamming the ±60 rail late in the run
    double prevErr = 0.0;
    h.runTicks(1000, Time{kDt}, [&](int tick) {
        odom.update();
        const double cmd = pid.update(target, odom.pose().x().value());
        h.commandBodyTwist(ChassisSpeeds{Velocity{cmd}, Velocity{0.0}, AngularVelocity{0.0}});
        const double err = h.truePose().x().value() - target;
        if (tick >= 800) {
            lateAmplitude = std::max(lateAmplitude, std::abs(err));
            if (err * prevErr < 0.0) {
                ++lateCrossings;
            }
            lateSaturated = lateSaturated || std::abs(cmd) >= 60.0;
        }
        prevErr = err;
    });
    // What overdriven P + saturation + lag REALLY does (first attempt asserted 5-inch
    // swings and was wrong — the lag low-passes the bang-bang; logged in A2-PROGRESS):
    // sustained high-frequency CHATTER. 10 s in, the loop still misses by ~3 orders of
    // magnitude more than the good-gain hold (< 5e-5), keeps crossing the target, and
    // the command still slams the rail — it will never settle.
    CHECK(lateAmplitude > 0.02);
    CHECK(lateCrossings >= 4);
    CHECK(lateSaturated);
}
