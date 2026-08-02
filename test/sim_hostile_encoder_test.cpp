// Adversarial tests for sim/hostile/encoder_hostility.hpp. Pins: the quantization
// grid (readings are exact multiples of the tick step) and its TELESCOPING bound
// (cumulative delta error ≤ one step — the property that keeps quantization benign
// for odometry), channel-selective freezes, the sentinel breach window (+∞ =
// PROS_ERR_F, the deliberate F4-contract violation), the permanent bump/skid lie,
// and end-to-end: odometry error under quantization stays bounded near one tick.

#include "doctest.h"

#include <cmath>

#include "shulib/kinematics/x_drive.hpp"
#include "shulib/localization/pilons_odometry.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/sim/hostile/encoder_hostility.hpp"
#include "shulib/sim/rng.hpp"
#include "shulib/sim/scenario.hpp"
#include "shulib/units/quantity.hpp"

using shulib::kinematics::xDrive;
using shulib::localization::PilonsOdometry;
using shulib::math::Angle;
using shulib::math::ChassisSpeeds;
using shulib::sim::EncoderHostileConfig;
using shulib::sim::EncoderHostileModel;
using shulib::sim::Rng;
using shulib::sim::SimHarness;
using shulib::sim::SimHarnessConfig;
using shulib::units::AngleDim;
using shulib::units::AngularVelocity;
using shulib::units::Length;
using shulib::units::Time;
using shulib::units::Velocity;

namespace {
constexpr double kTwoPi = 2.0 * Angle::kPi;

[[nodiscard]] bool onGrid(double value, double ticksPerRev) {
    const double step = kTwoPi / ticksPerRev;
    const double ticks = value / step;
    return std::abs(ticks - std::round(ticks)) < 1e-6;
}
}  // namespace

TEST_CASE("hostile encoder: readings snap to the device tick grid, error <= half a step") {
    EncoderHostileModel m{};
    Rng rng{1};
    const double driveStep = kTwoPi / 900.0;
    const double trackStep = kTwoPi / 36000.0;
    for (int i = 0; i < 500; ++i) {
        const double shaft = 0.0173 * i;  // deliberately off-grid truth
        const double d = m.driveEncoderPosition(0, AngleDim{shaft}, Time{0.01 * i}, rng).value();
        const double t = m.trackingEncoderPosition(0, AngleDim{shaft}, Time{0.01 * i}, rng).value();
        CHECK(onGrid(d, 900.0));
        CHECK(onGrid(t, 36000.0));
        CHECK(std::abs(d - shaft) <= 0.5 * driveStep + 1e-12);
        CHECK(std::abs(t - shaft) <= 0.5 * trackStep + 1e-12);
    }
}

TEST_CASE("hostile encoder: quantization TELESCOPES — cumulative delta error is one step, not a walk") {
    EncoderHostileModel m{};
    Rng rng{1};
    double accumulated = 0.0;  // what a delta-consumer (TrackingWheel) would total
    double prev = m.trackingEncoderPosition(1, AngleDim{0.0}, Time{0.0}, rng).value();
    double trueShaft = 0.0;
    for (int i = 1; i <= 10000; ++i) {
        trueShaft += 0.0129;  // ~7.4 rad/s at 100 Hz, off-grid on purpose
        const double q =
            m.trackingEncoderPosition(1, AngleDim{trueShaft}, Time{0.01 * i}, rng).value();
        accumulated += q - prev;
        prev = q;
    }
    // 129 rad of travel; the total error must be bounded by ONE tick step.
    CHECK(std::abs(accumulated - trueShaft) <= kTwoPi / 36000.0 + 1e-12);
}

TEST_CASE("hostile encoder: freeze events stop the selected channel, and only it") {
    EncoderHostileConfig cfg;
    cfg.driveFreezeAt = Time{1.0};
    cfg.driveFreezeWheel = 2;
    cfg.trackingFreezeAt = Time{2.0};
    cfg.trackingFreezeIndex = -1;  // all tracking wheels
    EncoderHostileModel m{cfg};
    Rng rng{1};

    // pre-freeze: everything tracks
    (void)m.driveEncoderPosition(2, AngleDim{1.0}, Time{0.99}, rng);
    const double frozenAtValue =
        m.driveEncoderPosition(2, AngleDim{1.0}, Time{0.999}, rng).value();
    (void)m.trackingEncoderPosition(0, AngleDim{5.0}, Time{1.99}, rng);

    // wheel 2 frozen from t=1.0; wheel 0 unaffected
    CHECK(m.driveEncoderPosition(2, AngleDim{9.9}, Time{1.5}, rng).value()
          == doctest::Approx(frozenAtValue));
    CHECK(m.driveEncoderPosition(2, AngleDim{50.0}, Time{60.0}, rng).value()
          == doctest::Approx(frozenAtValue));
    CHECK(m.driveEncoderPosition(0, AngleDim{7.0}, Time{1.5}, rng).value()
          == doctest::Approx(7.0).epsilon(1e-3));

    // ALL tracking wheels frozen from t=2.0
    const double t0 = m.trackingEncoderPosition(0, AngleDim{99.0}, Time{2.5}, rng).value();
    const double t1 = m.trackingEncoderPosition(1, AngleDim{99.0}, Time{2.5}, rng).value();
    CHECK(t0 == doctest::Approx(5.0).epsilon(1e-3));  // stuck at its last emission
    // wheel 1 was never read before its freeze: it latches the FIRST value it sees
    // (the frozen device's cache is whatever it last computed) and sticks there.
    CHECK(t1 == doctest::Approx(99.0).epsilon(1e-3));
    CHECK(m.trackingEncoderPosition(1, AngleDim{123.0}, Time{9.0}, rng).value()
          == doctest::Approx(t1));
}

TEST_CASE("hostile encoder: the sentinel breach emits PROS_ERR_F (+inf) for exactly its window") {
    EncoderHostileConfig cfg;
    cfg.sentinelOnTracking = true;
    cfg.sentinelIndex = 1;
    cfg.sentinelAt = Time{2.0};
    cfg.sentinelFor = Time{0.05};
    EncoderHostileModel m{cfg};
    Rng rng{1};

    CHECK(std::isfinite(m.trackingEncoderPosition(1, AngleDim{1.0}, Time{1.99}, rng).value()));
    for (double t : {2.0, 2.02, 2.049}) {
        const double v = m.trackingEncoderPosition(1, AngleDim{1.0}, Time{t}, rng).value();
        CHECK(std::isinf(v));  // the leaked PROS sentinel
        CHECK(v > 0.0);
        // the OTHER tracking wheel keeps working through the breach
        CHECK(std::isfinite(m.trackingEncoderPosition(0, AngleDim{1.0}, Time{t}, rng).value()));
    }
    CHECK(std::isfinite(m.trackingEncoderPosition(1, AngleDim{1.1}, Time{2.05}, rng).value()));
}

TEST_CASE("hostile encoder: a bump/skid permanently offsets the tracking count") {
    EncoderHostileConfig cfg;
    cfg.bumpAt = Time{1.0};
    cfg.bumpIndex = 0;
    cfg.bumpShaftRad = 2.5;
    EncoderHostileModel m{cfg};
    Rng rng{1};
    CHECK(m.trackingEncoderPosition(0, AngleDim{4.0}, Time{0.5}, rng).value()
          == doctest::Approx(4.0).epsilon(1e-4));
    for (double t : {1.0, 5.0, 60.0}) {
        CHECK(m.trackingEncoderPosition(0, AngleDim{4.0}, Time{t}, rng).value()
              == doctest::Approx(6.5).epsilon(1e-4));  // truth + the skid, forever
    }
    CHECK(m.trackingEncoderPosition(1, AngleDim{4.0}, Time{5.0}, rng).value()
          == doctest::Approx(4.0).epsilon(1e-4));  // the other wheel untouched
}

// ── End-to-end: real tick resolution does not meaningfully hurt the odometry —
// the error over a rich 8 s script stays bounded near ONE tracking tick of travel
// (the telescoping property, surviving arcStep + offset correction). ──
TEST_CASE("hostile encoder: odometry under quantization stays within a few tick-steps of truth") {
    EncoderHostileConfig cfg;  // quantization only (events off)
    EncoderHostileModel model{cfg};
    const auto kin = xDrive(Length{7.0});
    SimHarnessConfig hCfg;
    hCfg.plant.wheelFf = {.kS = 1.2, .kV = 0.17, .kA = 0.0};
    SimHarness h{kin, hCfg, nullptr, &model};
    PilonsOdometry odom{h.imu(), h.makeForwardTrackingWheel(), h.makeLateralTrackingWheel()};

    double worst = 0.0;
    h.runTicks(800, Time{0.01}, [&](int tick) {
        odom.update();
        worst = std::max(worst, std::hypot((odom.pose().x() - h.truePose().x()).value(),
                                           (odom.pose().y() - h.truePose().y()).value()));
        const int phase = tick / 100;
        const double vx = (phase % 2 == 0) ? 18.0 : -6.0;
        const double vy = (phase % 3 == 0) ? 0.0 : 9.0;
        const double om = (phase >= 4) ? 1.0 : -0.5;
        h.commandBodyTwist(ChassisSpeeds{Velocity{vx}, Velocity{vy}, AngularVelocity{om}});
    });
    // one tracking tick = (2π/36000)·1″ radius = 1.75e-4 in of travel; allow a few
    // ticks of transient mismatch across two wheels + offset correction.
    const double tickTravel = (kTwoPi / 36000.0) * 1.0;
    CHECK(worst < 10.0 * tickTravel);
    CHECK(worst > 0.0);  // and the pathology is LIVE (identity would read 0 here)
}

TEST_CASE("hostile encoder: rejects an out-of-range config") {
    EncoderHostileConfig bad;
    bad.driveTicksPerRev = 0.0;
    CHECK_THROWS_AS((EncoderHostileModel{bad}), shulib::PreconditionError);
}
