// Adversarial tests for sim/hostile/latency_hostility.hpp. Pins: the EXACT delay
// contract (output(t) == input(t − L) once the ring spans L), startup staleness,
// zero-latency identity, and the two consumer-visible truths the survival story
// needs: a rotating robot's reported heading trails by ω·L (measured against the
// derived number), and the odometry corruption from stale-heading pairing is
// BOUNDED and RECOVERS when the motion stops — latency delays, it does not destroy.

#include "doctest.h"

#include <cmath>

#include "shulib/kinematics/x_drive.hpp"
#include "shulib/localization/pilons_odometry.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/sim/hostile/latency_hostility.hpp"
#include "shulib/sim/rng.hpp"
#include "shulib/sim/scenario.hpp"
#include "shulib/units/quantity.hpp"

using shulib::kinematics::xDrive;
using shulib::localization::PilonsOdometry;
using shulib::math::Angle;
using shulib::math::ChassisSpeeds;
using shulib::math::Pose2d;
using shulib::sim::GpsTruth;
using shulib::sim::LatencyHostileConfig;
using shulib::sim::LatencyHostileModel;
using shulib::sim::Rng;
using shulib::sim::SimHarness;
using shulib::sim::SimHarnessConfig;
using shulib::units::AngleDim;
using shulib::units::AngularVelocity;
using shulib::units::Length;
using shulib::units::Time;
using shulib::units::Velocity;

TEST_CASE("hostile latency: output(t) equals input(t - L) exactly, per channel") {
    LatencyHostileConfig cfg;
    cfg.imuLatency = Time{0.03};
    cfg.gpsLatency = Time{0.05};
    cfg.trackingEncoderLatency = Time{0.02};
    LatencyHostileModel m{cfg};
    Rng rng{1};

    // feed a recognizable ramp at 10 ms cadence; after startup, expect a 3-sample
    // (30 ms) lag on the IMU, 5 on GPS, 2 on the tracking encoder — EXACT values.
    for (int i = 0; i <= 20; ++i) {
        const Time t{0.01 * i};
        const Angle h = m.imuHeading(Angle::radians(0.01 * i), t, rng);
        const double rate = m.imuYawRate(AngularVelocity{1.0 * i}, t, rng).value();
        const GpsTruth g = m.gps(
            GpsTruth{Pose2d{Length{1.0 * i}, Length{0.0}, Angle{}}, Length{1.0}, true}, t, rng);
        const double enc = m.trackingEncoderPosition(0, AngleDim{0.1 * i}, t, rng).value();
        if (i >= 5) {  // past every channel's startup
            CHECK(h.radians() == doctest::Approx(0.01 * (i - 3)).epsilon(1e-12));
            CHECK(rate == doctest::Approx(1.0 * (i - 3)).epsilon(1e-12));
            CHECK(g.pose.x().value() == doctest::Approx(1.0 * (i - 5)).epsilon(1e-12));
            CHECK(enc == doctest::Approx(0.1 * (i - 2)).epsilon(1e-12));
        }
    }
}

TEST_CASE("hostile latency: before the ring spans L, the OLDEST sample is served (stale, not absent)") {
    LatencyHostileConfig cfg;
    cfg.imuLatency = Time{1.0};  // far longer than the samples pushed
    LatencyHostileModel m{cfg};
    Rng rng{1};
    const Angle first = m.imuHeading(Angle::degrees(11.0), Time{0.00}, rng);
    CHECK(first.approxEqual(Angle::degrees(11.0), 1e-12));  // the only sample IS the oldest
    const Angle later = m.imuHeading(Angle::degrees(90.0), Time{0.01}, rng);
    CHECK(later.approxEqual(Angle::degrees(11.0), 1e-12));  // still serving the boot sample
}

TEST_CASE("hostile latency: zero latency is the exact identity") {
    LatencyHostileConfig cfg;
    cfg.imuLatency = Time{0.0};
    cfg.gpsLatency = Time{0.0};
    LatencyHostileModel m{cfg};
    Rng rng{1};
    for (int i = 0; i < 50; ++i) {
        const Time t{0.01 * i};
        CHECK(m.imuHeading(Angle::radians(0.02 * i), t, rng).radians() == 0.02 * i);
        CHECK(m.gps(GpsTruth{Pose2d{Length{2.0 * i}, Length{0.0}, Angle{}}, Length{1.0}, true},
                    t, rng)
                  .pose.x()
                  .value()
              == 2.0 * i);
    }
}

// ── The consumer-visible corruption: a spinning robot's reported heading trails
// truth by ω·L — measured through the PLANT, against the derived number. ──
TEST_CASE("hostile latency: reported heading trails a spinning robot by omega*L") {
    LatencyHostileConfig cfg;
    cfg.imuLatency = Time{0.02};
    LatencyHostileModel model{cfg};
    const auto kin = xDrive(Length{7.0});
    SimHarnessConfig hCfg;
    hCfg.plant.wheelFf = {.kS = 0.0, .kV = 0.17, .kA = 0.0};
    SimHarness h{kin, hCfg, nullptr, &model};

    h.commandBodyTwist(ChassisSpeeds{Velocity{0.0}, Velocity{0.0}, AngularVelocity{1.5}});
    h.runTicks(200, Time{0.01});  // 2 s of steady 1.5 rad/s spin
    const double lag = h.truePose().heading().errorTo(h.imu().heading());
    // reported is BEHIND truth: errorTo(reported) ≈ −ω·L = −0.03 rad (−1.7°)
    CHECK(lag == doctest::Approx(-1.5 * 0.02).epsilon(1e-6));
}

// ── Bounded corruption + recovery: stale-heading pairing smears odometry while
// translating+rotating, stops growing when the motion stops, and the heading
// estimate itself re-converges within L. ──
TEST_CASE("hostile latency: odometry corruption is bounded during motion and stops growing after") {
    LatencyHostileConfig cfg;
    cfg.imuLatency = Time{0.02};
    LatencyHostileModel model{cfg};
    const auto kin = xDrive(Length{7.0});
    SimHarnessConfig hCfg;
    hCfg.plant.wheelFf = {.kS = 0.0, .kV = 0.17, .kA = 0.0};
    SimHarness h{kin, hCfg, nullptr, &model};
    PilonsOdometry odom{h.imu(), h.makeForwardTrackingWheel(), h.makeLateralTrackingWheel()};

    // 3 s of simultaneous translate + rotate — the pairing-mismatch worst case.
    h.commandBodyTwist(ChassisSpeeds{Velocity{15.0}, Velocity{0.0}, AngularVelocity{1.2}});
    double errAtMotionEnd = 0.0;
    h.runTicks(300, Time{0.01}, [&](int) { odom.update(); });
    odom.update();
    errAtMotionEnd = std::hypot((odom.pose().x() - h.truePose().x()).value(),
                                (odom.pose().y() - h.truePose().y()).value());

    // The error is REAL (latency is live) but bounded: the per-tick smear is
    // ≈ v·(ω·L)·dt, so 3 s at 15 in/s under a 0.024 rad stale heading ≲ ~1.1 in.
    CHECK(errAtMotionEnd > 0.01);
    CHECK(errAtMotionEnd < 1.5);

    // Stop. The stale window drains; heading re-converges to truth within ~L…
    h.stopAllMotors();
    h.runTicks(50, Time{0.01}, [&](int) { odom.update(); });
    odom.update();
    CHECK(std::abs(h.truePose().heading().errorTo(h.imu().heading())) < 1e-9);

    // …with ONE last bounded increment — FOUND at A3: during the drain the heading
    // keeps advancing (stale samples catching up) while the wheels truthfully
    // report zero travel, so the offset correction "removes" a rotation the wheels
    // never saw → a phantom micro-translation ≤ |Δθ_drain|·|offset| ≈ ω·L·4.5″
    // ≈ 0.14 in here. Latency converts a clean stop into a small final lie.
    const double errAfterRest = std::hypot((odom.pose().x() - h.truePose().x()).value(),
                                           (odom.pose().y() - h.truePose().y()).value());
    CHECK(errAfterRest <= errAtMotionEnd + 0.2);

    // From then on the error is genuinely FROZEN: dead-reckoning keeps the residual
    // (latency corrupted the PATH, not the ongoing estimate).
    h.runTicks(100, Time{0.01}, [&](int) { odom.update(); });
    odom.update();
    const double errLater = std::hypot((odom.pose().x() - h.truePose().x()).value(),
                                       (odom.pose().y() - h.truePose().y()).value());
    CHECK(errLater == doctest::Approx(errAfterRest).epsilon(1e-9));
}

TEST_CASE("hostile latency: GPS latency delays the whole truth triple") {
    LatencyHostileConfig cfg;
    cfg.gpsLatency = Time{0.05};
    LatencyHostileModel model{cfg};
    const auto kin = xDrive(Length{7.0});
    SimHarnessConfig hCfg;
    hCfg.plant.wheelFf = {.kS = 0.0, .kV = 0.17, .kA = 0.0};
    SimHarness h{kin, hCfg, nullptr, &model};

    h.commandBodyTwist(ChassisSpeeds{Velocity{20.0}, Velocity{0.0}, AngularVelocity{0.0}});
    h.runTicks(100, Time{0.01});  // 1 s at 20 in/s
    // truth is at 20 in; the GPS shows where the robot was 50 ms (= 1 in) ago
    CHECK(h.truePose().x().value() == doctest::Approx(20.0).epsilon(1e-9));
    CHECK(h.gps().pose().x().value() == doctest::Approx(19.0).epsilon(1e-6));
}

TEST_CASE("hostile latency: rejects an out-of-range config") {
    LatencyHostileConfig bad;
    bad.imuLatency = Time{-0.01};
    CHECK_THROWS_AS((LatencyHostileModel{bad}), shulib::PreconditionError);
}
