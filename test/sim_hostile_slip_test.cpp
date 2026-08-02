// Adversarial tests for sim/hostile/slip_hostility.hpp. The brief's required slip
// signature — "encoders overcount while truth undershoots; odometry error grows in
// the predicted direction" — is pinned at the plant level for DRIVE encoders, and
// the architecture's answer (tracking-wheel odometry rides through drive slip) is
// pinned right next to it: both halves of A2's D9 encoder-reads-spin decision,
// finally exercised by real slip. Closed-loop convergence THROUGH a slip window
// proves feedback contains what open-loop cannot.

#include "doctest.h"

#include <cmath>
#include <numbers>

#include "shulib/control/pid.hpp"
#include "shulib/kinematics/x_drive.hpp"
#include "shulib/localization/pilons_odometry.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/sim/hostile/slip_hostility.hpp"
#include "shulib/sim/rng.hpp"
#include "shulib/sim/scenario.hpp"
#include "shulib/units/quantity.hpp"

using shulib::control::Pid;
using shulib::control::PidConfig;
using shulib::kinematics::xDrive;
using shulib::localization::PilonsOdometry;
using shulib::math::ChassisSpeeds;
using shulib::sim::Rng;
using shulib::sim::SimHarness;
using shulib::sim::SimHarnessConfig;
using shulib::sim::SlipHostileConfig;
using shulib::sim::SlipHostileModel;
using shulib::sim::SlipWindow;
using shulib::units::AngularVelocity;
using shulib::units::Length;
using shulib::units::Time;
using shulib::units::Velocity;

namespace {
[[nodiscard]] SimHarnessConfig laggedConfig() {
    SimHarnessConfig cfg;
    cfg.plant.wheelFf = {.kS = 1.2, .kV = 0.17, .kA = 0.051};  // τ = 0.3 s — real ramps
    return cfg;
}
}  // namespace

TEST_CASE("hostile slip: hook-level — a hard spin-up slips by slipRetain, steady state does not") {
    SlipHostileConfig cfg;
    cfg.accelThresholdInPerS2 = 80.0;
    cfg.slipRetain = 0.7;
    SlipHostileModel m{cfg};
    Rng rng{1};

    // baseline call (no history → no accel verdict, full grip)
    CHECK(m.wheelMotionVelocity(0, Velocity{0.0}, Time{0.00}, rng).value() == 0.0);
    // 0 → 30 in/s in 10 ms = 3000 in/s² ≫ 80 → traction breaks
    CHECK(m.wheelMotionVelocity(0, Velocity{30.0}, Time{0.01}, rng).value()
          == doctest::Approx(30.0 * 0.7));
    // holding 30 → accel 0 → grip restored
    CHECK(m.wheelMotionVelocity(0, Velocity{30.0}, Time{0.02}, rng).value()
          == doctest::Approx(30.0));
    // gentle change (0.5 in/s per 10 ms = 50 in/s² < 80) keeps grip
    CHECK(m.wheelMotionVelocity(0, Velocity{30.5}, Time{0.03}, rng).value()
          == doctest::Approx(30.5));
}

TEST_CASE("hostile slip: wheels have independent histories (one wheel's launch cannot slip another)") {
    SlipHostileModel m{};
    Rng rng{1};
    (void)m.wheelMotionVelocity(0, Velocity{0.0}, Time{0.00}, rng);
    (void)m.wheelMotionVelocity(1, Velocity{20.0}, Time{0.00}, rng);
    // wheel 0 launches hard; wheel 1 is steady at the same instant
    CHECK(m.wheelMotionVelocity(0, Velocity{25.0}, Time{0.01}, rng).value()
          == doctest::Approx(25.0 * 0.7));
    CHECK(m.wheelMotionVelocity(1, Velocity{20.0}, Time{0.01}, rng).value()
          == doctest::Approx(20.0));
}

TEST_CASE("hostile slip: a slip window grips only the masked wheels for exactly its span") {
    SlipHostileConfig cfg;
    cfg.accelThresholdInPerS2 = 1e9;  // isolate the window
    cfg.windows = {SlipWindow{Time{1.0}, Time{2.0}, 0.4, 0b0011u}};  // wheels 0 and 1
    SlipHostileModel m{cfg};
    Rng rng{1};
    CHECK(m.wheelMotionVelocity(0, Velocity{10.0}, Time{0.5}, rng).value() == doctest::Approx(10.0));
    CHECK(m.wheelMotionVelocity(0, Velocity{10.0}, Time{1.5}, rng).value() == doctest::Approx(4.0));
    CHECK(m.wheelMotionVelocity(1, Velocity{10.0}, Time{1.5}, rng).value() == doctest::Approx(4.0));
    CHECK(m.wheelMotionVelocity(2, Velocity{10.0}, Time{1.5}, rng).value() == doctest::Approx(10.0));
    CHECK(m.wheelMotionVelocity(0, Velocity{10.0}, Time{2.0}, rng).value() == doctest::Approx(10.0));
}

// ── THE required slip signature, end to end: drive encoders overcount while truth
// undershoots (predicted direction), and the tracking wheels tell the truth. ──
TEST_CASE("hostile slip: encoders overcount, truth undershoots, tracking odometry survives") {
    SlipHostileConfig cfg;
    cfg.accelThresholdInPerS2 = 1e9;  // window only: exact accounting
    // Window edges deliberately OFF the tick grid (t accumulates 0.01 with fp
    // rounding; an on-grid edge lets one boundary tick land on either side):
    cfg.windows = {SlipWindow{Time{0.995}, Time{2.995}, 0.6, 0u}};  // 200 ticks, keep 60%
    SlipHostileModel model{cfg};

    const auto kin = xDrive(Length{7.0});
    SimHarnessConfig hCfg;
    hCfg.plant.wheelFf = {.kS = 1.2, .kV = 0.17, .kA = 0.0};      // memoryless: exact numbers
    SimHarness h{kin, hCfg, nullptr, &model};
    PilonsOdometry odom{h.imu(), h.makeForwardTrackingWheel(), h.makeLateralTrackingWheel()};

    h.commandBodyTwist(ChassisSpeeds{Velocity{20.0}, Velocity{0.0}, AngularVelocity{0.0}});
    double worstOdomErr = 0.0;
    h.runTicks(400, Time{0.01}, [&](int) {
        odom.update();
        worstOdomErr = std::max(worstOdomErr, std::hypot(
            (odom.pose().x() - h.truePose().x()).value(),
            (odom.pose().y() - h.truePose().y()).value()));
    });
    odom.update();  // fold the last tick

    // truth: 4 s commanded at 20 in/s, but 2 s moved at only 60% → 40 + 24 = 64 in
    const double trueX = h.truePose().x().value();
    CHECK(trueX == doctest::Approx(64.0).epsilon(1e-6));

    // drive encoders integrated the SPIN: 80 in of wheel travel — they overcount
    const double wheelR = 3.25 / 2.0;
    const double wheelSpeed = 20.0 / std::numbers::sqrt2;  // per-wheel surface speed
    const double encImpliedWheelTravel = h.motor(2).position().value() * wheelR;
    CHECK(encImpliedWheelTravel == doctest::Approx(wheelSpeed * 4.0).epsilon(1e-6));
    CHECK(encImpliedWheelTravel > wheelSpeed * 3.0);  // ≫ the slipped reality (predicted direction)

    // the unpowered tracking wheels measured ACTUAL motion → odometry rode through
    CHECK(worstOdomErr < 1e-6);
    CHECK(odom.pose().x().value() == doctest::Approx(trueX).epsilon(1e-9));
}

TEST_CASE("hostile slip: with a lagged plant the launch slips, then grip returns as accel decays") {
    SlipHostileModel model{};  // defaults: accel-triggered, threshold 80, retain 0.7
    const auto kin = xDrive(Length{7.0});
    SimHarness h{kin, laggedConfig(), nullptr, &model};

    h.commandBodyTwist(ChassisSpeeds{Velocity{50.0}, Velocity{0.0}, AngularVelocity{0.0}});
    // Tick 1 BASELINES each wheel's spin history (no accel verdict yet — the model
    // cannot judge a jump it has not seen start); tick 2's Δspin/dt ≈ 114 in/s²
    // exceeds the 80 in/s² threshold → slipping. Wheel 0 spins NEGATIVE for +x on
    // an X-drive, so magnitudes are compared.
    h.plant().step(Time{0.01});
    h.plant().step(Time{0.01});
    const double earlyMotion = h.trueBodyTwist().vx().value();
    const double earlySpin =
        std::abs(h.plant().trueWheelSpin()[0].value()) * std::numbers::sqrt2;
    CHECK(earlyMotion < earlySpin * 0.75);  // body trails the spinning wheels

    // late in the ramp accel has decayed below threshold → grip restored
    h.runTicks(300, Time{0.01});
    const double lateMotion = h.trueBodyTwist().vx().value();
    const double lateSpin =
        std::abs(h.plant().trueWheelSpin()[0].value()) * std::numbers::sqrt2;
    CHECK(lateMotion == doctest::Approx(lateSpin).epsilon(1e-9));
}

TEST_CASE("hostile slip: a closed loop through a slip window still converges and holds") {
    SlipHostileConfig cfg;
    cfg.windows = {SlipWindow{Time{0.5}, Time{2.0}, 0.5, 0u}};  // slip mid-approach
    SlipHostileModel model{cfg};
    const auto kin = xDrive(Length{7.0});
    SimHarness h{kin, laggedConfig(), nullptr, &model};
    PilonsOdometry odom{h.imu(), h.makeForwardTrackingWheel(), h.makeLateralTrackingWheel()};
    Pid pid{PidConfig{.kP = 3.0, .outputMin = -60.0, .outputMax = 60.0}, h.clock()};

    const double target = 24.0;
    double worstLateHold = 0.0;
    h.runTicks(600, Time{0.01}, [&](int tick) {
        odom.update();
        const double cmd = pid.update(target, odom.pose().x().value());
        h.commandBodyTwist(ChassisSpeeds{Velocity{cmd}, Velocity{0.0}, AngularVelocity{0.0}});
        if (tick >= 500) {
            worstLateHold = std::max(worstLateHold,
                                     std::abs(h.truePose().x().value() - target));
        }
    });
    CHECK(std::abs(h.truePose().x().value() - target) < 0.05);  // feedback beat the slip
    CHECK(worstLateHold < 0.05);
}

TEST_CASE("hostile slip: rejects an out-of-range config") {
    SlipHostileConfig bad;
    bad.slipRetain = 1.5;
    CHECK_THROWS_AS((SlipHostileModel{bad}), shulib::PreconditionError);
    SlipHostileConfig bad2;
    bad2.windows = {SlipWindow{Time{2.0}, Time{1.0}, 0.5, 0u}};
    CHECK_THROWS_AS((SlipHostileModel{bad2}), shulib::PreconditionError);
}
