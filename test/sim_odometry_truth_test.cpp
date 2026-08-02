// The first end-to-end localization proof in the project (chunk A2 DoD): the REAL
// estimator stack — TrackingWheel → PilonsOdometry → Localizer — fed ONLY the
// synthesized F4 sensors, measured against the plant's independent ground truth.
//
// Why these bounds are honest (and deliberately tiny): the sensors are PERFECT at A2
// (identity degradation), the per-tick twist is constant (ZOH), and arcStep is exact
// for a constant twist — so the only error sources are the RK4-vs-arcStep gap
// (~1e-15/tick at real tick sizes) and floating-point accumulation. The documented
// bound, 1e-6 in over multi-second runs, is therefore ~6 orders above the expected
// error but ~6 below anything physical: it FAILS on any logic defect (a sign, an
// offset, a chord factor, a wrap) while never flaking on rounding. REALISTIC error
// bounds arrive with A3's noise models; certifying F2 accuracy on perfect sensors
// would be theater, which is exactly why the accuracy_spec_test.cpp acceptance stubs
// stay skipped until then.
//
// The mis-calibration case is the anti-agreeable-harness proof: the harness must be
// able to DETECT estimator error, quantitatively, or Phase E measures nothing. A 2%
// wheel-diameter lie must produce the analytically predicted 2% odometry error.

#include "doctest.h"

#include <cmath>

#include "shulib/localization/complementary_fusion.hpp"
#include "shulib/localization/localizer.hpp"
#include "shulib/localization/pilons_odometry.hpp"
#include "shulib/localization/tracking_wheel.hpp"
#include "shulib/kinematics/x_drive.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/sim/scenario.hpp"
#include "shulib/units/quantity.hpp"

using shulib::kinematics::xDrive;
using shulib::localization::ComplementaryFusion;
using shulib::localization::Localizer;
using shulib::localization::PilonsOdometry;
using shulib::localization::TrackingWheel;
using shulib::math::ChassisSpeeds;
using shulib::sim::randomBodyTwist;
using shulib::sim::SimHarness;
using shulib::sim::SimHarnessConfig;
using shulib::units::AngularVelocity;
using shulib::units::Length;
using shulib::units::Time;
using shulib::units::Velocity;

namespace {
[[nodiscard]] SimHarnessConfig instantConfig() {
    SimHarnessConfig cfg;
    cfg.plant.wheelFf = {.kS = 1.2, .kV = 0.17, .kA = 0.0};
    return cfg;
}
constexpr double kDt = 0.01;
constexpr double kBoundIn = 1e-6;  // the documented perfect-sensor tracking bound

[[nodiscard]] double posError(const shulib::math::Pose2d& a, const shulib::math::Pose2d& b) {
    return std::hypot((a.x() - b.x()).value(), (a.y() - b.y()).value());
}

/// A deliberately rich 8-segment script: forward, arc, strafe, spin, holonomic
/// drift, reversal — every motion class odometry must survive, in one run.
[[nodiscard]] ChassisSpeeds scriptedTwist(int tick) {
    const int phase = tick / 100;  // 1 s per segment
    switch (phase) {
        case 0: return {Velocity{20.0}, Velocity{0.0}, AngularVelocity{0.0}};    // forward
        case 1: return {Velocity{15.0}, Velocity{0.0}, AngularVelocity{1.2}};    // CCW arc
        case 2: return {Velocity{0.0}, Velocity{14.0}, AngularVelocity{0.0}};    // pure strafe
        case 3: return {Velocity{0.0}, Velocity{0.0}, AngularVelocity{-1.5}};    // CW spin
        case 4: return {Velocity{10.0}, Velocity{-8.0}, AngularVelocity{0.8}};   // holonomic
        case 5: return {Velocity{-18.0}, Velocity{0.0}, AngularVelocity{0.0}};   // reversal
        case 6: return {Velocity{-6.0}, Velocity{9.0}, AngularVelocity{-0.7}};   // holonomic 2
        default: return {Velocity{0.0}, Velocity{0.0}, AngularVelocity{0.0}};    // brake
    }
}
}  // namespace

// ── DoD: PilonsOdometry tracks truth within the documented bound, EVERY tick of a
// rich multi-second trajectory (not just the endpoint — a transient excursion that
// happened to cancel by the end must still fail) ──
TEST_CASE("sim odometry: PilonsOdometry tracks truth through a rich 8s script, every tick") {
    const auto kin = xDrive(Length{7.0});
    SimHarness h{kin, instantConfig()};
    PilonsOdometry odom{h.imu(), h.makeForwardTrackingWheel(), h.makeLateralTrackingWheel()};
    double worst = 0.0;
    h.runTicks(800, Time{kDt}, [&](int tick) {
        odom.update();
        worst = std::max(worst, posError(odom.pose(), h.truePose()));
        CHECK(odom.pose().heading().approxEqual(h.truePose().heading(), 1e-12));  // IMU-owned
        h.commandBodyTwist(scriptedTwist(tick));
    });
    CHECK(worst < kBoundIn);
    // sanity: the run actually went somewhere (a stationary robot proves nothing)
    CHECK(std::abs(h.truePose().x().value()) + std::abs(h.truePose().y().value()) > 10.0);
}

// ── Sweep, don't hand-pick: seeded random trajectories, same bound at every tick ──
TEST_CASE("sim odometry: seeded random 5s trajectories stay inside the bound (3 seeds)") {
    const auto kin = xDrive(Length{7.0});
    for (std::uint64_t seed : {7ULL, 1234ULL, 987654321ULL}) {
        SimHarnessConfig cfg = instantConfig();
        cfg.plant.seed = seed;
        SimHarness h{kin, cfg};
        PilonsOdometry odom{h.imu(), h.makeForwardTrackingWheel(), h.makeLateralTrackingWheel()};
        ChassisSpeeds cmd{};
        double worst = 0.0;
        h.runTicks(500, Time{kDt}, [&](int tick) {
            odom.update();
            worst = std::max(worst, posError(odom.pose(), h.truePose()));
            if (tick % 50 == 0) {  // a new random twist every half second
                cmd = randomBodyTwist(h.rng(), 15.0, 1.5);
            }
            h.commandBodyTwist(cmd);
        });
        CHECK(worst < kBoundIn);
    }
}

// ── The full Localizer (odom + IMU + empty correctors behind ComplementaryFusion)
// rides the same sensors and must track the same truth — and its quality flags must
// tell the honest dead-reckoning story (DeadReckon early, Degraded past the drift
// horizon with no fixes arriving) ──
TEST_CASE("sim odometry: the fused Localizer dead-reckons along truth with honest quality") {
    const auto kin = xDrive(Length{7.0});
    SimHarness h{kin, instantConfig()};
    PilonsOdometry odom{h.imu(), h.makeForwardTrackingWheel(), h.makeLateralTrackingWheel()};
    ComplementaryFusion fusion{};
    Localizer loc{h.clock(), h.imu(), odom, fusion};
    bool sawDeadReckon = false;
    double worst = 0.0;
    h.runTicks(800, Time{kDt}, [&](int tick) {
        loc.update();
        worst = std::max(worst, posError(loc.pose(), h.truePose()));
        if (tick == 5) {
            sawDeadReckon = (loc.qualityClass() == Localizer::Quality::DeadReckon);
        }
        h.commandBodyTwist(scriptedTwist(tick));
    });
    CHECK(worst < kBoundIn);
    CHECK(loc.pose().heading().approxEqual(h.truePose().heading(), 1e-12));
    CHECK(sawDeadReckon);            // early: healthy dead-reckoning
    CHECK(loc.isDeadReckoning());    // no corrector ever fired
    // ~100+ inches traveled with no absolute fix: trust MUST have degraded (M2 semantics).
    CHECK(loc.qualityClass() == Localizer::Quality::Degraded);
    CHECK(loc.quality() < 1.0);
}

// ── THE ANTI-AGREEABLE PROOF: a mis-calibrated estimator must be CAUGHT, and by the
// analytically predicted amount. Wheels configured 2% fat over-report travel by 2%;
// after 50 true inches the odometry must read ~51 — an error the truth harness sees
// as ~1.0 in. If this test can't see that error, Phase E measures nothing. ──
TEST_CASE("sim odometry: a 2% wheel-diameter mis-calibration is detected, quantitatively") {
    const auto kin = xDrive(Length{7.0});
    SimHarness h{kin, instantConfig()};
    // The estimator BELIEVES the wheels are 2.04" — the plant synthesizes with 2.00".
    TrackingWheel fatForward = TrackingWheel::forward(h.forwardEncoder(), Length{2.04}, Length{-3.0});
    TrackingWheel fatLateral = TrackingWheel::lateral(h.lateralEncoder(), Length{2.04}, Length{-4.5});
    PilonsOdometry odom{h.imu(), fatForward, fatLateral};
    h.commandBodyTwist(ChassisSpeeds{Velocity{25.0}, Velocity{0.0}, AngularVelocity{0.0}});
    h.runTicks(200, Time{kDt}, [&](int) { odom.update(); });
    odom.update();  // fold in the final tick's travel
    const double trueX = h.truePose().x().value();     // 50 by construction
    const double odomX = odom.pose().x().value();
    CHECK(trueX == doctest::Approx(50.0).epsilon(1e-9));
    CHECK(odomX == doctest::Approx(51.0).epsilon(1e-6));         // the PREDICTED 2% lie
    CHECK(std::abs(odomX - trueX) > 0.5);                        // and the harness SEES it
}

// ── The 60-second dead-reckon run: the M2 acceptance stub's system now exists.
// SCOPE, stated honestly: with perfect sensors this proves the LOGIC holds for a
// full match duration (no drift from the integration itself); it says NOTHING about
// F2's <1° on real hardware — that claim needs A3's modeled IMU drift, which is why
// the [acceptance][M2] stub in accuracy_spec_test.cpp remains skipped. ──
TEST_CASE("sim odometry: a full 60s dead-reckon run accumulates no integration drift") {
    const auto kin = xDrive(Length{7.0});
    SimHarness h{kin, instantConfig()};
    PilonsOdometry odom{h.imu(), h.makeForwardTrackingWheel(), h.makeLateralTrackingWheel()};
    ChassisSpeeds cmd{};
    double worst = 0.0;
    h.runTicks(6000, Time{kDt}, [&](int tick) {
        odom.update();
        worst = std::max(worst, posError(odom.pose(), h.truePose()));
        if (tick % 200 == 0) {
            cmd = randomBodyTwist(h.rng(), 15.0, 1.5);
        }
        h.commandBodyTwist(cmd);
    });
    odom.update();        // fold in the final tick (the loop updates BEFORE each step,
                          // so odometry would otherwise trail truth by one tick here)
    worst = std::max(worst, posError(odom.pose(), h.truePose()));
    CHECK(worst < 1e-5);  // 60s bound: one order looser than the 8s bound, still ~5
                          // orders below anything physical — pure numerical headroom
    CHECK(odom.pose().heading().approxEqual(h.truePose().heading(), 1e-12));
    CHECK(!odom.lastDeltaImplausible());
}
