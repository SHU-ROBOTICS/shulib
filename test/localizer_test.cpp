// Adversarial tests for Localizer — the fused-estimate orchestrator. The keystone is the
// HEADING-AUTHORITY invariant: a corrector that lies about heading must never rotate the fused
// pose (heading is IMU-owned, re-stamped as the last write). Other lenses: dead-reckon default,
// the gated nudge applied through the full stack, twist finite-difference + its dt guards, the
// quality scalar/flags, invalid-proposal screening, and setPose.

#include <array>
#include <cmath>
#include <limits>
#include <span>

#include "doctest.h"

#include "shulib/core/check.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_imu.hpp"
#include "shulib/hal/fake/fake_rotation.hpp"
#include "shulib/localization/complementary_fusion.hpp"
#include "shulib/localization/correction.hpp"
#include "shulib/localization/fake/fake_corrector.hpp"
#include "shulib/localization/i_corrector.hpp"
#include "shulib/localization/localizer.hpp"
#include "shulib/localization/pilons_odometry.hpp"
#include "shulib/localization/tracking_wheel.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/units/quantity.hpp"

using shulib::hal::fake::FakeClock;
using shulib::hal::fake::FakeImu;
using shulib::hal::fake::FakeRotation;
using shulib::localization::ComplementaryFusion;
using shulib::localization::ComplementaryFusionConfig;
using shulib::localization::CorrectionProposal;
using shulib::localization::ICorrector;
using shulib::localization::Localizer;
using shulib::localization::LocalizerConfig;
using shulib::localization::PilonsOdometry;
using shulib::localization::TrackingWheel;
using shulib::localization::fake::FakeCorrector;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::units::AngularVelocity;
using shulib::units::Length;
using shulib::units::Time;
using shulib::units::Velocity;

namespace {
// Wires the full stack with fakes. Diameter-2 wheels (radius 1) → a shaft reading in radians is
// inches of forward travel, so motion is injected directly via fwdRot.setPosition().
struct Rig {
    FakeClock clk;
    FakeImu imu;
    FakeRotation fwdRot, latRot;
    PilonsOdometry odom;
    ComplementaryFusion fusion;
    Localizer loc;

    explicit Rig(std::span<ICorrector* const> correctors = {}, LocalizerConfig cfg = {},
                 ComplementaryFusionConfig fcfg = {})
        : odom{imu, TrackingWheel::forward(fwdRot, Length{2.0}, Length{0.0}),
                    TrackingWheel::lateral(latRot, Length{2.0}, Length{0.0})},
          fusion{fcfg},
          loc{clk, imu, odom, fusion, correctors, cfg} {}

    // Advance time by dt and set the forward-wheel cumulative travel, then fuse one tick.
    void tick(double dt, double fwdTravel) {
        clk.advance(Time{dt});
        fwdRot.setPosition(shulib::units::AngleDim{fwdTravel});
        loc.update();
    }
};
}  // namespace

TEST_CASE("Localizer: with no correctors it dead-reckons the odometry forever") {
    Rig r;
    r.tick(0.01, 0.0);   // baseline
    r.tick(0.01, 5.0);   // 5" forward
    CHECK(r.loc.pose().x().value() == doctest::Approx(5.0));
    CHECK(r.loc.isDeadReckoning());
    r.tick(0.01, 9.0);   // +4"
    CHECK(r.loc.pose().x().value() == doctest::Approx(9.0));
    CHECK(r.loc.isDeadReckoning());
}

// KEYSTONE: a corrector that reports a WRONG heading must not rotate the fused pose.
TEST_CASE("Localizer: a corrector cannot move heading — heading stays IMU-owned") {
    FakeCorrector fc{"liar"};
    std::array<ICorrector*, 1> arr{&fc};
    Rig r{arr};
    r.imu.setHeading(Angle::degrees(0.0));
    r.tick(0.01, 0.0);  // baseline, corrector still invalid (default)

    CorrectionProposal p{};
    p.valid = true;
    p.fieldPose = Pose2d{Length{2.0}, Length{0.0}, Angle::degrees(90.0)};  // claims heading 90°!
    p.confidence = 1.0;
    p.positionStdDev = Length{0.5};
    fc.setProposal(p);
    r.tick(0.05, 0.0);  // fuse with the lying corrector

    CHECK(r.loc.pose().heading().degrees() == doctest::Approx(0.0));  // IMU's 0°, NOT 90°
    CHECK(r.loc.pose().x().value() > 0.0);          // position WAS nudged toward the fix
    CHECK(r.loc.pose().x().value() < 2.0);          // but never snapped to it
    CHECK_FALSE(r.loc.isDeadReckoning());           // a correction was applied
}

TEST_CASE("Localizer: twist is the fused-pose finite-difference; omega is the IMU rate") {
    Rig r;
    r.imu.setYawRate(AngularVelocity{1.25});
    r.tick(0.05, 0.0);   // baseline (dt unhealthy on the first tick)
    r.tick(0.05, 5.0);   // 5" over 0.05s → 100 in/s
    CHECK(r.loc.twist().vx().value() == doctest::Approx(100.0));
    CHECK(r.loc.twist().vy().value() == doctest::Approx(0.0));
    CHECK(r.loc.twist().omega().value() == doctest::Approx(1.25));  // straight from the IMU
}

TEST_CASE("Localizer: dt guards keep twist finite (no clock advance, and a huge gap)") {
    Rig r;
    r.tick(0.05, 0.0);
    r.tick(0.05, 5.0);                       // vx = 100
    // no clock advance → dt == 0 → keep last linear velocity, no divide-by-zero
    r.fwdRot.setPosition(shulib::units::AngleDim{6.0});
    r.loc.update();
    CHECK(std::isfinite(r.loc.twist().vx().value()));
    CHECK(r.loc.twist().vx().value() == doctest::Approx(100.0));  // unchanged
    // huge gap (> maxDt) → linear velocity zeroed, still finite
    r.tick(1.0, 20.0);
    CHECK(std::isfinite(r.loc.twist().vx().value()));
    CHECK(r.loc.twist().vx().value() == doctest::Approx(0.0));
}

TEST_CASE("Localizer: a far (out-of-gate) corrector is ignored → still dead-reckoning") {
    FakeCorrector fc;
    std::array<ICorrector*, 1> arr{&fc};
    Rig r{arr};
    r.tick(0.01, 0.0);
    CorrectionProposal p{};
    p.valid = true;
    p.fieldPose = Pose2d{Length{100.0}, Length{0.0}, Angle{}};  // 100" off ≫ 12" gate
    p.confidence = 1.0;
    p.positionStdDev = Length{0.5};
    fc.setProposal(p);
    r.tick(0.05, 0.0);
    CHECK(r.loc.isDeadReckoning());                       // gated out → no correction applied
    CHECK(r.loc.pose().x().value() == doctest::Approx(0.0));
    CHECK(r.loc.lastCorrection().gated);
}

TEST_CASE("Localizer: quality decays while dead-reckoning and springs back on a fix") {
    FakeCorrector fc;
    std::array<ICorrector*, 1> arr{&fc};
    Rig r{arr};  // default driftHorizon = 12"
    r.tick(0.05, 0.0);   // baseline
    r.tick(0.05, 3.0);   // dead-reckon 3" → driftFrac 0.25 → quality 0.75
    CHECK(r.loc.quality() == doctest::Approx(0.75));
    CHECK(r.loc.qualityClass() == Localizer::Quality::DeadReckon);

    CorrectionProposal p{};  // a fix AT the current pose (innovation ~0, in-gate, applied)
    p.valid = true;
    p.fieldPose = Pose2d{Length{3.0}, Length{0.0}, Angle{}};
    p.confidence = 1.0;
    p.positionStdDev = Length{0.5};
    fc.setProposal(p);
    r.tick(0.05, 3.0);   // no new travel; the fix resets distanceSinceCorrection
    CHECK(r.loc.quality() == doctest::Approx(1.0));
    CHECK(r.loc.qualityClass() == Localizer::Quality::Corrected);
}

TEST_CASE("Localizer: an un-ready IMU forces quality 0 / Uninitialized (boot window)") {
    Rig r;
    r.imu.setReady(false);
    r.tick(0.05, 0.0);
    CHECK(r.loc.quality() == doctest::Approx(0.0));
    CHECK(r.loc.qualityClass() == Localizer::Quality::Uninitialized);
}

TEST_CASE("Localizer: invalid proposals (NaN pose / zero std-dev / zero confidence) are screened out") {
    FakeCorrector fc;
    std::array<ICorrector*, 1> arr{&fc};
    Rig r{arr};
    r.tick(0.01, 0.0);
    CorrectionProposal p{};
    p.valid = true;
    p.fieldPose = Pose2d{Length{std::numeric_limits<double>::quiet_NaN()}, Length{0.0}, Angle{}};
    p.confidence = 1.0;
    p.positionStdDev = Length{0.5};
    fc.setProposal(p);
    r.tick(0.05, 0.0);
    CHECK(std::isfinite(r.loc.pose().x().value()));  // not poisoned
    CHECK(r.loc.isDeadReckoning());                  // screened ⇒ no correction
    CHECK_FALSE(r.loc.lastCorrection().gated);       // dropped BEFORE fusion (not merely gated by it)

    p.fieldPose = Pose2d{Length{1.0}, Length{0.0}, Angle{}};
    p.positionStdDev = Length{0.0};                  // zero std-dev ⇒ invalid
    fc.setProposal(p);
    r.tick(0.05, 0.0);
    CHECK(r.loc.isDeadReckoning());

    p.positionStdDev = Length{0.5};
    p.confidence = 0.0;                              // zero confidence ⇒ invalid (would else "apply" a 0 nudge)
    fc.setProposal(p);
    r.tick(0.05, 0.0);
    CHECK(r.loc.isDeadReckoning());
}

TEST_CASE("Localizer: the corrector is handed the PREDICTED pose and the tick dt") {
    FakeCorrector fc;
    std::array<ICorrector*, 1> arr{&fc};
    Rig r{arr};
    r.tick(0.05, 0.0);          // baseline
    r.tick(0.05, 7.0);          // odom predicts +7" forward
    CHECK(fc.lastPredicted().x().value() == doctest::Approx(7.0));  // predicted, not last-fused
    CHECK(fc.lastDt().value() == doctest::Approx(0.05));
}

TEST_CASE("Localizer: setPose teleports position, keeps IMU heading, injects no phantom velocity") {
    Rig r;
    r.imu.setHeading(Angle::degrees(30.0));
    r.tick(0.05, 0.0);
    r.tick(0.05, 5.0);
    r.loc.setPose(Pose2d{Length{50.0}, Length{-20.0}, Angle::degrees(999.0)});
    CHECK(r.loc.pose().x().value() == doctest::Approx(50.0));
    CHECK(r.loc.pose().y().value() == doctest::Approx(-20.0));
    CHECK(r.loc.pose().heading().degrees() == doctest::Approx(30.0));  // IMU, not 999

    r.tick(0.05, 5.0);  // no new wheel travel after the teleport baseline
    CHECK(r.loc.twist().vx().value() == doctest::Approx(0.0));  // no phantom velocity from the jump
    CHECK(r.loc.pose().x().value() == doctest::Approx(50.0));
}

TEST_CASE("Localizer: rejects an out-of-range config") {
    Rig probe;  // borrow its wired-up odom/imu/clock/fusion for the throwing constructions
    CHECK_THROWS_AS((Localizer{probe.clk, probe.imu, probe.odom, probe.fusion, {},
                               LocalizerConfig{.maxDt = 0.0}}),
                    shulib::PreconditionError);
    CHECK_THROWS_AS((Localizer{probe.clk, probe.imu, probe.odom, probe.fusion, {},
                               LocalizerConfig{.qFloor = 1.0}}),
                    shulib::PreconditionError);
}
