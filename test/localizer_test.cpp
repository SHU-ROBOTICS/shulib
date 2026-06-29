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

// A LOW-confidence fix clears drift only PARTIALLY — quality must not spring to 1.0 on a weak fix.
TEST_CASE("Localizer: a low-confidence fix clears drift proportionally (no false 1.0)") {
    FakeCorrector fc;
    std::array<ICorrector*, 1> arr{&fc};
    Rig r{arr};  // driftHorizon 12"
    r.tick(0.01, 0.0);
    r.tick(0.01, 6.0);  // dead-reckon 6" → quality 0.5
    CHECK(r.loc.quality() == doctest::Approx(0.5));

    CorrectionProposal p{};
    p.valid = true;
    p.fieldPose = Pose2d{Length{6.0}, Length{0.0}, Angle{}};
    p.confidence = 0.1;  // WEAK fix
    p.positionStdDev = Length{0.5};
    fc.setProposal(p);
    r.tick(0.01, 6.0);  // weak fix: drift 6" → 6·(1-0.1) = 5.4" → quality 1 - 5.4/12 = 0.55
    CHECK(r.loc.quality() == doctest::Approx(0.55));  // improved a little, NOT reset to 1.0
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

// CRITICAL-fix keystone: a PERSISTENT in-gate fix must let the fused pose CONVERGE toward it over
// many ticks (the pre-fix bug pinned it at a single-tick nudge forever, never converging).
TEST_CASE("Localizer: a persistent fix converges the fused pose toward it") {
    FakeCorrector fc;
    std::array<ICorrector*, 1> arr{&fc};
    Rig r{arr};
    r.tick(0.01, 0.0);  // baseline (corrector invalid)
    CorrectionProposal p{};
    p.valid = true;
    p.fieldPose = Pose2d{Length{5.0}, Length{0.0}, Angle{}};
    p.confidence = 1.0;
    p.positionStdDev = Length{0.5};
    fc.setProposal(p);

    r.tick(0.01, 0.0);
    const double afterOne = r.loc.pose().x().value();  // one nudge ≈ 0.12"
    for (int i = 0; i < 300; ++i) r.tick(0.01, 0.0);   // robot still, fix persists
    CHECK(afterOne < 0.2);                              // a single tick is only a small nudge...
    CHECK(r.loc.pose().x().value() == doctest::Approx(5.0));  // ...but it CONVERGES to the fix
    CHECK_FALSE(r.loc.isDeadReckoning());
}

// CRITICAL-fix keystone #2: an applied correction must PERSIST after the corrector goes silent —
// no snap-back to raw odom, no reverse twist spike (the pre-fix bug discarded it in one tick).
TEST_CASE("Localizer: an applied correction persists after the corrector goes quiet") {
    FakeCorrector fc;
    std::array<ICorrector*, 1> arr{&fc};
    Rig r{arr};
    r.tick(0.01, 0.0);
    CorrectionProposal p{};
    p.valid = true;
    p.fieldPose = Pose2d{Length{5.0}, Length{0.0}, Angle{}};
    p.confidence = 1.0;
    p.positionStdDev = Length{0.5};
    fc.setProposal(p);
    for (int i = 0; i < 5; ++i) r.tick(0.01, 0.0);  // accumulate part of the correction
    const double held = r.loc.pose().x().value();
    CHECK(held > 0.3);  // we actually moved toward the fix

    fc.setProposal(CorrectionProposal{});  // corrector goes invalid
    r.tick(0.01, 0.0);
    CHECK(r.loc.pose().x().value() == doctest::Approx(held));  // RETAINED, not snapped to 0
    CHECK(r.loc.isDeadReckoning());
    CHECK(std::abs(r.loc.twist().vx().value()) < 1.0);  // no reverse snap-back spike
}

TEST_CASE("Localizer: a non-finite (+Inf) confidence is screened, never poisons the pose") {
    FakeCorrector fc;
    std::array<ICorrector*, 1> arr{&fc};
    Rig r{arr};
    r.tick(0.01, 0.0);
    CorrectionProposal p{};
    p.valid = true;
    p.fieldPose = Pose2d{Length{2.0}, Length{0.0}, Angle{}};
    p.confidence = std::numeric_limits<double>::infinity();  // Inf > 0 would pass a naive >0 check
    p.positionStdDev = Length{0.5};
    fc.setProposal(p);
    r.tick(0.05, 0.0);
    CHECK(std::isfinite(r.loc.pose().x().value()));
    CHECK(r.loc.isDeadReckoning());              // screened out, no correction
    CHECK_FALSE(r.loc.lastCorrection().gated);   // dropped at the Localizer, never reached fusion
}

TEST_CASE("Localizer: a tiny-but-positive dt does not blow twist into a velocity spike") {
    Rig r;
    r.tick(0.05, 0.0);
    r.tick(0.05, 5.0);  // establishes a real velocity
    r.clk.advance(Time{1e-6});  // dt below minDt (1e-4)
    r.fwdRot.setPosition(shulib::units::AngleDim{5.5});  // 0.5" over 1µs would be 5e5 in/s unguarded
    r.loc.update();
    CHECK(std::isfinite(r.loc.twist().vx().value()));
    CHECK(r.loc.twist().vx().value() == doctest::Approx(0.0));  // untrusted dt ⇒ no velocity estimate
}

TEST_CASE("Localizer: a correction at a non-zero heading nudges in the field frame, heading IMU-owned") {
    FakeCorrector fc;
    std::array<ICorrector*, 1> arr{&fc};
    Rig r{arr};
    r.imu.setHeading(Angle::degrees(45.0));
    r.tick(0.01, 0.0);
    CorrectionProposal p{};
    p.valid = true;
    p.fieldPose = Pose2d{Length{0.0}, Length{4.0}, Angle::degrees(45.0)};  // field +Y fix
    p.confidence = 1.0;
    p.positionStdDev = Length{0.5};
    fc.setProposal(p);
    r.tick(0.05, 0.0);
    CHECK(r.loc.pose().heading().degrees() == doctest::Approx(45.0));  // IMU-owned
    CHECK(r.loc.pose().y().value() > 0.0);   // nudged toward the field +Y fix
    CHECK(r.loc.pose().x().value() == doctest::Approx(0.0));  // field-frame, not body-rotated
}

TEST_CASE("Localizer: two correctors are both gathered and fused") {
    FakeCorrector a{"gps"}, b{"tag"};
    std::array<ICorrector*, 2> arr{&a, &b};
    Rig r{arr};
    r.tick(0.01, 0.0);
    CorrectionProposal pa{}, pb{};
    pa.valid = true; pa.fieldPose = Pose2d{Length{4.0}, Length{0.0}, Angle{}};
    pa.confidence = 1.0; pa.positionStdDev = Length{0.5};
    pb.valid = true; pb.fieldPose = Pose2d{Length{0.0}, Length{4.0}, Angle{}};
    pb.confidence = 1.0; pb.positionStdDev = Length{0.5};
    a.setProposal(pa); b.setProposal(pb);
    r.tick(0.05, 0.0);
    CHECK(a.calls() >= 1);
    CHECK(b.calls() >= 1);
    CHECK(r.loc.pose().x().value() > 0.0);  // pulled toward A
    CHECK(r.loc.pose().y().value() > 0.0);  // and toward B
    CHECK_FALSE(r.loc.isDeadReckoning());
}

TEST_CASE("Localizer: lastCorrection reports the applied delta and the source name") {
    FakeCorrector fc{"gps"};
    std::array<ICorrector*, 1> arr{&fc};
    Rig r{arr};
    r.tick(0.01, 0.0);
    CorrectionProposal p{};
    p.valid = true; p.fieldPose = Pose2d{Length{4.0}, Length{0.0}, Angle{}};
    p.confidence = 1.0; p.positionStdDev = Length{0.5};
    fc.setProposal(p);
    r.tick(0.05, 0.0);
    CHECK(r.loc.lastCorrection().dx.value() > 0.0);   // nudged toward +X
    CHECK(r.loc.lastCorrection().dy.value() == doctest::Approx(0.0));
    CHECK(std::string{r.loc.lastCorrection().source} == "gps");  // real corrector name, not a literal
}

TEST_CASE("Localizer: distanceSinceCorrection grows while dead-reckoning and resets on a fix") {
    FakeCorrector fc;
    std::array<ICorrector*, 1> arr{&fc};
    Rig r{arr};
    r.tick(0.01, 0.0);
    r.tick(0.01, 4.0);  // 4" dead-reckon
    CHECK(r.loc.distanceSinceCorrection().value() == doctest::Approx(4.0));
    CorrectionProposal p{};
    p.valid = true; p.fieldPose = Pose2d{Length{4.0}, Length{0.0}, Angle{}};
    p.confidence = 1.0; p.positionStdDev = Length{0.5};
    fc.setProposal(p);
    r.tick(0.01, 4.0);  // a fix
    CHECK(r.loc.distanceSinceCorrection().value() == doctest::Approx(0.0));
}

TEST_CASE("Localizer: an implausible odometry tick is reported Degraded") {
    Rig r;
    r.tick(0.05, 0.0);
    r.imu.setHeading(Angle::degrees(120.0));  // 120° in one tick > PilonsOdometry's 90° gate
    r.tick(0.05, 0.0);
    CHECK(r.loc.qualityClass() == Localizer::Quality::Degraded);
    CHECK(r.loc.quality() < 1.0);  // scalar agrees with the class (no 1.0 on a Degraded tick)
}

TEST_CASE("Localizer: a non-finite IMU yaw-rate cannot poison twist.omega") {
    Rig r;
    r.tick(0.05, 0.0);
    r.imu.setYawRate(AngularVelocity{std::numeric_limits<double>::quiet_NaN()});
    r.tick(0.05, 1.0);
    CHECK(std::isfinite(r.loc.twist().omega().value()));
    CHECK(r.loc.twist().omega().value() == doctest::Approx(0.0));
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
