// GpsCorrector through the Localizer and out to the blackbox (chunk E2).
//
// This file closes the half of E1's tension T3 that E1 could not: "every gating decision
// is reconstructable after the fact from the file alone." E1 built the whole path and
// proved it with a SYNTHETIC corrector, because no real one existed — a test double
// producing the number 6.5 and that number arriving in a file proves plumbing, not
// gating. Here the numbers come from a real corrector making real decisions.
//
// Two properties, and they are different:
//   1. VISIBILITY — an off-strip GPS must be distinguishable in the record from "no
//      corrector is registered". Before E2 both produced GateReason::None, because a
//      declined proposal never reaches a fusion policy. Driving Skills has no GPS strip,
//      so this is the difference between a diagnosable Skills run and a mystery.
//   2. RECONSTRUCTABILITY — for each corrector-side verdict, the decoded record must
//      carry enough to re-derive the decision, not merely to name it.
//
// The rig is built from fakes rather than the A2 plant on purpose: this file is about
// what reaches the file, so every input is scripted and every expected value is chosen
// by the test rather than emerging from physics.

#include "doctest.h"

#include <array>
#include <cmath>
#include <span>
#include <vector>

#include "shulib/diag/blackbox_format.hpp"
#include "shulib/diag/blackbox_reader.hpp"
#include "shulib/diag/debug_record.hpp"
#include "shulib/diag/sd_sink.hpp"
#include "shulib/hal/fake/fake_block_sink.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_gps.hpp"
#include "shulib/hal/fake/fake_imu.hpp"
#include "shulib/hal/fake/fake_rotation.hpp"
#include "shulib/localization/complementary_fusion.hpp"
#include "shulib/localization/gps_corrector.hpp"
#include "shulib/localization/localizer.hpp"
#include "shulib/localization/pilons_odometry.hpp"
#include "shulib/localization/tracking_wheel.hpp"
#include "shulib/motion/motion_scheduler.hpp"

namespace bb = shulib::diag::blackbox;

using shulib::diag::DebugRecord;
using shulib::diag::GateReason;
using shulib::diag::SdSink;
using shulib::diag::SdSinkConfig;
using shulib::diag::SdSinkStorage;
using shulib::hal::fake::FakeBlockSink;
using shulib::hal::fake::FakeClock;
using shulib::hal::fake::FakeGps;
using shulib::hal::fake::FakeImu;
using shulib::hal::fake::FakeRotation;
using shulib::localization::ComplementaryFusion;
using shulib::localization::GpsCorrector;
using shulib::localization::GpsCorrectorConfig;
using shulib::localization::ICorrector;
using shulib::localization::Localizer;
using shulib::localization::PilonsOdometry;
using shulib::localization::TrackingWheel;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::motion::CommandIdStampSink;
using shulib::units::AngularVelocity;
using shulib::units::Length;
using shulib::units::Time;

namespace {

/// The E1 rig shape (localizer_test.cpp's Rig) with a real GpsCorrector wired in.
/// Diameter-2 tracking wheels: a shaft reading in radians is inches of travel, so a test
/// injects motion by setting the cumulative forward reading directly.
struct GpsRig {
    FakeClock clk;
    FakeImu imu;
    FakeGps gps;
    FakeRotation fwdRot;
    FakeRotation latRot;
    PilonsOdometry odom;
    ComplementaryFusion fusion;
    GpsCorrector corrector;
    std::array<ICorrector*, 1> correctors;
    Localizer loc;

    explicit GpsRig(const GpsCorrectorConfig& cfg = {}, bool wireCorrector = true)
        : odom{imu, TrackingWheel::forward(fwdRot, Length{2.0}, Length{0.0}),
                    TrackingWheel::lateral(latRot, Length{2.0}, Length{0.0})},
          corrector{clk, gps, imu, cfg},
          correctors{&corrector},
          loc{clk, imu, odom, fusion,
              wireCorrector ? std::span<ICorrector* const>{correctors}
                            : std::span<ICorrector* const>{}} {
        imu.setReady(true);
        imu.setYawRate(AngularVelocity{0.0});
    }

    void tick(double dt, double fwdTravel) {
        clk.advance(Time{dt});
        fwdRot.setPosition(shulib::units::AngleDim{fwdTravel});
        loc.update();
    }
};

/// Every tick record in the device's bytes, in file order. (Same helper as
/// blackbox_introspection_test.cpp — kept local so neither file constrains the other.)
std::vector<DebugRecord> decodeTicks(const FakeBlockSink& device) {
    std::vector<DebugRecord> out;
    bb::BlackboxReader reader{device.view()};
    REQUIRE(reader.status() == bb::ReadStatus::Ok);
    bb::BlackboxReader::Frame frame;
    while (reader.next(frame)) {
        if (frame.type != bb::FrameType::Tick) {
            continue;
        }
        DebugRecord r;
        bool corrupt = false;
        REQUIRE(bb::decodeTick(frame.payload, r, corrupt));
        CHECK_FALSE(corrupt);
        out.push_back(r);
    }
    return out;
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────────────
// 1. VISIBILITY — the off-strip case, which is the one that loses Skills matches
// ─────────────────────────────────────────────────────────────────────────────────────

// Would catch: THE SKILLS BUG. A GPS that is off the strip for the whole run producing
// exactly the same telemetry as a robot with no corrector at all. Before E2 the record
// said GateReason::None in both cases — "no correction proposal this tick" — and the
// per-source accounting said "none". A team reading that file cannot tell "the strip is
// missing, as expected in Driving Skills" from "the GPS was never wired up".
TEST_CASE("off-strip: the estimator says WHY it is dead-reckoning, and names the source") {
    GpsRig r;
    r.gps.setHasFix(false);
    r.gps.setPose(Pose2d{});  // the origin, as HA-31's adversarial model serves it

    r.tick(0.01, 0.0);
    for (int i = 1; i <= 20; ++i) {
        r.tick(0.01, static_cast<double>(i));
    }

    CHECK(r.loc.lastCorrection().audit.reason == GateReason::RejectedNoFix);
    CHECK(std::string_view{r.loc.lastCorrection().source} == "gps");
    CHECK(r.loc.isDeadReckoning());
    CHECK(r.corrector.noFixTicks() == 21);
    CHECK(r.corrector.acceptedFixes() == 0);
}

// Would catch: an off-strip GPS DRAGGING the estimate. The model serves the origin while
// it has no fix, deliberately, so a corrector that proposed anything at all would pull a
// robot at (20, 0) toward (0, 0). Proven by running the identical motion with and
// without the corrector wired and requiring the two estimates to agree EXACTLY: "degrades
// to dead-reckon-only" means the corrector contributes nothing, not merely a little.
TEST_CASE("off-strip: the estimate is bit-identical to having no corrector at all") {
    GpsRig withGps{{}, /*wireCorrector=*/true};
    GpsRig without{{}, /*wireCorrector=*/false};
    withGps.gps.setHasFix(false);
    withGps.gps.setPose(Pose2d{});
    without.gps.setHasFix(false);

    withGps.tick(0.01, 0.0);
    without.tick(0.01, 0.0);
    for (int i = 1; i <= 50; ++i) {
        const double travel = static_cast<double>(i);
        withGps.tick(0.01, travel);
        without.tick(0.01, travel);
    }

    CHECK(withGps.loc.pose().x().value() == without.loc.pose().x().value());
    CHECK(withGps.loc.pose().y().value() == without.loc.pose().y().value());
    CHECK(withGps.loc.pose().x().value() == doctest::Approx(50.0));  // and it really moved
    // The difference is entirely in what the run can TELL you afterwards.
    CHECK(withGps.loc.lastCorrection().audit.reason == GateReason::RejectedNoFix);
    CHECK(without.loc.lastCorrection().audit.reason == GateReason::None);
}

// Would catch: the substitution rule OVERWRITING a real fusion verdict. The policy's
// answer must win whenever it has one — a corrector's self-audit is only ever a
// fallback for the tick where nothing reached the policy.
TEST_CASE("substitution: a real fusion verdict is never overwritten by a corrector's") {
    GpsRig r;
    r.tick(0.01, 0.0);

    // A good fix, accepted by both the corrector and the policy.
    r.gps.setHasFix(true);
    r.gps.setRmsError(Length{1.0});
    r.gps.setPose(Pose2d{Length{1.0}, Length{0.0}, Angle::radians(0.0)});
    r.tick(0.01, 0.0);
    CHECK(r.loc.lastCorrection().audit.reason == GateReason::Accepted);
    CHECK(std::string_view{r.loc.lastCorrection().source} == "gps");
    // The policy's trust weight, not the corrector's sigma, occupies the slot on an
    // accepted tick — the E1 semantics for that field, unchanged.
    CHECK(r.loc.lastCorrection().audit.covarianceTrace > 0.0);
    CHECK(r.loc.lastCorrection().audit.covarianceTrace <= 1.0);
}

// FOUND BY MUTATION (E2-PROGRESS, 22:52). Would catch: the substitution rule dropping its
// `reason == None` guard — i.e. a SILENT corrector's verdict being stamped over a verdict
// the fusion policy actually rendered.
//
// Why no other test in this file could see it: with ONE corrector the guard is dead code,
// because `selfAuditSource` is only ever set when a proposal was DECLINED, and a declined
// proposal is exactly the case where the policy saw nothing and reported None. The guard
// only becomes load-bearing when a SECOND corrector exists — which is E3, one chunk away
// — and then the failure is that a tick where a fix WAS applied gets recorded as
// "RejectedNoFix". A record that says the estimator dead-reckoned on a tick where it
// corrected is worse than a record that says nothing.
//
// So: two correctors, one healthy and one off-strip, on the same tick.
TEST_CASE("substitution: a silent corrector cannot overwrite an applied fix's verdict") {
    FakeClock clk;
    FakeImu imu;
    FakeGps liveGps;
    FakeGps deadGps;
    FakeRotation fwdRot;
    FakeRotation latRot;
    PilonsOdometry odom{imu, TrackingWheel::forward(fwdRot, Length{2.0}, Length{0.0}),
                        TrackingWheel::lateral(latRot, Length{2.0}, Length{0.0})};
    ComplementaryFusion fusion;
    GpsCorrector live{clk, liveGps, imu, {}, "gps-front"};
    GpsCorrector dead{clk, deadGps, imu, {}, "gps-rear"};
    // Order matters to the bug: the HEALTHY corrector is asked first, so the policy has a
    // verdict by the time the silent one's self-audit is available to be substituted.
    std::array<ICorrector*, 2> both{&live, &dead};
    Localizer loc{clk, imu, odom, fusion, std::span<ICorrector* const>{both}};
    imu.setReady(true);
    imu.setYawRate(AngularVelocity{0.0});

    clk.advance(Time{0.01});
    loc.update();

    liveGps.setHasFix(true);
    liveGps.setRmsError(Length{1.0});
    liveGps.setPose(Pose2d{Length{2.0}, Length{0.0}, Angle::radians(0.0)});
    deadGps.setHasFix(false);  // off the strip, and saying so
    clk.advance(Time{0.01});
    loc.update();

    // Both correctors really did what the test needs them to have done.
    REQUIRE(live.acceptedFixes() == 1);
    REQUIRE(dead.noFixTicks() == 2);
    // The applied correction is what the record must report — not the silent source.
    CHECK(loc.lastCorrection().audit.reason == GateReason::Accepted);
    CHECK(std::string_view{loc.lastCorrection().source} == "gps-front");
    CHECK_FALSE(loc.isDeadReckoning());
    CHECK(loc.lastCorrection().audit.residualX.value() == doctest::Approx(2.0).epsilon(1e-6));
}

// ─────────────────────────────────────────────────────────────────────────────────────
// 2. RECONSTRUCTABILITY — from the decoded file, with nothing else
// ─────────────────────────────────────────────────────────────────────────────────────

// Would catch: a corrector-side verdict dying anywhere on the path — the corrector not
// reporting it, the Localizer dropping it with the declined proposal, the record
// producer not stamping it, or the encoder skipping the field. Each of the five
// corrector verdicts is provoked by a REAL condition (no strip, a re-read, a bad claim,
// a spin, a lie) rather than scripted, and each must arrive in the DECODED file.
TEST_CASE("blackbox: every real corrector verdict arrives in the decoded file") {
    GpsCorrectorConfig cfg;
    cfg.latency = Time{0.0};
    cfg.driftStdDevPerInch = 0.0;
    cfg.postFixStdDev = Length{1.0};
    cfg.rmsTrustFactor = 2.0;
    cfg.gateSigma = 4.0;
    cfg.maxReportedRms = Length{6.0};
    cfg.maxYawRate = AngularVelocity{3.0};
    GpsRig r{cfg};

    FakeBlockSink device;
    std::vector<DebugRecord> ring(32);
    std::vector<std::byte> buffer(65536);
    SdSink blackbox{device, r.clk, SdSinkStorage{ring, buffer},
                    SdSinkConfig{.streamTicks = true}};
    CommandIdStampSink stamp{blackbox};
    blackbox.open({});

    // One scripted tick per verdict. The record is emitted by hand (this file is not
    // testing the scheduler) but through the SAME stamping decorator a real run uses.
    const auto emitTick = [&]() {
        stamp.setEstimatorAudit(r.loc.lastCorrection());
        DebugRecord rec;
        rec.t = r.clk.now();
        rec.measuredPose = r.loc.pose();
        stamp.emit(rec);
    };

    r.tick(0.01, 0.0);  // settle: no fix yet

    // (a) RejectedNoFix — off the strip.
    r.gps.setHasFix(false);
    r.tick(0.01, 0.0);
    CHECK(r.loc.lastCorrection().audit.reason == GateReason::RejectedNoFix);
    emitTick();

    // (b) Accepted — a good fix, three inches out.
    r.gps.setHasFix(true);
    r.gps.setRmsError(Length{1.0});
    r.gps.setPose(Pose2d{Length{3.0}, Length{0.0}, Angle::radians(0.0)});
    r.tick(0.01, 0.0);
    CHECK(r.loc.lastCorrection().audit.reason == GateReason::Accepted);
    emitTick();

    // (c) RejectedStaleFix — the same sample re-read on the next loop tick.
    r.tick(0.01, 0.0);
    CHECK(r.loc.lastCorrection().audit.reason == GateReason::RejectedStaleFix);
    emitTick();

    // (d) RejectedSensorQuality — the device claims a fix and eight feet of error.
    r.gps.setPose(Pose2d{Length{3.5}, Length{0.0}, Angle::radians(0.0)});
    r.gps.setRmsError(Length{99.0});
    r.tick(0.01, 0.0);
    CHECK(r.loc.lastCorrection().audit.reason == GateReason::RejectedSensorQuality);
    emitTick();

    // (e) RejectedHighYawRate — a fresh fix taken mid-spin.
    r.imu.setYawRate(AngularVelocity{6.0});
    r.gps.setPose(Pose2d{Length{4.0}, Length{0.0}, Angle::radians(0.0)});
    r.gps.setRmsError(Length{1.0});
    r.tick(0.01, 0.0);
    CHECK(r.loc.lastCorrection().audit.reason == GateReason::RejectedHighYawRate);
    emitTick();

    // (f) RejectedNormalizedInnovation — a confident lie, well outside the gate.
    r.imu.setYawRate(AngularVelocity{0.0});
    r.gps.setPose(Pose2d{Length{60.0}, Length{-45.0}, Angle::radians(0.0)});
    r.gps.setRmsError(Length{1.0});
    r.tick(0.01, 0.0);
    CHECK(r.loc.lastCorrection().audit.reason == GateReason::RejectedNormalizedInnovation);
    emitTick();

    CHECK(blackbox.flush());

    const std::vector<DebugRecord> ticks = decodeTicks(device);
    REQUIRE(ticks.size() == 6);
    const GateReason expected[] = {
        GateReason::RejectedNoFix,        GateReason::Accepted,
        GateReason::RejectedStaleFix,     GateReason::RejectedSensorQuality,
        GateReason::RejectedHighYawRate,  GateReason::RejectedNormalizedInnovation};
    for (std::size_t i = 0; i < ticks.size(); ++i) {
        CAPTURE(i);
        CHECK(ticks[i].gateReason == expected[i]);
        // The Mahalanobis slot is EMPTY on every one of them. E2's gate is a normalized
        // innovation, not a Mahalanobis distance (tension T1), and a plausible number
        // here would be indistinguishable from the one E4's EKF will earn.
        CHECK(ticks[i].gateMahalanobis == 0.0);
    }
}

// Would catch: a gate rejection that names itself but cannot be CHECKED. This is the
// substance of "reconstructable from the file alone": starting from the decoded record
// and the documented gateSigma, and using nothing that is not on disk, re-derive the
// normalized innovation and confirm the verdict the corrector reached.
TEST_CASE("blackbox: a gate rejection is RE-DERIVED from the decoded record alone") {
    GpsCorrectorConfig cfg;
    cfg.latency = Time{0.0};
    cfg.driftStdDevPerInch = 0.0;
    cfg.postFixStdDev = Length{1.0};
    cfg.rmsTrustFactor = 2.0;
    cfg.gateSigma = 4.0;
    GpsRig r{cfg};

    FakeBlockSink device;
    std::vector<DebugRecord> ring(16);
    std::vector<std::byte> buffer(65536);
    SdSink blackbox{device, r.clk, SdSinkStorage{ring, buffer},
                    SdSinkConfig{.streamTicks = true}};
    CommandIdStampSink stamp{blackbox};
    blackbox.open({});

    r.tick(0.01, 0.0);
    r.gps.setHasFix(true);
    r.gps.setRmsError(Length{1.0});
    r.gps.setPose(Pose2d{Length{24.0}, Length{-18.0}, Angle::radians(0.0)});  // a 30" lie
    r.tick(0.01, 0.0);

    stamp.setEstimatorAudit(r.loc.lastCorrection());
    DebugRecord rec;
    rec.t = r.clk.now();
    stamp.emit(rec);
    CHECK(blackbox.flush());

    const std::vector<DebugRecord> ticks = decodeTicks(device);
    REQUIRE(ticks.size() == 1);
    const DebugRecord& got = ticks[0];

    // Everything from here uses ONLY `got` and the documented config.
    REQUIRE(got.gateReason == GateReason::RejectedNormalizedInnovation);
    const double residual = std::hypot(got.gateResidualX.value(), got.gateResidualY.value());
    const double sigmaEff = got.covarianceTrace;  // what the gate normalized by
    REQUIRE(sigmaEff > 0.0);
    const double nu = residual / sigmaEff;
    CHECK(nu > cfg.gateSigma);  // the verdict, re-derived

    // …and the numbers themselves are the ones a person would compute from the scenario:
    // a 30-inch disagreement against sigma_eff = hypot(2·1.0, 1.0) = sqrt(5).
    CHECK(residual == doctest::Approx(30.0).epsilon(1e-6));
    CHECK(sigmaEff == doctest::Approx(2.2360679774997896).epsilon(1e-9));
    CHECK(nu == doctest::Approx(13.416407864998739).epsilon(1e-6));
    // Nothing moved: a rejected fix leaves the estimate exactly where odometry put it.
    CHECK(got.correctionDx.value() == doctest::Approx(0.0).epsilon(1e-12));
    CHECK(got.correctionDy.value() == doctest::Approx(0.0).epsilon(1e-12));
}

// Would catch: a no-fix tick carrying junk in the residual slots. RejectedNoFix means
// there was no fix to difference against the prediction, so the residual must be
// ABSENT (zero), not a leftover from an earlier tick — a stale number in an audit field
// is exactly the invisible-wrong that E2's T1 ruling refuses for gateMahalanobis.
TEST_CASE("blackbox: a no-fix verdict carries no residual, because there is none") {
    GpsRig r;
    r.tick(0.01, 0.0);

    // First a real fix, so there ARE numbers in the audit to go stale.
    r.gps.setHasFix(true);
    r.gps.setRmsError(Length{1.0});
    r.gps.setPose(Pose2d{Length{2.0}, Length{1.0}, Angle::radians(0.0)});
    r.tick(0.01, 0.0);
    REQUIRE(r.loc.lastCorrection().audit.reason == GateReason::Accepted);
    REQUIRE(r.loc.lastCorrection().audit.residualX.value() != 0.0);

    // Then the strip disappears.
    r.gps.setHasFix(false);
    r.tick(0.01, 0.0);
    CHECK(r.loc.lastCorrection().audit.reason == GateReason::RejectedNoFix);
    CHECK(r.loc.lastCorrection().audit.residualX.value() == 0.0);
    CHECK(r.loc.lastCorrection().audit.residualY.value() == 0.0);
    CHECK(r.loc.lastCorrection().audit.covarianceTrace == 0.0);
    CHECK(r.loc.lastCorrection().audit.mahalanobis == 0.0);
}
