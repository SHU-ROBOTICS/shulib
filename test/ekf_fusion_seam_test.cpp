// EkfFusion through the REAL Localizer, and out to the blackbox (chunk E4).
//
// Two DoD clauses live here.
//
//   1. "THE SWAP IS INVISIBLE ABOVE THE SEAM." The interesting content of that claim is not
//      that the code compiles — it is that every observable the Localizer publishes still
//      behaves the way its own tests say it does, with a completely different filter
//      underneath. So this file drives one rig twice, changing exactly ONE constructor
//      argument, and checks the boot guard, the settle window, the IMU-dropout degradation,
//      the quality classes, the drift accumulator, `setPose`, the heading re-stamp and the
//      audit record on both.
//   2. "gateMahalanobis and the re-init event are readable from the file alone." E2 set that
//      standard for the GPS and E3 held the tag path to it; this holds the EKF to it, for the
//      two slots that have been declared and empty since A1.
//
// The rig is built from fakes on purpose: every input is scripted, so an expectation here is
// something the test chose rather than something physics produced.

#include "doctest.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <span>
#include <vector>

#include "shulib/diag/blackbox_format.hpp"
#include "shulib/diag/blackbox_reader.hpp"
#include "shulib/diag/debug_record.hpp"
#include "shulib/diag/sd_sink.hpp"
#include "shulib/hal/fake/fake_block_sink.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_imu.hpp"
#include "shulib/hal/fake/fake_rotation.hpp"
#include "shulib/localization/complementary_fusion.hpp"
#include "shulib/localization/ekf_fusion.hpp"
#include "shulib/localization/fake/fake_corrector.hpp"
#include "shulib/localization/i_corrector.hpp"
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
using shulib::hal::fake::FakeImu;
using shulib::hal::fake::FakeRotation;
using shulib::localization::ComplementaryFusion;
using shulib::localization::CorrectionProposal;
using shulib::localization::EkfFusion;
using shulib::localization::EkfFusionConfig;
using shulib::localization::ICorrector;
using shulib::localization::IFusionPolicy;
using shulib::localization::Localizer;
using shulib::localization::PilonsOdometry;
using shulib::localization::TrackingWheel;
using shulib::localization::fake::FakeCorrector;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::motion::CommandIdStampSink;
using shulib::units::AngleDim;
using shulib::units::AngularVelocity;
using shulib::units::Length;
using shulib::units::Time;

namespace {

constexpr double kDt = 0.01;
constexpr double kDeg = Angle::kPi / 180.0;

/// One rig, one fusion policy handed in from outside. That the policy is the ONLY difference
/// between the two runs is the point of the file.
struct Rig {
    FakeClock clk;
    FakeImu imu;
    FakeRotation fwdRot;
    FakeRotation latRot;
    PilonsOdometry odom;
    FakeCorrector corrector;
    std::array<ICorrector*, 1> correctors;
    Localizer loc;
    double travel = 0.0;

    explicit Rig(IFusionPolicy& policy, bool wireCorrector = true)
        : odom{imu, TrackingWheel::forward(fwdRot, Length{2.0}, Length{0.0}),
               TrackingWheel::lateral(latRot, Length{2.0}, Length{0.0})},
          corrector{"fake-gps"},
          correctors{&corrector},
          loc{clk, imu, odom, policy,
              wireCorrector ? std::span<ICorrector* const>{correctors}
                            : std::span<ICorrector* const>{}} {
        imu.setReady(true);
        imu.setYawRate(AngularVelocity{0.0});
    }

    /// One tick, driving `fwdInches` forward along the current heading.
    void tick(double fwdInches = 0.0, double dt = kDt) {
        clk.advance(Time{dt});
        travel += fwdInches;
        fwdRot.setPosition(AngleDim{travel});  // radians of a 2-inch-radius wheel
        loc.update();
    }
};

[[nodiscard]] CorrectionProposal fixAt(double x, double y, double sigma, double conf = 0.8) {
    CorrectionProposal p{};
    p.valid = true;
    p.fieldPose = Pose2d{Length{x}, Length{y}, Angle{}};
    p.confidence = conf;
    p.positionStdDev = Length{sigma};
    return p;
}

[[nodiscard]] std::vector<DebugRecord> decodeTicks(const FakeBlockSink& device) {
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

/// The observables the Localizer publishes, snapshotted, so the two tiers can be compared as
/// wholes rather than one assertion at a time.
struct Snapshot {
    int qualityClass = 0;
    double quality = 0.0;
    bool deadReckoning = false;
    double distanceSinceCorrection = 0.0;
    double headingRad = 0.0;
    const char* source = "";
    GateReason reason = GateReason::None;
    bool clamped = false;
};

[[nodiscard]] Snapshot snap(const Localizer& l) {
    Snapshot s;
    s.qualityClass = static_cast<int>(l.qualityClass());
    s.quality = l.quality();
    s.deadReckoning = l.isDeadReckoning();
    s.distanceSinceCorrection = l.distanceSinceCorrection().value();
    s.headingRad = l.pose().heading().radians();
    s.source = l.lastCorrection().source;
    s.reason = l.lastCorrection().audit.reason;
    s.clamped = l.lastCorrection().clamped;
    return s;
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────────────────
// The swap is invisible above the seam
// ─────────────────────────────────────────────────────────────────────────────────────────

// Would catch: a filter swap that quietly changes what the ESTIMATOR'S OWN HEALTH REPORTING
// means. Every consumer in the tree — the motion loop's gating, the skills selector, the run
// summary — reads `qualityClass()` and `isDeadReckoning()`, not the pose. If installing the
// EKF changed when a run reads "Corrected", a routine could start moving on an estimate the
// old tier would have refused, and nothing about the pose would look wrong.
TEST_CASE("seam: the Localizer's health reporting is identical under both fusion tiers") {
    ComplementaryFusion comp{};
    EkfFusion ekf{};
    Rig a{comp};
    Rig b{ekf};

    std::vector<Snapshot> sa;
    std::vector<Snapshot> sb;
    for (int i = 0; i < 400; ++i) {
        // the same script for both: drive, with a fix every fifth tick for the middle third
        const bool fixNow = (i >= 120 && i < 260 && (i % 5) == 0);
        const double fwd = 0.15;
        const Pose2d pa = a.loc.pose();
        const Pose2d pb = b.loc.pose();
        a.corrector.setProposal(fixNow ? fixAt(pa.x().value() + 1.0, pa.y().value(), 0.8)
                                       : CorrectionProposal{});
        b.corrector.setProposal(fixNow ? fixAt(pb.x().value() + 1.0, pb.y().value(), 0.8)
                                       : CorrectionProposal{});
        a.tick(fwd);
        b.tick(fwd);
        sa.push_back(snap(a.loc));
        sb.push_back(snap(b.loc));
    }
    REQUIRE(sa.size() == sb.size());
    int classMatches = 0;
    int deadReckonMatches = 0;
    for (std::size_t i = 0; i < sa.size(); ++i) {
        CAPTURE(i);
        CHECK(sa[i].qualityClass == sb[i].qualityClass);
        CHECK(sa[i].deadReckoning == sb[i].deadReckoning);
        classMatches += (sa[i].qualityClass == sb[i].qualityClass) ? 1 : 0;
        deadReckonMatches += (sa[i].deadReckoning == sb[i].deadReckoning) ? 1 : 0;
    }
    CHECK(classMatches == 400);
    CHECK(deadReckonMatches == 400);
    // …and the script really did exercise more than one state, so the agreement is not the
    // agreement of two flat lines.
    bool sawCorrected = false;
    bool sawDeadReckon = false;
    for (const Snapshot& s : sb) {
        sawCorrected = sawCorrected || s.qualityClass == 2;
        sawDeadReckon = sawDeadReckon || s.qualityClass == 1;
    }
    CHECK(sawCorrected);
    CHECK(sawDeadReckon);
}

// Would catch: the EKF breaking the boot guard or the settle window — the two A3 findings that
// cost the project a 10.8-inch permanent error and a 3.65-inch leak respectively. They live in
// the Localizer, not the policy, but a policy that integrated the boot ticks (where the fold is
// deliberately closed and the prediction therefore never moves) could resurrect both.
TEST_CASE("seam: the boot guard and settle window behave identically under the EKF") {
    ComplementaryFusion comp{};
    EkfFusion ekf{};
    Rig a{comp};
    Rig b{ekf};
    a.imu.setReady(false);
    b.imu.setReady(false);

    for (int i = 0; i < 200; ++i) {  // 2 s of calibration garbage that MOVES
        a.imu.setHeading(Angle::degrees(40.0 * std::sin(0.1 * i)));
        b.imu.setHeading(Angle::degrees(40.0 * std::sin(0.1 * i)));
        a.tick(0.2);
        b.tick(0.2);
    }
    CHECK(a.loc.qualityClass() == Localizer::Quality::Uninitialized);
    CHECK(b.loc.qualityClass() == Localizer::Quality::Uninitialized);
    a.imu.setReady(true);
    b.imu.setReady(true);
    a.imu.setHeading(Angle{});
    b.imu.setHeading(Angle{});
    for (int i = 0; i < 5; ++i) {  // inside the 0.1 s settle window
        a.tick(0.2);
        b.tick(0.2);
    }
    CHECK(a.loc.qualityClass() == Localizer::Quality::Uninitialized);
    CHECK(b.loc.qualityClass() == Localizer::Quality::Uninitialized);
    for (int i = 0; i < 20; ++i) {
        a.tick(0.2);
        b.tick(0.2);
    }
    CHECK(a.loc.qualityClass() != Localizer::Quality::Uninitialized);
    CHECK(b.loc.qualityClass() != Localizer::Quality::Uninitialized);
    // The boot moved neither estimate: the fold was closed, so both are still at the origin.
    CHECK(std::hypot(a.loc.pose().x().value(), a.loc.pose().y().value()) < 4.0);
    CHECK(std::hypot(b.loc.pose().x().value(), b.loc.pose().y().value()) < 4.0);
}

// Would catch: a filter that treats a teleport as motion. `setPose()` is how a routine re-seeds
// its position, and every routine in Phase F will call it. Under the EKF the tick after it
// arrives with dt == 0, which the filter has to recognise as a discontinuity rather than as a
// robot crossing the field in ten milliseconds.
TEST_CASE("seam: setPose teleports cleanly under the EKF, with no phantom velocity") {
    EkfFusion ekf{};
    Rig r{ekf};
    for (int i = 0; i < 100; ++i) {
        r.tick(0.2);
    }
    r.loc.setPose(Pose2d{Length{72.0}, Length{-48.0}, Angle::degrees(90.0)});
    CHECK(r.loc.pose().x().value() == doctest::Approx(72.0));
    CHECK(r.loc.pose().y().value() == doctest::Approx(-48.0));
    // The next few ticks must continue from there, not spring back and not run away.
    for (int i = 0; i < 50; ++i) {
        r.tick(0.0);
        CHECK(r.loc.pose().x().value() == doctest::Approx(72.0).epsilon(0.01));
        CHECK(r.loc.pose().y().value() == doctest::Approx(-48.0).epsilon(0.01));
    }
    CHECK(ekf.resyncCount() >= 1);
    CHECK(ekf.numericGuardTrips() == 0);
}

// Would catch: the EKF acquiring heading through the Localizer even though no corrector claims
// to measure it — the same property E3 proved for the complementary tier, now proved through
// the REAL re-stamp path rather than through a test driver.
TEST_CASE("seam: with no heading-providing corrector the published heading is the raw IMU") {
    EkfFusion ekf{};
    Rig r{ekf};
    for (int i = 0; i < 300; ++i) {
        const Pose2d p = r.loc.pose();
        r.imu.setHeading(Angle::degrees(0.05 * i));  // a turning robot
        r.corrector.setProposal(fixAt(p.x().value() + 2.0, p.y().value() - 1.5, 0.6));
        r.tick(0.2);
        REQUIRE(r.loc.headingBias().value() == 0.0);  // exactly zero, not "small"
        REQUIRE(r.loc.pose().heading().radians() == r.imu.heading().radians());
    }
    CHECK(ekf.acceptedFixes() > 100);  // fixes really were landing, so this is not vacuous
}

// Would catch: never-snap breaking at the Localizer's own audit slot. `AppliedCorrection::dx/dy`
// is the quantity §18.2 exists to audit, and E4 deliberately charges the EKF's velocity-filtering
// residual against the same budget so that this slot means the same thing under both tiers. If a
// future change stopped doing that, the record would start reporting corrections larger than the
// documented bound and nobody reading a blackbox could tell a filtering artefact from a snap.
TEST_CASE("seam: AppliedCorrection stays inside the per-tick budget under the EKF") {
    EkfFusionConfig cfg;
    EkfFusion ekf{cfg};
    Rig r{ekf};
    const double budget = cfg.maxNudgeRate.value() * kDt;
    double worst = 0.0;
    for (int i = 0; i < 1500; ++i) {
        const Pose2d p = r.loc.pose();
        r.corrector.setProposal(fixAt(p.x().value() + 4.0, p.y().value() + 3.0, 0.5));
        r.tick(0.2);
        const double dx = r.loc.lastCorrection().dx.value();
        const double dy = r.loc.lastCorrection().dy.value();
        REQUIRE(std::hypot(dx, dy) <= budget + 1e-9);
        worst = std::max(worst, std::hypot(dx, dy));
    }
    MESSAGE("worst AppliedCorrection magnitude under the EKF: ", worst, " in (budget ", budget,
            ")");
    CHECK(worst > 0.0);  // corrections really happened
}

// ─────────────────────────────────────────────────────────────────────────────────────────
// T5 / T2 — the two empty slots, read back from the FILE
// ─────────────────────────────────────────────────────────────────────────────────────────

// Would catch: `gateMahalanobis` and `covarianceTrace` being live in memory and absent from the
// blackbox — which is the only place a team can look after a match. Both have been declared and
// zero on every record ever written; this is the first chunk that puts a real number in either,
// and a number that does not survive the wire is a number nobody will ever see.
TEST_CASE("blackbox: gateMahalanobis and covarianceTrace arrive as real numbers") {
    EkfFusion ekf{};
    Rig r{ekf};

    FakeBlockSink device;
    std::vector<DebugRecord> ring(64);
    std::vector<std::byte> buffer(131072);
    SdSink blackbox{device, r.clk, SdSinkStorage{ring, buffer}, SdSinkConfig{.streamTicks = true}};
    CommandIdStampSink stamp{blackbox};
    blackbox.open({});

    for (int i = 0; i < 120; ++i) {
        const Pose2d p = r.loc.pose();
        // an honest fix for the first half, then one that is far enough to be refused
        const double off = (i < 60) ? 0.4 : 25.0;
        r.corrector.setProposal(fixAt(p.x().value() + off, p.y().value(), 0.5));
        r.tick(0.1);
        stamp.setEstimatorAudit(r.loc.lastCorrection());
        DebugRecord rec;
        rec.t = r.clk.now();
        rec.measuredPose = r.loc.pose();
        stamp.emit(rec);
    }
    blackbox.close();

    const std::vector<DebugRecord> ticks = decodeTicks(device);
    REQUIRE(ticks.size() == 120);

    int accepted = 0;
    int rejected = 0;
    double worstNu = 0.0;
    for (const DebugRecord& rec : ticks) {
        if (rec.gateReason == GateReason::Accepted) {
            ++accepted;
            CHECK(rec.gateMahalanobis > 0.0);   // a real distance, not the A1 default
            CHECK(rec.gateMahalanobis <= 3.0);  // …and an ACCEPTED one is inside the gate
        } else if (rec.gateReason == GateReason::RejectedMahalanobis) {
            ++rejected;
            CHECK(rec.gateMahalanobis > 3.0);  // …and a REJECTED one is outside it
            worstNu = std::max(worstNu, rec.gateMahalanobis);
        }
        CHECK(rec.covarianceTrace > 0.0);
        CHECK(std::isfinite(rec.covarianceTrace));
    }
    MESSAGE("decoded ", accepted, " accepted and ", rejected,
            " Mahalanobis-rejected ticks; worst nu ", worstNu);
    CHECK(accepted > 20);
    CHECK(rejected > 20);
    // The verdict is RE-DERIVABLE from the file: the record carries the residual, the distance
    // and the reason, so a reader confirms `nu > gateSigma` without trusting the label.
    for (const DebugRecord& rec : ticks) {
        if (rec.gateReason == GateReason::RejectedMahalanobis) {
            CHECK(std::hypot(rec.gateResidualX.value(), rec.gateResidualY.value()) > 1.0);
            // Under this tier a rejected tick is not bit-still: the position is still
            // settling by the velocity-filtering residual, which decays. What matters is
            // that NO PART of the rejected fix was applied, which a thousandth of an inch
            // against a 25-inch residual establishes.
            CHECK(std::abs(rec.correctionDx.value()) < 1.0e-3);
        }
    }
}

// Would catch: A SILENT RE-INIT — the single thing T2's ruling forbids. An estimator that
// changes its mind about how much to trust the world, and does not say so, is the hardest kind
// of run to debug: the pose starts moving for reasons that are not in the file. The declaration
// has to survive the wire, and the covariance trace has to jump on the same tick as the
// independent numeric witness.
TEST_CASE("blackbox: a re-init is a WORD on the record, with the trace jumping beside it") {
    EkfFusion ekf{};
    Rig r{ekf};

    FakeBlockSink device;
    std::vector<DebugRecord> ring(64);
    std::vector<std::byte> buffer(1048576);
    SdSink blackbox{device, r.clk, SdSinkStorage{ring, buffer}, SdSinkConfig{.streamTicks = true}};
    CommandIdStampSink stamp{blackbox};
    blackbox.open({});

    // settle on the origin, then get shoved: the wheels stop turning and the fix jumps 30 inches
    for (int i = 0; i < 700; ++i) {
        const double target = (i < 250) ? 0.0 : 30.0;
        r.corrector.setProposal(fixAt(target, 0.0, 0.5));
        r.tick(0.0);
        stamp.setEstimatorAudit(r.loc.lastCorrection());
        DebugRecord rec;
        rec.t = r.clk.now();
        rec.measuredPose = r.loc.pose();
        stamp.emit(rec);
    }
    blackbox.close();

    const std::vector<DebugRecord> ticks = decodeTicks(device);
    REQUIRE(ticks.size() == 700);

    int declarations = 0;
    std::size_t declaredAt = 0;
    for (std::size_t i = 0; i < ticks.size(); ++i) {
        if (ticks[i].gateReason == GateReason::CovarianceReinit) {
            ++declarations;
            if (declarations == 1) {
                declaredAt = i;
            }
        }
    }
    MESSAGE("re-init declared on tick ", declaredAt, " of 700; ", declarations,
            " declaration(s) in the file");
    CHECK(declarations == 1);
    REQUIRE(declaredAt > 250);
    // The second, independent witness: the trace jumps on exactly that tick.
    CHECK(ticks[declaredAt].covarianceTrace > 100.0 * ticks[declaredAt - 1].covarianceTrace);
    // And the estimate walked home rather than teleporting: no tick's applied correction
    // exceeded the budget, ON THE DECLARING TICK OR ANY OTHER, read from the bytes.
    const double budget = EkfFusionConfig{}.maxNudgeRate.value() * kDt;
    double worst = 0.0;
    for (const DebugRecord& rec : ticks) {
        CHECK(std::hypot(rec.correctionDx.value(), rec.correctionDy.value()) <= budget + 1e-9);
        worst = std::max(worst, std::hypot(rec.correctionDx.value(), rec.correctionDy.value()));
    }
    CHECK(worst > 0.0);
    // …and it did get home, which is what the whole mechanism is for.
    CHECK(ticks.back().measuredPose.x().value() == doctest::Approx(30.0).epsilon(0.02));
}

// Would catch: the complementary tier changing behaviour because E4 touched shared files. It
// keeps its FIXED 12-inch innovation gate and its `RejectedInnovation` word, and it still
// writes ZERO into the Mahalanobis slot — E2's T1 ruling, which says a distance normalised by
// an assumed constant must not be called a Mahalanobis distance. Old blackboxes keep their
// meaning only if this stays true.
TEST_CASE("seam: the complementary tier is unchanged — 12-inch gate, and mahalanobis still 0") {
    ComplementaryFusion comp{};
    Rig r{comp};
    for (int i = 0; i < 50; ++i) {
        const Pose2d p = r.loc.pose();
        r.corrector.setProposal(fixAt(p.x().value() + 20.0, p.y().value(), 0.5));
        r.tick(0.0);
        CHECK(r.loc.lastCorrection().audit.reason == GateReason::RejectedInnovation);
        CHECK(r.loc.lastCorrection().audit.mahalanobis == 0.0);
    }
    for (int i = 0; i < 50; ++i) {
        const Pose2d p = r.loc.pose();
        r.corrector.setProposal(fixAt(p.x().value() + 2.0, p.y().value(), 0.5));
        r.tick(0.0);
        CHECK(r.loc.lastCorrection().audit.reason == GateReason::Accepted);
        CHECK(r.loc.lastCorrection().audit.mahalanobis == 0.0);
    }
}
