// Tests for the absent-device ruling (chunk R3b §6): AbsentGps / AbsentTagSource /
// AbsentVision, and the §6.5 wiring rule that an absent source must never be polled.
//
// Adversarial focus, per test:
//   1. BIT-IDENTITY — a Localizer with a GpsCorrector over an AbsentGps must produce numbers
//      IDENTICAL (==, never Approx) to one with no corrector installed at all. This is the
//      test that proves the ruling: absence costs nothing and changes nothing. Bug caught: an
//      AbsentGps that leaks any influence into the estimate (a hasFix() that flips true, a
//      pose that drifts, an rmsError a gate ends up trusting).
//   2. INVARIANT SWEEP — hasFix() false, pose() finite, rmsError() finite AND non-negative,
//      on every one of many reads. Bug caught: a sentinel "no information" value that
//      violates the IGps contract (Inf, NaN, negative) and trips the tree's finite guards.
//   3. CORRECTOR NO-OP — an AprilTagCorrector polled over an AbsentTagSource never corrects
//      and never faults, WITH A MAPPED-TAG AMBUSH: the map deliberately contains the one tag
//      a fabricated observation would resolve against, placed so such a fix would be ACCEPTED
//      (innovation ~0). Bug caught: an AbsentTagSource that fabricates even one observation.
//      Without the ambush, a fabricated tag would bounce off the unmapped-id gate and the
//      test would stay green for the wrong reason.
//   4. THE TRAP IS REAL — a corrector polled over an AbsentTagSource is byte-indistinguishable
//      (every counter, the verdict, the anchor id) from one polled over a LIVE camera seeing
//      no tags. This is the EVIDENCE that §6.5's ruling is necessary: nothing downstream of
//      poll() can ever recover the difference, so the difference must be enforced at wiring
//      time, which is what test 5 checks.
//   5. THE WIRING RULE — kInstallTagCorrector / kInstallVisionPoller answer install for
//      present sources (a Fake IS a live source a test drives) and don't-install for Absent
//      ones, and a composition harness that follows the rule produces NO liveness record for
//      a robot with no camera. Bug caught: a composition root that installs and polls a
//      corrector over an absent source, manufacturing a false "camera alive, no tags" record.

#include "doctest.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "shulib/diag/debug_record.hpp"
#include "shulib/hal/absent_gps.hpp"
#include "shulib/hal/absent_tag_source.hpp"
#include "shulib/hal/absent_vision.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_imu.hpp"
#include "shulib/hal/fake/fake_rotation.hpp"
#include "shulib/hal/fake/fake_tag_source.hpp"
#include "shulib/hal/fake/fake_vision.hpp"
#include "shulib/localization/apriltag_corrector.hpp"
#include "shulib/localization/complementary_fusion.hpp"
#include "shulib/localization/gps_corrector.hpp"
#include "shulib/localization/i_corrector.hpp"
#include "shulib/localization/localizer.hpp"
#include "shulib/localization/pilons_odometry.hpp"
#include "shulib/localization/tag_map.hpp"
#include "shulib/localization/tracking_wheel.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/units/quantity.hpp"

using shulib::diag::GateReason;
using shulib::hal::AbsentGps;
using shulib::hal::AbsentTagSource;
using shulib::hal::AbsentVision;
using shulib::hal::kInstallTagCorrector;
using shulib::hal::kInstallVisionPoller;
using shulib::hal::fake::FakeClock;
using shulib::hal::fake::FakeImu;
using shulib::hal::fake::FakeRotation;
using shulib::hal::fake::FakeTagSource;
using shulib::hal::fake::FakeVision;
using shulib::localization::AprilTagCorrector;
using shulib::localization::ComplementaryFusion;
using shulib::localization::CorrectionProposal;
using shulib::localization::GpsCorrector;
using shulib::localization::GpsCorrectorConfig;
using shulib::localization::ICorrector;
using shulib::localization::Localizer;
using shulib::localization::PilonsOdometry;
using shulib::localization::TagMap;
using shulib::localization::TagPlacement;
using shulib::localization::TagProvenance;
using shulib::localization::TrackingWheel;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::units::AngleDim;
using shulib::units::Length;
using shulib::units::Time;

namespace {

constexpr double kDt = 0.01;

/// A GpsCorrectorConfig whose sensor-quality gate is wide open (maxReportedRms far above
/// AbsentGps::kNoInformationRmsInches). LOAD-BEARING for the mutation table, not a
/// convenience: with the default gate (6 in), a mutated `hasFix() == true` would be declined
/// as RejectedSensorQuality and the identity test would stay GREEN — a fake mutation. Wide
/// open, the test proves identity comes from hasFix() ALONE, not from a downstream gate
/// happening to save us, and mutation 1 actually bites (the one fold moves the fused pose by
/// ~1e-12 in, and the comparison below is on BITS, not Approx).
[[nodiscard]] GpsCorrectorConfig permissiveGpsConfig() {
    GpsCorrectorConfig cfg;
    cfg.maxReportedRms = Length{2.0 * AbsentGps::kNoInformationRmsInches};
    return cfg;
}

/// The localizer_test.cpp rig shape: diameter-2 wheels (radius 1), so a shaft reading in
/// radians is inches of forward travel, injected via fwdRot.setPosition(). No correctors.
struct PlainRig {
    FakeClock clk{Time{3.0}};  // NOT zero — an estimator assuming t0 == 0 would pass at 0
    FakeImu imu;
    FakeRotation fwdRot, latRot;
    PilonsOdometry odom;
    ComplementaryFusion fusion;
    Localizer loc;

    PlainRig()
        : odom{imu, TrackingWheel::forward(fwdRot, Length{2.0}, Length{0.0}),
               TrackingWheel::lateral(latRot, Length{2.0}, Length{0.0})},
          fusion{},
          loc{clk, imu, odom, fusion} {}

    void tick(double travelIn, double headingRad) {
        clk.advance(Time{kDt});
        imu.setHeading(Angle::radians(headingRad));
        fwdRot.setPosition(AngleDim{travelIn});
        loc.update();
    }
};

/// The same rig with ONE difference: a GpsCorrector wired over an AbsentGps. The identity
/// test's whole claim is that this difference is no difference at all.
struct AbsentGpsRig {
    FakeClock clk{Time{3.0}};
    FakeImu imu;
    FakeRotation fwdRot, latRot;
    AbsentGps gps;
    GpsCorrector corr;
    std::array<ICorrector*, 1> correctors;
    PilonsOdometry odom;
    ComplementaryFusion fusion;
    Localizer loc;

    AbsentGpsRig()
        : corr{clk, gps, imu, permissiveGpsConfig()},
          correctors{&corr},
          odom{imu, TrackingWheel::forward(fwdRot, Length{2.0}, Length{0.0}),
               TrackingWheel::lateral(latRot, Length{2.0}, Length{0.0})},
          fusion{},
          loc{clk, imu, odom, fusion, correctors} {}

    void tick(double travelIn, double headingRad) {
        clk.advance(Time{kDt});
        imu.setHeading(Angle::radians(headingRad));
        fwdRot.setPosition(AngleDim{travelIn});
        loc.update();
    }
};

}  // namespace

TEST_CASE("AbsentGps: absence costs nothing and changes nothing (bit-identity with no corrector)") {
    AbsentGpsRig a;
    PlainRig b;

    // Every step below is applied to BOTH rigs identically, and every tick asserts EXACT
    // (bit-level) equality — the one difference between the rigs is the corrector over the
    // AbsentGps, so any inequality is influence leaking out of "absent".
    auto tickBoth = [&](double travelIn, double headingRad) {
        a.tick(travelIn, headingRad);
        b.tick(travelIn, headingRad);
        CHECK(a.loc.pose().x().value() == b.loc.pose().x().value());
        CHECK(a.loc.pose().y().value() == b.loc.pose().y().value());
        CHECK(a.loc.pose().heading().radians() == b.loc.pose().heading().radians());
        CHECK(a.loc.isDeadReckoning() == b.loc.isDeadReckoning());
    };

    // A WITNESSED not-ready boot, then the settle window. Load-bearing for mutation 1, not
    // ceremony: correctors are first consulted on the first SETTLED tick, whose dt is healthy
    // (0.01 s) — consulted on tick 1 instead, dt would be 0, the fusion clamp would spend the
    // constant-sample AbsentGps's one-and-only fold on a zero nudge, and the mutation would
    // stay green for a reason unrelated to what this test proves.
    a.imu.setReady(false);
    b.imu.setReady(false);
    for (int i = 0; i < 5; ++i) {
        tickBoth(0.0, 0.0);
    }

    // A known offset from the AbsentGps's fixed origin, within the fusion policy's innovation
    // gate — so a mutated fix would be ACCEPTED, not conveniently rejected (the D5 standard).
    a.loc.setPose(Pose2d{Length{4.0}, Length{3.0}, Angle{}});
    b.loc.setPose(Pose2d{Length{4.0}, Length{3.0}, Angle{}});

    a.imu.setReady(true);
    b.imu.setReady(true);
    for (int i = 0; i < 15; ++i) {  // rides out bootSettleTime (0.1 s) with margin
        tickBoth(0.0, 0.0);
    }

    // Drive: accelerating forward travel with a slow sweep of heading — enough motion that
    // odometry, the heading fold and the corrector's own history all do real work.
    for (int i = 1; i <= 300; ++i) {
        const double t = static_cast<double>(i);
        tickBoth(0.35 * t, 0.004 * t);
    }
}

TEST_CASE("AbsentGps: the IGps contract holds on every read of a long sweep") {
    AbsentGps gps;

    // The fixed values, once, exactly: the origin pose and the documented constant. Exactness
    // matters — a pose that wanders or an rms that varies would break test 1's bit-identity.
    CHECK(gps.pose().x().value() == 0.0);
    CHECK(gps.pose().y().value() == 0.0);
    CHECK(gps.pose().heading().radians() == 0.0);
    CHECK(gps.rmsError().value() == AbsentGps::kNoInformationRmsInches);

    // The sweep. Not ceremony: finite_guard_test.cpp exists because this tree has been bitten
    // by non-finites, and rmsError()'s finite/non-negative rule is stated ON the seam.
    for (int i = 0; i < 2000; ++i) {
        CHECK(gps.hasFix() == false);
        CHECK(std::isfinite(gps.pose().x().value()));
        CHECK(std::isfinite(gps.pose().y().value()));
        CHECK(std::isfinite(gps.pose().heading().radians()));
        CHECK(std::isfinite(gps.rmsError().value()));
        CHECK(gps.rmsError().value() >= 0.0);
    }
}

TEST_CASE("AprilTagCorrector over AbsentTagSource: never corrects, never faults — against a mapped-tag ambush") {
    FakeClock clock{Time{10.0}};
    AbsentTagSource source;
    FakeImu imu;
    TagMap map;
    // THE AMBUSH: the map contains exactly the tag a fabricated observation would resolve
    // against — id 7, 24" straight ahead of the origin, facing back — so a fabricated
    // TagObservation{7, (24, 0, pi), 1.0} yields a fix AT the predicted pose: mapped, in the
    // trusted range band [6, 72], confidence 1.0, innovation ~0 → ACCEPTED. A fabricated tag
    // cannot hide behind the unmapped-id gate; "never corrects" fails the moment one exists.
    map.add(TagPlacement{.id = 7,
                         .fieldPose = Pose2d{Length{24.0}, Length{0.0}, Angle::radians(Angle::kPi)},
                         .provenance = TagProvenance::Invented,
                         .source = "R3b test ambush: placed so a fabricated tag would be ACCEPTED"});
    AprilTagCorrector corr{clock, source, imu, map};

    constexpr std::uint32_t kTicks = 500;
    const Pose2d predicted{};  // the origin — where the ambush fix would land
    for (std::uint32_t i = 0; i < kTicks; ++i) {
        clock.advance(Time{kDt});
        corr.poll();  // a fresh frame every tick, so nothing declines as merely stale
        const CorrectionProposal p = corr.propose(predicted, Time{kDt});
        CHECK(p.valid == false);
    }

    CHECK(corr.pollCount() == kTicks);
    CHECK(corr.acceptedFixes() == 0);
    CHECK(corr.noTagTicks() == kTicks);  // every frame read as "no tag in view"
    CHECK(corr.lastTagId() == -1);       // never anchored to anything
    CHECK(corr.droppedTags() == 0);
}

TEST_CASE("the §6.5 trap is real: polled absence is byte-indistinguishable from a live empty camera") {
    // Deliberately violates the wiring rule in a controlled scope: BOTH correctors are
    // installed and polled, one over an AbsentTagSource (a robot with NO camera), one over an
    // empty FakeTagSource (a LIVE camera with no tag in view). If every observable ends up
    // pairwise equal, then nothing downstream of poll() can ever tell the two robots apart —
    // which is exactly why the distinction must be enforced at wiring time (next test), and
    // why "just return an empty vector and let the corrector run" is not a harmless default.
    FakeClock clock{Time{10.0}};
    FakeImu imu;
    TagMap map;
    AbsentTagSource absent;
    FakeTagSource fake;  // default: no tags visible — a live camera seeing nothing
    AprilTagCorrector overAbsent{clock, absent, imu, map, {}, "absent"};
    AprilTagCorrector overFake{clock, fake, imu, map, {}, "fake"};

    const Pose2d predicted{};
    for (int i = 0; i < 200; ++i) {
        clock.advance(Time{kDt});
        overAbsent.poll();
        overFake.poll();
        const CorrectionProposal pa = overAbsent.propose(predicted, Time{kDt});
        const CorrectionProposal pf = overFake.propose(predicted, Time{kDt});
        CHECK(pa.valid == pf.valid);
        CHECK(pa.selfAudit.reason == pf.selfAudit.reason);
    }

    CHECK(overAbsent.pollCount() == overFake.pollCount());
    CHECK(overAbsent.noTagTicks() == overFake.noTagTicks());
    CHECK(overAbsent.noFrameTicks() == overFake.noFrameTicks());
    CHECK(overAbsent.staleTicks() == overFake.staleTicks());
    CHECK(overAbsent.acceptedFixes() == overFake.acceptedFixes());
    CHECK(overAbsent.lastVerdict() == overFake.lastVerdict());
    CHECK(overAbsent.lastTagId() == overFake.lastTagId());
    // The record both produced: "we looked N times, the camera was alive, no tag" — TRUE for
    // the FakeTagSource robot, FALSE for the AbsentTagSource robot. Same bytes, one lie.
    CHECK(overAbsent.noTagTicks() == 200);
}

namespace {

/// A composition harness that wires vision the way a composition root must: the corrector is
/// installed IF AND ONLY IF the LIBRARY's wiring rule says the declared source type is
/// pollable. The rule under test is shulib's kInstallTagCorrector — this harness only follows
/// it, so flipping the library's answer (mutation: specialize true for AbsentTagSource)
/// flips this harness's behavior and the checks below go red. The rule is NOT re-implemented
/// here; a test that asserts a rule only the test itself implements would prove nothing.
template <class SourceT>
struct VisionWiring {
    FakeClock clock{Time{5.0}};
    FakeImu imu;
    TagMap map;
    SourceT source;
    std::optional<AprilTagCorrector> corrector;  // installed only when the rule says install

    VisionWiring() {
        if constexpr (kInstallTagCorrector<SourceT>) {
            corrector.emplace(clock, source, imu, map);
        }
    }

    /// One vision-rate task tick: a task exists only where a corrector was installed.
    void visionTick() {
        clock.advance(Time{0.05});
        if (corrector.has_value()) {
            corrector->poll();
            (void)corrector->propose(Pose2d{}, Time{0.05});
        }
    }
};

}  // namespace

TEST_CASE("the §6.5 wiring rule: an absent source is never polled, so no false liveness record can exist") {
    // The rule itself, cited from the library. A Fake IS installable: it is a live source a
    // test drives, and present-but-empty is real information ("we looked, no tag"). Absent is
    // not: there is no camera to have looked with.
    CHECK(kInstallTagCorrector<FakeTagSource> == true);
    CHECK(kInstallTagCorrector<AbsentTagSource> == false);
    CHECK(kInstallVisionPoller<FakeVision> == true);
    CHECK(kInstallVisionPoller<AbsentVision> == false);

    // A robot WITH a (live, empty) camera: the corrector is installed, polled, and records
    // the true statement "we looked 50 times, the camera is alive, there was no tag".
    VisionWiring<FakeTagSource> withCamera;
    REQUIRE(withCamera.corrector.has_value());
    for (int i = 0; i < 50; ++i) {
        withCamera.visionTick();
    }
    CHECK(withCamera.corrector->pollCount() == 50);
    CHECK(withCamera.corrector->noTagTicks() == 50);

    // A robot with NO camera: the rule forbids installing the corrector at all, so the false
    // record "camera alive, no tags" CANNOT be produced — there is no counter to lie into.
    // Bug caught (mutation 5): a composition root that installs and drives the corrector
    // anyway; this harness would then emplace and poll it, and has_value() goes red.
    VisionWiring<AbsentTagSource> noCamera;
    CHECK_FALSE(noCamera.corrector.has_value());
    for (int i = 0; i < 50; ++i) {
        noCamera.visionTick();  // the vision task ticks; with no corrector there is no record
    }
    CHECK_FALSE(noCamera.corrector.has_value());
}
