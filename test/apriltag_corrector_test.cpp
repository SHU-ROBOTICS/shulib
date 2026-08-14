// Tests for localization/apriltag_corrector.hpp — the corrector's decisions in isolation (E3).
//
// The Localizer-level behaviour (accumulation, never-snap, two correctors, the blackbox) lives in
// apriltag_corrector_heading_test.cpp and apriltag_corrector_blackbox_test.cpp; this file drives
// propose() directly so each verdict can be provoked one at a time.
//
// ON THE GEOMETRY HELPER: `tagAsSeenFrom()` below composes a relative observation from a known
// robot pose, and the corrector inverts it with TagMap::robotPoseFromTag. Those two could in
// principle share a frame error and cancel — which is why the inversion is pinned INDEPENDENTLY
// in test/tag_map_test.cpp, by hand-worked literals and by a forward composition written there
// from scratch, BEFORE this file leans on it. The first case here is also worked entirely by hand.

#include "doctest.h"

#include <cmath>
#include <limits>
#include <vector>

#include "shulib/diag/debug_record.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_imu.hpp"
#include "shulib/hal/fake/fake_tag_source.hpp"
#include "shulib/hal/vision.hpp"
#include "shulib/localization/apriltag_corrector.hpp"
#include "shulib/localization/tag_map.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/units/quantity.hpp"

using shulib::diag::GateReason;
using shulib::hal::TagObservation;
using shulib::hal::fake::FakeClock;
using shulib::hal::fake::FakeImu;
using shulib::hal::fake::FakeTagSource;
using shulib::localization::AprilTagCorrector;
using shulib::localization::AprilTagCorrectorConfig;
using shulib::localization::CorrectionProposal;
using shulib::localization::TagMap;
using shulib::localization::TagPlacement;
using shulib::localization::TagProvenance;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::units::AngularVelocity;
using shulib::units::Length;
using shulib::units::Time;

namespace {

/// Where a tag at field pose `tag` appears in the body frame of a robot at field pose `robot`.
/// The inverse of the rigid composition, written out here:
///     rx =  dx·cos(Rθ) + dy·sin(Rθ)
///     ry = -dx·sin(Rθ) + dy·cos(Rθ)
///     rθ =  Tθ − Rθ
[[nodiscard]] Pose2d tagAsSeenFrom(const Pose2d& robot, const Pose2d& tag) {
    const double dx = tag.x().value() - robot.x().value();
    const double dy = tag.y().value() - robot.y().value();
    const double c = std::cos(robot.heading().radians());
    const double s = std::sin(robot.heading().radians());
    return Pose2d{Length{dx * c + dy * s}, Length{-dx * s + dy * c},
                  Angle::radians(tag.heading().radians() - robot.heading().radians())};
}

/// A tag placed `range` inches straight ahead of `robot`, facing straight back at it.
[[nodiscard]] Pose2d tagAhead(const Pose2d& robot, double range) {
    return Pose2d{Length{robot.x().value() + range * std::cos(robot.heading().radians())},
                  Length{robot.y().value() + range * std::sin(robot.heading().radians())},
                  Angle::radians(robot.heading().radians() + Angle::kPi)};
}

/// The standard rig: clock, tag source, IMU, a one-tag map, and the corrector over them.
struct Rig {
    FakeClock clock{Time{10.0}};  // NOT zero — a corrector that assumed t0 == 0 would pass anyway
    FakeTagSource source;
    FakeImu imu;
    TagMap map;
    AprilTagCorrectorConfig config{};

    /// Truth pose the scenes are built around: neither the origin nor heading 0.
    static Pose2d truth() { return Pose2d{Length{20.0}, Length{15.0}, Angle::degrees(40.0)}; }

    void mapTag(int id, const Pose2d& fieldPose) {
        map.add(TagPlacement{id, fieldPose, TagProvenance::Invented,
                             "host test fixture — not a field layout"});
    }
    [[nodiscard]] AprilTagCorrector make() {
        return AprilTagCorrector{clock, source, imu, map, config};
    }
};

}  // namespace

// ── the silences, which must all be different words ──────────────────────────────────────────

// Would catch: a never-polled corrector looking like an idle estimator instead of a wiring
// mistake. This is the footgun the two-method shape (T4) creates, and it must be diagnosable
// from the very first tick, not after a timeout.
TEST_CASE("AprilTagCorrector: never polled -> RejectedNoFix, immediately and forever") {
    Rig rig;
    rig.mapTag(1, tagAhead(Rig::truth(), 30.0));
    AprilTagCorrector corrector = rig.make();
    // A tag IS visible; nobody has polled, so the corrector cannot know.
    rig.source.setTags({TagObservation{1, tagAsSeenFrom(Rig::truth(), tagAhead(Rig::truth(), 30.0)),
                                       0.9}});

    for (int k = 0; k < 5; ++k) {
        const CorrectionProposal p = corrector.propose(Rig::truth(), Time{0.01});
        CHECK_FALSE(p.valid);
        CHECK(p.selfAudit.reason == GateReason::RejectedNoFix);
        CHECK(p.confidence == 0.0);  // NEVER a low-confidence pull
        rig.clock.advance(Time{0.01});
    }
    CHECK(corrector.pollCount() == 0);
    CHECK(corrector.noFrameTicks() == 5);
}

// Would catch: "the camera is alive and sees nothing" being confused with "the camera is dead",
// which call for completely different responses — one is normal driving, the other is a failed
// subsystem — and, worse, a no-tag tick producing a phantom pull toward some default pose.
TEST_CASE("AprilTagCorrector: polled with no tag in view -> RejectedNoFix, never a phantom pull") {
    Rig rig;
    rig.mapTag(1, tagAhead(Rig::truth(), 30.0));
    AprilTagCorrector corrector = rig.make();
    rig.source.clear();
    corrector.poll();

    const CorrectionProposal p = corrector.propose(Rig::truth(), Time{0.01});
    CHECK_FALSE(p.valid);
    CHECK(p.selfAudit.reason == GateReason::RejectedNoFix);
    CHECK(p.confidence == 0.0);
    CHECK(p.fieldPose.x().value() == 0.0);  // no pose at all, not a pose toward (0,0)
    CHECK(p.fieldPose.y().value() == 0.0);
    CHECK(corrector.pollCount() == 1);
    CHECK(corrector.noTagTicks() == 1);
    CHECK(corrector.noFrameTicks() == 0);  // NOT the never-polled word
}

// Would catch: a vision task that died mid-match going unnoticed. Without this, a stopped poller
// looks identical to an already-folded frame (RejectedStaleFix), which is the NORMAL steady
// state at every tick — so the failure would hide inside the expected noise for the whole run.
TEST_CASE("AprilTagCorrector: a stopped poller -> RejectedObservationAge, its own word") {
    Rig rig;
    const Pose2d tag = tagAhead(Rig::truth(), 30.0);
    rig.mapTag(1, tag);
    AprilTagCorrector corrector = rig.make();
    rig.source.setTags({TagObservation{1, tagAsSeenFrom(Rig::truth(), tag), 0.9}});
    corrector.poll();
    REQUIRE(corrector.propose(Rig::truth(), Time{0.01}).valid);  // the frame was good

    // Now the poller stops. Just inside the horizon: still merely "already folded".
    rig.clock.advance(Time{0.2});
    CHECK(corrector.propose(Rig::truth(), Time{0.01}).selfAudit.reason ==
          GateReason::RejectedStaleFix);
    // Past it: a different, louder word.
    rig.clock.advance(Time{0.1});
    const CorrectionProposal p = corrector.propose(Rig::truth(), Time{0.01});
    CHECK_FALSE(p.valid);
    CHECK(p.selfAudit.reason == GateReason::RejectedObservationAge);
    CHECK(corrector.staleFrameTicks() == 1);
}

// Would catch: one observation being folded once per control tick. Vision runs at ~20 Hz against
// a ~100 Hz loop, so folding every tick would count a single measurement five times and make the
// correction strength depend on the ratio of loop rate to camera cadence (E2's D3, same hazard).
TEST_CASE("AprilTagCorrector: one frame is folded exactly once") {
    Rig rig;
    const Pose2d tag = tagAhead(Rig::truth(), 30.0);
    rig.mapTag(1, tag);
    AprilTagCorrector corrector = rig.make();
    rig.source.setTags({TagObservation{1, tagAsSeenFrom(Rig::truth(), tag), 0.9}});
    corrector.poll();

    CHECK(corrector.propose(Rig::truth(), Time{0.01}).valid);
    for (int k = 0; k < 4; ++k) {
        rig.clock.advance(Time{0.01});
        const CorrectionProposal p = corrector.propose(Rig::truth(), Time{0.01});
        CHECK_FALSE(p.valid);
        CHECK(p.selfAudit.reason == GateReason::RejectedStaleFix);
    }
    CHECK(corrector.acceptedFixes() == 1);
    CHECK(corrector.staleTicks() == 4);

    corrector.poll();  // a NEW frame is new information, even with identical contents
    CHECK(corrector.propose(Rig::truth(), Time{0.01}).valid);
    CHECK(corrector.acceptedFixes() == 2);
}

// ── the tag map's own verdicts ───────────────────────────────────────────────────────────────

// Would catch: an unknown tag id being silently ignored, which makes a configuration error
// (a missing map entry, or an empty map) indistinguishable from the robot simply not seeing a
// tag — and the missing entry is the one the team can actually fix.
TEST_CASE("AprilTagCorrector: a tag with no map entry -> RejectedNoTagMapEntry, not NoFix") {
    Rig rig;
    const Pose2d tag = tagAhead(Rig::truth(), 30.0);

    SUBCASE("an id that is not in a populated map") {
        rig.mapTag(1, tag);
        AprilTagCorrector corrector = rig.make();
        rig.source.setTags({TagObservation{99, tagAsSeenFrom(Rig::truth(), tag), 0.9}});
        corrector.poll();
        const CorrectionProposal p = corrector.propose(Rig::truth(), Time{0.01});
        CHECK_FALSE(p.valid);
        CHECK(p.selfAudit.reason == GateReason::RejectedNoTagMapEntry);
        CHECK(corrector.unmappedRejects() == 1);
    }
    SUBCASE("an empty map — the default, since shulib ships no field layout") {
        AprilTagCorrector corrector = rig.make();
        rig.source.setTags({TagObservation{1, tagAsSeenFrom(Rig::truth(), tag), 0.9}});
        corrector.poll();
        CHECK(corrector.propose(Rig::truth(), Time{0.01}).selfAudit.reason ==
              GateReason::RejectedNoTagMapEntry);
    }
}

// Would catch: a missing map entry being outranked by a range or confidence rejection. The
// priority is documented, so it must be enforced: a configuration error the team can fix beats
// the field simply being the field.
TEST_CASE("AprilTagCorrector: an unmapped tag outranks range and confidence rejections") {
    Rig rig;
    const Pose2d nearTag = tagAhead(Rig::truth(), 2.0);  // inside minRange
    rig.mapTag(1, nearTag);
    AprilTagCorrector corrector = rig.make();
    rig.source.setTags({
        TagObservation{1, tagAsSeenFrom(Rig::truth(), nearTag), 0.9},  // in the map, too close
        TagObservation{42, tagAsSeenFrom(Rig::truth(), tagAhead(Rig::truth(), 30.0)), 0.9},
    });
    corrector.poll();
    CHECK(corrector.propose(Rig::truth(), Time{0.01}).selfAudit.reason ==
          GateReason::RejectedNoTagMapEntry);
}

// ── the quality band ─────────────────────────────────────────────────────────────────────────

// Would catch: the trusted range band existing only in the header. Outside it, planar PnP's
// heading is untrustworthy long before its position is — this corrector answers that with a
// band rather than a second noise model, so the band had better be real.
TEST_CASE("AprilTagCorrector: outside the trusted range band -> RejectedTagRange, both ends") {
    Rig rig;
    SUBCASE("too close (below minRange)") {
        const Pose2d tag = tagAhead(Rig::truth(), 3.0);
        rig.mapTag(1, tag);
        AprilTagCorrector corrector = rig.make();
        rig.source.setTags({TagObservation{1, tagAsSeenFrom(Rig::truth(), tag), 0.95}});
        corrector.poll();
        CHECK(corrector.propose(Rig::truth(), Time{0.01}).selfAudit.reason ==
              GateReason::RejectedTagRange);
    }
    SUBCASE("too far (above maxRange)") {
        const Pose2d tag = tagAhead(Rig::truth(), 100.0);
        rig.mapTag(1, tag);
        AprilTagCorrector corrector = rig.make();
        rig.source.setTags({TagObservation{1, tagAsSeenFrom(Rig::truth(), tag), 0.95}});
        corrector.poll();
        CHECK(corrector.propose(Rig::truth(), Time{0.01}).selfAudit.reason ==
              GateReason::RejectedTagRange);
        CHECK(corrector.rangeRejects() == 1);
    }
    SUBCASE("just inside both ends is accepted") {
        const Pose2d nearEdge = tagAhead(Rig::truth(), 6.5);
        const Pose2d farEdge = tagAhead(Rig::truth(), 71.5);
        rig.mapTag(1, nearEdge);
        rig.mapTag(2, farEdge);
        AprilTagCorrector corrector = rig.make();
        rig.source.setTags({TagObservation{1, tagAsSeenFrom(Rig::truth(), nearEdge), 0.9}});
        corrector.poll();
        CHECK(corrector.propose(Rig::truth(), Time{0.01}).valid);
        rig.source.setTags({TagObservation{2, tagAsSeenFrom(Rig::truth(), farEdge), 0.9}});
        corrector.poll();
        CHECK(corrector.propose(Rig::truth(), Time{0.01}).valid);
    }
}

// Would catch: a barely-detected tag still being folded. Without a floor, a 0.05-confidence
// detection produces a microscopic pull — but the Localizer sees `applied`, reports quality class
// Corrected, and a run with no usable anchor reads as anchored. That last consequence is why the
// floor exists at all (E2's D7, same argument).
TEST_CASE("AprilTagCorrector: a detection below the confidence floor -> RejectedSensorQuality") {
    Rig rig;
    const Pose2d tag = tagAhead(Rig::truth(), 30.0);
    rig.mapTag(1, tag);
    AprilTagCorrector corrector = rig.make();
    rig.source.setTags({TagObservation{1, tagAsSeenFrom(Rig::truth(), tag), 0.2}});
    corrector.poll();
    CHECK(corrector.propose(Rig::truth(), Time{0.01}).selfAudit.reason ==
          GateReason::RejectedSensorQuality);
    CHECK(corrector.qualityRejects() == 1);

    rig.source.setTags({TagObservation{1, tagAsSeenFrom(Rig::truth(), tag), 0.36}});
    corrector.poll();
    CHECK(corrector.propose(Rig::truth(), Time{0.01}).valid);  // just above the floor
}

// Would catch: a non-finite observation reaching the arithmetic — a NaN pose would propagate
// silently through the complementary filter and poison the estimate permanently. It must be
// screened as a quality failure, and it must NOT throw: propose() runs inside the control loop.
TEST_CASE("AprilTagCorrector: a non-finite observation is declined, not thrown and not folded") {
    Rig rig;
    const Pose2d tag = tagAhead(Rig::truth(), 30.0);
    rig.mapTag(1, tag);
    AprilTagCorrector corrector = rig.make();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    rig.source.setTags(
        {TagObservation{1, Pose2d{Length{nan}, Length{0.0}, Angle::degrees(180.0)}, 0.9}});
    corrector.poll();

    CorrectionProposal p;
    CHECK_NOTHROW(p = corrector.propose(Rig::truth(), Time{0.01}));
    CHECK_FALSE(p.valid);
    CHECK(p.selfAudit.reason == GateReason::RejectedSensorQuality);

    rig.source.setTags({TagObservation{1, tagAsSeenFrom(Rig::truth(), tag),
                                       std::numeric_limits<double>::infinity()}});
    corrector.poll();
    CHECK_NOTHROW(p = corrector.propose(Rig::truth(), Time{0.01}));
    CHECK_FALSE(p.valid);
}

// Would catch: a fix taken mid-spin being trusted. A spinning robot smears the tag across the
// frame and a rolling shutter skews it into a different quadrilateral, which PnP solves happily
// into a confidently wrong pose. Both signs, because a threshold on a signed rate is a classic
// one-sided bug.
TEST_CASE("AprilTagCorrector: a fix taken while spinning fast is declined, either direction") {
    for (const double rate : {3.0, -3.0}) {
        Rig rig;
        const Pose2d tag = tagAhead(Rig::truth(), 30.0);
        rig.mapTag(1, tag);
        AprilTagCorrector corrector = rig.make();
        rig.imu.setYawRate(AngularVelocity{rate});
        rig.source.setTags({TagObservation{1, tagAsSeenFrom(Rig::truth(), tag), 0.9}});
        corrector.poll();
        const CorrectionProposal p = corrector.propose(Rig::truth(), Time{0.01});
        CHECK_FALSE(p.valid);
        CHECK(p.selfAudit.reason == GateReason::RejectedHighYawRate);
        CHECK(corrector.yawRateRejects() == 1);
    }
}

// Would catch: a rejected frame being folded LATER, once the spin stops. It describes a moment
// already judged untrustworthy; re-using it applies a bad measurement to a newer pose (E2's D8).
TEST_CASE("AprilTagCorrector: a frame rejected mid-spin is consumed, not folded after the spin") {
    Rig rig;
    const Pose2d tag = tagAhead(Rig::truth(), 30.0);
    rig.mapTag(1, tag);
    AprilTagCorrector corrector = rig.make();
    rig.imu.setYawRate(AngularVelocity{3.0});
    rig.source.setTags({TagObservation{1, tagAsSeenFrom(Rig::truth(), tag), 0.9}});
    corrector.poll();
    CHECK_FALSE(corrector.propose(Rig::truth(), Time{0.01}).valid);

    rig.imu.setYawRate(AngularVelocity{0.0});  // the spin ends; the frame is still the old one
    rig.clock.advance(Time{0.01});
    const CorrectionProposal p = corrector.propose(Rig::truth(), Time{0.01});
    CHECK_FALSE(p.valid);
    CHECK(p.selfAudit.reason == GateReason::RejectedStaleFix);
    CHECK(corrector.acceptedFixes() == 0);
}

// ── the numbers ──────────────────────────────────────────────────────────────────────────────

// Would catch: sigma or the confidence being computed from the wrong quantity — the values that
// decide how hard every fix pulls. HAND-COMPUTED, so the assertion is arithmetic and not a
// second copy of the formula:
//     range = 30, confidence = 0.8, base = 1.0, perInch = 0.02
//     sigma_meas = (1.0 + 0.02·30) / 0.8 = 1.6 / 0.8 = 2.0
//     sigma_dr   = hypot(1.0, 0.02·0)    = 1.0            (no travel since the fix)
//     sigma_eff  = hypot(2.0, 1.0)       = sqrt(5) = 2.2360679774997896
//     confidence = 1.0 / (1.0 + 4.0)     = 0.2
TEST_CASE("AprilTagCorrector: sigma and confidence are the hand-computed values") {
    Rig rig;
    const Pose2d tag = tagAhead(Rig::truth(), 30.0);
    rig.mapTag(1, tag);
    AprilTagCorrector corrector = rig.make();
    rig.source.setTags({TagObservation{1, tagAsSeenFrom(Rig::truth(), tag), 0.8}});
    corrector.poll();

    const CorrectionProposal p = corrector.propose(Rig::truth(), Time{0.01});
    REQUIRE(p.valid);
    CHECK(p.positionStdDev.value() == doctest::Approx(2.2360679774997896).epsilon(1e-12));
    CHECK(p.confidence == doctest::Approx(0.2).epsilon(1e-12));
    CHECK(p.providesHeading);  // the whole point of this corrector
}

// Would catch: sigma responding to range or confidence in the wrong direction — a corrector that
// trusted a distant, poorly-detected tag MORE than a near, crisp one. A shape assertion, not a
// constant: E3 proves the logic, R4 measures the magnitudes.
TEST_CASE("AprilTagCorrector: sigma grows with range and shrinks with detector confidence") {
    auto sigmaFor = [](double range, double conf) {
        Rig rig;
        const Pose2d tag = tagAhead(Rig::truth(), range);
        rig.mapTag(1, tag);
        AprilTagCorrector corrector = rig.make();
        rig.source.setTags({TagObservation{1, tagAsSeenFrom(Rig::truth(), tag), conf}});
        corrector.poll();
        const CorrectionProposal p = corrector.propose(Rig::truth(), Time{0.01});
        REQUIRE(p.valid);
        return p.positionStdDev.value();
    };
    CHECK(sigmaFor(60.0, 0.9) > sigmaFor(20.0, 0.9));  // farther is worse
    CHECK(sigmaFor(30.0, 0.4) > sigmaFor(30.0, 0.95)); // less certain is worse
}

// Would catch: multi-tag selection picking arbitrarily (or picking the LAST one seen). With two
// tags in view the corrector must anchor to the better one, and must say WHICH — the first
// question anybody asks when a fix looks wrong.
TEST_CASE("AprilTagCorrector: with several tags in view, the smallest-sigma one wins") {
    Rig rig;
    const Pose2d near = tagAhead(Rig::truth(), 20.0);
    const Pose2d far = tagAhead(Rig::truth(), 65.0);
    rig.mapTag(1, far);
    rig.mapTag(2, near);
    AprilTagCorrector corrector = rig.make();

    SUBCASE("equal confidence: the nearer tag") {
        rig.source.setTags({TagObservation{1, tagAsSeenFrom(Rig::truth(), far), 0.9},
                            TagObservation{2, tagAsSeenFrom(Rig::truth(), near), 0.9}});
        corrector.poll();
        REQUIRE(corrector.propose(Rig::truth(), Time{0.01}).valid);
        CHECK(corrector.lastTagId() == 2);
    }
    SUBCASE("a much better detection beats a slightly nearer one") {
        rig.source.setTags({TagObservation{1, tagAsSeenFrom(Rig::truth(), far), 1.0},
                            TagObservation{2, tagAsSeenFrom(Rig::truth(), near), 0.36}});
        corrector.poll();
        REQUIRE(corrector.propose(Rig::truth(), Time{0.01}).valid);
        // sigma(far)  = (1 + 0.02·65) / 1.00  = 2.30
        // sigma(near) = (1 + 0.02·20) / 0.36  = 3.888...
        CHECK(corrector.lastTagId() == 1);
    }
}

// ── the fix itself ───────────────────────────────────────────────────────────────────────────

// Would catch: the absolute pose being wrong in position or heading. HAND-CHECKED end to end: a
// robot at (20, 15) facing 40 degrees sees tag 1, which the map places 30 inches ahead of it
// facing back. The proposed field pose must be the robot's own, to the last digit — and the
// heading must be the TAG-DERIVED one, which is what makes this corrector different in kind from
// E2's.
TEST_CASE("AprilTagCorrector: the proposal is the robot's absolute pose, heading included") {
    Rig rig;
    const Pose2d tag = tagAhead(Rig::truth(), 30.0);
    rig.mapTag(1, tag);
    AprilTagCorrector corrector = rig.make();
    rig.source.setTags({TagObservation{1, tagAsSeenFrom(Rig::truth(), tag), 0.9}});
    corrector.poll();

    // The prediction is WRONG by 2 inches and 3 degrees; the proposal must ignore it entirely and
    // report what the tag says — otherwise the "correction" would just echo the error back.
    const Pose2d predicted{Length{22.0}, Length{15.0}, Angle::degrees(43.0)};
    const CorrectionProposal p = corrector.propose(predicted, Time{0.01});
    REQUIRE(p.valid);
    CHECK(p.fieldPose.x().value() == doctest::Approx(20.0).epsilon(1e-9));
    CHECK(p.fieldPose.y().value() == doctest::Approx(15.0).epsilon(1e-9));
    CHECK(p.fieldPose.heading().degrees() == doctest::Approx(40.0).epsilon(1e-9));
    CHECK(p.providesHeading);
    CHECK(corrector.lastVerdict() == GateReason::Accepted);
    CHECK(corrector.lastTagId() == 1);
    // selfAudit stays empty on an ACCEPTED proposal: the fusion policy owns the audit for
    // anything that reaches it, and a corrector claiming Accepted here could be substituted onto
    // a tick where the Localizer screened the proposal out (E2's rule, still load-bearing).
    CHECK(p.selfAudit.reason == GateReason::None);
}

// ── the gate ─────────────────────────────────────────────────────────────────────────────────

// Would catch: the innovation gate being inverted, or normalizing by the wrong sigma. The
// boundary is hand-computed: sigma_eff = sqrt(5), gateSigma = 4, so the widest accepted residual
// is 4·sqrt(5) = 8.94427190999916 inches.
TEST_CASE("AprilTagCorrector: the normalized-innovation gate boundary is where the arithmetic says") {
    const Pose2d tag = tagAhead(Rig::truth(), 30.0);
    auto proposeWithOffset = [&](double offset) {
        Rig rig;
        rig.mapTag(1, tag);
        AprilTagCorrector corrector = rig.make();
        rig.source.setTags({TagObservation{1, tagAsSeenFrom(Rig::truth(), tag), 0.8}});
        corrector.poll();
        const Pose2d predicted{Length{20.0 + offset}, Length{15.0}, Angle::degrees(40.0)};
        return corrector.propose(predicted, Time{0.01});
    };
    CHECK(proposeWithOffset(8.9).valid);
    const CorrectionProposal rejected = proposeWithOffset(9.0);
    CHECK_FALSE(rejected.valid);
    CHECK(rejected.selfAudit.reason == GateReason::RejectedNormalizedInnovation);
    // The rejection carries the numbers the verdict was rendered on, so a reader can recompute
    // nu = |residual| / sigma from the blackbox alone.
    CHECK(rejected.selfAudit.residualX.value() == doctest::Approx(-9.0).epsilon(1e-9));
    CHECK(rejected.selfAudit.covarianceTrace ==
          doctest::Approx(2.2360679774997896).epsilon(1e-9));
}

// Would catch: gate lockout — the failure E2's D2 exists to prevent, re-created here because the
// same sigma model is in play. After a long blind stretch a TRUTHFUL fix looks outrageous and is
// rejected, and so is every fix after it, because nothing else can repair the estimate. The two
// rigs differ ONLY in driftStdDevPerInch.
TEST_CASE("AprilTagCorrector: the gate widens with dead-reckoned travel (anti-lockout)") {
    // Latency compensation is switched off here on purpose: it has its own cases below, and
    // leaving it on would mix a real 8-inch carry-forward into a test about GATE WIDTH.
    const double c = std::cos(40.0 * Angle::kPi / 180.0);
    const double s = std::sin(40.0 * Angle::kPi / 180.0);
    // The estimate dead-reckons 399 inches along its own heading, then a TRUTHFUL fix arrives
    // saying it is 15 inches further along than it believes.
    const double travelled = 399.0;
    const Pose2d truthFinal{Length{20.0 + (travelled + 15.0) * c},
                            Length{15.0 + (travelled + 15.0) * s}, Angle::degrees(40.0)};
    const Pose2d tag = tagAhead(truthFinal, 30.0);

    auto runWithGrowth = [&](double growth) {
        Rig rig;
        rig.config.driftStdDevPerInch = growth;
        rig.config.latency = Time{0.0};
        rig.mapTag(1, tag);
        AprilTagCorrector corrector = rig.make();
        Pose2d predicted = Rig::truth();
        for (int k = 0; k <= 399; ++k) {
            const double d = static_cast<double>(k);
            predicted = Pose2d{Length{20.0 + d * c}, Length{15.0 + d * s}, Angle::degrees(40.0)};
            (void)corrector.propose(predicted, Time{0.01});
            rig.clock.advance(Time{0.01});
        }
        REQUIRE(corrector.travelSinceFix().value() == doctest::Approx(travelled).epsilon(1e-9));
        rig.source.setTags({TagObservation{1, tagAsSeenFrom(truthFinal, tag), 0.8}});
        corrector.poll();
        return corrector.propose(predicted, Time{0.01});
    };
    // sigma_meas = 2.0 either way. With no widening sigma_eff = sqrt(5) and the gate is
    // 4·sqrt(5) = 8.94 inches — narrower than the truthful 15-inch fix, so the estimator is
    // locked out exactly when a correction is worth the most.
    CHECK_FALSE(runWithGrowth(0.0).valid);
    // With widening, sigma_dr = hypot(1, 0.02·399) = 8.04, sigma_eff = 8.29, gate = 33.2 inches.
    CHECK(runWithGrowth(0.02).valid);
}

// Would catch: the anti-lockout accumulator never resetting, which leaves the gate permanently
// wide after one blind stretch — so a LIE is accepted afterwards because the corrector still
// believes it is lost.
TEST_CASE("AprilTagCorrector: the travel accumulator resets on an accepted fix") {
    const double c = std::cos(40.0 * Angle::kPi / 180.0);
    const double s = std::sin(40.0 * Angle::kPi / 180.0);
    const Pose2d atEnd{Length{20.0 + 49.0 * c}, Length{15.0 + 49.0 * s}, Angle::degrees(40.0)};
    const Pose2d tag = tagAhead(atEnd, 30.0);

    Rig rig;
    rig.config.latency = Time{0.0};
    rig.mapTag(1, tag);
    AprilTagCorrector corrector = rig.make();
    Pose2d predicted = Rig::truth();
    for (int k = 0; k <= 49; ++k) {
        const double d = static_cast<double>(k);
        predicted = Pose2d{Length{20.0 + d * c}, Length{15.0 + d * s}, Angle::degrees(40.0)};
        (void)corrector.propose(predicted, Time{0.01});
        rig.clock.advance(Time{0.01});
    }
    CHECK(corrector.travelSinceFix().value() == doctest::Approx(49.0));

    rig.source.setTags({TagObservation{1, tagAsSeenFrom(atEnd, tag), 0.8}});
    corrector.poll();
    REQUIRE(corrector.propose(predicted, Time{0.01}).valid);
    CHECK(corrector.travelSinceFix().value() == 0.0);
}

// ── latency ──────────────────────────────────────────────────────────────────────────────────

// Would catch: latency compensation dropped, or applied backwards. A fix describes where the
// robot WAS ~80 ms ago; applied as if it described NOW it drags the estimate backwards along the
// direction of travel — a systematic lag larger than the sensor's own noise.
//
// THE EXACT CASE. The robot moves 1 inch per tick along +X for 20 ticks, the frame is captured 8
// ticks (80 ms) before it is read, and the tag says the robot was at x = 20 at capture. The
// compensated measurement must be the CURRENT x = 28, not 20.
TEST_CASE("AprilTagCorrector: a fix is carried forward by the odometry travelled since capture") {
    Rig rig;
    rig.config.driftStdDevPerInch = 0.05;  // keep the gate open across the deliberate 8-inch lag
    const Pose2d capturePose{Length{20.0}, Length{15.0}, Angle::degrees(0.0)};
    const Pose2d tag = tagAhead(capturePose, 30.0);
    rig.mapTag(1, tag);
    AprilTagCorrector corrector = rig.make();

    Pose2d predicted = capturePose;
    for (int k = 0; k < 9; ++k) {  // ticks at t = 10.00 .. 10.08; the frame describes t = 10.00
        predicted = Pose2d{Length{20.0 + static_cast<double>(k)}, Length{15.0}, Angle::degrees(0.0)};
        (void)corrector.propose(predicted, Time{0.01});
        if (k < 8) {
            rig.clock.advance(Time{0.01});
        }
    }
    REQUIRE(predicted.x().value() == doctest::Approx(28.0));

    // The frame is read NOW (t = 10.08) but describes t = 10.00, where the robot was at x = 20.
    rig.source.setTags({TagObservation{1, tagAsSeenFrom(capturePose, tag), 0.9}});
    corrector.poll();
    const CorrectionProposal p = corrector.propose(predicted, Time{0.01});
    REQUIRE(p.valid);
    CHECK(p.fieldPose.x().value() == doctest::Approx(28.0).epsilon(1e-9));  // NOT 20
}

// Would catch: heading latency compensation being absent — the half nobody thinks of. A tag fix
// describes which way the robot was pointing 80 ms ago; at a brisk 180 deg/s that is 14 degrees,
// fourteen times the entire heading error budget, and it would be folded as if it were current.
//
// THE EXACT CASE. The robot rotates 1 degree per tick. The frame is captured 8 ticks ago, when
// the robot faced 40 degrees; by now it faces 48. The compensated measurement must be 48.
TEST_CASE("AprilTagCorrector: heading is latency-compensated too, not just position") {
    Rig rig;
    const Pose2d capturePose{Length{20.0}, Length{15.0}, Angle::degrees(40.0)};
    const Pose2d tag = tagAhead(capturePose, 30.0);
    rig.mapTag(1, tag);
    AprilTagCorrector corrector = rig.make();

    Pose2d predicted = capturePose;
    for (int k = 0; k < 9; ++k) {
        // The IMU turns with the robot: it is the authority on ROTATION, and it is where the
        // corrector reads the "how far have we turned since capture" term from (deliberately —
        // reading it from the predicted heading would feed the yaw correction back into itself).
        rig.imu.setHeading(Angle::degrees(40.0 + static_cast<double>(k)));
        predicted = Pose2d{Length{20.0}, Length{15.0},
                           Angle::degrees(40.0 + static_cast<double>(k))};
        (void)corrector.propose(predicted, Time{0.01});
        if (k < 8) {
            rig.clock.advance(Time{0.01});
        }
    }
    REQUIRE(predicted.heading().degrees() == doctest::Approx(48.0));

    rig.source.setTags({TagObservation{1, tagAsSeenFrom(capturePose, tag), 0.9}});
    corrector.poll();
    const CorrectionProposal p = corrector.propose(predicted, Time{0.01});
    REQUIRE(p.valid);
    CHECK(p.fieldPose.heading().degrees() == doctest::Approx(48.0).epsilon(1e-9));  // NOT 40
}

// Would catch: the heading history interpolating RAW wrapped angles, which produces a garbage
// rotation exactly once per revolution — at the ±180 degree seam. The corrector stores an
// UNWRAPPED cumulative heading for this reason; if it did not, this case would compensate by
// roughly -352 degrees instead of +8.
TEST_CASE("AprilTagCorrector: heading latency compensation is correct across the ±180 seam") {
    Rig rig;
    const Pose2d capturePose{Length{20.0}, Length{15.0}, Angle::degrees(176.0)};
    const Pose2d tag = tagAhead(capturePose, 30.0);
    rig.mapTag(1, tag);
    AprilTagCorrector corrector = rig.make();

    Pose2d predicted = capturePose;
    for (int k = 0; k < 9; ++k) {  // 176 -> 184, i.e. across the seam to -176
        rig.imu.setHeading(Angle::degrees(176.0 + static_cast<double>(k)));
        predicted = Pose2d{Length{20.0}, Length{15.0},
                           Angle::degrees(176.0 + static_cast<double>(k))};
        (void)corrector.propose(predicted, Time{0.01});
        if (k < 8) {
            rig.clock.advance(Time{0.01});
        }
    }
    REQUIRE(predicted.heading().degrees() == doctest::Approx(-176.0));

    rig.source.setTags({TagObservation{1, tagAsSeenFrom(capturePose, tag), 0.9}});
    corrector.poll();
    const CorrectionProposal p = corrector.propose(predicted, Time{0.01});
    REQUIRE(p.valid);
    CHECK(std::abs(p.fieldPose.heading().errorTo(Angle::degrees(-176.0))) < 1e-9);
}

// Would catch: a stationary robot having its fix "compensated" anyway — the compensation must be
// exactly zero when nothing moved, or it becomes a source of error rather than a fix for one.
TEST_CASE("AprilTagCorrector: a stationary robot's fix is not moved by latency compensation") {
    Rig rig;
    const Pose2d tag = tagAhead(Rig::truth(), 30.0);
    rig.mapTag(1, tag);
    AprilTagCorrector corrector = rig.make();
    for (int k = 0; k < 20; ++k) {
        (void)corrector.propose(Rig::truth(), Time{0.01});
        rig.clock.advance(Time{0.01});
    }
    rig.source.setTags({TagObservation{1, tagAsSeenFrom(Rig::truth(), tag), 0.9}});
    corrector.poll();
    const CorrectionProposal p = corrector.propose(Rig::truth(), Time{0.01});
    REQUIRE(p.valid);
    CHECK(p.fieldPose.x().value() == doctest::Approx(20.0).epsilon(1e-12));
    CHECK(p.fieldPose.y().value() == doctest::Approx(15.0).epsilon(1e-12));
    CHECK(p.fieldPose.heading().degrees() == doctest::Approx(40.0).epsilon(1e-12));
}

// Would catch: more tags in one frame than the fixed snapshot holds writing past its end, or
// silently dropping the good one. Overflow must be bounded and the corrector must still work.
TEST_CASE("AprilTagCorrector: a frame with more tags than the snapshot holds stays bounded") {
    Rig rig;
    const Pose2d tag = tagAhead(Rig::truth(), 30.0);
    rig.mapTag(1, tag);
    AprilTagCorrector corrector = rig.make();
    std::vector<TagObservation> many;
    for (int k = 0; k < 40; ++k) {
        many.push_back(TagObservation{1, tagAsSeenFrom(Rig::truth(), tag), 0.9});
    }
    corrector.poll();  // the empty frame first, so the count is unambiguous
    rig.source.setTags(many);
    corrector.poll();
    CHECK_NOTHROW((void)corrector.propose(Rig::truth(), Time{0.01}));
    CHECK(corrector.acceptedFixes() == 1);
}
