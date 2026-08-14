// The chunk's keystone file (E3, tension T1): absolute YAW correction through the full stack —
// AprilTagCorrector -> ComplementaryFusion -> Localizer -> published pose.
//
// Four properties are load-bearing here and each has a named failure mode:
//
//  1. IT ACCUMULATES. The Localizer re-reads the IMU every tick, so a heading correction that
//     only decorated THIS tick's published heading would be discarded on the next one —
//     individually sane nudges, collectively useless. That is verbatim the M2 red team's
//     "corrections not accumulating" failure, and it is the reason the bias is persistent state.
//  2. IT NUDGES, NEVER SNAPS. No tick may move the published heading further from the raw IMU
//     reading than `maxHeadingNudgeRate · dt`. A yaw reset is worse than a position snap, because
//     every field-relative command issued afterwards inherits it.
//  3. IT SURVIVES THE IMU RE-STAMP ORDERING. The IMU is still read fresh and composed last; a
//     correction that landed BEFORE the re-stamp and was then overwritten would look like a
//     working feature in the corrector's own tests and do nothing at all in the robot.
//  4. IT COSTS NOTHING WHEN ABSENT. With no heading-providing corrector the published heading
//     must be the raw IMU reading, BIT-IDENTICALLY.

#include "doctest.h"

#include <array>
#include <cmath>
#include <span>
#include <string_view>

#include "shulib/diag/debug_record.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_gps.hpp"
#include "shulib/hal/fake/fake_imu.hpp"
#include "shulib/hal/fake/fake_rotation.hpp"
#include "shulib/hal/fake/fake_tag_source.hpp"
#include "shulib/localization/apriltag_corrector.hpp"
#include "shulib/localization/complementary_fusion.hpp"
#include "shulib/localization/fake/fake_corrector.hpp"
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
using shulib::hal::TagObservation;
using shulib::hal::fake::FakeClock;
using shulib::hal::fake::FakeGps;
using shulib::hal::fake::FakeImu;
using shulib::hal::fake::FakeRotation;
using shulib::hal::fake::FakeTagSource;
using shulib::localization::AprilTagCorrector;
using shulib::localization::AprilTagCorrectorConfig;
using shulib::localization::ComplementaryFusion;
using shulib::localization::ComplementaryFusionConfig;
using shulib::localization::GpsCorrector;
using shulib::localization::ICorrector;
using shulib::localization::Localizer;
using shulib::localization::PilonsOdometry;
using shulib::localization::TagMap;
using shulib::localization::TagPlacement;
using shulib::localization::TagProvenance;
using shulib::localization::TrackingWheel;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::units::Length;
using shulib::units::Time;

namespace {

constexpr double kDeg = Angle::kPi / 180.0;

/// Where a tag at field pose `tag` appears in the body frame of a robot at field pose `robot`.
/// (Pinned independently in test/tag_map_test.cpp before this file leans on it.)
[[nodiscard]] Pose2d tagAsSeenFrom(const Pose2d& robot, const Pose2d& tag) {
    const double dx = tag.x().value() - robot.x().value();
    const double dy = tag.y().value() - robot.y().value();
    const double c = std::cos(robot.heading().radians());
    const double s = std::sin(robot.heading().radians());
    return Pose2d{Length{dx * c + dy * s}, Length{-dx * s + dy * c},
                  Angle::radians(tag.heading().radians() - robot.heading().radians())};
}

/// The full stack over fakes. The robot sits still at a known TRUTH pose while the IMU reports a
/// heading that is wrong by a chosen amount — which is exactly the situation absolute yaw
/// correction exists for, with everything else held constant so nothing else can explain a change.
struct Stack {
    FakeClock clk{Time{5.0}};
    FakeImu imu;
    FakeRotation fwdRot, latRot;
    FakeTagSource source;
    TagMap map;
    PilonsOdometry odom;
    ComplementaryFusion fusion;
    AprilTagCorrector tagCorrector;
    std::array<ICorrector*, 1> correctors{};
    Localizer loc;

    /// Truth: neither the origin nor heading 0.
    static Pose2d truth() { return Pose2d{Length{24.0}, Length{-16.0}, Angle::degrees(35.0)}; }
    /// A tag 30 inches ahead of the truth pose, facing back at it.
    static Pose2d tagPose() {
        const Pose2d t = truth();
        return Pose2d{Length{t.x().value() + 30.0 * std::cos(t.heading().radians())},
                      Length{t.y().value() + 30.0 * std::sin(t.heading().radians())},
                      Angle::radians(t.heading().radians() + Angle::kPi)};
    }

    explicit Stack(const AprilTagCorrectorConfig& tcfg = {},
                   const ComplementaryFusionConfig& fcfg = {})
        : odom{imu, TrackingWheel::forward(fwdRot, Length{2.0}, Length{0.0}),
               TrackingWheel::lateral(latRot, Length{2.0}, Length{0.0})},
          fusion{fcfg},
          tagCorrector{clk, source, imu, map, tcfg},
          correctors{&tagCorrector},
          loc{clk, imu, odom, fusion,
              std::span<ICorrector* const>{correctors.data(), correctors.size()}, {}} {
        map.add(TagPlacement{7, tagPose(), TagProvenance::Invented,
                             "host test fixture — not a field layout"});
        loc.setPose(truth());
    }

    /// The IMU is wrong by `errDeg`; a tag consistent with the TRUTH pose is in view.
    void setImuError(double errDeg) {
        imu.setHeading(Angle::degrees(truth().heading().degrees() - errDeg));
    }
    void showTag() {
        source.setTags({TagObservation{7, tagAsSeenFrom(truth(), tagPose()), 0.9}});
    }
    /// One control tick. `pollNow` mimics the vision task running at its own, lower cadence.
    void tick(bool pollNow) {
        clk.advance(Time{0.01});
        if (pollNow) {
            tagCorrector.poll();
        }
        loc.update();
    }
    [[nodiscard]] double publishedHeadingDeg() const { return loc.pose().heading().degrees(); }
    [[nodiscard]] double rawImuHeadingDeg() const { return imu.heading().degrees(); }
    [[nodiscard]] double biasDeg() const { return loc.headingBias().value() / kDeg; }
};

}  // namespace

// ── 1. IT ACCUMULATES ────────────────────────────────────────────────────────────────────────

// Would catch: THE failure mode the M2 red team named — corrections that are individually sane
// and collectively useless. If the heading nudge were applied to the published pose rather than
// to persistent state, every tick would start from the raw IMU again and the published heading
// would oscillate one nudge away from the IMU forever, never converging. Here the error must
// SHRINK monotonically across sightings and end essentially closed.
TEST_CASE("heading: repeated tag sightings ACCUMULATE and converge on the truth") {
    Stack s;
    s.setImuError(4.0);  // the IMU thinks the robot is 4 degrees clockwise of where it is
    s.showTag();
    REQUIRE(s.biasDeg() == doctest::Approx(0.0));

    double previousError = 4.0;
    bool everWorse = false;
    int ticks = 0;
    double errorAt1s = -1.0;
    double errorAt3s = -1.0;
    for (int k = 0; k < 1500; ++k) {  // 15 seconds at 100 Hz
        const bool pollNow = (k % 5 == 0);  // ~20 Hz vision against a 100 Hz loop
        s.tick(pollNow);
        const double err =
            std::abs(Angle::degrees(s.publishedHeadingDeg()).errorTo(Stack::truth().heading())) /
            kDeg;
        if (err > previousError + 1e-9) {
            everWorse = true;
        }
        previousError = err;
        if (k == 99) {
            errorAt1s = err;
        }
        if (k == 299) {
            errorAt3s = err;
        }
        ++ticks;
    }
    CHECK(ticks == 1500);
    CHECK_FALSE(everWorse);           // monotone: no tick ever moved it away from truth
    CHECK(previousError < 0.02);      // and it actually got there
    CHECK(s.biasDeg() == doctest::Approx(4.0).epsilon(0.01));  // the learned IMU bias
    CHECK(s.biasDeg() <= 4.0 + 1e-9);  // approached from below; never overshot the measurement
    // The settling numbers, recorded rather than asserted tightly — this is the shape of the
    // correction, and R4 is what decides whether the constants behind it are right.
    MESSAGE("heading convergence from 4.0 deg: " << errorAt1s << " deg at 1 s, " << errorAt3s
                                                 << " deg at 3 s, " << previousError
                                                 << " deg at 15 s");
    CHECK(errorAt1s < 3.5);   // it is already moving in the first second
    CHECK(errorAt3s < 1.0);

    // The IMU itself was never touched: the published heading is the raw reading PLUS the bias.
    CHECK(s.rawImuHeadingDeg() == doctest::Approx(31.0).epsilon(1e-12));
    CHECK(s.publishedHeadingDeg() ==
          doctest::Approx(s.rawImuHeadingDeg() + s.biasDeg()).epsilon(1e-9));
}

// Would catch: the bias being recomputed from scratch each tick (or reset when the tags go away),
// which would make the correction evaporate the moment the camera loses sight of a tag — i.e. it
// would work in a demo and fail in a match. Once learned, the bias must PERSIST through a long
// blind stretch.
TEST_CASE("heading: the learned bias persists after the tags disappear") {
    Stack s;
    s.setImuError(3.0);
    s.showTag();
    for (int k = 0; k < 600; ++k) {
        s.tick(k % 5 == 0);
    }
    const double learned = s.biasDeg();
    REQUIRE(learned > 2.5);

    s.source.clear();  // the camera loses the tag entirely
    for (int k = 0; k < 500; ++k) {
        s.tick(k % 5 == 0);
    }
    s.tick(true);  // land on a POLL tick, so the verdict under test is this tick's own
    CHECK(s.biasDeg() == doctest::Approx(learned).epsilon(1e-12));  // unchanged, to the bit
    CHECK(s.loc.lastCorrection().audit.reason == GateReason::RejectedNoFix);
    CHECK(s.loc.lastCorrection().dtheta.value() == 0.0);
}

// ── 2. IT NUDGES, NEVER SNAPS ────────────────────────────────────────────────────────────────

// Would catch: a tag sighting SNAPPING the heading — the single worst thing this chunk could do,
// because every field-relative command issued afterwards inherits the jump. Asserted on EVERY
// tick, against a deliberately enormous 12-degree error and a maximally confident tag.
TEST_CASE("heading: no tick ever moves the heading more than the per-tick budget — never snap") {
    ComplementaryFusionConfig fcfg;  // defaults: 10 deg/s budget
    Stack s{{}, fcfg};
    s.setImuError(12.0);
    s.showTag();

    const double budgetDeg = fcfg.maxHeadingNudgeRate.value() * 0.01 / kDeg;  // per 10 ms tick
    REQUIRE(budgetDeg == doctest::Approx(0.1));
    s.tick(false);  // one settling tick: the IMU reading itself reaches the published pose here,
                    // and that is the IMU moving, not a correction
    double previousBias = s.biasDeg();
    double previousHeading = s.publishedHeadingDeg();
    int ticks = 0;
    for (int k = 0; k < 2000; ++k) {
        s.tick(k % 5 == 0);
        // The invariant, stated exactly: the CORRECTION is the bias, and it may move by at most
        // one tick's budget. (Real IMU rotation is not bounded by this and must not be.)
        CHECK(std::abs(s.biasDeg() - previousBias) <= budgetDeg + 1e-9);
        // The audit record says the same thing independently — this is the value telemetry sees.
        CHECK(std::abs(s.loc.lastCorrection().dtheta.value()) / kDeg <= budgetDeg + 1e-9);
        // And with the robot stationary the published heading moves only by the correction, so
        // the never-snap bound is visible on the published pose itself.
        const double now = s.publishedHeadingDeg();
        CHECK(std::abs(Angle::degrees(previousHeading).errorTo(Angle::degrees(now))) / kDeg <=
              budgetDeg + 1e-9);
        // NO OVERSHOOT, on EVERY tick — not just at the end. A nudge that is always a signed
        // fraction of the innovation toward an ABSOLUTE measurement is structurally incapable of
        // passing it, so a bias above 12 degrees at ANY point would mean the correction is being
        // fed back into itself somewhere. Asserted per tick because an overshoot during the
        // clamped approach would be transient and invisible to an end-of-run check.
        CHECK(s.biasDeg() <= 12.0 + 1e-9);
        previousBias = s.biasDeg();
        previousHeading = now;
        ++ticks;
    }
    CHECK(ticks == 2000);
    CHECK(std::abs(s.biasDeg() - 12.0) < 0.1);  // it did get all the way there, one nudge at a time
    CHECK(s.biasDeg() <= 12.0 + 1e-9);          // and never overshot the measurement
}

// Would catch: the heading gate being absent or inverted. A mirrored tag winding (which
// hal/vision_conversion.hpp proves is SILENT and flips the heading 180 degrees), a wrong map
// entry or a misidentified id all produce a huge heading innovation, and folding one would be
// far worse than folding nothing. Position must still be corrected: the two are gated
// independently because they are different measurements.
TEST_CASE("heading: an outrageous heading innovation is gated out, position still folds") {
    Stack s;
    s.setImuError(0.0);
    // The observation a robot at the RIGHT POSITION but facing 90 degrees away would produce, so
    // the position residual is ZERO and only the heading is absurd. Perturbing the relative
    // heading alone would NOT do this: the inversion rotates the position term by the robot's
    // heading, so the derived position moves too and the POSITION gate fires first — which would
    // make this test pass for the wrong reason.
    const Pose2d skewed{Stack::truth().x(), Stack::truth().y(),
                        Angle::degrees(Stack::truth().heading().degrees() + 90.0)};
    s.source.setTags({TagObservation{7, tagAsSeenFrom(skewed, Stack::tagPose()), 0.9}});

    for (int k = 0; k < 60; ++k) {
        s.tick(k % 5 == 0);
    }
    s.tick(true);  // land on a POLL tick so the audit under test is this tick's own verdict
    CHECK(s.biasDeg() == doctest::Approx(0.0).epsilon(1e-12));  // heading never moved
    CHECK(s.loc.lastCorrection().dtheta.value() == 0.0);
    // POSITION was accepted on the same tick — the two are gated independently.
    CHECK(s.loc.lastCorrection().audit.reason == GateReason::Accepted);
    // The gate's own account is on the record: a large heading residual beside a zero applied
    // heading nudge is exactly how a heading rejection reads.
    CHECK(std::abs(s.loc.lastCorrection().audit.residualHeading.value()) / kDeg > 45.0);
}

// Would catch: a heading correction firing while the estimator is not live yet. During the IMU
// boot/settle window the heading stream is garbage that MOVES, and a bias learned from it would
// be permanent — nothing later can tell it was learned from noise.
TEST_CASE("heading: no bias is learned while the IMU is still booting") {
    Stack s;
    s.setImuError(5.0);
    s.showTag();
    s.imu.setReady(false);
    for (int k = 0; k < 50; ++k) {
        s.tick(k % 5 == 0);
    }
    CHECK(s.biasDeg() == 0.0);
    CHECK(s.loc.qualityClass() == Localizer::Quality::Uninitialized);
}

// ── 3. IT SURVIVES THE IMU RE-STAMP ORDERING ────────────────────────────────────────────────

// Would catch: the bias being applied BEFORE the IMU re-stamp and then overwritten by it — a bug
// that passes every corrector-level test and does nothing whatsoever on the robot. The probe: the
// robot ROTATES while the bias is being learned. The IMU's rotation must arrive in full (so the
// IMU still owns heading CHANGE) with the bias riding on top of it.
TEST_CASE("heading: the IMU still owns every rotation; the bias rides on top of it") {
    Stack s;
    s.setImuError(3.0);
    s.showTag();
    for (int k = 0; k < 600; ++k) {  // learn the bias while stationary
        s.tick(k % 5 == 0);
    }
    const double learned = s.biasDeg();
    REQUIRE(learned > 2.5);
    const double before = s.publishedHeadingDeg();

    // Now the robot turns 20 degrees. Nothing else changes; the tags go away so no further
    // correction can be confused with the rotation.
    s.source.clear();
    s.imu.setHeading(Angle::degrees(s.rawImuHeadingDeg() + 20.0));
    s.tick(false);

    CHECK(s.publishedHeadingDeg() == doctest::Approx(before + 20.0).epsilon(1e-9));
    CHECK(s.biasDeg() == doctest::Approx(learned).epsilon(1e-12));
    // and the published heading is still exactly raw + bias, i.e. the composition is the last act
    CHECK(s.publishedHeadingDeg() ==
          doctest::Approx(s.rawImuHeadingDeg() + s.biasDeg()).epsilon(1e-9));
}

// Would catch: setPose() throwing the learned bias away. A routine that re-seeds its position
// mid-run would silently discard a correction that took a second of tag sightings to acquire —
// and a teleport says where the robot IS, not which way the IMU is wrong. It must also publish
// the CORRECTED heading, or setPose and update() would disagree about the same instant.
TEST_CASE("heading: setPose keeps the learned bias and publishes the corrected heading") {
    Stack s;
    s.setImuError(3.5);
    s.showTag();
    for (int k = 0; k < 600; ++k) {
        s.tick(k % 5 == 0);
    }
    const double learned = s.biasDeg();
    REQUIRE(learned > 3.0);

    s.loc.setPose(Pose2d{Length{60.0}, Length{-40.0}, Angle::degrees(0.0)});
    CHECK(s.loc.headingBias().value() == doctest::Approx(learned * kDeg).epsilon(1e-12));
    CHECK(s.loc.pose().x().value() == doctest::Approx(60.0));
    CHECK(s.loc.pose().heading().degrees() ==
          doctest::Approx(s.rawImuHeadingDeg() + learned).epsilon(1e-9));
}

// ── 4. IT COSTS NOTHING WHEN ABSENT ─────────────────────────────────────────────────────────

// Would catch: E3 changing the behaviour of every tree that has no tag corrector. The heading
// path must be inert unless something actually supplies an absolute heading — and inert means
// BIT-IDENTICAL, not "close enough", because a floating-point drift here would silently
// invalidate every accuracy number measured before this chunk.
TEST_CASE("heading: with no heading-providing corrector the published heading is the raw IMU, bitwise") {
    FakeClock clk{Time{5.0}};
    FakeImu imu;
    FakeRotation fwdRot, latRot;
    PilonsOdometry odom{imu, TrackingWheel::forward(fwdRot, Length{2.0}, Length{0.0}),
                        TrackingWheel::lateral(latRot, Length{2.0}, Length{0.0})};
    ComplementaryFusion fusion;
    Localizer loc{clk, imu, odom, fusion};

    const double headings[] = {0.0, 17.5, -93.25, 179.9, -179.9, 44.0};
    for (const double h : headings) {
        for (int k = 0; k < 3; ++k) {
            clk.advance(Time{0.01});
            imu.setHeading(Angle::degrees(h));
            fwdRot.setPosition(shulib::units::AngleDim{static_cast<double>(k)});
            loc.update();
            // == on doubles, deliberately: this is a bit-identity claim, not an approximation.
            CHECK(loc.pose().heading().radians() == imu.heading().radians());
        }
    }
    CHECK(loc.headingBias().value() == 0.0);
    CHECK(loc.lastCorrection().dtheta.value() == 0.0);
}

// ── TWO CORRECTORS — the substitution guard E2 closed becomes load-bearing ───────────────────

namespace {

/// GPS + tags over one plant. The first time in the project's history that two real correctors
/// have run together, which is what makes E2's substitution guard live code.
struct TwoSourceStack {
    FakeClock clk{Time{5.0}};
    FakeImu imu;
    FakeGps gps;
    FakeRotation fwdRot, latRot;
    FakeTagSource source;
    TagMap map;
    PilonsOdometry odom;
    ComplementaryFusion fusion;
    GpsCorrector gpsCorrector;
    AprilTagCorrector tagCorrector;
    std::array<ICorrector*, 2> correctors{};
    Localizer loc;

    static Pose2d truth() { return Pose2d{Length{24.0}, Length{-16.0}, Angle::degrees(35.0)}; }
    static Pose2d tagPose() {
        const Pose2d t = truth();
        return Pose2d{Length{t.x().value() + 30.0 * std::cos(t.heading().radians())},
                      Length{t.y().value() + 30.0 * std::sin(t.heading().radians())},
                      Angle::radians(t.heading().radians() + Angle::kPi)};
    }

    TwoSourceStack()
        : odom{imu, TrackingWheel::forward(fwdRot, Length{2.0}, Length{0.0}),
               TrackingWheel::lateral(latRot, Length{2.0}, Length{0.0})},
          gpsCorrector{clk, gps, imu},
          tagCorrector{clk, source, imu, map},
          correctors{&gpsCorrector, &tagCorrector},
          loc{clk, imu, odom, fusion,
              std::span<ICorrector* const>{correctors.data(), correctors.size()}, {}} {
        map.add(TagPlacement{7, tagPose(), TagProvenance::Invented, "host test fixture"});
        imu.setHeading(truth().heading());
        loc.setPose(truth());
    }

    void tick(bool pollNow) {
        clk.advance(Time{0.01});
        if (pollNow) {
            tagCorrector.poll();
        }
        loc.update();
    }
};

}  // namespace

// Would catch: two agreeing sources fighting each other, or the per-tick budget being exceeded
// because two proposals each got their own full allowance. The sum is clamped once more for
// exactly this reason.
TEST_CASE("two correctors: GPS and tags AGREEING pull together, within one tick's budget") {
    TwoSourceStack s;
    s.gps.setHasFix(true);
    s.gps.setRmsError(Length{0.5});
    s.source.setTags(
        {TagObservation{7, tagAsSeenFrom(TwoSourceStack::truth(), TwoSourceStack::tagPose()), 0.9}});
    // The estimate starts 4 inches off; BOTH sources say the same, correct thing.
    s.loc.setPose(Pose2d{Length{28.0}, Length{-16.0}, TwoSourceStack::truth().heading()});

    const ComplementaryFusionConfig fcfg;
    const double budget = fcfg.maxNudgeRate.value() * 0.01;
    Pose2d previous = s.loc.pose();
    for (int k = 0; k < 300; ++k) {
        s.gps.setPose(Pose2d{Length{24.0 + 0.001 * static_cast<double>(k)}, Length{-16.0},
                             TwoSourceStack::truth().heading()});  // jitters, so it is never stale
        s.tick(k % 5 == 0);
        const Pose2d now = s.loc.pose();
        CHECK(std::hypot(now.x().value() - previous.x().value(),
                         now.y().value() - previous.y().value()) <= budget + 1e-9);
        previous = now;
    }
    CHECK(std::abs(s.loc.pose().x().value() - 24.0) < 0.5);  // they converged, together
}

// Would catch: one loud source overriding a disagreeing one without bound — the case an EKF
// resolves with covariance and a complementary tier can only BOUND. What must hold is that the
// estimate lands somewhere between the two claims and never snaps to either.
TEST_CASE("two correctors: GPS and tags DISAGREEING never snap to either, and stay bounded") {
    TwoSourceStack s;
    s.gps.setHasFix(true);
    s.gps.setRmsError(Length{0.5});
    // The tag is truthful; the GPS is confidently 6 inches wrong in +Y.
    s.source.setTags(
        {TagObservation{7, tagAsSeenFrom(TwoSourceStack::truth(), TwoSourceStack::tagPose()), 0.9}});

    const ComplementaryFusionConfig fcfg;
    const double budget = fcfg.maxNudgeRate.value() * 0.01;
    Pose2d previous = s.loc.pose();
    for (int k = 0; k < 400; ++k) {
        s.gps.setPose(Pose2d{Length{24.0}, Length{-10.0 + 0.001 * static_cast<double>(k)},
                             TwoSourceStack::truth().heading()});
        s.tick(k % 5 == 0);
        const Pose2d now = s.loc.pose();
        CHECK(std::hypot(now.x().value() - previous.x().value(),
                         now.y().value() - previous.y().value()) <= budget + 1e-9);
        previous = now;
    }
    const double y = s.loc.pose().y().value();
    CHECK(y > -16.5);  // never snapped all the way to the tag's claim
    CHECK(y < -9.5);   // nor to the GPS's
}

// Would catch: THE HOLE E2 FOUND, from the other side. With two correctors, a silent source's
// verdict must not overwrite a real fusion verdict — and, new at E3, one source's ROUTINE
// staleness must not mask the other source's GENUINE failure. E2 measured that a healthy source
// spends most of its ticks stale, so under a plain first-wins rule the tag corrector's
// misconfiguration would be invisible for the entire run.
TEST_CASE("two correctors: an exceptional verdict outranks the other source's routine staleness") {
    TwoSourceStack s;
    s.gps.setHasFix(true);
    s.gps.setRmsError(Length{0.5});
    s.gps.setPose(TwoSourceStack::truth());
    // The tag corrector is misconfigured: the visible tag's id is not in the map.
    s.source.setTags({TagObservation{
        99, tagAsSeenFrom(TwoSourceStack::truth(), TwoSourceStack::tagPose()), 0.9}});

    s.tick(true);   // GPS folds its one fix
    s.tick(true);   // GPS is now STALE (unchanged sample); the tag id is unmapped

    CHECK(s.loc.lastCorrection().audit.reason == GateReason::RejectedNoTagMapEntry);
    CHECK(std::string_view{s.loc.lastCorrection().source} == "tags");
}

// Would catch: a silent source's verdict overwriting a REAL fusion verdict — the exact mutation
// that stayed green at E2 and could only be seen once a second corrector existed.
TEST_CASE("two correctors: a silent source never overwrites an APPLIED fix's verdict") {
    TwoSourceStack s;
    s.gps.setHasFix(true);
    s.gps.setRmsError(Length{0.5});
    s.gps.setPose(Pose2d{Length{26.0}, Length{-16.0}, TwoSourceStack::truth().heading()});
    s.source.clear();  // the tag corrector is silent all run

    s.tick(true);  // dt is 0 on the very first tick, so nothing can be APPLIED yet
    s.gps.setPose(Pose2d{Length{26.5}, Length{-16.0}, TwoSourceStack::truth().heading()});
    s.tick(true);
    CHECK(s.loc.lastCorrection().audit.reason == GateReason::Accepted);
    CHECK(std::string_view{s.loc.lastCorrection().source} == "gps");
    // The tag corrector DID decline with a reason this tick (fresh frame, no tag) — the point is
    // that its verdict must not be stamped over a tick where a fix was actually applied.
    CHECK(s.tagCorrector.lastVerdict() == GateReason::RejectedNoFix);
}

// Would catch: the tag corrector being unable to correct heading when a GPS is also registered —
// e.g. a policy that stopped at the first proposal, or one that let the GPS's pass-through
// heading (providesHeading == false, carrying the PREDICTION) be treated as a measurement and
// cancel the tag's. The GPS's heading must contribute exactly nothing.
TEST_CASE("two correctors: the GPS's pass-through heading never dilutes the tag's yaw fix") {
    TwoSourceStack s;
    s.gps.setHasFix(true);
    s.gps.setRmsError(Length{0.5});
    s.source.setTags(
        {TagObservation{7, tagAsSeenFrom(TwoSourceStack::truth(), TwoSourceStack::tagPose()), 0.9}});
    // The IMU is 3 degrees wrong; only the tag knows.
    s.imu.setHeading(Angle::degrees(TwoSourceStack::truth().heading().degrees() - 3.0));

    for (int k = 0; k < 600; ++k) {
        s.gps.setPose(Pose2d{Length{24.0 + 0.001 * static_cast<double>(k)}, Length{-16.0},
                             Angle::degrees(-150.0)});  // a GPS confidently wrong about heading
        s.tick(k % 5 == 0);
    }
    CHECK(s.loc.headingBias().value() / kDeg == doctest::Approx(3.0).epsilon(0.05));
    CHECK(std::abs(s.loc.pose().heading().errorTo(TwoSourceStack::truth().heading())) / kDeg <
          0.1);
}

// ── TWO HOLES FOUND BY THE E3 MUTATION CAMPAIGN, CLOSED HERE ────────────────────────────────

// Would catch: `CorrectionProposal::providesHeading` not being load-bearing — a corrector whose
// heading is a PASS-THROUGH of the prediction having that heading folded as if it were a
// measurement. Found by mutation: deleting the `providesHeading` check in ComplementaryFusion
// stayed GREEN, because the only pre-existing test of the property used a corrector claiming a
// 90-degree-wrong heading, which the HEADING GATE rejects anyway. The property was being proved
// by the wrong mechanism.
//
// So this case keeps the lie INSIDE the gate (5 degrees), where only `providesHeading` can stop
// it, and then shows the same corrector moving the bias once it does claim to measure heading.
TEST_CASE("heading: providesHeading is load-bearing — a small lie inside the gate is still ignored") {
    FakeClock clk{Time{5.0}};
    FakeImu imu;
    FakeRotation fwdRot, latRot;
    PilonsOdometry odom{imu, TrackingWheel::forward(fwdRot, Length{2.0}, Length{0.0}),
                        TrackingWheel::lateral(latRot, Length{2.0}, Length{0.0})};
    ComplementaryFusion fusion;
    shulib::localization::fake::FakeCorrector liar{"liar"};
    std::array<ICorrector*, 1> arr{&liar};
    Localizer loc{clk, imu, odom, fusion, std::span<ICorrector* const>{arr}};
    imu.setHeading(Angle::degrees(20.0));

    shulib::localization::CorrectionProposal p;
    p.valid = true;
    p.confidence = 1.0;
    p.positionStdDev = Length{0.5};
    p.providesHeading = false;  // "my heading field carries no information"
    p.fieldPose = Pose2d{Length{0.0}, Length{0.0}, Angle::degrees(25.0)};  // ...but it is 5 deg off
    liar.setProposal(p);

    for (int k = 0; k < 200; ++k) {
        clk.advance(Time{0.01});
        loc.update();
    }
    CHECK(loc.headingBias().value() == 0.0);  // exactly zero: not "small", ZERO
    CHECK(loc.pose().heading().degrees() == doctest::Approx(20.0).epsilon(1e-12));

    // The same 5-degree claim, now MARKED as a measurement, does move the bias — which proves
    // the flag is what decided, and not the gate or the magnitude.
    p.providesHeading = true;
    liar.setProposal(p);
    for (int k = 0; k < 200; ++k) {
        clk.advance(Time{0.01});
        loc.update();
    }
    CHECK(loc.headingBias().value() > 0.0);
    CHECK(loc.pose().heading().degrees() > 20.5);
}

// Would catch: the odometry delta still being rotated by the RAW IMU heading after a bias has
// been learned — so the reported heading is corrected while the dead-reckoned POSITION keeps
// accumulating a cross-track error of roughly `bias x distance`, which is most of what heading
// drift actually costs over a run. Found by mutation: disabling the rotation stayed GREEN,
// because every heading test until now kept the robot STATIONARY.
//
// THE PROBE. Learn a 4-degree bias while stationary, take the tags away, then drive 20 inches
// forward on the wheels. The odometry, working from the raw IMU heading of 31 degrees, hands up
// a delta of 20 inches at 31 degrees. The estimator believes the robot is pointing at 35, so the
// fused position must advance 20 inches at 35 degrees:
//       corrected  dy = 20*sin(35) = 11.4715
//       uncorrected dy = 20*sin(31) = 10.3006     (a 1.17-inch cross-track error, from ONE move)
TEST_CASE("heading: a learned bias also re-expresses the odometry delta (position, not just yaw)") {
    Stack s;
    s.setImuError(4.0);  // IMU says 31, truth is 35
    s.showTag();
    for (int k = 0; k < 1500; ++k) {
        s.tick(k % 5 == 0);
    }
    REQUIRE(s.biasDeg() == doctest::Approx(4.0).epsilon(0.01));
    s.source.clear();  // no more corrections: what follows is pure dead-reckoning
    s.tick(true);
    const Pose2d before = s.loc.pose();

    // 20 inches of forward travel on diameter-2 wheels (shaft radians == inches).
    s.fwdRot.setPosition(shulib::units::AngleDim{20.0});
    s.tick(false);
    const Pose2d after = s.loc.pose();

    const double dx = after.x().value() - before.x().value();
    const double dy = after.y().value() - before.y().value();
    CHECK(std::hypot(dx, dy) == doctest::Approx(20.0).epsilon(1e-6));  // the distance is the same
    CHECK(dy == doctest::Approx(20.0 * std::sin(35.0 * kDeg)).epsilon(1e-4));   // ...the DIRECTION
    CHECK(dx == doctest::Approx(20.0 * std::cos(35.0 * kDeg)).epsilon(1e-4));   //    is corrected
    // and, stated as the thing that would be wrong: it is NOT the raw IMU's 31 degrees.
    CHECK(std::abs(dy - 20.0 * std::sin(31.0 * kDeg)) > 1.0);
}
