// AprilTagCorrector through the Localizer and out to the blackbox (chunk E3).
//
// The DoD clause under test: "every tag gating decision is reconstructable from the file alone."
// E2 established that standard for the GPS; this file holds the tag path to it, and adds the two
// things that are new at E3 —
//
//   * a HEADING nudge, which arrives on `correctionDTheta`: the §18.2 slot declared at A1 as
//     "heading nudge (0 at M2: heading is IMU-owned) — E3", serialized since F9 at byte 340, and
//     zero on every record ever written before this chunk;
//   * a HEADING RESIDUAL on `gateResidualHeading`, declared at A1 as "E3 fills it".
//
// The rig is built from fakes on purpose: this file is about what reaches the file, so every
// input is scripted and every expected value is chosen by the test rather than emerging from
// physics.

#include "doctest.h"

#include <array>
#include <cmath>
#include <span>
#include <string_view>
#include <vector>

#include "shulib/diag/blackbox_format.hpp"
#include "shulib/diag/blackbox_reader.hpp"
#include "shulib/diag/debug_record.hpp"
#include "shulib/diag/sd_sink.hpp"
#include "shulib/hal/fake/fake_block_sink.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_imu.hpp"
#include "shulib/hal/fake/fake_rotation.hpp"
#include "shulib/hal/fake/fake_tag_source.hpp"
#include "shulib/localization/apriltag_corrector.hpp"
#include "shulib/localization/complementary_fusion.hpp"
#include "shulib/localization/localizer.hpp"
#include "shulib/localization/pilons_odometry.hpp"
#include "shulib/localization/tag_map.hpp"
#include "shulib/localization/tracking_wheel.hpp"
#include "shulib/motion/motion_scheduler.hpp"

namespace bb = shulib::diag::blackbox;

using shulib::diag::DebugRecord;
using shulib::diag::GateReason;
using shulib::diag::SdSink;
using shulib::diag::SdSinkConfig;
using shulib::diag::SdSinkStorage;
using shulib::hal::TagObservation;
using shulib::hal::fake::FakeBlockSink;
using shulib::hal::fake::FakeClock;
using shulib::hal::fake::FakeImu;
using shulib::hal::fake::FakeRotation;
using shulib::hal::fake::FakeTagSource;
using shulib::localization::AprilTagCorrector;
using shulib::localization::AprilTagCorrectorConfig;
using shulib::localization::ComplementaryFusion;
using shulib::localization::ComplementaryFusionConfig;
using shulib::localization::ICorrector;
using shulib::localization::Localizer;
using shulib::localization::PilonsOdometry;
using shulib::localization::TagMap;
using shulib::localization::TagPlacement;
using shulib::localization::TagProvenance;
using shulib::localization::TrackingWheel;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::motion::CommandIdStampSink;
using shulib::units::AngularVelocity;
using shulib::units::Length;
using shulib::units::Time;

namespace {

constexpr double kDeg = Angle::kPi / 180.0;

[[nodiscard]] Pose2d tagAsSeenFrom(const Pose2d& robot, const Pose2d& tag) {
    const double dx = tag.x().value() - robot.x().value();
    const double dy = tag.y().value() - robot.y().value();
    const double c = std::cos(robot.heading().radians());
    const double s = std::sin(robot.heading().radians());
    return Pose2d{Length{dx * c + dy * s}, Length{-dx * s + dy * c},
                  Angle::radians(tag.heading().radians() - robot.heading().radians())};
}

struct TagRig {
    FakeClock clk;
    FakeImu imu;
    FakeRotation fwdRot, latRot;
    FakeTagSource source;
    TagMap map;
    PilonsOdometry odom;
    ComplementaryFusion fusion;
    AprilTagCorrector corrector;
    std::array<ICorrector*, 1> correctors;
    Localizer loc;

    /// Truth: neither the origin nor heading 0.
    static Pose2d truth() { return Pose2d{Length{18.0}, Length{-9.0}, Angle::degrees(25.0)}; }
    static Pose2d tagPose() {
        const Pose2d t = truth();
        return Pose2d{Length{t.x().value() + 28.0 * std::cos(t.heading().radians())},
                      Length{t.y().value() + 28.0 * std::sin(t.heading().radians())},
                      Angle::radians(t.heading().radians() + Angle::kPi)};
    }

    explicit TagRig(const AprilTagCorrectorConfig& cfg = {}, bool wireCorrector = true,
                    const ComplementaryFusionConfig& fcfg = {})
        : odom{imu, TrackingWheel::forward(fwdRot, Length{2.0}, Length{0.0}),
               TrackingWheel::lateral(latRot, Length{2.0}, Length{0.0})},
          fusion{fcfg},
          corrector{clk, source, imu, map, cfg},
          correctors{&corrector},
          loc{clk, imu, odom, fusion,
              wireCorrector ? std::span<ICorrector* const>{correctors}
                            : std::span<ICorrector* const>{}} {
        imu.setReady(true);
        imu.setYawRate(AngularVelocity{0.0});
        imu.setHeading(truth().heading());
        map.add(TagPlacement{7, tagPose(), TagProvenance::Invented, "host test fixture"});
        loc.setPose(truth());
    }

    void showTruthfulTag(double confidence = 0.9) {
        source.setTags({TagObservation{7, tagAsSeenFrom(truth(), tagPose()), confidence}});
    }
    void tick(double dt, double fwdTravel, bool pollNow) {
        clk.advance(Time{dt});
        if (pollNow) {
            corrector.poll();
        }
        fwdRot.setPosition(shulib::units::AngleDim{fwdTravel});
        loc.update();
    }
};

/// Every tick record in the device's bytes, in file order.
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

// ── VISIBILITY ───────────────────────────────────────────────────────────────────────────────

// Would catch: a vision subsystem that is not helping being indistinguishable from one that is
// not installed. There are now FIVE different reasons a tag corrector can be silent and they
// need different responses from a team: nobody wired the poller, the poller died, the frame was
// already used, nothing is in view, or the tag's id is not in the map. A single "no fix" for all
// five would make the record useless exactly where it matters most.
TEST_CASE("tags: each kind of silence is a DIFFERENT word on the record") {
    {  // nobody ever polled — a wiring mistake, visible from the first tick
        TagRig r;
        r.showTruthfulTag();
        r.tick(0.01, 0.0, false);
        r.tick(0.01, 0.0, false);
        CHECK(r.loc.lastCorrection().audit.reason == GateReason::RejectedNoFix);
        CHECK(r.corrector.pollCount() == 0);
        CHECK(r.corrector.noFrameTicks() == 2);
    }
    {  // the camera is alive and there is no tag in view — normal driving
        TagRig r;
        r.source.clear();
        r.tick(0.01, 0.0, true);
        r.tick(0.01, 0.0, true);
        CHECK(r.loc.lastCorrection().audit.reason == GateReason::RejectedNoFix);
        CHECK(r.corrector.noTagTicks() == 2);
        CHECK(r.corrector.noFrameTicks() == 0);  // NOT the wiring word
    }
    {  // the poller stopped — a failed subsystem
        TagRig r;
        r.showTruthfulTag();
        r.tick(0.01, 0.0, true);
        r.tick(0.4, 0.0, false);
        CHECK(r.loc.lastCorrection().audit.reason == GateReason::RejectedObservationAge);
    }
    {  // the map does not know this tag — a configuration error the team can fix
        TagRig r;
        r.source.setTags({TagObservation{99, tagAsSeenFrom(TagRig::truth(), TagRig::tagPose()),
                                         0.9}});
        r.tick(0.01, 0.0, true);
        r.tick(0.01, 0.0, true);
        CHECK(r.loc.lastCorrection().audit.reason == GateReason::RejectedNoTagMapEntry);
    }
    {  // every visible tag is outside the trusted range band
        TagRig r;
        const Pose2d t = TagRig::truth();
        const Pose2d farTag{Length{t.x().value() + 200.0 * std::cos(t.heading().radians())},
                            Length{t.y().value() + 200.0 * std::sin(t.heading().radians())},
                            Angle::radians(t.heading().radians() + Angle::kPi)};
        r.map.add(TagPlacement{8, farTag, TagProvenance::Invented, "host test fixture"});
        r.source.setTags({TagObservation{8, tagAsSeenFrom(t, farTag), 0.9}});
        r.tick(0.01, 0.0, true);
        r.tick(0.01, 0.0, true);
        CHECK(r.loc.lastCorrection().audit.reason == GateReason::RejectedTagRange);
    }
}

// Would catch: a tag corrector that is not seeing tags nonetheless MOVING the estimate. "Degrades
// visibly, never a phantom pull" means it contributes nothing — bit for bit — not merely a
// little. Proven by running identical motion with and without the corrector wired.
TEST_CASE("tags: with no tags in view the estimate is bit-identical to having no corrector") {
    TagRig withTags{{}, /*wireCorrector=*/true};
    TagRig without{{}, /*wireCorrector=*/false};
    withTags.source.clear();
    without.source.clear();

    withTags.tick(0.01, 0.0, true);
    without.tick(0.01, 0.0, true);
    for (int i = 1; i <= 50; ++i) {
        const double travel = static_cast<double>(i);
        withTags.tick(0.01, travel, i % 5 == 0);
        without.tick(0.01, travel, i % 5 == 0);
    }

    CHECK(withTags.loc.pose().x().value() == without.loc.pose().x().value());
    CHECK(withTags.loc.pose().y().value() == without.loc.pose().y().value());
    CHECK(withTags.loc.pose().heading().radians() == without.loc.pose().heading().radians());
    CHECK(withTags.loc.headingBias().value() == 0.0);
    CHECK(withTags.loc.pose().x().value() != doctest::Approx(TagRig::truth().x().value()));
    // The difference is entirely in what the run can TELL you afterwards.
    CHECK(withTags.loc.lastCorrection().audit.reason == GateReason::RejectedNoFix);
    CHECK(without.loc.lastCorrection().audit.reason == GateReason::None);
    CHECK(std::string_view{withTags.loc.lastCorrection().source} == "tags");
    CHECK(std::string_view{without.loc.lastCorrection().source} == "none");
}

// ── RECONSTRUCTABILITY, through real bytes ───────────────────────────────────────────────────

// Would catch: a tag verdict that never reaches the file — a value the Localizer holds, the
// producer does not stamp, or the encoder skips. Each verdict is provoked by a REAL condition
// rather than scripted, and each must survive encode → device → decode.
TEST_CASE("blackbox: every real tag verdict arrives in the DECODED file") {
    AprilTagCorrectorConfig cfg;
    cfg.latency = Time{0.0};
    cfg.driftStdDevPerInch = 0.0;
    TagRig r{cfg};

    FakeBlockSink device;
    std::vector<DebugRecord> ring(32);
    std::vector<std::byte> buffer(65536);
    SdSink blackbox{device, r.clk, SdSinkStorage{ring, buffer},
                    SdSinkConfig{.streamTicks = true}};
    CommandIdStampSink stamp{blackbox};
    blackbox.open({});

    const auto emitTick = [&]() {
        stamp.setEstimatorAudit(r.loc.lastCorrection());
        DebugRecord rec;
        rec.t = r.clk.now();
        rec.measuredPose = r.loc.pose();
        stamp.emit(rec);
    };

    r.tick(0.01, 0.0, false);  // settle (dt is 0 on the very first tick)

    // (a) RejectedNoFix — nobody has polled.
    r.tick(0.01, 0.0, false);
    CHECK(r.loc.lastCorrection().audit.reason == GateReason::RejectedNoFix);
    emitTick();

    // (b) RejectedNoTagMapEntry — a tag is seen but the map has never heard of it.
    r.source.setTags({TagObservation{99, tagAsSeenFrom(TagRig::truth(), TagRig::tagPose()), 0.9}});
    r.tick(0.01, 0.0, true);
    CHECK(r.loc.lastCorrection().audit.reason == GateReason::RejectedNoTagMapEntry);
    emitTick();

    // (c) RejectedSensorQuality — a real tag, barely detected.
    r.showTruthfulTag(0.10);
    r.tick(0.01, 0.0, true);
    CHECK(r.loc.lastCorrection().audit.reason == GateReason::RejectedSensorQuality);
    emitTick();

    // (d) RejectedHighYawRate — a good frame taken mid-spin.
    r.showTruthfulTag();
    r.imu.setYawRate(AngularVelocity{3.0});
    r.tick(0.01, 0.0, true);
    CHECK(r.loc.lastCorrection().audit.reason == GateReason::RejectedHighYawRate);
    emitTick();
    r.imu.setYawRate(AngularVelocity{0.0});

    // (e) Accepted — a good fix, folded.
    r.showTruthfulTag();
    r.tick(0.01, 0.0, true);
    CHECK(r.loc.lastCorrection().audit.reason == GateReason::Accepted);
    emitTick();

    // (f) RejectedStaleFix — the same frame on the next loop tick.
    r.tick(0.01, 0.0, false);
    CHECK(r.loc.lastCorrection().audit.reason == GateReason::RejectedStaleFix);
    emitTick();

    // (g) RejectedObservationAge — the poller stops.
    r.tick(0.4, 0.0, false);
    CHECK(r.loc.lastCorrection().audit.reason == GateReason::RejectedObservationAge);
    emitTick();

    blackbox.close();
    const std::vector<DebugRecord> ticks = decodeTicks(device);
    REQUIRE(ticks.size() == 7);
    CHECK(ticks[0].gateReason == GateReason::RejectedNoFix);
    CHECK(ticks[1].gateReason == GateReason::RejectedNoTagMapEntry);
    CHECK(ticks[2].gateReason == GateReason::RejectedSensorQuality);
    CHECK(ticks[3].gateReason == GateReason::RejectedHighYawRate);
    CHECK(ticks[4].gateReason == GateReason::Accepted);
    CHECK(ticks[5].gateReason == GateReason::RejectedStaleFix);
    CHECK(ticks[6].gateReason == GateReason::RejectedObservationAge);
}

// Would catch: the HEADING nudge never reaching the file. `correctionDTheta` has been on the wire
// since F9 and zero on every record ever written; if E3 landed yaw correction without stamping
// it, the never-snap guarantee for heading would be unauditable from telemetry — which is the
// only place anyone can check it after a match.
TEST_CASE("blackbox: the heading nudge arrives on correctionDTheta, and it is bounded") {
    ComplementaryFusionConfig fcfg;
    AprilTagCorrectorConfig cfg;
    TagRig r{cfg, true, fcfg};
    // The IMU is 6 degrees wrong; the tag knows better.
    r.imu.setHeading(Angle::degrees(TagRig::truth().heading().degrees() - 6.0));
    r.showTruthfulTag();

    FakeBlockSink device;
    std::vector<DebugRecord> ring(64);
    std::vector<std::byte> buffer(131072);
    SdSink blackbox{device, r.clk, SdSinkStorage{ring, buffer},
                    SdSinkConfig{.streamTicks = true}};
    CommandIdStampSink stamp{blackbox};
    blackbox.open({});

    for (int k = 0; k < 40; ++k) {
        r.tick(0.01, 0.0, k % 5 == 0);
        stamp.setEstimatorAudit(r.loc.lastCorrection());
        DebugRecord rec;
        rec.t = r.clk.now();
        rec.measuredPose = r.loc.pose();
        stamp.emit(rec);
    }
    blackbox.close();

    const std::vector<DebugRecord> ticks = decodeTicks(device);
    REQUIRE(ticks.size() == 40);
    const double budgetRad = fcfg.maxHeadingNudgeRate.value() * 0.01;

    int nudged = 0;
    double total = 0.0;
    for (const DebugRecord& rec : ticks) {
        // NEVER-SNAP, audited from the FILE — which is the only place a team can check it after
        // a match. Every tick, without exception.
        CHECK(std::abs(rec.correctionDTheta.value()) <= budgetRad + 1e-12);
        if (rec.correctionDTheta.value() != 0.0) {
            ++nudged;
        }
        total += rec.correctionDTheta.value();
    }
    CHECK(nudged >= 6);              // the ~20 Hz vision cadence over 40 ticks
    CHECK(total > 0.0);              // and the nudges pull consistently one way, not at random
    // The file's own account and the estimator's live state agree.
    CHECK(r.loc.headingBias().value() == doctest::Approx(total).epsilon(1e-12));
}

// Would catch: a heading REJECTION being invisible. There is one `gateReason` slot and it is
// position-primary, so a heading-only rejection must be readable as "a large heading residual
// beside a zero heading correction". That reading rule is documented in complementary_fusion.hpp
// and in guide chapter 11; this is the test that makes it true.
TEST_CASE("blackbox: a gated heading reads as a big residual beside a zero nudge") {
    TagRig r;
    // The observation a robot at the RIGHT POSITION but facing 80 degrees away would produce.
    // The derived absolute pose therefore has a zero position residual and an 80-degree heading
    // residual — which is what a mirrored corner winding, a wrong map entry or a misidentified
    // id all look like from here. (Perturbing the relative heading alone does NOT do this: the
    // inversion rotates the position term by the robot's heading, so the derived position moves
    // too and the POSITION gate fires first. Learned the hard way; the distinction is the test.)
    const Pose2d skewed{TagRig::truth().x(), TagRig::truth().y(),
                        Angle::degrees(TagRig::truth().heading().degrees() + 80.0)};
    r.source.setTags({TagObservation{7, tagAsSeenFrom(skewed, TagRig::tagPose()), 0.9}});

    FakeBlockSink device;
    std::vector<DebugRecord> ring(32);
    std::vector<std::byte> buffer(65536);
    SdSink blackbox{device, r.clk, SdSinkStorage{ring, buffer},
                    SdSinkConfig{.streamTicks = true}};
    CommandIdStampSink stamp{blackbox};
    blackbox.open({});

    r.tick(0.01, 0.0, true);  // settle
    r.tick(0.01, 0.0, true);  // the frame that gets gated on heading
    stamp.setEstimatorAudit(r.loc.lastCorrection());
    DebugRecord rec;
    rec.t = r.clk.now();
    rec.measuredPose = r.loc.pose();
    stamp.emit(rec);
    blackbox.close();

    const std::vector<DebugRecord> ticks = decodeTicks(device);
    REQUIRE(ticks.size() == 1);
    // The reading rule, applied to the decoded bytes and nothing else:
    CHECK(std::abs(ticks[0].gateResidualHeading.value()) / kDeg == doctest::Approx(80.0).epsilon(1e-6));
    CHECK(ticks[0].correctionDTheta.value() == 0.0);
    CHECK(ticks[0].gateReason == GateReason::Accepted);  // POSITION was fine and was applied
    CHECK(r.loc.headingBias().value() == 0.0);
}

// Would catch: an ACCEPTED heading fix whose residual is not recorded, which would leave a reader
// unable to tell "the heading was right" from "the heading was never checked". On an accepted
// tick the residual must be the real innovation and the nudge must be a FRACTION of it — the
// numeric signature of a nudge rather than a snap, re-derived from the decoded record alone.
TEST_CASE("blackbox: an accepted heading fix records both its residual and its bounded nudge") {
    TagRig r;
    r.imu.setHeading(Angle::degrees(TagRig::truth().heading().degrees() - 2.0));
    r.showTruthfulTag();

    FakeBlockSink device;
    std::vector<DebugRecord> ring(32);
    std::vector<std::byte> buffer(65536);
    SdSink blackbox{device, r.clk, SdSinkStorage{ring, buffer},
                    SdSinkConfig{.streamTicks = true}};
    CommandIdStampSink stamp{blackbox};
    blackbox.open({});

    r.tick(0.01, 0.0, true);  // settle: dt is 0, nothing applied
    r.tick(0.01, 0.0, true);  // a fresh frame with a real 2-degree heading innovation
    stamp.setEstimatorAudit(r.loc.lastCorrection());
    DebugRecord rec;
    rec.t = r.clk.now();
    rec.measuredPose = r.loc.pose();
    stamp.emit(rec);
    blackbox.close();

    const std::vector<DebugRecord> ticks = decodeTicks(device);
    REQUIRE(ticks.size() == 1);
    const double residual = ticks[0].gateResidualHeading.value() / kDeg;
    const double nudge = ticks[0].correctionDTheta.value() / kDeg;
    CHECK(residual == doctest::Approx(2.0).epsilon(1e-6));
    CHECK(nudge > 0.0);
    CHECK(nudge < residual);                     // a nudge, not a snap — visible in the file
    CHECK(std::abs(nudge) <= 0.1 + 1e-12);       // and inside the documented per-tick budget
}
