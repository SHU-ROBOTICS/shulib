#pragma once
//
// AprilTagCorrector — the SECOND real corrector, and the FIRST source in the tree that can tell
// the estimator which way it is actually pointing (master plan §6/§8; WS5, chunk E3).
//
// E2's GpsCorrector bounded POSITION drift. Heading was still IMU-only, and heading is the half
// the team's `< 1°` end-of-run spec actually turns on — the master plan says so in as many
// words: "IMU-owned heading + absolute yaw correction (AprilTag/GPS) load-bearing, not
// optional". A tag observation is different in KIND from a GPS fix: `TagObservation::poseInRobot`
// is a relative POSE — position AND orientation — so against a tag whose field pose is known it
// yields an ABSOLUTE HEADING. That is what this class is for.
//
// It implements ICorrector behind the exact signature the seam has carried since M2. No caller
// changed; the Localizer does not know there are two kinds of corrector.
//
// ── THE TWO-METHOD SHAPE, AND WHY IT IS NOT NEGOTIABLE (chunk tension T4) ───────────────────
//   poll()     — call at VISION cadence, OFF the control loop. Reads ITagSource::tags().
//   propose()  — call every control tick. Reads the snapshot poll() left, plus the clock and the
//                IMU live. It NEVER touches ITagSource — that is what keeps it allocation-free.
//
// `ITagSource::tags()` returns `std::vector<TagObservation>` BY VALUE and is FROZEN (F4) — it
// cannot be changed, and it heap-allocates on every call. hal/vision.hpp is explicit that vision
// runs OFF the 10 ms control path and the adapter polls it at a lower rate. A propose() that
// called tags() would therefore put a heap allocation inside the control loop, which A1's cost
// contract forbids — and calling it "only every fifth tick" does not fix that, it just makes the
// allocation intermittent, which is worse to diagnose. So the cadence is the CALLER's, explicitly,
// and propose() is allocation-free on every path. Pinned, not asserted:
// test/apriltag_corrector_cost_test.cpp counts global allocations across 20,000 propose() calls.
//
// THE FOOTGUN THAT CREATES, stated plainly: a corrector nobody polls proposes nothing, forever,
// and silently. Three things make that diagnosable rather than mysterious — `pollCount()` is
// exposed, a never-polled corrector declines with `RejectedNoFix` (so the blackbox says so every
// tick), and a corrector whose poller STOPPED declines with `RejectedObservationAge`, which is a
// different word from every other kind of silence.
//
// ── WHAT IT DOES, AND IN WHAT ORDER ────────────────────────────────────────────────────────
//   1. record the predicted pose in a history ring (needed for latency, below);
//   2. never polled?          → decline, RejectedNoFix
//   3. snapshot too old?      → decline, RejectedObservationAge   (the poller stopped)
//   4. snapshot already used? → decline, RejectedStaleFix         (the double-count guard)
//   5. polled, saw nothing?   → decline, RejectedNoFix            (the off-camera path)
//   6. spinning fast?         → decline, RejectedHighYawRate      (motion blur / rolling shutter)
//   7. pick the single best tag against the map (below); no survivor → decline with the reason
//      that eliminated them, by a documented priority;
//   8. invert against the tag map → an absolute field POSE (x, y, AND heading);
//   9. compensate for pipeline latency, in position AND in heading;
//  10. normalized-innovation gate on position → decline, RejectedNormalizedInnovation;
//  11. otherwise propose, with providesHeading = true.
//
// ── WHY ONE TAG AND NOT ALL OF THEM ────────────────────────────────────────────────────────
// ICorrector returns ONE proposal, so N visible tags must become one answer. This class picks
// the tag with the SMALLEST estimated sigma and uses only that one. Rejected alternative:
// average the N absolute poses. Averaging positions is easy, averaging headings needs a circular
// mean, and both need a weighting scheme that is exactly the thing an EKF derives and a
// complementary tier can only invent. Worse, averaging HIDES disagreement: two tags implying
// poses 8 inches apart average to a confident, wrong answer with no residual to show for it,
// where "trust the best one" at least leaves the second tag's disagreement visible as a future
// innovation. Multi-tag triangulation is E4's, where a covariance makes it principled.
//
// ── SIGMA, AND WHY HEADING DOES NOT GET ITS OWN ────────────────────────────────────────────
//     sigma_meas = (baseStdDev + stdDevPerInch · range) / max(confidence, kMinConfidenceFloor)
//     sigma_dr   = hypot(postFixStdDev, driftStdDevPerInch · travelSinceFix)
//     confidence = sigma_dr² / (sigma_dr² + sigma_meas²)        — the scalar Kalman gain (E2 D1)
// Physically, PnP HEADING degrades faster with range than PnP POSITION does (the near-planar
// ambiguity — see hal/vision_conversion.hpp). Modelling that properly needs a second noise
// number on CorrectionProposal and a policy that uses it, which is E4's EKF. E3 handles it with
// the blunter instrument that does not require inventing a number: a TRUSTED RANGE BAND
// (A4: HA-73) outside which the observation is not used at all. Blunter, but honest about which
// of the two it is.
//
// ── LATENCY IS COMPENSATED IN HEADING TOO, NOT JUST POSITION ───────────────────────────────
// E2 compensates a GPS fix's ~50 ms of staleness by carrying it forward along the odometry. A
// tag fix needs the same treatment for POSITION and — new here — for HEADING, because a tag fix
// describes which way the robot was pointing ~80 ms ago (HA-71). At a brisk 180 deg/s that is
// 14 degrees, fourteen times the entire heading error budget. So the history ring carries an
// UNWRAPPED cumulative heading alongside the position, and the tag-derived heading is advanced
// by the rotation observed since capture. (Unwrapped, because interpolating raw wrapped headings
// across the ±π seam produces a garbage rotation exactly once per revolution.)
//
// That rotation is read from the IMU, NOT from `predicted.heading()` — see the note at step (1).
// Using the predicted heading feeds the correction back into itself and the bias overshoots.
//
// ── WHAT IT DELIBERATELY DOES NOT DO ───────────────────────────────────────────────────────
// * NO PnP. The corners→pose reduction is `hal::tagCornersToRobotPose`, a free pure function
//   called by R2's ADAPTER, and this file does not include it (chunk tension T3). The seam hands
//   this class an already-reduced `poseInRobot`.
// * NO SNAP — position or heading. This class only ever PROPOSES. How far the estimate moves is
//   the fusion policy's bounded, per-tick-clamped nudge (§13 #4), and for heading the Localizer's
//   bias accumulator moves by at most `maxHeadingNudgeRate · dt` per tick. There is no code path
//   here that writes a pose or a heading.
// * NO TAG MAP OF ITS OWN. Where the tags are is INPUT (tension T2, see tag_map.hpp). An empty
//   map makes every tag decline with RejectedNoTagMapEntry, loudly, rather than guessing.
//
// Pure w.r.t. its injected handles (clock, tag source, imu, map) and PROS-free. propose() never
// throws and never allocates; poll() allocates exactly as much as ITagSource::tags() does, off
// the control path, by design.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "shulib/core/check.hpp"
#include "shulib/diag/debug_record.hpp"
#include "shulib/hal/clock.hpp"
#include "shulib/hal/imu.hpp"
#include "shulib/hal/vision.hpp"
#include "shulib/localization/correction.hpp"
#include "shulib/localization/i_corrector.hpp"
#include "shulib/localization/tag_map.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::localization {

/// Tuning for AprilTagCorrector. Every default is PROVISIONAL — there is no robot, no camera and
/// no measured tag layout — and each carries its A4 Hardware Assumptions Register entry. E3
/// proves the corrector's LOGIC; R4 measures the constants. Nothing here was tuned to make the
/// simulated camera look good, which is an explicit non-goal of this chunk.
struct AprilTagCorrectorConfig {
    /// End-to-end delay between the instant a frame describes and the instant its reduced tags
    /// can be read (exposure + detect + PnP + transport). Larger than the GPS's because a tag
    /// pipeline does more work per frame. PROVISIONAL (A4: HA-71) — invented, ≈80 ms.
    units::Time latency{0.08};
    /// Decline once the newest snapshot is older than this: the vision task has stalled, died,
    /// or was never started. Distinct from "we looked and saw nothing" on purpose.
    /// PROVISIONAL (A4: HA-72).
    units::Time maxObservationAge{0.25};
    /// Trusted range band, measured from the ROBOT CENTRE. Below `minRange` the tag overfills
    /// the frame and is likely clipped; above `maxRange` the planar-PnP heading ambiguity
    /// (hal/vision_conversion.hpp) makes the orientation untrustworthy well before the position
    /// is. PROVISIONAL (A4: HA-73).
    units::Length minRange{6.0};
    /// Upper edge of that band (inches, from the robot centre). An observation outside
    /// [minRange, maxRange] is DISCARDED, not down-weighted — the blunt instrument E3 chose over
    /// inventing a second noise number for heading. Precondition: maxRange > minRange.
    units::Length maxRange{72.0};
    /// Detector confidence below this is not worth folding — the tag analogue of E2's
    /// sensor-quality ceiling (D7): without it, a 0.05-confidence detection is still folded with
    /// a microscopic pull, and the Localizer reports quality class Corrected on a run with no
    /// usable anchor. PROVISIONAL (A4: HA-74).
    double minConfidence = 0.35;
    /// Decline any observation taken while the yaw rate exceeded this. A spinning robot smears
    /// the tag across the frame, and a rolling shutter skews it into a different quadrilateral
    /// — which PnP will happily solve, into a confidently wrong pose. PROVISIONAL (A4: HA-75).
    units::AngularVelocity maxYawRate{2.0};
    /// Position 1σ of a tag fix at zero range and confidence 1, and its growth per inch of
    /// range. PROVISIONAL (A4: HA-76).
    units::Length baseStdDev{1.0};
    /// Growth of that 1σ per inch of RANGE — inches of σ per inch, so 0 makes a fix's σ
    /// range-independent. Must be >= 0.
    double stdDevPerInch = 0.02;
    /// Gate width in units of σ_eff, same meaning as E2's. PROVISIONAL (A4: HA-77).
    double gateSigma = 4.0;
    /// The estimate's position 1σ immediately after THIS source's fix is folded — the floor of
    /// σ_dr, so confidence is never 0. PROVISIONAL (A4: HA-78).
    units::Length postFixStdDev{1.0};
    /// Growth of the dead-reckoning 1σ per inch travelled since this source's last fix — the
    /// anti-lockout term E2's D2 exists to explain. PROVISIONAL (A4: HA-79).
    double driftStdDevPerInch = 0.02;
};

/// The corrector that turns one tag sighting into an ABSOLUTE field pose — position AND heading,
/// making it the first source in the tree that can tell the estimator which way it is actually
/// pointing. THE TWO-METHOD SHAPE IS THE CONTRACT, and getting it wrong fails silently: poll() is
/// the ONLY method that touches ITagSource, whose tags() returns a std::vector by value and so
/// heap-allocates, which is why poll() belongs on a VISION-rate task and propose() can run every
/// control tick allocating nothing. propose() is not sensor-free, though — it reads the injected
/// clock and the IMU (heading AND yaw rate) on every call, so both must be live and wired before
/// the control loop starts. A corrector nobody polls proposes nothing, forever — pollCount() and
/// a RejectedNoFix verdict every tick are what make that diagnosable. It picks the single best-σ
/// tag rather than averaging several, it computes no PnP (the seam hands it an already-reduced
/// pose), it owns no tag map, and it never writes a pose or a heading: it only ever PROPOSES, and
/// how far the estimate moves is the fusion policy's bounded nudge.
class AprilTagCorrector final : public ICorrector {
public:
    /// Ticks of predicted-pose history kept for latency compensation. 64 ticks is ~0.64 s at
    /// 100 Hz against an ~80 ms latency. Fixed capacity: the hot path never allocates.
    static constexpr std::size_t kHistory = 64;
    /// Tags kept from one poll. More than this in view at once means either a very tag-rich
    /// field or a detector hallucinating; either way the best-sigma pick only needs a few.
    static constexpr std::size_t kMaxTagsPerFrame = 8;
    /// Floor under the divisor in σ_meas, so a zero-confidence detection cannot produce an
    /// infinite σ (and, through it, a NaN). Below `minConfidence` anyway, so it is a numerical
    /// guard rather than a tuning knob — which is why it is a constant and not a config field.
    static constexpr double kMinConfidenceFloor = 0.05;

    /// `clock`, `tags`, `imu` and `map` are non-owning references that must outlive this
    /// corrector. `name` is the stable telemetry id reported by name() and stamped into
    /// AppliedCorrection::source.
    AprilTagCorrector(hal::IClock& clock, hal::ITagSource& tags, hal::IImu& imu, const TagMap& map,
                      const AprilTagCorrectorConfig& config = {}, const char* name = "tags")
        : clock_{clock}, tags_{tags}, imu_{imu}, map_{map}, config_{config}, name_{name} {
        SHULIB_PRECONDITION(config.latency.value() >= 0.0,
                            "AprilTagCorrector: latency must be >= 0");
        SHULIB_PRECONDITION(config.maxObservationAge.value() > 0.0,
                            "AprilTagCorrector: maxObservationAge must be > 0");
        SHULIB_PRECONDITION(config.minRange.value() >= 0.0,
                            "AprilTagCorrector: minRange must be >= 0");
        SHULIB_PRECONDITION(config.maxRange.value() > config.minRange.value(),
                            "AprilTagCorrector: maxRange must exceed minRange");
        SHULIB_PRECONDITION(config.minConfidence > 0.0 && config.minConfidence <= 1.0,
                            "AprilTagCorrector: minConfidence must be in (0, 1]");
        SHULIB_PRECONDITION(config.maxYawRate.value() > 0.0,
                            "AprilTagCorrector: maxYawRate must be > 0");
        SHULIB_PRECONDITION(config.baseStdDev.value() > 0.0,
                            "AprilTagCorrector: baseStdDev must be > 0");
        SHULIB_PRECONDITION(config.stdDevPerInch >= 0.0,
                            "AprilTagCorrector: stdDevPerInch must be >= 0");
        SHULIB_PRECONDITION(config.gateSigma > 0.0, "AprilTagCorrector: gateSigma must be > 0");
        SHULIB_PRECONDITION(config.postFixStdDev.value() > 0.0,
                            "AprilTagCorrector: postFixStdDev must be > 0");
        SHULIB_PRECONDITION(config.driftStdDevPerInch >= 0.0,
                            "AprilTagCorrector: driftStdDevPerInch must be >= 0");
        SHULIB_PRECONDITION(name != nullptr, "AprilTagCorrector: name must not be null");
    }

    /// Take one frame from the tag source. **Call this from a vision-rate task, NEVER from the
    /// control loop** (header note, tension T4): this is the method that allocates.
    ///
    /// A poll that sees NOTHING is still information — "we looked, the camera is alive, there
    /// was no tag" — and is recorded as such, which is how the off-camera path stays
    /// distinguishable from a dead vision task.
    void poll() {
        const std::vector<hal::TagObservation> seen = tags_.tags();  // the F4 allocation, off-path
        const std::size_t n = std::min(seen.size(), kMaxTagsPerFrame);
        for (std::size_t k = 0; k < n; ++k) {
            frame_[k] = seen[k];
        }
        frameCount_ = n;
        frameTime_ = clock_.now().value();
        ++frameSeq_;
        ++pollCount_;
        haveFrame_ = true;
    }

    /// One tick of the sequence in the header note. Never throws, never allocates; `dt` is
    /// unused because this corrector timestamps from the injected clock (E2's D5).
    [[nodiscard]] CorrectionProposal propose(const math::Pose2d& predicted,
                                             units::Time /*dt*/) override {
        const double now = clock_.now().value();
        const double px = predicted.x().value();
        const double py = predicted.y().value();
        const double ph = predicted.heading().radians();
        if (!std::isfinite(now) || !std::isfinite(px) || !std::isfinite(py) || !std::isfinite(ph)) {
            return decline(diag::GateReason::RejectedNoFix);
        }

        // (1) history + dead-reckon travel accounting. Both advance on EVERY tick, including
        // ticks with no tag — off-camera is precisely when σ_dr must keep growing (E2's D2).
        //
        // THE ROTATION HISTORY IS TAKEN FROM THE IMU, NOT FROM `predicted.heading()`. The term
        // means "how far has the robot TURNED since the frame was captured", and since E3 the
        // predicted heading is `imu + learned bias`, so its tick-to-tick change contains both
        // real rotation AND the estimator's own bias learning. Feeding that back in makes the
        // EFFECTIVE heading gain depend on the latency window and the loop rate — a hidden
        // coupling, and simply the wrong quantity. The IMU is the authority on rotation and its
        // reading is bias-free by construction, which is exactly what this needs.
        //
        // HONEST SCOPE OF THE CLAIM (E3): this is an argument from the algebra, not from a
        // measurement. Measured over a 12-degree correction the two versions differ by about
        // 0.1% in convergence rate (11.8852 vs 11.8987 degrees at 9 s) and NEITHER overshoots,
        // so no test in this suite separates them — the E3 record carries that as an open
        // mutation hole rather than pretending otherwise.
        const math::Angle imuHeading = imu_.heading();
        if (havePrev_) {
            travelSinceFix_ += std::hypot(px - prevX_, py - prevY_);
            unwrappedHeading_ += prevHeading_.errorTo(imuHeading);
        }
        prevX_ = px;
        prevY_ = py;
        prevHeading_ = imuHeading;
        havePrev_ = true;
        push(now, px, py, unwrappedHeading_);

        // (2) never polled. Not the same as "polled and saw nothing" — this one means the
        // caller never wired a vision task at all, and it says so from the very first tick.
        if (!haveFrame_) {
            ++noFrameTicks_;
            return decline(diag::GateReason::RejectedNoFix);
        }

        // (3) the poller stopped (or the vision task died mid-match). Its own word, because
        // "vision went away" and "no tag is in view" call for completely different responses.
        if (!std::isfinite(frameTime_) || now - frameTime_ > config_.maxObservationAge.value()) {
            ++staleFrameTicks_;
            return decline(diag::GateReason::RejectedObservationAge);
        }

        // (4) freshness: one frame is folded ONCE. At ~20 Hz vision against a ~100 Hz loop, a
        // corrector that folded every tick would count one observation five times (E2's D3).
        if (frameSeq_ == foldedSeq_) {
            ++staleTicks_;
            return decline(diag::GateReason::RejectedStaleFix);
        }
        foldedSeq_ = frameSeq_;  // consumed HERE, before any later rejection (E2's D8): a frame
                                 // taken mid-spin is skipped, not folded once the spin ends.

        // (5) the camera is alive and looking at nothing. The off-camera path — and NEVER a
        // low-confidence pull toward some default pose.
        if (frameCount_ == 0) {
            ++noTagTicks_;
            return decline(diag::GateReason::RejectedNoFix);
        }

        // (6) spinning too fast to trust the geometry (header note).
        const double yawRate = imu_.yawRate().value();
        if (!std::isfinite(yawRate) || std::abs(yawRate) > config_.maxYawRate.value()) {
            ++yawRateRejects_;
            return decline(diag::GateReason::RejectedHighYawRate);
        }

        // (7) pick the single best tag: smallest σ_meas among those that are in the map, inside
        // the trusted range band, and above the confidence floor.
        std::size_t bestIndex = kMaxTagsPerFrame;
        double bestSigma = 0.0;
        bool sawUnmapped = false;
        bool sawOutOfRange = false;
        bool sawLowConfidence = false;
        for (std::size_t k = 0; k < frameCount_; ++k) {
            const hal::TagObservation& obs = frame_[k];
            const double rx = obs.poseInRobot.x().value();
            const double ry = obs.poseInRobot.y().value();
            const double rh = obs.poseInRobot.heading().radians();
            if (!std::isfinite(rx) || !std::isfinite(ry) || !std::isfinite(rh) ||
                !std::isfinite(obs.confidence)) {
                sawLowConfidence = true;  // a non-finite detection is a quality failure, not a fix
                continue;
            }
            if (map_.find(obs.id) == nullptr) {
                sawUnmapped = true;
                continue;
            }
            const double range = std::hypot(rx, ry);
            if (range < config_.minRange.value() || range > config_.maxRange.value()) {
                sawOutOfRange = true;
                continue;
            }
            if (obs.confidence < config_.minConfidence) {
                sawLowConfidence = true;
                continue;
            }
            const double sigma = (config_.baseStdDev.value() + config_.stdDevPerInch * range) /
                                 std::max(obs.confidence, kMinConfidenceFloor);
            if (bestIndex == kMaxTagsPerFrame || sigma < bestSigma) {
                bestIndex = k;
                bestSigma = sigma;
            }
        }
        if (bestIndex == kMaxTagsPerFrame) {
            // Nothing survived. PRIORITY, documented so the verdict is deterministic and
            // meaningful: a MISSING MAP ENTRY outranks the others because it is a configuration
            // error the team can fix, where range and confidence are just the field being the
            // field. Every category also has its own counter, so the single reason slot is not
            // the only channel.
            if (sawUnmapped) {
                ++unmappedRejects_;
                return decline(diag::GateReason::RejectedNoTagMapEntry);
            }
            if (sawOutOfRange) {
                ++rangeRejects_;
                return decline(diag::GateReason::RejectedTagRange);
            }
            if (sawLowConfidence) {
                ++qualityRejects_;
                return decline(diag::GateReason::RejectedSensorQuality);
            }
            ++noTagTicks_;
            return decline(diag::GateReason::RejectedNoFix);
        }

        // (8) the inversion: tag field pose + tag-relative pose → absolute robot pose. The map
        // owns this arithmetic (tag_map.hpp), not this class.
        const hal::TagObservation& best = frame_[bestIndex];
        const TagPlacement* placement = map_.find(best.id);
        const math::Pose2d absolute =
            TagMap::robotPoseFromTag(placement->fieldPose, best.poseInRobot);
        const double zx0 = absolute.x().value();
        const double zy0 = absolute.y().value();
        if (!std::isfinite(zx0) || !std::isfinite(zy0)) {
            ++qualityRejects_;
            return decline(diag::GateReason::RejectedSensorQuality);
        }

        // (9) latency, in BOTH position and heading (header note). The frame became readable at
        // frameTime_ having been captured `latency` before that.
        const double captureTime = frameTime_ - config_.latency.value();
        double baseX = px;
        double baseY = py;
        double baseH = unwrappedHeading_;
        stateAt(captureTime, baseX, baseY, baseH);
        const double zx = zx0 + (px - baseX);
        const double zy = zy0 + (py - baseY);
        const double zh = absolute.heading().radians() + (unwrappedHeading_ - baseH);
        if (!std::isfinite(zx) || !std::isfinite(zy) || !std::isfinite(zh)) {
            ++qualityRejects_;
            return decline(diag::GateReason::RejectedSensorQuality);
        }

        // (10) the normalized-innovation gate, on POSITION. Same construction and the same
        // honest name as E2's (this is not a Mahalanobis distance; no covariance exists yet).
        const double sigmaMeas = bestSigma;
        const double sigmaDr = std::hypot(config_.postFixStdDev.value(),
                                          config_.driftStdDevPerInch * travelSinceFix_);
        const double sigmaEff = std::hypot(sigmaMeas, sigmaDr);
        const double residualX = zx - px;
        const double residualY = zy - py;
        const double residual = std::hypot(residualX, residualY);
        const double residualH = predicted.heading().errorTo(math::Angle::radians(zh));
        if (!std::isfinite(residual) || !std::isfinite(sigmaEff) || sigmaEff <= 0.0 ||
            residual > config_.gateSigma * sigmaEff) {
            ++innovationRejects_;
            return decline(diag::GateReason::RejectedNormalizedInnovation, residualX, residualY,
                           residualH, sigmaEff);
        }

        // (11) propose — position AND heading. `providesHeading` stops being RESERVED here.
        const double sdr2 = sigmaDr * sigmaDr;
        const double confidence = sdr2 / (sdr2 + sigmaMeas * sigmaMeas);
        travelSinceFix_ = 0.0;
        ++accepted_;
        lastVerdict_ = diag::GateReason::Accepted;
        lastTagId_ = best.id;

        CorrectionProposal p;
        p.valid = true;
        p.fieldPose =
            math::Pose2d{units::Length{zx}, units::Length{zy}, math::Angle::radians(zh)};
        p.confidence = confidence;
        p.positionStdDev = units::Length{sigmaEff};
        p.providesHeading = true;
        return p;
    }

    /// The stable telemetry id given at construction ("tags" unless overridden). Read it as an
    /// IDENTITY, not as attribution: the Localizer stamps AppliedCorrection::source with the
    /// FIRST corrector in registration order that returned a VALID proposal that tick, while the
    /// complementary policy folds the sum of every accepted proposal — so with two correctors
    /// registered the name tells you who was asked first, not whose fix moved the estimate. It
    /// also carries this name on the other path: when nothing reached the policy, source names
    /// the corrector whose DECLINE the record is reporting. Exact with one corrector only. The
    /// pointer is stored, NOT copied, so the caller's string must outlive this corrector.
    [[nodiscard]] const char* name() const noexcept override { return name_; }

    // ── per-source accounting (the "visible when vision is not helping" requirement) ────────

    /// What this corrector decided on the most recent propose() call.
    [[nodiscard]] diag::GateReason lastVerdict() const noexcept { return lastVerdict_; }
    /// The id of the tag most recently PROPOSED from, or -1 if none ever was. Names WHICH tag
    /// the estimate is anchored to, which is the first question when a fix looks wrong.
    [[nodiscard]] int lastTagId() const noexcept { return lastTagId_; }
    /// Frames taken from the tag source since construction. Zero means nobody is polling.
    [[nodiscard]] std::uint32_t pollCount() const noexcept { return pollCount_; }
    /// Valid proposals returned since construction (the Localizer screens them again, and the
    /// fusion policy may still gate one, so this is not a count of estimate moves). At most ONE
    /// per polled frame — a frame is folded once — so it can never exceed pollCount().
    [[nodiscard]] std::uint32_t acceptedFixes() const noexcept { return accepted_; }
    /// Ticks before the very first poll — the "nobody wired the vision task" number.
    [[nodiscard]] std::uint32_t noFrameTicks() const noexcept { return noFrameTicks_; }
    /// Ticks whose newest frame was older than maxObservationAge — the poller stopped.
    [[nodiscard]] std::uint32_t staleFrameTicks() const noexcept { return staleFrameTicks_; }
    /// Ticks that re-read a frame already folded (the normal steady state at 20 Hz vs 100 Hz).
    [[nodiscard]] std::uint32_t staleTicks() const noexcept { return staleTicks_; }
    /// Fresh frames with no tag in view at all — the off-camera path.
    [[nodiscard]] std::uint32_t noTagTicks() const noexcept { return noTagTicks_; }
    /// Fresh frames whose every tag was absent from the map. A configuration error, counted
    /// separately because it is the one the team can actually fix.
    [[nodiscard]] std::uint32_t unmappedRejects() const noexcept { return unmappedRejects_; }
    /// Fresh frames whose every tag was outside the trusted range band.
    [[nodiscard]] std::uint32_t rangeRejects() const noexcept { return rangeRejects_; }
    /// Fresh frames whose every tag was below the confidence floor (or non-finite).
    [[nodiscard]] std::uint32_t qualityRejects() const noexcept { return qualityRejects_; }
    /// Fresh frames declined because the robot was spinning too fast.
    [[nodiscard]] std::uint32_t yawRateRejects() const noexcept { return yawRateRejects_; }
    /// Fresh fixes declined by the normalized-innovation gate.
    [[nodiscard]] std::uint32_t innovationRejects() const noexcept { return innovationRejects_; }
    /// Distance the prediction has travelled since this source last proposed — the anti-lockout
    /// input, exposed so a test can prove the widening is real rather than asserted.
    [[nodiscard]] units::Length travelSinceFix() const noexcept {
        return units::Length{travelSinceFix_};
    }

private:
    /// A declined proposal carrying its reason and, for the gate, the numbers the verdict was
    /// rendered on — so a reader can recompute ν from the blackbox alone (E2's discipline).
    [[nodiscard]] CorrectionProposal decline(diag::GateReason reason, double residualX = 0.0,
                                             double residualY = 0.0, double residualHeading = 0.0,
                                             double sigmaEff = 0.0) noexcept {
        lastVerdict_ = reason;
        CorrectionProposal p;  // valid == false
        p.selfAudit.reason = reason;
        p.selfAudit.residualX = units::Length{residualX};
        p.selfAudit.residualY = units::Length{residualY};
        p.selfAudit.residualHeading = units::AngleDim{residualHeading};
        p.selfAudit.covarianceTrace = sigmaEff;
        return p;
    }

    void push(double t, double x, double y, double h) noexcept {
        hist_[head_] = Sample{t, x, y, h};
        head_ = (head_ + 1) % kHistory;
        if (count_ < kHistory) {
            ++count_;
        }
    }

    /// Predicted state at time `t`, by linear interpolation between the two bracketing ring
    /// samples. Clamps to the oldest sample when the ring does not reach back that far: partial
    /// compensation, never extrapolation past what was observed. The heading component is the
    /// UNWRAPPED cumulative angle, so this interpolation is exact across the ±π seam.
    void stateAt(double t, double& x, double& y, double& h) const noexcept {
        if (count_ == 0) {
            return;
        }
        const std::size_t oldest = (head_ + kHistory - count_) % kHistory;
        if (t <= hist_[oldest].t) {
            x = hist_[oldest].x;
            y = hist_[oldest].y;
            h = hist_[oldest].h;
            return;
        }
        for (std::size_t k = count_; k-- > 1;) {  // newest→oldest, looking for the bracket
            const Sample& newer = hist_[(oldest + k) % kHistory];
            const Sample& older = hist_[(oldest + k - 1) % kHistory];
            if (t >= older.t && t <= newer.t) {
                const double span = newer.t - older.t;
                const double f = span > 0.0 ? (t - older.t) / span : 1.0;
                x = older.x + f * (newer.x - older.x);
                y = older.y + f * (newer.y - older.y);
                h = older.h + f * (newer.h - older.h);
                return;
            }
        }
        const Sample& newest = hist_[(head_ + kHistory - 1) % kHistory];
        x = newest.x;  // t is at or after the newest sample: no motion to add
        y = newest.y;
        h = newest.h;
    }

    struct Sample {
        double t = 0.0;
        double x = 0.0;
        double y = 0.0;
        double h = 0.0;  ///< UNWRAPPED cumulative heading (header note)
    };

    hal::IClock& clock_;
    hal::ITagSource& tags_;
    hal::IImu& imu_;
    const TagMap& map_;
    AprilTagCorrectorConfig config_;
    const char* name_;

    std::array<hal::TagObservation, kMaxTagsPerFrame> frame_{};
    std::size_t frameCount_ = 0;
    double frameTime_ = 0.0;
    std::uint32_t frameSeq_ = 0;
    std::uint32_t foldedSeq_ = 0;
    bool haveFrame_ = false;

    std::array<Sample, kHistory> hist_{};
    std::size_t head_ = 0;
    std::size_t count_ = 0;

    double prevX_ = 0.0;
    double prevY_ = 0.0;
    math::Angle prevHeading_{};
    bool havePrev_ = false;
    double travelSinceFix_ = 0.0;
    double unwrappedHeading_ = 0.0;

    diag::GateReason lastVerdict_ = diag::GateReason::None;
    int lastTagId_ = -1;
    std::uint32_t pollCount_ = 0;
    std::uint32_t accepted_ = 0;
    std::uint32_t noFrameTicks_ = 0;
    std::uint32_t staleFrameTicks_ = 0;
    std::uint32_t staleTicks_ = 0;
    std::uint32_t noTagTicks_ = 0;
    std::uint32_t unmappedRejects_ = 0;
    std::uint32_t rangeRejects_ = 0;
    std::uint32_t qualityRejects_ = 0;
    std::uint32_t yawRateRejects_ = 0;
    std::uint32_t innovationRejects_ = 0;
};

}  // namespace shulib::localization
