#pragma once
//
// Localizer — the fused field-frame estimate (master plan §5/§6/§8; WS5). It is a thin, deterministic
// orchestrator over three already-tested pieces: PilonsOdometry (high-rate prediction), the IMU
// (heading authority), and a list of correctors (absolute fixes) behind an IFusionPolicy. It owns
// the four jobs the pieces below cannot, in a fixed five-step update():
//   1. dt — source it from the injected IClock and turn the per-tick position change into a Twist2d.
//   2. predict — advance PilonsOdometry; its pose is the dead-reckon prediction.
//   3. fuse — ask each corrector for an absolute proposal and fold the valid ones in as an
//      innovation-bounded, per-tick-clamped GATED NUDGE (position only), via the IFusionPolicy.
//   4. heading — compose the fused heading from the IMU as the LAST write of the tick, so no
//      corrector or policy can ever assign the robot a heading (decision #4). Since E3 the
//      composition is `imu.heading() + headingBias_`, where the bias moves by at most a bounded
//      nudge per tick — see the heading-bias note below.
//   5. publish — recompute the quality scalar + categorical flags from observable inputs.
//
// ── THE HEADING BIAS (added at E3, the absolute-yaw path M2 reserved) ──
// Until E3 nothing in the tree could tell the estimator its heading was wrong, so STEP 4 stamped
// the raw IMU reading and heading was IMU-owned in the strongest possible sense. That was correct
// while it lasted and is NOT what the accuracy spec needs: the master plan records that a raw V5
// IMU drifts ≈1°/min, so dead-reckoned heading alone will not hold the team's `< 1°` requirement
// across a 60-second run, and makes "IMU-owned heading + absolute yaw correction" load-bearing.
// AprilTagCorrector supplies the absolute heading; this is where it lands.
//
// The published heading is `imu.heading() + headingBias_`. The IMU remains the SOLE source of
// heading CHANGE — every rotation the robot makes still enters through imu.heading(), tick for
// tick, and PilonsOdometry is untouched — while the bias is a slowly-learned constant offset that
// moves by at most `maxHeadingNudgeRate · dt` per tick (ComplementaryFusion). There is no code
// path, here or anywhere below, that ASSIGNS a heading: the policy can only return an increment.
// That is what makes never-snap (§13 #4) structural for yaw rather than a promise.
//
// WHY THE BIAS MUST PERSIST, which is the whole design. The Localizer re-reads the IMU every tick,
// so a heading correction that only decorated THIS tick's published heading would be discarded on
// the next one — individually sane nudges, collectively useless. That is verbatim the M2 red
// team's "corrections not accumulating" failure mode. The accumulator is exactly the structure
// STEP 2 already uses for position (fusedX_ persists and is advanced by odom DELTAS, never reset
// to absolute odom). It is deliberately NOT capped: the nudge is always toward an absolute
// measurement, so the loop is closed and cannot run away, whereas a cap would silently lock out
// recovery from precisely the large real drift that makes the correction worth having — the same
// gate-lockout failure E2's D2 identified for position.
//
// AND THE CONSEQUENCE FOR POSITION. PilonsOdometry rotates its per-tick field delta by the RAW
// IMU heading. Left alone, a learned bias `b` would fix the reported heading while the position
// prediction kept accumulating a cross-track error of roughly `b × distance` — which is most of
// what heading drift actually costs over a run. So STEP 2 rotates the odometry delta by `b` before
// folding it. At `b == 0` this is skipped by an explicit early-out, so a tree with no
// heading-providing corrector is BIT-IDENTICAL to before E3 by construction, not by a
// floating-point argument.
//
// Swapping the complementary filter for an EKF later is a one-argument change (IFusionPolicy); the
// IPoseSource read seam and the ICorrector write seam never move. At M2 the corrector list is empty
// or holds only NullCorrector, so the default-tested path is dead-reckoning (odom + IMU).
//
// ── The IMU-readiness boot guard (added at A3, after the hostile fakes drew blood) ──
// A calibrating IMU emits GARBAGE THAT MOVES, and the odometry offset correction converts garbage
// heading swings into phantom translation (Δθ·offset per tick) — observed at A3: a robot sitting
// STILL through a 2 s calibration window ended 10.8 inches from where it started, permanently
// (no corrector at M2 can heal it), while quality honestly said 0. The flag was honest; the pose
// was not. So update() now distinguishes three IMU states:
//   * NEVER been ready (boot): odometry still consumes its wheel deltas tick-by-tick (so the
//     transition tick sees one tick's travel, not the whole boot's), but the fused position folds
//     NOTHING — no odom deltas, no corrector proposals. Quality: Uninitialized, 0.
//   * THE SETTLE WINDOW (the second A3 finding, caught only by the COMPOSED hostile model):
//     isReady() is a status flag; the heading STREAM is a data path with its own latency. When
//     calibration garbage and sensor latency are both live, ready flips true while the stream is
//     still serving delayed garbage — observed: one post-transition fold differenced against a
//     delayed-garbage prevHeading leaked 3.65 in. So after a WITNESSED not-ready phase, the fold
//     stays closed for `bootSettleTime` past the first ready tick (quality stays Uninitialized —
//     the estimate is not live yet). A boot that was ready from the very first update() has no
//     boundary in its stream and takes no settle hold, so the normal path is byte-identical to
//     before. Consequence, stated as the consumer contract: motion commanded before qualityClass
//     leaves Uninitialized is unaccounted — C1's loop waits for a live estimate (real autons wait
//     out calibration anyway).
//   * ready: normal operation.
//   * WAS ready, lost mid-run (dropout): deltas KEEP folding — the encoders are still good and a
//     stale-heading estimate beats a frozen one — but quality reports Degraded (NOT Uninitialized:
//     a robot that had a fix and lost its heading authority must be distinguishable from one still
//     booting, or a skills gate applies the wrong recovery) with the scalar pinned to 0.
// PilonsOdometry itself stays readiness-blind on purpose — its header assigns recovery policy to
// this layer, and a second gate below would hide this one's absence from the tests.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <span>

#include "shulib/core/check.hpp"
#include "shulib/hal/clock.hpp"
#include "shulib/hal/imu.hpp"
#include "shulib/localization/correction.hpp"
#include "shulib/localization/i_corrector.hpp"
#include "shulib/localization/i_fusion_policy.hpp"
#include "shulib/localization/i_pose_source.hpp"
#include "shulib/localization/pilons_odometry.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::localization {

struct LocalizerConfig {
    /// Above this tick dt (s), the linear-velocity finite-difference is not trusted (first tick after
    /// construction/teleport, or a loop stall) → zero linear velocity for that tick + a flagged tick.
    double maxDt = 0.1;
    /// Below this tick dt (s), the finite-difference is likewise not trusted (a near-zero interval
    /// would otherwise blow up into an unphysical velocity spike).
    double minDt = 1e-4;
    /// distanceSinceCorrection at which the quality scalar decays to qFloor (drift erodes trust as we
    /// dead-reckon farther — process noise scales with travel). Default ~ one foot — an INVENTED
    /// drift-rate guess until R4 measures real dead-reckon drift (A4 register HA-36).
    units::Length driftHorizon{12.0};
    /// Quality floor while dead-reckoning far from a fix, in [0,1).
    double qFloor = 0.2;
    /// How long after a WITNESSED not-ready→ready transition the fold stays closed while the
    /// delayed sensor data path flushes its boot-boundary garbage (the settle window — header
    /// note). Applies ONLY when a not-ready phase was observed; a ready-from-construction boot
    /// takes no hold. Must cover the worst sensor data-path latency; 0.1 s clears the ~50 ms
    /// GPS-class delay with margin (adequacy vs. REAL latencies: A4 register HA-35, R4 measures).
    double bootSettleTime = 0.1;
};

class Localizer final : public IPoseSource {
public:
    /// Categorical health for motion/skills gating (distinct from the [0,1] scalar).
    enum class Quality { Uninitialized, DeadReckon, Corrected, Degraded };

    /// At most this many correctors (GPS + AI-Vision tag + Pi tag + LIDAR today) — the valid-proposal
    /// buffer is fixed-capacity so the hot path never heap-allocates.
    static constexpr std::size_t kMaxCorrectors = 4;

    /// `correctors` is a NON-OWNING view: the backing array (and the correctors it points to) must
    /// outlive the Localizer. Empty at M2 (dead-reckon). All references are validated non-null.
    Localizer(hal::IClock& clock, hal::IImu& imu, PilonsOdometry& odom, IFusionPolicy& fusion,
              std::span<ICorrector* const> correctors = {}, const LocalizerConfig& config = {})
        : clock_{clock}, imu_{imu}, odom_{odom}, fusion_{fusion}, correctors_{correctors},
          config_{config}, pose_{odom.pose()} {
        SHULIB_PRECONDITION(config.maxDt > 0.0, "Localizer: maxDt must be > 0");
        SHULIB_PRECONDITION(config.driftHorizon.value() > 0.0, "Localizer: driftHorizon must be > 0");
        SHULIB_PRECONDITION(config.qFloor >= 0.0 && config.qFloor < 1.0,
                            "Localizer: qFloor must be in [0, 1)");
        SHULIB_PRECONDITION(config.bootSettleTime >= 0.0,
                            "Localizer: bootSettleTime must be >= 0");
        SHULIB_PRECONDITION(correctors.size() <= kMaxCorrectors, "Localizer: too many correctors");
        for (ICorrector* c : correctors_) {
            SHULIB_PRECONDITION(c != nullptr, "Localizer: a corrector is null");
        }
        fusedX_ = pose_.x().value();
        fusedY_ = pose_.y().value();
        lastFusedX_ = fusedX_;
        lastFusedY_ = fusedY_;
        lastOdomX_ = odom_.pose().x().value();
        lastOdomY_ = odom_.pose().y().value();
    }

    /// One fused tick (the five steps above).
    void update() {
        // STEP 1 — dt from the injected clock.
        const double now = clock_.now().value();
        const double dt = hasLast_ ? (now - lastNow_) : 0.0;
        const bool dtHealthy = hasLast_ && dt >= config_.minDt && dt <= config_.maxDt;

        // STEP 2 — predict: advance the PERSISTENT fused position by the per-tick odom field-frame
        // DELTA (NOT a reset to absolute odom). This is what lets a correction accumulate and persist
        // after a corrector goes quiet — the never-snap nudge can now actually converge & bound drift.
        // BOOT GUARD + SETTLE WINDOW (header note): before the IMU has EVER been ready, the deltas
        // are built on garbage heading — odometry still consumes its wheels, but the fused position
        // folds zero. After a WITNESSED not-ready phase, the fold stays closed a further
        // bootSettleTime past the first ready tick, because the heading data path may still be
        // serving delayed boot-boundary garbage after the status flag flips (the composed-hostility
        // finding). A ready-from-first-update boot sets settleUntil = now: no hold, no behavior
        // change on the normal path.
        const bool readyNow = imu_.isReady();
        if (!readyNow && !imuEverReady_) {
            sawNotReady_ = true;
        }
        if (readyNow && !imuEverReady_) {
            imuEverReady_ = true;
            settleUntil_ = sawNotReady_ ? now + config_.bootSettleTime : now;
        }
        const bool foldDeltas = imuEverReady_ && now >= settleUntil_;
        settled_ = foldDeltas;
        odom_.update();
        const math::Pose2d odomNow = odom_.pose();
        double odx = foldDeltas ? (odomNow.x().value() - lastOdomX_) : 0.0;
        double ody = foldDeltas ? (odomNow.y().value() - lastOdomY_) : 0.0;
        // E3: the odometry rotated this delta by the RAW IMU heading; re-express it under the
        // corrected one (header note). The early-out is not an optimisation — it is what makes a
        // no-heading-corrector tree bit-identical to pre-E3 by construction.
        if (headingBias_ != 0.0) {
            const double cb = std::cos(headingBias_);
            const double sb = std::sin(headingBias_);
            const double rx = odx * cb - ody * sb;
            const double ry = odx * sb + ody * cb;
            odx = rx;
            ody = ry;
        }
        const math::Angle rawHeading = imu_.heading();
        const math::Angle heading = biasedHeading(rawHeading);
        const math::Pose2d predicted{units::Length{fusedX_ + odx}, units::Length{fusedY_ + ody}, heading};

        // STEP 3 — gather VALID proposals (screened, incl. FINITE confidence so an Inf can't sail
        // through). Skipped entirely during boot (header note): an absolute fix folded into a
        // heading-garbage tick would move a pose the quality flags say does not exist yet.
        std::array<CorrectionProposal, kMaxCorrectors> buf{};
        std::array<const char*, kMaxCorrectors> names{};
        std::size_t n = 0;
        // E2: the verdict of a corrector that declined to propose and said why. A declined
        // proposal never reaches the fusion policy, so without this the record could not tell
        // "the GPS is off the strip" from "no corrector is registered" — see
        // CorrectionProposal::selfAudit. Kept as a plain pair, not a list: the record has ONE
        // gating slot.
        //
        // E3 — WHICH silent source wins that one slot, now that there are two. E2 left this as
        // "the first to decline", which was fine with one corrector and is a real choice with
        // two. Taken deliberately: an EXCEPTIONAL verdict outranks a ROUTINE one, where routine
        // means RejectedStaleFix. The reasoning is E2's own measurement — a healthy source
        // spends the large majority of its ticks stale (2520 stale against ~570 fresh in 30 s,
        // the ~20 Hz sensor cadence against the ~100 Hz loop), so under a plain first-wins rule
        // the first-registered corrector's ordinary staleness would mask a SECOND corrector's
        // genuine failure — an unmapped tag id, a dead vision task — for essentially the whole
        // run. Among equals, the first still wins, so the rule stays deterministic. Either way
        // AppliedCorrection::source names whose verdict it is, so the reason is never ambiguous
        // about which source it belongs to.
        GateAudit selfAudit{};
        const char* selfAuditSource = nullptr;
        bool selfAuditRoutine = false;
        if (foldDeltas) {
            for (ICorrector* c : correctors_) {
                const CorrectionProposal p = c->propose(predicted, units::Time{dt});
                if (p.valid && std::isfinite(p.confidence) && p.confidence > 0.0 &&
                    p.positionStdDev.value() > 0.0 &&
                    std::isfinite(p.fieldPose.x().value()) && std::isfinite(p.fieldPose.y().value()) &&
                    n < buf.size()) {
                    names[n] = c->name();
                    buf[n++] = p;
                } else if (p.selfAudit.reason != diag::GateReason::None) {
                    const bool routine = p.selfAudit.reason == diag::GateReason::RejectedStaleFix;
                    if (selfAuditSource == nullptr || (selfAuditRoutine && !routine)) {
                        selfAudit = p.selfAudit;
                        selfAuditSource = c->name();
                        selfAuditRoutine = routine;
                    }
                }
            }
        }

        // STEP 4 — fuse: an innovation-bounded, per-tick-clamped nudge on position against the
        // PREDICTED fused pose (so the innovation shrinks as the fused state converges), plus,
        // since E3, a bounded heading INCREMENT from any proposal that supplies an absolute
        // heading.
        const FusionResult fr =
            fusion_.fuse(predicted, std::span<const CorrectionProposal>{buf.data(), n}, units::Time{dt});

        // STEP 5 — heading composed from the IMU as the LAST write + publish.
        //
        // E3: fold this tick's bounded heading INCREMENT into the persistent bias FIRST, then
        // compose the published heading from the freshly-read IMU. That ordering is the whole
        // point — the correction lands in a state that survives the tick, and the IMU reading is
        // still the last thing applied, so no policy can assign a heading. A non-finite nudge is
        // dropped rather than allowed to poison the bias (the estimate must degrade, not die).
        const double nudgeH = fr.headingNudge.value();
        if (fr.headingApplied && std::isfinite(nudgeH)) {
            const double next = headingBias_ + nudgeH;
            if (std::isfinite(next)) {
                headingBias_ = next;
                appliedHeadingNudge_ = nudgeH;
            } else {
                appliedHeadingNudge_ = 0.0;
            }
        } else {
            appliedHeadingNudge_ = 0.0;
        }
        fusedX_ = fr.x.value();
        fusedY_ = fr.y.value();
        const math::Angle fusedHeading = biasedHeading(rawHeading);
        const math::Pose2d newPose{units::Length{fusedX_}, units::Length{fusedY_}, fusedHeading};

        // twist: linear from the fused-pose finite-difference (dt-guarded), omega from the IMU (finite).
        const double rawOmega = imu_.yawRate().value();
        const double omega = std::isfinite(rawOmega) ? rawOmega : 0.0;
        if (hasLast_ && dt > 0.0) {
            const double vx = dtHealthy ? (fusedX_ - lastFusedX_) / dt : 0.0;  // tiny/huge dt ⇒ 0
            const double vy = dtHealthy ? (fusedY_ - lastFusedY_) / dt : 0.0;
            twist_ = math::Twist2d{units::Velocity{vx}, units::Velocity{vy}, units::AngularVelocity{omega}};
        } else {
            // dt <= 0 (or first tick): keep last linear velocity, refresh omega from the sensor.
            twist_ = math::Twist2d{twist_.vx(), twist_.vy(), units::AngularVelocity{omega}};
        }

        // drift accumulator: an applied fix CLEARS it in proportion to the fix's confidence (a strong
        // fix zeroes drift uncertainty; a weak one barely dents it — so quality() can't spring to 1.0
        // on a microscopic-confidence fix). Otherwise it grows by the odom-driven travel.
        if (fr.applied) {
            const double retain = 1.0 - std::clamp(fr.appliedConfidence, 0.0, 1.0);
            distanceSinceCorrection_ = units::Length{distanceSinceCorrection_.value() * retain};
        } else {
            distanceSinceCorrection_ =
                units::Length{distanceSinceCorrection_.value() + std::hypot(odx, ody)};
        }

        deadReckoning_ = !fr.applied;
        // The gate's own account of this tick (E1) travels out on the audit record, so a
        // record producer can stamp the §18.2 gating slots without knowing which fusion
        // policy is installed — the same value flows whether the policy is today's
        // complementary tier or E4's EKF.
        //
        // E2 SUBSTITUTION RULE: the policy's verdict wins whenever it HAS one. It reports
        // `None` only when nothing reached it, and that is exactly the tick whose story lives
        // upstream in a corrector — an off-strip GPS, a fix rejected mid-spin, a re-reported
        // stale sample. Then, and only then, the corrector's own verdict is what the record
        // carries, and `source` names who declined rather than reading "none". A policy that
        // saw proposals always returns Accepted or a Rejected*, so this can never overwrite a
        // real fusion verdict — the `reason == None` guard is what makes that true, and it is
        // dead code with one corrector and load-bearing with two (found by mutation at E2;
        // pinned by test/gps_corrector_blackbox_test.cpp's two-corrector case).
        GateAudit tickAudit = fr.audit;
        const char* source = (fr.applied && n > 0) ? names[0] : "none";
        if (tickAudit.reason == diag::GateReason::None && selfAuditSource != nullptr) {
            tickAudit = selfAudit;
            source = selfAuditSource;
        }
        lastCorrection_ = AppliedCorrection{units::Length{fusedX_ - predicted.x().value()},
                                            units::Length{fusedY_ - predicted.y().value()},
                                            fr.gated || fr.headingGated,
                                            fr.clamped || fr.headingClamped,
                                            source,
                                            tickAudit,
                                            units::AngleDim{appliedHeadingNudge_}};
        refreshQuality(dtHealthy);

        pose_ = newPose;
        lastFusedX_ = fusedX_;
        lastFusedY_ = fusedY_;
        lastOdomX_ = odomNow.x().value();
        lastOdomY_ = odomNow.y().value();
        lastNow_ = now;
        hasLast_ = true;
        everUpdated_ = true;
    }

    // --- IPoseSource ---
    [[nodiscard]] math::Pose2d pose() const noexcept override { return pose_; }
    [[nodiscard]] math::Twist2d twist() const noexcept override { return twist_; }
    [[nodiscard]] double quality() const noexcept override { return quality_; }
    [[nodiscard]] bool isDeadReckoning() const noexcept override { return deadReckoning_; }

    // --- extra observability (telemetry / motion gating) ---
    [[nodiscard]] Quality qualityClass() const noexcept { return qualityClass_; }
    [[nodiscard]] units::Length distanceSinceCorrection() const noexcept { return distanceSinceCorrection_; }
    /// The last tick's applied correction AND the gate's account of why (`audit`, added
    /// at E1) — the values a record producer stamps into the §18.2 gating slots.
    [[nodiscard]] const AppliedCorrection& lastCorrection() const noexcept { return lastCorrection_; }
    /// Forwarding accessor for PilonsOdometry::lastDeltaImplausible() — added at C1
    /// (additive) so the motion loop can feed HealthMonitor's odomImplausible
    /// observable without holding the odometry itself. Raising stays POLICY: this
    /// only EXPOSES the flag; the Localizer still never raises faults (D3 at A3).
    [[nodiscard]] bool lastOdomDeltaImplausible() const noexcept {
        return odom_.lastDeltaImplausible();
    }

    /// The learned heading bias, in radians: how far the published heading sits from the raw IMU
    /// reading (E3). Exposed so a test can prove the correction ACCUMULATES rather than
    /// evaporating each tick — the M2 red team's failure mode — and so telemetry can say how far
    /// the IMU has been found to have drifted. Zero on any tree with no heading-providing
    /// corrector, exactly.
    [[nodiscard]] units::AngleDim headingBias() const noexcept {
        return units::AngleDim{headingBias_};
    }

    /// Teleport the POSITION (x, y); heading stays IMU-owned. Forwards to PilonsOdometry::setPose so
    /// the predictor and the fused belief never diverge, and re-baselines twist + dt so the teleport
    /// injects no phantom velocity next tick.
    ///
    /// E3: the learned heading bias is KEPT, deliberately. A teleport says where the robot IS, not
    /// which way the IMU is wrong; discarding a bias that took a second of tag sightings to learn,
    /// every time a routine re-seeds its position, would throw away the correction at exactly the
    /// moments a routine cares most. `p.heading()` is still ignored, as it always was.
    void setPose(const math::Pose2d& p) {
        odom_.setPose(p);
        fusedX_ = p.x().value();
        fusedY_ = p.y().value();
        pose_ = math::Pose2d{p.x(), p.y(), biasedHeading(imu_.heading())};
        lastFusedX_ = fusedX_;
        lastFusedY_ = fusedY_;
        lastOdomX_ = odom_.pose().x().value();  // re-baseline the odom delta so the jump isn't counted
        lastOdomY_ = odom_.pose().y().value();
        twist_ = math::Twist2d{};
        distanceSinceCorrection_ = units::Length{0.0};
        hasLast_ = false;  // next tick re-establishes dt (no phantom velocity from the jump)
    }

private:
    /// The published heading: the raw IMU reading plus the learned bias (E3, header note). The
    /// zero-bias early-out makes a tree with no heading-providing corrector bit-identical to
    /// pre-E3 by construction rather than by an argument about floating point, and it also keeps
    /// Angle::radians' finiteness precondition off the hot path in the common case.
    [[nodiscard]] math::Angle biasedHeading(math::Angle raw) const noexcept {
        if (headingBias_ == 0.0 || !std::isfinite(headingBias_)) {
            return raw;
        }
        return raw + math::Angle::radians(headingBias_);
    }

    void refreshQuality(bool dtHealthy) {
        const bool ready = imu_.isReady();
        const bool implausible = odom_.lastDeltaImplausible();
        if (!everUpdated_ || !settled_) {
            qualityClass_ = Quality::Uninitialized;  // booting/settling: no live estimate yet
        } else if (!ready) {
            // WAS ready, lost it mid-run (header note): the estimate exists and is decaying —
            // Degraded, not Uninitialized, so a consumer can tell loss from boot.
            qualityClass_ = Quality::Degraded;
        } else if (implausible || !dtHealthy) {
            qualityClass_ = Quality::Degraded;
        } else if (!deadReckoning_) {
            qualityClass_ = Quality::Corrected;
        } else if (distanceSinceCorrection_.value() > config_.driftHorizon.value()) {
            qualityClass_ = Quality::Degraded;
        } else {
            qualityClass_ = Quality::DeadReckon;
        }

        // Scalar, kept CONSISTENT with the class: 0 whenever the IMU is not ready (no heading
        // authority ⇒ no trust — boot AND mid-run loss alike), then driftTerm decayed by dt-health
        // and odom implausibility so it can't read 1.0 on a Degraded tick. (Graded
        // confidence-weighted trust per fix is the M3 EKF's job; M2 resets the drift term on any
        // accepted in-gate fix.)
        if (!everUpdated_ || !ready || !settled_) {
            quality_ = 0.0;
            return;
        }
        const double driftFrac = distanceSinceCorrection_.value() / config_.driftHorizon.value();
        const double driftTerm = std::clamp(1.0 - driftFrac, config_.qFloor, 1.0);
        const double dtTerm = dtHealthy ? 1.0 : 0.5;
        const double implausTerm = implausible ? 0.5 : 1.0;
        quality_ = driftTerm * dtTerm * implausTerm;
    }

    hal::IClock& clock_;
    hal::IImu& imu_;
    PilonsOdometry& odom_;
    IFusionPolicy& fusion_;
    std::span<ICorrector* const> correctors_;
    LocalizerConfig config_;

    math::Pose2d pose_{};
    math::Twist2d twist_{};
    units::Length distanceSinceCorrection_{0.0};
    AppliedCorrection lastCorrection_{};
    Quality qualityClass_ = Quality::Uninitialized;
    double quality_ = 0.0;
    bool deadReckoning_ = true;

    double fusedX_ = 0.0;     // PERSISTENT fused position — advanced by odom deltas + retained nudges
    double fusedY_ = 0.0;
    double lastOdomX_ = 0.0;  // odom pose at the previous tick (to take the per-tick odom delta)
    double lastOdomY_ = 0.0;
    double lastNow_ = 0.0;
    double lastFusedX_ = 0.0;  // fused position at the previous tick (for the twist finite-difference)
    double lastFusedY_ = 0.0;
    bool hasLast_ = false;
    bool everUpdated_ = false;
    bool imuEverReady_ = false;  // the boot guard's memory (header note; never resets mid-run)
    bool sawNotReady_ = false;   // a not-ready phase was witnessed → a boundary exists in the stream
    bool settled_ = false;       // fold open this tick (imuEverReady_ AND past the settle window)
    double settleUntil_ = 0.0;   // clock time the settle window ends (== firstReady if no boundary)

    // E3 — the persistent heading bias (header note). EXACTLY 0.0 on any tree with no
    // heading-providing corrector, which is what the bit-identity early-outs key on.
    double headingBias_ = 0.0;
    double appliedHeadingNudge_ = 0.0;  // this tick's net bias change, for AppliedCorrection::dtheta
};

}  // namespace shulib::localization
