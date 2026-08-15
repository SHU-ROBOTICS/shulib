#pragma once
//
// ComplementaryFusion — the M2 fusion policy (master plan §8 "complementary → EKF"; decision #4
// "innovation-bounded gated nudge, never snap"). It folds absolute proposals into the predicted
// POSITION as a bounded incremental pull, NEVER an assignment to the measured pose, so it is
// structurally incapable of snapping. An EkfFusion drops in behind the same IFusionPolicy at M3.
//
// Per valid proposal, given the odom-predicted position:
//   innovation = measured.position − predicted.position
//   • REJECT it if |innovation| > innovationGate  → a wild GPS/tag fix can never yank the pose.
//   • else nudge = (maxGain · confidence) · innovation, then CLAMP |nudge| ≤ maxNudgeRate · dt
//     (a RATE-based per-tick budget, so the never-snap bound is invariant to loop rate).
// Proposals sum, and the SUM is clamped once more to the same budget, so N correctors can never
// out-vote the per-tick limit. Empty proposals → the position is returned unchanged (dead-reckon).
//
// `confidence` (∈[0,1]) is the complementary-tier gain; `positionStdDev` is carried on the proposal
// for the M3 EKF's measurement noise R and is unused here.
//
// ── HEADING, ADDED AT E3 — AND WHY THIS STILL CANNOT SNAP ──────────────────────────────────
// Until E3 this policy could not touch heading at all, because there was no absolute heading in
// the tree to touch. `AprilTagCorrector` produces one, marks it with
// `CorrectionProposal::providesHeading`, and this policy folds it by exactly the same recipe it
// has always used for position:
//   headingInnovation = predicted.heading().errorTo(measured.heading())   (shortest signed)
//   • REJECT it if |innovation| > headingGate  → a mirrored tag or a wrong map entry can never
//     yank the robot's idea of which way it faces.
//   • else nudge = (maxHeadingGain · confidence) · innovation, CLAMPED to maxHeadingNudgeRate·dt.
// Nudges sum and the sum is clamped once more, so N sources cannot out-vote the budget.
//
// THE STRUCTURAL POINT: what leaves here is an INCREMENT (`FusionResult::headingNudge`), never an
// absolute heading. A policy that could return an absolute heading could snap; a policy that can
// only return a bounded increment cannot, whatever a corrector claims. The Localizer folds the
// increment into a persistent bias and composes the published heading from the IMU as the last
// write of the tick, so the IMU stays the sole source of heading CHANGE (decision #4) and this
// policy can only ever move a slow bias. The two regimes are both live and both tested: for
// innovations under about a degree the GAIN binds (so the bias settles smoothly instead of
// chattering), and above that the RATE CLAMP binds (so a large innovation can never arrive fast).
//
// Position and heading are gated INDEPENDENTLY. A fix may pass one and fail the other — they are
// different measurements with different failure modes — so `applied`/`gated` continue to describe
// POSITION exactly as they always did, and `headingApplied`/`headingGated` describe heading.
// `GateAudit::reason` likewise stays position-primary; a heading rejection is read off the record
// as a large `gateResidualHeading` with a zero `correctionDTheta`.

#include <algorithm>
#include <cmath>
#include <span>

#include "shulib/core/check.hpp"
#include "shulib/localization/correction.hpp"
#include "shulib/localization/i_fusion_policy.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::localization {

/// The six numbers that bound how hard an absolute fix may pull the estimate. Position and
/// heading each get three of the same kind: a GATE (reject an innovation larger than this
/// outright), a GAIN (the fraction of a surviving innovation taken per tick at confidence 1),
/// and a per-tick budget expressed as a RATE, so the never-snap bound does not move when the
/// loop rate does. Position and heading are configured separately because they are different
/// measurements with different failure modes — a mirrored tag ruins the heading while leaving
/// the position plausible. Every default is a conservative PROVISIONAL placeholder awaiting
/// M3 tuning; the ctor rejects an out-of-range one loudly rather than clamping it.
struct ComplementaryFusionConfig {
    /// Per-tick nudge budget as a RATE: the max position correction applied in a tick is
    /// `maxNudgeRate · dt`. Loop-rate-independent. (M3-tuned; conservative placeholder.)
    units::Velocity maxNudgeRate{12.0};
    /// Reject a proposal whose |innovation| exceeds this — the never-snap gate. (M3-tuned.)
    units::Length innovationGate{12.0};
    /// Fraction of the innovation pulled per tick at confidence == 1, in (0,1]. (M3-tuned.)
    double maxGain = 0.15;

    // ── heading (E3) ───────────────────────────────────────────────────────────────────────
    /// Reject a heading proposal whose |innovation| exceeds this — the never-snap gate for yaw.
    /// 15 degrees is ~15x the heading drift a 60-second match is expected to accumulate (the
    /// master plan's ~1 deg/min IMU figure, HA-20), so an innovation this large is far more
    /// likely to be a mirrored tag winding, a wrong tag-map entry or a misidentified id than
    /// real drift — and folding it would be worse than folding nothing.
    /// PROVISIONAL (A4: HA-80).
    units::AngleDim headingGate{15.0 * math::Angle::kPi / 180.0};
    /// Fraction of the heading innovation pulled per tick at confidence == 1, in (0,1]. The
    /// regulator near convergence. PROVISIONAL (A4: HA-81).
    double maxHeadingGain = 0.15;
    /// Per-tick heading budget as a RATE: at most `maxHeadingNudgeRate · dt` of bias change in
    /// one tick, loop-rate-independent, exactly as maxNudgeRate bounds position. This is the
    /// never-snap bound for yaw — the number that makes "a yaw reset can never happen" a
    /// property of the code rather than a promise. 10 deg/s. PROVISIONAL (A4: HA-82).
    units::AngularVelocity maxHeadingNudgeRate{10.0 * math::Angle::kPi / 180.0};
};

/// The M2 fusion policy: a gated, rate-limited NUDGE toward absolute fixes, never a snap.
/// It is structurally incapable of snapping, and that is the point rather than a tuning
/// achievement — position moves by at most `maxNudgeRate · dt` in a tick, and heading leaves
/// as a bounded INCREMENT instead of an absolute value, so no corrector can reset the estimate
/// however confident it claims to be. Proposals sum and the sum is clamped again, so N
/// correctors cannot out-vote one tick's budget either. Holds no state between calls: the same
/// prediction, proposals and dt always give the same answer. EkfFusion replaces it behind
/// IFusionPolicy at M3 without touching a caller.
class ComplementaryFusion final : public IFusionPolicy {
public:
    /// Copies `config`; nothing is referenced after construction, so the argument may be a
    /// temporary. Each field is a LOUD precondition rather than a silent clamp: rates ≥ 0,
    /// gates > 0, gains in (0, 1]. A zero gain is excluded on purpose — it is a policy that
    /// accepts every fix and then corrects by nothing, which looks like working fusion in every
    /// audit flag while the estimate dead-reckons.
    explicit ComplementaryFusion(const ComplementaryFusionConfig& config = {}) : config_{config} {
        SHULIB_PRECONDITION(config.maxNudgeRate.value() >= 0.0,
                            "ComplementaryFusion: maxNudgeRate must be >= 0");
        SHULIB_PRECONDITION(config.innovationGate.value() > 0.0,
                            "ComplementaryFusion: innovationGate must be > 0");
        SHULIB_PRECONDITION(config.maxGain > 0.0 && config.maxGain <= 1.0,
                            "ComplementaryFusion: maxGain must be in (0, 1]");
        SHULIB_PRECONDITION(config.headingGate.value() > 0.0,
                            "ComplementaryFusion: headingGate must be > 0");
        SHULIB_PRECONDITION(config.maxHeadingGain > 0.0 && config.maxHeadingGain <= 1.0,
                            "ComplementaryFusion: maxHeadingGain must be in (0, 1]");
        SHULIB_PRECONDITION(config.maxHeadingNudgeRate.value() >= 0.0,
                            "ComplementaryFusion: maxHeadingNudgeRate must be >= 0");
    }

    /// Fold `valid` into `predicted` and return the corrected absolute POSITION together with a
    /// bounded heading INCREMENT — never an absolute heading, which is what makes snapping
    /// impossible rather than merely unlikely.
    ///
    /// Position and heading are gated INDEPENDENTLY, so a fix may pass one and fail the other.
    /// Position: reject |measured − predicted| > innovationGate, else pull maxGain·confidence of
    /// it, clamped per proposal and once more on the sum to maxNudgeRate·dt. Heading: the same
    /// recipe over `predicted.heading().errorTo(measured)` (shortest signed, so the ±π seam
    /// costs nothing), but ONLY for proposals with `providesHeading` — everything else carries a
    /// pass-through of the prediction whose innovation is zero by construction. A non-finite
    /// innovation or confidence is rejected exactly like an out-of-gate one, and a confidence
    /// outside [0, 1] is clamped, so a corrector cannot amplify its own gain.
    ///
    /// Empty `valid` returns the predicted position unchanged (dead-reckoning). dt == 0 makes
    /// both per-tick budgets zero: the position comes back unchanged and `applied` /
    /// `headingApplied` are false, but the audit still reports the GATE's verdict rather than
    /// pretending no proposal arrived. `positionStdDev` is not read here — it is carried for the
    /// M3 EKF's measurement noise. Holds no state, so this is safe to call out of order.
    [[nodiscard]] FusionResult fuse(const math::Pose2d& predicted,
                                    std::span<const CorrectionProposal> valid,
                                    units::Time dt) override {
        const double px = predicted.x().value();
        const double py = predicted.y().value();
        const double maxNudge = config_.maxNudgeRate.value() * dt.value();  // inches this tick
        const double gate = config_.innovationGate.value();

        double sumX = 0.0;
        double sumY = 0.0;
        double maxConf = 0.0;  // strongest accepted fix → how much to trust the correction
        bool accepted = false;
        bool gated = false;
        bool clamped = false;
        // E1 introspection (GateAudit): the innovation this tick's verdict was rendered
        // ON — the strongest ACCEPTED proposal's if one passed, else the FIRST rejected
        // one's (the fix that actually triggered the rejection). Recorded whether or not
        // the nudge moved anything, because the audit is about the GATE's decision.
        double auditInnoX = 0.0;
        double auditInnoY = 0.0;
        bool haveAudit = false;

        // E3 heading state — kept in its own set of variables, and gated independently of
        // position, because they are different measurements with different failure modes.
        const double maxHeadingNudge = config_.maxHeadingNudgeRate.value() * dt.value();
        const double headingGate = config_.headingGate.value();
        double headingSum = 0.0;
        double maxHeadingConf = 0.0;
        double auditInnoHeading = 0.0;
        bool headingAccepted = false;
        bool headingGated = false;
        bool headingClamped = false;
        bool haveHeadingAudit = false;

        for (const CorrectionProposal& p : valid) {
            const double innoX = p.fieldPose.x().value() - px;
            const double innoY = p.fieldPose.y().value() - py;
            const double innoMag = std::hypot(innoX, innoY);
            // Reject a wild fix, OR any non-finite proposal field (innovation OR confidence) — a NaN/Inf
            // confidence would otherwise sail through (Inf > 0) and poison the nudge with NaN.
            if (!std::isfinite(innoMag) || !std::isfinite(p.confidence) || innoMag > gate) {
                if (!gated && !accepted) {  // the first rejection, and nothing has passed yet
                    auditInnoX = innoX;
                    auditInnoY = innoY;
                    haveAudit = true;
                }
                gated = true;
                continue;
            }
            if (!accepted || p.confidence > maxConf) {  // the strongest accepted fix wins the audit
                auditInnoX = innoX;
                auditInnoY = innoY;
                haveAudit = true;
            }
            accepted = true;  // this proposal passed the gate (a fix was incorporated)
            const double conf = std::clamp(p.confidence, 0.0, 1.0);  // a corrector can't amplify the gain
            maxConf = std::max(maxConf, conf);
            const double weight = config_.maxGain * conf;
            double nudgeX = weight * innoX;
            double nudgeY = weight * innoY;
            const double nudgeMag = std::hypot(nudgeX, nudgeY);
            if (nudgeMag > maxNudge) {                            // clamp this proposal to the budget
                const double scale = maxNudge / nudgeMag;
                nudgeX *= scale;
                nudgeY *= scale;
                clamped = true;
            }
            sumX += nudgeX;
            sumY += nudgeY;
        }

        // ── HEADING (E3), gated independently of position. Only proposals that CLAIM an
        // absolute heading are considered; everything else carries a pass-through of the
        // prediction, whose innovation would be exactly zero and whose inclusion would
        // therefore be a silent no-op that looked like a decision. ────────────────────────
        for (const CorrectionProposal& p : valid) {
            if (!p.providesHeading) {
                continue;
            }
            // Shortest signed rotation from the prediction to the measurement — math::Angle
            // owns the ±180° seam, so a fix at +179° against a prediction at -179° is 2° away
            // and not 358°. Getting this wrong would make the estimator spin the long way round
            // once per revolution.
            const double innoH = predicted.heading().errorTo(p.fieldPose.heading());
            if (!std::isfinite(innoH) || !std::isfinite(p.confidence) ||
                std::abs(innoH) > headingGate) {
                if (!headingGated && !headingAccepted) {  // the first rejection, none passed yet
                    auditInnoHeading = innoH;
                    haveHeadingAudit = true;
                }
                headingGated = true;
                continue;
            }
            const double conf = std::clamp(p.confidence, 0.0, 1.0);
            if (!headingAccepted || conf > maxHeadingConf) {  // strongest accepted wins the audit
                auditInnoHeading = innoH;
                haveHeadingAudit = true;
            }
            headingAccepted = true;
            maxHeadingConf = std::max(maxHeadingConf, conf);
            double nudgeH = config_.maxHeadingGain * conf * innoH;
            if (std::abs(nudgeH) > maxHeadingNudge) {
                nudgeH = std::copysign(maxHeadingNudge, nudgeH);
                headingClamped = true;
            }
            headingSum += nudgeH;
        }
        if (std::abs(headingSum) > maxHeadingNudge) {  // N sources can't out-vote the budget
            headingSum = std::copysign(maxHeadingNudge, headingSum);
            headingClamped = true;
        }
        const bool headingApplied = headingAccepted && maxHeadingNudge > 0.0;
        if (!headingApplied) {
            headingSum = 0.0;  // a dt==0 stall allows no motion, in heading as in position
        }

        const double sumMag = std::hypot(sumX, sumY);
        if (sumMag > maxNudge) {  // N proposals can't out-vote the per-tick budget
            const double scale = maxNudge / sumMag;
            sumX *= scale;
            sumY *= scale;
            clamped = true;
        }

        // "applied" means a fix was actually incorporated this tick: accepted by the gate AND the
        // per-tick budget allowed motion (maxNudge == 0 on a dt==0 stall ⇒ nothing could be applied).
        const bool applied = accepted && maxNudge > 0.0;

        // The E1 audit. `reason` reports the GATE's verdict (that is what the §18.2
        // field audits), so a proposal that passed the gate on a dt==0 tick — where the
        // per-tick budget allowed no motion at all — still reads Accepted with a zero
        // applied correction, rather than pretending no proposal arrived.
        // `covarianceTrace` carries the complementary tier's scalar TRUST WEIGHT, which
        // is exactly what debug_record.hpp reserves that slot for until E4's EKF exists;
        // `mahalanobis` stays 0 because this tier has no covariance to normalise by, and
        // a fabricated distance would be worse than an absent one.
        GateAudit audit;
        if (haveAudit) {
            audit.residualX = units::Length{auditInnoX};
            audit.residualY = units::Length{auditInnoY};
        }
        // E3: the heading innovation the heading verdict was rendered on, recorded whether it
        // was accepted or gated. This is the §18.2 `gateResidualHeading` slot, declared at A1
        // and empty until now. `reason` stays POSITION-primary (header note): a heading-only
        // rejection is read off the record as a large residualHeading beside a zero
        // correctionDTheta, which is unambiguous and needs no thirteenth enum value.
        if (haveHeadingAudit) {
            audit.residualHeading = units::AngleDim{auditInnoHeading};
        }
        if (accepted) {
            audit.reason = diag::GateReason::Accepted;
            audit.covarianceTrace = maxConf;
        } else if (gated) {
            audit.reason = diag::GateReason::RejectedInnovation;
        }
        FusionResult result{units::Length{px + sumX}, units::Length{py + sumY},
                            applied,                 gated,
                            clamped,                 applied ? maxConf : 0.0,
                            audit};
        result.headingNudge = units::AngleDim{headingSum};
        result.headingApplied = headingApplied;
        result.headingGated = headingGated;
        result.headingClamped = headingClamped;
        return result;
    }

private:
    ComplementaryFusionConfig config_;
};

}  // namespace shulib::localization
