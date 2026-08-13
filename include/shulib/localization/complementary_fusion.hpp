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
// for the M3 EKF's measurement noise R and is unused here. Heading is NEVER touched — the Localizer
// re-stamps it from the IMU after fusion, so this policy can only move x/y.

#include <algorithm>
#include <cmath>
#include <span>

#include "shulib/core/check.hpp"
#include "shulib/localization/correction.hpp"
#include "shulib/localization/i_fusion_policy.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::localization {

struct ComplementaryFusionConfig {
    /// Per-tick nudge budget as a RATE: the max position correction applied in a tick is
    /// `maxNudgeRate · dt`. Loop-rate-independent. (M3-tuned; conservative placeholder.)
    units::Velocity maxNudgeRate{12.0};
    /// Reject a proposal whose |innovation| exceeds this — the never-snap gate. (M3-tuned.)
    units::Length innovationGate{12.0};
    /// Fraction of the innovation pulled per tick at confidence == 1, in (0,1]. (M3-tuned.)
    double maxGain = 0.15;
};

class ComplementaryFusion final : public IFusionPolicy {
public:
    explicit ComplementaryFusion(const ComplementaryFusionConfig& config = {}) : config_{config} {
        SHULIB_PRECONDITION(config.maxNudgeRate.value() >= 0.0,
                            "ComplementaryFusion: maxNudgeRate must be >= 0");
        SHULIB_PRECONDITION(config.innovationGate.value() > 0.0,
                            "ComplementaryFusion: innovationGate must be > 0");
        SHULIB_PRECONDITION(config.maxGain > 0.0 && config.maxGain <= 1.0,
                            "ComplementaryFusion: maxGain must be in (0, 1]");
    }

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
        if (accepted) {
            audit.reason = diag::GateReason::Accepted;
            audit.covarianceTrace = maxConf;
        } else if (gated) {
            audit.reason = diag::GateReason::RejectedInnovation;
        }
        return FusionResult{units::Length{px + sumX}, units::Length{py + sumY},
                            applied, gated, clamped, applied ? maxConf : 0.0, audit};
    }

private:
    ComplementaryFusionConfig config_;
};

}  // namespace shulib::localization
