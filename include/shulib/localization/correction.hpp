#pragma once
//
// correction.hpp — the value types the localization fusion seam exchanges (master plan §8; WS5).
// These are the mechanism-agnostic currency that lets a complementary filter today and a 5-state
// SE(2) EKF later share ONE seam: a corrector PROPOSES an absolute fix, a fusion policy decides
// how hard to move toward it, and the Localizer records what was applied for telemetry.

#include "shulib/diag/debug_record.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::localization {

/// WHY the gate decided what it decided, as data (WS13/E1's estimator introspection).
///
/// The §18.2 record already has slots for these quantities — `gateResidualX/Y/Heading`,
/// `gateMahalanobis`, `gateReason`, `covarianceTrace` — declared at A1 and unpopulated
/// by design until a real gate exists. This struct is the CARRIER that lets a fusion
/// policy fill them: it rides out on FusionResult, the Localizer keeps it on
/// AppliedCorrection, and the record producer stamps it. Nothing about the frozen
/// IPoseSource / ICorrector / IFusionPolicy signatures changes — the seam was built
/// EKF-ready at M2 and stays exactly as shaped.
///
/// Why it matters: every tick after E2 makes a DECISION about whether to trust a
/// sensor fix, and those decisions are where fusion goes wrong. They are invisible
/// unless something writes them down, and the < 1° accuracy claim is certified by
/// exactly these numbers — residual, Mahalanobis distance, accept/reject reason —
/// rather than asserted.
///
/// HONEST SCOPE AT E1: the complementary tier fills `reason` (None / Accepted /
/// RejectedInnovation), the residual of the fix it acted on, and `covarianceTrace` as
/// the tier's scalar TRUST WEIGHT (which is what debug_record.hpp's own note reserves
/// that slot for until an EKF exists). `mahalanobis` stays 0 until E4 — a complementary
/// filter has no covariance to normalise by, and a fabricated distance would be worse
/// than an absent one. RejectedNoFix / RejectedHighYawRate are CORRECTOR-side verdicts
/// that E2 fills in.
struct GateAudit {
    units::Length residualX{};          ///< innovation (measured − predicted), field x
    units::Length residualY{};          ///< innovation, field y
    units::AngleDim residualHeading{};  ///< innovation, heading (radians) — E3 fills it
    double mahalanobis = 0.0;           ///< Mahalanobis distance of the fix — E4 fills it
    double covarianceTrace = 0.0;       ///< EKF trace (E4), or the tier's trust weight today
    diag::GateReason reason = diag::GateReason::None;  ///< why accepted/rejected
};

/// What ONE corrector offers this tick. An ABSOLUTE field pose + how much to trust it — never a
/// delta and never a "set". `valid == false` means "I have nothing usable this tick" (off-strip
/// GPS, no tag) and the proposal is ignored entirely (NOT a zero-confidence pull toward (0,0)).
struct CorrectionProposal {
    bool valid = false;                  ///< false ⇒ skip entirely (dead-reckon w.r.t. this source)
    math::Pose2d fieldPose{};            ///< absolute field pose the source believes the robot is at
    double confidence = 0.0;             ///< [0,1] peak trust; 0 ⇒ no pull even if valid
    units::Length positionStdDev{};      ///< 1σ position noise (R for an EKF / nudge weight); > 0 when valid
    /// LIVE SINCE E3 (was RESERVED at M2). `true` means `fieldPose.heading()` is an ABSOLUTE
    /// measured heading and the fusion policy may nudge toward it; `false` means the heading
    /// field is a pass-through of the prediction and carries no information (E2's GpsCorrector
    /// sets it false and passes the PREDICTED heading, deliberately, so that even a policy that
    /// read it would read the estimator's own answer).
    ///
    /// This is the additive path M2 reserved, taken exactly as written: a `headingNudge` on
    /// FusionResult which the Localizer folds into a persistent heading BIAS before composing
    /// the final heading from the IMU. The frozen IPoseSource / ICorrector / IFusionPolicy
    /// signatures did not move, and no existing construction of this struct changed meaning.
    bool providesHeading = false;
    /// The corrector's OWN account of this tick — APPENDED at E2, trailing and defaulted, so
    /// every existing construction of this struct still compiles and means the same thing
    /// (the same discipline E1 used to add `GateAudit` to `FusionResult`).
    ///
    /// WHY IT EXISTS. A corrector that returns `valid == false` is dropped by the Localizer
    /// and never reaches a fusion policy, so before E2 a corrector-side verdict had NO channel
    /// to the record: an off-strip GPS and an empty corrector list produced the same
    /// `GateReason::None`, and "the estimator is dead-reckoning because the strip is missing"
    /// was indistinguishable from "nobody asked". Driving Skills has no GPS strip, which makes
    /// that the difference between a diagnosable run and a mystery. `RejectedNoFix` and
    /// `RejectedHighYawRate` were reserved at A1 as corrector-side verdicts; this is the wire
    /// that carries them.
    ///
    /// CONTRACT. Set `selfAudit.reason` on every tick the corrector declines to propose, and
    /// leave it `None` when it does propose — the fusion policy owns the audit for proposals
    /// that reach it, and a corrector claiming `Accepted` here could otherwise be substituted
    /// into the record on a tick where the Localizer screened the proposal out and nothing was
    /// applied. The Localizer substitutes this audit ONLY when the policy returned no verdict
    /// of its own (see localizer.hpp, STEP 4).
    GateAudit selfAudit{};
};

/// What a fusion policy did this tick.
///
/// x/y are an ABSOLUTE fused position: predicted + a bounded nudge. `headingNudge` is NOT — it is
/// a bounded INCREMENT, and the difference is the whole safety argument. A policy that returned an
/// absolute heading could snap; a policy that can only return an increment cannot, no matter what
/// a corrector proposes or how confident it claims to be. The Localizer accumulates the increment
/// into a persistent heading bias and composes the published heading from the IMU as the final
/// write of the tick, so the IMU remains the sole source of heading CHANGE and the corrector can
/// only ever learn a slowly-moving BIAS (localizer.hpp, STEP 5).
struct FusionResult {
    units::Length x{};                   ///< fused field x (predicted + bounded nudge)
    units::Length y{};                   ///< fused field y
    bool applied = false;                ///< ≥1 proposal passed the gate and was incorporated
    bool gated = false;                  ///< a proposal was rejected by the innovation bound
    bool clamped = false;                ///< the per-tick budget bound the applied nudge
    double appliedConfidence = 0.0;      ///< [0,1] confidence of the strongest applied fix (0 if none);
                                         ///< drives how much the drift accumulator is cleared.
    GateAudit audit{};                   ///< WHY this tick decided as it did (E1) — APPENDED, so every
                                         ///< existing positional construction of this struct still
                                         ///< compiles and means the same thing.
    /// The bounded heading INCREMENT to fold into the estimator's heading bias this tick, in
    /// radians. APPENDED at E3, trailing and defaulted, on the same discipline E1 and E2 used:
    /// every existing construction of this struct still compiles and still means exactly what it
    /// meant, because a policy that does not set these leaves heading untouched.
    units::AngleDim headingNudge{};
    bool headingApplied = false;         ///< a proposal supplying an absolute heading was folded
    bool headingGated = false;            ///< a heading proposal was rejected by the heading bound
    bool headingClamped = false;          ///< the per-tick heading budget bound the nudge
};

/// The per-tick audit record the Localizer exposes via lastCorrection() — maps onto the §18.2
/// DebugRecord "applied-correction (dx,dy) + clamped + gating reason" so the never-snap guarantee
/// is observable in telemetry. dx/dy are the NET position change applied this tick.
struct AppliedCorrection {
    units::Length dx{};
    units::Length dy{};
    bool gated = false;                  ///< any proposal rejected as too far (innovation gate)
    bool clamped = false;                ///< the per-tick nudge budget was hit
    const char* source = "none";         ///< name() of the corrector applied, or "none"
    GateAudit audit{};                   ///< the gate's own account of this tick (E1) — this is the
                                         ///< value the record producer stamps into the §18.2 slots
    /// The NET heading change applied this tick, in radians — the §18.2 `correctionDTheta` slot,
    /// declared at A1 as "heading nudge (0 at M2: heading is IMU-owned) — E3" and filled here.
    /// APPENDED, trailing and defaulted, so every existing construction still compiles.
    /// This is what audits never-snap for HEADING the way dx/dy audit it for position.
    units::AngleDim dtheta{};
};

}  // namespace shulib::localization
