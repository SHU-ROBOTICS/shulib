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
    bool providesHeading = false;        ///< RESERVED. M2 ignores it entirely (heading is IMU-owned).
                                         ///< The M3 AprilTag-yaw path is an ADDITIVE extension — a
                                         ///< headingNudge field on FusionResult + the Localizer applying
                                         ///< it before the IMU re-stamp — which does NOT change the frozen
                                         ///< IPoseSource/ICorrector/IFusionPolicy signatures callers depend on.
};

/// What a fusion policy did to the POSITION this tick. Heading is never here — the Localizer
/// re-stamps it from the IMU as the final write, so no policy can own heading at M2.
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
};

}  // namespace shulib::localization
