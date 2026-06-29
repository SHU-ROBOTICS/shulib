#pragma once
//
// correction.hpp — the value types the localization fusion seam exchanges (master plan §8; WS5).
// These are the mechanism-agnostic currency that lets a complementary filter today and a 5-state
// SE(2) EKF later share ONE seam: a corrector PROPOSES an absolute fix, a fusion policy decides
// how hard to move toward it, and the Localizer records what was applied for telemetry.

#include "shulib/math/pose2d.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::localization {

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
};

}  // namespace shulib::localization
