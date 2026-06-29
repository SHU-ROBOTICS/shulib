#pragma once
//
// IFusionPolicy — the swap point that lets a complementary filter ship NOW and a 5-state SE(2) EKF
// drop in LATER behind the same seam (master plan §8: "complementary → EKF"). Given the predicted
// pose and the valid proposals, it returns the corrected POSITION only — heading is re-stamped from
// the IMU by the Localizer afterward, so a policy can never own heading at M2. ComplementaryFusion
// (the gated nudge) implements this now; EkfFusion (Kalman update + Mahalanobis gating) drops in at
// M3+ without changing ICorrector, the Localizer API, or a single caller.

#include <span>

#include "shulib/localization/correction.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::localization {

class IFusionPolicy {
public:
    virtual ~IFusionPolicy() = default;
    IFusionPolicy() = default;
    IFusionPolicy(const IFusionPolicy&) = default;
    IFusionPolicy(IFusionPolicy&&) = default;
    IFusionPolicy& operator=(const IFusionPolicy&) = default;
    IFusionPolicy& operator=(IFusionPolicy&&) = default;

    /// Fold the valid proposals into the predicted position and return the corrected (x, y) plus
    /// the audit flags. `valid` holds only already-screened proposals (valid && confidence>0 &&
    /// positionStdDev>0 && finite). `dt` is the tick duration (for a rate-based per-tick clamp).
    [[nodiscard]] virtual FusionResult fuse(const math::Pose2d& predicted,
                                            std::span<const CorrectionProposal> valid,
                                            units::Time dt) = 0;
};

}  // namespace shulib::localization
