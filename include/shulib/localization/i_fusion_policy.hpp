#pragma once
//
// IFusionPolicy — the swap point that lets a complementary filter ship NOW and a 5-state SE(2) EKF
// drop in LATER behind the same seam (master plan §8: "complementary → EKF"). Given the predicted
// pose and the valid proposals, it returns the corrected position AND, since E3, a bounded heading INCREMENT (FusionResult's headingNudge/headingApplied/headingGated/headingClamped, which localizer.hpp folds into a persistent heading bias). The pre-E3 wording said "POSITION only" and was left behind by that chunk — heading is re-stamped from
// the IMU by the Localizer afterward, so a policy can never own heading at M2. ComplementaryFusion
// (the gated nudge) implements this now; EkfFusion (Kalman update + Mahalanobis gating) drops in at
// M3+ without changing ICorrector, the Localizer API, or a single caller.

#include <span>

#include "shulib/localization/correction.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::localization {

/// The seam a fusion MECHANISM plugs into: how hard to move the estimate toward the absolute
/// fixes that arrived this tick. TWO tiers ship behind it: ComplementaryFusion (the gated,
/// rate-limited nudge, still the default) and EkfFusion (a 5-state SE(2) Kalman update with
/// Mahalanobis gating). Picking one is a single constructor argument to the Localizer, with
/// ICorrector, the Localizer API and every caller unchanged across the swap — which is the only
/// reason this is an interface and not a function.
/// An implementation MAY be stateful and may equally be pure, so callers must assume the worse
/// of the two: EkfFusion carries the filter state (x, P) across ticks, while ComplementaryFusion
/// holds nothing but its config and that tier's cross-tick memory lives in the Localizer instead
/// (fusedX_/fusedY_ and the heading bias). So fuse() is not guaranteed to be a function of its
/// arguments alone, and one policy object must not be shared between two Localizers. The
/// Localizer calls fuse() exactly once per update(), UNCONDITIONALLY — during the boot/settle
/// window it is still called, with an empty span of proposals.
class IFusionPolicy {
public:
    /// Virtual so an owner holding a policy polymorphically can destroy it — the Localizer is not
    /// that owner: it takes `IFusionPolicy&` and never deletes it, so the policy must outlive the
    /// Localizer that was handed it. The copy/move members are re-defaulted (a user-declared
    /// destructor suppresses the implicit MOVEs and deprecates the implicit copies) only so this
    /// base imposes no policy of its own. Copying a real one is tier-dependent and rarely what
    /// you want: ComplementaryFusion is pure config, but an EkfFusion copy clones a live BELIEF
    /// (state and covariance), which then ages independently of the original.
    virtual ~IFusionPolicy() = default;
    IFusionPolicy() = default;
    IFusionPolicy(const IFusionPolicy&) = default;
    IFusionPolicy(IFusionPolicy&&) = default;
    IFusionPolicy& operator=(const IFusionPolicy&) = default;
    IFusionPolicy& operator=(IFusionPolicy&&) = default;

    /// Fold the valid proposals into the predicted position and return the corrected (x, y),
    /// the bounded heading INCREMENT added at E3 (headingNudge and its three flags, which the
    /// Localizer folds into a persistent heading bias), and the audit flags. `valid` holds only already-screened proposals (valid && confidence>0 &&
    /// positionStdDev>0 && finite). `dt` is the tick duration (for a rate-based per-tick clamp).
    [[nodiscard]] virtual FusionResult fuse(const math::Pose2d& predicted,
                                            std::span<const CorrectionProposal> valid,
                                            units::Time dt) = 0;
};

}  // namespace shulib::localization
