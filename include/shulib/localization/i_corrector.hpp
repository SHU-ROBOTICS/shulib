#pragma once
//
// ICorrector — the WRITE seam: one source of ABSOLUTE position fixes (V5 GPS, AprilTag PnP, LIDAR
// scan-match) (master plan §6/§8). It is PULL, not push: the Localizer calls propose() each tick
// with the odom-PREDICTED pose and the tick dt, and the corrector returns a CorrectionProposal
// (or {valid=false}). The corrector owns ALL of its mess — HAL access, frame/lever-arm/PnP
// reduction, latency, staleness, gating — so the Localizer stays geometry-free and the trust math
// stays in one place. Pure w.r.t. its injected HAL handle; host-testable with a fake.
//
// At M2 the registered list is empty or holds only NullCorrector (below); GpsCorrector /
// AprilTagCorrector are concrete at M3 behind THIS exact signature, with no caller change.

#include "shulib/localization/correction.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::localization {

/// One source of ABSOLUTE field-pose fixes — V5 GPS, AprilTag PnP, LIDAR scan-match. PULL, not
/// push: the Localizer calls propose() once per tick with its odom-predicted pose, and nothing
/// here ever writes into the estimator. An implementation owns ALL of its own mess — HAL access,
/// frame/lever-arm/PnP reduction, latency, staleness, gating — so the Localizer stays
/// geometry-free and the trust math stays in one place. Pure with respect to its injected HAL
/// handle, which is what makes a corrector host-testable against a fake.
class ICorrector {
public:
    /// Polymorphic-base boilerplate: the destructor is virtual so a concrete corrector held as
    /// `ICorrector&`/`ICorrector*` destroys correctly, and DECLARING it is what suppresses the
    /// implicit copy/move, which are re-defaulted below. The base carries no state of its own.
    /// Ownership stays with the CALLER either way: the Localizer takes a NON-OWNING
    /// `span<ICorrector* const>` (at most kMaxCorrectors, each checked non-null at construction),
    /// so every corrector must outlive the Localizer it was handed to.
    virtual ~ICorrector() = default;
    ICorrector() = default;
    ICorrector(const ICorrector&) = default;
    ICorrector(ICorrector&&) = default;
    ICorrector& operator=(const ICorrector&) = default;
    ICorrector& operator=(ICorrector&&) = default;

    /// Propose an absolute fix given the odom-predicted pose this tick. MUST be non-throwing and
    /// MUST return {valid=false} when it has no usable fix (off-strip GPS, no tag) — never a
    /// zero-confidence pull. `dt` is the tick duration (seconds).
    [[nodiscard]] virtual CorrectionProposal propose(const math::Pose2d& predicted,
                                                     units::Time dt) = 0;

    /// Stable id for telemetry / per-source dead-reckon accounting.
    [[nodiscard]] virtual const char* name() const noexcept = 0;
};

/// The M2 placeholder: a registered source that never has a fix. Lets the fusion pipeline run and
/// be tested end-to-end (it just always dead-reckons) before any real corrector exists, and keeps
/// the seam visibly wired for telemetry. M3 replaces it with GpsCorrector/AprilTagCorrector.
class NullCorrector final : public ICorrector {
public:
    /// Always declines — a default-constructed proposal, so `valid == false` and
    /// `selfAudit.reason == None`. Both arguments are ignored, and the estimator dead-reckons
    /// this tick exactly as it would with no corrector registered at all.
    [[nodiscard]] CorrectionProposal propose(const math::Pose2d& /*predicted*/,
                                             units::Time /*dt*/) override {
        return CorrectionProposal{};  // valid == false
    }
    /// `"null"`. Because this corrector never proposes and never self-audits, the Localizer
    /// never reads it — the id exists so the seam is visibly wired, not to label a record.
    [[nodiscard]] const char* name() const noexcept override { return "null"; }
};

}  // namespace shulib::localization
