#pragma once
//
// Finite-value invariant guards (master plan §18.4) — the LOG-AND-RECOVER counterpart to
// SHULIB_PRECONDITION's throw. A precondition guards a CALLER contract (a violation is a
// bug; host tests must go red). These guards protect the RUNTIME DATA PATH: a NaN/Inf
// that appears mid-run (sensor pathology, a division that went bad) must be caught,
// logged as a fault, and replaced with a safe fallback — never propagated into the pose
// estimate, and never allowed to crash or abort the auton.
//
// The one absolute guarantee, pinned by test/finite_guard_test.cpp:
//
//     recoverFinite* ALWAYS returns a finite value — unconditionally.
//
// Even a non-finite FALLBACK (a broken caller) degrades to zero rather than letting a
// NaN through: the whole point is that a non-finite value cannot pass this line, so the
// guard cannot have a bypass path, not even a caller-error one. (In correct use the
// fallback is the last-known-good value, which is finite by induction.)
//
// Heading note: math::Angle cannot HOLD a non-finite value (its factories reject them),
// so a Pose2d's finiteness reduces to its x/y — which is why isFinitePose checks two
// fields, not three. That is a load-bearing property of Angle, not an omission here.

#include <cmath>
#include <string_view>

#include "shulib/diag/fault.hpp"
#include "shulib/math/pose2d.hpp"

namespace shulib::diag {

/// True iff the pose is finite. Heading is finite by construction (header note).
[[nodiscard]] inline bool isFinitePose(const math::Pose2d& p) noexcept {
    return std::isfinite(p.x().value()) && std::isfinite(p.y().value());
}

/// `candidate` if finite; otherwise raise `code` on the latch and return `fallback`
/// (degraded to 0.0 if the fallback is itself non-finite — the guarantee above).
/// `what` names the quantity for the fault line (e.g. "fused vx").
[[nodiscard]] inline double recoverFinite(double candidate, double fallback, FaultLatch& faults,
                                          FaultCode code, std::string_view subsystem,
                                          std::string_view what) noexcept {
    if (std::isfinite(candidate)) {
        return candidate;
    }
    faults.raise(code, subsystem, what);
    return std::isfinite(fallback) ? fallback : 0.0;
}

/// Pose overload: raises NAN_POSE. `fallback` should be the last-known-good pose; a
/// non-finite fallback degrades to the origin pose (finite, unconditionally).
[[nodiscard]] inline math::Pose2d recoverFinitePose(const math::Pose2d& candidate,
                                                    const math::Pose2d& fallback,
                                                    FaultLatch& faults,
                                                    std::string_view subsystem) noexcept {
    if (isFinitePose(candidate)) {
        return candidate;
    }
    faults.raise(FaultCode::NanPose, subsystem, "non-finite pose -> fallback");
    return isFinitePose(fallback) ? fallback : math::Pose2d{};
}

}  // namespace shulib::diag
