#pragma once
//
// sim::TruthState + advanceTruth — the plant's GROUND-TRUTH pose integrator
// (chunk A2, constraint 2 — the single most important constraint in the chunk).
//
// ── WHY THIS MUST NOT USE arcStep (read before touching) ────────────────────────────
// `PilonsOdometry` integrates pose with `localization::arcStep`. If the plant's truth
// were computed by that same arcStep, ANY error in arcStep would appear identically on
// both sides of every localization comparison and CANCEL OUT — odometry would look
// perfect while both sides made the same mistake, and every Phase E estimator-vs-truth
// measurement would be silently worthless. This trap is invisible once made: the tests
// stay green, the numbers look beautiful, and nothing is being proven.
//
// So truth here is advanced by CLASSIC RK4 on the SE(2) kinematic ODE
//
//     ẋ = vx·cos θ − vy·sin θ,   ẏ = vx·sin θ + vy·cos θ,   θ̇ = ω
//
// with fixed sub-stepping. RK4 shares ZERO algebraic structure with arcStep's
// half-angle chord form (k = 2·sin(Δθ/2)/Δθ rotated by the average heading): no chord
// factor, no Angle::errorTo, no wrap. The two are then TESTED AGAINST EACH OTHER
// (test/sim_truth_test.cpp) — a genuine two-sided check of arcStep that is only
// possible because they are independent.
//
// ── θ IS UNWRAPPED, ON PURPOSE (this is also the independence tripwire) ─────────────
// arcStep receives wrapped `Angle`s, so a per-tick rotation beyond π is
// STRUCTURALLY unrepresentable there — it aliases to its shorter complement before
// arcStep even runs (documented as arcStep's caller PRECONDITION). Truth has no such
// precondition: a plant driven with a large dt (or a fast spin) legitimately sweeps
// more than π in one step, and this integrator handles it exactly because θ is a raw
// accumulating double. That difference is what makes the independence MUTATION test
// possible at all: since arcStep is the exact closed form for a constant twist, an
// agreement test alone can NEVER catch a truth integrator that secretly reuses it
// (they would agree to machine precision) — but the >π-per-tick case red-flags it
// immediately (test/sim_truth_test.cpp, "beyond arcStep's wrap horizon").
//
// ── Accuracy budget (documented, and verified by Richardson in the tests) ───────────
// Classic RK4 has local error O(h⁵), global O(h⁴) over the tick. With the default
// 32 substeps and per-tick sweeps up to ~π, the endpoint error is ≲ 1e-12 inches per
// tick — far below every tolerance any A2/E test asserts (the truth-vs-arcStep suite
// pins agreement at 1e-9 absolute). The tests also verify convergence directly
// (N vs 2N substeps agree to ~h⁴), so the budget is measured, not assumed.
//
// The body twist is CONSTANT across the step by definition — the plant advances wheel
// velocities first, then integrates the resulting twist over dt (zero-order hold, the
// same per-tick model odometry's derivation assumes). Pure and total over finite
// inputs; no trig identities to go singular, no 0/0 to guard.

#include <cmath>

#include "shulib/core/check.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::sim {

/// The exact simulated pose. x/y in canonical inches; theta in radians, UNWRAPPED
/// (see header — wrapping happens only at the Pose2d conversion edge). Raw doubles
/// on purpose: this is the harness's private truth currency, not a HAL value, and
/// keeping it structurally distinct from Pose2d is part of what stops truth being
/// handed to an estimator by accident.
struct TruthState {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;

    /// The wrapped, typed view for assertions and sensor synthesis.
    [[nodiscard]] math::Pose2d pose() const {
        return math::Pose2d{units::Length{x}, units::Length{y}, math::Angle::radians(theta)};
    }
};

/// Advance `state` by `dt` under a constant BODY-frame twist, via `substeps` RK4
/// steps (see header for why RK4 and why not arcStep). dt must be finite and >= 0;
/// substeps >= 1. dt == 0 returns the state unchanged.
[[nodiscard]] inline TruthState advanceTruth(const TruthState& state, const math::Twist2d& body,
                                             units::Time dt, int substeps) {
    SHULIB_PRECONDITION(std::isfinite(dt.value()) && dt.value() >= 0.0,
                        "advanceTruth: dt must be finite and >= 0");
    SHULIB_PRECONDITION(substeps >= 1, "advanceTruth: substeps must be >= 1");

    const double vx = body.vx().value();
    const double vy = body.vy().value();
    const double w = body.omega().value();
    const double h = dt.value() / static_cast<double>(substeps);

    TruthState s = state;
    for (int i = 0; i < substeps; ++i) {
        // Classic RK4 on (x, y, θ). θ̇ = ω is constant, so the stage headings are
        // exact; the x/y stages sample the rotated body velocity at those headings.
        const double th0 = s.theta;
        const double thMid = s.theta + 0.5 * h * w;  // stages 2 and 3 share this heading
        const double th1 = s.theta + h * w;

        const double k1x = vx * std::cos(th0) - vy * std::sin(th0);
        const double k1y = vx * std::sin(th0) + vy * std::cos(th0);
        const double k23x = vx * std::cos(thMid) - vy * std::sin(thMid);
        const double k23y = vx * std::sin(thMid) + vy * std::cos(thMid);
        const double k4x = vx * std::cos(th1) - vy * std::sin(th1);
        const double k4y = vx * std::sin(th1) + vy * std::cos(th1);

        s.x += (h / 6.0) * (k1x + 4.0 * k23x + k4x);
        s.y += (h / 6.0) * (k1y + 4.0 * k23y + k4y);
        s.theta = th1;
    }
    return s;
}

}  // namespace shulib::sim
