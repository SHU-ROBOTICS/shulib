#pragma once
//
// arc_step.hpp — the one constant-curvature integration step (master plan §8 localization;
// the backbone of `PilonsOdometry`). This is the SE(2) exponential map: given how the
// tracking center moved in the BODY frame over one tick (forward + lateral travel) and how
// the heading changed over that same tick, it returns the FIELD-frame displacement (Δx, Δy).
//
// ── Why this exists as its own pure function ────────────────────────────────────────────
// It is the single most accuracy-critical line in the localizer (it runs ~100×/s and its
// error accumulates over a 60s run), so it is isolated, offset-agnostic, and exhaustively
// testable on its own. It is also GENERAL: it integrates any body twist, so the same
// primitive serves tracking-wheel odometry today and kinematics-integrated odometry (our
// holonomic X-drive) later — both just supply (forward, lateral, Δheading).
//
// ── Derivation (constant-twist exponential map on SE(2)) ────────────────────────────────
// Assume the body twist (v_f forward, v_l lateral, ω yaw-rate) is constant across the tick
// of length T. Heading starts at θ₀ and θ(t) = θ₀ + ω·t, so Δθ = ω·T. The field-frame
// velocity is the body velocity rotated by the instantaneous heading:
//     ẋ = v_f·cos θ(t) − v_l·sin θ(t)
//     ẏ = v_f·sin θ(t) + v_l·cos θ(t)
// Integrate t : 0→T. With the measured body travels f = v_f·T and l = v_l·T, and writing
// the integrals in closed form, the FIELD displacement is:
//     Δx = (f/Δθ)·sin Δθ        − (l/Δθ)·(1 − cos Δθ)     [rotated by θ₀]
//     Δy = (f/Δθ)·(1 − cos Δθ)  + (l/Δθ)·sin Δθ
// Apply the half-angle identities sin Δθ = 2sc, 1−cos Δθ = 2s² (s = sin(Δθ/2), c = cos(Δθ/2)):
//     (Δx, Δy)_field = k · R(θ₀ + Δθ/2) · (f, l),     k ≡ 2·sin(Δθ/2) / Δθ = sin(Δθ/2)/(Δθ/2).
// So the exact result is: scale the body travel by the chord factor k, then rotate it by the
// **AVERAGE** heading θ₀ + Δθ/2 (NOT the start heading, NOT the end heading). R(+θ) here is
// the same body→field rotation as math::robotToField (F1: +X fwd, +Y left, CCW).
//
//   * k = 2·sin(Δθ/2)/Δθ is a sinc: it is the ratio chord-length / arc-length. As Δθ→0 it
//     → 1 (the path is straight, chord == travel); the only singular point is Δθ == 0 (0/0),
//     guarded explicitly below. For all other Δθ, sin(Δθ/2)/(Δθ/2) is well-conditioned
//     (no cancellation), so no series approximation is needed.
//   * Rotating by the AVERAGE heading is the crux. The legacy `lodge` odometry rotated the
//     chord by the *new* heading (θ₀ + Δθ) — a systematic +Δθ/2 bias every tick. On a 90°
//     arc that lands a (10,10) move at (0,14.14); per-tick at 100 Hz it is a small but
//     accumulating drift that alone would blow the < 1° / sub-inch budget. Re-derived from
//     scratch here precisely so that bug is not inherited. (Verified against the exact
//     endpoint in arc_step_test.cpp.)
//
// Heading is IMU-OWNED: Δθ is taken from the two heading samples the caller passes (the IMU's
// canonical heading at the start and end of the tick), via Angle::errorTo (shortest signed,
// wrap-correct). Body offsets of the physical tracking wheels (wheel travel → center travel)
// are the CALLER's job — this function already works in tracking-center coordinates, keeping
// the geometry pure.
//
// PRECONDITION (caller-guaranteed; NOT checkable here): the TRUE rotation between the two
// samples is within (-π, π]. arcStep sees only the already-wrapped `Angle`s, so a real per-tick
// rotation > π is indistinguishable from its shorter alias (270° and −90° are the *same*
// `Angle`) — the wrap is lossy before arcStep is even called. At ~100 Hz this always holds
// (even 1000°/s is 10°/tick). The plausibility gate against an aliased/stalled heading sample
// therefore lives in `PilonsOdometry` (which sees consecutive ticks), not here.

#include <cmath>

#include "shulib/math/angle.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::localization {

/// One field-frame displacement from one integration step (canonical inches).
struct FieldDelta {
    units::Length dx{};
    units::Length dy{};
};

/// The tracking center's BODY-frame travel over one tick (+forward = +X body; +lateral = +Y
/// body / left). A named pair so a call site can never silently swap the two same-typed
/// `Length`s — the most likely integration foot-gun for an accuracy-critical primitive.
struct BodyTravel {
    units::Length forward{};
    units::Length lateral{};
};

/// Below this |Δθ| (radians) the step is treated as straight (k = 1). Only guards the exact
/// Δθ == 0 singularity (0/0); the chord factor is otherwise computed directly. At this bound
/// the dropped curvature term is O(Δθ²) ≈ 1e-18 — far below the inch we care about.
inline constexpr double kArcStraightEps = 1e-9;

/// Integrate one constant-curvature step. `travel` is the tracking CENTER's body-frame travel
/// over the tick; `headingStart`/`headingEnd` are the field headings at the tick boundaries
/// (IMU-owned). Returns the field-frame displacement to add to the pose. Pure and total over
/// finite inputs (see the PRECONDITION above on the per-tick rotation magnitude).
[[nodiscard]] inline FieldDelta arcStep(BodyTravel travel,
                                        math::Angle headingStart,
                                        math::Angle headingEnd) noexcept {
    const double f = travel.forward.value();
    const double l = travel.lateral.value();
    const double dTheta = headingStart.errorTo(headingEnd);  // shortest signed Δθ, wrap-correct
    const double half = 0.5 * dTheta;

    // Chord factor k = sin(Δθ/2)/(Δθ/2); → 1 as Δθ → 0 (straight-line limit).
    const double k = (std::abs(dTheta) < kArcStraightEps) ? 1.0 : (std::sin(half) / half);

    // Rotate the (scaled) body travel by the AVERAGE heading θ₀ + Δθ/2 — the load-bearing detail.
    const double thAvg = headingStart.radians() + half;
    const double c = std::cos(thAvg);
    const double s = std::sin(thAvg);

    return FieldDelta{units::Length{k * (f * c - l * s)},
                      units::Length{k * (f * s + l * c)}};
}

}  // namespace shulib::localization
