#pragma once
//
// Quantity<L, A, T, E, I> — compile-time dimensional analysis.
//
// The integer exponents of the five base dimensions live in the TYPE:
//     L = length, A = angle, T = time, E = electric potential (voltage),
//     I = electric current (amperes).
// (I was added 2026-06-19, additively, for IMotor/IBattery current() — F4 review.
//  The original four dims and their semantics are unchanged; F3 stays intact.)
// Consequences, all enforced by the compiler:
//   * adding/subtracting different dimensions FAILS TO COMPILE,
//   * multiply/divide compute the resulting dimension automatically
//     (Length / Time -> Velocity, Velocity / Time -> Acceleration, ...),
//   * the stored value is ALWAYS canonical: inch, radian, second, volt, AMPERE. The ampere
//     was missing from this list while the same header declared the current dimension and
//     the Current alias, and hal/pros/battery.hpp stores units::Current{ma / 1000.0} — a
//     canonical ampere, not a derived combination of the other four.
//
// This kills two bug classes at compile time: "degrees into cos/sin" and
// "milliseconds into a seconds-based gain". (master plan §7, §13 #3; Freeze F3.)
//
// Headings use the dedicated wrapping type shulib::math::Angle; the angle
// dimension here exists for RATES (e.g. angular velocity rad/s) and bookkeeping.

#include <compare>

namespace shulib::units {

/// A double that carries its DIMENSION in its type: the five integer exponents are
/// length, angle, time, voltage, current. The stored value is ALWAYS canonical —
/// inches, radians, seconds, volts, amperes, and combinations of them — so there is no
/// unit tag to read and no conversion left for a caller to forget. Same-dimension
/// arithmetic behaves normally; mixing dimensions under `+`/`-` names no overload and
/// fails to compile, while `*` and `/` derive the result's dimension. One double wide
/// and constexpr throughout, so the checking costs nothing at run time. A HEADING is
/// not this type: use math::Angle, which owns wrapping (see the banner above).
template <int L, int A, int T, int E, int I>
class Quantity {
public:
    /// The dimensioned zero. Never indeterminate — the stored value is 0.0 whether or
    /// not the declaration writes braces.
    constexpr Quantity() = default;

    /// Construct from a value ALREADY in canonical units. Explicit, so a bare double
    /// never silently becomes a dimensioned quantity — and unchecked, so a NaN, or a
    /// number that is really in degrees or milliseconds, is stored as though it were
    /// canonical. Prefer the units::literals (`24_in`, `20_ms`), which do the
    /// conversion once, where the number is written.
    constexpr explicit Quantity(double canonicalValue) noexcept : v_{canonicalValue} {}

    /// The value in canonical units (inch / radian / second / volt / ampere, or a derived
    /// combination). The one way out of the type — everything past this point is a bare
    /// double again, so unwrap as late as possible.
    [[nodiscard]] constexpr double value() const noexcept { return v_; }

    // --- same-dimension addition (mismatched dimensions are a different type -> no overload) ---
    /// Add in place. A different dimension is a different TYPE, so there is no conversion
    /// left to get wrong: `len += dt` names no overload at all.
    constexpr Quantity& operator+=(Quantity o) noexcept { v_ += o.v_; return *this; }
    /// Subtract in place. Signed — the value goes negative rather than saturating at zero.
    constexpr Quantity& operator-=(Quantity o) noexcept { v_ -= o.v_; return *this; }

    /// Sum, dimension unchanged — `+` is the one operator here that CANNOT derive a new type:
    /// the exponents `*` and `/` add and subtract simply carry through. A bare number is not an
    /// operand: the constructor is explicit and there is no `Quantity + double` overload (unlike
    /// `*` and `/`, which do take a dimensionless scalar), so `len + 2.0` names no overload at
    /// all — write `len + 2_in`.
    [[nodiscard]] friend constexpr Quantity operator+(Quantity a, Quantity b) noexcept {
        return Quantity{a.v_ + b.v_};
    }
    /// Difference `a - b`, signed and unclamped: `b` larger than `a` yields a negative
    /// quantity of the same dimension.
    [[nodiscard]] friend constexpr Quantity operator-(Quantity a, Quantity b) noexcept {
        return Quantity{a.v_ - b.v_};
    }
    /// Negation — reverses the sense of a signed quantity (a displacement, a velocity, a
    /// voltage). shulib declares no `abs` for Quantity; take `std::abs(q.value())` when a
    /// magnitude is what you want.
    [[nodiscard]] friend constexpr Quantity operator-(Quantity a) noexcept { return Quantity{-a.v_}; }

    // --- scalar scaling (dimension unchanged) ---
    /// Scale by a DIMENSIONLESS factor; the dimension is unchanged. When the factor itself
    /// carries units, use the Quantity-by-Quantity operator below — that one changes it.
    [[nodiscard]] friend constexpr Quantity operator*(Quantity a, double s) noexcept { return Quantity{a.v_ * s}; }
    /// The same scaling with the scalar written on the left, so `0.5 * dt` and `dt * 0.5`
    /// are both spellable and identical.
    [[nodiscard]] friend constexpr Quantity operator*(double s, Quantity a) noexcept { return Quantity{s * a.v_}; }
    /// Divide by a dimensionless factor; dimension unchanged. `s == 0` yields infinity by
    /// IEEE rules, not an error. There is deliberately no `double / Quantity` overload, and the
    /// asymmetry with the DOUBLED multiply above is the point rather than an oversight: this
    /// block is scalar SCALING, which leaves the dimension alone, while a scalar divided by a
    /// quantity always INVERTS it — a different operation, belonging with the
    /// Quantity-by-Quantity operators. Write `Number{1.0} / dt`, which produces the inverse
    /// dimension through exactly those.
    [[nodiscard]] friend constexpr Quantity operator/(Quantity a, double s) noexcept { return Quantity{a.v_ / s}; }

    // --- comparison (same dimension only) ---
    /// EXACT equality of the canonical doubles — there is no tolerance anywhere in here. Two
    /// quantities computed by different routes will rarely compare equal; test
    /// `std::abs((a - b).value())` against a tolerance you chose instead.
    [[nodiscard]] friend constexpr bool operator==(Quantity a, Quantity b) noexcept { return a.v_ == b.v_; }
    /// Orders by canonical value. `partial_ordering`, not `strong_ordering`, because doubles
    /// are: if either side is NaN the result is `unordered`, and `<`, `>`, `<=` and `>=` are
    /// then ALL false.
    [[nodiscard]] friend constexpr std::partial_ordering operator<=>(Quantity a, Quantity b) noexcept {
        return a.v_ <=> b.v_;
    }

private:
    double v_ = 0.0;
};

// --- dimensioned multiply / divide: exponents add / subtract ---
/// Multiply two dimensioned quantities: the five exponents ADD, so the result TYPE is
/// derived rather than declared anywhere. Velocity * Time -> Length; Voltage * Current
/// -> Power; Number * anything -> that same dimension.
template <int L1, int A1, int T1, int E1, int I1, int L2, int A2, int T2, int E2, int I2>
[[nodiscard]] constexpr auto operator*(Quantity<L1, A1, T1, E1, I1> a, Quantity<L2, A2, T2, E2, I2> b) noexcept
    -> Quantity<L1 + L2, A1 + A2, T1 + T2, E1 + E2, I1 + I2> {
    return Quantity<L1 + L2, A1 + A2, T1 + T2, E1 + E2, I1 + I2>{a.value() * b.value()};
}

/// Divide two dimensioned quantities: the five exponents SUBTRACT. Length / Time ->
/// Velocity, Velocity / Time -> Acceleration, Length / Length -> Number. Dividing by a
/// zero quantity yields infinity by IEEE rules, not an error.
template <int L1, int A1, int T1, int E1, int I1, int L2, int A2, int T2, int E2, int I2>
[[nodiscard]] constexpr auto operator/(Quantity<L1, A1, T1, E1, I1> a, Quantity<L2, A2, T2, E2, I2> b) noexcept
    -> Quantity<L1 - L2, A1 - A2, T1 - T2, E1 - E2, I1 - I2> {
    return Quantity<L1 - L2, A1 - A2, T1 - T2, E1 - E2, I1 - I2>{a.value() / b.value()};
}

// --- the canonical dimension aliases (the "6 dims" of §13 #3: 4 base + 2 derived) ---
using Number          = Quantity<0, 0, 0, 0, 0>;   ///< dimensionless — a ratio, a gain, Length/Length
using Length          = Quantity<1, 0, 0, 0, 0>;   ///< inches; signed — a displacement, not a size
using AngleDim        = Quantity<0, 1, 0, 0, 0>;   ///< radians, for RATES; a heading is math::Angle
using Time            = Quantity<0, 0, 1, 0, 0>;   ///< seconds; `20_ms` converts to 0.02 at the literal
using Voltage         = Quantity<0, 0, 0, 1, 0>;   ///< volts — what IMotor::setVoltage commands
using Current         = Quantity<0, 0, 0, 0, 1>;   ///< amperes — the 5th dim, for IMotor/IBattery
using Velocity        = Quantity<1, 0, -1, 0, 0>;  ///< in/s — Length/Time, derived, never declared
using Acceleration    = Quantity<1, 0, -2, 0, 0>;  ///< in/s^2 — Velocity/Time, i.e. Length/Time^2
using AngularVelocity = Quantity<0, 1, -1, 0, 0>;  ///< rad/s — a yaw RATE; ChassisSpeeds' third term
using Power           = Quantity<0, 0, 0, 1, 1>;   ///< watts (V·A) — proof that Current composes

}  // namespace shulib::units
