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
//   * the stored value is ALWAYS canonical: inch, radian, second, volt.
//
// This kills two bug classes at compile time: "degrees into cos/sin" and
// "milliseconds into a seconds-based gain". (master plan §7, §13 #3; Freeze F3.)
//
// Headings use the dedicated wrapping type shulib::math::Angle; the angle
// dimension here exists for RATES (e.g. angular velocity rad/s) and bookkeeping.

#include <compare>

namespace shulib::units {

template <int L, int A, int T, int E, int I>
class Quantity {
public:
    constexpr Quantity() = default;

    // Construct from a value already in CANONICAL units. Explicit: a bare double
    // never silently becomes a dimensioned quantity.
    constexpr explicit Quantity(double canonicalValue) noexcept : v_{canonicalValue} {}

    /// The value in canonical units (inch / radian / second / volt, or a derived combination).
    [[nodiscard]] constexpr double value() const noexcept { return v_; }

    // --- same-dimension addition (mismatched dimensions are a different type -> no overload) ---
    constexpr Quantity& operator+=(Quantity o) noexcept { v_ += o.v_; return *this; }
    constexpr Quantity& operator-=(Quantity o) noexcept { v_ -= o.v_; return *this; }

    [[nodiscard]] friend constexpr Quantity operator+(Quantity a, Quantity b) noexcept {
        return Quantity{a.v_ + b.v_};
    }
    [[nodiscard]] friend constexpr Quantity operator-(Quantity a, Quantity b) noexcept {
        return Quantity{a.v_ - b.v_};
    }
    [[nodiscard]] friend constexpr Quantity operator-(Quantity a) noexcept { return Quantity{-a.v_}; }

    // --- scalar scaling (dimension unchanged) ---
    [[nodiscard]] friend constexpr Quantity operator*(Quantity a, double s) noexcept { return Quantity{a.v_ * s}; }
    [[nodiscard]] friend constexpr Quantity operator*(double s, Quantity a) noexcept { return Quantity{s * a.v_}; }
    [[nodiscard]] friend constexpr Quantity operator/(Quantity a, double s) noexcept { return Quantity{a.v_ / s}; }

    // --- comparison (same dimension only) ---
    [[nodiscard]] friend constexpr bool operator==(Quantity a, Quantity b) noexcept { return a.v_ == b.v_; }
    [[nodiscard]] friend constexpr std::partial_ordering operator<=>(Quantity a, Quantity b) noexcept {
        return a.v_ <=> b.v_;
    }

private:
    double v_ = 0.0;
};

// --- dimensioned multiply / divide: exponents add / subtract ---
template <int L1, int A1, int T1, int E1, int I1, int L2, int A2, int T2, int E2, int I2>
[[nodiscard]] constexpr auto operator*(Quantity<L1, A1, T1, E1, I1> a, Quantity<L2, A2, T2, E2, I2> b) noexcept
    -> Quantity<L1 + L2, A1 + A2, T1 + T2, E1 + E2, I1 + I2> {
    return Quantity<L1 + L2, A1 + A2, T1 + T2, E1 + E2, I1 + I2>{a.value() * b.value()};
}

template <int L1, int A1, int T1, int E1, int I1, int L2, int A2, int T2, int E2, int I2>
[[nodiscard]] constexpr auto operator/(Quantity<L1, A1, T1, E1, I1> a, Quantity<L2, A2, T2, E2, I2> b) noexcept
    -> Quantity<L1 - L2, A1 - A2, T1 - T2, E1 - E2, I1 - I2> {
    return Quantity<L1 - L2, A1 - A2, T1 - T2, E1 - E2, I1 - I2>{a.value() / b.value()};
}

// --- the canonical dimension aliases (the "6 dims" of §13 #3: 4 base + 2 derived) ---
using Number          = Quantity<0, 0, 0, 0, 0>;   // dimensionless
using Length          = Quantity<1, 0, 0, 0, 0>;   // inches
using AngleDim        = Quantity<0, 1, 0, 0, 0>;   // radians (rates/bookkeeping; headings use math::Angle)
using Time            = Quantity<0, 0, 1, 0, 0>;   // seconds
using Voltage         = Quantity<0, 0, 0, 1, 0>;   // volts
using Current         = Quantity<0, 0, 0, 0, 1>;   // amperes
using Velocity        = Quantity<1, 0, -1, 0, 0>;  // in/s
using Acceleration    = Quantity<1, 0, -2, 0, 0>;  // in/s^2
using AngularVelocity = Quantity<0, 1, -1, 0, 0>;  // rad/s
using Power           = Quantity<0, 0, 0, 1, 1>;   // watts (V·A) — demonstrates current composes

}  // namespace shulib::units
