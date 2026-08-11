#pragma once
//
// Angle — a heading on SE(2). The one type that owns angle wrapping, so the
// "degrees into cos/sin" and "359° vs -1°" bug classes are impossible by
// construction.
//
// Conventions (LOCKED — master plan §7 / Freeze F3):
//   * Internal unit: RADIANS. Degrees appear only at this API boundary.
//   * Normalization interval: (-π, π]  — half-open at -π; +π is the canonical
//     boundary value, so an angle NEVER stores -π (it stores +π instead).
//   * errorTo() returns the shortest signed rotation, also in (-π, π]; the
//     exact-antipodal case (|Δ| == π) resolves deterministically to +π, never -π.
//   * CCW-positive, matching the field frame.
//
// All construction goes through radians()/degrees(), which reject non-finite
// input — a NaN can never enter the type.

#include <cmath>

#include "shulib/core/check.hpp"

namespace shulib::math {

class Angle {
public:
    /// Pi, exact to double precision. Public so tests/callers share one constant.
    static constexpr double kPi = 3.14159265358979323846;

    /// The zero heading.
    Angle() = default;

    /// Construct from radians; input must be finite. Result is wrapped to (-π, π].
    [[nodiscard]] static Angle radians(double r) {
        SHULIB_PRECONDITION(std::isfinite(r), "Angle::radians: non-finite input");
        return Angle{wrapRad(r)};
    }

    /// Construct from degrees; input must be finite. Result is wrapped to (-π, π].
    [[nodiscard]] static Angle degrees(double d) {
        SHULIB_PRECONDITION(std::isfinite(d), "Angle::degrees: non-finite input");
        return Angle{wrapRad(d * kDegToRad)};
    }

    [[nodiscard]] double radians() const noexcept { return rad_; }
    [[nodiscard]] double degrees() const noexcept { return rad_ * kRadToDeg; }

    /// Shortest signed rotation FROM this heading TO `target`, in (-π, π] radians.
    /// The exact-antipodal case resolves to +π (never -π) — see Freeze F3.
    [[nodiscard]] double errorTo(Angle target) const noexcept {
        return wrapRad(target.rad_ - rad_);
    }

    [[nodiscard]] Angle operator+(Angle o) const noexcept { return Angle{wrapRad(rad_ + o.rad_)}; }
    [[nodiscard]] Angle operator-(Angle o) const noexcept { return Angle{wrapRad(rad_ - o.rad_)}; }
    [[nodiscard]] Angle operator-() const noexcept { return Angle{wrapRad(-rad_)}; }

    /// True if the shortest error to `o` is within `tolRad`. Never compares
    /// raw doubles with ==, and is correct across the wrap boundary.
    [[nodiscard]] bool approxEqual(Angle o, double tolRad = 1e-9) const noexcept {
        return std::abs(errorTo(o)) <= tolRad;
    }

private:
    /// Trusted ctor — `wrappedRadians` is already in (-π, π].
    explicit Angle(double wrappedRadians) noexcept : rad_{wrappedRadians} {}

    static constexpr double kDegToRad = kPi / 180.0;
    static constexpr double kRadToDeg = 180.0 / kPi;

    /// Normalize any finite radian value to (-π, π].
    /// std::remainder gives [-π, π]; we then fold the -π endpoint up to +π.
    [[nodiscard]] static double wrapRad(double r) noexcept {
        double w = std::remainder(r, 2.0 * kPi);  // -> [-π, π]
        if (w <= -kPi) {
            w += 2.0 * kPi;                        // map -π up to +π  => (-π, π]
        }
        return w;
    }

    double rad_ = 0.0;
};

}  // namespace shulib::math
