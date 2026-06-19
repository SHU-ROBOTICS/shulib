#pragma once
//
// Pose2d — a rigid-body pose on SE(2): position (x, y) + heading.
//
// Position is type-safe Length (canonical inches); heading is the wrapping
// Angle, so pose comparison is correct across the ±180° seam. Built only on
// shulib::math::Angle and shulib::units (master plan §6, frame per §7 / F1).

#include <cmath>

#include "shulib/math/angle.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::math {

class Pose2d {
public:
    constexpr Pose2d() = default;
    constexpr Pose2d(units::Length x, units::Length y, Angle heading) noexcept
        : x_{x}, y_{y}, h_{heading} {}

    [[nodiscard]] constexpr units::Length x() const noexcept { return x_; }
    [[nodiscard]] constexpr units::Length y() const noexcept { return y_; }
    [[nodiscard]] constexpr Angle heading() const noexcept { return h_; }

    /// Component-wise closeness. Heading uses the SHORTEST angular error, so
    /// 179° vs -179° are 2° apart (not 358°) — comparison respects the wrap.
    [[nodiscard]] bool approxEqual(const Pose2d& o,
                                   units::Length posTol = units::Length{1e-6},
                                   double headTolRad = 1e-9) const noexcept {
        return std::abs((x_ - o.x_).value()) <= posTol.value()
            && std::abs((y_ - o.y_).value()) <= posTol.value()
            && std::abs(h_.errorTo(o.h_)) <= headTolRad;
    }

private:
    units::Length x_{};
    units::Length y_{};
    Angle h_{};
};

}  // namespace shulib::math
