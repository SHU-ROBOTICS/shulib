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

/// A rigid-body pose on SE(2): position plus heading. The type carries NO frame tag — nearly
/// every Pose2d in shulib is a FIELD pose (F1: +X right, +Y away from the red driver station,
/// heading 0 along +X, CCW-positive), but a body-frame pose uses this same type (an AprilTag's
/// pose relative to the robot is one), so which frame a given Pose2d is in is a fact about the
/// variable, not about the type. Position is a type-safe Length in canonical inches and heading
/// is the WRAPPING Angle, so comparing two poses is correct across the ±180° seam rather than
/// reporting 358° of error.
class Pose2d {
public:
    /// The origin at heading 0. NOT a "no pose" sentinel — it is indistinguishable from a robot
    /// genuinely sitting at the origin, which is why the Localizer publishes a separate
    /// `Quality` (Uninitialized / DeadReckon / …) instead of overloading a default pose.
    constexpr Pose2d() = default;
    /// Position in canonical inches; `heading` is already a wrapped Angle so nothing is
    /// normalized here. NO validation: a non-finite Length passes straight through (Angle's
    /// factories reject non-finite input, Length has no such guard), so callers that care screen
    /// it themselves — the Localizer runs std::isfinite over a corrector's proposed x/y before
    /// fusing it.
    constexpr Pose2d(units::Length x, units::Length y, Angle heading) noexcept
        : x_{x}, y_{y}, h_{heading} {}

    /// The x coordinate, in canonical inches. In the field frame +X points right, and heading 0
    /// points along it — so x is the "forward" axis for a robot at heading 0.
    [[nodiscard]] constexpr units::Length x() const noexcept { return x_; }
    /// The y coordinate, in canonical inches. In the field frame +Y points away from the red
    /// driver station; with CCW-positive heading that pins the whole sign convention.
    [[nodiscard]] constexpr units::Length y() const noexcept { return y_; }
    /// The heading, as the WRAPPING Angle: already normalized to (-π, π], CCW-positive, 0 along
    /// +X. Compare it with Angle::errorTo / approxEqual — differencing radians() as raw doubles
    /// reintroduces exactly the ±180° seam bug this type exists to make impossible.
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
