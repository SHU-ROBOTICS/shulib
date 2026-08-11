#pragma once
//
// desaturateUniform — the downstream wheel-speed safety scale (master plan §5
// data-flow, §13 #5).
//
// If any wheel exceeds `maxWheelSpeed` in magnitude, scale EVERY wheel by the
// same factor so the largest just reaches the limit. Scaling uniformly preserves
// the ratios between wheels — and therefore the *direction* of the commanded
// motion — trading only speed for feasibility. If the command already fits within
// budget it is returned unchanged (never scaled UP).
//
// This is deliberately separate from strafeAuthority() clamping (§13 #5): that
// shapes the command upstream; this is the last-line guarantee that no wheel is
// ever asked for more than it can give. toWheels() itself stays clamp-free.

#include "shulib/core/check.hpp"
#include "shulib/kinematics/wheel_speeds.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::kinematics {

[[nodiscard]] inline WheelSpeeds desaturateUniform(const WheelSpeeds& wheels,
                                                   units::Velocity maxWheelSpeed) {
    SHULIB_PRECONDITION(maxWheelSpeed.value() > 0.0,
                        "desaturateUniform: maxWheelSpeed must be > 0");

    const double peak = wheels.maxMagnitude().value();
    if (peak <= maxWheelSpeed.value()) {
        return wheels;  // within budget (incl. all-zero) — never scale up
    }

    const double scale = maxWheelSpeed.value() / peak;  // strictly in (0, 1)
    WheelSpeeds out{wheels.size()};
    for (int i = 0; i < wheels.size(); ++i) {
        out.set(i, wheels[i] * scale);
    }
    return out;
}

}  // namespace shulib::kinematics
