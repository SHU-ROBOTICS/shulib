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
// shapes the command upstream; this bounds the MAGNITUDE of a finite wheel set.
// It is NOT the last line, and used to claim it was. Non-finite input passes
// straight through: WheelSpeeds::maxMagnitude() uses std::max, which ignores NaN,
// so a NaN never wins the peak and the early return treats the set as within
// budget — and in the scaling branch NaN * scale is NaN either way. The actual
// last line is diag::recoverWheelVoltage at the motor edge (plausibility_guard.hpp
// invariant 3), which zeroes a non-finite volt and raises Implausible. Enforcing
// finiteness HERE was considered and rejected twice over: it would narrow the
// contract of LOCKED register row F5, and it would turn an A3 hostile-sensor
// pathology into a thrown precondition — an aborted motion where the design says
// log and recover. toWheels() itself stays clamp-free.

#include "shulib/core/check.hpp"
#include "shulib/kinematics/wheel_speeds.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::kinematics {

/// Scale EVERY wheel by ONE common factor so the largest just reaches `maxWheelSpeed`.
/// Uniform scaling preserves the ratios between wheels — and therefore the DIRECTION of the
/// commanded motion — trading only speed for feasibility. A set already inside the budget
/// (the all-zero set included) is returned unchanged: this never scales UP, so it cannot be
/// used to reach a speed floor. The result has the same size() as the input, and both the
/// input and the limit are canonical velocity. Precondition: maxWheelSpeed > 0 — a zero or
/// negative limit is a caller bug, not a request to hold still.
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
