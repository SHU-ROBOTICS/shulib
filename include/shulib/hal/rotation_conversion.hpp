#pragma once
//
// Rotation-sensor canonical conversions — the ONE place the V5 rotation sensor's
// centidegrees become shulib's canonical radians (§7: "convert exactly once, at
// the edge"). Pure, PROS-free, host- and mutation-testable in isolation; the
// hal/pros IRotation adapter is thin glue that CALLS these.
//
// V5 rotation sensor convention (vendored pros/rotation.hpp:195-242): both
// get_position() and get_velocity() are int32 CENTIDEGREES (position cumulative,
// never wrapping — HA-11; 36000 ticks/rev — HA-16; velocity in centideg/s —
// HA-105). The sensor's reversed flag is applied by PROS itself when the sensor
// is constructed on a NEGATIVE port (rotation.hpp:46-47) — that is the "exactly
// once" of rotation.hpp:4-6, and the adapter must NOT negate again on top.
//
// Sentinel note (T7): PROS_ERR (= INT32_MAX) is IN-BAND for an int32
// centidegree reading — 2147483647 centideg is ~59652 revolutions, unreachable
// in a match (HA-11's arithmetic) but perfectly finite. ONLY the adapter can
// screen it, BEFORE calling these. The conversions take double so the
// PROS_ERR_F backstop still works if a future float path appears.

#include <cmath>

#include "shulib/core/check.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::hal {

/// Cumulative centidegrees (get_position(), HA-11/HA-16) → canonical cumulative
/// radians. Deliberately NOT math::Angle: odometry integrates total
/// tracking-wheel travel, so this must never wrap (rotation.hpp:8-9).
/// π/18000 because 1 centidegree = (1/100)° = (1/100)·(π/180) rad. Drop the
/// scale and every tracking-wheel delta is 5730× — one inch of travel reads as
/// 477 feet, and odometry is garbage from the first tick.
[[nodiscard]] inline units::AngleDim rotationCentidegToCanonical(double centidegrees) {
    SHULIB_PRECONDITION(std::isfinite(centidegrees),
                        "rotationCentidegToCanonical: position must be finite (screen PROS_ERR first)");
    constexpr double kCentidegToRad = math::Angle::kPi / 18000.0;
    return units::AngleDim{centidegrees * kCentidegToRad};
}

/// Centidegrees-per-second (get_velocity(), HA-105) → canonical rad/s. Same
/// scale as position — one factor, one place.
[[nodiscard]] inline units::AngularVelocity rotationCentidegPerSecToCanonical(
    double centidegPerSec) {
    SHULIB_PRECONDITION(std::isfinite(centidegPerSec),
                        "rotationCentidegPerSecToCanonical: velocity must be finite (screen PROS_ERR first)");
    constexpr double kCentidegToRad = math::Angle::kPi / 18000.0;
    return units::AngularVelocity{centidegPerSec * kCentidegToRad};
}

}  // namespace shulib::hal
