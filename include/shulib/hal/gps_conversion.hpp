#pragma once
//
// GPS canonical conversions — the ONE place the VEX GPS frame becomes shulib's
// canonical frame (§7: "convert exactly once, at the edge"). Pure, PROS-free, so the
// frame math is fully host-testable; the hal/pros GPS adapter is thin glue.
//
// VEX GPS convention: heading in DEGREES [0,360), CLOCKWISE from North (0=N, 90=E,
// 180=S, 270=W) — confirmed in the vendored pros/gps.hpp. Position in METERS, (0,0) =
// field center, 4-quadrant Cartesian.
//
// shulib canonical (F1): inches; FIELD +X right / +Y away-from-red; BODY +X forward /
// +Y left; heading CCW-positive, +X = 0, wrapped (-π,π]. Handedness flips (CW→CCW) by
// subtraction.
//
// *** ASSUMPTION — position axes (VEX +X = East, +Y = North): UNVERIFIED. PROS does NOT
//     document the position-axis-to-compass binding. A WRONG axis label silently MIRRORS
//     the pose, and northHeadingDeg CANNOT recover it (that knob is a rotation; an axis
//     swap/flip is a reflection). Bench-measure the raw→wall mapping before any scored
//     run — see the field-cal oracle test in gps_conversion_test.cpp. ***
//
// ONE rotation parameter, northHeadingDeg = the canonical heading VEX-North points
// toward. Default 90° (VEX-North = canonical +Y = away from red); the other canonical
// value is 270/−90° (red at the +Y wall). FIELD/ALLIANCE setup fact, ONE owner = the
// robot's start pose (same authority as the IMU bootHeading), validated on field.
//
// ADAPTER BINDING CONTRACT (load-bearing, like the IMU's — enforce when hal/pros lands):
//  * Lever arm removed HERE (gpsRemoveLeverArm), ONE owner = robot config (BODY
//    forward/left INCHES — NOT the East/North meters of set_offset). Construct pros::Gps
//    via the PORT-ONLY ctor; the adapter MUST NOT use set_offset(), initialize_full(),
//    or the offset-taking ctors — any firmware offset makes get_position() report the
//    CENTER, double-subtracting the arm (inches of silent bias). Boot-check get_offset()==(0,0).
//  * The adapter MUST screen PROS_ERR_F (== INFINITY: a failed/off-strip/calibrating read)
//    and report IGps::hasFix() = false BEFORE calling this conversion — off-strip screening
//    is the adapter's job (§13 #4; Driving Skills has no strip). Feeding a sentinel here
//    THROWS by design (fail-loud backstop, NOT the off-strip path).
//  * rmsError() at the HAL edge MUST scale get_error() meters→inches (× kMetersToInches),
//    or the corrector's R is ~39× too small and good fixes get gated out.

#include <cmath>

#include "shulib/core/check.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::hal {

inline constexpr double kGpsDefaultNorthHeadingDeg = 90.0;       // VEX-North = canonical +Y
inline constexpr double kMetersToInches = 39.3700787401574803;   // 1 / 0.0254, exact-ish

/// VEX GPS heading (deg, CW from North, [0,360)) → canonical Angle (CCW from +X).
/// canonical = (canonical heading of North) − (CW degrees turned from North), wrapped.
[[nodiscard]] inline math::Angle gpsHeadingToCanonical(
    double headingDegCwFromNorth, double northHeadingDeg = kGpsDefaultNorthHeadingDeg) {
    // deg→rad via the sanctioned Angle boundary (also rejects non-finite heading);
    // subtraction flips CW→CCW; Angle wraps.
    return math::Angle::degrees(northHeadingDeg) - math::Angle::degrees(headingDegCwFromNorth);
}

/// Canonical pose of the GPS SENSOR (no lever-arm removal yet) from a raw VEX reading.
/// Position (meters, VEX East/North) is rotated into the canonical frame and scaled to
/// inches; heading is converted as above.
[[nodiscard]] inline math::Pose2d gpsSensorPose(
    double xMeters, double yMeters, double headingDegCwFromNorth,
    double northHeadingDeg = kGpsDefaultNorthHeadingDeg) {
    SHULIB_PRECONDITION(std::isfinite(xMeters) && std::isfinite(yMeters),
                        "gpsSensorPose: position must be finite");
    // θ_N = canonical direction of VEX-North; VEX-East sits at θ_N − 90°.
    const double thetaN = math::Angle::degrees(northHeadingDeg).radians();
    const double s = std::sin(thetaN);
    const double c = std::cos(thetaN);
    const double cxMeters = xMeters * s + yMeters * c;   // East·sinθ + North·cosθ
    const double cyMeters = -xMeters * c + yMeters * s;  // East·(−cosθ) + North·sinθ
    return math::Pose2d{units::Length{cxMeters * kMetersToInches},
                        units::Length{cyMeters * kMetersToInches},
                        gpsHeadingToCanonical(headingDegCwFromNorth, northHeadingDeg)};
}

/// Shift a SENSOR pose to the robot CENTER by removing the lever arm. The lever arm is
/// the GPS sensor's position in the BODY frame (forward = +X, left = +Y, per F1). The
/// sensor sits at center + R(heading)·leverArm, so center = sensor − R(heading)·leverArm.
[[nodiscard]] inline math::Pose2d gpsRemoveLeverArm(
    const math::Pose2d& sensorPose, units::Length leverArmForward, units::Length leverArmLeft) {
    SHULIB_PRECONDITION(std::isfinite(leverArmForward.value()) && std::isfinite(leverArmLeft.value()),
                        "gpsRemoveLeverArm: lever arm must be finite");
    const double h = sensorPose.heading().radians();
    const double c = std::cos(h);
    const double s = std::sin(h);
    const double lf = leverArmForward.value();
    const double ll = leverArmLeft.value();
    const double offX = c * lf - s * ll;  // R(+heading) · (forward, left) → field
    const double offY = s * lf + c * ll;
    return math::Pose2d{units::Length{sensorPose.x().value() - offX},
                        units::Length{sensorPose.y().value() - offY},
                        sensorPose.heading()};
}

/// Full path: raw VEX reading → canonical robot-CENTER pose.
[[nodiscard]] inline math::Pose2d gpsToRobotPose(
    double xMeters, double yMeters, double headingDegCwFromNorth,
    units::Length leverArmForward, units::Length leverArmLeft,
    double northHeadingDeg = kGpsDefaultNorthHeadingDeg) {
    return gpsRemoveLeverArm(
        gpsSensorPose(xMeters, yMeters, headingDegCwFromNorth, northHeadingDeg),
        leverArmForward, leverArmLeft);
}

}  // namespace shulib::hal
