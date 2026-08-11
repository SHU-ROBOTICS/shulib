#pragma once
//
// IMU canonical conversions — the ONE place the V5 inertial sensor's frame becomes
// shulib's canonical frame (§7: "convert exactly once, at the edge"). These are
// pure, PROS-free functions so the heading math — the crux of the < 1° requirement —
// is fully host-testable; the hal/pros IMU adapter is thin glue that reads pros::IMU
// and calls these.
//
// V5 IMU convention (confirmed against the vendored PROS headers — pros/imu.hpp: both
// get_rotation() and get_heading() document "clockwise rotations are positive"): the
// inertial sensor reports DEGREES, CLOCKWISE-positive, from the calibration orientation.
// shulib canonical: RADIANS, CCW-positive, +X = 0, wrapped to (-π, π]. Handedness flips
// by subtraction.
//
// ADAPTER BINDING CONTRACT (load-bearing — fix when the hal/pros IImu adapter lands):
//  * imuHeadingDegCw is CUMULATIVE CW degrees from calibration → bind to
//    pros::Imu::get_rotation() (theoretically unbounded), NOT get_heading() (bounded
//    [0,360), which loses the revolution continuity this cumulative contract assumes).
//    (A4 register HA-03.)
//  * The adapter MUST NOT call tare/tare_rotation/set_rotation/tare_heading/set_heading/
//    reset after calibration — each re-zeros the sensor independently of bootHeading and
//    silently invalidates the additive offset (the < 1° budget has no room to absorb it).
//    (A4 register HA-05.)
//  * bootHeading has ONE owner: the robot's canonical START heading (the init Pose2d
//    handed to the Localizer). The offset is applied exactly ONCE, here at the HAL edge;
//    no downstream consumer (odometry / Localizer / EKF) re-applies it. (A4 register HA-05.)
//
// ON-ROBOT VALIDATION still required: confirm the as-mounted sensor matches its own
// CW-positive doc strings (bench: a known +90° CW spin must DECREASE canonical heading
// by 90°). If a bench test ever disagrees, the sign of the subtraction is the line to flip.
// (A4 register HA-02, docs/hardware-assumptions.md; R3 settles it.)

#include <cmath>

#include "shulib/core/check.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::hal {

/// Convert a V5 IMU heading into a canonical field heading.
///   imuHeadingDegCw : degrees, CW-positive, measured from the calibration orientation.
///   bootHeading     : the robot's canonical field heading AT calibration (per-robot config).
/// Returns the canonical heading (CCW-positive, wrapped to (-π, π]). Non-finite input is
/// rejected by Angle::degrees (a NaN can never enter the type).
[[nodiscard]] inline math::Angle imuHeadingToCanonical(double imuHeadingDegCw,
                                                       math::Angle bootHeading) {
    // deg→rad through the sanctioned Angle boundary; subtraction flips CW→CCW; Angle wraps.
    return bootHeading - math::Angle::degrees(imuHeadingDegCw);
}

/// Convert a V5 IMU yaw rate (deg/s, CW-positive) to canonical rad/s (CCW-positive).
/// A rate does not wrap; this is the sanctioned deg/s→rad/s conversion (negate for CW→CCW).
///
/// CAVEAT (adapter): the CW-positive convention proven for heading does NOT automatically
/// extend to the rate source. PROS exposes no CW yaw-rate scalar — get_gyro_rate() returns
/// a raw body-axis struct whose z sign is UNDOCUMENTED. Prefer deriving yaw rate by
/// differentiating get_rotation() (documented CW-positive) so this negate is provably
/// correct; otherwise bench-verify get_gyro_rate().z's sign before trusting it.
/// (A4 register HA-04; R3 settles it.)
[[nodiscard]] inline units::AngularVelocity imuYawRateToCanonical(double degPerSecCw) {
    SHULIB_PRECONDITION(std::isfinite(degPerSecCw), "imuYawRateToCanonical: rate must be finite");
    constexpr double kDegToRad = math::Angle::kPi / 180.0;
    return units::AngularVelocity{-degPerSecCw * kDegToRad};
}

}  // namespace shulib::hal
