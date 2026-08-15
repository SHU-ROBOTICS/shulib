#pragma once
//
// IRotation — a rotation / tracking-wheel sensor (pros::Rotation) behind the HAL.
// Reports CUMULATIVE shaft rotation and angular velocity in canonical units
// (radians, rad/s). The hal/pros adapter converts centidegrees→radians exactly
// once and never negates: reversal is PROS's own, from a negative port number.
//
// position() is CUMULATIVE and must NOT wrap (it uses AngleDim, not the wrapping
// math::Angle): odometry integrates total tracking-wheel travel, not a heading.

#include "shulib/units/quantity.hpp"

namespace shulib::hal {

/// A rotation / tracking-wheel sensor behind the HAL, in canonical units (radians, rad/s). The
/// PROS adapter converts centidegrees to radians exactly once and NEVER negates: reversal is
/// applied once by PROS itself, decided by the SIGN of the port number the sensor is constructed
/// on, so a consumer re-applies neither: there is no `reversed` boolean anywhere in this API, and
/// the port number is the only place direction is chosen. This seam reports SHAFT rotation, not
/// distance: turning it into travel needs a wheel diameter, localization::TrackingWheel's job.
/// There is NO validity channel here: an implementation must always return a finite, plausible
/// value, so the PROS adapter screens the in-band PROS_ERR sentinel by holding the last good
/// reading rather than propagating it or zeroing. A frozen reading, not a zeroed one, is what
/// the loop's stuck-odometry cross-check is built to notice.
class IRotation {
public:
    /// All defaulted, and what is worth knowing here is the lifetime rather than the language
    /// rule: an implementation is REFERENCED and never owned — a TrackingWheel holds an
    /// `hal::IRotation&` and latches its travel baseline off it at construction — so the sensor
    /// object must outlive every wheel built on it, and every odometry built on those. The
    /// destructor is virtual only so that owning one through an `IRotation*` would still be
    /// well-defined; declaring it is what forces the copy and move members to be re-defaulted.
    virtual ~IRotation() = default;
    IRotation() = default;
    IRotation(const IRotation&) = default;
    IRotation(IRotation&&) = default;
    IRotation& operator=(const IRotation&) = default;
    IRotation& operator=(IRotation&&) = default;

    /// Cumulative shaft rotation (NOT wrapped) — total travel for odometry.
    [[nodiscard]] virtual units::AngleDim position() const = 0;

    /// Measured shaft angular velocity.
    [[nodiscard]] virtual units::AngularVelocity velocity() const = 0;
};

}  // namespace shulib::hal
