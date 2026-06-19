#pragma once
//
// IRotation — a rotation / tracking-wheel sensor (pros::Rotation) behind the HAL.
// Reports CUMULATIVE shaft rotation and angular velocity in canonical units
// (radians, rad/s). The hal/pros adapter converts centidegrees→radians and applies
// the sensor's reversed flag exactly once.
//
// position() is CUMULATIVE and must NOT wrap (it uses AngleDim, not the wrapping
// math::Angle): odometry integrates total tracking-wheel travel, not a heading.

#include "shulib/units/quantity.hpp"

namespace shulib::hal {

class IRotation {
public:
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
