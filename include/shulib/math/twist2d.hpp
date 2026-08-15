#pragma once
//
// Twist2d and ChassisSpeeds — the velocity currencies of the motion stack.
//
// Both carry (vx, vy, ω) with type-safe units: linear velocities are Velocity
// (in/s), the rotation rate is AngularVelocity (rad/s) — NOT an Angle, because
// a rate does not wrap. They are distinct TYPES on purpose:
//   * Twist2d       — an instantaneous pose derivative (e.g. from odometry).
//   * ChassisSpeeds  — a commanded chassis velocity (what motion asks the
//                      drivetrain to do). FRAME-AGNOSTIC: the type carries no frame, and the
//                      caller states one at every boundary that needs it — Chassis::drive
//                      takes an explicit math::Frame with no default, IKinematics::toWheels
//                      accepts a BODY-frame twist only, and frame.hpp's fieldToRobot /
//                      robotToField convert between them. This line used to say "FIELD frame
//                      until Chassis rotates it", which was true before the explicit Frame
//                      parameter and is now false of roughly half the ChassisSpeeds in the tree.
// Keeping them separate stops a measured twist being fed where a command is
// expected, and vice-versa. (master plan §5 data-flow, §6.)

#include <cmath>

#include "shulib/units/quantity.hpp"

namespace shulib::math {

/// A MEASURED instantaneous pose derivative: (vx, vy, ω) in in/s and rad/s. What an estimator or
/// a forward-kinematics call reports the robot is actually doing.
///
/// The FRAME is not carried by the type — whoever produces one states it, and both are in use:
/// `IKinematics::forward()` and the simulator's true twist are BODY frame, while
/// `IPoseSource::twist()` publishes the FIELD-frame derivative of the published pose. Read the
/// producer, never assume.
///
/// Distinct from `ChassisSpeeds` only so that a measurement cannot be passed where a command is
/// expected; there is no conversion between them, which is the point.
class Twist2d {
public:
    /// A stationary twist: all three components exactly zero.
    constexpr Twist2d() = default;
    /// Components are taken verbatim in canonical units — in/s, in/s, rad/s (CCW-positive) — and
    /// nothing is validated: a rate does not wrap, and non-finite input is the producer's problem.
    constexpr Twist2d(units::Velocity vx, units::Velocity vy, units::AngularVelocity omega) noexcept
        : vx_{vx}, vy_{vy}, w_{omega} {}

    /// Velocity along the frame's +X axis, in/s — forward in a body frame, field +X in a field one.
    [[nodiscard]] constexpr units::Velocity vx() const noexcept { return vx_; }
    /// Velocity along +Y, in/s — left in a body frame (the field shares that chirality).
    [[nodiscard]] constexpr units::Velocity vy() const noexcept { return vy_; }
    /// Rotation rate, rad/s, CCW-positive — and frame-invariant, so it survives any frame
    /// rotation untouched. An AngularVelocity rather than an Angle because a RATE never wraps.
    [[nodiscard]] constexpr units::AngularVelocity omega() const noexcept { return w_; }

    /// Component-wise ABSOLUTE comparison against a single tolerance, which is therefore read as
    /// in/s against vx/vy and as rad/s against omega — one number spanning two units, so pick it
    /// for whichever is tighter. `tol == 0` demands exact VALUE equality, which is weaker than
    /// bit-equality: -0.0 and +0.0 have different bit patterns and still compare equal, so a zero
    /// tolerance will not catch a negated stopped component. A non-finite component compares
    /// unequal to everything, itself included — inf minus inf is NaN, and NaN is `<= tol` never.
    [[nodiscard]] bool approxEqual(const Twist2d& o, double tol = 1e-9) const noexcept {
        return std::abs((vx_ - o.vx_).value()) <= tol
            && std::abs((vy_ - o.vy_).value()) <= tol
            && std::abs((w_ - o.w_).value()) <= tol;
    }

private:
    units::Velocity vx_{};
    units::Velocity vy_{};
    units::AngularVelocity w_{};
};

/// A COMMANDED chassis velocity: (vx, vy, ω) in in/s and rad/s — what motion asks the drivetrain
/// to do, before any clamp, desaturation or frame rotation.
///
/// The frame is NOT part of the type: `Chassis::drive` takes a `math::Frame` alongside it with no
/// default, `fieldToRobot`/`robotToField` map one to the other, and `IKinematics::toWheels`
/// accepts only a BODY-frame one. A ChassisSpeeds on its own therefore says nothing about which
/// way +X points; the parameter next to it does.
///
/// Nothing here clamps. Exceeding the drivetrain's capability is legal to construct and is bound
/// later, by the command pipeline (§13 #5: kinematics must never clamp).
class ChassisSpeeds {
public:
    /// A full stop: all three components exactly zero. This is what motions emit on exit.
    constexpr ChassisSpeeds() = default;
    /// Components are taken verbatim in canonical units — in/s, in/s, rad/s (CCW-positive).
    /// Unvalidated here; `Chassis::drive` is where finiteness becomes a precondition.
    constexpr ChassisSpeeds(units::Velocity vx, units::Velocity vy, units::AngularVelocity omega) noexcept
        : vx_{vx}, vy_{vy}, w_{omega} {}

    /// Commanded velocity along the frame's +X axis, in/s — forward when the frame is Body.
    [[nodiscard]] constexpr units::Velocity vx() const noexcept { return vx_; }
    /// Commanded velocity along +Y, in/s — strafe-left when the frame is Body, and the component
    /// the strafe-authority clamp bounds on a drivetrain that cannot fully honour it.
    [[nodiscard]] constexpr units::Velocity vy() const noexcept { return vy_; }
    /// Commanded rotation rate, rad/s, CCW-positive. Frame-invariant, so a Field→Body rotation
    /// passes it through unchanged.
    [[nodiscard]] constexpr units::AngularVelocity omega() const noexcept { return w_; }

    /// Component-wise ABSOLUTE comparison against a single tolerance, read as in/s against vx/vy
    /// and as rad/s against omega — one number spanning two units, so pick it for whichever is
    /// tighter. `tol == 0` demands exact VALUE equality, which is weaker than bit-equality: -0.0
    /// and +0.0 have different bit patterns and still compare equal, so a zero tolerance will not
    /// catch a desaturation that turned a commanded stop into a negative zero. A non-finite
    /// component compares unequal to everything, itself included — inf minus inf is NaN.
    [[nodiscard]] bool approxEqual(const ChassisSpeeds& o, double tol = 1e-9) const noexcept {
        return std::abs((vx_ - o.vx_).value()) <= tol
            && std::abs((vy_ - o.vy_).value()) <= tol
            && std::abs((w_ - o.w_).value()) <= tol;
    }

private:
    units::Velocity vx_{};
    units::Velocity vy_{};
    units::AngularVelocity w_{};
};

}  // namespace shulib::math
