#pragma once
//
// Twist2d and ChassisSpeeds — the velocity currencies of the motion stack.
//
// Both carry (vx, vy, ω) with type-safe units: linear velocities are Velocity
// (in/s), the rotation rate is AngularVelocity (rad/s) — NOT an Angle, because
// a rate does not wrap. They are distinct TYPES on purpose:
//   * Twist2d       — an instantaneous pose derivative (e.g. from odometry).
//   * ChassisSpeeds  — a commanded chassis velocity (what motion asks the
//                      drivetrain to do; FIELD frame until Chassis rotates it).
// Keeping them separate stops a measured twist being fed where a command is
// expected, and vice-versa. (master plan §5 data-flow, §6.)

#include <cmath>

#include "shulib/units/quantity.hpp"

namespace shulib::math {

class Twist2d {
public:
    constexpr Twist2d() = default;
    constexpr Twist2d(units::Velocity vx, units::Velocity vy, units::AngularVelocity omega) noexcept
        : vx_{vx}, vy_{vy}, w_{omega} {}

    [[nodiscard]] constexpr units::Velocity vx() const noexcept { return vx_; }
    [[nodiscard]] constexpr units::Velocity vy() const noexcept { return vy_; }
    [[nodiscard]] constexpr units::AngularVelocity omega() const noexcept { return w_; }

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

class ChassisSpeeds {
public:
    constexpr ChassisSpeeds() = default;
    constexpr ChassisSpeeds(units::Velocity vx, units::Velocity vy, units::AngularVelocity omega) noexcept
        : vx_{vx}, vy_{vy}, w_{omega} {}

    [[nodiscard]] constexpr units::Velocity vx() const noexcept { return vx_; }
    [[nodiscard]] constexpr units::Velocity vy() const noexcept { return vy_; }
    [[nodiscard]] constexpr units::AngularVelocity omega() const noexcept { return w_; }

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
