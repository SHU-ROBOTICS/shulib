#pragma once
//
// TankKinematics — a 2-wheel differential (skid-steer) drive. Body +X = right,
// +Y = forward, ω CCW-positive. The wheels drive along +Y; a commanded vx (strafe)
// is physically unachievable and is ignored — strafeAuthority() = 0.
//
// This is a DEDICATED impl, NOT a MatrixKinematics preset: tank is rank-2 (its
// strafe column is all-zero), so the holonomic engine's full-rank precondition
// correctly excludes it. The closed form (halfTrack = trackWidth/2):
//
//   left  = vy − ω·halfTrack          right = vy + ω·halfTrack
//   vy = (left + right)/2             ω    = (right − left)/(2·halfTrack)
//
// ω·halfTrack is the sanctioned radian-drop (rad/s·in → in/s); the inverse
// re-attaches it. Canonical wheel order: 0 = left, 1 = right.

#include "shulib/core/check.hpp"
#include "shulib/kinematics/desaturate.hpp"
#include "shulib/kinematics/kinematics.hpp"
#include "shulib/kinematics/wheel_speeds.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::kinematics {

class TankKinematics final : public IKinematics {
public:
    /// trackWidth = lateral distance between the left and right wheel contact lines.
    explicit TankKinematics(units::Length trackWidth) : halfTrack_{trackWidth.value() / 2.0} {
        SHULIB_PRECONDITION(halfTrack_ > 0.0, "TankKinematics: trackWidth must be > 0");
    }

    [[nodiscard]] WheelSpeeds toWheels(const math::ChassisSpeeds& body) const override {
        const double vy = body.vy().value();
        const double tangential = body.omega().value() * halfTrack_;  // rad/s·in → in/s (radian dropped)
        WheelSpeeds w{2};
        w.set(0, units::Velocity{vy - tangential});  // left
        w.set(1, units::Velocity{vy + tangential});  // right
        return w;  // vx (strafe) intentionally ignored; no clamping (§13 #5)
    }

    [[nodiscard]] math::Twist2d forward(const WheelSpeeds& wheels) const override {
        SHULIB_PRECONDITION(wheels.size() == 2, "TankKinematics::forward: expects 2 wheels");
        const double left = wheels[0].value();
        const double right = wheels[1].value();
        const double vy = (left + right) / 2.0;
        const double omega = (right - left) / (2.0 * halfTrack_);  // in/s ÷ in → rad/s (radian re-attached)
        return math::Twist2d{units::Velocity{0.0}, units::Velocity{vy}, units::AngularVelocity{omega}};
    }

    [[nodiscard]] WheelSpeeds desaturate(const WheelSpeeds& wheels,
                                         units::Velocity maxWheelSpeed) const override {
        return desaturateUniform(wheels, maxWheelSpeed);
    }

    [[nodiscard]] double strafeAuthority() const override { return 0.0; }
    [[nodiscard]] int wheelCount() const override { return 2; }

private:
    double halfTrack_;
};

}  // namespace shulib::kinematics
