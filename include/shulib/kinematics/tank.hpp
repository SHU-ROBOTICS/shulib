#pragma once
//
// TankKinematics — a 2-wheel differential (skid-steer) drive. Body frame per F1
// (frame.hpp): +X = FORWARD, +Y = left, ω CCW-positive. The wheels drive along +X
// (forward); a commanded vy (strafe) is physically unachievable and is ignored —
// strafeAuthority() = 0 (= max sustainable |vy|/|vx|, §13 #5).
//
// This is a DEDICATED impl, NOT a MatrixKinematics preset: tank is rank-2 (its
// strafe column is all-zero), so the holonomic engine's full-rank precondition
// correctly excludes it. The closed form (halfTrack = trackWidth/2):
//
//   left  = vx − ω·halfTrack          right = vx + ω·halfTrack
//   vx = (left + right)/2             ω    = (right − left)/(2·halfTrack)
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

/// A 2-wheel differential (skid-steer) drive. Canonical wheel order is 0 = LEFT, 1 = RIGHT, in
/// F1's body frame (+X forward, +Y left, ω CCW-positive). Deliberately a hand-written
/// IKinematics rather than a MatrixKinematics preset: tank's strafe column is all-zero, so it is
/// rank-2 and the holonomic engine's full-rank precondition correctly refuses it. Pure geometry —
/// no state, no clock, no HAL; the only thing it knows is the track width.
class TankKinematics final : public IKinematics {
public:
    /// trackWidth = lateral distance between the left and right wheel contact lines.
    explicit TankKinematics(units::Length trackWidth) : halfTrack_{trackWidth.value() / 2.0} {
        SHULIB_PRECONDITION(halfTrack_ > 0.0, "TankKinematics: trackWidth must be > 0");
    }

    /// Inverse kinematics: left = vx − ω·halfTrack, right = vx + ω·halfTrack, both in in/s, from
    /// a BODY-frame twist. The commanded vy is SILENTLY IGNORED — not clamped, not an error —
    /// because a tank cannot strafe and strafeAuthority() == 0 is how the motion layer is meant to
    /// have zeroed it already. Nothing here limits speed (§13 #5); that is desaturate()'s job.
    [[nodiscard]] WheelSpeeds toWheels(const math::ChassisSpeeds& body) const override {
        const double vx = body.vx().value();                          // +X = forward (F1)
        const double tangential = body.omega().value() * halfTrack_;  // rad/s·in → in/s (radian dropped)
        WheelSpeeds w{2};
        w.set(0, units::Velocity{vx - tangential});  // left
        w.set(1, units::Velocity{vx + tangential});  // right
        return w;  // vy (strafe) intentionally ignored; no clamping (§13 #5)
    }

    /// Forward kinematics for odometry: vx = (left + right)/2, ω = (right − left)/(2·halfTrack),
    /// and vy ALWAYS exactly 0 — this drivetrain cannot observe lateral motion, so a real skid
    /// sideways is reported as no motion at all. Exact left-inverse of toWheels() on the
    /// achievable (vx, ω) subspace, so forward(toWheels(t)) returns t only when t's vy was
    /// already 0. Precondition: exactly 2 wheels, in the canonical left-then-right order.
    [[nodiscard]] math::Twist2d forward(const WheelSpeeds& wheels) const override {
        SHULIB_PRECONDITION(wheels.size() == 2, "TankKinematics::forward: expects 2 wheels");
        const double left = wheels[0].value();
        const double right = wheels[1].value();
        const double vx = (left + right) / 2.0;                                // forward
        const double omega = (right - left) / (2.0 * halfTrack_);  // in/s ÷ in → rad/s (radian re-attached)
        return math::Twist2d{units::Velocity{vx}, units::Velocity{0.0}, units::AngularVelocity{omega}};
    }

    /// The uniform scale (desaturateUniform): if either wheel is over `maxWheelSpeed`, BOTH are
    /// scaled by the same factor, which preserves left:right and therefore the arc the command
    /// describes — the robot follows the same curve, more slowly. Returned unchanged when already
    /// within budget; never scales up. Precondition: maxWheelSpeed > 0.
    [[nodiscard]] WheelSpeeds desaturate(const WheelSpeeds& wheels,
                                         units::Velocity maxWheelSpeed) const override {
        return desaturateUniform(wheels, maxWheelSpeed);
    }

    /// Always 0: no lateral authority whatever, so the motion layer's
    /// |body vy| ≤ strafeAuthority()·maxLinearSpeed clamp reduces to "vy must be 0" here.
    [[nodiscard]] double strafeAuthority() const override { return 0.0; }

    /// Always 2 — the left and right KINEMATIC wheels. How many physical motors sit on each side
    /// is the HAL's business; kinematics sees one speed per side.
    [[nodiscard]] int wheelCount() const override { return 2; }

private:
    double halfTrack_;
};

}  // namespace shulib::kinematics
