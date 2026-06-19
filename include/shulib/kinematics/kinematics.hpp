#pragma once
//
// IKinematics — the drivetrain math contract. **Freeze F5** (frozen at M1).
//
// Maps a body-frame chassis twist (vx, vy, ω) to per-wheel surface speeds and
// back, plus a desaturation hook and the strafeAuthority() query. This is the
// ONLY thing the motion layer knows about a drivetrain's geometry.
//
// CONTRACT (master plan §5 data-flow, §13 #5 & #15; do not break without a
// versioned migration — that is the F5 promise):
//
//  * FRAME.  toWheels() takes a BODY-frame ChassisSpeeds. The single FIELD→BODY
//    rotation lives in Chassis, never here (F1). Kinematics is frame-agnostic
//    pure math.
//
//  * NO CLAMPING in toWheels().  It is pure inverse kinematics. Saturation is
//    handled in two SEPARATE places, on purpose (§13 #5):
//      1. upstream — the motion layer reads strafeAuthority() and clamps the
//         commanded vy *before* calling toWheels();
//      2. downstream — desaturate() applies the final wheel-speed safety scale.
//    toWheels() itself never limits anything, so odometry's forward() is an exact
//    inverse of an *unclamped* command.
//
//  * strafeAuthority() is a PURE READ-ONLY QUERY: the max sustainable |vy|/|vx|
//    a drive can hold. XDrive = 1.0 (symmetric), HDrive ≈ 0.35 (sysid-measured
//    default, not a hardcoded constant), Tank = 0.0 (cannot strafe). It computes
//    and clamps nothing.
//
//  * forward() is the inverse map (wheels → body twist) consumed by odometry. For
//    the linear drives it is the closed-form left-inverse of toWheels(); for an
//    achievable twist the round-trip forward(toWheels(t)) == t.
//
// THE HYBRID BACKEND (§13 #15, LOCKED 2026-06-19): this interface is the home for
// the nonlinear case (swerve — module angles a coefficient table can't express)
// and for the queries above. The *linear* holonomic drives (X / H / tank /
// mecanum) are a single implementation, MatrixKinematics, driven by a per-wheel
// [h, v, turn] coefficient matrix.

#include "shulib/kinematics/wheel_speeds.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::kinematics {

class IKinematics {
public:
    virtual ~IKinematics() = default;

    IKinematics() = default;
    IKinematics(const IKinematics&) = default;
    IKinematics(IKinematics&&) = default;
    IKinematics& operator=(const IKinematics&) = default;
    IKinematics& operator=(IKinematics&&) = default;

    /// Inverse kinematics: a BODY-frame commanded twist → per-wheel surface speeds.
    /// MUST NOT clamp or desaturate (§13 #5). size() of the result == wheelCount().
    [[nodiscard]] virtual WheelSpeeds toWheels(const math::ChassisSpeeds& body) const = 0;

    /// Forward kinematics: per-wheel surface speeds → BODY-frame twist (for odometry).
    /// Precondition: wheels.size() == wheelCount().
    [[nodiscard]] virtual math::Twist2d forward(const WheelSpeeds& wheels) const = 0;

    /// Direction-preserving scale so every wheel fits within `maxWheelSpeed`. If the
    /// command is already within budget it is returned unchanged. Linear drives use a
    /// uniform scale (desaturateUniform); swerve overrides to keep module angles.
    /// Precondition: maxWheelSpeed > 0.
    [[nodiscard]] virtual WheelSpeeds desaturate(const WheelSpeeds& wheels,
                                                 units::Velocity maxWheelSpeed) const = 0;

    /// PURE READ-ONLY query: max sustainable |vy|/|vx| (see contract above). Clamps nothing.
    [[nodiscard]] virtual double strafeAuthority() const = 0;

    /// Number of kinematic wheels this drivetrain exposes (the size() of toWheels()).
    [[nodiscard]] virtual int wheelCount() const = 0;
};

}  // namespace shulib::kinematics
