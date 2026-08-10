#pragma once
//
// frame.hpp — THE ONE PLACE a frame rotation is allowed (master plan §7 / Freeze F1).
//
// fieldToRobot / robotToField rotate a planar velocity between the FIELD frame
// and the ROBOT (body) frame by the robot's heading. The rotation rate ω is
// frame-invariant and passes through untouched.
//
// Convention (LOCKED, +X / CCW):
//   FIELD:  +X right, +Y away from the red station, heading 0 along +X, CCW-positive.
//   BODY:   +X forward, +Y left (same chirality as the field).
//   fieldToRobot applies R(-θ); robotToField applies R(+θ); they are inverses.
//
// Because `heading` is an Angle (radians internally), a degree value can never
// reach cos/sin here — the old "field-centric overwrite + degrees-into-trig"
// bug class is structurally impossible.

#include <cmath>
#include <cstdint>

#include "shulib/math/angle.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::math {

/// The two frames a chassis VELOCITY command can be expressed in — F1's
/// vocabulary, surfaced as a type at the API edge (chunk C4, for
/// `Chassis::drive(ChassisSpeeds, Frame)`). The caller must SAY which frame a
/// command is in; the parameter has no default anywhere, so silently assuming
/// the wrong frame — the classic bug class this rebuild exists to prevent — is
/// a compile error, not a heading-dependent drift.
///
/// ADDITIVE to this locked file: new vocabulary only. The F1 conventions and
/// the two rotations below are untouched.
enum class Frame : std::uint8_t {
    Field,  ///< +X right, +Y away from red, heading-independent (F1's world frame)
    Body,   ///< +X forward, +Y left — rotates with the robot (F1's robot frame)
};

/// Express a FIELD-frame chassis velocity in the ROBOT (body) frame. Applies R(-θ).
[[nodiscard]] inline ChassisSpeeds fieldToRobot(const ChassisSpeeds& field, Angle heading) noexcept {
    const double c = std::cos(heading.radians());
    const double s = std::sin(heading.radians());
    const double fx = field.vx().value();
    const double fy = field.vy().value();
    return ChassisSpeeds{units::Velocity{c * fx + s * fy},
                         units::Velocity{-s * fx + c * fy},
                         field.omega()};
}

/// Express a ROBOT (body) frame chassis velocity back in the FIELD frame. Applies R(+θ).
[[nodiscard]] inline ChassisSpeeds robotToField(const ChassisSpeeds& body, Angle heading) noexcept {
    const double c = std::cos(heading.radians());
    const double s = std::sin(heading.radians());
    const double bx = body.vx().value();
    const double by = body.vy().value();
    return ChassisSpeeds{units::Velocity{c * bx - s * by},
                         units::Velocity{s * bx + c * by},
                         body.omega()};
}

}  // namespace shulib::math
