#pragma once
//
// xDrive() — the symmetric 45° X-drive, as a MatrixKinematics preset (the hybrid
// backend §13 #15: a holonomic linear drive is just a coefficient table).
//
// Geometry — body frame per F1 (frame.hpp): +X = forward, +Y = left, ω CCW-positive.
// Canonical wheel order, by body angle φ measured CCW from +X (forward):
//   wheel 0: front-left  (position angle  45°)
//   wheel 1: back-left   (             135°)
//   wheel 2: back-right  (             225°)
//   wheel 3: front-right (             315°)
//
// Each omni's powered-roll direction is tangential, d̂ = (-sinφ, cosφ). The rigorous
// projection  wheel_i = d̂_i·(v_body + ω×r_i)  yields the row  [-sinφ, cosφ, R]  =
// [±c, ±c, R]  with c = √2/2 and R = driveRadius (center-to-wheel distance), where the
// first column weights vx (forward) and the second vy (left). Using the SAME magnitude
// c on every wheel makes the coefficient columns exactly orthogonal, so
// MatrixKinematics::forward() is an exact inverse.
//
// Pinned-by-test consequences of this geometry:
//   * Forward translation at V needs wheel surface speed V/√2 — i.e. the body
//     moves √2× faster forward than the wheels spin (the classic X-drive property).
//   * Pure rotation at ω drives EVERY wheel at R·ω with the same sign (all-equal =
//     spin in place — the X-drive signature).
//   * Strafe (+Y) is symmetric with forward (+X), so strafeAuthority() = 1.0.
//
// The physical motor→index mapping and per-motor polarity live in the HAL/config
// layer; kinematics only defines this canonical order.
//
// That the BUILT robot matches this idealized geometry (true 45° symmetric mounts,
// equal radii) is an A4-registered assumption until a physical drivetrain exists:
// A4 register HA-17 (docs/planning/hardware-assumptions.md); R3/R5 settle it.

#include <numbers>

#include "shulib/core/check.hpp"
#include "shulib/kinematics/matrix_kinematics.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::kinematics {

[[nodiscard]] inline MatrixKinematics xDrive(units::Length driveRadius) {
    const double r = driveRadius.value();
    SHULIB_PRECONDITION(r > 0.0, "xDrive: driveRadius must be > 0");
    const double c = std::numbers::sqrt2 / 2.0;  // √2/2, identical on all wheels → exact orthogonality
    return MatrixKinematics({{-c, +c, r},   // 0 front-left  (45°)
                             {-c, -c, r},   // 1 back-left   (135°)
                             {+c, -c, r},   // 2 back-right  (225°)
                             {+c, +c, r}},  // 3 front-right (315°)
                            1.0);
}

}  // namespace shulib::kinematics
