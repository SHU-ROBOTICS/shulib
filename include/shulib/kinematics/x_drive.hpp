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
// A4 register HA-17 (docs/hardware-assumptions.md); R3/R5 settle it.

#include <numbers>

#include "shulib/core/check.hpp"
#include "shulib/kinematics/matrix_kinematics.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::kinematics {

/// The symmetric 45° X-drive as a MatrixKinematics coefficient table: four omnis in the canonical
/// order front-left, back-left, back-right, front-right (body angles 45°, 135°, 225°, 315° CCW
/// from +X forward). `driveRadius` is the centre-to-wheel distance in INCHES and must be > 0; it
/// is the only geometry input, because every row is [±√2/2, ±√2/2, driveRadius]. Using the SAME
/// √2/2 magnitude on all four wheels makes the coefficient columns exactly orthogonal, so
/// forward() is an exact inverse rather than a least-squares fit. Consequences worth knowing at
/// the call site: strafeAuthority is 1.0 (strafe is symmetric with forward), and a forward command
/// at V asks each wheel for only V/√2. Returned BY VALUE — the caller owns it, and it must outlive
/// every IKinematics reference taken to it. Motor→index mapping and per-motor polarity are the
/// HAL/config layer's; this fixes only the canonical order.
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
