#pragma once
//
// Teleop stick mapping — the ONE place a driver's sticks become a body-frame
// drive request (chunk R3b, Session 2). THIS IS THE SEAM CHUNK T2 OWNS: T2
// (input shaping, drive modes, the feel properties) replaces the BODIES below
// with real driver feel — a continuous deadband, monotonic curves, optional
// slew, field-centric with its honest fallback — behind these same names, so
// every caller (the library teleop loop in src/main.cpp and the bench tester's
// DRIVE station) keeps feeling one robot.
//
// WHAT IT IS TODAY: the R1a loop's mapping, extracted VERBATIM from
// src/main.cpp and pinned bit-for-bit by test/stick_mapping_test.cpp against
// a copy of that original code. Deadband only, at 0.05 (HA-112: INVENTED —
// the deadband exists so a centred stick's ±2-count noise cannot creep the
// robot, and nothing about it is a measurement). The naive deadband here is
// DISCONTINUOUS (output jumps from 0 to 0.05 as the stick crosses the
// threshold) — chunk T2 names that as its first property to fix, and it
// is left exactly as it was on purpose: this chunk changes nothing about how
// the robot drives, it only moves where the mapping lives.
//
// AXIS CONVENTION (locked frame F1, body frame, CCW-positive):
//   * left stick pushed UP     → +forward  (+X body)
//   * left stick pushed LEFT   → +left     (+Y body)   — hal's LeftX is + when pushed RIGHT,
//                                                         so the raw axis is NEGATED first
//   * right stick pushed RIGHT → NEGATIVE yaw (clockwise) — again a negation of RightX
//   * controller not connected → all three exactly 0.0 (HA-103: a disconnected controller
//                                 reads 0 on every channel, so isConnected() is the ONLY
//                                 thing that separates "driver unplugged" from "sticks centred")
//
// ORDER OF OPERATIONS IS LOAD-BEARING for bit-identity: the raw axis is negated
// FIRST and the deadband applied SECOND, exactly as the R1a loop wrote
// `shaped(-axis)`. Deadband-then-negate gives the same VALUES but a different
// ZERO (−0.0 instead of +0.0 inside the deadband), and the test compares bits.
//
// PURE and PROS-free: takes numbers, returns numbers, no state, no clock, no
// preconditions — finiteness becomes a precondition where it always did, at
// Chassis::drive (chassis.hpp) and at the motor adapter. Host-tested in
// isolation; the CI guard keeps <pros/…> out of this tree.

#include "shulib/math/twist2d.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::teleop {

/// The stick deadband, as a fraction of full deflection (canonical [-1, 1] axes). INVENTED
/// (A4: HA-112) — it exists only so a centred stick's few counts of noise cannot creep the
/// robot. Applied as a hard cut (see deadbanded()); T2 owns making it continuous.
inline constexpr double kStickDeadband = 0.05;

/// What the driver's hands are doing, in the form the mapping needs: the three canonical
/// [-1, 1] axes it reads (IController::axis for LeftY / LeftX / RightX) and the controller's
/// POSITIVE validity signal (IController::isConnected). Built by the caller from an
/// IController each tick; a struct rather than an IController& so the mapping stays a pure
/// function of numbers and is testable without a fake controller.
struct StickInput {
    double leftY = 0.0;      ///< Left stick, vertical: + = pushed UP (hal::ControllerAxis::LeftY).
    double leftX = 0.0;      ///< Left stick, horizontal: + = pushed RIGHT (hal::ControllerAxis::LeftX).
    double rightX = 0.0;     ///< Right stick, horizontal: + = pushed RIGHT (hal::ControllerAxis::RightX).
    bool connected = false;  ///< IController::isConnected() — false forces a zero request.
};

/// The mapped drive request, DIMENSIONLESS: body-frame forward, left and CCW-yaw fractions in
/// [-1, 1] with the deadband and the axis signs already applied, all exactly 0.0 while the
/// controller is disconnected. Multiply by a speed budget (toChassisSpeeds) for the library
/// teleop loop, or by a voltage ceiling for an open-loop bench drive — the same request feeds
/// both, which is the point of having one.
struct DriveRequest {
    double forward = 0.0;  ///< +X body fraction: left stick UP is positive.
    double left = 0.0;     ///< +Y body fraction: left stick pushed LEFT is positive.
    double yawCcw = 0.0;   ///< CCW-positive yaw fraction: right stick pushed RIGHT is NEGATIVE (clockwise).
};

/// The R1a deadband, verbatim: an axis strictly inside (−kStickDeadband, +kStickDeadband)
/// becomes exactly 0.0; anything else — the threshold value itself included — passes through
/// untouched. Discontinuous at the threshold by construction (T2's first property to fix).
[[nodiscard]] constexpr double deadbanded(double axis) noexcept {
    return (axis > -kStickDeadband && axis < kStickDeadband) ? 0.0 : axis;
}

/// Sticks → dimensionless request. A disconnected controller yields the all-zero request no
/// matter what the axes read (HA-103). Otherwise: forward = deadbanded(leftY),
/// left = deadbanded(−leftX), yawCcw = deadbanded(−rightX) — NEGATE FIRST, then deadband,
/// exactly the original loop's `shaped(-axis)` (header: the order decides the sign of zero).
[[nodiscard]] constexpr DriveRequest mapSticks(const StickInput& in) noexcept {
    if (!in.connected) {
        return DriveRequest{};
    }
    return DriveRequest{deadbanded(in.leftY), deadbanded(-in.leftX), deadbanded(-in.rightX)};
}

/// Request × budgets → the body-frame ChassisSpeeds that Chassis::drive(…, Frame::Body)
/// takes: (forward·maxLinear, left·maxLinear, yawCcw·maxAngular). The budgets are the caller's
/// MotionConfig::maxLinearSpeed / maxAngularSpeed (HA-50, provisional) — passed in rather than
/// read here so this stays a pure function of its arguments.
[[nodiscard]] constexpr math::ChassisSpeeds toChassisSpeeds(const DriveRequest& request,
                                                            units::Velocity maxLinear,
                                                            units::AngularVelocity maxAngular)
    noexcept {
    return math::ChassisSpeeds{request.forward * maxLinear, request.left * maxLinear,
                               request.yawCcw * maxAngular};
}

/// The whole mapping in one call — what the library teleop loop uses each tick. Bit-identical
/// to the R1a loop it replaced: a disconnected controller returns a default-constructed
/// ChassisSpeeds (never 0·budget, so the answer does not depend on the budgets' values), and a
/// connected one returns toChassisSpeeds(mapSticks(in), …).
[[nodiscard]] constexpr math::ChassisSpeeds mapSticksToChassisSpeeds(
    const StickInput& in, units::Velocity maxLinear, units::AngularVelocity maxAngular) noexcept {
    if (!in.connected) {
        return math::ChassisSpeeds{};
    }
    return toChassisSpeeds(mapSticks(in), maxLinear, maxAngular);
}

}  // namespace shulib::teleop
