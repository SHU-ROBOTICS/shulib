#pragma once
//
// Motor canonical conversions — the ONE place the V5 smart motor's units become
// shulib's canonical units (§7: "convert exactly once, at the edge"). Pure,
// PROS-free functions, following the exact pattern of imu_conversion.hpp /
// gps_conversion.hpp: the unit arithmetic is host- and mutation-testable in
// isolation, and the hal/pros IMotor adapter is thin glue that CALLS these
// (never re-derives them — E2's HA-07 lesson: an obligation that lives in prose
// is an obligation nothing executes).
//
// V5 motor conventions (as read from the vendored PROS headers — every row is a
// registered belief, docs/hardware-assumptions.md):
//   * move_voltage() takes MILLIVOLTS in [-12000, 12000]  (motors.hpp:234; HA-94)
//   * get_position() returns degrees ONLY once the adapter has explicitly set
//     MotorUnits::degrees — the ctor default leaves whatever the firmware was
//     last told, possibly by a different program on a different day (HA-95/HA-98)
//   * get_actual_velocity() returns RPM                    (motors.hpp:404; HA-96)
//   * get_current_draw() returns mA                        (motors.hpp:426; HA-97)
//   * get_temperature() returns °C — identity, no conversion function needed
//
// shulib canonical (F3): volts, radians (cumulative, non-wrapping AngleDim),
// rad/s, amperes.
//
// ADAPTER BINDING CONTRACT (load-bearing, like the IMU's and GPS's):
//  * setVoltage() REJECTS a non-finite command (IMotor contract, motor.hpp:8-11)
//    BEFORE calling motorVoltageToMillivolts() — the conversion's own finiteness
//    precondition is the fail-loud backstop, NOT the rejection path. Silently
//    coercing non-finite to 0 is the exact mistake the M5 measurement prototype
//    made: it compiles, passes a clamp test, and turns a programming error into
//    a robot that coasts (brief L4).
//  * The adapter MUST set encoder units (degrees) AND gearing explicitly at
//    construction and READ BOTH BACK — pros::Motor's ctor defaults mean "leave
//    the device as it was", and a motor left in rotations reports 1/360 of what
//    this conversion assumes. Odometry is then silently wrong by 360× and
//    nothing crashes. (HA-98.)
//  * Sentinel screening (PROS_ERR / PROS_ERR_F) happens in the adapter BEFORE
//    these are called (T7: hold last good, never propagate, never zero). The
//    finiteness preconditions here catch PROS_ERR_F (= +inf) as a backstop;
//    PROS_ERR (= INT32_MAX) is in-band for integers and ONLY the adapter can
//    screen it.

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "shulib/core/check.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::hal {

/// Canonical volts → the V5's move_voltage() millivolt command, clamped to the
/// ±12 V hardware limit (kMaxMotorVoltage) and rounded to the nearest integer
/// millivolt. This is the ONE scale factor between shulib's motor command and
/// the wire (HA-94): drop it and every command is 1/1000 of the intended
/// torque — the robot hums instead of driving, and no test that checks "a
/// command was sent" can see it.
///
/// Non-finite input throws (fail-loud backstop) — but the ADAPTER must have
/// rejected it already; see the binding contract above.
[[nodiscard]] inline std::int32_t motorVoltageToMillivolts(units::Voltage volts) {
    SHULIB_PRECONDITION(std::isfinite(volts.value()),
                        "motorVoltageToMillivolts: voltage must be finite (adapter rejects first)");
    const double clamped =
        std::clamp(volts.value(), -kMaxMotorVoltage.value(), kMaxMotorVoltage.value());
    return static_cast<std::int32_t>(std::lround(clamped * 1000.0));
}

/// The voltage actually APPLIED after the ±12 V clamp (canonical volts) — the
/// value IMotor::commandedVoltage() must report, computed by the same clamp as
/// the millivolt command so telemetry can never disagree with the wire.
[[nodiscard]] inline units::Voltage motorVoltageApplied(units::Voltage volts) {
    SHULIB_PRECONDITION(std::isfinite(volts.value()),
                        "motorVoltageApplied: voltage must be finite (adapter rejects first)");
    return units::Voltage{
        std::clamp(volts.value(), -kMaxMotorVoltage.value(), kMaxMotorVoltage.value())};
}

/// Output-shaft degrees (get_position() AFTER the adapter set MotorUnits::degrees,
/// HA-95) → canonical cumulative radians. Deliberately NOT math::Angle: odometry
/// integrates total travel, so this must never wrap (motor.hpp:12-14).
[[nodiscard]] inline units::AngleDim motorPositionDegToCanonical(double positionDeg) {
    SHULIB_PRECONDITION(std::isfinite(positionDeg),
                        "motorPositionDegToCanonical: position must be finite (screen PROS_ERR_F first)");
    constexpr double kDegToRad = math::Angle::kPi / 180.0;
    return units::AngleDim{positionDeg * kDegToRad};
}

/// Output-shaft RPM (get_actual_velocity(), HA-96) → canonical rad/s.
/// 1 rev/min = 2π rad / 60 s.
[[nodiscard]] inline units::AngularVelocity motorRpmToCanonical(double rpm) {
    SHULIB_PRECONDITION(std::isfinite(rpm),
                        "motorRpmToCanonical: rpm must be finite (screen PROS_ERR_F first)");
    constexpr double kRpmToRadPerSec = 2.0 * math::Angle::kPi / 60.0;
    return units::AngularVelocity{rpm * kRpmToRadPerSec};
}

/// Milliamps (get_current_draw(), HA-97) → canonical amperes. The mA→A divide
/// happens exactly once, here (motor.hpp:61-62). Skip it and every current is
/// 1000× — stall detection fires instantly on a free-spinning motor; apply it
/// twice and it never fires at all. Takes a double because the adapter has
/// already screened the integer sentinel (PROS_ERR is in-band for int32 —
/// only the adapter can tell 2147483647 mA from a reading).
[[nodiscard]] inline units::Current motorMilliampsToCanonical(double milliamps) {
    SHULIB_PRECONDITION(std::isfinite(milliamps),
                        "motorMilliampsToCanonical: current must be finite (screen PROS_ERR first)");
    return units::Current{milliamps / 1000.0};
}

}  // namespace shulib::hal
