#pragma once
//
// IMotor — a single V5 smart motor behind the HAL. The control layer commands a
// canonical voltage; the motor reports its output-shaft rotation and angular
// velocity for odometry/telemetry. Units are canonical: volts, radians, rad/s (F3).
//
// Contract:
//  * setVoltage() CLAMPS the command to ±kMaxMotorVoltage (the V5's hard limit) and
//    REJECTS a non-finite voltage (red-on-failure host-side). commandedVoltage()
//    returns the value actually APPLIED after clamping, so telemetry reflects what
//    the motor really got — not an impossible request.
//  * position() is CUMULATIVE output-shaft rotation and must NOT wrap at ±π —
//    odometry integrates total travel, so it uses the non-wrapping AngleDim, never
//    the wrapping math::Angle.
//
// Wheel-speed saturation is handled upstream (desaturate, §13 #5); this ±12 V clamp
// is a separate, final HARDWARE limit, not a substitute for it.

#include "shulib/units/quantity.hpp"

namespace shulib::hal {

inline constexpr units::Voltage kMaxMotorVoltage{12.0};  // V5 hard limit (volts)

class IMotor {
public:
    virtual ~IMotor() = default;
    IMotor() = default;
    IMotor(const IMotor&) = default;
    IMotor(IMotor&&) = default;
    IMotor& operator=(const IMotor&) = default;
    IMotor& operator=(IMotor&&) = default;

    /// Command the motor (canonical volts). Clamped to ±kMaxMotorVoltage; non-finite rejected.
    virtual void setVoltage(units::Voltage volts) = 0;

    /// The voltage actually applied after clamping (telemetry/test readback).
    [[nodiscard]] virtual units::Voltage commandedVoltage() const = 0;

    /// Cumulative output-shaft rotation (NOT wrapped) — total travel for odometry.
    [[nodiscard]] virtual units::AngleDim position() const = 0;

    /// Measured output-shaft angular velocity.
    [[nodiscard]] virtual units::AngularVelocity velocity() const = 0;
};

}  // namespace shulib::hal
