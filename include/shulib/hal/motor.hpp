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

/// Behavior when the motor is at zero command. The V5 distinguishes coasting from active
/// braking/holding — voltage 0 is COAST, NOT a hold, which the guaranteed end-of-run park
/// (§M4) and holdPose/driveBrake (§M2) depend on.
enum class BrakeMode { Coast, Brake, Hold };

class IMotor {
public:
    virtual ~IMotor() = default;
    IMotor() = default;
    IMotor(const IMotor&) = default;
    IMotor(IMotor&&) = default;
    IMotor& operator=(const IMotor&) = default;
    IMotor& operator=(IMotor&&) = default;

    /// Command the motor (canonical volts). Clamped to ±kMaxMotorVoltage; non-finite rejected.
    /// VOLTAGE-ONLY by design: shulib owns the feedforward + PID + desaturation + brownout
    /// loop, not the V5's onboard velocity PID (§5 data-flow, §M2). A velocity command is
    /// deliberately not exposed.
    virtual void setVoltage(units::Voltage volts) = 0;

    /// The voltage actually applied after clamping (telemetry/test readback).
    [[nodiscard]] virtual units::Voltage commandedVoltage() const = 0;

    /// Set / read the brake mode. Hold is the active position hold the guaranteed park relies
    /// on — distinct from setVoltage(0) (= coast).
    virtual void setBrakeMode(BrakeMode mode) = 0;
    [[nodiscard]] virtual BrakeMode brakeMode() const = 0;

    /// Cumulative output-shaft rotation (NOT wrapped) — total travel for odometry.
    [[nodiscard]] virtual units::AngleDim position() const = 0;

    /// Measured output-shaft angular velocity.
    [[nodiscard]] virtual units::AngularVelocity velocity() const = 0;

    /// Measured current draw (canonical amperes). The PRIMARY capture/stall signal for
    /// manipulation sensor-confirm (M4: never advance on a failed grab), dock confirm (M3),
    /// stall homing (§8), and the per-wheel I field of the DebugRecord (§18.2). The pros
    /// adapter converts mA→A once at the edge.
    [[nodiscard]] virtual units::Current current() const = 0;

    /// Motor temperature in degrees Celsius (bare double — non-spatial, never combined with
    /// other units). Feeds the thermal monitor / thermal fault (§8, §18.4): a V5 motor
    /// throttles ~55 °C, corrupting kS/kV/kA, so the control layer must be able to see it.
    [[nodiscard]] virtual double temperature() const = 0;
};

}  // namespace shulib::hal
