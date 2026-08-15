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

/// The V5's own voltage ceiling (canonical volts). setVoltage() clamps to ±this, so it is the
/// FINAL hardware bound — applied after, and never as a substitute for, the upstream wheel-speed
/// desaturation that decides which wheel gives way when the kinematics ask for more than exists.
inline constexpr units::Voltage kMaxMotorVoltage{12.0};  // V5 hard limit (volts)

/// Behavior when the motor is at zero command. The V5 distinguishes coasting from active
/// braking/holding — voltage 0 is COAST, NOT a hold, which the guaranteed end-of-run park
/// (§M4) and holdPose/driveBrake (§M2) depend on.
enum class BrakeMode {
    /// Undriven and unresisted — the shaft free-wheels. This is where setVoltage(0) alone
    /// leaves a motor, which is why the guaranteed park sets a MODE instead of commanding zero.
    Coast,
    /// Electronically brakes the shaft to a stop and resists being turned, but does not remember
    /// where it stopped: shove the robot afterwards and it stays shoved.
    Brake,
    /// Actively drives back to the position the shaft held when the command went to zero — the
    /// only mode that DEFENDS a pose. holdPose/driveBrake and the guaranteed park are built on it.
    Hold
};

/// One V5 smart motor behind the HAL. The control layer hands it a canonical VOLTAGE and reads
/// back cumulative shaft rotation, speed, current and temperature — everything above this seam is
/// volts / radians / rad/s / amperes / °C, with the V5's native units converted away in the
/// adapter. An implementation holds no control policy of its own: on the COMMAND path it makes
/// exactly two judgements (clamp to ±kMaxMotorVoltage, reject a non-finite volts). The READ path
/// is NOT guaranteed to be a straight pass-through — this seam has no validity channel, so an
/// adapter over real hardware screens an unreadable device instead. ProsMotor does: a sentinel
/// read returns the LAST GOOD position/velocity/current/temperature and increments its
/// faultedReads() count, and temperature() is seeded at 20 °C, so a port that never answered at
/// all reports a room-temperature motor rather than an error. A reading taken from this seam can
/// therefore be FROZEN rather than fresh, which is precisely what the loop's
/// wheels-spin-but-no-motion cross-check (health_monitor.hpp, odomStalled) exists to catch.
class IMotor {
public:
    /// Virtual so an owner that really does hold a motor polymorphically can destroy it — but
    /// nothing in shulib is that owner: RobotContext, MotorMechanism and the motion ops all reach
    /// motors through a NON-OWNING `std::span<IMotor* const>` and never delete through it. The
    /// CALLER owns the concrete motors, and they must outlive every component they were handed
    /// to — in practice, the whole run. The copy/move members are re-defaulted (a user-declared
    /// destructor suppresses the implicit MOVEs and deprecates the implicit copies) purely so
    /// this base imposes no policy: it holds no state, and what a copy means belongs to the
    /// implementation — copying a ProsMotor duplicates a handle to the SAME physical port,
    /// along with a second, independently-ageing copy of its last-good read cache.
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
    /// The mode currently in effect. Not guaranteed to be a pure echo of setBrakeMode(): an
    /// adapter over real hardware reads the DEVICE back (ProsMotor does, falling back to the last
    /// commanded mode only when the port does not answer), so this is the motor's answer rather
    /// than the library's memory of the request.
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
