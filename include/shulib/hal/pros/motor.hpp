#pragma once
//
// ProsMotor — IMotor over pros::Motor (chunk R1a).
//
// BINDS:
//  * move_voltage(mV)          ← setVoltage()   (motor_conversion.hpp, HA-94)
//  * get_position() [degrees]  → position()     (motorPositionDegToCanonical, HA-95)
//  * get_actual_velocity() RPM → velocity()     (motorRpmToCanonical, HA-96)
//  * get_current_draw() mA     → current()      (motorMilliampsToCanonical, HA-97)
//  * get_temperature() °C      → temperature()  (identity — already canonical)
//  * set/get_brake_mode        ↔ setBrakeMode()/brakeMode() (mapped both ways below)
//
// WHY THE CTOR DEMANDS A GEARSET AND SETS UNITS EXPLICITLY (trap A, HA-98):
// pros::Motor's ctor defaults are MotorGears::invalid / MotorUnits::invalid,
// which mean "leave the device as it is" — get_position() then returns
// whatever encoder units that motor's firmware was last told, possibly by a
// different program on a different day. A motor left in rotations returns
// 1/360 of what this adapter believes, odometry is silently wrong by 360×,
// and nothing crashes. So this ctor:
//   1. passes an EXPLICIT gearset (no default parameter here — the cartridge
//      color is a physical fact of the built robot; velocity scaling depends
//      on it, so the adapter must not guess), and
//   2. explicitly sets MotorUnits::degrees, and
//   3. READS BOTH BACK and raises SHULIB_PRECONDITION if the device disagrees
//      — a motor that ignores configuration is a miswired robot, and finding
//      that at boot beats finding it mid-match.
//
// REVERSAL: a NEGATIVE port reverses the motor, applied by PROS itself —
// exactly once, there. This adapter never negates on top.
//
// SENTINELS (T7): IMotor has no validity channel, so PROS_ERR / PROS_ERR_F
// from a read is screened HERE: hold the last good value, never propagate
// (breaks the F4 finiteness contract A3's hostility models as a bug), never
// substitute zero (a zeroed encoder reads as "the robot stopped" — exactly
// the dead-encoder runaway the loop's ODO_STUCK cross-check exists to catch,
// and a zero would make it plausible instead of visible). A frozen position
// is what the loop's wheels-spin-but-no-motion cross-check is DESIGNED to
// see (health_monitor.hpp odomStalled); faultedReads() is exposed for
// telemetry. Raising the fault itself stays with the loop layer — hal/ is
// below diag/ and raising is policy (health_monitor.hpp header).
//
// NON-FINITE COMMAND (L4): rejected via SHULIB_PRECONDITION, exactly like
// FakeMotor (fake_motor.hpp:18-24) — NOT coerced to 0. The measurement
// prototype for this brief made that mistake; it compiles, passes a clamp
// test, and turns a programming error into a robot that coasts.
//
// HA register: HA-94..HA-98 (docs/hardware-assumptions.md).

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include "pros/error.h"
#include "pros/motors.hpp"
#pragma GCC diagnostic pop

#include <cmath>
#include <cstdint>

#include "shulib/core/check.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/hal/motor_conversion.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::hal::pros {

/// The drive cartridge in the physical motor — a construction fact the caller
/// must state (see header: velocity scaling is gearset-dependent, so there is
/// deliberately no default).
enum class MotorGearset {
    Red,    ///< 36:1, 100 RPM
    Green,  ///< 18:1, 200 RPM
    Blue,   ///< 6:1, 600 RPM
};

class ProsMotor final : public IMotor {
public:
    /// `port`: 1..21, NEGATIVE to reverse (PROS applies the reversal — once).
    /// `gearset`: the physical cartridge color. The ctor configures the device
    /// (degrees + gearset) and READS BOTH BACK — a disagreeing device raises
    /// SHULIB_PRECONDITION (header: why boot-loud beats match-silent).
    ProsMotor(std::int8_t port, MotorGearset gearset)
        : motor_{port, toProsGears(gearset), ::pros::v5::MotorUnits::degrees} {
        // Read-back (HA-98): the ctor above SET both; a healthy device echoes
        // them. `invalid` here means the port did not answer — also a failure.
        SHULIB_PRECONDITION(motor_.get_encoder_units() == ::pros::v5::MotorUnits::degrees,
                            "ProsMotor: encoder units read-back disagrees (device did not "
                            "accept degrees — wrong port, or not a motor?)");
        SHULIB_PRECONDITION(motor_.get_gearing() == toProsGears(gearset),
                            "ProsMotor: gearing read-back disagrees (device did not accept "
                            "the configured gearset — wrong port, or not a motor?)");
    }

    /// Clamp to ±kMaxMotorVoltage, REJECT non-finite (never coerce — L4), send
    /// as millivolts (HA-94). commandedVoltage() reflects the value APPLIED.
    void setVoltage(units::Voltage volts) override {
        SHULIB_PRECONDITION(std::isfinite(volts.value()),
                            "ProsMotor::setVoltage: voltage must be finite");
        commanded_ = motorVoltageApplied(volts);
        motor_.move_voltage(motorVoltageToMillivolts(volts));
    }

    [[nodiscard]] units::Voltage commandedVoltage() const override { return commanded_; }

    void setBrakeMode(BrakeMode mode) override {
        brakeMode_ = mode;
        motor_.set_brake_mode(toProsBrake(mode));
    }

    /// Reads the DEVICE back and maps to the canonical enum; an unreadable
    /// device (MotorBrake::invalid) holds the last commanded mode (T7).
    [[nodiscard]] BrakeMode brakeMode() const override {
        const auto raw = motor_.get_brake_mode();
        if (raw == ::pros::v5::MotorBrake::invalid) {
            return brakeMode_;  // screened: hold last commanded
        }
        return fromProsBrake(raw);
    }

    /// Cumulative output-shaft radians (never wrapped). Sentinel-screened:
    /// PROS_ERR_F holds the last good value (header, T7).
    [[nodiscard]] units::AngleDim position() const override {
        const double deg = motor_.get_position();
        if (!std::isfinite(deg)) {
            faultedReads_ += 1;
            return lastPosition_;
        }
        lastPosition_ = motorPositionDegToCanonical(deg);
        return lastPosition_;
    }

    [[nodiscard]] units::AngularVelocity velocity() const override {
        const double rpm = motor_.get_actual_velocity();
        if (!std::isfinite(rpm)) {
            faultedReads_ += 1;
            return lastVelocity_;
        }
        lastVelocity_ = motorRpmToCanonical(rpm);
        return lastVelocity_;
    }

    /// Canonical amperes. PROS_ERR (INT32_MAX) is IN-BAND for the int32 mA
    /// read — only this adapter can screen it (motor_conversion.hpp note).
    [[nodiscard]] units::Current current() const override {
        const std::int32_t ma = motor_.get_current_draw();
        if (ma == PROS_ERR) {
            faultedReads_ += 1;
            return lastCurrent_;
        }
        lastCurrent_ = motorMilliampsToCanonical(static_cast<double>(ma));
        return lastCurrent_;
    }

    [[nodiscard]] double temperature() const override {
        const double c = motor_.get_temperature();
        if (!std::isfinite(c)) {
            faultedReads_ += 1;
            return lastTemperature_;
        }
        lastTemperature_ = c;
        return lastTemperature_;
    }

    /// How many reads were screened to last-good (T7 observability): telemetry
    /// and the loop's health policy can see a flaky port without this seam
    /// growing a validity channel F4 does not have.
    [[nodiscard]] int faultedReads() const noexcept { return faultedReads_; }

private:
    [[nodiscard]] static ::pros::v5::MotorGears toProsGears(MotorGearset g) {
        switch (g) {
            case MotorGearset::Red: return ::pros::v5::MotorGears::red;
            case MotorGearset::Green: return ::pros::v5::MotorGears::green;
            case MotorGearset::Blue: return ::pros::v5::MotorGears::blue;
        }
        return ::pros::v5::MotorGears::green;  // unreachable; switch is exhaustive
    }

    [[nodiscard]] static ::pros::v5::MotorBrake toProsBrake(BrakeMode m) {
        switch (m) {
            case BrakeMode::Coast: return ::pros::v5::MotorBrake::coast;
            case BrakeMode::Brake: return ::pros::v5::MotorBrake::brake;
            case BrakeMode::Hold: return ::pros::v5::MotorBrake::hold;
        }
        return ::pros::v5::MotorBrake::coast;  // unreachable; switch is exhaustive
    }

    [[nodiscard]] static BrakeMode fromProsBrake(::pros::v5::MotorBrake m) {
        switch (m) {
            case ::pros::v5::MotorBrake::coast: return BrakeMode::Coast;
            case ::pros::v5::MotorBrake::brake: return BrakeMode::Brake;
            case ::pros::v5::MotorBrake::hold: return BrakeMode::Hold;
            case ::pros::v5::MotorBrake::invalid: break;
        }
        return BrakeMode::Coast;  // screened by the caller before this is reached
    }

    ::pros::v5::Motor motor_;
    units::Voltage commanded_{0.0};
    BrakeMode brakeMode_ = BrakeMode::Coast;
    // `mutable` because IMotor's readers are const but screening must remember
    // the last good value (T7's hold-last-good) — logically these caches are
    // part of the READ, not of the object's commanded state.
    mutable units::AngleDim lastPosition_{0.0};
    mutable units::AngularVelocity lastVelocity_{0.0};
    mutable units::Current lastCurrent_{0.0};
    mutable double lastTemperature_ = 20.0;
    mutable int faultedReads_ = 0;
};

}  // namespace shulib::hal::pros
