#pragma once
//
// FakeMotor — a deterministic IMotor for host tests. It applies the real contract
// (clamp to ±kMaxMotorVoltage, reject non-finite) so the clamp/validation logic is
// exercised off-robot, and it lets a test inject the position/velocity readings a
// physical encoder would produce.
//
// Since MECH-S-A (2026-11-05) it is also an hal::IMotorCurrentLimit — the motor's OWN current limit,
// set and read back (hal/motor_current_limit.hpp). The fake REMEMBERS the limit (2.5 A, the V5
// default, until set) and enforces NOTHING: a fake's current is whatever the test or the host test
// plant writes (sim::ArmPlant reads motor 0's limit and limits the joint's torque by it). Three knobs
// stand in for what a real device can do to a limit: refuse the write, answer a different number than
// was asked, or lose power (back to the 2.5 A default — A4 register HA-220); and writes are COUNTED,
// so "nothing was written" is testable.

#include <algorithm>
#include <cmath>

#include "shulib/core/check.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/hal/motor_current_limit.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::hal::fake {

/// A deterministic IMotor (and IMotorCurrentLimit) for host tests (header).
class FakeMotor final : public IMotor, public IMotorCurrentLimit {
public:
    void setVoltage(units::Voltage volts) override {
        SHULIB_PRECONDITION(std::isfinite(volts.value()),
                            "FakeMotor::setVoltage: voltage must be finite");
        commanded_ = units::Voltage{
            std::clamp(volts.value(), -kMaxMotorVoltage.value(), kMaxMotorVoltage.value())};
    }

    [[nodiscard]] units::Voltage commandedVoltage() const override { return commanded_; }

    void setBrakeMode(BrakeMode mode) override { brakeMode_ = mode; }
    [[nodiscard]] BrakeMode brakeMode() const override { return brakeMode_; }

    [[nodiscard]] units::AngleDim position() const override { return position_; }
    [[nodiscard]] units::AngularVelocity velocity() const override { return velocity_; }
    [[nodiscard]] units::Current current() const override { return current_; }
    [[nodiscard]] double temperature() const override { return temperature_; }

    // --- test injection: stand in for the encoder / current / thermal readings ---
    void setPosition(units::AngleDim p) { position_ = p; }
    void setVelocity(units::AngularVelocity v) { velocity_ = v; }
    void setCurrent(units::Current c) { current_ = c; }
    void setTemperature(double t) { temperature_ = t; }

    // --- the motor's own current limit (IMotorCurrentLimit; header) ---
    /// Remember `limit` as the device's — or, with the knobs, refuse it (false, nothing changes) or
    /// take a DIFFERENT number (the device's own answer). Counts every write that reached the device,
    /// refused ones included. PRECONDITION (the real adapter's, motorCurrentLimitToMilliamps'): `limit`
    /// finite, from 0.001 A (one milliamp) to kDefaultMotorCurrentLimit. The fake keeps the exact
    /// double; the device's whole-milliamp rounding is the PROS adapter's (pinned over the shim).
    [[nodiscard]] bool setCurrentLimit(units::Current limit) override {
        SHULIB_PRECONDITION(std::isfinite(limit.value()) && limit.value() >= 0.001 &&
                                limit.value() <= kDefaultMotorCurrentLimit.value(),
                            "FakeMotor::setCurrentLimit: the limit must be finite, from 0.001 A to 2.5 A");
        ++currentLimitWrites_;
        if (refuseCurrentLimit_) {
            return false;
        }
        limit_ = takesInstead_ > 0.0 ? units::Current{takesInstead_} : limit;
        return true;
    }
    /// The limit the "device" holds now (2.5 A until set — the V5 default). Counts every read
    /// (currentLimitReads() — how often a consumer asks the device, MECH-S-A's review).
    [[nodiscard]] units::Current currentLimit() const override {
        ++currentLimitReads_;
        return limit_;
    }

    // --- test knobs for the limit ---
    /// Every later setCurrentLimit() is refused (false; the limit unchanged) while `refuse` is true.
    void refuseCurrentLimit(bool refuse) { refuseCurrentLimit_ = refuse; }
    /// Every later accepted setCurrentLimit() leaves `amps` in the device instead of the asked value
    /// (the device's own answer — VEXos's sharing, A4 register HA-221); 0 = take what is asked.
    void currentLimitTakesInstead(double amps) { takesInstead_ = amps; }
    /// Something OTHER than the library set the device's limit (another program, VEXos): it now
    /// holds `limit`. Not a write — currentLimitWrites() does not count it.
    void setDeviceCurrentLimit(units::Current limit) { limit_ = limit; }
    /// The motor lost power and came back at its 2.5 A default (A4 register HA-220).
    void losePower() { limit_ = kDefaultMotorCurrentLimit; }
    /// How many setCurrentLimit() calls reached this motor, accepted or refused.
    [[nodiscard]] int currentLimitWrites() const { return currentLimitWrites_; }
    /// How many currentLimit() reads reached this motor (a read on a real port is a device call: the
    /// service's re-check must stay at about one per capped motor per second, never one per tick).
    [[nodiscard]] long currentLimitReads() const { return currentLimitReads_; }

private:
    units::Voltage commanded_{0.0};
    BrakeMode brakeMode_ = BrakeMode::Coast;
    units::AngleDim position_{0.0};
    units::AngularVelocity velocity_{0.0};
    units::Current current_{0.0};
    double temperature_ = 0.0;
    units::Current limit_ = kDefaultMotorCurrentLimit;
    bool refuseCurrentLimit_ = false;
    double takesInstead_ = 0.0;
    int currentLimitWrites_ = 0;
    mutable long currentLimitReads_ = 0;
};

}  // namespace shulib::hal::fake
