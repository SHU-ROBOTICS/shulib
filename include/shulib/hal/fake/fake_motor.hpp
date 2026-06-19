#pragma once
//
// FakeMotor — a deterministic IMotor for host tests. It applies the real contract
// (clamp to ±kMaxMotorVoltage, reject non-finite) so the clamp/validation logic is
// exercised off-robot, and it lets a test inject the position/velocity readings a
// physical encoder would produce.

#include <algorithm>
#include <cmath>

#include "shulib/core/check.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::hal::fake {

class FakeMotor final : public IMotor {
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

private:
    units::Voltage commanded_{0.0};
    BrakeMode brakeMode_ = BrakeMode::Coast;
    units::AngleDim position_{0.0};
    units::AngularVelocity velocity_{0.0};
    units::Current current_{0.0};
    double temperature_ = 0.0;
};

}  // namespace shulib::hal::fake
