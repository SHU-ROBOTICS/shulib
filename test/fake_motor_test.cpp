// Adversarial tests for FakeMotor / the IMotor contract. The behavior that matters:
// the ±12 V hardware clamp is real (an over-range command is limited, not passed
// through), non-finite commands are rejected (never reach the motor), the applied
// voltage read back is the post-clamp truth, and the encoder readbacks round-trip.

#include "doctest.h"

#include <limits>

#include "shulib/core/check.hpp"
#include "shulib/hal/fake/fake_motor.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/units/quantity.hpp"

using shulib::PreconditionError;
using shulib::hal::BrakeMode;
using shulib::hal::IMotor;
using shulib::hal::kMaxMotorVoltage;
using shulib::hal::fake::FakeMotor;
using shulib::units::AngleDim;
using shulib::units::AngularVelocity;
using shulib::units::Current;
using shulib::units::Voltage;

TEST_CASE("FakeMotor: defaults are all zero") {
    FakeMotor m;
    CHECK(m.commandedVoltage().value() == doctest::Approx(0.0));
    CHECK(m.position().value() == doctest::Approx(0.0));
    CHECK(m.velocity().value() == doctest::Approx(0.0));
}

TEST_CASE("FakeMotor: an in-range command is applied unchanged") {
    FakeMotor m;
    m.setVoltage(Voltage{5.0});
    CHECK(m.commandedVoltage().value() == doctest::Approx(5.0));
    m.setVoltage(Voltage{-7.5});
    CHECK(m.commandedVoltage().value() == doctest::Approx(-7.5));
}

TEST_CASE("FakeMotor: an over-range command is clamped to ±kMaxMotorVoltage") {
    FakeMotor m;
    m.setVoltage(Voltage{15.0});
    CHECK(m.commandedVoltage().value() == doctest::Approx(kMaxMotorVoltage.value()));   // +12
    m.setVoltage(Voltage{-20.0});
    CHECK(m.commandedVoltage().value() == doctest::Approx(-kMaxMotorVoltage.value()));  // -12
}

TEST_CASE("FakeMotor: the ±12 V boundary itself is passed through, not over-clamped") {
    FakeMotor m;
    m.setVoltage(kMaxMotorVoltage);
    CHECK(m.commandedVoltage().value() == doctest::Approx(12.0));
    m.setVoltage(Voltage{-kMaxMotorVoltage.value()});
    CHECK(m.commandedVoltage().value() == doctest::Approx(-12.0));
}

TEST_CASE("FakeMotor: a non-finite command is rejected and leaves state untouched") {
    FakeMotor m;
    m.setVoltage(Voltage{3.0});
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    CHECK_THROWS_AS(m.setVoltage(Voltage{nan}), PreconditionError);
    CHECK_THROWS_AS(m.setVoltage(Voltage{inf}), PreconditionError);
    CHECK_THROWS_AS(m.setVoltage(Voltage{-inf}), PreconditionError);
    CHECK(m.commandedVoltage().value() == doctest::Approx(3.0));  // last good command stands
}

TEST_CASE("FakeMotor: injected encoder readings round-trip") {
    FakeMotor m;
    m.setPosition(AngleDim{12.5});      // cumulative radians (NOT wrapped)
    m.setVelocity(AngularVelocity{3.0});
    CHECK(m.position().value() == doctest::Approx(12.5));   // > 2π: proves no wrap
    CHECK(m.velocity().value() == doctest::Approx(3.0));
}

TEST_CASE("FakeMotor: usable through the IMotor interface") {
    FakeMotor m;
    IMotor& motor = m;
    motor.setVoltage(Voltage{100.0});  // clamps to 12 through the interface
    CHECK(motor.commandedVoltage().value() == doctest::Approx(12.0));
}

TEST_CASE("FakeMotor: brake mode / current / temperature default and round-trip") {
    FakeMotor m;
    CHECK(m.brakeMode() == BrakeMode::Coast);  // safe default = coast
    CHECK(m.current().value() == doctest::Approx(0.0));
    CHECK(m.temperature() == doctest::Approx(0.0));

    m.setBrakeMode(BrakeMode::Hold);  // the park's active hold (≠ setVoltage(0))
    m.setCurrent(Current{2.5});       // amps draw, for grab/stall confirm + DebugRecord I
    m.setTemperature(48.0);           // °C, for the thermal monitor
    CHECK(m.brakeMode() == BrakeMode::Hold);
    CHECK(m.current().value() == doctest::Approx(2.5));
    CHECK(m.temperature() == doctest::Approx(48.0));
}
