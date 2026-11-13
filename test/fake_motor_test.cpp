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

// ═══ MECH-S-A: the motor's own CURRENT LIMIT (hal::IMotorCurrentLimit) on the fake ═══════════════
// What a test leans on: it REMEMBERS (2.5 A until set — the V5 default), it counts every write that
// reached it (refused ones too), and its three knobs do what a real device can: refuse, answer another
// number, lose power. And it refuses a limit no build could have stated (precondition).

TEST_CASE("MECH-S-A FakeMotor: the limit starts at the V5's 2.5 A, is remembered, and every write is COUNTED") {
    FakeMotor m;
    CHECK(m.currentLimit().value() == doctest::Approx(2.5));
    CHECK(m.currentLimitWrites() == 0);
    CHECK(m.setCurrentLimit(Current{1.2}));
    CHECK(m.currentLimit().value() == 1.2);  // exactly what was asked
    CHECK(m.currentLimitWrites() == 1);
    CHECK(m.setCurrentLimit(Current{2.5}));  // the default exactly: allowed
    CHECK(m.currentLimit().value() == 2.5);
    CHECK(m.currentLimitWrites() == 2);
    // the limit is not the reading: the fake enforces nothing (the plant does)
    m.setCurrent(Current{2.4});
    CHECK(m.current().value() == doctest::Approx(2.4));
}

TEST_CASE("MECH-S-A FakeMotor: the knobs -- refuse (false, unchanged, still counted), another number, lose power, set by someone else") {
    FakeMotor m;
    REQUIRE(m.setCurrentLimit(Current{1.0}));
    m.refuseCurrentLimit(true);
    CHECK_FALSE(m.setCurrentLimit(Current{1.5}));
    CHECK(m.currentLimit().value() == 1.0);   // BUG CAUGHT: a refused write that still changed the device
    CHECK(m.currentLimitWrites() == 2);       // it REACHED the device
    m.refuseCurrentLimit(false);
    m.currentLimitTakesInstead(2.0);
    CHECK(m.setCurrentLimit(Current{1.2}));   // accepted ...
    CHECK(m.currentLimit().value() == 2.0);   // ... but the device holds its own number (HA-221)
    m.currentLimitTakesInstead(0.0);
    REQUIRE(m.setCurrentLimit(Current{1.2}));
    m.losePower();
    CHECK(m.currentLimit().value() == 2.5);   // back at the default (HA-220)
    m.setDeviceCurrentLimit(Current{1.8});
    CHECK(m.currentLimit().value() == 1.8);
    CHECK(m.currentLimitWrites() == 4);       // neither a power loss nor another's setting is a write
}

TEST_CASE("MECH-S-A FakeMotor: a limit no build could state is a PRECONDITION -- 0, under one milliamp, negative, past 2.5 A, NaN, infinity") {
    FakeMotor m;
    for (const double bad : {0.0, 0.0009, -1.0, 2.5000001, 3.0, std::numeric_limits<double>::quiet_NaN(),
                             std::numeric_limits<double>::infinity()}) {
        CAPTURE(bad);
        CHECK_THROWS_AS((void)m.setCurrentLimit(Current{bad}), PreconditionError);
    }
    CHECK(m.currentLimitWrites() == 0);  // refused before it reached the "device"
    CHECK(m.currentLimit().value() == 2.5);
    CHECK(m.setCurrentLimit(Current{0.001}));  // one milliamp, the smallest a build can state
}

TEST_CASE("MECH-S-A FakeMotor: every currentLimit() READ is counted (the re-check's pace is a test's to see); a write is not a read") {
    FakeMotor m;
    CHECK(m.currentLimitReads() == 0);
    (void)m.currentLimit();
    (void)m.currentLimit();
    CHECK(m.currentLimitReads() == 2);
    REQUIRE(m.setCurrentLimit(Current{1.2}));
    m.losePower();
    m.setDeviceCurrentLimit(Current{1.8});
    CHECK(m.currentLimitReads() == 2);  // a write, a power loss and another's setting read nothing
    const FakeMotor& asConst = m;
    (void)asConst.currentLimit();       // through a const reference too (the interface's read is const)
    CHECK(m.currentLimitReads() == 3);
}
