// Adapter tests for ProsMotor THROUGH THE HOST SHIM (chunk R1a). What these can
// and cannot prove: the shim tests the adapter against our BELIEF about PROS
// (HA-94..98); it cannot test the belief — hardware tests the belief. What it
// CAN prove, and the pure conversion tests cannot: that the adapter actually
// CALLS the conversions and the configuration calls (C5's D-5 hole and E1's
// sink hole, both found by mutation — wiring invisible to every pure test).

#include "doctest.h"

#include <limits>

#include "pros/shim_control.hpp"

#include "shulib/core/check.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/hal/pros/motor.hpp"
#include "shulib/units/quantity.hpp"

using shulib::PreconditionError;
using shulib::hal::BrakeMode;
using shulib::hal::pros::MotorGearset;
using shulib::hal::pros::ProsMotor;
using shulib::units::Voltage;

TEST_CASE("ProsMotor: setVoltage sends MILLIVOLTS to the wire (the ×1000 is wired in)") {
    // BUG CAUGHT (mutation M1/M14): the adapter sending volts as millivolts —
    // move_voltage(6) instead of move_voltage(6000): 1/1000 torque, the robot
    // hums. Only a wire-level assertion can see this; the conversion test
    // cannot know it was CALLED.
    pros::shim::resetAll();
    ProsMotor m{1, MotorGearset::Green};
    m.setVoltage(Voltage{6.0});
    CHECK(pros::shim::motorState(1).lastVoltageMv == 6000);
    m.setVoltage(Voltage{-3.25});
    CHECK(pros::shim::motorState(1).lastVoltageMv == -3250);
}

TEST_CASE("ProsMotor: the ±12 V clamp holds at the wire AND in commandedVoltage()") {
    // BUG CAUGHT: clamping only one of the two paths — telemetry disagreeing
    // with the wire, or a 13500 mV command reaching a device documented to
    // ±12000.
    pros::shim::resetAll();
    ProsMotor m{1, MotorGearset::Green};
    m.setVoltage(Voltage{13.5});
    CHECK(pros::shim::motorState(1).lastVoltageMv == 12000);
    CHECK(m.commandedVoltage().value() == doctest::Approx(12.0));
    m.setVoltage(Voltage{-99.0});
    CHECK(pros::shim::motorState(1).lastVoltageMv == -12000);
    CHECK(m.commandedVoltage().value() == doctest::Approx(-12.0));
}

TEST_CASE("ProsMotor: a non-finite voltage is REJECTED, not coerced to zero (L4)") {
    // BUG CAUGHT (mutation M9): silent coerce-to-zero — the measurement
    // prototype's own recorded mistake. A NaN command must throw (host policy)
    // and must NOT reach the wire as 0; the last commanded value must survive.
    pros::shim::resetAll();
    ProsMotor m{1, MotorGearset::Green};
    m.setVoltage(Voltage{5.0});
    const double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK_THROWS_AS(m.setVoltage(Voltage{nan}), PreconditionError);
    // The wire and telemetry still hold the last good command — no zero snuck in.
    CHECK(pros::shim::motorState(1).lastVoltageMv == 5000);
    CHECK(m.commandedVoltage().value() == doctest::Approx(5.0));
}

TEST_CASE("ProsMotor: ctor SETS encoder units + gearing and READS THEM BACK (trap A)") {
    // BUG CAUGHT (mutation M5): trusting pros::Motor's leave-as-is defaults —
    // the shim's adversarial port state starts at rotations/red (what "a
    // different program on a different day" left), so an adapter that skips
    // configuration reads 1/360 positions. The ctor must configure AND verify.
    pros::shim::resetAll();
    CHECK(pros::shim::motorState(4).units == pros::v5::MotorUnits::rotations);  // adversarial start
    ProsMotor m{4, MotorGearset::Blue};
    CHECK(pros::shim::motorState(4).units == pros::v5::MotorUnits::degrees);
    CHECK(pros::shim::motorState(4).gearing == pros::v5::MotorGears::blue);
    // And the position path is degree-correct end to end:
    pros::shim::motorState(4).positionOutputDeg = 180.0;
    CHECK(m.position().value() == doctest::Approx(3.14159265358979312));
}

TEST_CASE("ProsMotor: position converts output-shaft degrees → cumulative radians") {
    // BUG CAUGHT (mutation M14): the adapter reading get_position() and
    // returning it raw (degrees labeled as radians) — a 57.3× odometry error
    // that no conversion-only test can see. Hand-computed: 720° = 4π =
    // 12.566370614359172 rad — and it must NOT wrap.
    pros::shim::resetAll();
    ProsMotor m{2, MotorGearset::Green};
    pros::shim::motorState(2).positionOutputDeg = 720.0;
    CHECK(m.position().value() == doctest::Approx(12.566370614359172));
}

TEST_CASE("ProsMotor: velocity converts RPM → rad/s; current mA → A (wired, not just defined)") {
    // BUG CAUGHT (mutations M2/M14): the ÷1000 or ×2π/60 existing in the
    // header but the adapter returning raw reads. 2500 mA as 2500 A makes
    // stall detection fire on a free wheel; 200 RPM as 200 rad/s makes the
    // feedforward think the wheel is 9.5× too fast.
    pros::shim::resetAll();
    ProsMotor m{3, MotorGearset::Green};
    pros::shim::motorState(3).velocityRpm = 200.0;
    pros::shim::motorState(3).currentMa = 2500;
    pros::shim::motorState(3).temperatureC = 43.5;
    CHECK(m.velocity().value() == doctest::Approx(20.943951023931955));
    CHECK(m.current().value() == doctest::Approx(2.5));
    CHECK(m.temperature() == doctest::Approx(43.5));
}

TEST_CASE("ProsMotor: brake modes map BOTH directions through the device") {
    // BUG CAUGHT: a one-way or crossed mapping (Coast→hold is the killer: the
    // guaranteed end-of-run park relies on Hold ≠ Coast, and a crossed map
    // parks as a freewheel on an incline). Set each canonical mode, verify the
    // DEVICE saw the right PROS enum, and read it back through the adapter.
    pros::shim::resetAll();
    ProsMotor m{5, MotorGearset::Green};

    m.setBrakeMode(BrakeMode::Hold);
    CHECK(pros::shim::motorState(5).brake == pros::v5::MotorBrake::hold);
    CHECK(m.brakeMode() == BrakeMode::Hold);

    m.setBrakeMode(BrakeMode::Brake);
    CHECK(pros::shim::motorState(5).brake == pros::v5::MotorBrake::brake);
    CHECK(m.brakeMode() == BrakeMode::Brake);

    m.setBrakeMode(BrakeMode::Coast);
    CHECK(pros::shim::motorState(5).brake == pros::v5::MotorBrake::coast);
    CHECK(m.brakeMode() == BrakeMode::Coast);
}

TEST_CASE("ProsMotor: sentinel reads hold the LAST GOOD value — never propagate, never zero (T7)") {
    // BUG CAUGHT (mutation M8): a PROS_ERR_F position reaching odometry (NaN
    // pose within one tick), or a screened read substituting 0 — which reads
    // as "the robot stopped", makes the dead-encoder runaway PLAUSIBLE, and
    // defeats the ODO_STUCK cross-check designed to catch exactly that.
    pros::shim::resetAll();
    ProsMotor m{6, MotorGearset::Green};
    pros::shim::motorState(6).positionOutputDeg = 90.0;
    pros::shim::motorState(6).velocityRpm = 60.0;
    pros::shim::motorState(6).currentMa = 800;
    const double goodPos = m.position().value();
    const double goodVel = m.velocity().value();
    const double goodCur = m.current().value();
    CHECK(goodPos == doctest::Approx(1.5707963267948966));

    pros::shim::motorState(6).disconnected = true;
    CHECK(m.position().value() == doctest::Approx(goodPos));  // held, not 0, not inf
    CHECK(m.velocity().value() == doctest::Approx(goodVel));
    CHECK(m.current().value() == doctest::Approx(goodCur));
    CHECK(m.faultedReads() == 3);  // and the screen is OBSERVABLE, not silent

    // Recovery: a good read resumes live values.
    pros::shim::motorState(6).disconnected = false;
    pros::shim::motorState(6).positionOutputDeg = 180.0;
    CHECK(m.position().value() == doctest::Approx(3.14159265358979312));
}

TEST_CASE("ProsMotor: a NEGATIVE port reverses through PROS — exactly once") {
    // BUG CAUGHT: the adapter negating on top of PROS's port-sign reversal —
    // a double negation reads forward again, and the "reversed" motor of a
    // mirrored drivetrain half fights the other three.
    pros::shim::resetAll();
    ProsMotor m{-7, MotorGearset::Green};
    m.setVoltage(Voltage{6.0});
    // The shim applies the port sign itself (as PROS does): the wire shows the
    // negated command, and the adapter did NOT add a second negation.
    CHECK(pros::shim::motorState(7).lastVoltageMv == -6000);
    pros::shim::motorState(7).positionOutputDeg = 90.0;
    CHECK(m.position().value() == doctest::Approx(-1.5707963267948966));
}
