// Tests for RobotContext — the composition root. Adversarial focus: every handle is
// non-null-validated (a null seam must throw at construction, never surface as a crash
// mid-auton), the accessors hand out the wired handles, and the M1 Definition of Done —
// "the same kinematics pipeline runs by swapping only RobotContext" — is demonstrated by
// driving field→body→wheels with the heading read THROUGH the context.

#include "doctest.h"

#include <array>
#include <numbers>
#include <span>

#include "shulib/chassis/robot_context.hpp"
#include "shulib/core/check.hpp"
#include "shulib/hal/fake/fake_battery.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_gps.hpp"
#include "shulib/hal/fake/fake_imu.hpp"
#include "shulib/hal/fake/fake_motor.hpp"
#include "shulib/hal/fake/fake_tag_source.hpp"
#include "shulib/hal/fake/fake_vision.hpp"
#include "shulib/hal/null_sink.hpp"
#include "shulib/hal/telemetry_sink.hpp"
#include "shulib/kinematics/x_drive.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/frame.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/units/quantity.hpp"

using shulib::PreconditionError;
using shulib::chassis::RobotContext;
using shulib::chassis::RobotContextConfig;
using shulib::hal::IMotor;
using shulib::hal::LogLevel;
using shulib::hal::NullSink;
using shulib::hal::fake::FakeBattery;
using shulib::hal::fake::FakeClock;
using shulib::hal::fake::FakeGps;
using shulib::hal::fake::FakeImu;
using shulib::hal::fake::FakeMotor;
using shulib::hal::fake::FakeTagSource;
using shulib::hal::fake::FakeVision;
using shulib::kinematics::xDrive;
using shulib::math::Angle;
using shulib::math::ChassisSpeeds;
using shulib::math::fieldToRobot;
using shulib::units::AngularVelocity;
using shulib::units::Length;
using shulib::units::Time;
using shulib::units::Velocity;
using shulib::units::Voltage;

namespace {
// Owns a full set of fakes + a 4-motor array, and builds a valid config referencing them.
struct Bench {
    FakeClock clock;
    FakeMotor m0, m1, m2, m3;
    std::array<IMotor*, 4> motors{&m0, &m1, &m2, &m3};
    FakeImu imu;
    FakeGps gps;
    FakeBattery battery;
    NullSink telemetry;
    FakeTagSource tags;
    FakeVision vision;

    [[nodiscard]] RobotContextConfig config() {
        return RobotContextConfig{.clock = &clock,
                                  .driveMotors = std::span<IMotor* const>{motors},
                                  .imu = &imu,
                                  .gps = &gps,
                                  .battery = &battery,
                                  .telemetry = &telemetry,
                                  .tags = &tags,
                                  .vision = &vision};
    }
};
}  // namespace

TEST_CASE("RobotContext: accessors hand out the wired handles") {
    Bench b;
    b.clock.advance(Time{0.5});
    b.imu.setHeading(Angle::degrees(30.0));
    b.battery.setVoltage(Voltage{12.3});

    RobotContext ctx{b.config()};
    CHECK(ctx.clock().now().value() == doctest::Approx(0.5));
    CHECK(ctx.imu().heading().approxEqual(Angle::degrees(30.0)));
    CHECK(static_cast<int>(ctx.driveMotors().size()) == 4);
    CHECK(ctx.battery().voltage().value() == doctest::Approx(12.3));
    CHECK(ctx.gps().hasFix() == false);              // fake default
    CHECK(ctx.tags().tags().empty());
    CHECK(ctx.vision().objects().empty());
    ctx.telemetry().log(LogLevel::Info, "TEST", "context wired");  // sink reachable, no throw
}

TEST_CASE("RobotContext: a null or empty handle is rejected at construction") {
    Bench b;

    RobotContextConfig nullImu = b.config();
    nullImu.imu = nullptr;
    CHECK_THROWS_AS((RobotContext{nullImu}), PreconditionError);

    RobotContextConfig noMotors = b.config();
    noMotors.driveMotors = {};
    CHECK_THROWS_AS((RobotContext{noMotors}), PreconditionError);

    std::array<IMotor*, 2> withNull{&b.m0, nullptr};
    RobotContextConfig nullMotor = b.config();
    nullMotor.driveMotors = std::span<IMotor* const>{withNull};
    CHECK_THROWS_AS((RobotContext{nullMotor}), PreconditionError);

    RobotContextConfig nullTelemetry = b.config();
    nullTelemetry.telemetry = nullptr;
    CHECK_THROWS_AS((RobotContext{nullTelemetry}), PreconditionError);
}

TEST_CASE("RobotContext: M1 DoD — the kinematics pipeline runs through the context") {
    Bench b;
    b.imu.setHeading(Angle::degrees(0.0));  // robot facing +X
    RobotContext ctx{b.config()};

    const auto kin = xDrive(Length{8.0});
    // the drivetrain's wheels and the context's motors agree in count/order
    CHECK(static_cast<int>(ctx.driveMotors().size()) == kin.wheelCount());

    // Drive forward (+X) at 10 in/s in the FIELD. The motion layer rotates FIELD→BODY using
    // the heading READ FROM THE CONTEXT, then kinematics produces the wheel speeds. Those
    // numbers are pure math — IDENTICAL whether the context is fake-backed (host) or
    // pros-backed (V5): swapping only RobotContext changes nothing here. That is the M1 DoD.
    const ChassisSpeeds fieldCmd{Velocity{10.0}, Velocity{0.0}, AngularVelocity{0.0}};
    const ChassisSpeeds body = fieldToRobot(fieldCmd, ctx.imu().heading());
    const auto w = kin.toWheels(body);

    const double c = 10.0 / std::numbers::sqrt2;  // V/√2 per the X-drive forward signature
    CHECK(w[0].value() == doctest::Approx(-c));    // forward pattern {-, -, +, +}·(V/√2)
    CHECK(w[1].value() == doctest::Approx(-c));
    CHECK(w[2].value() == doctest::Approx(+c));
    CHECK(w[3].value() == doctest::Approx(+c));
}
