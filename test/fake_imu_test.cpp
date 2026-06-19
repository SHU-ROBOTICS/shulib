// FakeImu is a pure injectable (canonical values in, canonical values out), so its
// tests verify the round-trip, the calibration gate, and interface polymorphism —
// the real heading math lives in imu_conversion_test.

#include "doctest.h"

#include "shulib/hal/fake/fake_imu.hpp"
#include "shulib/hal/imu.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/units/quantity.hpp"

using shulib::hal::IImu;
using shulib::hal::fake::FakeImu;
using shulib::math::Angle;
using shulib::units::AngularVelocity;

TEST_CASE("FakeImu: sensible defaults") {
    FakeImu imu;
    CHECK(imu.heading().approxEqual(Angle::degrees(0.0)));
    CHECK(imu.yawRate().value() == doctest::Approx(0.0));
    CHECK(imu.isReady());  // default: calibrated/ready
    CHECK(imu.pitch().approxEqual(Angle::degrees(0.0)));
    CHECK(imu.roll().approxEqual(Angle::degrees(0.0)));
}

TEST_CASE("FakeImu: injected canonical values round-trip, including the ±180° seam") {
    FakeImu imu;
    imu.setHeading(Angle::degrees(180.0));   // seam value stays +π
    imu.setYawRate(AngularVelocity{-1.25});
    imu.setPitch(Angle::degrees(3.0));
    imu.setRoll(Angle::degrees(-2.0));
    imu.setReady(false);  // simulate still-calibrating (not yet trustworthy)

    CHECK(imu.heading().approxEqual(Angle::degrees(180.0)));
    CHECK(imu.heading().radians() > 0.0);    // +π, not -π
    CHECK(imu.yawRate().value() == doctest::Approx(-1.25));
    CHECK(imu.pitch().approxEqual(Angle::degrees(3.0)));
    CHECK(imu.roll().approxEqual(Angle::degrees(-2.0)));
    CHECK_FALSE(imu.isReady());
}

TEST_CASE("FakeImu: usable through the IImu interface") {
    FakeImu imu;
    imu.setHeading(Angle::degrees(45.0));
    const IImu& view = imu;
    CHECK(view.heading().approxEqual(Angle::degrees(45.0)));
}
