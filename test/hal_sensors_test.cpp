// Contract tests for the simple pure-read HAL sensors (IRotation / IDistance /
// IOptical / IBattery). These have no internal logic to mutation-check — the rigor
// is the interface contract: sane defaults, exact round-trip, the documented
// semantics (e.g. IRotation::position is cumulative / non-wrapping), and usability
// through the interface. The real conversion logic lives in the hal/pros adapters.

#include "doctest.h"

#include "shulib/hal/battery.hpp"
#include "shulib/hal/distance.hpp"
#include "shulib/hal/fake/fake_battery.hpp"
#include "shulib/hal/fake/fake_distance.hpp"
#include "shulib/hal/fake/fake_optical.hpp"
#include "shulib/hal/fake/fake_rotation.hpp"
#include "shulib/hal/optical.hpp"
#include "shulib/hal/rotation.hpp"
#include "shulib/units/quantity.hpp"

using shulib::hal::IBattery;
using shulib::hal::IDistance;
using shulib::hal::IOptical;
using shulib::hal::IRotation;
using shulib::hal::fake::FakeBattery;
using shulib::hal::fake::FakeDistance;
using shulib::hal::fake::FakeOptical;
using shulib::hal::fake::FakeRotation;
using shulib::units::AngleDim;
using shulib::units::AngularVelocity;
using shulib::units::Length;
using shulib::units::Voltage;

TEST_CASE("FakeRotation: defaults zero; cumulative position round-trips WITHOUT wrapping") {
    FakeRotation r;
    CHECK(r.position().value() == doctest::Approx(0.0));
    CHECK(r.velocity().value() == doctest::Approx(0.0));

    r.setPosition(AngleDim{100.0});  // ~16 revolutions of cumulative travel
    r.setVelocity(AngularVelocity{-4.5});
    CHECK(r.position().value() == doctest::Approx(100.0));  // > 2π proves no wrap
    CHECK(r.velocity().value() == doctest::Approx(-4.5));

    const IRotation& view = r;
    CHECK(view.position().value() == doctest::Approx(100.0));
}

TEST_CASE("FakeDistance: defaults to no-confidence; values round-trip") {
    FakeDistance d;
    CHECK(d.confidence() == doctest::Approx(0.0));  // default: no usable reading

    d.setDistance(Length{6.5});
    d.setConfidence(0.9);
    CHECK(d.distance().value() == doctest::Approx(6.5));
    CHECK(d.confidence() == doctest::Approx(0.9));

    const IDistance& view = d;
    CHECK(view.distance().value() == doctest::Approx(6.5));
}

TEST_CASE("FakeOptical: channels round-trip independently") {
    FakeOptical o;
    CHECK(o.hue() == doctest::Approx(0.0));

    o.setHue(212.0);
    o.setSaturation(0.8);
    o.setBrightness(0.6);
    o.setProximity(0.95);
    CHECK(o.hue() == doctest::Approx(212.0));
    CHECK(o.saturation() == doctest::Approx(0.8));
    CHECK(o.brightness() == doctest::Approx(0.6));
    CHECK(o.proximity() == doctest::Approx(0.95));

    const IOptical& view = o;
    CHECK(view.hue() == doctest::Approx(212.0));
}

TEST_CASE("FakeBattery: voltage and capacity round-trip (drives brownout sims)") {
    FakeBattery b;
    CHECK(b.voltage().value() == doctest::Approx(0.0));

    b.setVoltage(Voltage{11.4});
    b.setCapacity(0.55);
    CHECK(b.voltage().value() == doctest::Approx(11.4));
    CHECK(b.capacity() == doctest::Approx(0.55));

    const IBattery& view = b;
    CHECK(view.voltage().value() == doctest::Approx(11.4));
}
