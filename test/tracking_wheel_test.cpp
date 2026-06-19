// Adversarial tests for TrackingWheel — the sensor→linear-travel adapter. Built to fail on a
// wrong diameter→radius factor, a cumulative-vs-incremental slip, a dropped baseline, or a sign.

#include "doctest.h"

#include "shulib/core/check.hpp"
#include "shulib/hal/fake/fake_rotation.hpp"
#include "shulib/localization/tracking_wheel.hpp"
#include "shulib/units/quantity.hpp"

using shulib::PreconditionError;
using shulib::hal::fake::FakeRotation;
using shulib::localization::TrackingWheel;
using shulib::units::AngleDim;
using shulib::units::Length;

namespace {
constexpr double kPi = 3.14159265358979323846;
}  // namespace

TEST_CASE("TrackingWheel: one full shaft revolution travels exactly one circumference") {
    FakeRotation rot;
    TrackingWheel w = TrackingWheel::forward(rot, Length{2.75}, Length{0.0});
    rot.setPosition(AngleDim{2.0 * kPi});                      // one revolution
    CHECK(w.travelDelta().value() == doctest::Approx(kPi * 2.75));  // πD, pins the ½·D radius
}

TEST_CASE("TrackingWheel: deltas are incremental, never cumulative") {
    FakeRotation rot;
    TrackingWheel w = TrackingWheel::forward(rot, Length{2.0}, Length{0.0});  // radius 1 → rad==in
    rot.setPosition(AngleDim{2.0 * kPi});
    CHECK(w.travelDelta().value() == doctest::Approx(2.0 * kPi));  // first step
    rot.setPosition(AngleDim{4.0 * kPi});
    CHECK(w.travelDelta().value() == doctest::Approx(2.0 * kPi));  // second step (not 4π)
    CHECK(w.travelDelta().value() == doctest::Approx(0.0));        // no further motion
}

TEST_CASE("TrackingWheel: reverse rotation yields negative travel") {
    FakeRotation rot;
    TrackingWheel w = TrackingWheel::forward(rot, Length{2.0}, Length{0.0});
    rot.setPosition(AngleDim{-3.0});
    CHECK(w.travelDelta().value() == doctest::Approx(-3.0));
}

TEST_CASE("TrackingWheel: offset is reported signed and unchanged") {
    FakeRotation rot;
    TrackingWheel w = TrackingWheel::forward(rot, Length{2.0}, Length{-6.5});
    CHECK(w.offset().value() == doctest::Approx(-6.5));
}

TEST_CASE("TrackingWheel: factories stamp the role (forward vs lateral)") {
    FakeRotation rot;
    CHECK(TrackingWheel::forward(rot, Length{2.0}, Length{1.0}).role() == TrackingWheel::Role::Forward);
    CHECK(TrackingWheel::lateral(rot, Length{2.0}, Length{1.0}).role() == TrackingWheel::Role::Lateral);
}

TEST_CASE("TrackingWheel: a nonzero starting position is not counted as travel") {
    FakeRotation rot;
    rot.setPosition(AngleDim{100.0});                          // sensor already spun before attach
    TrackingWheel w = TrackingWheel::forward(rot, Length{2.0}, Length{0.0});  // baseline captured here
    CHECK(w.travelDelta().value() == doctest::Approx(0.0));    // no phantom jump
    rot.setPosition(AngleDim{105.0});
    CHECK(w.travelDelta().value() == doctest::Approx(5.0));
}

TEST_CASE("TrackingWheel: reset re-baselines to the current reading") {
    FakeRotation rot;
    TrackingWheel w = TrackingWheel::forward(rot, Length{2.0}, Length{0.0});
    rot.setPosition(AngleDim{10.0});
    w.reset();                                                // baseline now 10
    CHECK(w.travelDelta().value() == doctest::Approx(0.0));
    rot.setPosition(AngleDim{12.0});
    CHECK(w.travelDelta().value() == doctest::Approx(2.0));
}

TEST_CASE("TrackingWheel: rejects a non-positive diameter") {
    FakeRotation rot;
    CHECK_THROWS_AS((void)TrackingWheel::forward(rot, Length{0.0}, Length{0.0}), PreconditionError);
    CHECK_THROWS_AS((void)TrackingWheel::lateral(rot, Length{-2.0}, Length{0.0}), PreconditionError);
}
