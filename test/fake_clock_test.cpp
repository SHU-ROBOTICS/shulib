// Adversarial tests for FakeClock — the deterministic test clock. The behavior
// that matters: it starts where told, accumulates advances exactly, and ENFORCES
// monotonicity (negative advance / backward set must throw, never silently rewind
// time and corrupt every dt downstream).

#include "doctest.h"

#include "shulib/core/check.hpp"
#include "shulib/hal/clock.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/units/quantity.hpp"

using shulib::PreconditionError;
using shulib::hal::IClock;
using shulib::hal::fake::FakeClock;
using shulib::units::Time;

TEST_CASE("FakeClock: starts at zero by default and at the given start otherwise") {
    FakeClock def;
    CHECK(def.now().value() == doctest::Approx(0.0));

    FakeClock started{Time{1.5}};
    CHECK(started.now().value() == doctest::Approx(1.5));
}

TEST_CASE("FakeClock: advances accumulate exactly") {
    FakeClock c;
    c.advance(Time{0.010});  // one 10ms tick
    c.advance(Time{0.005});
    CHECK(c.now().value() == doctest::Approx(0.015));
}

TEST_CASE("FakeClock: a zero advance is allowed and leaves time unchanged") {
    FakeClock c{Time{2.0}};
    c.advance(Time{0.0});
    CHECK(c.now().value() == doctest::Approx(2.0));
}

TEST_CASE("FakeClock: a negative advance violates monotonicity and throws") {
    FakeClock c{Time{1.0}};
    CHECK_THROWS_AS(c.advance(Time{-0.001}), PreconditionError);
    CHECK(c.now().value() == doctest::Approx(1.0));  // state untouched after the throw
}

TEST_CASE("FakeClock: set forward (or equal) is fine; set backward throws") {
    FakeClock c{Time{1.0}};
    c.set(Time{1.0});  // equal is allowed
    CHECK(c.now().value() == doctest::Approx(1.0));
    c.set(Time{3.0});  // forward
    CHECK(c.now().value() == doctest::Approx(3.0));
    CHECK_THROWS_AS(c.set(Time{2.999}), PreconditionError);  // backward
    CHECK(c.now().value() == doctest::Approx(3.0));
}

TEST_CASE("FakeClock: usable through the IClock interface") {
    FakeClock c{Time{0.25}};
    const IClock& clock = c;
    CHECK(clock.now().value() == doctest::Approx(0.25));
    c.advance(Time{0.25});
    CHECK(clock.now().value() == doctest::Approx(0.5));  // the interface sees the same clock
}
