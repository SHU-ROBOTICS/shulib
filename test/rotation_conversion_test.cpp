// Adversarial tests for the rotation-sensor→canonical conversions (chunk R1a).
// Every expected value is a HAND-COMPUTED LITERAL — never the header's own
// constant re-imported (E2's lesson: a wrong constant must not satisfy both
// sides of the ==).

#include "doctest.h"

#include <limits>

#include "shulib/core/check.hpp"
#include "shulib/hal/rotation_conversion.hpp"

using shulib::PreconditionError;
using shulib::hal::rotationCentidegPerSecToCanonical;
using shulib::hal::rotationCentidegToCanonical;

TEST_CASE("rotationCentidegToCanonical: centidegrees to radians via π/18000 (HA-16)") {
    // BUG CAUGHT: mutation M3's target — the centidegree scale dropped (read
    // as plain degrees, ×100 error) or half-applied. One full revolution is
    // 36000 centideg = 2π rad; a ×100 error turns one inch of tracking-wheel
    // travel into 100 and odometry is garbage from the first tick.
    // Hand-computed: 36000 centideg → 6.283185307179586 rad;
    // 9000 → 1.5707963267948966; 100 centideg = 1° → 0.017453292519943295.
    CHECK(rotationCentidegToCanonical(36000.0).value() == doctest::Approx(6.283185307179586));
    CHECK(rotationCentidegToCanonical(9000.0).value() == doctest::Approx(1.5707963267948966));
    CHECK(rotationCentidegToCanonical(100.0).value() == doctest::Approx(0.017453292519943295));
    CHECK(rotationCentidegToCanonical(-18000.0).value() == doctest::Approx(-3.14159265358979312));
    CHECK(rotationCentidegToCanonical(0.0).value() == doctest::Approx(0.0));
}

TEST_CASE("rotationCentidegToCanonical: cumulative — multiple revolutions never wrap") {
    // BUG CAUGHT: routing position through the wrapping math::Angle — the
    // tracking wheel crosses ±180° every few inches of travel, and each wrap
    // would teleport the integrated pose by a wheel circumference.
    // Hand-computed: 108000 centideg = 3 rev = 6π = 18.84955592153876 rad.
    CHECK(rotationCentidegToCanonical(108000.0).value() == doctest::Approx(18.84955592153876));
    // Monotone through what a wrapping type would fold:
    CHECK(rotationCentidegToCanonical(19000.0).value()
          > rotationCentidegToCanonical(17000.0).value());
}

TEST_CASE("rotationCentidegPerSecToCanonical: the same one scale for the rate") {
    // BUG CAUGHT: position and velocity converted with DIFFERENT scales (a
    // hand-typo'd second constant) — the odometry stall cross-check compares
    // spin against travel, and disagreeing scales make a healthy wheel look
    // stalled (or a stalled one healthy).
    // Hand-computed: 36000 centideg/s = 1 rev/s → 6.283185307179586 rad/s.
    CHECK(rotationCentidegPerSecToCanonical(36000.0).value()
          == doctest::Approx(6.283185307179586));
    CHECK(rotationCentidegPerSecToCanonical(-9000.0).value()
          == doctest::Approx(-1.5707963267948966));
    CHECK(rotationCentidegPerSecToCanonical(0.0).value() == doctest::Approx(0.0));
}

TEST_CASE("rotation conversions: non-finite is rejected by the backstop") {
    // BUG CAUGHT: a sentinel slipping past the adapter's screen and INTO the
    // conversion — must throw, never hand the estimators an infinity.
    const double inf = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK_THROWS_AS((void)rotationCentidegToCanonical(inf), PreconditionError);
    CHECK_THROWS_AS((void)rotationCentidegPerSecToCanonical(nan), PreconditionError);
}
