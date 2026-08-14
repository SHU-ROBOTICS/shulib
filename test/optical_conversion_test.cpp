// Adversarial tests for the optical-sensor→canonical conversions (chunk R1b).
// Every expected value is a HAND-COMPUTED LITERAL — never the header's own
// constant re-imported (E2's lesson: a wrong constant must not satisfy both
// sides of the ==).

#include "doctest.h"

#include <limits>

#include "shulib/core/check.hpp"
#include "shulib/hal/optical_conversion.hpp"

using shulib::PreconditionError;
using shulib::hal::opticalHueToCanonical;
using shulib::hal::opticalProximityToCanonical;
using shulib::hal::opticalUnitIntervalToCanonical;

TEST_CASE("opticalHueToCanonical: identity inside the documented 0–359.999 range (HA-116)") {
    // BUG CAUGHT: an accidental scale (÷360 "to normalize" hue) — a color
    // COMPARISON like "hue within ±15° of red" would then compare degrees
    // against a fraction and match everything (or nothing).
    CHECK(opticalHueToCanonical(0.0) == doctest::Approx(0.0));
    CHECK(opticalHueToCanonical(120.0) == doctest::Approx(120.0));
    CHECK(opticalHueToCanonical(359.999) == doctest::Approx(359.999));
}

TEST_CASE("opticalHueToCanonical: 360.0 and beyond cannot leak through the half-open contract") {
    // BUG CAUGHT: a raw 360.0 (or an out-of-spec 400) reaching a caller that
    // trusts hue() ∈ [0, 360) — a modulo-based color window would wrap it to
    // ~0° and read a stray reflection as RED.
    CHECK(opticalHueToCanonical(360.0) == doctest::Approx(359.999));
    CHECK(opticalHueToCanonical(400.0) == doctest::Approx(359.999));
    CHECK(opticalHueToCanonical(-1.0) == doctest::Approx(0.0));
}

TEST_CASE("opticalUnitIntervalToCanonical: identity on 0–1.0 with the defensive clamp (HA-116)") {
    // BUG CAUGHT: a ÷255 wrongly applied to saturation/brightness (they are
    // ALREADY 0–1.0 per the vendored doc — the int channels are the ones
    // that scale). Saturation 0.8 would read 0.0031 and every color-confirm
    // saturation floor would reject every real object.
    CHECK(opticalUnitIntervalToCanonical(0.0) == doctest::Approx(0.0));
    CHECK(opticalUnitIntervalToCanonical(0.25) == doctest::Approx(0.25));
    CHECK(opticalUnitIntervalToCanonical(1.0) == doctest::Approx(1.0));
    CHECK(opticalUnitIntervalToCanonical(1.5) == doctest::Approx(1.0));
    CHECK(opticalUnitIntervalToCanonical(-0.5) == doctest::Approx(0.0));
}

TEST_CASE("opticalProximityToCanonical: raw 0–255 to [0,1] via ÷255 (HA-117)") {
    // BUG CAUGHT: mutation 4's target — the ÷255 dropped. Raw 51 would read
    // 51.0, so "proximity > 0.8" is true the moment anything reflects at
    // all, and capture-confirm fires on an empty intake.
    // Hand-computed: 255 → 1.0; 51 → 0.2; 204 → 0.8;
    // 128 → 0.5019607843137255.
    CHECK(opticalProximityToCanonical(255.0) == doctest::Approx(1.0));
    CHECK(opticalProximityToCanonical(51.0) == doctest::Approx(0.2));
    CHECK(opticalProximityToCanonical(204.0) == doctest::Approx(0.8));
    CHECK(opticalProximityToCanonical(128.0) == doctest::Approx(0.5019607843137255));
    CHECK(opticalProximityToCanonical(0.0) == doctest::Approx(0.0));
    // Out-of-range clamps — the contract is unconditional:
    CHECK(opticalProximityToCanonical(300.0) == doctest::Approx(1.0));
    CHECK(opticalProximityToCanonical(-10.0) == doctest::Approx(0.0));
}

TEST_CASE("optical conversions: non-finite is rejected by the backstop") {
    // BUG CAUGHT: PROS_ERR_F (INFINITY) slipping past the adapter's screen
    // and INTO a conversion — must throw, never hand color logic an infinity.
    const double inf = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK_THROWS_AS((void)opticalHueToCanonical(inf), PreconditionError);
    CHECK_THROWS_AS((void)opticalUnitIntervalToCanonical(nan), PreconditionError);
    CHECK_THROWS_AS((void)opticalProximityToCanonical(inf), PreconditionError);
}
