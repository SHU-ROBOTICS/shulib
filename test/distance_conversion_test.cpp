// Adversarial tests for the distance-sensor→canonical conversions (chunk R1b).
// Every expected value is a HAND-COMPUTED LITERAL — never the header's own
// constant re-imported (E2's lesson: a wrong constant must not satisfy both
// sides of the ==).

#include "doctest.h"

#include <limits>

#include "shulib/core/check.hpp"
#include "shulib/hal/distance_conversion.hpp"

using shulib::PreconditionError;
using shulib::hal::distanceConfidenceToCanonical;
using shulib::hal::distanceMmToCanonical;

TEST_CASE("distanceMmToCanonical: millimeters to inches via 1/25.4 (HA-113)") {
    // BUG CAUGHT: mutation 1's target — the mm→inch scale dropped (raw mm
    // returned as inches, a 25.4× error) or inverted (×25.4). A game piece
    // 3 in away would read 76.2 in and no capture-confirm threshold in the
    // manipulation layer would ever fire.
    // Hand-computed: 25.4 mm = 1.0 in exactly; 200 mm = 7.874015748031496 in;
    // 100 mm = 3.937007874015748 in; 1000 mm = 39.37007874015748 in.
    CHECK(distanceMmToCanonical(25.4).value() == doctest::Approx(1.0));
    CHECK(distanceMmToCanonical(200.0).value() == doctest::Approx(7.874015748031496));
    CHECK(distanceMmToCanonical(100.0).value() == doctest::Approx(3.937007874015748));
    CHECK(distanceMmToCanonical(1000.0).value() == doctest::Approx(39.37007874015748));
    CHECK(distanceMmToCanonical(0.0).value() == doctest::Approx(0.0));
}

TEST_CASE("distanceMmToCanonical: the in-band 9999 converts to ~393.66 in — plausible, which is the trap") {
    // BUG CAUGHT: nothing directly — this pins the NUMBER the T4 rule exists
    // for: 9999 mm = 393.66141732283464 in, a perfectly plausible-looking
    // reading (a far wall), NOT an obvious sentinel. The adapter test proves
    // the 9999→confidence-0 mapping; this proves WHY that mapping is the only
    // defence (the value itself cannot be recognized downstream).
    CHECK(distanceMmToCanonical(9999.0).value() == doctest::Approx(393.66141732283464));
}

TEST_CASE("distanceConfidenceToCanonical: raw 0–63 to [0,1] via ÷63 (HA-115)") {
    // BUG CAUGHT: mutation 3's target — the ÷63 dropped. Raw 63 would read
    // as 63.0, so every caller threshold ("confidence > 0.5") is satisfied
    // by ANY detection however weak, and thresholding stops meaning anything.
    // Hand-computed: 63 → 1.0; 21 → 0.3333333333333333; 42 →
    // 0.6666666666666666; 0 → 0.0.
    CHECK(distanceConfidenceToCanonical(63.0) == doctest::Approx(1.0));
    CHECK(distanceConfidenceToCanonical(21.0) == doctest::Approx(0.3333333333333333));
    CHECK(distanceConfidenceToCanonical(42.0) == doctest::Approx(0.6666666666666666));
    CHECK(distanceConfidenceToCanonical(0.0) == doctest::Approx(0.0));
}

TEST_CASE("distanceConfidenceToCanonical: out-of-range raw is clamped — the contract is unconditional") {
    // BUG CAUGHT: a firmware quirk returning 64+ (or a negative) leaking a
    // >1 confidence into callers that assume [0,1] — the defensive-clamp
    // rule the controller conversion set (clamping is defence, not policy).
    CHECK(distanceConfidenceToCanonical(64.0) == doctest::Approx(1.0));
    CHECK(distanceConfidenceToCanonical(1000.0) == doctest::Approx(1.0));
    CHECK(distanceConfidenceToCanonical(-5.0) == doctest::Approx(0.0));
}

TEST_CASE("distance conversions: non-finite is rejected by the backstop") {
    // BUG CAUGHT: a sentinel (PROS_ERR_F-shaped infinity) slipping past the
    // adapter's screen and INTO the conversion — must throw, never hand a
    // capture-confirm threshold an infinity.
    const double inf = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK_THROWS_AS((void)distanceMmToCanonical(inf), PreconditionError);
    CHECK_THROWS_AS((void)distanceConfidenceToCanonical(nan), PreconditionError);
}
