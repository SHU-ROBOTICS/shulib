// Adapter tests for ProsDistance THROUGH THE HOST SHIM (chunk R1b). The shim
// tests the adapter against our belief about PROS (HA-113/114/115); hardware
// tests the belief. What only these can prove: the mm→inch conversion is
// CALLED, the 9999 rule fires, the close-range confidence rule fires, and the
// PROS_ERR screen works.

#include "doctest.h"

#include "pros/shim_control.hpp"

#include "shulib/hal/pros/distance.hpp"

using shulib::hal::pros::ProsDistance;

TEST_CASE("ProsDistance: distance converts mm → inches (wired)") {
    // BUG CAUGHT (mutations 1/5): the 1/25.4 scale dropped, or the adapter
    // computing the conversion and returning the raw value anyway — 254 mm
    // must read 10.0 in, not 254, not 6451.6.
    pros::shim::resetAll();
    ProsDistance d{7};
    pros::shim::distanceState(7).distanceMm = 254;
    CHECK(d.distance().value() == doctest::Approx(10.0));
    pros::shim::distanceState(7).distanceMm = 1000;
    CHECK(d.distance().value() == doctest::Approx(39.37007874015748));
}

TEST_CASE("ProsDistance: THE 9999 RULE — no object maps to confidence 0, distance stays finite-far (T4)") {
    // BUG CAUGHT (mutation 2 — the most important in the chunk): 9999 passed
    // through as a real reading with nonzero confidence. The shim's DEFAULT
    // state is exactly the trap: an empty intake (9999) with a HIGH raw
    // confidence (63). An adapter that trusts confidence without checking
    // the distance hands dock-confirm a wall 33 feet away and calls it real.
    pros::shim::resetAll();
    ProsDistance d{7};
    // Shim defaults: distanceMm = 9999, confidenceRaw = 63 (adversarial).
    CHECK(d.confidence() == doctest::Approx(0.0));
    // The distance itself stays FINITE (F4 finiteness) and FAR — the honest
    // conversion of 9999, failing every proximity threshold:
    CHECK(d.distance().value() == doctest::Approx(393.66141732283464));
    // And it is a READING, not a fault — an empty intake is a normal state:
    CHECK(d.faultedReads() == 0);
}

TEST_CASE("ProsDistance: confidence converts 0–63 → [0,1] above 200 mm (wired)") {
    // BUG CAUGHT (mutations 3/5): the ÷63 dropped (raw 21 reads 21.0 — every
    // threshold saturates) or computed-then-discarded.
    pros::shim::resetAll();
    ProsDistance d{7};
    pros::shim::distanceState(7).distanceMm = 500;  // > 200 mm: raw channel live
    pros::shim::distanceState(7).confidenceRaw = 21;
    CHECK(d.confidence() == doctest::Approx(0.3333333333333333));
    pros::shim::distanceState(7).confidenceRaw = 63;
    CHECK(d.confidence() == doctest::Approx(1.0));
}

TEST_CASE("ProsDistance: at or below 200 mm a returned distance IS the detection — confidence 1.0") {
    // BUG CAUGHT: the close-range rule missing — PROS documents confidence
    // as "only available when distance is > 200mm" (HA-115), so passing the
    // undefined raw channel through at 100 mm could read "object touching
    // the sensor, zero confidence" and a capture-confirm would REFUSE the
    // grab it most needs to confirm. The shim poisons the below-200 channel
    // with 0 (its adversarial default) so exactly that mistake goes red.
    pros::shim::resetAll();
    ProsDistance d{7};
    pros::shim::distanceState(7).distanceMm = 100;  // close — and detected
    // (shim's confidenceBelow200Raw default = 0: the poisoned channel)
    CHECK(d.confidence() == doctest::Approx(1.0));
    CHECK(d.distance().value() == doctest::Approx(3.937007874015748));
    // Boundary: 200 mm itself is inside the undocumented range ("> 200mm"):
    pros::shim::distanceState(7).distanceMm = 200;
    CHECK(d.confidence() == doctest::Approx(1.0));
    // Just past the boundary the raw channel is live again:
    pros::shim::distanceState(7).distanceMm = 201;
    pros::shim::distanceState(7).confidenceRaw = 42;
    CHECK(d.confidence() == doctest::Approx(0.6666666666666666));
}

TEST_CASE("ProsDistance: PROS_ERR (device failure) holds last good distance, reports no confidence (T7)") {
    // BUG CAUGHT (mutation 6): the sentinel screen removed — INT32_MAX mm
    // converts to ~84.5 million inches of phantom object, or (screened to
    // zero) "an object touching the sensor", the dangerous direction for
    // capture logic. Device failure is DISTINCT from 9999: it counts in
    // faultedReads().
    pros::shim::resetAll();
    ProsDistance d{9};
    pros::shim::distanceState(9).distanceMm = 254;
    CHECK(d.distance().value() == doctest::Approx(10.0));

    pros::shim::distanceState(9).disconnected = true;
    CHECK(d.distance().value() == doctest::Approx(10.0));  // held, not INT32_MAX
    CHECK(d.confidence() == doctest::Approx(0.0));          // no usable return
    CHECK(d.faultedReads() == 2);

    pros::shim::distanceState(9).disconnected = false;
    pros::shim::distanceState(9).distanceMm = 508;
    CHECK(d.distance().value() == doctest::Approx(20.0));
}

TEST_CASE("ProsDistance: dead from boot reads FAR, never 'object touching the sensor'") {
    // BUG CAUGHT: a last-good hold initialized to 0.0 — a sensor unplugged
    // since boot would read 0 inches = "object pressed against the sensor",
    // and a capture-confirm thresholding distance alone would confirm a grab
    // that never happened. The initial hold must be the far no-object value.
    pros::shim::resetAll();
    ProsDistance d{11};
    pros::shim::distanceState(11).disconnected = true;
    CHECK(d.distance().value() == doctest::Approx(393.66141732283464));
    CHECK(d.confidence() == doctest::Approx(0.0));
}

// Bug caught (DEFECTS1 item I8): the T7 initial hold spelled HA-114's sentinel and the mm→inch
// scale BY HAND (`9999.0 / 25.4`) while the file's own named constant and shared converter —
// kDistanceNoObjectMm and distanceMmToCanonical(), from an already-included header — were used
// two accessors above. The magic number and the scale lived in two places, so a change to
// either would have moved the screen and left the initial hold behind. This pins that they
// agree, which is the property the duplication put at risk.
TEST_CASE("I8: the boot hold is the SAME no-object value the screen recognises") {
    pros::shim::resetAll();
    pros::shim::distanceState(11).disconnected = true;   // dead from boot: never a good read
    ProsDistance dead{11};
    const double heldInches = dead.distance().value();

    // Derived from the named constant through the shared converter — NOT re-spelled here.
    const double expected =
        shulib::hal::distanceMmToCanonical(
            static_cast<double>(shulib::hal::kDistanceNoObjectMm)).value();
    CHECK(heldInches == doctest::Approx(expected));
    CHECK(heldInches > 300.0);        // far, not 0.0: never "object touching the lens"
    CHECK(dead.faultedReads() == 1);

    // NEGATIVE CONTROL: a live sensor REPORTING the sentinel converts to the same value, which
    // is what makes "the boot hold is the no-object reading" true rather than coincidental.
    pros::shim::resetAll();
    pros::shim::distanceState(12).distanceMm = shulib::hal::kDistanceNoObjectMm;
    ProsDistance live{12};
    CHECK(live.distance().value() == doctest::Approx(expected));
    CHECK(live.confidence() == doctest::Approx(0.0));
}
