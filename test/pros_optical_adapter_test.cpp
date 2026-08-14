// Adapter tests for ProsOptical THROUGH THE HOST SHIM (chunk R1b). The shim
// tests the adapter against our belief about PROS (HA-116/117/118); hardware
// tests the belief — including proximity's larger-is-closer polarity, which
// the vendored doc does NOT state (HA-117). What only these can prove: each
// channel's conversion is CALLED and each sentinel screen works.

#include "doctest.h"

#include "pros/shim_control.hpp"

#include "shulib/hal/pros/optical.hpp"

using shulib::hal::pros::ProsOptical;

TEST_CASE("ProsOptical: hue/saturation/brightness pass through (wired, no stray scale)") {
    // BUG CAUGHT (mutation 5's shape): an accidental ÷360 or ÷255 on the
    // channels PROS already reports in canonical ranges — hue 208° would
    // read 0.578 and a "within ±15° of blue" window would never match.
    pros::shim::resetAll();
    ProsOptical o{8};
    pros::shim::opticalState(8).hue = 208.5;
    pros::shim::opticalState(8).saturation = 0.62;
    pros::shim::opticalState(8).brightness = 0.31;
    CHECK(o.hue() == doctest::Approx(208.5));
    CHECK(o.saturation() == doctest::Approx(0.62));
    CHECK(o.brightness() == doctest::Approx(0.31));
}

TEST_CASE("ProsOptical: proximity converts 0–255 → [0,1] (wired)") {
    // BUG CAUGHT (mutations 4/5): the ÷255 dropped (raw 51 reads 51.0 and
    // every proximity threshold fires on an empty intake) or the conversion
    // computed and the raw value returned anyway.
    pros::shim::resetAll();
    ProsOptical o{8};
    pros::shim::opticalState(8).proximity = 51;
    CHECK(o.proximity() == doctest::Approx(0.2));
    pros::shim::opticalState(8).proximity = 255;
    CHECK(o.proximity() == doctest::Approx(1.0));
    pros::shim::opticalState(8).proximity = 128;
    CHECK(o.proximity() == doctest::Approx(0.5019607843137255));
}

TEST_CASE("ProsOptical: PROS_ERR_F on the double channels holds last good — never an infinity (T7)") {
    // BUG CAUGHT (mutation 6's family): the sentinel screen removed on a
    // seam with NO validity channel — INFINITY through hue() breaks the F4
    // finiteness contract, and a screen-to-zero would read hue 0.0 = RED, a
    // confident wrong answer to "what color is this game piece".
    pros::shim::resetAll();
    ProsOptical o{6};
    pros::shim::opticalState(6).hue = 120.0;
    pros::shim::opticalState(6).saturation = 0.8;
    pros::shim::opticalState(6).brightness = 0.5;
    CHECK(o.hue() == doctest::Approx(120.0));
    CHECK(o.saturation() == doctest::Approx(0.8));
    CHECK(o.brightness() == doctest::Approx(0.5));

    pros::shim::opticalState(6).disconnected = true;
    CHECK(o.hue() == doctest::Approx(120.0));         // held — green, not "red"
    CHECK(o.saturation() == doctest::Approx(0.8));
    CHECK(o.brightness() == doctest::Approx(0.5));
    CHECK(o.faultedReads() == 3);
}

TEST_CASE("ProsOptical: PROS_ERR on the proximity channel holds last good (T7 — the int32 sentinel)") {
    // BUG CAUGHT: screening only the PROS_ERR_F channels — proximity's
    // sentinel is the INT32 one, and unscreened INT32_MAX ÷ 255 ≈ 8.4
    // million clamps to 1.0 = "object touching the sensor": a dead sensor
    // would CONFIRM every capture forever.
    pros::shim::resetAll();
    ProsOptical o{6};
    pros::shim::opticalState(6).proximity = 51;
    CHECK(o.proximity() == doctest::Approx(0.2));

    pros::shim::opticalState(6).disconnected = true;
    CHECK(o.proximity() == doctest::Approx(0.2));  // held — not 1.0
    CHECK(o.faultedReads() == 1);

    pros::shim::opticalState(6).disconnected = false;
    pros::shim::opticalState(6).proximity = 204;
    CHECK(o.proximity() == doctest::Approx(0.8));
}
