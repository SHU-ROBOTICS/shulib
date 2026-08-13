// Adapter tests for ProsRotation THROUGH THE HOST SHIM (chunk R1a). The shim
// tests the adapter against our belief about PROS (HA-11/16/105); hardware
// tests the belief. What only these can prove: the centidegree conversion is
// CALLED, the PROS_ERR screen works, and reversal happens exactly once.

#include "doctest.h"

#include "pros/shim_control.hpp"

#include "shulib/hal/pros/rotation.hpp"

using shulib::hal::pros::ProsRotation;

TEST_CASE("ProsRotation: position converts centidegrees → cumulative radians (wired)") {
    // BUG CAUGHT (mutations M3/M14): the π/18000 scale dropped or the raw
    // int32 returned as radians — 36000 centideg (one revolution) must read
    // 2π = 6.283185307179586, not 36000, not 100.
    pros::shim::resetAll();
    ProsRotation r{3};
    pros::shim::rotationState(3).positionCentideg = 36000;
    CHECK(r.position().value() == doctest::Approx(6.283185307179586));
    // Cumulative: 3 revolutions never wrap. 108000 → 6π = 18.84955592153876.
    pros::shim::rotationState(3).positionCentideg = 108000;
    CHECK(r.position().value() == doctest::Approx(18.84955592153876));
}

TEST_CASE("ProsRotation: velocity converts centideg/s → rad/s with the SAME scale") {
    // BUG CAUGHT: position and velocity on different scales — the odometry
    // stall cross-check (spin vs travel) would misjudge a healthy wheel.
    pros::shim::resetAll();
    ProsRotation r{3};
    pros::shim::rotationState(3).velocityCentidegPerSec = -9000;
    CHECK(r.velocity().value() == doctest::Approx(-1.5707963267948966));
}

TEST_CASE("ProsRotation: PROS_ERR holds the last good value — never propagates, never zeroes (T7)") {
    // BUG CAUGHT (mutation M8): INT32_MAX centidegrees converting to ~59652
    // revolutions of phantom travel in one tick (odometry teleports miles), or
    // a screened read substituting 0 — "the wheel stopped", the exact reading
    // that makes a dead encoder invisible to the ODO_STUCK cross-check.
    pros::shim::resetAll();
    ProsRotation r{4};
    pros::shim::rotationState(4).positionCentideg = 9000;
    const double good = r.position().value();
    CHECK(good == doctest::Approx(1.5707963267948966));

    pros::shim::rotationState(4).disconnected = true;
    CHECK(r.position().value() == doctest::Approx(good));
    CHECK(r.velocity().value() == doctest::Approx(0.0));  // last good was the initial 0
    CHECK(r.faultedReads() == 2);

    pros::shim::rotationState(4).disconnected = false;
    pros::shim::rotationState(4).positionCentideg = 18000;
    CHECK(r.position().value() == doctest::Approx(3.14159265358979312));
}

TEST_CASE("ProsRotation: a NEGATIVE port reverses through PROS — the adapter never negates on top") {
    // BUG CAUGHT: double reversal (port sign + an adapter negate) reading
    // forward again — the mirrored tracking pod would integrate travel with
    // the wrong sign and the pose would walk off at twice the real speed.
    pros::shim::resetAll();
    ProsRotation r{-5};
    pros::shim::rotationState(5).positionCentideg = 9000;
    CHECK(r.position().value() == doctest::Approx(-1.5707963267948966));
}
