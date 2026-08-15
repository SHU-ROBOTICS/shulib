// Adapter tests for ProsGps THROUGH THE HOST SHIM (chunk R1a). The shim tests
// the adapter against our belief about PROS (HA-06..08, HA-106); hardware
// tests the belief. What only these can prove: the conversions are CALLED
// (gpsToRobotPose, gpsRmsErrorToCanonical), the boot-check runs, sentinels are
// screened BEFORE conversion, and the forbidden offset paths are never touched.

#include "doctest.h"

#include "pros/shim_control.hpp"

#include "shulib/core/check.hpp"
#include "shulib/hal/pros/gps.hpp"
#include "shulib/units/quantity.hpp"

using shulib::PreconditionError;
using shulib::hal::pros::ProsGps;
using shulib::units::Length;

TEST_CASE("ProsGps: pose runs the FULL canonical conversion — meters→inches, frame, heading") {
    // BUG CAUGHT (mutations M7/M14 shape): the adapter returning raw meters,
    // or skipping gpsSensorPose's rotation. At the default northHeading=90°:
    // (1 m East, 1 m North, facing North) → canonical
    // (39.3700787401574803 in, 39.3700787401574803 in, +90°). Hand-computed
    // from the DEFINITION of the inch (0.0254 m), not from kMetersToInches.
    pros::shim::resetAll();
    ProsGps gps{7, Length{0.0}, Length{0.0}};
    pros::shim::gpsState(7).xMeters = 1.0;
    pros::shim::gpsState(7).yMeters = 1.0;
    pros::shim::gpsState(7).yawDegCwFromNorth = 0.0;
    REQUIRE(gps.hasFix());
    CHECK(gps.pose().x().value() == doctest::Approx(39.3700787401574803));
    CHECK(gps.pose().y().value() == doctest::Approx(39.3700787401574803));
    CHECK(gps.pose().heading().degrees() == doctest::Approx(90.0));
}

TEST_CASE("ProsGps: the lever arm is removed exactly ONCE (HA-06's other half)") {
    // BUG CAUGHT: the lever arm dropped (pose = sensor, constant bias) or
    // applied twice. Sensor 2″ forward of center, robot facing North
    // (canonical +90°): sensor = center + R(90°)·(2,0) = center + (0,2), so
    // center.y = sensor.y − 2. Hand-computed: 39.3700787401574803 − 2 =
    // 37.3700787401574803.
    pros::shim::resetAll();
    ProsGps gps{7, Length{2.0}, Length{0.0}};
    pros::shim::gpsState(7).xMeters = 1.0;
    pros::shim::gpsState(7).yMeters = 1.0;
    pros::shim::gpsState(7).yawDegCwFromNorth = 0.0;
    REQUIRE(gps.hasFix());
    CHECK(gps.pose().x().value() == doctest::Approx(39.3700787401574803));
    CHECK(gps.pose().y().value() == doctest::Approx(37.3700787401574803));
}

TEST_CASE("ProsGps: rmsError CALLS gpsRmsErrorToCanonical — meters scale to inches (HA-07)") {
    // BUG CAUGHT (mutation M7): get_error()'s meters passed straight through —
    // the corrector's R is ~39× too small, good fixes get gated out, and the
    // GPS goes silently dead (E2 built the function to prevent exactly this).
    // Hand-computed: 0.023 m × (1 in / 0.0254 m) = 0.905511811023622 in.
    pros::shim::resetAll();
    ProsGps gps{8, Length{0.0}, Length{0.0}};
    pros::shim::gpsState(8).errorMeters = 0.023;
    REQUIRE(gps.hasFix());
    CHECK(gps.rmsError().value() == doctest::Approx(0.905511811023622));
}

TEST_CASE("ProsGps: boot-check — a preconfigured firmware offset REFUSES to construct (HA-06)") {
    // BUG CAUGHT (mutation M6): skipping the get_offset()==(0,0) check — a
    // device another program configured makes get_position() report the
    // CENTER, this adapter subtracts the lever arm AGAIN, and every fix
    // carries inches of heading-dependent bias. Loud at boot, not biased at
    // match.
    pros::shim::resetAll();
    pros::shim::gpsState(9).offsetX = 0.05;  // 5 cm of firmware offset left behind
    CHECK_THROWS_AS((ProsGps{9, Length{0.0}, Length{0.0}}), PreconditionError);
}

TEST_CASE("ProsGps: sentinels screen to hasFix()==false BEFORE conversion — pose never throws (HA-08)") {
    // BUG CAUGHT (mutation M8): PROS_ERR_F fed into gpsSensorPose(), which
    // THROWS by design (the fail-loud backstop) — an unscreened adapter turns
    // every off-strip tick (all of Driving Skills!) into a crash. Screened:
    // no-fix + last good pose held, finite, no throw.
    pros::shim::resetAll();
    ProsGps gps{10, Length{0.0}, Length{0.0}};
    pros::shim::gpsState(10).xMeters = 0.5;
    pros::shim::gpsState(10).yMeters = 0.0;
    REQUIRE(gps.hasFix());
    const double goodX = gps.pose().x().value();
    CHECK(goodX == doctest::Approx(19.68503937007874));  // 0.5 m in inches, hand-computed

    pros::shim::gpsState(10).noFix = true;  // off the strip
    CHECK_FALSE(gps.hasFix());
    CHECK_NOTHROW((void)gps.pose());
    CHECK(gps.pose().x().value() == doctest::Approx(goodX));  // stale-finite, held
    CHECK(gps.faultedReads() > 0);

    pros::shim::gpsState(10).noFix = false;  // back on the strip
    CHECK(gps.hasFix());
}

TEST_CASE("ProsGps: never touches set_offset/initialize_full/offset-ctors (the forbidden paths)") {
    // BUG CAUGHT: any "helpful" configuration call sneaking into the adapter —
    // the shim counts every forbidden path; after construction and a full
    // read cycle the count must be zero.
    pros::shim::resetAll();
    ProsGps gps{11, Length{1.0}, Length{-2.0}};
    (void)gps.pose();
    (void)gps.rmsError();
    (void)gps.hasFix();
    CHECK(pros::shim::gpsState(11).forbiddenConfigCalls == 0);
}

TEST_CASE("ProsGps: unreadable-at-boot defers the check; a bad offset found later = dead GPS, no throw") {
    // BUG CAUGHT: the deferred re-verify throwing from pose()/hasFix() (the
    // MUST-NOT-THROW read paths — this chunk's own first design had exactly
    // that bug), or an unverifiable device being trusted anyway.
    pros::shim::resetAll();
    pros::shim::gpsState(12).noFix = true;  // device can't be read at construction
    pros::shim::gpsState(12).offsetX = 0.05;  // and it is secretly configured
    ProsGps gps{12, Length{0.0}, Length{0.0}};  // constructs fine — check deferred
    CHECK_FALSE(gps.hasFix());  // unverified → never trusted

    pros::shim::gpsState(12).noFix = false;  // device wakes up, offset now readable
    CHECK_NOTHROW((void)gps.hasFix());
    CHECK_FALSE(gps.hasFix());  // bad offset discovered → permanently no-fix, honestly dead
    CHECK_NOTHROW((void)gps.pose());
}

// Bug caught (DEFECTS1 item D5): refresh() has two screen-to-no-fix paths and only one
// counted. Once verifyOffset() sets offsetRejected_ — the deferred discovery of a configured
// firmware offset — the device is no-fix for the ENTIRE run, and faultedReads() stayed at 0
// forever. An operator using the counter to answer "why is the GPS dead?" got the least
// informative possible answer in exactly the case the header's HA-06 discussion cares most
// about: the one failure this class treats as permanent was the one it reported zero times.
TEST_CASE("D5: a permanently offset-rejected GPS is VISIBLE in faultedReads()") {
    pros::shim::resetAll();
    // The deferred path, which is the only way to reach offsetRejected_: the offset is
    // UNREADABLE at construction (so the boot check defers instead of throwing), and readable
    // and NON-ZERO afterwards — a device another program configured, discovered late.
    pros::shim::gpsState(9).noFix = true;
    ProsGps gps{9, Length{0.0}, Length{0.0}};
    pros::shim::gpsState(9).noFix = false;
    pros::shim::gpsState(9).offsetX = 3.0;

    (void)gps.hasFix();
    (void)gps.pose();
    CHECK_FALSE(gps.hasFix());       // permanently untrusted, for the whole run
    CHECK(gps.faultedReads() > 0);   // was 0 FOREVER — the defect

    // NEGATIVE CONTROL: a healthy GPS with no configured offset counts nothing at all, so the
    // count above is the screen firing and not merely "this class counts everything".
    pros::shim::resetAll();
    ProsGps clean{10, Length{0.0}, Length{0.0}};
    (void)clean.hasFix();
    (void)clean.pose();
    CHECK(clean.faultedReads() == 0);
}
