// FakeGps is a pure injectable; the real frame math lives in gps_conversion_test.
// These verify defaults (safe: no fix), round-trip, and interface polymorphism.

#include "doctest.h"

#include <cmath>

#include "shulib/hal/fake/fake_gps.hpp"
#include "shulib/hal/gps.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/units/quantity.hpp"

using shulib::hal::IGps;
using shulib::hal::fake::FakeGps;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::units::Length;

TEST_CASE("FakeGps: default is a safe no-fix state") {
    FakeGps gps;
    CHECK_FALSE(gps.hasFix());  // must not be trusted until told otherwise
    CHECK(gps.rmsError().value() == doctest::Approx(0.0));
    CHECK(gps.pose().approxEqual(Pose2d{}));
}

TEST_CASE("FakeGps: injected state round-trips") {
    FakeGps gps;
    const Pose2d p{Length{24.0}, Length{-12.0}, Angle::degrees(135.0)};
    gps.setPose(p);
    gps.setRmsError(Length{0.5});
    gps.setHasFix(true);

    CHECK(gps.pose().approxEqual(p));
    CHECK(gps.rmsError().value() == doctest::Approx(0.5));
    CHECK(gps.hasFix());
}

TEST_CASE("FakeGps: a no-fix pose() is finite and does not throw (dead-reckon contract)") {
    FakeGps gps;
    gps.setHasFix(false);
    const Pose2d p = gps.pose();  // safe to read even with no fix; the fuser ignores it
    CHECK(std::isfinite(p.x().value()));
    CHECK(std::isfinite(p.y().value()));
    CHECK(std::isfinite(p.heading().radians()));
}

TEST_CASE("FakeGps: usable through the IGps interface") {
    FakeGps gps;
    gps.setHasFix(true);
    gps.setPose(Pose2d{Length{1.0}, Length{2.0}, Angle::degrees(0.0)});
    const IGps& view = gps;
    CHECK(view.hasFix());
    CHECK(view.pose().x().value() == doctest::Approx(1.0));
}
