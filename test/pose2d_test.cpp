// Tests for the geometry value types. The thin parts (construct/accessor) get a
// round-trip; the one piece of real logic — wrap-aware pose comparison — gets an
// adversarial test that a naive (raw-subtraction) heading compare would fail.
// Compile-time checks prove the types reject the wrong units.

#include <type_traits>

#include "doctest.h"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/units/literals.hpp"
#include "shulib/units/quantity.hpp"

using shulib::math::Angle;
using shulib::math::ChassisSpeeds;
using shulib::math::Pose2d;
using shulib::math::Twist2d;
using namespace shulib::units;
using namespace shulib::units::literals;

// ----- compile-time: the types must reject wrong dimensions -----
namespace {
// Pose2d takes (Length, Length, Angle) — not a Time where a Length goes, and
// not a bare Angle where a Velocity goes, etc.
static_assert(std::is_constructible_v<Pose2d, Length, Length, Angle>);
static_assert(!std::is_constructible_v<Pose2d, Time, Length, Angle>);   // x must be a Length
static_assert(!std::is_constructible_v<Pose2d, Length, Length, double>); // heading must be an Angle
static_assert(std::is_constructible_v<Twist2d, Velocity, Velocity, AngularVelocity>);
static_assert(!std::is_constructible_v<Twist2d, Length, Velocity, AngularVelocity>); // vx must be a Velocity
// Twist2d and ChassisSpeeds are DISTINCT types (a measured twist != a command)
static_assert(!std::is_same_v<Twist2d, ChassisSpeeds>);
}  // namespace

TEST_CASE("Pose2d construct + accessor round-trip") {
    const Pose2d p{12.0_in, -6.0_in, 30.0_deg};
    CHECK(p.x().value()         == doctest::Approx(12.0));
    CHECK(p.y().value()         == doctest::Approx(-6.0));
    CHECK(p.heading().degrees() == doctest::Approx(30.0));
}

TEST_CASE("Pose2d::approxEqual is wrap-aware (the adversarial case)") {
    const Pose2d a{0.0_in, 0.0_in, Angle::degrees(179)};
    const Pose2d b{0.0_in, 0.0_in, Angle::degrees(-179)};   // only 2° away across the seam
    // naive |179 - (-179)| = 358° would say "far"; the shortest error is 2°.
    CHECK(a.approxEqual(b, 1e-6_in, 3.0 * Angle::kPi / 180.0));    // within a 3° tolerance -> equal
    CHECK_FALSE(a.approxEqual(b, 1e-6_in, 1.0 * Angle::kPi / 180.0)); // within 1° -> not equal
}

TEST_CASE("Pose2d::approxEqual catches a position difference") {
    const Pose2d a{10.0_in, 0.0_in, 0.0_deg};
    const Pose2d b{10.5_in, 0.0_in, 0.0_deg};
    CHECK_FALSE(a.approxEqual(b));                  // default µ-inch tolerance
    CHECK(a.approxEqual(b, 1.0_in));                // within an inch -> equal
}

TEST_CASE("Twist2d / ChassisSpeeds round-trip and compare") {
    const Twist2d t{Velocity{18.0}, Velocity{4.0}, AngularVelocity{0.5}};
    CHECK(t.vx().value()    == doctest::Approx(18.0));
    CHECK(t.vy().value()    == doctest::Approx(4.0));
    CHECK(t.omega().value() == doctest::Approx(0.5));
    CHECK(t.approxEqual(Twist2d{Velocity{18.0}, Velocity{4.0}, AngularVelocity{0.5}}));
    CHECK_FALSE(t.approxEqual(Twist2d{Velocity{18.0}, Velocity{4.0}, AngularVelocity{0.6}}));

    const ChassisSpeeds c{Velocity{1.0}, Velocity{2.0}, AngularVelocity{3.0}};
    CHECK(c.vy().value() == doctest::Approx(2.0));
}
