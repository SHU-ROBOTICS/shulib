// THE frame-freeze regression tests (master plan §7 / Freeze F1).
//
// This pins the +X/CCW convention so the old "field-centric overwrite +
// degrees-into-trig" bug can never recur. Pinned cases nail the ABSOLUTE
// direction (a sign flip in one axis is caught); sweeps assert the structural
// invariants (round-trip, magnitude, ω-invariance) across all headings.

#include <cmath>
#include <random>

#include "doctest.h"
#include "shulib/math/angle.hpp"
#include "shulib/math/frame.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/units/quantity.hpp"

using shulib::math::Angle;
using shulib::math::ChassisSpeeds;
using shulib::math::fieldToRobot;
using shulib::math::robotToField;
using shulib::units::AngularVelocity;
using shulib::units::Velocity;

namespace {
constexpr double kPi = Angle::kPi;
ChassisSpeeds speeds(double vx, double vy, double w) {
    return ChassisSpeeds{Velocity{vx}, Velocity{vy}, AngularVelocity{w}};
}
}  // namespace

TEST_CASE("frame: heading 0 is the identity") {
    const ChassisSpeeds s = speeds(3.0, -4.0, 1.5);
    CHECK(fieldToRobot(s, Angle::degrees(0)).approxEqual(s));
    CHECK(robotToField(s, Angle::degrees(0)).approxEqual(s));
}

TEST_CASE("frame: pinned absolute directions (a one-axis sign flip fails these)") {
    // Robot facing +Y (heading 90° CCW). Field +X is to the robot's RIGHT (-Y body).
    CHECK(fieldToRobot(speeds(1.0, 0.0, 0.7), Angle::degrees(90)).approxEqual(speeds(0.0, -1.0, 0.7)));
    // Field +Y is straight ahead for that robot (+X body, forward).
    CHECK(fieldToRobot(speeds(0.0, 1.0, 0.0), Angle::degrees(90)).approxEqual(speeds(1.0, 0.0, 0.0)));
    // heading 180°: field +X -> body -X.
    CHECK(fieldToRobot(speeds(1.0, 0.0, 0.0), Angle::degrees(180)).approxEqual(speeds(-1.0, 0.0, 0.0)));
    // heading -90°: field +X -> body +Y.
    CHECK(fieldToRobot(speeds(1.0, 0.0, 0.0), Angle::degrees(-90)).approxEqual(speeds(0.0, 1.0, 0.0)));
}

TEST_CASE("frame: round-trip is identity for every heading and velocity") {
    std::mt19937 rng(54321u);
    std::uniform_real_distribution<double> hd(-2.0 * kPi, 2.0 * kPi);
    std::uniform_real_distribution<double> vd(-50.0, 50.0);
    for (int i = 0; i < 50000; ++i) {
        const Angle h = Angle::radians(hd(rng));
        const ChassisSpeeds s = speeds(vd(rng), vd(rng), vd(rng));
        CHECK(robotToField(fieldToRobot(s, h), h).approxEqual(s, 1e-9));
        CHECK(fieldToRobot(robotToField(s, h), h).approxEqual(s, 1e-9));
    }
}

TEST_CASE("frame: rotation preserves translational speed and leaves ω untouched") {
    std::mt19937 rng(13579u);
    std::uniform_real_distribution<double> hd(-2.0 * kPi, 2.0 * kPi);
    std::uniform_real_distribution<double> vd(-50.0, 50.0);
    for (int i = 0; i < 50000; ++i) {
        const Angle h = Angle::radians(hd(rng));
        const ChassisSpeeds s = speeds(vd(rng), vd(rng), vd(rng));
        const ChassisSpeeds b = fieldToRobot(s, h);
        const double magIn  = std::hypot(s.vx().value(), s.vy().value());
        const double magOut = std::hypot(b.vx().value(), b.vy().value());
        CHECK(magOut == doctest::Approx(magIn));            // length preserved
        CHECK(b.omega().value() == doctest::Approx(s.omega().value()));  // ω frame-invariant
    }
}

TEST_CASE("frame: fieldToRobot(s, θ) == robotToField(s, -θ)") {
    std::mt19937 rng(2468u);
    std::uniform_real_distribution<double> hd(-kPi, kPi);
    std::uniform_real_distribution<double> vd(-20.0, 20.0);
    for (int i = 0; i < 20000; ++i) {
        const double th = hd(rng);
        const ChassisSpeeds s = speeds(vd(rng), vd(rng), vd(rng));
        CHECK(fieldToRobot(s, Angle::radians(th)).approxEqual(robotToField(s, Angle::radians(-th)), 1e-9));
    }
}
