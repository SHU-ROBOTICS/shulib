// Engine-level tests for MatrixKinematics, using SYNTHETIC orthogonal tables with
// hand-computable numbers — deliberately isolated from any real drivetrain
// geometry so an engine bug and a geometry bug can never hide inside each other.
// (The X-drive preset and its physical properties are tested separately.)
//
// The table used for the round-trip is intentionally ASYMMETRIC (Σh²=4, Σv²=16,
// Σturn²=4) so that a forward() that divides by the wrong column sum is caught —
// a symmetric table would mask that bug.

#include "doctest.h"

#include <initializer_list>

#include "shulib/core/check.hpp"
#include "shulib/kinematics/matrix_kinematics.hpp"
#include "shulib/kinematics/wheel_speeds.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/units/quantity.hpp"

using shulib::PreconditionError;
using shulib::kinematics::MatrixKinematics;
using shulib::kinematics::WheelSpeeds;
using shulib::math::ChassisSpeeds;
using shulib::math::Twist2d;
using shulib::units::AngularVelocity;
using shulib::units::Velocity;

namespace {

// A valid, orthogonal, rank-3, asymmetric synthetic drive (NOT physical).
MatrixKinematics synthetic() {
    return MatrixKinematics({{-1.0, +2.0, 1.0},
                             {-1.0, -2.0, 1.0},
                             {+1.0, -2.0, 1.0},
                             {+1.0, +2.0, 1.0}},
                            0.75);
}

WheelSpeeds wheels(std::initializer_list<double> vs) {
    WheelSpeeds w{static_cast<int>(vs.size())};
    int i = 0;
    for (double v : vs) {
        w.set(i++, Velocity{v});
    }
    return w;
}

}  // namespace

TEST_CASE("MatrixKinematics: a valid table constructs and reports its shape") {
    const MatrixKinematics k = synthetic();
    CHECK(k.wheelCount() == 4);
    CHECK(k.strafeAuthority() == doctest::Approx(0.75));
}

TEST_CASE("MatrixKinematics: toWheels applies the coefficient rows exactly") {
    const MatrixKinematics k = synthetic();
    // vx=2, vy=0.5, ω=-1  →  per row  h·vx + v·vy + turn·ω
    const WheelSpeeds w = k.toWheels(ChassisSpeeds{Velocity{2.0}, Velocity{0.5}, AngularVelocity{-1.0}});
    REQUIRE(w.size() == 4);
    CHECK(w[0].value() == doctest::Approx(-2.0));  // -2 +1 -1
    CHECK(w[1].value() == doctest::Approx(-4.0));  // -2 -1 -1
    CHECK(w[2].value() == doctest::Approx(0.0));   //  2 -1 -1
    CHECK(w[3].value() == doctest::Approx(2.0));   //  2 +1 -1
}

TEST_CASE("MatrixKinematics: forward inverts the coefficient rows exactly") {
    const MatrixKinematics k = synthetic();
    const Twist2d t = k.forward(wheels({-2.0, -4.0, 0.0, 2.0}));
    CHECK(t.vx().value() == doctest::Approx(2.0));
    CHECK(t.vy().value() == doctest::Approx(0.5));
    CHECK(t.omega().value() == doctest::Approx(-1.0));
}

TEST_CASE("MatrixKinematics: forward∘toWheels is identity across a swept grid") {
    const MatrixKinematics k = synthetic();
    for (int a = -3; a <= 3; ++a) {
        for (int b = -3; b <= 3; ++b) {
            for (int c = -3; c <= 3; ++c) {
                const ChassisSpeeds cmd{Velocity{static_cast<double>(a)},
                                        Velocity{static_cast<double>(b)},
                                        AngularVelocity{static_cast<double>(c)}};
                const Twist2d back = k.forward(k.toWheels(cmd));
                const Twist2d want{Velocity{static_cast<double>(a)},
                                   Velocity{static_cast<double>(b)},
                                   AngularVelocity{static_cast<double>(c)}};
                CHECK(back.approxEqual(want, 1e-9));
            }
        }
    }
}

TEST_CASE("MatrixKinematics: desaturate delegates to the uniform scale") {
    const MatrixKinematics k = synthetic();
    const WheelSpeeds out = k.desaturate(wheels({3.0, -12.0, 6.0, 0.0}), Velocity{6.0});
    CHECK(out.maxMagnitude().value() == doctest::Approx(6.0));  // peak 12 → scaled onto 6
    CHECK(out[0].value() == doctest::Approx(1.5));
}

// --- construction preconditions: malformed tables must throw, not mis-behave ---

TEST_CASE("MatrixKinematics: an empty table is rejected") {
    CHECK_THROWS_AS(MatrixKinematics({}, 1.0), PreconditionError);
}

TEST_CASE("MatrixKinematics: a rank-deficient table (dead column) is rejected") {
    // v column all-zero → cannot strafe → not fully holonomic. (This is a tank.)
    CHECK_THROWS_AS(MatrixKinematics({{-1.0, 0.0, 1.0}, {+1.0, 0.0, 1.0}}, 0.0),
                    PreconditionError);
}

TEST_CASE("MatrixKinematics: a non-orthogonal table is rejected") {
    // h·v = 1 ≠ 0 → forward() projection would mis-invert.
    CHECK_THROWS_AS(MatrixKinematics({{1.0, 1.0, 0.0}, {1.0, 0.0, 1.0}}, 1.0),
                    PreconditionError);
}

TEST_CASE("MatrixKinematics: a negative strafeAuthority is rejected") {
    CHECK_THROWS_AS(MatrixKinematics({{-1.0, +2.0, 1.0},
                                      {-1.0, -2.0, 1.0},
                                      {+1.0, -2.0, 1.0},
                                      {+1.0, +2.0, 1.0}},
                                     -0.5),
                    PreconditionError);
}

TEST_CASE("MatrixKinematics: forward with the wrong wheel count is rejected") {
    const MatrixKinematics k = synthetic();  // expects 4
    CHECK_THROWS_AS((void)k.forward(wheels({1.0, 2.0, 3.0})), PreconditionError);
}
