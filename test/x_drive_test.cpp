// Physical-geometry tests for the X-drive preset. These pin the *signature* of an
// X-drive — the patterns that would silently break if a coefficient sign or the
// √2 factor were wrong — not just that "some numbers come out".

#include "doctest.h"

#include <cmath>
#include <numbers>

#include "shulib/core/check.hpp"
#include "shulib/kinematics/matrix_kinematics.hpp"
#include "shulib/kinematics/wheel_speeds.hpp"
#include "shulib/kinematics/x_drive.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/units/quantity.hpp"

using shulib::PreconditionError;
using shulib::kinematics::MatrixKinematics;
using shulib::kinematics::WheelSpeeds;
using shulib::kinematics::xDrive;
using shulib::math::ChassisSpeeds;
using shulib::math::Twist2d;
using shulib::units::AngularVelocity;
using shulib::units::Length;
using shulib::units::Velocity;

namespace {
constexpr double kR = 10.0;                  // drive radius, inches
const double kInvSqrt2 = std::numbers::sqrt2 / 2.0;  // √2/2 ≈ 0.7071
}  // namespace

TEST_CASE("xDrive: shape and strafe authority") {
    const MatrixKinematics k = xDrive(Length{kR});
    CHECK(k.wheelCount() == 4);
    CHECK(k.strafeAuthority() == doctest::Approx(1.0));  // symmetric drive
}

TEST_CASE("xDrive: a positive driveRadius is required") {
    CHECK_THROWS_AS(xDrive(Length{0.0}), PreconditionError);
    CHECK_THROWS_AS(xDrive(Length{-5.0}), PreconditionError);
}

TEST_CASE("xDrive: pure forward (+X=vx) — wheels spin at V/√2 (body is √2× faster than wheels)") {
    const MatrixKinematics k = xDrive(Length{kR});
    const double V = 10.0;
    // forward is +X = vx (F1).
    const WheelSpeeds w = k.toWheels(ChassisSpeeds{Velocity{V}, Velocity{0.0}, AngularVelocity{0.0}});

    // forward (h-column) pattern {-, -, +, +}·(V/√2): each magnitude is V/√2, not V.
    CHECK(w[0].value() == doctest::Approx(-V * kInvSqrt2));
    CHECK(w[1].value() == doctest::Approx(-V * kInvSqrt2));
    CHECK(w[2].value() == doctest::Approx(+V * kInvSqrt2));
    CHECK(w[3].value() == doctest::Approx(+V * kInvSqrt2));
    // the √2 property, stated directly: peak wheel speed × √2 == body forward speed.
    CHECK(w.maxMagnitude().value() * std::numbers::sqrt2 == doctest::Approx(V));
}

TEST_CASE("xDrive: pure strafe (+Y=vy) — symmetric with forward, pattern {+,-,-,+}") {
    const MatrixKinematics k = xDrive(Length{kR});
    const double V = 8.0;
    // strafe is +Y = vy (F1: +Y left).
    const WheelSpeeds w = k.toWheels(ChassisSpeeds{Velocity{0.0}, Velocity{V}, AngularVelocity{0.0}});
    CHECK(w[0].value() == doctest::Approx(+V * kInvSqrt2));
    CHECK(w[1].value() == doctest::Approx(-V * kInvSqrt2));
    CHECK(w[2].value() == doctest::Approx(-V * kInvSqrt2));
    CHECK(w[3].value() == doctest::Approx(+V * kInvSqrt2));
}

TEST_CASE("xDrive: pure rotation — EVERY wheel at R·ω, same sign (the spin signature)") {
    const MatrixKinematics k = xDrive(Length{kR});
    const double w_rate = 2.0;  // rad/s
    const WheelSpeeds w = k.toWheels(ChassisSpeeds{Velocity{0.0}, Velocity{0.0}, AngularVelocity{w_rate}});
    for (int i = 0; i < 4; ++i) {
        CHECK(w[i].value() == doctest::Approx(kR * w_rate));  // all equal, all +R·ω
    }
}

TEST_CASE("xDrive: forward∘toWheels is identity across a swept grid") {
    const MatrixKinematics k = xDrive(Length{kR});
    for (int a = -2; a <= 2; ++a) {
        for (int b = -2; b <= 2; ++b) {
            for (int c = -2; c <= 2; ++c) {
                const double vx = static_cast<double>(a) * 3.0;
                const double vy = static_cast<double>(b) * 3.0;
                const double om = static_cast<double>(c) * 0.5;
                const ChassisSpeeds cmd{Velocity{vx}, Velocity{vy}, AngularVelocity{om}};
                const Twist2d back = k.forward(k.toWheels(cmd));
                CHECK(back.approxEqual(Twist2d{Velocity{vx}, Velocity{vy}, AngularVelocity{om}}, 1e-9));
            }
        }
    }
}

TEST_CASE("xDrive: a pure-spin wheel pattern reads back as pure rotation") {
    const MatrixKinematics k = xDrive(Length{kR});
    // all wheels equal → no translation, only yaw.
    WheelSpeeds spin{4};
    for (int i = 0; i < 4; ++i) {
        spin.set(i, Velocity{5.0});
    }
    const Twist2d t = k.forward(spin);
    CHECK(t.vx().value() == doctest::Approx(0.0));
    CHECK(t.vy().value() == doctest::Approx(0.0));
    CHECK(t.omega().value() == doctest::Approx(5.0 / kR));  // R·ω = 5 → ω = 5/R
}
