// Geometry tests for TankKinematics. The signatures that matter: forward drives
// both wheels equally, rotation drives them oppositely, strafe is ignored
// entirely, and the forward map recovers (0, vy, ω) — never a phantom vx.

#include "doctest.h"

#include "shulib/core/check.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/kinematics/wheel_speeds.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/units/quantity.hpp"

using shulib::PreconditionError;
using shulib::kinematics::TankKinematics;
using shulib::kinematics::WheelSpeeds;
using shulib::math::ChassisSpeeds;
using shulib::math::Twist2d;
using shulib::units::AngularVelocity;
using shulib::units::Length;
using shulib::units::Velocity;

namespace {
constexpr double kTrack = 10.0;  // → halfTrack 5
constexpr double kHalf = 5.0;
}  // namespace

TEST_CASE("TankKinematics: shape and zero strafe authority") {
    const TankKinematics k{Length{kTrack}};
    CHECK(k.wheelCount() == 2);
    CHECK(k.strafeAuthority() == doctest::Approx(0.0));
}

TEST_CASE("TankKinematics: a positive trackWidth is required") {
    CHECK_THROWS_AS(TankKinematics{Length{0.0}}, PreconditionError);
    CHECK_THROWS_AS(TankKinematics{Length{-3.0}}, PreconditionError);
}

TEST_CASE("TankKinematics: pure forward drives both wheels equally") {
    const TankKinematics k{Length{kTrack}};
    const WheelSpeeds w = k.toWheels(ChassisSpeeds{Velocity{0.0}, Velocity{7.0}, AngularVelocity{0.0}});
    CHECK(w[0].value() == doctest::Approx(7.0));
    CHECK(w[1].value() == doctest::Approx(7.0));
}

TEST_CASE("TankKinematics: pure rotation drives wheels oppositely (±ω·halfTrack)") {
    const TankKinematics k{Length{kTrack}};
    const WheelSpeeds w = k.toWheels(ChassisSpeeds{Velocity{0.0}, Velocity{0.0}, AngularVelocity{2.0}});
    CHECK(w[0].value() == doctest::Approx(-2.0 * kHalf));  // left  = -ω·halfTrack
    CHECK(w[1].value() == doctest::Approx(+2.0 * kHalf));  // right = +ω·halfTrack
}

TEST_CASE("TankKinematics: commanded strafe (vx) is ignored entirely") {
    const TankKinematics k{Length{kTrack}};
    const WheelSpeeds noStrafe = k.toWheels(ChassisSpeeds{Velocity{0.0}, Velocity{4.0}, AngularVelocity{1.0}});
    const WheelSpeeds withStrafe = k.toWheels(ChassisSpeeds{Velocity{9.0}, Velocity{4.0}, AngularVelocity{1.0}});
    CHECK(noStrafe.approxEqual(withStrafe));  // vx must not leak into the wheels
}

TEST_CASE("TankKinematics: forward recovers (0, vy, ω) from known wheels") {
    const TankKinematics k{Length{kTrack}};
    const WheelSpeeds w = [] {
        WheelSpeeds s{2};
        s.set(0, Velocity{2.0});  // left
        s.set(1, Velocity{6.0});  // right
        return s;
    }();
    const Twist2d t = k.forward(w);
    CHECK(t.vx().value() == doctest::Approx(0.0));        // never a phantom strafe
    CHECK(t.vy().value() == doctest::Approx(4.0));        // (2+6)/2
    CHECK(t.omega().value() == doctest::Approx(0.4));     // (6-2)/(2·5)
}

TEST_CASE("TankKinematics: forward∘toWheels is identity for achievable twists (vx=0)") {
    const TankKinematics k{Length{kTrack}};
    for (int b = -3; b <= 3; ++b) {
        for (int c = -3; c <= 3; ++c) {
            const double vy = static_cast<double>(b) * 2.0;
            const double om = static_cast<double>(c) * 0.5;
            const ChassisSpeeds cmd{Velocity{0.0}, Velocity{vy}, AngularVelocity{om}};
            const Twist2d back = k.forward(k.toWheels(cmd));
            CHECK(back.approxEqual(Twist2d{Velocity{0.0}, Velocity{vy}, AngularVelocity{om}}, 1e-9));
        }
    }
}

TEST_CASE("TankKinematics: forward with the wrong wheel count is rejected") {
    const TankKinematics k{Length{kTrack}};
    WheelSpeeds three{3};
    CHECK_THROWS_AS((void)k.forward(three), PreconditionError);
}
