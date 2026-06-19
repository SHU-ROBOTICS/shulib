// Geometry tests for TankKinematics. The signatures that matter: forward drives
// both wheels equally, rotation drives them oppositely, strafe is ignored
// entirely, and the forward map recovers (0, vy, ω) — never a phantom vx.

#include "doctest.h"

#include <cmath>

#include "shulib/core/check.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/kinematics/wheel_speeds.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/frame.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/units/quantity.hpp"

using shulib::PreconditionError;
using shulib::kinematics::TankKinematics;
using shulib::kinematics::WheelSpeeds;
using shulib::math::Angle;
using shulib::math::ChassisSpeeds;
using shulib::math::fieldToRobot;
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

TEST_CASE("TankKinematics: pure forward (+X) drives both wheels equally") {
    const TankKinematics k{Length{kTrack}};
    // forward is +X = vx (F1), NOT vy.
    const WheelSpeeds w = k.toWheels(ChassisSpeeds{Velocity{7.0}, Velocity{0.0}, AngularVelocity{0.0}});
    CHECK(w[0].value() == doctest::Approx(7.0));
    CHECK(w[1].value() == doctest::Approx(7.0));
}

TEST_CASE("TankKinematics: pure rotation drives wheels oppositely (±ω·halfTrack)") {
    const TankKinematics k{Length{kTrack}};
    const WheelSpeeds w = k.toWheels(ChassisSpeeds{Velocity{0.0}, Velocity{0.0}, AngularVelocity{2.0}});
    CHECK(w[0].value() == doctest::Approx(-2.0 * kHalf));  // left  = -ω·halfTrack
    CHECK(w[1].value() == doctest::Approx(+2.0 * kHalf));  // right = +ω·halfTrack
}

TEST_CASE("TankKinematics: commanded strafe (vy) is ignored entirely") {
    const TankKinematics k{Length{kTrack}};
    const WheelSpeeds noStrafe = k.toWheels(ChassisSpeeds{Velocity{4.0}, Velocity{0.0}, AngularVelocity{1.0}});
    const WheelSpeeds withStrafe = k.toWheels(ChassisSpeeds{Velocity{4.0}, Velocity{9.0}, AngularVelocity{1.0}});
    CHECK(noStrafe.approxEqual(withStrafe));  // vy (strafe) must not leak into the wheels
}

TEST_CASE("TankKinematics: forward recovers (vx, 0, ω) from known wheels — never a phantom strafe") {
    const TankKinematics k{Length{kTrack}};
    const WheelSpeeds w = [] {
        WheelSpeeds s{2};
        s.set(0, Velocity{2.0});  // left
        s.set(1, Velocity{6.0});  // right
        return s;
    }();
    const Twist2d t = k.forward(w);
    CHECK(t.vx().value() == doctest::Approx(4.0));        // (2+6)/2 — forward
    CHECK(t.vy().value() == doctest::Approx(0.0));        // never a phantom strafe
    CHECK(t.omega().value() == doctest::Approx(0.4));     // (6-2)/(2·5)
}

TEST_CASE("TankKinematics: forward∘toWheels is identity for achievable twists (vy=0)") {
    const TankKinematics k{Length{kTrack}};
    for (int b = -3; b <= 3; ++b) {
        for (int c = -3; c <= 3; ++c) {
            const double vx = static_cast<double>(b) * 2.0;
            const double om = static_cast<double>(c) * 0.5;
            const ChassisSpeeds cmd{Velocity{vx}, Velocity{0.0}, AngularVelocity{om}};
            const Twist2d back = k.forward(k.toWheels(cmd));
            CHECK(back.approxEqual(Twist2d{Velocity{vx}, Velocity{0.0}, AngularVelocity{om}}, 1e-9));
        }
    }
}

TEST_CASE("TankKinematics: forward with the wrong wheel count is rejected") {
    const TankKinematics k{Length{kTrack}};
    WheelSpeeds three{3};
    CHECK_THROWS_AS((void)k.forward(three), PreconditionError);
}

// Cross-layer regression guard: kinematics MUST agree with the F1 body frame
// (frame.hpp). A robot at heading θ commanded to move in the field direction it
// FACES must come out of fieldToRobot as pure +X (vx) and drive both wheels
// equally forward. This is exactly the invariant that broke when the kinematics
// briefly used +Y as forward — at θ=0 the body twist is (V,0), so a vy-as-forward
// tank would read 0 and the wheels would be dead.
TEST_CASE("TankKinematics: consistent with the F1 body frame (forward = the heading direction)") {
    const TankKinematics k{Length{kTrack}};
    const double V = 12.0;
    for (int hd = 0; hd < 360; hd += 45) {
        const Angle heading = Angle::degrees(static_cast<double>(hd));
        // field velocity pointing ALONG the heading (the robot's facing direction):
        const ChassisSpeeds fieldVel{Velocity{V * std::cos(heading.radians())},
                                     Velocity{V * std::sin(heading.radians())},
                                     AngularVelocity{0.0}};
        const ChassisSpeeds body = fieldToRobot(fieldVel, heading);
        const WheelSpeeds w = k.toWheels(body);
        CHECK(w[0].value() == doctest::Approx(V));  // pure forward → both wheels at V
        CHECK(w[1].value() == doctest::Approx(V));
    }
}
