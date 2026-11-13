// TANK1 — the pure geometry of tank steering (motion/tank_steer.hpp), and TankSteerConfig's
// refusals. No plant here: every expected number is derived by hand in the comment beside it, so
// the functions are checked against arithmetic a reader can redo, not against themselves.
// (The closed-loop behaviour is motion_tank_steer_test.cpp.)
//
//   U1  the line frame: along/cross at target headings 0, 90, -90, 180 and across the ±180 seam
//   U2  forward sign: robot LEFT of the line, target AHEAD ⇒ bend turns RIGHT
//   U3  reverse sign: robot LEFT of the line, target BEHIND ⇒ bend turns LEFT (backing onto it)
//   U4  the cap: |bend| ≤ maxSteer over a ±1000 in grid, reached, never exceeded
//   U5  the fade: 0 at arrival exactly, linear inside fadeDistance, continuous; the settle bound
//   U6  the engage window: inclusive at maxSteer, one ulp past it refused, maxSteer 0 refuses all
//   U7  TankSteerConfig::valid() and MotionConfig::validate(): every bad field refused
//   U8  the lead: the predicted cross IS where a robot driving straight would be; its sign
//       forward and backing up; leadTime 0 returns the error now; the settle bound holds with it

#include "doctest.h"

#include <cmath>
#include <limits>
#include <numbers>

#include "shulib/core/check.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/motion/motion_config.hpp"
#include "shulib/motion/tank_steer.hpp"
#include "shulib/units/quantity.hpp"

using shulib::PreconditionError;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::motion::MotionConfig;
using shulib::motion::tankLeadCross;
using shulib::motion::TankLineError;
using shulib::motion::tankLegEngages;
using shulib::motion::tankLineError;
using shulib::motion::TankSteerConfig;
using shulib::motion::tankSteerAngle;
using shulib::units::Length;

namespace {

Pose2d P(double x, double y, double deg) {
    return Pose2d{Length{x}, Length{y}, Angle::degrees(deg)};
}

constexpr double kTight = 1e-12;  // trig of a non-axis angle: a few ulps, never more

}  // namespace

// ═══ U1 — the line frame ═══════════════════════════════════════════════════════════════

// Bug caught: along/cross swapped, a sign flipped on either, or the frame taken from the
// ROBOT's heading instead of the target's (the robot poses below face every other way, so a
// frame built from the wrong heading gives different numbers).
TEST_CASE("U1 tankLineError: along/cross at the axis headings, hand-derived") {
    // Target (10, 20) facing 0 (+x). Robot at (4, 23): 6 in BEFORE the target along +x
    // (along = 4 − 10 = −6) and 3 in to its LEFT (+y is left of +x: cross = +3).
    TankLineError e = tankLineError(P(4, 23, 137), P(10, 20, 0));
    CHECK(e.along == -6.0);
    CHECK(e.cross == 3.0);
    // Same target, robot PAST it and to the RIGHT: (13, 18) ⇒ along +3, cross −2.
    e = tankLineError(P(13, 18, -40), P(10, 20, 0));
    CHECK(e.along == 3.0);
    CHECK(e.cross == -2.0);

    // Facing 90 (+y): left of +y is −x. Robot (7, 25): dy = +5 ⇒ along +5 (past);
    // dx = −3 ⇒ LEFT, cross +3.
    e = tankLineError(P(7, 25, 10), P(10, 20, 90));
    CHECK(e.along == doctest::Approx(5.0).epsilon(kTight));
    CHECK(e.cross == doctest::Approx(3.0).epsilon(kTight));

    // Facing −90 (−y): left of −y is +x. Robot (12, 26): dy = +6 ⇒ BEFORE it (along −6);
    // dx = +2 ⇒ LEFT, cross +2.
    e = tankLineError(P(12, 26, 0), P(10, 20, -90));
    CHECK(e.along == doctest::Approx(-6.0).epsilon(kTight));
    CHECK(e.cross == doctest::Approx(2.0).epsilon(kTight));

    // Facing 180 (−x): left of −x is −y. Robot (15, 17): dx = +5 ⇒ BEFORE (along −5);
    // dy = −3 ⇒ LEFT, cross +3.
    e = tankLineError(P(15, 17, 0), P(10, 20, 180));
    CHECK(e.along == doctest::Approx(-5.0).epsilon(kTight));
    CHECK(e.cross == doctest::Approx(3.0).epsilon(kTight));
}

// Bug caught: a frame that jumps at the ±180 seam — a target heading of +179.9 and one of
// −179.9 are 0.2 degrees apart, so the same robot must be on the SAME side of both lines.
TEST_CASE("U1 tankLineError: continuous across the ±180 seam") {
    // A robot 1 in toward −y of a target facing ~−x is to the target's LEFT on both sides of
    // the seam. cross = cos(th)·dy = −cos(±179.9°) = +0.99999848 on both; along = sin(th)·dy
    // = ∓0.0017453 — the only sign that flips is the tiny along-track one, as it must.
    const TankLineError a = tankLineError(P(0, -1, 0), P(0, 0, 179.9));
    const TankLineError b = tankLineError(P(0, -1, 0), P(0, 0, -179.9));
    const double c = std::cos(179.9 * std::numbers::pi / 180.0);  // −0.99999847691
    CHECK(a.cross == doctest::Approx(-c).epsilon(kTight));
    CHECK(b.cross == doctest::Approx(-c).epsilon(kTight));
    CHECK(a.cross > 0.9999);
    CHECK(a.along == doctest::Approx(-0.0017453283658983088).epsilon(1e-9));
    CHECK(b.along == doctest::Approx(0.0017453283658983088).epsilon(1e-9));
}

// Bug caught: any error in the decomposition at a non-axis heading. Construct the robot FROM a
// chosen (along, cross) in the target's frame and require the function to recover them.
TEST_CASE("U1 tankLineError: recovers (along, cross) from a pose built in the line frame") {
    for (const double deg : {0.0, 17.0, 90.0, 133.0, 179.9, -179.9, -90.0, -61.0}) {
        const double th = deg * std::numbers::pi / 180.0;
        for (const double along : {-40.0, -0.3, 0.0, 2.5, 30.0}) {
            for (const double cross : {-12.0, -0.1, 0.0, 0.7, 6.0}) {
                const double x = 5.0 + along * std::cos(th) - cross * std::sin(th);
                const double y = -8.0 + along * std::sin(th) + cross * std::cos(th);
                const TankLineError e = tankLineError(P(x, y, 0), P(5, -8, deg));
                CHECK(e.along == doctest::Approx(along).epsilon(1e-9).scale(1.0));
                CHECK(e.cross == doctest::Approx(cross).epsilon(1e-9).scale(1.0));
            }
        }
    }
}

// ═══ U2 / U3 — the direction of the bend ════════════════════════════════════════════════

// Bug caught: the bend's sign flipped (it would steer AWAY from the line — M1). Physically:
// facing +x and driving FORWARD from y > 0, the heading must go NEGATIVE (turn right) for the
// velocity's y-component to point back at the line.
TEST_CASE("U2 forward approach: robot LEFT of the line ⇒ bend turns RIGHT, and mirrored") {
    const TankSteerConfig cfg{};
    // along −20 (well outside the fade: factor 1), cross +2 ⇒ bend = −(1)·(0.1·2) = −0.2.
    CHECK(tankSteerAngle(TankLineError{.along = -20.0, .cross = 2.0}, cfg)
          == doctest::Approx(-0.2).epsilon(1e-15));
    CHECK(tankSteerAngle(TankLineError{.along = -20.0, .cross = -2.0}, cfg)
          == doctest::Approx(0.2).epsilon(1e-15));
    // Odd in cross, zero on the line.
    CHECK(tankSteerAngle(TankLineError{.along = -20.0, .cross = 0.0}, cfg) == 0.0);
}

// Bug caught: the direction taken from |along| (M2) — backing onto the line would then steer
// the WRONG way. Facing +x and backing up (velocity −x) from y > 0, the heading must go
// POSITIVE: velocity = −v·(cos h, sin h) has y-component −v·sin h < 0, back toward the line.
TEST_CASE("U3 reverse approach: robot LEFT of the line with the target BEHIND ⇒ bend turns LEFT") {
    const TankSteerConfig cfg{};
    // along +20 (past the target: it lies behind along the heading): factor −1 ⇒ bend = +0.2.
    CHECK(tankSteerAngle(TankLineError{.along = 20.0, .cross = 2.0}, cfg)
          == doctest::Approx(0.2).epsilon(1e-15));
    CHECK(tankSteerAngle(TankLineError{.along = 20.0, .cross = -2.0}, cfg)
          == doctest::Approx(-0.2).epsilon(1e-15));
    // The forward and reverse bends are exact mirrors.
    for (const double cross : {-7.0, -0.4, 0.4, 7.0}) {
        CHECK(tankSteerAngle(TankLineError{.along = 20.0, .cross = cross}, cfg)
              == -tankSteerAngle(TankLineError{.along = -20.0, .cross = cross}, cfg));
    }
}

// ═══ U4 — the cap ═══════════════════════════════════════════════════════════════════════

// Bug caught: the bend clamp removed (M3) — a 100 in offset would ask for a 10 rad "bend",
// i.e. a planned turn; or the direction factor left unclamped (M5) — far from the target it
// would multiply the (capped) bend by along/fade, 166× at 1000 in.
TEST_CASE("U4 the cap: |bend| <= maxSteer over a ±1000 in grid, reached, never exceeded") {
    const TankSteerConfig cfg{};
    double peak = 0.0;
    int atCap = 0;
    for (int i = -100; i <= 100; ++i) {
        for (int j = -100; j <= 100; ++j) {
            const TankLineError e{.along = i * 10.0, .cross = j * 10.0};
            const double b = tankSteerAngle(e, cfg);
            REQUIRE(std::isfinite(b));
            REQUIRE(std::abs(b) <= cfg.maxSteer);
            peak = std::max(peak, std::abs(b));
            if (std::abs(b) == cfg.maxSteer) {
                ++atCap;
            }
        }
    }
    CHECK(peak == cfg.maxSteer);  // the cap is reached, exactly
    CHECK(atCap > 1000);          // …over most of the grid (|cross| ≥ 2.62 in, |along| ≥ 6 in)
    // The same under a config whose gain alone would ask for more than a quarter turn.
    TankSteerConfig hot{};
    hot.crossGain = 50.0;
    hot.maxSteer = 0.5;
    CHECK(std::abs(tankSteerAngle(TankLineError{.along = -1000.0, .cross = 1000.0}, hot)) == 0.5);
    CHECK(std::abs(tankSteerAngle(TankLineError{.along = 1000.0, .cross = -1000.0}, hot)) == 0.5);
}

// ═══ U5 — the fade and the settle bound ═════════════════════════════════════════════════

// Bug caught: the fade removed (M4: direction = along < 0 ? 1 : −1) — the bend would stay
// at full strength through arrival and jump sign across it, holding a settle open or making
// the heading chatter at the target.
TEST_CASE("U5 the fade: zero at arrival exactly, linear inside fadeDistance, continuous") {
    const TankSteerConfig cfg{};  // fade 6 in, gain 0.1
    for (const double cross : {-3.0, -0.5, 0.5, 3.0}) {
        CHECK(tankSteerAngle(TankLineError{.along = 0.0, .cross = cross}, cfg) == 0.0);
        const double full = tankSteerAngle(TankLineError{.along = -6.0, .cross = cross}, cfg);
        // −along/fade = 3/6 = 0.5 exactly, so the half-fade bend is exactly half the full one.
        CHECK(tankSteerAngle(TankLineError{.along = -3.0, .cross = cross}, cfg) == 0.5 * full);
        CHECK(tankSteerAngle(TankLineError{.along = -1.5, .cross = cross}, cfg) == 0.25 * full);
        // Beyond the fade distance the factor is saturated.
        CHECK(tankSteerAngle(TankLineError{.along = -60.0, .cross = cross}, cfg) == full);
        // Continuity through arrival: a millionth of an inch either side of the target, the
        // bends differ by ~1e-7 of a radian — not by twice the full bend (≥ 0.1 rad here).
        const double eps = 1e-6;
        const double before = tankSteerAngle(TankLineError{.along = -eps, .cross = cross}, cfg);
        const double after = tankSteerAngle(TankLineError{.along = eps, .cross = cross}, cfg);
        CHECK(std::abs(before - after) < 1e-6);
        CHECK(std::abs(full) >= 0.05);
    }
}

// Bug caught: a bend that can hold the heading criterion open inside the translation
// tolerance — the settle bound the header states: |along|, |cross| ≤ 0.5 ⇒ |bend| ≤
// (0.5/6)·0.1·0.5 = 0.0041667 rad, a fifth of headingSettle.maxError (0.02).
TEST_CASE("U5 the settle bound: inside the translation tolerance |bend| <= 0.0042 rad") {
    const TankSteerConfig cfg{};
    const MotionConfig mc{};
    const double bound = (0.5 / cfg.fadeDistance) * cfg.crossGain * 0.5;
    CHECK(bound == doctest::Approx(0.004166666666666667).epsilon(1e-12));
    double peak = 0.0;
    for (int i = 0; i <= 100; ++i) {
        for (int j = 0; j <= 100; ++j) {
            const TankLineError e{.along = -0.5 + i * 0.01, .cross = -0.5 + j * 0.01};
            peak = std::max(peak, std::abs(tankSteerAngle(e, cfg)));
        }
    }
    CHECK(peak <= 0.0042);
    CHECK(peak == doctest::Approx(bound).epsilon(1e-9));  // the corner reaches it
    CHECK(peak < mc.headingSettle.maxError / 4.0);
}

// ═══ U6 — the engage window ═════════════════════════════════════════════════════════════

// Bug caught: the window made exclusive (M8), or a leg engaging with TANK1 switched off.
TEST_CASE("U6 tankLegEngages: inclusive at maxSteer, one ulp past refused, maxSteer 0 refuses "
          "all") {
    const TankSteerConfig cfg{};
    CHECK(tankLegEngages(0.0, cfg));
    CHECK(tankLegEngages(cfg.maxSteer, cfg));
    CHECK(tankLegEngages(-cfg.maxSteer, cfg));
    const double past = std::nextafter(cfg.maxSteer, std::numeric_limits<double>::infinity());
    CHECK_FALSE(tankLegEngages(past, cfg));
    CHECK_FALSE(tankLegEngages(-past, cfg));
    CHECK_FALSE(tankLegEngages(std::numbers::pi, cfg));
    TankSteerConfig off{};
    off.maxSteer = 0.0;
    for (const double e : {0.0, -0.0, 1e-300, -1e-300, 0.1, std::numbers::pi}) {
        CHECK_FALSE(tankLegEngages(e, off));
    }
}

// ═══ U7 — refusals ══════════════════════════════════════════════════════════════════════

// Bug caught: a non-finite or out-of-range knob reaching the tick loop (a NaN gain steers NaN;
// a zero fade divides by zero; maxSteer ≥ π/2 reverses the drive along the line), or
// validate() not descending into tankSteer at all.
TEST_CASE("U7 TankSteerConfig::valid() and MotionConfig::validate() refuse every bad field") {
    CHECK(TankSteerConfig{}.valid());
    CHECK_NOTHROW(MotionConfig{}.validate());
    // The documented off switches are VALID configs.
    {
        MotionConfig mc{};
        mc.tankSteer.maxSteer = 0.0;
        CHECK_NOTHROW(mc.validate());
        mc.tankSteer.yawCorrectionLimit = 0.0;
        mc.tankSteer.crossGain = 0.0;
        mc.tankSteer.leadTime = 0.0;
        CHECK_NOTHROW(mc.validate());
    }
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    double TankSteerConfig::* const fields[] = {
        &TankSteerConfig::crossGain,       &TankSteerConfig::maxSteer,
        &TankSteerConfig::fadeDistance,    &TankSteerConfig::leadTime,
        &TankSteerConfig::yawObserverTime, &TankSteerConfig::yawCorrectionLimit};
    for (double TankSteerConfig::* f : fields) {
        for (const double bad : {nan, inf, -inf, -1e-9, -1.0}) {
            MotionConfig mc{};
            mc.tankSteer.*f = bad;
            CHECK_FALSE(mc.tankSteer.valid());
            CHECK_THROWS_AS(mc.validate(), PreconditionError);
        }
    }
    // The range edges.
    for (const double bad : {std::numbers::pi / 2.0, 1.5707963267948966, 2.0, std::numbers::pi}) {
        MotionConfig mc{};
        mc.tankSteer.maxSteer = bad;
        CHECK_THROWS_AS(mc.validate(), PreconditionError);
    }
    {
        MotionConfig mc{};
        mc.tankSteer.maxSteer = std::nextafter(std::numbers::pi / 2.0, 0.0);  // just under: legal
        CHECK_NOTHROW(mc.validate());
    }
    {
        MotionConfig mc{};
        mc.tankSteer.fadeDistance = 0.0;
        CHECK_THROWS_AS(mc.validate(), PreconditionError);
    }
    {
        MotionConfig mc{};
        mc.tankSteer.yawObserverTime = 0.0;
        CHECK_THROWS_AS(mc.validate(), PreconditionError);
    }
}

// ═══ U8 — the lead ══════════════════════════════════════════════════════════════════════

// Bug caught: the lead's sign flipped (it would steer on a cross that GROWS as the robot closes
// on the line — anti-damping: the overshoot the Part A review found, made worse), the heading
// taken the wrong way round, or the speed's sign dropped (a reverse leg predicted as a forward
// one). The oracle is geometry, not the formula: a robot that drives STRAIGHT along its heading
// at speed v for leadTime seconds ends exactly where tankLeadCross says its cross-track error
// will be — so move the pose and ask tankLineError.
TEST_CASE("U8 tankLeadCross: the predicted cross is where a robot driving straight would be") {
    TankSteerConfig cfg{};
    cfg.leadTime = 0.25;  // a long lead, so the prediction moves the robot visibly
    for (const double targetDeg : {0.0, 37.0, 90.0, 179.9, -179.9, -120.0}) {
        for (const double offDeg : {-14.0, -3.0, 0.0, 5.0, 14.9}) {
            for (const double v : {-60.0, -7.5, 0.0, 12.0, 60.0}) {
                const Pose2d target = P(10.0, -4.0, targetDeg);
                const double h = (targetDeg + offDeg) * std::numbers::pi / 180.0;
                const Pose2d now = P(-20.0, 3.0, targetDeg + offDeg);
                const double d = v * cfg.leadTime;  // straight-line travel in the lead time
                const Pose2d later{Length{now.x().value() + d * std::cos(h)},
                                   Length{now.y().value() + d * std::sin(h)}, now.heading()};
                const double headingFromTarget =
                    now.heading().radians() - target.heading().radians();
                const double predicted =
                    tankLeadCross(tankLineError(now, target), headingFromTarget, v, cfg);
                const double actual = tankLineError(later, target).cross;
                CHECK(predicted == doctest::Approx(actual).epsilon(1e-9).scale(1.0));
            }
        }
    }
}

// Bug caught: the same, checked against hand arithmetic. Robot 2 in LEFT of a line along +x,
// nose 10 degrees RIGHT of it, 60 in/s forward, lead 0.06 s: it closes on the line at
// 60·sin(10°) = 10.419 in/s, so in 0.06 s it is 0.62513 in closer: 2 − 0.62513 = 1.37487.
// Backing up at 60 in/s with the nose 10 degrees LEFT closes on it just as fast. Nose on the
// line's heading, or no speed: the error now, exactly. Heading AWAY from the line: it grows.
TEST_CASE("U8 tankLeadCross: hand-derived values, forward and backing up") {
    TankSteerConfig cfg{};
    cfg.leadTime = 0.06;
    const TankLineError left{.along = -20.0, .cross = 2.0};
    const double ten = 10.0 * std::numbers::pi / 180.0;
    const double closing = 60.0 * std::sin(ten) * 0.06;  // 0.6251334…
    CHECK(closing == doctest::Approx(0.62513344).epsilon(1e-7));
    CHECK(tankLeadCross(left, -ten, 60.0, cfg) == doctest::Approx(2.0 - closing).epsilon(1e-15));
    CHECK(tankLeadCross(left, +ten, -60.0, cfg) == doctest::Approx(2.0 - closing).epsilon(1e-15));
    CHECK(tankLeadCross(left, +ten, 60.0, cfg) == doctest::Approx(2.0 + closing).epsilon(1e-15));
    CHECK(tankLeadCross(left, 0.0, 60.0, cfg) == 2.0);
    CHECK(tankLeadCross(left, ten, 0.0, cfg) == 2.0);
    // leadTime 0 is the bend without a lead: the error now, bit for bit, whatever the motion.
    TankSteerConfig none{};
    none.leadTime = 0.0;
    for (const double cross : {-7.25, -0.0, 0.0, 0.3, 11.0}) {
        for (const double v : {-60.0, 0.0, 60.0}) {
            const TankLineError e{.along = -5.0, .cross = cross};
            const double c = tankLeadCross(e, 0.2, v, none);
            CHECK(c == cross);
        }
    }
    // Closing on the line makes the bend SMALLER than the error now would (the damping);
    // leaving it makes it larger.
    const double bendNow = tankSteerAngle(left, cfg);
    const TankLineError closingLine{.along = -20.0, .cross = tankLeadCross(left, -ten, 60.0, cfg)};
    const TankLineError leavingLine{.along = -20.0, .cross = tankLeadCross(left, ten, 60.0, cfg)};
    const double bendClosing = tankSteerAngle(closingLine, cfg);
    const double bendLeaving = tankSteerAngle(leavingLine, cfg);
    CHECK(std::abs(bendClosing) < std::abs(bendNow));
    CHECK(std::abs(bendLeaving) > std::abs(bendNow));
}

// Bug caught: a lead that could hold a settle open. Inside the translation tolerance the leg is
// commanded at most translation.kP × 0.5 in = 1.5 in/s (either way) and its heading is within
// headingSettle.maxError (0.02 rad) of the target's, so the predicted cross is at most 0.5 +
// 0.06·1.5·sin(0.02) = 0.5018 in and the bend stays under the header's 0.0042 rad.
TEST_CASE("U8 the settle bound holds with the lead") {
    const TankSteerConfig cfg{};
    const MotionConfig mc{};
    const double vMax = mc.translation.kP * mc.translationSettle.maxError;  // 1.5 in/s
    CHECK(vMax == 1.5);
    double peak = 0.0;
    for (int i = 0; i <= 20; ++i) {
        for (int j = 0; j <= 20; ++j) {
            for (const double off : {-0.02, 0.0, 0.02}) {
                for (const double v : {-vMax, 0.0, vMax}) {
                    const TankLineError e{.along = -0.5 + i * 0.05, .cross = -0.5 + j * 0.05};
                    const TankLineError ahead{.along = e.along,
                                              .cross = tankLeadCross(e, off, v, cfg)};
                    peak = std::max(peak, std::abs(tankSteerAngle(ahead, cfg)));
                }
            }
        }
    }
    CHECK(peak <= 0.0042);
    CHECK(peak > 0.0041);  // the corner reaches the bound: the grid is not vacuous
    CHECK(peak < mc.headingSettle.maxError / 4.0);
}
