// STEP1-B — motion/leg_progress.hpp, the PURE geometry a partway trigger reads (the STEP1 brief's D7 (a),
// B5): a drive's progress is the distance ALONG THE STRAIGHT LINE from its live start to its target (the
// projection), a turn's the angle turned toward its target the short way; a zero-size leg has no
// direction and reads 0 everywhere. Each case attacks ONE rule and names the bug it would catch. Pure
// math: no plant, no robot.

#include "doctest.h"

#include <cmath>

#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/motion/leg_progress.hpp"

using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::motion::kMinLegLength;
using shulib::motion::kMinTurnRad;
using shulib::motion::LegProgress;
using shulib::motion::legProgressAlong;
using shulib::motion::turnProgress;
using shulib::units::Length;

namespace {
Pose2d at(double x, double y, double deg = 0.0) { return Pose2d{Length{x}, Length{y}, Angle::degrees(deg)}; }
constexpr double kPi = 3.14159265358979323846;
}  // namespace

TEST_CASE("STEP1-B leg progress: a drive's progress is the PROJECTION on the start->target line (D7 (a))") {
    // Bug caught: "60 %" read as the distance still to go (a robot off its line gains progress by moving
    // sideways — D7 (b)), or as the path travelled (D7 (c)), or the target's HEADING leaking in.
    const LegProgress p = legProgressAlong(at(0, 0), at(40, 0, 90), at(24, 0, -30));
    CHECK(p.live);
    CHECK(p.hasDirection);
    CHECK(p.length.value() == 40.0);
    CHECK(p.along.value() == 24.0);
    CHECK(p.fraction == 0.6);  // EXACTLY 24 / 40 — the trigger's `>=` lands on it
    CHECK(p.turnedRad == 0.0);
    CHECK(p.sizeRad == 0.0);
    CHECK(p.positionTolerance.value() == 0.0);  // the caller's (the chassis's decorator) to fill
    // Sideways: 10 in off the line, not one inch along it.
    const LegProgress side = legProgressAlong(at(0, 0), at(40, 0), at(0, 10));
    CHECK(side.along.value() == 0.0);
    CHECK(side.fraction == 0.0);
    // ... while "distance to go" would have read 40 - hypot(40, 10) < 0 and "travelled" 10: neither is 0.
    const LegProgress offLine = legProgressAlong(at(0, 0), at(40, 0), at(30, 7));
    CHECK(offLine.along.value() == 30.0);
}

TEST_CASE("STEP1-B leg progress: any direction, any start; overshoot past 1 and backwards below 0") {
    // Bug caught: a projection that assumes the leg runs along +x, ignores where it STARTED, or clamps
    // the fraction into [0, 1] (an overshoot must still read past the mark; a push backwards below 0).
    const LegProgress diag = legProgressAlong(at(10, 20), at(40, 60), at(13, 24));  // a 3-4-5 leg of 50
    CHECK(diag.length.value() == doctest::Approx(50.0).epsilon(1e-15));
    CHECK(diag.along.value() == doctest::Approx(5.0).epsilon(1e-15));
    CHECK(diag.fraction == doctest::Approx(0.1).epsilon(1e-15));
    const LegProgress perp = legProgressAlong(at(10, 20), at(40, 60), at(14, 17));  // (4, -3): perpendicular
    CHECK(std::abs(perp.along.value()) < 1e-12);
    const LegProgress over = legProgressAlong(at(0, 0), at(40, 0), at(44, 0));
    CHECK(over.fraction == doctest::Approx(1.1));
    const LegProgress back = legProgressAlong(at(0, 0), at(40, 0), at(-2, 0));
    CHECK(back.fraction == doctest::Approx(-0.05));
    CHECK(back.along.value() == -2.0);
    // A reverse leg (target behind, the robot backing): progress is still along start->target.
    const LegProgress rev = legProgressAlong(at(0, 0, 0), at(-30, 0, 0), at(-12, 0, 0));
    CHECK(rev.fraction == doctest::Approx(0.4));
}

TEST_CASE("STEP1-B leg progress: a ZERO-length drive has no direction -- every field 0 (B5), at the exact edge") {
    // Bug caught: a 0/0 fraction (NaN — a NaN compares false with everything, but an `inf` from a tiny
    // length compares TRUE with every mark), or the edge moved off kMinLegLength.
    const LegProgress z = legProgressAlong(at(5, 5), at(5, 5), at(9, 9));
    CHECK(z.live);
    CHECK_FALSE(z.hasDirection);
    CHECK(z.fraction == 0.0);
    CHECK(z.along.value() == 0.0);
    CHECK(z.length.value() == 0.0);
    // Just under the edge: still no direction, and nothing huge.
    const LegProgress under = legProgressAlong(at(0, 0), at(0.5e-6, 0), at(3, 0));
    CHECK_FALSE(under.hasDirection);
    CHECK(under.fraction == 0.0);
    // EXACTLY at the edge (hypot(1e-6, 0) == 1e-6): a direction (the rule is >=).
    REQUIRE(std::hypot(kMinLegLength, 0.0) == kMinLegLength);
    const LegProgress edge = legProgressAlong(at(0, 0), at(kMinLegLength, 0), at(kMinLegLength, 0));
    CHECK(edge.hasDirection);
    CHECK(edge.length.value() == kMinLegLength);
    CHECK(edge.fraction == doctest::Approx(1.0));
}

TEST_CASE("STEP1-B leg progress: a turn's progress is the angle turned the SHORT way, over its size") {
    // Bug caught: the turned angle measured the long way, with the wrong sign for a CW turn, or across
    // the ±180 wrap as a jump of 360.
    const LegProgress ccw = turnProgress(Angle::degrees(0), Angle::degrees(90), Angle::degrees(54));
    CHECK(ccw.live);
    CHECK(ccw.hasDirection);
    CHECK(ccw.sizeRad == doctest::Approx(kPi / 2));
    CHECK(ccw.turnedRad == doctest::Approx(54.0 * kPi / 180.0));
    CHECK(ccw.fraction == doctest::Approx(0.6));
    CHECK(ccw.along.value() == 0.0);
    CHECK(ccw.length.value() == 0.0);
    const LegProgress cw = turnProgress(Angle::degrees(0), Angle::degrees(-90), Angle::degrees(-45));
    CHECK(cw.fraction == doctest::Approx(0.5));
    CHECK(cw.turnedRad > 0.0);  // turned TOWARD the target is positive whichever way that is
    const LegProgress wrong = turnProgress(Angle::degrees(0), Angle::degrees(-90), Angle::degrees(10));
    CHECK(wrong.fraction < 0.0);  // the wrong way: below 0, never "progress"
    // Across the wrap: 170 -> -170 is +20 CCW; at 180 it has turned 10, at -175 it has turned 15.
    const LegProgress w1 = turnProgress(Angle::degrees(170), Angle::degrees(-170), Angle::degrees(180));
    CHECK(w1.sizeRad == doctest::Approx(20.0 * kPi / 180.0));
    CHECK(w1.fraction == doctest::Approx(0.5));
    const LegProgress w2 = turnProgress(Angle::degrees(170), Angle::degrees(-170), Angle::degrees(-175));
    CHECK(w2.fraction == doctest::Approx(0.75));
    // An overshoot reads past 1.
    const LegProgress over = turnProgress(Angle::degrees(0), Angle::degrees(90), Angle::degrees(99));
    CHECK(over.fraction == doctest::Approx(1.1));
}

TEST_CASE("STEP1-B leg progress: an exact 180 turn resolves CCW, as TurnTo turns") {
    // Bug caught: a turn of exactly 180 measured clockwise while TurnTo turns counter-clockwise — every
    // trigger on it would read negative progress and never fire.
    const LegProgress h = turnProgress(Angle::degrees(0), Angle::degrees(180), Angle::degrees(90));
    CHECK(h.sizeRad == doctest::Approx(kPi));
    CHECK(h.fraction == doctest::Approx(0.5));
    const LegProgress hw = turnProgress(Angle::degrees(0), Angle::degrees(180), Angle::degrees(-90));
    CHECK(hw.fraction == doctest::Approx(-0.5));
}

TEST_CASE("STEP1-B leg progress: a ZERO-size turn has no direction (B5), at the exact edge") {
    // Bug caught: a turn to the heading the robot already has dividing by 0, or reading a sign.
    const LegProgress z = turnProgress(Angle::degrees(30), Angle::degrees(30), Angle::degrees(31));
    CHECK(z.live);
    CHECK_FALSE(z.hasDirection);
    CHECK(z.fraction == 0.0);
    CHECK(z.turnedRad == 0.0);
    CHECK(z.sizeRad == 0.0);
    const LegProgress under = turnProgress(Angle::radians(0.0), Angle::radians(0.5e-9), Angle::radians(0.1));
    CHECK_FALSE(under.hasDirection);
    const LegProgress edge = turnProgress(Angle::radians(0.0), Angle::radians(kMinTurnRad), Angle::radians(kMinTurnRad));
    CHECK(edge.hasDirection);
    CHECK(edge.fraction == 1.0);
}
