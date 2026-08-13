// Adversarial tests for the GPS→canonical conversion. Each case targets a way the
// frame math could be wrong: heading handedness (CW-from-North vs CCW-from-+X), the
// North-alignment offset, the position-frame rotation (tested at NON-trivial θ_N so a
// rotation sign error can't hide behind the identity case), the meters→inches scale,
// and lever-arm removal direction.
//
// ── E2 addition: the INDEPENDENT oracles ───────────────────────────────────────────
// The cases written before E2 pin the position scale against `kMetersToInches`
// IMPORTED FROM THE HEADER UNDER TEST. That is the shared-oracle blindness that has
// bitten this project three times (C1, C3, C4): if the constant were 3.937 instead of
// 39.37, every one of those assertions would still pass, because the same wrong number
// sits on both sides of the ==. The scale had never been pinned against an absolute.
//
// The cases at the bottom of this file, marked [oracle], fix that. They share NOTHING
// with the implementation:
//   * the inch scale comes from the DEFINITION (1 inch ≡ 0.0254 m exactly), so
//     0.0254 m must be 1.0 inch and 0.3048 m must be 12.0 inches — numbers a person
//     can check without opening the header;
//   * the frame rotation is re-derived from the stated physical convention (VEX-North
//     points along canonical θ_N; VEX-East is 90° CLOCKWISE of North, i.e. θ_N − 90°)
//     rather than by calling the function being tested;
//   * the lever arm is re-derived from the body-frame definition (forward is the
//     heading unit vector; left is 90° CCW of forward), at seven headings including
//     ones where both components are non-zero — a sign error in the cross term is
//     invisible at heading 0.

#include "doctest.h"

#include <cmath>
#include <limits>

#include "shulib/core/check.hpp"
#include "shulib/hal/gps_conversion.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/units/quantity.hpp"

using shulib::PreconditionError;
using shulib::hal::gpsHeadingToCanonical;
using shulib::hal::gpsRemoveLeverArm;
using shulib::hal::gpsRmsErrorToCanonical;
using shulib::hal::gpsSensorPose;
using shulib::hal::gpsToRobotPose;
using shulib::hal::kMetersToInches;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::units::Length;

// --- heading: CW-from-North → canonical CCW-from-+X (default North = +Y) ---

TEST_CASE("gpsHeadingToCanonical: compass cardinals map correctly (North=+Y default)") {
    CHECK(gpsHeadingToCanonical(0.0).approxEqual(Angle::degrees(90.0)));     // N → +Y
    CHECK(gpsHeadingToCanonical(90.0).approxEqual(Angle::degrees(0.0)));     // E → +X
    CHECK(gpsHeadingToCanonical(180.0).approxEqual(Angle::degrees(-90.0)));  // S → -Y
    const Angle west = gpsHeadingToCanonical(270.0);                         // W → -X
    CHECK(west.approxEqual(Angle::degrees(180.0)));
    CHECK(west.radians() > 0.0);  // +π, never -π (F3 tie-break survives the conversion)
}

TEST_CASE("gpsHeadingToCanonical: a CW (compass) increase DECREASES canonical heading") {
    CHECK(gpsHeadingToCanonical(45.0).approxEqual(Angle::degrees(45.0)));   // 90 - 45
    CHECK(gpsHeadingToCanonical(135.0).approxEqual(Angle::degrees(-45.0))); // 90 - 135
}

TEST_CASE("gpsHeadingToCanonical: the North-alignment offset is honored") {
    // northHeadingDeg = 0 ⇒ VEX-North aligned with canonical +X.
    CHECK(gpsHeadingToCanonical(0.0, 0.0).approxEqual(Angle::degrees(0.0)));
    CHECK(gpsHeadingToCanonical(90.0, 0.0).approxEqual(Angle::degrees(-90.0)));
}

TEST_CASE("gpsHeadingToCanonical: a non-finite heading is rejected") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK_THROWS_AS((void)gpsHeadingToCanonical(nan), PreconditionError);
}

// --- position: VEX (East, North) meters → canonical (x, y) inches ---

TEST_CASE("gpsSensorPose: default frame maps East→+X, North→+Y with meters→inches") {
    const Pose2d east = gpsSensorPose(1.0, 0.0, 0.0);  // 1 m East
    CHECK(east.x().value() == doctest::Approx(kMetersToInches));  // → +X inches
    CHECK(east.y().value() == doctest::Approx(0.0));

    const Pose2d north = gpsSensorPose(0.0, 1.0, 0.0);  // 1 m North
    CHECK(north.x().value() == doctest::Approx(0.0));
    CHECK(north.y().value() == doctest::Approx(kMetersToInches));  // → +Y inches

    const Pose2d both = gpsSensorPose(2.0, 3.0, 0.0);
    CHECK(both.x().value() == doctest::Approx(2.0 * kMetersToInches));
    CHECK(both.y().value() == doctest::Approx(3.0 * kMetersToInches));
}

TEST_CASE("gpsSensorPose: the position rotation follows the North offset (θ_N ≠ 90)") {
    // northHeadingDeg = 0 ⇒ North = +X, East = -Y. (Distinguishes a rotation bug that
    // the identity θ_N=90 case would hide.)
    const Pose2d north = gpsSensorPose(0.0, 1.0, 0.0, /*northHeadingDeg=*/0.0);
    CHECK(north.x().value() == doctest::Approx(kMetersToInches));  // North → +X
    CHECK(north.y().value() == doctest::Approx(0.0));

    const Pose2d east = gpsSensorPose(1.0, 0.0, 0.0, /*northHeadingDeg=*/0.0);
    CHECK(east.x().value() == doctest::Approx(0.0));
    CHECK(east.y().value() == doctest::Approx(-kMetersToInches));  // East → -Y
}

TEST_CASE("gpsSensorPose: carries the converted heading and rejects non-finite position") {
    const Pose2d p = gpsSensorPose(0.0, 0.0, 90.0);  // heading 90° CW (East)
    CHECK(p.heading().approxEqual(Angle::degrees(0.0)));  // East → canonical +X

    const double inf = std::numeric_limits<double>::infinity();
    CHECK_THROWS_AS((void)gpsSensorPose(inf, 0.0, 0.0), PreconditionError);
    CHECK_THROWS_AS((void)gpsSensorPose(0.0, inf, 0.0), PreconditionError);
}

// --- lever arm: sensor pose → robot center ---

TEST_CASE("gpsRemoveLeverArm: a zero lever arm leaves the pose unchanged") {
    const Pose2d sensor{Length{10.0}, Length{20.0}, Angle::degrees(30.0)};
    CHECK(gpsRemoveLeverArm(sensor, Length{0.0}, Length{0.0}).approxEqual(sensor));
}

TEST_CASE("gpsRemoveLeverArm: a forward-mounted sensor shifts the center back along heading") {
    // heading 0 (facing +X): a 6" forward sensor sits +6 in X of center → center.x -= 6.
    const Pose2d s0{Length{10.0}, Length{20.0}, Angle::degrees(0.0)};
    const Pose2d c0 = gpsRemoveLeverArm(s0, Length{6.0}, Length{0.0});
    CHECK(c0.x().value() == doctest::Approx(4.0));
    CHECK(c0.y().value() == doctest::Approx(20.0));
    CHECK(c0.heading().approxEqual(Angle::degrees(0.0)));  // heading untouched

    // heading 90° (facing +Y): forward is +Y → center.y -= 6.
    const Pose2d s90{Length{10.0}, Length{20.0}, Angle::degrees(90.0)};
    const Pose2d c90 = gpsRemoveLeverArm(s90, Length{6.0}, Length{0.0});
    CHECK(c90.x().value() == doctest::Approx(10.0));
    CHECK(c90.y().value() == doctest::Approx(14.0));
}

TEST_CASE("gpsRemoveLeverArm: a left-mounted sensor shifts the center along +Y at heading 0") {
    // heading 0: left is +Y → a 6" left sensor sits +6 in Y of center → center.y -= 6.
    const Pose2d s{Length{10.0}, Length{20.0}, Angle::degrees(0.0)};
    const Pose2d c = gpsRemoveLeverArm(s, Length{0.0}, Length{6.0});
    CHECK(c.x().value() == doctest::Approx(10.0));
    CHECK(c.y().value() == doctest::Approx(14.0));
}

TEST_CASE("gpsRemoveLeverArm: left lever at heading 90° exercises the cross (sin·left) term") {
    // facing +Y, body-left points to field -X → a 6" left sensor sits at center.x − 6
    // → center.x = sensor.x + 6. (Pins the s·ll term that forward-only / heading-0 miss.)
    const Pose2d s{Length{10.0}, Length{20.0}, Angle::degrees(90.0)};
    const Pose2d c = gpsRemoveLeverArm(s, Length{0.0}, Length{6.0});
    CHECK(c.x().value() == doctest::Approx(16.0));
    CHECK(c.y().value() == doctest::Approx(20.0));
}

// --- full path ---

TEST_CASE("gpsToRobotPose: end-to-end raw reading → canonical center pose") {
    // GPS at field center, compass heading 0 (North → canonical +Y / facing +Y),
    // sensor mounted 10" forward of center → center is 10" behind in -Y.
    const Pose2d center = gpsToRobotPose(0.0, 0.0, 0.0, Length{10.0}, Length{0.0});
    CHECK(center.x().value() == doctest::Approx(0.0));
    CHECK(center.y().value() == doctest::Approx(-10.0));
    CHECK(center.heading().approxEqual(Angle::degrees(90.0)));
}

TEST_CASE("gpsRemoveLeverArm: a non-finite lever arm is rejected (closes the last NaN-into-pose hole)") {
    const Pose2d s{Length{10.0}, Length{20.0}, Angle::degrees(0.0)};
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    CHECK_THROWS_AS((void)gpsRemoveLeverArm(s, Length{nan}, Length{0.0}), PreconditionError);
    CHECK_THROWS_AS((void)gpsRemoveLeverArm(s, Length{0.0}, Length{inf}), PreconditionError);
}

TEST_CASE("gpsToRobotPose: a PROS_ERR_F (off-strip/failed) read fails loud, not silently") {
    // PROS_ERR_F == INFINITY. The adapter must screen this to hasFix()=false BEFORE
    // converting (§13 #4). If a sentinel ever reaches the converter it THROWS — the
    // fail-loud backstop, never a NaN/Inf pose into the estimate.
    const double s = std::numeric_limits<double>::infinity();
    CHECK_THROWS_AS((void)gpsToRobotPose(s, s, s, Length{5.0}, Length{0.0}), PreconditionError);    // all-sentinel
    CHECK_THROWS_AS((void)gpsToRobotPose(0.0, 0.0, s, Length{5.0}, Length{0.0}), PreconditionError);  // heading-only
}

// FIELD-CAL ORACLE (skipped — runs on hardware). The position-axis→compass binding
// (VEX +X = East, +Y = North) is UNVERIFIED from PROS and a wrong guess MIRRORS the pose.
// (A4 register HA-01, docs/hardware-assumptions.md — R3 unskips this with measured values.)
// Before a scored run: place the GPS at a known +1 m-East / +1 m-North point at a known
// heading, read raw get_position()/get_heading(), and replace the expected values below
// with the MEASURED raw→canonical mapping. Until then this records the missing oracle.
TEST_CASE("gpsSensorPose: FIELD-CAL axis oracle — bench-measure before trusting" * doctest::skip()) {
    const Pose2d east = gpsSensorPose(1.0, 0.0, 0.0);
    CHECK(east.x().value() == doctest::Approx(kMetersToInches));   // ASSUMED East → +X
    const Pose2d north = gpsSensorPose(0.0, 1.0, 0.0);
    CHECK(north.y().value() == doctest::Approx(kMetersToInches));  // ASSUMED North → +Y
}

// ═══════════════════════════════════════════════════════════════════════════════════
// [oracle] — the INDEPENDENT pins added at E2 (file header note). Nothing below shares
// a constant or a formula with the code it checks.
// ═══════════════════════════════════════════════════════════════════════════════════

// Would catch: HA-07 skipped, double-applied, or applied with a wrong constant — the
// silent factor of 39.37 that makes the corrector's R absurd in one direction or the
// other. NOTHING here reads kMetersToInches: the international inch is DEFINED as
// exactly 0.0254 m, so these expectations are arithmetic a person can do on paper.
TEST_CASE("[oracle] gpsRmsErrorToCanonical: metres→inches, against the DEFINITION") {
    // 0.0254 m IS one inch, by definition. If the scale is missing this reads 0.0254;
    // if it is applied twice it reads 39.37.
    CHECK(gpsRmsErrorToCanonical(0.0254).value() == doctest::Approx(1.0).epsilon(1e-12));
    // One foot.
    CHECK(gpsRmsErrorToCanonical(0.3048).value() == doctest::Approx(12.0).epsilon(1e-12));
    // A tile-and-a-half (VEX tiles are 24"): 36 inches is 0.9144 m.
    CHECK(gpsRmsErrorToCanonical(0.9144).value() == doctest::Approx(36.0).epsilon(1e-12));
    // Zero is a legitimate reading and must not be special-cased away.
    CHECK(gpsRmsErrorToCanonical(0.0).value() == 0.0);
    // A HEALTHY on-strip claim (HA-29's ~1 inch class) lands in inches, not metres:
    // 0.025 m is a hair under an inch. Reading ~0.025 here would mean R is 39× too
    // small and every good fix gets gated out; reading ~39 would mean lies sail in.
    CHECK(gpsRmsErrorToCanonical(0.025).value() > 0.9);
    CHECK(gpsRmsErrorToCanonical(0.025).value() < 1.0);
}

// Would catch: a sentinel or a negative rms reaching the corrector's R. PROS_ERR_F is
// +INFINITY, and the adapter must screen it to hasFix()==false BEFORE asking for an
// error value (HA-08). An Inf R would make sigma infinite and the gate accept anything.
TEST_CASE("[oracle] gpsRmsErrorToCanonical: sentinels and negatives fail loud") {
    const double inf = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK_THROWS_AS((void)gpsRmsErrorToCanonical(inf), PreconditionError);
    CHECK_THROWS_AS((void)gpsRmsErrorToCanonical(nan), PreconditionError);
    CHECK_THROWS_AS((void)gpsRmsErrorToCanonical(-0.01), PreconditionError);
}

namespace {

/// The inch scale as a LITERAL, derived by hand from 1 in ≡ 0.0254 m: 1/0.0254 =
/// 39.3700787401574803… Deliberately NOT kMetersToInches — that is the number under
/// test, and an oracle that imports it proves nothing.
constexpr double kInchesPerMetreByHand = 39.37007874015748;

/// VEX (East, North) metres → canonical (x, y) inches, re-derived from the CONVENTION
/// rather than from gpsSensorPose: VEX-North points along canonical bearing θ_N, and
/// VEX-East is 90° CLOCKWISE of North, so East points along θ_N − 90°. Sum the two
/// basis vectors and scale.
void frameOracle(double eastM, double northM, double northHeadingDeg, double& x, double& y) {
    const double toRad = std::acos(-1.0) / 180.0;
    const double thetaNorth = northHeadingDeg * toRad;
    const double thetaEast = (northHeadingDeg - 90.0) * toRad;
    x = (eastM * std::cos(thetaEast) + northM * std::cos(thetaNorth)) * kInchesPerMetreByHand;
    y = (eastM * std::sin(thetaEast) + northM * std::sin(thetaNorth)) * kInchesPerMetreByHand;
}

/// Robot CENTRE from a SENSOR pose, re-derived from the body-frame definition (F1):
/// forward is the heading unit vector, left is 90° CCW of forward, so the sensor sits
/// at centre + lf·forward + ll·left and the centre is the sensor minus that.
void leverArmOracle(double sensorX, double sensorY, double headingRad, double lf, double ll,
                    double& cx, double& cy) {
    const double fwdX = std::cos(headingRad);
    const double fwdY = std::sin(headingRad);
    const double leftX = std::cos(headingRad + std::acos(-1.0) / 2.0);
    const double leftY = std::sin(headingRad + std::acos(-1.0) / 2.0);
    cx = sensorX - (lf * fwdX + ll * leftX);
    cy = sensorY - (lf * fwdY + ll * leftY);
}

}  // namespace

// Would catch: HA-01 implemented with the axes swapped, an axis negated, or the
// rotation applied in the wrong direction — a MIRRORED pose that northHeadingDeg
// cannot undo. The oracle is built from the compass convention, not from the code, so
// a shared sign error cannot cancel. Non-cardinal θ_N values are included on purpose:
// at θ_N = 90° the rotation is the identity and hides almost everything.
TEST_CASE("[oracle] gpsSensorPose: the East/North→field rotation, from the convention") {
    struct Case {
        double eastM;
        double northM;
        double northHeadingDeg;
    };
    const Case cases[] = {
        {1.0, 0.0, 90.0},    {0.0, 1.0, 90.0},   {2.0, 3.0, 90.0},    {2.0, 3.0, 0.0},
        {2.0, 3.0, 270.0},   {2.0, 3.0, 37.0},   {-1.25, 0.75, 37.0}, {-1.25, 0.75, 143.0},
        {0.6, -2.4, 217.5},  {1.0, 1.0, 45.0},
    };
    for (const Case& c : cases) {
        CAPTURE(c.eastM);
        CAPTURE(c.northM);
        CAPTURE(c.northHeadingDeg);
        double ox = 0.0;
        double oy = 0.0;
        frameOracle(c.eastM, c.northM, c.northHeadingDeg, ox, oy);
        const Pose2d got = gpsSensorPose(c.eastM, c.northM, 0.0, c.northHeadingDeg);
        CHECK(got.x().value() == doctest::Approx(ox).epsilon(1e-12));
        CHECK(got.y().value() == doctest::Approx(oy).epsilon(1e-12));
    }
}

// Would catch: the frame test above passing because BOTH sides are mirrored. These
// four expectations are written out in plain language with no trigonometry at all —
// the default frame (VEX-North = canonical +Y) makes East → +X and North → +Y, and one
// metre East is 39.37 inches of +X. If HA-01 is wrong on the real device, THIS is the
// assertion whose text tells a person what was assumed.
TEST_CASE("[oracle] gpsSensorPose: the default frame in words, with hand-computed inches") {
    const Pose2d east = gpsSensorPose(1.0, 0.0, 0.0);  // 1 m due East
    CHECK(east.x().value() == doctest::Approx(39.37007874015748).epsilon(1e-12));
    CHECK(east.y().value() == doctest::Approx(0.0).epsilon(1e-12));

    const Pose2d north = gpsSensorPose(0.0, 1.0, 0.0);  // 1 m due North
    CHECK(north.x().value() == doctest::Approx(0.0).epsilon(1e-12));
    CHECK(north.y().value() == doctest::Approx(39.37007874015748).epsilon(1e-12));

    // Half a metre East of centre is 19.685 inches — a number you can check on paper.
    const Pose2d half = gpsSensorPose(0.5, 0.0, 0.0);
    CHECK(half.x().value() == doctest::Approx(19.68503937007874).epsilon(1e-12));
}

// Would catch: a lever-arm SIGN flip, the forward/left components swapped, or the
// rotation applied as R(−h) instead of R(h). The pre-E2 cases used one non-zero
// component at a time at headings 0 and 90 only; with both components non-zero at
// seven headings, a cross-term error has nowhere to hide. The oracle is derived from
// "left is 90° CCW of forward", not from gpsRemoveLeverArm.
TEST_CASE("[oracle] gpsRemoveLeverArm: seven headings, both components non-zero") {
    const double headings[] = {0.0, 30.0, 90.0, 135.0, 180.0, -60.0, 217.0};
    const double toRad = std::acos(-1.0) / 180.0;
    struct Arm {
        double forward;
        double left;
    };
    const Arm arms[] = {{6.0, 4.0}, {-3.5, 2.25}, {5.0, -7.5}, {0.0, 3.0}, {4.0, 0.0}};
    for (const double hDeg : headings) {
        for (const Arm& a : arms) {
            CAPTURE(hDeg);
            CAPTURE(a.forward);
            CAPTURE(a.left);
            const Pose2d sensor{Length{10.0}, Length{-20.0}, Angle::degrees(hDeg)};
            double cx = 0.0;
            double cy = 0.0;
            leverArmOracle(10.0, -20.0, hDeg * toRad, a.forward, a.left, cx, cy);
            const Pose2d got = gpsRemoveLeverArm(sensor, Length{a.forward}, Length{a.left});
            CHECK(got.x().value() == doctest::Approx(cx).epsilon(1e-12));
            CHECK(got.y().value() == doctest::Approx(cy).epsilon(1e-12));
            // Heading is never touched by lever-arm removal.
            CHECK(got.heading().approxEqual(Angle::degrees(hDeg)));
        }
    }
}

// Would catch: the oracle above being wrong in the same way as the code. One case
// worked out entirely by hand, with the arithmetic in the comment:
//   heading 30°, sensor 6" forward and 4" left of centre.
//   forward = (cos30, sin30) = (0.8660254037844387, 0.5)
//   left    = (-sin30, cos30) = (-0.5, 0.8660254037844387)
//   offset  = 6·forward + 4·left = (5.196152422706632 − 2, 3 + 3.4641016151377544)
//           = (3.196152422706632, 6.464101615137754)
//   centre  = sensor − offset = (100 − 3.196…, 200 − 6.464…)
TEST_CASE("[oracle] gpsRemoveLeverArm: one case computed by hand, start to finish") {
    const Pose2d sensor{Length{100.0}, Length{200.0}, Angle::degrees(30.0)};
    const Pose2d centre = gpsRemoveLeverArm(sensor, Length{6.0}, Length{4.0});
    CHECK(centre.x().value() == doctest::Approx(96.80384757729337).epsilon(1e-12));
    CHECK(centre.y().value() == doctest::Approx(193.53589838486225).epsilon(1e-12));
}

// Would catch: a lever arm that is right at every heading but wrong in COMPOSITION —
// removing the arm before rotating the frame, say. Full raw→centre path with a
// non-default North alignment and a non-trivial heading, against both oracles chained
// by hand. GPS at 1 m East / 0.5 m North of field centre, compass heading 210° CW from
// North, sensor 8" forward / 3" left, θ_N = 90° (default).
TEST_CASE("[oracle] gpsToRobotPose: raw reading → centre, both oracles chained") {
    const double eastM = 1.0;
    const double northM = 0.5;
    const double compassDeg = 210.0;
    double sx = 0.0;
    double sy = 0.0;
    frameOracle(eastM, northM, 90.0, sx, sy);
    // Canonical heading = θ_N − compass = 90 − 210 = −120°, which Angle wraps to −120°.
    const double headingRad = -120.0 * std::acos(-1.0) / 180.0;
    double cx = 0.0;
    double cy = 0.0;
    leverArmOracle(sx, sy, headingRad, 8.0, 3.0, cx, cy);

    const Pose2d got = gpsToRobotPose(eastM, northM, compassDeg, Length{8.0}, Length{3.0});
    CHECK(got.x().value() == doctest::Approx(cx).epsilon(1e-12));
    CHECK(got.y().value() == doctest::Approx(cy).epsilon(1e-12));
    CHECK(got.heading().approxEqual(Angle::degrees(-120.0)));
}
