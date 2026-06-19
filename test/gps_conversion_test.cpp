// Adversarial tests for the GPS→canonical conversion. Each case targets a way the
// frame math could be wrong: heading handedness (CW-from-North vs CCW-from-+X), the
// North-alignment offset, the position-frame rotation (tested at NON-trivial θ_N so a
// rotation sign error can't hide behind the identity case), the meters→inches scale,
// and lever-arm removal direction.

#include "doctest.h"

#include <limits>

#include "shulib/core/check.hpp"
#include "shulib/hal/gps_conversion.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/units/quantity.hpp"

using shulib::PreconditionError;
using shulib::hal::gpsHeadingToCanonical;
using shulib::hal::gpsRemoveLeverArm;
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
// Before a scored run: place the GPS at a known +1 m-East / +1 m-North point at a known
// heading, read raw get_position()/get_heading(), and replace the expected values below
// with the MEASURED raw→canonical mapping. Until then this records the missing oracle.
TEST_CASE("gpsSensorPose: FIELD-CAL axis oracle — bench-measure before trusting" * doctest::skip()) {
    const Pose2d east = gpsSensorPose(1.0, 0.0, 0.0);
    CHECK(east.x().value() == doctest::Approx(kMetersToInches));   // ASSUMED East → +X
    const Pose2d north = gpsSensorPose(0.0, 1.0, 0.0);
    CHECK(north.y().value() == doctest::Approx(kMetersToInches));  // ASSUMED North → +Y
}
