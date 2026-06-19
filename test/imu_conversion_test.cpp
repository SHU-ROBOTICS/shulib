// Adversarial tests for the IMU→canonical conversion — the crux of the < 1° heading
// requirement. Every case targets a specific way this could be wrong: handedness
// (CW vs CCW), the boot offset, the ±180° seam + tie-break, wrap, sub-degree
// precision, and direction monotonicity.

#include "doctest.h"

#include <limits>

#include "shulib/core/check.hpp"
#include "shulib/hal/imu_conversion.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/units/quantity.hpp"

using shulib::PreconditionError;
using shulib::hal::imuHeadingToCanonical;
using shulib::hal::imuYawRateToCanonical;
using shulib::math::Angle;

namespace {
constexpr double kPi = Angle::kPi;
}

TEST_CASE("imuHeadingToCanonical: at calibration the heading equals bootHeading") {
    CHECK(imuHeadingToCanonical(0.0, Angle::degrees(0.0)).approxEqual(Angle::degrees(0.0)));
    CHECK(imuHeadingToCanonical(0.0, Angle::degrees(90.0)).approxEqual(Angle::degrees(90.0)));
    CHECK(imuHeadingToCanonical(0.0, Angle::degrees(-37.0)).approxEqual(Angle::degrees(-37.0)));
}

TEST_CASE("imuHeadingToCanonical: a CW turn DECREASES the canonical heading (handedness)") {
    // IMU reads +90 (turned 90° clockwise) from boot=0 → canonical -90°.
    CHECK(imuHeadingToCanonical(90.0, Angle::degrees(0.0)).approxEqual(Angle::degrees(-90.0)));
    // boot facing +Y (90°), then 90° CW → now facing +X (0°).
    CHECK(imuHeadingToCanonical(90.0, Angle::degrees(90.0)).approxEqual(Angle::degrees(0.0)));
}

TEST_CASE("imuHeadingToCanonical: the ±180° seam resolves to +π, never -π (tie-break)") {
    const Angle r = imuHeadingToCanonical(180.0, Angle::degrees(0.0));
    CHECK(r.approxEqual(Angle::degrees(180.0)));
    CHECK(r.radians() > 0.0);                       // +π, not -π
    CHECK(r.radians() == doctest::Approx(kPi));
}

TEST_CASE("imuHeadingToCanonical: wraps past a full turn") {
    // 270° CW from 0 → wrap(-270°) = +90°.
    CHECK(imuHeadingToCanonical(270.0, Angle::degrees(0.0)).approxEqual(Angle::degrees(90.0)));
    // a full 360° returns to bootHeading.
    CHECK(imuHeadingToCanonical(360.0, Angle::degrees(37.0)).approxEqual(Angle::degrees(37.0)));
    // beyond a full turn (cumulative rotation) still wraps correctly: 450° = 90°.
    CHECK(imuHeadingToCanonical(450.0, Angle::degrees(0.0)).approxEqual(Angle::degrees(-90.0)));
}

TEST_CASE("imuHeadingToCanonical: the boot offset combines additively") {
    // 45° CW from boot=30° → 30 - 45 = -15°.
    CHECK(imuHeadingToCanonical(45.0, Angle::degrees(30.0)).approxEqual(Angle::degrees(-15.0)));
}

TEST_CASE("imuHeadingToCanonical: sub-degree precision is preserved (the < 1° stakes)") {
    // a 0.3° reading must convert to -0.3° with error far below 1°.
    const Angle r = imuHeadingToCanonical(0.3, Angle::degrees(0.0));
    CHECK(std::abs(r.errorTo(Angle::degrees(-0.3))) < 1e-9);  // < 6e-8 degrees of conversion error
}

TEST_CASE("imuHeadingToCanonical: heading decreases monotonically as the IMU reads up (pre-seam)") {
    double prev = imuHeadingToCanonical(0.0, Angle::degrees(0.0)).radians();
    for (int deg = 10; deg <= 170; deg += 10) {
        const double cur = imuHeadingToCanonical(static_cast<double>(deg), Angle::degrees(0.0)).radians();
        CHECK(cur < prev);  // CW-up ⇒ canonical-down, no spurious jump before the seam
        prev = cur;
    }
}

TEST_CASE("imuHeadingToCanonical: a non-finite reading is rejected") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    CHECK_THROWS_AS((void)imuHeadingToCanonical(nan, Angle::degrees(0.0)), PreconditionError);
    CHECK_THROWS_AS((void)imuHeadingToCanonical(inf, Angle::degrees(0.0)), PreconditionError);
}

TEST_CASE("imuHeadingToCanonical: negative readings (CCW cumulative rotation) convert correctly") {
    // get_rotation() goes NEGATIVE on CCW turns — the real hot path, never fed above.
    CHECK(imuHeadingToCanonical(-90.0, Angle::degrees(0.0)).approxEqual(Angle::degrees(90.0)));
    CHECK(imuHeadingToCanonical(-270.0, Angle::degrees(0.0)).approxEqual(Angle::degrees(-90.0)));
    CHECK(imuHeadingToCanonical(-450.0, Angle::degrees(0.0)).approxEqual(Angle::degrees(90.0)));
    // negative antipode must still resolve to +π, never -π (guards an fmod-style wrap)
    const Angle r = imuHeadingToCanonical(-180.0, Angle::degrees(0.0));
    CHECK(r.approxEqual(Angle::degrees(180.0)));
    CHECK(r.radians() > 0.0);
}

TEST_CASE("imuHeadingToCanonical: continuity holds THROUGH the ±180° seam") {
    // sweeping the IMU up across the seam, each shortest-step must be a steady small
    // DECREASE — no spurious jump where the canonical heading folds +π↔-π.
    double prev = imuHeadingToCanonical(170.0, Angle::degrees(0.0)).radians();
    for (int deg = 172; deg <= 190; deg += 2) {
        const Angle cur = imuHeadingToCanonical(static_cast<double>(deg), Angle::degrees(0.0));
        const double step = Angle::radians(prev).errorTo(cur);  // shortest signed delta
        CHECK(step < 0.0);                                       // still decreasing across the seam
        CHECK(step == doctest::Approx(-2.0 * kPi / 180.0));      // exactly -2°, no fold artifact
        prev = cur.radians();
    }
}

TEST_CASE("imuHeadingToCanonical: a bootHeading exactly at the +π boundary composes correctly") {
    const Angle boot = Angle::degrees(180.0);  // stored as +π (F3: never -π)
    CHECK(imuHeadingToCanonical(0.0, boot).approxEqual(Angle::degrees(180.0)));
    CHECK(imuHeadingToCanonical(90.0, boot).approxEqual(Angle::degrees(90.0)));    // 180 - 90
    CHECK(imuHeadingToCanonical(-90.0, boot).approxEqual(Angle::degrees(-90.0)));  // 180 + 90 → wrap
    CHECK(imuHeadingToCanonical(1.0, boot).approxEqual(Angle::degrees(179.0)));
}

TEST_CASE("imuYawRateToCanonical: CW rate becomes a negative canonical rate") {
    CHECK(imuYawRateToCanonical(0.0).value() == doctest::Approx(0.0));
    CHECK(imuYawRateToCanonical(90.0).value() == doctest::Approx(-kPi / 2.0));   // 90°/s CW → -π/2 rad/s
    CHECK(imuYawRateToCanonical(180.0).value() == doctest::Approx(-kPi));
    CHECK(imuYawRateToCanonical(-45.0).value() == doctest::Approx(kPi / 4.0));   // CCW input → positive
}

TEST_CASE("imuYawRateToCanonical: a non-finite rate is rejected (NaN, +inf, -inf)") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    CHECK_THROWS_AS((void)imuYawRateToCanonical(nan), PreconditionError);
    CHECK_THROWS_AS((void)imuYawRateToCanonical(inf), PreconditionError);
    CHECK_THROWS_AS((void)imuYawRateToCanonical(-inf), PreconditionError);
}
