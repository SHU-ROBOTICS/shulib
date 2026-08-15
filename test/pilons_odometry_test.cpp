// Adversarial tests for PilonsOdometry — tracking-wheel dead-reckoning with IMU-owned heading.
// Endpoints come from /tmp/odom_cases.py (which composes the independently-verified arcStep with
// the forward-sim-verified offset correction). The keystone is PURE IN-PLACE ROTATION: with
// offset wheels it MUST produce zero position change — the test that pins both offset signs.
//
// Test rig: diameter = 2.0  →  radius 1.0  →  a shaft reading in radians equals inches of travel,
// so a wheel's travel is set directly via FakeRotation::setPosition.

#include <cmath>
#include <limits>

#include "doctest.h"

#include "shulib/core/check.hpp"
#include "shulib/hal/fake/fake_imu.hpp"
#include "shulib/hal/fake/fake_rotation.hpp"
#include "shulib/localization/pilons_odometry.hpp"
#include "shulib/localization/tracking_wheel.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/units/quantity.hpp"

using shulib::hal::fake::FakeImu;
using shulib::hal::fake::FakeRotation;
using shulib::localization::PilonsOdometry;
using shulib::localization::PilonsOdometryConfig;
using shulib::localization::TrackingWheel;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::units::AngleDim;
using shulib::units::Length;

namespace {
constexpr double kPi = Angle::kPi;
constexpr double kDeg = kPi / 180.0;

TrackingWheel fwdWheel(FakeRotation& r, double leftOffset) {
    return TrackingWheel::forward(r, Length{2.0}, Length{leftOffset});
}
TrackingWheel latWheel(FakeRotation& r, double forwardOffset) {
    return TrackingWheel::lateral(r, Length{2.0}, Length{forwardOffset});
}
}  // namespace

// KEYSTONE: a pure in-place rotation with OFFSET wheels must not move the position at all.
// A flipped offset sign turns a turn-in-place into a phantom translation — the classic odom bug.
TEST_CASE("PilonsOdometry: pure CCW in-place rotation with offset wheels does not translate") {
    FakeImu imu;
    FakeRotation fwdRot, latRot;
    imu.setHeading(Angle::degrees(0.0));
    const double bF = 5.0, aL = -3.0;
    PilonsOdometry odom{imu, fwdWheel(fwdRot, bF), latWheel(latRot, aL)};

    const double dth = 30.0 * kDeg;
    imu.setHeading(Angle::degrees(30.0));
    fwdRot.setPosition(AngleDim{-dth * bF});  // exactly what a pure rotation makes each wheel read
    latRot.setPosition(AngleDim{dth * aL});
    odom.update();

    CHECK(odom.pose().x().value() == doctest::Approx(0.0));
    CHECK(odom.pose().y().value() == doctest::Approx(0.0));
    CHECK(odom.pose().heading().degrees() == doctest::Approx(30.0));
}

// Mirror with NEGATIVE Δθ — pins that the offset correction uses SIGNED Δθ, not |Δθ|.
TEST_CASE("PilonsOdometry: pure CLOCKWISE in-place rotation also does not translate") {
    FakeImu imu;
    FakeRotation fwdRot, latRot;
    imu.setHeading(Angle::degrees(0.0));
    const double bF = 4.0, aL = 2.0;
    PilonsOdometry odom{imu, fwdWheel(fwdRot, bF), latWheel(latRot, aL)};

    const double dth = -40.0 * kDeg;  // clockwise
    imu.setHeading(Angle::degrees(-40.0));
    fwdRot.setPosition(AngleDim{-dth * bF});
    latRot.setPosition(AngleDim{dth * aL});
    odom.update();

    CHECK(odom.pose().x().value() == doctest::Approx(0.0));
    CHECK(odom.pose().y().value() == doctest::Approx(0.0));
    CHECK(odom.pose().heading().degrees() == doctest::Approx(-40.0));
}

// Pure rotation ACROSS the ±180° seam (170° → −170° is +20° the short way). Pins errorTo (not a
// raw h1−h0 subtraction, which would integrate −340°) and prevHeading carry across the wrap.
TEST_CASE("PilonsOdometry: pure rotation across the ±180° seam takes the short way, no drift") {
    FakeImu imu;
    FakeRotation fwdRot, latRot;
    imu.setHeading(Angle::degrees(170.0));
    const double bF = 4.0, aL = 2.0;
    PilonsOdometry odom{imu, fwdWheel(fwdRot, bF), latWheel(latRot, aL)};

    const double dth = 20.0 * kDeg;  // 170 → −170 short way
    imu.setHeading(Angle::degrees(-170.0));
    fwdRot.setPosition(AngleDim{-dth * bF});
    latRot.setPosition(AngleDim{dth * aL});
    odom.update();

    CHECK(odom.pose().x().value() == doctest::Approx(0.0));
    CHECK(odom.pose().y().value() == doctest::Approx(0.0));
    CHECK(odom.pose().heading().degrees() == doctest::Approx(-170.0));
    CHECK_FALSE(odom.lastDeltaImplausible());
}

// Forward travel across the seam — pins arcStep's AVERAGE-heading rotation through the wrap
// (175° → −175° is +10°, so the average heading is exactly 180°).
TEST_CASE("PilonsOdometry: forward travel across the ±180° seam rotates by the average heading") {
    FakeImu imu;
    FakeRotation fwdRot, latRot;
    imu.setHeading(Angle::degrees(175.0));
    PilonsOdometry odom{imu, fwdWheel(fwdRot, 0.0), latWheel(latRot, 0.0)};

    imu.setHeading(Angle::degrees(-175.0));
    fwdRot.setPosition(AngleDim{6.0});
    odom.update();

    CHECK(odom.pose().x().value() == doctest::Approx(-5.992387464));
    CHECK(odom.pose().y().value() == doctest::Approx(0.0));
    CHECK(odom.pose().heading().degrees() == doctest::Approx(-175.0));
}

TEST_CASE("PilonsOdometry: straight drive at heading 90° moves purely +Y") {
    FakeImu imu;
    FakeRotation fwdRot, latRot;
    imu.setHeading(Angle::degrees(90.0));
    PilonsOdometry odom{imu, fwdWheel(fwdRot, 0.0), latWheel(latRot, 0.0),
                        Pose2d{Length{0.0}, Length{0.0}, Angle::degrees(90.0)}};

    fwdRot.setPosition(AngleDim{12.0});  // 12" forward, no heading change
    odom.update();

    CHECK(odom.pose().x().value() == doctest::Approx(0.0));
    CHECK(odom.pose().y().value() == doctest::Approx(12.0));
    CHECK(odom.pose().heading().degrees() == doctest::Approx(90.0));
}

// Combined holonomic arc with offset wheels and a nonzero start pose (oracle case C2).
TEST_CASE("PilonsOdometry: holonomic arc accumulates onto the start pose") {
    FakeImu imu;
    FakeRotation fwdRot, latRot;
    imu.setHeading(Angle::degrees(25.0));
    PilonsOdometry odom{imu, fwdWheel(fwdRot, 2.5), latWheel(latRot, -3.25),
                        Pose2d{Length{10.0}, Length{20.0}, Angle::degrees(25.0)}};

    imu.setHeading(Angle::degrees(45.0));
    fwdRot.setPosition(AngleDim{4.127335374});  // measured travels that yield center (5, 2)
    latRot.setPosition(AngleDim{0.865535986});
    odom.update();

    CHECK(odom.pose().x().value() == doctest::Approx(12.933660180));
    CHECK(odom.pose().y().value() == doctest::Approx(24.483343376));
    CHECK(odom.pose().heading().degrees() == doctest::Approx(45.0));
}

// Two ticks compose: straight, then a quarter-arc. Pins that state carries across update()s.
TEST_CASE("PilonsOdometry: accumulates across multiple ticks") {
    FakeImu imu;
    FakeRotation fwdRot, latRot;
    imu.setHeading(Angle::degrees(0.0));
    PilonsOdometry odom{imu, fwdWheel(fwdRot, 0.0), latWheel(latRot, 0.0)};

    fwdRot.setPosition(AngleDim{5.0});      // tick 1: 5" straight at heading 0
    odom.update();
    CHECK(odom.pose().x().value() == doctest::Approx(5.0));
    CHECK(odom.pose().y().value() == doctest::Approx(0.0));

    imu.setHeading(Angle::degrees(90.0));   // tick 2: another 5" while turning 0→90°
    fwdRot.setPosition(AngleDim{10.0});
    odom.update();
    CHECK(odom.pose().x().value() == doctest::Approx(8.183098862));
    CHECK(odom.pose().y().value() == doctest::Approx(3.183098862));
    CHECK(odom.pose().heading().degrees() == doctest::Approx(90.0));
}

// Heading advances to a NON-initial value every tick, so tick 2's start heading must be tick 1's
// end heading — pins that prevHeading_ is carried forward (not stuck at the construction value).
TEST_CASE("PilonsOdometry: each tick's arc starts from the previous tick's heading") {
    FakeImu imu;
    FakeRotation fwdRot, latRot;
    imu.setHeading(Angle::degrees(0.0));
    PilonsOdometry odom{imu, fwdWheel(fwdRot, 0.0), latWheel(latRot, 0.0)};

    imu.setHeading(Angle::degrees(30.0));   // tick 1: 0 → 30°, 4" forward
    fwdRot.setPosition(AngleDim{4.0});
    odom.update();

    imu.setHeading(Angle::degrees(60.0));   // tick 2: 30 → 60° (NOT 0 → 60), another 4"
    fwdRot.setPosition(AngleDim{8.0});
    odom.update();

    CHECK(odom.pose().x().value() == doctest::Approx(6.615946745));
    CHECK(odom.pose().y().value() == doctest::Approx(3.819718634));
    CHECK(odom.pose().heading().degrees() == doctest::Approx(60.0));
}

TEST_CASE("PilonsOdometry: a plausible tick is not flagged; an oversized Δθ is") {
    FakeImu imu;
    FakeRotation fwdRot, latRot;
    imu.setHeading(Angle::degrees(0.0));
    PilonsOdometry odom{imu, fwdWheel(fwdRot, 0.0), latWheel(latRot, 0.0)};  // gate = π/2 (90°)

    imu.setHeading(Angle::degrees(30.0));
    odom.update();
    CHECK_FALSE(odom.lastDeltaImplausible());

    imu.setHeading(Angle::degrees(130.0));  // +100° in one tick > 90° gate
    odom.update();
    CHECK(odom.lastDeltaImplausible());
}

// A flagged (oversized-Δθ) tick STILL integrates its finite motion (flag-but-integrate policy).
TEST_CASE("PilonsOdometry: a flagged oversized-Δθ tick still integrates its finite motion") {
    FakeImu imu;
    FakeRotation fwdRot, latRot;
    imu.setHeading(Angle::degrees(0.0));
    PilonsOdometry odom{imu, fwdWheel(fwdRot, 0.0), latWheel(latRot, 0.0)};

    imu.setHeading(Angle::degrees(100.0));  // > 90° gate
    fwdRot.setPosition(AngleDim{5.0});
    odom.update();

    CHECK(odom.lastDeltaImplausible());
    CHECK(odom.pose().x().value() == doctest::Approx(2.821266394));  // motion still integrated
    CHECK(odom.pose().y().value() == doctest::Approx(3.362254361));
}

// Boundary: |Δθ| exactly AT the threshold is not flagged (strict >, not >=).
TEST_CASE("PilonsOdometry: Δθ exactly at the gate threshold is not flagged") {
    FakeImu imu;
    FakeRotation fwdRot, latRot;
    imu.setHeading(Angle::degrees(0.0));
    const double thresh = Angle::degrees(0.0).errorTo(Angle::degrees(30.0));  // exact Δθ this tick makes
    PilonsOdometry odom{imu, fwdWheel(fwdRot, 0.0), latWheel(latRot, 0.0), Pose2d{},
                        PilonsOdometryConfig{.maxTickRotation = shulib::units::AngleDim{thresh}}};

    imu.setHeading(Angle::degrees(30.0));
    odom.update();
    CHECK_FALSE(odom.lastDeltaImplausible());  // |Δθ| > thresh is false at equality
}

// A non-finite sensor reading must FREEZE the pose, never poison it with NaN.
TEST_CASE("PilonsOdometry: a non-finite sensor reading freezes the pose instead of poisoning it") {
    FakeImu imu;
    FakeRotation fwdRot, latRot;
    imu.setHeading(Angle::degrees(0.0));
    PilonsOdometry odom{imu, fwdWheel(fwdRot, 0.0), latWheel(latRot, 0.0),
                        Pose2d{Length{3.0}, Length{4.0}, Angle{}}};

    fwdRot.setPosition(AngleDim{std::numeric_limits<double>::quiet_NaN()});
    odom.update();

    CHECK(std::isfinite(odom.pose().x().value()));
    CHECK(std::isfinite(odom.pose().y().value()));
    CHECK(odom.pose().x().value() == doctest::Approx(3.0));  // frozen at last good
    CHECK(odom.pose().y().value() == doctest::Approx(4.0));
    CHECK(odom.lastDeltaImplausible());
}

TEST_CASE("PilonsOdometry: setPose teleports position; heading stays IMU-owned") {
    FakeImu imu;
    FakeRotation fwdRot, latRot;
    imu.setHeading(Angle::degrees(45.0));
    PilonsOdometry odom{imu, fwdWheel(fwdRot, 0.0), latWheel(latRot, 0.0)};

    odom.setPose(Pose2d{Length{5.0}, Length{-7.0}, Angle::degrees(999.0)});  // heading ignored
    CHECK(odom.pose().x().value() == doctest::Approx(5.0));
    CHECK(odom.pose().y().value() == doctest::Approx(-7.0));
    CHECK(odom.pose().heading().degrees() == doctest::Approx(45.0));  // from the IMU, not 999

    // A no-motion tick after the teleport must not inject any phantom drift or rotation.
    odom.update();
    CHECK(odom.pose().x().value() == doctest::Approx(5.0));
    CHECK(odom.pose().y().value() == doctest::Approx(-7.0));
    CHECK(odom.pose().heading().degrees() == doctest::Approx(45.0));
}

// setPose must NOT re-baseline the wheels: travel rolled before the teleport still counts after it.
TEST_CASE("PilonsOdometry: setPose preserves wheel baselines; a following move integrates from it") {
    FakeImu imu;
    FakeRotation fwdRot, latRot;
    imu.setHeading(Angle::degrees(0.0));
    PilonsOdometry odom{imu, fwdWheel(fwdRot, 0.0), latWheel(latRot, 0.0)};  // baselines reset to 0

    fwdRot.setPosition(AngleDim{2.0});  // wheel rolled 2" before the teleport (no update yet)
    odom.setPose(Pose2d{Length{5.0}, Length{5.0}, Angle{}});
    odom.update();                      // that 2" must now count, from the teleported origin
    CHECK(odom.pose().x().value() == doctest::Approx(7.0));  // 5 + 2
    CHECK(odom.pose().y().value() == doctest::Approx(5.0));
}

// Construction re-baselines the wheels: a shaft total accrued before the odom exists is not travel.
TEST_CASE("PilonsOdometry: construction re-baselines the wheels (pre-spin is not counted)") {
    FakeImu imu;
    FakeRotation fwdRot, latRot;
    imu.setHeading(Angle::degrees(0.0));
    auto fwd = fwdWheel(fwdRot, 0.0);    // wheel baseline captured at 0
    auto lat = latWheel(latRot, 0.0);
    fwdRot.setPosition(AngleDim{50.0});  // 50" spun between wheel- and odom-construction
    PilonsOdometry odom{imu, fwd, lat};  // ctor reset() re-baselines to 50

    odom.update();                       // no further motion
    CHECK(odom.pose().x().value() == doctest::Approx(0.0));  // no phantom 50" jump
    CHECK(odom.pose().y().value() == doctest::Approx(0.0));
}

// Heading is IMU-owned from construction: the seeded pose's heading is informational only.
TEST_CASE("PilonsOdometry: pose heading is IMU-owned from construction") {
    FakeImu imu;
    FakeRotation fwdRot, latRot;
    imu.setHeading(Angle::degrees(10.0));
    PilonsOdometry odom{imu, fwdWheel(fwdRot, 0.0), latWheel(latRot, 0.0),
                        Pose2d{Length{3.0}, Length{4.0}, Angle::degrees(99.0)}};  // 99° seeded

    CHECK(odom.pose().x().value() == doctest::Approx(3.0));
    CHECK(odom.pose().y().value() == doctest::Approx(4.0));
    CHECK(odom.pose().heading().degrees() == doctest::Approx(10.0));  // IMU wins over the seed
}

TEST_CASE("PilonsOdometry: swapped wheel roles are rejected at construction") {
    FakeImu imu;
    FakeRotation r1, r2;
    auto fwd = fwdWheel(r1, 0.0);
    auto lat = latWheel(r2, 0.0);
    CHECK_THROWS_AS((PilonsOdometry{imu, lat, fwd}), shulib::PreconditionError);  // forward/lateral swapped
}

TEST_CASE("PilonsOdometry: rejects a non-positive plausibility bound") {
    FakeImu imu;
    FakeRotation fwdRot, latRot;
    CHECK_THROWS_AS((PilonsOdometry{imu, fwdWheel(fwdRot, 0.0), latWheel(latRot, 0.0), Pose2d{},
                                    PilonsOdometryConfig{.maxTickRotation = shulib::units::AngleDim{0.0}}}),
                    shulib::PreconditionError);
}

// Bug caught (DEFECTS1 item N1 — found by this chunk's triage, NOT on DOCS2's list): this
// class gated |Δθ| and never |Δtravel|, and the asymmetry was not cosmetic. A tracking pod
// that is dead or not yet enumerated at construction reads 0, so TrackingWheel baselines at 0;
// on the tick it finally answers, it differences the pod's TRUE cumulative position against
// that 0 and injects a one-tick phantom translation. Measured at 28.4 in for a pod waking at
// 1000 degrees, and unbounded in general — a pod enumerating after several seconds of
// powered-down shaft movement reports far more — a pose teleport that the odometry's own plausibility check
// waved through, because the only thing it checked was heading.
TEST_CASE("N1: a pod that enumerates late cannot teleport the pose") {
    FakeImu imu;
    FakeRotation fwdRot, latRot;
    imu.setHeading(Angle::degrees(0.0));
    fwdRot.setPosition(shulib::units::AngleDim{0.0});   // dead/unenumerated at construction
    PilonsOdometry odom{imu, fwdWheel(fwdRot, 0.0), latWheel(latRot, 0.0), Pose2d{}};

    odom.update();
    REQUIRE_FALSE(odom.lastDeltaImplausible());
    const double xBefore = odom.pose().x().value();

    // The pod enumerates and reports its true cumulative position: 1000 degrees.
    fwdRot.setPosition(shulib::units::AngleDim{5000.0 * Angle::kPi / 180.0});
    odom.update();

    // FLAGGED, where it used to pass in complete silence. Note the honest scope: the delta is
    // reported, not withheld — this class has no clock, so the bound is dt-blind, and a gate
    // that can silently stop accumulating odometry on a slow loop is worse than the jump it
    // would prevent. HealthMonitor is what turns this flag into a fault.
    CHECK(odom.lastDeltaImplausible());
    CHECK(odom.pose().x().value() != doctest::Approx(xBefore));  // still integrated, by design

    // NEGATIVE CONTROL: an ordinary tick of real motion is NOT flagged, so the gate marks the
    // teleport rather than marking everything.
    const double xAfterJump = odom.pose().x().value();
    fwdRot.setPosition(shulib::units::AngleDim{5000.0 * Angle::kPi / 180.0 + 0.2});
    odom.update();
    CHECK_FALSE(odom.lastDeltaImplausible());
    CHECK(odom.pose().x().value() > xAfterJump);
}
