// C1 FRAME CORRECTNESS — the classic killer bug class, attacked four ways:
//   1. heading sweep (a frame bug is a heading-DEPENDENT miss),
//   2. rotational equivariance (rotate the ENTIRE problem — start, target,
//      field — by an arbitrary angle: the outcome must rotate with it),
//   3. mirror equivariance (reflect the problem across the field X axis),
//   4. the ±180° seam (F3's exact-antipodal +π tie-break, both approach
//      directions, and a crossing THROUGH the seam).
// Equivariance is the strongest frame test there is: a fieldToRobot/robotToField
// swap, a transposed rotation, or a sign slip survives a fixed-heading test but
// cannot survive a problem rotated by 37°. Mutation #1 (swap the F1 transform)
// must red here.

#include "doctest.h"

#include <cmath>

#include "motion_test_rig.hpp"
#include "shulib/kinematics/x_drive.hpp"
#include "shulib/math/frame.hpp"
#include "shulib/motion/move_to_pose.hpp"
#include "shulib/motion/turn_to.hpp"

using namespace motion_rig;
using shulib::control::ExitReason;
using shulib::kinematics::xDrive;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::motion::MoveToPose;
using shulib::motion::TurnTo;
using shulib::units::Time;

namespace {

/// Rotate a FIELD pose about the origin by `alpha` (position rotates, heading
/// shifts). The independent oracle for the equivariance cases — hand math, not
/// frame.hpp (the code under test must not grade itself).
Pose2d rotated(const Pose2d& p, double alpha) {
    const double c = std::cos(alpha);
    const double s = std::sin(alpha);
    const double x = p.x().value();
    const double y = p.y().value();
    return Pose2d{shulib::units::Length{c * x - s * y}, shulib::units::Length{s * x + c * y},
                  Angle::radians(p.heading().radians() + alpha)};
}

/// Mirror a FIELD pose across the field X axis: y → −y, θ → −θ.
Pose2d mirrored(const Pose2d& p) {
    return Pose2d{p.x(), shulib::units::Length{-p.y().value()},
                  Angle::radians(-p.heading().radians())};
}

/// Run MoveToPose start→target on a fresh rig; returns the final TRUE pose.
Pose2d runMove(const Pose2d& start, const Pose2d& target, double& outTime) {
    const auto kin = xDrive(Length{7.0});
    auto cfg = plantConfig();
    cfg.plant.initialPose = start;
    MotionRig rig{kin, cfg};
    MoveToPose m{rig.deps, target, motionConfig()};
    REQUIRE(rig.run(m, 900) == ExitReason::Settled);
    outTime = rig.h.clock().now().value();
    return rig.h.truePose();
}

}  // namespace

// ── 1. Heading sweep: same FIELD target, every start heading. ──
// Bug caught: any use of the wrong frame (or wrong rotation sign) makes the
// miss grow with the start heading — at 180° a swapped transform drives AWAY
// from the target.
TEST_CASE("C1 frames: a field target is reached regardless of start heading (sweep)") {
    const Pose2d target{Length{18.0}, Length{27.0}, Angle::degrees(60.0)};
    for (const double h0 :
         {0.0, 30.0, 60.0, 90.0, 135.0, 179.0, -179.0, -135.0, -90.0, -45.0, -10.0}) {
        CAPTURE(h0);
        double t = 0.0;
        const Pose2d end =
            runMove(Pose2d{Length{0.0}, Length{0.0}, Angle::degrees(h0)}, target, t);
        CHECK(posErr(end, target) < 0.6);
        CHECK(headErr(end, target) < 0.025);
    }
}

// ── 2. Rotational equivariance: rotate the whole problem by arbitrary angles. ──
// Bug caught: everything the heading sweep catches PLUS any hidden field-axis
// asymmetry (e.g. unequal x/y gains, an axis-ordered clamp) — the rotated
// problem must land on the rotated answer, and take the same time doing it.
TEST_CASE("C1 frames: rotating the entire problem rotates the outcome (equivariance)") {
    const Pose2d start{Length{5.0}, Length{-3.0}, Angle::degrees(20.0)};
    const Pose2d target{Length{30.0}, Length{15.0}, Angle::degrees(80.0)};
    double baseTime = 0.0;
    const Pose2d baseEnd = runMove(start, target, baseTime);

    for (const double alphaDeg : {37.0, -113.0, 90.0, 180.0}) {
        CAPTURE(alphaDeg);
        const double alpha = alphaDeg * Angle::kPi / 180.0;
        double t = 0.0;
        const Pose2d end = runMove(rotated(start, alpha), rotated(target, alpha), t);
        const Pose2d expected = rotated(baseEnd, alpha);
        CHECK(posErr(end, expected) < 0.02);       // fp-trig scale, not control scale
        CHECK(headErr(end, expected) < 5e-4);
        CHECK(std::abs(t - baseTime) < 0.05);      // same problem ⇒ same duration
    }
}

// ── 3. Mirror equivariance: reflect the problem across the field X axis. ──
// Bug caught: a chirality error (CW/CCW slip anywhere in the chain) — a pure
// sign bug that rotational equivariance alone can miss, because a consistent
// handedness error commutes with rotations but NOT with reflections.
TEST_CASE("C1 frames: mirroring the problem mirrors the outcome") {
    const Pose2d start{Length{4.0}, Length{6.0}, Angle::degrees(35.0)};
    const Pose2d target{Length{28.0}, Length{22.0}, Angle::degrees(-70.0)};
    double baseTime = 0.0;
    const Pose2d baseEnd = runMove(start, target, baseTime);
    double t = 0.0;
    const Pose2d end = runMove(mirrored(start), mirrored(target), t);
    const Pose2d expected = mirrored(baseEnd);
    CHECK(posErr(end, expected) < 0.02);
    CHECK(headErr(end, expected) < 5e-4);
    CHECK(std::abs(t - baseTime) < 0.05);
}

// ── 4. The ±180° seam. ──
// Bugs caught: long-way-around turns (raw θ subtraction instead of F3's
// shortest error — the mutation-#8 home); a nondeterministic or CW resolution
// of the exact-antipodal case (F3 pins it to +π = CCW); wrap corruption when a
// motion CROSSES the seam.
TEST_CASE("C1 seam: TurnTo takes the short way from both sides of ±180°") {
    const auto kin = xDrive(Length{7.0});
    struct Case {
        double startDeg;
        double targetDeg;
        double expectedDeltaDeg;  // signed TRUE rotation (unwrapped truth θ)
    };
    const Case cases[] = {
        {170.0, 180.0, +10.0},    // approach the seam CCW
        {-170.0, 180.0, -10.0},   // approach the seam CW (shortest is negative)
        {170.0, -160.0, +30.0},   // CROSS the seam going CCW
        {-170.0, 160.0, -30.0},   // cross it going CW
        {150.0, -170.0, +40.0},   // land past the seam
    };
    for (const Case& c : cases) {
        CAPTURE(c.startDeg);
        CAPTURE(c.targetDeg);
        auto cfg = plantConfig();
        cfg.plant.initialPose = Pose2d{Length{0.0}, Length{0.0}, Angle::degrees(c.startDeg)};
        MotionRig rig{kin, cfg};
        const double theta0 = rig.h.plant().truthState().theta;
        TurnTo m{rig.deps, Angle::degrees(c.targetDeg), motionConfig()};
        REQUIRE(rig.run(m, 600) == ExitReason::Settled);
        const double deltaDeg =
            (rig.h.plant().truthState().theta - theta0) * 180.0 / Angle::kPi;
        // UNWRAPPED truth rotation: the short way, not the 320°+ way around.
        CHECK(deltaDeg == doctest::Approx(c.expectedDeltaDeg).epsilon(0.15));
        CHECK(std::abs(rig.h.truePose().heading().errorTo(Angle::degrees(c.targetDeg))) < 0.025);
    }
}

TEST_CASE("C1 seam: the exact-antipodal turn resolves CCW (+pi), deterministically — F3") {
    const auto kin = xDrive(Length{7.0});
    for (int rep = 0; rep < 2; ++rep) {  // twice: DETERMINISTICALLY the same way
        CAPTURE(rep);
        MotionRig rig{kin};  // start heading exactly 0
        const double theta0 = rig.h.plant().truthState().theta;
        TurnTo m{rig.deps, Angle::radians(Angle::kPi), motionConfig()};
        REQUIRE(rig.run(m, 700) == ExitReason::Settled);
        const double delta = rig.h.plant().truthState().theta - theta0;
        CHECK(delta > 0.0);  // CCW — the F3 tie-break, visible in TRUE rotation
        CHECK(delta == doctest::Approx(Angle::kPi).epsilon(0.02));
    }
}

// ── Heading stays wrapped, every single tick. ──
// Bug caught: an unwrapped heading escaping into the estimate/record stream
// (the "359° vs −1°" class the Angle type exists to kill) during seam-crossing
// motion.
TEST_CASE("C1 seam: the estimate's heading stays in (-pi, pi] through a seam-crossing move") {
    const auto kin = xDrive(Length{7.0});
    auto cfg = plantConfig();
    cfg.plant.initialPose = Pose2d{Length{0.0}, Length{0.0}, Angle::degrees(170.0)};
    MotionRig rig{kin, cfg};
    MoveToPose m{rig.deps, Pose2d{Length{20.0}, Length{-15.0}, Angle::degrees(-150.0)},
                 motionConfig()};
    m.start();
    auto reason = ExitReason::Running;
    for (int i = 0; i < 900 && reason == ExitReason::Running; ++i) {
        rig.loc.update();
        const double h = rig.loc.pose().heading().radians();
        REQUIRE(h > -Angle::kPi);
        REQUIRE(h <= Angle::kPi);
        reason = m.tick();
        if (reason == ExitReason::Running) {
            rig.h.plant().step(Time{0.01});
        }
    }
    CHECK(reason == ExitReason::Settled);
}
