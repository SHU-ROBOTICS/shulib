// C1 FULL-ROUTINE ACCURACY — the test that matters most for real use: long
// hand-chained waypoint routines (the shape of a skills run: translations,
// turns, strafes, reversals), graded against GROUND TRUTH, under A3's composed
// hostility and on the clean plant, swept over routine LENGTH and seeds.
//
// The structural claim under test: shulib targets ABSOLUTE FIELD POSES, so a
// miss on move k must NOT shift move k+1's target — each move re-targets
// absolutely and self-corrects. Error must therefore be attributable to:
//   * move COUNT      — must be FLAT (growth = per-move compounding, a C1
//                       defect). Proven by the CLEAN sweep: with no sensor
//                       lies there is no drift clock, so any count-growth
//                       would be structural — and none is allowed.
//   * total DISTANCE / NET DISPLACEMENT — growth = scale/geometry bias (a
//                       register item: HA-12/13/14, R3's problem). Proven
//                       LIVE by a deliberately 2%-miscalibrated twin whose
//                       arrival-error-vs-displacement slope must read ≈ 2%,
//                       while the calibrated twin reads ≈ 0.
//   * elapsed TIME    — growth = localizer drift (EXPECTED at M2 with no
//                       correctors; Phase E's problem). Reported with slopes.
// TurnTo legs are mixed in deliberately: they add TIME at ~zero DISTANCE,
// decorrelating the two regressors so the univariate slopes mean something.
//
// Scope honesty: v1 is stop-and-settle BY DESIGN (no min-velocity blending —
// an explicit Frontier item). The settle overhead per waypoint is measured and
// reported so the cost of that choice is a number, not a guess.

#include "doctest.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "motion_test_rig.hpp"
#include "shulib/kinematics/x_drive.hpp"
#include "shulib/localization/tracking_wheel.hpp"
#include "shulib/motion/move_to_pose.hpp"
#include "shulib/motion/strafe_to.hpp"
#include "shulib/motion/turn_to.hpp"
#include "shulib/sim/hostile/composed.hpp"
#include "shulib/sim/rng.hpp"

using namespace motion_rig;
using shulib::control::ExitReason;
using shulib::kinematics::xDrive;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::motion::MoveToPose;
using shulib::motion::StrafeTo;
using shulib::motion::TurnTo;
using shulib::sim::FullHostility;
using shulib::sim::Rng;
using shulib::units::Time;

namespace {

constexpr double kMoveTimeout = 8.0;  // covers boot wait + the longest leg

struct WaypointResult {
    double t = 0.0;             // clock at arrival (s)
    double distTravelled = 0.0; // cumulative TRUE path length at arrival (in)
    double displacement = 0.0;  // |target| from the field origin (in)
    double legLength = 0.0;     // |target − previous target| (in)
    double errTrue = 0.0;       // |truth − target| at the settled instant (in)
    double errHead = 0.0;       // |truth heading − target heading| (rad)
    double dx = 0.0;            // signed truth−target, for the bias check (in)
    double dy = 0.0;
};

struct RoutineResult {
    std::vector<WaypointResult> waypoints;
    double finalErr = 0.0;
    double finalHeadErr = 0.0;
    double worstArrival = 0.0;
    double worstHeadArrival = 0.0;
    double totalTime = 0.0;
    double totalDistance = 0.0;
    int moveCount = 0;  // motions executed (incl. TurnTo legs)
};

/// Generate waypoint k+1 from waypoint k: a bounded random hop (10–45 in legs,
/// arbitrary heading), clamped to the ±55 in field. Reversals arise naturally.
Pose2d nextWaypoint(Rng& rng, const Pose2d& from) {
    const double dx = rng.uniform(-35.0, 35.0);
    const double dy = rng.uniform(-35.0, 35.0);
    const double x = std::clamp(from.x().value() + dx, -55.0, 55.0);
    const double y = std::clamp(from.y().value() + dy, -55.0, 55.0);
    return Pose2d{Length{x}, Length{y}, Angle::radians(rng.uniform(-Angle::kPi, Angle::kPi))};
}

/// Run one motion on an existing rig, accumulating true path length.
ExitReason runOn(MotionRig& rig, shulib::motion::IMotion& m, double& distAccum) {
    m.start();
    auto reason = ExitReason::Running;
    Pose2d prev = rig.h.truePose();
    for (int i = 0; i < 1600 && reason == ExitReason::Running; ++i) {
        rig.loc.update();
        reason = m.tick();
        if (reason == ExitReason::Running) {
            rig.h.plant().step(Time{0.01});
        }
        const Pose2d now = rig.h.truePose();
        distAccum += posErr(now, prev);
        prev = now;
    }
    return reason;
}

/// One full hand-chained routine of `n` waypoints (every 3rd followed by a
/// pure TurnTo leg; every 7th an explicit StrafeTo) on a fresh rig.
RoutineResult runRoutine(int n, std::uint64_t seed, shulib::sim::DegradationModel* hostile) {
    const auto kin = xDrive(Length{7.0});
    auto pcfg = plantConfig();
    pcfg.plant.seed = seed;
    MotionRig rig{kin, pcfg, nullptr, hostile};

    Rng wp{seed * 2654435761ULL};
    RoutineResult out;
    Pose2d target{};
    double dist = 0.0;
    for (int k = 0; k < n; ++k) {
        const Pose2d prevTarget = target;
        target = nextWaypoint(wp, target);
        const bool strafeLeg = (k % 7 == 3);  // cadence chosen so NO sweep length
                                              // (5/10/20/40) ENDS on a strafe leg —
                                              // final heading must be truly graded
        ExitReason r = ExitReason::Running;
        if (strafeLeg) {
            StrafeTo m{rig.deps, target.x(), target.y(), motionConfig(), kMoveTimeout};
            r = runOn(rig, m, dist);
            // StrafeTo holds the CURRENT heading; re-stamp the bookkeeping
            // target with what was actually commanded:
            target = Pose2d{target.x(), target.y(), rig.h.truePose().heading() /*≈held*/};
        } else {
            MoveToPose m{rig.deps, target, motionConfig(), kMoveTimeout};
            r = runOn(rig, m, dist);
        }
        REQUIRE(r == ExitReason::Settled);
        ++out.moveCount;

        WaypointResult w;
        w.t = rig.h.clock().now().value();
        w.distTravelled = dist;
        w.displacement = std::hypot(target.x().value(), target.y().value());
        w.legLength = posErr(prevTarget, target);
        w.errTrue = posErr(rig.h.truePose(), target);
        w.errHead = strafeLeg ? 0.0 : headErr(rig.h.truePose(), target);
        w.dx = rig.h.truePose().x().value() - target.x().value();
        w.dy = rig.h.truePose().y().value() - target.y().value();
        out.waypoints.push_back(w);
        out.worstArrival = std::max(out.worstArrival, w.errTrue);
        out.worstHeadArrival = std::max(out.worstHeadArrival, w.errHead);

        if (k % 3 == 2) {  // a pure-rotation leg: time passes, distance doesn't
            const Angle newHeading =
                Angle::radians(wp.uniform(-Angle::kPi, Angle::kPi));
            TurnTo t{rig.deps, newHeading, motionConfig(), kMoveTimeout};
            REQUIRE(runOn(rig, t, dist) == ExitReason::Settled);
            target = Pose2d{target.x(), target.y(), newHeading};
            ++out.moveCount;
        }
    }
    out.finalErr = posErr(rig.h.truePose(), target);
    out.finalHeadErr = headErr(rig.h.truePose(), target);
    out.totalTime = rig.h.clock().now().value();
    out.totalDistance = dist;
    return out;
}

/// Least-squares slope of y against x.
double slope(const std::vector<double>& x, const std::vector<double>& y) {
    const auto n = static_cast<double>(x.size());
    double sx = 0.0;
    double sy = 0.0;
    double sxx = 0.0;
    double sxy = 0.0;
    for (std::size_t i = 0; i < x.size(); ++i) {
        sx += x[i];
        sy += y[i];
        sxx += x[i] * x[i];
        sxy += x[i] * y[i];
    }
    const double denom = n * sxx - sx * sx;
    return (denom != 0.0) ? (n * sxy - sx * sy) / denom : 0.0;
}

}  // namespace

// ═══ CLEAN sweep: the count-compounding pin ════════════════════════════════════════
// With perfect sensors there is no drift clock: any error growth with routine
// length would be STRUCTURAL per-move compounding — the defect absolute
// targeting exists to make impossible. Every length must land inside
// tolerance-class error, 5 moves or 40.
TEST_CASE("C1 routine: CLEAN plant — error is FLAT in move count (5/10/20/40), tolerance-class") {
    double sumDx = 0.0;
    double sumDy = 0.0;
    double sumSettleOverhead = 0.0;
    int arrivals = 0;
    for (const int n : {5, 10, 20, 40}) {
        CAPTURE(n);
        const RoutineResult r = runRoutine(n, 77, nullptr);
        MESSAGE("clean n=", n, " moves=", r.moveCount, " finalErr=", r.finalErr,
                "in worstArrival=", r.worstArrival, "in time=", r.totalTime,
                "s dist=", r.totalDistance, "in");
        CHECK(r.finalErr < 0.8);        // ≈ the settle tolerance, regardless of length
        CHECK(r.worstArrival < 0.8);    // …at EVERY waypoint, not just the last
        CHECK(r.finalHeadErr < 0.03);
        for (const WaypointResult& w : r.waypoints) {
            sumDx += w.dx;
            sumDy += w.dy;
            ++arrivals;
        }
        // stop-and-settle overhead: time beyond the ideal cruise, per waypoint
        sumSettleOverhead += (r.totalTime - r.totalDistance / 60.0)
                             / static_cast<double>(r.moveCount);
    }
    // Directional bias: settling must not systematically exit on one side.
    const double meanDx = sumDx / arrivals;
    const double meanDy = sumDy / arrivals;
    MESSAGE("clean arrival bias: mean dx=", meanDx, "in mean dy=", meanDy, "in over ",
            arrivals, " arrivals");
    CHECK(std::abs(meanDx) < 0.15);
    CHECK(std::abs(meanDy) < 0.15);
    MESSAGE("stop-and-settle overhead ~", sumSettleOverhead / 4.0,
            " s/motion (provisional gains; informs the Frontier blending item)");
}

// ═══ HOSTILE sweep: bounded error, attributed by regression ════════════════════════
// Under the full composed A3 world (drift, noise, quantization, sag, slip,
// latency, calibration), end-of-routine error must stay BOUNDED and must not
// blow up with move count. The three univariate slopes are computed over every
// waypoint of every routine and REPORTED — count-attribution comes from the
// clean sweep above (structurally flat), displacement-attribution from the
// miscalibration twin below, so the residual time-slope is the M2 drift story
// Phase E exists to close.
TEST_CASE("C1 routine: HOSTILE sweep — bounded end error across 5/10/20/40 moves, slopes reported") {
    std::vector<double> vCount;
    std::vector<double> vTime;
    std::vector<double> vDist;
    std::vector<double> vErr;
    double worstFinal = 0.0;
    double worstArrival = 0.0;
    double worstHead = 0.0;
    for (const int n : {5, 10, 20, 40}) {
        for (const std::uint64_t seed : {11ULL, 22ULL}) {
            CAPTURE(n);
            CAPTURE(seed);
            FullHostility world{};
            const RoutineResult r = runRoutine(n, seed, &world.model());
            MESSAGE("hostile n=", n, " seed=", seed, " moves=", r.moveCount,
                    " finalErr=", r.finalErr, "in worstArrival=", r.worstArrival,
                    "in headFinal=", r.finalHeadErr * 180.0 / Angle::kPi,
                    "deg time=", r.totalTime, "s dist=", r.totalDistance, "in");
            worstFinal = std::max(worstFinal, r.finalErr);
            worstArrival = std::max(worstArrival, r.worstArrival);
            worstHead = std::max(worstHead, std::max(r.finalHeadErr, r.worstHeadArrival));
            int idx = 0;
            for (const WaypointResult& w : r.waypoints) {
                vCount.push_back(static_cast<double>(++idx));
                vTime.push_back(w.t);
                vDist.push_back(w.distTravelled);
                vErr.push_back(w.errTrue);
            }
        }
    }
    // The bound the team can rely on at M2 (no correctors yet). Derivation,
    // not a wish: with dead-reckon-only fusion the dominant term is IMU drift
    // rotating the odometry frame — err ≲ Σ |leg| · headErr(t). At the HA-20
    // worst bias (1°/min) over ~95 s and ~900 in of path the PESSIMISTIC
    // ceiling is ~12 in; observed worst across the sweep is ~4.1 in (typical
    // boots draw well under the bound). 5.0 in = observed + margin, still 2.4×
    // inside the physics ceiling — Phase E's correctors are what shrink it.
    CHECK(worstArrival < 5.0);
    CHECK(worstFinal < 5.0);
    CHECK(worstHead < 0.06);  // ~3.4°: drift-class, not failure-class
    MESSAGE("HOSTILE worst: arrival=", worstArrival, "in final=", worstFinal,
            "in heading=", worstHead * 180.0 / Angle::kPi, "deg");
    MESSAGE("slopes: err-vs-moveIndex=", slope(vCount, vErr),
            " in/move | err-vs-time=", slope(vTime, vErr),
            " in/s | err-vs-distance=", slope(vDist, vErr), " in/in");
}

// ═══ The count-vs-time discriminator: same ground covered, twice the moves ═════════
// Family A: 8 moves × ~30 in. Family B: 16 moves × ~15 in — same total
// distance, MORE moves. If any per-move compounding existed, B would carry
// ~2× A's error. It must not: the only extra cost B may pay is the extra
// TIME its additional settle tails take (drift).
TEST_CASE("C1 routine: halving leg length while doubling move count does not compound error") {
    auto runFamily = [&](int n, double leg, std::uint64_t seed) {
        const auto kin = xDrive(Length{7.0});
        auto pcfg = plantConfig();
        pcfg.plant.seed = seed;
        FullHostility world{};
        MotionRig rig{kin, pcfg, nullptr, &world.model()};
        Rng dir{seed};
        Pose2d target{};
        double dist = 0.0;
        double x = 0.0;
        double y = 0.0;
        for (int k = 0; k < n; ++k) {
            // a bounded random walk with fixed leg length (stays on the field)
            const double ang = dir.uniform(-Angle::kPi, Angle::kPi);
            double nx = std::clamp(x + leg * std::cos(ang), -50.0, 50.0);
            double ny = std::clamp(y + leg * std::sin(ang), -50.0, 50.0);
            x = nx;
            y = ny;
            target = Pose2d{Length{x}, Length{y}, Angle::radians(dir.uniform(-1.0, 1.0))};
            MoveToPose m{rig.deps, target, motionConfig(), kMoveTimeout};
            REQUIRE(runOn(rig, m, dist) == ExitReason::Settled);
        }
        return std::pair{posErr(rig.h.truePose(), target), rig.h.clock().now().value()};
    };
    double errA = 0.0;
    double errB = 0.0;
    double tA = 0.0;
    double tB = 0.0;
    for (const std::uint64_t seed : {5ULL, 6ULL, 7ULL}) {
        const auto [ea, ta] = runFamily(8, 30.0, seed);
        const auto [eb, tb] = runFamily(16, 15.0, seed);
        errA += ea / 3.0;
        errB += eb / 3.0;
        tA += ta / 3.0;
        tB += tb / 3.0;
    }
    MESSAGE("family A (8x30in): meanErr=", errA, "in meanTime=", tA,
            "s | family B (16x15in): meanErr=", errB, "in meanTime=", tB, "s");
    // B may pay drift for its extra settle time; it must NOT pay per-move error:
    CHECK(errB < errA + 2.0);
    CHECK(errB < 4.0);
}

// ═══ The DISTANCE/DISPLACEMENT attribution, demonstrated live ══════════════════════
// A 2% tracking-wheel diameter miscalibration (the A2 anti-agreeable trick)
// makes the estimate read 2% long, so the robot settles ~2% of its net
// displacement short of every target: arrival error vs |target| must regress
// at ≈ 2% — while the calibrated twin regresses at ≈ 0. THIS is the signature
// R3's calibration walk exists to remove (HA-12/13/14 blast radius), and the
// routine diagnostics demonstrably catch it.
TEST_CASE("C1 routine: a 2% wheel miscalibration shows as an error-vs-displacement slope of ~2%") {
    const auto kin = xDrive(Length{7.0});
    auto runScaled = [&](double diameterScale) {
        auto pcfg = plantConfig();
        shulib::sim::SimHarness h{kin, pcfg};
        // Odometry configured with a WRONG diameter (plant synthesizes 2.0):
        auto fwd = shulib::localization::TrackingWheel::forward(
            h.forwardEncoder(), Length{2.0 * diameterScale}, pcfg.forwardWheelLeftOffset);
        auto lat = shulib::localization::TrackingWheel::lateral(
            h.lateralEncoder(), Length{2.0 * diameterScale}, pcfg.lateralWheelForwardOffset);
        shulib::localization::PilonsOdometry odom{h.imu(), fwd, lat};
        shulib::localization::ComplementaryFusion fusion{};
        shulib::localization::Localizer loc{h.clock(), h.imu(), odom, fusion};
        shulib::hal::fake::FakeTelemetrySink sink;
        shulib::diag::FaultLatch latch{sink, h.clock()};
        shulib::diag::HealthMonitor health{latch};
        shulib::motion::MotionDeps deps{.ctx = &h.context(), .localizer = &loc,
                                        .kinematics = &kin, .faults = &latch,
                                        .health = &health};
        // Waypoints marching outward so displacement varies 10 → 50 in:
        std::vector<double> disp;
        std::vector<double> err;
        for (const double d : {10.0, 20.0, 30.0, 40.0, 50.0}) {
            const Pose2d target{Length{d}, Length{d * 0.4}, Angle::degrees(15.0)};
            MoveToPose m{deps, target, motionConfig(), kMoveTimeout};
            m.start();
            auto reason = ExitReason::Running;
            for (int i = 0; i < 1600 && reason == ExitReason::Running; ++i) {
                loc.update();
                reason = m.tick();
                if (reason == ExitReason::Running) {
                    h.plant().step(Time{0.01});
                }
            }
            REQUIRE(reason == ExitReason::Settled);
            disp.push_back(std::hypot(target.x().value(), target.y().value()));
            err.push_back(posErr(h.truePose(), target));
        }
        return slope(disp, err);
    };
    const double calibrated = runScaled(1.0);
    const double miscal = runScaled(1.02);
    MESSAGE("error-vs-displacement slope: calibrated=", calibrated, " miscal(2%)=", miscal);
    CHECK(std::abs(calibrated) < 0.005);          // no scale bias when calibrated
    CHECK(miscal > 0.010);                        // the 2% lie is VISIBLE…
    CHECK(miscal < 0.035);                        // …and quantitatively ~2%
}
