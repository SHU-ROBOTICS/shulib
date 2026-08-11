// C3 ROUTINE ACCURACY, X vs H — the M2 DoD made quantitative: the SAME
// scheduler-driven waypoint routine (same generator, same seeds, same cadence —
// literally the same auton) runs on the 24″ X-bot and the 15″ H-bot, and the
// H-bot's error must obey the same STRUCTURE:
//
//   * error vs MOVE COUNT — FLAT (the clean sweep: no drift clock exists, so any
//     growth would be per-move compounding — an authority-clamp interaction
//     defect C1's X-only suite could not have seen);
//   * error vs TIME — the M2 drift carrier. The H-bot pays MORE total time for
//     the same waypoints (strafe-limited legs), so its hostile error may run
//     higher than X's BY THE DRIFT PHYSICS — that difference is information,
//     reported side by side, never hidden;
//   * error vs DISTANCE — near-flat (calibrated tracking wheels are drivetrain-
//     independent; the H-bot uses the identical odometry).
//
// Everything runs through the C2 MotionScheduler (async + waitUntilSettled on a
// PlantPacer) — so this file is simultaneously the at-scale proof that the
// scheduler runs full H-drive routines with zero drivetrain awareness.

#include "doctest.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "motion_test_rig.hpp"
#include "shulib/kinematics/x_drive.hpp"
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

namespace {

// H legs can be strafe-limited; the budget covers boot + the longest lateral leg
// on BOTH drivetrains (identical budget so the comparison stays honest).
constexpr double kMoveTimeout = 14.0;

struct WaypointResult {
    double t = 0.0;
    double distTravelled = 0.0;
    double errTrue = 0.0;
    double errHead = 0.0;
    double dx = 0.0;
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
    int moveCount = 0;
};

/// C1's waypoint generator, verbatim (same draws for the same seed — the "same
/// auton" is literal: identical waypoint lists on both drivetrains).
Pose2d nextWaypoint(Rng& rng, const Pose2d& from) {
    const double dx = rng.uniform(-35.0, 35.0);
    const double dy = rng.uniform(-35.0, 35.0);
    const double x = std::clamp(from.x().value() + dx, -55.0, 55.0);
    const double y = std::clamp(from.y().value() + dy, -55.0, 55.0);
    return Pose2d{Length{x}, Length{y}, Angle::radians(rng.uniform(-Angle::kPi, Angle::kPi))};
}

/// One scheduler-driven routine of `n` waypoints on `kin` (C1's cadence: every
/// 3rd waypoint followed by a pure TurnTo, every 7th an explicit StrafeTo).
RoutineResult runRoutine(const shulib::kinematics::IKinematics& kin, int n, std::uint64_t seed,
                         shulib::sim::DegradationModel* hostile) {
    auto pcfg = plantConfig();
    pcfg.plant.seed = seed;
    SchedulerRig sr{kin, pcfg, nullptr, hostile};

    Rng wp{seed * 2654435761ULL};
    RoutineResult out;
    Pose2d target{};
    for (int k = 0; k < n; ++k) {
        target = nextWaypoint(wp, target);
        const bool strafeLeg = (k % 7 == 3);
        ExitReason r = ExitReason::Running;
        if (strafeLeg) {
            StrafeTo m{sr.sched.deps(), target.x(), target.y(), motionConfig(), kMoveTimeout};
            r = sr.run(m);
            target = Pose2d{target.x(), target.y(), sr.rig.h.truePose().heading() /*≈held*/};
        } else {
            MoveToPose m{sr.sched.deps(), target, motionConfig(), kMoveTimeout};
            r = sr.run(m);
        }
        REQUIRE(r == ExitReason::Settled);
        ++out.moveCount;

        WaypointResult w;
        w.t = sr.rig.h.clock().now().value();
        w.distTravelled = sr.pacer.distance;
        w.errTrue = posErr(sr.rig.h.truePose(), target);
        w.errHead = strafeLeg ? 0.0 : headErr(sr.rig.h.truePose(), target);
        w.dx = sr.rig.h.truePose().x().value() - target.x().value();
        w.dy = sr.rig.h.truePose().y().value() - target.y().value();
        out.waypoints.push_back(w);
        out.worstArrival = std::max(out.worstArrival, w.errTrue);
        out.worstHeadArrival = std::max(out.worstHeadArrival, w.errHead);

        if (k % 3 == 2) {
            const Angle newHeading = Angle::radians(wp.uniform(-Angle::kPi, Angle::kPi));
            TurnTo t{sr.sched.deps(), newHeading, motionConfig(), kMoveTimeout};
            REQUIRE(sr.run(t) == ExitReason::Settled);
            target = Pose2d{target.x(), target.y(), newHeading};
            ++out.moveCount;
        }
    }
    out.finalErr = posErr(sr.rig.h.truePose(), target);
    out.finalHeadErr = headErr(sr.rig.h.truePose(), target);
    out.totalTime = sr.rig.h.clock().now().value();
    out.totalDistance = sr.pacer.distance;

    // Engine bookkeeping must agree with physics on BOTH drivetrains:
    REQUIRE(sr.sched.motionsStarted() == out.moveCount);
    REQUIRE(sr.sched.motionsSettled() == out.moveCount);
    REQUIRE(sr.sched.motionsCancelled() == 0);
    REQUIRE(sr.sched.motionsAborted() == 0);
    return out;
}

double slope(const std::vector<double>& x, const std::vector<double>& y) {
    const auto n = static_cast<double>(x.size());
    double sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0;
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

// ═══ CLEAN sweep, X and H side by side: the count-compounding pin ══════════════════
// Bug caught: per-move error compounding that only appears when the authority
// clamp participates in every lateral leg — the structural failure mode C3 was
// ordered before F6 to find. Both drivetrains must stay tolerance-class at every
// length; the H-bot's numbers print NEXT TO the X-bot's (the difference is
// information — expected: same accuracy, more seconds).
TEST_CASE("C3 routine: CLEAN plant — error FLAT in move count on BOTH drivetrains "
          "(5/10/20/40, same auton, same seed)") {
    const auto xKin = xDrive(Length{7.0});
    const auto hKin = hBotKinematics();
    for (const int n : {5, 10, 20, 40}) {
        CAPTURE(n);
        const RoutineResult x = runRoutine(xKin, n, 77, nullptr);
        const RoutineResult h = runRoutine(hKin, n, 77, nullptr);
        MESSAGE("clean n=", n, "  X: finalErr=", x.finalErr, "in worst=", x.worstArrival,
                "in t=", x.totalTime, "s d=", x.totalDistance, "in  |  H: finalErr=",
                h.finalErr, "in worst=", h.worstArrival, "in t=", h.totalTime,
                "s d=", h.totalDistance, "in");
        // Identical structural bar for both (C1's clean bounds):
        for (const RoutineResult* r : {&x, &h}) {
            CHECK(r->finalErr < 0.8);
            CHECK(r->worstArrival < 0.8);
            CHECK(r->finalHeadErr < 0.03);
        }
        // The H-bot pays TIME, never ACCURACY, for its limited authority:
        CHECK(h.totalTime >= x.totalTime - 1.0);
    }
}

// ═══ HOSTILE sweep on the H-bot: bounded error, attributed by regression ═══════════
// Bug caught: unbounded/blowing error under the composed hostile world on the
// second drivetrain, or error that grows in COUNT rather than TIME (compounding
// hiding behind drift). X runs the same seeds alongside so the side-by-side is
// measured in one place, not quoted across documents.
TEST_CASE("C3 routine: HOSTILE sweep — H-bot bounded across 5/10/20/40 moves, slopes "
          "reported, X side by side") {
    const auto xKin = xDrive(Length{7.0});
    const auto hKin = hBotKinematics();
    std::vector<double> vCount, vTime, vDist, vErr;
    double worstFinalH = 0.0, worstArrivalH = 0.0, worstHeadH = 0.0;
    double worstFinalX = 0.0, worstArrivalX = 0.0;
    double totalTimeH = 0.0, totalTimeX = 0.0;
    for (const int n : {5, 10, 20, 40}) {
        for (const std::uint64_t seed : {11ULL, 22ULL}) {
            CAPTURE(n);
            CAPTURE(seed);
            FullHostility worldX{};
            const RoutineResult x = runRoutine(xKin, n, seed, &worldX.model());
            FullHostility worldH{};
            const RoutineResult h = runRoutine(hKin, n, seed, &worldH.model());
            MESSAGE("hostile n=", n, " seed=", seed, "  X: final=", x.finalErr, "in worst=",
                    x.worstArrival, "in t=", x.totalTime, "s  |  H: final=", h.finalErr,
                    "in worst=", h.worstArrival, "in head=",
                    h.worstHeadArrival * 180.0 / Angle::kPi, "deg t=", h.totalTime, "s");
            worstFinalX = std::max(worstFinalX, x.finalErr);
            worstArrivalX = std::max(worstArrivalX, x.worstArrival);
            worstFinalH = std::max(worstFinalH, h.finalErr);
            worstArrivalH = std::max(worstArrivalH, h.worstArrival);
            worstHeadH = std::max(worstHeadH, std::max(h.finalHeadErr, h.worstHeadArrival));
            totalTimeX += x.totalTime;
            totalTimeH += h.totalTime;
            int idx = 0;
            for (const WaypointResult& w : h.waypoints) {
                vCount.push_back(static_cast<double>(++idx));
                vTime.push_back(w.t);
                vDist.push_back(w.distTravelled);
                vErr.push_back(w.errTrue);
            }
        }
    }
    // The H-bot's bound, DERIVED as C1 derived X's (not wished): the dominant M2
    // term is IMU drift rotating the odometry frame — err ≲ Σ|leg|·headErr(t).
    // MEASURED: the H-bot covers the same ~900 in worst path in only ~1.3% more
    // total time than X (351 vs 347 s) — the turn-while-drive migration keeps
    // MoveToPose legs near X speed; only the explicit StrafeTo legs (1 in 7) pay
    // the crab — so H's drift ceiling is essentially X's ~12 in at the HA-20
    // worst bias. Observed worst across this sweep: 4.03 in (X alongside: 4.13).
    // 6.0 in = observed + ~50% margin, 2× inside the physics ceiling. Phase E's
    // correctors shrink both.
    CHECK(worstArrivalH < 6.0);
    CHECK(worstFinalH < 6.0);
    CHECK(worstHeadH < 0.06);
    MESSAGE("HOSTILE worst — X: arrival=", worstArrivalX, "in final=", worstFinalX,
            "in (Σt=", totalTimeX, "s) | H: arrival=", worstArrivalH, "in final=",
            worstFinalH, "in (Σt=", totalTimeH, "s)");
    MESSAGE("H slopes: err-vs-moveIndex=", slope(vCount, vErr), " in/move | err-vs-time=",
            slope(vTime, vErr), " in/s | err-vs-distance=", slope(vDist, vErr), " in/in");
}

// ═══ The count-vs-time discriminator, on the drive where it matters MORE ═══════════
// Same ground, twice the moves. The H-bot's settle tails are longer than X's
// (authority-limited final approach on lateral legs), so if per-move compounding
// existed anywhere in the clamp path, family B would pay it twice as often.
// It must pay only TIME (drift), never per-move error.
TEST_CASE("C3 routine: H-bot — halving leg length while doubling move count does not "
          "compound error") {
    const auto kin = hBotKinematics();
    auto runFamily = [&](int n, double leg, std::uint64_t seed) {
        auto pcfg = plantConfig();
        pcfg.plant.seed = seed;
        FullHostility world{};
        SchedulerRig sr{kin, pcfg, nullptr, &world.model()};
        Rng dir{seed};
        Pose2d target{};
        double x = 0.0, y = 0.0;
        for (int k = 0; k < n; ++k) {
            const double ang = dir.uniform(-Angle::kPi, Angle::kPi);
            x = std::clamp(x + leg * std::cos(ang), -50.0, 50.0);
            y = std::clamp(y + leg * std::sin(ang), -50.0, 50.0);
            target = Pose2d{Length{x}, Length{y}, Angle::radians(dir.uniform(-1.0, 1.0))};
            MoveToPose m{sr.sched.deps(), target, motionConfig(), kMoveTimeout};
            REQUIRE(sr.run(m) == ExitReason::Settled);
        }
        return std::pair{posErr(sr.rig.h.truePose(), target), sr.rig.h.clock().now().value()};
    };
    double errA = 0.0, errB = 0.0, tA = 0.0, tB = 0.0;
    for (const std::uint64_t seed : {5ULL, 6ULL, 7ULL}) {
        const auto [ea, ta] = runFamily(8, 30.0, seed);
        const auto [eb, tb] = runFamily(16, 15.0, seed);
        errA += ea / 3.0;
        errB += eb / 3.0;
        tA += ta / 3.0;
        tB += tb / 3.0;
    }
    MESSAGE("H family A (8x30in): meanErr=", errA, "in meanTime=", tA,
            "s | family B (16x15in): meanErr=", errB, "in meanTime=", tB, "s");
    CHECK(errB < errA + 2.0);  // B pays its extra drift seconds, nothing per-move
    CHECK(errB < 4.5);
}
