// C2 FULL-ROUTINE ACCURACY THROUGH THE SCHEDULER — the DoD item "a full
// routine through the scheduler reproduces C1's accuracy baseline", proven the
// strongest way available: the SAME waypoint generator, seeds and cadence as
// C1's hand-chained suite (motion_routine_test.cpp), driven through
// async()/waitUntilSettled(), graded against GROUND TRUTH — plus a per-leg
// BIT-IDENTITY check against the hand-chained twin (the scheduler added
// engine, not physics).
//
// C1's numbers this must reproduce: clean routines FLAT in move count
// (0.00–0.24 in across 5/10/20/40 moves); hostile worst ≈ 4.1 in over ~95 s
// attributed to time-drift; stop-and-settle overhead ≈ 1.2 s/motion.

#include "doctest.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>
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
using shulib::motion::MotionScheduler;
using shulib::motion::MoveToPose;
using shulib::motion::StrafeTo;
using shulib::motion::TurnTo;
using shulib::sim::FullHostility;
using shulib::sim::Rng;
using shulib::sim::TruthSample;
using shulib::units::Time;

namespace {

constexpr double kMoveTimeout = 8.0;  // covers boot wait + the longest leg (C1's value)

struct WaypointResult {
    double t = 0.0;
    double distTravelled = 0.0;
    double errTrue = 0.0;
    double errHead = 0.0;
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

/// C1's waypoint generator, verbatim (motion_routine_test.cpp): same draws,
/// same clamps — so seed 77 produces the SAME routine C1 hand-chained.
Pose2d nextWaypoint(Rng& rng, const Pose2d& from) {
    const double dx = rng.uniform(-35.0, 35.0);
    const double dy = rng.uniform(-35.0, 35.0);
    const double x = std::clamp(from.x().value() + dx, -55.0, 55.0);
    const double y = std::clamp(from.y().value() + dy, -55.0, 55.0);
    return Pose2d{Length{x}, Length{y}, Angle::radians(rng.uniform(-Angle::kPi, Angle::kPi))};
}

/// One full routine of `n` waypoints (C1's cadence: every 3rd waypoint gets a
/// pure TurnTo leg, every 7th is a StrafeTo), EITHER hand-chained (the C1
/// loop, motions from rig.deps) OR scheduled (async + waitUntilSettled,
/// motions from sched.deps()). Same rig config either way.
RoutineResult runRoutine(int n, std::uint64_t seed, bool viaScheduler, bool hostile) {
    const auto kin = xDrive(Length{7.0});
    auto pcfg = plantConfig();
    pcfg.plant.seed = seed;
    FullHostility world{};
    shulib::sim::DegradationModel* model = hostile ? &world.model() : nullptr;
    SchedulerRig s{kin, pcfg, nullptr, model};

    double handDist = 0.0;  // the hand loop's own distance bookkeeping
    auto runOne = [&](shulib::motion::IMotion& m) -> ExitReason {
        if (viaScheduler) {
            return s.run(m);
        }
        m.start();
        auto reason = ExitReason::Running;
        Pose2d prev = s.rig.h.truePose();
        for (int i = 0; i < 1600 && reason == ExitReason::Running; ++i) {
            s.rig.loc.update();
            reason = m.tick();
            if (reason == ExitReason::Running) {
                s.rig.h.plant().step(Time{0.01});
            }
            const Pose2d now = s.rig.h.truePose();
            handDist += posErr(now, prev);
            prev = now;
        }
        return reason;
    };
    const shulib::motion::MotionDeps& deps = viaScheduler ? s.sched.deps() : s.rig.deps;

    Rng wp{seed * 2654435761ULL};
    RoutineResult out;
    Pose2d target{};
    for (int k = 0; k < n; ++k) {
        target = nextWaypoint(wp, target);
        const bool strafeLeg = (k % 7 == 3);
        ExitReason r = ExitReason::Running;
        if (strafeLeg) {
            StrafeTo m{deps, target.x(), target.y(), motionConfig(), kMoveTimeout};
            r = runOne(m);
            target = Pose2d{target.x(), target.y(), s.rig.h.truePose().heading()};
        } else {
            MoveToPose m{deps, target, motionConfig(), kMoveTimeout};
            r = runOne(m);
        }
        REQUIRE(r == ExitReason::Settled);
        ++out.moveCount;

        WaypointResult w;
        w.t = s.rig.h.clock().now().value();
        w.distTravelled = viaScheduler ? s.pacer.distance : handDist;
        w.errTrue = posErr(s.rig.h.truePose(), target);
        w.errHead = strafeLeg ? 0.0 : headErr(s.rig.h.truePose(), target);
        out.waypoints.push_back(w);
        out.worstArrival = std::max(out.worstArrival, w.errTrue);
        out.worstHeadArrival = std::max(out.worstHeadArrival, w.errHead);

        if (k % 3 == 2) {
            const Angle newHeading = Angle::radians(wp.uniform(-Angle::kPi, Angle::kPi));
            TurnTo t{deps, newHeading, motionConfig(), kMoveTimeout};
            REQUIRE(runOne(t) == ExitReason::Settled);
            target = Pose2d{target.x(), target.y(), newHeading};
            ++out.moveCount;
        }
    }
    out.finalErr = posErr(s.rig.h.truePose(), target);
    out.finalHeadErr = headErr(s.rig.h.truePose(), target);
    out.totalTime = s.rig.h.clock().now().value();
    out.totalDistance = viaScheduler ? s.pacer.distance : handDist;
    if (viaScheduler) {
        // Engine bookkeeping must agree with what physically happened.
        REQUIRE(s.sched.motionsStarted() == out.moveCount);
        REQUIRE(s.sched.motionsSettled() == out.moveCount);
        REQUIRE(s.sched.motionsCancelled() == 0);
        REQUIRE(s.sched.motionsAborted() == 0);
    }
    return out;
}

}  // namespace

// ═══ Bit-identity against the C1 hand-chained twin ═════════════════════════════════

// Bug caught: ANY deviation of the scheduler's formalized loop from the C1
// hand loop the accuracy baseline was measured on — an extra/dropped/reordered
// localizer update or plant step shows up as a bit difference immediately.
// This is what makes C1's §3 numbers carry over VERBATIM, not approximately.
TEST_CASE("C2 routine: scheduled == hand-chained, bit for bit (clean and hostile)") {
    for (const bool hostile : {false, true}) {
        CAPTURE(hostile);
        const RoutineResult hand = runRoutine(5, 77, false, hostile);
        const RoutineResult sched = runRoutine(5, 77, true, hostile);
        REQUIRE(hand.moveCount == sched.moveCount);
        REQUIRE(hand.waypoints.size() == sched.waypoints.size());
        CHECK(hand.totalTime == sched.totalTime);
        CHECK(hand.finalErr == sched.finalErr);
        CHECK(hand.finalHeadErr == sched.finalHeadErr);
        for (std::size_t i = 0; i < hand.waypoints.size(); ++i) {
            CHECK(hand.waypoints[i].t == sched.waypoints[i].t);
            CHECK(hand.waypoints[i].errTrue == sched.waypoints[i].errTrue);
            CHECK(hand.waypoints[i].errHead == sched.waypoints[i].errHead);
        }
        MESSAGE("bit-identity (hostile=", hostile, "): finalErr=", sched.finalErr,
                "in time=", sched.totalTime, "s over ", sched.moveCount, " motions");
    }
}

// ═══ CLEAN sweep through the scheduler: the count-compounding pin ══════════════════

// Bug caught: per-move error compounding INTRODUCED BY THE ENGINE — with
// perfect sensors there is no drift clock, so any growth with routine length
// through the scheduler would be scheduler-added state leakage across
// boundaries. Bounds are C1's (tolerance-class at every length).
TEST_CASE("C2 routine: CLEAN sweep 5/10/20/40 through the scheduler — error FLAT in move count") {
    double sumSettleOverhead = 0.0;
    for (const int n : {5, 10, 20, 40}) {
        CAPTURE(n);
        const RoutineResult r = runRoutine(n, 77, true, false);
        MESSAGE("clean n=", n, " moves=", r.moveCount, " finalErr=", r.finalErr,
                "in worstArrival=", r.worstArrival, "in time=", r.totalTime,
                "s dist=", r.totalDistance, "in");
        CHECK(r.finalErr < 0.8);
        CHECK(r.worstArrival < 0.8);
        CHECK(r.finalHeadErr < 0.03);
        sumSettleOverhead +=
            (r.totalTime - r.totalDistance / 60.0) / static_cast<double>(r.moveCount);
    }
    MESSAGE("stop-and-settle overhead through the scheduler ~", sumSettleOverhead / 4.0,
            " s/motion (v1 is stop-and-settle by design; the Frontier blending item's cost)");
}

// ═══ HOSTILE sweep through the scheduler: bounded, attributed ══════════════════════

// Bug caught: the engine breaking the hostile-world bound C1 measured (worst
// ≈ 4.1 in over ~95 s) — the asserted 5.0 in is C1's derived bound, unchanged;
// growth beyond it through the scheduler is engine-added error.
TEST_CASE("C2 routine: HOSTILE sweep 5/10/20/40 x seeds — bounded end error, no policy aborts") {
    double worstFinal = 0.0;
    double worstArrival = 0.0;
    double worstHead = 0.0;
    for (const int n : {5, 10, 20, 40}) {
        for (const std::uint64_t seed : {11ULL, 22ULL}) {
            CAPTURE(n);
            CAPTURE(seed);
            const RoutineResult r = runRoutine(n, seed, true, true);
            MESSAGE("hostile n=", n, " seed=", seed, " moves=", r.moveCount,
                    " finalErr=", r.finalErr, "in worstArrival=", r.worstArrival,
                    "in headFinal=", r.finalHeadErr * 180.0 / Angle::kPi,
                    "deg time=", r.totalTime, "s dist=", r.totalDistance, "in");
            worstFinal = std::max(worstFinal, r.finalErr);
            worstArrival = std::max(worstArrival, r.worstArrival);
            worstHead = std::max(worstHead, std::max(r.finalHeadErr, r.worstHeadArrival));
        }
    }
    CHECK(worstArrival < 5.0);  // C1's derived bound (physics ceiling ~12 in), held
    CHECK(worstFinal < 5.0);
    CHECK(worstHead < 0.06);
    MESSAGE("HOSTILE worst through the scheduler: arrival=", worstArrival,
            "in final=", worstFinal, "in heading=", worstHead * 180.0 / Angle::kPi, "deg");
}

// ═══ Seeded determinism (the DoD item) ═════════════════════════════════════════════

// Bug caught: any hidden nondeterminism in the engine — wall-clock, shared
// RNG, iteration-order dependence. Same seed must give a BYTE-identical truth
// stream and identical bookkeeping; a different seed must genuinely diverge.
TEST_CASE("C2 routine: same seed -> byte-identical truth stream; different seed diverges") {
    struct SamplingPacer final : shulib::motion::ITickPacer {
        explicit SamplingPacer(shulib::sim::SimHarness& harness) : h{&harness} {}
        void pace() override {
            if (paces >= 200000) {
                throw std::runtime_error("SamplingPacer: cap exceeded");
            }
            ++paces;
            h->plant().step(Time{0.01});
            samples.push_back(h->sample());
        }
        shulib::sim::SimHarness* h;
        int paces = 0;
        std::vector<TruthSample> samples;
    };

    const auto kin = xDrive(Length{7.0});
    auto runSeeded = [&](std::uint64_t seed) {
        auto pcfg = plantConfig();
        pcfg.plant.seed = seed;
        FullHostility world{};
        MotionRig rig{kin, pcfg, nullptr, &world.model()};
        SamplingPacer pacer{rig.h};
        MotionScheduler sched{rig.deps, pacer};
        Rng wp{seed * 2654435761ULL};
        Pose2d target{};
        for (int k = 0; k < 3; ++k) {
            target = nextWaypoint(wp, target);
            MoveToPose m{sched.deps(), target, motionConfig(), kMoveTimeout};
            sched.async(m);
            REQUIRE(sched.waitUntilSettled() == ExitReason::Settled);
        }
        struct Out {
            std::vector<TruthSample> samples;
            double fx, fy, ft;
            int faults;
        };
        return Out{std::move(pacer.samples), rig.loc.pose().x().value(),
                   rig.loc.pose().y().value(), rig.h.clock().now().value(),
                   rig.latch.faultCount()};
    };

    const auto a = runSeeded(33);
    const auto b = runSeeded(33);
    REQUIRE(a.samples.size() == b.samples.size());
    REQUIRE(!a.samples.empty());
    CHECK(std::memcmp(a.samples.data(), b.samples.data(),
                      a.samples.size() * sizeof(TruthSample))
          == 0);
    CHECK(a.fx == b.fx);
    CHECK(a.fy == b.fy);
    CHECK(a.ft == b.ft);
    CHECK(a.faults == b.faults);

    const auto c = runSeeded(34);
    const bool sameSize = (a.samples.size() == c.samples.size());
    const bool sameBytes =
        sameSize
        && std::memcmp(a.samples.data(), c.samples.data(),
                       a.samples.size() * sizeof(TruthSample))
               == 0;
    CHECK_FALSE(sameBytes);  // a different seed is a different physical run
}
