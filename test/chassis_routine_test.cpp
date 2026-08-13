// C4 ROUTINES + GUARANTEES THROUGH THE FACADE — the DoD items "a complete
// hand-written auton on all three drivetrains", "routine accuracy through the
// facade matches prior baselines, flat in move count", and "every lower-layer
// guarantee verified THROUGH the facade". A facade that quietly loses a
// guarantee is worse than no facade, so each one is re-pinned here through the
// public verbs, not below them.
//
// The keystone is the BIT-IDENTITY twin: the same routine (same generator,
// seeds, cadence as C2/C3's suites) run once through facade verbs and once
// through a hand-built scheduler must agree to the bit, clean AND hostile —
// the strongest available statement that the facade added API, not physics.
// With that pinned, C1/C2/C3's measured baselines carry over VERBATIM.

#include "doctest.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "motion_test_rig.hpp"
#include "shulib/chassis/chassis.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/kinematics/x_drive.hpp"
#include "shulib/motion/move_to_pose.hpp"
#include "shulib/motion/strafe_to.hpp"
#include "shulib/motion/turn_to.hpp"
#include "shulib/sim/hostile/composed.hpp"
#include "shulib/sim/hostile/encoder_hostility.hpp"
#include "shulib/sim/hostile/imu_hostility.hpp"
#include "shulib/sim/rng.hpp"

using namespace motion_rig;
using shulib::chassis::TrajectoryResult;
using shulib::control::ExitReason;
using shulib::diag::FaultCode;
using shulib::hal::BrakeMode;
using shulib::hal::fake::FakeTelemetrySink;
using shulib::kinematics::TankKinematics;
using shulib::kinematics::xDrive;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::motion::MotionState;
using shulib::motion::MoveToPose;
using shulib::motion::StrafeTo;
using shulib::motion::TurnTo;
using shulib::motion::WaitResult;
using shulib::sim::EncoderHostileConfig;
using shulib::sim::EncoderHostileModel;
using shulib::sim::FullHostility;
using shulib::sim::ImuHostileConfig;
using shulib::sim::ImuHostileModel;
using shulib::sim::Rng;
using shulib::units::AngularVelocity;
using shulib::units::Length;
using shulib::units::Time;
using shulib::units::Velocity;

namespace {

constexpr double kMoveTimeoutX = 8.0;   // C2's X budget
constexpr double kMoveTimeoutH = 14.0;  // C3's shared X/H budget (strafe-limited legs)

struct WaypointResult {
    double t = 0.0;
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

/// C1's waypoint generator, verbatim — same draws for the same seed, so the
/// facade runs LITERALLY the same auton C1/C2/C3 measured.
Pose2d nextWaypoint(Rng& rng, const Pose2d& from) {
    const double dx = rng.uniform(-35.0, 35.0);
    const double dy = rng.uniform(-35.0, 35.0);
    const double x = std::clamp(from.x().value() + dx, -55.0, 55.0);
    const double y = std::clamp(from.y().value() + dy, -55.0, 55.0);
    return Pose2d{Length{x}, Length{y}, Angle::radians(rng.uniform(-Angle::kPi, Angle::kPi))};
}

/// C2's cadence (every 3rd waypoint a pure turn, every 7th a strafe), driven
/// EITHER through facade verbs OR through a hand-built SchedulerRig — the two
/// arms of the bit-identity twin. Identical rig config either way.
RoutineResult runRoutine(const shulib::kinematics::IKinematics& kin, int n,
                         std::uint64_t seed, bool viaFacade, bool hostile,
                         double moveTimeout) {
    auto pcfg = plantConfig();
    pcfg.plant.seed = seed;
    FullHostility world{};
    shulib::sim::DegradationModel* model = hostile ? &world.model() : nullptr;

    // Exactly one of the two rigs is used; both wire the identical stack.
    ChassisRig cr{kin, pcfg, nullptr, viaFacade ? model : nullptr};
    SchedulerRig sr{kin, pcfg, nullptr, viaFacade ? nullptr : model};

    auto truePose = [&] { return viaFacade ? cr.rig.h.truePose() : sr.rig.h.truePose(); };
    auto now = [&] {
        return viaFacade ? cr.rig.h.clock().now().value() : sr.rig.h.clock().now().value();
    };

    Rng wp{seed * 2654435761ULL};
    RoutineResult out;
    Pose2d target{};
    for (int k = 0; k < n; ++k) {
        target = nextWaypoint(wp, target);
        const bool strafeLeg = (k % 7 == 3);
        ExitReason r = ExitReason::Running;
        if (strafeLeg) {
            if (viaFacade) {
                r = cr.chassis.strafeTo(target.x(), target.y(),
                                        {.timeout = Time{moveTimeout}});
            } else {
                StrafeTo m{sr.sched.deps(), target.x(), target.y(), motionConfig(),
                           moveTimeout};
                r = sr.run(m);
            }
            target = Pose2d{target.x(), target.y(), truePose().heading()};
        } else {
            if (viaFacade) {
                r = cr.chassis.moveTo(target, {.timeout = Time{moveTimeout}});
            } else {
                MoveToPose m{sr.sched.deps(), target, motionConfig(), moveTimeout};
                r = sr.run(m);
            }
        }
        REQUIRE(r == ExitReason::Settled);
        ++out.moveCount;

        WaypointResult w;
        w.t = now();
        w.errTrue = posErr(truePose(), target);
        w.errHead = strafeLeg ? 0.0 : headErr(truePose(), target);
        out.waypoints.push_back(w);
        out.worstArrival = std::max(out.worstArrival, w.errTrue);
        out.worstHeadArrival = std::max(out.worstHeadArrival, w.errHead);

        if (k % 3 == 2) {
            const Angle newHeading = Angle::radians(wp.uniform(-Angle::kPi, Angle::kPi));
            if (viaFacade) {
                REQUIRE(cr.chassis.turnTo(newHeading, {.timeout = Time{moveTimeout}})
                        == ExitReason::Settled);
            } else {
                TurnTo t{sr.sched.deps(), newHeading, motionConfig(), moveTimeout};
                REQUIRE(sr.run(t) == ExitReason::Settled);
            }
            target = Pose2d{target.x(), target.y(), newHeading};
            ++out.moveCount;
        }
    }
    out.finalErr = posErr(truePose(), target);
    out.finalHeadErr = headErr(truePose(), target);
    out.totalTime = now();
    out.totalDistance = viaFacade ? cr.pacer.distance : sr.pacer.distance;

    // Engine bookkeeping must agree with physics through BOTH paths:
    const auto& sched = viaFacade ? cr.chassis.scheduler() : sr.sched;
    REQUIRE(sched.motionsStarted() == out.moveCount);
    REQUIRE(sched.motionsSettled() == out.moveCount);
    REQUIRE(sched.motionsCancelled() == 0);
    REQUIRE(sched.motionsAborted() == 0);
    return out;
}

}  // namespace

// ═══ The bit-identity twin (the keystone) ══════════════════════════════════════════

// Bug caught: ANY deviation of the facade's verb path from the scheduler path
// the C2/C3 baselines were measured on — an extra localizer update, a config
// reshaped in effectiveConfig, a dropped pace, a different construction order.
// Bit-equality means C1/C2/C3's accuracy numbers carry over VERBATIM, and the
// facade demonstrably added API, not physics.
TEST_CASE("C4 twin: facade verbs == hand-built scheduler, bit for bit (clean and "
          "hostile)") {
    const auto kin = xDrive(Length{7.0});
    for (const bool hostile : {false, true}) {
        CAPTURE(hostile);
        const RoutineResult facade = runRoutine(kin, 5, 77, true, hostile, kMoveTimeoutX);
        const RoutineResult sched = runRoutine(kin, 5, 77, false, hostile, kMoveTimeoutX);
        REQUIRE(facade.moveCount == sched.moveCount);
        REQUIRE(facade.waypoints.size() == sched.waypoints.size());
        CHECK(facade.totalTime == sched.totalTime);
        CHECK(facade.totalDistance == sched.totalDistance);
        CHECK(facade.finalErr == sched.finalErr);
        CHECK(facade.finalHeadErr == sched.finalHeadErr);
        for (std::size_t i = 0; i < facade.waypoints.size(); ++i) {
            CHECK(facade.waypoints[i].t == sched.waypoints[i].t);
            CHECK(facade.waypoints[i].errTrue == sched.waypoints[i].errTrue);
            CHECK(facade.waypoints[i].errHead == sched.waypoints[i].errHead);
        }
        MESSAGE("twin (hostile=", hostile, "): finalErr=", facade.finalErr,
                "in time=", facade.totalTime, "s over ", facade.moveCount, " motions");
    }
}

// ═══ Accuracy through the facade: flat in move count, X and H ══════════════════════

// Bug caught: facade-added per-move error compounding (state leaking between
// verb calls — a stale PID, a re-used motion, a drifting effectiveConfig).
// With clean sensors there is no drift clock, so ANY growth with routine
// length through the facade is facade-added. Bounds are C1/C3's, unchanged.
TEST_CASE("C4 routine: CLEAN sweep 5/10/20/40 through facade verbs — error FLAT in "
          "move count on X AND H") {
    const auto xKin = xDrive(Length{7.0});
    const auto hKin = hBotKinematics();
    for (const int n : {5, 10, 20, 40}) {
        CAPTURE(n);
        const RoutineResult x = runRoutine(xKin, n, 77, true, false, kMoveTimeoutX);
        const RoutineResult h = runRoutine(hKin, n, 77, true, false, kMoveTimeoutH);
        MESSAGE("facade clean n=", n, "  X: finalErr=", x.finalErr, "in worst=",
                x.worstArrival, "in t=", x.totalTime, "s  |  H: finalErr=", h.finalErr,
                "in worst=", h.worstArrival, "in t=", h.totalTime, "s");
        for (const RoutineResult* r : {&x, &h}) {
            CHECK(r->finalErr < 0.8);
            CHECK(r->worstArrival < 0.8);
            CHECK(r->finalHeadErr < 0.03);
        }
    }
}

// Bug caught: the facade breaking the hostile-world bounds C2/C3 measured —
// error must stay bounded and attributable to drift (time), never diverge.
// Bounds are C2's (X: 5.0 in) and C3's (H: 6.0 in), unchanged.
TEST_CASE("C4 routine: HOSTILE sweep 5/10/20/40 through facade verbs — bounded on X "
          "AND H, no policy aborts") {
    const auto xKin = xDrive(Length{7.0});
    const auto hKin = hBotKinematics();
    double worstX = 0.0;
    double worstH = 0.0;
    double worstHeadX = 0.0;
    double worstHeadH = 0.0;
    for (const int n : {5, 10, 20, 40}) {
        CAPTURE(n);
        const RoutineResult x = runRoutine(xKin, n, 11, true, true, kMoveTimeoutX);
        const RoutineResult h = runRoutine(hKin, n, 11, true, true, kMoveTimeoutH);
        MESSAGE("facade hostile n=", n, "  X: final=", x.finalErr, "in worst=",
                x.worstArrival, "in  |  H: final=", h.finalErr, "in worst=",
                h.worstArrival, "in");
        worstX = std::max(worstX, std::max(x.finalErr, x.worstArrival));
        worstH = std::max(worstH, std::max(h.finalErr, h.worstArrival));
        worstHeadX = std::max(worstHeadX, std::max(x.finalHeadErr, x.worstHeadArrival));
        worstHeadH = std::max(worstHeadH, std::max(h.finalHeadErr, h.worstHeadArrival));
    }
    CHECK(worstX < 5.0);      // C2's X bound, held through the facade
    CHECK(worstH < 6.0);      // C3's H bound, held through the facade
    CHECK(worstHeadX < 0.06);
    CHECK(worstHeadH < 0.06);
    MESSAGE("facade HOSTILE worst — X: ", worstX, "in  H: ", worstH, "in");
}

// Bug caught: the facade being unusable on the drivetrain with NO strafe
// authority — a tank auton is turnTo-to-bearing + moveTo (the AUTHOR plans
// the turn; the library never sequences one silently, D12). Error must stay
// tolerance-class at every length. NEW baseline: C1-C3 swept tank along-axis
// trials but never a full multi-waypoint tank routine.
TEST_CASE("C4 routine: TANK — turn-then-drive auton, error flat in move count "
          "(5/10/20/40; new baseline)") {
    const TankKinematics kin{Length{12.0}};
    for (const int n : {5, 10, 20, 40}) {
        CAPTURE(n);
        auto pcfg = plantConfig();
        pcfg.plant.seed = 77;
        ChassisRig c{kin, pcfg};
        Rng wp{77ULL * 2654435761ULL};
        Pose2d target{};
        double worst = 0.0;
        for (int k = 0; k < n; ++k) {
            const Pose2d next = nextWaypoint(wp, target);
            // The AUTHOR's plan: face the waypoint from where we actually are…
            const Pose2d here = c.chassis.pose();
            const Angle bearing = Angle::radians(
                std::atan2((next.y() - here.y()).value(), (next.x() - here.x()).value()));
            REQUIRE(c.chassis.turnTo(bearing, {.timeout = Time{kMoveTimeoutX}})
                    == ExitReason::Settled);
            // …then drive the line, holding that bearing.
            target = Pose2d{next.x(), next.y(), bearing};
            REQUIRE(c.chassis.moveTo(target, {.timeout = Time{kMoveTimeoutH}})
                    == ExitReason::Settled);
            worst = std::max(worst, posErr(c.rig.h.truePose(), target));
        }
        MESSAGE("tank clean n=", n, " finalErr=", posErr(c.rig.h.truePose(), target),
                "in worstArrival=", worst, "in t=", c.rig.h.clock().now().value(), "s");
        CHECK(posErr(c.rig.h.truePose(), target) < 1.0);  // tank baseline: tolerance +
        CHECK(worst < 1.0);                               // uncorrectable-lateral margin
        CHECK(c.chassis.scheduler().motionsAborted() == 0);
        CHECK_FALSE(c.rig.latch.hasFault());
    }
}

// ═══ The complete hand-written auton, all three drivetrains ════════════════════════

// Bug caught: any verb combination a real auton uses failing to compose —
// this is the DoD's end-to-end case: seed pose → move → turn → strafe (where
// physical) → trajectory → deliberate wait → hold → park, graded on truth,
// bookkeeping coherent, zero faults. The facade IS the auton API; this test
// is a facsimile of the thing students will actually write.
TEST_CASE("C4 auton: a complete hand-written routine through the facade on X, H, and "
          "tank") {
    struct Drivetrain {
        const char* name;
        const shulib::kinematics::IKinematics* kin;
        bool canStrafe;
        double timeout;
    };
    const auto xKin = xDrive(Length{7.0});
    const auto hKin = hBotKinematics();
    const TankKinematics tKin{Length{12.0}};
    const Drivetrain drivetrains[] = {{"X", &xKin, true, kMoveTimeoutX},
                                      {"H", &hKin, true, kMoveTimeoutH},
                                      {"tank", &tKin, false, kMoveTimeoutH}};
    for (const Drivetrain& d : drivetrains) {
        CAPTURE(std::string{d.name});
        const Pose2d start{Length{-40.0}, Length{-40.0}, Angle{}};
        auto pcfg = plantConfig();
        pcfg.plant.initialPose = start;
        ChassisRig c{*d.kin, pcfg};
        auto& ch = c.chassis;
        ch.setPose(start);  // the auton's first line: seed the measured start

        const shulib::chassis::MotionOptions o{.timeout = Time{d.timeout}};
        int expectedMotions = 0;
        // Leg 1: drive out. The tank AUTHOR plans a bearing turn first (the
        // library never sequences one silently — D12's honesty, the author's
        // choice): its leg-1 target lies off the start heading's line.
        Pose2d leg1{Length{-10.0}, Length{-30.0}, Angle{}};
        if (!d.canStrafe) {
            const Angle bearing = Angle::radians(
                std::atan2((leg1.y() - start.y()).value(), (leg1.x() - start.x()).value()));
            REQUIRE(ch.turnTo(bearing, o) == ExitReason::Settled);
            leg1 = Pose2d{leg1.x(), leg1.y(), bearing};
            ++expectedMotions;
        }
        REQUIRE(ch.moveTo(leg1, o) == ExitReason::Settled);
        // Leg 2: face the work.
        REQUIRE(ch.turnTo(Angle::degrees(90.0), o) == ExitReason::Settled);
        // Leg 3: lateral reposition — strafe where the drivetrain can, a
        // planned turn-and-drive where it can't (tank honesty, author-planned).
        expectedMotions += 2;
        if (d.canStrafe) {
            REQUIRE(ch.strafeTo(Length{5.0}, Length{-30.0}, o) == ExitReason::Settled);
            ++expectedMotions;
        } else {
            REQUIRE(ch.turnTo(Angle::degrees(0.0), o) == ExitReason::Settled);
            REQUIRE(ch.moveTo(Pose2d{Length{5.0}, Length{-30.0}, Angle{}}, o)
                    == ExitReason::Settled);
            expectedMotions += 2;
        }
        // Leg 4: a two-waypoint trajectory toward the goal. Tank gets
        // along-heading waypoints (the author's plan again).
        Pose2d wpA{Length{5.0}, Length{-10.0}, Angle::degrees(90.0)};
        Pose2d wpB{Length{5.0}, Length{10.0}, Angle::degrees(90.0)};
        if (!d.canStrafe) {
            REQUIRE(ch.turnTo(Angle::degrees(90.0), o) == ExitReason::Settled);
            ++expectedMotions;
        }
        const TrajectoryResult traj = ch.followTrajectory({wpA, wpB}, o);
        REQUIRE(traj.succeeded());
        CHECK(traj.completedLegs == 2);
        expectedMotions += 2;
        // A deliberate strategy wait (e.g. "let the ring drop"): bounded poll.
        CHECK(ch.waitUntil([] { return false; }, Time{0.2}) == WaitResult::TimedOut);
        // Hold the scoring spot briefly against contact, then park.
        REQUIRE(ch.hold(Time{0.3}, o) == ExitReason::Settled);
        REQUIRE(ch.brake(o) == ExitReason::Settled);
        expectedMotions += 2;

        // Graded on ground truth: at the last waypoint, holding its heading.
        CHECK(posErr(c.rig.h.truePose(), wpB) < 1.2);
        CHECK(headErr(c.rig.h.truePose(), wpB) < 0.04);
        // Bookkeeping coherent; a clean auton latches nothing.
        CHECK(ch.scheduler().motionsStarted() == expectedMotions);
        CHECK(ch.scheduler().motionsSettled() == expectedMotions);
        CHECK(ch.scheduler().motionsCancelled() == 0);
        CHECK(ch.scheduler().motionsAborted() == 0);
        CHECK_FALSE(c.rig.latch.hasFault());
        MESSAGE("auton [", std::string{d.name}, "]: ", expectedMotions, " motions in ",
                c.rig.h.clock().now().value(), "s, final err ",
                posErr(c.rig.h.truePose(), wpB), "in");
    }
}

// ═══ followTrajectory: chaining and its honest failure mode ════════════════════════

// Bug caught: legs blended/skipped (the corner proves each waypoint was
// visited), an off-by-one dropping the last waypoint, or bookkeeping that
// disagrees with the legs actually run.
TEST_CASE("C4 trajectory: waypoints chain as settled legs — the corner is really "
          "driven") {
    const auto kin = xDrive(Length{7.0});
    ChassisRig c{kin};
    const Pose2d corner{Length{20.0}, Length{0.0}, Angle{}};
    const Pose2d goal{Length{20.0}, Length{20.0}, Angle{}};
    const TrajectoryResult r =
        c.chassis.followTrajectory({corner, goal}, {.timeout = Time{8.0}});
    REQUIRE(r.succeeded());
    CHECK(r.completedLegs == 2);
    CHECK(r.totalLegs == 2);
    CHECK(posErr(c.rig.h.truePose(), goal) < 1.0);
    CHECK(c.chassis.scheduler().motionsStarted() == 2);
    // Around the corner is ~40 in; the diagonal shortcut only ~28. The true
    // path length proves leg 1 settled AT the corner rather than being cut.
    CHECK(c.pacer.distance > 36.0);
}

// Bug caught: the chain continuing after a failed leg (chasing waypoints
// while lost — compounding blindly), or the result mislabeling where it broke.
TEST_CASE("C4 trajectory: a failing leg STOPS the chain and reports honestly") {
    const auto kin = xDrive(Length{7.0});
    ChassisRig c{kin};
    // Leg 1 settles inside the budget (~1.1 s: exponential PID tail + hold);
    // leg 2 (~63 in: cap-limited cruise + the same tail ≈ 2.0 s) cannot;
    // leg 3 must never start.
    const TrajectoryResult r = c.chassis.followTrajectory(
        {Pose2d{Length{6.0}, Length{0.0}, Angle{}},
         Pose2d{Length{55.0}, Length{40.0}, Angle{}},
         Pose2d{Length{0.0}, Length{40.0}, Angle{}}},
        {.timeout = Time{1.5}});
    CHECK_FALSE(r.succeeded());
    CHECK(r.exit == ExitReason::TimedOut);
    CHECK(r.completedLegs == 1);
    CHECK(r.totalLegs == 3);
    CHECK(c.chassis.scheduler().motionsStarted() == 2);  // leg 3 never attempted
    CHECK(c.chassis.scheduler().motionsTimedOut() == 1);
    // The timeout stop left the motors stopped (C1's exit behaviour, held):
    for (int w = 0; w < c.rig.h.motorCount(); ++w) {
        CHECK(c.rig.h.motor(w).commandedVoltage().value() == 0.0);
    }
}

// Bug caught: the C3 turn-while-drive visibility contract lost behind the
// facade — a lateral-dominant H-drive trajectory leg must flag SFB on the
// id-stamped record stream (never silent), while still arriving.
TEST_CASE("C4 trajectory: H-drive lateral leg runs authority-limited and VISIBLY so "
          "through the facade") {
    FakeTelemetrySink sink;
    const auto kin = hBotKinematics();
    ChassisRig c{kin, plantConfig(), &sink};
    // Pure +Y leg with heading held at 0: body vy dominates → the clamp binds.
    const TrajectoryResult r = c.chassis.followTrajectory(
        {Pose2d{Length{0.0}, Length{24.0}, Angle{}}}, {.timeout = Time{kMoveTimeoutH}});
    REQUIRE(r.succeeded());
    int sfbStamped = 0;
    for (int i = 0; i < sink.recordCount(); ++i) {
        const auto& rec = sink.recordAt(i);
        if (rec.strafeFallbackActive) {
            CHECK(rec.activeCommandId == 1);  // attributed to the leg, structurally
            ++sfbStamped;
        }
    }
    CHECK(sfbStamped > 20);  // the fallback engaged, sustained and visible
}

// ═══ Guarantees through the facade ═════════════════════════════════════════════════

// Bug caught: the ODO_STUCK abort policy lost behind the facade — the ONE
// fault that must stop a motion (the estimate is lying) has to surface as a
// Cancelled verb return with the causal fault named, promptly, drive safed.
TEST_CASE("C4 guarantee: ODO_STUCK aborts a facade moveTo promptly into the safe "
          "state, cause named") {
    EncoderHostileConfig enc;
    enc.trackingFreezeAt = Time{1.0};
    enc.trackingFreezeIndex = -1;  // both tracking channels die mid-motion
    EncoderHostileModel model{enc};
    const auto kin = xDrive(Length{7.0});
    ChassisRig c{kin, plantConfig(), nullptr, &model};

    const ExitReason r = c.chassis.moveTo(Pose2d{Length{40.0}, Length{0.0}, Angle{}},
                                          {.timeout = Time{6.0}});
    CHECK(r == ExitReason::Cancelled);
    CHECK(c.chassis.lastCompleted().abortFault == FaultCode::OdoStuck);
    CHECK(c.chassis.scheduler().motionsAborted() == 1);
    CHECK(c.rig.h.clock().now().value() < 2.0);  // aborted promptly, NOT the watchdog
    for (int w = 0; w < c.rig.h.motorCount(); ++w) {
        CHECK(c.rig.h.motor(w).commandedVoltage().value() == 0.0);
        CHECK(c.rig.h.motor(w).brakeMode() == BrakeMode::Brake);
    }
    // Truth-vs-estimate damage stayed bounded (the abort did its job):
    CHECK(posErr(c.rig.h.truePose(), c.rig.loc.pose()) < 8.0);
}

// Bug caught: an over-aggressive facade policy aborting on survivable faults
// — C1 pinned that a mid-run IMU loss keeps driving (Degraded does not gate).
TEST_CASE("C4 guarantee: IMU_LOST mid-verb does NOT abort — the verb still settles") {
    ImuHostileConfig cfg;
    cfg.calibrationEnd = Time{0.0};  // live from the start; isolate the dropout
    cfg.dropoutAt = Time{1.0};
    ImuHostileModel imu{cfg};
    const auto kin = xDrive(Length{7.0});
    ChassisRig c{kin, plantConfig(), nullptr, &imu};
    REQUIRE(c.chassis.moveTo(Pose2d{Length{30.0}, Length{8.0}, Angle::degrees(30.0)},
                             {.timeout = Time{8.0}})
            == ExitReason::Settled);
    CHECK(c.rig.latch.raiseCount(FaultCode::ImuLost) == 1);  // seen, once — not fatal
    CHECK(c.chassis.scheduler().motionsAborted() == 0);
}

// Bug caught: the wait-for-live contract lost behind the facade — a verb
// issued during IMU calibration must WAIT motionless (bounded), then land;
// with a never-live estimate it must exit TimedOut instead of hanging or
// driving blind.
TEST_CASE("C4 guarantee: boot window through the facade — waits motionless then "
          "lands; never-live times out having never moved") {
    const auto kin = xDrive(Length{7.0});

    // Normal boot: ~2 s calibration, then the verb completes.
    {
        ImuHostileModel imu{ImuHostileConfig{}};
        ChassisRig c{kin, plantConfig(), nullptr, &imu};
        const Pose2d target{Length{24.0}, Length{0.0}, Angle{}};
        REQUIRE(c.chassis.moveTo(target, {.timeout = Time{8.0}}) == ExitReason::Settled);
        CHECK(c.rig.h.clock().now().value() > 1.9);  // it genuinely waited out boot
        CHECK(posErr(c.rig.h.truePose(), target) < 1.5);
        CHECK_FALSE(c.rig.latch.hasFault());  // boot is normal, not a fault
    }
    // Never-live: bounded TimedOut, zero movement, fault raised.
    {
        ImuHostileConfig cfg;
        cfg.calibrationEnd = Time{1e18};  // the IMU never comes up
        ImuHostileModel imu{cfg};
        ChassisRig c{kin, plantConfig(), nullptr, &imu};
        const double t0 = c.rig.h.clock().now().value();
        CHECK(c.chassis.moveTo(Pose2d{Length{24.0}, Length{0.0}, Angle{}},
                               {.timeout = Time{1.5}})
              == ExitReason::TimedOut);
        CHECK(c.rig.h.clock().now().value() - t0 >= 1.5);
        CHECK(c.rig.h.clock().now().value() - t0 < 2.0);   // bounded, not hung
        CHECK(posErr(c.rig.h.truePose(), Pose2d{}) < 1e-6);  // never moved
        CHECK(c.rig.latch.raiseCount(FaultCode::MotionTimeout) == 1);
    }
}

// Bug caught: facade cancel() failing to reach the active motion (a verb
// started through the Tier-3 seam must be cancellable through the facade —
// one slot, one cancel), or the safe state not landing synchronously.
TEST_CASE("C4 guarantee: cancel() mid-motion through the facade — safe state NOW, "
          "boundary recorded, rest reached") {
    const auto kin = xDrive(Length{7.0});
    ChassisRig c{kin};
    MoveToPose m{c.chassis.deps(), Pose2d{Length{40.0}, Length{0.0}, Angle{}},
                 motionConfig(), 8.0};
    c.chassis.scheduler().async(m);
    for (int i = 0; i < 40; ++i) {  // genuinely at speed
        (void)c.chassis.scheduler().tick();
        c.pacer.pace();
    }
    REQUIRE(c.chassis.scheduler().hasActiveMotion());
    c.chassis.cancel();
    // Safe state applied BY THE CALL, before any further tick:
    for (int w = 0; w < c.rig.h.motorCount(); ++w) {
        CHECK(c.rig.h.motor(w).commandedVoltage().value() == 0.0);
        CHECK(c.rig.h.motor(w).brakeMode() == BrakeMode::Brake);
    }
    CHECK(m.state() == MotionState::Cancelled);
    CHECK(c.chassis.lastExitReason() == ExitReason::Cancelled);
    CHECK(c.chassis.lastCompleted().abortFault == FaultCode::None);  // commanded, not fault
    // And the world confirms: coasting ends, the robot reaches rest.
    Pose2d prev = c.rig.h.truePose();
    double lastStep = 1e9;
    for (int i = 0; i < 50; ++i) {
        c.pacer.pace();
        const Pose2d nowP = c.rig.h.truePose();
        lastStep = posErr(nowP, prev);
        prev = nowP;
    }
    CHECK(lastStep < 0.02);  // at rest (per-tick truth displacement ~zero)
}

// Bug caught: strafeAuthority() passthrough lying about the held kinematics
// (C3 §11 #2 adopted it as a READ-ONLY passthrough, exactly the F5 value).
TEST_CASE("C4 passthrough: strafeAuthority reports the drivetrain's F5 value") {
    const auto xKin = xDrive(Length{7.0});
    const auto hKin = hBotKinematics();
    const TankKinematics tKin{Length{12.0}};
    ChassisRig x{xKin};
    ChassisRig h{hKin};
    ChassisRig t{tKin};
    CHECK(x.chassis.strafeAuthority() == 1.0);
    CHECK(h.chassis.strafeAuthority() == doctest::Approx(0.35));
    CHECK(t.chassis.strafeAuthority() == 0.0);
}
