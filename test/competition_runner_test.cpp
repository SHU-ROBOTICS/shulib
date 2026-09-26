// competition::runAuton — one registered auton, from its start pose to its guaranteed end, on the
// host test plant (chunk COMP-a; the COMP brief's rulings C1, C2, C5, C6 and amendments B9, B12,
// B13, B14, B16, B18, and §2 item 6's adversarial list). Every case names the bug it would catch.
//
// Graded on PLANT GROUND TRUTH (the A2 truth integrator, independent of the estimate and of the
// guard) and on counts the test keeps itself. The start-pose cases are built so the re-seat CAN
// FAIL (B13): the plant's IMU disagrees with the declared start by a stated angle, so a runner
// that skipped the re-seat drives the routine rotated by exactly that angle.

#include "doctest.h"

#include <array>
#include <cmath>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "motion_test_rig.hpp"
#include "shulib/chassis/chassis.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/competition/acceptance_run.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/competition/match_runner.hpp"
#include "shulib/config/robot_config.hpp"
#include "shulib/config/route.hpp"
#include "shulib/control/exit_group.hpp"
#include "shulib/core/check.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/hal/fake/fake_device_provider.hpp"
#include "shulib/hal/fake/fake_motor.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/hal/mechanism.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/manipulation/mechanism_op.hpp"
#include "shulib/manipulation/mechanism_outcome.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/robot/robot.hpp"
#include "shulib/runner/command_registry.hpp"
#include "shulib/sequence/run_guard.hpp"
#include "shulib/sim/degradation.hpp"
#include "shulib/sim/scenario.hpp"
#include "shulib/units/quantity.hpp"

namespace cfgns = shulib::config;
using motion_rig::MotionRig;
using motion_rig::PlantPacer;
using motion_rig::chassisConfig;
using motion_rig::headErr;
using motion_rig::motionConfig;
using motion_rig::plantConfig;
using motion_rig::posErr;
using shulib::chassis::Chassis;
using shulib::chassis::Routine;
using shulib::chassis::RoutineResult;
using shulib::competition::acceptanceRunEntry;
using shulib::competition::AutonContext;
using shulib::competition::AutonEntry;
using shulib::competition::AutonKind;
using shulib::competition::AutonRunOutcome;
using shulib::competition::AutonRunResult;
using shulib::competition::runAuton;
using shulib::competition::StartPose;
using shulib::control::ExitReason;
using shulib::diag::FaultCode;
using shulib::hal::BrakeMode;
using shulib::hal::DescriptionSource;
using shulib::hal::DrivetrainDescription;
using shulib::hal::IMechanism;
using shulib::hal::IMotor;
using shulib::hal::MotorCartridge;
using shulib::hal::MotorMechanism;
using shulib::hal::fake::FakeDeviceProvider;
using shulib::hal::fake::FakeMotor;
using shulib::hal::fake::FakeTelemetrySink;
using shulib::kinematics::IKinematics;
using shulib::kinematics::MatrixKinematics;
using shulib::kinematics::TankKinematics;
using shulib::kinematics::xDrive;
using shulib::manipulation::MechanismDeps;
using shulib::manipulation::MechanismOutcome;
using shulib::manipulation::RunUntilConfirmed;
using shulib::manipulation::RunUntilConfirmedConfig;
using shulib::manipulation::StallConfig;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::sequence::GuardedWaitResult;
using shulib::sequence::RunGuard;
using shulib::sim::DegradationModel;
using shulib::sim::Rng;
using shulib::sim::SimHarness;
using shulib::sim::SimHarnessConfig;
using shulib::sim::TruthSample;
using shulib::units::AngularVelocity;
using shulib::units::Current;
using shulib::units::Length;
using shulib::units::Time;
using shulib::units::Voltage;

namespace {

constexpr double kTick = 0.01;
constexpr double kDeg = Angle::kPi / 180.0;
const Time kMatch{30.0};

// ── the plant's IMU, made to DISAGREE with the declared start (B13) ────────────────────

/// The IMU reads `truth + offsetDeg`, and — until `readyAt` — reports NOT ready while serving
/// calibration garbage that MOVES (the A3 finding: a calibrating IMU is not a quiet zero). The
/// garbage's phase is chosen so it is far from every post-calibration reading at T+0, the instant
/// runAuton is called: a garbage reading of exactly 0 there once let a "re-seat before the wait"
/// mutant compute the right bias by accident (M09b, found green and closed by this phase).
struct DisagreeingImu final : DegradationModel {
    DisagreeingImu(double offset, double ready) : offsetDeg{offset}, readyAt{ready} {}
    [[nodiscard]] Angle imuHeading(Angle truth, Time now, Rng& /*rng*/) override {
        if (now.value() < readyAt) {
            return Angle::radians(2.5 * std::sin(9.0 * now.value() + 1.0));
        }
        return truth + Angle::degrees(offsetDeg);
    }
    [[nodiscard]] bool imuReady(bool r, Time now) override { return r && now.value() >= readyAt; }
    double offsetDeg;
    double readyAt;
};

// ── the competition program's stack on the plant: plant -> watch -> guard -> chassis ────

/// B7's watch reduced to a script (run_guard_abandon_test.cpp has the reasoning): asks the guard
/// to abandon on every pace from `disableAt` on, while a run is live.
struct WatchPacer final : shulib::motion::ITickPacer {
    WatchPacer(shulib::motion::ITickPacer& inner, shulib::hal::IClock& clock)
        : next{&inner}, clk{&clock} {}
    void pace() override {
        if (guard != nullptr && guard->running() && clk->now().value() >= disableAt) {
            guard->requestAbandon();
        }
        next->pace();
    }
    shulib::motion::ITickPacer* next;
    shulib::hal::IClock* clk;
    RunGuard* guard = nullptr;
    double disableAt = 1e9;
};

/// Records ground truth at every pace (the robot_builder_test RecordingPacer's shape) — for the
/// bit-identity comparisons. Samples AFTER the tick and BEFORE the plant advances.
struct RecordingPacer final : shulib::motion::ITickPacer {
    explicit RecordingPacer(SimHarness& harness) : h{&harness} {}
    void pace() override {
        if (truth.size() > 200000) {
            throw std::runtime_error("RecordingPacer: pace cap exceeded");
        }
        truth.push_back(h->sample());
        h->plant().step(Time{kTick});
    }
    SimHarness* h;
    std::vector<TruthSample> truth;
};

/// Counts every pace that reaches it, then forwards — the "counting inner pacer" of §2 item 1.
struct CountingPacer final : shulib::motion::ITickPacer {
    explicit CountingPacer(shulib::motion::ITickPacer& inner) : next{&inner} {}
    void pace() override {
        ++count;
        next->pace();
    }
    shulib::motion::ITickPacer* next;
    int count = 0;
};

bool sameTruth(const std::vector<TruthSample>& a, const std::vector<TruthSample>& b) {
    return a.size() == b.size()
           && (a.empty() || std::memcmp(a.data(), b.data(), a.size() * sizeof(TruthSample)) == 0);
}

/// The hand-wired competition stack over the A2 plant.
struct CompRig {
    FakeTelemetrySink sink;
    MotionRig rig;
    PlantPacer plant;
    WatchPacer watch;
    RunGuard guard;
    Chassis chassis;

    CompRig(const IKinematics& kin, const SimHarnessConfig& cfg, DegradationModel* degradation)
        : rig{kin, cfg, &sink, degradation},
          plant{rig.h},
          watch{plant, rig.h.clock()},
          guard{watch},
          chassis{rig.deps, guard, chassisConfig()} {
        watch.guard = &guard;
    }

    [[nodiscard]] double now() { return rig.h.clock().now().value(); }

    [[nodiscard]] bool lineContains(const char* subsystem, const std::string& needle) const {
        for (int i = 0; i < sink.size(); ++i) {
            if (sink.at(i).subsystem == subsystem
                && sink.at(i).message.find(needle) != std::string::npos) {
                return true;
            }
        }
        return false;
    }

    void checkDriveSafe() {
        for (const IMotor* m : rig.h.context().driveMotors()) {
            CHECK(m->commandedVoltage().value() == 0.0);
            CHECK(m->brakeMode() == BrakeMode::Brake);
        }
    }

    /// No estimator pathology was raised by the re-seat (B14): no ODO_STUCK, no IMPLAUSIBLE.
    void checkNoSeatFaults() {
        CHECK(rig.latch.raiseCount(FaultCode::OdoStuck) == 0);
        CHECK(rig.latch.raiseCount(FaultCode::Implausible) == 0);
    }
};

SimHarnessConfig at(const Pose2d& start) {
    SimHarnessConfig cfg = plantConfig();
    cfg.plant.initialPose = start;
    return cfg;
}

AutonEntry entryFor(const char* name, shulib::competition::AutonRoutine fn, StartPose start,
                    double endActionAt, double hardStopAt) {
    AutonEntry e{};
    e.name = name;
    e.kind = AutonKind::Match;
    e.routine = fn;
    e.start = start;
    e.endActionAt = Time{endActionAt};
    e.hardStopAt = Time{hardStopAt};
    return e;
}

// ── routines (function pointers: a routine is data the students register — no captures) ──

/// In the FIELD frame, from (-48, -24) facing 90 deg: turn to face +x, drive 24 in along it.
RoutineResult turnThenDriveEast(AutonContext& ctx) {
    Routine r{ctx.chassis, "turn-then-east"};
    r.turnTo(Angle::degrees(0.0), {.timeout = Time{4.0}})
        .moveTo(Pose2d{Length{-24.0}, Length{-24.0}, Angle::degrees(0.0)}, {.timeout = Time{5.0}});
    return r.result();
}

/// In the START frame: drive 24 in straight ahead.
RoutineResult straight24(AutonContext& ctx) {
    Routine r{ctx.chassis, "straight-24"};
    r.moveTo(Pose2d{Length{24.0}, Length{0.0}, Angle{}}, {.timeout = Time{5.0}});
    return r.result();
}

int gStallCalls = 0;
/// Stalls in the GUARD's wait (the wait the deadline can cut).
RoutineResult stallInGuardWait(AutonContext& ctx) {
    ++gStallCalls;
    Routine r{ctx.chassis, "stall-guard-wait"};
    r.then([&ctx] {
        return ctx.guard.waitFor([] { return false; }, Time{60.0}) == GuardedWaitResult::Satisfied;
    });
    return r.result();
}

/// Stalls in a FROZEN wait (Chassis::wait) the deadline CANNOT cut.
RoutineResult stallInFrozenWait(AutonContext& ctx) {
    ctx.chassis.wait(Time{5.0});
    Routine r{ctx.chassis, "after-frozen-wait"};
    r.moveTo(Pose2d{Length{300.0}, Length{0.0}, Angle{}}, {.timeout = Time{60.0}});
    return r.result();
}

/// Drives toward a point it can never reach inside the budget.
RoutineResult driveFar(AutonContext& ctx) {
    Routine r{ctx.chassis, "drive-far"};
    r.moveTo(Pose2d{Length{300.0}, Length{0.0}, Angle{}}, {.timeout = Time{60.0}});
    return r.result();
}

const Pose2d kPark{Length{8.0}, Length{-10.0}, Angle::degrees(0.0)};
bool parkEndAction(AutonContext& ctx) {
    return ctx.chassis.moveTo(kPark, {.timeout = Time{5.0}}) == ExitReason::Settled;
}
int gParkCalls = 0;
/// The same park, counting its calls — so a case can prove the ENTRY's end action never ran.
bool countingParkEndAction(AutonContext& ctx) {
    ++gParkCalls;
    return parkEndAction(ctx);
}
bool hangingEndAction(AutonContext& ctx) {
    return ctx.chassis.moveTo(Pose2d{Length{-300.0}, Length{0.0}, Angle{}},
                              {.timeout = Time{60.0}})
           == ExitReason::Settled;
}

/// B16's four legs, from a start that is NOT the origin and NOT facing 0 (a zero start would let a
/// runner that seats a route from the wrong source — or not at all — pass unseen): from (-48, -24)
/// facing 90 deg, 24 in north, turn to face +x, 24 in east. In the FIELD frame, ending at
/// (-24, 0) facing 0.
const Pose2d kRouteStart{Length{-48.0}, Length{-24.0}, Angle::degrees(90.0)};
const Pose2d kRouteEnd{Length{-24.0}, Length{0.0}, Angle::degrees(0.0)};

/// The same four legs the route below carries, as a Routine (B16's differential).
RoutineResult fourLegs(AutonContext& ctx) {
    Routine r{ctx.chassis, "four-legs"};
    r.startAt(kRouteStart)
        .moveTo(Pose2d{Length{-48.0}, Length{0.0}, Angle::degrees(90.0)})
        .turnTo(Angle::degrees(0.0))
        .moveTo(kRouteEnd);
    return r.result();
}

cfgns::Route fourLegRoute() {
    cfgns::Route r{};
    std::strcpy(r.name, "four_legs");
    r.waypoints[0].motion = cfgns::WaypointMotion::Start;  // headingMode Fixed: it STATES 90 deg
    r.waypoints[0].x = kRouteStart.x();
    r.waypoints[0].y = kRouteStart.y();
    r.waypoints[0].heading = kRouteStart.heading();
    r.waypoints[1].motion = cfgns::WaypointMotion::MoveToPose;
    r.waypoints[1].x = Length{-48.0};
    r.waypoints[1].y = Length{0.0};
    r.waypoints[1].heading = Angle::degrees(90.0);
    r.waypoints[2].motion = cfgns::WaypointMotion::TurnTo;
    r.waypoints[2].heading = Angle::degrees(0.0);
    r.waypoints[3].motion = cfgns::WaypointMotion::MoveToPose;
    r.waypoints[3].x = kRouteEnd.x();
    r.waypoints[3].y = kRouteEnd.y();
    r.waypoints[3].heading = kRouteEnd.heading();
    r.count = 4;
    return r;
}

/// A mechanism for the routines below to reach without captures (a routine is a plain function).
struct Pivot {
    FakeMotor motor;
    std::array<IMotor*, 1> motors{&motor};
    MotorMechanism mech;
    MechanismDeps deps;
    RunUntilConfirmedConfig cfg{.voltage = Voltage{9.0},
                                .timeout = Time{60.0},
                                .stall = StallConfig{.currentAtLeast = Current{999.0},
                                                     .speedAtMost = AngularVelocity{0.0},
                                                     .persistence = Time{10.0}}};
    Pivot(CompRig& g, const char* name)
        : mech{motors, BrakeMode::Hold, name},
          deps{.clock = &g.rig.h.clock(), .faults = &g.rig.latch, .telemetry = &g.sink} {}
};
Pivot* gPivot = nullptr;
bool gPivotSafeAtEndAction = false;
bool gConfirmed = false;
bool confirmedFn() { return gConfirmed; }
/// The operation OUTLIVES the routine (a global), so what ends it is provably the GUARD's
/// cancel-all — not the operation's own cancel-on-destruction when the routine's stack unwinds.
std::optional<RunUntilConfirmed<bool (*)()>> gOp;

RoutineResult grabUntilCut(AutonContext& ctx) {
    gOp.emplace(gPivot->mech, gPivot->deps, gPivot->cfg, &confirmedFn, "grab");
    Routine r{ctx.chassis, "grab-until-cut"};
    r.then([&ctx] {
        gOp->start();
        return ctx.guard.waitFor([] { return gOp->tick() != MechanismOutcome::Running; },
                                 Time{60.0})
               == GuardedWaitResult::Satisfied;
    });
    return r.result();
}
bool noteSafeEndAction(AutonContext& ctx) {
    gPivotSafeAtEndAction = !gPivot->mech.claimed()
                            && gPivot->motor.commandedVoltage().value() == 0.0
                            && gPivot->motor.brakeMode() == BrakeMode::Hold;
    return ctx.chassis.brake() == ExitReason::Settled;
}

RoutineResult energizeThenThrow(AutonContext& ctx) {
    ctx.chassis.drive(shulib::math::ChassisSpeeds{shulib::units::Velocity{20.0},
                                                  shulib::units::Velocity{0.0},
                                                  AngularVelocity{0.0}},
                      shulib::math::Frame::Body);
    static_cast<MotorMechanism*>(ctx.mechanisms[0])->setVoltage(Voltage{9.0});
    throw std::runtime_error("author bug");
}

}  // namespace

// ═══ the start pose (C2, B12, B13, B14) ═════════════════════════════════════════════════

// Bug caught (B13, the mutation "drop the heading re-seat"): a runner that seats the position but
// leaves the heading to the IMU. The robot is placed at (-48, -24) FACING 90 deg; the IMU reads
// 0 deg there (truth - 90 deg: a V5 IMU boots at 0 whichever way the robot faces — HA-05). The
// routine turns to face +x and drives 24 in. With the re-seat, truth ends at (-24, -24) facing 0;
// without it the estimate is rotated 90 deg from truth, the "turn" is skipped as already done and
// the drive goes north — about 34 in from the right place.
TEST_CASE("B13 FIELD start: the IMU disagrees by 90 deg and the re-seat still lands the routine") {
    const TankKinematics kin{Length{12.0}};
    DisagreeingImu imu{-90.0, 0.0};
    const Pose2d start{Length{-48.0}, Length{-24.0}, Angle::degrees(90.0)};
    CompRig g{kin, at(start), &imu};
    const AutonEntry e = entryFor("field start", &turnThenDriveEast,
                                  StartPose::field(start), 20.0, 25.0);

    const AutonRunResult res = runAuton(g.chassis, g.guard, e, nullptr, g.sink, {}, kMatch);

    REQUIRE(res.outcome == AutonRunOutcome::RoutineRan);
    CHECK(res.imuAtSeat.approxEqual(Angle::degrees(0.0), 1e-9));  // the disagreement, stated
    CHECK(res.seated.heading().approxEqual(Angle::degrees(90.0), 1e-12));
    CHECK(res.routine.ok);
    const Pose2d expected{Length{-24.0}, Length{-24.0}, Angle::degrees(0.0)};
    CHECK(posErr(g.rig.h.truePose(), expected) < 1.0);
    CHECK(headErr(g.rig.h.truePose(), expected) < 2.0 * kDeg);
    CHECK(res.guard.endActionSucceeded);
    CHECK(res.guard.pacesSeen > 0);
    CHECK(g.lineContains("COMP", "re-seated FIELD"));
    g.checkNoSeatFaults();  // B14: the seat itself raised no pathology
    g.checkDriveSafe();
}

// Bug caught: RELATIVE start not zeroing the pre-auton drift. The robot is placed facing 30 deg in
// the field; the IMU booted at the placement (0) and then DRIFTED 15 deg while the robot waited
// (it now reads 15 deg with the robot unmoved). RELATIVE says: wherever it stands is (0, 0) facing
// 0 — so "drive 24 in straight" must go along the robot's own 30 deg line, ending 24 in out along
// it. Without the re-seat the estimate starts at 15 deg and the leg is aimed 15 deg off.
TEST_CASE("B13 RELATIVE start: a 15 deg pre-auton drift is zeroed at the start") {
    const TankKinematics kin{Length{12.0}};
    DisagreeingImu imu{-30.0 + 15.0, 0.0};
    const Pose2d placed{Length{10.0}, Length{5.0}, Angle::degrees(30.0)};
    CompRig g{kin, at(placed), &imu};
    const AutonEntry e = entryFor("relative start", &straight24, StartPose::relative(), 20.0, 25.0);

    const AutonRunResult res = runAuton(g.chassis, g.guard, e, nullptr, g.sink, {}, kMatch);

    REQUIRE(res.outcome == AutonRunOutcome::RoutineRan);
    CHECK(res.imuAtSeat.approxEqual(Angle::degrees(15.0), 1e-9));  // the drift, stated
    CHECK(res.routine.ok);
    const Pose2d expected{Length{10.0 + 24.0 * std::cos(30.0 * kDeg)},
                          Length{5.0 + 24.0 * std::sin(30.0 * kDeg)}, Angle::degrees(30.0)};
    CHECK(posErr(g.rig.h.truePose(), expected) < 1.0);
    CHECK(headErr(g.rig.h.truePose(), expected) < 2.0 * kDeg);
    g.checkNoSeatFaults();
}

// Bug caught (B12, the mutation "re-seat before the wait"): an auton started while the IMU is still
// calibrating. The guard arms FIRST (the schedule runs from the auton's start), the runner waits
// MOTIONLESS for a live estimate with the guard's own wait, and only then re-seats — a re-seat
// during calibration would bake a moving garbage heading into the bias (the Localizer refuses it
// loudly). The run is slow, not wrong.
TEST_CASE("B12 the IMU still calibrating: arm, wait for a live estimate, THEN re-seat") {
    const TankKinematics kin{Length{12.0}};
    DisagreeingImu imu{-90.0, 2.0};  // ready at T+2.0 s, and then 90 deg off, as above
    const Pose2d start{Length{-48.0}, Length{-24.0}, Angle::degrees(90.0)};
    CompRig g{kin, at(start), &imu};
    const AutonEntry e = entryFor("early start", &turnThenDriveEast,
                                  StartPose::field(start), 20.0, 25.0);

    const AutonRunResult res = runAuton(g.chassis, g.guard, e, nullptr, g.sink, {}, kMatch);

    REQUIRE(res.outcome == AutonRunOutcome::RoutineRan);
    CHECK(res.armedAt.value() == 0.0);          // armed at the call, not after the wait (B12)
    CHECK(res.liveAfter.value() >= 2.0);        // it waited out the calibration...
    CHECK(res.liveAfter.value() < 2.0 + 0.3);   // ...plus the settle window, not more
    CHECK(res.imuAtSeat.approxEqual(Angle::degrees(0.0), 1e-9));  // seated on the LIVE reading
    CHECK(res.routine.ok);
    const Pose2d expected{Length{-24.0}, Length{-24.0}, Angle::degrees(0.0)};
    CHECK(posErr(g.rig.h.truePose(), expected) < 1.0);
    CHECK(headErr(g.rig.h.truePose(), expected) < 2.0 * kDeg);
}

// Bug caught: a runner that starts the routine on a dead estimate (or spins on it past the
// deadline). An IMU that never goes live before the deadline: the auton never starts, nothing is
// re-seated, the result SAYS so, and the guard still ends the run safe.
TEST_CASE("B12 an estimate that never goes live: NotStarted, said out loud, robot safe") {
    const MatrixKinematics kin = xDrive(Length{7.0});
    DisagreeingImu imu{0.0, 1000.0};
    CompRig g{kin, at(Pose2d{}), &imu};
    gStallCalls = 0;
    const AutonEntry e = entryFor("never live", &stallInGuardWait, StartPose::relative(), 2.0, 3.0);

    const AutonRunResult res = runAuton(g.chassis, g.guard, e, nullptr, g.sink, {}, kMatch);

    CHECK(res.outcome == AutonRunOutcome::NotStarted);
    CHECK_FALSE(res.started());
    CHECK(gStallCalls == 0);  // the routine never ran
    CHECK(std::string{res.why}.find("never went live") != std::string::npos);
    CHECK(std::string{res.why}.find("ran out of time") != std::string::npos);
    CHECK(res.guard.scoringCut);
    CHECK(res.guard.scoringEnded.value() <= 2.0 + 2.0 * kTick);
    CHECK_FALSE(g.guard.running());
    g.checkDriveSafe();
}

// Bug caught (the COMP-a review's finding 3): a run that never STARTED handing its end-of-run slot
// to the ENTRY's park anyway. That park is written for the frame the re-seat establishes, and a
// NotStarted run never seated one. Built so the hazard is live: the IMU comes ready at T+2.1,
// just AFTER endActionAt (T+2.0), inside the end action's runway — so a field-frame park would
// wait out the boot, then drive to (8, -10) in the BOOT frame (a heading of the IMU's 0 over a
// robot really facing 90 deg, a position never seated). The slot must be filled by brake: the
// park never called, the robot never moved, and the log says why.
TEST_CASE("B12 a run that never started brakes -- the entry's park is not driven in an un-seated frame") {
    const MatrixKinematics kin = xDrive(Length{7.0});
    DisagreeingImu imu{-90.0, 2.1};
    const Pose2d placed{Length{-48.0}, Length{-24.0}, Angle::degrees(90.0)};
    CompRig g{kin, at(placed), &imu};
    gStallCalls = 0;
    gParkCalls = 0;
    AutonEntry e = entryFor("late imu", &stallInGuardWait, StartPose::field(placed), 2.0, 6.0);
    e.endAction = &countingParkEndAction;

    const AutonRunResult res = runAuton(g.chassis, g.guard, e, nullptr, g.sink, {}, kMatch);

    CHECK(res.outcome == AutonRunOutcome::NotStarted);
    CHECK(gStallCalls == 0);
    CHECK(gParkCalls == 0);         // the entry's park never ran...
    CHECK(res.guard.endActionRan);  // ...the slot did, as brake
    CHECK(g.lineContains("COMP", "never started -- its end action is NOT run"));
    CHECK(posErr(g.rig.h.truePose(), placed) < 0.05);  // nothing drove anywhere
    CHECK(headErr(g.rig.h.truePose(), placed) < 0.5 * kDeg);
    CHECK_FALSE(g.guard.running());
    g.checkDriveSafe();
}

// Bug caught (the COMP-a review's finding 4): the likeliest real sequence on a field — the robot
// disabled while its IMU is still calibrating — reported as the wrong cause, or run through the
// end action, or re-seated anyway. The IMU is ready at T+2.0; the field disables at T+0.5. The
// live-wait must end AT the abandon (not at the deadline), the result must say it was abandoned
// (not "ran out of time"), the end action must be skipped, and nothing may be re-seated.
TEST_CASE("C5 an abandon during the live-wait: NotStarted, 'was abandoned', skipped, nothing seated") {
    const MatrixKinematics kin = xDrive(Length{7.0});
    DisagreeingImu imu{-90.0, 2.0};
    const Pose2d placed{Length{-48.0}, Length{-24.0}, Angle::degrees(90.0)};
    CompRig g{kin, at(placed), &imu};
    g.watch.disableAt = 0.5;
    gStallCalls = 0;
    gParkCalls = 0;
    AutonEntry e =
        entryFor("disabled early", &stallInGuardWait, StartPose::field(placed), 20.0, 25.0);
    e.endAction = &countingParkEndAction;

    const AutonRunResult res = runAuton(g.chassis, g.guard, e, nullptr, g.sink, {}, kMatch);

    CHECK(res.outcome == AutonRunOutcome::NotStarted);
    CHECK_FALSE(res.started());
    CHECK(gStallCalls == 0);
    CHECK(gParkCalls == 0);
    CHECK(std::string{res.why}.find("was abandoned") != std::string::npos);
    CHECK(std::string{res.why}.find("ran out of time") == std::string::npos);
    CHECK(res.guard.abandoned);
    CHECK_FALSE(res.guard.endActionRan);
    CHECK_FALSE(res.guard.scoringCut);  // the deadline (T+20) never came
    CHECK(res.liveAfter.value() >= 0.5);
    CHECK(res.liveAfter.value() <= 0.5 + 3.0 * kTick);  // the wait ended at the abandon
    // Nothing was re-seated: no seat line, the result's seat fields untouched, no bias.
    CHECK_FALSE(g.lineContains("COMP", "re-seated"));
    CHECK(res.seated.x().value() == 0.0);
    CHECK(res.seated.y().value() == 0.0);
    CHECK(res.seated.heading().radians() == 0.0);
    CHECK(g.chassis.deps().localizer->headingBias().value() == 0.0);
    CHECK(posErr(g.rig.h.truePose(), placed) < 1e-9);
    CHECK_FALSE(g.guard.running());
    g.checkDriveSafe();
}

// ═══ the guaranteed end (§2 item 6) ═════════════════════════════════════════════════════

// Bug caught (the mutation "skip the end action on a cut"): a routine stalled in the guard's own
// wait never reaching its park. The deadline cuts the wait AT endActionAt and the end action
// FIRES — the park is reached on the plant.
TEST_CASE("§2.6 the end action FIRES when the routine stalls in guard.waitFor") {
    const MatrixKinematics kin = xDrive(Length{7.0});
    CompRig g{kin, at(Pose2d{}), nullptr};
    AutonEntry e = entryFor("stall then park", &stallInGuardWait, StartPose::relative(), 2.0, 9.0);
    e.endAction = &parkEndAction;

    const AutonRunResult res = runAuton(g.chassis, g.guard, e, nullptr, g.sink, {}, kMatch);

    REQUIRE(res.outcome == AutonRunOutcome::RoutineRan);
    CHECK_FALSE(res.routine.ok);  // the chain stopped on the cut wait — honestly
    CHECK(res.guard.scoringCut);
    CHECK(res.guard.scoringEnded.value() >= 2.0);
    CHECK(res.guard.scoringEnded.value() <= 2.0 + 2.0 * kTick);
    CHECK(res.guard.endActionRan);
    CHECK(res.guard.endActionSucceeded);
    CHECK(posErr(g.rig.h.truePose(), kPark) < 2.0);
    CHECK_FALSE(res.guard.floorFired);
}

// Bug caught: the documentation of the frozen waits rotting (F2's formula, pinned from the runner's
// side). Chassis::wait CANNOT be cut: a 5 s wait begun at T+0 against a 1 s deadline runs to T+5,
// and only then does the chain's next motion get refused and the end action start. The guide tells
// students to use guard.pause()/guard.waitFor() for exactly this reason.
TEST_CASE("§2.6 a wait the guard CANNOT cut ends late by its own remainder, exactly") {
    const MatrixKinematics kin = xDrive(Length{7.0});
    CompRig g{kin, at(Pose2d{}), nullptr};
    const AutonEntry e = entryFor("frozen wait", &stallInFrozenWait, StartPose::relative(), 1.0, 9.0);

    const AutonRunResult res = runAuton(g.chassis, g.guard, e, nullptr, g.sink, {}, kMatch);

    REQUIRE(res.outcome == AutonRunOutcome::RoutineRan);
    CHECK(res.guard.scoringCut);
    // The wait began when the estimate went live (liveAfter, one tick here) and ran its full 5 s:
    // it paid the whole remainder past the T+1 deadline, then ONE refused motion (a tick or two).
    const double waitEnded = res.liveAfter.value() + 5.0;
    CHECK(res.guard.scoringEnded.value() >= waitEnded - kTick);
    CHECK(res.guard.scoringEnded.value() <= waitEnded + 3.0 * kTick);
    CHECK(res.guard.postExpiryCancels >= 1);
    CHECK(posErr(g.rig.h.truePose(), Pose2d{}) < 0.01);  // the refused leg moved nothing
    CHECK(res.guard.endActionRan);
}

// Bug caught: an end action that never ends leaving the robot live past the hard stop. The floor
// cuts it and forces every device safe.
TEST_CASE("§2.6 the hard stop forces safe when the end action hangs") {
    const MatrixKinematics kin = xDrive(Length{7.0});
    CompRig g{kin, at(Pose2d{}), nullptr};
    AutonEntry e = entryFor("hanging park", &straight24, StartPose::relative(), 1.0, 3.5);
    e.endAction = &hangingEndAction;

    const AutonRunResult res = runAuton(g.chassis, g.guard, e, nullptr, g.sink, {}, kMatch);

    CHECK(res.guard.floorFired);
    CHECK(res.guard.endActionRan);
    CHECK_FALSE(res.guard.endActionSucceeded);
    CHECK(res.guard.endActionEnded.value() <= 3.5 + 2.0 * kTick);
    CHECK(g.lineContains("SEQ", "hard stop"));
    CHECK(g.lineContains("COMP", "HARD STOP fired"));
    g.checkDriveSafe();
}

// Bug caught (the mutation "drop abandon()'s cancel"): a mid-auton disable that leaves the routine
// driving, or strands it. The field "disables" the robot at T+1.5 during the acceptance run: the
// motion is cut, the routine UNWINDS on its own stack and returns its verdict (nothing dangles),
// the end action is skipped, the guard disarms when run() returns, the drive is safe, and the
// scheduler is idle and callable by the next mode.
TEST_CASE("C5 an abandon mid-routine: cut, unwound, safe, and the chassis callable afterwards") {
    const TankKinematics kin{Length{12.0}};
    CompRig g{kin, at(Pose2d{}), nullptr};
    g.watch.disableAt = 1.5;
    const AutonEntry e = acceptanceRunEntry(AutonKind::Match, Time{20.0}, Time{25.0});

    const AutonRunResult res = runAuton(g.chassis, g.guard, e, nullptr, g.sink, {}, kMatch);

    REQUIRE(res.outcome == AutonRunOutcome::RoutineRan);  // it ran, and it returned
    CHECK_FALSE(res.routine.ok);
    CHECK(res.routine.cause == shulib::chassis::RoutineStopCause::MotionFailed);
    CHECK(res.routine.exit == ExitReason::Cancelled);
    CHECK(res.routine.skipped >= 1);
    CHECK(res.guard.abandoned);
    CHECK_FALSE(res.guard.endActionRan);
    CHECK_FALSE(g.guard.running());
    CHECK(g.lineContains("COMP", "ABANDONED"));
    g.checkDriveSafe();
    CHECK_FALSE(g.chassis.scheduler().hasActiveMotion());
    const Pose2d here = g.chassis.pose();
    CHECK(g.chassis.turnTo(here.heading() + Angle::degrees(20.0), {.timeout = Time{4.0}})
          == ExitReason::Settled);
}

// Bug caught: a throwing routine (a precondition — a programming error) leaving the drive or a
// mechanism energized, or the guard armed so the next mode's every pace is judged. The guard safes
// everything on the unwind and the exception reaches the caller (the auton task's body reports it).
TEST_CASE("C5 a throwing routine: everything safed, the guard disarmed, and it stays loud") {
    const MatrixKinematics kin = xDrive(Length{7.0});
    CompRig g{kin, at(Pose2d{}), nullptr};
    Pivot pivot{g, "left pivot"};
    std::array<IMechanism*, 1> mechs{&pivot.mech};
    const AutonEntry e = entryFor("throws", &energizeThenThrow, StartPose::relative(), 5.0, 6.0);

    CHECK_THROWS_AS((void)runAuton(g.chassis, g.guard, e, nullptr, g.sink, mechs, kMatch),
                    std::runtime_error);
    CHECK_FALSE(g.guard.running());
    g.checkDriveSafe();
    CHECK(pivot.motor.commandedVoltage().value() == 0.0);
    CHECK(pivot.motor.brakeMode() == BrakeMode::Hold);
}

// ═══ mechanisms (B9) ════════════════════════════════════════════════════════════════════

// Bug caught (B9): an absent pivot refusing the whole auton, or the PRESENT one left energized.
// The program passes only the Robot's present mechanisms (the absent one is logged by name there);
// the auton runs, and the present pivot is cancelled THROUGH ITS CLAIM and safe at the cut — the
// end action sees it safe — and safe at the end.
TEST_CASE("B9 one pivot absent: the auton runs; the present pivot is safed at the cut and the end") {
    const MatrixKinematics kin = xDrive(Length{7.0});
    CompRig g{kin, at(Pose2d{}), nullptr};
    Pivot present{g, "left pivot"};
    Pivot absent{g, "right pivot"};
    absent.motor.setVoltage(Voltage{3.0});  // untouched by the run: it is not in the span
    std::array<IMechanism*, 1> presentOnly{&present.mech};
    gPivot = &present;
    gPivotSafeAtEndAction = false;
    AutonEntry e = entryFor("grab", &grabUntilCut, StartPose::relative(), 1.5, 8.0);
    e.endAction = &noteSafeEndAction;

    const AutonRunResult res = runAuton(g.chassis, g.guard, e, nullptr, g.sink, presentOnly, kMatch);

    REQUIRE(res.outcome == AutonRunOutcome::RoutineRan);
    CHECK(res.guard.scoringCut);
    CHECK(gPivotSafeAtEndAction);  // safe AT the cut, before the end action ran
    CHECK_FALSE(present.mech.claimed());
    CHECK(present.motor.commandedVoltage().value() == 0.0);
    CHECK(present.motor.brakeMode() == BrakeMode::Hold);
    CHECK(absent.motor.commandedVoltage().value() == 3.0);
    REQUIRE(gOp.has_value());
    CHECK(gOp->outcome() == MechanismOutcome::Cancelled);  // the guard cancelled it, alive
    gOp.reset();
    gPivot = nullptr;
}

// ═══ refusals, before anything arms (C6, B9, B16) ═══════════════════════════════════════

// Bug caught (the mutation "arm an unset schedule", and its siblings): a runner that arms a run it
// should have refused — RunGuard::run would THROW on an unset schedule (on the field), and would
// happily run a hard stop past the period. Each is refused with a sentence naming the auton,
// BEFORE the guard arms: no "armed" line, no pace, no motion, the routine never called.
TEST_CASE("C6 refusals: unset schedule, past the period, no command registry, a null mechanism") {
    const MatrixKinematics kin = xDrive(Length{7.0});
    const cfgns::Route route = fourLegRoute();
    struct Case {
        const char* what;
        AutonEntry entry;
        bool nullMechanism;
        const char* needle;
    };
    AutonEntry unset = entryFor("unset", &stallInGuardWait, StartPose::relative(), 0.0, 0.0);
    AutonEntry late = entryFor("late", &stallInGuardWait, StartPose::relative(), 20.0, 35.0);
    AutonEntry routeEntry = entryFor("route", nullptr, StartPose{}, 20.0, 25.0);
    routeEntry.route = &route;
    AutonEntry fine = entryFor("fine", &stallInGuardWait, StartPose::relative(), 20.0, 25.0);
    const Case cases[] = {
        {"unset schedule", unset, false, "auton 'unset': its schedule is UNSET"},
        {"past the period", late, false, "past its MATCH period of 30.00 s"},
        {"route without registry", routeEntry, false, "needs a command registry"},
        {"null mechanism", fine, true, "mechanism slot 0 is null"},
    };
    for (const Case& c : cases) {
        CAPTURE(std::string{c.what});
        CompRig g{kin, at(Pose2d{}), nullptr};
        gStallCalls = 0;
        std::array<IMechanism*, 1> nullMech{nullptr};
        const std::span<IMechanism* const> mechs =
            c.nullMechanism ? std::span<IMechanism* const>{nullMech}
                            : std::span<IMechanism* const>{};
        AutonRunResult res{};
        CHECK_NOTHROW(res = runAuton(g.chassis, g.guard, c.entry, nullptr, g.sink, mechs, kMatch));
        CHECK(res.outcome == AutonRunOutcome::Refused);
        CHECK(std::string{res.why}.find(c.needle) != std::string::npos);
        CHECK(g.lineContains("COMP", c.needle));
        CHECK_FALSE(g.lineContains("SEQ", "armed"));  // the guard never armed
        CHECK(res.guard.pacesSeen == 0);
        CHECK(gStallCalls == 0);
        CHECK(g.now() == 0.0);  // no time passed: nothing ran
    }
}

// Bug caught: the preconditions a PROGRAMMING error trips — a runner called while the guard is
// already running, or with a motion still armed (the re-seat would move a heading under it, B14).
TEST_CASE("B14 runAuton's preconditions: no live run, no active motion") {
    const MatrixKinematics kin = xDrive(Length{7.0});
    CompRig g{kin, at(Pose2d{}), nullptr};
    const AutonEntry e = entryFor("fine", &straight24, StartPose::relative(), 5.0, 6.0);
    shulib::motion::MoveToPose m{g.chassis.deps(), Pose2d{Length{10.0}, Length{0.0}, Angle{}},
                                 motionConfig(), 5.0};
    g.chassis.scheduler().async(m);
    CHECK_THROWS_AS((void)runAuton(g.chassis, g.guard, e, nullptr, g.sink, {}, kMatch),
                    shulib::PreconditionError);
    g.chassis.cancel();
    bool threwInside = false;
    (void)g.guard.run(
        g.chassis, shulib::sequence::RunGuardConfig{.endActionAt = Time{1.0}, .hardStopAt = Time{2.0}},
        [&] {
            try {
                (void)runAuton(g.chassis, g.guard, e, nullptr, g.sink, {}, kMatch);
            } catch (const shulib::PreconditionError&) {
                threwInside = true;
            }
        },
        [] { return true; });
    CHECK(threwInside);
}

// ═══ routes (B16) ═══════════════════════════════════════════════════════════════════════

// Bug caught (B16, and the COMP-a review's finding 1): a route entry SEATED from the wrong source —
// the entry's own (unset) start instead of its route's Start — or not seated at all; a route
// auton that behaves differently from the same legs written as a routine; a Start leg that undoes
// the seated heading. Built so each can fail (B13's rule): the route's Start is (-48, -24) facing
// 90 deg, the plant stands there, and its IMU reads 0 deg (truth - 90). A runner that seated a
// route from `entry.start` would seat (0, 0) facing 0 over an IMU of 0 — a zero bias — and one
// that skipped the seat would leave the raw 0 deg too; the Start leg then puts the POSITION right
// and the whole route drives rotated by 90 deg. So: the result says the seat came from the
// route's Start; ground truth ends at the field-frame end pose; and the same legs as a Routine,
// seated from the same pose over the same disagreeing IMU, drive the plant BIT-IDENTICALLY at every
// tick. The result is tagged: RouteRan with the route's report, RoutineRan with the routine's.
TEST_CASE("B16 a route entry is seated from its Start, and runs bit-identically to the same routine") {
    const MatrixKinematics kin = xDrive(Length{7.0});
    const cfgns::Route route = fourLegRoute();
    shulib::runner::CommandRegistry commands;

    struct RecRig {
        DisagreeingImu imu{-90.0, 0.0};  // truth 90 deg at the Start; the IMU reads 0 (B13)
        FakeTelemetrySink sink;
        MotionRig rig;
        RecordingPacer rec;
        RunGuard guard;
        Chassis chassis;
        explicit RecRig(const IKinematics& k)
            : rig{k, at(kRouteStart), &sink, &imu}, rec{rig.h}, guard{rec},
              chassis{rig.deps, guard, chassisConfig()} {}
        [[nodiscard]] bool lineContains(const std::string& needle) const {
            for (int i = 0; i < sink.size(); ++i) {
                if (sink.at(i).subsystem == "COMP"
                    && sink.at(i).message.find(needle) != std::string::npos) {
                    return true;
                }
            }
            return false;
        }
    };
    RecRig a{kin};
    RecRig b{kin};

    AutonEntry routeEntry = entryFor("as route", nullptr, StartPose{}, 25.0, 28.0);
    routeEntry.route = &route;
    REQUIRE_FALSE(routeEntry.start.isSet());  // the seat can only come from the route's Start
    const AutonEntry routineEntry =
        entryFor("as routine", &fourLegs, StartPose::field(kRouteStart), 25.0, 28.0);

    const AutonRunResult ra = runAuton(a.chassis, a.guard, routeEntry, &commands, a.sink, {}, kMatch);
    const AutonRunResult rb = runAuton(b.chassis, b.guard, routineEntry, nullptr, b.sink, {}, kMatch);

    REQUIRE(ra.outcome == AutonRunOutcome::RouteRan);
    REQUIRE(rb.outcome == AutonRunOutcome::RoutineRan);
    CHECK(ra.route.ok());
    CHECK(ra.route.legsRun == 4);
    CHECK(rb.routine.ok);
    CHECK(rb.routine.completed == 4);
    // Seated FROM THE ROUTE'S START, over an IMU that disagreed by the full 90 deg.
    CHECK(ra.seated.x().value() == -48.0);
    CHECK(ra.seated.y().value() == -24.0);
    CHECK(ra.seated.heading().approxEqual(Angle::degrees(90.0), 1e-12));
    CHECK(ra.imuAtSeat.approxEqual(Angle::degrees(0.0), 1e-9));
    CHECK(a.lineContains("re-seated FIELD (-48.00, -24.00) in facing 90.0 deg"));
    // In the field frame, on ground truth: the route lands where its legs say.
    CHECK(posErr(a.rig.h.truePose(), kRouteEnd) < 1.0);
    CHECK(headErr(a.rig.h.truePose(), kRouteEnd) < 2.0 * kDeg);
    CHECK(a.rig.latch.raiseCount(FaultCode::OdoStuck) == 0);
    CHECK(a.rig.latch.raiseCount(FaultCode::Implausible) == 0);
    // And the routine twin, seated the same way, is the same robot to the bit.
    CHECK(rb.seated.x().value() == ra.seated.x().value());
    CHECK(rb.seated.y().value() == ra.seated.y().value());
    CHECK(rb.seated.heading().radians() == ra.seated.heading().radians());
    REQUIRE(a.rec.truth.size() == b.rec.truth.size());
    CHECK(sameTruth(a.rec.truth, b.rec.truth));
}

// ═══ robot two's shape, through the builder (C1, B18) ═══════════════════════════════════

namespace {

constexpr std::int8_t kLeft[5] = {1, -2, 3, -4, 5};
constexpr std::int8_t kRight[5] = {-11, 12, -13, 14, -15};
constexpr std::uint8_t kImuPort = 10;
constexpr double kTrackWidth = 12.0;
constexpr double kWheelDiameter = 4.0;  // NOT the plant's default (robot_builder_test's reason)

/// Robot two's shape: a five-motor-a-side tank on drive-encoder odometry, no GPS, no pods.
cfgns::RobotConfig robotTwoShape() {
    DrivetrainDescription d{.cartridge = MotorCartridge::Blue,
                            .imuPort = kImuPort,
                            .trackWidth = Length{kTrackWidth},
                            .source = DescriptionSource::RobotMeasured,
                            .provenance = "COMP-a host fixture: never measured on a robot"};
    for (std::size_t i = 0; i < 5; ++i) {
        d.left.signedPorts[i] = kLeft[i];
        d.right.signedPorts[i] = kRight[i];
    }
    d.left.count = 5;
    d.right.count = 5;
    d.left.inchesPerRadian = Length{0.5 * kWheelDiameter};
    d.right.inchesPerRadian = Length{0.5 * kWheelDiameter};
    cfgns::RobotConfig c{.identity = {.name = "Robot Two", .team = "1234A", .season = "Override"},
                         .drivetrain = {.kind = cfgns::DrivetrainKind::Tank, .tank = {.description = d}},
                         .odometry = {.kind = cfgns::OdometryKind::DriveEncoders}};
    c.tuning.motion = motionConfig();
    return c;
}

/// The host's "robot plugged in": the plant, a pacer, and a provider bound to the plant's devices
/// (robot_builder_test.cpp's PlantWorld, reduced to the tank). `Pacer` is the world's pacer type.
template <typename Pacer>
struct World {
    SimHarness h;
    FakeTelemetrySink sink;
    Pacer pacer;
    FakeDeviceProvider devices;

    World(const IKinematics& kin, const SimHarnessConfig& cfg)
        : h{kin, cfg}, sink{}, pacer{h}, devices{h.clock(), h.battery(), sink, pacer} {
        const cfgns::RobotConfig c = robotTwoShape();
        const DrivetrainDescription& d = c.drivetrain.tank.description;
        const shulib::hal::DriveSideDescription* sides[2] = {&d.left, &d.right};
        for (int wheel = 0; wheel < 2; ++wheel) {
            const shulib::hal::DriveSideDescription& side = *sides[wheel];
            devices.bindMotor(static_cast<std::uint8_t>(
                                  DrivetrainDescription::portMagnitude(side.signedPorts[0])),
                              h.motor(wheel));
            std::array<FakeMotor*, 4> extras{};
            for (std::size_t i = 1; i < side.count; ++i) {
                extras[i - 1] = &devices.fakeMotor(static_cast<std::uint8_t>(
                    DrivetrainDescription::portMagnitude(side.signedPorts[i])));
            }
            h.plant().attachCoupledMembers(wheel, std::span<FakeMotor* const>{extras.data(), 4});
        }
        devices.bindImu(kImuPort, h.imu());
    }
};

SimHarnessConfig robotTwoWorld(const Pose2d& start) {
    SimHarnessConfig cfg = plantConfig();
    cfg.plant.initialPose = start;
    cfg.plant.driveWheelDiameter = Length{kWheelDiameter};
    return cfg;
}

}  // namespace

// Bug caught (C1, "default bit-identical"): BuildOptions::pacer changing the robot. A Robot built
// with an IDLE guard as its pacer must drive the plant BIT-IDENTICALLY to one built with the
// provider's pacer — the guard is inert until run() arms it (run_guard.hpp), and forwarding the
// pacer must add nothing else.
TEST_CASE("C1 BuildOptions::pacer: an idle guard as the pacer is bit-identical to the default") {
    const TankKinematics kin{Length{kTrackWidth}};
    const Pose2d start{Length{-12.0}, Length{6.0}, Angle::degrees(30.0)};
    const Pose2d target{Length{-12.0 + 30.0 * std::cos(30.0 * kDeg)},
                        Length{6.0 + 30.0 * std::sin(30.0 * kDeg)}, Angle::degrees(30.0)};
    const cfgns::RobotConfig config = robotTwoShape();

    World<RecordingPacer> plain{kin, robotTwoWorld(start)};
    shulib::robot::Robot a{config, plain.devices};
    World<RecordingPacer> guarded{kin, robotTwoWorld(start)};
    RunGuard guard{guarded.pacer};
    shulib::robot::Robot b{config, guarded.devices, shulib::robot::BuildOptions{.pacer = &guard}};

    for (shulib::robot::Robot* r : {&a, &b}) {
        r->chassis().setPose(start);
        CHECK(r->chassis().moveTo(target, {.timeout = Time{8.0}}) == ExitReason::Settled);
        CHECK(r->chassis().turnTo(Angle::degrees(70.0), {.timeout = Time{8.0}})
              == ExitReason::Settled);
    }
    REQUIRE_FALSE(plain.pacer.truth.empty());
    CHECK(sameTruth(plain.pacer.truth, guarded.pacer.truth));
}

// Bug caught (C1, the mutation "drop the BuildOptions::pacer forwarding"): a Robot that keeps the
// provider's pacer when given the guard — every guarantee void, and silently. Through the builder,
// the acceptance run (B18) on robot two's shape: EVERY pace of the run passed through the guard
// (a counting pacer under it agrees with the guard's own count), and the run lands where the
// acceptance run says it does: back on its start mark, turned round.
TEST_CASE("C1/B18 through the builder: the guard sees every pace; the acceptance run lands") {
    const TankKinematics kin{Length{kTrackWidth}};
    const Pose2d placed{Length{-12.0}, Length{6.0}, Angle::degrees(30.0)};
    World<PlantPacer> world{kin, robotTwoWorld(placed)};
    CountingPacer counting{world.pacer};
    RunGuard guard{counting};
    shulib::robot::Robot robot{robotTwoShape(), world.devices,
                               shulib::robot::BuildOptions{.pacer = &guard}};
    const AutonEntry e = acceptanceRunEntry(AutonKind::Match, Time{25.0}, Time{28.0});

    const int before = counting.count;
    const AutonRunResult res =
        runAuton(robot.chassis(), guard, e, nullptr, world.sink, {}, kMatch);

    REQUIRE(res.outcome == AutonRunOutcome::RoutineRan);
    CHECK(res.routine.ok);
    CHECK(res.routine.completed == 4);
    CHECK(res.guard.pacesSeen > 0);
    CHECK(res.guard.pacesSeen == counting.count - before);  // every pace went through the guard
    CHECK(res.guard.endActionSucceeded);
    const Pose2d expected{placed.x(), placed.y(), placed.heading() + Angle::degrees(180.0)};
    CHECK(posErr(world.h.truePose(), expected) < 1.5);
    CHECK(headErr(world.h.truePose(), expected) < 3.0 * kDeg);
    CHECK(robot.faults().raiseCount(FaultCode::OdoStuck) == 0);
    CHECK(robot.faults().raiseCount(FaultCode::Implausible) == 0);
}

// Bug caught (C1, "a routine is cut at endActionAt on the plant"): the guard wired through the
// builder but not in the loop. A leg that cannot finish is cut AT the deadline, and the plant stops
// moving there.
TEST_CASE("C1 through the builder: a routine is cut at endActionAt on the plant") {
    const TankKinematics kin{Length{kTrackWidth}};
    World<PlantPacer> world{kin, robotTwoWorld(Pose2d{})};
    RunGuard guard{world.pacer};
    shulib::robot::Robot robot{robotTwoShape(), world.devices,
                               shulib::robot::BuildOptions{.pacer = &guard}};
    const AutonEntry e = entryFor("far", &driveFar, StartPose::relative(), 2.0, 6.0);

    const AutonRunResult res =
        runAuton(robot.chassis(), guard, e, nullptr, world.sink, {}, kMatch);

    CHECK(res.guard.scoringCut);
    CHECK(res.guard.scoringEnded.value() >= 2.0);
    CHECK(res.guard.scoringEnded.value() <= 2.0 + 2.0 * kTick);
    CHECK(res.routine.exit == ExitReason::Cancelled);
    CHECK(world.h.truePose().x().value() > 5.0);    // it was driving…
    CHECK(world.h.truePose().x().value() < 150.0);  // …and it did not get there
    CHECK_FALSE(guard.running());
}
