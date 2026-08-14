// F1 INTEGRATION — a mechanism operation inside the existing loop owners: the
// scheduler's waitUntil (the concurrency the holonomic scoring cycle needs)
// and Routine::then() (the recipe seam). Every case names the bug it would
// catch.
//
// The mechanism's motors here are ALWAYS separate FakeMotors, never the
// drivetrain's: an intake and a drive are different devices, and keeping them
// separate is what lets the bit-identity case prove the mechanism changed
// NOTHING about the motion.

#include "doctest.h"

#include <array>
#include <string>

#include "mechanism_test_rig.hpp"
#include "motion_test_rig.hpp"
#include "shulib/chassis/chassis.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/hal/fake/fake_digital_out.hpp"
#include "shulib/hal/fake/fake_motor.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/hal/mechanism.hpp"
#include "shulib/kinematics/x_drive.hpp"
#include "shulib/manipulation/mechanism_op.hpp"
#include "shulib/motion/move_to_pose.hpp"

using motion_rig::ChassisRig;
using motion_rig::SchedulerRig;
using motion_rig::chassisConfig;
using motion_rig::motionConfig;
using motion_rig::posErr;
using shulib::chassis::Routine;
using shulib::chassis::RoutineResult;
using shulib::chassis::RoutineStopCause;
using shulib::control::ExitReason;
using shulib::diag::FaultCode;
using shulib::hal::BrakeMode;
using shulib::hal::IDigitalOut;
using shulib::hal::IMotor;
using shulib::hal::MotorMechanism;
using shulib::hal::PneumaticMechanism;
using shulib::hal::fake::FakeDigitalOut;
using shulib::hal::fake::FakeMotor;
using shulib::hal::fake::FakeTelemetrySink;
using shulib::kinematics::xDrive;
using shulib::manipulation::ActuateAndConfirm;
using shulib::manipulation::ActuateAndConfirmConfig;
using shulib::manipulation::MechanismDeps;
using shulib::manipulation::MechanismOutcome;
using shulib::manipulation::RunUntilConfirmed;
using shulib::manipulation::RunUntilConfirmedConfig;
using shulib::manipulation::StallConfig;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::motion::MoveToPose;
using shulib::motion::WaitResult;
using shulib::units::AngularVelocity;
using shulib::units::Current;
using shulib::units::Length;
using shulib::units::Time;
using shulib::units::Voltage;

namespace {

const Pose2d kTarget{Length{40.0}, Length{0.0}, Angle{}};

RunUntilConfirmedConfig intakeCfg(double timeout = 2.0) {
    return {.voltage = Voltage{6.0},
            .timeout = Time{timeout},
            .stall = StallConfig{.currentAtLeast = Current{2.0},
                                 .speedAtMost = AngularVelocity{0.1},
                                 .persistence = Time{0.05}}};
}

/// Every DRIVE motor at 0 V under Brake — the drive's parked state (the
/// recipe error policy), asserted at the plant's fake motors.
template <typename Rig>
void checkDriveParked(Rig& rig) {
    for (int w = 0; w < rig.h.motorCount(); ++w) {
        CHECK(rig.h.motor(w).commandedVoltage().value() == 0.0);
        CHECK(rig.h.motor(w).brakeMode() == BrakeMode::Brake);
    }
}

int countRtnWarns(const FakeTelemetrySink& sink, const char* needle) {
    int n = 0;
    for (int i = 0; i < sink.size(); ++i) {
        const auto& e = sink.at(i);
        if (e.level == shulib::hal::LogLevel::Warn && e.subsystem == "RTN" &&
            e.message.find(needle) != std::string::npos) {
            ++n;
        }
    }
    return n;
}

}  // namespace

// THE T3 CONCURRENCY PROOF — "intake while driving", through the EXISTING
// scheduler seam, with the world evolution proven bit-identical to a run with
// no mechanism at all.
// Bug caught: a mechanism tick tripping the scheduler's re-entrancy guards
// (inTick_/inWait_), the wait double-ticking either party (the integer tick
// arithmetic below cannot survive a double tick), or the mechanism's presence
// perturbing the motion (any perturbation breaks EXACT pose equality).
TEST_CASE("A mechanism operates while a motion runs: both progress, motion unchanged") {
    const auto kin = xDrive(Length{7.0});

    // Run 1: motion + mechanism operating inside the waitUntil predicate.
    SchedulerRig s{kin};
    FakeMotor ma;
    FakeMotor mb;
    std::array<IMotor*, 2> motors{&ma, &mb};
    MotorMechanism intake{motors, BrakeMode::Coast, "intake"};
    MechanismDeps deps{.clock = &s.rig.h.clock(),
                       .faults = &s.rig.latch,
                       .telemetry = &s.rig.faultSink};
    int iter = 0;  // the test's own count of predicate evaluations
    RunUntilConfirmed op{intake, deps, intakeCfg(5.0), [&] { return iter > 40; },
                         "capture"};

    MoveToPose m{s.sched.deps(), kTarget, motionConfig(), 8.0};
    s.sched.async(m);
    op.start();
    const WaitResult w = s.sched.waitUntil(
        [&] {
            ++iter;
            return op.tick() != MechanismOutcome::Running;
        },
        5.0);

    // Hand arithmetic, integer-exact: pred call k has iter == k; the op's
    // confirm (iter > 40) first holds on call 41, so the op succeeds on its
    // 41st tick, after exactly 40 scheduler ticks of the motion.
    CHECK(w == WaitResult::Satisfied);
    CHECK(iter == 41);
    CHECK(op.outcome() == MechanismOutcome::Succeeded);
    // Both progressed: the op finished, and the motion is STILL DRIVING —
    // 0.4 s into an ~4 s move (no pre-emption, no precondition trip).
    CHECK(s.sched.hasActiveMotion());
    // The intake ended in ITS safe state while the drive keeps its command.
    CHECK(ma.commandedVoltage().value() == 0.0);
    CHECK(ma.brakeMode() == BrakeMode::Coast);

    CHECK(s.sched.waitUntilSettled() == ExitReason::Settled);
    const Pose2d withMech = s.rig.h.truePose();
    CHECK(posErr(withMech, kTarget) < 0.6);

    // Run 2: the twin with NO mechanism — same seed, same wait shape.
    SchedulerRig s2{kin};
    MoveToPose m2{s2.sched.deps(), kTarget, motionConfig(), 8.0};
    s2.sched.async(m2);
    int iter2 = 0;
    const WaitResult w2 = s2.sched.waitUntil(
        [&] {
            ++iter2;
            return iter2 >= 41;
        },
        5.0);
    CHECK(w2 == WaitResult::Satisfied);
    CHECK(s2.sched.waitUntilSettled() == ExitReason::Settled);
    const Pose2d bare = s2.rig.h.truePose();

    // EXACT equality — the mechanism touched nothing the motion reads.
    CHECK(withMech.x().value() == bare.x().value());
    CHECK(withMech.y().value() == bare.y().value());
    CHECK(withMech.heading().radians() == bare.heading().radians());
}

// THE C2 ABORT-MASK CONSEQUENCE, confirmed deliberately rather than
// discovered later: MechanismStalled (bit 11) is NOT in the default abort
// mask, so a jammed intake mid-drive latches its fault and the DRIVE KEEPS
// DRIVING. Bug caught: the new code landing in the abort mask by accident
// (a jammed intake ending every auton), or the fault not latching at all.
TEST_CASE("A mechanism stall during a motion latches its fault and does NOT abort the drive") {
    const auto kin = xDrive(Length{7.0});
    SchedulerRig s{kin};
    FakeMotor ma;
    std::array<IMotor*, 1> motors{&ma};
    MotorMechanism intake{motors, BrakeMode::Coast, "intake"};
    ma.setCurrent(Current{2.5});             // jam signature from the start
    ma.setVelocity(AngularVelocity{0.02});
    MechanismDeps deps{.clock = &s.rig.h.clock(),
                       .faults = &s.rig.latch,
                       .telemetry = &s.rig.faultSink};
    RunUntilConfirmed op{intake, deps, intakeCfg(5.0), [] { return false; }, "capture"};

    MoveToPose m{s.sched.deps(), kTarget, motionConfig(), 8.0};
    s.sched.async(m);
    op.start();
    const WaitResult w = s.sched.waitUntil(
        [&] { return op.tick() != MechanismOutcome::Running; }, 5.0);

    CHECK(w == WaitResult::Satisfied);
    CHECK(op.outcome() == MechanismOutcome::Stalled);
    CHECK(s.rig.latch.raiseCount(FaultCode::MechanismStalled) == 1);
    // The drive survived its intake: still active, and it goes on to SETTLE.
    CHECK(s.sched.hasActiveMotion());
    CHECK(s.sched.waitUntilSettled() == ExitReason::Settled);
    CHECK(s.sched.lastCompleted().abortFault == FaultCode::None);
    CHECK(posErr(s.rig.h.truePose(), kTarget) < 0.6);
}

// ── then() integration ──────────────────────────────────────────────────────────────

// THE RECIPE IDIOM, end to end: start the op, let the chassis's own wait tick
// it, return the outcome. Bug caught: the idiom not working between motions
// (a broken interaction between then(), waitUntil with no active motion, and
// the op), or a successful mechanism step failing to continue the chain.
TEST_CASE("then(): the mechanism idiom — a confirmed grab continues the chain") {
    const auto kin = xDrive(Length{7.0});
    FakeTelemetrySink sink;
    ChassisRig c{kin, motion_rig::plantConfig(), &sink};
    FakeMotor ma;
    std::array<IMotor*, 1> motors{&ma};
    MotorMechanism intake{motors, BrakeMode::Coast, "intake"};
    MechanismDeps deps{.clock = &c.rig.h.clock(),
                       .faults = &c.rig.latch,
                       .telemetry = &sink};
    int iter = 0;
    RunUntilConfirmed op{intake, deps, intakeCfg(), [&] { return iter > 5; }, "grab"};

    Routine r{c.chassis, "score"};
    r.moveTo(Pose2d{Length{20.0}, Length{0.0}, Angle{}}, {.timeout = Time{8.0}})
        .then(
            [&] {
                op.start();
                (void)c.chassis.waitUntil(
                    [&] {
                        ++iter;
                        return op.tick() != MechanismOutcome::Running;
                    },
                    Time{2.0});
                return op.outcome();
            },
            "grab")
        .moveTo(Pose2d{Length{30.0}, Length{10.0}, Angle{}}, {.timeout = Time{8.0}});

    CHECK(r.ok());
    const RoutineResult res = r.result();
    CHECK(res.steps == 3);
    CHECK(res.completed == 3);
    CHECK(op.outcome() == MechanismOutcome::Succeeded);
}

// THE T2 GUARANTEE at the Routine layer: an unconfirmed operation CANNOT read
// as success. Bug caught: MechanismOutcome mapped truthily (Unconfirmed == 2
// would be "true"), the chain continuing past a failed grab, the stop cause
// not naming the mechanism class, the transcript not naming WHICH way it
// failed, or the drive not parked by the stop.
TEST_CASE("then(): an UNCONFIRMED grab stops the chain — never success") {
    const auto kin = xDrive(Length{7.0});
    FakeTelemetrySink sink;
    ChassisRig c{kin, motion_rig::plantConfig(), &sink};
    FakeDigitalOut line;
    std::array<IDigitalOut*, 1> lines{&line};
    PneumaticMechanism clamp{lines, false, "clamp"};
    MechanismDeps deps{.clock = &c.rig.h.clock(),
                       .faults = &c.rig.latch,
                       .telemetry = &sink};
    ActuateAndConfirm op{clamp, deps,
                         ActuateAndConfirmConfig{.target = true,
                                                 .actuationTime = Time{0.1},
                                                 .confirmWindow = Time{0.1}},
                         [] { return false; },  // the jaws closed on nothing
                         "grab"};

    Routine r{c.chassis, "score"};
    r.moveTo(Pose2d{Length{20.0}, Length{0.0}, Angle{}}, {.timeout = Time{8.0}})
        .then(
            [&] {
                op.start();
                (void)c.chassis.waitUntil(
                    [&] { return op.tick() != MechanismOutcome::Running; }, Time{2.0});
                return op.outcome();
            },
            "grab")
        .moveTo(Pose2d{Length{30.0}, Length{10.0}, Angle{}}, {.timeout = Time{8.0}})
        .brake();

    CHECK_FALSE(r.ok());  // THE assertion this chunk exists for
    const RoutineResult res = r.result();
    CHECK(res.cause == RoutineStopCause::MechanismFailed);
    CHECK(std::string{res.stoppedName} == "grab");
    CHECK(res.stoppedAt == 2);
    CHECK(res.steps == 4);
    CHECK(res.skipped == 2);  // the second move and the brake never ran
    CHECK(res.exit == ExitReason::Running);  // no MOTION verdict here (convention)
    // The transcript names the failure class AND the specific verdict.
    CHECK(countRtnWarns(sink, "mechanism FAILED") == 1);
    CHECK(countRtnWarns(sink, "UNCONFIRMED") == 1);
    // The stop parked the drive; the op had already disposed its mechanism.
    checkDriveParked(c.rig);
    CHECK(c.rig.latch.faultCount() == 0);  // unconfirmed is strategy — no fault
}

// Bug caught: a STALLED mechanism step failing to latch its fault through the
// recipe path, or reading as anything but a stopped chain.
TEST_CASE("then(): a STALLED grab stops the chain and latches MechanismStalled") {
    const auto kin = xDrive(Length{7.0});
    FakeTelemetrySink sink;
    ChassisRig c{kin, motion_rig::plantConfig(), &sink};
    FakeMotor ma;
    std::array<IMotor*, 1> motors{&ma};
    MotorMechanism intake{motors, BrakeMode::Coast, "intake"};
    ma.setCurrent(Current{2.5});
    ma.setVelocity(AngularVelocity{0.02});
    MechanismDeps deps{.clock = &c.rig.h.clock(),
                       .faults = &c.rig.latch,
                       .telemetry = &sink};
    RunUntilConfirmed op{intake, deps, intakeCfg(), [] { return false; }, "capture"};

    Routine r{c.chassis, "score"};
    r.then(
         [&] {
             op.start();
             (void)c.chassis.waitUntil(
                 [&] { return op.tick() != MechanismOutcome::Running; }, Time{2.0});
             return op.outcome();
         },
         "capture")
        .moveTo(kTarget, {.timeout = Time{8.0}});

    CHECK_FALSE(r.ok());
    CHECK(r.result().cause == RoutineStopCause::MechanismFailed);
    CHECK(r.result().skipped == 1);
    CHECK(countRtnWarns(sink, "STALLED") == 1);
    CHECK(c.rig.latch.raiseCount(FaultCode::MechanismStalled) == 1);
    CHECK(ma.commandedVoltage().value() == 0.0);  // intake safed by its own exit
}

// Bug caught: an author returning op.tick()'s or op.outcome()'s RUNNING to
// then() — an operation nobody drove to completion. It must stop the chain
// LOUDLY with RUNNING named, never count as success and never pass silently
// (the reviewer's 3(b): a silently truncated operation is a failure mode).
TEST_CASE("then(): a still-Running outcome stops the chain loudly") {
    const auto kin = xDrive(Length{7.0});
    FakeTelemetrySink sink;
    ChassisRig c{kin, motion_rig::plantConfig(), &sink};

    Routine r{c.chassis, "score"};
    r.then([&] { return MechanismOutcome::Running; }, "forgot-to-drive")
        .moveTo(kTarget, {.timeout = Time{8.0}});

    CHECK_FALSE(r.ok());
    CHECK(r.result().cause == RoutineStopCause::MechanismFailed);
    CHECK(countRtnWarns(sink, "RUNNING") == 1);
    CHECK(r.result().skipped == 1);
}

// Bug caught: the F1 then() branch changing what the EXISTING accepted return
// types mean — the three legacy forms must behave exactly as documented at
// D1/D3 (their own suites re-verify this in depth; this is the same-file
// sanity pin).
TEST_CASE("then(): void / bool / ExitReason forms are unchanged in meaning") {
    const auto kin = xDrive(Length{7.0});
    FakeTelemetrySink sink;
    ChassisRig c{kin, motion_rig::plantConfig(), &sink};

    Routine r{c.chassis, "legacy"};
    int ran = 0;
    r.then([&] { ++ran; })                            // void: always succeeds
        .then([&] { return true; }, "ok-bool")        // bool true: succeeds
        .then([&] { return ExitReason::Settled; },    // ExitReason honored
              "ok-exit");
    CHECK(r.ok());
    CHECK(ran == 1);
    CHECK(r.result().completed == 3);

    r.then([&] { return false; }, "fail-bool");  // bool false: ActionFailed
    CHECK_FALSE(r.ok());
    CHECK(r.result().cause == RoutineStopCause::ActionFailed);
    CHECK(r.result().stoppedAt == 4);
}
