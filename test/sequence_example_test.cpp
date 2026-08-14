// F2 DOC EXAMPLES — every ```cpp listing the F2 documentation added (guide
// chapters 6/9/12/14 + the cookbook's match-clock recipe) compiles and RUNS
// here, against the simulated plant, so a documented idiom can never become a
// lie with a code block around it (the C8/D3 anti-rot rule; the doc gate
// checks every quoted line appears verbatim in a test/*example*_test.cpp).
//
// WHERE A DURATION MATTERS, ASSERT AGAINST THE SIMULATED CLOCK — never a
// sibling literal (D2's green hole). Every schedule literal here is the
// EXAMPLE AUTHOR'S number, quoted as such in the docs: the library has no
// default to quote.

#include "doctest.h"

#include <array>

#include "motion_test_rig.hpp"
#include "shulib/chassis/chassis.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/hal/fake/fake_motor.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/hal/mechanism.hpp"
#include "shulib/sequence/run_guard.hpp"
#include "shulib/units/literals.hpp"

using namespace shulib::units::literals;
using motion_rig::MotionRig;
using motion_rig::PlantPacer;
using motion_rig::chassisConfig;
using motion_rig::posErr;
using shulib::chassis::Chassis;
using shulib::chassis::Routine;
using shulib::control::ExitReason;
using shulib::hal::BrakeMode;
using shulib::hal::IMechanism;
using shulib::hal::IMotor;
using shulib::hal::MotorMechanism;
using shulib::kinematics::MatrixKinematics;
using shulib::kinematics::xDrive;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::sequence::GuardedWaitResult;
using shulib::sequence::RunGuard;
using shulib::sequence::RunGuardConfig;
using shulib::sequence::RunGuardReport;
using shulib::units::Length;
using shulib::units::Time;
using shulib::units::Velocity;

// ── sequence-01: the whole guarded-run idiom (cookbook "Fit the match window";
// short forms quoted in guide ch. 6 and ch. 14) ─────────────────────────────────────
//
// One guard wraps the pacer; the Chassis is constructed WITH the guard; the
// entire auton runs inside guard.run() with both instants and the end action
// supplied by the author. The scoring chain here deliberately stalls (a leg
// its speed cannot finish inside the budget) so the test proves the
// documented promise: the end action still runs, the step after the cut is
// refused, and the robot ends where the AUTHOR said — graded on plant truth
// (the rig starts at the origin so truth and estimate share a frame).
TEST_CASE("sequence-01: the guarded run — scoring stalls, the end action still runs") {
    MatrixKinematics kin = xDrive(Length{7.0});
    shulib::hal::fake::FakeTelemetrySink sink;
    MotionRig rig{kin, motion_rig::plantConfig(), &sink};
    PlantPacer pacer{rig.h};
    const shulib::motion::MotionDeps& deps = rig.deps;
    const shulib::chassis::ChassisConfig config = chassisConfig();

    // The listing the docs quote, from here —
    shulib::sequence::RunGuard guard{pacer};   // wraps YOUR tick pacer
    Chassis chassis{deps, guard, config};      // the guard IS the pacer

    const Pose2d endPose{90_in, -24_in, 0_deg};     // YOUR end position (a fixture here)

    const RunGuardReport report = guard.run(
        chassis,
        {.endActionAt = 12_s, .hardStopAt = 14.5_s},  // YOUR schedule — no defaults exist
        [&] {
            Routine r{chassis, "skills"};
            r.startAt(Pose2d{0_in, 0_in, 0_deg})
                .moveTo(Pose2d{300_in, 0_in, 0_deg},
                        {.timeout = 60_s, .maxLinearSpeed = Velocity{10.0}})
                .moveTo(Pose2d{0_in, 24_in, 90_deg}, {.timeout = 6_s});
        },
        [&] {
            Routine end{chassis, "skills/end"};
            end.moveTo(endPose, {.timeout = 4_s}).brake({.timeout = 1_s});
            return end.ok();
        });

    if (!report.endActionSucceeded) { /* transcript already says why (SEQ lines) */ }
    // — to here.

    CHECK(report.scoringCut);       // the stalled leg was cut at T+12, honestly
    CHECK(report.endActionRan);
    CHECK(report.endActionSucceeded);
    CHECK(posErr(rig.h.truePose(), endPose) < 2.0);  // ground truth: it ended there
    // Exactly ONE post-deadline cancel — the cut. The second leg was never
    // even armed: the chain's own stop policy SKIPPED it (skips are instant),
    // which is why a Routine pays at most one step's lateness (guide ch. 9).
    CHECK(report.postExpiryCancels == 1);
    // The cut landed AT the deadline (the simulated clock, not a copied literal):
    CHECK(report.scoringEnded.value() >= 12.0);
    CHECK(report.scoringEnded.value() < 12.1);
}

// ── sequence-02: the deadline-aware wait inside a chain (guide ch. 9) ───────────────
//
// Routine::waitFor and pause are FROZEN and cannot see the run deadline (they
// pay their own full budget — ch. 9 documents the bound). The guard's waitFor
// is the deadline-aware spelling, and `== Satisfied` through then()'s bool arm
// stops the chain on EITHER failure: the wait's own timeout OR the run
// expiring. This test pins the expiring case: the chain must stop, and the
// step after the wait must never run.
TEST_CASE("sequence-02: guard.waitFor in a then() halts the chain at the deadline") {
    MatrixKinematics kin = xDrive(Length{7.0});
    MotionRig rig{kin};
    PlantPacer pacer{rig.h};
    RunGuard guard{pacer};
    Chassis chassis{rig.deps, guard, chassisConfig()};
    bool partnerSignal = false;  // never comes — the partner is stuck
    int stepsAfterWait = 0;

    (void)guard.run(
        chassis, {.endActionAt = 3_s, .hardStopAt = 10_s},
        [&] {
            Routine r{chassis, "with-partner"};
            r.startAt(Pose2d{0_in, 0_in, 0_deg})
                .then([&] { return guard.waitFor([&] { return partnerSignal; }, 30_s)
                                == GuardedWaitResult::Satisfied; },
                      "wait-partner")
                .then([&] { ++stepsAfterWait; }, "score");
            CHECK_FALSE(r.ok());  // the chain stopped at the wait
        },
        [] { return true; });

    CHECK(stepsAfterWait == 0);  // nothing scored past the deadline
    // The wait ended AT the deadline, not after its own 30 s budget:
    CHECK(rig.h.clock().now().value() < 4.0);
}

// ── sequence-03: hold position until the buzzer (guide ch. 14 / cookbook) ───────────
//
// remaining() during the end action counts down to the HARD STOP — the
// "stay parked, actively, until time is really up" beat. The pin: the end
// action consumed (nearly) the whole runway, and everything ended safe.
TEST_CASE("sequence-03: remaining() lets the end action hold to the buzzer") {
    MatrixKinematics kin = xDrive(Length{7.0});
    MotionRig rig{kin};
    PlantPacer pacer{rig.h};
    RunGuard guard{pacer};
    Chassis chassis{rig.deps, guard, chassisConfig()};
    chassis.setPose(Pose2d{});

    const RunGuardReport report = guard.run(
        chassis, {.endActionAt = 2_s, .hardStopAt = 6_s},
        [&] { (void)chassis.moveTo(Pose2d{300_in, 0_in, 0_deg}, {.timeout = 60_s}); },
        [&] {
            (void)chassis.moveTo(Pose2d{20_in, 0_in, 0_deg}, {.timeout = 3_s});
            if (guard.remaining() > 200_ms) {
                (void)chassis.hold(guard.remaining());  // actively hold to the buzzer
            }
            return true;
        });

    CHECK(report.endActionRan);
    // The hold really ran to (within a settle-check of) the floor:
    CHECK(report.endActionEnded.value() > 5.5);
    for (const IMotor* m : rig.h.context().driveMotors()) {
        CHECK(m->commandedVoltage().value() == 0.0);
        CHECK(m->brakeMode() == BrakeMode::Brake);
    }
}
