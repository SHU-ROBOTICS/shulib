// F2 DoD SUITE — THE POINT OF THE CHUNK (build-order: "a deliberately stalled
// scoring loop still ends parked, verified against the plant with the clock
// driven to the match limit. This test is the entire point of the chunk.").
// Every case names the bug it would catch.
//
// THE TRAP, in its F2 form, and this file's counter: if the test's notion of
// "the match ended" were the guard's own clock, a guard that never fires and
// a test that never checks would agree perfectly. So the match limit here is
// driven by an INDEPENDENT SCRIPT — TallyPacer counts its own paces and
// multiplies by its own dt; it reads no clock, no guard state, nothing the
// guard computes — and every claim is asserted against PLANT GROUND TRUTH
// (h.truePose(), the A2 truth integrator, deliberately independent of the
// estimate) plus raw device state, at the tick the INDEPENDENT tally says the
// match ended. The guard's RunGuardReport is read only to check the guard's
// own honesty, never as evidence the robot moved.
//
// The schedule in these tests (15 s limit, act at T+9, floor at T+14.5) is a
// TEST SCRIPT with no field claim: match length and lead time are the
// caller's numbers everywhere in this library, and the park pose is a
// fixture coordinate, not a field position.

#include "doctest.h"

#include <array>
#include <span>
#include <utility>
#include <vector>

#include "motion_test_rig.hpp"
#include "shulib/chassis/chassis.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/control/exit_group.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/hal/mechanism.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/manipulation/mechanism_op.hpp"
#include "shulib/manipulation/mechanism_outcome.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/motion/motion_scheduler.hpp"
#include "shulib/motion/move_to_pose.hpp"
#include "shulib/sequence/run_guard.hpp"
#include "shulib/units/quantity.hpp"

using motion_rig::MotionRig;
using motion_rig::PlantPacer;
using motion_rig::chassisConfig;
using motion_rig::motionConfig;
using motion_rig::posErr;
using shulib::chassis::Chassis;
using shulib::chassis::Routine;
using shulib::control::ExitReason;
using shulib::diag::FaultCode;
using shulib::hal::BrakeMode;
using shulib::hal::IMechanism;
using shulib::hal::IMotor;
using shulib::hal::MotorMechanism;
using shulib::hal::fake::FakeTelemetrySink;
using shulib::kinematics::MatrixKinematics;
using shulib::kinematics::xDrive;
using shulib::manipulation::MechanismDeps;
using shulib::manipulation::MechanismOutcome;
using shulib::manipulation::RunUntilConfirmed;
using shulib::manipulation::RunUntilConfirmedConfig;
using shulib::manipulation::StallConfig;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::motion::ITickPacer;
using shulib::motion::MoveToPose;
using shulib::sequence::GuardedWaitResult;
using shulib::sequence::RunGuard;
using shulib::sequence::RunGuardConfig;
using shulib::sequence::RunGuardReport;
using shulib::units::AngularVelocity;
using shulib::units::Current;
using shulib::units::Length;
using shulib::units::Time;
using shulib::units::Voltage;

namespace {

/// THE INDEPENDENT SCRIPT (file banner). Sits between the guard and the
/// plant; counts its own paces at its own dt; snapshots plant truth + raw
/// device state at the pace its OWN tally crosses the limit. Reads nothing
/// the guard computes — not the sim clock, not guard state, not the report.
struct TallyPacer final : ITickPacer {
    struct DeviceState {
        double volts = 0.0;
        BrakeMode mode = BrakeMode::Coast;
    };
    struct Snapshot {
        bool taken = false;
        Pose2d truth{};
        std::vector<DeviceState> drive;
        std::vector<DeviceState> mech;
    };

    TallyPacer(PlantPacer& inner, MotionRig& rig, double limitSeconds,
               std::vector<const IMotor*> mechMotors = {})
        : inner_{&inner}, rig_{&rig}, limit_{limitSeconds}, mechMotors_{std::move(mechMotors)} {}

    void pace() override {
        inner_->pace();
        elapsed_ += kDt;  // OWN tally: pace count × own dt, no clock read
        if (!snap_.taken && elapsed_ >= limit_ - 1e-9) {
            take();
        }
    }

    /// Drive the world to the limit after run() returned (nobody paces once
    /// the caller's code is done — the test owns post-run time).
    void driveToLimit(ITickPacer& viaChain) {
        while (!snap_.taken) {
            viaChain.pace();
        }
    }

    static constexpr double kDt = 0.01;  // MUST equal PlantPacer's dt (asserted below)

    Snapshot snap_;

private:
    void take() {
        snap_.taken = true;
        snap_.truth = rig_->h.truePose();
        for (const IMotor* m : rig_->h.context().driveMotors()) {
            snap_.drive.push_back({m->commandedVoltage().value(), m->brakeMode()});
        }
        for (const IMotor* m : mechMotors_) {
            snap_.mech.push_back({m->commandedVoltage().value(), m->brakeMode()});
        }
    }

    PlantPacer* inner_;
    MotionRig* rig_;
    double limit_;
    double elapsed_ = 0.0;
    std::vector<const IMotor*> mechMotors_;
};

/// The DoD stack: plant → PlantPacer → TallyPacer (independent script) →
/// RunGuard → Chassis. The guard's checks run before the tally counts, the
/// tally counts before... no: the guard checks, then the tally's inner pace
/// steps the plant, then the tally counts — so at snapshot time the world is
/// exactly at the tally's own elapsed instant.
struct DodRig {
    MatrixKinematics kin = xDrive(Length{7.0});
    FakeTelemetrySink sink;
    MotionRig rig;
    PlantPacer plant;
    TallyPacer tally;
    RunGuard guard;
    Chassis chassis;

    explicit DodRig(double limitSeconds, std::vector<const IMotor*> mechMotors = {})
        : rig{kin, motion_rig::plantConfig(), &sink},
          plant{rig.h},
          tally{plant, rig, limitSeconds, std::move(mechMotors)},
          guard{tally},
          chassis{rig.deps, guard, chassisConfig()} {
        // The tally's dt assumption must match the plant pacer's actual dt,
        // or the "independent" tally would be independently WRONG.
        REQUIRE(plant.tickDt.value() == TallyPacer::kDt);
        chassis.setPose(Pose2d{});
    }

    /// Assert the snapshot: at the INDEPENDENT limit, the robot is at the
    /// park fixture and every device is safe — voltage AND mode, per motor.
    void checkParkedAndSafeAtLimit(const Pose2d& park, BrakeMode mechDeclared) {
        REQUIRE(tally.snap_.taken);
        // Plant truth vs the fixture: moveTo settles inside ~0.75 in
        // (HA-52's tolerance) and the estimate/truth drift of a benign run
        // stays inside ~1 in — 2.0 in is a test bound over those two, not an
        // accuracy claim (the F2 accuracy row is untouched by this suite).
        CHECK(posErr(tally.snap_.truth, park) < 2.0);
        for (const auto& d : tally.snap_.drive) {
            CHECK(d.volts == 0.0);
            CHECK(d.mode == BrakeMode::Brake);
        }
        for (const auto& m : tally.snap_.mech) {
            CHECK(m.volts == 0.0);
            CHECK(m.mode == mechDeclared);
        }
    }
};

/// The DoD schedule (a test script, not a field claim — file banner).
constexpr double kLimit = 15.0;
inline RunGuardConfig dodSchedule(std::span<IMechanism* const> mechs = {}) {
    return RunGuardConfig{.endActionAt = Time{9.0}, .hardStopAt = Time{14.5},
                          .mechanisms = mechs};
}

}  // namespace

// ═══ THE DoD TEST, stalled four ways ═══════════════════════════════════════════════
//
// Bug caught (all four cases): the headline failure this chunk exists to
// convert — a stalled scoring loop running to the buzzer with the robot
// mid-field: 0 points from a run that behaved exactly as designed
// (measurement 4: motors safe through the overrun, and safe-but-unparked
// scores nothing). Each stall shape is a different way the naive design dies.

// Stall 1: a motion that never settles (measurement 1's shape) — a 300 in leg
// against a 60 s timeout. The guard must cut it at T+9 and park.
TEST_CASE("F2 DoD: a motion that never settles — parked and safe at the limit") {
    DodRig d{kLimit};
    const Pose2d park{Length{40.0}, Length{-20.0}, Angle::degrees(0.0)};
    ExitReason scoringVerdict = ExitReason::Running;

    const RunGuardReport rep = d.guard.run(
        d.chassis, dodSchedule(),
        [&] {
            Routine r{d.chassis, "dod-stall-motion"};
            r.moveTo(Pose2d{Length{300.0}, Length{0.0}, Angle{}},
                     {.timeout = Time{60.0}, .maxLinearSpeed =
                          shulib::units::Velocity{10.0}});
            scoringVerdict = r.result().exit;
        },
        [&] {
            Routine p{d.chassis, "dod-park"};
            p.moveTo(park, {.timeout = Time{4.0}}).brake({.timeout = Time{1.0}});
            return p.ok();
        });

    CHECK(rep.scoringCut);
    CHECK(scoringVerdict == ExitReason::Cancelled);  // the chain saw the cut honestly
    CHECK(rep.endActionSucceeded);
    d.tally.driveToLimit(d.guard);
    d.checkParkedAndSafeAtLimit(park, BrakeMode::Coast);
}

// Stall 2: a mechanism that never confirms — the intake spins for a ring that
// never comes, on a 60 s budget. The guard's wait must cut at T+9, cancel-all
// must render the operation inert THROUGH THE CLAIM, and the park must run.
TEST_CASE("F2 DoD: a mechanism that never confirms — parked, op inert, mech safe") {
    shulib::hal::fake::FakeMotor mechMotor;
    std::array<IMotor*, 1> motors{&mechMotor};
    MotorMechanism mech{motors, BrakeMode::Hold, "intake"};
    std::array<IMechanism*, 1> mechs{&mech};

    DodRig d{kLimit, {&mechMotor}};
    MechanismDeps mdeps{.clock = &d.rig.h.clock(), .faults = &d.rig.latch,
                        .telemetry = &d.sink};
    RunUntilConfirmedConfig mcfg{
        .voltage = Voltage{9.0},
        .timeout = Time{60.0},  // its own watchdog would fire far past the buzzer
        .stall = StallConfig{.currentAtLeast = Current{999.0},
                             .speedAtMost = AngularVelocity{0.0},
                             .persistence = Time{10.0}}};
    RunUntilConfirmed grab{mech, mdeps, mcfg, [] { return false; }, "grab"};
    const Pose2d park{Length{24.0}, Length{-24.0}, Angle::degrees(90.0)};
    GuardedWaitResult waitVerdict = GuardedWaitResult::Satisfied;

    const RunGuardReport rep = d.guard.run(
        d.chassis, dodSchedule(mechs),
        [&] {
            grab.start();
            waitVerdict = d.guard.waitFor(
                [&] { return grab.tick() != MechanismOutcome::Running; }, Time{60.0});
        },
        [&] {
            Routine p{d.chassis, "dod-park"};
            p.moveTo(park, {.timeout = Time{4.0}}).brake({.timeout = Time{1.0}});
            return p.ok();
        });

    CHECK(waitVerdict == GuardedWaitResult::RunExpired);  // not Satisfied (M6)
    CHECK(grab.outcome() == MechanismOutcome::Cancelled);
    CHECK(rep.endActionSucceeded);
    d.tally.driveToLimit(d.guard);
    d.checkParkedAndSafeAtLimit(park, BrakeMode::Hold);
    // The stay-safe half (measurement 13): tick the dead op once more, at the
    // very end — it must not re-energize the parked robot's mechanism.
    CHECK(grab.tick() == MechanismOutcome::Cancelled);
    CHECK(mechMotor.commandedVoltage().value() == 0.0);
    CHECK(mechMotor.brakeMode() == BrakeMode::Hold);
}

// Stall 3: a waitFor whose condition never comes true — "wait for the
// alliance partner's signal" and the partner never signals.
TEST_CASE("F2 DoD: a wait whose condition never holds — parked and safe at the limit") {
    DodRig d{kLimit};
    const Pose2d park{Length{24.0}, Length{-24.0}, Angle::degrees(90.0)};

    const RunGuardReport rep = d.guard.run(
        d.chassis, dodSchedule(),
        [&] {
            (void)d.guard.waitFor([] { return false; }, Time{60.0});
        },
        [&] {
            Routine p{d.chassis, "dod-park"};
            p.moveTo(park, {.timeout = Time{4.0}}).brake({.timeout = Time{1.0}});
            return p.ok();
        });

    CHECK(rep.scoringCut);
    CHECK(rep.endActionSucceeded);
    d.tally.driveToLimit(d.guard);
    d.checkParkedAndSafeAtLimit(park, BrakeMode::Coast);
}

// Stall 4: a fault cascade — every scoring attempt is fault-aborted (scripted
// ODO_STUCK raises; scripted BECAUSE a hostile-sensor cascade corrupts the
// estimate by design, which would make the plant-truth park assertion
// meaningless — the hostile suites own that pathology; the CASCADE mechanics
// are what this case pins) and a stubborn author retries until the budget
// expires. The retries after T+9 are refused; the park still runs.
TEST_CASE("F2 DoD: a fault-abort cascade with a stubborn retry loop — parked anyway") {
    DodRig d{kLimit};
    const Pose2d park{Length{30.0}, Length{-10.0}, Angle::degrees(0.0)};
    int attempts = 0;
    int abortedAttempts = 0;

    const RunGuardReport rep = d.guard.run(
        d.chassis, dodSchedule(),
        [&] {
            // Slow legs (5 in/s): ~80 aborted attempts drift the robot tens of
            // inches, not the full 300 — at the default 60 in/s the cascade
            // drives most of the field and the park honestly cannot return in
            // its budget (this test's first draft did exactly that).
            shulib::motion::MotionConfig legCfg = motionConfig();
            legCfg.maxLinearSpeed = shulib::units::Velocity{5.0};
            while (!d.guard.expired()) {
                ++attempts;
                MoveToPose leg{d.chassis.deps(),
                               Pose2d{Length{300.0}, Length{0.0}, Angle{}},
                               legCfg, /*timeout=*/5.0};
                d.chassis.scheduler().async(leg);
                int ticks = 0;
                (void)d.chassis.waitUntil(
                    [&] {
                        if (++ticks == 10 && d.chassis.scheduler().hasActiveMotion()) {
                            d.rig.latch.raise(FaultCode::OdoStuck, "TST",
                                              "scripted cascade");
                        }
                        return !d.chassis.scheduler().hasActiveMotion();
                    },
                    Time{2.0});
                if (d.chassis.scheduler().hasActiveMotion()) {
                    d.chassis.scheduler().cancel();  // never leave a stack motion armed
                }
                if (d.chassis.lastCompleted().abortFault == FaultCode::OdoStuck) {
                    ++abortedAttempts;
                }
            }
        },
        [&] {
            Routine p{d.chassis, "dod-park"};
            p.moveTo(park, {.timeout = Time{4.0}}).brake({.timeout = Time{1.0}});
            return p.ok();
        });

    CHECK(attempts > 10);          // the cascade really cascaded
    CHECK(abortedAttempts > 10);   // aborted by the C2 fault policy, not by luck
    CHECK(rep.scoringCut);
    CHECK(rep.endActionSucceeded);
    d.tally.driveToLimit(d.guard);
    d.checkParkedAndSafeAtLimit(park, BrakeMode::Coast);
}

// ═══ The measured-failure regressions that need the DoD stack ═════════════════════

// Bug caught: measurement 10's ordering regressing — the deadline check
// moving to the other side of the plant step buys one tick of travel per
// refusal. Asserted as plant-truth distance between the instant of the cut
// and the end of a 40-refusal storm: 0.0000 in with check-then-step.
TEST_CASE("F2 regression (M10): post-deadline travel is ZERO, pinned against truth") {
    DodRig d{kLimit};
    Pose2d truthAtCut{};
    Pose2d truthAfterStorm{};
    ExitReason cutVerdict = ExitReason::Running;
    // 10 in/s so the 300 in leg genuinely cannot arrive by T+9 (~90 in) — at
    // the default 60 in/s it SETTLES at ~5.3 s and no cut ever happens (this
    // test's own first draft measured 0.22 in of settle chatter between
    // legitimate motions and briefly blamed the guard for it).
    const shulib::chassis::MotionOptions slow{.timeout = Time{60.0},
                                              .maxLinearSpeed =
                                                  shulib::units::Velocity{10.0}};

    (void)d.guard.run(
        d.chassis, dodSchedule(),
        [&] {
            cutVerdict = d.chassis.moveTo(Pose2d{Length{300.0}, Length{0.0}, Angle{}},
                                          slow);
            truthAtCut = d.rig.h.truePose();
            for (int i = 0; i < 40; ++i) {
                (void)d.chassis.moveTo(Pose2d{Length{300.0}, Length{0.0}, Angle{}},
                                       slow);
            }
            truthAfterStorm = d.rig.h.truePose();
        },
        [] { return true; });

    CHECK(cutVerdict == ExitReason::Cancelled);  // the cut REALLY happened
    CHECK(posErr(truthAfterStorm, truthAtCut) == 0.0);  // exactly zero: the plant
    // is memoryless (kA = 0) and the cancel lands before the step — one tick
    // of drift here means the ordering flipped (measured 10.79 in for the
    // step-then-check design over a full overrun).
}

// Bug caught: measurement 2 regressing SILENTLY in either direction — the
// frozen scheduler-level wait is expected to pay its full own budget (the
// guard cannot cut it; 2801 invisible cancels were measured). If this test
// goes red because the lateness SHRANK, a legal cut for frozen waits exists
// and chapter 9's documented limit must be rewritten, not just re-greened.
TEST_CASE("F2 regression (M2): the frozen wait pays its full budget across the "
          "deadline — the documented hole, pinned") {
    DodRig d{kLimit};
    double waitReturnedAt = 0.0;

    (void)d.guard.run(
        d.chassis, dodSchedule(),
        [&] {
            // Starts at ~T+0 with a 12 s budget: crosses T+9 with 3 s of
            // remainder, all of which is paid.
            (void)d.chassis.waitUntil([] { return false; }, Time{12.0});
            waitReturnedAt = d.rig.h.clock().now().value();
        },
        [] { return true; });

    CHECK(waitReturnedAt >= 12.0);
    CHECK(waitReturnedAt <= 12.0 + 3.0 * TallyPacer::kDt);
}

// Bug caught: the guard's own account drifting from ground truth — the
// report must be honest about WHEN scoring ended (measurement 3's realistic
// lateness, quantified through the report the docs tell users to read).
TEST_CASE("F2 regression (M3): report.scoringEnded matches the frozen-pause "
          "lateness formula") {
    DodRig d{kLimit};

    const RunGuardReport rep = d.guard.run(
        d.chassis, dodSchedule(),
        [&] {
            Routine r{d.chassis, "late-pauses"};
            // pause(6 s) from ~T+8.2: crosses T+9 with ~5.2 s remainder.
            r.pause(Time{8.2})     // ends ~T+8.2 (before the deadline)
                .pause(Time{6.0})  // the crossing pause: pays until ~T+14.2
                .moveTo(Pose2d{Length{300.0}, Length{0.0}, Angle{}},
                        {.timeout = Time{60.0}});  // refused, chain stops
        },
        [] { return true; });

    // Formula: remainder of the crossing pause (14.2) + ≤ a few ticks for the
    // refused motion. NOT 9.0 (a cut pause would be a doc rewrite) and NOT
    // 14.2 + 60 (the motion must be refused, not run).
    CHECK(rep.scoringEnded.value() >= 14.2 - TallyPacer::kDt);
    CHECK(rep.scoringEnded.value() <= 14.2 + 5.0 * TallyPacer::kDt);
    CHECK(rep.scoringCut);
}
