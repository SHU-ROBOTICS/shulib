// F2 RUN GUARD SUITE — sequence/run_guard.hpp's mechanics, against the full
// C4 stack over the A2 plant. Every case names the bug it would catch.
//
// The trap, in its F2 form (six chunks bitten): if the test's notion of "time
// is up" is the same clock the guard reads, a guard that never fires and a
// test that never checks agree perfectly. The cases below therefore assert
// against PLANT GROUND TRUTH (rig.h.truePose(), the A2 truth integrator —
// independent of the estimate AND of the guard) and, where an instant
// matters, measure it in ticks/paces counted by the test itself. The
// end-to-end DoD suite (sequence_end_of_run_test.cpp) drives the deadline
// from a fully independent pace-tally script; this file covers the guard's
// unit mechanics.

#include "doctest.h"

#include <array>
#include <stdexcept>
#include <string>

#include "motion_test_rig.hpp"
#include "shulib/chassis/chassis.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/control/exit_group.hpp"
#include "shulib/core/check.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/hal/mechanism.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/manipulation/mechanism_op.hpp"
#include "shulib/manipulation/mechanism_outcome.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/sequence/run_guard.hpp"
#include "shulib/units/quantity.hpp"

using motion_rig::MotionRig;
using motion_rig::PlantPacer;
using motion_rig::chassisConfig;
using motion_rig::posErr;
using shulib::PreconditionError;
using shulib::chassis::Chassis;
using shulib::chassis::Routine;
using shulib::chassis::RoutineStopCause;
using shulib::control::ExitReason;
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
using shulib::motion::WaitResult;
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

/// The full guarded stack: plant → PlantPacer → RunGuard → Chassis. The
/// harness sink records every log line, so "never silent" is assertable.
struct GuardRig {
    MatrixKinematics kin = xDrive(Length{7.0});
    FakeTelemetrySink sink;
    MotionRig rig;
    PlantPacer plant;
    RunGuard guard;
    Chassis chassis;

    GuardRig()
        : rig{kin, motion_rig::plantConfig(), &sink},
          plant{rig.h},
          guard{plant},
          chassis{rig.deps, guard, chassisConfig()} {}

    [[nodiscard]] double now() { return rig.h.clock().now().value(); }

    [[nodiscard]] int seqLines() const {
        int n = 0;
        for (int i = 0; i < sink.size(); ++i) {
            if (sink.at(i).subsystem == "SEQ") {
                ++n;
            }
        }
        return n;
    }

    [[nodiscard]] bool seqLineContains(const std::string& needle) const {
        for (int i = 0; i < sink.size(); ++i) {
            if (sink.at(i).subsystem == "SEQ"
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
};

/// A guard-side mechanism: one fake motor behind a MotorMechanism whose
/// declared safe mode is Hold — chosen so the measured half-safe state
/// (`brake=Hold, V=9.0`) is expressible and a mode-only assertion would lie.
struct GuardMech {
    shulib::hal::fake::FakeMotor motor;
    std::array<IMotor*, 1> motors{&motor};
    MotorMechanism mech{motors, BrakeMode::Hold, "lift"};
    MechanismDeps deps;
    RunUntilConfirmedConfig cfg{
        .voltage = Voltage{9.0},
        .timeout = Time{60.0},  // its own watchdog would fire long after any test deadline
        .stall = StallConfig{.currentAtLeast = Current{999.0},
                             .speedAtMost = AngularVelocity{0.0},
                             .persistence = Time{10.0}}};

    explicit GuardMech(GuardRig& g)
        : deps{.clock = &g.rig.h.clock(), .faults = &g.rig.latch, .telemetry = &g.sink} {}
};

constexpr double kTick = 0.01;  // the PlantPacer dt every rig here uses

}  // namespace

// ── inert by default (the D3 §2.1 binding instruction) ──────────────────────────────

// Bug caught: a guard that is not inert until armed — D3 §2.1's exact warning
// ("a deadline that silently clamped every subsequent step's timeout would
// change the behaviour of existing routines — a breaking change wearing an
// additive costume"). Wiring the guard into the Chassis must change NOTHING
// until run() goes live: bit-identical clock and truth pose against an
// unguarded twin, zero log lines, zero refusals.
TEST_CASE("F2 inert-by-default: an unarmed guard is a bit-identical pass-through") {
    GuardRig guarded;
    // The unguarded twin: same stack, raw pacer.
    MatrixKinematics kin = xDrive(Length{7.0});
    MotionRig rig{kin};
    PlantPacer plant{rig.h};
    Chassis bare{rig.deps, plant, chassisConfig()};

    const Pose2d target{Length{20.0}, Length{-8.0}, Angle::degrees(45.0)};
    guarded.chassis.setPose(Pose2d{});
    bare.setPose(Pose2d{});
    CHECK(guarded.chassis.moveTo(target, {.timeout = Time{8.0}}) == ExitReason::Settled);
    CHECK(bare.moveTo(target, {.timeout = Time{8.0}}) == ExitReason::Settled);

    CHECK(guarded.now() == rig.h.clock().now().value());  // bit-identical time
    CHECK(guarded.rig.h.truePose().x().value() == rig.h.truePose().x().value());
    CHECK(guarded.rig.h.truePose().y().value() == rig.h.truePose().y().value());
    CHECK(guarded.seqLines() == 0);  // the guard said nothing, did nothing
}

// ── configuration is validated, loudly, before anything moves ───────────────────────

// Bug caught: a nonsense schedule (zero/negative/NaN instants, floor before
// the act, a null mechanism) surviving until mid-run, where the failure would
// be a mystery instead of a thrown call-site error.
TEST_CASE("F2 config: nonsense schedules are refused at the door") {
    GuardRig g;
    auto nop = [] {};
    auto act = [] { return true; };
    CHECK_THROWS_AS(g.guard.run(g.chassis, RunGuardConfig{.endActionAt = Time{0.0},
                                                          .hardStopAt = Time{1.0}},
                                nop, act),
                    PreconditionError);
    CHECK_THROWS_AS(g.guard.run(g.chassis, RunGuardConfig{.endActionAt = Time{2.0},
                                                          .hardStopAt = Time{1.0}},
                                nop, act),
                    PreconditionError);
    IMechanism* nullMech = nullptr;
    std::array<IMechanism*, 1> mechs{nullMech};
    CHECK_THROWS_AS(g.guard.run(g.chassis, RunGuardConfig{.endActionAt = Time{1.0},
                                                          .hardStopAt = Time{2.0},
                                                          .mechanisms = mechs},
                                nop, act),
                    PreconditionError);
    CHECK_FALSE(g.guard.running());  // a refused run never armed
}

// ── the cut, and verdict honesty (measurements 1 and 8) ─────────────────────────────

// Bug caught: the two halves of measurement 8's silent hijack. (a) The
// caller's cut verb must return its HONEST verdict — Cancelled, the motion
// was cancelled — never Settled describing an end action it did not issue.
// (b) The end action's verdict lives in the guard's report and its log lines
// — a run whose transcript is silent about the cut is the worst outcome
// measured in the campaign.
TEST_CASE("F2 verdict honesty: the cut verb reports Cancelled; the end action "
          "reports through the guard; nothing is silent") {
    GuardRig g;
    g.chassis.setPose(Pose2d{});
    ExitReason scoringVerdict = ExitReason::Running;
    const Pose2d park{Length{10.0}, Length{-15.0}, Angle::degrees(0.0)};

    const RunGuardReport rep = g.guard.run(
        g.chassis,
        RunGuardConfig{.endActionAt = Time{2.0}, .hardStopAt = Time{8.0}},
        [&] {
            // A 300 in leg that cannot finish inside the budget: the guard
            // must cut it AT the deadline.
            scoringVerdict = g.chassis.moveTo(
                Pose2d{Length{300.0}, Length{0.0}, Angle{}}, {.timeout = Time{60.0}});
        },
        [&] {
            return g.chassis.moveTo(park, {.timeout = Time{4.0}});
        });

    CHECK(scoringVerdict == ExitReason::Cancelled);  // (a) the honest verdict
    CHECK(rep.scoringCut);
    // The cut landed AT the deadline (zero-latency unwind, measurement 1):
    // scoring returned within a tick of T+2.00.
    CHECK(rep.scoringEnded.value() >= 2.0);
    CHECK(rep.scoringEnded.value() <= 2.0 + 2.0 * kTick);
    CHECK(rep.endActionRan);
    CHECK(rep.endActionSucceeded);                   // (b) reported HERE, not in (a)
    CHECK(posErr(g.rig.h.truePose(), park) < 2.0);   // plant truth: it parked
    CHECK(rep.pacesSeen > 0);
    // Not silent: armed + expired + act-start + verdict, all on the transcript.
    CHECK(g.seqLineContains("armed"));
    CHECK(g.seqLineContains("expired"));
    CHECK(g.seqLineContains("end-of-run action"));
    g.checkDriveSafe();
}

// ── the latch (T7; measurements 9, 10, 11) ──────────────────────────────────────────

// Bug caught: measurement 9's naive design — cancel-only expiry, which a
// retrying caller defeats (80.55 in of post-deadline travel measured, target
// REACHED). With the latch every retry is refused before the world advances:
// zero post-deadline travel, counted refusals, and the chain of retries burns
// paces, not inches.
TEST_CASE("F2 latch: a retrying caller is refused — zero post-deadline travel") {
    GuardRig g;
    g.chassis.setPose(Pose2d{});
    Pose2d truthAtCut{};
    Pose2d truthAfterRetries{};
    const Pose2d far{Length{300.0}, Length{0.0}, Angle{}};

    const RunGuardReport rep = g.guard.run(
        g.chassis,
        RunGuardConfig{.endActionAt = Time{2.0}, .hardStopAt = Time{10.0}},
        [&] {
            (void)g.chassis.moveTo(far, {.timeout = Time{60.0}});  // the cut
            truthAtCut = g.rig.h.truePose();
            for (int i = 0; i < 30; ++i) {
                // measurement 9's retry storm, now against the latch
                (void)g.chassis.moveTo(far, {.timeout = Time{60.0}});
            }
            truthAfterRetries = g.rig.h.truePose();
        },
        [] { return true; });

    // The ordering pin (measurement 10): the deadline check runs BEFORE the
    // plant step, so 31 cut/refused motions moved the robot 0.0000 in — the
    // step-then-check mutation turns this into one tick of travel per refusal.
    CHECK(posErr(truthAfterRetries, truthAtCut) < 0.01);
    CHECK(rep.postExpiryCancels == 31);  // 1 cut + 30 refusals, all counted
    CHECK(g.seqLineContains("refused"));
    CHECK(rep.scoringCut);
}

// Bug caught: measurement 11 — the end action's own motion being cut after
// one tick like any post-deadline motion. The exemption window must let it
// run to its own verdict while scoring stays latched off.
TEST_CASE("F2 latch: the end action is exempt — its motion RUNS while scoring "
          "stays refused") {
    GuardRig g;
    g.chassis.setPose(Pose2d{});
    const Pose2d park{Length{12.0}, Length{9.0}, Angle::degrees(90.0)};
    ExitReason endVerdict = ExitReason::Running;

    (void)g.guard.run(
        g.chassis, RunGuardConfig{.endActionAt = Time{1.0}, .hardStopAt = Time{10.0}},
        [&] {
            (void)g.chassis.moveTo(Pose2d{Length{300.0}, Length{0.0}, Angle{}},
                                   {.timeout = Time{60.0}});
        },
        [&] {
            endVerdict = g.chassis.moveTo(park, {.timeout = Time{5.0}});
            return endVerdict;
        });

    CHECK(endVerdict == ExitReason::Settled);       // ran to ITS verdict, not 1 tick
    CHECK(posErr(g.rig.h.truePose(), park) < 2.0);  // and physically got there
}

// Bug caught: a guard that stays latched after run() returns — post-run code
// (a test rerun, driver control) would find every verb refused forever.
TEST_CASE("F2 latch: run() ending disarms the guard — the robot is handed back") {
    GuardRig g;
    g.chassis.setPose(Pose2d{});
    (void)g.guard.run(
        g.chassis, RunGuardConfig{.endActionAt = Time{1.0}, .hardStopAt = Time{2.0}},
        [&] {
            (void)g.chassis.moveTo(Pose2d{Length{300.0}, Length{0.0}, Angle{}},
                                   {.timeout = Time{60.0}});
        },
        [] {});
    CHECK_FALSE(g.guard.running());
    const Pose2d after{Length{5.0}, Length{5.0}, Angle{}};
    CHECK(g.chassis.moveTo(after, {.timeout = Time{8.0}}) == ExitReason::Settled);
    CHECK(posErr(g.rig.h.truePose(), after) < 2.0);
}

// ── the hard floor (T2) ─────────────────────────────────────────────────────────────

// Bug caught: a floor conditional on the end action having finished — the
// listed mutation. hardStopAt is UNCONDITIONAL: it fires mid-end-action,
// cuts the end action's motion, and safes every device.
TEST_CASE("F2 floor: fires DURING a still-running end action and safes everything") {
    GuardRig g;
    GuardMech m{g};
    std::array<IMechanism*, 1> mechs{&m.mech};
    g.chassis.setPose(Pose2d{});
    ExitReason endVerdict = ExitReason::Running;

    const RunGuardReport rep = g.guard.run(
        g.chassis,
        RunGuardConfig{.endActionAt = Time{1.0}, .hardStopAt = Time{2.5},
                       .mechanisms = mechs},
        [] { /* scoring finishes instantly — all the time belongs to the act */ },
        [&] {
            // An end action that would outlive the floor: a 300 in "park".
            endVerdict = g.chassis.moveTo(Pose2d{Length{300.0}, Length{0.0}, Angle{}},
                                          {.timeout = Time{60.0}});
            return endVerdict;
        });

    CHECK(rep.floorFired);
    CHECK(endVerdict == ExitReason::Cancelled);  // the floor cut it — honestly
    CHECK_FALSE(rep.endActionSucceeded);
    CHECK(g.seqLineContains("hard stop"));
    g.checkDriveSafe();
    CHECK(m.motor.commandedVoltage().value() == 0.0);  // mechanism too
    CHECK(m.motor.brakeMode() == BrakeMode::Hold);     // ...in its DECLARED state
}

// Bug caught: measurements 12/13/14 in the floor's mechanism half — an
// operation still being ticked across the floor re-commanding its voltage
// after applySafeState (the half-safe `brake=Hold, V=9.0`). The floor must
// cancel the operation THROUGH THE CLAIM (inert, not repainted), and a
// further tick must not re-energize.
TEST_CASE("F2 floor: a live operation is cancelled through the claim and STAYS safe") {
    GuardRig g;
    GuardMech m{g};
    std::array<IMechanism*, 1> mechs{&m.mech};
    g.chassis.setPose(Pose2d{});
    bool confirmed = false;
    RunUntilConfirmed op{m.mech, m.deps, m.cfg, [&] { return confirmed; }, "grab"};

    (void)g.guard.run(
        g.chassis,
        RunGuardConfig{.endActionAt = Time{1.0}, .hardStopAt = Time{2.0},
                       .mechanisms = mechs},
        [] {},
        [&] {
            // The end action keeps ticking a never-confirming op until the
            // floor: the guard must end the seesaw by making the op INERT.
            op.start();
            (void)g.guard.waitFor([&] { return op.tick() != MechanismOutcome::Running; },
                                  Time{60.0});
            return op.outcome();
        });

    CHECK(op.outcome() == MechanismOutcome::Cancelled);
    CHECK(m.motor.commandedVoltage().value() == 0.0);
    CHECK(m.motor.brakeMode() == BrakeMode::Hold);
    // Ticked AGAIN after the run: an inert op commands nothing (measurement
    // 13's regression — applySafeState alone survives exactly 2 device events).
    CHECK(op.tick() == MechanismOutcome::Cancelled);
    CHECK(m.motor.commandedVoltage().value() == 0.0);
    CHECK(m.motor.brakeMode() == BrakeMode::Hold);
    CHECK_FALSE(m.mech.claimed());
}

// ── cancel-all strictly precedes the act (the claim landmine) ───────────────────────

// Bug caught: the listed mutation "the end action starts before the scoring
// loop is latched off" — concretely: a stalled operation's unreleased claim
// makes the end action's own operation THROW at start() (mechanism_op.hpp's
// precondition). cancel-all must have released everything first.
TEST_CASE("F2 cancel-all: a stalled claim is released BEFORE the end action needs it") {
    GuardRig g;
    GuardMech m{g};
    std::array<IMechanism*, 1> mechs{&m.mech};
    g.chassis.setPose(Pose2d{});
    bool confirmed = false;
    RunUntilConfirmed stalled{m.mech, m.deps, m.cfg, [&] { return confirmed; }, "grab"};
    MechanismOutcome endOpOutcome = MechanismOutcome::Running;

    const RunGuardReport rep = g.guard.run(
        g.chassis,
        RunGuardConfig{.endActionAt = Time{1.0}, .hardStopAt = Time{8.0},
                       .mechanisms = mechs},
        [&] {
            stalled.start();  // claims the mechanism...
            (void)g.guard.waitFor(
                [&] { return stalled.tick() != MechanismOutcome::Running; }, Time{60.0});
            // ...and is still Running (claim HELD) when the deadline cuts the wait
        },
        [&] {
            // The end action's op on the SAME mechanism: before the F2
            // cancel-all ordering this start() threw.
            bool done = true;
            RunUntilConfirmed verify{m.mech, m.deps, m.cfg, [&] { return done; },
                                     "park-verify"};
            verify.start();
            endOpOutcome = verify.tick();
            return endOpOutcome;
        });

    CHECK(stalled.outcome() == MechanismOutcome::Cancelled);  // cancel-all got it
    CHECK(endOpOutcome == MechanismOutcome::Succeeded);       // the act could claim
    CHECK(rep.endActionSucceeded);
}

// Bug caught: an ANONYMOUS claim (bare tryClaim — F1 code, third-party ops)
// wedging the end action forever. The guard cannot cancel what it cannot
// see; it must force-release, Warn, and count — not throw, not stay stuck.
TEST_CASE("F2 cancel-all: an anonymous claim is force-released with a Warn") {
    GuardRig g;
    GuardMech m{g};
    std::array<IMechanism*, 1> mechs{&m.mech};
    g.chassis.setPose(Pose2d{});
    REQUIRE(m.mech.tryClaim());  // an operation the guard cannot reach

    const RunGuardReport rep = g.guard.run(
        g.chassis,
        RunGuardConfig{.endActionAt = Time{1.0}, .hardStopAt = Time{8.0},
                       .mechanisms = mechs},
        [] {}, [] { return true; });

    CHECK(rep.anonymousClaimsReleased == 1);
    CHECK_FALSE(m.mech.claimed());
    CHECK(g.seqLineContains("anonymous claim"));
}

// ── the deadline-aware waits (T4, T5; measurements 2, 5, 6) ─────────────────────────

// Bug caught: the verdict trap of measurement 6 — a deadline folded into a
// predicate returning Satisfied, which a chain maps to success and keeps
// scoring. The guard's wait has its own chain-halting verdict, and RunExpired
// WINS a tie with Satisfied.
TEST_CASE("F2 waitFor: the three verdicts, and RunExpired wins the tie") {
    GuardRig g;
    g.chassis.setPose(Pose2d{});
    (void)g.guard.run(
        g.chassis, RunGuardConfig{.endActionAt = Time{4.0}, .hardStopAt = Time{20.0}},
        [&] {
            // (a) Satisfied: condition arrives well inside every budget.
            const double t0 = g.now();
            CHECK(g.guard.waitFor([&] { return g.now() >= t0 + 0.5; }, Time{2.0})
                  == GuardedWaitResult::Satisfied);
            // (b) TimedOut: the WAIT's own budget ends first — run still live.
            CHECK(g.guard.waitFor([] { return false; }, Time{0.5})
                  == GuardedWaitResult::TimedOut);
            CHECK_FALSE(g.guard.expired());
            // (c) the tie: a predicate that becomes true EXACTLY at the run
            // deadline (T+4.0). Satisfied would keep the chain scoring past
            // the buzzer — RunExpired must win.
            CHECK(g.guard.waitFor([&] { return g.now() >= 4.0; }, Time{60.0})
                  == GuardedWaitResult::RunExpired);
            // (d) post-expiry waits return instantly, without calling pred.
            int predCalls = 0;
            const double before = g.now();
            CHECK(g.guard.waitFor([&] { ++predCalls; return true; }, Time{60.0})
                  == GuardedWaitResult::RunExpired);
            CHECK(predCalls == 0);       // the latch applies to waits too
            CHECK(g.now() == before);    // zero ticks, zero time
        },
        [] { return true; });
}

// Bug caught: measurement 5's shape done WRONG — a deadline-aware wait whose
// cut arrives late. The RunExpired return must land within a tick of the
// deadline (0.0000 s measured for the composite-predicate design).
TEST_CASE("F2 waitFor: the cut lands AT the deadline, not after") {
    GuardRig g;
    g.chassis.setPose(Pose2d{});
    (void)g.guard.run(
        g.chassis, RunGuardConfig{.endActionAt = Time{2.0}, .hardStopAt = Time{20.0}},
        [&] {
            CHECK(g.guard.waitFor([] { return false; }, Time{60.0})
                  == GuardedWaitResult::RunExpired);
            CHECK(g.now() >= 2.0);
            CHECK(g.now() <= 2.0 + 2.0 * kTick);  // zero-latency, ± one pace
        },
        [] { return true; });
}

// Bug caught: pause() losing either half of its contract — a full sleep must
// read Satisfied (pause is not a failure), a cut sleep must read RunExpired
// (and end AT the deadline, not after its full duration — the whole point).
TEST_CASE("F2 pause: full sleep is Satisfied; the deadline cuts it short") {
    GuardRig g;
    g.chassis.setPose(Pose2d{});
    (void)g.guard.run(
        g.chassis, RunGuardConfig{.endActionAt = Time{2.0}, .hardStopAt = Time{20.0}},
        [&] {
            const double t0 = g.now();
            CHECK(g.guard.pause(Time{0.5}) == GuardedWaitResult::Satisfied);
            CHECK(g.now() >= t0 + 0.5);
            CHECK(g.guard.pause(Time{60.0}) == GuardedWaitResult::RunExpired);
            CHECK(g.now() <= 2.0 + 2.0 * kTick);  // cut at T+2, not at T+60.5
        },
        [] { return true; });
}

// Bug caught: the guard's verdict failing to HALT a Tier-2 chain — the
// measurement 6 trap at the recipe layer. The documented idiom (`== Satisfied`
// through then()'s bool arm) must stop the chain on RunExpired and skip the
// rest.
TEST_CASE("F2 waitFor: the then() idiom halts a Routine chain at the deadline") {
    GuardRig g;
    g.chassis.setPose(Pose2d{});
    int lateSteps = 0;
    (void)g.guard.run(
        g.chassis, RunGuardConfig{.endActionAt = Time{1.0}, .hardStopAt = Time{20.0}},
        [&] {
            Routine r{g.chassis, "guarded"};
            r.then([&] { return g.guard.waitFor([] { return false; }, Time{60.0})
                             == GuardedWaitResult::Satisfied; },
                   "wait-ring")
                .then([&] { ++lateSteps; }, "score-late");
            CHECK_FALSE(r.ok());
            CHECK(r.result().cause == RoutineStopCause::ActionFailed);
            CHECK(r.result().skipped == 1);
        },
        [] { return true; });
    CHECK(lateSteps == 0);  // nothing scored past the buzzer
}

// ── the frozen-surface lateness bounds (T4's documented limit; measurements 2, 3) ───

// Bug caught: someone "fixing" the frozen waits silently — or the documented
// formula rotting. Chassis::waitUntil (frozen, F6) CANNOT be cut: it runs to
// its own budget, and the lateness equals the unexpired remainder. This test
// PINS the limit so the documentation stays true; if a legal way to cut the
// frozen waits ever lands, this goes red and the docs get rewritten.
TEST_CASE("F2 frozen-wait limit: chassis.waitUntil pays its full remainder "
          "(the documented formula)") {
    GuardRig g;
    g.chassis.setPose(Pose2d{});
    (void)g.guard.run(
        g.chassis, RunGuardConfig{.endActionAt = Time{1.0}, .hardStopAt = Time{20.0}},
        [&] {
            // Starts at ~T+0; deadline T+1; its own budget 3 s. The deadline
            // fires mid-wait with 2 s of remainder — ALL of it is paid.
            const double t0 = g.now();
            CHECK(g.chassis.waitUntil([] { return false; }, Time{3.0})
                  == WaitResult::TimedOut);
            CHECK(g.now() >= t0 + 3.0);              // the full budget ran
            CHECK(g.now() <= t0 + 3.0 + 2.0 * kTick);
        },
        [] { return true; });
}

// Bug caught: the measurement-3 shape regressing to WORSE than the formula —
// a Routine pause crossing the deadline pays its remainder, and then the
// chain's FIRST post-deadline motion is refused, which stops the chain (so a
// Routine pays ONE wait's remainder, not the sum of all later budgets).
TEST_CASE("F2 frozen-wait limit: a Routine pays one pause remainder, then the "
          "latch stops the chain") {
    GuardRig g;
    g.chassis.setPose(Pose2d{});
    RunGuardReport rep{};
    int chainSkipped = 0;
    rep = g.guard.run(
        g.chassis, RunGuardConfig{.endActionAt = Time{1.0}, .hardStopAt = Time{20.0}},
        [&] {
            Routine r{g.chassis, "frozen-chain"};
            r.pause(Time{3.0})                        // crosses T+1; pays until T+3
                .moveTo(Pose2d{Length{300.0}, Length{0.0}, Angle{}},
                        {.timeout = Time{60.0}})      // refused: stops the chain
                .pause(Time{3.0})                     // skipped — never paid
                .pause(Time{3.0});                    // skipped — never paid
            chainSkipped = r.result().skipped;
        },
        [] { return true; });

    CHECK(chainSkipped == 2);
    CHECK(rep.scoringCut);
    // One remainder (2 s past the deadline) + the refused motion (≤ a tick),
    // NOT 1 + 2 + 3·3 s: scoring ended by ~T+3, far before the pauses' sum.
    CHECK(rep.scoringEnded.value() >= 3.0 - kTick);
    CHECK(rep.scoringEnded.value() <= 3.0 + 3.0 * kTick);
    CHECK(rep.postExpiryCancels >= 1);
}

// ── run() edges ─────────────────────────────────────────────────────────────────────

// Bug caught: scoring finishing early being made to WAIT for the deadline —
// inventing a policy ("the library decides when you park") the design
// explicitly refuses. Early return ⇒ the end action runs immediately.
TEST_CASE("F2 run: scoring returning early runs the end action immediately") {
    GuardRig g;
    g.chassis.setPose(Pose2d{});
    const RunGuardReport rep = g.guard.run(
        g.chassis, RunGuardConfig{.endActionAt = Time{30.0}, .hardStopAt = Time{40.0}},
        [&] {
            (void)g.chassis.moveTo(Pose2d{Length{10.0}, Length{0.0}, Angle{}},
                                   {.timeout = Time{8.0}});
        },
        [] { return true; });
    CHECK_FALSE(rep.scoringCut);              // it was never cut
    CHECK(rep.endActionRan);
    CHECK(rep.endActionEnded.value() < 30.0);  // and did not wait for T+30
}

// Bug caught: a throw out of scoring being converted into a park (hiding a
// programming error) or, worse, leaving devices live (measurement 15's
// energy state). The guard cancels-all on the unwind and RETHROWS.
TEST_CASE("F2 run: a throwing scoring loop safes everything and stays loud") {
    GuardRig g;
    GuardMech m{g};
    std::array<IMechanism*, 1> mechs{&m.mech};
    g.chassis.setPose(Pose2d{});
    bool confirmed = false;
    RunUntilConfirmed op{m.mech, m.deps, m.cfg, [&] { return confirmed; }, "grab"};
    bool endActionRan = false;

    CHECK_THROWS_AS(
        g.guard.run(g.chassis,
                    RunGuardConfig{.endActionAt = Time{10.0}, .hardStopAt = Time{20.0},
                                   .mechanisms = mechs},
                    [&] {
                        op.start();
                        CHECK(op.tick() == MechanismOutcome::Running);  // energized
                        throw std::runtime_error("author bug");
                    },
                    [&] {
                        endActionRan = true;
                        return true;
                    }),
        std::runtime_error);

    CHECK_FALSE(endActionRan);  // a broken program gets no park driven for it
    CHECK_FALSE(g.guard.running());
    g.checkDriveSafe();
    CHECK(m.motor.commandedVoltage().value() == 0.0);
    CHECK(m.motor.brakeMode() == BrakeMode::Hold);
    CHECK(op.outcome() == MechanismOutcome::Cancelled);
}

// Bug caught: run() from inside run() (or a wait outside one) proceeding into
// nonsense state instead of failing loudly at the call site.
TEST_CASE("F2 run: re-entry and out-of-run waits are loud preconditions") {
    GuardRig g;
    g.chassis.setPose(Pose2d{});
    CHECK_THROWS_AS((void)g.guard.waitFor([] { return true; }, Time{1.0}),
                    PreconditionError);
    CHECK_THROWS_AS((void)g.guard.pause(Time{1.0}), PreconditionError);
    CHECK_THROWS_AS((void)g.guard.expired(), PreconditionError);
    bool threw = false;
    (void)g.guard.run(
        g.chassis, RunGuardConfig{.endActionAt = Time{5.0}, .hardStopAt = Time{10.0}},
        [&] {
            try {
                (void)g.guard.run(g.chassis,
                                  RunGuardConfig{.endActionAt = Time{1.0},
                                                 .hardStopAt = Time{2.0}},
                                  [] {}, [] {});
            } catch (const PreconditionError&) {
                threw = true;
            }
        },
        [] { return true; });
    CHECK(threw);
}

// Bug caught: the mis-wiring the guard cannot prevent — a Chassis built with
// the RAW pacer while the guard "guards" nothing. The report must expose
// pacesSeen == 0 and the transcript must Warn that the guarantee never
// applied, because a silently-void guarantee is measurement 8's silence in a
// different coat.
TEST_CASE("F2 run: a guard that was never the pacer says so, loudly") {
    MatrixKinematics kin = xDrive(Length{7.0});
    FakeTelemetrySink sink;
    MotionRig rig{kin, motion_rig::plantConfig(), &sink};
    PlantPacer plant{rig.h};
    RunGuard guard{plant};
    Chassis chassis{rig.deps, plant, chassisConfig()};  // WRONG: raw pacer
    chassis.setPose(Pose2d{});

    const RunGuardReport rep = guard.run(
        chassis, RunGuardConfig{.endActionAt = Time{30.0}, .hardStopAt = Time{40.0}},
        [&] {
            (void)chassis.moveTo(Pose2d{Length{10.0}, Length{0.0}, Angle{}},
                                 {.timeout = Time{8.0}});
        },
        [] { return true; });

    CHECK(rep.pacesSeen == 0);
    bool warned = false;
    for (int i = 0; i < sink.size(); ++i) {
        if (sink.at(i).subsystem == "SEQ"
            && sink.at(i).message.find("never ran") != std::string::npos) {
            warned = true;
        }
    }
    CHECK(warned);
}

// Bug caught: the end-action verdict conventions drifting from then()'s —
// void must mean "performed", a false bool / non-Succeeded outcome must land
// as failure in the report AND on the transcript (never silently).
TEST_CASE("F2 end action: the four return conventions are honored") {
    GuardRig g;
    g.chassis.setPose(Pose2d{});
    SUBCASE("void = performed") {
        const RunGuardReport rep = g.guard.run(
            g.chassis, RunGuardConfig{.endActionAt = Time{1.0}, .hardStopAt = Time{2.0}},
            [] {}, [] {});
        CHECK(rep.endActionSucceeded);
        CHECK(g.seqLineContains("succeeded"));
    }
    SUBCASE("bool false = failed, and the transcript says so") {
        const RunGuardReport rep = g.guard.run(
            g.chassis, RunGuardConfig{.endActionAt = Time{1.0}, .hardStopAt = Time{2.0}},
            [] {}, [] { return false; });
        CHECK_FALSE(rep.endActionSucceeded);
        CHECK(g.seqLineContains("FAILED"));
    }
    SUBCASE("MechanismOutcome: only Succeeded succeeds") {
        const RunGuardReport rep = g.guard.run(
            g.chassis, RunGuardConfig{.endActionAt = Time{1.0}, .hardStopAt = Time{2.0}},
            [] {}, [] { return MechanismOutcome::Unconfirmed; });
        CHECK_FALSE(rep.endActionSucceeded);
    }
}

// Bug caught (MUTATION M10 — found GREEN in this chunk's first campaign run,
// closed by this test): the composite wait predicate re-ordered pred-first
// (`pred() || expiredNow()`). The started-after-expiry case is masked by the
// pre-check, so the original "pred is not called post-expiry" assertion never
// saw the difference — but a wait IN FLIGHT crossing the deadline calls the
// scoring predicate one more time at a post-deadline clock, and a predicate
// that ticks a mechanism op commands one more 9 V device event past the
// buzzer. The latch must hold MID-FLIGHT: no predicate call may ever observe
// a post-deadline clock.
TEST_CASE("F2 waitFor: a predicate in flight is never called at or after the deadline") {
    GuardRig g;
    g.chassis.setPose(Pose2d{});
    (void)g.guard.run(
        g.chassis, RunGuardConfig{.endActionAt = Time{1.0}, .hardStopAt = Time{20.0}},
        [&] {
            double lastPredCallAt = -1.0;
            CHECK(g.guard.waitFor(
                      [&] {
                          lastPredCallAt = g.now();
                          return false;
                      },
                      Time{60.0})
                  == GuardedWaitResult::RunExpired);
            CHECK(lastPredCallAt >= 0.0);  // it WAS being polled before expiry...
            CHECK(lastPredCallAt < 1.0);   // ...and never once at/after the deadline
        },
        [] { return true; });
}

// Bug caught: THE FLOOR SILENTLY NOT FIRING. Found by an independent reviewer
// probe after the chunk's own 14-mutation campaign came back all-red, which is
// the point of running one. C2's waitUntil evaluates `pred` BEFORE pace(), so
// an end action that ticks an operation through guard.waitFor lets the guard's
// own predicate observe the floor and return — correctly ending the wait —
// while pace() never runs fireFloor(). Measured before the fix: the mechanism
// sat at 9.000 V past the floor, report.floorFired read FALSE, and the "all
// devices safed unconditionally" Warn was never logged.
//
// The devices were still safed when run() returned, so this was never a
// runaway. It was worse in a subtler way: a run where the hard floor MATTERED
// was indistinguishable from one where it never came up — no flag, no line, no
// trace. That is exactly the observability failure E1 spent a chunk closing
// (DebugRecord::fault had no producer for seventeen chunks while TermSink
// rendered it).
//
// Without the fix in expiredNow(), the first two CHECKs below fail alone.
TEST_CASE("F2 floor: fires when a WAIT reaches it first, not only when pace() does") {
    GuardRig g;
    GuardMech m{g};
    std::array<IMechanism*, 1> mechs{&m.mech};
    g.chassis.setPose(Pose2d{});

    bool floorSeenInsideEndAction = false;

    const RunGuardReport rep = g.guard.run(
        g.chassis,
        RunGuardConfig{.endActionAt = Time{0.5}, .hardStopAt = Time{1.5},
                       .mechanisms = mechs},
        [] { /* scoring ends at once — the end action owns the rest */ },
        [&] {
            // The end action ticks an operation through the guard's OWN wait.
            // Nothing here ever becomes true, so the wait can only end at the
            // floor — and it ends via the PREDICATE, before any pace() sees it.
            const GuardedWaitResult w =
                g.guard.waitFor([&] { return false; }, Time{30.0});
            floorSeenInsideEndAction = (w == GuardedWaitResult::RunExpired);
            return true;
        });

    // The floor must have FIRED, not merely been observed.
    CHECK(rep.floorFired);
    CHECK(g.seqLineContains("hard stop"));

    CHECK(floorSeenInsideEndAction);
    g.checkDriveSafe();
    CHECK(m.motor.commandedVoltage().value() == 0.0);
    CHECK(m.motor.brakeMode() == BrakeMode::Hold);
}
