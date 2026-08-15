// F1 OPERATION CONTRACT — RunUntilConfirmed / ActuateAndConfirm against the
// contract in manipulation/mechanism_op.hpp. Every case names the bug it would
// catch.
//
// THE TRAP, and its counter (the rule that has bitten five chunks, in its F1
// form): the fake and the operation must not share a notion of "done". Every
// timeline below is HAND-WRITTEN FROM THE CONTRACT before it ran — tick N
// commanded, tick N+k confirmed, verdict at tick N+k — and asserted against
// those integer literals, never against any fake's opinion of completion. The
// cadence convention (mechanism_test_rig.hpp): tick i happens at t = i*dt.

#include "doctest.h"

#include <array>
#include <cmath>
#include <type_traits>

#include "mechanism_test_rig.hpp"
#include "shulib/core/check.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/hal/fake/fake_digital_out.hpp"
#include "shulib/hal/fake/fake_motor.hpp"
#include "shulib/hal/mechanism.hpp"
#include "shulib/manipulation/mechanism_op.hpp"
#include "shulib/motion/motion.hpp"

using mech_rig::DriveResult;
using mech_rig::OpRig;
using mech_rig::RecordingMotor;
using mech_rig::countMechWarns;
using mech_rig::drive;
using shulib::PreconditionError;
using shulib::diag::FaultCode;
using shulib::hal::BrakeMode;
using shulib::hal::IDigitalOut;
using shulib::hal::IMotor;
using shulib::hal::MotorMechanism;
using shulib::hal::PneumaticMechanism;
using shulib::hal::fake::FakeDigitalOut;
using shulib::hal::fake::FakeMotor;
using shulib::manipulation::ActuateAndConfirm;
using shulib::manipulation::ActuateAndConfirmConfig;
using shulib::manipulation::AlwaysConfirmed;
using shulib::manipulation::IMechanismOp;
using shulib::manipulation::MechanismDeps;
using shulib::manipulation::MechanismOutcome;
using shulib::manipulation::RunUntilConfirmed;
using shulib::manipulation::RunUntilConfirmedConfig;
using shulib::manipulation::StallConfig;
using shulib::units::AngularVelocity;
using shulib::units::Current;
using shulib::units::Time;
using shulib::units::Voltage;

namespace {

// One motor-mechanism op rig: clock/latch/sink + two FakeMotors as an intake.
struct MotorOpRig {
    OpRig r;
    FakeMotor a;
    FakeMotor b;
    std::array<IMotor*, 2> motors{&a, &b};
    MotorMechanism mech{motors, BrakeMode::Coast, "intake"};

    RunUntilConfirmedConfig cfg{
        .voltage = Voltage{6.0},
        .timeout = Time{0.5},
        .stall = StallConfig{.currentAtLeast = Current{2.0},
                             .speedAtMost = AngularVelocity{0.1},
                             .persistence = Time{0.05}}};
};

}  // namespace

// ── structural pins ─────────────────────────────────────────────────────────────────

// Bug caught: a mechanism operation deriving from IMotion — it would be
// passable to MotionScheduler::async(), where it would PRE-EMPT AND CANCEL the
// active DRIVE motion. The hierarchies are separate on purpose; this pin makes
// the separation a build fact, not a convention.
static_assert(!std::is_base_of_v<shulib::motion::IMotion, IMechanismOp>,
              "IMechanismOp must not be an IMotion (async(op) must not compile)");

// Bug caught: MechanismOutcome growing a bool conversion — `if (outcome)`
// would make Unconfirmed(2) truthy, the exact silent success T2 exists to
// prevent. Scoped enum, no conversion: pinned.
static_assert(!std::is_convertible_v<MechanismOutcome, bool>,
              "MechanismOutcome must never convert to bool");

// Bug caught: reordered/renumbered outcome values — log lines and any future
// wire use re-labelled silently. Explicit values, append-only, like FaultCode.
TEST_CASE("MechanismOutcome: values and spellings are stable") {
    CHECK(static_cast<int>(MechanismOutcome::Running) == 0);
    CHECK(static_cast<int>(MechanismOutcome::Succeeded) == 1);
    CHECK(static_cast<int>(MechanismOutcome::Unconfirmed) == 2);
    CHECK(static_cast<int>(MechanismOutcome::TimedOut) == 3);
    CHECK(static_cast<int>(MechanismOutcome::Cancelled) == 4);
    CHECK(static_cast<int>(MechanismOutcome::Stalled) == 5);
    CHECK(std::string_view{mechanismOutcomeName(MechanismOutcome::Unconfirmed)} ==
          "UNCONFIRMED");
    CHECK(std::string_view{mechanismOutcomeName(MechanismOutcome::Stalled)} == "STALLED");
    CHECK(std::string_view{mechanismOutcomeName(static_cast<MechanismOutcome>(99))} ==
          "UNKNOWN");
}

// ── RunUntilConfirmed ───────────────────────────────────────────────────────────────

// THE DoD CASE. Bug caught: an operation that cannot be driven through the
// seam to a confirmed completion, a confirm evaluated from the fake's opinion
// rather than the world's (the pred here is the TEST's own counter — nothing
// the op or the fake owns), or an exit that leaves the intake energized.
TEST_CASE("RunUntilConfirmed: hand-written timeline — confirmed at tick 8, exactly") {
    MotorOpRig x;
    int loopTick = 0;  // the test's independent notion of time
    RunUntilConfirmed op{x.mech, x.r.deps, x.cfg, [&] { return loopTick >= 8; }, "grab"};

    op.start();
    CHECK(x.mech.claimed());
    MechanismOutcome o = MechanismOutcome::Running;
    int exitTick = -1;
    for (int i = 0; i < 100; ++i) {
        o = op.tick();
        if (o != MechanismOutcome::Running) {
            exitTick = i;
            break;
        }
        // Every Running tick commands the configured voltage — at the DEVICE.
        CHECK(x.a.commandedVoltage().value() == 6.0);
        CHECK(x.b.commandedVoltage().value() == 6.0);
        x.r.clock.advance(Time{0.01});
        ++loopTick;
    }
    // The contract, as literals: ticks 0..7 Running, verdict at tick 8.
    CHECK(o == MechanismOutcome::Succeeded);
    CHECK(exitTick == 8);
    CHECK(op.finished());
    // Exit disposition, at the bottom of the stack.
    CHECK(x.a.commandedVoltage().value() == 0.0);
    CHECK(x.a.brakeMode() == BrakeMode::Coast);
    CHECK_FALSE(x.mech.claimed());
    // A clean success is silent: no fault, no MECH warn.
    CHECK(x.r.latch.faultCount() == 0);
    CHECK(countMechWarns(x.r.sink, "") == 0);
}

// Bug caught: confirm evaluated AFTER commanding — an already-satisfied
// condition (the ring is already held) would spin the intake for one tick.
// The C2 pred-before-first-tick shape, pinned at the device: zero energizing
// events ever.
TEST_CASE("RunUntilConfirmed: true-on-entry succeeds without ever energizing") {
    OpRig r;
    RecordingMotor rec;
    std::array<IMotor*, 1> motors{&rec};
    MotorMechanism mech{motors, BrakeMode::Brake, "intake"};
    RunUntilConfirmedConfig cfg{
        .voltage = Voltage{6.0},
        .timeout = Time{0.5},
        .stall = StallConfig{.currentAtLeast = Current{2.0},
                             .speedAtMost = AngularVelocity{0.1},
                             .persistence = Time{0.05}}};
    RunUntilConfirmed op{mech, r.deps, cfg, [] { return true; }, "grab"};

    op.start();
    CHECK(op.tick() == MechanismOutcome::Succeeded);
    CHECK_FALSE(rec.everEnergized());
    // The safe state still lands (mode, then 0 V — two events per motor).
    CHECK(rec.brakeMode() == BrakeMode::Brake);
    CHECK(rec.commandedVoltage().value() == 0.0);
}

// Bug caught: the watchdog disarmed, armed at first tick instead of start(),
// or accumulated from dt instead of the absolute deadline. Timeout 0.5 s at
// dt = 0.01: expiry lands exactly at tick 50 (t = 0.50), confirm never true.
// NO FAULT is the T6 ruling: a healthy mechanism whose world did not
// cooperate is strategy, not pathology — the Warn line is the visibility.
TEST_CASE("RunUntilConfirmed: never-confirm exits TimedOut at tick 50, no fault") {
    MotorOpRig x;
    RunUntilConfirmed op{x.mech, x.r.deps, x.cfg, [] { return false; }, "grab"};
    op.start();
    const DriveResult res = drive(op, x.r.clock, 0.01, 200);
    CHECK(res.outcome == MechanismOutcome::TimedOut);
    CHECK(res.exitTick == 50);
    CHECK(x.r.latch.faultCount() == 0);  // T6: no MechanismTimeout code exists
    CHECK(countMechWarns(x.r.sink, "TIMED_OUT") == 1);
    CHECK(x.a.commandedVoltage().value() == 0.0);
    CHECK_FALSE(x.mech.claimed());
}

// Bug caught: "armed in start()" broken — a caller that starts the operation
// and first ticks it late (a routine doing other work) must still be inside
// the SAME budget; re-arming at first tick would silently extend it.
TEST_CASE("RunUntilConfirmed: the watchdog runs from start(), not from the first tick") {
    MotorOpRig x;
    RunUntilConfirmed op{x.mech, x.r.deps, x.cfg, [] { return false; }, "grab"};
    op.start();               // t = 0 — the budget starts HERE
    x.r.clock.advance(Time{0.6});  // caller dawdles past the whole budget
    CHECK(op.tick() == MechanismOutcome::TimedOut);  // first tick, already over
}

// THE NO-HANG PROOF under an adversarial clock. Bug caught: a watchdog that
// needs well-behaved dt — zero-dt stretches (a frozen segment), microscopic
// steps and a 5 s spike must all still exit at the hand-computed tick. The
// deadline is absolute: cumulative time crosses 0.5 s at tick 4 below.
TEST_CASE("RunUntilConfirmed: adversarial clock — erratic dt still exits, exactly") {
    MotorOpRig x;
    RunUntilConfirmed op{x.mech, x.r.deps, x.cfg, [] { return false; }, "grab"};
    op.start();
    // dt AFTER tick i:      t at tick i:
    //   tick 0: +0.0          0.00
    //   tick 1: +0.2          0.00   (zero-dt: same instant, twice)
    //   tick 2: +0.0          0.20
    //   tick 3: +0.4          0.20
    //   tick 4:  —            0.60 ≥ 0.5 → TimedOut
    const double dts[] = {0.0, 0.2, 0.0, 0.4};
    int i = 0;
    MechanismOutcome o = MechanismOutcome::Running;
    for (; i < 10; ++i) {
        o = op.tick();
        if (o != MechanismOutcome::Running) {
            break;
        }
        REQUIRE(i < 4);  // must not need more advances than the schedule has
        x.r.clock.advance(Time{dts[i]});
    }
    CHECK(o == MechanismOutcome::TimedOut);
    CHECK(i == 4);

    SUBCASE("one giant jump lands the very next tick") {
        MotorOpRig y;
        RunUntilConfirmed op2{y.mech, y.r.deps, y.cfg, [] { return false; }, "grab"};
        op2.start();
        CHECK(op2.tick() == MechanismOutcome::Running);
        y.r.clock.advance(Time{1.0e6});
        CHECK(op2.tick() == MechanismOutcome::TimedOut);
    }
}

// Bug caught: the stall detector never trips (mutation: always healthy), the
// fault raise dropped while the verdict stays (E1's exact hole class — this
// case asserts the LATCH, not just the outcome), or the persistence window
// mis-measured. Signature from t=0, persistence 0.05 s ⇒ Stalled at tick 5.
TEST_CASE("RunUntilConfirmed: jam signature → Stalled at tick 5 + MechanismStalled latched") {
    MotorOpRig x;
    x.a.setCurrent(Current{2.5});
    x.b.setCurrent(Current{2.4});
    x.a.setVelocity(AngularVelocity{0.02});
    x.b.setVelocity(AngularVelocity{0.02});
    RunUntilConfirmed op{x.mech, x.r.deps, x.cfg, [] { return false; }, "grab"};
    op.start();
    const DriveResult res = drive(op, x.r.clock, 0.01, 100);
    CHECK(res.outcome == MechanismOutcome::Stalled);
    CHECK(res.exitTick == 5);
    // The E1 lesson: the verdict AND the latch, independently.
    CHECK(x.r.latch.raiseCount(FaultCode::MechanismStalled) == 1);
    CHECK(x.r.latch.firstFault() == FaultCode::MechanismStalled);
    CHECK(x.a.commandedVoltage().value() == 0.0);
    CHECK(x.a.brakeMode() == BrakeMode::Coast);  // the INTAKE safe state — not Hold
    CHECK_FALSE(x.mech.claimed());
}

// Bug caught: a start-up transient read as a jam — stall-grade current with
// the shaft still MOVING must never trip (spin-up draws stall current), and a
// single healthy sample must RESET the persistence window, not pause it.
TEST_CASE("StallDetector via the op: transients and interrupted windows do not trip") {
    SUBCASE("high current + moving shaft = spin-up, runs to the watchdog") {
        MotorOpRig x;
        x.a.setCurrent(Current{2.5});
        x.b.setCurrent(Current{2.5});
        x.a.setVelocity(AngularVelocity{8.0});  // well above the 0.1 floor
        x.b.setVelocity(AngularVelocity{8.0});
        RunUntilConfirmed op{x.mech, x.r.deps, x.cfg, [] { return false; }, "grab"};
        op.start();
        const DriveResult res = drive(op, x.r.clock, 0.01, 200);
        CHECK(res.outcome == MechanismOutcome::TimedOut);  // never Stalled
        CHECK(x.r.latch.raiseCount(FaultCode::MechanismStalled) == 0);
    }
    SUBCASE("a healthy sample resets the window: trip at tick 10, not tick 5") {
        // Binary-exact times, deliberately: dt = 2^-7 s and persistence = 5·dt,
        // so the hand timeline IS the float timeline. (The first draft used
        // dt = 0.01 and learned that accumulated decimal dust shifts the trip
        // by one tick — an ideal-arithmetic literal would have pinned a lie.)
        const double dt = 0.0078125;  // 2^-7 — exact in binary
        MotorOpRig x;
        RunUntilConfirmedConfig cfg = x.cfg;
        cfg.stall.persistence = Time{5.0 * dt};  // exact: 0.0390625
        RunUntilConfirmed op{x.mech, x.r.deps, cfg, [] { return false; }, "grab"};
        op.start();
        MechanismOutcome o = MechanismOutcome::Running;
        int exitTick = -1;
        for (int i = 0; i < 100; ++i) {
            // Signature on ticks 0-3, healthy on tick 4, signature from tick 5.
            const bool healthy = (i == 4);
            const auto vel = AngularVelocity{healthy ? 8.0 : 0.02};
            const auto cur = Current{healthy ? 0.3 : 2.5};
            x.a.setVelocity(vel);
            x.b.setVelocity(vel);
            x.a.setCurrent(cur);
            x.b.setCurrent(cur);
            o = op.tick();
            if (o != MechanismOutcome::Running) {
                exitTick = i;
                break;
            }
            x.r.clock.advance(Time{dt});
        }
        // Window restarts at t = 5·dt (tick 5) ⇒ the full 5·dt persistence
        // elapses at t = 10·dt, i.e. tick 10 — and had the reset been missing,
        // the trip would have landed at tick 5 with the uninterrupted case.
        CHECK(o == MechanismOutcome::Stalled);
        CHECK(exitTick == 10);
    }
}

// Bug caught: the tick's check order quietly reversed (stall before confirm).
// Order is LOAD-BEARING physics, not style: for a motorized clamp the current
// spike IS the capture signature, so a grab routinely stalls INTO its own
// confirmation — and on the tick where both hold, success must win (the
// ExitGroup settled-beats-timeout rule, mirrored). Stall-first would report a
// completed grab as a jam. Found by planning the mutation campaign: the order
// swap stayed green against every other case in this file.
TEST_CASE("RunUntilConfirmed: confirm and stall on the SAME tick — success wins") {
    const double dt = 0.0078125;  // binary-exact (the FP lesson)
    MotorOpRig x;
    RunUntilConfirmedConfig cfg = x.cfg;
    cfg.stall.persistence = Time{5.0 * dt};
    // Stall signature from tick 0 (injected directly — no command dependence):
    // the window opens at t = 0, so the detector would trip at t = 5·dt.
    x.a.setCurrent(Current{2.5});
    x.b.setCurrent(Current{2.5});
    x.a.setVelocity(AngularVelocity{0.02});
    x.b.setVelocity(AngularVelocity{0.02});
    // The confirmation ALSO becomes true exactly at t = 5·dt.
    RunUntilConfirmed op{x.mech, x.r.deps, cfg,
                         [&] { return x.r.clock.now().value() >= 5.0 * dt; }, "grab"};
    op.start();
    const DriveResult res = drive(op, x.r.clock, dt, 100);
    CHECK(res.outcome == MechanismOutcome::Succeeded);  // NOT Stalled
    CHECK(res.exitTick == 5);
    CHECK(x.r.latch.raiseCount(FaultCode::MechanismStalled) == 0);
}

// THE CANCEL CONTRACT, all four clauses, mirrored from IMotion and pinned as
// mirrored. Bug caught: any quiet divergence between the two contracts — skip
// the safe state, overwrite a completed verdict, command a never-started
// mechanism, raise a fault, or fail to be idempotent.
TEST_CASE("RunUntilConfirmed: the cancel contract, clause by clause") {
    SUBCASE("running → Cancelled + safe state at the device + claim released") {
        MotorOpRig x;
        RunUntilConfirmed op{x.mech, x.r.deps, x.cfg, [] { return false; }, "grab"};
        op.start();
        (void)op.tick();
        CHECK(x.a.commandedVoltage().value() == 6.0);  // energized mid-run
        op.cancel();
        CHECK(op.outcome() == MechanismOutcome::Cancelled);
        CHECK(x.a.commandedVoltage().value() == 0.0);
        CHECK(x.a.brakeMode() == BrakeMode::Coast);
        CHECK_FALSE(x.mech.claimed());
        CHECK(x.r.latch.faultCount() == 0);  // cancel raises nothing

        // Idempotent: dirty the device, cancel again — re-safed, verdict kept.
        x.mech.setVoltage(Voltage{3.0});
        op.cancel();
        CHECK(op.outcome() == MechanismOutcome::Cancelled);
        CHECK(x.a.commandedVoltage().value() == 0.0);
    }
    SUBCASE("completed verdict is PRESERVED — cancel still re-safes") {
        MotorOpRig x;
        RunUntilConfirmed op{x.mech, x.r.deps, x.cfg, [] { return true; }, "grab"};
        op.start();
        CHECK(op.tick() == MechanismOutcome::Succeeded);
        x.mech.setVoltage(Voltage{4.0});  // someone energized it afterwards
        op.cancel();
        CHECK(op.outcome() == MechanismOutcome::Succeeded);  // history not rewritten
        CHECK(x.a.commandedVoltage().value() == 0.0);        // but safe NOW
    }
    SUBCASE("never started → complete no-op at the device") {
        OpRig r;
        RecordingMotor rec;
        std::array<IMotor*, 1> motors{&rec};
        MotorMechanism mech{motors, BrakeMode::Hold, "lift"};
        RunUntilConfirmedConfig cfg{
            .voltage = Voltage{6.0},
            .timeout = Time{0.5},
            .stall = StallConfig{.currentAtLeast = Current{2.0},
                                 .speedAtMost = AngularVelocity{0.1},
                                 .persistence = Time{0.05}}};
        RunUntilConfirmed op{mech, r.deps, cfg, [] { return false; }, "grab"};
        op.cancel();
        CHECK(rec.eventCount() == 0);  // not even a safe state — no relationship yet
        CHECK_FALSE(op.started());
        CHECK(op.outcome() == MechanismOutcome::Running);
    }
}

// Bug caught: a finished operation that re-commands when a stale caller keeps
// ticking it — the pre-empt-inertness rule that makes one-active structural
// (C2's argument, mirrored). Also: reuse via start() must fully re-arm.
TEST_CASE("RunUntilConfirmed: finished is inert; start() re-arms completely") {
    OpRig r;
    RecordingMotor rec;
    std::array<IMotor*, 1> motors{&rec};
    MotorMechanism mech{motors, BrakeMode::Coast, "intake"};
    RunUntilConfirmedConfig cfg{
        .voltage = Voltage{6.0},
        .timeout = Time{0.5},
        .stall = StallConfig{.currentAtLeast = Current{2.0},
                             .speedAtMost = AngularVelocity{0.1},
                             .persistence = Time{0.05}}};
    int loopTick = 0;
    RunUntilConfirmed op{mech, r.deps, cfg, [&] { return loopTick >= 2; }, "grab"};

    op.start();
    const DriveResult first = [&] {
        for (int i = 0; i < 100; ++i) {
            const auto o = op.tick();
            if (o != MechanismOutcome::Running) {
                return DriveResult{o, i};
            }
            r.clock.advance(Time{0.01});
            ++loopTick;
        }
        return DriveResult{};
    }();
    CHECK(first.outcome == MechanismOutcome::Succeeded);
    CHECK(first.exitTick == 2);

    const int eventsAtExit = rec.eventCount();
    for (int i = 0; i < 5; ++i) {
        CHECK(op.tick() == MechanismOutcome::Succeeded);  // cached, inert
    }
    CHECK(rec.eventCount() == eventsAtExit);  // NOT ONE new device command

    // Re-arm: the same object runs again, from a fresh watchdog and claim.
    loopTick = 0;
    op.start();
    CHECK(mech.claimed());
    CHECK(op.outcome() == MechanismOutcome::Running);
    (void)op.tick();
    op.cancel();
    CHECK(op.outcome() == MechanismOutcome::Cancelled);
}

// Bug caught: the claim not enforced at start() — two operations silently
// double-driving one mechanism (the structural T3 rule). The refusal must be
// LOUD, must not disturb the running operation, and must clear on its exit.
TEST_CASE("Two operations on one mechanism: loud collision, then legal handoff") {
    MotorOpRig x;
    RunUntilConfirmed opA{x.mech, x.r.deps, x.cfg, [] { return false; }, "opA"};
    RunUntilConfirmed opB{x.mech, x.r.deps, x.cfg, [] { return false; }, "opB"};

    opA.start();
    CHECK_THROWS_AS(opB.start(), PreconditionError);
    CHECK(opA.tick() == MechanismOutcome::Running);  // A unharmed by B's attempt

    opA.cancel();  // A's exit releases the claim…
    opB.start();   // …and B may now start.
    CHECK(opB.tick() == MechanismOutcome::Running);
    opB.cancel();
}

// Bug caught: nonsense config accepted quietly — a 0 V "run", a NaN voltage,
// a non-positive timeout, or null deps must be loud construction failures,
// not a mid-auton surprise.
TEST_CASE("RunUntilConfirmed: construction preconditions") {
    MotorOpRig x;
    auto never = [] { return false; };
    auto cfgWith = [&](double v, double t) {
        RunUntilConfirmedConfig c = x.cfg;
        c.voltage = Voltage{v};
        c.timeout = Time{t};
        return c;
    };
    CHECK_THROWS_AS((RunUntilConfirmed{x.mech, x.r.deps, cfgWith(0.0, 0.5), never}),
                    PreconditionError);
    CHECK_THROWS_AS(
        (RunUntilConfirmed{x.mech, x.r.deps, cfgWith(std::nan(""), 0.5), never}),
        PreconditionError);
    CHECK_THROWS_AS((RunUntilConfirmed{x.mech, x.r.deps, cfgWith(6.0, 0.0), never}),
                    PreconditionError);
    MechanismDeps bad{.clock = nullptr, .faults = &x.r.latch, .telemetry = &x.r.sink};
    CHECK_THROWS_AS((RunUntilConfirmed{x.mech, bad, x.cfg, never}), PreconditionError);
}

// ── ActuateAndConfirm ───────────────────────────────────────────────────────────────

namespace {

// One discrete-op rig: clock/latch/sink + a clamp whose declared safe command
// is OPEN (false) — the polarity that makes the T4 split observable: applying
// "safe" on success would visibly un-grab.
struct AirOpRig {
    OpRig r;
    FakeDigitalOut line;
    std::array<IDigitalOut*, 1> lines{&line};
    PneumaticMechanism mech{lines, false, "clamp"};
    ActuateAndConfirmConfig cfg{
        .target = true, .actuationTime = Time{0.30}, .confirmWindow = Time{0.20}};
};

}  // namespace

// THE UNCONFIRMED TIMELINE — the verdict this whole vocabulary exists for,
// hand-written from the contract. Bug caught: confirmation consulted during
// actuation (reads the PRE-actuation world), the window mis-measured, the
// verdict misreported, a fault minted for a strategy outcome, or — the T4
// split — the completed actuation undone by its own exit.
TEST_CASE("ActuateAndConfirm: never-confirm — Unconfirmed at tick 50, exactly") {
    AirOpRig x;
    int predCalls = 0;
    ActuateAndConfirm op{x.mech, x.r.deps, x.cfg,
                         [&] {
                             ++predCalls;
                             return false;
                         },
                         "grab"};
    op.start();
    // Contract → literals (dt = 0.01, act 0.30, window 0.20):
    //   ticks  0..29  t <  0.30  Running, solenoid commanded, pred NOT consulted
    //   ticks 30..49  t <  0.50  Running, pred consulted once per tick (false)
    //   tick     50   t == 0.50  pred consulted, false → Unconfirmed
    MechanismOutcome o = MechanismOutcome::Running;
    int exitTick = -1;
    for (int i = 0; i < 200; ++i) {
        o = op.tick();
        if (i == 0) {
            CHECK(x.line.commanded());  // commanded on the FIRST tick
        }
        if (i < 30) {
            CHECK(predCalls == 0);  // the pre-actuation world is never asked
        }
        if (o != MechanismOutcome::Running) {
            exitTick = i;
            break;
        }
        x.r.clock.advance(Time{0.01});
    }
    CHECK(o == MechanismOutcome::Unconfirmed);
    CHECK(exitTick == 50);
    CHECK(predCalls == 21);  // ticks 30..50 inclusive
    // The T4 split: the completed actuation PERSISTS (line still true — the
    // jaws stay where the act left them), the declared safe state was NOT
    // applied by the exit…
    CHECK(x.line.commanded());
    // …and it is strategy, not pathology: Warn line, no fault.
    CHECK(x.r.latch.faultCount() == 0);
    CHECK(countMechWarns(x.r.sink, "UNCONFIRMED") == 1);
    CHECK_FALSE(x.mech.claimed());

    // cancel() IS the path that forces the declared safe state — and the
    // Unconfirmed verdict is preserved while it does.
    op.cancel();
    CHECK_FALSE(x.line.commanded());
    CHECK(op.outcome() == MechanismOutcome::Unconfirmed);
}

// Bug caught: a confirmation that arrives mid-window not honored at its exact
// tick, or a success that applies the safe state and un-grabs its own goal.
TEST_CASE("ActuateAndConfirm: confirm at t=0.35 — Succeeded at tick 35, grip kept") {
    AirOpRig x;
    ActuateAndConfirm op{
        x.mech, x.r.deps, x.cfg,
        [&] { return x.r.clock.now().value() >= 0.35; },  // the world confirms late
        "grab"};
    op.start();
    const DriveResult res = drive(op, x.r.clock, 0.01, 200);
    CHECK(res.outcome == MechanismOutcome::Succeeded);
    CHECK(res.exitTick == 35);
    CHECK(x.line.commanded());  // success does NOT un-actuate (T4 split)
    CHECK_FALSE(x.mech.claimed());
    CHECK(x.r.latch.faultCount() == 0);
    CHECK(countMechWarns(x.r.sink, "") == 0);
}

// Bug caught: an always-true confirmation consulted before the actuation
// deadline — it would "succeed" at tick 0 on the pre-actuation state of the
// world (the clamp's sensor still reporting the PREVIOUS grab). Exactly one
// consult, exactly at the deadline.
TEST_CASE("ActuateAndConfirm: an eager confirm cannot fire before actuation completes") {
    AirOpRig x;
    int predCalls = 0;
    ActuateAndConfirm op{x.mech, x.r.deps, x.cfg,
                         [&] {
                             ++predCalls;
                             return true;
                         },
                         "grab"};
    op.start();
    const DriveResult res = drive(op, x.r.clock, 0.01, 200);
    CHECK(res.outcome == MechanismOutcome::Succeeded);
    CHECK(res.exitTick == 30);  // the first instant the world may be believed
    CHECK(predCalls == 1);
}

// Bug caught: the window edge resolved against success — a confirmation
// arriving on the same tick the window closes must win (ExitGroup's
// settled-beats-timeout rule, mirrored).
TEST_CASE("ActuateAndConfirm: confirm exactly at the window edge wins") {
    AirOpRig x;
    ActuateAndConfirm op{x.mech, x.r.deps, x.cfg,
                         [&] { return x.r.clock.now().value() >= 0.50 - 1e-12; }, "grab"};
    op.start();
    const DriveResult res = drive(op, x.r.clock, 0.01, 200);
    CHECK(res.outcome == MechanismOutcome::Succeeded);  // not Unconfirmed
    CHECK(res.exitTick == 50);
}

// Bug caught: the degenerate fire-and-forget shape broken — zero actuation
// time + zero window + AlwaysConfirmed is the deploy verb, and it must
// complete on the first tick with the line commanded.
TEST_CASE("ActuateAndConfirm: act 0 + window 0 + AlwaysConfirmed = one-tick deploy") {
    AirOpRig x;
    ActuateAndConfirmConfig cfg{
        .target = true, .actuationTime = Time{0.0}, .confirmWindow = Time{0.0}};
    ActuateAndConfirm op{x.mech, x.r.deps, cfg, AlwaysConfirmed{}, "deploy"};
    op.start();
    CHECK(op.tick() == MechanismOutcome::Succeeded);
    CHECK(x.line.commanded());
}

// THE NO-HANG PROOF for the deadline-pair form. Bug caught: a deadline that
// re-arms or accumulates — a clock leaping past BOTH deadlines must resolve
// on the very next tick (to Unconfirmed here: the one consult says no).
TEST_CASE("ActuateAndConfirm: adversarial jump past both deadlines — one tick, done") {
    AirOpRig x;
    int predCalls = 0;
    ActuateAndConfirm op{x.mech, x.r.deps, x.cfg,
                         [&] {
                             ++predCalls;
                             return false;
                         },
                         "grab"};
    op.start();
    CHECK(op.tick() == MechanismOutcome::Running);
    x.r.clock.advance(Time{100.0});
    CHECK(op.tick() == MechanismOutcome::Unconfirmed);
    CHECK(predCalls == 1);
}

// Bug caught: cancel mid-actuation not forcing the declared safe state, or a
// cancelled discrete op left claiming its mechanism.
TEST_CASE("ActuateAndConfirm: cancel mid-actuation forces the declared safe state") {
    AirOpRig x;
    auto never = [] { return false; };
    ActuateAndConfirm op{x.mech, x.r.deps, x.cfg, never, "grab"};
    op.start();
    (void)op.tick();
    CHECK(x.line.commanded());  // actuating
    op.cancel();
    CHECK(op.outcome() == MechanismOutcome::Cancelled);
    CHECK_FALSE(x.line.commanded());  // declared safe (open)
    CHECK_FALSE(x.mech.claimed());
    CHECK(x.r.latch.faultCount() == 0);
}

// Bug caught: negative/NaN phase times accepted quietly.
TEST_CASE("ActuateAndConfirm: construction preconditions") {
    AirOpRig x;
    auto cfgWith = [](double act, double window) {
        return ActuateAndConfirmConfig{
            .target = true, .actuationTime = Time{act}, .confirmWindow = Time{window}};
    };
    CHECK_THROWS_AS(
        (ActuateAndConfirm{x.mech, x.r.deps, cfgWith(-0.1, 0.2), AlwaysConfirmed{}}),
        PreconditionError);
    CHECK_THROWS_AS(
        (ActuateAndConfirm{x.mech, x.r.deps, cfgWith(0.3, std::nan("")), AlwaysConfirmed{}}),
        PreconditionError);
}

// ── the claim token as an OWNERSHIP token, not just a collision detector (DEFECTS1) ──

// Bug caught (item A24): cancel() commanded applySafeState() whenever started_ was true,
// with no check that this operation still HOLDS the claim. finish() has already released it,
// so a retained, exited operation could safe a mechanism a DIFFERENT live operation now
// owned — defeating the one-operation-per-mechanism guarantee the token exists to provide.
// op2 running at 12 V drops to brake + 0 V silently, no fault, no Warn; and op2 re-commands
// its VOLTAGE next tick but not its BRAKE MODE, which is the half-safe state run_guard's T6
// note names as the reason applySafeState() alone is never trusted.
TEST_CASE("A24: a finished operation cannot safe a mechanism another operation now holds") {
    MotorOpRig x;
    RunUntilConfirmed op1{x.mech, x.r.deps, x.cfg, [] { return true; }, "op1"};
    op1.start();
    REQUIRE(op1.tick() == MechanismOutcome::Succeeded);   // finished: claim released
    REQUIRE_FALSE(x.mech.claimed());

    RunUntilConfirmed op2{x.mech, x.r.deps, x.cfg, [] { return false; }, "op2"};
    op2.start();
    REQUIRE(op2.tick() == MechanismOutcome::Running);
    // NEGATIVE CONTROL: op2 really is driving, or the check below proves nothing.
    REQUIRE(x.a.commandedVoltage().value() == doctest::Approx(6.0));
    REQUIRE(x.mech.claimed());

    op1.cancel();  // the stale call

    CHECK(x.a.commandedVoltage().value() == doctest::Approx(6.0));  // op2 undisturbed
    CHECK(x.mech.claimed());                                        // op2 still owns it
    CHECK(op2.outcome() == MechanismOutcome::Running);
    CHECK(op1.outcome() == MechanismOutcome::Succeeded);            // history not rewritten
    CHECK(x.r.latch.faultCount() == 0);
}

// Bug caught (item A25): the same hole on a DISCRETE actuator, where it is worse. finish()
// deliberately does NOT apply the safe state for a successful ActuateAndConfirm — a clamp
// whose safe state is "open" would fling its game piece the instant a grab succeeded (the T4
// split). So a stale cancel() on an exited object drove the line to the declared safe value
// and UN-DID an actuation a second, currently-claiming operation had just performed.
TEST_CASE("A25: a stale cancel cannot un-do a live operation's actuation") {
    AirOpRig x;
    ActuateAndConfirm op1{x.mech, x.r.deps, x.cfg, AlwaysConfirmed{}, "grab1"};
    op1.start();
    for (int i = 0; i < 100 && op1.outcome() == MechanismOutcome::Running; ++i) {
        (void)op1.tick();
        x.r.clock.advance(Time{0.01});
    }
    REQUIRE(op1.outcome() == MechanismOutcome::Succeeded);
    REQUIRE(x.line.commanded() == true);      // the actuation stands (T4)
    REQUIRE_FALSE(x.mech.claimed());

    ActuateAndConfirm op2{x.mech, x.r.deps, x.cfg, AlwaysConfirmed{}, "grab2"};
    op2.start();
    (void)op2.tick();
    REQUIRE(x.mech.claimed());
    REQUIRE(x.line.commanded() == true);

    op1.cancel();  // the stale call

    CHECK(x.line.commanded() == true);   // was FALSE — op2's grab silently opened
    CHECK(x.mech.claimed());
}

// The other half of the same rule, pinned so the fix cannot be read as "cancel stopped
// re-safing": with NOBODY else holding the claim, a finished operation's cancel() still
// applies the safe state, exactly as the cancel contract has always said.
TEST_CASE("A24/A25: with no other claimant, a finished cancel() still re-safes") {
    MotorOpRig x;
    RunUntilConfirmed op{x.mech, x.r.deps, x.cfg, [] { return true; }, "grab"};
    op.start();
    REQUIRE(op.tick() == MechanismOutcome::Succeeded);
    x.mech.setVoltage(Voltage{4.0});     // someone energized it afterwards
    op.cancel();
    CHECK(x.a.commandedVoltage().value() == 0.0);
    CHECK(op.outcome() == MechanismOutcome::Succeeded);
}

// Bug caught (item A10): IMechanism holds the claim as VALUE state and defaulted its
// copy/move members, so a copied mechanism arrived already claimed(), with claimant() aimed
// at an operation registered against the ORIGINAL. A legitimate tryClaim(copy) then failed
// for no visible reason, and F2's end-of-run guard walking a span containing the copy
// reached claimant() and cancelled an operation driving the original — whose own claim was
// never released. The operations already delete copy/move for the mirror reason.
TEST_CASE("A10: a mechanism cannot be copied or moved out of its claim") {
    static_assert(!std::is_copy_constructible_v<shulib::hal::MotorMechanism>);
    static_assert(!std::is_move_constructible_v<shulib::hal::MotorMechanism>);
    static_assert(!std::is_copy_assignable_v<shulib::hal::MotorMechanism>);
    static_assert(!std::is_copy_constructible_v<PneumaticMechanism>);
    static_assert(!std::is_move_constructible_v<PneumaticMechanism>);
    static_assert(!std::is_copy_constructible_v<shulib::hal::IMechanism>);
    // NEGATIVE CONTROL: it is still ordinarily constructible where it lives.
    MotorOpRig x;
    CHECK_FALSE(x.mech.claimed());
}
