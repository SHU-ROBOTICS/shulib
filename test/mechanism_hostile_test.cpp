// F1 HOSTILE SUITE — the three worlds build-order.md §F1 names (a jammed
// intake, an unconfirmed grab, a stalled lift) plus the lying devices that
// prove the operation layer's guards do not depend on sensor honesty. Every
// case names the bug it would catch.
//
// Times are BINARY-EXACT throughout (dt = 2^-7 s; windows/persistences are
// multiples of dt), so every hand-written tick literal is the float timeline
// exactly — the lesson the op suite's first red taught.

#include "doctest.h"

#include <array>

#include "mechanism_test_rig.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/hal/fake/fake_digital_out.hpp"
#include "shulib/hal/fake/fake_motor.hpp"
#include "shulib/hal/mechanism.hpp"
#include "shulib/manipulation/mechanism_op.hpp"
#include "shulib/sim/hostile/mechanism_hostility.hpp"

using mech_rig::DriveResult;
using mech_rig::OpRig;
using mech_rig::countMechWarns;
using mech_rig::drive;
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
using shulib::manipulation::MechanismOutcome;
using shulib::manipulation::RunUntilConfirmed;
using shulib::manipulation::RunUntilConfirmedConfig;
using shulib::manipulation::StallConfig;
using shulib::sim::ConfirmAfter;
using shulib::sim::JammedMotor;
using shulib::sim::JammedMotorConfig;
using shulib::sim::LyingSpinMotor;
using shulib::sim::LyingSpinMotorConfig;
using shulib::sim::NeverConfirm;
using shulib::units::AngleDim;
using shulib::units::AngularVelocity;
using shulib::units::Current;
using shulib::units::Time;
using shulib::units::Voltage;

namespace {

constexpr double kDt = 0.0078125;  // 2^-7 — binary exact

/// Stall thresholds used across this file: trip on ≥ 2 A with the shaft at or
/// under 0.1 rad/s for 5·dt continuously.
StallConfig stallCfg() {
    return {.currentAtLeast = Current{2.0},
            .speedAtMost = AngularVelocity{0.1},
            .persistence = Time{5.0 * kDt}};
}

/// A FULL-voltage run (12 V): under JammedMotor the reported jam current is
/// stallCurrentAt12V·|V|/12 = 2.5 A, which clears the 2 A threshold. (At 6 V
/// it would read 1.25 A and correctly NOT trip — the scaling is deliberate
/// physics, and why StallConfig has no library defaults.)
RunUntilConfirmedConfig fullPowerCfg() {
    return {.voltage = Voltage{12.0}, .timeout = Time{0.5}, .stall = stallCfg()};
}

/// Healthy inner readings a hostile wrapper corrupts.
void makeHealthy(FakeMotor& m) {
    m.setVelocity(AngularVelocity{15.0});
    m.setCurrent(Current{0.6});
    m.setPosition(AngleDim{3.0});
}

}  // namespace

// THE LIVENESS PIN — A3's "a dead composed model cannot ship", and the killer
// for the named mutation "make the hostile fake's jam injection a no-op".
// Bug caught: a jam window that does not measurably change what the devices
// report — every hostile case downstream would be silently testing a healthy
// world.
TEST_CASE("JammedMotor: the jam measurably bites, and only inside its window") {
    OpRig r;  // just the clock
    FakeMotor inner;
    makeHealthy(inner);
    JammedMotor jam{inner, r.clock,
                    JammedMotorConfig{.start = Time{32.0 * kDt}, .end = Time{64.0 * kDt}}};
    jam.setVoltage(Voltage{12.0});

    // Before the window (t = 0): pass-through, bit for bit.
    CHECK(jam.velocity().value() == 15.0);
    CHECK(jam.current().value() == 0.6);
    CHECK(jam.position().value() == 3.0);

    // Inside the window (t = 40·dt): stall signature, position frozen. The
    // freeze latches at the FIRST in-window read, so read once before moving
    // the inner truth.
    r.clock.set(Time{40.0 * kDt});
    CHECK(jam.velocity().value() == doctest::Approx(0.05));  // creep, not 15
    CHECK(jam.current().value() == doctest::Approx(2.5));    // stall @ 12 V, not 0.6
    CHECK(jam.position().value() == 3.0);                    // latched at window entry
    inner.setPosition(AngleDim{9.0});                        // the truth keeps moving…
    CHECK(jam.position().value() == 3.0);                    // …the jammed shaft doesn't

    // Current scales with the command: half volts, half stall current.
    jam.setVoltage(Voltage{6.0});
    CHECK(jam.current().value() == doctest::Approx(1.25));
    jam.setVoltage(Voltage{-12.0});
    CHECK(jam.velocity().value() == doctest::Approx(-0.05));  // creep follows sign

    // After the window: honest again, including the position channel.
    r.clock.set(Time{64.0 * kDt});
    CHECK(jam.velocity().value() == 15.0);
    CHECK(jam.position().value() == 9.0);
}

// THE JAMMED INTAKE, end to end through the seam. Bug caught: an operation
// that keeps grinding a jammed mechanism at stall current until the thermal
// fault (~55 °C) instead of detecting, safing and reporting within the
// persistence window.
// Hand timeline (dt = 2^-7, jam opens at t = 32·dt, persistence 5·dt):
//   ticks  0..31  healthy readings           → Running
//   tick     32   signature appears, window opens at t = 32·dt
//   tick     37   t − 32·dt = 5·dt           → Stalled
TEST_CASE("Jammed intake: Stalled at tick 37, fault latched, intake safed") {
    OpRig r;
    FakeMotor inner;
    makeHealthy(inner);
    JammedMotor jam{inner, r.clock, JammedMotorConfig{.start = Time{32.0 * kDt}}};
    std::array<IMotor*, 1> motors{&jam};
    MotorMechanism intake{motors, BrakeMode::Coast, "intake"};
    RunUntilConfirmed op{intake, r.deps, fullPowerCfg(), NeverConfirm{}, "capture"};

    op.start();
    const DriveResult res = drive(op, r.clock, kDt, 200);
    CHECK(res.outcome == MechanismOutcome::Stalled);
    CHECK(res.exitTick == 37);
    CHECK(r.latch.raiseCount(FaultCode::MechanismStalled) == 1);
    CHECK(r.latch.firstFault() == FaultCode::MechanismStalled);
    // Safed at the BOTTOM — the inner FakeMotor, through the hostile wrapper.
    CHECK(inner.commandedVoltage().value() == 0.0);
    CHECK(inner.brakeMode() == BrakeMode::Coast);
    CHECK_FALSE(intake.claimed());
}

// THE STALLED LIFT: jammed from the first commanded tick (driven into its
// hard stop), declared safe state Hold. Bug caught: the lift's declared Hold
// not reaching the physical motor on the failure path — the T4 asymmetry has
// to hold under hostility, not only in the clean device test.
// Hand timeline — and note the tick-1 subtlety, which is physics, not fuzz:
// the jam CURRENT scales with the commanded voltage (I ≈ V/R at stall), and
// nothing has been commanded yet on tick 0's read, so tick 0 reads 0 A.
//   tick 0  reads 0 A (no command yet)      → Running; commands 12 V
//   tick 1  reads 2.5 A + creep, window opens at t = dt
//   tick 6  t − dt = 5·dt = persistence     → Stalled
TEST_CASE("Stalled lift: Stalled at tick 6, and the motor ends in HOLD") {
    OpRig r;
    FakeMotor inner;
    makeHealthy(inner);
    JammedMotor jam{inner, r.clock, JammedMotorConfig{}};  // jammed from t = 0
    std::array<IMotor*, 1> motors{&jam};
    MotorMechanism lift{motors, BrakeMode::Hold, "lift"};
    RunUntilConfirmed op{lift, r.deps, fullPowerCfg(), NeverConfirm{}, "liftRaise"};

    op.start();
    const DriveResult res = drive(op, r.clock, kDt, 200);
    CHECK(res.outcome == MechanismOutcome::Stalled);
    CHECK(res.exitTick == 6);
    CHECK(inner.brakeMode() == BrakeMode::Hold);  // the LOADED-LIFT safe state
    CHECK(inner.commandedVoltage().value() == 0.0);
    CHECK(r.latch.raiseCount(FaultCode::MechanismStalled) == 1);
}

// THE LYING DEVICE — the fake that lies, as the trap counter requires. The
// shaft truly stops (position channel = truth) but velocity and current keep
// reporting healthy values, so the stall detector CORRECTLY never trips: a
// detector can only know what the sensors say. Bug caught: an operation whose
// no-hang guarantee depends on any sensor telling the truth — the watchdog
// must exit it at its hand-computed tick with every sensor lying (timeout
// 0.5 s = 64·dt → TimedOut at tick 64), and no false Stalled may appear.
TEST_CASE("LyingSpinMotor: every sensor lies — the watchdog still ends it at tick 64") {
    OpRig r;
    FakeMotor inner;
    makeHealthy(inner);
    LyingSpinMotor liar{inner, r.clock,
                        LyingSpinMotorConfig{.start = Time{0.0},
                                             .reportedVelocity = AngularVelocity{15.0},
                                             .reportedCurrent = Current{0.6}}};
    std::array<IMotor*, 1> motors{&liar};
    MotorMechanism intake{motors, BrakeMode::Coast, "intake"};
    RunUntilConfirmed op{intake, r.deps, fullPowerCfg(), NeverConfirm{}, "capture"};

    // The truth channel: the shaft froze at its stall-instant position.
    inner.setPosition(AngleDim{7.0});
    CHECK(liar.position().value() == 7.0);  // frozen value latched at first read
    inner.setPosition(AngleDim{20.0});      // truth says the world moved on…
    CHECK(liar.position().value() == 7.0);  // …the stalled shaft did not
    CHECK(liar.velocity().value() == 15.0);  // the lie
    CHECK(liar.current().value() == 0.6);    // the lie

    op.start();
    const DriveResult res = drive(op, r.clock, kDt, 200);
    CHECK(res.outcome == MechanismOutcome::TimedOut);  // bounded, sensor-honesty-free
    CHECK(res.exitTick == 64);
    CHECK(r.latch.raiseCount(FaultCode::MechanismStalled) == 0);  // no false stall
    CHECK(r.latch.faultCount() == 0);  // and no fault at all (T6: timeout = strategy)
    CHECK(inner.commandedVoltage().value() == 0.0);  // still safed on exit
}

// THE UNCONFIRMED GRAB, in its named hostile form. Bug caught: the
// no-feedback reality of a solenoid leaking into the verdict — dead air is
// INVISIBLE at the device (digital_out.hpp), so the ONLY honest place this
// world can appear is the confirm channel, and the verdict must be
// Unconfirmed with the device still reporting "commanded".
// Hand timeline (act = 16·dt, window = 16·dt): Unconfirmed at tick 32.
TEST_CASE("Unconfirmed grab: dead air is invisible at the device; the verdict says so") {
    OpRig r;
    FakeDigitalOut line;
    std::array<IDigitalOut*, 1> lines{&line};
    PneumaticMechanism clamp{lines, false, "clamp"};
    ActuateAndConfirm op{clamp, r.deps,
                         ActuateAndConfirmConfig{.target = true,
                                                 .actuationTime = Time{16.0 * kDt},
                                                 .confirmWindow = Time{16.0 * kDt}},
                         NeverConfirm{}, "grab"};
    op.start();
    const DriveResult res = drive(op, r.clock, kDt, 200);
    CHECK(res.outcome == MechanismOutcome::Unconfirmed);
    CHECK(res.exitTick == 32);
    CHECK(line.commanded());  // the device was told, and can say nothing more
    CHECK(r.latch.faultCount() == 0);
    CHECK(countMechWarns(r.sink, "UNCONFIRMED") == 1);
}

// Bug caught: a confirmation that arrives late-but-inside the window being
// missed (window arithmetic off by a phase), asserted at its exact tick.
// act = 16·dt, window = 16·dt, confirm from t = 24·dt → Succeeded at tick 24.
TEST_CASE("ConfirmAfter inside the window: Succeeded at tick 24, exactly") {
    OpRig r;
    FakeDigitalOut line;
    std::array<IDigitalOut*, 1> lines{&line};
    PneumaticMechanism clamp{lines, false, "clamp"};
    ActuateAndConfirm op{clamp, r.deps,
                         ActuateAndConfirmConfig{.target = true,
                                                 .actuationTime = Time{16.0 * kDt},
                                                 .confirmWindow = Time{16.0 * kDt}},
                         ConfirmAfter{r.clock, Time{24.0 * kDt}}, "grab"};
    op.start();
    const DriveResult res = drive(op, r.clock, kDt, 200);
    CHECK(res.outcome == MechanismOutcome::Succeeded);
    CHECK(res.exitTick == 24);
    CHECK(line.commanded());  // success keeps the grip (the T4 split)
}

// THE TRUST BOUNDARY, demonstrated rather than pretended away: a confirm
// channel that lies TRUE (ConfirmAfter{clock, 0}) over a world where the
// intake is jammed and nothing was captured produces a false Succeeded —
// because the predicate is the operation's ONLY eye on the task, and it is
// trusted (mechanism_op.hpp). Bug caught: someone "fixing" this by teaching
// the op to second-guess its confirmation — the documented boundary would
// have silently moved, and F3's obligation to confirm on real sensors would
// evaporate. If this behaviour ever changes, it must change ON PURPOSE.
TEST_CASE("A confirm that lies true yields a false Succeeded — the documented boundary") {
    OpRig r;
    FakeMotor inner;
    makeHealthy(inner);
    JammedMotor jam{inner, r.clock, JammedMotorConfig{}};  // jammed the whole time
    std::array<IMotor*, 1> motors{&jam};
    MotorMechanism intake{motors, BrakeMode::Coast, "intake"};
    RunUntilConfirmed op{intake, r.deps, fullPowerCfg(),
                         ConfirmAfter{r.clock, Time{0.0}},  // instantly "confirmed"
                         "capture"};
    op.start();
    CHECK(op.tick() == MechanismOutcome::Succeeded);  // the lie is believed
    CHECK(r.latch.faultCount() == 0);
    // The stall never got a chance to be seen: confirm is checked first, and
    // the operation finished on tick 0. That is the boundary, stated plainly.
}

// COMPOSITION + ABLATION, the A3 rule: hostile motor models are decorators,
// so composing pathologies is nesting and removing one removes exactly that
// pathology. Bug caught: a wrapper that swallows or distorts its inner
// model's hostility (composition would not be attributable by ablation).
TEST_CASE("Nested jam windows compose; ablation removes exactly one window") {
    OpRig r;
    FakeMotor inner;
    makeHealthy(inner);
    JammedMotor early{inner, r.clock,
                      JammedMotorConfig{.start = Time{16.0 * kDt}, .end = Time{24.0 * kDt}}};
    JammedMotor late{early, r.clock,
                     JammedMotorConfig{.start = Time{40.0 * kDt}, .end = Time{48.0 * kDt}}};
    late.setVoltage(Voltage{12.0});

    struct Probe {
        double t;
        double composedVel;  // through BOTH wrappers
        double ablatedVel;   // through `early` only (the outer removed)
    };
    // Hand table: healthy = 15, creep = 0.05.
    const Probe probes[] = {
        {8.0 * kDt, 15.0, 15.0},    // before both windows
        {20.0 * kDt, 0.05, 0.05},   // inside EARLY's window (both see it)
        {30.0 * kDt, 15.0, 15.0},   // between windows
        {44.0 * kDt, 0.05, 15.0},   // inside LATE's window — ablation differs HERE
        {56.0 * kDt, 15.0, 15.0},   // after both
    };
    for (const Probe& p : probes) {
        r.clock.set(Time{p.t});
        CHECK(late.velocity().value() == doctest::Approx(p.composedVel));
        CHECK(early.velocity().value() == doctest::Approx(p.ablatedVel));
    }
}
