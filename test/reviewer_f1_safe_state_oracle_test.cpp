// REVIEWER-AUTHORED INDEPENDENT ORACLE — chunk F1's load-bearing claim (T4): the
// DECLARED safe state, and the split by actuator physics.
//
// The claim, as the headers state it:
//   * MotorMechanism operations apply the DECLARED brake mode + 0 V on EVERY exit
//     (Succeeded / Stalled / TimedOut / Cancelled) — "stop the energy input"
//     (mechanism_op.hpp, "Which exits apply the safe state").
//   * ActuateAndConfirm LEAVES THE COMMANDED STATE IN PLACE on Succeeded and
//     Unconfirmed and applies the declared safe command ONLY on cancel() — a
//     solenoid burns nothing holding state, and un-commanding a completed
//     actuation undoes the act (same section; and ActuateAndConfirm::finish()'s
//     own comment).
//   * mechanism.hpp claims an ORDER inside MotorMechanism::applySafeState(): the
//     safe brake mode is written, THEN zero volts, "so the stop lands under the
//     declared semantics and never a momentary coast".
//
// This file was written from the HEADERS ONLY (hal/mechanism.hpp, hal/digital_out.hpp,
// hal/motor.hpp, manipulation/*.hpp, hal/fake/*). The chunk's own tests and its
// mechanism_test_rig.hpp were deliberately NOT read before this file was written and
// run: a test that shares a rig with the code under test shares its model, and a
// shared model cancels the error on both sides. Every rig, timeline and expectation
// below is this file's own.
//
// The physical claim itself (does BrakeMode::Hold actually hold a LOADED cascade
// lift?) is a hardware fact and is NOT testable here — mechanism.hpp already marks it
// PROVISIONAL (A4: HA-92). What IS testable, and all that is claimed below, is that
// the DEVICES at the bottom of the stack end each path in the declared state.
//
// Assertion discipline: every expectation reads the DEVICE — FakeMotor::brakeMode() /
// commandedVoltage(), FakeDigitalOut::commanded() / setCount() — per device, never the
// mechanism's own record and never MotorMechanism::commandedVoltage() (which reads
// motors_.front() only and is blind to motors 2..N by construction).
//
// Timelines are binary-exact (0.125 = 2^-3, 0.25 = 2^-2, 0.5 = 2^-1): a decimal clock
// step carries FP dust and lands an ideal-arithmetic literal a tick off a deadline.

#include "doctest.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <span>
#include <vector>

#include "shulib/core/check.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/hal/digital_out.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_digital_out.hpp"
#include "shulib/hal/fake/fake_motor.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/hal/mechanism.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/manipulation/mechanism_op.hpp"
#include "shulib/units/quantity.hpp"

namespace {

using shulib::hal::BrakeMode;
using shulib::hal::IDigitalOut;
using shulib::hal::IMechanism;
using shulib::hal::IMotor;
using shulib::hal::kMaxMotorVoltage;
using shulib::hal::MotorMechanism;
using shulib::hal::PneumaticMechanism;
using shulib::hal::fake::FakeClock;
using shulib::hal::fake::FakeDigitalOut;
using shulib::hal::fake::FakeMotor;
using shulib::hal::fake::FakeTelemetrySink;
using shulib::manipulation::ActuateAndConfirm;
using shulib::manipulation::ActuateAndConfirmConfig;
using shulib::manipulation::MechanismDeps;
using shulib::manipulation::MechanismOutcome;
using shulib::manipulation::RunUntilConfirmed;
using shulib::manipulation::RunUntilConfirmedConfig;
using shulib::manipulation::StallConfig;
using shulib::units::AngularVelocity;
using shulib::units::Current;
using shulib::units::Time;
using shulib::units::Voltage;

// ── the reviewer's own rigs ─────────────────────────────────────────────────────────

/// A confirmation the TEST drives explicitly: the operation reads exactly what this
/// file last wrote, with no clock coupling and no hidden state of its own.
struct FlagConfirm {
    const bool* flag;
    [[nodiscard]] bool operator()() const noexcept { return *flag; }
};

/// THREE motors on one shaft — not one, and not two. A safe state applied to
/// motors_.front() only is a real and cheap bug (the mechanism's own
/// commandedVoltage() reads exactly that motor), and it is invisible to any group
/// smaller than two. All three are PRE-SEEDED to BrakeMode::Brake: a third value,
/// neither declaration under test, so "ended in Hold" and "ended in Coast" can only
/// mean applySafeState() actually wrote it. (FakeMotor's own default is Coast —
/// testing a Coast declaration against that default would pass on a mechanism that
/// never touched the brake mode at all.)
struct MotorRig {
    static constexpr std::size_t kMotors = 3;

    FakeMotor motor[kMotors];
    IMotor* ptrs[kMotors] = {&motor[0], &motor[1], &motor[2]};
    FakeClock clock{Time{0.0}};
    FakeTelemetrySink sink;
    shulib::diag::FaultLatch faults{sink, clock};
    MotorMechanism mech;
    MechanismDeps deps{.clock = &clock, .faults = &faults, .telemetry = &sink};

    explicit MotorRig(BrakeMode declaredSafe)
        : mech{std::span<IMotor* const>{ptrs}, declaredSafe, "reviewerLift"} {
        for (FakeMotor& m : motor) {
            m.setBrakeMode(BrakeMode::Brake);
        }
    }
};

/// TWO solenoid lines on one circuit, for the same reason the motor rig has three.
struct PneumaticRig {
    static constexpr std::size_t kLines = 2;

    FakeDigitalOut line[kLines];
    IDigitalOut* ptrs[kLines] = {&line[0], &line[1]};
    FakeClock clock{Time{0.0}};
    FakeTelemetrySink sink;
    shulib::diag::FaultLatch faults{sink, clock};
    PneumaticMechanism mech;
    MechanismDeps deps{.clock = &clock, .faults = &faults, .telemetry = &sink};

    explicit PneumaticRig(bool declaredSafe)
        : mech{std::span<IDigitalOut* const>{ptrs}, declaredSafe, "reviewerClamp"} {}
};

/// An IMotor that records the ORDER of the commands it received. FakeMotor stores
/// only final state, and final state cannot see a momentary coast: "0 V then Hold"
/// and "Hold then 0 V" leave identical readbacks. This spy is still the bottom of the
/// stack — it IS the device the mechanism commands — and it applies the real ±12 V
/// clamp and non-finite rejection so it is not a softer device than FakeMotor.
class OrderSpyMotor final : public IMotor {
public:
    enum class Call { SetBrakeMode, SetVoltage };

    struct Event {
        Call what;
        BrakeMode modeAtCall;  ///< the brake mode in force WHEN this call landed
        double volts;          ///< the post-clamp voltage in force when this call landed
    };

    void setVoltage(Voltage volts) override {
        SHULIB_PRECONDITION(std::isfinite(volts.value()),
                            "OrderSpyMotor::setVoltage: voltage must be finite");
        volts_ = Voltage{std::clamp(volts.value(), -kMaxMotorVoltage.value(),
                                    kMaxMotorVoltage.value())};
        log_.push_back(Event{Call::SetVoltage, mode_, volts_.value()});
    }
    [[nodiscard]] Voltage commandedVoltage() const override { return volts_; }

    void setBrakeMode(BrakeMode mode) override {
        mode_ = mode;
        log_.push_back(Event{Call::SetBrakeMode, mode_, volts_.value()});
    }
    [[nodiscard]] BrakeMode brakeMode() const override { return mode_; }

    [[nodiscard]] shulib::units::AngleDim position() const override { return position_; }
    [[nodiscard]] AngularVelocity velocity() const override { return velocity_; }
    [[nodiscard]] Current current() const override { return current_; }
    [[nodiscard]] double temperature() const override { return 0.0; }

    void setCurrent(Current c) { current_ = c; }
    void setVelocity(AngularVelocity v) { velocity_ = v; }

    /// Seed state WITHOUT logging (rig setup is not part of the timeline under test).
    void seedBrakeMode(BrakeMode mode) noexcept { mode_ = mode; }
    void clearLog() noexcept { log_.clear(); }
    [[nodiscard]] const std::vector<Event>& log() const noexcept { return log_; }

private:
    Voltage volts_{0.0};
    BrakeMode mode_ = BrakeMode::Coast;
    shulib::units::AngleDim position_{0.0};
    AngularVelocity velocity_{0.0};
    Current current_{0.0};
    std::vector<Event> log_;
};

// ── hand-computed configuration ─────────────────────────────────────────────────────

/// Run at 8 V (exact); watchdog 0.25 s; stall = 2 A held for 0.125 s under 0.5 rad/s.
RunUntilConfirmedConfig motorConfig() {
    return RunUntilConfirmedConfig{.voltage = Voltage{8.0},
                                   .timeout = Time{0.25},
                                   .stall = StallConfig{.currentAtLeast = Current{2.0},
                                                        .speedAtMost = AngularVelocity{0.5},
                                                        .persistence = Time{0.125}}};
}

/// Actuation completes at t = 0.25; the confirm window closes at t = 0.75.
ActuateAndConfirmConfig airConfig(bool target) {
    return ActuateAndConfirmConfig{
        .target = target, .actuationTime = Time{0.25}, .confirmWindow = Time{0.5}};
}

// ── bottom-of-the-stack assertions ──────────────────────────────────────────────────

/// EVERY motor, individually. Never MotorMechanism::commandedVoltage() (front only).
void checkEveryMotor(const MotorRig& rig, BrakeMode expectedMode, double expectedVolts) {
    for (std::size_t i = 0; i < MotorRig::kMotors; ++i) {
        CAPTURE(i);
        CHECK(rig.motor[i].brakeMode() == expectedMode);
        CHECK(rig.motor[i].commandedVoltage().value() == expectedVolts);
    }
}

/// EVERY line, individually.
void checkEveryLine(const PneumaticRig& rig, bool expected) {
    for (std::size_t i = 0; i < PneumaticRig::kLines; ++i) {
        CAPTURE(i);
        CHECK(rig.line[i].commanded() == expected);
    }
}

// ── the four motor exit paths, on a hand-computed timeline ──────────────────────────

enum class MotorExit { Success, Timeout, Stall, Cancel };

/// Drives one operation to one exit and returns the verdict. The mid-run assertion is
/// ANTI-VACUITY armour: it proves the motors really were energized at 8 V before the
/// exit, so a later "0 V" means a stop and not a mechanism that never commanded
/// anything.
MechanismOutcome driveMotorTo(MotorRig& rig, MotorExit exit) {
    bool confirmed = false;
    RunUntilConfirmed<FlagConfirm> op{rig.mech, rig.deps, motorConfig(), FlagConfirm{&confirmed},
                                      "reviewerRun"};
    if (exit == MotorExit::Stall) {
        for (FakeMotor& m : rig.motor) {
            m.setCurrent(Current{3.0});               // 3.0 >= 2.0 A  → stall-grade
            m.setVelocity(AngularVelocity{0.0});      // 0.0 <= 0.5 rad/s → not turning
        }
    }

    op.start();                                        // t = 0
    CHECK(op.tick() == MechanismOutcome::Running);     // t = 0: 8 V commanded
    for (std::size_t i = 0; i < MotorRig::kMotors; ++i) {
        CAPTURE(i);
        REQUIRE(rig.motor[i].commandedVoltage().value() == 8.0);
    }

    switch (exit) {
        case MotorExit::Success:
            confirmed = true;
            return op.tick();                          // t = 0: confirm is checked first
        case MotorExit::Timeout:
            rig.clock.advance(Time{0.125});            // t = 0.125 < 0.25 → still running
            CHECK(op.tick() == MechanismOutcome::Running);
            rig.clock.advance(Time{0.125});            // t = 0.25 == timeout, EXACTLY
            return op.tick();
        case MotorExit::Stall:
            rig.clock.advance(Time{0.125});            // window opened at t=0; 0.125 s held
            return op.tick();                          // (0.125 < 0.25, so not a timeout)
        case MotorExit::Cancel:
            op.cancel();                               // t = 0, mid-run
            return op.outcome();
    }
    return MechanismOutcome::Running;  // unreachable
}

}  // namespace

// ════════════════════════════════════════════════════════════════════════════════════
// CLAIM 1 — a MOTOR mechanism ends in its DECLARED safe state on EVERY exit path.
// ════════════════════════════════════════════════════════════════════════════════════

TEST_CASE("F1 oracle (motor / declared HOLD): every exit ends Hold + 0 V on EVERY motor") {
    // BUG THIS CATCHES: an exit path that forgets applySafeState(). A loaded lift left
    // coasting at the end of a grab drops its stack; the paths are separate code
    // (finish() vs cancel()), so forgetting exactly one is the realistic failure and a
    // single-path test would not see it. It also catches a safe state applied only to
    // the first motor of the group — the other two keep whatever they had.
    MotorRig rig{BrakeMode::Hold};
    MechanismOutcome out = MechanismOutcome::Running;

    SUBCASE("succeeded") {
        out = driveMotorTo(rig, MotorExit::Success);
        CHECK(out == MechanismOutcome::Succeeded);
    }
    SUBCASE("timed out") {
        out = driveMotorTo(rig, MotorExit::Timeout);
        CHECK(out == MechanismOutcome::TimedOut);
    }
    SUBCASE("stalled") {
        out = driveMotorTo(rig, MotorExit::Stall);
        CHECK(out == MechanismOutcome::Stalled);
    }
    SUBCASE("cancelled") {
        out = driveMotorTo(rig, MotorExit::Cancel);
        CHECK(out == MechanismOutcome::Cancelled);
    }

    CHECK(out != MechanismOutcome::Running);        // the path really exited
    checkEveryMotor(rig, BrakeMode::Hold, 0.0);     // ...and every device is safe
}

TEST_CASE("F1 oracle (motor / declared COAST): every exit ends Coast + 0 V on EVERY motor") {
    // BUG THIS CATCHES: a HARDCODED safe brake mode. A library that always applies
    // Hold satisfies the test above and cooks a jammed intake at stall current to the
    // ~55 C thermal fault — the exact failure T4 says the per-mechanism declaration
    // exists to prevent. Only the second direction can see it, which is why both are
    // here. (The motors are pre-seeded to Brake, so "ended Coast" cannot be the
    // FakeMotor default sitting untouched.)
    MotorRig rig{BrakeMode::Coast};
    MechanismOutcome out = MechanismOutcome::Running;

    SUBCASE("succeeded") {
        out = driveMotorTo(rig, MotorExit::Success);
        CHECK(out == MechanismOutcome::Succeeded);
    }
    SUBCASE("timed out") {
        out = driveMotorTo(rig, MotorExit::Timeout);
        CHECK(out == MechanismOutcome::TimedOut);
    }
    SUBCASE("stalled") {
        out = driveMotorTo(rig, MotorExit::Stall);
        CHECK(out == MechanismOutcome::Stalled);
    }
    SUBCASE("cancelled") {
        out = driveMotorTo(rig, MotorExit::Cancel);
        CHECK(out == MechanismOutcome::Cancelled);
    }

    CHECK(out != MechanismOutcome::Running);
    checkEveryMotor(rig, BrakeMode::Coast, 0.0);
}

TEST_CASE("F1 oracle (motor): the brake mode is in force BEFORE the zero-volt command") {
    // BUG THIS CATCHES: zeroing the volts first and setting the brake mode after. Both
    // orders leave identical final state, so every readback-only test passes — but on
    // a V5, 0 V with the previous mode still in force IS a coast, so the loaded lift
    // free-falls for the width of that window. mechanism.hpp claims the order
    // explicitly ("safe brake mode on every motor, THEN zero volts"); only an ordered
    // record at the device can hold it to that.
    OrderSpyMotor spy[3];
    IMotor* ptrs[3] = {&spy[0], &spy[1], &spy[2]};
    for (OrderSpyMotor& s : spy) {
        s.seedBrakeMode(BrakeMode::Brake);  // neither Hold nor Coast; not logged
    }
    MotorMechanism mech{std::span<IMotor* const>{ptrs}, BrakeMode::Hold, "reviewerSpyLift"};
    FakeClock clock{Time{0.0}};
    FakeTelemetrySink sink;
    shulib::diag::FaultLatch faults{sink, clock};
    const MechanismDeps deps{.clock = &clock, .faults = &faults, .telemetry = &sink};

    bool confirmed = false;
    RunUntilConfirmed<FlagConfirm> op{mech, deps, motorConfig(), FlagConfirm{&confirmed},
                                      "reviewerSpyRun"};
    op.start();
    CHECK(op.tick() == MechanismOutcome::Running);  // 8 V on every motor
    for (OrderSpyMotor& s : spy) {
        s.clearLog();  // from here the log is the SAFE-STATE timeline alone
    }
    confirmed = true;
    CHECK(op.tick() == MechanismOutcome::Succeeded);

    for (std::size_t i = 0; i < 3; ++i) {
        CAPTURE(i);
        const std::vector<OrderSpyMotor::Event>& events = spy[i].log();
        REQUIRE(events.size() == 2);  // exactly one brake write and one volt write
        CHECK(events[0].what == OrderSpyMotor::Call::SetBrakeMode);
        CHECK(events[0].modeAtCall == BrakeMode::Hold);
        CHECK(events[1].what == OrderSpyMotor::Call::SetVoltage);
        CHECK(events[1].volts == 0.0);
        // The claim, stated as physics rather than as call order: at the instant the
        // motor was told 0 V, the declared brake mode was ALREADY in force.
        CHECK(events[1].modeAtCall == BrakeMode::Hold);
    }
}

TEST_CASE("F1 oracle (motor): a confirmation true on ENTRY still lands in the declared state") {
    // BUG THIS CATCHES: an early-out that returns Succeeded without going through the
    // safe-state path because "nothing was commanded, so nothing needs stopping". The
    // brake mode is a DECLARATION about how the mechanism rests, not a cleanup for
    // voltage this operation happened to apply: a lift already at its target must
    // still end holding, or it sags while the caller believes the op succeeded.
    OrderSpyMotor spy[2];
    IMotor* ptrs[2] = {&spy[0], &spy[1]};
    for (OrderSpyMotor& s : spy) {
        s.seedBrakeMode(BrakeMode::Brake);
    }
    MotorMechanism mech{std::span<IMotor* const>{ptrs}, BrakeMode::Hold, "reviewerEntryLift"};
    FakeClock clock{Time{0.0}};
    FakeTelemetrySink sink;
    shulib::diag::FaultLatch faults{sink, clock};
    const MechanismDeps deps{.clock = &clock, .faults = &faults, .telemetry = &sink};

    bool confirmed = true;  // already holding the ring, before the first tick
    RunUntilConfirmed<FlagConfirm> op{mech, deps, motorConfig(), FlagConfirm{&confirmed},
                                      "reviewerEntryRun"};
    op.start();
    CHECK(op.tick() == MechanismOutcome::Succeeded);

    for (std::size_t i = 0; i < 2; ++i) {
        CAPTURE(i);
        CHECK(spy[i].brakeMode() == BrakeMode::Hold);
        CHECK(spy[i].commandedVoltage().value() == 0.0);
        for (const OrderSpyMotor::Event& e : spy[i].log()) {
            CHECK(e.volts == 0.0);  // the 8 V run voltage was NEVER commanded
        }
    }
}

TEST_CASE("F1 oracle (motor): ticking a FINISHED operation re-energizes nothing") {
    // BUG THIS CATCHES: a finished-guard that falls through, so the next loop
    // iteration re-commands the run voltage. The mechanism would be safe for exactly
    // one tick and then spin back up under a caller who has already read Succeeded and
    // moved on — a stopped-then-restarted intake with nobody watching it.
    MotorRig rig{BrakeMode::Hold};
    bool confirmed = false;
    RunUntilConfirmed<FlagConfirm> op{rig.mech, rig.deps, motorConfig(), FlagConfirm{&confirmed},
                                      "reviewerPostExitRun"};
    op.start();
    CHECK(op.tick() == MechanismOutcome::Running);
    confirmed = true;
    CHECK(op.tick() == MechanismOutcome::Succeeded);
    checkEveryMotor(rig, BrakeMode::Hold, 0.0);

    confirmed = false;                                       // the world moved on
    rig.clock.advance(Time{1.0});                            // well past the 0.25 s budget
    CHECK(op.tick() == MechanismOutcome::Succeeded);         // cached verdict
    CHECK(op.tick() == MechanismOutcome::Succeeded);
    checkEveryMotor(rig, BrakeMode::Hold, 0.0);              // and still safe at the device
}

TEST_CASE("F1 oracle (motor): cancel() on a NEVER-STARTED operation commands nothing") {
    // BUG THIS CATCHES: an unconditional applySafeState() in cancel(). It looks
    // harmless and is not: an operation object constructed but never started has no
    // relationship to its mechanism yet, so cancelling it would reach across and
    // re-brake a mechanism another piece of code is legitimately driving by hand.
    MotorRig rig{BrakeMode::Hold};
    rig.mech.setVoltage(Voltage{6.0});  // someone is driving it directly, on purpose

    bool confirmed = false;
    RunUntilConfirmed<FlagConfirm> op{rig.mech, rig.deps, motorConfig(), FlagConfirm{&confirmed},
                                      "reviewerUnstarted"};
    op.cancel();

    checkEveryMotor(rig, BrakeMode::Brake, 6.0);  // untouched: seeded mode, hand command
    CHECK(op.outcome() == MechanismOutcome::Running);
}

// ════════════════════════════════════════════════════════════════════════════════════
// CLAIM 2 — a DISCRETE actuator KEEPS its commanded state on success/unconfirmed and
//           takes the declared safe command ONLY on cancel.
// ════════════════════════════════════════════════════════════════════════════════════

TEST_CASE("F1 oracle (discrete): SUCCESS retains the commanded state — safe is NOT applied") {
    // BUG THIS CATCHES: applying the declared safe state on success. A clamp whose
    // declared safe command is OPEN would fling the goal it just grabbed, at the exact
    // instant the grab reported Succeeded — the failure is invisible in the verdict
    // (still "Succeeded") and visible only at the line. Both directions are here
    // because a hardcoded set(false) on exit satisfies one of them.
    SUBCASE("safe = OPEN(false), grab commands CLOSED(true)") {
        PneumaticRig rig{false};  // declared safe: open
        bool confirmed = false;
        ActuateAndConfirm<FlagConfirm> op{rig.mech, rig.deps, airConfig(true),
                                          FlagConfirm{&confirmed}, "reviewerGrab"};
        op.start();                                       // t=0; act deadline 0.25
        CHECK(op.tick() == MechanismOutcome::Running);    // t=0: commands true, mid-actuation
        checkEveryLine(rig, true);
        rig.clock.advance(Time{0.25});                    // t = 0.25 == actuation deadline
        confirmed = true;
        CHECK(op.tick() == MechanismOutcome::Succeeded);

        checkEveryLine(rig, true);  // STILL closed — the goal is still in the clamp
    }
    SUBCASE("safe = CLOSED(true), release commands OPEN(false)") {
        PneumaticRig rig{true};  // declared safe: closed
        rig.mech.set(true);      // start from the safe/closed state, so false is a real change
        checkEveryLine(rig, true);

        bool confirmed = false;
        ActuateAndConfirm<FlagConfirm> op{rig.mech, rig.deps, airConfig(false),
                                          FlagConfirm{&confirmed}, "reviewerRelease"};
        op.start();
        CHECK(op.tick() == MechanismOutcome::Running);
        checkEveryLine(rig, false);
        rig.clock.advance(Time{0.25});
        confirmed = true;
        CHECK(op.tick() == MechanismOutcome::Succeeded);

        checkEveryLine(rig, false);  // STILL open — a completed release stays released
    }
}

TEST_CASE("F1 oracle (discrete): UNCONFIRMED retains the commanded state too") {
    // BUG THIS CATCHES: treating the failure verdict as a reason to revert. Unconfirmed
    // means "the solenoid fired and the sensor did not agree" — reverting turns the
    // caller's retry/undo decision into something the library already made, and on a
    // clamp that DID grab (with a dead confirm sensor) it drops the goal.
    PneumaticRig rig{false};  // declared safe: open
    bool confirmed = false;   // never confirms
    ActuateAndConfirm<FlagConfirm> op{rig.mech, rig.deps, airConfig(true),
                                      FlagConfirm{&confirmed}, "reviewerGrabUnconfirmed"};
    op.start();                                        // t=0; act 0.25, confirm window to 0.75
    CHECK(op.tick() == MechanismOutcome::Running);      // t = 0
    rig.clock.advance(Time{0.5});                       // t = 0.5: past actuation, inside window
    CHECK(op.tick() == MechanismOutcome::Running);
    rig.clock.advance(Time{0.25});                      // t = 0.75 == confirm deadline, EXACTLY
    CHECK(op.tick() == MechanismOutcome::Unconfirmed);

    checkEveryLine(rig, true);  // the commanded state persists for the caller to decide on
    CHECK(rig.sink.size() >= 1);  // ...and it was reported, not silent
}

TEST_CASE("F1 oracle (discrete): CANCEL applies the declared safe command, both directions") {
    // BUG THIS CATCHES: a cancel path that leaves a solenoid where it is. cancel() is
    // the park-guard hammer — it is where a team's declaration ("clamp stays closed at
    // the buzzer" / "cylinder retracts inside the expansion limit") is actually cashed
    // in. Both directions are here because a hardcoded value satisfies exactly one.
    SUBCASE("safe = OPEN(false)") {
        PneumaticRig rig{false};
        bool confirmed = false;
        ActuateAndConfirm<FlagConfirm> op{rig.mech, rig.deps, airConfig(true),
                                          FlagConfirm{&confirmed}, "reviewerCancelOpen"};
        op.start();
        CHECK(op.tick() == MechanismOutcome::Running);
        checkEveryLine(rig, true);  // mid-run: commanded away from safe (anti-vacuity)
        op.cancel();
        CHECK(op.outcome() == MechanismOutcome::Cancelled);
        checkEveryLine(rig, false);  // declared safe
    }
    SUBCASE("safe = CLOSED(true)") {
        PneumaticRig rig{true};
        bool confirmed = false;
        ActuateAndConfirm<FlagConfirm> op{rig.mech, rig.deps, airConfig(false),
                                          FlagConfirm{&confirmed}, "reviewerCancelClosed"};
        op.start();
        CHECK(op.tick() == MechanismOutcome::Running);
        checkEveryLine(rig, false);  // mid-run: commanded away from safe (anti-vacuity)
        op.cancel();
        CHECK(op.outcome() == MechanismOutcome::Cancelled);
        checkEveryLine(rig, true);  // declared safe
    }
}

TEST_CASE("F1 oracle (discrete): cancel AFTER success forces safe but preserves the verdict") {
    // BUG THIS CATCHES: the two halves of the cancel contract falling out of sync — a
    // cancel that skips the safe state because the op already finished (F2's park guard
    // then cannot force a finished mechanism safe at the buzzer), or one that rewrites
    // a real Succeeded into Cancelled (the outcome a caller logs and branches on would
    // then be a lie about what happened).
    PneumaticRig rig{false};  // declared safe: open
    bool confirmed = false;
    ActuateAndConfirm<FlagConfirm> op{rig.mech, rig.deps, airConfig(true),
                                      FlagConfirm{&confirmed}, "reviewerGrabThenPark"};
    op.start();
    CHECK(op.tick() == MechanismOutcome::Running);
    rig.clock.advance(Time{0.25});
    confirmed = true;
    CHECK(op.tick() == MechanismOutcome::Succeeded);
    checkEveryLine(rig, true);

    op.cancel();  // the buzzer
    checkEveryLine(rig, false);                        // safe state IS applied
    CHECK(op.outcome() == MechanismOutcome::Succeeded);  // verdict PRESERVED
}

TEST_CASE("F1 oracle (discrete): cancel() on a NEVER-STARTED operation commands nothing") {
    // BUG THIS CATCHES: an unconditional applySafeState() in cancel(). On a solenoid it
    // is worse than on a motor: cancelling an unstarted op would fire a cylinder that
    // no operation ever commanded, moving real hardware from a line of code whose whole
    // meaning is "stop".
    PneumaticRig rig{true};  // declared safe: closed
    bool confirmed = false;
    ActuateAndConfirm<FlagConfirm> op{rig.mech, rig.deps, airConfig(false),
                                      FlagConfirm{&confirmed}, "reviewerUnstartedAir"};
    op.cancel();

    for (std::size_t i = 0; i < PneumaticRig::kLines; ++i) {
        CAPTURE(i);
        CHECK(rig.line[i].setCount() == 0);   // the line was never touched at all
        CHECK(rig.line[i].commanded() == false);
    }
    CHECK(op.outcome() == MechanismOutcome::Running);
}

// ════════════════════════════════════════════════════════════════════════════════════
// CLAIM 3 — THE SPLIT ITSELF: the two rules must not collapse into one.
// ════════════════════════════════════════════════════════════════════════════════════

TEST_CASE("F1 oracle (THE SPLIT): motor-on-success and discrete-on-success must DIFFER") {
    // BUG THIS CATCHES: the two rules quietly unifying. Both collapses are physical
    // failures and each one keeps half the suite green: "always apply safe on exit"
    // flings the goal out of a clamp the moment the grab succeeds, and "never apply
    // safe on success" leaves the loaded lift coasting down after it arrives. Asserting
    // both halves in ONE case is what stops a future edit from unifying them and still
    // passing two separate, individually-green tests.
    MotorRig lift{BrakeMode::Hold};   // declared safe: Hold + 0 V
    PneumaticRig clamp{false};        // declared safe: OPEN (false)

    // Both mechanisms are driven to Succeeded, on the same clock arithmetic.
    CHECK(driveMotorTo(lift, MotorExit::Success) == MechanismOutcome::Succeeded);

    bool confirmed = false;
    ActuateAndConfirm<FlagConfirm> grab{clamp.mech, clamp.deps, airConfig(true),
                                        FlagConfirm{&confirmed}, "reviewerSplitGrab"};
    grab.start();
    CHECK(grab.tick() == MechanismOutcome::Running);
    clamp.clock.advance(Time{0.25});
    confirmed = true;
    CHECK(grab.tick() == MechanismOutcome::Succeeded);

    // Read the devices, and compare the two answers to the SAME question:
    // "did this mechanism end in its declared safe state?"
    bool motorEndedAtSafe = true;
    for (const FakeMotor& m : lift.motor) {
        motorEndedAtSafe = motorEndedAtSafe && m.brakeMode() == BrakeMode::Hold &&
                           m.commandedVoltage().value() == 0.0;
    }
    bool discreteEndedAtSafe = true;
    for (const FakeDigitalOut& l : clamp.line) {
        discreteEndedAtSafe = discreteEndedAtSafe && l.commanded() == false;  // false == declared safe
    }

    CHECK(motorEndedAtSafe);        // motors: stop the energy input
    CHECK_FALSE(discreteEndedAtSafe);  // air: the completed action persists
    CHECK(motorEndedAtSafe != discreteEndedAtSafe);  // the split, in one assertion
}

TEST_CASE("F1 oracle (IMechanism seam): applySafeState() through the base reaches EVERY device") {
    // BUG THIS CATCHES: F2's end-of-run park guard walking a heterogeneous list and
    // forcing only the FIRST motor / FIRST line of each group safe. The guard reports
    // success, the group's own readback (front motor / front line) agrees, and the rest
    // of the hardware stays energized past the buzzer.
    MotorRig rig{BrakeMode::Hold};
    for (FakeMotor& m : rig.motor) {
        m.setVoltage(Voltage{9.0});  // energized by hand; brake mode still the seeded Brake
    }
    IMechanism& motorSeam = rig.mech;
    motorSeam.applySafeState();
    checkEveryMotor(rig, BrakeMode::Hold, 0.0);

    PneumaticRig air{true};  // declared safe: closed
    air.mech.set(false);     // commanded open by hand
    checkEveryLine(air, false);
    IMechanism& airSeam = air.mech;
    airSeam.applySafeState();
    checkEveryLine(air, true);
}
