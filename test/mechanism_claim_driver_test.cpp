// F2 CLAIM-HOOK + MID-FLIGHT-DESTRUCTION SUITE — the Rule 4 fixes made at F1's
// layer while building the end-of-run guard (hal/mechanism.hpp claimant-hook
// banner; mechanism_op.hpp destructor notes). Every case names the bug it
// would catch.
//
// The trap, in this suite's form: never ask the operation whether it is safe —
// ask the DEVICE (RecordingMotor / FakeDigitalOut at the bottom of the stack),
// and never assert brake mode alone: the measured half-safe state is
// `brake=Hold, V=9.0`, which passes any mode-only assertion while energized.

#include "doctest.h"

#include <array>

#include "mechanism_test_rig.hpp"
#include "shulib/hal/fake/fake_digital_out.hpp"
#include "shulib/hal/mechanism.hpp"
#include "shulib/manipulation/mechanism_op.hpp"
#include "shulib/units/quantity.hpp"

using mech_rig::OpRig;
using mech_rig::RecordingMotor;
using shulib::hal::BrakeMode;
using shulib::hal::ICancellable;
using shulib::hal::IDigitalOut;
using shulib::hal::IMotor;
using shulib::hal::MotorMechanism;
using shulib::hal::PneumaticMechanism;
using shulib::hal::fake::FakeDigitalOut;
using shulib::manipulation::ActuateAndConfirm;
using shulib::manipulation::ActuateAndConfirmConfig;
using shulib::manipulation::MechanismOutcome;
using shulib::manipulation::RunUntilConfirmed;
using shulib::manipulation::RunUntilConfirmedConfig;
using shulib::manipulation::StallConfig;
using shulib::units::AngularVelocity;
using shulib::units::Current;
using shulib::units::Time;
using shulib::units::Voltage;

namespace {

/// An intake-shaped rig whose declared safe mode is HOLD on purpose: it makes
/// the measured half-safe state (`brake=Hold, V=9.0`) expressible, so a test
/// that asserted mode alone would go green on an energized mechanism — the
/// exact blindness this suite exists to rule out.
struct HookRig {
    OpRig r;
    RecordingMotor motor;
    std::array<IMotor*, 1> motors{&motor};
    MotorMechanism mech{motors, BrakeMode::Hold, "lift"};

    RunUntilConfirmedConfig cfg{
        .voltage = Voltage{9.0},
        .timeout = Time{5.0},
        .stall = StallConfig{.currentAtLeast = Current{999.0},  // never trips
                             .speedAtMost = AngularVelocity{0.0},
                             .persistence = Time{10.0}}};
};

/// Device-level safety check: voltage AND mode together (file banner).
void checkDeviceSafe(const RecordingMotor& m, BrakeMode declared) {
    CHECK(m.commandedVoltage().value() == 0.0);
    CHECK(m.brakeMode() == declared);
}

}  // namespace

// ── the claimant hook ───────────────────────────────────────────────────────────────

// Bug caught: the F1 gap the F2 measurements exposed — the claim said THAT a
// mechanism was driven but not BY WHAT, so a guard holding span<IMechanism*>
// could not reach a stalled operation. The hook must expose the claimant.
TEST_CASE("F2 claim hook: a started operation is reachable through claimant()") {
    HookRig h;
    bool confirmed = false;
    RunUntilConfirmed op{h.mech, h.r.deps, h.cfg, [&] { return confirmed; }, "grab"};
    CHECK(h.mech.claimant() == nullptr);  // unclaimed: nothing to reach
    op.start();
    CHECK(h.mech.claimed());
    CHECK(h.mech.claimant() == static_cast<ICancellable*>(&op));  // reachable
}

// Bug caught: a claimant left registered after the operation exits — a guard
// would cancel() a finished (possibly reused-for-something-else) object.
TEST_CASE("F2 claim hook: every exit path clears the registration") {
    HookRig h;
    SUBCASE("success clears it") {
        bool confirmed = false;
        RunUntilConfirmed op{h.mech, h.r.deps, h.cfg, [&] { return confirmed; }, "grab"};
        op.start();
        CHECK(op.tick() == MechanismOutcome::Running);
        CHECK(h.mech.claimant() != nullptr);
        confirmed = true;
        CHECK(op.tick() == MechanismOutcome::Succeeded);
        CHECK(h.mech.claimant() == nullptr);
        CHECK_FALSE(h.mech.claimed());
    }
    SUBCASE("cancel clears it") {
        bool confirmed = false;
        RunUntilConfirmed op{h.mech, h.r.deps, h.cfg, [&] { return confirmed; }, "grab"};
        op.start();
        op.cancel();
        CHECK(h.mech.claimant() == nullptr);
        CHECK_FALSE(h.mech.claimed());
    }
}

// Bug caught: cancelling through the hal-tier pointer behaving differently
// from cancelling the operation directly — the guard uses ONLY the base
// pointer and must get the full contract: inert, safe (voltage AND mode, at
// the device), claim released, verdict Cancelled.
TEST_CASE("F2 claim hook: cancel through ICancellable* is the full cancel contract") {
    HookRig h;
    bool confirmed = false;
    RunUntilConfirmed op{h.mech, h.r.deps, h.cfg, [&] { return confirmed; }, "grab"};
    op.start();
    CHECK(op.tick() == MechanismOutcome::Running);  // commands 9 V
    CHECK(h.motor.commandedVoltage().value() == 9.0);

    ICancellable* reach = h.mech.claimant();
    REQUIRE(reach != nullptr);
    reach->cancel();

    CHECK(op.outcome() == MechanismOutcome::Cancelled);
    checkDeviceSafe(h.motor, BrakeMode::Hold);
    CHECK_FALSE(h.mech.claimed());
    // The M13 regression, at the unit level: a further tick must NOT
    // re-energize (applySafeState alone lasts exactly one tick; op-cancel is
    // what makes safe STAY safe).
    const int events = h.motor.eventCount();
    CHECK(op.tick() == MechanismOutcome::Cancelled);
    CHECK(h.motor.eventCount() == events);  // inert: not one more device event
    checkDeviceSafe(h.motor, BrakeMode::Hold);
}

// Bug caught: the anonymous overload silently registering something, or the
// registering overload refusing a legal claim. Both spellings must coexist:
// bare tryClaim() stays legal (F1 tests, third-party ops) and is invisible to
// the guard by DESIGN — the Warn-and-force-release path, tested at the guard.
TEST_CASE("F2 claim hook: anonymous claims still work and read as claimant-less") {
    HookRig h;
    CHECK(h.mech.tryClaim());
    CHECK(h.mech.claimed());
    CHECK(h.mech.claimant() == nullptr);   // anonymous: nothing to reach
    CHECK_FALSE(h.mech.tryClaim());        // still exclusive
    h.mech.releaseClaim();
    CHECK_FALSE(h.mech.claimed());
}

// ── mid-flight destruction (the found F1 defect) ────────────────────────────────────

// Bug caught: THE DEFECT ITSELF — an operation destroyed mid-flight (the
// timeout-mismatch idiom: the caller's wait gave up before the op's watchdog)
// left its mechanism CLAIMED FOREVER and ENERGIZED at the last commanded
// voltage, and the stuck claim made the end action's operation throw at
// start(). Before the F2 destructor this test's final section was reality.
TEST_CASE("F2 destructor: a running operation destroyed mid-flight cancels itself") {
    HookRig h;
    {
        bool confirmed = false;
        RunUntilConfirmed op{h.mech, h.r.deps, h.cfg, [&] { return confirmed; }, "grab"};
        op.start();
        CHECK(op.tick() == MechanismOutcome::Running);
        CHECK(h.motor.commandedVoltage().value() == 9.0);  // energized mid-flight
        // scope exits with the op RUNNING — the wait "gave up" on it
    }
    checkDeviceSafe(h.motor, BrakeMode::Hold);  // NOT left at 9 V
    CHECK_FALSE(h.mech.claimed());              // NOT claimed forever
    CHECK(h.mech.claimant() == nullptr);        // NOT a dangling registration

    // ...and the end action's operation can now start instead of throwing.
    bool confirmed2 = true;
    RunUntilConfirmed next{h.mech, h.r.deps, h.cfg, [&] { return confirmed2; }, "park-verify"};
    next.start();
    CHECK(next.tick() == MechanismOutcome::Succeeded);
}

// Bug caught: an UNCONDITIONAL destructor cancel — cancel() on a finished
// discrete op re-applies the declared safe state, so every successfully
// grabbed clamp would be un-grabbed the moment its operation went out of
// scope (the T4 persist rule, enforced through destruction).
TEST_CASE("F2 destructor: a FINISHED operation is left untouched — success persists") {
    OpRig r;
    FakeDigitalOut line;
    std::array<IDigitalOut*, 1> lines{&line};
    PneumaticMechanism clamp{lines, /*safe=*/false, "clamp"};  // declared safe: open
    {
        bool confirmed = false;
        ActuateAndConfirm op{clamp, r.deps,
                             ActuateAndConfirmConfig{.target = true,
                                                     .actuationTime = Time{0.1},
                                                     .confirmWindow = Time{0.5}},
                             [&] { return confirmed; }, "grab"};
        op.start();
        CHECK(op.tick() == MechanismOutcome::Running);  // commands closed
        r.clock.advance(Time{0.1});
        confirmed = true;
        CHECK(op.tick() == MechanismOutcome::Succeeded);
        CHECK(line.commanded() == true);  // grabbed
    }
    CHECK(line.commanded() == true);  // STILL grabbed — destruction changed nothing
}

// Bug caught: a never-started operation's destructor touching the device (the
// cancel contract's third clause, carried through to destruction).
TEST_CASE("F2 destructor: a never-started operation touches nothing") {
    HookRig h;
    {
        bool confirmed = false;
        RunUntilConfirmed op{h.mech, h.r.deps, h.cfg, [&] { return confirmed; }, "grab"};
        (void)op;
    }
    CHECK(h.motor.eventCount() == 0);  // not one device event
    CHECK_FALSE(h.mech.claimed());
}
