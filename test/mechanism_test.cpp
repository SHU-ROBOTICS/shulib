// F1 DEVICE SEAM — hal::IMechanism, MotorMechanism, PneumaticMechanism,
// IDigitalOut and the fakes. Every case names the bug it would catch.
//
// The load-bearing assertions here are all at the BOTTOM of the stack — on
// FakeMotor / FakeDigitalOut / RecordingMotor device state — never on a
// mechanism's own record of what it thinks it commanded. That is the F1 form
// of the rule that has caught five chunks' shared-model traps.

#include "doctest.h"

#include <array>
#include <span>

#include "mechanism_test_rig.hpp"
#include "shulib/core/check.hpp"
#include "shulib/hal/fake/fake_digital_out.hpp"
#include "shulib/hal/fake/fake_mechanism.hpp"
#include "shulib/hal/fake/fake_motor.hpp"
#include "shulib/hal/mechanism.hpp"

using mech_rig::RecordingMotor;
using shulib::PreconditionError;
using shulib::hal::BrakeMode;
using shulib::hal::IDigitalOut;
using shulib::hal::IMechanism;
using shulib::hal::IMotor;
using shulib::hal::MotorMechanism;
using shulib::hal::PneumaticMechanism;
using shulib::hal::fake::FakeDigitalOut;
using shulib::hal::fake::FakeMechanism;
using shulib::hal::fake::FakeMotor;
using shulib::units::AngularVelocity;
using shulib::units::Current;
using shulib::units::Voltage;

// Bug caught: the command fan-out drops a motor (one motor of a coupled pair
// left uncommanded fights the other at 0 V), or the readback reports the
// mechanism's own record instead of the device's — the clamp makes them
// differ, and only the device knows.
TEST_CASE("MotorMechanism: commands every motor; readback comes from the device") {
    FakeMotor a;
    FakeMotor b;
    std::array<IMotor*, 2> motors{&a, &b};
    MotorMechanism mech{motors, BrakeMode::Coast, "intake"};

    mech.setVoltage(Voltage{6.0});
    CHECK(a.commandedVoltage().value() == 6.0);
    CHECK(b.commandedVoltage().value() == 6.0);
    CHECK(mech.commandedVoltage().value() == 6.0);

    // Ask for the impossible: the DEVICE clamps to ±12, and the mechanism's
    // readback must report what the motor really got, not the request.
    mech.setVoltage(Voltage{15.0});
    CHECK(a.commandedVoltage().value() == 12.0);
    CHECK(mech.commandedVoltage().value() == 12.0);
}

// Bug caught: applySafeState commands zero volts BEFORE the brake mode — a
// momentary coast under load (the exact ordering applyCancelSafeState()
// documents for the drive, asserted here as event order, which voltage/mode
// snapshots cannot see).
TEST_CASE("MotorMechanism: safe state lands mode-first on every motor") {
    RecordingMotor a;
    RecordingMotor b;
    std::array<IMotor*, 2> motors{&a, &b};
    MotorMechanism mech{motors, BrakeMode::Hold, "lift"};

    mech.setVoltage(Voltage{-8.0});
    a.setVelocity(AngularVelocity{5.0});
    mech.applySafeState();

    for (const RecordingMotor* m : {&a, &b}) {
        CHECK(m->commandedVoltage().value() == 0.0);
        CHECK(m->brakeMode() == BrakeMode::Hold);
        // Event order: ... setVoltage(-8), setBrakeMode(Hold), setVoltage(0).
        const auto& ev = m->events();
        REQUIRE(ev.size() == 3);
        CHECK(ev[0].kind == RecordingMotor::Event::Kind::Voltage);
        CHECK(ev[0].volts == -8.0);
        CHECK(ev[1].kind == RecordingMotor::Event::Kind::Brake);
        CHECK(ev[1].mode == BrakeMode::Hold);
        CHECK(ev[2].kind == RecordingMotor::Event::Kind::Voltage);
        CHECK(ev[2].volts == 0.0);
    }
}

// Bug caught: one library-wide mechanism safe state — the T4 physical mistake.
// A lift declared Hold and an intake declared Coast must end in DIFFERENT
// device states after the identical applySafeState() call. If someone
// "simplifies" the declaration away, these two cannot both pass.
TEST_CASE("T4 asymmetry: lift ends Hold, intake ends Coast — at the motor") {
    FakeMotor liftA;
    FakeMotor liftB;
    FakeMotor intakeA;
    std::array<IMotor*, 2> liftMotors{&liftA, &liftB};
    std::array<IMotor*, 1> intakeMotors{&intakeA};
    MotorMechanism lift{liftMotors, BrakeMode::Hold, "lift"};
    MotorMechanism intake{intakeMotors, BrakeMode::Coast, "intake"};

    lift.setVoltage(Voltage{7.0});
    intake.setVoltage(Voltage{9.0});
    lift.applySafeState();
    intake.applySafeState();

    CHECK(liftA.brakeMode() == BrakeMode::Hold);
    CHECK(liftB.brakeMode() == BrakeMode::Hold);
    CHECK(intakeA.brakeMode() == BrakeMode::Coast);
    CHECK(liftA.commandedVoltage().value() == 0.0);
    CHECK(intakeA.commandedVoltage().value() == 0.0);
    CHECK(lift.safeBrakeMode() == BrakeMode::Hold);
    CHECK(intake.safeBrakeMode() == BrakeMode::Coast);
}

// Bug caught: a MEAN current would average a one-motor jam signature away
// (2.6 A + 0.4 A reads 1.5 A — under a 2 A threshold); the max cannot. And a
// max VELOCITY would hide a stalled shaft behind one lying encoder; the mean
// at least dilutes it — the group readers must be max-current / mean-velocity
// exactly.
TEST_CASE("MotorMechanism: maxCurrent is the max magnitude; meanVelocity the mean") {
    FakeMotor a;
    FakeMotor b;
    std::array<IMotor*, 2> motors{&a, &b};
    MotorMechanism mech{motors, BrakeMode::Brake, "intake"};

    a.setCurrent(Current{0.4});
    b.setCurrent(Current{-2.6});  // magnitude counts — sign is direction, not load
    a.setVelocity(AngularVelocity{2.0});
    b.setVelocity(AngularVelocity{4.0});

    CHECK(mech.maxCurrent().value() == doctest::Approx(2.6));
    CHECK(mech.meanVelocity().value() == doctest::Approx(3.0));
}

// Bug caught: a claim that is not exclusive (two operations silently driving
// one mechanism — the collision C2 made structural for motions), or a release
// that does not actually release.
TEST_CASE("IMechanism claim token: exclusive, releasable, observable") {
    FakeMechanism mech;
    CHECK_FALSE(mech.claimed());
    CHECK(mech.tryClaim());
    CHECK(mech.claimed());
    CHECK_FALSE(mech.tryClaim());  // second claimant refused
    mech.releaseClaim();
    CHECK_FALSE(mech.claimed());
    CHECK(mech.tryClaim());  // claimable again
    mech.releaseClaim();
    mech.releaseClaim();  // double release is harmless
    CHECK_FALSE(mech.claimed());
}

// Bug caught: the pneumatic fan-out drops a line (a two-cylinder clamp closing
// one jaw), readback reporting the request rather than the device, or a safe
// state that ignores the DECLARED value (both polarities must work — "safe"
// is a per-robot fact, not a constant).
TEST_CASE("PneumaticMechanism: fan-out, device readback, declared safe value") {
    FakeDigitalOut left;
    FakeDigitalOut right;
    std::array<IDigitalOut*, 2> lines{&left, &right};

    SUBCASE("safe = retracted (false)") {
        PneumaticMechanism clamp{lines, false, "clamp"};
        clamp.set(true);
        CHECK(left.commanded());
        CHECK(right.commanded());
        CHECK(clamp.commanded());
        clamp.applySafeState();
        CHECK_FALSE(left.commanded());
        CHECK_FALSE(right.commanded());
        CHECK_FALSE(clamp.safeCommand());
    }
    SUBCASE("safe = engaged (true) — a clamp that must keep its goal") {
        PneumaticMechanism clamp{lines, true, "clamp"};
        clamp.set(false);
        clamp.applySafeState();
        CHECK(left.commanded());
        CHECK(right.commanded());
        CHECK(clamp.safeCommand());
    }
}

// Bug caught: FakeDigitalOut losing command history — an idempotent
// re-command and a toggle storm look identical in commanded() alone; the
// count is what tells a test the difference.
TEST_CASE("FakeDigitalOut: commanded readback plus set() count") {
    FakeDigitalOut out;
    CHECK_FALSE(out.commanded());
    CHECK(out.setCount() == 0);
    out.set(true);
    out.set(true);
    out.set(false);
    CHECK_FALSE(out.commanded());
    CHECK(out.setCount() == 3);
}

// Bug caught: an IMechanism that cannot be implemented without devices (the
// H2 hal/sim shape — a VexBuilder joint has no IMotor), or a park-guard walk
// that misses a mechanism. This is F2's park guard in miniature: one
// heterogeneous list, one uniform verb, every declared safe state reached —
// asserted per concrete type at the device level.
TEST_CASE("A heterogeneous span<IMechanism*> can be forced safe uniformly") {
    FakeMotor liftMotor;
    std::array<IMotor*, 1> liftMotors{&liftMotor};
    MotorMechanism lift{liftMotors, BrakeMode::Hold, "lift"};

    FakeDigitalOut line;
    std::array<IDigitalOut*, 1> lines{&line};
    PneumaticMechanism clamp{lines, true, "clamp"};

    FakeMechanism joint{"sim-joint"};

    lift.setVoltage(Voltage{5.0});
    clamp.set(false);

    std::array<IMechanism*, 3> all{&lift, &clamp, &joint};
    for (IMechanism* m : all) {
        m->applySafeState();  // the guard needs nothing but the base type
    }

    CHECK(liftMotor.commandedVoltage().value() == 0.0);
    CHECK(liftMotor.brakeMode() == BrakeMode::Hold);
    CHECK(line.commanded());  // the DECLARED safe value, not "off"
    CHECK(joint.safeStateApplications() == 1);
}

// Bug caught: silently accepting a mechanism wired from nothing — an empty
// motor list or a null device would surface later as a crash mid-auton
// instead of a loud construction failure.
TEST_CASE("Construction preconditions are loud") {
    std::array<IMotor*, 0> noMotors{};
    CHECK_THROWS_AS((MotorMechanism{noMotors, BrakeMode::Coast, "x"}), PreconditionError);

    FakeMotor m;
    std::array<IMotor*, 2> withNull{&m, nullptr};
    CHECK_THROWS_AS((MotorMechanism{withNull, BrakeMode::Coast, "x"}), PreconditionError);

    std::array<IDigitalOut*, 0> noLines{};
    CHECK_THROWS_AS((PneumaticMechanism{noLines, false, "x"}), PreconditionError);
}
