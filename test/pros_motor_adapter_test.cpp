// Adapter tests for ProsMotor THROUGH THE HOST SHIM (chunk R1a). What these can
// and cannot prove: the shim tests the adapter against our BELIEF about PROS
// (HA-94..98); it cannot test the belief — hardware tests the belief. What it
// CAN prove, and the pure conversion tests cannot: that the adapter actually
// CALLS the conversions and the configuration calls (C5's D-5 hole and E1's
// sink hole, both found by mutation — wiring invisible to every pure test).

#include "doctest.h"

#include <limits>

#include "pros/shim_control.hpp"

#include "shulib/core/check.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/hal/pros/motor.hpp"
#include "shulib/units/quantity.hpp"

using shulib::PreconditionError;
using shulib::hal::BrakeMode;
using shulib::hal::pros::MotorGearset;
using shulib::hal::pros::ProsMotor;
using shulib::units::Voltage;

TEST_CASE("ProsMotor: setVoltage sends MILLIVOLTS to the wire (the ×1000 is wired in)") {
    // BUG CAUGHT (mutation M1/M14): the adapter sending volts as millivolts —
    // move_voltage(6) instead of move_voltage(6000): 1/1000 torque, the robot
    // hums. Only a wire-level assertion can see this; the conversion test
    // cannot know it was CALLED.
    pros::shim::resetAll();
    ProsMotor m{1, MotorGearset::Green};
    m.setVoltage(Voltage{6.0});
    CHECK(pros::shim::motorState(1).lastVoltageMv == 6000);
    m.setVoltage(Voltage{-3.25});
    CHECK(pros::shim::motorState(1).lastVoltageMv == -3250);
}

TEST_CASE("ProsMotor: the ±12 V clamp holds at the wire AND in commandedVoltage()") {
    // BUG CAUGHT: clamping only one of the two paths — telemetry disagreeing
    // with the wire, or a 13500 mV command reaching a device documented to
    // ±12000.
    pros::shim::resetAll();
    ProsMotor m{1, MotorGearset::Green};
    m.setVoltage(Voltage{13.5});
    CHECK(pros::shim::motorState(1).lastVoltageMv == 12000);
    CHECK(m.commandedVoltage().value() == doctest::Approx(12.0));
    m.setVoltage(Voltage{-99.0});
    CHECK(pros::shim::motorState(1).lastVoltageMv == -12000);
    CHECK(m.commandedVoltage().value() == doctest::Approx(-12.0));
}

TEST_CASE("ProsMotor: a non-finite voltage is REJECTED, not coerced to zero (L4)") {
    // BUG CAUGHT (mutation M9): silent coerce-to-zero — the measurement
    // prototype's own recorded mistake. A NaN command must throw (host policy)
    // and must NOT reach the wire as 0; the last commanded value must survive.
    pros::shim::resetAll();
    ProsMotor m{1, MotorGearset::Green};
    m.setVoltage(Voltage{5.0});
    const double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK_THROWS_AS(m.setVoltage(Voltage{nan}), PreconditionError);
    // The wire and telemetry still hold the last good command — no zero snuck in.
    CHECK(pros::shim::motorState(1).lastVoltageMv == 5000);
    CHECK(m.commandedVoltage().value() == doctest::Approx(5.0));
}

TEST_CASE("ProsMotor: ctor SETS encoder units + gearing and READS THEM BACK (trap A)") {
    // BUG CAUGHT (mutation M5): trusting pros::Motor's leave-as-is defaults —
    // the shim's adversarial port state starts at rotations/red (what "a
    // different program on a different day" left), so an adapter that skips
    // configuration reads 1/360 positions. The ctor must configure AND verify.
    pros::shim::resetAll();
    CHECK(pros::shim::motorState(4).units == pros::v5::MotorUnits::rotations);  // adversarial start
    ProsMotor m{4, MotorGearset::Blue};
    CHECK(pros::shim::motorState(4).units == pros::v5::MotorUnits::degrees);
    CHECK(pros::shim::motorState(4).gearing == pros::v5::MotorGears::blue);
    // And the position path is degree-correct end to end:
    pros::shim::motorState(4).positionOutputDeg = 180.0;
    CHECK(m.position().value() == doctest::Approx(3.14159265358979312));
}

TEST_CASE("ProsMotor: position converts output-shaft degrees → cumulative radians") {
    // BUG CAUGHT (mutation M14): the adapter reading get_position() and
    // returning it raw (degrees labeled as radians) — a 57.3× odometry error
    // that no conversion-only test can see. Hand-computed: 720° = 4π =
    // 12.566370614359172 rad — and it must NOT wrap.
    pros::shim::resetAll();
    ProsMotor m{2, MotorGearset::Green};
    pros::shim::motorState(2).positionOutputDeg = 720.0;
    CHECK(m.position().value() == doctest::Approx(12.566370614359172));
}

TEST_CASE("ProsMotor: velocity converts RPM → rad/s; current mA → A (wired, not just defined)") {
    // BUG CAUGHT (mutations M2/M14): the ÷1000 or ×2π/60 existing in the
    // header but the adapter returning raw reads. 2500 mA as 2500 A makes
    // stall detection fire on a free wheel; 200 RPM as 200 rad/s makes the
    // feedforward think the wheel is 9.5× too fast.
    pros::shim::resetAll();
    ProsMotor m{3, MotorGearset::Green};
    pros::shim::motorState(3).velocityRpm = 200.0;
    pros::shim::motorState(3).currentMa = 2500;
    pros::shim::motorState(3).temperatureC = 43.5;
    CHECK(m.velocity().value() == doctest::Approx(20.943951023931955));
    CHECK(m.current().value() == doctest::Approx(2.5));
    CHECK(m.temperature() == doctest::Approx(43.5));
}

TEST_CASE("ProsMotor: brake modes map BOTH directions through the device") {
    // BUG CAUGHT: a one-way or crossed mapping (Coast→hold is the killer: the
    // guaranteed end-of-run park relies on Hold ≠ Coast, and a crossed map
    // parks as a freewheel on an incline). Set each canonical mode, verify the
    // DEVICE saw the right PROS enum, and read it back through the adapter.
    pros::shim::resetAll();
    ProsMotor m{5, MotorGearset::Green};

    m.setBrakeMode(BrakeMode::Hold);
    CHECK(pros::shim::motorState(5).brake == pros::v5::MotorBrake::hold);
    CHECK(m.brakeMode() == BrakeMode::Hold);

    m.setBrakeMode(BrakeMode::Brake);
    CHECK(pros::shim::motorState(5).brake == pros::v5::MotorBrake::brake);
    CHECK(m.brakeMode() == BrakeMode::Brake);

    m.setBrakeMode(BrakeMode::Coast);
    CHECK(pros::shim::motorState(5).brake == pros::v5::MotorBrake::coast);
    CHECK(m.brakeMode() == BrakeMode::Coast);
}

TEST_CASE("ProsMotor: sentinel reads hold the LAST GOOD value — never propagate, never zero (T7)") {
    // BUG CAUGHT (mutation M8): a PROS_ERR_F position reaching odometry (NaN
    // pose within one tick), or a screened read substituting 0 — which reads
    // as "the robot stopped", makes the dead-encoder runaway PLAUSIBLE, and
    // defeats the ODO_STUCK cross-check designed to catch exactly that.
    pros::shim::resetAll();
    ProsMotor m{6, MotorGearset::Green};
    pros::shim::motorState(6).positionOutputDeg = 90.0;
    pros::shim::motorState(6).velocityRpm = 60.0;
    pros::shim::motorState(6).currentMa = 800;
    const double goodPos = m.position().value();
    const double goodVel = m.velocity().value();
    const double goodCur = m.current().value();
    CHECK(goodPos == doctest::Approx(1.5707963267948966));

    pros::shim::motorState(6).disconnected = true;
    CHECK(m.position().value() == doctest::Approx(goodPos));  // held, not 0, not inf
    CHECK(m.velocity().value() == doctest::Approx(goodVel));
    CHECK(m.current().value() == doctest::Approx(goodCur));
    CHECK(m.faultedReads() == 3);  // and the screen is OBSERVABLE, not silent

    // Recovery: a good read resumes live values.
    pros::shim::motorState(6).disconnected = false;
    pros::shim::motorState(6).positionOutputDeg = 180.0;
    CHECK(m.position().value() == doctest::Approx(3.14159265358979312));
}

TEST_CASE("ProsMotor: a NEGATIVE port reverses through PROS — exactly once") {
    // BUG CAUGHT: the adapter negating on top of PROS's port-sign reversal —
    // a double negation reads forward again, and the "reversed" motor of a
    // mirrored drivetrain half fights the other three.
    pros::shim::resetAll();
    ProsMotor m{-7, MotorGearset::Green};
    m.setVoltage(Voltage{6.0});
    // The shim applies the port sign itself (as PROS does): the wire shows the
    // negated command, and the adapter did NOT add a second negation.
    CHECK(pros::shim::motorState(7).lastVoltageMv == -6000);
    pros::shim::motorState(7).positionOutputDeg = 90.0;
    CHECK(m.position().value() == doctest::Approx(-1.5707963267948966));
}

// Bug caught (DEFECTS1 items I9 + E1, reported twice from two passes): brakeMode() is the
// FIFTH screening site in this class and was the only one that did not increment
// faultedReads_. The counter's own doc says "how many reads were screened to last-good (T7
// observability): telemetry and the loop's health policy can see a flaky port" — false as
// written while one of five screens was silent, so a port failing only its brake-mode read
// looked perfectly healthy to the one channel that exists to notice.
TEST_CASE("I9/E1: a screened brake-mode read is COUNTED like the other four") {
    pros::shim::resetAll();
    ProsMotor m{5, MotorGearset::Green};
    REQUIRE(m.faultedReads() == 0);

    pros::shim::motorState(5).disconnected = true;
    const BrakeMode held = m.brakeMode();
    CHECK(held == BrakeMode::Coast);      // still holds the last commanded value
    CHECK(m.faultedReads() == 1);         // was 0 — the defect

    // NEGATIVE CONTROL: a healthy read neither screens nor counts.
    pros::shim::motorState(5).disconnected = false;
    m.setBrakeMode(BrakeMode::Hold);
    CHECK(m.brakeMode() == BrakeMode::Hold);
    CHECK(m.faultedReads() == 1);
}

// Bug caught (item E2): the ctor guarded the persistent-device-state trap for encoder units
// and gearing (set both, then SHULIB_PRECONDITION on the read-back, per trap A / HA-98) but
// left brake mode entirely inherited from whatever program last configured the port — and
// brakeMode_ is also the value brakeMode() falls back to when the device read is invalid. So
// a port left in Hold by a previous session, dying before any setBrakeMode(), reported Coast
// forever: the T7 fallback contradicting the device from boot.
TEST_CASE("E2: the T7 brake-mode fallback is seeded from the device, not from a guess") {
    pros::shim::resetAll();
    pros::shim::motorState(6).brake = ::pros::v5::MotorBrake::hold;  // left by another program

    ProsMotor m{6, MotorGearset::Green};
    pros::shim::motorState(6).disconnected = true;   // port dies before any command
    CHECK(m.brakeMode() == BrakeMode::Hold);             // was Coast — the guess

    // NEGATIVE CONTROL: with nothing inherited, the fallback is still Coast.
    pros::shim::resetAll();
    ProsMotor fresh{7, MotorGearset::Green};
    pros::shim::motorState(7).disconnected = true;
    CHECK(fresh.brakeMode() == BrakeMode::Coast);
}

// ═══ DEFECTS2 — a brake mode reaches the motor only through brake() ═════════════════════
// PROS applies a brake mode ONLY in brake(): move_voltage() "will not respect brake modes"
// (vendored motors.h:253-254). Until DEFECTS2 this adapter sent every zero as move_voltage(0),
// so a declared Hold went limp and the drive's "0 V + Brake" coasted on a V5 — and every host
// test passed, because the shim had no brake() and its `brake` field had nothing to do with how
// the motor was driven. The shim now carries a per-port CONTROL STATE (Voltage / Stopped, and the
// mode a stop was issued under), so each case below asserts the thing that reaches the motor —
// never the adapter's own brakeMode() echo, which read Hold all through the defect.

namespace {

using pros::shim::MotorControl;

/// Port `p` is STOPPED by brake() under `mode` — the only state in which PROS applies a mode.
bool stoppedUnder(int p, pros::v5::MotorBrake mode) {
    const auto& s = pros::shim::motorState(p);
    return s.control == MotorControl::Stopped && s.stoppedUnder == mode;
}

/// Port `p` is under VOLTAGE control at exactly 0 mV — the brake mode IGNORED (HA-142).
bool zeroVoltsModeIgnored(int p) {
    const auto& s = pros::shim::motorState(p);
    return s.control == MotorControl::Voltage && s.lastVoltageMv == 0;
}

}  // namespace

TEST_CASE("DEFECTS2 a1: Hold at 0 V reaches the motor as brake() — the stopped state, not 0 V") {
    // BUG CAUGHT (mutation "revert to plain move_voltage(0)"): the defect itself — a declared
    // Hold sent as a 0 V command whose mode PROS ignores. brakeMode() reads Hold either way, so
    // only the control state can tell the two apart.
    pros::shim::resetAll();
    ProsMotor m{6, MotorGearset::Green};
    m.setBrakeMode(BrakeMode::Hold);
    m.setVoltage(Voltage{8.0});
    const int movesBefore = pros::shim::motorState(6).moveVoltageCalls;
    m.setVoltage(Voltage{0.0});
    CHECK(stoppedUnder(6, pros::v5::MotorBrake::hold));
    CHECK(pros::shim::motorState(6).brakeCalls == 1);
    CHECK(pros::shim::motorState(6).moveVoltageCalls == movesBefore);  // no 0 V command beside it
    CHECK(m.commandedVoltage().value() == 0.0);  // the applied volts stay 0 V (brief item 1)
    CHECK(m.brakeMode() == BrakeMode::Hold);
}

TEST_CASE("DEFECTS2 a2: Brake at 0 V reaches the motor as brake() under Brake") {
    // BUG CAUGHT: the drive's cancel safe state ("0 V + Brake") coasting on a V5 — the same
    // defect for the other mode, and the one every library cancel path goes through.
    pros::shim::resetAll();
    ProsMotor m{11, MotorGearset::Blue};
    m.setVoltage(Voltage{-12.0});
    m.setBrakeMode(BrakeMode::Brake);
    m.setVoltage(Voltage{0.0});
    CHECK(stoppedUnder(11, pros::v5::MotorBrake::brake));
    CHECK(pros::shim::motorState(11).brakeCalls == 1);
}

TEST_CASE("DEFECTS2 a3 (the control): Coast at 0 V is STILL move_voltage(0) — every call, no "
          "brake()") {
    // BUG CAUGHT (mutation "drop the Coast branch" — Coast would brake): Calibrate, shulib Drive
    // and the tester's DRIVE station set Coast, and their MEASURED behaviour (HA-137) rests on
    // the 0 V path. Coast must send exactly what it sent before DEFECTS2: move_voltage(0) on
    // every zero command, and never brake().
    pros::shim::resetAll();
    ProsMotor m{3, MotorGearset::Green};
    m.setBrakeMode(BrakeMode::Coast);
    m.setVoltage(Voltage{5.0});
    const int movesBefore = pros::shim::motorState(3).moveVoltageCalls;
    for (int tick = 0; tick < 4; ++tick) {
        m.setVoltage(Voltage{0.0});
    }
    CHECK(zeroVoltsModeIgnored(3));
    CHECK(pros::shim::motorState(3).brakeCalls == 0);
    CHECK(pros::shim::motorState(3).moveVoltageCalls == movesBefore + 4);  // one per call
    // And re-stating Coast while at 0 V sends nothing beyond set_brake_mode (pre-DEFECTS2 traffic).
    m.setBrakeMode(BrakeMode::Coast);
    CHECK(pros::shim::motorState(3).moveVoltageCalls == movesBefore + 4);
    CHECK(pros::shim::motorState(3).brakeCalls == 0);
}

TEST_CASE("DEFECTS2 a4: a non-zero command after a hold LEAVES the stop — and the next zero "
          "stops again") {
    // BUG CAUGHT: a stop that latches — a held claw that will not move when commanded, or (the
    // mirror) an adapter that believes it is still stopped after driving and so never re-stops.
    pros::shim::resetAll();
    ProsMotor m{7, MotorGearset::Red};
    m.setBrakeMode(BrakeMode::Hold);
    m.setVoltage(Voltage{0.0});
    REQUIRE(stoppedUnder(7, pros::v5::MotorBrake::hold));
    m.setVoltage(Voltage{6.0});
    CHECK(pros::shim::motorState(7).control == MotorControl::Voltage);
    CHECK(pros::shim::motorState(7).lastVoltageMv == 6000);
    m.setVoltage(Voltage{0.0});
    CHECK(stoppedUnder(7, pros::v5::MotorBrake::hold));
    CHECK(pros::shim::motorState(7).brakeCalls == 2);  // one stop per transition into zero
}

TEST_CASE("DEFECTS2 a5: the ORDER of mode and zero never matters — Hold then 0 V, 0 V then Hold") {
    // BUG CAUGHT (mutation "drop the re-issue in setBrakeMode"): a caller that commands 0 V
    // first and declares the mode second (a mechanism stopped, THEN told to hold) — the mode
    // lands in the device's register but nothing applies it: the shaft stays at 0 V, limp.
    pros::shim::resetAll();
    SUBCASE("mode, then zero") {
        ProsMotor m{8, MotorGearset::Green};
        m.setVoltage(Voltage{4.0});
        m.setBrakeMode(BrakeMode::Hold);
        m.setVoltage(Voltage{0.0});
        CHECK(stoppedUnder(8, pros::v5::MotorBrake::hold));
        CHECK(pros::shim::motorState(8).brakeCalls == 1);
    }
    SUBCASE("zero, then mode — the stop is re-issued UNDER the new mode") {
        ProsMotor m{8, MotorGearset::Green};
        m.setBrakeMode(BrakeMode::Coast);
        m.setVoltage(Voltage{4.0});
        m.setVoltage(Voltage{0.0});
        REQUIRE(zeroVoltsModeIgnored(8));
        m.setBrakeMode(BrakeMode::Hold);
        CHECK(stoppedUnder(8, pros::v5::MotorBrake::hold));  // brake() came AFTER set_brake_mode
        CHECK(pros::shim::motorState(8).brakeCalls == 1);
    }
    SUBCASE("stopped under Brake, then Hold: re-stopped under Hold") {
        ProsMotor m{8, MotorGearset::Green};
        m.setBrakeMode(BrakeMode::Brake);
        m.setVoltage(Voltage{0.0});
        REQUIRE(stoppedUnder(8, pros::v5::MotorBrake::brake));
        m.setBrakeMode(BrakeMode::Hold);
        CHECK(stoppedUnder(8, pros::v5::MotorBrake::hold));
        CHECK(pros::shim::motorState(8).brakeCalls == 2);
    }
    SUBCASE("stopped under Hold, then Coast: back to 0 V, as Coast has always meant") {
        ProsMotor m{8, MotorGearset::Green};
        m.setBrakeMode(BrakeMode::Hold);
        m.setVoltage(Voltage{0.0});
        REQUIRE(stoppedUnder(8, pros::v5::MotorBrake::hold));
        m.setBrakeMode(BrakeMode::Coast);
        CHECK(zeroVoltsModeIgnored(8));
        CHECK(pros::shim::motorState(8).brakeCalls == 1);
    }
    SUBCASE("a mode change while DRIVING issues no stop") {
        ProsMotor m{8, MotorGearset::Green};
        m.setVoltage(Voltage{4.0});
        m.setBrakeMode(BrakeMode::Hold);
        CHECK(pros::shim::motorState(8).control == MotorControl::Voltage);
        CHECK(pros::shim::motorState(8).lastVoltageMv == 4000);
        CHECK(pros::shim::motorState(8).brakeCalls == 0);
    }
}

TEST_CASE("DEFECTS2 a6: the decision is keyed on the MILLIVOLTS SENT — a sub-millivolt command "
          "is a stop") {
    // BUG CAUGHT (mutation "key on the raw double"): 0.0004 V rounds to 0 mV on the wire — a
    // command indistinguishable from zero at the motor — but is not == 0.0. Keyed on the double,
    // it would go out as move_voltage(0) with the mode ignored: a controller that settles to
    // within half a millivolt of zero would never hold. And the converse: 0.0006 V is 1 mV, a
    // DRIVING command, and must not stop.
    pros::shim::resetAll();
    ProsMotor m{9, MotorGearset::Green};
    m.setBrakeMode(BrakeMode::Hold);
    m.setVoltage(Voltage{3.0});
    m.setVoltage(Voltage{0.0004});
    CHECK(stoppedUnder(9, pros::v5::MotorBrake::hold));
    CHECK(m.commandedVoltage().value() == doctest::Approx(0.0004));  // APPLIED volts, unrounded
    m.setVoltage(Voltage{0.0006});
    CHECK(pros::shim::motorState(9).control == MotorControl::Voltage);
    CHECK(pros::shim::motorState(9).lastVoltageMv == 1);
    m.setVoltage(Voltage{-0.0004});
    CHECK(stoppedUnder(9, pros::v5::MotorBrake::hold));
    CHECK(pros::shim::motorState(9).brakeCalls == 2);
}

TEST_CASE("DEFECTS2 a7 (the seeded-mode trap): a port the DEVICE left in Hold, never given a mode "
          "by this program, does NOT brake at zero") {
    // BUG CAUGHT (mutation "key on the device-seeded brakeMode_"): the ctor seeds brakeMode_ from
    // the device for the T7 fallback (E2). A drive motor an earlier program left in Hold would
    // then brake() at centred sticks in a program that never asked for Hold. The stop keys on
    // what THIS program commanded; with nothing commanded a zero stays move_voltage(0), exactly
    // as before DEFECTS2.
    pros::shim::resetAll();
    pros::shim::motorState(12).brake = pros::v5::MotorBrake::hold;  // left by another program
    ProsMotor m{12, MotorGearset::Blue};
    REQUIRE(m.brakeMode() == BrakeMode::Hold);  // the seed is real — the trap is armed
    m.setVoltage(Voltage{6.0});
    m.setVoltage(Voltage{0.0});
    CHECK(pros::shim::motorState(12).brakeCalls == 0);
    CHECK(zeroVoltsModeIgnored(12));
    // Once THIS program says Hold, the same zero holds.
    m.setBrakeMode(BrakeMode::Hold);
    CHECK(stoppedUnder(12, pros::v5::MotorBrake::hold));
}

TEST_CASE("DEFECTS2 a8: a zero REPEATED under the same mode sends no second brake() — the stop is "
          "issued on the transition, never every tick") {
    // BUG CAUGHT (mutation "brake() on every zero"): landmine 2. A live operation re-sends its
    // volts every tick, and DriveBrake re-commands Brake + 0 V every tick; a brake() per tick
    // under Hold may re-capture the held position each time and let a loaded shaft creep
    // (HA-144, invented). Ten ticks of the DriveBrake pattern: ONE brake().
    pros::shim::resetAll();
    ProsMotor m{13, MotorGearset::Green};
    m.setVoltage(Voltage{9.0});
    for (int tick = 0; tick < 10; ++tick) {
        m.setBrakeMode(BrakeMode::Hold);
        m.setVoltage(Voltage{0.0});
    }
    CHECK(stoppedUnder(13, pros::v5::MotorBrake::hold));
    CHECK(pros::shim::motorState(13).brakeCalls == 1);
    CHECK(pros::shim::motorState(13).setBrakeModeCalls == 10);  // the mode itself: as before
}

TEST_CASE("DEFECTS2 a9: a brake() the port REFUSES is not believed — the next zero retries it") {
    // BUG CAUGHT (mutation "record the stop whatever brake() returned"): a port that drops for
    // one tick answers PROS_ERR; believing the stop happened would make every later zero send
    // nothing, and the motor would come back from the dropout un-stopped.
    pros::shim::resetAll();
    ProsMotor m{14, MotorGearset::Green};
    m.setBrakeMode(BrakeMode::Hold);
    m.setVoltage(Voltage{5.0});
    pros::shim::motorState(14).disconnected = true;
    m.setVoltage(Voltage{0.0});
    CHECK(pros::shim::motorState(14).brakeCalls == 0);  // refused
    pros::shim::motorState(14).disconnected = false;
    m.setVoltage(Voltage{0.0});
    CHECK(stoppedUnder(14, pros::v5::MotorBrake::hold));
    CHECK(pros::shim::motorState(14).brakeCalls == 1);
}

TEST_CASE("DEFECTS2 a9b: a set_brake_mode() the port REFUSES is not believed either — brake() "
          "stops under the device's REGISTER, so the mode is re-sent before it") {
    // BUG CAUGHT (review finding; mutations "drop the mode re-send" and "believe the stop whatever
    // the mode write returned"): brake() applies the register, not what this program meant. A
    // mode write lost to a dropout, followed by a zero once the port is back, used to stop the
    // motor under the register's OLD mode (Coast here) and record it as Hold — after which every
    // zero sent nothing: the claw limp while the adapter believed it held.
    pros::shim::resetAll();  // a fresh port's register is Coast
    ProsMotor m{14, MotorGearset::Green};
    m.setVoltage(Voltage{5.0});
    SUBCASE("the dropout spans setBrakeMode(); the port is back for the zero") {
        pros::shim::motorState(14).disconnected = true;
        m.setBrakeMode(BrakeMode::Hold);  // refused: the register stays Coast
        pros::shim::motorState(14).disconnected = false;
        REQUIRE(pros::shim::motorState(14).brake == pros::v5::MotorBrake::coast);
        m.setVoltage(Voltage{0.0});  // ONE zero — the one-shot caller's whole chance
        CHECK(pros::shim::motorState(14).brake == pros::v5::MotorBrake::hold);  // re-sent first
        CHECK(stoppedUnder(14, pros::v5::MotorBrake::hold));
        CHECK(pros::shim::motorState(14).setBrakeModeCalls == 1);
        CHECK(pros::shim::motorState(14).brakeCalls == 1);
        m.setVoltage(Voltage{0.0});  // and now it IS believed: a repeat sends nothing
        CHECK(pros::shim::motorState(14).brakeCalls == 1);
        CHECK(pros::shim::motorState(14).setBrakeModeCalls == 1);
    }
    SUBCASE("the mode write refused twice, brake() accepted: stopped, NOT believed, retried") {
        pros::shim::motorState(14).refuseSetBrakeMode = 2;  // setBrakeMode() and the re-send
        m.setBrakeMode(BrakeMode::Hold);
        m.setVoltage(Voltage{0.0});
        // The motor did stop — under the register's Coast, which is not what was asked.
        CHECK(stoppedUnder(14, pros::v5::MotorBrake::coast));
        CHECK(pros::shim::motorState(14).brakeCalls == 1);
        m.setVoltage(Voltage{0.0});  // the next zero retries BOTH calls
        CHECK(stoppedUnder(14, pros::v5::MotorBrake::hold));
        CHECK(pros::shim::motorState(14).brakeCalls == 2);
        m.setVoltage(Voltage{0.0});
        CHECK(pros::shim::motorState(14).brakeCalls == 2);  // believed only now
    }
    SUBCASE("the control: a mode write that was accepted is never re-sent by the stop") {
        m.setBrakeMode(BrakeMode::Hold);
        m.setVoltage(Voltage{0.0});
        CHECK(stoppedUnder(14, pros::v5::MotorBrake::hold));
        CHECK(pros::shim::motorState(14).setBrakeModeCalls == 1);  // the pre-review traffic
    }
}

TEST_CASE("DEFECTS2 a10: a non-finite command under Hold is still REJECTED and touches nothing") {
    // BUG CAUGHT: the new zero path swallowing the L4 rejection (a NaN "rounding" to a stop).
    pros::shim::resetAll();
    ProsMotor m{15, MotorGearset::Green};
    m.setBrakeMode(BrakeMode::Hold);
    m.setVoltage(Voltage{5.0});
    const double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK_THROWS_AS(m.setVoltage(Voltage{nan}), PreconditionError);
    CHECK(pros::shim::motorState(15).brakeCalls == 0);
    CHECK(pros::shim::motorState(15).lastVoltageMv == 5000);
    CHECK(m.commandedVoltage().value() == doctest::Approx(5.0));
}

TEST_CASE("DEFECTS2 a11: a DRIVING command clears the zero record — a mode stated after zero, "
          "then drive, issues no stop") {
    // BUG CAUGHT (review finding; mutation "atZero_ set only inside the zero branch" — a record
    // that sticks after the FIRST stop): a5's "mode change while driving" starts from an adapter
    // that was never at zero, so it could not see it. With the record stuck, every setBrakeMode()
    // to Brake or Hold after one stop re-issues a stop over a RUNNING motor — under a Brake or
    // Hold release mode the driver loop's per-tick re-assert would stop the drive dead on every
    // driving tick, and a mechanism re-stating its mode mid-motion would stop.
    pros::shim::resetAll();
    ProsMotor m{10, MotorGearset::Green};
    m.setBrakeMode(BrakeMode::Hold);
    m.setVoltage(Voltage{0.0});
    REQUIRE(stoppedUnder(10, pros::v5::MotorBrake::hold));
    m.setVoltage(Voltage{6.0});
    REQUIRE(pros::shim::motorState(10).control == MotorControl::Voltage);
    const int movesBefore = pros::shim::motorState(10).moveVoltageCalls;
    SUBCASE("the same mode re-stated (the driver loop's re-assert)") {
        m.setBrakeMode(BrakeMode::Hold);
    }
    SUBCASE("another stopping mode") {
        m.setBrakeMode(BrakeMode::Brake);
    }
    SUBCASE("Coast") {
        m.setBrakeMode(BrakeMode::Coast);
    }
    CHECK(pros::shim::motorState(10).control == MotorControl::Voltage);
    CHECK(pros::shim::motorState(10).lastVoltageMv == 6000);
    CHECK(pros::shim::motorState(10).brakeCalls == 1);  // the first stop only
    CHECK(pros::shim::motorState(10).moveVoltageCalls == movesBefore);  // no 0 V either
}
