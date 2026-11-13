// sim/mechanism_plant.hpp — the host test plant's mechanisms (chunk MECH-A2, the MECH brief's §1.12:
// the scaffolding; MECH-A3 gave the joint its two homing sensors and a wedge — the cases at the end). Every constant here is INVENTED (HA-183); these cases pin that
// the MODEL does what its header says, each against a number derived by hand from the header's
// equations — never against the plant's own output — so a plant that drifted from its stated
// physics turns a case red before it can make a behaviour test pass for the wrong reason.

#include "doctest.h"

#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <stdexcept>
#include <string>

#include "shulib/core/check.hpp"
#include "shulib/hal/fake/fake_digital_in.hpp"
#include "shulib/hal/fake/fake_motor.hpp"
#include "shulib/hal/fake/fake_rotation.hpp"
#include "shulib/sim/mechanism_plant.hpp"
#include "shulib/units/quantity.hpp"

namespace {
namespace hal = shulib::hal;
using shulib::sim::ArmPlant;
using shulib::sim::ArmPlantConfig;
using shulib::sim::FlywheelPlant;
using shulib::sim::FlywheelPlantConfig;
using shulib::units::Voltage;
constexpr double kPi = 3.14159265358979323846;
constexpr double kD2R = kPi / 180.0;

ArmPlantConfig armCfg() {
    ArmPlantConfig c;
    c.gearRatio = 5.0;
    c.kS = 0.3;
    c.kV = 1.2;
    c.kA = 0.0;
    c.kG = 0.0;
    c.minStopDeg = -10.0;
    c.maxStopDeg = 170.0;
    return c;
}
}  // namespace

TEST_CASE("MECH-A2 plant: a free joint, kA 0 -- speed (V - kS)/kV at once; motor readings through the gearing; current from the back-EMF") {
    hal::fake::FakeMotor m1, m2;
    std::array<hal::fake::FakeMotor*, 2> ms{&m1, &m2};
    ArmPlant p{ms, armCfg()};
    m1.setVoltage(Voltage{6.0});  // the plant reads motor 0's command (a group is commanded alike)
    p.step(0.01);
    const double w = (6.0 - 0.3) / 1.2;  // joint rad/s
    CHECK(p.trueSpeedDegS() == doctest::Approx(w / kD2R));
    CHECK(p.trueDeg() == doctest::Approx(w * 0.01 / kD2R));
    for (const hal::fake::FakeMotor* m : ms) {  // EVERY motor of the group is published alike
        CHECK(m->velocity().value() == doctest::Approx(w * 5.0));
        CHECK(m->position().value() == doctest::Approx(w * 0.01 * 5.0));
        CHECK(m->current().value() == doctest::Approx(std::abs(6.0 - 1.2 * w) * 2.5 / 12.0));  // = kS's share
    }
    CHECK(p.appliedVolts() == doctest::Approx(6.0));
    // inside the static band: no motion
    m1.setVoltage(Voltage{-0.25});
    const double before = p.trueDeg();
    p.step(0.01);
    CHECK(p.trueDeg() == doctest::Approx(before));
    CHECK(p.trueSpeedDegS() == 0.0);
}

TEST_CASE("MECH-A2 plant: kA > 0 -- the EXACT first-order step toward the steady speed, tau = kA/kV") {
    hal::fake::FakeMotor m;
    std::array<hal::fake::FakeMotor*, 1> ms{&m};
    ArmPlantConfig c = armCfg();
    c.kA = 0.12;  // tau 0.1 s
    ArmPlant p{ms, c};
    m.setVoltage(Voltage{12.0});
    const double oss = (12.0 - 0.3) / 1.2;
    p.step(0.01);
    const double w1 = oss * (1.0 - std::exp(-0.1));
    CHECK(p.trueSpeedDegS() == doctest::Approx(w1 / kD2R));
    p.step(0.02);
    const double w2 = oss + (w1 - oss) * std::exp(-0.2);
    CHECK(p.trueSpeedDegS() == doctest::Approx(w2 / kD2R));
}

TEST_CASE("MECH-A2 plant: a HARD STOP -- the joint stops AT it, reads zero speed and stall-grade current pushing in, both ends") {
    hal::fake::FakeMotor m;
    std::array<hal::fake::FakeMotor*, 1> ms{&m};
    ArmPlant p{ms, armCfg()};
    m.setVoltage(Voltage{12.0});
    for (int i = 0; i < 300; ++i) {
        p.step(0.01);
    }
    CHECK(p.trueDeg() == doctest::Approx(170.0));
    CHECK(p.trueSpeedDegS() == 0.0);
    CHECK(m.velocity().value() == 0.0);
    CHECK(m.current().value() == doctest::Approx(2.5));  // |12 - 0| * 2.5/12, clamped at 2.5
    m.setVoltage(Voltage{6.0});
    p.step(0.01);
    CHECK(m.current().value() == doctest::Approx(1.25));
    m.setVoltage(Voltage{-12.0});
    for (int i = 0; i < 400; ++i) {
        p.step(0.01);
    }
    CHECK(p.trueDeg() == doctest::Approx(-10.0));
    CHECK(p.trueSpeedDegS() == 0.0);
    CHECK(m.current().value() == doctest::Approx(2.5));
    // pulling AWAY from a stop moves at once
    m.setVoltage(Voltage{12.0});
    p.step(0.01);
    CHECK(p.trueSpeedDegS() > 0.0);
}

TEST_CASE("MECH-A2 plant: GRAVITY as a load in volts -- an arm's cosine, a lift's constant, and a piece's extra load") {
    hal::fake::FakeMotor m;
    std::array<hal::fake::FakeMotor*, 1> ms{&m};
    ArmPlantConfig c = armCfg();
    c.kG = 2.0;
    c.levelDeg = 45.0;
    c.startDeg = 45.0;  // level: the full load
    ArmPlant arm{ms, c};
    m.setVoltage(Voltage{2.0});  // exactly the load: inside the static band -> holds
    arm.step(0.01);
    CHECK(arm.trueSpeedDegS() == 0.0);
    m.setVoltage(Voltage{1.0});  // less than the load by more than kS: falls at (1 - 2 + 0.3)/1.2
    arm.step(0.01);
    CHECK(arm.trueSpeedDegS() == doctest::Approx(((1.0 - 2.0) + 0.3) / 1.2 / kD2R));
    // 90 degrees from level: the cosine is 0 -- no load
    hal::fake::FakeMotor m2;
    std::array<hal::fake::FakeMotor*, 1> ms2{&m2};
    c.startDeg = 135.0;
    ArmPlant up{ms2, c};
    m2.setVoltage(Voltage{1.0});
    up.step(0.01);
    CHECK(up.trueSpeedDegS() == doctest::Approx((1.0 - 0.3) / 1.2 / kD2R).epsilon(1e-9));
    // a LIFT: the same load anywhere; a piece adds to it
    hal::fake::FakeMotor m3;
    std::array<hal::fake::FakeMotor*, 1> ms3{&m3};
    ArmPlantConfig l = armCfg();
    l.kG = 1.0;
    l.constantGravity = true;
    l.startDeg = 100.0;
    ArmPlant lift{ms3, l};
    m3.setVoltage(Voltage{1.2});  // 1.2 - 1.0 = 0.2 <= kS: holds
    lift.step(0.01);
    CHECK(lift.trueSpeedDegS() == 0.0);
    lift.addLoadVolts(0.8);  // 1.2 - 1.8 = -0.6: falls at (-0.6 + 0.3)/1.2
    lift.step(0.01);
    CHECK(lift.trueSpeedDegS() == doctest::Approx(-0.3 / 1.2 / kD2R));
}

TEST_CASE("MECH-A2 plant: the BRAKE MODE at 0 V -- Hold restores toward where the command stopped (and draws current); Brake damps; Coast falls") {
    const auto settle = [](hal::BrakeMode mode) {
        hal::fake::FakeMotor m;
        std::array<hal::fake::FakeMotor*, 1> ms{&m};
        ArmPlantConfig c = armCfg();
        c.kG = 2.0;
        c.levelDeg = 90.0;
        c.startDeg = 90.0;  // level: the full 2 V load
        ArmPlant p{ms, c};
        m.setVoltage(Voltage{2.0});
        p.step(0.01);  // held by the command: the hold point captured at 90
        m.setBrakeMode(mode);
        m.setVoltage(Voltage{0.0});
        double minDeg = 90.0;
        for (int i = 0; i < 300; ++i) {
            p.step(0.01);
            minDeg = std::min(minDeg, p.trueDeg());
        }
        struct Out {
            double deg, minDeg, applied, amps;
        };
        return Out{p.trueDeg(), minDeg, p.appliedVolts(), m.current().value()};
    };
    const auto hold = settle(hal::BrakeMode::Hold);
    // Hold: it sags until the restoring volts carry the load within the static band:
    // stiffness*(sag rad) >= kG - kS  ->  about (2.0 - 0.3)/40 rad = 2.4 deg
    CHECK(90.0 - hold.deg == doctest::Approx((2.0 - 0.3) / 40.0 / kD2R).epsilon(0.02));
    CHECK(hold.applied > 1.6);        // the motor APPLIES its hold volts ...
    CHECK(hold.amps > 0.3);           // ... and draws current for them
    // Coast: free, it falls until the load itself is inside the static band -- the load is
    // kG*cos(theta - 90), so it never reaches the stop at -10: it creeps toward
    // theta* = 90 - acos(kS/kG) = 8.63 deg from above (the load there is exactly kS)
    const double rest = 90.0 - std::acos(0.3 / 2.0) / kD2R;
    const auto coast = settle(hal::BrakeMode::Coast);
    CHECK(coast.deg > rest);
    CHECK(coast.deg < rest + 1.5);
    CHECK(coast.minDeg > rest);  // never past it
    CHECK(coast.applied == 0.0);
    // Brake: the same rest angle, approached SLOWER (damping B adds to kV): after 3 s it is
    // still above where Coast got to, and it never passed the rest angle either
    const auto brake = settle(hal::BrakeMode::Brake);
    CHECK(brake.deg < 89.0);       // it falls ...
    CHECK(brake.deg > coast.deg);  // ... slower than coasting
    CHECK(brake.minDeg > rest);
}

TEST_CASE("MECH-A2 plant: Hold and Brake are IMPLICIT -- a FAST memoryless joint never rings (the explicit form flipped its speed every step)") {
    // a Blue-ish joint, gear 1: kV 0.19 V per rad/s. Explicitly, Brake's gain B/kV = 15.6 and Hold's
    // S*dt/kV = 2.1 -- both above 1, so "volts from the last step" flipped the speed's sign each step.
    for (const hal::BrakeMode mode : {hal::BrakeMode::Hold, hal::BrakeMode::Brake}) {
        CAPTURE(static_cast<int>(mode));
        hal::fake::FakeMotor m;
        std::array<hal::fake::FakeMotor*, 1> ms{&m};
        ArmPlantConfig c = armCfg();
        c.gearRatio = 1.0;
        c.kV = 12.0 / 62.8;
        c.kS = 0.1;
        c.kG = 1.0;
        c.levelDeg = 90.0;
        c.startDeg = 60.0;
        ArmPlant p{ms, c};
        m.setVoltage(Voltage{6.0});
        for (int i = 0; i < 3; ++i) {
            p.step(0.01);  // moving up fast: the hold point is captured mid-motion
        }
        REQUIRE(p.trueSpeedDegS() > 100.0);
        m.setBrakeMode(mode);
        m.setVoltage(Voltage{0.0});
        int signFlips = 0;
        double last = p.trueSpeedDegS();
        double fastest = 0.0;  // after the first step at 0 V
        for (int i = 0; i < 300; ++i) {
            p.step(0.01);
            const double w = p.trueSpeedDegS();
            if ((w > 0.0 && last < 0.0) || (w < 0.0 && last > 0.0)) {
                ++signFlips;
            }
            if (i > 0) {
                fastest = std::max(fastest, std::abs(w));
            }
            last = w;
            CHECK(std::abs(p.appliedVolts()) <= 12.0);
        }
        CHECK(signFlips <= 1);  // at most one reversal (it was moving UP when the volts stopped), never a ring
        if (mode == hal::BrakeMode::Hold) {
            // Hold: at rest near its hold point (the sag that carries the load: (kG - kS)/S rad at most)
            CHECK(std::abs(p.trueSpeedDegS()) < 1.0);
        } else {
            // Brake: still falling (Brake is not Hold), but never faster than the damped steady speed
            // (kG - kS)/(kV + B) -- the explicit form swung between the +-12 V clamps instead
            CHECK(p.trueSpeedDegS() < 0.0);
            CHECK(fastest <= (1.0 - 0.1) / (c.kV + c.brakeDamping) / kD2R + 1e-9);
        }
    }
}

TEST_CASE("MECH-A2 plant: Hold holds where the command STOPPED -- the angle at the end of the last commanded step, not one step before it") {
    // A FAST joint (gear 1, 600 rpm free, kA 0, no gravity): one 10 ms step at 6 V moves it ~17.6 deg.
    // A Hold captured BEFORE the step integrated held one step behind and drove the joint back that far
    // the moment the command stopped (the MECH-A2 review's probe: 17.55 deg at 6 V, 35.54 at 12 V).
    for (const double volts : {6.0, 12.0, -9.0}) {
        CAPTURE(volts);
        hal::fake::FakeMotor m;
        std::array<hal::fake::FakeMotor*, 1> ms{&m};
        ArmPlantConfig c;
        c.gearRatio = 1.0;
        c.kS = 0.1;
        c.kV = 12.0 / 62.8;
        c.kA = 0.0;
        c.kG = 0.0;  // no gravity: no sag — where it stops is where Hold must keep it
        c.startDeg = 0.0;
        c.minStopDeg = -720.0;
        c.maxStopDeg = 720.0;
        ArmPlant p{ms, c};
        m.setBrakeMode(hal::BrakeMode::Hold);
        m.setVoltage(Voltage{volts});
        for (int i = 0; i < 20; ++i) {
            p.step(0.01);
        }
        const double stoppedAt = p.trueDeg();
        const double travelPerStep = std::abs(p.trueSpeedDegS()) * 0.01;
        REQUIRE(travelPerStep > 10.0);  // fast enough that one step of lag is unmistakable
        m.setVoltage(Voltage{0.0});
        for (int i = 0; i < 300; ++i) {
            p.step(0.01);
        }
        CHECK(p.trueDeg() == doctest::Approx(stoppedAt).epsilon(1e-9));
        CHECK(std::abs(p.trueDeg() - stoppedAt) < 0.01 * travelPerStep);
        CHECK(p.trueSpeedDegS() == doctest::Approx(0.0));
    }
}

TEST_CASE("MECH-A2 plant: an OBSTACLE blocks the joint from below; a JAM pins it both ways; a joint above the obstacle is free") {
    hal::fake::FakeMotor m;
    std::array<hal::fake::FakeMotor*, 1> ms{&m};
    ArmPlant p{ms, armCfg()};
    p.setObstacleDeg(60.0);
    m.setVoltage(Voltage{12.0});
    for (int i = 0; i < 200; ++i) {
        p.step(0.01);
    }
    CHECK(p.trueDeg() == doctest::Approx(60.0));
    CHECK(m.current().value() == doctest::Approx(2.5));
    p.clearObstacle();
    p.step(0.01);
    CHECK(p.trueDeg() > 60.0);
    // the jam: both directions, current from the applied volts
    p.setJammed(true);
    const double at = p.trueDeg();
    m.setVoltage(Voltage{-9.0});
    for (int i = 0; i < 10; ++i) {
        p.step(0.01);
    }
    CHECK(p.trueDeg() == doctest::Approx(at));
    CHECK(m.velocity().value() == 0.0);
    CHECK(m.current().value() == doctest::Approx(9.0 * 2.5 / 12.0));
    p.setJammed(false);
    // above an obstacle set below it: free (it only blocks from below)
    p.setObstacleDeg(at - 20.0);
    m.setVoltage(Voltage{6.0});
    p.step(0.01);
    CHECK(p.trueDeg() > at);
}

TEST_CASE("MECH-A2 plant: the encoders read zero at encoderZeroDeg; an attached rotation sensor reads the ABSOLUTE joint angle") {
    hal::fake::FakeMotor m;
    std::array<hal::fake::FakeMotor*, 1> ms{&m};
    hal::fake::FakeRotation rot;
    ArmPlantConfig c = armCfg();
    c.startDeg = 30.0;
    c.encoderZeroDeg = 30.0;  // the encoders read 0 where the joint started
    ArmPlant p{ms, c};
    p.attachRotation(&rot);
    CHECK(m.position().value() == doctest::Approx(0.0));
    CHECK(rot.position().value() == doctest::Approx(30.0 * kD2R));
    m.setVoltage(Voltage{6.0});
    p.step(0.01);
    CHECK(m.position().value() == doctest::Approx((p.trueDeg() - 30.0) * kD2R * 5.0));
    CHECK(rot.position().value() == doctest::Approx(p.trueDeg() * kD2R));
    CHECK(rot.velocity().value() == doctest::Approx(p.trueSpeedDegS() * kD2R));
}

TEST_CASE("MECH-A2 plant: a FLYWHEEL -- its steady speed, a shot, a jam's stall current, Brake vs Coast at 0 V") {
    hal::fake::FakeMotor m;
    std::array<hal::fake::FakeMotor*, 1> ms{&m};
    FlywheelPlantConfig c;
    c.gearRatio = 1.0 / 6.0;
    c.kS = 0.4;
    c.kV = 0.02;
    c.kA = 0.004;
    FlywheelPlant f{ms, c};
    m.setVoltage(Voltage{12.0});
    for (int i = 0; i < 300; ++i) {  // 15 time constants
        f.step(0.01);
    }
    const double oss = (12.0 - 0.4) / 0.02;  // wheel rad/s
    CHECK(f.trueRpm() == doctest::Approx(oss * 60.0 / (2.0 * kPi)).epsilon(1e-6));
    CHECK(m.velocity().value() == doctest::Approx(oss / 6.0).epsilon(1e-6));
    f.shot(0.2);
    CHECK(f.trueRpm() == doctest::Approx(0.8 * oss * 60.0 / (2.0 * kPi)).epsilon(1e-6));
    f.setJammed(true);
    f.step(0.01);
    CHECK(f.trueRpm() == 0.0);
    CHECK(m.current().value() == doctest::Approx(2.5));
    f.setJammed(false);
    // spin up again, then 0 V: Brake slows it faster than Coast
    const auto decay = [&](hal::BrakeMode mode) {
        hal::fake::FakeMotor mm;
        std::array<hal::fake::FakeMotor*, 1> one{&mm};
        FlywheelPlant w{one, c};
        mm.setVoltage(Voltage{12.0});
        for (int i = 0; i < 300; ++i) {
            w.step(0.01);
        }
        mm.setBrakeMode(mode);
        mm.setVoltage(Voltage{0.0});
        for (int i = 0; i < 20; ++i) {
            w.step(0.01);
        }
        return w.trueRpm();
    };
    CHECK(decay(hal::BrakeMode::Brake) < decay(hal::BrakeMode::Coast));
    // HOLD at 0 V brakes a wheel too (the header: "Brake or Hold" — a wheel has no position to hold),
    // exactly as Brake does: MECH-A3's flywheel tuning reads this
    CHECK(decay(hal::BrakeMode::Hold) < decay(hal::BrakeMode::Coast));
    CHECK(decay(hal::BrakeMode::Hold) == doctest::Approx(decay(hal::BrakeMode::Brake)));
    CHECK(decay(hal::BrakeMode::Coast) > 0.0);
}

TEST_CASE("MECH-A2 plant: the preconditions -- no motors, a null motor, a nonsense constant, stops out of order, dt <= 0") {
    hal::fake::FakeMotor m;
    std::array<hal::fake::FakeMotor*, 1> ms{&m};
    std::array<hal::fake::FakeMotor*, 1> nul{nullptr};
    CHECK_THROWS_AS(ArmPlant(std::span<hal::fake::FakeMotor* const>{}, armCfg()), shulib::PreconditionError);
    CHECK_THROWS_AS(ArmPlant(nul, armCfg()), shulib::PreconditionError);
    ArmPlantConfig bad = armCfg();
    bad.kV = 0.0;
    CHECK_THROWS_AS(ArmPlant(ms, bad), shulib::PreconditionError);
    bad = armCfg();
    bad.kA = -1.0;
    CHECK_THROWS_AS(ArmPlant(ms, bad), shulib::PreconditionError);
    bad = armCfg();
    bad.minStopDeg = 200.0;
    CHECK_THROWS_AS(ArmPlant(ms, bad), shulib::PreconditionError);
    bad = armCfg();
    bad.startDeg = 171.0;
    CHECK_THROWS_AS(ArmPlant(ms, bad), shulib::PreconditionError);
    ArmPlant ok{ms, armCfg()};
    CHECK_THROWS_AS(ok.step(0.0), shulib::PreconditionError);
    CHECK_THROWS_AS(ok.step(std::nan("")), shulib::PreconditionError);
    FlywheelPlantConfig fb;
    fb.gearRatio = 0.0;
    CHECK_THROWS_AS(FlywheelPlant(ms, fb), shulib::PreconditionError);
    FlywheelPlant fw{ms, FlywheelPlantConfig{}};
    CHECK_THROWS_AS(fw.step(-0.01), shulib::PreconditionError);
}

TEST_CASE("MECH-A3 plant: an angle sensor MOUNTED -- absolute within one turn at power-up (HA-186's belief), cumulative after, either way round") {
    hal::fake::FakeMotor m;
    std::array<hal::fake::FakeMotor*, 1> ms{&m};
    ArmPlantConfig c = armCfg();
    c.startDeg = 20.0;
    ArmPlant p{ms, c};
    hal::fake::FakeRotation fwd;
    hal::fake::FakeRotation rev;
    // reads 350 at the joint's 0: at 20 it reads 370 -> wrapped into ONE turn, 10
    p.attachRotation(&fwd, shulib::sim::RotationMount{.readsAtJointZeroDeg = 350.0});
    CHECK(fwd.position().value() / kD2R == doctest::Approx(10.0));
    m.setVoltage(Voltage{6.0});
    p.step(0.1);
    const double moved = p.trueDeg() - 20.0;
    REQUIRE(moved > 0.0);
    CHECK(fwd.position().value() / kD2R == doctest::Approx(10.0 + moved));  // CUMULATIVE from there, no re-wrap
    CHECK(fwd.velocity().value() == doctest::Approx(p.trueSpeedDegS() * kD2R));
    // REVERSED, reading 180 at the joint's 0: at the joint's current angle it reads 180 - angle
    p.attachRotation(&rev, shulib::sim::RotationMount{.reversed = true, .readsAtJointZeroDeg = 180.0});
    const double at = p.trueDeg();
    CHECK(rev.position().value() / kD2R == doctest::Approx(180.0 - at));
    p.step(0.1);
    CHECK(rev.position().value() / kD2R == doctest::Approx(180.0 - p.trueDeg()));
    CHECK(rev.velocity().value() == doctest::Approx(-p.trueSpeedDegS() * kD2R));
    // a NEGATIVE power-up reading wraps up into the turn: reads -30 -> 330
    ArmPlant q{ms, armCfg()};
    hal::fake::FakeRotation neg;
    q.attachRotation(&neg, shulib::sim::RotationMount{.readsAtJointZeroDeg = -30.0});
    CHECK(neg.position().value() / kD2R == doctest::Approx(330.0));
    // the plain attach reads the joint angle itself, never wrapped (MECH-A2's form, unchanged)
    hal::fake::FakeRotation plain;
    ArmPlantConfig big = armCfg();
    big.maxStopDeg = 1000.0;
    big.startDeg = 500.0;
    ArmPlant b{ms, big};
    b.attachRotation(&plain);
    CHECK(plain.position().value() / kD2R == doctest::Approx(500.0));
}

TEST_CASE("MECH-A3 plant: a limit SWITCH on the joint -- pressed at or below (or above) its angle, at its pressed level") {
    hal::fake::FakeMotor m;
    std::array<hal::fake::FakeMotor*, 1> ms{&m};
    ArmPlantConfig c = armCfg();
    c.startDeg = 5.0;
    ArmPlant p{ms, c};
    hal::fake::FakeDigitalIn bottom;
    hal::fake::FakeDigitalIn top;
    hal::fake::FakeDigitalIn low;
    p.attachSwitch(&bottom, shulib::sim::SwitchMount{.atDeg = 0.0});
    p.attachSwitch(&top, shulib::sim::SwitchMount{.atDeg = 4.0, .atTop = true});
    CHECK(top.state());  // 5 >= 4: pressed — the last attached publishes (one switch per plant)
    p.attachSwitch(&low, shulib::sim::SwitchMount{.atDeg = 0.0, .pressedLevel = false});
    CHECK(low.state());  // not pressed, and a pressed switch reads LOW: released reads high
    m.setVoltage(Voltage{-6.0});
    for (int i = 0; i < 100 && p.trueDeg() > -1.0; ++i) {
        p.step(0.01);
    }
    REQUIRE(p.trueDeg() <= 0.0);
    CHECK_FALSE(low.state());  // pressed: low
    p.attachSwitch(&bottom, shulib::sim::SwitchMount{.atDeg = 0.0});
    CHECK(bottom.state());
}

TEST_CASE("MECH-A3 plant: a WEDGE below the joint stops it there; cleared, it goes on to the hard stop; a joint below it is free") {
    hal::fake::FakeMotor m;
    std::array<hal::fake::FakeMotor*, 1> ms{&m};
    ArmPlantConfig c = armCfg();
    c.startDeg = 60.0;
    ArmPlant p{ms, c};
    p.setWedgeDeg(25.0);
    m.setVoltage(Voltage{-6.0});
    for (int i = 0; i < 300; ++i) {
        p.step(0.01);
    }
    CHECK(p.trueDeg() == doctest::Approx(25.0));
    CHECK(p.trueSpeedDegS() == 0.0);
    CHECK(m.current().value() == doctest::Approx(6.0 * 2.5 / 12.0));  // pushing into it: stall-grade for 6 V
    p.clearWedge();
    for (int i = 0; i < 300; ++i) {
        p.step(0.01);
    }
    CHECK(p.trueDeg() == doctest::Approx(-10.0));  // the hard stop
    p.setWedgeDeg(25.0);  // a wedge ABOVE the joint is not in its way going down
    m.setVoltage(Voltage{6.0});
    for (int i = 0; i < 10; ++i) {
        p.step(0.01);
    }
    CHECK(p.trueDeg() > -10.0);
}

// ═══ MECH-S-A: the MOTOR'S OWN CURRENT LIMIT in the plant (the MECH-S brief's CA6; HA-219 believed, the
// limiter INVENTED like every constant here — HA-183) ═══════════════════════════════════════════════
namespace {
/// A joint with inertia (τ = kA/kV = 50 ms), no gravity of its own, starting on its bottom stop.
ArmPlantConfig massCfg() {
    ArmPlantConfig c;
    c.gearRatio = 1.0;
    c.kS = 0.3;
    c.kV = 1.2;
    c.kA = 0.06;
    c.kG = 0.0;
    c.constantGravity = true;
    c.minStopDeg = 0.0;
    c.maxStopDeg = 3000.0;
    c.startDeg = 0.0;
    return c;
}

/// The MECH-S brief's p_plantlimit case at plant level: a lift (red, direct, its true kV the cartridge's,
/// 1.2 V of gravity) pushed at 12 V for 0.8 s, then flipped to -12 V — every sample's bits folded into
/// an FNV-1a digest, and seven samples around the flip as they are.
struct Reversal {
    std::uint64_t digest = 1469598103934665603ULL;
    double samples[7][4]{};
};
Reversal reversal(double limitAmps) {
    ArmPlantConfig c;
    c.gearRatio = 1.0;
    const double freeRadS = 100.0 * 2.0 * kPi / 60.0;
    c.kS = 0.3;
    c.kV = 12.0 / freeRadS;
    c.kA = 0.05 * c.kV;
    c.kG = 1.2;
    c.constantGravity = true;
    c.minStopDeg = -1.0;
    c.maxStopDeg = 24.5 * 360.0 / (kPi * 1.5);
    std::array<hal::fake::FakeMotor, 2> m{};
    std::array<hal::fake::FakeMotor*, 2> ms{&m[0], &m[1]};
    if (limitAmps > 0.0) {
        for (auto* q : ms) {
            REQUIRE(q->setCurrentLimit(shulib::units::Current{limitAmps}));
        }
    }
    ArmPlant p{ms, c};
    Reversal r;
    const auto mix = [&r](double d) {
        std::uint64_t b = 0;
        std::memcpy(&b, &d, 8);
        for (int k = 0; k < 8; ++k) {
            r.digest ^= (b >> (8 * k)) & 0xffU;
            r.digest *= 1099511628211ULL;
        }
    };
    for (int i = 0; i < 110; ++i) {
        for (auto* q : ms) {
            q->setVoltage(Voltage{i < 80 ? 12.0 : -12.0});
        }
        p.step(0.01);
        const double s[4] = {p.trueDeg(), p.trueSpeedDegS(), p.appliedVolts(), m[0].current().value()};
        for (const double d : s) {
            mix(d);
        }
        if (i >= 78 && i <= 84) {
            std::memcpy(r.samples[i - 78], s, sizeof s);
        }
    }
    return r;
}
}  // namespace

TEST_CASE("MECH-S-A CA6 plant: AT THE DEFAULT the limiter is BYPASSED -- the reversal's samples are ef6162e's, byte for byte") {
    // The pins were taken from the plant BEFORE MECH-S-A (ef6162e's sim/mechanism_plant.hpp, built in the
    // run's scratch probe p_reversal.cpp: digest and samples identical before and after). The flip's two
    // ticks read 2.5 A — the plant's READING clamp — while |V - kV·w| passes 12 V (the brief's 18.26 V): an
    // EXISTING inaccuracy, kept on purpose (re-check #15). BUG CAUGHT (MA11): a limiter active at the
    // default re-solves those two ticks and every sample after them.
    const double pinned[7][4] = {
        {0x1.87099da2058bfp+8, 0x1.067ffda2aa065p+9, 0x1.8p+3, 0x1.4000142d8852cp-2},
        {0x1.8c499d981c09dp+8, 0x1.067ffe1064972p+9, 0x1.8p+3, 0x1.400010852e29bp-2},
        {0x1.8f6aad864515cp+8, 0x1.38ea390808adfp+8, -0x1.8p+3, 0x1.4p+1},
        {0x1.90cf3867952f6p+8, 0x1.168c800693fc7p+7, -0x1.8p+3, 0x1.4p+1},
        {0x1.90c7d223e1059p+8, -0x1.71f939302a1ccp+1, -0x1.8p+3, 0x1.3e755c8fcc9fdp+1},
        {0x1.8f96736e882f8p+8, -0x1.dd23fb5ace899p+6, -0x1.8p+3, 0x1.00619a3828ba8p+1},
        {0x1.8d711f93fd4d9p+8, -0x1.ad2982bc8080ep+7, -0x1.8p+3, 0x1.9b1d0f9b7732bp+0},
    };
    for (const double limit : {0.0, 2.5}) {  // never set, and set to the default exactly
        CAPTURE(limit);
        const Reversal r = reversal(limit);
        CHECK(r.digest == 0xeb8a40335b400333ULL);
        for (int i = 0; i < 7; ++i) {
            for (int k = 0; k < 4; ++k) {
                CAPTURE(i);
                CAPTURE(k);
                CHECK(std::memcmp(&r.samples[i][k], &pinned[i][k], 8) == 0);
            }
        }
    }
    // and a limit BELOW the default does change them (the pin is not blind)
    CHECK(reversal(2.4).digest != 0xeb8a40335b400333ULL);
}

TEST_CASE("MECH-S-A CA6 plant: a joint pushed into a hard stop under a 1.2 A limit READS at most 1.2 A -- 2.5 A uncapped") {
    const auto pushed = [](double limit) {
        hal::fake::FakeMotor m;
        std::array<hal::fake::FakeMotor*, 1> ms{&m};
        if (limit > 0.0) {
            REQUIRE(m.setCurrentLimit(shulib::units::Current{limit}));
        }
        ArmPlant p{ms, armCfg()};
        m.setVoltage(Voltage{12.0});
        double worst = 0.0;
        for (int i = 0; i < 300; ++i) {
            p.step(0.01);
            worst = std::max(worst, m.current().value());
        }
        struct Out {
            double deg, amps, worst, applied;
        };
        return Out{p.trueDeg(), m.current().value(), worst, p.appliedVolts()};
    };
    const auto free = pushed(0.0);
    const auto capped = pushed(1.2);
    CHECK(free.deg == doctest::Approx(170.0));
    CHECK(free.amps == doctest::Approx(2.5));
    CHECK(capped.deg == doctest::Approx(170.0));          // it still gets there (12 V moves it freely)
    CHECK(capped.amps == doctest::Approx(1.2));            // stalled AT the limit
    CHECK(capped.worst <= 1.2 + 1e-12);                    // every sample, the impact's included
    CHECK(capped.applied == doctest::Approx(1.2 * 12.0 / 2.5));  // the volts it pushes with: 5.76 V
}

TEST_CASE("MECH-S-A CA6 plant: the limit limits TORQUE, not only the reading -- a load above its torque (below 2.5 A's) is never lifted") {
    // 1.2 A = 5.76 V of torque. A 7.0 V load (1.46 A's worth) is above it and below 2.5 A's 12 V: uncapped
    // the joint climbs; capped it stays on its bottom stop. A 5.0 V load (below the limit's torque) still
    // climbs capped — the limit is not stricter than itself.
    const auto climb = [](double load, double limit) {
        hal::fake::FakeMotor m;
        std::array<hal::fake::FakeMotor*, 1> ms{&m};
        if (limit > 0.0) {
            REQUIRE(m.setCurrentLimit(shulib::units::Current{limit}));
        }
        ArmPlantConfig c = massCfg();
        c.kG = load;
        ArmPlant p{ms, c};
        m.setVoltage(Voltage{12.0});
        double worst = 0.0;
        for (int i = 0; i < 200; ++i) {
            p.step(0.01);
            worst = std::max(worst, m.current().value());
        }
        struct Out {
            double deg, worst;
        };
        return Out{p.trueDeg(), worst};
    };
    CHECK(climb(7.0, 0.0).deg > 100.0);     // uncapped: it climbs
    const auto held = climb(7.0, 1.2);
    CHECK(held.deg == 0.0);                 // BUG CAUGHT (MA4): a plant that ignores the limit lifts it
    CHECK(held.worst <= 1.2 + 1e-12);
    const auto light = climb(5.0, 1.2);
    CHECK(light.deg > 100.0);               // under the limit's torque: it climbs
    CHECK(light.worst <= 1.2 + 1e-12);
}

TEST_CASE("MECH-S-A CA6 plant: the Hold brake at 0 V is limited like any command -- a load the limit cannot hold sags") {
    // Hold restores with up to 12 V; under a 0.5 A limit (2.4 V of torque) a 3.0 V load (less 0.3 V of
    // friction: 2.7 V) cannot be held — it sags to its stop; uncapped it holds (its 40 V/rad stiffness sags
    // ~3.9 degrees under 2.7 V).
    const auto held = [](double limit) {
        hal::fake::FakeMotor m;
        std::array<hal::fake::FakeMotor*, 1> ms{&m};
        if (limit > 0.0) {
            REQUIRE(m.setCurrentLimit(shulib::units::Current{limit}));
        }
        ArmPlantConfig c = massCfg();
        c.kG = 3.0;
        c.startDeg = 90.0;
        ArmPlant p{ms, c};
        m.setVoltage(Voltage{3.3});
        p.step(0.01);
        m.setBrakeMode(hal::BrakeMode::Hold);
        m.setVoltage(Voltage{0.0});
        double worst = 0.0;
        for (int i = 0; i < 300; ++i) {
            p.step(0.01);
            worst = std::max(worst, m.current().value());
        }
        struct Out {
            double deg, worst;
        };
        return Out{p.trueDeg(), worst};
    };
    const auto free = held(0.0);
    CHECK(free.deg > 85.0);
    const auto capped = held(0.5);
    CHECK(capped.deg < 10.0);             // it could not be held
    CHECK(capped.worst <= 0.5 + 1e-12);
}

TEST_CASE("MECH-S-A CA6 plant: a JAMMED joint under a limit applies at most the limit's volts, both ways") {
    hal::fake::FakeMotor m;
    std::array<hal::fake::FakeMotor*, 1> ms{&m};
    REQUIRE(m.setCurrentLimit(shulib::units::Current{1.0}));
    ArmPlant p{ms, armCfg()};
    p.setJammed(true);
    m.setVoltage(Voltage{12.0});
    p.step(0.01);
    CHECK(p.appliedVolts() == doctest::Approx(4.8));  // 1.0 A * 12 / 2.5
    CHECK(m.current().value() == doctest::Approx(1.0));
    m.setVoltage(Voltage{-12.0});
    p.step(0.01);
    CHECK(p.appliedVolts() == doctest::Approx(-4.8));
    CHECK(m.current().value() == doctest::Approx(1.0));
    m.setVoltage(Voltage{3.0});  // inside the limit: untouched
    p.step(0.01);
    CHECK(p.appliedVolts() == doctest::Approx(3.0));
}

TEST_CASE("MECH-S-A CA6 plant (review): a load the limit cannot carry even at -12 V falls at the -12 V step's speed, its READING clamped at the limit") {
    // A 6 V load held up by 3 V under a 0.3 A limit (1.44 V of torque): the bisection's far end, -12 V, still
    // leaves |V - kV·w| at kG - kS = 5.7 V (1.19 A's worth), so no volts are inside the limit — limitedAdvance
    // takes the -12 V end's step (the header: this plant has no mass to fall with) and publish() clamps the
    // reading. BUG CAUGHT (review P1): the reading clamp removed — every sample read 1.19 A under a 0.3 A limit.
    hal::fake::FakeMotor m;
    std::array<hal::fake::FakeMotor*, 1> ms{&m};
    REQUIRE(m.setCurrentLimit(shulib::units::Current{0.3}));
    ArmPlantConfig c = armCfg();  // memoryless (kA 0): each step's speed is its steady speed exactly
    c.constantGravity = true;
    c.kG = 6.0;
    c.minStopDeg = -1.0e6;
    c.maxStopDeg = 1.0e6;
    ArmPlant p{ms, c};
    m.setVoltage(Voltage{3.0});
    const double fallRadS = (-12.0 - c.kG + c.kS) / c.kV;  // the -12 V step's speed: -14.75 rad/s
    for (int i = 0; i < 20; ++i) {
        CAPTURE(i);
        p.step(0.01);
        CHECK(m.current().value() <= 0.3 + 1e-12);              // clamped at the limit, every sample
        CHECK(m.current().value() == doctest::Approx(0.3));
        CHECK(p.appliedVolts() == -12.0);                       // the far end's step
        CHECK(p.trueSpeedDegS() == doctest::Approx(fallRadS / kD2R));
    }
    // uncapped, the same 3 V against the 6 V load: it falls slower and reads its true 1.19 A (the clamp is
    // the LIMIT's, not a second reading rule)
    hal::fake::FakeMotor u;
    std::array<hal::fake::FakeMotor*, 1> us{&u};
    ArmPlant q{us, c};
    u.setVoltage(Voltage{3.0});
    q.step(0.01);
    CHECK(q.appliedVolts() == 3.0);
    CHECK(u.current().value() == doctest::Approx((c.kG - c.kS) * 2.5 / 12.0));
}
