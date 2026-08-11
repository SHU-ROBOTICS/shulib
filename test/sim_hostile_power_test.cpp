// Adversarial tests for sim/hostile/power_hostility.hpp. Pins: sag follows LOAD
// (drive hard → battery reads lower; back off → it recovers), the pack ceiling on
// effective voltage, the brownout collapse (motors get exactly 0 V at/below cutoff
// while the run continues), the thermal ramp with the VEX 50/25/12.5% droop steps,
// and plant-level liveness (a hostile-powered run demonstrably travels less).

#include "doctest.h"

#include <cmath>

#include "shulib/kinematics/x_drive.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/sim/hostile/power_hostility.hpp"
#include "shulib/sim/rng.hpp"
#include "shulib/sim/scenario.hpp"
#include "shulib/units/quantity.hpp"

using shulib::kinematics::xDrive;
using shulib::math::ChassisSpeeds;
using shulib::sim::PowerHostileConfig;
using shulib::sim::PowerHostileModel;
using shulib::sim::Rng;
using shulib::sim::SimHarness;
using shulib::sim::SimHarnessConfig;
using shulib::units::AngularVelocity;
using shulib::units::Length;
using shulib::units::Time;
using shulib::units::Velocity;
using shulib::units::Voltage;

namespace {
/// Drive one plant tick's worth of hook calls by hand: 4 wheels at `volts`, then
/// the battery read after the (simulated) clock advance — the plant's call order.
double tickAndReadBattery(PowerHostileModel& m, double volts, double t, double dt, Rng& rng) {
    for (int w = 0; w < 4; ++w) {
        (void)m.effectiveVoltage(w, Voltage{volts}, Time{t}, rng);
    }
    return m.batteryVoltage(Voltage{12.6}, Time{t + dt}, rng).value();
}
}  // namespace

TEST_CASE("hostile power: battery sag follows load — hard drive reads lower, idle recovers") {
    PowerHostileConfig cfg;
    cfg.dischargeRatePerS = 0.0;  // isolate the sag term
    PowerHostileModel m{cfg};
    Rng rng{1};

    const double idle0 = tickAndReadBattery(m, 0.0, 0.00, 0.01, rng);
    CHECK(idle0 == doctest::Approx(12.6));  // no load, no discharge → nominal

    const double loaded = tickAndReadBattery(m, 12.0, 0.01, 0.01, rng);
    CHECK(loaded == doctest::Approx(12.6 - 0.02 * 48.0));  // 4×12 V of demand → 0.96 V sag

    const double recovered = tickAndReadBattery(m, 0.0, 0.02, 0.01, rng);
    CHECK(recovered == doctest::Approx(12.6));  // stick release → reading comes back
}

TEST_CASE("hostile power: discharge alone drains the pack linearly in time") {
    PowerHostileConfig cfg;
    cfg.sagPerCommandedVolt = 0.0;
    PowerHostileModel m{cfg};
    Rng rng{1};
    CHECK(m.batteryVoltage(Voltage{12.6}, Time{0.0}, rng).value() == doctest::Approx(12.6));
    CHECK(m.batteryVoltage(Voltage{12.6}, Time{60.0}, rng).value()
          == doctest::Approx(12.6 - 0.005 * 60.0));
}

TEST_CASE("hostile power: the pack ceiling caps effective voltage below the F4 clamp") {
    PowerHostileConfig cfg;
    cfg.dischargeRatePerS = 0.0;
    cfg.sagPerCommandedVolt = 0.05;  // aggressive: 4×12 V → 2.4 V sag
    cfg.cutoffVolts = Voltage{5.0};  // push brownout away: this case isolates the CEILING
    PowerHostileModel m{cfg};
    Rng rng{1};

    // Tick 1 establishes the load; capture the pack the battery hook reports…
    (void)tickAndReadBattery(m, 12.0, 0.00, 0.01, rng);
    const double pack = m.batteryVoltage(Voltage{12.6}, Time{0.02}, rng).value();
    CHECK(pack == doctest::Approx(12.6 - 0.05 * 48.0));  // 10.2 V

    // …tick 2's effective voltage is capped by that sagged pack, both signs.
    const double eff = m.effectiveVoltage(0, Voltage{12.0}, Time{0.01}, rng).value();
    CHECK(eff == doctest::Approx(pack).epsilon(1e-9));
    const double effNeg = m.effectiveVoltage(1, Voltage{-12.0}, Time{0.01}, rng).value();
    CHECK(effNeg == doctest::Approx(-pack).epsilon(1e-9));
}

TEST_CASE("hostile power: at/below cutoff the brain cuts motor power to exactly zero") {
    PowerHostileConfig cfg;
    cfg.dischargeRatePerS = 0.0;
    cfg.sagPerCommandedVolt = 0.05;
    cfg.fallbackNominal = Voltage{10.6};  // a nearly-dead pack (no battery call yet)
    PowerHostileModel m{cfg};
    Rng rng{1};

    // Load the pack hard: next tick's pack = 10.6 − 2.4 = 8.2 V ≤ 10.5 → cut.
    for (int w = 0; w < 4; ++w) {
        (void)m.effectiveVoltage(w, Voltage{12.0}, Time{0.0}, rng);
    }
    for (int w = 0; w < 4; ++w) {
        CHECK(m.effectiveVoltage(w, Voltage{12.0}, Time{0.01}, rng).value() == 0.0);
    }
    // The battery reading reports the collapse (bounded at ≥ 0), and the model
    // keeps answering — a brownout is a state, never a crash.
    const double v = m.batteryVoltage(Voltage{10.6}, Time{0.02}, rng).value();
    CHECK(v < 10.5);
    CHECK(v >= 0.0);
}

TEST_CASE("hostile power: sustained drive heats the motor through the VEX droop steps; idle cools") {
    PowerHostileConfig cfg;
    cfg.dischargeRatePerS = 0.0;
    cfg.sagPerCommandedVolt = 0.0;           // isolate thermal
    cfg.heatRatePerV2 = 0.01;  // accelerated bench config (not the default) —
    cfg.coolRatePerS = 0.02;   // equilibrium 25 + 0.01·144/0.02 = 97 °C, so the ramp
                               // genuinely crosses both throttle steps
    PowerHostileModel m{cfg};
    Rng rng{1};

    CHECK(m.temperatureC(0) == doctest::Approx(25.0));

    // Drive wheel 0 at 12 V until it crosses the first throttle step.
    double t = 0.0;
    double effAtCross = 0.0;
    bool crossed = false;
    for (int i = 0; i < 5000 && !crossed; ++i) {
        t = 0.01 * i;
        effAtCross = m.effectiveVoltage(0, Voltage{12.0}, Time{t}, rng).value();
        crossed = m.temperatureC(0) >= 55.0;
    }
    REQUIRE(crossed);  // the ramp is live
    CHECK(effAtCross == doctest::Approx(12.0 * 0.5).epsilon(1e-6));  // 50% step engaged
    CHECK(m.maxTemperatureC() == doctest::Approx(m.temperatureC(0)));

    // Keep pushing: the 60 °C step quarters the drive.
    bool at60 = false;
    double effAt60 = 0.0;
    for (int i = 0; i < 20000 && !at60; ++i) {
        t += 0.01;
        effAt60 = m.effectiveVoltage(0, Voltage{12.0}, Time{t}, rng).value();
        at60 = m.temperatureC(0) >= 60.0;
    }
    REQUIRE(at60);
    CHECK(effAt60 == doctest::Approx(12.0 * 0.25).epsilon(1e-6));

    // Idle: the wheel cools back toward ambient and the droop releases.
    for (int i = 0; i < 20000 && m.temperatureC(0) >= 55.0; ++i) {
        t += 0.01;
        (void)m.effectiveVoltage(0, Voltage{0.0}, Time{t}, rng);
    }
    CHECK(m.temperatureC(0) < 55.0);
    t += 0.01;
    CHECK(m.effectiveVoltage(0, Voltage{6.0}, Time{t}, rng).value()
          == doctest::Approx(6.0).epsilon(1e-6));  // full drive restored
}

TEST_CASE("hostile power: wheels heat independently (a hot wheel does not throttle a cool one)") {
    PowerHostileConfig cfg;
    cfg.sagPerCommandedVolt = 0.0;
    cfg.dischargeRatePerS = 0.0;
    cfg.heatRatePerV2 = 0.01;
    cfg.coolRatePerS = 0.0;
    PowerHostileModel m{cfg};
    Rng rng{1};
    for (int i = 0; i < 3000; ++i) {
        (void)m.effectiveVoltage(0, Voltage{12.0}, Time{0.01 * i}, rng);  // hammered
        (void)m.effectiveVoltage(1, Voltage{2.0}, Time{0.01 * i}, rng);   // gentle
    }
    CHECK(m.temperatureC(0) >= 55.0);
    CHECK(m.temperatureC(1) < 40.0);
    const double t = 30.0;
    CHECK(m.effectiveVoltage(1, Voltage{6.0}, Time{t}, rng).value()
          == doctest::Approx(6.0).epsilon(1e-6));  // the cool wheel drives clean
}

// ── Plant-level liveness: DEFAULT power hostility measurably slows a real run —
// a dead default (a no-op'd model) reds here. The command deliberately SATURATES
// (full-stick 100 in/s → wheels demand >13 V → the F4 clamp holds them at 12 V),
// because under true-voltage-control semantics sag only bites when demand exceeds
// the sagged pack CEILING — a 6 V cruise on an 11.6 V pack is unaffected, exactly
// as on hardware (observed while writing this test; recorded in the A3 log). ──
TEST_CASE("hostile power: a default-hostile full-throttle run travels measurably less") {
    const auto kin = xDrive(Length{7.0});
    SimHarnessConfig cfg;
    cfg.plant.wheelFf = {.kS = 1.2, .kV = 0.17, .kA = 0.0};

    SimHarness clean{kin, cfg};
    clean.commandBodyTwist(ChassisSpeeds{Velocity{100.0}, Velocity{0.0}, AngularVelocity{0.0}});
    clean.runTicks(500, Time{0.01});  // 5 s full throttle, F4-clamped to 12 V

    PowerHostileModel hostilePower{};  // the PROVISIONAL defaults
    SimHarness hostile{kin, cfg, nullptr, &hostilePower};
    hostile.commandBodyTwist(ChassisSpeeds{Velocity{100.0}, Velocity{0.0}, AngularVelocity{0.0}});
    hostile.runTicks(500, Time{0.01});

    const double cleanX = clean.truePose().x().value();
    const double hostileX = hostile.truePose().x().value();
    CHECK(hostileX < cleanX - 1.0);  // the sagged ceiling cost real distance
    CHECK(hostileX > 0.5 * cleanX);  // but the realistic default is degradation, not death
    // and the battery the code under test reads shows the sag
    CHECK(hostile.battery().voltage().value() < 12.6 - 0.5);
}

TEST_CASE("hostile power: rejects an out-of-range config") {
    PowerHostileConfig bad;
    bad.sagPerCommandedVolt = -0.1;
    CHECK_THROWS_AS((PowerHostileModel{bad}), shulib::PreconditionError);
    PowerHostileConfig bad2;
    bad2.throttleTempC = 20.0;  // below ambient
    CHECK_THROWS_AS((PowerHostileModel{bad2}), shulib::PreconditionError);
}
