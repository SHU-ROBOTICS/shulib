// mechanism/service.hpp — the SPEED behaviour (a Flywheel) on the host test plant (chunk MECH-A3, the
// MECH brief's §1.7 and §3.3 P3). Say "host test plant", never "simulator": every plant constant is
// INVENTED (HA-183); the controller runs the library's STARTING values (HA-182).
//
// ═══ WHAT THIS FILE TRIES TO BREAK ══════════════════════════════════════════════════
//   * P3: robot x's flywheel (Blue, geared up 6:1 — 3600 rpm free) to "shoot" = 3000 rpm in 2 s over 27
//     plants — loss kV ×1.0/1.1/1.2, friction kS 0.2/0.4/0.8, spin-up τ 0.1/0.3/0.5 s — never tuned:
//     every case the plant CAN reach is at speed but the stated slowest; the unreachable ones TIME OUT
//     honestly and KEEP SPINNING (an unreachable wheel is not a fault); a 20 % shot is recovered.
//   * THE INTEGRAL'S WINDOW earns its place: the same gains with the integral acting from the start
//     (a reference loop in this file, the ablation) overshoot by more than 150 rpm; the library's
//     loop by less than 60.
//   * A JAMMED wheel (the re-check's case H) trips the REQUIRED stall signature, before "at speed" and
//     after it — 0 V, never 30 s at 12 V / 2.5 A; after its step, the command number is bumped.
//   * The clamp (a computed speed past the free speed, logged once), stop(), the mode changes, and
//     the angle sensor as the speed reading.
//   * THE REVIEW'S HOLES (MECH-A3's review, 2026-10-22): the integral's 3 V clamp (a wheel dragged
//     short inside the window), its FREEZE outside the window (a shot's recovery), its reset at a new
//     spin-up, a reverse spin (the window and kS mirrored), the bound's exact edge, find()'s preset
//     values, and the boot line's tuned marks.
// Thresholds are the measured number plus a stated margin (`scratch/mech-a3/probes/p2.txt`, the
// brief's P3 reproduced to the hundredth on this code).

#include "doctest.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <span>
#include <string>

#include "mechanism_fixtures.hpp"
#include "mechanism_position_rig.hpp"
#include "shulib/core/check.hpp"
#include "shulib/mechanism/kinds.hpp"
#include "shulib/mechanism/service.hpp"

namespace {

namespace mech = shulib::mechanism;
namespace sim = shulib::sim;
namespace units = shulib::units;
namespace rx = mech_fixture::robot_x;
using mech::MechanismOutcome;
using pos_rig::FlyRig;

constexpr double kPi = 3.14159265358979323846;
/// The starting kV of a 3600 rpm wheel — the plant's nominal kV is the same physics (12 V at free speed).
constexpr double kNominalKv = 12.0 / (3600.0 * 2.0 * kPi / 60.0);

template <class F>
std::string preconditionOf(F&& f) {
    try {
        f();
    } catch (const shulib::PreconditionError& e) {
        return e.what();
    }
    return {};
}

sim::FlywheelPlantConfig wheel(double kvx, double kS, double tau) {
    sim::FlywheelPlantConfig pc;
    pc.gearRatio = 1.0 / 6.0;
    pc.kS = kS;
    pc.kV = kNominalKv * kvx;
    pc.kA = pc.kV * tau;
    return pc;
}

}  // namespace

TEST_CASE("MECH-A3 speed: P3 -- 27 plants, the starting values never tuned: every REACHABLE speed is reached but the stated slowest; the rest TIME OUT and keep spinning") {
    // Measured (the A3 probe = the brief's P3): 20/27 at speed in 0.31..1.87 s, overshoot <= 49.7 rpm,
    // a 20 % shot back at speed in <= 1.35 s; the slowest reachable (kVx 1.2, kS 0.2, tau 0.5) and the six
    // unreachable ones TIMED_OUT at 2.00 s, still commanded 11.6..12.0 V. Margins: overshoot 60, back in
    // 1.5 s, at speed in 2 s.
    int reached = 0;
    int reachable = 0;
    int reachableReached = 0;
    double worstOver = 0.0;
    for (const double kvx : {1.0, 1.1, 1.2}) {
        for (const double kS : {0.2, 0.4, 0.8}) {
            for (const double tau : {0.1, 0.3, 0.5}) {
                CAPTURE(kvx);
                CAPTURE(kS);
                CAPTURE(tau);
                const sim::FlywheelPlantConfig pc = wheel(kvx, kS, tau);
                FlyRig r{rx::flywheel.spec(), pc};
                const double topRpm = (12.0 - kS) / pc.kV * 60.0 / (2.0 * kPi);
                const bool canReach = topRpm >= 3000.0 - 60.0;  // "at speed" is within 60 rpm of 3000
                r.slot().spinUpPreset(0, units::Time{2.0});
                double over = 0.0;
                for (int i = 0; i < 210; ++i) {
                    r.tick();
                    over = std::max(over, r.plant->trueRpm() - 3000.0);
                }
                const mech::StepReport rep = r.slot().lastStep();
                CHECK(rep.outcome != MechanismOutcome::Running);
                CHECK((rep.outcome == MechanismOutcome::Succeeded || rep.outcome == MechanismOutcome::TimedOut));
                worstOver = std::max(worstOver, over);
                if (rep.outcome == MechanismOutcome::TimedOut) {
                    CHECK(rep.elapsed == doctest::Approx(2.0).epsilon(0.01));
                    CHECK(r.slot().spinning());        // it KEEPS spinning (N2's exception, on purpose)
                    CHECK(r.volts() >= 11.0);          // still commanded hard
                    CHECK(r.slot().holdsClaim());
                } else {
                    ++reached;
                    CHECK(rep.doneBy == mech::DoneBy::Reached);
                    CHECK(std::abs(r.slot().rpm() - 3000.0) <= 60.0 + 30.0);  // within the band at the verdict (+ drift)
                }
                if (canReach) {
                    ++reachable;
                    reachableReached += rep.outcome == MechanismOutcome::Succeeded ? 1 : 0;
                } else {
                    CHECK(rep.outcome == MechanismOutcome::TimedOut);  // never a false "at speed"
                }
                if (rep.outcome == MechanismOutcome::Succeeded) {
                    // a 20 % shot: it drops out of "at speed" and comes back (measured <= 1.35 s)
                    r.plant->shot(0.2);
                    r.tick(2);  // the shot reaches the motors' readings at the next world step
                    CHECK_FALSE(r.slot().atSpeed());
                    double back = -1.0;
                    for (int k = 0; k < 300 && back < 0.0; ++k) {
                        r.tick();
                        if (r.slot().atSpeed()) {
                            back = (k + 1) * 0.01;
                        }
                    }
                    CHECK(back > 0.0);
                    CHECK(back <= 1.5);  // measured <= 1.35 s (the brief: "a 20 % shot recovered in <= 1.35 s")
                    CHECK(r.slot().lastStep().outcome == MechanismOutcome::Succeeded);  // the step's verdict stands
                }
            }
        }
    }
    CHECK(reached == 20);
    CHECK(reachable == 21);
    CHECK(reachableReached >= 20);  // every reachable one but the stated slowest
    CHECK(worstOver <= 60.0);
}

TEST_CASE("MECH-A3 speed: the INTEGRAL'S WINDOW earns its place -- the same gains with the integral from the start overshoot by > 150 rpm") {
    // The ablation, re-derived here as a reference loop (the library's window is a constant): the SAME
    // feedforward, P and I gains, the I clamped to the same 3 V, but integrating from the first tick.
    // Matched plant (kVx 1.0, kS 0.2, tau 0.3). Measured: the library's loop overshoots 31.2 rpm; the
    // reference 233.3 (the brief's P3, on its own plant: "+219 rpm matched").
    const mech::MechanismSpec spec = rx::flywheel.spec();
    const sim::FlywheelPlantConfig pc = wheel(1.0, 0.2, 0.3);
    FlyRig lib{spec, pc};
    lib.slot().spinUpPreset(0, units::Time{2.0});
    double libOver = 0.0;
    for (int i = 0; i < 300; ++i) {
        lib.tick();
        libOver = std::max(libOver, lib.plant->trueRpm() - 3000.0);
    }
    // the reference: a plant of its own, driven by hand
    std::array<shulib::hal::fake::FakeMotor, 1> m{};
    std::array<shulib::hal::fake::FakeMotor*, 1> fm{&m[0]};
    sim::FlywheelPlant plant{std::span<shulib::hal::fake::FakeMotor* const>{fm.data(), 1}, pc};
    const double target = 3000.0 * 2.0 * kPi / 60.0;
    double integral = 0.0;
    double refOver = 0.0;
    for (int i = 0; i < 300; ++i) {
        const double w = m[0].velocity().value() / spec.speed.gearRatio;
        const double e = target - w;
        integral = std::clamp(integral + e * 0.01, -3.0 / spec.speed.kI, 3.0 / spec.speed.kI);
        const double v = std::clamp(spec.speed.kV * target + spec.speed.kP * e + spec.speed.kI * integral, -12.0, 12.0);
        m[0].setVoltage(units::Voltage{v});
        plant.step(0.01);
        refOver = std::max(refOver, plant.trueRpm() - 3000.0);
    }
    CHECK(libOver < 60.0);
    CHECK(refOver > 150.0);
}

TEST_CASE("MECH-A3 speed: case H -- a JAMMED wheel trips its REQUIRED stall signature: 0 V, never 30 s at stall current") {
    // The re-check's case H: spinUp("shoot", 2 s) on a wheel that cannot turn. Before N10: TIMED_OUT, then
    // 12.00 V / 2.50 A for 30 s. Now: STALLED (robot x's 2.0 A below 30 rpm for 0.5 s), 0 V.
    FlyRig r{rx::flywheel.spec(), wheel(1.0, 0.4, 0.2)};
    r.plant->setJammed(true);
    r.slot().spinUpPreset(0, units::Time{2.0});
    double atStall = 0.0;
    for (int i = 0; i < 3000; ++i) {
        r.tick();
        atStall += r.motors[0].current().value() >= 2.0 ? 0.01 : 0.0;
    }
    CHECK(r.slot().lastStep().outcome == MechanismOutcome::Stalled);
    CHECK(r.slot().lastStep().elapsed <= 0.6);  // its 0.5 s window from the first tick
    CHECK(atStall <= 0.6);
    CHECK(r.stallFaults() == 1);
    CHECK(r.volts() == 0.0);
    CHECK_FALSE(r.mech->claimed());
    CHECK_FALSE(r.slot().spinning());
}

TEST_CASE("MECH-A3 speed: a jam AFTER 'at speed' still trips the signature; the toggle that started it is superseded") {
    FlyRig r{rx::flywheel.spec(), wheel(1.0, 0.4, 0.1)};
    r.slot().spinUpPreset(0, units::Time{2.0});
    for (int i = 0; i < 100 && r.slot().running(); ++i) {
        r.tick();
    }
    REQUIRE(r.slot().lastStep().outcome == MechanismOutcome::Succeeded);
    const std::uint32_t seq = r.slot().commandSeq();
    r.tick(50);
    REQUIRE(r.slot().spinning());
    r.plant->setJammed(true);
    r.tick(100);
    CHECK(r.slot().lastStep().outcome == MechanismOutcome::Stalled);
    CHECK(r.slot().commandSeq() == seq + 1);
    CHECK(r.stallFaults() == 1);
    CHECK(r.volts() == 0.0);
    CHECK_FALSE(r.slot().spinning());
    CHECK_FALSE(r.slot().atSpeed());
    bool said = false;
    for (int i = 0; i < r.sink.size(); ++i) {
        said = said || r.sink.at(i).message.find("(a stated stall signature tripped while spinning)") != std::string::npos;
    }
    CHECK(said);
}

TEST_CASE("MECH-A3 speed: a COMPUTED speed past the free speed is CLAMPED and logged once; a constant one is a precondition") {
    FlyRig r{rx::flywheel.spec(), wheel(1.0, 0.2, 0.1)};
    r.slot().spinUp(4000.0, units::Time{2.0}, /*computed=*/true);
    CHECK(r.slot().targetRpm() == doctest::Approx(3600.0));
    int clamped = 0;
    for (int i = 0; i < r.sink.size(); ++i) {
        clamped += r.sink.at(i).message == "'flywheel': speed 4000 rpm clamped to its free speed 3600 rpm" ? 1 : 0;
    }
    CHECK(clamped == 1);
    r.tick(100);
    for (int i = 0; i < r.sink.size(); ++i) {
        clamped -= r.sink.at(i).message.find("clamped to its free speed") != std::string::npos ? 1 : 0;
    }
    CHECK(clamped == 0);  // once, never per tick
    CHECK(preconditionOf([&] { r.slot().spinUp(3600.5, units::Time{1.0}, false); }).find("CONSTANT speed") !=
          std::string::npos);
    CHECK(preconditionOf([&] { r.slot().spinUp(std::numeric_limits<double>::quiet_NaN(), units::Time{1.0}); })
              .find("finite") != std::string::npos);
    CHECK(preconditionOf([&] { r.slot().spinUp(3000.0, units::Time{0.0}); }).find("time limit") != std::string::npos);
    CHECK(preconditionOf([&] { r.slot().spinUpPreset(1, units::Time{1.0}); }).find("no such preset") != std::string::npos);
}

TEST_CASE("MECH-A3 speed: stop(), the mode changes and the guard's reach -- the wheel coasts, safe, released") {
    FlyRig r{rx::flywheel.spec(), wheel(1.0, 0.2, 0.1)};
    r.slot().spinUpPreset(0, units::Time{2.0});
    r.tick(20);
    REQUIRE(r.slot().running());
    r.slot().stop();
    CHECK(r.slot().lastStep().outcome == MechanismOutcome::Cancelled);
    CHECK(r.volts() == 0.0);
    CHECK(r.motors[0].brakeMode() == shulib::hal::BrakeMode::Coast);
    CHECK_FALSE(r.slot().spinning());
    CHECK(r.slot().targetRpm() == 0.0);
    // spinning after its step: driver entry stops it (finding 8's rule for every motor kind)
    r.slot().spinUpPreset(0, units::Time{2.0});
    r.tick(150);
    REQUIRE(r.slot().spinning());
    r.svc.enterDriverControl();
    CHECK_FALSE(r.slot().spinning());
    CHECK(r.volts() == 0.0);
    // the guard's cancel mid-step
    r.slot().spinUpPreset(0, units::Time{2.0});
    r.tick(5);
    r.slot().cancel();
    CHECK(r.slot().lastStep().outcome == MechanismOutcome::Cancelled);
    CHECK_FALSE(r.mech->claimed());
    r.tick(5);
    CHECK(r.volts() == 0.0);
}

TEST_CASE("MECH-A3 speed: an angle sensor on the wheel IS its speed -- the motors' reading through the gearing otherwise") {
    // A flywheel declared with an angle sensor: its rpm() reads the sensor (the mounting applied); a
    // sensor reading half the motors' speed makes the loop drive the motors to twice the speed.
    using namespace shulib::mechanism::words;
    static constexpr Flywheel kSensed{.name = "fly", .motors = {Forward}, .cartridge = Blue, .gearing = gears(72, 12), .angleSensor = rotationSensor(Reversed), .safe = Coast, .stall = {.amps = 2.0, .belowMotorRpm = 30, .lasting = 0.5_s}, .atSpeed = within(60, 0.1_s), .presets = {{"slow", 1000}}};
    const mech::MechanismSpec spec = kSensed.spec();
    shulib::hal::fake::FakeClock clock;
    shulib::hal::fake::FakeTelemetrySink sink;
    shulib::diag::FaultLatch faults{sink, clock};
    pos_rig::PlantWorld world;
    world.clock = &clock;
    shulib::hal::fake::FakeMotor motor;
    std::array<shulib::hal::IMotor*, 1> im{&motor};
    shulib::hal::MotorMechanism group{im, shulib::hal::BrakeMode::Coast, "fly"};
    shulib::hal::fake::FakeRotation sensor;
    mech::MechanismService svc{world};
    const mech::Devices d{.motors = &group, .angleSensor = &sensor};
    svc.bind(std::span<const mech::MechanismSpec>{&spec, 1}, std::span<const mech::Devices>{&d, 1}, clock, faults,
             sink);
    sensor.setVelocity(units::AngularVelocity{-1000.0 * 2.0 * kPi / 60.0});  // mounted reversed: -1000 reads +1000
    motor.setVelocity(units::AngularVelocity{0.0});
    CHECK(svc.speed(0).rpm() == doctest::Approx(1000.0));
}

TEST_CASE("MECH-A3 speed: the integral is CLAMPED -- a wheel dragged short of its speed for 3 s overshoots by at most what 3 V allows") {
    // A drag (0.8 % of the speed every tick) holds the wheel at ~2857 rpm, inside the integral's 10 %
    // window but short of 3000: the integral grows to its 3 V clamp. Released, the wheel overshoots by
    // 233 rpm (measured) while the clamped integral unwinds; without the clamp the integral winds on
    // (the review's mutation: 540 rpm). Margin: 300.
    FlyRig r{rx::flywheel.spec(), wheel(1.0, 0.2, 0.3)};
    r.slot().spinUpPreset(0, units::Time{2.0});
    double held = 1e9;
    for (int i = 0; i < 300; ++i) {
        r.plant->shot(0.008);
        r.tick();
        held = i > 150 ? std::min(held, r.plant->trueRpm()) : held;
    }
    REQUIRE(held > 3000.0 * (1.0 - mech::kSpeedIntegralWindow));  // inside the window: the integral acts
    REQUIRE(held < 3000.0 - 60.0);                                 // ... and short of the band
    double over = 0.0;
    for (int i = 0; i < 300; ++i) {
        r.tick();
        over = std::max(over, r.plant->trueRpm() - 3000.0);
    }
    CHECK(over <= 300.0);
    CHECK(r.slot().spinning());
}

TEST_CASE("MECH-A3 speed: the integral is FROZEN outside its window, never cleared -- a shot's recovery keeps the trim it had") {
    // A plant 10 % slower than the controller believes (kVx 1.1) needs the integral's trim to sit in the
    // band. A 20 % shot drops it out of the window; the frozen integral still pushes, so it is back at
    // speed in 0.43 s (measured); an integral CLEARED outside the window (the review's mutation) must
    // re-integrate from zero: 0.57 s. Margin: 0.50.
    FlyRig r{rx::flywheel.spec(), wheel(1.1, 0.2, 0.3)};
    r.slot().spinUpPreset(0, units::Time{3.0});
    r.tick(300);
    REQUIRE(r.slot().atSpeed());
    r.plant->shot(0.2);
    r.tick(2);
    REQUIRE_FALSE(r.slot().atSpeed());
    double back = -1.0;
    for (int k = 0; k < 300 && back < 0.0; ++k) {
        r.tick();
        back = r.slot().atSpeed() ? (k + 1) * 0.01 : back;
    }
    CHECK(back > 0.0);
    CHECK(back <= 0.50);
}

TEST_CASE("MECH-A3 speed: a NEW spin-up starts its integral at ZERO -- never the last spin's wound-up trim") {
    // 4 s toward 3500 rpm on a wheel that tops out at 3218 (inside the window: the integral reaches its 3 V
    // clamp), stopped, coasted to rest, then spun to 2000: measured overshoot 0.4 rpm. A spin-up that
    // carried the old integral (the review's mutation) starts with 3 V extra frozen in: 172.5 rpm over.
    // Margin: 50.
    FlyRig r{rx::flywheel.spec(), wheel(1.1, 0.2, 0.3)};
    r.slot().spinUp(3500.0, units::Time{3.0}, false);
    r.tick(400);
    REQUIRE(r.plant->trueRpm() > 3500.0 * (1.0 - mech::kSpeedIntegralWindow));
    r.slot().stop();
    r.tick(600);
    r.slot().spinUp(2000.0, units::Time{3.0}, false);
    double over = 0.0;
    for (int i = 0; i < 300; ++i) {
        r.tick();
        over = std::max(over, r.plant->trueRpm() - 2000.0);
    }
    CHECK(r.slot().lastStep().outcome == MechanismOutcome::Succeeded);
    CHECK(over <= 50.0);
}

TEST_CASE("MECH-A3 speed: a REVERSE spin is the forward one mirrored -- its integral's window opens, its kS pushes backward") {
    // kVx 1.1: without the integral the wheel sits 166 rpm short of 2000 — outside the 60 rpm band, inside
    // the 200 rpm window. Measured: both directions SUCCEEDED in 0.74 s at |1966| rpm. A window keyed on
    // the signed target never opens backward (the review's mutation: TIMED_OUT at -1903 rpm).
    for (const double rpm : {2000.0, -2000.0}) {
        CAPTURE(rpm);
        FlyRig r{rx::flywheel.spec(), wheel(1.1, 0.2, 0.3)};
        r.slot().spinUp(rpm, units::Time{3.0}, false);
        for (int i = 0; i < 400 && r.slot().running(); ++i) {
            r.tick();
        }
        CHECK(r.slot().lastStep().outcome == MechanismOutcome::Succeeded);
        CHECK(std::abs(r.slot().rpm() - rpm) <= 60.0 + 30.0);
    }
    // a tuned kS: exactly kS in the spin's direction on the first tick (two wheels, one gain apart)
    for (const double rpm : {300.0, -300.0}) {
        CAPTURE(rpm);
        mech::MechanismSpec withKs = rx::flywheel.spec();
        withKs.speed.kS = 0.5;
        FlyRig a{rx::flywheel.spec(), wheel(1.0, 0.2, 0.3)};
        FlyRig b{withKs, wheel(1.0, 0.2, 0.3)};
        a.slot().spinUp(rpm, units::Time{2.0}, false);
        b.slot().spinUp(rpm, units::Time{2.0}, false);
        a.tick();
        b.tick();
        CHECK(b.volts() - a.volts() == doctest::Approx(rpm > 0.0 ? 0.5 : -0.5).epsilon(1e-9));
    }
}

TEST_CASE("MECH-A3 speed: a spin-up's bound is met EXACTLY at its edge (1/128 s ticks: TIMED_OUT on the third)") {
    FlyRig r{rx::flywheel.spec(), wheel(1.0, 0.2, 0.3), 1.0 / 128.0};
    r.slot().spinUpPreset(0, units::Time{2.0 / 128.0});
    r.tick(2);
    CHECK(r.slot().running());
    r.tick();
    CHECK(r.slot().lastStep().outcome == MechanismOutcome::TimedOut);
    CHECK(r.slot().lastStep().elapsed == 2.0 / 128.0);
    CHECK(r.slot().spinning());  // and it keeps spinning
}

TEST_CASE("MECH-A3 speed: find() -- a same-named flywheel's goal is this one's only at THIS one's preset and speed") {
    using namespace shulib::mechanism::words;
    static constexpr Flywheel kOther{.name = "flywheel", .motors = {Forward}, .cartridge = Blue, .gearing = gears(72, 12), .safe = Coast, .stall = {.amps = 2.0, .belowMotorRpm = 30, .lasting = 0.5_s}, .atSpeed = within(60, 0.1_s), .presets = {{"shoot", 2500}, {"nope", 1000}}};
    static constexpr Flywheel kSame{.name = "flywheel", .motors = {Forward}, .cartridge = Blue, .gearing = gears(72, 12), .safe = Coast, .stall = {.amps = 2.0, .belowMotorRpm = 30, .lasting = 0.5_s}, .atSpeed = within(60, 0.1_s), .presets = {{"shoot", 3000}}};
    FlyRig r{rx::flywheel.spec(), wheel(1.0, 0.2, 0.3)};  // robot x's: "shoot" at 3000
    CHECK(r.svc.find(rx::flywheel["shoot"]) == &r.slot());
    CHECK(r.svc.find(kSame["shoot"]) == &r.slot());   // the same preset at the same speed
    CHECK(r.svc.find(kOther["nope"]) == nullptr);      // a preset this flywheel does not have
    CHECK(r.svc.find(kOther["shoot"]) == nullptr);     // its "shoot" at ANOTHER speed
    CHECK(r.svc.find(kOther.at(1200)) == &r.slot());   // a number: no preset to check
}

TEST_CASE("MECH-A3 speed: the boot line marks a flywheel's TUNED numbers -- one, and all four") {
    char line[320];
    mech::MechanismSpec one = rx::flywheel.spec();
    one.speed.kP = 0.08;
    mech::startingValuesLine(line, sizeof line, one);
    CHECK(std::string{line} ==
          "flywheel: runs kV 0.0318 kP 0.080 (tuned) (V per output rad/s) kI 0.200 (V per output rad) -- the rest are "
          "starting values, PROVISIONAL (HA-182)");
    mech::MechanismSpec all = rx::flywheel.spec();
    all.speed.kS = 0.3;
    all.speed.kV = 0.04;
    all.speed.kP = 0.08;
    all.speed.kI = 0.3;
    mech::startingValuesLine(line, sizeof line, all);
    CHECK(std::string{line} ==
          "flywheel: runs kS 0.300 (tuned) kV 0.0400 (tuned) kP 0.080 (tuned) (V per output rad/s) kI 0.300 (tuned) "
          "(V per output rad) -- every number stated by its .tuning");
}
