// mechanism/kinds.hpp — THE WORDS a team writes, into the spec (chunk MECH-A1, the MECH brief's §3.1
// "the words -> the spec", "where it was written", "the conversions").
//
// ═══ WHAT THIS FILE TRIES TO BREAK ══════════════════════════════════════════════════
//   * Every word sets EXACTLY the fields it names — robot x, robot two and robot y (P5b: every
//     optional word) read back field by field, with the literal the team wrote beside each.
//   * The STARTING values (N7; HA-182, registered by MECH-A3) for Red, Green and
//     Blue through 1:1, 5:1 and 7:1, from first principles (the cartridge's free rpm x 6 / the
//     reduction = the output's degrees per second) — and `.tuning` replaces EXACTLY what it names.
//   * The GEARING's direction: gears(12, 60) is FIVE motor turns per output turn — the inverse would
//     move a 95-degree preset 3.8 degrees and call it done (the re-check's U1 case).
//   * WHERE each declaration was written: its `.name` line, read from the fixture's own text (not a
//     typed number), so a capture of the wrong line (the closing brace, GCC 13's initializer-list
//     line) is red; and the wiring line for each wire(...).
//   * The CONVERSIONS (P2g): spool, linkage at 1/2/3 stages and a measured table round-trip within
//     1e-12 across their range; past the range they clamp (the table's ends; the linkage's asin).
//   * The goals a declaration makes, and the words that convert both ways (BrakeWord).
// Every form AT THE EXACT LIMITS compiles here — the positive half of test/compile_refusals/.

#include "doctest.h"

#include <cmath>
#include <cstring>
#include <string>
#include <vector>

#include "mechanism_fixtures.hpp"
#include "source_pins.hpp"
#include "shulib/mechanism/kinds.hpp"
#include "shulib/mechanism/spec.hpp"

using shulib::mechanism::Confirm;
using shulib::mechanism::Geometry;
using shulib::mechanism::Gravity;
using shulib::mechanism::HomeMethod;
using shulib::mechanism::Kind;
using shulib::mechanism::MechanismSpec;
using shulib::mechanism::PositionSpec;
using shulib::mechanism::Refusal;
using shulib::mechanism::SensorKind;
using shulib::mechanism::specRefusal;
using shulib::mechanism::Units;

namespace rx = mech_fixture::robot_x;
namespace ry = mech_fixture::robot_y;
namespace r2 = mech_fixture::robot_two;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double rpmToRadS(double rpm) { return rpm * 2.0 * kPi / 60.0; }

/// The 1-based line of the FIRST line of the fixture file containing `text` (0 if none) — the pins
/// read the file, so a moved line moves its pin with it.
std::uint32_t fixtureLineOf(const std::string& text) {
    const std::string file = source_pins::readFile("test/mechanism_fixtures.hpp");
    const std::size_t at = file.find(text);
    REQUIRE_MESSAGE(at != std::string::npos, "the fixture no longer contains: ", text);
    std::uint32_t line = 1;
    for (std::size_t i = 0; i < at; ++i) {
        line += file[i] == '\n' ? 1U : 0U;
    }
    return line;
}

bool endsWith(const char* s, const char* suffix) {
    if (s == nullptr) {
        return false;
    }
    const std::string a{s};
    const std::string b{suffix};
    return a.size() >= b.size() && a.compare(a.size() - b.size(), b.size(), b) == 0;
}

using namespace shulib::mechanism::words;

/// An arm of `cartridge` through gears(driving, driven), nothing tuned — the starting values' case.
template <shulib::hal::MotorCartridge C, int Driving, int Driven>
constexpr MechanismSpec startingArm() {
    constexpr Arm a{.name = "probe", .motors = {Forward}, .cartridge = C, .gearing = gears(Driving, Driven), .safe = Hold,
                    .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0, 90},
                    .gravity = noGravity, .home = restsAtStop(0)};
    return a.spec();
}

/// The same for a flywheel.
template <shulib::hal::MotorCartridge C, int Driving, int Driven>
constexpr MechanismSpec startingFlywheel() {
    constexpr Flywheel f{.name = "probe", .motors = {Forward}, .cartridge = C, .gearing = gears(Driving, Driven),
                         .safe = Coast, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s},
                         .atSpeed = within(50, 0.1_s)};
    return f.spec();
}

void checkStartingArm(const MechanismSpec& s, double cartridgeRpm, double reduction) {
    const double freeDegS = cartridgeRpm * 6.0 / reduction;  // output degrees per second at 12 V
    CHECK(specRefusal(s) == Refusal::None);
    CHECK(s.position.gearRatio == doctest::Approx(reduction).epsilon(1e-15));
    CHECK(s.position.kP == 0.25);
    CHECK(s.position.kI == 0.3);
    CHECK(s.position.kD == 0.004);
    CHECK(s.position.kS == 0.0);
    CHECK(s.position.kA == 0.0);
    CHECK(s.position.kV == doctest::Approx(12.0 / freeDegS).epsilon(1e-12));
    CHECK(s.position.maxVelocity == doctest::Approx(0.75 * freeDegS).epsilon(1e-12));
    CHECK(s.position.maxAcceleration == doctest::Approx(0.75 * freeDegS / 0.25).epsilon(1e-12));
    CHECK(s.position.tolerance == 1.5);
    CHECK(s.position.settleTime == 0.1);
}

void checkStartingFlywheel(const MechanismSpec& s, double cartridgeRpm, double reduction) {
    const double freeRpm = cartridgeRpm / reduction;
    CHECK(specRefusal(s) == Refusal::None);
    CHECK(s.speed.maxRpm == doctest::Approx(freeRpm).epsilon(1e-12));
    CHECK(s.speed.kV == doctest::Approx(12.0 / rpmToRadS(freeRpm)).epsilon(1e-12));
    CHECK(s.speed.kP == 0.05);
    CHECK(s.speed.kI == 0.2);
    CHECK(s.speed.kS == 0.0);
}

}  // namespace

TEST_CASE("MECH-A1 words: robot x's arm, field by field") {
    const MechanismSpec s = rx::arm.spec();
    CHECK(std::string{s.name} == "arm");
    CHECK(s.kind == Kind::Position);
    CHECK(s.motorCount == 1);
    CHECK_FALSE(s.motors[0].reversed);
    CHECK(s.lineCount == 0);
    CHECK(s.cartridge == shulib::hal::MotorCartridge::Green);
    CHECK(s.safeMode == shulib::config::SafeBrakeMode::Hold);
    CHECK(s.angleSensor.kind == SensorKind::None);
    CHECK(s.homeSwitch.kind == SensorKind::None);
    CHECK(s.pieceSensor.kind == SensorKind::None);
    const PositionSpec& p = s.position;
    CHECK(p.units == Units::Degrees);
    CHECK(p.geometry == Geometry::Rotary);
    CHECK(p.gearRatio == 5.0);  // gears(12, 60): FIVE motor turns per arm turn
    CHECK(p.minPosition == 0.0);
    CHECK(p.maxPosition == 160.0);
    CHECK(p.gravity == Gravity::Cosine);
    CHECK(p.kG == 1.6);
    CHECK(p.horizontalAt == 45.0);
    CHECK(p.home == HomeMethod::RestsAtStop);
    CHECK(p.homePosition == 0.0);
    CHECK(p.homeVolts == 0.0);
    CHECK_FALSE(p.homeStall.written());
    CHECK(p.stall.amps == 2.0);
    CHECK(p.stall.speedRadS == doctest::Approx(rpmToRadS(10.0)).epsilon(1e-15));
    CHECK(p.stall.time == 0.5);
    CHECK(p.unhomedVolts == 0.0);
    REQUIRE(p.presetCount == 3);
    CHECK(std::string{p.presets[0].name} == "stow");
    CHECK(p.presets[0].value == 0.0);
    CHECK(std::string{p.presets[1].name} == "score");
    CHECK(p.presets[1].value == 95.0);
    CHECK(std::string{p.presets[2].name} == "high");
    CHECK(p.presets[2].value == 140.0);
    // The starting values for Green through 5:1: 200 rpm x 6 / 5 = 240 deg/s free.
    CHECK(p.kV == doctest::Approx(0.05).epsilon(1e-12));
    CHECK(p.maxVelocity == doctest::Approx(180.0).epsilon(1e-12));
    CHECK(specRefusal(s) == Refusal::None);
}

TEST_CASE("MECH-A1 words: robot x's lift, flywheel, intake and hood, field by field") {
    SUBCASE("the two-motor lift in inches") {
        const MechanismSpec s = rx::lift.spec();
        CHECK(s.kind == Kind::Position);
        REQUIRE(s.motorCount == 2);
        CHECK_FALSE(s.motors[0].reversed);
        CHECK(s.motors[1].reversed);  // `Reversed`: the mirrored right-hand motor
        CHECK(s.cartridge == shulib::hal::MotorCartridge::Red);
        CHECK(s.position.units == Units::Inches);
        CHECK(s.position.geometry == Geometry::Spool);
        CHECK(s.position.spoolDiameterIn == 1.5);
        CHECK(s.position.gearRatio == 1.0);  // direct
        CHECK(s.position.maxPosition == 24.0);
        CHECK(s.position.gravity == Gravity::Constant);
        CHECK(s.position.kG == 1.0);
        CHECK(s.position.home == HomeMethod::Switch);
        CHECK(s.position.homeVolts == -3.0);
        CHECK(s.homeSwitch.kind == SensorKind::Switch);  // implied by homeBySwitch
        CHECK(s.homeSwitch.pressedLevel);                 // HA-184's claim
        REQUIRE(s.position.presetCount == 3);
        CHECK(s.position.presets[2].value == 22.0);
        // The starting cruise: Red direct = 600 deg/s at the spool; 75 % through pi x 1.5 / 360 in/deg
        // — the brief's measured "5.89 in/s".
        CHECK(s.position.maxVelocity == doctest::Approx(0.75 * 600.0 * kPi * 1.5 / 360.0).epsilon(1e-12));
        CHECK(s.position.maxVelocity == doctest::Approx(5.89).epsilon(1e-3));
        CHECK(s.position.tolerance == 0.25);  // a lift's starting "reached": 0.25 in
    }
    SUBCASE("the flywheel geared UP 6:1") {
        const MechanismSpec s = rx::flywheel.spec();
        CHECK(s.kind == Kind::Speed);
        CHECK(s.speed.gearRatio == doctest::Approx(12.0 / 72.0).epsilon(1e-15));
        CHECK(rx::flywheel.freeRpm() == doctest::Approx(3600.0).epsilon(1e-12));
        CHECK(s.speed.maxRpm == doctest::Approx(3600.0).epsilon(1e-12));
        CHECK(s.speed.atSpeedBandRpm == 60.0);
        CHECK(s.speed.atSpeedTime == 0.1);
        CHECK(s.speed.stall.speedRadS == doctest::Approx(rpmToRadS(30.0)).epsilon(1e-15));
        REQUIRE(s.speed.presetCount == 1);
        CHECK(std::string{s.speed.presets[0].name} == "shoot");
        CHECK(s.speed.presets[0].value == 3000.0);
        CHECK(s.safeMode == shulib::config::SafeBrakeMode::Coast);
    }
    SUBCASE("the intake with its distance sensor") {
        const MechanismSpec s = rx::intake.spec();
        CHECK(s.kind == Kind::Power);
        CHECK(s.pieceSensor.kind == SensorKind::Distance);
        CHECK(s.pieceSensor.seesWithinIn == 2.0);
        CHECK(s.power.maxVolts == 12.0);
        CHECK_FALSE(s.power.stall.written());  // optional on a power: none written
    }
    SUBCASE("the three-position hood: each position's level on each line") {
        const MechanismSpec s = rx::hood.spec(2);
        CHECK(s.kind == Kind::Pneumatic);
        CHECK(s.motorCount == 0);
        CHECK(s.lineCount == 2);
        REQUIRE(s.pneumatic.positionCount == 3);
        CHECK(std::string{s.pneumatic.positions[0].name} == "down");
        CHECK_FALSE(s.pneumatic.positions[0].levels[0]);
        CHECK_FALSE(s.pneumatic.positions[0].levels[1]);
        CHECK(s.pneumatic.positions[1].levels[0]);
        CHECK_FALSE(s.pneumatic.positions[1].levels[1]);
        CHECK(s.pneumatic.positions[2].levels[0]);
        CHECK(s.pneumatic.positions[2].levels[1]);
        for (std::size_t i = 0; i < 3; ++i) {
            CHECK(s.pneumatic.positions[i].levelCount == 2);
            CHECK(s.pneumatic.positions[i].confirm == Confirm::None);
        }
        CHECK(s.pneumatic.start == 0);
        CHECK(s.pneumatic.safe == 0);
        CHECK(s.pneumatic.startNamed);
        CHECK(s.pneumatic.safeNamed);
        CHECK(s.pneumatic.actuationTime == 0.3);
        CHECK(s.pneumatic.confirmWindow == 0.0);
        CHECK(s.pneumatic.levelsSource == nullptr);  // hand-written: the declaration's line is named
    }
}

TEST_CASE("MECH-A1 words: robot two's claws -- the claw's guess is ON its positions line") {
    const MechanismSpec p6 = r2::clawPivot6.spec();
    CHECK(p6.kind == Kind::Power);
    CHECK(std::string{p6.name} == "claw pivot 6");
    CHECK(p6.cartridge == shulib::hal::MotorCartridge::Red);
    CHECK(p6.safeMode == shulib::config::SafeBrakeMode::Hold);
    CHECK_FALSE(p6.power.stall.written());  // no signature: the driver is the only cut-off (HA-154)
    const MechanismSpec c = r2::claw.spec(1);
    CHECK(c.kind == Kind::Pneumatic);
    REQUIRE(c.pneumatic.positionCount == 2);
    CHECK(std::string{c.pneumatic.positions[0].name} == "closed");
    CHECK_FALSE(c.pneumatic.positions[0].levels[0]);  // closed = Low: HA-159's guess
    CHECK(c.pneumatic.positions[0].confirm == Confirm::Sees);  // `grabs`
    CHECK(std::string{c.pneumatic.positions[1].name} == "open");
    CHECK(c.pneumatic.positions[1].levels[0]);
    CHECK(c.pneumatic.start == 0);
    CHECK(c.pneumatic.safe == 0);
    CHECK(c.pneumatic.actuationTime == 0.5);
    CHECK(c.pieceSensor.kind == SensorKind::None);
}

TEST_CASE("MECH-A1 words: robot y -- every OPTIONAL word sets exactly its field (P5b)") {
    SUBCASE("an absolute wrist: a REVERSED rotation sensor, unhomedVolts and .tuning") {
        const MechanismSpec s = ry::wrist.spec();
        CHECK(s.angleSensor.kind == SensorKind::Rotation);
        CHECK(s.angleSensor.reversed);
        CHECK(s.position.home == HomeMethod::Absolute);
        CHECK(s.position.homePosition == 0.0);
        CHECK(s.position.absoluteSensorAtZeroDeg == 180.0);
        CHECK(s.position.unhomedVolts == 4.0);
        CHECK(s.position.gearRatio == 3.0);
        CHECK(s.position.stall.time == doctest::Approx(0.15).epsilon(1e-15));
        // .tuning = {.kP = 0.3, .within = 2.0}: EXACTLY those two replaced (the brief's P5b numbers)
        CHECK(s.position.kP == 0.3);
        CHECK(s.position.tolerance == 2.0);
        // ... and nothing else: a twin with no .tuning differs in those two fields only.
        constexpr Arm untuned{.name = "wrist", .motors = {Forward}, .cartridge = Green, .gearing = gears(12, 36),
                              .angleSensor = rotationSensor(Reversed), .safe = Hold,
                              .stall = {.amps = 1.6, .belowMotorRpm = 3, .lasting = 0.15_s}, .range = {-90, 90},
                              .gravity = noGravity, .home = absoluteSensor(0, 180), .presets = {{"flat", 0}},
                              .unhomedVolts = 4_volt};
        const MechanismSpec u = untuned.spec();
        CHECK(u.position.kP == 0.25);
        CHECK(u.position.tolerance == 1.5);
        CHECK(u.position.kI == s.position.kI);
        CHECK(u.position.kD == s.position.kD);
        CHECK(u.position.kS == s.position.kS);
        CHECK(u.position.kV == s.position.kV);
        CHECK(u.position.kA == s.position.kA);
        CHECK(u.position.maxVelocity == s.position.maxVelocity);
        CHECK(u.position.maxAcceleration == s.position.maxAcceleration);
        CHECK(u.position.settleTime == s.position.settleTime);
    }
    SUBCASE("a stall-homed arm") {
        const MechanismSpec s = ry::swing.spec();
        CHECK(s.safeMode == shulib::config::SafeBrakeMode::Brake);
        CHECK(s.position.home == HomeMethod::Stall);
        CHECK(s.position.homeVolts == -3.0);
        CHECK(s.position.homeStall.amps == 0.45);
        CHECK(s.position.homeStall.speedRadS == doctest::Approx(rpmToRadS(2.0)).epsilon(1e-15));
        CHECK(s.position.homeStall.time == doctest::Approx(0.2).epsilon(1e-15));
        CHECK(s.position.gravity == Gravity::Cosine);
        CHECK(s.position.horizontalAt == 30.0);
        CHECK(s.position.gearRatio == 7.0);
    }
    SUBCASE("a DR4B linkage") {
        const MechanismSpec s = ry::dr4b.spec();
        CHECK(s.position.geometry == Geometry::Arm);
        CHECK(s.position.armLengthIn == 14.0);
        CHECK(s.position.pivotHeightIn == 10.0);
        CHECK(s.position.armStages == 2);
        CHECK(s.motors[1].reversed);
        CHECK(s.position.home == HomeMethod::RestsAtStop);
        // Its STARTING profile, from first principles (MECH-A1's review: no case pinned a linkage's):
        // Red through 5:1 is 100 x 6 / 5 = 120 output deg/s free; a linkage's steepest slope, at level,
        // is stages x arm x pi/180 in/deg = 2 x 14 x pi/180; 75 % of that is the cruise, reached in 0.25 s.
        const double inPerDeg = 2.0 * 14.0 * kPi / 180.0;
        CHECK(s.position.maxVelocity == doctest::Approx(0.75 * 120.0 * inPerDeg).epsilon(1e-12));
        CHECK(s.position.maxVelocity == doctest::Approx(14.0 * kPi).epsilon(1e-12));  // 43.98 in/s
        CHECK(s.position.maxAcceleration == doctest::Approx(0.75 * 120.0 * inPerDeg / 0.25).epsilon(1e-12));
        CHECK(s.position.tolerance == 0.25);
    }
    SUBCASE("a measured, never-homed tilt") {
        const MechanismSpec s = ry::tilt.spec();
        CHECK(s.position.geometry == Geometry::Table);
        REQUIRE(s.position.tablePoints == 3);
        CHECK(s.position.table[1][0] == 6.0);
        CHECK(s.position.table[1][1] == 30.0);
        CHECK(s.position.table[2][0] == 14.0);
        CHECK(s.position.table[2][1] == 70.0);
        CHECK(s.position.home == HomeMethod::None);
        CHECK(s.position.unhomedVolts == 3.0);
        // Its STARTING profile from its FIRST table segment: (6 - 0) in over (30 - 0) deg = 0.2 in/deg;
        // Red through 7:1 is 600 / 7 output deg/s free (an inverted scale would read 5 in/deg).
        const double freeDegS = 100.0 * 6.0 / 7.0;
        CHECK(s.position.maxVelocity == doctest::Approx(0.75 * freeDegS * 0.2).epsilon(1e-12));
        CHECK(s.position.maxAcceleration == doctest::Approx(0.75 * freeDegS * 0.2 / 0.25).epsilon(1e-12));
    }
    SUBCASE("an optical roller, a limit-switch catapult, a sensed grip") {
        CHECK(ry::roller.spec().pieceSensor.kind == SensorKind::Optical);
        CHECK(ry::roller.spec().pieceSensor.seesProximity == 0.5);
        const MechanismSpec cat = ry::catapult.spec();
        CHECK(cat.pieceSensor.kind == SensorKind::Switch);
        CHECK(cat.power.stall.amps == 2.0);
        CHECK(cat.power.stall.time == doctest::Approx(0.2).epsilon(1e-15));
        const MechanismSpec g = ry::grip.spec(1);
        CHECK(g.pneumatic.positions[0].confirm == Confirm::Sees);         // grabs
        CHECK(g.pneumatic.positions[1].confirm == Confirm::StopsSeeing);  // releases
        CHECK(g.pneumatic.start == 1);                                     // "loose"
        CHECK(g.pneumatic.safe == 0);                                      // "shut"
        CHECK(g.pieceSensor.kind == SensorKind::Distance);
        CHECK(g.pieceSensor.seesWithinIn == 1.5);
        CHECK(g.pneumatic.confirmWindow == doctest::Approx(0.2).epsilon(1e-15));
    }
    for (const MechanismSpec& s : ry::kRobotYMechanisms.specSpan()) {
        CAPTURE(s.name);
        CHECK(specRefusal(s) == Refusal::None);
    }
}

TEST_CASE("MECH-A1 words: .tuning -- EVERY field stated lands in the spec, each its own (N7's 'overridable', one by one)") {
    // Each value distinct from every starting value and from each other, so a field read into the
    // wrong place, or ignored, is a red line (MECH-A1's review: only kP and within were pinned).
    constexpr Arm tuned{.name = "tuned", .motors = {Forward}, .cartridge = Green, .gearing = gears(12, 60), .safe = Hold,
                        .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0, 160},
                        .gravity = noGravity, .home = restsAtStop(0),
                        .tuning = {.kP = 0.31, .kI = 0.41, .kD = 0.051, .kS = 0.61, .kV = 0.071, .kA = 0.081,
                                   .maxSpeed = 91.0, .maxAcceleration = 101.0, .within = 1.1, .settle = 0.21_s}};
    const PositionSpec p = tuned.spec().position;
    CHECK(p.kP == 0.31);
    CHECK(p.kI == 0.41);
    CHECK(p.kD == 0.051);
    CHECK(p.kS == 0.61);
    CHECK(p.kV == 0.071);
    CHECK(p.kA == 0.081);
    CHECK(p.maxVelocity == 91.0);
    CHECK(p.maxAcceleration == 101.0);
    CHECK(p.tolerance == 1.1);
    CHECK(p.settleTime == doctest::Approx(0.21).epsilon(1e-15));
    CHECK(specRefusal(tuned.spec()) == Refusal::None);
    // A stated top speed alone: the acceleration's starting value follows IT (reached in 0.25 s).
    constexpr Arm fast{.name = "fast", .motors = {Forward}, .cartridge = Green, .gearing = gears(12, 60), .safe = Hold,
                       .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0, 160},
                       .gravity = noGravity, .home = restsAtStop(0), .tuning = {.maxSpeed = 40.0}};
    CHECK(fast.spec().position.maxAcceleration == doctest::Approx(160.0).epsilon(1e-12));
    constexpr Flywheel spun{.name = "spun", .motors = {Forward}, .cartridge = Blue, .gearing = gears(72, 12), .safe = Coast,
                            .stall = {.amps = 2.0, .belowMotorRpm = 30, .lasting = 0.5_s}, .atSpeed = within(60, 0.1_s),
                            .tuning = {.kS = 0.11, .kV = 0.021, .kP = 0.031, .kI = 0.041}};
    const shulib::mechanism::SpeedSpec q = spun.spec().speed;
    CHECK(q.kS == 0.11);
    CHECK(q.kV == 0.021);
    CHECK(q.kP == 0.031);
    CHECK(q.kI == 0.041);
    CHECK(specRefusal(spun.spec()) == Refusal::None);
}

TEST_CASE("MECH-A1 words: the STARTING values (N7) for Red/Green/Blue x 1:1, 5:1, 7:1, from first principles") {
    using shulib::hal::MotorCartridge;
    checkStartingArm(startingArm<MotorCartridge::Red, 1, 1>(), 100.0, 1.0);
    checkStartingArm(startingArm<MotorCartridge::Red, 12, 60>(), 100.0, 5.0);
    checkStartingArm(startingArm<MotorCartridge::Red, 12, 84>(), 100.0, 7.0);
    checkStartingArm(startingArm<MotorCartridge::Green, 1, 1>(), 200.0, 1.0);
    checkStartingArm(startingArm<MotorCartridge::Green, 12, 60>(), 200.0, 5.0);
    checkStartingArm(startingArm<MotorCartridge::Green, 12, 84>(), 200.0, 7.0);
    checkStartingArm(startingArm<MotorCartridge::Blue, 1, 1>(), 600.0, 1.0);
    checkStartingArm(startingArm<MotorCartridge::Blue, 12, 60>(), 600.0, 5.0);
    checkStartingArm(startingArm<MotorCartridge::Blue, 12, 84>(), 600.0, 7.0);
    checkStartingFlywheel(startingFlywheel<MotorCartridge::Red, 1, 1>(), 100.0, 1.0);
    checkStartingFlywheel(startingFlywheel<MotorCartridge::Green, 12, 60>(), 200.0, 5.0);
    checkStartingFlywheel(startingFlywheel<MotorCartridge::Blue, 72, 12>(), 600.0, 12.0 / 72.0);
}

TEST_CASE("MECH-A1 words: the GEARING is tooth counts, driving then driven -- stages multiply") {
    static_assert(gears(12, 60).motorTurnsPerTurn == 5.0);
    static_assert(gears(60, 12).motorTurnsPerTurn == 0.2);
    static_assert(direct.motorTurnsPerTurn == 1.0);
    static_assert(gears(1, 1).motorTurnsPerTurn == direct.motorTurnsPerTurn);
    static_assert((gears(12, 36) * gears(12, 60)).motorTurnsPerTurn == 15.0);
    static_assert(!std::is_convertible_v<int, shulib::mechanism::Gearing>);    // `.gearing = 5`: n41
    static_assert(!std::is_convertible_v<double, shulib::mechanism::Gearing>);
    CHECK(gears(12, 60).motorTurnsPerTurn == 5.0);
}

TEST_CASE("MECH-A1 words: WHERE each declaration and each wire(...) was written -- its own line, read from the file") {
    const auto& t = rx::kRobotXMechanisms;
    struct Pin {
        const MechanismSpec& spec;
        const char* nameLine;
    };
    for (const Pin& pin : {Pin{t.specs[0], "    .name = \"arm\","}, Pin{t.specs[1], "    .name = \"lift\","},
                           Pin{t.specs[2], "    .name = \"flywheel\","}, Pin{t.specs[3], "    .name = \"intake\","},
                           Pin{t.specs[4], "    .name = \"hood\","}}) {
        CAPTURE(pin.nameLine);
        const std::uint32_t expected = fixtureLineOf(pin.nameLine);
        CHECK(pin.spec.declaredLine == expected);
        CHECK(endsWith(pin.spec.declaredFile, "test/mechanism_fixtures.hpp"));  // the SUFFIX: host and brain differ
        for (std::size_t m = 0; m < pin.spec.motorCount; ++m) {
            CHECK(pin.spec.motors[m].line == expected);  // every motor role points at the `.name` line
        }
    }
    CHECK(t.specs[4].pneumatic.levelsLine == fixtureLineOf("    .name = \"hood\","));
    CHECK(t.specs[1].homeSwitch.line == fixtureLineOf("    .name = \"lift\","));
    // A sensor word records where IT was written.
    CHECK(t.specs[3].pieceSensor.line == fixtureLineOf("    .pieceSensor = distanceSensor(2_in),"));
    // The wiring: each mechanism's wire(...) line, captured where motors(...) / lines(...) is written.
    CHECK(t.wiring[0].line == fixtureLineOf("    wire(arm, motors(1)),"));
    CHECK(t.wiring[1].line == fixtureLineOf("    wire(lift, motors(2, 3), homeSwitch('B')),"));
    CHECK(t.wiring[4].line == fixtureLineOf("    wire(hood, lines('C', 'D')));"));
    CHECK(endsWith(t.wiring[4].file, "test/mechanism_fixtures.hpp"));
    // Robot two's one-line declarations: the line they are on.
    CHECK(r2::kRobotTwoMechanisms.specs[0].declaredLine ==
          fixtureLineOf("inline constexpr Power clawPivot6{.name = \"claw pivot 6\""));
    CHECK(r2::kRobotTwoMechanisms.specs[2].declaredLine == fixtureLineOf("    .name = \"claw\","));
}

TEST_CASE("MECH-A1 words: a name past 23 characters and a preset name past 15 are refused, never cut silently") {
    constexpr Power ok23{.name = "abcdefghijklmnopqrstuvw", .motors = {Forward}, .cartridge = Red, .safe = Hold};
    constexpr Power long24{.name = "abcdefghijklmnopqrstuvwx", .motors = {Forward}, .cartridge = Red, .safe = Hold};
    CHECK(specRefusal(ok23.spec()) == Refusal::None);
    CHECK(std::string{ok23.spec().name} == "abcdefghijklmnopqrstuvw");
    CHECK(specRefusal(long24.spec()) == Refusal::AMechanismNameDoesNotFit23Characters);
    constexpr Arm preset15{.name = "a", .motors = {Forward}, .cartridge = Red, .gearing = direct, .safe = Hold,
                           .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0, 90},
                           .gravity = noGravity, .home = restsAtStop(0), .presets = {{"fifteen_chars__", 0}}};
    constexpr Arm preset16{.name = "a", .motors = {Forward}, .cartridge = Red, .gearing = direct, .safe = Hold,
                           .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0, 90},
                           .gravity = noGravity, .home = restsAtStop(0), .presets = {{"sixteen_chars___", 0}}};
    CHECK(specRefusal(preset15.spec()) == Refusal::None);
    CHECK(std::string{preset15.spec().position.presets[0].name} == "fifteen_chars__");
    CHECK(specRefusal(preset16.spec()) == Refusal::APresetOrPositionNameDoesNotFit15Characters);
}

TEST_CASE("MECH-A1 words: too many roles, levels, positions or table points are COUNTED, so the list refuses them") {
    constexpr Power five{.name = "five", .motors = {Forward, Forward, Forward, Forward, Forward}, .cartridge = Red,
                         .safe = Hold};
    CHECK(five.spec().motorCount == 5);
    CHECK(specRefusal(five.spec()) == Refusal::AMotorMechanismNeeds14MotorRoles);
    constexpr Pneumatic four{.name = "four",
                             .positions = {{"a", {Low}}, {"b", {High}}, {"c", {Low}}, {"d", {High}}},
                             .start = "a", .safe = "a", .moveTime = 0.2_s};
    CHECK(four.spec(1).pneumatic.positionCount == 4);
    CHECK(specRefusal(four.spec(1)) == Refusal::Pneumatic2Or3Positions);
    constexpr shulib::mechanism::Levels three{true, false, true};
    CHECK(three.tooMany);
    CHECK(three.count == 2);
    constexpr Pneumatic threeLevels{.name = "three", .positions = {{"a", {Low, Low, High}}, {"b", {High, High, High}}},
                                    .start = "a", .safe = "a", .moveTime = 0.2_s};
    CHECK(threeLevels.spec(2).pneumatic.positions[0].levelCount == 3);  // the wiring refuses 3 != 2 (U2)
    constexpr HeightAt nine[] = {{0_in, 0}, {1_in, 1}, {2_in, 2}, {3_in, 3}, {4_in, 4},
                                 {5_in, 5}, {6_in, 6}, {7_in, 7}, {8_in, 8}};
    constexpr Lift tall{.name = "tall", .motors = {Forward}, .cartridge = Red, .gearing = direct, .shape = measured(nine),
                        .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s},
                        .range = {0_in, 8_in}, .gravity = noGravity, .home = restsAtStop(0_in)};
    CHECK(tall.spec().position.tablePoints == 9);
    CHECK(specRefusal(tall.spec()) == Refusal::PositionALookupTableNeeds28Points);
    CHECK(linkage(14_in, 10_in, -1).stages == 0);   // a negative stage count never wraps to 255
    CHECK(linkage(14_in, 10_in, 300).stages == 255);
}

TEST_CASE("MECH-A1 words: the CONVERSIONS round-trip within 1e-12, and clamp past their range (P2g)") {
    SUBCASE("rotary: the identity") {
        const PositionSpec& p = rx::arm.spec().position;
        for (double d = -200.0; d <= 200.0; d += 7.5) {
            CHECK(shulib::mechanism::angleDegToPosition(p, d) == d);
            CHECK(shulib::mechanism::positionToAngleDeg(p, d) == d);
        }
    }
    SUBCASE("spool: across and past its limits") {
        const PositionSpec& p = rx::lift.spec().position;
        for (double x = -10.0; x <= 34.0; x += 0.25) {
            const double deg = shulib::mechanism::positionToAngleDeg(p, x);
            CHECK(std::abs(shulib::mechanism::angleDegToPosition(p, deg) - x) < 1e-12);
        }
        // one spool turn is pi x 1.5 in (the bench page's reading — the brief's §3.6)
        CHECK(shulib::mechanism::angleDegToPosition(p, 360.0) == doctest::Approx(kPi * 1.5).epsilon(1e-15));
    }
    SUBCASE("linkage: 1, 2 and 3 stages, across their range; past it the asin clamps") {
        for (std::uint8_t stages = 1; stages <= 3; ++stages) {
            PositionSpec p = ry::dr4b.spec().position;
            p.armStages = stages;
            const double reach = static_cast<double>(stages) * p.armLengthIn;
            for (double h = p.pivotHeightIn - reach; h <= p.pivotHeightIn + reach; h += reach / 40.0) {
                const double deg = shulib::mechanism::positionToAngleDeg(p, h);
                CAPTURE(h);
                CHECK(std::abs(shulib::mechanism::angleDegToPosition(p, deg) - h) < 1e-12);
            }
            CHECK(shulib::mechanism::positionToAngleDeg(p, p.pivotHeightIn + reach + 5.0) == p.horizontalAt + 90.0);
            CHECK(shulib::mechanism::positionToAngleDeg(p, p.pivotHeightIn - reach - 5.0) == p.horizontalAt - 90.0);
        }
    }
    SUBCASE("measured: across its rows; past either end it clamps") {
        const PositionSpec& p = ry::tilt.spec().position;
        for (double h = 0.0; h <= 14.0; h += 0.125) {
            const double deg = shulib::mechanism::positionToAngleDeg(p, h);
            CHECK(std::abs(shulib::mechanism::angleDegToPosition(p, deg) - h) < 1e-12);
        }
        CHECK(shulib::mechanism::positionToAngleDeg(p, -3.0) == 0.0);
        CHECK(shulib::mechanism::positionToAngleDeg(p, 20.0) == 70.0);
        CHECK(shulib::mechanism::angleDegToPosition(p, -5.0) == 0.0);
        CHECK(shulib::mechanism::angleDegToPosition(p, 90.0) == 14.0);
        CHECK(shulib::mechanism::positionToAngleDeg(p, 10.0) == doctest::Approx(50.0).epsilon(1e-15));  // P2g's 10 in -> 50 deg
        PositionSpec one = p;
        one.tablePoints = 1;  // never a valid spec; the conversion stays total
        CHECK(shulib::mechanism::positionToAngleDeg(one, 3.0) == 0.0);
    }
}

TEST_CASE("MECH-A1 words: the goals a declaration makes, and the words that convert both ways") {
    constexpr shulib::mechanism::PositionGoal score = rx::arm["score"];
    CHECK(score.mechanism == &rx::arm);
    CHECK(std::string{score.mechanismName} == "arm");
    CHECK(std::string{score.presetName} == "score");
    CHECK(score.value == 95.0);
    CHECK_FALSE(score.computed);
    // Every constant form AT ITS EXACT LIMIT compiles (the build check's positive half).
    constexpr shulib::mechanism::PositionGoal top = rx::arm.to(160);
    constexpr shulib::mechanism::PositionGoal bottom = rx::arm.to(0);
    constexpr shulib::mechanism::PositionGoal liftTop = rx::lift.to(24_in);
    constexpr shulib::mechanism::PositionGoal liftBottom = rx::lift.to(0_in);  // a lift AT its bottom compiles
    constexpr shulib::mechanism::SpeedGoal fastest = rx::flywheel.at(3600);
    constexpr shulib::mechanism::SpeedGoal reverse = rx::flywheel.at(-3600);
    CHECK(top.presetName == nullptr);
    CHECK(top.value == 160.0);
    CHECK(bottom.value == 0.0);
    CHECK(liftTop.value == 24.0);
    CHECK(liftBottom.value == 0.0);
    CHECK(fastest.rpm == 3600.0);
    CHECK(reverse.rpm == -3600.0);
    // "At speed" the instant it is in the band: a 0 s lasting is a declaration's own choice, accepted.
    static constexpr Flywheel instant{.name = "instant", .motors = {Forward}, .cartridge = Blue, .gearing = gears(72, 12),
                               .safe = Coast, .stall = {.amps = 2.0, .belowMotorRpm = 30, .lasting = 0.5_s},
                               .atSpeed = within(60, 0_s)};
    static constexpr shulib::mechanism::MechanismTable withInstant = mechanisms(wire(instant, motors(1)));
    CHECK(withInstant.specs[0].speed.atSpeedTime == 0.0);
    // A COMPUTED target is carried as-is and marked — the service clamps it (MECH-A3), never the build.
    const shulib::mechanism::PositionGoal past = rx::arm.clampedTo(175.0);
    CHECK(past.computed);
    CHECK(past.value == 175.0);
    CHECK(rx::lift.clampedTo(30_in).computed);
    // Every goal carries its mechanism's UNITS -- find() matches on them (MECH-A3): an arm's in degrees,
    // a lift's in inches, whichever word made it.
    CHECK(past.units == Units::Degrees);
    CHECK(score.units == Units::Degrees);
    CHECK(top.units == Units::Degrees);
    CHECK(liftTop.units == Units::Inches);
    CHECK(rx::lift["high"].units == Units::Inches);
    CHECK(rx::lift.clampedTo(30_in).units == Units::Inches);
    CHECK(rx::flywheel.clampedTo(5000.0).computed);
    constexpr shulib::mechanism::SpeedGoal shoot = rx::flywheel["shoot"];
    CHECK(shoot.rpm == 3000.0);
    constexpr shulib::mechanism::PneumaticGoal middle = rx::hood["middle"];
    CHECK(middle.position == 1);
    CHECK(std::string{middle.positionName} == "middle");
    CHECK(middle.mechanism == &rx::hood);
    constexpr shulib::mechanism::PneumaticGoal open = r2::claw["open"];
    CHECK(open.position == 1);
    CHECK(rx::lift["high"].value == 22.0);
    // ONE word for both brake modes (N11).
    const shulib::config::SafeBrakeMode safe = Hold;
    const shulib::hal::BrakeMode drive = Coast;
    CHECK(safe == shulib::config::SafeBrakeMode::Hold);
    CHECK(drive == shulib::hal::BrakeMode::Coast);
    CHECK(static_cast<shulib::config::SafeBrakeMode>(Brake) == shulib::config::SafeBrakeMode::Brake);
    CHECK(static_cast<shulib::hal::BrakeMode>(Brake) == shulib::hal::BrakeMode::Brake);
}
