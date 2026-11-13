// Guide chapter 18 ("Mechanisms: arms, lifts and flywheels"), compiled and run (cases guide-18a to
// guide-18c; chunk MECH-A3 — and guide-18g, the current-cap section's lift, chunk MECH-S-A). Every ```cpp line the chapter shows is a line of this file —
// tools/api_doc_tool.py's check-examples fails the build if one drifts — and each case below checks,
// on the host test plant (invented models, HA-183), what the chapter says that listing does.

#include "doctest.h"

#include <cmath>
#include <span>
#include <string>

#include "mechanism_position_rig.hpp"
#include "shulib/mechanism/kinds.hpp"
#include "shulib/mechanism/service.hpp"
#include "shulib/mechanism/wiring.hpp"

namespace chapter18 {
using namespace shulib::mechanism::words;

inline constexpr Lift lift{
    .name = "lift",
    .motors = {Forward, Reversed},        // the right-hand motor is mounted mirrored
    .cartridge = Red,
    .gearing = direct,
    .shape = spool(1.5_in),
    .safe = Hold,
    .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s},
    .range = {0_in, 24_in},
    .gravity = constantGravity(1.0_volt),
    .home = homeBySwitch(0_in, -3_volt),  // down at 3 V until its switch is pressed: that is 0 in
    .presets = {{"floor", 0_in}, {"low", 8_in}, {"high", 22_in}},
};
inline constexpr Flywheel flywheel{
    .name = "flywheel",
    .motors = {Forward},
    .cartridge = Blue,
    .gearing = gears(72, 12),             // geared UP 6:1, so 3600 rpm free
    .safe = Coast,
    .stall = {.amps = 2.0, .belowMotorRpm = 30, .lasting = 0.5_s},
    .atSpeed = within(60, 0.1_s),         // "at speed": within 60 rpm for 0.1 s
    .presets = {{"shoot", 3000}},
};
// The chapter's `.tuning` line, in an arm of its own (the chapter's arm is chapter 10's).
inline constexpr Arm tunedArm{
    .name = "arm",
    .motors = {Forward},
    .cartridge = Green,
    .gearing = gears(12, 60),
    .safe = Hold,
    .stall = {.amps = 2.0, .belowMotorRpm = 10, .lasting = 0.5_s},
    .range = {0, 160},
    .gravity = cosine(1.6_volt, 45),
    .home = restsAtStop(0),
    .tuning = {.kP = 0.3, .within = 2.0},
};
// The current-cap section's string lift (MECH-S-A).
inline constexpr Lift stringLift{
    .name = "string lift",
    .motors = {Forward, Reversed},
    .cartridge = Red,
    .gearing = direct,
    .shape = spool(1.5_in),
    .safe = Hold,
    .stall = {.amps = 0.9, .belowMotorRpm = 5, .lasting = 0.5_s},  // below the cap, with room
    .currentCap = {.amps = 1.2},                                    // each motor: at most 1.2 A
    .range = {0_in, 24_in},
    .gravity = constantGravity(1.0_volt),
    .home = restsAtStop(0_in),
};
}  // namespace chapter18

namespace {
namespace mech = shulib::mechanism;
namespace sim = shulib::sim;
namespace units = shulib::units;
using mech::MechanismOutcome;
}  // namespace

TEST_CASE("guide-18a: the lift homes by its switch, then goes to 'high' and HOLDS; the flywheel is at speed and KEEPS spinning") {
    // The lift's plant: the spool is the joint, constant gravity, its switch pressed at the bottom.
    sim::ArmPlantConfig pc = pos_rig::armPlant(100, 1, 1.0, 1.0, 0.3, 20.0, 0.0, -1.0, 24.5 / (3.14159265358979323846 * 1.5) * 360.0);
    pc.constantGravity = true;
    pos_rig::ArmRig r{chapter18::lift.spec(), pc, pos_rig::Wiring{.homeSwitch = sim::SwitchMount{.atDeg = 0.0}}};
    CHECK_FALSE(r.slot().homed());                                  // "before it does, a move is NOT_HOMED"
    r.slot().moveToPreset(2, units::Time{6.0});
    CHECK(r.slot().lastStep().outcome == MechanismOutcome::NotHomed);
    r.slot().home(units::Time{3.0});                                 // "a homing STEP pushes until it is"
    r.runStep();
    REQUIRE(r.slot().lastStep().outcome == MechanismOutcome::Succeeded);
    r.slot().moveToPreset(2, units::Time{6.0});                      // "high" = 22 in
    r.runStep();
    CHECK(r.slot().lastStep().outcome == MechanismOutcome::Succeeded);
    r.tick(100);
    CHECK(r.slot().position() == doctest::Approx(22.0).epsilon(0.01));  // "it goes on HOLDING there"
    CHECK(r.slot().holdsClaim());

    pos_rig::FlyRig f{chapter18::flywheel.spec(), pos_rig::flywheelPlant(600, 1.0 / 6.0, 1.0, 0.4, 0.2)};
    CHECK(chapter18::flywheel.freeRpm() == doctest::Approx(3600.0));  // "geared UP 6:1, so 3600 rpm free"
    f.slot().spinUpPreset(0, units::Time{2.0});
    for (int i = 0; i < 200 && f.slot().running(); ++i) {
        f.tick();
    }
    CHECK(f.slot().lastStep().outcome == MechanismOutcome::Succeeded);
    f.tick(100);
    CHECK(f.slot().spinning());                                      // "it keeps spinning until something stops it"
    CHECK(std::abs(f.slot().rpm() - 3000.0) <= 60.0);
}

TEST_CASE("guide-18b: .tuning replaces what it names; the boot line marks those '(tuned)' and keeps the rest's starting values") {
    const mech::MechanismSpec s = chapter18::tunedArm.spec();
    CHECK(s.position.kP == 0.3);
    CHECK(s.position.tolerance == 2.0);
    CHECK(s.position.kI == 0.3);  // a starting value: not named in .tuning
    char line[320];
    mech::startingValuesLine(line, sizeof line, s);
    CHECK(std::string{line} ==
          "arm: runs kP 0.300 (tuned) kI 0.300 kD 0.0040 kV 0.0500, cruise 180.0 deg/s in 0.25 s, reached within "
          "2.00 deg (tuned) for 0.10 s -- the rest are starting values, PROVISIONAL (HA-182)");
    // the chapter's verbatim boot line of an untuned arm
    mech::MechanismSpec untuned = s;
    const mech::PositionSpec st = mech::positionStartingValues(s);
    untuned.position.kP = st.kP;
    untuned.position.tolerance = st.tolerance;
    mech::startingValuesLine(line, sizeof line, untuned);
    CHECK(std::string{line} ==
          "arm: starting values kP 0.250 kI 0.300 kD 0.0040 kV 0.0500, cruise 180.0 deg/s in 0.25 s, reached within "
          "1.50 deg for 0.10 s -- PROVISIONAL (HA-182); .tuning overrides");
}

TEST_CASE("guide-18c: the chapter's log lines -- a computed target clamped, once; a home that says where its zero went") {
    pos_rig::ArmRig r{chapter18::tunedArm.spec(), pos_rig::armPlant(200, 5, 1.6, 1.0, 0.3)};
    CHECK(r.mechLines().front() == "'arm': homed -- +0.0 deg from where its encoders read zero at boot");
    r.slot().moveTo(175.0, units::Time{3.0}, /*computed=*/true);
    r.runStep();
    CHECK(r.mechCount("'arm': target 175.00 clamped to the travel limit 160.00 (soft stop)") == 1);
    CHECK(r.slot().lastStep().outcome == MechanismOutcome::Succeeded);
}

TEST_CASE("guide-18g: the current-cap section's lift -- its cap on BOTH motors, read back, said once; no margin Warn (0.3 A under)") {
    pos_rig::ArmRig r{chapter18::stringLift.spec(), pos_rig::armPlant(100, 1, 1.0, 1.0, 0.3)};
    CHECK(r.motors[0].currentLimit().value() == 1.2);
    CHECK(r.motors[1].currentLimit().value() == 1.2);
    // the chapter's boot line, verbatim (bound from bare specs here, so no port is named — none is in it)
    CHECK(r.mechCount("'string lift': current cap 1.20 A per motor (2 motors: 2.40 A in all) -- set and read back") ==
          1);
    CHECK(r.mechCount("is within 0.2 A of its current cap") == 0);
}
