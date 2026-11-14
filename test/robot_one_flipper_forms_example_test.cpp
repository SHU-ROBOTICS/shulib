// ROBOT ONE'S FLIPPER — ALL THREE FORMS, READY (chunk ROB1-A, 2026-11-11; the ROB1 brief's D6, test T13). Say
// "host test plant", never "simulator". The builder still owes the flipper's KIND (AF1-S's D7): robot one's
// file declares NO flipper until he names it; then the matching block below is pasted under `cascade` in
// src/robots/mechanism_table.hpp (and its line into src/team/robot_one_driver.hpp), its numbers typed into
// the facts ledger first (a G1-F edit) — guide chapter 21 has the paste-ready text. The flipper's own numbers
// here (a gearing, angles, a stroke time) are THIS TEST'S, invented: none is robot one's. Nothing here has
// run on a robot.
//
// WHAT THIS FILE TRIES TO BREAK
//   * EACH FORM WITH ROBOT ONE'S REAL CASCADE (T13): the list compiles (every rule the library checks), the
//     flipper's typed kind agrees with it and the cascade's does not stop it (robot one's two fact checks),
//     its default control compiles beside the cascade's map (Y free, no button twice), it BINDS over the fake
//     provider, and a blind flip runs: the arm reaches "flip" on the host test plant; the pneumatic's lines
//     go to "flip" at once and the step succeeds after its move time; the passive bar declares nothing.
//   * THE ROW (D6): the Drive screen's mechanism row fits the SMALL budget with each form.
//   * GM24: the pneumatic's `.moveTime` removed — the build refuses it by name (the library's spec refusal;
//     test/compile_refusals/s82 pins it outside this file so the suite still builds).

#include "doctest.h"

#include <cstddef>
#include <string_view>

#include "common/driver_panel.hpp"
#include "mechanism_position_rig.hpp"
#include "mechanism_service_rig.hpp"
#include "robots/mechanism_table.hpp"
#include "shulib/mechanism/driver_inputs.hpp"
#include "shulib/mechanism/kinds.hpp"
#include "shulib/mechanism/service.hpp"
#include "shulib/mechanism/wiring.hpp"
#include "shulib/units/literals.hpp"

namespace sb = shulib::bench;
namespace facts = sb::robot_one_facts;
using shulib::hal::ControllerButton;
using shulib::mechanism::MechanismOutcome;
using shulib::units::Time;

// ═══ THE THREE FORMS, as guide chapter 21 shows them (the flipper's numbers invented — this test's) ════
namespace flipper_forms {
using namespace shulib::mechanism::words;
using shulib::bench::robot_one::cascade;
using Button = shulib::hal::ControllerButton;

// (1) A MOTOR ARM on port 10: stowed at 0 degrees, flipped at 95, resting on its stop when stowed.
inline constexpr Arm flipperArm{.name = "flipper", .motors = {Forward}, .cartridge = Green, .gearing = gears(12, 36),
                                .safe = Hold, .stall = {.amps = 1.0, .belowMotorRpm = 5, .lasting = 0.3_s},
                                .currentCap = {.amps = 1.5}, .range = {0, 120}, .gravity = noGravity,
                                .home = restsAtStop(0), .presets = {{"stow", 0}, {"flip", 95}}};
inline constexpr MechanismTable kWithArm = mechanisms(
    wire(cascade, motors(facts::kCascadePorts.value().first, facts::kCascadePorts.value().second)),
    wire(flipperArm, motors(10)));
inline constexpr DriverControls kArmMap = controls(kWithArm,
    whileHeld(cascade, Button::L1, Button::L2, 12_volt),
    preset(cascade[facts::lv(0).name], Button::Down, 2_s), preset(cascade[facts::lv(1).name], Button::Left, 2_s),
    preset(cascade[facts::lv(2).name], Button::Up, 2_s), preset(cascade[facts::lv(3).name], Button::Right, 2_s),
    homeOnPress(cascade, Button::B, 3_s), homeAtDriverStart(cascade, 3_s),
    whileHeld(flipperArm, Button::R1, Button::R2, 8_volt));  // the flipper: R1 out / R2 back while held

// (2) A PNEUMATIC on 3-wire A: stowed at the start and when made safe; its measured stroke time.
inline constexpr Pneumatic flipperAir{.name = "flipper", .positions = {{"stow", {Low}}, {"flip", {High}}},
                                      .start = "stow", .safe = "stow", .moveTime = 0.4_s};
inline constexpr MechanismTable kWithAir = mechanisms(
    wire(cascade, motors(facts::kCascadePorts.value().first, facts::kCascadePorts.value().second)),
    wire(flipperAir, lines('A')));
inline constexpr DriverControls kAirMap = controls(kWithAir,
    whileHeld(cascade, Button::L1, Button::L2, 12_volt),
    preset(cascade[facts::lv(0).name], Button::Down, 2_s), preset(cascade[facts::lv(1).name], Button::Left, 2_s),
    preset(cascade[facts::lv(2).name], Button::Up, 2_s), preset(cascade[facts::lv(3).name], Button::Right, 2_s),
    homeOnPress(cascade, Button::B, 3_s), homeAtDriverStart(cascade, 3_s),
    toggle(flipperAir, Button::R1));  // the flipper: each R1 press flips or stows it

// (3) A PASSIVE BAR: nothing to declare — robot one's own list and map are the form.
inline constexpr const MechanismTable& kPassive = sb::robot_one::kRobotOneMechanisms;
}  // namespace flipper_forms

namespace ff = flipper_forms;

// Each form agrees with its typed kind, and its motors are all declared (the two fact checks, at build).
static_assert(sb::flipperKindMatches(facts::FlipperKind::MotorArm, ff::kWithArm));
static_assert(sb::flipperKindMatches(facts::FlipperKind::Pneumatic, ff::kWithAir));
static_assert(sb::flipperKindMatches(facts::FlipperKind::PassiveBar, ff::kPassive));
static_assert(!sb::flipperKindMatches(facts::FlipperKind::Pneumatic, ff::kWithArm));
static_assert(!sb::flipperKindMatches(facts::FlipperKind::MotorArm, ff::kWithAir));
static_assert(sb::everyListedMotorDeclared(facts::PortList{{facts::kCascadePorts.value().first,
                                                            facts::kCascadePorts.value().second, 10}, 3},
                                           ff::kWithArm));
// The arm's motor on port 10 is NOT declared by robot one's list as it stands: listing it on the Devices
// screen before its block is pasted stops every robot-one build (landmine 4).
static_assert(!sb::everyListedMotorDeclared(facts::PortList{{facts::kCascadePorts.value().first,
                                                             facts::kCascadePorts.value().second, 10}, 3},
                                            sb::robot_one::kRobotOneMechanisms));
static_assert(ff::kArmMap.avoids(ControllerButton::Y) && ff::kAirMap.avoids(ControllerButton::Y));

// THE OTHER DIRECTION (the review of ROB1-A, RV8/RV9): a kind that declares NOTHING ("not on the robot yet", a
// passive bar) is refused against a list that DOES declare a flipper, of either form...
static_assert(!sb::flipperKindMatches(facts::FlipperKind::NotOnTheRobotYet, ff::kWithArm));
static_assert(!sb::flipperKindMatches(facts::FlipperKind::NotOnTheRobotYet, ff::kWithAir));
static_assert(!sb::flipperKindMatches(facts::FlipperKind::PassiveBar, ff::kWithArm));
static_assert(!sb::flipperKindMatches(facts::FlipperKind::PassiveBar, ff::kWithAir));
// ...and a motor arm is an ARM — a position mechanism in DEGREES: a Lift (inches) named "flipper" is not one.
namespace not_an_arm {
inline constexpr shulib::mechanism::Lift flipperLift = [] {
    shulib::mechanism::Lift l = sb::robot_one::cascade;
    l.name = "flipper";
    return l;
}();
inline constexpr sb::MechanismTable kWithLift =
    shulib::mechanism::mechanisms(shulib::mechanism::wire(flipperLift, shulib::mechanism::motors(11, 12)));
}  // namespace not_an_arm
static_assert(!sb::flipperKindMatches(facts::FlipperKind::MotorArm, not_an_arm::kWithLift));
static_assert(!sb::flipperKindMatches(facts::FlipperKind::Pneumatic, not_an_arm::kWithLift));
static_assert(!sb::flipperKindMatches(facts::FlipperKind::PassiveBar, not_an_arm::kWithLift));

TEST_CASE("robot one flipper (T13): each form fits the Drive screen's row beside the cascade") {
    // BUG CAUGHT: a flipper form whose row runs past the SMALL budget — the build would refuse it the day it
    // is pasted, on the robot's schedule instead of here.
    CHECK(sb::clawRowWorstLength(ff::kWithArm) <= shulib::calibration::kSmallCols);
    CHECK(sb::clawRowWorstLength(ff::kWithAir) <= shulib::calibration::kSmallCols);
    CHECK(sb::clawRowWorstLength(ff::kPassive) <= shulib::calibration::kSmallCols);
    CHECK(ff::kWithArm.count == 2);
    CHECK(ff::kWithAir.count == 2);
    CHECK(ff::kPassive.count == 1);
}

TEST_CASE("robot one flipper (T13): the MOTOR ARM form binds over the fake provider beside the cascade, and a blind flip reaches 'flip' on the host test plant") {
    // BUG CAUGHT: an arm form that compiles but cannot bind beside the cascade (a port or role clash at
    // boot), or whose "flip" it cannot reach on its own declaration.
    mech_rig::Rig both{ff::kWithArm};
    CHECK(both.svc.slotPtr(0) != nullptr);
    CHECK(both.svc.slotPtr(1) != nullptr);
    CHECK(both.svc.slotPtr(1)->built());
    // The arm alone on the host test plant: its stops at 0 and 120 degrees, resting stowed.
    const shulib::mechanism::MechanismSpec& arm = ff::kWithArm.specs[1];
    shulib::sim::ArmPlantConfig pc = pos_rig::armPlant(pos_rig::cartridgeRpm(arm.cartridge), arm.position.gearRatio,
                                                       0.0, 1.0, 0.3, 0.0, 45.0, 0.0, 120.0);
    pos_rig::ArmRig r{arm, pc};
    r.slot().moveToPreset(1, Time{2.0});  // "flip"
    r.runStep(1000);
    CHECK(r.slot().lastStep().outcome == MechanismOutcome::Succeeded);
    CHECK(std::abs(r.trueDeg() - 95.0) <= arm.position.tolerance + 0.5);
}

TEST_CASE("robot one flipper (T13): the PNEUMATIC form binds beside the cascade; a blind flip drives its line at once and succeeds after its stroke time") {
    // BUG CAUGHT: a pneumatic form whose flip never reaches the line, or that is counted done before its
    // stroke has had time to happen.
    mech_rig::Rig r{ff::kWithAir};
    REQUIRE(r.svc.slotPtr(1) != nullptr);
    REQUIRE(r.svc.slotPtr(1)->built());
    shulib::mechanism::PneumaticSlot& air = r.svc.pneumatic(1);
    CHECK(air.commandedPosition() == 0);  // stowed at the start
    air.set(1);                            // "flip"
    CHECK(air.commandedPosition() == 1);   // the line, at once
    CHECK(air.running());
    r.pace(20);                            // 0.2 s: not yet its 0.4 s stroke
    CHECK(air.running());
    r.pace(30);
    CHECK_FALSE(air.running());
    CHECK(air.lastStep().outcome != MechanismOutcome::Running);
}
