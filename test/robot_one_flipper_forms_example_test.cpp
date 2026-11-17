// ROBOT ONE'S FLIPPER — ALL THREE FORMS (chunk ROB1-A, 2026-11-11; the ROB1 brief's D6, test T13). Say "host
// test plant", never "simulator". ROB1-A wrote the three forms ready while the flipper's KIND was owed (AF1-S's
// D7). On 2026-11-15 the builder named the FRONT TOGGLE's pistons (B: two forward pistons on one solenoid) and
// the coordinator made them the autons' flipper, a PNEUMATIC — so ROB1-C2 declared robot one's real flipper in
// the pneumatic form below (its numbers from the facts ledger: src/robots/mechanism_table.hpp), and this file
// keeps all three forms as the tested examples
// guide chapter 21 shows (a robot built later may have another kind). The forms' own numbers here (a port, a
// gearing, angles, a stroke time) are THIS TEST'S, invented: none is robot one's. Nothing here has run on a
// robot.
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
//   * ROBOT ONE'S REAL FLIPPER (ROB1-C2): the front toggle IS the pneumatic form — a Pneumatic named "flipper",
//     two positions on ONE line — its typed kind agrees with robot one's list, and AF1S-A1's Toggle step takes
//     its two positions as the push and the retract (competition/toggle_visit.hpp's flipper pair check).

#include "doctest.h"

#include <cstddef>
#include <string_view>

#include "common/driver_panel.hpp"
#include "mechanism_position_rig.hpp"
#include "mechanism_service_rig.hpp"
#include "robots/mechanism_table.hpp"
#include "shulib/competition/toggle_visit.hpp"
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

// The cascade's map in every form (robot one's real one since ROB1-C2: R1 / R2 while held, homed at the
// first driver start); the flipper's control is the only line that differs.
// (1) A MOTOR ARM on port 11 (a port robot one's list leaves free): stowed at 0 degrees, flipped at 95,
// resting on its stop when stowed.
inline constexpr Arm flipperArm{.name = "flipper", .motors = {Forward}, .cartridge = Green, .gearing = gears(12, 36),
                                .safe = Hold, .stall = {.amps = 1.0, .belowMotorRpm = 5, .lasting = 0.3_s},
                                .currentCap = {.amps = 1.5}, .range = {0, 120}, .gravity = noGravity,
                                .home = restsAtStop(0), .presets = {{"stow", 0}, {"flip", 95}}};
inline constexpr MechanismTable kWithArm = mechanisms(
    wire(cascade, motors(facts::kCascadePorts.value().first, facts::kCascadePorts.value().second)),
    wire(flipperArm, motors(11)));
inline constexpr DriverControls kArmMap = controls(kWithArm,
    whileHeld(cascade, Button::R1, Button::R2, 12_volt), homeAtDriverStart(cascade, 3_s),
    whileHeld(flipperArm, Button::Up, Button::Down, 8_volt));  // the flipper: Up out / Down back while held

// (2) A PNEUMATIC on 3-wire D: stowed at the start and when made safe; its measured stroke time. (Robot one's
// real flipper — the front toggle — is this form: src/robots/mechanism_table.hpp.)
inline constexpr Pneumatic flipperAir{.name = "flipper", .positions = {{"stow", {Low}}, {"flip", {High}}},
                                      .start = "stow", .safe = "stow", .moveTime = 0.4_s};
inline constexpr MechanismTable kWithAir = mechanisms(
    wire(cascade, motors(facts::kCascadePorts.value().first, facts::kCascadePorts.value().second)),
    wire(flipperAir, lines('D')));
inline constexpr DriverControls kAirMap = controls(kWithAir,
    whileHeld(cascade, Button::R1, Button::R2, 12_volt), homeAtDriverStart(cascade, 3_s),
    toggle(flipperAir, Button::B));  // the flipper: each B press flips or stows it

// (3) A PASSIVE BAR: nothing to declare — a list with no flipper is the form.
inline constexpr MechanismTable kPassive =
    mechanisms(wire(cascade, motors(facts::kCascadePorts.value().first, facts::kCascadePorts.value().second)));
}  // namespace flipper_forms

namespace ff = flipper_forms;

// Each form agrees with its typed kind, and its motors are all declared (the two fact checks, at build).
static_assert(sb::flipperKindMatches(facts::FlipperKind::MotorArm, ff::kWithArm));
static_assert(sb::flipperKindMatches(facts::FlipperKind::Pneumatic, ff::kWithAir));
static_assert(sb::flipperKindMatches(facts::FlipperKind::PassiveBar, ff::kPassive));
static_assert(!sb::flipperKindMatches(facts::FlipperKind::Pneumatic, ff::kWithArm));
static_assert(!sb::flipperKindMatches(facts::FlipperKind::MotorArm, ff::kWithAir));
static_assert(sb::everyListedMotorDeclared(facts::PortList{{facts::kCascadePorts.value().first,
                                                            facts::kCascadePorts.value().second, 11}, 3},
                                           ff::kWithArm));
// The arm's motor on port 11 is NOT declared by robot one's list as it stands: listing it on the Devices
// screen before its block is written stops every robot-one build (landmine 4).
static_assert(!sb::everyListedMotorDeclared(facts::PortList{{facts::kCascadePorts.value().first,
                                                             facts::kCascadePorts.value().second, 11}, 3},
                                            sb::robot_one::kRobotOneMechanisms));

// ROBOT ONE'S REAL FLIPPER (ROB1-C2): the front toggle, the PNEUMATIC form — its typed kind agrees with the
// list, the other kinds do not, and it is a two-position Pneumatic on ONE line, exactly the form above.
static_assert(sb::flipperKindMatches(facts::FlipperKind::Pneumatic, sb::robot_one::kRobotOneMechanisms));
static_assert(facts::kFlipperKind.value() == facts::FlipperKind::Pneumatic);
static_assert(!sb::flipperKindMatches(facts::FlipperKind::MotorArm, sb::robot_one::kRobotOneMechanisms));
static_assert(!sb::flipperKindMatches(facts::FlipperKind::PassiveBar, sb::robot_one::kRobotOneMechanisms));
static_assert(!sb::flipperKindMatches(facts::FlipperKind::NotOnTheRobotYet, sb::robot_one::kRobotOneMechanisms));
// AF1S-A1's Toggle step takes the front toggle: its push ("open") and its retract ("closed") are two positions
// of ONE pneumatic (competition/toggle_visit.hpp's pair check, the step's own refusal) — the tested pneumatic
// form there is test/toggle_visit_rig.hpp's flipPiston.
static_assert(shulib::competition::detail::flipperPairProblem(sb::robot_one::flipper["open"],
                                                              sb::robot_one::flipper["closed"]) ==
              shulib::competition::ToggleVisitProblem::None);
static_assert(shulib::competition::detail::flipperPairProblem(sb::robot_one::flipper["open"],
                                                              sb::robot_one::flipper["open"]) !=
              shulib::competition::ToggleVisitProblem::None);  // the check is not blind: the same goal twice
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

TEST_CASE("robot one flipper (ROB1-C2): robot one's REAL flipper is the front toggle -- a Pneumatic named 'flipper' on ONE line (the builder's B), two positions, starting and made safe in its UNPOWERED one") {
    // BUG CAUGHT: the front toggle declared on the wrong letter, with a second line it does not have (two
    // pistons on ONE solenoid), or starting in its powered position (it would move at power-up).
    const sb::MechanismTable& one = sb::robot_one::kRobotOneMechanisms;
    int found = -1;
    for (std::size_t i = 0; i < one.count; ++i) {
        if (std::string_view{one.specs[i].name} == "flipper") {
            found = static_cast<int>(i);
        }
    }
    REQUIRE(found >= 0);
    const auto i = static_cast<std::size_t>(found);
    const shulib::mechanism::MechanismSpec& s = one.specs[i];
    CHECK(s.kind == shulib::mechanism::Kind::Pneumatic);
    CHECK(s.lineCount == 1);
    CHECK(s.pneumatic.positionCount == 2);
    CHECK(static_cast<int>(one.wiring[i].lines[0]) == facts::kFlipperLine.value() - 'A' + 1);
    CHECK(facts::kFlipperLine.value() == 'B');
    // its start and safe: the position whose level is LOW (the solenoid unpowered)
    CHECK(s.pneumatic.positions[s.pneumatic.start].levels[0] == shulib::mechanism::words::Low);
    CHECK(s.pneumatic.positions[s.pneumatic.safe].levels[0] == shulib::mechanism::words::Low);
    CHECK(s.pneumatic.actuationTime == facts::kFlipperStrokeTime.value().value());
}

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
