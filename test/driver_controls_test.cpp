// driver_controls_test.cpp — T4 Part 1a: the GENERIC half of a driver file
// (src/common/driver_controls.hpp), PROS-free — hand-built controls over the fake device provider.
//
// ═══ WHAT THIS FILE PINS ════════════════════════════════════════════════════════════
//   * each of the five compile-time predicates, broken ONE way at a time (a failing static_assert
//     in a driver file must name the real fault, so each predicate must fail on its own property
//     and pass on every other) — and robot two's real set passing all five;
//   * DriverBindings driven directly with sets the driver file's checks would refuse at compile
//     time: validBindings()' refusal is reachable here, a binding that cannot be built THROWS (why
//     every program binds inside its boot's try), and every refusal leaves NOTHING bound;
//   * driverHeading() — THE heading rule — with ONE of its three terms flipped at a time. It had no
//     test before T4 Part 1a: both copies lived in program files the host cannot compile;
//   * that the rule has ONE copy left in src/ (a source pin).
// robot two's driver file itself, through the PROS shim, is test/robot_two_driver_test.cpp's.
//
// NO HARDWARE CLAIM: fakes only.

#include "doctest.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>

#include "common/boot_source.hpp"
#include "common/driver_controls.hpp"
#include "robots/mechanism_table.hpp"
#include "shulib/core/check.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/hal/controller.hpp"
#include "shulib/hal/drivetrain_description.hpp"
#include "shulib/hal/fake/fake_battery.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_device_provider.hpp"
#include "shulib/hal/fake/fake_device_registry.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/localization/localizer.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/motion/motion_scheduler.hpp"
#include "shulib/robot/robot.hpp"
#include "shulib/units/literals.hpp"
#include "shulib/units/quantity.hpp"

using shulib::PreconditionError;
using shulib::bench::controlButtonsDistinct;
using shulib::bench::controlsAvoid;
using shulib::bench::controlsCoverTable;
using shulib::bench::controlsNameTable;
using shulib::bench::DriverBindings;
using shulib::bench::driverHeading;
using shulib::bench::HoldControl;
using shulib::bench::holdVoltsValid;
using shulib::bench::kRobotTwoMechanisms;
using shulib::bench::MechanismTable;
using shulib::bench::ToggleControl;
using shulib::hal::ControllerButton;
using shulib::localization::Localizer;
using namespace shulib::units::literals;

namespace {

using B = ControllerButton;
using Holds = std::span<const HoldControl>;
using Toggles = std::span<const ToggleControl>;

// Robot two's REAL set, spelled as the driver file spells it (the driver file's own copy is
// test/robot_two_driver_test.cpp's to read; this one is the baseline each break below edits ONE
// field of).
constexpr HoldControl kTwoHolds[] = {
    {"claw pivot 6", B::R1, B::R2, 12_volt},
    {"claw pivot 7", B::L1, B::L2, 12_volt},
};
constexpr ToggleControl kTwoToggles[] = {{"claw", B::A}};

// The real set passes all five, at COMPILE time (the form a driver file uses).
static_assert(controlButtonsDistinct(kTwoHolds, kTwoToggles));
static_assert(controlsAvoid(kTwoHolds, kTwoToggles, B::Y));
static_assert(holdVoltsValid(kTwoHolds));
static_assert(controlsNameTable(kTwoHolds, kTwoToggles, kRobotTwoMechanisms));
static_assert(controlsCoverTable(kTwoHolds, kTwoToggles, kRobotTwoMechanisms));

/// Every predicate at once, so a break can be shown to fail ITS predicate and pass the other four.
struct Verdicts {
    bool distinct;
    bool avoid;
    bool volts;
    bool names;
    bool covers;
};
Verdicts judge(Holds h, Toggles t, const MechanismTable& table = kRobotTwoMechanisms,
               ControllerButton rezero = B::Y) {
    return Verdicts{controlButtonsDistinct(h, t), controlsAvoid(h, t, rezero), holdVoltsValid(h),
                    controlsNameTable(h, t, table), controlsCoverTable(h, t, table)};
}
/// True when exactly the named predicate fails and the other four pass.
bool onlyFails(const Verdicts& v, const char* which) {
    const std::string_view w{which};
    return v.distinct == (w != "distinct") && v.avoid == (w != "avoid")
           && v.volts == (w != "volts") && v.names == (w != "names") && v.covers == (w != "covers");
}

// ── robot two over the FAKE provider ────────────────────────────────────────────────
/// Robot two's 2026-09-25 record (the fixture test/teleop_claws_test.cpp and
/// test/boot_source_test.cpp retype — each lives in its file's anonymous namespace).
shulib::hal::DrivetrainDescription robotTwoRecord() {
    shulib::hal::DrivetrainDescription d{};
    d.kind = shulib::hal::DrivetrainKind::Tank;
    d.cartridge = shulib::hal::MotorCartridge::Blue;
    d.imuPort = 2;
    const std::int8_t left[4] = {-16, 17, -18, -20};
    const std::int8_t right[4] = {11, -12, 13, 15};
    for (std::size_t i = 0; i < 4; ++i) {
        d.left.signedPorts[i] = left[i];
        d.right.signedPorts[i] = right[i];
    }
    d.left.count = 4;
    d.right.count = 4;
    d.left.inchesPerRadian = shulib::units::Length{1.25442};
    d.right.inchesPerRadian = shulib::units::Length{1.36703};
    d.trackWidth = shulib::units::Length{12.006};
    d.source = shulib::hal::DescriptionSource::RobotMeasured;
    std::snprintf(d.provenance, sizeof d.provenance, "%s", "driver controls test (the 09-25 record)");
    return d;
}

struct NoPace final : shulib::motion::ITickPacer {
    void pace() override {}
};

/// Robot two built from `table` over the fake provider: every drive port, both pivots and the IMU
/// in the census; the IMU ready (a fresh FakeImu is), nothing ticked yet.
struct FakeRobotTwo {
    explicit FakeRobotTwo(const MechanismTable& table = kRobotTwoMechanisms) {
        battery.setVoltage(shulib::units::Voltage{12.8});  // a fresh FakeBattery reads 0 V: a brownout
        shulib::bench::RecordRead record{};
        record.description = robotTwoRecord();
        record.decoded = true;
        shulib::hal::fake::FakeDeviceRegistry census;
        for (const int p : {6, 7, 11, 12, 13, 14, 15, 16, 17, 18, 20}) {
            census.setKind(p, shulib::hal::DeviceKind::Motor);
        }
        census.setKind(2, shulib::hal::DeviceKind::Imu);
        boot = shulib::bench::decideBootSourceFrom(record, census,
                                                   shulib::hal::DrivetrainDescription{}, "", table,
                                                   "robot two", [](const char*) {});
        REQUIRE_MESSAGE(boot.ok, boot.why);
        robot.emplace(boot.built.config, devices, boot.built.buildOptions());
    }
    shulib::hal::fake::FakeClock clock;
    shulib::hal::fake::FakeBattery battery;
    shulib::hal::fake::FakeTelemetrySink sink;
    NoPace pacer;
    shulib::hal::fake::FakeDeviceProvider devices{clock, battery, sink, pacer};
    shulib::bench::BootOutcome boot;
    std::optional<shulib::robot::Robot> robot;
};

}  // namespace

// ═══ The five predicates, one property each ════════════════════════════════════════════
TEST_CASE("driver controls: controlButtonsDistinct — a button in two holds, a pivot whose forward "
          "is its back, a toggle on a hold's button and two toggles on one button each fail it, "
          "and ONLY it") {
    // BUG CAUGHT: one button driving two mechanisms (or one pivot both ways) compiling, because
    // the check compared only holds with holds, or only different controls.
    CHECK(onlyFails(judge(kTwoHolds, kTwoToggles), "none"));  // the real set: nothing fails
    const HoldControl r1Twice[] = {{"claw pivot 6", B::R1, B::R2, 12_volt},
                                   {"claw pivot 7", B::R1, B::L2, 12_volt}};
    CHECK(onlyFails(judge(r1Twice, kTwoToggles), "distinct"));
    const HoldControl forwardIsBack[] = {{"claw pivot 6", B::R1, B::R1, 12_volt},
                                         {"claw pivot 7", B::L1, B::L2, 12_volt}};
    CHECK(onlyFails(judge(forwardIsBack, kTwoToggles), "distinct"));
    const ToggleControl onL2[] = {{"claw", B::L2}};
    CHECK(onlyFails(judge(kTwoHolds, onL2), "distinct"));
    // Two toggles on one button, over a table with two pneumatic entries.
    MechanismTable twoClaws = kRobotTwoMechanisms;
    twoClaws.pneumatics[1] = kRobotTwoMechanisms.pneumatics[0];
    twoClaws.pneumatics[1].name = "wing";
    twoClaws.pneumatics[1].adiLine = 2;
    twoClaws.pneumaticCount = 2;
    const ToggleControl bothOnA[] = {{"claw", B::A}, {"wing", B::A}};
    CHECK(onlyFails(judge(kTwoHolds, bothOnA, twoClaws), "distinct"));
    const ToggleControl apart[] = {{"claw", B::A}, {"wing", B::B}};
    CHECK(onlyFails(judge(kTwoHolds, apart, twoClaws), "none"));
    // Any byte, not only the twelve named buttons: two uses of one out-of-range byte collide.
    const auto odd = static_cast<ControllerButton>(40);
    const HoldControl oddTwice[] = {{"claw pivot 6", odd, B::R2, 12_volt},
                                    {"claw pivot 7", B::L1, odd, 12_volt}};
    CHECK_FALSE(controlButtonsDistinct(oddTwice, kTwoToggles));
    // Empty lists are a valid (if useless) set.
    CHECK(controlButtonsDistinct(Holds{}, Toggles{}));
}

TEST_CASE("driver controls: controlsAvoid — a hold's forward, a hold's back or a toggle on the "
          "re-zero button each fail it, and ONLY it") {
    // BUG CAUGHT: a claw on the field-centric re-zero button (every claw press would re-zero the
    // field frame), checked for one position of the button only.
    CHECK(controlsAvoid(kTwoHolds, kTwoToggles, B::Y));
    CHECK(onlyFails(judge(kTwoHolds, kTwoToggles, kRobotTwoMechanisms, B::R1), "avoid"));  // fwd
    CHECK(onlyFails(judge(kTwoHolds, kTwoToggles, kRobotTwoMechanisms, B::L2), "avoid"));  // back
    CHECK(onlyFails(judge(kTwoHolds, kTwoToggles, kRobotTwoMechanisms, B::A), "avoid"));   // toggle
    CHECK(onlyFails(judge(kTwoHolds, kTwoToggles, kRobotTwoMechanisms, B::X), "none"));
}

TEST_CASE("driver controls: holdVoltsValid — 0, 12.0001, NaN, -1 and +infinity each fail it, and "
          "ONLY it; 12 exactly and a tiny positive pass") {
    // BUG CAUGHT: a comparison written so NaN passes (`v <= 0 || v > 12` is false for NaN), a
    // closed lower bound (0 V is not a command), or an open upper one (12 V is the rail).
    const double bad[] = {0.0, 12.0001, std::numeric_limits<double>::quiet_NaN(), -1.0,
                          std::numeric_limits<double>::infinity()};
    for (const double v : bad) {
        CAPTURE(v);
        const HoldControl h[] = {{"claw pivot 6", B::R1, B::R2, shulib::units::Voltage{v}},
                                 {"claw pivot 7", B::L1, B::L2, 12_volt}};
        CHECK(onlyFails(judge(h, kTwoToggles), "volts"));
    }
    const double good[] = {12.0, 0.001};
    for (const double v : good) {
        CAPTURE(v);
        const HoldControl h[] = {{"claw pivot 6", B::R1, B::R2, 12_volt},
                                 {"claw pivot 7", B::L1, B::L2, shulib::units::Voltage{v}}};
        CHECK(onlyFails(judge(h, kTwoToggles), "none"));
    }
}

TEST_CASE("driver controls: controlsNameTable — a name the table lacks, a hold on a PNEUMATIC "
          "entry, a toggle on a MOTOR entry, two controls on one name and a null name each fail "
          "it (and a lost entry fails coverage too, as it must)") {
    // BUG CAUGHT: a typo in a mechanism's name compiling and leaving that pivot unbound; a hold on
    // the claw (a pneumatic) or a toggle on a pivot; one mechanism bound twice.
    const HoldControl typo[] = {{"claw pivot 6", B::R1, B::R2, 12_volt},
                                {"claw pivot 66", B::L1, B::L2, 12_volt}};
    const Verdicts t = judge(typo, kTwoToggles);
    CHECK_FALSE(t.names);
    CHECK_FALSE(t.covers);  // pivot 7 has no control now — a second, true, fault
    CHECK((t.distinct && t.avoid && t.volts));
    const HoldControl holdOnClaw[] = {{"claw pivot 6", B::R1, B::R2, 12_volt},
                                      {"claw pivot 7", B::L1, B::L2, 12_volt},
                                      {"claw", B::X, B::B, 12_volt}};
    CHECK(onlyFails(judge(holdOnClaw, kTwoToggles), "names"));  // the claw IS covered (by A)
    const ToggleControl toggleOnPivot[] = {{"claw", B::A}, {"claw pivot 6", B::X}};
    CHECK(onlyFails(judge(kTwoHolds, toggleOnPivot), "names"));
    const HoldControl sameNameTwice[] = {{"claw pivot 6", B::R1, B::R2, 12_volt},
                                         {"claw pivot 7", B::L1, B::L2, 12_volt},
                                         {"claw pivot 6", B::X, B::B, 12_volt}};
    CHECK(onlyFails(judge(sameNameTwice, kTwoToggles), "names"));
    const ToggleControl clawTwice[] = {{"claw", B::A}, {"claw", B::X}};
    CHECK(onlyFails(judge(kTwoHolds, clawTwice), "names"));
    // A hold and a toggle on ONE name, over a table that (wrongly) declares both kinds under it.
    MechanismTable bothKinds = kRobotTwoMechanisms;
    bothKinds.pneumatics[1] = kRobotTwoMechanisms.pneumatics[0];
    bothKinds.pneumatics[1].name = "claw pivot 6";
    bothKinds.pneumatics[1].adiLine = 2;
    bothKinds.pneumaticCount = 2;
    const ToggleControl crossKind[] = {{"claw", B::A}, {"claw pivot 6", B::X}};
    CHECK(onlyFails(judge(kTwoHolds, crossKind, bothKinds), "names"));
    const HoldControl nullName[] = {{"claw pivot 6", B::R1, B::R2, 12_volt},
                                    {"claw pivot 7", B::L1, B::L2, 12_volt},
                                    {nullptr, B::X, B::B, 12_volt}};
    CHECK(onlyFails(judge(nullName, kTwoToggles), "names"));
    // The KIND alone (mutation run, T4-PROGRESS: the cases above all name a mechanism some other
    // control also names, so the "twice" rule refused them even with the kind rule gone). Here the
    // wrong-kind name is named by nothing else: only the kind rule can refuse it. (Coverage fails
    // too — the mechanism has no control of ITS kind — which is true and not this rule's.)
    MechanismTable withWing = kRobotTwoMechanisms;
    withWing.pneumatics[1] = kRobotTwoMechanisms.pneumatics[0];
    withWing.pneumatics[1].name = "wing";
    withWing.pneumatics[1].adiLine = 2;
    withWing.pneumaticCount = 2;
    const HoldControl holdOnWing[] = {{"claw pivot 6", B::R1, B::R2, 12_volt},
                                      {"claw pivot 7", B::L1, B::L2, 12_volt},
                                      {"wing", B::X, B::B, 12_volt}};
    CHECK_FALSE(controlsNameTable(holdOnWing, kTwoToggles, withWing));
    MechanismTable withIntake = kRobotTwoMechanisms;
    withIntake.motors[2] = kRobotTwoMechanisms.motors[0];
    withIntake.motors[2].name = "intake";
    withIntake.motors[2].signedPort = 8;
    withIntake.motorCount = 3;
    const ToggleControl toggleOnIntake[] = {{"claw", B::A}, {"intake", B::X}};
    CHECK_FALSE(controlsNameTable(kTwoHolds, toggleOnIntake, withIntake));
}

TEST_CASE("driver controls: controlsCoverTable — a table entry with no control fails it, and "
          "ONLY it: a pivot's line removed, the claw's line removed, an extra entry nobody names") {
    // BUG CAUGHT: a mechanism declared on the robot that no button works (the controls are for a
    // different robot) compiling cleanly.
    const HoldControl sixOnly[] = {{"claw pivot 6", B::R1, B::R2, 12_volt}};
    CHECK(onlyFails(judge(sixOnly, kTwoToggles), "covers"));
    CHECK(onlyFails(judge(kTwoHolds, Toggles{}), "covers"));
    MechanismTable withIntake = kRobotTwoMechanisms;
    withIntake.motors[2] = kRobotTwoMechanisms.motors[0];
    withIntake.motors[2].name = "intake";
    withIntake.motors[2].signedPort = 8;
    withIntake.motorCount = 3;
    CHECK(onlyFails(judge(kTwoHolds, kTwoToggles, withIntake), "covers"));
    // …and an empty table with no controls is covered (a robot with no mechanisms).
    CHECK(controlsCoverTable(Holds{}, Toggles{}, shulib::bench::kNoMechanisms));
    CHECK(controlsNameTable(Holds{}, Toggles{}, shulib::bench::kNoMechanisms));
}

// ═══ DriverBindings, driven directly ════════════════════════════════════════════════════
TEST_CASE("driver controls: DriverBindings refuses at RUN time what the driver file's checks "
          "refuse at compile time — a toggle or a hold on the re-zero button with validBindings()' "
          "own sentence, one mechanism bound twice — and every refusal leaves nothing bound") {
    // BUG CAUGHT: validBindings() dropped from bind() — the loop would then refuse the same set by
    // THROWING inside driver control (its precondition), the one place a program cannot afford it.
    FakeRobotTwo r;
    DriverBindings b;
    char why[192] = "";
    REQUIRE(b.bind(*r.robot, kTwoHolds, kTwoToggles, B::Y, why, sizeof why));
    CHECK(std::string_view{why}.empty());
    CHECK(b.holds().size() == 2U);
    CHECK(b.toggles().size() == 1U);
    CHECK_FALSE(b.bind(*r.robot, kTwoHolds, kTwoToggles, B::A, why, sizeof why));
    CHECK(std::string_view{why}
          == "button A toggles mechanism 'claw' and is also the driver-control re-zero button -- "
             "give the re-zero another button");
    CHECK(b.holds().empty());
    CHECK(b.toggles().empty());
    CHECK_FALSE(b.bind(*r.robot, kTwoHolds, kTwoToggles, B::R2, why, sizeof why));
    CHECK(std::string_view{why}
          == "button R2 drives mechanism 'claw pivot 6' and is also the driver-control re-zero "
             "button -- give the re-zero another button");
    CHECK(b.holds().empty());
    // One mechanism named by two holds (distinct buttons): the loop's own "bound twice" refusal.
    const HoldControl sixTwice[] = {{"claw pivot 6", B::R1, B::R2, 12_volt},
                                    {"claw pivot 7", B::L1, B::L2, 12_volt},
                                    {"claw pivot 6", B::X, B::B, 12_volt}};
    CHECK_FALSE(b.bind(*r.robot, sixTwice, kTwoToggles, B::Y, why, sizeof why));
    CHECK(std::string_view{why}.starts_with("mechanism 'claw pivot 6' is bound twice"));
    CHECK(b.holds().empty());
    CHECK(b.toggles().empty());
    // A clean bind still works after every refusal.
    CHECK(b.bind(*r.robot, kTwoHolds, kTwoToggles, B::Y, why, sizeof why));
    CHECK(b.holds().size() == 2U);
}

TEST_CASE("driver controls: a hold bind() cannot build (13 V) THROWS — why every program binds "
          "inside its boot's try — and unbind() then leaves nothing") {
    // BUG CAUGHT (T3 test 1m's intent): a volts the compile-time check would refuse reaching bind()
    // and escaping initialize(); a catch that tears the robot down with a half-built set still
    // holding its mechanisms' addresses.
    FakeRobotTwo r;
    DriverBindings b;
    const HoldControl hot[] = {{"claw pivot 6", B::R1, B::R2, 12_volt},
                               {"claw pivot 7", B::L1, B::L2, 13_volt}};
    char why[192] = "";
    CHECK_THROWS_AS((void)b.bind(*r.robot, hot, kTwoToggles, B::Y, why, sizeof why),
                    PreconditionError);
    CHECK(b.holds().size() == 1U);  // pivot 6 was built before pivot 7 threw: the catch cleans up
    b.unbind();
    CHECK(b.holds().empty());
    CHECK(b.toggles().empty());
    CHECK(b.holdFor("claw pivot 6") == nullptr);
    CHECK(b.toggleFor("claw") == nullptr);
    CHECK_FALSE(b.anyReleasedLeftoverClaim());
}

TEST_CASE("driver controls: DriverBindings refuses, by name — more controls than a robot can have, "
          "a null name, a name the robot does not declare (a hold's AND a toggle's), the wrong "
          "kind (a hold's AND a toggle's), a declared mechanism no control names — and each "
          "refusal, made on a binder BOUND to robot two a moment before, leaves NOTHING bound") {
    // BUG CAUGHT: the storage overrun a ninth control would be; a null name dereferenced; a toggle
    // (or a hold) naming a mechanism the robot does not declare quietly SKIPPED, its button then
    // doing nothing (the toggle's refusal had no test until the Part 1a review: mutation RV-M7
    // survived); a control quietly skipped when it names the wrong kind; and a refusal that
    // returns false while the PREVIOUS bindings stay live — unbind() moved below the refusals
    // (RV-M1), which survived while every refusal here ran on a binder that was never bound.
    FakeRobotTwo r;
    DriverBindings b;
    char why[192] = "";
    // Bind robot two cleanly, then make the refusal: it must DROP what was bound.
    const auto refusedAfterABind = [&](Holds holds, Toggles toggles) {
        REQUIRE(b.bind(*r.robot, kTwoHolds, kTwoToggles, B::Y, why, sizeof why));
        REQUIRE(b.holds().size() == 2U);
        REQUIRE(b.toggles().size() == 1U);
        const bool bound = b.bind(*r.robot, holds, toggles, B::Y, why, sizeof why);
        CHECK(b.holds().empty());
        CHECK(b.toggles().empty());
        CHECK(b.holdFor("claw pivot 6") == nullptr);
        CHECK(b.toggleFor("claw") == nullptr);
        return !bound;
    };
    HoldControl nine[9];
    for (HoldControl& h : nine) h = kTwoHolds[0];
    CHECK(refusedAfterABind(nine, kTwoToggles));
    CHECK(std::string_view{why}
          == "9 hold and 1 toggle controls: a robot declares at most 8 mechanisms, so there can be "
             "no more of either");
    ToggleControl nineToggles[9];
    for (ToggleControl& t : nineToggles) t = kTwoToggles[0];
    CHECK(refusedAfterABind(kTwoHolds, nineToggles));
    CHECK(std::string_view{why}
          == "2 hold and 9 toggle controls: a robot declares at most 8 mechanisms, so there can be "
             "no more of either");
    const HoldControl nullName[] = {{"claw pivot 6", B::R1, B::R2, 12_volt},
                                    {nullptr, B::L1, B::L2, 12_volt}};
    CHECK(refusedAfterABind(nullName, kTwoToggles));
    CHECK(std::string_view{why} == "hold control 1 names no mechanism (a null name)");
    const ToggleControl nullToggle[] = {{nullptr, B::A}};
    CHECK(refusedAfterABind(kTwoHolds, nullToggle));
    CHECK(std::string_view{why} == "toggle control 0 names no mechanism (a null name)");
    // A name robot two does not declare — a hold's (a typo), then a toggle's (a second valve the
    // robot does not have: without the refusal, B would silently do nothing).
    const HoldControl typo[] = {{"claw pivot 6", B::R1, B::R2, 12_volt},
                                {"claw pivot 66", B::L1, B::L2, 12_volt}};
    CHECK(refusedAfterABind(typo, kTwoToggles));
    CHECK(std::string_view{why}
          == "a control names 'claw pivot 66', which the robot does not declare");
    const ToggleControl wing[] = {{"claw", B::A}, {"wing", B::B}};
    CHECK(refusedAfterABind(kTwoHolds, wing));
    CHECK(std::string_view{why} == "a control names 'wing', which the robot does not declare");
    // The wrong kind — a hold on the claw (pneumatic), a toggle on a pivot (motor).
    const HoldControl onClaw[] = {{"claw pivot 6", B::R1, B::R2, 12_volt},
                                  {"claw pivot 7", B::L1, B::L2, 12_volt},
                                  {"claw", B::X, B::B, 12_volt}};
    CHECK(refusedAfterABind(onClaw, kTwoToggles));
    CHECK(std::string_view{why} == "'claw' is not a motor mechanism: a hold control needs one");
    const ToggleControl onPivot[] = {{"claw", B::A}, {"claw pivot 6", B::X}};
    CHECK(refusedAfterABind(kTwoHolds, onPivot));
    CHECK(std::string_view{why}
          == "'claw pivot 6' is not a pneumatic mechanism: a toggle control needs one");
    // A declared mechanism with no control — pivot 7's line gone; the claw's list gone.
    const HoldControl sixOnly[] = {{"claw pivot 6", B::R1, B::R2, 12_volt}};
    CHECK(refusedAfterABind(sixOnly, kTwoToggles));
    CHECK(std::string_view{why} == "the robot declares 'claw pivot 7', and no control names it");
    CHECK(refusedAfterABind(kTwoHolds, Toggles{}));
    CHECK(std::string_view{why} == "the robot declares 'claw', and no control names it");
    // A null `why` is allowed: the refusal still refuses, and still drops what was bound.
    REQUIRE(b.bind(*r.robot, kTwoHolds, kTwoToggles, B::Y, why, sizeof why));
    CHECK_FALSE(b.bind(*r.robot, kTwoHolds, onPivot, B::Y, nullptr, 0));
    CHECK(b.holds().empty());
    CHECK(b.toggles().empty());
    // Nothing a refusal did survives it: a clean bind works.
    CHECK(b.bind(*r.robot, kTwoHolds, kTwoToggles, B::Y, why, sizeof why));
    CHECK(b.holds().size() == 2U);
    CHECK(b.toggles().size() == 1U);
}

TEST_CASE("driver controls: DriverBindings' capacity is EIGHT of each kind — eight holds or eight "
          "toggles are not refused as too many (a later rule refuses these, by its own sentence), "
          "nine are") {
    // BUG CAUGHT: the capacity check written `>=` (mutation RV-M3, which survived until the Part 1a
    // review): a robot may declare 8 mechanisms, and 8 controls would then be refused with a
    // sentence saying a robot cannot have 8. (Eight holds can never ALL bind — they need 16
    // buttons and a controller has 12 — so each set below is refused later, and the pin is WHICH
    // sentence refuses it.)
    FakeRobotTwo r;
    DriverBindings b;
    char why[192] = "";
    HoldControl eight[8];
    for (HoldControl& h : eight) h = kTwoHolds[0];  // all name pivot 6: pivot 7 then has none
    CHECK_FALSE(b.bind(*r.robot, eight, kTwoToggles, B::Y, why, sizeof why));
    CHECK(std::string_view{why} == "the robot declares 'claw pivot 7', and no control names it");
    ToggleControl eightToggles[8];
    for (ToggleControl& t : eightToggles) t = kTwoToggles[0];  // all name the claw, on A
    CHECK_FALSE(b.bind(*r.robot, kTwoHolds, eightToggles, B::Y, why, sizeof why));
    // Built, then refused by the loop's own check (validBindings: a button used twice).
    CHECK(std::string_view{why}
          == "button A drives both mechanism 'claw' and mechanism 'claw' -- one button, one job");
    CHECK(b.holds().empty());
    CHECK(b.toggles().empty());
    HoldControl nine[9];
    for (HoldControl& h : nine) h = kTwoHolds[0];
    CHECK_FALSE(b.bind(*r.robot, nine, kTwoToggles, B::Y, why, sizeof why));
    CHECK(std::string_view{why}.starts_with("9 hold and 1 toggle controls"));
}

// ═══ THE heading rule, one term at a time ═══════════════════════════════════════════════
TEST_CASE("driver controls T7: driverHeading() trusts the heading only when the estimate is live, "
          "the IMU is ready AND no IMU_LOST was raised — each term flipped alone makes it "
          "untrusted; trusted, it hands over the pose's heading") {
    // BUG CAUGHT: any of the three terms dropped (field-centric would then turn the sticks by a
    // heading that is still booting, or that the IMU itself disowns, or that was lost this run).
    // Each case below leaves the OTHER two terms true, so each term is the only reason.
    const auto tick = [](FakeRobotTwo& r) {
        r.clock.advance(10_ms);
        r.robot->localizer().update();
    };
    SUBCASE("(a) the localizer Uninitialized, the IMU READY, no fault => untrusted") {
        FakeRobotTwo r;
        REQUIRE(r.devices.fakeImu(2).isReady());
        REQUIRE(r.robot->faults().raiseCount(shulib::diag::FaultCode::ImuLost) == 0);
        REQUIRE(r.robot->localizer().qualityClass() == Localizer::Quality::Uninitialized);
        CHECK_FALSE(driverHeading(*r.robot).trusted);
        tick(r);  // ONE update never makes the estimate live (localizer.hpp's contract)
        REQUIRE(r.robot->localizer().qualityClass() == Localizer::Quality::Uninitialized);
        CHECK_FALSE(driverHeading(*r.robot).trusted);
    }
    SUBCASE("(b) the localizer live, then the IMU NOT ready, no fault => untrusted") {
        FakeRobotTwo r;
        tick(r);
        tick(r);
        REQUIRE(r.robot->localizer().qualityClass() != Localizer::Quality::Uninitialized);
        REQUIRE(driverHeading(*r.robot).trusted);
        r.devices.fakeImu(2).setReady(false);
        REQUIRE(r.robot->localizer().qualityClass() != Localizer::Quality::Uninitialized);
        CHECK_FALSE(driverHeading(*r.robot).trusted);
        tick(r);  // the localizer now calls it Degraded — still live, still the IMU's term alone
        REQUIRE(r.robot->localizer().qualityClass() != Localizer::Quality::Uninitialized);
        CHECK_FALSE(driverHeading(*r.robot).trusted);
    }
    SUBCASE("(c) live, ready, ONE IMU_LOST raised => untrusted") {
        FakeRobotTwo r;
        tick(r);
        tick(r);
        REQUIRE(driverHeading(*r.robot).trusted);
        r.robot->faults().raise(shulib::diag::FaultCode::ImuLost, "test", "one IMU_LOST");
        REQUIRE(r.devices.fakeImu(2).isReady());
        REQUIRE(r.robot->localizer().qualityClass() != Localizer::Quality::Uninitialized);
        CHECK_FALSE(driverHeading(*r.robot).trusted);
        // Another fault is not the IMU's: trust is not taken away by it.
        FakeRobotTwo other;
        tick(other);
        tick(other);
        other.robot->faults().raise(shulib::diag::FaultCode::Brownout, "test", "not the IMU");
        CHECK(driverHeading(*other.robot).trusted);
    }
    SUBCASE("(d) live, ready, no fault => trusted, and the radians are the pose's heading") {
        FakeRobotTwo r;
        r.devices.fakeImu(2).setHeading(shulib::math::Angle::radians(0.7));
        tick(r);
        tick(r);
        r.devices.fakeImu(2).setHeading(shulib::math::Angle::radians(1.1));
        tick(r);
        const shulib::teleop::HeadingSample h = driverHeading(*r.robot);
        CHECK(h.trusted);
        const double pose = r.robot->localizer().pose().heading().radians();
        CHECK(h.radians == pose);
        CHECK(std::abs(pose) > 0.1);  // a heading that moved, so "equals" is not 0 == 0
    }
}

// ═══ ONE heading rule in src/ (a source pin) ═════════════════════════════════════════════
TEST_CASE("driver controls: the heading rule has ONE copy in src/ — src/common/driver_controls.hpp "
          "— and \"shulib Teleop\", the X-drive and robot two's driver file each call it") {
    // BUG CAUGHT: a second copy of the rule creeping back into a program file (the two programs
    // carried one each until T4 Part 1a; neither program file compiles on the host, so a copy
    // there would drift untested).
    namespace fs = std::filesystem;
    const fs::path root = fs::path{SHULIB_SOURCE_DIR} / "src";
    REQUIRE(fs::is_directory(root));
    const auto count = [](const std::string& hay, const std::string& needle) {
        std::size_t n = 0;
        for (std::size_t at = hay.find(needle); at != std::string::npos;
             at = hay.find(needle, at + needle.size())) {
            ++n;
        }
        return n;
    };
    std::size_t ruleCopies = 0;
    std::size_t oldName = 0;
    std::string ruleFile;
    for (const auto& e : fs::recursive_directory_iterator(root)) {
        if (!e.is_regular_file()) continue;
        std::ifstream in{e.path()};
        std::stringstream text;
        text << in.rdbuf();
        const std::string s = text.str();
        const std::size_t here = count(s, "raiseCount(diag::FaultCode::ImuLost)");
        if (here > 0) ruleFile = fs::relative(e.path(), root).generic_string();
        ruleCopies += here;
        oldName += count(s, "headingSample(");
    }
    CHECK(ruleCopies == 1U);
    CHECK(ruleFile == "common/driver_controls.hpp");
    CHECK(oldName == 0U);
    const auto read = [&root](const char* rel) {
        std::ifstream in{root / rel};
        REQUIRE(in.good());
        std::stringstream text;
        text << in.rdbuf();
        return text.str();
    };
    CHECK(count(read("programs/teleop_program.cpp"), "[&r] { return driverHeading(r); }") == 1U);
    CHECK(count(read("programs/xdrive_program.cpp"), "[&r] { return driverHeading(r); }") == 1U);
    CHECK(count(read("team/robot_two_driver.hpp"), "[&robot] { return driverHeading(robot); }")
          == 1U);
}
