// driver_panel_claws_test.cpp — robot two's CLAWS, end to end through the REAL src/ table, robot
// two's REAL driver file, hal::pros::ProsDeviceProvider and the PROS shim (T3 Part 2's test 1,
// written for "shulib Teleop" as test/teleop_claws_test.cpp; re-pointed at T4 Part 3, when Teleop
// retired, at the driver file every driver-control program binds through; renamed in Part 3's late
// step, when the words it pins moved into the shared driver panel).
//
// ═══ WHAT THIS FILE PINS ════════════════════════════════════════════════════════════
// src/common/driver_panel.hpp holds every sentence a driver-control program says about the claws
// (moved there from src/programs/teleop_claws.hpp at T4 Part 3, unchanged; the programs themselves
// cannot be compiled on the host: no screen shim). So the cases below build the programs' own
// composition — the boot decision over robot two's 2026-09-25 record and the shim's registry, with
// the REAL kRobotTwoMechanisms; robot::Robot over the REAL provider; the driver file's binder
// (DriverBindings, src/common/driver_controls.hpp) with the driver file's controls and its re-zero
// button (Y); the library loop with the bindings — and read the result at the bottom of the stack:
// each port's millivolts and brake() calls in the shim, and every level 3-wire port A was ever
// driven to. Plus every sentence the worksheet quotes (Station J), the merged fault row, and the
// boot lines' buttons read from the CONTROLS (T4 Part 3: the buttons left the table). Until T4
// Part 3 the fixture bound "shulib Teleop"'s TeleopClaws from the table's buttons; every word pin
// below is the one that form printed, but for the BUILT line's end, which named "shulib Teleop"
// (the T4 progress log, Part 3, maps each change). The chooser, the controller's lines and the
// PROTECTION row — the shared panel's other half — are test/driver_panel_test.cpp's.
//
// NO HARDWARE CLAIM. The shim models our BELIEF about PROS (its own header says so): nothing here
// shows a claw moving, a pivot holding a load (HA-156), or which way either turns (HA-153), and the
// claw's polarity is kClawOpenCommand's guess (HA-159). Station J is where those are measured.

#include "doctest.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "common/driver_controls.hpp"
#include "common/driver_panel.hpp"
#include "pros/shim_control.hpp"
#include "shulib/config/robot_config.hpp"
#include "shulib/core/check.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/hal/controller.hpp"
#include "shulib/hal/drivetrain_description.hpp"
#include "shulib/hal/fake/fake_battery.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_device_provider.hpp"
#include "shulib/hal/fake/fake_device_registry.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/hal/mechanism.hpp"
#include "shulib/hal/pros/controller.hpp"
#include "shulib/hal/pros/device_provider.hpp"
#include "shulib/hal/pros/device_registry.hpp"
#include "shulib/robot/robot.hpp"
#include "shulib/teleop/driver_control.hpp"
#include "shulib/teleop/mechanism_binding.hpp"
#include "shulib/teleop/teleop_loop.hpp"
#include "shulib/units/literals.hpp"
#include "shulib/units/quantity.hpp"
#include "team/robot_two_driver.hpp"

using shulib::bench::anyClawAbsent;
using shulib::bench::BootOutcome;
using shulib::bench::clawRowText;
using shulib::bench::decideBootSourceFrom;
using shulib::bench::DriverBindings;
using shulib::bench::faultRowText;
using shulib::bench::HoldControl;
using shulib::bench::kRobotTwoMechanisms;
using shulib::bench::MechanismTable;
using shulib::bench::motorClawBootLine;
using shulib::bench::pneumaticClawBootLine;
using shulib::bench::RecordRead;
using shulib::bench::RobotTwoDriver;
using shulib::bench::ToggleControl;
using shulib::diag::FaultCode;
using shulib::hal::ControllerButton;
using shulib::hal::DescriptionSource;
using shulib::hal::DrivetrainDescription;
using shulib::hal::DrivetrainKind;
using shulib::hal::MotorCartridge;
using shulib::hal::pros::ControllerId;
using shulib::hal::pros::ProsController;
using shulib::hal::pros::ProsDeviceProvider;
using shulib::hal::pros::ProsDeviceRegistry;
using shulib::teleop::DriverLoopConfig;
using shulib::teleop::DriverProfile;
using shulib::teleop::DriveShaper;
using shulib::teleop::DriveShaperState;
using shulib::teleop::HeadingSample;
using shulib::units::Length;
using shulib::units::Time;
namespace cfgns = shulib::config;

namespace {

// ── robot two, as the program boots it ───────────────────────────────────────────────
/// Robot two's calibration record as the 2026-09-25 capture printed it ([SES] ports MEASURED
/// L -16 +17 -18 -20 R +11 -12 +13 +15 IMU2 ipr 1.2544/1.3670 TW12.01 — boot_source_test.cpp's
/// fixture, retyped: it lives in that file's anonymous namespace).
DrivetrainDescription robotTwoRecord() {
    DrivetrainDescription d{};
    d.kind = DrivetrainKind::Tank;
    d.cartridge = MotorCartridge::Blue;
    d.imuPort = 2;
    const std::int8_t left[4] = {-16, 17, -18, -20};
    const std::int8_t right[4] = {11, -12, 13, 15};
    for (std::size_t i = 0; i < 4; ++i) {
        d.left.signedPorts[i] = left[i];
        d.right.signedPorts[i] = right[i];
    }
    d.left.count = 4;
    d.right.count = 4;
    d.left.inchesPerRadian = Length{1.25442};
    d.right.inchesPerRadian = Length{1.36703};
    d.trackWidth = Length{12.006};
    d.source = DescriptionSource::RobotMeasured;
    std::snprintf(d.provenance, sizeof d.provenance, "%s", "teleop claws test (the 09-25 record)");
    return d;
}

constexpr int kDrivePorts[8] = {16, 17, 18, 20, 11, 12, 13, 15};

/// The shim's controller button index for a library button (misc.hpp's enum: L1 = 6 … A = 17, in
/// ControllerButton's own order).
std::size_t shimButton(ControllerButton b) { return 6U + static_cast<std::size_t>(b); }

pros::shim::ControllerState& pad() { return pros::shim::controllerState(pros::E_CONTROLLER_MASTER); }

void press(ControllerButton b, bool down) { pad().digital[shimButton(b)] = down; }

/// The 3-wire line A's write history — every level it was ever driven to, construction included.
const std::vector<std::int32_t>& lineA() { return pros::shim::adiLine(0, 1).writes; }

/// What one tick left at the bottom of the stack, read in perTick (after drive() and the bindings).
struct Snap {
    std::array<std::int32_t, 22> mv{};
    std::array<int, 22> brakes{};
    std::array<int, 22> moves{};
    std::array<bool, 22> stopped{};
    std::size_t lineWrites = 0;
    std::string clawRow;
};

/// The driver file's re-zero button (Y): the loop's, and what every claw button must avoid.
constexpr ControllerButton kRezero = RobotTwoDriver::kLoop.rezeroButton;

/// A driver-control program's composition for robot two, on the host: the boot decision (the REAL
/// table), the Robot over the REAL provider, the claws bound by the driver file's binder with the
/// driver file's controls and its re-zero button (Y), the program's controller over the shim.
/// `plugged` says which claw pivots carry a motor. `table` is robot two's REAL table unless a case
/// edits one entry of a copy (test 1n flips a pivot's sign, as Station J's fix does); `holds` and
/// `toggles` are the driver file's controls unless a case renames one (test 1n's 23-character
/// name). All three are borrowed and must outlive the fixture. (Until T4 Part 3 this was "shulib
/// Teleop"'s composition, binding its TeleopClaws from the table's own buttons.)
struct RobotTwoClaws {
    explicit RobotTwoClaws(bool sixPlugged = true, bool sevenPlugged = true,
                           const MechanismTable& mechanismTable = kRobotTwoMechanisms,
                           std::span<const HoldControl> holdControls = RobotTwoDriver::kHolds,
                           std::span<const ToggleControl> toggleControls = RobotTwoDriver::kToggles)
        : table{&mechanismTable}, holds{holdControls}, toggles{toggleControls} {
        pros::shim::resetAll();
        for (const int p : {11, 12, 13, 14, 15, 16, 17, 18, 20}) {  // the 09-25 census (19 empty)
            pros::shim::setPluggedAtPort(p, pros::c::E_DEVICE_MOTOR);
        }
        pros::shim::setPluggedAtPort(2, pros::c::E_DEVICE_IMU);
        if (sixPlugged) pros::shim::setPluggedAtPort(6, pros::c::E_DEVICE_MOTOR);
        if (sevenPlugged) pros::shim::setPluggedAtPort(7, pros::c::E_DEVICE_MOTOR);
        const ProsDeviceRegistry registry;
        RecordRead record{};
        record.description = robotTwoRecord();
        record.decoded = true;
        boot = decideBootSourceFrom(record, registry, DrivetrainDescription{},
                                    "track width, external ratio (motor->wheel)",
                                    *table, "2026 TANK CHASSIS (robot two)",
                                    [this](const char* l) { bootLines.emplace_back(l); });
        REQUIRE_MESSAGE(boot.ok, boot.why);
        devices.emplace();
        const shulib::robot::BuildVerdict v =
            shulib::robot::validate(boot.built.config, *devices, boot.built.buildOptions());
        REQUIRE_MESSAGE(v.ok, v.why);
        robot.emplace(boot.built.config, *devices, boot.built.buildOptions());
        char why[192] = "";
        REQUIRE_MESSAGE(claws.bind(*robot, holds, toggles, kRezero, why, sizeof why), why);
    }

    /// Run the program's loop for `ticks` ticks; `script(k)` runs at the TOP of tick k (inside the
    /// disabled predicate, which the loop polls first). The re-zero button is the driver file's (Y).
    std::vector<Snap> run(int ticks, const std::function<void(int)>& script) {
        std::vector<Snap> log;
        int k = 0;
        const DriverLoopConfig cfg{.tickPeriod = Time{0.01}, .rezeroButton = kRezero};
        shulib::teleop::runDriverControl(
            robot->chassis(), master, devices->pacer(), shaper, cfg, claws.holds(),
            claws.toggles(),
            [&] {
                if (k >= ticks) return true;
                script(k);
                return false;
            },
            [] { return HeadingSample{.radians = 0.0, .trusted = true}; },
            [&](const shulib::math::ChassisSpeeds&, bool, const DriveShaperState&) {
                Snap s;
                for (int p = 1; p <= 21; ++p) {
                    const auto& m = pros::shim::motorState(p);
                    s.mv[static_cast<std::size_t>(p)] = m.lastVoltageMv;
                    s.brakes[static_cast<std::size_t>(p)] = m.brakeCalls;
                    s.moves[static_cast<std::size_t>(p)] = m.moveVoltageCalls;
                    s.stopped[static_cast<std::size_t>(p)] =
                        m.control == pros::shim::MotorControl::Stopped;
                }
                s.lineWrites = lineA().size();
                s.clawRow = clawRowText(*table, *robot, claws.anyReleasedLeftoverClaim()).text;
                log.push_back(s);
                ++k;
            });
        return log;
    }

    const MechanismTable* table;
    std::span<const HoldControl> holds;
    std::span<const ToggleControl> toggles;
    BootOutcome boot;
    std::vector<std::string> bootLines;
    std::optional<ProsDeviceProvider> devices;
    std::optional<shulib::robot::Robot> robot;
    DriverBindings claws;
    ProsController master{ControllerId::Master};
    DriveShaper shaper{DriverProfile{}};
};

/// The sticks both runs of the drive comparison share: forward and turn, never centred for long.
void sticks(int k) {
    pad().analog[pros::E_CONTROLLER_ANALOG_LEFT_Y] =
        static_cast<std::int32_t>(100.0 * std::sin(0.09 * k));
    pad().analog[pros::E_CONTROLLER_ANALOG_RIGHT_X] =
        static_cast<std::int32_t>(60.0 * std::cos(0.05 * k));
}

/// The button script of test 1: each of R1, R2, L1, L2 held ALONE for ten ticks, with ten ticks
/// of nothing between.
void buttons(int k) {
    press(ControllerButton::R1, k >= 10 && k < 20);
    press(ControllerButton::R2, k >= 30 && k < 40);
    press(ControllerButton::L1, k >= 50 && k < 60);
    press(ControllerButton::L2, k >= 70 && k < 80);
}

}  // namespace

// ═══ Test 1 (the T3 brief). The pivots, through the shim, with the REAL table ═══════════
TEST_CASE("T3 P2 test 1: through ProsDeviceProvider and the PROS shim with the REAL table — R1 "
          "alone drives port 6 at +12000 mV and leaves 7 alone, R2 -12000, L1 port 7 +12000, L2 "
          "-12000; each release is ONE brake() under HOLD; the drive ports get exactly what they "
          "get with no button") {
    // BUG CAUGHT: swapped buttons (R1/R2 or R1/L1) in the table or the binding; swapped pivots; a
    // wrong sign or voltage; a pivot built with Coast (its release would coast, not hold); a
    // release re-sent every tick (a brake() each tick may re-capture the hold point, DEFECTS2's
    // HA); a binding that reaches a drive port, or a drive that notices a claw button.
    std::vector<Snap> with;
    {
        RobotTwoClaws t;
        with = t.run(90, [](int k) {
            sticks(k);
            buttons(k);
        });
    }
    REQUIRE(with.size() == 90U);
    // Loop entry: each pivot made safe once (Hold, then brake()) — before anything is pressed.
    CHECK(with[0].brakes[6] == 1);
    CHECK(with[0].brakes[7] == 1);
    CHECK(with[9].stopped[6]);
    CHECK(with[9].stopped[7]);
    // R1 alone: port 6 forward at full power, in the tick it was read; port 7 untouched.
    for (std::size_t k = 10; k < 20; ++k) {
        CAPTURE(k);
        CHECK(with[k].mv[6] == 12000);
        CHECK_FALSE(with[k].stopped[6]);
        CHECK(with[k].moves[7] == with[9].moves[7]);
        CHECK(with[k].brakes[7] == 1);
    }
    CHECK(pros::shim::motorState(6).gearing == pros::v5::MotorGears::red);  // RED, not the drive's
    // The release: ONE brake() under HOLD, in the release tick, then silence for ten ticks.
    CHECK(with[20].stopped[6]);
    CHECK(with[20].brakes[6] == 2);
    for (std::size_t k = 20; k < 30; ++k) {
        CAPTURE(k);
        CHECK(with[k].brakes[6] == 2);
        CHECK(with[k].moves[6] == with[20].moves[6]);
    }
    // R2 alone: port 6 back at full power.
    CHECK(with[30].mv[6] == -12000);
    CHECK(with[39].mv[6] == -12000);
    CHECK(with[40].brakes[6] == 3);
    // L1 alone: port 7 forward, port 6 left where it was.
    CHECK(with[50].mv[7] == 12000);
    CHECK(with[59].mv[7] == 12000);
    CHECK(with[59].moves[6] == with[40].moves[6]);
    CHECK(with[59].brakes[6] == 3);
    CHECK(with[60].brakes[7] == 2);
    CHECK(with[70].mv[7] == -12000);
    CHECK(with[80].brakes[7] == 3);
    CHECK(with[80].stopped[7]);
    // Every stop landed under HOLD — the table's safe mode, reaching the motor (DEFECTS2).
    CHECK(pros::shim::motorState(6).stoppedUnder == pros::v5::MotorBrake::hold);
    CHECK(pros::shim::motorState(7).stoppedUnder == pros::v5::MotorBrake::hold);

    // The SAME sticks with no button: the drive ports get bit-identical millivolts every tick.
    std::vector<Snap> without;
    {
        RobotTwoClaws t;
        without = t.run(90, [](int k) { sticks(k); });
    }
    REQUIRE(without.size() == 90U);
    int moving = 0;
    for (std::size_t k = 0; k < 90; ++k) {
        for (const int p : kDrivePorts) {
            CAPTURE(k);
            CAPTURE(p);
            CHECK(with[k].mv[static_cast<std::size_t>(p)] == without[k].mv[static_cast<std::size_t>(p)]);
        }
        moving += std::abs(with[k].mv[16]) > 1000 ? 1 : 0;
        CHECK(without[k].mv[6] == 0);  // and without a button, no pivot ever moved
        CHECK(without[k].mv[7] == 0);
    }
    CHECK(moving > 30);  // the drive really moved — identity of two idle runs would prove nothing
}

TEST_CASE("T3 P2 test 1b: both buttons of ONE pivot held => it HOLDS (neither wins), releasing one "
          "lets the other drive; R1 + L2 together drive both pivots, opposite ways, in one tick") {
    // BUG CAUGHT: "forward wins" on both held (ruling R2); the two pivots sharing one mechanism
    // (R1 would move both claws).
    RobotTwoClaws t;
    const auto log = t.run(20, [](int k) {
        press(ControllerButton::R1, k >= 2 && k < 12);
        press(ControllerButton::R2, k >= 5 && k < 9);
        press(ControllerButton::L2, k >= 14 && k < 17);
        press(ControllerButton::R1, (k >= 2 && k < 12) || (k >= 14 && k < 17));
    });
    REQUIRE(log.size() == 20U);
    CHECK(log[4].mv[6] == 12000);
    CHECK(log[5].stopped[6]);  // both held: HOLD, at once
    CHECK(log[5].brakes[6] == 2);
    CHECK(log[8].brakes[6] == 2);  // …once, not every tick
    CHECK(log[9].mv[6] == 12000);  // R2 let go: R1 drives again
    CHECK_FALSE(log[9].stopped[6]);
    CHECK(log[14].mv[6] == 12000);  // R1 + L2 in ONE tick
    CHECK(log[14].mv[7] == -12000);
}

// ═══ The claw on A ═══════════════════════════════════════════════════════════════════
TEST_CASE("T3 P2 test 1c: the claw on A through the shim — the FIRST level 3-wire port A ever sees "
          "is the START level (no boot twitch) and the toggle's initial level is that same level; "
          "each press flips it ONCE; a controller drop and a disable leave it where it is") {
    // BUG CAUGHT: the line constructed at the opposite level and then set (a twitch at boot); the
    // toggle's initial level taken from anything but the start command (the claw would move when
    // driver control starts); the claw's POSITION handed over as its level; a flip on a held A; a
    // drop or a disable that commands the claw.
    RobotTwoClaws t;
    const bool start = shulib::bench::pneumaticStartCommand(kRobotTwoMechanisms.pneumatics[0]);
    CHECK(start == !shulib::bench::kClawOpenCommand);  // CLOSED at the start, through the polarity
    // Built: the line was driven ONCE, at the start level, and never anything before it.
    REQUIRE(lineA() == std::vector<std::int32_t>{start ? 1 : 0});
    // The config's start level and the toggle's initial level are ONE value (the boot's decision 3).
    const cfgns::MechanismConfig& claw = t.robot->config().mechanisms[2];
    REQUIRE(t.claws.toggleFor("claw") != nullptr);
    CHECK(t.claws.toggleFor("claw")->initialState() == cfgns::toLineValue(claw.pneumaticStart));
    CHECK(t.claws.toggleFor("claw")->initialState() == start);
    CHECK(t.claws.toggleFor("claw")->button() == ControllerButton::A);

    const auto log = t.run(40, [](int k) {
        press(ControllerButton::A, (k >= 5 && k < 9) || (k >= 15 && k < 17) || (k >= 21 && k < 24));
        pad().connected = !(k >= 20 && k < 26);  // the controller drops, A "down" meanwhile
        press(ControllerButton::Y, k >= 30 && k < 33);  // the re-zero: never the claw's
    });
    REQUIRE(log.size() == 40U);
    const std::int32_t s = start ? 1 : 0;
    const std::int32_t o = start ? 0 : 1;
    // Entry: the toggle's own initial level — the SAME level, so the valve does not move.
    CHECK(log[0].lineWrites == 2U);
    CHECK(lineA()[1] == s);
    CHECK(log[4].lineWrites == 2U);
    CHECK(log[5].lineWrites == 3U);  // the press at 5: OPEN
    CHECK(lineA()[2] == o);
    CHECK(log[8].lineWrites == 3U);  // held: nothing more
    CHECK(log[15].lineWrites == 4U);  // the press at 15: CLOSED again
    CHECK(lineA()[3] == s);
    // The drop (20-25, with A "down" in the shim at 21-23) and the reconnect: nothing.
    for (std::size_t k = 16; k < 40; ++k) {
        CAPTURE(k);
        CHECK(log[k].lineWrites == 4U);
    }
    // The disable at the end of the run commanded nothing to the claw either …
    CHECK(lineA().size() == 4U);
    // … and PROS calling opcontrol() again sends no initial level (once in the binding's life).
    t.run(3, [](int) {});
    CHECK(lineA().size() == 4U);
    CHECK(lineA().front() == s);
    // The pivots never moved on A or Y.
    CHECK(log.back().moves[6] == log[0].moves[6]);
    CHECK(log.back().moves[7] == log[0].moves[7]);
}

// ═══ The re-zero button, Y ═════════════════════════════════════════════════════════
TEST_CASE("T3 P2 test 1d: the re-zero button is Y — the driver file's claws bind with it, and are "
          "REFUSED with the old re-zero A (the claw's button), before anything is bound") {
    // BUG CAUGHT: the re-zero left on A (every claw press would also re-zero field-centric), or on
    // a pivot's shoulder button. (T4 Part 3: the re-zero is the driver file's kLoop.rezeroButton
    // since the table's kTeleopRezeroButton retired; the refusals are the library's
    // validBindings(), unchanged, reached through the driver file's binder.)
    CHECK(kRezero == ControllerButton::Y);
    RobotTwoClaws t;
    REQUIRE(t.claws.holds().size() == 2U);
    REQUIRE(t.claws.toggles().size() == 1U);
    char why[192] = "";
    CHECK_FALSE(t.claws.bind(*t.robot, RobotTwoDriver::kHolds, RobotTwoDriver::kToggles,
                             ControllerButton::A, why, sizeof why));
    CHECK(std::string_view{why}
          == "button A toggles mechanism 'claw' and is also the driver-control re-zero button -- "
             "give the re-zero another button");
    CHECK(t.claws.holds().empty());  // a refused set leaves NO binding behind
    CHECK(t.claws.toggles().empty());
    CHECK_FALSE(t.claws.bind(*t.robot, RobotTwoDriver::kHolds, RobotTwoDriver::kToggles,
                             ControllerButton::R2, why, sizeof why));
    CHECK(std::string_view{why}
          == "button R2 drives mechanism 'claw pivot 6' and is also the driver-control re-zero "
             "button -- give the re-zero another button");
    CHECK(t.claws.bind(*t.robot, RobotTwoDriver::kHolds, RobotTwoDriver::kToggles,
                       ControllerButton::Y, why, sizeof why));
}

// ═══ ABSENT ═══════════════════════════════════════════════════════════════════════════
TEST_CASE("T3 P2 test 1e: a pivot the provider refused (port 7 EMPTY) is ABSENT — no binding, "
          "L1/L2 do nothing, its boot line carries the provider's own sentence, the row and the "
          "controller say so — and the drive still runs") {
    // BUG CAUGHT: an absent claw that refuses the whole program (the drive would not run); a
    // binding over a missing motor; a boot line that hides WHY.
    RobotTwoClaws t{true, false};
    CHECK(t.robot->builtMechanismAt(1) == nullptr);
    CHECK(t.claws.holdFor("claw pivot 7") == nullptr);
    REQUIRE(t.claws.holdFor("claw pivot 6") != nullptr);
    CHECK(t.claws.holds().size() == 1U);
    CHECK(std::string{motorClawBootLine(kRobotTwoMechanisms, RobotTwoDriver::kHolds, *t.robot, 1).text}
          == std::string{"claw pivot 7: ABSENT -- port 7: nothing plugged in (expected a motor) -- "
                         "L1/L2 do nothing; the drive still runs"});
    CHECK(anyClawAbsent(*t.robot));
    const auto log = t.run(10, [](int k) {
        sticks(k + 5);
        press(ControllerButton::L1, true);
        press(ControllerButton::R1, k >= 5);
    });
    CHECK(pros::shim::motorState(7).moveVoltageCalls == 0);  // never even opened
    CHECK(pros::shim::motorState(7).setGearingCalls == 0);
    CHECK(log[6].mv[6] == 12000);                            // the built pivot works
    CHECK(std::abs(log[6].mv[16]) > 0);                      // and the drive runs
    CHECK(log[6].clawRow == "CLAW 6 FWD 20C | 7 ABSENT | CLAW CLOSED");
    // The port check did NOT call the missing claw anything — it is simply not there.
    CHECK(t.bootLines[0].find("claw pivots (declared, not drive motors): 6.") != std::string::npos);
}

TEST_CASE("T3 P2 test 1f: a claw whose 3-wire line the provider refused is ABSENT — no toggle, A "
          "does nothing, its boot line carries the provider's sentence (a FakeDeviceProvider: the "
          "brain's provider refuses only a line outside 1..8, which the config refuses first)") {
    shulib::hal::fake::FakeClock clock;
    shulib::hal::fake::FakeBattery battery;
    shulib::hal::fake::FakeTelemetrySink sink;
    struct NoPace final : shulib::motion::ITickPacer {
        void pace() override {}
    } pacer;
    shulib::hal::fake::FakeDeviceProvider devices{clock, battery, sink, pacer};
    devices.refuseLine(1);
    RecordRead record{};
    record.description = robotTwoRecord();
    record.decoded = true;
    shulib::hal::fake::FakeDeviceRegistry census;
    for (const int p : {6, 7, 11, 12, 13, 14, 15, 16, 17, 18, 20}) {
        census.setKind(p, shulib::hal::DeviceKind::Motor);
    }
    census.setKind(2, shulib::hal::DeviceKind::Imu);
    const BootOutcome boot = decideBootSourceFrom(
        record, census, DrivetrainDescription{}, "", kRobotTwoMechanisms, "robot two",
        [](const char*) {});
    REQUIRE(boot.ok);
    shulib::robot::Robot robot{boot.built.config, devices, boot.built.buildOptions()};
    DriverBindings claws;
    char why[192] = "";
    REQUIRE(claws.bind(robot, RobotTwoDriver::kHolds, RobotTwoDriver::kToggles, kRezero, why,
                       sizeof why));
    CHECK(claws.toggles().empty());
    CHECK(claws.toggleFor("claw") == nullptr);
    CHECK(claws.holds().size() == 2U);
    const std::string line =
        pneumaticClawBootLine(kRobotTwoMechanisms, RobotTwoDriver::kToggles, robot, 0).text;
    CHECK(line.starts_with("claw: ABSENT -- "));
    CHECK(line.find(robot.mechanismAbsence(2)) != std::string::npos);
    CHECK(line.ends_with(" -- A does nothing; the drive still runs"));
    CHECK(std::string{clawRowText(kRobotTwoMechanisms, robot, claws.anyReleasedLeftoverClaim()).text}
          == "CLAW 6 HOLD 0C | 7 HOLD 0C | CLAW ABSENT");  // a fake motor reads 0 C
}

// ═══ The words Station J quotes ════════════════════════════════════════════════════════
TEST_CASE("T3 P2 test 1g: the boot lines and the claw row, word for word — the texts worksheet "
          "Station J quotes") {
    // BUG CAUGHT: a boot line that drifts from the table or the driver file (a button, a port, a
    // volts, the stop word); the claw's start reported as the wrong position or level; a row that
    // runs past the SMALL budget. (T4 Part 3: the BUILT lines' end, `-- in shulib Teleop only`,
    // became `-- buttons: src/team/robot_two_driver.hpp` — the one reworded pin, logged; every
    // other word is unedited. The banner's claw sentence, clawBannerLine(), and its two pins RETIRED
    // with "shulib Teleop", the only program that printed it — worksheet J.3.2 with them.)
    RobotTwoClaws t;
    CHECK(std::string{motorClawBootLine(kRobotTwoMechanisms, RobotTwoDriver::kHolds, *t.robot, 0).text}
          == std::string{"claw pivot 6: BUILT -- port 6 (typed +6), RED, R1 forward / R2 reverse at "
                         "12 V, HOLD on release -- buttons: src/team/robot_two_driver.hpp"});
    CHECK(std::string{motorClawBootLine(kRobotTwoMechanisms, RobotTwoDriver::kHolds, *t.robot, 1).text}
          == std::string{"claw pivot 7: BUILT -- port 7 (typed +7), RED, L1 forward / L2 reverse at "
                         "12 V, HOLD on release -- buttons: src/team/robot_two_driver.hpp"});
    CHECK(std::string{
              pneumaticClawBootLine(kRobotTwoMechanisms, RobotTwoDriver::kToggles, *t.robot, 0).text}
          == std::string{"claw: BUILT -- 3-wire port A, CLOSED at the start (line low; line high "
                         "opens it -- polarity UNVERIFIED, HA-159); A toggles it; a disable leaves "
                         "it where it is"});
    // The row, before driver control: both pivots at 0 V (their stop word), the shim's 20 C.
    CHECK(std::string{clawRowText(kRobotTwoMechanisms, *t.robot, t.claws.anyReleasedLeftoverClaim())
                          .text}
          == "CLAW 6 HOLD 20C | 7 HOLD 20C | CLAW CLOSED");
    pros::shim::motorState(6).temperatureC = 47.6;
    pros::shim::motorState(7).temperatureC = 1234.0;  // clamped, never wider than the budget
    const auto log = t.run(6, [](int k) {
        press(ControllerButton::R1, k >= 1);
        press(ControllerButton::L2, k >= 2);
        press(ControllerButton::A, k == 3);
    });
    CHECK(log[1].clawRow == "CLAW 6 FWD 48C | 7 HOLD 999C | CLAW CLOSED");
    CHECK(log[2].clawRow == "CLAW 6 FWD 48C | 7 REV 999C | CLAW CLOSED");
    CHECK(log[3].clawRow == "CLAW 6 FWD 48C | 7 REV 999C | CLAW OPEN");
    CHECK(std::string{clawRowText(shulib::bench::kNoMechanisms, *t.robot, false).text}
          == "claws: none declared for this robot");
    CHECK(shulib::bench::clawRowWorstLength(shulib::bench::kNoMechanisms)
          == std::string_view{"claws: none declared for this robot"}.size());
    // THE BUDGET GUARD, tied to a row the code prints (review round 1, finding 4). The compile-time
    // check only says clawRowWorstLength() <= 50, so an UNDER-count (a word, a separator or
    // ` FREED` left out of the sum) would keep it passing while checking nothing — and robot two's
    // true worst row is exactly 50, where the row's `%.50s` would then silently cut FREED. So:
    // the function's answer for robot two is pinned EXACTLY, and the widest row robot two can
    // show — both pivots stopped at the clamp's 999 C, the claw CLOSED, a released claim — is
    // rendered by the code and has exactly that length, every character kept.
    CHECK(shulib::bench::clawRowWorstLength(kRobotTwoMechanisms) == 50U);
    {
        RobotTwoClaws worst;
        pros::shim::motorState(6).temperatureC = 5000.0;
        pros::shim::motorState(7).temperatureC = 1000.0;
        REQUIRE(worst.robot->mechanismAt(1)->tryClaim());  // released at entry: the row ends FREED
        const auto w = worst.run(1, [](int) {});
        REQUIRE(w.size() == 1U);
        CHECK(w[0].clawRow == "CLAW 6 HOLD 999C | 7 HOLD 999C | CLAW CLOSED FREED");
        CHECK(w[0].clawRow.size() == shulib::bench::clawRowWorstLength(kRobotTwoMechanisms));
    }
    // Every word is ASCII (a brain screen's font).
    for (const std::string& s :
         {std::string{motorClawBootLine(kRobotTwoMechanisms, RobotTwoDriver::kHolds, *t.robot, 0).text},
          std::string{
              pneumaticClawBootLine(kRobotTwoMechanisms, RobotTwoDriver::kToggles, *t.robot, 0).text},
          log[3].clawRow}) {
        for (const char c : s) {
            CHECK(static_cast<unsigned char>(c) < 0x80U);
        }
    }
}

// ═══ The claim ruling, end to end ═══════════════════════════════════════════════════════
TEST_CASE("T3 P2 test 1h: a claim an autonomous operation left on pivot 6 and on the claw is "
          "RELEASED at driver-control entry — the pivot is made safe and R1 drives from the first "
          "tick, the claw gets its initial level — and the row says FREED") {
    // BUG CAUGHT: the driver locked out of a claw for the whole driver period by a claim nobody
    // will ever release (the task that held it was deleted by the field's disable); a release the
    // panel does not show.
    RobotTwoClaws t;
    REQUIRE(t.robot->mechanismAt(0)->tryClaim());
    REQUIRE(t.robot->pneumaticMechanismAt(2)->tryClaim());
    const auto log = t.run(4, [](int) { press(ControllerButton::R1, true); });
    CHECK_FALSE(t.robot->mechanismAt(0)->claimed());
    CHECK_FALSE(t.robot->pneumaticMechanismAt(2)->claimed());
    CHECK(log[0].brakes[6] == 1);  // made safe at entry (only possible after the release)
    CHECK(log[0].mv[6] == 12000);
    CHECK(log[0].lineWrites == 2U);  // the initial level went out
    CHECK(log[0].clawRow == "CLAW 6 FWD 20C | 7 HOLD 20C | CLAW CLOSED FREED");
    // The next entry finds nothing to release: the word goes away.
    const auto again = t.run(2, [](int) { press(ControllerButton::R1, false); });
    CHECK(again[0].clawRow == "CLAW 6 HOLD 20C | 7 HOLD 20C | CLAW CLOSED");
    // EACH KIND ALONE says FREED — a released claim on only a pivot, then on only the claw (the
    // first run had both, so a panel reading one kind would have passed it).
    REQUIRE(t.robot->mechanismAt(1)->tryClaim());
    const auto pivotOnly = t.run(1, [](int) {});
    CHECK_FALSE(t.robot->mechanismAt(1)->claimed());
    CHECK(pivotOnly[0].clawRow == "CLAW 6 HOLD 20C | 7 HOLD 20C | CLAW CLOSED FREED");
    REQUIRE(t.robot->pneumaticMechanismAt(2)->tryClaim());
    const std::size_t writesBefore = lineA().size();
    const auto clawOnly = t.run(1, [](int) {});
    CHECK_FALSE(t.robot->pneumaticMechanismAt(2)->claimed());
    CHECK(clawOnly[0].clawRow == "CLAW 6 HOLD 20C | 7 HOLD 20C | CLAW CLOSED FREED");
    CHECK(lineA().size() == writesBefore);  // the release fired nothing at the claw
}

// ═══ BY NAME, never by index (T4 Part 3; test 1i and 1m until then) ═══════════════════════
// Until T4 Part 3 test 1i pinned TeleopClaws::bind()'s refusals of a robot not built from the table
// in its ORDER (another count, another name or the other kind at an entry's index), and test 1m
// that its 13 V entry THREW. Both retired with TeleopClaws. The driver file's binder resolves each
// control BY NAME, so its refusals are by name: test/robot_two_driver_test.cpp T10 (a name the
// robot lacks, a hold on a pneumatic, an extra mechanism no control names, no claw — each leaving
// nothing bound) and test/driver_controls_test.cpp's "refuses, by name"; 1m's 13 V throw is
// driver_controls_test's "a hold bind() cannot build (13 V) THROWS" (it says so). What 1i's
// "swapped" sub-case refused is now CORRECT, and is pinned here instead: a robot built from a table
// whose two pivots are listed in the other order gets the same buttons on the same motors.
TEST_CASE("T3 P2 test 1i (T4 Part 3): the driver file binds BY NAME — a robot built from a table "
          "with the pivots in the OTHER order binds, R1 still drives port 6 and L1 port 7, and each "
          "boot line still names its own pivot's buttons") {
    // BUG CAUGHT: a binder or a boot line that pairs a control with the mechanism at an INDEX (R1
    // would drive port 7 on such a robot, and its boot line would name the other pivot's buttons).
    MechanismTable swapped = kRobotTwoMechanisms;
    swapped.motors[0] = kRobotTwoMechanisms.motors[1];
    swapped.motors[1] = kRobotTwoMechanisms.motors[0];
    RobotTwoClaws t{true, true, swapped};
    REQUIRE(std::string_view{t.robot->config().mechanisms[0].name} == "claw pivot 7");
    REQUIRE(t.claws.holds().size() == 2U);
    CHECK(t.claws.holdFor("claw pivot 6")->forwardButton() == ControllerButton::R1);
    CHECK(t.claws.holdFor("claw pivot 7")->forwardButton() == ControllerButton::L1);
    CHECK(std::string{motorClawBootLine(swapped, RobotTwoDriver::kHolds, *t.robot, 0).text}
          == std::string{"claw pivot 7: BUILT -- port 7 (typed +7), RED, L1 forward / L2 reverse at "
                         "12 V, HOLD on release -- buttons: src/team/robot_two_driver.hpp"});
    CHECK(std::string{motorClawBootLine(swapped, RobotTwoDriver::kHolds, *t.robot, 1).text}
          == std::string{"claw pivot 6: BUILT -- port 6 (typed +6), RED, R1 forward / R2 reverse at "
                         "12 V, HOLD on release -- buttons: src/team/robot_two_driver.hpp"});
    const auto log = t.run(6, [](int k) {
        press(ControllerButton::R1, k >= 1 && k < 3);
        press(ControllerButton::L1, k >= 4);
    });
    CHECK(log[1].mv[6] == 12000);
    CHECK(log[1].mv[7] == 0);
    CHECK(log[4].mv[7] == 12000);
    CHECK(log[4].stopped[6]);
}

// ═══ The boot lines read the CONTROLS (T4 Part 3, the brief's P3-T5) ══════════════════════
TEST_CASE("T4 Part 3 P3-T5: the claw boot lines take the buttons and the volts from the driver "
          "file's CONTROLS, found by name — a controls copy with R1/R2 swapped prints R2 forward / "
          "R1 reverse, an ABSENT pivot names its buttons from the controls, other volts print, and "
          "an entry no control names prints `?`") {
    // BUG CAUGHT: a boot line that still reads the buttons from anywhere but the controls (the
    // table's old fields cannot compile any more — this is the run-time proof), one that pairs a
    // control by INDEX, or one that keeps a hard-coded 12 V.
    using B = ControllerButton;
    using namespace shulib::units::literals;
    const HoldControl swappedSix[] = {{"claw pivot 6", B::R2, B::R1, 12_volt},
                                      {"claw pivot 7", B::L1, B::L2, 12_volt}};
    // The controls in the OTHER order, with pivot 7 at 6 V — the line must find each by NAME.
    const HoldControl reordered[] = {{"claw pivot 7", B::L1, B::L2, 6_volt},
                                     {"claw pivot 6", B::R1, B::R2, 12_volt}};
    const ToggleControl clawOnB[] = {{"claw", B::B}};
    {
        RobotTwoClaws t{true, false, kRobotTwoMechanisms, swappedSix};  // port 7 EMPTY
        CHECK(std::string{motorClawBootLine(kRobotTwoMechanisms, swappedSix, *t.robot, 0).text}
              == std::string{"claw pivot 6: BUILT -- port 6 (typed +6), RED, R2 forward / R1 reverse "
                             "at 12 V, HOLD on release -- buttons: src/team/robot_two_driver.hpp"});
        // The ABSENT pivot's buttons come from the controls too.
        const HoldControl swappedSeven[] = {{"claw pivot 6", B::R1, B::R2, 12_volt},
                                            {"claw pivot 7", B::L2, B::L1, 12_volt}};
        CHECK(std::string{motorClawBootLine(kRobotTwoMechanisms, swappedSeven, *t.robot, 1).text}
              == std::string{"claw pivot 7: ABSENT -- port 7: nothing plugged in (expected a "
                             "motor) -- L2/L1 do nothing; the drive still runs"});
        CHECK(std::string{motorClawBootLine(kRobotTwoMechanisms, RobotTwoDriver::kHolds, *t.robot, 1)
                              .text}
              == std::string{"claw pivot 7: ABSENT -- port 7: nothing plugged in (expected a "
                             "motor) -- L1/L2 do nothing; the drive still runs"});
    }
    RobotTwoClaws t;
    CHECK(std::string{motorClawBootLine(kRobotTwoMechanisms, reordered, *t.robot, 0).text}
          == std::string{"claw pivot 6: BUILT -- port 6 (typed +6), RED, R1 forward / R2 reverse at "
                         "12 V, HOLD on release -- buttons: src/team/robot_two_driver.hpp"});
    CHECK(std::string{motorClawBootLine(kRobotTwoMechanisms, reordered, *t.robot, 1).text}
          == std::string{"claw pivot 7: BUILT -- port 7 (typed +7), RED, L1 forward / L2 reverse at "
                         "6 V, HOLD on release -- buttons: src/team/robot_two_driver.hpp"});
    CHECK(std::string{pneumaticClawBootLine(kRobotTwoMechanisms, clawOnB, *t.robot, 0).text}
          == std::string{"claw: BUILT -- 3-wire port A, CLOSED at the start (line low; line high "
                         "opens it -- polarity UNVERIFIED, HA-159); B toggles it; a disable leaves "
                         "it where it is"});
    // An entry no control names (only a hand-built call can reach it: a driver file that misses an
    // entry does not compile, controlsCoverTable) prints `?` and 0 V — never another entry's.
    const HoldControl onlySeven[] = {{"claw pivot 7", B::L1, B::L2, 12_volt}};
    CHECK(std::string{motorClawBootLine(kRobotTwoMechanisms, onlySeven, *t.robot, 0).text}
          == std::string{"claw pivot 6: BUILT -- port 6 (typed +6), RED, ? forward / ? reverse at "
                         "0 V, HOLD on release -- buttons: src/team/robot_two_driver.hpp"});
    CHECK(std::string{pneumaticClawBootLine(kRobotTwoMechanisms, {}, *t.robot, 0).text}
          == std::string{"claw: BUILT -- 3-wire port A, CLOSED at the start (line low; line high "
                         "opens it -- polarity UNVERIFIED, HA-159); ? toggles it; a disable leaves "
                         "it where it is"});
    // The toggle is found BY NAME too (this run's mutation CL3, "the FIRST toggle", survived the
    // three lines above — robot two has one toggle, and an EMPTY list has no first): a list whose
    // only toggle names another mechanism prints `?`, and one with the claw's toggle SECOND prints
    // the claw's button, never the first entry's.
    const ToggleControl otherOnly[] = {{"not the claw", B::X}};
    CHECK(std::string{pneumaticClawBootLine(kRobotTwoMechanisms, otherOnly, *t.robot, 0).text}
          == std::string{"claw: BUILT -- 3-wire port A, CLOSED at the start (line low; line high "
                         "opens it -- polarity UNVERIFIED, HA-159); ? toggles it; a disable leaves "
                         "it where it is"});
    const ToggleControl clawSecond[] = {{"not the claw", B::X}, {"claw", B::B}};
    CHECK(std::string{pneumaticClawBootLine(kRobotTwoMechanisms, clawSecond, *t.robot, 0).text}
          == std::string{"claw: BUILT -- 3-wire port A, CLOSED at the start (line low; line high "
                         "opens it -- polarity UNVERIFIED, HA-159); B toggles it; a disable leaves "
                         "it where it is"});
}

// ═══ The merged fault row (ruling R3) ══════════════════════════════════════════════════
TEST_CASE("T3 P2 test 1j: the ONE fault row — `faults: none`, `FAULT n=<n> first <A> last <B>` when "
          "it fits 50 characters, else `FAULT n=<n> last <B>` — and EVERY pair of fault names, at "
          "any count, fits") {
    // BUG CAUGHT: a merged row that runs off the screen for the long names (MOTOR_GROUP_DISAGREE
    // with MECHANISM_STALLED is 59 characters in the full form); a row that drops the LAST fault
    // (the one to act on) when it shortens.
    CHECK(std::string{faultRowText(0, FaultCode::None, FaultCode::None).text} == "faults: none");
    CHECK(std::string{faultRowText(1, FaultCode::ImuLost, FaultCode::ImuLost).text}
          == "FAULT n=1 first IMU_LOST last IMU_LOST");
    CHECK(std::string{faultRowText(3, FaultCode::ImuLost, FaultCode::Brownout).text}
          == "FAULT n=3 first IMU_LOST last BROWNOUT");
    CHECK(std::string{faultRowText(12, FaultCode::MotorGroupDisagree, FaultCode::MechanismStalled)
                          .text}
          == "FAULT n=12 last MECHANISM_STALLED");
    // The fault robot two actually raises (review round 1, finding 1): its name is 20 characters,
    // so ONE of it already needs 62 in the full form — the row shows only the latest. Station F's
    // row 9 quotes exactly this line.
    CHECK(std::string{faultRowText(1, FaultCode::MotorGroupDisagree, FaultCode::MotorGroupDisagree)
                          .text}
          == "FAULT n=1 last MOTOR_GROUP_DISAGREE");
    // The budget's EDGE: exactly 50 characters is the full form (`<=`, not `<`) — the widest pair
    // that still names both, and the only one Station F quotes with MOTOR_GROUP_DISAGREE in it.
    const std::string edge =
        faultRowText(2, FaultCode::ImuLost, FaultCode::MotorGroupDisagree).text;
    CHECK(edge == "FAULT n=2 first IMU_LOST last MOTOR_GROUP_DISAGREE");
    CHECK(edge.size() == 50U);
    int shortened = 0;
    for (int a = 0; a < 64; ++a) {
        for (int b = 0; b < 64; ++b) {
            const auto fa = static_cast<FaultCode>(a);
            const auto fb = static_cast<FaultCode>(b);
            if (std::string_view{shulib::diag::faultCodeName(fa)} == "UNKNOWN"
                || std::string_view{shulib::diag::faultCodeName(fb)} == "UNKNOWN") {
                continue;
            }
            for (const int n : {1, 99, 2147483647}) {
                const std::string row = faultRowText(n, fa, fb).text;
                CAPTURE(row);
                CHECK(row.size() <= 50U);
                CHECK(row.find(shulib::diag::faultCodeName(fb)) != std::string::npos);
                shortened += row.find(" first ") == std::string::npos ? 1 : 0;
            }
        }
    }
    CHECK(shortened > 0);  // the fallback really ran
}

// ═══ The toggle's initial level is the config's START level — with start != safe ═══════════
namespace {

/// A table whose pneumatic START and SAFE differ, and whose polarities differ — the cases robot
/// two's claw cannot tell apart (its start and safe are both CLOSED, and CLOSED is LOW, which is
/// also what a CLOSED position read as a bool gives).
constexpr MechanismTable kStartIsNotSafe = {
    .motors = {{.name = "lift",
                .signedPort = -9,
                .cartridge = shulib::bench::Cartridge::Green,
                .safeMode = shulib::hal::BrakeMode::Brake,
                .provenance = "test"}},
    .motorCount = 1,
    .pneumatics = {{.name = "clamp",
                    .adiLine = 3,
                    .openCommand = false,  // LOW opens: CLOSED (the start) is HIGH
                    .start = shulib::bench::PneumaticPosition::Closed,
                    .safe = shulib::bench::PneumaticPosition::Open,
                    .provenance = "test"},
                   {.name = "wing",
                    .adiLine = 8,
                    .openCommand = true,  // HIGH opens: OPEN (the start) is HIGH
                    .start = shulib::bench::PneumaticPosition::Open,
                    .safe = shulib::bench::PneumaticPosition::Closed,
                    .provenance = "test"}},
    .pneumaticCount = 2,
};

/// Its controls — the buttons and the volts the table carried until T4 Part 3 moved them to a
/// driver file's controls (Up / Down at 8 V for the lift, X for the clamp, B for the wing).
constexpr HoldControl kStartIsNotSafeHolds[] = {
    {"lift", ControllerButton::Up, ControllerButton::Down, shulib::units::Voltage{8.0}}};
constexpr ToggleControl kStartIsNotSafeToggles[] = {{"clamp", ControllerButton::X},
                                                    {"wing", ControllerButton::B}};

}  // namespace

TEST_CASE("T3 P2 test 1k: each toggle's initial level is its config's START level — the level the "
          "builder drove the line to — never its SAFE level and never its position; both polarities") {
    // BUG CAUGHT: bind() giving the toggle pneumaticSafeCommand() (the claw would move to its safe
    // level the moment driver control started), or the start POSITION as a level (backwards for
    // the polarity where LOW opens).
    shulib::hal::fake::FakeClock clock;
    shulib::hal::fake::FakeBattery battery;
    shulib::hal::fake::FakeTelemetrySink sink;
    struct NoPace final : shulib::motion::ITickPacer {
        void pace() override {}
    } pacer;
    shulib::hal::fake::FakeDeviceProvider devices{clock, battery, sink, pacer};
    shulib::hal::fake::FakeDeviceRegistry census;
    for (const int p : {11, 12, 13, 15, 16, 17, 18, 20}) {
        census.setKind(p, shulib::hal::DeviceKind::Motor);
    }
    census.setKind(2, shulib::hal::DeviceKind::Imu);
    RecordRead record{};
    record.description = robotTwoRecord();
    record.decoded = true;
    const BootOutcome boot = decideBootSourceFrom(record, census, DrivetrainDescription{}, "",
                                                  kStartIsNotSafe, "test", [](const char*) {});
    REQUIRE_MESSAGE(boot.ok, boot.why);
    shulib::robot::Robot robot{boot.built.config, devices, boot.built.buildOptions()};
    DriverBindings claws;
    char why[192] = "";
    REQUIRE_MESSAGE(claws.bind(robot, kStartIsNotSafeHolds, kStartIsNotSafeToggles, kRezero, why,
                               sizeof why),
                    why);
    REQUIRE(claws.toggles().size() == 2U);
    const std::uint8_t lines[2] = {3, 8};
    const char* const names[2] = {"clamp", "wing"};
    for (std::size_t j = 0; j < 2; ++j) {
        CAPTURE(j);
        const cfgns::MechanismConfig& m = robot.config().mechanisms[1 + j];
        const bool start = cfgns::toLineValue(m.pneumaticStart);
        CHECK(start);  // both START levels are HIGH (see the table)
        CHECK_FALSE(cfgns::toLineValue(m.pneumaticSafe));  // both SAFE levels are LOW
        REQUIRE(claws.toggleFor(names[j]) != nullptr);
        CHECK(claws.toggleFor(names[j])->initialState() == start);
        CHECK(devices.digitalOutInitialState(lines[j]) == start);  // the builder made it there
    }
    REQUIRE(claws.holdFor("lift") != nullptr);
    CHECK(claws.holdFor("lift")->volts().value() == 8.0);
    CHECK(claws.holdFor("lift")->forwardButton() == ControllerButton::Up);
    CHECK(claws.holdFor("lift")->reverseButton() == ControllerButton::Down);
    // The claw row for a table robot two's cannot imitate: a stopped mechanism says ITS declared
    // mode (Brake, not robot two's Hold), a flipped port prints as its magnitude, and each claw's
    // word is read through its OWN polarity — the clamp is HIGH and CLOSED, the wing HIGH and OPEN.
    CHECK(std::string{clawRowText(kStartIsNotSafe, robot, claws.anyReleasedLeftoverClaim()).text}
          == "CLAW 9 BRAKE 0C | CLAMP CLOSED | WING OPEN");
}

// ═══ A pivot FLIPPED in the table — Station J's fix for a claw that moves the wrong way ═══════
TEST_CASE("T3 P2 test 1n: a pivot flipped to -6 in the table is still PORT 6 in every word the "
          "program prints — the BUILT boot line says `port 6 (typed -6)`, the claw row `6` whether "
          "the pivot is built or ABSENT — and a 23-character name is printed whole") {
    // BUG CAUGHT (review round 1, finding 5): a text that prints the SIGNED port — after the flip
    // Station J asks for, the boot line would read `port -6 (typed -6)` and the row `CLAW -6 ...`,
    // neither of which is a port on the brain. Tests 6u / 6v pin the port CHECK's magnitudes; only
    // this case reaches the words. Also: the flip really reaches the motor — the same R1 now sends
    // the shim's port 6 the reversed command. (T4 Part 3: the banner's `R1/R2 port 6` pin retired
    // with clawBannerLine(); the BUILT line's end is reworded as test 1g's.)
    MechanismTable flipped = kRobotTwoMechanisms;
    flipped.motors[0].signedPort = -6;
    {
        RobotTwoClaws t{true, true, flipped};
        CHECK(std::string{motorClawBootLine(flipped, RobotTwoDriver::kHolds, *t.robot, 0).text}
              == std::string{"claw pivot 6: BUILT -- port 6 (typed -6), RED, R1 forward / R2 reverse "
                             "at 12 V, HOLD on release -- buttons: src/team/robot_two_driver.hpp"});
        CHECK(std::string{clawRowText(flipped, *t.robot, t.claws.anyReleasedLeftoverClaim()).text}
              == "CLAW 6 HOLD 20C | 7 HOLD 20C | CLAW CLOSED");
        const auto log = t.run(3, [](int k) { press(ControllerButton::R1, k >= 1); });
        // The library still commands +12 V ("forward" is the button's word, FWD on the row); the
        // sign lives in the port entry and PROS applies it once, so the DEVICE turns the other way
        // (the shim's record of what reached the motor: -12000 mV). That is the whole fix.
        CHECK(log[1].clawRow == "CLAW 6 FWD 20C | 7 HOLD 20C | CLAW CLOSED");
        CHECK(log[1].mv[6] == -12000);
    }
    {
        RobotTwoClaws t{false, true, flipped};  // the flipped pivot's port EMPTY
        CHECK(t.robot->builtMechanismAt(0) == nullptr);
        CHECK(std::string{motorClawBootLine(flipped, RobotTwoDriver::kHolds, *t.robot, 0).text}
              == std::string{"claw pivot 6: ABSENT -- port 6: nothing plugged in (expected a motor) "
                             "-- R1/R2 do nothing; the drive still runs"});
        CHECK(std::string{clawRowText(flipped, *t.robot, t.claws.anyReleasedLeftoverClaim()).text}
              == "CLAW 6 ABSENT | 7 HOLD 20C | CLAW CLOSED");
    }
    // A name of 23 characters — the most a config name field holds — is carried WHOLE into the
    // config (copyMechanismName's bound) and printed whole by the boot line, and the binder finds
    // it by that whole name (a controls copy with the same name: the table and the driver file must
    // agree on a name, which controlsNameTable checks at compile time for robot two's real pair).
    MechanismTable longName = kRobotTwoMechanisms;
    longName.motors[0].name = "claw pivot six, the lef";  // 23
    REQUIRE(std::string_view{longName.motors[0].name}.size() == 23U);
    const HoldControl longHolds[] = {
        {"claw pivot six, the lef", ControllerButton::R1, ControllerButton::R2,
         shulib::units::Voltage{12.0}},
        RobotTwoDriver::kHolds[1]};
    RobotTwoClaws t{true, true, longName, longHolds};
    REQUIRE(t.claws.holdFor("claw pivot six, the lef") != nullptr);
    CHECK(std::string_view{t.robot->config().mechanisms[0].name} == "claw pivot six, the lef");
    CHECK(std::string_view{t.robot->builtMechanismAt(0)->name()} == "claw pivot six, the lef");
    CHECK(std::string{motorClawBootLine(longName, longHolds, *t.robot, 0).text}
              .starts_with("claw pivot six, the lef: BUILT -- port 6 (typed +6), RED, R1 forward"));
}

// ═══ "shulib Teleop"'s own wiring, read from its source — RETIRED (T4 Part 3) ═══════════════════
// Test 1l read src/programs/teleop_program.cpp, which T4 Part 3 deleted with "shulib Teleop". Its
// pins moved to the program that replaced it: test/drive_program_source_test.cpp reads
// src/programs/drive_program.cpp for the same things — the driver file bound inside the boot
// boundary's try and unbound first in its catch, the loop reached only through the driver file,
// the boot lines printed from the build's table AND the driver file's controls, the exit line —
// and keeps 1l's pin on src/common/boot_source.cpp (the boot source hands the build's mechanism
// table to the decision). Its re-zero pins went with the table's kTeleopRezeroButton: the re-zero
// is the driver file's own constant (test 1d above), and the driver file static_asserts that no
// control takes it (controlsAvoid).
