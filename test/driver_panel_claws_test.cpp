// driver_panel_claws_test.cpp — every word a driver-control program on robot two says about its
// MECHANISMS, end to end through the REAL src/ list, robot two's REAL driver file, the mechanism
// service, hal::pros::ProsDeviceProvider and the PROS shim (T3 Part 2's test 1, written for "shulib
// Teleop" as test/teleop_claws_test.cpp; re-pointed at the driver file at T4 Part 3; moved onto the
// mechanism core's words by MECH-A5, 2026-10-29).
//
// ═══ WHAT THIS FILE PINS ════════════════════════════════════════════════════════════
// src/common/driver_panel.hpp holds every sentence a program says about the mechanisms (the
// programs themselves cannot be compiled on the host: no screen shim). Since MECH-A5:
//   * THE MECHANISM BOOT LINES — one block per mechanism: what it IS, one line per ROLE
//     (mechanism::roleLine over robot::MechanismParts' facts), one per DRIVER CONTROL, and its state
//     (homed; no stall signature; starting values) — robot two's, word for word as worksheet
//     Stations J and K quote them, BUILT and ABSENT; and each line builder over the fixture robots'
//     lists and controls (every kind and every control word), so a line a robot two cannot reach is
//     pinned too;
//   * THE ROUTES LINE the two auton programs print;
//   * THE CLAW ROW — read from the MECHANISM SERVICE (a pneumatic's position by its declaration's
//     levels) and the Robot (a pivot's commanded volts and temperature), FREED from the driver file;
//   * the merged fault row (unchanged).
// What the claws DO on each input is test/robot_two_driver_test.cpp's (its M5a proves it equal to
// T3's bindings tick by tick); the cases here that drive buttons do so for the words they show.
// The chooser, the controller's lines and the PROTECTION row are test/driver_panel_test.cpp's.
//
// NO HARDWARE CLAIM. The shim models our BELIEF about PROS (its own header says so): nothing here
// shows a claw moving, a pivot holding a load (HA-156), or which way either turns (HA-153), and the
// claw's levels are the robot file's guess (HA-159). Station J is where those are measured.

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

#include "common/boot_source.hpp"
#include "common/driver_panel.hpp"
#include "mechanism_driver_fixtures.hpp"
#include "mechanism_fixtures.hpp"
#include "pros/shim_control.hpp"
#include "robots/mechanism_table.hpp"
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
#include "shulib/hal/pros/device_provider.hpp"
#include "shulib/hal/pros/device_registry.hpp"
#include "shulib/mechanism/route_ids.hpp"
#include "shulib/mechanism/service.hpp"
#include "shulib/robot/robot.hpp"
#include "shulib/teleop/driver_control.hpp"
#include "shulib/units/quantity.hpp"
#include "team/robot_two_driver.hpp"

using shulib::bench::anyClawAbsent;
using shulib::bench::BootOutcome;
using shulib::bench::clawRowText;
using shulib::bench::decideBootSourceFrom;
using shulib::bench::faultRowText;
using shulib::bench::MechanismLine;
using shulib::bench::MechanismTable;
using shulib::bench::RecordRead;
using shulib::bench::RobotTwoDriver;
using shulib::diag::FaultCode;
using shulib::hal::ControllerButton;
using shulib::hal::DescriptionSource;
using shulib::hal::DrivetrainDescription;
using shulib::hal::DrivetrainKind;
using shulib::hal::MotorCartridge;
using shulib::hal::pros::ProsDeviceProvider;
using shulib::hal::pros::ProsDeviceRegistry;
using shulib::mechanism::MechanismService;
using shulib::units::Length;
namespace cfgns = shulib::config;
namespace panel_fixture {
using namespace shulib::mechanism::words;
/// A list robot two's cannot imitate (the row case above): a REVERSED Green Brake roller on 9, a clamp
/// on C whose 'closed' is HIGH, starting 'closed' and made safe 'open', and a wing on H that starts
/// 'open' and is made safe 'closed'.
inline constexpr Power roller{.name = "roller", .motors = {Reversed}, .cartridge = Green, .safe = Brake};
inline constexpr Pneumatic clamp{.name = "clamp", .positions = {{"closed", {High}}, {"open", {Low}}},
                                 .start = "closed", .safe = "open", .moveTime = 0.2_s};
inline constexpr Pneumatic wing{.name = "wing", .positions = {{"closed", {Low}}, {"open", {High}}},
                                .start = "open", .safe = "closed", .moveTime = 0.2_s};
inline constexpr MechanismTable kList = mechanisms(wire(roller, motors(9)), wire(clamp, lines('C')),
                                                   wire(wing, lines('H')));
/// The longest name a Power can carry in a list (19 characters: its route ids, "claw_pivot_six_lef_stop",
/// must fit 23 — route_ids.hpp).
inline constexpr Power longPivot{.name = "claw pivot six, lef", .motors = {Forward}, .cartridge = Red, .safe = Hold};
inline constexpr MechanismTable kLong = mechanisms(wire(longPivot, motors(6)));
}  // namespace panel_fixture
using panel_fixture::kList;
using panel_fixture::kLong;
namespace two = shulib::bench::robot_two;
namespace rx = mech_fixture::robot_x;
namespace ry = mech_fixture::robot_y;

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
    std::snprintf(d.provenance, sizeof d.provenance, "%s", "driver panel mechanisms test (the 09-25 record)");
    return d;
}

constexpr int kDrivePorts[8] = {16, 17, 18, 20, 11, 12, 13, 15};

/// The shim's controller button index for a library button (misc.hpp's enum: L1 = 6 … A = 17, in
/// ControllerButton's own order).
std::size_t shimButton(ControllerButton b) { return 6U + static_cast<std::size_t>(b); }
pros::shim::ControllerState& pad() { return pros::shim::controllerState(pros::E_CONTROLLER_MASTER); }
void press(ControllerButton b, bool down) { pad().digital[shimButton(b)] = down; }

/// A driver-control program's composition for robot two, on the host ("shulib Drive"'s): the boot
/// decision (the REAL list unless a case hands a copy), the Robot over the REAL provider, the
/// mechanism service around the raw tick bound over the Robot's built mechanisms, the driver file
/// bound to it. `sixPlugged` / `sevenPlugged` say which claw pivots carry a motor.
struct RobotTwoClaws {
    explicit RobotTwoClaws(bool sixPlugged = true, bool sevenPlugged = true,
                           const MechanismTable& list = two::kRobotTwoMechanisms)
        : table{&list} {
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
                                    "track width, external ratio (motor->wheel)", *table,
                                    "2026 TANK CHASSIS (robot two)",
                                    [this](const char* l) { bootLines.emplace_back(l); });
        REQUIRE_MESSAGE(boot.ok, boot.why);
        devices.emplace();
        const shulib::robot::BuildVerdict v =
            shulib::robot::validate(boot.built.config, *devices, boot.built.buildOptions());
        REQUIRE_MESSAGE(v.ok, v.why);
        robot.emplace(boot.built.config, *devices, boot.built.buildOptions());
        mechanisms.emplace(devices->pacer());
        mechanisms->bind(*table, robot->mechanismParts().devices(), robot->context().clock(), robot->faults(),
                         robot->context().telemetry());
        char why[192] = "";
        REQUIRE_MESSAGE(driver.bind(*mechanisms, why, sizeof why), why);
    }
    /// Every mechanism boot line, as the programs print them (the driver file's controls).
    [[nodiscard]] std::vector<MechanismLine> lines(const shulib::mechanism::DriverControls* controls =
                                                       &RobotTwoDriver::kControls) const {
        std::vector<MechanismLine> out;
        shulib::bench::mechanismBootLines(*table, *robot, *mechanisms, controls,
                                          [&out](const MechanismLine& m) { out.push_back(m); });
        return out;
    }
    /// The claw row after each of `ticks` driver-control ticks; `script(k)` at the top of tick k.
    std::vector<std::string> run(int ticks, const std::function<void(int)>& script) {
        std::vector<std::string> rows;
        int k = 0;
        driver.run(
            *robot, *mechanisms, devices->pacer(),
            [&] {
                if (k >= ticks) return true;
                script(k);
                return false;
            },
            [&](const shulib::math::ChassisSpeeds&, bool, const shulib::teleop::DriveShaperState&) {
                rows.emplace_back(
                    clawRowText(*table, *robot, *mechanisms, driver.releasedLeftoverAtEntry(*mechanisms)).text);
                ++k;
            });
        return rows;
    }

    const MechanismTable* table;
    BootOutcome boot;
    std::vector<std::string> bootLines;
    std::optional<ProsDeviceProvider> devices;
    std::optional<shulib::robot::Robot> robot;
    std::optional<MechanismService> mechanisms;
    RobotTwoDriver driver;
};

/// The texts of `lines`, and every line at most 199 characters and ASCII (one TermSink line, a
/// brain's font).
std::vector<std::string> texts(const std::vector<MechanismLine>& lines) {
    std::vector<std::string> out;
    for (const MechanismLine& m : lines) {
        const std::string t{m.text};
        CHECK(t.size() <= 199U);
        for (const char c : t) {
            CHECK(static_cast<unsigned char>(c) < 0x80U);
        }
        out.push_back(t);
    }
    return out;
}

/// The index in `list` of the mechanism named `name`.
std::size_t indexIn(const MechanismTable& list, std::string_view name) {
    for (std::size_t i = 0; i < list.count; ++i) {
        if (std::string_view{list.specs[i].name} == name) return i;
    }
    FAIL("no mechanism named ", name);
    return 0;
}

}  // namespace

// ═══ Robot two's mechanism boot lines, word for word ═════════════════════════════════════════
TEST_CASE("MECH-A5 boot lines: robot two's, word for word — what each mechanism IS, one line per ROLE, "
          "one per CONTROL, and the pivots' no-stall line — in the list's order, the texts worksheet "
          "Stations J and K quote") {
    // BUG CAUGHT: a boot line drifting from the robot file (a port, a direction, the cartridge, the
    // stop word, the claw's levels or its start) or from the driver file (a button, the volts); the
    // list's order lost; a role line in other words than the library's; the pivots' only cut-off
    // (the driver) left unsaid; a line past one TermSink line, or not ASCII. (Until MECH-A5 two lines
    // per claw — `claw pivot 6: BUILT -- port 6 (typed +6), RED, R1 forward / R2 reverse at 12 V, HOLD
    // on release -- buttons: src/team/robot_two_driver.hpp` — said most of this.)
    RobotTwoClaws t;
    const std::vector<MechanismLine> lines = t.lines();
    CHECK(texts(lines) == std::vector<std::string>{
                             "claw pivot 6: POWER -- RED, motor 1 forward; let go: HOLD",
                             "claw pivot 6: motor 1 -> port 6 ok",
                             "claw pivot 6: hold-to-run R1 / R2, 12 V",
                             "'claw pivot 6': no stall signature -- held against its stop it is cut only by the driver "
                             "letting go (HA-154)",
                             "claw pivot 7: POWER -- RED, motor 1 forward; let go: HOLD",
                             "claw pivot 7: motor 1 -> port 7 ok",
                             "claw pivot 7: hold-to-run L1 / L2, 12 V",
                             "'claw pivot 7': no stall signature -- held against its stop it is cut only by the driver "
                             "letting go (HA-154)",
                             "claw: PNEUMATIC -- starts 'closed', safe 'closed'; closed = A low, open = A high (levels "
                             "UNVERIFIED until a bench station sees them); a disable leaves it where it is",
                             "claw: line 1 -> 3-wire A not checkable (the brain cannot see a 3-wire device)",
                             "claw: toggle on A ('closed' / 'open')"});
    for (const MechanismLine& m : lines) {
        CHECK(m.level == shulib::hal::LogLevel::Info);  // nothing absent: no warning
    }
    // The Auton Tester's block (no driver controls): the same, without the control lines.
    const std::vector<std::string> noControls = texts(t.lines(nullptr));
    CHECK(noControls.size() == 8U);
    for (const std::string& l : noControls) {
        CHECK(l.find("hold-to-run") == std::string::npos);
        CHECK(l.find("toggle on") == std::string::npos);
    }
}

TEST_CASE("MECH-A5 boot lines: an ABSENT pivot (port 7 EMPTY) — its role line carries the provider's own "
          "sentence and its control line says the buttons do nothing, both Warn; the row and the "
          "controller say so — and the drive still runs") {
    // BUG CAUGHT: an absent claw that refuses the whole program; a boot line that hides WHY, or still
    // promises its buttons. (Until MECH-A5: `claw pivot 7: ABSENT -- port 7: nothing plugged in
    // (expected a motor) -- L1/L2 do nothing; the drive still runs`.)
    RobotTwoClaws t{true, false};
    CHECK(t.robot->builtMechanismAt(1) == nullptr);
    const std::vector<MechanismLine> lines = t.lines();
    REQUIRE(lines.size() == 11U);
    CHECK(std::string{lines[5].text} == "claw pivot 7: motor 1 -> port 7 ABSENT: port 7: nothing plugged in (expected a motor)");
    CHECK(lines[5].level == shulib::hal::LogLevel::Warn);
    CHECK(std::string{lines[6].text}
          == "claw pivot 7: hold-to-run L1 / L2, 12 V -- ABSENT: does nothing; the drive still runs");
    CHECK(lines[6].level == shulib::hal::LogLevel::Warn);
    CHECK(lines[2].level == shulib::hal::LogLevel::Info);  // pivot 6 is fine
    CHECK(anyClawAbsent(*t.robot));
    const auto rows = t.run(10, [](int k) {
        press(ControllerButton::L1, true);
        press(ControllerButton::R1, k >= 5);
    });
    CHECK(pros::shim::motorState(7).moveVoltageCalls == 0);  // never even opened
    CHECK(pros::shim::motorState(7).setGearingCalls == 0);
    CHECK(rows[6] == "CLAW 6 FWD 20C | 7 ABSENT | CLAW CLOSED");
    // The port check did NOT call the missing claw anything — it is simply not there.
    CHECK(t.bootLines[0].find("claw pivots (declared, not drive motors): 6.") != std::string::npos);
}

TEST_CASE("MECH-A5 boot lines: a claw whose 3-wire line the provider refused is ABSENT — its role line "
          "has the provider's sentence, its toggle line says A does nothing, the row says ABSENT (a "
          "FakeDeviceProvider: the brain's refuses only a line outside 1..8, which the config refuses "
          "first)") {
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
    const BootOutcome boot = decideBootSourceFrom(record, census, DrivetrainDescription{}, "",
                                                  two::kRobotTwoMechanisms, "robot two", [](const char*) {});
    REQUIRE(boot.ok);
    shulib::robot::Robot robot{boot.built.config, devices, boot.built.buildOptions()};
    MechanismService svc{pacer};
    svc.bind(two::kRobotTwoMechanisms, robot.mechanismParts().devices(), robot.context().clock(), robot.faults(),
             robot.context().telemetry());
    std::vector<MechanismLine> lines;
    shulib::bench::mechanismBootLines(two::kRobotTwoMechanisms, robot, svc, &RobotTwoDriver::kControls,
                                      [&lines](const MechanismLine& m) { lines.push_back(m); });
    REQUIRE(lines.size() == 11U);
    const std::string role{lines[9].text};
    CHECK(role.starts_with("claw: line 1 -> 3-wire A ABSENT: "));
    CHECK(role.find(robot.mechanismAbsence(2)) != std::string::npos);
    CHECK(lines[9].level == shulib::hal::LogLevel::Warn);
    CHECK(std::string{lines[10].text}
          == "claw: toggle on A ('closed' / 'open') -- ABSENT: does nothing; the drive still runs");
    CHECK(std::string{clawRowText(two::kRobotTwoMechanisms, robot, svc, false).text}
          == "CLAW 6 HOLD 0C | 7 HOLD 0C | CLAW ABSENT");  // a fake motor reads 0 C
}

// ═══ Each line builder over every kind and every control word ════════════════════════════════
TEST_CASE("MECH-A5 boot lines: what each kind IS — an ARM, a two-motor LIFT with a mirrored motor, a "
          "FLYWHEEL, a Power with a sensor, a three-position pneumatic on two lines (robot x), and a "
          "Power WITH a stall signature has no no-stall line (robot y's catapult)") {
    // BUG CAUGHT: a kind word wrong (a lift called an arm), a Reversed motor printed forward (the
    // direction check's whole subject), the made-safe word for a motor kind lost, a pneumatic's levels
    // per line in the wrong order, its start or safe position swapped, the no-stall line on a Power
    // that HAS a signature (a false alarm), or none on one that has not.
    const MechanismTable& x = rx::kRobotXMechanisms;
    const auto decl = [&x](std::string_view n) {
        return std::string{shulib::bench::mechanismDeclarationLine(x, indexIn(x, n)).text};
    };
    CHECK(decl("arm") == "arm: ARM -- GREEN, motor 1 forward; made safe: HOLD");
    CHECK(decl("lift") == "lift: LIFT -- RED, motor 1 forward, motor 2 reversed; made safe: HOLD");
    CHECK(decl("flywheel") == "flywheel: FLYWHEEL -- BLUE, motor 1 forward; made safe: COAST");
    CHECK(decl("intake") == "intake: POWER -- BLUE, motor 1 forward; let go: COAST");
    CHECK(decl("hood")
          == "hood: PNEUMATIC -- starts 'down', safe 'down'; down = C low D low, middle = C high D low, up = C "
             "high D high (levels UNVERIFIED until a bench station sees them); a disable leaves it where it is");
    // The state lines: the arm rests at its stop (homed at bind); the lift homes by switch — NOT
    // HOMED until something homes it; the flywheel and the arm print their starting values; the
    // intake (no signature) its no-stall line; robot y's catapult (a signature) none.
    const auto state = [](const MechanismTable& list, std::string_view n, bool homed) {
        std::vector<std::string> out;
        shulib::bench::mechanismStateLines(list, indexIn(list, n), true, homed,
                                           [&out](const MechanismLine& m) { out.emplace_back(m.text); });
        return out;
    };
    const std::vector<std::string> arm = state(x, "arm", true);
    REQUIRE(arm.size() == 2U);
    CHECK(arm[0] == "arm: homed at boot");
    CHECK(arm[1].starts_with("arm: starting values kP 0.250"));
    CHECK(arm[1].ends_with("-- PROVISIONAL (HA-182); .tuning overrides"));
    const std::vector<std::string> lift = state(x, "lift", false);
    REQUIRE(lift.size() == 2U);
    CHECK(lift[0] == "lift: NOT HOMED -- its buttons do nothing until it is homed (homeAtDriverStart, homeOnPress, "
                     "or an auton's r.home)");
    CHECK(lift[1].starts_with("lift: starting values"));
    const std::vector<std::string> flywheel = state(x, "flywheel", false);
    REQUIRE(flywheel.size() == 1U);  // a speed mechanism has no homed state
    CHECK(flywheel[0].starts_with("flywheel: starting values kV"));
    CHECK(state(x, "intake", false)
          == std::vector<std::string>{"'intake': no stall signature -- held against its stop it is cut only by the "
                                      "driver letting go (HA-154)"});
    CHECK(state(x, "hood", false).empty());  // a pneumatic: no state line
    CHECK(state(ry::kRobotYMechanisms, "catapult", false).empty());  // a Power WITH a stall signature
    // An ABSENT position mechanism says nothing about homing (its role line said ABSENT).
    std::vector<std::string> absent;
    shulib::bench::mechanismStateLines(x, indexIn(x, "lift"), false, false,
                                       [&absent](const MechanismLine& m) { absent.emplace_back(m.text); });
    REQUIRE(absent.size() == 1U);
    CHECK(absent[0].starts_with("lift: starting values"));
}

TEST_CASE("MECH-A5 boot lines: one line per DRIVER CONTROL, every control word — a hold, three presets, a "
          "driver-start home, a flywheel toggle, autonOnly (robot x's controls); a homing press and "
          "pneumatic presets (its alternative); each control on its OWN mechanism") {
    // BUG CAUGHT: a control line naming another mechanism's buttons (a control paired by index), a
    // preset's name or time limit wrong, a pneumatic preset printed with a time limit it does not
    // take, a toggle on a flywheel printed as a pneumatic's, autonOnly printed as a control.
    const MechanismTable& x = rx::kRobotXMechanisms;
    const auto linesFor = [&x](const shulib::mechanism::DriverControls& controls) {
        std::vector<std::string> out;
        for (const shulib::mechanism::Control& c : controls.all()) {
            const int i = x.indexOf(c.mechanism);
            REQUIRE(i >= 0);
            out.emplace_back(shulib::bench::mechanismControlLine(x, static_cast<std::size_t>(i), c, true).text);
        }
        return out;
    };
    CHECK(linesFor(rx::kControls)
          == std::vector<std::string>{"arm: hold-to-run R1 / R2, 6 V",
                                      "arm: preset 'stow' on DOWN (1.50 s)",
                                      "arm: preset 'score' on RIGHT (1.50 s)",
                                      "arm: preset 'high' on UP (1.50 s)",
                                      "lift: hold-to-run L1 / L2, 12 V",
                                      "lift: homed when driver control starts, if nothing has homed it (1.50 s)",
                                      "flywheel: toggle on X (spin to 'shoot' / stop)",
                                      "intake: hold-to-run A / B, 12 V",
                                      "hood: no driver control (autonOnly: the autons' alone)"});
    CHECK(linesFor(rx::kAltControls)
          == std::vector<std::string>{"arm: hold-to-run R1 / R2, 6 V",
                                      "lift: hold-to-run L1 / L2, 12 V",
                                      "lift: home on LEFT (1.50 s)",
                                      "flywheel: toggle on X (spin to 'shoot' / stop)",
                                      "intake: hold-to-run A / B, 12 V",
                                      "hood: preset 'down' on DOWN",
                                      "hood: preset 'middle' on RIGHT",
                                      "hood: preset 'up' on UP"});
    // autonOnly is never ABSENT-marked (it commands nothing either way); a control on an ABSENT one is.
    const shulib::mechanism::Control& hoodOnly = rx::kControls.c[8];
    REQUIRE(hoodOnly.kind == shulib::mechanism::ControlKind::AutonOnly);
    CHECK(std::string{shulib::bench::mechanismControlLine(x, 4, hoodOnly, false).text}
          == "hood: no driver control (autonOnly: the autons' alone)");
    const MechanismLine absent = shulib::bench::mechanismControlLine(x, 0, rx::kControls.c[0], false);
    CHECK(std::string{absent.text} == "arm: hold-to-run R1 / R2, 6 V -- ABSENT: does nothing; the drive still runs");
    CHECK(absent.level == shulib::hal::LogLevel::Warn);
}

TEST_CASE("MECH-A5 the ROUTES line: how many mechanism route ids a robot's list published, the first and "
          "the last, and the registry's slots left — robot two's six, robot x's seventeen, none") {
    // BUG CAUGHT: a boot that does not say which ids a route may name (a route editor's marker naming
    // an id the robot does not have is only refused at RUN time, as an unknown id), a count that is
    // not the registered one, or the slots left for the team's own ids misstated.
    CHECK(std::string{shulib::bench::routesBootLine(shulib::mechanism::routeIdsOf(two::kRobotTwoMechanisms.specSpan())).text}
          == "ROUTES: 6 mechanism route ids published (claw_pivot_6_run .. claw_open); 26 of the registry's 32 left "
             "for the team's own");
    CHECK(std::string{shulib::bench::routesBootLine(shulib::mechanism::routeIdsOf(rx::kRobotXMechanisms.specSpan())).text}
          == "ROUTES: 17 mechanism route ids published (arm_stow .. hood_up); 15 of the registry's 32 left for the "
             "team's own");
    CHECK(std::string{shulib::bench::routesBootLine(shulib::mechanism::routeIdsOf(shulib::bench::kNoMechanisms.specSpan())).text}
          == "ROUTES: no mechanism route ids (this robot declares no mechanism); 32 of the registry's 32 left for "
             "the team's own");
    MechanismTable one = two::kRobotTwoMechanisms;
    one.count = 1;  // pivot 6 alone: its run and stop ids
    CHECK(std::string{shulib::bench::routesBootLine(shulib::mechanism::routeIdsOf(one.specSpan())).text}
          == "ROUTES: 2 mechanism route ids published (claw_pivot_6_run .. claw_pivot_6_stop); 30 of the registry's "
             "32 left for the team's own");
}

// ═══ The claw row ═════════════════════════════════════════════════════════════════════════
TEST_CASE("MECH-A5 the claw row: read from the SERVICE and the robot — the row Station J quotes, word for "
          "word, before driver control and while R1, L2 and A work; the 999 C clamp; the budget pinned to "
          "the widest row the code can print") {
    // BUG CAUGHT: the claw's word read from anywhere but its declaration's levels (a swapped .positions
    // line would then show OPEN for a closed claw), a pivot's word not what its motor was commanded, a
    // row that runs past the SMALL budget. The strings are the ones the row printed before MECH-A5.
    RobotTwoClaws t;
    CHECK(std::string{clawRowText(two::kRobotTwoMechanisms, *t.robot, *t.mechanisms, false).text}
          == "CLAW 6 HOLD 20C | 7 HOLD 20C | CLAW CLOSED");
    pros::shim::motorState(6).temperatureC = 47.6;
    pros::shim::motorState(7).temperatureC = 1234.0;  // clamped, never wider than the budget
    const auto rows = t.run(6, [](int k) {
        press(ControllerButton::R1, k >= 1);
        press(ControllerButton::L2, k >= 2);
        press(ControllerButton::A, k == 3);
    });
    CHECK(rows[1] == "CLAW 6 FWD 48C | 7 HOLD 999C | CLAW CLOSED");
    CHECK(rows[2] == "CLAW 6 FWD 48C | 7 REV 999C | CLAW CLOSED");
    CHECK(rows[3] == "CLAW 6 FWD 48C | 7 REV 999C | CLAW OPEN");
    CHECK(std::string{clawRowText(shulib::bench::kNoMechanisms, *t.robot, *t.mechanisms, false).text}
          == "claws: none declared for this robot");
    CHECK(shulib::bench::clawRowWorstLength(shulib::bench::kNoMechanisms)
          == std::string_view{"claws: none declared for this robot"}.size());
    // THE BUDGET GUARD, tied to a row the code prints: robot two's worst is pinned EXACTLY, and the
    // widest row it can show — both pivots at the clamp's 999 C, the claw CLOSED, a released claim —
    // is rendered by the code and has exactly that length, every character kept.
    CHECK(shulib::bench::clawRowWorstLength(two::kRobotTwoMechanisms) == 50U);
    {
        RobotTwoClaws worst;
        pros::shim::motorState(6).temperatureC = 5000.0;
        pros::shim::motorState(7).temperatureC = 1000.0;
        REQUIRE(worst.robot->mechanismAt(1)->tryClaim());  // released at entry: the row ends FREED
        const auto w = worst.run(1, [](int) {});
        REQUIRE(w.size() == 1U);
        CHECK(w[0] == "CLAW 6 HOLD 999C | 7 HOLD 999C | CLAW CLOSED FREED");
        CHECK(w[0].size() == shulib::bench::clawRowWorstLength(two::kRobotTwoMechanisms));
    }
    for (const char c : rows[3]) {
        CHECK(static_cast<unsigned char>(c) < 0x80U);
    }
}

TEST_CASE("MECH-A5 the claw row: a claim an autonomous operation left on pivot 6 and on the claw is "
          "RELEASED at driver-control entry — the row says FREED for THAT entry, each kind alone too, and "
          "not after the next") {
    // BUG CAUGHT: FREED read from the service's lifetime count (it would never go away), or from the
    // pivots only; a released claim the panel does not show.
    RobotTwoClaws t;
    REQUIRE(t.robot->mechanismAt(0)->tryClaim());
    REQUIRE(t.robot->pneumaticMechanismAt(2)->tryClaim());
    const auto rows = t.run(4, [](int) { press(ControllerButton::R1, true); });
    CHECK_FALSE(t.robot->mechanismAt(0)->claimed());
    CHECK_FALSE(t.robot->pneumaticMechanismAt(2)->claimed());
    CHECK(rows[0] == "CLAW 6 FWD 20C | 7 HOLD 20C | CLAW CLOSED FREED");
    const auto again = t.run(2, [](int) { press(ControllerButton::R1, false); });
    CHECK(again[0] == "CLAW 6 HOLD 20C | 7 HOLD 20C | CLAW CLOSED");
    REQUIRE(t.robot->mechanismAt(1)->tryClaim());
    const auto pivotOnly = t.run(1, [](int) {});
    CHECK(pivotOnly[0] == "CLAW 6 HOLD 20C | 7 HOLD 20C | CLAW CLOSED FREED");
    REQUIRE(t.robot->pneumaticMechanismAt(2)->tryClaim());
    const std::size_t writesBefore = pros::shim::adiLine(0, 1).writes.size();
    const auto clawOnly = t.run(1, [](int) {});
    CHECK(clawOnly[0] == "CLAW 6 HOLD 20C | 7 HOLD 20C | CLAW CLOSED FREED");
    CHECK(pros::shim::adiLine(0, 1).writes.size() == writesBefore);  // the release fired nothing at the claw
}

TEST_CASE("MECH-A5 the claw row and the boot lines for a list robot two's cannot imitate: a REVERSED "
          "Brake pivot on 9 and two pneumatics whose start and safe differ, one whose 'closed' is HIGH — "
          "each word read through ITS OWN declaration") {
    // BUG CAUGHT: a stopped mechanism showing robot two's HOLD instead of its own stop word; a
    // Reversed motor printed as a negative port (the row and the role line name the PHYSICAL port);
    // a pneumatic's word read from a fixed level instead of its own declaration's levels (the clamp is
    // HIGH and CLOSED, the wing HIGH and OPEN); a start read from the safe position.
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
    const BootOutcome boot =
        decideBootSourceFrom(record, census, DrivetrainDescription{}, "", kList, "test", [](const char*) {});
    REQUIRE_MESSAGE(boot.ok, boot.why);
    shulib::robot::Robot robot{boot.built.config, devices, boot.built.buildOptions()};
    MechanismService svc{pacer};
    svc.bind(kList, robot.mechanismParts().devices(), robot.context().clock(), robot.faults(),
             robot.context().telemetry());
    CHECK(robot.config().mechanisms[0].motorPorts[0] == -9);  // the config carries the sign...
    CHECK(std::string{clawRowText(kList, robot, svc, false).text} == "CLAW 9 BRAKE 0C | CLAMP CLOSED | WING OPEN");
    CHECK(devices.digitalOutInitialState(3));  // ...and each line was made at its START level (HIGH both)
    CHECK(devices.digitalOutInitialState(8));
    std::vector<std::string> lines;
    shulib::bench::mechanismBootLines(kList, robot, svc, nullptr,
                                      [&lines](const MechanismLine& m) { lines.emplace_back(m.text); });
    CHECK(lines == std::vector<std::string>{
                       "roller: POWER -- GREEN, motor 1 reversed; let go: BRAKE",
                       "roller: motor 1 -> port 9 ok",  // the PHYSICAL port
                       "'roller': no stall signature -- held against its stop it is cut only by the driver letting go "
                       "(HA-154)",
                       "clamp: PNEUMATIC -- starts 'closed', safe 'open'; closed = C high, open = C low (levels "
                       "UNVERIFIED until a bench station sees them); a disable leaves it where it is",
                       "clamp: line 1 -> 3-wire C not checkable (the brain cannot see a 3-wire device)",
                       "wing: PNEUMATIC -- starts 'open', safe 'closed'; closed = H low, open = H high (levels "
                       "UNVERIFIED until a bench station sees them); a disable leaves it where it is",
                       "wing: line 1 -> 3-wire H not checkable (the brain cannot see a 3-wire device)"});
    // The longest name a list can carry is printed whole.
    CHECK(std::string{shulib::bench::mechanismDeclarationLine(kLong, 0).text}
          == "claw pivot six, lef: POWER -- RED, motor 1 forward; let go: HOLD");
}

TEST_CASE("MECH-A5 a pivot declared REVERSED (Station J's fix for a claw that moves the wrong way) is still "
          "PORT 6 in every word — its declaration line says `motor 1 reversed`, its role line `port 6`, the "
          "row `6` — and the same R1 sends the motor the reversed command") {
    // BUG CAUGHT: a word that prints the SIGNED port (the boot line would read `port -6`, which is no
    // port on the brain); the flip not reaching the motor.
    MechanismTable flipped = two::kRobotTwoMechanisms;
    flipped.specs[0].motors[0].reversed = true;
    RobotTwoClaws t{true, true, flipped};
    const std::vector<std::string> lines = texts(t.lines());
    CHECK(lines[0] == "claw pivot 6: POWER -- RED, motor 1 reversed; let go: HOLD");
    CHECK(lines[1] == "claw pivot 6: motor 1 -> port 6 ok");
    CHECK(std::string{clawRowText(flipped, *t.robot, *t.mechanisms, false).text}
          == "CLAW 6 HOLD 20C | 7 HOLD 20C | CLAW CLOSED");
    std::int32_t mv = 0;
    const auto rows = t.run(3, [&mv](int k) {
        press(ControllerButton::R1, k >= 1);
        if (k == 2) mv = pros::shim::motorState(6).lastVoltageMv;  // what tick 1 commanded
    });
    // The library still commands +12 V ("forward" is the button's word, FWD on the row); the sign is in
    // the config's port and PROS applies it once, so the DEVICE turns the other way.
    CHECK(rows[1] == "CLAW 6 FWD 20C | 7 HOLD 20C | CLAW CLOSED");
    CHECK(mv == -12000);
    RobotTwoClaws absent{false, true, flipped};
    CHECK(std::string{clawRowText(flipped, *absent.robot, *absent.mechanisms, false).text}
          == "CLAW 6 ABSENT | 7 HOLD 20C | CLAW CLOSED");
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

