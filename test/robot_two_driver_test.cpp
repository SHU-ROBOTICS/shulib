// robot_two_driver_test.cpp — robot two's DRIVER FILE (src/team/robot_two_driver.hpp), end to end
// through the REAL src/ list, hal::pros::ProsDeviceProvider and the PROS shim (T4 Part 1a; moved onto
// the mechanism core by MECH-A5, 2026-10-29: the driver file is `controls(...)` + the feel + the loop,
// run by src/common/driver_file.hpp's DriverFile through the robot's MECHANISM SERVICE).
//
// ═══ WHAT THIS FILE PINS ════════════════════════════════════════════════════════════
// The driver file is what "shulib Drive" runs (src/programs/drive_program.cpp) and the competition
// program's opcontrol(); neither program TU compiles on the host. So each case builds the composition
// a program does: the boot decision over robot two's 2026-09-25 record and the shim's registry with
// the REAL robot_two::kRobotTwoMechanisms (or a copy with ONE field changed), robot::Robot over the
// REAL provider, ONE mechanism::MechanismService bound over the Robot's built mechanisms, the driver
// file bound to it, then RobotTwoDriver::run() — and reads the result at the bottom of the stack: each
// port's millivolts, brake() calls and brake mode in the shim, and every level 3-wire port A was ever
// driven to. The cases carry the T4 brief's names (T1-T10); T7 (the heading rule) is
// test/driver_heading_test.cpp's. Where the shim cannot show a thing — a log line goes to the
// provider's stdout — the same driver file runs over the fake provider, whose telemetry a test reads.
//
// ═══ THE CLAWS BEHAVE EXACTLY AS THEY DID (MECH-A5) ═════════════════════════════════
// "(M5a) the same script through T3/T4's bindings and through the driver file" runs ONE long input
// script — every button alone and in pairs, A's presses short and held, controller drops with buttons
// held and pressed DURING the drop, field disables mid-hold and re-entries — through BOTH paths on the
// same robot: T3's MechanismBinding / ToggleBinding, exactly as T4's DriverBindings bound them until
// MECH-A5 (the library keeps them, superseded — the MECH brief's N4), and robot two's driver file today.
// Every tick, ports 6 and 7 (millivolts, brake() calls, move calls, stopped, the brake mode) and line A
// (every write) must be IDENTICAL. Every T-case below then pins the same behaviour by its own numbers.
//
// NO HARDWARE CLAIM. The shim models our BELIEF about PROS: nothing here shows a claw moving, a
// pivot holding a load (HA-156), which way either turns (HA-153), or which level opens the claw
// (HA-159). Worksheet Stations J and K are where those are measured.

#include "doctest.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <initializer_list>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "common/boot_source.hpp"
#include "common/driver_heading.hpp"
#include "common/driver_panel.hpp"
#include "mechanism_driver_fixtures.hpp"
#include "pros/shim_control.hpp"
#include "robots/mechanism_table.hpp"
#include "shulib/core/check.hpp"
#include "shulib/hal/controller.hpp"
#include "shulib/hal/drivetrain_description.hpp"
#include "shulib/hal/fake/fake_battery.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_device_provider.hpp"
#include "shulib/hal/fake/fake_device_registry.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/hal/pros/controller.hpp"
#include "shulib/hal/pros/device_provider.hpp"
#include "shulib/hal/pros/device_registry.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/mechanism/service.hpp"
#include "shulib/robot/robot.hpp"
#include "shulib/teleop/driver_control.hpp"
#include "shulib/teleop/input_shaping.hpp"
#include "shulib/teleop/mechanism_binding.hpp"
#include "shulib/teleop/teleop_loop.hpp"
#include "shulib/teleop/coupled_side_monitor.hpp"
#include "shulib/teleop/drive_protection.hpp"
#include "shulib/teleop/drivetrain_degradation.hpp"
#include "shulib/units/quantity.hpp"
#include "source_pins.hpp"
#include "team/robot_two_driver.hpp"

using shulib::PreconditionError;
using shulib::bench::BootOutcome;
using shulib::bench::clawRowText;
using shulib::bench::decideBootSourceFrom;
using shulib::bench::MechanismTable;
using shulib::bench::RecordRead;
using shulib::bench::RobotTwoDriver;
using shulib::hal::ControllerButton;
using shulib::mechanism::MechanismService;
namespace two = shulib::bench::robot_two;

namespace {

// ── robot two, as a program will boot it ──────────────────────────────────────────────
/// Robot two's calibration record as the 2026-09-25 capture printed it (boot_source_test.cpp's
/// fixture, retyped: it lives in that file's anonymous namespace).
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
    std::snprintf(d.provenance, sizeof d.provenance, "%s", "robot two driver test (the 09-25 record)");
    return d;
}

/// The eight drive ports, SIGNED as the record types them (forward is sign * mV > 0).
constexpr int kSignedDrivePorts[8] = {-16, 17, -18, -20, 11, -12, 13, 15};

std::size_t shimButton(ControllerButton b) { return 6U + static_cast<std::size_t>(b); }
pros::shim::ControllerState& pad() { return pros::shim::controllerState(pros::E_CONTROLLER_MASTER); }
void press(ControllerButton b, bool down) { pad().digital[shimButton(b)] = down; }
void stick(int leftY, int rightX) {
    pad().analog[pros::E_CONTROLLER_ANALOG_LEFT_Y] = leftY;
    pad().analog[pros::E_CONTROLLER_ANALOG_RIGHT_X] = rightX;
}
/// The sticks every drive comparison shares: forward and turn, never centred for long.
void sticks(int k) {
    stick(static_cast<std::int32_t>(100.0 * std::sin(0.09 * k)),
          static_cast<std::int32_t>(60.0 * std::cos(0.05 * k)));
}
/// 3-wire line A's write history — every level it was ever driven to, construction included.
const std::vector<std::int32_t>& lineA() { return pros::shim::adiLine(0, 1).writes; }

/// What one tick left at the bottom of the stack, read in perTick (after drive() and the mechanisms).
struct Snap {
    std::array<std::int32_t, 22> mv{};
    std::array<int, 22> brakes{};
    std::array<int, 22> moves{};
    std::array<bool, 22> stopped{};
    std::array<pros::v5::MotorBrake, 22> mode{};
    std::size_t lineWrites = 0;
    std::string clawRow;
};
/// `svc` null: no claw row (the old bindings' path has no service to read it from).
Snap snap(const shulib::robot::Robot& robot, const MechanismService* svc, bool freed) {
    Snap s;
    for (int p = 1; p <= 21; ++p) {
        const auto& m = pros::shim::motorState(p);
        const auto i = static_cast<std::size_t>(p);
        s.mv[i] = m.lastVoltageMv;
        s.brakes[i] = m.brakeCalls;
        s.moves[i] = m.moveVoltageCalls;
        s.stopped[i] = m.control == pros::shim::MotorControl::Stopped;
        s.mode[i] = m.brake;
    }
    s.lineWrites = lineA().size();
    if (svc != nullptr) {
        s.clawRow = clawRowText(two::kRobotTwoMechanisms, robot, *svc, freed).text;
    }
    return s;
}

/// A program's composition for robot two, on the host: the boot decision (the REAL list unless a
/// case hands a copy with ONE field changed), the Robot over the REAL provider, the mechanism service
/// bound over the Robot's built mechanisms, the driver file bound to it. `sixPlugged` / `sevenPlugged`
/// say which claw pivots carry a motor; `extraMotor` (not 0) plugs one more motor port. `bindNow` false leaves
/// the service and the driver unbound (a case binds them, or runs the OLD bindings instead).
struct Rig {
    explicit Rig(bool sixPlugged = true, bool sevenPlugged = true,
                 const MechanismTable& table = two::kRobotTwoMechanisms, bool bindNow = true,
                 int extraMotor = 0)
        : list{&table} {
        pros::shim::resetAll();
        for (const int p : {11, 12, 13, 14, 15, 16, 17, 18, 20}) {  // the 09-25 census (19 empty)
            pros::shim::setPluggedAtPort(p, pros::c::E_DEVICE_MOTOR);
        }
        pros::shim::setPluggedAtPort(2, pros::c::E_DEVICE_IMU);
        if (sixPlugged) pros::shim::setPluggedAtPort(6, pros::c::E_DEVICE_MOTOR);
        if (sevenPlugged) pros::shim::setPluggedAtPort(7, pros::c::E_DEVICE_MOTOR);
        if (extraMotor != 0) pros::shim::setPluggedAtPort(extraMotor, pros::c::E_DEVICE_MOTOR);
        const shulib::hal::pros::ProsDeviceRegistry registry;
        RecordRead record{};
        record.description = robotTwoRecord();
        record.decoded = true;
        boot = decideBootSourceFrom(record, registry, shulib::hal::DrivetrainDescription{},
                                    "track width, external ratio (motor->wheel)", table,
                                    "2026 TANK CHASSIS (robot two)", [](const char*) {});
        REQUIRE_MESSAGE(boot.ok, boot.why);
        devices.emplace();
        const shulib::robot::BuildVerdict v =
            shulib::robot::validate(boot.built.config, *devices, boot.built.buildOptions());
        REQUIRE_MESSAGE(v.ok, v.why);
        robot.emplace(boot.built.config, *devices, boot.built.buildOptions());
        // The program's order ("shulib Drive"): the service around the raw tick, bound over the
        // Robot's own built mechanisms — then the driver file bound to it.
        mechanisms.emplace(devices->pacer());
        if (bindNow) {
            bindService();
            REQUIRE_MESSAGE(driver.bind(*mechanisms, why, sizeof why), why);  // the driver file's refusal, if any
        }
    }
    void bindService() {
        mechanisms->bind(*list, robot->mechanismParts().devices(), robot->context().clock(), robot->faults(),
                         robot->context().telemetry());
    }

    /// RobotTwoDriver::run() for `ticks` ticks; `script(k)` runs at the TOP of tick k (inside the
    /// disabled predicate, which the loop polls first).
    std::vector<Snap> run(int ticks, const std::function<void(int)>& script) {
        std::vector<Snap> log;
        int k = 0;
        driver.run(
            *robot, *mechanisms, devices->pacer(),
            [&] {
                if (k >= ticks) return true;
                script(k);
                return false;
            },
            [&](const shulib::math::ChassisSpeeds&, bool, const shulib::teleop::DriveShaperState&) {
                log.push_back(snap(*robot, &*mechanisms, driver.releasedLeftoverAtEntry(*mechanisms)));
                ++k;
            });
        return log;
    }

    const MechanismTable* list;
    BootOutcome boot;
    std::optional<shulib::hal::pros::ProsDeviceProvider> devices;
    std::optional<shulib::robot::Robot> robot;
    std::optional<MechanismService> mechanisms;  // after the Robot: destroyed first (it points into it)
    RobotTwoDriver driver;
    char why[192] = "";  // the driver file's bind() reason (empty when it bound)
};

// ── the same robot over the FAKE provider (its telemetry is readable; a line can be refused) ──
struct NoPace final : shulib::motion::ITickPacer {
    void pace() override {}
};
struct FakeRig {
    explicit FakeRig(int refusedLine = 0) {
        pros::shim::resetAll();  // the controller the driver file reads is still the shim's
        battery.setVoltage(shulib::units::Voltage{12.8});  // a fresh FakeBattery reads 0 V: a brownout
        if (refusedLine != 0) devices.refuseLine(static_cast<std::uint8_t>(refusedLine));
        RecordRead record{};
        record.description = robotTwoRecord();
        record.decoded = true;
        shulib::hal::fake::FakeDeviceRegistry census;
        for (const int p : {6, 7, 11, 12, 13, 14, 15, 16, 17, 18, 20}) {
            census.setKind(p, shulib::hal::DeviceKind::Motor);
        }
        census.setKind(2, shulib::hal::DeviceKind::Imu);
        boot = decideBootSourceFrom(record, census, shulib::hal::DrivetrainDescription{}, "",
                                    two::kRobotTwoMechanisms, "robot two", [](const char*) {});
        REQUIRE_MESSAGE(boot.ok, boot.why);
        robot.emplace(boot.built.config, devices, boot.built.buildOptions());
        mechanisms.emplace(pacer);
        mechanisms->bind(two::kRobotTwoMechanisms, robot->mechanismParts().devices(), robot->context().clock(),
                         robot->faults(), robot->context().telemetry());
    }
    template <class Script>
    void run(int ticks, Script script) {
        int k = 0;
        driver.run(
            *robot, *mechanisms, pacer,
            [&] {
                if (k >= ticks) return true;
                script(k);
                return false;
            },
            [&](const shulib::math::ChassisSpeeds&, bool, const shulib::teleop::DriveShaperState&) {
                ++k;
            });
    }
    double volts(int port) { return devices.fakeMotor(static_cast<std::uint8_t>(port)).commandedVoltage().value(); }

    shulib::hal::fake::FakeClock clock;
    shulib::hal::fake::FakeBattery battery;
    shulib::hal::fake::FakeTelemetrySink sink;
    NoPace pacer;
    shulib::hal::fake::FakeDeviceProvider devices{clock, battery, sink, pacer};
    BootOutcome boot;
    std::optional<shulib::robot::Robot> robot;
    std::optional<MechanismService> mechanisms;
    RobotTwoDriver driver;
};

/// A copy of robot two's REAL list (a hand-built list: the build's checks ran on the original only).
MechanismTable robotTwoCopy() { return two::kRobotTwoMechanisms; }

}  // namespace

// ═══ M5a — the claws exactly as T3/T4's bindings drove them ═════════════════════════════════
// The script (file banner): R1, R2, L1, L2 alone; R1+R2 and L1+L2 together; a release of one of two;
// A short, A held; R1 held through a drop and back; A pressed DURING a drop and still down at the
// reconnect, then released and pressed again; R1 + L1 held into a field DISABLE, then a second entry
// with R1 still held (it drives at once — a hold is a level); a third entry with A down at entry (not
// a press); sticks moving throughout.
namespace {
/// One tick's controller, by tick number within one entry's run (`entry` 0, 1, 2).
void claws(int entry, int k) {
    sticks(k + 7 * entry);
    const auto in = [k](int a, int b) { return k >= a && k < b; };
    pad().connected = true;
    if (entry == 0) {
        press(ControllerButton::R1, in(5, 15) || in(40, 50) || in(90, 120));
        press(ControllerButton::R2, in(20, 28) || in(44, 47));        // R1+R2 together at 44..46
        press(ControllerButton::L1, in(30, 38) || in(60, 70));
        press(ControllerButton::L2, in(64, 75));                       // L1+L2 together at 64..69
        press(ControllerButton::A, in(52, 54) || in(78, 88) || in(101, 112) || in(118, 121));
        pad().connected = !in(95, 110);  // a drop with R1 held; A pressed during it (101..)
    } else if (entry == 1) {
        press(ControllerButton::R1, k < 12);  // still held from the disabled entry: drives at once
        press(ControllerButton::L1, in(4, 9));
        press(ControllerButton::A, in(15, 17));
        press(ControllerButton::R2, false);
        press(ControllerButton::L2, false);
    } else {
        press(ControllerButton::A, k < 6 || in(10, 12));  // down AT entry: not a press; then one press
        press(ControllerButton::R1, false);
        press(ControllerButton::L1, in(2, 4));
    }
}
/// Ports 6 and 7 and line A, as one comparable record.
struct ClawTick {
    std::int32_t mv6, mv7;
    int brakes6, brakes7, moves6, moves7;
    bool stopped6, stopped7;
    pros::v5::MotorBrake mode6, mode7;
    std::size_t lineWrites;
    bool operator==(const ClawTick&) const = default;
};
ClawTick clawTick(const Snap& s) {
    return ClawTick{s.mv[6], s.mv[7], s.brakes[6], s.brakes[7], s.moves[6], s.moves[7], s.stopped[6], s.stopped[7],
                    s.mode[6], s.mode[7], s.lineWrites};
}
}  // namespace

TEST_CASE("MECH-A5 (M5a): robot two's claws behave EXACTLY as T3/T4's bindings drove them -- the same "
          "script through the old bindings and through the driver file, tick by tick, ports 6 and 7 and "
          "line A identical, across three entries, a drop and a disable") {
    // BUG CAUGHT: ANY change in what a claw does on any input when robot two moved onto the mechanism
    // service — a hold-to-run at another voltage or a tick late, a release that brakes twice or not at
    // all, both buttons driving, a drop that does not release (or re-sends the stop), a toggle that
    // flips on a held A, on a drop, on a reconnect with A down, or on a press at entry; a disable that
    // moves the claw; a later entry that re-sends the start level; a claw row reading another state.
    constexpr int kTicks[3] = {125, 20, 14};
    // THE OLD PATH: T3's bindings over the same robot, exactly as T4's DriverBindings bound them (by
    // NAME — config index 0, 1, 2 — at the driver file's old buttons and volts; the toggle's initial
    // level the config's START level), run by the T3 overload of the library's loop.
    std::vector<ClawTick> before;
    std::vector<std::int32_t> lineBefore;
    {
        Rig r{true, true, two::kRobotTwoMechanisms, /*bindNow=*/false};
        shulib::teleop::MechanismBinding six{*r.robot->mechanismAt(0), ControllerButton::R1, ControllerButton::R2,
                                             shulib::units::Voltage{12.0}};
        shulib::teleop::MechanismBinding seven{*r.robot->mechanismAt(1), ControllerButton::L1, ControllerButton::L2,
                                               shulib::units::Voltage{12.0}};
        shulib::teleop::ToggleBinding claw{*r.robot->pneumaticMechanismAt(2), ControllerButton::A,
                                           shulib::config::toLineValue(r.robot->config().mechanisms[2].pneumaticStart)};
        shulib::teleop::MechanismBinding* holds[] = {&six, &seven};
        shulib::teleop::ToggleBinding* toggles[] = {&claw};
        shulib::hal::pros::ProsController master{shulib::hal::pros::ControllerId::Master};
        shulib::teleop::DriveShaper shaper{RobotTwoDriver::kFeel};
        for (int entry = 0; entry < 3; ++entry) {
            int k = 0;
            shulib::teleop::runDriverControl(
                r.robot->chassis(), master, r.devices->pacer(), shaper, RobotTwoDriver::kLoop,
                std::span<shulib::teleop::MechanismBinding* const>{holds},
                std::span<shulib::teleop::ToggleBinding* const>{toggles},
                [&] {
                    if (k >= kTicks[entry]) return true;
                    claws(entry, k);
                    return false;
                },
                [&] { return shulib::bench::driverHeading(*r.robot); },
                [&](const shulib::math::ChassisSpeeds&, bool, const shulib::teleop::DriveShaperState&) {
                    before.push_back(clawTick(snap(*r.robot, nullptr, false)));
                    ++k;
                });
        }
        lineBefore = lineA();
    }
    // THE NEW PATH: robot two's driver file, through the mechanism service.
    std::vector<ClawTick> after;
    std::vector<std::int32_t> lineAfter;
    {
        Rig r;
        for (int entry = 0; entry < 3; ++entry) {
            for (const Snap& s : r.run(kTicks[entry], [entry](int k) { claws(entry, k); })) {
                after.push_back(clawTick(s));
            }
        }
        lineAfter = lineA();
    }
    REQUIRE(before.size() == 159U);
    REQUIRE(after.size() == before.size());
    int differing = 0;
    for (std::size_t i = 0; i < before.size(); ++i) {
        CAPTURE(i);
        if (!(after[i] == before[i])) {
            ++differing;
            CHECK(after[i].mv6 == before[i].mv6);
            CHECK(after[i].mv7 == before[i].mv7);
            CHECK(after[i].brakes6 == before[i].brakes6);
            CHECK(after[i].brakes7 == before[i].brakes7);
            CHECK(after[i].moves6 == before[i].moves6);
            CHECK(after[i].moves7 == before[i].moves7);
            CHECK(after[i].stopped6 == before[i].stopped6);
            CHECK(after[i].stopped7 == before[i].stopped7);
            CHECK(after[i].mode6 == before[i].mode6);
            CHECK(after[i].mode7 == before[i].mode7);
            CHECK(after[i].lineWrites == before[i].lineWrites);
        }
    }
    CHECK(differing == 0);
    CHECK(lineAfter == lineBefore);  // every level line A was ever driven to, construction included
    // ...and the script really exercised what it claims (two identical idle runs would prove nothing):
    // each pivot driven both ways, released, both-held, a toggle that flipped, a drop, a disable.
    int six12 = 0;
    int sixMinus = 0;
    int seven12 = 0;
    int sevenMinus = 0;
    for (const ClawTick& t : after) {
        six12 += t.mv6 == 12000 ? 1 : 0;
        sixMinus += t.mv6 == -12000 ? 1 : 0;
        seven12 += t.mv7 == 12000 ? 1 : 0;
        sevenMinus += t.mv7 == -12000 ? 1 : 0;
    }
    CHECK(six12 > 20);
    CHECK(sixMinus > 5);
    CHECK(seven12 > 10);
    CHECK(sevenMinus > 3);
    CHECK(after.back().brakes6 > 6);
    CHECK(lineAfter.size() >= 5U);  // built, the first entry's start, and flips
}

// ═══ T1 — the right ports ═══════════════════════════════════════════════════════════════
TEST_CASE("T4 1a T1: through the driver file, R1 alone drives port 6 at +12000 mV in the tick it is "
          "read and leaves 7 alone, R2 -12000, L1 port 7 +12000, L2 -12000; each release is ONE "
          "brake() under HOLD; the drive ports get exactly what they get with no button; the service's "
          "slot for each pivot moves ITS port") {
    // BUG CAUGHT: R1/R2 swapped in the driver file; the two pivots' declarations swapped in its
    // controls (R1 would move port 7); a control bound to the wrong slot; a release re-sent every
    // tick; a control that reaches a drive port, or a drive that notices a claw button; a slot found
    // by position instead of by name.
    std::vector<Snap> with;
    {
        Rig r;
        REQUIRE(r.driver.bound());
        with = r.run(90, [](int k) {
            sticks(k);
            press(ControllerButton::R1, k >= 10 && k < 20);
            press(ControllerButton::R2, k >= 30 && k < 40);
            press(ControllerButton::L1, k >= 50 && k < 60);
            press(ControllerButton::L2, k >= 70 && k < 80);
        });
    }
    REQUIRE(with.size() == 90U);
    CHECK(with[0].brakes[6] == 1);  // loop entry: each pivot made safe once, before any press
    CHECK(with[0].brakes[7] == 1);
    for (std::size_t k = 10; k < 20; ++k) {
        CAPTURE(k);
        CHECK(with[k].mv[6] == 12000);  // R1: port 6 forward, full power, in the tick it is read
        CHECK_FALSE(with[k].stopped[6]);
        CHECK(with[k].moves[7] == with[9].moves[7]);  // port 7 untouched
        CHECK(with[k].brakes[7] == 1);
    }
    CHECK(with[20].stopped[6]);  // the release: ONE brake() under HOLD, in the release tick
    CHECK(with[20].brakes[6] == 2);
    for (std::size_t k = 20; k < 30; ++k) {
        CAPTURE(k);
        CHECK(with[k].brakes[6] == 2);
        CHECK(with[k].moves[6] == with[20].moves[6]);
    }
    CHECK(with[30].mv[6] == -12000);  // R2: back
    CHECK(with[39].mv[6] == -12000);
    CHECK(with[40].brakes[6] == 3);
    CHECK(with[50].mv[7] == 12000);  // L1: port 7 forward; port 6 left where it was
    CHECK(with[59].mv[7] == 12000);
    CHECK(with[59].moves[6] == with[40].moves[6]);
    CHECK(with[60].brakes[7] == 2);
    CHECK(with[70].mv[7] == -12000);  // L2: back
    CHECK(with[80].brakes[7] == 3);
    CHECK(pros::shim::motorState(6).stoppedUnder == pros::v5::MotorBrake::hold);
    CHECK(pros::shim::motorState(7).stoppedUnder == pros::v5::MotorBrake::hold);

    std::vector<Snap> without;
    {
        Rig r;
        without = r.run(90, [](int k) { sticks(k); });
    }
    REQUIRE(without.size() == 90U);
    int moving = 0;
    for (std::size_t k = 0; k < 90; ++k) {
        for (const int sp : kSignedDrivePorts) {
            const auto p = static_cast<std::size_t>(std::abs(sp));
            CAPTURE(k);
            CAPTURE(p);
            CHECK(with[k].mv[p] == without[k].mv[p]);
        }
        moving += std::abs(with[k].mv[16]) > 1000 ? 1 : 0;
        CHECK(without[k].mv[6] == 0);  // no button: no pivot ever moved
        CHECK(without[k].mv[7] == 0);
    }
    CHECK(moving > 30);  // the drive really moved — two idle runs agreeing would prove nothing

    // The service's slot BY NAME — pushed through ITS mechanism, ITS port moves.
    Rig r;
    const shulib::mechanism::Slot* six = r.mechanisms->slotFor("claw pivot 6");
    const shulib::mechanism::Slot* seven = r.mechanisms->slotFor("claw pivot 7");
    REQUIRE(six != nullptr);
    REQUIRE(seven != nullptr);
    CHECK(six->index() == 0);
    CHECK(seven->index() == 1);
    r.mechanisms->power(six->index()).drive(shulib::units::Voltage{5.0});
    r.mechanisms->tickFromDriverLoop();
    CHECK(pros::shim::motorState(6).lastVoltageMv == 5000);
    CHECK(pros::shim::motorState(7).lastVoltageMv == 0);
    r.mechanisms->power(seven->index()).drive(shulib::units::Voltage{-4.0});
    r.mechanisms->tickFromDriverLoop();
    CHECK(pros::shim::motorState(7).lastVoltageMv == -4000);
    CHECK(pros::shim::motorState(6).lastVoltageMv == 5000);
    CHECK(r.mechanisms->slotFor("intake") == nullptr);  // not declared
    CHECK(r.mechanisms->slotFor(nullptr) == nullptr);
}

// ═══ T2 — the claw ═══════════════════════════════════════════════════════════════════════
TEST_CASE("T4 1a T2: the claw through the driver file — line A's FIRST level ever is the start "
          "level and the first entry sends the same; each press flips it ONCE; a disable and a "
          "second run() write nothing; the service's 'claw' slot is the one on line A") {
    // BUG CAUGHT: the claw's start level inverted; a flip on a held A; a disable, or the next entry,
    // that commands the claw; the toggle never handed to the loop; the slot found by position.
    Rig r;
    const std::int32_t s = 0;  // CLOSED = LOW (robot two's .positions line; HA-159's guess)
    const std::int32_t o = 1;
    REQUIRE(lineA() == std::vector<std::int32_t>{s});  // built at the start level, nothing before
    const auto log = r.run(40, [](int k) {
        press(ControllerButton::A, (k >= 5 && k < 9) || (k >= 15 && k < 17));
    });
    REQUIRE(log.size() == 40U);
    CHECK(log[0].lineWrites == 2U);  // the first entry sends the start level…
    CHECK(lineA()[1] == s);          // …the same level: the valve does not move
    CHECK(log[4].lineWrites == 2U);
    CHECK(log[5].lineWrites == 3U);  // the press: OPEN
    CHECK(lineA()[2] == o);
    CHECK(log[8].lineWrites == 3U);  // held: nothing more
    CHECK(log[15].lineWrites == 4U);  // the next press: CLOSED
    CHECK(lineA()[3] == s);
    CHECK(log[39].lineWrites == 4U);
    CHECK(lineA().size() == 4U);  // the disable that ended the run commanded nothing
    r.run(3, [](int) {});
    CHECK(lineA().size() == 4U);  // and the next entry sends no start level
    // The 'claw' slot IS line A's: a step through it reaches line A.
    const shulib::mechanism::Slot* claw = r.mechanisms->slotFor("claw");
    REQUIRE(claw != nullptr);
    CHECK(claw->kind() == shulib::mechanism::Kind::Pneumatic);
    r.mechanisms->pneumatic(claw->index()).set(1);  // 'open'
    CHECK(lineA().size() == 5U);
    CHECK(lineA().back() == o);
    // The pivots never moved on A.
    CHECK(log.back().moves[6] == log[0].moves[6]);
    CHECK(log.back().moves[7] == log[0].moves[7]);
}

TEST_CASE("T4 1a T2c: the claw starts at the robot's START position, not its SAFE one — a list copy "
          "whose claw starts 'open' and is safe 'closed': line A's first level and the first entry's "
          "are OPEN, and the first press closes it") {
    // BUG CAUGHT: the start level taken from the SAFE position (robot two's real claw has start ==
    // safe == 'closed', so T2, T2b and T3 cannot tell the two apart).
    MechanismTable startsOpen = robotTwoCopy();
    startsOpen.specs[2].pneumatic.start = 1;  // 'open'
    startsOpen.specs[2].pneumatic.safe = 0;   // 'closed'
    Rig r{true, true, startsOpen};
    const std::int32_t open = 1;
    REQUIRE(lineA() == std::vector<std::int32_t>{open});
    const auto log = r.run(10, [](int k) { press(ControllerButton::A, k >= 4 && k < 6); });
    CHECK(log[0].lineWrites == 2U);
    CHECK(lineA()[1] == open);  // the entry: OPEN, the start — never the safe CLOSED
    CHECK(log[3].lineWrites == 2U);
    CHECK(log[4].lineWrites == 3U);
    CHECK(lineA()[2] == 1 - open);  // the first press CLOSES it
}

TEST_CASE("T4 1a T2b: the team lead's ruling — \"starts closed — whatever autonomous left it at\": "
          "an auton that opens the claw is overruled ONCE by the first entry; and a line set OPEN "
          "before the service is bound still starts CLOSED at that entry") {
    // BUG CAUGHT: the start read from what the line is commanded to NOW (the call-order hazard: a
    // bind after an auton would keep the claw open), or the first entry not sending it; a second
    // entry re-sending it (the claw would move on every re-enable).
    const std::int32_t s = 0;  // CLOSED = LOW
    {
        Rig r;
        r.robot->pneumaticMechanism("claw")->set(true);  // an auton opens it
        REQUIRE(lineA().size() == 2U);
        REQUIRE(lineA().back() == 1 - s);
        r.run(3, [](int) {});
        CHECK(lineA().size() == 3U);  // exactly ONE write at the first entry…
        CHECK(lineA().back() == s);   // …CLOSED
        r.run(3, [](int) {});         // a disable, then driver control again
        CHECK(lineA().size() == 3U);  // nothing
    }
    {
        Rig r{true, true, two::kRobotTwoMechanisms, /*bindNow=*/false};
        r.robot->pneumaticMechanism("claw")->set(true);  // OPEN before the service exists
        r.bindService();
        char why[192] = "";
        REQUIRE_MESSAGE(r.driver.bind(*r.mechanisms, why, sizeof why), why);
        r.run(2, [](int) {});
        CHECK(lineA().size() == 3U);
        CHECK(lineA().back() == s);  // the DECLARATION's start, not the line's level now
    }
}

TEST_CASE("T4 1a T3: the claw follows the ROBOT FILE, not a restated CLOSED — a list copy with Low and "
          "High swapped (Station J.9's fix): the first level and the entry level are both the swapped "
          "'closed', and the first press still flips it once") {
    // BUG CAUGHT: a driver file or a service that hard-codes CLOSED as a line level (false) — right
    // for today's guess, wrong the day J.9 swaps the levels.
    MechanismTable swapped = robotTwoCopy();
    swapped.specs[2].pneumatic.positions[0].levels[0] = true;   // closed = HIGH
    swapped.specs[2].pneumatic.positions[1].levels[0] = false;  // open = LOW
    Rig r{true, true, swapped};
    const std::int32_t closed = 1;  // the swapped CLOSED
    REQUIRE(lineA() == std::vector<std::int32_t>{closed});
    const auto log = r.run(8, [](int k) { press(ControllerButton::A, k >= 3 && k < 5); });
    CHECK(log[0].lineWrites == 2U);
    CHECK(lineA()[1] == closed);
    CHECK(log[3].lineWrites == 3U);
    CHECK(lineA()[2] == 1 - closed);
    CHECK(log[7].lineWrites == 3U);
}

// ═══ T4 / T4b — ABSENT mechanisms ═══════════════════════════════════════════════════════
TEST_CASE("T4 1a T4: a pivot whose port is EMPTY is ABSENT — the driver file binds, its buttons "
          "command nothing anywhere, the other pivot works, and the drive runs; for port 7 and for "
          "port 6") {
    // BUG CAUGHT: an absent claw refusing the whole driver (the drive would not run); a control over a
    // missing motor (a crash).
    {
        Rig r{true, false};
        CHECK_FALSE(r.mechanisms->slotFor("claw pivot 7")->built());
        REQUIRE(r.mechanisms->slotFor("claw pivot 6")->built());
        const auto log = r.run(12, [](int k) {
            sticks(k + 5);
            press(ControllerButton::L1, k < 6);
            press(ControllerButton::L2, k >= 3 && k < 8);
            press(ControllerButton::R1, k >= 8);
        });
        CHECK(pros::shim::motorState(7).moveVoltageCalls == 0);  // never even opened
        for (std::size_t k = 0; k < 8; ++k) {
            CAPTURE(k);
            CHECK(log[k].moves[6] == log[0].moves[6]);  // L1/L2 reached no other motor either
        }
        CHECK(log[9].mv[6] == 12000);
        CHECK(std::abs(log[9].mv[16]) > 0);
        CHECK(log[9].clawRow == "CLAW 6 FWD 20C | 7 ABSENT | CLAW CLOSED");
    }
    {
        Rig r{false, true};
        CHECK_FALSE(r.mechanisms->slotFor("claw pivot 6")->built());
        const auto log = r.run(8, [](int k) {
            sticks(k + 5);
            press(ControllerButton::R1, k < 4);
            press(ControllerButton::L1, k >= 4);
        });
        CHECK(pros::shim::motorState(6).moveVoltageCalls == 0);
        CHECK(log[5].mv[7] == 12000);
        CHECK(std::abs(log[5].mv[16]) > 0);
    }
}

TEST_CASE("T4 1a T4b: a claw whose 3-wire line the provider refused is ABSENT — the driver file binds, "
          "A commands nothing, the pivots and the drive run (the fake provider: the brain's refuses only "
          "a line outside 1..8, which the config refuses first)") {
    // BUG CAUGHT (T3 test 1f's intent): a toggle over a missing line (a crash), or an absent claw
    // refusing the driver.
    FakeRig f{1};
    char why[192] = "";
    REQUIRE_MESSAGE(f.driver.bind(*f.mechanisms, why, sizeof why), why);
    CHECK_FALSE(f.mechanisms->slotFor("claw")->built());
    double pivot = 0.0;
    double drive = 0.0;
    f.run(10, [&f, &pivot, &drive](int k) {
        stick(90, 0);
        press(ControllerButton::A, k >= 2 && k < 4);
        press(ControllerButton::R1, k >= 5);
        if (k == 9) {  // what tick 8 commanded (the disable that ends the run safes them all)
            pivot = f.volts(6);
            drive = f.volts(16);
        }
    });
    CHECK(f.devices.fakeDigitalOut(1).setCount() == 0);  // A reached no line
    CHECK(pivot == 12.0);                                 // the pivots work
    CHECK(std::abs(drive) > 1.0);                         // and the drive runs
}

// ═══ T5 / T6 — the feel and the release ═════════════════════════════════════════════════
TEST_CASE("T4 1a T5: the feel is the library's default, field-centric OFF — the drive ports get "
          "exactly what a direct runDriverControl with DriverProfile{} gives on the same inputs, "
          "and with the heading TRUSTED and turned 90 degrees a forward stick still drives both "
          "sides forward") {
    // BUG CAUGHT: a feel that is not the library's default (a curve, a slew, another deadband);
    // field-centric ON — which only shows once the heading is trusted AND has turned, so this case
    // makes it both (a field-centric robot turned 90 degrees would take "forward" as sideways, which a
    // tank cannot do).
    // Y (the re-zero) is pressed once the heading is trusted: field-centric LATCHES robot-centric
    // when its first sample is untrusted (tick 0 comes before the estimate is live) and only a
    // re-zero clears that — so without the press a field-centric mutation would hide in the latch.
    // In ArcadeBody a press only pre-arms the zero, and the direct run presses it too.
    const auto script = [](int k) {
        stick(k < 30 ? 90 : 60, k >= 45 ? 40 : 0);
        press(ControllerButton::Y, k >= 5 && k < 7);
        if (k == 20) pros::shim::imuState(2).rotationDegCw = 90.0;  // the robot "turned"
    };
    std::vector<Snap> viaDriver;
    shulib::teleop::HeadingSample seen{};
    {
        Rig r;
        pros::shim::imuState(2).calibrating = false;  // the IMU finished calibrating
        viaDriver = r.run(60, script);
        seen = shulib::bench::driverHeading(*r.robot);
    }
    CHECK(seen.trusted);  // the heading WAS trusted — so field-centric would have used it
    CHECK(std::abs(std::abs(seen.radians) - shulib::math::Angle::kPi / 2.0) < 0.05);
    std::vector<Snap> direct;
    {
        Rig r;
        pros::shim::imuState(2).calibrating = false;
        shulib::hal::pros::ProsController master{shulib::hal::pros::ControllerId::Master};
        shulib::teleop::DriveShaper shaper{shulib::teleop::DriverProfile{}};
        const shulib::teleop::DriverLoopConfig cfg{
            .tickPeriod = shulib::units::Time{0.01}, .rezeroButton = ControllerButton::Y};
        int k = 0;
        shulib::teleop::runDriverControl(
            r.robot->chassis(), master, r.devices->pacer(), shaper, cfg,
            [&] {
                if (k >= 60) return true;
                script(k);
                return false;
            },
            [&] { return shulib::bench::driverHeading(*r.robot); },
            [&](const shulib::math::ChassisSpeeds&, bool, const shulib::teleop::DriveShaperState&) {
                direct.push_back(snap(*r.robot, nullptr, false));
                ++k;
            });
    }
    REQUIRE(viaDriver.size() == 60U);
    REQUIRE(direct.size() == 60U);
    for (std::size_t k = 0; k < 60; ++k) {
        for (const int sp : kSignedDrivePorts) {
            const auto p = static_cast<std::size_t>(std::abs(sp));
            CAPTURE(k);
            CAPTURE(p);
            CHECK(viaDriver[k].mv[p] == direct[k].mv[p]);
        }
    }
    // Forward (the stick straight up) drives EVERY drive motor forward — before and after the
    // heading turned: sign(port) * mV > 0 on all eight.
    for (const std::size_t k : {std::size_t{10}, std::size_t{25}, std::size_t{40}}) {
        for (const int sp : kSignedDrivePorts) {
            const auto p = static_cast<std::size_t>(std::abs(sp));
            CAPTURE(k);
            CAPTURE(sp);
            CHECK((sp > 0 ? 1 : -1) * viaDriver[k].mv[p] > 1000);
        }
    }
}

TEST_CASE("T4 1a T6: sticks let go => every drive motor COASTS at 0 V, every tick; the field's "
          "disable => the library's cancel() BRAKES every drive motor") {
    // BUG CAUGHT: the driver file's releaseBrakeMode not Coast (a release would stop the drive
    // hard — not the team lead's ruling); a disable that leaves the drive coasting.
    Rig r;
    const auto log = r.run(30, [](int k) { stick(k < 10 ? 100 : 0, 0); });
    REQUIRE(log.size() == 30U);
    CHECK(std::abs(log[5].mv[16]) > 1000);
    for (std::size_t k = 12; k < 30; ++k) {  // let go at tick 10 (the deadband and the rest)
        for (const int sp : kSignedDrivePorts) {
            const auto p = static_cast<std::size_t>(std::abs(sp));
            CAPTURE(k);
            CAPTURE(p);
            CHECK(log[k].mv[p] == 0);
            CHECK_FALSE(log[k].stopped[p]);  // a 0 V command, never a brake()
            CHECK(log[k].mode[p] == pros::v5::MotorBrake::coast);
        }
    }
    for (const int sp : kSignedDrivePorts) {  // the run ended on the disable
        const auto& m = pros::shim::motorState(std::abs(sp));
        CAPTURE(sp);
        CHECK(m.control == pros::shim::MotorControl::Stopped);
        CHECK(m.stoppedUnder == pros::v5::MotorBrake::brake);
    }
}

// ═══ T8 — the driver binds ONCE ═════════════════════════════════════════════════════════
TEST_CASE("T4 1a T8 (MECH-A5): the driver file binds ONCE, at boot — a second bind is a precondition, "
          "and the first binding still drives; a bind before the service is bound is one too") {
    // BUG CAUGHT: a second bind that silently re-binds (two sets of button states over one service),
    // or one that leaves the first unusable; a program's composition mistake (a second bind, a
    // service not bound yet) reported as the DRIVER FILE's refusal — a team would go looking in its
    // controls for a program's bug. (Until MECH-A5 bind() could be called again and unbind() emptied
    // the bindings: the driver's inputs now belong to the service they were bound to, for the
    // program's life — DriverInputs' rule.)
    char why[192] = "";
    {
        Rig r;
        try {
            (void)r.driver.bind(*r.mechanisms, why, sizeof why);
            FAIL("a second bind was not a precondition");
        } catch (const PreconditionError& e) {
            CHECK(std::string_view{e.what()} == "DriverFile::bind: called twice -- bind once, at boot");
        }
        CHECK(r.driver.bound());
        const auto log = r.run(4, [](int) { press(ControllerButton::R1, true); });
        CHECK(log[1].mv[6] == 12000);
    }
    Rig unbound{true, true, two::kRobotTwoMechanisms, /*bindNow=*/false};
    RobotTwoDriver d;
    try {
        (void)d.bind(*unbound.mechanisms, why, sizeof why);
        FAIL("a bind before the service was bound was not a precondition");
    } catch (const PreconditionError& e) {
        CHECK(std::string_view{e.what()} ==
              "DriverFile::bind: the mechanism service is not bound -- bind it first");
    }
    CHECK_FALSE(d.bound());
}

// ═══ T9 — T3's rules, through the driver file ═══════════════════════════════════════════
TEST_CASE("T4 1a T9: through the driver file, a controller drop with R1 held is ONE Hold, and R1 "
          "held after the reconnect drives again") {
    // BUG CAUGHT: the driver file's controller not the one the loop reads, or a drop that re-sends
    // the stop every tick.
    Rig r;
    const auto log = r.run(16, [](int k) {
        press(ControllerButton::R1, true);
        pad().connected = !(k >= 5 && k < 10);
    });
    CHECK(log[4].mv[6] == 12000);
    CHECK(log[5].stopped[6]);  // the drop: a release
    CHECK(log[5].brakes[6] == 2);
    CHECK(log[9].brakes[6] == 2);  // once
    CHECK(log[10].mv[6] == 12000);  // reconnected, still held: it drives again
    CHECK_FALSE(log[10].stopped[6]);
}

TEST_CASE("MECH-A5 T9b: a controller DROP never flips the claw (MECH-A4's C1 fix, inherited) — A up "
          "through a drop; A pressed DURING a drop and still down at the reconnect; then released and "
          "pressed again: ONE flip, the last") {
    // BUG CAUGHT (MECH-A4's review blocker C1, through robot two's REAL driver file): a drop read as a
    // press (feeding "held" into an edge whose last level was up is itself a rising edge) — robot
    // two's claw would OPEN on every controller drop and drop what it holds; or a reconnect with A
    // still down taken as a press. It rests on HA-147 (the connected flag and the buttons come back
    // together); Station J.9.5 settles it, for this path too.
    Rig r;
    const auto log = r.run(70, [](int k) {
        pad().connected = !((k >= 10 && k < 20) || (k >= 30 && k < 40));
        press(ControllerButton::A, (k >= 35 && k < 50) || (k >= 55 && k < 58));
    });
    REQUIRE(log.size() == 70U);
    CHECK(log[49].lineWrites == 2U);  // built + the first entry: no flip through either drop
    CHECK(lineA().size() >= 3U);
    CHECK(log[55].lineWrites == 3U);  // released at 50, pressed at 55: the ONE flip
    CHECK(lineA()[2] == 1);           // OPEN
    CHECK(log[69].lineWrites == 3U);
}

TEST_CASE("MECH-A5 review: in driver control each A press is a claw STEP that logs its verdict — "
          "an opening press SUCCEEDED, a closing press UNCONFIRMED (no piece sensor; counted done) at "
          "WARN, a press within the move time CANCELLED (replaced), a disable within it CANCELLED "
          "(driver control ended) — and the pivots' holds log nothing") {
    // BUG CAUGHT: a change to what a driver's press writes to serial and the card's log that no test
    // sees. T3's ToggleBinding logged nothing per press; since MECH-A5 the claw's A goes through the
    // mechanism service (PneumaticSlot::set — a step, like an auton's), so the service says each
    // press's verdict once its move time has passed. These are the lines Station J.9.3/J.9.4 and
    // Station K now expect, and guide 18 / guide 11 describe (MECH-A5's review: M5a compares ports
    // and line writes only, so it could not see them). The clock moves 10 ms a tick, as the brain's.
    FakeRig f;
    char why[192] = "";
    REQUIRE_MESSAGE(f.driver.bind(*f.mechanisms, why, sizeof why), why);
    const int before = f.sink.size();
    f.run(230, [&f](int k) {
        f.clock.advance(shulib::units::Time{0.010});
        press(ControllerButton::A, k == 10 || k == 100 || k == 200 || k == 203);
        // R1 held for a while: a hold-to-run logs no verdict (it is not a step)
        press(ControllerButton::R1, k >= 20 && k < 60);
    });
    std::vector<std::string> mech;
    for (int i = before; i < f.sink.size(); ++i) {
        const auto& e = f.sink.at(i);
        if (e.subsystem == "MECH") {
            mech.push_back(std::string{e.level == shulib::hal::LogLevel::Warn ? "[WARN] " : "[INFO] "} + e.message);
        }
    }
    const std::vector<std::string> expected{
        "[INFO] 'claw': SUCCEEDED -- time in 0.50s",
        "[WARN] 'claw': UNCONFIRMED -- time in 0.50s (no piece sensor to confirm it; counted done)",
        "[WARN] 'claw': CANCELLED in 0.03s (replaced by a newer command)",
        "[WARN] 'claw': CANCELLED in 0.26s (driver control ended)",  // pressed at 2.04 s, disabled at 2.30 s
    };
    std::string all;
    for (const std::string& line : mech) {
        all += line + "\n";
    }
    INFO("the MECH lines:\n" << all);
    REQUIRE(mech.size() == expected.size());
    for (std::size_t i = 0; i < expected.size(); ++i) {
        CHECK(mech[i] == expected[i]);
    }
}

TEST_CASE("T4 1a T9: a claim left on pivot 6 before run() is RELEASED at entry with ONE [WARN][MECH] "
          "line, R1 drives from the first tick, and releasedLeftoverAtEntry() — what the claw row's "
          "FREED reads — is true after that entry and false after the next") {
    // BUG CAUGHT: the driver locked out of a pivot by a claim nobody will release (a deleted auton
    // task); a release the log or the screen does not show; FREED remembered past the entry it
    // belongs to (MECH-A5: the service's count is cumulative, the driver file compares it with the
    // count at THIS entry). The fake provider: the shim's log is on stdout, which no test reads.
    FakeRig f;
    char why[192] = "";
    REQUIRE_MESSAGE(f.driver.bind(*f.mechanisms, why, sizeof why), why);
    CHECK_FALSE(f.driver.releasedLeftoverAtEntry(*f.mechanisms));  // before any run
    REQUIRE(f.robot->mechanismAt(0)->tryClaim());
    const int before = f.sink.size();
    double firstTick = 0.0;
    f.run(3, [&f, &firstTick](int k) {
        press(ControllerButton::R1, true);
        if (k == 1) firstTick = f.volts(6);  // what tick 0 commanded
    });
    CHECK_FALSE(f.robot->mechanismAt(0)->claimed());
    CHECK(firstTick == 12.0);
    int released = 0;
    for (int i = before; i < f.sink.size(); ++i) {
        const auto& e = f.sink.at(i);
        if (e.message.find("RELEASED") != std::string::npos) {
            ++released;
            CHECK(e.level == shulib::hal::LogLevel::Warn);
            CHECK(e.subsystem == "MECH");
            CHECK(e.message
                  == "'claw pivot 6': a claim left by another owner RELEASED as driver control began (its "
                     "operation was not cancelled -- it may have lived on a deleted task)");
        }
    }
    CHECK(released == 1);
    CHECK(f.driver.releasedLeftoverAtEntry(*f.mechanisms));
    CHECK(std::string{clawRowText(two::kRobotTwoMechanisms, *f.robot, *f.mechanisms,
                                  f.driver.releasedLeftoverAtEntry(*f.mechanisms))
                          .text}
          == "CLAW 6 HOLD 0C | 7 HOLD 0C | CLAW CLOSED FREED");  // safed at the disable; 0 C fake
    const int afterFirst = f.sink.size();
    f.run(2, [](int) { press(ControllerButton::R1, false); });
    CHECK_FALSE(f.driver.releasedLeftoverAtEntry(*f.mechanisms));
    for (int i = afterFirst; i < f.sink.size(); ++i) {
        CHECK(f.sink.at(i).message.find("RELEASED") == std::string::npos);
    }
    CHECK(std::string{clawRowText(two::kRobotTwoMechanisms, *f.robot, *f.mechanisms,
                                  f.driver.releasedLeftoverAtEntry(*f.mechanisms))
                          .text}
          == "CLAW 6 HOLD 0C | 7 HOLD 0C | CLAW CLOSED");
    // The claw's claim alone says it too (a panel reading only the pivots would miss it).
    REQUIRE(f.robot->pneumaticMechanismAt(2)->tryClaim());
    f.run(1, [](int) {});
    CHECK(f.driver.releasedLeftoverAtEntry(*f.mechanisms));
}

// ═══ T10 — a robot these controls are not for ═══════════════════════════════════════════
TEST_CASE("T4 1a T10: the driver file REFUSES, with nothing bound, a robot its controls are not for — "
          "no 'claw pivot 7'; 'claw pivot 7' pneumatic; an extra 'intake' no control names; no claw; a "
          "THREE-position claw — each reason NAMING the mechanism, and binds robot two afterwards") {
    // BUG CAUGHT (T3 test 1i's intent): an undeclared name read as "absent" (pivot 7's buttons
    // silently dead on a robot built from another list; the claw's A silently dead on a robot with no
    // claw); a hold resolved against a pneumatic mechanism; a declared mechanism nobody can work (the
    // driver file's own coverage check — the build checks the controls against robot two's list, and
    // this checks them against the robot the SERVICE was bound to); a toggle bound to a claw with a
    // third position (the library's own check, DriverInputs::bind, reported in its own words); a
    // refusal that leaves a half-bound driver (a later bind would then be refused as "twice"); and —
    // MECH-A5's review — a refusal that THROWS (the program would show "the library graph REFUSED to
    // build", not its driver-file headline) or whose reason does not NAME the mechanism (T4's binder
    // named it, and Station F's table promises it). Each list is a hand-built copy of robot two's
    // (the build's checks ran on the original only).
    MechanismTable fewer = robotTwoCopy();  // pivot 6 and the claw: no pivot 7
    fewer.specs[1] = fewer.specs[2];
    fewer.wiring[1] = fewer.wiring[2];
    fewer.declared[1] = fewer.declared[2];
    fewer.count = 2;
    MechanismTable kinds = robotTwoCopy();  // 'claw pivot 7' a PNEUMATIC on line B
    kinds.specs[1] = kinds.specs[2];
    std::snprintf(kinds.specs[1].name, sizeof kinds.specs[1].name, "%s", "claw pivot 7");
    kinds.wiring[1] = kinds.wiring[2];
    std::snprintf(kinds.wiring[1].mechanism, sizeof kinds.wiring[1].mechanism, "%s", "claw pivot 7");
    kinds.wiring[1].lines[0] = shulib::mechanism::Line::B;
    MechanismTable withIntake = robotTwoCopy();  // a fourth mechanism no control names
    withIntake.specs[3] = withIntake.specs[0];
    std::snprintf(withIntake.specs[3].name, sizeof withIntake.specs[3].name, "%s", "intake");
    withIntake.wiring[3] = withIntake.wiring[0];
    std::snprintf(withIntake.wiring[3].mechanism, sizeof withIntake.wiring[3].mechanism, "%s", "intake");
    withIntake.wiring[3].motors[0] = 8;
    withIntake.count = 4;
    MechanismTable noClaw = robotTwoCopy();  // the pivots only
    noClaw.count = 2;
    MechanismTable threeWay = robotTwoCopy();  // 'claw' on lines A and B with a THIRD position
    {
        auto& q = threeWay.specs[2];
        q.lineCount = 2;
        for (std::size_t i = 0; i < 2; ++i) {
            q.pneumatic.positions[i].levels[1] = false;  // closed {Low, Low}, open {High, Low}
            q.pneumatic.positions[i].levelCount = 2;
        }
        std::snprintf(q.pneumatic.positions[2].name, sizeof q.pneumatic.positions[2].name, "%s", "half");
        q.pneumatic.positions[2].levels[0] = false;  // half {Low, High}
        q.pneumatic.positions[2].levels[1] = true;
        q.pneumatic.positions[2].levelCount = 2;
        q.pneumatic.positionCount = 3;
        threeWay.wiring[2].lines[1] = shulib::mechanism::Line::B;
    }
    struct Case {
        const char* what;
        const MechanismTable* list;
        const char* why;
        int extraMotor;  // 0: none
    };
    for (const Case k :
         {Case{"no claw pivot 7", &fewer, "a control names 'claw pivot 7', which the robot does not declare", 0},
          Case{"claw pivot 7 a pneumatic", &kinds,
               "'claw pivot 7' is a pneumatic mechanism on this robot: its control is for a power one", 0},
          Case{"an extra intake", &withIntake, "the robot declares 'intake', and no control names it", 8},
          Case{"no claw", &noClaw, "a control names 'claw', which the robot does not declare", 0},
          Case{"a three-position claw", &threeWay,
               "DriverInputs::bind: a control's preset, position, speed or homing is not this robot's "
               "mechanism's (another robot's declaration of the same name)",
               0}}) {
        CAPTURE(k.what);
        Rig r{true, true, *k.list, /*bindNow=*/false, k.extraMotor};
        r.bindService();
        RobotTwoDriver d;
        char why[192] = "stale";
        bool bound = true;
        CHECK_NOTHROW(bound = d.bind(*r.mechanisms, why, sizeof why));
        CHECK_FALSE(bound);
        CHECK(std::string_view{why} == k.why);
        CHECK_FALSE(d.bound());  // nothing bound: it can still bind robot two
    }
    // A refusal with no room for its reason still refuses (and writes nothing).
    {
        Rig r{true, true, noClaw, /*bindNow=*/false};
        r.bindService();
        RobotTwoDriver d;
        CHECK_FALSE(d.bind(*r.mechanisms, nullptr, 0));
        CHECK_FALSE(d.bound());
    }
    Rig r;
    RobotTwoDriver d;
    char why[192] = "stale";
    CHECK(d.bind(*r.mechanisms, why, sizeof why));
    CHECK(std::string_view{why}.empty());  // a bind that succeeds says nothing
    CHECK(d.bound());
}

TEST_CASE("T4 1a: the shared claw row — clawRowText(list, robot, service, freed) — prints the row "
          "Station J quotes, byte for byte, with FREED exactly when `freed`") {
    // BUG CAUGHT: the shared form reading a different mechanism, the claw's word read from anywhere
    // but its declaration's levels, or FREED dropped — a driver-control program's row would differ
    // from the one Station J quotes (the strings below are what the row printed before MECH-A5).
    Rig r;
    CHECK(std::string{clawRowText(two::kRobotTwoMechanisms, *r.robot, *r.mechanisms, false).text}
          == "CLAW 6 HOLD 20C | 7 HOLD 20C | CLAW CLOSED");
    CHECK(std::string{clawRowText(two::kRobotTwoMechanisms, *r.robot, *r.mechanisms, true).text}
          == "CLAW 6 HOLD 20C | 7 HOLD 20C | CLAW CLOSED FREED");
    r.mechanisms->pneumatic(2).set(1);  // 'open'
    CHECK(std::string{clawRowText(two::kRobotTwoMechanisms, *r.robot, *r.mechanisms, false).text}
          == "CLAW 6 HOLD 20C | 7 HOLD 20C | CLAW OPEN");
}

TEST_CASE("MECH-A5: the driver file holds only the team's decisions — the [driver file two] block wraps "
          "the WHICH BUTTON heading and the controls statement ONLY (G3a's ask), byte for byte the "
          "fixture's; the feel and the loop are after it; RobotTwoDriver is the shared DriverFile") {
    // BUG CAUGHT: G3a's `controls` region swallowing the hand-written feel (its generator would then
    // overwrite the team's kFeel / kLoop — G3a's RC5, D4 accepted 2026-09-29); a marker doubled; the
    // fixture every library test of robot two's controls runs drifting from the real file; plumbing
    // (a class, a bind(), static_asserts) creeping back into the team's file (the MECH brief's N6).
    const std::string file = source_pins::readFile("src/team/robot_two_driver.hpp");
    const std::string begin = "// [driver file two]\n";
    const std::string end = "// [/driver file two]\n";
    REQUIRE(file.find(begin) != std::string::npos);
    REQUIRE(file.find(end) != std::string::npos);
    CHECK(file.find(begin, file.find(begin) + 1) == std::string::npos);
    CHECK(file.find(end, file.find(end) + 1) == std::string::npos);
    const std::size_t b = file.find(begin) + begin.size();
    const std::string region = file.substr(b, file.find(end) - b);
    const std::string controls =
        "inline constexpr DriverControls kControls = controls(kRobotTwoMechanisms,\n"
        "    whileHeld(clawPivot6, Button::R1, Button::R2, 12_volt),  // forward / back, full power\n"
        "    whileHeld(clawPivot7, Button::L1, Button::L2, 12_volt),  // forward / back, full power\n"
        "    toggle(claw, Button::A));                                // each press opens or closes it\n";
    CHECK(region.find("// ═══ WHICH BUTTON DOES WHAT") == 0);
    CHECK(region.size() == region.find('\n') + 1 + controls.size());  // the heading, then the statement, nothing else
    CHECK(region.substr(region.find('\n') + 1) == controls);
    CHECK(region.find("kFeel") == std::string::npos);
    CHECK(region.find("kLoop") == std::string::npos);
    CHECK(file.find("inline constexpr DriverProfile kFeel{.mode = ArcadeBody};") > file.find(end));
    CHECK(file.find("inline constexpr DriverLoopConfig kLoop{") > file.find(end));
    // The fixture's controls (test/mechanism_driver_fixtures.hpp) state the same statement.
    CHECK(source_pins::readFile("test/mechanism_driver_fixtures.hpp").find(controls) != std::string::npos);
    // No plumbing in the team's file: no class, no static_assert, no tick period, no library namespace.
    CHECK(file.find("class ") == std::string::npos);
    CHECK(file.find("static_assert") == std::string::npos);
    CHECK(file.find(".tickPeriod") == std::string::npos);
    CHECK(file.find("teleop::") == std::string::npos);
    CHECK(file.find("hal::") == std::string::npos);
    CHECK(file.find("using RobotTwoDriver = DriverFile<robot_two::kControls, robot_two::kFeel, robot_two::kLoop>;")
          != std::string::npos);
    // ...and the real controls ARE the fixture's, control by control.
    const shulib::mechanism::DriverControls& real = two::kControls;
    const shulib::mechanism::DriverControls& fixture = mech_fixture::robot_two::kControls;
    REQUIRE(real.count == 3);
    REQUIRE(fixture.count == real.count);
    for (std::size_t i = 0; i < real.count; ++i) {
        CAPTURE(i);
        CHECK(real.c[i].kind == fixture.c[i].kind);
        CHECK(std::string_view{real.c[i].mechanismName} == fixture.c[i].mechanismName);
        CHECK(real.c[i].button == fixture.c[i].button);
        CHECK(real.c[i].back == fixture.c[i].back);
        CHECK(real.c[i].volts == fixture.c[i].volts);
    }
    // The loop's numbers as the library takes them: the team's, with the BRAIN's tick supplied.
    CHECK(RobotTwoDriver::kLoop.tickPeriod.value() == 0.01);
    CHECK(RobotTwoDriver::kLoop.rezeroButton == ControllerButton::Y);
    CHECK(RobotTwoDriver::kLoop.releaseBrakeMode == shulib::hal::BrakeMode::Coast);
    CHECK(RobotTwoDriver::kFeel.mode == shulib::teleop::DriveMode::ArcadeBody);
    CHECK(two::kLoop.tickPeriod.value() == 0.0);  // the file states none
}


// THE TICK'S EQUALITY EDGE (DriverFile's static_assert; its refusal is test/compile_refusals/
// n56_a_tick_that_is_not_the_brains.cpp, 20 ms): a driver file that states the BRAIN's tick exactly —
// 10 ms, the same double as hal::pros::ProsTickPacer::kTickMs / 1000.0 — compiles, and so does one
// that states none (robot two's). If the static_assert compared with `<` or rounded, this would not
// build.
namespace tick_edge {
using namespace shulib::mechanism::words;
inline constexpr DriverLoopConfig kStated{.tickPeriod = 10_ms, .rezeroButton = Button::Y, .releaseBrakeMode = Coast};
using StatedTick = shulib::bench::DriverFile<two::kControls, two::kFeel, kStated>;
static_assert(StatedTick::kLoop.tickPeriod == shulib::bench::kBrainTick);
static_assert(kStated.tickPeriod == shulib::bench::kBrainTick, "the edge: exactly the brain's tick");
}  // namespace tick_edge

// P2-18 (the T4 brief's §3; built in Part 3's late step with Part 2b's items) — the driver file's
// PROTECTIONS paragraph, its every number against its source. BUG CAUGHT: the paragraph a student
// defends drifting from what the library does under it — a default changed (write "2.5 A", or a
// 99-tick cut) and the file still telling the driver the old number, or the stop's duration
// shortened to "until the field disables the robot" (the marks live in the groups: the REST OF
// THE RUN). The paragraph is read from the file (comments are not compiled), joined into one line,
// and every number in it is checked against the config the file's kLoop runs with.
TEST_CASE("P2-18: the driver file's protections paragraph states the library's own numbers") {
    const std::string raw = source_pins::readFile("src/team/robot_two_driver.hpp");
    const std::size_t from = raw.find("// The library protects the drive under this file");
    REQUIRE(from != std::string::npos);
    const std::size_t to = raw.find("\n//\n", from);
    REQUIRE(to != std::string::npos);
    std::string para;
    for (std::size_t at = from; at < to;) {
        const std::size_t eol = raw.find('\n', at);
        std::string line = raw.substr(at, eol - at);
        REQUIRE(line.rfind("// ", 0) == 0);
        para += (para.empty() ? "" : " ") + line.substr(3);
        at = eol + 1;
    }
    const shulib::teleop::DriveProtectionConfig& p = RobotTwoDriver::kLoop.protection;
    const double tickMs = RobotTwoDriver::kLoop.tickPeriod.value() * 1000.0;
    // The defaults ARE what the file runs with (kLoop names no protection field).
    CHECK(p.overCurrentAmps == shulib::teleop::DriveProtectionConfig{}.overCurrentAmps);
    CHECK(p.cutTicks == shulib::teleop::DriveProtectionConfig{}.cutTicks);
    CHECK(p.overCurrentTicks == shulib::teleop::DriveProtectionConfig{}.overCurrentTicks);
    CHECK(p.overCurrentCuts);  // the paragraph says the over-current STOPS both sides
    char want[160];
    std::snprintf(want, sizeof want, "drawing more than %.1f A while it is driven, for %.0f ms",
                  p.overCurrentAmps, p.overCurrentTicks * tickMs);
    CAPTURE(para);
    CHECK(para.find(want) != std::string::npos);
    // The fight's window is the groups' monitors' (SideMonitorConfig::persistTicks), the same 250
    // ms the paragraph gives both — pinned equal to the over-current's by the library.
    CHECK(shulib::teleop::SideMonitorConfig{}.persistTicks == p.overCurrentTicks);
    std::snprintf(want, sizeof want, "stops BOTH sides for %.0f s, and then driving resumes",
                  p.cutTicks * tickMs / 1000.0);
    CHECK(para.find(want) != std::string::npos);
    CHECK(p.cutTicks * tickMs == 1000.0);  // "1 s" is whole: a 99-tick cut would print "1 s" too
    const char* const words[] = {"zero", "one", "two", "three", "four", "five"};
    REQUIRE(shulib::teleop::kMaxDeadTotal >= 0);
    REQUIRE(shulib::teleop::kMaxDeadTotal < 6);
    std::snprintf(want, sizeof want, "once more than %s are dead, the drive stays at 0 V for the rest "
                                     "of the run",
                  words[shulib::teleop::kMaxDeadTotal]);
    CHECK(para.find(want) != std::string::npos);
    CHECK(para.find("every later start of driver control included, until the program is started "
                    "again")
          != std::string::npos);
    CHECK(para.find("until the field disables") == std::string::npos);
    CHECK(para.find("None of those numbers has been measured (HA-132, HA-175, HA-176)")
          != std::string::npos);
}
