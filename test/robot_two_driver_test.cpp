// robot_two_driver_test.cpp — T4 Part 1a: robot two's DRIVER FILE (src/team/robot_two_driver.hpp),
// end to end through the REAL src/ table, hal::pros::ProsDeviceProvider and the PROS shim.
//
// ═══ WHAT THIS FILE PINS ════════════════════════════════════════════════════════════
// The driver file is what the competition program's opcontrol() will run (chunk COMP-b), and the
// driver program after it (T4 Part 3); no program runs it yet. So each case below builds the
// composition a program will: the boot decision over robot two's 2026-09-25 record and the shim's
// registry with the REAL kRobotTwoMechanisms (or a copy with ONE field changed), robot::Robot over
// the REAL provider, RobotTwoDriver::bind(), then RobotTwoDriver::run() — and reads the result at
// the bottom of the stack: each port's millivolts, brake() calls and brake mode in the shim, and
// every level 3-wire port A was ever driven to. The cases carry the T4 brief's names (T1-T10); T7
// (the heading rule) is test/driver_controls_test.cpp's. Where the shim cannot show a thing — the
// loop's [WARN][TEL] line goes to the provider's stdout — the same driver file runs over the fake
// provider, whose telemetry a test can read.
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
#include "common/driver_controls.hpp"
#include "programs/teleop_claws.hpp"
#include "pros/shim_control.hpp"
#include "robots/mechanism_table.hpp"
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
#include "shulib/robot/robot.hpp"
#include "shulib/teleop/driver_control.hpp"
#include "shulib/teleop/input_shaping.hpp"
#include "shulib/teleop/teleop_loop.hpp"
#include "shulib/units/quantity.hpp"
#include "team/robot_two_driver.hpp"

using shulib::bench::BootOutcome;
using shulib::bench::clawRowText;
using shulib::bench::controlsMatchTable;
using shulib::bench::decideBootSourceFrom;
using shulib::bench::kRobotTwoMechanisms;
using shulib::bench::kTeleopRezeroButton;
using shulib::bench::MechanismTable;
using shulib::bench::PneumaticPosition;
using shulib::bench::RecordRead;
using shulib::bench::RobotTwoDriver;
using shulib::hal::ControllerButton;

// ═══ THE INTERIM TIE (T4 Part 1a → 1b), seen by the host suite too ════════════════════
// The same two checks src/programs/teleop_program.cpp makes (that file is not host-compiled). The
// second is written against the table's reserved button: Teleop's own kRezeroButton lives in its
// anonymous namespace, and is static_assert-tied to kTeleopRezeroButton there.
static_assert(controlsMatchTable(RobotTwoDriver::kHolds, RobotTwoDriver::kToggles,
                                 kRobotTwoMechanisms),
              "the driver file and the mechanism table disagree about a button -- change both "
              "until T4 Part 1b");
static_assert(RobotTwoDriver::kLoop.rezeroButton == kTeleopRezeroButton,
              "the driver file's re-zero button and the table's kTeleopRezeroButton disagree -- "
              "change both until T4 Part 1b");

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

/// What one tick left at the bottom of the stack, read in perTick (after drive() and the bindings).
struct Snap {
    std::array<std::int32_t, 22> mv{};
    std::array<int, 22> brakes{};
    std::array<int, 22> moves{};
    std::array<bool, 22> stopped{};
    std::array<pros::v5::MotorBrake, 22> mode{};
    std::size_t lineWrites = 0;
    std::string clawRow;
};
Snap snap(const shulib::robot::Robot& robot, bool freed) {
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
    s.clawRow = clawRowText(kRobotTwoMechanisms, robot, freed).text;
    return s;
}

/// A program's composition for robot two, on the host: the boot decision (the REAL table unless a
/// case hands a copy with ONE field changed), the Robot over the REAL provider, the driver file
/// bound. `sixPlugged` / `sevenPlugged` say which claw pivots carry a motor; `extraMotors` plugs
/// more motor ports (a copy that declares an intake on 8).
struct Rig {
    explicit Rig(bool sixPlugged = true, bool sevenPlugged = true,
                 const MechanismTable& table = kRobotTwoMechanisms, bool bindNow = true,
                 std::initializer_list<int> extraMotors = {}) {
        pros::shim::resetAll();
        for (const int p : {11, 12, 13, 14, 15, 16, 17, 18, 20}) {  // the 09-25 census (19 empty)
            pros::shim::setPluggedAtPort(p, pros::c::E_DEVICE_MOTOR);
        }
        pros::shim::setPluggedAtPort(2, pros::c::E_DEVICE_IMU);
        if (sixPlugged) pros::shim::setPluggedAtPort(6, pros::c::E_DEVICE_MOTOR);
        if (sevenPlugged) pros::shim::setPluggedAtPort(7, pros::c::E_DEVICE_MOTOR);
        for (const int p : extraMotors) pros::shim::setPluggedAtPort(p, pros::c::E_DEVICE_MOTOR);
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
        if (bindNow) {
            REQUIRE_MESSAGE(driver.bind(*robot, why, sizeof why), why);
        }
    }

    /// RobotTwoDriver::run() for `ticks` ticks; `script(k)` runs at the TOP of tick k (inside the
    /// disabled predicate, which the loop polls first).
    std::vector<Snap> run(int ticks, const std::function<void(int)>& script) {
        std::vector<Snap> log;
        int k = 0;
        driver.run(
            *robot, devices->pacer(),
            [&] {
                if (k >= ticks) return true;
                script(k);
                return false;
            },
            [&](const shulib::math::ChassisSpeeds&, bool, const shulib::teleop::DriveShaperState&) {
                log.push_back(snap(*robot, driver.bindings().anyReleasedLeftoverClaim()));
                ++k;
            });
        return log;
    }

    BootOutcome boot;
    std::optional<shulib::hal::pros::ProsDeviceProvider> devices;
    std::optional<shulib::robot::Robot> robot;
    RobotTwoDriver driver;  // declared after the robot: destroyed first, as a program's catch does
    char why[192] = "";
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
                                    kRobotTwoMechanisms, "robot two", [](const char*) {});
        REQUIRE_MESSAGE(boot.ok, boot.why);
        robot.emplace(boot.built.config, devices, boot.built.buildOptions());
    }
    template <class Script>
    void run(int ticks, Script script) {
        int k = 0;
        driver.run(
            *robot, pacer,
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
    RobotTwoDriver driver;
};

}  // namespace

// ═══ T1 — the right ports ═══════════════════════════════════════════════════════════════
TEST_CASE("T4 1a T1: through the driver file, R1 alone drives port 6 at +12000 mV in the tick it is "
          "read and leaves 7 alone, R2 -12000, L1 port 7 +12000, L2 -12000; each release is ONE "
          "brake() under HOLD; the drive ports get exactly what they get with no button; holdFor() "
          "names the binding that moves each port") {
    // BUG CAUGHT: R1/R2 swapped in the driver file; the two holds' names swapped (R1 would move
    // port 7); a hold bound to the wrong mechanism; a release re-sent every tick; a binding that
    // reaches a drive port, or a drive that notices a claw button; holdFor() answering by position.
    std::vector<Snap> with;
    {
        Rig r;
        REQUIRE(r.driver.bindings().holds().size() == 2U);
        REQUIRE(r.driver.bindings().toggles().size() == 1U);
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

    // holdFor(): the binding by NAME — pushed through ITS mechanism, ITS port moves.
    Rig r;
    const auto* six = r.driver.bindings().holdFor("claw pivot 6");
    const auto* seven = r.driver.bindings().holdFor("claw pivot 7");
    REQUIRE(six != nullptr);
    REQUIRE(seven != nullptr);
    CHECK(six->forwardButton() == ControllerButton::R1);
    CHECK(seven->forwardButton() == ControllerButton::L1);
    six->mechanism().setVoltage(shulib::units::Voltage{5.0});
    CHECK(pros::shim::motorState(6).lastVoltageMv == 5000);
    CHECK(pros::shim::motorState(7).lastVoltageMv == 0);
    seven->mechanism().setVoltage(shulib::units::Voltage{-4.0});
    CHECK(pros::shim::motorState(7).lastVoltageMv == -4000);
    CHECK(pros::shim::motorState(6).lastVoltageMv == 5000);
    CHECK(r.driver.bindings().holdFor("claw") == nullptr);    // a toggle's name, not a hold's
    CHECK(r.driver.bindings().holdFor("intake") == nullptr);  // not declared
    CHECK(r.driver.bindings().holdFor(nullptr) == nullptr);
}

// ═══ T2 — the claw ═══════════════════════════════════════════════════════════════════════
TEST_CASE("T4 1a T2: the claw through the driver file — line A's FIRST level ever is the start "
          "level and the first entry sends the same; each press flips it ONCE; a disable and a "
          "second run() write nothing; toggleFor(\"claw\") is the binding on line A") {
    // BUG CAUGHT: the toggle's initial level inverted; a flip on a held A; a disable, or the next
    // entry, that commands the claw; the toggles never handed to the loop; toggleFor() by position.
    Rig r;
    const bool start = shulib::bench::pneumaticStartCommand(kRobotTwoMechanisms.pneumatics[0]);
    REQUIRE(start == !shulib::bench::kClawOpenCommand);  // CLOSED, through the polarity
    const std::int32_t s = start ? 1 : 0;
    const std::int32_t o = start ? 0 : 1;
    REQUIRE(lineA() == std::vector<std::int32_t>{s});  // built at the start level, nothing before
    const auto* claw = r.driver.bindings().toggleFor("claw");
    REQUIRE(claw != nullptr);
    CHECK(claw->initialState() == start);
    CHECK(claw->button() == ControllerButton::A);
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
    CHECK(lineA().size() == 4U);  // and the next entry sends no initial level
    // toggleFor("claw") IS line A's binding: commanding through it reaches line A.
    claw->mechanism().set(!start);
    CHECK(lineA().size() == 5U);
    CHECK(lineA().back() == o);
    CHECK(r.driver.bindings().toggleFor("claw pivot 6") == nullptr);  // a hold's name
    CHECK(r.driver.bindings().toggleFor("wing") == nullptr);
    // The pivots never moved on A.
    CHECK(log.back().moves[6] == log[0].moves[6]);
    CHECK(log.back().moves[7] == log[0].moves[7]);
}

TEST_CASE("T4 1a T2c: the toggle starts at the robot's START level, not its SAFE one — a table copy "
          "whose claw starts OPEN and is safe CLOSED: line A's first level and the first entry's "
          "are OPEN, and the first press closes it") {
    // BUG CAUGHT: the binder reading pneumaticSafe (robot two's real claw has start == safe ==
    // CLOSED, so T2, T2b and T3 cannot tell the two apart).
    MechanismTable startsOpen = kRobotTwoMechanisms;
    startsOpen.pneumatics[0].start = PneumaticPosition::Open;
    startsOpen.pneumatics[0].safe = PneumaticPosition::Closed;
    Rig r{true, true, startsOpen};
    const std::int32_t open = shulib::bench::kClawOpenCommand ? 1 : 0;
    REQUIRE(lineA() == std::vector<std::int32_t>{open});
    const auto log = r.run(10, [](int k) { press(ControllerButton::A, k >= 4 && k < 6); });
    CHECK(log[0].lineWrites == 2U);
    CHECK(lineA()[1] == open);  // the entry: OPEN, the start — never the safe CLOSED
    CHECK(log[3].lineWrites == 2U);
    CHECK(log[4].lineWrites == 3U);
    CHECK(lineA()[2] == 1 - open);  // the first press CLOSES it
}

TEST_CASE("T4 1a T2b: the team lead's ruling — \"starts closed — whatever autonomous left it at\": "
          "an auton that opens the claw after bind() is overruled ONCE by the first entry; and a "
          "line set OPEN before bind() still gives a toggle that starts CLOSED") {
    // BUG CAUGHT: the toggle's initial level read from what the line is commanded to NOW (the
    // call-order hazard: bind() after an auton would then keep the claw open), or the first entry
    // not sending it; a second entry re-sending it (the claw would move on every re-enable).
    const bool start = shulib::bench::pneumaticStartCommand(kRobotTwoMechanisms.pneumatics[0]);
    const std::int32_t s = start ? 1 : 0;
    {
        Rig r;
        r.robot->pneumaticMechanism("claw")->set(!start);  // an auton opens it
        REQUIRE(lineA().size() == 2U);
        REQUIRE(lineA().back() == 1 - s);
        r.run(3, [](int) {});
        CHECK(lineA().size() == 3U);  // exactly ONE write at the first entry…
        CHECK(lineA().back() == s);   // …CLOSED
        r.run(3, [](int) {});         // a disable, then driver control again
        CHECK(lineA().size() == 3U);  // nothing
    }
    {
        Rig r{true, true, kRobotTwoMechanisms, /*bindNow=*/false};
        r.robot->pneumaticMechanism("claw")->set(!start);  // OPEN before the binding exists
        REQUIRE(r.driver.bind(*r.robot, r.why, sizeof r.why));
        const auto* claw = r.driver.bindings().toggleFor("claw");
        REQUIRE(claw != nullptr);
        CHECK(claw->initialState() == start);  // the CONFIG's start, not the line's level now
        r.run(2, [](int) {});
        CHECK(lineA().size() == 3U);
        CHECK(lineA().back() == s);
    }
}

TEST_CASE("T4 1a T3: the claw follows the ROBOT, not a restated CLOSED — a table copy with the "
          "polarity flipped (Station J.9's fix): the first level and the entry level are both the "
          "flipped one, and the first press still flips it once") {
    // BUG CAUGHT: a driver file or binder that hard-codes CLOSED as a line level (false) — right
    // for today's guess, wrong the day J.9 flips the polarity.
    MechanismTable flipped = kRobotTwoMechanisms;
    flipped.pneumatics[0].openCommand = !shulib::bench::kClawOpenCommand;
    Rig r{true, true, flipped};
    const std::int32_t closed = shulib::bench::kClawOpenCommand ? 1 : 0;  // the flipped CLOSED
    REQUIRE(lineA() == std::vector<std::int32_t>{closed});
    const auto log = r.run(8, [](int k) { press(ControllerButton::A, k >= 3 && k < 5); });
    CHECK(log[0].lineWrites == 2U);
    CHECK(lineA()[1] == closed);
    CHECK(log[3].lineWrites == 3U);
    CHECK(lineA()[2] == 1 - closed);
    CHECK(log[7].lineWrites == 3U);
}

// ═══ T4 / T4b — ABSENT mechanisms ═══════════════════════════════════════════════════════
TEST_CASE("T4 1a T4: a pivot whose port is EMPTY is ABSENT — bind() is true with ONE hold, its "
          "buttons command nothing anywhere, the other pivot works, and the drive runs; for port 7 "
          "and for port 6") {
    // BUG CAUGHT: an absent claw refusing the whole driver (the drive would not run); a binding
    // over a missing motor (a crash).
    {
        Rig r{true, false};
        CHECK(r.driver.bindings().holds().size() == 1U);
        CHECK(r.driver.bindings().holdFor("claw pivot 7") == nullptr);
        REQUIRE(r.driver.bindings().holdFor("claw pivot 6") != nullptr);
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
        CHECK(r.driver.bindings().holds().size() == 1U);
        CHECK(r.driver.bindings().holdFor("claw pivot 6") == nullptr);
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

TEST_CASE("T4 1a T4b: a claw whose 3-wire line the provider refused is ABSENT — bind() is true with "
          "2 holds and 0 toggles, A commands nothing, and the drive runs (the fake provider: the "
          "brain's refuses only a line outside 1..8, which the config refuses first)") {
    // BUG CAUGHT (T3 test 1f's intent, carried before 1f retires): a toggle built over a missing
    // line (a crash), or an absent claw refusing the driver.
    FakeRig f{1};
    char why[192] = "";
    REQUIRE_MESSAGE(f.driver.bind(*f.robot, why, sizeof why), why);
    CHECK(f.driver.bindings().holds().size() == 2U);
    CHECK(f.driver.bindings().toggles().empty());
    CHECK(f.driver.bindings().toggleFor("claw") == nullptr);
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
                direct.push_back(snap(*r.robot, false));
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

// ═══ T8 — bind() twice, unbind() ════════════════════════════════════════════════════════
TEST_CASE("T4 1a T8: bind() twice leaves exactly 2 holds and 1 toggle, no stale entry; unbind() "
          "leaves empty spans") {
    // BUG CAUGHT: bind() that appends to what an earlier call bound (a second bind would bind each
    // pivot twice); unbind() that leaves a span pointing at bindings it reset.
    Rig r;
    const auto* firstSix = r.driver.bindings().holdFor("claw pivot 6");
    REQUIRE(r.driver.bind(*r.robot, r.why, sizeof r.why));
    CHECK(r.driver.bindings().holds().size() == 2U);
    CHECK(r.driver.bindings().toggles().size() == 1U);
    CHECK(r.driver.bindings().holdFor("claw pivot 6") == firstSix);  // the same slot, rebuilt
    CHECK(r.driver.bindings().holdFor("claw pivot 6")->forwardButton() == ControllerButton::R1);
    CHECK(r.driver.bindings().holdFor("claw pivot 7")->forwardButton() == ControllerButton::L1);
    r.driver.unbind();
    CHECK(r.driver.bindings().holds().empty());
    CHECK(r.driver.bindings().toggles().empty());
    CHECK(r.driver.bindings().holdFor("claw pivot 6") == nullptr);
    CHECK(r.driver.bindings().toggleFor("claw") == nullptr);
    REQUIRE(r.driver.bind(*r.robot, r.why, sizeof r.why));  // and binds again
    CHECK(r.driver.bindings().holds().size() == 2U);
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

TEST_CASE("T4 1a T9: a claim left on pivot 6 before run() is RELEASED at entry with ONE "
          "[WARN][TEL] line, R1 drives from the first tick, and anyReleasedLeftoverClaim() — "
          "what the claw row's FREED reads — is true after that entry and false after the next") {
    // BUG CAUGHT: the driver locked out of a pivot by a claim nobody will release (a deleted auton
    // task); a release the log or the screen does not show. (The fake provider: the shim's is on
    // stdout, which no test reads.)
    FakeRig f;
    char why[192] = "";
    REQUIRE_MESSAGE(f.driver.bind(*f.robot, why, sizeof why), why);
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
        if (e.message.find("RELEASED a claim") != std::string::npos) {
            ++released;
            CHECK(e.level == shulib::hal::LogLevel::Warn);
            CHECK(e.subsystem == "TEL");
            CHECK(e.message
                  == "driver control: RELEASED a claim left on mechanism 'claw pivot 6' at entry "
                     "(an operation that outlived its task) -- its buttons work from the first "
                     "tick");
        }
    }
    CHECK(released == 1);
    CHECK(f.driver.bindings().anyReleasedLeftoverClaim());
    CHECK(std::string{clawRowText(kRobotTwoMechanisms, *f.robot,
                                  f.driver.bindings().anyReleasedLeftoverClaim())
                          .text}
          == "CLAW 6 HOLD 0C | 7 HOLD 0C | CLAW CLOSED FREED");  // safed at the disable; 0 C fake
    const int afterFirst = f.sink.size();
    f.run(2, [](int) { press(ControllerButton::R1, false); });
    CHECK_FALSE(f.driver.bindings().anyReleasedLeftoverClaim());
    for (int i = afterFirst; i < f.sink.size(); ++i) {
        CHECK(f.sink.at(i).message.find("RELEASED a claim") == std::string::npos);
    }
    CHECK(std::string{clawRowText(kRobotTwoMechanisms, *f.robot,
                                  f.driver.bindings().anyReleasedLeftoverClaim())
                          .text}
          == "CLAW 6 HOLD 0C | 7 HOLD 0C | CLAW CLOSED");
    // The claw's claim alone says it too (a panel reading only the holds would miss it).
    REQUIRE(f.robot->pneumaticMechanismAt(2)->tryClaim());
    f.run(1, [](int) {});
    CHECK(f.driver.bindings().anyReleasedLeftoverClaim());
}

// ═══ T10 — a robot these controls are not for ═══════════════════════════════════════════
TEST_CASE("T4 1a T10: bind() REFUSES, by name and with nothing bound, a robot these controls are "
          "not for — no 'claw pivot 7'; 'claw pivot 7' pneumatic; an extra 'intake' no control "
          "names; no claw — each on a driver file BOUND to robot two a moment before, and binds "
          "again on robot two afterwards") {
    // BUG CAUGHT (T3 test 1i's intent): an undeclared name read as "absent" (pivot 7's buttons
    // silently dead on a robot built from another table; the claw's A silently dead on a robot
    // with no claw — the toggle's own refusal, which no case reached until the Part 1a review:
    // mutation RV-M7); a hold resolved against a pneumatic mechanism; a declared mechanism nobody
    // can work; and a refusal that leaves the PREVIOUS robot's bindings live (RV-M1: every case
    // here refused on a driver file that had never been bound, so "nothing bound" held trivially).
    RobotTwoDriver d;
    char why[192] = "";
    // Robot two over the FAKE provider — no shim state, so it stays alive (and its bindings valid)
    // while the refusing robot is built on the shim right after it.
    const auto bindTwoFirst = [&](FakeRig& two) {
        REQUIRE(d.bind(*two.robot, why, sizeof why));
        REQUIRE(d.bindings().holds().size() == 2U);
        REQUIRE(d.bindings().toggles().size() == 1U);
    };
    {
        FakeRig two;
        bindTwoFirst(two);
        MechanismTable fewer = kRobotTwoMechanisms;
        fewer.motors[1] = shulib::bench::MotorMechanismEntry{};
        fewer.motorCount = 1;
        Rig r{true, true, fewer, false};
        CHECK_FALSE(d.bind(*r.robot, why, sizeof why));
        CHECK(std::string_view{why}
              == "a control names 'claw pivot 7', which the robot does not declare");
        CHECK(d.bindings().holds().empty());
        CHECK(d.bindings().toggles().empty());
    }
    {
        FakeRig two;
        bindTwoFirst(two);
        MechanismTable kinds = kRobotTwoMechanisms;
        kinds.motors[1] = shulib::bench::MotorMechanismEntry{};
        kinds.motorCount = 1;
        kinds.pneumatics[1] = kRobotTwoMechanisms.pneumatics[0];
        kinds.pneumatics[1].name = "claw pivot 7";
        kinds.pneumatics[1].adiLine = 2;
        kinds.pneumatics[1].toggle = ControllerButton::B;
        kinds.pneumaticCount = 2;
        Rig r{true, true, kinds, false};
        CHECK_FALSE(d.bind(*r.robot, why, sizeof why));
        CHECK(std::string_view{why}
              == "'claw pivot 7' is not a motor mechanism: a hold control needs one");
        CHECK(d.bindings().holds().empty());
        CHECK(d.bindings().toggles().empty());
    }
    {
        FakeRig two;
        bindTwoFirst(two);
        MechanismTable withIntake = kRobotTwoMechanisms;
        withIntake.motors[2] = kRobotTwoMechanisms.motors[0];
        withIntake.motors[2].name = "intake";
        withIntake.motors[2].signedPort = 8;
        withIntake.motors[2].forward = ControllerButton::X;
        withIntake.motors[2].reverse = ControllerButton::B;
        withIntake.motorCount = 3;
        Rig r{true, true, withIntake, false, {8}};
        CHECK_FALSE(d.bind(*r.robot, why, sizeof why));
        CHECK(std::string_view{why} == "the robot declares 'intake', and no control names it");
        CHECK(d.bindings().holds().empty());
        CHECK(d.bindings().toggles().empty());
    }
    {
        FakeRig two;
        bindTwoFirst(two);
        MechanismTable noClaw = kRobotTwoMechanisms;  // the pivots only: the toggle's name is gone
        noClaw.pneumatics[0] = shulib::bench::PneumaticMechanismEntry{};
        noClaw.pneumaticCount = 0;
        Rig r{true, true, noClaw, false};
        CHECK_FALSE(d.bind(*r.robot, why, sizeof why));
        CHECK(std::string_view{why} == "a control names 'claw', which the robot does not declare");
        CHECK(d.bindings().holds().empty());
        CHECK(d.bindings().toggles().empty());
    }
    Rig r{true, true, kRobotTwoMechanisms, false};
    CHECK(d.bind(*r.robot, why, sizeof why));
    CHECK(d.bindings().holds().size() == 2U);
    CHECK(d.bindings().toggles().size() == 1U);
    d.unbind();
}

// ═══ The interim tie, broken by hand ════════════════════════════════════════════════════
TEST_CASE("T4 1a: the interim tie (controlsMatchTable) — robot two's driver file matches its table; "
          "a table copy with ONE button, the volts or the toggle changed does not, and neither does "
          "one that lacks an entry a control names — the toggle's, a hold's, or a hold's declared "
          "only as the other kind") {
    // BUG CAUGHT: a tie that compares only the holds, only the forward buttons, or only by
    // position — a button changed in the table alone would then build a Teleop whose claws differ
    // from the competition program's.
    CHECK(controlsMatchTable(RobotTwoDriver::kHolds, RobotTwoDriver::kToggles, kRobotTwoMechanisms));
    MechanismTable t = kRobotTwoMechanisms;
    t.motors[0].forward = ControllerButton::X;
    CHECK_FALSE(controlsMatchTable(RobotTwoDriver::kHolds, RobotTwoDriver::kToggles, t));
    t = kRobotTwoMechanisms;
    t.motors[1].reverse = ControllerButton::X;
    CHECK_FALSE(controlsMatchTable(RobotTwoDriver::kHolds, RobotTwoDriver::kToggles, t));
    t = kRobotTwoMechanisms;
    t.motors[1].volts = 11.0;
    CHECK_FALSE(controlsMatchTable(RobotTwoDriver::kHolds, RobotTwoDriver::kToggles, t));
    t = kRobotTwoMechanisms;
    t.pneumatics[0].toggle = ControllerButton::B;
    CHECK_FALSE(controlsMatchTable(RobotTwoDriver::kHolds, RobotTwoDriver::kToggles, t));
    t = kRobotTwoMechanisms;  // the claw's entry gone: the toggle has no entry to match
    t.pneumaticCount = 0;
    CHECK_FALSE(controlsMatchTable(RobotTwoDriver::kHolds, RobotTwoDriver::kToggles, t));
    // A HOLD with no entry of its kind (mutation RV-M2 — the hold loop starting from "matched" —
    // survived the Part 1a review until these two): pivot 7's entry gone; then pivot 7 declared
    // only as a PNEUMATIC entry of the same name, so no motor entry answers it.
    t = kRobotTwoMechanisms;
    t.motors[1] = shulib::bench::MotorMechanismEntry{};
    t.motorCount = 1;
    CHECK_FALSE(controlsMatchTable(RobotTwoDriver::kHolds, RobotTwoDriver::kToggles, t));
    t.pneumatics[1] = kRobotTwoMechanisms.pneumatics[0];
    t.pneumatics[1].name = "claw pivot 7";
    t.pneumatics[1].adiLine = 2;
    t.pneumatics[1].toggle = ControllerButton::L1;
    t.pneumaticCount = 2;
    CHECK_FALSE(controlsMatchTable(RobotTwoDriver::kHolds, RobotTwoDriver::kToggles, t));
    t = kRobotTwoMechanisms;  // the two pivots' entries in the other ORDER: still the same facts
    t.motors[0] = kRobotTwoMechanisms.motors[1];
    t.motors[1] = kRobotTwoMechanisms.motors[0];
    CHECK(controlsMatchTable(RobotTwoDriver::kHolds, RobotTwoDriver::kToggles, t));
}

TEST_CASE("T4 1a: the shared claw row — clawRowText(table, robot, freed) — prints what Teleop's "
          "TeleopClaws form prints, byte for byte, with FREED exactly when `freed`") {
    // BUG CAUGHT: the shared form reading a different entry, or dropping FREED, so the competition
    // program's row would differ from the one Station J quotes.
    Rig r;
    shulib::bench::TeleopClaws claws;
    REQUIRE(claws.bind(kRobotTwoMechanisms, *r.robot, kTeleopRezeroButton, r.why, sizeof r.why));
    CHECK(std::string{clawRowText(kRobotTwoMechanisms, *r.robot, false).text}
          == std::string{clawRowText(kRobotTwoMechanisms, *r.robot, claws).text});
    CHECK(std::string{clawRowText(kRobotTwoMechanisms, *r.robot, false).text}
          == "CLAW 6 HOLD 20C | 7 HOLD 20C | CLAW CLOSED");
    CHECK(std::string{clawRowText(kRobotTwoMechanisms, *r.robot, true).text}
          == "CLAW 6 HOLD 20C | 7 HOLD 20C | CLAW CLOSED FREED");
    claws.unbind();
}
