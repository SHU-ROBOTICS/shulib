// ROBOT TWO'S DRIVER FILE (chunk ROB1-BT, 2026-11-16): robot one's map with robot two's name — robot two is
// robot one's twin. Say "host test plant", never "simulator". Its record below is INVENTED; its mechanisms are
// robot two's real declarations on its facts file's PLACEHOLDERS (robot one's numbers, every one owed). Run over
// the FAKE provider, the controller read through the PROS shim (test/robot_one_driver_test.cpp's rig). Nothing
// here has run on a robot. (Robot two's OLD claw driver file is test/retired_robot_two_driver_test.cpp's now.)
//
// WHAT THIS FILE TRIES TO BREAK
//   * ON THE HOST, through RobotTwoDriver: the build's robot-two driver really is robot one's map on robot two's
//     mechanisms — the cascade homes DOWN at the first start; R1 up, R2 down; L1 runs the RIGHT toggle only, L2
//     the LEFT only; A, B and X each flip ITS piston only. (test/robot_twin_test.cpp holds the TEXT equal.)
//   * THE FILE HOLDS ONLY THE TEAM'S DECISIONS (MECH-A5's rule, kept for the twin): the `[driver file two]`
//     block wraps the WHICH BUTTON heading and the controls statement only; the feel after it; no plumbing.
//   * THE BOOT IN ROBOT TWO'S WORDS: a record that names robot ONE is refused before anything is built.

#include "doctest.h"

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <string>
#include <string_view>

#include "common/boot_source.hpp"
#include "common/robot_words.hpp"
#include "pros/shim_control.hpp"
#include "robots/mechanism_table.hpp"
#include "shulib/hal/controller.hpp"
#include "shulib/hal/drivetrain_description.hpp"
#include "shulib/hal/fake/fake_battery.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_device_provider.hpp"
#include "shulib/hal/fake/fake_device_registry.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/mechanism/service.hpp"
#include "shulib/robot/robot.hpp"
#include "shulib/units/quantity.hpp"
#include "source_pins.hpp"
#include "team/robot_two_driver.hpp"

namespace sb = shulib::bench;
namespace facts = sb::robot_two_facts;
using shulib::hal::ControllerButton;
using shulib::mechanism::MechanismService;
using sb::RecordRead;

namespace {

/// An INVENTED robot-two record (its drive is Calibrate's to discover): a tank on 3-5 / 6-8, IMU on 12, clear
/// of the mechanisms' ports; it NAMES robot two, as every record Calibrate writes since ROB1-BT does.
shulib::hal::DrivetrainDescription inventedRecord(const char* provenance) {
    shulib::hal::DrivetrainDescription d{};
    d.kind = shulib::hal::DrivetrainKind::Tank;
    d.cartridge = shulib::hal::MotorCartridge::Blue;
    d.imuPort = 12;
    const std::int8_t left[3] = {-3, 4, -5};
    const std::int8_t right[3] = {6, -7, 8};
    for (std::size_t i = 0; i < 3; ++i) {
        d.left.signedPorts[i] = left[i];
        d.right.signedPorts[i] = right[i];
    }
    d.left.count = 3;
    d.right.count = 3;
    d.left.inchesPerRadian = shulib::units::Length{1.3};
    d.right.inchesPerRadian = shulib::units::Length{1.3};
    d.trackWidth = shulib::units::Length{12.0};
    d.source = shulib::hal::DescriptionSource::RobotMeasured;
    std::snprintf(d.provenance, sizeof d.provenance, "%s", provenance);
    return d;
}

std::size_t shimButton(ControllerButton b) { return 6U + static_cast<std::size_t>(b); }
pros::shim::ControllerState& pad() { return pros::shim::controllerState(pros::E_CONTROLLER_MASTER); }
void press(ControllerButton b, bool down) { pad().digital[shimButton(b)] = down; }

/// Each driver tick advances the fake clock by the brain's 10 ms.
struct ClockPace final : shulib::motion::ITickPacer {
    shulib::hal::fake::FakeClock* clock = nullptr;
    void pace() override { clock->advance(shulib::units::Time{0.01}); }
};

/// The census robot two's boot reads: the invented drive, its IMU, every mechanism motor.
shulib::hal::fake::FakeDeviceRegistry census() {
    shulib::hal::fake::FakeDeviceRegistry c;
    for (const int p : {3, 4, 5, 6, 7, 8, facts::kCascadePorts.value().first, facts::kCascadePorts.value().second,
                        facts::kRightTogglePort.value(), facts::leftTogglePort()}) {
        c.setKind(p, shulib::hal::DeviceKind::Motor);
    }
    c.setKind(12, shulib::hal::DeviceKind::Imu);
    return c;
}

/// Robot two's boot decision, in robot two's words, over a record with `provenance`.
sb::BootOutcome bootTwo(const char* provenance) {
    RecordRead record{};
    record.description = inventedRecord(provenance);
    record.decoded = true;
    return sb::decideBootSourceFrom(record, census(), shulib::hal::DrivetrainDescription{}, "",
                                    sb::robot_two::kRobotTwoMechanisms, "2026 CASCADE ROBOT (robot two)",
                                    [](const char*) {}, sb::WordsOf<sb::RobotId::RobotTwo>{});
}

/// Robot two's composition on the host: its boot, the Robot over the fake provider, the service, its driver.
struct TwoRig {
    TwoRig() {
        pacer.clock = &clock;
        pros::shim::resetAll();
        pad().connected = true;
        battery.setVoltage(shulib::units::Voltage{12.8});
        boot = bootTwo("robot=robot-two robot two driver test (INVENTED)");
        REQUIRE_MESSAGE(boot.ok, boot.why);
        robot.emplace(boot.built.config, devices, boot.built.buildOptions());
        mechanisms.emplace(pacer);
        mechanisms->bind(sb::robot_two::kRobotTwoMechanisms, robot->mechanismParts().devices(),
                         robot->context().clock(), robot->faults(), robot->context().telemetry());
        char why[192] = "";
        REQUIRE_MESSAGE(driver.bind(*mechanisms, why, sizeof why), why);
    }
    /// Run the driver loop `ticks` ticks; `script(k)` runs at the top of tick k; `after(k)` after it.
    template <class Script, class After>
    void run(int ticks, Script script, After after) {
        int k = 0;
        driver.run(
            *robot, *mechanisms, pacer,
            [&] {
                if (k >= ticks) return true;
                script(k);
                return false;
            },
            [&](const shulib::math::ChassisSpeeds&, bool, const shulib::teleop::DriveShaperState&) {
                after(k);
                ++k;
            });
    }
    double volts(int port) { return devices.fakeMotor(static_cast<std::uint8_t>(port)).commandedVoltage().value(); }
    bool line(char letter) { return devices.fakeDigitalOut(static_cast<std::uint8_t>(letter - 'A' + 1)).commanded(); }

    shulib::hal::fake::FakeClock clock;
    shulib::hal::fake::FakeBattery battery;
    shulib::hal::fake::FakeTelemetrySink sink;
    ClockPace pacer;
    shulib::hal::fake::FakeDeviceProvider devices{clock, battery, sink, pacer};
    sb::BootOutcome boot;
    std::optional<shulib::robot::Robot> robot;
    std::optional<MechanismService> mechanisms;
    sb::RobotTwoDriver driver;
};

}  // namespace

TEST_CASE("robot two driver: on the host -- the cascade homes DOWN at the first start; R1 up, R2 down; L1 runs ONLY the right toggle, L2 ONLY the left; A, B, X each flip ONLY their piston") {
    // BUG CAUGHT: robot two's build running another map than robot one's (the twin's whole point), or its
    // controls bound to the wrong mechanisms (L1 running the left toggle), or a piston firing at power-up.
    TwoRig r;
    const int c1 = facts::kCascadePorts.value().first;
    const int right = facts::kRightTogglePort.value();
    const int left = facts::leftTogglePort();
    const char claw = facts::kClawLine.value();
    const char flip = facts::kFlipperLine.value();
    const char ramp = facts::kRampLine.value();
    double home = 0.0, up = 0.0, down = 0.0, l1Right = 0.0, l1Left = 99.0, l2Right = 99.0, l2Left = 0.0;
    bool startClaw = true, aClaw = false, aFlip = true, aRamp = true, bFlip = false, xRamp = false;
    r.run(
        620,
        [](int k) {
            const auto in = [k](int a, int b) { return k >= a && k < b; };
            press(ControllerButton::R1, in(330, 360));
            press(ControllerButton::R2, in(365, 395));
            press(ControllerButton::L1, in(400, 420));
            press(ControllerButton::L2, in(430, 450));
            press(ControllerButton::A, in(470, 475));
            press(ControllerButton::B, in(520, 525));
            press(ControllerButton::X, in(570, 575));
        },
        [&](int k) {
            if (k == 3) { home = r.volts(c1); startClaw = r.line(claw); }
            if (k == 350) up = r.volts(c1);
            if (k == 385) down = r.volts(c1);
            if (k == 410) { l1Right = r.volts(right); l1Left = r.volts(left); }
            if (k == 440) { l2Right = r.volts(right); l2Left = r.volts(left); }
            if (k == 500) { aClaw = r.line(claw); aFlip = r.line(flip); aRamp = r.line(ramp); }
            if (k == 550) bFlip = r.line(flip);
            if (k == 600) xRamp = r.line(ramp);
        });
    CHECK(home < 0.0);  // DOWN, at the first driver start
    CHECK(up > 0.0);
    CHECK(down < 0.0);
    CHECK(l1Right == doctest::Approx(12.0));
    CHECK(l1Left == doctest::Approx(0.0));
    CHECK(l2Right == doctest::Approx(0.0));
    CHECK(l2Left == doctest::Approx(12.0));
    CHECK_FALSE(startClaw);  // made at Low: nothing moves at power-up
    CHECK(aClaw);
    CHECK_FALSE(aFlip);
    CHECK_FALSE(aRamp);
    CHECK(bFlip);
    CHECK(xRamp);
}

TEST_CASE("robot two driver (MECH-A5's rule): the [driver file two] block is the WHICH BUTTON heading and the controls statement ONLY; the feel after it; no plumbing") {
    // BUG CAUGHT: G3a's `controls` region (and the route maker's) swallowing the hand-written feel, a marker
    // doubled, plumbing (a class, a static_assert, a tick period) creeping into the team's file.
    const std::string file = source_pins::readFile("src/team/robot_two_driver.hpp");
    const std::string begin = "// [driver file two]\n";
    const std::string end = "// [/driver file two]\n";
    REQUIRE(source_pins::count(file, begin) == 1);
    REQUIRE(source_pins::count(file, end) == 1);
    const std::size_t b = file.find(begin) + begin.size();
    const std::string region = file.substr(b, file.find(end) - b);
    CHECK(region.find("// ═══ WHICH BUTTON DOES WHAT") == 0);
    CHECK(source_pins::count(region, "inline constexpr DriverControls kControls = controls(kRobotTwoMechanisms,") == 1);
    CHECK(region.find("kFeel") == std::string::npos);
    CHECK(region.find("kLoop") == std::string::npos);
    CHECK(file.find("inline constexpr DriverProfile kFeel{.mode = ArcadeBody};") > file.find(end));
    for (const char* m : {"// [control words two]\n", "// [/control words two]\n", "// [feel two]\n", "// [/feel two]\n"}) {
        CAPTURE(m);
        CHECK(source_pins::count(file, m) == 1);  // each marker alone on its line, once
    }
    CHECK(file.find("class ") == std::string::npos);
    CHECK(file.find("static_assert") == std::string::npos);
    CHECK(file.find(".tickPeriod") == std::string::npos);
}

TEST_CASE("robot two driver (G1-B): robot two's boot refuses a record that names robot ONE, or none, before anything is built") {
    // BUG CAUGHT: robot one's SD card in robot two's brain — the twins are wired the same, so the port check
    // passes, and robot two would drive on robot one's measured directions, scale and width in silence.
    const sb::BootOutcome theirs = bootTwo("robot=robot-one Nov 16 2026 10:00:00");
    CHECK_FALSE(theirs.ok);
    CHECK(std::string_view{theirs.headline} == "the card's record is ANOTHER ROBOT'S");
    CHECK(std::string_view{theirs.why} == "this card's record is robot=robot-one's; this is robot=robot-two -- run "
                                          "shulib Calibrate (slot 4) on THIS robot");
    CHECK(theirs.built.config.mechanismCount == 0);  // nothing built
    const sb::BootOutcome none = bootTwo("shulib Calibrate Nov 10 2026 10:00:00 | ROBOT-MEASURED: 6 ports");
    CHECK_FALSE(none.ok);
    CHECK(std::string_view{none.headline} == "the card's record names NO ROBOT");
    const sb::BootOutcome mine = bootTwo("robot=robot-two Nov 16 2026 10:00:00 | ROBOT-MEASURED: 6 ports");
    CHECK(mine.ok);
}
