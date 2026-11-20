// ROBOT TWO'S DRIVER FILE (chunk ROB1-BT, 2026-11-16; its map its own since 2026-11-20). Robot two is robot one's
// twin. Say "host test plant", never "simulator". Its record below is INVENTED; its mechanisms are
// robot two's real declarations on its facts file's PLACEHOLDERS (robot one's numbers, every one owed). Run over
// the FAKE provider, the controller read through the PROS shim (test/robot_one_driver_test.cpp's rig). Nothing
// here has run on a robot. (Robot two's OLD claw driver file is test/retired_robot_two_driver_test.cpp's now.)
//
// WHAT THIS FILE TRIES TO BREAK
//   * ON THE HOST, through RobotTwoDriver: robot two's OWN map (the team lead, 2026-11-20: each robot's buttons
//     are its own) on robot two's mechanisms — the cascade homes DOWN at the first start, its up button up; each
//     side toggle's button runs ONLY its toggle; each piston's button flips ONLY its piston. The rules any map
//     keeps and the header map's rows (GM22), as robot one's file checks robot one's.
//   * THE FILE HOLDS ONLY THE TEAM'S DECISIONS (MECH-A5's rule, kept for the twin): the `[driver file two]`
//     block wraps the WHICH BUTTON heading and the controls statement only; the feel after it; no plumbing.
//   * THE BOOT IN ROBOT TWO'S WORDS: a record that names robot ONE is refused before anything is built.

#include "doctest.h"

#include <algorithm>
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
#include "driver_map_rows.hpp"
#include "source_pins.hpp"
#include "team/robot_two_driver.hpp"

namespace sb = shulib::bench;
namespace facts = sb::robot_two_facts;
using shulib::hal::ControllerButton;
using shulib::mechanism::ControlKind;
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

// ═══ the map, as written: robot two's OWN (the team lead, 2026-11-20: "can you relax that rule") ═══════════

namespace {
/// The control of `kind` on the mechanism called `name` (nullptr when none).
const shulib::mechanism::Control* controlFor(ControlKind kind, std::string_view name) {
    for (const shulib::mechanism::Control& c : sb::robot_two::kControls.all()) {
        if (c.kind == kind && std::string_view{c.mechanismName} == name) return &c;
    }
    return nullptr;
}
/// The control a driver's button runs (nullptr: the button is free).
const shulib::mechanism::Control* controlOn(ControllerButton b) {
    for (const shulib::mechanism::Control& c : sb::robot_two::kControls.all()) {
        if (c.kind == ControlKind::AutonOnly || c.kind == ControlKind::HomeAtDriverStart) continue;
        if (c.button == b || (c.kind == ControlKind::Hold && c.back == b)) return &c;
    }
    return nullptr;
}
}  // namespace

TEST_CASE("robot two driver: the RULES any team map keeps -- the re-zero button free, every cascade level through the ledger, each side toggle on its own port") {
    // BUG CAUGHT: a control on the loop's re-zero button; a cascade level by a quoted name or a constant; a side
    // toggle wired to the other side's port. WHICH button does what is the team's, on each robot (not pinned).
    CHECK(sb::robot_two::kControls.avoids(sb::RobotTwoDriver::kLoop.rezeroButton));
    CHECK(controlOn(sb::RobotTwoDriver::kLoop.rezeroButton) == nullptr);
    const std::string code =
        source_pins::squash(source_pins::stripComments(source_pins::readFile("src/team/robot_two_driver.hpp")));
    CHECK(source_pins::count(code, "preset(cascade") == source_pins::count(code, "preset(cascade[facts::lv("));
    CHECK(source_pins::count(code, "cascade[\"") == 0);
    const auto& list = sb::robot_two::kRobotTwoMechanisms;
    int sides = 0;
    for (std::size_t i = 0; i < list.count; ++i) {
        if (std::string_view{list.specs[i].name} == "right toggle") {
            CHECK(list.wiring[i].motors[0] == facts::kRightTogglePort.value());
            ++sides;
        }
        if (std::string_view{list.specs[i].name} == "left toggle") {
            CHECK(list.wiring[i].motors[0] == facts::leftTogglePort());
            ++sides;
        }
    }
    CHECK(sides == 2);
}

TEST_CASE("robot two driver (GM22): every button with a job (and the re-zero) has ONE row in robot two's header map; a FREE row names only free buttons") {
    // BUG CAUGHT: robot two's buttons changed without its map (the students read the map; the robot runs the
    // file), or a row left for a button nothing uses. Robot one's file has the same case.
    const auto rows = driver_map::mapRows(source_pins::readFile("src/team/robot_two_driver.hpp"));
    REQUIRE(rows.size() >= 3);
    const ControllerButton rezero = sb::RobotTwoDriver::kLoop.rezeroButton;
    const auto jobRows = [&](ControllerButton b) {
        int n = 0;
        for (const auto& [key, words] : rows) {
            const auto bs = driver_map::keyButtons(key);
            n += words.find("FREE") == std::string::npos && std::find(bs.begin(), bs.end(), b) != bs.end() ? 1 : 0;
        }
        return n;
    };
    bool startHome = false;
    for (const shulib::mechanism::Control& c : sb::robot_two::kControls.all()) {
        startHome = startHome || c.kind == ControlKind::HomeAtDriverStart;
        if (c.kind == ControlKind::AutonOnly || c.kind == ControlKind::HomeAtDriverStart) continue;
        CAPTURE(driver_map::buttonName(c.button));
        CHECK(jobRows(c.button) == 1);
        if (c.kind == ControlKind::Hold && !c.oneWay) {
            CAPTURE(driver_map::buttonName(c.back));
            CHECK(jobRows(c.back) == 1);
        }
    }
    CHECK(jobRows(rezero) == 1);
    CHECK(std::count_if(rows.begin(), rows.end(), [](const auto& r) { return r.first == "(start)"; }) == (startHome ? 1 : 0));
    for (const auto& [key, words] : rows) {
        CAPTURE(key);
        for (const ControllerButton b : driver_map::keyButtons(key)) {
            CAPTURE(driver_map::buttonName(b));
            const bool free = controlOn(b) == nullptr && b != rezero;
            CHECK(free == (words.find("FREE") != std::string::npos));  // FREE exactly when it has no job
        }
    }
}

TEST_CASE("robot two driver: on the host -- homes DOWN at the first start; the cascade's up button up, its down button down; each side toggle's button runs ONLY its toggle; each piston's button flips ONLY its piston") {
    // BUG CAUGHT: robot two's controls bound to the wrong mechanisms (the right toggle's button running the
    // left toggle), a cascade going DOWN on its up button, a piston firing at power-up or with another's button.
    // Each button is read from robot two's OWN map (src/team/robot_two_driver.hpp), whatever the team chose.
    const auto* jog = controlFor(ControlKind::Hold, "cascade");
    const auto* home = controlFor(ControlKind::HomeAtDriverStart, "cascade");
    const auto* rightHold = controlFor(ControlKind::Hold, "right toggle");
    const auto* leftHold = controlFor(ControlKind::Hold, "left toggle");
    const char lines[3] = {facts::kClawLine.value(), facts::kFlipperLine.value(), facts::kRampLine.value()};
    const char* const names[3] = {"claw", "flipper", "ramp"};
    const shulib::mechanism::Control* toggles[3] = {};
    for (int i = 0; i < 3; ++i) {
        toggles[i] = controlFor(ControlKind::Toggle, names[i]);
        if (toggles[i] == nullptr) MESSAGE("robot two's map has no toggle on the " << names[i]);
    }
    const bool twoWay = jog != nullptr && !jog->oneWay;
    if (!twoWay) MESSAGE("robot two's map has no two-button hold on the cascade: its jog is not driven here");
    TwoRig r;
    const int c1 = facts::kCascadePorts.value().first;
    const int right = facts::kRightTogglePort.value();
    const int left = facts::leftTogglePort();
    const int t0 = home != nullptr ? static_cast<int>(home->bound * 100.0) + 20 : 20;
    double firstHome = 0.0, up = 0.0, down = 0.0;
    double rightOn = 0.0, rightOther = 99.0, rightReleased = 99.0, leftOn = 0.0, leftOther = 99.0, leftReleased = 99.0;
    bool start[3] = {true, true, true};
    bool seen[3][2][3] = {};
    // The schedule (10 ms ticks) past the driver-start home's bound (t0): the cascade's up button, its down button,
    // the right toggle's, the left toggle's; then each piston's button twice (pressed at 200 + 80 i and 40 later).
    r.run(
        t0 + 460,
        [&](int k) {
            const auto in = [k, t0](int a, int b) { return k >= t0 + a && k < t0 + b; };
            if (twoWay) {
                press(jog->button, in(10, 40));
                press(jog->back, in(45, 75));
            }
            if (rightHold != nullptr) press(rightHold->button, in(90, 120));
            if (leftHold != nullptr) press(leftHold->button, in(140, 170));
            for (int i = 0; i < 3; ++i) {
                if (toggles[i] == nullptr) continue;
                const int a = 200 + 80 * i;
                press(toggles[i]->button, in(a, a + 5) || in(a + 40, a + 45));
            }
        },
        [&](int k) {
            if (k == 3) firstHome = r.volts(c1);
            if (k == 10) for (int j = 0; j < 3; ++j) start[j] = r.line(lines[j]);
            if (k == t0 + 30) up = r.volts(c1);
            if (k == t0 + 65) down = r.volts(c1);
            if (k == t0 + 110) { rightOn = r.volts(right); rightOther = r.volts(left); }
            if (k == t0 + 130) rightReleased = r.volts(right);
            if (k == t0 + 160) { leftOn = r.volts(left); leftOther = r.volts(right); }
            if (k == t0 + 180) leftReleased = r.volts(left);
            for (int i = 0; i < 3; ++i) {
                for (int p = 0; p < 2; ++p) {
                    if (k == t0 + 220 + 80 * i + 40 * p) for (int j = 0; j < 3; ++j) seen[i][p][j] = r.line(lines[j]);
                }
            }
        });
    if (home != nullptr) CHECK(firstHome < 0.0);  // DOWN, at the first driver start
    if (twoWay) {
        CHECK(up > 0.0);
        CHECK(down < 0.0);
    }
    if (rightHold != nullptr) {
        CHECK(rightOn == doctest::Approx(rightHold->volts));
        CHECK(rightOther == doctest::Approx(0.0));
        CHECK(rightReleased == doctest::Approx(0.0));
    }
    if (leftHold != nullptr) {
        CHECK(leftOn == doctest::Approx(leftHold->volts));
        CHECK(leftOther == doctest::Approx(0.0));
        CHECK(leftReleased == doctest::Approx(0.0));
    }
    for (int j = 0; j < 3; ++j) CHECK_FALSE(start[j]);  // made at Low: nothing moves at power-up
    for (int i = 0; i < 3; ++i) {
        if (toggles[i] == nullptr) continue;
        CAPTURE(names[i]);
        for (int j = 0; j < 3; ++j) {
            CAPTURE(names[j]);
            CHECK(seen[i][0][j] == (i == j));  // the first press: that piston, and only it
            CHECK_FALSE(seen[i][1][j]);        // the second: back
        }
    }
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
    CHECK(file.find("inline constexpr DriverProfile kFeel") > file.find(end));  // its VALUE is the team's
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
