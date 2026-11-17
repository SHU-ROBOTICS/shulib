// ROBOT ONE'S DRIVER FILE (chunk ROB1-A, 2026-11-11; the ROB1 brief's D7, test T11; the builder's map since
// ROB1-C2, 2026-11-15). Say "host test plant", never "simulator". Its record below is INVENTED (a tank on ports
// nobody has read — Calibrate discovers robot one's drive), its mechanisms are their real declarations on the
// facts file's values — typed, or PLACEHOLDERS where still owed. Run over the FAKE provider, with the
// controller read through the PROS shim (robot two's driver test's FakeRig pattern). Nothing here has run on a
// robot.
//
// WHAT THIS FILE TRIES TO BREAK
//   * THE MAP IS THE TEAM'S (T11, GM22 — re-pointed by RMP1-B, the RMP1 brief's §6.4): the students may change
//     it, by hand or with the route builder's Controls tab, after the freeze too (its D15), so nothing here
//     pins WHICH button does what. Pinned: the RULES — the loop's re-zero button free; every cascade level
//     through the ledger (`facts::lv(i).name`); each side toggle on its own port; every button kControls
//     takes has ONE row in the header's map, every other button row says FREE (the rows' order against the code:
//     the route builder's CT23, node). The rows' WORDS are the team's and no test judges them: the route builder
//     writes its own words for a control whose job it changes, and refuses kept words naming a button whose job
//     changed.
//   * ON THE HOST, each button READ FROM kControls: the cascade homes DOWN at the first driver start (if the
//     map homes it); its hold's up button drives it up and its down button down (never past its unhomed volts
//     before it is homed), both = neither; the re-zero moves nothing; a side toggle's button runs ITS toggle
//     only, stopped on release; a piston's toggle flips ITS piston only; every piston line MADE at Low.
//   * THE ROW (D6, ROB1-C2): robot one's six mechanisms print the COMPACT row (MECH ...), within the SMALL budget.

#include "doctest.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "common/boot_source.hpp"
#include "common/driver_panel.hpp"
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
#include "shulib/mechanism/driver_inputs.hpp"
#include "shulib/mechanism/service.hpp"
#include "shulib/robot/robot.hpp"
#include "shulib/teleop/mechanism_binding.hpp"
#include "shulib/units/quantity.hpp"
#include "source_pins.hpp"
#include "team/robot_one_driver.hpp"

namespace sb = shulib::bench;
namespace facts = sb::robot_one_facts;
using shulib::hal::ControllerButton;
using shulib::mechanism::ControlKind;
using shulib::mechanism::MechanismService;
using sb::BootOutcome;
using sb::RecordRead;
using sb::RobotOneDriver;

namespace {

const shulib::mechanism::Control* controlOn(ControllerButton b) {
    for (const shulib::mechanism::Control& c : sb::robot_one::kControls.all()) {
        if (c.kind == ControlKind::AutonOnly || c.kind == ControlKind::HomeAtDriverStart) continue;
        if (c.button == b || (c.kind == ControlKind::Hold && c.back == b)) return &c;
    }
    return nullptr;
}

/// An INVENTED robot-one record: a tank on ports 3-5 (left) and 6-8 (right), IMU on 12 — clear of the
/// mechanisms' ports (10 and 1, 9 and 2). Robot one's drive ports are Calibrate's to discover
/// (src/robots/chassis_table.hpp).
shulib::hal::DrivetrainDescription inventedRecord() {
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
    std::snprintf(d.provenance, sizeof d.provenance, "%s", "robot=robot-one robot one driver test (INVENTED)");
    return d;
}

/// A button's name as the driver file spells it (hal::ControllerButton's order).
const char* nameOf(ControllerButton b) {
    static constexpr const char* kNames[12] = {"L1", "L2", "R1", "R2", "Up", "Down", "Left", "Right", "X", "B", "Y", "A"};
    return kNames[static_cast<std::size_t>(b)];
}

std::size_t shimButton(ControllerButton b) { return 6U + static_cast<std::size_t>(b); }
pros::shim::ControllerState& pad() { return pros::shim::controllerState(pros::E_CONTROLLER_MASTER); }
void press(ControllerButton b, bool down) { pad().digital[shimButton(b)] = down; }

/// The provider's pacer: each driver tick advances the fake clock by the brain's 10 ms (the driver-start
/// home's bound is TIME; a clock that never moves would hold it for ever).
struct ClockPace final : shulib::motion::ITickPacer {
    shulib::hal::fake::FakeClock* clock = nullptr;
    void pace() override { clock->advance(shulib::units::Time{0.01}); }
};

/// Robot one's composition on the host: the boot decision in robot one's words, the Robot over the fake
/// provider, the service bound over its built cascade, robot one's driver file bound to it.
struct OneRig {
    OneRig() {
        pacer.clock = &clock;
        pros::shim::resetAll();
        pad().connected = true;
        battery.setVoltage(shulib::units::Voltage{12.8});
        RecordRead record{};
        record.description = inventedRecord();
        record.decoded = true;
        shulib::hal::fake::FakeDeviceRegistry census;
        for (const int p : {3, 4, 5, 6, 7, 8, facts::kCascadePorts.value().first, facts::kCascadePorts.value().second,
                            facts::kRightTogglePort.value(), facts::leftTogglePort()}) {
            census.setKind(p, shulib::hal::DeviceKind::Motor);
        }
        census.setKind(12, shulib::hal::DeviceKind::Imu);
        boot = sb::decideBootSourceFrom(record, census, shulib::hal::DrivetrainDescription{}, "",
                                        sb::robot_one::kRobotOneMechanisms, "2026 CASCADE ROBOT (robot one)",
                                        [](const char*) {}, sb::WordsOf<sb::RobotId::RobotOne>{});
        REQUIRE_MESSAGE(boot.ok, boot.why);
        robot.emplace(boot.built.config, devices, boot.built.buildOptions());
        mechanisms.emplace(pacer);
        mechanisms->bind(sb::robot_one::kRobotOneMechanisms, robot->mechanismParts().devices(),
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
    double cascade1() { return volts(facts::kCascadePorts.value().first); }
    double cascade2() { return volts(facts::kCascadePorts.value().second); }
    double right() { return volts(facts::kRightTogglePort.value()); }
    double left() { return volts(facts::leftTogglePort()); }
    /// The level the driver file's inputs last wrote to 3-wire `letter` (A..H).
    bool line(char letter) { return devices.fakeDigitalOut(static_cast<std::uint8_t>(letter - 'A' + 1)).commanded(); }

    shulib::hal::fake::FakeClock clock;
    shulib::hal::fake::FakeBattery battery;
    shulib::hal::fake::FakeTelemetrySink sink;
    ClockPace pacer;
    shulib::hal::fake::FakeDeviceProvider devices{clock, battery, sink, pacer};
    BootOutcome boot;
    std::optional<shulib::robot::Robot> robot;
    std::optional<MechanismService> mechanisms;
    RobotOneDriver driver;
};

}  // namespace

// ═══ the map, as written ═══════════════════════════════════════════════════════════════

namespace {
/// The control of `kind` on the mechanism called `name` (nullptr when none).
const shulib::mechanism::Control* controlFor(ControlKind kind, std::string_view name) {
    for (const shulib::mechanism::Control& c : sb::robot_one::kControls.all()) {
        if (c.kind == kind && std::string_view{c.mechanismName} == name) return &c;
    }
    return nullptr;
}
}  // namespace

TEST_CASE("robot one driver (T11, re-pointed by RMP1-B): the RULES any team map keeps -- the re-zero button free, every cascade level through the ledger, each side toggle on its own port") {
    // BUG CAUGHT: a control on the loop's re-zero button; a cascade level by a quoted name or a constant (a
    // level renamed in the facts file then names nothing, or another level); a side toggle wired to the other
    // side's port. WHICH button does what is the team's (§6.4) and is not pinned.
    CHECK(sb::robot_one::kControls.avoids(RobotOneDriver::kLoop.rezeroButton));
    CHECK(controlOn(RobotOneDriver::kLoop.rezeroButton) == nullptr);
    const std::string code =
        source_pins::squash(source_pins::stripComments(source_pins::readFile("src/team/robot_one_driver.hpp")));
    CHECK(source_pins::count(code, "preset(cascade") == source_pins::count(code, "preset(cascade[facts::lv("));
    CHECK(source_pins::count(code, "cascade[\"") == 0);
    const auto& list = sb::robot_one::kRobotOneMechanisms;
    for (std::size_t i = 0; i < list.count; ++i) {
        if (std::string_view{list.specs[i].name} == "right toggle") {
            CHECK(list.wiring[i].motors[0] == facts::kRightTogglePort.value());
        }
        if (std::string_view{list.specs[i].name} == "left toggle") {
            CHECK(list.wiring[i].motors[0] == facts::leftTogglePort());
        }
    }
}

namespace {
/// The header map's rows (`//   <key>  <words>`, continuation lines joined): key -> words. Read from the line after
/// the route builder's `// [control words …]` marker, as the page reads it (the `[the map]` heading above it is the
/// team's prose, one line or more).
std::vector<std::pair<std::string, std::string>> mapRows(const std::string& text) {
    std::vector<std::pair<std::string, std::string>> rows;
    std::size_t at = text.find("// [control words ");
    if (at == std::string::npos) at = text.find("// [the map]");
    REQUIRE(at != std::string::npos);
    at = text.find('\n', at) + 1;
    while (at < text.size()) {
        const std::size_t end = text.find('\n', at);
        const std::string line = text.substr(at, end - at);
        at = end + 1;
        if (line.starts_with("// [")) continue;  // the route builder's marker lines
        if (!line.starts_with("//   ")) break;
        if (line.size() > 5 && line[5] == ' ') {
            if (!rows.empty()) rows.back().second += " " + line.substr(5);
            continue;
        }
        const std::size_t gap = line.find("  ", 5);
        rows.emplace_back(line.substr(5, gap - 5), gap == std::string::npos ? "" : line.substr(gap));
    }
    return rows;
}

/// The buttons a map key names, read as the route builder reads them ("R1 / R2", "Left/Right", "d-pad", "B, X").
std::vector<ControllerButton> keyButtons(const std::string& key) {
    std::vector<ControllerButton> out;
    std::size_t from = 0;
    while (from <= key.size()) {
        std::size_t to = key.find_first_of("/,", from);
        if (to == std::string::npos) to = key.size();
        std::string word = key.substr(from, to - from);
        word.erase(0, word.find_first_not_of(' '));
        word.erase(word.find_last_not_of(' ') + 1);
        if (word == "d-pad") {
            for (const auto b : {ControllerButton::Up, ControllerButton::Down, ControllerButton::Left, ControllerButton::Right}) out.push_back(b);
        }
        for (int i = 0; i < 12; ++i) {
            if (word == nameOf(static_cast<ControllerButton>(i))) out.push_back(static_cast<ControllerButton>(i));
        }
        from = to + 1;
    }
    return out;
}
}  // namespace

TEST_CASE("robot one driver (GM22, re-pointed by RMP1-B): every button with a job (and the re-zero) has ONE row in the header's map; a FREE row names only free buttons") {
    // BUG CAUGHT (GM22): a control added without its row, or a row left for a button nothing uses (the students
    // read the map; the robot runs the file); the d-pad called FREE while one of its buttons has a job (or is the
    // re-zero). Keys as the route builder writes them ("R1 / R2", "d-pad"). The rows' WORDS are the team's: not
    // judged here or anywhere (the file's header says so).
    const auto rows = mapRows(source_pins::readFile("src/team/robot_one_driver.hpp"));
    REQUIRE(rows.size() >= 3);
    const ControllerButton rezero = RobotOneDriver::kLoop.rezeroButton;
    const auto jobRows = [&](ControllerButton b) {
        int n = 0;
        for (const auto& [key, words] : rows) {
            const auto bs = keyButtons(key);
            n += words.find("FREE") == std::string::npos && std::find(bs.begin(), bs.end(), b) != bs.end() ? 1 : 0;
        }
        return n;
    };
    bool startHome = false;
    for (const shulib::mechanism::Control& c : sb::robot_one::kControls.all()) {
        startHome = startHome || c.kind == ControlKind::HomeAtDriverStart;
        if (c.kind == ControlKind::AutonOnly || c.kind == ControlKind::HomeAtDriverStart) continue;
        CAPTURE(nameOf(c.button));
        CHECK(jobRows(c.button) == 1);
        if (c.kind == ControlKind::Hold && !c.oneWay) {
            CAPTURE(nameOf(c.back));
            CHECK(jobRows(c.back) == 1);
        }
    }
    CHECK(jobRows(rezero) == 1);
    CHECK(std::count_if(rows.begin(), rows.end(), [](const auto& r) { return r.first == "(start)"; }) == (startHome ? 1 : 0));
    for (const auto& [key, words] : rows) {
        CAPTURE(key);
        for (const ControllerButton b : keyButtons(key)) {
            CAPTURE(nameOf(b));
            const bool free = controlOn(b) == nullptr && b != rezero;
            CHECK(free == (words.find("FREE") != std::string::npos));  // FREE exactly when it has no job
        }
    }
}

// ═══ on the host, through the real driver file ═════════════════════════════════════════════

TEST_CASE("robot one driver (T11): on the host -- homes DOWN at the first start; the cascade's up button up, its down button down (within its unhomed volts), both = neither; the re-zero moves nothing; each side toggle's button runs ONLY its toggle, stopped on release") {
    // BUG CAUGHT: a cascade that does not home at the first driver start; its up button driving it DOWN; both
    // buttons fighting; the re-zero moving a mechanism; the right toggle's button running the LEFT toggle (or
    // both); a side toggle that keeps running after its button is let go. Each button is the TEAM'S (§6.4).
    const auto* jog = controlFor(ControlKind::Hold, "cascade");
    const auto* home = controlFor(ControlKind::HomeAtDriverStart, "cascade");
    const auto* rightHold = controlFor(ControlKind::Hold, "right toggle");
    const auto* leftHold = controlFor(ControlKind::Hold, "left toggle");
    const ControllerButton rezero = RobotOneDriver::kLoop.rezeroButton;
    const bool twoWay = jog != nullptr && !jog->oneWay;
    if (!twoWay) MESSAGE("the team's map has no two-button hold on the cascade: its jog is not driven here");
    OneRig r;
    double firstHome = 0.0;
    double up = 0.0;
    double down = 0.0;
    double both = 99.0;
    double withY = 99.0;
    double rightOn = 0.0, rightOther = 99.0, rightReleased = 99.0;
    double leftOn = 0.0, leftOther = 99.0, leftReleased = 99.0;
    const double unhomed = facts::kCascadeUnhomedVolts.value();
    // The schedule (10 ms ticks), past the driver-start home's bound (t0): the cascade's up button, its down
    // button, both, nothing with the re-zero, the right toggle's button, the left toggle's.
    const int t0 = home != nullptr ? static_cast<int>(home->bound * 100.0) + 20 : 20;
    r.run(
        t0 + 280,
        [&](int k) {
            const auto in = [k, t0](int a, int b) { return k >= t0 + a && k < t0 + b; };
            if (twoWay) {
                press(jog->button, in(10, 40) || in(80, 110));
                press(jog->back, in(45, 75) || in(80, 110));
            }
            press(rezero, in(120, 150));
            if (rightHold != nullptr) press(rightHold->button, in(160, 190));
            if (leftHold != nullptr) press(leftHold->button, in(210, 240));
        },
        [&](int k) {
            if (k == 3) firstHome = r.cascade1();
            if (k == t0 + 30) up = r.cascade1();
            if (k == t0 + 65) down = r.cascade1();
            if (k == t0 + 105) both = r.cascade1();
            if (k == t0 + 145) withY = r.cascade1();
            if (k == t0 + 180) { rightOn = r.right(); rightOther = r.left(); }
            if (k == t0 + 200) rightReleased = r.right();
            if (k == t0 + 230) { leftOn = r.left(); leftOther = r.right(); }
            if (k == t0 + 260) leftReleased = r.left();
        });
    MESSAGE("robot one on the host: home " << firstHome << " V, up " << up << " V, down " << down << " V, both "
                                           << both << " V, re-zero " << withY << " V; right toggle " << rightOn
                                           << " (left " << rightOther << "); left toggle " << leftOn << " (right "
                                           << leftOther << ")");
    if (home != nullptr) {
        CHECK(firstHome < 0.0);  // DOWN, at the first driver start
        CHECK(firstHome >= facts::kCascadeHomingPushVolts.value() - 1e-9);
    }
    if (twoWay) {
        CHECK(up > 0.0);  // the up button = UP
        CHECK(up <= unhomed + 1e-9);  // never past its unhomed volts before it is homed
        CHECK(down < 0.0);
        CHECK(down >= -unhomed - 1e-9);
        CHECK(both == doctest::Approx(0.0));
        // The second motor gets the same command (its mirroring is its adapter's, once — never the file's).
        CHECK(std::abs(r.cascade2()) == doctest::Approx(std::abs(r.cascade1())));
    }
    CHECK(withY == doctest::Approx(0.0));
    // The right toggle's button: the RIGHT toggle at its volts, the left untouched; let go: stopped. And the
    // mirror. ONE way each: +volts ("run"); WHICH way that turns the motor is its declaration's direction fact,
    // applied ONCE by the motor's SIGNED port (PROS reverses inside the adapter — a host fake indexed by |port|
    // reads the same command either way, so the sign is checked structurally).
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
    const auto signOf = [&](int port) { return r.devices.motorSignedPort(static_cast<std::uint8_t>(port)); };
    CHECK((signOf(facts::kRightTogglePort.value()) < 0) == facts::kRightToggleDirection.value().reversed);
    CHECK((signOf(facts::leftTogglePort()) < 0) == facts::kLeftToggleDirection.value().reversed);
    CHECK(std::abs(signOf(facts::kRightTogglePort.value())) == facts::kRightTogglePort.value());
    CHECK(std::abs(signOf(facts::leftTogglePort())) == facts::leftTogglePort());
}

TEST_CASE("robot one driver (T11): on the host -- every piston's line is MADE at Low (nothing moves at power-up); each piston's toggle button flips ONLY that piston, and back on its second press") {
    // BUG CAUGHT: a piston that starts in its POWERED position (it would fire at power-up); the claw's button
    // firing the ramp; a toggle that does not come back on its second press; one press flipping two pistons.
    OneRig r;
    const char lines[3] = {facts::kClawLine.value(), facts::kFlipperLine.value(), facts::kRampLine.value()};
    const char* const names[3] = {"claw", "flipper", "ramp"};
    for (const char l : lines) {
        CAPTURE(l);
        const auto line = static_cast<std::uint8_t>(l - 'A' + 1);
        REQUIRE(r.devices.digitalOutMade(line));
        CHECK(r.devices.digitalOutInitialState(line) == shulib::mechanism::words::Low);
    }
    const shulib::mechanism::Control* toggles[3] = {};
    for (int i = 0; i < 3; ++i) {
        toggles[i] = controlFor(ControlKind::Toggle, names[i]);
        if (toggles[i] == nullptr) MESSAGE("the team's map has no toggle on the " << names[i]);
    }
    // piston i: pressed at 20 + 80 i, seen at 40 + 80 i; pressed again at 60 + 80 i, seen at 80 + 80 i
    bool seen[3][2][3] = {};
    bool start[3] = {true, true, true};
    r.run(
        260,
        [&](int k) {
            for (int i = 0; i < 3; ++i) {
                if (toggles[i] == nullptr) continue;
                const int a = 20 + 80 * i;
                press(toggles[i]->button, (k >= a && k < a + 5) || (k >= a + 40 && k < a + 45));
            }
        },
        [&](int k) {
            if (k == 10) for (int j = 0; j < 3; ++j) start[j] = r.line(lines[j]);
            for (int i = 0; i < 3; ++i) {
                for (int p = 0; p < 2; ++p) {
                    if (k == 40 + 80 * i + 40 * p) for (int j = 0; j < 3; ++j) seen[i][p][j] = r.line(lines[j]);
                }
            }
        });
    for (int j = 0; j < 3; ++j) CHECK_FALSE(start[j]);  // Low: the unpowered position, at the first start too
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

TEST_CASE("robot one driver (D6, ROB1-C2): robot one's six mechanisms print the COMPACT row -- MECH, each motor by port, each piston by name -- and it fits the SMALL budget") {
    // BUG CAUGHT: robot one's row beginning CLAW; robot one's full row (98 characters for six mechanisms) cut
    // off at 50 on the screen; a mechanism missing from the compact row; a piston shown in a position its lines
    // are not at.
    OneRig r;
    const sb::ClawRow one = sb::clawRowText(sb::robot_one::kRobotOneMechanisms, *r.robot, *r.mechanisms, false,
                                            sb::WordsOf<sb::RobotId::RobotOne>{});
    CAPTURE(one.text);
    const std::string_view row{one.text};
    CHECK(sb::clawRowWorstLength(sb::robot_one::kRobotOneMechanisms) > shulib::calibration::kSmallCols);
    CHECK(sb::compactRowWorstLength(sb::robot_one::kRobotOneMechanisms) <= shulib::calibration::kSmallCols);
    CHECK(sb::mechanismRowWorstLength(sb::robot_one::kRobotOneMechanisms) ==
          sb::compactRowWorstLength(sb::robot_one::kRobotOneMechanisms));
    CHECK(row.starts_with("MECH " + std::to_string(facts::kCascadePorts.value().first) + "H"));
    CHECK(row.find("CLAW ") == std::string_view::npos);  // never the claw row's lead word
    CHECK(row.find(" " + std::to_string(facts::kRightTogglePort.value()) + "B") != std::string_view::npos);
    CHECK(row.find(" " + std::to_string(facts::leftTogglePort()) + "B") != std::string_view::npos);
    CHECK(row.find(" CLAW:C") != std::string_view::npos);  // its unpowered position on the placeholders: closed
    CHECK(row.find(" FLIP:C") != std::string_view::npos);
    CHECK(row.find(" RAMP:U") != std::string_view::npos);
    CHECK(row.size() <= sb::compactRowWorstLength(sb::robot_one::kRobotOneMechanisms) - 6);  // no FREED here
    const sb::ClawRow freed = sb::clawRowText(sb::robot_one::kRobotOneMechanisms, *r.robot, *r.mechanisms, true,
                                              sb::WordsOf<sb::RobotId::RobotOne>{});
    CHECK(std::string_view{freed.text}.ends_with(" FREED"));
    CHECK(std::string_view{freed.text}.size() <= shulib::calibration::kSmallCols);
    // A list whose full row FITS still prints the full row on robot one (ROB1-A's form — the cascade alone).
    static constexpr sb::MechanismTable kCascadeOnly = shulib::mechanism::mechanisms(shulib::mechanism::wire(
        sb::robot_one::cascade,
        shulib::mechanism::motors(facts::kCascadePorts.value().first, facts::kCascadePorts.value().second)));
    CHECK(sb::mechanismRowWorstLength(kCascadeOnly) == sb::clawRowWorstLength(kCascadeOnly));
}

TEST_CASE("robot one driver (D6, ROB1-C2 review): the compact row says F while R1 drives the cascade and R while R2 does; a temperature past 99 C prints 99, below 0 prints 00, unreadable prints --; a list whose full row fits prints the FULL row, begun MECH") {
    // BUG CAUGHT: the row telling the driver "10R" while the cascade goes UP (the F / R letters swapped — the
    // review's RV3); a hot motor printed with three digits or a cold one negative (the clamp gone — RV4); an
    // unreadable temperature printed as a number; robot one's FULL row always compacted (RV1) or begun CLAW
    // (the row word's copy gone — RV2: ROB1-A's GM14 again). All survived the suite before this case.
    OneRig r;
    const auto& list = sb::robot_one::kRobotOneMechanisms;
    const int cas = facts::kCascadePorts.value().first;
    const int right = facts::kRightTogglePort.value();
    const int left = facts::leftTogglePort();
    // The temperatures are set just before each row is drawn and put back after, so no protection ever
    // acts on them (the row only reads them).
    const auto rowWith = [&](double casC, double rightC, double leftC) {
        r.devices.fakeMotor(static_cast<std::uint8_t>(cas)).setTemperature(casC);
        r.devices.fakeMotor(static_cast<std::uint8_t>(right)).setTemperature(rightC);
        r.devices.fakeMotor(static_cast<std::uint8_t>(left)).setTemperature(leftC);
        const std::string row{sb::clawRowText(list, *r.robot, *r.mechanisms, false,
                                              sb::WordsOf<sb::RobotId::RobotOne>{}).text};
        for (const int p : {cas, right, left}) {
            r.devices.fakeMotor(static_cast<std::uint8_t>(p)).setTemperature(30.0);
        }
        return row;
    };
    std::string up;
    std::string down;
    // Past the driver-start home's bound (tick 320), the cascade's up button is held, then its down button
    // (the team's, from kControls -- §6.4).
    const auto* jog = controlFor(ControlKind::Hold, "cascade");
    const bool twoWay = jog != nullptr && !jog->oneWay;
    r.run(
        400,
        [&](int k) {
            if (!twoWay) return;
            press(jog->button, k >= 330 && k < 360);
            press(jog->back, k >= 365 && k < 395);
        },
        [&](int k) {
            if (k == 350) up = rowWith(120.0, -5.0, std::nan(""));
            if (k == 385) down = rowWith(99.6, 0.4, 41.0);
        });
    MESSAGE("robot one's compact row: R1 held '" << up << "', R2 held '" << down << "'");
    if (twoWay) {
        CHECK(up.find(" " + std::to_string(cas) + "F99") != std::string::npos);    // UP: F; 120 C: 99
        CHECK(down.find(" " + std::to_string(cas) + "R99") != std::string::npos);  // DOWN: R; 99.6 C (past 99): 99
    }
    CHECK(up.find(" " + std::to_string(right) + "B00") != std::string::npos);  // -5 C: 00
    CHECK(up.find(" " + std::to_string(left) + "B--") != std::string::npos);   // unreadable: --
    CHECK(down.find(" " + std::to_string(right) + "B00") != std::string::npos);  // 0.4 C rounds to 00
    CHECK(down.find(" " + std::to_string(left) + "B41") != std::string::npos);
    // A list whose full row FITS (the cascade alone) prints the FULL row on robot one: begun MECH, and from its
    // fifth character on byte for byte the claw-words row (ROB1-A's pin).
    static constexpr sb::MechanismTable kCascadeOnly = shulib::mechanism::mechanisms(shulib::mechanism::wire(
        sb::robot_one::cascade,
        shulib::mechanism::motors(facts::kCascadePorts.value().first, facts::kCascadePorts.value().second)));
    const std::string mechRow{
        sb::clawRowText(kCascadeOnly, *r.robot, *r.mechanisms, false, sb::WordsOf<sb::RobotId::RobotOne>{}).text};
    const std::string clawRow{sb::clawRowText(kCascadeOnly, *r.robot, *r.mechanisms, false).text};
    CAPTURE(mechRow);
    CAPTURE(clawRow);
    CHECK(mechRow.starts_with("MECH "));
    CHECK(clawRow.starts_with("CLAW "));
    CHECK(mechRow.substr(4) == clawRow.substr(4));
    CHECK(mechRow.size() > 8);
}

TEST_CASE("robot one driver (ROB1-C2): the boot line names a side toggle's ONE button, one way -- never a back button it does not have") {
    // BUG CAUGHT: the boot printing "hold-to-run L1 / L1" (the one-button hold's repeated button read as a back
    // button) — the driver would look for a second button that does nothing.
    const auto& list = sb::robot_one::kRobotOneMechanisms;
    int seen = 0;
    for (const shulib::mechanism::Control& c : sb::robot_one::kControls.all()) {
        if (c.kind != ControlKind::Hold || !c.oneWay) continue;
        std::size_t i = 0;
        while (i < list.count && std::string_view{list.specs[i].name} != c.mechanismName) ++i;
        REQUIRE(i < list.count);
        const std::string text = sb::mechanismControlLine(list, i, c, true).text;
        CAPTURE(text);
        const std::string want = std::string{c.mechanismName} + ": hold-to-run " +
                                 shulib::teleop::controllerButtonWord(c.button) + " (one way), " +
                                 std::to_string(std::lround(c.volts)) + " V";
        CHECK(text == want);
        ++seen;
    }
    MESSAGE("the team's map has " << seen << " one-button hold(s)");
    // The map is the team's (§6.4), so the one-button line is also checked on one made here: L1 on the right
    // toggle at 12 V.
    shulib::mechanism::Control one{};
    one.kind = ControlKind::Hold;
    one.oneWay = true;
    one.button = ControllerButton::L1;
    one.back = ControllerButton::L1;
    one.volts = 12.0;
    one.mechanismName = "right toggle";
    std::size_t rightAt = 0;
    while (rightAt < list.count && std::string_view{list.specs[rightAt].name} != "right toggle") ++rightAt;
    REQUIRE(rightAt < list.count);
    CHECK(std::string{sb::mechanismControlLine(list, rightAt, one, true).text} ==
          "right toggle: hold-to-run L1 (one way), 12 V");
    // ...and a two-button hold keeps its two buttons on its line (the team's, from kControls).
    const auto* jog = controlFor(ControlKind::Hold, "cascade");
    if (jog != nullptr && !jog->oneWay) {
        CHECK(std::string{sb::mechanismControlLine(list, 0, *jog, true).text} ==
              std::string{"cascade: hold-to-run "} + shulib::teleop::controllerButtonWord(jog->button) + " / " +
                  shulib::teleop::controllerButtonWord(jog->back) + ", " + std::to_string(std::lround(jog->volts)) +
                  " V");
    }
}
