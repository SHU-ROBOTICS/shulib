// ROBOT ONE'S DRIVER FILE (chunk ROB1-A, 2026-11-11; the ROB1 brief's D7, test T11; the builder's map since
// ROB1-C2, 2026-11-15). Say "host test plant", never "simulator". Its record below is INVENTED (a tank on ports
// nobody has read — Calibrate discovers robot one's drive), its mechanisms are their real declarations on the
// facts file's values — typed, or PLACEHOLDERS where still owed. Run over the FAKE provider, with the
// controller read through the PROS shim (robot two's driver test's FakeRig pattern). Nothing here has run on a
// robot.
//
// WHAT THIS FILE TRIES TO BREAK
//   * THE BUILDER'S MAP, AS WRITTEN (T11, re-pinned at ROB1-C2 — it pins the RULE, the builder's words of
//     2026-11-15): R1 / R2 the cascade while held (R1 up); A the claw, B the front toggle (the flipper), X the
//     ramp — each a toggle; L1 the RIGHT side toggle and L2 the LEFT, each a ONE-button hold (no back button);
//     one driver-start home; Y (the loop's re-zero) and the whole d-pad FREE; no preset, no other control.
//   * THE BANNER IS THE FILE (GM22): every button the driver file's [the map] block names does what kControls
//     says — a button swapped in the file but not the banner turns this red.
//   * ON THE HOST, through the real driver file: the cascade homes DOWN at the first driver start; R1 drives it
//     up and R2 down (never past its unhomed volts before it is homed); R1 + R2 together = neither; Y moves
//     nothing; L1 runs the RIGHT toggle only, L2 the LEFT only, each ONE way and stopped on release; A, B and X
//     each flip ITS piston and no other; every piston's line is MADE at Low (nothing moves at power-up).
//   * THE ROW (D6, ROB1-C2): robot one's six mechanisms print the COMPACT row (MECH ...), within the SMALL budget.

#include "doctest.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <string>
#include <string_view>

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

TEST_CASE("robot one driver (T11): the BUILDER'S map -- R1/R2 the cascade, A the claw, B the front toggle, X the ramp, L1 the RIGHT side toggle and L2 the LEFT (one button each), a driver-start home; Y and the d-pad free") {
    // BUG CAUGHT: a button the builder named doing something else (the claw on B, the ramp on A); the cascade's
    // up on R2; a side toggle given a second button (its "back" would steal one the builder left free) or swapped
    // side; a preset left on the d-pad (the builder: "for now it will be manual"); Y taken (the loop's re-zero
    // would fight a control); a second home control the builder never asked for.
    const auto* cascade = controlOn(ControllerButton::R1);
    REQUIRE(cascade != nullptr);
    CHECK(cascade->kind == ControlKind::Hold);
    CHECK(std::string_view{cascade->mechanismName} == "cascade");
    CHECK(cascade->button == ControllerButton::R1);  // R1 = UP (+volts)
    CHECK(cascade->back == ControllerButton::R2);    // R2 = DOWN
    CHECK_FALSE(cascade->oneWay);
    CHECK(cascade->volts == 12.0);
    CHECK(controlOn(ControllerButton::R2) == cascade);
    const struct {
        ControllerButton button;
        const char* mechanism;
    } toggles[3] = {{ControllerButton::A, "claw"}, {ControllerButton::B, "flipper"}, {ControllerButton::X, "ramp"}};
    for (const auto& t : toggles) {
        CAPTURE(t.mechanism);
        const auto* c = controlOn(t.button);
        REQUIRE(c != nullptr);
        CHECK(c->kind == ControlKind::Toggle);
        CHECK(std::string_view{c->mechanismName} == t.mechanism);
    }
    const struct {
        ControllerButton button;
        const char* mechanism;
    } sides[2] = {{ControllerButton::L1, "right toggle"}, {ControllerButton::L2, "left toggle"}};
    for (const auto& t : sides) {
        CAPTURE(t.mechanism);
        const auto* c = controlOn(t.button);
        REQUIRE(c != nullptr);
        CHECK(c->kind == ControlKind::Hold);
        CHECK(c->oneWay);  // ONE button: it runs one way, and takes no back button
        CHECK(c->button == t.button);
        CHECK(std::string_view{c->mechanismName} == t.mechanism);
        CHECK(c->volts == 12.0);
        CHECK(c->mechanismKind == shulib::mechanism::Kind::Power);
    }
    // ...the right toggle's mechanism is wired to the RIGHT toggle's port fact, the left to the other.
    const auto& list = sb::robot_one::kRobotOneMechanisms;
    for (std::size_t i = 0; i < list.count; ++i) {
        if (std::string_view{list.specs[i].name} == "right toggle") {
            CHECK(list.wiring[i].motors[0] == facts::kRightTogglePort.value());
        }
        if (std::string_view{list.specs[i].name} == "left toggle") {
            CHECK(list.wiring[i].motors[0] == facts::leftTogglePort());
        }
    }
    const auto* home = controlFor(ControlKind::HomeAtDriverStart, "cascade");
    REQUIRE(home != nullptr);
    CHECK(home->bound == 3.0);
    int presets = 0;
    int homes = 0;
    for (const auto& c : sb::robot_one::kControls.all()) {
        presets += c.kind == ControlKind::Preset ? 1 : 0;
        homes += (c.kind == ControlKind::HomeAtDriverStart || c.kind == ControlKind::HomeOnPress) ? 1 : 0;
    }
    CHECK(presets == 0);  // manual for now: no preset anywhere
    CHECK(homes == 1);    // the one driver-start home, nothing on a button
    CHECK(sb::robot_one::kControls.count == 7);
    for (const ControllerButton b : {ControllerButton::Y, ControllerButton::Up, ControllerButton::Down,
                                     ControllerButton::Left, ControllerButton::Right}) {
        CAPTURE(static_cast<int>(b));
        CHECK(sb::robot_one::kControls.avoids(b));
        CHECK(controlOn(b) == nullptr);
    }
    CHECK(RobotOneDriver::kLoop.rezeroButton == ControllerButton::Y);
}

TEST_CASE("robot one driver (T11, GM22): the BANNER's [the map] says what the FILE does -- every button as the builder named it") {
    // BUG CAUGHT (GM22): a button swapped in the file but not the banner — the students read the banner; the
    // robot runs the file. Each banner line and each code line is pinned, once.
    const std::string text = source_pins::readFile("src/team/robot_one_driver.hpp");
    CHECK(source_pins::count(text, "// [the map]") == 1);
    for (const char* banner : {"//   R1           HOLD: the cascade goes UP; let go: it stops and HOLDS where it is",
                               "//   R2           HOLD: the cascade goes DOWN; let go: it stops and holds",
                               "//   A            the claw: press = open, press again = close",
                               "//   B            the front toggle (the flipper): press = open (out), press again = close",
                               "//   X            the ramp: press = down (the builder's \"open\"), press again = up",
                               "//   L1           HOLD: the RIGHT side toggle runs (one way only); let go: it stops",
                               "//   L2           HOLD: the LEFT side toggle runs (one way only); let go: it stops",
                               "//   Y            field-centric re-zero",
                               "//   d-pad        FREE",
                               "//   (start)      the FIRST time driver control starts in a run, the cascade HOMES"}) {
        CAPTURE(banner);
        CHECK(source_pins::count(text, banner) == 1);
    }
    const std::string code = source_pins::squash(source_pins::stripComments(text));
    for (const char* line : {"whileHeld(cascade, Button::R1, Button::R2, 12_volt)", "toggle(claw, Button::A)",
                             "toggle(flipper, Button::B)", "toggle(ramp, Button::X)",
                             "whileHeld(rightToggle, Button::L1, 12_volt)", "whileHeld(leftToggle, Button::L2, 12_volt)",
                             "homeAtDriverStart(cascade, 3_s)"}) {
        CAPTURE(line);
        CHECK(source_pins::count(code, line) == 1);
    }
    CHECK(source_pins::count(code, "preset(") == 0);
    CHECK(source_pins::count(code, "homeOnPress(") == 0);
    // the ONE line the team may delete is marked as such
    CHECK(source_pins::count(text, "homeAtDriverStart(cascade, 3_s));                      // [delete-able]") == 1);
}

// ═══ on the host, through the real driver file ═════════════════════════════════════════════

TEST_CASE("robot one driver (T11): on the host -- homes DOWN at the first start; R1 up, R2 down (within its unhomed volts), both = neither; Y moves nothing; L1 runs ONLY the right toggle, L2 ONLY the left, each stopped on release") {
    // BUG CAUGHT: a cascade that does not home at the first driver start; R1 driving it DOWN; both triggers
    // fighting; Y moving a mechanism; L1 running the LEFT toggle (or both); a side toggle that keeps running
    // after its button is let go; a side toggle driven backward by a button the builder never gave it.
    OneRig r;
    double firstHome = 0.0;
    double up = 0.0;
    double down = 0.0;
    double both = 99.0;
    double withY = 99.0;
    double l1Right = 0.0, l1Left = 99.0, l1Released = 99.0;
    double l2Right = 99.0, l2Left = 0.0, l2Released = 99.0;
    const double unhomed = facts::kCascadeUnhomedVolts.value();
    // The schedule (10 ms ticks): 0-4 the driver-start home; past its 3 s bound (tick 320) R1, then R2, then
    // both, then nothing with Y, then L1, then L2.
    r.run(
        600,
        [](int k) {
            const auto in = [k](int a, int b) { return k >= a && k < b; };
            press(ControllerButton::R1, in(330, 360) || in(400, 430));
            press(ControllerButton::R2, in(365, 395) || in(400, 430));
            press(ControllerButton::Y, in(440, 470));
            press(ControllerButton::L1, in(480, 510));
            press(ControllerButton::L2, in(530, 560));
        },
        [&](int k) {
            if (k == 3) firstHome = r.cascade1();
            if (k == 350) up = r.cascade1();
            if (k == 385) down = r.cascade1();
            if (k == 425) both = r.cascade1();
            if (k == 465) withY = r.cascade1();
            if (k == 500) { l1Right = r.right(); l1Left = r.left(); }
            if (k == 520) l1Released = r.right();
            if (k == 550) { l2Right = r.right(); l2Left = r.left(); }
            if (k == 580) l2Released = r.left();
        });
    MESSAGE("robot one on the host: home " << firstHome << " V, R1 " << up << " V, R2 " << down << " V, both " << both
                                           << " V, Y " << withY << " V; L1 right " << l1Right << " left " << l1Left
                                           << "; L2 right " << l2Right << " left " << l2Left);
    CHECK(firstHome < 0.0);  // DOWN, at the first driver start
    CHECK(firstHome >= facts::kCascadeHomingPushVolts.value() - 1e-9);
    CHECK(up > 0.0);  // R1 = UP
    CHECK(up <= unhomed + 1e-9);  // never past its unhomed volts before it is homed
    CHECK(down < 0.0);  // R2 = DOWN
    CHECK(down >= -unhomed - 1e-9);
    CHECK(both == doctest::Approx(0.0));
    CHECK(withY == doctest::Approx(0.0));
    // The second motor gets the same command (its mirroring is its adapter's, once — never the file's).
    CHECK(std::abs(r.cascade2()) == doctest::Approx(std::abs(r.cascade1())));
    // L1: the RIGHT toggle at the full 12 V, the left untouched; let go: stopped. L2: the mirror.
    CHECK(l1Left == doctest::Approx(0.0));
    CHECK(l1Released == doctest::Approx(0.0));
    CHECK(l2Right == doctest::Approx(0.0));
    CHECK(l2Released == doctest::Approx(0.0));
    // ...ONE way each: the button commands +12 V ("run"), never -12 V; WHICH way that turns the motor is its
    // declaration's direction fact, applied ONCE, by the motor's SIGNED port (PROS reverses inside the adapter —
    // a host fake indexed by |port| reads the same command either way, so the sign is checked structurally, as
    // the provider's motorSignedPort() exists to). The two run mirror-image ways on the placeholders.
    CHECK(l1Right == doctest::Approx(12.0));
    CHECK(l2Left == doctest::Approx(12.0));
    const auto signOf = [&](int port) { return r.devices.motorSignedPort(static_cast<std::uint8_t>(port)); };
    CHECK((signOf(facts::kRightTogglePort.value()) < 0) == facts::kRightToggleDirection.value().reversed);
    CHECK((signOf(facts::leftTogglePort()) < 0) == facts::kLeftToggleDirection.value().reversed);
    CHECK(std::abs(signOf(facts::kRightTogglePort.value())) == facts::kRightTogglePort.value());
    CHECK(std::abs(signOf(facts::leftTogglePort())) == facts::leftTogglePort());
}

TEST_CASE("robot one driver (T11): on the host -- every piston's line is MADE at Low (nothing moves at power-up); A flips ONLY the claw, B ONLY the front toggle, X ONLY the ramp, each back on its second press") {
    // BUG CAUGHT: a piston that starts in its POWERED position (it would fire at power-up); A firing the ramp; a
    // toggle that does not come back on its second press; one press flipping two pistons.
    OneRig r;
    const char claw = facts::kClawLine.value();
    const char flip = facts::kFlipperLine.value();
    const char ramp = facts::kRampLine.value();
    for (const char l : {claw, flip, ramp}) {
        CAPTURE(l);
        const auto line = static_cast<std::uint8_t>(l - 'A' + 1);
        REQUIRE(r.devices.digitalOutMade(line));
        CHECK(r.devices.digitalOutInitialState(line) == shulib::mechanism::words::Low);
    }
    struct Seen {
        bool claw, flip, ramp;
    };
    Seen start{}, afterA{}, afterA2{}, afterB{}, afterX{};
    const auto seen = [&] { return Seen{r.line(claw), r.line(flip), r.line(ramp)}; };
    r.run(
        200,
        [](int k) {
            const auto in = [k](int a, int b) { return k >= a && k < b; };
            press(ControllerButton::A, in(20, 25) || in(60, 65));
            press(ControllerButton::B, in(100, 105));
            press(ControllerButton::X, in(140, 145));
        },
        [&](int k) {
            if (k == 10) start = seen();
            if (k == 40) afterA = seen();
            if (k == 80) afterA2 = seen();
            if (k == 120) afterB = seen();
            if (k == 160) afterX = seen();
        });
    CHECK_FALSE(start.claw);  // Low: the unpowered position, at the first driver start too
    CHECK_FALSE(start.flip);
    CHECK_FALSE(start.ramp);
    CHECK(afterA.claw);  // A: the claw, and only the claw
    CHECK_FALSE(afterA.flip);
    CHECK_FALSE(afterA.ramp);
    CHECK_FALSE(afterA2.claw);  // A again: back
    CHECK(afterB.flip);  // B: the front toggle only
    CHECK_FALSE(afterB.claw);
    CHECK_FALSE(afterB.ramp);
    CHECK(afterX.ramp);  // X: the ramp only
    CHECK(afterX.flip);  // (the front toggle stays where B put it)
    CHECK_FALSE(afterX.claw);
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
    // Past the driver-start home's 3 s bound (tick 320), R1 is held, then R2 (T11's schedule).
    r.run(
        400,
        [](int k) {
            press(ControllerButton::R1, k >= 330 && k < 360);
            press(ControllerButton::R2, k >= 365 && k < 395);
        },
        [&](int k) {
            if (k == 350) up = rowWith(120.0, -5.0, std::nan(""));
            if (k == 385) down = rowWith(99.6, 0.4, 41.0);
        });
    MESSAGE("robot one's compact row: R1 held '" << up << "', R2 held '" << down << "'");
    CHECK(up.find(" " + std::to_string(cas) + "F99") != std::string::npos);    // UP: F; 120 C: 99
    CHECK(down.find(" " + std::to_string(cas) + "R99") != std::string::npos);  // DOWN: R; 99.6 C (past 99): 99
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
                                 (c.button == ControllerButton::L1 ? "L1" : "L2") + " (one way), 12 V";
        CHECK(text == want);
        ++seen;
    }
    CHECK(seen == 2);
    // ...and the cascade's two-button hold keeps its two buttons on its line.
    CHECK(std::string{sb::mechanismControlLine(list, 0, sb::robot_one::kControls.c[0], true).text} ==
          "cascade: hold-to-run R1 / R2, 12 V");
}
