// ROBOT ONE'S DEFAULT DRIVER FILE (chunk ROB1-A, 2026-11-11; the ROB1 brief's D7, test T11). Say "host test
// plant", never "simulator". Robot one does not exist in this project's records: its record below is
// INVENTED (a tank on ports nobody has read), its cascade is its real declaration on the ledger's
// PLACEHOLDERS. Run over the FAKE provider, with the controller read through the PROS shim (robot two's
// driver test's FakeRig pattern). Nothing here has run on a robot.
//
// WHAT THIS FILE TRIES TO BREAK
//   * THE BANNER IS THE FILE (GM22): every button src/team/robot_one_driver.hpp's banner names does exactly
//     what the file's kControls says — a level's button swapped in the file but not the banner turns this red;
//     Y (the loop's re-zero) and R1 / R2 (held for the flipper, D6), X and A are FREE.
//   * ON THE HOST, through the real driver file: the cascade homes DOWN at the first driver start; L1 drives it
//     up and L2 down (never past its unhomed volts before it is homed); L1 + L2 together = neither; Y moves
//     nothing; B homes it (DOWN) again.
//   * THE ROW (D6): the Drive screen's mechanism row reads `MECH` on robot one and fits the SMALL budget —
//     for today's list and for each of the three flipper forms (test/robot_one_flipper_forms_example_test.cpp).

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

/// An INVENTED robot-one record: a tank on ports 1-3 (left) and 4-6 (right), IMU on 12 — clear of the
/// cascade's placeholder ports. Nothing about robot one's drive is known (src/robots/chassis_table.hpp).
shulib::hal::DrivetrainDescription inventedRecord() {
    shulib::hal::DrivetrainDescription d{};
    d.kind = shulib::hal::DrivetrainKind::Tank;
    d.cartridge = shulib::hal::MotorCartridge::Blue;
    d.imuPort = 12;
    const std::int8_t left[3] = {-1, 2, -3};
    const std::int8_t right[3] = {4, -5, 6};
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
    std::snprintf(d.provenance, sizeof d.provenance, "%s", "robot one driver test (INVENTED)");
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
        for (const int p : {1, 2, 3, 4, 5, 6, facts::kCascadePorts.value().first, facts::kCascadePorts.value().second}) {
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

TEST_CASE("robot one driver (T11): the default map -- L1/L2 the cascade, the d-pad its first four levels BY THE LEDGER'S NAMES, B home, a driver-start home; Y, R1, R2, X, A free") {
    // BUG CAUGHT: a level bound by a typed string instead of the ledger's name (renaming a level would then
    // break the driver file), a level on the wrong button, Y taken (the loop's re-zero would fight a
    // control), R1/R2 taken before the flipper's kind is known.
    const auto* hold = controlOn(ControllerButton::L1);
    REQUIRE(hold != nullptr);
    CHECK(hold->kind == ControlKind::Hold);
    CHECK(hold->button == ControllerButton::L1);
    CHECK(hold->back == ControllerButton::L2);
    CHECK(hold->volts == 12.0);
    CHECK(std::string_view{hold->mechanismName} == "cascade");
    const ControllerButton dpad[4] = {ControllerButton::Down, ControllerButton::Left, ControllerButton::Up,
                                      ControllerButton::Right};
    for (std::size_t i = 0; i < 4; ++i) {
        CAPTURE(i);
        const auto* p = controlOn(dpad[i]);
        REQUIRE(p != nullptr);
        CHECK(p->kind == ControlKind::Preset);
        CHECK(std::string_view{p->presetName} == facts::lv(i).name);
        CHECK(p->value == facts::lv(i).at.value());
    }
    const auto* home = controlOn(ControllerButton::B);
    REQUIRE(home != nullptr);
    CHECK(home->kind == ControlKind::HomeOnPress);
    int starts = 0;
    for (const auto& c : sb::robot_one::kControls.all()) {
        starts += c.kind == ControlKind::HomeAtDriverStart ? 1 : 0;
    }
    CHECK(starts == 1);
    for (const ControllerButton b : {ControllerButton::Y, ControllerButton::R1, ControllerButton::R2,
                                     ControllerButton::X, ControllerButton::A}) {
        CAPTURE(static_cast<int>(b));
        CHECK(sb::robot_one::kControls.avoids(b));
    }
    CHECK(RobotOneDriver::kLoop.rezeroButton == ControllerButton::Y);
}

TEST_CASE("robot one driver (T11, GM22): the BANNER says what the FILE does -- every d-pad level, L1/L2 and B as the banner names them") {
    // BUG CAUGHT (GM22): a level's button swapped in the file but not the banner — the students read the
    // banner; the robot runs the file.
    const std::string text = source_pins::readFile("src/team/robot_one_driver.hpp");
    const struct {
        const char* banner;
        ControllerButton button;
        std::size_t slot;
    } levels[4] = {{"d-pad Down   the cascade to its FIRST level  (the facts ledger's level slot 0", ControllerButton::Down, 0},
                   {"d-pad Left   the cascade to its SECOND level (slot 1", ControllerButton::Left, 1},
                   {"d-pad Up     the cascade to its THIRD level  (slot 2", ControllerButton::Up, 2},
                   {"d-pad Right  the cascade to its FOURTH level (slot 3", ControllerButton::Right, 3}};
    for (const auto& l : levels) {
        CAPTURE(l.banner);
        CHECK(text.find(l.banner) != std::string::npos);
        const auto* p = controlOn(l.button);
        REQUIRE(p != nullptr);
        CHECK(std::string_view{p->presetName} == facts::lv(l.slot).name);
    }
    // ...and the FILE's lines say the same slots on the same buttons (a swap in the code alone is caught
    // above by the control; a swap in the code's comment alone here).
    const std::string code = source_pins::squash(source_pins::stripComments(text));
    CHECK(source_pins::count(code, "preset(cascade[facts::lv(0).name], Button::Down, 2_s)") == 1);
    CHECK(source_pins::count(code, "preset(cascade[facts::lv(1).name], Button::Left, 2_s)") == 1);
    CHECK(source_pins::count(code, "preset(cascade[facts::lv(2).name], Button::Up, 2_s)") == 1);
    CHECK(source_pins::count(code, "preset(cascade[facts::lv(3).name], Button::Right, 2_s)") == 1);
    CHECK(text.find("L1 / L2      the cascade up / down at full power while held") != std::string::npos);
    CHECK(text.find("B            home the cascade") != std::string::npos);
    CHECK(text.find("Y            field-centric re-zero") != std::string::npos);
    CHECK(text.find("A DEFAULT") != std::string::npos);  // the team's to change (D7)
}

// ═══ on the host, through the real driver file ═════════════════════════════════════════════

TEST_CASE("robot one driver (T11): on the host -- homes DOWN at the first start; L1 up, L2 down (within its unhomed volts), both = neither; Y moves nothing; B homes DOWN") {
    // BUG CAUGHT: a cascade that does not home at the first driver start (every level button would be dead
    // after a restart), L1 driving it DOWN, both triggers fighting, Y moving a mechanism, B pushing UP.
    OneRig r;
    double firstHome = 0.0;
    double up = 0.0;
    double down = 0.0;
    double both = 99.0;
    double withY = 99.0;
    double bHome = 0.0;
    const double unhomed = facts::kCascadeUnhomedVolts.value();
    // The schedule (10 ms ticks): 0-4 the driver-start home; past its 3 s bound (tick 320) L1, then L2,
    // then both, then nothing with Y, then B.
    r.run(
        620,
        [](int k) {
            const auto in = [k](int a, int b) { return k >= a && k < b; };
            press(ControllerButton::L1, in(330, 360) || in(400, 430));
            press(ControllerButton::L2, in(365, 395) || in(400, 430));
            press(ControllerButton::Y, in(440, 470));
            press(ControllerButton::B, in(500, 502));
        },
        [&](int k) {
            if (k == 3) firstHome = r.cascade1();
            if (k == 350) up = r.cascade1();
            if (k == 385) down = r.cascade1();
            if (k == 425) both = r.cascade1();
            if (k == 465) withY = r.cascade1();
            if (k == 510) bHome = r.cascade1();
        });
    MESSAGE("robot one on the host: home " << firstHome << " V, L1 " << up << " V, L2 " << down << " V, both " << both
                                           << " V, Y " << withY << " V, B " << bHome << " V");
    CHECK(firstHome < 0.0);  // DOWN, at the first driver start
    CHECK(firstHome >= facts::kCascadeHomingPushVolts.value() - 1e-9);
    CHECK(up > 0.0);
    CHECK(up <= unhomed + 1e-9);  // never past its unhomed volts before it is homed
    CHECK(down < 0.0);
    CHECK(down >= -unhomed - 1e-9);
    CHECK(both == doctest::Approx(0.0));
    CHECK(withY == doctest::Approx(0.0));
    CHECK(bHome < 0.0);  // B homes DOWN
    // The second motor gets the same command (its mirroring is its adapter's, once — never the file's).
    CHECK(std::abs(r.cascade2()) == doctest::Approx(std::abs(r.cascade1())));
}

TEST_CASE("robot one driver (D6): the Drive screen's mechanism row says MECH on robot one, CLAW on robot two's words, and fits the SMALL budget") {
    // BUG CAUGHT (GM14's robot one half): robot one's row beginning CLAW; a row that runs past the SMALL
    // budget on robot one's list.
    OneRig r;
    const sb::ClawRow one = sb::clawRowText(sb::robot_one::kRobotOneMechanisms, *r.robot, *r.mechanisms, false,
                                            sb::WordsOf<sb::RobotId::RobotOne>{});
    CAPTURE(one.text);
    CHECK(std::string_view{one.text}.starts_with("MECH " + std::to_string(facts::kCascadePorts.value().first) + " "));
    CHECK(std::string_view{one.text}.find("CLAW") == std::string_view::npos);
    const sb::ClawRow asTwo = sb::clawRowText(sb::robot_one::kRobotOneMechanisms, *r.robot, *r.mechanisms, false);
    CHECK(std::string_view{asTwo.text}.starts_with("CLAW "));  // the default words are robot two's (the host is the bench build)
    CHECK(std::string_view{one.text}.substr(4) == std::string_view{asTwo.text}.substr(4));
    CHECK(sb::clawRowWorstLength(sb::robot_one::kRobotOneMechanisms) <= shulib::calibration::kSmallCols);
}
