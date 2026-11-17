// driver_heading_test.cpp — THE heading rule every driver-control program uses
// (src/common/driver_heading.hpp's driverHeading()), over the fake provider (chunk T4 Part 1a,
// 2026-09-27, in test/driver_controls_test.cpp; MOVED here by the mechanism core's MECH-A5,
// 2026-10-29, when src/common/driver_controls.hpp retired — the MECH brief's re-check T6).
//
// ═══ WHAT THIS FILE PINS ════════════════════════════════════════════════════════════
//   * driverHeading() with ONE of its three terms flipped at a time (T7, unchanged from its old home);
//   * that the rule has ONE copy left in src/, in its own header, and that the X-drive and the shared
//     driver file (src/common/driver_file.hpp, which every robot's driver file runs) each call it once
//     (a source pin);
//   * that src/common/driver_controls.hpp is GONE and no src/ file includes it (the retired header's
//     grep, the MECH brief's §3.5).
// What test/driver_controls_test.cpp held besides — HoldControl / ToggleControl / DriverBindings and
// their five predicates — was DELETED with those types: their rules are mechanism::controls(...)'s
// build checks now (test/mechanism_driver_loop_test.cpp and test/compile_refusals/d_*, MECH-A4) and
// DriverFile's static_asserts (test/compile_refusals/n56_*, n57_*; and since RMP1-B its feel check, rm01_*).
//
// NO HARDWARE CLAIM: fakes only.

#include "mechanism_fixtures.hpp"
#include "doctest.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>

#include "common/boot_source.hpp"
#include "common/driver_heading.hpp"
#include "robots/mechanism_table.hpp"
#include "shulib/diag/fault.hpp"
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

using shulib::bench::driverHeading;
using shulib::localization::Localizer;
using namespace shulib::units::literals;

namespace {

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
    std::snprintf(d.provenance, sizeof d.provenance, "%s", "driver heading test (the 09-25 record)");
    return d;
}

struct NoPace final : shulib::motion::ITickPacer {
    void pace() override {}
};

/// Robot two built from `table` over the fake provider: every drive port, both pivots and the IMU
/// in the census; the IMU ready (a fresh FakeImu is), nothing ticked yet.
struct FakeRobotTwo {
    explicit FakeRobotTwo(const shulib::bench::MechanismTable& table = mech_fixture::robot_two::kRobotTwoMechanisms) {
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

TEST_CASE("driver heading T7: driverHeading() trusts the heading only when the estimate is live, "
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
TEST_CASE("driver heading: the rule has ONE copy in src/ — src/common/driver_heading.hpp — and the "
          "X-drive and the shared driver file each call it once; driver_controls.hpp is GONE") {
    // BUG CAUGHT: a second copy of the rule creeping back into a program file (the two programs
    // carried one each until T4 Part 1a; neither program file compiles on the host, so a copy there
    // would drift untested), a driver file calling it itself (every robot's driver file would then
    // carry plumbing — the MECH brief's N6), or the retired src/common/driver_controls.hpp coming back
    // (its HoldControl / ToggleControl were a second, unchecked way to bind a claw).
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
    std::size_t includesRetired = 0;
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
        includesRetired += count(s, "#include \"common/driver_controls.hpp\"");
    }
    CHECK(ruleCopies == 1U);
    CHECK(ruleFile == "common/driver_heading.hpp");
    CHECK(oldName == 0U);
    CHECK(includesRetired == 0U);
    CHECK_FALSE(fs::exists(root / "common" / "driver_controls.hpp"));
    const auto read = [&root](const char* rel) {
        std::ifstream in{root / rel};
        REQUIRE(in.good());
        std::stringstream text;
        text << in.rdbuf();
        return text.str();
    };
    // "shulib Drive" and the competition program reach it only through the driver file's run(), and a
    // team's driver file names it nowhere: the shared DriverFile calls it.
    CHECK(count(read("programs/drive_program.cpp"), "driverHeading(") == 0U);
    CHECK(count(read("programs/competition_program.cpp"), "driverHeading(") == 0U);
    CHECK(count(read("team/robot_two_driver.hpp"), "driverHeading(") == 0U);
    CHECK(count(read("programs/xdrive_program.cpp"), "[&r] { return driverHeading(r); }") == 1U);
    CHECK(count(read("common/driver_file.hpp"), "[&robot] { return driverHeading(robot); }") == 1U);
    CHECK(count(read("programs/xdrive_program.cpp"), "#include \"common/driver_heading.hpp\"") == 1U);
}
