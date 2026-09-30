// xdrive_config_test.cpp — G1 Part 2c: the invented X-drive as DATA (src/robots/xdrive_config.hpp,
// included from src/ the way chassis_table_test.cpp includes src/robots/chassis_table.hpp — the
// host build compiles test/*.cpp and, of src/, only the team's PROS-free src/team/autons.cpp).
//
// ═══ WHAT THIS FILE PROVES, AND HOW IT TRIES TO BREAK IT ═════════════════════════════
//   * kInventedXDrive IS a constant expression — the tree's first constexpr RobotConfig. Proved
//     by static_asserts that read it (a `static const` would not compile here), including a
//     constexpr COPY of the whole value.
//   * It is the SAME ROBOT src/main.cpp hand-wired until Part 2c: every port, SIGN, cartridge,
//     radius, pod size and offset, and the GPS and its lever arm, against a TRANSCRIPTION of
//     main.cpp's constants at ea58fbc (typed here as literals, never read from the header under
//     test) — and the session header's port map against the text main.cpp's portMapString()
//     printed, formatted here from those transcribed constants with main.cpp's own format.
//   * The builder makes it into the graph main.cpp made — four single-motor wheels (no groups),
//     pods, the library's DEFAULT tuning untouched (main.cpp built `Chassis{deps, pacer}`), a
//     strafe authority of 1.00 (the old banner's "facade alive" check) — PLUS the one documented
//     difference: a GPS corrector (xdrive_config.hpp says why). Every motor is requested with its
//     sign AS STORED and the GREEN cartridge (the fake records both; a fake cannot show a sign by
//     behaving differently, so the sign is checked structurally).
//   * Through the PROS shim and the REAL hal::pros::ProsDeviceProvider: on robot two's census it
//     REFUSES at port 1 (the radio) and opens nothing; on the old comp bot's census it refuses at
//     port 1 (empty); on a brain carrying exactly the invented devices it builds, and a +6 V wheel
//     command reaches ports 3 and 4 NEGATED at the device — PROS's reversal, applied once.
//
// NO HARDWARE CLAIM: this X-drive build has never booted on a robot (the one X-drive boot on
// record, 2026-08-12, was the old hand-wired graph over FAKE devices). The shim tests the provider
// against our belief about PROS (HA-138's indexing, HA-141's kinds), and cannot test the belief.

#include "doctest.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <optional>
#include <string>

#include "robots/xdrive_config.hpp"
#include "pros/shim_control.hpp"
#include "shulib/chassis/chassis.hpp"
#include "shulib/config/robot_config.hpp"
#include "shulib/core/check.hpp"
#include "shulib/diag/session_info.hpp"
#include "shulib/hal/absent_gps.hpp"
#include "shulib/hal/drivetrain_description.hpp"
#include "shulib/hal/fake/fake_battery.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_device_provider.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/hal/pros/device_provider.hpp"
#include "shulib/robot/robot.hpp"
#include "shulib/units/quantity.hpp"

namespace {

namespace cfgns = shulib::config;
using shulib::bench::kInventedXDrive;
using shulib::hal::MotorCartridge;
using shulib::hal::fake::FakeBattery;
using shulib::hal::fake::FakeClock;
using shulib::hal::fake::FakeDeviceProvider;
using shulib::hal::fake::FakeTelemetrySink;
using shulib::hal::pros::ProsDeviceProvider;
using shulib::units::Voltage;

// ═══ 1. A CONSTANT EXPRESSION — proved at compile time, so this file does not build otherwise ═══
// Every read below is in a static_assert, which only a constant expression can satisfy: had the
// header declared `static const RobotConfig` (the brief's fallback), none of these would compile.
static_assert(kInventedXDrive.drivetrain.kind == cfgns::DrivetrainKind::XDrive,
              "kInventedXDrive must be readable at compile time");
static_assert(kInventedXDrive.drivetrain.wheelCount() == 4
                  && kInventedXDrive.drivetrain.driveMotorCount() == 4,
              "the F5 wheel-count cross-check, evaluated by the compiler");
/// The whole value copied in a constant expression — the form G3's generator will emit.
constexpr cfgns::RobotConfig kCopiedAtCompileTime = kInventedXDrive;
static_assert(kCopiedAtCompileTime.drivetrain.xdrive.ports[2] == -3
                  && kCopiedAtCompileTime.sensors.gps.port == 9
                  && kCopiedAtCompileTime.odometry.wheels.lateral.offset.value() == -4.5,
              "a constexpr copy carries the fields");

// ═══ THE TRANSCRIPTION — src/main.cpp at ea58fbc, typed by hand (never read from the header) ═══
//   :297-304  kFrontLeftPort = 1, kBackLeftPort = 2, kBackRightPort = -3, kFrontRightPort = -4,
//             kForwardEncoderPort = 5, kLateralEncoderPort = 6, kGpsPort = 9, kImuPort = 10
//   :308-309  kDriveGearset = MotorGearset::Green
//   :343-344  kinematics::xDrive(units::Length{7.0})
//   :361-362  ProsGps gps{kGpsPort, Length{0.0}, Length{0.0}}
//   :395-400  TrackingWheel::forward(forwardEncoder, Length{2.0}, Length{-3.0}),
//             TrackingWheel::lateral(lateralEncoder, Length{2.0}, Length{-4.5})
//   :424      Chassis chassis{deps, pacer}  — the default ChassisConfig
constexpr int kOldFrontLeft = 1, kOldBackLeft = 2, kOldBackRight = -3, kOldFrontRight = -4;
constexpr int kOldForwardEncoder = 5, kOldLateralEncoder = 6;
constexpr unsigned kOldGps = 9, kOldImu = 10;
constexpr double kOldDriveRadius = 7.0;
constexpr double kOldPodDiameter = 2.0, kOldForwardOffset = -3.0, kOldLateralOffset = -4.5;

/// main.cpp's portMapString(), transcribed: its format over its constants.
std::string oldPortMapString() {
    char buf[shulib::diag::kMaxPortMapBytes];
    std::snprintf(buf, sizeof buf, "FL%d BL%d BR%d FR%d ROTF%d ROTL%d GPS%u IMU%u", kOldFrontLeft,
                  kOldBackLeft, kOldBackRight, kOldFrontRight, kOldForwardEncoder,
                  kOldLateralEncoder, kOldGps, kOldImu);
    return buf;
}

struct NoopPacer final : shulib::motion::ITickPacer {
    void pace() override {}
};

/// A host world with no plant: the provider's own fakes on every port.
struct World {
    FakeClock clock{};
    FakeBattery battery{};
    FakeTelemetrySink sink{};
    NoopPacer pacer{};
    FakeDeviceProvider devices{clock, battery, sink, pacer};
};

}  // namespace

// ═══ 2. The same robot main.cpp wired, field by field ═════════════════════════════════
TEST_CASE("G1 2c xdrive 1: kInventedXDrive is main.cpp's invented X-drive, every field — signs, "
          "cartridge, geometry, pods, GPS — and says what it is") {
    // BUG CAUGHT: a number or a SIGN changed in the move from a hand-wired graph to data — the
    // one class of change that makes the config quietly describe a different robot.
    const cfgns::RobotConfig& c = kInventedXDrive;
    char why[192] = "";
    CHECK_MESSAGE(c.valid(why, sizeof why), why);

    REQUIRE(c.drivetrain.kind == cfgns::DrivetrainKind::XDrive);
    const cfgns::XDriveDrivetrain& x = c.drivetrain.xdrive;
    // x_drive.hpp's canonical wheel order: FL, BL, BR, FR.
    CHECK(x.ports[0] == kOldFrontLeft);
    CHECK(x.ports[1] == kOldBackLeft);
    CHECK(x.ports[2] == kOldBackRight);
    CHECK(x.ports[3] == kOldFrontRight);
    CHECK(x.cartridge == MotorCartridge::Green);
    CHECK(x.imuPort == kOldImu);
    CHECK(x.driveRadius.value() == kOldDriveRadius);

    REQUIRE(c.odometry.kind == cfgns::OdometryKind::TrackingWheels);
    CHECK(c.odometry.wheels.forward.port == kOldForwardEncoder);
    CHECK(c.odometry.wheels.forward.wheelDiameter.value() == kOldPodDiameter);
    CHECK(c.odometry.wheels.forward.offset.value() == kOldForwardOffset);
    CHECK(c.odometry.wheels.lateral.port == kOldLateralEncoder);
    CHECK(c.odometry.wheels.lateral.wheelDiameter.value() == kOldPodDiameter);
    CHECK(c.odometry.wheels.lateral.offset.value() == kOldLateralOffset);

    REQUIRE(c.sensors.gps.present);
    CHECK(c.sensors.gps.port == kOldGps);
    CHECK(c.sensors.gps.leverArmForward.value() == 0.0);
    CHECK(c.sensors.gps.leverArmLeft.value() == 0.0);

    CHECK(c.mechanismCount == 0);
    // The HA-111 label lives in the name (XDriveDrivetrain has no provenance field), whole — a
    // 32-byte field, so a longer label would have been cut, and the cut would drop the number.
    CHECK(std::string{c.identity.name} == "X-drive INVENTED ports (HA-111)");
    CHECK(c.identity.team[0] == '\0');
    CHECK(c.identity.season[0] == '\0');
}

// ═══ 3. The graph the builder makes of it ══════════════════════════════════════════════
TEST_CASE("G1 2c xdrive 2: robot::Robot builds it over a provider into main.cpp's graph — plus the "
          "GPS corrector — asking for every device as the config states it") {
    World w;
    const shulib::robot::BuildVerdict v = shulib::robot::validate(kInventedXDrive, w.devices);
    CAPTURE(v.why);
    REQUIRE(v.ok);
    std::optional<shulib::robot::Robot> robot;
    robot.emplace(kInventedXDrive, w.devices);
    shulib::robot::Robot& r = *robot;

    SUBCASE("every wheel motor asked for with its SIGN as stored and the GREEN cartridge") {
        // BUG CAUGHT: a sign dropped between the config and the adapter (the fake cannot show it
        // by behaving differently — it records it).
        const int stored[4] = {kOldFrontLeft, kOldBackLeft, kOldBackRight, kOldFrontRight};
        for (const int s : stored) {
            CAPTURE(s);
            const auto port = static_cast<std::uint8_t>(s < 0 ? -s : s);
            CHECK(w.devices.motorRequested(port));
            CHECK(static_cast<int>(w.devices.motorSignedPort(port)) == s);
            CHECK(w.devices.motorCartridge(port) == MotorCartridge::Green);
        }
        // …and nothing else: no motor on a sensor port, none on a port the config never names.
        for (std::uint8_t p = 5; p <= 21; ++p) {
            CAPTURE(static_cast<int>(p));
            CHECK(w.devices.motorRequestCount(p) == 0);
        }
        CHECK(w.devices.gpsLeverForward(static_cast<std::uint8_t>(kOldGps)).value() == 0.0);
    }
    SUBCASE("four single-motor wheels in the F5 order, no groups, pods, and the facade alive") {
        CHECK(r.motorGroupCount() == 0);
        CHECK(r.driveOdometry() == nullptr);  // tracking wheels, not drive encoders
        const auto motors = r.context().driveMotors();
        REQUIRE(motors.size() == 4);
        CHECK(motors[0] == &w.devices.fakeMotor(1));
        CHECK(motors[1] == &w.devices.fakeMotor(2));
        CHECK(motors[2] == &w.devices.fakeMotor(3));
        CHECK(motors[3] == &w.devices.fakeMotor(4));
        // main.cpp's boot banner: "facade alive: strafeAuthority=1.00 (X-drive: 1.00)".
        CHECK(r.chassis().strafeAuthority() == 1.0);
    }
    SUBCASE("the ONE difference from main.cpp's graph: a GPS corrector, and the GPS itself live") {
        CHECK(r.hasGpsCorrector());
        CHECK(dynamic_cast<shulib::hal::AbsentGps*>(&r.context().gps()) == nullptr);
    }
    SUBCASE("the library's DEFAULT tuning, untouched — main.cpp built Chassis{deps, pacer}") {
        // BUG CAUGHT: gains typed into the invented robot, or the builder's tank-only stall
        // wiring applied to a holonomic drivetrain. Compared with a default ChassisConfig field by
        // field, exactly (there is no operator==, and padding rules out a memcmp).
        const shulib::chassis::ChassisConfig defaults{};
        const shulib::motion::MotionConfig& got = r.chassis().motionConfig();
        CHECK(got.maxLinearSpeed.value() == defaults.motion.maxLinearSpeed.value());
        CHECK(got.maxAngularSpeed.value() == defaults.motion.maxAngularSpeed.value());
        CHECK(got.maxWheelSpeed.value() == defaults.motion.maxWheelSpeed.value());
        CHECK(got.wheelFf.kS == defaults.motion.wheelFf.kS);
        CHECK(got.wheelFf.kV == defaults.motion.wheelFf.kV);
        CHECK(got.wheelFf.kA == defaults.motion.wheelFf.kA);
        CHECK(got.stall.independentMotionSource == defaults.motion.stall.independentMotionSource);
        CHECK(kInventedXDrive.tuning.motion.maxLinearSpeed.value()
              == defaults.motion.maxLinearSpeed.value());
        CHECK(kInventedXDrive.tuning.scheduler.abortFaultMask == defaults.scheduler.abortFaultMask);
    }
    SUBCASE("the session header's port map is the text main.cpp's portMapString() printed") {
        // BUG CAUGHT: every session header of this build changing shape under a log reader.
        char map[shulib::diag::kMaxPortMapBytes] = "";
        r.portMap(map, sizeof map);
        CHECK(std::string{map} == oldPortMapString());
        CHECK(std::string{map} == "FL1 BL2 BR-3 FR-4 ROTF5 ROTL6 GPS9 IMU10");  // the literal
    }
    SUBCASE("the banner names it INVENTED, with its signed wheels, radius, pods and GPS") {
        char banner[320] = "";
        r.describe(banner, sizeof banner);
        CHECK(std::string{banner}
              == "X-drive INVENTED ports (HA-111) [] x-drive TYPED | wheels 1,2,-3,-4 | R 7.000 in | "
                 "IMU10 | tracking-wheels | GPS present tags ABSENT vision ABSENT");
        // The program logs it as "robot::Robot: <banner>" through TermSink, which cuts at 200.
        CHECK(std::strlen(banner) + std::strlen("robot::Robot: ") <= 200);
    }
}

// ═══ 4. Through the PROS shim and the REAL provider: it cannot boot on any robot that exists ═══
TEST_CASE("G1 2c xdrive 3: over the real ProsDeviceProvider it REFUSES on robot two and on the old "
          "comp bot, naming port 1 and opening nothing — and builds on a brain that has its "
          "devices, every sign reaching the device as stored") {
    pros::shim::resetAll();

    SUBCASE("robot two (radio on 1, IMU on 2, camera on 3, motors on 11-18 and 20)") {
        // The census Bench Tests read off robot two (R3d-PROGRESS, 2026-09-25): the sentence the
        // program would paint and print, word for word.
        pros::shim::setPluggedAtPort(1, pros::c::E_DEVICE_RADIO);
        pros::shim::setPluggedAtPort(2, pros::c::E_DEVICE_IMU);
        pros::shim::setPluggedAtPort(3, pros::c::E_DEVICE_AIVISION);
        for (const int p : {11, 12, 13, 14, 15, 16, 17, 18, 20}) {
            pros::shim::setPluggedAtPort(p, pros::c::E_DEVICE_MOTOR);
        }
        ProsDeviceProvider devices;
        const shulib::robot::BuildVerdict v = shulib::robot::validate(kInventedXDrive, devices);
        CHECK_FALSE(v.ok);
        CHECK(std::string{v.why}
              == "an X-drive wheel motor cannot be made: port 1: the radio (expected a motor)");
        // Nothing was opened: the refusal came from the registry, before any adapter.
        for (const int p : {1, 2, 3, 4}) {
            CAPTURE(p);
            CHECK(pros::shim::motorState(p).setGearingCalls == 0);
        }
        CHECK(pros::shim::imuState(10).resetCalls == 0);
        // And the constructor refuses by precondition with the same sentence — the graph
        // boundary's catch in src/programs/xdrive_program.cpp is what turns this into a screen.
        ProsDeviceProvider again;
        std::optional<shulib::robot::Robot> robot;
        CHECK_THROWS_WITH_AS(robot.emplace(kInventedXDrive, again),
                             "an X-drive wheel motor cannot be made: port 1: the radio (expected a "
                             "motor)",
                             shulib::PreconditionError);
    }
    SUBCASE("the old comp bot (HA-111's measurement: drive 11/12/14 and 15-18, IMU on 4)") {
        for (const int p : {11, 12, 14, 15, 16, 17, 18}) {
            pros::shim::setPluggedAtPort(p, pros::c::E_DEVICE_MOTOR);
        }
        pros::shim::setPluggedAtPort(4, pros::c::E_DEVICE_IMU);
        ProsDeviceProvider devices;
        const shulib::robot::BuildVerdict v = shulib::robot::validate(kInventedXDrive, devices);
        CHECK_FALSE(v.ok);
        CHECK(std::string{v.why}
              == "an X-drive wheel motor cannot be made: port 1: nothing plugged in (expected a "
                 "motor)");
    }
    SUBCASE("a brain carrying exactly the invented devices: it builds, and the signs land once") {
        for (const int p : {1, 2, 3, 4}) {
            pros::shim::setPluggedAtPort(p, pros::c::E_DEVICE_MOTOR);
        }
        pros::shim::setPluggedAtPort(5, pros::c::E_DEVICE_ROTATION);
        pros::shim::setPluggedAtPort(6, pros::c::E_DEVICE_ROTATION);
        pros::shim::setPluggedAtPort(9, pros::c::E_DEVICE_GPS);
        pros::shim::setPluggedAtPort(10, pros::c::E_DEVICE_IMU);
        ProsDeviceProvider devices;
        const shulib::robot::BuildVerdict v = shulib::robot::validate(kInventedXDrive, devices);
        CAPTURE(v.why);
        REQUIRE(v.ok);
        std::optional<shulib::robot::Robot> robot;
        robot.emplace(kInventedXDrive, devices);
        CHECK(pros::shim::imuState(10).resetCalls == 1);  // asked twice, calibrated once
        const auto motors = robot->context().driveMotors();
        REQUIRE(motors.size() == 4);
        for (std::size_t i = 0; i < 4; ++i) {
            motors[i]->setVoltage(Voltage{6.0});
        }
        // FL 1 and BL 2 stored positive, BR -3 and FR -4 stored NEGATIVE: PROS reverses the
        // latter at the device, and nothing between the config and the adapter negates anything.
        CHECK(pros::shim::motorState(1).lastVoltageMv == 6000);
        CHECK(pros::shim::motorState(2).lastVoltageMv == 6000);
        CHECK(pros::shim::motorState(3).lastVoltageMv == -6000);
        CHECK(pros::shim::motorState(4).lastVoltageMv == -6000);
        for (const int p : {1, 2, 3, 4}) {
            CAPTURE(p);
            CHECK(pros::shim::motorState(p).gearing == pros::v5::MotorGears::green);
        }
        char map[shulib::diag::kMaxPortMapBytes] = "";
        robot->portMap(map, sizeof map);
        CHECK(std::string{map} == "FL1 BL2 BR-3 FR-4 ROTF5 ROTL6 GPS9 IMU10");
        CHECK(robot->hasGpsCorrector());
    }
}
