// The robot file's CAMERA, end to end: config::CameraConfig and RobotConfig::valid()'s camera checks,
// the fake provider's aiVision(), and robot::Robot building the camera — camera(), cameraAbsence() and
// the boot line's camera words in every state the seam can report (chunk VIS1-A1, the VIS1 brief's
// §1.6-§1.8, D5, and §2.2 "the config", "the provider" and "the Robot").
//
// ═══ WHAT THIS FILE TRIES TO BREAK ══════════════════════════════════════════════════
//   * D5 — A REFUSED CAMERA NEVER REFUSES THE ROBOT: a refused port, a provider that refuses with no
//     reason, the older Vision sensor on the port (through the REAL provider and the shim) — each
//     builds a robot that drives, with camera() nullptr and cameraAbsence() saying why (the run's own
//     mutation: the Robot refusing the build on a refused camera).
//   * cameraAbsence() is "" EXACTLY when camera() is non-null; "no camera is declared" for port 0, and
//     the provider is then never asked.
//   * The boot line's camera words, read through the SEAM's status() (never a downcast): model OK,
//     check pending, DIFFERENT with the first differing name, NOT set up with its problem, objects off,
//     "camera ABSENT", and a refused camera — every byte printable ASCII even when a reason is not (DEL
//     included), the camera LAST so a short buffer cuts it and never the drive — in the tank, x-drive AND
//     h-drive banners; a provider's refusal longer than the Robot keeps is cut to its buffer.
//   * valid(): a motor, the IMU or a mechanism sensor typed on the camera's port refused NAMING BOTH (the
//     run's own mutation: valid() not claiming the camera's port); a port past 21; a settings problem;
//     a SET mount with a problem — and an UNSET mount ACCEPTED (the bench can bring a camera up a piece at a time).
//   * The fake provider: refused like its other devices, the SAME camera for equal settings, a refusal
//     for different ones, and readFaults(port) nullptr until the camera is SERVED, then the camera itself.

#include "doctest.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <string>

#include "pros/shim_control.hpp"

#include "shulib/config/camera_config.hpp"
#include "shulib/config/robot_config.hpp"
#include "shulib/hal/drivetrain_description.hpp"
#include "shulib/hal/fake/fake_battery.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_device_provider.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/hal/pros/device_provider.hpp"
#include "shulib/hal/vision_frames.hpp"
#include "shulib/motion/motion_scheduler.hpp"
#include "shulib/robot/robot.hpp"
#include "shulib/units/quantity.hpp"
#include "shulib/vision/override_elements.hpp"

namespace cfgns = shulib::config;
namespace hal = shulib::hal;
using shulib::hal::fake::FakeDeviceProvider;
using shulib::robot::Robot;
using shulib::units::Length;

namespace {

struct NullPacer final : shulib::motion::ITickPacer {
    void pace() override {}
};

struct FakeWorld {
    shulib::hal::fake::FakeClock clock;
    shulib::hal::fake::FakeBattery battery;
    shulib::hal::fake::FakeTelemetrySink sink;
    NullPacer pacer;
    FakeDeviceProvider devices{clock, battery, sink, pacer};
};

hal::AiVisionSettings objectsOnly() {
    hal::AiVisionSettings s{};
    s.objects = true;
    s.expectedClasses = shulib::vision::override2026::kOverrideClassNames;
    s.expectedClassCount = shulib::vision::override2026::kModelClassCount;
    return s;
}

/// A two-motor-per-side tank on 11/-12 and -13/14, the IMU on 21, the camera (if any) on `cameraPort`.
cfgns::RobotConfig tank(std::uint8_t cameraPort) {
    cfgns::RobotConfig c{};
    std::snprintf(c.identity.name, sizeof c.identity.name, "%s", "Cam Bot");
    c.drivetrain.kind = cfgns::DrivetrainKind::Tank;
    hal::DrivetrainDescription& d = c.drivetrain.tank.description;
    d.kind = hal::DrivetrainKind::Tank;
    d.cartridge = hal::MotorCartridge::Blue;
    d.imuPort = 21;
    d.left.signedPorts[0] = 11;
    d.left.signedPorts[1] = -12;
    d.left.count = 2;
    d.right.signedPorts[0] = -13;
    d.right.signedPorts[1] = 14;
    d.right.count = 2;
    d.left.inchesPerRadian = Length{1.5};
    d.right.inchesPerRadian = Length{1.5};
    d.trackWidth = Length{12.0};
    d.source = hal::DescriptionSource::TypedTable;
    c.odometry.kind = cfgns::OdometryKind::DriveEncoders;
    c.sensors.camera.port = cameraPort;
    c.sensors.camera.settings = objectsOnly();
    return c;
}

std::string validWhy(const cfgns::RobotConfig& c) {
    char why[200] = "";
    return c.valid(why, sizeof why) ? std::string{"(valid)"} : std::string{why};
}

std::string banner(const Robot& r) {
    char buf[400] = "";
    r.describe(buf, sizeof buf);
    return std::string{buf};
}

bool ascii(const std::string& s) {
    for (const char ch : s) {
        const auto b = static_cast<unsigned char>(ch);
        if (b < 0x20U || b >= 0x7FU) {
            return false;
        }
    }
    return true;
}

bool endsWith(const std::string& s, const std::string& tail) {
    return s.size() >= tail.size() && s.compare(s.size() - tail.size(), std::string::npos, tail) == 0;
}

/// A provider that forwards everything to the fake but REFUSES the camera, giving `reason` — "" by
/// default (no reason at all), or one LONGER than the Robot keeps.
struct SilentCameraRefusal final : hal::IDeviceProvider {
    explicit SilentCameraRefusal(FakeDeviceProvider& in, const char* why = "") : inner{&in}, reason{why} {}
    FakeDeviceProvider* inner;
    const char* reason;
    int cameraAsks = 0;
    [[nodiscard]] hal::IMotor* motor(std::int8_t p, hal::MotorCartridge c) override { return inner->motor(p, c); }
    [[nodiscard]] hal::IImu* imu(std::uint8_t p) override { return inner->imu(p); }
    [[nodiscard]] hal::IGps* gps(std::uint8_t p, Length f, Length l) override { return inner->gps(p, f, l); }
    [[nodiscard]] hal::IRotation* rotation(std::uint8_t p) override { return inner->rotation(p); }
    [[nodiscard]] hal::IDigitalOut* digitalOut(std::uint8_t line, bool initial) override {
        return inner->digitalOut(line, initial);
    }
    [[nodiscard]] hal::IDistance* distance(std::uint8_t p) override { return inner->distance(p); }
    [[nodiscard]] hal::IOptical* optical(std::uint8_t p) override { return inner->optical(p); }
    [[nodiscard]] hal::IDigitalIn* digitalIn(std::uint8_t l) override { return inner->digitalIn(l); }
    [[nodiscard]] hal::IVisionFrames* aiVision(std::uint8_t, const hal::AiVisionSettings&) override {
        ++cameraAsks;
        return nullptr;
    }
    [[nodiscard]] const hal::IReadFaults* readFaults(std::uint8_t p) const noexcept override {
        return inner->readFaults(p);
    }
    [[nodiscard]] const hal::IReadFaults* lineReadFaults(std::uint8_t l) const noexcept override {
        return inner->lineReadFaults(l);
    }
    [[nodiscard]] hal::IClock& clock() override { return inner->clock(); }
    [[nodiscard]] hal::IBattery& battery() override { return inner->battery(); }
    [[nodiscard]] hal::ITelemetrySink& telemetry() override { return inner->telemetry(); }
    [[nodiscard]] shulib::motion::ITickPacer& pacer() override { return inner->pacer(); }
    [[nodiscard]] const char* lastRefusal() const noexcept override { return reason; }
};

/// A provider that forwards everything to the fake but THROWS from aiVision() — a contract breach (the
/// interface says nullptr and a refusal), which must still never stop the robot (D5).
struct ThrowingCamera final : hal::IDeviceProvider {
    ThrowingCamera(FakeDeviceProvider& in, bool nonStandard) : inner{&in}, throwsNonStandard{nonStandard} {}
    FakeDeviceProvider* inner;
    bool throwsNonStandard;
    [[nodiscard]] hal::IMotor* motor(std::int8_t p, hal::MotorCartridge c) override { return inner->motor(p, c); }
    [[nodiscard]] hal::IImu* imu(std::uint8_t p) override { return inner->imu(p); }
    [[nodiscard]] hal::IGps* gps(std::uint8_t p, Length f, Length l) override { return inner->gps(p, f, l); }
    [[nodiscard]] hal::IRotation* rotation(std::uint8_t p) override { return inner->rotation(p); }
    [[nodiscard]] hal::IDigitalOut* digitalOut(std::uint8_t line, bool initial) override {
        return inner->digitalOut(line, initial);
    }
    [[nodiscard]] hal::IDistance* distance(std::uint8_t p) override { return inner->distance(p); }
    [[nodiscard]] hal::IOptical* optical(std::uint8_t p) override { return inner->optical(p); }
    [[nodiscard]] hal::IDigitalIn* digitalIn(std::uint8_t l) override { return inner->digitalIn(l); }
    [[nodiscard]] hal::IVisionFrames* aiVision(std::uint8_t, const hal::AiVisionSettings&) override {
        if (throwsNonStandard) {
            throw 42;  // not even a std::exception
        }
        throw std::runtime_error("cable fault");
    }
    [[nodiscard]] const hal::IReadFaults* readFaults(std::uint8_t p) const noexcept override {
        return inner->readFaults(p);
    }
    [[nodiscard]] const hal::IReadFaults* lineReadFaults(std::uint8_t l) const noexcept override {
        return inner->lineReadFaults(l);
    }
    [[nodiscard]] hal::IClock& clock() override { return inner->clock(); }
    [[nodiscard]] hal::IBattery& battery() override { return inner->battery(); }
    [[nodiscard]] hal::ITelemetrySink& telemetry() override { return inner->telemetry(); }
    [[nodiscard]] shulib::motion::ITickPacer& pacer() override { return inner->pacer(); }
    [[nodiscard]] const char* lastRefusal() const noexcept override { return ""; }
};

}  // namespace

// ═══ The config ════════════════════════════════════════════════════════════════════

TEST_CASE("VIS1-A1 config: no camera by default; a camera on a free port with good settings and an UNSET "
          "mount is VALID") {
    cfgns::RobotConfig c = tank(0);
    CHECK(c.sensors.camera.port == 0);
    CHECK(validWhy(c) == "(valid)");
    c = tank(3);
    CHECK_FALSE(c.sensors.camera.placement.measured);  // UNSET: not a config error (a piece at a time)
    CHECK(validWhy(c) == "(valid)");
    // A designated initializer that names only `.gps` still compiles and still means "no camera".
    const cfgns::SensorsConfig s{.gps = {}};
    CHECK(s.camera.port == 0);
}

TEST_CASE("VIS1-A1 config: anything else typed on the camera's port is refused NAMING BOTH") {
    // BUG CAUGHT (the run's own: valid() not claiming the camera's port): a drive motor and the camera
    // on one port would both be built — and the camera's set-up would write to a motor.
    CHECK(validWhy(tank(11)) == "port 11 is claimed by the LEFT drive and by the AI Vision camera");
    CHECK(validWhy(tank(14)) == "port 14 is claimed by the RIGHT drive and by the AI Vision camera");
    CHECK(validWhy(tank(12)) == "port 12 is claimed by the LEFT drive and by the AI Vision camera");
    CHECK(validWhy(tank(21)) == "port 21 is claimed by the IMU and by the AI Vision camera");
    for (std::uint8_t p = 1; p <= 21; ++p) {
        CAPTURE(static_cast<int>(p));
        const bool driveOrImu = p == 11 || p == 12 || p == 13 || p == 14 || p == 21;
        CHECK((validWhy(tank(p)) == "(valid)") == !driveOrImu);
    }
}

TEST_CASE("VIS1-A1 config: a camera port past 21, a settings problem, and a SET mount with a problem are "
          "each refused by name") {
    CHECK(validWhy(tank(22)) == "camera port 22 is not a port in 1..21 (0 = no camera)");
    CHECK(validWhy(tank(255)) == "camera port 255 is not a port in 1..21 (0 = no camera)");
    cfgns::RobotConfig c = tank(3);
    c.sensors.camera.settings = hal::AiVisionSettings{};
    CHECK(validWhy(c) == "the camera's settings (port 3): nothing turned on: no colours and no objects");
    c = tank(3);
    c.sensors.camera.settings.colourCount = 1;  // an UNSET descriptor
    CHECK(validWhy(c).rfind("the camera's settings (port 3): UNSET", 0) == 0);
    // A camera on port 0 is never checked, whatever its block says.
    c = tank(0);
    c.sensors.camera.settings = hal::AiVisionSettings{};
    CHECK(validWhy(c) == "(valid)");

    c = tank(3);
    cfgns::CameraPlacement& p = c.sensors.camera.placement;
    p.measured = true;
    p.height = Length{10.0};
    p.positionTolerance = Length{0.25};
    p.heightTolerance = Length{0.25};
    p.yawToleranceDeg = 1.0;
    p.pitchToleranceDeg = 1.0;
    p.provenance = "host test";
    CHECK(validWhy(c) == "(valid)");
    p.pitchToleranceDeg = 0.0;
    CHECK(validWhy(c)
          == "the camera's mount (port 3): a mount tolerance is zero, UNSET or not a number -- state how well "
             "it was measured");
    p.pitchToleranceDeg = 1.0;
    p.height = Length{40.0};
    CHECK(validWhy(c) == "the camera's mount (port 3): camera height outside (0, 36) in");
    p.height = Length{10.0};
    p.pitchDeg = 50.0;
    CHECK(validWhy(c) == "the camera's mount (port 3): pitch outside +-45 degrees");
    p.pitchDeg = 0.0;
    p.provenance = "";
    CHECK(validWhy(c) == "the camera's mount (port 3): no provenance -- who measured the mount, how, when?");
}

TEST_CASE("VIS1-A1 config: the camera is claimed AFTER every older claim -- a drive collision keeps its own "
          "message") {
    cfgns::RobotConfig c = tank(3);
    c.drivetrain.tank.description.right.signedPorts[1] = 11;  // a drive collision AND a camera
    const std::string why = validWhy(c);
    CHECK(why.find("AI Vision camera") == std::string::npos);
    CHECK(why != "(valid)");
}

// ═══ The fake provider ═════════════════════════════════════════════════════════════

TEST_CASE("VIS1-A1 fake provider: refused like its other devices; the same camera for equal settings; "
          "readFaults nullptr until SERVED, then the camera itself") {
    // BUG CAUGHT: readFaults answering a camera nobody asked for (A1's "only where served" rule, kept
    // for the camera — VIS1's re-check T9).
    FakeWorld w;
    CHECK(w.devices.readFaults(3) == nullptr);
    CHECK_FALSE(w.devices.cameraMade(3));
    CHECK(w.devices.aiVision(0, objectsOnly()) == nullptr);
    CHECK(std::string{w.devices.lastRefusal()} == "port 0 is not a port in 1..21");
    CHECK(w.devices.aiVision(22, objectsOnly()) == nullptr);
    CHECK(w.devices.aiVision(3, hal::AiVisionSettings{}) == nullptr);
    CHECK(std::string{w.devices.lastRefusal()}
          == "port 3: the camera's settings: nothing turned on: no colours and no objects");
    CHECK_FALSE(w.devices.cameraMade(3));
    CHECK(w.devices.readFaults(3) == nullptr);

    hal::IVisionFrames* cam = w.devices.aiVision(3, objectsOnly());
    REQUIRE(cam != nullptr);
    CHECK(cam == &w.devices.fakeCamera(3));
    CHECK(w.devices.cameraMade(3));
    CHECK(hal::sameAiVisionSettings(w.devices.cameraSettings(3), objectsOnly()));
    CHECK(w.devices.aiVision(3, objectsOnly()) == cam);
    hal::AiVisionSettings other = objectsOnly();
    other.expectedClassCount = 4;
    CHECK(w.devices.aiVision(3, other) == nullptr);
    CHECK(std::string{w.devices.lastRefusal()}
          == "port 3: the camera was already set up with other settings -- one owner");
    CHECK(w.devices.aiVision(3, objectsOnly()) == cam);

    const hal::IReadFaults* rf = w.devices.readFaults(3);
    REQUIRE(rf != nullptr);
    CHECK(rf == static_cast<const hal::IReadFaults*>(&w.devices.fakeCamera(3)));
    w.devices.fakeCamera(3).setFailing(true);
    CHECK(cam->read().status == hal::FrameStatus::ReadFault);
    CHECK(rf->faultedReads() == 1);
    CHECK(w.devices.readFaults(4) == nullptr);

    w.devices.refusePort(5);
    CHECK(w.devices.aiVision(5, objectsOnly()) == nullptr);
    CHECK(std::string{w.devices.lastRefusal()} == "port 5 has no AI Vision camera");
    CHECK(w.devices.readFaults(5) == nullptr);
}

// ═══ The Robot ═════════════════════════════════════════════════════════════════════

TEST_CASE("VIS1-A1 Robot: a declared camera is BUILT -- the provider's own, with the config's settings -- "
          "and cameraAbsence() is empty") {
    FakeWorld w;
    const cfgns::RobotConfig c = tank(3);
    REQUIRE(c.valid());
    Robot robot{c, w.devices};
    REQUIRE(robot.camera() != nullptr);
    CHECK(robot.camera() == &w.devices.fakeCamera(3));
    CHECK(std::string{robot.cameraAbsence()}.empty());
    CHECK(hal::sameAiVisionSettings(w.devices.cameraSettings(3), c.sensors.camera.settings));
    const Robot& constRobot = robot;
    CHECK(constRobot.camera() == robot.camera());
    // The builder never reads a frame.
    CHECK(w.devices.fakeCamera(3).reads() == 0U);
}

TEST_CASE("VIS1-A1 Robot: port 0 -- no camera, 'no camera is declared', and the provider is never asked") {
    FakeWorld w;
    Robot robot{tank(0), w.devices};
    CHECK(robot.camera() == nullptr);
    CHECK(std::string{robot.cameraAbsence()} == "no camera is declared");
    for (std::uint8_t p = 1; p <= 21; ++p) {
        CHECK_FALSE(w.devices.cameraMade(p));
    }
    CHECK(endsWith(banner(robot), "tags ABSENT walls ABSENT camera ABSENT"));
}

TEST_CASE("VIS1-A1 Robot D5: a REFUSED camera never refuses the robot -- it builds and DRIVES, the camera "
          "absent with the provider's sentence") {
    // BUG CAUGHT (the run's own: the Robot refusing the build on a refused camera): a camera cable out
    // at an event must never cost the match.
    FakeWorld w;
    w.devices.refusePort(3);
    const cfgns::RobotConfig c = tank(3);
    const shulib::robot::BuildVerdict v = shulib::robot::validate(c, w.devices);
    CAPTURE(v.why);
    REQUIRE(v.ok);
    std::optional<Robot> built;
    REQUIRE_NOTHROW(built.emplace(c, w.devices));
    Robot& robot = *built;
    CHECK(robot.camera() == nullptr);
    CHECK(std::string{robot.cameraAbsence()} == "port 3 has no AI Vision camera");
    // It drives: the drive motors exist and take a command.
    REQUIRE(robot.context().driveMotors().size() == 2U);
    robot.context().driveMotors()[0]->setVoltage(shulib::units::Voltage{6.0});
    CHECK(w.devices.fakeMotor(11).commandedVoltage().value() == doctest::Approx(6.0));
    CHECK(endsWith(banner(robot), "tags ABSENT walls ABSENT camera port 3 ABSENT (port 3 has no AI Vision camera)"));

    // A provider that refuses with NO reason still leaves one.
    FakeWorld w2;
    SilentCameraRefusal silent{w2.devices};
    Robot quiet{c, silent};
    CHECK(silent.cameraAsks == 1);
    CHECK(quiet.camera() == nullptr);
    CHECK(std::string{quiet.cameraAbsence()} == "the device provider refused the camera and gave no reason");

    // A refusal LONGER than the Robot keeps is cut to its buffer, never copied past it (review X23: the
    // interface does not bound lastRefusal(), so the copy's bound is the only one).
    FakeWorld w5;
    const std::string huge = "port 3: " + std::string(600, 'x');
    SilentCameraRefusal wordy{w5.devices, huge.c_str()};
    Robot longWinded{c, wordy};
    CHECK(longWinded.camera() == nullptr);
    const std::string kept{longWinded.cameraAbsence()};
    CHECK(kept.size() == 159U);  // the 160-byte buffer, NUL included
    CHECK(kept == huge.substr(0, 159));

    // A provider that THROWS (a contract breach) still never stops the robot: the camera is absent, and
    // the throw's words are the reason.
    FakeWorld w3;
    ThrowingCamera throwing{w3.devices, false};
    std::optional<Robot> survivor;
    REQUIRE_NOTHROW(survivor.emplace(c, throwing));
    CHECK(survivor->camera() == nullptr);
    CHECK(std::string{survivor->cameraAbsence()} == "the device provider threw making the camera: cable fault");
    CHECK(endsWith(banner(*survivor),
                   "camera port 3 ABSENT (the device provider threw making the camera: cable fault)"));
    FakeWorld w4;
    ThrowingCamera odd{w4.devices, true};
    std::optional<Robot> survivor2;
    REQUIRE_NOTHROW(survivor2.emplace(c, odd));
    CHECK(survivor2->camera() == nullptr);
    CHECK(std::string{survivor2->cameraAbsence()} == "the device provider threw making the camera");
}

TEST_CASE("VIS1-A1 Robot: the boot line's camera words in EVERY state the seam reports, read through "
          "status(), ASCII only, LAST") {
    FakeWorld w;
    Robot robot{tank(3), w.devices};
    REQUIRE(robot.camera() != nullptr);
    shulib::hal::fake::FakeVisionFrames& cam = w.devices.fakeCamera(3);
    const std::string head = "Cam Bot [] tank TYPED | L 11,-12 R -13,14 | ipr L1.50000 R1.50000 W 12.000 in | "
                             "IMU21 | drive-encoders | GPS ABSENT tags ABSENT walls ABSENT ";  // L6-A2: the walls words

    // Default (nothing set): NOT set up.
    CHECK(banner(robot) == head + "camera port 3 NOT set up");
    cam.setStatus({false, "a colour write failed (PROS_ERR)", hal::ModelCheck::NotAsked, ""});
    CHECK(banner(robot) == head + "camera port 3 NOT set up (a colour write failed (PROS_ERR))");
    cam.setStatus({true, "", hal::ModelCheck::Matches, ""});
    CHECK(banner(robot) == head + "camera port 3 set up, model OK");
    cam.setStatus({true, "", hal::ModelCheck::Unreadable, "class 0's name could not be read (PROS_ERR)"});
    CHECK(banner(robot)
          == head + "camera port 3 set up, model check pending (class 0's name could not be read (PROS_ERR))");
    cam.setStatus({true, "", hal::ModelCheck::Different,
                   "class 0 is 'blueBall', the robot file expects 'redBluePin'"});
    CHECK(banner(robot)
          == head + "camera port 3 set up, model DIFFERENT (class 0 is 'blueBall', the robot file expects "
                    "'redBluePin')");
    cam.setStatus({true, "", hal::ModelCheck::NotAsked, ""});
    CHECK(banner(robot) == head + "camera port 3 set up, objects off");
    // A model problem is printed only for a model verdict that has one; a set-up that holds prints none.
    cam.setStatus({true, "stale words", hal::ModelCheck::Matches, "stale too"});
    CHECK(banner(robot) == head + "camera port 3 set up, model OK");
    // Non-ASCII from below becomes '?' — every byte drawable.
    cam.setStatus({true, "", hal::ModelCheck::Different, "class 0 is 'caf\xC3\xA9', \x01 x\x7F"});
    const std::string odd = banner(robot);
    CHECK(ascii(odd));
    CHECK(endsWith(odd, "(class 0 is 'caf?" "?', ? x?)"));  // split: "??'" would be a trigraph
    CHECK(odd.find('\x7F') == std::string::npos);  // DEL is not printable either (review X2)
    // A null reason from a provider's adapter is not dereferenced.
    cam.setStatus({false, nullptr, hal::ModelCheck::NotAsked, nullptr});
    CHECK(banner(robot) == head + "camera port 3 NOT set up");
    // The builder still never read a frame.
    CHECK(cam.reads() == 0U);
}

TEST_CASE("VIS1-A1 Robot: a short buffer cuts the CAMERA's words, never the drive; a huge reason is "
          "bounded") {
    FakeWorld w;
    Robot robot{tank(3), w.devices};
    std::string longReason(600, 'x');
    w.devices.fakeCamera(3).setStatus({true, "", hal::ModelCheck::Different, longReason.c_str()});
    char buf[180];
    std::memset(buf, '\xAB', sizeof buf);
    robot.describe(buf, 150);
    const std::string s{buf};
    CHECK(s.size() == 149U);
    CHECK(buf[150] == '\xAB');  // the canary past the cap
    CHECK(s.find("L 11,-12 R -13,14") != std::string::npos);
    char big[1200] = "";
    robot.describe(big, sizeof big);
    const std::string b{big};
    CHECK(b.find("camera port 3 set up, model DIFFERENT (xxx") != std::string::npos);
    CHECK(b.size() < 700U);  // the camera's part is bounded whatever the adapter says
    CHECK(ascii(b));
}

TEST_CASE("VIS1-A1 Robot: the H-DRIVE banner ends with the camera's words too (every describe() branch)") {
    // BUG CAUGHT (review X18): the tank and x-drive banners were pinned, the h-drive's was not — a
    // revert of its branch to the old "vision ABSENT" survived.
    FakeWorld w;
    cfgns::RobotConfig h = tank(3);
    h.drivetrain.kind = cfgns::DrivetrainKind::HDrive;
    h.drivetrain.hdrive = {.ports = {-1, 2, 4},
                           .cartridge = hal::MotorCartridge::Blue,
                           .imuPort = 21,
                           .trackWidth = Length{12.0},
                           .strafeWheelOffset = Length{-4.0}};
    h.odometry = {.kind = cfgns::OdometryKind::TrackingWheels,  // a holonomic drive has no two sides
                  .wheels = {.forward = {.port = 5, .wheelDiameter = Length{2.0}, .offset = Length{-3.0}},
                             .lateral = {.port = 6, .wheelDiameter = Length{2.0}, .offset = Length{-4.5}}}};
    CAPTURE(validWhy(h));
    REQUIRE(h.valid());
    Robot robot{h, w.devices};
    REQUIRE(robot.camera() != nullptr);
    w.devices.fakeCamera(3).setStatus({true, "", hal::ModelCheck::Matches, ""});
    const std::string b = banner(robot);
    CAPTURE(b);
    CHECK(b.find("h-drive TYPED") != std::string::npos);
    CHECK(endsWith(b, "tags ABSENT walls ABSENT camera port 3 set up, model OK"));
    CHECK(b.find("vision") == std::string::npos);
    FakeWorld w2;
    h.sensors.camera.port = 0;
    Robot none{h, w2.devices};
    CHECK(endsWith(banner(none), "tags ABSENT walls ABSENT camera ABSENT"));
}

TEST_CASE("VIS1-A1 Robot over the REAL provider and the shim: the Classroom model reads DIFFERENT naming "
          "the first name; the OLDER Vision sensor leaves the camera ABSENT and the robot builds") {
    pros::shim::resetAll();
    for (const int p : {11, 12, 13, 14}) {
        pros::shim::setPluggedAtPort(p, pros::c::E_DEVICE_MOTOR);
    }
    pros::shim::setPluggedAtPort(21, pros::c::E_DEVICE_IMU);
    const cfgns::RobotConfig c = tank(3);

    SUBCASE("an AI Vision sensor carrying VEX's Classroom model") {
        pros::shim::setPluggedAtPort(3, pros::c::E_DEVICE_AIVISION);
        shulib::hal::pros::ProsDeviceProvider devices;
        Robot robot{c, devices};
        REQUIRE(robot.camera() != nullptr);
        CHECK(std::string{robot.cameraAbsence()}.empty());
        CHECK(pros::shim::aiVisionState(3).modes == pros::E_AIVISION_MODE_OBJECTS);  // written at boot
        CHECK(endsWith(banner(robot),
                       "camera port 3 set up, model DIFFERENT (class 0 is 'blueBall', the robot file expects "
                       "'redBluePin')"));
        CHECK(pros::shim::aiVisionState(3).countReads == 0);  // the builder never reads a frame
    }
    SUBCASE("the Override model") {
        pros::shim::setPluggedAtPort(3, pros::c::E_DEVICE_AIVISION);
        auto& st = pros::shim::aiVisionState(3);
        st.classNames = {"redBluePin", "blueYellowPin", "redYellowPin", "yellowPin", "cup"};
        st.classCount = 5;
        shulib::hal::pros::ProsDeviceProvider devices;
        Robot robot{c, devices};
        CHECK(endsWith(banner(robot), "camera port 3 set up, model OK"));
    }
    SUBCASE("the OLDER Vision sensor on the camera's port (D4): absent by name, the robot built") {
        pros::shim::setPluggedAtPort(3, pros::c::E_DEVICE_VISION);
        shulib::hal::pros::ProsDeviceProvider devices;
        const shulib::robot::BuildVerdict v = shulib::robot::validate(c, devices);
        CAPTURE(v.why);
        REQUIRE(v.ok);
        Robot robot{c, devices};
        CHECK(robot.camera() == nullptr);
        CHECK(std::string{robot.cameraAbsence()} == "port 3: a vision sensor (expected an AI Vision sensor)");
        CHECK(endsWith(banner(robot),
                       "camera port 3 ABSENT (port 3: a vision sensor (expected an AI Vision sensor))"));
        CHECK(pros::shim::aiVisionState(3).modeWrites == 0);  // never written
    }
}
