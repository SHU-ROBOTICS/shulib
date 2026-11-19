// The robot file's TOGGLE BLOCK and the Robot's Toggle-camera slot (chunk AF1S-A2, 2026-11-15; the AF1S
// brief's §1.3 and §2.2's robot_toggle_camera_test): vision::ToggleCameraConfig, vision::toggleCameraCheck(),
// vision::ToggleCamera, robot::Robot's toggleCamera() / toggleCameraAbsence(), the context's second OPTIONAL
// slot and the boot line's camera words. Host only: nothing here has run on a robot.
//
// ═══ WHAT THIS FILE TRIES TO BREAK ══════════════════════════════════════════════════════════════
//   * THE SLOT IS BUILT ONLY when the block is declared, a camera is declared, its mount is MEASURED (and
//     inside the geometry's limits), the tracker's and the feed's numbers are accepted, the three colour
//     ids are colours the camera is taught, and the camera was BUILT — each refusal alone leaves it NOT
//     READY with ITS sentence (M-A2-3: the slot built on an UNMEASURED mount).
//   * A REFUSED BLOCK NEVER REFUSES THE ROBOT (M-A2-2): validate() and valid() pass, the constructor does
//     not throw, the robot DRIVES — for every refusal, a provider that refuses or throws the camera included.
//   * The slot reaches the CHASSIS: `chassis().deps().ctx` is the motion scheduler's shadow context, and the
//     slot pointer must be carried there (M-A2-1, at the unit level).
//   * The FIRST reason wins, in the documented order — the config's own checks before "the camera was not
//     built".
//   * DECLARED means ANY number set — each of the block's 17 numbers alone, a NaN too; all zero is not.
//   * The slot asks the provider for NOTHING: the requests with the block are exactly the requests without.
//   * The boot line: with no block, EXACTLY what it was; with one, ", toggles ready" or ", toggles OFF (why)"
//     after the camera's own words — which stay bounded where they were, ASCII, and LAST.
//   * The check is usable AT BUILD (`static_assert`) and agrees with the Robot.
//   (AF1S-A2's review, 2026-11-16:) the feed reads at the BLOCK's period — on the period's equality edge, not
//   one double before it; "no block" is the FIRST reason when there is neither a camera nor a block; RED is
//   named before BLUE when both are untaught; and the const `toggleCamera()` answers as the non-const one.

#include "doctest.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "shulib/config/robot_config.hpp"
#include "shulib/core/check.hpp"
#include "shulib/hal/fake/fake_battery.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_device_provider.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/motion/motion_scheduler.hpp"
#include "shulib/robot/robot.hpp"
#include "shulib/vision/toggle_camera.hpp"
#include "toggle_robot_rig.hpp"

namespace cfgns = shulib::config;
namespace hal = shulib::hal;
namespace vision = shulib::vision;
namespace trr = toggle_robot_rig;
using shulib::hal::fake::FakeDeviceProvider;
using shulib::robot::Robot;
using shulib::units::Length;

namespace {

struct NullPacer final : shulib::motion::ITickPacer {
    void pace() override {}
};

/// The provider, its camera on port 3 SET UP with the AI model off (the words "set up, objects off").
struct FakeWorld {
    shulib::hal::fake::FakeClock clock;
    shulib::hal::fake::FakeBattery battery;
    shulib::hal::fake::FakeTelemetrySink sink;
    NullPacer pacer;
    FakeDeviceProvider devices{clock, battery, sink, pacer};
    FakeWorld() { devices.fakeCamera(trr::kCameraPort).setStatus({true, "", hal::ModelCheck::NotAsked, ""}); }
};

std::string banner(const Robot& r) {
    char buf[1200] = "";
    r.describe(buf, sizeof buf);
    return std::string{buf};
}

bool endsWith(const std::string& s, const std::string& tail) {
    return s.size() >= tail.size() && s.compare(s.size() - tail.size(), std::string::npos, tail) == 0;
}

bool ascii(const std::string& s) {
    return std::all_of(s.begin(), s.end(), [](char ch) {
        const auto b = static_cast<unsigned char>(ch);
        return b >= 0x20U && b < 0x7FU;
    });
}

/// The recipe robot's file with no Toggle block.
cfgns::RobotConfig withoutBlock() {
    cfgns::RobotConfig c = trr::robotFile();
    c.sensors.toggleCamera = {};
    return c;
}

/// A provider that forwards everything to the fake and RECORDS, in order, every device it is asked for
/// (test/robot_wall_sensors_test.cpp's, for this file) — the order a Robot's constructor makes its requests.
struct RecordingProvider final : hal::IDeviceProvider {
    explicit RecordingProvider(FakeDeviceProvider& in) : inner{&in} {}
    FakeDeviceProvider* inner;
    mutable std::vector<std::string> asked;
    void note(const char* what, int port) const { asked.push_back(std::string{what} + " " + std::to_string(port)); }
    [[nodiscard]] hal::IMotor* motor(std::int8_t p, hal::MotorCartridge c) override {
        note("motor", p);
        return inner->motor(p, c);
    }
    [[nodiscard]] hal::IMotorCurrentLimit* currentLimit(std::uint8_t p) noexcept override {
        note("currentLimit", p);
        return inner->currentLimit(p);
    }
    [[nodiscard]] hal::IImu* imu(std::uint8_t p) override {
        note("imu", p);
        return inner->imu(p);
    }
    [[nodiscard]] hal::IGps* gps(std::uint8_t p, Length f, Length l) override {
        note("gps", p);
        return inner->gps(p, f, l);
    }
    [[nodiscard]] hal::IRotation* rotation(std::uint8_t p) override {
        note("rotation", p);
        return inner->rotation(p);
    }
    [[nodiscard]] hal::IDigitalOut* digitalOut(std::uint8_t line, bool initial) override {
        note("digitalOut", line);
        return inner->digitalOut(line, initial);
    }
    [[nodiscard]] hal::IDistance* distance(std::uint8_t p) override {
        note("distance", p);
        return inner->distance(p);
    }
    [[nodiscard]] hal::IOptical* optical(std::uint8_t p) override {
        note("optical", p);
        return inner->optical(p);
    }
    [[nodiscard]] hal::IDigitalIn* digitalIn(std::uint8_t l) override {
        note("digitalIn", l);
        return inner->digitalIn(l);
    }
    [[nodiscard]] hal::IVisionFrames* aiVision(std::uint8_t p, const hal::AiVisionSettings& s) override {
        note("aiVision", p);
        return inner->aiVision(p, s);
    }
    [[nodiscard]] const hal::IReadFaults* readFaults(std::uint8_t p) const noexcept override {
        note("readFaults", p);
        return inner->readFaults(p);
    }
    [[nodiscard]] const hal::IReadFaults* lineReadFaults(std::uint8_t l) const noexcept override {
        note("lineReadFaults", l);
        return inner->lineReadFaults(l);
    }
    [[nodiscard]] hal::IClock& clock() override {
        note("clock", 0);
        return inner->clock();
    }
    [[nodiscard]] hal::IBattery& battery() override { return inner->battery(); }
    [[nodiscard]] hal::ITelemetrySink& telemetry() override { return inner->telemetry(); }
    [[nodiscard]] shulib::motion::ITickPacer& pacer() override { return inner->pacer(); }
    [[nodiscard]] const char* lastRefusal() const noexcept override { return inner->lastRefusal(); }
};

/// A provider that forwards everything to the fake but THROWS from aiVision() (a contract breach).
struct ThrowingCamera final : hal::IDeviceProvider {
    explicit ThrowingCamera(FakeDeviceProvider& in) : inner{&in} {}
    FakeDeviceProvider* inner;
    [[nodiscard]] hal::IMotor* motor(std::int8_t p, hal::MotorCartridge c) override { return inner->motor(p, c); }
    [[nodiscard]] hal::IMotorCurrentLimit* currentLimit(std::uint8_t p) noexcept override {
        return inner->currentLimit(p);
    }
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

/// Every refusal's robot: built, not refused, DRIVING, its slot present in the context (declared) but not
/// READY, its sentence `why` everywhere it is read, the boot line ending ", toggles OFF (why)".
void checkRefusedButDrives(const cfgns::RobotConfig& c, hal::IDeviceProvider& devices, FakeDeviceProvider& fakes,
                           const std::string& why) {
    char validWhy[200] = "";
    CHECK(c.valid(validWhy, sizeof validWhy));  // valid() never checks the block
    CAPTURE(validWhy);
    const shulib::robot::BuildVerdict v = shulib::robot::validate(c, devices);
    CAPTURE(v.why);
    REQUIRE(v.ok);  // M-A2-2: a refused block never refuses the robot
    std::optional<Robot> built;
    REQUIRE_NOTHROW(built.emplace(c, devices));
    Robot& robot = *built;
    CHECK(robot.toggleCamera() == nullptr);
    CHECK(std::as_const(robot).toggleCamera() == nullptr);  // the const door too (a status page's const Robot&)
    CHECK(std::string{robot.toggleCameraAbsence()} == why);
    // The context's slot: present (the block is DECLARED), NOT ready, the same sentence — and the chassis's
    // shadow context carries the same pointer.
    vision::ToggleCamera* slot = robot.context().toggleCamera();
    REQUIRE(slot != nullptr);
    CHECK_FALSE(slot->ready());
    CHECK(slot->feed() == nullptr);
    CHECK(slot->model() == nullptr);
    CHECK(std::string{slot->absence()} == why);
    CHECK(robot.chassis().deps().ctx->toggleCamera() == slot);
    // It drives: a drive motor takes a command.
    REQUIRE(robot.context().driveMotors().size() == 2U);
    robot.context().driveMotors()[0]->setVoltage(shulib::units::Voltage{6.0});
    CHECK(fakes.fakeMotor(16).commandedVoltage().value() == doctest::Approx(6.0));
    const std::string b = banner(robot);
    CHECK(endsWith(b, ", toggles OFF (" + why + ")"));
    CHECK(ascii(b));
}

const std::string kNoBlock = "no Toggle block is declared (the robot file's sensors.toggleCamera)";
const std::string kNotBuilt = "the camera was not built (the boot line's camera words say why)";

}  // namespace

// ═══ Built ════════════════════════════════════════════════════════════════════════════════════

// Bug caught: a Robot that builds a slot but hands the step something else — no feed, the wrong mount, the
// library's numbers in place of the team's — or one that READS the camera at boot (the feed's first read
// would then be stale by the time an auton looks), or a slot that never reaches the Chassis.
TEST_CASE("AF1S-A2 Robot: a camera, a MEASURED mount and the block set -- the slot READY, the context's and the "
          "chassis's slot the Robot's, the team's numbers, nothing read at boot, ', toggles ready'") {
    FakeWorld w;
    const cfgns::RobotConfig c = trr::robotFile();
    REQUIRE(c.valid());
    Robot robot{c, w.devices};
    vision::ToggleCamera* slot = robot.toggleCamera();
    REQUIRE(slot != nullptr);
    CHECK(slot->ready());
    CHECK(std::as_const(robot).toggleCamera() == slot);  // the const door hands out the same slot
    CHECK(std::string{robot.toggleCameraAbsence()}.empty());
    CHECK(std::string{slot->absence()}.empty());
    CHECK(robot.context().toggleCamera() == slot);
    CHECK(robot.chassis().deps().ctx->toggleCamera() == slot);  // M-A2-1: the scheduler's shadow carries it
    CHECK(robot.chassis().deps().ctx != &robot.context());       // (it IS a different context — the shadow)
    REQUIRE(slot->feed() != nullptr);
    REQUIRE(slot->model() != nullptr);
    // The model is the CONFIG's measured mount — every number.
    const cfgns::CameraPlacement& p = slot->model()->placement();
    CHECK(p.measured);
    CHECK(p.forward.value() == c.sensors.camera.placement.forward.value());
    CHECK(p.height.value() == c.sensors.camera.placement.height.value());
    CHECK(p.pitchDeg == c.sensors.camera.placement.pitchDeg);
    CHECK(p.yawDeg == c.sensors.camera.placement.yawDeg);
    CHECK(std::string{p.provenance} == c.sensors.camera.placement.provenance);
    // The tracker numbers are the BLOCK's.
    CHECK(slot->tracker().ids.red == c.sensors.toggleCamera.tracker.ids.red);
    CHECK(slot->tracker().ids.blue == c.sensors.toggleCamera.tracker.ids.blue);
    CHECK(slot->tracker().ids.yellow == c.sensors.toggleCamera.tracker.ids.yellow);
    CHECK(slot->tracker().holdSeconds == c.sensors.toggleCamera.tracker.holdSeconds);
    CHECK(slot->tracker().frozenMoveIn == c.sensors.toggleCamera.tracker.frozenMoveIn);
    CHECK(slot->tracker().bearingWindowDeg == c.sensors.toggleCamera.tracker.bearingWindowDeg);
    // The feed reads the ROBOT's camera — and has not read it yet: the builder never ticks.
    CHECK(w.devices.fakeCamera(trr::kCameraPort).reads() == 0U);
    CHECK(slot->feed()->stats().reads == 0U);
    const double t0 = w.clock.now().value();
    CHECK(slot->feed()->tick());  // a first tick reads THE robot's camera (the provider's fake on port 3)
    CHECK(w.devices.fakeCamera(trr::kCameraPort).reads() == 1U);
    // The feed reads at the BLOCK's period (review, 2026-11-16 — mutants R5/R6, the period replaced by a
    // constant, survived): one tick just under a full period after the first read began reads nothing; one ON
    // the period's equality edge (the first instant the feed's own arithmetic calls a full period) reads.
    const double period = c.sensors.toggleCamera.feed.minReadPeriodSeconds;
    REQUIRE(period == 0.033);
    double edge = t0 + period;
    while (edge - t0 < period) {
        edge = std::nextafter(edge, HUGE_VAL);
    }
    const double under = std::nextafter(edge, 0.0);
    REQUIRE(under - t0 < period);
    w.clock.set(shulib::units::Time{under});
    CHECK_FALSE(slot->feed()->tick());
    CHECK(w.devices.fakeCamera(trr::kCameraPort).reads() == 1U);
    w.clock.set(shulib::units::Time{edge});
    CHECK(slot->feed()->tick());
    CHECK(w.devices.fakeCamera(trr::kCameraPort).reads() == 2U);
    CHECK(endsWith(banner(robot), "walls ABSENT camera port 3 set up, objects off, toggles ready"));
}

// Bug caught: a robot file with no Toggle block handed a slot anyway (the step would then say "not built
// yet" where the true word is "no Toggle block is declared"), or its boot line changed.
TEST_CASE("AF1S-A2 Robot: NO Toggle block -- no slot in the context or the chassis, the sentence names the "
          "block, and the boot line is EXACTLY what it was") {
    FakeWorld w;
    const cfgns::RobotConfig c = withoutBlock();
    CHECK_FALSE(c.sensors.toggleCamera.declared());
    Robot robot{c, w.devices};
    CHECK(robot.toggleCamera() == nullptr);
    CHECK(std::string{robot.toggleCameraAbsence()} == kNoBlock);
    CHECK(robot.context().toggleCamera() == nullptr);
    CHECK(robot.chassis().deps().ctx->toggleCamera() == nullptr);
    CHECK(robot.camera() != nullptr);  // the camera itself is built as ever
    const std::string b = banner(robot);
    CHECK(endsWith(b, "walls ABSENT camera port 3 set up, objects off"));
    CHECK(b.find("toggles") == std::string::npos);
    // The FIRST reason in the documented order is the block's (review, 2026-11-16 — mutant R4, the camera's
    // check moved first, survived): with NEITHER a camera NOR a block — most robot files today — the sentence
    // is "no Toggle block", not "no camera".
    CHECK(std::string{vision::toggleCameraCheck(cfgns::CameraConfig{}, vision::ToggleCameraConfig{}).why} == kNoBlock);
    CHECK(std::string{vision::toggleCameraCheck(cfgns::CameraConfig{}, vision::ToggleCameraConfig{}).part}.empty());
    {
        FakeWorld w3;
        cfgns::RobotConfig neither = withoutBlock();
        neither.sensors.camera = {};
        const Robot r3{neither, w3.devices};
        CHECK(r3.camera() == nullptr);
        CHECK(r3.toggleCamera() == nullptr);
        CHECK(std::string{r3.toggleCameraAbsence()} == kNoBlock);
    }
    // And the WHOLE line equals a Robot with the block's line less its Toggle words.
    FakeWorld w2;
    const Robot withBlock{trr::robotFile(), w2.devices};
    const std::string bb = banner(withBlock);
    REQUIRE(endsWith(bb, ", toggles ready"));
    CHECK(bb.substr(0, bb.size() - std::strlen(", toggles ready")) == b);
}

// ═══ Each refusal: NOT READY, its sentence, and the robot drives (M-A2-2, M-A2-3) ═══════════════════

TEST_CASE("AF1S-A2 Robot: each refusal ALONE leaves the slot NOT READY with ITS sentence -- and never refuses "
          "the robot, which builds and DRIVES") {
    SUBCASE("no camera declared (port 0), the block typed") {
        FakeWorld w;
        cfgns::RobotConfig c = trr::robotFile();
        c.sensors.camera = {};
        checkRefusedButDrives(c, w.devices, w.devices, "no camera is declared");
        Robot robot{c, w.devices};
        CHECK(endsWith(banner(robot), "camera ABSENT, toggles OFF (no camera is declared)"));
    }
    SUBCASE("M-A2-3: the mount UNMEASURED") {
        FakeWorld w;
        cfgns::RobotConfig c = trr::robotFile();
        c.sensors.camera.placement.measured = false;
        checkRefusedButDrives(c, w.devices, w.devices,
                              "the camera's mount is UNSET -- measure it from the robot's tracking centre and type it");
        // The camera is still built and read (its colours need no mount): only the Toggle slot refuses.
        Robot robot{c, w.devices};
        CHECK(robot.camera() != nullptr);
    }
    SUBCASE("the mount past the geometry's limit (valid() accepts it; the model refuses it)") {
        FakeWorld w;
        cfgns::RobotConfig c = trr::robotFile();
        c.sensors.camera.placement.pitchDeg = 44.0;
        c.sensors.camera.placement.pitchToleranceDeg = 12.0;  // 56 > 55
        checkRefusedButDrives(c, w.devices, w.devices,
                              "the pitch with its tolerance reaches past 55 degrees -- the geometry holds only inside "
                              "it (re-measure the tolerance: a level on the camera's back face reads about 1 degree)");
    }
    SUBCASE("a tracker number UNSET") {
        FakeWorld w;
        cfgns::RobotConfig c = trr::robotFile();
        c.sensors.toggleCamera.tracker.holdSeconds = 0.0;
        checkRefusedButDrives(c, w.devices, w.devices,
                              "the Toggle block's tracker: holdSeconds UNSET -- how long a colour must hold before it "
                              "is reported");
    }
    SUBCASE("a tracker number NaN") {
        FakeWorld w;
        cfgns::RobotConfig c = trr::robotFile();
        c.sensors.toggleCamera.tracker.frozenMoveIn = std::numeric_limits<double>::quiet_NaN();
        checkRefusedButDrives(c, w.devices, w.devices, "the Toggle block's tracker: a tracker number is not a finite number");
    }
    SUBCASE("the feed's period UNSET") {
        FakeWorld w;
        cfgns::RobotConfig c = trr::robotFile();
        c.sensors.toggleCamera.feed.minReadPeriodSeconds = 0.0;
        checkRefusedButDrives(c, w.devices, w.devices,
                              "the Toggle block's feed: minReadPeriodSeconds UNSET -- the team's read period, seconds "
                              "(about 0.033 for 30 a second)");
    }
    SUBCASE("the RED id not taught to the camera") {
        FakeWorld w;
        cfgns::RobotConfig c = trr::robotFile();
        c.sensors.camera.settings.colours[0].id = 4;  // RED taught as 4; the block says 1
        checkRefusedButDrives(c, w.devices, w.devices,
                              "the Toggle block: its RED id is not one of the colours the camera's settings write");
    }
    SUBCASE("the BLUE id not taught to the camera") {
        FakeWorld w;
        cfgns::RobotConfig c = trr::robotFile();
        c.sensors.toggleCamera.tracker.ids.blue = 5;  // the camera teaches 1, 2, 3
        checkRefusedButDrives(c, w.devices, w.devices,
                              "the Toggle block: its BLUE id is not one of the colours the camera's settings write");
    }
    SUBCASE("the RED and the BLUE ids both untaught: the RED is named (the documented order)") {
        FakeWorld w;
        cfgns::RobotConfig c = trr::robotFile();
        c.sensors.toggleCamera.tracker.ids.red = 4;  // the camera teaches 1, 2, 3
        c.sensors.toggleCamera.tracker.ids.blue = 5;
        checkRefusedButDrives(c, w.devices, w.devices,
                              "the Toggle block: its RED id is not one of the colours the camera's settings write");
    }
    SUBCASE("the YELLOW id not taught: only TWO colours written") {
        FakeWorld w;
        cfgns::RobotConfig c = trr::robotFile();
        c.sensors.camera.settings.colourCount = 2;  // the third descriptor is not WRITTEN
        checkRefusedButDrives(c, w.devices, w.devices,
                              "the Toggle block: its YELLOW id is not one of the colours the camera's settings write");
    }
    SUBCASE("the camera taught no colours at all (the AI model only)") {
        FakeWorld w;
        cfgns::RobotConfig c = trr::robotFile();
        hal::AiVisionSettings s{};
        s.objects = true;
        s.expectedClasses = shulib::vision::override2026::kOverrideClassNames;
        s.expectedClassCount = shulib::vision::override2026::kModelClassCount;
        c.sensors.camera.settings = s;
        checkRefusedButDrives(c, w.devices, w.devices,
                              "the Toggle block: the camera's settings write no colours -- teach it the Toggle's three "
                              "faces (the AI Vision Utility) and type them in the camera's block");
    }
    SUBCASE("the camera REFUSED by the provider") {
        FakeWorld w;
        w.devices.refusePort(trr::kCameraPort);
        const cfgns::RobotConfig c = trr::robotFile();
        checkRefusedButDrives(c, w.devices, w.devices, kNotBuilt);
        Robot robot{c, w.devices};
        CHECK(robot.camera() == nullptr);
        CHECK(endsWith(banner(robot), "camera port 3 ABSENT (port 3 has no AI Vision camera), toggles OFF (" +
                                          kNotBuilt + ")"));
    }
    SUBCASE("a provider that THROWS making the camera") {
        FakeWorld w;
        ThrowingCamera throwing{w.devices};
        checkRefusedButDrives(trr::robotFile(), throwing, w.devices, kNotBuilt);
    }
}

// Bug caught: a check order that reports a later problem first (a team fixes the wrong thing), or "the camera
// was not built" hiding a mistake typed in the file.
TEST_CASE("AF1S-A2 Robot: the FIRST reason wins, in the documented order -- the file's own checks before the "
          "device") {
    const auto absenceOf = [](cfgns::RobotConfig c, bool refuseCamera) {
        FakeWorld w;
        if (refuseCamera) {
            w.devices.refusePort(trr::kCameraPort);
        }
        Robot robot{c, w.devices};
        return std::string{robot.toggleCameraAbsence()};
    };
    cfgns::RobotConfig c = trr::robotFile();
    c.sensors.camera.placement.measured = false;
    c.sensors.toggleCamera.tracker.holdSeconds = 0.0;
    c.sensors.toggleCamera.feed.minReadPeriodSeconds = 0.0;
    c.sensors.toggleCamera.tracker.ids.blue = 6;
    CHECK(absenceOf(c, true).rfind("the camera's mount is UNSET", 0) == 0);  // mount, before all of them
    c.sensors.camera.placement.measured = true;
    CHECK(absenceOf(c, true).rfind("the Toggle block's tracker: holdSeconds UNSET", 0) == 0);
    c.sensors.toggleCamera.tracker.holdSeconds = 0.15;
    CHECK(absenceOf(c, true).rfind("the Toggle block's feed:", 0) == 0);
    c.sensors.toggleCamera.feed.minReadPeriodSeconds = 0.033;
    CHECK(absenceOf(c, true).rfind("the Toggle block: its BLUE id", 0) == 0);
    c.sensors.toggleCamera.tracker.ids.blue = 2;
    CHECK(absenceOf(c, true) == kNotBuilt);  // the file is right: now the device
    CHECK(absenceOf(c, false).empty());
    // The tracker's ids are checked BEFORE they are matched to the camera's colours (an UNSET id says so).
    cfgns::RobotConfig d = trr::robotFile();
    d.sensors.toggleCamera.tracker.ids = {};
    d.sensors.toggleCamera.tracker.holdSeconds = 0.15;  // still declared through its other numbers
    CHECK(absenceOf(d, false).rfind("the Toggle block's tracker: a Toggle colour id is UNSET", 0) == 0);
}

// ═══ Declared; the build-time check; the object ════════════════════════════════════════════════

// Bug caught: a `declared()` that misses a field — a block typed with only that number set would be silently
// IGNORED (no slot, no refusal, the step's NO CAMERA naming "no Toggle block" while the team typed one).
TEST_CASE("AF1S-A2 ToggleCameraConfig: ANY one of its 17 numbers set declares the block (a NaN too); all zero "
          "does not") {
    CHECK_FALSE(vision::ToggleCameraConfig{}.declared());
    const auto with = [](int k, double v) {
        vision::ToggleCameraConfig b{};
        vision::ToggleTrackerConfig& t = b.tracker;
        switch (k) {
            case 0: t.ids.red = static_cast<std::uint8_t>(v); break;
            case 1: t.ids.blue = static_cast<std::uint8_t>(v); break;
            case 2: t.ids.yellow = static_cast<std::uint8_t>(v); break;
            case 3: t.holdSeconds = v; break;
            case 4: t.aliveSeconds = v; break;
            case 5: t.minAspect = v; break;
            case 6: t.minLengthFraction = v; break;
            case 7: t.maxLengthFraction = v; break;
            case 8: t.minRangeIn = v; break;
            case 9: t.maxRangeIn = v; break;
            case 10: t.bearingWindowDeg = v; break;
            case 11: t.frozenMoveIn = v; break;
            case 12: t.frozenTurnDeg = v; break;
            case 13: t.positionToleranceIn = v; break;
            case 14: t.maxPoseToleranceIn = v; break;
            case 15: t.maxFixRadiusIn = v; break;
            default: b.feed.minReadPeriodSeconds = v; break;
        }
        return b;
    };
    for (int k = 0; k <= 16; ++k) {
        CAPTURE(k);
        CHECK(with(k, 1.0).declared());
        if (k >= 3) {
            CHECK(with(k, -0.5).declared());  // a negative number is set (and refused by name)
            CHECK(with(k, std::numeric_limits<double>::quiet_NaN()).declared());
            CHECK(with(k, std::numeric_limits<double>::denorm_min()).declared());
        }
        CHECK_FALSE(with(k, 0.0).declared());
    }
    // A NaN block is declared AND refused by name (never silently ignored).
    FakeWorld w;
    cfgns::RobotConfig c = withoutBlock();
    c.sensors.toggleCamera.feed.minReadPeriodSeconds = std::numeric_limits<double>::quiet_NaN();
    const Robot robot{c, w.devices};
    CHECK(robot.context().toggleCamera() != nullptr);
    CHECK(std::string{robot.toggleCameraAbsence()}.rfind("the Toggle block's tracker: a Toggle colour id is UNSET", 0) == 0);
}

namespace {
constexpr shulib::hal::ColourDescriptor kTaught[3] = {
    {.id = 1, .red = 200, .hueRange = 10.0F, .saturationRange = 0.2F, .name = "r", .provenance = "p"},
    {.id = 2, .blue = 200, .hueRange = 10.0F, .saturationRange = 0.2F, .name = "b", .provenance = "p"},
    {.id = 3, .red = 200, .green = 200, .hueRange = 10.0F, .saturationRange = 0.2F, .name = "y", .provenance = "p"}};
constexpr cfgns::CameraConfig kCamera{
    .port = 3,
    .settings = {.colours = {kTaught[0], kTaught[1], kTaught[2]}, .colourCount = 3},
    .placement = {.measured = true, .forward = Length{7.0}, .height = Length{10.0}, .positionTolerance = Length{0.25},
                  .heightTolerance = Length{0.25}, .yawToleranceDeg = 1.0, .pitchToleranceDeg = 1.0, .provenance = "p"}};
constexpr vision::ToggleCameraConfig kBlock{
    .tracker = {.ids = {1, 2, 3}, .holdSeconds = 0.15, .aliveSeconds = 0.1, .minAspect = 3.0, .minLengthFraction = 0.6,
                .maxLengthFraction = 1.3, .minRangeIn = 2.0, .maxRangeIn = 96.0, .bearingWindowDeg = 25.0,
                .frozenMoveIn = 2.0, .frozenTurnDeg = 2.0},
    .feed = {.minReadPeriodSeconds = 0.033}};
// THE BUILD-TIME FORM a robot file may write (robot_config.hpp's valid() note):
static_assert(vision::toggleCameraProblem(kCamera, kBlock) == nullptr);
static_assert(vision::toggleCameraProblem(kCamera, vision::ToggleCameraConfig{}) != nullptr);  // not declared
static_assert(vision::toggleCameraProblem(cfgns::CameraConfig{}, kBlock) != nullptr);          // no camera
}  // namespace

TEST_CASE("AF1S-A2 toggleCameraCheck: usable AT BUILD (static_assert), each refusal's part and sentence, and the "
          "SAME verdict the Robot reaches") {
    CHECK(vision::toggleCameraCheck(kCamera, kBlock).why == nullptr);
    const vision::ToggleCameraCheck none = vision::toggleCameraCheck(kCamera, vision::ToggleCameraConfig{});
    CHECK(std::string{none.part}.empty());
    CHECK(std::string{none.why} == kNoBlock);
    vision::ToggleCameraConfig bad = kBlock;
    bad.tracker.minAspect = 1.0;
    const vision::ToggleCameraCheck t = vision::toggleCameraCheck(kCamera, bad);
    CHECK(std::string{t.part} == "the Toggle block's tracker: ");
    CHECK(std::string{t.why} == "minAspect UNSET or not above 1 (a band is wider than tall)");
    // A Robot built from the same two blocks reaches the same verdict.
    FakeWorld w;
    cfgns::RobotConfig c = withoutBlock();
    c.sensors.toggleCamera = bad;
    const Robot robot{c, w.devices};
    CHECK(std::string{robot.toggleCameraAbsence()} == std::string{t.part} + t.why);
}

// Bug caught: an object that says "ready" before it is built, hands out a feed it does not have, or can be
// filled twice (a second fill would re-point a feed the step may hold).
TEST_CASE("AF1S-A2 ToggleCamera: EMPTY until built ('not built yet', no feed, no model); filled ONCE; '' exactly "
          "when ready") {
    vision::ToggleCamera slot;
    CHECK_FALSE(slot.ready());
    CHECK(slot.feed() == nullptr);
    CHECK(slot.model() == nullptr);
    CHECK(std::string{slot.absence()} == "not built yet");
    FakeWorld w;
    hal::fake::FakeVisionFrames cam;
    slot.build(&cam, w.clock, kCamera, kBlock);
    CHECK(slot.ready());
    CHECK(slot.feed() != nullptr);
    CHECK(slot.model() != nullptr);
    CHECK(std::string{slot.absence()}.empty());
    CHECK(cam.reads() == 0U);
    CHECK_THROWS_AS(slot.build(&cam, w.clock, kCamera, kBlock), shulib::PreconditionError);
    CHECK(slot.ready());  // the failed second fill changed nothing
    vision::ToggleCamera refused;
    refused.build(nullptr, w.clock, kCamera, kBlock);
    CHECK_FALSE(refused.ready());
    CHECK(std::string{refused.absence()} == kNotBuilt);
    CHECK_THROWS_AS(refused.build(&cam, w.clock, kCamera, kBlock), shulib::PreconditionError);
    CHECK_FALSE(refused.ready());
}

// ═══ The provider, valid(), the boot line ═════════════════════════════════════════════════════

// Bug caught: a slot that asks the provider for a device (a second camera, a read-fault count) — every graph
// built before AF1S-A2 must be exactly what it was, request for request.
TEST_CASE("AF1S-A2 Robot: the slot asks the provider for NOTHING -- the requests with the block are exactly the "
          "requests without it") {
    const auto requests = [](const cfgns::RobotConfig& c) {
        FakeWorld w;
        RecordingProvider rec{w.devices};
        const Robot robot{c, rec};
        return rec.asked;
    };
    const std::vector<std::string> without = requests(withoutBlock());
    const std::vector<std::string> with = requests(trr::robotFile());
    REQUIRE(std::find(without.begin(), without.end(), std::string{"aiVision 3"}) != without.end());
    CHECK(with == without);
    // And NOTHING after the camera, in either: the slot's fill is the constructor's last act, and it asks the
    // provider for nothing — not a device, not even its clock (the context's is the same one). A comparison of the
    // two lists alone cannot see a request made with or without a block alike; this can.
    REQUIRE_FALSE(with.empty());
    CHECK(with.back() == "aiVision 3");
    CHECK(without.back() == "aiVision 3");
}

// Bug caught (M-A2-2 at the config): valid() refusing a robot whose Toggle block is wrong — a typo in a
// tracker number would stop the robot from booting, driving and running every other auton.
TEST_CASE("AF1S-A2 config: valid() never checks the Toggle block -- a block with every number wrong is still a "
          "valid robot") {
    cfgns::RobotConfig c = trr::robotFile();
    c.sensors.toggleCamera.tracker.ids = {9, 9, 9};
    c.sensors.toggleCamera.tracker.holdSeconds = -1.0;
    c.sensors.toggleCamera.tracker.minRangeIn = std::numeric_limits<double>::infinity();
    c.sensors.toggleCamera.feed.minReadPeriodSeconds = std::numeric_limits<double>::quiet_NaN();
    char why[200] = "";
    CHECK(c.valid(why, sizeof why));
    CAPTURE(why);
    FakeWorld w;
    CHECK(shulib::robot::validate(c, w.devices).ok);
    std::optional<Robot> robot;
    REQUIRE_NOTHROW(robot.emplace(c, w.devices));
    CHECK(robot->toggleCamera() == nullptr);
    CHECK(std::string{robot->toggleCameraAbsence()} == "the Toggle block's tracker: a Toggle colour id is outside 1..7");
}

// Bug caught: the Toggle words pushing the drive out of a short buffer, a camera reason that now runs longer
// than it did (the camera's own words must stay bounded where they were), or a byte the brain cannot draw.
TEST_CASE("AF1S-A2 Robot: the boot line -- a short buffer cuts the Toggle words, never the drive; the camera's own "
          "words stay bounded where they were; ASCII") {
    FakeWorld w;
    cfgns::RobotConfig c = trr::robotFile();
    c.sensors.camera.placement.measured = false;
    Robot robot{c, w.devices};
    const std::string full = banner(robot);
    REQUIRE(endsWith(full, ", toggles OFF (the camera's mount is UNSET -- measure it from the robot's tracking "
                           "centre and type it)"));
    char buf[200];
    std::memset(buf, '\xAB', sizeof buf);
    robot.describe(buf, 150);
    const std::string s{buf};
    CHECK(s.size() == 149U);
    CHECK(buf[150] == '\xAB');
    CHECK(s.find("L -16,17,-18,-20 R 11,-12,13,15") != std::string::npos);
    CHECK(full.substr(0, 149) == s);
    // A huge camera reason: the camera's own words are cut at 223 characters, as before AF1S-A2, and the
    // Toggle words still follow them, whole.
    const std::string huge(600, 'x');
    w.devices.fakeCamera(trr::kCameraPort).setStatus({true, "", hal::ModelCheck::Different, huge.c_str()});
    const std::string b = banner(robot);
    const std::size_t at = b.find("camera port 3 set up, model DIFFERENT (xxx");
    CAPTURE(b);
    REQUIRE(at != std::string::npos);
    const std::size_t toggles = b.find(", toggles OFF (");
    REQUIRE(toggles != std::string::npos);
    CHECK(toggles - at == 223U);  // kCameraOwnWordsBytes - 1: what the camera's words always had
    CHECK(endsWith(b, "type it)"));
    CHECK(ascii(b));
    // The same reason with no block: the line ends where the camera's own words end — as it always did.
    FakeWorld w2;
    Robot plain{withoutBlock(), w2.devices};
    w2.devices.fakeCamera(trr::kCameraPort).setStatus({true, "", hal::ModelCheck::Different, huge.c_str()});
    const std::string p = banner(plain);
    CHECK(p.size() - p.find("camera port 3 set up, model DIFFERENT (xxx") == 223U);
}
