// The robot file's SIDE DISTANCE SENSORS, built: robot::Robot over the fake provider — wallSensor(),
// wallSensorAbsence(), wallSensors(), the context's OPTIONAL slot and the scheduler's SHADOW context
// carrying it, the boot line's `walls` words — and the everyday wall reset SEATING through the Robot's
// own Chassis on the host test plant (chunk L6-A2; the L6 brief's §1.5, §1.6, D5 and test T13).
//
// ═══ WHAT THIS FILE TRIES TO BREAK ══════════════════════════════════════════════════
//   * D5 — A REFUSED SIDE SENSOR NEVER REFUSES THE ROBOT: a refused port, a provider that refuses with
//     no reason, one that THROWS — each builds a robot that drives, that side nullptr and its absence
//     saying why (mutation M27: the Robot refusing the build on a refused sensor).
//   * The absence is COPIED: the provider's sentence is valid only until its next refusal, so a later
//     refusal must not rewrite it (M28: the provider's pointer kept).
//   * wallSensorAbsence() is "" EXACTLY when wallSensor() is non-null; "not declared" for port 0, and
//     the provider is then never asked.
//   * THE SLOT reaches the Chassis: `robot.chassis().deps().ctx` is the scheduler's SHADOW context,
//     assembled field by field — a slot not carried there is nullptr in every chassis and every
//     everyday reset refuses NO SENSOR while every Robot test passes (M31: the shadow's line dropped).
//     A robot file that declares NO side hands no slot (NO SENSOR is then the true word); a context
//     built without the slot still constructs.
//   * The boot line's `walls` words in every case (both, one absent, mount unset, one side, none, and
//     the longest form, 41 characters, kept whole), ASCII, before the camera's — so a short buffer cuts the camera's words, never the walls'.
//   * The everyday call SEATS on the host test plant through the Robot's own Chassis and a running
//     RunGuard (competition::runAuton, a FIELD start): the brief's S1 through the everyday form.

#include "doctest.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "shulib/chassis/robot_context.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/competition/auton_wall_reset.hpp"
#include "shulib/config/robot_config.hpp"
#include "shulib/config/wall_sensor_config.hpp"
#include "shulib/core/check.hpp"
#include "shulib/hal/fake/fake_battery.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_device_provider.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/localization/override_walls.hpp"
#include "shulib/localization/wall_reset.hpp"
#include "shulib/motion/motion_scheduler.hpp"
#include "shulib/robot/robot.hpp"
#include "shulib/units/literals.hpp"
#include "shulib/vision/override_elements.hpp"
#include "wall_robot_rig.hpp"

namespace cfgns = shulib::config;
namespace hal = shulib::hal;
namespace loc = shulib::localization;
using shulib::hal::fake::FakeDeviceProvider;
using shulib::localization::WallSide;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::robot::Robot;
using shulib::units::Length;
using namespace shulib::units::literals;

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

/// Robot two's shape with both side sensors declared and measured (the plant's true mounts).
cfgns::RobotConfig bothSides() {
    return wall_robot::robotWith(wall_robot::wallsFor(wall_rig::trueLeft(), wall_rig::trueRight()));
}

std::string banner(const Robot& r) {
    char buf[400] = "";
    r.describe(buf, sizeof buf);
    return std::string{buf};
}

bool endsWith(const std::string& s, const std::string& tail) {
    return s.size() >= tail.size() && s.compare(s.size() - tail.size(), std::string::npos, tail) == 0;
}

/// A provider that forwards everything to the fake, but for the distance sensor on `port`: refuses it
/// with `reason` (Refuse), or throws (ThrowStd / ThrowOther).
struct OddDistance final : hal::IDeviceProvider {
    enum class Mode { Refuse, ThrowStd, ThrowOther };
    OddDistance(FakeDeviceProvider& in, std::uint8_t p, Mode m, const char* why = "")
        : inner{&in}, port{p}, mode{m}, reason{why} {}
    FakeDeviceProvider* inner;
    std::uint8_t port;
    Mode mode;
    const char* reason;
    bool refusedLast = false;
    int asks = 0;
    [[nodiscard]] hal::IMotor* motor(std::int8_t p, hal::MotorCartridge c) override { return inner->motor(p, c); }
    [[nodiscard]] hal::IImu* imu(std::uint8_t p) override { return inner->imu(p); }
    [[nodiscard]] hal::IGps* gps(std::uint8_t p, Length f, Length l) override { return inner->gps(p, f, l); }
    [[nodiscard]] hal::IRotation* rotation(std::uint8_t p) override { return inner->rotation(p); }
    [[nodiscard]] hal::IDigitalOut* digitalOut(std::uint8_t line, bool initial) override {
        return inner->digitalOut(line, initial);
    }
    [[nodiscard]] hal::IDistance* distance(std::uint8_t p) override {
        if (p != port) {
            refusedLast = false;
            return inner->distance(p);
        }
        ++asks;
        if (mode == Mode::ThrowStd) {
            throw std::runtime_error("cable fault");
        }
        if (mode == Mode::ThrowOther) {
            throw 42;  // not even a std::exception
        }
        refusedLast = true;
        return nullptr;
    }
    [[nodiscard]] hal::IOptical* optical(std::uint8_t p) override { return inner->optical(p); }
    [[nodiscard]] hal::IDigitalIn* digitalIn(std::uint8_t l) override { return inner->digitalIn(l); }
    [[nodiscard]] hal::IVisionFrames* aiVision(std::uint8_t p, const hal::AiVisionSettings& s) override {
        return inner->aiVision(p, s);
    }
    [[nodiscard]] const hal::IReadFaults* readFaults(std::uint8_t p) const noexcept override {
        return inner->readFaults(p);
    }
    [[nodiscard]] const hal::IReadFaults* lineReadFaults(std::uint8_t l) const noexcept override {
        return inner->lineReadFaults(l);
    }
    // MECH-S-A added the motor current limit to the provider (merged after L6-A2 was built); forwarded.
    [[nodiscard]] hal::IMotorCurrentLimit* currentLimit(std::uint8_t p) noexcept override {
        return inner->currentLimit(p);
    }
    [[nodiscard]] hal::IClock& clock() override { return inner->clock(); }
    [[nodiscard]] hal::IBattery& battery() override { return inner->battery(); }
    [[nodiscard]] hal::ITelemetrySink& telemetry() override { return inner->telemetry(); }
    [[nodiscard]] shulib::motion::ITickPacer& pacer() override { return inner->pacer(); }
    [[nodiscard]] const char* lastRefusal() const noexcept override {
        return refusedLast ? reason : inner->lastRefusal();
    }
};

/// A provider that forwards everything to the fake and RECORDS, in order, every device it is asked for
/// (the per-port read-fault lookups too) — the order a Robot's constructor makes its requests in.
struct RecordingProvider final : hal::IDeviceProvider {
    explicit RecordingProvider(FakeDeviceProvider& in) : inner{&in} {}
    FakeDeviceProvider* inner;
    mutable std::vector<std::string> asked;
    void note(const char* what, int port) const { asked.push_back(std::string{what} + " " + std::to_string(port)); }
    [[nodiscard]] hal::IMotor* motor(std::int8_t p, hal::MotorCartridge c) override {
        note("motor", p);
        return inner->motor(p, c);
    }
    // MECH-S-A added the motor current limit to the provider (merged after L6-A2 was built). A limit is a
    // view of a motor already asked for, not a device request, so it is forwarded and NOT recorded.
    [[nodiscard]] hal::IMotorCurrentLimit* currentLimit(std::uint8_t p) noexcept override {
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
    [[nodiscard]] hal::IClock& clock() override { return inner->clock(); }
    [[nodiscard]] hal::IBattery& battery() override { return inner->battery(); }
    [[nodiscard]] hal::ITelemetrySink& telemetry() override { return inner->telemetry(); }
    [[nodiscard]] shulib::motion::ITickPacer& pacer() override { return inner->pacer(); }
    [[nodiscard]] const char* lastRefusal() const noexcept override { return inner->lastRefusal(); }
};

}  // namespace

// ═══ Built ═════════════════════════════════════════════════════════════════════════

// Bug caught: a Robot that builds the sensors but hands the reset something else — the wrong port's
// sensor, no read-fault count (every reset then says "faults n/a" on a robot whose provider screens),
// the config's mount or limits not carried, or a slot that does not reach the Chassis.
TEST_CASE("L6 T13 the Robot builds both side sensors through the provider -- each with its read-fault count, "
          "its mount and the robot's limits -- and the slot reaches the Chassis's SHADOW context") {
    FakeWorld w;
    cfgns::RobotConfig c = bothSides();
    c.sensors.walls.limits.rangeMax = 14.0_in;  // a typed limit, carried as typed
    REQUIRE(c.valid());
    Robot robot{c, w.devices};

    REQUIRE(robot.wallSensor(WallSide::Left) != nullptr);
    CHECK(robot.wallSensor(WallSide::Left) == &w.devices.fakeDistance(wall_robot::kLeftWallPort));
    CHECK(robot.wallSensor(WallSide::Right) == &w.devices.fakeDistance(wall_robot::kRightWallPort));
    CHECK(std::string{robot.wallSensorAbsence(WallSide::Left)}.empty());
    CHECK(std::string{robot.wallSensorAbsence(WallSide::Right)}.empty());
    const loc::WallSensors& ws = robot.wallSensors();
    CHECK(ws.left.faults != nullptr);
    CHECK(ws.left.faults == w.devices.readFaults(wall_robot::kLeftWallPort));
    CHECK(ws.right.faults == w.devices.readFaults(wall_robot::kRightWallPort));
    CHECK(ws.left.faults != ws.right.faults);
    CHECK(ws.left.mount.measured);
    CHECK(ws.left.mount.left.value() == 7.5);
    CHECK(ws.right.mount.left.value() == -7.5);
    CHECK(ws.left.mount.height.value() == 10.5);
    CHECK(std::string{ws.left.mount.provenance} == "host test plant");
    CHECK(ws.limits.rangeMax.value() == 14.0);
    CHECK(ws.log.seated == 0);
    CHECK(ws.log.refused == 0);
    const Robot& constRobot = robot;
    CHECK(constRobot.wallSensor(WallSide::Left) == robot.wallSensor(WallSide::Left));

    // The slot: the Robot's context, and — the trap — the scheduler's shadow context every
    // chassis.deps().ctx is (M31). Filled in the constructor's BODY: what the shadow hands out is the
    // live object, sensors built.
    CHECK(robot.context().wallSensors() == &robot.wallSensors());
    loc::WallSensors* viaChassis = robot.chassis().deps().ctx->wallSensors();
    REQUIRE(viaChassis != nullptr);
    CHECK(viaChassis == &robot.wallSensors());
    CHECK(viaChassis->left.sensor == robot.wallSensor(WallSide::Left));
    CHECK(viaChassis->right.sensor == robot.wallSensor(WallSide::Right));
    CHECK(robot.chassis().deps().ctx != &robot.context());  // it IS the shadow, not the Robot's own

    CHECK(endsWith(banner(robot), "tags ABSENT walls L4 R5 camera ABSENT"));
}

// Bug caught (§1.5 — "LAST, after the camera"): the side sensors requested EARLIER in the constructor —
// above the camera, or above the mechanisms — so that adding a `walls` block to a robot file reorders
// every request after it, and a provider whose behaviour depends on what it was asked first (a port
// refused once, a device made once) answers the camera or a mechanism differently. Pinned as an
// equality: the requests with both sides declared are EXACTLY the requests without them, in order,
// followed by the side sensors' own and nothing else.
TEST_CASE("L6 T13 the side sensors are requested LAST -- every earlier request, the mechanism's and the "
          "camera's among them, exactly as it was without them") {
    const auto requests = [](bool withWalls) {
        FakeWorld w;
        RecordingProvider rec{w.devices};
        cfgns::RobotConfig c = withWalls ? bothSides() : wall_robot::robotWith({});
        c.mechanisms[0] = {.name = "claw pivot 6",
                           .motorPorts = {6},
                           .motorCount = 1,
                           .cartridge = hal::MotorCartridge::Red,
                           .safeMode = cfgns::SafeBrakeMode::Hold};
        c.mechanismCount = 1;
        c.sensors.camera.port = 3;
        c.sensors.camera.settings.objects = true;
        c.sensors.camera.settings.expectedClasses = shulib::vision::override2026::kOverrideClassNames;
        c.sensors.camera.settings.expectedClassCount = shulib::vision::override2026::kModelClassCount;
        REQUIRE(c.valid());
        const Robot robot{c, rec};
        return rec.asked;  // the constructor's requests only
    };
    const std::vector<std::string> without = requests(false);
    const std::vector<std::string> with = requests(true);
    const auto has = [](const std::vector<std::string>& v, const char* what) {
        return std::find(v.begin(), v.end(), std::string{what}) != v.end();
    };
    // The order is meaningful only if the earlier requests include the camera and the mechanism.
    REQUIRE(has(without, "aiVision 3"));
    REQUIRE(has(without, "motor 6"));
    CHECK_FALSE(has(without, "distance 4"));
    CHECK_FALSE(has(without, "distance 5"));
    REQUIRE(with.size() > without.size());
    const std::vector<std::string> head(with.begin(),
                                        with.begin() + static_cast<std::ptrdiff_t>(without.size()));
    CHECK(head == without);
    const std::vector<std::string> tail(with.begin() + static_cast<std::ptrdiff_t>(without.size()), with.end());
    CAPTURE(tail.size());
    CHECK(tail == std::vector<std::string>{"distance 4", "readFaults 4", "distance 5", "readFaults 5"});
}

// Bug caught: a robot file with no side declared handed the slot anyway — every reset would then
// refuse "sensor absent: not declared" where the true word is NO SENSOR (this robot file declares no
// wall sensors: add the block) — or the provider asked for a sensor nobody declared.
TEST_CASE("L6 T13 port 0 on both sides: no slot (the reset's NO SENSOR), 'not declared', the provider never "
          "asked, 'walls ABSENT'") {
    FakeWorld w;
    Robot robot{wall_robot::robotWith({}), w.devices};
    CHECK(robot.wallSensor(WallSide::Left) == nullptr);
    CHECK(robot.wallSensor(WallSide::Right) == nullptr);
    CHECK(std::string{robot.wallSensorAbsence(WallSide::Left)} == "not declared");
    CHECK(std::string{robot.wallSensorAbsence(WallSide::Right)} == "not declared");
    for (std::uint8_t p = 1; p <= 21; ++p) {
        CHECK(w.devices.readFaults(p) == nullptr);  // no distance (or optical) sensor was ever served
    }
    CHECK(robot.context().wallSensors() == nullptr);
    CHECK(robot.chassis().deps().ctx->wallSensors() == nullptr);
    CHECK(endsWith(banner(robot), "tags ABSENT walls ABSENT camera ABSENT"));

    // One side declared: the slot exists, the undeclared side reads "not declared".
    cfgns::WallSensorsConfig leftOnly = wall_robot::wallsFor(wall_rig::trueLeft(), wall_rig::trueRight());
    leftOnly.right = {};
    FakeWorld w2;
    Robot one{wall_robot::robotWith(leftOnly), w2.devices};
    CHECK(one.wallSensor(WallSide::Left) != nullptr);
    CHECK(one.wallSensor(WallSide::Right) == nullptr);
    CHECK(std::string{one.wallSensorAbsence(WallSide::Right)} == "not declared");
    CHECK(w2.devices.readFaults(wall_robot::kRightWallPort) == nullptr);
    CHECK(one.chassis().deps().ctx->wallSensors() == &one.wallSensors());
    CHECK(endsWith(banner(one), "tags ABSENT walls L4 camera ABSENT"));
}

// ═══ D5: refused, never refusing ═══════════════════════════════════════════════════

// Bug caught (M27 — the Robot refusing the build on a refused side sensor; M28 — the provider's
// pointer kept rather than copied): a cable out at an event must never cost the match, and the reason
// a side is absent must still say why after the provider has refused something else.
TEST_CASE("L6 T13 D5: a REFUSED side never refuses the robot -- it builds and DRIVES, that side absent with "
          "the provider's sentence, COPIED") {
    FakeWorld w;
    w.devices.refusePort(wall_robot::kRightWallPort);
    const cfgns::RobotConfig c = bothSides();
    const shulib::robot::BuildVerdict v = shulib::robot::validate(c, w.devices);
    CAPTURE(v.why);
    REQUIRE(v.ok);
    std::optional<Robot> built;
    REQUIRE_NOTHROW(built.emplace(c, w.devices));
    Robot& robot = *built;
    CHECK(robot.wallSensor(WallSide::Left) != nullptr);
    CHECK(robot.wallSensor(WallSide::Right) == nullptr);
    CHECK(robot.wallSensors().right.faults == nullptr);
    CHECK(std::string{robot.wallSensorAbsence(WallSide::Right)} == "port 5 has no distance sensor");
    CHECK(std::string{robot.wallSensors().right.absence} == "port 5 has no distance sensor");
    CHECK(robot.wallSensors().right.mount.measured);  // the mount is carried, sensor or not
    // It drives: the drive motors exist and take a command.
    REQUIRE(robot.context().driveMotors().size() == 2U);
    robot.context().driveMotors()[0]->setVoltage(shulib::units::Voltage{6.0});
    CHECK(w.devices.fakeMotor(16).commandedVoltage().value() == doctest::Approx(6.0));
    CHECK(endsWith(banner(robot), "tags ABSENT walls L4 R5(ABSENT) camera ABSENT"));

    // COPIED: the provider refuses something else; its sentence changes; ours does not (M28).
    w.devices.refusePort(9);
    CHECK(w.devices.distance(9) == nullptr);
    REQUIRE(std::string{w.devices.lastRefusal()} == "port 9 has no distance sensor");
    CHECK(std::string{robot.wallSensorAbsence(WallSide::Right)} == "port 5 has no distance sensor");

    // The LEFT refused instead: the order of the words follows the sides, not the refusal.
    FakeWorld w2;
    w2.devices.refusePort(wall_robot::kLeftWallPort);
    Robot leftGone{c, w2.devices};
    CHECK(leftGone.wallSensor(WallSide::Left) == nullptr);
    CHECK(leftGone.wallSensor(WallSide::Right) != nullptr);
    CHECK(std::string{leftGone.wallSensorAbsence(WallSide::Left)} == "port 4 has no distance sensor");
    CHECK(endsWith(banner(leftGone), "tags ABSENT walls L4(ABSENT) R5 camera ABSENT"));
}

TEST_CASE("L6 T13 D5: a provider that refuses with NO reason, one with a reason longer than the Robot keeps, "
          "and one that THROWS -- each leaves the side absent, named, and the robot built") {
    const cfgns::RobotConfig c = bothSides();

    FakeWorld w1;
    OddDistance silent{w1.devices, wall_robot::kLeftWallPort, OddDistance::Mode::Refuse, ""};
    Robot quiet{c, silent};
    CHECK(silent.asks == 1);
    CHECK(quiet.wallSensor(WallSide::Left) == nullptr);
    CHECK(std::string{quiet.wallSensorAbsence(WallSide::Left)}
          == "the device provider refused the wall sensor and gave no reason");
    CHECK(quiet.wallSensor(WallSide::Right) != nullptr);  // the other side is asked for all the same

    // A refusal LONGER than the Robot keeps is cut to its buffer, never copied past it.
    FakeWorld w2;
    const std::string huge = "port 4: " + std::string(600, 'x');
    OddDistance wordy{w2.devices, wall_robot::kLeftWallPort, OddDistance::Mode::Refuse, huge.c_str()};
    Robot longWinded{c, wordy};
    const std::string kept{longWinded.wallSensorAbsence(WallSide::Left)};
    CHECK(kept.size() == 159U);  // the 160-byte buffer, NUL included
    CHECK(kept == huge.substr(0, 159));

    FakeWorld w3;
    OddDistance throwing{w3.devices, wall_robot::kRightWallPort, OddDistance::Mode::ThrowStd};
    std::optional<Robot> survivor;
    REQUIRE_NOTHROW(survivor.emplace(c, throwing));
    CHECK(survivor->wallSensor(WallSide::Right) == nullptr);
    CHECK(survivor->wallSensors().right.faults == nullptr);
    CHECK(std::string{survivor->wallSensorAbsence(WallSide::Right)}
          == "the device provider threw making the RIGHT wall sensor: cable fault");
    CHECK(endsWith(banner(*survivor), "walls L4 R5(ABSENT) camera ABSENT"));

    FakeWorld w4;
    OddDistance odd{w4.devices, wall_robot::kLeftWallPort, OddDistance::Mode::ThrowOther};
    std::optional<Robot> survivor2;
    REQUIRE_NOTHROW(survivor2.emplace(c, odd));
    CHECK(survivor2->wallSensor(WallSide::Left) == nullptr);
    CHECK(std::string{survivor2->wallSensorAbsence(WallSide::Left)}
          == "the device provider threw making the LEFT wall sensor");
}

// ═══ The boot line ═════════════════════════════════════════════════════════════════

// Bug caught: a boot line that says the side sensors are fine when one is missing or its mount is
// unmeasured (the reset will refuse every time and nobody looking at the boot screen would know), or
// one whose wall words displace the drive's numbers from a short buffer.
TEST_CASE("L6 T13 the boot line's walls words: both, one absent, mount unset, one side, none -- exact, ASCII, "
          "before the camera's") {
    const std::string head =
        "Wall Bot [1234A] tank MEASURED | L -16,17,-18,-20 R 11,-12,13,15 | ipr L2.00000 R2.00000 W 12.000 in | "
        "IMU2 | drive-encoders | GPS ABSENT tags ABSENT ";
    {
        FakeWorld w;
        Robot r{bothSides(), w.devices};
        CHECK(banner(r) == head + "walls L4 R5 camera ABSENT");
    }
    {
        FakeWorld w;
        cfgns::RobotConfig c = bothSides();
        c.sensors.walls.left.mount.measured = false;  // UNSET: the robot still boots
        REQUIRE(c.valid());
        Robot r{c, w.devices};
        CHECK(r.wallSensor(WallSide::Left) != nullptr);  // built all the same
        CHECK(banner(r) == head + "walls L4 R5 MOUNT UNSET camera ABSENT");
    }
    {
        FakeWorld w;
        cfgns::RobotConfig c = bothSides();
        c.sensors.walls.right.mount.measured = false;
        Robot r{c, w.devices};
        CHECK(banner(r) == head + "walls L4 R5 MOUNT UNSET camera ABSENT");
    }
    {
        FakeWorld w;
        w.devices.refusePort(4);
        w.devices.refusePort(5);
        cfgns::RobotConfig c = bothSides();
        c.sensors.walls.left.mount.measured = false;
        Robot r{c, w.devices};
        CHECK(banner(r) == head + "walls L4(ABSENT) R5(ABSENT) MOUNT UNSET camera ABSENT");
    }
    {
        // The LONGEST walls form (a review finding of 2026-11-05: a buffer shrunk below it went
        // unnoticed while only one-digit ports were pinned): two two-digit ports, both refused, a mount
        // unmeasured — 41 characters, the whole of it kept.
        FakeWorld w;
        w.devices.refusePort(19);
        w.devices.refusePort(21);
        cfgns::RobotConfig c = bothSides();
        c.sensors.walls.left.port = 19;
        c.sensors.walls.right.port = 21;
        c.sensors.walls.right.mount.measured = false;
        REQUIRE(c.valid());
        Robot r{c, w.devices};
        const std::string longest = "walls L19(ABSENT) R21(ABSENT) MOUNT UNSET";
        REQUIRE(longest.size() == 41U);
        CHECK(banner(r) == head + longest + " camera ABSENT");
    }
    {
        FakeWorld w;
        cfgns::WallSensorsConfig rightOnly = wall_robot::wallsFor(wall_rig::trueLeft(), wall_rig::trueRight());
        rightOnly.left = {};
        rightOnly.right.port = 21;
        Robot r{wall_robot::robotWith(rightOnly), w.devices};
        CHECK(banner(r) == head + "walls R21 camera ABSENT");
    }
    {
        FakeWorld w;
        Robot r{wall_robot::robotWith({}), w.devices};
        CHECK(banner(r) == head + "walls ABSENT camera ABSENT");
    }
    {
        // A camera on port 3 too: the walls words come BEFORE the camera's, so a short buffer cuts the
        // camera's words and keeps the walls'.
        FakeWorld w;
        cfgns::RobotConfig c = bothSides();
        c.sensors.camera.port = 3;
        c.sensors.camera.settings.objects = true;
        c.sensors.camera.settings.expectedClasses = shulib::vision::override2026::kOverrideClassNames;
        c.sensors.camera.settings.expectedClassCount = shulib::vision::override2026::kModelClassCount;
        REQUIRE(c.valid());
        Robot r{c, w.devices};
        const std::string full = banner(r);
        CHECK(full.find("tags ABSENT walls L4 R5 camera port 3") != std::string::npos);
        char shortBuf[200];
        r.describe(shortBuf, head.size() + 15);
        const std::string cut{shortBuf};
        CHECK(cut == head + "walls L4 R5 ca");
        for (const char ch : full) {
            CHECK((static_cast<unsigned char>(ch) >= 0x20U && static_cast<unsigned char>(ch) < 0x7FU));
        }
    }
}

// ═══ The context's optional slot ═══════════════════════════════════════════════════

// Bug caught: the slot turned into a requirement — every existing caller (the host harness, the rigs,
// a hand-wired robot) would stop constructing — or a slot handed in that the context does not hand out.
TEST_CASE("L6 T13 the context's walls slot is OPTIONAL: a context built without it constructs and hands out "
          "nullptr; one built with it hands out the same pointer, and the scheduler's shadow carries it") {
    const shulib::kinematics::TankKinematics kin{Length{11.0}};
    motion_rig::MotionRig rig{kin};
    // The harness's own context (sim/scenario.hpp) never names the slot.
    CHECK(rig.h.context().wallSensors() == nullptr);
    loc::WallSensors walls{};
    const shulib::chassis::RobotContextConfig base{.clock = &rig.h.context().clock(),
                                                   .driveMotors = rig.h.context().driveMotors(),
                                                   .imu = &rig.h.context().imu(),
                                                   .gps = &rig.h.context().gps(),
                                                   .battery = &rig.h.context().battery(),
                                                   .telemetry = &rig.h.context().telemetry(),
                                                   .tags = &rig.h.context().tags(),
                                                   .vision = &rig.h.context().vision()};
    std::optional<shulib::chassis::RobotContext> without;
    REQUIRE_NOTHROW(without.emplace(base));
    CHECK(without->wallSensors() == nullptr);
    shulib::chassis::RobotContextConfig withCfg = base;
    withCfg.walls = &walls;
    shulib::chassis::RobotContext with{withCfg};
    CHECK(with.wallSensors() == &walls);
    // Every other slot is still required: the optional one did not loosen them.
    shulib::chassis::RobotContextConfig noVision = withCfg;
    noVision.vision = nullptr;
    CHECK_THROWS_AS(shulib::chassis::RobotContext{noVision}, shulib::PreconditionError);

    // The scheduler's shadow context carries the slot, null or not (M31).
    const shulib::motion::MotionDeps depsWith{.ctx = &with,
                                              .localizer = &rig.loc,
                                              .kinematics = rig.deps.kinematics,
                                              .faults = &rig.latch,
                                              .health = &rig.health};
    motion_rig::PlantPacer pacer{rig.h};
    const shulib::motion::MotionScheduler schedWith{depsWith, pacer};
    CHECK(schedWith.deps().ctx->wallSensors() == &walls);
    CHECK(schedWith.deps().ctx != &with);
    const shulib::motion::MotionScheduler schedWithout{rig.deps, pacer};
    CHECK(schedWithout.deps().ctx->wallSensors() == nullptr);
}

// ═══ The everyday call through the Robot (S1, the brief's T13 and T17) ══════════════

namespace {

loc::WallResetReport gSeatReport{};
/// D4's sequence in a FIELD-start auton: a leg ALONG the left wall, square up, reset x against it.
shulib::chassis::RoutineResult leftWallLeg(shulib::competition::AutonContext& ctx) {
    gSeatReport = {};
    shulib::chassis::Routine r{ctx, "left-wall-leg"};
    r.face(-55.7_in, -38_in);
    r.driveTo(-55.7_in, -38_in);
    r.turnTo(90_deg);
    r.then([&ctx] {
        gSeatReport = shulib::competition::resetFromWall(
            ctx, {.wall = loc::override2026::kLeftWall, .sensor = WallSide::Left, .maxCorrection = 3_in});
        return gSeatReport.continues();
    }, "left wall x");
    return r.result();
}

}  // namespace

// Bug caught: a Robot whose unit tests all pass but whose everyday reset cannot seat — the slot not
// reaching the Chassis (M31: every call NO SENSOR), the provider's sensor not the one the plant
// publishes into, the outcome never filed in the robot's log, or the line sent somewhere else.
TEST_CASE("L6 T13/T17 S1 through the everyday call: a FIELD-start auton on the Robot's own Chassis, the start "
          "typed 2.5 in off, SEATS x within 0.1 in of the truth, files it, one WALL line") {
    // Placed 2.5 in NEARER the left wall than the auton's start says (x -53.2 typed, -55.705 true):
    // the leg ends with the estimate at x -55.7 and the robot at about -58.2 — 4.5 in from the wall.
    const Pose2d placed{Length{-55.705}, Length{-50.0}, Angle::degrees(90.0)};
    wall_robot::WallWorld world{placed};
    Robot& robot = world.build(bothSides());
    const shulib::competition::AutonEntry e = wall_robot::entryFor(
        "left wall", &leftWallLeg,
        shulib::competition::StartPose::field(Length{-53.205}, Length{-50.0}, Angle::degrees(90.0)));

    const shulib::competition::AutonRunResult res = world.run(e);

    REQUIRE(res.outcome == shulib::competition::AutonRunOutcome::RoutineRan);
    CAPTURE(loc::wallResetVerdictWord(gSeatReport.verdict));
    REQUIRE(gSeatReport.seated());
    CHECK(res.routine.ok);
    const double truthX = world.h.truePose().x().value();
    CHECK(std::abs(robot.localizer().pose().x().value() - truthX) < 0.1);
    CHECK(gSeatReport.applied.value() == doctest::Approx(-2.5).epsilon(0.05));
    // Filed in the robot's own log — the one Robot::wallSensors() hands a program to read.
    CHECK(robot.wallSensors().log.seated == 1);
    CHECK(robot.wallSensors().log.refused == 0);
    CHECK(robot.wallSensors().log.last.seated());
    CHECK(robot.wallSensors().log.last.measured.value() == gSeatReport.measured.value());
    // RC31 / landmine 23 through the EVERYDAY form: the seat (2.5 in, over the pose guard's 2.25 in per
    // 10-ms tick) lands after the guard's wait returns, and what follows it — the run's own waits and
    // end — starts on a dt-0 tick: no advisory IMPLAUSIBLE, and no ODO_STUCK, from the deliberate jump.
    REQUIRE(std::abs(gSeatReport.applied.value()) > 2.25);
    CHECK(robot.faults().raiseCount(shulib::diag::FaultCode::Implausible) == 0);
    CHECK(robot.faults().raiseCount(shulib::diag::FaultCode::OdoStuck) == 0);
    // ONE WALL line, Info, through the run's telemetry.
    REQUIRE(world.linesFrom("WALL") == 1);
    for (int i = 0; i < world.sink.size(); ++i) {
        if (world.sink.at(i).subsystem == "WALL") {
            CHECK(world.sink.at(i).level == hal::LogLevel::Info);
            CHECK(world.sink.at(i).message.rfind("left wall x SEATED (LEFT sensor): ", 0) == 0);
        }
    }
}
