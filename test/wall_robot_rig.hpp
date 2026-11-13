#pragma once
//
// wall_robot_rig.hpp — a robot::Robot built from a robot file that DECLARES its side distance sensors
// (chunk L6-A2), over the host fake provider and the host test plant: "the robot plugged in" for the
// everyday wall reset (competition/auton_wall_reset.hpp). competition_runner_test.cpp's World, with walls.
//
// The robot is robot two's shape — a four-motor-a-side tank on drive-encoder odometry, robot two's
// real drive ports (L -16,17,-18,-20  R 11,-12,13,15, IMU 2) — with a side sensor on port 4 (left) and
// 5 (right), robot two's census ports. The sensors' TRUE mounts live in the field plant's side
// sensors (sim::SideSensorPlant — wall_reset_rig.hpp's trueLeft()/trueRight(): 7.5 in out, at the
// fore-aft centre, 10.5 in up); the mounts the ROBOT FILE types are `wall_rig::believedFrom` of them.
//
// The provider serves its OWN FakeDistance per port (it has no bind hook for a distance sensor), so
// the pacer copies each side plant's published reading — and its read-fault state — into the
// provider's fakes on ports 4 and 5 after every plant step: the Robot reads exactly what the plant
// published, through the provider, as it would on a brain.
//
// The Chassis's pacer is a RunGuard over that pacer (BuildOptions::pacer), so competition::runAuton —
// the competition program's call — runs an auton on it and the everyday call samples through the
// guard's own wait. Every pace is capped (motion_rig's rule) and the memory rule caps the binary.

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <stdexcept>

#include "motion_test_rig.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/competition/match_runner.hpp"
#include "shulib/config/robot_config.hpp"
#include "shulib/config/wall_sensor_config.hpp"
#include "shulib/hal/drivetrain_description.hpp"
#include "shulib/hal/fake/fake_device_provider.hpp"
#include "shulib/hal/fake/fake_motor.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/motion/motion_scheduler.hpp"
#include "shulib/robot/robot.hpp"
#include "shulib/sequence/run_guard.hpp"
#include "shulib/sim/field_plant.hpp"
#include "shulib/sim/scenario.hpp"
#include "shulib/units/quantity.hpp"
#include "wall_reset_rig.hpp"

namespace wall_robot {

inline constexpr std::int8_t kLeftPorts[4] = {-16, 17, -18, -20};
inline constexpr std::int8_t kRightPorts[4] = {11, -12, 13, 15};
inline constexpr std::uint8_t kImuPort = 2;
inline constexpr std::uint8_t kLeftWallPort = 4;
inline constexpr std::uint8_t kRightWallPort = 5;
inline constexpr double kTrackWidth = 12.0;
inline constexpr double kWheelDiameter = 4.0;  // NOT the plant's default (robot_builder_test's reason)
/// The match period the runs below are judged against (the program's constant; COMP's C6).
inline constexpr double kMatchSeconds = 30.0;

/// The robot file's side sensors for the plant's TRUE mounts: ports 4 and 5, each mount the tape
/// would give (wall_rig::believedFrom), the library's limits.
inline shulib::config::WallSensorsConfig wallsFor(const shulib::sim::SideSensorPlantConfig& l,
                                                  const shulib::sim::SideSensorPlantConfig& r) {
    shulib::config::WallSensorsConfig w{};
    w.left.port = kLeftWallPort;
    w.left.mount = wall_rig::believedFrom(l);
    w.right.port = kRightWallPort;
    w.right.mount = wall_rig::believedFrom(r);
    w.note = "host test plant: both centred, 10.5 in up";
    return w;
}

/// Robot two's shape (header) with `walls` as its side sensors.
inline shulib::config::RobotConfig robotWith(const shulib::config::WallSensorsConfig& walls) {
    shulib::hal::DrivetrainDescription d{.cartridge = shulib::hal::MotorCartridge::Blue,
                                         .imuPort = kImuPort,
                                         .trackWidth = shulib::units::Length{kTrackWidth},
                                         .source = shulib::hal::DescriptionSource::RobotMeasured,
                                         .provenance = "L6-A2 host fixture: never measured on a robot"};
    for (std::size_t i = 0; i < 4; ++i) {
        d.left.signedPorts[i] = kLeftPorts[i];
        d.right.signedPorts[i] = kRightPorts[i];
    }
    d.left.count = 4;
    d.right.count = 4;
    d.left.inchesPerRadian = shulib::units::Length{0.5 * kWheelDiameter};
    d.right.inchesPerRadian = shulib::units::Length{0.5 * kWheelDiameter};
    shulib::config::RobotConfig c{
        .identity = {.name = "Wall Bot", .team = "1234A", .season = "Override"},
        .drivetrain = {.kind = shulib::config::DrivetrainKind::Tank, .tank = {.description = d}},
        .odometry = {.kind = shulib::config::OdometryKind::DriveEncoders}};
    c.sensors.walls = walls;
    c.tuning.motion = motion_rig::motionConfig();
    return c;
}

/// The plant world's pacer (header): the drive plant one tick, both side plants at the new TRUE
/// pose, and their published values copied into the provider's fakes on ports 4 and 5.
struct WallWorldPacer final : shulib::motion::ITickPacer {
    WallWorldPacer(shulib::sim::SimHarness& harness, shulib::sim::SideSensorPlant& l,
                   shulib::sim::SideSensorPlant& r)
        : h{&harness}, left{&l}, right{&r} {}

    void pace() override {
        if (paces >= cap) {
            throw std::runtime_error("WallWorldPacer: pace cap exceeded — a wait failed to end");
        }
        ++paces;
        h->plant().step(shulib::units::Time{0.01});
        publish();
    }

    /// The side plants at the current TRUE pose, copied into the provider's fakes.
    void publish() {
        const shulib::math::Pose2d t = h->truePose();
        left->step(t);
        right->step(t);
        if (devices != nullptr) {
            mirror(*left, kLeftWallPort);
            mirror(*right, kRightWallPort);
        }
    }

    void mirror(shulib::sim::SideSensorPlant& plant, std::uint8_t port) {
        devices->fakeDistance(port).setDistance(plant.sensor().distance());
        devices->fakeDistance(port).setConfidence(plant.sensor().confidence());
        devices->fakeReadFaults(port).failing(plant.faults().isFailing());
    }

    shulib::sim::SimHarness* h;
    shulib::sim::SideSensorPlant* left;
    shulib::sim::SideSensorPlant* right;
    shulib::hal::fake::FakeDeviceProvider* devices = nullptr;  ///< set once the provider exists
    int paces = 0;
    int cap = 20000;
};

/// The world: the plant at TRUE pose `start`, the field and its side sensors, the provider bound to
/// the plant's motors and IMU, a RunGuard over the pacer — and the Robot, built by `build()`.
struct WallWorld {
    shulib::kinematics::TankKinematics kin{shulib::units::Length{kTrackWidth}};
    shulib::sim::SimHarness h;
    shulib::hal::fake::FakeTelemetrySink sink;
    shulib::sim::FieldModel field;
    shulib::sim::SideSensorPlant leftPlant;
    shulib::sim::SideSensorPlant rightPlant;
    WallWorldPacer pacer;
    shulib::hal::fake::FakeDeviceProvider devices;
    shulib::sequence::RunGuard guard{pacer};
    std::optional<shulib::robot::Robot> robot;

    explicit WallWorld(const shulib::math::Pose2d& start,
                       const shulib::sim::SideSensorPlantConfig& l = wall_rig::trueLeft(),
                       const shulib::sim::SideSensorPlantConfig& r = wall_rig::trueRight(),
                       const shulib::sim::FieldModel& fieldModel = shulib::sim::FieldModel::override2026())
        : h{kin, configAt(start)},
          sink{},
          field{fieldModel},
          leftPlant{field, l},
          rightPlant{field, r},
          pacer{h, leftPlant, rightPlant},
          devices{h.clock(), h.battery(), sink, pacer} {
        const shulib::hal::DriveSideDescription* sides[2] = {nullptr, nullptr};
        const shulib::config::RobotConfig shape = robotWith({});
        sides[0] = &shape.drivetrain.tank.description.left;
        sides[1] = &shape.drivetrain.tank.description.right;
        for (int wheel = 0; wheel < 2; ++wheel) {
            const shulib::hal::DriveSideDescription& side = *sides[wheel];
            devices.bindMotor(static_cast<std::uint8_t>(
                                  shulib::hal::DrivetrainDescription::portMagnitude(side.signedPorts[0])),
                              h.motor(wheel));
            std::array<shulib::hal::fake::FakeMotor*, 4> extras{};
            for (std::size_t i = 1; i < side.count; ++i) {
                extras[i - 1] = &devices.fakeMotor(static_cast<std::uint8_t>(
                    shulib::hal::DrivetrainDescription::portMagnitude(side.signedPorts[i])));
            }
            h.plant().attachCoupledMembers(
                wheel, std::span<shulib::hal::fake::FakeMotor* const>{extras.data(), static_cast<std::size_t>(side.count) - 1U});
        }
        devices.bindImu(kImuPort, h.imu());
        pacer.devices = &devices;
        pacer.publish();
    }

    static shulib::sim::SimHarnessConfig configAt(const shulib::math::Pose2d& start) {
        shulib::sim::SimHarnessConfig cfg = motion_rig::plantConfig();
        cfg.plant.initialPose = start;
        cfg.plant.driveWheelDiameter = shulib::units::Length{kWheelDiameter};
        return cfg;
    }

    /// Build the Robot from `cfg` (the guard as its pacer). Returns it.
    shulib::robot::Robot& build(const shulib::config::RobotConfig& cfg) {
        robot.emplace(cfg, devices, shulib::robot::BuildOptions{.pacer = &guard});
        return *robot;
    }

    /// One auton through competition::runAuton — the competition program's call — on the Robot.
    shulib::competition::AutonRunResult run(const shulib::competition::AutonEntry& e) {
        return shulib::competition::runAuton(robot->chassis(), guard, e, nullptr, sink, {},
                                             shulib::units::Time{kMatchSeconds});
    }

    /// How many log lines `subsystem` has written, and the last one's text.
    int linesFrom(const char* subsystem) const {
        int n = 0;
        for (int i = 0; i < sink.size(); ++i) {
            if (sink.at(i).subsystem == subsystem) {
                ++n;
            }
        }
        return n;
    }
};

/// A MATCH entry for `fn`, starting at `start`, on a 25 s / 28 s schedule.
inline shulib::competition::AutonEntry entryFor(const char* name, shulib::competition::AutonRoutine fn,
                                                shulib::competition::StartPose start,
                                                double endActionAt = 25.0, double hardStopAt = 28.0) {
    shulib::competition::AutonEntry e{};
    e.name = name;
    e.kind = shulib::competition::AutonKind::Match;
    e.routine = fn;
    e.start = start;
    e.endActionAt = shulib::units::Time{endActionAt};
    e.hardStopAt = shulib::units::Time{hardStopAt};
    return e;
}

}  // namespace wall_robot
