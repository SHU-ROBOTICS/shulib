#pragma once
//
// toggle_robot_rig.hpp — "the robot plugged in" for the EVERYDAY Toggle step (chunk AF1S-A2, 2026-11-15): a
// REAL robot::Robot, built from a robot file that declares the AI Vision camera AND its Toggle block, over the
// host fake provider, on the host test plants, run through competition::runAuton — the competition
// program's call — exactly as the competition program and the Auton Tester run it. Say "host test plant",
// never "simulator": every number here is the TEST's (invented); nothing has run on a robot.
//
// One pacer chain, the competition program's own (competition_program.cpp, B7):
//
//     Chassis -> RunGuard -> MechanismService -> World (the plants step, the clock advances)
//
// THE DRIVE is robot two's shape (test/wall_robot_rig.hpp's robotWith — a four-motor-a-side tank on drive
// encoders), each side's first port bound to the A2 drive plant and the rest coupled to it, as L6-A2's rig
// does. THE CAMERA is the robot file's (port 3, three colours taught: RED 1, BLUE 2, YELLOW 3; the mount
// VIS1-A2's rig believes, 10 in up, MEASURED); the provider serves its OWN FakeVisionFrames there, so the
// world renders the host camera model (sim::CameraPlant — VIS1-A2's TRUE camera, moved inside the stated
// tolerances) from the drive plant's TRUE pose after every pace and hands that frame to the provider's
// fake: the Robot's feed reads exactly what the plant published, through the provider, as on a brain. THE
// TOGGLE BLOCK is VIS1-A2's probe numbers (camrig::tcfg) and a 0.033-s read period. THE MECHANISMS are the
// recipe robot's (namespace toggle_robot below — a robot file's words): a FLIPPER (an Arm, AF1S-A1's rig's
// numbers, on port 5, on that rig's arm plant) and a CASCADE (a stall-homed Lift with a MEASURED shape, the
// cascade-like declaration RPV1-B1's preview test homes, on ports 6 and 7, on a plant built from the
// declaration's own numbers — test/fakes_by_kind.hpp's armPlantFrom). THE TOGGLES are sim::TogglePlant
// (AF1S-A1), one per wall the test asks for, each reading the flipper's TRUE angle and the robot's TRUE
// front and drawn into the camera model every pace.
//
// Test-only (no TEST_CASE here). Every pace is capped (motion_rig's rule) and the memory rule caps the binary.

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

#include "camera_scene_rig.hpp"
#include "fakes_by_kind.hpp"
#include "motion_test_rig.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/competition/match_runner.hpp"
#include "shulib/competition/toggle_visit.hpp"
#include "shulib/config/robot_config.hpp"
#include "shulib/hal/drivetrain_description.hpp"
#include "shulib/hal/fake/fake_device_provider.hpp"
#include "shulib/hal/fake/fake_motor.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/mechanism/kinds.hpp"
#include "shulib/mechanism/service.hpp"
#include "shulib/mechanism/wiring.hpp"
#include "shulib/robot/robot.hpp"
#include "shulib/sequence/run_guard.hpp"
#include "shulib/sim/camera_plant.hpp"
#include "shulib/sim/mechanism_plant.hpp"
#include "shulib/sim/scenario.hpp"
#include "shulib/sim/toggle_plant.hpp"
#include "shulib/units/literals.hpp"
#include "shulib/units/quantity.hpp"
#include "shulib/vision/toggle_camera.hpp"
#include "toggle_visit_rig.hpp"
#include "wall_robot_rig.hpp"

// ═══ THE RECIPE ROBOT'S MECHANISMS — a robot file's words (every number the TEST's) ═══════════════════════
namespace toggle_robot {
using namespace shulib::mechanism::words;
using namespace shulib::units::literals;

/// The cascade's measured shape: a row at the bottom stop and one per 8 in (the cascade-like declaration
/// test/route_preview_test.cpp's B1 case homes and lifts on the host test plant).
inline constexpr HeightAt kCascadeRows[] = {{0_in, 0}, {8_in, 360}, {16_in, 720}, {24_in, 1080}, {40_in, 1800}};

/// THE CASCADE: two motors, homed by stalling gently DOWN into its bottom stop (no limit switch), three levels.
inline constexpr Lift cascade{
    .name = "cascade",
    .motors = {Forward, Reversed},
    .cartridge = Red,
    .gearing = gears(12, 60),
    .shape = measured(kCascadeRows),
    .safe = Hold,
    .stall = {.amps = 1.0, .belowMotorRpm = 5, .lasting = 0.3_s},
    .currentCap = {.amps = 1.5},
    .range = {0_in, 40_in},
    .gravity = constantGravity(1.0_volt),
    .home = homeByStall(0_in, -6_volt, {.amps = 1.0, .belowMotorRpm = 5, .lasting = 0.3_s}),
    .presets = {{"floor", 0_in}, {"travel", 4_in}, {"score", 16_in}},
    .unhomedVolts = 4_volt,
};

/// THE FLIPPER: a motor arm, stowed on its stop at 0, flipped out to 90 (AF1S-A1's rig's arm, renamed).
inline constexpr Arm flipper{
    .name = "flipper",
    .motors = {Forward},
    .cartridge = Green,
    .gearing = gears(12, 60),
    .safe = Hold,
    .stall = {.amps = 1.0, .belowMotorRpm = 5, .lasting = 0.15_s},
    .range = {0, 120},
    .gravity = noGravity,
    .home = restsAtStop(0),
    .presets = {{"stow", 0}, {"flip", 90}},
};

/// Where each one is plugged in.
inline constexpr MechanismTable kToggleRobotMechanisms = mechanisms(wire(flipper, motors(5)), wire(cascade, motors(6, 7)));
}  // namespace toggle_robot

namespace toggle_robot_rig {

namespace sc = shulib::competition;
namespace sim = shulib::sim;
namespace vision = shulib::vision;
namespace fake = shulib::hal::fake;
namespace mech = shulib::mechanism;
namespace cfgns = shulib::config;
namespace ov = shulib::vision::override2026;
using shulib::units::Length;
using shulib::units::Time;

inline constexpr std::uint8_t kCameraPort = 3;
inline constexpr std::uint8_t kFlipperPort = 5;
inline constexpr std::uint8_t kCascadePorts[2] = {6, 7};
inline constexpr double kLensHeight = 10.0;

/// One colour descriptor the robot file types (every value the test's; a provenance, as the check demands).
inline shulib::hal::ColourDescriptor taught(std::uint8_t id, const char* name) {
    shulib::hal::ColourDescriptor d{};
    d.id = id;
    d.red = 200;
    d.green = 40;
    d.blue = 40;
    d.hueRange = 10.0F;
    d.saturationRange = 0.2F;
    d.name = name;
    d.provenance = "host test fixture: never measured";
    return d;
}

/// The camera's settings: the Toggle's three faces taught (RED 1, BLUE 2, YELLOW 3), the AI model off.
inline shulib::hal::AiVisionSettings toggleColours() {
    shulib::hal::AiVisionSettings s{};
    s.colours[0] = taught(tv_rig::kRed, "toggle red");
    s.colours[1] = taught(tv_rig::kBlue, "toggle blue");
    s.colours[2] = taught(tv_rig::kYellow, "toggle yellow");
    s.colourCount = 3;
    return s;
}

/// The Toggle block (VIS1-A2's probe numbers; the team's come from Step 0.3).
inline vision::ToggleCameraConfig toggleBlock() {
    vision::ToggleCameraConfig b{};
    b.tracker = camrig::tcfg();
    b.feed.minReadPeriodSeconds = 0.033;
    return b;
}

/// The recipe robot's file: robot two's drive shape (wall_robot::robotWith), the camera on port 3 MEASURED,
/// the Toggle block declared, the flipper and the cascade.
inline cfgns::RobotConfig robotFile() {
    cfgns::RobotConfig c = wall_robot::robotWith({});
    std::snprintf(c.identity.name, sizeof c.identity.name, "%s", "Toggle Bot");
    c.sensors.camera.port = kCameraPort;
    c.sensors.camera.settings = toggleColours();
    c.sensors.camera.placement = camrig::believed(kLensHeight);
    c.sensors.camera.note = "front, centred";
    c.sensors.toggleCamera = toggleBlock();
    const mech::MechanismTable& t = toggle_robot::kToggleRobotMechanisms;
    for (std::size_t i = 0; i < t.count; ++i) {
        c.mechanisms[i] = mech::toMechanismConfig(t.specs[i], t.wiring[i]);
    }
    c.mechanismCount = static_cast<std::uint8_t>(t.count);
    return c;
}

/// One Toggle the world carries: its plant's config (on its wall).
struct ToggleSpec {
    ov::ToggleId wall = ov::toggles::kRightWall;
    sim::TogglePlantConfig plant = tv_rig::redSideToggle();
};

/// The rig's options.
struct Options {
    shulib::math::Pose2d start = tv_rig::parkedAt(ov::toggles::kRightWall, 16.0);  ///< the plant's TRUE start
    std::vector<ToggleSpec> toggles = {ToggleSpec{}};  ///< the Toggles on the field (default: the right wall's)
    double cascadeStartIn = 0.0;  ///< where the cascade truly is at boot, inches above its bottom stop
    std::uint32_t seed = 7;       ///< the camera's jitter seed
};

struct World;

/// The world below the service (header): every plant one 10-ms tick, then the camera's frame published.
struct WorldPacer final : shulib::motion::ITickPacer {
    World* w = nullptr;
    long paces = 0;
    long cap = 30000;
    std::function<void()> onPace;  ///< runs FIRST each pace (a test's script)
    void pace() override;
};

/// The robot plugged in (header). Members in LIFETIME order: the plants and the provider before the
/// service, the service before the guard, the guard before the Robot (its pacer for the Robot's life).
struct World {
    Options opt;
    shulib::kinematics::TankKinematics kin{Length{wall_robot::kTrackWidth}};
    shulib::sim::SimHarness h;
    fake::FakeTelemetrySink sink;
    WorldPacer pacer;
    fake::FakeDeviceProvider devices;
    fake::FakeMotor* flipperF[1];
    fake::FakeMotor* cascadeF[2];
    sim::ArmPlant flipperPlant;
    sim::ArmPlant cascadePlant;
    std::vector<std::unique_ptr<sim::TogglePlant>> toggles;
    std::vector<sim::TogglePlantConfig> toggleConfigs;
    std::vector<sim::ColourQuad> extras;  ///< other things in the camera's scene
    sim::CameraPlant camPlant;
    mech::MechanismService svc{pacer};
    shulib::sequence::RunGuard guard{svc};
    std::optional<shulib::robot::Robot> robot;

    static shulib::sim::SimHarnessConfig harness(const Options& o) {
        shulib::sim::SimHarnessConfig cfg = motion_rig::plantConfig();
        cfg.plant.initialPose = o.start;
        cfg.plant.driveWheelDiameter = Length{wall_robot::kWheelDiameter};
        return cfg;
    }
    static sim::ArmPlantConfig cascadeConfig(const Options& o) {
        const mech::MechanismTable& t = toggle_robot::kToggleRobotMechanisms;
        sim::ArmPlantConfig c = tester_rig::FakesByKind::armPlantFrom(t.specs[1]);
        // The cascade truly sits `cascadeStartIn` up at boot; its encoders read 0 there (where the program
        // started) — what a stall home is FOR.
        c.startDeg = mech::positionToAngleDeg(t.specs[1].position, o.cascadeStartIn);
        c.encoderZeroDeg = c.startDeg;
        return c;
    }

    explicit World(const Options& o = {})
        : opt{o},
          h{kin, harness(o)},
          devices{h.clock(), h.battery(), sink, pacer},
          flipperF{&devices.fakeMotor(kFlipperPort)},
          cascadeF{&devices.fakeMotor(kCascadePorts[0]), &devices.fakeMotor(kCascadePorts[1])},
          flipperPlant{flipperF, tv_rig::flipArmPlant()},
          cascadePlant{cascadeF, cascadeConfig(o)},
          camPlant{camrig::truth(kLensHeight), o.seed} {
        // The drive, as test/wall_robot_rig.hpp binds it: each side's first port to the plant, the rest
        // coupled to it.
        const cfgns::RobotConfig shape = wall_robot::robotWith({});
        const shulib::hal::DriveSideDescription* sides[2] = {&shape.drivetrain.tank.description.left,
                                                             &shape.drivetrain.tank.description.right};
        for (int wheel = 0; wheel < 2; ++wheel) {
            const shulib::hal::DriveSideDescription& side = *sides[wheel];
            devices.bindMotor(static_cast<std::uint8_t>(
                                  shulib::hal::DrivetrainDescription::portMagnitude(side.signedPorts[0])),
                              h.motor(wheel));
            std::array<fake::FakeMotor*, 4> more{};
            for (std::size_t i = 1; i < side.count; ++i) {
                more[i - 1] = &devices.fakeMotor(static_cast<std::uint8_t>(
                    shulib::hal::DrivetrainDescription::portMagnitude(side.signedPorts[i])));
            }
            h.plant().attachCoupledMembers(
                wheel, std::span<fake::FakeMotor* const>{more.data(), static_cast<std::size_t>(side.count) - 1U});
        }
        devices.bindImu(wall_robot::kImuPort, h.imu());
        devices.fakeCamera(kCameraPort).setStatus({true, "", shulib::hal::ModelCheck::NotAsked, ""});
        camPlant.setJitterPx(1.0);
        for (const ToggleSpec& t : o.toggles) {
            const sim::TogglePlantConfig c = tv_rig::onWall(t.plant, t.wall);
            toggleConfigs.push_back(c);
            toggles.push_back(std::make_unique<sim::TogglePlant>(c));
            toggles.back()->attachArm(flipperPlant);
            toggles.back()->setOurs(sc::oursFor(t.wall) == vision::ToggleColour::Red ? tv_rig::kRed : tv_rig::kBlue);
        }
        pacer.w = this;
        publish();
    }

    /// Build the Robot from `cfg` (the guard as its pacer) and bind the service over its own mechanisms —
    /// the competition program's order (B7). Returns it.
    shulib::robot::Robot& build(const cfgns::RobotConfig& cfg = robotFile()) {
        robot.emplace(cfg, devices, shulib::robot::BuildOptions{.pacer = &guard});
        svc.bind(toggle_robot::kToggleRobotMechanisms, robot->mechanismParts().devices(), robot->context().clock(),
                 robot->faults(), sink);
        return *robot;
    }

    /// One auton through competition::runAuton — the competition program's call — with the service.
    sc::AutonRunResult run(const sc::AutonEntry& e) {
        return sc::runAuton(robot->chassis(), guard, e, nullptr, sink, svc, Time{60.0});
    }

    [[nodiscard]] shulib::math::Pose2d truth() { return h.plant().truePose(); }
    [[nodiscard]] double now() { return h.clock().now().value(); }

    /// The camera model redrawn from the Toggles and the extras, then rendered from the TRUE pose and handed to
    /// the provider's fake camera (header).
    void publish() {
        camPlant.clear();
        for (const sim::ColourQuad& q : extras) {
            camPlant.addQuad(q);
        }
        for (auto& t : toggles) {
            t->draw(camPlant);
        }
        const shulib::math::Pose2d p = truth();
        devices.fakeCamera(kCameraPort)
            .setNext(camPlant.read(p.x().value(), p.y().value(), p.heading().radians(), Time{now()}));
    }

    /// Lines of `subsystem`, in order.
    [[nodiscard]] std::vector<std::string> lines(const char* subsystem) const {
        std::vector<std::string> out;
        for (int i = 0; i < sink.size(); ++i) {
            if (sink.at(i).subsystem == subsystem) {
                out.push_back(sink.at(i).message);
            }
        }
        return out;
    }
};

inline void WorldPacer::pace() {
    if (paces >= cap) {
        throw std::runtime_error("toggle_robot_rig::WorldPacer: pace cap exceeded -- a wait failed to end");
    }
    ++paces;
    if (onPace) {
        onPace();
    }
    const double dt = 0.01;
    w->flipperPlant.step(dt);
    w->cascadePlant.step(dt);
    w->h.plant().step(Time{dt});
    for (std::size_t i = 0; i < w->toggles.size(); ++i) {
        const sim::TogglePlantConfig& c = w->toggleConfigs[i];
        w->toggles[i]->step(dt, w->flipperPlant.trueDeg(), tv_rig::frontToWall(w->truth(), c.wallX, c.wallY, c.inX, c.inY));
    }
    w->publish();
}

/// A SKILLS entry for `fn` (55 s / 58 s, inside the 60-s skills period), from `start`.
inline sc::AutonEntry skillsEntry(const char* name, sc::AutonRoutine fn, sc::StartPose start,
                                  sc::AutonKind kind = sc::AutonKind::Skills) {
    sc::AutonEntry e{};
    e.name = name;
    e.kind = kind;
    e.routine = fn;
    e.start = start;
    e.endActionAt = Time{kind == sc::AutonKind::Skills ? 55.0 : 25.0};
    e.hardStopAt = Time{kind == sc::AutonKind::Skills ? 58.0 : 28.0};
    return e;
}

/// The visit request the ctx scenes use: AF1S-A1's placeholder numbers (tv_rig::armRequest), on the recipe
/// robot's flipper.
inline sc::ToggleVisitRequest request(ov::ToggleId id = ov::toggles::kRightWall) {
    sc::ToggleVisitRequest r = tv_rig::armRequest(id);
    r.flipper = toggle_robot::flipper["flip"];
    r.back = toggle_robot::flipper["stow"];
    return r;
}

}  // namespace toggle_robot_rig
