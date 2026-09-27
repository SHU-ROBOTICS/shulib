// robot::Robot — a config, wired, and the proof that it is the SAME robot
// (chunk G1 Part 1, brief §4 tests 4–10).
//
// ── What the load-bearing cases here actually prove ─────────────────────────────────
// Ruling B4: the builder adds a WAY TO WIRE and never a different physics. The only test that
// can show that is a differential one — build the same robot twice, once by hand and once from
// data, drive both across the A2 plant, and require the two runs to agree BIT FOR BIT: the
// per-tick ground-truth samples compared with memcmp (not Approx), the commanded volts compared
// as doubles, the exit reason and the tick count equal. A tolerance would hide exactly the class
// of defect this exists to catch: an object constructed with a slightly different argument, an
// extra corrector, a stall-check geometry that was defaulted instead of derived, a localizer
// updated in the wrong place in the loop.
//
// The longhand rigs below are therefore written out IN FULL rather than reusing a helper the
// builder also uses — a shared helper would make the two sides agree by construction and prove
// nothing. They are transcriptions of src/main.cpp's TankRobot (which "shulib Teleop" wired by
// hand until G1 Part 2b moved it onto robot::Robot) and X-drive Robot, and of
// chassis_facade_test.cpp's standalone case, with the harness standing in for hal/pros.
//
// ── NO HARDWARE CLAIM ───────────────────────────────────────────────────────────────
// Everything here runs on the A2 plant behind the F4 fakes. No config has built a graph over a
// real device: since G1 Part 2b "shulib Teleop" builds one over the PROS provider, and that
// version has not run on hardware; since G1 Part 2c the invented X-drive is built the same way
// (src/robots/xdrive_config.hpp — test/xdrive_config_test.cpp), and no program wires a graph by hand.

#include "doctest.h"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "mechanism_test_rig.hpp"
#include "motion_test_rig.hpp"
#include "shulib/config/robot_config.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/diag/session_info.hpp"
#include "shulib/hal/fake/fake_device_provider.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/robot/robot.hpp"
#include "shulib/sim/hostile/composed.hpp"
#include "shulib/sim/scenario.hpp"

namespace cfgns = shulib::config;
using motion_rig::motionConfig;
using motion_rig::plantConfig;
using shulib::control::ExitReason;
using shulib::hal::DescriptionSource;
using shulib::hal::DriveGeometry;
using shulib::hal::DrivetrainDescription;
using shulib::hal::IMotor;
using shulib::hal::MotorCartridge;
using shulib::hal::MotorGroup;
using shulib::hal::fake::FakeDeviceProvider;
using shulib::hal::fake::FakeMotor;
using shulib::hal::fake::FakeTelemetrySink;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::sim::SimHarness;
using shulib::sim::SimHarnessConfig;
using shulib::sim::TruthSample;
using shulib::units::Length;
using shulib::units::Time;

namespace {

// ── the port map every rig in this file uses ────────────────────────────────────────
// Five per side, as robot two's gear train has, with the signs a real wiring job produces.
constexpr std::int8_t kLeft[5] = {1, -2, 3, -4, 5};
constexpr std::int8_t kRight[5] = {-11, 12, -13, 14, -15};
constexpr std::uint8_t kImuPort = 10;
constexpr std::uint8_t kForwardPodPort = 6;
constexpr std::uint8_t kLateralPodPort = 7;
constexpr std::uint8_t kGpsPort = 9;
constexpr std::int8_t kXWheels[4] = {1, 2, 3, 4};  // FL, BL, BR, FR — the F5 order
constexpr double kTrackWidth = 12.0;
constexpr double kDriveRadius = 7.0;

/// The drive wheel this fixture's plant synthesizes encoders with, and the inches-per-radian a
/// description of it carries (a 1:1 shaft ratio, so the number is just the radius).
///
/// DELIBERATELY NOT 3.25 in, which is the A2 plant's own default AND the stand-in default of
/// hal::DriveGeometry and of OdoStallCheckConfig's wheel slots (A4: HA-14). Measured while
/// running mutation M8 (drop the builder's stall-geometry derivation): with a 3.25 in fixture the
/// mutant kept the DEFAULT geometry, the default happened to equal the fixture's, and the
/// mutation was a no-op that no test could have caught. A fixture that shares a value with the
/// library default cannot test that the value was plumbed.
constexpr double kPlantWheelDiameter = 4.0;
constexpr double kPlantInchesPerRadian = 0.5 * kPlantWheelDiameter;

/// A tank config with `membersPerSide` motors on each side and the given estimator.
cfgns::RobotConfig tankConfig(std::size_t membersPerSide, cfgns::OdometryKind odometry) {
    DrivetrainDescription d{.cartridge = MotorCartridge::Blue,
                            .imuPort = kImuPort,
                            .trackWidth = Length{kTrackWidth},
                            .source = DescriptionSource::RobotMeasured,
                            .provenance = "G1 host fixture: never measured on a robot"};
    for (std::size_t i = 0; i < membersPerSide; ++i) {
        d.left.signedPorts[i] = kLeft[i];
        d.right.signedPorts[i] = kRight[i];
    }
    d.left.count = static_cast<std::uint8_t>(membersPerSide);
    d.right.count = static_cast<std::uint8_t>(membersPerSide);
    d.left.inchesPerRadian = Length{kPlantInchesPerRadian};
    d.right.inchesPerRadian = Length{kPlantInchesPerRadian};

    cfgns::RobotConfig c{.identity = {.name = "Robot Two", .team = "1234A", .season = "Push Back"},
                         .drivetrain = {.kind = cfgns::DrivetrainKind::Tank,
                                        .tank = {.description = d}},
                         .odometry = {.kind = odometry}};
    if (odometry == cfgns::OdometryKind::TrackingWheels) {
        c.odometry.wheels = {.forward = {.port = kForwardPodPort,
                                         .wheelDiameter = Length{2.0},
                                         .offset = Length{-3.0}},
                             .lateral = {.port = kLateralPodPort,
                                         .wheelDiameter = Length{2.0},
                                         .offset = Length{-4.5}}};
    }
    c.tuning.motion = motionConfig();  // the gains that MATCH the plant (motion_test_rig.hpp)
    return c;
}

/// An X-drive config with tracking wheels, with or without a GPS.
cfgns::RobotConfig xdriveConfig(bool withGps) {
    cfgns::RobotConfig c{
        .identity = {.name = "Robot One", .team = "1234A", .season = "Push Back"},
        .drivetrain = {.kind = cfgns::DrivetrainKind::XDrive,
                       .xdrive = {.ports = {kXWheels[0], kXWheels[1], kXWheels[2], kXWheels[3]},
                                  .cartridge = MotorCartridge::Green,
                                  .imuPort = kImuPort,
                                  .driveRadius = Length{kDriveRadius}}},
        .odometry = {.kind = cfgns::OdometryKind::TrackingWheels,
                     .wheels = {.forward = {.port = kForwardPodPort,
                                            .wheelDiameter = Length{2.0},
                                            .offset = Length{-3.0}},
                                .lateral = {.port = kLateralPodPort,
                                            .wheelDiameter = Length{2.0},
                                            .offset = Length{-4.5}}}}};
    if (withGps) {
        c.sensors.gps = {.present = true,
                         .port = kGpsPort,
                         .leverArmForward = Length{0.0},
                         .leverArmLeft = Length{0.0}};
    }
    c.tuning.motion = motionConfig();
    return c;
}

/// Everything a run produces that must match bit for bit.
struct Trace {
    std::vector<TruthSample> truth;  ///< ground truth at each tick, memcmp-compared
    std::vector<double> volts;       ///< every plant wheel's commanded volts, per tick
    ExitReason exit = ExitReason::Running;
    int ticks = 0;
};

/// The pacer every rig here uses: motion_test_rig.hpp's PlantPacer with a recorder in front of
/// the step. It samples AFTER the tick and BEFORE the plant advances, so a sample is exactly the
/// state the tick was rendered on — the same instant motion_test_rig's hand loops record.
/// Hard-capped for the same reason PlantPacer is: a wait that never ends must turn a test RED
/// rather than hang the suite.
struct RecordingPacer final : shulib::motion::ITickPacer {
    explicit RecordingPacer(SimHarness& harness, Time dt = Time{0.01}, int cap = 200000)
        : h{&harness}, tickDt{dt}, capPaces{cap} {}

    void pace() override {
        if (trace.ticks >= capPaces) {
            throw std::runtime_error("RecordingPacer: pace cap exceeded — a wait failed to end");
        }
        ++trace.ticks;
        trace.truth.push_back(h->sample());
        for (int i = 0; i < h->motorCount(); ++i) {
            trace.volts.push_back(h->motor(i).commandedVoltage().value());
        }
        h->plant().step(tickDt);
    }

    SimHarness* h;
    Time tickDt;
    int capPaces;
    Trace trace;
};

bool sameTruth(const std::vector<TruthSample>& a, const std::vector<TruthSample>& b) {
    if (a.size() != b.size()) {
        return false;
    }
    return a.empty() || std::memcmp(a.data(), b.data(), a.size() * sizeof(TruthSample)) == 0;
}

/// Bit identity, spelled out: never Approx.
void requireIdentical(const Trace& longhand, const Trace& built) {
    REQUIRE(built.exit == longhand.exit);
    REQUIRE(built.ticks == longhand.ticks);
    REQUIRE(sameTruth(longhand.truth, built.truth));
    REQUIRE(longhand.volts.size() == built.volts.size());
    for (std::size_t i = 0; i < longhand.volts.size(); ++i) {
        REQUIRE(longhand.volts[i] == built.volts[i]);  // bits, not Approx
    }
}

// ═══ the host world: the plant, its fakes, the pacer, and a provider bound to them ═══
//
// This is the host's stand-in for "a robot exists and is plugged in" — the thing
// hal::pros::ProsDeviceProvider will be on the brain (G1 Part 2). It is NOT part of the
// builder: it owns the devices, the builder borrows them, exactly as ruling B2 says.
struct PlantWorld {
    SimHarness h;
    FakeTelemetrySink sink;
    RecordingPacer pacer;
    FakeDeviceProvider devices;

    PlantWorld(const shulib::kinematics::IKinematics& kin, const SimHarnessConfig& cfg,
               shulib::sim::DegradationModel* degradation = nullptr)
        : h{kin, cfg, nullptr, degradation},
          sink{},
          pacer{h},
          devices{h.clock(), h.battery(), sink, pacer} {}

    /// Bind every port `c` names to the matching plant device, and hang each tank side's EXTRA
    /// members on the plant's coupled train for that wheel. The plant reads the primary motor's
    /// commanded voltage when the members agree and their mean when they do not, so the primary
    /// MUST be inside the group the builder makes — which is what binding the first port of each
    /// side to h.motor(wheel) accomplishes. (fake_device_provider.hpp carries the measurement.)
    void wire(const cfgns::RobotConfig& c) {
        if (c.drivetrain.kind == cfgns::DrivetrainKind::Tank) {
            const DrivetrainDescription& d = c.drivetrain.tank.description;
            const shulib::hal::DriveSideDescription* sides[2] = {&d.left, &d.right};
            for (int wheel = 0; wheel < 2; ++wheel) {
                const shulib::hal::DriveSideDescription& side = *sides[wheel];
                devices.bindMotor(
                    static_cast<std::uint8_t>(
                        DrivetrainDescription::portMagnitude(side.signedPorts[0])),
                    h.motor(wheel));
                std::array<FakeMotor*, shulib::hal::kMaxSmartPorts> extras{};
                std::size_t n = 0;
                for (std::size_t i = 1; i < side.count; ++i) {
                    extras[n++] = &devices.fakeMotor(static_cast<std::uint8_t>(
                        DrivetrainDescription::portMagnitude(side.signedPorts[i])));
                }
                if (n > 0) {
                    h.plant().attachCoupledMembers(
                        wheel, std::span<FakeMotor* const>{extras.data(), n});
                }
            }
        } else {
            const std::int8_t* ports = c.drivetrain.kind == cfgns::DrivetrainKind::XDrive
                                           ? c.drivetrain.xdrive.ports
                                           : c.drivetrain.hdrive.ports;
            const int wheels = c.drivetrain.wheelCount();
            for (int i = 0; i < wheels; ++i) {
                devices.bindMotor(
                    static_cast<std::uint8_t>(DrivetrainDescription::portMagnitude(ports[i])),
                    h.motor(i));
            }
        }
        devices.bindImu(c.drivetrain.imuPort(), h.imu());
        if (c.sensors.gps.present) {
            devices.bindGps(c.sensors.gps.port, h.gps());
        }
        if (c.odometry.kind == cfgns::OdometryKind::TrackingWheels) {
            devices.bindRotation(c.odometry.wheels.forward.port, h.forwardEncoder());
            devices.bindRotation(c.odometry.wheels.lateral.port, h.lateralEncoder());
        }
    }
};

/// A PlantWorld already wired for `c`, plus the Robot built from it. Two lines of that is the
/// DoD sentence; the rest is the host stand-in for the brain.
struct BuiltRobot {
    PlantWorld world;
    shulib::robot::Robot robot;

    BuiltRobot(const cfgns::RobotConfig& c, const shulib::kinematics::IKinematics& kin,
               const SimHarnessConfig& cfg, shulib::sim::DegradationModel* degradation = nullptr)
        : world{kin, cfg, degradation}, robot{wired(world, c), world.devices} {}

private:
    /// Wire the world BEFORE the Robot's constructor runs (the member initializer needs an
    /// expression, and the binding must happen first or the builder gets unbound fakes).
    static const cfgns::RobotConfig& wired(PlantWorld& w, const cfgns::RobotConfig& c) {
        w.wire(c);
        return c;
    }
};

/// Drive the comparison SEQUENCE through a facade and hand back the trace its pacer recorded:
/// one moveTo, then one turnTo.
///
/// THE TURN IS NOT DECORATION. It was added after a measurement: with a single along-axis move,
/// a tank's two sides are commanded the SAME voltage every tick, so mutation M7 — swapping the
/// two MotorGroups in the kinematic wheel order, which is a genuinely wrong robot — was nearly
/// invisible (it turned one of four cases red, and only through an exit-reason difference). A
/// rotation commands the two sides OPPOSITE voltages, which is what makes a left/right swap show
/// up on the first tick. A bit-identity test is only as strong as the run it compares.
Trace runLeg(shulib::chassis::Chassis& chassis, RecordingPacer& pacer, const Pose2d& start,
             const Pose2d& target, double timeoutS = 8.0) {
    chassis.setPose(start);
    const ExitReason moved = chassis.moveTo(target, {.timeout = Time{timeoutS}});
    const ExitReason turned =
        chassis.turnTo(target.heading() + Angle::degrees(40.0), {.timeout = Time{timeoutS}});
    Trace t = pacer.trace;
    t.exit = moved == ExitReason::Settled ? turned : moved;
    return t;
}

// ═══ the LONGHAND rigs — transcriptions of src/main.cpp, written out in full ═════════

/// src/main.cpp's TankRobot as it was until G1 Part 2b, by hand, over the plant: five coupled
/// motors per side behind two MotorGroups, drive-encoder odometry over those groups, no GPS.
/// Member order IS initialization order, and it is the order robot/robot.hpp declares its own
/// members in.
struct LonghandTank {
    SimHarness h;
    shulib::hal::fake::FakeMotor extra[2][4]{};
    IMotor* leftPtrs[5]{};
    IMotor* rightPtrs[5]{};
    std::optional<MotorGroup> left;
    std::optional<MotorGroup> right;
    std::array<IMotor*, 2> driveMotors{};
    std::array<MotorGroup*, 2> groups{};
    shulib::hal::AbsentGps absentGps{};
    shulib::hal::AbsentTagSource tags{};
    shulib::hal::AbsentVision vision{};
    FakeTelemetrySink sink{};
    shulib::diag::FaultLatch faults;
    shulib::diag::HealthMonitor health;
    DriveGeometry leftGeometry;
    DriveGeometry rightGeometry;
    std::optional<shulib::localization::DriveEncoderOdometry> driveOdom;
    std::optional<shulib::localization::PilonsOdometry> pilonsOdom;
    shulib::localization::ComplementaryFusion fusion{};
    std::optional<shulib::localization::Localizer> loc;
    std::optional<shulib::chassis::RobotContext> ctx;
    shulib::motion::MotionDeps deps{};
    RecordingPacer pacer;
    std::optional<shulib::chassis::Chassis> chassis;

    LonghandTank(const shulib::kinematics::IKinematics& kin, std::size_t membersPerSide,
                 cfgns::OdometryKind odometry, const SimHarnessConfig& cfg,
                 shulib::sim::DegradationModel* degradation = nullptr)
        : h{kin, cfg, nullptr, degradation},
          faults{sink, h.clock()},
          health{faults},
          leftGeometry{DriveGeometry::fromInchesPerRadian(Length{kPlantInchesPerRadian})},
          rightGeometry{DriveGeometry::fromInchesPerRadian(Length{kPlantInchesPerRadian})},
          pacer{h} {
        for (int wheel = 0; wheel < 2; ++wheel) {
            IMotor** out = wheel == 0 ? leftPtrs : rightPtrs;
            out[0] = &h.motor(wheel);
            std::array<FakeMotor*, 4> ex{};
            for (std::size_t i = 1; i < membersPerSide; ++i) {
                out[i] = &extra[wheel][i - 1];
                ex[i - 1] = &extra[wheel][i - 1];
            }
            if (membersPerSide > 1) {
                h.plant().attachCoupledMembers(
                    wheel, std::span<FakeMotor* const>{ex.data(), membersPerSide - 1});
            }
        }
        left.emplace(std::span<IMotor* const>{leftPtrs, membersPerSide});
        right.emplace(std::span<IMotor* const>{rightPtrs, membersPerSide});
        driveMotors = {&*left, &*right};
        groups = {&*left, &*right};
        if (odometry == cfgns::OdometryKind::DriveEncoders) {
            driveOdom.emplace(h.imu(), *left, *right, leftGeometry, rightGeometry,
                              Length{kTrackWidth});
        } else {
            pilonsOdom.emplace(h.imu(), h.makeForwardTrackingWheel(),
                               h.makeLateralTrackingWheel());
        }
        shulib::localization::IOdometry& odom =
            driveOdom.has_value()
                ? static_cast<shulib::localization::IOdometry&>(*driveOdom)
                : static_cast<shulib::localization::IOdometry&>(*pilonsOdom);
        loc.emplace(h.clock(), h.imu(), odom, fusion);
        ctx.emplace(shulib::chassis::RobotContextConfig{
            .clock = &h.clock(),
            .driveMotors = std::span<IMotor* const>{driveMotors.data(), 2},
            .imu = &h.imu(),
            .gps = &absentGps,
            .battery = &h.battery(),
            .telemetry = &sink,
            .tags = &tags,
            .vision = &vision});
        deps = {.ctx = &*ctx,
                .localizer = &*loc,
                .kinematics = &kin,
                .faults = &faults,
                .health = &health,
                .motorGroups = std::span<MotorGroup* const>{groups.data(), 2}};
        // src/main.cpp's tankChassisConfig() (until G1 Part 2b), by hand: the stall check is told
        // the drivetrain's real geometry, and told that a drive-encoder odometry is not an
        // independent motion source. This is the one thing robot::Robot derives rather than asking a person for.
        shulib::chassis::ChassisConfig chassisCfg{.motion = motionConfig()};
        chassisCfg.motion.stall.setAllWheels(leftGeometry);
        chassisCfg.motion.stall.independentMotionSource =
            odometry != cfgns::OdometryKind::DriveEncoders;
        chassis.emplace(deps, pacer, chassisCfg);
    }
};

/// chassis_facade_test.cpp's standalone case, by hand, over the plant — plus the GPS corrector
/// when `withGps`, which is the only difference the builder makes for a GPS-carrying robot.
struct LonghandXDrive {
    SimHarness h;
    std::array<IMotor*, 4> driveMotors{};
    shulib::hal::AbsentGps absentGps{};
    shulib::hal::AbsentTagSource tags{};
    shulib::hal::AbsentVision vision{};
    FakeTelemetrySink sink{};
    shulib::diag::FaultLatch faults;
    shulib::diag::HealthMonitor health;
    shulib::localization::PilonsOdometry odom;
    shulib::localization::ComplementaryFusion fusion{};
    std::optional<shulib::localization::GpsCorrector> gpsCorrector;
    std::array<shulib::localization::ICorrector*, 1> correctors{};
    std::optional<shulib::localization::Localizer> loc;
    std::optional<shulib::chassis::RobotContext> ctx;
    shulib::motion::MotionDeps deps{};
    RecordingPacer pacer;
    std::optional<shulib::chassis::Chassis> chassis;

    LonghandXDrive(const shulib::kinematics::IKinematics& kin, bool withGps,
                   const SimHarnessConfig& cfg,
                   shulib::sim::DegradationModel* degradation = nullptr)
        : h{kin, cfg, nullptr, degradation},
          faults{sink, h.clock()},
          health{faults},
          odom{h.imu(), h.makeForwardTrackingWheel(), h.makeLateralTrackingWheel()},
          pacer{h} {
        for (int i = 0; i < 4; ++i) {
            driveMotors[static_cast<std::size_t>(i)] = &h.motor(i);
        }
        std::size_t correctorCount = 0;
        if (withGps) {
            gpsCorrector.emplace(h.clock(), h.gps(), h.imu());
            correctors[0] = &*gpsCorrector;
            correctorCount = 1;
        }
        loc.emplace(h.clock(), h.imu(), odom, fusion,
                    std::span<shulib::localization::ICorrector* const>{correctors.data(),
                                                                       correctorCount});
        ctx.emplace(shulib::chassis::RobotContextConfig{
            .clock = &h.clock(),
            .driveMotors = std::span<IMotor* const>{driveMotors.data(), 4},
            .imu = &h.imu(),
            .gps = withGps ? static_cast<shulib::hal::IGps*>(&h.gps())
                           : static_cast<shulib::hal::IGps*>(&absentGps),
            .battery = &h.battery(),
            .telemetry = &sink,
            .tags = &tags,
            .vision = &vision});
        deps = {.ctx = &*ctx,
                .localizer = &*loc,
                .kinematics = &kin,
                .faults = &faults,
                .health = &health};
        chassis.emplace(deps, pacer, shulib::chassis::ChassisConfig{.motion = motionConfig()});
    }
};

// ── the legs ────────────────────────────────────────────────────────────────────────
// Two different ones, because the two drivetrains can honestly do different things and
// pretending otherwise would be designing the test around the library rather than around the
// robot. A TANK has no lateral authority, so a laterally-offset target exits TimedOut on purpose
// (C1's drivetrain honesty, chassis.hpp); its leg is therefore along its own heading, exactly as
// motor_group_plant_test's alongAxisTrial draws one. An X-DRIVE translates and rotates at once,
// so its leg does both.
const Pose2d kStart{Length{-12.0}, Length{6.0}, Angle::degrees(30.0)};
const Pose2d kHolonomicTarget{Length{18.0}, Length{-9.0}, Angle::degrees(-20.0)};
/// 30 in along the start heading of 30°, heading held: cos 30° and sin 30° spelled out so the
/// fixture is a constant a reader can check rather than a call.
const Pose2d kTankTarget{Length{-12.0 + 30.0 * 0.86602540378443865},
                         Length{6.0 + 30.0 * 0.5}, Angle::degrees(30.0)};

SimHarnessConfig worldConfig(std::uint64_t seed = 1) {
    SimHarnessConfig cfg = plantConfig();
    cfg.plant.initialPose = kStart;
    cfg.plant.seed = seed;
    cfg.plant.driveWheelDiameter = Length{kPlantWheelDiameter};  // see kPlantWheelDiameter
    return cfg;
}

}  // namespace

// ═══ 4. validate() refuses BEFORE anything is constructed, and names the thing ═══════
// Bug caught: a builder that discovers a dead port halfway through wiring — leaving a
// half-constructed graph, an exception out of initialize(), and a person on a competition field
// with a stack trace instead of "port 13 has no motor".
TEST_CASE("G1 builder 4: validate() names the refused port, the bad field, and passes a good pair") {
    const shulib::kinematics::TankKinematics kin{Length{kTrackWidth}};

    SUBCASE("a device the provider refuses — the port is in the verdict") {
        const cfgns::RobotConfig c = tankConfig(5, cfgns::OdometryKind::DriveEncoders);
        PlantWorld world{kin, worldConfig()};
        world.wire(c);
        world.devices.refusePort(13);  // the third RIGHT motor, stored as -13
        const shulib::robot::BuildVerdict v = shulib::robot::validate(c, world.devices);
        CHECK_FALSE(v.ok);
        const std::string why{v.why};
        CHECK(why.find("port 13") != std::string::npos);
        CHECK(why.find("RIGHT drive motor") != std::string::npos);
    }
    SUBCASE("a refused IMU, a refused GPS and a refused tracking wheel each name themselves") {
        const shulib::kinematics::MatrixKinematics xkin =
            shulib::kinematics::xDrive(Length{kDriveRadius});
        const cfgns::RobotConfig c = xdriveConfig(/*withGps=*/true);
        struct Case {
            std::uint8_t port;
            const char* phrase;
        };
        const Case cases[] = {{kImuPort, "IMU"},
                              {kGpsPort, "GPS"},
                              {kForwardPodPort, "forward tracking wheel"},
                              {kLateralPodPort, "lateral tracking wheel"}};
        for (const Case& k : cases) {
            CAPTURE(k.phrase);
            PlantWorld world{xkin, worldConfig()};
            world.wire(c);
            world.devices.refusePort(k.port);
            const shulib::robot::BuildVerdict v = shulib::robot::validate(c, world.devices);
            CHECK_FALSE(v.ok);
            const std::string why{v.why};
            CHECK(why.find(k.phrase) != std::string::npos);
            CHECK(why.find(std::to_string(static_cast<int>(k.port))) != std::string::npos);
        }
    }
    SUBCASE("wheel count != motor count — the F5 cross-check, naming the field") {
        // A wheel whose port was left at 0. The config itself is VALID (0 is "no port", the
        // tree's meaning for it everywhere); this is the one check that owns the completeness
        // rule, and it is the same check motion::MotionDeps::validate() makes at the bottom of
        // the stack — where it could only have said "3 != 4".
        cfgns::RobotConfig c = xdriveConfig(/*withGps=*/false);
        c.drivetrain.xdrive.ports[2] = 0;
        REQUIRE(c.valid());
        const shulib::kinematics::MatrixKinematics xkin =
            shulib::kinematics::xDrive(Length{kDriveRadius});
        PlantWorld world{xkin, worldConfig()};
        const shulib::robot::BuildVerdict v = shulib::robot::validate(c, world.devices);
        CHECK_FALSE(v.ok);
        const std::string why{v.why};
        CHECK(why.find("4 kinematic wheels") != std::string::npos);
        CHECK(why.find("3 motor ports") != std::string::npos);
        CHECK(why.find("drivetrain.ports") != std::string::npos);
    }
    SUBCASE("an invalid CONFIG is refused before any device is asked for") {
        cfgns::RobotConfig c = tankConfig(5, cfgns::OdometryKind::DriveEncoders);
        c.drivetrain.tank.description.trackWidth = Length{0.0};
        PlantWorld world{kin, worldConfig()};
        const shulib::robot::BuildVerdict v = shulib::robot::validate(c, world.devices);
        CHECK_FALSE(v.ok);
        CHECK(std::string{v.why}.find("track width") != std::string::npos);
        // Not one device was opened: the config never got past its own rules.
        for (std::uint8_t p = 1; p <= 21; ++p) {
            CHECK_FALSE(world.devices.motorRequested(p));
        }
    }
    SUBCASE("a good pair passes, and every port the config names was asked for — and no other") {
        const cfgns::RobotConfig c = tankConfig(5, cfgns::OdometryKind::DriveEncoders);
        PlantWorld world{kin, worldConfig()};
        world.wire(c);
        const shulib::robot::BuildVerdict v = shulib::robot::validate(c, world.devices);
        CHECK(v.ok);
        CHECK(std::strlen(v.why) == 0);
        bool expected[22] = {};
        for (std::size_t i = 0; i < 5; ++i) {
            expected[DrivetrainDescription::portMagnitude(kLeft[i])] = true;
            expected[DrivetrainDescription::portMagnitude(kRight[i])] = true;
        }
        for (std::uint8_t p = 1; p <= 21; ++p) {
            CAPTURE(static_cast<int>(p));
            CHECK(world.devices.motorRequested(p) == expected[p]);
        }
        // The human-stated, unverifiable cartridge reached the adapter rather than being dropped.
        CHECK(world.devices.motorCartridge(1) == MotorCartridge::Blue);
        CHECK(world.devices.motorCartridge(15) == MotorCartridge::Blue);
        // And every port was handed over AS STORED — sign included. A fake cannot show a sign
        // error by driving differently (PROS applies the reversal inside the real adapter), so
        // the one-place-for-a-sign rule is checked structurally or not at all.
        //
        // TWICE, and that is not redundancy: validate() and the Robot's own wiring pass ask the
        // provider separately, and the provider records the MOST RECENT request per port. A
        // check made only after validate() proves nothing about the wiring pass — measured, as
        // mutation M14, which dropped the sign in wireMotors() alone and stayed green until this
        // second half existed.
        const auto signsAreAsStored = [&](const char* when) {
            CAPTURE(when);
            for (std::size_t i = 0; i < 5; ++i) {
                CAPTURE(i);
                CHECK(world.devices.motorSignedPort(static_cast<std::uint8_t>(
                          DrivetrainDescription::portMagnitude(kLeft[i]))) == kLeft[i]);
                CHECK(world.devices.motorSignedPort(static_cast<std::uint8_t>(
                          DrivetrainDescription::portMagnitude(kRight[i]))) == kRight[i]);
            }
        };
        signsAreAsStored("after validate()");
        shulib::robot::Robot robot{c, world.devices};
        signsAreAsStored("after the Robot wired itself");
    }
}

// ═══ 5. TANK on the plant, bit-identical to the longhand graph ═══════════════════════
// Ruling B4. Five coupled members per side, through a clean and a fully hostile world.
TEST_CASE("G1 builder 5: a tank Robot drives the plant BIT-IDENTICALLY to the hand-wired graph "
          "— drive encoders, five members per side, clean world") {
    const shulib::kinematics::TankKinematics kin{Length{kTrackWidth}};
    const cfgns::RobotConfig c = tankConfig(5, cfgns::OdometryKind::DriveEncoders);

    LonghandTank longhand{kin, 5, cfgns::OdometryKind::DriveEncoders, worldConfig()};
    const Trace a = runLeg(*longhand.chassis, longhand.pacer, kStart, kTankTarget);

    BuiltRobot built{c, kin, worldConfig()};
    const Trace b = runLeg(built.robot.chassis(), built.world.pacer, kStart, kTankTarget);

    REQUIRE(a.exit == ExitReason::Settled);
    REQUIRE(a.ticks > 50);  // a real run, not an instant exit
    requireIdentical(a, b);
    CHECK(built.robot.motorGroupCount() == 2);
}

TEST_CASE("G1 builder 5b: the same tank, bit-identical through the FULL hostile world") {
    const shulib::kinematics::TankKinematics kin{Length{kTrackWidth}};
    const cfgns::RobotConfig c = tankConfig(5, cfgns::OdometryKind::DriveEncoders);
    for (std::uint64_t seed = 1; seed <= 3; ++seed) {
        CAPTURE(seed);
        shulib::sim::FullHostility worldA{};
        shulib::sim::FullHostility worldB{};
        LonghandTank longhand{kin,          5, cfgns::OdometryKind::DriveEncoders,
                              worldConfig(seed), &worldA.model()};
        const Trace a = runLeg(*longhand.chassis, longhand.pacer, kStart, kTankTarget);
        BuiltRobot built{c, kin, worldConfig(seed), &worldB.model()};
        const Trace b = runLeg(built.robot.chassis(), built.world.pacer, kStart, kTankTarget);
        REQUIRE(a.ticks > 50);
        requireIdentical(a, b);
    }
}

TEST_CASE("G1 builder 5c: a tank with TRACKING WHEELS — the motor_group_plant_test GroupRig "
          "composition — is bit-identical too") {
    // The brief names GroupRig for test 5, and GroupRig runs PilonsOdometry over the harness's
    // tracking wheels rather than the drive encoders. Both readings are covered: 5/5b are the
    // drive-encoder tank (what src/main.cpp's TankRobot was, and robot two's robot::Robot is), and
    // this is GroupRig's exact composition, reached from a config by choosing a different
    // odometry kind.
    const shulib::kinematics::TankKinematics kin{Length{kTrackWidth}};
    const cfgns::RobotConfig c = tankConfig(5, cfgns::OdometryKind::TrackingWheels);
    LonghandTank longhand{kin, 5, cfgns::OdometryKind::TrackingWheels, worldConfig()};
    const Trace a = runLeg(*longhand.chassis, longhand.pacer, kStart, kTankTarget);
    BuiltRobot built{c, kin, worldConfig()};
    const Trace b = runLeg(built.robot.chassis(), built.world.pacer, kStart, kTankTarget);
    REQUIRE(a.exit == ExitReason::Settled);
    requireIdentical(a, b);
}

TEST_CASE("G1 builder 5d: one member per side — the degenerate group — is bit-identical too") {
    const shulib::kinematics::TankKinematics kin{Length{kTrackWidth}};
    const cfgns::RobotConfig c = tankConfig(1, cfgns::OdometryKind::DriveEncoders);
    LonghandTank longhand{kin, 1, cfgns::OdometryKind::DriveEncoders, worldConfig()};
    const Trace a = runLeg(*longhand.chassis, longhand.pacer, kStart, kTankTarget);
    BuiltRobot built{c, kin, worldConfig()};
    const Trace b = runLeg(built.robot.chassis(), built.world.pacer, kStart, kTankTarget);
    REQUIRE(a.exit == ExitReason::Settled);
    requireIdentical(a, b);
}

// ═══ 5f. The health wiring is actually CONNECTED, not merely present ════════════════
// Bug caught: MotionDeps::motorGroups left empty by the builder. Nothing about a healthy run
// changes if it is — measured, as mutation M13, which emptied that span and left every
// bit-identity case GREEN — because the groups' disagreement observable is only EVALUATED
// through that span, and a healthy group never disagrees. So the only way to test the wiring is
// to break a motor and require the fault. This is motor_group_plant_test's case 3, reached from
// a config instead of from a hand-wired rig.
TEST_CASE("G1 builder 5f: a FROZEN member of a builder-made group is counted and raises "
          "MOTOR_GROUP_DISAGREE — the deps' motorGroups span is wired") {
    const shulib::kinematics::TankKinematics kin{Length{kTrackWidth}};
    BuiltRobot built{tankConfig(5, cfgns::OdometryKind::DriveEncoders), kin, worldConfig()};
    // Member 2 of the LEFT wheel: one of the four the provider owns and the test attached to the
    // plant's coupled train. A dead port does not take a rotor out of a gear box, so the plant
    // keeps dragging it — it reports 0 rad while its mates turn, the frozen-encoder signature.
    built.world.h.plant().setMemberFault(0, 2, shulib::sim::MemberFault::Frozen);
    const Trace t = runLeg(built.robot.chassis(), built.world.pacer, kStart, kTankTarget);

    CHECK(t.ticks > 50);
    CHECK(built.robot.faults().raiseCount(shulib::diag::FaultCode::MotorGroupDisagree) >= 1);
    CHECK(built.robot.faults().firstFault() == shulib::diag::FaultCode::MotorGroupDisagree);
    CHECK(built.robot.motorGroupCount() == 2);

    // A healthy robot raises nothing — so the case above is detecting the fault, not just
    // observing a counter that is always non-zero.
    BuiltRobot healthy{tankConfig(5, cfgns::OdometryKind::DriveEncoders), kin, worldConfig()};
    (void)runLeg(healthy.robot.chassis(), healthy.world.pacer, kStart, kTankTarget);
    CHECK(healthy.robot.faults().raiseCount(shulib::diag::FaultCode::MotorGroupDisagree) == 0);
}

// ═══ 5e. The ONE thing the builder DERIVES rather than asking a person for ══════════
// src/main.cpp's tankChassisConfig() told the stall check (until G1 Part 2b) the drivetrain's real
// per-wheel geometry and that a drive-encoder odometry is not an independent motion source. The
// builder has both facts already, so it fills them in — and this case asserts that on the FACADE,
// not on the plant, because a healthy run never trips the stall check and therefore cannot show
// the difference. (That is not a guess: mutations M8 and M9 were run against the bit-identity
// cases alone and stayed GREEN. This case is what turns them red.)
TEST_CASE("G1 builder 5e: the builder derives the stall check's geometry and motion-source flag "
          "from the drivetrain, and leaves a holonomic robot's tuning alone") {
    const shulib::kinematics::TankKinematics kin{Length{kTrackWidth}};

    SUBCASE("a tank on drive encoders") {
        BuiltRobot built{tankConfig(5, cfgns::OdometryKind::DriveEncoders), kin, worldConfig()};
        const auto& stall = built.robot.chassis().motionConfig().stall;
        for (std::size_t w = 0; w < 2; ++w) {
            CAPTURE(w);
            CHECK(stall.wheels[w].inchesPerRadian().value() == kPlantInchesPerRadian);
        }
        // …which is NOT the library's stand-in default, so this check is doing work.
        CHECK(shulib::hal::DriveGeometry{}.inchesPerRadian().value() != kPlantInchesPerRadian);
        // The odometry IS the drive wheels, so the cross-check has nothing independent to compare
        // against and must never report a verdict (odo_stall_check.hpp §2).
        CHECK_FALSE(stall.independentMotionSource);
    }
    SUBCASE("a tank on tracking wheels — now there IS an independent motion source") {
        BuiltRobot built{tankConfig(5, cfgns::OdometryKind::TrackingWheels), kin, worldConfig()};
        const auto& stall = built.robot.chassis().motionConfig().stall;
        CHECK(stall.wheels[0].inchesPerRadian().value() == kPlantInchesPerRadian);
        CHECK(stall.independentMotionSource);
    }
    SUBCASE("a holonomic robot's tuning is passed through UNTOUCHED") {
        // An X-drive config carries no per-wheel scale (its odometry is the pods), so there is
        // nothing to derive and the builder invents nothing — which is also exactly what
        // src/main.cpp's hand-wired X-drive graph did until G1 Part 2c: `Chassis chassis{deps,
        // pacer}`, default config.
        const shulib::kinematics::MatrixKinematics xkin =
            shulib::kinematics::xDrive(Length{kDriveRadius});
        BuiltRobot built{xdriveConfig(false), xkin, worldConfig()};
        const auto& stall = built.robot.chassis().motionConfig().stall;
        CHECK(stall.wheels[0].inchesPerRadian().value()
              == shulib::hal::DriveGeometry{}.inchesPerRadian().value());
        CHECK(stall.independentMotionSource);
    }
    SUBCASE("the caller's OWN tuning still comes through — the builder adds, it does not replace") {
        cfgns::RobotConfig c = tankConfig(5, cfgns::OdometryKind::DriveEncoders);
        c.tuning.motion.maxLinearSpeed = shulib::units::Velocity{17.0};
        BuiltRobot built{c, kin, worldConfig()};
        CHECK(built.robot.chassis().motionConfig().maxLinearSpeed.value() == 17.0);
    }
}

// ═══ 6. X-DRIVE on the plant, bit-identical ═════════════════════════════════════════
TEST_CASE("G1 builder 6: an X-drive Robot with tracking wheels and NO GPS is bit-identical to "
          "chassis_facade_test's standalone case, written out longhand") {
    const shulib::kinematics::MatrixKinematics kin = shulib::kinematics::xDrive(Length{kDriveRadius});
    const cfgns::RobotConfig c = xdriveConfig(/*withGps=*/false);
    LonghandXDrive longhand{kin, /*withGps=*/false, worldConfig()};
    const Trace a = runLeg(*longhand.chassis, longhand.pacer, kStart, kHolonomicTarget);
    BuiltRobot built{c, kin, worldConfig()};
    const Trace b = runLeg(built.robot.chassis(), built.world.pacer, kStart, kHolonomicTarget);
    REQUIRE(a.exit == ExitReason::Settled);
    REQUIRE(a.ticks > 50);
    requireIdentical(a, b);
    CHECK(built.robot.motorGroupCount() == 0);  // holonomic wheels are single motors
}

TEST_CASE("G1 builder 6b: an X-drive Robot WITH a GPS is bit-identical to the same graph plus a "
          "hand-wired GpsCorrector — and DIFFERENT from the one without") {
    const shulib::kinematics::MatrixKinematics kin = shulib::kinematics::xDrive(Length{kDriveRadius});
    const cfgns::RobotConfig c = xdriveConfig(/*withGps=*/true);
    LonghandXDrive longhand{kin, /*withGps=*/true, worldConfig()};
    const Trace a = runLeg(*longhand.chassis, longhand.pacer, kStart, kHolonomicTarget);
    BuiltRobot built{c, kin, worldConfig()};
    const Trace b = runLeg(built.robot.chassis(), built.world.pacer, kStart, kHolonomicTarget);
    REQUIRE(a.exit == ExitReason::Settled);
    requireIdentical(a, b);

    // And the corrector is doing something: the SAME config without a GPS produces a different
    // run. Without this line the case above would also pass if the builder installed no
    // corrector at all and the longhand rig's corrector never proposed.
    LonghandXDrive noGps{kin, /*withGps=*/false, worldConfig()};
    const Trace c2 = runLeg(*noGps.chassis, noGps.pacer, kStart, kHolonomicTarget);
    CHECK_FALSE(sameTruth(a.truth, c2.truth));
}

// ═══ 7. Absent is EXPLICIT, and present is INSTALLED ════════════════════════════════
// Bug caught: a corrector installed unconditionally (so a GPS-less robot reports quality
// "Corrected" from a source that does not exist), and a nullptr where hal::AbsentGps belongs.
TEST_CASE("G1 builder 7: no GPS means AbsentGps and NO corrector; a GPS means the corrector is "
          "installed and visible in the records") {
    const shulib::kinematics::MatrixKinematics kin = shulib::kinematics::xDrive(Length{kDriveRadius});

    SUBCASE("absent") {
        BuiltRobot built{xdriveConfig(false), kin, worldConfig()};
        CHECK_FALSE(built.robot.hasGpsCorrector());
        // The context's GPS is the explicit Absent object, not a fake and not a null: it is
        // never a fix, and it says so honestly rather than by being missing.
        shulib::hal::IGps& gps = built.robot.context().gps();
        CHECK_FALSE(gps.hasFix());
        CHECK(dynamic_cast<shulib::hal::AbsentGps*>(&gps) != nullptr);
        CHECK(dynamic_cast<shulib::hal::fake::FakeGps*>(&gps) == nullptr);
        // Drive a leg: no corrector ever applies, so the estimate never reports a source.
        (void)runLeg(built.robot.chassis(), built.world.pacer, kStart, kHolonomicTarget);
        CHECK(std::string{built.robot.localizer().lastCorrection().source} == "none");
        CHECK(built.robot.localizer().qualityClass()
              != shulib::localization::Localizer::Quality::Corrected);
    }
    SUBCASE("present") {
        BuiltRobot built{xdriveConfig(true), kin, worldConfig()};
        CHECK(built.robot.hasGpsCorrector());
        shulib::hal::IGps& gps = built.robot.context().gps();
        CHECK(dynamic_cast<shulib::hal::AbsentGps*>(&gps) == nullptr);
        CHECK(dynamic_cast<shulib::hal::fake::FakeGps*>(&gps) != nullptr);
        // And the lever arm the config states reached the provider — the number that is
        // subtracted from every fix, once, at the HAL edge.
        CHECK(built.world.devices.gpsLeverForward(kGpsPort).value() == 0.0);
        (void)runLeg(built.robot.chassis(), built.world.pacer, kStart, kHolonomicTarget);
        CHECK(std::string{built.robot.localizer().lastCorrection().source} == "gps");
    }
    SUBCASE("a tank with no GPS gets AbsentGps too — the robot-two shape") {
        const shulib::kinematics::TankKinematics tank{Length{kTrackWidth}};
        BuiltRobot built{tankConfig(5, cfgns::OdometryKind::DriveEncoders), tank, worldConfig()};
        CHECK_FALSE(built.robot.hasGpsCorrector());
        CHECK(dynamic_cast<shulib::hal::AbsentGps*>(&built.robot.context().gps()) != nullptr);
        // Tags and vision are ABSENT unconditionally in Part 1 — no camera adapter exists.
        CHECK(dynamic_cast<shulib::hal::AbsentTagSource*>(&built.robot.context().tags())
              != nullptr);
        CHECK(dynamic_cast<shulib::hal::AbsentVision*>(&built.robot.context().vision())
              != nullptr);
    }
}

// ═══ 8. "TWO LINES" — the DoD sentence, as code ═════════════════════════════════════
TEST_CASE("G1 builder 8: a hand-built config produces a working Chassis in two lines") {
    // The host's stand-in for "a robot exists and is plugged in" — the A2 plant, its fakes, the
    // pacer, and the ports bound to them. On the brain this line is `ProsDeviceProvider devices;`
    // and nothing else (G1 Part 2); it is not part of the two lines, any more than the robot's
    // wiring harness is part of the program.
    const shulib::kinematics::TankKinematics kin{Length{kTrackWidth}};
    const cfgns::RobotConfig config = tankConfig(5, cfgns::OdometryKind::DriveEncoders);
    PlantWorld world{kin, worldConfig()};
    world.wire(config);

    // ── THE TWO LINES ──────────────────────────────────────────────────────────────────
    FakeDeviceProvider& devices = world.devices;
    shulib::robot::Robot robot{config, devices};
    // ───────────────────────────────────────────────────────────────────────────────────

    // …and it drives. Graded on the plant's GROUND TRUTH, never on the estimate the motion
    // itself read.
    robot.chassis().setPose(kStart);
    CHECK(robot.chassis().moveTo(kTankTarget, {.timeout = Time{8.0}}) == ExitReason::Settled);
    CHECK(motion_rig::posErr(world.h.truePose(), kTankTarget) < 1.0);
    CHECK(motion_rig::headErr(world.h.truePose(), kTankTarget) < 0.03);
    CHECK(robot.chassis().turnTo(Angle::degrees(70.0), {.timeout = Time{8.0}})
          == ExitReason::Settled);
    CHECK(std::abs(world.h.truePose().heading().errorTo(Angle::degrees(70.0))) < 0.03);
}

// ═══ 9. describe(): the banner is DATA, derived from the config it was built with ════
TEST_CASE("G1 builder 9: describe() names the identity, the source word, the ports as stored, "
          "the geometry and every absent device") {
    const shulib::kinematics::TankKinematics kin{Length{kTrackWidth}};
    BuiltRobot built{tankConfig(5, cfgns::OdometryKind::DriveEncoders), kin, worldConfig()};
    char buf[320] = "";
    built.robot.describe(buf, sizeof buf);
    const std::string s{buf};
    CAPTURE(s);
    CHECK(s.find("Robot Two") != std::string::npos);
    CHECK(s.find("1234A") != std::string::npos);
    CHECK(s.find("tank") != std::string::npos);
    CHECK(s.find("MEASURED") != std::string::npos);  // not TYPED — the source word is a claim
    // Both sides' ports AS STORED, signs included: the one place a sign lives.
    CHECK(s.find("L 1,-2,3,-4,5") != std::string::npos);
    CHECK(s.find("R -11,12,-13,14,-15") != std::string::npos);
    CHECK(s.find("W 12.000 in") != std::string::npos);
    CHECK(s.find("IMU10") != std::string::npos);
    CHECK(s.find("drive-encoders") != std::string::npos);
    CHECK(s.find("GPS ABSENT") != std::string::npos);
    CHECK(s.find("tags ABSENT") != std::string::npos);
    CHECK(s.find("vision ABSENT") != std::string::npos);

    const shulib::kinematics::MatrixKinematics xkin =
        shulib::kinematics::xDrive(Length{kDriveRadius});
    BuiltRobot x{xdriveConfig(true), xkin, worldConfig()};
    x.robot.describe(buf, sizeof buf);
    const std::string xs{buf};
    CAPTURE(xs);
    CHECK(xs.find("x-drive") != std::string::npos);
    CHECK(xs.find("wheels 1,2,3,4") != std::string::npos);
    CHECK(xs.find("tracking-wheels") != std::string::npos);
    CHECK(xs.find("GPS present") != std::string::npos);

    // Truncation, not overflow, and always a NUL — with the canary past the end intact.
    char small[40];
    std::memset(small, '\xAB', sizeof small);
    built.robot.describe(small, 24);
    CHECK(std::strlen(small) < 24);
    CHECK(small[24] == '\xAB');
    built.robot.describe(nullptr, 0);
}

// ═══ 10. Construction PRECONDITIONS on the verdict, carrying the same sentence ═══════
// Ruling B3. Bug caught: a constructor that builds anyway and fails later — or one that refuses
// with a different message from the one validate() would have given, so a person who ran the
// check by hand and a person who did not see two different stories about one robot.
TEST_CASE("G1 builder 10: a config that fails validate() throws from Robot's constructor, with "
          "the verdict's own sentence") {
    const shulib::kinematics::TankKinematics kin{Length{kTrackWidth}};

    SUBCASE("a refused port") {
        const cfgns::RobotConfig c = tankConfig(5, cfgns::OdometryKind::DriveEncoders);
        PlantWorld world{kin, worldConfig()};
        world.wire(c);
        world.devices.refusePort(13);
        const shulib::robot::BuildVerdict v = shulib::robot::validate(c, world.devices);
        REQUIRE_FALSE(v.ok);
        try {
            shulib::robot::Robot robot{c, world.devices};
            FAIL("Robot's constructor accepted a config validate() had refused");
        } catch (const shulib::PreconditionError& e) {
            CHECK(std::string{e.what()} == std::string{v.why});
            CHECK(std::string{e.what()}.find("port 13") != std::string::npos);
        }
    }
    SUBCASE("an invalid config field") {
        cfgns::RobotConfig c = tankConfig(5, cfgns::OdometryKind::DriveEncoders);
        c.drivetrain.tank.description.left.inchesPerRadian = Length{0.0};
        PlantWorld world{kin, worldConfig()};
        const shulib::robot::BuildVerdict v = shulib::robot::validate(c, world.devices);
        REQUIRE_FALSE(v.ok);
        try {
            shulib::robot::Robot robot{c, world.devices};
            FAIL("Robot's constructor accepted an invalid config");
        } catch (const shulib::PreconditionError& e) {
            CHECK(std::string{e.what()} == std::string{v.why});
            CHECK(std::string{e.what()}.find("inches per radian") != std::string::npos);
        }
    }
    SUBCASE("a wheel with no port") {
        cfgns::RobotConfig c = xdriveConfig(false);
        c.drivetrain.xdrive.ports[1] = 0;
        const shulib::kinematics::MatrixKinematics xkin =
            shulib::kinematics::xDrive(Length{kDriveRadius});
        PlantWorld world{xkin, worldConfig()};
        const shulib::robot::BuildVerdict v = shulib::robot::validate(c, world.devices);
        REQUIRE_FALSE(v.ok);
        CHECK_THROWS_AS((shulib::robot::Robot{c, world.devices}), shulib::PreconditionError);
    }
}

// ═══ 10b. The config a Robot was built from is COPIED, not borrowed ═════════════════
// Bug caught: a boot sequence that decodes a record into a local, builds a Robot from it, and
// lets the local die — the graph then reads a config that no longer exists, and the first thing
// to notice is a banner full of garbage.
TEST_CASE("G1 builder 10b: Robot holds its config by value") {
    const shulib::kinematics::TankKinematics kin{Length{kTrackWidth}};
    PlantWorld world{kin, worldConfig()};
    std::optional<shulib::robot::Robot> robot;
    {
        cfgns::RobotConfig scoped = tankConfig(5, cfgns::OdometryKind::DriveEncoders);
        world.wire(scoped);
        robot.emplace(scoped, world.devices);
        scoped = cfgns::RobotConfig{};  // the record's buffer is reused for the next thing
    }
    char buf[320] = "";
    robot->describe(buf, sizeof buf);
    CHECK(std::string{buf}.find("Robot Two") != std::string::npos);
    CHECK(robot->config().valid());
}

// ═════════════════════════════════════════════════════════════════════════════════════
// G1 PART 2a (2026-09-24) — ABSENT DRIVE PORTS (ruling M1), portMap(), driveOdometry()
// ═════════════════════════════════════════════════════════════════════════════════════
// Brief §4 tests 2, 3 and the portMap half of 5. Every case above this banner is Part 1's and
// is untouched: they build with DEFAULT options, and their bit-identity is the pin that the
// options changed nothing when nothing is absent (their assertion counts are recorded in the
// Part 2a log, before and after).

namespace {

/// A PlantWorld wired for `c`, then told that `absent` are DEAD — the provider REFUSES them, the
/// way the real provider refuses an empty port — and a Robot built from it with those ports in
/// its BuildOptions. The dead motors stay in the plant's coupled trains, uncommanded at 0 V: a
/// dead port does not take a motor out of a gear box, and the plant's mean-of-members model
/// (drive_plant.hpp, HA-137) then drives the wheel at the average of four commanded members and
/// one silent one — a degraded robot, not a merely renamed one.
struct DegradedRobot {
    PlantWorld world;
    shulib::robot::Robot robot;

    DegradedRobot(const cfgns::RobotConfig& c, std::span<const int> absent,
                  const shulib::kinematics::IKinematics& kin, const SimHarnessConfig& cfg)
        : world{kin, cfg}, robot{wiredAndRefused(world, c, absent), world.devices, {absent}} {}

private:
    static const cfgns::RobotConfig& wiredAndRefused(PlantWorld& w, const cfgns::RobotConfig& c,
                                                     std::span<const int> absent) {
        w.wire(c);
        for (const int p : absent) {
            w.devices.refusePort(static_cast<std::uint8_t>(p));
        }
        return c;
    }
};

/// The two side groups of a tank Robot, through the Tier-3 seam a panel uses.
const MotorGroup& group(shulib::robot::Robot& r, std::size_t side) {
    return *r.chassis().deps().motorGroups[side];
}

/// src/main.cpp's `tankPortMapString()` (lines 678-719 at 018424c), TRANSCRIBED — the body
/// verbatim, with the file-scope `g_description` made a parameter and the static buffer made the
/// caller's. It is the reference `writePortMap()` must equal byte for byte, so that Part 2b can
/// swap the generator without a single session header changing. Written out rather than shared
/// for Part 1's reason: a helper both sides used would make them agree by construction.
void transcribedTankPortMapString(const DrivetrainDescription& g_description,
                                  char (&buf)[shulib::diag::kMaxPortMapBytes]) {
    constexpr std::size_t kSideCap = 40;
    char l[kSideCap] = "";
    char r[kSideCap] = "";
    const shulib::hal::DriveSideDescription* sides[2] = {&g_description.left,
                                                         &g_description.right};
    char* out[2] = {l, r};
    for (int side = 0; side < 2; ++side) {
        std::size_t used = 0;
        for (std::size_t i = 0; i < sides[side]->count; ++i) {
            char one[8];
            std::snprintf(one, sizeof one, "%s%+d", i ? " " : "",
                          static_cast<int>(sides[side]->signedPorts[i]));
            for (const char* c = one; *c != '\0' && used + 1 < kSideCap; ++c) {
                out[side][used++] = *c;
            }
            out[side][used] = '\0';
        }
    }
    char full[200];
    std::snprintf(full, sizeof full, "%s L %s R %s IMU%u ipr %.4f/%.4f TW%.2f",
                  g_description.source == shulib::hal::DescriptionSource::RobotMeasured
                      ? "MEASURED"
                      : "TYPED",
                  l, r, static_cast<unsigned>(g_description.imuPort),
                  g_description.left.inchesPerRadian.value(),
                  g_description.right.inchesPerRadian.value(),
                  g_description.trackWidth.value());
    std::snprintf(buf, sizeof buf, "%.*s", static_cast<int>(sizeof buf) - 1, full);
}

/// A tank description with `leftCount` ports 1..leftCount on the left (alternating signs, +1
/// first) and `rightCount` on the right (ports 1..rightCount, -1 first) — shapes no valid Robot
/// carries once a side is wide (the ports collide), which is exactly why writePortMap() is also a
/// free function: it formats what is stored and validates nothing.
cfgns::RobotConfig wideTankConfig(std::uint8_t leftCount, std::uint8_t rightCount = 0) {
    cfgns::RobotConfig c = tankConfig(1, cfgns::OdometryKind::DriveEncoders);
    DrivetrainDescription& d = c.drivetrain.tank.description;
    for (std::uint8_t i = 0; i < leftCount; ++i) {
        const auto port = static_cast<std::int8_t>(i + 1);
        d.left.signedPorts[i] = (i % 2 == 0) ? port : static_cast<std::int8_t>(-port);
    }
    d.left.count = leftCount;
    if (rightCount > 0) {
        for (std::uint8_t i = 0; i < rightCount; ++i) {
            const auto port = static_cast<std::int8_t>(i + 1);
            d.right.signedPorts[i] = (i % 2 == 0) ? static_cast<std::int8_t>(-port) : port;
        }
        d.right.count = rightCount;
    }
    return c;
}

}  // namespace

// ═══ 2a-2. Absent ports: the builder obeys the boot check's list ═════════════════════
TEST_CASE("G1 2a builder A1: one absent port — validate() passes, the Robot CONSTRUCTS, the group "
          "reports 4 of 5 present, the port is NEVER requested, describe() names it") {
    // BUG CAUGHT (the brief's mutations "drop the options in prechecked()" and "request absent
    // ports anyway"): the audit's defect 1 — validate() tolerant, the constructor not, so a robot
    // the check passed threw inside its own constructor — and a builder that asked the provider
    // for a port the boot check had already reported dead (naming it twice, and on a real robot
    // refusing the build over a port the policy had accepted). The provider REFUSES port 3 here,
    // as the real one refuses an empty port, so either mutation turns this case red.
    const shulib::kinematics::TankKinematics kin{Length{kTrackWidth}};
    const cfgns::RobotConfig c = tankConfig(5, cfgns::OdometryKind::DriveEncoders);
    const int absent[] = {3};
    PlantWorld world{kin, worldConfig()};
    world.wire(c);
    world.devices.refusePort(3);

    // Without the list, the build is refused — naming the port, as before Part 2a.
    const shulib::robot::BuildVerdict refused = shulib::robot::validate(c, world.devices);
    CHECK_FALSE(refused.ok);
    CHECK(std::string{refused.why}.find("port 3") != std::string::npos);
    const int askedWithoutList = world.devices.motorRequestCount(3);
    CHECK(askedWithoutList == 1);  // the counter really does count a REFUSED request

    // With it: the verdict passes, and the constructor — which re-runs the verdict — agrees.
    const shulib::robot::BuildOptions options{.absentDrivePorts = absent};
    const shulib::robot::BuildVerdict v = shulib::robot::validate(c, world.devices, options);
    CAPTURE(v.why);
    REQUIRE(v.ok);
    std::optional<shulib::robot::Robot> robot;
    REQUIRE_NOTHROW(robot.emplace(c, world.devices, options));

    // Port 3 was never asked for by the tolerant verdict nor by the Robot's two passes.
    CHECK(world.devices.motorRequestCount(3) == askedWithoutList);
    // Every other drive port was.
    for (std::size_t i = 0; i < 5; ++i) {
        CAPTURE(i);
        if (DrivetrainDescription::portMagnitude(kLeft[i]) != 3) {
            CHECK(world.devices.motorRequestCount(static_cast<std::uint8_t>(
                      DrivetrainDescription::portMagnitude(kLeft[i])))
                  > 0);
        }
        CHECK(world.devices.motorRequestCount(static_cast<std::uint8_t>(
                  DrivetrainDescription::portMagnitude(kRight[i])))
              > 0);
    }
    // The LEFT group has a hole in slot 2 (port 3 is the third stored entry); the right is whole.
    const MotorGroup& left = group(*robot, 0);
    const MotorGroup& right = group(*robot, 1);
    CHECK(left.memberCount() == 5);
    CHECK(left.presentCount() == 4);
    CHECK(left.absentMask() == (std::uint32_t{1} << 2));
    CHECK_FALSE(left.isPresent(2));
    CHECK(right.presentCount() == 5);
    CHECK(right.absentMask() == 0);

    char buf[320] = "";
    robot->describe(buf, sizeof buf);
    CAPTURE(std::string{buf});
    CHECK(std::string{buf}.find("DEGRADED: absent 3 |") != std::string::npos);
    // …and the stored ports are still printed as stored: absence is not a rewrite of the wiring.
    CHECK(std::string{buf}.find("L 1,-2,3,-4,5") != std::string::npos);
}

TEST_CASE("G1 2a builder A2: DEFAULT options are Part 1's builder exactly — same graph, same run, "
          "same banner") {
    // BUG CAUGHT: an options path that changes a complete robot — a default that is not empty,
    // an absent check that drops a present member, a banner segment printed when nothing is
    // absent. Three ways of spelling "no options" are driven across the plant and compared bit
    // for bit; Part 1's own cases above (5, 5b, 5c, 5d, 6, 6b) are the other half of this pin.
    const shulib::kinematics::TankKinematics kin{Length{kTrackWidth}};
    const cfgns::RobotConfig c = tankConfig(5, cfgns::OdometryKind::DriveEncoders);

    BuiltRobot twoArg{c, kin, worldConfig()};
    const Trace a = runLeg(twoArg.robot.chassis(), twoArg.world.pacer, kStart, kTankTarget);

    PlantWorld w2{kin, worldConfig()};
    w2.wire(c);
    shulib::robot::Robot defaulted{c, w2.devices, shulib::robot::BuildOptions{}};
    const Trace b = runLeg(defaulted.chassis(), w2.pacer, kStart, kTankTarget);

    PlantWorld w3{kin, worldConfig()};
    w3.wire(c);
    const std::span<const int> empty{};
    shulib::robot::Robot explicitEmpty{c, w3.devices, {empty}};
    const Trace d = runLeg(explicitEmpty.chassis(), w3.pacer, kStart, kTankTarget);

    REQUIRE(a.exit == ExitReason::Settled);
    requireIdentical(a, b);
    requireIdentical(a, d);
    char ba[320] = "";
    char bb[320] = "";
    twoArg.robot.describe(ba, sizeof ba);
    defaulted.describe(bb, sizeof bb);
    CHECK(std::string{ba} == std::string{bb});
    CHECK(std::string{ba}.find("DEGRADED") == std::string::npos);
    CHECK(group(defaulted, 0).absentMask() == 0);
    CHECK(group(defaulted, 1).absentMask() == 0);
}

TEST_CASE("G1 2a builder A3: every new refusal names the port — in validate() AND, with the same "
          "sentence, from the constructor") {
    // BUG CAUGHT: a list the builder cannot honour being honoured anyway — a port that is not a
    // drive port silently ignored (a typo in the boot check's output would vanish), a signed
    // port accepted as its magnitude, a side emptied into a MotorGroup precondition three objects
    // deep, an X-drive given a hole its single-motor wheels cannot have.
    const shulib::kinematics::TankKinematics kin{Length{kTrackWidth}};
    const cfgns::RobotConfig tank = tankConfig(5, cfgns::OdometryKind::DriveEncoders);
    struct Case {
        std::string label;
        std::vector<int> absent;
        std::string phrase;
    };
    const Case cases[] = {
        {"a pod port, not a drive port", {7}, "absent port 7 is not one of this drivetrain's drive ports"},
        {"the IMU port", {10}, "absent port 10 is not one of"},
        {"port 0", {0}, "absent port 0 is not one of"},
        {"port 22", {22}, "absent port 22 is not one of"},
        {"a SIGNED port (-2) instead of its number", {-2}, "absent port -2 is not one of"},
        {"a good port then a bad one", {3, 8}, "absent port 8 is not one of"},
        {"the whole LEFT side", {1, 2, 3, 4, 5}, "every LEFT drive port is absent (ports 1 2 3 4 5)"},
        {"the whole RIGHT side, in another order", {15, 11, 13, 12, 14},
         "every RIGHT drive port is absent (ports 11 12 13 14 15)"},
    };
    for (const Case& k : cases) {
        CAPTURE(k.label);
        PlantWorld world{kin, worldConfig()};
        world.wire(tank);
        const shulib::robot::BuildOptions options{
            .absentDrivePorts = std::span<const int>{k.absent.data(), k.absent.size()}};
        const shulib::robot::BuildVerdict v = shulib::robot::validate(tank, world.devices, options);
        CHECK_FALSE(v.ok);
        CAPTURE(v.why);
        CHECK(std::string{v.why}.find(k.phrase) != std::string::npos);
        // Refused BEFORE any device was asked for.
        for (std::uint8_t p = 1; p <= 21; ++p) {
            CHECK(world.devices.motorRequestCount(p) == 0);
        }
        try {
            shulib::robot::Robot robot{tank, world.devices, options};
            FAIL("Robot's constructor accepted an absent list validate() had refused");
        } catch (const shulib::PreconditionError& e) {
            CHECK(std::string{e.what()} == std::string{v.why});
        }
    }

    SUBCASE("a one-member side cannot lose its only member") {
        const cfgns::RobotConfig lone = tankConfig(1, cfgns::OdometryKind::DriveEncoders);
        PlantWorld world{kin, worldConfig()};
        world.wire(lone);
        const int absent[] = {11};
        const shulib::robot::BuildVerdict v =
            shulib::robot::validate(lone, world.devices, {.absentDrivePorts = absent});
        CHECK_FALSE(v.ok);
        CHECK(std::string{v.why} == "every RIGHT drive port is absent (ports 11) -- a side needs "
                                    "at least one motor that answers");
    }
    SUBCASE("a list on an X-drive") {
        const shulib::kinematics::MatrixKinematics xkin =
            shulib::kinematics::xDrive(Length{kDriveRadius});
        const cfgns::RobotConfig x = xdriveConfig(false);
        PlantWorld world{xkin, worldConfig()};
        world.wire(x);
        const int absent[] = {2};
        const shulib::robot::BuildVerdict v =
            shulib::robot::validate(x, world.devices, {.absentDrivePorts = absent});
        CHECK_FALSE(v.ok);
        CHECK(std::string{v.why}
              == "absent drive port 2 was given for a x-drive drivetrain -- only a tank side can "
                 "run with an absent member");
        CHECK(world.devices.motorRequestCount(2) == 0);
        CHECK_THROWS_AS((shulib::robot::Robot{x, world.devices, {.absentDrivePorts = absent}}),
                        shulib::PreconditionError);
    }
    SUBCASE("everything else is unchanged: another refused port still refuses the build") {
        PlantWorld world{kin, worldConfig()};
        world.wire(tank);
        world.devices.refusePort(3);
        world.devices.refusePort(13);  // dead, but NOT on the boot check's list
        const int absent[] = {3};
        const shulib::robot::BuildVerdict v =
            shulib::robot::validate(tank, world.devices, {.absentDrivePorts = absent});
        CHECK_FALSE(v.ok);
        CHECK(std::string{v.why} == "a RIGHT drive motor cannot be made: port 13 has no motor");
    }
    SUBCASE("a repeated entry is one absent port, not two") {
        PlantWorld world{kin, worldConfig()};
        world.wire(tank);
        world.devices.refusePort(4);
        const int absent[] = {4, 4, 4};
        shulib::robot::Robot robot{tank, world.devices, {.absentDrivePorts = absent}};
        CHECK(group(robot, 0).presentCount() == 4);
        CHECK(world.devices.motorRequestCount(4) == 0);
    }
}

// ═══ 2a-3. The degraded robot DRIVES, on the plant, and reports ════════════════════════
TEST_CASE("G1 2a builder A4: a robot with dead ports DRIVES the plant, keeps an honest estimate, "
          "reports honestly, raises no false fight — and is NOT the healthy robot") {
    // BUG CAUGHT: an absent member that breaks the drive (a hole commanded, a hole read into the
    // median, the odometry reading a nullptr), one that reads as a FIGHTING member to the
    // coupled-side monitor (a dead port must never raise MOTOR_GROUP_DISAGREE — the monitor is
    // told present = false), and one that changes nothing at all (a "degraded" robot that is
    // secretly whole would pass every other line here, so the run must DIFFER from the healthy
    // twin's).
    //
    // WHAT THIS DOES NOT ASSERT, measured and deliberate: that the degraded robot SETTLES. On
    // this plant it does not — with one member of five silent on a side, the 30 in heading-held
    // moveTo times out a few inches off the line, and even a symmetric loss (one dead per side)
    // misses the settle window. The plant models a silent member as dragging the wheel to the
    // MEAN of its members' volts (drive_plant.hpp, HA-137), i.e. 80 % speed; a real train of
    // four powered motors and one unpowered one is voltage-limited and loses far less speed than
    // that, so the plant is PESSIMISTIC here and the missed settle is a statement about the
    // model and the provisional feedforward, not about robot two. What IS a property of the
    // library is asserted: it drives, it knows where it is, and it says what happened.
    const shulib::kinematics::TankKinematics kin{Length{kTrackWidth}};
    const cfgns::RobotConfig c = tankConfig(5, cfgns::OdometryKind::DriveEncoders);

    BuiltRobot healthy{c, kin, worldConfig()};
    const Trace h = runLeg(healthy.robot.chassis(), healthy.world.pacer, kStart, kTankTarget);
    REQUIRE(h.exit == ExitReason::Settled);

    struct Shape {
        std::string label;
        std::vector<int> absent;
        std::string banner;
    };
    const Shape shapes[] = {{"one dead LEFT member", {3}, "DEGRADED: absent 3 |"},
                            {"one dead per side, the left one the plant's PRIMARY", {1, 13},
                             "DEGRADED: absent 1,13 |"},
                            {"two dead on the right", {12, 14}, "DEGRADED: absent 12,14 |"}};
    for (const Shape& s : shapes) {
        CAPTURE(s.label);
        DegradedRobot degraded{c, std::span<const int>{s.absent.data(), s.absent.size()}, kin,
                               worldConfig()};
        const Trace t = runLeg(degraded.robot.chassis(), degraded.world.pacer, kStart, kTankTarget);
        const Pose2d truth = degraded.world.h.truePose();
        // It REPORTS honestly: settled or timed out — never cancelled by a fault it invented.
        CHECK((t.exit == ExitReason::Settled || t.exit == ExitReason::TimedOut));
        CHECK(degraded.robot.faults().raiseCount(shulib::diag::FaultCode::MotorGroupDisagree)
              == 0);
        // It DRIVES: most of the 30 in leg, on the plant's ground truth.
        CHECK(motion_rig::posErr(truth, kStart) > 20.0);
        // It KNOWS where it is: the estimate — drive-encoder odometry over the groups' medians of
        // the PRESENT members — agrees with the ground truth. A hole read as a member (a zero, or
        // a neighbour's value) is what would pull these apart.
        CHECK(motion_rig::posErr(degraded.robot.chassis().pose(), truth) < 1.0);
        CHECK(motion_rig::headErr(degraded.robot.chassis().pose(), truth) < 0.03);
        // The dead ports were never asked for, and the banner names them.
        for (const int p : s.absent) {
            CHECK(degraded.world.devices.motorRequestCount(static_cast<std::uint8_t>(p)) == 0);
        }
        char buf[320] = "";
        degraded.robot.describe(buf, sizeof buf);
        CHECK(std::string{buf}.find(s.banner) != std::string::npos);
        // A degraded robot is a DIFFERENT robot: the silent members drag the train, so the
        // ground truth cannot match the healthy run's.
        CHECK_FALSE(sameTruth(h.truth, t.truth));
    }
}

// ═══ 2a-5a. portMap(): the session header's field, generated and bounded ═════════════
TEST_CASE("G1 2a builder A5: portMap() is byte-identical to src/main.cpp's tankPortMapString() for "
          "every tank it can describe, and to portMapString()'s shape for the X-drive") {
    // BUG CAUGHT: a generator that drifts from today's text (Part 2b swaps one for the other, and
    // every session header since R3b would change shape under the team's log readers), a sign
    // dropped, a side swapped, the source word lost.
    const shulib::kinematics::TankKinematics kin{Length{kTrackWidth}};

    SUBCASE("the fixture tank, as a Robot") {
        BuiltRobot built{tankConfig(5, cfgns::OdometryKind::DriveEncoders), kin, worldConfig()};
        char got[shulib::diag::kMaxPortMapBytes] = "";
        built.robot.portMap(got, sizeof got);
        CHECK(std::string{got}
              == "MEASURED L +1 -2 +3 -4 +5 R -11 +12 -13 +14 -15 IMU10 ipr 2.0000/2.0000 TW12.00");
        char ref[shulib::diag::kMaxPortMapBytes] = "";
        transcribedTankPortMapString(built.robot.config().drivetrain.tank.description, ref);
        CHECK(std::string{got} == std::string{ref});
    }
    SUBCASE("every width from 1 to 21 ports on a side, TYPED and MEASURED — against the transcription") {
        for (std::uint8_t n = 1; n <= 21; ++n) {
            for (const DescriptionSource src :
                 {DescriptionSource::RobotMeasured, DescriptionSource::TypedTable}) {
                CAPTURE(static_cast<int>(n));
                cfgns::RobotConfig c = wideTankConfig(n);
                c.drivetrain.tank.description.source = src;
                char got[shulib::diag::kMaxPortMapBytes] = "";
                shulib::robot::writePortMap(c, got, sizeof got);
                char ref[shulib::diag::kMaxPortMapBytes] = "";
                transcribedTankPortMapString(c.drivetrain.tank.description, ref);
                CHECK(std::string{got} == std::string{ref});
            }
        }
    }
    SUBCASE("a tank's GPS and pods are named between the IMU and the geometry") {
        cfgns::RobotConfig c = tankConfig(5, cfgns::OdometryKind::TrackingWheels);
        c.sensors.gps = {.present = true, .port = kGpsPort};
        char got[shulib::diag::kMaxPortMapBytes] = "";
        shulib::robot::writePortMap(c, got, sizeof got);
        CHECK(std::string{got}.find(" IMU10 GPS9 ROTF6 ROTL7 ipr 2.0000/2.0000") != std::string::npos);
    }
    SUBCASE("the X-drive, with and without a GPS, and an H-drive") {
        const shulib::kinematics::MatrixKinematics xkin =
            shulib::kinematics::xDrive(Length{kDriveRadius});
        BuiltRobot x{xdriveConfig(true), xkin, worldConfig()};
        char got[shulib::diag::kMaxPortMapBytes] = "";
        x.robot.portMap(got, sizeof got);
        // main.cpp's retired portMapString() format (G1 Part 2c), over this fixture's numbers.
        char ref[shulib::diag::kMaxPortMapBytes] = "";
        std::snprintf(ref, sizeof ref, "FL%d BL%d BR%d FR%d ROTF%d ROTL%d GPS%u IMU%u", 1, 2, 3, 4,
                      6, 7, 9U, 10U);
        CHECK(std::string{got} == std::string{ref});
        CHECK(std::string{got} == "FL1 BL2 BR3 FR4 ROTF6 ROTL7 GPS9 IMU10");

        shulib::robot::writePortMap(xdriveConfig(false), got, sizeof got);
        CHECK(std::string{got} == "FL1 BL2 BR3 FR4 ROTF6 ROTL7 IMU10");

        cfgns::RobotConfig hcfg = xdriveConfig(false);
        hcfg.drivetrain.kind = cfgns::DrivetrainKind::HDrive;
        hcfg.drivetrain.hdrive = {.ports = {-1, 2, 3}, .imuPort = kImuPort};
        shulib::robot::writePortMap(hcfg, got, sizeof got);
        CHECK(std::string{got} == "L-1 R2 S3 ROTF6 ROTL7 IMU10");

        cfgns::RobotConfig unset = xdriveConfig(false);
        unset.drivetrain.kind = static_cast<cfgns::DrivetrainKind>(0);
        shulib::robot::writePortMap(unset, got, sizeof got);
        CHECK(std::string{got} == "UNSET drivetrain");
    }
    SUBCASE("absence is not written into the map: it is what the robot is WIRED to be") {
        const cfgns::RobotConfig c = tankConfig(5, cfgns::OdometryKind::DriveEncoders);
        const int absent[] = {3};
        DegradedRobot degraded{c, absent, kin, worldConfig()};
        char got[shulib::diag::kMaxPortMapBytes] = "";
        degraded.robot.portMap(got, sizeof got);
        char whole[shulib::diag::kMaxPortMapBytes] = "";
        shulib::robot::writePortMap(c, whole, sizeof whole);
        CHECK(std::string{got} == std::string{whole});
    }
}

TEST_CASE("G1 2a builder A6: a 21-port side TRUNCATES — it never overflows, and it always "
          "terminates") {
    // BUG CAUGHT (the brief's mutation "drop the bound"): the generator writing past the 96-byte
    // session-header field. The destination is a larger buffer painted with a canary, and the
    // generator is told the field's size — so an overflow lands inside this test's own array
    // and shows up as a bitten canary instead of as undefined behaviour.
    //
    // Two bounds exist and both are exercised. ONE 21-port side is capped at 39 characters (the
    // side cap tankPortMapString() has), and the line still fits the field. BOTH sides at 21
    // ports cannot fit, and the FIELD bound is what stops it — the one "drop the bound" breaks.
    SUBCASE("one 21-port side: the SIDE is capped, and the line fits") {
        const cfgns::RobotConfig wide = wideTankConfig(21, 1);
        char buf[256];
        std::memset(buf, '\xAB', sizeof buf);
        shulib::robot::writePortMap(wide, buf, shulib::diag::kMaxPortMapBytes);
        const std::string s{buf};
        CAPTURE(s);
        CHECK(s.rfind("MEASURED L +1 -2 +3 -4 +5", 0) == 0);
        CHECK(s.substr(11, 39) == "+1 -2 +3 -4 +5 -6 +7 -8 +9 -10 +11 -12 ");
        CHECK(s.substr(50, 5) == " R -1");
        CHECK(s == "MEASURED L +1 -2 +3 -4 +5 -6 +7 -8 +9 -10 +11 -12  R -1 IMU10 ipr "
                   "2.0000/2.0000 TW12.00");
        for (std::size_t i = s.size() + 1; i < sizeof buf; ++i) {
            CAPTURE(i);
            CHECK(buf[i] == '\xAB');
        }
    }
    SUBCASE("both sides at 21 ports: the FIELD bound stops it at 95 characters and a NUL") {
        const cfgns::RobotConfig wide = wideTankConfig(21, 21);
        char buf[256];
        std::memset(buf, '\xAB', sizeof buf);
        shulib::robot::writePortMap(wide, buf, shulib::diag::kMaxPortMapBytes);
        CHECK(std::strlen(buf) == shulib::diag::kMaxPortMapBytes - 1);  // it filled the field…
        for (std::size_t i = shulib::diag::kMaxPortMapBytes; i < sizeof buf; ++i) {
            CAPTURE(i);
            CHECK(buf[i] == '\xAB');                         // …and not one byte more
        }
        const std::string s{buf};
        CHECK(s.substr(92) == " IM");  // cut mid-token, exactly where the field ends
        char ref[shulib::diag::kMaxPortMapBytes] = "";
        transcribedTankPortMapString(wide.drivetrain.tank.description, ref);
        CHECK(s == std::string{ref});
    }
    SUBCASE("the degenerate caps: nothing through a null or a zero cap; a cap of 1 is \"\"") {
        const cfgns::RobotConfig wide = wideTankConfig(21, 21);
        char tiny[4] = {'x', 'y', 'z', '\0'};
        shulib::robot::writePortMap(wide, tiny, 0);
        CHECK(tiny[0] == 'x');
        shulib::robot::writePortMap(wide, nullptr, 96);
        shulib::robot::writePortMap(wide, tiny, 1);
        CHECK(tiny[0] == '\0');
        CHECK(tiny[1] == 'y');
        char ten[16];
        std::memset(ten, '\xAB', sizeof ten);
        shulib::robot::writePortMap(wide, ten, 10);
        CHECK(std::string{ten} == "MEASURED ");
        CHECK(ten[10] == '\xAB');
    }
}

// ═══ 2a. driveOdometry(): the panel's cross-check, without reaching into private members ═══
TEST_CASE("G1 2a builder A7: driveOdometry() is the Localizer's drive-encoder odometry on a "
          "drive-encoder tank, and nullptr otherwise") {
    const shulib::kinematics::TankKinematics kin{Length{kTrackWidth}};
    BuiltRobot tank{tankConfig(5, cfgns::OdometryKind::DriveEncoders), kin, worldConfig()};
    const shulib::localization::DriveEncoderOdometry* odo = tank.robot.driveOdometry();
    REQUIRE(odo != nullptr);
    CHECK(odo->leftInchesPerRadian().value() == kPlantInchesPerRadian);
    CHECK(odo->trackWidth().value() == kTrackWidth);
    (void)runLeg(tank.robot.chassis(), tank.world.pacer, kStart, kTankTarget);
    // It is the estimator the Localizer runs: its pose moved with the robot, and the same
    // object is handed out every time.
    CHECK(tank.robot.driveOdometry() == odo);
    CHECK(odo->pose().x().value() != 0.0);

    BuiltRobot pods{tankConfig(5, cfgns::OdometryKind::TrackingWheels), kin, worldConfig()};
    CHECK(pods.robot.driveOdometry() == nullptr);
    const shulib::kinematics::MatrixKinematics xkin =
        shulib::kinematics::xDrive(Length{kDriveRadius});
    BuiltRobot x{xdriveConfig(false), xkin, worldConfig()};
    CHECK(x.robot.driveOdometry() == nullptr);
}

// ═══════════════════ T3 — mechanisms built from the config (brief test 14) ═══════════════════
// The builder half of the claws: robot::Robot turns each MechanismConfig with motors into ONE
// hal::MotorMechanism, asks for its motors with the MECHANISM's cartridge, names it, exposes it by
// name and as the span an end-of-run guard walks — and a motor the provider will not make leaves
// THAT mechanism absent, never the robot. The drive must not notice any of it.
namespace {

constexpr std::int8_t kClaw6 = 6;
constexpr std::int8_t kClaw7 = -7;  // stored NEGATIVE: the sign must reach the provider as is

/// Robot two's shape for this file: the five-per-side drive-encoder tank, plus two separate RED
/// claw pivots on 6 and 7 with Hold as their declared safe state.
cfgns::RobotConfig tankWithClaws() {
    cfgns::RobotConfig c = tankConfig(5, cfgns::OdometryKind::DriveEncoders);
    c.mechanisms[0] = {.name = "claw pivot 6",
                       .motorPorts = {kClaw6},
                       .motorCount = 1,
                       .cartridge = MotorCartridge::Red,
                       .safeMode = cfgns::SafeBrakeMode::Hold};
    c.mechanisms[1] = {.name = "claw pivot 7",
                       .motorPorts = {kClaw7},
                       .motorCount = 1,
                       .cartridge = MotorCartridge::Red,
                       .safeMode = cfgns::SafeBrakeMode::Hold};
    c.mechanismCount = 2;
    return c;
}

/// The report contract every Robot keeps (B2 review round 1, finding 3), over every index a boot
/// or panel line could ask for AND past the slot storage: `builtMechanismAt(i)` is non-null
/// EXACTLY when `mechanismAbsence(i)` is empty — a built mechanism of either kind, and nothing
/// else — and when it is non-null it IS the typed lookup's object, the motor one or the pneumatic
/// one, never both. An index not below the declared count finds nothing and says so. (Asking
/// past the storage is safe only because every accessor goes through ONE bound; with that bound
/// removed, index `mechanismCount()` — still inside the array — already fails the sentence check,
/// so the loss is seen without relying on a read past the array.)
void checkReportContract(const shulib::robot::Robot& r) {
    for (std::size_t i = 0; i <= cfgns::kMaxMechanisms + 1; ++i) {
        CAPTURE(i);
        const shulib::hal::IMechanism* built = r.builtMechanismAt(i);
        const std::string why{r.mechanismAbsence(i)};
        CHECK((built != nullptr) == why.empty());
        const shulib::hal::IMechanism* motor = r.mechanismAt(i);
        const shulib::hal::IMechanism* air = r.pneumaticMechanismAt(i);
        CHECK((motor == nullptr || air == nullptr));
        CHECK(built == (motor != nullptr ? motor : air));
        if (i >= r.mechanismCount()) {
            CHECK(built == nullptr);
            CHECK(why == "no mechanism is declared at this index");
        }
    }
}

}  // namespace

TEST_CASE("T3 builder M1: validate() checks a mechanism's CONFIG but never asks for its motors — "
          "and refuses a claw on a drive port before any device is asked for") {
    // BUG CAUGHT: validate() asking for mechanism motors (a refused claw would then refuse the
    // whole robot); a claw sharing a drive port reaching the provider with its RED cartridge
    // (the on-robot provider treats that as a precondition — it throws on the brain).
    const shulib::kinematics::TankKinematics kin{Length{kTrackWidth}};
    SUBCASE("a good claw config: valid, and ports 6 and 7 not asked for by validate()") {
        const cfgns::RobotConfig c = tankWithClaws();
        PlantWorld world{kin, worldConfig()};
        world.wire(c);
        const shulib::robot::BuildVerdict v = shulib::robot::validate(c, world.devices);
        CHECK(v.ok);
        CHECK(world.devices.motorRequestCount(6) == 0);
        CHECK(world.devices.motorRequestCount(7) == 0);
    }
    SUBCASE("a claw on the LEFT drive's port 3: refused, naming both owners, nothing asked for") {
        cfgns::RobotConfig c = tankWithClaws();
        c.mechanisms[0].motorPorts[0] = -3;
        PlantWorld world{kin, worldConfig()};
        world.wire(c);
        const shulib::robot::BuildVerdict v = shulib::robot::validate(c, world.devices);
        CHECK_FALSE(v.ok);
        const std::string why{v.why};
        CHECK(why.find("port 3") != std::string::npos);
        CHECK(why.find("LEFT drive") != std::string::npos);
        CHECK(why.find("mechanism motor") != std::string::npos);
        for (std::uint8_t p = 1; p <= 21; ++p) {
            CHECK(world.devices.motorRequestCount(p) == 0);
        }
        CHECK_THROWS_AS(shulib::robot::Robot(c, world.devices), shulib::PreconditionError);
    }
    SUBCASE("a claw on a tracking-wheel port (a pods robot uses 6 and 7 for its pods)") {
        cfgns::RobotConfig c = tankWithClaws();
        c.odometry = tankConfig(5, cfgns::OdometryKind::TrackingWheels).odometry;
        PlantWorld world{kin, worldConfig()};
        const shulib::robot::BuildVerdict v = shulib::robot::validate(c, world.devices);
        CHECK_FALSE(v.ok);
        CHECK(std::string{v.why}.find("tracking wheel") != std::string::npos);
    }
    SUBCASE("an UNSET cartridge or safe mode is refused before any device is asked for") {
        for (const bool cartridge : {true, false}) {
            CAPTURE(cartridge);
            cfgns::RobotConfig c = tankWithClaws();
            if (cartridge) {
                c.mechanisms[1].cartridge = static_cast<MotorCartridge>(0);
            } else {
                c.mechanisms[1].safeMode = static_cast<cfgns::SafeBrakeMode>(0);
            }
            PlantWorld world{kin, worldConfig()};
            const shulib::robot::BuildVerdict v = shulib::robot::validate(c, world.devices);
            CHECK_FALSE(v.ok);
            const std::string why{v.why};
            CHECK(why.find("'claw pivot 7'") != std::string::npos);
            CHECK(why.find(cartridge ? "cartridge" : "safe brake mode") != std::string::npos);
            for (std::uint8_t p = 1; p <= 21; ++p) {
                CHECK(world.devices.motorRequestCount(p) == 0);
            }
        }
    }
}

TEST_CASE("T3 builder M2: the Robot builds TWO single-motor mechanisms, names them from its OWN "
          "copy of the config, asks with each one's RED cartridge and stored sign, declares Hold, "
          "and commands nothing") {
    // BUG CAUGHT: one two-motor mechanism for two claws (both would move on R1); the DRIVE's
    // cartridge handed to a claw (BLUE gearing on a RED motor — a silent 6x speed error); a sign
    // dropped; a name borrowed from the caller's config (dangling when the caller's copy dies);
    // Coast as the safe mode; a builder that writes a motor while constructing.
    const shulib::kinematics::TankKinematics kin{Length{kTrackWidth}};
    cfgns::RobotConfig c = tankWithClaws();
    PlantWorld world{kin, worldConfig()};
    world.wire(c);
    mech_rig::RecordingMotor six;
    mech_rig::RecordingMotor seven;
    world.devices.bindMotor(6, six);
    world.devices.bindMotor(7, seven);
    shulib::robot::Robot robot{c, world.devices};
    c.mechanisms[0].name[0] = 'X';  // the caller's copy changes; the Robot's must not

    REQUIRE(robot.mechanismCount() == 2);
    shulib::hal::MotorMechanism* m6 = robot.mechanismAt(0);
    shulib::hal::MotorMechanism* m7 = robot.mechanismAt(1);
    REQUIRE(m6 != nullptr);
    REQUIRE(m7 != nullptr);
    CHECK(m6 != m7);
    CHECK(std::string{m6->name()} == "claw pivot 6");
    CHECK(std::string{m7->name()} == "claw pivot 7");
    CHECK(m6->name() == robot.config().mechanisms[0].name);  // borrowed from the ROBOT's copy
    CHECK(robot.mechanism("claw pivot 6") == m6);
    CHECK(robot.mechanism("claw pivot 7") == m7);
    CHECK(m6->safeBrakeMode() == shulib::hal::BrakeMode::Hold);
    CHECK(m7->safeBrakeMode() == shulib::hal::BrakeMode::Hold);
    REQUIRE(m6->motors().size() == 1U);
    REQUIRE(m7->motors().size() == 1U);
    CHECK(m6->motors()[0] == &six);
    CHECK(m7->motors()[0] == &seven);
    CHECK(robot.mechanismAbsence(0)[0] == '\0');
    CHECK(robot.mechanismAbsence(1)[0] == '\0');
    // The cartridge and the sign that reached the provider are the MECHANISM's.
    CHECK(world.devices.motorCartridge(6) == MotorCartridge::Red);
    CHECK(world.devices.motorCartridge(7) == MotorCartridge::Red);
    CHECK(world.devices.motorCartridge(1) == MotorCartridge::Blue);  // the drive keeps its own
    CHECK(world.devices.motorSignedPort(6) == kClaw6);
    CHECK(world.devices.motorSignedPort(7) == kClaw7);
    CHECK(world.devices.motorRequestCount(6) == 1);
    CHECK(world.devices.motorRequestCount(7) == 1);
    // The end-of-run guard's span: both, in config order, as the same objects.
    const std::span<shulib::hal::IMechanism* const> all = robot.mechanisms();
    REQUIRE(all.size() == 2U);
    CHECK(all[0] == static_cast<shulib::hal::IMechanism*>(m6));
    CHECK(all[1] == static_cast<shulib::hal::IMechanism*>(m7));
    // Built, not commanded.
    CHECK(six.eventCount() == 0);
    CHECK(seven.eventCount() == 0);
    // Two mechanisms, not one: commanding 6 does not reach 7.
    m6->setVoltage(shulib::units::Voltage{12.0});
    CHECK(six.eventCount() == 1);
    CHECK(seven.eventCount() == 0);
    // Lookups that must find nothing.
    CHECK(robot.mechanism("claw pivot") == nullptr);  // a prefix is not the name
    CHECK(robot.mechanism("Xlaw pivot 6") == nullptr);
    CHECK(robot.mechanism(nullptr) == nullptr);
    CHECK(robot.mechanismAt(2) == nullptr);
    CHECK(robot.builtMechanismAt(2) == nullptr);
    // The first index past the declared count is still INSIDE the slot array, so a lost bound is
    // seen here as a wrong answer ("" instead of the sentence) — before the index-99 probe below,
    // which without the bound would be a read past the array (undefined behaviour, not a test).
    CHECK(std::string{robot.mechanismAbsence(2)} == "no mechanism is declared at this index");
    CHECK(std::string{robot.mechanismAbsence(99)} == "no mechanism is declared at this index");
    // The kind-neutral question finds both, as the same objects.
    CHECK(robot.builtMechanismAt(0) == static_cast<shulib::hal::IMechanism*>(m6));
    CHECK(robot.builtMechanismAt(1) == static_cast<shulib::hal::IMechanism*>(m7));
    const shulib::robot::Robot& constRobot = robot;
    CHECK(constRobot.mechanism("claw pivot 7") == m7);
    CHECK(constRobot.mechanismAt(0) == m6);
    checkReportContract(robot);
}

TEST_CASE("T3 builder M3: a claw motor the provider refuses leaves THAT mechanism ABSENT with the "
          "provider's sentence — the robot still builds and drives, bit-identically") {
    // BUG CAUGHT: an unplugged claw refusing the whole robot (the driver loses the drive for a
    // claw); an absent mechanism left in the guard's span (a null or a half-built object to make
    // safe); an absent mechanism with an empty reason; a drive graph perturbed by the refusal.
    const shulib::kinematics::TankKinematics kin{Length{kTrackWidth}};
    const cfgns::RobotConfig c = tankWithClaws();

    LonghandTank longhand{kin, 5, cfgns::OdometryKind::DriveEncoders, worldConfig()};
    const Trace a = runLeg(*longhand.chassis, longhand.pacer, kStart, kTankTarget);

    PlantWorld world{kin, worldConfig()};
    world.wire(c);
    world.devices.refusePort(7);
    REQUIRE(shulib::robot::validate(c, world.devices).ok);
    std::optional<shulib::robot::Robot> robot;
    REQUIRE_NOTHROW(robot.emplace(c, world.devices));
    CHECK(robot->mechanismAt(0) != nullptr);
    CHECK(robot->mechanismAt(1) == nullptr);
    CHECK(robot->mechanism("claw pivot 7") == nullptr);
    const std::string why{robot->mechanismAbsence(1)};
    CHECK(why.find("port 7") != std::string::npos);  // the provider's own sentence
    CHECK(why == "port 7 has no motor");
    REQUIRE(robot->mechanisms().size() == 1U);
    CHECK(robot->mechanisms()[0] == static_cast<shulib::hal::IMechanism*>(robot->mechanismAt(0)));
    CHECK(robot->builtMechanismAt(1) == nullptr);  // absent to the kind-neutral question too
    checkReportContract(*robot);
    CHECK(world.devices.motorRequestCount(7) == 1);  // asked once, by the Robot, not by validate()

    const Trace b = runLeg(robot->chassis(), world.pacer, kStart, kTankTarget);
    REQUIRE(a.exit == ExitReason::Settled);
    requireIdentical(a, b);

    SUBCASE("a two-motor mechanism whose SECOND motor is refused is absent as a whole") {
        cfgns::RobotConfig lift = tankWithClaws();
        lift.mechanisms[1] = {.name = "lift",
                              .motorPorts = {16, -17},
                              .motorCount = 2,
                              .cartridge = MotorCartridge::Green,
                              .safeMode = cfgns::SafeBrakeMode::Hold};
        PlantWorld w2{kin, worldConfig()};
        w2.wire(lift);
        w2.devices.refusePort(17);
        const shulib::robot::Robot r{lift, w2.devices};
        CHECK(r.mechanismAt(1) == nullptr);
        CHECK(std::string{r.mechanismAbsence(1)}.find("port 17") != std::string::npos);
        CHECK(w2.devices.motorRequestCount(16) == 1);  // the first was made, then left unused
    }
    SUBCASE("a mechanism with neither motors nor lines is reported as not built, and is not in "
            "the span") {
        // (Until T3's addendum B2 this subcase was a pneumatic-only mechanism, reported "not
        // built"; since B2 that one IS built — builder P1 — and the empty mechanism is the case
        // left with nothing to build.)
        cfgns::RobotConfig air = tankWithClaws();
        air.mechanisms[2] = {.name = "placeholder"};
        air.mechanismCount = 3;
        PlantWorld w3{kin, worldConfig()};
        w3.wire(air);
        shulib::robot::Robot r{air, w3.devices};
        CHECK(r.mechanismCount() == 3);
        CHECK(r.mechanismAt(2) == nullptr);
        CHECK(r.pneumaticMechanismAt(2) == nullptr);
        CHECK(std::string{r.mechanismAbsence(2)}
              == "no motors and no 3-wire lines -- nothing to build");
        CHECK(r.builtMechanismAt(2) == nullptr);
        CHECK(r.mechanisms().size() == 2U);
        checkReportContract(r);
    }
}

TEST_CASE("T3 builder M4: a tank WITH claws configured drives the plant bit-identically to the "
          "hand-wired graph that has none — the mechanisms are invisible to the drive") {
    // BUG CAUGHT: mechanism construction moved ahead of the drive graph (a different provider
    // request order, a different member layout feeding the same objects), or a claw motor wired
    // into a drive group. The no-mechanism pins (5, 5b, 5c, 5d, A2) are unchanged and still run.
    const shulib::kinematics::TankKinematics kin{Length{kTrackWidth}};
    LonghandTank longhand{kin, 5, cfgns::OdometryKind::DriveEncoders, worldConfig()};
    const Trace a = runLeg(*longhand.chassis, longhand.pacer, kStart, kTankTarget);
    BuiltRobot built{tankWithClaws(), kin, worldConfig()};
    const Trace b = runLeg(built.robot.chassis(), built.world.pacer, kStart, kTankTarget);
    REQUIRE(a.exit == ExitReason::Settled);
    REQUIRE(a.ticks > 50);
    requireIdentical(a, b);
    CHECK(built.robot.mechanisms().size() == 2U);
    CHECK(built.robot.motorGroupCount() == 2);
}

TEST_CASE("T3 builder M5: a provider that refuses a claw motor WITHOUT a reason still leaves a "
          "reason; indices past the declared mechanisms find nothing") {
    // BUG CAUGHT: an ABSENT mechanism reported with an empty sentence (a boot line reading
    // "claw pivot 7 ABSENT: " tells nobody anything); an accessor that indexes the slot array with
    // any number it is handed (index 99 reads past eight slots).
    struct SilentRefusal final : shulib::hal::IDeviceProvider {
        explicit SilentRefusal(FakeDeviceProvider& in) : inner{&in} {}
        FakeDeviceProvider* inner;
        [[nodiscard]] IMotor* motor(std::int8_t p, MotorCartridge c) override {
            return DrivetrainDescription::portMagnitude(p) == 7 ? nullptr : inner->motor(p, c);
        }
        [[nodiscard]] shulib::hal::IImu* imu(std::uint8_t p) override { return inner->imu(p); }
        [[nodiscard]] shulib::hal::IGps* gps(std::uint8_t p, Length f, Length l) override {
            return inner->gps(p, f, l);
        }
        [[nodiscard]] shulib::hal::IRotation* rotation(std::uint8_t p) override {
            return inner->rotation(p);
        }
        [[nodiscard]] shulib::hal::IDigitalOut* digitalOut(std::uint8_t line, bool initial) override {
            return inner->digitalOut(line, initial);
        }
        [[nodiscard]] shulib::hal::IClock& clock() override { return inner->clock(); }
        [[nodiscard]] shulib::hal::IBattery& battery() override { return inner->battery(); }
        [[nodiscard]] shulib::hal::ITelemetrySink& telemetry() override {
            return inner->telemetry();
        }
        [[nodiscard]] shulib::motion::ITickPacer& pacer() override { return inner->pacer(); }
        [[nodiscard]] const char* lastRefusal() const noexcept override { return ""; }
    };
    const shulib::kinematics::TankKinematics kin{Length{kTrackWidth}};
    const cfgns::RobotConfig c = tankWithClaws();
    PlantWorld world{kin, worldConfig()};
    world.wire(c);
    SilentRefusal silent{world.devices};
    shulib::robot::Robot robot{c, silent};
    CHECK(robot.mechanismAt(1) == nullptr);
    CHECK(std::string{robot.mechanismAbsence(1)}
          == "the device provider refused a motor and gave no reason");
    CHECK(robot.mechanismAt(0) != nullptr);
    CHECK(std::string{robot.mechanismAbsence(0)}.empty());
    // Past the declared count, and past the slot array itself: nothing, no read past it, and a
    // sentence rather than the empty string that means "built" (B2 review round 1, finding 3).
    for (const std::size_t i : {std::size_t{2}, std::size_t{7}, std::size_t{8}, std::size_t{99}}) {
        CAPTURE(i);
        CHECK(robot.mechanismAt(i) == nullptr);
        CHECK(robot.builtMechanismAt(i) == nullptr);
        CHECK(std::string{robot.mechanismAbsence(i)} == "no mechanism is declared at this index");
    }
    checkReportContract(robot);
}

TEST_CASE("T3 builder M6: each mechanism is built with ITS OWN declared safe mode — Coast, Brake "
          "and Hold side by side — and with EVERY one of its motors") {
    // BUG CAUGHT (review round 1; both mutants SURVIVED every earlier case, because every
    // mechanism built above declares Hold and has one motor): a Robot that ignores the declared
    // safe mode and builds Hold for everything (an intake declared Coast would sit at stall
    // current when jammed, the mechanism.hpp banner's own warning); a two-motor mechanism built
    // over its FIRST motor only (a config-built lift or intake would drive one motor and leave
    // its partner on the same shaft undriven).
    const shulib::kinematics::TankKinematics kin{Length{kTrackWidth}};
    cfgns::RobotConfig c = tankConfig(5, cfgns::OdometryKind::DriveEncoders);
    c.mechanisms[0] = {.name = "intake",
                       .motorPorts = {16, -17},
                       .motorCount = 2,
                       .cartridge = MotorCartridge::Green,
                       .safeMode = cfgns::SafeBrakeMode::Coast};
    c.mechanisms[1] = {.name = "arm",
                       .motorPorts = {18},
                       .motorCount = 1,
                       .cartridge = MotorCartridge::Red,
                       .safeMode = cfgns::SafeBrakeMode::Brake};
    c.mechanisms[2] = {.name = "claw",
                       .motorPorts = {-19},
                       .motorCount = 1,
                       .cartridge = MotorCartridge::Red,
                       .safeMode = cfgns::SafeBrakeMode::Hold};
    c.mechanismCount = 3;
    REQUIRE(c.valid(nullptr, 0));
    PlantWorld world{kin, worldConfig()};
    world.wire(c);
    mech_rig::RecordingMotor m16;
    mech_rig::RecordingMotor m17;
    mech_rig::RecordingMotor m18;
    mech_rig::RecordingMotor m19;
    world.devices.bindMotor(16, m16);
    world.devices.bindMotor(17, m17);
    world.devices.bindMotor(18, m18);
    world.devices.bindMotor(19, m19);
    shulib::robot::Robot robot{c, world.devices};

    shulib::hal::MotorMechanism* intake = robot.mechanism("intake");
    shulib::hal::MotorMechanism* arm = robot.mechanism("arm");
    shulib::hal::MotorMechanism* claw = robot.mechanism("claw");
    REQUIRE(intake != nullptr);
    REQUIRE(arm != nullptr);
    REQUIRE(claw != nullptr);
    CHECK(robot.mechanisms().size() == 3U);

    // Each one's declared mode, not one mode for all.
    CHECK(intake->safeBrakeMode() == shulib::hal::BrakeMode::Coast);
    CHECK(arm->safeBrakeMode() == shulib::hal::BrakeMode::Brake);
    CHECK(claw->safeBrakeMode() == shulib::hal::BrakeMode::Hold);

    // The intake is BOTH of its motors, in config order, each asked for with its own sign.
    REQUIRE(intake->motors().size() == 2U);
    CHECK(intake->motors()[0] == &m16);
    CHECK(intake->motors()[1] == &m17);
    CHECK(world.devices.motorSignedPort(16) == 16);
    CHECK(world.devices.motorSignedPort(17) == -17);
    CHECK(world.devices.motorCartridge(16) == MotorCartridge::Green);
    CHECK(world.devices.motorCartridge(17) == MotorCartridge::Green);
    CHECK(world.devices.motorSignedPort(19) == -19);
    REQUIRE(arm->motors().size() == 1U);
    REQUIRE(claw->motors().size() == 1U);

    // One command reaches both intake motors — and nothing else.
    intake->setVoltage(shulib::units::Voltage{6.0});
    REQUIRE(m16.eventCount() == 1);
    REQUIRE(m17.eventCount() == 1);
    CHECK(m16.events()[0].kind == mech_rig::RecordingMotor::Event::Kind::Voltage);
    CHECK(m16.events()[0].volts == 6.0);
    CHECK(m17.events()[0].volts == 6.0);
    CHECK(m18.eventCount() == 0);
    CHECK(m19.eventCount() == 0);

    // The safe state, at the MOTORS: each mechanism's own mode first, then 0 V — on every motor.
    using Kind = mech_rig::RecordingMotor::Event::Kind;
    const auto safeWith = [](const mech_rig::RecordingMotor& m, shulib::hal::BrakeMode mode) {
        const auto& ev = m.events();
        return ev.size() >= 2 && ev[ev.size() - 2].kind == Kind::Brake
               && ev[ev.size() - 2].mode == mode && ev.back().kind == Kind::Voltage
               && ev.back().volts == 0.0;
    };
    intake->applySafeState();
    arm->applySafeState();
    claw->applySafeState();
    CHECK(safeWith(m16, shulib::hal::BrakeMode::Coast));
    CHECK(safeWith(m17, shulib::hal::BrakeMode::Coast));
    CHECK(safeWith(m18, shulib::hal::BrakeMode::Brake));
    CHECK(safeWith(m19, shulib::hal::BrakeMode::Hold));
}

// ═══════════════ T3's addendum B2 — the pneumatic claw built from the config ═══════════════
// The builder half of the claw on 3-wire port A: robot::Robot turns each MechanismConfig with
// 3-wire lines into ONE hal::PneumaticMechanism, asks the provider for each line at the config's
// START level (making a line DRIVES it — HA-119 — and the fake provider does exactly that, once),
// declares its SAFE level, names it, exposes it by name and in the guard's span — and a line the
// provider will not make leaves THAT mechanism absent, never the robot. The drive must not notice.
namespace {

/// A 3-wire line that records EVERY level it is driven to, in order.
class RecordingLine final : public shulib::hal::IDigitalOut {
public:
    void set(bool value) override {
        writes.push_back(value);
        level = value;
    }
    [[nodiscard]] bool commanded() const override { return level; }
    std::vector<bool> writes;
    bool level = false;
};

/// tankWithClaws() plus a pneumatic claw on 3-wire port A that STARTS High and is SAFE Low — two
/// different levels on purpose, so a builder that swapped them, or made the line at a default,
/// cannot pass.
cfgns::RobotConfig tankWithAirClaw() {
    cfgns::RobotConfig c = tankWithClaws();
    c.mechanisms[2] = {.name = "claw",
                       .pneumaticPorts = {1},
                       .pneumaticCount = 1,
                       .pneumaticSafe = cfgns::PneumaticLevel::Low,
                       .pneumaticStart = cfgns::PneumaticLevel::High};
    c.mechanismCount = 3;
    return c;
}

}  // namespace

TEST_CASE("B2 builder FP1: the fake provider's digital out — the same line is the SAME object, "
          "DRIVEN once at the stated level when made and never again; out of range and refused "
          "lines are refused by name") {
    // BUG CAUGHT: a provider that makes a fresh line per request (half the graph on an object
    // nobody commands); one that re-drives the line on every request (the claw snaps back to its
    // boot level whenever anything asks for it again); one that never drives it at all (a host
    // test would then pass a builder that states no boot level); a letter byte translated.
    const shulib::kinematics::TankKinematics kin{Length{kTrackWidth}};
    PlantWorld world{kin, worldConfig()};
    FakeDeviceProvider& d = world.devices;
    shulib::hal::IDigitalOut* a = d.digitalOut(1, true);
    REQUIRE(a != nullptr);
    CHECK(a == &d.fakeDigitalOut(1));
    CHECK(d.fakeDigitalOut(1).setCount() == 1);  // made = driven, once
    CHECK(a->commanded());
    CHECK(d.digitalOutMade(1));
    CHECK(d.digitalOutInitialState(1));
    a->set(false);
    CHECK(d.digitalOut(1, true) == a);            // the same object…
    CHECK(d.fakeDigitalOut(1).setCount() == 2);   // …and NOT driven again
    CHECK_FALSE(a->commanded());
    CHECK(d.digitalOutRequestCount(1) == 2);
    CHECK_THROWS_AS((void)d.digitalOut(1, false), shulib::PreconditionError);
    // Another line is another object, made at ITS level.
    shulib::hal::IDigitalOut* h = d.digitalOut(8, false);
    REQUIRE(h != nullptr);
    CHECK(h != a);
    CHECK(d.fakeDigitalOut(8).setCount() == 1);
    CHECK_FALSE(d.digitalOutInitialState(8));
    // Out of range, by number — 'A' (65) included: one spelling at the seam.
    for (const int bad : {0, 9, 65, 255}) {
        CAPTURE(bad);
        CHECK(d.digitalOut(static_cast<std::uint8_t>(bad), true) == nullptr);
        CHECK(std::string{d.lastRefusal()}
              == "3-wire port " + std::to_string(bad)
                     + " is not a brain 3-wire port (1..8 are A..H)");
    }
    CHECK(d.fakeDigitalOut(1).setCount() == 2);  // the letter did not reach line 1
    // A refused line: counted, refused by letter, never made, never driven.
    d.refuseLine(3);
    CHECK(d.digitalOut(3, true) == nullptr);
    CHECK(std::string{d.lastRefusal()} == "3-wire port C (line 3) has no digital out");
    CHECK(d.digitalOutRequestCount(3) == 1);
    CHECK_FALSE(d.digitalOutMade(3));
    CHECK(d.fakeDigitalOut(3).setCount() == 0);
    d.allowLine(3);
    CHECK(d.digitalOut(3, true) != nullptr);
    // A bound line is the one served — and it is driven when made, like any other.
    RecordingLine mine;
    d.bindDigitalOut(5, mine);
    CHECK(d.digitalOut(5, false) == &mine);
    CHECK(mine.writes == std::vector<bool>{false});
}

TEST_CASE("B2 builder P1: the Robot builds and NAMES the pneumatic claw — one PneumaticMechanism "
          "over its line, made at the START level, safe at the SAFE level, found by "
          "pneumaticMechanism(), in the guard's span — and the drive is bit-identical") {
    // BUG CAUGHT: the line made at the SAFE level or at a default (the claw opens or closes at boot
    // on a level nobody stated); the safe level taken from the start level; a name borrowed from
    // the caller's config; the claw left out of the guard's span (an end-of-run guard could not
    // stop it); the motor lookup handing back a pneumatic mechanism; validate() asking for the line
    // (asking DRIVES it); a claw that perturbs the drive.
    const shulib::kinematics::TankKinematics kin{Length{kTrackWidth}};
    cfgns::RobotConfig c = tankWithAirClaw();
    {
        PlantWorld probe{kin, worldConfig()};
        probe.wire(c);
        REQUIRE(shulib::robot::validate(c, probe.devices).ok);
        CHECK(probe.devices.digitalOutRequestCount(1) == 0);  // validate() never asks for a line
    }
    LonghandTank longhand{kin, 5, cfgns::OdometryKind::DriveEncoders, worldConfig()};
    const Trace a = runLeg(*longhand.chassis, longhand.pacer, kStart, kTankTarget);

    PlantWorld world{kin, worldConfig()};
    world.wire(c);
    RecordingLine lineA;
    world.devices.bindDigitalOut(1, lineA);
    std::optional<shulib::robot::Robot> robot;
    robot.emplace(c, world.devices);
    c.mechanisms[2].name[0] = 'X';  // the caller's copy changes; the Robot's must not

    // Made at START (High) — exactly once, and nothing else was put on the line.
    CHECK(lineA.writes == std::vector<bool>{true});
    CHECK(world.devices.digitalOutRequestCount(1) == 1);
    CHECK(world.devices.digitalOutInitialState(1));

    shulib::hal::PneumaticMechanism* claw = robot->pneumaticMechanism("claw");
    REQUIRE(claw != nullptr);
    CHECK(std::string{claw->name()} == "claw");
    CHECK(claw->name() == robot->config().mechanisms[2].name);  // borrowed from the ROBOT's copy
    CHECK(claw->safeCommand() == false);                        // SAFE is Low, not START's High
    REQUIRE(claw->lines().size() == 1U);
    CHECK(claw->lines()[0] == &lineA);
    CHECK(robot->pneumaticMechanismAt(2) == claw);
    CHECK(robot->mechanismAt(2) == nullptr);           // the MOTOR lookup does not hand it out
    CHECK(robot->mechanism("claw") == nullptr);
    CHECK(robot->pneumaticMechanism("claw pivot 6") == nullptr);  // and the reverse
    CHECK(robot->pneumaticMechanismAt(0) == nullptr);
    CHECK(robot->mechanism("claw pivot 6") != nullptr);
    CHECK(std::string{robot->mechanismAbsence(2)}.empty());
    // The kind-neutral question — the one a boot line asks — finds the claw: the typed motor
    // lookup's nullptr above is NOT "absent" (B2 review round 1, finding 3).
    CHECK(robot->builtMechanismAt(2) == static_cast<shulib::hal::IMechanism*>(claw));
    CHECK(robot->pneumaticMechanism("Xlaw") == nullptr);
    CHECK(robot->pneumaticMechanism(nullptr) == nullptr);
    CHECK(robot->pneumaticMechanismAt(3) == nullptr);
    // Inside the slot array, so a lost bound shows here, before the index-99 probe (see M2).
    CHECK(std::string{robot->mechanismAbsence(3)} == "no mechanism is declared at this index");
    CHECK(robot->pneumaticMechanismAt(99) == nullptr);
    const shulib::robot::Robot& constRobot = *robot;
    CHECK(constRobot.pneumaticMechanism("claw") == claw);
    CHECK(constRobot.pneumaticMechanismAt(2) == claw);
    CHECK(constRobot.builtMechanismAt(2) == static_cast<const shulib::hal::IMechanism*>(claw));
    checkReportContract(constRobot);
    // The guard's span: both pivots, then the claw, in config order.
    const std::span<shulib::hal::IMechanism* const> all = robot->mechanisms();
    REQUIRE(all.size() == 3U);
    CHECK(all[0] == static_cast<shulib::hal::IMechanism*>(robot->mechanismAt(0)));
    CHECK(all[1] == static_cast<shulib::hal::IMechanism*>(robot->mechanismAt(1)));
    CHECK(all[2] == static_cast<shulib::hal::IMechanism*>(claw));
    // Through the span, the guard's one capability lands the SAFE level on the line.
    all[2]->applySafeState();
    CHECK(lineA.writes == (std::vector<bool>{true, false}));
    claw->set(true);
    CHECK(lineA.writes.back());

    const Trace b = runLeg(robot->chassis(), world.pacer, kStart, kTankTarget);
    REQUIRE(a.exit == ExitReason::Settled);
    requireIdentical(a, b);
    CHECK(lineA.writes.size() == 3U);  // driving the robot put nothing on the claw

    SUBCASE("a TWO-line mechanism: both lines, in config order, each made at the start level") {
        cfgns::RobotConfig two = tankWithClaws();
        two.mechanisms[2] = {.name = "clamp",
                             .pneumaticPorts = {5, 3},
                             .pneumaticCount = 2,
                             .pneumaticSafe = cfgns::PneumaticLevel::High,
                             .pneumaticStart = cfgns::PneumaticLevel::Low};
        two.mechanismCount = 3;
        PlantWorld w2{kin, worldConfig()};
        w2.wire(two);
        RecordingLine e;
        RecordingLine cLine;
        w2.devices.bindDigitalOut(5, e);
        w2.devices.bindDigitalOut(3, cLine);
        const shulib::robot::Robot r{two, w2.devices};
        const shulib::hal::PneumaticMechanism* clamp = r.pneumaticMechanism("clamp");
        REQUIRE(clamp != nullptr);
        REQUIRE(clamp->lines().size() == 2U);
        CHECK(clamp->lines()[0] == &e);
        CHECK(clamp->lines()[1] == &cLine);
        CHECK(e.writes == std::vector<bool>{false});
        CHECK(cLine.writes == std::vector<bool>{false});
        CHECK(clamp->safeCommand());
    }
}

TEST_CASE("B2 builder P2: a 3-wire line the provider refuses leaves THAT mechanism ABSENT with the "
          "provider's sentence — the robot still builds and drives, bit-identically") {
    // BUG CAUGHT: a refused line refusing the whole robot (the driver loses the drive for a claw);
    // an absent claw left in the guard's span; an absent claw with an empty reason; a claw built
    // over the lines that WERE made (a two-valve clamp firing half its cylinders).
    const shulib::kinematics::TankKinematics kin{Length{kTrackWidth}};
    const cfgns::RobotConfig c = tankWithAirClaw();
    LonghandTank longhand{kin, 5, cfgns::OdometryKind::DriveEncoders, worldConfig()};
    const Trace a = runLeg(*longhand.chassis, longhand.pacer, kStart, kTankTarget);

    PlantWorld world{kin, worldConfig()};
    world.wire(c);
    world.devices.refuseLine(1);
    REQUIRE(shulib::robot::validate(c, world.devices).ok);
    std::optional<shulib::robot::Robot> robot;
    REQUIRE_NOTHROW(robot.emplace(c, world.devices));
    CHECK(robot->pneumaticMechanismAt(2) == nullptr);
    CHECK(robot->pneumaticMechanism("claw") == nullptr);
    CHECK(std::string{robot->mechanismAbsence(2)} == "3-wire port A (line 1) has no digital out");
    CHECK(robot->builtMechanismAt(2) == nullptr);
    REQUIRE(robot->mechanisms().size() == 2U);  // the two pivots, not the claw
    CHECK(robot->mechanismAt(0) != nullptr);
    CHECK(robot->mechanismAt(1) != nullptr);
    checkReportContract(*robot);
    const Trace b = runLeg(robot->chassis(), world.pacer, kStart, kTankTarget);
    REQUIRE(a.exit == ExitReason::Settled);
    requireIdentical(a, b);

    SUBCASE("a two-line mechanism whose SECOND line is refused is absent as a whole") {
        cfgns::RobotConfig two = tankWithClaws();
        two.mechanisms[2] = {.name = "clamp",
                             .pneumaticPorts = {2, 4},
                             .pneumaticCount = 2,
                             .pneumaticSafe = cfgns::PneumaticLevel::Low,
                             .pneumaticStart = cfgns::PneumaticLevel::Low};
        two.mechanismCount = 3;
        PlantWorld w2{kin, worldConfig()};
        w2.wire(two);
        w2.devices.refuseLine(4);
        shulib::robot::Robot r{two, w2.devices};
        CHECK(r.pneumaticMechanismAt(2) == nullptr);
        CHECK(std::string{r.mechanismAbsence(2)} == "3-wire port D (line 4) has no digital out");
        CHECK(r.mechanisms().size() == 2U);
        // The first line was made (and so driven to the start level), then left unused.
        CHECK(w2.devices.digitalOutMade(2));
        CHECK(w2.devices.digitalOutRequestCount(4) == 1);
    }
    SUBCASE("a provider that refuses a line WITHOUT a reason still leaves a reason") {
        struct SilentLine final : shulib::hal::IDeviceProvider {
            explicit SilentLine(FakeDeviceProvider& in) : inner{&in} {}
            FakeDeviceProvider* inner;
            [[nodiscard]] IMotor* motor(std::int8_t p, MotorCartridge m) override {
                return inner->motor(p, m);
            }
            [[nodiscard]] shulib::hal::IImu* imu(std::uint8_t p) override { return inner->imu(p); }
            [[nodiscard]] shulib::hal::IGps* gps(std::uint8_t p, Length f, Length l) override {
                return inner->gps(p, f, l);
            }
            [[nodiscard]] shulib::hal::IRotation* rotation(std::uint8_t p) override {
                return inner->rotation(p);
            }
            [[nodiscard]] shulib::hal::IDigitalOut* digitalOut(std::uint8_t, bool) override {
                return nullptr;
            }
            [[nodiscard]] shulib::hal::IClock& clock() override { return inner->clock(); }
            [[nodiscard]] shulib::hal::IBattery& battery() override { return inner->battery(); }
            [[nodiscard]] shulib::hal::ITelemetrySink& telemetry() override {
                return inner->telemetry();
            }
            [[nodiscard]] shulib::motion::ITickPacer& pacer() override { return inner->pacer(); }
            [[nodiscard]] const char* lastRefusal() const noexcept override { return ""; }
        };
        PlantWorld w3{kin, worldConfig()};
        w3.wire(c);
        SilentLine silent{w3.devices};
        shulib::robot::Robot r{c, silent};
        CHECK(r.pneumaticMechanismAt(2) == nullptr);
        CHECK(std::string{r.mechanismAbsence(2)}
              == "the device provider refused a 3-wire line and gave no reason");
        CHECK(r.mechanisms().size() == 2U);
    }
}

TEST_CASE("B2 builder P3: an index past the declared mechanisms finds nothing — the first slot past "
          "the storage included, on a heap-placed Robot") {
    // BUG CAUGHT: an accessor that indexes the slot array with any number it is handed.
    // HISTORY, because it is why the accessors look the way they do. Each accessor used to carry
    // its own bound, and for an index between the declared count and the capacity the slots are
    // empty anyway — so only an index at or past the capacity told a bound apart from its
    // absence, and there the missing bound is a read past the array: undefined behaviour, which
    // the ordinary suite cannot promise to see (measured: mutant MR28, pneumaticMechanismAt()
    // without its bound, SURVIVED one campaign and was caught in the next only because a stray
    // byte happened to be non-zero; AddressSanitizer caught it every time, here). Since B2's
    // review round 1 every indexed accessor goes through ONE private bound, and an undeclared
    // index answers a SENTENCE from mechanismAbsence() where an empty slot holds "" — so index 3
    // (inside the array) already shows a lost bound, deterministically, in the ordinary suite.
    // The heap placement and the index kMaxMechanisms stay: under AddressSanitizer a read past
    // the array still lands in the allocation's redzone and is reported, as a second witness.
    const shulib::kinematics::TankKinematics kin{Length{kTrackWidth}};
    const cfgns::RobotConfig c = tankWithAirClaw();
    PlantWorld world{kin, worldConfig()};
    world.wire(c);
    auto robot = std::make_unique<shulib::robot::Robot>(c, world.devices);
    REQUIRE(robot->pneumaticMechanismAt(2) != nullptr);
    for (const std::size_t i : {std::size_t{3}, cfgns::kMaxMechanisms - 1, cfgns::kMaxMechanisms,
                                cfgns::kMaxMechanisms + 1}) {
        CAPTURE(i);
        CHECK(robot->pneumaticMechanismAt(i) == nullptr);
        CHECK(robot->mechanismAt(i) == nullptr);
        CHECK(robot->builtMechanismAt(i) == nullptr);
        CHECK(std::string{robot->mechanismAbsence(i)} == "no mechanism is declared at this index");
        const shulib::robot::Robot& constRobot = *robot;
        CHECK(constRobot.pneumaticMechanismAt(i) == nullptr);
        CHECK(constRobot.mechanismAt(i) == nullptr);
        CHECK(constRobot.builtMechanismAt(i) == nullptr);
    }
}

TEST_CASE("B2 builder P4: a boot report that asks builtMechanismAt() tells each declared mechanism "
          "the truth — a working pneumatic claw is BUILT, never 'ABSENT' with an empty reason") {
    // BUG CAUGHT (B2 review round 1, finding 3): the loop a boot line or a panel row writes —
    // "for each declared mechanism: built, or ABSENT with its reason" — written against the typed
    // motor lookup, which is nullptr for a BUILT pneumatic mechanism, reports the working claw as
    // "ABSENT: " with nothing after it. The report below is written the way the header says to
    // write it, over one robot holding every case: a built motor mechanism, a motor mechanism
    // whose motor was refused, the built claw, a pneumatic mechanism whose line was refused, and
    // one with nothing to build.
    const shulib::kinematics::TankKinematics kin{Length{kTrackWidth}};
    cfgns::RobotConfig c = tankWithAirClaw();  // pivots 0 and 1, the claw on line 1 at index 2
    c.mechanisms[3] = {.name = "clamp",
                       .pneumaticPorts = {4},
                       .pneumaticCount = 1,
                       .pneumaticSafe = cfgns::PneumaticLevel::Low,
                       .pneumaticStart = cfgns::PneumaticLevel::Low};
    c.mechanisms[4] = {.name = "placeholder"};
    c.mechanismCount = 5;
    REQUIRE(c.valid());
    PlantWorld world{kin, worldConfig()};
    world.wire(c);
    world.devices.refusePort(7);
    world.devices.refuseLine(4);
    const shulib::robot::Robot robot{c, world.devices};

    std::vector<std::string> report;
    for (std::size_t i = 0; i < robot.mechanismCount(); ++i) {
        const shulib::hal::IMechanism* m = robot.builtMechanismAt(i);
        report.push_back(m != nullptr ? std::string{m->name()} + ": built"
                                      : std::string{robot.config().mechanisms[i].name}
                                            + " ABSENT: " + robot.mechanismAbsence(i));
    }
    CHECK(report == std::vector<std::string>{
                        "claw pivot 6: built",
                        "claw pivot 7 ABSENT: port 7 has no motor",
                        "claw: built",
                        "clamp ABSENT: 3-wire port D (line 4) has no digital out",
                        "placeholder ABSENT: no motors and no 3-wire lines -- nothing to build"});
    // The trap, shown: the typed motor lookup says nullptr for the built claw.
    CHECK(robot.mechanismAt(2) == nullptr);
    CHECK(robot.builtMechanismAt(2) != nullptr);
    checkReportContract(robot);
}
