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
// nothing. They are transcriptions of src/main.cpp's TankRobot and X-drive Robot, and of
// chassis_facade_test.cpp's standalone case, with the harness standing in for hal/pros.
//
// ── NO HARDWARE CLAIM ───────────────────────────────────────────────────────────────
// Everything here runs on the A2 plant behind the F4 fakes. No config has built a graph over a
// real device; src/main.cpp is untouched and still wires both of its graphs by hand.

#include "doctest.h"

#include <array>
#include <cmath>
#include <cstring>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "motion_test_rig.hpp"
#include "shulib/config/robot_config.hpp"
#include "shulib/diag/fault.hpp"
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

/// src/main.cpp's TankRobot, by hand, over the plant: five coupled motors per side behind two
/// MotorGroups, drive-encoder odometry over those groups, no GPS. Member order IS initialization
/// order, and it is the order robot/robot.hpp declares its own members in.
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
        // src/main.cpp's tankChassisConfig(), by hand: the stall check is told the drivetrain's
        // real geometry, and told that a drive-encoder odometry is not an independent motion
        // source. This is the one thing robot::Robot derives rather than asking a person for.
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
    // drive-encoder tank (what src/main.cpp's TankRobot actually is), and this is GroupRig's
    // exact composition, reached from a config by choosing a different odometry kind.
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
// src/main.cpp's tankChassisConfig() tells the stall check the drivetrain's real per-wheel
// geometry and tells it that a drive-encoder odometry is not an independent motion source. The
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
        // src/main.cpp's X-drive graph does: `Chassis chassis{deps, pacer}`, default config.
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
