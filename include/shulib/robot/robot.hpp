#pragma once
//
// robot::Robot — a config::RobotConfig, wired (chunk G1 Part 1, 2026-09-18).
//
// ── What this class is ──────────────────────────────────────────────────────────────
// The composition root, written once, for every robot this library will ever run. It owns
// the whole PROS-free object graph — kinematics, motor groups, geometry, odometry, fusion,
// the GPS corrector when there is a GPS, the Localizer, the fault latch, the health
// monitor, the RobotContext, the MotionDeps and the Chassis — and it BORROWS the devices
// from an hal::IDeviceProvider, which is the one object that knows what a port is on this
// target. Two lines at the composition root:
//
//     ProsDeviceProvider devices;                 // the adapters, made on demand  (Part 2)
//     robot::Robot robot{kRobotConfig, devices};  // the whole graph, wired
//
// and `robot.chassis()` is the same facade a hand-wired graph produces.
//
// ── IT ADDS A WAY TO WIRE; IT NEVER CHANGES THE PHYSICS (ruling B4) ─────────────────
// The graph below is the SAME objects, in the SAME order, with the SAME arguments as
// src/main.cpp's two hand-written graphs and as the host rigs. That is not an aspiration:
// it is the acceptance test. test/robot_builder_test.cpp drives a builder-made Chassis and
// a hand-wired one across the A2 plant and requires the per-tick ground-truth samples to be
// memcmp-EQUAL, the commanded volts equal, and the exit reason and tick count equal —
// through a clean and a hostile scenario. If the builder and the longhand disagree by one
// ulp, the builder has changed something, and a rounding note is not an acceptable answer.
//
// THE BUILDER CONSTRUCTS AND NEVER TICKS. No localizer update, no motion tick, no motor
// write happens anywhere in this file. That is what makes the ordering above a discipline
// rather than a hazard — nothing here has an observable time-ordering, because no time
// passes and no device is written during construction.
//
// ── validate() is PURE and TOTAL; construction PRECONDITIONS on it (ruling B3) ──────
// A robot that cannot be built says WHICH PORT OR FIELD before a single object exists. That
// is the R3b refusal-at-boot rule, made library: `validate(cfg, devices)` checks the config
// against its own rules, makes the F5 wheel-count cross-check, and asks the provider for
// every device the config names — returning a verdict with a sentence. The constructor runs
// exactly that check FIRST and throws PreconditionError carrying the same sentence.
//
// Rejected: a factory returning std::optional<Robot>. Chassis is non-copyable and
// non-movable (its scheduler is pinned by a self-referential command-id stamp), so Robot is
// too, and returning one by value would depend on elision games the V5 toolchain should not
// be asked to play. A Robot is constructed WHERE IT WILL LIVE: `static Robot r{…}` in a
// main, `std::optional<Robot>::emplace` in a test.
//
// ── ABSENT IS NOT NULL ──────────────────────────────────────────────────────────────
// A robot with no GPS gets hal::AbsentGps in its context and NO corrector in its Localizer.
// It never gets a nullptr (RobotContext's preconditions throw on one, and they are right to)
// and it never gets a fake — a test double inside a competition binary erases the difference
// between "no device is installed" and "a test injects here". Tags and vision are ABSENT
// unconditionally in Part 1, because no camera adapter exists yet (R2 owns it) and a field
// for hardware the library cannot open would be a promise rather than a description.
//
// ── NO HARDWARE CLAIM ───────────────────────────────────────────────────────────────
// A Robot built from a config has driven the A2 host plant, through fakes, and nothing else.
// No config has ever been used on a physical robot; src/main.cpp is unchanged and still
// wires both of its graphs by hand — collapsing it is Part 2.
//
// PROS-free, sim-free, allocation-free in the control path (the graph is one in-place
// aggregate, sized by hal::kMaxSmartPorts — no unique_ptr anywhere, as with every other
// graph in this tree).

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <span>

#include "shulib/chassis/chassis.hpp"
#include "shulib/chassis/robot_context.hpp"
#include "shulib/config/robot_config.hpp"
#include "shulib/core/check.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/diag/health_monitor.hpp"
#include "shulib/hal/absent_gps.hpp"
#include "shulib/hal/absent_tag_source.hpp"
#include "shulib/hal/absent_vision.hpp"
#include "shulib/hal/device_provider.hpp"
#include "shulib/hal/drive_geometry.hpp"
#include "shulib/hal/motor_group.hpp"
#include "shulib/kinematics/h_drive.hpp"
#include "shulib/kinematics/matrix_kinematics.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/kinematics/wheel_speeds.hpp"
#include "shulib/kinematics/x_drive.hpp"
#include "shulib/localization/complementary_fusion.hpp"
#include "shulib/localization/drive_encoder_odometry.hpp"
#include "shulib/localization/gps_corrector.hpp"
#include "shulib/localization/i_corrector.hpp"
#include "shulib/localization/localizer.hpp"
#include "shulib/localization/odometry.hpp"
#include "shulib/localization/pilons_odometry.hpp"
#include "shulib/localization/tracking_wheel.hpp"
#include "shulib/motion/motion.hpp"
#include "shulib/motion/motion_scheduler.hpp"

namespace shulib::robot {

/// The answer to "can this robot be built, on this target, right now?" — one bool and one
/// sentence. The sentence is what a person reads off the brain screen when the robot refuses to
/// boot, so it names the PORT or the FIELD and nothing else; 192 bytes is enough for the longest
/// message any check below can produce and small enough to sit on a boot stack.
struct BuildVerdict {
    /// True when every check passed and `Robot` may be constructed from the same pair.
    bool ok = false;
    /// The FIRST thing that was wrong, naming the port or the config field. Empty when `ok`.
    char why[192]{};
};

/// Can `cfg` be built with `devices`? PURE, TOTAL, and the precondition `Robot`'s constructor
/// runs (ruling B3). Three passes, in this order:
///
///  1. **The config's own rules** — `config::RobotConfig::valid()`, whose message (including a
///     tank description's own refusal, verbatim) surfaces unchanged.
///  2. **The F5 cross-check** — the number of KINEMATIC WHEELS the drivetrain has must equal the
///     number of `hal::IMotor` handles the builder will put in the context. This is the same
///     check `motion::MotionDeps::validate()` makes at the bottom of the stack, and it is the
///     ONLY thing standing between a config typo and an unchecked `std::span::operator[]` on the
///     hot path — so it is made HERE too, before the deps exist, where it can name the config
///     field instead of saying "3 != 4". (It lives in exactly these two places and nowhere else;
///     `RobotConfig::valid()` deliberately does not duplicate it — see its documentation.)
///  3. **Every device obtainable** — each drive port, the IMU, the GPS if one is configured, and
///     each tracking-wheel encoder is requested from the provider. The first `nullptr` becomes
///     the verdict, carrying the provider's own `lastRefusal()` so the port that is empty or dead
///     is named by the layer that actually looked.
///
/// It is not `const` on the provider because obtaining a device may construct one; it is
/// idempotent, so calling it and then constructing a `Robot` asks for the same objects twice and
/// gets the same addresses both times.
[[nodiscard]] inline BuildVerdict validate(const config::RobotConfig& cfg,
                                           hal::IDeviceProvider& devices) {
    BuildVerdict verdict;
    if (!cfg.valid(verdict.why, sizeof verdict.why)) {
        return verdict;
    }
    const int wheels = cfg.drivetrain.wheelCount();
    const int motors = cfg.drivetrain.driveMotorCount();
    if (wheels != motors) {
        std::snprintf(verdict.why, sizeof verdict.why,
                      "%s drivetrain has %d kinematic wheels but %d motor ports -- a wheel whose "
                      "port is 0 has nothing to command (check drivetrain.ports)",
                      config::drivetrainKindWord(cfg.drivetrain.kind), wheels, motors);
        return verdict;
    }
    const hal::MotorCartridge cartridge = cfg.drivetrain.cartridge();
    const auto askMotor = [&](std::int8_t signedPort, const char* role) noexcept -> bool {
        if (devices.motor(signedPort, cartridge) != nullptr) {
            return true;
        }
        std::snprintf(verdict.why, sizeof verdict.why, "%s cannot be made: %s", role,
                      devices.lastRefusal());
        return false;
    };
    switch (cfg.drivetrain.kind) {
        case config::DrivetrainKind::Tank: {
            const hal::DrivetrainDescription& d = cfg.drivetrain.tank.description;
            for (std::size_t i = 0; i < d.left.count; ++i) {
                if (!askMotor(d.left.signedPorts[i], "a LEFT drive motor")) {
                    return verdict;
                }
            }
            for (std::size_t i = 0; i < d.right.count; ++i) {
                if (!askMotor(d.right.signedPorts[i], "a RIGHT drive motor")) {
                    return verdict;
                }
            }
            break;
        }
        case config::DrivetrainKind::XDrive:
            for (const std::int8_t p : cfg.drivetrain.xdrive.ports) {
                if (!askMotor(p, "an X-drive wheel motor")) {
                    return verdict;
                }
            }
            break;
        case config::DrivetrainKind::HDrive:
            for (const std::int8_t p : cfg.drivetrain.hdrive.ports) {
                if (!askMotor(p, "an H-drive wheel motor")) {
                    return verdict;
                }
            }
            break;
    }
    if (devices.imu(cfg.drivetrain.imuPort()) == nullptr) {
        std::snprintf(verdict.why, sizeof verdict.why, "the IMU cannot be made: %s",
                      devices.lastRefusal());
        return verdict;
    }
    if (cfg.sensors.gps.present
        && devices.gps(cfg.sensors.gps.port, cfg.sensors.gps.leverArmForward,
                       cfg.sensors.gps.leverArmLeft)
               == nullptr) {
        std::snprintf(verdict.why, sizeof verdict.why, "the GPS cannot be made: %s",
                      devices.lastRefusal());
        return verdict;
    }
    if (cfg.odometry.kind == config::OdometryKind::TrackingWheels) {
        if (devices.rotation(cfg.odometry.wheels.forward.port) == nullptr) {
            std::snprintf(verdict.why, sizeof verdict.why,
                          "the forward tracking wheel cannot be made: %s", devices.lastRefusal());
            return verdict;
        }
        if (devices.rotation(cfg.odometry.wheels.lateral.port) == nullptr) {
            std::snprintf(verdict.why, sizeof verdict.why,
                          "the lateral tracking wheel cannot be made: %s", devices.lastRefusal());
            return verdict;
        }
    }
    verdict.ok = true;
    return verdict;
}

/// A whole robot: the PROS-free object graph a `config::RobotConfig` describes, owned, with its
/// devices borrowed from an `hal::IDeviceProvider`.
///
/// **The provider must outlive the Robot**, and so must everything the provider borrows (on the
/// host: the harness whose clock, battery, sink and pacer it was given). The Robot owns the
/// graph; it owns no hardware, by ruling B2.
///
/// Neither copyable nor movable, for the same structural reason `chassis::Chassis` is not: the
/// graph is a web of references into this object's own storage, so a copy or a move would leave
/// half of it pointing at the wrong place. Construct it where it will live.
class Robot {
public:
    /// Build the whole graph. PRECONDITION: `validate(cfg, devices)` passes — and this runs it,
    /// so a config that cannot be built throws `PreconditionError` carrying the verdict's own
    /// sentence, before any member exists. `cfg` is COPIED (a boot sequence that decodes a
    /// record into a local and hands out a reference to it is the dangling-reference bug this
    /// copy makes impossible); `devices` is borrowed and must outlive the Robot.
    Robot(const config::RobotConfig& cfg, hal::IDeviceProvider& devices)
        : cfg_{prechecked(cfg, devices)},
          tankKin_{makeTankKinematics(cfg_)},
          matrixKin_{makeMatrixKinematics(cfg_)},
          kin_{tankKin_.has_value() ? static_cast<const kinematics::IKinematics*>(&*tankKin_)
                                    : static_cast<const kinematics::IKinematics*>(&*matrixKin_)},
          wiring_{wireMotors(cfg_, devices)},
          left_{makeSideGroup(cfg_, wiring_.leftMembers, wiring_.leftCount)},
          right_{makeSideGroup(cfg_, wiring_.rightMembers, wiring_.rightCount)},
          driveMotors_{makeDriveMotors(wiring_, left_, right_)},
          driveMotorCount_{static_cast<std::size_t>(cfg_.drivetrain.driveMotorCount())},
          groups_{makeGroupArray(left_, right_)},
          groupCount_{left_.has_value() ? std::size_t{2} : std::size_t{0}},
          imu_{devices.imu(cfg_.drivetrain.imuPort())},
          absentGps_{},
          gps_{cfg_.sensors.gps.present ? devices.gps(cfg_.sensors.gps.port,
                                                      cfg_.sensors.gps.leverArmForward,
                                                      cfg_.sensors.gps.leverArmLeft)
                                        : static_cast<hal::IGps*>(&absentGps_)},
          tags_{},
          vision_{},
          faults_{devices.telemetry(), devices.clock()},
          health_{faults_},
          leftGeometry_{sideGeometry(cfg_, /*right=*/false)},
          rightGeometry_{sideGeometry(cfg_, /*right=*/true)},
          driveOdom_{makeDriveEncoderOdometry(cfg_, left_, right_, *imu_, leftGeometry_,
                                              rightGeometry_)},
          pilonsOdom_{makePilonsOdometry(cfg_, devices, *imu_)},
          odom_{driveOdom_.has_value()
                    ? static_cast<localization::IOdometry*>(&*driveOdom_)
                    : static_cast<localization::IOdometry*>(&*pilonsOdom_)},
          fusion_{},
          gpsCorrector_{makeGpsCorrector(cfg_, devices, *gps_, *imu_)},
          correctors_{gpsCorrector_.has_value()
                          ? static_cast<localization::ICorrector*>(&*gpsCorrector_)
                          : nullptr},
          correctorCount_{gpsCorrector_.has_value() ? std::size_t{1} : std::size_t{0}},
          localizer_{devices.clock(), *imu_, *odom_, fusion_,
                     std::span<localization::ICorrector* const>{correctors_.data(),
                                                                correctorCount_}},
          ctx_{chassis::RobotContextConfig{
              .clock = &devices.clock(),
              .driveMotors = std::span<hal::IMotor* const>{driveMotors_.data(), driveMotorCount_},
              .imu = imu_,
              .gps = gps_,
              .battery = &devices.battery(),
              .telemetry = &devices.telemetry(),
              .tags = &tags_,
              .vision = &vision_}},
          deps_{.ctx = &ctx_,
                .localizer = &localizer_,
                .kinematics = kin_,
                .faults = &faults_,
                .health = &health_,
                .motorGroups =
                    std::span<hal::MotorGroup* const>{groups_.data(), groupCount_}},
          chassis_{deps_, devices.pacer(), effectiveTuning(cfg_)} {}

    /// Non-copyable: the graph is a web of references into this object's own storage.
    Robot(const Robot&) = delete;
    /// Non-movable, same reasoning — and `chassis::Chassis` is non-movable in any case.
    Robot(Robot&&) = delete;
    /// Non-copy-assignable, same reasoning.
    Robot& operator=(const Robot&) = delete;
    /// Non-move-assignable, same reasoning.
    Robot& operator=(Robot&&) = delete;
    /// Destroys the graph. The provider and everything it borrows must still be alive.
    ~Robot() = default;

    /// The public facade every auton is written against — the whole point of the class.
    [[nodiscard]] chassis::Chassis& chassis() noexcept { return chassis_; }
    /// The facade, read-only.
    [[nodiscard]] const chassis::Chassis& chassis() const noexcept { return chassis_; }

    /// The fused estimator, for a caller that wants the quality class or a corrector's record
    /// rather than just the pose.
    [[nodiscard]] localization::Localizer& localizer() noexcept { return localizer_; }
    /// The estimator, read-only.
    [[nodiscard]] const localization::Localizer& localizer() const noexcept { return localizer_; }

    /// The run-scoped fault latch — what the run summary and the boot banner read.
    [[nodiscard]] diag::FaultLatch& faults() noexcept { return faults_; }
    /// The fault latch, read-only.
    [[nodiscard]] const diag::FaultLatch& faults() const noexcept { return faults_; }

    /// The health monitor (the A3 pathology → fault policy).
    [[nodiscard]] diag::HealthMonitor& health() noexcept { return health_; }
    /// The health monitor, read-only.
    [[nodiscard]] const diag::HealthMonitor& health() const noexcept { return health_; }

    /// The one object every layer reads hardware through — the Tier-3 seam for code that wants a
    /// device this facade does not expose.
    [[nodiscard]] chassis::RobotContext& context() noexcept { return ctx_; }
    /// The context, read-only.
    [[nodiscard]] const chassis::RobotContext& context() const noexcept { return ctx_; }

    /// The config this robot was built from, as stored.
    [[nodiscard]] const config::RobotConfig& config() const noexcept { return cfg_; }

    /// Whether this robot has a GPS corrector installed — the observable half of "absent is
    /// explicit": false means `hal::AbsentGps` is in the context and the Localizer's corrector
    /// span is EMPTY, not that a corrector is present and quiet.
    [[nodiscard]] bool hasGpsCorrector() const noexcept { return gpsCorrector_.has_value(); }

    /// How many `hal::MotorGroup`s this robot has — 2 on a tank (one per side), 0 on a holonomic
    /// drivetrain, whose wheels are single motors.
    [[nodiscard]] std::size_t motorGroupCount() const noexcept { return groupCount_; }

    /// The BANNER LINE: identity, the drivetrain kind and where its numbers came from, both
    /// sides' ports AS STORED (signs included — the one place a sign lives), the geometry that
    /// governs, the estimator, and each optional device as present or ABSENT. Truncates rather
    /// than overflowing; writes at least a NUL whenever cap > 0.
    ///
    /// It is built from the CONFIG this robot was constructed with, not from a second copy of the
    /// numbers, so it cannot drift from the wiring — the same property src/main.cpp's
    /// `portMapString()` has and for the same reason.
    void describe(char* buf, std::size_t cap) const {
        if (buf == nullptr || cap == 0) {
            return;
        }
        constexpr std::size_t kSideCap = 72;
        char left[kSideCap] = "";
        char right[kSideCap] = "";
        const char* name = cfg_.identity.name[0] != '\0' ? cfg_.identity.name : "(unnamed)";
        switch (cfg_.drivetrain.kind) {
            case config::DrivetrainKind::Tank: {
                const hal::DrivetrainDescription& d = cfg_.drivetrain.tank.description;
                appendPorts(left, kSideCap, d.left.signedPorts, d.left.count);
                appendPorts(right, kSideCap, d.right.signedPorts, d.right.count);
                std::snprintf(buf, cap,
                              "%s [%s] tank %s | L %s R %s | ipr L%.5f R%.5f W %.3f in | IMU%u | "
                              "%s | GPS %s tags ABSENT vision ABSENT",
                              name, cfg_.identity.team,
                              d.source == hal::DescriptionSource::RobotMeasured ? "MEASURED"
                                                                                : "TYPED",
                              left, right, leftGeometry_.inchesPerRadian().value(),
                              rightGeometry_.inchesPerRadian().value(), d.trackWidth.value(),
                              static_cast<unsigned>(d.imuPort),
                              config::odometryKindWord(cfg_.odometry.kind),
                              gpsWord());
                return;
            }
            case config::DrivetrainKind::XDrive: {
                const config::XDriveDrivetrain& x = cfg_.drivetrain.xdrive;
                appendPorts(left, kSideCap, x.ports, 4);
                std::snprintf(buf, cap,
                              "%s [%s] x-drive TYPED | wheels %s | R %.3f in | IMU%u | %s | "
                              "GPS %s tags ABSENT vision ABSENT",
                              name, cfg_.identity.team, left, x.driveRadius.value(),
                              static_cast<unsigned>(x.imuPort),
                              config::odometryKindWord(cfg_.odometry.kind), gpsWord());
                return;
            }
            case config::DrivetrainKind::HDrive: {
                const config::HDriveDrivetrain& hcfg = cfg_.drivetrain.hdrive;
                appendPorts(left, kSideCap, hcfg.ports, 3);
                std::snprintf(buf, cap,
                              "%s [%s] h-drive TYPED | wheels %s | W %.3f in strafe@%.3f in | "
                              "IMU%u | %s | GPS %s tags ABSENT vision ABSENT",
                              name, cfg_.identity.team, left, hcfg.trackWidth.value(),
                              hcfg.strafeWheelOffset.value(), static_cast<unsigned>(hcfg.imuPort),
                              config::odometryKindWord(cfg_.odometry.kind), gpsWord());
                return;
            }
        }
        std::snprintf(buf, cap, "%s [%s] UNSET drivetrain", name, cfg_.identity.team);
    }

private:
    /// The borrowed `hal::IMotor` handles, gathered from the provider BY PORT before any group
    /// exists. One array per tank side (a side is N physical motors behind one group); one flat
    /// array for a holonomic drivetrain, whose wheels are single motors in the F5 wheel order.
    struct MotorWiring {
        /// The LEFT side's members, in the description's stored order. Tank only.
        hal::IMotor* leftMembers[hal::kMaxSmartPorts]{};
        /// The RIGHT side's members. Tank only.
        hal::IMotor* rightMembers[hal::kMaxSmartPorts]{};
        /// How many entries of `leftMembers` are real.
        std::size_t leftCount = 0;
        /// How many entries of `rightMembers` are real.
        std::size_t rightCount = 0;
        /// The holonomic wheels' motors, in the F5 wheel order. Holonomic only.
        hal::IMotor* wheels[static_cast<std::size_t>(kinematics::WheelSpeeds::kMaxWheels)]{};
        /// How many entries of `wheels` are real.
        std::size_t wheelCount = 0;
    };

    /// Run the verdict and throw its sentence if it fails, then hand back `cfg` for the member
    /// initializer. Being the FIRST member's initializer is what makes the refusal happen before
    /// any object exists (ruling B3).
    [[nodiscard]] static const config::RobotConfig& prechecked(const config::RobotConfig& cfg,
                                                               hal::IDeviceProvider& devices) {
        const BuildVerdict verdict = validate(cfg, devices);
        SHULIB_PRECONDITION(verdict.ok, verdict.why);
        return cfg;
    }

    /// The tank kinematics, or nothing on a holonomic drivetrain. The track width comes from the
    /// description as ONE number and is never retyped — MEASURED from a powered spin (effectively
    /// larger than a tape's, HA-135) or typed, and either way it arrives here already decided.
    [[nodiscard]] static std::optional<kinematics::TankKinematics> makeTankKinematics(
        const config::RobotConfig& cfg) {
        if (cfg.drivetrain.kind != config::DrivetrainKind::Tank) {
            return std::nullopt;
        }
        return std::optional<kinematics::TankKinematics>{
            std::in_place, cfg.drivetrain.tank.description.trackWidth};
    }

    /// The holonomic kinematics (x_drive.hpp's or h_drive.hpp's coefficient table), or nothing on
    /// a tank. Value-constructed from config DATA, exactly as chassis.hpp's standalone recipe
    /// does it by hand.
    [[nodiscard]] static std::optional<kinematics::MatrixKinematics> makeMatrixKinematics(
        const config::RobotConfig& cfg) {
        switch (cfg.drivetrain.kind) {
            case config::DrivetrainKind::XDrive:
                return std::optional<kinematics::MatrixKinematics>{
                    kinematics::xDrive(cfg.drivetrain.xdrive.driveRadius)};
            case config::DrivetrainKind::HDrive:
                return std::optional<kinematics::MatrixKinematics>{kinematics::hDrive(
                    {.trackWidth = cfg.drivetrain.hdrive.trackWidth,
                     .strafeWheelOffset = cfg.drivetrain.hdrive.strafeWheelOffset})};
            case config::DrivetrainKind::Tank:
                break;
        }
        return std::nullopt;
    }

    /// Ask the provider for every drive motor the config names, handing each signed port over AS
    /// STORED so the sign is applied exactly once, inside the adapter. A null here would be a
    /// provider that refused after `validate()` said it would not — a contract breach, and a loud
    /// one, because the alternative is a nullptr inside a MotorGroup.
    [[nodiscard]] static MotorWiring wireMotors(const config::RobotConfig& cfg,
                                                hal::IDeviceProvider& devices) {
        MotorWiring w;
        const hal::MotorCartridge cartridge = cfg.drivetrain.cartridge();
        const auto take = [&](std::int8_t signedPort) -> hal::IMotor* {
            hal::IMotor* m = devices.motor(signedPort, cartridge);
            SHULIB_PRECONDITION(m != nullptr,
                                "robot::Robot: the provider refused a drive motor that "
                                "validate() had already obtained");
            return m;
        };
        switch (cfg.drivetrain.kind) {
            case config::DrivetrainKind::Tank: {
                const hal::DrivetrainDescription& d = cfg.drivetrain.tank.description;
                for (std::size_t i = 0; i < d.left.count; ++i) {
                    w.leftMembers[w.leftCount++] = take(d.left.signedPorts[i]);
                }
                for (std::size_t i = 0; i < d.right.count; ++i) {
                    w.rightMembers[w.rightCount++] = take(d.right.signedPorts[i]);
                }
                break;
            }
            case config::DrivetrainKind::XDrive:
                for (const std::int8_t p : cfg.drivetrain.xdrive.ports) {
                    w.wheels[w.wheelCount++] = take(p);
                }
                break;
            case config::DrivetrainKind::HDrive:
                for (const std::int8_t p : cfg.drivetrain.hdrive.ports) {
                    w.wheels[w.wheelCount++] = take(p);
                }
                break;
        }
        return w;
    }

    /// ONE `hal::MotorGroup` over a tank side's members — the object that IS the motor for that
    /// kinematic wheel (R3b Part 1: N physical motors on one side go behind one group, or the
    /// surplus is silently uncommanded). Nothing on a holonomic drivetrain, whose wheels are
    /// single motors.
    [[nodiscard]] static std::optional<hal::MotorGroup> makeSideGroup(
        const config::RobotConfig& cfg, hal::IMotor* const* members, std::size_t count) {
        if (cfg.drivetrain.kind != config::DrivetrainKind::Tank) {
            return std::nullopt;
        }
        return std::optional<hal::MotorGroup>{
            std::in_place, std::span<hal::IMotor* const>{members, count}};
    }

    /// The context's drive-motor array, in KINEMATIC WHEEL ORDER: the two groups on a tank
    /// (0 = left, 1 = right, as `kinematics::TankKinematics` numbers them), or the holonomic
    /// wheels straight through in the F5 order.
    [[nodiscard]] static std::array<hal::IMotor*,
                                    static_cast<std::size_t>(kinematics::WheelSpeeds::kMaxWheels)>
    makeDriveMotors(const MotorWiring& wiring, std::optional<hal::MotorGroup>& left,
                    std::optional<hal::MotorGroup>& right) {
        std::array<hal::IMotor*, static_cast<std::size_t>(kinematics::WheelSpeeds::kMaxWheels)>
            out{};
        if (left.has_value()) {
            out[0] = &*left;
            out[1] = &*right;
            return out;
        }
        for (std::size_t i = 0; i < wiring.wheelCount && i < out.size(); ++i) {
            out[i] = wiring.wheels[i];
        }
        return out;
    }

    /// The groups the health tick evaluates each loop (`motion::MotionDeps::motorGroups`), or an
    /// array of nulls with count 0 on a holonomic drivetrain.
    [[nodiscard]] static std::array<hal::MotorGroup*, 2> makeGroupArray(
        std::optional<hal::MotorGroup>& left, std::optional<hal::MotorGroup>& right) {
        if (!left.has_value()) {
            return {nullptr, nullptr};
        }
        return {&*left, &*right};
    }

    /// One side's drive geometry, straight from the description — a separate object per side,
    /// because a calibration measures each side separately and an asymmetric drivetrain is
    /// representable. A holonomic drivetrain has no per-side wheel scale in its config (its
    /// odometry is the tracking wheels), so it gets the library default, which nothing reads.
    [[nodiscard]] static hal::DriveGeometry sideGeometry(const config::RobotConfig& cfg,
                                                          bool right) {
        if (cfg.drivetrain.kind != config::DrivetrainKind::Tank) {
            return hal::DriveGeometry{};
        }
        const hal::DrivetrainDescription& d = cfg.drivetrain.tank.description;
        return right ? d.rightGeometry() : d.leftGeometry();
    }

    /// Drive-encoder odometry over the two side GROUPS (R3b Part 2) — or nothing when this robot
    /// runs tracking wheels. The groups are the IMotors it reads, so a frozen member is already
    /// excluded by the group's median before the estimator ever sees a number.
    [[nodiscard]] static std::optional<localization::DriveEncoderOdometry>
    makeDriveEncoderOdometry(const config::RobotConfig& cfg, std::optional<hal::MotorGroup>& left,
                             std::optional<hal::MotorGroup>& right, hal::IImu& imu,
                             hal::DriveGeometry leftGeometry, hal::DriveGeometry rightGeometry) {
        if (cfg.odometry.kind != config::OdometryKind::DriveEncoders) {
            return std::nullopt;
        }
        return std::optional<localization::DriveEncoderOdometry>{
            std::in_place,  imu,          *left,
            *right,         leftGeometry, rightGeometry,
            cfg.drivetrain.tank.description.trackWidth};
    }

    /// Two-pod (Pilons) odometry — or nothing when this robot runs drive encoders. The ROLE of
    /// each wheel is fixed by which factory builds it, so a pair that would have been handed over
    /// in the wrong order cannot be expressed here at all.
    [[nodiscard]] static std::optional<localization::PilonsOdometry> makePilonsOdometry(
        const config::RobotConfig& cfg, hal::IDeviceProvider& devices, hal::IImu& imu) {
        if (cfg.odometry.kind != config::OdometryKind::TrackingWheels) {
            return std::nullopt;
        }
        const config::TrackingWheelConfig& f = cfg.odometry.wheels.forward;
        const config::TrackingWheelConfig& l = cfg.odometry.wheels.lateral;
        hal::IRotation* forwardSensor = devices.rotation(f.port);
        hal::IRotation* lateralSensor = devices.rotation(l.port);
        SHULIB_PRECONDITION(forwardSensor != nullptr && lateralSensor != nullptr,
                            "robot::Robot: the provider refused a tracking-wheel encoder that "
                            "validate() had already obtained");
        return std::optional<localization::PilonsOdometry>{
            std::in_place, imu,
            localization::TrackingWheel::forward(*forwardSensor, f.wheelDiameter, f.offset),
            localization::TrackingWheel::lateral(*lateralSensor, l.wheelDiameter, l.offset)};
    }

    /// The GPS corrector — installed if and only if the config says a GPS is present. When it is
    /// absent there is NO corrector and the Localizer's corrector span is empty, which is a
    /// different thing from a corrector that never proposes: a quiet corrector still shows in the
    /// records as a live source, and a robot that has no GPS must not look like one that has a
    /// silent one (landmine 5).
    [[nodiscard]] static std::optional<localization::GpsCorrector> makeGpsCorrector(
        const config::RobotConfig& cfg, hal::IDeviceProvider& devices, hal::IGps& gps,
        hal::IImu& imu) {
        if (!cfg.sensors.gps.present) {
            return std::nullopt;
        }
        return std::optional<localization::GpsCorrector>{std::in_place, devices.clock(), gps, imu};
    }

    /// The ChassisConfig the facade is built with: the caller's tuning, plus the one thing the
    /// builder knows and a person should not have to retype — the stall check's per-wheel drive
    /// geometry, and the fact that a drivetrain whose odometry IS its drive encoders has no
    /// independent motion source to cross-check against (odo_stall_check.hpp's §2 ruling).
    /// Exactly what `src/main.cpp`'s `tankChassisConfig()` does by hand, and applied only where
    /// that function applies it: a holonomic drivetrain's config carries no per-wheel scale, so
    /// its tuning is passed through untouched — which is also what main.cpp's X-drive graph does.
    [[nodiscard]] static config::TuningConfig effectiveTuning(const config::RobotConfig& cfg) {
        config::TuningConfig tuning = cfg.tuning;
        if (cfg.drivetrain.kind == config::DrivetrainKind::Tank) {
            tuning.motion.stall.setAllWheels(cfg.drivetrain.tank.description.leftGeometry());
            tuning.motion.stall.independentMotionSource =
                cfg.odometry.kind != config::OdometryKind::DriveEncoders;
        }
        return tuning;
    }

    /// "GPS present" or "ABSENT", for the banner.
    [[nodiscard]] const char* gpsWord() const noexcept {
        return cfg_.sensors.gps.present ? "present" : "ABSENT";
    }

    /// Write `count` signed ports into `out` as "1,-2,3", bounded by `cap` and always
    /// NUL-terminated. Appended BY HAND rather than with a moving snprintf for the same reason
    /// src/main.cpp's `tankPortMapString()` does it: the compiler cannot bound a
    /// `cap - strlen(dst)` argument, so -Wformat-truncation fires on a call that is in fact safe,
    /// and the on-robot build gate treats that warning as an error.
    static void appendPorts(char* out, std::size_t cap, const std::int8_t* ports,
                            std::size_t count) {
        std::size_t n = 0;
        for (std::size_t i = 0; i < count; ++i) {
            char one[8] = "";
            std::snprintf(one, sizeof one, "%s%d", i == 0 ? "" : ",",
                          static_cast<int>(ports[i]));
            for (std::size_t k = 0; one[k] != '\0' && n + 1 < cap; ++k) {
                out[n++] = one[k];
            }
        }
        if (cap > 0) {
            out[n < cap ? n : cap - 1] = '\0';
        }
    }

    // ── the graph, in CONSTRUCTION ORDER (declaration order IS initialization order) ────
    //    kinematics → motors → groups → sensors → diagnostics → geometry → odometry →
    //    fusion → correctors → localizer → context → deps → chassis. The same list, in the
    //    same order, as src/main.cpp's TankRobot and X-drive Robot.

    /// The config this robot was built from, COPIED (see the constructor).
    ///
    /// The PROVIDER is deliberately NOT stored beside it. Every device this graph needs was taken
    /// during construction and is held where it is used; a second handle to the provider would be
    /// an unused member that reads like an ownership claim, and ruling B2 is that the builder owns
    /// no hardware. The provider must still outlive the Robot — the class banner and the
    /// constructor both say so — but that is a lifetime contract, not a field.
    config::RobotConfig cfg_;

    std::optional<kinematics::TankKinematics> tankKin_;
    std::optional<kinematics::MatrixKinematics> matrixKin_;
    const kinematics::IKinematics* kin_;

    MotorWiring wiring_;
    std::optional<hal::MotorGroup> left_;
    std::optional<hal::MotorGroup> right_;
    std::array<hal::IMotor*, static_cast<std::size_t>(kinematics::WheelSpeeds::kMaxWheels)>
        driveMotors_;
    std::size_t driveMotorCount_;
    std::array<hal::MotorGroup*, 2> groups_;
    std::size_t groupCount_;

    hal::IImu* imu_;
    hal::AbsentGps absentGps_;
    hal::IGps* gps_;
    hal::AbsentTagSource tags_;
    hal::AbsentVision vision_;

    diag::FaultLatch faults_;
    diag::HealthMonitor health_;

    hal::DriveGeometry leftGeometry_;
    hal::DriveGeometry rightGeometry_;

    std::optional<localization::DriveEncoderOdometry> driveOdom_;
    std::optional<localization::PilonsOdometry> pilonsOdom_;
    localization::IOdometry* odom_;
    localization::ComplementaryFusion fusion_;

    std::optional<localization::GpsCorrector> gpsCorrector_;
    std::array<localization::ICorrector*, 1> correctors_;
    std::size_t correctorCount_;

    localization::Localizer localizer_;
    chassis::RobotContext ctx_;
    motion::MotionDeps deps_;
    chassis::Chassis chassis_;
};

}  // namespace shulib::robot
