// main.cpp — the PROS entry point, rewired onto the shulib v2 core (chunk C7,
// WS11/M2), with the hal/pros adapters wired in at chunk R1a.
//
// ═══ READ THIS FIRST: what this binary is, and what it is NOT ═══════════════════════
// Since R1a every HAL seam below is a REAL hal/pros adapter over a physical V5
// device: motors, IMU, GPS, battery, rotation sensors, controller, USB serial,
// controller LCD, real time, and the tick-boundary pacer. The fakes are gone
// from this file (vision/tags excepted — R2's).
//
// WHAT THAT DOES **NOT** MEAN — the governing constraint, unchanged:
//   * THE LIBRARY HAS STILL NEVER DRIVEN A ROBOT. The adapters are host-tested
//     against a hand-written shim of PROS — which tests the adapter against
//     our BELIEF about PROS, never the belief itself. The beliefs are
//     inventoried in docs/hardware-assumptions.md (HA-94 onward) and settle on
//     a bench, not in a test suite.
//   * The PORT MAP BELOW IS INVENTED (A4: HA-111). No robot has been measured.
//     Uploading this to a brain whose devices are not on exactly these ports
//     will fault loudly at boot (the adapters read their configuration back).
//   * autonomous() still commands NO motion: R3 owns hardware validation, and
//     an unvalidated stack driving blind on a field is worse than one that
//     says so over serial.
//   * M2's on-robot clause — validated on a V5 — is OPEN, owned by chunk R3.
//
// ═══ The PROS boundary ══════════════════════════════════════════════════════════════
// This TU (and main.h) may include <pros/...>; include/shulib/ may NOT —
// except include/shulib/hal/pros/ (the adapter tree, R1a), which is the ONE
// path the CI guard exempts, by exact path anchor. Everything else in the
// library remains PROS-free, and CI enforces it.
//
// ═══ What R1a resolved here (this file carried fourteen R1 to-do markers) ═══════════
//   * hal/pros adapters for clock/motors/imu/gps/battery/rotation — each
//     applying its canonical conversion at the seam (imu_conversion.hpp,
//     gps_conversion.hpp, motor_conversion.hpp, rotation_conversion.hpp).
//   * ProsTickPacer over pros::Task::delay_until — replaces V5DelayPacer,
//     whose whole job was advancing a fake clock that no longer exists.
//   * The on-robot precondition policy — installed FIRST in initialize(),
//     before the object graph constructs and before any other task exists.
//   * Session-header emission — the PROS Makefile now injects the git build
//     hash (same contract as test/CMakeLists.txt); a missing hash is LOUD.
//   * The teleop loop — chassis.drive(speeds, Frame::Body) from the master
//     controller at the tick cadence. Deadband/curves/slew are chunk T2's
//     (HA-112 records the raw mapping as invented).
//
// ═══ Still fake, deliberately ═══════════════════════════════════════════════════════
//   * FakeTagSource / FakeVision — the M3 vision pipeline is R2's (camera).

#include "main.h"

#include <cstdio>
#include <cstdint>

#include "shulib/chassis/chassis.hpp"
#include "shulib/chassis/robot_context.hpp"
#include "shulib/core/check.hpp"
#include "shulib/diag/build_info.hpp"
#include "shulib/diag/controller_display.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/diag/health_monitor.hpp"
#include "shulib/diag/session_info.hpp"
#include "shulib/diag/term_sink.hpp"
#include "shulib/hal/controller.hpp"
#include "shulib/hal/fake/fake_tag_source.hpp"
#include "shulib/hal/fake/fake_vision.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/hal/pros/battery.hpp"
#include "shulib/hal/pros/char_sink.hpp"
#include "shulib/hal/pros/clock.hpp"
#include "shulib/hal/pros/controller.hpp"
#include "shulib/hal/pros/gps.hpp"
#include "shulib/hal/pros/imu.hpp"
#include "shulib/hal/pros/line_display.hpp"
#include "shulib/hal/pros/motor.hpp"
#include "shulib/hal/pros/rotation.hpp"
#include "shulib/hal/pros/tick_pacer.hpp"
#include "shulib/hal/telemetry_sink.hpp"
#include "shulib/kinematics/matrix_kinematics.hpp"
#include "shulib/kinematics/x_drive.hpp"
#include "shulib/localization/complementary_fusion.hpp"
#include "shulib/localization/localizer.hpp"
#include "shulib/localization/pilons_odometry.hpp"
#include "shulib/localization/tracking_wheel.hpp"
#include "shulib/math/frame.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/motion/motion.hpp"
#include "shulib/motion/motion_scheduler.hpp"
#include "shulib/units/quantity.hpp"

namespace {

// ── PROVISIONAL PORT MAP — INVENTED (A4: HA-111). No robot has been measured;
//    bench runbook step 0 reads the real device list off the brain and fixes
//    these. Motor SIGNS (which side reverses) are part of the same guess and
//    are settled by the open-loop spin check (runbook step 2) BEFORE any
//    closed-loop run. Wheel order is x_drive.hpp's canonical (FL, BL, BR, FR)
//    — the kinematics rows and the motor array MUST agree or every motion is
//    wrong.
constexpr std::int8_t kFrontLeftPort = 1;
constexpr std::int8_t kBackLeftPort = 2;
constexpr std::int8_t kBackRightPort = -3;   // sign INVENTED with the rest (HA-111)
constexpr std::int8_t kFrontRightPort = -4;  // sign INVENTED with the rest (HA-111)
constexpr std::int8_t kForwardEncoderPort = 5;
constexpr std::int8_t kLateralEncoderPort = 6;
constexpr std::uint8_t kGpsPort = 9;
constexpr std::uint8_t kImuPort = 10;

// The drive cartridge: GREEN is HA-15's INVENTED stand-in until the build
// team's cartridges are read off the physical motors (runbook step 0).
constexpr shulib::hal::pros::MotorGearset kDriveGearset =
    shulib::hal::pros::MotorGearset::Green;

// Teleop mapping constants — INVENTED (A4: HA-112); chunk T2 owns the real
// driver-feel layer (curves, slew, per-driver tuning). The deadband exists
// only so a centred stick's ±2-count noise cannot creep the robot.
constexpr double kTeleopDeadband = 0.05;

/// The on-robot precondition policy (check.hpp §18.4): raise the fault code on
/// the latch (visible in telemetry + the run summary), then throw the same
/// PreconditionError the host policy throws — the motion scheduler catches it
/// at the task boundary and converts it to FAULT_ABORT + a safe drivetrain
/// state. During Robot construction the latch does not exist yet, so the
/// handler degrades to plain-throw — still loud, still never-returns.
/// (What an UNCAUGHT throw out of initialize() does on the brain is exactly
/// bench runbook step 1's measurement — brief T8: demonstrate, don't assert.)
shulib::diag::FaultLatch* g_faults = nullptr;
[[noreturn]] void robotPreconditionHandler(const char* message) {
    if (g_faults != nullptr) {
        g_faults->raise(shulib::diag::FaultCode::Precondition, "CHK", message);
    }
    throw shulib::PreconditionError(message);
}

/// The whole robot, wired once. Member order IS initialization order — each
/// object is declared after everything it borrows, because the upper layers
/// borrow their dependencies rather than owning them (chassis.hpp
/// "Construction"). This is the §16.2 standalone promise executed for real:
/// plain C++, no config file, no builder, no codegen — now over real devices.
struct Robot {
    // ── drivetrain: config DATA, value-constructed. X-drive at the HA-17
    //    stand-in 7.0" drive radius — R3 measures the real machine's geometry.
    shulib::kinematics::MatrixKinematics kin = shulib::kinematics::xDrive(
        shulib::units::Length{7.0});

    // ── HAL: hal/pros adapters over real V5 devices (chunk R1a). Every port
    //    number is HA-111's invented stand-in; every unit conversion is
    //    applied exactly once, inside the adapter, per the binding contracts.
    shulib::hal::pros::ProsClock clock{};
    shulib::hal::pros::ProsMotor frontLeft{kFrontLeftPort, kDriveGearset},
        backLeft{kBackLeftPort, kDriveGearset}, backRight{kBackRightPort, kDriveGearset},
        frontRight{kFrontRightPort, kDriveGearset};
    //    Order is x_drive.hpp's canonical wheel order (FL, BL, BR, FR).
    shulib::hal::IMotor* driveMotors[4] = {&frontLeft, &backLeft, &backRight, &frontRight};
    //    bootHeading Angle{0}: ONE owner = the robot's canonical start pose
    //    (imu_conversion.hpp HA-05). R3 wires the real start pose when autons
    //    land; until then the robot boots believing it faces +X.
    shulib::hal::pros::ProsImu imu{kImuPort, shulib::math::Angle{}, clock};
    //    Lever arm (0,0) is HA-10's invented stand-in — tape-measure at R3.
    //    Construction is PORT-ONLY + the get_offset()==(0,0) boot check (HA-06).
    shulib::hal::pros::ProsGps gps{kGpsPort, shulib::units::Length{0.0},
                                   shulib::units::Length{0.0}};
    shulib::hal::pros::ProsBattery battery{};
    shulib::hal::pros::ProsRotation forwardEncoder{kForwardEncoderPort},
        lateralEncoder{kLateralEncoderPort};
    shulib::hal::pros::ProsController master{shulib::hal::pros::ControllerId::Master};
    //    (VEX U's partner controller is one more line when T2 wires the second
    //    driver: ProsController partner{ControllerId::Partner};)
    shulib::hal::fake::FakeTagSource tags{};   // stub until the M3 vision pipeline (R2 camera)
    shulib::hal::fake::FakeVision vision{};    // stub until the M3 vision pipeline (R2 camera)

    // ── diagnostics: REAL on-target — TermSink writes the V5 USB serial via
    //    the promoted ProsCharSink (formerly this file's private StdoutCharSink),
    //    and the D-4 fault display writes the controller LCD (3×19, HA-57 —
    //    with the HA-107 column-count conflict registered for the bench).
    shulib::hal::pros::ProsCharSink usb{};
    shulib::diag::TermSink telemetry{clock, usb};
    shulib::diag::FaultLatch faults{telemetry, clock};
    shulib::diag::HealthMonitor health{faults};
    shulib::hal::pros::ProsLineDisplay controllerScreen{};
    shulib::diag::ControllerFaultDisplay faultDisplay{controllerScreen, faults, battery};

    // ── localization: the M2 dead-reckon stack. Tracking-wheel geometry is the
    //    HA-12/HA-13 stand-in set (2.0" wheels, -3.0"/-4.5" offsets — the same
    //    numbers the host sim exercises); R3 replaces them with measurements.
    shulib::localization::PilonsOdometry odom{
        imu,
        shulib::localization::TrackingWheel::forward(forwardEncoder, shulib::units::Length{2.0},
                                                     shulib::units::Length{-3.0}),
        shulib::localization::TrackingWheel::lateral(lateralEncoder, shulib::units::Length{2.0},
                                                     shulib::units::Length{-4.5})};
    shulib::localization::ComplementaryFusion fusion{};
    shulib::localization::Localizer localizer{clock, imu, odom, fusion};

    // ── the one object every layer reads hardware through (master plan §5).
    shulib::chassis::RobotContext ctx{{.clock = &clock,
                                       .driveMotors = driveMotors,
                                       .imu = &imu,
                                       .gps = &gps,
                                       .battery = &battery,
                                       .telemetry = &telemetry,
                                       .tags = &tags,
                                       .vision = &vision}};

    // ── motion + the public facade. Gains/budgets are the MotionConfig
    //    defaults: ALL PROVISIONAL (HA-45/50/51/52) until R5 tunes on hardware.
    shulib::motion::MotionDeps deps{.ctx = &ctx,
                                    .localizer = &localizer,
                                    .kinematics = &kin,
                                    .faults = &faults,
                                    .health = &health};
    //    The real tick pacer (R1a): pros::Task::delay_until anchored to the
    //    10 ms boundary — the fake-clock advance V5DelayPacer needed is gone.
    shulib::hal::pros::ProsTickPacer pacer{};
    shulib::chassis::Chassis chassis{deps, pacer};
};

/// Constructed on first use (initialize()), alive for the whole run. A
/// function-local static sidesteps static-init-order hazards and makes the
/// construction point explicit in the boot flow.
Robot& robot() {
    static Robot r;
    return r;
}

/// The §18.5 port-map string, built from the SAME constants the adapters are
/// constructed with — it cannot drift from the wiring above.
const char* portMapString() {
    static char buf[shulib::diag::kMaxPortMapBytes];
    std::snprintf(buf, sizeof buf, "FL%d BL%d BR%d FR%d ROTF%d ROTL%d GPS%u IMU%u",
                  static_cast<int>(kFrontLeftPort), static_cast<int>(kBackLeftPort),
                  static_cast<int>(kBackRightPort), static_cast<int>(kFrontRightPort),
                  static_cast<int>(kForwardEncoderPort), static_cast<int>(kLateralEncoderPort),
                  static_cast<unsigned>(kGpsPort), static_cast<unsigned>(kImuPort));
    return buf;
}

/// Teleop stick shaping: deadband only (HA-112 — T2 owns real driver feel).
[[nodiscard]] double shaped(double axis) {
    return (axis > -kTeleopDeadband && axis < kTeleopDeadband) ? 0.0 : axis;
}

}  // namespace

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * Order is load-bearing (check.hpp's concurrency contract): the precondition
 * policy is installed FIRST — before the Robot constructs (its adapter ctors
 * carry read-back preconditions) and before any other task exists. Then the
 * object graph, the fault-latch hookup, IMU calibration, and the §18.5
 * session header with the build hash the Makefile injected.
 */
void initialize() {
    shulib::setPreconditionHandler(&robotPreconditionHandler);

    Robot& r = robot();
    g_faults = &r.faults;

    // Start IMU calibration now (non-blocking); readings are garbage until
    // isReady() (HA-23) and the motion layer's wait-for-live already gates on
    // it. A second calibrate() anywhere is a precondition violation (HA-05).
    r.imu.calibrate();

    // ── the §18.5 session header: provenance FIRST, missing hash LOUD.
    shulib::diag::SessionInfo session{};
    session.buildHash = shulib::diag::compiledBuildHash();
    session.routineId = "(none: autons land at R3)";
    session.alliance = "";  // renders "-": no field, no alliance yet
    session.side = "";
    session.portMap = portMapString();
    shulib::diag::emitSessionHeader(r.telemetry, session, r.battery.voltage());

    r.telemetry.log(shulib::hal::LogLevel::Info, "R1A",
                    "shulib v2 core wired over hal/pros adapters: X-drive kinematics + "
                    "Pilons odometry + fused localizer + motion scheduler + Chassis facade");
    r.telemetry.log(shulib::hal::LogLevel::Warn, "R1A",
                    "HARDWARE-UNVALIDATED: adapters are host-tested against a PROS shim "
                    "only; port map is invented (HA-111); R3 owns first motion");

    // One live query THROUGH the facade, so the banner is evidence of a working
    // object graph rather than prose: strafeAuthority() reads Chassis →
    // kinematics (F5) and must be 1.00 on this X-drive.
    char line[64];
    std::snprintf(line, sizeof line, "facade alive: strafeAuthority=%.2f (X-drive: 1.00)",
                  r.chassis.strafeAuthority());
    r.telemetry.log(shulib::hal::LogLevel::Info, "R1A", line);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code.
 *
 * Deliberately commands NO motion, still: the adapters exist (R1a) but the
 * stack is hardware-UNVALIDATED — no bench session has confirmed sign
 * conventions, the port map is invented, and every gain is provisional. An
 * unvalidated auton driving blind on a field is indistinguishable from a
 * runaway. The v2 autons exist and settle in host sim
 * (test/chassis_routine_test.cpp chains them through this same facade); R3
 * wires them here after the bench runbook settles the R3 register group.
 */
void autonomous() {
    Robot& r = robot();
    r.telemetry.log(shulib::hal::LogLevel::Warn, "R1A",
                    "autonomous(): no motion until R3 validates the stack on hardware "
                    "(adapters landed at R1a; the register's R3 group is unsettled)");
}

/**
 * Runs the operator control code.
 *
 * The R1a teleop loop: master-controller sticks → body-frame
 * chassis.drive() at the tick cadence. Mapping (HA-112, invented, T2 owns
 * refinement): left stick = translation (up = +X forward, LEFT-pushed stick =
 * +Y left), right stick X = yaw (right-pushed = clockwise = −ω). A
 * disconnected controller commands zero twist — isConnected() is the positive
 * validity signal that distinguishes a dropped controller from centred
 * sticks (HA-103).
 */
void opcontrol() {
    Robot& r = robot();
    r.telemetry.log(shulib::hal::LogLevel::Info, "R1A",
                    "opcontrol(): teleop drive loop live (deadband-only mapping, HA-112; "
                    "driver-feel shaping is chunk T2)");
    const shulib::motion::MotionConfig& cfg = r.chassis.motionConfig();
    while (true) {
        shulib::math::ChassisSpeeds command{};  // zero twist unless the driver says otherwise
        if (r.master.isConnected()) {
            const double fwd = shaped(r.master.axis(shulib::hal::ControllerAxis::LeftY));
            const double left = shaped(-r.master.axis(shulib::hal::ControllerAxis::LeftX));
            const double yaw = shaped(-r.master.axis(shulib::hal::ControllerAxis::RightX));
            command = shulib::math::ChassisSpeeds{fwd * cfg.maxLinearSpeed,
                                                  left * cfg.maxLinearSpeed,
                                                  yaw * cfg.maxAngularSpeed};
        }
        r.chassis.drive(command, shulib::math::Frame::Body);
        r.faultDisplay.update(r.clock.now());
        r.pacer.pace();
    }
}
