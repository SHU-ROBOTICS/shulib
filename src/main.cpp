// main.cpp — the PROS entry point, rewired onto the shulib v2 core (chunk C7, WS11/M2).
//
// ═══ READ THIS FIRST: what this binary is, and what it is NOT ═══════════════════════
// This file closes M2's STRUCTURAL clause: the legacy tree is deleted and the only
// stack main.cpp wires is the v2 core — kinematics → localization → motion →
// chassis facade → diagnostics. `make` builds it into an uploadable V5 hot/cold
// package.
//
// IT HAS NEVER RUN ON A ROBOT, AND IT CANNOT DRIVE ONE YET.
//   * The hal/pros adapters that would connect IMotor / IImu / IRotation / IGps /
//     IBattery to physical V5 devices are chunk R1's deliverable and DO NOT EXIST.
//     Every hardware seam below is a shipped in-memory fake standing in the
//     adapter's slot, marked TODO(R1) at the exact line an adapter replaces.
//   * Uploading this package gives you a brain that boots, prints the v2
//     diagnostics banner over USB serial (`pros terminal`), and moves nothing.
//   * M2's on-robot clause — "main.cpp runs entirely on the new core", validated
//     on a V5 — is OPEN, owned by chunk R3.
// If you came here to learn whether shulib works on a robot: it has not been
// tried. 659 host test cases / ~915,000 assertions say the LOGIC works off-robot
// (three drivetrains, closed-loop sim, simulated hardware faults); no claim about
// physical hardware is made anywhere in this tree. Every assumption a robot will
// have to settle is inventoried in docs/hardware-assumptions.md.
//
// ═══ Why wire fakes at all? ═════════════════════════════════════════════════════════
// Because the wiring itself is the deliverable: this file proves, on the real
// toolchain at the real -mcpu/-mfpu flags, that a COMPLETE v2 object graph
// constructs, links, and boots inside a PROS binary. When R1's adapters land,
// each TODO(R1) line swaps a fake for an adapter and NOTHING ELSE changes — that
// swap-only property is exactly what RobotContext exists to guarantee (master
// plan §5), and this file is its first on-target consumer. The wiring below is
// deliberately the SAME shape as the file-free construction test
// (test/chassis_facade_test.cpp "C4 standalone") — the documented plain-C++
// recipe, laid out on the brain.
//
// ═══ The PROS boundary ══════════════════════════════════════════════════════════════
// This TU (and main.h) may include <pros/...>; include/shulib/ may NOT — CI's
// PROS-free guard covers the entire library tree since C7. main.cpp is
// deliberately the only file in the project that sees both worlds.
//
// ═══ What R1 adds here (so the gap is visible in code, not just prose) ══════════════
//   TODO(R1): hal/pros adapters for clock/motors/imu/gps/battery/rotation, each
//             applying its canonical conversion (imu_conversion.hpp HA-02,
//             gps_conversion.hpp HA-01) at the seam.
//   TODO(R1): tick-boundary pacer (pros::Task::delay_until against the 10 ms
//             motion tick) replacing V5DelayPacer's fake-clock advance.
//   TODO(R1): install the on-robot precondition policy (raise the fault code on
//             the latch, then throw — check.hpp §18.4) in initialize(), before
//             any other task exists.
//   TODO(R1): session header emission (diag/session_info.hpp) once the PROS
//             Makefile injects the git build hash the way test/CMakeLists.txt
//             already does — C5 made a missing hash LOUD by design.

#include "main.h"

#include <cstdio>
#include <string_view>

#include "shulib/chassis/chassis.hpp"
#include "shulib/chassis/robot_context.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/diag/health_monitor.hpp"
#include "shulib/diag/term_sink.hpp"
#include "shulib/hal/char_sink.hpp"
#include "shulib/hal/fake/fake_battery.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_gps.hpp"
#include "shulib/hal/fake/fake_imu.hpp"
#include "shulib/hal/fake/fake_motor.hpp"
#include "shulib/hal/fake/fake_rotation.hpp"
#include "shulib/hal/fake/fake_tag_source.hpp"
#include "shulib/hal/fake/fake_vision.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/hal/telemetry_sink.hpp"
#include "shulib/kinematics/matrix_kinematics.hpp"
#include "shulib/kinematics/x_drive.hpp"
#include "shulib/localization/complementary_fusion.hpp"
#include "shulib/localization/localizer.hpp"
#include "shulib/localization/pilons_odometry.hpp"
#include "shulib/localization/tracking_wheel.hpp"
#include "shulib/motion/motion.hpp"
#include "shulib/motion/motion_scheduler.hpp"
#include "shulib/units/quantity.hpp"

namespace {

/// V5 USB serial through newlib stdout — the ONE real piece of hardware I/O in
/// this file (the PROS kernel wires stdout to the USB serial; `pros terminal`
/// displays it). Flush per write: boot-banner visibility is worth more than
/// buffered throughput here, and the diagnostics layer already rate-limits.
/// write() must not throw (ICharSink contract) — fwrite/fflush cannot.
struct StdoutCharSink final : shulib::hal::ICharSink {
    void write(std::string_view text) override {
        std::fwrite(text.data(), 1, text.size(), stdout);
        std::fflush(stdout);
    }
};

/// Advances the world to the next control-tick instant (the C2 pacer seam).
/// On the real robot this becomes R1's tick-boundary pacer; while the clock is
/// a fake it must ALSO advance that fake, or C2's stalled-pacer tripwire fires
/// (by design) on the first blocking verb.
struct V5DelayPacer final : shulib::motion::ITickPacer {
    static constexpr int kTickMs = 10;  ///< the motion tick (HA-32's 100 Hz loop)

    explicit V5DelayPacer(shulib::hal::fake::FakeClock& clock) : clock_{&clock} {}
    void pace() override {
        pros::delay(kTickMs);  // real time passes on the brain…
        // …and the fake clock follows it. TODO(R1): delete with the fake clock —
        // the pros-backed IClock reads real time and needs no help.
        clock_->advance(shulib::units::Time{kTickMs / 1000.0});
    }

private:
    shulib::hal::fake::FakeClock* clock_;
};

/// The whole robot, wired once. Member order IS initialization order — each
/// object is declared after everything it borrows, because the upper layers
/// borrow their dependencies rather than owning them (chassis.hpp
/// "Construction"). This is the §16.2 standalone promise executed for real:
/// plain C++, no config file, no builder, no codegen.
struct Robot {
    // ── drivetrain: config DATA, value-constructed. X-drive at the HA-17
    //    stand-in 7.0" drive radius — R3 measures the real machine's geometry.
    shulib::kinematics::MatrixKinematics kin = shulib::kinematics::xDrive(
        shulib::units::Length{7.0});

    // ── HAL seams. Every fake occupies the exact slot R1's adapter fills.
    shulib::hal::fake::FakeClock clock{};    // TODO(R1): IClock over PROS time
    shulib::hal::fake::FakeMotor frontLeft{}, backLeft{}, backRight{}, frontRight{};
    // TODO(R1): ^ IMotor over pros::Motor, ports from R3's measured port map.
    //    Order is x_drive.hpp's canonical wheel order (FL, BL, BR, FR) — the
    //    kinematics rows and this array MUST agree or every motion is wrong.
    shulib::hal::IMotor* driveMotors[4] = {&frontLeft, &backLeft, &backRight, &frontRight};
    shulib::hal::fake::FakeImu imu{};        // TODO(R1): IImu over pros::Imu —
    //    MUST apply imu_conversion.hpp's canonical conversion (HA-02).
    shulib::hal::fake::FakeGps gps{};        // TODO(R1): IGps over pros::Gps —
    //    MUST apply gps_conversion.hpp's canonical conversion (HA-01).
    shulib::hal::fake::FakeBattery battery{};  // TODO(R1): IBattery over PROS battery
    shulib::hal::fake::FakeRotation forwardEncoder{}, lateralEncoder{};
    // TODO(R1): ^ IRotation over pros::Rotation (the two tracking-wheel pods).
    shulib::hal::fake::FakeTagSource tags{};   // stub until the M3 vision pipeline (R2 camera)
    shulib::hal::fake::FakeVision vision{};    // stub until the M3 vision pipeline (R2 camera)

    // ── diagnostics: REAL on-target today — TermSink writes the V5 USB serial.
    StdoutCharSink usb{};
    shulib::diag::TermSink telemetry{clock, usb};
    shulib::diag::FaultLatch faults{telemetry, clock};
    shulib::diag::HealthMonitor health{faults};

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
    V5DelayPacer pacer{clock};
    shulib::chassis::Chassis chassis{deps, pacer};
};

/// Constructed on first use (initialize()), alive for the whole run. A
/// function-local static sidesteps static-init-order hazards and makes the
/// construction point explicit in the boot flow.
Robot& robot() {
    static Robot r;
    return r;
}

}  // namespace

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * Constructs the entire v2 object graph (the Robot struct above), then prints
 * an honest boot banner through the v2 diagnostics layer itself — so a `pros
 * terminal` reader sees exactly what this binary is and is not.
 */
void initialize() {
    Robot& r = robot();
    r.telemetry.log(shulib::hal::LogLevel::Info, "C7",
                    "shulib v2 core wired: X-drive kinematics + Pilons odometry + fused "
                    "localizer + motion scheduler + Chassis facade");
    r.telemetry.log(shulib::hal::LogLevel::Warn, "C7",
                    "HAL is fake-backed: hal/pros adapters are chunk R1 — this binary "
                    "CANNOT drive hardware");

    // One live query THROUGH the facade, so the banner is evidence of a working
    // object graph rather than prose: strafeAuthority() reads Chassis →
    // kinematics (F5) and must be 1.00 on this X-drive.
    char line[64];
    std::snprintf(line, sizeof line, "facade alive: strafeAuthority=%.2f (X-drive: 1.00)",
                  r.chassis.strafeAuthority());
    r.telemetry.log(shulib::hal::LogLevel::Info, "C7", line);
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
 * Deliberately commands NO motion: with fake-backed HAL there is nothing real
 * to drive, and a routine that "runs" against in-memory fakes on a live field
 * would be indistinguishable from a hang. The v2 autons exist and settle in
 * host sim (test/chassis_routine_test.cpp chains them through this same
 * facade); they are wired here at R3, after R1's adapters make motion real.
 */
void autonomous() {
    Robot& r = robot();
    r.telemetry.log(shulib::hal::LogLevel::Warn, "C7",
                    "autonomous(): no motion until hal/pros adapters land (R1) and the "
                    "stack is hardware-validated (R3)");
}

/**
 * Runs the operator control code.
 *
 * TODO(R1): the teleop loop becomes chassis.drive(speeds, Frame) at the loop
 * cadence — controller input via a pros::Controller adapter, frame named at
 * the call site (field-centric driving is a one-word change there). Until the
 * adapters exist this loop just keeps the task alive.
 */
void opcontrol() {
    Robot& r = robot();
    r.telemetry.log(shulib::hal::LogLevel::Info, "C7",
                    "opcontrol(): idle — teleop drive() loop arrives with R1's adapters");
    while (true) {
        pros::delay(250);
    }
}
