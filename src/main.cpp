// main.cpp — the PROS entry point, rewired onto the shulib v2 core (chunk C7,
// WS11/M2), with the hal/pros adapters wired in at chunk R1a, and — since R3b
// Parts 1–3 (2026-09-14) — the LIBRARY'S OWN TELEOP LOOP FOR ROBOT TWO.
//
// ═══ READ THIS FIRST: what this binary is, and what it is NOT ═══════════════════════
// Since R1a every HAL seam below is a REAL hal/pros adapter over a physical V5
// device: motors, IMU, GPS, battery, rotation sensors, controller, USB serial,
// controller LCD, real time, and the tick-boundary pacer. The fakes are gone
// from this file ENTIRELY — since R3b even vision/tags, which are now explicit
// ABSENT devices (hal/absent_*.hpp), not test doubles.
//
// WHAT THAT DOES **NOT** MEAN — the governing constraint, unchanged:
//   * THE LIBRARY HAS STILL NEVER DRIVEN A ROBOT. The adapters are host-tested
//     against a hand-written shim of PROS — which tests the adapter against
//     our BELIEF about PROS, never the belief itself. The beliefs are
//     inventoried in docs/hardware-assumptions.md (HA-94 onward) and settle on
//     a bench, not in a test suite. Since R3b Parts 1–3 the library's teleop
//     loop BUILDS for robot two ("shulib Teleop", below); it has NOT run.
//   * The X-DRIVE PORT MAP BELOW IS INVENTED (A4: HA-111). No X-drive robot has
//     been measured. Uploading that variant to a brain whose devices are not on
//     exactly these ports will fault loudly at boot (the adapters read their
//     configuration back). Robot two's ports, signs, IMU port and geometry live
//     in src/chassis_table.hpp, MEASURED, and are never retyped here.
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
//     (HA-112 records the raw mapping as invented). Since R3b Session 2 the
//     stick → ChassisSpeeds mapping itself lives in the PROS-free
//     shulib/teleop/stick_mapping.hpp (host-tested, bit-identical to what this
//     file carried), so the bench tester's DRIVE station and this loop share it.
//     Since R3b Parts 1–3 the LOOP BODY is one function, runTeleopLoop(), that
//     BOTH object graphs (the invented X-drive, robot two's tank) call — the
//     Session 2 brief's "one mapping, one loop" — plus one addition from Part
//     0b's finding: it polls the field DISABLE and exits cleanly (PROS deletes
//     the task on disable and a deleted task runs no destructor; VEXos cuts the
//     motors itself, and the clean exit lets the facade brake them first).
//
// ═══ Absent, deliberately (chunk R3b §6) ════════════════════════════════════════════
//   * AbsentTagSource / AbsentVision — NO camera is installed on any current robot.
//     These replaced the FakeTagSource/FakeVision stubs: a test fake in a competition
//     binary is test scaffolding, and it erases the difference between "this robot has
//     no such device" and "a test will inject readings here". An absent source is never
//     polled (§6.5) — no corrector, no vision task, no false "camera alive" record —
//     and the boot log announces the absence once. R2 (camera) swaps in the real
//     adapter, visibly, on this exact line.
//   * AbsentGps — robot two (the tank graph below) has no GPS; its context wiring
//     spells that as `.gps = &gps` over a hal::AbsentGps. The invented X-drive graph
//     keeps its ProsGps: HA-111's port map defines it WITH a GPS on port 9.

#include "main.h"

// ═══ WHICH ROBOT, WHICH PROGRAM? (GATE1; third variant at R3b S2; PROGRAM axis at Part 0b;
//     the LIBRARY program at Parts 1–3) ═══════════════════════════════════════════════
// The X-drive wiring below is PRESERVED VERBATIM and relabelled -- it is the only
// artifact of the 2026-08-12 whole-object-graph boot, and its ports are invented
// (HA-111). It CANNOT boot on the measured bench robot: it puts motors on 1/2/-3/-4
// (port 4 is the IMU), rotation sensors on 5/6, and a GPS on 9 -- every one of those
// throws an adapter read-back precondition at boot.
//
// Three robot variants on one validated Makefile switch, and a SECOND AXIS for the program
// -- any other value of either is an $(error):
//   make                              → ROBOT=bench  the measured tank BENCH bot: runs the bench
//                                       tester ("Bench Tests")
//   make ROBOT=tank                   → the 2026 tank chassis, ROBOT TWO: runs the bench tester
//                                       over robot two's SIGNED chassis table (src/chassis_table.hpp)
//   make ROBOT=tank PROGRAM=drive     → "shulib Drive": robot two DRIVES from the sticks through
//                                       the hal/pros adapters with dead-port tolerance
//                                       (src/drive_program.cpp) -- NOT the motion stack
//   make ROBOT=tank PROGRAM=library   → "shulib Teleop" (R3b Parts 1-3): THE LIBRARY drives robot
//                                       two -- the tank object graph below, built from the chassis
//                                       table, through Chassis::drive(). Slot 2.
//   make ROBOT=xdrive                 → the invented X-drive wiring below (HA-111)
// (The instruction here used to say `make CXXFLAGS_EXTRA=-D...` — common.mk consumes
//  EXTRA_CXXFLAGS, the names were transposed, and the documented command silently built
//  the BENCH variant. GATE1 replaced it with the validated ROBOT switch in the Makefile,
//  and tools/src_build_gate.py now asserts per-build that the defines really land --
//  as whole command-line TOKENS, and end-to-end via the beacon string below.)
#if defined(SHULIB_ROBOT_XDRIVE_INVENTED) && defined(SHULIB_ROBOT_TANK_2026)
#error "two robot variant defines are set at once -- select exactly one with make ROBOT=..."
#endif
#if defined(SHULIB_PROGRAM_DRIVE) && defined(SHULIB_PROGRAM_LIBRARY)
#error "two PROGRAM defines are set at once -- select exactly one with make PROGRAM=..."
#endif
#if defined(SHULIB_PROGRAM_DRIVE) && !defined(SHULIB_ROBOT_TANK_2026)
#error "PROGRAM=drive needs ROBOT=tank -- only robot two's chassis table carries measured signs"
#endif
#if defined(SHULIB_PROGRAM_LIBRARY) && !defined(SHULIB_ROBOT_TANK_2026)
#error "PROGRAM=library needs ROBOT=tank -- only robot two's chassis table is signed, measured, and carries geometry"
#endif
#if defined(SHULIB_ROBOT_XDRIVE_INVENTED)
#define SHULIB_RUNS_BENCH_TESTER 0
#define SHULIB_RUNS_TANK_LIBRARY 0
#elif defined(SHULIB_PROGRAM_LIBRARY)
#define SHULIB_RUNS_BENCH_TESTER 0
#define SHULIB_RUNS_TANK_LIBRARY 1  // robot two, driven by the LIBRARY (the tank graph below)
#else
#define SHULIB_RUNS_BENCH_TESTER 1  // bench AND tank: a tester-variant robot (no library graph);
                                    // which PROGRAM runs in opcontrol() is the second axis
#define SHULIB_RUNS_TANK_LIBRARY 0
#endif
// The competition-state screens exist for every program that is NOT the invented X-drive
// (which the src build gate expects to emit no unused-function warning from this file).
#define SHULIB_STATE_SCREENS (SHULIB_RUNS_BENCH_TESTER || SHULIB_RUNS_TANK_LIBRARY)

#if SHULIB_RUNS_BENCH_TESTER
#include "bench_r3a.hpp"
#if defined(SHULIB_PROGRAM_DRIVE)
#include "drive_program.hpp"
#endif
#endif

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <optional>
#include <span>
#include <string_view>

#include "shulib/chassis/chassis.hpp"
#include "shulib/chassis/robot_context.hpp"
#include "shulib/core/check.hpp"
#include "shulib/diag/build_info.hpp"
#include "shulib/diag/controller_display.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/diag/health_monitor.hpp"
#include "shulib/diag/session_info.hpp"
#include "shulib/diag/term_sink.hpp"
#include "shulib/hal/absent_tag_source.hpp"
#include "shulib/hal/absent_vision.hpp"
#include "shulib/hal/controller.hpp"
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
#include "shulib/teleop/stick_mapping.hpp"
#include "shulib/units/quantity.hpp"

#if SHULIB_RUNS_TANK_LIBRARY
#include "chassis_table.hpp"
#include "shulib/hal/absent_gps.hpp"
#include "shulib/hal/drive_geometry.hpp"
#include "shulib/hal/motor_group.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/localization/drive_encoder_odometry.hpp"
#include "shulib/motion/odo_stall_check.hpp"
#endif

namespace {

// ═══ THE VARIANT IDENTITY BEACON (R3b Session 2; asserted by tools/src_build_gate.py) ═══
// One string per BUILD, selected by the SAME preprocessor facts that select the wiring and
// the program, and printed at boot so it is referenced and survives into the linked package.
// The src build gate asserts the expected beacon IS in bin/hot.package.elf and the other
// four are NOT (matched WITH the string's NUL terminator, because `…=tank` is a prefix of
// `…=tank-drive` and of `…=tank-library`). That is the end-to-end proof that the #if chain
// in THIS file took the branch the Makefile asked for: the gate's compile-line check can see
// a define that never landed, but not a macro renamed here -- and a tank build that silently
// carried the bench bot's chassis table onto robot two is exactly HA-111's defect class
// again; a "shulib Drive" or "shulib Teleop" upload that was silently the tester is the
// same class on the second axis.
#if defined(SHULIB_ROBOT_XDRIVE_INVENTED)
constexpr const char kVariantBeacon[] = "shulib-robot-variant=xdrive";
#elif defined(SHULIB_ROBOT_TANK_2026) && defined(SHULIB_PROGRAM_DRIVE)
constexpr const char kVariantBeacon[] = "shulib-robot-variant=tank-drive";
#elif defined(SHULIB_ROBOT_TANK_2026) && defined(SHULIB_PROGRAM_LIBRARY)
constexpr const char kVariantBeacon[] = "shulib-robot-variant=tank-library";
#elif defined(SHULIB_ROBOT_TANK_2026)
constexpr const char kVariantBeacon[] = "shulib-robot-variant=tank";
#else
constexpr const char kVariantBeacon[] = "shulib-robot-variant=bench";
#endif

#if SHULIB_STATE_SCREENS
// ═══ COMPETITION-STATE VISIBILITY (2026-09-10, robot two's brain) ══════════════════════
// The tester runs inside opcontrol(), so it exists only while VEXos reports DRIVER
// CONTROL, ENABLED. The first time a controller was linked to a brain running it, the
// program restarted and then showed a BLACK screen for as long as the controller stayed
// linked: PROS never started the driver task, and nothing on the brain said why. Every
// non-driver entry point below now paints the screen with the state VEXos reports and what
// to do about it, and the raw status bits print at boot -- so "the touchscreen does not
// work" cannot be the diagnosis again (A4 register HA-128: launching from the controller's
// own menu arms the pretend match). Guarded, so the xdrive variant (which the src build
// gate expects to emit no unused-function warning) never sees an unused helper. The
// library program ("shulib Teleop") shares these screens: it, too, drives only in driver
// control, enabled.
const char* competitionBits(std::uint8_t s, char* buf, std::size_t n) {
    std::snprintf(buf, n, "0x%02x field=%s disabled=%s auton=%s", static_cast<unsigned>(s),
                  (s & COMPETITION_CONNECTED) ? "yes" : "no",
                  (s & COMPETITION_DISABLED) ? "yes" : "no",
                  (s & COMPETITION_AUTONOMOUS) ? "yes" : "no");
    return buf;
}

void benchStateScreen(const char* state, const char* why) {
    char bits[64];
    competitionBits(pros::c::competition_get_status(), bits, sizeof bits);
    std::printf("[R3A] competition state: %s (%s) -- %s\n", state, bits, why);
    std::fflush(stdout);
    pros::c::screen_set_eraser(0x000000);
    pros::c::screen_erase();
    pros::c::screen_set_pen(0xF0C000);
    pros::c::screen_print_at(pros::E_TEXT_LARGE, 10, 30, "%s", state);
    pros::c::screen_set_pen(0xFFFFFF);
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 10, 80,
                             "This program runs ONLY in DRIVER CONTROL, enabled.");
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 10, 100, "%s", why);
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 10, 130, "status bits %s", bits);
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 10, 160,
                             "Look at the CONTROLLER's screen: it names this state.");
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 10, 176,
                             "Unplug any competition switch or field cable from the");
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 10, 192,
                             "controller's smart port. This screen clears by itself.");
}
#endif  // SHULIB_STATE_SCREENS

// ── PROVISIONAL PORT MAP — INVENTED (A4: HA-111). No X-drive robot has been measured;
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

// The X-drive's drive cartridge: GREEN is HA-15's INVENTED stand-in until the build
// team's cartridges are read off the physical motors (runbook step 0).
constexpr shulib::hal::pros::MotorGearset kDriveGearset =
    shulib::hal::pros::MotorGearset::Green;

// (The teleop deadband constant and shaped() that lived here moved, unchanged, to
//  shulib/teleop/stick_mapping.hpp at R3b Session 2 -- HA-112 still invented, T2 still
//  the owner of real driver feel.)

/// The on-robot precondition policy (check.hpp §18.4): raise the fault code on
/// the latch (visible in telemetry + the run summary), then throw the same
/// PreconditionError the host policy throws — the motion scheduler catches it
/// at the task boundary and converts it to FAULT_ABORT + a safe drivetrain
/// state. During Robot construction the latch does not exist yet, so the
/// handler degrades to plain-throw — still loud, still never-returns.
/// (What an UNCAUGHT throw out of initialize() does on the brain is exactly
/// bench runbook step 1's measurement — brief T8: demonstrate, don't assert.
/// The tank library graph CATCHES it at the graph boundary and refuses, below.)
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
    //    ABSENT, deliberately (R3b §6): no camera is installed on any current robot, and a
    //    test fake in a competition binary erases "no device" vs "a test injects here". No
    //    corrector or vision task is wired over these — kInstallTagCorrector<AbsentTagSource>
    //    is false, so polling one would manufacture a false "camera alive, no tags" record
    //    (§6.5). The boot log announces the absence once; R2 (camera) swaps in the real
    //    adapter here. (Robot two's tank graph below spells its missing GPS as
    //    `.gps = &gps` over a hal::AbsentGps. THIS invented X-drive graph keeps its
    //    ProsGps: HA-111's port map defines it WITH a GPS on port 9, and
    //    portMapString() below says so.)
    shulib::hal::AbsentTagSource tags{};
    shulib::hal::AbsentVision vision{};

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

// ═══ THE TELEOP LOOP — ONE function, both graphs (R3b Session 2 §6.2; Parts 1–3) ═══════
/// The R1a loop body, unchanged in shape: read the sticks (channels polled only while the
/// controller is connected — a dropped controller commands zero twist, HA-103), map them
/// through the ONE shared mapping (HA-112, invented; T2 owns driver feel), drive BODY-frame
/// through the facade, let the caller paint its panel, pace to the tick boundary. Plus Part
/// 0b's finding: a field DISABLE is polled and exits the loop CLEANLY — PROS deletes the
/// opcontrol task on disable and a deleted task runs no destructor, so this is the only path
/// on which the facade's cancel() (0 V + Brake) runs before VEXos cuts the motors itself;
/// PROS calls opcontrol() again when driver control resumes. A function template so that the
/// builds which do not call it (bench, tank, tank-drive) emit no -Wunused-function for it.
template <typename PerTick>
void runTeleopLoop(shulib::chassis::Chassis& chassis, const shulib::hal::IController& master,
                   shulib::motion::ITickPacer& pacer, PerTick&& perTick) {
    const shulib::motion::MotionConfig& cfg = chassis.motionConfig();
    while (true) {
        if (pros::c::competition_is_disabled()) {
            chassis.cancel();  // the panic-stop safe state; VEXos disables output too
            return;
        }
        shulib::teleop::StickInput sticks{.connected = master.isConnected()};
        if (sticks.connected) {
            sticks.leftY = master.axis(shulib::hal::ControllerAxis::LeftY);
            sticks.leftX = master.axis(shulib::hal::ControllerAxis::LeftX);
            sticks.rightX = master.axis(shulib::hal::ControllerAxis::RightX);
        }
        const shulib::math::ChassisSpeeds command = shulib::teleop::mapSticksToChassisSpeeds(
            sticks, cfg.maxLinearSpeed, cfg.maxAngularSpeed);
        chassis.drive(command, shulib::math::Frame::Body);
        perTick(command, sticks.connected);
        pacer.pace();
    }
}

#if SHULIB_RUNS_TANK_LIBRARY
// ═══ ROBOT TWO'S OBJECT GRAPH — "shulib Teleop" (R3b Parts 1–3, 2026-09-14) ═══════════
// Built ENTIRELY from src/chassis_table.hpp: ten ProsMotors from the SIGNED ports with the
// table's cartridge (the sign lives in the table entry and is applied by PROS once, in the
// adapter — the group never negates), two hal::MotorGroups in the kinematics' wheel order
// (left, right), ProsImu on the table's port, the explicit absent devices, ProsBattery,
// the diagnostics stack exactly as the X-drive graph has it, DriveEncoderOdometry from the
// two groups + the IMU + the table's geometry, ComplementaryFusion, Localizer, RobotContext,
// MotionDeps (with the groups, so MOTOR_GROUP_DISAGREE fires in drive()), ProsTickPacer,
// Chassis. NOTHING here is typed twice: the port-map string, the geometry and the
// kinematics all read the table.
//
// BOOT REFUSAL, NOT FAULT-ABORT: if the table's geometry is UNSET, or the IMU port is 0, or
// the table contradicts itself, initialize() refuses BEFORE any adapter is constructed and
// paints the state screen with exactly what is missing; if a ProsMotor (or the IMU)
// constructor throws, the graph boundary catches it and refuses the same way. opcontrol()
// then idles: no adapter exists, nothing is powered. DEAD-PORT TOLERANCE IS NOT IN SCOPE
// for this program — the group requires N >= 1 PRESENT members at construction, and one
// refused adapter refuses the whole graph; "shulib Drive" (slot 1) is the tolerant program
// until R3d. The panel and the boot log say so.
//
// EXPECT IT TO FEEL DIFFERENT from "shulib Drive", and the panel says so: the pipeline
// commands wheel SPEEDS through feedforward with PROVISIONAL gains (HA-45: kV = 12/70 V per
// in/s, a placeholder), capped by MotionConfig::maxLinearSpeed / maxWheelSpeed, plus battery
// compensation. If it is slow or saturates, that is R5's measurement showing up, not a bug;
// the panel prints the budgets and the gains in use.
namespace tanklib {

constexpr const char* kBuildStamp = __DATE__ " " __TIME__;
constexpr int kPanelEveryTicks = 10;  // 100 ms
constexpr int kLcdEveryTicks = 20;    // 200 ms = the 5 Hz ceiling; writes only on change
constexpr std::int16_t kUsableW = 480;
constexpr std::uint32_t kColBg = 0x101418, kColBar = 0x1E5AA8, kColText = 0xF0F0F0,
                        kColDim = 0x9AA4AE, kColGood = 0x39C36E, kColWarn = 0xE8B23A,
                        kColBad = 0xE2564A, kColSub = 0xC8D8F0;

shulib::hal::pros::MotorGearset gearsetOf(shulib::bench::Cartridge c) {
    switch (c) {
        case shulib::bench::Cartridge::Red: return shulib::hal::pros::MotorGearset::Red;
        case shulib::bench::Cartridge::Green: return shulib::hal::pros::MotorGearset::Green;
        default: return shulib::hal::pros::MotorGearset::Blue;  // Unset is refused before this
    }
}

/// The ten adapters, constructed from the table in ONE place, in table order per side, and
/// the two pointer arrays the groups view. A throwing ProsMotor constructor propagates out
/// of here — and out of TankRobot — to initialize()'s catch: the refusal path.
struct TankMotors {
    std::optional<shulib::hal::pros::ProsMotor> storage[shulib::bench::kMaxPort];
    shulib::hal::IMotor* left[shulib::bench::kMaxPort] = {};
    shulib::hal::IMotor* right[shulib::bench::kMaxPort] = {};
    std::size_t leftN = 0;
    std::size_t rightN = 0;

    TankMotors(const shulib::bench::ChassisTable& t, shulib::hal::pros::MotorGearset gearset) {
        std::size_t k = 0;
        for (std::size_t i = 0; i < t.leftCount; ++i, ++k) {
            storage[k].emplace(t.left[i], gearset);  // AS TYPED: the one place a sign lives
            left[leftN++] = &*storage[k];
        }
        for (std::size_t i = 0; i < t.rightCount; ++i, ++k) {
            storage[k].emplace(t.right[i], gearset);
            right[rightN++] = &*storage[k];
        }
    }
    [[nodiscard]] std::span<shulib::hal::IMotor* const> leftSpan() const {
        return std::span<shulib::hal::IMotor* const>{left, leftN};
    }
    [[nodiscard]] std::span<shulib::hal::IMotor* const> rightSpan() const {
        return std::span<shulib::hal::IMotor* const>{right, rightN};
    }
};

/// The chassis config for this drivetrain: the MotionConfig defaults (PROVISIONAL gains and
/// budgets, HA-45/50/51/52) with the stall check told the truth about its inputs — every
/// wheel slot carries the table's geometry (the SAME object the odometry converts with), and
/// there is NO independent motion source (the §2 ruling: odo_stall_check.hpp).
shulib::chassis::ChassisConfig tankChassisConfig(shulib::hal::DriveGeometry geometry) {
    shulib::chassis::ChassisConfig cfg{};
    cfg.motion.stall.setAllWheels(geometry);
    cfg.motion.stall.independentMotionSource = false;
    return cfg;
}

struct TankRobot {
    const shulib::bench::ChassisTable& table = shulib::bench::kChassis;
    // ── drivetrain: config DATA from the table. Track width MEASURED by tape, never typed
    //    here (UNSET refuses before this constructor runs).
    shulib::kinematics::TankKinematics kin{shulib::bench::tableTrackWidth(table)};
    shulib::hal::pros::ProsClock clock{};
    // ── the ten adapters, then ONE IMotor per kinematic wheel: the groups (R3b Part 1).
    TankMotors motors{table, gearsetOf(table.cartridge)};
    shulib::hal::MotorGroup left{motors.leftSpan()};
    shulib::hal::MotorGroup right{motors.rightSpan()};
    shulib::hal::IMotor* driveMotors[2] = {&left, &right};  // TankKinematics: 0 = left, 1 = right
    shulib::hal::MotorGroup* groups[2] = {&left, &right};
    // ── sensors: the IMU on the table's port (bootHeading 0: the robot boots facing +X);
    //    everything else ABSENT, explicitly (R3b §6 — never polled, announced once).
    shulib::hal::pros::ProsImu imu{table.imuPort, shulib::math::Angle{}, clock};
    shulib::hal::AbsentGps gps{};
    shulib::hal::AbsentTagSource tags{};
    shulib::hal::AbsentVision vision{};
    shulib::hal::pros::ProsBattery battery{};
    shulib::hal::pros::ProsController master{shulib::hal::pros::ControllerId::Master};
    // ── diagnostics: exactly as the X-drive graph has it.
    shulib::hal::pros::ProsCharSink usb{};
    shulib::diag::TermSink telemetry{clock, usb};
    shulib::diag::FaultLatch faults{telemetry, clock};
    shulib::diag::HealthMonitor health{faults};
    shulib::hal::pros::ProsLineDisplay lcd{};
    // ── ONE drive geometry, instantiated from the table and handed to BOTH consumers. Robot
    //    two is believed symmetric, so both sides get the same object; the type represents
    //    asymmetry the day a per-side measurement says otherwise.
    shulib::hal::DriveGeometry geometry = shulib::bench::tableDriveGeometry(table);
    // ── localization: drive-encoder odometry over the two groups (R3b Part 2), fused.
    shulib::localization::DriveEncoderOdometry odom{imu, left, right, geometry, geometry,
                                                    shulib::bench::tableTrackWidth(table)};
    shulib::localization::ComplementaryFusion fusion{};
    shulib::localization::Localizer localizer{clock, imu, odom, fusion};
    shulib::chassis::RobotContext ctx{{.clock = &clock,
                                       .driveMotors = driveMotors,
                                       .imu = &imu,
                                       .gps = &gps,
                                       .battery = &battery,
                                       .telemetry = &telemetry,
                                       .tags = &tags,
                                       .vision = &vision}};
    shulib::motion::MotionDeps deps{.ctx = &ctx,
                                    .localizer = &localizer,
                                    .kinematics = &kin,
                                    .faults = &faults,
                                    .health = &health,
                                    .motorGroups = groups};
    shulib::hal::pros::ProsTickPacer pacer{};
    shulib::chassis::Chassis chassis{deps, pacer, tankChassisConfig(geometry)};
};

/// Constructed in initialize() inside the graph-boundary try; a throwing adapter leaves it
/// unconstructed and g_tank null (the refusal path). Function-local static, like robot().
TankRobot& tankRobot() {
    static TankRobot r;
    return r;
}
TankRobot* g_tank = nullptr;
char g_refusalHeadline[64] = "";
char g_refusalWhy[160] = "";

/// The §18.5 port-map string, built from the TABLE — never retyped.
const char* tankPortMapString() {
    // Sized so the whole string fits kMaxPortMapBytes: five signed ports render as 19
    // characters per side ("-11 +12 -13 +14 -15"); a 24-byte side buffer bounds a longer
    // table to what fits, and the session header sanitizes/truncates on top.
    static char buf[shulib::diag::kMaxPortMapBytes];
    char l[24], r[24];
    shulib::bench::portsToString(shulib::bench::kChassis.left, shulib::bench::kChassis.leftCount,
                                 shulib::bench::kChassis.signsMeasured, l, sizeof l);
    shulib::bench::portsToString(shulib::bench::kChassis.right, shulib::bench::kChassis.rightCount,
                                 shulib::bench::kChassis.signsMeasured, r, sizeof r);
    std::snprintf(buf, sizeof buf, "L %s R %s IMU%u W%.2f TW%.2f G%.3f", l, r,
                  static_cast<unsigned>(shulib::bench::kChassis.imuPort),
                  shulib::bench::kChassis.wheelDiameterIn, shulib::bench::kChassis.trackWidthIn,
                  shulib::bench::kChassis.externalRatio);
    return buf;
}

/// Paint a full-screen refusal (the drive program's shape) and say it on serial. Nothing is
/// powered: no adapter was constructed, or the one that threw took the graph with it.
void paintRefusal() {
    pros::c::screen_set_eraser(kColBg);
    pros::c::screen_erase();
    pros::c::screen_set_pen(kColBad);
    pros::c::screen_fill_rect(0, 0, kUsableW, 34);
    pros::c::screen_set_eraser(kColBad);
    pros::c::screen_set_pen(kColText);
    pros::c::screen_print_at(pros::E_TEXT_MEDIUM, 8, 6, "shulib TELEOP -- REFUSED AT BOOT");
    pros::c::screen_set_eraser(kColBg);
    pros::c::screen_set_pen(kColBad);
    pros::c::screen_print_at(pros::E_TEXT_MEDIUM, 8, 48, "%.34s", g_refusalHeadline);
    pros::c::screen_set_pen(kColText);
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 84, "%.56s", g_refusalWhy);
    if (std::strlen(g_refusalWhy) > 56) {
        pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 100, "%.56s", g_refusalWhy + 56);
    }
    pros::c::screen_set_pen(kColDim);
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 128,
                             "The library graph did NOT construct. Nothing is powered.");
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 144,
                             "Geometry/IMU: measure, type into src/chassis_table.hpp,");
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 160,
                             "rebuild, upload. An adapter refusal: Bench Tests (slot 3).");
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 184,
                             "No dead-port tolerance here: shulib Drive (slot 1) has it.");
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 208, "BUILD %s", kBuildStamp);
    std::printf("[R3B] REFUSED AT BOOT: %s -- %s\n", g_refusalHeadline, g_refusalWhy);
    std::fflush(stdout);
}

void refuse(const char* headline, const char* why) {
    std::snprintf(g_refusalHeadline, sizeof g_refusalHeadline, "%s", headline);
    std::snprintf(g_refusalWhy, sizeof g_refusalWhy, "%s", why);
    paintRefusal();
}

/// One row of the panel, erased then printed (the drive program's helper, duplicated on
/// purpose: two ~60-line status pages, no shared screen library yet — D0b-9).
void rowText(std::int16_t y, std::uint32_t colour, const char* text) {
    pros::c::screen_set_eraser(kColBg);
    pros::c::screen_erase_rect(0, y, kUsableW, static_cast<std::int16_t>(y + 16));
    pros::c::screen_set_pen(colour);
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, y, "%s", text);
}

const char* qualityName(shulib::localization::Localizer::Quality q) {
    using Q = shulib::localization::Localizer::Quality;
    switch (q) {
        case Q::Uninitialized: return "UNINIT";
        case Q::DeadReckon: return "DEADRECKON";
        case Q::Corrected: return "CORRECTED";
        case Q::Degraded: return "DEGRADED";
    }
    return "?";
}

/// The brain panel's fixed header: name, what this is, build stamp.
void drawHeader() {
    pros::c::screen_set_eraser(kColBg);
    pros::c::screen_erase();
    pros::c::screen_set_pen(kColBar);
    pros::c::screen_fill_rect(0, 0, kUsableW, 32);
    pros::c::screen_set_eraser(kColBar);
    pros::c::screen_set_pen(kColText);
    pros::c::screen_print_at(pros::E_TEXT_MEDIUM, 8, 2, "shulib TELEOP");
    pros::c::screen_set_pen(kColSub);
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 20, "THE LIBRARY drives: Chassis::drive()");
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 250, 20, "BUILD %s", kBuildStamp);
    pros::c::screen_set_eraser(kColBg);
}

/// The per-tick panel + LCD for the library teleop (brief §4): session essentials, the
/// estimate and its quality class, the two group commands, each group's member count /
/// disagreeing count, the heading cross-check, battery, any latched fault by name, the
/// budgets and gains in use, and the two honest notes (no independent stall source; no
/// dead-port tolerance). LCD: `LIB 12V Bxx.xV` / `hdg ±ddd.d deg` / last fault or `ok`.
struct TankPanel {
    TankRobot& r;
    int tick = 0;
    char lcdRow[3][24] = {"", "", ""};

    void operator()(const shulib::math::ChassisSpeeds& command, bool connected) {
        ++tick;
        char line[96];
        if (tick % kPanelEveryTicks == 0) {
            const shulib::math::Pose2d pose = r.localizer.pose();
            const shulib::localization::Localizer::Quality q = r.localizer.qualityClass();
            std::snprintf(line, sizeof line, "cmd vx %+5.1f in/s  w %+5.2f rad/s   %s",
                          command.vx().value(), command.omega().value(),
                          connected ? "" : "NO CONTROLLER -> 0");
            rowText(38, connected ? kColText : kColWarn, line);
            std::snprintf(line, sizeof line, "est x %+7.1f  y %+7.1f  hdg %+6.1f deg  %s",
                          pose.x().value(), pose.y().value(),
                          pose.heading().radians() * 180.0 / shulib::math::Angle::kPi,
                          qualityName(q));
            rowText(56, q == shulib::localization::Localizer::Quality::Degraded ? kColWarn
                                                                                 : kColText,
                    line);
            std::snprintf(line, sizeof line, "L %+5.1fV %u members %d disagree | R %+5.1fV %u members %d disagree",
                          r.left.commandedVoltage().value(),
                          static_cast<unsigned>(r.left.memberCount()),
                          r.left.disagreeingMembers(), r.right.commandedVoltage().value(),
                          static_cast<unsigned>(r.right.memberCount()),
                          r.right.disagreeingMembers());
            rowText(74, (r.left.disagreeingMembers() + r.right.disagreeingMembers()) > 0
                            ? kColBad
                            : kColGood,
                    line);
            std::snprintf(line, sizeof line,
                          "hdg xcheck %+7.4f rad/tick (enc-IMU)   battery %5.2f V   L/R travel %+6.2f %+6.2f",
                          r.odom.lastHeadingDisagreement(), r.battery.voltage().value(),
                          r.odom.lastSideTravel().left.value(),
                          r.odom.lastSideTravel().right.value());
            rowText(92, kColText, line);
            if (r.faults.hasFault()) {
                std::snprintf(line, sizeof line, "FAULT latched: first %s, last %s, n=%d",
                              shulib::diag::faultCodeName(r.faults.firstFault()),
                              shulib::diag::faultCodeName(r.faults.lastFault()),
                              r.faults.faultCount());
                rowText(110, kColBad, line);
            } else {
                rowText(110, kColGood, "faults: none");
            }
            const shulib::motion::MotionConfig& cfg = r.chassis.motionConfig();
            std::snprintf(line, sizeof line,
                          "budgets lin %.0f ang %.1f wheel %.0f | ff kS %.2f kV %.3f PROVISIONAL HA-45",
                          cfg.maxLinearSpeed.value(), cfg.maxAngularSpeed.value(),
                          cfg.maxWheelSpeed.value(), cfg.wheelFf.kS, cfg.wheelFf.kV);
            rowText(128, kColDim, line);
            rowText(146, kColDim, "FEEL differs from shulib Drive: speeds via feedforward, not raw volts");
            rowText(164, kColWarn, "stall check: NO independent source (odometry IS the wheels)");
            rowText(182, kColWarn, "dead ports: NOT tolerated here -- shulib Drive (slot 1) is");
            std::snprintf(line, sizeof line, "table %s", tankPortMapString());
            rowText(200, kColDim, line);
        }
        if (tick % kLcdEveryTicks == 0) {
            char want[3][24];
            std::snprintf(want[0], sizeof want[0], "LIB %2.0fV B%4.1fV",
                          shulib::hal::kMaxMotorVoltage.value(), r.battery.voltage().value());
            std::snprintf(want[1], sizeof want[1], "hdg %+6.1f deg",
                          r.localizer.pose().heading().radians() * 180.0 / shulib::math::Angle::kPi);
            std::snprintf(want[2], sizeof want[2], "%.19s",
                          r.faults.hasFault() ? shulib::diag::faultCodeName(r.faults.lastFault())
                          : !connected       ? "no controller 0V"
                                             : "ok");
            for (int row = 0; row < 3; ++row) {
                if (std::strcmp(want[row], lcdRow[row]) != 0) {
                    r.lcd.setLine(row, want[row]);
                    std::memcpy(lcdRow[row], want[row], sizeof lcdRow[row]);
                }
            }
        }
    }
};

}  // namespace tanklib
#endif  // SHULIB_RUNS_TANK_LIBRARY

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

#if SHULIB_RUNS_BENCH_TESTER
    // R3a/R3b: a tester-variant robot (bench bot, or the 2026 tank chassis). The X-drive
    // graph below is NOT constructed -- its ports are invented and every adapter ctor
    // would throw here. The program runs from opcontrol(); this only proves the binary
    // booted, and says WHICH build it is (the beacon the src build gate asserts).
#if defined(SHULIB_PROGRAM_DRIVE)
    std::printf("\n[R3B] booted: shulib DRIVE build [%s].\n"
                "[R3B] robot two drives from the sticks through the hal/pros adapters (open-loop\n"
                "[R3B] volts, dead-port tolerance, 1 s non-fatal cuts) -- never through the motion\n"
                "[R3B] stack. The library has still not driven a robot.\n"
                "[R3B] the X-drive object graph is deliberately NOT constructed.\n",
                kVariantBeacon);
#else
    std::printf("\n[R3A] booted: bench-tester build [%s].\n"
                "[R3A] read-only EXCEPT the tester's DRIVE station (R3b Session 2), which powers\n"
                "[R3A] the drive motors through the hal/pros adapters behind six safety gates --\n"
                "[R3A] never through the motion stack. The library has still not driven a robot.\n"
                "[R3A] the X-drive object graph is deliberately NOT constructed.\n",
                kVariantBeacon);
#endif
    {
        char bits[64];
        std::printf("[R3A] competition status at boot: %s -- the tester runs only in DRIVER "
                    "CONTROL, enabled; any other state paints the screen with the reason.\n",
                    competitionBits(pros::c::competition_get_status(), bits, sizeof bits));
    }
    std::fflush(stdout);
    return;
#elif SHULIB_RUNS_TANK_LIBRARY
    using namespace tanklib;
    std::printf("\n[R3B] booted: shulib TELEOP build [%s] -- THE LIBRARY drives robot two through\n"
                "[R3B] Chassis::drive() over two hal::MotorGroups and drive-encoder odometry, built\n"
                "[R3B] from src/chassis_table.hpp. Not yet run on hardware before this boot: the\n"
                "[R3B] first run is wheels-up. The invented X-drive graph is NOT constructed.\n"
                "[R3B] BUILD STAMP %s\n",
                kVariantBeacon, kBuildStamp);
    {
        char bits[64];
        std::printf("[R3B] competition status at boot: %s -- the loop runs only in DRIVER CONTROL, "
                    "enabled; any other state paints the screen with the reason.\n",
                    competitionBits(pros::c::competition_get_status(), bits, sizeof bits));
    }
    std::fflush(stdout);

    // ── the table must be consistent, signed, and COMPLETE for the library — geometry and
    //    IMU included — or nothing is constructed (never invented, never fault-aborted).
    char why[160];
    if (!shulib::bench::tableConsistent(shulib::bench::kChassis, why, sizeof why)) {
        refuse("chassis table CONTRADICTS ITSELF", why);
        return;
    }
    if (!shulib::bench::kChassis.signsMeasured) {
        refuse("chassis table has NO SIGNS", "only a table with MEASURED signs can drive");
        return;
    }
    if (shulib::bench::describeMissingForLibrary(shulib::bench::kChassis, why, sizeof why)) {
        refuse("chassis table UNSET for the library:", why);  // `why` names each missing field
        return;
    }
    // ── the graph, at its boundary: a throwing adapter refuses the whole program.
    try {
        TankRobot& r = tankRobot();
        g_tank = &r;
    } catch (const shulib::PreconditionError& e) {
        refuse("an adapter REFUSED at construction", e.what());
        return;
    }
    TankRobot& r = *g_tank;
    g_faults = &r.faults;
    r.telemetry.log(shulib::hal::LogLevel::Info, "R3B", kVariantBeacon);

    // Start IMU calibration now (non-blocking); readings are garbage until isReady()
    // (HA-23) and the motion layer's wait-for-live already gates on it — the boot-window
    // rule: a body-frame drive() works during calibration, a field-frame one waits.
    r.imu.calibrate();

    // ── the §18.5 session header: provenance FIRST, missing hash LOUD; the port map, the
    //    signs and the geometry all come from the table, never retyped.
    shulib::diag::SessionInfo session{};
    session.buildHash = shulib::diag::compiledBuildHash();
    session.routineId = "(none: teleop only; autons land later)";
    session.alliance = "";
    session.side = "";
    session.portMap = tankPortMapString();
    shulib::diag::emitSessionHeader(r.telemetry, session, r.battery.voltage());

    char line[240];
    std::snprintf(line, sizeof line,
                  "library graph BUILT from src/chassis_table.hpp: %s | cartridge %s (a belief the "
                  "adapter WROTE and read back) | geometry: %s | %u+%u ProsMotors behind two "
                  "MotorGroups (median reads; signs in the table only)",
                  r.table.robot, shulib::bench::cartridgeWord(r.table.cartridge),
                  r.table.geometryProvenance, static_cast<unsigned>(r.left.memberCount()),
                  static_cast<unsigned>(r.right.memberCount()));
    r.telemetry.log(shulib::hal::LogLevel::Info, "R3B", line);
    r.telemetry.log(shulib::hal::LogLevel::Warn, "R3B",
                    "HARDWARE-UNVALIDATED: the library's motion stack has not driven a robot "
                    "before this program; gains and budgets are PROVISIONAL (HA-45/50/51/52) -- "
                    "expect it to FEEL different from shulib Drive (speeds via feedforward, "
                    "capped by the budgets), which is R5's measurement showing, not a bug");
    r.telemetry.log(shulib::hal::LogLevel::Info, "R3B",
                    "absent devices (deliberate): gps=ABSENT tags=ABSENT vision=ABSENT -- no "
                    "GPS or camera on robot two; nothing is polled over an absent source");
    // The §2 ruling, said ONCE at boot (odo_stall_check.hpp carries the reasoning).
    r.telemetry.log(shulib::hal::LogLevel::Warn, "R3B", shulib::motion::kNoIndependentStallSourceNote);
    r.telemetry.log(shulib::hal::LogLevel::Info, "R3B",
                    "dead-port tolerance: NONE in this program (a MotorGroup needs every member "
                    "constructed; one refused adapter refuses the graph) -- shulib Drive (slot 1) "
                    "is the tolerant program until R3d");
    std::snprintf(line, sizeof line,
                  "facade alive: strafeAuthority=%.2f (tank: 0.00); stall check canDetectStall=%s; "
                  "budgets lin %.0f in/s ang %.2f rad/s wheel %.0f in/s; ff kS %.2f kV %.4f kA %.4f",
                  r.chassis.strafeAuthority(), "false (by config)",
                  r.chassis.motionConfig().maxLinearSpeed.value(),
                  r.chassis.motionConfig().maxAngularSpeed.value(),
                  r.chassis.motionConfig().maxWheelSpeed.value(), r.chassis.motionConfig().wheelFf.kS,
                  r.chassis.motionConfig().wheelFf.kV, r.chassis.motionConfig().wheelFf.kA);
    r.telemetry.log(shulib::hal::LogLevel::Info, "R3B", line);
#else
    Robot& r = robot();
    g_faults = &r.faults;
    r.telemetry.log(shulib::hal::LogLevel::Info, "R3B", kVariantBeacon);

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

    // The absent-device announcement (R3b §6.4, T5's no-SD-card precedent): construction
    // succeeded and the degradation is honest, so the composition root owns saying it out
    // loud, ONCE — an absent camera otherwise reads exactly like one that never sees a tag.
    r.telemetry.log(shulib::hal::LogLevel::Info, "R3B",
                    "absent devices (deliberate): tags=ABSENT vision=ABSENT -- no camera is "
                    "installed; no corrector or vision task is wired over an absent source");

    // One live query THROUGH the facade, so the banner is evidence of a working
    // object graph rather than prose: strafeAuthority() reads Chassis →
    // kinematics (F5) and must be 1.00 on this X-drive.
    char line[64];
    std::snprintf(line, sizeof line, "facade alive: strafeAuthority=%.2f (X-drive: 1.00)",
                  r.chassis.strafeAuthority());
    r.telemetry.log(shulib::hal::LogLevel::Info, "R1A", line);
#endif  // SHULIB_RUNS_BENCH_TESTER / SHULIB_RUNS_TANK_LIBRARY
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol.
 */
void disabled() {
#if SHULIB_STATE_SCREENS
    benchStateScreen("DISABLED",
                     "VEXos reports the robot DISABLED, so the driver task is not running.");
#endif
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch.
 */
void competition_initialize() {
#if SHULIB_STATE_SCREENS
    benchStateScreen("FIELD CONTROL CONNECTED",
                     "A competition switch or field is connected through the controller.");
#endif
}

/**
 * Runs the user autonomous code.
 *
 * Deliberately commands NO motion, still: the adapters exist (R1a) but the
 * stack is hardware-UNVALIDATED — no bench session has confirmed sign
 * conventions on the library path, and every gain is provisional. An
 * unvalidated auton driving blind on a field is indistinguishable from a
 * runaway. The v2 autons exist and settle in host sim
 * (test/chassis_routine_test.cpp chains them through this same facade); R3
 * wires them here after the bench runbook settles the R3 register group.
 */
void autonomous() {
#if SHULIB_RUNS_BENCH_TESTER
    benchStateScreen("AUTONOMOUS",
                     "No motion here, ever: the tester's DRIVE station lives under driver control.");
    return;
#elif SHULIB_RUNS_TANK_LIBRARY
    benchStateScreen("AUTONOMOUS",
                     "No motion here yet: the library teleop drives only under driver control.");
    if (tanklib::g_tank != nullptr) {
        tanklib::g_tank->telemetry.log(shulib::hal::LogLevel::Warn, "R3B",
                                       "autonomous(): no motion until the teleop loop has been "
                                       "validated on hardware and the gains measured (R5)");
    }
    return;
#else
    Robot& r = robot();
    r.telemetry.log(shulib::hal::LogLevel::Warn, "R1A",
                    "autonomous(): no motion until R3 validates the stack on hardware "
                    "(adapters landed at R1a; the register's R3 group is unsettled)");
#endif
}

/**
 * Runs the operator control code.
 *
 * The R1a teleop loop (runTeleopLoop above): master-controller sticks →
 * body-frame chassis.drive() at the tick cadence. Mapping (HA-112, invented,
 * T2 owns refinement) is shulib::teleop::mapSticksToChassisSpeeds — the
 * PROS-free, host-tested function this loop used to carry inline, bit-identical
 * (test/stick_mapping_test.cpp pins it against the old code): left stick =
 * translation (up = +X forward, LEFT-pushed stick = +Y left), right stick X =
 * yaw (right-pushed = clockwise = −ω). A disconnected controller commands zero
 * twist — isConnected() is the positive validity signal that distinguishes a
 * dropped controller from centred sticks (HA-103).
 */
void opcontrol() {
#if SHULIB_RUNS_BENCH_TESTER
#if defined(SHULIB_PROGRAM_DRIVE)
    // "shulib Drive" (R3b Part 0b): NO chooser -- this build IS the drive program, by the team
    // lead's ruling (a program picked by name from the slot list must do what its name says).
    // Returns on a field DISABLE (its loop polls for it, so its all-stop guard runs before
    // PROS tears the task down); PROS calls opcontrol() again when driver control resumes.
    shulib::bench::runDrive();
    return;
#else
    shulib::bench::runR3a();  // never returns; powers motors ONLY in its DRIVE station
    return;
#endif
#elif SHULIB_RUNS_TANK_LIBRARY
    using namespace tanklib;
    if (g_tank == nullptr) {
        // Refused at boot: no adapter exists, nothing is powered. Keep the reason on screen
        // (a state screen may have replaced it) and idle.
        paintRefusal();
        while (true) {
            pros::delay(250);
        }
    }
    TankRobot& r = *g_tank;
    r.telemetry.log(shulib::hal::LogLevel::Info, "R3B",
                    "opcontrol(): the LIBRARY's teleop loop is live on robot two -- "
                    "Chassis::drive(Body) over two MotorGroups; deadband-only mapping (HA-112)");
    drawHeader();
    TankPanel panel{r};
    runTeleopLoop(r.chassis, r.master, r.pacer, panel);
    r.telemetry.log(shulib::hal::LogLevel::Info, "R3B",
                    "opcontrol(): field DISABLED -- loop exited cleanly after cancel() (0 V + Brake)");
#else
    Robot& r = robot();
    r.telemetry.log(shulib::hal::LogLevel::Info, "R1A",
                    "opcontrol(): teleop drive loop live (deadband-only mapping, HA-112, "
                    "shulib/teleop/stick_mapping.hpp; driver-feel shaping is chunk T2)");
    runTeleopLoop(r.chassis, r.master, r.pacer, [&](const shulib::math::ChassisSpeeds&, bool) {
        r.faultDisplay.update(r.clock.now());
    });
#endif  // SHULIB_RUNS_BENCH_TESTER / SHULIB_RUNS_TANK_LIBRARY
}
