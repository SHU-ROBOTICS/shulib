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
#if (defined(SHULIB_PROGRAM_DRIVE) + defined(SHULIB_PROGRAM_LIBRARY) \
     + defined(SHULIB_PROGRAM_CALIBRATE)) > 1
#error "two PROGRAM defines are set at once -- select exactly one with make PROGRAM=..."
#endif
#if defined(SHULIB_PROGRAM_DRIVE) && !defined(SHULIB_ROBOT_TANK_2026)
#error "PROGRAM=drive needs ROBOT=tank -- only robot two's chassis table carries measured signs"
#endif
#if defined(SHULIB_PROGRAM_LIBRARY) && !defined(SHULIB_ROBOT_TANK_2026)
#error "PROGRAM=library needs ROBOT=tank -- only robot two's chassis table is signed, measured, and carries geometry"
#endif
#if defined(SHULIB_PROGRAM_CALIBRATE) && !defined(SHULIB_ROBOT_TANK_2026)
#error "PROGRAM=calibrate needs ROBOT=tank -- the calibrator supports only a tank drivetrain and takes the cartridge from robot two's chassis table"
#endif
#if defined(SHULIB_ROBOT_XDRIVE_INVENTED)
#define SHULIB_RUNS_BENCH_TESTER 0
#define SHULIB_RUNS_TANK_LIBRARY 0
#define SHULIB_RUNS_CALIBRATE 0
#elif defined(SHULIB_PROGRAM_LIBRARY)
#define SHULIB_RUNS_BENCH_TESTER 0
#define SHULIB_RUNS_TANK_LIBRARY 1  // robot two, driven by the LIBRARY (the tank graph below)
#define SHULIB_RUNS_CALIBRATE 0
#elif defined(SHULIB_PROGRAM_CALIBRATE)
#define SHULIB_RUNS_BENCH_TESTER 0
#define SHULIB_RUNS_TANK_LIBRARY 0
#define SHULIB_RUNS_CALIBRATE 1  // robot two MEASURES ITSELF (src/calibrate_program.cpp, R3d P2)
#else
#define SHULIB_RUNS_BENCH_TESTER 1  // bench AND tank: a tester-variant robot (no library graph);
                                    // which PROGRAM runs in opcontrol() is the second axis
#define SHULIB_RUNS_TANK_LIBRARY 0
#define SHULIB_RUNS_CALIBRATE 0
#endif
// The competition-state screens exist for every program that is NOT the invented X-drive
// (which the src build gate expects to emit no unused-function warning from this file).
// "shulib Calibrate" paints its OWN not-driver-control screen from inside its loop (it powers
// motors and must say so in its own words), but it still needs these for disabled() and
// competition_initialize(), which run before and after its loop exists.
#define SHULIB_STATE_SCREENS \
    (SHULIB_RUNS_BENCH_TESTER || SHULIB_RUNS_TANK_LIBRARY || SHULIB_RUNS_CALIBRATE)

#if SHULIB_RUNS_BENCH_TESTER
#include "bench_r3a.hpp"
#if defined(SHULIB_PROGRAM_DRIVE)
#include "drive_program.hpp"
#endif
#endif
#if SHULIB_RUNS_CALIBRATE
#include "calibrate_program.hpp"
#include "shulib/calibration/calibration_record.hpp"  // kRecordFileName, for the boot banner
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
#include "shulib/calibration/boot_check.hpp"
#include "shulib/calibration/calibration_record.hpp"
#include "shulib/hal/absent_gps.hpp"
#include "shulib/hal/device_registry.hpp"
#include "shulib/hal/drive_geometry.hpp"
#include "shulib/hal/drivetrain_description.hpp"
#include "shulib/hal/motor_group.hpp"
#include "shulib/hal/pros/block_source.hpp"
#include "shulib/hal/pros/device_registry.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/localization/drive_encoder_odometry.hpp"
#include "shulib/motion/odo_stall_check.hpp"
#endif

namespace {

// ═══ THE VARIANT IDENTITY BEACON (R3b Session 2; asserted by tools/src_build_gate.py) ═══
// One string per BUILD, selected by the SAME preprocessor facts that select the wiring and
// the program, and printed at boot so it is referenced and survives into the linked package.
// The src build gate asserts the expected beacon IS in bin/hot.package.elf and the other
// five are NOT (matched WITH the string's NUL terminator, because `…=tank` is a prefix of
// `…=tank-drive`, `…=tank-library` and `…=tank-calibrate`). That is the end-to-end proof that the #if chain
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
#elif defined(SHULIB_ROBOT_TANK_2026) && defined(SHULIB_PROGRAM_CALIBRATE)
constexpr const char kVariantBeacon[] = "shulib-robot-variant=tank-calibrate";
/// The BUILD stamp for the calibrate banner. The git hash names the COMMIT; this names the
/// BUILD, and only the second one answers "did my upload actually land?" (the tester's
/// reasoning, 2026-08-18). src/calibrate_program.cpp carries its own copy for its own screen;
/// both are __DATE__ __TIME__ of their own translation unit, so a partial rebuild that left one
/// TU stale would show two different stamps -- which is information, not a defect.
constexpr const char kCalibrateBuildStamp[] = __DATE__ " " __TIME__;
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
// ═══ ROBOT TWO'S OBJECT GRAPH — "shulib Teleop" (R3b Parts 1–3; rebuilt on the DRIVETRAIN
//     DESCRIPTION at R3d Part 2, 2026-09-18) ═════════════════════════════════════════
// Built ENTIRELY from ONE hal::DrivetrainDescription: N ProsMotors from its SIGNED ports (the
// sign lives inside the port entry and is applied by PROS once, in the adapter — the group
// never negates), two hal::MotorGroups in the kinematics' wheel order (left, right), ProsImu
// on its IMU port, the explicit absent devices, ProsBattery, the diagnostics stack exactly as
// the X-drive graph has it, DriveEncoderOdometry from the two groups + the IMU + the
// description's two geometries, ComplementaryFusion, Localizer, RobotContext, MotionDeps (with
// the groups, so MOTOR_GROUP_DISAGREE fires in drive()), ProsTickPacer, Chassis.
//
// ── WHERE THE DESCRIPTION COMES FROM (R3d Part 2, rulings P2/P3) ────────────────────
// TWO sources, one representation, and a fixed order that is not configurable:
//   1. the CALIBRATION RECORD on the SD card (/usd/shulib_drivetrain.bin), written by
//      "shulib Calibrate" after the robot measured itself. It is VERIFIED against the brain's
//      registry on the SAME boot that reads it, BEFORE any adapter is constructed
//      (calibration/boot_check.hpp): a port that now carries a different DEVICE means the
//      robot was rewired and REFUSES; ports that are simply gone are tolerated by the shared
//      dead-port policy and become ABSENT group members.
//   2. else src/chassis_table.hpp's typed table, through tableToDescription() — which is the
//      ONLY path from the table into this graph. It is used with a WARNING, because it was
//      typed from a build that may no longer exist, and it is registry-verified too.
//   3. else refuse, naming BOTH reasons.
// A record ALWAYS beats a table (it was produced by moving THIS robot); a record that fails
// its port check NEVER falls back to the table (the table is older still). calibration::
// chooseBootSource is that decision, as a host-tested table rather than an if-chain here.
//
// BOOT REFUSAL, NOT FAULT-ABORT: initialize() refuses BEFORE any adapter is constructed and
// paints the screen with exactly what is missing; if a ProsMotor (or the IMU) constructor
// throws, the graph boundary catches it and refuses the same way. opcontrol() then idles: no
// adapter exists, nothing is powered.
//
// DEAD PORTS ARE NOW TOLERATED HERE, at BOOT (R3d Part 2, ruling P4): a recorded drive port
// the registry reports EMPTY becomes a `nullptr` slot in its group's span, under the SAME
// pure policy "shulib Drive" uses (teleop::evaluateDegradation — a side below three answering,
// or more than two dead in total, refuses). The panel says `DEGRADED: dead <ports>`. What is
// still NOT tolerated is a port dying AFTER boot inside this graph: that is a later chunk, and
// "shulib Drive" (slot 1) remains the program with runtime absence detection.
//
// EXPECT IT TO FEEL DIFFERENT from "shulib Drive", and the panel says so: the pipeline
// commands wheel SPEEDS through feedforward with PROVISIONAL gains (HA-45: kV = 12/70 V per
// in/s, a placeholder), capped by MotionConfig::maxLinearSpeed / maxWheelSpeed, plus battery
// compensation. If it is slow or saturates, that is R5's measurement showing up, not a bug;
// the panel prints the budgets and the gains in use.
namespace tanklib {

namespace calib = shulib::calibration;

constexpr const char* kBuildStamp = __DATE__ " " __TIME__;
constexpr int kPanelEveryTicks = 10;  // 100 ms
constexpr int kLcdEveryTicks = 20;    // 200 ms = the 5 Hz ceiling; writes only on change
constexpr std::int16_t kUsableW = 480;
constexpr std::uint32_t kColBg = 0x101418, kColBar = 0x1E5AA8, kColText = 0xF0F0F0,
                        kColDim = 0x9AA4AE, kColGood = 0x39C36E, kColWarn = 0xE8B23A,
                        kColBad = 0xE2564A, kColSub = 0xC8D8F0;

shulib::hal::pros::MotorGearset gearsetOf(shulib::hal::MotorCartridge c) {
    switch (c) {
        case shulib::hal::MotorCartridge::Red: return shulib::hal::pros::MotorGearset::Red;
        case shulib::hal::MotorCartridge::Green: return shulib::hal::pros::MotorGearset::Green;
        default: return shulib::hal::pros::MotorGearset::Blue;  // refused before this if unset
    }
}

// ── THE CHOSEN DESCRIPTION, decided in initialize() before anything is constructed ───
// File-scope storage rather than a member, because TankRobot takes it BY REFERENCE and the
// boot sequence has to finish deciding before the graph's constructor runs.
shulib::hal::DrivetrainDescription g_description{};
calib::BootVerdict g_verdict{};
calib::BootSource g_source = calib::BootSource::Refuse;
const char* g_sourceWhy = "";
bool g_recordDecoded = false;
char g_recordWhy[192] = "";

/// The N adapters, constructed from the DESCRIPTION in ONE place, and the two pointer arrays
/// the groups view. A port the boot check found MISSING becomes a `nullptr` slot: the group
/// skips it, excludes it from every read, and reports it absent (hal/motor_group.hpp, ruling
/// P4). A throwing ProsMotor constructor propagates out of here — and out of TankRobot — to
/// initialize()'s catch: the refusal path.
struct TankMotors {
    std::optional<shulib::hal::pros::ProsMotor> storage[shulib::hal::kMaxSmartPorts];
    shulib::hal::IMotor* left[shulib::hal::kMaxSmartPorts] = {};
    shulib::hal::IMotor* right[shulib::hal::kMaxSmartPorts] = {};
    std::size_t leftN = 0;
    std::size_t rightN = 0;
    std::size_t absentCount = 0;

    TankMotors(const shulib::hal::DrivetrainDescription& d, const calib::BootVerdict& verdict,
               shulib::hal::pros::MotorGearset gearset) {
        std::size_t k = 0;
        const shulib::hal::DriveSideDescription* sides[2] = {&d.left, &d.right};
        shulib::hal::IMotor** out[2] = {left, right};
        std::size_t* n[2] = {&leftN, &rightN};
        for (int sideIndex = 0; sideIndex < 2; ++sideIndex) {
            const shulib::hal::DriveSideDescription& side = *sides[sideIndex];
            for (std::size_t i = 0; i < side.count; ++i, ++k) {
                const std::int8_t signedPort = side.signedPorts[i];
                const int port = shulib::hal::DrivetrainDescription::portMagnitude(signedPort);
                if (verdict.isMissing(port)) {
                    // A DEAD PORT: no adapter is constructed (its ctor would throw) and the
                    // slot stays a hole. The slot is KEPT so the panel's indices do not shift.
                    out[sideIndex][(*n[sideIndex])++] = nullptr;
                    ++absentCount;
                    continue;
                }
                storage[k].emplace(signedPort, gearset);  // AS STORED: the one place a sign lives
                out[sideIndex][(*n[sideIndex])++] = &*storage[k];
            }
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
/// wheel slot carries the drivetrain's geometry (the SAME object the odometry converts with),
/// and there is NO independent motion source (the §2 ruling: odo_stall_check.hpp).
shulib::chassis::ChassisConfig tankChassisConfig(shulib::hal::DriveGeometry geometry) {
    shulib::chassis::ChassisConfig cfg{};
    cfg.motion.stall.setAllWheels(geometry);
    cfg.motion.stall.independentMotionSource = false;
    return cfg;
}

struct TankRobot {
    /// THE ONE INPUT. Everything below is derived from it; nothing here reads the chassis
    /// table (landmine 1: two sources inside one graph is how a record and a table come to
    /// disagree about the same robot).
    const shulib::hal::DrivetrainDescription& desc = g_description;
    // ── drivetrain: config DATA from the description. The track width is MEASURED (from a
    //    powered spin, effectively larger than a tape's — HA-135) or typed; either way it
    //    arrives here as one number and is never retyped.
    shulib::kinematics::TankKinematics kin{desc.trackWidth};
    shulib::hal::pros::ProsClock clock{};
    // ── the adapters, then ONE IMotor per kinematic wheel: the groups (R3b Part 1), with a
    //    hole where a recorded port is dead (R3d Part 2).
    TankMotors motors{desc, g_verdict, gearsetOf(desc.cartridge)};
    shulib::hal::MotorGroup left{motors.leftSpan()};
    shulib::hal::MotorGroup right{motors.rightSpan()};
    shulib::hal::IMotor* driveMotors[2] = {&left, &right};  // TankKinematics: 0 = left, 1 = right
    shulib::hal::MotorGroup* groups[2] = {&left, &right};
    // ── sensors: the IMU on the description's port (bootHeading 0: the robot boots facing
    //    +X); everything else ABSENT, explicitly (R3b §6 — never polled, announced once).
    shulib::hal::pros::ProsImu imu{desc.imuPort, shulib::math::Angle{}, clock};
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
    // ── TWO drive geometries, one per side, straight from the description. A calibration
    //    measures each side separately, and an asymmetric drivetrain is representable; the
    //    stall check takes the LEFT one because it wants a single number per wheel slot and
    //    the two agree on any symmetric robot (the boot log prints both, so a disagreement is
    //    visible rather than averaged away).
    shulib::hal::DriveGeometry leftGeometry = desc.leftGeometry();
    shulib::hal::DriveGeometry rightGeometry = desc.rightGeometry();
    // ── localization: drive-encoder odometry over the two groups (R3b Part 2), fused.
    shulib::localization::DriveEncoderOdometry odom{imu,        left,           right,
                                                    leftGeometry, rightGeometry, desc.trackWidth};
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
    shulib::chassis::Chassis chassis{deps, pacer, tankChassisConfig(leftGeometry)};
};

/// Constructed in initialize() inside the graph-boundary try; a throwing adapter leaves it
/// unconstructed and g_tank null (the refusal path). Function-local static, like robot().
TankRobot& tankRobot() {
    static TankRobot r;
    return r;
}
TankRobot* g_tank = nullptr;
char g_refusalHeadline[64] = "";
char g_refusalWhy[192] = "";

/// The §18.5 port-map string, built from the DESCRIPTION — never retyped, and carrying the
/// SOURCE word first, because "a robot measured this" and "a person typed this off a tape"
/// are not the same claim and the session header must be able to say which it is looking at.
const char* tankPortMapString() {
    static char buf[shulib::diag::kMaxPortMapBytes];
    constexpr std::size_t kSideCap = 40;
    char l[kSideCap] = "";
    char r[kSideCap] = "";
    const shulib::hal::DriveSideDescription* sides[2] = {&g_description.left,
                                                         &g_description.right};
    char* out[2] = {l, r};
    // Appended BY HAND rather than with a moving snprintf: the compiler cannot bound a
    // `cap - strlen(dst)` argument, so -Wformat-truncation (which the src build gate treats as
    // a disallowed warning) fires on a call that is in fact safe. An explicit bound is both
    // safe and provably so.
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
    // Assembled in a GENEROUS local buffer and then bounded into the session header's fixed
    // field, rather than formatted straight into it: a 21-port-per-side description would
    // overrun 96 bytes, the compiler cannot bound a float conversion, and -Wformat-truncation
    // is a disallowed warning in the src build gate. One bounded copy is provably right.
    // (emitSessionHeader sanitizes and truncates on top of this, at the same width.)
    static_assert(shulib::diag::kMaxPortMapBytes >= 32, "the port map field must fit something");
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
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 84, "%.58s", g_refusalWhy);
    if (std::strlen(g_refusalWhy) > 58) {
        pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 100, "%.58s", g_refusalWhy + 58);
    }
    if (std::strlen(g_refusalWhy) > 116) {
        pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 116, "%.58s", g_refusalWhy + 116);
    }
    pros::c::screen_set_pen(kColDim);
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 140,
                             "The library graph did NOT construct. Nothing is powered.");
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 156,
                             "Run shulib Calibrate (slot 4): the robot measures itself and");
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 172,
                             "writes the answer to the SD card. This program boots from it.");
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 192,
                             "An adapter refusal: Bench Tests (slot 3) diagnoses it.");
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 212, "BUILD %s", kBuildStamp);
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
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 8, 20,
                             "THE LIBRARY drives: Chassis::drive() -- NEVER RUN BEFORE");
    pros::c::screen_print_at(pros::E_TEXT_SMALL, 300, 20, "BUILD %s", kBuildStamp);
    pros::c::screen_set_eraser(kColBg);
}

/// The per-tick panel + LCD for the library teleop: session essentials, the estimate and its
/// quality class, the two group commands, each group's PRESENT count / disagreeing count, the
/// heading cross-check, battery, any latched fault by name, the budgets and gains in use, the
/// honest notes, and — since R3d Part 2 — WHERE THE DESCRIPTION CAME FROM and which ports are
/// dead. LCD: `LIB 12V Bxx.xV` / `hdg ±ddd.d deg` / last fault or `ok`.
struct TankPanel {
    TankRobot& r;
    int tick = 0;
    char lcdRow[3][24] = {"", "", ""};

    void operator()(const shulib::math::ChassisSpeeds& command, bool connected) {
        ++tick;
        char line[120];
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
            std::snprintf(line, sizeof line,
                          "L %+5.1fV %u/%u present %d disagree | R %+5.1fV %u/%u present %d disagree",
                          r.left.commandedVoltage().value(),
                          static_cast<unsigned>(r.left.presentCount()),
                          static_cast<unsigned>(r.left.memberCount()),
                          r.left.disagreeingMembers(), r.right.commandedVoltage().value(),
                          static_cast<unsigned>(r.right.presentCount()),
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
            // WHERE THE NUMBERS CAME FROM, and what is dead. A typed table on a rebuilt robot
            // is a belief, not a measurement, and the driver must be able to see which one is
            // driving them.
            if (g_source == calib::BootSource::UseTable) {
                rowText(182, kColWarn,
                        "TYPED TABLE -- unverified since the rebuild; run shulib Calibrate");
            } else if (g_verdict.decision == calib::BootDecision::Degraded) {
                std::snprintf(line, sizeof line, "MEASURED record | %.70s", g_verdict.reason);
                rowText(182, kColWarn, line);
            } else {
                rowText(182, kColGood,
                        "MEASURED record from the SD card, verified against the ports");
            }
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
                          : g_verdict.decision == calib::BootDecision::Degraded ? "DEGRADED see brain"
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
#elif SHULIB_RUNS_CALIBRATE
    // "shulib Calibrate" (R3d Part 2): NO object graph is constructed here. The program finds
    // its own motors from the brain's registry when opcontrol() runs it, because a calibration
    // that starts from a typed port list can only ever CONFIRM the typing. This only proves the
    // binary booted and says WHICH build it is (the beacon the src build gate asserts).
    std::printf("\n[R3D] booted: shulib CALIBRATE build [%s].\n"
                "[R3D] robot two MEASURES ITS OWN DRIVETRAIN -- which ports drive, which way each\n"
                "[R3D] is wired, which side each is on, inches per radian per side, and the\n"
                "[R3D] EFFECTIVE track width -- then writes them to /usd/%s.\n"
                "[R3D] It POWERS THE MOTORS. THIS VERSION (R3d Part 2F) first ran on robot two on\n"
                "[R3D] 2026-09-24 -- run 1 STOPPED on SPIN_DISAGREE, run 2 finished (unrecorded);\n"
                "[R3D] the Part 2 version finished none of five starts. Every threshold in it is\n"
                "[R3D] INVENTED (A4 register HA-136).\n"
                "[R3D] no object graph is constructed at boot; the X-drive wiring is NOT built.\n"
                "[R3D] BUILD STAMP %s\n",
                kVariantBeacon, shulib::calibration::kRecordFileName, kCalibrateBuildStamp);
    {
        char bits[64];
        std::printf("[R3D] competition status at boot: %s -- calibration runs only in DRIVER "
                    "CONTROL, enabled; any other state stops the run and says so.\n",
                    competitionBits(pros::c::competition_get_status(), bits, sizeof bits));
    }
    std::fflush(stdout);
    return;
#elif SHULIB_RUNS_TANK_LIBRARY
    using namespace tanklib;
    std::printf("\n[R3B] booted: shulib TELEOP build [%s] -- THE LIBRARY drives robot two through\n"
                "[R3B] Chassis::drive() over two hal::MotorGroups and drive-encoder odometry, built\n"
                "[R3B] from the CALIBRATION RECORD on the SD card when there is one, else from\n"
                "[R3B] src/chassis_table.hpp. Not yet run on hardware before this boot: the first\n"
                "[R3B] run is wheels-up. The invented X-drive graph is NOT constructed.\n"
                "[R3B] BUILD STAMP %s\n",
                kVariantBeacon, kBuildStamp);
    {
        char bits[64];
        std::printf("[R3B] competition status at boot: %s -- the loop runs only in DRIVER CONTROL, "
                    "enabled; any other state paints the screen with the reason.\n",
                    competitionBits(pros::c::competition_get_status(), bits, sizeof bits));
    }
    std::fflush(stdout);

    // ═══ WHICH DESCRIPTION DRIVES THIS ROBOT (R3d Part 2) ════════════════════════════
    // Order fixed, not configurable (ruling P3): the card's MEASURED record, else the typed
    // table, else refuse. Everything below happens BEFORE a single adapter is constructed
    // (ruling P2), so a rewired robot is refused rather than driven.
    {
        // 1. the record on the card. Read into a buffer LARGER than a record on purpose: a
        //    longer file must show up as a LENGTH refusal, not as "the first 180 bytes fitted".
        std::byte raw[shulib::calibration::kRecordBytes + 32]{};
        std::size_t got = 0;
        {
            shulib::hal::pros::ProsBlockSource card{shulib::calibration::kRecordFileName};
            if (!card.isOpen()) {
                std::snprintf(g_recordWhy, sizeof g_recordWhy,
                              "no calibration record: %s could not be opened (no SD card, or the "
                              "robot has never been calibrated)",
                              card.path());
            } else {
                got = card.readAll(raw);
            }
        }
        if (got > 0) {
            g_recordDecoded = shulib::calibration::decodeRecord(
                std::span<const std::byte>{raw, got}, g_description, g_recordWhy,
                sizeof g_recordWhy);
        }
        // 2. verify it against the ports, on THIS boot, before anything is constructed.
        const shulib::hal::pros::ProsDeviceRegistry registry;
        if (g_recordDecoded) {
            g_verdict = shulib::calibration::verifyAgainstRegistry(g_description, registry);
            std::printf("[R3B] calibration record: DECODED, port check %s -- %s\n",
                        shulib::calibration::bootDecisionWord(g_verdict.decision),
                        g_verdict.reason);
        } else {
            std::printf("[R3B] calibration record: NOT USED -- %s\n", g_recordWhy);
        }
        // 3. the table as the second source. tableToDescription() is the ONLY path from the
        //    table into this graph (landmine 1); nothing below reads kChassis for a number.
        const shulib::hal::DrivetrainDescription fromTable =
            shulib::bench::tableToDescription(shulib::bench::kChassis);
        char tableWhy[192];
        const bool tableValid = fromTable.valid(tableWhy, sizeof tableWhy);
        if (!tableValid) {
            char missing[160];
            (void)shulib::bench::describeMissingForDescription(shulib::bench::kChassis, missing,
                                                               sizeof missing);
            std::printf("[R3B] typed table: cannot describe a drivetrain -- UNSET: %s (%s)\n",
                        missing, tableWhy);
        }
        // 4. the decision, as a host-tested table rather than an if-chain here.
        const shulib::calibration::BootSourceChoice choice =
            shulib::calibration::chooseBootSource(g_recordDecoded, g_verdict.decision, tableValid);
        g_source = choice.source;
        g_sourceWhy = choice.why;
        std::printf("[R3B] boot source: %s -- %s\n",
                    g_source == shulib::calibration::BootSource::UseRecord  ? "RECORD"
                    : g_source == shulib::calibration::BootSource::UseTable ? "TYPED TABLE"
                                                                            : "REFUSE",
                    g_sourceWhy);
        if (g_source == shulib::calibration::BootSource::UseTable) {
            g_description = fromTable;
            g_verdict = shulib::calibration::verifyAgainstRegistry(g_description, registry);
            std::printf("[R3B] WARNING: driving from the TYPED TABLE -- unverified since the "
                        "rebuild; run shulib Calibrate (slot 4). Port check %s -- %s\n",
                        shulib::calibration::bootDecisionWord(g_verdict.decision),
                        g_verdict.reason);
            if (g_verdict.decision == shulib::calibration::BootDecision::Refuse) {
                refuse("the TYPED TABLE does not match the ports", g_verdict.reason);
                return;
            }
        } else if (g_source == shulib::calibration::BootSource::Refuse) {
            char both[192];
            std::snprintf(both, sizeof both, "%.80s | %.100s", g_recordWhy,
                          g_recordDecoded ? g_verdict.reason : tableWhy);
            refuse("NO USABLE DRIVETRAIN DESCRIPTION", both);
            return;
        }
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
    //    signs and the geometry all come from the DESCRIPTION, never retyped.
    shulib::diag::SessionInfo session{};
    session.buildHash = shulib::diag::compiledBuildHash();
    session.routineId = "(none: teleop only; autons land later)";
    session.alliance = "";
    session.side = "";
    session.portMap = tankPortMapString();
    shulib::diag::emitSessionHeader(r.telemetry, session, r.battery.voltage());

    char line[240];
    {
        char described[128];
        r.desc.describe(described, sizeof described);
        std::snprintf(line, sizeof line, "library graph BUILT from the %s: %.120s",
                      g_source == shulib::calibration::BootSource::UseRecord
                          ? "CALIBRATION RECORD on the SD card"
                          : "TYPED TABLE (src/chassis_table.hpp)",
                      described);
        r.telemetry.log(shulib::hal::LogLevel::Info, "R3B", line);
        std::snprintf(line, sizeof line,
                      "%u+%u slots behind two MotorGroups, %u+%u PRESENT -- median reads are over "
                      "the PRESENT members; the sign lives inside each port entry and PROS "
                      "applies it once",
                      static_cast<unsigned>(r.left.memberCount()),
                      static_cast<unsigned>(r.right.memberCount()),
                      static_cast<unsigned>(r.left.presentCount()),
                      static_cast<unsigned>(r.right.presentCount()));
        r.telemetry.log(shulib::hal::LogLevel::Info, "R3B", line);
        std::snprintf(line, sizeof line, "description provenance: %.190s", r.desc.provenance);
        r.telemetry.log(shulib::hal::LogLevel::Info, "R3B", line);
        std::snprintf(line, sizeof line, "boot source: %.200s", g_sourceWhy);
        r.telemetry.log(g_source == shulib::calibration::BootSource::UseRecord
                            ? shulib::hal::LogLevel::Info
                            : shulib::hal::LogLevel::Warn,
                        "R3B", line);
        std::snprintf(line, sizeof line, "port check: %.220s", g_verdict.reason);
        r.telemetry.log(g_verdict.decision == shulib::calibration::BootDecision::Ok
                            ? shulib::hal::LogLevel::Info
                            : shulib::hal::LogLevel::Warn,
                        "R3B", line);
        // The chassis table's CROSS-CHECK, and the only other place this graph may read it
        // (landmine 1): if the card's record and the typed table disagree about which ports
        // drive, say so. A disagreement is not an error — the record measured THIS robot and
        // the table was typed from a build that may no longer exist — but it is exactly the
        // thing a person should be told before driving.
        if (g_source == shulib::calibration::BootSource::UseRecord) {
            const shulib::hal::DrivetrainDescription fromTable =
                shulib::bench::tableToDescription(shulib::bench::kChassis);
            int differing = 0;
            for (int p = 1; p <= static_cast<int>(shulib::hal::kMaxSmartPorts); ++p) {
                if (r.desc.signedPortOf(p) != fromTable.signedPortOf(p)) {
                    ++differing;
                }
            }
            if (differing > 0) {
                std::snprintf(line, sizeof line,
                              "CROSS-CHECK: the MEASURED record and the typed chassis table "
                              "disagree about %d port(s). The record wins (it moved THIS robot); "
                              "the table is stale and should be updated or deleted.",
                              differing);
                r.telemetry.log(shulib::hal::LogLevel::Warn, "R3B", line);
            }
        }
    }
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
    if (r.motors.absentCount > 0) {
        std::snprintf(line, sizeof line,
                      "DEGRADED: %u recorded drive port(s) are EMPTY and were NOT constructed; "
                      "their group slots are holes and every read is over the PRESENT members. "
                      "%.80s",
                      static_cast<unsigned>(r.motors.absentCount), g_verdict.reason);
        r.telemetry.log(shulib::hal::LogLevel::Warn, "R3B", line);
    }
    r.telemetry.log(shulib::hal::LogLevel::Info, "R3B",
                    "dead-port tolerance: AT BOOT ONLY (R3d Part 2) -- a recorded port the "
                    "registry reports EMPTY becomes an absent group member under the same policy "
                    "shulib Drive uses. A port that dies AFTER boot is still shulib Drive's (slot "
                    "1): runtime absence detection is not in this graph yet");
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
#elif SHULIB_RUNS_CALIBRATE
    benchStateScreen("AUTONOMOUS",
                     "No motion here, ever: calibration powers the motors and lives under driver "
                     "control, where a person is standing by the battery.");
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
#if SHULIB_RUNS_CALIBRATE
    // "shulib Calibrate" (R3d Part 2): NO chooser -- this build IS the calibrate program, by the
    // team lead's ruling (a program picked by name from the slot list must do what its name
    // says). It runs its own 10 ms loop, exits cleanly on a field DISABLE (its all-stop guard
    // runs before PROS tears the task down), and idles on its result screen so an UNSAVED
    // result is not thrown away by the program ending.
    shulib::bench::runCalibrate();
    return;
#elif SHULIB_RUNS_BENCH_TESTER
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
