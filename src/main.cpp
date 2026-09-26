// main.cpp — the PROS entry point, rewired onto the shulib v2 core (chunk C7,
// WS11/M2), with the hal/pros adapters wired in at chunk R1a, and — since R3b
// Parts 1–3 (2026-09-14) — the LIBRARY'S OWN TELEOP LOOP FOR ROBOT TWO, which since
// G1 Part 2b (2026-09-26) lives in src/teleop_program.cpp, built on robot::Robot: this
// file's tank-library branches only forward to it. (G1 Part 2c makes the whole file a
// dispatcher; until then the invented X-drive graph below stays here.)
//
// ═══ READ THIS FIRST: what this binary is, and what it is NOT ═══════════════════════
// Since R1a every HAL seam below is a REAL hal/pros adapter over a physical V5
// device: motors, IMU, GPS, battery, rotation sensors, controller, USB serial,
// controller LCD, real time, and the tick-boundary pacer. The fakes are gone
// from this file ENTIRELY — since R3b even vision/tags, which are now explicit
// ABSENT devices (hal/absent_*.hpp), not test doubles.
//
// WHAT THAT DOES **NOT** MEAN — the governing constraint:
//   * NO LIBRARY-COMMANDED MOTION HAS RUN ON A ROBOT (R3b open). The adapters are
//     host-tested against a hand-written shim of PROS — which tests the adapter
//     against our BELIEF about PROS, never the belief itself. The beliefs are
//     inventoried in docs/hardware-assumptions.md (HA-94 onward) and settle on
//     a bench, not in a test suite. The PREVIOUS "shulib Teleop" (build
//     v0.1.1-319-g2780462) drove robot two from the calibration record on
//     2026-09-25 -- eight starts, open-loop under driver control through the
//     library's own graph. THIS VERSION of "shulib Teleop" (G1 Part 2b, built on
//     robot::Robot, src/teleop_program.cpp) has not run on hardware.
//   * The X-DRIVE PORT MAP BELOW IS INVENTED (A4: HA-111). No X-drive robot has
//     been measured. Uploading that variant to a brain whose devices are not on
//     exactly these ports will fault loudly at boot (the adapters read their
//     configuration back). Robot two's ports, signs, IMU port and geometry come
//     from the calibration record on its SD card (src/boot_source.{hpp,cpp}), else from
//     src/chassis_table.hpp, and are never retyped here.
//   * autonomous() commands NO motion in any build of this file: the bench tester,
//     "shulib Drive", "shulib Calibrate" and "shulib Teleop" never will (the team
//     lead's ruling, 2026-09-25: autonomous routines run only in the competition
//     program's autonomous()), and the invented X-drive cannot boot on a real robot.
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
//     both object graphs called until G1 Part 2b moved "shulib Teleop" onto the
//     library's own loop (teleop::runDriverControl, with T2's feel layer); only the
//     invented X-drive calls it now — plus one addition from Part
//     0b's finding: it polls the field DISABLE and exits cleanly (PROS deletes
//     the task on disable and a deleted task runs no destructor; VEXos cuts the
//     motors itself, and the clean exit lets the facade brake them first).
//
// ═══ Absent, deliberately (chunk R3b §6) ════════════════════════════════════════════
//   * AbsentTagSource / AbsentVision — NO camera ADAPTER exists yet (R2 owns it). Robot two
//     has carried an AI Vision camera on port 3 since 2026-09-24; no library adapter reads it
//     (Bench Tests' station 12 shows its raw output, R2a).
//     These replaced the FakeTagSource/FakeVision stubs: a test fake in a competition
//     binary is test scaffolding, and it erases the difference between "this robot has
//     no such device" and "a test will inject readings here". An absent source is never
//     polled (§6.5) — no corrector, no vision task, no false "camera alive" record —
//     and the boot log announces the absence once. R2 (camera) swaps in the real
//     adapter, visibly, on this exact line.
//   * AbsentGps — robot two has no GPS; its config says `gps.present = false` and
//     robot::Robot installs a hal::AbsentGps and no corrector (src/boot_source.hpp).
//     The invented X-drive graph keeps its ProsGps: HA-111's port map defines it
//     WITH a GPS on port 9.

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
//   make ROBOT=tank PROGRAM=library   → "shulib Teleop" (R3b Parts 1-3; rebuilt on robot::Robot
//                                       at G1 Part 2b): THE LIBRARY drives robot two --
//                                       src/teleop_program.cpp, built from the calibration record
//                                       on the SD card (else the chassis table). Slot 2.
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
#error "PROGRAM=library needs ROBOT=tank -- only robot two has a calibration record and a signed chassis table"
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
#define SHULIB_RUNS_TANK_LIBRARY 1  // robot two, driven by the LIBRARY (src/teleop_program.cpp)
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
// "shulib Teleop" lives in its own file since G1 Part 2b: robot::Robot over the PROS device
// provider, the boot source in src/boot_source.{hpp,cpp}, the panel, the driver-control loop.
#include "teleop_program.hpp"
#endif
// The precondition handler's hand-off to a graph built in ANOTHER translation unit (G1 Part 2b).
// Included and DEFINED for every build, because every src/*.cpp is compiled into every build.
#include "fault_latch_hook.hpp"

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
/// "shulib Teleop"'s graph boundary, in src/teleop_program.cpp, CATCHES it and refuses.)
/// The latch is attached by whichever graph exists: the X-drive graph below sets it directly;
/// a graph built in another translation unit ("shulib Teleop", G1 Part 2b) calls
/// shulib::bench::attachFaultLatch(), defined after this namespace for every build.
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
    //    ABSENT, deliberately (R3b §6): no camera adapter exists yet (R2), and a
    //    test fake in a competition binary erases "no device" vs "a test injects here". No
    //    corrector or vision task is wired over these — kInstallTagCorrector<AbsentTagSource>
    //    is false, so polling one would manufacture a false "camera alive, no tags" record
    //    (§6.5). The boot log announces the absence once; R2 (camera) swaps in the real
    //    adapter here. (Robot two's graph — robot::Robot, since G1 Part 2b — gets a
    //    hal::AbsentGps from its config's `gps.present = false`. THIS invented X-drive
    //    graph keeps its ProsGps: HA-111's port map defines it WITH a GPS on port 9, and
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

// ═══ THE X-DRIVE'S TELEOP LOOP (R3b Session 2 §6.2; Parts 1–3; X-drive-only since G1 Part 2b) ═══
/// Until G1 Part 2b BOTH object graphs in this file called this loop. "shulib Teleop" now runs the
/// LIBRARY's loop (teleop::runDriverControl, with the DriveShaper feel layer) from
/// src/teleop_program.cpp; the invented X-drive graph below is the only caller left, and G1 Part
/// 2c removes this function with that graph. The R1a loop body, unchanged in shape: read the sticks (channels polled only while the
/// controller is connected — a dropped controller commands zero twist, HA-103), map them
/// through the ONE shared mapping (HA-112, invented; T2 owns driver feel), drive BODY-frame
/// through the facade, let the caller paint its panel, pace to the tick boundary. Plus Part
/// 0b's finding: a field DISABLE is polled and exits the loop CLEANLY — PROS deletes the
/// opcontrol task on disable and a deleted task runs no destructor, so this is the only path
/// on which the facade's cancel() (0 V + Brake) runs before VEXos cuts the motors itself;
/// PROS calls opcontrol() again when driver control resumes. A function template so that the
/// builds which do not call it (every build but xdrive) emit no -Wunused-function for it.
template <typename PerTick>
void runTeleopLoop(shulib::chassis::Chassis& chassis, const shulib::hal::IController& master,
                   shulib::motion::ITickPacer& pacer, PerTick&& perTick) {
    const shulib::motion::MotionConfig& cfg = chassis.motionConfig();
    // DEFECTS2: since the adapter makes Brake/Hold real at 0 V, a released stick after a cancel()
    // (which leaves the drive in Brake) would BRAKE. The driver's release mode is COAST (the team
    // lead's ruling for Teleop, applied here too) -- at entry and after every drive(), exactly as
    // teleop::runDriverControl does it. G1 Part 2c removes this loop with the X-drive graph.
    const auto coastDrive = [&chassis] {
        for (shulib::hal::IMotor* m : chassis.deps().ctx->driveMotors()) {
            m->setBrakeMode(shulib::hal::BrakeMode::Coast);
        }
    };
    coastDrive();
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
        coastDrive();
        perTick(command, sticks.connected);
        pacer.pace();
    }
}

}  // namespace

/// fault_latch_hook.hpp: point the precondition handler above at a latch that lives in another
/// translation unit. Defined for EVERY build (every src/*.cpp is compiled into every build, and
/// src/teleop_program.cpp calls this); --gc-sections drops it where nothing does.
void shulib::bench::attachFaultLatch(shulib::diag::FaultLatch* latch) noexcept {
    g_faults = latch;
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * Order is load-bearing (check.hpp's concurrency contract): the precondition
 * policy is installed FIRST — before any graph constructs (its adapter ctors
 * carry read-back preconditions) and before any other task exists. Then the
 * selected program's boot: for the invented X-drive, the object graph, the
 * fault-latch hookup, IMU calibration, and the §18.5 session header with the
 * build hash the Makefile injected; for "shulib Teleop", the same steps inside
 * shulib::bench::teleopInitialize() (src/teleop_program.cpp).
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
    // "shulib Teleop" (G1 Part 2b): the program lives in src/teleop_program.cpp -- robot::Robot
    // over a hal::pros::ProsDeviceProvider, from the boot source (src/boot_source.{hpp,cpp}). Only the
    // BEACON and the competition bits are formatted here: the beacon must stay a literal that THIS
    // branch references, or --gc-sections drops it and the src build gate's beacon step fails
    // (the G1 Part 2b brief's A2; the teleop program is compiled into every build, so a literal
    // there would land in the other builds' packages).
    {
        char bits[64];
        shulib::bench::teleopInitialize(
            kVariantBeacon, competitionBits(pros::c::competition_get_status(), bits, sizeof bits));
    }
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
    session.routineId = "(none: teleop only)";  // no program here ever runs an auton (header)
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
                    "absent devices (deliberate): tags=ABSENT vision=ABSENT -- no camera adapter "
                    "exists yet; no corrector or vision task is wired over an absent source");

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
 * Commands NO motion in ANY build of this file, deliberately. The bench tester,
 * "shulib Drive", "shulib Calibrate" and "shulib Teleop" never will: the team
 * lead's ruling (2026-09-25) is that autonomous routines run only in the
 * competition program's autonomous() (COMP), never inside a teleop or bench
 * program. The invented X-drive graph cannot boot on a real robot (HA-111) and
 * still says, over serial, that it commands nothing. The v2 motions exist and
 * settle on the host test plant (test/chassis_routine_test.cpp chains them
 * through the same facade); no library-commanded motion has run on a robot.
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
    // "shulib Teleop" NEVER moves here, and no routine is ever added here (the team lead's
    // ruling, 2026-09-25: autonomous routines run only in the competition program's
    // autonomous()). The state screen says so in the program's own sentence (at most 50
    // characters, checked in src/teleop_program.cpp); the program logs the same sentence once.
    benchStateScreen("AUTONOMOUS", shulib::bench::kTeleopAutonomousReason);
    shulib::bench::teleopAutonomous();
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
 * "shulib Teleop" (G1 Part 2b) runs the LIBRARY's loop, teleop::runDriverControl,
 * with the DriveShaper feel layer (src/teleop_program.cpp). The invented X-drive
 * still runs the R1a teleop loop (runTeleopLoop above): master-controller sticks →
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
    // "shulib Teleop" (G1 Part 2b): teleop::runDriverControl over the robot::Robot built in
    // initialize(), with the DriveShaper feel layer (src/teleop_program.cpp). Returns on a field
    // DISABLE after cancel() (0 V + Brake); PROS calls opcontrol() again when driver control
    // resumes. When the boot refused, it repaints the refusal and idles, powering nothing.
    shulib::bench::runTeleop();
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
