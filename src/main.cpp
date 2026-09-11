// main.cpp — the PROS entry point, rewired onto the shulib v2 core (chunk C7,
// WS11/M2), with the hal/pros adapters wired in at chunk R1a.
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
//     (HA-112 records the raw mapping as invented). Since R3b Session 2 the
//     stick → ChassisSpeeds mapping itself lives in the PROS-free
//     shulib/teleop/stick_mapping.hpp (host-tested, bit-identical to what this
//     file carried), so the bench tester's DRIVE station and this loop share it.
//
// ═══ Absent, deliberately (chunk R3b §6) ════════════════════════════════════════════
//   * AbsentTagSource / AbsentVision — NO camera is installed on any current robot.
//     These replaced the FakeTagSource/FakeVision stubs: a test fake in a competition
//     binary is test scaffolding, and it erases the difference between "this robot has
//     no such device" and "a test will inject readings here". An absent source is never
//     polled (§6.5) — no corrector, no vision task, no false "camera alive" record —
//     and the boot log announces the absence once. R2 (camera) swaps in the real
//     adapter, visibly, on this exact line.

#include "main.h"

// ═══ WHICH ROBOT IS THIS BINARY FOR? (chunk R3a §4.2 item 1; third variant at R3b S2) ═══
// The X-drive wiring below is PRESERVED VERBATIM and relabelled -- it is the only
// artifact of the 2026-08-12 whole-object-graph boot, and its ports are invented
// (HA-111). It CANNOT boot on the measured bench robot: it puts motors on 1/2/-3/-4
// (port 4 is the IMU), rotation sensors on 5/6, and a GPS on 9 -- every one of those
// throws an adapter read-back precondition at boot.
//
// THREE robot variants on one validated Makefile switch, and a SECOND AXIS for the program
// (R3b Part 0b) -- any other value of either is an $(error):
//   make                            → ROBOT=bench  the measured tank BENCH bot: runs the bench
//                                     tester ("Bench Tests")
//   make ROBOT=tank                 → the 2026 tank chassis, ROBOT TWO: runs the bench tester
//                                     over robot two's SIGNED chassis table
//                                     (src/chassis_table.hpp); no library graph yet (Part 3)
//   make ROBOT=tank PROGRAM=drive   → "shulib Drive": robot two DRIVES from the sticks through
//                                     the hal/pros adapters with dead-port tolerance
//                                     (src/drive_program.cpp) -- its own program in its own
//                                     slot, by the team lead's ruling; NOT the motion stack
//   make ROBOT=xdrive               → the invented X-drive wiring below (HA-111)
// (The instruction here used to say `make CXXFLAGS_EXTRA=-D...` — common.mk consumes
//  EXTRA_CXXFLAGS, the names were transposed, and the documented command silently built
//  the BENCH variant. GATE1 replaced it with the validated ROBOT switch in the Makefile,
//  and tools/src_build_gate.py now asserts per-variant that the define really lands --
//  as a whole command-line TOKEN, and end-to-end via the beacon string below.)
#if defined(SHULIB_ROBOT_XDRIVE_INVENTED) && defined(SHULIB_ROBOT_TANK_2026)
#error "two robot variant defines are set at once -- select exactly one with make ROBOT=..."
#endif
#if defined(SHULIB_PROGRAM_DRIVE) && !defined(SHULIB_ROBOT_TANK_2026)
#error "PROGRAM=drive needs ROBOT=tank -- only robot two's chassis table carries measured signs"
#endif
#if defined(SHULIB_ROBOT_XDRIVE_INVENTED)
#define SHULIB_RUNS_BENCH_TESTER 0
#else
#define SHULIB_RUNS_BENCH_TESTER 1  // bench AND tank: a tester-variant robot (no library graph);
                                    // which PROGRAM runs in opcontrol() is the second axis
#endif

#if SHULIB_RUNS_BENCH_TESTER
#include "bench_r3a.hpp"
#if defined(SHULIB_PROGRAM_DRIVE)
#include "drive_program.hpp"
#endif
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

namespace {

// ═══ THE VARIANT IDENTITY BEACON (R3b Session 2; asserted by tools/src_build_gate.py) ═══
// One string per BUILD, selected by the SAME preprocessor facts that select the wiring and
// the program, and printed at boot so it is referenced and survives into the linked package.
// The src build gate asserts the expected beacon IS in bin/hot.package.elf and the other
// three are NOT (matched WITH the string's NUL terminator, because `…=tank` is a prefix of
// `…=tank-drive`). That is the end-to-end proof that the #if chain in THIS file took the
// branch the Makefile asked for: the gate's compile-line check can see a define that never
// landed, but not a macro renamed here -- and a tank build that silently carried the bench
// bot's chassis table onto robot two is exactly HA-111's defect class again; a "shulib
// Drive" upload that was silently the tester is the same class on the second axis.
#if defined(SHULIB_ROBOT_XDRIVE_INVENTED)
constexpr const char kVariantBeacon[] = "shulib-robot-variant=xdrive";
#elif defined(SHULIB_ROBOT_TANK_2026) && defined(SHULIB_PROGRAM_DRIVE)
constexpr const char kVariantBeacon[] = "shulib-robot-variant=tank-drive";
#elif defined(SHULIB_ROBOT_TANK_2026)
constexpr const char kVariantBeacon[] = "shulib-robot-variant=tank";
#else
constexpr const char kVariantBeacon[] = "shulib-robot-variant=bench";
#endif

#if SHULIB_RUNS_BENCH_TESTER
// ═══ COMPETITION-STATE VISIBILITY (2026-09-10, robot two's brain) ══════════════════════
// The tester runs inside opcontrol(), so it exists only while VEXos reports DRIVER
// CONTROL, ENABLED. The first time a controller was linked to a brain running it, the
// program restarted and then showed a BLACK screen for as long as the controller stayed
// linked: PROS never started the driver task, and nothing on the brain said why. Every
// non-driver entry point below now paints the screen with the state VEXos reports and what
// to do about it, and the raw status bits print at boot -- so "the touchscreen does not
// work" cannot be the diagnosis again. Guarded, so the xdrive variant (which the src build
// gate expects to emit no unused-function warning) never sees an unused helper.
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
#endif  // SHULIB_RUNS_BENCH_TESTER

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
    //    ABSENT, deliberately (R3b §6): no camera is installed on any current robot, and a
    //    test fake in a competition binary erases "no device" vs "a test injects here". No
    //    corrector or vision task is wired over these — kInstallTagCorrector<AbsentTagSource>
    //    is false, so polling one would manufacture a false "camera alive, no tags" record
    //    (§6.5). The boot log announces the absence once; R2 (camera) swaps in the real
    //    adapter here. (The BENCH robot also has no GPS — its context wiring, which lands
    //    with R3b pieces 1–2, spells that as `.gps = &absentGps`. THIS invented X-drive
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
#endif  // SHULIB_RUNS_BENCH_TESTER
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol.
 */
void disabled() {
#if SHULIB_RUNS_BENCH_TESTER
    benchStateScreen("DISABLED",
                     "VEXos reports the robot DISABLED, so the driver task is not running.");
#endif
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch.
 */
void competition_initialize() {
#if SHULIB_RUNS_BENCH_TESTER
    benchStateScreen("FIELD CONTROL CONNECTED",
                     "A competition switch or field is connected through the controller.");
#endif
}

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
#if SHULIB_RUNS_BENCH_TESTER
    benchStateScreen("AUTONOMOUS",
                     "No motion here, ever: the tester's DRIVE station lives under driver control.");
    return;
#else
    Robot& r = robot();
    r.telemetry.log(shulib::hal::LogLevel::Warn, "R1A",
                    "autonomous(): no motion until R3 validates the stack on hardware "
                    "(adapters landed at R1a; the register's R3 group is unsettled)");
#endif  // SHULIB_RUNS_BENCH_TESTER
}

/**
 * Runs the operator control code.
 *
 * The R1a teleop loop: master-controller sticks → body-frame
 * chassis.drive() at the tick cadence. Mapping (HA-112, invented, T2 owns
 * refinement) is shulib::teleop::mapSticksToChassisSpeeds — the PROS-free,
 * host-tested function this loop used to carry inline, bit-identical
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
#else
    Robot& r = robot();
    r.telemetry.log(shulib::hal::LogLevel::Info, "R1A",
                    "opcontrol(): teleop drive loop live (deadband-only mapping, HA-112, "
                    "shulib/teleop/stick_mapping.hpp; driver-feel shaping is chunk T2)");
    const shulib::motion::MotionConfig& cfg = r.chassis.motionConfig();
    while (true) {
        // The axes are read only while connected -- the exact call pattern of the loop
        // before the extraction (a disconnected controller's channels are never polled),
        // so nothing about this loop's device traffic changed either.
        shulib::teleop::StickInput sticks{.connected = r.master.isConnected()};
        if (sticks.connected) {
            sticks.leftY = r.master.axis(shulib::hal::ControllerAxis::LeftY);
            sticks.leftX = r.master.axis(shulib::hal::ControllerAxis::LeftX);
            sticks.rightX = r.master.axis(shulib::hal::ControllerAxis::RightX);
        }
        const shulib::math::ChassisSpeeds command = shulib::teleop::mapSticksToChassisSpeeds(
            sticks, cfg.maxLinearSpeed, cfg.maxAngularSpeed);
        r.chassis.drive(command, shulib::math::Frame::Body);
        r.faultDisplay.update(r.clock.now());
        r.pacer.pace();
    }
#endif  // SHULIB_RUNS_BENCH_TESTER
}
