// xdrive_program.cpp — the INVENTED X-DRIVE, built on robot::Robot (G1 Part 2c, 2026-09-26).
//
// ═══ WHAT CHANGED AT G1 PART 2c, AND WHAT DID NOT ════════════════════════════════════
// Until Part 2c this program was src/main.cpp's `struct Robot` (a hand-wired object graph, built
// by a function-local static on first use), `portMapString()`, and `runTeleopLoop()` — a 30-line
// loop mapping the sticks through the raw R1a mapping. Now:
//   * the ROBOT is data — src/robots/xdrive_config.hpp's `kInventedXDrive`, the same numbers — and the
//     GRAPH is robot::Robot over hal::pros::ProsDeviceProvider, the way "shulib Teleop" builds
//     robot two. The builder installs a GPS corrector the hand-wired graph did not have (the
//     config header says why);
//   * the LOOP is teleop::runDriverControl with the DriveShaper's default profile (ArcadeBody:
//     the old mapping's axes, with a continuous deadband instead of a hard one), field-centric
//     OFF (ruling M5 of the G1 Part 2 brief), the drive COASTING on stick release (DEFECTS2's
//     releaseBrakeMode, the team lead's ruling for driver control) — so runTeleopLoop is gone;
//   * the BOOT no longer lets an adapter's throw escape initialize(): the provider asks the
//     brain's registry before it opens a port, validate() names the first port that is not what
//     the config says, and the program paints and prints that refusal with nothing powered.
// What did NOT change: the ports, signs, cartridge and geometry; the session header's port map
// (robot::Robot::portMap() prints what portMapString() printed, byte for byte — pinned by
// test/xdrive_config_test.cpp); the controller's fault display; the boot lines that are still true
// (G1-PROGRESS "Part 2c" diffs every one).
//
// ═══ RUN STATUS ═════════════════════════════════════════════════════════════════════
// THIS VERSION HAS NEVER BOOTED, and cannot on any robot that exists: no X-drive robot exists and
// its ports are invented (A4 register HA-111) — on robot two the provider refuses port 1 (the
// radio) before anything is opened. An X-drive build booted on a V5 ONCE: 2026-08-12, the old
// hand-wired graph over FAKE devices (build-order.md, "the three things nothing had touched") —
// it drove nothing and could not. None is on record booting since R1a gave it real adapters.
//
// ═══ STORAGE ════════════════════════════════════════════════════════════════════════
// Exactly "shulib Teleop"'s pattern (src/programs/teleop_program.cpp, "STORAGE"; the G1 Part 2 brief's
// landmine 1): ONE function-local static of EMPTY optionals — a constructor that cannot throw —
// emplaced once from xdriveInitialize(). Never at namespace scope (built before initialize() has
// installed the precondition handler, in an order nobody controls, and kept alive by a static
// initializer in every one of the seven builds this file compiles into).
//
// Compiled in all seven builds (the brief's A12): no SHULIB_* macro is read here, no beacon literal
// lives here (landmine 10), and the one symbol it needs from src/main.cpp (attachFaultLatch) is
// defined in all seven. ASCII only in every runtime string; every screen string goes through
// printText(); only a literal "%s" reaches a PROS print.

#include "programs/xdrive_program.hpp"

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <string_view>

#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "pros/screen.h"

#include "common/driver_controls.hpp"
#include "common/fault_latch_hook.hpp"
#include "common/screen_text.hpp"
#include "robots/xdrive_config.hpp"
#include "shulib/calibration/calibrate_panel.hpp"
#include "shulib/core/check.hpp"
#include "shulib/diag/build_info.hpp"
#include "shulib/diag/controller_display.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/diag/session_info.hpp"
#include "shulib/hal/controller.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/hal/pros/controller.hpp"
#include "shulib/hal/pros/device_provider.hpp"
#include "shulib/hal/pros/line_display.hpp"
#include "shulib/hal/pros/tick_pacer.hpp"
#include "shulib/hal/telemetry_sink.hpp"
#include "shulib/robot/robot.hpp"
#include "shulib/teleop/driver_control.hpp"
#include "shulib/teleop/drive_modes.hpp"
#include "shulib/teleop/input_shaping.hpp"
#include "shulib/teleop/teleop_loop.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::bench {
namespace {

using calibration::kMediumCols;
using calibration::kSmallCols;
using calibration::TextSize;

// ═══ DRIVER CONTROL — the choices this program makes ═════════════════════════════════
/// The driver's feel: the library's DEFAULT profile — ArcadeBody, deadband 0.05 continuous, no
/// expo, no slew (every number PROVISIONAL, A4 register HA-112; no driver has felt it). An X-drive
/// is where field-centric earns its keep, and it stays OFF anyway: the first run of a drivetrain
/// must not also be the first run of a frame rotation (the G1 Part 2 brief's ruling M5).
constexpr teleop::DriverProfile kProfile{};
/// The loop's configuration, built ONCE, here, and handed to runDriverControl exactly as it is —
/// so the checks below read what the loop RECEIVES, not a copy of it: a field changed or dropped
/// HERE fails the build. (What they cannot see: runXdrive() passing some other config instead of
/// this one — keep the call passing kLoopConfig by name.) It carries:
///  * the tick period: the pacer's (ProsTickPacer::kTickMs, HA-32);
///  * the field-centric re-zero button: `Y`. Harmless in ArcadeBody. Never a shoulder button (the
///    library's own default, L1, is a claw pivot's button on robot two — T3), and the button
///    "shulib Teleop"'s re-zero moves to at T3, so the two programs agree once T3 lands;
///  * the drive's mode on stick release: Coast (the team lead's ruling for driver control) —
///    STATED, not left to the default, so the boot line and the loop cannot disagree.
constexpr teleop::DriverLoopConfig kLoopConfig{
    .tickPeriod = units::Time{static_cast<double>(hal::pros::ProsTickPacer::kTickMs) / 1000.0},
    .rezeroButton = hal::ControllerButton::Y,
    .releaseBrakeMode = hal::BrakeMode::Coast};
static_assert(kLoopConfig.tickPeriod.value() > 0.0,
              "a zero tick period trips runDriverControl's precondition before anything moves");
static_assert(kLoopConfig.rezeroButton != hal::ControllerButton::L1
                  && kLoopConfig.rezeroButton != hal::ControllerButton::L2
                  && kLoopConfig.rezeroButton != hal::ControllerButton::R1
                  && kLoopConfig.rezeroButton != hal::ControllerButton::R2,
              "the re-zero button may not be a shoulder button");

/// What the driver line says — ArcadeBody, deadband 0.05, no expo, no slew, re-zero Y, release
/// COAST — checked against the profile and the loop configuration the loop is given, so a change
/// to either (or to the library's defaults) cannot leave it saying the old words
/// (src/programs/teleop_program.cpp's driverLineMatchesProfile(), for this program's profile).
constexpr bool driverLineMatchesProfile() {
    const teleop::AxisShaping* axes[3] = {&kProfile.forward, &kProfile.left, &kProfile.yaw};
    for (const teleop::AxisShaping* a : axes) {
        if (a->deadband != 0.05 || a->expo != 0.0) {
            return false;
        }
    }
    return kProfile.mode == teleop::DriveMode::ArcadeBody && !(kProfile.slew.ratePerSecond > 0.0)
           && kLoopConfig.releaseBrakeMode == hal::BrakeMode::Coast
           && kLoopConfig.rezeroButton == hal::ControllerButton::Y;
}
static_assert(driverLineMatchesProfile(),
              "kDriverLine names ArcadeBody, deadband 0.05, no expo, no slew, button Y and COAST: "
              "reword it with the profile");

// ═══ THE SERIAL LINES (TermSink cuts a message at 200 bytes: every literal is checked) ═════
constexpr const char* kTag = "R1A";  // this graph's serial prefix since R1a
constexpr std::size_t kLogMessageBytes = 200;  // diag/term_sink.hpp's kMaxMessageBytes
constexpr std::string_view kWiredLine =
    "shulib v2 core wired by robot::Robot over hal::pros::ProsDeviceProvider (G1 Part 2c): X-drive "
    "kinematics + Pilons odometry + GPS corrector + fused localizer + motion scheduler + Chassis";
// Printed only when the graph BUILT, i.e. on a brain that carries exactly the invented devices —
// so it says what is true at that boot, and makes no claim about boots before it.
constexpr std::string_view kUnvalidatedLine =
    "HARDWARE-UNVALIDATED: this X-drive graph is built on REAL adapters, but every port and sign "
    "in it is INVENTED (HA-111) -- nothing here was measured; no motion it commands is validated";
constexpr std::string_view kAbsentDevicesLine =
    "absent devices (deliberate): tags=ABSENT vision=ABSENT -- no camera adapter exists yet; no "
    "corrector or vision task is wired over an absent source";
constexpr std::string_view kDriverLine =
    "driver control: teleop::runDriverControl + DriveShaper, the default profile (ArcadeBody; "
    "continuous deadband 0.05, no expo, no slew -- HA-112, unfelt); re-zero Y; release COAST";
constexpr std::string_view kOpcontrolLive =
    "opcontrol(): teleop drive loop live (teleop::runDriverControl, DriveShaper default profile "
    "ArcadeBody, release COAST; HA-112, unfelt)";
constexpr std::string_view kAutonomousLine =
    "autonomous(): NO motion here, ever -- autonomous routines run only in the competition "
    "program's autonomous() (the team lead's ruling, 2026-09-25)";
static_assert(kWiredLine.size() <= kLogMessageBytes && kUnvalidatedLine.size() <= kLogMessageBytes
                  && kAbsentDevicesLine.size() <= kLogMessageBytes
                  && kDriverLine.size() <= kLogMessageBytes
                  && kOpcontrolLive.size() <= kLogMessageBytes
                  && kAutonomousLine.size() <= kLogMessageBytes,
              "a fixed boot line would be cut by TermSink at 200 bytes -- split it");

// ═══ THE REFUSAL SCREEN — the one screen this program paints ═══════════════════════════
constexpr std::uint32_t kColBg = 0x101418, kColText = 0xF0F0F0, kColDim = 0x9AA4AE,
                        kColWarn = 0xE8B23A, kColBad = 0xE2564A;
constexpr std::int16_t kRowX = 8;
constexpr std::int16_t kWhyFirstY = 40;
constexpr std::int16_t kRowPitch = 16;  // SMALL's estimated line height (HA-136)
constexpr std::size_t kWhyRows = 5;

namespace text {
inline constexpr std::string_view kRefusedWord = "X-DRIVE REFUSED AT BOOT";
inline constexpr std::string_view kInvented = "Every port in this build is INVENTED (HA-111).";
inline constexpr std::string_view kNothingPowered = "The graph was NOT built. Nothing is powered.";
inline constexpr std::string_view kRobotTwo = "Robot two runs shulib Teleop (slot 2) instead.";
inline constexpr std::string_view kMoreOnSerial = "(the full text is on the serial log)";
static_assert(calibration::screenText(kRefusedWord, kMediumCols)
                  && calibration::screenText(kInvented, kSmallCols)
                  && calibration::screenText(kNothingPowered, kSmallCols)
                  && calibration::screenText(kRobotTwo, kSmallCols)
                  && calibration::screenText(kMoreOnSerial, kSmallCols),
              "xdrive_program: a screen literal does not fit its budget, or is not ASCII, or "
              "contains a percent sign");
}  // namespace text
static_assert(kWhyFirstY + kRowPitch * static_cast<int>(kWhyRows) + 4 < 150,
              "the reason's rows would run into the fixed lines below them");

// ═══ PROGRAM STATE (header: one function-local static of empty optionals) ═════════════════
struct XDriveState {
    // The provider BEFORE the robot: the robot borrows the provider's devices, so it is destroyed
    // first (reverse declaration order); the displays after the robot, whose latch they read.
    std::optional<hal::pros::ProsDeviceProvider> devices{};
    std::optional<robot::Robot> robot{};
    // The program's OWN controller and controller screen: the provider supplies neither.
    std::optional<hal::pros::ProsController> master{};
    std::optional<hal::pros::ProsLineDisplay> lcd{};
    std::optional<diag::ControllerFaultDisplay> faultDisplay{};
    // One driver, one shaper, held across opcontrol() calls.
    teleop::DriveShaper shaper{kProfile};
    char portMap[diag::kMaxPortMapBytes]{};
    char refusal[192]{};
};

XDriveState& state() {
    static XDriveState s;
    return s;
}

/// The refusal, on the screen (every string through printText()) and on serial.
void paintRefusal(const char* why) {
    pros::c::screen_set_eraser(kColBg);
    pros::c::screen_erase();
    printText(TextSize::Medium, kRowX, 8, kColBad, kColBg, text::kRefusedWord.data());
    const auto w = calibration::wrapWords<kSmallCols, kWhyRows>(why != nullptr ? why : "");
    for (std::size_t k = 0; k < w.count; ++k) {
        printText(TextSize::Small, kRowX,
                  static_cast<std::int16_t>(kWhyFirstY + kRowPitch * static_cast<int>(k)),
                  kColText, kColBg, w.line[k]);
    }
    if (!w.complete) {
        printText(TextSize::Small, kRowX, 124, kColDim, kColBg, text::kMoreOnSerial.data());
    }
    printText(TextSize::Small, kRowX, 150, kColWarn, kColBg, text::kInvented.data());
    printText(TextSize::Small, kRowX, 166, kColDim, kColBg, text::kNothingPowered.data());
    printText(TextSize::Small, kRowX, 182, kColDim, kColBg, text::kRobotTwo.data());
    std::printf("[%s] X-DRIVE REFUSED AT BOOT: %s -- every port in this build is INVENTED "
                "(HA-111); nothing is powered\n",
                kTag, why != nullptr ? why : "");
    std::fflush(stdout);
}

void refuse(const char* why) {
    XDriveState& st = state();
    std::snprintf(st.refusal, sizeof st.refusal, "%.191s", why != nullptr ? why : "");
    paintRefusal(st.refusal);
}

// The heading the feel layer trusts is src/common/driver_controls.hpp's driverHeading() — the ONE
// heading rule since T4 Part 1a (2026-09-27), shared with "shulib Teleop" and every driver file,
// and host-tested there. (G1 Part 2c wrote it here a second time, deliberately, while T3 was
// changing Teleop in parallel, and listed the merge as owed; T4 Part 1a made it.)

}  // namespace

void xdriveInitialize(const char* variantBeacon, const char* competitionStatusBits) {
    XDriveState& st = state();
    // ── the banner FIRST — the beacon and the competition status bits, before the graph boundary,
    //    as every other program prints them: a REFUSED boot (on every robot that exists) is the one
    //    this build will actually have, and its serial capture must still say which build it was.
    std::printf("\n[%s] booted: INVENTED X-DRIVE build [%s] -- every port and sign INVENTED\n"
                "[%s] (HA-111); on a robot without exactly those devices it refuses below.\n",
                kTag, variantBeacon, kTag);
    std::printf("[%s] competition status at boot: %s -- the drive loop runs only in DRIVER "
                "CONTROL, enabled; this program paints no state screen.\n",
                kTag, competitionStatusBits != nullptr ? competitionStatusBits : "");
    std::fflush(stdout);

    // ── the graph, at its boundary: validate() AND the construction, once, into the optionals.
    //    A provider configuration conflict is a precondition, so the try covers both.
    try {
        st.devices.emplace();
        const robot::BuildVerdict verdict = robot::validate(kInventedXDrive, *st.devices);
        if (!verdict.ok) {
            st.devices.reset();
            refuse(verdict.why);
            return;
        }
        st.robot.emplace(kInventedXDrive, *st.devices);
    } catch (const PreconditionError& e) {
        st.robot.reset();
        st.devices.reset();
        refuse(e.what());
        return;
    }
    robot::Robot& r = *st.robot;
    hal::ITelemetrySink& log = st.devices->telemetry();
    attachFaultLatch(&r.faults());  // a precondition after boot latches FaultCode::Precondition
    st.master.emplace(hal::pros::ControllerId::Master);
    st.lcd.emplace();
    st.faultDisplay.emplace(*st.lcd, r.faults(), r.context().battery());

    // ── the §18.5 session header: provenance FIRST, a missing hash LOUD; the port map GENERATED
    //    from the config (robot::Robot::portMap()), never retyped.
    r.portMap(st.portMap, sizeof st.portMap);
    diag::SessionInfo session{};
    session.buildHash = diag::compiledBuildHash();
    session.routineId = "(none: teleop only)";  // no program here ever runs an auton
    session.alliance = "";                      // renders "-": no field, no alliance
    session.side = "";
    session.portMap = st.portMap;
    diag::emitSessionHeader(log, session, r.context().battery().voltage());

    log.log(hal::LogLevel::Info, kTag, kWiredLine);
    {
        char banner[200];
        r.describe(banner, sizeof banner);
        char line[216];
        std::snprintf(line, sizeof line, "robot::Robot: %.180s", banner);
        log.log(hal::LogLevel::Info, kTag, line);
    }
    log.log(hal::LogLevel::Warn, kTag, kUnvalidatedLine);
    // The absent-device announcement (R3b §6.4): said ONCE, because an absent camera otherwise
    // reads exactly like one that never sees a tag.
    log.log(hal::LogLevel::Info, "R3B", kAbsentDevicesLine);
    // One live query THROUGH the facade, so the banner is evidence of a working graph rather than
    // prose: strafeAuthority() reads Chassis -> kinematics (F5) and must be 1.00 on this X-drive.
    char line[64];
    std::snprintf(line, sizeof line, "facade alive: strafeAuthority=%.2f (X-drive: 1.00)",
                  r.chassis().strafeAuthority());
    log.log(hal::LogLevel::Info, kTag, line);
    log.log(hal::LogLevel::Info, kTag, kDriverLine);
}

void xdriveAutonomous() {
    XDriveState& st = state();
    if (!st.robot.has_value()) {
        return;  // refused at boot: nothing was built, and nothing moves
    }
    st.devices->telemetry().log(hal::LogLevel::Warn, kTag, kAutonomousLine);
}

void runXdrive() {
    XDriveState& st = state();
    if (!st.robot.has_value()) {
        // Refused at boot: no graph exists, nothing is powered. Keep the reason on screen and idle.
        paintRefusal(st.refusal);
        while (true) {
            pros::delay(250);
        }
    }
    robot::Robot& r = *st.robot;
    st.devices->telemetry().log(hal::LogLevel::Info, kTag, kOpcontrolLive);
    hal::IClock& clock = st.devices->clock();
    diag::ControllerFaultDisplay& faultDisplay = *st.faultDisplay;
    teleop::runDriverControl(
        r.chassis(), *st.master, st.devices->pacer(), st.shaper, kLoopConfig,
        [] { return pros::c::competition_is_disabled() != 0; },
        [&r] { return driverHeading(r); },
        [&clock, &faultDisplay](const math::ChassisSpeeds&, bool, const teleop::DriveShaperState&) {
            faultDisplay.update(clock.now());
        });
}

}  // namespace shulib::bench
