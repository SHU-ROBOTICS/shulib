// drive_program.cpp — "shulib Drive": robot two's DRIVER PROGRAM — the team's driver file
// (src/team/robot_two_driver.hpp) on robot::Robot, built from the calibration record (chunk T4
// Part 3, 2026-10-01; the team lead's rulings R3 and R5, 2026-09-27).
//
// ═══ WHAT CHANGED AT T4 PART 3 ═════════════════════════════════════════════════════════
// This file's content was REPLACED, its name kept (R3: "shulib Drive" is the same name, slot 1,
// `PROGRAM=drive` and beacon `tank-drive`). What it held until then — the old "shulib Drive" of R3b
// Part 0b: ten bare ProsMotors from the typed chassis table, open-loop volts, its own SD text log,
// its own panel, and the protections — is gone from here:
//   * the open-loop drive: the library drives now (the driver file's feel, Chassis::drive()); ruling
//     M4's raw-volts baseline lives on in Bench Tests' DRIVE station (src/programs/bench_r3a.cpp);
//   * the protections: MOVED into the library at T4 Part 2 (teleop/drive_protection.hpp), applied
//     inside teleop::runDriverControl for every program — the driver file reaches them with no
//     wiring of its own;
//   * the SD text log: the PROVIDER's copy now (hal/pros/device_provider.hpp, `drive_log.txt`,
//     events and summaries only — the team lead's N5 (a) and A (a));
//   * the typed table's refusals and the per-port adapters: the boot source's record check and the
//     Robot's absent ports (src/common/boot_source.hpp).
// And "shulib Teleop" (src/programs/teleop_program.cpp, retired at T4 Part 3, R5) MOVED in: its
// boot order, its storage rule, its boot lines (without its banner's claw sentence), its run status
// row and source rows, and its panel — split into this program's MAIN, DETAILS and MORE pages
// (src/programs/drive_panel.hpp, pure and host-tested). Nothing was copied: both files lost what
// this one gained.
//
// ═══ WHICH END IS FORWARD ═══════════════════════════════════════════════════════════
// Stick up drives toward the end that LED the Calibrate push saved in the record: on robot two
// since 2026-09-25, the AI Vision (camera, port 3) end — the 11/20 end. The OLD "shulib Drive"
// drove toward the typed table's front, the 15/16 end: a brain still holding the old package drives
// the OTHER way on the same stick (worksheet Station F.0: tell them apart before a wheel moves —
// the old one's serial tag was `[R3B]`, this one's is `[DRV]`).
//
// ═══ ORDER AT BOOT (initialize()) ═══════════════════════════════════════════════════
// src/main.cpp installs the precondition handler FIRST, then calls driveInitialize(): the banner and
// competition status (from main.cpp's arguments — the beacon stays a literal in
// src/build_selection.hpp's one chain), the boot source (which prints its own decision lines), then
// the graph, ONCE, into std::optional storage inside the graph boundary's ONE try — the provider
// (which opens the SD copy), validate(), the Robot's construction, and the driver file's bind(),
// because a provider configuration conflict and a binding's volts are preconditions — then the
// fault latch handed to main.cpp's handler (fault_latch_hook.hpp), the session header, and the boot
// lines. The banner and the boot source's lines come BEFORE the provider, so they are on serial
// only; everything from the provider's construction on is in the SD copy too — a refusal decided
// after it included: refuseBuilt() says it through the provider's sink before releasing it (the
// T4-3 review), so the card's drive_log.txt, truncated by this boot, says why this boot refused.
// (A refusal decided BEFORE the provider — the boot source's — is serial and the screen only: no
// copy is open yet, and the card keeps the previous boot's file.) The copy's two exceptions, each
// counted by the provider (serialCopyDroppedLines(); the boot line says it as of the boot only):
// a line that found another task mid-copy, and — when PROS deletes a task while it is mid-copy
// (HA-161 (a), reasoned; this program's opcontrol() logs a line right after its loop sees the
// disable, the moment PROS deletes the task) — every later line of that boot: the card's copy ends
// there and serial keeps them all (diag/log_copy_sink.hpp, "A writer that never comes back").
//
// ═══ STORAGE: one function-local static ═════════════════════════════════════════════
// robot::Robot and the provider are non-movable, and construction order across translation units
// is undefined, so neither is built at namespace scope; a function-local static whose constructor
// THROWS would be re-attempted on the next call, so the static below holds empty optionals and
// members whose constructors cannot throw and touch no hardware — the driver file's
// RobotTwoDriver: its ProsController touches no device until it is read (hal/pros/controller.hpp),
// its DriveShaper's constructor is noexcept, and its bindings start empty. driveInitialize()
// emplaces into the optionals, once. Function-local rather than namespace-scope for one more
// reason: this file compiles in every build, and a namespace-scope object with a non-trivial
// destructor registers that destructor from a static initializer the linker must keep — dragging
// the whole graph's code into every other program's package. A function-local static is
// initialized on first call, so in the builds that never call it, --gc-sections drops everything
// here.
//
// Compiled in every build: nothing here reads a build switch (only src/build_selection.hpp's one
// chain does), no beacon literal lives here, and every symbol it needs from main.cpp
// (attachFaultLatch) is defined in every build. ASCII only in every runtime string; only a literal
// "%s" reaches a PROS print.
//
// ═══ THE SCREEN AND THE CONTROLLER (DriveScreen, below) ═══════════════════════════════
// Three pages — MAIN (the driver's: the PROTECTION row, the controller row, a DETAILS button),
// DETAILS and MORE — built from src/programs/drive_panel.hpp's pure model, whose rows the shared
// driver-panel header words (src/common/driver_panel.hpp, shared with the competition program).
// Only the visible page is painted, through the shared painters (src/common/screen_paint.hpp); its
// rows are rebuilt at most every 200 ms and a row is repainted only when its text or tone changed
// (driveRepaint(), host-tested): PROS leaks every screen print. A touch flips the page on its
// RELEASE edge, once the page has been up the arming delay, and never for a press held across a
// flip (DrivePager); the touch is read through src/common/screen_touch.hpp's reduction, one PROS
// read per tick. The controller's three lines come from the ONE chooser and go out ONE line per
// write through its writer (driver_panel.hpp's controllerScreenLines() and ControllerLineWriter),
// the protections' REFUSED / CUT / dead words included. A boot that refuses paints the shared
// refusal screen (paintRefusalScreen) in place of all of it, and opcontrol() paints it again and
// idles. Every layout number is an ESTIMATE (A4 register HA-136); nothing here has been seen on a
// brain.

#include "programs/drive_program.hpp"

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <string_view>

#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "pros/screen.h"

#include "common/boot_source.hpp"
#include "common/driver_controls.hpp"
#include "common/driver_panel.hpp"
#include "common/fault_latch_hook.hpp"
#include "common/log_copy_line.hpp"
#include "common/screen_paint.hpp"
#include "common/screen_text.hpp"
#include "common/screen_touch.hpp"
#include "programs/drive_panel.hpp"
#include "robots/mechanism_table.hpp"
#include "team/robot_two_driver.hpp"
#include "shulib/calibration/boot_check.hpp"
#include "shulib/calibration/calibrate_panel.hpp"
#include "shulib/core/check.hpp"
#include "shulib/diag/build_info.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/diag/session_info.hpp"
#include "shulib/hal/controller.hpp"
#include "shulib/hal/motor_group.hpp"
#include "shulib/hal/line_display.hpp"
#include "shulib/hal/pros/device_provider.hpp"
#include "shulib/hal/pros/line_display.hpp"
#include "shulib/hal/pros/tick_pacer.hpp"
#include "shulib/hal/telemetry_sink.hpp"
#include "shulib/localization/drive_encoder_odometry.hpp"
#include "shulib/localization/localizer.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/motion/odo_stall_check.hpp"
#include "shulib/robot/robot.hpp"
#include "shulib/teleop/coupled_side_monitor.hpp"
#include "shulib/teleop/drive_protection.hpp"
#include "shulib/teleop/drivetrain_degradation.hpp"
#include "shulib/teleop/driver_control.hpp"
#include "shulib/teleop/input_shaping.hpp"
#include "shulib/teleop/mechanism_binding.hpp"
#include "shulib/teleop/teleop_loop.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::bench {
namespace {

/// Every serial line of this program carries this prefix — NEW at T4 Part 3, so a capture tells
/// this program from the old "shulib Drive" (`[R3B] booted: shulib DRIVE`).
constexpr const char* kTag = "DRV";

/// The SD copy's file on the card (BARE: the provider's ProsBlockSink owns the /usd/ prefix) —
/// one file per boot, as the old program's was.
constexpr const char* kLogCopyFile = "drive_log.txt";

/// The BUILD stamp (the tester's reasoning, 2026-08-18: the git hash names the COMMIT, this names
/// the BUILD, and only the second answers "did my upload actually land?"). The serial banner prints
/// it whole; the pages' header bar carries its time.
constexpr const char* kBuildStamp = __DATE__ " " __TIME__;
constexpr std::string_view kBuildTimeText = "BUILD " __TIME__;
static_assert(calibration::screenText(kBuildTimeText, calibration::kSmallCols)
                  && calibration::screenText(std::string_view{kDriveAutonomousReason},
                                             calibration::kSmallCols),
              "drive_program: the build time or the autonomous reason does not fit the SMALL budget");
static_assert(refusalBarFits(kDriveRefusedWord) && refusalFooterFits(kDriveRefusalFooter),
              "drive_program: the refusal screen's bar word or footer does not fit the painter");

/// How long a protection cut holds — the config the driver file's loop runs with (its cutTicks
/// times its tick), what the PROTECTION row says; never retyped.
constexpr double kCutSeconds =
    RobotTwoDriver::kLoop.protection.cutTicks * RobotTwoDriver::kLoop.tickPeriod.value();

// ═══ PROS's READS: one line each, the meaning in host-tested code ═════════════════════════════
/// One read of the touch screen, reduced by src/common/screen_touch.hpp's screenTouchFrom().
ScreenTouch readTouch() {
    return screenTouchFrom(pros::c::screen_touch_status(), static_cast<int>(pros::E_TOUCH_ERROR));
}

// ═══ THE RUN STATUS — one sentence, said at boot on serial ═════════════════════════════
constexpr const char* kRunStatus =
    "THIS VERSION of \"shulib Drive\" (T4 Part 3: robot two's driver file on robot::Robot) has not "
    "run on hardware. The program it replaced in slot 1 drove robot two open-loop on 2026-09-17, "
    "and the library program it retired drove it from the calibration record on 2026-09-25. No "
    "library-commanded motion has run (R3b open). Its claws (the pivots on 6 and 7, the pneumatic "
    "claw on 3-wire A) have never been driven.";

// ═══ TermSink CUTS A MESSAGE AT 200 BYTES (diag/term_sink.hpp, kMaxMessageBytes) ════════════
// Four boot lines were cut on the brain on 2026-09-25 (the port check, the run-status line, the
// stall note, the dead-port tolerance line). A line this program owns is therefore either a
// literal checked below to be <= 200 bytes, or goes through logWrapped(), which splits it at
// spaces into pieces of at most kLogPieceCols characters — the second and later prefixed "... " —
// each its own log() call, so every word reaches the serial log (and the SD copy).
constexpr std::size_t kLogMessageBytes = 200;  // term_sink.hpp's kMaxMessageBytes (private there)
constexpr std::size_t kLogPieceCols = 196;     // + "... " is exactly kLogMessageBytes
static_assert(kLogPieceCols + 4 <= kLogMessageBytes, "a continuation piece would be cut");

void logWrapped(hal::ITelemetrySink& sink, hal::LogLevel level, const char* text) {
    const auto w = calibration::wrapWords<kLogPieceCols, 4>(text != nullptr ? text : "");
    for (std::size_t i = 0; i < w.count; ++i) {
        if (i == 0) {
            sink.log(level, kTag, w.line[0]);
            continue;
        }
        char piece[kLogPieceCols + 8];
        std::snprintf(piece, sizeof piece, "... %s", w.line[i]);
        sink.log(level, kTag, piece);
    }
    if (!w.complete) {
        sink.log(level, kTag, "... (the rest of this line did not fit four log lines)");
    }
}

// The fixed boot lines, each <= kLogMessageBytes so TermSink never cuts them (checked below).
constexpr std::string_view kProvisionalLine =
    "gains and budgets are PROVISIONAL (HA-45/50/51/52): speeds go through the feedforward and are "
    "capped by the budgets -- R5's measurement owes them, so the feel is not final";
constexpr std::string_view kAbsentDevicesLine =
    "absent devices (deliberate): gps=ABSENT tags=ABSENT vision=ABSENT -- robot two has no GPS, "
    "and its AI Vision camera (port 3) has no adapter yet; nothing is polled over an absent source";
constexpr std::string_view kDeadPortLine1 =
    "dead-port tolerance at BOOT (R3d Part 2): a recorded port the registry reports EMPTY becomes "
    "an absent group member under the boot check's policy.";
constexpr std::string_view kDeadPortLine2 =
    "dead-port tolerance MID-RUN (T4 Part 2, the library's driver loop): a drive port reading "
    "frozen while its side turns is dropped after 250 ms; 3+ dead in total stops the drive for "
    "the run";
constexpr std::string_view kOpcontrolLive =
    "opcontrol(): robot two's driver file is live -- teleop::runDriverControl over "
    "Chassis::drive(Body) and two MotorGroups, with the library's drive protections";
static_assert(kProvisionalLine.size() <= kLogMessageBytes
                  && kAbsentDevicesLine.size() <= kLogMessageBytes
                  && kDeadPortLine1.size() <= kLogMessageBytes
                  && kDeadPortLine2.size() <= kLogMessageBytes
                  && kOpcontrolLive.size() <= kLogMessageBytes,
              "a fixed boot line would be cut by TermSink at 200 bytes -- split it");

/// What the boot's driver line says about the feel — "continuous deadband 0.05, no expo, no slew"
/// on all three axes — checked against the profile the driver file actually runs, so a change to
/// the profile, or to the library's defaults under it, cannot leave the line saying the old numbers.
constexpr bool driverLineMatchesFeel() {
    const teleop::DriverProfile p = RobotTwoDriver::kFeel;
    const teleop::AxisShaping* axes[3] = {&p.forward, &p.left, &p.yaw};
    for (const teleop::AxisShaping* a : axes) {
        if (a->deadband != 0.05 || a->expo != 0.0) {
            return false;
        }
    }
    return !(p.slew.ratePerSecond > 0.0);
}
static_assert(driverLineMatchesFeel(),
              "the driver line says deadband 0.05, no expo, no slew: reword it with the driver file's "
              "feel");

// ═══ PROGRAM STATE (header: one function-local static) ═════════════════════════════════
struct DriveState {
    BootOutcome boot{};
    // The provider BEFORE the robot: the robot borrows the provider's devices, so it must be
    // destroyed first (reverse declaration order), and the provider must outlive it.
    std::optional<hal::pros::ProsDeviceProvider> devices{};
    std::optional<robot::Robot> robot{};
    // Robot two's driver file — the buttons, the feel and the loop — bound ONCE, inside
    // driveInitialize()'s try; held here, never on runDrive()'s stack, because the loop holds the
    // bindings by address and PROS deletes the opcontrol task on a field disable. Its constructor
    // cannot throw and touches no hardware (header, STORAGE).
    RobotTwoDriver driver{};
    // The controller's three lines (emplaced after the graph is built; touches no hardware).
    std::optional<hal::pros::ProsLineDisplay> lcd{};
    char portMap[diag::kMaxPortMapBytes]{};
    char refusalHeadline[64]{};
    char refusalWhy[192]{};
};

DriveState& state() {
    static DriveState s;
    return s;
}

/// The refusal screen (src/common/screen_paint.hpp's paintRefusalScreen(): the red bar with
/// kDriveRefusedWord and the BUILD time, the headline, the reason, kDriveRefusalFooter) and, unless
/// `serialLine` is false (refuseBuilt() said it through the provider's sink already), its serial
/// line, `[DRV] REFUSED: <headline> -- <why>`. Nothing is powered: no graph was built, and every
/// adapter the provider may have made was released.
void paintRefusal(bool serialLine = true) {
    DriveState& st = state();
    paintRefusalScreen(RefusalScreen{.tag = kTag,
                                     .barWord = kDriveRefusedWord.data(),
                                     .buildTime = kBuildTimeText.data(),
                                     .headline = st.refusalHeadline,
                                     .why = st.refusalWhy,
                                     .footer = kDriveRefusalFooter,
                                     .logToSerial = serialLine});
}

/// A refusal at boot: remembered (opcontrol() paints it again — a state screen may have replaced
/// it), painted, and said on serial.
void refuse(const char* headline, const char* why) {
    DriveState& st = state();
    std::snprintf(st.refusalHeadline, sizeof st.refusalHeadline, "%.63s", headline);
    std::snprintf(st.refusalWhy, sizeof st.refusalWhy, "%.191s", why);
    paintRefusal();
}

/// A refusal decided AFTER the provider was built: said through the provider's sink FIRST —
/// serial and the SD copy, which this boot already truncated, so the card keeps why this boot
/// refused (src/common/log_copy_line.hpp's logBootRefusal(); the T4-3 review) — then the graph
/// released (no binding may outlive the mechanisms it points at, nor the Robot the provider), then
/// remembered and painted with no serial line of its own (the sink said it). With NO provider (a
/// throw inside its own construction) nothing was said, so the painter says it on serial (the T4-3
/// verification: a refusal said nowhere on serial).
void refuseBuilt(const char* headline, const char* why) {
    DriveState& st = state();
    std::snprintf(st.refusalHeadline, sizeof st.refusalHeadline, "%.63s", headline);
    std::snprintf(st.refusalWhy, sizeof st.refusalWhy, "%.191s", why);
    const bool logged = st.devices.has_value();
    if (logged) {
        logBootRefusal(st.devices->telemetry(), kTag, st.refusalHeadline, st.refusalWhy);
    }
    st.driver.unbind();
    st.robot.reset();
    st.devices.reset();
    paintRefusal(!logged);
}

// ═══ THE SCREEN: the pages, the page flip and the controller's lines ══════════════════════
/// The program's screen (header, THE SCREEN AND THE CONTROLLER), called once per tick by the driver
/// file's loop with a FOURTH argument: the drive protections' state. Every tick it reads the touch
/// (a flip repaints at once); every kRefreshTicks (200 ms — live numbers at most 5 Hz, R3d Part 2F's
/// rule) it reads the robot, the boot and the protections, builds every row the three pages show
/// (src/programs/drive_panel.hpp) and repaints what changed on the visible page; every
/// kLcdEveryTicks it hands the controller's facts to the ONE chooser and writes at most ONE line.
class DriveScreen {
public:
    static constexpr int kRefreshTicks = 20;
    static constexpr int kLcdEveryTicks = 5;

    /// `lcd` is the controller's three lines; `atStart` the touch read when the screen first
    /// appears (a tap made before it is never replayed).
    DriveScreen(const robot::Robot& r, const BootOutcome& boot, const RobotTwoDriver& driver,
                hal::ILineDisplay& lcd, const ScreenTouch& atStart)
        : r_{r},
          boot_{boot},
          driver_{driver},
          lcd_{lcd},
          pager_{atStart.sample.releaseCount, atStart.pressCount} {
        const auto groups = r_.chassis().deps().motorGroups;
        SHULIB_PRECONDITION(groups.size() == 2 && groups[0] != nullptr && groups[1] != nullptr,
                            "DriveScreen: a tank robot has exactly two motor groups");
        left_ = groups[0];
        right_ = groups[1];
        rows_.buildTime = kBuildTimeText;
        rows_.status = driveRow(kDriveScreenRunStatus, ui::Tone::Amber);
        // WHERE THE NUMBERS CAME FROM, and what is dead: a typed table on a rebuilt robot is a
        // belief, not a measurement, and the driver must be able to see which one is driving them
        // (drive_panel.hpp's driveSourceRows() words and colours them).
        driveSourceRows(rows_, boot_.source == calibration::BootSource::UseTable,
                        boot_.verdict.decision == calibration::BootDecision::Degraded,
                        boot_.verdict.reason);
    }

    void operator()(const math::ChassisSpeeds& command, bool connected,
                    const teleop::DriveShaperState& shaper,
                    const teleop::DriveProtectionState& protection) {
        ++tick_;
        const double now = r_.context().clock().now().value();
        const ScreenTouch touch = readTouch();
        const bool flipped = pager_.touch(touch.sample, touch.pressCount, now);
        if (tick_ == 1 || flipped || tick_ % kRefreshTicks == 0) {
            refreshRows(command, connected, shaper, protection);
            paint(driveView(pager_.page(), rows_, pager_.armed(now)));
        }
        if (tick_ % kLcdEveryTicks == 1) {  // the FIRST tick too
            refreshLcd(connected, protection, now);
        }
    }

private:
    /// Every live row the three pages show: the robot, the boot and the protections read into
    /// DriveLiveFacts here, the rows made from them by drive_panel.hpp's driveLiveRows() (pure,
    /// host-tested: which row says what, in which colour).
    void refreshRows(const math::ChassisSpeeds& command, bool connected,
                     const teleop::DriveShaperState& shaper,
                     const teleop::DriveProtectionState& protection) {
        const math::Pose2d pose = r_.localizer().pose();
        const localization::DriveEncoderOdometry* odom = r_.driveOdometry();
        const diag::FaultLatch& faults = r_.faults();
        const bool freed = driver_.bindings().anyReleasedLeftoverClaim();
        const ClawRow claw = clawRowText(kMechanisms, r_, freed);
        DriveLiveFacts f{};
        f.cutSeconds = kCutSeconds;
        f.connected = connected;
        f.cmdVx = command.vx().value();
        f.cmdOmega = command.omega().value();
        f.estX = pose.x().value();
        f.estY = pose.y().value();
        f.estHeadingDeg = pose.heading().radians() * 180.0 / math::Angle::kPi;
        f.quality = r_.localizer().qualityClass();
        f.mode = RobotTwoDriver::kFeel.mode;
        f.fieldCentric = shaper.fieldCentric;
        f.hasOdometry = odom != nullptr;
        f.headingDisagreement = odom != nullptr ? odom->lastHeadingDisagreement() : 0.0;
        f.batteryVolts = r_.context().battery().voltage().value();
        f.travelLeft = odom != nullptr ? odom->lastSideTravel().left.value() : 0.0;
        f.travelRight = odom != nullptr ? odom->lastSideTravel().right.value() : 0.0;
        f.faultCount = faults.hasFault() ? faults.faultCount() : 0;
        f.firstFault = faults.firstFault();
        f.lastFault = faults.lastFault();
        f.clawText = claw.text;
        f.clawAbsent = anyClawAbsent(r_);
        f.clawFreed = freed;
        driveLiveRows(rows_, f, protection, *left_, *right_);
    }

    /// The visible page: what driveRepaint() says changed since the last paint, and nothing else.
    void paint(const DriveView& next) {
        const DriveRepaint plan = driveRepaint(painted_, next, havePainted_);
        if (plan.whole) {
            clearPanel();
            paintBar(next.bar, next.header);
            paintLine(next.barRight, toneRgb(next.bar));
            for (std::size_t i = 0; i < next.lineCount; ++i) {
                paintLine(next.lines[i], kPanelBg);
            }
            for (std::size_t i = 0; i < next.buttonCount; ++i) {
                paintButton(next.buttons[i].rect, next.buttons[i].label, next.buttons[i].armed,
                            ui::Tone::Blue);
            }
        } else {
            if (plan.header) {
                paintLine(next.header, toneRgb(next.bar));
            }
            if (plan.barRight) {
                paintLine(next.barRight, toneRgb(next.bar));
            }
            for (std::size_t i = 0; i < kDriveMaxLines; ++i) {
                if ((plan.lines & (1u << i)) == 0) {
                    continue;
                }
                // A row that went is painted EMPTY over its old place (driveLineToPaint()).
                paintLine(driveLineToPaint(painted_, next, i), kPanelBg);
            }
            for (std::size_t i = 0; i < next.buttonCount; ++i) {
                if ((plan.buttons & (1u << i)) != 0) {
                    paintButton(next.buttons[i].rect, next.buttons[i].label,
                                next.buttons[i].armed, ui::Tone::Blue);
                }
            }
        }
        painted_ = next;
        havePainted_ = true;
    }

    /// The controller's three lines: the facts read here, the words chosen by the ONE chooser
    /// (src/common/driver_panel.hpp), ONE changed line written through its writer (line 2 first,
    /// at most one write per 0.1 s).
    void refreshLcd(bool connected, const teleop::DriveProtectionState& protection, double now) {
        ControllerFacts f{};
        f.batteryVolts = r_.context().battery().voltage().value();
        f.headingDegrees = r_.localizer().pose().heading().radians() * 180.0 / math::Angle::kPi;
        f.hasFault = r_.faults().hasFault();
        f.lastFault = r_.faults().lastFault();
        f.connected = connected;
        f.degraded = boot_.verdict.decision == calibration::BootDecision::Degraded;
        f.clawAbsent = anyClawAbsent(r_);
        addProtectionFacts(f, protection);  // REFUSED, CUT, DEGRADED mid-run, the dead ports
        const ControllerScreen want = controllerScreenLines(f);
        const int row = lcdWriter_.next(want, now);  // ONE line, line 2 first, >= 0.1 s apart
        if (row >= 0) {
            lcd_.setLine(row, want.line[row]);
        }
    }

    const robot::Robot& r_;
    const BootOutcome& boot_;
    const RobotTwoDriver& driver_;
    hal::ILineDisplay& lcd_;
    const hal::MotorGroup* left_ = nullptr;
    const hal::MotorGroup* right_ = nullptr;
    int tick_ = 0;
    DriveRows rows_{};
    DrivePager pager_;
    DriveView painted_{};
    bool havePainted_ = false;
    ControllerLineWriter lcdWriter_{};
};

/// The boot lines after the session header — every line "shulib Teleop" said that is still true,
/// in its order, each within TermSink's 200 bytes; the driver line and the protections line are
/// GENERATED from the driver file's constants and the library's defaults it runs with, never
/// retyped.
void logBootLines(robot::Robot& r, hal::ITelemetrySink& log, const BootOutcome& boot,
                  const hal::pros::ProsDeviceProvider& devices) {
    char line[240];
    // THE SD COPY (N5 (a)): where it is, or that it did not open and why (no card, or a card that
    // would not take the file) — once (the T5 rule), in the words src/common/log_copy_line.hpp
    // makes for both programs that keep one.
    {
        const LogCopyLine copy = logCopyBootLine(
            logCopyOpenFrom(devices.serialCopyOpen(), pros::c::usd_is_installed() != 0, false,
                            false),
            devices.serialCopyPath(), devices.serialCopyDroppedLines());
        log.log(copy.warn ? hal::LogLevel::Warn : hal::LogLevel::Info, kTag, copy.text);
    }
    const hal::DrivetrainDescription& d = r.config().drivetrain.tank.description;
    {
        char described[128];
        d.describe(described, sizeof described);
        std::snprintf(line, sizeof line, "library graph BUILT from the %s: %.120s",
                      boot.source == calibration::BootSource::UseRecord
                          ? "CALIBRATION RECORD on the SD card"
                          : "TYPED TABLE (src/robots/chassis_table.hpp)",
                      described);
        log.log(hal::LogLevel::Info, kTag, line);
    }
    {
        char banner[240];
        r.describe(banner, sizeof banner);
        std::snprintf(line, sizeof line, "robot::Robot: %.220s", banner);
        logWrapped(log, hal::LogLevel::Info, line);
    }
    const auto groups = r.chassis().deps().motorGroups;
    std::snprintf(line, sizeof line,
                  "%u+%u slots behind two MotorGroups, %u+%u PRESENT -- median reads are over the "
                  "PRESENT members; the sign lives inside each port entry and PROS applies it once",
                  static_cast<unsigned>(groups[0]->memberCount()),
                  static_cast<unsigned>(groups[1]->memberCount()),
                  static_cast<unsigned>(groups[0]->presentCount()),
                  static_cast<unsigned>(groups[1]->presentCount()));
    log.log(hal::LogLevel::Info, kTag, line);
    std::snprintf(line, sizeof line, "description provenance: %.190s", d.provenance);
    log.log(hal::LogLevel::Info, kTag, line);
    std::snprintf(line, sizeof line, "boot source: %.200s", boot.sourceWhy);
    log.log(driveBootLevel(boot.source == calibration::BootSource::UseRecord), kTag, line);
    {
        char check[340];
        std::snprintf(check, sizeof check, "port check: %s", boot.verdict.reason);
        logWrapped(log, driveBootLevel(boot.verdict.decision == calibration::BootDecision::Ok),
                   check);
    }
    {
        char status[520];
        std::snprintf(status, sizeof status, "HARDWARE-UNVALIDATED: %s", kRunStatus);
        logWrapped(log, hal::LogLevel::Warn, status);
    }
    log.log(hal::LogLevel::Warn, kTag, kProvisionalLine);
    log.log(hal::LogLevel::Info, kTag, kAbsentDevicesLine);
    // The §2 ruling, said ONCE at boot (odo_stall_check.hpp carries the reasoning) — in pieces,
    // because the library's note is longer than TermSink's 200 bytes.
    logWrapped(log, hal::LogLevel::Warn, motion::kNoIndependentStallSourceNote);
    if (driveDegradedBootLine(line, sizeof line, boot.built.absent().size(),
                              boot.verdict.reason)) {
        logWrapped(log, hal::LogLevel::Warn, line);
    }
    log.log(hal::LogLevel::Info, kTag, kDeadPortLine1);
    log.log(hal::LogLevel::Info, kTag, kDeadPortLine2);
    const motion::MotionConfig& cfg = r.chassis().motionConfig();
    std::snprintf(line, sizeof line,
                  "facade alive: strafeAuthority=%.2f (tank: 0.00); stall check canDetectStall=%s; "
                  "budgets lin %.0f in/s ang %.2f rad/s wheel %.0f in/s; ff kS %.2f kV %.4f kA %.4f",
                  r.chassis().strafeAuthority(),
                  cfg.stall.independentMotionSource ? "true" : "false (by config)",
                  cfg.maxLinearSpeed.value(), cfg.maxAngularSpeed.value(),
                  cfg.maxWheelSpeed.value(), cfg.wheelFf.kS, cfg.wheelFf.kV, cfg.wheelFf.kA);
    log.log(hal::LogLevel::Info, kTag, line);
    // THE DRIVER LINE: generated from the driver file (src/team/robot_two_driver.hpp), so it cannot
    // drift from what the loop is given.
    std::snprintf(line, sizeof line,
                  "driver control: robot two's driver file (src/team/robot_two_driver.hpp) -> "
                  "teleop::runDriverControl + DriveShaper, the default profile (continuous deadband "
                  "0.05, no expo, no slew -- HA-112, unfelt); re-zero button %.5s; mode %.12s",
                  teleop::controllerButtonWord(RobotTwoDriver::kLoop.rezeroButton),
                  modeWord(RobotTwoDriver::kFeel.mode));
    logWrapped(log, hal::LogLevel::Info, line);
    // THE PROTECTIONS LINE: generated from the config the driver file's loop runs with
    // (DriverLoopConfig::protection, the library's defaults — the driver file names none) and the
    // groups' monitors' window, never retyped (the T4 brief's D4; drive_panel.hpp's
    // driveProtectionsLine() words it, host-tested against those configs).
    {
        char protections[360];
        (void)driveProtectionsLine(protections, sizeof protections,
                                   RobotTwoDriver::kLoop.protection,
                                   RobotTwoDriver::kLoop.tickPeriod.value(),
                                   teleop::SideMonitorConfig{}.persistTicks);
        logWrapped(log, hal::LogLevel::Info, protections);
    }
    // THE CLAWS: one line per declared mechanism, BUILT or ABSENT with the provider's own sentence
    // (src/common/driver_panel.hpp words them from the table AND the driver file's controls — the
    // competition program prints the same lines; Station J.3.1 / J.9.1 quote them). An ABSENT one is a warning: its buttons do nothing, and the
    // drive runs anyway.
    for (std::size_t i = 0; i < motorEntryCount(kMechanisms); ++i) {
        logWrapped(log, r.builtMechanismAt(i) != nullptr ? hal::LogLevel::Info : hal::LogLevel::Warn,
                   motorClawBootLine(kMechanisms, RobotTwoDriver::kHolds, r, i).text);
    }
    for (std::size_t j = 0; j < pneumaticEntryCount(kMechanisms); ++j) {
        logWrapped(log,
                   r.builtMechanismAt(mechanismConfigIndexOfPneumatic(kMechanisms, j)) != nullptr
                       ? hal::LogLevel::Info
                       : hal::LogLevel::Warn,
                   pneumaticClawBootLine(kMechanisms, RobotTwoDriver::kToggles, r, j).text);
    }
}

}  // namespace

void driveInitialize(const char* variantBeacon, const char* competitionStatusBits) {
    DriveState& st = state();
    std::printf("\n[%s] booted: shulib DRIVE build [%s] -- robot two's DRIVER FILE drives it\n"
                "[%s] (src/team/robot_two_driver.hpp) through robot::Robot: two hal::MotorGroups,\n"
                "[%s] drive-encoder odometry and Chassis::drive(), built from the CALIBRATION RECORD\n"
                "[%s] on the SD card, with the library's drive protections. Forward is the record's\n"
                "[%s] front. This is NOT the old shulib Drive (its tag was [R3B]).\n"
                "[%s] %s\n"
                "[%s] BUILD STAMP %s\n",
                kTag, variantBeacon, kTag, kTag, kTag, kTag, kTag, kRunStatus, kTag, kBuildStamp);
    std::printf("[%s] competition status at boot: %s -- the loop runs only in DRIVER CONTROL, "
                "enabled; any other state paints the screen with the reason.\n",
                kTag, competitionStatusBits);
    std::fflush(stdout);

    // ── which description drives this robot: decided, and printed, before any adapter exists.
    st.boot = decideBootSource(kTag);
    if (!st.boot.ok) {
        refuse(st.boot.headline, st.boot.why);
        return;
    }

    // ── the graph, at its boundary: the provider (and its SD copy), validate() AND the
    //    construction AND the driver file's bind(), once, in ONE try. The options are the boot
    //    check's absent ports and nothing else, through `built` — boot_source.hpp's one door — so a
    //    port it found dead is never requested and is named once.
    try {
        st.devices.emplace(kLogCopyFile);
        const robot::BuildVerdict verdict =
            robot::validate(st.boot.built.config, *st.devices, st.boot.built.buildOptions());
        if (!verdict.ok) {
            refuseBuilt("the library graph REFUSED to build", verdict.why);
            return;
        }
        st.robot.emplace(st.boot.built.config, *st.devices, st.boot.built.buildOptions());
        // Robot two's driver file binds its buttons BY NAME against the robot just built (a binding's
        // constructor may throw, and nothing that can throw runs after driver control starts). An
        // ABSENT mechanism gets no binding; a robot these controls are not for is refused.
        char why[192] = "";
        if (!st.driver.bind(*st.robot, why, sizeof why)) {
            refuseBuilt("the driver file REFUSED this robot", why);
            return;
        }
    } catch (const PreconditionError& e) {
        // refuseBuilt() unbinds FIRST: no binding may outlive the mechanisms it points at.
        refuseBuilt("the library graph REFUSED to build", e.what());
        return;
    }
    robot::Robot& r = *st.robot;
    hal::ITelemetrySink& log = st.devices->telemetry();
    attachFaultLatch(&r.faults());  // a precondition after boot latches FaultCode::Precondition
    st.lcd.emplace();               // the controller's three lines (touches no hardware)
    log.log(hal::LogLevel::Info, kTag, variantBeacon);

    // ── the §18.5 session header: provenance FIRST, a missing hash LOUD; the port map GENERATED
    //    from the config by robot::Robot::portMap() — never retyped.
    r.portMap(st.portMap, sizeof st.portMap);
    diag::SessionInfo session{};
    session.buildHash = diag::compiledBuildHash();
    session.routineId = "(none: driver control only)";
    session.alliance = "";
    session.side = "";
    session.portMap = st.portMap;
    diag::emitSessionHeader(log, session, r.context().battery().voltage());

    logBootLines(r, log, st.boot, *st.devices);
}

void driveAutonomous() {
    DriveState& st = state();
    if (!st.robot.has_value()) {
        return;  // refused at boot: nothing was built, and the boot said why (serial)
    }
    char line[96];
    std::snprintf(line, sizeof line, "autonomous(): %s", kDriveAutonomousReason);
    st.devices->telemetry().log(hal::LogLevel::Warn, kTag, line);
}

void runDrive() {
    DriveState& st = state();
    if (!st.robot.has_value()) {
        // Refused at boot: no graph exists, nothing is powered. Paint the refusal again (a state
        // screen may have replaced it; its serial line says why once more) and idle.
        paintRefusal();
        while (true) {
            pros::delay(250);
        }
    }
    robot::Robot& r = *st.robot;
    hal::ITelemetrySink& log = st.devices->telemetry();
    log.log(hal::LogLevel::Info, kTag, kOpcontrolLive);
    DriveScreen screen{r, st.boot, st.driver, *st.lcd, readTouch()};
    st.driver.run(r, st.devices->pacer(), [] { return pros::c::competition_is_disabled() != 0; },
                  screen);
    // This line races PROS's deletion of this task (HA-161 (a)): one that lands while the line is
    // being copied to the card ends the card's copy for the rest of the boot (serial keeps every
    // line; diag/log_copy_sink.hpp, "A writer that never comes back").
    char line[200];
    std::snprintf(line, sizeof line,
                  "opcontrol(): field DISABLED -- the driver file's loop exited cleanly after "
                  "cancel() (0 V + Brake); each claw pivot made safe, the pneumatic claw left as it "
                  "was; %lu screen prints so far",
                  static_cast<unsigned long>(g_screenPrints));
    log.log(hal::LogLevel::Info, kTag, line);
}

}  // namespace shulib::bench
