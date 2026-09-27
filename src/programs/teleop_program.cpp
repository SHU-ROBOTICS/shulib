// teleop_program.cpp — "shulib Teleop": THE LIBRARY drives robot two, built on robot::Robot
// (G1 Part 2b, 2026-09-26).
//
// ═══ WHAT CHANGED AT G1 PART 2b, AND WHAT DID NOT ════════════════════════════════════
// Until Part 2b this program was ~430 lines inside src/main.cpp: a hand-wired object graph
// (`TankMotors`, `TankRobot`), a port-map builder (`tankPortMapString`), a panel (`TankPanel`),
// the boot-source block, and a 22-line loop (`runTeleopLoop`) mapping the sticks through the raw
// R1a mapping. Now:
//   * the GRAPH is robot::Robot, built from a config::RobotConfig over a
//     hal::pros::ProsDeviceProvider — the library's own composition root (G1 Parts 1 and 2a).
//     Same objects, same order, same arguments as the hand-wired TankRobot (robot.hpp's ruling
//     B4, pinned bit-for-bit on the host plant by test/robot_builder_test.cpp);
//   * the BOOT SOURCE is src/common/boot_source.{hpp,cpp}, shared with the competition program, its
//     decisions unchanged;
//   * the LOOP is teleop::runDriverControl with the DriveShaper feel layer (T2), default profile
//     ArcadeBody — the ONE feel change the team lead chose (the brief's ruling M4, restated
//     2026-09-26 "switch"): identical to the old mapping outside the stick's deadband, and a
//     continuous deadband instead of a hard one around the centre (test/teleop_loop_test.cpp,
//     the brief's test 5). Field-centric is selectable (kFieldCentric) but OFF (ruling M5);
//   * the PANEL is re-laid out to fit the screen's SMALL budget (the brief's A3), painted only
//     where it changed, live numbers at most 5 Hz, every string through src/common/screen_text.hpp's
//     printText().
// What did NOT change: the boot decisions, the dead-port policy (applied once, by the boot check),
// the stall-check wiring (robot::Robot::effectiveTuning does what tankChassisConfig() did), the
// provisional gains and budgets, the session header's port map (robot::Robot::portMap() prints
// what tankPortMapString() printed, byte for byte), the controller-screen lines, and every boot
// line that is still true (the brief's A10 diff, in docs/internal/chunks/G1-PROGRESS.md).
//
// ═══ THE CLAWS (T3 Part 2, 2026-09-26) ══════════════════════════════════════════════
// Robot two's claws are driven HERE, in slot 2 — never in "shulib Drive" (slot 1). The boot
// source converts src/robots/mechanism_table.hpp into the config (src/common/boot_source.hpp), so robot::Robot
// builds the two pivots (ports 6 and 7) and the pneumatic claw (3-wire A) — the claw's line made
// at its START level, CLOSED — and the port check names the pivots as claw pivots, never as NEW
// motors; a record that lists a claw port as a DRIVE port refuses the boot ("recalibrate").
// teleopInitialize() binds them inside its try (src/programs/teleop_claws.hpp: R1/R2 -> port 6, L1/L2 ->
// port 7, 12 V, HOLD on release; A toggles the claw, its initial level the SAME start level) and
// validates the set against the re-zero button, which moved A -> Y (the team lead's ruling); an
// ABSENT claw gets no binding and says so, and the drive runs. runTeleop() hands the bindings to
// the library loop, which releases a claim an operation left on a claw at entry and logs it (the
// team lead's ruling). The panel's two fault rows are one row now, and the freed row is the claws'
// (ruling R3). NOTHING OF THIS HAS RUN ON A ROBOT (worksheet Station J).
//
// ═══ RUN STATUS ═════════════════════════════════════════════════════════════════════
// THIS VERSION has not run on hardware (kRunStatus below says it at every boot). The PREVIOUS
// version drove robot two from the calibration record on 2026-09-25 — open-loop, under a driver;
// no library-COMMANDED motion (a moveTo) has run anywhere, so R3b stays open.
//
// ═══ WHICH END IS FORWARD ═══════════════════════════════════════════════════════════
// Stick up drives toward the end that LED the Calibrate push saved in the record: on robot two
// since 2026-09-25, the AI Vision (camera, port 3) end — the 11/20 end. "shulib Drive" and Bench
// Tests' MOTOR WATCH use the typed table, whose front is the 15/16 end, so the same stick drives
// OPPOSITE ways in slots 1 and 2 (R3a-BENCH-WORKSHEET.md Station F says so before the first drive).
//
// ═══ ORDER AT BOOT (initialize()) ═══════════════════════════════════════════════════
// src/main.cpp installs the precondition handler FIRST, then calls teleopInitialize(): the banner
// and competition status (from main.cpp's arguments — the beacon stays a literal in
// src/build_selection.hpp's one chain, the brief's A2 and G1 Part 2c final), the boot source (which prints its own decision lines), then the
// graph, ONCE, into std::optional storage inside the graph boundary's try — validate() AND the
// construction, because a provider configuration conflict is a precondition — then the fault
// latch handed to main.cpp's handler (fault_latch_hook.hpp), the session header, and the boot
// lines. The provider starts the IMU's calibration when the builder first asks for it, so this
// program never calls calibrate() itself.
//
// ═══ STORAGE: one function-local static of EMPTY optionals ══════════════════════════
// robot::Robot and the provider are non-movable, and construction order across translation units
// is undefined, so neither is built at namespace scope; a function-local static whose constructor
// THROWS would be re-attempted on the next call, so the static below holds only empty optionals
// (a constructor that cannot throw) and teleopInitialize() emplaces into them, once. Function-local
// rather than namespace-scope for one more reason: this file compiles in all six builds, and a
// namespace-scope object with a non-trivial destructor registers that destructor from a static
// initializer the linker must keep — dragging the whole graph's code into the tester's and the
// other programs' packages. A function-local static is initialized on first call, so in the builds
// that never call it, --gc-sections drops everything here.
//
// Compiled in all six builds (the brief's A12): nothing here reads a build switch (only
// src/build_selection.hpp's one chain does — G1 Part 2c final), no beacon literal lives here (landmine 10), and every symbol it needs from main.cpp
// (attachFaultLatch) is defined in all six. ASCII only in every runtime string; every screen
// string goes through printText(); only a literal "%s" reaches a PROS print.

#include "programs/teleop_program.hpp"

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <optional>
#include <string_view>

#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "pros/screen.h"

#include "common/boot_source.hpp"
#include "common/fault_latch_hook.hpp"
#include "robots/mechanism_table.hpp"
#include "common/screen_text.hpp"
#include "programs/teleop_claws.hpp"
#include "shulib/calibration/boot_check.hpp"
#include "shulib/calibration/calibrate_panel.hpp"
#include "shulib/core/check.hpp"
#include "shulib/diag/build_info.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/diag/session_info.hpp"
#include "shulib/hal/controller.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/hal/motor_group.hpp"
#include "shulib/hal/pros/controller.hpp"
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
#include "shulib/teleop/driver_control.hpp"
#include "shulib/teleop/drive_modes.hpp"
#include "shulib/teleop/input_shaping.hpp"
#include "shulib/teleop/mechanism_binding.hpp"
#include "shulib/teleop/teleop_loop.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::bench {
namespace {

using calibration::TextSize;
using calibration::kMediumCols;
using calibration::kSmallCols;

/// Every serial line of this program carries the prefix it has carried since R3b Parts 1-3.
constexpr const char* kTag = "R3B";

/// The BUILD stamp (the tester's reasoning, 2026-08-18: the git hash names the COMMIT, this names
/// the BUILD, and only the second answers "did my upload actually land?"). The serial banner prints
/// it whole; the panel's header carries its time.
constexpr const char* kBuildStamp = __DATE__ " " __TIME__;
constexpr std::string_view kBuildTimeText = "BUILD " __TIME__;

// ═══ THE RUN STATUS — one sentence, said at boot on serial (the brief's A1, verbatim) ═════════
constexpr const char* kRunStatus =
    "THIS VERSION of \"shulib Teleop\" (G1 Part 2b, built on robot::Robot) has not run on "
    "hardware. The previous version (build v0.1.1-319-g2780462) drove robot two from the "
    "calibration record on 2026-09-25 -- eight starts, open-loop under driver control through "
    "the library's own graph; no library-commanded motion has run (R3b open). Its claws (T3: the "
    "pivots on 6 and 7, the pneumatic claw on 3-wire A) have never been driven.";

// ═══ DRIVER CONTROL — the choices this program makes (rulings M4/M5) ═══════════════════════
/// Field-centric is SELECTABLE, and OFF: the first hardware run of this version must not also be
/// the first run of a frame rotation (ruling M5). Set true to drive ArcadeField; the heading source
/// below is what it trusts, and it falls back to robot-centric (and LATCHES until a re-zero) the
/// moment that heading is not trusted.
constexpr bool kFieldCentric = false;
/// The driver's field-centric re-zero button ("the way the robot faces now is away from me").
/// Harmless in ArcadeBody, where a press only pre-arms the zero. **Y — the team lead's ruling of
/// 2026-09-26** (T3's addendum §0): A became the pneumatic claw's toggle, so the re-zero moved from
/// A to Y. The boot line names it from this constant (buttonWord below), never from a second
/// spelling.
constexpr hal::ControllerButton kRezeroButton = hal::ControllerButton::Y;
// TIED to the mechanism table's reserved button (src/robots/mechanism_table.hpp, kTeleopRezeroButton, whose
// own check 6 keeps every declared binding button off it): the two can never drift apart.
static_assert(kRezeroButton == kTeleopRezeroButton,
              "Teleop's re-zero button and src/robots/mechanism_table.hpp's kTeleopRezeroButton disagree");
// NEVER a binding button — robot two's claw pivots take L1/L2 and R1/R2, its claw takes A — and a
// re-zero on a claw button would re-zero the field frame every time the claw moved. Checked against
// both tables, and against the four shoulder buttons for any table (teleop::DriverLoopConfig's own
// default is L1 — a program must state its button, as this one does). validBindings() checks the
// same at run time, in teleopInitialize(), before the loop ever starts.
static_assert(mechanismButtonsAvoid(kRobotTwoMechanisms, kRezeroButton)
                  && mechanismButtonsAvoid(kMechanisms, kRezeroButton),
              "the re-zero button is a claw's binding button");
static_assert(kRezeroButton != hal::ControllerButton::L1 && kRezeroButton != hal::ControllerButton::L2
                  && kRezeroButton != hal::ControllerButton::R1
                  && kRezeroButton != hal::ControllerButton::R2,
              "the re-zero button may not be a shoulder button: T3 binds them to the claw pivots");

/// A controller button as the boot line names it.
constexpr const char* buttonWord(hal::ControllerButton b) {
    using B = hal::ControllerButton;
    switch (b) {
        case B::L1: return "L1";
        case B::L2: return "L2";
        case B::R1: return "R1";
        case B::R2: return "R2";
        case B::Up: return "Up";
        case B::Down: return "Down";
        case B::Left: return "Left";
        case B::Right: return "Right";
        case B::X: return "X";
        case B::B: return "B";
        case B::Y: return "Y";
        case B::A: return "A";
    }
    return "?";
}
constexpr std::size_t kLongestButtonWord = 5;  // "Right"

/// The driver's feel: the library's DEFAULT profile (deadband 0.05 continuous, no expo, no slew —
/// every number PROVISIONAL, A4 register HA-112; no driver has felt it) with the mode above.
constexpr teleop::DriverProfile teleopProfile() {
    teleop::DriverProfile p{};
    p.mode = kFieldCentric ? teleop::DriveMode::ArcadeField : teleop::DriveMode::ArcadeBody;
    return p;
}

/// What the boot's driver line (kDriverLine below) says about the feel — "continuous deadband 0.05,
/// no expo, no slew" on all three axes — checked against the profile this program actually runs, so
/// a change to the profile, or to the library's defaults under it, cannot leave the line saying
/// the old numbers.
constexpr bool driverLineMatchesProfile() {
    const teleop::DriverProfile p = teleopProfile();
    const teleop::AxisShaping* axes[3] = {&p.forward, &p.left, &p.yaw};
    for (const teleop::AxisShaping* a : axes) {
        if (a->deadband != 0.05 || a->expo != 0.0) {
            return false;
        }
    }
    return !(p.slew.ratePerSecond > 0.0);
}
static_assert(driverLineMatchesProfile(),
              "kDriverLine says deadband 0.05, no expo, no slew: reword it with the profile");

// ═══ TermSink CUTS A MESSAGE AT 200 BYTES (diag/term_sink.hpp, kMaxMessageBytes) ════════════
// Four boot lines were cut on the brain on 2026-09-25 (the port check, the run-status line, the
// stall note, the dead-port tolerance line). A line this program owns is therefore either a
// literal checked below to be <= 200 bytes, or goes through logWrapped(), which splits it at
// spaces into pieces of at most kLogPieceCols characters — the second and later prefixed "... " —
// each its own log() call, so every word reaches the serial log.
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
    "gains and budgets are PROVISIONAL (HA-45/50/51/52) -- expect it to FEEL different from "
    "shulib Drive (speeds via feedforward, capped by the budgets), which is R5's measurement "
    "showing, not a bug";
constexpr std::string_view kAbsentDevicesLine =
    "absent devices (deliberate): gps=ABSENT tags=ABSENT vision=ABSENT -- robot two has no GPS, "
    "and its AI Vision camera (port 3) has no adapter yet; nothing is polled over an absent source";
constexpr std::string_view kDeadPortLine1 =
    "dead-port tolerance: AT BOOT ONLY (R3d Part 2) -- a recorded port the registry reports EMPTY "
    "becomes an absent group member under the same policy shulib Drive uses.";
constexpr std::string_view kDeadPortLine2 =
    "dead-port tolerance: a port that dies AFTER boot is still shulib Drive's (slot 1): runtime "
    "absence detection is not in this graph yet";
// The driver line's fixed words; the button and field-centric's state are appended from the
// constants above (logBootLines), so neither can drift from what the loop is given.
constexpr std::string_view kDriverLine =
    "driver control: teleop::runDriverControl + DriveShaper, the default profile (continuous "
    "deadband 0.05, no expo, no slew -- HA-112, unfelt); re-zero button";
constexpr std::string_view kDriverLineFieldCentric = "; field-centric: ";
constexpr std::string_view kSelectableOn = "ON (selectable)";
constexpr std::string_view kSelectableOff = "OFF (selectable)";
constexpr std::string_view kOpcontrolLive =
    "opcontrol(): the LIBRARY's teleop loop is live on robot two -- teleop::runDriverControl, "
    "Chassis::drive(Body) over two MotorGroups; DriveShaper feel, default profile (HA-112)";
static_assert(kProvisionalLine.size() <= kLogMessageBytes && kAbsentDevicesLine.size() <= kLogMessageBytes
                  && kDeadPortLine1.size() <= kLogMessageBytes
                  && kDeadPortLine2.size() <= kLogMessageBytes
                  && kDriverLine.size() + 1 + kLongestButtonWord + kDriverLineFieldCentric.size()
                             + kSelectableOff.size()
                         <= kLogMessageBytes
                  && kOpcontrolLive.size() <= kLogMessageBytes,
              "a fixed boot line would be cut by TermSink at 200 bytes -- split it");

// ═══ THE SCREEN: colours, positions, the program's own words ═══════════════════════════
constexpr std::uint32_t kColBg = 0x101418, kColBar = 0x1E5AA8, kColText = 0xF0F0F0,
                        kColDim = 0x9AA4AE, kColGood = 0x39C36E, kColWarn = 0xE8B23A,
                        kColBad = 0xE2564A;
constexpr std::int16_t kBarBottom = 26;  // the header bar, rows 0..26 (Calibrate's)
constexpr std::int16_t kHeaderWordX = 8;
constexpr std::int16_t kHeaderWordY = 4;
constexpr std::int16_t kBuildX = 340;
constexpr std::int16_t kBuildY = 7;
constexpr std::int16_t kRowX = 8;           // every panel row starts here: the whole SMALL budget
constexpr std::int16_t kFirstRowY = 32;
constexpr std::int16_t kRowPitch = 16;      // SMALL's estimated line height (HA-136)
constexpr int kPanelEveryTicks = 20;        // 200 ms: live numbers at most 5 Hz
constexpr int kLcdEveryTicks = 20;          // 200 ms: and the controller screen only on change

/// The panel's rows, top to bottom. Row 0 and the two source rows never change after boot; the
/// rest are live. TWELVE, and a thirteenth fails the static_assert below: so when T3 Part 2 needed
/// a row for the claws, the two fault rows became one (the team lead's ruling R3, 2026-09-26).
enum PanelRow : int {
    kRowStatus = 0,  // static: this version's run status
    kRowCmd,         // the command, or NO CONTROLLER
    kRowEst,         // the fused estimate and its quality class (G.6.3 reads est x / hdg here)
    kRowDriver,      // the driver mode and field-centric's state (new at G1 Part 2b)
    kRowLeft,        // the LEFT group: volts, present/members, disagreeing
    kRowRight,       // the RIGHT group
    kRowXcheck,      // the encoder-vs-IMU heading cross-check and the battery
    kRowTravel,      // the last tick's per-side travel
    kRowFault,       // faults: none / FAULT n= first .. last — ONE row since T3 Part 2 (ruling R3)
    kRowClaws,       // the claws (T3 Part 2): each pivot's command word and temperature, the claw
    kRowSource1,     // static: where the description came from (G.6.1 / G.6.2 quote it)
    kRowSource2,     // static: its second line
    kRowCount,
};
static_assert(kFirstRowY + kRowPitch * kRowCount <= calibration::kPanelLastRow + 1,
              "the panel's rows run past the screen's last usable row");

/// Rows that never change, and the refusal screen's fixed words. Checked at compile time against
/// the SMALL / MEDIUM budgets, ASCII-only and '%'-free (calibration::screenText), the way
/// calibrate_program.cpp checks its own.
namespace text {
inline constexpr std::string_view kHeaderWord = "shulib TELEOP";
inline constexpr std::string_view kRefusedWord = "TELEOP REFUSED AT BOOT";
inline constexpr std::string_view kScreenRunStatus = "THIS VERSION has not run on hardware (G1 2b)";
inline constexpr std::string_view kNothingPowered = "Library graph NOT built. Nothing is powered.";
inline constexpr std::string_view kCalibrateFix =
    "Run shulib Calibrate (slot 4): the robot measures itself and saves the record Teleop boots "
    "from.";
inline constexpr std::string_view kAdapterHint = "Adapter refused? Bench Tests (slot 3) diagnoses.";
inline constexpr std::string_view kMoreOnSerial = "(the full text is on the serial log)";
inline constexpr std::string_view kTypedSource =
    "TYPED TABLE -- unverified since the rebuild; run shulib Calibrate";
inline constexpr std::string_view kMeasuredOk =
    "MEASURED record from the SD card, verified against the ports";

struct Checked {
    std::string_view text;
    std::size_t cols;
};
inline constexpr Checked kAll[] = {
    {kHeaderWord, calibration::kHeaderCols},   {kRefusedWord, calibration::kHeaderCols},
    {kScreenRunStatus, kSmallCols},            {kNothingPowered, kSmallCols},
    {kAdapterHint, kSmallCols},                {kMoreOnSerial, kSmallCols},
    {kBuildTimeText, kSmallCols},
    {std::string_view{kTeleopAutonomousReason}, kSmallCols},
};
constexpr bool allFit() {
    for (const Checked& c : kAll) {
        if (!calibration::screenText(c.text, c.cols)) {
            return false;
        }
    }
    return true;
}
static_assert(allFit(), "teleop_program: a screen literal does not fit its budget, or is not "
                        "ASCII, or contains a percent sign");
// The wrapped paragraphs must fit their rows WHOLE (wrapWords never splits a word and says when
// text did not fit), and their words must be screen-safe.
static_assert(calibration::wrapWords<kSmallCols, 2>(kCalibrateFix).complete
                  && calibration::screenText(kCalibrateFix, 2 * kSmallCols),
              "the Calibrate hint must fit two SMALL rows");
static_assert(calibration::wrapWords<kSmallCols, 2>(kTypedSource).complete
                  && calibration::wrapWords<kSmallCols, 2>(kMeasuredOk).complete,
              "a source row must fit its two SMALL rows");
// The header word must end before the build time, and the build time before the right edge, at
// the widths the budgets imply (472 px over the budget) — the same estimate Calibrate checks.
static_assert(kHeaderWordX + static_cast<int>(calibration::kHeaderCols)
                          * (472 / static_cast<int>(kMediumCols))
                  < kBuildX,
              "the header word could reach the build time");
static_assert(kBuildX + static_cast<int>(kBuildTimeText.size())
                            * (472 / static_cast<int>(kSmallCols))
                  <= 472,
              "the build time could run off the right edge");
}  // namespace text

// ═══ THE SOURCE ROWS: two SMALL rows, and a marker when the text is longer ═══════════════
/// The two source rows for `text`. When it fits two rows, they are its wrap. When it does not —
/// every DEGRADED verdict: its reason always ends "NEW motors not in the calibration: <list>" —
/// the rows end at the LAST WHOLE SENTENCE that fits with " ..." after it, so the rows never stop
/// mid-thought on a stray word and always say that more exists (the whole reason is the serial
/// log's `port check:` line). With no sentence end that fits, row 2 ends at the last whole word
/// that leaves room for " ...". constexpr, so the robot-two case below is checked at compile time.
struct SourceRows {
    char row[2][kSmallCols + 1]{};
};

constexpr std::string_view kMoreMark = " ...";

constexpr SourceRows sourceRows(std::string_view text) {
    SourceRows out{};
    auto copyInto = [](char* dst, std::string_view s, std::size_t cap) {
        std::size_t n = 0;
        for (; n < s.size() && n + 1 < cap; ++n) {
            dst[n] = s[n];
        }
        dst[n] = '\0';
    };
    const auto whole = calibration::wrapWords<kSmallCols, 2>(text);
    if (whole.complete) {
        for (std::size_t k = 0; k < whole.count; ++k) {
            copyInto(out.row[k], whole.line[k], sizeof out.row[k]);
        }
        return out;
    }
    // The last sentence end (". ") whose text, plus the marker, still fits the two rows.
    for (std::size_t end = text.size(); end-- > 1;) {
        if (text[end] != ' ' || text[end - 1] != '.') {
            continue;
        }
        char cand[2 * kSmallCols + 8]{};
        const std::size_t keep = end;  // up to and including the full stop
        if (keep + kMoreMark.size() >= sizeof cand) {
            continue;
        }
        for (std::size_t k = 0; k < keep; ++k) {
            cand[k] = text[k];
        }
        for (std::size_t k = 0; k < kMoreMark.size(); ++k) {
            cand[keep + k] = kMoreMark[k];
        }
        const auto cut =
            calibration::wrapWords<kSmallCols, 2>(std::string_view{cand, keep + kMoreMark.size()});
        if (cut.complete) {
            for (std::size_t k = 0; k < cut.count; ++k) {
                copyInto(out.row[k], cut.line[k], sizeof out.row[k]);
            }
            return out;
        }
    }
    // No sentence end fits: row 2 keeps the whole words that leave room for the marker.
    copyInto(out.row[0], whole.line[0], sizeof out.row[0]);
    const auto tail = calibration::wrapWords<kSmallCols - kMoreMark.size(), 1>(whole.line[1]);
    char second[kSmallCols + 1]{};
    std::size_t n = 0;
    for (; tail.count > 0 && tail.line[0][n] != '\0'; ++n) {
        second[n] = tail.line[0][n];
    }
    for (std::size_t k = 0; k < kMoreMark.size(); ++k) {
        second[n + k] = kMoreMark[k];
    }
    copyInto(out.row[1], std::string_view{second, n + kMoreMark.size()}, sizeof out.row[1]);
    return out;
}

// The case the review found, as the robot would print it: port 13 dead on the 2026-09-25 record,
// with motor 14 NEW (boot_check.hpp's DEGRADED sentence). Row 2 used to end on "... 1). NEW".
constexpr SourceRows kRobotTwoDegradedRows = sourceRows(
    "MEASURED record | DEGRADED: dead 13 -- driving without them (left dead 0, right dead 1). NEW "
    "motors not in the calibration: 14");
static_assert(std::string_view{kRobotTwoDegradedRows.row[0]}
                      == "MEASURED record | DEGRADED: dead 13 -- driving"
                  && std::string_view{kRobotTwoDegradedRows.row[1]}
                         == "without them (left dead 0, right dead 1). ...",
              "the DEGRADED source rows must end at a whole sentence and say more exists");
// Text that fits is its plain wrap, with no marker.
constexpr SourceRows kMeasuredOkRows = sourceRows(text::kMeasuredOk);
static_assert(std::string_view{kMeasuredOkRows.row[0]}
                      == "MEASURED record from the SD card, verified against"
                  && std::string_view{kMeasuredOkRows.row[1]} == "the ports",
              "a source row that fits must be its plain wrap");
// No sentence end at all: whole words, then the marker, within the budget.
constexpr SourceRows kNoSentenceRows = sourceRows(
    "aaaa bbbb cccc dddd eeee ffff gggg hhhh iiii jjjj kkkk llll mmmm nnnn oooo pppp qqqq rrrr "
    "ssss tttt uuuu");
static_assert(std::string_view{kNoSentenceRows.row[0]}
                      == "aaaa bbbb cccc dddd eeee ffff gggg hhhh iiii jjjj"
                  && std::string_view{kNoSentenceRows.row[1]}
                         == "kkkk llll mmmm nnnn oooo pppp qqqq rrrr ssss ...",
              "with no sentence end, row 2 keeps the whole words that leave room for the marker");

/// The header bar: the program's name at the left, the build time at the right.
void paintHeaderBar(std::uint32_t bar, std::string_view word) {
    pros::c::screen_set_eraser(kColBg);
    pros::c::screen_erase();
    pros::c::screen_set_pen(bar);
    pros::c::screen_fill_rect(0, 0, 479, kBarBottom);
    printText(TextSize::Medium, kHeaderWordX, kHeaderWordY, kColText, bar, word.data());
    printText(TextSize::Small, kBuildX, kBuildY, kColText, bar, kBuildTimeText.data());
}

constexpr std::int16_t rowY(int row) {
    return static_cast<std::int16_t>(kFirstRowY + kRowPitch * row);
}

// ═══ PROGRAM STATE (header: one function-local static of empty optionals) ═════════════════
struct TeleopState {
    BootOutcome boot{};
    // The provider BEFORE the robot: the robot borrows the provider's devices, so it must be
    // destroyed first (reverse declaration order), and the provider must outlive it.
    std::optional<hal::pros::ProsDeviceProvider> devices{};
    std::optional<robot::Robot> robot{};
    // The program's OWN controller and controller screen: the provider supplies neither.
    std::optional<hal::pros::ProsController> master{};
    std::optional<hal::pros::ProsLineDisplay> lcd{};
    // One driver, one shaper, held across opcontrol() calls so field-centric's zero and latch
    // survive a disable.
    teleop::DriveShaper shaper{teleopProfile()};
    // The claws' bindings (T3 Part 2): bound ONCE, inside teleopInitialize()'s try, from the table
    // over the robot above; held here — never on runTeleop()'s stack — because the loop holds them
    // by address and PROS deletes the opcontrol task on a field disable.
    TeleopClaws claws{};
    char portMap[diag::kMaxPortMapBytes]{};
    char refusalHeadline[64]{};
    char refusalWhy[192]{};
};

TeleopState& state() {
    static TeleopState s;
    return s;
}

// ═══ THE REFUSAL SCREEN ═════════════════════════════════════════════════════════════════
/// A full-screen refusal, every string through printText(): the headline wrapped over two MEDIUM
/// rows, the reason over four SMALL rows (and a pointer to the serial log when it is longer), what
/// is powered (nothing), and where to go next. Also the serial line. Nothing is powered: no graph
/// was built, and every adapter the provider may have made was released.
void paintRefusal(const char* headline, const char* why) {
    paintHeaderBar(kColBad, text::kRefusedWord);
    const auto h = calibration::wrapWords<kMediumCols, 2>(headline != nullptr ? headline : "");
    for (std::size_t k = 0; k < h.count; ++k) {
        printText(TextSize::Medium, kRowX, static_cast<std::int16_t>(34 + 22 * static_cast<int>(k)),
                  kColBad, kColBg, h.line[k]);
    }
    const auto w = calibration::wrapWords<kSmallCols, 4>(why != nullptr ? why : "");
    for (std::size_t k = 0; k < w.count; ++k) {
        printText(TextSize::Small, kRowX, static_cast<std::int16_t>(82 + 16 * static_cast<int>(k)),
                  kColText, kColBg, w.line[k]);
    }
    if (!w.complete || !h.complete) {
        printText(TextSize::Small, kRowX, 146, kColDim, kColBg, text::kMoreOnSerial.data());
    }
    printText(TextSize::Small, kRowX, 166, kColDim, kColBg, text::kNothingPowered.data());
    constexpr auto fix = calibration::wrapWords<kSmallCols, 2>(text::kCalibrateFix);
    for (std::size_t k = 0; k < fix.count; ++k) {
        printText(TextSize::Small, kRowX, static_cast<std::int16_t>(182 + 16 * static_cast<int>(k)),
                  kColDim, kColBg, fix.line[k]);
    }
    printText(TextSize::Small, kRowX, 214, kColDim, kColBg, text::kAdapterHint.data());
    std::printf("[%s] REFUSED AT BOOT: %s -- %s\n", kTag, headline != nullptr ? headline : "",
                why != nullptr ? why : "");
    std::fflush(stdout);
}

void refuse(const char* headline, const char* why) {
    TeleopState& st = state();
    std::snprintf(st.refusalHeadline, sizeof st.refusalHeadline, "%s", headline);
    std::snprintf(st.refusalWhy, sizeof st.refusalWhy, "%s", why);
    paintRefusal(st.refusalHeadline, st.refusalWhy);
}

// ═══ THE HEADING THE FEEL LAYER TRUSTS (driver_control.hpp's contract) ════════════════════
/// The fused heading, trusted only when the estimate is live (past Uninitialized — motion.hpp's
/// wait-for-live contract), the IMU itself reports ready, and no IMU_LOST fault has been raised
/// this run. Degraded has four causes and says nothing specific about the heading, so it is not
/// the test. Read before this tick's drive(), so it is the previous tick's estimate (10 ms old).
teleop::HeadingSample headingSample(const robot::Robot& r) {
    const bool trusted =
        r.localizer().qualityClass() != localization::Localizer::Quality::Uninitialized
        && r.context().imu().isReady() && r.faults().raiseCount(diag::FaultCode::ImuLost) == 0;
    return teleop::HeadingSample{.radians = r.localizer().pose().heading().radians(),
                                 .trusted = trusted};
}

const char* qualityName(localization::Localizer::Quality q) {
    using Q = localization::Localizer::Quality;
    switch (q) {
        case Q::Uninitialized: return "UNINIT";
        case Q::DeadReckon: return "DEADRECKON";
        case Q::Corrected: return "CORRECTED";
        case Q::Degraded: return "DEGRADED";
    }
    return "?";
}

const char* modeWord(teleop::DriveMode m) {
    switch (m) {
        case teleop::DriveMode::ArcadeBody: return "ARCADE BODY";
        case teleop::DriveMode::ArcadeField: return "ARCADE FIELD";
        case teleop::DriveMode::TankSticks: return "TANK STICKS";
    }
    return "?";
}

/// Field-centric's state in the words the panel prints (teleop/ has no word function).
const char* fieldCentricWord(const teleop::FieldCentricState& s) {
    switch (s.reason) {
        case teleop::FieldCentricReason::Active: return "ON";
        case teleop::FieldCentricReason::HeadingNotTrusted: return "FELL BACK";
        case teleop::FieldCentricReason::LatchedUntilRezero: return "LATCHED";
        case teleop::FieldCentricReason::NotSelected: return "OFF";
    }
    return "?";
}

// ═══ THE PANEL + THE CONTROLLER SCREEN ═══════════════════════════════════════════════════
/// The brain panel and the controller's three lines, called once per tick by runDriverControl.
///
/// SCREEN RULES (R3d Part 2F's): the rows are recomputed every kPanelEveryTicks (200 ms — live
/// numbers at most 5 Hz) and a row is REPAINTED only when its text or colour changed, padded with
/// spaces over the old text (VEXos draws text pen-on-eraser, so no erase is needed). Every
/// screen_print_at leaks its buffer in PROS 4.2.2, and the old panel printed ten rows every 100 ms;
/// the count is reported when the loop exits. Rows that never change (the budgets and gains, the
/// feel note, the stall note, the port map) are serial-only now: the boot lines carry them.
///
/// LCD (3 x 19, HA-57): `LIB 12V Bxx.xV` / `hdg +ddd.d deg` / the last fault, `no controller 0V`,
/// `DEGRADED see brain`, `CLAW ABSENT` (T3 Part 2: a declared claw was not built) or `ok` — in that
/// order of urgency, written only when a line changed.
class TeleopPanel {
public:
    TeleopPanel(const robot::Robot& r, hal::ILineDisplay& lcd, const BootOutcome& boot,
                const TeleopClaws& claws)
        : r_{r}, lcd_{lcd}, boot_{boot}, claws_{claws} {
        const auto groups = r_.chassis().deps().motorGroups;
        SHULIB_PRECONDITION(groups.size() == 2 && groups[0] != nullptr && groups[1] != nullptr,
                            "TeleopPanel: a tank robot has exactly two motor groups");
        left_ = groups[0];
        right_ = groups[1];
    }

    /// Paint everything: the header, the static rows, and mark every live row for repainting.
    void paintAll() {
        paintHeaderBar(kColBar, text::kHeaderWord);
        for (Row& row : rows_) {
            row = Row{};
        }
        paintRow(kRowStatus, kColWarn, text::kScreenRunStatus.data());
        // WHERE THE NUMBERS CAME FROM, and what is dead: a typed table on a rebuilt robot is a
        // belief, not a measurement, and the driver must be able to see which one is driving them.
        char source[400];
        std::uint32_t colour = kColGood;
        if (boot_.source == calibration::BootSource::UseTable) {
            std::snprintf(source, sizeof source, "%s", text::kTypedSource.data());
            colour = kColWarn;
        } else if (boot_.verdict.decision == calibration::BootDecision::Degraded) {
            std::snprintf(source, sizeof source, "MEASURED record | %s", boot_.verdict.reason);
            colour = kColWarn;
        } else {
            std::snprintf(source, sizeof source, "%s", text::kMeasuredOk.data());
        }
        // Two rows, ending at a whole sentence with " ..." when the reason is longer
        // (sourceRows(): a DEGRADED reason always is; the serial `port check:` line has it all).
        const SourceRows rows = sourceRows(source);
        paintRow(kRowSource1, colour, rows.row[0]);
        paintRow(kRowSource2, colour, rows.row[1]);
        tick_ = 0;
    }

    void operator()(const math::ChassisSpeeds& command, bool connected,
                    const teleop::DriveShaperState& shaper) {
        ++tick_;
        if (tick_ % kPanelEveryTicks == 0) {
            refreshRows(command, connected, shaper);
        }
        if (tick_ % kLcdEveryTicks == 0) {
            refreshLcd(connected);
        }
    }

private:
    struct Row {
        char text[kSmallCols + 1]{};
        std::uint32_t colour = 0;
        bool painted = false;
    };

    /// Repaint `row` only when its text or colour changed, covering the old text with spaces.
    void paintRow(int row, std::uint32_t colour, const char* want) {
        Row& have = rows_[row];
        char next[kSmallCols + 1];
        std::snprintf(next, sizeof next, "%.50s", want);
        if (have.painted && have.colour == colour && std::strcmp(have.text, next) == 0) {
            return;
        }
        printText(TextSize::Small, kRowX, rowY(row), colour, kColBg, next,
                  have.painted ? std::strlen(have.text) : 0);
        std::memcpy(have.text, next, sizeof have.text);
        have.colour = colour;
        have.painted = true;
    }

    void refreshRows(const math::ChassisSpeeds& command, bool connected,
                     const teleop::DriveShaperState& shaper) {
        char line[64];
        std::snprintf(line, sizeof line, "cmd vx %+6.1f in/s  w %+5.2f rad/s%s",
                      command.vx().value(), command.omega().value(),
                      connected ? "" : "  NO CONTROLLER");
        paintRow(kRowCmd, connected ? kColText : kColWarn, line);

        const math::Pose2d pose = r_.localizer().pose();
        const localization::Localizer::Quality q = r_.localizer().qualityClass();
        std::snprintf(line, sizeof line, "est x %+7.1f  y %+7.1f  hdg %+6.1f %s", pose.x().value(),
                      pose.y().value(), pose.heading().radians() * 180.0 / math::Angle::kPi,
                      qualityName(q));
        paintRow(kRowEst, q == localization::Localizer::Quality::Degraded ? kColWarn : kColText,
                 line);

        const bool fcTrouble = shaper.fieldCentric.fellBack;
        std::snprintf(line, sizeof line, "driver %s  field-centric %s",
                      modeWord(state().shaper.profile().mode),
                      fieldCentricWord(shaper.fieldCentric));
        paintRow(kRowDriver, fcTrouble ? kColWarn : kColText, line);

        const hal::MotorGroup* groups[2] = {left_, right_};
        const char* names[2] = {"L", "R"};
        for (int s = 0; s < 2; ++s) {
            const hal::MotorGroup& g = *groups[s];
            std::snprintf(line, sizeof line, "%s %+5.1f V  %u/%u present  %d disagree", names[s],
                          g.commandedVoltage().value(), static_cast<unsigned>(g.presentCount()),
                          static_cast<unsigned>(g.memberCount()), g.disagreeingMembers());
            const bool trouble = g.disagreeingMembers() > 0;
            const bool partial = g.presentCount() < g.memberCount();
            paintRow(s == 0 ? kRowLeft : kRowRight,
                     trouble ? kColBad : (partial ? kColWarn : kColGood), line);
        }

        const localization::DriveEncoderOdometry* odom = r_.driveOdometry();
        const double battery = r_.context().battery().voltage().value();
        if (odom != nullptr) {
            std::snprintf(line, sizeof line, "hdg xcheck %+7.4f rad/tick  batt %5.2f V",
                          odom->lastHeadingDisagreement(), battery);
            paintRow(kRowXcheck, kColText, line);
            std::snprintf(line, sizeof line, "L/R travel %+6.2f %+6.2f in (last tick)",
                          odom->lastSideTravel().left.value(),
                          odom->lastSideTravel().right.value());
            paintRow(kRowTravel, kColDim, line);
        } else {
            std::snprintf(line, sizeof line, "batt %5.2f V", battery);
            paintRow(kRowXcheck, kColText, line);
            paintRow(kRowTravel, kColDim, "");
        }

        // ONE fault row since T3 Part 2 (ruling R3): src/programs/teleop_claws.hpp's faultRowText().
        const diag::FaultLatch& f = r_.faults();
        const ClawRow faults = faultRowText(f.hasFault() ? f.faultCount() : 0, f.firstFault(),
                                            f.lastFault());
        paintRow(kRowFault, f.hasFault() ? kColBad : kColGood, faults.text);

        // The claw row (T3 Part 2): amber while a claw is ABSENT or a claim was released at this
        // entry (FREED) — each is something the driver must know before trusting the buttons.
        const ClawRow claws = clawRowText(kMechanisms, r_, claws_);
        paintRow(kRowClaws,
                 anyClawAbsent(r_) || claws_.anyReleasedLeftoverClaim() ? kColWarn : kColText,
                 claws.text);
    }

    void refreshLcd(bool connected) {
        char want[3][24];
        std::snprintf(want[0], sizeof want[0], "LIB %2.0fV B%4.1fV",
                      hal::kMaxMotorVoltage.value(), r_.context().battery().voltage().value());
        std::snprintf(want[1], sizeof want[1], "hdg %+6.1f deg",
                      r_.localizer().pose().heading().radians() * 180.0 / math::Angle::kPi);
        std::snprintf(want[2], sizeof want[2], "%.19s",
                      r_.faults().hasFault() ? diag::faultCodeName(r_.faults().lastFault())
                      : !connected           ? "no controller 0V"
                      : boot_.verdict.decision == calibration::BootDecision::Degraded
                          ? "DEGRADED see brain"
                      : anyClawAbsent(r_) ? kLcdClawAbsent.data()
                                          : "ok");
        for (int row = 0; row < 3; ++row) {
            if (std::strcmp(want[row], lcdRow_[row]) != 0) {
                lcd_.setLine(row, want[row]);
                std::memcpy(lcdRow_[row], want[row], sizeof lcdRow_[row]);
            }
        }
    }

    const robot::Robot& r_;
    hal::ILineDisplay& lcd_;
    const BootOutcome& boot_;
    const TeleopClaws& claws_;
    const hal::MotorGroup* left_ = nullptr;
    const hal::MotorGroup* right_ = nullptr;
    int tick_ = 0;
    Row rows_[kRowCount]{};
    char lcdRow_[3][24] = {"", "", ""};
};

/// The boot lines after the session header — every line the previous program said that is still
/// true, in its order, each within TermSink's 200 bytes (the brief's A10; the diff is in the log).
void logBootLines(robot::Robot& r, hal::ITelemetrySink& log, const BootOutcome& boot) {
    char line[240];
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
    log.log(boot.source == calibration::BootSource::UseRecord ? hal::LogLevel::Info
                                                              : hal::LogLevel::Warn,
            kTag, line);
    {
        char check[340];
        std::snprintf(check, sizeof check, "port check: %s", boot.verdict.reason);
        logWrapped(log,
                   boot.verdict.decision == calibration::BootDecision::Ok ? hal::LogLevel::Info
                                                                          : hal::LogLevel::Warn,
                   check);
    }
    {
        char status[480];  // kRunStatus is 422 bytes since T3 Part 2 (the claws' sentence)
        std::snprintf(status, sizeof status, "HARDWARE-UNVALIDATED: %s", kRunStatus);
        logWrapped(log, hal::LogLevel::Warn, status);
    }
    log.log(hal::LogLevel::Warn, kTag, kProvisionalLine);
    log.log(hal::LogLevel::Info, kTag, kAbsentDevicesLine);
    // The §2 ruling, said ONCE at boot (odo_stall_check.hpp carries the reasoning) — in pieces,
    // because the library's note is longer than TermSink's 200 bytes.
    logWrapped(log, hal::LogLevel::Warn, motion::kNoIndependentStallSourceNote);
    const std::size_t deadPorts = boot.built.absent().size();
    if (deadPorts > 0) {
        std::snprintf(line, sizeof line,
                      "DEGRADED: %u recorded drive port(s) are EMPTY and were NOT constructed; "
                      "their group slots are holes and every read is over the PRESENT members. "
                      "%.80s",
                      static_cast<unsigned>(deadPorts), boot.verdict.reason);
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
    std::snprintf(line, sizeof line, "%s %s%s%s", kDriverLine.data(), buttonWord(kRezeroButton),
                  kDriverLineFieldCentric.data(),
                  kFieldCentric ? kSelectableOn.data() : kSelectableOff.data());
    log.log(hal::LogLevel::Info, kTag, line);
    // THE CLAWS (T3 Part 2): one line per declared mechanism, BUILT or ABSENT with the provider's
    // own sentence (src/programs/teleop_claws.hpp words them; Station J.3.1 / J.9.1 quote them). An ABSENT
    // one is a warning: its buttons do nothing, and the drive runs anyway.
    for (std::size_t i = 0; i < motorEntryCount(kMechanisms); ++i) {
        logWrapped(log, r.builtMechanismAt(i) != nullptr ? hal::LogLevel::Info : hal::LogLevel::Warn,
                   motorClawBootLine(kMechanisms, r, i).text);
    }
    for (std::size_t j = 0; j < pneumaticEntryCount(kMechanisms); ++j) {
        logWrapped(log,
                   r.builtMechanismAt(mechanismConfigIndexOfPneumatic(kMechanisms, j)) != nullptr
                       ? hal::LogLevel::Info
                       : hal::LogLevel::Warn,
                   pneumaticClawBootLine(kMechanisms, r, j).text);
    }
}

}  // namespace

void teleopInitialize(const char* variantBeacon, const char* competitionStatusBits) {
    TeleopState& st = state();
    std::printf("\n[%s] booted: shulib TELEOP build [%s] -- THE LIBRARY drives robot two through\n"
                "[%s] robot::Robot (G1 Part 2b): two hal::MotorGroups, drive-encoder odometry and\n"
                "[%s] Chassis::drive(), built from the CALIBRATION RECORD on the SD card when\n"
                "[%s] there is one, else from src/robots/chassis_table.hpp. The invented X-drive graph\n"
                "[%s] is NOT constructed.\n"
                "[%s] %s\n"
                "[%s] BUILD STAMP %s\n",
                kTag, variantBeacon, kTag, kTag, kTag, kTag, kTag, kRunStatus, kTag, kBuildStamp);
    // The banner's claw sentence (T3 Part 2; Station J.3.2): generated from the table, so it
    // cannot drift from the buttons the loop is given.
    std::printf("[%s] %s\n", kTag, clawBannerLine(kMechanisms, kRezeroButton).text);
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

    // ── the graph, at its boundary: validate() AND the construction, once, into the optionals.
    //    The options are the boot check's absent ports and nothing else, through `built` —
    //    boot_source.hpp's one door, the one test 6 goes through — so a port it found dead is
    //    never requested and is named once.
    try {
        st.devices.emplace();
        const robot::BuildVerdict verdict =
            robot::validate(st.boot.built.config, *st.devices, st.boot.built.buildOptions());
        if (!verdict.ok) {
            st.devices.reset();
            refuse("the library graph REFUSED to build", verdict.why);
            return;
        }
        st.robot.emplace(st.boot.built.config, *st.devices, st.boot.built.buildOptions());
        // The claws' bindings (T3 Part 2), built and VALIDATED here, inside the boundary's try —
        // a binding's constructor may throw, and nothing that can throw runs after driver control
        // starts. An ABSENT claw gets no binding; the set is checked against kRezeroButton.
        char why[192] = "";
        if (!st.claws.bind(kMechanisms, *st.robot, kRezeroButton, why, sizeof why)) {
            st.robot.reset();
            st.devices.reset();
            refuse("the claw buttons REFUSED", why);
            return;
        }
    } catch (const PreconditionError& e) {
        st.claws.unbind();  // no binding may outlive the mechanisms it points at
        st.robot.reset();
        st.devices.reset();
        refuse("the library graph REFUSED to build", e.what());
        return;
    }
    robot::Robot& r = *st.robot;
    hal::ITelemetrySink& log = st.devices->telemetry();
    attachFaultLatch(&r.faults());  // a precondition after boot latches FaultCode::Precondition
    st.master.emplace(hal::pros::ControllerId::Master);
    st.lcd.emplace();
    log.log(hal::LogLevel::Info, kTag, variantBeacon);

    // ── the §18.5 session header: provenance FIRST, a missing hash LOUD; the port map GENERATED
    //    from the config by robot::Robot::portMap() — never retyped, never describe() (longer than
    //    the 96-byte field).
    r.portMap(st.portMap, sizeof st.portMap);
    diag::SessionInfo session{};
    session.buildHash = diag::compiledBuildHash();
    session.routineId = "(none: teleop only)";
    session.alliance = "";
    session.side = "";
    session.portMap = st.portMap;
    diag::emitSessionHeader(log, session, r.context().battery().voltage());

    logBootLines(r, log, st.boot);
}

void teleopAutonomous() {
    TeleopState& st = state();
    if (!st.robot.has_value()) {
        return;  // refused at boot: nothing was built, and the state screen already says why
    }
    char line[96];
    std::snprintf(line, sizeof line, "autonomous(): %s", kTeleopAutonomousReason);
    st.devices->telemetry().log(hal::LogLevel::Warn, kTag, line);
}

void runTeleop() {
    TeleopState& st = state();
    if (!st.robot.has_value()) {
        // Refused at boot: no graph exists, nothing is powered. Keep the reason on screen (a
        // state screen may have replaced it) and idle.
        paintRefusal(st.refusalHeadline, st.refusalWhy);
        while (true) {
            pros::delay(250);
        }
    }
    robot::Robot& r = *st.robot;
    hal::ITelemetrySink& log = st.devices->telemetry();
    log.log(hal::LogLevel::Info, kTag, kOpcontrolLive);
    TeleopPanel panel{r, *st.lcd, st.boot, st.claws};
    panel.paintAll();
    const teleop::DriverLoopConfig loopCfg{
        .tickPeriod = units::Time{static_cast<double>(hal::pros::ProsTickPacer::kTickMs) / 1000.0},
        .rezeroButton = kRezeroButton};
    // The bindings overload (T3 Part 2): the pivots hold-to-run, the claw's toggle. The set was
    // validated in teleopInitialize() against the same kRezeroButton, so the loop's own check of it
    // before the first tick cannot refuse. A claim an operation left on a claw is RELEASED at entry
    // and logged by the loop itself ([WARN][TEL]; the team lead's ruling, 2026-09-26).
    teleop::runDriverControl(
        r.chassis(), *st.master, st.devices->pacer(), st.shaper, loopCfg, st.claws.holds(),
        st.claws.toggles(), [] { return pros::c::competition_is_disabled() != 0; },
        [&r] { return headingSample(r); }, panel);
    char line[200];
    std::snprintf(line, sizeof line,
                  "opcontrol(): field DISABLED -- loop exited cleanly after cancel() (0 V + Brake); "
                  "each claw pivot made safe, the pneumatic claw left as it was; %lu screen prints "
                  "so far",
                  static_cast<unsigned long>(g_screenPrints));
    log.log(hal::LogLevel::Info, kTag, line);
}

}  // namespace shulib::bench
