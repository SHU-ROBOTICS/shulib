#pragma once
//
// drive_panel.hpp — "shulib Drive"'s SCREEN as a pure model: what its MAIN, DETAILS and MORE pages
// say, where each button is, and which touch flips a page (chunk T4 Part 3, 2026-10-01; the team
// lead's rulings R5 and R7, and the T4 brief's D4).
//
// ═══ WHAT IS ON WHICH PAGE (R5, D4) ════════════════════════════════════════════════════
//   MAIN, the driver's page:   the header bar (`shulib DRIVE`, `BUILD hh:mm:ss`), the PROTECTION
//                              row, the controller row, and a DETAILS button. Nothing else: on the
//                              field the driver reads the controller, not the brain, so the brain's
//                              MAIN page carries what a cut or a dead port would explain, and one
//                              touch away is everything else.
//   DETAILS (page 1):          R5's four, verbatim from the ruling of 2026-09-27 — "the fault row,
//                              the claw row, where the drivetrain numbers came from, the run
//                              status" — then the command, the estimate, the driver mode and the
//                              battery with the heading cross-check. MORE and BACK buttons.
//   MORE (page 2):             the LEFT and RIGHT groups (present/members, disagreeing, the ports
//                              dead), the last tick's travel, and the protections' counters (the
//                              mode with its cuts and dead list, the last cut's words, each side's
//                              peak current this entry). A BACK button.
// So nothing "shulib Teleop" showed is lost (its twelve rows moved here, split), and the two pages
// hold what one could not (the brief: "If that does not fit one page, DETAILS gets a second page
// (MORE), with R5's four on its first").
//
// ═══ A ROBOT CONSOLE LATER (R7) ═══════════════════════════════════════════════════════
// Every row is made by a PURE function from plain values (or a library object the program hands
// in), and every page is a DriveView of competition::MenuLine rows and DriveButtons — data, so a
// later robot console can stream the same pages over the wire (H1's) and a remote touch is one
// more source of ui::TouchSample. The program only paints what driveView() returns.
//
// ═══ THE PAGE FLIP (DrivePager) ═════════════════════════════════════════════════════════
// A touch flips a page on the RELEASE edge of a press that LANDED after the page appeared, and only
// once the page has been up ui::kArmingDelaySeconds (the library's touch_panel rules: a tap made
// before the program existed is never replayed, an error read never moves the count, and a finger
// that was already down when the page changed — a press held across a flip — does nothing).
//
// ═══ WHAT IS NOT HERE, AND WHERE IT COMES FROM ════════════════════════════════════════
// The PROTECTION row's words, the controller row, the claw row, the fault row and the controller's
// three lines (with their ONE chooser and the writer that sends them) are the shared driver-panel
// header's (src/common/driver_panel.hpp — chunk COMP-b, T4 Part 2b's words, and the claw words
// moved there from src/programs/teleop_claws.hpp when T4 Part 3 retired "shulib Teleop"): the
// competition program shows the same words, so they live once. The program reads the robot, hands
// those rows in, and paints. The painting itself — the colours, the bar, a line, a button, the
// refusal screen — is the shared painters' (src/common/screen_paint.hpp); what to repaint between
// two views of a page is driveRepaint() below, pure, so "a row is repainted only when it changed"
// is a host test rather than a hope.
//
// ═══ THE PROGRAM'S GLUE, HERE TOO (the T4-3 review, 2026-10-08) ═══════════════════════════
// drive_program.cpp is compiled by no host test (it includes PROS), so whatever it decides is
// guarded only by source pins — and ten edits to its row building and boot lines (a CUT painted
// white, the claw row's amber dropped, MORE's L row reading the RIGHT group, a protections line
// saying the wrong numbers, a DEGRADED boot line never said) left the whole suite green. So the
// decisions moved here, pure, from plain facts: driveSourceRows() (the source rows and their
// tone), driveLiveRows() (every live row from DriveLiveFacts, the protections' state and the two
// groups), driveLineToPaint() (what a partial repaint paints for a row, a row that went erased),
// driveProtectionsLine(), driveDegradedBootLine() and driveBootLevel() (the boot's generated words
// and levels). The program reads the robot into DriveLiveFacts and calls them; its calls are
// pinned (test/drive_program_source_test.cpp), the decisions host-tested.
//
// PROS-free, header-only; host-tested in test/drive_panel_test.cpp. EVERY LAYOUT NUMBER IS AN
// ESTIMATE (A4 register HA-136). NOTHING HERE HAS RUN ON A ROBOT.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string_view>

#include "common/driver_panel.hpp"
#include "shulib/calibration/calibrate_panel.hpp"
#include "shulib/competition/auton_menu.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/hal/motor_group.hpp"
#include "shulib/hal/telemetry_sink.hpp"
#include "shulib/localization/localizer.hpp"
#include "shulib/teleop/drive_modes.hpp"
#include "shulib/teleop/drive_protection.hpp"
#include "shulib/teleop/field_centric.hpp"
#include "shulib/ui/touch_panel.hpp"

namespace shulib::bench {

// ═══ THE PROGRAM'S OWN WORDS, checked against their budgets where they are written ═════════
/// The header bar's word on every page (the header budget).
inline constexpr std::string_view kDriveHeaderWord = "shulib DRIVE";
/// The DETAILS page's header word.
inline constexpr std::string_view kDriveDetailsWord = "shulib DRIVE  DETAILS";
/// The MORE page's header word.
inline constexpr std::string_view kDriveMoreWord = "shulib DRIVE  MORE";
/// The refusal screen's bar word: what a boot that refused says, in the red bar.
inline constexpr std::string_view kDriveRefusedWord = "DRIVE REFUSED AT BOOT";
/// DETAILS' first row: this version's run status, said on the screen (the boot says it in full).
inline constexpr std::string_view kDriveScreenRunStatus = "THIS VERSION has not run on hardware (T4 3)";
/// The refusal screen's footer: what is powered, and the fix (two SMALL rows, whole).
inline constexpr std::string_view kDriveRefusalFooter =
    "Nothing is powered. Run shulib Calibrate (slot 4) to save the record this program boots from.";
/// Where the drivetrain numbers came from: the typed table (a belief, not a measurement).
inline constexpr std::string_view kDriveTypedSource =
    "TYPED TABLE -- unverified since the rebuild; run shulib Calibrate";
/// ...or the calibration record, verified against the ports on this boot.
inline constexpr std::string_view kDriveMeasuredOk =
    "MEASURED record from the SD card, verified against the ports";
/// The buttons' labels (MEDIUM).
inline constexpr std::string_view kDriveDetailsLabel = "DETAILS";
inline constexpr std::string_view kDriveMoreLabel = "MORE";
inline constexpr std::string_view kDriveBackLabel = "BACK";

static_assert(calibration::screenText(kDriveHeaderWord, calibration::kHeaderCols)
                  && calibration::screenText(kDriveDetailsWord, calibration::kHeaderCols)
                  && calibration::screenText(kDriveMoreWord, calibration::kHeaderCols)
                  && calibration::screenText(kDriveRefusedWord, calibration::kHeaderCols),
              "drive_panel: a header word does not fit the header budget, or is not screen-safe");
static_assert(calibration::screenText(kDriveScreenRunStatus, calibration::kSmallCols)
                  && calibration::screenText(kDriveDetailsLabel, calibration::kMediumCols)
                  && calibration::screenText(kDriveMoreLabel, calibration::kMediumCols)
                  && calibration::screenText(kDriveBackLabel, calibration::kMediumCols),
              "drive_panel: a row or a label does not fit its budget, or is not screen-safe");
static_assert(calibration::wrapWords<calibration::kSmallCols, 2>(kDriveRefusalFooter).complete
                  && calibration::screenText(kDriveRefusalFooter, 2 * calibration::kSmallCols),
              "drive_panel: the refusal footer must fit two SMALL rows, whole");
static_assert(calibration::wrapWords<calibration::kSmallCols, 2>(kDriveTypedSource).complete
                  && calibration::wrapWords<calibration::kSmallCols, 2>(kDriveMeasuredOk).complete,
              "drive_panel: a source row must fit its two SMALL rows");

// ═══ THE SOURCE ROWS: two SMALL rows, and a marker when the text is longer ═══════════════
// (Moved from "shulib Teleop" at T4 Part 3, unchanged, with its three checks.) The two source rows
// for `text`. When it fits two rows, they are its wrap. When it does not — every DEGRADED verdict:
// its reason always ends "NEW motors not in the calibration: <list>" — the rows end at the LAST
// WHOLE SENTENCE that fits with " ..." after it, so the rows never stop mid-thought on a stray word
// and always say that more exists (the whole reason is the serial log's `port check:` line). With
// no sentence end that fits, row 2 ends at the last whole word that leaves room for " ...".
// constexpr, so the robot-two case below is checked at compile time.

/// The two SOURCE rows (DETAILS).
struct SourceRows {
    char row[2][calibration::kSmallCols + 1]{};  ///< each NUL-terminated
};

/// The marker a cut source text ends with.
inline constexpr std::string_view kMoreMark = " ...";

/// The two source rows for `text` (above).
constexpr SourceRows sourceRows(std::string_view text) {
    constexpr std::size_t kCols = calibration::kSmallCols;
    SourceRows out{};
    auto copyInto = [](char* dst, std::string_view s, std::size_t cap) {
        std::size_t n = 0;
        for (; n < s.size() && n + 1 < cap; ++n) {
            dst[n] = s[n];
        }
        dst[n] = '\0';
    };
    const auto whole = calibration::wrapWords<kCols, 2>(text);
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
        char cand[2 * kCols + 8]{};
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
            calibration::wrapWords<kCols, 2>(std::string_view{cand, keep + kMoreMark.size()});
        if (cut.complete) {
            for (std::size_t k = 0; k < cut.count; ++k) {
                copyInto(out.row[k], cut.line[k], sizeof out.row[k]);
            }
            return out;
        }
    }
    // No sentence end fits: row 2 keeps the whole words that leave room for the marker.
    copyInto(out.row[0], whole.line[0], sizeof out.row[0]);
    const auto tail = calibration::wrapWords<kCols - kMoreMark.size(), 1>(whole.line[1]);
    char second[kCols + 1]{};
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
inline constexpr SourceRows kRobotTwoDegradedRows = sourceRows(
    "MEASURED record | DEGRADED: dead 13 -- driving without them (left dead 0, right dead 1). NEW "
    "motors not in the calibration: 14");
static_assert(std::string_view{kRobotTwoDegradedRows.row[0]}
                      == "MEASURED record | DEGRADED: dead 13 -- driving"
                  && std::string_view{kRobotTwoDegradedRows.row[1]}
                         == "without them (left dead 0, right dead 1). ...",
              "the DEGRADED source rows must end at a whole sentence and say more exists");
// Text that fits is its plain wrap, with no marker.
inline constexpr SourceRows kMeasuredOkRows = sourceRows(kDriveMeasuredOk);
static_assert(std::string_view{kMeasuredOkRows.row[0]}
                      == "MEASURED record from the SD card, verified against"
                  && std::string_view{kMeasuredOkRows.row[1]} == "the ports",
              "a source row that fits must be its plain wrap");
// No sentence end at all: whole words, then the marker, within the budget.
inline constexpr SourceRows kNoSentenceRows = sourceRows(
    "aaaa bbbb cccc dddd eeee ffff gggg hhhh iiii jjjj kkkk llll mmmm nnnn oooo pppp qqqq rrrr "
    "ssss tttt uuuu");
static_assert(std::string_view{kNoSentenceRows.row[0]}
                      == "aaaa bbbb cccc dddd eeee ffff gggg hhhh iiii jjjj"
                  && std::string_view{kNoSentenceRows.row[1]}
                         == "kkkk llll mmmm nnnn oooo pppp qqqq rrrr ssss ...",
              "with no sentence end, row 2 keeps the whole words that leave room for the marker");

// ═══ THE WORDS FOR A STATE (moved from "shulib Teleop" at T4 Part 3) ═══════════════════════

/// The localizer's quality class as the estimate row prints it.
constexpr const char* qualityName(localization::Localizer::Quality q) noexcept {
    using Q = localization::Localizer::Quality;
    switch (q) {
        case Q::Uninitialized: return "UNINIT";
        case Q::DeadReckon: return "DEADRECKON";
        case Q::Corrected: return "CORRECTED";
        case Q::Degraded: return "DEGRADED";
    }
    return "?";
}

/// The driver mode as the mode row prints it.
constexpr const char* modeWord(teleop::DriveMode m) noexcept {
    switch (m) {
        case teleop::DriveMode::ArcadeBody: return "ARCADE BODY";
        case teleop::DriveMode::ArcadeField: return "ARCADE FIELD";
        case teleop::DriveMode::TankSticks: return "TANK STICKS";
    }
    return "?";
}

/// Field-centric's state in the words the mode row prints (teleop/ has no word function).
constexpr const char* fieldCentricWord(const teleop::FieldCentricState& s) noexcept {
    switch (s.reason) {
        case teleop::FieldCentricReason::Active: return "ON";
        case teleop::FieldCentricReason::HeadingNotTrusted: return "FELL BACK";
        case teleop::FieldCentricReason::LatchedUntilRezero: return "LATCHED";
        case teleop::FieldCentricReason::NotSelected: return "OFF";
    }
    return "?";
}

/// The protections' mode as the MORE page's counters row prints it.
constexpr const char* protectionModeWord(teleop::ProtectionMode m) noexcept {
    switch (m) {
        case teleop::ProtectionMode::Driving: return "DRIVING";
        case teleop::ProtectionMode::Degraded: return "DEGRADED";
        case teleop::ProtectionMode::Cut: return "CUT";
        case teleop::ProtectionMode::Refused: return "REFUSED";
    }
    return "?";
}

// ═══ THE ROWS — each a competition::MenuLine (text and tone; driveView() places it) ═══════════

/// One row: `text` cut to the SMALL budget through the screen-safe copy, in `tone`.
inline competition::MenuLine driveRow(std::string_view text, ui::Tone tone) noexcept {
    competition::MenuLine l{};
    l.size = ui::TextSize::Small;
    l.tone = tone;
    ui::copyScreenText(l.text, sizeof l.text, text, ui::kSmallCols);
    return l;
}

/// DETAILS: the command the loop was handed — `cmd vx +12.0 in/s  w +0.50 rad/s`, and
/// `  NO CONTROLLER` (amber) when the controller is gone (it reads 0 on every stick, HA-103).
inline competition::MenuLine driveCmdRow(double vxInchesPerSecond, double omegaRadPerSecond,
                                         bool connected) noexcept {
    char line[64];
    std::snprintf(line, sizeof line, "cmd vx %+6.1f in/s  w %+5.2f rad/s%s", vxInchesPerSecond,
                  omegaRadPerSecond, connected ? "" : "  NO CONTROLLER");
    return driveRow(line, connected ? ui::Tone::White : ui::Tone::Amber);
}

/// DETAILS: the fused estimate and its quality class (Stations F and G.6.3 read `est x` / `hdg`
/// here) — amber while DEGRADED.
inline competition::MenuLine driveEstRow(double xInches, double yInches, double headingDegrees,
                                         localization::Localizer::Quality q) noexcept {
    char line[64];
    std::snprintf(line, sizeof line, "est x %+7.1f  y %+7.1f  hdg %+6.1f %.10s", xInches, yInches,
                  headingDegrees, qualityName(q));
    return driveRow(line, q == localization::Localizer::Quality::Degraded ? ui::Tone::Amber
                                                                         : ui::Tone::White);
}

/// DETAILS: the driver mode and field-centric's state — amber while field-centric fell back.
inline competition::MenuLine driveModeRow(teleop::DriveMode mode,
                                          const teleop::FieldCentricState& fc) noexcept {
    char line[64];
    std::snprintf(line, sizeof line, "driver %.12s  field-centric %.9s", modeWord(mode),
                  fieldCentricWord(fc));
    return driveRow(line, fc.fellBack ? ui::Tone::Amber : ui::Tone::White);
}

/// DETAILS: the battery, with the encoder-vs-IMU heading cross-check when the robot has drive
/// encoders (`hasOdometry`).
inline competition::MenuLine driveBatteryRow(bool hasOdometry, double headingDisagreement,
                                             double batteryVolts) noexcept {
    char line[64];
    if (hasOdometry) {
        std::snprintf(line, sizeof line, "hdg xcheck %+7.4f rad/tick  batt %5.2f V",
                      headingDisagreement, batteryVolts);
    } else {
        std::snprintf(line, sizeof line, "batt %5.2f V", batteryVolts);
    }
    return driveRow(line, ui::Tone::White);
}

/// MORE: the last tick's per-side travel (grey: a diagnostic, not a state), or nothing without
/// drive encoders.
inline competition::MenuLine driveTravelRow(bool hasOdometry, double leftInches,
                                            double rightInches) noexcept {
    if (!hasOdometry) {
        return driveRow("", ui::Tone::Grey);
    }
    char line[64];
    std::snprintf(line, sizeof line, "L/R travel %+6.2f %+6.2f in (last tick)", leftInches,
                  rightInches);
    return driveRow(line, ui::Tone::Grey);
}

/// MORE: one side's group — `L +12.0 V  3/4 present  0 disagree  dead 18`: its commanded volts,
/// its present members of all, the members its fight monitor holds disagreeing, and its DEAD
/// ports (a boot hole or a runtime mark, by label). Red while one disagrees, amber while one is
/// missing, green otherwise.
inline competition::MenuLine driveGroupRow(const char* side, const hal::MotorGroup& g) {
    char line[96];
    int n = std::snprintf(line, sizeof line, "%.1s %+5.1f V  %u/%u present  %d disagree", side,
                          g.commandedVoltage().value(), static_cast<unsigned>(g.presentCount()),
                          static_cast<unsigned>(g.memberCount()), g.disagreeingMembers());
    const std::uint32_t dead = g.absentMask() | g.runtimeAbsentMask();
    bool first = true;
    for (std::size_t i = 0; i < g.memberCount() && i < 32 && n > 0
                            && static_cast<std::size_t>(n) < sizeof line;
         ++i) {
        if ((dead & (std::uint32_t{1} << i)) == 0) continue;
        n += std::snprintf(line + n, sizeof line - static_cast<std::size_t>(n), "%s %d",
                           first ? "  dead" : "", g.label(i));
        first = false;
    }
    const bool trouble = g.disagreeingMembers() > 0;
    const bool partial = g.presentCount() < g.memberCount();
    return driveRow(line, trouble ? ui::Tone::Red : (partial ? ui::Tone::Amber : ui::Tone::Green));
}

/// MORE: the protections' counters — `protections DEGRADED  cuts 2  dead 18 13`. Green while
/// driving, amber while degraded, red while a cut or a refusal is in force.
inline competition::MenuLine driveProtectionCountsRow(const teleop::DriveProtectionState& s) {
    char line[160];
    std::snprintf(line, sizeof line, "protections %.8s  cuts %d%s%.40s", protectionModeWord(s.mode),
                  s.cutCount, s.dead[0] != '\0' ? "  dead " : "", s.dead);
    const ui::Tone tone = s.mode == teleop::ProtectionMode::Driving    ? ui::Tone::Green
                          : s.mode == teleop::ProtectionMode::Degraded ? ui::Tone::Amber
                                                                       : ui::Tone::Red;
    return driveRow(line, tone);
}

/// MORE: the last cut's words (`last CUT: FIGHT on LEFT -- port(s) 18`), or that there was none
/// at this driver-control entry.
inline competition::MenuLine driveLastCutRow(const teleop::DriveProtectionState& s) {
    if (s.lastCut[0] == '\0') {
        return driveRow("last cut: none this entry", ui::Tone::Grey);
    }
    char line[160];
    std::snprintf(line, sizeof line, "last %.90s", s.lastCut);
    return driveRow(line, ui::Tone::Amber);
}

/// MORE: each side's highest member current this entry — `peak A this entry: L 2.31  R 1.80` for
/// a tank's two groups (LEFT, then RIGHT, the chassis's order); `g0 … g3` for any other count.
inline competition::MenuLine drivePeaksRow(const teleop::DriveProtectionState& s) {
    char line[96];
    int n = std::snprintf(line, sizeof line, "peak A this entry:");
    const std::size_t groups =
        s.groupCount < teleop::kMaxProtectedGroups ? s.groupCount : teleop::kMaxProtectedGroups;
    for (std::size_t g = 0; g < groups && n > 0 && static_cast<std::size_t>(n) < sizeof line;
         ++g) {
        const double a = std::isfinite(s.peakAmps[g]) ? s.peakAmps[g] : 0.0;
        if (groups == 2) {
            n += std::snprintf(line + n, sizeof line - static_cast<std::size_t>(n), " %s %.2f",
                               g == 0 ? "L" : " R", a);
        } else {
            n += std::snprintf(line + n, sizeof line - static_cast<std::size_t>(n), "  g%u %.2f",
                               static_cast<unsigned>(g), a);
        }
    }
    return driveRow(line, ui::Tone::White);
}

// ═══ THE PAGES ═════════════════════════════════════════════════════════════════════════

/// Which page is up.
enum class DrivePage : std::uint8_t {
    Main,     ///< the driver's page
    Details,  ///< R5's four, then the command, estimate, mode and battery
    More,     ///< the groups, the travel and the protections' counters
};

/// The page's buttons.
enum class DriveButtonId : std::uint8_t {
    Details,  ///< MAIN -> DETAILS
    More,     ///< DETAILS -> MORE
    Back,     ///< DETAILS or MORE -> MAIN
};

/// One button on a page.
struct DriveButton {
    DriveButtonId id = DriveButtonId::Back;  ///< which
    ui::Rect rect{};                         ///< where it is drawn (its hit box is ui::hitBox(rect))
    char label[ui::kMediumCols + 1]{};       ///< its label, MEDIUM, screen-safe
    bool armed = false;                      ///< false while the arming delay runs: drawn greyed, never hit
};

/// The most SMALL rows one page carries (nine rows of 16 px from y 32 end at 176, above the
/// button row at 180).
inline constexpr std::size_t kDriveMaxLines = 9;
/// The most buttons one page carries.
inline constexpr std::size_t kDriveMaxButtons = 2;

// ── layout (every number an ESTIMATE, A4 register HA-136) ───────────────────────────────────
/// The first row's top, under the header bar ("shulib Teleop"'s).
inline constexpr std::int16_t kDriveRowY0 = 32;
/// A row's left edge.
inline constexpr std::int16_t kDriveRowX = 8;
/// The bar's right-hand text (the BUILD stamp): "shulib Teleop"'s place for it.
inline constexpr std::int16_t kDriveBarRightX = 340;
/// The right-hand button slot (DETAILS on MAIN; BACK on DETAILS and MORE) — Calibrate's ABORT slot.
inline constexpr ui::Rect kDriveRightSlot{330, 180, 474, 234};
/// The left-hand button slot (MORE on DETAILS).
inline constexpr ui::Rect kDriveLeftSlot{6, 180, 150, 234};
static_assert(kDriveRowY0 + ui::kSmallHeight * static_cast<std::int16_t>(kDriveMaxLines)
                  <= kDriveRightSlot.y0,
              "a page's rows must end above its button row");
static_assert(ui::onScreen(kDriveRightSlot) && ui::onScreen(kDriveLeftSlot)
                  && !ui::overlaps(ui::hitBox(kDriveLeftSlot), ui::hitBox(kDriveRightSlot)),
              "the button slots must be on screen and their hit boxes apart");

/// Everything one page says, and which touches mean something.
struct DriveView {
    DrivePage page = DrivePage::Main;                    ///< which page
    ui::Tone bar = ui::Tone::Blue;                       ///< the header bar's colour
    competition::MenuLine header{};                      ///< the bar's word, left (MEDIUM, white)
    competition::MenuLine barRight{};                    ///< the BUILD stamp, right (SMALL, white)
    competition::MenuLine lines[kDriveMaxLines]{};       ///< the SMALL rows, top to bottom
    std::uint8_t lineCount = 0;                          ///< how many of `lines` are real
    DriveButton buttons[kDriveMaxButtons]{};             ///< the page's buttons
    std::uint8_t buttonCount = 0;                        ///< how many of `buttons` are real
};

/// Every row the three pages show, by name — the program fills it each refresh from the robot,
/// the boot and the protections' state; driveView() picks the page's and places them.
struct DriveRows {
    std::string_view buildTime{};          ///< "BUILD hh:mm:ss" (the bar's right)
    // MAIN
    competition::MenuLine protection{};    ///< the PROTECTION row (the shared panel's words)
    competition::MenuLine controller{};    ///< the controller row (the shared panel's words)
    // DETAILS: R5's four first (the run status, the two SOURCE rows, the CLAW row, the fault row)
    competition::MenuLine status{};        ///< this version's run status
    competition::MenuLine source1{};       ///< where the drivetrain numbers came from, row 1
    competition::MenuLine source2{};       ///< ...row 2
    competition::MenuLine claw{};          ///< the claw row (driver_panel.hpp's clawRowText)
    competition::MenuLine fault{};         ///< the ONE fault row (driver_panel.hpp's faultRowText)
    competition::MenuLine cmd{};           ///< driveCmdRow()
    competition::MenuLine est{};           ///< driveEstRow()
    competition::MenuLine mode{};          ///< driveModeRow()
    competition::MenuLine battery{};       ///< driveBatteryRow()
    // MORE
    competition::MenuLine left{};          ///< driveGroupRow("L", …)
    competition::MenuLine right{};         ///< driveGroupRow("R", …)
    competition::MenuLine travel{};        ///< driveTravelRow()
    competition::MenuLine protections{};   ///< driveProtectionCountsRow()
    competition::MenuLine lastCut{};       ///< driveLastCutRow()
    competition::MenuLine peaks{};         ///< drivePeaksRow()
};

// ═══ THE PROGRAM'S GLUE, AS PURE FUNCTIONS (file banner) ═══════════════════════════════════

/// DETAILS' two SOURCE rows into `rows`: where the drivetrain numbers came from — the TYPED TABLE
/// (`fromTable`; amber: a belief, not a measurement), a DEGRADED record (`degraded`; amber,
/// `MEASURED record | <reason>`, the port check's), or the record verified against the ports
/// (green). A null `reason` prints as "".
inline void driveSourceRows(DriveRows& rows, bool fromTable, bool degraded, const char* reason) {
    char source[400];
    ui::Tone tone = ui::Tone::Green;
    if (fromTable) {
        std::snprintf(source, sizeof source, "%.*s", static_cast<int>(kDriveTypedSource.size()),
                      kDriveTypedSource.data());
        tone = ui::Tone::Amber;
    } else if (degraded) {
        std::snprintf(source, sizeof source, "MEASURED record | %.360s",
                      reason != nullptr ? reason : "");
        tone = ui::Tone::Amber;
    } else {
        std::snprintf(source, sizeof source, "%.*s", static_cast<int>(kDriveMeasuredOk.size()),
                      kDriveMeasuredOk.data());
    }
    const SourceRows two = sourceRows(source);
    rows.source1 = driveRow(two.row[0], tone);
    rows.source2 = driveRow(two.row[1], tone);
}

/// What the live rows are made from, read by the program from the robot at each refresh — plain
/// values, so the choices made from them are host-tested (file banner).
struct DriveLiveFacts {
    double cutSeconds = 0.0;       ///< how long a protection cut holds (the driver file's config)
    bool connected = false;        ///< the controller answers
    double cmdVx = 0.0;            ///< the command the loop was handed: vx, in/s
    double cmdOmega = 0.0;         ///< ...and omega, rad/s
    double estX = 0.0;             ///< the fused estimate: x, inches
    double estY = 0.0;             ///< ...y, inches
    double estHeadingDeg = 0.0;    ///< ...heading, degrees
    /// ...and its quality class.
    localization::Localizer::Quality quality = localization::Localizer::Quality::Uninitialized;
    teleop::DriveMode mode{};                   ///< the driver mode
    teleop::FieldCentricState fieldCentric{};   ///< field-centric's state
    bool hasOdometry = false;      ///< the robot has drive encoders
    double headingDisagreement = 0.0;  ///< encoder-vs-IMU heading, rad/tick (with drive encoders)
    double batteryVolts = 0.0;     ///< the pack
    double travelLeft = 0.0;       ///< the last tick's LEFT travel, inches (with drive encoders)
    double travelRight = 0.0;      ///< ...RIGHT
    int faultCount = 0;            ///< the fault latch's count (0: no fault)
    diag::FaultCode firstFault = diag::FaultCode::None;  ///< the latch's first fault
    diag::FaultCode lastFault = diag::FaultCode::None;   ///< ...and its last
    std::string_view clawText{};   ///< the claw row's words (driver_panel.hpp's clawRowText)
    bool clawAbsent = false;       ///< a claw is ABSENT (driver_panel.hpp's anyClawAbsent)
    bool clawFreed = false;        ///< a claim was released at this entry (FREED)
};

/// Every LIVE row of the three pages into `rows`, from `f`, the protections' `state` and the
/// drive's two groups, `left` then `right`: MAIN's PROTECTION and controller rows (the shared
/// panel's words), DETAILS' fault and claw rows (red while any fault is latched; amber while a
/// claw is ABSENT or a claim was FREED — each something the driver must know before trusting the
/// buttons), the command, estimate, mode and battery, and MORE's six. The run status and the
/// source rows are the boot's, set once (driveSourceRows()).
inline void driveLiveRows(DriveRows& rows, const DriveLiveFacts& f,
                          const teleop::DriveProtectionState& state, const hal::MotorGroup& left,
                          const hal::MotorGroup& right) {
    // MAIN: what explains a drive that stopped, and whether the sticks reach the robot.
    const PanelRow guard = protectionRowText(state, f.cutSeconds);
    rows.protection = driveRow(guard.text, guard.tone);
    const PanelRow controller = controllerRowText(f.connected);
    rows.controller = driveRow(controller.text, controller.tone);
    // DETAILS
    rows.cmd = driveCmdRow(f.cmdVx, f.cmdOmega, f.connected);
    rows.est = driveEstRow(f.estX, f.estY, f.estHeadingDeg, f.quality);
    rows.mode = driveModeRow(f.mode, f.fieldCentric);
    rows.battery = driveBatteryRow(f.hasOdometry, f.hasOdometry ? f.headingDisagreement : 0.0,
                                   f.batteryVolts);
    const bool faulted = f.faultCount > 0;
    rows.fault = driveRow(faultRowText(faulted ? f.faultCount : 0, f.firstFault, f.lastFault).text,
                          faulted ? ui::Tone::Red : ui::Tone::Green);
    rows.claw = driveRow(f.clawText, f.clawAbsent || f.clawFreed ? ui::Tone::Amber : ui::Tone::White);
    // MORE
    rows.travel = driveTravelRow(f.hasOdometry, f.hasOdometry ? f.travelLeft : 0.0,
                                 f.hasOdometry ? f.travelRight : 0.0);
    rows.left = driveGroupRow("L", left);
    rows.right = driveGroupRow("R", right);
    rows.protections = driveProtectionCountsRow(state);
    rows.lastCut = driveLastCutRow(state);
    rows.peaks = drivePeaksRow(state);
}

/// The boot's PROTECTIONS line into `out`, GENERATED from the config the driver file's loop runs
/// with (`p`, its `tickSeconds`, and the groups' monitors' `persistTicks` — SideMonitorConfig's),
/// never retyped (the T4 brief's D4): the frozen rule, the dead limit, the fight and over-current
/// cut and its length, and whether over-current cuts or only warns. Returns snprintf's count.
inline int driveProtectionsLine(char* out, std::size_t cap, const teleop::DriveProtectionConfig& p,
                                double tickSeconds, int persistTicks) {
    const double tickMs = tickSeconds * 1000.0;
    return std::snprintf(out, cap,
                         "drive protections (the library's, in every program): a drive port "
                         "reading frozen %d reads is dropped after %.0f ms more; more than %d dead "
                         "in total stops the drive for the run; a fight, or over %.2f A for %.0f ms, "
                         "cuts both sides for %.0f ms (over-current cuts %s) -- HA-132/175/176, "
                         "unmeasured",
                         p.frozenSuspectReads, persistTicks * tickMs, teleop::kMaxDeadTotal,
                         p.overCurrentAmps, p.overCurrentTicks * tickMs, p.cutTicks * tickMs,
                         p.overCurrentCuts ? "ON" : "OFF: warnings only");
}

/// The boot's DEGRADED line into `out` when `deadPorts` recorded drive ports were EMPTY at boot —
/// true, said as a WARNING; false and `out` untouched when none were. `reason` is the port check's
/// (at most 80 characters of it). A null `reason` prints as "".
inline bool driveDegradedBootLine(char* out, std::size_t cap, std::size_t deadPorts,
                                  const char* reason) {
    if (deadPorts == 0) {
        return false;
    }
    std::snprintf(out, cap,
                  "DEGRADED: %u recorded drive port(s) are EMPTY and were NOT constructed; their "
                  "group slots are holes and every read is over the PRESENT members. %.80s",
                  static_cast<unsigned>(deadPorts), reason != nullptr ? reason : "");
    return true;
}

/// The level a boot line about a decision is logged at: INFO when it went the good way (`good`:
/// the boot source is the record, the port check is OK), a WARNING otherwise (the typed table; a
/// DEGRADED or refused port check).
[[nodiscard]] constexpr hal::LogLevel driveBootLevel(bool good) noexcept {
    return good ? hal::LogLevel::Info : hal::LogLevel::Warn;
}

namespace detail {
/// Place `row` as the view's next SMALL row (its text and tone kept; its place and size set).
inline void addDriveRow(DriveView& v, const competition::MenuLine& row) noexcept {
    if (v.lineCount >= kDriveMaxLines) {
        return;
    }
    competition::MenuLine& l = v.lines[v.lineCount];
    l = row;
    l.x = kDriveRowX;
    l.y = static_cast<std::int16_t>(kDriveRowY0 + ui::kSmallHeight * v.lineCount);
    l.size = ui::TextSize::Small;
    ++v.lineCount;
}

/// Add a button.
inline void addDriveButton(DriveView& v, DriveButtonId id, ui::Rect rect, std::string_view label,
                           bool armed) noexcept {
    if (v.buttonCount >= kDriveMaxButtons) {
        return;
    }
    DriveButton& b = v.buttons[v.buttonCount++];
    b.id = id;
    b.rect = rect;
    ui::copyScreenText(b.label, sizeof b.label, label, ui::kMediumCols);
    b.armed = armed;
}

/// The bar: `word` left (MEDIUM, white), the build stamp right (SMALL, white).
inline void setDriveBar(DriveView& v, std::string_view word, std::string_view buildTime) noexcept {
    v.bar = ui::Tone::Blue;
    v.header.x = 8;
    v.header.y = 4;
    v.header.size = ui::TextSize::Medium;
    v.header.tone = ui::Tone::White;
    ui::copyScreenText(v.header.text, sizeof v.header.text, word, calibration::kHeaderCols);
    v.barRight.x = kDriveBarRightX;
    v.barRight.y = 7;
    v.barRight.size = ui::TextSize::Small;
    v.barRight.tone = ui::Tone::White;
    ui::copyScreenText(v.barRight.text, sizeof v.barRight.text, buildTime, ui::kSmallCols);
}
}  // namespace detail

/// The view of `page` from `rows` (file banner: what is on which page), its buttons `armed` or not
/// (DrivePager::armed()).
inline DriveView driveView(DrivePage page, const DriveRows& rows, bool armed) noexcept {
    DriveView v{};
    v.page = page;
    switch (page) {
        case DrivePage::Main:
            detail::setDriveBar(v, kDriveHeaderWord, rows.buildTime);
            detail::addDriveRow(v, rows.protection);
            detail::addDriveRow(v, rows.controller);
            detail::addDriveButton(v, DriveButtonId::Details, kDriveRightSlot, kDriveDetailsLabel,
                                   armed);
            break;
        case DrivePage::Details:
            detail::setDriveBar(v, kDriveDetailsWord, rows.buildTime);
            detail::addDriveRow(v, rows.status);
            detail::addDriveRow(v, rows.source1);
            detail::addDriveRow(v, rows.source2);
            detail::addDriveRow(v, rows.claw);
            detail::addDriveRow(v, rows.fault);
            detail::addDriveRow(v, rows.cmd);
            detail::addDriveRow(v, rows.est);
            detail::addDriveRow(v, rows.mode);
            detail::addDriveRow(v, rows.battery);
            detail::addDriveButton(v, DriveButtonId::More, kDriveLeftSlot, kDriveMoreLabel, armed);
            detail::addDriveButton(v, DriveButtonId::Back, kDriveRightSlot, kDriveBackLabel, armed);
            break;
        case DrivePage::More:
            detail::setDriveBar(v, kDriveMoreWord, rows.buildTime);
            detail::addDriveRow(v, rows.left);
            detail::addDriveRow(v, rows.right);
            detail::addDriveRow(v, rows.travel);
            detail::addDriveRow(v, rows.protections);
            detail::addDriveRow(v, rows.lastCut);
            detail::addDriveRow(v, rows.peaks);
            detail::addDriveButton(v, DriveButtonId::Back, kDriveRightSlot, kDriveBackLabel, armed);
            break;
    }
    return v;
}

/// A layout key for `page` (ui::LayoutKey over the page and its buttons' rectangles): the arming
/// clock restarts when it changes, which is exactly a flip.
inline std::uint32_t drivePageKey(DrivePage page) noexcept {
    ui::LayoutKey k;
    k.mix(static_cast<std::uint32_t>(page));
    const DriveView v = driveView(page, DriveRows{}, false);
    for (std::size_t i = 0; i < v.buttonCount; ++i) {
        k.mix(static_cast<std::uint32_t>(v.buttons[i].id));
        k.mix(v.buttons[i].rect);
    }
    return k.value();
}

/// What to repaint between the view ON THE SCREEN (`was`) and the one to show (`now`).
struct DriveRepaint {
    bool whole = false;          ///< everything: a new page, a new bar colour, or nothing painted yet
    bool header = false;         ///< the bar's word changed (on the same page)
    bool barRight = false;       ///< the BUILD stamp changed
    std::uint16_t lines = 0;     ///< bit i: row i's text or tone changed (or it appeared, or went)
    std::uint8_t buttons = 0;    ///< bit i: button i changed (its label, place or armed state)
};

/// THE REPAINT RULE (file banner): only the visible page is painted, and only what changed — a row
/// whose text or tone is the same is never repainted (PROS leaks every screen print; R3d Part 2F's
/// rule). `painted` is false before the first paint (everything then). A different page, or a
/// different bar colour, is a WHOLE repaint; a row present on one view and not the other is
/// repainted (an empty row erases what was there).
[[nodiscard]] inline DriveRepaint driveRepaint(const DriveView& was, const DriveView& now,
                                               bool painted) noexcept {
    DriveRepaint r{};
    if (!painted || was.page != now.page || was.bar != now.bar) {
        r.whole = true;
        return r;
    }
    const auto differs = [](const competition::MenuLine& a, const competition::MenuLine& b) {
        return a.tone != b.tone || std::string_view{a.text} != std::string_view{b.text};
    };
    r.header = differs(was.header, now.header);
    r.barRight = differs(was.barRight, now.barRight);
    const std::size_t rows = was.lineCount > now.lineCount ? was.lineCount : now.lineCount;
    for (std::size_t i = 0; i < rows && i < kDriveMaxLines; ++i) {
        const bool inWas = i < was.lineCount;
        const bool inNow = i < now.lineCount;
        if (inWas != inNow || (inNow && differs(was.lines[i], now.lines[i]))) {
            r.lines = static_cast<std::uint16_t>(r.lines | (1u << i));
        }
    }
    const std::size_t buttons = was.buttonCount > now.buttonCount ? was.buttonCount
                                                                   : now.buttonCount;
    for (std::size_t i = 0; i < buttons && i < kDriveMaxButtons; ++i) {
        const bool inWas = i < was.buttonCount;
        const bool inNow = i < now.buttonCount;
        const DriveButton& a = was.buttons[i];
        const DriveButton& b = now.buttons[i];
        if (inWas != inNow
            || (inNow && (a.id != b.id || !(a.rect == b.rect) || a.armed != b.armed
                          || std::string_view{a.label} != std::string_view{b.label}))) {
            r.buttons = static_cast<std::uint8_t>(r.buttons | (1u << i));
        }
    }
    return r;
}

/// Row `i` as a PARTIAL repaint paints it (driveRepaint() marked it): the row `now` shows there,
/// or — when `now` has no row `i` — the row `was` showed there with its text EMPTIED, painted over
/// its old place, so a row that went away is erased and never left standing.
[[nodiscard]] inline competition::MenuLine driveLineToPaint(const DriveView& was,
                                                           const DriveView& now,
                                                           std::size_t i) noexcept {
    if (i < now.lineCount && i < kDriveMaxLines) {
        return now.lines[i];
    }
    competition::MenuLine gone = i < kDriveMaxLines ? was.lines[i] : competition::MenuLine{};
    gone.text[0] = '\0';
    return gone;
}

/// THE PAGE FLIP (file banner). One per run of the program, fed every touch read.
class DrivePager {
public:
    /// `releaseCountAtStart` and `pressCountAtStart`: PROS's release and press counts when the
    /// screen first appears, so a tap made before the program existed is never replayed.
    DrivePager(std::int32_t releaseCountAtStart, std::int32_t pressCountAtStart) noexcept
        : edge_{releaseCountAtStart}, pressAtLayout_{pressCountAtStart} {}

    /// The page up now.
    [[nodiscard]] DrivePage page() const noexcept { return page_; }

    /// Whether the page's buttons take touches at `nowSeconds` (the arming delay after the page
    /// appeared). Call it each refresh, as the view is made: it also tells the arming clock which
    /// page is up.
    [[nodiscard]] bool armed(double nowSeconds) noexcept {
        return ui::ArmingClock::armedAfter(clock_.observe(drivePageKey(page_), nowSeconds));
    }

    /// One touch read at `nowSeconds` — `s` the release-edge sample, `pressCount` PROS's press count
    /// (+1 each time a finger lands). Returns true when it FLIPPED the page: a NEW release, of a
    /// press that landed after the page appeared, on an armed button of the page.
    bool touch(const ui::TouchSample& s, std::int32_t pressCount, double nowSeconds) noexcept {
        const bool armedNow = armed(nowSeconds);
        if (!edge_.take(s)) {
            return false;  // no new release (or an error read, which never moves the count)
        }
        if (!armedNow || pressCount <= pressAtLayout_) {
            return false;  // too soon after the page appeared, or a press held across the flip
        }
        const DriveView v = driveView(page_, DriveRows{}, true);
        const int hit = ui::hitTest(v.buttons, v.buttonCount, s.x, s.y);
        if (hit < 0) {
            return false;
        }
        switch (v.buttons[hit].id) {
            case DriveButtonId::Details: page_ = DrivePage::Details; break;
            case DriveButtonId::More: page_ = DrivePage::More; break;
            case DriveButtonId::Back: page_ = DrivePage::Main; break;
        }
        pressAtLayout_ = pressCount;
        (void)clock_.observe(drivePageKey(page_), nowSeconds);  // the new page's delay starts now
        return true;
    }

private:
    ui::ReleaseEdge edge_;
    ui::ArmingClock clock_{};
    DrivePage page_ = DrivePage::Main;
    std::int32_t pressAtLayout_;
};

}  // namespace shulib::bench
