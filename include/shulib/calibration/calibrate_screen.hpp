#pragma once
//
// calibrate_screen — how the standard Calibrate PAINTS the brain's screen (chunk CAL1, Part A1,
// 2026-10-08). The palette; the painter, which brings the screen to a new calibration::PanelView by
// drawing only the difference; and the program's own screens (starting, cannot start, not driver
// control, a program fault, and the competition-state screen). Everything it draws goes through
// hal::IBrainScreen, so the host suite RUNS it (hal::fake::FakeBrainScreen records every call)
// where until now only what the screen SAYS (calibrate_panel.hpp's viewFor() and TouchGate) was
// host-tested.
//
// ── Where it comes from ─────────────────────────────────────────────────────────────
// Its rules, its palette and its drawing calls, in their order, are those of the painter in
// "shulib Calibrate" (src/programs/calibrate_program.cpp — `PanelPainter`, its header and button
// helpers and its own screens, written at R3d Part 2F-2 on 2026-09-24), re-expressed through the
// seam. CAL1 Part B (2026-11-05) moved robot two onto the library's Calibrate and deleted that
// program and its painter; this file is the only one. Its words are the program's, with the
// words that named robot two generalized for every team (no slot numbers, no "Bench Tests": a
// team's project may have neither), and a few added for the standard program's refusals.
//
// ── The painter's rules ─────────────────────────────────────────────────────────────
//   * a change of screen, or of any line's place or size, is a full repaint;
//   * a line whose COLOUR changed, the header, and any button change are painted the tick they
//     happen;
//   * a line whose TEXT changed (a live number) is repainted at most every kLiveRepaintSeconds.
//     PROS 4.2.2's print_at is BELIEVED to leak its formatted buffer on every call (R3d Part 2F's
//     reading of the vendored libpros.a found no free; never measured), so a live number reprinted
//     every 10 ms tick would leak megabytes over a few runs; 5 Hz is legible and leaks two orders
//     of magnitude less. The screen counts its prints (IBrainScreen::prints()) so a program can
//     report the count;
//   * an in-place text change never erases: the new text is printed padded with spaces to the old
//     text's length, and VEXos draws text pen-on-ERASER (an opaque cell — a hardware finding, Bench
//     Tests' menu), so the old characters are covered without a flicker;
//   * a button that vanished is erased to the ground; one that is new, moved, re-labelled or
//     armed is painted whole.
// Every text budget, line height and position is an ESTIMATE (A4 register HA-136); every literal
// here is checked against its budget at compile time (screen_text::kAll), and every other text is
// cut by the screen's sanitizer (hal::screenSafeLine()), which also turns a '%' into '?'. An own
// screen that had to cut or drop text a CALLER passed (paintCannot(), paintNotDriverControl(),
// paintStateScreen()) says so on its last line: the full text is on the serial log. Two cuts are
// NOT said, because they are clamps that keep the layout whole, not messages: render() cuts a
// view's header word at kHeaderCols, each line at its size's budget and each button label at
// MEDIUM's; and the header, on every screen that has one, cuts the build time at kBuildTimeCols.
// The views viewFor() builds are not expected to reach the first (their literals are checked at
// compile time, and calibrate_panel's host test checks every view its calibration runs reach
// against these budgets); "BUILD hh:mm:ss" is exactly the second's width.
//
// PROS-free, allocation-free. NOTHING HERE HAS RUN ON A ROBOT in this form; the calls it makes
// are the ones "shulib Calibrate" has made on robot two since 2026-09-24.

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string_view>

#include "shulib/calibration/calibrate_panel.hpp"
#include "shulib/hal/brain_screen.hpp"

namespace shulib::calibration {

/// The least time between two in-place repaints of one line whose TEXT changed and whose colour
/// did not — a live number. 5 Hz: legible, and it bounds PROS's believed per-print leak (header).
inline constexpr double kLiveRepaintSeconds = 0.2;

/// The characters the header's build-time text may take: "BUILD hh:mm:ss" is exactly this many.
/// A longer text is cut here, so it can never run into the screen's right edge (checked below).
inline constexpr std::size_t kBuildTimeCols = 14;

// ── The palette ─────────────────────────────────────────────────────────────────────
// Dark ground; the header bar's colour IS the state (blue unpowered, amber powered, green saved,
// red stopped). Contrast ratios (WCAG formula) against what each is drawn on: white text on the
// ground 16.2:1, amber 9.6, green 8.1, grey 7.3, red 5.0; white on the blue bar 6.0; the dark
// ground colour on the amber, green and red bars 9.6 / 8.1 / 5.0; white on the ABORT fill 5.6.
// The old panel's live buttons were a 1.38:1 fill on this ground.

inline constexpr std::uint32_t kColBg = 0x101418;         ///< the ground, and dark text on a bright bar
inline constexpr std::uint32_t kColText = 0xF0F0F0;       ///< ordinary text
inline constexpr std::uint32_t kColBlue = 0x1E5AA8;       ///< the unpowered header; the question boxes
inline constexpr std::uint32_t kColAmber = 0xE8B23A;      ///< powered: stand clear
inline constexpr std::uint32_t kColGreen = 0x39C36E;      ///< ready, saved, past the floor; DONE's fill
inline constexpr std::uint32_t kColRed = 0xE2564A;        ///< stopped, not saved; ABORT's edge
inline constexpr std::uint32_t kColGrey = 0x9AA4AE;       ///< quiet text; a greyed button's label
inline constexpr std::uint32_t kColGreyFill = 0x3A4450;   ///< a greyed (arming) button's fill
inline constexpr std::uint32_t kColAbortFill = 0xB0302A;  ///< ABORT's fill (white on it: 5.6:1)
inline constexpr std::uint32_t kColEdge = 0xC8D8F0;       ///< a live button's edge; Blue text, if ever asked

namespace detail {

/// The seam's text size for one of the panel's.
[[nodiscard]] constexpr hal::ScreenTextSize screenSizeOf(TextSize s) noexcept {
    return s == TextSize::Small   ? hal::ScreenTextSize::Small
           : s == TextSize::Large ? hal::ScreenTextSize::Large
                                  : hal::ScreenTextSize::Medium;
}

/// printText()'s call, through the seam: cut at the size's budget, padded to `padTo`.
inline void printPanelText(hal::IBrainScreen& s, TextSize size, std::int16_t x, std::int16_t y,
                           std::uint32_t pen, std::uint32_t eraser, const char* text,
                           std::size_t padTo = 0) {
    s.print(screenSizeOf(size), x, y, pen, eraser, text, budgetFor(size), padTo);
}

/// `text` copied into `buf` — at most a line's worth (hal::kScreenLineBytes - 1 characters, wider
/// than every budget) — and terminated, so a string_view, which need not be NUL-terminated, can be
/// handed on as a C string without anything reading past the view. THE one copy: every view the
/// painter draws or measures goes through it. Returns `buf`.
inline const char* terminatedCopy(std::string_view text,
                                  char (&buf)[hal::kScreenLineBytes]) noexcept {
    const std::size_t n = text.size() < sizeof buf - 1 ? text.size() : sizeof buf - 1;
    for (std::size_t i = 0; i < n; ++i) {
        buf[i] = text[i];
    }
    buf[n] = '\0';
    return buf;
}

/// The same for a string_view (through terminatedCopy()), cut at `cols`.
inline void printPanelView(hal::IBrainScreen& s, TextSize size, std::int16_t x, std::int16_t y,
                           std::uint32_t pen, std::uint32_t eraser, std::string_view text,
                           std::size_t cols) {
    char buf[hal::kScreenLineBytes];
    s.print(screenSizeOf(size), x, y, pen, eraser, terminatedCopy(text, buf), cols);
}

/// The same, cut at the size's budget.
inline void printPanelView(hal::IBrainScreen& s, TextSize size, std::int16_t x, std::int16_t y,
                           std::uint32_t pen, std::uint32_t eraser, std::string_view text) {
    printPanelView(s, size, x, y, pen, eraser, text, budgetFor(size));
}

/// How many characters `text` draws before any cut — what hal::screenSafeLine() would write with
/// no budget (a UTF-8 dash or other character is one; capped at hal::kScreenLineBytes - 1, which
/// is wider than every budget). A screen compares it with a line's budget to know it cut the
/// text. Null draws nothing.
[[nodiscard]] inline std::size_t columnsOf(const char* text) noexcept {
    char out[hal::kScreenLineBytes];
    return hal::screenSafeLine(out, sizeof out, text, sizeof out);
}

/// The same for a string_view (through terminatedCopy()).
[[nodiscard]] inline std::size_t columnsOf(std::string_view text) noexcept {
    char in[hal::kScreenLineBytes];
    return columnsOf(terminatedCopy(text, in));
}

/// A text tone as a colour on the ground.
[[nodiscard]] constexpr std::uint32_t panelTextColour(PanelTone t) noexcept {
    switch (t) {
        case PanelTone::Amber: return kColAmber;
        case PanelTone::Green: return kColGreen;
        case PanelTone::Red: return kColRed;
        case PanelTone::Grey: return kColGrey;
        case PanelTone::Blue: return kColEdge;  // no panel line is Blue today; light blue if one is
        case PanelTone::White: return kColText;
    }
    return kColText;
}

/// A header tone as the bar's fill.
[[nodiscard]] constexpr std::uint32_t panelBarColour(PanelTone t) noexcept {
    switch (t) {
        case PanelTone::Amber: return kColAmber;
        case PanelTone::Green: return kColGreen;
        case PanelTone::Red: return kColRed;
        case PanelTone::Grey: return kColGreyFill;
        case PanelTone::Blue:
        case PanelTone::White: return kColBlue;
    }
    return kColBlue;
}

/// The text colour that reads on a header bar of tone `t`.
[[nodiscard]] constexpr std::uint32_t panelBarTextColour(PanelTone t) noexcept {
    return (t == PanelTone::Amber || t == PanelTone::Green || t == PanelTone::Red) ? kColBg
                                                                                   : kColText;
}

// ── the header bar: the step word left (MEDIUM), the build time right (SMALL) ─────────
inline constexpr std::int16_t kPainterHeaderWordX = 8;
inline constexpr std::int16_t kPainterHeaderWordY = 4;
inline constexpr std::int16_t kPainterBuildX = 340;
inline constexpr std::int16_t kPainterBuildY = 7;

// The header word (at most kHeaderCols MEDIUM characters) must end before the build time, and the
// build time (at most kBuildTimeCols SMALL characters) before the right edge — at the widths the
// budgets themselves imply (472 px over the budget), which are ESTIMATES like the budgets.
static_assert(kPainterHeaderWordX
                      + static_cast<int>(kHeaderCols) * (472 / static_cast<int>(kMediumCols))
                  < kPainterBuildX,
              "the header word could reach the build time");
static_assert(kPainterBuildX
                      + static_cast<int>(kBuildTimeCols) * (472 / static_cast<int>(kSmallCols))
                  <= 472,
              "the build time could run off the right edge");

// ── the competition-state screen's colours: src/common/state_screens.cpp's, unchanged ──
inline constexpr std::uint32_t kStateGround = 0x000000;
inline constexpr std::uint32_t kStateWord = 0xF0C000;
inline constexpr std::uint32_t kStateText = 0xFFFFFF;

// The one WRAPPED shape a screen literal may take: the competition-state screen's `why`, SMALL,
// broken by wrapWords() onto at most this many lines (paintStateScreen() draws it with these).
inline constexpr std::size_t kStateWhyLines = 2;

/// True when `text` wraps completely onto `lines` lines of `cols` characters — the shape
/// paintStateScreen() draws its `why` in. Any other wrapped shape is refused (false): a literal
/// meant for one adds its case here, with the screen that draws it.
[[nodiscard]] constexpr bool wrapsCompletely(std::string_view text, std::size_t cols,
                                             std::size_t lines) noexcept {
    return cols == kSmallCols && lines == kStateWhyLines
           && wrapWords<kSmallCols, kStateWhyLines>(text).complete;
}

}  // namespace detail

/// The program's own words, each a named constant so the compile-time check at the end of this
/// namespace can see it. A new literal belongs HERE, with its budget in kAll, and its comment
/// begins with the size it is drawn at ("Header word", LARGE, MEDIUM, SMALL — a host test holds
/// that size, kAll's budget and the size the painter draws it at to one another). The positions
/// they are drawn at assume the panel's estimated line heights (LARGE 34, MEDIUM 20, SMALL 16 px)
/// and keep every line's bottom above row 236.
namespace screen_text {
/// Header word (kHeaderCols): the first screen, while the census runs.
inline constexpr std::string_view kHeaderStarting = "CALIBRATE";
/// Header word: a refusal before any run could start.
inline constexpr std::string_view kHeaderCannot = "CANNOT START";
/// Header word: the loop stopped — not driver control, or a program fault.
inline constexpr std::string_view kHeaderStopped = "STOPPED -- MOTORS OFF";
/// LARGE (kLargeCols): the first screen's first line.
inline constexpr std::string_view kHandsOff = "HANDS OFF";
/// LARGE: the first screen's second line.
inline constexpr std::string_view kStartingUp = "STARTING UP";
/// LARGE: the drive motors' cartridge is not stated.
inline constexpr std::string_view kCartridgeUnset = "CARTRIDGE UNSET";
/// LARGE: the drive kind is not stated.
inline constexpr std::string_view kDriveUnset = "DRIVE KIND UNSET";
/// LARGE: the stated drive kind is not a tank, the one kind Calibrate measures.
inline constexpr std::string_view kNotTank = "NOT A TANK";
/// LARGE: the stated facts contradict themselves (a mechanism port named twice, out of range…).
inline constexpr std::string_view kFactsBad = "FACTS REFUSED";
/// LARGE: the census did not find exactly one IMU.
inline constexpr std::string_view kImuCount = "IMU COUNT NOT 1";
/// LARGE: fewer than two drive candidates.
inline constexpr std::string_view kTooFewMotors = "TOO FEW MOTORS";
/// LARGE: a motor could not be made.
inline constexpr std::string_view kMotorsRefused = "MOTORS REFUSED";
/// LARGE: the IMU could not be made.
inline constexpr std::string_view kImuRefused = "IMU REFUSED";
/// LARGE: something the program did not expect stopped it.
inline constexpr std::string_view kProgramFault = "PROGRAM FAULT";
/// LARGE: the competition state is not driver control, enabled.
inline constexpr std::string_view kNotDriverControl = "NOT DRIVER CONTROL";
/// MEDIUM (kMediumCols): the first screen's third line.
inline constexpr std::string_view kFindingDevices = "finding the motors and the IMU";
/// SMALL (kSmallCols): a refusal's footer.
inline constexpr std::string_view kNothingPowered = "nothing is powered -- fix it, then restart it";
/// SMALL: where to look next after a refusal, when its text fitted on the screen.
inline constexpr std::string_view kDiagnosisTool = "the census is on the serial log";
/// SMALL: where to look next when a refusal's text did NOT fit on the screen.
inline constexpr std::string_view kMoreOnSerial = "the full text is on the serial log";
/// SMALL: a program fault's footer.
inline constexpr std::string_view kFaultIdle = "every motor is at 0 V, coasting";
/// SMALL: the not-driver-control screen, line 1.
inline constexpr std::string_view kNdc1 = "It powers the motors, so it runs ONLY in";
/// SMALL: the not-driver-control screen, line 2.
inline constexpr std::string_view kNdc2 = "DRIVER CONTROL, enabled. Motors: 0 V, coasting.";
/// SMALL: the not-driver-control screen, line 3.
inline constexpr std::string_view kNdc3 = "Unplug any competition switch or field cable,";
/// SMALL: the not-driver-control screen, line 4.
inline constexpr std::string_view kNdc4 = "and NEVER start it from the CONTROLLER's menu.";
/// SMALL: the not-driver-control screen, line 5.
inline constexpr std::string_view kNdc5 = "Any result is on the serial log too ([R3D]).";
/// SMALL: the competition-state screen (disabled(), competition_initialize(), autonomous()),
/// under the state's name.
inline constexpr std::string_view kState1 = "This program runs ONLY in DRIVER CONTROL, enabled.";
/// SMALL: the competition-state screen, what to look at.
inline constexpr std::string_view kState3 = "Look at the CONTROLLER: it names this state.";
/// SMALL: the competition-state screen, what to do (first line).
inline constexpr std::string_view kState4 = "Unplug any competition switch or field cable";
/// SMALL: the competition-state screen, what to do (second line).
inline constexpr std::string_view kState5 = "from the controller. This clears by itself.";
/// LARGE: the competition-state screen's name for disabled() — what a program passes as
/// paintStateScreen()'s `state`.
inline constexpr std::string_view kStateNameDisabled = "DISABLED";
/// LARGE: the name for competition_initialize() (robot two's other programs say "FIELD CONTROL
/// CONNECTED", 23 characters — five past the LARGE budget).
inline constexpr std::string_view kStateNameConnected = "FIELD CONNECTED";
/// LARGE: the name for autonomous().
inline constexpr std::string_view kStateNameAutonomous = "AUTONOMOUS";
/// SMALL, wrapped onto two lines: why disabled() shows the state screen — paintStateScreen()'s
/// `why`.
inline constexpr std::string_view kStateWhyDisabled =
    "VEXos reports the robot DISABLED, so the driver task is not running.";
/// SMALL, wrapped onto two lines: why competition_initialize() does.
inline constexpr std::string_view kStateWhyConnected =
    "A competition switch or field is connected through the controller.";
/// SMALL, wrapped onto two lines: why autonomous() does.
inline constexpr std::string_view kStateWhyAutonomous =
    "No motion here, ever: calibration powers the motors only while a person stands by the "
    "battery.";

/// One literal and the budget it must fit.
struct Checked {
    std::string_view text;  ///< the literal
    std::size_t cols;       ///< its line's budget
    /// 1: one line of `cols`. More: wrapped by wrapWords() onto at most this many lines of `cols`
    /// (detail::wrapsCompletely() names the one wrapped shape a screen draws).
    std::size_t lines = 1;
};

/// EVERY literal above with its budget — the list the compile-time check walks (and a host test
/// holds this list and the literals above to one another, so a literal cannot be left out).
inline constexpr Checked kAll[] = {
    {kHeaderStarting, kHeaderCols},  {kHeaderCannot, kHeaderCols},
    {kHeaderStopped, kHeaderCols},   {kHandsOff, kLargeCols},
    {kStartingUp, kLargeCols},       {kCartridgeUnset, kLargeCols},
    {kDriveUnset, kLargeCols},       {kNotTank, kLargeCols},
    {kFactsBad, kLargeCols},         {kImuCount, kLargeCols},
    {kTooFewMotors, kLargeCols},     {kMotorsRefused, kLargeCols},
    {kImuRefused, kLargeCols},       {kProgramFault, kLargeCols},
    {kNotDriverControl, kLargeCols}, {kFindingDevices, kMediumCols},
    {kNothingPowered, kSmallCols},   {kDiagnosisTool, kSmallCols},
    {kMoreOnSerial, kSmallCols},     {kFaultIdle, kSmallCols},
    {kNdc1, kSmallCols},             {kNdc2, kSmallCols},
    {kNdc3, kSmallCols},             {kNdc4, kSmallCols},
    {kNdc5, kSmallCols},             {kState1, kSmallCols},
    {kState3, kSmallCols},           {kState4, kSmallCols},
    {kState5, kSmallCols},           {kStateNameDisabled, kLargeCols},
    {kStateNameConnected, kLargeCols},
    {kStateNameAutonomous, kLargeCols},
    {kStateWhyDisabled, kSmallCols, detail::kStateWhyLines},
    {kStateWhyConnected, kSmallCols, detail::kStateWhyLines},
    {kStateWhyAutonomous, kSmallCols, detail::kStateWhyLines},
};

/// True when `c` fits: printable ASCII with no '%', and either one line of at most `c.cols`
/// characters or — a wrapped literal — every word on its `c.lines` lines.
[[nodiscard]] constexpr bool fits(const Checked& c) noexcept {
    if (c.lines <= 1) {
        return screenText(c.text, c.cols);
    }
    return screenText(c.text, c.text.size()) && detail::wrapsCompletely(c.text, c.cols, c.lines);
}

/// True when every literal in `list` fits its budget and is printable ASCII with no '%'. This
/// header's static_assert runs it over kAll at compile time, so a budget change that breaks a
/// literal fails the build. (A host test holds that fits() and allFit() refuse what they must — a
/// word one character over, a '%', a non-ASCII byte, a reason that does not wrap, a bad entry
/// anywhere in a list — so a check weakened to pass everything fails to build that test.)
template <std::size_t N>
[[nodiscard]] constexpr bool allFit(const Checked (&list)[N]) noexcept {
    for (const Checked& c : list) {
        if (!fits(c)) {
            return false;
        }
    }
    return true;
}
static_assert(allFit(kAll), "calibrate_screen: a screen literal does not fit its budget, or is not "
                            "ASCII, or contains a percent sign");
}  // namespace screen_text

/// Draws calibration screens on one IBrainScreen: the header bar (with the build TIME, so a stale
/// upload shows at a glance), PanelViews by difference (the header's rules), and the program's own
/// screens — each of which paints over whatever was there and marks the painter's record stale,
/// so the next render() repaints in full. Non-owning: the screen must outlive the painter. Not
/// thread-safe: one task paints.
class CalibratePainter {
public:
    /// Paint on `screen`. `buildTime` is the header's right-hand text — "BUILD hh:mm:ss", cut at
    /// kBuildTimeCols without a word on screen — and must outlive the painter (a string literal,
    /// or the program's own buffer); null paints none.
    CalibratePainter(hal::IBrainScreen& screen, const char* buildTime) noexcept
        : s_{screen}, buildTime_{buildTime != nullptr ? buildTime : ""} {}

    /// The next render() is a full repaint (a run was rebuilt, or another screen painted over it).
    void invalidate() noexcept { valid_ = false; }

    /// Bring the screen to `v` at `now` seconds, painting only what changed (the header's rules).
    /// `v`'s header word must stay valid while it is on the screen (the panel's words are string
    /// literals). Text past its budget — the header word past kHeaderCols, a line past its
    /// size's, a button label past MEDIUM's — is cut WITHOUT a word on screen: a view is expected
    /// to fit, and the views viewFor() builds are checked to (the file comment says how).
    void render(const PanelView& v, double now) {
        bool full = !valid_ || v.screen != screen_ || v.lineCount != lineCount_;
        for (std::size_t i = 0; !full && i < v.lineCount; ++i) {
            full = !samePlace(v.lines[i], lines_[i]);
        }
        if (full) {
            eraseContent();
            paintHeader(v.headerTone, v.headerWord);
            for (std::size_t i = 0; i < v.lineCount; ++i) {
                const PanelLine& l = v.lines[i];
                detail::printPanelText(s_, l.size, l.x, l.y, detail::panelTextColour(l.tone),
                                       kColBg, l.text);
                lineAt_[i] = now;
            }
            for (std::size_t b = 0; b < v.buttonCount; ++b) {
                paintButton(v.buttons[b]);
            }
            adopt(v);
            valid_ = true;
            return;
        }
        if (v.headerTone != headerTone_ || std::strcmp(v.headerWord, headerWord_) != 0) {
            paintHeader(v.headerTone, v.headerWord);
            headerTone_ = v.headerTone;
            headerWord_ = v.headerWord;
        }
        for (std::size_t i = 0; i < v.lineCount; ++i) {
            const PanelLine& want = v.lines[i];
            PanelLine& have = lines_[i];
            const bool toneChanged = want.tone != have.tone;
            const bool textChanged = std::strcmp(want.text, have.text) != 0;
            if (!toneChanged && (!textChanged || now - lineAt_[i] < kLiveRepaintSeconds)) {
                continue;  // unchanged, or a live number that was repainted too recently
            }
            detail::printPanelText(s_, want.size, want.x, want.y,
                                   detail::panelTextColour(want.tone), kColBg, want.text,
                                   std::strlen(have.text));
            have = want;
            lineAt_[i] = now;
        }
        // Buttons: erase what vanished, then paint what is new or changed (armed, moved).
        for (std::size_t o = 0; o < buttonCount_; ++o) {
            bool kept = false;
            for (std::size_t b = 0; b < v.buttonCount && !kept; ++b) {
                kept = v.buttons[b].id == buttons_[o].id && v.buttons[b].rect == buttons_[o].rect;
            }
            if (!kept) {
                eraseButton(buttons_[o]);
            }
        }
        for (std::size_t b = 0; b < v.buttonCount; ++b) {
            bool same = false;
            for (std::size_t o = 0; o < buttonCount_ && !same; ++o) {
                same = sameButton(v.buttons[b], buttons_[o]);
            }
            if (!same) {
                paintButton(v.buttons[b]);
            }
        }
        buttonCount_ = v.buttonCount;
        for (std::size_t b = 0; b < v.buttonCount; ++b) {
            buttons_[b] = v.buttons[b];
        }
    }

    /// The first screen, while the census runs and the devices are made.
    void paintStarting() {
        valid_ = false;
        eraseContent();
        paintHeader(PanelTone::Blue, screen_text::kHeaderStarting);
        detail::printPanelView(s_, TextSize::Large, 8, 30, kColText, kColBg,
                               screen_text::kHandsOff);
        detail::printPanelView(s_, TextSize::Large, 8, 66, kColText, kColBg,
                               screen_text::kStartingUp);
        detail::printPanelView(s_, TextSize::Medium, 8, 110, kColGrey, kColBg,
                               screen_text::kFindingDevices);
    }

    /// A screen the run never gets past: a header of `tone`, a LARGE `headline`, `why` wrapped in
    /// MEDIUM (three lines at most), `fix` wrapped in SMALL (two lines), a `footer`, and where to
    /// look next — or, when ANY of the five did not fit (a word cut, a line dropped), that the
    /// whole text is on the serial log. `header`, `headline` and `footer` are meant to be
    /// screen_text's words (checked at compile time); any other text is cut at its budget, and
    /// said to be.
    void paintCannot(std::string_view header, PanelTone tone, std::string_view headline,
                     const char* why, const char* fix, std::string_view footer) {
        valid_ = false;
        eraseContent();
        paintHeader(tone, header);
        detail::printPanelView(s_, TextSize::Large, 8, 34, kColRed, kColBg, headline);
        bool cut = detail::columnsOf(header) > kHeaderCols
                   || detail::columnsOf(headline) > kLargeCols
                   || detail::columnsOf(footer) > kSmallCols;
        const auto w = wrapWords<kMediumCols, 3>(why != nullptr ? why : "");
        for (std::size_t k = 0; k < w.count; ++k) {
            detail::printPanelText(s_, TextSize::Medium, 8,
                                   static_cast<std::int16_t>(76 + 22 * static_cast<int>(k)),
                                   kColText, kColBg, w.line[k]);
        }
        cut = cut || !w.complete;
        const auto f = wrapWords<kSmallCols, 2>(fix != nullptr ? fix : "");
        for (std::size_t k = 0; k < f.count; ++k) {
            detail::printPanelText(s_, TextSize::Small, 8,
                                   static_cast<std::int16_t>(146 + 16 * static_cast<int>(k)),
                                   kColAmber, kColBg, f.line[k]);
        }
        cut = cut || !f.complete;
        detail::printPanelView(s_, TextSize::Small, 8, 186, kColGrey, kColBg, footer);
        detail::printPanelView(s_, TextSize::Small, 8, 202, kColGrey, kColBg,
                               cut ? screen_text::kMoreOnSerial : screen_text::kDiagnosisTool);
    }

    /// The loop's own "not driver control" screen. `bits` is the competition status in words
    /// ("status bits 0x.. field=.. disabled=.. auton=.."), one SMALL line — and, when it did not
    /// fit that line, a last line saying the full text is on the serial log.
    void paintNotDriverControl(const char* bits) {
        valid_ = false;
        eraseContent();
        paintHeader(PanelTone::Red, screen_text::kHeaderStopped);
        detail::printPanelView(s_, TextSize::Large, 8, 34, kColAmber, kColBg,
                               screen_text::kNotDriverControl);
        detail::printPanelView(s_, TextSize::Small, 8, 80, kColText, kColBg, screen_text::kNdc1);
        detail::printPanelView(s_, TextSize::Small, 8, 96, kColText, kColBg, screen_text::kNdc2);
        detail::printPanelText(s_, TextSize::Small, 8, 120, kColGrey, kColBg, bits);
        detail::printPanelView(s_, TextSize::Small, 8, 146, kColText, kColBg, screen_text::kNdc3);
        detail::printPanelView(s_, TextSize::Small, 8, 162, kColText, kColBg, screen_text::kNdc4);
        detail::printPanelView(s_, TextSize::Small, 8, 188, kColGrey, kColBg, screen_text::kNdc5);
        if (detail::columnsOf(bits) > kSmallCols) {
            detail::printPanelView(s_, TextSize::Small, 8, 204, kColGrey, kColBg,
                                   screen_text::kMoreOnSerial);
        }
    }

    /// The competition-state screen for disabled(), competition_initialize() and autonomous(): the
    /// whole screen erased to black, `state` LARGE in amber, the rule, `why` wrapped in SMALL (two
    /// lines at most), `bits` (the status in words), and what to do — and, when `state`, `why` or
    /// `bits` did not fit, a last line saying the full text is on the serial log. Pass
    /// screen_text's words (kStateName*, kStateWhy*): those are checked at compile time to fit.
    /// Its colours are those of robot two's other programs' state screen
    /// (src/common/state_screens.cpp) and its layout follows that screen's, with two changes: `why`
    /// wraps onto two lines (y 100 and 116; that screen prints one, uncut, at 100) and the status
    /// bits move down to y 136 (from 130). Its words are re-cut to the budgets, which three of that
    /// screen's SMALL lines (53, 53 and 54 characters) and its 23-character LARGE name for
    /// competition_initialize() exceed. It paints no header, so a build-time text is not needed.
    void paintStateScreen(const char* state, const char* why, const char* bits) {
        using detail::kStateGround;
        using detail::kStateText;
        valid_ = false;
        s_.eraseAll(kStateGround);
        detail::printPanelText(s_, TextSize::Large, 10, 30, detail::kStateWord, kStateGround,
                               state);
        detail::printPanelView(s_, TextSize::Small, 10, 80, kStateText, kStateGround,
                               screen_text::kState1);
        const auto w = wrapWords<kSmallCols, detail::kStateWhyLines>(why != nullptr ? why : "");
        for (std::size_t k = 0; k < w.count; ++k) {
            detail::printPanelText(s_, TextSize::Small, 10,
                                   static_cast<std::int16_t>(100 + 16 * static_cast<int>(k)),
                                   kStateText, kStateGround, w.line[k]);
        }
        detail::printPanelText(s_, TextSize::Small, 10, 136, kStateText, kStateGround, bits);
        detail::printPanelView(s_, TextSize::Small, 10, 160, kStateText, kStateGround,
                               screen_text::kState3);
        detail::printPanelView(s_, TextSize::Small, 10, 176, kStateText, kStateGround,
                               screen_text::kState4);
        detail::printPanelView(s_, TextSize::Small, 10, 192, kStateText, kStateGround,
                               screen_text::kState5);
        const bool cut = detail::columnsOf(state) > kLargeCols || !w.complete
                         || detail::columnsOf(bits) > kSmallCols;
        if (cut) {
            detail::printPanelView(s_, TextSize::Small, 10, 210, kColGrey, kStateGround,
                                   screen_text::kMoreOnSerial);
        }
    }

private:
    [[nodiscard]] static bool sameButton(const PanelButton& a, const PanelButton& b) {
        return a.id == b.id && a.rect == b.rect && a.armed == b.armed
               && std::strcmp(a.label, b.label) == 0;
    }
    [[nodiscard]] static bool samePlace(const PanelLine& a, const PanelLine& b) {
        return a.x == b.x && a.y == b.y && a.size == b.size;
    }
    /// Fill the header bar with `tone`'s colour and write `word` and the build time on it. The
    /// word is cut at kHeaderCols — not at MEDIUM's wider budget — so it can never reach the build
    /// time (the layout static_assert above holds for kHeaderCols characters only).
    void paintHeader(PanelTone tone, std::string_view word) {
        const std::uint32_t bar = detail::panelBarColour(tone);
        const std::uint32_t ink = detail::panelBarTextColour(tone);
        const PanelRect r = kHeaderBar;
        s_.fillRect(r.x0, r.y0, r.x1, r.y1, bar);
        detail::printPanelView(s_, TextSize::Medium, detail::kPainterHeaderWordX,
                               detail::kPainterHeaderWordY, ink, bar, word, kHeaderCols);
        s_.print(hal::ScreenTextSize::Small, detail::kPainterBuildX, detail::kPainterBuildY, ink,
                 bar, buildTime_, kBuildTimeCols);
    }
    /// Erase everything below the header bar, down to the last usable row.
    void eraseContent() {
        s_.eraseRect(0, static_cast<std::int16_t>(kHeaderBar.y1 + 1),
                     static_cast<std::int16_t>(kPanelWidth - 1), kPanelLastRow, kColBg);
    }
    /// One button: its fill says what it is (DONE green, a question or RUN AGAIN blue, ABORT dark
    /// red, a greyed button — still arming — dark grey), a light edge drawn twice, and its label
    /// centred at the width the MEDIUM budget implies.
    void paintButton(const PanelButton& b) {
        std::uint32_t fill = kColBlue;
        std::uint32_t edge = kColEdge;
        std::uint32_t label = kColText;
        if (!b.armed) {
            fill = kColGreyFill;
            edge = kColGrey;
            label = kColGrey;
        } else if (b.id == PanelButtonId::Abort) {
            fill = kColAbortFill;
            edge = kColRed;
        } else if (b.id == PanelButtonId::Done) {
            fill = kColGreen;
            label = kColBg;
        }
        const PanelRect r = b.rect;
        s_.fillRect(r.x0, r.y0, r.x1, r.y1, fill);
        s_.outlineRect(r.x0, r.y0, r.x1, r.y1, edge);
        s_.outlineRect(static_cast<std::int16_t>(r.x0 + 1), static_cast<std::int16_t>(r.y0 + 1),
                       static_cast<std::int16_t>(r.x1 - 1), static_cast<std::int16_t>(r.y1 - 1),
                       edge);
        constexpr int kCharW = 472 / static_cast<int>(kMediumCols);
        const int width = r.x1 - r.x0 + 1;
        const int height = r.y1 - r.y0 + 1;
        const int textW = static_cast<int>(std::strlen(b.label)) * kCharW;
        const int x = r.x0 + (width > textW + 8 ? (width - textW) / 2 : 4);
        const int y = r.y0 + (height - kMediumHeight) / 2;
        detail::printPanelText(s_, TextSize::Medium, static_cast<std::int16_t>(x),
                               static_cast<std::int16_t>(y), label, fill, b.label);
    }
    /// Where a button was, back to the ground.
    void eraseButton(const PanelButton& b) {
        s_.eraseRect(b.rect.x0, b.rect.y0, b.rect.x1, b.rect.y1, kColBg);
    }
    /// Record `v` as what is now on the screen.
    void adopt(const PanelView& v) {
        screen_ = v.screen;
        headerTone_ = v.headerTone;
        headerWord_ = v.headerWord;
        lineCount_ = v.lineCount;
        for (std::size_t i = 0; i < v.lineCount; ++i) {
            lines_[i] = v.lines[i];
        }
        buttonCount_ = v.buttonCount;
        for (std::size_t b = 0; b < v.buttonCount; ++b) {
            buttons_[b] = v.buttons[b];
        }
    }

    hal::IBrainScreen& s_;
    const char* buildTime_;
    bool valid_ = false;
    PanelScreen screen_ = PanelScreen::NotStarted;
    PanelTone headerTone_ = PanelTone::Blue;
    const char* headerWord_ = "";  // the panel's words are string literals: the pointer stays valid
    PanelLine lines_[kPanelMaxLines]{};
    double lineAt_[kPanelMaxLines]{};
    std::uint8_t lineCount_ = 0;
    PanelButton buttons_[kPanelMaxButtons]{};
    std::uint8_t buttonCount_ = 0;
};

}  // namespace shulib::calibration
