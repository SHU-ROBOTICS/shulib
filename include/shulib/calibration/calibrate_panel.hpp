#pragma once
//
// calibrate_panel — the calibration screen as a PURE MODEL (chunk R3d Part 2F-1, 2026-09-24).
//
// ── Why this exists ─────────────────────────────────────────────────────────────────
// The first night the calibrator ran on robot two, the team lead's report on its screen was
// "I CANT READ IT AND THE BUTTONS DONT WORK I DONT KNOW AFTER I PUSH IT WHAT IM SUPPOSED TO DO".
// The capture showed every touch reaching the calibrator — the buttons worked; the SCREEN did
// not: the instruction was cut mid-sentence, the one number that mattered was small and grey,
// DONE was offered with nothing pushed (two of five runs ended that way), and FRONT FIRST
// appeared exactly where DONE had just been, so a reflexive second tap would silently answer
// the next question. None of that was testable, because all of it lived in a PROS-bound
// program. This header is the part of the screen that decides WHAT it says and WHICH touches
// mean something — so those decisions are unit-tested on the host rather than eyeballed on the
// robot. The program (src/programs/calibrate_program.cpp, Part 2F-2) only RENDERS a PanelView.
//
// ── What it decides ─────────────────────────────────────────────────────────────────
//  * viewFor(): per state, a header colour and word, an instruction of at most two LARGE lines,
//    one big live number, a NEXT line, and the buttons that are LIVE, each with its rectangle.
//  * The layout rules that make a mis-tap harmless, checked at COMPILE time on the constants and
//    at test time on every reachable view:
//      - ABORT sits in the SAME rectangle in every state that shows it; in the two POWERED
//        phases it grows to the whole button row, which still contains that rectangle.
//      - DONE is drawn only when DrivetrainCalibrator::pushReady() says so.
//      - Questions (FRONT / BACK) and RUN AGAIN are boxes in the MIDDLE of the screen, never in
//        the button row — so no button of a new layout lies under the button whose touch
//        caused the change (the double-tap hazard).
//      - After EVERY layout change the new non-ABORT buttons are drawn greyed and dead for
//        kArmingDelaySeconds; ABORT is never delayed.
//  * hitTest(): padded boxes; a greyed button never hits.
//  * TouchGate: the release edge only, error reads ignored (and never replayed), the arming
//    clock, and a re-check against the calibrator before any answer is sent.
//  * The STOPPED wording per Refusal code — a LARGE headline, a WHY line and a DO line — so the
//    screen explains a stop from a CODE, never by parsing prose.
//  * wrapWords(): breaks text only at spaces — it never splits a word or a number, and it never
//    drops text silently (it says when the text did not fit).
//  * (CAL1 Part A2a, 2026-10-18 — ADDITIONS ONLY: every existing call paints today's screen, field
//    for field) the standard Calibrate's screens (calibration/calibrate_program.hpp): FrontRule
//    ByPush (four steps, PUSH FRONT FIRST, the front rule — never the WHICH END view); the two HELD
//    screens, FRONT FLIPPED and MECHANISM PORT TURNED (SaveOutcome::Held, FinishFacts::hold; the
//    second once per declared port that turned, FinishFacts::holdPort, its boxes re-greyed each
//    time), whose left box is today's FRONT FIRST id relabelled and read as
//    TouchAction::SaveHeld; the SAVED header after USE NEW; and the finished screen's NOTE row. No
//    PanelButtonId is added — today's program and tests switch over them; CAL1 Part B renames and
//    removes them.
//
// ── What it does NOT decide ─────────────────────────────────────────────────────────
// Pixels a font lands on, colours' RGB values, the build stamp, redraw-only-what-changed, the
// SD card, serial output, RUN AGAIN's rebuild of the calibrator (a new DrivetrainCalibrator —
// drivetrain_calibrator.hpp says how), and the competition-state screens: all the program's.
//
// ── EVERY TEXT BUDGET IS AN ESTIMATE ────────────────────────────────────────────────
// The V5's three text sizes have never been measured in this project (Bench Tests has a SCREEN
// RULER for it). The budgets below — LARGE 18, MEDIUM 36, SMALL 50 characters per 472-pixel line
// — are conservative guesses, each ONE named constant, and every literal in this file is checked
// against them by static_assert: when the ruler is read, changing a constant is all it takes, and
// any text that no longer fits fails the build naming itself. The line heights and positions are
// estimates of the same kind.
//
// ── ASCII ONLY, AND NEVER A PERCENT SIGN ────────────────────────────────────────────
// PROS 4.2.2 draws text through VEXos's own display routines, whose fonts carry ASCII only, and
// it hands the already-formatted text to VEXos AS A FORMAT STRING — a '%' on screen is re-read as
// a conversion with no argument behind it. Every literal here is checked for both at compile
// time; every formatted line is checked by the host tests on every reachable view.
//
// PROS-free, allocation-free, no exceptions thrown; pure functions of the calibrator's public
// queries plus the time since the layout changed. NOTHING HERE HAS RUN ON A ROBOT.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string_view>

#include "shulib/calibration/drivetrain_calibrator.hpp"

namespace shulib::calibration {

// ── budgets (ESTIMATES until the SCREEN RULER is read — one constant each) ─────────

/// Characters per line in the LARGE size. ESTIMATE, never measured on a V5 (A4: HA-136).
inline constexpr std::size_t kLargeCols = 18;
/// Characters per line in the MEDIUM size. ESTIMATE (A4: HA-136).
inline constexpr std::size_t kMediumCols = 36;
/// Characters per line in the SMALL size. ESTIMATE (A4: HA-136).
inline constexpr std::size_t kSmallCols = 50;
/// The header word's own budget, MEDIUM: it shares the header bar with the build time the program
/// prints at the right. ESTIMATE (A4: HA-136).
inline constexpr std::size_t kHeaderCols = 24;
/// The note beside RUN AGAIN on the outcome screens, MEDIUM: the left half of the line only.
/// ESTIMATE (A4: HA-136).
inline constexpr std::size_t kNoteCols = 18;

/// The height a line of each size is laid out for, pixels. ESTIMATES, like the budgets (A4: HA-136).
inline constexpr std::int16_t kSmallHeight = 16;
/// See kSmallHeight.
inline constexpr std::int16_t kMediumHeight = 20;
/// See kSmallHeight.
inline constexpr std::int16_t kLargeHeight = 34;

// ── geometry (480 x 240 usable; every pixel at y < 236) ────────────────────────────

/// The screen's usable width, pixels.
inline constexpr std::int16_t kPanelWidth = 480;
/// The lowest row anything may be drawn on: VEXos keeps the rows below.
inline constexpr std::int16_t kPanelLastRow = 235;
/// How far a touch may land outside a button, left/right, and still hit it — a resistive screen's
/// release point wanders. Kept under half of every gap between two buttons, so a pad never makes
/// two buttons overlap. ESTIMATE (A4: HA-136).
inline constexpr std::int16_t kHitPadX = 4;
/// How far a touch may land above or below a button and still hit it (the robot's capture has a
/// release exactly on a button's top edge). ESTIMATE (A4: HA-136).
inline constexpr std::int16_t kHitPadY = 10;
/// How long the new buttons stay greyed and dead after a layout change: long enough that a
/// reflexive second tap lands on nothing, short enough not to feel broken. ESTIMATE (A4: HA-136).
inline constexpr double kArmingDelaySeconds = 0.5;

/// A rectangle in screen pixels with INCLUSIVE corners — PROS's own drawing convention.
struct PanelRect {
    std::int16_t x0 = 0;  ///< left column
    std::int16_t y0 = 0;  ///< top row
    std::int16_t x1 = 0;  ///< right column (inclusive)
    std::int16_t y1 = 0;  ///< bottom row (inclusive)
    /// Equal corners.
    friend constexpr bool operator==(const PanelRect&, const PanelRect&) = default;
};

/// The rectangle a touch must land in to hit a button drawn at `r`: padded kHitPadX sideways and
/// kHitPadY up and down.
[[nodiscard]] constexpr PanelRect hitBox(PanelRect r) noexcept {
    return PanelRect{static_cast<std::int16_t>(r.x0 - kHitPadX),
                     static_cast<std::int16_t>(r.y0 - kHitPadY),
                     static_cast<std::int16_t>(r.x1 + kHitPadX),
                     static_cast<std::int16_t>(r.y1 + kHitPadY)};
}

/// True when (x, y) lies inside `r`, edges included.
[[nodiscard]] constexpr bool contains(PanelRect r, int x, int y) noexcept {
    return x >= r.x0 && x <= r.x1 && y >= r.y0 && y <= r.y1;
}

/// True when `a` and `b` share at least one pixel.
[[nodiscard]] constexpr bool overlaps(PanelRect a, PanelRect b) noexcept {
    return a.x0 <= b.x1 && b.x0 <= a.x1 && a.y0 <= b.y1 && b.y0 <= a.y1;
}

/// True when `inner` lies wholly inside `outer`.
[[nodiscard]] constexpr bool within(PanelRect inner, PanelRect outer) noexcept {
    return inner.x0 >= outer.x0 && inner.x1 <= outer.x1 && inner.y0 >= outer.y0
           && inner.y1 <= outer.y1;
}

/// The header bar: its colour is the state (blue unpowered, amber powered, green saved, red
/// stopped).
inline constexpr PanelRect kHeaderBar{0, 0, 479, 26};
/// DONE's place, the primary slot at the bottom left.
inline constexpr PanelRect kPrimarySlot{6, 180, 300, 234};
/// ABORT's place in EVERY state that shows it — the bottom right. Never moves under a thumb.
inline constexpr PanelRect kAbortSlot{330, 180, 474, 234};
/// ABORT in the two POWERED phases: the whole button row, still containing kAbortSlot.
inline constexpr PanelRect kAbortRow{6, 180, 474, 234};
/// FRONT FIRST: a big box in the MIDDLE, never in the button row.
inline constexpr PanelRect kFrontBox{6, 72, 234, 158};
/// BACK FIRST: the same, on the right.
inline constexpr PanelRect kBackBox{246, 72, 474, 158};
/// RUN AGAIN on the outcome screens: middle right, clear of the button row.
inline constexpr PanelRect kRunAgainBox{246, 112, 474, 158};

/// The three text sizes PROS offers.
enum class TextSize : std::uint8_t {
    Small,   ///< kSmallCols per line
    Medium,  ///< kMediumCols per line
    Large,   ///< kLargeCols per line
};

/// The budget, in characters, of one line of `size`.
[[nodiscard]] constexpr std::size_t budgetFor(TextSize size) noexcept {
    return size == TextSize::Large ? kLargeCols : size == TextSize::Medium ? kMediumCols : kSmallCols;
}

/// The height a line of `size` is laid out for, pixels (an ESTIMATE).
[[nodiscard]] constexpr std::int16_t heightFor(TextSize size) noexcept {
    return size == TextSize::Large ? kLargeHeight
           : size == TextSize::Medium ? kMediumHeight
                                      : kSmallHeight;
}

/// A colour by MEANING; the program maps each to RGB.
enum class PanelTone : std::uint8_t {
    Blue,   ///< waiting / unpowered (the header)
    Amber,  ///< powered — stand clear; or "not yet" on a number
    Green,  ///< ready / saved / past the floor
    Red,    ///< stopped / not saved / out of bounds
    Grey,   ///< a greyed button, or a quiet line
    White,  ///< ordinary text
};

/// What a screen is showing.
enum class PanelScreen : std::uint8_t {
    NotStarted,  ///< the calibrator is Idle
    ImuWait,     ///< hands off while the IMU starts
    Push,        ///< push it forward along the tape
    WhichEnd,    ///< FRONT or BACK first?
    Pulses,      ///< powered: one motor at a time (AMBER)
    Spin,        ///< powered: the spin (AMBER)
    Saved,       ///< the run finished (SAVED / NOT SAVED / MEASURED)
    Stopped,     ///< the run was refused or aborted
    Held,        ///< (CAL1) the save WAITS for the operator: SAVE (or USE NEW) or RUN AGAIN
};

/// Whether the program managed to write the result — the panel cannot know, so it is told.
enum class SaveOutcome : std::uint8_t {
    NotAttempted,  ///< not written (yet): the header says MEASURED
    Saved,         ///< written, flushed, read back and compared: SAVED
    NotSaved,      ///< the write or the read-back failed: NOT SAVED, copy the numbers
    Held,          ///< (CAL1) not written YET: the operator chooses (HoldReason says why)
};

/// Which front a SAVED record carries when the operator was asked (CAL1) — the header says it.
enum class FrontChoice : std::uint8_t {
    /// not asked: no FRONT FLIPPED hold — the same front, the card's first record, or a front that
    /// could not be checked (files that do not decode, or CANNOT TELL)
    None,
    New,   ///< USE NEW: the end pushed first this time is the front, against the card's record
};

/// Why a save is HELD (CAL1): the screen the operator answers. One question per screen, in this
/// order: FRONT FLIPPED, then MECHANISM PORT TURNED — one screen per declared port that turned
/// (FinishFacts::holdPort).
enum class HoldReason : std::uint8_t {
    None,             ///< not held
    FrontFlipped,     ///< the card's record calls the end that led this push its BACK
    MechanismTurned,  ///< a DECLARED mechanism motor turned with the push, as a drive motor would
};

/// What the program tells the finished and held screens (CAL1): the panel cannot know these. The
/// default is today's finished screen exactly — no hold, no NOTE, no front choice.
struct FinishFacts {
    FrontChoice front = FrontChoice::None;  ///< New: the SAVED header says the front is new
    HoldReason hold = HoldReason::None;     ///< which held screen — read only with SaveOutcome::Held
    /// MechanismTurned: the declared port this screen asks about — one screen per port that turned.
    /// A different port is a NEW question: TouchGate::view() restarts the arming clock (re-greys
    /// the boxes), as it does when the reason changes. 0 = none.
    int holdPort = 0;
    const char* holdLine1 = nullptr;  ///< MechanismTurned, MEDIUM: "port 6: 1692 deg, drive 1690 deg"
    const char* holdLine2 = nullptr;  ///< MechanismTurned, SMALL: what the card's record says of it
    const char* note = nullptr;       ///< the finished screen's one amber NOTE row, or null
};

/// How the front is decided (CAL1, Parts A — Part B removes AskedAfterPush and this parameter).
enum class FrontRule : std::uint8_t {
    AskedAfterPush,  ///< today's: the WHICH END question after DONE
    ByPush,          ///< the standard Calibrate: the end that leads the push IS the front
};

/// Which button.
enum class PanelButtonId : std::uint8_t {
    Done,      ///< "I have finished pushing" — drawn only when pushReady()
    Front,     ///< FRONT FIRST
    Back,      ///< BACK FIRST
    Abort,     ///< stop now — always armed
    RunAgain,  ///< on the outcome screens: the program rebuilds the calibrator
};

/// One button of a view.
struct PanelButton {
    /// Which button.
    PanelButtonId id = PanelButtonId::Abort;
    /// Where it is drawn (its hit box is hitBox(rect)).
    PanelRect rect{};
    /// Its label, MEDIUM, ASCII.
    const char* label = "";
    /// False while the arming delay runs: drawn GREYED and never hit.
    bool armed = true;
};

/// The longest line a view carries, plus its terminator.
inline constexpr std::size_t kPanelLineCap = kSmallCols + 1;
/// The most text lines one view carries.
inline constexpr std::size_t kPanelMaxLines = 10;
/// The most buttons one view carries.
inline constexpr std::size_t kPanelMaxButtons = 3;

/// One line of text at a fixed place and size.
struct PanelLine {
    /// Left edge, pixels.
    std::int16_t x = 0;
    /// Top edge, pixels.
    std::int16_t y = 0;
    /// Its size; the text fits budgetFor(size) (the host tests check every reachable line).
    TextSize size = TextSize::Medium;
    /// Its colour, by meaning.
    PanelTone tone = PanelTone::White;
    /// The text, NUL-terminated, ASCII, no '%'.
    char text[kPanelLineCap]{};
};

/// Everything one frame of the screen says, and which touches mean something.
struct PanelView {
    /// What the screen is showing.
    PanelScreen screen = PanelScreen::NotStarted;
    /// The header bar's colour — the state at a glance.
    PanelTone headerTone = PanelTone::Blue;
    /// The step word, left of the header bar, MEDIUM, at most kHeaderCols.
    const char* headerWord = "";
    /// The text lines, in drawing order.
    PanelLine lines[kPanelMaxLines]{};
    /// How many entries of `lines` are real.
    std::uint8_t lineCount = 0;
    /// The buttons (live or greyed), ABORT always last when present.
    PanelButton buttons[kPanelMaxButtons]{};
    /// How many entries of `buttons` are real.
    std::uint8_t buttonCount = 0;
};

/// The STOPPED screen's words for one refusal code.
struct StoppedWords {
    /// LARGE, at most kLargeCols.
    const char* headline;
    /// MEDIUM, at most kMediumCols: what happened.
    const char* why;
    /// MEDIUM, at most kMediumCols: what to do.
    const char* doLine;
};

/// The STOPPED wording for `code` — keyed on the CODE, never on the reason's prose, which the
/// screen shows below it in SMALL for the numbers. Every entry is checked at compile time.
[[nodiscard]] constexpr StoppedWords stoppedWords(Refusal code) noexcept {
    switch (code) {
        case Refusal::None: return {"STOPPED", "no reason was recorded", "RUN AGAIN"};
        case Refusal::Aborted:
            return {"YOU STOPPED IT", "the run was stopped on purpose", "RUN AGAIN when it is safe"};
        case Refusal::ImuNotReady:
            return {"IMU NOT READY", "the IMU never finished starting", "check its cable, RUN AGAIN"};
        case Refusal::PushTooFew:
            return {"PUSH NOT SEEN", "fewer than 2 motors turned", "push it along the tape, then DONE"};
        case Refusal::PushCurved:
            return {"PUSH CURVED", "it turned while it was pushed", "push it again, STRAIGHT on the tape"};
        case Refusal::PushTooShort:
            return {"PUSH TOO SHORT", "the wheels turned under 1 turn", "push it the full length of the tape"};
        case Refusal::RestTimeout:
            return {"NOT SETTLING", "something kept moving before a pulse", "hands off it, then RUN AGAIN"};
        case Refusal::NonDriveMoved:
            return {"WRONG MOTOR MOVED", "a motor off the drive turned", "check the port shown, RUN AGAIN"};
        case Refusal::PulseContradiction:
            return {"SIGN MISMATCH", "the push and the pulse disagree", "check that port's cable and gears"};
        case Refusal::PulseRolled:
            return {"IT ROLLED", "it moved but did not turn", "on the FLOOR? then report the port"};
        case Refusal::PulseStill:
            return {"NOTHING MOVED", "even the top pulse moved nothing", "on the FLOOR? a wheel stuck?"};
        case Refusal::OneSided:
            return {"ONE SIDE ONLY", "every motor turned it one way", "check the wiring, RUN AGAIN"};
        case Refusal::SpinWrongWay:
            return {"SPUN WRONG WAY", "the sides came out swapped", "RUN AGAIN; if it repeats, report"};
        case Refusal::SpinNoStart:
            return {"COULD NOT SPIN", "the drive could not turn the robot", "clear space round it, RUN AGAIN"};
        case Refusal::SpinTimeout:
            return {"SPIN TOO SLOW", "it did not finish its turns in time", "clear space round it, RUN AGAIN"};
        case Refusal::SpinFight:
            return {"MOTOR FIGHTING", "one motor fought its side", "check the port shown before driving"};
        case Refusal::SpinWidthInvalid:
            return {"BAD SPIN", "the spin gave an impossible width", "RUN AGAIN; if it repeats, report"};
        case Refusal::SpinDisagree:
            return {"SPINS DISAGREE", "the two spins gave different widths", "clear the floor, RUN AGAIN"};
        case Refusal::MissedTick:
            return {"PROGRAM STALLED", "the program missed its timing", "RUN AGAIN; if it repeats, report"};
        case Refusal::OutOfOrder:
            return {"WRONG BUTTON", "an answer came at the wrong time", "RUN AGAIN and follow the screen"};
        case Refusal::InvalidDescription:
            return {"RESULT INVALID", "the numbers do not fit together", "RUN AGAIN; if it repeats, report"};
    }
    return {"STOPPED", "an unknown refusal", "RUN AGAIN"};
}

/// True when `s` may be drawn in a line of `cols` characters: printable ASCII only, no '%', and
/// no longer than `cols`. The compile-time gate for every literal below.
[[nodiscard]] constexpr bool screenText(std::string_view s, std::size_t cols) noexcept {
    if (s.size() > cols) {
        return false;
    }
    for (const char c : s) {
        const auto u = static_cast<unsigned char>(c);
        if (u < 0x20 || u > 0x7E || c == '%') {
            return false;
        }
    }
    return true;
}

/// The words the screens use, each a named constant so the compile-time checks below can see it.
/// A new literal belongs HERE, with its budget in the table at the end of this namespace.
namespace panel_text {
/// Header words (kHeaderCols).
inline constexpr std::string_view kHeaderIdle = "CALIBRATE";
/// See kHeaderIdle.
inline constexpr std::string_view kHeaderImu = "1/5 IMU";
/// See kHeaderIdle.
inline constexpr std::string_view kHeaderPush = "2/5 PUSH";
/// See kHeaderIdle.
inline constexpr std::string_view kHeaderWhichEnd = "3/5 WHICH END";
/// See kHeaderIdle.
inline constexpr std::string_view kHeaderPulses = "4/5 PULSES";
/// See kHeaderIdle.
inline constexpr std::string_view kHeaderSpin = "5/5 SPIN";
/// See kHeaderIdle.
inline constexpr std::string_view kHeaderSaved = "SAVED";
/// See kHeaderIdle.
inline constexpr std::string_view kHeaderNotSaved = "NOT SAVED";
/// See kHeaderIdle.
inline constexpr std::string_view kHeaderMeasured = "MEASURED";
/// See kHeaderIdle.
inline constexpr std::string_view kHeaderStopped = "STOPPED -- MOTORS OFF";
/// Instructions (kLargeCols).
inline constexpr std::string_view kNotStarted = "NOT STARTED";
/// See kNotStarted.
inline constexpr std::string_view kHandsOff = "HANDS OFF";
/// See kNotStarted.
inline constexpr std::string_view kImuWarming = "IMU WARMING UP";
/// See kNotStarted.
inline constexpr std::string_view kPushForward = "PUSH IT FORWARD";
/// See kNotStarted.
inline constexpr std::string_view kWhichEnd = "WHICH END FIRST?";
/// See kNotStarted.
inline constexpr std::string_view kStandClear = "STAND CLEAR";
/// See kNotStarted.
inline constexpr std::string_view kTwitching = "TWITCHING MOTORS";
/// See kNotStarted.
inline constexpr std::string_view kSpinning = "SPINNING";
/// See kNotStarted.
inline constexpr std::string_view kSettling = "SETTLING";
/// NEXT / detail lines (kMediumCols).
inline constexpr std::string_view kNextDoneWhenMoved = "DONE shows once the motors turn";
/// See kNextDoneWhenMoved.
inline constexpr std::string_view kNextLetGo = "let go -- DONE shows when it stops";
/// See kNextDoneWhenMoved.
inline constexpr std::string_view kNextTouchDone = "touch DONE -- then say which end";
/// See kNextDoneWhenMoved.
inline constexpr std::string_view kNextTwitch = "NEXT: motors twitch -- stand clear";
/// See kNextDoneWhenMoved.
inline constexpr std::string_view kNextSpins = "NEXT: it spins -- stand clear";
/// See kNextDoneWhenMoved.
inline constexpr std::string_view kNextSaves = "NEXT: it saves -- stay clear";
/// See kNextDoneWhenMoved.
inline constexpr std::string_view kSettlingHandsOff = "settling -- hands off";
/// See kNextDoneWhenMoved.
inline constexpr std::string_view kSpinAgain = "hands off -- it spins again";
/// See kNextDoneWhenMoved.
inline constexpr std::string_view kSpinFirst = "hands off -- it spins next";
/// Outcome lines (kSmallCols).
inline constexpr std::string_view kCopyThenRunAgain = "NEXT: copy these, then RUN AGAIN to check";
/// See kCopyThenRunAgain.
inline constexpr std::string_view kCopyNow = "NOT SAVED -- copy these numbers NOW";
/// See kCopyThenRunAgain.
inline constexpr std::string_view kCopyThese = "copy these numbers";
/// See kCopyThenRunAgain.
inline constexpr std::string_view kFullReasonOnSerial = "the full reason is on the serial log";
/// Button labels (MEDIUM; a label also fits its box: 12 characters).
inline constexpr std::string_view kLabelDone = "DONE";
/// See kLabelDone.
inline constexpr std::string_view kLabelFront = "FRONT FIRST";
/// See kLabelDone.
inline constexpr std::string_view kLabelBack = "BACK FIRST";
/// See kLabelDone.
inline constexpr std::string_view kLabelAbort = "ABORT";
/// See kLabelDone.
inline constexpr std::string_view kLabelRunAgain = "RUN AGAIN";
/// The standard Calibrate's step headers (FrontRule::ByPush, CAL1): four steps, no WHICH END.
inline constexpr std::string_view kHeaderImu4 = "1/4 IMU";
/// See kHeaderImu4.
inline constexpr std::string_view kHeaderPush4 = "2/4 PUSH";
/// See kHeaderImu4.
inline constexpr std::string_view kHeaderPulses4 = "3/4 PULSES";
/// See kHeaderImu4.
inline constexpr std::string_view kHeaderSpin4 = "4/4 SPIN";
/// Header word: the FRONT FLIPPED held screen (CAL1).
inline constexpr std::string_view kHeaderFlipped = "FRONT FLIPPED";
/// Header word: the held screen when a declared mechanism motor turned with the push (CAL1).
inline constexpr std::string_view kHeaderMechTurned = "MECHANISM PORT TURNED";
/// Header word: SAVED after USE NEW (CAL1).
inline constexpr std::string_view kHeaderSavedNew = "SAVED -- NEW FRONT";
/// LARGE: the push instruction under FrontRule::ByPush.
inline constexpr std::string_view kPushFrontFirst = "PUSH FRONT FIRST";
/// LARGE: every held screen's headline.
inline constexpr std::string_view kNothingSavedYet = "NOTHING SAVED YET";
/// MEDIUM: the NEXT line once DONE shows, under ByPush.
inline constexpr std::string_view kNextTouchDoneClear = "touch DONE -- then stand clear";
/// MEDIUM: the rule, shown while the IMU warms up, under ByPush.
inline constexpr std::string_view kFrontRule = "the end that leads the push is FRONT";
/// MEDIUM: FRONT FLIPPED — what the records say (never what the operator did: a LEFT/RIGHT cable
/// swap gives the same record as the other end leading).
inline constexpr std::string_view kFlipWhat = "the card calls the end you led BACK";
/// SMALL: FRONT FLIPPED — RUN AGAIN's case.
inline constexpr std::string_view kFlipAgain = "RUN AGAIN: you led with the BACK by mistake";
/// SMALL: FRONT FLIPPED — USE NEW's case (a cable swap between the sides looks exactly like a flip).
inline constexpr std::string_view kFlipNew = "USE NEW: you led with the FRONT (cables swapped?)";
/// SMALL: FRONT FLIPPED — the warning.
inline constexpr std::string_view kFlipWarn = "USE NEW after a wrong push reverses every auton";
/// SMALL: MECHANISM PORT TURNED — when the card's record does not drive with the port.
inline constexpr std::string_view kMechNotOnCard = "it is declared a mechanism: not in this record";
/// SMALL: MECHANISM PORT TURNED — SAVE's case.
inline constexpr std::string_view kMechSave = "SAVE: it is a mechanism the push turns -- keep it";
/// SMALL: MECHANISM PORT TURNED — RUN AGAIN's case.
inline constexpr std::string_view kMechAgain = "RUN AGAIN = save nothing: a drive motor? fix facts";
/// Label: the held screen's left box on FRONT FLIPPED (today's FRONT FIRST button, relabelled — no
/// new PanelButtonId before CAL1 Part B: today's program and tests switch over the ids).
inline constexpr std::string_view kLabelNew = "USE NEW";
/// Label: the held screen's left box on MECHANISM PORT TURNED.
inline constexpr std::string_view kLabelSave = "SAVE";
/// SMALL: the finished screen's NOTE after a first record (no record on the card).
inline constexpr std::string_view kNoteFirst = "FIRST RECORD -- the end you pushed first is FRONT";
/// SMALL: the NOTE when the front could not be checked (the card's record does not decode, or the
/// check could not tell).
inline constexpr std::string_view kNoteNotChecked = "FRONT NOT CHECKED -- see the log";
/// SMALL: the warning row's ending when the NOTE takes its second line (CAL1).
inline constexpr std::string_view kWarningCut = " ...";

/// One literal and the budget it must fit.
struct Checked {
    std::string_view text;  ///< the literal
    std::size_t cols;       ///< its line's budget
};

/// EVERY literal above with its budget — the list the compile-time check walks.
inline constexpr Checked kAll[] = {
    {kHeaderIdle, kHeaderCols},      {kHeaderImu, kHeaderCols},
    {kHeaderPush, kHeaderCols},      {kHeaderWhichEnd, kHeaderCols},
    {kHeaderPulses, kHeaderCols},    {kHeaderSpin, kHeaderCols},
    {kHeaderSaved, kHeaderCols},     {kHeaderNotSaved, kHeaderCols},
    {kHeaderMeasured, kHeaderCols},  {kHeaderStopped, kHeaderCols},
    {kNotStarted, kLargeCols},       {kHandsOff, kLargeCols},
    {kImuWarming, kLargeCols},       {kPushForward, kLargeCols},
    {kWhichEnd, kLargeCols},         {kStandClear, kLargeCols},
    {kTwitching, kLargeCols},        {kSpinning, kLargeCols},
    {kSettling, kLargeCols},         {kNextDoneWhenMoved, kMediumCols},
    {kNextLetGo, kMediumCols},       {kNextTouchDone, kMediumCols},
    {kNextTwitch, kMediumCols},      {kNextSpins, kMediumCols},
    {kNextSaves, kMediumCols},       {kSettlingHandsOff, kMediumCols},
    {kSpinAgain, kMediumCols},       {kSpinFirst, kMediumCols},
    {kCopyThenRunAgain, kSmallCols}, {kCopyNow, kSmallCols},
    {kCopyThese, kSmallCols},        {kFullReasonOnSerial, kSmallCols},
    {kLabelDone, 12},                {kLabelFront, 12},
    {kLabelBack, 12},                {kLabelAbort, 12},
    {kLabelRunAgain, 12},
    {kHeaderImu4, kHeaderCols},      {kHeaderPush4, kHeaderCols},
    {kHeaderPulses4, kHeaderCols},   {kHeaderSpin4, kHeaderCols},
    {kHeaderFlipped, kHeaderCols},   {kHeaderMechTurned, kHeaderCols},
    {kHeaderSavedNew, kHeaderCols},  {kPushFrontFirst, kLargeCols},
    {kNothingSavedYet, kLargeCols},  {kNextTouchDoneClear, kMediumCols},
    {kFrontRule, kMediumCols},       {kFlipWhat, kMediumCols},
    {kFlipAgain, kSmallCols},        {kFlipNew, kSmallCols},
    {kFlipWarn, kSmallCols},         {kMechNotOnCard, kSmallCols},
    {kMechSave, kSmallCols},         {kMechAgain, kSmallCols},
    {kLabelNew, 12},                 {kLabelSave, 12},
    {kNoteFirst, kSmallCols},        {kNoteNotChecked, kSmallCols},
    {kWarningCut, kSmallCols},
};
}  // namespace panel_text

/// True when every named literal and every STOPPED wording fits its budget and is ASCII with no
/// '%' — evaluated at compile time below, so a budget change that breaks one fails the build.
[[nodiscard]] constexpr bool panelLiteralsFit() noexcept {
    for (const panel_text::Checked& c : panel_text::kAll) {
        if (!screenText(c.text, c.cols)) {
            return false;
        }
    }
    for (int r = 0; r <= static_cast<int>(Refusal::InvalidDescription); ++r) {
        const StoppedWords w = stoppedWords(static_cast<Refusal>(r));
        if (!screenText(w.headline, kLargeCols) || !screenText(w.why, kMediumCols)
            || !screenText(w.doLine, kMediumCols)) {
            return false;
        }
    }
    return true;
}
static_assert(panelLiteralsFit(),
              "calibrate_panel: a literal does not fit its line budget, or is not ASCII, or "
              "contains a percent sign");

/// True when the fixed layout keeps its promises: everything on the screen above row 236; ABORT's
/// slot inside the powered row; and no middle box (FRONT, BACK, RUN AGAIN) whose HIT box reaches
/// the hit box of anything in the button row — so a second tap where DONE or ABORT was lands on
/// nothing in the next layout. FRONT and BACK do not touch each other.
[[nodiscard]] constexpr bool panelLayoutHolds() noexcept {
    const PanelRect all[] = {kHeaderBar, kPrimarySlot, kAbortSlot, kAbortRow,
                             kFrontBox,  kBackBox,     kRunAgainBox};
    for (const PanelRect& r : all) {
        if (r.x0 < 0 || r.y0 < 0 || r.x1 >= kPanelWidth || r.y1 > kPanelLastRow || r.x0 > r.x1
            || r.y0 > r.y1) {
            return false;
        }
    }
    if (!within(kAbortSlot, kAbortRow) || overlaps(hitBox(kPrimarySlot), hitBox(kAbortSlot))) {
        return false;
    }
    const PanelRect middle[] = {kFrontBox, kBackBox, kRunAgainBox};
    for (const PanelRect& m : middle) {
        if (overlaps(hitBox(m), hitBox(kAbortRow)) || overlaps(hitBox(m), hitBox(kPrimarySlot))) {
            return false;
        }
    }
    return !overlaps(hitBox(kFrontBox), hitBox(kBackBox));
}
static_assert(panelLayoutHolds(),
              "calibrate_panel: the fixed layout breaks a promise (off screen, ABORT moved out of "
              "its row, or a middle box within reach of the button row)");

/// Text broken into at most `Lines` lines of at most `Cols` characters — see wrapWords().
template <std::size_t Cols, std::size_t Lines>
struct Wrapped {
    /// The lines, each NUL-terminated.
    char line[Lines][Cols + 1]{};
    /// How many lines are used.
    std::size_t count = 0;
    /// False when the text did not fit: a word longer than `Cols` (it is never split), or more
    /// text than `Lines` lines hold (the lines hold what fitted, in order; the rest is NOT shown
    /// and this flag says so).
    bool complete = true;
};

/// Break `text` into lines of at most `Cols` characters, ONLY at spaces: a word — and therefore a
/// number like 1.37500 — is never split, runs of spaces collapse to one, and nothing is dropped
/// silently (Wrapped::complete). constexpr, so a literal's wrapping can be checked at compile
/// time. Allocation-free.
template <std::size_t Cols, std::size_t Lines>
[[nodiscard]] constexpr Wrapped<Cols, Lines> wrapWords(std::string_view text) noexcept {
    static_assert(Cols > 0 && Lines > 0, "wrapWords: at least one column and one line");
    Wrapped<Cols, Lines> w{};
    std::size_t line = 0;
    std::size_t len = 0;
    bool any = false;
    std::size_t i = 0;
    const std::size_t n = text.size();
    while (i < n) {
        while (i < n && text[i] == ' ') {
            ++i;
        }
        if (i >= n) {
            break;
        }
        std::size_t j = i;
        while (j < n && text[j] != ' ') {
            ++j;
        }
        const std::size_t wordLen = j - i;
        if (wordLen > Cols) {
            w.complete = false;  // never split a word: stop, and say the text did not fit
            break;
        }
        if (any && len + 1 + wordLen > Cols) {
            if (line + 1 >= Lines) {
                w.complete = false;  // out of lines: what fitted stays, the flag says the rest
                break;
            }
            ++line;
            len = 0;
        }
        if (len > 0) {
            w.line[line][len++] = ' ';
        }
        for (std::size_t k = 0; k < wordLen; ++k) {
            w.line[line][len++] = text[i + k];
        }
        w.line[line][len] = '\0';
        any = true;
        i = j;
    }
    w.count = any ? line + 1 : 0;
    return w;
}

namespace detail {

/// Append a line (the text is cut at kPanelLineCap − 1; the host tests check that no reachable
/// line comes near its size's budget, so a cut here is a test failure, never a silent one).
inline void addPanelLine(PanelView& v, std::int16_t x, std::int16_t y, TextSize size, PanelTone tone,
                    std::string_view text) {
    if (v.lineCount >= kPanelMaxLines) {
        return;
    }
    PanelLine& l = v.lines[v.lineCount++];
    l.x = x;
    l.y = y;
    l.size = size;
    l.tone = tone;
    std::size_t k = 0;
    for (; k < text.size() && k + 1 < kPanelLineCap; ++k) {
        l.text[k] = text[k];
    }
    l.text[k] = '\0';
}

/// Append a button. ABORT is always armed; every other button waits out the arming delay.
inline void addPanelButton(PanelView& v, PanelButtonId id, PanelRect rect, std::string_view label,
                      double secondsSinceLayoutChange) {
    if (v.buttonCount >= kPanelMaxButtons) {
        return;
    }
    v.buttons[v.buttonCount++] =
        PanelButton{id, rect, label.data(),
                    id == PanelButtonId::Abort || secondsSinceLayoutChange >= kArmingDelaySeconds};
}

// The line positions of the RUNNING screens (ESTIMATES, like the heights they assume).
inline constexpr std::int16_t kTextX = 8;
inline constexpr std::int16_t kInstr1Y = 30;
inline constexpr std::int16_t kInstr2Y = 66;
inline constexpr std::int16_t kLiveY = 102;
inline constexpr std::int16_t kDetailY = 138;
inline constexpr std::int16_t kNextY = 160;
// ... and of the OUTCOME screens.
inline constexpr std::int16_t kHeadlineY = 30;
inline constexpr std::int16_t kLine1Y = 70;
inline constexpr std::int16_t kLine2Y = 92;
inline constexpr std::int16_t kNoteY = 118;
inline constexpr std::int16_t kSmall1Y = 180;
inline constexpr std::int16_t kSmall2Y = 196;
inline constexpr std::int16_t kSmall3Y = 212;

/// The L / R / OUT chips as the pulses decide them: "L 11 12  R 16  OUT 18", or — when that would
/// not fit a MEDIUM line — the counts alone.
inline void panelChips(const DrivetrainCalibrator& cal, char* out, std::size_t cap) {
    const CalibrationReport& rep = cal.report();
    char buf[3 * hal::kMaxSmartPorts * 4 + 16] = "";
    std::size_t used = 0;
    const auto put = [&](const char* s) {
        const int k = std::snprintf(buf + used, sizeof buf - used, "%s", s);
        used += static_cast<std::size_t>(k > 0 ? k : 0);
        used = used < sizeof buf ? used : sizeof buf - 1;
    };
    const auto group = [&](const char* tag, int want, bool excludedGroup) {
        bool first = true;
        for (std::size_t i = 0; i < rep.portCount; ++i) {
            const PortObservation& o = rep.ports[i];
            const bool member = excludedGroup ? o.excluded : (!o.excluded && o.side == want);
            if (!member) {
                continue;
            }
            if (first) {
                put(used > 0 ? "  " : "");
                put(tag);
                first = false;
            }
            char p[8];
            std::snprintf(p, sizeof p, " %d", static_cast<int>(o.port));
            put(p);
        }
    };
    group("L", -1, false);
    group("R", +1, false);
    group("OUT", 0, true);
    if (used <= kMediumCols) {
        std::snprintf(out, cap, "%s", buf);
        return;
    }
    std::snprintf(out, cap, "L %u  R %u  OUT %u", static_cast<unsigned>(cal.status().leftCount),
                  static_cast<unsigned>(cal.status().rightCount),
                  static_cast<unsigned>(cal.status().excludedCount));
}

}  // namespace detail

/// The view for this frame. `secondsSinceLayoutChange` arms the non-ABORT buttons once it reaches
/// kArmingDelaySeconds (TouchGate::view() tracks it); `save` tells the finished screen whether the
/// program managed to write the result. A pure function of the calibrator's public queries.
///
/// CAL1 (additions only — the defaults are today's screens, field for field): `rule` ByPush is the
/// standard Calibrate's — four steps (1/4..4/4), PUSH FRONT FIRST, the front rule while the IMU
/// warms up, and never the WHICH END view (the program answers that question itself, the tick DONE
/// is accepted). `save` Held paints the held screen `finish.hold` names — FRONT FLIPPED or
/// MECHANISM PORT TURNED, NOTHING SAVED YET, a left box (today's FRONT FIRST id, labelled USE NEW
/// or SAVE) and RUN AGAIN (today's id, which discards the run). `finish.front` New makes a SAVED
/// header "SAVED -- NEW FRONT" in amber; `finish.note` is one amber SMALL row on the finished
/// screen (a calibrator warning that needs two rows is then cut to one, ending " ...").
[[nodiscard]] inline PanelView viewFor(const DrivetrainCalibrator& cal,
                                       double secondsSinceLayoutChange,
                                       SaveOutcome save = SaveOutcome::NotAttempted,
                                       FrontRule rule = FrontRule::AskedAfterPush,
                                       const FinishFacts& finish = FinishFacts{}) {
    using namespace panel_text;
    using detail::addPanelButton;
    using detail::addPanelLine;
    namespace d = detail;
    PanelView v{};
    const CalibratorStatus& st = cal.status();
    const CalibratorConfig& cfg = cal.config();
    const double secs = secondsSinceLayoutChange;
    const bool byPush = rule == FrontRule::ByPush;
    char buf[kPanelLineCap];

    switch (cal.phase()) {
        case Phase::Idle:
            v.screen = PanelScreen::NotStarted;
            v.headerTone = PanelTone::Blue;
            v.headerWord = kHeaderIdle.data();
            addPanelLine(v, d::kTextX, d::kInstr1Y, TextSize::Large, PanelTone::White, kNotStarted);
            addPanelButton(v, PanelButtonId::Abort, kAbortSlot, kLabelAbort, secs);
            return v;
        case Phase::WaitImu:
            v.screen = PanelScreen::ImuWait;
            v.headerTone = PanelTone::Blue;
            v.headerWord = byPush ? kHeaderImu4.data() : kHeaderImu.data();
            addPanelLine(v, d::kTextX, d::kInstr1Y, TextSize::Large, PanelTone::White, kHandsOff);
            addPanelLine(v, d::kTextX, d::kInstr2Y, TextSize::Large, PanelTone::White, kImuWarming);
            std::snprintf(buf, sizeof buf, "%.1f s", st.phaseSeconds);
            addPanelLine(v, d::kTextX, d::kLiveY, TextSize::Large, PanelTone::Amber, buf);
            if (byPush) {  // CAL1: the rule, before the push it governs
                addPanelLine(v, d::kTextX, d::kDetailY, TextSize::Medium, PanelTone::White,
                             kFrontRule);
            }
            std::snprintf(buf, sizeof buf, "NEXT: push it %.0f in along the tape",
                          cfg.pushDistance.value());
            addPanelLine(v, d::kTextX, d::kNextY, TextSize::Medium, PanelTone::White, buf);
            addPanelButton(v, PanelButtonId::Abort, kAbortSlot, kLabelAbort, secs);
            return v;
        case Phase::PushCapture: {
            if (!byPush && cal.accepts(Answer::FrontFirst)) {
                v.screen = PanelScreen::WhichEnd;
                v.headerTone = PanelTone::Blue;
                v.headerWord = kHeaderWhichEnd.data();
                addPanelLine(v, d::kTextX, d::kInstr1Y, TextSize::Large, PanelTone::White, kWhichEnd);
                addPanelLine(v, d::kTextX, d::kNextY, TextSize::Medium, PanelTone::Amber, kNextTwitch);
                addPanelButton(v, PanelButtonId::Front, kFrontBox, kLabelFront, secs);
                addPanelButton(v, PanelButtonId::Back, kBackBox, kLabelBack, secs);
                addPanelButton(v, PanelButtonId::Abort, kAbortSlot, kLabelAbort, secs);
                return v;
            }
            v.screen = PanelScreen::Push;
            v.headerTone = PanelTone::Blue;
            v.headerWord = byPush ? kHeaderPush4.data() : kHeaderPush.data();
            const bool ready = cal.pushReady();
            addPanelLine(v, d::kTextX, d::kInstr1Y, TextSize::Large, PanelTone::White,
                         byPush ? kPushFrontFirst : kPushForward);
            std::snprintf(buf, sizeof buf, "%.0f IN ALONG TAPE", cfg.pushDistance.value());
            addPanelLine(v, d::kTextX, d::kInstr2Y, TextSize::Large, PanelTone::White, buf);
            std::snprintf(buf, sizeof buf, "%u of %u turned", static_cast<unsigned>(st.movedNow),
                          static_cast<unsigned>(st.candidates));
            addPanelLine(v, d::kTextX, d::kLiveY, TextSize::Large,
                    ready ? PanelTone::Green : PanelTone::Amber, buf);
            const bool curved = std::abs(st.driftRad) > cfg.straightBound.value();
            if (curved) {
                std::snprintf(buf, sizeof buf, "drift %+.1f deg -- TOO CURVED",
                              st.driftRad / kDegreesToRadians);
            } else {
                std::snprintf(buf, sizeof buf, "drift %+.1f deg", st.driftRad / kDegreesToRadians);
            }
            addPanelLine(v, d::kTextX, d::kDetailY, TextSize::Medium,
                    curved ? PanelTone::Red : PanelTone::White, buf);
            if (ready) {
                addPanelLine(v, d::kTextX, d::kNextY, TextSize::Medium, PanelTone::Green,
                        byPush ? kNextTouchDoneClear : kNextTouchDone);
            } else if (st.movedNow < 2) {
                addPanelLine(v, d::kTextX, d::kNextY, TextSize::Medium, PanelTone::White,
                        kNextDoneWhenMoved);
            } else if (!st.pushFarEnough) {
                std::snprintf(buf, sizeof buf, "keep pushing -- the full %.0f in",
                              cfg.pushDistance.value());
                addPanelLine(v, d::kTextX, d::kNextY, TextSize::Medium, PanelTone::White, buf);
            } else {
                addPanelLine(v, d::kTextX, d::kNextY, TextSize::Medium, PanelTone::White, kNextLetGo);
            }
            if (ready) {
                // DONE only when the push is plausibly complete — the finish's own predicates
                // (pushReady()). Two of robot two's five runs ended on a DONE drawn too early.
                addPanelButton(v, PanelButtonId::Done, kPrimarySlot, kLabelDone, secs);
            }
            addPanelButton(v, PanelButtonId::Abort, kAbortSlot, kLabelAbort, secs);
            return v;
        }
        case Phase::SideAssign: {
            v.screen = PanelScreen::Pulses;
            v.headerTone = PanelTone::Amber;
            v.headerWord = byPush ? kHeaderPulses4.data() : kHeaderPulses.data();
            addPanelLine(v, d::kTextX, d::kInstr1Y, TextSize::Large, PanelTone::Amber, kStandClear);
            addPanelLine(v, d::kTextX, d::kInstr2Y, TextSize::Large, PanelTone::Amber, kTwitching);
            std::snprintf(buf, sizeof buf, "PORT %d  %.0f V", static_cast<int>(st.pulsePort),
                          st.pulseLevelVolts);
            addPanelLine(v, d::kTextX, d::kLiveY, TextSize::Large, PanelTone::Amber, buf);
            if (st.pulsing) {
                const bool past = std::abs(st.liveYawRad) >= cfg.pulseYawFloor.value();
                std::snprintf(buf, sizeof buf, "%u of %u  yaw %+.1f of %.1f deg",
                              static_cast<unsigned>(st.pulseOrdinal),
                              static_cast<unsigned>(st.pulseTotal),
                              st.liveYawRad / kDegreesToRadians,
                              cfg.pulseYawFloor.value() / kDegreesToRadians);
                addPanelLine(v, d::kTextX, d::kDetailY, TextSize::Medium,
                        past ? PanelTone::Green : PanelTone::White, buf);
            } else {
                addPanelLine(v, d::kTextX, d::kDetailY, TextSize::Medium, PanelTone::White,
                        kSettlingHandsOff);
            }
            d::panelChips(cal, buf, sizeof buf);
            addPanelLine(v, d::kTextX, d::kNextY, TextSize::Medium, PanelTone::White, buf);
            addPanelButton(v, PanelButtonId::Abort, kAbortRow, kLabelAbort, secs);
            return v;
        }
        case Phase::SpinTest: {
            v.screen = PanelScreen::Spin;
            v.headerTone = PanelTone::Amber;
            v.headerWord = byPush ? kHeaderSpin4.data() : kHeaderSpin.data();
            addPanelLine(v, d::kTextX, d::kInstr1Y, TextSize::Large, PanelTone::Amber, kStandClear);
            addPanelLine(v, d::kTextX, d::kInstr2Y, TextSize::Large, PanelTone::Amber, kSpinning);
            if (st.spinDirection != 0.0) {
                std::snprintf(buf, sizeof buf, "%s %.1f/%.0f TURNS",
                              st.spinDirection > 0.0 ? "CCW" : "CW", st.spinTurns,
                              cfg.spinTurns);
                addPanelLine(v, d::kTextX, d::kLiveY, TextSize::Large, PanelTone::Amber, buf);
                const bool stepped = st.spinLevelVolts > cfg.spinLevels[0].value();
                std::snprintf(buf, sizeof buf, "at %.0f V%s", st.spinLevelVolts,
                              stepped ? " -- stepped up" : "");
                addPanelLine(v, d::kTextX, d::kDetailY, TextSize::Medium,
                        stepped ? PanelTone::Amber : PanelTone::White, buf);
            } else {
                addPanelLine(v, d::kTextX, d::kLiveY, TextSize::Large, PanelTone::Amber, kSettling);
                addPanelLine(v, d::kTextX, d::kDetailY, TextSize::Medium, PanelTone::White,
                        cal.report().spinCount == 0 ? kSpinFirst : kSpinAgain);
            }
            addPanelLine(v, d::kTextX, d::kNextY, TextSize::Medium, PanelTone::White, kNextSaves);
            addPanelButton(v, PanelButtonId::Abort, kAbortRow, kLabelAbort, secs);
            return v;
        }
        case Phase::Done: {
            if (save == SaveOutcome::Held) {
                // CAL1: nothing is written until the operator touches the left box (USE NEW or
                // SAVE — today's FRONT FIRST id, relabelled: no new PanelButtonId before Part B,
                // because today's program and tests switch over the ids) or RUN AGAIN (today's id,
                // in the middle right: it discards this run). A field disable writes nothing.
                v.screen = PanelScreen::Held;
                v.headerTone = PanelTone::Amber;
                addPanelLine(v, d::kTextX, d::kInstr1Y, TextSize::Large, PanelTone::Amber,
                             kNothingSavedYet);
                if (finish.hold == HoldReason::MechanismTurned) {
                    v.headerWord = kHeaderMechTurned.data();
                    addPanelLine(v, d::kTextX, d::kNextY, TextSize::Medium, PanelTone::Amber,
                                 finish.holdLine1 != nullptr ? finish.holdLine1 : "");
                    addPanelLine(v, d::kTextX, d::kSmall1Y, TextSize::Small, PanelTone::White,
                                 finish.holdLine2 != nullptr ? finish.holdLine2
                                                             : kMechNotOnCard.data());
                    addPanelLine(v, d::kTextX, d::kSmall2Y, TextSize::Small, PanelTone::White,
                                 kMechSave);
                    addPanelLine(v, d::kTextX, d::kSmall3Y, TextSize::Small, PanelTone::Amber,
                                 kMechAgain);
                    addPanelButton(v, PanelButtonId::Front, kFrontBox, kLabelSave, secs);
                } else {
                    v.headerWord = kHeaderFlipped.data();
                    addPanelLine(v, d::kTextX, d::kNextY, TextSize::Medium, PanelTone::Amber,
                                 kFlipWhat);
                    addPanelLine(v, d::kTextX, d::kSmall1Y, TextSize::Small, PanelTone::White,
                                 kFlipAgain);
                    addPanelLine(v, d::kTextX, d::kSmall2Y, TextSize::Small, PanelTone::White,
                                 kFlipNew);
                    addPanelLine(v, d::kTextX, d::kSmall3Y, TextSize::Small, PanelTone::Amber,
                                 kFlipWarn);
                    addPanelButton(v, PanelButtonId::Front, kFrontBox, kLabelNew, secs);
                }
                addPanelButton(v, PanelButtonId::RunAgain, kBackBox, kLabelRunAgain, secs);
                return v;
            }
            const bool newFront = finish.front == FrontChoice::New;
            v.screen = PanelScreen::Saved;
            v.headerTone = save == SaveOutcome::Saved
                               ? (newFront ? PanelTone::Amber : PanelTone::Green)
                           : save == SaveOutcome::NotSaved ? PanelTone::Red
                                                           : PanelTone::Blue;
            v.headerWord = save == SaveOutcome::Saved
                               ? (newFront ? kHeaderSavedNew.data() : kHeaderSaved.data())
                           : save == SaveOutcome::NotSaved ? kHeaderNotSaved.data()
                                                           : kHeaderMeasured.data();
            const hal::DrivetrainDescription& r = cal.result();
            std::snprintf(buf, sizeof buf, "LEFT %u  RIGHT %u", static_cast<unsigned>(r.left.count),
                          static_cast<unsigned>(r.right.count));
            addPanelLine(v, d::kTextX, d::kHeadlineY, TextSize::Large, PanelTone::Green, buf);
            std::snprintf(buf, sizeof buf, "ipr L %.5f R %.5f", r.left.inchesPerRadian.value(),
                          r.right.inchesPerRadian.value());
            addPanelLine(v, d::kTextX, d::kLine1Y, TextSize::Medium, PanelTone::White, buf);
            std::snprintf(buf, sizeof buf, "track %.3f in", r.trackWidth.value());
            addPanelLine(v, d::kTextX, d::kLine2Y, TextSize::Medium, PanelTone::White, buf);
            const CalibrationReport& rep = cal.report();
            if (rep.excludedCount > 0) {
                std::size_t used =
                    static_cast<std::size_t>(std::snprintf(buf, sizeof buf, "LEFT OUT"));
                for (std::size_t k = 0; k < rep.excludedCount; ++k) {
                    char p[8];
                    const int len = std::snprintf(p, sizeof p, " %d",
                                                  static_cast<int>(rep.excludedPorts[k]));
                    if (len <= 0 || used + static_cast<std::size_t>(len) > kNoteCols) {
                        std::snprintf(buf, sizeof buf, "LEFT OUT: %u PORTS",
                                      static_cast<unsigned>(rep.excludedCount));
                        break;
                    }
                    std::snprintf(buf + used, sizeof buf - used, "%s", p);
                    used += static_cast<std::size_t>(len);
                }
                addPanelLine(v, d::kTextX, d::kNoteY, TextSize::Medium, PanelTone::Amber, buf);
            }
            addPanelLine(v, d::kTextX, d::kSmall1Y, TextSize::Small,
                    save == SaveOutcome::NotSaved ? PanelTone::Red : PanelTone::White,
                    save == SaveOutcome::Saved      ? kCopyThenRunAgain
                    : save == SaveOutcome::NotSaved ? kCopyNow
                                                    : kCopyThese);
            const bool note = finish.note != nullptr && finish.note[0] != '\0';
            if (cal.warning()[0] != '\0') {
                const auto w = wrapWords<kSmallCols, 2>(cal.warning());
                if (note && w.count > 1) {
                    // CAL1: the NOTE takes the third row, so the warning keeps ONE row and ends
                    // " ..." — the full text stays on the serial log.
                    constexpr std::size_t kKeep = kSmallCols - kWarningCut.size();
                    const auto one = wrapWords<kKeep, 1>(cal.warning());
                    std::size_t n = 0;
                    for (; one.count > 0 && n < kKeep && one.line[0][n] != '\0'; ++n) {
                        buf[n] = one.line[0][n];
                    }
                    for (const char c : kWarningCut) {
                        buf[n++] = c;
                    }
                    buf[n] = '\0';
                    addPanelLine(v, d::kTextX, d::kSmall2Y, TextSize::Small, PanelTone::Amber, buf);
                } else {
                    for (std::size_t k = 0; k < w.count; ++k) {
                        addPanelLine(v, d::kTextX, k == 0 ? d::kSmall2Y : d::kSmall3Y,
                                TextSize::Small, PanelTone::Amber, w.line[k]);
                    }
                }
            }
            if (note) {
                addPanelLine(v, d::kTextX, d::kSmall3Y, TextSize::Small, PanelTone::Amber,
                             finish.note);
            }
            addPanelButton(v, PanelButtonId::RunAgain, kRunAgainBox, kLabelRunAgain, secs);
            return v;
        }
        case Phase::Aborted: {
            v.screen = PanelScreen::Stopped;
            v.headerTone = PanelTone::Red;
            v.headerWord = kHeaderStopped.data();
            const StoppedWords words = stoppedWords(cal.refusal());
            addPanelLine(v, d::kTextX, d::kHeadlineY, TextSize::Large, PanelTone::Red, words.headline);
            addPanelLine(v, d::kTextX, d::kLine1Y, TextSize::Medium, PanelTone::White, words.why);
            addPanelLine(v, d::kTextX, d::kLine2Y, TextSize::Medium, PanelTone::White, words.doLine);
            if (cal.refusalPort() > 0) {
                std::snprintf(buf, sizeof buf, "PORT %d", cal.refusalPort());
                addPanelLine(v, d::kTextX, d::kNoteY, TextSize::Medium, PanelTone::Red, buf);
            }
            // The reason's own sentence, for its numbers — wrapped, never split mid-word; when it
            // does not fit three SMALL lines the third says where the rest is.
            const auto w = wrapWords<kSmallCols, 3>(cal.reason());
            const std::int16_t ys[3] = {d::kSmall1Y, d::kSmall2Y, d::kSmall3Y};
            const std::size_t shown = w.complete ? w.count : (w.count < 3 ? w.count : 2);
            for (std::size_t k = 0; k < shown; ++k) {
                addPanelLine(v, d::kTextX, ys[k], TextSize::Small, PanelTone::Grey, w.line[k]);
            }
            if (!w.complete) {
                addPanelLine(v, d::kTextX, ys[shown], TextSize::Small, PanelTone::Grey,
                        kFullReasonOnSerial);
            }
            addPanelButton(v, PanelButtonId::RunAgain, kRunAgainBox, kLabelRunAgain, secs);
            return v;
        }
    }
    return v;
}

/// The index of the ARMED button whose hit box (hitBox()) contains (x, y), or −1. A greyed button
/// is never hit; neither is anything outside every button.
[[nodiscard]] constexpr int hitTest(const PanelView& v, int x, int y) noexcept {
    for (std::size_t i = 0; i < v.buttonCount; ++i) {
        if (v.buttons[i].armed && contains(hitBox(v.buttons[i].rect), x, y)) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

/// A number that changes exactly when the LAYOUT does — the screen and every button's identity and
/// rectangle — and NOT when a button merely arms. TouchGate restarts its arming clock on a change.
[[nodiscard]] constexpr std::uint32_t layoutKey(const PanelView& v) noexcept {
    std::uint32_t h = 2166136261u;  // FNV-1a over the layout's bytes
    const auto mix = [&h](std::uint32_t value) {
        for (int k = 0; k < 4; ++k) {
            h ^= (value >> (8 * k)) & 0xFFu;
            h *= 16777619u;
        }
    };
    mix(static_cast<std::uint32_t>(v.screen));
    mix(v.buttonCount);
    for (std::size_t i = 0; i < v.buttonCount; ++i) {
        const PanelButton& b = v.buttons[i];
        mix(static_cast<std::uint32_t>(b.id));
        mix(static_cast<std::uint32_t>(static_cast<std::uint16_t>(b.rect.x0))
            | (static_cast<std::uint32_t>(static_cast<std::uint16_t>(b.rect.y0)) << 16));
        mix(static_cast<std::uint32_t>(static_cast<std::uint16_t>(b.rect.x1))
            | (static_cast<std::uint32_t>(static_cast<std::uint16_t>(b.rect.y1)) << 16));
    }
    return h;
}

/// One read of the touch screen, PROS-free: what screen_touch_status() returns, reduced to what a
/// release-edge reader needs.
struct TouchSample {
    /// The read FAILED (PROS: E_TOUCH_ERROR, every field −1). Ignored — and it must not move the
    /// release count, or the next good read would replay the last touch.
    bool error = false;
    /// PROS's release_count: it only ever increases, one per finger lifted.
    std::int32_t releaseCount = 0;
    /// Where the LAST touch was — stale while idle, so it means something only on a new release.
    std::int16_t x = 0;
    /// See x.
    std::int16_t y = 0;
};

/// What a touch should do.
enum class TouchAction : std::uint8_t {
    None,      ///< nothing — no new release, a miss, a greyed button, or a stale layout
    Answer,    ///< send TouchResult::answer to the calibrator
    RunAgain,  ///< rebuild the calibrator (the program's job — drivetrain_calibrator.hpp)
    SaveHeld,  ///< (CAL1) a held screen's USE NEW / SAVE: save this run AS MEASURED
};

/// The outcome of one touch read.
struct TouchResult {
    /// What to do.
    TouchAction action = TouchAction::None;
    /// The answer, when action is Answer.
    Answer answer = Answer::Abort;
    /// The button index hit in the DRAWN view (−1 for none) — for the serial log.
    int hit = -1;
    /// True when this read carried a NEW release (logged even when it hits nothing).
    bool newRelease = false;
};

/// The touch discipline, in one place: the release EDGE only; error reads ignored and never
/// replayed; the layout clock that arms buttons kArmingDelaySeconds after every layout change; and
/// a re-check against the calibrator before any answer — because the view a finger landed on may
/// be one frame older than the state machine.
class TouchGate {
public:
    /// `releaseCountAtStart`: PROS's release_count when the panel first appears, so a tap made
    /// before the program existed is never replayed as an answer.
    explicit TouchGate(std::int32_t releaseCountAtStart) noexcept
        : lastRelease_{releaseCountAtStart} {}

    /// The view to paint now. Restarts the arming clock at `nowSeconds` whenever the layout
    /// (layoutKey()) differs from the last view this returned — so the program never has to track
    /// it, and a new layout's buttons come up greyed on the very first frame they appear. `rule`
    /// and `finish` are viewFor()'s (CAL1; the defaults are today's). CAL1: on a held screen the
    /// clock ALSO restarts when the QUESTION changes — the hold's reason, or (MECHANISM PORT TURNED,
    /// one screen per port) its port: FRONT FLIPPED's USE NEW and every MECHANISM PORT TURNED's
    /// SAVE share one box and one id, so layoutKey() alone cannot see the next question arrive, and
    /// a reflexive second tap would land on the next question's live box.
    [[nodiscard]] PanelView view(const DrivetrainCalibrator& cal, double nowSeconds,
                                 SaveOutcome save = SaveOutcome::NotAttempted,
                                 FrontRule rule = FrontRule::AskedAfterPush,
                                 const FinishFacts& finish = FinishFacts{}) {
        PanelView v = viewFor(cal, haveKey_ ? nowSeconds - changedAt_ : 0.0, save, rule, finish);
        const std::uint32_t key = layoutKey(v);
        const HoldReason hold = v.screen == PanelScreen::Held ? finish.hold : HoldReason::None;
        const int holdPort = hold == HoldReason::MechanismTurned ? finish.holdPort : 0;
        if (!haveKey_ || key != lastKey_ || hold != lastHold_ || holdPort != lastHoldPort_) {
            haveKey_ = true;
            lastKey_ = key;
            lastHold_ = hold;
            lastHoldPort_ = holdPort;
            changedAt_ = nowSeconds;
            v = viewFor(cal, 0.0, save, rule, finish);
        }
        return v;
    }

    /// One touch read against the view that is ON THE SCREEN (`drawn`). A new release that lands on
    /// an armed button becomes an answer only if the calibrator still accepts it now (DONE also
    /// needs pushReady()); RUN AGAIN only once the run has ended. CAL1: on a HELD screen the left
    /// box (the FRONT FIRST id, relabelled) is SaveHeld — never an answer — and only while the run
    /// is Done.
    [[nodiscard]] TouchResult touch(const TouchSample& s, const PanelView& drawn,
                                    const DrivetrainCalibrator& cal) {
        TouchResult r{};
        if (s.error || s.releaseCount == lastRelease_) {
            return r;  // an error read, or no new release: x/y are stale — never act on them
        }
        lastRelease_ = s.releaseCount;
        r.newRelease = true;
        r.hit = hitTest(drawn, s.x, s.y);
        if (r.hit < 0) {
            return r;
        }
        switch (drawn.buttons[static_cast<std::size_t>(r.hit)].id) {
            case PanelButtonId::RunAgain:
                if (cal.phase() == Phase::Done || cal.phase() == Phase::Aborted) {
                    r.action = TouchAction::RunAgain;
                }
                return r;
            case PanelButtonId::Done: return offer(r, cal, Answer::Done, cal.pushReady());
            case PanelButtonId::Front:
                if (drawn.screen == PanelScreen::Held) {  // CAL1: USE NEW / SAVE, never an answer
                    if (cal.phase() == Phase::Done) {
                        r.action = TouchAction::SaveHeld;
                    }
                    return r;
                }
                return offer(r, cal, Answer::FrontFirst, true);
            case PanelButtonId::Back: return offer(r, cal, Answer::BackFirst, true);
            case PanelButtonId::Abort: return offer(r, cal, Answer::Abort, true);
        }
        return r;
    }

private:
    [[nodiscard]] static TouchResult offer(TouchResult r, const DrivetrainCalibrator& cal,
                                           Answer a, bool alsoReady) {
        if (alsoReady && cal.accepts(a)) {
            r.action = TouchAction::Answer;
            r.answer = a;
        }
        return r;
    }

    std::int32_t lastRelease_;
    std::uint32_t lastKey_ = 0;
    bool haveKey_ = false;
    double changedAt_ = 0.0;
    HoldReason lastHold_ = HoldReason::None;  // CAL1: the held screen's question, for the clock
    int lastHoldPort_ = 0;                    // CAL1: ... and, on MECHANISM PORT TURNED, its port
};

}  // namespace shulib::calibration
