#pragma once
//
// mechanism/tune_panel.hpp — "shulib Tune"'s brain screen as a PURE MODEL: what each page says, where
// each button is, which touches mean something (chunk TUNE1-B1, 2026-11-10; the MECH-S/TUNE1 brief's
// D7 (a) and D8 (a), with its re-check amendments #3, #7, #8 and #14).
//
// ── Why a model ─────────────────────────────────────────────────────────────────────
// The Tune program (mechanism/tune_program.hpp) is how a team tunes a mechanism ON THE ROBOT: a
// cascade lift's gains, its top speed, its homing push and its current cap, found at the bench by a
// student who is not a programmer. What the screen says and where its buttons are is what decides
// whether that works, so it is host-tested here, page by page, at every mechanism kind and the
// longest name a mechanism can have (23 characters) — the house's pattern since R3d Part 2F-1
// (calibration/calibrate_panel.hpp) and COMP-a (ui/touch_panel.hpp, whose budgets this uses: one set
// of budgets for every screen, all ESTIMATES until the SCREEN RULER is read — A4 register HA-136).
//
// ── The pages ───────────────────────────────────────────────────────────────────────
//   CLEAR        stage 1 of the clear confirm (re-check #7): NOTHING is made before READY — making a
//                3-wire line drives it to its start level (HA-119: robot two's claw moves at boot).
//                READY is greyed, with why in red, while no controller is connected, a field or switch
//                is plugged in or a controller button is held.
//   TUNE         the mechanism (‹ › picks one), its state, the selected number with [−] [+] and
//                FINE, TEST A / TEST B, HOME, UNDO, RESET, COPY, STRING, STOP and EXIT.
//   CONFIRM      stage 2 of the clear confirm: before a mechanism's FIRST motion in the session
//                (a TEST, a HOME or the stick), GO or BACK.
//   REPORT       a TEST move's report: the verdict, the time, the farthest past the target, the
//                settle, the most current, the final error, the hold current over the second after
//                it settled (or WHY there is none: the hold ENDED, it was stopped), and a WARNING when
//                the stall signature is not above that hold or the cap not above the peak (re-check #3:
//                the four currents' order). A power run shows no settle and no hold (n/a). Its STOP is
//                where the TUNE page's is, never greyed (the page flips here as a test ends).
//   STRING CHECK EMPTY in this run: MECH-S-B (after Nov 21) fills it.
//   EXITED       after EXIT: every limit this program changed written back, every mechanism released,
//                and what EXIT COPIED (changed numbers never copied) or RESET (a cap that could not go
//                back alone) on the way.
//   STOPPED      a program error: everything released, the error's words, nothing more.
//
// ── The rules every page keeps (tested in test/tune_panel_test.cpp) ─────────────────
// Every rectangle inside 480 × 236; every pair of buttons further apart than twice the hit pads (so
// no padded hit box overlaps another); no text line under a button; every text within its size's
// budget; printable ASCII with no '%' (ui::screenText — PROS hands text to VEXos as a FORMAT string).
// Every word a page shows is built HERE from plain facts the program hands in (TuneFacts), so the
// budgets hold for every value a mechanism can have: numbers are printed %.5g (at most 11
// characters), a status sentence is wrapped onto two lines and cut there (the program logs it whole).
// A new layout's buttons are dead for ui::kArmingDelaySeconds (a reflexive second tap lands on
// nothing) — the program says so in TuneFacts::armed — EXCEPT the STOP of the tuning and REPORT pages
// (the same cell on both), which is always armed: a reflexive tap on it only stops everything, and the
// page a test starts on (after GO) must be stoppable from its first tick.
//
// PROS-free, allocation-free, header-only; nothing here draws, reads a touch or knows a clock.
// NOTHING HERE HAS RUN ON A ROBOT.

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string_view>

#include "shulib/mechanism/live_tuning.hpp"
#include "shulib/mechanism/spec.hpp"
#include "shulib/ui/touch_panel.hpp"

namespace shulib::mechanism {

/// The Tune program's pages (header).
enum class TunePage : std::uint8_t {
    Clear,        ///< stage 1 of the clear confirm: nothing is made yet
    Tune,         ///< the tuning page
    Confirm,      ///< stage 2: before a mechanism's first motion in the session
    Report,       ///< a TEST move's report
    StringCheck,  ///< STRING CHECK — empty until MECH-S-B
    Exited,       ///< after EXIT: limits written back, everything released
    Stopped,      ///< a program error: everything released
};

/// Every button any page has.
enum class TuneButton : std::uint8_t {
    Ready,        ///< stage 1: make the mechanisms
    PrevMech,     ///< the previous mechanism
    NextMech,     ///< the next mechanism
    PrevParam,    ///< the previous number
    NextParam,    ///< the next number
    Minus,        ///< one [−] press on the selected number
    Plus,         ///< one [+] press on the selected number
    Fine,         ///< FINE on/off (a gain's factor 1.05 instead of 1.25)
    TestA,        ///< a test move to target A
    TestB,        ///< a test move to target B
    Home,         ///< the declaration's homing step
    Undo,         ///< undo the last change
    Reset,        ///< this mechanism back to the robot file's numbers
    Copy,         ///< log this mechanism's COPY block
    StringCheck,  ///< the STRING CHECK page (a lift's)
    Stop,         ///< stop every mechanism
    Exit,         ///< write back every limit changed, release everything
    Go,           ///< stage 2: the motion may start
    Back,         ///< back to the tuning page
    Again,        ///< the same test again
    TuneAgain,    ///< after EXIT: back to the tuning page
};

/// One button as a page draws it.
struct TuneButtonView {
    /// Which button.
    TuneButton id = TuneButton::Stop;
    /// Where it is drawn (its hit box is ui::hitBox() of this).
    ui::Rect rect{};
    /// Its label (SMALL text, centred). Room for the longest label written in tuneView() ("READY - I AM CLEAR",
    /// 18 characters) — a 16-byte box once cut it to "READY - I AM CL"; tune_panel_test pins every label whole.
    char label[24]{};
    /// It takes touches (false: greyed — not available, or the layout is still arming).
    bool armed = false;
    /// Its colour by meaning (STOP red, GO and READY green, the rest blue).
    ui::Tone tone = ui::Tone::Blue;
};

/// One text line as a page draws it.
struct TuneLine {
    /// The text, at most kSmallCols characters (a LARGE line at most kLargeCols).
    char text[ui::kSmallCols + 1]{};
    /// Its size.
    ui::TextSize size = ui::TextSize::Small;
    /// Its left column.
    std::int16_t x = 0;
    /// Its top row.
    std::int16_t y = 0;
    /// Its colour by meaning.
    ui::Tone tone = ui::Tone::White;
};

/// The most text lines a page has.
inline constexpr std::size_t kTuneMaxLines = 10;
/// The most buttons a page has.
inline constexpr std::size_t kTuneMaxButtons = 16;

/// One page as it is to be drawn: its lines and its buttons.
struct TuneView {
    /// Which page.
    TunePage page = TunePage::Clear;
    /// Its text lines.
    std::array<TuneLine, kTuneMaxLines> lines{};
    /// How many lines are real.
    std::size_t lineCount = 0;
    /// Its buttons.
    std::array<TuneButtonView, kTuneMaxButtons> buttons{};
    /// How many buttons are real.
    std::size_t buttonCount = 0;

    /// The LAYOUT's key: the page and every button's identity and rectangle — never whether a button
    /// is armed, never text (ui::LayoutKey). An ArmingClock restarts when it changes.
    [[nodiscard]] std::uint32_t layoutKey() const noexcept {
        ui::LayoutKey k;
        k.mix(static_cast<std::uint32_t>(page));
        for (std::size_t i = 0; i < buttonCount; ++i) {
            k.mix(static_cast<std::uint32_t>(buttons[i].id));
            k.mix(buttons[i].rect);
        }
        return k.value();
    }
    /// The button with `id`, or nullptr when the page has none.
    [[nodiscard]] const TuneButtonView* button(TuneButton id) const noexcept {
        for (std::size_t i = 0; i < buttonCount; ++i) {
            if (buttons[i].id == id) {
                return &buttons[i];
            }
        }
        return nullptr;
    }
};

/// Which test a report is of.
enum class TuneTest : std::uint8_t {
    None,  ///< none
    A,     ///< TEST A
    B,     ///< TEST B
};

/// A TEST move's report (re-check #3, #7): what the program measured from its samples
/// (tune_program.hpp's tuneReportFrom()). A field the move cannot have is negative ("n/a").
struct TuneReport {
    /// Which test.
    TuneTest test = TuneTest::None;
    /// The mechanism's kind.
    Kind kind = Kind::Position;
    /// The mechanism's unit word ("in", "deg", "rpm", "V").
    const char* unit = "";
    /// The target (a position, an output rpm, or the volts of a power run).
    double target = 0.0;
    /// The verdict (manipulation::mechanismOutcomeName's word).
    const char* verdict = "";
    /// Seconds from the command to the verdict (the service's StepReport::elapsed).
    double seconds = 0.0;
    /// The move's time limit, seconds (re-check #7: the plan's time × 2 + the settle time + 0.5 s,
    /// at most 10 s).
    double bound = 0.0;
    /// The farthest past the target in the direction of travel, ≥ 0 (negative: n/a).
    double overshoot = -1.0;
    /// Seconds from the first sample inside "reached" (or the at-speed band) to the verdict
    /// (negative: never inside, or n/a).
    double settle = -1.0;
    /// The most current any of its motors drew during the move, A.
    double peakAmps = 0.0;
    /// Where it ended minus the target (negative values are short of it upward; n/a for a power run
    /// — see `hasFinalError`).
    double finalError = 0.0;
    /// `finalError` means something (not a power run).
    bool hasFinalError = false;
    /// The mean current over the second after it settled, A (negative: n/a — the move did not
    /// succeed, a power run, or `holdEnded` / `holdStopped` says why).
    double holdAmps = -1.0;
    /// The hold ENDED inside its second: the stall signature tripped on the hold itself (V1) — the
    /// hold is n/a and the stall warning is set.
    bool holdEnded = false;
    /// A STOP ended the hold second (`stoppedBy` names it): the hold is n/a, with no warning of its own.
    bool holdStopped = false;
    /// Its stall signature's amps (0: it states none).
    double stallAmps = 0.0;
    /// Its current cap, A per motor (0: none).
    double capAmps = 0.0;
    /// WARNING: the stall signature is not above the hold current — an ordinary hold can trip it.
    bool warnStallNotAboveHold = false;
    /// WARNING: the cap is not above the peak — the cap limited the move.
    bool warnCapNotAbovePeak = false;
    /// For a CANCELLED move: what stopped it ("a controller button"), else "".
    const char* stoppedBy = "";
};

/// The facts the TUNE page shows about the selected mechanism.
struct TuneMechFacts {
    /// Its name (at most 23 characters; the program passes the spec's).
    const char* name = "";
    /// Its kind.
    Kind kind = Kind::Power;
    /// A position mechanism's units.
    Units units = Units::Degrees;
    /// Built at boot (false: ABSENT).
    bool built = true;
    /// A position mechanism has found its zero.
    bool homed = false;
    /// A position mechanism has a homing step (`.home` other than neverHomed).
    bool homes = false;
    /// Where it is (a position mechanism, its units) or how fast (a flywheel, output rpm), or the
    /// volts commanded (a power mechanism).
    double now = 0.0;
    /// What it aims at (a position's hold or move target, a flywheel's target rpm; 0 when none).
    double target = 0.0;
    /// A flywheel is spinning / a position mechanism holds or moves.
    bool active = false;
    /// Its last verdict's word ("" before any).
    const char* lastVerdict = "";
    /// Any of its numbers differ from the robot file's.
    bool changed = false;
    /// Its current cap differs from the robot file's (re-check #8: a cap set here stays in the motor).
    bool capChanged = false;
};

/// One live number as the TUNE page shows it.
struct TuneParamFacts {
    /// Its page name ("kP"; live_tuning.hpp's Parameter::name). Null: no number (a pneumatic).
    const char* name = nullptr;
    /// Its live value.
    double value = 0.0;
    /// The robot file's value.
    double file = 0.0;
    /// Its unit word (tunableUnit()).
    const char* unit = "";
    /// How a press moves it.
    StepRule rule = StepRule::Gain;
    /// It is a current cap the mechanism does not state (live and in the file): the page says how one
    /// starts ([−] from none sets the motor's 2.5 A less one step; [+] from none is above the motor's
    /// most and refused — live_tuning.hpp's step rule).
    bool capNone = false;
};

/// Everything a page shows, as plain facts (the program fills it every frame).
struct TuneFacts {
    /// Which page.
    TunePage page = TunePage::Clear;
    /// The layout has been up long enough to take touches (ui::ArmingClock::armedAfter).
    bool armed = false;
    /// How many mechanisms the robot declares.
    std::size_t mechCount = 0;
    /// Which one is shown (0-based).
    std::size_t mech = 0;
    /// The shown mechanism.
    TuneMechFacts m{};
    /// The selected number.
    TuneParamFacts sel{};
    /// The number after it (for the "next:" line), name null at the end of the list.
    TuneParamFacts next{};
    /// Which number is selected (0-based) and how many the mechanism has.
    std::size_t param = 0;
    /// How many numbers the mechanism has live.
    std::size_t paramCount = 0;
    /// FINE is on.
    bool fine = false;
    /// A test move, a homing or a test's hold second is running: every button but STOP is greyed.
    bool busy = false;
    /// How many changes UNDO can reach.
    std::size_t undoDepth = 0;
    /// The controller is connected.
    bool controller = false;
    /// VEXos reports a field or a competition switch plugged in.
    bool field = false;
    /// A controller button is held (every button is a STOP): READY waits.
    bool buttonHeld = false;
    /// ANY mechanism's live current cap differs from the robot file's (the TUNE page's safety line: a
    /// cap set here stays in the motor — EXIT before you stop).
    bool anyCapChanged = false;
    /// The last sentence (an applied change, a refusal, a verdict): wrapped onto two lines, cut there.
    const char* status = "";
    /// TEST A's target in the mechanism's unit, and its name ("floor", or "1/4" of the range).
    double testA = 0.0;
    /// TEST B's target.
    double testB = 0.0;
    /// TEST A's name.
    const char* testAName = "";
    /// TEST B's name.
    const char* testBName = "";
    /// CONFIRM: what will move it ("TEST A", "HOME", "the stick").
    const char* confirmWhat = "";
    /// REPORT: the report.
    TuneReport report{};
    /// EXITED: how many limits were written back.
    int limitsWritten = 0;
    /// EXITED: how many mechanisms' changed numbers EXIT COPIED to the log first (never copied since
    /// their last change).
    int copiedAtExit = 0;
    /// EXITED: how many mechanisms EXIT had to RESET (a live signature at or above the file's cap, so
    /// the cap alone could not go back) — each COPIED first.
    int resetAtExit = 0;
    /// STOPPED: why.
    const char* stoppedWhy = "";
};

namespace tune_layout {
/// The SMALL line pitch on every page, pixels (ui::kSmallHeight).
inline constexpr std::int16_t kPitch = ui::kSmallHeight;
/// The tuning page's three top lines.
inline constexpr std::int16_t kTitleY = 0;
/// The state line.
inline constexpr std::int16_t kStateY = 16;
/// The safety line (the controller, the field, a cap left changed).
inline constexpr std::int16_t kSafetyY = 32;
/// Row 1 (‹ › ^ v − + FINE).
inline constexpr ui::Rect kRow1{0, 52, 479, 81};
/// The selected number's three lines.
inline constexpr std::int16_t kParamY = 86;
/// Row 2 (TEST A, TEST B, HOME, UNDO, RESET, COPY).
inline constexpr ui::Rect kRow2{0, 136, 479, 165};
/// The two status lines.
inline constexpr std::int16_t kStatusY = 169;
/// Row 3 (STRING, STOP, EXIT) — and the other pages' one row of buttons.
inline constexpr ui::Rect kRow3{0, 204, 479, 235};
/// A LARGE title's top row on the other pages.
inline constexpr std::int16_t kLargeTitleY = 2;
/// The other pages' first SMALL line.
inline constexpr std::int16_t kBodyY = 42;
}  // namespace tune_layout

namespace detail {

/// A button at columns [x0, x1] of `row`.
[[nodiscard]] constexpr ui::Rect tuneCell(ui::Rect row, int x0, int x1) noexcept {
    return ui::Rect{static_cast<std::int16_t>(x0), row.y0, static_cast<std::int16_t>(x1), row.y1};
}

/// The word for a mechanism's kind on the title line (at most 9 characters).
[[nodiscard]] constexpr const char* tuneKindWord(Kind k, Units u) noexcept {
    switch (k) {
        case Kind::Position: return u == Units::Inches ? "LIFT" : "ARM";
        case Kind::Speed: return "FLYWHEEL";
        case Kind::Power: return "POWER";
        case Kind::Pneumatic: return "PNEUMATIC";
    }
    return "?";
}

/// The unit word of a position mechanism ("in", "deg").
[[nodiscard]] constexpr const char* tuneUnitWord(Units u) noexcept { return u == Units::Inches ? "in" : "deg"; }

/// `v` bounded to ±99999 (every number on the screen fits its column).
[[nodiscard]] inline double tuneBounded(double v) noexcept {
    if (!std::isfinite(v)) {
        return 0.0;
    }
    return v > 99999.0 ? 99999.0 : v < -99999.0 ? -99999.0 : v;
}

/// What one press does, in at most 20 characters ("x1.25", "+-0.05 V").
inline void tuneRuleWords(char* out, std::size_t cap, StepRule r, bool fine, Units u) noexcept {
    const bool in = u == Units::Inches;
    switch (r) {
        case StepRule::Gain: std::snprintf(out, cap, "%s", fine ? "x or / 1.05 (FINE)" : "x or / 1.25"); return;
        case StepRule::ZeroGain: std::snprintf(out, cap, "%s", "x1.25, from 0 kV*0.01"); return;
        case StepRule::Volts: std::snprintf(out, cap, "%s", "+-0.05 V"); return;
        case StepRule::Amps: std::snprintf(out, cap, "%s", "+-0.05 A"); return;
        case StepRule::Percent: std::snprintf(out, cap, "%s", "x or / 1.05"); return;
        case StepRule::Reached: std::snprintf(out, cap, "%s", in ? "+-0.05 in" : "+-0.5 deg"); return;
        case StepRule::Seconds: std::snprintf(out, cap, "%s", "+-0.05 s"); return;
        case StepRule::Limit: std::snprintf(out, cap, "%s", in ? "+-0.25 in" : "+-1 deg"); return;
    }
    std::snprintf(out, cap, "%s", "");
}

/// Wrap `s` onto two lines of at most `cols` characters, breaking the first at its last space
/// within `cols` (or hard at `cols`); the second is cut at `cols`.
inline void tuneWrap(const char* s, char* a, char* b, std::size_t cols) noexcept {
    a[0] = '\0';
    b[0] = '\0';
    if (s == nullptr) {
        return;
    }
    const std::size_t n = std::strlen(s);
    if (n <= cols) {
        ui::copyScreenText(a, cols + 1, std::string_view{s, n}, cols);
        return;
    }
    std::size_t cut = cols;
    while (cut > 0 && s[cut] != ' ') {
        --cut;
    }
    if (cut == 0) {
        cut = cols;
    }
    ui::copyScreenText(a, cols + 1, std::string_view{s, cut}, cols);
    std::size_t rest = cut;
    while (rest < n && s[rest] == ' ') {
        ++rest;
    }
    ui::copyScreenText(b, cols + 1, std::string_view{s + rest, n - rest}, cols);
}

/// Build a page: lines and buttons added in order.
class TuneViewBuilder {
public:
    explicit TuneViewBuilder(TunePage p) noexcept { v_.page = p; }
    /// A line of `size` at (x, y), `tone`, its text cut to its budget (ui::copyScreenText).
    void line(ui::TextSize size, std::int16_t x, std::int16_t y, ui::Tone tone, const char* text) noexcept {
        if (v_.lineCount >= kTuneMaxLines) {
            return;
        }
        TuneLine& l = v_.lines[v_.lineCount++];
        l.size = size;
        l.x = x;
        l.y = y;
        l.tone = tone;
        ui::copyScreenText(l.text, sizeof l.text, text != nullptr ? text : "", ui::budgetFor(size));
    }
    /// A button.
    void button(TuneButton id, ui::Rect r, const char* label, bool armed, ui::Tone tone = ui::Tone::Blue) noexcept {
        if (v_.buttonCount >= kTuneMaxButtons) {
            return;
        }
        TuneButtonView& b = v_.buttons[v_.buttonCount++];
        b.id = id;
        b.rect = r;
        b.armed = armed;
        b.tone = tone;
        ui::copyScreenText(b.label, sizeof b.label, label, sizeof b.label - 1);
    }
    /// The page built.
    [[nodiscard]] const TuneView& view() const noexcept { return v_; }

private:
    TuneView v_{};
};

}  // namespace detail

/// The SMALL characters a button `r` fits (at the width the SMALL budget implies over 472 px).
[[nodiscard]] constexpr std::size_t tuneLabelCols(ui::Rect r) noexcept {
    const int w = r.x1 - r.x0 + 1 - 4;
    return w <= 0 ? 0 : static_cast<std::size_t>(w / (472 / static_cast<int>(ui::kSmallCols)));
}

/// THE PAGE for `f` (header): every word, every button and whether it is armed. Pure.
[[nodiscard]] inline TuneView tuneView(const TuneFacts& f) noexcept {
    using detail::tuneCell;
    namespace L = tune_layout;
    char t[160];
    const bool arm = f.armed;
    switch (f.page) {
        case TunePage::Clear: {
            detail::TuneViewBuilder b{TunePage::Clear};
            b.line(ui::TextSize::Large, 8, L::kLargeTitleY, ui::Tone::Blue, "shulib TUNE");
            b.line(ui::TextSize::Small, 8, L::kBodyY, ui::Tone::White, "Change a mechanism's numbers on this screen,");
            b.line(ui::TextSize::Small, 8, L::kBodyY + 16, ui::Tone::White, "watch a TEST move, then COPY them to the file.");
            b.line(ui::TextSize::Small, 8, L::kBodyY + 40, ui::Tone::Amber, "READY makes every mechanism: a PNEUMATIC MOVES");
            b.line(ui::TextSize::Small, 8, L::kBodyY + 56, ui::Tone::Amber, "to its start now. Robot on blocks, hands clear.");
            b.line(ui::TextSize::Small, 8, L::kBodyY + 80, ui::Tone::White, "ANY controller button STOPS every mechanism.");
            b.line(ui::TextSize::Small, 8, L::kBodyY + 96, ui::Tone::White, "A cap set here stays in the motor: EXIT before");
            b.line(ui::TextSize::Small, 8, L::kBodyY + 112, ui::Tone::White, "you stop the program from the brain.");
            // READY makes the mechanisms (a pneumatic MOVES): never with no controller (it carries the
            // STOP), a field or switch plugged in, or a button held (re-check #7) — greyed, and why in red
            const char* blocked = !f.controller ? "No controller: connect it first (it is the STOP)."
                                  : f.field     ? "Field or switch plugged in: unplug it first."
                                  : f.buttonHeld ? "A controller button is held: let go first."
                                                 : nullptr;
            if (blocked != nullptr) {
                b.line(ui::TextSize::Small, 8, L::kStatusY, ui::Tone::Red, blocked);
            } else {
                char s1[ui::kSmallCols + 1];
                char s2[ui::kSmallCols + 1];
                detail::tuneWrap(f.status, s1, s2, ui::kSmallCols);
                b.line(ui::TextSize::Small, 0, L::kStatusY, ui::Tone::White, s1);
                b.line(ui::TextSize::Small, 0, static_cast<std::int16_t>(L::kStatusY + 16), ui::Tone::White, s2);
            }
            b.button(TuneButton::Ready, tuneCell(L::kRow3, 0, 299), "READY - I AM CLEAR", arm && blocked == nullptr,
                     ui::Tone::Green);
            return b.view();
        }
        case TunePage::Tune: {
            detail::TuneViewBuilder b{TunePage::Tune};
            const TuneMechFacts& m = f.m;
            const bool pos = m.kind == Kind::Position;
            const char* u = pos ? detail::tuneUnitWord(m.units) : m.kind == Kind::Speed ? "rpm" : "V";
            // the title: "1/3 cascade  LIFT  NOT HOMED"
            std::snprintf(t, sizeof t, "%u/%u %.23s  %s  %s", static_cast<unsigned>(f.mech + 1),
                          static_cast<unsigned>(f.mechCount), m.name != nullptr ? m.name : "",
                          detail::tuneKindWord(m.kind, m.units),
                          !m.built ? "ABSENT" : pos ? (m.homed ? "HOMED" : "NOT HOMED") : "");
            b.line(ui::TextSize::Small, 0, L::kTitleY, !m.built ? ui::Tone::Red : ui::Tone::Blue, t);
            // the state line
            const char* last = m.lastVerdict != nullptr && m.lastVerdict[0] != '\0' ? m.lastVerdict : "none";
            if (!m.built) {
                std::snprintf(t, sizeof t, "%s", "ABSENT at boot: see the terminal for why");
            } else if (m.kind == Kind::Pneumatic) {
                std::snprintf(t, sizeof t, "%s", "a pneumatic: nothing on it is live");
            } else if (pos && !m.homed) {
                std::snprintf(t, sizeof t, "not homed%s, last %s", m.homes ? ": press HOME" : "", last);
            } else if (pos) {
                std::snprintf(t, sizeof t, "at %.2f %s, %s %.2f, last %s", detail::tuneBounded(m.now), u,
                              m.active ? "to" : "was to", detail::tuneBounded(m.target), last);
            } else if (m.kind == Kind::Speed) {
                std::snprintf(t, sizeof t, "%s %.0f rpm, to %.0f, last %s", m.active ? "spinning" : "stopped,",
                              detail::tuneBounded(m.now), detail::tuneBounded(m.target), last);
            } else {
                std::snprintf(t, sizeof t, "volts %+.1f, last %s", detail::tuneBounded(m.now), last);
            }
            b.line(ui::TextSize::Small, 0, L::kStateY, ui::Tone::White, t);
            // the safety line, by priority
            const char* safety = "any controller button STOPS every mechanism";
            ui::Tone st = ui::Tone::Grey;
            if (f.busy) {
                safety = "MOVING -- any controller button STOPS it";
                st = ui::Tone::Amber;
            } else if (f.field) {
                safety = "FIELD or SWITCH plugged in: nothing moves";
                st = ui::Tone::Red;
            } else if (!f.controller) {
                safety = "NO CONTROLLER (it is the STOP): nothing moves";
                st = ui::Tone::Red;
            } else if (m.capChanged || f.anyCapChanged) {
                safety = "cap changed: EXIT or RESET before you stop";
                st = ui::Tone::Amber;
            }
            b.line(ui::TextSize::Small, 0, L::kSafetyY, st, safety);
            // the selected number
            const bool live = f.sel.name != nullptr && f.paramCount > 0;
            if (live) {
                const bool differs = f.sel.value != f.sel.file;
                std::snprintf(t, sizeof t, "> %-14.14s %.5g %.11s%s", f.sel.name, detail::tuneBounded(f.sel.value),
                              f.sel.unit != nullptr ? f.sel.unit : "", differs ? " *" : "");
                b.line(ui::TextSize::Small, 0, L::kParamY, differs ? ui::Tone::Green : ui::Tone::White, t);
                char rule[24];
                detail::tuneRuleWords(rule, sizeof rule, f.sel.rule, f.fine, m.units);
                if (f.sel.capNone) {
                    // no cap stated: [+] would ask 2.55 A (above the motor's most: refused); [-] starts one
                    std::snprintf(t, sizeof t, "%s", "  file: none; - starts a cap at 2.45 A");
                } else {
                    std::snprintf(t, sizeof t, "  file %.5g, a press: %s", detail::tuneBounded(f.sel.file), rule);
                }
                b.line(ui::TextSize::Small, 0, static_cast<std::int16_t>(L::kParamY + 16), ui::Tone::Grey, t);
                std::snprintf(t, sizeof t, "  %u of %u%s%.14s", static_cast<unsigned>(f.param + 1),
                              static_cast<unsigned>(f.paramCount), f.next.name != nullptr ? ", next: " : "",
                              f.next.name != nullptr ? f.next.name : "");
                b.line(ui::TextSize::Small, 0, static_cast<std::int16_t>(L::kParamY + 32), ui::Tone::Grey, t);
            } else {
                b.line(ui::TextSize::Small, 0, L::kParamY, ui::Tone::Grey, "  no live numbers on this mechanism");
            }
            // the status sentence, two lines
            char s1[ui::kSmallCols + 1];
            char s2[ui::kSmallCols + 1];
            detail::tuneWrap(f.status, s1, s2, ui::kSmallCols);
            b.line(ui::TextSize::Small, 0, L::kStatusY, ui::Tone::White, s1);
            b.line(ui::TextSize::Small, 0, static_cast<std::int16_t>(L::kStatusY + 16), ui::Tone::White, s2);
            // the buttons
            const bool idle = arm && !f.busy;
            const bool motor = m.built && m.kind != Kind::Pneumatic;
            b.button(TuneButton::PrevMech, tuneCell(L::kRow1, 0, 47), "<", idle && f.mechCount > 1);
            b.button(TuneButton::NextMech, tuneCell(L::kRow1, 57, 104), ">", idle && f.mechCount > 1);
            b.button(TuneButton::PrevParam, tuneCell(L::kRow1, 124, 171), "^", idle && live);
            b.button(TuneButton::NextParam, tuneCell(L::kRow1, 181, 228), "v", idle && live);
            b.button(TuneButton::Minus, tuneCell(L::kRow1, 248, 307), "-", idle && live);
            b.button(TuneButton::Plus, tuneCell(L::kRow1, 317, 376), "+", idle && live);
            b.button(TuneButton::Fine, tuneCell(L::kRow1, 396, 479), f.fine ? "FINE on" : "FINE", idle && live);
            b.button(TuneButton::TestA, tuneCell(L::kRow2, 0, 69), "TEST A", idle && motor);
            b.button(TuneButton::TestB, tuneCell(L::kRow2, 80, 149), "TEST B", idle && motor);
            b.button(TuneButton::Home, tuneCell(L::kRow2, 160, 229), "HOME", idle && motor && pos && m.homes);
            std::snprintf(t, sizeof t, "UNDO %u", static_cast<unsigned>(f.undoDepth));
            b.button(TuneButton::Undo, tuneCell(L::kRow2, 240, 309), f.undoDepth > 0 ? t : "UNDO",
                     idle && f.undoDepth > 0);
            b.button(TuneButton::Reset, tuneCell(L::kRow2, 320, 389), "RESET", idle && m.changed);
            b.button(TuneButton::Copy, tuneCell(L::kRow2, 400, 469), "COPY", idle && live);
            b.button(TuneButton::StringCheck, tuneCell(L::kRow3, 0, 89), "STRING",
                     idle && motor && pos && m.units == Units::Inches);
            // STOP is NEVER greyed — not even while a new layout arms: a reflexive tap on it only stops
            // everything, and the page that follows GO (a test starting) must be stoppable at once.
            b.button(TuneButton::Stop, tuneCell(L::kRow3, 109, 349), "STOP", true, ui::Tone::Red);
            b.button(TuneButton::Exit, tuneCell(L::kRow3, 369, 479), "EXIT", idle);
            return b.view();
        }
        case TunePage::Confirm: {
            detail::TuneViewBuilder b{TunePage::Confirm};
            b.line(ui::TextSize::Large, 8, L::kLargeTitleY, ui::Tone::Amber, "FIRST MOTION");
            std::snprintf(t, sizeof t, "'%.23s' moves for the first time", f.m.name != nullptr ? f.m.name : "");
            b.line(ui::TextSize::Small, 8, L::kBodyY, ui::Tone::White, t);
            std::snprintf(t, sizeof t, "this session, by %.20s.", f.confirmWhat != nullptr ? f.confirmWhat : "");
            b.line(ui::TextSize::Small, 8, L::kBodyY + 16, ui::Tone::White, t);
            b.line(ui::TextSize::Small, 8, L::kBodyY + 40, ui::Tone::Amber, "Hands, cables and the string clear of it.");
            b.line(ui::TextSize::Small, 8, L::kBodyY + 56, ui::Tone::White, "ANY controller button STOPS it.");
            b.line(ui::TextSize::Small, 8, L::kBodyY + 80, ui::Tone::Grey, "Asked once per mechanism each time Tune starts.");
            b.button(TuneButton::Go, tuneCell(L::kRow3, 0, 229), "GO", arm, ui::Tone::Green);
            b.button(TuneButton::Back, tuneCell(L::kRow3, 249, 479), "BACK", arm);
            return b.view();
        }
        case TunePage::Report: {
            detail::TuneViewBuilder b{TunePage::Report};
            const TuneReport& r = f.report;
            std::snprintf(t, sizeof t, "TEST %s '%.23s'", r.test == TuneTest::B ? "B" : "A",
                          f.m.name != nullptr ? f.m.name : "");
            b.line(ui::TextSize::Small, 0, L::kTitleY, ui::Tone::Blue, t);
            const bool ok = r.verdict != nullptr && std::strcmp(r.verdict, "SUCCEEDED") == 0;
            std::snprintf(t, sizeof t, "to %.2f %.3s: %.11s in %.2f s (limit %.2f s)", detail::tuneBounded(r.target),
                          r.unit != nullptr ? r.unit : "", r.verdict != nullptr ? r.verdict : "", r.seconds, r.bound);
            b.line(ui::TextSize::Small, 0, L::kStateY, ok ? ui::Tone::Green : ui::Tone::Red, t);
            if (r.stoppedBy != nullptr && r.stoppedBy[0] != '\0') {
                std::snprintf(t, sizeof t, "stopped by %.36s", r.stoppedBy);
                b.line(ui::TextSize::Small, 0, L::kSafetyY, ui::Tone::Amber, t);
            }
            std::int16_t y = 52;
            const bool power = r.kind == Kind::Power;  // a power run has no settle and no hold: n/a, unsaid
            if (r.overshoot >= 0.0) {
                std::snprintf(t, sizeof t, "overshoot %.3f %.3s past the target", detail::tuneBounded(r.overshoot),
                              r.unit != nullptr ? r.unit : "");
                b.line(ui::TextSize::Small, 0, y, ui::Tone::White, t);
                y = static_cast<std::int16_t>(y + 16);
            }
            if (!power) {
                if (r.settle >= 0.0) {
                    std::snprintf(t, sizeof t, "settled in %.2f s after first reaching it", r.settle);
                } else {
                    std::snprintf(t, sizeof t, "%s", "never settled inside \"reached\"");
                }
                b.line(ui::TextSize::Small, 0, y, ui::Tone::White, t);
                y = static_cast<std::int16_t>(y + 16);
            }
            if (r.hasFinalError) {
                std::snprintf(t, sizeof t, "final error %+.3f %.3s", detail::tuneBounded(r.finalError),
                              r.unit != nullptr ? r.unit : "");
                b.line(ui::TextSize::Small, 0, y, ui::Tone::White, t);
                y = static_cast<std::int16_t>(y + 16);
            }
            // the hold: its number, or WHY there is none (never the n/a sentinel as a number)
            if (power) {
                std::snprintf(t, sizeof t, "peak %.2f A", r.peakAmps);
            } else if (r.holdAmps >= 0.0) {
                std::snprintf(t, sizeof t, "peak %.2f A, hold %.2f A (mean, 1 s after)", r.peakAmps, r.holdAmps);
            } else if (r.holdEnded) {
                std::snprintf(t, sizeof t, "peak %.2f A; hold ENDED (the stall tripped on it)", r.peakAmps);
            } else if (r.holdStopped) {
                std::snprintf(t, sizeof t, "peak %.2f A; hold n/a: stopped in its second", r.peakAmps);
            } else if (ok) {
                std::snprintf(t, sizeof t, "peak %.2f A (no hold measured)", r.peakAmps);
            } else {
                std::snprintf(t, sizeof t, "peak %.2f A (no hold: the move did not succeed)", r.peakAmps);
            }
            b.line(ui::TextSize::Small, 0, y, ui::Tone::White, t);
            y = static_cast<std::int16_t>(y + 16);
            if (r.warnStallNotAboveHold) {
                if (r.holdEnded || r.holdAmps < 0.0) {
                    std::snprintf(t, sizeof t, "WARNING: stall %.2f A trips on the hold: raise it", r.stallAmps);
                } else {
                    std::snprintf(t, sizeof t, "WARNING: stall %.2f A not above the hold %.2f A", r.stallAmps,
                                  r.holdAmps);
                }
                b.line(ui::TextSize::Small, 0, y, ui::Tone::Red, t);
                y = static_cast<std::int16_t>(y + 16);
            }
            if (r.warnCapNotAbovePeak) {
                std::snprintf(t, sizeof t, "WARNING: cap %.2f A not above the peak %.2f A", r.capAmps, r.peakAmps);
                b.line(ui::TextSize::Small, 0, y, ui::Tone::Red, t);
                y = static_cast<std::int16_t>(y + 16);
            }
            b.line(ui::TextSize::Small, 0, static_cast<std::int16_t>(L::kRow3.y0 - 20), ui::Tone::Grey,
                   "the terminal and /usd/tune_log.txt have it too");
            // STOP where the TUNE page's STOP is, and NEVER greyed: the page flips here as a test ends, so a
            // reflexive tap meant for STOP lands on STOP (never on AGAIN, which would start the move again)
            b.button(TuneButton::Again, tuneCell(L::kRow3, 0, 89), "AGAIN", arm);
            b.button(TuneButton::Stop, tuneCell(L::kRow3, 109, 349), "STOP", true, ui::Tone::Red);
            b.button(TuneButton::Back, tuneCell(L::kRow3, 369, 479), "BACK", arm);
            return b.view();
        }
        case TunePage::StringCheck: {
            detail::TuneViewBuilder b{TunePage::StringCheck};
            b.line(ui::TextSize::Large, 8, L::kLargeTitleY, ui::Tone::Blue, "STRING CHECK");
            b.line(ui::TextSize::Small, 8, L::kBodyY, ui::Tone::White, "Not built yet: the snapped-string watch and");
            b.line(ui::TextSize::Small, 8, L::kBodyY + 16, ui::Tone::White, "this page come after the first competition.");
            b.line(ui::TextSize::Small, 8, L::kBodyY + 40, ui::Tone::Amber, "Today a snapped string is NOT detected.");
            b.button(TuneButton::Back, tuneCell(L::kRow3, 249, 479), "BACK", arm);
            return b.view();
        }
        case TunePage::Exited: {
            detail::TuneViewBuilder b{TunePage::Exited};
            b.line(ui::TextSize::Large, 8, L::kLargeTitleY, ui::Tone::Green, "EXITED");
            std::snprintf(t, sizeof t, "%d motor limit%s written back to the robot file's.", f.limitsWritten,
                          f.limitsWritten == 1 ? "" : "s");
            b.line(ui::TextSize::Small, 8, L::kBodyY, ui::Tone::White, t);
            b.line(ui::TextSize::Small, 8, L::kBodyY + 16, ui::Tone::White, "Every mechanism is released. You may stop");
            b.line(ui::TextSize::Small, 8, L::kBodyY + 32, ui::Tone::White, "the program now: live numbers are not kept.");
            std::int16_t ey = static_cast<std::int16_t>(L::kBodyY + 56);
            if (f.copiedAtExit > 0) {
                std::snprintf(t, sizeof t, "EXIT COPIED %d mechanism%s changed numbers to", f.copiedAtExit,
                              f.copiedAtExit == 1 ? "'s" : "s'");
                b.line(ui::TextSize::Small, 8, ey, ui::Tone::Amber, t);
                b.line(ui::TextSize::Small, 8, static_cast<std::int16_t>(ey + 16), ui::Tone::Amber,
                       "the terminal and /usd/tune_log.txt: put them");
                b.line(ui::TextSize::Small, 8, static_cast<std::int16_t>(ey + 32), ui::Tone::Amber,
                       "into the robot file by hand.");
                ey = static_cast<std::int16_t>(ey + 48);
            } else {
                b.line(ui::TextSize::Small, 8, ey, ui::Tone::White, "Every change was COPIED already (or none made).");
                ey = static_cast<std::int16_t>(ey + 16);
            }
            if (f.resetAtExit > 0) {
                std::snprintf(t, sizeof t, "%d RESET to the file: a signature >= its cap.", f.resetAtExit);
                b.line(ui::TextSize::Small, 8, ey, ui::Tone::Red, t);
            }
            b.button(TuneButton::TuneAgain, tuneCell(L::kRow3, 249, 479), "TUNE AGAIN", arm);
            return b.view();
        }
        case TunePage::Stopped: {
            detail::TuneViewBuilder b{TunePage::Stopped};
            b.line(ui::TextSize::Large, 8, L::kLargeTitleY, ui::Tone::Red, "STOPPED");
            b.line(ui::TextSize::Small, 8, L::kBodyY, ui::Tone::White, "A program error: every mechanism released.");
            char s1[ui::kSmallCols + 1];
            char s2[ui::kSmallCols + 1];
            detail::tuneWrap(f.stoppedWhy, s1, s2, ui::kSmallCols);
            b.line(ui::TextSize::Small, 0, L::kBodyY + 24, ui::Tone::Amber, s1);
            b.line(ui::TextSize::Small, 0, L::kBodyY + 40, ui::Tone::Amber, s2);
            b.line(ui::TextSize::Small, 8, L::kBodyY + 64, ui::Tone::White, "Stop the program and start it again.");
            return b.view();
        }
    }
    return detail::TuneViewBuilder{f.page}.view();
}

/// The button an accepted touch at (x, y) hits on `v` — only an ARMED one (ui::hitTest) — or false.
[[nodiscard]] inline bool tuneHit(const TuneView& v, int x, int y, TuneButton& out) noexcept {
    const int i = ui::hitTest(v.buttons.data(), v.buttonCount, x, y);
    if (i < 0) {
        return false;
    }
    out = v.buttons[static_cast<std::size_t>(i)].id;
    return true;
}

}  // namespace shulib::mechanism
