#pragma once
//
// competition_screen.hpp -- "shulib Competition"'s SCREEN as pure models (chunk COMP-b, 2026-09-30):
// WHO paints in each phase (the screen owner), WHEN a painter repaints whole, and WHAT each of the
// program's own pages says. The program only paints what these return (through the shared painters
// of src/common/screen_paint.hpp) and hands in the facts it read; the host suite walks every page
// and every handover (test/competition_screen_test.cpp).
//
// ═══ ONE PAINTER AT A TIME: THE SCREEN OWNER (the COMP brief's B20, RC3; the coordinator's ND4) ═════
// Two tasks paint, never at once, and PROS's phase tasks paint nothing:
//   * the AUTON TASK paints the MENU after the boot and in competition_initialize() and disabled()
//     (it polls the touch screen there -- AutonMenu's view() and touch() belong to ONE task,
//     auton_menu.hpp), and the AUTON STATUS in autonomous() (the status page before runAuton, the
//     result after; nothing during a run);
//   * the OPCONTROL TASK paints the DRIVER page from the moment it takes the screen: after the auton
//     task's terminal word, or -- when B2's constant passes first -- with AUTON ENDING (then the auton
//     task's result paint is skipped, and the result is only logged);
//   * competition_initialize(), disabled() and autonomous() hand the screen over and paint NOTHING:
//     PROS deletes them mid-statement on a mode change (A4 register HA-161 (a)).
// The owner is ONE atomic (ScreenOwnerCell). A phase function's handover always lands (handTo); the
// program's own handovers are compare-exchanges (handOver), so a stale one never takes the screen
// back from a newer owner -- the driver loop that ends hands the screen to the menu only if nobody
// took it first. Every handover moves a GENERATION on, so the task that takes the screen knows it
// did, and paints the frame WHOLE (OwnerTracker).
//
// ═══ WHY A TAKE PAINTS TWICE ════════════════════════════════════════════════════════════
// A handover does not wait for the old owner to finish a paint: a painter PROS deleted mid-paint
// would otherwise hold the screen for ever. So a paint the old owner had in flight can land just
// after the new owner's first whole paint; the new owner paints whole AGAIN kRetakeRepaintSeconds
// later (one 5 Hz refresh), which covers it. Each painter checks the owner before every paint.
//
// PROS-free, header-only. NOTHING HERE HAS RUN ON A ROBOT (worksheet Station K).

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <iterator>
#include <string_view>

#include "common/driver_panel.hpp"
#include "programs/competition_run.hpp"
#include "shulib/competition/auton_menu.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/competition/match_runner.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/ui/touch_panel.hpp"

namespace shulib::bench {

// ═══ THE SCREEN OWNER ═══════════════════════════════════════════════════════════════════════

/// Who paints (file banner).
enum class ScreenOwner : std::uint8_t {
    Menu = 0,         ///< the auton task: the menu (or, when autons are refused, why)
    AutonStatus = 1,  ///< the auton task: the auton's status, then its result
    Driver = 2,       ///< the opcontrol task: the driver page (or AUTON ENDING)
};

/// The owner as a word for the log.
[[nodiscard]] constexpr const char* screenOwnerWord(ScreenOwner o) noexcept {
    switch (o) {
        case ScreenOwner::Menu: return "the menu";
        case ScreenOwner::AutonStatus: return "the auton status";
        case ScreenOwner::Driver: return "the driver page";
    }
    return "unknown";
}

/// THE ATOMIC SCREEN OWNER (file banner): who holds the screen, and a generation that moves on at
/// every handover. Any task may call any member.
class ScreenOwnerCell {
public:
    /// What a painter reads, in ONE load: the owner and the handover count.
    struct Snapshot {
        ScreenOwner owner = ScreenOwner::Menu;  ///< who holds the screen
        std::uint32_t generation = 0;           ///< handovers since construction
    };

    /// The owner and the generation, read together.
    [[nodiscard]] Snapshot snapshot() const noexcept { return unpack(word_.load()); }

    /// True when `who` holds the screen now -- the check before every paint ("a paint from a
    /// non-owner is refused").
    [[nodiscard]] bool holds(ScreenOwner who) const noexcept { return snapshot().owner == who; }

    /// A PHASE FUNCTION's handover: the screen goes to `to` whoever holds it, and the generation
    /// moves on (so `to`'s painter paints whole, even when it held the screen already). Returns the
    /// previous owner.
    ScreenOwner handTo(ScreenOwner to) noexcept {
        std::uint32_t w = word_.load();
        while (!word_.compare_exchange_weak(w, pack(to, unpack(w).generation + 1))) {
        }
        return unpack(w).owner;
    }

    /// A PROGRAM's conditional handover: from `from` to `to` only if `from` holds the screen now
    /// (compare-exchange) -- false, and nothing changed, when someone took it first.
    bool handOver(ScreenOwner from, ScreenOwner to) noexcept {
        std::uint32_t w = word_.load();
        for (;;) {
            if (unpack(w).owner != from) {
                return false;
            }
            if (word_.compare_exchange_weak(w, pack(to, unpack(w).generation + 1))) {
                return true;
            }
        }
    }

private:
    [[nodiscard]] static constexpr std::uint32_t pack(ScreenOwner o, std::uint32_t g) noexcept {
        return (g << 2) | static_cast<std::uint32_t>(o);
    }
    [[nodiscard]] static constexpr Snapshot unpack(std::uint32_t w) noexcept {
        return Snapshot{static_cast<ScreenOwner>(w & 3u), w >> 2};
    }
    std::atomic<std::uint32_t> word_{pack(ScreenOwner::Menu, 0)};
};

/// A take's second whole paint comes this long after the first (file banner): one 5 Hz refresh.
inline constexpr double kRetakeRepaintSeconds = 0.2;
/// Live text repaints at most this often (PROS leaks every print).
inline constexpr double kLiveRepaintSeconds = 0.2;

/// What a painter does this tick.
enum class PaintAction : std::uint8_t {
    None,     ///< not the owner: paint nothing
    Whole,    ///< just took the screen (or the second paint of a take): the frame, whole
    Changes,  ///< the owner still: only what changed, at most every kLiveRepaintSeconds
};

/// One painter's memory of the owner (file banner): Whole on a take and again
/// kRetakeRepaintSeconds later, Changes while it keeps the screen, None when it does not hold it.
class OwnerTracker {
public:
    /// Decide this tick's paint for the painter `mine`, from ONE snapshot, at robot-clock `now`.
    [[nodiscard]] PaintAction next(ScreenOwnerCell::Snapshot s, ScreenOwner mine,
                                   double now) noexcept {
        if (s.owner != mine) {
            held_ = false;
            return PaintAction::None;
        }
        if (!held_ || s.generation != generation_) {
            held_ = true;
            generation_ = s.generation;
            againAt_ = now + kRetakeRepaintSeconds;
            return PaintAction::Whole;
        }
        if (againAt_ >= 0.0 && now >= againAt_) {
            againAt_ = -1.0;
            return PaintAction::Whole;
        }
        return PaintAction::Changes;
    }

private:
    bool held_ = false;
    std::uint32_t generation_ = 0;
    double againAt_ = -1.0;
};

// ═══ THE MENU PHASE, ONE IDLE TICK ═══════════════════════════════════════════════════════════
// The auton task reads the touch screen EVERY idle tick, so the menu's release edge sees each
// release exactly once — and decides here what that read may do. The rules, each pinned by
// test/competition_screen_test.cpp:
//   * a release is ACTED ON only against the selector frame that is ON THE SCREEN and still ours —
//     never on a take (the frame is being painted now), never while another task holds the screen,
//     never while autons are refused: those reads are CONSUMED against an empty frame, so a tap made
//     during driver control, or on a refusal page, can never become a choice later;
//   * a take paints WHOLE (the selector, or — when autons are refused — the autons-refused page);
//   * a refusal that arrives while the selector is up (a program error caught on the idle loop)
//     replaces the selector AT ONCE: a menu must never invite a choice that will be refused.

/// What the menu phase does this tick.
struct MenuPhaseStep {
    bool consumeBlind = false;    ///< read the touch against an EMPTY frame: consumed, acts on nothing
    bool paintRefused = false;    ///< paint the autons-refused page, whole
    bool paintMenuWhole = false;  ///< paint the selector's frame whole (it becomes the drawn frame)
    bool actOnDrawn = false;      ///< the selector is up and ours: this read against the drawn frame
    bool menuUpAfter = false;     ///< the selector is on the screen after this tick
};

/// The menu phase's decision (above) from this tick's paint action, whether autons are refused,
/// and whether the selector was on the screen before this tick. PURE.
[[nodiscard]] constexpr MenuPhaseStep menuPhaseStep(PaintAction action, bool refused,
                                                    bool menuUp) noexcept {
    MenuPhaseStep s{};
    if (action == PaintAction::None) {
        s.consumeBlind = true;  // another task holds the screen
        return s;
    }
    if (refused) {
        s.consumeBlind = true;
        s.paintRefused = action == PaintAction::Whole || menuUp;  // a take, or it just arrived
        return s;
    }
    if (action == PaintAction::Whole || !menuUp) {
        s.consumeBlind = true;
        s.paintMenuWhole = true;
        s.menuUpAfter = true;
        return s;
    }
    s.actOnDrawn = true;
    s.menuUpAfter = true;
    return s;
}

// ═══ THE PAGES ═══════════════════════════════════════════════════════════════════════════

/// The most SMALL lines one of the program's pages carries.
inline constexpr std::size_t kCompetitionMaxLines = 8;

/// Everything one of the program's own pages says (the menu is competition::AutonMenu's view; a
/// refusal is the shared refusal screen). No buttons: nothing on these pages is touched.
struct CompetitionView {
    ui::Tone bar = ui::Tone::Blue;                           ///< the bar's colour
    competition::MenuLine header{};                          ///< the bar's text, left (MEDIUM)
    competition::MenuLine barRight{};                        ///< the bar's text, right (SMALL)
    competition::MenuLine big{};                             ///< one LARGE line, when `hasBig`
    bool hasBig = false;                                     ///< the page has a big word
    competition::MenuLine lines[kCompetitionMaxLines]{};     ///< SMALL lines
    std::uint8_t lineCount = 0;                              ///< how many of `lines` are real
    std::uint8_t droppedLines = 0;  ///< lines a page tried to add past the eight (must be 0)
    std::uint8_t unsafeLines = 0;   ///< lines over budget / not screen-safe BEFORE the copy (0)
};

// ── layout (every number an ESTIMATE, A4 register HA-136) ───────────────────────────────────
/// The bar's right-hand text (the BUILD stamp): "shulib Drive"'s place for it ("shulib Teleop"'s
/// until T4 Part 3).
inline constexpr std::int16_t kCompetitionBarRightX = 340;
/// The big word's top, under the bar.
inline constexpr std::int16_t kCompetitionBigY = 34;
/// A page's first SMALL line under a big word: eight lines end at y 200.
inline constexpr std::int16_t kCompetitionLineY0 = 72;
/// The driver page's first row (no big word): "shulib Drive"'s ("shulib Teleop"'s until T4 Part 3).
inline constexpr std::int16_t kCompetitionDriverRowY0 = 32;
static_assert(kCompetitionLineY0 + ui::kSmallHeight * static_cast<std::int16_t>(kCompetitionMaxLines)
                  <= ui::kPanelLastRow,
              "a page's eight lines must end above the rows VEXos keeps");

// ── the page literals, checked where they are written ────────────────────────────────────────
inline constexpr std::string_view kCompetitionName = "shulib COMPETITION";
inline constexpr std::string_view kBigRunning = "AUTON RUNNING";
inline constexpr std::string_view kBigEnding = "AUTON ENDING";
inline constexpr std::string_view kLineOwnsRobot = "the robot is the auton's until it ends";
inline constexpr std::string_view kLineEndsAtOnce = "a disable or the period's end stops it at once";
inline constexpr std::string_view kLineNoneChosen = "no auton was chosen: nothing moves";
inline constexpr std::string_view kLineChooseOne = "choose one on this menu while disabled";
inline constexpr std::string_view kLineTable1 = "this boot used the TYPED TABLE: its front";
inline constexpr std::string_view kLineTable2 = "may be the OTHER end -- run shulib Calibrate";
inline constexpr std::string_view kLineError1 = "a program error earlier on this boot";
inline constexpr std::string_view kLineError2 = "restart the program to run autons again";
inline constexpr std::string_view kLineDriverRuns = "driver control still runs";
inline constexpr std::string_view kLineLate1 = "autonomous was over when the auton task served it";
inline constexpr std::string_view kLineThrew = "a program error -- the robot is stopped";
inline constexpr std::string_view kLineRefusedLater = "autons are REFUSED until a restart";
inline constexpr std::string_view kLineNothingMoved = "nothing moved";
inline constexpr std::string_view kLineDriverNext = "driver control starts when the field says";
inline constexpr std::string_view kLineEnding1 = "driver control waits for the auton task:";
inline constexpr std::string_view kLineEnding3 = "a wait the guard cannot cut runs to its end";
inline constexpr std::string_view kLineEnding4 = "(Chassis::wait, Routine::pause) -- autons";
inline constexpr std::string_view kLineEnding5 = "should use guard.pause() / guard.waitFor()";
inline constexpr std::string_view kNoFaultsThisRun = "faults this run: none";
static_assert(ui::screenText(kCompetitionName, ui::kMediumCols)
                  && ui::screenText(kBigRunning, ui::kLargeCols)
                  && ui::screenText(kBigEnding, ui::kLargeCols)
                  && ui::screenText(kLineOwnsRobot, ui::kSmallCols)
                  && ui::screenText(kLineEndsAtOnce, ui::kSmallCols)
                  && ui::screenText(kLineNoneChosen, ui::kSmallCols)
                  && ui::screenText(kLineChooseOne, ui::kSmallCols)
                  && ui::screenText(kLineTable1, ui::kSmallCols)
                  && ui::screenText(kLineTable2, ui::kSmallCols)
                  && ui::screenText(kLineError1, ui::kSmallCols)
                  && ui::screenText(kLineError2, ui::kSmallCols)
                  && ui::screenText(kLineDriverRuns, ui::kSmallCols)
                  && ui::screenText(kLineLate1, ui::kSmallCols)
                  && ui::screenText(kLineThrew, ui::kSmallCols)
                  && ui::screenText(kLineRefusedLater, ui::kSmallCols)
                  && ui::screenText(kLineNothingMoved, ui::kSmallCols)
                  && ui::screenText(kLineDriverNext, ui::kSmallCols)
                  && ui::screenText(kLineEnding1, ui::kSmallCols)
                  && ui::screenText(kLineEnding3, ui::kSmallCols)
                  && ui::screenText(kLineEnding4, ui::kSmallCols)
                  && ui::screenText(kLineEnding5, ui::kSmallCols)
                  && ui::screenText(kNoFaultsThisRun, ui::kSmallCols),
              "a competition-page literal does not fit its budget, or is not screen-safe");

namespace detail {
/// One line at its place, counted when it is over its budget BEFORE the safety copy cuts it.
inline void setCompetitionLine(CompetitionView& v, competition::MenuLine& l, std::int16_t x,
                               std::int16_t y, ui::TextSize size, ui::Tone tone,
                               std::string_view text) noexcept {
    l.x = x;
    l.y = y;
    l.size = size;
    l.tone = tone;
    if (!ui::screenText(text, ui::budgetFor(size))) {
        ++v.unsafeLines;
    }
    ui::copyScreenText(l.text, sizeof l.text, text, ui::budgetFor(size));
}

/// The next SMALL line, from `y0`; counted (never silent) past the eighth.
inline void addCompetitionLine(CompetitionView& v, std::int16_t y0, ui::Tone tone,
                               std::string_view text) noexcept {
    if (v.lineCount >= std::size(v.lines)) {
        ++v.droppedLines;
        return;
    }
    setCompetitionLine(v, v.lines[v.lineCount], 6,
                       static_cast<std::int16_t>(y0 + ui::kSmallHeight * v.lineCount),
                       ui::TextSize::Small, tone, text);
    ++v.lineCount;
}

/// Up to `maxLines` SMALL lines of `text`, broken at spaces (a word longer than a line is cut at
/// the budget) -- for sentences the page did not write: a thrown precondition, runAuton's refusal.
inline void addCompetitionWrapped(CompetitionView& v, std::int16_t y0, ui::Tone tone,
                                  std::string_view text, int maxLines) noexcept {
    int used = 0;
    while (!text.empty() && used < maxLines) {
        std::size_t take = text.size();
        if (take > ui::kSmallCols) {
            take = ui::kSmallCols;
            const std::size_t space = text.substr(0, ui::kSmallCols + 1).rfind(' ');
            if (space != std::string_view::npos && space > 0) {
                take = space;
            }
        }
        addCompetitionLine(v, y0, tone, text.substr(0, take));
        text.remove_prefix(take);
        while (!text.empty() && text.front() == ' ') {
            text.remove_prefix(1);
        }
        ++used;
    }
}

/// The bar: `header` left (MEDIUM, white), `right` at the bar's right (SMALL), in `tone`.
inline void setCompetitionBar(CompetitionView& v, ui::Tone tone, std::string_view header,
                              std::string_view right) noexcept {
    v.bar = tone;
    setCompetitionLine(v, v.header, 6, 4, ui::TextSize::Medium, ui::Tone::White, header);
    setCompetitionLine(v, v.barRight, kCompetitionBarRightX, 7, ui::TextSize::Small,
                       ui::Tone::White, right);
}
}  // namespace detail

/// The tone a result word is painted in: FINISHED green, NONE grey, ABANDONED, CUT BY DEADLINE and
/// TOO LATE amber (the field or the clock ended it -- not a fault of the program's), DONE, n FAILED
/// amber (STEP1-A: the routine ran to its end, carrying on past n failed steps), every other word
/// red.
[[nodiscard]] inline ui::Tone competitionResultTone(std::string_view word) noexcept {
    // STEP1-A: every "DONE, n FAILED" word is amber -- matched by its prefix, the one place a prefix
    // match is right: each such word is generated by competitionDoneWord, nowhere else.
    return word == "FINISHED" ? ui::Tone::Green
           : word == "NONE"   ? ui::Tone::Grey
           : (word == "ABANDONED" || word == "CUT BY DEADLINE" || word == "TOO LATE"
              || word.starts_with("DONE, "))
               ? ui::Tone::Amber
               : ui::Tone::Red;
}

namespace detail {
/// The lines a skipped request's pages share (status and result say the same).
inline void addCompetitionSkipLines(CompetitionView& v, const CompetitionRunRecord& rec) noexcept {
    switch (rec.skip) {
        case CompetitionSkip::NoneChosen:
            addCompetitionLine(v, kCompetitionLineY0, ui::Tone::White, kLineNoneChosen);
            addCompetitionLine(v, kCompetitionLineY0, ui::Tone::Grey, kLineChooseOne);
            return;
        case CompetitionSkip::BootFromTable:
            addCompetitionLine(v, kCompetitionLineY0, ui::Tone::Red, kLineTable1);
            addCompetitionLine(v, kCompetitionLineY0, ui::Tone::Red, kLineTable2);
            addCompetitionLine(v, kCompetitionLineY0, ui::Tone::White, kLineDriverRuns);
            return;
        case CompetitionSkip::AfterProgramError:
            addCompetitionLine(v, kCompetitionLineY0, ui::Tone::Red, kLineError1);
            addCompetitionLine(v, kCompetitionLineY0, ui::Tone::Red, kLineError2);
            addCompetitionLine(v, kCompetitionLineY0, ui::Tone::White, kLineDriverRuns);
            return;
        case CompetitionSkip::TooLate: {
            char text[2 * ui::kSmallCols + 1] = "";
            addCompetitionLine(v, kCompetitionLineY0, ui::Tone::Amber, kLineLate1);
            std::snprintf(text, sizeof text, "the status said: %.15s (status 0x%02x)",
                          competitionCauseShort(rec.cause),
                          static_cast<unsigned>(rec.statusAtLatch));
            addCompetitionLine(v, kCompetitionLineY0, ui::Tone::White, text);
            addCompetitionLine(v, kCompetitionLineY0, ui::Tone::White, kLineNothingMoved);
            addCompetitionLine(v, kCompetitionLineY0, ui::Tone::White, kLineDriverRuns);
            return;
        }
        case CompetitionSkip::None:
            return;
    }
}

/// "AUTON: <name>" for the bar.
inline void competitionAutonHeader(char (&out)[2 * ui::kSmallCols + 1], const char* name) noexcept {
    std::snprintf(out, sizeof out, "%s%.29s", competition::kMenuHeaderPrefix.data(), name);
}
}  // namespace detail

/// THE STATUS PAGE (autonomous(), before runAuton -- RC3): what is about to run, or why nothing
/// will. `buildStamp` is the program's "BUILD hh:mm:ss".
[[nodiscard]] inline CompetitionView competitionStatusView(
    const CompetitionRunRecord& rec, const competition::AutonRegistry& registry,
    std::string_view buildStamp) {
    CompetitionView v{};
    char text[2 * ui::kSmallCols + 1] = "";
    detail::competitionAutonHeader(text, detail::competitionEntryName(registry, rec.entry));
    if (rec.skip == CompetitionSkip::None) {
        const competition::AutonEntry& e = registry.at(static_cast<std::size_t>(rec.entry));
        detail::setCompetitionBar(v, ui::Tone::Amber, text, buildStamp);
        detail::setCompetitionLine(v, v.big, 6, kCompetitionBigY, ui::TextSize::Large,
                                   ui::Tone::Amber, kBigRunning);
        v.hasBig = true;
        std::snprintf(text, sizeof text, "%s park T+%.1fs stop T+%.1fs %s",
                      competition::autonKindWord(e.kind), e.endActionAt.value(),
                      e.hardStopAt.value(), competition::startModeWord(e.effectiveStart().mode()));
        detail::addCompetitionLine(v, kCompetitionLineY0, ui::Tone::White, text);
        detail::addCompetitionLine(v, kCompetitionLineY0, ui::Tone::White, kLineOwnsRobot);
        detail::addCompetitionLine(v, kCompetitionLineY0, ui::Tone::Amber, kLineEndsAtOnce);
        return v;
    }
    const char* word = competitionResultWord(rec);  // NONE, TOO LATE or REFUSED for a skip
    const ui::Tone tone = competitionResultTone(word);
    detail::setCompetitionBar(v, tone, text, buildStamp);
    detail::setCompetitionLine(v, v.big, 6, kCompetitionBigY, ui::TextSize::Large, tone, word);
    v.hasBig = true;
    detail::addCompetitionSkipLines(v, rec);
    return v;
}

/// THE RESULT PAGE (autonomous(), after FINISHED, painted only while the auton task still holds the
/// screen -- RC3): the result word big, then how it ended, the end action and the guard's clock,
/// where it was seated, the END estimate, this request's faults, and what comes next.
[[nodiscard]] inline CompetitionView competitionResultView(
    const CompetitionRunRecord& rec, const competition::AutonRegistry& registry,
    std::string_view buildStamp) {
    CompetitionView v{};
    char text[2 * ui::kSmallCols + 1] = "";
    const char* word = competitionResultWord(rec);
    const ui::Tone tone = competitionResultTone(word);
    detail::competitionAutonHeader(text, detail::competitionEntryName(registry, rec.entry));
    detail::setCompetitionBar(v, tone, text, buildStamp);
    detail::setCompetitionLine(v, v.big, 6, kCompetitionBigY, ui::TextSize::Large, tone, word);
    v.hasBig = true;
    if (rec.threw) {
        detail::addCompetitionLine(v, kCompetitionLineY0, ui::Tone::Red, kLineThrew);
        detail::addCompetitionWrapped(v, kCompetitionLineY0, ui::Tone::White, rec.threwWhat, 3);
        detail::addCompetitionLine(v, kCompetitionLineY0, ui::Tone::White, kLineDriverRuns);
        detail::addCompetitionLine(v, kCompetitionLineY0, ui::Tone::Red, kLineRefusedLater);
        return v;
    }
    if (rec.skip != CompetitionSkip::None) {
        detail::addCompetitionSkipLines(v, rec);
        return v;
    }
    if (rec.run.outcome == competition::AutonRunOutcome::Refused) {
        detail::addCompetitionWrapped(v, kCompetitionLineY0, ui::Tone::Red, rec.run.why, 4);
        detail::addCompetitionLine(v, kCompetitionLineY0, ui::Tone::White, kLineNothingMoved);
        return v;
    }
    const sequence::RunGuardReport& g = rec.run.guard;
    // (1) how it ended -- and a request the guard never acted on, in red words
    if (g.abandoned) {
        std::snprintf(text, sizeof text, "ABANDONED: %.15s (status 0x%02x)",
                      competitionCauseShort(rec.cause), static_cast<unsigned>(rec.statusAtLatch));
    } else if (abandonIgnored(rec)) {
        std::snprintf(text, sizeof text, "abandon IGNORED: %.15s",
                      competitionCauseShort(rec.cause));
    } else {
        std::snprintf(text, sizeof text, "%s, %s",
                      competition::autonRunOutcomeWord(rec.run.outcome),
                      g.scoringCut ? "cut at the deadline" : "in time");
    }
    detail::addCompetitionLine(v, kCompetitionLineY0,
                               abandonIgnored(rec) ? ui::Tone::Red : ui::Tone::White, text);
    // (2) the end action and the guard's clock: CUT (never FAILED) when the abandon landed in it.
    // (The pace count is the log's line 2 only: with it this line passed the SMALL budget at the
    // worst case -- test/competition_screen_test.cpp (S10).)
    std::snprintf(text, sizeof text, "end action %s; scoring ended T+%.2fs",
                  !g.endActionRan       ? "skipped"
                  : g.abandoned         ? "CUT"
                  : g.endActionSucceeded ? "succeeded"
                                         : "FAILED",
                  g.scoringEnded.value());
    detail::addCompetitionLine(v, kCompetitionLineY0, ui::Tone::White, text);
    // (3)-(4) where it was seated, and where the estimate says it ended
    std::snprintf(text, sizeof text, "START (%.1f, %.1f) %.1f deg", rec.run.seated.x().value(),
                  rec.run.seated.y().value(), rec.run.seated.heading().degrees());
    detail::addCompetitionLine(v, kCompetitionLineY0, ui::Tone::White, text);
    std::snprintf(text, sizeof text, "END   (%.1f, %.1f) %.1f deg%s", rec.endEstimate.x().value(),
                  rec.endEstimate.y().value(), rec.endEstimate.heading().degrees(),
                  rec.endLive ? "" : " NOT LIVE");
    detail::addCompetitionLine(v, kCompetitionLineY0,
                               rec.endLive ? ui::Tone::Green : ui::Tone::Amber, text);
    // (5) THIS request's faults (never the boot's)
    if (rec.faultsAfter == rec.faultsBefore) {
        detail::addCompetitionLine(v, kCompetitionLineY0, ui::Tone::White, kNoFaultsThisRun);
    } else {
        std::snprintf(text, sizeof text, "faults this run: %d, last %.20s",
                      rec.faultsAfter - rec.faultsBefore, diag::faultCodeName(rec.lastFault));
        detail::addCompetitionLine(v, kCompetitionLineY0, ui::Tone::Red, text);
    }
    // (6) what comes next
    detail::addCompetitionLine(v, kCompetitionLineY0, ui::Tone::Grey, kLineDriverNext);
    return v;
}

/// THE AUTON ENDING PAGE (opcontrol(), B2's constant passed before the terminal word): which auton
/// is still running, that it was not handed back within `waitedSeconds`, and why that can happen at
/// all. Painted ONCE, so its words are a fact that stays true while the wait goes on -- never a
/// running count that would freeze on the screen (the log line carries the time).
[[nodiscard]] inline CompetitionView competitionEndingView(const char* autonName,
                                                           double waitedSeconds,
                                                           std::string_view buildStamp) {
    CompetitionView v{};
    char text[2 * ui::kSmallCols + 1] = "";
    detail::setCompetitionBar(v, ui::Tone::Amber, kCompetitionName, buildStamp);
    detail::setCompetitionLine(v, v.big, 6, kCompetitionBigY, ui::TextSize::Large, ui::Tone::Amber,
                               kBigEnding);
    v.hasBig = true;
    detail::addCompetitionLine(v, kCompetitionLineY0, ui::Tone::White, kLineEnding1);
    std::snprintf(text, sizeof text, "still running: '%.29s'",
                  autonName != nullptr ? autonName : "");
    detail::addCompetitionLine(v, kCompetitionLineY0, ui::Tone::Amber, text);
    std::snprintf(text, sizeof text, "not handed back within %.2f s", waitedSeconds);
    detail::addCompetitionLine(v, kCompetitionLineY0, ui::Tone::Amber, text);
    detail::addCompetitionLine(v, kCompetitionLineY0, ui::Tone::Grey, kLineEnding3);
    detail::addCompetitionLine(v, kCompetitionLineY0, ui::Tone::Grey, kLineEnding4);
    detail::addCompetitionLine(v, kCompetitionLineY0, ui::Tone::Grey, kLineEnding5);
    return v;
}

/// What the driver page's AUTON row says: the last request's word and auton, or why autons are
/// refused -- ND1's and ND2's sentence on the screen during driver control (the log has it all).
[[nodiscard]] inline PanelRow competitionAutonRow(const CompetitionRunRecord& last,
                                                  const competition::AutonRegistry& registry,
                                                  bool programError, bool bootFromRecord) {
    PanelRow r;
    char text[2 * ui::kSmallCols + 1] = "";
    if (programError) {
        std::snprintf(text, sizeof text, "%s", "AUTONS REFUSED: a program error -- restart");
        r.tone = ui::Tone::Red;
    } else if (!bootFromRecord) {
        std::snprintf(text, sizeof text, "%s", "AUTONS REFUSED: typed-table boot -- Calibrate");
        r.tone = ui::Tone::Red;
    } else if (last.request == 0) {
        std::snprintf(text, sizeof text, "%s", "auton: none requested on this boot");
        r.tone = ui::Tone::Grey;
    } else {
        const char* word = competitionResultWord(last);
        std::snprintf(text, sizeof text, "auton %.15s: %.27s", word,
                      detail::competitionEntryName(registry, last.entry));
        r.tone = competitionResultTone(word);
    }
    ui::copyScreenText(r.text, sizeof r.text, text, ui::kSmallCols);
    return r;
}

/// What the driver page shows, read by the program each refresh (it owns the robot by then). Every
/// row but the AUTON row is src/common/driver_panel.hpp's (shared with "shulib Drive", never
/// copied).
struct CompetitionDriverFacts {
    PanelRow protection{};           ///< protectionRowText() -- the drive protections' state
    PanelRow auton{};                ///< competitionAutonRow()
    PanelRow controller{};           ///< controllerRowText()
    const char* faultRow = "";       ///< faultRowText()
    bool faultRed = false;           ///< the fault latch holds a fault
    const char* clawRow = "";        ///< clawRowText()
    bool clawAmber = false;          ///< a claw ABSENT, or a claim released at this entry (FREED)
    std::string_view buildStamp{};   ///< "BUILD hh:mm:ss"
};

/// THE DRIVER PAGE (opcontrol(), after the handover -- B11 as T4 Part 1a replaced it): the header,
/// the PROTECTION row (T4 Part 2b, added in Part 3's late step: FIRST, as on "shulib Drive"'s MAIN
/// -- it is what explains a drive that stopped), the AUTON row, the CONTROLLER row, the ONE fault
/// row and the CLAW row.
[[nodiscard]] inline CompetitionView competitionDriverView(const CompetitionDriverFacts& f) {
    CompetitionView v{};
    detail::setCompetitionBar(v, ui::Tone::Blue, kCompetitionName, f.buildStamp);
    detail::addCompetitionLine(v, kCompetitionDriverRowY0, f.protection.tone, f.protection.text);
    detail::addCompetitionLine(v, kCompetitionDriverRowY0, f.auton.tone, f.auton.text);
    detail::addCompetitionLine(v, kCompetitionDriverRowY0, f.controller.tone, f.controller.text);
    detail::addCompetitionLine(v, kCompetitionDriverRowY0,
                               f.faultRed ? ui::Tone::Red : ui::Tone::Green,
                               f.faultRow != nullptr ? f.faultRow : "");
    detail::addCompetitionLine(v, kCompetitionDriverRowY0,
                               f.clawAmber ? ui::Tone::Amber : ui::Tone::White,
                               f.clawRow != nullptr ? f.clawRow : "");
    return v;
}

}  // namespace shulib::bench
