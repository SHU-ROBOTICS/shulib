#pragma once
//
// auton_tester_panel.hpp -- "shulib Auton Tester"'s SCREEN as a pure model (COMP-t, 2026-09-28):
// the pages, what each says, which touches mean something, and when RUN may be touched. The
// program only paints a TesterView and hands in touch samples and the facts it read (the brief's
// R7 and R8); the host suite walks every page (test/auton_tester_panel_test.cpp).
//
// ═══ THE PAGES ══════════════════════════════════════════════════════════════════════
//   PICK      competition::AutonMenu's OWN view -- the selector the competition program shows.
//             Choosing an auton goes to READY; NONE stays.
//   READY     the chosen auton, how the robot booted (D9), the live estimate, why RUN is or is
//             not allowed (R4), the abort's words, the SD log, the faults and the BUILD stamp;
//             < BACK bottom-left, RUN bottom-right.
//   COUNTDOWN 3 s, "STEP BACK"; nothing moves; no buttons (the watch owns every touch).
//   RUNNING   the elapsed time, "ANY BUTTON ABORTS", the live estimate, and ABORTING once latched.
//   RESULT    testerResultWord() big, the run's six lines; < PICK bottom-left, AGAIN bottom-CENTRE.
//
// ═══ NO TAP IS EVER REPLAYED, AND NO NEW BUTTON APPEARS UNDER A FINGER ═════════════════
// Every page entry takes a FRESH release edge at the current release count and restarts the arming
// clock, so a tap is never replayed on the next page and a new page's buttons are dead for
// ui::kArmingDelaySeconds (PICK by emplacing a NEW AutonMenu: it is non-movable and its edge is
// private). RESULT entered with a finger still DOWN (an abort touch held through the end of the
// run) keeps its buttons grey until that finger's release has been consumed -- it acts on nothing
// -- and only then starts the arming clock (the brief's R7; A4 register HA-178 (a) is the belief
// that press_count rises when the finger lands). Buttons live only in the bottom row, and each
// transition puts nothing where the button just pressed was: READY is reached from PICK's middle
// rows, RESULT's PICK and AGAIN are never where RUN is, AGAIN leads to READY (never straight to
// RUN).
//
// ═══ EVERY STRING FITS, AND NONE IS DROPPED ════════════════════════════════════════════
// Every literal is static_asserted against its budget (SMALL 50 / MEDIUM 36 / LARGE 18, A4
// register HA-136), and every formatted field carries a precision. A view COUNTS what it could not
// hold -- a line past its eight (`droppedLines`) and a line that did not pass ui::screenText()
// BEFORE the safety copy cut it (`unsafeLines`) -- so a test fails on a silent cut instead of
// reading the cut text as fine (the first probe's view held 7 lines and dropped READY's 8th
// without a word).
//
// PROS-free, header-only. NOTHING HERE HAS RUN ON A ROBOT (worksheet Station M).

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <iterator>
#include <optional>
#include <string_view>

#include "common/auton_run_log.hpp"
#include "common/screen_paint.hpp"
#include "common/state_screens.hpp"
#include "programs/auton_abort_watch.hpp"
#include "programs/auton_tester_run.hpp"
#include "shulib/competition/auton_menu.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/competition/match_runner.hpp"
#include "shulib/core/check.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/ui/touch_panel.hpp"

namespace shulib::bench {

// competition_get_status()'s bits (kStatusDisabled, kStatusAutonomous, kStatusConnected) are
// src/common/state_screens.hpp's since COMP-b's first step (2026-09-30): the competition program's
// watch reads the same three, and they are defined once, checked against PROS there.

/// Which page is up.
enum class TesterPage : std::uint8_t { Pick, Ready, Countdown, Running, Result };

/// What the screen needs this frame -- read by the program (it owns the robot), never by the model.
struct TesterFacts {
    double now = 0.0;                          ///< the robot clock, seconds
    math::Pose2d estimate{};                   ///< the fused estimate
    bool estimateLive = false;                 ///< qualityClass() != Uninitialized
    bool controllerConnected = false;          ///< the master controller
    std::uint8_t competitionStatus = 0;        ///< competition_get_status()
    AbortCause abortCause = AbortCause::None;  ///< the watch's latch
    bool sdLogging = false;                    ///< the SD run log is open
    bool bootFromRecord = false;               ///< the boot chose the calibration RECORD (D9)
    int faultCount = 0;                        ///< the fault latch's count (the whole boot)
    diag::FaultCode lastFault = diag::FaultCode::None;  ///< its last fault
    const char* buildStamp = "BUILD --:--:--";  ///< the program's "BUILD " __TIME__ (upload check)
};

/// What a touch asks the program to do.
enum class TesterActionKind : std::uint8_t { None, Run };
/// A touch's outcome for the program.
struct TesterAction {
    TesterActionKind kind = TesterActionKind::None;  ///< Run = start the countdown for `entry`
    int entry = -1;                                  ///< the registry index to run
};

/// Which of the tester's own buttons.
enum class TesterButtonId : std::uint8_t { Back, Run, Pick, Again };

/// One of the tester's own buttons.
struct TesterButton {
    TesterButtonId id = TesterButtonId::Back;  ///< which
    ui::Rect rect{};                           ///< where (hit box: ui::hitBox(rect))
    char label[ui::kMediumCols + 1]{};         ///< MEDIUM, screen-safe
    bool armed = false;                        ///< false while greyed (arming, or RUN not allowed)
};

/// The most SMALL lines a page carries (READY uses all eight).
inline constexpr std::size_t kTesterMaxLines = 8;

/// Everything one frame says. On PICK only `menu` is used.
struct TesterView {
    TesterPage page = TesterPage::Pick;             ///< which page
    competition::MenuView menu{};                   ///< PICK: the selector's own view
    competition::MenuLine header{};                 ///< the bar's text (MEDIUM)
    ui::Tone bar = ui::Tone::Blue;                  ///< the bar's colour
    competition::MenuLine big[2]{};                 ///< LARGE/MEDIUM lines (countdown/running/result)
    competition::MenuLine lines[kTesterMaxLines]{};  ///< SMALL lines
    std::uint8_t lineCount = 0;                     ///< how many of `lines` are real
    std::uint8_t bigCount = 0;                      ///< how many of `big` are real
    TesterButton buttons[2]{};                      ///< the page's buttons
    std::uint8_t buttonCount = 0;                   ///< how many are real
    std::uint8_t droppedLines = 0;  ///< lines a page tried to add past kTesterMaxLines (must be 0)
    std::uint8_t unsafeLines = 0;   ///< lines over budget / not screen-safe BEFORE the copy (0)
};

// ── layout (every number an ESTIMATE like every screen number, A4 register HA-136) ─────────────
/// READY's first SMALL line: eight lines at 16 px end at y 158, above the bottom row (y 180).
inline constexpr std::int16_t kTesterReadyLineY0 = 30;
/// RESULT's first SMALL line, under the LARGE word: six lines end at y 168.
inline constexpr std::int16_t kTesterResultLineY0 = 72;
/// RUNNING's first SMALL line, under its two LARGE lines.
inline constexpr std::int16_t kTesterRunningLineY0 = 140;
static_assert(kTesterReadyLineY0 + ui::kSmallHeight * static_cast<std::int16_t>(kTesterMaxLines)
                  < competition::kMenuPrevSlot.y0 - ui::kHitPadY,
              "READY's eight lines must end above the bottom row's padded hit boxes");
static_assert(kTesterResultLineY0 + ui::kSmallHeight * 6
                  < competition::kMenuPrevSlot.y0 - ui::kHitPadY,
              "RESULT's six lines must end above the bottom row's padded hit boxes");

// ── the page literals, checked where they are written ────────────────────────────────────────
inline constexpr std::string_view kReadyHint = "RUN counts 3 s, then it moves. Step back.";
inline constexpr std::string_view kAbortHint = "ANY BUTTON or a touch of the screen aborts.";
inline constexpr std::string_view kNotLive = "estimate NOT LIVE yet (the IMU is calibrating)";
inline constexpr std::string_view kNoController = "NO CONTROLLER: RUN needs it (the abort)";
inline constexpr std::string_view kCompetition = "field/switch/Timed Run on: start from the brain";
inline constexpr std::string_view kDisabledWord = "the robot is DISABLED";
inline constexpr std::string_view kRunAllowed = "READY: touch RUN, then step back";
inline constexpr std::string_view kSdOn = "SD: runs appended to shulib_auton_runs.txt";
// The file READY names is the one both programs append to (src/common/auton_run_log.hpp).
static_assert(kSdOn.ends_with(kAutonRunLogFile), "READY names another file than the run log");
inline constexpr std::string_view kSdOff = "SD: NO CARD -- this boot's runs on serial only";
inline constexpr std::string_view kBootRecord = "BOOT RECORD: forward = the end that led Calibrate";
inline constexpr std::string_view kBootTable = "BOOT TYPED TABLE: its front may be the OTHER end";
inline constexpr std::string_view kNeedsRecord = "RUN needs the calibration RECORD: run Calibrate";
inline constexpr std::string_view kCountdownSub = "ANY BUTTON OR TOUCH ABORTS";
inline constexpr std::string_view kBigAbort = "ANY BUTTON ABORTS";
inline constexpr std::string_view kMeasureTape = "measure the tape now, then PICK or AGAIN";
inline constexpr std::string_view kNothingMoved = "nothing moved: runAuton was never called";
inline constexpr std::string_view kThrewLine = "a program error -- the drive was stopped";
inline constexpr std::string_view kMoreOnSerial = "(the full text is on the serial log)";
inline constexpr std::string_view kPickOrAgain = "then PICK or AGAIN";
inline constexpr std::string_view kNoFaults = "faults this run: none";
inline constexpr std::string_view kBackLabel = "< BACK";
inline constexpr std::string_view kRunLabel = "RUN";
inline constexpr std::string_view kPickLabel = "< PICK";
inline constexpr std::string_view kAgainLabel = "AGAIN";
static_assert(ui::screenText(kReadyHint, ui::kSmallCols) && ui::screenText(kAbortHint, ui::kSmallCols)
              && ui::screenText(kNotLive, ui::kSmallCols)
              && ui::screenText(kNoController, ui::kSmallCols)
              && ui::screenText(kCompetition, ui::kSmallCols)
              && ui::screenText(kDisabledWord, ui::kSmallCols)
              && ui::screenText(kRunAllowed, ui::kSmallCols) && ui::screenText(kSdOn, ui::kSmallCols)
              && ui::screenText(kSdOff, ui::kSmallCols) && ui::screenText(kBootRecord, ui::kSmallCols)
              && ui::screenText(kBootTable, ui::kSmallCols)
              && ui::screenText(kNeedsRecord, ui::kSmallCols)
              && ui::screenText(kMeasureTape, ui::kSmallCols)
              && ui::screenText(kNothingMoved, ui::kSmallCols)
              && ui::screenText(kThrewLine, ui::kSmallCols)
              && ui::screenText(kMoreOnSerial, ui::kSmallCols)
              && ui::screenText(kPickOrAgain, ui::kSmallCols)
              && ui::screenText(kNoFaults, ui::kSmallCols)
              && ui::screenText(kCountdownSub, ui::kMediumCols)
              && ui::screenText(kBigAbort, ui::kLargeCols)
              && ui::screenText(kBackLabel, ui::kMediumCols)
              && ui::screenText(kRunLabel, ui::kMediumCols)
              && ui::screenText(kPickLabel, ui::kMediumCols)
              && ui::screenText(kAgainLabel, ui::kMediumCols),
              "an Auton Tester screen literal does not fit its budget, or is not screen-safe");

/// May RUN be touched (the brief's R4)? VEXos reports plain driver control with no field, switch
/// or Timed Run (a controller-menu start arms a pretend match: HA-128), the boot chose the
/// calibration RECORD (D9: its forward is the end that led Calibrate), a controller is connected
/// (it carries the abort -- D5), and the estimate is live. `why` gets the FIRST failing reason in
/// that order, or kRunAllowed. The page's arming delay is the view's, on top of this.
[[nodiscard]] inline bool runAllowed(const TesterFacts& f, std::string_view* why) noexcept {
    std::string_view w = kRunAllowed;
    bool ok = false;
    if ((f.competitionStatus & kStatusDisabled) != 0) {
        w = kDisabledWord;
    } else if ((f.competitionStatus & (kStatusAutonomous | kStatusConnected)) != 0) {
        w = kCompetition;
    } else if (!f.bootFromRecord) {
        w = kNeedsRecord;
    } else if (!f.controllerConnected) {
        w = kNoController;
    } else if (!f.estimateLive) {
        w = kNotLive;
    } else {
        ok = true;
    }
    if (why != nullptr) {
        *why = w;
    }
    return ok;
}

/// True when two frames show the same page and the same buttons (ids, rectangles, labels, armed,
/// chosen). A difference means a FULL repaint now, so the frame a touch is tested against is always
/// the one on the screen; a text-only difference may wait for the 5 Hz repaint (the brief's R8).
[[nodiscard]] inline bool sameButtons(const TesterView& a, const TesterView& b) noexcept {
    if (a.page != b.page) {
        return false;
    }
    if (a.page == TesterPage::Pick) {
        // The selector's frames: src/common/screen_paint.hpp's sameMenuButtons() since COMP-b (the
        // competition program repaints the same selector by the same rule; moved, never copied).
        return sameMenuButtons(a.menu, b.menu);
    }
    if (a.buttonCount != b.buttonCount) {
        return false;
    }
    for (std::size_t i = 0; i < a.buttonCount; ++i) {
        const TesterButton& x = a.buttons[i];
        const TesterButton& y = b.buttons[i];
        if (x.id != y.id || !(x.rect == y.rect) || x.armed != y.armed
            || std::string_view{x.label} != std::string_view{y.label}) {
            return false;
        }
    }
    return true;
}

/// The screen (file banner). Borrows the registry, which must be complete and outlive it. One task
/// only -- the tester's.
class TesterScreen {
public:
    /// The registry is borrowed for the screen's life. The page starts as PICK; call showPick()
    /// with the current release count before the first view().
    explicit TesterScreen(const competition::AutonRegistry& registry) noexcept
        : registry_{&registry}, edge_{0} {}

    /// Show PICK with a FRESH selector (its edge at `releaseCount`, its arming clock restarted).
    void showPick(std::int32_t releaseCount) {
        menu_.reset();
        menu_.emplace(*registry_, releaseCount);
        enter(TesterPage::Pick, releaseCount);
        entry_ = -1;
    }
    /// Show READY for registry index `entry`.
    void showReady(int entry, std::int32_t releaseCount) {
        SHULIB_PRECONDITION(entry >= 0 && static_cast<std::size_t>(entry) < registry_->size(),
                            "TesterScreen::showReady: no such auton");
        menu_.reset();
        entry_ = entry;
        enter(TesterPage::Ready, releaseCount);
    }
    /// Show the countdown, started at `now` (runTesterOnce's observer).
    void showCountdown(double now) {
        started_ = now;
        enter(TesterPage::Countdown, edgeCount_);
    }
    /// Show the run, started at `now` (runTesterOnce's observer, on the quiet tick).
    void showRunning(double now) {
        started_ = now;
        enter(TesterPage::Running, edgeCount_);
    }
    /// Show the result of `rec` -- its OWN entry (`rec.entry`), never whatever page was up before.
    /// `rec` is borrowed: it must outlive the page (the program's static state). `fingerDown`: a
    /// finger was on the glass at this read (an abort touch still held); its release, whenever it
    /// comes, is consumed and acts on nothing, and the arming clock starts only after it.
    void showResult(const TesterRunRecord& rec, std::int32_t releaseCount, bool fingerDown) {
        SHULIB_PRECONDITION(rec.entry >= 0
                                && static_cast<std::size_t>(rec.entry) < registry_->size(),
                            "TesterScreen::showResult: the record names no auton");
        result_ = &rec;
        entry_ = rec.entry;
        enter(TesterPage::Result, releaseCount);
        liftPending_ = fingerDown;
    }

    /// Which page is up.
    [[nodiscard]] TesterPage page() const noexcept { return page_; }
    /// The entry READY / the countdown / the run / the result are about (-1 on PICK).
    [[nodiscard]] int entry() const noexcept { return entry_; }
    /// True while RESULT waits for a held finger to lift (its buttons stay grey).
    [[nodiscard]] bool waitingForLift() const noexcept { return liftPending_; }

    /// The frame to paint at `f.now` (the arming clock observes it).
    [[nodiscard]] TesterView view(const TesterFacts& f) {
        SHULIB_PRECONDITION(page_ != TesterPage::Pick || menu_.has_value(),
                            "TesterScreen::view: showPick() was never called");
        TesterView v{};
        v.page = page_;
        if (page_ == TesterPage::Pick) {
            v.menu = menu_->view(f.now);
            return v;
        }
        build(v, f, 0.0);
        const double since = clock_.observe(layoutKey(v), f.now);
        v = TesterView{};
        v.page = page_;
        build(v, f, since);
        return v;
    }

    /// One touch read against the frame ON THE SCREEN (`drawn`). Pages without buttons ignore
    /// touches (the watch owns the abort touch on COUNTDOWN and RUNNING); a frame whose page is not
    /// the current one acts on nothing; RUN is re-checked against runAllowed() at the release.
    [[nodiscard]] TesterAction touch(const ui::TouchSample& s, const TesterView& drawn,
                                     const TesterFacts& f) {
        TesterAction a{};
        if (page_ == TesterPage::Pick) {
            SHULIB_PRECONDITION(menu_.has_value(), "TesterScreen::touch: showPick() was never called");
            // The menu's own edge takes the release even when the drawn frame is not PICK's (its
            // menu view is then empty), so a release is consumed exactly once and never replayed.
            const competition::MenuTouch t = menu_->touch(
                s, drawn.page == TesterPage::Pick ? drawn.menu : competition::MenuView{});
            if (t.action == competition::MenuTouchAction::Chose && t.choice >= 0) {
                showReady(t.choice, s.releaseCount);
            }
            return a;
        }
        if (!edge_.take(s)) {
            return a;
        }
        edgeCount_ = s.releaseCount;
        if (liftPending_) {
            liftPending_ = false;        // the held abort touch lifted: it acts on nothing,
            clock_ = ui::ArmingClock{};  // and the buttons arm kArmingDelaySeconds from now
            return a;
        }
        if (drawn.page != page_) {
            return a;
        }
        const int hit = ui::hitTest(drawn.buttons, drawn.buttonCount, s.x, s.y);
        if (hit < 0) {
            return a;
        }
        switch (drawn.buttons[static_cast<std::size_t>(hit)].id) {
            case TesterButtonId::Back:
            case TesterButtonId::Pick:
                showPick(s.releaseCount);
                return a;
            case TesterButtonId::Again:
                if (page_ == TesterPage::Result) {
                    showReady(entry_, s.releaseCount);  // READY again -- never straight to RUN
                }
                return a;
            case TesterButtonId::Run:
                if (page_ == TesterPage::Ready && runAllowed(f, nullptr)) {
                    a.kind = TesterActionKind::Run;
                    a.entry = entry_;
                }
                return a;
        }
        return a;
    }

private:
    void enter(TesterPage p, std::int32_t releaseCount) {
        liftPending_ = false;
        page_ = p;
        edge_ = ui::ReleaseEdge{releaseCount};
        edgeCount_ = releaseCount;
        clock_ = ui::ArmingClock{};
    }

    static void setLine(TesterView& v, competition::MenuLine& l, std::int16_t x, std::int16_t y,
                        ui::TextSize size, ui::Tone tone, std::string_view text) noexcept {
        l.x = x;
        l.y = y;
        l.size = size;
        l.tone = tone;
        if (!ui::screenText(text, ui::budgetFor(size))) {
            ++v.unsafeLines;  // measured BEFORE the safety copy cuts it (file banner)
        }
        ui::copyScreenText(l.text, sizeof l.text, text, ui::budgetFor(size));
    }

    void addLine(TesterView& v, ui::Tone tone, std::string_view text) const noexcept {
        if (v.lineCount >= std::size(v.lines)) {
            ++v.droppedLines;  // never silent: a test fails on it (file banner)
            return;
        }
        setLine(v, v.lines[v.lineCount], 6,
                static_cast<std::int16_t>(lineY0() + ui::kSmallHeight * v.lineCount),
                ui::TextSize::Small, tone, text);
        ++v.lineCount;
    }

    [[nodiscard]] std::int16_t lineY0() const noexcept {
        return page_ == TesterPage::Ready    ? kTesterReadyLineY0
               : page_ == TesterPage::Result ? kTesterResultLineY0
                                             : kTesterRunningLineY0;
    }

    static void addButton(TesterView& v, TesterButtonId id, ui::Rect r, std::string_view label,
                          bool armed) noexcept {
        TesterButton& b = v.buttons[v.buttonCount++];
        b.id = id;
        b.rect = r;
        ui::copyScreenText(b.label, sizeof b.label, label, ui::kMediumCols);
        b.armed = armed;
    }

    [[nodiscard]] static std::uint32_t layoutKey(const TesterView& v) noexcept {
        ui::LayoutKey k;
        k.mix(static_cast<std::uint32_t>(v.page));
        k.mix(v.buttonCount);
        for (std::size_t i = 0; i < v.buttonCount; ++i) {
            k.mix(static_cast<std::uint32_t>(v.buttons[i].id));
            k.mix(v.buttons[i].rect);
        }
        return k.value();
    }

    /// Up to `maxLines` SMALL lines of `text`, broken at spaces (a word longer than a line is cut
    /// at the budget). For sentences the page did not write: a refusal, a thrown precondition.
    void addWrapped(TesterView& v, ui::Tone tone, std::string_view text, int maxLines) const {
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
            addLine(v, tone, text.substr(0, take));
            text.remove_prefix(take);
            while (!text.empty() && text.front() == ' ') {
                text.remove_prefix(1);
            }
            ++used;
        }
    }

    void build(TesterView& v, const TesterFacts& f, double since) const {
        const competition::AutonEntry& e = registry_->at(static_cast<std::size_t>(entry_));
        // Formatted into TWICE the widest budget, so a line that runs past its budget reaches
        // setLine() whole and is COUNTED there (`unsafeLines`) — formatted into a budget-sized
        // buffer, snprintf itself would cut it silently (found by mutation, 2026-09-28).
        char text[2 * ui::kSmallCols + 1] = "";
        std::snprintf(text, sizeof text, "%s%.29s", competition::kMenuHeaderPrefix.data(), e.name);
        setLine(v, v.header, 6, 4, ui::TextSize::Medium, ui::Tone::White, text);
        const bool armedNow = !liftPending_ && ui::ArmingClock::armedAfter(since);
        switch (page_) {
            case TesterPage::Ready: {
                v.bar = ui::Tone::Blue;
                // (1) the schedule and the start
                std::snprintf(text, sizeof text, "%s park T+%.1fs stop T+%.1fs %s",
                              competition::autonKindWord(e.kind), e.endActionAt.value(),
                              e.hardStopAt.value(),
                              competition::startModeWord(e.effectiveStart().mode()));
                addLine(v, ui::Tone::White, text);
                // (2) how the robot booted: its forward (D9)
                addLine(v, f.bootFromRecord ? ui::Tone::White : ui::Tone::Red,
                        f.bootFromRecord ? kBootRecord : kBootTable);
                // (3) the live estimate
                if (f.estimateLive) {
                    std::snprintf(text, sizeof text, "EST x %.1f y %.1f in, facing %.1f deg LIVE",
                                  f.estimate.x().value(), f.estimate.y().value(),
                                  f.estimate.heading().degrees());
                    addLine(v, ui::Tone::Green, text);
                } else {
                    addLine(v, ui::Tone::Amber, kNotLive);
                }
                // (4) the RUN gate's first failing reason, or READY
                std::string_view why;
                const bool ok = runAllowed(f, &why);
                addLine(v, ok ? ui::Tone::Green : ui::Tone::Amber, why);
                // (5)-(6) what RUN does, and the abort
                addLine(v, ui::Tone::White, kReadyHint);
                addLine(v, ui::Tone::Amber, kAbortHint);
                // (7) the SD run log
                addLine(v, ui::Tone::Grey, f.sdLogging ? kSdOn : kSdOff);
                // (8) the faults (the whole boot's) and the upload check
                if (f.faultCount == 0) {
                    std::snprintf(text, sizeof text, "faults: none  %.14s", f.buildStamp);
                } else {
                    std::snprintf(text, sizeof text, "FAULT n=%d last %.15s  %.14s", f.faultCount,
                                  diag::faultCodeName(f.lastFault), f.buildStamp);
                }
                addLine(v, f.faultCount == 0 ? ui::Tone::Grey : ui::Tone::Red, text);
                addButton(v, TesterButtonId::Back, competition::kMenuPrevSlot, kBackLabel, armedNow);
                addButton(v, TesterButtonId::Run, competition::kMenuNextSlot, kRunLabel,
                          armedNow && ok);
                return;
            }
            case TesterPage::Countdown: {
                v.bar = ui::Tone::Amber;
                const double left = kTesterCountdownSeconds - (f.now - started_);
                std::snprintf(text, sizeof text, "STEP BACK  %d",
                              static_cast<int>(std::ceil(left > 0.0 ? left : 0.0)));
                setLine(v, v.big[0], 6, 50, ui::TextSize::Large, ui::Tone::Amber, text);
                setLine(v, v.big[1], 6, 100, ui::TextSize::Medium, ui::Tone::White, kCountdownSub);
                v.bigCount = 2;
                return;
            }
            case TesterPage::Running: {
                v.bar = ui::Tone::Green;
                std::snprintf(text, sizeof text, "RUNNING T+%.1f", f.now - started_);
                setLine(v, v.big[0], 6, 40, ui::TextSize::Large, ui::Tone::Green, text);
                setLine(v, v.big[1], 6, 90, ui::TextSize::Large, ui::Tone::Amber, kBigAbort);
                v.bigCount = 2;
                if (f.abortCause != AbortCause::None) {
                    std::snprintf(text, sizeof text, "ABORTING: %.15s",
                                  abortCauseShort(f.abortCause));
                    addLine(v, ui::Tone::Red, text);
                }
                std::snprintf(text, sizeof text, "EST x %.1f y %.1f in, facing %.1f deg",
                              f.estimate.x().value(), f.estimate.y().value(),
                              f.estimate.heading().degrees());
                addLine(v, ui::Tone::White, text);
                return;
            }
            case TesterPage::Result: {
                buildResult(v, text, sizeof text);
                addButton(v, TesterButtonId::Pick, competition::kMenuPrevSlot, kPickLabel, armedNow);
                addButton(v, TesterButtonId::Again, competition::kMenuNoneSlot, kAgainLabel,
                          armedNow);
                return;
            }
            case TesterPage::Pick:
                return;
        }
    }

    /// RESULT's words (file banner): the big word, then the run's six lines -- or, for a countdown
    /// abort, a throw or a refusal, what happened instead.
    void buildResult(TesterView& v, char* text, std::size_t cap) const {
        const TesterRunRecord& r = *result_;
        const ui::Tone tone = resultTone();
        v.bar = tone;
        setLine(v, v.big[0], 6, 32, ui::TextSize::Large, tone, testerResultWord(r));
        v.bigCount = 1;
        char b[40] = "";
        buttonNamesOf(r.buttons, b, sizeof b);
        const bool byButton = r.cause == AbortCause::Button;
        if (r.countdownAborted) {
            std::snprintf(text, cap, "aborted in the countdown by: %.15s%s%.14s",
                          abortCauseShort(r.cause), byButton ? " " : "", byButton ? b : "");
            addLine(v, ui::Tone::Amber, text);
            addLine(v, ui::Tone::White, kNothingMoved);
            addLine(v, ui::Tone::White, kPickOrAgain);
            return;
        }
        if (r.threw) {
            addLine(v, ui::Tone::Red, kThrewLine);
            addWrapped(v, ui::Tone::White, r.threwWhat, 3);
            addLine(v, ui::Tone::Grey, kMoreOnSerial);
            addLine(v, ui::Tone::White, kPickOrAgain);
            return;
        }
        if (r.run.outcome == competition::AutonRunOutcome::Refused) {
            addWrapped(v, ui::Tone::Red, r.run.why, 4);
            addLine(v, ui::Tone::White, kNothingMoved);
            addLine(v, ui::Tone::White, kPickOrAgain);
            return;
        }
        const sequence::RunGuardReport& g = r.run.guard;
        // (1) how it ended -- and an abort the guard never acted on, named in red words
        if (g.abandoned) {
            std::snprintf(text, cap, "aborted by: %.15s%s%.14s", abortCauseShort(r.cause),
                          byButton ? " " : "", byButton ? b : "");
        } else if (abortIgnored(r)) {
            std::snprintf(text, cap, "abort IGNORED: %.15s%s%.14s", abortCauseShort(r.cause),
                          byButton ? " " : "", byButton ? b : "");
        } else {
            std::snprintf(text, cap, "%s, %s", competition::autonRunOutcomeWord(r.run.outcome),
                          g.scoringCut ? "cut at the deadline" : "in time");
        }
        addLine(v, ui::Tone::White, text);
        // (2) the end action and the guard's clock: an abort DURING the end action cut it (the
        //     guard voids its verdict) -- CUT, never FAILED, which would send the operator looking
        //     for a brake fault that does not exist
        std::snprintf(text, cap, "end action %s; scoring T+%.2fs, %d paces",
                      !g.endActionRan       ? "skipped"
                      : g.abandoned         ? "CUT"
                      : g.endActionSucceeded ? "succeeded"
                                             : "FAILED",
                      g.scoringEnded.value(), g.pacesSeen);
        addLine(v, ui::Tone::White, text);
        // (3)-(4) where it was seated, and where the estimate says it ended
        std::snprintf(text, cap, "START (%.1f, %.1f) %.1f deg", r.run.seated.x().value(),
                      r.run.seated.y().value(), r.run.seated.heading().degrees());
        addLine(v, ui::Tone::White, text);
        std::snprintf(text, cap, "END   (%.1f, %.1f) %.1f deg%s", r.endEstimate.x().value(),
                      r.endEstimate.y().value(), r.endEstimate.heading().degrees(),
                      r.endLive ? "" : " NOT LIVE");
        addLine(v, r.endLive ? ui::Tone::Green : ui::Tone::Amber, text);
        // (5) THIS run's faults (never the boot's)
        if (r.faultsAfter == r.faultsBefore) {
            addLine(v, ui::Tone::White, kNoFaults);
        } else {
            std::snprintf(text, cap, "faults this run: %d, last %.20s",
                          r.faultsAfter - r.faultsBefore, diag::faultCodeName(r.lastFault));
            addLine(v, ui::Tone::Red, text);
        }
        // (6) what to do now
        addLine(v, ui::Tone::Amber, kMeasureTape);
    }

    [[nodiscard]] ui::Tone resultTone() const noexcept {
        const std::string_view w = testerResultWord(*result_);
        return w == "FINISHED"                                ? ui::Tone::Green
               : (w == "ABORTED" || w == "CUT BY DEADLINE") ? ui::Tone::Amber
                                                              : ui::Tone::Red;
    }

    const competition::AutonRegistry* registry_;
    std::optional<competition::AutonMenu> menu_{};
    ui::ReleaseEdge edge_;
    std::int32_t edgeCount_ = 0;
    ui::ArmingClock clock_{};
    TesterPage page_ = TesterPage::Pick;
    int entry_ = -1;
    double started_ = 0.0;
    const TesterRunRecord* result_ = nullptr;
    bool liftPending_ = false;
};

}  // namespace shulib::bench
