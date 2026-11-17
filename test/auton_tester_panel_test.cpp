// auton_tester_panel_test.cpp — "shulib Auton Tester"'s SCREEN as a pure model
// (src/programs/auton_tester_panel.hpp; chunk COMP-t, 2026-09-28; the brief's §5 tests
// (24)–(30), (27b)). Every case names the bug it would catch.
//
// The screen is the last thing a person touches before a robot moves on the bench. So the model is
// walked here as the program drives it — each "tick" asks for the frame to paint (the painter), then
// hands the touch read to touch() against THAT frame — and the rules are tried against the touch
// that would break them: a tap replayed on the next page, a new button under a finger, RUN armed
// when it must not be, a line cut or dropped without a word. The budgets are ESTIMATES (A4 register
// HA-136); nothing here has run on a robot (worksheet Station M).

#include "doctest.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <string_view>

#include "programs/auton_abort_watch.hpp"
#include "programs/auton_tester_panel.hpp"
#include "programs/auton_tester_run.hpp"
#include "team/autons.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/competition/auton_menu.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/competition/match_runner.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/math/reflection.hpp"
#include "shulib/ui/touch_panel.hpp"

using shulib::bench::AbortCause;
using shulib::bench::runAllowed;
using shulib::bench::sameButtons;
using shulib::bench::TesterAction;
using shulib::bench::TesterActionKind;
using shulib::bench::TesterButtonId;
using shulib::bench::TesterFacts;
using shulib::bench::TesterPage;
using shulib::bench::TesterRunRecord;
using shulib::bench::TesterScreen;
using shulib::bench::TesterView;
using shulib::competition::AutonRegistry;
using shulib::competition::AutonRunOutcome;
using shulib::competition::kMenuEntryRow0;
using shulib::competition::kMenuEntryRow1;
using shulib::competition::kMenuNextSlot;
using shulib::competition::kMenuNoneSlot;
using shulib::competition::kMenuPrevSlot;
using shulib::diag::FaultCode;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::ui::Rect;
using shulib::ui::TouchSample;
using shulib::units::Length;
using shulib::units::Time;

namespace {

AutonRegistry teamRegistry() {
    AutonRegistry reg{shulib::bench::kAutonPeriods};
    shulib::bench::registerAutons(reg);
    return reg;
}

/// The facts on a bench where RUN is allowed.
TesterFacts okFacts(double now) {
    TesterFacts f;
    f.now = now;
    f.estimate = Pose2d{Length{1.5}, Length{-0.25}, Angle::degrees(2.0)};
    f.estimateLive = true;
    f.controllerConnected = true;
    f.competitionStatus = 0;
    f.bootFromRecord = true;
    f.sdLogging = true;
    f.buildStamp = "BUILD 12:34:56";
    return f;
}

std::int16_t cx(Rect r) { return static_cast<std::int16_t>((r.x0 + r.x1) / 2); }
std::int16_t cy(Rect r) { return static_cast<std::int16_t>((r.y0 + r.y1) / 2); }

/// A touch read carrying release number `count` at the centre of `r`.
TouchSample releaseAt(std::int32_t count, Rect r) {
    TouchSample s;
    s.releaseCount = count;
    s.x = cx(r);
    s.y = cy(r);
    return s;
}

/// The program's loop, reduced: paint (view), then touch against what was painted.
struct Driver {
    TesterScreen& screen;
    TesterView drawn{};
    std::int32_t releases = 10;
    TesterView paint(const TesterFacts& f) {
        drawn = screen.view(f);
        return drawn;
    }
    TesterAction tap(Rect where, const TesterFacts& f) {
        ++releases;
        return screen.touch(releaseAt(releases, where), drawn, f);
    }
    TesterAction idle(const TesterFacts& f) {
        TouchSample s;
        s.releaseCount = releases;
        return screen.touch(s, drawn, f);
    }
};

const shulib::bench::TesterButton* button(const TesterView& v, TesterButtonId id) {
    for (int i = 0; i < v.buttonCount; ++i) {
        if (v.buttons[i].id == id) {
            return &v.buttons[i];
        }
    }
    return nullptr;
}

/// A finished run of entry `entry`, as a RESULT page is handed it.
TesterRunRecord finished(int entry) {
    TesterRunRecord r{};
    r.number = 1;
    r.entry = entry;
    r.run.outcome = AutonRunOutcome::RoutineRan;
    r.run.routine.ok = true;
    r.run.routine.completed = 1;
    r.run.routine.steps = 1;
    r.run.guard.endActionRan = true;
    r.run.guard.endActionSucceeded = true;
    r.run.guard.scoringEnded = Time{1.80};
    r.run.guard.pacesSeen = 192;
    r.endEstimate = Pose2d{Length{39.77}, Length{0.0}, Angle::degrees(0.0)};
    r.endLive = true;
    return r;
}

bool fits(const shulib::competition::MenuLine& l) {
    return shulib::ui::screenText(l.text, shulib::ui::budgetFor(l.size));
}

void checkClean(const TesterView& v) {
    CHECK(v.droppedLines == 0);
    CHECK(v.unsafeLines == 0);
    for (int i = 0; i < v.lineCount; ++i) {
        CHECK(fits(v.lines[i]));
    }
    for (int i = 0; i < v.bigCount; ++i) {
        CHECK(fits(v.big[i]));
    }
    CHECK(fits(v.header));
}

}  // namespace

// (24) Bug caught: a line cut or dropped WITHOUT A WORD — the first probe's view held 7 lines and
// silently dropped READY's 8th (mutation 33); a formatted line that overflows its budget at the
// worst case and is cut by the safety copy (read as fine unless measured BEFORE the copy — the
// view's `unsafeLines`); a page that emits fewer lines than §3.4 specifies.
TEST_CASE("panel (24): every page's lines fit at the worst case, none dropped, the right count") {
    AutonRegistry reg{shulib::bench::kAutonPeriods};
    shulib::competition::AutonEntry worst{};
    worst.name = "abcdefghijklmnopqrstuvwxyz012";  // 29 characters, the name budget
    worst.kind = shulib::competition::AutonKind::Skills;
    worst.routine = [](shulib::competition::AutonContext&) { return shulib::chassis::RoutineResult{}; };
    worst.start = shulib::competition::StartPose::relative();
    worst.endActionAt = Time{59.9};
    worst.hardStopAt = Time{60.0};
    reg.add(worst);
    TesterScreen screen{reg};

    // READY, the worst case: the longest numbers the field allows, 99999 faults, the longest
    // fault name, the BUILD stamp — EIGHT lines, each within 50, none cut.
    TesterFacts f = okFacts(100.0);
    f.estimate = Pose2d{Length{-999.9}, Length{-999.9}, Angle::degrees(-179.9)};
    f.faultCount = 99999;
    f.lastFault = FaultCode::GpsGateReject;
    screen.showReady(0, 0);
    TesterView v = screen.view(f);
    CHECK(v.lineCount == 8);
    checkClean(v);
    CHECK(std::string_view{v.lines[7].text} == "FAULT n=99999 last GPS_GATE_REJECT  BUILD 12:34:56");
    CHECK(std::strlen(v.lines[7].text) == 50);  // exactly the budget (the brief measured it)
    f.lastFault = FaultCode::MotorGroupDisagree;  // longer than 15: cut by its PRECISION, not by
    v = screen.view(f);                           // the safety copy
    checkClean(v);
    for (const bool live : {true, false}) {
        for (const bool record : {true, false}) {
            for (const bool sd : {true, false}) {
                f.estimateLive = live;
                f.bootFromRecord = record;
                f.sdLogging = sd;
                f.faultCount = 0;
                v = screen.view(f);
                CHECK(v.lineCount == 8);
                checkClean(v);
            }
        }
    }
    CHECK(std::string_view{v.header.text} == "AUTON: abcdefghijklmnopqrstuvwxyz012");

    // COUNTDOWN and RUNNING.
    screen.showCountdown(200.0);
    f = okFacts(200.0);
    v = screen.view(f);
    CHECK(v.bigCount == 2);
    CHECK(v.buttonCount == 0);
    checkClean(v);
    screen.showRunning(300.0);
    f = okFacts(359.9);
    f.estimate = Pose2d{Length{-999.9}, Length{-999.9}, Angle::degrees(-179.9)};
    f.abortCause = AbortCause::ControllerLost;
    v = screen.view(f);
    CHECK(v.bigCount == 2);
    CHECK(v.lineCount == 2);
    CHECK(v.buttonCount == 0);
    checkClean(v);

    // RESULT for a run: SIX lines, at the worst case.
    TesterRunRecord r = finished(0);
    r.run.guard.scoringEnded = Time{60.0};
    r.run.guard.pacesSeen = 6000;
    r.run.seated = Pose2d{Length{-999.9}, Length{-999.9}, Angle::degrees(-179.9)};
    r.endEstimate = Pose2d{Length{-999.9}, Length{-999.9}, Angle::degrees(-179.9)};
    r.endLive = false;
    r.faultsAfter = 99999;
    r.lastFault = FaultCode::MotorGroupDisagree;
    r.run.guard.abandoned = true;
    r.cause = AbortCause::Button;
    r.buttons = 0x0FFF;
    screen.showResult(r, 0, false);
    v = screen.view(f);
    CHECK(v.lineCount == 6);
    CHECK(v.bigCount == 1);
    CHECK(v.buttonCount == 2);
    checkClean(v);
    CHECK(std::string_view{v.lines[3].text} == "END   (-999.9, -999.9) -179.9 deg NOT LIVE");
    r.endLive = true;
    v = screen.view(f);
    CHECK(std::string_view{v.lines[3].text} == "END   (-999.9, -999.9) -179.9 deg");  // no trailing
    // An abort during the end action (abandoned, the end action ran): CUT, never FAILED.
    CHECK(std::string_view{v.lines[1].text} == "end action CUT; scoring T+60.00s, 6000 paces");
    // An abort the guard never acted on, by every button: the red words, within 50.
    r.run.guard.abandoned = false;
    screen.showResult(r, 0, false);
    v = screen.view(f);
    CHECK(v.lineCount == 6);
    checkClean(v);
    CHECK(std::string_view{v.big[0].text} == "ABORT IGNORED");
    CHECK(std::string_view{v.lines[0].text} == "abort IGNORED: button L1+L2+R1+R2+UP");
    CHECK(std::string_view{v.lines[1].text} == "end action succeeded; scoring T+60.00s, 6000 paces");
    // The countdown abort (three lines), the throw, the refusal (each within 8, none cut).
    r = finished(0);
    r.countdownAborted = true;
    r.cause = AbortCause::Button;
    r.buttons = 0x0FFF;
    screen.showResult(r, 0, false);
    v = screen.view(f);
    CHECK(v.lineCount == 3);
    checkClean(v);
    r = finished(0);
    r.threw = true;
    std::memset(r.threwWhat, 'x', sizeof r.threwWhat - 1);
    screen.showResult(r, 0, false);
    v = screen.view(f);
    CHECK(v.lineCount == 6);
    checkClean(v);
    r = finished(0);
    r.run.outcome = AutonRunOutcome::Refused;
    std::snprintf(r.run.why, sizeof r.run.why, "%s",
                  "auton 'abcdefghijklmnopqrstuvwxyz012': a route auton needs a command registry "
                  "to fire its markers, and this sentence is long enough to need three lines");
    screen.showResult(r, 0, false);
    v = screen.view(f);
    CHECK(v.lineCount >= 4);
    CHECK(v.lineCount <= 6);
    checkClean(v);
}

// (25) Bug caught: RUN armed when it must not be — without a controller (mutation 10: the abort
// would have no hands), from a boot on the TYPED TABLE (mutation 32; D9: its front may be the other
// end), in a competition mode, disabled, or before the estimate is live — and the reason READY
// names being the wrong one (the precedence is disabled > a competition bit > not the RECORD > no
// controller > not live).
TEST_CASE("panel (25): runAllowed — the truth table and the order of its reasons") {
    std::string_view why;
    TesterFacts f = okFacts(1.0);
    CHECK(runAllowed(f, &why));
    CHECK(why == shulib::bench::kRunAllowed);

    // Each condition alone.
    f = okFacts(1.0);
    f.estimateLive = false;
    CHECK_FALSE(runAllowed(f, &why));
    CHECK(why == shulib::bench::kNotLive);
    f = okFacts(1.0);
    f.controllerConnected = false;
    CHECK_FALSE(runAllowed(f, &why));
    CHECK(why == shulib::bench::kNoController);
    f = okFacts(1.0);
    f.bootFromRecord = false;
    CHECK_FALSE(runAllowed(f, &why));
    CHECK(why == shulib::bench::kNeedsRecord);
    CHECK(why == "RUN needs the calibration RECORD: run Calibrate");
    for (const std::uint8_t bits : {shulib::bench::kStatusAutonomous, shulib::bench::kStatusConnected,
                                    static_cast<std::uint8_t>(0x0f & ~1u)}) {
        f = okFacts(1.0);
        f.competitionStatus = bits;
        CHECK_FALSE(runAllowed(f, &why));
        CHECK(why == shulib::bench::kCompetition);
    }
    f = okFacts(1.0);
    f.competitionStatus = shulib::bench::kStatusDisabled;
    CHECK_FALSE(runAllowed(f, &why));
    CHECK(why == shulib::bench::kDisabledWord);
    f.competitionStatus = 0x08;  // SYSTEM only: not a reason (only the three bits count)
    CHECK(runAllowed(f, &why));

    // The order: everything wrong at once names DISABLED; then peel one at a time.
    f = okFacts(1.0);
    f.competitionStatus = 0x0f;  // HA-128's pretend match: disabled + autonomous + connected
    f.bootFromRecord = false;
    f.controllerConnected = false;
    f.estimateLive = false;
    CHECK_FALSE(runAllowed(f, &why));
    CHECK(why == shulib::bench::kDisabledWord);
    f.competitionStatus = shulib::bench::kStatusConnected;
    CHECK(runAllowed(f, &why) == false);
    CHECK(why == shulib::bench::kCompetition);
    f.competitionStatus = 0;
    CHECK_FALSE(runAllowed(f, &why));
    CHECK(why == shulib::bench::kNeedsRecord);
    f.bootFromRecord = true;
    CHECK_FALSE(runAllowed(f, &why));
    CHECK(why == shulib::bench::kNoController);
    f.controllerConnected = true;
    CHECK_FALSE(runAllowed(f, &why));
    CHECK(why == shulib::bench::kNotLive);
    f.estimateLive = true;
    CHECK(runAllowed(f, nullptr));

    // READY says how the robot booted: RECORD, or red TYPED TABLE; and RUN greys out with the
    // reason on the table.
    const AutonRegistry reg = teamRegistry();
    TesterScreen screen{reg};
    screen.showReady(0, 0);
    TesterFacts g = okFacts(10.0);
    (void)screen.view(g);
    g.now = 11.0;
    TesterView v = screen.view(g);
    CHECK(std::string_view{v.lines[1].text} == shulib::bench::kBootRecord);
    CHECK(v.lines[1].tone != shulib::ui::Tone::Red);
    CHECK(button(v, TesterButtonId::Run)->armed);
    g.bootFromRecord = false;
    v = screen.view(g);
    CHECK(std::string_view{v.lines[1].text} == shulib::bench::kBootTable);
    CHECK(v.lines[1].tone == shulib::ui::Tone::Red);
    CHECK(std::string_view{v.lines[3].text} == shulib::bench::kNeedsRecord);
    CHECK_FALSE(button(v, TesterButtonId::Run)->armed);
}

// (26) Bug caught: a flow that goes somewhere else — PICK's choice not opening READY for THAT
// auton, BACK not giving a FRESH menu (its entries live at once: a reflexive tap chooses again), RUN
// armed before the arming delay or when R4 fails, RUN's release not asking to run the right entry,
// AGAIN going anywhere but READY for the same auton, PICK not going back to the selector.
TEST_CASE("panel (26): PICK -> READY -> RUN; BACK; RESULT -> AGAIN / PICK") {
    const AutonRegistry reg = teamRegistry();
    TesterScreen screen{reg};
    Driver d{screen};
    screen.showPick(d.releases);
    TesterFacts f = okFacts(0.0);
    (void)d.paint(f);
    f.now = 0.6;  // the menu's entries arm after 0.5 s
    (void)d.paint(f);
    CHECK(d.tap(kMenuEntryRow0, f).kind == TesterActionKind::None);
    CHECK(screen.page() == TesterPage::Ready);
    CHECK(screen.entry() == 0);

    // READY: both greyed on the first frame; after 0.5 s BACK is armed and RUN too (R4 holds).
    f.now = 1.0;
    TesterView v = d.paint(f);
    REQUIRE(v.buttonCount == 2);
    CHECK_FALSE(button(v, TesterButtonId::Back)->armed);
    CHECK_FALSE(button(v, TesterButtonId::Run)->armed);
    CHECK(d.tap(kMenuNextSlot, f).kind == TesterActionKind::None);  // greyed: nothing
    f.now = 1.49;
    v = d.paint(f);
    CHECK_FALSE(button(v, TesterButtonId::Run)->armed);
    f.now = 1.5;
    v = d.paint(f);
    CHECK(button(v, TesterButtonId::Run)->armed);
    CHECK(button(v, TesterButtonId::Back)->armed);
    // RUN while R4 fails (the controller drops between the paint and the release): nothing.
    TesterFacts noPad = f;
    noPad.controllerConnected = false;
    CHECK(d.tap(kMenuNextSlot, noPad).kind == TesterActionKind::None);
    CHECK(screen.page() == TesterPage::Ready);
    // ... and painted greyed with R4 failing.
    v = d.paint(noPad);
    CHECK_FALSE(button(v, TesterButtonId::Run)->armed);
    CHECK(button(v, TesterButtonId::Back)->armed);
    // RUN's release, allowed: Run(0).
    v = d.paint(f);
    const TesterAction a = d.tap(kMenuNextSlot, f);
    CHECK(a.kind == TesterActionKind::Run);
    CHECK(a.entry == 0);

    // BACK -> PICK with a FRESH menu: NONE live at once, the entries greyed 0.5 s.
    v = d.paint(f);
    CHECK(d.tap(kMenuPrevSlot, f).kind == TesterActionKind::None);
    CHECK(screen.page() == TesterPage::Pick);
    CHECK(screen.entry() == -1);
    f.now = 2.0;
    v = d.paint(f);
    REQUIRE(v.page == TesterPage::Pick);
    bool noneArmed = false;
    bool entryArmed = false;
    for (int i = 0; i < v.menu.buttonCount; ++i) {
        const auto& b = v.menu.buttons[i];
        if (b.id == shulib::competition::MenuButtonId::None) {
            noneArmed = b.armed;
        } else {
            entryArmed = entryArmed || b.armed;
        }
    }
    CHECK(noneArmed);
    CHECK_FALSE(entryArmed);
    CHECK(std::string_view{v.menu.header.text} == "AUTON: NONE");  // a fresh menu: NONE chosen
    // NONE: stays on PICK.
    (void)d.tap(kMenuNoneSlot, f);
    CHECK(screen.page() == TesterPage::Pick);
    // Entry 1 (row 1) after the delay -> READY(1).
    f.now = 2.6;
    (void)d.paint(f);
    (void)d.tap(kMenuEntryRow1, f);
    CHECK(screen.page() == TesterPage::Ready);
    CHECK(screen.entry() == 1);

    // RESULT (of entry 1) -> AGAIN -> READY(1), never RUN.
    const TesterRunRecord rec = finished(1);
    screen.showResult(rec, d.releases, false);
    f.now = 10.0;
    (void)d.paint(f);
    f.now = 10.5;
    v = d.paint(f);
    REQUIRE(button(v, TesterButtonId::Again) != nullptr);
    CHECK(button(v, TesterButtonId::Again)->armed);
    CHECK(d.tap(kMenuNoneSlot, f).kind == TesterActionKind::None);
    CHECK(screen.page() == TesterPage::Ready);
    CHECK(screen.entry() == 1);
    // RESULT -> PICK.
    screen.showResult(rec, d.releases, false);
    f.now = 20.0;
    (void)d.paint(f);
    f.now = 20.5;
    (void)d.paint(f);
    (void)d.tap(kMenuPrevSlot, f);
    CHECK(screen.page() == TesterPage::Pick);
}

// (27) Bug caught (mutation 11: READY keeping PICK's edge): a tap REPLAYED on the next page — the
// release that chose on PICK acting again on READY; an abort touch's release, arriving after the
// RESULT page is up, pressing whatever is under it; an error read (-1 everywhere) taken as a tap.
TEST_CASE("panel (27): no replay — a release acts once, on the page it was made on") {
    const AutonRegistry reg = teamRegistry();
    TesterScreen screen{reg};
    Driver d{screen};
    screen.showPick(d.releases);
    TesterFacts f = okFacts(0.0);
    (void)d.paint(f);
    f.now = 0.6;
    (void)d.paint(f);
    const TouchSample chose = releaseAt(d.releases + 1, kMenuEntryRow0);
    d.releases += 1;
    (void)screen.touch(chose, d.drawn, f);
    REQUIRE(screen.page() == TesterPage::Ready);
    // The same release read again, on READY, even long after its buttons armed: nothing.
    f.now = 5.0;
    (void)d.paint(f);
    f.now = 6.0;
    (void)d.paint(f);
    TouchSample again = chose;
    again.x = cx(kMenuNextSlot);  // and even if its x/y now point at RUN
    again.y = cy(kMenuNextSlot);
    CHECK(screen.touch(again, d.drawn, f).kind == TesterActionKind::None);
    CHECK(screen.page() == TesterPage::Ready);

    // An error read (every field -1) never acts, and does not move the edge.
    TouchSample err;
    err.error = true;
    err.releaseCount = -1;
    err.x = cx(kMenuNextSlot);
    err.y = cy(kMenuNextSlot);
    CHECK(screen.touch(err, d.drawn, f).kind == TesterActionKind::None);
    // ... so the next GOOD release still acts (on RUN).
    CHECK(d.tap(kMenuNextSlot, f).kind == TesterActionKind::Run);

    // RESULT: an abort touch's release arriving within 0.5 s, over PICK — nothing.
    const TesterRunRecord rec = finished(0);
    screen.showResult(rec, d.releases, false);
    f.now = 30.0;
    (void)d.paint(f);
    f.now = 30.2;
    (void)d.paint(f);
    CHECK(d.tap(kMenuPrevSlot, f).kind == TesterActionKind::None);
    CHECK(screen.page() == TesterPage::Result);
}

// (27b) Bug caught (the re-check's finding 13; mutation 31): RESULT entered with a finger STILL
// DOWN (an abort touch held through the end of the run) — its release, whenever the operator lets
// go, must act on NOTHING, and the buttons must stay grey until 0.5 s after the first frame after
// that release. With no finger down the plain 0.5 s rule applies (a release at 0.6 s acts).
TEST_CASE("panel (27b): RESULT with a finger down waits for that finger, then 0.5 s") {
    const AutonRegistry reg = teamRegistry();
    const TesterRunRecord rec = finished(0);
    for (const Rect over : {kMenuPrevSlot, kMenuNoneSlot}) {
        CAPTURE(over.x0);
        TesterScreen screen{reg};
        Driver d{screen};
        screen.showResult(rec, d.releases, /*fingerDown=*/true);
        CHECK(screen.waitingForLift());
        TesterFacts f = okFacts(100.0);
        TesterView v = d.paint(f);
        f.now = 102.0;  // two seconds later, the finger still down: grey
        v = d.paint(f);
        CHECK_FALSE(button(v, TesterButtonId::Pick)->armed);
        CHECK_FALSE(button(v, TesterButtonId::Again)->armed);
        // The held finger lifts — over a button. It acts on nothing.
        CHECK(d.tap(over, f).kind == TesterActionKind::None);
        CHECK(screen.page() == TesterPage::Result);
        CHECK_FALSE(screen.waitingForLift());
        f.now = 102.01;  // the first frame after the lift: the clock starts here
        v = d.paint(f);
        CHECK_FALSE(button(v, TesterButtonId::Pick)->armed);
        f.now = 102.5;
        v = d.paint(f);
        CHECK_FALSE(button(v, TesterButtonId::Pick)->armed);  // 0.49 s: still grey
        f.now = 102.52;
        v = d.paint(f);
        CHECK(button(v, TesterButtonId::Pick)->armed);
        CHECK(button(v, TesterButtonId::Again)->armed);
        (void)d.tap(over, f);  // now a tap acts
        CHECK(screen.page() == (over == kMenuPrevSlot ? TesterPage::Pick : TesterPage::Ready));
    }
    // No finger down: the plain rule — a release at 0.6 s acts.
    TesterScreen screen{reg};
    Driver d{screen};
    screen.showResult(rec, d.releases, false);
    CHECK_FALSE(screen.waitingForLift());
    TesterFacts f = okFacts(50.0);
    (void)d.paint(f);
    f.now = 50.6;
    (void)d.paint(f);
    (void)d.tap(kMenuPrevSlot, f);
    CHECK(screen.page() == TesterPage::Pick);
}

// (28) Bug caught (mutation 12: AGAIN placed at RUN's slot): a NEW button under the finger that
// just pressed — a reflexive double tap answering a question the first tap asked (AGAIN under RUN
// would start the countdown twice in a row; a PICK entry under READY's rows would re-choose). For
// every transition, the press point of the button that caused it must hit NOTHING on the page it
// leads to.
TEST_CASE("panel (28): no transition puts a button under the finger that caused it") {
    const AutonRegistry reg = teamRegistry();
    TesterScreen screen{reg};
    TesterFacts f = okFacts(0.0);
    const auto nothingAt = [&](const TesterView& v, Rect pressed) {
        const int x = cx(pressed);
        const int y = cy(pressed);
        for (int i = 0; i < v.buttonCount; ++i) {
            if (shulib::ui::contains(shulib::ui::hitBox(v.buttons[i].rect), x, y)) {
                return false;
            }
        }
        for (int i = 0; i < v.menu.buttonCount; ++i) {
            if (shulib::ui::contains(shulib::ui::hitBox(v.menu.buttons[i].rect), x, y)) {
                return false;
            }
        }
        return true;
    };
    // PICK's entry rows -> READY: nothing in either row on READY.
    screen.showReady(0, 0);
    TesterView ready = screen.view(f);
    CHECK(nothingAt(ready, kMenuEntryRow0));
    CHECK(nothingAt(ready, kMenuEntryRow1));
    // READY's RUN -> COUNTDOWN / RUNNING / RESULT: nothing at RUN's slot on any of them.
    screen.showCountdown(1.0);
    CHECK(nothingAt(screen.view(f), kMenuNextSlot));
    screen.showRunning(4.0);
    CHECK(nothingAt(screen.view(f), kMenuNextSlot));
    const TesterRunRecord rec = finished(0);
    screen.showResult(rec, 0, false);
    const TesterView result = screen.view(f);
    CHECK(nothingAt(result, kMenuNextSlot));
    // RESULT's AGAIN (bottom centre) -> READY: nothing there (READY's RUN is bottom right).
    CHECK(button(result, TesterButtonId::Again)->rect == kMenuNoneSlot);
    CHECK(button(result, TesterButtonId::Pick)->rect == kMenuPrevSlot);
    CHECK(nothingAt(ready, kMenuNoneSlot));
    // RESULT's PICK and READY's BACK (bottom left) -> PICK: a fresh menu has no PREV there.
    screen.showPick(0);
    const TesterView pick = screen.view(f);
    CHECK(nothingAt(pick, kMenuPrevSlot));
    // READY's buttons are only in the bottom row (the rows it is reached from are empty).
    for (int i = 0; i < ready.buttonCount; ++i) {
        CHECK(ready.buttons[i].rect.y0 >= kMenuPrevSlot.y0);
    }
}

// (29) Bug caught: a countdown that reads the wrong second (or never reaches 1), a RUNNING page
// whose clock does not run, or one that never says ABORTING once the watch latched.
TEST_CASE("panel (29): COUNTDOWN reads 3/2/1; RUNNING's T+ and ABORTING once latched") {
    const AutonRegistry reg = teamRegistry();
    TesterScreen screen{reg};
    screen.showReady(0, 0);  // the countdown always follows READY (its entry)
    screen.showCountdown(10.0);
    TesterFacts f = okFacts(10.0);
    CHECK(std::string_view{screen.view(f).big[0].text} == "STEP BACK  3");
    f.now = 10.99;
    CHECK(std::string_view{screen.view(f).big[0].text} == "STEP BACK  3");
    f.now = 11.01;
    CHECK(std::string_view{screen.view(f).big[0].text} == "STEP BACK  2");
    f.now = 12.5;
    CHECK(std::string_view{screen.view(f).big[0].text} == "STEP BACK  1");
    CHECK(std::string_view{screen.view(f).big[1].text} == shulib::bench::kCountdownSub);
    CHECK(screen.view(f).bar == shulib::ui::Tone::Amber);

    screen.showRunning(13.0);
    f.now = 25.3;
    TesterView v = screen.view(f);
    CHECK(std::string_view{v.big[0].text} == "RUNNING T+12.3");
    CHECK(std::string_view{v.big[1].text} == "ANY BUTTON ABORTS");
    CHECK(v.bar == shulib::ui::Tone::Green);
    CHECK(v.lineCount == 1);  // the live estimate only
    f.abortCause = AbortCause::Button;
    v = screen.view(f);
    CHECK(std::string_view{v.lines[0].text} == "ABORTING: button");
    CHECK(v.lines[0].tone == shulib::ui::Tone::Red);
}

// (30) Bug caught: a painter that repaints only text when a button changed (a touch tested against
// a frame not on the screen) — or does a full repaint for every text change (PROS leaks each
// print); and RESULT built from whatever page was up before instead of the record's own entry
// (mutation 13: the probe indexed -1 and the registry threw).
TEST_CASE("panel (30): sameButtons, and RESULT takes the record's entry") {
    const AutonRegistry reg = teamRegistry();
    TesterScreen screen{reg};
    screen.showReady(0, 0);
    TesterFacts f = okFacts(0.0);
    const TesterView grey = screen.view(f);
    f.now = 0.6;
    const TesterView armed = screen.view(f);
    CHECK_FALSE(sameButtons(grey, armed));  // an armed flip is a button change
    f.estimate = Pose2d{Length{9.0}, Length{1.0}, Angle::degrees(5.0)};
    const TesterView moved = screen.view(f);
    CHECK(std::string_view{moved.lines[2].text} != std::string_view{armed.lines[2].text});
    CHECK(sameButtons(armed, moved));  // a text-only change is not
    screen.showCountdown(1.0);
    const TesterView countdown = screen.view(f);
    CHECK_FALSE(sameButtons(moved, countdown));  // another page

    // Every button field, one at a time, on a copy (the review: only `armed` was ever varied).
    const auto everyField = [](const TesterView& base) {
        REQUIRE(base.buttonCount >= 1);
        TesterView c = base;
        CHECK(sameButtons(base, c));
        c.buttons[0].armed = !c.buttons[0].armed;
        CHECK_FALSE(sameButtons(base, c));
        c = base;
        c.buttons[0].id = c.buttons[0].id == TesterButtonId::Back ? TesterButtonId::Pick
                                                                  : TesterButtonId::Back;
        CHECK_FALSE(sameButtons(base, c));
        c = base;
        c.buttons[0].rect.x0 = static_cast<std::int16_t>(c.buttons[0].rect.x0 + 1);
        CHECK_FALSE(sameButtons(base, c));
        c = base;
        c.buttons[0].label[0] = c.buttons[0].label[0] == 'Z' ? 'Y' : 'Z';
        CHECK_FALSE(sameButtons(base, c));
        c = base;
        c.buttonCount = static_cast<std::uint8_t>(c.buttonCount - 1);
        CHECK_FALSE(sameButtons(base, c));
        c = base;
        c.lines[0].text[0] = c.lines[0].text[0] == 'Z' ? 'Y' : 'Z';  // text only
        c.header.text[0] = c.header.text[0] == 'Z' ? 'Y' : 'Z';
        CHECK(sameButtons(base, c));
    };
    everyField(armed);

    // PICK (the review, 2026-09-29: H4 — PICK's branch collapsed to "always the same" — and H4b —
    // its `armed` dropped — both SURVIVED). PICK's entries arm half a second after every entry to the
    // page: the frame at 0.0 s (grey) and at 0.6 s (armed) must differ, or the painter never repaints
    // PICK whole and a grey-looking entry could be chosen by a tap. Then every menu-button field on a
    // copy, the chosen highlight among them; a text-only change (the header) is not a button change.
    {
        TesterScreen pick{reg};
        pick.showPick(0);
        TesterFacts pf = okFacts(0.0);
        const TesterView p0 = pick.view(pf);
        pf.now = 0.6;
        const TesterView p6 = pick.view(pf);
        REQUIRE(p0.page == TesterPage::Pick);
        REQUIRE(p0.menu.buttonCount >= 2);
        CHECK_FALSE(p0.menu.buttons[0].armed);  // an entry: greyed on the first frame
        CHECK(p6.menu.buttons[0].armed);
        CHECK_FALSE(sameButtons(p0, p6));
        pf.now = 0.7;
        CHECK(sameButtons(p6, pick.view(pf)));  // nothing changed: no repaint
        const auto menuField = [&](const TesterView& base) {
            TesterView c = base;
            c.menu.buttons[0].armed = !c.menu.buttons[0].armed;
            CHECK_FALSE(sameButtons(base, c));
            c = base;
            c.menu.buttons[0].chosen = !c.menu.buttons[0].chosen;
            CHECK_FALSE(sameButtons(base, c));
            c = base;
            c.menu.buttons[0].entry = c.menu.buttons[0].entry + 1;
            CHECK_FALSE(sameButtons(base, c));
            c = base;
            c.menu.buttons[0].id = shulib::competition::MenuButtonId::Next;
            CHECK_FALSE(sameButtons(base, c));
            c = base;
            c.menu.buttons[0].rect.y1 = static_cast<std::int16_t>(c.menu.buttons[0].rect.y1 + 1);
            CHECK_FALSE(sameButtons(base, c));
            c = base;
            c.menu.buttons[0].label[0] = c.menu.buttons[0].label[0] == 'Z' ? 'Y' : 'Z';
            CHECK_FALSE(sameButtons(base, c));
            c = base;
            c.menu.buttonCount = static_cast<std::uint8_t>(c.menu.buttonCount - 1);
            CHECK_FALSE(sameButtons(base, c));
            c = base;
            c.menu.page = c.menu.page + 1;
            CHECK_FALSE(sameButtons(base, c));
            c = base;
            c.menu.header.text[0] = c.menu.header.text[0] == 'Z' ? 'Y' : 'Z';  // text only
            c.menu.info.text[0] = c.menu.info.text[0] == 'Z' ? 'Y' : 'Z';
            c.menu.status.text[0] = c.menu.status.text[0] == 'Z' ? 'Y' : 'Z';
            CHECK(sameButtons(base, c));
        };
        menuField(p0);
        menuField(p6);
        // The chosen highlight moving (NONE -> an entry), as the selector draws it.
        shulib::competition::AutonMenu menu{reg, 0};
        (void)menu.view(0.0);  // the first frame starts the menu's arming clock
        TesterView noneChosen{};
        noneChosen.page = TesterPage::Pick;
        noneChosen.menu = menu.view(0.6);
        REQUIRE(noneChosen.menu.buttons[0].armed);
        const shulib::competition::MenuTouch t =
            menu.touch(releaseAt(1, kMenuEntryRow0), noneChosen.menu);
        REQUIRE(t.action == shulib::competition::MenuTouchAction::Chose);
        TesterView entryChosen{};
        entryChosen.page = TesterPage::Pick;
        entryChosen.menu = menu.view(0.7);
        CHECK_FALSE(sameButtons(noneChosen, entryChosen));
    }

    // RESULT takes the record's entry, not the page that was up (READY of entry 0 here).
    screen.showReady(0, 0);
    const TesterRunRecord rec = finished(1);
    screen.showResult(rec, 0, false);
    CHECK(screen.entry() == 1);
    const TesterView v = screen.view(f);
    CHECK(std::string_view{v.header.text} == "AUTON: TEST 24in turn back");
    // A record naming no auton is a programming error, refused loudly.
    TesterRunRecord bad = finished(0);
    bad.entry = -1;
    CHECK_THROWS_AS(screen.showResult(bad, 0, false), shulib::PreconditionError);
}

// MIRROR1-A T8 (the MIRROR1 brief's D8, R9): a MIRRORED run's START line. Bug caught: robot two's real
// start shown without MIR (the drivers would not know it is the mirrored seat), or "-0.0" on the screen —
// robot one at x = 0 is robot two's y = 0, and the reflection negates it (MM17: the + 0.0 dropped); the
// START MIR line cut at the screen's OWN worst case; an unmirrored START line changed.
TEST_CASE("panel MIRROR1 T8: START MIR at the screen's worst case, a zero coordinate, a RELATIVE start") {
    const shulib::math::Reflection diag = shulib::math::Reflection::across(shulib::math::Reflection::Line::AntiDiagonal);
    const shulib::math::Reflection flip = shulib::math::Reflection::across(shulib::math::Reflection::Line::XAxis);
    AutonRegistry reg = teamRegistry();
    TesterScreen screen{reg};
    const TesterFacts f = okFacts(100.0);
    TesterRunRecord r = finished(0);
    r.run.seated = Pose2d{Length{-999.9}, Length{-999.9}, Angle::degrees(-179.9)};
    r.run.mirrored = true;
    screen.showResult(r, 0, false);
    TesterView v = screen.view(f);
    checkClean(v);
    CHECK(std::string_view{v.lines[2].text} == "START MIR (-999.9, -999.9) -179.9 deg");
    r.run.mirrored = false;
    screen.showResult(r, 0, false);
    v = screen.view(f);
    CHECK(std::string_view{v.lines[2].text} == "START (-999.9, -999.9) -179.9 deg");  // unmirrored: unchanged
    // robot one at (0, -24) facing 90 -> robot two at (24, 0) facing 180: "0.0", never "-0.0"
    r.run.mirrored = true;
    r.run.seated = diag.apply(Pose2d{Length{0.0}, Length{-24.0}, Angle::degrees(90.0)});
    screen.showResult(r, 0, false);
    v = screen.view(f);
    CHECK(std::string_view{v.lines[2].text} == "START MIR (24.0, 0.0) 180.0 deg");
    // a mirrored RELATIVE start: (0, 0, 0) through the robot-frame mirror
    r.run.seated = flip.apply(Pose2d{});
    screen.showResult(r, 0, false);
    v = screen.view(f);
    CHECK(std::string_view{v.lines[2].text} == "START MIR (0.0, 0.0) 0.0 deg");
}
