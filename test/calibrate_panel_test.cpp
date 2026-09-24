// calibration::viewFor / hitTest / TouchGate / wrapWords / stoppedWords — the calibration screen as
// a pure model (chunk R3d Part 2F-1), driven over the SAME plant and scripted operator as the
// calibrator's own cases (calibration_test_rig.hpp). NOTHING HERE HAS RUN ON A ROBOT, and no pixel
// here has been seen on a V5: the budgets and positions are estimates until the SCREEN RULER is
// read (calibrate_panel.hpp). What these cases prove is the screen's LOGIC — which buttons exist,
// where, when they arm, what a touch means — against the state machine it serves.
//
// Each case names the bug it catches and the mutation that must turn it red:
//   P1  every button drawn LIVE is one the calibrator accepts right now, in every reachable state
//       of seven runs (one clean, one with a slipping member, five that stop); ABORT is on every
//       running screen, RUN AGAIN on every finished one; every line fits its size's budget, is
//       ASCII with no '%', and stays on the screen clear of every button
//                                                        (draw DONE on accepts() instead of ready)
//   P2  DONE is not drawn at 0 or 1 moved, nor before far enough, nor while the robot moves; it is
//       drawn once pushReady() says so                  (the same mutation)
//   P3  ABORT has the identical rectangle on every screen that shows it; the powered screens widen
//       it to the whole row, which contains that rectangle         (move ABORT on one screen)
//   P4  no button of a new layout lies under the button whose touch caused the change — a second
//       tap where DONE / FRONT / ABORT was lands on NOTHING, even after the arming delay
//                                                        (put FRONT where DONE was)
//   P5  the arming delay: a tap on a new button within 0.5 s lands on nothing; at 0.5 s it counts;
//       ABORT is never delayed                          (drop the arming delay)
//   P6  TouchGate: the release edge only; stale x/y never act; an error read is ignored and never
//       replayed; a stale view's button is re-checked against the calibrator
//   P7  hitTest's padding, exactly at its edges; a greyed button never hits
//   P8  wrapWords never splits a word or a number and never drops text silently
//   P9  the literal checks: every literal and every STOPPED wording fits, is ASCII, has no '%';
//       the header's source has no "%%" and no non-ASCII byte inside a string literal
//   P10 the STOPPED screen for each reachable refusal: its code's words, the port, and the reason
//       wrapped whole
//   P11 the finished screen: the counts, the scales, the width, the LEFT OUT port, per SaveOutcome
//   P12 TouchGate::view arms a layout after 0.5 s and re-greys it on every layout change

#include "doctest.h"

#include <cstring>
#include <fstream>
#include <set>
#include <string>
#include <vector>

#include "calibration_test_rig.hpp"
#include "shulib/calibration/calibrate_panel.hpp"

using namespace calib_rig;
using shulib::calibration::budgetFor;
using shulib::calibration::heightFor;
using shulib::calibration::hitBox;
using shulib::calibration::hitTest;
using shulib::calibration::kAbortRow;
using shulib::calibration::kAbortSlot;
using shulib::calibration::kArmingDelaySeconds;
using shulib::calibration::kFrontBox;
using shulib::calibration::kHeaderCols;
using shulib::calibration::kHitPadX;
using shulib::calibration::kHitPadY;
using shulib::calibration::kLargeCols;
using shulib::calibration::kMediumCols;
using shulib::calibration::kPanelLastRow;
using shulib::calibration::kPanelWidth;
using shulib::calibration::kPrimarySlot;
using shulib::calibration::kRunAgainBox;
using shulib::calibration::kSmallCols;
using shulib::calibration::layoutKey;
using shulib::calibration::overlaps;
using shulib::calibration::PanelButton;
using shulib::calibration::PanelButtonId;
using shulib::calibration::PanelRect;
using shulib::calibration::PanelScreen;
using shulib::calibration::PanelTone;
using shulib::calibration::PanelView;
using shulib::calibration::Refusal;
using shulib::calibration::SaveOutcome;
using shulib::calibration::stoppedWords;
using shulib::calibration::TextSize;
using shulib::calibration::TouchAction;
using shulib::calibration::TouchGate;
using shulib::calibration::TouchSample;
using shulib::calibration::viewFor;
using shulib::calibration::within;
using shulib::calibration::wrapWords;

/// Rectangles print as their corners in a failure message (doctest shows "{?}" otherwise).
template <>
struct doctest::StringMaker<shulib::calibration::PanelRect> {
    static doctest::String convert(const shulib::calibration::PanelRect& r) {
        return ("{" + std::to_string(r.x0) + ", " + std::to_string(r.y0) + ", " + std::to_string(r.x1)
                + ", " + std::to_string(r.y1) + "}")
            .c_str();
    }
};

namespace {

/// A long time after the last layout change: every button armed.
constexpr double kArmed = 10.0;

[[nodiscard]] const PanelButton* findButton(const PanelView& v, PanelButtonId id) {
    for (std::size_t i = 0; i < v.buttonCount; ++i) {
        if (v.buttons[i].id == id) {
            return &v.buttons[i];
        }
    }
    return nullptr;
}

[[nodiscard]] bool hasLine(const PanelView& v, const char* text) {
    for (std::size_t i = 0; i < v.lineCount; ++i) {
        if (std::strcmp(v.lines[i].text, text) == 0) {
            return true;
        }
    }
    return false;
}

[[nodiscard]] bool running(Phase p) { return p != Phase::Done && p != Phase::Aborted; }

/// What a button's touch would mean to the calibrator right now — true when it is legal AND wise.
[[nodiscard]] bool buttonMeansSomething(const PanelButton& b, const DrivetrainCalibrator& cal) {
    switch (b.id) {
        case PanelButtonId::Done: return cal.accepts(Answer::Done) && cal.pushReady();
        case PanelButtonId::Front: return cal.accepts(Answer::FrontFirst);
        case PanelButtonId::Back: return cal.accepts(Answer::BackFirst);
        case PanelButtonId::Abort: return cal.accepts(Answer::Abort) && running(cal.phase());
        case PanelButtonId::RunAgain: return !running(cal.phase());
    }
    return false;
}

/// The pixel box a line occupies, from its size's estimated character width (the budget spread
/// over a 472-pixel line) and height.
[[nodiscard]] PanelRect lineBox(const shulib::calibration::PanelLine& l) {
    const auto charW = static_cast<int>(472 / budgetFor(l.size));
    const auto len = static_cast<int>(std::strlen(l.text));
    return PanelRect{l.x, l.y, static_cast<std::int16_t>(l.x + len * charW - 1),
                     static_cast<std::int16_t>(l.y + heightFor(l.size) - 1)};
}

/// Every text rule for one view: header word, every line (budget, ASCII, no '%', on screen, clear
/// of every button), every label.
void checkText(const PanelView& v) {
    CHECK(std::strlen(v.headerWord) <= kHeaderCols);
    CHECK(screenSafe(v.headerWord));
    for (std::size_t i = 0; i < v.lineCount; ++i) {
        const auto& l = v.lines[i];
        CAPTURE(std::string{l.text});
        CHECK(std::strlen(l.text) <= budgetFor(l.size));
        CHECK(screenSafe(l.text));
        const PanelRect box = lineBox(l);
        CHECK(box.x1 < kPanelWidth);
        CHECK(box.y1 <= kPanelLastRow);
        for (std::size_t b = 0; b < v.buttonCount; ++b) {
            CHECK_FALSE(overlaps(box, v.buttons[b].rect));
        }
    }
    for (std::size_t b = 0; b < v.buttonCount; ++b) {
        CHECK(screenSafe(v.buttons[b].label));
        CHECK(std::strlen(v.buttons[b].label) <= 12);
        CHECK(v.buttons[b].rect.y1 <= kPanelLastRow);
    }
}

/// A run observed through the panel: every tick, the view with everything armed, checked.
struct Observed {
    std::set<PanelScreen> screens;
    std::vector<PanelRect> abortRects[8];  // per PanelScreen
    int views = 0;
};

void observe(CalibRig& rig, Observed& seen) {
    const PanelView v = viewFor(rig.cal, kArmed);
    ++seen.views;
    seen.screens.insert(v.screen);
    checkText(v);
    bool abort = false;
    bool runAgain = false;
    for (std::size_t i = 0; i < v.buttonCount; ++i) {
        const PanelButton& b = v.buttons[i];
        CAPTURE(static_cast<int>(v.screen));
        CAPTURE(static_cast<int>(b.id));
        CHECK(b.armed);
        CHECK(buttonMeansSomething(b, rig.cal));
        if (b.id == PanelButtonId::Abort) {
            abort = true;
            seen.abortRects[static_cast<std::size_t>(v.screen)].push_back(b.rect);
        }
        runAgain = runAgain || b.id == PanelButtonId::RunAgain;
    }
    CHECK(abort == running(rig.cal.phase()));
    CHECK(runAgain == !running(rig.cal.phase()));
}

/// Drive a run to its end, observing the panel before every tick and once at the end.
Phase observedRun(CalibRig& rig, RunOptions opt, Observed& seen) {
    const auto inner = opt.hook;
    opt.hook = [&seen, inner](CalibRig& r, const Script& s) {
        observe(r, seen);
        if (inner) {
            inner(r, s);
        }
    };
    rig.cal.start(facts());
    const Phase end = runCalibration(rig, opt);
    observe(rig, seen);
    return end;
}

/// Tap the CENTRE of a button: a new release at its middle.
[[nodiscard]] TouchSample tapAt(std::int32_t& releases, const PanelRect& r) {
    return TouchSample{false, ++releases, static_cast<std::int16_t>((r.x0 + r.x1) / 2),
                       static_cast<std::int16_t>((r.y0 + r.y1) / 2)};
}

}  // namespace

// ═══ P1. Every live button means something, in every reachable state ══════════════
TEST_CASE("CalibratePanel P1: every button drawn live is accepted by the calibrator, in every "
          "reachable state of seven runs; every line fits and stays clear of the buttons") {
    Observed seen;
    {
        CalibRig rig{CalibratorConfig{}, calibPlant(0.0)};
        REQUIRE(observedRun(rig, RunOptions{}, seen) == Phase::Done);
    }
    {
        CalibRig rig{CalibratorConfig{}, calibPlant(0.0, kUsualKs, 0.7)};
        rig.setFault(1, 2, MemberFault::Decoupled);
        REQUIRE(observedRun(rig, RunOptions{}, seen) == Phase::Done);
    }
    {
        CalibRig rig{CalibratorConfig{}, calibPlant(1.0)};  // rolls: stopped at the first pulse
        REQUIRE(observedRun(rig, RunOptions{}, seen) == Phase::Aborted);
    }
    {
        CalibRig rig{CalibratorConfig{}, calibPlant(0.0)};  // a curved push
        RunOptions opt;
        opt.curve = 0.85;
        REQUIRE(observedRun(rig, opt, seen) == Phase::Aborted);
    }
    {
        CalibRig rig{CalibratorConfig{}, calibPlant(0.0)};  // DONE before the push (an old screen)
        RunOptions opt;
        opt.pushInches = 0.0;
        opt.waitForReady = false;
        REQUIRE(observedRun(rig, opt, seen) == Phase::Aborted);
    }
    {
        CalibRig rig{CalibratorConfig{}, calibPlant(0.0)};  // a fight in the spin
        RunOptions opt;
        opt.hook = [](CalibRig& r, const Script&) {
            if (r.cal.phase() == Phase::SpinTest) {
                r.setFault(1, 4, MemberFault::SignFlipped);
            }
        };
        REQUIRE(observedRun(rig, opt, seen) == Phase::Aborted);
    }
    {
        CalibRig rig{CalibratorConfig{}, calibPlant(0.0)};  // ABORT in the middle of the pulses
        RunOptions opt;
        opt.hook = [](CalibRig& r, const Script&) {
            if (r.cal.status().pulseOrdinal == 4 && r.cal.status().pulsing) {
                r.cal.answer(Answer::Abort);
            }
        };
        REQUIRE(observedRun(rig, opt, seen) == Phase::Aborted);
    }
    // Every screen was reached, so "every reachable state" is not a hope.
    for (const PanelScreen s : {PanelScreen::ImuWait, PanelScreen::Push, PanelScreen::WhichEnd,
                                PanelScreen::Pulses, PanelScreen::Spin, PanelScreen::Saved,
                                PanelScreen::Stopped}) {
        CAPTURE(static_cast<int>(s));
        CHECK(seen.screens.count(s) == 1);
    }
    MESSAGE("views checked: ", seen.views);
    CHECK(seen.views > 10000);  // one per tick of seven runs (17504 when written) — not a sample

    // The NotStarted screen, which no run shows for longer than a construction.
    CalibRig idle{CalibratorConfig{}, calibPlant(0.0)};
    const PanelView v = viewFor(idle.cal, kArmed);
    CHECK(v.screen == PanelScreen::NotStarted);
    checkText(v);
}

// ═══ P2. DONE only when the push is ready ═══════════════════════════════════════════
TEST_CASE("CalibratePanel P2: DONE is not drawn at 0 or 1 moved, nor before far enough, nor while "
          "the robot moves — and is drawn once pushReady() says so") {
    SUBCASE("0 moved, then 1 moved") {
        CalibRig rig{CalibratorConfig{}, calibPlant(0.0)};
        rig.cal.start(facts());
        for (int t = 0; t < 100 && rig.cal.phase() != Phase::PushCapture; ++t) {
            rig.cal.tick();
            rig.h.plant().step(Time{kDt});
        }
        PanelView v = viewFor(rig.cal, kArmed);
        CHECK(v.screen == PanelScreen::Push);
        CHECK(findButton(v, PanelButtonId::Done) == nullptr);
        CHECK(hasLine(v, "0 of 12 turned"));
        CHECK(hasLine(v, "DONE shows once the motors turn"));
        rig.idle[0].setPosition(shulib::units::AngleDim{40.0});
        rig.cal.tick();
        v = viewFor(rig.cal, kArmed);
        CHECK(hasLine(v, "1 of 12 turned"));
        CHECK(findButton(v, PanelButtonId::Done) == nullptr);
    }
    SUBCASE("a short push that has STOPPED: every port moved, at rest, not far enough") {
        // Added after a mutation came back GREEN here: dropping "far enough" from pushReady() went
        // unseen by the subcase below, whose not-far-enough moment is always mid-push, where the
        // at-rest clause hides it. A push that stops short is the case that isolates the clause.
        CalibRig rig{CalibratorConfig{}, calibPlant(0.0)};
        rig.cal.start(facts());
        RunOptions opt;
        opt.pushInches = 4.0;
        // Stop the script the moment the robot is still — BEFORE its operator could touch DONE —
        // so what is judged is the SCREEN, not an operator who took a DONE the screen offered.
        opt.stopWhen = [](CalibRig& r) {
            return r.cal.status().movedNow == 10 && r.trueForwardSpeed() == 0.0;
        };
        REQUIRE(runCalibration(rig, opt) == Phase::PushCapture);
        rig.cal.tick();
        REQUIRE(rig.cal.status().atRest);
        const PanelView v = viewFor(rig.cal, kArmed);
        CHECK_FALSE(rig.cal.status().pushFarEnough);
        CHECK(findButton(v, PanelButtonId::Done) == nullptr);
        CHECK(hasLine(v, "keep pushing -- the full 48 in"));
    }
    SUBCASE("every port moved, not far enough, then far enough but moving, then ready") {
        CalibRig rig{CalibratorConfig{}, calibPlant(0.0)};
        rig.cal.start(facts());
        RunOptions opt;
        opt.stopWhen = [](CalibRig& r) { return r.cal.status().movedNow == 10; };
        REQUIRE(runCalibration(rig, opt) == Phase::PushCapture);
        PanelView v = viewFor(rig.cal, kArmed);
        CHECK_FALSE(rig.cal.status().pushFarEnough);
        CHECK(findButton(v, PanelButtonId::Done) == nullptr);
        CHECK(hasLine(v, "keep pushing -- the full 48 in"));
        opt.stopWhen = [](CalibRig& r) { return r.cal.status().pushFarEnough; };
        REQUIRE(runCalibration(rig, opt) == Phase::PushCapture);
        v = viewFor(rig.cal, kArmed);
        CHECK_FALSE(rig.cal.status().atRest);
        CHECK(findButton(v, PanelButtonId::Done) == nullptr);
        CHECK(hasLine(v, "let go -- DONE shows when it stops"));
        opt.stopWhen = [](CalibRig& r) { return r.cal.pushReady(); };
        REQUIRE(runCalibration(rig, opt) == Phase::PushCapture);
        rig.cal.tick();
        v = viewFor(rig.cal, kArmed);
        const PanelButton* done = findButton(v, PanelButtonId::Done);
        REQUIRE(done != nullptr);
        CHECK(done->rect == kPrimarySlot);
        CHECK(hasLine(v, "touch DONE -- then say which end"));
        CHECK(v.lines[2].tone == PanelTone::Green);  // the live count turns green when ready
    }
}

// ═══ P3. ABORT never moves ══════════════════════════════════════════════════════════
TEST_CASE("CalibratePanel P3: ABORT has the identical rectangle on every screen that shows it; the "
          "powered screens widen it to the whole row, which contains that rectangle") {
    Observed seen;
    CalibRig rig{CalibratorConfig{}, calibPlant(0.0)};
    REQUIRE(observedRun(rig, RunOptions{}, seen) == Phase::Done);
    CalibRig idle{CalibratorConfig{}, calibPlant(0.0)};
    seen.abortRects[static_cast<std::size_t>(PanelScreen::NotStarted)].push_back(
        findButton(viewFor(idle.cal, kArmed), PanelButtonId::Abort)->rect);
    for (const PanelScreen s : {PanelScreen::NotStarted, PanelScreen::ImuWait, PanelScreen::Push,
                                PanelScreen::WhichEnd}) {
        CAPTURE(static_cast<int>(s));
        const auto& rects = seen.abortRects[static_cast<std::size_t>(s)];
        REQUIRE_FALSE(rects.empty());
        for (const PanelRect& r : rects) {
            CHECK(r == kAbortSlot);
        }
    }
    for (const PanelScreen s : {PanelScreen::Pulses, PanelScreen::Spin}) {
        CAPTURE(static_cast<int>(s));
        const auto& rects = seen.abortRects[static_cast<std::size_t>(s)];
        REQUIRE_FALSE(rects.empty());
        for (const PanelRect& r : rects) {
            CHECK(r == kAbortRow);
            CHECK(within(kAbortSlot, r));  // a thumb on the usual ABORT spot still hits ABORT
        }
    }
    CHECK(seen.abortRects[static_cast<std::size_t>(PanelScreen::Saved)].empty());
    CHECK(seen.abortRects[static_cast<std::size_t>(PanelScreen::Stopped)].empty());
}

// ═══ P4. The double-tap hazard ══════════════════════════════════════════════════════
// On robot two, FRONT FIRST appeared exactly where DONE had just been; every FRONT answer that
// night landed inside DONE's old box. A reflexive second tap would answer the next question.
TEST_CASE("CalibratePanel P4: no button of a new layout lies under the button whose touch caused "
          "the change — a second tap there lands on nothing, even after the arming delay") {
    struct Transition {
        const char* name;
        PanelButtonId tap;
        RunOptions opt;
        double kappa;
    };
    std::vector<Transition> cases;
    {
        RunOptions o;  // DONE → which end
        cases.push_back({"DONE -> WHICH END", PanelButtonId::Done, o, 0.0});
    }
    {
        RunOptions o;  // DONE on a curved push → STOPPED
        o.curve = 0.85;
        cases.push_back({"DONE -> STOPPED (curved)", PanelButtonId::Done, o, 0.0});
    }
    {
        RunOptions o;
        o.front = Answer::FrontFirst;
        cases.push_back({"FRONT -> PULSES", PanelButtonId::Front, o, 0.0});
    }
    {
        RunOptions o;
        o.front = Answer::BackFirst;
        cases.push_back({"BACK -> PULSES", PanelButtonId::Back, o, 0.0});
    }
    {
        RunOptions o;
        cases.push_back({"ABORT (push) -> STOPPED", PanelButtonId::Abort, o, 0.0});
    }
    {
        RunOptions o;
        cases.push_back({"ABORT (pulses) -> STOPPED", PanelButtonId::Abort, o, 0.0});
    }
    for (Transition& t : cases) {
        CAPTURE(std::string{t.name});
        CalibRig rig{CalibratorConfig{}, calibPlant(t.kappa)};
        rig.cal.start(facts());
        std::int32_t releases = 0;
        TouchGate gate{releases};
        double now = 0.0;
        // Drive the script up to the moment the tapped button is on screen and armed.
        const std::string name = t.name;
        const PanelButtonId tapId = t.tap;
        RunOptions opt = t.opt;
        opt.stopWhen = [&](CalibRig& r) {
            const PanelView v = viewFor(r.cal, kArmed);
            if (name == "ABORT (pulses) -> STOPPED") {
                return v.screen == PanelScreen::Pulses;
            }
            if (name == "ABORT (push) -> STOPPED") {
                return v.screen == PanelScreen::Push;
            }
            return findButton(v, tapId) != nullptr;
        };
        REQUIRE(runCalibration(rig, opt) != Phase::Aborted);
        rig.cal.tick();
        // The screen as drawn, armed (the layout has been up for a while).
        (void)gate.view(rig.cal, now);
        now += 1.0;
        const PanelView before = gate.view(rig.cal, now);
        const PanelButton* b = findButton(before, tapId);
        REQUIRE(b != nullptr);
        REQUIRE(b->armed);
        const PanelRect causing = b->rect;
        const auto first = gate.touch(tapAt(releases, causing), before, rig.cal);
        REQUIRE(first.action == TouchAction::Answer);
        rig.cal.answer(first.answer);
        // The very next frame: a new layout.
        now += 0.01;
        const PanelView after = gate.view(rig.cal, now);
        CHECK(layoutKey(after) != layoutKey(before));
        for (std::size_t i = 0; i < after.buttonCount; ++i) {
            CAPTURE(static_cast<int>(after.buttons[i].id));
            CHECK_FALSE(overlaps(hitBox(after.buttons[i].rect), hitBox(causing)));
        }
        // The reflexive second tap, at the same spot: nothing — at once, and after arming too.
        CHECK(gate.touch(tapAt(releases, causing), after, rig.cal).action == TouchAction::None);
        now += kArmingDelaySeconds + 0.1;
        const PanelView armed = gate.view(rig.cal, now);
        const auto late = gate.touch(tapAt(releases, causing), armed, rig.cal);
        CHECK(late.action == TouchAction::None);
        CHECK(late.hit == -1);
    }
}

// ═══ P5. The arming delay ═══════════════════════════════════════════════════════════
TEST_CASE("CalibratePanel P5: a tap on a new button within the arming delay lands on nothing; at "
          "the delay it counts; ABORT is never delayed") {
    CalibRig rig{CalibratorConfig{}, calibPlant(0.0)};
    rig.cal.start(facts());
    RunOptions opt;
    opt.stopWhen = [](CalibRig& r) { return r.cal.pushReady(); };
    REQUIRE(runCalibration(rig, opt) == Phase::PushCapture);
    rig.cal.tick();
    std::int32_t releases = 7;
    TouchGate gate{releases};
    double now = 100.0;
    PanelView v = gate.view(rig.cal, now);  // DONE has just appeared: greyed
    const PanelButton* done = findButton(v, PanelButtonId::Done);
    REQUIRE(done != nullptr);
    CHECK_FALSE(done->armed);
    CHECK(findButton(v, PanelButtonId::Abort)->armed);  // never delayed
    CHECK(gate.touch(tapAt(releases, kPrimarySlot), v, rig.cal).action == TouchAction::None);
    now += 0.3;
    v = gate.view(rig.cal, now);
    CHECK_FALSE(findButton(v, PanelButtonId::Done)->armed);
    CHECK(gate.touch(tapAt(releases, kPrimarySlot), v, rig.cal).action == TouchAction::None);
    now += kArmingDelaySeconds - 0.3;
    v = gate.view(rig.cal, now);
    REQUIRE(findButton(v, PanelButtonId::Done)->armed);
    auto r = gate.touch(tapAt(releases, kPrimarySlot), v, rig.cal);
    REQUIRE(r.action == TouchAction::Answer);
    CHECK(r.answer == Answer::Done);
    rig.cal.answer(r.answer);
    // The new layout (FRONT / BACK): greyed at once — but ABORT answers at 0.0 s.
    now += 0.01;
    v = gate.view(rig.cal, now);
    REQUIRE(v.screen == PanelScreen::WhichEnd);
    CHECK_FALSE(findButton(v, PanelButtonId::Front)->armed);
    CHECK(gate.touch(tapAt(releases, kFrontBox), v, rig.cal).action == TouchAction::None);
    r = gate.touch(tapAt(releases, kAbortSlot), v, rig.cal);
    CHECK(r.action == TouchAction::Answer);
    CHECK(r.answer == Answer::Abort);
}

// ═══ P6. The touch discipline ═══════════════════════════════════════════════════════
TEST_CASE("CalibratePanel P6: release edge only, stale coordinates never act, an error read is "
          "never replayed, and a stale view is re-checked against the calibrator") {
    CalibRig rig{CalibratorConfig{}, calibPlant(0.0)};
    rig.cal.start(facts());
    RunOptions opt;
    opt.stopWhen = [](CalibRig& r) { return r.cal.pushReady(); };
    REQUIRE(runCalibration(rig, opt) == Phase::PushCapture);
    rig.cal.tick();
    const PanelView v = viewFor(rig.cal, kArmed);
    REQUIRE(findButton(v, PanelButtonId::Done) != nullptr);
    const std::int16_t cx = (kPrimarySlot.x0 + kPrimarySlot.x1) / 2;
    const std::int16_t cy = (kPrimarySlot.y0 + kPrimarySlot.y1) / 2;

    TouchGate gate{5};
    // The panel appears with release_count 5 and x/y on DONE from a tap BEFORE the panel: never
    // an answer.
    CHECK(gate.touch(TouchSample{false, 5, cx, cy}, v, rig.cal).action == TouchAction::None);
    // An error read (every field −1) is ignored, and does not move the count …
    CHECK(gate.touch(TouchSample{true, -1, -1, -1}, v, rig.cal).action == TouchAction::None);
    // … so the next good read, still at count 5, replays nothing.
    const auto replay = gate.touch(TouchSample{false, 5, cx, cy}, v, rig.cal);
    CHECK(replay.action == TouchAction::None);
    CHECK_FALSE(replay.newRelease);
    // A real release: one answer, once.
    const auto real = gate.touch(TouchSample{false, 6, cx, cy}, v, rig.cal);
    CHECK(real.newRelease);
    CHECK(real.action == TouchAction::Answer);
    CHECK(real.answer == Answer::Done);
    CHECK(gate.touch(TouchSample{false, 6, cx, cy}, v, rig.cal).action == TouchAction::None);
    // A STALE view: DONE is still painted, but the calibrator has moved on (the answer above was
    // sent). A tap on the painted DONE must not reach answer() — it would REFUSE the run.
    rig.cal.answer(real.answer);
    REQUIRE(rig.cal.accepts(Answer::FrontFirst));
    const auto stale = gate.touch(TouchSample{false, 7, cx, cy}, v, rig.cal);
    CHECK(stale.hit == 0);                        // it hit the painted DONE …
    CHECK(stale.action == TouchAction::None);     // … and nothing was sent
    CHECK(rig.cal.phase() == Phase::PushCapture);
    // RUN AGAIN painted on a stale view while the run is still going: nothing.
    PanelView fake = v;
    fake.buttons[0] = PanelButton{PanelButtonId::RunAgain, kRunAgainBox, "RUN AGAIN", true};
    CHECK(gate.touch(TouchSample{false, 8, (kRunAgainBox.x0 + kRunAgainBox.x1) / 2,
                                 (kRunAgainBox.y0 + kRunAgainBox.y1) / 2},
                     fake, rig.cal)
              .action
          == TouchAction::None);
}

// ═══ P7. hitTest's padding ══════════════════════════════════════════════════════════
TEST_CASE("CalibratePanel P7: a touch hits a button up to its padding and no further; a greyed "
          "button never hits") {
    PanelView v{};
    v.buttonCount = 2;
    v.buttons[0] = PanelButton{PanelButtonId::Done, kPrimarySlot, "DONE", true};
    v.buttons[1] = PanelButton{PanelButtonId::Abort, kAbortSlot, "ABORT", true};
    const int mid = (kPrimarySlot.x0 + kPrimarySlot.x1) / 2;
    CHECK(hitTest(v, mid, kPrimarySlot.y0 - kHitPadY) == 0);      // the robot's top-edge release
    CHECK(hitTest(v, mid, kPrimarySlot.y0 - kHitPadY - 1) == -1);
    CHECK(hitTest(v, mid, kPrimarySlot.y1 + kHitPadY) == 0);
    CHECK(hitTest(v, mid, kPrimarySlot.y1 + kHitPadY + 1) == -1);
    CHECK(hitTest(v, kPrimarySlot.x1 + kHitPadX, 200) == 0);
    CHECK(hitTest(v, kPrimarySlot.x1 + kHitPadX + 1, 200) == -1);  // the gap belongs to nobody
    CHECK(hitTest(v, kAbortSlot.x0 - kHitPadX, 200) == 1);
    CHECK(hitTest(v, kAbortSlot.x0 - kHitPadX - 1, 200) == -1);
    v.buttons[0].armed = false;
    CHECK(hitTest(v, mid, 200) == -1);  // greyed: drawn, never hit
    CHECK(hitTest(v, 400, 200) == 1);
}

// ═══ P8. wrapWords ══════════════════════════════════════════════════════════════════
TEST_CASE("CalibratePanel P8: wrapWords breaks only at spaces, never splits a word or a number, "
          "and never drops text silently") {
    constexpr auto w = wrapWords<20, 3>("ipr L 1.37500 R 1.37500, track 13.700 in");
    static_assert(w.complete);  // compile-time wrapping is available to a caller's own checks
    CHECK(w.count == 3);
    CHECK(std::string{w.line[0]} == "ipr L 1.37500 R");
    CHECK(std::string{w.line[1]} == "1.37500, track");
    CHECK(std::string{w.line[2]} == "13.700 in");
    // No text lost: the lines rejoin to the input.
    CHECK(std::string{w.line[0]} + " " + w.line[1] + " " + w.line[2]
          == "ipr L 1.37500 R 1.37500, track 13.700 in");
    // Runs of spaces collapse; leading and trailing spaces vanish.
    const auto s = wrapWords<10, 2>("  ab    cd  ");
    CHECK(s.complete);
    CHECK(s.count == 1);
    CHECK(std::string{s.line[0]} == "ab cd");
    // A word longer than a line is never split: the wrap stops and says so.
    const auto longWord = wrapWords<8, 3>("port 18 1234567890 x");
    CHECK_FALSE(longWord.complete);
    CHECK(longWord.count == 1);
    CHECK(std::string{longWord.line[0]} == "port 18");
    // More text than lines: what fitted, in order, and the flag.
    const auto tooMuch = wrapWords<5, 2>("aa bb cc dd ee");
    CHECK_FALSE(tooMuch.complete);
    CHECK(tooMuch.count == 2);
    CHECK(std::string{tooMuch.line[0]} == "aa bb");
    CHECK(std::string{tooMuch.line[1]} == "cc dd");
    // Exactly full is complete; empty is zero lines.
    CHECK(wrapWords<5, 1>("aa bb").complete);
    CHECK(wrapWords<5, 1>("").count == 0);
    CHECK(wrapWords<5, 1>("   ").count == 0);
    // Every real refusal wraps into the STOPPED screen's SMALL lines without splitting a word.
    const auto reason = wrapWords<kSmallCols, 6>(
        "port 13 is still turning at 3.140 rad/s after 3.0 s of waiting for rest before pulsing "
        "port 14 -- a motor that reports motion while nothing is driven is a frozen reading or a "
        "moving robot");
    CHECK(reason.complete);
    for (std::size_t k = 0; k < reason.count; ++k) {
        CHECK(std::strlen(reason.line[k]) <= kSmallCols);
    }
}

// ═══ P9. The literal checks ═════════════════════════════════════════════════════════
TEST_CASE("CalibratePanel P9: every literal fits its budget and is ASCII without a percent sign; "
          "the header's source carries no percent format and no non-ASCII string literal") {
    CHECK(shulib::calibration::panelLiteralsFit());
    CHECK(shulib::calibration::panelLayoutHolds());
    std::set<std::string> headlines;
    for (int r = 1; r <= static_cast<int>(Refusal::InvalidDescription); ++r) {
        const auto w = stoppedWords(static_cast<Refusal>(r));
        CAPTURE(r);
        CHECK(std::strlen(w.headline) <= kLargeCols);
        CHECK(std::strlen(w.why) <= kMediumCols);
        CHECK(std::strlen(w.doLine) <= kMediumCols);
        CHECK(headlines.insert(w.headline).second);  // one headline per code: they say different things
    }
    // The source itself: no "%%" (the only way a format prints a percent sign), and every string
    // literal ASCII. (Comments keep their typography; they are read in an editor, not on a V5.)
    std::ifstream in{std::string{SHULIB_SOURCE_DIR} + "/include/shulib/calibration/calibrate_panel.hpp"};
    REQUIRE(in.good());
    std::string line;
    int literals = 0;
    while (std::getline(in, line)) {
        CHECK(line.find("%%") == std::string::npos);
        bool inString = false;
        for (std::size_t i = 0; i < line.size(); ++i) {
            const char c = line[i];
            if (!inString && c == '/' && i + 1 < line.size() && line[i + 1] == '/') {
                break;  // the rest of the line is a comment
            }
            if (c == '"' && (i == 0 || line[i - 1] != '\\') && !(i > 0 && line[i - 1] == '\'')) {
                inString = !inString;
                literals += inString ? 1 : 0;
                continue;
            }
            if (inString) {
                CAPTURE(line);
                CHECK(static_cast<unsigned char>(c) < 0x80);
            }
        }
    }
    CHECK(literals > 40);
}

// ═══ P10. The STOPPED screen ════════════════════════════════════════════════════════
TEST_CASE("CalibratePanel P10: the STOPPED screen says its code's words, names the port, and shows "
          "the reason wrapped whole") {
    struct Row {
        Refusal code;
        int port;
    };
    CalibRig rolled{CalibratorConfig{}, calibPlant(1.0)};
    rolled.cal.start(facts());
    (void)runCalibration(rolled, RunOptions{});
    CalibRig curved{CalibratorConfig{}, calibPlant(0.0)};
    curved.cal.start(facts());
    RunOptions c;
    c.curve = 0.85;
    (void)runCalibration(curved, c);
    for (CalibRig* rig : {&rolled, &curved}) {
        REQUIRE(rig->cal.phase() == Phase::Aborted);
        const PanelView v = viewFor(rig->cal, kArmed);
        CHECK(v.screen == PanelScreen::Stopped);
        CHECK(v.headerTone == PanelTone::Red);
        CHECK(std::string{v.headerWord} == "STOPPED -- MOTORS OFF");
        const auto words = stoppedWords(rig->cal.refusal());
        CHECK(hasLine(v, words.headline));
        CHECK(hasLine(v, words.why));
        CHECK(hasLine(v, words.doLine));
        if (rig->cal.refusalPort() > 0) {
            CHECK(hasLine(v, ("PORT " + std::to_string(rig->cal.refusalPort())).c_str()));
        }
        // The SMALL lines rejoin to the reason (when it fits three lines) — nothing split, nothing
        // lost; or the third says where the rest is.
        std::string joined;
        bool pointer = false;
        for (std::size_t i = 0; i < v.lineCount; ++i) {
            if (v.lines[i].size == TextSize::Small) {
                if (std::strcmp(v.lines[i].text, "the full reason is on the serial log") == 0) {
                    pointer = true;
                    continue;
                }
                joined += (joined.empty() ? "" : " ") + std::string{v.lines[i].text};
            }
        }
        std::string reason{rig->cal.reason()};
        if (!pointer) {
            CHECK(joined == reason);
        } else {
            CHECK(reason.rfind(joined, 0) == 0);  // what is shown is the reason's own beginning
        }
        checkText(v);
    }
    CHECK(rolled.cal.refusal() == Refusal::PulseRolled);
    CHECK(hasLine(viewFor(rolled.cal, kArmed), "IT ROLLED"));
    CHECK(hasLine(viewFor(rolled.cal, kArmed), "PORT 11"));
    CHECK(hasLine(viewFor(curved.cal, kArmed), "PUSH CURVED"));
}

// ═══ P11. The finished screen ═══════════════════════════════════════════════════════
TEST_CASE("CalibratePanel P11: the finished screen shows the counts, the scales, the width and the "
          "LEFT OUT port, headed per SaveOutcome") {
    CalibRig rig{CalibratorConfig{}, calibPlant(0.0, kUsualKs, 0.7)};
    rig.setFault(1, 2, MemberFault::Decoupled);
    rig.cal.start(facts());
    REQUIRE(runCalibration(rig, RunOptions{}) == Phase::Done);
    const PanelView saved = viewFor(rig.cal, kArmed, SaveOutcome::Saved);
    CHECK(saved.screen == PanelScreen::Saved);
    CHECK(saved.headerTone == PanelTone::Green);
    CHECK(std::string{saved.headerWord} == "SAVED");
    CHECK(hasLine(saved, "LEFT 5  RIGHT 4"));
    CHECK(hasLine(saved, "ipr L 1.62500 R 1.62500"));
    CHECK(hasLine(saved, "track 12.500 in"));
    CHECK(hasLine(saved, "LEFT OUT 18"));
    CHECK(hasLine(saved, "NEXT: copy these, then RUN AGAIN to check"));
    CHECK(hasLine(saved, "port 18 turns but does not move the robot -- its"));
    CHECK(hasLine(saved, "gear is slipping; it is left out"));
    REQUIRE(saved.buttonCount == 1);
    CHECK(saved.buttons[0].id == PanelButtonId::RunAgain);
    CHECK(saved.buttons[0].rect == kRunAgainBox);
    checkText(saved);
    const PanelView notSaved = viewFor(rig.cal, kArmed, SaveOutcome::NotSaved);
    CHECK(notSaved.headerTone == PanelTone::Red);
    CHECK(std::string{notSaved.headerWord} == "NOT SAVED");
    CHECK(hasLine(notSaved, "NOT SAVED -- copy these numbers NOW"));
    const PanelView measured = viewFor(rig.cal, kArmed);
    CHECK(measured.headerTone == PanelTone::Blue);
    CHECK(std::string{measured.headerWord} == "MEASURED");
}

// ═══ P12. The layout clock ══════════════════════════════════════════════════════════
TEST_CASE("CalibratePanel P12: TouchGate::view greys a layout for the arming delay and re-greys on "
          "every layout change, never on a merely changing number") {
    CalibRig rig{CalibratorConfig{}, calibPlant(0.0)};
    rig.cal.start(facts());
    for (int t = 0; t < 100 && rig.cal.phase() != Phase::PushCapture; ++t) {
        rig.cal.tick();
        rig.h.plant().step(Time{kDt});
    }
    TouchGate gate{0};
    PanelView v = gate.view(rig.cal, 0.0);
    REQUIRE(v.screen == PanelScreen::Push);
    CHECK(findButton(v, PanelButtonId::Abort)->armed);
    // Push the robot a little: the count changes, the layout does not — no re-greying.
    rig.pushPrimaries(2.2, 2.2);
    for (int t = 0; t < 30; ++t) {
        rig.cal.tick();
        rig.h.plant().step(Time{kDt});
    }
    rig.stopPush();
    const PanelView mid = gate.view(rig.cal, 0.3);
    CHECK(layoutKey(mid) == layoutKey(v));
    // A layout that stays up arms at the delay, not before.
    RunOptions opt;
    opt.stopWhen = [](CalibRig& r) { return r.cal.pushReady(); };
    REQUIRE(runCalibration(rig, opt) == Phase::PushCapture);
    rig.cal.tick();
    v = gate.view(rig.cal, 10.0);  // DONE appears: a layout change
    CHECK_FALSE(findButton(v, PanelButtonId::Done)->armed);
    v = gate.view(rig.cal, 10.0 + kArmingDelaySeconds - 0.01);
    CHECK_FALSE(findButton(v, PanelButtonId::Done)->armed);
    v = gate.view(rig.cal, 10.0 + kArmingDelaySeconds);
    CHECK(findButton(v, PanelButtonId::Done)->armed);
    // The robot moves again: DONE vanishes (a layout change) and comes back GREYED.
    rig.pushPrimaries(2.2, 2.2);
    rig.cal.tick();
    rig.h.plant().step(Time{kDt});
    rig.cal.tick();
    v = gate.view(rig.cal, 11.0);
    CHECK(findButton(v, PanelButtonId::Done) == nullptr);
    rig.stopPush();
    rig.h.plant().step(Time{kDt});
    rig.cal.tick();
    v = gate.view(rig.cal, 11.1);
    REQUIRE(findButton(v, PanelButtonId::Done) != nullptr);
    CHECK_FALSE(findButton(v, PanelButtonId::Done)->armed);
}
