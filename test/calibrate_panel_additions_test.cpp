// calibration::viewFor / TouchGate — CAL1 Part A2a's ADDITIONS to the calibration panel (the brief's
// §2.A test 9, and the panel's half of the held screens), driven over the same plant and scripted
// operator as the panel's own cases (calibration_test_rig.hpp). NOTHING HERE HAS RUN ON A ROBOT.
// test/calibrate_panel_test.cpp is today's panel's, and is not edited (§2.A test 9: no existing
// test TU is).
//
// Each case names the bug it catches:
//   N1  the DEFAULTS ARE TODAY'S SCREENS: every view of four runs (clean, slipping, curved, aborted),
//       at three arming times and every SaveOutcome today's program passes, is the same field for
//       field whether viewFor()/TouchGate::view() are called as today's program calls them or with
//       FrontRule::AskedAfterPush and FinishFacts{} spelled out — so a default flipped to the
//       standard Calibrate's (ByPush) turns this red
//   N2  ByPush: never the WHICH END view or a FRONT/BACK button, in every state of a run; the four
//       steps 1/4..4/4; the front rule while the IMU warms up; PUSH FRONT FIRST; the DONE line
//   N3  the held views: FRONT FLIPPED's and MECHANISM PORT TURNED's words, sizes and places, the
//       left box (today's FRONT FIRST id) labelled USE NEW / SAVE and RUN AGAIN beside it, every
//       line inside its budget and clear of both boxes
//   N4  TouchGate on a held screen: the left box is SaveHeld (never an answer), RUN AGAIN is
//       RunAgain, and neither acts unless the run has ended
//   N5  the arming clock restarts when the HOLD's question changes — FRONT FLIPPED's USE NEW and
//       MECHANISM PORT TURNED's SAVE share one box, one id and one layoutKey(), so a reflexive second
//       tap must land on a GREYED box
//   N5b one MECHANISM PORT TURNED question PER PORT (the CAL1-A2a review): a new holdPort restarts
//       the clock as a new reason does — same reason, same box, same layoutKey() — and the same port
//       does not; FRONT FLIPPED reads no port
//   N6  the finished screen: the NOTE takes the third SMALL row, a two-row calibrator warning keeps
//       one row cut with " ...", a one-row warning stays whole; USE NEW's SAVED header is amber
//       "SAVED -- NEW FRONT" and a NOT SAVED after it still says NOT SAVED

#include "doctest.h"

#include <cstring>
#include <string>
#include <vector>

#include "calibration_test_rig.hpp"
#include "shulib/calibration/calibrate_panel.hpp"

using namespace calib_rig;
using shulib::calibration::budgetFor;
using shulib::calibration::FinishFacts;
using shulib::calibration::FrontChoice;
using shulib::calibration::FrontRule;
using shulib::calibration::heightFor;
using shulib::calibration::HoldReason;
using shulib::calibration::kArmingDelaySeconds;
using shulib::calibration::kBackBox;
using shulib::calibration::kFrontBox;
using shulib::calibration::kHeaderCols;
using shulib::calibration::kPanelLastRow;
using shulib::calibration::kPanelWidth;
using shulib::calibration::overlaps;
using shulib::calibration::PanelButton;
using shulib::calibration::PanelButtonId;
using shulib::calibration::PanelRect;
using shulib::calibration::PanelScreen;
using shulib::calibration::PanelTone;
using shulib::calibration::PanelView;
using shulib::calibration::SaveOutcome;
using shulib::calibration::TextSize;
using shulib::calibration::TouchAction;
using shulib::calibration::TouchGate;
using shulib::calibration::TouchSample;
using shulib::calibration::viewFor;

namespace {

constexpr double kArmed = 10.0;

/// Where two views first differ, FIELD FOR FIELD — the screen, the header, every line (place,
/// size, tone, text) and every button (id, rectangle, label, armed), nothing past the counts — or
/// "" when they are the same. (One string per view, so a run of 100,000 views costs one check.)
std::string viewDiff(const PanelView& a, const PanelView& b) {
    if (a.screen != b.screen) {
        return "screen";
    }
    if (a.headerTone != b.headerTone || std::string{a.headerWord} != std::string{b.headerWord}) {
        return std::string{"header "} + a.headerWord + " / " + b.headerWord;
    }
    if (a.lineCount != b.lineCount) {
        return "line count";
    }
    for (std::size_t i = 0; i < a.lineCount; ++i) {
        const auto& x = a.lines[i];
        const auto& y = b.lines[i];
        if (x.x != y.x || x.y != y.y || x.size != y.size || x.tone != y.tone
            || std::string{x.text} != std::string{y.text}) {
            return "line " + std::to_string(i) + ": '" + x.text + "' / '" + y.text + "'";
        }
    }
    if (a.buttonCount != b.buttonCount) {
        return "button count";
    }
    for (std::size_t i = 0; i < a.buttonCount; ++i) {
        const auto& x = a.buttons[i];
        const auto& y = b.buttons[i];
        if (x.id != y.id || !(x.rect == y.rect) || std::string{x.label} != std::string{y.label}
            || x.armed != y.armed) {
            return "button " + std::to_string(i) + ": " + x.label + " / " + y.label;
        }
    }
    return "";
}

void checkSameView(const PanelView& a, const PanelView& b) { CHECK(viewDiff(a, b) == ""); }

/// Views compared and the first that differed (with where), for one CHECK per run.
struct Tally {
    int compared = 0;
    int differing = 0;
    std::string first;
    void add(const PanelView& a, const PanelView& b, const char* what) {
        ++compared;
        const std::string d = viewDiff(a, b);
        if (!d.empty()) {
            if (differing++ == 0) {
                first = std::string{what} + ": " + d;
            }
        }
    }
};

/// The pixel box a line occupies (the panel test's estimate: the budget spread over 472 pixels).
PanelRect lineBox(const shulib::calibration::PanelLine& l) {
    const auto charW = static_cast<int>(472 / budgetFor(l.size));
    const auto len = static_cast<int>(std::strlen(l.text));
    return PanelRect{l.x, l.y, static_cast<std::int16_t>(l.x + len * charW - 1),
                     static_cast<std::int16_t>(l.y + heightFor(l.size) - 1)};
}

/// The panel's text rules for one view: header, every line inside its budget and the screen and
/// clear of every button, every label at most 12.
void checkLayout(const PanelView& v) {
    CHECK(std::strlen(v.headerWord) <= kHeaderCols);
    for (std::size_t i = 0; i < v.lineCount; ++i) {
        const auto& l = v.lines[i];
        CAPTURE(std::string{l.text});
        CHECK(std::strlen(l.text) <= budgetFor(l.size));
        const PanelRect box = lineBox(l);
        CHECK(box.x1 < kPanelWidth);
        CHECK(box.y1 <= kPanelLastRow);
        for (std::size_t b = 0; b < v.buttonCount; ++b) {
            CHECK_FALSE(overlaps(box, v.buttons[b].rect));
        }
    }
    for (std::size_t b = 0; b < v.buttonCount; ++b) {
        CHECK(std::strlen(v.buttons[b].label) <= 12);
    }
}

/// Run `opt` on a fresh rig, calling `each` with the calibrator before every tick and at the end.
template <class Each>
Phase drive(CalibRig& rig, RunOptions opt, Each each) {
    const auto inner = opt.hook;
    opt.hook = [&each, inner](CalibRig& r, const Script& s) {
        each(r.cal);
        if (inner) {
            inner(r, s);
        }
    };
    rig.cal.start(facts());
    const Phase end = runCalibration(rig, opt);
    each(rig.cal);
    return end;
}

[[nodiscard]] bool hasLine(const PanelView& v, const char* text, std::int16_t y = -1) {
    for (std::size_t i = 0; i < v.lineCount; ++i) {
        if (std::strcmp(v.lines[i].text, text) == 0 && (y < 0 || v.lines[i].y == y)) {
            return true;
        }
    }
    return false;
}

[[nodiscard]] const PanelButton* findButton(const PanelView& v, PanelButtonId id) {
    for (std::size_t i = 0; i < v.buttonCount; ++i) {
        if (v.buttons[i].id == id) {
            return &v.buttons[i];
        }
    }
    return nullptr;
}

/// A finished (Done) run's calibrator on `rig` — with robot two's slipping member when `slip` (its
/// plant built with the drag that needs).
void finishClean(CalibRig& rig, bool slip = false) {
    if (slip) {
        rig.setFault(1, 2, MemberFault::Decoupled);
    }
    rig.cal.start(facts());
    REQUIRE(runCalibration(rig, RunOptions{}) == Phase::Done);
}

TouchSample tapAt(std::int32_t& releases, const PanelRect& r) {
    return TouchSample{false, ++releases, static_cast<std::int16_t>((r.x0 + r.x1) / 2),
                       static_cast<std::int16_t>((r.y0 + r.y1) / 2)};
}

}  // namespace

TEST_CASE("CalibratePanel additions N1: the defaulted viewFor() and TouchGate::view() ARE today's screens -- "
          "field for field against AskedAfterPush and FinishFacts{} spelled out, in every state of four runs") {
    Tally tally;
    const auto check = [&tally](const DrivetrainCalibrator& cal) {
        for (const double t : {0.0, 0.25, kArmed}) {
            for (const SaveOutcome s : {SaveOutcome::NotAttempted, SaveOutcome::Saved, SaveOutcome::NotSaved}) {
                const PanelView today = s == SaveOutcome::NotAttempted ? viewFor(cal, t) : viewFor(cal, t, s);
                tally.add(today, viewFor(cal, t, s, FrontRule::AskedAfterPush, FinishFacts{}), "viewFor");
            }
        }
    };
    {
        CalibRig rig{CalibratorConfig{}, calibPlant(0.0)};
        REQUIRE(drive(rig, RunOptions{}, check) == Phase::Done);
    }
    {
        CalibRig rig{CalibratorConfig{}, calibPlant(0.0, kUsualKs, 0.7)};
        rig.setFault(1, 2, MemberFault::Decoupled);
        REQUIRE(drive(rig, RunOptions{}, check) == Phase::Done);
    }
    {
        CalibRig rig{CalibratorConfig{}, calibPlant(0.0)};
        RunOptions opt;
        opt.curve = 0.85;
        REQUIRE(drive(rig, opt, check) == Phase::Aborted);
    }
    {
        CalibRig rig{CalibratorConfig{}, calibPlant(0.0)};
        RunOptions opt;
        opt.hook = [](CalibRig& r, const Script&) {
            if (r.cal.status().pulseOrdinal == 4 && r.cal.status().pulsing) {
                r.cal.answer(Answer::Abort);
            }
        };
        REQUIRE(drive(rig, opt, check) == Phase::Aborted);
    }
    // TouchGate::view(), defaulted and spelled out, over the same calibrator and clock: two gates,
    // the same layout changes, the same arming.
    {
        CalibRig rig{CalibratorConfig{}, calibPlant(0.0)};
        TouchGate a{0};
        TouchGate b{0};
        double now = 0.0;
        REQUIRE(drive(rig, RunOptions{}, [&](const DrivetrainCalibrator& cal) {
                    now += 0.01;
                    tally.add(a.view(cal, now),
                              b.view(cal, now, SaveOutcome::NotAttempted, FrontRule::AskedAfterPush, FinishFacts{}),
                              "TouchGate::view");
                }) == Phase::Done);
    }
    MESSAGE("views compared: ", tally.compared);
    CAPTURE(tally.first);
    CHECK(tally.differing == 0);
    CHECK(tally.compared > 50000);  // every tick of five runs, at three times and three outcomes
}

TEST_CASE("CalibratePanel additions N2: under ByPush the WHICH END view and its FRONT/BACK buttons never "
          "appear; the steps are 1/4..4/4, the front rule shows while the IMU warms up, the push says PUSH "
          "FRONT FIRST") {
    std::vector<std::string> headers;
    bool sawRule = false;
    bool sawPushFront = false;
    bool sawDoneLine = false;
    int views = 0;
    CalibRig rig{CalibratorConfig{}, calibPlant(0.0)};
    REQUIRE(drive(rig, RunOptions{}, [&](const DrivetrainCalibrator& cal) {
                const PanelView v = viewFor(cal, kArmed, SaveOutcome::NotAttempted, FrontRule::ByPush);
                ++views;
                CHECK(v.screen != PanelScreen::WhichEnd);
                CHECK(findButton(v, PanelButtonId::Front) == nullptr);
                CHECK(findButton(v, PanelButtonId::Back) == nullptr);
                checkLayout(v);
                if (headers.empty() || headers.back() != v.headerWord) {
                    headers.emplace_back(v.headerWord);
                }
                if (v.screen == PanelScreen::ImuWait) {
                    sawRule = sawRule || hasLine(v, "the end that leads the push is FRONT", 138);
                }
                if (v.screen == PanelScreen::Push) {
                    sawPushFront = sawPushFront || hasLine(v, "PUSH FRONT FIRST", 30);
                    sawDoneLine = sawDoneLine || hasLine(v, "touch DONE -- then stand clear", 160);
                }
            }) == Phase::Done);
    // the finished header is MEASURED: these views pass SaveOutcome::NotAttempted
    const std::vector<std::string> want = {"1/4 IMU", "2/4 PUSH", "3/4 PULSES", "4/4 SPIN", "MEASURED"};
    CHECK(headers == want);
    CHECK(sawRule);
    CHECK(sawPushFront);
    CHECK(sawDoneLine);
    CHECK(views > 1000);
    // The rule is ByPush's alone: today's IMU screen has no such line.
    CalibRig fresh{CalibratorConfig{}, calibPlant(0.0)};
    fresh.cal.start(facts());  // started, not yet ticked: the IMU step
    REQUIRE(fresh.cal.phase() == Phase::WaitImu);
    CHECK_FALSE(hasLine(viewFor(fresh.cal, kArmed), "the end that leads the push is FRONT"));
    CHECK(std::string{viewFor(fresh.cal, kArmed).headerWord} == "1/5 IMU");
}

TEST_CASE("CalibratePanel additions N3: the held views -- FRONT FLIPPED's and MECHANISM PORT TURNED's words in "
          "their sizes and places, the relabelled left box and RUN AGAIN, every line clear of both") {
    CalibRig rig{CalibratorConfig{}, calibPlant(0.0)};
    finishClean(rig);
    FinishFacts flip;
    flip.hold = HoldReason::FrontFlipped;
    const PanelView f = viewFor(rig.cal, kArmed, SaveOutcome::Held, FrontRule::ByPush, flip);
    CHECK(f.screen == PanelScreen::Held);
    CHECK(f.headerTone == PanelTone::Amber);
    CHECK(std::string{f.headerWord} == "FRONT FLIPPED");
    REQUIRE(f.lineCount == 5);
    CHECK(hasLine(f, "NOTHING SAVED YET", 30));
    CHECK(f.lines[0].size == TextSize::Large);
    CHECK(f.lines[0].tone == PanelTone::Amber);
    CHECK(hasLine(f, "the card calls the end you led BACK", 160));
    CHECK(f.lines[1].size == TextSize::Medium);
    CHECK(hasLine(f, "RUN AGAIN: you led with the BACK by mistake", 180));
    CHECK(hasLine(f, "USE NEW: you led with the FRONT (cables swapped?)", 196));
    CHECK(hasLine(f, "USE NEW after a wrong push reverses every auton", 212));
    CHECK(f.lines[4].tone == PanelTone::Amber);
    REQUIRE(f.buttonCount == 2);
    CHECK(f.buttons[0].id == PanelButtonId::Front);  // today's id, relabelled — no new id before Part B
    CHECK(f.buttons[0].rect == kFrontBox);
    CHECK(std::string{f.buttons[0].label} == "USE NEW");
    CHECK(f.buttons[1].id == PanelButtonId::RunAgain);
    CHECK(f.buttons[1].rect == kBackBox);
    CHECK(std::string{f.buttons[1].label} == "RUN AGAIN");
    checkLayout(f);

    FinishFacts mech;
    mech.hold = HoldReason::MechanismTurned;
    mech.holdLine1 = "port 12: 1692 deg, drive 1692 deg";
    mech.holdLine2 = "the card's record DRIVES with port 12";
    const PanelView m = viewFor(rig.cal, kArmed, SaveOutcome::Held, FrontRule::ByPush, mech);
    CHECK(std::string{m.headerWord} == "MECHANISM PORT TURNED");
    REQUIRE(m.lineCount == 5);
    CHECK(hasLine(m, "NOTHING SAVED YET", 30));
    CHECK(hasLine(m, "port 12: 1692 deg, drive 1692 deg", 160));
    CHECK(hasLine(m, "the card's record DRIVES with port 12", 180));
    CHECK(hasLine(m, "SAVE: it is a mechanism the push turns -- keep it", 196));
    CHECK(hasLine(m, "RUN AGAIN = save nothing: a drive motor? fix facts", 212));
    REQUIRE(m.buttonCount == 2);
    CHECK(m.buttons[0].id == PanelButtonId::Front);
    CHECK(std::string{m.buttons[0].label} == "SAVE");
    CHECK(m.buttons[1].id == PanelButtonId::RunAgain);
    checkLayout(m);
    // no card record that drives with the port: the panel's own line
    mech.holdLine2 = nullptr;
    const PanelView m2 = viewFor(rig.cal, kArmed, SaveOutcome::Held, FrontRule::ByPush, mech);
    CHECK(hasLine(m2, "it is declared a mechanism: not in this record", 180));
    // The SAME layout key for both held questions (why N5's restart exists)
    CHECK(shulib::calibration::layoutKey(f) == shulib::calibration::layoutKey(m));
    // A hold is read only with SaveOutcome::Held: the same facts with Saved paint the SAVED screen
    const PanelView s = viewFor(rig.cal, kArmed, SaveOutcome::Saved, FrontRule::ByPush, flip);
    CHECK(s.screen == PanelScreen::Saved);
}

TEST_CASE("CalibratePanel additions N4: TouchGate on a held screen -- the left box is SaveHeld, never an "
          "answer; RUN AGAIN is RunAgain; neither acts before the run has ended") {
    CalibRig rig{CalibratorConfig{}, calibPlant(0.0)};
    finishClean(rig);
    FinishFacts flip;
    flip.hold = HoldReason::FrontFlipped;
    std::int32_t releases = 0;
    TouchGate gate{0};
    PanelView v = gate.view(rig.cal, 0.0, SaveOutcome::Held, FrontRule::ByPush, flip);
    v = gate.view(rig.cal, 1.0, SaveOutcome::Held, FrontRule::ByPush, flip);  // armed
    REQUIRE(v.buttons[0].armed);
    auto r = gate.touch(tapAt(releases, kFrontBox), v, rig.cal);
    CHECK(r.newRelease);
    CHECK(r.action == TouchAction::SaveHeld);
    CHECK(rig.cal.phase() == Phase::Done);  // nothing was answered to the calibrator
    r = gate.touch(tapAt(releases, kBackBox), v, rig.cal);
    CHECK(r.action == TouchAction::RunAgain);
    // a held view over a calibrator that is still running (a stale view): the left box does nothing
    CalibRig running{CalibratorConfig{}, calibPlant(0.0)};
    running.cal.start(facts());
    running.cal.tick();
    REQUIRE(running.cal.phase() != Phase::Done);
    TouchGate g2{0};
    r = g2.touch(tapAt(releases, kFrontBox), v, running.cal);
    CHECK(r.action == TouchAction::None);
    CHECK(running.cal.phase() != Phase::Done);
}

TEST_CASE("CalibratePanel additions N5: the arming clock RESTARTS when the hold's question changes -- the "
          "box that was USE NEW comes back GREYED as SAVE, so a reflexive second tap lands on nothing") {
    CalibRig rig{CalibratorConfig{}, calibPlant(0.0)};
    finishClean(rig);
    FinishFacts flip;
    flip.hold = HoldReason::FrontFlipped;
    FinishFacts mech;
    mech.hold = HoldReason::MechanismTurned;
    mech.holdLine1 = "port 6: 1692 deg, drive 1692 deg";
    TouchGate gate{0};
    std::int32_t releases = 0;
    PanelView v = gate.view(rig.cal, 0.0, SaveOutcome::Held, FrontRule::ByPush, flip);
    CHECK_FALSE(v.buttons[0].armed);
    v = gate.view(rig.cal, 1.0, SaveOutcome::Held, FrontRule::ByPush, flip);
    REQUIRE(v.buttons[0].armed);
    REQUIRE(gate.touch(tapAt(releases, kFrontBox), v, rig.cal).action == TouchAction::SaveHeld);
    // the second question, 0.1 s later: same box, same id, same layout key — and GREYED
    v = gate.view(rig.cal, 1.1, SaveOutcome::Held, FrontRule::ByPush, mech);
    CHECK(std::string{v.headerWord} == "MECHANISM PORT TURNED");
    CHECK_FALSE(v.buttons[0].armed);
    CHECK_FALSE(v.buttons[1].armed);
    const auto reflex = gate.touch(tapAt(releases, kFrontBox), v, rig.cal);
    CHECK(reflex.newRelease);
    CHECK(reflex.action == TouchAction::None);
    // armed again only after the full delay from the change, never sooner
    v = gate.view(rig.cal, 1.1 + kArmingDelaySeconds - 0.01, SaveOutcome::Held, FrontRule::ByPush, mech);
    CHECK_FALSE(v.buttons[0].armed);
    v = gate.view(rig.cal, 1.1 + kArmingDelaySeconds, SaveOutcome::Held, FrontRule::ByPush, mech);
    CHECK(v.buttons[0].armed);
    CHECK(gate.touch(tapAt(releases, kFrontBox), v, rig.cal).action == TouchAction::SaveHeld);
    // and the same hold, asked again, does NOT restart the clock (only a CHANGE of question does)
    v = gate.view(rig.cal, 3.0, SaveOutcome::Held, FrontRule::ByPush, mech);
    CHECK(v.buttons[0].armed);
}

TEST_CASE("CalibratePanel additions N5b: one MECHANISM PORT TURNED question per PORT -- a new port is a new "
          "question (same reason, same box, same layoutKey()): the clock restarts and SAVE comes back GREYED; "
          "the same port does not restart it, and FRONT FLIPPED ignores holdPort") {
    CalibRig rig{CalibratorConfig{}, calibPlant(0.0)};
    finishClean(rig);
    FinishFacts port6;
    port6.hold = HoldReason::MechanismTurned;
    port6.holdPort = 6;
    port6.holdLine1 = "port 6: 1692 deg, drive 1692 deg";
    FinishFacts port12 = port6;
    port12.holdPort = 12;
    port12.holdLine1 = "port 12: 1692 deg, drive 1692 deg";
    port12.holdLine2 = "the card's record DRIVES with port 12";
    TouchGate gate{0};
    std::int32_t releases = 0;
    PanelView v = gate.view(rig.cal, 0.0, SaveOutcome::Held, FrontRule::ByPush, port6);
    CHECK_FALSE(v.buttons[0].armed);
    v = gate.view(rig.cal, 1.0, SaveOutcome::Held, FrontRule::ByPush, port6);
    REQUIRE(v.buttons[0].armed);
    const std::uint32_t key6 = layoutKey(v);
    REQUIRE(gate.touch(tapAt(releases, kFrontBox), v, rig.cal).action == TouchAction::SaveHeld);
    // port 12's question, 0.1 s later: the same layout key — and GREYED, both boxes
    v = gate.view(rig.cal, 1.1, SaveOutcome::Held, FrontRule::ByPush, port12);
    CHECK(layoutKey(v) == key6);
    CHECK_FALSE(v.buttons[0].armed);
    CHECK_FALSE(v.buttons[1].armed);
    const auto reflex = gate.touch(tapAt(releases, kFrontBox), v, rig.cal);
    CHECK(reflex.newRelease);
    CHECK(reflex.action == TouchAction::None);
    v = gate.view(rig.cal, 1.1 + kArmingDelaySeconds - 0.01, SaveOutcome::Held, FrontRule::ByPush, port12);
    CHECK_FALSE(v.buttons[0].armed);
    v = gate.view(rig.cal, 1.1 + kArmingDelaySeconds, SaveOutcome::Held, FrontRule::ByPush, port12);
    CHECK(v.buttons[0].armed);
    // the same port again: no restart
    v = gate.view(rig.cal, 3.0, SaveOutcome::Held, FrontRule::ByPush, port12);
    CHECK(v.buttons[0].armed);
    // FRONT FLIPPED reads no port: a holdPort beside it changes nothing
    FinishFacts flipA;
    flipA.hold = HoldReason::FrontFlipped;
    flipA.holdPort = 5;
    FinishFacts flipB = flipA;
    flipB.holdPort = 9;
    TouchGate g2{0};
    v = g2.view(rig.cal, 0.0, SaveOutcome::Held, FrontRule::ByPush, flipA);
    v = g2.view(rig.cal, 1.0, SaveOutcome::Held, FrontRule::ByPush, flipA);
    REQUIRE(v.buttons[0].armed);
    v = g2.view(rig.cal, 1.1, SaveOutcome::Held, FrontRule::ByPush, flipB);
    CHECK(v.buttons[0].armed);
}

TEST_CASE("CalibratePanel additions N6: the finished screen -- the NOTE takes the third row and a two-row "
          "warning keeps one, cut with ' ...'; a one-row warning stays whole; USE NEW's SAVED header") {
    SUBCASE("a two-row warning and a NOTE") {
        CalibRig rig{CalibratorConfig{}, calibPlant(0.0, kUsualKs, 0.7)};
        finishClean(rig, true);
        const std::string warning = rig.cal.warning();
        REQUIRE(warning.size() > shulib::calibration::kSmallCols);  // the case needs two rows
        FinishFacts note;
        note.note = "FIRST RECORD -- the end you pushed first is FRONT";
        const PanelView v = viewFor(rig.cal, kArmed, SaveOutcome::Saved, FrontRule::ByPush, note);
        checkLayout(v);
        const PanelView plain = viewFor(rig.cal, kArmed, SaveOutcome::Saved, FrontRule::ByPush);
        // without a NOTE: today's two warning rows, 196 and 212, both amber
        int warningRows = 0;
        for (std::size_t i = 0; i < plain.lineCount; ++i) {
            if ((plain.lines[i].y == 196 || plain.lines[i].y == 212) && plain.lines[i].tone == PanelTone::Amber) {
                ++warningRows;
            }
        }
        CHECK(warningRows == 2);
        // with it: row 196 is the warning's first words and " ...", row 212 the NOTE
        std::string cut;
        for (std::size_t i = 0; i < v.lineCount; ++i) {
            if (v.lines[i].y == 196) {
                cut = v.lines[i].text;
                CHECK(v.lines[i].tone == PanelTone::Amber);
            }
        }
        REQUIRE(cut.size() > 4);
        CHECK(cut.size() <= shulib::calibration::kSmallCols);
        CHECK(cut.substr(cut.size() - 4) == " ...");
        CHECK(warning.rfind(cut.substr(0, cut.size() - 4), 0) == 0);  // the warning's own first words
        CHECK(hasLine(v, "FIRST RECORD -- the end you pushed first is FRONT", 212));
        int at212 = 0;
        for (std::size_t i = 0; i < v.lineCount; ++i) {
            at212 += v.lines[i].y == 212 ? 1 : 0;
        }
        CHECK(at212 == 1);  // the warning's second row is gone, not drawn under the NOTE
    }
    SUBCASE("no warning, a NOTE; and the SAVED header after USE NEW") {
        CalibRig rig{CalibratorConfig{}, calibPlant(0.0)};
        finishClean(rig);
        REQUIRE(std::string{rig.cal.warning()}.empty());
        FinishFacts f;
        f.note = "FRONT NOT CHECKED -- see the log";
        f.front = FrontChoice::New;
        const PanelView v = viewFor(rig.cal, kArmed, SaveOutcome::Saved, FrontRule::ByPush, f);
        CHECK(std::string{v.headerWord} == "SAVED -- NEW FRONT");
        CHECK(v.headerTone == PanelTone::Amber);
        CHECK(hasLine(v, "FRONT NOT CHECKED -- see the log", 212));
        checkLayout(v);
        const PanelView ns = viewFor(rig.cal, kArmed, SaveOutcome::NotSaved, FrontRule::ByPush, f);
        CHECK(std::string{ns.headerWord} == "NOT SAVED");
        CHECK(ns.headerTone == PanelTone::Red);
        const PanelView plain = viewFor(rig.cal, kArmed, SaveOutcome::Saved, FrontRule::ByPush);
        CHECK(std::string{plain.headerWord} == "SAVED");
        CHECK(plain.headerTone == PanelTone::Green);
        // an empty NOTE is no NOTE
        FinishFacts empty;
        empty.note = "";
        checkSameView(viewFor(rig.cal, kArmed, SaveOutcome::Saved, FrontRule::ByPush, empty), plain);
    }
}
