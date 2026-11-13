// Adversarial tests for "shulib Tune"'s screen model — include/shulib/mechanism/tune_panel.hpp (chunk
// TUNE1-B1, 2026-11-10; the MECH-S/TUNE1 brief's TB1). Every page, at every mechanism kind, the longest
// name a mechanism can have (23 characters) and the widest numbers, is held to the panel's rules:
// every rectangle on the screen (480 x 236), no two padded hit boxes overlapping, no text line under a
// button, every text within its size's budget, printable ASCII and no '%'. The budgets are ESTIMATES
// (A4 register HA-136) — the rules are what is tested, not the pixels. NOTHING HERE HAS RUN ON A ROBOT.

#include "doctest.h"

#include <cmath>
#include <cstring>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

#include "shulib/mechanism/tune_panel.hpp"
#include "shulib/mechanism/tune_program.hpp"

namespace {

namespace mech = shulib::mechanism;
namespace ui = shulib::ui;
using mech::TuneButton;
using mech::TunePage;

constexpr const char* kLongName = "a twenty-three char lft";  // 23 characters: the longest a name can be
static_assert(std::char_traits<char>::length("a twenty-three char lft") == 23);

/// The pixels a line of `l` covers (its budget's width at the size's estimated character width).
[[nodiscard]] ui::Rect lineRect(const mech::TuneLine& l) {
    const int charW = 472 / static_cast<int>(ui::budgetFor(l.size));
    const int w = static_cast<int>(std::strlen(l.text)) * charW;
    return ui::Rect{l.x, l.y, static_cast<std::int16_t>(l.x + (w > 0 ? w - 1 : 0)),
                    static_cast<std::int16_t>(l.y + ui::heightFor(l.size) - 1)};
}

/// Every rule a page keeps (header), with the page's name in every failure.
void checkView(const mech::TuneView& v, const std::string& what) {
    CAPTURE(what);
    REQUIRE(v.lineCount <= mech::kTuneMaxLines);
    REQUIRE(v.buttonCount <= mech::kTuneMaxButtons);
    for (std::size_t i = 0; i < v.lineCount; ++i) {
        const mech::TuneLine& l = v.lines[i];
        CAPTURE(l.text);
        CHECK(ui::screenText(l.text, ui::budgetFor(l.size)));
        CHECK(ui::onScreen(lineRect(l)));
        for (std::size_t b = 0; b < v.buttonCount; ++b) {
            CAPTURE(v.buttons[b].label);
            CHECK_FALSE(ui::overlaps(lineRect(l), v.buttons[b].rect));
        }
    }
    for (std::size_t a = 0; a < v.buttonCount; ++a) {
        const mech::TuneButtonView& x = v.buttons[a];
        CAPTURE(x.label);
        CHECK(ui::onScreen(x.rect));
        CHECK(ui::screenText(x.label, mech::tuneLabelCols(x.rect)));
        CHECK(std::strlen(x.label) > 0);
        for (std::size_t b = a + 1; b < v.buttonCount; ++b) {
            CAPTURE(v.buttons[b].label);
            CHECK_FALSE(ui::overlaps(ui::hitBox(x.rect), ui::hitBox(v.buttons[b].rect)));
            CHECK(x.id != v.buttons[b].id);
        }
    }
}

/// The widest facts a page can be handed.
[[nodiscard]] mech::TuneFacts widest(TunePage page, mech::Kind kind, mech::Units units, double big) {
    mech::TuneFacts f{};
    f.page = page;
    f.armed = true;
    f.mechCount = 8;
    f.mech = 7;
    f.m.name = kLongName;
    f.m.kind = kind;
    f.m.units = units;
    f.m.built = true;
    f.m.homed = big > 0.0;
    f.m.homes = true;
    f.m.now = big;
    f.m.target = -big;
    f.m.active = true;
    f.m.lastVerdict = "UNCONFIRMED";
    f.m.changed = true;
    f.m.capChanged = true;
    f.sel = mech::TuneParamFacts{"homing lasting", big, -big, "V/(deg/s^2)", mech::StepRule::ZeroGain};
    f.next.name = "homing lasting";
    f.param = 23;
    f.paramCount = 24;
    f.fine = true;
    f.undoDepth = 16;
    f.controller = true;
    f.status = "'a twenty-three char lft': a refusal sentence long enough to need both of the status lines "
               "and then some more words past them that are cut, with 100% of a percent sign in it";
    f.testA = big;
    f.testB = -big;
    f.confirmWhat = "STRING CHECK";
    f.report.test = mech::TuneTest::B;
    f.report.kind = kind;
    f.report.unit = "deg";
    f.report.target = -big;
    f.report.verdict = "UNCONFIRMED";
    f.report.seconds = 10.0;
    f.report.bound = 10.0;
    f.report.overshoot = big;
    f.report.settle = 9.99;
    f.report.peakAmps = 2.5;
    f.report.finalError = -big;
    f.report.hasFinalError = true;
    f.report.holdAmps = 2.5;
    f.report.stallAmps = 2.5;
    f.report.capAmps = 2.5;
    f.report.warnStallNotAboveHold = true;
    f.report.warnCapNotAbovePeak = true;
    f.report.stoppedBy = "the controller disconnected";
    f.limitsWritten = 8;
    f.copiedAtExit = 8;
    f.resetAtExit = 8;
    f.anyCapChanged = true;
    f.stoppedWhy = "a precondition failed: LiveTuning::apply: attach() the service first -- with words past "
                   "two lines of the screen, which are cut there";
    return f;
}

const TunePage kEveryPage[] = {TunePage::Clear,       TunePage::Tune,   TunePage::Confirm, TunePage::Report,
                               TunePage::StringCheck, TunePage::Exited, TunePage::Stopped};
const mech::Kind kEveryKind[] = {mech::Kind::Position, mech::Kind::Speed, mech::Kind::Power, mech::Kind::Pneumatic};

}  // namespace

TEST_CASE("Tune panel TB1: EVERY page, at every kind, the longest name and the widest numbers, keeps the "
          "rules -- on the screen, no hit boxes overlapping, no line under a button, every text in its "
          "budget, ASCII with no '%'") {
    // BUG CAUGHT (MB2): a page's button off the panel, or two buttons whose padded hit boxes overlap (a
    // tap meant for [+] lands on TEST); a line cut wrong, or one drawn under a button.
    const double numbers[] = {0.0, 1.0, -1.0, 123456789.0, -123456789.0, 1e-9, 3.14159265358979,
                              std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()};
    for (TunePage page : kEveryPage) {
        for (mech::Kind kind : kEveryKind) {
            for (mech::Units units : {mech::Units::Degrees, mech::Units::Inches}) {
                for (double big : numbers) {
                    for (bool busy : {false, true}) {
                        for (bool built : {true, false}) {
                            mech::TuneFacts f = widest(page, kind, units, big);
                            f.busy = busy;
                            f.m.built = built;
                            checkView(mech::tuneView(f), std::string{"page "} + std::to_string(static_cast<int>(page)) +
                                                             " kind " + std::to_string(static_cast<int>(kind)) +
                                                             " busy " + std::to_string(busy));
                            f.field = true;
                            f.controller = false;
                            f.sel.name = nullptr;
                            f.paramCount = 0;
                            f.report.overshoot = -1.0;
                            f.report.settle = -1.0;
                            f.report.holdAmps = -1.0;
                            f.report.hasFinalError = false;
                            f.status = "";
                            checkView(mech::tuneView(f), "the sparse facts");
                            // the other words a page can say: a hold that ENDED / was STOPPED, READY held
                            // back by a button, a cap the file does not state, nothing copied at EXIT
                            f.controller = true;
                            f.field = false;
                            f.buttonHeld = true;
                            f.report.holdEnded = true;
                            f.copiedAtExit = 0;
                            f.sel = mech::TuneParamFacts{"current cap", 0.0, 0.0, "A", mech::StepRule::Amps, true};
                            f.paramCount = 4;
                            checkView(mech::tuneView(f), "the hold ENDED, a button held");
                            f.report.holdEnded = false;
                            f.report.holdStopped = true;
                            f.buttonHeld = false;
                            checkView(mech::tuneView(f), "the hold STOPPED");
                        }
                    }
                }
            }
        }
    }
}

TEST_CASE("Tune panel TB1b: a TEST button's label carries its target and fits its button at every value; a "
          "power run's says FWD / REV") {
    const mech::TuneView v = mech::tuneView(widest(TunePage::Tune, mech::Kind::Position, mech::Units::Inches, 1.0));
    const mech::TuneButtonView* a = v.button(TuneButton::TestA);
    REQUIRE(a != nullptr);
    for (double t : {0.0, 22.0, -1.5, 99.99, 100.0, -9999.0, 123456.0, -1e9, 0.001, 3000.0}) {
        for (mech::Kind k : kEveryKind) {
            char label[16];
            mech::tuneTestLabel(label, sizeof label, 'A', k, t);
            CAPTURE(label);
            CHECK(ui::screenText(label, mech::tuneLabelCols(a->rect)));
        }
    }
    char label[16];
    mech::tuneTestLabel(label, sizeof label, 'A', mech::Kind::Position, 22.0);
    CHECK(std::string_view{label} == "A 22");
    mech::tuneTestLabel(label, sizeof label, 'B', mech::Kind::Power, -12.0);
    CHECK(std::string_view{label} == "B REV");
    mech::tuneTestLabel(label, sizeof label, 'A', mech::Kind::Speed, 3000.0);
    CHECK(std::string_view{label} == "A 3000");
}

TEST_CASE("Tune panel: a layout still ARMING takes no touch; busy greys every button but STOP; what is "
          "armed follows the facts") {
    mech::TuneFacts f = widest(TunePage::Tune, mech::Kind::Position, mech::Units::Inches, 1.0);
    f.armed = false;
    mech::TuneView v = mech::tuneView(f);
    for (std::size_t i = 0; i < v.buttonCount; ++i) {
        CAPTURE(v.buttons[i].label);
        const bool stop = v.buttons[i].id == TuneButton::Stop;
        CHECK(v.buttons[i].armed == stop);  // STOP is never greyed (header)
        mech::TuneButton hit{};
        const ui::Rect r = v.buttons[i].rect;
        CHECK(mech::tuneHit(v, (r.x0 + r.x1) / 2, (r.y0 + r.y1) / 2, hit) == stop);
    }
    // every other page: nothing armed while arming — but the REPORT page's STOP (the TUNE page's cell)
    for (TunePage page : kEveryPage) {
        f.page = page;
        if (page == TunePage::Tune) {
            continue;
        }
        v = mech::tuneView(f);
        for (std::size_t i = 0; i < v.buttonCount; ++i) {
            CAPTURE(static_cast<int>(page));
            CAPTURE(v.buttons[i].label);
            CHECK(v.buttons[i].armed == (page == TunePage::Report && v.buttons[i].id == TuneButton::Stop));
        }
    }
    f.page = TunePage::Tune;
    f.armed = true;
    f.busy = true;
    v = mech::tuneView(f);
    for (std::size_t i = 0; i < v.buttonCount; ++i) {
        CAPTURE(v.buttons[i].label);
        CHECK(v.buttons[i].armed == (v.buttons[i].id == TuneButton::Stop));
    }
    f.busy = false;
    f.undoDepth = 0;
    f.m.changed = false;
    f.mechCount = 1;
    f.m.kind = mech::Kind::Power;
    v = mech::tuneView(f);
    CHECK_FALSE(v.button(TuneButton::Undo)->armed);
    CHECK_FALSE(v.button(TuneButton::Reset)->armed);
    CHECK_FALSE(v.button(TuneButton::PrevMech)->armed);
    CHECK_FALSE(v.button(TuneButton::Home)->armed);         // a power mechanism has no homing step
    CHECK_FALSE(v.button(TuneButton::StringCheck)->armed);  // a lift's only
    CHECK(v.button(TuneButton::TestA)->armed);
    CHECK(std::string_view{v.button(TuneButton::Undo)->label} == "UNDO");
    // the hit test answers the armed button under the tap, padded
    const ui::Rect r = v.button(TuneButton::TestA)->rect;
    mech::TuneButton hit{};
    REQUIRE(mech::tuneHit(v, r.x0 - ui::kHitPadX, r.y0 - ui::kHitPadY, hit));
    CHECK(hit == TuneButton::TestA);
}

TEST_CASE("Tune panel: the layout key changes with the page and never with text or arming (the arming clock "
          "restarts only on a new layout)") {
    mech::TuneFacts f = widest(TunePage::Tune, mech::Kind::Position, mech::Units::Inches, 1.0);
    const std::uint32_t k = mech::tuneView(f).layoutKey();
    f.armed = false;
    f.status = "something else";
    f.m.now = 5.0;
    f.busy = true;
    CHECK(mech::tuneView(f).layoutKey() == k);
    f.page = TunePage::Report;
    CHECK(mech::tuneView(f).layoutKey() != k);
}

TEST_CASE("Tune panel: the status sentence wraps at a space onto two lines, cut there; a '%' never reaches "
          "the screen") {
    char a[ui::kSmallCols + 1];
    char b[ui::kSmallCols + 1];
    mech::detail::tuneWrap("short", a, b, ui::kSmallCols);
    CHECK(std::string_view{a} == "short");
    CHECK(std::string_view{b}.empty());
    const std::string longer(49, 'x');
    const std::string s = longer + " word and 100% more";
    mech::detail::tuneWrap(s.c_str(), a, b, ui::kSmallCols);
    CHECK(std::string_view{a} == longer);
    CHECK(std::string_view{b} == "word and 100? more");
    const std::string nospace(130, 'y');
    mech::detail::tuneWrap(nospace.c_str(), a, b, ui::kSmallCols);
    CHECK(std::strlen(a) == ui::kSmallCols);
    CHECK(std::strlen(b) == ui::kSmallCols);
}

namespace {
/// Every line a view draws, in order, one per line (for a failure's INFO).
std::string linesOf(const mech::TuneView& v) {
    std::string out;
    for (std::size_t i = 0; i < v.lineCount; ++i) {
        out += v.lines[i].text;
        out += '\n';
    }
    return out;
}
bool hasLine(const mech::TuneView& v, std::string_view want) {
    for (std::size_t i = 0; i < v.lineCount; ++i) {
        if (std::string_view{v.lines[i].text} == want) {
            return true;
        }
    }
    return false;
}
bool anyLineContains(const mech::TuneView& v, std::string_view part) {
    for (std::size_t i = 0; i < v.lineCount; ++i) {
        if (std::string_view{v.lines[i].text}.find(part) != std::string_view::npos) {
            return true;
        }
    }
    return false;
}
/// A plain report page: a lift's TEST B SUCCEEDED, peak 1.20 A, its hold 0.40 A under a 0.60 A signature.
mech::TuneFacts reportFacts() {
    mech::TuneFacts f{};
    f.page = TunePage::Report;
    f.armed = true;
    f.mechCount = 1;
    f.m.name = "lift";
    f.m.kind = mech::Kind::Position;
    f.m.units = mech::Units::Inches;
    f.report.test = mech::TuneTest::B;
    f.report.kind = mech::Kind::Position;
    f.report.unit = "in";
    f.report.target = 12.0;
    f.report.verdict = "SUCCEEDED";
    f.report.seconds = 2.35;
    f.report.bound = 5.06;
    f.report.overshoot = 0.02;
    f.report.settle = 0.12;
    f.report.peakAmps = 1.2;
    f.report.finalError = 0.004;
    f.report.hasFinalError = true;
    f.report.holdAmps = 0.4;
    f.report.stallAmps = 0.6;
    return f;
}
}  // namespace

TEST_CASE("Tune panel: the REPORT never shows the hold's n/a sentinel as a number -- a hold that ENDED, one "
          "STOPPED, a move that failed and a POWER run each say what they are, and keep the page's rules") {
    // BUG CAUGHT: a hold cut by its own stall signature (re-check #3's V1 case) drawn as "no hold: it did
    // not settle" under SUCCEEDED, and "WARNING: stall 0.60 A not above the hold -1.00 A"; a power run's
    // SUCCEEDED drawn with "never settled".
    mech::TuneFacts f = reportFacts();
    mech::TuneView v = mech::tuneView(f);
    checkView(v, "a plain report");
    CHECK(hasLine(v, "peak 1.20 A, hold 0.40 A (mean, 1 s after)"));
    CHECK(hasLine(v, "settled in 0.12 s after first reaching it"));
    // the hold ENDED (its stall signature tripped on it): the warning says so, never the -1
    f.report.holdAmps = -1.0;
    f.report.holdEnded = true;
    f.report.warnStallNotAboveHold = true;
    v = mech::tuneView(f);
    checkView(v, "a hold that ENDED");
    INFO(linesOf(v));
    CHECK(hasLine(v, "peak 1.20 A; hold ENDED (the stall tripped on it)"));
    CHECK(hasLine(v, "WARNING: stall 0.60 A trips on the hold: raise it"));
    CHECK_FALSE(anyLineContains(v, "-1.00"));
    CHECK_FALSE(anyLineContains(v, "did not settle"));
    CHECK_FALSE(anyLineContains(v, "not above the hold"));
    // a STOP in the hold second: n/a, said so, and no warning of its own
    f.report.holdEnded = false;
    f.report.holdStopped = true;
    f.report.warnStallNotAboveHold = false;
    f.report.stoppedBy = "STOP on the screen";
    v = mech::tuneView(f);
    checkView(v, "a hold STOPPED");
    CHECK(hasLine(v, "peak 1.20 A; hold n/a: stopped in its second"));
    CHECK(hasLine(v, "stopped by STOP on the screen"));
    CHECK_FALSE(anyLineContains(v, "WARNING"));
    CHECK_FALSE(anyLineContains(v, "-1.00"));
    // a move that did not succeed: no hold, and why
    f.report.holdStopped = false;
    f.report.stoppedBy = "";
    f.report.verdict = "TIMED_OUT";
    f.report.settle = -1.0;
    v = mech::tuneView(f);
    CHECK(hasLine(v, "peak 1.20 A (no hold: the move did not succeed)"));
    CHECK(hasLine(v, "never settled inside \"reached\""));
    // the mean's warning, when there IS a hold: its number
    f = reportFacts();
    f.report.holdAmps = 0.85;
    f.report.warnStallNotAboveHold = true;
    CHECK(hasLine(mech::tuneView(f), "WARNING: stall 0.60 A not above the hold 0.85 A"));
    // a POWER run: no settle, no hold — n/a, unsaid; never "never settled" under SUCCEEDED
    f = reportFacts();
    f.report.kind = mech::Kind::Power;
    f.report.unit = "V";
    f.report.target = 12.0;
    f.report.overshoot = -1.0;
    f.report.settle = -1.0;
    f.report.hasFinalError = false;
    f.report.holdAmps = -1.0;
    v = mech::tuneView(f);
    checkView(v, "a power run");
    INFO(linesOf(v));
    CHECK(hasLine(v, "to 12.00 V: SUCCEEDED in 2.35 s (limit 5.06 s)"));
    CHECK(hasLine(v, "peak 1.20 A"));
    CHECK_FALSE(anyLineContains(v, "settle"));
    CHECK_FALSE(anyLineContains(v, "hold"));
}

TEST_CASE("Tune panel: the REPORT page's STOP is the TUNE page's STOP -- the same cell, armed from the first "
          "frame -- and nothing that starts a move sits where STOP was") {
    // BUG CAUGHT: the page flipping from TUNE to REPORT as a test ends put AGAIN where STOP had been: a
    // reflexive tap meant for STOP (watching an overshoot) started the move again with no GO.
    mech::TuneFacts f = widest(TunePage::Tune, mech::Kind::Position, mech::Units::Inches, 1.0);
    const mech::TuneView tune = mech::tuneView(f);
    f.page = TunePage::Report;
    f.armed = false;  // the first frame after the flip: the layout still arming
    const mech::TuneView report = mech::tuneView(f);
    const mech::TuneButtonView* stopT = tune.button(TuneButton::Stop);
    const mech::TuneButtonView* stopR = report.button(TuneButton::Stop);
    REQUIRE(stopT != nullptr);
    REQUIRE(stopR != nullptr);
    CHECK(stopR->rect.x0 == stopT->rect.x0);
    CHECK(stopR->rect.x1 == stopT->rect.x1);
    CHECK(stopR->rect.y0 == stopT->rect.y0);
    CHECK(stopR->rect.y1 == stopT->rect.y1);
    CHECK(stopR->armed);
    mech::TuneButton hit{};
    // every point of the TUNE page's STOP hit box lands on the REPORT page's STOP or on nothing
    const ui::Rect box = ui::hitBox(stopT->rect);
    for (int x = box.x0; x <= box.x1; x += 3) {
        for (int y = box.y0; y <= box.y1; y += 3) {
            mech::TuneFacts armed = f;
            armed.armed = true;
            if (mech::tuneHit(mech::tuneView(armed), x, y, hit)) {
                CAPTURE(x);
                CAPTURE(y);
                CHECK(hit == TuneButton::Stop);
            }
        }
    }
}

TEST_CASE("Tune panel: CLEAR greys READY with why in red -- no controller, a field or switch, a button held -- "
          "and the TUNE page says how a cap starts from none and that ANY mechanism's cap is changed") {
    mech::TuneFacts f{};
    f.page = TunePage::Clear;
    f.armed = true;
    f.controller = true;
    mech::TuneView v = mech::tuneView(f);
    checkView(v, "clear, all well");
    CHECK(v.button(TuneButton::Ready)->armed);
    struct Case {
        bool controller, field, held;
        const char* why;
    };
    const Case cases[] = {{false, false, false, "No controller: connect it first (it is the STOP)."},
                          {true, true, false, "Field or switch plugged in: unplug it first."},
                          {true, false, true, "A controller button is held: let go first."}};
    for (const Case& c : cases) {
        CAPTURE(c.why);
        f.controller = c.controller;
        f.field = c.field;
        f.buttonHeld = c.held;
        v = mech::tuneView(f);
        checkView(v, "clear, blocked");
        CHECK_FALSE(v.button(TuneButton::Ready)->armed);
        CHECK(hasLine(v, c.why));
    }
    // a refused READY's sentence shows on CLEAR's status lines
    f = mech::TuneFacts{};
    f.page = TunePage::Clear;
    f.armed = true;
    f.controller = true;
    f.status = "READY refused -- a controller button is held: let go first; nothing was made";
    v = mech::tuneView(f);
    checkView(v, "clear, a refusal");
    CHECK(hasLine(v, "READY refused -- a controller button is held: let"));
    CHECK(hasLine(v, "go first; nothing was made"));
    // TUNE: a cap the file does not state — how one starts
    f = widest(TunePage::Tune, mech::Kind::Position, mech::Units::Inches, 1.0);
    f.sel = mech::TuneParamFacts{"current cap", 0.0, 0.0, "A", mech::StepRule::Amps, true};
    v = mech::tuneView(f);
    CHECK(hasLine(v, "  file: none; - starts a cap at 2.45 A"));
    // the safety line: ANOTHER mechanism's cap changed is enough
    f.m.capChanged = false;
    f.anyCapChanged = true;
    CHECK(hasLine(mech::tuneView(f), "cap changed: EXIT or RESET before you stop"));
    f.anyCapChanged = false;
    CHECK(hasLine(mech::tuneView(f), "any controller button STOPS every mechanism"));
}

TEST_CASE("Tune panel: EXITED says what EXIT COPIED and RESET on the way -- words for one and for many") {
    mech::TuneFacts f{};
    f.page = TunePage::Exited;
    f.armed = true;
    f.limitsWritten = 1;
    f.copiedAtExit = 1;
    f.resetAtExit = 1;
    mech::TuneView v = mech::tuneView(f);
    checkView(v, "exited, one of each");
    INFO(linesOf(v));
    CHECK(hasLine(v, "1 motor limit written back to the robot file's."));
    CHECK(hasLine(v, "EXIT COPIED 1 mechanism's changed numbers to"));
    CHECK(hasLine(v, "the terminal and /usd/tune_log.txt: put them"));
    CHECK(hasLine(v, "1 RESET to the file: a signature >= its cap."));
    f.copiedAtExit = 2;
    f.resetAtExit = 0;
    v = mech::tuneView(f);
    CHECK(hasLine(v, "EXIT COPIED 2 mechanisms' changed numbers to"));
    CHECK_FALSE(anyLineContains(v, "RESET"));
    f.copiedAtExit = 0;
    v = mech::tuneView(f);
    CHECK(hasLine(v, "Every change was COPIED already (or none made)."));
}
