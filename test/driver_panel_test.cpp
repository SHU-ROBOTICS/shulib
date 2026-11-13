// driver_panel_test.cpp — the driver-control screen's SHARED pure pieces
// (src/common/driver_panel.hpp; chunk COMP-b, 2026-09-30; the COMP brief's B11 as T4 Part 1a
// replaced it): the controller's three lines with their ONE chooser, MOVED out of "shulib Teleop"'s
// panel so the competition program shows the same words, and the controller row a brain page
// shows; since T4 Part 3 (Part 2b's item 6) the PROTECTION row and the chooser's REFUSED / CUT /
// dead words too (D6-D8), for both programs that hand robot two to a driver. The claw words that
// moved into the same header are test/driver_panel_claws_test.cpp's. Every case names the bug it
// would catch.
//
// Before the move the chooser lived inside teleop_program.cpp's refreshLcd(), which the host cannot
// compile (pros/screen.h): its words and their order were guarded by one source pin. Here the host
// runs it over every state — and against Teleop's own pre-move expression, restated below as the
// ORACLE, byte for byte, so "what Teleop's controller shows did not change" is a measurement.

#include "doctest.h"

#include <cstdio>
#include <cstring>
#include <limits>
#include <string>
#include <string_view>

#include "common/driver_panel.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/teleop/drive_protection.hpp"
#include "shulib/ui/touch_panel.hpp"

using shulib::bench::ControllerFacts;
using shulib::bench::ControllerScreen;
using shulib::bench::controllerScreenLines;
using shulib::bench::controllerWord;
using shulib::diag::FaultCode;

namespace {

/// "shulib Teleop"'s refreshLcd() lines as they were written BEFORE the move (teleop_program.cpp
/// at 0f56301, word for word): the oracle the moved chooser must equal wherever Teleop can be.
struct TeleopLines {
    char want[3][24];
};
TeleopLines teleopBeforeTheMove(double battery, double headingDeg, bool hasFault, FaultCode last,
                                bool connected, bool degraded, bool clawAbsent) {
    TeleopLines t{};
    std::snprintf(t.want[0], sizeof t.want[0], "LIB %2.0fV B%4.1fV",
                  shulib::hal::kMaxMotorVoltage.value(), battery);
    std::snprintf(t.want[1], sizeof t.want[1], "hdg %+6.1f deg", headingDeg);
    std::snprintf(t.want[2], sizeof t.want[2], "%.19s",
                  hasFault ? shulib::diag::faultCodeName(last)
                  : !connected ? "no controller 0V"
                  : degraded   ? "DEGRADED see brain"
                  : clawAbsent ? shulib::bench::kLcdClawAbsent.data()
                               : "ok");
    return t;
}

constexpr FaultCode kCodes[] = {
    FaultCode::None,          FaultCode::Precondition,     FaultCode::NanPose,
    FaultCode::LoopOverrun,   FaultCode::OdoStuck,         FaultCode::ImuLost,
    FaultCode::GpsGateReject, FaultCode::Brownout,         FaultCode::MotionTimeout,
    FaultCode::MotorOverTemp, FaultCode::Implausible,      FaultCode::MechanismStalled,
    FaultCode::MotorGroupDisagree,
};

}  // namespace

// (D1) Bug caught: the move changing what Teleop's controller says — a word reworded, the order of
// urgency swapped (a fault hidden behind "no controller 0V"), a format drifted, or a line no longer
// cut at 19 characters — anywhere in Teleop's reachable states (autonEnding is never set by Teleop).
// Every combination of the facts, every fault code, several batteries and headings, against the
// oracle, byte for byte.
TEST_CASE("driver panel (D1): the moved chooser equals Teleop's pre-move lines in every state") {
    int states = 0;
    for (const double battery : {0.0, 7.25, 12.94, 12.95, 99.9}) {
        for (const double heading : {-179.95, -45.0, 0.0, 0.04, 123.45, 1234.5}) {
            for (int bits = 0; bits < 16; ++bits) {
                for (const FaultCode code : kCodes) {
                    ControllerFacts f{};
                    f.batteryVolts = battery;
                    f.headingDegrees = heading;
                    f.hasFault = (bits & 1) != 0;
                    f.lastFault = code;
                    f.connected = (bits & 2) != 0;
                    f.degraded = (bits & 4) != 0;
                    f.clawAbsent = (bits & 8) != 0;
                    const ControllerScreen got = controllerScreenLines(f);
                    const TeleopLines want = teleopBeforeTheMove(
                        battery, heading, f.hasFault, code, f.connected, f.degraded, f.clawAbsent);
                    for (int row = 0; row < 3; ++row) {
                        CAPTURE(row);
                        CHECK(std::string{got.line[row]} == std::string{want.want[row]});
                        CHECK(std::strlen(got.line[row]) <= shulib::bench::kControllerCols);
                    }
                    ++states;
                }
            }
        }
    }
    CHECK(states == 5 * 6 * 16 * 13);
}

// (D2) Bug caught: AUTON ENDING (the competition program's B2 word) placed BELOW a latched fault or
// a lost controller — the driver would read "no controller 0V" and blame the controller while the
// sticks do nothing because the auton task still owns the robot — or leaking into a state where it
// is not set. It outranks every other word, and is chosen only when set.
TEST_CASE("driver panel (D2): AUTON ENDING outranks every other word, and only when set") {
    for (int bits = 0; bits < 16; ++bits) {
        ControllerFacts f{};
        f.hasFault = (bits & 1) != 0;
        f.lastFault = FaultCode::MotionTimeout;
        f.connected = (bits & 2) != 0;
        f.degraded = (bits & 4) != 0;
        f.clawAbsent = (bits & 8) != 0;
        CAPTURE(bits);
        f.autonEnding = true;
        CHECK(std::string_view{controllerWord(f)} == "AUTON ENDING");
        f.autonEnding = false;
        CHECK(std::string_view{controllerWord(f)} != "AUTON ENDING");
    }
    // The words, by name, in their order: the first that holds.
    ControllerFacts f{};
    f.connected = true;
    CHECK(std::string_view{controllerWord(f)} == "ok");
    f.clawAbsent = true;
    CHECK(std::string_view{controllerWord(f)} == "CLAW ABSENT");
    f.degraded = true;
    CHECK(std::string_view{controllerWord(f)} == "DEGRADED see brain");
    f.connected = false;
    CHECK(std::string_view{controllerWord(f)} == "no controller 0V");
    f.hasFault = true;
    f.lastFault = FaultCode::OdoStuck;
    CHECK(std::string_view{controllerWord(f)} == "ODO_STUCK");
    f.autonEnding = true;
    CHECK(std::string_view{controllerWord(f)} == "AUTON ENDING");
    CHECK(std::string_view{controllerScreenLines(f).line[2]} == "AUTON ENDING");
}

// (D3) Bug caught: a controller line past the controller's 19 characters (HA-57: the controller
// shows 3 x 19; a longer write is cut by VEXos where the program did not choose), at the worst
// case of every number and the longest fault name.
TEST_CASE("driver panel (D3): every controller line fits 19 characters at the worst case") {
    ControllerFacts f{};
    f.batteryVolts = 99.94;
    f.headingDegrees = -9999.9;
    f.hasFault = true;
    f.lastFault = FaultCode::MotorGroupDisagree;  // 20 characters: cut to 19
    const ControllerScreen s = controllerScreenLines(f);
    for (int row = 0; row < 3; ++row) {
        CAPTURE(s.line[row]);
        CHECK(std::strlen(s.line[row]) <= shulib::bench::kControllerCols);
    }
    CHECK(std::string{s.line[2]} == "MOTOR_GROUP_DISAGRE");
    CHECK(std::string{s.line[0]} == "LIB 12V B99.9V");
}

// (D4) Bug caught: the brain page's controller row saying CONNECTED when the controller is gone
// (the driver's sticks read 0, the drive gets 0 V — HA-103 — and nothing on the screen says why),
// in the wrong tone, or over the SMALL budget.
TEST_CASE("driver panel (D4): the controller row — connected white, lost red, in budget") {
    const shulib::bench::PanelRow ok = shulib::bench::controllerRowText(true);
    const shulib::bench::PanelRow lost = shulib::bench::controllerRowText(false);
    CHECK(std::string{ok.text} == "controller: CONNECTED");
    CHECK(ok.tone == shulib::ui::Tone::White);
    CHECK(std::string{lost.text} == "NO CONTROLLER -- the drive gets 0 V");
    CHECK(lost.tone == shulib::ui::Tone::Red);
    CHECK(shulib::ui::screenText(ok.text, shulib::ui::kSmallCols));
    CHECK(shulib::ui::screenText(lost.text, shulib::ui::kSmallCols));
}

namespace {
/// Three controller lines, by hand.
ControllerScreen screenOf(const char* l0, const char* l1, const char* l2) {
    ControllerScreen s;
    std::snprintf(s.line[0], sizeof s.line[0], "%s", l0);
    std::snprintf(s.line[1], sizeof s.line[1], "%s", l1);
    std::snprintf(s.line[2], sizeof s.line[2], "%s", l2);
    return s;
}
}  // namespace

// (D5) Bug caught (COMP-b's review: "AUTON ENDING can stay on the controller for the whole driver
// period"): the controller's lines written in a BURST -- PROS's own doc says text updates faster
// than 10 ms wired / 50 ms over VEXnet are not applied, and the adapter drops a failed write
// silently, so the second and third lines of a burst were lost while the program believed them
// written (line 2, written last, was the likeliest; its "ok" never changes, so it stayed stale).
// The writer: ONE line per write, never two writes within kControllerWriteGapSeconds, line 2 first
// whenever it changed (the word: a fault, AUTON ENDING), lines 0 and 1 in turn (neither starves
// while the other keeps changing -- tested in BOTH directions), an outside write (AUTON ENDING)
// remembered and its gap kept, and a forgotten screen written again from line 2.
TEST_CASE("driver panel (D5): the controller writer -- one line per write, line 2 first, gapped") {
    using shulib::bench::ControllerLineWriter;
    using shulib::bench::kControllerWriteGapSeconds;
    REQUIRE(kControllerWriteGapSeconds >= 0.05);  // at least PROS's VEXnet figure
    ControllerLineWriter w;
    const ControllerScreen a = screenOf("LIB 12V B12.9V", "hdg   +0.0 deg", "ok");
    // A fresh controller: all three differ. Line 2 first, then 0, then 1 -- each a gap apart.
    CHECK(w.next(a, 1.00) == 2);
    CHECK(w.next(a, 1.00) == -1);                                     // same instant: nothing
    CHECK(w.next(a, 1.00 + kControllerWriteGapSeconds * 0.99) == -1);  // inside the gap: nothing
    CHECK(w.next(a, 1.10) == 0);
    CHECK(w.next(a, 1.20) == 1);
    CHECK(w.next(a, 1.30) == -1);  // nothing changed: nothing written
    CHECK(std::string{w.shown(2)} == "ok");
    // Line 1 changes at every write, and line 0 once: line 0 still gets its turn.
    ControllerScreen b = screenOf("LIB 12V B12.8V", "hdg   +1.0 deg", "ok");
    CHECK(w.next(b, 1.40) == 0);
    CHECK(w.next(b, 1.50) == 1);
    b = screenOf("LIB 12V B12.8V", "hdg   +2.0 deg", "ok");
    CHECK(w.next(b, 1.60) == 1);
    // A fault: line 2 jumps the queue even while line 1 keeps changing.
    b = screenOf("LIB 12V B12.7V", "hdg   +3.0 deg", "MOTION_TIMEOUT");
    CHECK(w.next(b, 1.70) == 2);
    CHECK(w.next(b, 1.80) == 0);  // then 0 (1 was written last of the two) ...
    CHECK(w.next(b, 1.90) == 1);  // ... then 1
    // The mirror: line 0 changes at every write (the battery, under driving load) and line 1 once:
    // line 1 still gets its turn, at the write after line 0's -- the heading never freezes. (The
    // verification round's V04: a writer that never noted which of the two it wrote last always put
    // line 0 first, and only the direction above was tested.)
    ControllerScreen m = screenOf("LIB 12V B12.6V", "hdg   +4.0 deg", "MOTION_TIMEOUT");
    CHECK(w.next(m, 2.00) == 0);  // both changed; 1 was written last of the two: 0 first ...
    m = screenOf("LIB 12V B12.5V", "hdg   +4.0 deg", "MOTION_TIMEOUT");
    CHECK(w.next(m, 2.10) == 1);  // ... and line 1 next, though line 0 changed again
    CHECK(std::string{w.shown(1)} == "hdg   +4.0 deg");
    m = screenOf("LIB 12V B12.4V", "hdg   +4.0 deg", "MOTION_TIMEOUT");
    CHECK(w.next(m, 2.20) == 0);
    m = screenOf("LIB 12V B12.3V", "hdg   +4.0 deg", "MOTION_TIMEOUT");
    CHECK(w.next(m, 2.30) == 0);  // line 1 is current: line 0 alone, every write
    CHECK(std::string{w.shown(0)} == "LIB 12V B12.3V");
    // AUTON ENDING written outside (the competition program's wait): remembered, and the gap runs
    // from it -- the driver loop's first write cannot follow it within the gap, and is line 2.
    ControllerLineWriter c;
    c.forget();
    c.wrote(2, "AUTON ENDING", 5.00);
    const ControllerScreen driving = screenOf("LIB 12V B12.9V", "hdg   +0.0 deg", "ok");
    CHECK(c.next(driving, 5.01) == -1);
    CHECK(c.next(driving, 5.10) == 2);
    CHECK(std::string{c.shown(2)} == "ok");
    // forget(): a new driver period writes every line again, line 2 first -- and the gap holds.
    c.forget();
    CHECK(c.next(driving, 5.15) == -1);
    CHECK(c.next(driving, 5.20) == 2);
    CHECK(c.next(driving, 5.30) == 0);
    CHECK(c.next(driving, 5.40) == 1);
    CHECK(std::string{c.shown(7)}.empty());  // a row past the three: nothing, never a crash
}

// ═══ T4 Part 2b's words (built in Part 3's late step): the PROTECTION row and the chooser ═══════

namespace {
using shulib::teleop::DriveProtectionState;
using shulib::teleop::DriveVerdict;
using shulib::teleop::ProtectionMode;
DriveProtectionState stateOf(ProtectionMode m) {
    DriveProtectionState s{};
    s.mode = m;
    return s;
}
void put(char (&dst)[shulib::teleop::kProtectionTextBytes], const char* text) {
    std::snprintf(dst, sizeof dst, "%s", text);
}
}  // namespace

// (D6) Bug caught: the PROTECTION row hiding what stopped the drive — a cut shown as OK, a refusal
// without its count or without "for the run", the dead ports dropped from DEGRADED, a frozen port
// never shown (on its way to dead in 250 ms), over-current WARNINGS (cuts switched off, N1) shown as
// a clean OK — or in the wrong colour (a cut amber), or over the SMALL budget, or with a character
// the screen cannot carry. Every mode, by its exact words.
TEST_CASE("driver panel (D6): the PROTECTION row's words and tone in every mode") {
    using shulib::bench::protectionRowText;
    using shulib::ui::Tone;
    shulib::bench::PanelRow r = protectionRowText(stateOf(ProtectionMode::Driving), 1.0);
    CHECK(std::string{r.text} == "drive OK");
    CHECK(r.tone == Tone::Green);
    DriveProtectionState s = stateOf(ProtectionMode::Driving);
    put(s.frozen, "13");
    r = protectionRowText(s, 1.0);
    CHECK(std::string{r.text} == "drive OK -- frozen 13");
    CHECK(r.tone == Tone::Amber);
    s = stateOf(ProtectionMode::Driving);
    s.overCurrentWarnings = 1;
    CHECK(std::string{protectionRowText(s, 1.0).text} == "drive OK -- 1 over-current warning");
    s.overCurrentWarnings = 2;
    r = protectionRowText(s, 1.0);
    CHECK(std::string{r.text} == "drive OK -- 2 over-current warnings");
    CHECK(r.tone == Tone::Amber);
    // Both at once (warnings happen only with the over-current cut switched off): the FROZEN
    // reading is named -- a port about to be dropped is the more urgent of the two, and the MORE
    // page and the serial log carry the warnings (the T4-3 review's RV3: their order was unpinned).
    put(s.frozen, "13");
    r = protectionRowText(s, 1.0);
    CHECK(std::string{r.text} == "drive OK -- frozen 13");
    CHECK(r.tone == Tone::Amber);
    s = stateOf(ProtectionMode::Degraded);
    put(s.dead, "18 13");
    r = protectionRowText(s, 1.0);
    CHECK(std::string{r.text} == "DEGRADED dead 18 13");
    CHECK(r.tone == Tone::Amber);
    put(s.frozen, "14");
    CHECK(std::string{protectionRowText(s, 1.0).text} == "DEGRADED dead 18 13 frozen 14");
    s = stateOf(ProtectionMode::Cut);
    s.cutCount = 2;
    put(s.lastCut, "CUT: FIGHT on LEFT -- port(s) 18");
    r = protectionRowText(s, 1.0);
    CHECK(std::string{r.text} == "CUT 1 s #2: FIGHT on LEFT -- port(s) 18");
    CHECK(r.tone == Tone::Red);
    put(s.lastCut, "port 17 (LEFT) over 2.4 A");  // words without the log's "CUT: " prefix
    CHECK(std::string{protectionRowText(s, 1.0).text} == "CUT 1 s #2: port 17 (LEFT) over 2.4 A");
    // The log's own over-current words are longer than the row: cut at the SMALL budget (the
    // serial log and "shulib Drive"'s MORE page carry the whole), never past it.
    put(s.lastCut, "CUT: port 17 (LEFT) over 2.4 A for 250 ms");
    CHECK(std::string{protectionRowText(s, 1.0).text}
          == std::string{"CUT 1 s #2: port 17 (LEFT) over 2.4 A for 250 ms"}.substr(0, 50));
    CHECK(std::string{protectionRowText(s, std::numeric_limits<double>::quiet_NaN()).text}.rfind(
              "CUT 0 s #2: ", 0)
          == 0);  // a cut length that is not a number never prints "nan"
    // The length is the CONFIG's, handed in, never typed into the row: a loop configured for a
    // half-second or a 2 s cut says so (a mutation printing "1 s" whatever the config passed every
    // check above, which all hand in 1 s -- the T4 late step's LPR2).
    put(s.lastCut, "CUT: FIGHT on LEFT -- port(s) 18");
    CHECK(std::string{protectionRowText(s, 0.5).text} == "CUT 0.5 s #2: FIGHT on LEFT -- port(s) 18");
    CHECK(std::string{protectionRowText(s, 2.0).text} == "CUT 2 s #2: FIGHT on LEFT -- port(s) 18");
    s = stateOf(ProtectionMode::Refused);
    s.verdict.totalDead = 3;
    r = protectionRowText(s, 1.0);
    CHECK(std::string{r.text} == "REFUSED: 3 drive motors dead -- 0 V for the run");
    CHECK(r.tone == Tone::Red);
    // Every row at its worst case: full dead and frozen lists, a cut's words filling the log's
    // buffer, a '%' in them -- within the SMALL budget and screen-safe, always.
    for (const ProtectionMode m : {ProtectionMode::Driving, ProtectionMode::Degraded,
                                   ProtectionMode::Cut, ProtectionMode::Refused}) {
        DriveProtectionState w = stateOf(m);
        for (std::size_t i = 0; i + 1 < sizeof w.dead; ++i) {
            w.dead[i] = '9';
            w.frozen[i] = '8';
            w.lastCut[i] = i == 10 ? '%' : 'c';
        }
        w.cutCount = 2147483647;
        w.overCurrentWarnings = 2147483647;
        w.verdict.totalDead = -2147483647;
        CAPTURE(static_cast<int>(m));
        const shulib::bench::PanelRow worst = protectionRowText(w, 1e300);
        CHECK(std::strlen(worst.text) <= shulib::ui::kSmallCols);
        CHECK(shulib::ui::screenText(worst.text, shulib::ui::kSmallCols));
    }
}

// (D7) Bug caught (the T4 brief's D4; the mutation "CUT ranked below a fault"): a state in force
// hidden behind a latched history -- a cut or a refusal shown as an old fault, or as "no controller"
// / DEGRADED / CLAW ABSENT -- so the driver blames the wrong thing while the sticks do nothing. The
// order of urgency, over EVERY pair of words held at once: AUTON ENDING > REFUSED > CUT > the last
// fault > no controller > DEGRADED > CLAW ABSENT > ok.
TEST_CASE("driver panel (D7): the ONE chooser's full order, every pair -- REFUSED > CUT > fault > ...") {
    struct Word {
        const char* text;
        void (*set)(ControllerFacts&);
    };
    const Word order[] = {
        {"AUTON ENDING", [](ControllerFacts& f) { f.autonEnding = true; }},
        {"REFUSED see brain", [](ControllerFacts& f) { f.refused = true; }},
        {"CUT see brain", [](ControllerFacts& f) { f.cut = true; }},
        {"MOTION_TIMEOUT",
         [](ControllerFacts& f) {
             f.hasFault = true;
             f.lastFault = FaultCode::MotionTimeout;
         }},
        {"no controller 0V", [](ControllerFacts& f) { f.connected = false; }},
        {"DEGRADED see brain", [](ControllerFacts& f) { f.degraded = true; }},
        {"CLAW ABSENT", [](ControllerFacts& f) { f.clawAbsent = true; }},
    };
    constexpr std::size_t n = sizeof order / sizeof order[0];
    int pairs = 0;
    for (std::size_t hi = 0; hi < n; ++hi) {
        ControllerFacts alone{};
        alone.connected = true;
        order[hi].set(alone);
        CAPTURE(order[hi].text);
        CHECK(std::string_view{controllerWord(alone)} == order[hi].text);
        for (std::size_t lo = hi + 1; lo < n; ++lo) {
            ControllerFacts both{};
            both.connected = true;
            order[lo].set(both);
            order[hi].set(both);
            CAPTURE(order[lo].text);
            CHECK(std::string_view{controllerWord(both)} == order[hi].text);
            ++pairs;
        }
    }
    CHECK(pairs == 21);
    ControllerFacts ok{};
    ok.connected = true;
    CHECK(std::string_view{controllerWord(ok)} == "ok");
    for (const char* w : {"REFUSED see brain", "CUT see brain"}) {
        CHECK(std::strlen(w) <= shulib::bench::kControllerCols);
    }
}

// (D8) Bug caught: the protections' state not reaching the controller -- a cut or a refusal never
// said, a runtime death never DEGRADED, the dead ports never on line 1 -- or the hand-over CLEARING
// a word the program set (the boot's DEGRADED, a refusal) because the protections' state says
// otherwise this tick; line 1 past 19 characters with a long dead list.
TEST_CASE("driver panel (D8): addProtectionFacts -- REFUSED, CUT, DEGRADED and line 1's dead ports; "
          "it only ever SETS") {
    using shulib::bench::addProtectionFacts;
    ControllerFacts f{};
    f.connected = true;
    f.headingDegrees = 12.0;
    DriveProtectionState s = stateOf(ProtectionMode::Driving);
    addProtectionFacts(f, s);
    CHECK_FALSE(f.refused);
    CHECK_FALSE(f.cut);
    CHECK_FALSE(f.degraded);
    CHECK(std::string{controllerScreenLines(f).line[1]} == "hdg  +12.0 deg");
    CHECK(std::string{controllerScreenLines(f).line[2]} == "ok");
    // A runtime death: DEGRADED by the mid-run verdict, and line 1 names the dead.
    s = stateOf(ProtectionMode::Degraded);
    s.verdict.verdict = DriveVerdict::DriveDegraded;
    put(s.dead, "18 13");
    addProtectionFacts(f, s);
    CHECK(f.degraded);
    ControllerScreen lines = controllerScreenLines(f);
    CHECK(std::string{lines.line[1]} == "dead 18 13");
    CHECK(std::string{lines.line[2]} == "DEGRADED see brain");
    // A cut over it, then a refusal over that.
    s.mode = ProtectionMode::Cut;
    addProtectionFacts(f, s);
    CHECK(std::string{controllerScreenLines(f).line[2]} == "CUT see brain");
    s.mode = ProtectionMode::Refused;
    addProtectionFacts(f, s);
    CHECK(std::string{controllerScreenLines(f).line[2]} == "REFUSED see brain");
    // It only SETS: a later Driving state does not clear what the program already set.
    ControllerFacts boot{};
    boot.connected = true;
    boot.degraded = true;  // the boot's port check
    addProtectionFacts(boot, stateOf(ProtectionMode::Driving));
    CHECK(boot.degraded);
    CHECK(std::string{controllerScreenLines(boot).line[2]} == "DEGRADED see brain");
    // ... and the same for a REFUSED or a CUT the program set itself (the T4-3 review's RV14: only
    // DEGRADED was pinned, though this case's comment names a refusal too).
    ControllerFacts set{};
    set.connected = true;
    set.refused = true;
    set.cut = true;
    addProtectionFacts(set, stateOf(ProtectionMode::Driving));
    CHECK(set.refused);
    CHECK(set.cut);
    CHECK(std::string{controllerScreenLines(set).line[2]} == "REFUSED see brain");
    // A long dead list: line 1 cut at 19 characters.
    DriveProtectionState many = stateOf(ProtectionMode::Degraded);
    put(many.dead, "11 12 13 14 15 16 17 18 19 20");
    ControllerFacts g{};
    addProtectionFacts(g, many);
    CHECK(std::strlen(controllerScreenLines(g).line[1]) <= shulib::bench::kControllerCols);
    CHECK(std::string{controllerScreenLines(g).line[1]} == "dead 11 12 13 14 15");
}
