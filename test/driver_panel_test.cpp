// driver_panel_test.cpp — the driver-control screen's SHARED pure pieces
// (src/common/driver_panel.hpp; chunk COMP-b, 2026-09-30; the COMP brief's B11 as T4 Part 1a
// replaced it): the controller's three lines with their ONE chooser, MOVED out of "shulib Teleop"'s
// panel so the competition program shows the same words, and the controller row a brain page
// shows. Every case names the bug it would catch.
//
// Before the move the chooser lived inside teleop_program.cpp's refreshLcd(), which the host cannot
// compile (pros/screen.h): its words and their order were guarded by one source pin. Here the host
// runs it over every state — and against Teleop's own pre-move expression, restated below as the
// ORACLE, byte for byte, so "what Teleop's controller shows did not change" is a measurement.

#include "doctest.h"

#include <cstdio>
#include <cstring>
#include <string>
#include <string_view>

#include "common/driver_panel.hpp"
#include "programs/teleop_claws.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/hal/motor.hpp"
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
