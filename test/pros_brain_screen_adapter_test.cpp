// hal::pros::ProsBrainScreen through the host PROS shim (chunk CAL1 Part A1, the brief's §2.A test
// 8a), and the shim pieces Part A1 adds for CAL1 (test/pros_shim/pros/screen.h, misc.h, motors.h,
// rtos.hpp's advance hook, and test/pros_shim/main.h). NOTHING HERE HAS RUN ON A ROBOT.
//
// HONEST LIMIT (the shim's own, shim_control.hpp): these cases test the adapter against our
// BELIEF about PROS — the calls today's "shulib Calibrate" makes, transcribed from the vendored
// headers — not the belief itself. Hardware tests that.
//
// Each case names the bug it catches:
//   A1  each draw is today's PROS sequence: the pen or the eraser set, then the draw — and print()
//       sets the eraser, then the pen, then prints through a LITERAL "%s" (no text a caller passes
//       ever reaches VEXos as a format)
//   A2  print() sanitizes, cuts at `cols`, pads, and maps the three sizes to PROS's three PLAIN
//       formats (the shim records every format apart, the centred ones included)
//   A3  the touch: a good read passes the RELEASE count and the place (a press alone moves no
//       release); an E_TOUCH_ERROR read is an error, never a tap
//   A4  prints() counts every print and nothing else
//   A5  the painter through this adapter draws EXACTLY what it draws on hal::fake::FakeBrainScreen
//       — the fake is a faithful model of the adapter, so the host tests on the fake hold for it
//   A5b the fake's TOUCH reads equal this adapter's, read for read: a press, a release, a tap,
//       error reads (every field -1) and the good read after them
//   A6  the shim pieces: the competition reads (the vendored bits); the three C motor reads (the
//       device state, each unit, counts by the reported gearing, a negative port, a port that does
//       not answer — and NOT ONE write); the advance hook (called AFTER the clock moved, by every
//       delay() and every delay_until() that blocks — the library's ProsTickPacer included — never
//       by one whose wake instant is now or past, uninstalled by resetTime(), and with none
//       installed the clock behaves as before); the host main.h's five entry points, each with C
//       linkage (checked at compile time), and its shim guard

#include "doctest.h"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

#include "main.h"  // the host shim's: the five PROS entry points (A6)
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "pros/screen.h"
#include "pros/shim_control.hpp"
#include "shulib/calibration/calibrate_panel.hpp"
#include "shulib/calibration/calibrate_screen.hpp"
#include "shulib/hal/brain_screen.hpp"
#include "shulib/hal/fake/fake_brain_screen.hpp"
#include "shulib/hal/pros/brain_screen.hpp"
#include "shulib/hal/pros/tick_pacer.hpp"  // A6c: the pacer a program's 10 ms loop waits with

namespace {

using shulib::calibration::CalibratePainter;
using shulib::calibration::PanelButton;
using shulib::calibration::PanelButtonId;
using shulib::calibration::PanelLine;
using shulib::calibration::PanelRect;
using shulib::calibration::PanelScreen;
using shulib::calibration::PanelTone;
using shulib::calibration::PanelView;
using shulib::calibration::TextSize;
using shulib::hal::ScreenTextSize;
using shulib::hal::fake::FakeBrainScreen;
using shulib::hal::pros::ProsBrainScreen;

const std::vector<std::string>& calls() { return pros::shim::screenState().calls; }
const std::vector<std::string>& draws() { return pros::shim::screenState().log; }

PanelView pushView(const char* number, PanelTone tone, bool doneArmed, bool withDone = true) {
    PanelView v;
    v.screen = PanelScreen::Push;
    v.headerTone = PanelTone::Blue;
    v.headerWord = "2/5 PUSH";
    PanelLine a;
    a.x = 8;
    a.y = 30;
    a.size = TextSize::Large;
    std::snprintf(a.text, sizeof a.text, "%s", "PUSH IT");
    PanelLine b;
    b.x = 8;
    b.y = 102;
    b.size = TextSize::Large;
    b.tone = tone;
    std::snprintf(b.text, sizeof b.text, "%s", number);
    v.lines[0] = a;
    v.lines[1] = b;
    v.lineCount = 2;
    std::uint8_t n = 0;
    if (withDone) {
        v.buttons[n++] = PanelButton{PanelButtonId::Done, PanelRect{6, 180, 300, 234}, "DONE",
                                     doneArmed};
    }
    v.buttons[n++] = PanelButton{PanelButtonId::Abort, PanelRect{330, 180, 474, 234}, "ABORT", true};
    v.buttonCount = n;
    return v;
}

/// The same painting, on any screen: every kind of render and every own screen.
void paintTheSequence(CalibratePainter& p) {
    p.paintStarting();
    p.render(pushView("0.0 in", PanelTone::Amber, false), 0.0);
    p.render(pushView("0.4 in", PanelTone::Amber, false), 0.1);   // throttled
    p.render(pushView("1.2 in", PanelTone::Amber, true), 0.5);    // armed + live number
    p.render(pushView("12.0 in", PanelTone::Green, true), 0.51);  // colour change
    p.render(pushView("12.0 in", PanelTone::Green, true, false), 0.6);  // DONE vanishes
    PanelView other = pushView("50% \xE2\x80\x94 x", PanelTone::Red, true);
    other.screen = PanelScreen::Stopped;
    other.headerTone = PanelTone::Red;
    p.render(other, 1.0);  // full repaint, a '%' and a dash in a line
    p.paintCannot("CANNOT START", PanelTone::Red, "TOO FEW MOTORS",
                  "the census found 1 motor; a tank needs at least two drive motors",
                  "re-seat the drive cables and run again",
                  "nothing is powered -- fix it, then restart it");
    p.paintNotDriverControl("status bits 0x01 field=no disabled=yes auton=no");
    p.paintStateScreen("DISABLED", "the robot is disabled", "status bits 0x01");
    p.render(pushView("1.0 in", PanelTone::Amber, true), 2.0);
}

std::string readSource(const char* rel) {
    std::ifstream in(std::filesystem::path(SHULIB_SOURCE_DIR) / rel);
    REQUIRE_MESSAGE(in.good(), "cannot open ", rel);
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

}  // namespace

TEST_CASE("ProsBrainScreen A1: each draw is today's PROS sequence, and print() is eraser, pen, "
          "then print_at through a literal \"%s\"") {
    // BUG CAUGHT: a draw before its colour is set (the previous colour would be used); print_at
    // handed the caller's text as its FORMAT (a '%' in a refusal reason would be read as a
    // conversion by VEXos); the pen set after the print.
    pros::shim::resetScreen();
    ProsBrainScreen s;
    s.fillRect(6, 180, 300, 234, 0x39C36E);
    s.outlineRect(6, 180, 300, 234, 0xC8D8F0);
    s.eraseRect(0, 27, 479, 235, 0x101418);
    s.eraseAll(0x000000);
    s.print(ScreenTextSize::Medium, 8, 4, 0x101418, 0xE8B23A, "100% -- %s %d", 36);
    const std::vector<std::string> want = {
        "set_pen 39c36e",
        "fill 6 180 300 234 pen=39c36e",
        "set_pen c8d8f0",
        "rect 6 180 300 234 pen=c8d8f0",
        "set_eraser 101418",
        "erase 0 27 479 235 eraser=101418",
        "set_eraser 000000",
        "erase-all eraser=000000",
        "set_eraser e8b23a",
        "set_pen 101418",
        "print_at format '%s'",
        "print M 8 4 pen=101418 eraser=e8b23a '100? -- ?s ?d'",
    };
    CHECK(calls() == want);
    // The adapter's source holds exactly one print_at, and its format is the literal "%s".
    const std::string src = readSource("include/shulib/hal/pros/brain_screen.hpp");
    std::size_t n = 0;
    for (std::size_t at = src.find("::pros::c::screen_print_at("); at != std::string::npos;
         at = src.find("::pros::c::screen_print_at(", at + 1)) {
        ++n;
    }
    CHECK(n == 1);
    CHECK(src.find("::pros::c::screen_print_at(formatOf(size), x, y, \"%s\", out);")
          != std::string::npos);
}

TEST_CASE("ProsBrainScreen A2: print() sanitizes, cuts at cols, pads, and maps each size") {
    // BUG CAUGHT: the adapter printing the raw text (a control byte or a UTF-8 dash drawn as
    // garbage); the cut ignored (a line runs off the screen); SMALL and LARGE swapped.
    pros::shim::resetScreen();
    ProsBrainScreen s;
    s.print(ScreenTextSize::Small, 1, 2, 0xFFFFFF, 0x000000, "a\tb \xE2\x80\x94 c", 50);
    s.print(ScreenTextSize::Large, 3, 4, 0xFFFFFF, 0x000000, "ABCDEFGHIJKLMNOPQRSTUVWXYZ", 18);
    s.print(ScreenTextSize::Medium, 5, 6, 0xFFFFFF, 0x000000, "pad", 36, 7);
    s.print(ScreenTextSize::Small, 7, 8, 0xFFFFFF, 0x000000, nullptr, 50, 2);
    const std::vector<std::string> want = {
        "print S 1 2 pen=ffffff eraser=000000 'a?b - c'",
        "print L 3 4 pen=ffffff eraser=000000 'ABCDEFGHIJKLMNOPQR'",
        "print M 5 6 pen=ffffff eraser=000000 'pad    '",
        "print S 7 8 pen=ffffff eraser=000000 '  '",
    };
    CHECK(draws() == want);
    // The shim tells all five PROS formats apart, so a size mapped to a CENTRED format records
    // something the fake never does (LARGE and LARGE_CENTER would otherwise both read "L").
    pros::shim::resetScreen();
    const pros::text_format_e_t formats[] = {pros::E_TEXT_SMALL, pros::E_TEXT_MEDIUM,
                                             pros::E_TEXT_LARGE, pros::E_TEXT_MEDIUM_CENTER,
                                             pros::E_TEXT_LARGE_CENTER};
    for (const pros::text_format_e_t f : formats) {
        pros::c::screen_print_at(f, 0, 0, "%s", "x");
    }
    CHECK(draws() == std::vector<std::string>{"print S 0 0 pen=ffffff eraser=000000 'x'",
                                              "print M 0 0 pen=ffffff eraser=000000 'x'",
                                              "print L 0 0 pen=ffffff eraser=000000 'x'",
                                              "print MC 0 0 pen=ffffff eraser=000000 'x'",
                                              "print LC 0 0 pen=ffffff eraser=000000 'x'"});
}

TEST_CASE("ProsBrainScreen A3: a good touch read passes the count and the place; an error read is "
          "an error, never a tap") {
    // BUG CAUGHT (the tester's own defect of R2a, 2026-09-24): an E_TOUCH_ERROR read — every field
    // -1 — taken as a release at (-1, -1); the release count or the place dropped; the PRESS count
    // read as the release count (a reader would act when the finger goes DOWN, against the
    // release-edge design — the shim moves the two counts in separate steps so this can show).
    pros::shim::resetScreen();
    ProsBrainScreen s;
    shulib::hal::ScreenTouch t = s.touch();
    CHECK_FALSE(t.error);
    CHECK(t.releaseCount == 0);
    // A finger down: the place moves, the release count does not.
    pros::shim::screenPress(240, 130);
    t = s.touch();
    CHECK_FALSE(t.error);
    CHECK(t.releaseCount == 0);
    CHECK(t.x == 240);
    CHECK(t.y == 130);
    // Lifted: one release.
    pros::shim::screenRelease();
    t = s.touch();
    CHECK_FALSE(t.error);
    CHECK(t.releaseCount == 1);
    CHECK(t.x == 240);
    CHECK(t.y == 130);
    pros::shim::screenState().touchError = true;
    t = s.touch();
    CHECK(t.error);
    CHECK(t.releaseCount == -1);
    CHECK(t.x == -1);
    CHECK(t.y == -1);
    CHECK(pros::shim::screenState().touchReads == 4);
    // Held and pressed reads are not errors.
    pros::shim::screenState().touchError = false;
    pros::shim::screenState().touch.touch_status = pros::E_TOUCH_HELD;
    CHECK_FALSE(s.touch().error);
}

TEST_CASE("ProsBrainScreen A4: prints() counts every print and nothing else") {
    // BUG CAUGHT: a count that misses prints (a program's report of how many it made would
    // under-state them) or counts fills and erases too.
    pros::shim::resetScreen();
    ProsBrainScreen s;
    s.fillRect(0, 0, 1, 1, 0);
    s.eraseAll(0);
    CHECK(s.prints() == 0);
    for (int k = 0; k < 7; ++k) {
        s.print(ScreenTextSize::Small, 0, 0, 0, 0, "x", 50);
    }
    CHECK(s.prints() == 7);
    CHECK(pros::shim::screenState().prints == 7);
    ProsBrainScreen other;  // the count is per object
    CHECK(other.prints() == 0);
}

TEST_CASE("ProsBrainScreen A5: the painter through the PROS adapter draws exactly what it draws on "
          "the fake") {
    // BUG CAUGHT: the fake drifting from the adapter (a host test of a program's screen on the fake
    // would then say nothing about the robot's) — a sanitizer applied in one and not the other, a
    // colour recorded from the wrong call, a size mapped differently.
    pros::shim::resetScreen();
    ProsBrainScreen adapter;
    CalibratePainter onPros{adapter, "BUILD 01:02:03"};
    paintTheSequence(onPros);
    FakeBrainScreen fake;
    CalibratePainter onFake{fake, "BUILD 01:02:03"};
    paintTheSequence(onFake);
    REQUIRE(fake.log().size() > 60);
    CHECK(draws() == fake.log());
    CHECK(adapter.prints() == fake.prints());
}

TEST_CASE("ProsBrainScreen A5b: the fake's touch reads equal the adapter's, read for read -- a "
          "press, a release, a tap, error reads, and the good read after them") {
    // BUG CAUGHT: the fake drifting from PROS on the touch — an error read that keeps the last tap
    // where PROS answers -1 in every field (a reader that takes its touch baseline from an error
    // read would see a phantom tap on the robot, the next good read a "new" release at the last
    // place — DONE or ABORT — and nothing on the fake, where A2's program tests run); a press
    // that moves the release count on one and not the other.
    pros::shim::resetScreen();
    ProsBrainScreen adapter;
    FakeBrainScreen fake;
    std::vector<shulib::hal::ScreenTouch> onPros;
    std::vector<shulib::hal::ScreenTouch> onFake;
    const auto read = [&]() {
        onPros.push_back(adapter.touch());
        onFake.push_back(fake.touch());
    };
    read();  // 0: nothing yet
    pros::shim::screenPress(100, 50);
    fake.press(100, 50);
    read();  // 1: a finger down
    pros::shim::screenRelease();
    fake.release();
    read();  // 2: lifted
    pros::shim::screenTap(400, 200);
    fake.tap(400, 200);
    read();  // 3: a tap
    pros::shim::screenState().touchError = true;
    fake.setTouchError(true);
    read();  // 4: an error read
    pros::shim::screenTap(10, 20);
    fake.tap(10, 20);
    read();  // 5: an error read, a tap made meanwhile
    pros::shim::screenState().touchError = false;
    fake.setTouchError(false);
    read();  // 6: the good read after them
    REQUIRE(onPros.size() == 7);
    REQUIRE(onFake.size() == onPros.size());
    for (std::size_t i = 0; i < onPros.size(); ++i) {
        INFO("read ", i);
        CHECK(onFake[i].error == onPros[i].error);
        CHECK(onFake[i].releaseCount == onPros[i].releaseCount);
        CHECK(onFake[i].x == onPros[i].x);
        CHECK(onFake[i].y == onPros[i].y);
    }
    // And the reads are PROS's: no release on a press, -1 everywhere on an error, the tap after.
    CHECK(onPros[1].releaseCount == 0);
    CHECK(onPros[4].error);
    CHECK(onPros[4].releaseCount == -1);
    CHECK(onPros[4].x == -1);
    CHECK(onPros[6].releaseCount == 3);
    CHECK(onPros[6].x == 10);
}

TEST_CASE("PROS shim (CAL1 pieces) A6a: the competition reads return the programmed status and the "
          "vendored bits") {
    // BUG CAUGHT: a shim whose bits differ from PROS's (a host test would then pass a program that
    // misreads the field on the robot); a read that is not counted.
    pros::shim::resetCompetition();
    static_assert(COMPETITION_DISABLED == 1 && COMPETITION_AUTONOMOUS == 2
                  && COMPETITION_CONNECTED == 4 && COMPETITION_SYSTEM == 8);
    CHECK(pros::c::competition_get_status() == 0);
    CHECK(pros::c::competition_is_disabled() == 0);
    pros::shim::competitionStatus() = COMPETITION_DISABLED | COMPETITION_CONNECTED;
    CHECK(pros::c::competition_get_status() == 5);
    CHECK(pros::c::competition_is_disabled() == 1);
    CHECK(pros::c::competition_is_connected() == 1);
    CHECK(pros::c::competition_is_autonomous() == 0);
    pros::shim::competitionStatus() = COMPETITION_AUTONOMOUS;
    CHECK(pros::c::competition_is_autonomous() == 1);
    CHECK(pros::c::competition_is_disabled() == 0);
    CHECK(pros::shim::competitionReads() == 8);
    pros::shim::resetCompetition();
    CHECK(pros::shim::competitionStatus() == 0);
    CHECK(pros::shim::competitionReads() == 0);
}

TEST_CASE("PROS shim (CAL1 pieces) A6b: the three C motor reads read the device and write NOTHING") {
    // BUG CAUGHT: a C read that writes the gearing or the units (the watch it serves must leave a
    // declared mechanism motor exactly as it found it); counts converted at one fixed gearing (a
    // degrees conversion by the REPORTED gearing would pass for green and be wrong for red and
    // blue); a negative port not negating; a dead port answering a number.
    pros::shim::resetAll();
    pros::shim::resetMotorCReads();
    auto& m = pros::shim::motorState(6);
    m.positionOutputDeg = 90.0;
    const auto before = m;  // every write counter, and the state, as found
    // The shim's adversarial default: rotations, red.
    CHECK(pros::c::motor_get_encoder_units(6) == pros::E_MOTOR_ENCODER_ROTATIONS);
    CHECK(pros::c::motor_get_gearing(6) == pros::E_MOTOR_GEARSET_36);
    CHECK(pros::c::motor_get_position(6) == doctest::Approx(0.25));
    CHECK(pros::c::motor_get_position(-6) == doctest::Approx(-0.25));
    m.units = pros::MotorUnits::degrees;
    CHECK(pros::c::motor_get_position(6) == doctest::Approx(90.0));
    m.units = pros::MotorUnits::counts;
    CHECK(pros::c::motor_get_position(6) == doctest::Approx(450.0));  // red: 1800 a turn
    m.gearing = pros::MotorGears::green;
    CHECK(pros::c::motor_get_gearing(6) == pros::E_MOTOR_GEARSET_18);
    CHECK(pros::c::motor_get_position(6) == doctest::Approx(225.0));  // green: 900 a turn
    m.gearing = pros::MotorGears::blue;
    CHECK(pros::c::motor_get_gearing(6) == pros::E_MOTOR_GEAR_BLUE);
    CHECK(pros::c::motor_get_position(6) == doctest::Approx(75.0));  // blue: 300 a turn
    CHECK(pros::c::motor_get_encoder_units(6) == pros::E_MOTOR_ENCODER_COUNTS);
    m.gearing = pros::MotorGears::invalid;
    CHECK(pros::c::motor_get_position(6) == PROS_ERR_F);
    m.units = pros::MotorUnits::invalid;
    m.gearing = pros::MotorGears::blue;
    CHECK(pros::c::motor_get_position(6) == PROS_ERR_F);
    m.units = pros::MotorUnits::degrees;
    m.disconnected = true;
    CHECK(pros::c::motor_get_position(6) == PROS_ERR_F);
    CHECK(pros::c::motor_get_encoder_units(6) == pros::E_MOTOR_ENCODER_INVALID);
    CHECK(pros::c::motor_get_gearing(6) == pros::E_MOTOR_GEARSET_INVALID);
    m.disconnected = false;
    // NOT ONE write, on this port or any other.
    CHECK(m.setGearingCalls == before.setGearingCalls);
    CHECK(m.setEncoderUnitsCalls == before.setEncoderUnitsCalls);
    CHECK(m.moveVoltageCalls == before.moveVoltageCalls);
    CHECK(m.setBrakeModeCalls == before.setBrakeModeCalls);
    CHECK(m.brakeCalls == before.brakeCalls);
    CHECK(m.positionOutputDeg == before.positionOutputDeg);
    for (int p = 1; p <= 21; ++p) {
        const auto& s = pros::shim::motorState(p);
        CHECK(s.setGearingCalls + s.setEncoderUnitsCalls + s.moveVoltageCalls
                  + s.setBrakeModeCalls + s.brakeCalls
              == 0);
    }
    // The reads were counted, by |port|.
    CHECK(pros::shim::motorCReads()[6].position == 9);
    CHECK(pros::shim::motorCReads()[6].units == 3);
    CHECK(pros::shim::motorCReads()[6].gearing == 4);
    CHECK(pros::shim::motorCReads()[7].position == 0);
    pros::shim::resetMotorCReads();
    CHECK(pros::shim::motorCReads()[6].position == 0);
}

namespace {
/// One advance the hook saw: its (from, to), and the clock as the hook READ it.
struct Advance {
    std::uint64_t from;
    std::uint64_t to;
    std::uint64_t clockInHook;
    [[nodiscard]] std::pair<std::uint64_t, std::uint64_t> span() const { return {from, to}; }
};
std::vector<Advance>& advances() {
    static std::vector<Advance> a;
    return a;
}
void recordAdvance(std::uint64_t from, std::uint64_t to) {
    advances().push_back(Advance{from, to, pros::micros()});
}
/// Every advance so far was reported AFTER the clock moved: a plant stepped in the hook reads
/// the new time.
void checkClockMovedFirst() {
    for (const Advance& a : advances()) {
        CHECK(a.clockInHook == a.to);
    }
}
}  // namespace

TEST_CASE("PROS shim (CAL1 pieces) A6c: the advance hook -- called by every delay() and every "
          "delay_until() that blocks (the library's tick pacer included), uninstalled by "
          "resetTime(); with none, the clock is as before") {
    // BUG CAUGHT: a hook that fires before the clock moves (a plant stepped in it would read the
    // old time — each report records the clock the hook READS, which must be the `to`); a wait
    // the hook never sees — the library's ProsTickPacer waits with delay_until, so a program's
    // 10 ms loop on the host test plant would run with the plant frozen; a hook called by a
    // delay_until that does not block — its wake instant exactly now, or already past (a plant
    // stepped by zero time, or twice for one wait); a hook left installed across test cases (a
    // later case's clock would step a plant that no longer exists); a clock that behaves
    // differently with no hook (every existing pacing test would then be testing something new).
    pros::shim::resetTime();
    advances().clear();
    // No hook: the clock and its counters as they always were — delay_until included.
    pros::delay(10);
    pros::Task::delay(5);
    CHECK(pros::millis() == 15);
    CHECK(pros::micros() == 15000);
    CHECK(pros::shim::timeState().delayCalls == 2);
    std::uint32_t quiet = 15;
    pros::Task::delay_until(&quiet, 10);
    CHECK(pros::micros() == 25000);
    CHECK(quiet == 25);
    CHECK(pros::shim::timeState().delayUntilCalls == 1);
    pros::shim::resetTime();
    pros::delay(15);
    // Installed: every advance reports (from, to), after the clock moved.
    pros::shim::advanceHook() = &recordAdvance;
    pros::delay(10);
    pros::shim::advanceUs(250);
    REQUIRE(advances().size() == 2);
    CHECK(advances()[0].span() == std::pair<std::uint64_t, std::uint64_t>{15000, 25000});
    CHECK(advances()[1].span() == std::pair<std::uint64_t, std::uint64_t>{25000, 25250});
    // A delay_until that blocks reports its wait once, ending at the wake instant.
    std::uint32_t prev = pros::millis();  // 25 (the clock is at 25.25 ms)
    pros::Task::delay_until(&prev, 10);
    REQUIRE(advances().size() == 3);
    CHECK(advances()[2].span() == std::pair<std::uint64_t, std::uint64_t>{25250, 35000});
    CHECK(pros::millis() == 35);
    CHECK(prev == 35);
    // One whose wake instant is EXACTLY now does not block: no advance, no call.
    std::uint32_t exact = 25;  // 25 + 10 = 35 ms, the clock's time to the microsecond
    REQUIRE(pros::micros() == 35000);
    pros::Task::delay_until(&exact, 10);
    CHECK(advances().size() == 3);
    CHECK(pros::micros() == 35000);
    CHECK(exact == 35);
    // One whose wake instant is already past does not block: no advance, no call.
    std::uint32_t stale = 0;
    pros::Task::delay_until(&stale, 10);
    CHECK(advances().size() == 3);
    CHECK(pros::micros() == 35000);
    CHECK(stale == 10);
    // The library's own tick pacer, which a program's 10 ms loop paces with: its first pace
    // anchors and waits one tick, and every pace after waits to the next boundary — each wait
    // reported, so a host test plant stepped in the hook steps with the loop.
    shulib::hal::pros::ProsTickPacer pacer;
    REQUIRE(advances().size() == 3);
    checkClockMovedFirst();  // the delay(), advanceUs() and delay_until() reports above
    advances().clear();
    pacer.pace();
    pacer.pace();
    pros::shim::advanceUs(3000);  // a 3 ms tick body: the next pace absorbs it
    pacer.pace();
    REQUIRE(advances().size() == 4);
    CHECK(advances()[0].span() == std::pair<std::uint64_t, std::uint64_t>{35000, 45000});
    CHECK(advances()[1].span() == std::pair<std::uint64_t, std::uint64_t>{45000, 55000});
    CHECK(advances()[2].span() == std::pair<std::uint64_t, std::uint64_t>{55000, 58000});
    CHECK(advances()[3].span() == std::pair<std::uint64_t, std::uint64_t>{58000, 65000});
    checkClockMovedFirst();
    // resetTime() uninstalls it.
    pros::shim::resetTime();
    CHECK(pros::shim::advanceHook() == nullptr);
    pros::delay(10);
    std::uint32_t after = 0;
    pros::Task::delay_until(&after, 20);
    CHECK(advances().size() == 4);
    CHECK(pros::millis() == 20);
}

// A6d's compile-time half: the five entry points redeclared here WITH C linkage. Were any of them
// declared in the host main.h with C++ linkage, this block would not compile — one function given
// two language linkages in one translation unit is ill-formed, and the compiler must say so
// ([dcl.link]). A team's file defines them under main.h's declarations, so their linkage is the
// one the PROS kernel links against.
extern "C" {
void autonomous(void);
void initialize(void);
void disabled(void);
void competition_initialize(void);
void opcontrol(void);
}

TEST_CASE("PROS shim (CAL1 pieces) A6d: the host main.h declares PROS's five entry points, and "
          "carries the shim's guard") {
    // BUG CAUGHT: a host main.h missing an entry point or declaring one with another signature — a
    // team's file that defines all five would then not compile in the suite as written; and a host
    // main.h without the shim's #error guard. The fence test's guard scan
    // (test/pros_adapter_fence_test.cpp, "shim: every header") walks test/pros_shim/pros/ only, and
    // this file sits beside that folder, so this case is its one check: were test/pros_shim/ ever
    // on a robot build's include path, the guard makes `#include "main.h"` stop that build here
    // rather than answer it with a file that is not PROS's.
    static_assert(std::is_same_v<decltype(&initialize), void (*)()>);
    static_assert(std::is_same_v<decltype(&disabled), void (*)()>);
    static_assert(std::is_same_v<decltype(&competition_initialize), void (*)()>);
    static_assert(std::is_same_v<decltype(&autonomous), void (*)()>);
    static_assert(std::is_same_v<decltype(&opcontrol), void (*)()>);
    // And each with C linkage, as the project's include/main.h has it — proven at compile time
    // above; here, the text: each declaration INSIDE the extern "C" block (after its opening
    // brace and before the brace that closes it), never merely after one.
    const std::string shim = readSource("test/pros_shim/main.h");
    const std::string real = readSource("include/main.h");
    const auto blockOpen = shim.find("extern \"C\" {");
    REQUIRE(blockOpen != std::string::npos);
    const auto blockClose = shim.find('}', blockOpen);
    REQUIRE(blockClose != std::string::npos);
    for (const char* decl : {"void autonomous(void);", "void initialize(void);",
                             "void disabled(void);", "void competition_initialize(void);",
                             "void opcontrol(void);"}) {
        CHECK_MESSAGE(real.find(decl) != std::string::npos, decl, " is not in include/main.h");
        const auto at = shim.find(decl);
        CHECK_MESSAGE(at != std::string::npos, decl, " is not in the host main.h");
        CHECK_MESSAGE(at > blockOpen, decl, " is declared before the extern \"C\" block");
        CHECK_MESSAGE(at < blockClose, decl, " is declared after the extern \"C\" block closed");
    }
    // The guard, as every header under test/pros_shim/pros/ carries it: the #ifndef, then the
    // #error inside it, both before the first declaration.
    const auto guard = shim.find("#ifndef SHULIB_HOST_PROS_SHIM");
    const auto error = shim.find("#error", guard == std::string::npos ? 0 : guard);
    const auto firstDecl = shim.find("extern \"C\"");
    CHECK_MESSAGE(guard != std::string::npos, "the host main.h has no SHULIB_HOST_PROS_SHIM guard");
    CHECK_MESSAGE(error != std::string::npos, "the host main.h's guard has no #error");
    CHECK(error < firstDecl);
}
