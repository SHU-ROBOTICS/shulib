// Adapter tests for ProsClock, ProsTickPacer, ProsBattery, ProsCharSink and
// ProsLineDisplay THROUGH THE HOST SHIM (chunk R1a). The shim tests the
// adapter against our belief about PROS (HA-99..102, HA-57/107); hardware
// tests the belief — the battery's units especially (the vendored source
// documents NONE).

#include "doctest.h"

#include <cstdio>
#include <string>

#include "pros/shim_control.hpp"

#include "shulib/hal/pros/battery.hpp"
#include "shulib/hal/pros/char_sink.hpp"
#include "shulib/hal/pros/clock.hpp"
#include "shulib/hal/pros/line_display.hpp"
#include "shulib/hal/pros/tick_pacer.hpp"

using shulib::hal::pros::DisplayController;
using shulib::hal::pros::ProsBattery;
using shulib::hal::pros::ProsCharSink;
using shulib::hal::pros::ProsClock;
using shulib::hal::pros::ProsLineDisplay;
using shulib::hal::pros::ProsTickPacer;

TEST_CASE("ProsClock: µs → seconds ×1e-6, monotonic (HA-101)") {
    // BUG CAUGHT: binding millis() (1 ms quantization = 10% of the control
    // tick, a 10% error in every PID derivative), or a wrong scale — 2.5 s of
    // shim time must read exactly 2.5, not 2500 (ms passed through) and not
    // 2.5e6 (raw µs).
    pros::shim::resetAll();
    ProsClock clock;
    CHECK(clock.now().value() == doctest::Approx(0.0));
    pros::shim::advanceUs(2'500'000);
    CHECK(clock.now().value() == doctest::Approx(2.5));
    // Sub-millisecond resolution IS the reason micros() won (brief T6):
    pros::shim::advanceUs(250);  // +0.25 ms — invisible to a millis() binding
    CHECK(clock.now().value() == doctest::Approx(2.50025));
    // Monotonic: repeated reads never decrease.
    const double a = clock.now().value();
    CHECK(clock.now().value() >= a);
}

TEST_CASE("ProsTickPacer: delay_until anchors cadence to the timeline, not the work (HA-102)") {
    // BUG CAUGHT: pacing with pros::delay(10) — each tick's processing time
    // ADDS to the period (2 ms of work → a 12 ms loop, 20% slow, and profiles
    // integrate the error forever). delay_until must land ticks at exactly
    // t0+10, t0+20, t0+30 ms even when work consumes time in between.
    pros::shim::resetAll();
    pros::shim::advanceUs(5'000'000);  // pacing starts late: t0 = 5000 ms
    ProsTickPacer pacer;
    pacer.pace();  // first pace anchors at t0 and wakes at t0+10
    CHECK(pros::millis() == 5010);
    pros::shim::advanceUs(2'000);  // 2 ms of "work" inside the tick
    pacer.pace();
    CHECK(pros::millis() == 5020);  // still lands on the boundary — work absorbed
    pacer.pace();
    CHECK(pros::millis() == 5030);
    CHECK(pros::shim::timeState().delayUntilCalls == 3);
    CHECK(pros::shim::timeState().lastDelayUntilDelta == 10);
}

TEST_CASE("ProsBattery: mV→V, mA→A, percent→[0,1] — all three scales wired (HA-99/100)") {
    // BUG CAUGHT: the ÷1000 dropped — 12600 "volts" makes brownout
    // compensation divide every motor command by ~1000 (the robot creeps), or
    // the reverse error floors it. THE UNIT ITSELF IS THE WEAKEST BELIEF IN
    // R1a (not in the vendored source at all — website only); the bench
    // measures the raw integer before anything drives.
    pros::shim::resetAll();
    ProsBattery b;
    pros::shim::batteryState().voltageMv = 12345;
    pros::shim::batteryState().currentMa = 2500;
    pros::shim::batteryState().capacityPercent = 87.5;
    CHECK(b.voltage().value() == doctest::Approx(12.345));
    CHECK(b.current().value() == doctest::Approx(2.5));
    CHECK(b.capacity() == doctest::Approx(0.875));
}

TEST_CASE("ProsBattery: sentinel reads hold last-good — never zero (a 0 V read floors every command)") {
    // BUG CAUGHT (mutation M8 shape): PROS_ERR passing through as 2147483.647 V
    // (brownout compensation multiplies every command by ~0), or a screen
    // substituting 0 V (reads as the deepest possible brownout — same crash,
    // other direction).
    pros::shim::resetAll();
    ProsBattery b;
    pros::shim::batteryState().voltageMv = 12000;
    pros::shim::batteryState().currentMa = 750;
    pros::shim::batteryState().capacityPercent = 80.0;
    // Prime last-good with one real read of each quantity:
    CHECK(b.voltage().value() == doctest::Approx(12.0));
    CHECK(b.current().value() == doctest::Approx(0.75));
    CHECK(b.capacity() == doctest::Approx(0.8));
    pros::shim::batteryState().errored = true;
    CHECK(b.voltage().value() == doctest::Approx(12.0));  // held, not 2147483.647, not 0
    CHECK(b.current().value() == doctest::Approx(0.75));
    CHECK(b.capacity() == doctest::Approx(0.8));
    CHECK(b.faultedReads() == 3);
}

TEST_CASE("ProsCharSink: verbatim bytes, flushed, never throws (the ICharSink contract)") {
    // BUG CAUGHT: sanitizing/reframing in the sink (the FORMATTER owns that —
    // double-sanitizing corrupts the §18.3 line shape), or buffering without
    // flush (a crash eats the last lines — the ones that matter).
    std::FILE* f = std::tmpfile();
    REQUIRE(f != nullptr);
    {
        ProsCharSink sink{f};
        sink.write("[t=  1.00] [SES] run start\n");
        sink.write("second line\n");
    }
    std::rewind(f);
    char buf[128] = {};
    const std::size_t n = std::fread(buf, 1, sizeof buf - 1, f);
    CHECK(std::string(buf, n) == "[t=  1.00] [SES] run start\nsecond line\n");
    std::fclose(f);
}

TEST_CASE("ProsLineDisplay: TRUNCATES at 19 columns — never wraps (HA-57)") {
    // BUG CAUGHT: a >19-char status line wrapping onto the row below and
    // corrupting it — the D-4 display would show fault text over the pose
    // row. One setLine call must produce exactly one set_text of exactly
    // kCols bytes.
    pros::shim::resetAll();
    ProsLineDisplay d{DisplayController::Master};
    d.setLine(0, "0123456789ABCDEFGHIJKLMNOP");  // 26 chars in
    auto& s = pros::shim::controllerState(pros::E_CONTROLLER_MASTER);
    CHECK(s.setTextCalls == 1);
    CHECK(s.lcdLines[0] == "0123456789ABCDEFGHI");  // 19 out — the J never lands
    CHECK(s.lcdLines[0].size() == 19);
}

TEST_CASE("ProsLineDisplay: pads to a full-row OVERWRITE — no ghost tail from the previous text") {
    // BUG CAUGHT: set_text leaving the old row's tail beyond the new text's
    // end — "ARM OK" over "ARMED FAULT" reading "ARM OKFAULT" on the driver's
    // screen mid-match.
    pros::shim::resetAll();
    ProsLineDisplay d{DisplayController::Master};
    d.setLine(1, "ARMED FAULT");
    d.setLine(1, "ARM OK");
    auto& s = pros::shim::controllerState(pros::E_CONTROLLER_MASTER);
    CHECK(s.lcdLines[1] == "ARM OK             ");  // 19 bytes, space-padded
    CHECK(s.lcdLines[1].size() == 19);
}

TEST_CASE("ProsLineDisplay: an out-of-range row is dropped, never thrown (MUST NOT throw)") {
    // BUG CAUGHT: a bad row index reaching set_text (PROS returns an error we
    // would ignore anyway) or throwing from the no-throw seam — the display
    // path runs inside the fault handler, where a throw would mask the
    // original fault.
    pros::shim::resetAll();
    ProsLineDisplay d{DisplayController::Master};
    CHECK_NOTHROW(d.setLine(3, "off the end"));
    CHECK_NOTHROW(d.setLine(-1, "before the start"));
    CHECK(pros::shim::controllerState(pros::E_CONTROLLER_MASTER).setTextCalls == 0);
}
