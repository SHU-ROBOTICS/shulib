// Tests for diag/controller_display.hpp + hal/line_display.hpp — D-4. What each targets:
//  * CONTENT: the screen must carry the latched FIRST fault by name and a one-word
//    state — the "why did it stop" a student reads at the field with no laptop.
//  * WRITE DISCIPLINE: unchanged rows must not be rewritten (V5 controller writes
//    are slow and rate-limited; a per-tick repaint starves the display).
//  * GEOMETRY: content must fit the 19-column device; the seam truncates, never
//    wraps (a wrapped row corrupts the row below it).

#include "doctest.h"

#include <string_view>

#include <algorithm>

#include "shulib/diag/controller_display.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/hal/fake/fake_battery.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_line_display.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/hal/line_display.hpp"
#include "shulib/units/quantity.hpp"

using shulib::PreconditionError;
using shulib::diag::ControllerFaultDisplay;
using shulib::diag::FaultCode;
using shulib::diag::FaultLatch;
using shulib::hal::ILineDisplay;
using shulib::hal::fake::FakeBattery;
using shulib::hal::fake::FakeClock;
using shulib::hal::fake::FakeLineDisplay;
using shulib::hal::fake::FakeTelemetrySink;
using shulib::units::Time;
using shulib::units::Voltage;

namespace {
struct DisplayRig {
    FakeTelemetrySink sink;
    FakeClock clock;
    FaultLatch latch{sink, clock};
    FakeBattery battery{};
    FakeLineDisplay screen;
    ControllerFaultDisplay display{screen, latch, battery};
};
}  // namespace

// Bug caught: the healthy screen not saying so (a blank screen and a crashed
// program must never look alike), or a row's shape drifting.
TEST_CASE("D-4: healthy content — OK state, no fault, battery + count") {
    DisplayRig rig;
    rig.battery.setVoltage(Voltage{12.4});
    rig.display.update(Time{12.34});
    CHECK(rig.screen.row(0) == "OK    t    12.3s");
    CHECK(rig.screen.row(1) == "flt none");
    CHECK(rig.screen.row(2) == "batt 12.4V n 0");
}

// Bug caught: THE D-4 requirement — the latched FIRST fault (the root cause) not
// reaching the screen by name, or a cascade replacing it with the loudest-latest.
TEST_CASE("D-4: a latched fault flips the state word and names the FIRST fault") {
    DisplayRig rig;
    rig.battery.setVoltage(Voltage{11.9});
    rig.latch.raise(FaultCode::OdoStuck, "LOC", "");
    rig.latch.raise(FaultCode::MotionTimeout, "MOT", "");  // cascade
    rig.display.update(Time{45.6});
    CHECK(rig.screen.row(0) == "FAULT t    45.6s");
    CHECK(rig.screen.row(1) == "flt ODO_STUCK");  // the ROOT CAUSE, not the cascade
    CHECK(rig.screen.row(2) == "batt 11.9V n 2");
}

// Bug caught: the per-tick repaint — a 100 Hz loop calling update() must cost
// ZERO device writes in steady state (write counts are the probe; on the real
// V5 each write is slow and firmware-rate-limited).
TEST_CASE("D-4: unchanged rows are never rewritten — steady state costs no device "
          "writes") {
    DisplayRig rig;
    rig.battery.setVoltage(Voltage{12.4});
    rig.display.update(Time{10.00});
    const int after1 = rig.screen.totalWrites();
    CHECK(after1 == 3);  // first paint writes all rows
    // 9 more ticks inside the same 0.1 s clock quantum, same battery, no fault:
    for (int i = 1; i < 10; ++i) {
        rig.display.update(Time{10.00 + 0.009 * i});
    }
    CHECK(rig.screen.totalWrites() == after1);  // zero rewrites
    // The clock crossing its 0.1 s quantum repaints ONLY row 0.
    rig.display.update(Time{10.13});
    CHECK(rig.screen.writeCount(0) == 2);
    CHECK(rig.screen.writeCount(1) == 1);
    CHECK(rig.screen.writeCount(2) == 1);
}

// Bug caught: millivolt battery jitter repainting row 2 every tick (the 0.1 V
// display quantum exists exactly so noise is not a write).
TEST_CASE("D-4: sub-quantum battery jitter does not repaint; a real sag does") {
    DisplayRig rig;
    rig.battery.setVoltage(Voltage{12.40});
    rig.display.update(Time{1.0});
    rig.battery.setVoltage(Voltage{12.43});  // jitter inside the 0.1 V quantum
    rig.display.update(Time{1.0});
    CHECK(rig.screen.writeCount(2) == 1);
    rig.battery.setVoltage(Voltage{11.9});   // a real sag
    rig.display.update(Time{1.0});
    CHECK(rig.screen.writeCount(2) == 2);
    CHECK(rig.screen.row(2) == "batt 11.9V n 0");
}

// Bug caught: content silently outgrowing the 19-column device — the longest
// fault spellings must FIT (they do, exactly), and anything longer truncates at
// the seam rather than wrapping into the next row.
TEST_CASE("D-4: the longest fault names fit the 19-column row; the seam truncates, "
          "never wraps") {
    DisplayRig rig;
    rig.latch.raise(FaultCode::GpsGateReject, "LOC", "");  // 15-char name
    rig.display.update(Time{0.0});
    CHECK(rig.screen.row(1) == "flt GPS_GATE_REJECT");
    CHECK(rig.screen.row(1).size() == 19);  // exactly the device width — no slack left

    FakeLineDisplay screen;
    screen.setLine(0, "0123456789012345678PAST-THE-EDGE");
    CHECK(screen.row(0) == "0123456789012345678");  // truncated at kCols
    CHECK(ILineDisplay::kRows == 3);
    CHECK(ILineDisplay::kCols == 19);  // HA-57: the geometry claim, pinned
}

// Bug caught: the fake accepting out-of-range rows (a test could "write row 5"
// and pass while the real device corrupts memory or throws firmware-side).
TEST_CASE("D-4: FakeLineDisplay bounds its rows loudly") {
    FakeLineDisplay screen;
    CHECK_THROWS_AS(screen.setLine(-1, "x"), PreconditionError);
    CHECK_THROWS_AS(screen.setLine(3, "x"), PreconditionError);
    CHECK_THROWS_AS((void)screen.row(3), PreconditionError);
    CHECK_THROWS_AS((void)screen.writeCount(-1), PreconditionError);
}

// Bug caught (DEFECTS1 item D2): the banner claimed row 1's fault-name column was "checked
// by static math here" and there was no static_assert in the file at all — and the
// arithmetic it asserted went stale the day chunk F1 appended MECHANISM_STALLED (17 chars
// against a 15-char budget). The compile-time check now exists in the header; this pins the
// runtime half so the two cannot drift: the longest name is genuinely over budget, the seam
// truncates rather than wraps, and the truncation still names exactly one code.
TEST_CASE("D2: the longest fault name overruns row 1 and still identifies one code") {
    const int budget = shulib::hal::ILineDisplay::kCols - 4;  // "flt "
    CHECK(budget == 15);

    const std::string_view longest{shulib::diag::faultCodeName(FaultCode::MechanismStalled)};
    CHECK(longest.size() == 17);           // over budget — the banner said 15 was the max
    const std::string_view truncated = longest.substr(0, static_cast<std::size_t>(budget));
    CHECK(truncated == "MECHANISM_STALL");

    // No OTHER code shares that prefix, which is the property the static_assert enforces and
    // the only one that makes a truncated row trustworthy.
    for (int i = 0; i <= 11; ++i) {
        const auto code = static_cast<FaultCode>(i);
        if (code == FaultCode::MechanismStalled) { continue; }
        const std::string_view other{shulib::diag::faultCodeName(code)};
        CHECK(other.substr(0, std::min(static_cast<std::size_t>(budget), other.size()))
              != truncated);
    }
}
