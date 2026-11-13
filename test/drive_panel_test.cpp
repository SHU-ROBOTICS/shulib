// drive_panel_test.cpp — "shulib Drive"'s SCREEN as a pure model (src/programs/drive_panel.hpp;
// chunk T4 Part 3, 2026-10-01; the T4 brief's P3-T2): what its MAIN, DETAILS and MORE pages hold
// and in which order, every row's words and tone, every row within its budget, and the page flip
// — on the RELEASE edge only, never across a held press, never inside the arming delay.
//
// The program cannot be compiled on the host (it paints through pros/screen.h), so everything a
// page says is made here, by pure functions the program only calls; test/drive_program_source_test
// .cpp pins that the program calls them. NOTHING HERE HAS RUN ON A ROBOT; every layout number is
// an ESTIMATE (A4 register HA-136).
//
// (The PROTECTION row's words, the controller row and the controller's three lines are the shared
// driver-panel header's — test/driver_panel_test.cpp pins them for both programs; this file hands
// those two MAIN rows in as plain rows. What the program repaints between two views is
// driveRepaint(), pinned here; the program's paint follows its plan — the source pins say so.)

#include "doctest.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <set>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "programs/drive_panel.hpp"
#include "shulib/hal/fake/fake_motor.hpp"
#include "shulib/hal/motor_group.hpp"
#include "shulib/units/quantity.hpp"

using shulib::bench::DriveButtonId;
using shulib::bench::DrivePage;
using shulib::bench::DrivePager;
using shulib::bench::DriveRows;
using shulib::bench::DriveView;
using shulib::bench::driveRow;
using shulib::bench::driveView;
using shulib::competition::MenuLine;
using shulib::teleop::DriveProtectionState;
using shulib::teleop::ProtectionMode;
using shulib::ui::Tone;
using shulib::ui::TouchSample;
namespace bench = shulib::bench;
namespace ui = shulib::ui;

namespace {

/// A DriveRows whose every row says its own name, in a tone of its own — so a view that shows the
/// wrong row, or a row twice, or drops a tone, is caught by name.
DriveRows namedRows() {
    DriveRows r;
    r.buildTime = "BUILD 12:34:56";
    const auto set = [](MenuLine& l, const char* name, Tone t) { l = driveRow(name, t); };
    set(r.protection, "row:protection", Tone::Green);
    set(r.controller, "row:controller", Tone::Red);
    set(r.status, "row:status", Tone::Amber);
    set(r.source1, "row:source1", Tone::Amber);
    set(r.source2, "row:source2", Tone::Green);
    set(r.claw, "row:claw", Tone::White);
    set(r.fault, "row:fault", Tone::Red);
    set(r.cmd, "row:cmd", Tone::White);
    set(r.est, "row:est", Tone::Amber);
    set(r.mode, "row:mode", Tone::White);
    set(r.battery, "row:battery", Tone::White);
    set(r.left, "row:left", Tone::Green);
    set(r.right, "row:right", Tone::Amber);
    set(r.travel, "row:travel", Tone::Grey);
    set(r.protections, "row:protections", Tone::Red);
    set(r.lastCut, "row:lastCut", Tone::Amber);
    set(r.peaks, "row:peaks", Tone::White);
    return r;
}

std::vector<std::string> lineTexts(const DriveView& v) {
    std::vector<std::string> out;
    for (std::size_t i = 0; i < v.lineCount; ++i) out.emplace_back(v.lines[i].text);
    return out;
}

/// A screen-safe row of the SMALL budget: printable ASCII, no '%', at most 50 characters.
bool screenSafe(const char* text) { return ui::screenText(text, ui::kSmallCols); }

/// The centre of a button's rectangle.
TouchSample tapAt(ui::Rect r, std::int32_t releaseCount) {
    TouchSample s;
    s.releaseCount = releaseCount;
    s.x = static_cast<std::int16_t>((r.x0 + r.x1) / 2);
    s.y = static_cast<std::int16_t>((r.y0 + r.y1) / 2);
    return s;
}

}  // namespace

// ═══ What is on which page (R5, D4) ═══════════════════════════════════════════════════════════
TEST_CASE("P3-T2: MAIN holds exactly the header bar, the PROTECTION row, the controller row and a "
          "DETAILS button — nothing else") {
    // BUG CAUGHT: the driver's page crowded with diagnostics again (R5: "the fault row, the claw
    // row, where the drivetrain numbers came from, the run status" are DETAILS'), a MAIN row
    // dropped, or the two swapped.
    const DriveRows rows = namedRows();
    const DriveView v = driveView(DrivePage::Main, rows, true);
    CHECK(v.page == DrivePage::Main);
    CHECK(std::string_view{v.header.text} == "shulib DRIVE");
    CHECK(std::string_view{v.barRight.text} == "BUILD 12:34:56");
    CHECK(v.bar == Tone::Blue);
    CHECK(lineTexts(v) == std::vector<std::string>{"row:protection", "row:controller"});
    CHECK(v.lines[0].tone == Tone::Green);  // a row keeps its own tone
    CHECK(v.lines[1].tone == Tone::Red);
    REQUIRE(v.buttonCount == 1U);
    CHECK(v.buttons[0].id == DriveButtonId::Details);
    CHECK(std::string_view{v.buttons[0].label} == "DETAILS");
    CHECK(v.buttons[0].rect == bench::kDriveRightSlot);
    CHECK(v.buttons[0].armed);
}

TEST_CASE("P3-T2: DETAILS opens with R5's four — the run status, where the drivetrain numbers came "
          "from (two rows), the claw row, the fault row — then the command, the estimate, the "
          "mode and the battery; MORE and BACK") {
    // BUG CAUGHT: R5's four pushed below the fold or reordered; the source's second row lost (a
    // DEGRADED reason's sentence would end mid-thought); a button on the wrong side.
    const DriveView v = driveView(DrivePage::Details, namedRows(), true);
    CHECK(std::string_view{v.header.text} == "shulib DRIVE  DETAILS");
    CHECK(lineTexts(v) == std::vector<std::string>{"row:status", "row:source1", "row:source2",
                                                   "row:claw", "row:fault", "row:cmd", "row:est",
                                                   "row:mode", "row:battery"});
    REQUIRE(v.buttonCount == 2U);
    CHECK(v.buttons[0].id == DriveButtonId::More);
    CHECK(v.buttons[0].rect == bench::kDriveLeftSlot);
    CHECK(std::string_view{v.buttons[0].label} == "MORE");
    CHECK(v.buttons[1].id == DriveButtonId::Back);
    CHECK(v.buttons[1].rect == bench::kDriveRightSlot);
    CHECK(std::string_view{v.buttons[1].label} == "BACK");
}

TEST_CASE("P3-T2: MORE holds the two groups, the travel and the protections' counters — the mode "
          "with its cuts and dead, the last cut, the peaks — and BACK") {
    const DriveView v = driveView(DrivePage::More, namedRows(), true);
    CHECK(std::string_view{v.header.text} == "shulib DRIVE  MORE");
    CHECK(lineTexts(v) == std::vector<std::string>{"row:left", "row:right", "row:travel",
                                                   "row:protections", "row:lastCut", "row:peaks"});
    REQUIRE(v.buttonCount == 1U);
    CHECK(v.buttons[0].id == DriveButtonId::Back);
    CHECK(v.buttons[0].rect == bench::kDriveRightSlot);
}

TEST_CASE("P3-T2: every row shows on exactly ONE page, each placed SMALL at x 8 from y 32 in 16 px "
          "steps above the button row; buttons are greyed (not armed) on request") {
    // BUG CAUGHT: a row of "shulib Teleop"'s panel lost in the split (it showed twelve; the three
    // pages show all seventeen of this program's), a row shown twice, rows drawn over the buttons.
    const DriveRows rows = namedRows();
    std::multiset<std::string> seen;
    for (const DrivePage p : {DrivePage::Main, DrivePage::Details, DrivePage::More}) {
        CAPTURE(static_cast<int>(p));
        const DriveView v = driveView(p, rows, false);
        for (std::size_t i = 0; i < v.lineCount; ++i) {
            const MenuLine& l = v.lines[i];
            seen.insert(l.text);
            CHECK(l.x == 8);
            CHECK(l.y == 32 + 16 * static_cast<int>(i));
            CHECK(l.size == ui::TextSize::Small);
            CHECK(l.y + ui::kSmallHeight <= bench::kDriveRightSlot.y0);
        }
        for (std::size_t b = 0; b < v.buttonCount; ++b) {
            CHECK_FALSE(v.buttons[b].armed);
            CHECK(ui::onScreen(v.buttons[b].rect));
        }
    }
    CHECK(seen.size() == 17U);
    CHECK(std::set<std::string>(seen.begin(), seen.end()).size() == 17U);  // none twice
}

// ═══ The rows' words ════════════════════════════════════════════════════════════════════════
TEST_CASE("P3-T2: the DETAILS rows — the command (amber and NO CONTROLLER when it is gone), the "
          "estimate (amber while DEGRADED), the mode (amber when field-centric fell back), the "
          "battery with or without the heading cross-check") {
    using Q = shulib::localization::Localizer::Quality;
    {
        const MenuLine l = bench::driveCmdRow(12.0, 0.5, true);
        CHECK(std::string_view{l.text} == "cmd vx  +12.0 in/s  w +0.50 rad/s");
        CHECK(l.tone == Tone::White);
        const MenuLine gone = bench::driveCmdRow(-3.3, -1.5, false);
        CHECK(std::string_view{gone.text} == "cmd vx   -3.3 in/s  w -1.50 rad/s  NO CONTROLLER");
        CHECK(gone.tone == Tone::Amber);
    }
    {
        const MenuLine l = bench::driveEstRow(10.0, -5.5, 90.0, Q::Corrected);
        CHECK(std::string_view{l.text} == "est x   +10.0  y    -5.5  hdg  +90.0 CORRECTED");
        CHECK(l.tone == Tone::White);
        const MenuLine bad = bench::driveEstRow(0.0, 0.0, 0.0, Q::Degraded);
        CHECK(std::string_view{bad.text} == "est x    +0.0  y    +0.0  hdg   +0.0 DEGRADED");
        CHECK(bad.tone == Tone::Amber);
        CHECK(std::string_view{bench::qualityName(Q::Uninitialized)} == "UNINIT");
        CHECK(std::string_view{bench::qualityName(Q::DeadReckon)} == "DEADRECKON");
    }
    {
        shulib::teleop::FieldCentricState off;
        const MenuLine l = bench::driveModeRow(shulib::teleop::DriveMode::ArcadeBody, off);
        CHECK(std::string_view{l.text} == "driver ARCADE BODY  field-centric OFF");
        CHECK(l.tone == Tone::White);
        shulib::teleop::FieldCentricState fell;
        fell.fellBack = true;
        fell.reason = shulib::teleop::FieldCentricReason::HeadingNotTrusted;
        const MenuLine f = bench::driveModeRow(shulib::teleop::DriveMode::ArcadeField, fell);
        CHECK(std::string_view{f.text} == "driver ARCADE FIELD  field-centric FELL BACK");
        CHECK(f.tone == Tone::Amber);
    }
    {
        CHECK(std::string_view{bench::driveBatteryRow(true, 0.0123, 12.6).text}
              == "hdg xcheck +0.0123 rad/tick  batt 12.60 V");
        CHECK(std::string_view{bench::driveBatteryRow(false, 5.0, 12.6).text} == "batt 12.60 V");
    }
}

TEST_CASE("P3-T2: the MORE rows — a side's group (its volts, present of all, disagreeing, and its "
          "DEAD ports by label: a boot hole AND a runtime mark), the travel, the protections' "
          "counters per mode, the last cut, the peaks") {
    // BUG CAUGHT: a runtime-dead port left off the dead list (only the boot holes read), the
    // present count taken from the slots, a mode shown in the wrong colour, a NaN peak printed.
    using shulib::hal::IMotor;
    using shulib::hal::fake::FakeMotor;
    std::array<FakeMotor, 4> motors{};
    std::array<IMotor*, 4> slots{&motors[0], nullptr, &motors[2], &motors[3]};  // 17: a boot hole
    const int labels[4] = {16, 17, 18, 20};
    shulib::hal::MotorGroup left{std::span<IMotor* const>{slots}, {}, labels};
    REQUIRE(left.markAbsent(2));  // 18 dies mid-run
    {
        const MenuLine l = bench::driveGroupRow("LEFT", left);
        CHECK(std::string_view{l.text} == "L  +0.0 V  2/4 present  0 disagree  dead 17 18");
        CHECK(l.tone == Tone::Amber);
    }
    std::array<FakeMotor, 4> rm{};
    std::array<IMotor*, 4> rs{&rm[0], &rm[1], &rm[2], &rm[3]};
    const int rlabels[4] = {11, 12, 13, 15};
    shulib::hal::MotorGroup right{std::span<IMotor* const>{rs}, {}, rlabels};
    right.setVoltage(shulib::units::Voltage{6.5});
    {
        const MenuLine l = bench::driveGroupRow("RIGHT", right);
        CHECK(std::string_view{l.text} == "R  +6.5 V  4/4 present  0 disagree");
        CHECK(l.tone == Tone::Green);
    }
    CHECK(std::string_view{bench::driveTravelRow(true, 1.5, -2.25).text}
          == "L/R travel  +1.50  -2.25 in (last tick)");
    CHECK(bench::driveTravelRow(true, 1.5, -2.25).tone == Tone::Grey);
    CHECK(std::string_view{bench::driveTravelRow(false, 1.5, -2.25).text}.empty());

    DriveProtectionState s{};
    CHECK(std::string_view{bench::driveProtectionCountsRow(s).text} == "protections DRIVING  cuts 0");
    CHECK(bench::driveProtectionCountsRow(s).tone == Tone::Green);
    s.mode = ProtectionMode::Degraded;
    s.cutCount = 2;
    std::snprintf(s.dead, sizeof s.dead, "%s", "18 13");
    CHECK(std::string_view{bench::driveProtectionCountsRow(s).text}
          == "protections DEGRADED  cuts 2  dead 18 13");
    CHECK(bench::driveProtectionCountsRow(s).tone == Tone::Amber);
    s.mode = ProtectionMode::Cut;
    CHECK(bench::driveProtectionCountsRow(s).tone == Tone::Red);
    CHECK(std::string_view{bench::driveProtectionCountsRow(s).text}.starts_with("protections CUT "));
    s.mode = ProtectionMode::Refused;
    CHECK(bench::driveProtectionCountsRow(s).tone == Tone::Red);
    CHECK(std::string_view{bench::driveProtectionCountsRow(s).text}
              .starts_with("protections REFUSED "));

    DriveProtectionState none{};
    CHECK(std::string_view{bench::driveLastCutRow(none).text} == "last cut: none this entry");
    CHECK(bench::driveLastCutRow(none).tone == Tone::Grey);
    std::snprintf(none.lastCut, sizeof none.lastCut, "%s", "CUT: FIGHT on LEFT -- port(s) 18");
    CHECK(std::string_view{bench::driveLastCutRow(none).text}
          == "last CUT: FIGHT on LEFT -- port(s) 18");
    CHECK(bench::driveLastCutRow(none).tone == Tone::Amber);

    DriveProtectionState p{};
    p.groupCount = 2;
    p.peakAmps[0] = 2.31;
    p.peakAmps[1] = 1.8;
    CHECK(std::string_view{bench::drivePeaksRow(p).text} == "peak A this entry: L 2.31  R 1.80");
    p.peakAmps[1] = std::numeric_limits<double>::quiet_NaN();
    CHECK(std::string_view{bench::drivePeaksRow(p).text} == "peak A this entry: L 2.31  R 0.00");
    p.groupCount = 3;
    p.peakAmps[1] = 2.0;
    p.peakAmps[2] = 3.0;
    CHECK(std::string_view{bench::drivePeaksRow(p).text}
          == "peak A this entry:  g0 2.31  g1 2.00  g2 3.00");
    p.groupCount = 99;  // never read past the array
    CHECK(std::string_view{bench::drivePeaksRow(p).text}.starts_with("peak A this entry:  g0 2.31"));
}

TEST_CASE("P3-T2: every row is screen-safe — printable ASCII, no '%', within the SMALL budget — at "
          "the extremes: huge and NaN numbers, the longest dead list and last cut") {
    // BUG CAUGHT: a row that runs off the screen, or carries a '%' PROS would read as a format.
    using Q = shulib::localization::Localizer::Quality;
    const double big = 1e12;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    std::vector<MenuLine> rows = {
        bench::driveCmdRow(big, -big, false),
        bench::driveCmdRow(nan, nan, true),
        bench::driveEstRow(-big, big, -1e9, Q::DeadReckon),
        bench::driveEstRow(nan, nan, nan, Q::Uninitialized),
        bench::driveBatteryRow(true, -big, big),
        bench::driveTravelRow(true, -big, big),
        driveRow("100% sure -- a stray percent", Tone::White),
    };
    DriveProtectionState s{};
    s.mode = ProtectionMode::Refused;
    s.cutCount = 2147483647;
    std::memset(s.dead, '9', sizeof s.dead - 1);
    std::memset(s.lastCut, 'C', sizeof s.lastCut - 1);
    s.groupCount = 4;
    for (double& a : s.peakAmps) a = big;
    rows.push_back(bench::driveProtectionCountsRow(s));
    rows.push_back(bench::driveLastCutRow(s));
    rows.push_back(bench::drivePeaksRow(s));
    for (const MenuLine& l : rows) {
        CAPTURE(l.text);
        CHECK(screenSafe(l.text));
        CHECK(std::string_view{l.text}.find('%') == std::string_view::npos);
    }
    // The program's own literals (each also static_asserted where it is written).
    for (const std::string_view w : {bench::kDriveHeaderWord, bench::kDriveDetailsWord,
                                     bench::kDriveMoreWord, bench::kDriveRefusedWord}) {
        CHECK(shulib::calibration::screenText(w, shulib::calibration::kHeaderCols));
    }
    CHECK(shulib::calibration::screenText(bench::kDriveScreenRunStatus, ui::kSmallCols));
}

TEST_CASE("P3-T2: the source rows — the typed table and the verified record each fit their two "
          "rows whole; a DEGRADED reason too long for two ends at a whole sentence with ` ...`") {
    // BUG CAUGHT (moved from "shulib Teleop", whose review found row 2 ending "... 1). NEW"): a
    // source row cut mid-word or mid-thought, or one that does not say more exists.
    const bench::SourceRows typed = bench::sourceRows(bench::kDriveTypedSource);
    CHECK(std::string_view{typed.row[0]} == "TYPED TABLE -- unverified since the rebuild; run");
    CHECK(std::string_view{typed.row[1]} == "shulib Calibrate");
    const bench::SourceRows degraded = bench::sourceRows(
        "MEASURED record | DEGRADED: dead 13 -- driving without them (left dead 0, right dead 1). "
        "NEW motors not in the calibration: 14");
    CHECK(std::string_view{degraded.row[1]}.ends_with(". ..."));
    for (const auto& row : degraded.row) CHECK(screenSafe(row));
}

// ═══ The page flip (DrivePager) ═════════════════════════════════════════════════════════════
TEST_CASE("P3-T2: the page flips on the RELEASE edge of a press that landed after the page "
          "appeared, once its arming delay has run — never on a press alone, never inside the "
          "delay, never on an error read, never off a button") {
    // BUG CAUGHT: a flip on the PRESS (a finger still down would flip, and the next page's button
    // under it would take the release); a tap made before the program existed replayed; a
    // reflexive double-tap on DETAILS landing on BACK in the same place a moment later; an error
    // read moving the count.
    DrivePager pager{7, 3};  // PROS's counts when the screen first appears: 7 releases, 3 presses
    CHECK(pager.page() == DrivePage::Main);
    CHECK_FALSE(pager.armed(0.0));
    CHECK_FALSE(pager.armed(0.49));
    CHECK(pager.armed(0.5));
    // A tap made BEFORE the program existed (counts unchanged) is never replayed.
    CHECK_FALSE(pager.touch(tapAt(bench::kDriveRightSlot, 7), 3, 1.0));
    // A PRESS alone — a finger down on DETAILS, its release still to come — flips nothing.
    CHECK_FALSE(pager.touch(tapAt(bench::kDriveRightSlot, 7), 4, 1.1));
    CHECK(pager.page() == DrivePage::Main);
    // An error read (every field -1 in PROS) flips nothing and moves nothing.
    TouchSample err;
    err.error = true;
    err.releaseCount = -1;
    CHECK_FALSE(pager.touch(err, -1, 1.2));
    // A release OFF every button flips nothing (and is consumed).
    TouchSample off;
    off.releaseCount = 8;
    off.x = 200;
    off.y = 100;
    CHECK_FALSE(pager.touch(off, 4, 1.3));
    // The release of a NEW press on DETAILS: the flip.
    CHECK(pager.touch(tapAt(bench::kDriveRightSlot, 9), 5, 1.4));
    CHECK(pager.page() == DrivePage::Details);
    // The same sample read again (no new release) does nothing.
    CHECK_FALSE(pager.touch(tapAt(bench::kDriveRightSlot, 9), 5, 1.5));
    // A reflexive second tap in the same place 0.2 s later lands on BACK — inside the new page's
    // arming delay: nothing, and the release is consumed (never replayed when the delay ends).
    CHECK_FALSE(pager.armed(1.6));
    CHECK_FALSE(pager.touch(tapAt(bench::kDriveRightSlot, 10), 6, 1.6));
    CHECK_FALSE(pager.touch(tapAt(bench::kDriveRightSlot, 10), 6, 2.5));
    CHECK(pager.page() == DrivePage::Details);
    // After the delay, MORE (left slot) flips to MORE, and BACK there to MAIN.
    CHECK(pager.armed(2.5));
    CHECK(pager.touch(tapAt(bench::kDriveLeftSlot, 11), 7, 2.6));
    CHECK(pager.page() == DrivePage::More);
    CHECK_FALSE(pager.touch(tapAt(bench::kDriveRightSlot, 12), 8, 2.7));  // MORE's delay
    CHECK(pager.touch(tapAt(bench::kDriveRightSlot, 13), 9, 3.2));
    CHECK(pager.page() == DrivePage::Main);
    // MAIN has no left button: a tap there does nothing.
    CHECK_FALSE(pager.touch(tapAt(bench::kDriveLeftSlot, 14), 10, 4.0));
    CHECK(pager.page() == DrivePage::Main);
}

TEST_CASE("P3-T2: a press held ACROSS a flip does nothing — its release lands on the new page's "
          "button, after the delay, and is still refused; the hit box's pad counts") {
    // BUG CAUGHT: a press that began on the old page releasing onto the new page's button in the
    // same place (only the arming delay stood between them — a slow finger outlasts it).
    DrivePager pager{0, 0};
    (void)pager.armed(0.0);
    CHECK(pager.touch(tapAt(bench::kDriveRightSlot, 1), 1, 1.0));  // tap 1 flips to DETAILS
    REQUIRE(pager.page() == DrivePage::Details);
    // A release whose press count did NOT advance since the flip: that finger went down before
    // the page appeared. Even long after the delay, it is refused.
    CHECK_FALSE(pager.touch(tapAt(bench::kDriveRightSlot, 2), 1, 5.0));
    CHECK(pager.page() == DrivePage::Details);
    // A release inside the hit box's pad but outside the drawn rectangle still hits.
    TouchSample padded;
    padded.releaseCount = 3;
    padded.x = static_cast<std::int16_t>(bench::kDriveRightSlot.x0 - ui::kHitPadX);
    padded.y = static_cast<std::int16_t>(bench::kDriveRightSlot.y0 - ui::kHitPadY);
    CHECK(pager.touch(padded, 2, 5.1));
    CHECK(pager.page() == DrivePage::Main);
}

// ═══ The repaint rule (driveRepaint): only the visible page, only what changed ═════════════════
// BUG CAUGHT: a page painted whole on every refresh (PROS leaks every screen print — R3d Part 2F's
// rule — and the panel flickers), a row whose text or tone changed left stale on the screen, a flip
// that repaints only the rows that differ (the old page's other rows and its buttons would stay), a
// button left greyed after its arming delay ran, or the first paint skipped.
TEST_CASE("P3-T2: driveRepaint — whole for the first paint, a new page or a new bar; else only the "
          "rows, the bar texts and the buttons that changed") {
    const DriveRows rows = namedRows();
    const DriveView main = driveView(DrivePage::Main, rows, false);
    // The first paint is whole, whatever it is compared with.
    CHECK(bench::driveRepaint(main, main, false).whole);
    // The same view again: nothing at all.
    const bench::DriveRepaint same = bench::driveRepaint(main, main, true);
    CHECK_FALSE(same.whole);
    CHECK_FALSE(same.header);
    CHECK_FALSE(same.barRight);
    CHECK(same.lines == 0);
    CHECK(same.buttons == 0);
    // A flip is whole, both ways.
    const DriveView details = driveView(DrivePage::Details, rows, true);
    CHECK(bench::driveRepaint(main, details, true).whole);
    CHECK(bench::driveRepaint(details, main, true).whole);
    // One row's TEXT changes: that row alone.
    DriveRows changed = namedRows();
    changed.controller = driveRow("row:controller", Tone::Red);
    changed.protection = driveRow("CUT 1 s #1: FIGHT on LEFT", Tone::Green);
    bench::DriveRepaint p = bench::driveRepaint(main, driveView(DrivePage::Main, changed, false), true);
    CHECK_FALSE(p.whole);
    CHECK(p.lines == 0b01);
    CHECK(p.buttons == 0);
    // One row's TONE changes (the same words, red now): that row alone.
    changed = namedRows();
    changed.controller = driveRow("row:controller", Tone::White);
    p = bench::driveRepaint(main, driveView(DrivePage::Main, changed, false), true);
    CHECK(p.lines == 0b10);
    // The arming delay ran out: the button alone, nothing else.
    p = bench::driveRepaint(main, driveView(DrivePage::Main, rows, true), true);
    CHECK_FALSE(p.whole);
    CHECK(p.lines == 0);
    CHECK(p.buttons == 0b01);
    // DETAILS: every row changes its text (a refresh) — every row, and the two buttons only if
    // their state changed.
    DriveRows next = namedRows();
    for (MenuLine* l : {&next.status, &next.source1, &next.source2, &next.claw, &next.fault,
                        &next.cmd, &next.est, &next.mode, &next.battery}) {
        std::string t = std::string{l->text} + "'";
        *l = driveRow(t, l->tone);
    }
    p = bench::driveRepaint(details, driveView(DrivePage::Details, next, true), true);
    CHECK_FALSE(p.whole);
    CHECK(p.lines == 0x1FF);  // nine rows
    CHECK(p.buttons == 0);
    // The BUILD stamp (a new build time in the rows) repaints the bar's right text alone.
    DriveRows stamped = namedRows();
    stamped.buildTime = "BUILD 23:59:59";
    p = bench::driveRepaint(main, driveView(DrivePage::Main, stamped, false), true);
    CHECK(p.barRight);
    CHECK_FALSE(p.header);
    CHECK(p.lines == 0);
    // A view with fewer rows than the one painted (never on one page today): the missing rows are
    // marked, so the program erases them.
    DriveView fewer = main;
    fewer.lineCount = 1;
    p = bench::driveRepaint(main, fewer, true);
    CHECK(p.lines == 0b10);
}

// ═══ THE PROGRAM'S GLUE, PURE (the T4-3 review, 2026-10-08) ════════════════════════════════════
// drive_program.cpp is compiled by no host test, and ten edits to its row building and boot lines
// survived the whole suite (the review's RS1-RS10). These cases pin the decisions where they now
// live; test/drive_program_source_test.cpp pins that the program calls them.

TEST_CASE("P3-T2 (T4-3): driveSourceRows -- the typed table and a DEGRADED record amber, the "
          "verified record green; the table wins") {
    // BUG CAUGHT: the typed table's rows painted GREEN (a belief shown as a measurement; RS9), a
    // DEGRADED record's reason left off, or a verified record shown amber.
    DriveRows rows{};
    bench::driveSourceRows(rows, true, false, "ignored");
    const bench::SourceRows typed = bench::sourceRows(bench::kDriveTypedSource);
    CHECK(std::string_view{rows.source1.text} == typed.row[0]);
    CHECK(std::string_view{rows.source2.text} == typed.row[1]);
    CHECK(rows.source1.tone == Tone::Amber);
    CHECK(rows.source2.tone == Tone::Amber);
    bench::driveSourceRows(rows, true, true, "DEGRADED: dead 13");  // a table boot: the table's words
    CHECK(std::string_view{rows.source1.text} == typed.row[0]);
    CHECK(rows.source1.tone == Tone::Amber);

    bench::driveSourceRows(rows, false, true, "DEGRADED: dead 13");
    CHECK(std::string_view{rows.source1.text} == "MEASURED record | DEGRADED: dead 13");
    CHECK(rows.source1.tone == Tone::Amber);
    CHECK(rows.source2.tone == Tone::Amber);

    bench::driveSourceRows(rows, false, false, "ignored");
    CHECK(std::string_view{rows.source1.text} == bench::kMeasuredOkRows.row[0]);
    CHECK(std::string_view{rows.source2.text} == bench::kMeasuredOkRows.row[1]);
    CHECK(rows.source1.tone == Tone::Green);
    CHECK(rows.source2.tone == Tone::Green);

    bench::driveSourceRows(rows, false, true, nullptr);  // never crashes
    CHECK(std::string_view{rows.source1.text} == "MEASURED record |");
}

namespace {

/// Robot two's two drive sides as fake groups: LEFT 11-14 at +3 V, RIGHT 20-17 at -4 V, so a row
/// that reads the wrong side says the other's volts and ports.
struct TwoSides {
    std::array<shulib::hal::fake::FakeMotor, 4> lm{};
    std::array<shulib::hal::fake::FakeMotor, 4> rm{};
    std::array<shulib::hal::IMotor*, 4> ls{&lm[0], &lm[1], &lm[2], &lm[3]};
    std::array<shulib::hal::IMotor*, 4> rs{&rm[0], &rm[1], &rm[2], &rm[3]};
    int llabels[4] = {11, 12, 13, 14};
    int rlabels[4] = {20, 19, 18, 17};
    shulib::hal::MotorGroup left{std::span<shulib::hal::IMotor* const>{ls}, {}, llabels};
    shulib::hal::MotorGroup right{std::span<shulib::hal::IMotor* const>{rs}, {}, rlabels};
    TwoSides() {
        left.setVoltage(shulib::units::Voltage{3.0});
        right.setVoltage(shulib::units::Voltage{-4.0});
    }
};

bool sameRow(const MenuLine& a, const MenuLine& b) {
    return a.tone == b.tone && std::string_view{a.text} == std::string_view{b.text};
}

}  // namespace

TEST_CASE("P3-T2 (T4-3): driveLiveRows -- every live row from the facts, the protections' state and "
          "the two sides, each in the colour its meaning needs") {
    // BUG CAUGHT (the review's RS1, RS2, RS5, RS6): the PROTECTION row's CUT painted white (its red
    // is the brief's D4 and worksheet F.1's row 1); the claw row not amber for a FREED claim or an
    // ABSENT claw; MORE's L row reading the RIGHT group; the fault row always green; any row built
    // from the wrong fact.
    using Q = shulib::localization::Localizer::Quality;
    TwoSides sides;
    DriveProtectionState cut{};
    cut.mode = ProtectionMode::Cut;
    cut.cutCount = 1;
    std::snprintf(cut.lastCut, sizeof cut.lastCut, "%s", "CUT: FIGHT on LEFT -- port(s) 18");
    cut.groupCount = 2;
    cut.peakAmps[0] = 2.1;
    cut.peakAmps[1] = 1.4;
    bench::DriveLiveFacts f{};
    f.cutSeconds = 1.0;
    f.connected = true;
    f.cmdVx = 12.0;
    f.cmdOmega = -0.5;
    f.estX = 10.0;
    f.estY = -5.0;
    f.estHeadingDeg = 90.0;
    f.quality = Q::Degraded;
    f.hasOdometry = true;
    f.headingDisagreement = 0.002;
    f.batteryVolts = 12.4;
    f.travelLeft = 0.5;
    f.travelRight = -0.25;
    f.faultCount = 3;
    f.firstFault = shulib::diag::FaultCode::MotorGroupDisagree;
    f.lastFault = shulib::diag::FaultCode::LoopOverrun;
    f.clawText = "claws: pivot6 OK  pivot7 OK  A open";
    DriveRows rows{};
    bench::driveLiveRows(rows, f, cut, sides.left, sides.right);

    // MAIN: the shared panel's words and tones.
    const bench::PanelRow guard = bench::protectionRowText(cut, 1.0);
    CHECK(std::string_view{rows.protection.text} == guard.text);
    CHECK(rows.protection.tone == Tone::Red);
    CHECK(rows.protection.tone == guard.tone);
    CHECK(sameRow(rows.controller, driveRow(bench::controllerRowText(true).text, Tone::White)));
    // DETAILS: each row the builder makes from ITS fact.
    CHECK(sameRow(rows.cmd, bench::driveCmdRow(12.0, -0.5, true)));
    CHECK(sameRow(rows.est, bench::driveEstRow(10.0, -5.0, 90.0, Q::Degraded)));
    CHECK(rows.est.tone == Tone::Amber);
    CHECK(sameRow(rows.battery, bench::driveBatteryRow(true, 0.002, 12.4)));
    CHECK(std::string_view{rows.fault.text} == "FAULT n=3 last LOOP_OVERRUN");
    CHECK(rows.fault.tone == Tone::Red);
    CHECK(std::string_view{rows.claw.text} == "claws: pivot6 OK  pivot7 OK  A open");
    CHECK(rows.claw.tone == Tone::White);
    // MORE: LEFT's volts and ports on the L row, RIGHT's on the R row.
    CHECK(std::string_view{rows.left.text}.starts_with("L  +3.0 V  4/4 present"));
    CHECK(std::string_view{rows.right.text}.starts_with("R  -4.0 V  4/4 present"));
    CHECK(sameRow(rows.travel, bench::driveTravelRow(true, 0.5, -0.25)));
    CHECK(sameRow(rows.protections, bench::driveProtectionCountsRow(cut)));
    CHECK(sameRow(rows.lastCut, bench::driveLastCutRow(cut)));
    CHECK(sameRow(rows.peaks, bench::drivePeaksRow(cut)));

    // The fault row green with nothing latched; the claw row AMBER for an absent claw and for a
    // freed claim, each alone; the controller row red when it is gone; the cut length the config's.
    f.faultCount = 0;
    f.clawAbsent = true;
    f.connected = false;
    f.cutSeconds = 0.5;
    bench::driveLiveRows(rows, f, cut, sides.left, sides.right);
    CHECK(std::string_view{rows.fault.text} == "faults: none");
    CHECK(rows.fault.tone == Tone::Green);
    CHECK(rows.claw.tone == Tone::Amber);
    CHECK(rows.controller.tone == Tone::Red);
    CHECK(std::string_view{rows.protection.text} == bench::protectionRowText(cut, 0.5).text);
    CHECK(sameRow(rows.cmd, bench::driveCmdRow(12.0, -0.5, false)));
    f.clawAbsent = false;
    f.clawFreed = true;
    bench::driveLiveRows(rows, f, cut, sides.left, sides.right);
    CHECK(rows.claw.tone == Tone::Amber);
    // Without drive encoders: the battery row has no cross-check, and the travel row is empty.
    f.hasOdometry = false;
    bench::driveLiveRows(rows, f, cut, sides.left, sides.right);
    CHECK(sameRow(rows.battery, bench::driveBatteryRow(false, 0.0, 12.4)));
    CHECK(std::string_view{rows.travel.text}.empty());
    // A drive merely degraded: the PROTECTION row's amber, not red.
    DriveProtectionState degraded{};
    degraded.mode = ProtectionMode::Degraded;
    std::snprintf(degraded.dead, sizeof degraded.dead, "%s", "18");
    bench::driveLiveRows(rows, f, degraded, sides.left, sides.right);
    CHECK(rows.protection.tone == Tone::Amber);
}

TEST_CASE("P3-T2 (T4-3): driveLineToPaint -- a row that WENT is painted empty at its old place; a "
          "row still there is the new one") {
    // BUG CAUGHT (the review's RS3): a partial repaint re-printing a row that went away instead of
    // erasing it (MAIN's rows left standing under DETAILS' shorter list, or the reverse).
    DriveRows rows = namedRows();
    const DriveView details = driveView(DrivePage::Details, rows, true);
    DriveView shorter = details;
    shorter.lineCount = static_cast<std::uint8_t>(details.lineCount - 2);
    const MenuLine kept = bench::driveLineToPaint(details, shorter, 0);
    CHECK(std::string_view{kept.text} == std::string_view{shorter.lines[0].text});
    const std::size_t gone = details.lineCount - 1;
    const MenuLine erased = bench::driveLineToPaint(details, shorter, gone);
    CHECK(std::string_view{erased.text}.empty());
    CHECK(erased.x == details.lines[gone].x);
    CHECK(erased.y == details.lines[gone].y);
    CHECK(erased.size == details.lines[gone].size);
    CHECK(std::string_view{details.lines[gone].text} == "row:battery");  // `was` is untouched
    // Past the array: an empty row, never a read out of bounds.
    CHECK(std::string_view{bench::driveLineToPaint(details, shorter, 99).text}.empty());
}

TEST_CASE("P3-T2 (T4-3): the protections boot line is GENERATED from the config the loop runs with "
          "-- the library's defaults word for word, and a changed config changes the words") {
    // BUG CAUGHT (the review's RS4, RS7): a cut length typed in (250 ms where the config says
    // 1000), over-current said OFF while it is ON, or any number not the config's — the line's
    // comment promised "generated, never retyped" and nothing checked the words.
    using shulib::teleop::DriveProtectionConfig;
    using shulib::teleop::SideMonitorConfig;
    char line[360];
    (void)bench::driveProtectionsLine(line, sizeof line, DriveProtectionConfig{}, 0.01,
                                      SideMonitorConfig{}.persistTicks);
    CHECK(std::string_view{line}
          == "drive protections (the library's, in every program): a drive port reading frozen 5 "
             "reads is dropped after 250 ms more; more than 2 dead in total stops the drive for the "
             "run; a fight, or over 2.40 A for 250 ms, cuts both sides for 1000 ms (over-current "
             "cuts ON) -- HA-132/175/176, unmeasured");
    DriveProtectionConfig changed{};
    changed.frozenSuspectReads = 7;
    changed.overCurrentAmps = 2.1;
    changed.overCurrentTicks = 30;
    changed.cutTicks = 50;
    changed.overCurrentCuts = false;
    (void)bench::driveProtectionsLine(line, sizeof line, changed, 0.02, 40);
    CHECK(std::string_view{line}
          == "drive protections (the library's, in every program): a drive port reading frozen 7 "
             "reads is dropped after 800 ms more; more than 2 dead in total stops the drive for the "
             "run; a fight, or over 2.10 A for 600 ms, cuts both sides for 1000 ms (over-current "
             "cuts OFF: warnings only) -- HA-132/175/176, unmeasured");
    changed.cutTicks = 25;
    (void)bench::driveProtectionsLine(line, sizeof line, changed, 0.02, 40);
    CHECK(std::string_view{line}.find("cuts both sides for 500 ms") != std::string_view::npos);
}

TEST_CASE("P3-T2 (T4-3): the DEGRADED boot line and the decisions' levels") {
    // BUG CAUGHT (the review's RS8, RS10): one dead drive port at boot getting no DEGRADED line, a
    // line said with no port dead, or a DEGRADED port check (or a typed-table boot) logged as INFO.
    char line[240] = "untouched";
    CHECK_FALSE(bench::driveDegradedBootLine(line, sizeof line, 0, "DEGRADED: dead 13"));
    CHECK(std::string_view{line} == "untouched");
    REQUIRE(bench::driveDegradedBootLine(line, sizeof line, 1, "DEGRADED: dead 13"));
    CHECK(std::string_view{line}
          == "DEGRADED: 1 recorded drive port(s) are EMPTY and were NOT constructed; their group "
             "slots are holes and every read is over the PRESENT members. DEGRADED: dead 13");
    REQUIRE(bench::driveDegradedBootLine(line, sizeof line, 2, nullptr));
    CHECK(std::string_view{line}.starts_with("DEGRADED: 2 recorded drive port(s) are EMPTY"));
    CHECK(bench::driveBootLevel(true) == shulib::hal::LogLevel::Info);
    CHECK(bench::driveBootLevel(false) == shulib::hal::LogLevel::Warn);
}
