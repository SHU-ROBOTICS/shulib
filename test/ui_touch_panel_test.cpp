// ui::touch_panel — the generic panel pieces (chunk COMP-a; the COMP brief's ruling C4 and
// amendment B20). Every case names the bug it would catch.
//
// ── B20: ONE set of budgets ─────────────────────────────────────────────────────────
// calibration/calibrate_panel.hpp keeps its own copy of these constants until it is moved onto
// ui/ (a later, separate change — ruling C4). Until then the two copies are held EQUAL here, at
// COMPILE time: edit one after the SCREEN RULER is read and not the other, and the host build
// fails naming the constant. (Why here and not in a header: ui/touch_panel.hpp's banner.)

#include "doctest.h"

#include <cstring>
#include <string_view>

#include "shulib/calibration/calibrate_panel.hpp"
#include "shulib/ui/touch_panel.hpp"

namespace ui = shulib::ui;
namespace cal = shulib::calibration;

static_assert(ui::kLargeCols == cal::kLargeCols, "B20: ui and calibration LARGE budgets differ");
static_assert(ui::kMediumCols == cal::kMediumCols, "B20: ui and calibration MEDIUM budgets differ");
static_assert(ui::kSmallCols == cal::kSmallCols, "B20: ui and calibration SMALL budgets differ");
static_assert(ui::kSmallHeight == cal::kSmallHeight, "B20: SMALL line heights differ");
static_assert(ui::kMediumHeight == cal::kMediumHeight, "B20: MEDIUM line heights differ");
static_assert(ui::kLargeHeight == cal::kLargeHeight, "B20: LARGE line heights differ");
static_assert(ui::kPanelWidth == cal::kPanelWidth, "B20: panel widths differ");
static_assert(ui::kPanelLastRow == cal::kPanelLastRow, "B20: last drawable rows differ");
static_assert(ui::kHitPadX == cal::kHitPadX, "B20: horizontal hit pads differ");
static_assert(ui::kHitPadY == cal::kHitPadY, "B20: vertical hit pads differ");
static_assert(ui::kArmingDelaySeconds == cal::kArmingDelaySeconds, "B20: arming delays differ");

// Bug caught: the generic rectangle helpers drifting from the calibrate panel's, which a program
// painting both screens relies on meaning the same thing.
TEST_CASE("B20 the rectangle helpers agree with the calibrate panel's, case by case") {
    const ui::Rect a{10, 20, 30, 40};
    const cal::PanelRect ca{10, 20, 30, 40};
    const ui::Rect h = ui::hitBox(a);
    const cal::PanelRect ch = cal::hitBox(ca);
    CHECK(h.x0 == ch.x0);
    CHECK(h.y0 == ch.y0);
    CHECK(h.x1 == ch.x1);
    CHECK(h.y1 == ch.y1);
    for (int x = 0; x < 50; x += 3) {
        for (int y = 0; y < 60; y += 3) {
            CHECK(ui::contains(a, x, y) == cal::contains(ca, x, y));
        }
    }
    const ui::Rect b{30, 40, 50, 60};
    CHECK(ui::overlaps(a, b) == cal::overlaps(ca, cal::PanelRect{30, 40, 50, 60}));
    CHECK(ui::overlaps(a, b));  // the shared corner pixel counts (inclusive corners)
    CHECK_FALSE(ui::overlaps(a, ui::Rect{31, 41, 50, 60}));
    CHECK(ui::within(ui::Rect{11, 21, 29, 39}, a));
    CHECK_FALSE(ui::within(ui::Rect{9, 21, 29, 39}, a));
    for (auto s : {ui::TextSize::Small, ui::TextSize::Medium, ui::TextSize::Large}) {
        const auto cs = static_cast<cal::TextSize>(static_cast<std::uint8_t>(s));
        CHECK(ui::budgetFor(s) == cal::budgetFor(cs));
        CHECK(ui::heightFor(s) == cal::heightFor(cs));
    }
}

// Bug caught: a screen test that lets a byte through that VEXos would mangle — the '%' PROS
// re-reads as a format, a control character, a UTF-8 byte the ASCII font cannot draw — or that
// refuses a line exactly at its budget.
TEST_CASE("C4 screenText and copyScreenText: ASCII only, no percent, bounded") {
    CHECK(ui::screenText("AUTON: left rush", ui::kMediumCols));
    CHECK(ui::screenText(std::string_view{"123456789012345678"}, ui::kLargeCols));
    CHECK_FALSE(ui::screenText(std::string_view{"1234567890123456789"}, ui::kLargeCols));
    CHECK_FALSE(ui::screenText("100%", 50));
    CHECK_FALSE(ui::screenText("a\tb", 50));
    CHECK_FALSE(ui::screenText("caf\xc3\xa9", 50));
    CHECK_FALSE(ui::screenText(std::string_view{"a\x7f", 2}, 50));

    char out[8];
    CHECK(ui::copyScreenText(out, sizeof out, "50% caf\xc3\xa9", 50) == 7);  // cap-bounded
    CHECK(std::strcmp(out, "50? caf") == 0);
    CHECK(ui::copyScreenText(out, sizeof out, "abcdef", 3) == 3);  // budget-bounded
    CHECK(std::strcmp(out, "abc") == 0);
    CHECK(ui::copyScreenText(out, 1, "abc", 3) == 0);
    CHECK(out[0] == '\0');
    CHECK(ui::copyScreenText(nullptr, 4, "abc", 3) == 0);
}

// Bug caught: the release edge replaying a touch — an error read that moves the count (the next
// good read would then look new), or a repeated count acted on twice.
TEST_CASE("C4 ReleaseEdge: new releases only; errors never move the count") {
    ui::ReleaseEdge edge{10};
    CHECK_FALSE(edge.take({.error = false, .releaseCount = 10, .x = 1, .y = 1}));
    CHECK_FALSE(edge.take({.error = true, .releaseCount = 11, .x = -1, .y = -1}));
    CHECK(edge.take({.error = false, .releaseCount = 11, .x = 5, .y = 5}));
    CHECK_FALSE(edge.take({.error = false, .releaseCount = 11, .x = 5, .y = 5}));
    CHECK(edge.take({.error = false, .releaseCount = 13, .x = 5, .y = 5}));  // two lifts, one read
}

// Bug caught: an arming clock that restarts on every frame (buttons never arm) or never restarts
// (a new layout is live on its first frame).
TEST_CASE("C4 ArmingClock and LayoutKey: restart on a new layout only") {
    ui::LayoutKey k1;
    k1.mix(ui::Rect{0, 0, 10, 10});
    ui::LayoutKey k2;
    k2.mix(ui::Rect{0, 0, 10, 11});
    CHECK(k1.value() != k2.value());
    ui::ArmingClock clk;
    CHECK(clk.observe(k1.value(), 5.0) == 0.0);
    CHECK(clk.observe(k1.value(), 5.3) == doctest::Approx(0.3));
    CHECK_FALSE(ui::ArmingClock::armedAfter(0.3));
    CHECK(clk.observe(k1.value(), 5.6) == doctest::Approx(0.6));
    CHECK(ui::ArmingClock::armedAfter(0.6));
    CHECK(clk.observe(k2.value(), 5.7) == 0.0);  // new layout: back to greyed
}

// Bug caught: a hit test that ignores the armed flag, or picks a button the padded box does not
// reach.
TEST_CASE("C4 hitTest: armed buttons only, padded boxes") {
    struct B {
        ui::Rect rect;
        bool armed;
    };
    const B buttons[2] = {{ui::Rect{10, 10, 50, 30}, true}, {ui::Rect{100, 10, 150, 30}, false}};
    CHECK(ui::hitTest(buttons, 2, 30, 20) == 0);
    CHECK(ui::hitTest(buttons, 2, 30, 10 - ui::kHitPadY) == 0);      // the pad reaches
    CHECK(ui::hitTest(buttons, 2, 30, 10 - ui::kHitPadY - 1) == -1);  // one pixel past it
    CHECK(ui::hitTest(buttons, 2, 120, 20) == -1);                    // greyed
}
