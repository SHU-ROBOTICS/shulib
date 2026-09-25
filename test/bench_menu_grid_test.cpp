// Adversarial tests for Bench Tests' menu grid (chunk R2a, 2026-09-24): the 2 × 5 menu became
// 3 × 4 so twelve stations fit, and buttonBox()/hitTest() were generalized from a hard-coded two
// columns to N. The geometry lives in src/bench_r3a.hpp, PROS-free on purpose, so this test can
// include it by relative path (the chassis_table.hpp precedent) and try to break it.
//
// What a broken grid costs on the robot: a tap that launches the WRONG station (10 DRIVE powers
// motors), a tap that launches NOTHING (robot two's first recorded tap, 2026-09-10, released one
// pixel into a gap and did nothing), or a stale / error coordinate that launches something with
// nobody touching the screen. Every case below names the bug it catches. The drawing itself
// (colours, fonts, what the panel actually shows) cannot be tested here — nothing in this file
// has run on a V5.

#include "doctest.h"

#include <algorithm>
#include <initializer_list>

#include "../src/bench_r3a.hpp"
#include "shulib/calibration/calibrate_panel.hpp"

using shulib::bench::menu_grid::buttonBox;
using shulib::bench::menu_grid::colsUsed;
using shulib::bench::menu_grid::extent;
using shulib::bench::menu_grid::hitTest;
using shulib::bench::menu_grid::kDotClear;
using shulib::bench::menu_grid::kDotDX;
using shulib::bench::menu_grid::kDotDY;
using shulib::bench::menu_grid::kDotR;
using shulib::bench::menu_grid::kLabelInsetX;
using shulib::bench::menu_grid::kLastRow;
using shulib::bench::menu_grid::kLine1DY;
using shulib::bench::menu_grid::kLine2DY;
using shulib::bench::menu_grid::kMenu;
using shulib::bench::menu_grid::kMenuHeaderBottom;
using shulib::bench::menu_grid::kPanelW;
using shulib::bench::menu_grid::kStationRow;
using shulib::bench::menu_grid::kStationRowHit;
using shulib::bench::menu_grid::kStripeW;
using shulib::bench::menu_grid::line1Px;
using shulib::bench::menu_grid::line2Px;
using shulib::bench::menu_grid::Rect;
using shulib::bench::menu_grid::rowsOf;
using shulib::bench::menu_grid::Spec;

namespace {

[[nodiscard]] bool overlap(const Rect& a, const Rect& b) {
    return a.x0 <= b.x1 && b.x0 <= a.x1 && a.y0 <= b.y1 && b.y0 <= a.y1;
}

/// THE ORACLE: the 2-column hit test exactly as it shipped before R2a (bench_r3a.cpp at 10de29f,
/// `hitTest` and `buttonBox` with kBtnW 232, kBtnH 36, kBtnX0 6, kBtnY0 38, kGap 4, ten entries),
/// re-typed here verbatim so the generalized helper can be compared with it pixel for pixel.
[[nodiscard]] int oldHitTest(int tx, int ty) {
    constexpr int kBtnW = 232, kBtnH = 36, kBtnX0 = 6, kBtnY0 = 38, kGap = 4, kMenuCount = 10;
    auto box = [&](int i, int& x0, int& y0, int& x1, int& y1) {
        const int col = i % 2;
        const int row = i / 2;
        x0 = kBtnX0 + col * (kBtnW + kGap);
        y0 = kBtnY0 + row * (kBtnH + kGap);
        x1 = x0 + kBtnW;
        y1 = y0 + kBtnH;
    };
    int x0 = 0, y0 = 0, x1 = 0, y1 = 0;
    box(0, x0, y0, x1, y1);
    const int gridX0 = x0, gridY0 = y0;
    box(kMenuCount - 1, x0, y0, x1, y1);
    const int gridX1 = x1, gridY1 = y1;
    if (tx < gridX0 || tx > gridX1 || ty < gridY0 || ty > gridY1) return -1;
    const int col = std::min(1, (tx - gridX0) / (kBtnW + kGap));
    const int row = std::min(kMenuCount / 2 - 1 + kMenuCount % 2, (ty - gridY0) / (kBtnH + kGap));
    const int i = row * 2 + col;
    return i < kMenuCount ? i : -1;
}

}  // namespace

TEST_CASE("menu grid: twelve buttons, each TALLER than the old 36 px, all inside the usable panel "
          "and below the header, none overlapping") {
    // BUG CAUGHT: a layout that shrinks the touch targets (the brief's floor is today's 36 px),
    // draws a row off the bottom of the 240-row panel (the 2026-08-18 bug: a layout checked
    // against 272 rendered its last row off the screen), or lets two buttons share a pixel.
    CHECK(kMenu.count == 12);
    CHECK(rowsOf(kMenu) == 4);
    CHECK(colsUsed(kMenu) == 3);
    for (int i = 0; i < kMenu.count; ++i) {
        CAPTURE(i);
        const Rect r = buttonBox(kMenu, i);
        CHECK(r.y1 - r.y0 >= 36);          // not smaller than the 2 x 5 menu's buttons
        CHECK(r.y1 - r.y0 == 46);          // what R2a chose (R2a-PROGRESS R2a-R1)
        CHECK(r.x0 >= 0);
        CHECK(r.x1 <= kPanelW - 1);
        CHECK(r.y0 > kMenuHeaderBottom);   // the header bar (0..32) stays dead
        CHECK(r.y1 <= kLastRow);           // every pixel at y < 236
        for (int j = i + 1; j < kMenu.count; ++j) {
            CAPTURE(j);
            CHECK_FALSE(overlap(r, buttonBox(kMenu, j)));
        }
    }
    // The exact rectangles the log's table records (R2a-R1), so a silent geometry edit shows.
    const Rect first = buttonBox(kMenu, 0);
    const Rect last = buttonBox(kMenu, 11);
    CHECK(first.x0 == 6);
    CHECK(first.y0 == 38);
    CHECK(first.x1 == 159);
    CHECK(first.y1 == 84);
    CHECK(last.x0 == 320);
    CHECK(last.y0 == 188);
    CHECK(last.x1 == 473);
    CHECK(last.y1 == 234);
    // Symmetric margins: six columns clear on the left (0..5) and on the right (474..479).
    CHECK(first.x0 == kPanelW - 1 - last.x1);
}

TEST_CASE("menu grid: every pixel of a button hits THAT button, and every pixel of the grid hits "
          "SOME button (no dead gap)") {
    // BUG CAUGHT: a hit test that disagrees with the drawing (a tap on button 11's face runs
    // station 10, which powers motors), or a gap that swallows a tap on a resistive screen.
    for (int i = 0; i < kMenu.count; ++i) {
        const Rect r = buttonBox(kMenu, i);
        for (int y = r.y0; y <= r.y1; ++y) {
            for (int x = r.x0; x <= r.x1; ++x) {
                if (hitTest(kMenu, x, y) != i) {
                    FAIL_CHECK("button " << i << " pixel (" << x << "," << y << ") hit "
                                         << hitTest(kMenu, x, y));
                }
            }
        }
    }
    const Rect e = extent(kMenu);
    int dead = 0;
    for (int y = e.y0; y <= e.y1; ++y) {
        for (int x = e.x0; x <= e.x1; ++x) {
            if (hitTest(kMenu, x, y) < 0) ++dead;
        }
    }
    CHECK(dead == 0);
}

TEST_CASE("menu grid: a gap pixel belongs to the button ABOVE it or to its LEFT") {
    // BUG CAUGHT: the gap rule flipped (a release between 10 DRIVE and 11 DISTANCE running
    // DISTANCE is harmless; one between 9 and 10 running DRIVE is not what the old rule did,
    // and the worksheet's muscle memory was built on the old rule).
    const Rect b0 = buttonBox(kMenu, 0);   // row 0, col 0
    const Rect b1 = buttonBox(kMenu, 1);   // row 0, col 1
    const Rect b3 = buttonBox(kMenu, 3);   // row 1, col 0
    REQUIRE(b1.x0 - b0.x1 == kMenu.gap);   // gap - 1 pixels strictly between
    for (int x = b0.x1 + 1; x < b1.x0; ++x) {
        CAPTURE(x);
        CHECK(hitTest(kMenu, x, b0.y0 + 10) == 0);   // horizontal gap -> the LEFT button
    }
    for (int y = b0.y1 + 1; y < b3.y0; ++y) {
        CAPTURE(y);
        CHECK(hitTest(kMenu, b0.x0 + 10, y) == 0);   // vertical gap -> the button ABOVE
    }
    // The corner where four buttons meet goes to the upper-left one.
    CHECK(hitTest(kMenu, b0.x1 + 1, b0.y1 + 1) == 0);
    // The gap just above 10 DRIVE (index 9: bottom row, left) belongs to 7 RUN ALL (index 6),
    // the button above it -- never to DRIVE, the one station that powers motors.
    const Rect b9 = buttonBox(kMenu, 9);
    CHECK(hitTest(kMenu, b9.x0 + 10, b9.y0 - 1) == 6);
    CHECK(hitTest(kMenu, b9.x0 + 10, b9.y0) == 9);
}

TEST_CASE("menu grid: the header strip, the margins and the strip below hit NOTHING -- including "
          "(0,0) and an error read's (-1,-1)") {
    // BUG CAUGHT: a stale coordinate launching a station with nobody touching the screen. The
    // at-rest touch status carries the LAST touch's x/y, and an E_TOUCH_ERROR read carries -1 in
    // every field (vendored screen.h:726-729); the earlier touch fix's invariant is that neither
    // can hit a button.
    const Rect e = extent(kMenu);
    CHECK(hitTest(kMenu, 0, 0) == -1);
    CHECK(hitTest(kMenu, -1, -1) == -1);
    CHECK(hitTest(kMenu, -32768, -32768) == -1);
    for (int x = -1; x <= kPanelW; ++x) {
        for (int y = -1; y < e.y0; ++y) {            // the header bar and the strip under it
            if (hitTest(kMenu, x, y) != -1) FAIL_CHECK("header (" << x << "," << y << ") hit");
        }
        for (int y = e.y1 + 1; y <= 240; ++y) {      // below the grid
            if (hitTest(kMenu, x, y) != -1) FAIL_CHECK("bottom (" << x << "," << y << ") hit");
        }
    }
    for (int y = -1; y <= 240; ++y) {
        for (int x = -1; x < e.x0; ++x) {            // left margin
            if (hitTest(kMenu, x, y) != -1) FAIL_CHECK("left (" << x << "," << y << ") hit");
        }
        for (int x = e.x1 + 1; x <= kPanelW; ++x) {  // right margin
            if (hitTest(kMenu, x, y) != -1) FAIL_CHECK("right (" << x << "," << y << ") hit");
        }
    }
}

TEST_CASE("menu grid: ROW-MAJOR, so the menu reads 1 2 3 / 4 5 6 / 7 8 9 / 10 11 12") {
    // BUG CAUGHT: a column-major layout would put station 2 under station 1 and move 10 DRIVE
    // into the top row -- the numbers would survive and every position would change.
    for (int i = 0; i < kMenu.count; ++i) {
        CAPTURE(i);
        const Rect r = buttonBox(kMenu, i);
        const Rect first = buttonBox(kMenu, (i / 3) * 3);
        CHECK(r.y0 == first.y0);                     // same row as its row's first button
        if (i % 3 != 0) CHECK(r.x0 > buttonBox(kMenu, i - 1).x1);
        if (i >= 3) CHECK(r.y0 > buttonBox(kMenu, i - 3).y1);
        // The centre of the button's face hits it.
        CHECK(hitTest(kMenu, (r.x0 + r.x1) / 2, (r.y0 + r.y1) / 2) == i);
    }
    // Station 10 DRIVE (index 9) opens the bottom row, on the left; 12 (index 11) ends it.
    CHECK(buttonBox(kMenu, 9).x0 == buttonBox(kMenu, 0).x0);
    CHECK(buttonBox(kMenu, 11).y0 == buttonBox(kMenu, 9).y0);
}

TEST_CASE("menu grid: the generalized helper reproduces the old 2 x 5 menu PIXEL FOR PIXEL") {
    // BUG CAUGHT: the generalization changing two-column behaviour. The shipped 2 x 5 menu ran
    // on both robots for a month; with its own numbers, the new helper must agree with it at
    // every point on and around the panel.
    constexpr Spec kOld{10, 2, 6, 38, 232, 36, 4};
    int disagreements = 0;
    for (int y = -2; y <= 241; ++y) {
        for (int x = -2; x <= 481; ++x) {
            if (hitTest(kOld, x, y) != oldHitTest(x, y)) ++disagreements;
        }
    }
    CHECK(disagreements == 0);
    // And its rectangles are the old ones.
    CHECK(buttonBox(kOld, 9).x0 == 242);
    CHECK(buttonBox(kOld, 9).x1 == 474);
    CHECK(buttonBox(kOld, 9).y1 == 234);
}

TEST_CASE("menu grid: a PARTIAL last row keeps the right column live and makes the empty slot "
          "dead") {
    // BUG CAUGHT: the old idiom took the grid's right edge from the LAST button. With eleven
    // buttons in three columns the last button is in the MIDDLE column, so that idiom would make
    // the whole right column (stations 3, 6, 9) dead. And the empty slot must not hit button 11,
    // which does not exist.
    constexpr Spec kEleven{11, 3, 6, 38, 153, 46, 4};
    const Rect e = extent(kEleven);
    CHECK(e.x1 == buttonBox(kEleven, 2).x1);                 // the rightmost column, not index 10
    CHECK(e.x1 > buttonBox(kEleven, 10).x1);
    for (int i : {2, 5, 8}) {
        CAPTURE(i);
        const Rect r = buttonBox(kEleven, i);
        CHECK(hitTest(kEleven, r.x1 - 1, r.y0 + 1) == i);    // the right column is alive
    }
    const Rect ghost = buttonBox(kEleven, 11);                // where a 12th would be
    for (int y = ghost.y0; y <= ghost.y1; ++y) {
        for (int x = ghost.x0; x <= ghost.x1; ++x) {
            if (hitTest(kEleven, x, y) != -1) FAIL_CHECK("empty slot (" << x << "," << y << ")");
        }
    }
    // A single, partial row: the extent is only as wide as the buttons that exist.
    constexpr Spec kTwoOfThree{2, 3, 6, 186, 153, 48, 4};
    CHECK(colsUsed(kTwoOfThree) == 2);
    CHECK(extent(kTwoOfThree).x1 == buttonBox(kTwoOfThree, 1).x1);
    CHECK(hitTest(kTwoOfThree, 400, 200) == -1);
    // Degenerate specs hit nothing and do not divide by zero.
    CHECK(hitTest(Spec{0, 3, 6, 38, 153, 46, 4}, 50, 50) == -1);
    CHECK(hitTest(Spec{5, 0, 6, 38, 153, 46, 4}, 50, 50) == -1);
}

TEST_CASE("menu grid: inside a button, the stripe, both label lines and the status dot fit and "
          "do not touch") {
    // BUG CAUGHT: a label drawn over the hands-on stripe, over the status dot, or across the
    // button's edge -- the old menu put the label at x0+10 and the dot at x1-16 on the SAME
    // line, and the longest old label ran into the dot at the proven MEDIUM width.
    using shulib::calibration::kMediumCols;
    using shulib::calibration::kMediumHeight;
    const Rect r = buttonBox(kMenu, 0);
    const int stripeLast = r.x0 + kStripeW - 1;
    const int textX = r.x0 + kLabelInsetX;
    CHECK(stripeLast < textX - 1);                             // at least one clear column
    const int line1Last = textX + line1Px(kMenu) - 1;
    CHECK(line1Last == r.x1 - 1);                              // up to, not onto, the edge
    const int dotLeft = r.x1 - kDotDX - kDotR;
    const int dotRight = r.x1 - kDotDX + kDotR;
    const int dotTop = r.y0 + kDotDY - kDotR;
    const int dotBottom = r.y0 + kDotDY + kDotR;
    const int line2Last = textX + line2Px(kMenu) - 1;
    CHECK(line2Last == dotLeft - kDotClear);
    CHECK(line2Last < dotLeft - 1);                            // clear of the dot
    CHECK(dotRight < r.x1);                                    // inside the edge line
    CHECK(dotTop > r.y0);
    CHECK(dotBottom < r.y1);
    // Line 2 sits one proven MEDIUM pitch (22 px) under line 1, and its text cell ends above the
    // bottom edge at calibrate's MEDIUM height estimate (A4: HA-136).
    CHECK(kLine2DY - kLine1DY == 22);
    CHECK(r.y0 + kLine2DY + kMediumHeight <= r.y1);
    CHECK(kLine1DY > 0);                                       // below the top edge line
    // The dot sits beside line 2, within its text cell's height.
    CHECK(dotTop >= r.y0 + kLine2DY);
    CHECK(dotBottom <= r.y0 + kLine2DY + kMediumHeight);
    // The character budgets these pixels give at the proven MEDIUM upper bound of 472 px / 36
    // characters (R3d-PROGRESS 15:43-15:50): 11 on line 1, 9 on line 2. bench_r3a.cpp
    // static_asserts every label against the same arithmetic.
    CHECK(line1Px(kMenu) * static_cast<int>(kMediumCols) / 472 == 11);
    CHECK(line2Px(kMenu) * static_cast<int>(kMediumCols) / 472 == 9);
}

TEST_CASE("the stations' button row: inside the panel, in the menu's columns, and its hit area "
          "contains every drawn button and adds only a strip ABOVE") {
    // BUG CAUGHT: a RECORD / DONE whose drawn face does not hit (the calibrate complaint "THE
    // BUTTONS DONT WORK"), a hit area that swallows the object rows above it, or a row drawn
    // off the panel. The row shares the menu's columns ON PURPOSE (RECORD and DONE sit in the
    // same place on stations 11 and 12) -- and that is why the stations arm their buttons after
    // 0.5 s: menu buttons 11 and 12 sit exactly where RECORD and DONE are drawn (R2a-R10).
    CHECK(kStationRow.count == 3);
    CHECK(kStationRowHit.count == 3);
    CHECK(rowsOf(kStationRow) == 1);
    for (int i = 0; i < 3; ++i) {
        CAPTURE(i);
        const Rect d = buttonBox(kStationRow, i);
        const Rect h = buttonBox(kStationRowHit, i);
        CHECK(d.y1 <= kLastRow);
        CHECK(d.x1 <= kPanelW - 1);
        CHECK(d.y1 - d.y0 >= 36);
        CHECK(d.x0 == buttonBox(kMenu, i).x0);                 // the menu's columns
        CHECK(d.x1 == buttonBox(kMenu, i).x1);
        CHECK(h.x0 == d.x0);                                   // no sideways growth
        CHECK(h.x1 == d.x1);
        CHECK(h.y1 == d.y1);                                   // no growth downward
        CHECK(h.y0 == d.y0 - 8);                               // 8 px above, and only that
        for (int y = d.y0; y <= d.y1; ++y) {
            for (int x = d.x0; x <= d.x1; ++x) {
                if (hitTest(kStationRowHit, x, y) != i) FAIL_CHECK("slot " << i << " (" << x << ","
                                                                            << y << ")");
            }
        }
    }
    // Menu buttons 11 and 12 (indices 10, 11) overlap RECORD (slot 1) and DONE (slot 2).
    CHECK(overlap(buttonBox(kMenu, 10), buttonBox(kStationRow, 1)));
    CHECK(overlap(buttonBox(kMenu, 11), buttonBox(kStationRow, 2)));
    // Anything above the hit strip is not a button: the object rows and the status line live there.
    for (int x = 0; x < kPanelW; ++x) {
        if (hitTest(kStationRowHit, x, kStationRowHit.y0 - 1) != -1) FAIL_CHECK("x " << x);
    }
}
