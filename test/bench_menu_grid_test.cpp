// Adversarial tests for Bench Tests' menu grid (chunk R2a, 2026-09-24): the 2 × 5 menu became
// 3 × 4 so twelve stations fit, and buttonBox()/hitTest() were generalized from a hard-coded two
// columns to N. MECH-A6 (2026-10-30, the MECH brief's N5 (a)) made it 3 × 5 — five rows of 36 px, room
// for 15 — for station 13 MECHANISMS: stations 1-12 keep their numbers and their places (row, column),
// the buttons are the 2 × 5 menu's height again, and each button's second line is SMALL. The geometry
// lives in src/programs/bench_r3a.hpp, PROS-free on purpose, so this test can include it from src/ (the
// chassis_table.hpp precedent) and try to break it.
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

#include "programs/bench_r3a.hpp"
#include "shulib/calibration/calibrate_panel.hpp"

using shulib::bench::menu_grid::buttonBox;
using shulib::bench::menu_grid::colsUsed;
using shulib::bench::menu_grid::extent;
using shulib::bench::menu_grid::hitTest;
using shulib::bench::menu_grid::kAnswerRow4;
using shulib::bench::menu_grid::kAnswerRow4Hit;
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
using shulib::bench::menu_grid::kMenuSlots;
using shulib::bench::menu_grid::kPanelW;
using shulib::bench::menu_grid::kSmallPitch;
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

/// THE ORACLE for the 3 x 5 menu (MECH-A6): its hit test re-typed from the rows and columns the
/// header's comment states — columns 6-159 / 163-316 / 320-473, rows 38-74 / 78-114 / 118-154 /
/// 158-194 / 198-234, a gap pixel to the button above / to the left, thirteen buttons — by hand, never
/// through buttonBox() or extent(), so the generalized helper is checked against an independent reading.
[[nodiscard]] int newHitTest(int tx, int ty) {
    constexpr int kColX0[3] = {6, 163, 320};
    constexpr int kRowY0[5] = {38, 78, 118, 158, 198};
    if (tx < 6 || tx > 473 || ty < 38 || ty > 234) return -1;
    int col = 2;
    for (int c = 0; c < 2; ++c) {
        if (tx < kColX0[c + 1]) {
            col = c;
            break;
        }
    }
    int row = 4;
    for (int r = 0; r < 4; ++r) {
        if (ty < kRowY0[r + 1]) {
            row = r;
            break;
        }
    }
    const int i = row * 3 + col;
    return i < 13 ? i : -1;
}

/// R2a's 3 x 4 menu, the spec it replaced (bench_r3a.hpp at 81eb666): twelve buttons, 46 px tall.
constexpr Spec kR2aMenu{12, 3, 6, 38, 153, 46, 4};

}  // namespace

TEST_CASE("menu grid: thirteen buttons — room for FIFTEEN — each 36 px tall (the 2 x 5 menu's), all inside "
          "the usable panel and below the header, none overlapping") {
    // BUG CAUGHT: a layout that shrinks the touch targets below the 2 x 5 menu's 36 px, draws a row off
    // the bottom of the 240-row panel (the 2026-08-18 bug: a layout checked against 272 rendered its
    // last row off the screen), lets two buttons share a pixel, or has no room for the two stations
    // the rest's pages bring later (the MECH brief's N5 (a): 15).
    CHECK(kMenu.count == 13);
    CHECK(rowsOf(kMenu) == 5);
    CHECK(colsUsed(kMenu) == 3);
    CHECK(kMenuSlots == 15);
    const Spec full{kMenuSlots, kMenu.cols, kMenu.x0, kMenu.y0, kMenu.w, kMenu.h, kMenu.gap};
    CHECK(rowsOf(full) == 5);  // fifteen fill the same five rows
    for (int i = 0; i < full.count; ++i) {
        CAPTURE(i);
        const Rect r = buttonBox(full, i);
        CHECK(r.y1 - r.y0 >= 36);          // not smaller than the 2 x 5 menu's buttons
        CHECK(r.y1 - r.y0 == 36);          // what MECH-A6 chose (N5 (a): 38 + 5 x 40 - 4 = 234)
        CHECK(r.x0 >= 0);
        CHECK(r.x1 <= kPanelW - 1);
        CHECK(r.y0 > kMenuHeaderBottom);   // the header bar (0..32) stays dead
        CHECK(r.y1 <= kLastRow);           // every pixel at y < 236
        for (int j = i + 1; j < full.count; ++j) {
            CAPTURE(j);
            CHECK_FALSE(overlap(r, buttonBox(full, j)));
        }
    }
    // The exact rectangles, so a silent geometry edit shows.
    const Rect first = buttonBox(kMenu, 0);
    const Rect twelve = buttonBox(kMenu, 11);
    const Rect thirteen = buttonBox(kMenu, 12);
    const Rect lastSlot = buttonBox(full, 14);
    CHECK(first.x0 == 6);
    CHECK(first.y0 == 38);
    CHECK(first.x1 == 159);
    CHECK(first.y1 == 74);
    CHECK(twelve.x0 == 320);
    CHECK(twelve.y0 == 158);
    CHECK(twelve.x1 == 473);
    CHECK(twelve.y1 == 194);
    CHECK(thirteen.x0 == 6);
    CHECK(thirteen.y0 == 198);
    CHECK(thirteen.y1 == 234);
    CHECK(lastSlot.x1 == 473);
    CHECK(lastSlot.y1 == 234);  // the last of fifteen ends where R2a's last row did
    // Symmetric margins: six columns clear on the left (0..5) and on the right (474..479).
    CHECK(first.x0 == kPanelW - 1 - twelve.x1);
}

TEST_CASE("menu grid: stations 1-12 keep their PLACES — the same row and column, the same columns' pixels — "
          "and the helper still draws R2a's 3 x 4 grid pixel for pixel") {
    // BUG CAUGHT: the fifth row reordering the menu (a station whose button moved is a worksheet step whose
    // muscle memory is wrong — and 10 DRIVE powers motors), a column that moved sideways, or the helper's
    // own behaviour changed under the new spec (R2a's 3 x 4 grid is the oracle the generalized helper must
    // still reproduce with its numbers).
    for (int i = 0; i < 12; ++i) {
        CAPTURE(i);
        const Rect was = buttonBox(kR2aMenu, i);
        const Rect now = buttonBox(kMenu, i);
        CHECK(now.x0 == was.x0);  // the same column, pixel for pixel
        CHECK(now.x1 == was.x1);
        // the same row: the row index is the same, so the rows keep their order top to bottom
        CHECK((now.y0 - kMenu.y0) / (kMenu.h + kMenu.gap) == (was.y0 - kR2aMenu.y0) / (kR2aMenu.h + kR2aMenu.gap));
        // and a tap on the centre of the old button's ROW and COLUMN in the new grid hits the same station
        CHECK(hitTest(kMenu, (now.x0 + now.x1) / 2, (now.y0 + now.y1) / 2) == i);
    }
    // R2a's grid through the helper, every pixel on and around the panel, against its stated rows
    // (38-84 / 88-134 / 138-184 / 188-234) re-typed here.
    int disagreements = 0;
    for (int y = -2; y <= 241; ++y) {
        for (int x = -2; x <= 481; ++x) {
            int want = -1;
            if (x >= 6 && x <= 473 && y >= 38 && y <= 234) {
                const int col = x < 163 ? 0 : x < 320 ? 1 : 2;
                const int row = y < 88 ? 0 : y < 138 ? 1 : y < 188 ? 2 : 3;
                want = row * 3 + col;
            }
            if (hitTest(kR2aMenu, x, y) != want) ++disagreements;
        }
    }
    CHECK(disagreements == 0);
    CHECK(buttonBox(kR2aMenu, 11).y1 == 234);
    CHECK(buttonBox(kR2aMenu, 0).y1 == 84);
}

TEST_CASE("menu grid: the 3 x 5 hit test agrees with an independently re-typed oracle at EVERY pixel on and "
          "around the panel") {
    // BUG CAUGHT: any hit-test slip in the new geometry — a dead gap, an off-by-one row edge, the two
    // empty slots of the last row answering 14 or 15, or a margin that launches a station.
    int disagreements = 0;
    for (int y = -2; y <= 241; ++y) {
        for (int x = -2; x <= 481; ++x) {
            if (hitTest(kMenu, x, y) != newHitTest(x, y)) {
                if (++disagreements <= 5) FAIL_CHECK("(" << x << "," << y << "): " << hitTest(kMenu, x, y)
                                                         << " vs " << newHitTest(x, y));
            }
        }
    }
    CHECK(disagreements == 0);
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
    // Inside the extent every pixel hits a button — except the two EMPTY slots of the partial fifth
    // row (and the gap between them), which hit nothing: from column 1's left edge, in row 5's band.
    const Rect e = extent(kMenu);
    const Rect thirteen = buttonBox(kMenu, 12);
    const int emptyX0 = buttonBox(kMenu, 1).x0;
    int dead = 0;
    int wrong = 0;
    for (int y = e.y0; y <= e.y1; ++y) {
        for (int x = e.x0; x <= e.x1; ++x) {
            const bool isDead = hitTest(kMenu, x, y) < 0;
            const bool emptySlot = y >= thirteen.y0 && x >= emptyX0;
            if (isDead != emptySlot) ++wrong;
            if (isDead) ++dead;
        }
    }
    CHECK(wrong == 0);
    CHECK(dead == (e.x1 - emptyX0 + 1) * (e.y1 - thirteen.y0 + 1));
    CHECK(emptyX0 == 163);
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
    // The gap just above 10 DRIVE (index 9: row 4, left) belongs to 7 RUN ALL (index 6),
    // the button above it -- never to DRIVE, which powers motors.
    const Rect b9 = buttonBox(kMenu, 9);
    CHECK(hitTest(kMenu, b9.x0 + 10, b9.y0 - 1) == 6);
    CHECK(hitTest(kMenu, b9.x0 + 10, b9.y0) == 9);
    // The gap just above 13 MECHANISMS (index 12: row 5, left) belongs to 10 DRIVE above it. Both
    // stations power motors; each opens on its own confirm prompt before anything moves (station 10's
    // wheels-up question, station 13's CLEAR), so a gap tap there powers nothing by itself.
    const Rect b12 = buttonBox(kMenu, 12);
    CHECK(hitTest(kMenu, b12.x0 + 10, b12.y0 - 1) == 9);
    CHECK(hitTest(kMenu, b12.x0 + 10, b12.y0) == 12);
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

TEST_CASE("menu grid: ROW-MAJOR, so the menu reads 1 2 3 / 4 5 6 / 7 8 9 / 10 11 12 / 13") {
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
    // Station 10 DRIVE (index 9) opens row 4, on the left; 12 (index 11) ends it; 13 MECHANISMS
    // (index 12) opens row 5, on the left, alone — its row's other two slots hit nothing.
    CHECK(buttonBox(kMenu, 9).x0 == buttonBox(kMenu, 0).x0);
    CHECK(buttonBox(kMenu, 11).y0 == buttonBox(kMenu, 9).y0);
    CHECK(buttonBox(kMenu, 12).x0 == buttonBox(kMenu, 0).x0);
    CHECK(buttonBox(kMenu, 12).y0 > buttonBox(kMenu, 9).y1);
    const Spec full{kMenuSlots, kMenu.cols, kMenu.x0, kMenu.y0, kMenu.w, kMenu.h, kMenu.gap};
    for (int ghost : {13, 14}) {
        CAPTURE(ghost);
        const Rect g = buttonBox(full, ghost);
        for (int y = g.y0; y <= g.y1; ++y) {
            for (int x = g.x0; x <= g.x1; ++x) {
                if (hitTest(kMenu, x, y) != -1) FAIL_CHECK("empty slot (" << x << "," << y << ")");
            }
        }
    }
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
    // line, and the longest old label ran into the dot at the proven MEDIUM width; and since
    // MECH-A6's 36 px rows, a second line that runs past the bottom edge or into the first.
    using shulib::calibration::kMediumCols;
    using shulib::calibration::kMediumHeight;
    using shulib::calibration::kSmallCols;
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
    // Line 1 (MEDIUM) ends where line 2 (SMALL) begins, at calibrate's MEDIUM height estimate (A4:
    // HA-136), and line 2 ends inside the bottom edge at the bench's own SMALL pitch (bench_r3a.cpp's
    // kLineH, static_asserted equal to kSmallPitch).
    CHECK(kLine1DY > 0);                                       // below the top edge line
    CHECK(kLine1DY + kMediumHeight <= kLine2DY);               // line 1's cell clear of line 2
    CHECK(r.y0 + kLine2DY + kSmallPitch <= r.y1);              // line 2's cell inside the bottom edge
    CHECK(kSmallPitch == 13);
    // The dot sits beside line 2, within its text cell's height.
    CHECK(dotTop >= r.y0 + kLine2DY);
    CHECK(dotBottom <= r.y0 + kLine2DY + kSmallPitch);
    // The character budgets these pixels give at the proven upper bounds of 472 px per 36 MEDIUM
    // and 50 SMALL characters (R3d-PROGRESS 15:43-15:50): 11 on line 1 (MEDIUM), 13 on line 2
    // (SMALL). bench_r3a.cpp static_asserts every label against the same arithmetic.
    CHECK(line1Px(kMenu) * static_cast<int>(kMediumCols) / 472 == 11);
    CHECK(line2Px(kMenu) * static_cast<int>(kSmallCols) / 472 == 13);
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
    // Menu buttons 11 and 12 (indices 10, 11) overlap RECORD (slot 1) and DONE (slot 2), and since
    // MECH-A6 menu button 13 (index 12) overlaps slot 0 (station 13's NEXT): every station arms its row
    // 0.5 s late for exactly this.
    CHECK(overlap(buttonBox(kMenu, 10), buttonBox(kStationRow, 1)));
    CHECK(overlap(buttonBox(kMenu, 11), buttonBox(kStationRow, 2)));
    CHECK(overlap(buttonBox(kMenu, 12), buttonBox(kStationRow, 0)));
    // Anything above the hit strip is not a button: the object rows and the status line live there.
    for (int x = 0; x < kPanelW; ++x) {
        if (hitTest(kStationRowHit, x, kStationRowHit.y0 - 1) != -1) FAIL_CHECK("x " << x);
    }
}

TEST_CASE("station 13's four-answer row: inside the panel, in the station row's band, its hit area containing "
          "every drawn button and adding only a strip ABOVE") {
    // BUG CAUGHT: a three-position pneumatic's question with an answer off the panel, two answers sharing a
    // pixel (a tap on 'middle' judged as 'up'), or a hit area that swallows the question's text above it.
    CHECK(kAnswerRow4.count == 4);
    CHECK(rowsOf(kAnswerRow4) == 1);
    for (int i = 0; i < 4; ++i) {
        CAPTURE(i);
        const Rect d = buttonBox(kAnswerRow4, i);
        const Rect h = buttonBox(kAnswerRow4Hit, i);
        CHECK(d.x0 >= 0);
        CHECK(d.x1 <= kPanelW - 1);
        CHECK(d.y0 == kStationRow.y0);                         // the station row's band
        CHECK(d.y1 == buttonBox(kStationRow, 0).y1);
        CHECK(d.y1 <= kLastRow);
        CHECK(d.y1 - d.y0 >= 36);
        CHECK(h.x0 == d.x0);
        CHECK(h.x1 == d.x1);
        CHECK(h.y1 == d.y1);
        CHECK(h.y0 == d.y0 - 8);
        for (int j = i + 1; j < 4; ++j) CHECK_FALSE(overlap(d, buttonBox(kAnswerRow4, j)));
        for (int y = d.y0; y <= d.y1; ++y) {
            for (int x = d.x0; x <= d.x1; ++x) {
                if (hitTest(kAnswerRow4Hit, x, y) != i) FAIL_CHECK("slot " << i << " (" << x << "," << y << ")");
            }
        }
    }
    CHECK(buttonBox(kAnswerRow4, 3).x1 == 462);
    for (int x = 0; x < kPanelW; ++x) {
        if (hitTest(kAnswerRow4Hit, x, kAnswerRow4Hit.y0 - 1) != -1) FAIL_CHECK("x " << x);
    }
}
