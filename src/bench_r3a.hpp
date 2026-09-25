#pragma once
//
// R3a's bench validation entry point (chunk R3a, §4.2 item 1), extended at R3b
// Session 2 with the per-variant chassis table and the DRIVE station, and at R2a
// (2026-09-24) with stations 11 DISTANCE and 12 AI VISION and the three-column menu.
//
// Declared in its own header so src/main.cpp can dispatch to it under the
// compile-time robot selector WITHOUT the X-drive object graph ever being
// constructed — that graph's ports are invented (HA-111) and every adapter
// ctor read-back would throw at boot on the bench robot. Both tester variants
// (ROBOT=bench, the measured bench bot; ROBOT=tank, the 2026 chassis whose
// table carries MEASURED SIGNS since R3b Part 0b) boot into this as "Bench
// Tests" -- unless built with PROGRAM=drive, which boots src/drive_program.cpp
// ("shulib Drive") instead. The chassis table both share is src/chassis_table.hpp.
//
// This lives in src/, not include/shulib/, deliberately: it makes raw
// <pros/*> calls to print RAW value beside CANONICAL value, which is the
// whole point of the chunk, and include/shulib/ is PROS-free by CI guard.
//
// THIS HEADER ITSELF IS PROS-FREE (since R2a): besides runR3a() it carries only the
// menu's pure GEOMETRY (menu_grid below), so test/bench_menu_grid_test.cpp can include it
// by relative path and try to break the hit test on the host -- the chassis_table.hpp
// precedent. Nothing here draws; bench_r3a.cpp draws from it.

#include <cstdint>

namespace shulib::bench {

/// The whole bench session: device census, per-device raw-vs-canonical report,
/// loop-rate measurement, a live motor watch that captures signs — and, since R3b
/// Session 2, ONE station (10 DRIVE) that POWERS the drive motors through the
/// hal/pros adapters behind six safety gates. Since R2a, station 11 DISTANCE shows the
/// distance sensors' raw PROS reads beside the library adapter's, and station 12 AI VISION
/// shows the camera's detections -- it WRITES the camera's own detection settings (tag +
/// object detection on, the USB bounding-box overlay on, a tag family when asked) and says
/// so on its screen, but powers nothing. Every station except 10 commands no motion.
/// Never returns.
void runR3a();

/// THE MENU GRID — pure geometry (R2a, 2026-09-24; the 2 × 5 menu became 3 × 4 so twelve
/// stations fit without shrinking a target).
///
/// Every rectangle has INCLUSIVE corners, PROS's drawing convention: screen_fill_rect(x0, y0,
/// x1, y1) paints x1 - x0 + 1 columns. A Spec's `w`/`h` are therefore x1 - x0 and y1 - y0 of one
/// button (the old kBtnW/kBtnH), and `gap` is the PITCH minus `w` — gap - 1 pixels lie between
/// two buttons.
///
/// THE HIT-TEST RULES, kept from the 2-column menu and now tested:
///  * NO DEAD GAP inside the grid: a gap pixel belongs to the button above / to the left of it
///    (integer division of the pitch). On robot two's brain the first recorded tap (2026-09-10,
///    `touch: x=364 y=117`) released one pixel into a gap and did nothing, on a resistive screen
///    whose release point wanders a few pixels.
///  * DEAD ZONES OUTSIDE it: the header strip above, the margins beside and the strip below hit
///    nothing — so a stale or error coordinate ((0,0), or −1 from an E_TOUCH_ERROR read) can
///    never launch a station.
///  * THE RIGHT EDGE COMES FROM THE RIGHTMOST COLUMN, not from the last button: on a partial last
///    row the last button is not in the rightmost column, and taking the extent from it (the
///    2-column code's idiom) would make the whole right column dead.
namespace menu_grid {

/// A rectangle, inclusive corners.
struct Rect {
    std::int16_t x0;
    std::int16_t y0;
    std::int16_t x1;
    std::int16_t y1;
};

/// `count` buttons, `cols` per row, ROW-MAJOR (button i is at row i / cols, column i % cols),
/// the first one's top-left at (x0, y0).
struct Spec {
    int count;
    int cols;
    std::int16_t x0;
    std::int16_t y0;
    std::int16_t w;    ///< x1 - x0 of one button
    std::int16_t h;    ///< y1 - y0 of one button
    std::int16_t gap;  ///< pitch - w (and pitch - h)
};

/// Rows needed for `count` buttons.
constexpr int rowsOf(const Spec& g) {
    return g.cols > 0 ? (g.count + g.cols - 1) / g.cols : 0;
}

/// Columns actually occupied (fewer than `cols` when there is only one, partial, row).
constexpr int colsUsed(const Spec& g) { return g.count < g.cols ? g.count : g.cols; }

/// Button i's rectangle.
constexpr Rect buttonBox(const Spec& g, int i) {
    const int col = i % g.cols;
    const int row = i / g.cols;
    const int x0 = g.x0 + col * (g.w + g.gap);
    const int y0 = g.y0 + row * (g.h + g.gap);
    return Rect{static_cast<std::int16_t>(x0), static_cast<std::int16_t>(y0),
                static_cast<std::int16_t>(x0 + g.w), static_cast<std::int16_t>(y0 + g.h)};
}

/// The grid's extent: from the first button's top-left to the far corner of the RIGHTMOST
/// column and the LAST row (see the header note on why not the last button's corner).
constexpr Rect extent(const Spec& g) {
    const int x1 = g.x0 + (colsUsed(g) - 1) * (g.w + g.gap) + g.w;
    const int y1 = g.y0 + (rowsOf(g) - 1) * (g.h + g.gap) + g.h;
    return Rect{g.x0, g.y0, static_cast<std::int16_t>(x1), static_cast<std::int16_t>(y1)};
}

/// Which button (tx, ty) hits, or -1: outside the extent, or in an EMPTY slot of a partial
/// last row. Inside the extent, gap pixels belong to the button above / to the left.
constexpr int hitTest(const Spec& g, int tx, int ty) {
    if (g.count <= 0 || g.cols <= 0) return -1;
    const Rect e = extent(g);
    if (tx < e.x0 || tx > e.x1 || ty < e.y0 || ty > e.y1) return -1;
    int col = (tx - e.x0) / (g.w + g.gap);
    int row = (ty - e.y0) / (g.h + g.gap);
    // Defensive only: inside the extent the division already lands in range (the last
    // column's and row's gap lies OUTSIDE the extent). Kept so a future edit to extent()
    // cannot index past the grid.
    if (col > colsUsed(g) - 1) col = colsUsed(g) - 1;
    if (row > rowsOf(g) - 1) row = rowsOf(g) - 1;
    const int i = row * g.cols + col;
    return i < g.count ? i : -1;
}

/// The usable panel: 480 wide, and nothing drawn at y >= 236 (VEXos keeps the rows below;
/// liblvgl's lv_conf.h says 240, and the tester keeps a 4 px margin — a measured lesson,
/// bench_r3a.cpp's kUsableH note).
inline constexpr int kPanelW = 480;
inline constexpr int kLastRow = 235;
/// The menu's header bar is painted over y 0..32; the grid must start below it.
inline constexpr int kMenuHeaderBottom = 32;

/// THE MENU: twelve stations, three across, four down — 1 2 3 / 4 5 6 / 7 8 9 / 10 11 12.
/// Columns 6-159 / 163-316 / 320-473 (6 px margins both sides); rows 38-84 / 88-134 /
/// 138-184 / 188-234. Each button is 46 px tall (y1 - y0), taller than the 2 × 5 menu's 36.
inline constexpr Spec kMenu{12, 3, 6, 38, 153, 46, 4};

// ── what is drawn INSIDE a menu button (bench_r3a.cpp's drawMenu), as offsets ──────
/// The hands-on stripe: x0 .. x0 + kStripeW - 1, full height.
inline constexpr int kStripeW = 6;
/// Both label lines start here (2 px clear of the stripe).
inline constexpr int kLabelInsetX = 8;
/// Line 1's and line 2's top, from y0 (MEDIUM at a 22 px pitch — the IMU rotate screen's two
/// MEDIUM lines at y 44 / 66 are the hardware-read precedent).
inline constexpr int kLine1DY = 3;
inline constexpr int kLine2DY = 25;
/// The status dot: centre (x1 - kDotDX, y0 + kDotDY), radius kDotR — beside line 2.
inline constexpr int kDotDX = 14;
inline constexpr int kDotDY = 34;
inline constexpr int kDotR = 6;
/// Line 2 stops this many pixels short of the dot's left edge.
inline constexpr int kDotClear = 3;

/// Pixels available to label line 1: from x0 + kLabelInsetX to x1 - 1 (the edge is drawn at x1).
constexpr int line1Px(const Spec& g) { return g.w - kLabelInsetX; }
/// Pixels available to label line 2: from x0 + kLabelInsetX to kDotClear short of the dot.
constexpr int line2Px(const Spec& g) {
    return (g.w - kDotDX - kDotR - kDotClear) - kLabelInsetX + 1;
}

/// THE STATIONS' BUTTON ROW (11 DISTANCE, 12 AI VISION): three slots across the bottom, the
/// menu's columns — slot 0 (left) FAMILY on station 12 and empty on 11, slot 1 (middle) RECORD,
/// slot 2 (right) DONE, so RECORD and DONE sit in the same place on both stations.
inline constexpr Spec kStationRow{3, 3, 6, 186, 153, 48, 4};
/// Where a touch HITS the row: the same columns, extended 8 px ABOVE the drawn buttons (a
/// resistive release lands high — robot two's calibrate capture has one exactly on a button's
/// top edge). Nothing is drawn in those 8 px but the end of the status line.
inline constexpr Spec kStationRowHit{3, 3, 6, 178, 153, 56, 4};

}  // namespace menu_grid
}  // namespace shulib::bench
