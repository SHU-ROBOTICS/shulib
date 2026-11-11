#pragma once
//
// screen_text.hpp — THE ONE PATH FROM A PROGRAM TO THE BRAIN'S SCREEN (G1 Part 2b, 2026-09-26).
//
// printText() was written for "shulib Calibrate" at R3d Part 2F-2 (2026-09-24), inside
// src/programs/calibrate_program.cpp's anonymous namespace. G1 Part 2b moved it here, VERBATIM (only
// the `inline` a header needs was added), so that "shulib Teleop" painted through the same sanitizer
// (until T4 Part 3 retired it; "shulib Drive", src/programs/drive_program.cpp, holds its words now),
// and the competition program (COMP) can too. Since CAL1 Part B (2026-11-05) "shulib Calibrate" is
// the library's standard Calibrate, which paints through hal::IBrainScreen and no longer calls this;
// the drive, competition and X-drive programs and src/common/screen_paint.cpp still do.
//
// ═══ ONLY A LITERAL "%s" EVER REACHES A PROS SCREEN CALL ═══════════════════════════
// PROS hands the already-formatted text to VEXos AS A FORMAT STRING, so a '%' that reaches the
// screen is re-read as a conversion with no argument behind it; and VEXos's fonts carry ASCII
// only. printText() copies the text into a buffer sized from the size's budget THROUGH THE
// LIBRARY'S ONE SANITIZER, hal::screenSafeLine() (include/shulib/hal/brain_screen.hpp — the rule
// every IBrainScreen::print() uses: '%' and control bytes become '?', a UTF-8 em/en dash '-', any
// other non-ASCII character one '?'; the hal/pros adapters' precondition messages carry em dashes,
// and a refusal screen shows them), cut at the budget, and prints it through "%s". Until CAL1 Part
// B the same rule was written out here a second time (A1's host test held the two copies equal);
// now there is one. src/ compiles under -Wformat-nonliteral and -Wformat-truncation, which the src
// build gate FAILS on.
//
// ═══ WHAT IT TAKES FROM THE LIBRARY, AND DOES NOT CHANGE ════════════════════════════
// The text sizes (calibration::TextSize) and their budgets (calibration::budgetFor, kSmallCols
// 50 / kMediumCols 36 / kLargeCols 18 characters per line — ESTIMATES, A4 register HA-136) come
// from include/shulib/calibration/calibrate_panel.hpp, which this header does NOT modify (the
// COMP brief's §0 row "The boot source" / COMP-b §3.1). A literal a program prints is checked
// against those budgets at compile time with calibration::screenText(), as the library panel's
// `panel_text::kAll` and src/programs/drive_panel.hpp's own words do.
//
// ═══ WHY NOT "Bench Tests" TOO ══════════════════════════════════════════════════════
// src/programs/bench_r3a.cpp keeps its OWN sanitizer (panelSafe/screenText, R2a): the same rules, but cut
// at the tester's 54-column scrolling-log width, not the SMALL budget, and with its own print
// counter — and its widths ran on hardware as they are (R2a-R3). It does not include this header,
// which is what lets both files name their counter g_screenPrints without a clash (the tester's
// lives in its anonymous namespace; this one is shulib::bench::g_screenPrints).
//
// PROS-bound (pros/screen.h), so it lives in src/, never include/shulib/. Header-only: each
// program that includes it is its own build (one program per upload), so the one counter below
// counts that program's prints.

#include <cstddef>
#include <cstdint>

#include "pros/screen.h"

#include "shulib/calibration/calibrate_panel.hpp"
#include "shulib/hal/brain_screen.hpp"

namespace shulib::bench {

using calibration::TextSize;

/// The PROS text format for one of the library's three text sizes.
inline pros::text_format_e_t formatOf(TextSize s) {
    switch (s) {
        case TextSize::Small: return pros::E_TEXT_SMALL;
        case TextSize::Large: return pros::E_TEXT_LARGE;
        case TextSize::Medium: return pros::E_TEXT_MEDIUM;
    }
    return pros::E_TEXT_MEDIUM;
}

// The widest budget is SMALL's, so kPanelLineCap (kSmallCols + 1) holds a line of any size.
static_assert(calibration::kLargeCols <= calibration::kSmallCols
                  && calibration::kMediumCols <= calibration::kSmallCols,
              "printText's buffer is sized from the SMALL budget, the widest");

/// Every call to screen_print_at since the program started — PROS leaks each one's buffer, so the
/// count is reported at every outcome (each program's own serial record says where).
inline std::uint32_t g_screenPrints = 0;

/// THE ONE PATH TO THE SCREEN (header): `text` through hal::screenSafeLine() — '%' and control
/// bytes become '?', a UTF-8 em/en dash becomes '-', any other non-ASCII character becomes ONE
/// '?', and the copy stops at the size's budget — padded with spaces to `padTo` characters (never
/// past the budget: an in-place update covers the old, longer text), then printed through a
/// literal "%s" with `pen` on `eraser`.
inline void printText(TextSize size, std::int16_t x, std::int16_t y, std::uint32_t pen,
                      std::uint32_t eraser, const char* text, std::size_t padTo = 0) {
    char out[calibration::kPanelLineCap];
    (void)hal::screenSafeLine(out, sizeof out, text, calibration::budgetFor(size), padTo);
    pros::c::screen_set_eraser(eraser);
    pros::c::screen_set_pen(pen);
    pros::c::screen_print_at(formatOf(size), x, y, "%s", out);
    ++g_screenPrints;
}

}  // namespace shulib::bench
