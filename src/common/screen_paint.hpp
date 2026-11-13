#pragma once
//
// screen_paint.hpp — HOW A PROGRAM PAINTS ITS PAGES: the colours of the ui:: tones, one line, one
// button, the header bar, the auton selector's frame, and the full-screen REFUSAL (moved here at
// chunk COMP-b's first step, 2026-09-30 — the COMP brief's decision ND3 (b), which gives the
// refusal painter "a PROS-bound src/common/ pair (a PROS-free header and a PROS-bound .cpp, as
// state_screens is)").
//
// ── Why here ────────────────────────────────────────────────────────────────────────
// "shulib Auton Tester" (COMP-t, 2026-09-28) painted all of this in its own .cpp: its refusal
// screen ("wrapped like Teleop's", its log says), and the auton selector (competition::AutonMenu's
// view) as its PICK page. The competition program (COMP-b) paints the SAME selector and a refusal
// of the same shape, and the COMP brief's rule for a piece two programs need is that it moves to
// one shared place and is never copied (B11; ND3 (c), near-copies, was rejected). So the tester's
// painters moved here, and the tester now calls them — the same calls, in the same order, with the
// same colours and positions, so what its screen shows did not change. "shulib Teleop" kept its own
// refusal painter until T4 Part 3 retired it (its controller lines had moved to
// src/common/driver_panel.hpp at COMP-b); the new "shulib Drive" paints its refusal HERE, and its
// pages with the line, button and bar painters below — three programs, one painter.
//
// ── What each piece paints ──────────────────────────────────────────────────────────
//   toneRgb()            a ui::Tone (a colour by MEANING) as the RGB the brain draws;
//   clearPanel()         the whole screen to the background;
//   paintLine()          one competition::MenuLine, through printText() at its size's budget;
//   paintButton()        one button: its rectangle filled (grey while not armed), its label;
//   paintBar()           the header bar in a tone, with its line on it;
//   paintMenuFrame()     the auton selector's WHOLE frame (clear, bar, info, status, buttons);
//   paintMenuText()      only the selector's text lines that changed (same buttons);
//   paintRefusalScreen() a full-screen refusal and its serial line (below);
//   sameMenuButtons()    whether two selector frames need a whole repaint (pure).
// Every string reaches the screen through src/common/screen_text.hpp's printText() — ASCII, no
// '%', cut at the budget, printed through a literal "%s" (PROS re-reads printed text as a format).
//
// ── The refusal screen ──────────────────────────────────────────────────────────────
// A RED bar with the program's name and its BUILD time; the headline over two MEDIUM rows; the
// reason over four SMALL rows, with "(the full text is on the serial log)" when either did not
// fit; the footer over two SMALL rows. And one serial line, `[<tag>] REFUSED: <headline> --
// <why>` — wrapped into `[<tag>] ... ` pieces, one write() each, when it passes one serial line's
// 256 bytes (SER1). The layout is COMP-t's, unchanged (every number an ESTIMATE, A4 register HA-136).
// A program checks its own literals against the painter's budgets at compile time with
// refusalBarFits() and refusalFooterFits() below.
//
// PROS-free header (the host suite includes it for the budget checks); the painting is in
// src/common/screen_paint.cpp, which is PROS-bound (pros/screen.h). Compiled in every build like
// every .cpp under src/; --gc-sections drops what a build's program never calls. NOTHING HERE HAS
// RUN ON A ROBOT.

#include <cstddef>
#include <cstdint>
#include <string_view>

#include "shulib/calibration/calibrate_panel.hpp"
#include "shulib/competition/auton_menu.hpp"
#include "shulib/ui/touch_panel.hpp"

namespace shulib::bench {

/// The background every page is painted on (COMP-t's).
inline constexpr std::uint32_t kPanelBg = 0x101418;

/// A ui::Tone as RGB (COMP-t's palette): Blue 0x1E5AA8, Amber 0xE0A000, Green 0x2E8B3A, Red
/// 0xC0392B, Grey 0x707070, White 0xF0F0F0. Constexpr and PROS-free, so a host test pins it.
[[nodiscard]] constexpr std::uint32_t toneRgb(ui::Tone t) noexcept {
    switch (t) {
        case ui::Tone::Blue: return 0x1E5AA8;
        case ui::Tone::Amber: return 0xE0A000;
        case ui::Tone::Green: return 0x2E8B3A;
        case ui::Tone::Red: return 0xC0392B;
        case ui::Tone::Grey: return 0x707070;
        case ui::Tone::White: return 0xF0F0F0;
    }
    return 0xF0F0F0;
}

/// True when two lines would paint differently (their tone or their text).
[[nodiscard]] inline bool lineDiffers(const competition::MenuLine& a,
                                      const competition::MenuLine& b) noexcept {
    return a.tone != b.tone || std::string_view{a.text} != std::string_view{b.text};
}

/// The refusal's pointer to the serial log, when its headline or reason did not fit.
inline constexpr std::string_view kRefusalMoreOnSerial = "(the full text is on the serial log)";
static_assert(calibration::screenText(kRefusalMoreOnSerial, calibration::kSmallCols),
              "the refusal's serial pointer must fit one SMALL row");

/// True when `word` fits the refusal bar's program name (the header budget, screen-safe).
[[nodiscard]] constexpr bool refusalBarFits(std::string_view word) noexcept {
    return calibration::screenText(word, calibration::kHeaderCols);
}

/// True when `footer` fits the refusal screen's two SMALL footer rows, whole, and is screen-safe.
[[nodiscard]] constexpr bool refusalFooterFits(std::string_view footer) noexcept {
    return calibration::wrapWords<calibration::kSmallCols, 2>(footer).complete
           && calibration::screenText(footer, 2 * calibration::kSmallCols);
}

/// Everything one refusal screen says (file banner). `barWord` and `buildTime` must be
/// NUL-terminated (literals); `headline` and `why` may be null (painted empty).
struct RefusalScreen {
    const char* tag = "";          ///< the program's serial prefix, e.g. "ATS"
    const char* barWord = "";      ///< the program's name in the red bar (refusalBarFits)
    const char* buildTime = "";    ///< "BUILD hh:mm:ss", at the bar's right (SMALL)
    const char* headline = "";     ///< over two MEDIUM rows
    const char* why = "";          ///< over four SMALL rows
    std::string_view footer{};     ///< over two SMALL rows (refusalFooterFits)
    /// Print the `[<tag>] REFUSED: ...` serial line too (the default). A page REPAINTED many times
    /// -- the competition program's autons-refused page, painted at every take of the screen --
    /// says its words on serial once, where it decides, and passes false here.
    bool logToSerial = true;
};

/// True when two views of the auton selector have the same page and the same buttons (ids, entries,
/// rectangles, labels, armed, chosen). A difference means a WHOLE repaint now, so the frame a touch
/// is tested against is always the one on the screen; a text-only difference may wait for the next
/// live repaint. Shared by every program that paints the selector (the tester's sameButtons() and
/// the competition program's menu), moved here from the tester at COMP-b.
[[nodiscard]] inline bool sameMenuButtons(const competition::MenuView& a,
                                          const competition::MenuView& b) noexcept {
    if (a.page != b.page || a.buttonCount != b.buttonCount) {
        return false;
    }
    for (std::size_t i = 0; i < a.buttonCount; ++i) {
        const competition::MenuButton& x = a.buttons[i];
        const competition::MenuButton& y = b.buttons[i];
        if (x.id != y.id || x.entry != y.entry || !(x.rect == y.rect) || x.armed != y.armed
            || x.chosen != y.chosen || std::string_view{x.label} != std::string_view{y.label}) {
            return false;
        }
    }
    return true;
}

/// The whole screen to the background.
void clearPanel();

/// One line at its place, size and tone, on `eraser`, through printText() at its size's budget.
void paintLine(const competition::MenuLine& line, std::uint32_t eraser);

/// One button: its rectangle filled with `tone` (grey when not `armed`), its MEDIUM label at
/// (x0 + 8, y0 + 16) in white.
void paintButton(ui::Rect rect, const char* label, bool armed, ui::Tone tone);

/// The header bar (rows 0..26) filled with `bar`, with `header` painted on it.
void paintBar(ui::Tone bar, const competition::MenuLine& header);

/// The auton selector's WHOLE frame: clear, the bar in the header's tone, the info and status
/// lines, and every button (the chosen one green, the rest blue; greyed while not armed).
void paintMenuFrame(const competition::MenuView& view);

/// Only the selector's text lines that changed between `was` and `now` (the same buttons).
void paintMenuText(const competition::MenuView& was, const competition::MenuView& now);

/// The full-screen refusal and its serial line (file banner).
void paintRefusalScreen(const RefusalScreen& screen);

}  // namespace shulib::bench
