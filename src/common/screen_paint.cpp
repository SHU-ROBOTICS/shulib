// screen_paint.cpp — the painters (header: why they are shared, and what each paints). Moved from
// src/programs/auton_tester_program.cpp at COMP-b's first step (2026-09-30): the same PROS calls,
// in the same order, with the same colours and positions, so the tester's screen is unchanged.

#include "common/screen_paint.hpp"

#include <cstddef>
#include <cstdint>

#include "pros/screen.h"

#include "common/screen_text.hpp"
#include "shulib/hal/pros/char_sink.hpp"

namespace shulib::bench {
namespace {

/// A ui:: size as the size printText() takes (the two enums are held to the same budgets by the
/// programs' static_asserts and the host suite's).
TextSize printSizeOf(ui::TextSize s) {
    switch (s) {
        case ui::TextSize::Small: return TextSize::Small;
        case ui::TextSize::Medium: return TextSize::Medium;
        case ui::TextSize::Large: return TextSize::Large;
    }
    return TextSize::Small;
}

}  // namespace

void clearPanel() {
    pros::c::screen_set_eraser(kPanelBg);
    pros::c::screen_erase();
}

void paintLine(const competition::MenuLine& line, std::uint32_t eraser) {
    printText(printSizeOf(line.size), line.x, line.y, toneRgb(line.tone), eraser, line.text,
              ui::budgetFor(line.size));
}

void paintButton(ui::Rect rect, const char* label, bool armed, ui::Tone tone) {
    const std::uint32_t fill = armed ? toneRgb(tone) : toneRgb(ui::Tone::Grey);
    pros::c::screen_set_pen(fill);
    pros::c::screen_fill_rect(rect.x0, rect.y0, rect.x1, rect.y1);
    printText(TextSize::Medium, static_cast<std::int16_t>(rect.x0 + 8),
              static_cast<std::int16_t>(rect.y0 + 16), toneRgb(ui::Tone::White), fill, label);
}

void paintBar(ui::Tone bar, const competition::MenuLine& header) {
    pros::c::screen_set_pen(toneRgb(bar));
    pros::c::screen_fill_rect(0, 0, 479, 26);
    paintLine(header, toneRgb(bar));
}

void paintMenuFrame(const competition::MenuView& view) {
    clearPanel();
    paintBar(view.header.tone, view.header);
    paintLine(view.info, kPanelBg);
    paintLine(view.status, kPanelBg);
    for (std::size_t i = 0; i < view.buttonCount; ++i) {
        const competition::MenuButton& b = view.buttons[i];
        paintButton(b.rect, b.label, b.armed, b.chosen ? ui::Tone::Green : ui::Tone::Blue);
    }
}

void paintMenuText(const competition::MenuView& was, const competition::MenuView& now) {
    if (lineDiffers(was.header, now.header)) {
        paintLine(now.header, toneRgb(now.header.tone));
    }
    if (lineDiffers(was.info, now.info)) {
        paintLine(now.info, kPanelBg);
    }
    if (lineDiffers(was.status, now.status)) {
        paintLine(now.status, kPanelBg);
    }
}

void paintRefusalScreen(const RefusalScreen& screen) {
    const char* headline = screen.headline != nullptr ? screen.headline : "";
    const char* why = screen.why != nullptr ? screen.why : "";
    clearPanel();
    pros::c::screen_set_pen(toneRgb(ui::Tone::Red));
    pros::c::screen_fill_rect(0, 0, 479, 26);
    printText(TextSize::Medium, 8, 4, toneRgb(ui::Tone::White), toneRgb(ui::Tone::Red),
              screen.barWord);
    printText(TextSize::Small, 330, 8, toneRgb(ui::Tone::White), toneRgb(ui::Tone::Red),
              screen.buildTime);
    const auto h = calibration::wrapWords<calibration::kMediumCols, 2>(headline);
    for (std::size_t k = 0; k < h.count; ++k) {
        printText(TextSize::Medium, 8, static_cast<std::int16_t>(34 + 22 * static_cast<int>(k)),
                  toneRgb(ui::Tone::Red), kPanelBg, h.line[k]);
    }
    const auto w = calibration::wrapWords<calibration::kSmallCols, 4>(why);
    for (std::size_t k = 0; k < w.count; ++k) {
        printText(TextSize::Small, 8, static_cast<std::int16_t>(82 + 16 * static_cast<int>(k)),
                  toneRgb(ui::Tone::White), kPanelBg, w.line[k]);
    }
    if (!w.complete || !h.complete) {
        printText(TextSize::Small, 8, 146, toneRgb(ui::Tone::Grey), kPanelBg,
                  kRefusalMoreOnSerial.data());
    }
    const auto foot = calibration::wrapWords<calibration::kSmallCols, 2>(screen.footer);
    for (std::size_t k = 0; k < foot.count; ++k) {
        printText(TextSize::Small, 8, static_cast<std::int16_t>(182 + 16 * static_cast<int>(k)),
                  toneRgb(ui::Tone::Grey), kPanelBg, foot.line[k]);
    }
    if (screen.logToSerial) {
        // ONE write() per piece (SER1, HA-189): a headline of 63 and a reason of 191 characters
        // can pass one serial line's kSerialLineBytes, so the line WRAPS — never cut, never printf.
        hal::pros::serialWrapped(screen.tag != nullptr ? screen.tag : "",
                                 {"REFUSED: ", headline, " -- ", why});
    }
}

}  // namespace shulib::bench
