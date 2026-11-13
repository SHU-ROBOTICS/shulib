#pragma once
//
// ProsBrainScreen — hal::IBrainScreen over PROS's screen API (chunk CAL1, Part A1, 2026-10-08).
//
// Each call is the PROS sequence robot two's own "shulib Calibrate" (src/programs/calibrate_program.cpp,
// until CAL1 Part B made it the standard one, 2026-11-05) made inline: set the pen (or the eraser),
// then draw. print() is the sequence of printText()
// (src/common/screen_text.hpp): the text through the sanitizer (hal::screenSafeLine(), the same
// rules), then set the eraser, set the pen, and print the sanitized line through a LITERAL "%s" —
// so no text a caller passes is ever read by VEXos as a format. The touch read is PROS's
// screen_touch_status(), an E_TOUCH_ERROR read reported as an error (PROS fills every field with
// -1 then), never as a tap.
//
// One difference from the program's inline calls, and why it changes nothing on the screen: the
// program outlines a button twice after ONE set_pen; through this seam each outlineRect() sets the
// pen itself, so the second outline sets the same pen again. The pixels are the same.
//
// NO HARDWARE CLAIM BEYOND THE PROGRAM'S: these are the calls "shulib Calibrate" has made on robot
// two since 2026-09-24 (R3d Part 2F-2), in the same order; THIS adapter has not run on a robot.
// Its host test (test/pros_brain_screen_adapter_test.cpp) runs it through the host PROS shim,
// which records every call — that checks the adapter against our BELIEF about PROS, not the
// belief itself.

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include "pros/screen.h"
#pragma GCC diagnostic pop

#include <cstddef>
#include <cstdint>

#include "shulib/hal/brain_screen.hpp"

namespace shulib::hal::pros {

/// The brain's own touch screen, through PROS (header). Holds only its print count.
class ProsBrainScreen final : public IBrainScreen {
public:
    /// screen_set_pen(rgb), then screen_fill_rect.
    void fillRect(std::int16_t x0, std::int16_t y0, std::int16_t x1, std::int16_t y1,
                  std::uint32_t rgb) override {
        ::pros::c::screen_set_pen(rgb);
        ::pros::c::screen_fill_rect(x0, y0, x1, y1);
    }
    /// screen_set_pen(rgb), then screen_draw_rect.
    void outlineRect(std::int16_t x0, std::int16_t y0, std::int16_t x1, std::int16_t y1,
                     std::uint32_t rgb) override {
        ::pros::c::screen_set_pen(rgb);
        ::pros::c::screen_draw_rect(x0, y0, x1, y1);
    }
    /// screen_set_eraser(rgb), then screen_erase_rect.
    void eraseRect(std::int16_t x0, std::int16_t y0, std::int16_t x1, std::int16_t y1,
                   std::uint32_t rgb) override {
        ::pros::c::screen_set_eraser(rgb);
        ::pros::c::screen_erase_rect(x0, y0, x1, y1);
    }
    /// screen_set_eraser(rgb), then screen_erase.
    void eraseAll(std::uint32_t rgb) override {
        ::pros::c::screen_set_eraser(rgb);
        ::pros::c::screen_erase();
    }
    /// screenSafeLine(text, cols, padTo), then screen_set_eraser, screen_set_pen and
    /// screen_print_at(size, x, y, "%s", line) — printText()'s order.
    void print(ScreenTextSize size, std::int16_t x, std::int16_t y, std::uint32_t pen,
               std::uint32_t eraser, const char* text, std::size_t cols,
               std::size_t padTo = 0) override {
        char out[kScreenLineBytes];
        screenSafeLine(out, sizeof out, text, cols, padTo);
        ::pros::c::screen_set_eraser(eraser);
        ::pros::c::screen_set_pen(pen);
        ::pros::c::screen_print_at(formatOf(size), x, y, "%s", out);
        ++prints_;
    }
    /// screen_touch_status(), an E_TOUCH_ERROR read reported as an error.
    [[nodiscard]] ScreenTouch touch() override {
        const ::pros::screen_touch_status_s_t t = ::pros::c::screen_touch_status();
        ScreenTouch s;
        s.error = t.touch_status == ::pros::E_TOUCH_ERROR;
        s.releaseCount = t.release_count;
        s.x = t.x;
        s.y = t.y;
        return s;
    }
    /// Every print() through this object.
    [[nodiscard]] std::uint32_t prints() const override { return prints_; }

private:
    /// The PROS text format for a seam size (printText()'s formatOf()).
    static ::pros::text_format_e_t formatOf(ScreenTextSize s) {
        switch (s) {
            case ScreenTextSize::Small: return ::pros::E_TEXT_SMALL;
            case ScreenTextSize::Large: return ::pros::E_TEXT_LARGE;
            case ScreenTextSize::Medium: return ::pros::E_TEXT_MEDIUM;
        }
        return ::pros::E_TEXT_MEDIUM;
    }
    std::uint32_t prints_ = 0;
};

}  // namespace shulib::hal::pros
