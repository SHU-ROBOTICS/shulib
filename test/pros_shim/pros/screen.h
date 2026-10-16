#pragma once
//
// HOST SHIM for <pros/screen.h> (chunk CAL1, Part A1, 2026-10-08) — the brain screen's C API, as
// far as the library's PROS screen adapter (include/shulib/hal/pros/brain_screen.hpp) and the
// programs' printText() (src/common/screen_text.hpp) call it.
//
// Two records, because two questions are asked of a screen:
//   * `log` — every DRAW, one line of text each, in EXACTLY the words hal::fake::FakeBrainScreen
//     records for the same draw (fill / rect / erase / erase-all / print, with the pen or eraser
//     in force at that moment) — so a program's screen through the PROS adapter can be compared,
//     call for call, with the same program's screen on the fakes;
//   * `calls` — every C call in order, the pen and eraser settings included, and print_at's
//     FORMAT string as passed — so a test can check that the adapter sets the eraser, then the
//     pen, then prints through a literal "%s", which `log` alone cannot show.
// The touch status is programmable in PROS's two steps — a press moves the place and the press
// count, the release moves the release count (screenPress(), screenRelease(); screenTap() is
// both) — and an error read returns what PROS returns: E_TOUCH_ERROR, every field -1. A print's
// format is recorded exactly (S, M, L, MC, LC — formatLetters()), so a size mapped to a centred
// format is visible.
//
// Types and enumerator values transcribed from the vendored include/pros/screen.h :49-87
// (text_format_e_t, last_touch_e_t, screen_touch_status_s_t); the functions' signatures from its
// :149-750 (each returns 1 here, PROS's success). print_at formats with vsnprintf, as PROS hands
// the formatted text on — so a '%' that reached it would be read as a conversion here too.
//
// Reset with pros::shim::resetScreen(): pros::shim::resetAll() (shim_control.hpp) does not reach
// this file, which is newer than it.
//
// HONEST LIMIT: this shim tests the adapter against OUR BELIEF about PROS; it cannot test the
// belief. Hardware tests the belief (shim_control.hpp says it in full).

#ifndef SHULIB_HOST_PROS_SHIM
#error "test/pros_shim/ is the HOST TEST shim for PROS — it must NEVER reach a robot build. \
Only test/CMakeLists.txt defines SHULIB_HOST_PROS_SHIM."
#endif

#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace pros {

typedef enum {
    E_TEXT_SMALL = 0,
    E_TEXT_MEDIUM,
    E_TEXT_LARGE,
    E_TEXT_MEDIUM_CENTER,
    E_TEXT_LARGE_CENTER
} text_format_e_t;

typedef enum { E_TOUCH_RELEASED = 0, E_TOUCH_PRESSED, E_TOUCH_HELD, E_TOUCH_ERROR } last_touch_e_t;

typedef struct screen_touch_status_s {
    last_touch_e_t touch_status;
    std::int16_t x;
    std::int16_t y;
    std::int32_t press_count;
    std::int32_t release_count;
} screen_touch_status_s_t;

namespace shim {

/// The screen's state, shared by every call below.
struct ScreenState {
    std::uint32_t pen = 0xFFFFFF;    ///< what screen_set_pen() last set
    std::uint32_t eraser = 0x000000; ///< what screen_set_eraser() last set
    std::vector<std::string> log;    ///< every draw, in order (header: FakeBrainScreen's words)
    std::vector<std::string> calls;  ///< every C call, in order, settings and formats included
    screen_touch_status_s_t touch{E_TOUCH_RELEASED, 0, 0, 0, 0};  ///< what a good read returns
    bool touchError = false;         ///< true: every read is an E_TOUCH_ERROR read
    std::uint64_t prints = 0;        ///< screen_print_at() calls
    int touchReads = 0;              ///< screen_touch_status() calls
    /// Optional observer of every draw (a test's operator model), or null.
    void (*observer)(const std::string&) = nullptr;
};
inline ScreenState& screenState() {
    static ScreenState s;
    return s;
}
inline void resetScreen() { screenState() = ScreenState{}; }

inline void screenRecord(const std::string& line) {
    screenState().log.push_back(line);
    screenState().calls.push_back(line);
    if (screenState().observer != nullptr) {
        screenState().observer(line);
    }
}
inline std::string hex(std::uint32_t c) {
    char b[16];
    std::snprintf(b, sizeof b, "%06x", static_cast<unsigned>(c));
    return b;
}
inline std::string rect(std::int16_t x0, std::int16_t y0, std::int16_t x1, std::int16_t y1) {
    char b[64];
    std::snprintf(b, sizeof b, "%d %d %d %d", static_cast<int>(x0), static_cast<int>(y0),
                  static_cast<int>(x1), static_cast<int>(y1));
    return b;
}

/// A finger DOWN at (x, y): the next good read says E_TOUCH_PRESSED there, with one more PRESS
/// (press_count rises when a finger LANDS — A4 register HA-178 (a), reasoned from the vendored
/// text) — and the release count unchanged, as PROS counts a touch on its release. The two
/// counters move in separate steps here, so a test can tell an adapter that reads press_count
/// from one that reads release_count.
inline void screenPress(std::int16_t x, std::int16_t y) {
    auto& t = screenState().touch;
    t.touch_status = E_TOUCH_PRESSED;
    t.x = x;
    t.y = y;
    t.press_count += 1;
}
/// The finger lifted where it was: E_TOUCH_RELEASED, with one more release.
inline void screenRelease() {
    auto& t = screenState().touch;
    t.touch_status = E_TOUCH_RELEASED;
    t.release_count += 1;
}
/// A tap at (x, y): screenPress(x, y), then screenRelease().
inline void screenTap(std::int16_t x, std::int16_t y) {
    screenPress(x, y);
    screenRelease();
}

/// The letters a print's FORMAT is recorded as — one per enumerator of text_format_e_t, so an
/// adapter that maps a size to a CENTRED format (or to a value PROS does not have) records
/// something no FakeBrainScreen print can: S, M, L, MC, LC, or '?'. (FakeBrainScreen records only
/// S, M and L: the seam has three sizes, each meant as PROS's plain format.)
inline const char* formatLetters(text_format_e_t f) {
    switch (f) {
        case E_TEXT_SMALL: return "S";
        case E_TEXT_MEDIUM: return "M";
        case E_TEXT_LARGE: return "L";
        case E_TEXT_MEDIUM_CENTER: return "MC";
        case E_TEXT_LARGE_CENTER: return "LC";
    }
    return "?";
}

}  // namespace shim

namespace c {

inline std::uint32_t screen_set_pen(std::uint32_t color) {
    shim::screenState().pen = color;
    shim::screenState().calls.push_back("set_pen " + shim::hex(color));
    return 1;
}
inline std::uint32_t screen_set_eraser(std::uint32_t color) {
    shim::screenState().eraser = color;
    shim::screenState().calls.push_back("set_eraser " + shim::hex(color));
    return 1;
}
inline std::uint32_t screen_erase() {
    shim::screenRecord("erase-all eraser=" + shim::hex(shim::screenState().eraser));
    return 1;
}
inline std::uint32_t screen_fill_rect(std::int16_t x0, std::int16_t y0, std::int16_t x1,
                                      std::int16_t y1) {
    shim::screenRecord("fill " + shim::rect(x0, y0, x1, y1) + " pen="
                       + shim::hex(shim::screenState().pen));
    return 1;
}
inline std::uint32_t screen_draw_rect(std::int16_t x0, std::int16_t y0, std::int16_t x1,
                                      std::int16_t y1) {
    shim::screenRecord("rect " + shim::rect(x0, y0, x1, y1) + " pen="
                       + shim::hex(shim::screenState().pen));
    return 1;
}
inline std::uint32_t screen_erase_rect(std::int16_t x0, std::int16_t y0, std::int16_t x1,
                                       std::int16_t y1) {
    shim::screenRecord("erase " + shim::rect(x0, y0, x1, y1) + " eraser="
                       + shim::hex(shim::screenState().eraser));
    return 1;
}
inline std::uint32_t screen_print_at(text_format_e_t txt_fmt, const std::int16_t x,
                                     const std::int16_t y, const char* text, ...)
    __attribute__((format(printf, 4, 5)));
inline std::uint32_t screen_print_at(text_format_e_t txt_fmt, const std::int16_t x,
                                     const std::int16_t y, const char* text, ...) {
    char buf[512];
    va_list args;
    va_start(args, text);
    std::vsnprintf(buf, sizeof buf, text, args);
    va_end(args);
    const char* sz = shim::formatLetters(txt_fmt);
    char head[64];
    std::snprintf(head, sizeof head, "print %s %d %d", sz, static_cast<int>(x),
                  static_cast<int>(y));
    shim::screenState().calls.push_back(std::string{"print_at format '"} + text + "'");
    shim::screenRecord(std::string{head} + " pen=" + shim::hex(shim::screenState().pen)
                       + " eraser=" + shim::hex(shim::screenState().eraser) + " '" + buf + "'");
    ++shim::screenState().prints;
    return 1;
}
inline screen_touch_status_s_t screen_touch_status() {
    shim::screenState().touchReads += 1;
    if (shim::screenState().touchError) {
        return screen_touch_status_s_t{E_TOUCH_ERROR, -1, -1, -1, -1};
    }
    return shim::screenState().touch;
}

}  // namespace c
}  // namespace pros
