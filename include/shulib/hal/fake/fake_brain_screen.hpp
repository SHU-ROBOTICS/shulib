#pragma once
//
// FakeBrainScreen — the host's IBrainScreen (chunk CAL1, Part A1, 2026-10-08). Every call is
// RECORDED as one line of text, in exactly the words the host PROS shim's screen
// (test/pros_shim/pros/screen.h) records for the same draw through hal::pros::ProsBrainScreen —
// so a program's screen on the fakes can be compared, call for call, with the same program's
// screen through the PROS adapter (test/pros_brain_screen_adapter_test.cpp holds the two equal).
// The touch is programmable — a press moves the place, a release moves the release count, as PROS
// does — and an error read reads as PROS's does (every field -1); the same test holds the fake's
// touch reads equal to the adapter's, read for read.
//
// The recorded words:
//   fill x0 y0 x1 y1 pen=rrggbb
//   rect x0 y0 x1 y1 pen=rrggbb
//   erase x0 y0 x1 y1 eraser=rrggbb
//   erase-all eraser=rrggbb
//   print S|M|L x y pen=rrggbb eraser=rrggbb 'the text AFTER screenSafeLine()'
//
// Like every IBrainScreen it sanitizes in print(), so what a test reads is what VEXos would be
// handed. Host-test infrastructure: it allocates (std::string, std::vector). PROS-free, sim-free.

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "shulib/hal/brain_screen.hpp"

namespace shulib::hal::fake {

/// A recording brain screen with a programmable touch (header).
class FakeBrainScreen final : public IBrainScreen {
public:
    /// Records "fill x0 y0 x1 y1 pen=rrggbb".
    void fillRect(std::int16_t x0, std::int16_t y0, std::int16_t x1, std::int16_t y1,
                  std::uint32_t rgb) override {
        record("fill " + rect(x0, y0, x1, y1) + " pen=" + hex(rgb));
    }
    /// Records "rect x0 y0 x1 y1 pen=rrggbb".
    void outlineRect(std::int16_t x0, std::int16_t y0, std::int16_t x1, std::int16_t y1,
                     std::uint32_t rgb) override {
        record("rect " + rect(x0, y0, x1, y1) + " pen=" + hex(rgb));
    }
    /// Records "erase x0 y0 x1 y1 eraser=rrggbb".
    void eraseRect(std::int16_t x0, std::int16_t y0, std::int16_t x1, std::int16_t y1,
                   std::uint32_t rgb) override {
        record("erase " + rect(x0, y0, x1, y1) + " eraser=" + hex(rgb));
    }
    /// Records "erase-all eraser=rrggbb".
    void eraseAll(std::uint32_t rgb) override { record("erase-all eraser=" + hex(rgb)); }
    /// Records "print S|M|L x y pen=… eraser=… 'text'" — the text AFTER screenSafeLine(), cut at
    /// `cols` and padded to `padTo`, exactly as the PROS adapter hands it to VEXos.
    void print(ScreenTextSize size, std::int16_t x, std::int16_t y, std::uint32_t pen,
               std::uint32_t eraser, const char* text, std::size_t cols,
               std::size_t padTo = 0) override {
        char out[kScreenLineBytes];
        screenSafeLine(out, sizeof out, text, cols, padTo);
        char head[48];
        std::snprintf(head, sizeof head, "print %s %d %d",
                      size == ScreenTextSize::Small    ? "S"
                      : size == ScreenTextSize::Medium ? "M"
                                                       : "L",
                      static_cast<int>(x), static_cast<int>(y));
        record(std::string{head} + " pen=" + hex(pen) + " eraser=" + hex(eraser) + " '" + out
               + "'");
        ++prints_;
    }
    /// The programmed touch status: the last good status (press(), release(), tap()) — or, while
    /// setTouchError(true) holds, what PROS answers a FAILED read with: the error flag and every
    /// other field -1 (the vendored pros/screen.h says so; the host shim and
    /// hal::pros::ProsBrainScreen give the same, and a host test holds this fake's reads equal to
    /// the adapter's, read for read). So a reader that takes a release count from an error read
    /// sees the same phantom change here as on the robot.
    [[nodiscard]] ScreenTouch touch() override {
        if (touchError_) {
            ScreenTouch failed;
            failed.error = true;
            failed.releaseCount = -1;
            failed.x = -1;
            failed.y = -1;
            return failed;
        }
        return touch_;
    }
    /// Every print() so far.
    [[nodiscard]] std::uint32_t prints() const override { return prints_; }

    /// A finger DOWN at (x, y), not yet lifted: the touch reads there, and the release count does
    /// not move — PROS counts a touch on its release, so a release-edge reader acts on nothing yet.
    void press(int x, int y) {
        touch_.x = static_cast<std::int16_t>(x);
        touch_.y = static_cast<std::int16_t>(y);
    }
    /// The finger lifted where it was: one more release — the edge a reader acts on.
    void release() { ++touch_.releaseCount; }
    /// A tap at (x, y): press(x, y), then release() — what PROS reports after a finger is lifted.
    void tap(int x, int y) {
        press(x, y);
        release();
    }
    /// Make every following touch read a FAILED read (true) — the error flag and every other field
    /// -1, as PROS answers — or good reads again (false). The last good status is kept, and taps
    /// made meanwhile still count, so the first good read afterwards shows exactly what changed.
    void setTouchError(bool error) { touchError_ = error; }
    /// Every recorded call, in order.
    [[nodiscard]] const std::vector<std::string>& log() const noexcept { return log_; }
    /// An observer called with every recorded line as it is recorded (a test's operator model,
    /// which taps only what the screen has drawn), or null for none.
    void observe(void (*fn)(const std::string& line, void* ctx), void* ctx) {
        observer_ = fn;
        ctx_ = ctx;
    }

private:
    static std::string hex(std::uint32_t c) {
        char b[16];
        std::snprintf(b, sizeof b, "%06x", static_cast<unsigned>(c));
        return b;
    }
    static std::string rect(std::int16_t x0, std::int16_t y0, std::int16_t x1, std::int16_t y1) {
        char b[64];
        std::snprintf(b, sizeof b, "%d %d %d %d", static_cast<int>(x0), static_cast<int>(y0),
                      static_cast<int>(x1), static_cast<int>(y1));
        return b;
    }
    void record(const std::string& line) {
        log_.push_back(line);
        if (observer_ != nullptr) {
            observer_(line, ctx_);
        }
    }

    std::vector<std::string> log_;
    ScreenTouch touch_{};  // the last good status (never has `error` set)
    bool touchError_ = false;
    std::uint32_t prints_ = 0;
    void (*observer_)(const std::string&, void*) = nullptr;
    void* ctx_ = nullptr;
};

}  // namespace shulib::hal::fake
