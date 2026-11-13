#pragma once
//
// IBrainScreen — the V5 BRAIN's own touch screen, as a seam (chunk CAL1, Part A1, 2026-10-08).
//
// Why a seam: the standard Calibrate is library code, and the library is PROS-free outside
// hal/pros/ (CI enforces it, path-anchored). What a calibration screen paints — which lines to
// redraw, in which colour, padded how far — is LOGIC, and logic belongs where the host suite can
// RUN it. So the library's painter (calibration/calibrate_screen.hpp) draws through this seam:
// hal::pros::ProsBrainScreen on the robot, hal::fake::FakeBrainScreen in a host test (it records
// every call as one line of text, so a screen can be compared call for call).
//
// ═══ ONLY SANITIZED TEXT EVER REACHES THE SCREEN ════════════════════════════════════
// We BELIEVE PROS hands printed text to VEXos AS A FORMAT STRING — R3d Part 2F's reading of PROS
// 4.2.2, the belief printText() (src/common/screen_text.hpp) was written on; read, never measured,
// and not yet an entry of the A4 register — so a '%' that reached the screen would be re-read as a
// conversion with no argument behind it; and VEXos's fonts carry ASCII only. Sanitizing costs
// nothing if the belief is wrong and saves the screen if it is right. screenSafeLine()
// below is the screen's sanitizer for the library — every IBrainScreen::print() passes its text
// through it, so a caller can never hand VEXos a format character. Its rules are those printText()
// (src/common/screen_text.hpp, the robot programs' one path to the screen, written for "shulib
// Calibrate" at R3d Part 2F-2 on 2026-09-24) was written with: '%' and control bytes become '?', a
// UTF-8 en or em dash becomes '-', any other non-ASCII character becomes ONE '?', the text is cut at
// the size's budget, then padded with spaces (never past the budget). In CAL1 Part A1 the two were
// the same rules in two places, held equal by a host test; since CAL1 Part B (2026-11-05)
// printText() CALLS this one, so there is one rule. (Bench Tests,
// src/programs/bench_r3a.cpp, keeps its own copy at its 54-column scrolling-log width, which ran on
// hardware as it is; whether Bench Tests joins the library is decided with the starter project.)
//
// NOT one of the frozen F4 ten — an ADDITIVE device seam, like ILineDisplay (the controller's
// LCD). PROS-free, allocation-free. Contract: every call is synchronous and MUST NOT THROW — a
// screen has no fallback channel, and the serial log carries the same words.

#include <cstddef>
#include <cstdint>

namespace shulib::hal {

/// The three text sizes the brain prints (PROS's E_TEXT_SMALL / MEDIUM / LARGE). How many
/// characters each fits across the 480-pixel screen is the caller's budget, not this seam's:
/// calibration::budgetFor() states the library's ESTIMATES (A4 register HA-136).
enum class ScreenTextSize : std::uint8_t {
    Small,   ///< PROS E_TEXT_SMALL
    Medium,  ///< PROS E_TEXT_MEDIUM
    Large,   ///< PROS E_TEXT_LARGE
};

/// One read of the touch screen, reduced to what a release-edge reader needs. The screen is
/// RESISTIVE and PROS counts a touch on its RELEASE; x and y are where the LAST touch was and
/// are stale while nobody touches it.
struct ScreenTouch {
    /// The read FAILED (PROS answered E_TOUCH_ERROR, every field -1): never a tap.
    bool error = false;
    /// PROS's release_count: only ever increases, one per finger lifted. A reader acts on a
    /// change of it, never on its value.
    std::int32_t releaseCount = 0;
    /// Where the last touch was, in pixels from the left edge (stale while idle).
    std::int16_t x = 0;
    /// Where the last touch was, in pixels from the top edge (stale while idle).
    std::int16_t y = 0;
};

/// The buffer the shipped screens sanitize a line into, terminator included. Wider than any text
/// budget (SMALL, the widest, is 50 characters), so the budget — never this buffer — decides
/// where a line is cut.
inline constexpr std::size_t kScreenLineBytes = 128;

/// THE SCREEN'S SANITIZER (header): `text` copied into `out` (capacity `cap`, always terminated
/// when cap > 0) — '%' and control bytes (below 0x20, and 0x7F) become '?', a UTF-8 en or em dash
/// becomes '-', any other non-ASCII character becomes ONE '?' — cut at `cols` characters (and at
/// cap − 1), then padded with spaces to `padTo` characters, never past the cut. Returns the
/// characters written. A null `text` writes "". Never reads past `text`'s terminator.
inline std::size_t screenSafeLine(char* out, std::size_t cap, const char* text, std::size_t cols,
                                  std::size_t padTo = 0) noexcept {
    if (out == nullptr || cap == 0) {
        return 0;
    }
    const std::size_t limit = cols < cap - 1 ? cols : cap - 1;
    std::size_t k = 0;
    const auto* p = reinterpret_cast<const unsigned char*>(text != nullptr ? text : "");
    while (*p != 0 && k < limit) {
        const unsigned char c = *p;
        if (c >= 0x80) {
            // A UTF-8 sequence. p[1] is read only when p[0] is not the terminator and p[2] only
            // when p[1] is 0x80 — neither read passes the string's end.
            if (c == 0xE2 && p[1] == 0x80 && (p[2] == 0x93 || p[2] == 0x94)) {
                out[k++] = '-';  // an en or em dash
                p += 3;
                continue;
            }
            if (c >= 0xC0) {
                out[k++] = '?';  // a lead byte: one '?' for the whole character
            }
            ++p;  // a continuation byte (or the lead byte just replaced) is dropped
            continue;
        }
        out[k++] = (c < 0x20 || c == 0x7F || c == '%') ? '?' : static_cast<char>(c);
        ++p;
    }
    while (k < padTo && k < limit) {
        out[k++] = ' ';
    }
    out[k] = '\0';
    return k;
}

/// The brain's touch screen. Coordinates are pixels, corners INCLUSIVE (PROS's convention); colours
/// are 0xRRGGBB. Every call is synchronous and MUST NOT throw: a screen has no fallback channel,
/// and the serial log carries the same words. A painter holds one by reference and never owns it.
class IBrainScreen {
public:
    /// Polymorphic-base boilerplate: the destructor is virtual so deleting through an
    /// `IBrainScreen*` is well-defined. Nothing in this tree does — a screen is REFERENCED by its
    /// painter, never owned, so it must outlive every painter bound to it.
    virtual ~IBrainScreen() = default;
    /// Default-constructible: the base holds nothing.
    IBrainScreen() = default;
    /// Copyable at the base (it holds no state); each concrete screen decides for itself.
    IBrainScreen(const IBrainScreen&) = default;
    /// Movable, for the same reason.
    IBrainScreen(IBrainScreen&&) = default;
    /// Copy-assignable, for the same reason.
    IBrainScreen& operator=(const IBrainScreen&) = default;
    /// Move-assignable, for the same reason.
    IBrainScreen& operator=(IBrainScreen&&) = default;

    /// Fill the rectangle (x0, y0)–(x1, y1) with `rgb`.
    virtual void fillRect(std::int16_t x0, std::int16_t y0, std::int16_t x1, std::int16_t y1,
                          std::uint32_t rgb) = 0;
    /// Draw the rectangle's one-pixel outline in `rgb`.
    virtual void outlineRect(std::int16_t x0, std::int16_t y0, std::int16_t x1, std::int16_t y1,
                             std::uint32_t rgb) = 0;
    /// Erase the rectangle to `rgb` (the eraser colour).
    virtual void eraseRect(std::int16_t x0, std::int16_t y0, std::int16_t x1, std::int16_t y1,
                           std::uint32_t rgb) = 0;
    /// Erase the whole screen to `rgb`.
    virtual void eraseAll(std::uint32_t rgb) = 0;
    /// Print `text` at (x, y) in `size`, pen `pen` on `eraser` (VEXos draws text pen-on-ERASER,
    /// an opaque cell), AFTER screenSafeLine(text, cols, padTo) — every implementation sanitizes,
    /// so a caller can never hand VEXos a format character. The same default `padTo` on every
    /// override, so a call means the same through the base or a concrete type.
    virtual void print(ScreenTextSize size, std::int16_t x, std::int16_t y, std::uint32_t pen,
                       std::uint32_t eraser, const char* text, std::size_t cols,
                       std::size_t padTo = 0) = 0;
    /// One read of the touch status.
    [[nodiscard]] virtual ScreenTouch touch() = 0;
    /// Every print() through this object since it was made. PROS 4.2.2's print is BELIEVED to leak
    /// its formatted buffer on every call — R3d Part 2F's reading of the vendored libpros.a (no
    /// free found), never measured — so a program reports how many it has made.
    [[nodiscard]] virtual std::uint32_t prints() const = 0;
};

}  // namespace shulib::hal
