#pragma once
//
// ui::touch_panel — the GENERIC pieces of a brain-screen panel model (chunk COMP-a, 2026-09-26).
//
// ── Why this exists ─────────────────────────────────────────────────────────────────
// R3d Part 2F-1 built the first screen in this project whose decisions are host-tested
// (calibration/calibrate_panel.hpp): what it says, where each button is, which touches mean
// something. Most of that header is not about calibration at all — the text budgets, the
// rectangles and their padded hit boxes, the ASCII / no-'%' rule, the release-edge touch read,
// the arming delay after a layout change. The competition program's auton selector needs exactly
// those, and copying them would give the brain two sets of budgets that drift the day the SCREEN
// RULER is read. So they live here, button-id-agnostic, and competition::AutonMenu is built on
// them.
//
// ── calibrate_panel.hpp is NOT changed (COMP brief ruling C4) ───────────────────────
// The pending bench session's Calibrate runs on it, and a refactor under a program that is about
// to go to the robot is the wrong trade. Moving it onto this header is a later, separate change.
// Until then the two sets of budgets are held EQUAL by static_assert in the host suite
// (test/ui_touch_panel_test.cpp — ruling B20: one set of budgets), so changing one without the
// other fails the build, naming the constant.
//   Rejected: the static_assert here or in auton_menu.hpp. It would make every competition
// header include the whole drivetrain calibrator (calibrate_panel.hpp pulls it in) to compare
// seven numbers; the host suite already includes both, and it runs on every build and in CI.
//
// ── EVERY BUDGET IS AN ESTIMATE (A4: HA-136) ────────────────────────────────────────
// LARGE 18, MEDIUM 36, SMALL 50 characters per 472-pixel line; the line heights; the hit pads;
// the arming delay. None has been measured on a V5 (Bench Tests button 9 is the screen ruler).
// HA-136 is the entry that owns them; it names calibrate_panel.hpp, and these are its twins.
//
// ── ASCII ONLY, AND NEVER A PERCENT SIGN ────────────────────────────────────────────
// PROS 4.2.2 hands printed text to VEXos AS A FORMAT STRING: a '%' on screen is re-read as a
// conversion with no argument behind it, and the fonts carry printable ASCII only. screenText()
// is the one test for both, usable in a static_assert on a literal and at run time on a name.
//
// PROS-free, allocation-free, header-only; nothing here draws, reads a touch, or knows a clock —
// the program does all three and hands the results in. NOTHING HERE HAS RUN ON A ROBOT.

#include <cstddef>
#include <cstdint>
#include <string_view>

namespace shulib::ui {

// ── budgets (ESTIMATES until the SCREEN RULER is read — one constant each; A4: HA-136) ──

/// Characters per line in the LARGE size. ESTIMATE, never measured on a V5 (A4: HA-136).
inline constexpr std::size_t kLargeCols = 18;
/// Characters per line in the MEDIUM size. ESTIMATE (A4: HA-136).
inline constexpr std::size_t kMediumCols = 36;
/// Characters per line in the SMALL size. ESTIMATE (A4: HA-136).
inline constexpr std::size_t kSmallCols = 50;

/// The height a SMALL line is laid out for, pixels. ESTIMATE (A4: HA-136).
inline constexpr std::int16_t kSmallHeight = 16;
/// The height a MEDIUM line is laid out for, pixels. ESTIMATE (A4: HA-136).
inline constexpr std::int16_t kMediumHeight = 20;
/// The height a LARGE line is laid out for, pixels. ESTIMATE (A4: HA-136).
inline constexpr std::int16_t kLargeHeight = 34;

// ── geometry (480 x 240 usable; every pixel at y < 236) ────────────────────────────

/// The screen's usable width, pixels.
inline constexpr std::int16_t kPanelWidth = 480;
/// The lowest row anything may be drawn on: VEXos keeps the rows below.
inline constexpr std::int16_t kPanelLastRow = 235;
/// How far a touch may land outside a button, left/right, and still hit it — a resistive screen's
/// release point wanders. A layout keeps every horizontal gap between two buttons above twice
/// this, so a pad never makes two buttons overlap. ESTIMATE (A4: HA-136).
inline constexpr std::int16_t kHitPadX = 4;
/// How far a touch may land above or below a button and still hit it. A layout keeps every
/// vertical gap between two buttons above twice this. ESTIMATE (A4: HA-136).
inline constexpr std::int16_t kHitPadY = 10;
/// How long a new layout's buttons stay greyed and dead: long enough that a reflexive second tap
/// lands on nothing, short enough not to feel broken. ESTIMATE (A4: HA-136).
inline constexpr double kArmingDelaySeconds = 0.5;

/// A rectangle in screen pixels with INCLUSIVE corners — PROS's own drawing convention.
struct Rect {
    std::int16_t x0 = 0;  ///< left column
    std::int16_t y0 = 0;  ///< top row
    std::int16_t x1 = 0;  ///< right column (inclusive)
    std::int16_t y1 = 0;  ///< bottom row (inclusive)
    /// Equal corners.
    friend constexpr bool operator==(const Rect&, const Rect&) = default;
};

/// The rectangle a touch must land in to hit a button drawn at `r`: padded kHitPadX sideways and
/// kHitPadY up and down.
[[nodiscard]] constexpr Rect hitBox(Rect r) noexcept {
    return Rect{static_cast<std::int16_t>(r.x0 - kHitPadX), static_cast<std::int16_t>(r.y0 - kHitPadY),
                static_cast<std::int16_t>(r.x1 + kHitPadX), static_cast<std::int16_t>(r.y1 + kHitPadY)};
}

/// True when (x, y) lies inside `r`, edges included.
[[nodiscard]] constexpr bool contains(Rect r, int x, int y) noexcept {
    return x >= r.x0 && x <= r.x1 && y >= r.y0 && y <= r.y1;
}

/// True when `a` and `b` share at least one pixel.
[[nodiscard]] constexpr bool overlaps(Rect a, Rect b) noexcept {
    return a.x0 <= b.x1 && b.x0 <= a.x1 && a.y0 <= b.y1 && b.y0 <= a.y1;
}

/// True when `inner` lies wholly inside `outer`.
[[nodiscard]] constexpr bool within(Rect inner, Rect outer) noexcept {
    return inner.x0 >= outer.x0 && inner.x1 <= outer.x1 && inner.y0 >= outer.y0
           && inner.y1 <= outer.y1;
}

/// True when `r` is drawable: corners ordered, on the screen, and above the rows VEXos keeps.
[[nodiscard]] constexpr bool onScreen(Rect r) noexcept {
    return r.x0 >= 0 && r.y0 >= 0 && r.x0 <= r.x1 && r.y0 <= r.y1 && r.x1 < kPanelWidth
           && r.y1 <= kPanelLastRow;
}

/// The three text sizes PROS offers.
enum class TextSize : std::uint8_t {
    Small,   ///< kSmallCols per line
    Medium,  ///< kMediumCols per line
    Large,   ///< kLargeCols per line
};

/// The budget, in characters, of one line of `size`.
[[nodiscard]] constexpr std::size_t budgetFor(TextSize size) noexcept {
    return size == TextSize::Large ? kLargeCols : size == TextSize::Medium ? kMediumCols : kSmallCols;
}

/// The height a line of `size` is laid out for, pixels (an ESTIMATE).
[[nodiscard]] constexpr std::int16_t heightFor(TextSize size) noexcept {
    return size == TextSize::Large ? kLargeHeight
           : size == TextSize::Medium ? kMediumHeight
                                      : kSmallHeight;
}

/// A colour by MEANING; the program maps each to RGB.
enum class Tone : std::uint8_t {
    Blue,   ///< waiting / neutral (a header)
    Amber,  ///< attention — something is about to happen, or "not yet"
    Green,  ///< chosen / ready / good
    Red,    ///< stopped / refused
    Grey,   ///< a greyed (dead) button, or a quiet line
    White,  ///< ordinary text
};

/// True when `s` may be drawn in a line of `cols` characters: printable ASCII only (0x20..0x7E),
/// no '%', and no longer than `cols`. The one test for every literal (in a static_assert) and every
/// runtime string (a registered name) that reaches the screen.
[[nodiscard]] constexpr bool screenText(std::string_view s, std::size_t cols) noexcept {
    if (s.size() > cols) {
        return false;
    }
    for (const char c : s) {
        const auto u = static_cast<unsigned char>(c);
        if (u < 0x20 || u > 0x7E || c == '%') {
            return false;
        }
    }
    return true;
}

/// Copy `s` into `out` (capacity `cap`, always NUL-terminated when cap > 0), stopping at `cols`
/// characters and replacing anything screenText() would refuse with '?'. The run-time safety net
/// under a string that was already checked — a panel must never be the place a bad byte reaches
/// VEXos. Returns the number of characters written.
inline std::size_t copyScreenText(char* out, std::size_t cap, std::string_view s,
                                  std::size_t cols) noexcept {
    if (out == nullptr || cap == 0) {
        return 0;
    }
    std::size_t n = 0;
    for (const char c : s) {
        if (n >= cols || n + 1 >= cap) {
            break;
        }
        const auto u = static_cast<unsigned char>(c);
        out[n++] = (u < 0x20 || u > 0x7E || c == '%') ? '?' : c;
    }
    out[n] = '\0';
    return n;
}

/// A number that changes exactly when a LAYOUT does: FNV-1a over 32-bit values the model mixes in
/// (the screen, every button's identity and rectangle — never whether a button is armed, and never
/// text that changes while the buttons stay put). An ArmingClock restarts on a change.
class LayoutKey {
public:
    /// Mix one 32-bit value in, byte by byte.
    constexpr void mix(std::uint32_t value) noexcept {
        for (int k = 0; k < 4; ++k) {
            h_ ^= (value >> (8 * k)) & 0xFFu;
            h_ *= 16777619u;
        }
    }
    /// Mix a rectangle in (both corners).
    constexpr void mix(Rect r) noexcept {
        mix(static_cast<std::uint32_t>(static_cast<std::uint16_t>(r.x0))
            | (static_cast<std::uint32_t>(static_cast<std::uint16_t>(r.y0)) << 16));
        mix(static_cast<std::uint32_t>(static_cast<std::uint16_t>(r.x1))
            | (static_cast<std::uint32_t>(static_cast<std::uint16_t>(r.y1)) << 16));
    }
    /// The key so far.
    [[nodiscard]] constexpr std::uint32_t value() const noexcept { return h_; }

private:
    std::uint32_t h_ = 2166136261u;
};

/// The arming clock: tell it each frame's layout key and the time, and it says how long the layout
/// has been on the screen. A new layout's buttons are dead until kArmingDelaySeconds have passed.
class ArmingClock {
public:
    /// Report the layout about to be painted at `nowSeconds`; returns the seconds it has been on
    /// the screen — 0 on the first call and on every change of key.
    double observe(std::uint32_t layoutKey, double nowSeconds) noexcept {
        if (!haveKey_ || layoutKey != key_) {
            haveKey_ = true;
            key_ = layoutKey;
            changedAt_ = nowSeconds;
        }
        return nowSeconds - changedAt_;
    }
    /// True when a layout that has been up `sinceChange` seconds may take touches.
    [[nodiscard]] static constexpr bool armedAfter(double sinceChange) noexcept {
        return sinceChange >= kArmingDelaySeconds;
    }

private:
    std::uint32_t key_ = 0;
    bool haveKey_ = false;
    double changedAt_ = 0.0;
};

/// One read of the touch screen, PROS-free: what screen_touch_status() returns, reduced to what a
/// release-edge reader needs.
struct TouchSample {
    /// The read FAILED (PROS: E_TOUCH_ERROR, every field -1). Ignored — and it must not move the
    /// release count, or the next good read would replay the last touch.
    bool error = false;
    /// PROS's release_count: it only ever increases, one per finger lifted.
    std::int32_t releaseCount = 0;
    /// Where the LAST touch was — stale while idle, so it means something only on a new release.
    std::int16_t x = 0;
    /// See x.
    std::int16_t y = 0;
};

/// The release EDGE, in one place: a sample means something only when it carries a release this
/// reader has not seen; an error read is ignored and never moves the count (so it cannot replay).
class ReleaseEdge {
public:
    /// `releaseCountAtStart`: PROS's release_count when the panel first appears, so a tap made
    /// before the program existed is never replayed.
    explicit ReleaseEdge(std::int32_t releaseCountAtStart) noexcept : last_{releaseCountAtStart} {}

    /// True when `s` carries a NEW release (its x/y then describe it); false for an error read or
    /// a stale sample.
    [[nodiscard]] bool take(const TouchSample& s) noexcept {
        if (s.error || s.releaseCount == last_) {
            return false;
        }
        last_ = s.releaseCount;
        return true;
    }

private:
    std::int32_t last_;
};

/// The index of the first ARMED button in `buttons[0..count)` whose hit box contains (x, y), or -1.
/// Button-id-agnostic: any type with `.rect` (a Rect) and `.armed` (a bool) will do.
template <typename Button>
[[nodiscard]] constexpr int hitTest(const Button* buttons, std::size_t count, int x, int y) noexcept {
    for (std::size_t i = 0; i < count; ++i) {
        if (buttons[i].armed && contains(hitBox(buttons[i].rect), x, y)) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

}  // namespace shulib::ui
