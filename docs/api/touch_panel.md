<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/ui/touch_panel.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `touch_panel.hpp`

ui::touch_panel — the GENERIC pieces of a brain-screen panel model.

This header declares **7** types (25 members), **10** free functions, and **11** constants.

Extracted from [`include/shulib/ui/touch_panel.hpp`](../../include/shulib/ui/touch_panel.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`kLargeCols`](#klargecols) — *constant*
- [`kMediumCols`](#kmediumcols) — *constant*
- [`kSmallCols`](#ksmallcols) — *constant*
- [`kSmallHeight`](#ksmallheight) — *constant*
- [`kMediumHeight`](#kmediumheight) — *constant*
- [`kLargeHeight`](#klargeheight) — *constant*
- [`kPanelWidth`](#kpanelwidth) — *constant*
- [`kPanelLastRow`](#kpanellastrow) — *constant*
- [`kHitPadX`](#khitpadx) — *constant*
- [`kHitPadY`](#khitpady) — *constant*
- [`kArmingDelaySeconds`](#karmingdelayseconds) — *constant*
- [`struct Rect`](#struct-rect)
  - [`x0`](#rect-x0)
  - [`y0`](#rect-y0)
  - [`x1`](#rect-x1)
  - [`y1`](#rect-y1)
  - [`operator==`](#rect-operator-eq-eq)
- [`hitBox`](#hitbox) — *free function*
- [`contains`](#contains) — *free function*
- [`overlaps`](#overlaps) — *free function*
- [`within`](#within) — *free function*
- [`onScreen`](#onscreen) — *free function*
- [`enum class TextSize`](#enum-class-textsize)
  - [`Small`](#textsize-small)
  - [`Medium`](#textsize-medium)
  - [`Large`](#textsize-large)
- [`budgetFor`](#budgetfor) — *free function*
- [`heightFor`](#heightfor) — *free function*
- [`enum class Tone`](#enum-class-tone)
  - [`Blue`](#tone-blue)
  - [`Amber`](#tone-amber)
  - [`Green`](#tone-green)
  - [`Red`](#tone-red)
  - [`Grey`](#tone-grey)
  - [`White`](#tone-white)
- [`screenText`](#screentext) — *free function*
- [`copyScreenText`](#copyscreentext) — *free function*
- [`class LayoutKey`](#class-layoutkey)
  - [`mix`](#layoutkey-mix)
  - [`mix (overload 2)`](#layoutkey-mix-2)
  - [`value`](#layoutkey-value)
- [`class ArmingClock`](#class-armingclock)
  - [`observe`](#armingclock-observe)
  - [`armedAfter`](#armingclock-armedafter)
- [`struct TouchSample`](#struct-touchsample)
  - [`error`](#touchsample-error)
  - [`releaseCount`](#touchsample-releasecount)
  - [`x`](#touchsample-x)
  - [`y`](#touchsample-y)
- [`class ReleaseEdge`](#class-releaseedge)
  - [`ReleaseEdge`](#releaseedge-releaseedge)
  - [`take`](#releaseedge-take)
- [`hitTest`](#hittest) — *free function*

<a id="klargecols"></a>

## `kLargeCols`

```cpp
inline constexpr std::size_t kLargeCols = 18
```

Characters per line in the LARGE size. ESTIMATE, never measured on a V5 (A4: HA-136).

*constant, declared at [`include/shulib/ui/touch_panel.hpp:47`](../../include/shulib/ui/touch_panel.hpp#L47).*

<a id="kmediumcols"></a>

## `kMediumCols`

```cpp
inline constexpr std::size_t kMediumCols = 36
```

Characters per line in the MEDIUM size. ESTIMATE (A4: HA-136).

*constant, declared at [`include/shulib/ui/touch_panel.hpp:49`](../../include/shulib/ui/touch_panel.hpp#L49).*

<a id="ksmallcols"></a>

## `kSmallCols`

```cpp
inline constexpr std::size_t kSmallCols = 50
```

Characters per line in the SMALL size. ESTIMATE (A4: HA-136).

*constant, declared at [`include/shulib/ui/touch_panel.hpp:51`](../../include/shulib/ui/touch_panel.hpp#L51).*

<a id="ksmallheight"></a>

## `kSmallHeight`

```cpp
inline constexpr std::int16_t kSmallHeight = 16
```

The height a SMALL line is laid out for, pixels. ESTIMATE (A4: HA-136).

*constant, declared at [`include/shulib/ui/touch_panel.hpp:54`](../../include/shulib/ui/touch_panel.hpp#L54).*

<a id="kmediumheight"></a>

## `kMediumHeight`

```cpp
inline constexpr std::int16_t kMediumHeight = 20
```

The height a MEDIUM line is laid out for, pixels. ESTIMATE (A4: HA-136).

*constant, declared at [`include/shulib/ui/touch_panel.hpp:56`](../../include/shulib/ui/touch_panel.hpp#L56).*

<a id="klargeheight"></a>

## `kLargeHeight`

```cpp
inline constexpr std::int16_t kLargeHeight = 34
```

The height a LARGE line is laid out for, pixels. ESTIMATE (A4: HA-136).

*constant, declared at [`include/shulib/ui/touch_panel.hpp:58`](../../include/shulib/ui/touch_panel.hpp#L58).*

<a id="kpanelwidth"></a>

## `kPanelWidth`

```cpp
inline constexpr std::int16_t kPanelWidth = 480
```

The screen's usable width, pixels.

*constant, declared at [`include/shulib/ui/touch_panel.hpp:63`](../../include/shulib/ui/touch_panel.hpp#L63).*

<a id="kpanellastrow"></a>

## `kPanelLastRow`

```cpp
inline constexpr std::int16_t kPanelLastRow = 235
```

The lowest row anything may be drawn on: VEXos keeps the rows below.

*constant, declared at [`include/shulib/ui/touch_panel.hpp:65`](../../include/shulib/ui/touch_panel.hpp#L65).*

<a id="khitpadx"></a>

## `kHitPadX`

```cpp
inline constexpr std::int16_t kHitPadX = 4
```

How far a touch may land outside a button, left/right, and still hit it — a resistive screen's release point wanders. A layout keeps every horizontal gap between two buttons above twice this, so a pad never makes two buttons overlap. ESTIMATE (A4: HA-136).

*constant, declared at [`include/shulib/ui/touch_panel.hpp:69`](../../include/shulib/ui/touch_panel.hpp#L69).*

<a id="khitpady"></a>

## `kHitPadY`

```cpp
inline constexpr std::int16_t kHitPadY = 10
```

How far a touch may land above or below a button and still hit it. A layout keeps every vertical gap between two buttons above twice this. ESTIMATE (A4: HA-136).

*constant, declared at [`include/shulib/ui/touch_panel.hpp:72`](../../include/shulib/ui/touch_panel.hpp#L72).*

<a id="karmingdelayseconds"></a>

## `kArmingDelaySeconds`

```cpp
inline constexpr double kArmingDelaySeconds = 0.5
```

How long a new layout's buttons stay greyed and dead: long enough that a reflexive second tap lands on nothing, short enough not to feel broken. ESTIMATE (A4: HA-136).

*constant, declared at [`include/shulib/ui/touch_panel.hpp:75`](../../include/shulib/ui/touch_panel.hpp#L75).*

<a id="struct-rect"></a>

## `struct Rect`

```cpp
struct Rect
```

A rectangle in screen pixels with INCLUSIVE corners — PROS's own drawing convention.

*struct, declared at [`include/shulib/ui/touch_panel.hpp:78`](../../include/shulib/ui/touch_panel.hpp#L78).*

<a id="rect-x0"></a>

### `Rect::x0`

```cpp
std::int16_t x0 = 0
```

left column

*field, declared at [`include/shulib/ui/touch_panel.hpp:79`](../../include/shulib/ui/touch_panel.hpp#L79).*

<a id="rect-y0"></a>

### `Rect::y0`

```cpp
std::int16_t y0 = 0
```

top row

*field, declared at [`include/shulib/ui/touch_panel.hpp:80`](../../include/shulib/ui/touch_panel.hpp#L80).*

<a id="rect-x1"></a>

### `Rect::x1`

```cpp
std::int16_t x1 = 0
```

right column (inclusive)

*field, declared at [`include/shulib/ui/touch_panel.hpp:81`](../../include/shulib/ui/touch_panel.hpp#L81).*

<a id="rect-y1"></a>

### `Rect::y1`

```cpp
std::int16_t y1 = 0
```

bottom row (inclusive)

*field, declared at [`include/shulib/ui/touch_panel.hpp:82`](../../include/shulib/ui/touch_panel.hpp#L82).*

<a id="rect-operator-eq-eq"></a>

### `Rect::operator==`

```cpp
friend constexpr bool operator==(const Rect&, const Rect&) = default
```

Equal corners.

*function, declared at [`include/shulib/ui/touch_panel.hpp:84`](../../include/shulib/ui/touch_panel.hpp#L84).*

<a id="hitbox"></a>

## `hitBox`

```cpp
[[nodiscard]] constexpr Rect hitBox(Rect r) noexcept
```

The rectangle a touch must land in to hit a button drawn at `r`: padded kHitPadX sideways and kHitPadY up and down.

*free function, declared at [`include/shulib/ui/touch_panel.hpp:89`](../../include/shulib/ui/touch_panel.hpp#L89).*

<a id="contains"></a>

## `contains`

```cpp
[[nodiscard]] constexpr bool contains(Rect r, int x, int y) noexcept
```

True when (x, y) lies inside `r`, edges included.

*free function, declared at [`include/shulib/ui/touch_panel.hpp:95`](../../include/shulib/ui/touch_panel.hpp#L95).*

<a id="overlaps"></a>

## `overlaps`

```cpp
[[nodiscard]] constexpr bool overlaps(Rect a, Rect b) noexcept
```

True when `a` and `b` share at least one pixel.

*free function, declared at [`include/shulib/ui/touch_panel.hpp:100`](../../include/shulib/ui/touch_panel.hpp#L100).*

<a id="within"></a>

## `within`

```cpp
[[nodiscard]] constexpr bool within(Rect inner, Rect outer) noexcept
```

True when `inner` lies wholly inside `outer`.

*free function, declared at [`include/shulib/ui/touch_panel.hpp:105`](../../include/shulib/ui/touch_panel.hpp#L105).*

<a id="onscreen"></a>

## `onScreen`

```cpp
[[nodiscard]] constexpr bool onScreen(Rect r) noexcept
```

True when `r` is drawable: corners ordered, on the screen, and above the rows VEXos keeps.

*free function, declared at [`include/shulib/ui/touch_panel.hpp:111`](../../include/shulib/ui/touch_panel.hpp#L111).*

<a id="enum-class-textsize"></a>

## `enum class TextSize`

```cpp
enum class TextSize : std::uint8_t
```

The three text sizes PROS offers.

*enum class, declared at [`include/shulib/ui/touch_panel.hpp:117`](../../include/shulib/ui/touch_panel.hpp#L117).*

<a id="textsize-small"></a>

### `TextSize::Small`

```cpp
Small
```

kSmallCols per line

*enumerator, declared at [`include/shulib/ui/touch_panel.hpp:118`](../../include/shulib/ui/touch_panel.hpp#L118).*

<a id="textsize-medium"></a>

### `TextSize::Medium`

```cpp
Medium
```

kMediumCols per line

*enumerator, declared at [`include/shulib/ui/touch_panel.hpp:119`](../../include/shulib/ui/touch_panel.hpp#L119).*

<a id="textsize-large"></a>

### `TextSize::Large`

```cpp
Large
```

kLargeCols per line

*enumerator, declared at [`include/shulib/ui/touch_panel.hpp:120`](../../include/shulib/ui/touch_panel.hpp#L120).*

<a id="budgetfor"></a>

## `budgetFor`

```cpp
[[nodiscard]] constexpr std::size_t budgetFor(TextSize size) noexcept
```

The budget, in characters, of one line of `size`.

*free function, declared at [`include/shulib/ui/touch_panel.hpp:124`](../../include/shulib/ui/touch_panel.hpp#L124).*

<a id="heightfor"></a>

## `heightFor`

```cpp
[[nodiscard]] constexpr std::int16_t heightFor(TextSize size) noexcept
```

The height a line of `size` is laid out for, pixels (an ESTIMATE).

*free function, declared at [`include/shulib/ui/touch_panel.hpp:129`](../../include/shulib/ui/touch_panel.hpp#L129).*

<a id="enum-class-tone"></a>

## `enum class Tone`

```cpp
enum class Tone : std::uint8_t
```

A colour by MEANING; the program maps each to RGB.

*enum class, declared at [`include/shulib/ui/touch_panel.hpp:136`](../../include/shulib/ui/touch_panel.hpp#L136).*

<a id="tone-blue"></a>

### `Tone::Blue`

```cpp
Blue
```

waiting / neutral (a header)

*enumerator, declared at [`include/shulib/ui/touch_panel.hpp:137`](../../include/shulib/ui/touch_panel.hpp#L137).*

<a id="tone-amber"></a>

### `Tone::Amber`

```cpp
Amber
```

attention — something is about to happen, or "not yet"

*enumerator, declared at [`include/shulib/ui/touch_panel.hpp:138`](../../include/shulib/ui/touch_panel.hpp#L138).*

<a id="tone-green"></a>

### `Tone::Green`

```cpp
Green
```

chosen / ready / good

*enumerator, declared at [`include/shulib/ui/touch_panel.hpp:139`](../../include/shulib/ui/touch_panel.hpp#L139).*

<a id="tone-red"></a>

### `Tone::Red`

```cpp
Red
```

stopped / refused

*enumerator, declared at [`include/shulib/ui/touch_panel.hpp:140`](../../include/shulib/ui/touch_panel.hpp#L140).*

<a id="tone-grey"></a>

### `Tone::Grey`

```cpp
Grey
```

a greyed (dead) button, or a quiet line

*enumerator, declared at [`include/shulib/ui/touch_panel.hpp:141`](../../include/shulib/ui/touch_panel.hpp#L141).*

<a id="tone-white"></a>

### `Tone::White`

```cpp
White
```

ordinary text

*enumerator, declared at [`include/shulib/ui/touch_panel.hpp:142`](../../include/shulib/ui/touch_panel.hpp#L142).*

<a id="screentext"></a>

## `screenText`

```cpp
[[nodiscard]] constexpr bool screenText(std::string_view s, std::size_t cols) noexcept
```

True when `s` may be drawn in a line of `cols` characters: printable ASCII only (0x20..0x7E), no '%', and no longer than `cols`. The one test for every literal (in a static_assert) and every runtime string (a registered name) that reaches the screen.

*free function, declared at [`include/shulib/ui/touch_panel.hpp:148`](../../include/shulib/ui/touch_panel.hpp#L148).*

<a id="copyscreentext"></a>

## `copyScreenText`

```cpp
inline std::size_t copyScreenText(char* out, std::size_t cap, std::string_view s, std::size_t cols) noexcept
```

Copy `s` into `out` (capacity `cap`, always NUL-terminated when cap > 0), stopping at `cols` characters and replacing anything screenText() would refuse with '?'. The run-time safety net under a string that was already checked — a panel must never be the place a bad byte reaches VEXos. Returns the number of characters written.

*free function, declared at [`include/shulib/ui/touch_panel.hpp:165`](../../include/shulib/ui/touch_panel.hpp#L165).*

<a id="class-layoutkey"></a>

## `class LayoutKey`

```cpp
class LayoutKey
```

A number that changes exactly when a LAYOUT does: FNV-1a over 32-bit values the model mixes in (the screen, every button's identity and rectangle — never whether a button is armed, and never text that changes while the buttons stay put). An ArmingClock restarts on a change.

*class, declared at [`include/shulib/ui/touch_panel.hpp:185`](../../include/shulib/ui/touch_panel.hpp#L185).*

<a id="layoutkey-mix"></a>

### `LayoutKey::mix`

```cpp
constexpr void mix(std::uint32_t value) noexcept
```

Mix one 32-bit value in, byte by byte.

*function, declared at [`include/shulib/ui/touch_panel.hpp:188`](../../include/shulib/ui/touch_panel.hpp#L188).*

<a id="layoutkey-mix-2"></a>

### `LayoutKey::mix (overload 2)`

```cpp
constexpr void mix(Rect r) noexcept
```

Mix a rectangle in (both corners).

*function, declared at [`include/shulib/ui/touch_panel.hpp:195`](../../include/shulib/ui/touch_panel.hpp#L195).*

<a id="layoutkey-value"></a>

### `LayoutKey::value`

```cpp
[[nodiscard]] constexpr std::uint32_t value() const noexcept
```

The key so far.

*function, declared at [`include/shulib/ui/touch_panel.hpp:202`](../../include/shulib/ui/touch_panel.hpp#L202).*

<a id="class-armingclock"></a>

## `class ArmingClock`

```cpp
class ArmingClock
```

The arming clock: tell it each frame's layout key and the time, and it says how long the layout has been on the screen. A new layout's buttons are dead until kArmingDelaySeconds have passed.

*class, declared at [`include/shulib/ui/touch_panel.hpp:210`](../../include/shulib/ui/touch_panel.hpp#L210).*

<a id="armingclock-observe"></a>

### `ArmingClock::observe`

```cpp
double observe(std::uint32_t layoutKey, double nowSeconds) noexcept
```

Report the layout about to be painted at `nowSeconds`; returns the seconds it has been on the screen — 0 on the first call and on every change of key.

*function, declared at [`include/shulib/ui/touch_panel.hpp:214`](../../include/shulib/ui/touch_panel.hpp#L214).*

<a id="armingclock-armedafter"></a>

### `ArmingClock::armedAfter`

```cpp
[[nodiscard]] static constexpr bool armedAfter(double sinceChange) noexcept
```

True when a layout that has been up `sinceChange` seconds may take touches.

*function, declared at [`include/shulib/ui/touch_panel.hpp:223`](../../include/shulib/ui/touch_panel.hpp#L223).*

<a id="struct-touchsample"></a>

## `struct TouchSample`

```cpp
struct TouchSample
```

One read of the touch screen, PROS-free: what screen_touch_status() returns, reduced to what a release-edge reader needs.

*struct, declared at [`include/shulib/ui/touch_panel.hpp:235`](../../include/shulib/ui/touch_panel.hpp#L235).*

<a id="touchsample-error"></a>

### `TouchSample::error`

```cpp
bool error = false
```

The read FAILED (PROS: E_TOUCH_ERROR, every field -1). Ignored — and it must not move the release count, or the next good read would replay the last touch.

*field, declared at [`include/shulib/ui/touch_panel.hpp:238`](../../include/shulib/ui/touch_panel.hpp#L238).*

<a id="touchsample-releasecount"></a>

### `TouchSample::releaseCount`

```cpp
std::int32_t releaseCount = 0
```

PROS's release_count: it only ever increases, one per finger lifted.

*field, declared at [`include/shulib/ui/touch_panel.hpp:240`](../../include/shulib/ui/touch_panel.hpp#L240).*

<a id="touchsample-x"></a>

### `TouchSample::x`

```cpp
std::int16_t x = 0
```

Where the LAST touch was — stale while idle, so it means something only on a new release.

*field, declared at [`include/shulib/ui/touch_panel.hpp:242`](../../include/shulib/ui/touch_panel.hpp#L242).*

<a id="touchsample-y"></a>

### `TouchSample::y`

```cpp
std::int16_t y = 0
```

See x.

*field, declared at [`include/shulib/ui/touch_panel.hpp:244`](../../include/shulib/ui/touch_panel.hpp#L244).*

<a id="class-releaseedge"></a>

## `class ReleaseEdge`

```cpp
class ReleaseEdge
```

The release EDGE, in one place: a sample means something only when it carries a release this reader has not seen; an error read is ignored and never moves the count (so it cannot replay).

*class, declared at [`include/shulib/ui/touch_panel.hpp:249`](../../include/shulib/ui/touch_panel.hpp#L249).*

<a id="releaseedge-releaseedge"></a>

### `ReleaseEdge::ReleaseEdge`

```cpp
explicit ReleaseEdge(std::int32_t releaseCountAtStart) noexcept
```

`releaseCountAtStart`: PROS's release_count when the panel first appears, so a tap made before the program existed is never replayed.

*function, declared at [`include/shulib/ui/touch_panel.hpp:253`](../../include/shulib/ui/touch_panel.hpp#L253).*

<a id="releaseedge-take"></a>

### `ReleaseEdge::take`

```cpp
[[nodiscard]] bool take(const TouchSample& s) noexcept
```

True when `s` carries a NEW release (its x/y then describe it); false for an error read or a stale sample.

*function, declared at [`include/shulib/ui/touch_panel.hpp:257`](../../include/shulib/ui/touch_panel.hpp#L257).*

<a id="hittest"></a>

## `hitTest`

```cpp
template <typename Button> [[nodiscard]] constexpr int hitTest(const Button* buttons, std::size_t count, int x, int y) noexcept
```

The index of the first ARMED button in `buttons[0..count)` whose hit box contains (x, y), or -1. Button-id-agnostic: any type with `.rect` (a Rect) and `.armed` (a bool) will do.

*free function, declared at [`include/shulib/ui/touch_panel.hpp:272`](../../include/shulib/ui/touch_panel.hpp#L272).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 35 lines</summary>

```text

 ui::touch_panel — the GENERIC pieces of a brain-screen panel model (chunk COMP-a, 2026-09-26).

 ── Why this exists ─────────────────────────────────────────────────────────────────
 R3d Part 2F-1 built the first screen in this project whose decisions are host-tested
 (calibration/calibrate_panel.hpp): what it says, where each button is, which touches mean
 something. Most of that header is not about calibration at all — the text budgets, the
 rectangles and their padded hit boxes, the ASCII / no-'%' rule, the release-edge touch read,
 the arming delay after a layout change. The competition program's auton selector needs exactly
 those, and copying them would give the brain two sets of budgets that drift the day the SCREEN
 RULER is read. So they live here, button-id-agnostic, and competition::AutonMenu is built on
 them.

 ── calibrate_panel.hpp is NOT changed (COMP brief ruling C4) ───────────────────────
 The pending bench session's Calibrate runs on it, and a refactor under a program that is about
 to go to the robot is the wrong trade. Moving it onto this header is a later, separate change.
 Until then the two sets of budgets are held EQUAL by static_assert in the host suite
 (test/ui_touch_panel_test.cpp — ruling B20: one set of budgets), so changing one without the
 other fails the build, naming the constant.
   Rejected: the static_assert here or in auton_menu.hpp. It would make every competition
 header include the whole drivetrain calibrator (calibrate_panel.hpp pulls it in) to compare
 seven numbers; the host suite already includes both, and it runs on every build and in CI.

 ── EVERY BUDGET IS AN ESTIMATE (A4: HA-136) ────────────────────────────────────────
 LARGE 18, MEDIUM 36, SMALL 50 characters per 472-pixel line; the line heights; the hit pads;
 the arming delay. None has been measured on a V5 (Bench Tests button 9 is the screen ruler).
 HA-136 is the entry that owns them; it names calibrate_panel.hpp, and these are its twins.

 ── ASCII ONLY, AND NEVER A PERCENT SIGN ────────────────────────────────────────────
 PROS 4.2.2 hands printed text to VEXos AS A FORMAT STRING: a '%' on screen is re-read as a
 conversion with no argument behind it, and the fonts carry printable ASCII only. screenText()
 is the one test for both, usable in a static_assert on a literal and at run time on a name.

 PROS-free, allocation-free, header-only; nothing here draws, reads a touch, or knows a clock —
 the program does all three and hands the results in. NOTHING HERE HAS RUN ON A ROBOT.
```

</details>
