<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/brain_screen.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `brain_screen.hpp`

IBrainScreen — the V5 BRAIN's own touch screen, as a seam.

This header declares **3** types (20 members), **1** free function, and **1** constant.

Extracted from [`include/shulib/hal/brain_screen.hpp`](../../include/shulib/hal/brain_screen.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in [How it works](../guide/README.md); worked recipes live in [Recipes](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`enum class ScreenTextSize`](#enum-class-screentextsize)
  - [`Small`](#screentextsize-small)
  - [`Medium`](#screentextsize-medium)
  - [`Large`](#screentextsize-large)
- [`struct ScreenTouch`](#struct-screentouch)
  - [`error`](#screentouch-error)
  - [`releaseCount`](#screentouch-releasecount)
  - [`x`](#screentouch-x)
  - [`y`](#screentouch-y)
- [`kScreenLineBytes`](#kscreenlinebytes) — *constant*
- [`screenSafeLine`](#screensafeline) — *free function*
- [`class IBrainScreen`](#class-ibrainscreen)
  - [`~IBrainScreen`](#ibrainscreen-destructor-ibrainscreen)
  - [`IBrainScreen`](#ibrainscreen-ibrainscreen)
  - [`IBrainScreen (overload 2)`](#ibrainscreen-ibrainscreen-2)
  - [`IBrainScreen (overload 3)`](#ibrainscreen-ibrainscreen-3)
  - [`operator=`](#ibrainscreen-operator-eq)
  - [`operator= (overload 2)`](#ibrainscreen-operator-eq-2)
  - [`fillRect`](#ibrainscreen-fillrect)
  - [`outlineRect`](#ibrainscreen-outlinerect)
  - [`eraseRect`](#ibrainscreen-eraserect)
  - [`eraseAll`](#ibrainscreen-eraseall)
  - [`print`](#ibrainscreen-print)
  - [`touch`](#ibrainscreen-touch)
  - [`prints`](#ibrainscreen-prints)

<a id="enum-class-screentextsize"></a>

## `enum class ScreenTextSize`

```cpp
enum class ScreenTextSize : std::uint8_t
```

The three text sizes the brain prints (PROS's E_TEXT_SMALL / MEDIUM / LARGE). How many characters each fits across the 480-pixel screen is the caller's budget, not this seam's: calibration::budgetFor() states the library's ESTIMATES (A4 register HA-136).

*enum class, declared at [`include/shulib/hal/brain_screen.hpp:41`](../../include/shulib/hal/brain_screen.hpp#L41).*

<a id="screentextsize-small"></a>

### `ScreenTextSize::Small`

```cpp
Small
```

PROS E_TEXT_SMALL

*enumerator, declared at [`include/shulib/hal/brain_screen.hpp:42`](../../include/shulib/hal/brain_screen.hpp#L42).*

<a id="screentextsize-medium"></a>

### `ScreenTextSize::Medium`

```cpp
Medium
```

PROS E_TEXT_MEDIUM

*enumerator, declared at [`include/shulib/hal/brain_screen.hpp:43`](../../include/shulib/hal/brain_screen.hpp#L43).*

<a id="screentextsize-large"></a>

### `ScreenTextSize::Large`

```cpp
Large
```

PROS E_TEXT_LARGE

*enumerator, declared at [`include/shulib/hal/brain_screen.hpp:44`](../../include/shulib/hal/brain_screen.hpp#L44).*

<a id="struct-screentouch"></a>

## `struct ScreenTouch`

```cpp
struct ScreenTouch
```

One read of the touch screen, reduced to what a release-edge reader needs. The screen is RESISTIVE and PROS counts a touch on its RELEASE; x and y are where the LAST touch was and are stale while nobody touches it.

*struct, declared at [`include/shulib/hal/brain_screen.hpp:50`](../../include/shulib/hal/brain_screen.hpp#L50).*

<a id="screentouch-error"></a>

### `ScreenTouch::error`

```cpp
bool error = false
```

The read FAILED (PROS answered E_TOUCH_ERROR, every field -1): never a tap.

*field, declared at [`include/shulib/hal/brain_screen.hpp:52`](../../include/shulib/hal/brain_screen.hpp#L52).*

<a id="screentouch-releasecount"></a>

### `ScreenTouch::releaseCount`

```cpp
std::int32_t releaseCount = 0
```

PROS's release_count: only ever increases, one per finger lifted. A reader acts on a change of it, never on its value.

*field, declared at [`include/shulib/hal/brain_screen.hpp:55`](../../include/shulib/hal/brain_screen.hpp#L55).*

<a id="screentouch-x"></a>

### `ScreenTouch::x`

```cpp
std::int16_t x = 0
```

Where the last touch was, in pixels from the left edge (stale while idle).

*field, declared at [`include/shulib/hal/brain_screen.hpp:57`](../../include/shulib/hal/brain_screen.hpp#L57).*

<a id="screentouch-y"></a>

### `ScreenTouch::y`

```cpp
std::int16_t y = 0
```

Where the last touch was, in pixels from the top edge (stale while idle).

*field, declared at [`include/shulib/hal/brain_screen.hpp:59`](../../include/shulib/hal/brain_screen.hpp#L59).*

<a id="kscreenlinebytes"></a>

## `kScreenLineBytes`

```cpp
inline constexpr std::size_t kScreenLineBytes = 128
```

The buffer the shipped screens sanitize a line into, terminator included. Wider than any text budget (SMALL, the widest, is 50 characters), so the budget — never this buffer — decides where a line is cut.

*constant, declared at [`include/shulib/hal/brain_screen.hpp:65`](../../include/shulib/hal/brain_screen.hpp#L65).*

<a id="screensafeline"></a>

## `screenSafeLine`

```cpp
inline std::size_t screenSafeLine(char* out, std::size_t cap, const char* text, std::size_t cols, std::size_t padTo = 0) noexcept
```

THE SCREEN'S SANITIZER (header): `text` copied into `out` (capacity `cap`, always terminated when cap > 0) — '%' and control bytes (below 0x20, and 0x7F) become '?', a UTF-8 en or em dash becomes '-', any other non-ASCII character becomes ONE '?' — cut at `cols` characters (and at cap − 1), then padded with spaces to `padTo` characters, never past the cut. Returns the characters written. A null `text` writes "". Never reads past `text`'s terminator.

*free function, declared at [`include/shulib/hal/brain_screen.hpp:72`](../../include/shulib/hal/brain_screen.hpp#L72).*

<a id="class-ibrainscreen"></a>

## `class IBrainScreen`

```cpp
class IBrainScreen
```

The brain's touch screen. Coordinates are pixels, corners INCLUSIVE (PROS's convention); colours are 0xRRGGBB. Every call is synchronous and MUST NOT throw: a screen has no fallback channel, and the serial log carries the same words. A painter holds one by reference and never owns it.

*class, declared at [`include/shulib/hal/brain_screen.hpp:109`](../../include/shulib/hal/brain_screen.hpp#L109).*

<a id="ibrainscreen-destructor-ibrainscreen"></a>

### `IBrainScreen::~IBrainScreen`

```cpp
virtual ~IBrainScreen() = default
```

Polymorphic-base boilerplate: the destructor is virtual so deleting through an `IBrainScreen*` is well-defined. Nothing in this tree does — a screen is REFERENCED by its painter, never owned, so it must outlive every painter bound to it.

*function, declared at [`include/shulib/hal/brain_screen.hpp:114`](../../include/shulib/hal/brain_screen.hpp#L114).*

<a id="ibrainscreen-ibrainscreen"></a>

### `IBrainScreen::IBrainScreen`

```cpp
IBrainScreen() = default
```

Polymorphic-base boilerplate: the destructor is virtual so deleting through an `IBrainScreen*` is well-defined. Nothing in this tree does — a screen is REFERENCED by its painter, never owned, so it must outlive every painter bound to it. Default-constructible: the base holds nothing.

*function, declared at [`include/shulib/hal/brain_screen.hpp:116`](../../include/shulib/hal/brain_screen.hpp#L116).*

<a id="ibrainscreen-ibrainscreen-2"></a>

### `IBrainScreen::IBrainScreen (overload 2)`

```cpp
IBrainScreen(const IBrainScreen&) = default
```

Polymorphic-base boilerplate: the destructor is virtual so deleting through an `IBrainScreen*` is well-defined. Nothing in this tree does — a screen is REFERENCED by its painter, never owned, so it must outlive every painter bound to it. Default-constructible: the base holds nothing. Copyable at the base (it holds no state); each concrete screen decides for itself.

*function, declared at [`include/shulib/hal/brain_screen.hpp:118`](../../include/shulib/hal/brain_screen.hpp#L118).*

<a id="ibrainscreen-ibrainscreen-3"></a>

### `IBrainScreen::IBrainScreen (overload 3)`

```cpp
IBrainScreen(IBrainScreen&&) = default
```

Polymorphic-base boilerplate: the destructor is virtual so deleting through an `IBrainScreen*` is well-defined. Nothing in this tree does — a screen is REFERENCED by its painter, never owned, so it must outlive every painter bound to it. Default-constructible: the base holds nothing. Copyable at the base (it holds no state); each concrete screen decides for itself. Movable, for the same reason.

*function, declared at [`include/shulib/hal/brain_screen.hpp:120`](../../include/shulib/hal/brain_screen.hpp#L120).*

<a id="ibrainscreen-operator-eq"></a>

### `IBrainScreen::operator=`

```cpp
IBrainScreen& operator=(const IBrainScreen&) = default
```

Polymorphic-base boilerplate: the destructor is virtual so deleting through an `IBrainScreen*` is well-defined. Nothing in this tree does — a screen is REFERENCED by its painter, never owned, so it must outlive every painter bound to it. Default-constructible: the base holds nothing. Copyable at the base (it holds no state); each concrete screen decides for itself. Movable, for the same reason. Copy-assignable, for the same reason.

*function, declared at [`include/shulib/hal/brain_screen.hpp:122`](../../include/shulib/hal/brain_screen.hpp#L122).*

<a id="ibrainscreen-operator-eq-2"></a>

### `IBrainScreen::operator= (overload 2)`

```cpp
IBrainScreen& operator=(IBrainScreen&&) = default
```

Polymorphic-base boilerplate: the destructor is virtual so deleting through an `IBrainScreen*` is well-defined. Nothing in this tree does — a screen is REFERENCED by its painter, never owned, so it must outlive every painter bound to it. Default-constructible: the base holds nothing. Copyable at the base (it holds no state); each concrete screen decides for itself. Movable, for the same reason. Copy-assignable, for the same reason. Move-assignable, for the same reason.

*function, declared at [`include/shulib/hal/brain_screen.hpp:124`](../../include/shulib/hal/brain_screen.hpp#L124).*

<a id="ibrainscreen-fillrect"></a>

### `IBrainScreen::fillRect`

```cpp
virtual void fillRect(std::int16_t x0, std::int16_t y0, std::int16_t x1, std::int16_t y1, std::uint32_t rgb) = 0
```

Fill the rectangle (x0, y0)–(x1, y1) with `rgb`.

*function, declared at [`include/shulib/hal/brain_screen.hpp:127`](../../include/shulib/hal/brain_screen.hpp#L127).*

<a id="ibrainscreen-outlinerect"></a>

### `IBrainScreen::outlineRect`

```cpp
virtual void outlineRect(std::int16_t x0, std::int16_t y0, std::int16_t x1, std::int16_t y1, std::uint32_t rgb) = 0
```

Draw the rectangle's one-pixel outline in `rgb`.

*function, declared at [`include/shulib/hal/brain_screen.hpp:130`](../../include/shulib/hal/brain_screen.hpp#L130).*

<a id="ibrainscreen-eraserect"></a>

### `IBrainScreen::eraseRect`

```cpp
virtual void eraseRect(std::int16_t x0, std::int16_t y0, std::int16_t x1, std::int16_t y1, std::uint32_t rgb) = 0
```

Erase the rectangle to `rgb` (the eraser colour).

*function, declared at [`include/shulib/hal/brain_screen.hpp:133`](../../include/shulib/hal/brain_screen.hpp#L133).*

<a id="ibrainscreen-eraseall"></a>

### `IBrainScreen::eraseAll`

```cpp
virtual void eraseAll(std::uint32_t rgb) = 0
```

Erase the whole screen to `rgb`.

*function, declared at [`include/shulib/hal/brain_screen.hpp:136`](../../include/shulib/hal/brain_screen.hpp#L136).*

<a id="ibrainscreen-print"></a>

### `IBrainScreen::print`

```cpp
virtual void print(ScreenTextSize size, std::int16_t x, std::int16_t y, std::uint32_t pen, std::uint32_t eraser, const char* text, std::size_t cols, std::size_t padTo = 0) = 0
```

Print `text` at (x, y) in `size`, pen `pen` on `eraser` (VEXos draws text pen-on-ERASER, an opaque cell), AFTER screenSafeLine(text, cols, padTo) — every implementation sanitizes, so a caller can never hand VEXos a format character. The same default `padTo` on every override, so a call means the same through the base or a concrete type.

*function, declared at [`include/shulib/hal/brain_screen.hpp:141`](../../include/shulib/hal/brain_screen.hpp#L141).*

<a id="ibrainscreen-touch"></a>

### `IBrainScreen::touch`

```cpp
[[nodiscard]] virtual ScreenTouch touch() = 0
```

One read of the touch status.

*function, declared at [`include/shulib/hal/brain_screen.hpp:145`](../../include/shulib/hal/brain_screen.hpp#L145).*

<a id="ibrainscreen-prints"></a>

### `IBrainScreen::prints`

```cpp
[[nodiscard]] virtual std::uint32_t prints() const = 0
```

Every print() through this object since it was made. PROS 4.2.2's print is BELIEVED to leak its formatted buffer on every call — R3d Part 2F's reading of the vendored libpros.a (no free found), never measured — so a program reports how many it has made.

*function, declared at [`include/shulib/hal/brain_screen.hpp:149`](../../include/shulib/hal/brain_screen.hpp#L149).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 30 lines</summary>

```text

 IBrainScreen — the V5 BRAIN's own touch screen, as a seam (chunk CAL1, Part A1, 2026-10-08).

 Why a seam: the standard Calibrate is library code, and the library is PROS-free outside
 hal/pros/ (CI enforces it, path-anchored). What a calibration screen paints — which lines to
 redraw, in which colour, padded how far — is LOGIC, and logic belongs where the host suite can
 RUN it. So the library's painter (calibration/calibrate_screen.hpp) draws through this seam:
 hal::pros::ProsBrainScreen on the robot, hal::fake::FakeBrainScreen in a host test (it records
 every call as one line of text, so a screen can be compared call for call).

 ═══ ONLY SANITIZED TEXT EVER REACHES THE SCREEN ════════════════════════════════════
 We BELIEVE PROS hands printed text to VEXos AS A FORMAT STRING — R3d Part 2F's reading of PROS
 4.2.2, the belief printText() (src/common/screen_text.hpp) was written on; read, never measured,
 and not yet an entry of the A4 register — so a '%' that reached the screen would be re-read as a
 conversion with no argument behind it; and VEXos's fonts carry ASCII only. Sanitizing costs
 nothing if the belief is wrong and saves the screen if it is right. screenSafeLine()
 below is the screen's sanitizer for the library — every IBrainScreen::print() passes its text
 through it, so a caller can never hand VEXos a format character. Its rules are those printText()
 (src/common/screen_text.hpp, the robot programs' one path to the screen, written for "shulib
 Calibrate" at R3d Part 2F-2 on 2026-09-24) was written with: '%' and control bytes become '?', a
 UTF-8 en or em dash becomes '-', any other non-ASCII character becomes ONE '?', the text is cut at
 the size's budget, then padded with spaces (never past the budget). In CAL1 Part A1 the two were
 the same rules in two places, held equal by a host test; since CAL1 Part B (2026-11-05)
 printText() CALLS this one, so there is one rule. (Bench Tests,
 src/programs/bench_r3a.cpp, keeps its own copy at its 54-column scrolling-log width, which ran on
 hardware as it is; whether Bench Tests joins the library is decided with the starter project.)

 NOT one of the frozen F4 ten — an ADDITIVE device seam, like ILineDisplay (the controller's
 LCD). PROS-free, allocation-free. Contract: every call is synchronous and MUST NOT THROW — a
 screen has no fallback channel, and the serial log carries the same words.
```

</details>
