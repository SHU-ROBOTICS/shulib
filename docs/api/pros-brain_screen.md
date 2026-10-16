<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/pros/brain_screen.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `brain_screen.hpp`

ProsBrainScreen — hal::IBrainScreen over PROS's screen API.

This header declares **1** type (7 members).

Extracted from [`include/shulib/hal/pros/brain_screen.hpp`](../../include/shulib/hal/pros/brain_screen.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class ProsBrainScreen`](#class-prosbrainscreen)
  - [`fillRect`](#prosbrainscreen-fillrect)
  - [`outlineRect`](#prosbrainscreen-outlinerect)
  - [`eraseRect`](#prosbrainscreen-eraserect)
  - [`eraseAll`](#prosbrainscreen-eraseall)
  - [`print`](#prosbrainscreen-print)
  - [`touch`](#prosbrainscreen-touch)
  - [`prints`](#prosbrainscreen-prints)

<a id="class-prosbrainscreen"></a>

## `class ProsBrainScreen`

```cpp
class ProsBrainScreen final : public IBrainScreen
```

The brain's own touch screen, through PROS (header). Holds only its print count.

*class, declared at [`include/shulib/hal/pros/brain_screen.hpp:37`](../../include/shulib/hal/pros/brain_screen.hpp#L37).*

<a id="prosbrainscreen-fillrect"></a>

### `ProsBrainScreen::fillRect`

```cpp
void fillRect(std::int16_t x0, std::int16_t y0, std::int16_t x1, std::int16_t y1, std::uint32_t rgb) override
```

screen_set_pen(rgb), then screen_fill_rect.

*function, declared at [`include/shulib/hal/pros/brain_screen.hpp:40`](../../include/shulib/hal/pros/brain_screen.hpp#L40).*

<a id="prosbrainscreen-outlinerect"></a>

### `ProsBrainScreen::outlineRect`

```cpp
void outlineRect(std::int16_t x0, std::int16_t y0, std::int16_t x1, std::int16_t y1, std::uint32_t rgb) override
```

screen_set_pen(rgb), then screen_draw_rect.

*function, declared at [`include/shulib/hal/pros/brain_screen.hpp:46`](../../include/shulib/hal/pros/brain_screen.hpp#L46).*

<a id="prosbrainscreen-eraserect"></a>

### `ProsBrainScreen::eraseRect`

```cpp
void eraseRect(std::int16_t x0, std::int16_t y0, std::int16_t x1, std::int16_t y1, std::uint32_t rgb) override
```

screen_set_eraser(rgb), then screen_erase_rect.

*function, declared at [`include/shulib/hal/pros/brain_screen.hpp:52`](../../include/shulib/hal/pros/brain_screen.hpp#L52).*

<a id="prosbrainscreen-eraseall"></a>

### `ProsBrainScreen::eraseAll`

```cpp
void eraseAll(std::uint32_t rgb) override
```

screen_set_eraser(rgb), then screen_erase.

*function, declared at [`include/shulib/hal/pros/brain_screen.hpp:58`](../../include/shulib/hal/pros/brain_screen.hpp#L58).*

<a id="prosbrainscreen-print"></a>

### `ProsBrainScreen::print`

```cpp
void print(ScreenTextSize size, std::int16_t x, std::int16_t y, std::uint32_t pen, std::uint32_t eraser, const char* text, std::size_t cols, std::size_t padTo = 0) override
```

screenSafeLine(text, cols, padTo), then screen_set_eraser, screen_set_pen and screen_print_at(size, x, y, "%s", line) — printText()'s order.

*function, declared at [`include/shulib/hal/pros/brain_screen.hpp:64`](../../include/shulib/hal/pros/brain_screen.hpp#L64).*

<a id="prosbrainscreen-touch"></a>

### `ProsBrainScreen::touch`

```cpp
[[nodiscard]] ScreenTouch touch() override
```

screen_touch_status(), an E_TOUCH_ERROR read reported as an error.

*function, declared at [`include/shulib/hal/pros/brain_screen.hpp:75`](../../include/shulib/hal/pros/brain_screen.hpp#L75).*

<a id="prosbrainscreen-prints"></a>

### `ProsBrainScreen::prints`

```cpp
[[nodiscard]] std::uint32_t prints() const override
```

Every print() through this object.

*function, declared at [`include/shulib/hal/pros/brain_screen.hpp:85`](../../include/shulib/hal/pros/brain_screen.hpp#L85).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 20 lines</summary>

```text

 ProsBrainScreen — hal::IBrainScreen over PROS's screen API (chunk CAL1, Part A1, 2026-10-08).

 Each call is the PROS sequence "shulib Calibrate" (src/programs/calibrate_program.cpp) makes
 inline today: set the pen (or the eraser), then draw. print() is the sequence of printText()
 (src/common/screen_text.hpp): the text through the sanitizer (hal::screenSafeLine(), the same
 rules), then set the eraser, set the pen, and print the sanitized line through a LITERAL "%s" —
 so no text a caller passes is ever read by VEXos as a format. The touch read is PROS's
 screen_touch_status(), an E_TOUCH_ERROR read reported as an error (PROS fills every field with
 -1 then), never as a tap.

 One difference from the program's inline calls, and why it changes nothing on the screen: the
 program outlines a button twice after ONE set_pen; through this seam each outlineRect() sets the
 pen itself, so the second outline sets the same pen again. The pixels are the same.

 NO HARDWARE CLAIM BEYOND THE PROGRAM'S: these are the calls "shulib Calibrate" has made on robot
 two since 2026-09-24 (R3d Part 2F-2), in the same order; THIS adapter has not run on a robot.
 Its host test (test/pros_brain_screen_adapter_test.cpp) runs it through the host PROS shim,
 which records every call — that checks the adapter against our BELIEF about PROS, not the
 belief itself.
```

</details>
