<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/line_display.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `line_display.hpp`

ILineDisplay — where short status LINES physically go (the V5 controller's LCD; a captured fake in tests).

This header declares **1** type (9 members).

Extracted from [`include/shulib/hal/line_display.hpp`](../../include/shulib/hal/line_display.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class ILineDisplay`](#class-ilinedisplay)
  - [`kRows`](#ilinedisplay-krows)
  - [`kCols`](#ilinedisplay-kcols)
  - [`~ILineDisplay`](#ilinedisplay-destructor-ilinedisplay)
  - [`ILineDisplay`](#ilinedisplay-ilinedisplay)
  - [`ILineDisplay (overload 2)`](#ilinedisplay-ilinedisplay-2)
  - [`ILineDisplay (overload 3)`](#ilinedisplay-ilinedisplay-3)
  - [`operator=`](#ilinedisplay-operator-eq)
  - [`operator= (overload 2)`](#ilinedisplay-operator-eq-2)
  - [`setLine`](#ilinedisplay-setline)

<a id="class-ilinedisplay"></a>

## `class ILineDisplay`

```cpp
class ILineDisplay
```

Where short status LINES physically go — the V5 controller's LCD, or a capturing fake in tests. A ROW device, not a byte stream: three fixed rows overwritten in place, no scrollback, so the verb is "set row i to text" rather than "append bytes". That shape is what lets content code rewrite only the rows that CHANGED, which matters because V5 controller writes are slow and firmware-rate-limited — a per-tick full repaint is how the display starves. Not one of the frozen F4 ten; an additive diagnostics-output seam, like ICharSink.

*class, declared at [`include/shulib/hal/line_display.hpp:39`](../../include/shulib/hal/line_display.hpp#L39).*

<a id="ilinedisplay-krows"></a>

### `ILineDisplay::kRows`

```cpp
static constexpr int kRows = 3
```

The V5 controller text grid. PROVISIONAL (A4: HA-57) — see header.

*field, declared at [`include/shulib/hal/line_display.hpp:42`](../../include/shulib/hal/line_display.hpp#L42).*

<a id="ilinedisplay-kcols"></a>

### `ILineDisplay::kCols`

```cpp
static constexpr int kCols = 19
```

Columns per row. Text beyond it is TRUNCATED by implementations and never wrapped — a wrapped status row would overwrite the row below it. Also unverified against real firmware: the vendored PROS header implies 15 columns, community practice says 19, and neither is a measurement (A4: HA-57, HA-107).

*field, declared at [`include/shulib/hal/line_display.hpp:47`](../../include/shulib/hal/line_display.hpp#L47).*

<a id="ilinedisplay-destructor-ilinedisplay"></a>

### `ILineDisplay::~ILineDisplay`

```cpp
virtual ~ILineDisplay() = default
```

Polymorphic-base boilerplate: the destructor is virtual so deleting through an `ILineDisplay*` is well-defined, and declaring it suppresses the implicit copy/move, which are re-defaulted here. Nothing in this tree deletes one that way, though — the seam is stateless (the rows live in the implementation) and a display is REFERENCED, never owned: ControllerFaultDisplay holds a non-owning `ILineDisplay&`, so the implementation must outlive every display bound to it.

*function, declared at [`include/shulib/hal/line_display.hpp:55`](../../include/shulib/hal/line_display.hpp#L55).*

<a id="ilinedisplay-ilinedisplay"></a>

### `ILineDisplay::ILineDisplay`

```cpp
ILineDisplay() = default
```

*Covered by the comment on [`~ILineDisplay`](#ilinedisplay-destructor-ilinedisplay) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/line_display.hpp:56`](../../include/shulib/hal/line_display.hpp#L56).*

<a id="ilinedisplay-ilinedisplay-2"></a>

### `ILineDisplay::ILineDisplay (overload 2)`

```cpp
ILineDisplay(const ILineDisplay&) = default
```

*Covered by the comment on [`~ILineDisplay`](#ilinedisplay-destructor-ilinedisplay) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/line_display.hpp:57`](../../include/shulib/hal/line_display.hpp#L57).*

<a id="ilinedisplay-ilinedisplay-3"></a>

### `ILineDisplay::ILineDisplay (overload 3)`

```cpp
ILineDisplay(ILineDisplay&&) = default
```

*Covered by the comment on [`~ILineDisplay`](#ilinedisplay-destructor-ilinedisplay) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/line_display.hpp:58`](../../include/shulib/hal/line_display.hpp#L58).*

<a id="ilinedisplay-operator-eq"></a>

### `ILineDisplay::operator=`

```cpp
ILineDisplay& operator=(const ILineDisplay&) = default
```

*Covered by the comment on [`~ILineDisplay`](#ilinedisplay-destructor-ilinedisplay) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/line_display.hpp:59`](../../include/shulib/hal/line_display.hpp#L59).*

<a id="ilinedisplay-operator-eq-2"></a>

### `ILineDisplay::operator= (overload 2)`

```cpp
ILineDisplay& operator=(ILineDisplay&&) = default
```

*Covered by the comment on [`~ILineDisplay`](#ilinedisplay-destructor-ilinedisplay) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/line_display.hpp:60`](../../include/shulib/hal/line_display.hpp#L60).*

<a id="ilinedisplay-setline"></a>

### `ILineDisplay::setLine`

```cpp
virtual void setLine(int row, std::string_view text) = 0
```

Overwrite row `row` (0-based, caller keeps row < kRows) with `text`, truncated at kCols. MUST NOT throw.

*function, declared at [`include/shulib/hal/line_display.hpp:64`](../../include/shulib/hal/line_display.hpp#L64).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 26 lines</summary>

```text

 ILineDisplay — where short status LINES physically go (the V5 controller's LCD;
 a captured fake in tests). Diagnostics-plan D-4's SEAM (WS13, chunk C5).

 Why a seam and not a PROS call: the core is PROS-free and CI enforces it —
 exactly the ICharSink pattern (A1). C5 builds the CONTENT and this seam; the
 actual pros::Controller::set_text adapter is R1's glue, one file, zero core
 changes. Tests use hal::fake::FakeLineDisplay and assert exact rows.

 Why line-oriented (not ICharSink reuse): the controller LCD is a ROW device —
 three fixed rows, overwritten in place, no scrollback — so "append bytes" is
 the wrong verb; "set row i to text" is the device's real contract, and it is
 what lets content code express "rewrite only what changed" (V5 controller
 writes are slow and firmware-rate-limited; a per-tick full repaint is how a
 display starves — see ControllerFaultDisplay).

 Geometry: kRows = 3, kCols = 19 — the V5 controller's documented text grid.
 HARDWARE CLAIM, honest scope: unverified against real firmware until R1.
 PROVISIONAL (A4: HA-57). Implementations TRUNCATE text beyond kCols (never
 wrap — a wrapped status row would corrupt the row below it).

 NOT part of the frozen F4 ten (that freeze covers the 10 runtime robot-HAL
 interfaces); an ADDITIVE diagnostics-output seam, like ICharSink before it.

 Contract: setLine() is synchronous on the caller's task, MUST NOT throw, and
 row is in [0, kRows) — a bad row is the CALLER's precondition to keep.
```

</details>
