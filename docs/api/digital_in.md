<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/digital_in.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `digital_in.hpp`

IDigitalIn — a single digital input line behind the HAL (chunk R1b): a limit switch, a bumper, a jumper — any two-state sensor on an ADI port.

This header declares **1** type (7 members).

Extracted from [`include/shulib/hal/digital_in.hpp`](../../include/shulib/hal/digital_in.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class IDigitalIn`](#class-idigitalin)
  - [`~IDigitalIn`](#idigitalin-destructor-idigitalin)
  - [`IDigitalIn`](#idigitalin-idigitalin)
  - [`IDigitalIn (overload 2)`](#idigitalin-idigitalin-2)
  - [`IDigitalIn (overload 3)`](#idigitalin-idigitalin-3)
  - [`operator=`](#idigitalin-operator-eq)
  - [`operator= (overload 2)`](#idigitalin-operator-eq-2)
  - [`state`](#idigitalin-state)

<a id="class-idigitalin"></a>

## `class IDigitalIn`

```cpp
class IDigitalIn
```

One digital input line behind the HAL — a limit switch, a bumper, a jumper: any two-state sensor on an ADI port. The input sibling of IDigitalOut, and a seam with exactly one degree of freedom, which is why three things are deliberately absent and belong to the CONSUMER instead: * NO debouncing. A homing switch and a collision bumper want different filters, and this layer cannot know which it is serving — a time constant here would be an unmeasured constant chosen by the one layer with no idea what the switch is for. * NO edge detection. "Was it JUST pressed" is per-consumer state; put a `hal::ButtonEdge` above the seam, one per consumer, exactly as driver buttons do. * NO validity channel. A dead ADI port is indistinguishable from a working one at this level; catching a dead switch is a cross-check's job (homing travel limits), not this seam's.  It is deliberately OUTSIDE the F4 runtime-interface freeze, and said out loud rather than left to be inferred: it was added ahead of any consumer, while the lift-homing question (switch or stall?) is still open, so it may still move. A real consumer is what would freeze it.

*class, declared at [`include/shulib/hal/digital_in.hpp:58`](../../include/shulib/hal/digital_in.hpp#L58).*

<a id="idigitalin-destructor-idigitalin"></a>

### `IDigitalIn::~IDigitalIn`

```cpp
virtual ~IDigitalIn() = default
```

The rule-of-five set plus the default constructor — six `= default`s, none of which add behaviour, because this seam holds no state of its own. Spelled out in full because the suppression chain is easy to get wrong: a user-declared destructor suppresses only the implicit MOVE members (the copies survive it, their generation merely deprecated), re-declaring the moves is what would DELETE those copies, and declaring any constructor at all is what costs you the implicit default one. Writing only `virtual ~IFoo() = default;` therefore leaves a new interface still copyable, with every "move" of it quietly resolving to that copy. What copying a concrete IMPLEMENTATION means is that implementation's business — the interface makes no claim.

*function, declared at [`include/shulib/hal/digital_in.hpp:69`](../../include/shulib/hal/digital_in.hpp#L69).*

<a id="idigitalin-idigitalin"></a>

### `IDigitalIn::IDigitalIn`

```cpp
IDigitalIn() = default
```

*Covered by the comment on [`~IDigitalIn`](#idigitalin-destructor-idigitalin) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/digital_in.hpp:70`](../../include/shulib/hal/digital_in.hpp#L70).*

<a id="idigitalin-idigitalin-2"></a>

### `IDigitalIn::IDigitalIn (overload 2)`

```cpp
IDigitalIn(const IDigitalIn&) = default
```

*Covered by the comment on [`~IDigitalIn`](#idigitalin-destructor-idigitalin) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/digital_in.hpp:71`](../../include/shulib/hal/digital_in.hpp#L71).*

<a id="idigitalin-idigitalin-3"></a>

### `IDigitalIn::IDigitalIn (overload 3)`

```cpp
IDigitalIn(IDigitalIn&&) = default
```

*Covered by the comment on [`~IDigitalIn`](#idigitalin-destructor-idigitalin) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/digital_in.hpp:72`](../../include/shulib/hal/digital_in.hpp#L72).*

<a id="idigitalin-operator-eq"></a>

### `IDigitalIn::operator=`

```cpp
IDigitalIn& operator=(const IDigitalIn&) = default
```

*Covered by the comment on [`~IDigitalIn`](#idigitalin-destructor-idigitalin) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/digital_in.hpp:73`](../../include/shulib/hal/digital_in.hpp#L73).*

<a id="idigitalin-operator-eq-2"></a>

### `IDigitalIn::operator= (overload 2)`

```cpp
IDigitalIn& operator=(IDigitalIn&&) = default
```

*Covered by the comment on [`~IDigitalIn`](#idigitalin-destructor-idigitalin) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/digital_in.hpp:74`](../../include/shulib/hal/digital_in.hpp#L74).*

<a id="idigitalin-state"></a>

### `IDigitalIn::state`

```cpp
[[nodiscard]] virtual bool state() const = 0
```

The line's current level, raw and unfiltered (a LEVEL, not an edge — per-consumer edge detection lives in ButtonEdge, above the seam; header note). What "true" means physically belongs to the mechanism that owns the line.

*function, declared at [`include/shulib/hal/digital_in.hpp:80`](../../include/shulib/hal/digital_in.hpp#L80).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 38 lines</summary>

```text

 IDigitalIn — a single digital input line behind the HAL (chunk R1b): a limit
 switch, a bumper, a jumper — any two-state sensor on an ADI port. The input
 sibling of IDigitalOut (digital_out.hpp), added by the same additive-sibling
 path (F1's IDigitalOut, R1a's IController).

 ── Built on an open question, and saying so ────────────────────────────────────────
 The lift-homing question (switch or stall?) was asked 2026-08-13 and is NOT
 answered. This seam exists on the "cheap now, expensive to discover at R3
 with the robot on the bench" ruling — it has NO consumer yet, which departs
 from F1's earned-interface standard deliberately and out loud. If the answer
 comes back "stall", this is a small unused sibling; that cost was accepted
 when the seam was ruled in. What keeps the shape honest: a digital input has
 exactly one degree of freedom, so there is nothing to guess at.

 Contract (each clause inherited from IDigitalOut's rulings, digital_out.hpp):
  * state() is the RAW LEVEL, now. What "true" means physically (pressed?
    released? beam broken?) is plumbing-dependent and belongs to the mechanism
    that owns the line, never to this seam.
  * NO debouncing, deliberately. A homing switch wants a different filter from
    a collision bumper, and the adapter cannot know which it is serving —
    a time constant baked in here would be an unmeasured constant in the one
    layer with no idea what the switch is for. Debounce is the consumer's.
  * NO edge detection, deliberately. "Was it JUST pressed" is per-consumer
    state: reuse hal::ButtonEdge (controller.hpp) above the seam, exactly as
    driver buttons do — never PROS's adi DigitalIn::get_new_press(), which
    CONSUMES the event so a second consumer silently misses every press
    (HA-121, the ADI sibling of HA-104).
  * NO validity channel, like IDigitalOut: a dead ADI port is
    indistinguishable from a working one at this seam. Detecting a dead
    switch is a cross-check's job (e.g. homing travel limits), not this
    seam's.

 NOT part of the F4 freeze (locked 2026-06-19 with the ten runtime
 interfaces). This is chunk R1b's addition, outside it — register row F14
 records the NON-freeze out loud (D2's lesson: silence in that register
 reads as "frozen"). Freeze trigger: a real consumer — F3's homing decision,
 if it comes back "switch".
```

</details>
