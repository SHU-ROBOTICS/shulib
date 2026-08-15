<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/pros/digital_in.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `digital_in.hpp`

ProsDigitalIn — IDigitalIn over pros::adi::DigitalIn (chunk R1b): a limit switch / bumper line behind the HAL.

This header declares **1** type (4 members).

Extracted from [`include/shulib/hal/pros/digital_in.hpp`](../../include/shulib/hal/pros/digital_in.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class ProsDigitalIn`](#class-prosdigitalin)
  - [`ProsDigitalIn`](#prosdigitalin-prosdigitalin)
  - [`ProsDigitalIn (overload 2)`](#prosdigitalin-prosdigitalin-2)
  - [`state`](#prosdigitalin-state)
  - [`faultedReads`](#prosdigitalin-faultedreads)

<a id="class-prosdigitalin"></a>

## `class ProsDigitalIn`

```cpp
class ProsDigitalIn final : public IDigitalIn
```

`IDigitalIn` over `pros::adi::DigitalIn` — a limit switch or bumper line behind the HAL.  Binds `get_value()` and NEVER `get_new_press()`: PROS's ADI edge detection CONSUMES the press as it reads it, so with two consumers one silently misses every press. Edge detection belongs above the seam in `hal::ButtonEdge`, one instance per consumer. A guard test greps this file to keep the forbidden binding structurally absent.  A REFUSED READ IS SCREENED, not passed through. `get_value()` answers PROS_ERR on a dead port, and PROS_ERR is not 0, so an unscreened binding would report a dead port as PRESSED — a homing switch permanently "pressed" is a lift that believes it is home while it climbs into the hard stop. This adapter holds the last good level instead and counts the read in `faultedReads()`. Mapping to `false` would be no better: a hard "released" is as much a lie as a hard "pressed", and the consumer's cross-checks (homing travel limits) are designed around last-good.  Two constructors, one class: a brain ADI port, or an expander's {smart port, ADI port}. Where the wire lands is a construction fact, never a type.  `state()` is const but caches, so one instance must not be read concurrently from two tasks.

*class, declared at [`include/shulib/hal/pros/digital_in.hpp:71`](../../include/shulib/hal/pros/digital_in.hpp#L71).*

<a id="prosdigitalin-prosdigitalin"></a>

### `ProsDigitalIn::ProsDigitalIn`

```cpp
explicit ProsDigitalIn(std::uint8_t adiPort)
```

Brain ADI port ('a'–'h', 'A'–'H', or 1–8).

*function, declared at [`include/shulib/hal/pros/digital_in.hpp:74`](../../include/shulib/hal/pros/digital_in.hpp#L74).*

<a id="prosdigitalin-prosdigitalin-2"></a>

### `ProsDigitalIn::ProsDigitalIn (overload 2)`

```cpp
ProsDigitalIn(std::uint8_t smartPort, std::uint8_t adiPort)
```

Expander form: {smartPort 1–21, adiPort as above}.

*function, declared at [`include/shulib/hal/pros/digital_in.hpp:77`](../../include/shulib/hal/pros/digital_in.hpp#L77).*

<a id="prosdigitalin-state"></a>

### `ProsDigitalIn::state`

```cpp
[[nodiscard]] bool state() const override
```

The raw level via get_value() — NEVER the consuming get_new_press() (header). PROS_ERR is screened to the last good level (T7).

*function, declared at [`include/shulib/hal/pros/digital_in.hpp:82`](../../include/shulib/hal/pros/digital_in.hpp#L82).*

<a id="prosdigitalin-faultedreads"></a>

### `ProsDigitalIn::faultedReads`

```cpp
[[nodiscard]] int faultedReads() const noexcept
```

How many reads were screened to last-good (T7 observability).

*function, declared at [`include/shulib/hal/pros/digital_in.hpp:93`](../../include/shulib/hal/pros/digital_in.hpp#L93).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 36 lines</summary>

```text

 ProsDigitalIn — IDigitalIn over pros::adi::DigitalIn (chunk R1b): a limit
 switch / bumper line behind the HAL.

 BINDS:
  * get_value() [int32 level, 1/0; HA-121] → state() — a LEVEL, deliberately.
    This adapter NEVER calls get_new_press(): PROS's ADI edge detection
    CONSUMES the press when read ("1 if the button is pressed and had not
    been pressed the last time this function was called", vendored
    adi.hpp:711-712), so with two consumers one silently loses — the exact
    trap the controller adapter already rules on (HA-104; HA-121 is its ADI
    sibling). Per-consumer edge detection lives ABOVE the seam in
    hal::ButtonEdge (controller.hpp), one instance per consumer. The fence
    guard test greps this file to keep the forbidden binding structurally
    absent.

 ADDRESSING (T6, HA-120): one class, two constructors — brain ADI port
 ('a'–'h', 'A'–'H', or 1–8) or an expander's {smartPort, adiPort} — same as
 ProsDigitalOut, same reasoning: the seam is identical, so where the wire
 lands is a construction fact, never a type.

 SENTINELS (T7): the seam has no validity channel (digital_in.hpp — a dead
 ADI port is indistinguishable from a working one). get_value() returns
 PROS_ERR when the port refuses (HA-121), and PROS_ERR != 0, so an
 UNSCREENED binding would read a dead port as PRESSED — a homing switch
 permanently "pressed" means a lift that believes it is homed while it
 climbs into the hard stop. Screened: hold the last good level, count in
 faultedReads(). Never map to false either (a hard "released" is as wrong
 as a hard "pressed"; last-good is what the consumer's own cross-checks —
 homing travel limits — are designed around).

 DELIBERATELY NOT here: debouncing (the seam's own ruling — a filter
 constant belongs to the consumer that knows what the switch is for) and
 any homing routine (F3's, season content the students author).

 HA register: HA-120, HA-121 (docs/hardware-assumptions.md).
```

</details>
