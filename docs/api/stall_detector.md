<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/manipulation/stall_detector.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `stall_detector.hpp`

StallDetector — the jam/stall decision for motor mechanisms.

This header declares **2** types (7 members).

Extracted from [`include/shulib/manipulation/stall_detector.hpp`](../../include/shulib/manipulation/stall_detector.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`struct StallConfig`](#struct-stallconfig)
  - [`currentAtLeast`](#stallconfig-currentatleast)
  - [`speedAtMost`](#stallconfig-speedatmost)
  - [`persistence`](#stallconfig-persistence)
- [`class StallDetector`](#class-stalldetector)
  - [`StallDetector`](#stalldetector-stalldetector)
  - [`reset`](#stalldetector-reset)
  - [`update`](#stalldetector-update)
  - [`config`](#stalldetector-config)

<a id="struct-stallconfig"></a>

## `struct StallConfig`

```cpp
struct StallConfig
```

The three numbers that define "stalled" for ONE mechanism. There are NO defaults, deliberately: a 5.5 W and an 11 W motor stall at different currents, and a lift and an intake tolerate different windows, so any library-chosen value would be silently wrong for half of them. Concrete mechanisms pick their own and register them; hardware measurement replaces the estimates later. Validation of all three happens in `StallDetector`'s constructor, not here.

*struct, declared at [`include/shulib/manipulation/stall_detector.hpp:37`](../../include/shulib/manipulation/stall_detector.hpp#L37).*

<a id="stallconfig-currentatleast"></a>

### `StallConfig::currentAtLeast`

```cpp
units::Current currentAtLeast
```

Current at or above this (amps, > 0) counts as stall-grade.

*field, declared at [`include/shulib/manipulation/stall_detector.hpp:39`](../../include/shulib/manipulation/stall_detector.hpp#L39).*

<a id="stallconfig-speedatmost"></a>

### `StallConfig::speedAtMost`

```cpp
units::AngularVelocity speedAtMost
```

|shaft velocity| at or below this (rad/s, >= 0) counts as not turning.

*field, declared at [`include/shulib/manipulation/stall_detector.hpp:41`](../../include/shulib/manipulation/stall_detector.hpp#L41).*

<a id="stallconfig-persistence"></a>

### `StallConfig::persistence`

```cpp
units::Time persistence
```

Both conditions must hold CONTINUOUSLY for this long (seconds, >= 0; 0 = trip on the first qualifying sample — legitimate for a mechanism with no spin-up, wrong for most).

*field, declared at [`include/shulib/manipulation/stall_detector.hpp:45`](../../include/shulib/manipulation/stall_detector.hpp#L45).*

<a id="class-stalldetector"></a>

## `class StallDetector`

```cpp
class StallDetector
```

The jam/stall decision for a motor mechanism: HIGH CURRENT with the shaft NOT TURNING, held continuously for a persistence window. That window is the whole difference between a jam and a spin-up — accelerating from rest draws stall-grade current for the first few ticks with the shaft still nearly stopped, and without a window every start reads as a jam.  Both signals are compared by MAGNITUDE, so direction is irrelevant: a mechanism jammed while running in reverse trips exactly the same way.  STATEFUL — it remembers when the current window opened, so what update() answers depends on the samples that came before it and not on this one alone. Re-feeding an IDENTICAL sample is harmless: `windowStart_` moves only when a CLOSED window opens, so a duplicate read within one tick returns the same answer and leaves the window where it was. What the sequence does buy is the other direction — one healthy sample in the middle closes the window, and the persistence count starts over from the next qualifying sample. It holds NO clock: the caller supplies `now`, which is what makes it loop-rate independent and testable without one.  HONEST LIMIT, stated rather than discovered: it knows only what the sensors say. A motor whose encoder reports "spinning" under a true stall passes straight through it. That is why the operation layer's watchdog is the backstop — the watchdog needs no sensor honesty at all.

*class, declared at [`include/shulib/manipulation/stall_detector.hpp:67`](../../include/shulib/manipulation/stall_detector.hpp#L67).*

<a id="stalldetector-stalldetector"></a>

### `StallDetector::StallDetector`

```cpp
explicit StallDetector(const StallConfig& config)
```

Thresholds are required and CHECKED, not clamped: `currentAtLeast` must be finite and > 0, `speedAtMost` and `persistence` finite and >= 0; anything else is a precondition failure. The config is COPIED here, so mutating the caller's StallConfig afterward changes nothing. No window is open yet — the first update() carrying the signature is what starts the clock.

*function, declared at [`include/shulib/manipulation/stall_detector.hpp:73`](../../include/shulib/manipulation/stall_detector.hpp#L73).*

<a id="stalldetector-reset"></a>

### `StallDetector::reset`

```cpp
void reset() noexcept
```

Re-arm (forget any partial window) — an operation's start() calls this.

*function, declared at [`include/shulib/manipulation/stall_detector.hpp:86`](../../include/shulib/manipulation/stall_detector.hpp#L86).*

<a id="stalldetector-update"></a>

### `StallDetector::update`

```cpp
[[nodiscard]] bool update(units::Time now, units::Current current, units::AngularVelocity velocity)
```

Feed one sample; true once the stall signature has held for the full persistence window. The window RESETS on any healthy sample — a jam that intermittently slips is a chatter problem for R4's thresholds, not a reason to remember stale evidence.

*function, declared at [`include/shulib/manipulation/stall_detector.hpp:92`](../../include/shulib/manipulation/stall_detector.hpp#L92).*

<a id="stalldetector-config"></a>

### `StallDetector::config`

```cpp
[[nodiscard]] const StallConfig& config() const noexcept
```

The thresholds this detector is using — the constructor's copy, so it is the authority on what was actually accepted. Read-only: there is no setter, and re-tuning means constructing a new detector. Useful for telemetry that wants to report the threshold a trip fired at.

*function, declared at [`include/shulib/manipulation/stall_detector.hpp:110`](../../include/shulib/manipulation/stall_detector.hpp#L110).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 22 lines</summary>

```text

 StallDetector — the jam/stall decision for motor mechanisms (chunk F1).
 The physical signature of a stalled or jammed shaft is HIGH CURRENT with the
 shaft NOT TURNING, held for long enough to rule out a start-up transient
 (spin-up draws stall-grade current for the first few ticks with the shaft
 still accelerating from rest — a persistence window is what keeps that from
 reading as a jam). motor.hpp names current() "the PRIMARY capture/stall
 signal for manipulation sensor-confirm"; this class is that sentence made
 executable.

 The thresholds are REQUIRED constructor parameters with NO defaults, on
 C2's waitUntil precedent (an explicit finite timeout, "no invented default
 constant"): a 5.5 W and an 11 W motor stall at different currents, a lift and
 an intake tolerate different windows, and a library default would be wrong
 for half of them silently. F3's concrete primitives choose values per
 mechanism and register them; R4 measures them.

 Honest limits, stated: the detector can only know what the sensors say. A
 motor whose encoder lies "spinning" under a true stall passes right through
 it — which is why the operation layer's watchdog is the backstop and needs no
 sensor honesty at all (mechanism_op.hpp), and why the F1 hostile suite
 includes exactly that lying motor.
```

</details>
