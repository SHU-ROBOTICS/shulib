<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/pros/battery.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `battery.hpp`

ProsBattery — IBattery over the pros::battery namespace (chunk R1a): the brownout-compensation input behind the HAL.

This header declares **1** type (4 members).

Extracted from [`include/shulib/hal/pros/battery.hpp`](../../include/shulib/hal/pros/battery.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class ProsBattery`](#class-prosbattery)
  - [`voltage`](#prosbattery-voltage)
  - [`current`](#prosbattery-current)
  - [`capacity`](#prosbattery-capacity)
  - [`faultedReads`](#prosbattery-faultedreads)

<a id="class-prosbattery"></a>

## `class ProsBattery`

```cpp
class ProsBattery final : public IBattery
```

IBattery over the pros::battery free functions: millivolts and milliamps divided by 1000 here, once, and capacity percent scaled to [0, 1] and clamped so a device quirk cannot hand the core a 1.02.  READ THE HEADER BANNER BEFORE TRUSTING A NUMBER FROM THIS CLASS. The mV/mA belief is the weakest conversion provenance in the PROS adapter set — it comes from PROS's website, while the misc.h this tree vendors documents these as int32 with no unit at all. A 1000x error would silently destroy brownout compensation, which bounds every DRIVE command — the ±battery clamp lives in the per-wheel drive pipeline, and mechanism motors bypass it entirely, keeping only IMotor's fixed ±12 V clamp. So the bench procedure checks the raw integer reads ~12600 rather than ~12.6 before anything drives.  IBattery has no validity channel, so a sentinel read HOLDS the last good value and is counted in faultedReads() — never zero, because 0 V would read as the deepest possible brownout and floor every command. Pre-first-read defaults are a healthy fresh pack for the same reason: this class is never silent, but it is also never alarming by accident.

*class, declared at [`include/shulib/hal/pros/battery.hpp:65`](../../include/shulib/hal/pros/battery.hpp#L65).*

<a id="prosbattery-voltage"></a>

### `ProsBattery::voltage`

```cpp
[[nodiscard]] units::Voltage voltage() const override
```

Canonical volts (mV ÷ 1000 — HA-99, the website-only belief).

*function, declared at [`include/shulib/hal/pros/battery.hpp:68`](../../include/shulib/hal/pros/battery.hpp#L68).*

<a id="prosbattery-current"></a>

### `ProsBattery::current`

```cpp
[[nodiscard]] units::Current current() const override
```

Canonical amperes (mA ÷ 1000 — HA-99).

*function, declared at [`include/shulib/hal/pros/battery.hpp:79`](../../include/shulib/hal/pros/battery.hpp#L79).*

<a id="prosbattery-capacity"></a>

### `ProsBattery::capacity`

```cpp
[[nodiscard]] double capacity() const override
```

[0, 1] (percent ÷ 100 — HA-100), clamped to the contract's range.

*function, declared at [`include/shulib/hal/pros/battery.hpp:90`](../../include/shulib/hal/pros/battery.hpp#L90).*

<a id="prosbattery-faultedreads"></a>

### `ProsBattery::faultedReads`

```cpp
[[nodiscard]] int faultedReads() const noexcept
```

How many reads were screened to last-good (T7 observability).

*function, declared at [`include/shulib/hal/pros/battery.hpp:101`](../../include/shulib/hal/pros/battery.hpp#L101).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 30 lines</summary>

```text

 ProsBattery — IBattery over the pros::battery namespace (chunk R1a): the
 brownout-compensation input behind the HAL.

 BINDS: battery::get_voltage(), battery::get_current(), battery::get_capacity().

 ═══ THE UNIT BELIEF HERE IS THE WEAKEST IN R1a — read this before trusting
 a number ═══════════════════════════════════════════════════════════════════
 The vendored misc.h documents get_voltage()/get_current() as int32 with NO
 UNIT AT ALL ("the current voltage of the battery", misc.h:718-750). The
 mV / mA belief below comes from PROS's WEBSITE, not from the source this
 tree vendors — a strictly weaker provenance than every other conversion in
 this chunk, and it is registered as such (HA-99). The stakes: a 1000× error
 here silently destroys brownout compensation, which BOUNDS every DRIVE
 command and scales nothing — control::compensateForBattery() clamps each
 wheel's desired volts to ±the measured pack, in the per-wheel drive pipeline
 only. Mechanism motors never reach it: their volts are bounded by IMotor's
 fixed ±12 V clamp alone. So the bench runbook's battery step runs before any
 driving step, and checks the raw integer is ~12600, not ~12.6.

 CONVERTS: mV→V and mA→A (÷1000, once, here); capacity percent→[0,1]
 (÷100, HA-100), clamped so a device quirk can never hand the core 1.02.

 SENTINELS (T7): IBattery has no validity channel. PROS_ERR / PROS_ERR_F
 reads hold the last good value — never zero: a 0 V battery reading would
 read as the deepest possible brownout and floor every motor command.
 Defaults before any good read are a healthy fresh pack (12.6 V, HA-46) for
 the same reason. faultedReads() exposes the screen count.

 HA register: HA-99, HA-100, HA-46.
```

</details>
