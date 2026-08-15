<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/pros/clock.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `clock.hpp`

ProsClock — IClock over the V5's real time.

This header declares **1** type (1 member).

Extracted from [`include/shulib/hal/pros/clock.hpp`](../../include/shulib/hal/pros/clock.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class ProsClock`](#class-prosclock)
  - [`now`](#prosclock-now)

<a id="class-prosclock"></a>

## `class ProsClock`

```cpp
class ProsClock final : public IClock
```

IClock over the V5's real time: `pros::micros()` scaled to canonical SECONDS, ×1e-6, exactly once, here. Deliberately not `millis()` — 1 ms of quantization is 10% of the 10 ms control tick, and every PID derivative term and profile timing divides by dt, so a millisecond clock injects a 10% error into each of them that the host plant can never reproduce because FakeClock is exact. Monotonic within a run (an unsigned counter from boot; uint64 microseconds do not wrap in any run, whereas millis()'s uint32 would at ~49.7 days). The epoch is program start, which satisfies IClock's "a fixed per-run epoch" without being any particular zero. Stateless and free of pacing — tick_pacer.hpp owns waiting.

*class, declared at [`include/shulib/hal/pros/clock.hpp:51`](../../include/shulib/hal/pros/clock.hpp#L51).*

<a id="prosclock-now"></a>

### `ProsClock::now`

```cpp
[[nodiscard]] units::Time now() const override
```

Seconds since program start. Monotonic non-decreasing (header note).

*function, declared at [`include/shulib/hal/pros/clock.hpp:54`](../../include/shulib/hal/pros/clock.hpp#L54).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 29 lines</summary>

```text

 ProsClock — IClock over the V5's real time (chunk R1a).

 BINDS: pros::micros() (uint64 µs since program start — HA-101), NOT
 pros::millis(). millis() quantizes to 1 ms, which is 10% of the 10 ms
 control tick: PID derivative terms and profile timing divide by dt, so a
 10%-quantized dt is a 10% error in every derivative term — and the host
 plant has never seen it, because FakeClock is exact. micros() costs nothing
 and removes the whole question (brief T6; the rejected alternative was
 millis() on "the tick is 10 ms so ms is enough", which conflates the tick
 period with the measurement of it).

 CONVERTS: µs → canonical seconds, ×1e-6, exactly once, here (clock.hpp:9-10:
 "the V5's milliseconds are converted to seconds exactly once, in the
 hal/pros adapter" — this adapter converts microseconds instead, same rule).

 MONOTONICITY (clock.hpp:12): micros() is an unsigned counter from boot;
 uint64 µs wraps after ~584,000 years, so within any run now() never
 decreases. (millis()'s uint32 would wrap at ~49.7 days — irrelevant to a
 match, but the reason the wrap is stated rather than silently assumed.)

 Epoch: program start (whatever micros() counts from) — IClock requires "a
 fixed per-run epoch", not a particular zero.

 DELIBERATELY NOT here: no tick pacing (tick_pacer.hpp owns that), no
 timeouts, no dt bookkeeping — one seam, one job.

 HA register: HA-101 (docs/hardware-assumptions.md). PROVISIONAL until the
 bench session confirms micros() advances ~1:1 with wall time.
```

</details>
