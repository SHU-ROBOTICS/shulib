<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/control/settled_util.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `settled_util.hpp`

SettledUtil — the motion exit check.

This header declares **2** types (7 members).

Extracted from [`include/shulib/control/settled_util.hpp`](../../include/shulib/control/settled_util.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`struct SettleConfig`](#struct-settleconfig)
  - [`maxError`](#settleconfig-maxerror)
  - [`maxErrorRate`](#settleconfig-maxerrorrate)
  - [`settleTime`](#settleconfig-settletime)
- [`class SettledUtil`](#class-settledutil)
  - [`SettledUtil`](#settledutil-settledutil)
  - [`update`](#settledutil-update)
  - [`reset`](#settledutil-reset)
  - [`isSettled`](#settledutil-issettled)

<a id="struct-settleconfig"></a>

## `struct SettleConfig`

```cpp
struct SettleConfig
```

The three-part settle criterion, as one value. Bare doubles like PidConfig: the UNITS are the caller's error units and this struct never learns them — MotionConfig's three instances are where they are pinned (inches for translation, radians for heading, in/s for DriveBrake's averaged speed norm). Every default is 0, which is deliberately NOT a working configuration: it demands an exactly-zero error at an exactly-zero rate, so a default-constructed SettleConfig effectively never settles. Set all three.

*struct, declared at [`include/shulib/control/settled_util.hpp:29`](../../include/shulib/control/settled_util.hpp#L29).*

<a id="settleconfig-maxerror"></a>

### `SettleConfig::maxError`

```cpp
double maxError = 0.0
```

|error| ≤ this is close enough (INCLUSIVE bound).

*field, declared at [`include/shulib/control/settled_util.hpp:30`](../../include/shulib/control/settled_util.hpp#L30).*

<a id="settleconfig-maxerrorrate"></a>

### `SettleConfig::maxErrorRate`

```cpp
double maxErrorRate = 0.0
```

|d(error)/dt| ≤ this (units/s) — no settling mid-overshoot.

*field, declared at [`include/shulib/control/settled_util.hpp:31`](../../include/shulib/control/settled_util.hpp#L31).*

<a id="settleconfig-settletime"></a>

### `SettleConfig::settleTime`

```cpp
double settleTime = 0.0
```

Both must hold continuously for this long (s); 0 ⇒ one tick.

*field, declared at [`include/shulib/control/settled_util.hpp:32`](../../include/shulib/control/settled_util.hpp#L32).*

<a id="class-settledutil"></a>

## `class SettledUtil`

```cpp
class SettledUtil
```

The exit check as a stateful object: feed it one error per tick, it answers "settled?". It measures the error RATE itself by differencing consecutive update() calls against the injected clock, so it is not interchangeable with a stateless predicate: the answer depends on the history since the last reset(), not on this tick's error alone. Calling update() twice in one tick is SAFE but lossy — the second call sees dt == 0, mutates nothing and repeats the previous verdict, so the error it was handed is DISCARDED and never becomes the rate baseline. One instance per criterion, owned by the motion and baseline carried across a motion boundary would differentiate a jump that never happened.

*class, declared at [`include/shulib/control/settled_util.hpp:43`](../../include/shulib/control/settled_util.hpp#L43).*

<a id="settledutil-settledutil"></a>

### `SettledUtil::SettledUtil`

```cpp
SettledUtil(const SettleConfig& config, hal::IClock& clock)
```

All three bounds must be ≥ 0 or this raises. `clock` is BORROWED — it must outlive this object, and it is the same clock the rest of the motion reads, which is what makes the settle verdict reproducible under a fake clock. Construction opens no window: the first update() only establishes the rate baseline.

*function, declared at [`include/shulib/control/settled_util.hpp:49`](../../include/shulib/control/settled_util.hpp#L49).*

<a id="settledutil-update"></a>

### `SettledUtil::update`

```cpp
[[nodiscard]] bool update(double error)
```

Feed the current error; returns true once settled (and stays true while it remains so).

*function, declared at [`include/shulib/control/settled_util.hpp:56`](../../include/shulib/control/settled_util.hpp#L56).*

<a id="settledutil-reset"></a>

### `SettledUtil::reset`

```cpp
void reset()
```

Drop the rate baseline, the open window AND the verdict. The next update() ALWAYS answers false — it only re-establishes the baseline, so there is no rate yet — and the one after it can answer true only when settleTime is 0, because the window it opens starts at that same instant; with settleTime > 0 the earliest true arrives settleTime later, however close to target the robot already is. Call it at every motion start(): re-using a baseline across motions differentiates the gap between two unrelated errors. The config and the clock reference are untouched.

*function, declared at [`include/shulib/control/settled_util.hpp:93`](../../include/shulib/control/settled_util.hpp#L93).*

<a id="settledutil-issettled"></a>

### `SettledUtil::isSettled`

```cpp
[[nodiscard]] bool isSettled() const noexcept
```

The verdict the last update() computed — a pure read that touches neither the clock nor the window, so it is safe to call repeatedly per tick (telemetry, exit logic).

*function, declared at [`include/shulib/control/settled_util.hpp:101`](../../include/shulib/control/settled_util.hpp#L101).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 13 lines</summary>

```text

 SettledUtil — the motion exit check (master plan §13 #9, OkapiLib-style). A motion is
 SETTLED only when ALL THREE hold continuously for a settle time:
   * |error| ≤ maxError            (close enough), AND
   * |d(error)/dt| ≤ maxErrorRate  (not still moving — no settling mid-overshoot), AND
   * both have held for ≥ settleTime.

 Requiring the RATE (not just position) is what stops a motion from declaring victory while
 flying through the target; requiring the HELD time stops single-tick flukes. dt comes from
 an injected clock, so it is deterministic and host-testable. The first call (no rate yet)
 and any dt ≤ 0 tick never report settled.

 Bare double error, like Pid: the motion layer feeds per-axis error in matching units.
```

</details>
