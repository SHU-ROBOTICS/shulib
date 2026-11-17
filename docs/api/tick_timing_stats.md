<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/diag/tick_timing_stats.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `tick_timing_stats.hpp`

TickTimingStats — the statistics of a loop's tick-to-tick dt, as DATA.

This header declares **3** types (16 members).

Extracted from [`include/shulib/diag/tick_timing_stats.hpp`](../../include/shulib/diag/tick_timing_stats.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in [How it works](../guide/README.md); worked recipes live in [Recipes](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`struct TickTimingSummary`](#struct-ticktimingsummary)
  - [`count`](#ticktimingsummary-count)
  - [`overBudget`](#ticktimingsummary-overbudget)
  - [`overDouble`](#ticktimingsummary-overdouble)
  - [`meanS`](#ticktimingsummary-means)
  - [`sigmaS`](#ticktimingsummary-sigmas)
  - [`minS`](#ticktimingsummary-mins)
  - [`maxS`](#ticktimingsummary-maxs)
  - [`valid`](#ticktimingsummary-valid)
- [`struct TickTimingConfig`](#struct-ticktimingconfig)
  - [`nominal`](#ticktimingconfig-nominal)
  - [`budget`](#ticktimingconfig-budget)
- [`class TickTimingStats`](#class-ticktimingstats)
  - [`TickTimingStats`](#ticktimingstats-ticktimingstats)
  - [`tick`](#ticktimingstats-tick)
  - [`rebaseline`](#ticktimingstats-rebaseline)
  - [`reset`](#ticktimingstats-reset)
  - [`summary`](#ticktimingstats-summary)
  - [`config`](#ticktimingstats-config)

<a id="struct-ticktimingsummary"></a>

## `struct TickTimingSummary`

```cpp
struct TickTimingSummary
```

What TickTimingStats measured, as a fixed-width value (the characterization record persists it field by field). Every time is in SECONDS. All zero until two ticks have been seen.

*struct, declared at [`include/shulib/diag/tick_timing_stats.hpp:45`](../../include/shulib/diag/tick_timing_stats.hpp#L45).*

<a id="ticktimingsummary-count"></a>

### `TickTimingSummary::count`

```cpp
std::uint32_t count = 0
```

How many dt values were measured — the ticks AFTER the baseline tick.

*field, declared at [`include/shulib/diag/tick_timing_stats.hpp:47`](../../include/shulib/diag/tick_timing_stats.hpp#L47).*

<a id="ticktimingsummary-overbudget"></a>

### `TickTimingSummary::overBudget`

```cpp
std::uint32_t overBudget = 0
```

How many of them were AT OR OVER the budget (INCLUSIVE — LoopMonitor's boundary).

*field, declared at [`include/shulib/diag/tick_timing_stats.hpp:49`](../../include/shulib/diag/tick_timing_stats.hpp#L49).*

<a id="ticktimingsummary-overdouble"></a>

### `TickTimingSummary::overDouble`

```cpp
std::uint32_t overDouble = 0
```

How many were STRICTLY over twice the nominal period — a tick that missed at least one whole period. Exactly 2× is not counted: that is one missed period to the last bit, and "over" keeps this count from depending on the last bit of a clock read.

*field, declared at [`include/shulib/diag/tick_timing_stats.hpp:53`](../../include/shulib/diag/tick_timing_stats.hpp#L53).*

<a id="ticktimingsummary-means"></a>

### `TickTimingSummary::meanS`

```cpp
double meanS = 0.0
```

The mean dt.

*field, declared at [`include/shulib/diag/tick_timing_stats.hpp:55`](../../include/shulib/diag/tick_timing_stats.hpp#L55).*

<a id="ticktimingsummary-sigmas"></a>

### `TickTimingSummary::sigmaS`

```cpp
double sigmaS = 0.0
```

The SAMPLE standard deviation of dt (divisor count − 1); 0 with fewer than two dt values.

*field, declared at [`include/shulib/diag/tick_timing_stats.hpp:57`](../../include/shulib/diag/tick_timing_stats.hpp#L57).*

<a id="ticktimingsummary-mins"></a>

### `TickTimingSummary::minS`

```cpp
double minS = 0.0
```

The shortest dt.

*field, declared at [`include/shulib/diag/tick_timing_stats.hpp:59`](../../include/shulib/diag/tick_timing_stats.hpp#L59).*

<a id="ticktimingsummary-maxs"></a>

### `TickTimingSummary::maxS`

```cpp
double maxS = 0.0
```

The longest dt.

*field, declared at [`include/shulib/diag/tick_timing_stats.hpp:61`](../../include/shulib/diag/tick_timing_stats.hpp#L61).*

<a id="ticktimingsummary-valid"></a>

### `TickTimingSummary::valid`

```cpp
[[nodiscard]] bool valid() const noexcept
```

True when the summary holds together: every time finite, the spread non-negative, neither count past `count`, min <= max when anything was measured, and all zero when nothing was. The characterization record's decoder refuses a summary that fails this.

*function, declared at [`include/shulib/diag/tick_timing_stats.hpp:66`](../../include/shulib/diag/tick_timing_stats.hpp#L66).*

<a id="struct-ticktimingconfig"></a>

## `struct TickTimingConfig`

```cpp
struct TickTimingConfig
```

The two periods TickTimingStats judges a dt against. Copied at construction.

*struct, declared at [`include/shulib/diag/tick_timing_stats.hpp:79`](../../include/shulib/diag/tick_timing_stats.hpp#L79).*

<a id="ticktimingconfig-nominal"></a>

### `TickTimingConfig::nominal`

```cpp
units::Time nominal{0.01}
```

The loop's intended period. 10 ms — the tick every shulib loop is paced at (A4 register HA-32). Must be > 0.

*field, declared at [`include/shulib/diag/tick_timing_stats.hpp:82`](../../include/shulib/diag/tick_timing_stats.hpp#L82).*

<a id="ticktimingconfig-budget"></a>

### `TickTimingConfig::budget`

```cpp
units::Time budget{0.015}
```

The dt at which a tick counts as late, INCLUSIVE. 15 ms — LoopMonitorConfig's default, so the two tools call the same tick late. Must be strictly greater than `nominal`, for LoopMonitor's reason: a budget equal to the period counts every healthy tick.

*field, declared at [`include/shulib/diag/tick_timing_stats.hpp:86`](../../include/shulib/diag/tick_timing_stats.hpp#L86).*

<a id="class-ticktimingstats"></a>

## `class TickTimingStats`

```cpp
class TickTimingStats
```

The online statistics of a loop's dt (header). Construct once per measured stretch — a segment, a capture — or rebaseline() across a deliberate pause.

*class, declared at [`include/shulib/diag/tick_timing_stats.hpp:91`](../../include/shulib/diag/tick_timing_stats.hpp#L91).*

<a id="ticktimingstats-ticktimingstats"></a>

### `TickTimingStats::TickTimingStats`

```cpp
explicit TickTimingStats(const TickTimingConfig& config = {})
```

Copies `config`. Precondition: nominal finite and > 0, budget finite and > nominal.

*function, declared at [`include/shulib/diag/tick_timing_stats.hpp:94`](../../include/shulib/diag/tick_timing_stats.hpp#L94).*

<a id="ticktimingstats-tick"></a>

### `TickTimingStats::tick`

```cpp
units::Time tick(units::Time now)
```

Record one tick at time `now` and return its dt — or 0 on a baseline tick (the first after construction or rebaseline()), which enters no statistic. Precondition: `now` finite.

*function, declared at [`include/shulib/diag/tick_timing_stats.hpp:104`](../../include/shulib/diag/tick_timing_stats.hpp#L104).*

<a id="ticktimingstats-rebaseline"></a>

### `TickTimingStats::rebaseline`

```cpp
void rebaseline() noexcept
```

The next tick() only baselines, so a DELIBERATE gap (a pause between two captures) is not measured as a tick. Keeps every statistic — LoopMonitor::reset()'s semantics.

*function, declared at [`include/shulib/diag/tick_timing_stats.hpp:136`](../../include/shulib/diag/tick_timing_stats.hpp#L136).*

<a id="ticktimingstats-reset"></a>

### `TickTimingStats::reset`

```cpp
void reset() noexcept
```

Forget everything: the statistics and the baseline.

*function, declared at [`include/shulib/diag/tick_timing_stats.hpp:139`](../../include/shulib/diag/tick_timing_stats.hpp#L139).*

<a id="ticktimingstats-summary"></a>

### `TickTimingStats::summary`

```cpp
[[nodiscard]] TickTimingSummary summary() const noexcept
```

The statistics so far (TickTimingSummary's fields).

*function, declared at [`include/shulib/diag/tick_timing_stats.hpp:142`](../../include/shulib/diag/tick_timing_stats.hpp#L142).*

<a id="ticktimingstats-config"></a>

### `TickTimingStats::config`

```cpp
[[nodiscard]] const TickTimingConfig& config() const noexcept
```

The config in force.

*function, declared at [`include/shulib/diag/tick_timing_stats.hpp:157`](../../include/shulib/diag/tick_timing_stats.hpp#L157).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 32 lines</summary>

```text

 TickTimingStats — the statistics of a loop's tick-to-tick dt, as DATA (chunk R5a Part A, 2026-09-29).

 ── Why this exists beside LoopMonitor ──────────────────────────────────────────────
 LoopMonitor (loop_monitor.hpp) answers ONE question per tick — did this tick reach its budget? —
 and raises LOOP_OVERRUN through the FaultLatch when it did. What it does not keep is the SHAPE of
 the loop's timing: the mean dt, its spread, the fastest and slowest tick, how many ticks were
 late and how many missed a whole period. The hostile suites certify every controller against an
 INVENTED jitter schedule (A4 register HA-32 loop rate, HA-34 jitter — sim/hostile/composed.hpp's
 JitterSchedule: ±20 %, 2 % spikes at 5×), and nothing on the robot has ever measured what the real
 loop does. shulib Calibrate's GAINS and REST steps (calibration/drive_characterizer.hpp,
 calibration/rest_capture.hpp) record these statistics for every segment and capture, so a robot
 session produces the numbers those two register entries wait for — PARTIALLY: Calibrate's loop
 does less work per tick than a motion loop, so its timing is a floor for a motion loop's, not a
 measurement of it (HA-32/34's settle lines say so).

 A SIBLING, not an edit: LoopMonitor is untouched, raises nothing new, and its boundary is the one
 used here — a tick whose dt is AT OR OVER the budget counts (INCLUSIVE; loop_monitor.hpp's header
 gives the deadline-scheduling reason). The first tick after construction or rebaseline() only
 baselines, exactly as LoopMonitor's does: there is no previous tick to difference against, so it
 is not a dt of 0 and never enters the statistics.

 ── Numerics ────────────────────────────────────────────────────────────────────────
 Mean and spread by Welford's online update (mean and the sum of squared deviations about the
 running mean) — one pass, no stored samples, and no Σx² − (Σx)²/n cancellation: at a 10 ms dt the
 naive form subtracts two numbers ~10⁴ times larger than the variance it wants. The test pins the
 mean and σ against a two-pass reference to 1e-12 over a jittered run, and min, max and every count
 EXACTLY.

 PURE and allocation-free: times in, numbers out; it reads no clock (the caller passes `now`, which
 is how the characterizer shares one clock read per tick between everything it measures). Raises
 nothing. Single-task by contract, like the rest of diag/.
```

</details>
