<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/diag/loop_monitor.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `loop_monitor.hpp`

LoopMonitor — loop-overrun / tick-timing detection.

This header declares **2** types (6 members).

Extracted from [`include/shulib/diag/loop_monitor.hpp`](../../include/shulib/diag/loop_monitor.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`struct LoopMonitorConfig`](#struct-loopmonitorconfig)
  - [`budget`](#loopmonitorconfig-budget)
- [`class LoopMonitor`](#class-loopmonitor)
  - [`LoopMonitor`](#loopmonitor-loopmonitor)
  - [`tick`](#loopmonitor-tick)
  - [`worstDt`](#loopmonitor-worstdt)
  - [`overrunCount`](#loopmonitor-overruncount)
  - [`reset`](#loopmonitor-reset)

<a id="struct-loopmonitorconfig"></a>

## `struct LoopMonitorConfig`

```cpp
struct LoopMonitorConfig
```

LoopMonitor's one tuning knob, taken BY VALUE at construction — editing the struct afterwards has no effect on a live monitor. The 15 ms default leaves 5 ms of margin on the nominal 10 ms control loop; see `budget` for why it must not simply equal the tick period.

*struct, declared at [`include/shulib/diag/loop_monitor.hpp:42`](../../include/shulib/diag/loop_monitor.hpp#L42).*

<a id="loopmonitorconfig-budget"></a>

### `LoopMonitorConfig::budget`

```cpp
units::Time budget{0.015}
```

The dt at which a tick counts as an overrun (INCLUSIVE — see header). Must be > 0 and strictly greater than the nominal tick period.

*field, declared at [`include/shulib/diag/loop_monitor.hpp:45`](../../include/shulib/diag/loop_monitor.hpp#L45).*

<a id="class-loopmonitor"></a>

## `class LoopMonitor`

```cpp
class LoopMonitor
```

Loop-overrun detection: it measures the real dt between consecutive tick() calls on the INJECTED clock and raises FaultCode::LoopOverrun through the latch when a tick reaches its budget. It exists because a blown control tick silently corrupts every dt-dependent computation downstream — PID derivative and integral, profile sampling, the odometry twist — which is how a promised sub-degree heading quietly decays into drift nobody can explain. Single-task by contract, like the rest of the diagnostics layer.

*class, declared at [`include/shulib/diag/loop_monitor.hpp:54`](../../include/shulib/diag/loop_monitor.hpp#L54).*

<a id="loopmonitor-loopmonitor"></a>

### `LoopMonitor::LoopMonitor`

```cpp
LoopMonitor(hal::IClock& clock, FaultLatch& faults, const LoopMonitorConfig& config = {})
```

`clock` and `faults` are held BY REFERENCE and must outlive the monitor; `config` is copied. `config.budget` must be > 0 (precondition) and, to be usable at all, strictly greater than the nominal tick period — a tick exactly AT the budget is an overrun, so a 10 ms budget on a 10 ms loop faults on every tick.

*function, declared at [`include/shulib/diag/loop_monitor.hpp:60`](../../include/shulib/diag/loop_monitor.hpp#L60).*

<a id="loopmonitor-tick"></a>

### `LoopMonitor::tick`

```cpp
units::Time tick()
```

Call exactly once per loop iteration. Returns this tick's measured dt (0 on the baseline tick). Raises LOOP_OVERRUN via the latch when dt >= budget.

*function, declared at [`include/shulib/diag/loop_monitor.hpp:67`](../../include/shulib/diag/loop_monitor.hpp#L67).*

<a id="loopmonitor-worstdt"></a>

### `LoopMonitor::worstDt`

```cpp
[[nodiscard]] units::Time worstDt() const noexcept
```

Largest dt observed since construction (the §18.3 "worst loop dt" summary quantity, consumed at C5). Time{0} until two ticks have happened.

*function, declared at [`include/shulib/diag/loop_monitor.hpp:89`](../../include/shulib/diag/loop_monitor.hpp#L89).*

<a id="loopmonitor-overruncount"></a>

### `LoopMonitor::overrunCount`

```cpp
[[nodiscard]] int overrunCount() const noexcept
```

How many ticks have reached the budget since construction. It counts TICKS, not episodes — a loop that stays slow increments (and raises, and logs) once per tick — and reset() does not clear it, so this is a whole-run total. Baseline ticks never count.

*function, declared at [`include/shulib/diag/loop_monitor.hpp:93`](../../include/shulib/diag/loop_monitor.hpp#L93).*

<a id="loopmonitor-reset"></a>

### `LoopMonitor::reset`

```cpp
void reset() noexcept
```

Re-baseline after a DELIBERATE gap (run boundary, pause): the next tick() only baselines, so the gap is not misreported as an overrun. Keeps worstDt/counts.

*function, declared at [`include/shulib/diag/loop_monitor.hpp:97`](../../include/shulib/diag/loop_monitor.hpp#L97).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 26 lines</summary>

```text

 LoopMonitor — loop-overrun / tick-timing detection (master plan §18.4; WS13, chunk A1).

 Why this exists: a blown control tick silently corrupts every dt-dependent computation
 downstream — PID derivative/integral, profile sampling, the odometry twist — which is
 exactly how the promised < 1° heading quietly degrades. The monitor measures the real
 dt between consecutive tick() calls on the injected clock and raises LOOP_OVERRUN
 through the FaultLatch when a tick reaches its budget, so a timing pathology becomes a
 visible fault instead of a mystery drift.

 BOUNDARY SEMANTICS (pinned by test, mutation-checked): a tick whose dt satisfies

     dt >= budget          — INCLUSIVE at the budget —

 is an overrun. The budget is a hard deadline in the deadline-scheduling sense: a loop
 that consumes its entire budget has zero margin left and the next tick already starts
 late, so "exactly at budget" is a miss, not a pass. Consequence for configuration: the
 budget must be strictly GREATER than the nominal tick period (a healthy 10ms loop has
 dt == 10ms every tick; a 10ms budget would fault permanently — use e.g. 15ms).

 The first tick() after construction or reset() only baselines the clock (there is no
 previous tick to difference against) — it can never fault, returns dt = 0, and does
 not count toward worstDt(). reset() exists for deliberate pauses (e.g. between runs)
 so a legitimate gap is not reported as an overrun.

 Single-task by contract, like the rest of diag/ (see fault.hpp's concurrency note).
```

</details>
