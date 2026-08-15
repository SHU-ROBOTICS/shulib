<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/diag/motion_result.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `motion_result.hpp`

MotionResult — the per-motion result line, as data + one formatter.

This header declares **2** types (15 members) and **2** free functions.

Extracted from [`include/shulib/diag/motion_result.hpp`](../../include/shulib/diag/motion_result.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`enum class MotionOutcome`](#enum-class-motionoutcome)
  - [`Settled`](#motionoutcome-settled)
  - [`TimedOut`](#motionoutcome-timedout)
  - [`Cancelled`](#motionoutcome-cancelled)
  - [`FaultAbort`](#motionoutcome-faultabort)
  - [`Superseded`](#motionoutcome-superseded)
  - [`Unset`](#motionoutcome-unset)
- [`motionOutcomeName`](#motionoutcomename) — *free function*
- [`struct MotionResult`](#struct-motionresult)
  - [`id`](#motionresult-id)
  - [`name`](#motionresult-name)
  - [`outcome`](#motionresult-outcome)
  - [`abortFault`](#motionresult-abortfault)
  - [`duration`](#motionresult-duration)
  - [`hasPathData`](#motionresult-haspathdata)
  - [`finalPose`](#motionresult-finalpose)
  - [`overshoot`](#motionresult-overshoot)
  - [`drift`](#motionresult-drift)
- [`emitResultLine`](#emitresultline) — *free function*

<a id="enum-class-motionoutcome"></a>

## `enum class MotionOutcome`

```cpp
enum class MotionOutcome : std::uint8_t
```

§18.4's motion exit-reason codes, at the BOUNDARY level (header note). WIRE-STABLE: explicit values, append-only, pinned by test.

*enum class, declared at [`include/shulib/diag/motion_result.hpp:57`](../../include/shulib/diag/motion_result.hpp#L57).*

<a id="motionoutcome-settled"></a>

### `MotionOutcome::Settled`

```cpp
Settled = 0
```

arrived within tolerances — the ✓ case

*enumerator, declared at [`include/shulib/diag/motion_result.hpp:58`](../../include/shulib/diag/motion_result.hpp#L58).*

<a id="motionoutcome-timedout"></a>

### `MotionOutcome::TimedOut`

```cpp
TimedOut = 1
```

the watchdog fired first

*enumerator, declared at [`include/shulib/diag/motion_result.hpp:59`](../../include/shulib/diag/motion_result.hpp#L59).*

<a id="motionoutcome-cancelled"></a>

### `MotionOutcome::Cancelled`

```cpp
Cancelled = 2
```

stopped by the caller (user cancel / panic stop)

*enumerator, declared at [`include/shulib/diag/motion_result.hpp:60`](../../include/shulib/diag/motion_result.hpp#L60).*

<a id="motionoutcome-faultabort"></a>

### `MotionOutcome::FaultAbort`

```cpp
FaultAbort = 3
```

the scheduler's fault policy forced the stop (causal code attached)

*enumerator, declared at [`include/shulib/diag/motion_result.hpp:61`](../../include/shulib/diag/motion_result.hpp#L61).*

<a id="motionoutcome-superseded"></a>

### `MotionOutcome::Superseded`

```cpp
Superseded = 4
```

pre-empted: a newer motion took the slot

*enumerator, declared at [`include/shulib/diag/motion_result.hpp:62`](../../include/shulib/diag/motion_result.hpp#L62).*

<a id="motionoutcome-unset"></a>

### `MotionOutcome::Unset`

```cpp
Unset = 5
```

No producer has written this field yet. APPENDED (value 5, append-only per the enum rule above) and made the DEFAULT, because the previous default was `Settled` — the one value meaning success — so a result line whose producer forgot the field rendered "✓ SETTLED" for a motion that never happened. That is the opposite polarity to this same struct's `hasPathData`, which defaults false precisely so over/drift render "n/a" rather than a fabricated 0.00. There was no value to give the field until this one.

*enumerator, declared at [`include/shulib/diag/motion_result.hpp:69`](../../include/shulib/diag/motion_result.hpp#L69).*

<a id="motionoutcomename"></a>

## `motionOutcomeName`

```cpp
[[nodiscard]] constexpr const char* motionOutcomeName(MotionOutcome outcome) noexcept
```

§18.4 spelling for the line. Never null; out-of-range renders, never crashes.

*free function, declared at [`include/shulib/diag/motion_result.hpp:73`](../../include/shulib/diag/motion_result.hpp#L73).*

<a id="struct-motionresult"></a>

## `struct MotionResult`

```cpp
struct MotionResult
```

One finished motion's result, as the boundary saw it (a value type; the motion-layer glue builds it from CompletedMotion — motion/run_reporter.hpp).

*struct, declared at [`include/shulib/diag/motion_result.hpp:87`](../../include/shulib/diag/motion_result.hpp#L87).*

<a id="motionresult-id"></a>

### `MotionResult::id`

```cpp
std::uint32_t id = 0
```

the command id it ran under

*field, declared at [`include/shulib/diag/motion_result.hpp:88`](../../include/shulib/diag/motion_result.hpp#L88).*

<a id="motionresult-name"></a>

### `MotionResult::name`

```cpp
std::string_view name{}
```

IMotion::name() (stable literal)

*field, declared at [`include/shulib/diag/motion_result.hpp:89`](../../include/shulib/diag/motion_result.hpp#L89).*

<a id="motionresult-outcome"></a>

### `MotionResult::outcome`

```cpp
MotionOutcome outcome = MotionOutcome::Unset
```

How the motion ended. Drives the glanceable pass/fail column — only Settled renders ✓ — and decides whether `abortFault` is meaningful (it is rendered iff this is FaultAbort). Defaults to Unset, the pessimistic value: a record whose producer forgot this field renders "✗ UNSET" rather than the checkmark and SETTLED it used to claim.

*field, declared at [`include/shulib/diag/motion_result.hpp:94`](../../include/shulib/diag/motion_result.hpp#L94).*

<a id="motionresult-abortfault"></a>

### `MotionResult::abortFault`

```cpp
FaultCode abortFault = FaultCode::None
```

causal code iff FaultAbort

*field, declared at [`include/shulib/diag/motion_result.hpp:95`](../../include/shulib/diag/motion_result.hpp#L95).*

<a id="motionresult-duration"></a>

### `MotionResult::duration`

```cpp
units::Time duration{}
```

end − start

*field, declared at [`include/shulib/diag/motion_result.hpp:96`](../../include/shulib/diag/motion_result.hpp#L96).*

<a id="motionresult-haspathdata"></a>

### `MotionResult::hasPathData`

```cpp
bool hasPathData = false
```

record stream flowed (header note)

*field, declared at [`include/shulib/diag/motion_result.hpp:97`](../../include/shulib/diag/motion_result.hpp#L97).*

<a id="motionresult-finalpose"></a>

### `MotionResult::finalPose`

```cpp
math::Pose2d finalPose{}
```

estimate at the boundary (always real)

*field, declared at [`include/shulib/diag/motion_result.hpp:98`](../../include/shulib/diag/motion_result.hpp#L98).*

<a id="motionresult-overshoot"></a>

### `MotionResult::overshoot`

```cpp
units::Length overshoot{}
```

see header; valid iff hasPathData

*field, declared at [`include/shulib/diag/motion_result.hpp:99`](../../include/shulib/diag/motion_result.hpp#L99).*

<a id="motionresult-drift"></a>

### `MotionResult::drift`

```cpp
units::AngleDim drift{}
```

|final heading error|; valid iff hasPathData

*field, declared at [`include/shulib/diag/motion_result.hpp:100`](../../include/shulib/diag/motion_result.hpp#L100).*

<a id="emitresultline"></a>

## `emitResultLine`

```cpp
inline void emitResultLine(hal::ITelemetrySink& sink, const MotionResult& r)
```

Format + log the §18.3 result line (one [MOT] Info line; byte shape pinned by test). ✓ marks SETTLED; every other outcome is ✗ — a glanceable pass/fail column. FAULT_ABORT carries its causal code: "✗FAULT_ABORT=ODO_STUCK".

*free function, declared at [`include/shulib/diag/motion_result.hpp:106`](../../include/shulib/diag/motion_result.hpp#L106).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 40 lines</summary>

```text

 MotionResult — the per-motion result line, as data + one formatter (§18.3/§18.4;
 WS13, chunk C5). The second idea salvaged from the legacy "logging extreme" code,
 rebuilt the §18 way: STRUCTURED FIELDS a formatter renders, never an essay
 emitted from a motion loop.

 Target shape (§18.3, byte-pinned by test/motion_result_test.cpp):

   [t=  12.50] [MOT] MoveToPose#7 ✓SETTLED final(  24.1,  36.0,  90.1°) over  0.20" drift  0.1°   1.16s

 The line rides log() as [MOT]-tagged Info text (the FaultLatch precedent), so it
 lands in the tick stream exactly where the motion ended, in exact §18.3 shape.

 ── The outcome vocabulary is §18.4's, verbatim ─────────────────────────────────────
 §18.4 names the motion exit-reason codes: SETTLED / TIMEOUT / CANCELLED /
 FAULT_ABORT / SUPERSEDED. control::ExitReason deliberately carries only the first
 three verbs plus Running — the scheduler's BOUNDARY knows more than the motion
 does (a Cancelled exit whose cause was the fault policy is FAULT_ABORT; one whose
 cause was pre-emption is SUPERSEDED). This enum is that boundary vocabulary,
 wire-stable like FaultCode's (explicit values, append-only, pinned by test).
 diag/ cannot name motion-layer types (dependency leaf), which is exactly why the
 RESULT vocabulary — not the motion vocabulary — lives here.

 ── Numbers must be TRUE (the brief's constraint 3) ─────────────────────────────────
 hasPathData says whether the record stream actually flowed for this motion (it
 does not with NullSink, and a motion cancelled in the boot window never had a
 live estimate). When false, over/drift render "n/a" — the line NEVER fabricates
 a 0.00 it has no data behind. finalPose is always real (the scheduler reads the
 estimate at the boundary). The reported values are the ESTIMATE's story; the C5
 tests bound estimate-vs-truth divergence across all three drivetrains so "what
 the motion believed" provably tracks "what the motion did".

 Semantics of the two derived quantities (computed by the scheduler's stats sink):
   * over (inches): how far the robot pushed PAST its target — max projection of
     (measured − target) onto the start→target direction, floored at 0. For a
     stationary-target motion (turn/hold: |target − start| < ~0.1"), the direction
     is undefined, so it degrades to max |measured − target| (worst wander from
     the point) — the honest analogue.
   * drift (degrees): |final heading error| — how far the heading ended from the
     target heading (the §18.3 sample's "drift 0.1°").
```

</details>
