<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/diag/motion_result.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `motion_result.hpp`

MotionResult — the per-motion result line, as data + one formatter.

This header declares **2** types (17 members) and **2** free functions.

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
  - [`hasCrossTrack`](#motionresult-hascrosstrack)
  - [`crossTrack`](#motionresult-crosstrack)
- [`emitResultLine`](#emitresultline) — *free function*

<a id="enum-class-motionoutcome"></a>

## `enum class MotionOutcome`

```cpp
enum class MotionOutcome : std::uint8_t
```

§18.4's motion exit-reason codes, at the BOUNDARY level (header note). WIRE-STABLE: explicit values, append-only, pinned by test.

*enum class, declared at [`include/shulib/diag/motion_result.hpp:77`](../../include/shulib/diag/motion_result.hpp#L77).*

<a id="motionoutcome-settled"></a>

### `MotionOutcome::Settled`

```cpp
Settled = 0
```

arrived within tolerances — the ✓ case

*enumerator, declared at [`include/shulib/diag/motion_result.hpp:78`](../../include/shulib/diag/motion_result.hpp#L78).*

<a id="motionoutcome-timedout"></a>

### `MotionOutcome::TimedOut`

```cpp
TimedOut = 1
```

the watchdog fired first

*enumerator, declared at [`include/shulib/diag/motion_result.hpp:79`](../../include/shulib/diag/motion_result.hpp#L79).*

<a id="motionoutcome-cancelled"></a>

### `MotionOutcome::Cancelled`

```cpp
Cancelled = 2
```

stopped by the caller (user cancel / panic stop)

*enumerator, declared at [`include/shulib/diag/motion_result.hpp:80`](../../include/shulib/diag/motion_result.hpp#L80).*

<a id="motionoutcome-faultabort"></a>

### `MotionOutcome::FaultAbort`

```cpp
FaultAbort = 3
```

the scheduler's fault policy forced the stop (causal code attached)

*enumerator, declared at [`include/shulib/diag/motion_result.hpp:81`](../../include/shulib/diag/motion_result.hpp#L81).*

<a id="motionoutcome-superseded"></a>

### `MotionOutcome::Superseded`

```cpp
Superseded = 4
```

pre-empted: a newer motion took the slot

*enumerator, declared at [`include/shulib/diag/motion_result.hpp:82`](../../include/shulib/diag/motion_result.hpp#L82).*

<a id="motionoutcome-unset"></a>

### `MotionOutcome::Unset`

```cpp
Unset = 5
```

No producer has written this field yet. APPENDED (value 5, append-only per the enum rule above) and made the DEFAULT, because the previous default was `Settled` — the one value meaning success — so a result line whose producer forgot the field rendered "✓ SETTLED" for a motion that never happened. That is the opposite polarity to this same struct's `hasPathData`, which defaults false precisely so over/drift render "n/a" rather than a fabricated 0.00. There was no value to give the field until this one.

*enumerator, declared at [`include/shulib/diag/motion_result.hpp:89`](../../include/shulib/diag/motion_result.hpp#L89).*

<a id="motionoutcomename"></a>

## `motionOutcomeName`

```cpp
[[nodiscard]] constexpr const char* motionOutcomeName(MotionOutcome outcome) noexcept
```

§18.4 spelling for the line. Never null; out-of-range renders, never crashes.

*free function, declared at [`include/shulib/diag/motion_result.hpp:93`](../../include/shulib/diag/motion_result.hpp#L93).*

<a id="struct-motionresult"></a>

## `struct MotionResult`

```cpp
struct MotionResult
```

One finished motion's result, as the boundary saw it (a value type; the motion-layer glue builds it from CompletedMotion — motion/run_reporter.hpp).

*struct, declared at [`include/shulib/diag/motion_result.hpp:107`](../../include/shulib/diag/motion_result.hpp#L107).*

<a id="motionresult-id"></a>

### `MotionResult::id`

```cpp
std::uint32_t id = 0
```

the command id it ran under

*field, declared at [`include/shulib/diag/motion_result.hpp:108`](../../include/shulib/diag/motion_result.hpp#L108).*

<a id="motionresult-name"></a>

### `MotionResult::name`

```cpp
std::string_view name{}
```

IMotion::name() (stable literal)

*field, declared at [`include/shulib/diag/motion_result.hpp:109`](../../include/shulib/diag/motion_result.hpp#L109).*

<a id="motionresult-outcome"></a>

### `MotionResult::outcome`

```cpp
MotionOutcome outcome = MotionOutcome::Unset
```

How the motion ended. Drives the glanceable pass/fail column — only Settled renders ✓ — and decides whether `abortFault` is meaningful (it is rendered iff this is FaultAbort). Defaults to Unset, the pessimistic value: a record whose producer forgot this field renders "✗ UNSET" rather than the checkmark and SETTLED it used to claim.

*field, declared at [`include/shulib/diag/motion_result.hpp:114`](../../include/shulib/diag/motion_result.hpp#L114).*

<a id="motionresult-abortfault"></a>

### `MotionResult::abortFault`

```cpp
FaultCode abortFault = FaultCode::None
```

causal code iff FaultAbort

*field, declared at [`include/shulib/diag/motion_result.hpp:115`](../../include/shulib/diag/motion_result.hpp#L115).*

<a id="motionresult-duration"></a>

### `MotionResult::duration`

```cpp
units::Time duration{}
```

end − start

*field, declared at [`include/shulib/diag/motion_result.hpp:116`](../../include/shulib/diag/motion_result.hpp#L116).*

<a id="motionresult-haspathdata"></a>

### `MotionResult::hasPathData`

```cpp
bool hasPathData = false
```

record stream flowed (header note)

*field, declared at [`include/shulib/diag/motion_result.hpp:117`](../../include/shulib/diag/motion_result.hpp#L117).*

<a id="motionresult-finalpose"></a>

### `MotionResult::finalPose`

```cpp
math::Pose2d finalPose{}
```

estimate at the boundary (always real)

*field, declared at [`include/shulib/diag/motion_result.hpp:118`](../../include/shulib/diag/motion_result.hpp#L118).*

<a id="motionresult-overshoot"></a>

### `MotionResult::overshoot`

```cpp
units::Length overshoot{}
```

see header; valid iff hasPathData

*field, declared at [`include/shulib/diag/motion_result.hpp:119`](../../include/shulib/diag/motion_result.hpp#L119).*

<a id="motionresult-drift"></a>

### `MotionResult::drift`

```cpp
units::AngleDim drift{}
```

|final heading error|; valid iff hasPathData

*field, declared at [`include/shulib/diag/motion_result.hpp:120`](../../include/shulib/diag/motion_result.hpp#L120).*

<a id="motionresult-hascrosstrack"></a>

### `MotionResult::hasCrossTrack`

```cpp
bool hasCrossTrack = false
```

TANK1 Part B (appended): whether the line carries the ` xt` column — the producer's decision (motion::CompletedMotion::hasCrossTrack says when: a tank motion that set out to translate, with path data). False by default, so a producer that never heard of the column renders the §18.3 line exactly as before. Rendered only together with hasPathData (header).

*field, declared at [`include/shulib/diag/motion_result.hpp:125`](../../include/shulib/diag/motion_result.hpp#L125).*

<a id="motionresult-crosstrack"></a>

### `MotionResult::crossTrack`

```cpp
units::Length crossTrack{}
```

The final signed cross-track error, inches, + to the LEFT of the target heading (header); valid iff hasCrossTrack.

*field, declared at [`include/shulib/diag/motion_result.hpp:128`](../../include/shulib/diag/motion_result.hpp#L128).*

<a id="emitresultline"></a>

## `emitResultLine`

```cpp
inline void emitResultLine(hal::ITelemetrySink& sink, const MotionResult& r)
```

Format + log the §18.3 result line (one [MOT] Info line; byte shape pinned by test). ✓ marks SETTLED; every other outcome is ✗ — a glanceable pass/fail column. FAULT_ABORT carries its causal code: "✗FAULT_ABORT=ODO_STUCK". A tank's translating motion appends ` xt <signed inches>"` after the duration (header); nothing before it moves.

*free function, declared at [`include/shulib/diag/motion_result.hpp:136`](../../include/shulib/diag/motion_result.hpp#L136).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 60 lines, click to expand</summary>

```text

 MotionResult — the per-motion result line, as data + one formatter (§18.3/§18.4;
 WS13, chunk C5). The second idea salvaged from the legacy "logging extreme" code,
 rebuilt the §18 way: STRUCTURED FIELDS a formatter renders, never an essay
 emitted from a motion loop.

 Target shape (§18.3, byte-pinned by test/motion_result_test.cpp):

   [t=  12.50] [MOT] MoveToPose#7 ✓SETTLED final(  24.1,  36.0,  90.1°) over  0.20" drift  0.1°   1.16s

 and, on a TANK, for a motion that set out to translate (TANK1 Part B, API 2.3 — one column
 APPENDED after the duration; every byte before it is the §18.3 shape above, unchanged):

   [t=  12.50] [MOT] MoveToPose#7 ✓SETTLED final(  24.1,  36.0,  90.1°) over  0.20" drift  0.1°   1.16s xt  -0.10"

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
   * xt (inches, SIGNED; TANK1 Part B): the final cross-track error — the last aggregated
     estimate's distance off the target's LINE (the line through the target point along the
     target heading), + to the LEFT of the target heading whichever way the robot drove (the
     `cross` of motion/tank_steer.hpp). Rendered only when the producer says it applies
     (hasCrossTrack): a motion that set out to TRANSLATE on a drivetrain with NO strafe
     authority — a tank — because that is the one sideways error such a drive cannot strafe
     out, so it is the number that says whether a leg that stopped short stopped BESIDE its
     line. Turns, brakes and holds of "here" do not set out to translate; X-drive and H-drive
     lines never carry it (except an H-drive declared with NO strafe authority — a dead strafe
     wheel, strafeAuthority() exactly 0 — which is a tank here as it is to MoveToPose's
     steering). Whatever the outcome: a TIMEOUT, CANCELLED, SUPERSEDED or FAULT_ABORT leg
     carries it too — those are the legs it exists for. Like over/drift it is the record
     stream's number, so it needs hasPathData: with no path data the column is simply absent
     (never a fabricated 0.00 — and never "n/a" either, because without the stream the
     producer cannot tell whether the motion translated at all).
```

</details>
