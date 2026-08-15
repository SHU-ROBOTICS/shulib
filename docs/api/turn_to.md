<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/motion/turn_to.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `turn_to.hpp`

TurnTo — rotate in place to a FIELD heading.

This header declares **1** type (8 members).

Extracted from [`include/shulib/motion/turn_to.hpp`](../../include/shulib/motion/turn_to.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class TurnTo`](#class-turnto)
  - [`TurnTo`](#turnto-turnto)
  - [`start`](#turnto-start)
  - [`tick`](#turnto-tick)
  - [`cancel`](#turnto-cancel)
  - [`exitReason`](#turnto-exitreason)
  - [`state`](#turnto-state)
  - [`name`](#turnto-name)
  - [`target`](#turnto-target)

<a id="class-turnto"></a>

## `class TurnTo`

```cpp
class TurnTo final : public IMotion
```

Rotate IN PLACE to a FIELD heading: ω from the heading PID, body vx = vy = 0. Both the controller and the exit test are fed math::Angle::errorTo — the shortest signed rotation in (-π, π], with an exact antipode resolving to +π every time — so a target 350° "away" is a 10° error the other way and no raw θ difference ever reaches a gain. The one translation-free primitive with NO drivetrain-authority caveat: every supported drive can rotate, tank included, and the stall cross-check's rotation term keeps a pure turn from reading as ODO_STUCK.

*class, declared at [`include/shulib/motion/turn_to.hpp:46`](../../include/shulib/motion/turn_to.hpp#L46).*

<a id="turnto-turnto"></a>

### `TurnTo::TurnTo`

```cpp
TurnTo(const MotionDeps& deps, math::Angle target, const MotionConfig& config = {}, double timeout = 0.0)
```

Rotate to `target` (FIELD heading). `timeout` (s) bounds the whole motion including any boot wait; 0 selects config.defaultTimeout.

*function, declared at [`include/shulib/motion/turn_to.hpp:50`](../../include/shulib/motion/turn_to.hpp#L50).*

<a id="turnto-start"></a>

### `TurnTo::start`

```cpp
void start() override
```

Arm, or fully re-arm, the turn: PID, stall detector, settle state and watchdog all reset, and the motion re-enters the boot wait. A finished TurnTo is reusable this way — but it is never re-AIMED, since target() is fixed at construction and start() reads nothing from the estimator.

*function, declared at [`include/shulib/motion/turn_to.hpp:77`](../../include/shulib/motion/turn_to.hpp#L77).*

<a id="turnto-tick"></a>

### `TurnTo::tick`

```cpp
[[nodiscard]] control::ExitReason tick() override
```

One control tick, emitting one DebugRecord. Precondition: start() was called, and the caller must have updated the Localizer FIRST — this reads the estimate, it does not advance it. While quality is still Uninitialized it commands zero volts and makes no settle progress, but the WATCHDOG RUNS THROUGH THAT WAIT, so a never-live estimate exits TimedOut (raising MOTION_TIMEOUT) rather than hanging. Settled beats a simultaneous timeout. Once a non-Running verdict is returned the motion is finished: further calls are no-ops that return the cached verdict and leave the motors stopped.

*function, declared at [`include/shulib/motion/turn_to.hpp:95`](../../include/shulib/motion/turn_to.hpp#L95).*

<a id="turnto-cancel"></a>

### `TurnTo::cancel`

```cpp
void cancel() override
```

The cancel contract (motion.hpp): safe state whenever started, verdict only if still running, Idle untouched, idempotent, never raises.

*function, declared at [`include/shulib/motion/turn_to.hpp:166`](../../include/shulib/motion/turn_to.hpp#L166).*

<a id="turnto-exitreason"></a>

### `TurnTo::exitReason`

```cpp
[[nodiscard]] control::ExitReason exitReason() const noexcept override
```

The latched verdict: Running until an exit, then Settled, TimedOut or Cancelled. Once set it is never rewritten — a later cancel() still applies the safe state but preserves this, because a turn that settled really did settle.

*function, declared at [`include/shulib/motion/turn_to.hpp:187`](../../include/shulib/motion/turn_to.hpp#L187).*

<a id="turnto-state"></a>

### `TurnTo::state`

```cpp
[[nodiscard]] MotionState state() const noexcept override
```

The motion-layer state, and the value stamped into DebugRecord.activeCommandState: Idle before start(), WaitingForEstimate through the boot wait, Running once an estimate is live, then whichever exit state matches exitReason().

*function, declared at [`include/shulib/motion/turn_to.hpp:191`](../../include/shulib/motion/turn_to.hpp#L191).*

<a id="turnto-name"></a>

### `TurnTo::name`

```cpp
[[nodiscard]] const char* name() const noexcept override
```

The stable telemetry and result-line id — always the literal "TurnTo", a static string with no lifetime for the caller to manage.

*function, declared at [`include/shulib/motion/turn_to.hpp:194`](../../include/shulib/motion/turn_to.hpp#L194).*

<a id="turnto-target"></a>

### `TurnTo::target`

```cpp
[[nodiscard]] math::Angle target() const noexcept
```

The FIELD heading this instance was built to reach. Fixed for the object's lifetime: a TurnTo is re-armed by start(), never re-aimed, so a new heading means a new TurnTo.

*function, declared at [`include/shulib/motion/turn_to.hpp:197`](../../include/shulib/motion/turn_to.hpp#L197).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 20 lines</summary>

```text

 TurnTo — rotate in place to a FIELD heading (chunk C1).

 A pure heading loop: ω from the heading PID, body vx = vy = 0. The error fed
 to the controller AND to the exit logic is math::Angle::errorTo — F3's
 shortest signed rotation in (-π, π], with the exact-antipodal case resolving
 to +π deterministically. That single choice is what makes the ±180° seam
 safe: a target 350° "away" is an error of −10°, a target at exactly ±180°
 turns CCW (+π) every time, and no raw θ difference ever reaches a gain.

 Exit is the unmodified 1-scalar ExitGroup (SettledUtil on |errH| + Watchdog)
 — constraint 6's plumbing, reused as-is. Everything else (wait-for-live gate,
 stall/health wiring, record emission, F5 output pipeline) matches
 MoveToPose's documented pipeline; the two share idioms deliberately so the
 C4 facade inherits one shape.

 TANK NOTE: rotation is achievable on every supported drive (tank included),
 so TurnTo is the one C1 translation-free primitive with no authority caveat.
 The stall cross-check's rotation term (odo_stall_check.hpp) makes a pure turn
 immune to false ODO_STUCK — pinned by test.
```

</details>
