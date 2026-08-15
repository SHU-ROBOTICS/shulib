<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/control/exit_group.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `exit_group.hpp`

ExitReason / ExitGroup — the motion-exit decision.

This header declares **2** types (9 members).

Extracted from [`include/shulib/control/exit_group.hpp`](../../include/shulib/control/exit_group.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`enum class ExitReason`](#enum-class-exitreason)
  - [`Running`](#exitreason-running)
  - [`Settled`](#exitreason-settled)
  - [`TimedOut`](#exitreason-timedout)
  - [`Cancelled`](#exitreason-cancelled)
- [`class ExitGroup`](#class-exitgroup)
  - [`ExitGroup`](#exitgroup-exitgroup)
  - [`start`](#exitgroup-start)
  - [`check`](#exitgroup-check)
  - [`settled`](#exitgroup-settled)
  - [`watchdog`](#exitgroup-watchdog)

<a id="enum-class-exitreason"></a>

## `enum class ExitReason`

```cpp
enum class ExitReason
```

Why a motion stopped — the ONE vocabulary shared by IMotion::exitReason(), the scheduler, the fault path and every logged result line (§18.4 exit-reason codes).

*enum class, declared at [`include/shulib/control/exit_group.hpp:27`](../../include/shulib/control/exit_group.hpp#L27).*

<a id="exitreason-running"></a>

### `ExitReason::Running`

```cpp
Running
```

no exit condition has fired yet: tick() again next loop iteration

*enumerator, declared at [`include/shulib/control/exit_group.hpp:28`](../../include/shulib/control/exit_group.hpp#L28).*

<a id="exitreason-settled"></a>

### `ExitReason::Settled`

```cpp
Settled
```

the settle criteria (error AND its rate) held for their full settle time

*enumerator, declared at [`include/shulib/control/exit_group.hpp:29`](../../include/shulib/control/exit_group.hpp#L29).*

<a id="exitreason-timedout"></a>

### `ExitReason::TimedOut`

```cpp
TimedOut
```

the watchdog deadline passed first — the hang guard, not a tuning knob

*enumerator, declared at [`include/shulib/control/exit_group.hpp:30`](../../include/shulib/control/exit_group.hpp#L30).*

<a id="exitreason-cancelled"></a>

### `ExitReason::Cancelled`

```cpp
Cancelled
```

stopped from outside via IMotion::cancel() (chunk C2; never returned by ExitGroup::check() — see header note)

*enumerator, declared at [`include/shulib/control/exit_group.hpp:31`](../../include/shulib/control/exit_group.hpp#L31).*

<a id="class-exitgroup"></a>

## `class ExitGroup`

```cpp
class ExitGroup
```

Settling (success) and the watchdog (hang guard) as ONE verdict per tick. Settled WINS a tie — a motion that settles on the very tick the deadline passes is a success, not a timeout. The group can only ever return Running / Settled / TimedOut; Cancelled is imposed from outside and never originates here. STATEFUL: check() advances the settle window from the injected clock, so call it exactly once per tick, in order.

*class, declared at [`include/shulib/control/exit_group.hpp:40`](../../include/shulib/control/exit_group.hpp#L40).*

<a id="exitgroup-exitgroup"></a>

### `ExitGroup::ExitGroup`

```cpp
ExitGroup(const SettleConfig& settle, double timeout, hal::IClock& clock)
```

`settle` is applied to whatever error check() is later fed — the motion owns the units. `timeout` is the watchdog deadline in SECONDS and must be > 0 (Watchdog's precondition). `clock` is held BY REFERENCE by both halves and must outlive the group; it is the only time source either uses. Construction arms nothing — start() does.

*function, declared at [`include/shulib/control/exit_group.hpp:46`](../../include/shulib/control/exit_group.hpp#L46).*

<a id="exitgroup-start"></a>

### `ExitGroup::start`

```cpp
void start()
```

Arm the group at the start of a motion.

*function, declared at [`include/shulib/control/exit_group.hpp:50`](../../include/shulib/control/exit_group.hpp#L50).*

<a id="exitgroup-check"></a>

### `ExitGroup::check`

```cpp
[[nodiscard]] ExitReason check(double error)
```

One tick: feed the current error, get the exit verdict.

*function, declared at [`include/shulib/control/exit_group.hpp:56`](../../include/shulib/control/exit_group.hpp#L56).*

<a id="exitgroup-settled"></a>

### `ExitGroup::settled`

```cpp
[[nodiscard]] const SettledUtil& settled() const noexcept
```

The settle half, exposed for telemetry only. isSettled() here is a pure read of the verdict the last check() computed, so it is true EXACTLY when that check() returned Settled — it is not a separate "was it close?" measure, and after a TimedOut exit it reads false by construction (settling is tested first, and losing that test is what let the watchdog branch run at all). Const on purpose: check() is the one way to feed it, so a caller cannot advance the settle window behind the group's back.

*function, declared at [`include/shulib/control/exit_group.hpp:72`](../../include/shulib/control/exit_group.hpp#L72).*

<a id="exitgroup-watchdog"></a>

### `ExitGroup::watchdog`

```cpp
[[nodiscard]] const Watchdog& watchdog() const noexcept
```

The timer half, for inspection only — elapsed() is seconds since start(), which is how long the motion has been running. Const on purpose: start() is the only legal way to (re)arm it.

*function, declared at [`include/shulib/control/exit_group.hpp:77`](../../include/shulib/control/exit_group.hpp#L77).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 16 lines</summary>

```text

 ExitReason / ExitGroup — the motion-exit decision (master plan §M2; §18.4 exit-reason
 codes). A motion exits when it SETTLES (success) or the WATCHDOG fires (TimedOut — the
 hang guard). check() reports WHICH fired, so every IMotion can log a motion exit-reason.
 Settled takes priority over a simultaneous timeout (a motion that settled right at the
 deadline still counts as a success). start() arms both (resets settling, starts the timer).

 More exit conditions (e.g. stall via motor current) are additive later — they slot in as
 extra branches without changing this contract.

 `Cancelled` was appended at chunk C2 via exactly that additive path: it is an exit a
 MOTION reports after IMotion::cancel() (scheduler pre-emption, user cancel, or a
 fault-policy abort) — ExitGroup::check() itself can never return it, because settling
 and timing out are the only verdicts the group's own criteria can render. Cancellation
 is imposed from outside; the enum carries it so every consumer of "why did this motion
 end?" has one vocabulary.
```

</details>
