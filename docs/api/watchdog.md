<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/control/watchdog.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `watchdog.hpp`

Watchdog — a hard timeout primitive.

This header declares **1** type (6 members).

Extracted from [`include/shulib/control/watchdog.hpp`](../../include/shulib/control/watchdog.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class Watchdog`](#class-watchdog)
  - [`Watchdog`](#watchdog-watchdog)
  - [`start`](#watchdog-start)
  - [`elapsed`](#watchdog-elapsed)
  - [`expired`](#watchdog-expired)
  - [`started`](#watchdog-started)
  - [`reset`](#watchdog-reset)

<a id="class-watchdog"></a>

## `class Watchdog`

```cpp
class Watchdog
```

A hard timeout: arm it, then ask whether the deadline has passed (§M2, "a motion can never hang"). CLOCK-driven rather than tick-counting, so a control loop that runs SLOW still times out after the same interval on the injected clock's timeline instead of after some fixed number of ticks — which is what lets a crawling motion exit as TimedOut and its guaranteed end-of-run park fire.  It is a POLLED predicate, not a timer. Nothing here owns a task, a callback or an alarm: expired() computes clock-now minus start-time only when someone asks, and the only callers are the motions' own tick(). A caller that stops asking — a tick() blocked on a deadlocked mutex, a control task that died — is NOT rescued by this class, so the §M2 guarantee reaches exactly as far as the polling does.  The motion layer arms one per motion, but nothing here is motion-specific: it bounds any wait.  Constructed DISARMED — expired() is false and elapsed() is a precondition failure until start(). The clock is held by non-owning reference and must outlive the Watchdog.

*class, declared at [`include/shulib/control/watchdog.hpp:31`](../../include/shulib/control/watchdog.hpp#L31).*

<a id="watchdog-watchdog"></a>

### `Watchdog::Watchdog`

```cpp
Watchdog(double timeout, hal::IClock& clock)
```

`timeout` is in SECONDS and must be FINITE and > 0; a zero or negative deadline is a caller bug, not a request to fire immediately, and an infinite one is a watchdog that can never expire — the single thing this class exists to make impossible. Finiteness was unchecked until DEFECTS1: `> 0.0` is satisfied by infinity, so `cfg.defaultTimeout = inf` built a motion that ran forever with no TimedOut exit. `clock` is stored by reference, never copied — pass the same IClock the surrounding loop reads, so the deadline lives on one timeline (and in a test, on the fake clock the test advances). Does NOT begin counting: call start().

*function, declared at [`include/shulib/control/watchdog.hpp:41`](../../include/shulib/control/watchdog.hpp#L41).*

<a id="watchdog-start"></a>

### `Watchdog::start`

```cpp
void start()
```

(Re)arm: start counting from now.

*function, declared at [`include/shulib/control/watchdog.hpp:47`](../../include/shulib/control/watchdog.hpp#L47).*

<a id="watchdog-elapsed"></a>

### `Watchdog::elapsed`

```cpp
[[nodiscard]] double elapsed() const
```

Seconds since start(). Precondition: started.

*function, declared at [`include/shulib/control/watchdog.hpp:53`](../../include/shulib/control/watchdog.hpp#L53).*

<a id="watchdog-expired"></a>

### `Watchdog::expired`

```cpp
[[nodiscard]] bool expired() const
```

True once `timeout` has elapsed since start(); always false before start().

*function, declared at [`include/shulib/control/watchdog.hpp:59`](../../include/shulib/control/watchdog.hpp#L59).*

<a id="watchdog-started"></a>

### `Watchdog::started`

```cpp
[[nodiscard]] bool started() const noexcept
```

Whether it is armed — i.e. start() has been called and reset() has not. Says nothing about whether the deadline has passed; that is expired().

*function, declared at [`include/shulib/control/watchdog.hpp:65`](../../include/shulib/control/watchdog.hpp#L65).*

<a id="watchdog-reset"></a>

### `Watchdog::reset`

```cpp
void reset() noexcept
```

DISARM, not "restart". expired() reverts to false, elapsed() becomes a precondition failure again, and no deadline is running until the next start(). To begin the countdown again from now, call start() — it re-arms on its own and does not need this first.

*function, declared at [`include/shulib/control/watchdog.hpp:69`](../../include/shulib/control/watchdog.hpp#L69).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 5 lines</summary>

```text

 Watchdog — a hard timeout primitive (master plan §M2: "a motion can never hang"). Clock-
 driven: start() records the clock time; expired() becomes true once `timeout` seconds have
 elapsed. The motion layer arms one per motion, so even a stalled control loop still exits
 (→ TimedOut) and the guaranteed end-of-run park can fire. Reusable for any bounded wait.
```

</details>
