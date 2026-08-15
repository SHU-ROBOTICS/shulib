<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/clock.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `clock.hpp`

IClock — the single source of "now" for the whole stack.

This header declares **1** type (7 members).

Extracted from [`include/shulib/hal/clock.hpp`](../../include/shulib/hal/clock.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class IClock`](#class-iclock)
  - [`~IClock`](#iclock-destructor-iclock)
  - [`IClock`](#iclock-iclock)
  - [`IClock (overload 2)`](#iclock-iclock-2)
  - [`IClock (overload 3)`](#iclock-iclock-3)
  - [`operator=`](#iclock-operator-eq)
  - [`operator= (overload 2)`](#iclock-operator-eq-2)
  - [`now`](#iclock-now)

<a id="class-iclock"></a>

## `class IClock`

```cpp
class IClock
```

The single source of "now" for the whole stack, in SECONDS (F3). PID dt, motion profiles, the motion watchdog and settling all read time through this one seam, which is what makes every timed behaviour reproducible on the host instead of dependent on how fast the machine ran. The V5 clock — whose milliseconds are converted to seconds exactly once, in the PROS adapter — and the deterministic test clock are just two implementations of it.

*class, declared at [`include/shulib/hal/clock.hpp:23`](../../include/shulib/hal/clock.hpp#L23).*

<a id="iclock-destructor-iclock"></a>

### `IClock::~IClock`

```cpp
virtual ~IClock() = default
```

The seam is READ-ONLY by construction: now() is its only member, so nothing that holds an IClock& can reset it, set it, or sleep on it. Time moves only through an implementation's OWN type — a host test holds the concrete FakeClock, calls advance() on that, and passes the reference down — so "what moved time" has exactly one answer per run. The virtual destructor makes destruction through an IClock* well-defined; the defaulted copy/move set restores the move operations that declaring a destructor suppresses, so a concrete clock stays movable. The interface is abstract and stateless — there is no IClock value to copy.

*function, declared at [`include/shulib/hal/clock.hpp:32`](../../include/shulib/hal/clock.hpp#L32).*

<a id="iclock-iclock"></a>

### `IClock::IClock`

```cpp
IClock() = default
```

*Covered by the comment on [`~IClock`](#iclock-destructor-iclock) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/clock.hpp:33`](../../include/shulib/hal/clock.hpp#L33).*

<a id="iclock-iclock-2"></a>

### `IClock::IClock (overload 2)`

```cpp
IClock(const IClock&) = default
```

*Covered by the comment on [`~IClock`](#iclock-destructor-iclock) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/clock.hpp:34`](../../include/shulib/hal/clock.hpp#L34).*

<a id="iclock-iclock-3"></a>

### `IClock::IClock (overload 3)`

```cpp
IClock(IClock&&) = default
```

*Covered by the comment on [`~IClock`](#iclock-destructor-iclock) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/clock.hpp:35`](../../include/shulib/hal/clock.hpp#L35).*

<a id="iclock-operator-eq"></a>

### `IClock::operator=`

```cpp
IClock& operator=(const IClock&) = default
```

*Covered by the comment on [`~IClock`](#iclock-destructor-iclock) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/clock.hpp:36`](../../include/shulib/hal/clock.hpp#L36).*

<a id="iclock-operator-eq-2"></a>

### `IClock::operator= (overload 2)`

```cpp
IClock& operator=(IClock&&) = default
```

*Covered by the comment on [`~IClock`](#iclock-destructor-iclock) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/clock.hpp:37`](../../include/shulib/hal/clock.hpp#L37).*

<a id="iclock-now"></a>

### `IClock::now`

```cpp
[[nodiscard]] virtual units::Time now() const = 0
```

Seconds elapsed since a fixed per-run epoch. Monotonic non-decreasing.

*function, declared at [`include/shulib/hal/clock.hpp:40`](../../include/shulib/hal/clock.hpp#L40).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 11 lines</summary>

```text

 IClock — the single source of "now" for the whole stack. Injecting time (rather
 than calling pros::millis() / the OS directly) is what makes every timed
 behavior — PID dt, motion profiles, the motion watchdog, settling — DETERMINISTIC
 and host-testable. The real V5 clock and a deterministic test clock are just two
 implementations of this one interface.

 Canonical time unit is SECONDS (§7, F3). The V5's milliseconds are converted to
 seconds exactly once, in the hal/pros adapter — ms never leak upward.

 Contract: now() is MONOTONIC (never decreases) within a run.
```

</details>
