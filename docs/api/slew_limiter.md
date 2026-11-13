<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/teleop/slew_limiter.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `slew_limiter.hpp`

SlewLimiter — the optional rate limit on ONE driver axis.

This header declares **2** types (9 members).

Extracted from [`include/shulib/teleop/slew_limiter.hpp`](../../include/shulib/teleop/slew_limiter.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`struct SlewConfig`](#struct-slewconfig)
  - [`ratePerSecond`](#slewconfig-ratepersecond)
- [`class SlewLimiter`](#class-slewlimiter)
  - [`SlewLimiter`](#slewlimiter-slewlimiter)
  - [`SlewLimiter (overload 2)`](#slewlimiter-slewlimiter-2)
  - [`SlewLimiter (overload 3)`](#slewlimiter-slewlimiter-3)
  - [`update`](#slewlimiter-update)
  - [`reset`](#slewlimiter-reset)
  - [`value`](#slewlimiter-value)
  - [`ratePerSecond`](#slewlimiter-ratepersecond)
  - [`isOff`](#slewlimiter-isoff)

<a id="struct-slewconfig"></a>

## `struct SlewConfig`

```cpp
struct SlewConfig
```

How hard an axis is allowed to be slewed, as a named setting rather than a bare double — so a DriverProfile reads `slew.ratePerSecond` and so per-axis rates can be added later without changing any caller's spelling.  PROVISIONAL (A4: HA-112) — like every other number in the feel layer, the default is INVENTED. Here the invention is the weakest one available: zero, meaning the feature is off and the signal is untouched.

*struct, declared at [`include/shulib/teleop/slew_limiter.hpp:40`](../../include/shulib/teleop/slew_limiter.hpp#L40).*

<a id="slewconfig-ratepersecond"></a>

### `SlewConfig::ratePerSecond`

```cpp
double ratePerSecond = 0.0
```

Maximum change in the limited signal per second. `0` (the default) means OFF: the limiter passes its target through bit for bit. Negative values are read as OFF too — see SlewLimiter::update() for why a nonsensical rate disables rather than inverts.

*field, declared at [`include/shulib/teleop/slew_limiter.hpp:44`](../../include/shulib/teleop/slew_limiter.hpp#L44).*

<a id="class-slewlimiter"></a>

## `class SlewLimiter`

```cpp
class SlewLimiter
```

A first-order rate limit on one scalar signal: the output may move at most `ratePerSecond × dt` per `update()`, in either direction.  ONE double of state (the last output), so three of these cover a DriveRequest's three axes at the cost of three doubles. Stateful on purpose — a rate limit is a memory of what was commanded last tick, and pretending otherwise would push that memory into every caller.  OFF IS OFF: with `ratePerSecond <= 0` (the default), `update(target, dt)` returns `target` itself — not a value that rounds to it. Tests compare bits.  Time is an ARGUMENT, never a reading: the limiter has no clock, so a host test steps it with any dt it likes and the V5 loop hands it the pacer's measured dt. A dt of 0 (or a negative or NaN one) advances nothing, which is the only honest answer when no time has passed.

*class, declared at [`include/shulib/teleop/slew_limiter.hpp:61`](../../include/shulib/teleop/slew_limiter.hpp#L61).*

<a id="slewlimiter-slewlimiter"></a>

### `SlewLimiter::SlewLimiter`

```cpp
constexpr SlewLimiter() noexcept = default
```

A limiter that is OFF (rate 0) and starts from 0.

*function, declared at [`include/shulib/teleop/slew_limiter.hpp:64`](../../include/shulib/teleop/slew_limiter.hpp#L64).*

<a id="slewlimiter-slewlimiter-2"></a>

### `SlewLimiter::SlewLimiter (overload 2)`

```cpp
explicit constexpr SlewLimiter(double ratePerSecond) noexcept
```

A limiter at `ratePerSecond` (units of the limited signal per second), starting from 0. `<= 0` means OFF.

*function, declared at [`include/shulib/teleop/slew_limiter.hpp:68`](../../include/shulib/teleop/slew_limiter.hpp#L68).*

<a id="slewlimiter-slewlimiter-3"></a>

### `SlewLimiter::SlewLimiter (overload 3)`

```cpp
explicit constexpr SlewLimiter(const SlewConfig& config) noexcept
```

A limiter configured from a profile's SlewConfig.

*function, declared at [`include/shulib/teleop/slew_limiter.hpp:71`](../../include/shulib/teleop/slew_limiter.hpp#L71).*

<a id="slewlimiter-update"></a>

### `SlewLimiter::update`

```cpp
constexpr double update(double target, double dtSeconds) noexcept
```

Advance one tick toward `target` and return the limited output.  OFF (`ratePerSecond <= 0`, which includes the default 0 and any nonsensical negative rate): returns `target` unchanged, bit for bit, and adopts it as the current value so that nothing stale is left behind. A negative rate is read as OFF rather than as a backwards limit because a limiter that ran the axis AWAY from its target would be a silent, undebuggable control inversion; OFF is the failure mode a driver can feel and name.  ON: if `dt <= 0` (or is NaN) the value does not move and the previous output is returned — no time has passed, so nothing may change. Otherwise the output steps toward `target` by at most `ratePerSecond × dt`, and SNAPS EXACTLY to `target` when it is within reach — so a held stick settles on the commanded value instead of creeping toward it forever, and a sign reversal is limited THROUGH zero rather than jumping it.

*function, declared at [`include/shulib/teleop/slew_limiter.hpp:88`](../../include/shulib/teleop/slew_limiter.hpp#L88).*

<a id="slewlimiter-reset"></a>

### `SlewLimiter::reset`

```cpp
constexpr void reset(double value = 0.0) noexcept
```

Forget the history and start again from `value` (default 0). What a program calls when control changes hands — a disabled-to-enabled transition, a controller drop, a mode switch — so the next tick is not limited against a stale command from before.

*function, declared at [`include/shulib/teleop/slew_limiter.hpp:111`](../../include/shulib/teleop/slew_limiter.hpp#L111).*

<a id="slewlimiter-value"></a>

### `SlewLimiter::value`

```cpp
[[nodiscard]] constexpr double value() const noexcept
```

The last value `update()` returned (0 before the first call, or whatever `reset()` set).

*function, declared at [`include/shulib/teleop/slew_limiter.hpp:114`](../../include/shulib/teleop/slew_limiter.hpp#L114).*

<a id="slewlimiter-ratepersecond"></a>

### `SlewLimiter::ratePerSecond`

```cpp
[[nodiscard]] constexpr double ratePerSecond() const noexcept
```

The configured rate, in units of the limited signal per second. `<= 0` means OFF.

*function, declared at [`include/shulib/teleop/slew_limiter.hpp:117`](../../include/shulib/teleop/slew_limiter.hpp#L117).*

<a id="slewlimiter-isoff"></a>

### `SlewLimiter::isOff`

```cpp
[[nodiscard]] constexpr bool isOff() const noexcept
```

True when this limiter does nothing at all — the default state, and the state the library ships in. A panel can say "slew: off" without knowing the rate's units.

*function, declared at [`include/shulib/teleop/slew_limiter.hpp:121`](../../include/shulib/teleop/slew_limiter.hpp#L121).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 28 lines</summary>

```text

 SlewLimiter — the optional rate limit on ONE driver axis (chunk T2).

 WHY IT IS OFF BY DEFAULT, AND WHY THAT IS THE WHOLE POINT OF THE FILE.
 A slew limit protects the drivetrain: it turns a stick slammed from -1 to +1
 into a ramp instead of a current spike. It also ADDS LAG, and lag is the enemy
 of feel — a driver who cannot predict the robot's response stops trusting it,
 and an untrusted robot is driven slowly. Those two facts pull in opposite
 directions, so the library refuses to choose silently: the default rate is 0,
 which means OFF, and OFF is structural rather than incidental — `update()`
 returns its argument BIT FOR BIT (same sign of zero, same NaN payload, no
 arithmetic performed at all), so switching the feature on is the only thing
 that can ever change a number.

 UNITS: `ratePerSecond` is in units of the LIMITED SIGNAL per second. Every
 caller in this tree limits a DIMENSIONLESS request axis in [-1, 1]
 (stick_mapping.hpp's DriveRequest), so a rate of 2.0 means "full reverse to
 full forward takes one second" (a span of 2 at 2 per second). Nothing here
 knows about volts, inches or radians.

 NO FEEL CLAIM. No driver has ever felt this code. There is no recommended
 rate, and the default is not a tuned value — it is the absence of a value.

 PURE-ish and PROS-free: one double of state, no clock (dt is passed in), no
 allocation, no exceptions, every member constexpr. It is TOTAL — no
 preconditions, no asserts — because a teleop tick must not be able to abort
 the program; finiteness becomes a precondition where it always did, at
 Chassis::drive (chassis.hpp) and at the motor adapter.
```

</details>
