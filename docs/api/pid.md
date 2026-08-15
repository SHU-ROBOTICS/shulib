<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/control/pid.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `pid.hpp`

Pid — a single-axis PID controller.

This header declares **2** types (11 members).

Extracted from [`include/shulib/control/pid.hpp`](../../include/shulib/control/pid.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`struct PidConfig`](#struct-pidconfig)
  - [`kP`](#pidconfig-kp)
  - [`kI`](#pidconfig-ki)
  - [`kD`](#pidconfig-kd)
  - [`integralLimit`](#pidconfig-integrallimit)
  - [`outputMin`](#pidconfig-outputmin)
  - [`outputMax`](#pidconfig-outputmax)
- [`class Pid`](#class-pid)
  - [`Pid`](#pid-pid)
  - [`update`](#pid-update)
  - [`reset`](#pid-reset)
  - [`lastError`](#pid-lasterror)
  - [`integralAccumulator`](#pid-integralaccumulator)

<a id="struct-pidconfig"></a>

## `struct PidConfig`

```cpp
struct PidConfig
```

Gains and the two bounds, every one of them in the CALLER's units (header note): kP multiplies whatever error unit is fed in, and the three terms must sum to the unit the caller wants back. All-default is a controller that returns 0 for every error, with no clamping anywhere.

*struct, declared at [`include/shulib/control/pid.hpp:31`](../../include/shulib/control/pid.hpp#L31).*

<a id="pidconfig-kp"></a>

### `PidConfig::kP`

```cpp
double kP = 0.0
```

Output per unit of error. The only term that ever applies on the first tick.

*field, declared at [`include/shulib/control/pid.hpp:33`](../../include/shulib/control/pid.hpp#L33).*

<a id="pidconfig-ki"></a>

### `PidConfig::kI`

```cpp
double kI = 0.0
```

Output per unit of accumulated error·seconds. Exactly 0 skips integration entirely — the accumulator is not even advanced, so integralAccumulator() stays 0 for a P/PD controller.

*field, declared at [`include/shulib/control/pid.hpp:36`](../../include/shulib/control/pid.hpp#L36).*

<a id="pidconfig-kd"></a>

### `PidConfig::kD`

```cpp
double kD = 0.0
```

Output per unit of error rate, realized as MINUS kD times the measurement's rate of change: a measurement climbing at 1 unit/s with kD = 2 SUBTRACTS 2 from the output. Differentiating the measurement instead of the error is what keeps a setpoint step from kicking D; while the setpoint is held the two agree, because then d(error)/dt = −d(measurement)/dt.

*field, declared at [`include/shulib/control/pid.hpp:41`](../../include/shulib/control/pid.hpp#L41).*

<a id="pidconfig-integrallimit"></a>

### `PidConfig::integralLimit`

```cpp
double integralLimit = std::numeric_limits<double>::infinity()
```

Symmetric ± clamp on the I-TERM (kI·∫e·dt), not on the raw accumulator — the accumulator is then back-calculated to match, which is what makes windup past this bound impossible rather than merely invisible. Must be ≥ 0; the default, infinity, is no anti-windup limit at all.

*field, declared at [`include/shulib/control/pid.hpp:45`](../../include/shulib/control/pid.hpp#L45).*

<a id="pidconfig-outputmin"></a>

### `PidConfig::outputMin`

```cpp
double outputMin = -std::numeric_limits<double>::infinity()
```

Lower clamp on the returned output, applied after P + I + D are summed. Must be ≤ outputMax (checked at construction). Default −infinity: unclamped.

*field, declared at [`include/shulib/control/pid.hpp:48`](../../include/shulib/control/pid.hpp#L48).*

<a id="pidconfig-outputmax"></a>

### `PidConfig::outputMax`

```cpp
double outputMax = std::numeric_limits<double>::infinity()
```

Upper clamp on the returned output. Default +infinity: unclamped.

*field, declared at [`include/shulib/control/pid.hpp:50`](../../include/shulib/control/pid.hpp#L50).*

<a id="class-pid"></a>

## `class Pid`

```cpp
class Pid
```

One axis of PID, distinguished from the textbook loop by three properties the header argues for and the suite pins: derivative on measurement, back-calculated anti-windup, and dt taken from an INJECTED clock instead of read from the OS.  STATEFUL: every update() overwrites the dt baseline and the remembered measurement, and (only when kI != 0) advances the integral, so the output depends on the call history and not on this tick's arguments alone. The first update() after construction or reset() has no baseline and applies P only. A repeat call is NOT automatically a different number, though: with kI == 0 and an unchanged measurement both I and D contribute nothing, so a P or PD controller returns the same output twice. Use one instance per axis, and reset() between motions — otherwise the previous motion's integral rides into the next one.

*class, declared at [`include/shulib/control/pid.hpp:64`](../../include/shulib/control/pid.hpp#L64).*

<a id="pid-pid"></a>

### `Pid::Pid`

```cpp
Pid(const PidConfig& config, hal::IClock& clock)
```

`config` is copied (later edits to the caller's struct do nothing); `clock` is a NON-OWNING reference that must outlive this controller and is the sole source of dt. Rejects non-finite gains, a negative integralLimit and outputMin > outputMax — all at construction, so a controller that cannot be trusted never reaches a match.

*function, declared at [`include/shulib/control/pid.hpp:70`](../../include/shulib/control/pid.hpp#L70).*

<a id="pid-update"></a>

### `Pid::update`

```cpp
[[nodiscard]] double update(double setpoint, double measurement)
```

One control step: returns the clamped control output for (setpoint − measurement).

*function, declared at [`include/shulib/control/pid.hpp:78`](../../include/shulib/control/pid.hpp#L78).*

<a id="pid-reset"></a>

### `Pid::reset`

```cpp
void reset()
```

Clear integral + derivative history (e.g. between motions). Gains/limits unchanged.

*function, declared at [`include/shulib/control/pid.hpp:106`](../../include/shulib/control/pid.hpp#L106).*

<a id="pid-lasterror"></a>

### `Pid::lastError`

```cpp
[[nodiscard]] double lastError() const noexcept
```

setpoint − measurement as of the most recent update(), for telemetry — the law never reads it back. 0 before the first update() and after reset(); recorded on every tick, including the dt ≤ 0 ticks that contribute only P.

*function, declared at [`include/shulib/control/pid.hpp:116`](../../include/shulib/control/pid.hpp#L116).*

<a id="pid-integralaccumulator"></a>

### `Pid::integralAccumulator`

```cpp
[[nodiscard]] double integralAccumulator() const noexcept
```

The raw ∫e·dt in error·seconds, AFTER the anti-windup back-calculation — multiply by kI to recover the I-term that was actually added. Stays exactly 0 when kI == 0 (nothing accumulates) and is zeroed by reset(). Exposed so a test can prove the clamp bounds the accumulator itself and not just the output.

*function, declared at [`include/shulib/control/pid.hpp:122`](../../include/shulib/control/pid.hpp#L122).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 16 lines</summary>

```text

 Pid — a single-axis PID controller (master plan §M2 control). The design choices that
 matter, each pinned by a test:
  * DERIVATIVE ON MEASUREMENT (not on error): D differentiates the measurement, so a
    setpoint step produces NO derivative kick — only real motion drives D.
  * INTEGRAL ANTI-WINDUP: the I-term is clamped to ±integralLimit and the accumulator is
    back-calculated, so it can never wind up past the clamp.
  * OUTPUT CLAMP to [outputMin, outputMax].
  * INJECTED CLOCK for dt — deterministic and host-testable via FakeClock. The first
    update (and any dt ≤ 0 tick) applies P ONLY: no derivative divide-by-zero, no integral
    step on a zero interval.

 Units are bare `double` BY DESIGN: a PID is a generic numeric law; the caller (the motion
 layer) applies it per-axis with matching units (error in inches/radians → output in
 in/s·rad/s or volts) and owns unit consistency. Inputs are assumed finite (guaranteed by
 the HAL finiteness convention, §7).
```

</details>
