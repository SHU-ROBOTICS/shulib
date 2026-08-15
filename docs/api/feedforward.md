<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/control/feedforward.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `feedforward.hpp`

Feedforward — the kS/kV/kA motor feedforward (master plan §M2): the open-loop voltage to achieve a target velocity + acceleration, so the PID only has to correct the residual.

This header declares **3** types (8 members) and **1** free function.

Extracted from [`include/shulib/control/feedforward.hpp`](../../include/shulib/control/feedforward.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`struct FeedforwardGains`](#struct-feedforwardgains)
  - [`kS`](#feedforwardgains-ks)
  - [`kV`](#feedforwardgains-kv)
  - [`kA`](#feedforwardgains-ka)
- [`class Feedforward`](#class-feedforward)
  - [`Feedforward`](#feedforward-feedforward)
  - [`calculate`](#feedforward-calculate)
  - [`calculate (overload 2)`](#feedforward-calculate-2)
- [`struct CompensatedVoltage`](#struct-compensatedvoltage)
  - [`voltage`](#compensatedvoltage-voltage)
  - [`brownoutLimited`](#compensatedvoltage-brownoutlimited)
- [`compensateForBattery`](#compensateforbattery) — *free function*

<a id="struct-feedforwardgains"></a>

## `struct FeedforwardGains`

```cpp
struct FeedforwardGains
```

The three constants of V = kS·sign(v) + kV·v + kA·a, characterized OFFLINE by sysid — nothing in this library tunes them. They describe ONE wheel, against that wheel's surface speed in in/s. All-zero (the default) is legal and yields 0 V for every request, so an uncharacterized drivetrain goes dead rather than wrong.

*struct, declared at [`include/shulib/control/feedforward.hpp:31`](../../include/shulib/control/feedforward.hpp#L31).*

<a id="feedforwardgains-ks"></a>

### `FeedforwardGains::kS`

```cpp
double kS = 0.0
```

Volts to break static friction. Applied at FULL magnitude with the sign of the commanded velocity and not at all at exactly v == 0, so the law steps by ±kS across zero.

*field, declared at [`include/shulib/control/feedforward.hpp:34`](../../include/shulib/control/feedforward.hpp#L34).*

<a id="feedforwardgains-kv"></a>

### `FeedforwardGains::kV`

```cpp
double kV = 0.0
```

Volt·s/in — volts per in/s of wheel surface speed (the back-EMF term; roughly the rail voltage divided by free speed).

*field, declared at [`include/shulib/control/feedforward.hpp:37`](../../include/shulib/control/feedforward.hpp#L37).*

<a id="feedforwardgains-ka"></a>

### `FeedforwardGains::kA`

```cpp
double kA = 0.0
```

Volt·s²/in — the inertia term. 0 (the default) leaves a steady-state-only feedforward: correct while cruising, and behind by the whole acceleration term on every ramp.

*field, declared at [`include/shulib/control/feedforward.hpp:40`](../../include/shulib/control/feedforward.hpp#L40).*

<a id="class-feedforward"></a>

## `class Feedforward`

```cpp
class Feedforward
```

The open-loop half of the control law: the voltage that should ALREADY hold a wheel at the requested speed, so the PID beside it only has to correct the residual. Stateless and clock-free — the same arguments always give the same volts, and nothing accumulates between calls. One instance serves a whole drivetrain: the command pipeline applies it in turn to each wheel's desaturated speed, because the gains describe a wheel, not a particular motor.

*class, declared at [`include/shulib/control/feedforward.hpp:48`](../../include/shulib/control/feedforward.hpp#L48).*

<a id="feedforward-feedforward"></a>

### `Feedforward::Feedforward`

```cpp
explicit Feedforward(const FeedforwardGains& gains)
```

Copies `gains`; there is no setter, so re-characterizing means constructing a new Feedforward. Rejects a non-finite gain here, at setup, rather than letting a NaN reach a motor command later.

*function, declared at [`include/shulib/control/feedforward.hpp:53`](../../include/shulib/control/feedforward.hpp#L53).*

<a id="feedforward-calculate"></a>

### `Feedforward::calculate`

```cpp
[[nodiscard]] units::Voltage calculate(units::Velocity velocity, units::Acceleration acceleration) const
```

V = kS·sign(v) + kV·v + kA·a for ONE wheel: `velocity` is that wheel's surface speed (in/s), `acceleration` its surface acceleration (in/s²), and the result is volts. Only the kS term follows sign(v) — the SUM need not, and is not meant to: on a hard deceleration kA·a outweighs kS + kV·v and the law asks for voltage AGAINST the direction of travel, which is the braking authority kA exists to supply (kS = 1, kV = 0.5, kA = 0.1 at v = +10 in/s, a = −100 in/s² gives −4 V). Deliberately UNBOUNDED — the result routinely exceeds the rail on an aggressive request, and compensateForBattery() makes it commandable.

*function, declared at [`include/shulib/control/feedforward.hpp:65`](../../include/shulib/control/feedforward.hpp#L65).*

<a id="feedforward-calculate-2"></a>

### `Feedforward::calculate (overload 2)`

```cpp
[[nodiscard]] units::Voltage calculate(units::Velocity velocity) const
```

Cruise form: the same law with acceleration = 0, i.e. the steady-state voltage that HOLDS the wheel at `velocity`. This is the overload the command pipeline uses, because WheelSpeeds carries speeds only — there is no per-wheel acceleration channel to pass.

*function, declared at [`include/shulib/control/feedforward.hpp:76`](../../include/shulib/control/feedforward.hpp#L76).*

<a id="struct-compensatedvoltage"></a>

## `struct CompensatedVoltage`

```cpp
struct CompensatedVoltage
```

What compensateForBattery() returns: the voltage that may actually be commanded, plus whether getting it there cost anything. The pair travels together on purpose — a clamped voltage that arrives without its flag is indistinguishable from a request that simply was not very big.

*struct, declared at [`include/shulib/control/feedforward.hpp:87`](../../include/shulib/control/feedforward.hpp#L87).*

<a id="compensatedvoltage-voltage"></a>

### `CompensatedVoltage::voltage`

```cpp
units::Voltage voltage
```

`desired` clamped into ±battery; safe to hand to IMotor

*field, declared at [`include/shulib/control/feedforward.hpp:88`](../../include/shulib/control/feedforward.hpp#L88).*

<a id="compensatedvoltage-brownoutlimited"></a>

### `CompensatedVoltage::brownoutLimited`

```cpp
bool brownoutLimited
```

True when |desired| exceeded the battery and was cut down — the drive asked for more than the rail could give and is now voltage-starved, not merely slow. Nothing in the library acts on this today (the command pipeline reads only `voltage`); it is the channel a caller reads to tell those two apart.

*field, declared at [`include/shulib/control/feedforward.hpp:93`](../../include/shulib/control/feedforward.hpp#L93).*

<a id="compensateforbattery"></a>

## `compensateForBattery`

```cpp
[[nodiscard]] inline CompensatedVoltage compensateForBattery(units::Voltage desired, units::Voltage battery)
```

Limit `desired` to what `battery` can deliver (±battery), flagging saturation.

*free function, declared at [`include/shulib/control/feedforward.hpp:97`](../../include/shulib/control/feedforward.hpp#L97).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 16 lines</summary>

```text

 Feedforward — the kS/kV/kA motor feedforward (master plan §M2): the open-loop voltage to
 achieve a target velocity + acceleration, so the PID only has to correct the residual.

   V = kS·sign(v) + kV·v + kA·a

 kS [volts] overcomes static friction in the direction of motion; kV [volt·s/in] is the
 back-EMF / velocity term; kA [volt·s²/in] is the inertia / acceleration term. Gains are
 bare doubles (characterized offline by sysid); inputs are TYPED (Velocity, Acceleration)
 and the output is a Voltage — typed at the boundary, like Pid.

 Voltage / brownout compensation: compensateForBattery() limits a desired voltage to what
 the battery can actually deliver (±battery) and flags when it saturated — so the motion
 layer knows it is voltage-starved and the guaranteed end-of-run park still fires as the
 battery collapses (§M2, §18). We command actual voltage (IMotor::setVoltage), so the only
 battery effect is this ceiling; the kV/kS/kA themselves are battery-independent.
```

</details>
