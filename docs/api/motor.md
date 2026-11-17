<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/motor.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `motor.hpp`

IMotor — a single V5 smart motor behind the HAL.

This header declares **2** types (17 members) and **1** constant.

Extracted from [`include/shulib/hal/motor.hpp`](../../include/shulib/hal/motor.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in [How it works](../guide/README.md); worked recipes live in [Recipes](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`kMaxMotorVoltage`](#kmaxmotorvoltage) — *constant*
- [`enum class BrakeMode`](#enum-class-brakemode)
  - [`Coast`](#brakemode-coast)
  - [`Brake`](#brakemode-brake)
  - [`Hold`](#brakemode-hold)
- [`class IMotor`](#class-imotor)
  - [`~IMotor`](#imotor-destructor-imotor)
  - [`IMotor`](#imotor-imotor)
  - [`IMotor (overload 2)`](#imotor-imotor-2)
  - [`IMotor (overload 3)`](#imotor-imotor-3)
  - [`operator=`](#imotor-operator-eq)
  - [`operator= (overload 2)`](#imotor-operator-eq-2)
  - [`setVoltage`](#imotor-setvoltage)
  - [`commandedVoltage`](#imotor-commandedvoltage)
  - [`setBrakeMode`](#imotor-setbrakemode)
  - [`brakeMode`](#imotor-brakemode)
  - [`position`](#imotor-position)
  - [`velocity`](#imotor-velocity)
  - [`current`](#imotor-current)
  - [`temperature`](#imotor-temperature)

<a id="kmaxmotorvoltage"></a>

## `kMaxMotorVoltage`

```cpp
inline constexpr units::Voltage kMaxMotorVoltage{12.0}
```

The V5's own voltage ceiling (canonical volts). setVoltage() clamps to ±this, so it is the FINAL hardware bound — applied after, and never as a substitute for, the upstream wheel-speed desaturation that decides which wheel gives way when the kinematics ask for more than exists.

*constant, declared at [`include/shulib/hal/motor.hpp:26`](../../include/shulib/hal/motor.hpp#L26).*

<a id="enum-class-brakemode"></a>

## `enum class BrakeMode`

```cpp
enum class BrakeMode
```

What a motor does when it is commanded to ZERO. The contract: a stop is "set the mode, then setVoltage(0)" — either order — and the motor then behaves as the mode says; with no mode ever set, or Coast, a zero is simply no voltage. The guaranteed end-of-run park (§M4) and holdPose/driveBrake (§M2) depend on that difference. How it reaches a V5 is the adapter's business, and it is not obvious: PROS applies a mode ONLY through its brake() call — move_voltage() "will not respect brake modes" — so ProsMotor sends a zero under a Brake or Hold the program commanded as brake(), and every other zero as 0 V (hal/pros/motor.hpp, THE STOP; A4 register HA-142). Until DEFECTS2 (2026-09-26) every zero went out as 0 V, so by PROS's own text no Brake or Hold stop in the library applied its mode on a V5. The fakes echo the mode and the A2 plant pins a wheel at 0 V under Brake/Hold: neither is evidence of what a motor does.

*enum class, declared at [`include/shulib/hal/motor.hpp:39`](../../include/shulib/hal/motor.hpp#L39).*

<a id="brakemode-coast"></a>

### `BrakeMode::Coast`

```cpp
Coast
```

PROS: the motor "coasts when stopped". The zero goes out as 0 V and the shaft is left undriven — BELIEVED to free-wheel, which Calibrate's one-sided pulses and the plant's coast-follow model rest on; what a V5 physically does at 0 V is A4 register HA-143 (unknown).

*enumerator, declared at [`include/shulib/hal/motor.hpp:44`](../../include/shulib/hal/motor.hpp#L44).*

<a id="brakemode-brake"></a>

### `BrakeMode::Brake`

```cpp
Brake
```

PROS: the motor "brakes when stopped". Resists being turned while stopped. That it does not return to where it stopped (shove the robot and it stays shoved) is our belief, not PROS's text.

*enumerator, declared at [`include/shulib/hal/motor.hpp:48`](../../include/shulib/hal/motor.hpp#L48).*

<a id="brakemode-hold"></a>

### `BrakeMode::Hold`

```cpp
Hold
```

PROS: the motor "actively holds position when stopped" — the only mode that DEFENDS a position; holdPose/driveBrake and the guaranteed park are built on it. WHICH position is held (believed: the shaft's when the stop was issued — and a repeated stop may re-capture it, A4 register HA-144) and how hard it holds under load (HA-92) are not in PROS's text.

*enumerator, declared at [`include/shulib/hal/motor.hpp:53`](../../include/shulib/hal/motor.hpp#L53).*

<a id="class-imotor"></a>

## `class IMotor`

```cpp
class IMotor
```

One V5 smart motor behind the HAL. The control layer hands it a canonical VOLTAGE and reads back cumulative shaft rotation, speed, current and temperature — everything above this seam is volts / radians / rad/s / amperes / °C, with the V5's native units converted away in the adapter. An implementation holds no control policy of its own: on the COMMAND path it makes exactly two judgements (clamp to ±kMaxMotorVoltage, reject a non-finite volts). The READ path is NOT guaranteed to be a straight pass-through — this seam has no validity channel, so an adapter over real hardware screens an unreadable device instead. ProsMotor does: a sentinel read returns the LAST GOOD position/velocity/current/temperature and increments its faultedReads() count, and temperature() is seeded at 20 °C, so a port that never answered at all reports a room-temperature motor rather than an error. A reading taken from this seam can therefore be FROZEN rather than fresh, which is precisely what the loop's wheels-spin-but-no-motion cross-check (health_monitor.hpp, odomStalled) exists to catch.

*class, declared at [`include/shulib/hal/motor.hpp:68`](../../include/shulib/hal/motor.hpp#L68).*

<a id="imotor-destructor-imotor"></a>

### `IMotor::~IMotor`

```cpp
virtual ~IMotor() = default
```

Virtual so an owner that really does hold a motor polymorphically can destroy it — but nothing in shulib is that owner: RobotContext, MotorMechanism and the motion ops all reach motors through a NON-OWNING `std::span<IMotor* const>` and never delete through it. The CALLER owns the concrete motors, and they must outlive every component they were handed to — in practice, the whole run. The copy/move members are re-defaulted (a user-declared destructor suppresses the implicit MOVEs and deprecates the implicit copies) purely so this base imposes no policy: it holds no state, and what a copy means belongs to the implementation — copying a ProsMotor duplicates a handle to the SAME physical port, along with a second, independently-ageing copy of its last-good read cache.

*function, declared at [`include/shulib/hal/motor.hpp:79`](../../include/shulib/hal/motor.hpp#L79).*

<a id="imotor-imotor"></a>

### `IMotor::IMotor`

```cpp
IMotor() = default
```

*Covered by the comment on [`~IMotor`](#imotor-destructor-imotor) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/motor.hpp:80`](../../include/shulib/hal/motor.hpp#L80).*

<a id="imotor-imotor-2"></a>

### `IMotor::IMotor (overload 2)`

```cpp
IMotor(const IMotor&) = default
```

*Covered by the comment on [`~IMotor`](#imotor-destructor-imotor) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/motor.hpp:81`](../../include/shulib/hal/motor.hpp#L81).*

<a id="imotor-imotor-3"></a>

### `IMotor::IMotor (overload 3)`

```cpp
IMotor(IMotor&&) = default
```

*Covered by the comment on [`~IMotor`](#imotor-destructor-imotor) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/motor.hpp:82`](../../include/shulib/hal/motor.hpp#L82).*

<a id="imotor-operator-eq"></a>

### `IMotor::operator=`

```cpp
IMotor& operator=(const IMotor&) = default
```

*Covered by the comment on [`~IMotor`](#imotor-destructor-imotor) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/motor.hpp:83`](../../include/shulib/hal/motor.hpp#L83).*

<a id="imotor-operator-eq-2"></a>

### `IMotor::operator= (overload 2)`

```cpp
IMotor& operator=(IMotor&&) = default
```

*Covered by the comment on [`~IMotor`](#imotor-destructor-imotor) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/motor.hpp:84`](../../include/shulib/hal/motor.hpp#L84).*

<a id="imotor-setvoltage"></a>

### `IMotor::setVoltage`

```cpp
virtual void setVoltage(units::Voltage volts) = 0
```

Command the motor (canonical volts). Clamped to ±kMaxMotorVoltage; non-finite rejected. VOLTAGE-ONLY by design: shulib owns the feedforward + PID + desaturation + brownout loop, not the V5's onboard velocity PID (§5 data-flow, §M2). A velocity command is deliberately not exposed.

*function, declared at [`include/shulib/hal/motor.hpp:90`](../../include/shulib/hal/motor.hpp#L90).*

<a id="imotor-commandedvoltage"></a>

### `IMotor::commandedVoltage`

```cpp
[[nodiscard]] virtual units::Voltage commandedVoltage() const = 0
```

The voltage actually applied after clamping (telemetry/test readback).

*function, declared at [`include/shulib/hal/motor.hpp:93`](../../include/shulib/hal/motor.hpp#L93).*

<a id="imotor-setbrakemode"></a>

### `IMotor::setBrakeMode`

```cpp
virtual void setBrakeMode(BrakeMode mode) = 0
```

Set the mode a ZERO command stops under (BrakeMode's contract: mode and zero, either order). Hold is the active position hold the guaranteed park relies on; without a mode set, or under Coast, setVoltage(0) is just no voltage.

*function, declared at [`include/shulib/hal/motor.hpp:98`](../../include/shulib/hal/motor.hpp#L98).*

<a id="imotor-brakemode"></a>

### `IMotor::brakeMode`

```cpp
[[nodiscard]] virtual BrakeMode brakeMode() const = 0
```

The mode currently in effect. Not guaranteed to be a pure echo of setBrakeMode(): an adapter over real hardware reads the DEVICE back (ProsMotor does, falling back to the last commanded mode only when the port does not answer), so this is the motor's answer rather than the library's memory of the request.

*function, declared at [`include/shulib/hal/motor.hpp:103`](../../include/shulib/hal/motor.hpp#L103).*

<a id="imotor-position"></a>

### `IMotor::position`

```cpp
[[nodiscard]] virtual units::AngleDim position() const = 0
```

Cumulative output-shaft rotation (NOT wrapped) — total travel for odometry.

*function, declared at [`include/shulib/hal/motor.hpp:106`](../../include/shulib/hal/motor.hpp#L106).*

<a id="imotor-velocity"></a>

### `IMotor::velocity`

```cpp
[[nodiscard]] virtual units::AngularVelocity velocity() const = 0
```

Measured output-shaft angular velocity.

*function, declared at [`include/shulib/hal/motor.hpp:109`](../../include/shulib/hal/motor.hpp#L109).*

<a id="imotor-current"></a>

### `IMotor::current`

```cpp
[[nodiscard]] virtual units::Current current() const = 0
```

Measured current draw (canonical amperes). The PRIMARY capture/stall signal for manipulation sensor-confirm (M4: never advance on a failed grab), dock confirm (M3), stall homing (§8), and the per-wheel I field of the DebugRecord (§18.2). The pros adapter converts mA→A once at the edge.

*function, declared at [`include/shulib/hal/motor.hpp:115`](../../include/shulib/hal/motor.hpp#L115).*

<a id="imotor-temperature"></a>

### `IMotor::temperature`

```cpp
[[nodiscard]] virtual double temperature() const = 0
```

Motor temperature in degrees Celsius (bare double — non-spatial, never combined with other units). Feeds the thermal monitor / thermal fault (§8, §18.4): a V5 motor throttles ~55 °C, corrupting kS/kV/kA, so the control layer must be able to see it.

*function, declared at [`include/shulib/hal/motor.hpp:120`](../../include/shulib/hal/motor.hpp#L120).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 16 lines</summary>

```text

 IMotor — a single V5 smart motor behind the HAL. The control layer commands a
 canonical voltage; the motor reports its output-shaft rotation and angular
 velocity for odometry/telemetry. Units are canonical: volts, radians, rad/s (F3).

 Contract:
  * setVoltage() CLAMPS the command to ±kMaxMotorVoltage (the V5's hard limit) and
    REJECTS a non-finite voltage (red-on-failure host-side). commandedVoltage()
    returns the value actually APPLIED after clamping, so telemetry reflects what
    the motor really got — not an impossible request.
  * position() is CUMULATIVE output-shaft rotation and must NOT wrap at ±π —
    odometry integrates total travel, so it uses the non-wrapping AngleDim, never
    the wrapping math::Angle.

 Wheel-speed saturation is handled upstream (desaturate, §13 #5); this ±12 V clamp
 is a separate, final HARDWARE limit, not a substitute for it.
```

</details>
