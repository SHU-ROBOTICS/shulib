<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/imu.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `imu.hpp`

IImu — the inertial sensor behind the HAL, reporting in shulib's CANONICAL frame (the V5's clockwise/degrees convention is converted away in the hal/pros adapter via imu_conversion.hpp, so everything above this line is CCW-positive radians).

This header declares **1** type (11 members).

Extracted from [`include/shulib/hal/imu.hpp`](../../include/shulib/hal/imu.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class IImu`](#class-iimu)
  - [`~IImu`](#iimu-destructor-iimu)
  - [`IImu`](#iimu-iimu)
  - [`IImu (overload 2)`](#iimu-iimu-2)
  - [`IImu (overload 3)`](#iimu-iimu-3)
  - [`operator=`](#iimu-operator-eq)
  - [`operator= (overload 2)`](#iimu-operator-eq-2)
  - [`heading`](#iimu-heading)
  - [`yawRate`](#iimu-yawrate)
  - [`isReady`](#iimu-isready)
  - [`pitch`](#iimu-pitch)
  - [`roll`](#iimu-roll)

<a id="class-iimu"></a>

## `class IImu`

```cpp
class IImu
```

The inertial sensor seam, reporting in shulib's CANONICAL frame: CCW-positive radians with +X = 0. The V5's clockwise-degrees convention is converted away once, inside the PROS adapter, so no code above this line ever converts an angle again. heading() is the load-bearing sub-degree quantity the whole accuracy claim rests on; every reading is untrustworthy until isReady() returns true. Implementations must not throw and must return finite values.

*class, declared at [`include/shulib/hal/imu.hpp:21`](../../include/shulib/hal/imu.hpp#L21).*

<a id="iimu-destructor-iimu"></a>

### `IImu::~IImu`

```cpp
virtual ~IImu() = default
```

All defaulted, and the fact worth carrying away is a lifetime one rather than a language one: an implementation is REFERENCED and never owned anywhere in this tree — RobotContext keeps a non-owning `IImu*`, and Localizer, PilonsOdometry and both correctors each hold an `IImu&` — so the adapter you construct must outlive every one of them. The destructor is virtual only so that owning one through an `IImu*` would still be well-defined; declaring it is what forces the copy and move members to be re-defaulted here.

*function, declared at [`include/shulib/hal/imu.hpp:29`](../../include/shulib/hal/imu.hpp#L29).*

<a id="iimu-iimu"></a>

### `IImu::IImu`

```cpp
IImu() = default
```

*Covered by the comment on [`~IImu`](#iimu-destructor-iimu) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/imu.hpp:30`](../../include/shulib/hal/imu.hpp#L30).*

<a id="iimu-iimu-2"></a>

### `IImu::IImu (overload 2)`

```cpp
IImu(const IImu&) = default
```

*Covered by the comment on [`~IImu`](#iimu-destructor-iimu) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/imu.hpp:31`](../../include/shulib/hal/imu.hpp#L31).*

<a id="iimu-iimu-3"></a>

### `IImu::IImu (overload 3)`

```cpp
IImu(IImu&&) = default
```

*Covered by the comment on [`~IImu`](#iimu-destructor-iimu) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/imu.hpp:32`](../../include/shulib/hal/imu.hpp#L32).*

<a id="iimu-operator-eq"></a>

### `IImu::operator=`

```cpp
IImu& operator=(const IImu&) = default
```

*Covered by the comment on [`~IImu`](#iimu-destructor-iimu) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/imu.hpp:33`](../../include/shulib/hal/imu.hpp#L33).*

<a id="iimu-operator-eq-2"></a>

### `IImu::operator= (overload 2)`

```cpp
IImu& operator=(IImu&&) = default
```

*Covered by the comment on [`~IImu`](#iimu-destructor-iimu) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/imu.hpp:34`](../../include/shulib/hal/imu.hpp#L34).*

<a id="iimu-heading"></a>

### `IImu::heading`

```cpp
[[nodiscard]] virtual math::Angle heading() const = 0
```

Canonical field heading: CCW-positive, +X = 0, wrapped to (-π, π].

*function, declared at [`include/shulib/hal/imu.hpp:37`](../../include/shulib/hal/imu.hpp#L37).*

<a id="iimu-yawrate"></a>

### `IImu::yawRate`

```cpp
[[nodiscard]] virtual units::AngularVelocity yawRate() const = 0
```

Canonical yaw rate (CCW-positive).

*function, declared at [`include/shulib/hal/imu.hpp:40`](../../include/shulib/hal/imu.hpp#L40).*

<a id="iimu-isready"></a>

### `IImu::isReady`

```cpp
[[nodiscard]] virtual bool isReady() const = 0
```

True once the IMU is calibrated and its readings are trustworthy (false during boot calibration). POSITIVE polarity by convention — every HAL health predicate reads true = usable (cf. IGps::hasFix), so `if (imu.isReady())` can never read backwards.

*function, declared at [`include/shulib/hal/imu.hpp:45`](../../include/shulib/hal/imu.hpp#L45).*

<a id="iimu-pitch"></a>

### `IImu::pitch`

```cpp
[[nodiscard]] virtual math::Angle pitch() const = 0
```

Chassis pitch and roll (canonical, for tip detection).

*function, declared at [`include/shulib/hal/imu.hpp:48`](../../include/shulib/hal/imu.hpp#L48).*

<a id="iimu-roll"></a>

### `IImu::roll`

```cpp
[[nodiscard]] virtual math::Angle roll() const = 0
```

Chassis roll, as a wrapped math::Angle. Unlike heading(), the SIGN is NOT yet a settled convention: the PROS adapter passes the sensor's as-mounted sign through unnegated (open hardware assumption HA-110), so consume the MAGNITUDE until a bench measurement fixes it. That is enough for the tip detection this exists for.

*function, declared at [`include/shulib/hal/imu.hpp:53`](../../include/shulib/hal/imu.hpp#L53).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 8 lines</summary>

```text

 IImu — the inertial sensor behind the HAL, reporting in shulib's CANONICAL frame
 (the V5's clockwise/degrees convention is converted away in the hal/pros adapter
 via imu_conversion.hpp, so everything above this line is CCW-positive radians).

 heading() is the load-bearing < 1° quantity. yawRate() feeds the fused estimate.
 isCalibrating() gates trust at boot (readings are garbage until calibration ends).
 pitch()/roll() exist for tip detection (master plan M2).
```

</details>
