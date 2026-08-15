<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/rotation.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `rotation.hpp`

IRotation — a rotation / tracking-wheel sensor (pros::Rotation) behind the HAL.

This header declares **1** type (8 members).

Extracted from [`include/shulib/hal/rotation.hpp`](../../include/shulib/hal/rotation.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class IRotation`](#class-irotation)
  - [`~IRotation`](#irotation-destructor-irotation)
  - [`IRotation`](#irotation-irotation)
  - [`IRotation (overload 2)`](#irotation-irotation-2)
  - [`IRotation (overload 3)`](#irotation-irotation-3)
  - [`operator=`](#irotation-operator-eq)
  - [`operator= (overload 2)`](#irotation-operator-eq-2)
  - [`position`](#irotation-position)
  - [`velocity`](#irotation-velocity)

<a id="class-irotation"></a>

## `class IRotation`

```cpp
class IRotation
```

A rotation / tracking-wheel sensor behind the HAL, in canonical units (radians, rad/s). The PROS adapter converts centidegrees to radians exactly once and NEVER negates: reversal is applied once by PROS itself, decided by the SIGN of the port number the sensor is constructed on, so a consumer re-applies neither: there is no `reversed` boolean anywhere in this API, and the port number is the only place direction is chosen. This seam reports SHAFT rotation, not distance: turning it into travel needs a wheel diameter, localization::TrackingWheel's job. There is NO validity channel here: an implementation must always return a finite, plausible value, so the PROS adapter screens the in-band PROS_ERR sentinel by holding the last good reading rather than propagating it or zeroing.  TWO CORRECTIONS TO THE NEXT SENTENCE, both measured at DEFECTS1 and both left standing here because the sentence is published. (1) The stuck-odometry cross-check reads IMotor, never this seam, and it works in DELTAS — so a pod frozen at 0 and one frozen at 12345 are indistinguishable to it, and the zero is not what hides a dead pod. (2) The screen's "never zero" holds only AFTER a first good read: the PROS adapter's last-good caches start at 0, so a pod that faults from the very first tick publishes 0 rad and 0 rad/s for the whole run, with faultedReads() > 0 beside a zero output as the only tell. The sibling seam (hal/motor.hpp) already carries its cold-start caveat; this one did not.  A frozen reading, not a zeroed one, is what the loop's stuck-odometry cross-check is built to notice.

*class, declared at [`include/shulib/hal/rotation.hpp:36`](../../include/shulib/hal/rotation.hpp#L36).*

<a id="irotation-destructor-irotation"></a>

### `IRotation::~IRotation`

```cpp
virtual ~IRotation() = default
```

All defaulted, and what is worth knowing here is the lifetime rather than the language rule: an implementation is REFERENCED and never owned — a TrackingWheel holds an `hal::IRotation&` and latches its travel baseline off it at construction — so the sensor object must outlive every wheel built on it, and every odometry built on those. The destructor is virtual only so that owning one through an `IRotation*` would still be well-defined; declaring it is what forces the copy and move members to be re-defaulted.

*function, declared at [`include/shulib/hal/rotation.hpp:44`](../../include/shulib/hal/rotation.hpp#L44).*

<a id="irotation-irotation"></a>

### `IRotation::IRotation`

```cpp
IRotation() = default
```

*Covered by the comment on [`~IRotation`](#irotation-destructor-irotation) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/rotation.hpp:45`](../../include/shulib/hal/rotation.hpp#L45).*

<a id="irotation-irotation-2"></a>

### `IRotation::IRotation (overload 2)`

```cpp
IRotation(const IRotation&) = default
```

*Covered by the comment on [`~IRotation`](#irotation-destructor-irotation) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/rotation.hpp:46`](../../include/shulib/hal/rotation.hpp#L46).*

<a id="irotation-irotation-3"></a>

### `IRotation::IRotation (overload 3)`

```cpp
IRotation(IRotation&&) = default
```

*Covered by the comment on [`~IRotation`](#irotation-destructor-irotation) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/rotation.hpp:47`](../../include/shulib/hal/rotation.hpp#L47).*

<a id="irotation-operator-eq"></a>

### `IRotation::operator=`

```cpp
IRotation& operator=(const IRotation&) = default
```

*Covered by the comment on [`~IRotation`](#irotation-destructor-irotation) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/rotation.hpp:48`](../../include/shulib/hal/rotation.hpp#L48).*

<a id="irotation-operator-eq-2"></a>

### `IRotation::operator= (overload 2)`

```cpp
IRotation& operator=(IRotation&&) = default
```

*Covered by the comment on [`~IRotation`](#irotation-destructor-irotation) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/rotation.hpp:49`](../../include/shulib/hal/rotation.hpp#L49).*

<a id="irotation-position"></a>

### `IRotation::position`

```cpp
[[nodiscard]] virtual units::AngleDim position() const = 0
```

Cumulative shaft rotation (NOT wrapped) — total travel for odometry.

*function, declared at [`include/shulib/hal/rotation.hpp:52`](../../include/shulib/hal/rotation.hpp#L52).*

<a id="irotation-velocity"></a>

### `IRotation::velocity`

```cpp
[[nodiscard]] virtual units::AngularVelocity velocity() const = 0
```

Measured shaft angular velocity.

*function, declared at [`include/shulib/hal/rotation.hpp:55`](../../include/shulib/hal/rotation.hpp#L55).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 8 lines</summary>

```text

 IRotation — a rotation / tracking-wheel sensor (pros::Rotation) behind the HAL.
 Reports CUMULATIVE shaft rotation and angular velocity in canonical units
 (radians, rad/s). The hal/pros adapter converts centidegrees→radians exactly
 once and never negates: reversal is PROS's own, from a negative port number.

 position() is CUMULATIVE and must NOT wrap (it uses AngleDim, not the wrapping
 math::Angle): odometry integrates total tracking-wheel travel, not a heading.
```

</details>
