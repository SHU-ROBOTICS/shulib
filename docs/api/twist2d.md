<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/math/twist2d.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `twist2d.hpp`

Twist2d and ChassisSpeeds — the velocity currencies of the motion stack.

This header declares **2** types (12 members).

Extracted from [`include/shulib/math/twist2d.hpp`](../../include/shulib/math/twist2d.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class Twist2d`](#class-twist2d)
  - [`Twist2d`](#twist2d-twist2d)
  - [`Twist2d (overload 2)`](#twist2d-twist2d-2)
  - [`vx`](#twist2d-vx)
  - [`vy`](#twist2d-vy)
  - [`omega`](#twist2d-omega)
  - [`approxEqual`](#twist2d-approxequal)
- [`class ChassisSpeeds`](#class-chassisspeeds)
  - [`ChassisSpeeds`](#chassisspeeds-chassisspeeds)
  - [`ChassisSpeeds (overload 2)`](#chassisspeeds-chassisspeeds-2)
  - [`vx`](#chassisspeeds-vx)
  - [`vy`](#chassisspeeds-vy)
  - [`omega`](#chassisspeeds-omega)
  - [`approxEqual`](#chassisspeeds-approxequal)

<a id="class-twist2d"></a>

## `class Twist2d`

```cpp
class Twist2d
```

A MEASURED instantaneous pose derivative: (vx, vy, ω) in in/s and rad/s. What an estimator or a forward-kinematics call reports the robot is actually doing.  The FRAME is not carried by the type — whoever produces one states it, and both are in use: `IKinematics::forward()` and the simulator's true twist are BODY frame, while `IPoseSource::twist()` publishes the FIELD-frame derivative of the published pose. Read the producer, never assume.  Distinct from `ChassisSpeeds` only so that a measurement cannot be passed where a command is expected; there is no conversion between them, which is the point.

*class, declared at [`include/shulib/math/twist2d.hpp:30`](../../include/shulib/math/twist2d.hpp#L30).*

<a id="twist2d-twist2d"></a>

### `Twist2d::Twist2d`

```cpp
constexpr Twist2d() = default
```

A stationary twist: all three components exactly zero.

*function, declared at [`include/shulib/math/twist2d.hpp:33`](../../include/shulib/math/twist2d.hpp#L33).*

<a id="twist2d-twist2d-2"></a>

### `Twist2d::Twist2d (overload 2)`

```cpp
constexpr Twist2d(units::Velocity vx, units::Velocity vy, units::AngularVelocity omega) noexcept
```

A stationary twist: all three components exactly zero. Components are taken verbatim in canonical units — in/s, in/s, rad/s (CCW-positive) — and nothing is validated: a rate does not wrap, and non-finite input is the producer's problem.

*function, declared at [`include/shulib/math/twist2d.hpp:36`](../../include/shulib/math/twist2d.hpp#L36).*

<a id="twist2d-vx"></a>

### `Twist2d::vx`

```cpp
[[nodiscard]] constexpr units::Velocity vx() const noexcept
```

Velocity along the frame's +X axis, in/s — forward in a body frame, field +X in a field one.

*function, declared at [`include/shulib/math/twist2d.hpp:40`](../../include/shulib/math/twist2d.hpp#L40).*

<a id="twist2d-vy"></a>

### `Twist2d::vy`

```cpp
[[nodiscard]] constexpr units::Velocity vy() const noexcept
```

Velocity along +Y, in/s — left in a body frame (the field shares that chirality).

*function, declared at [`include/shulib/math/twist2d.hpp:42`](../../include/shulib/math/twist2d.hpp#L42).*

<a id="twist2d-omega"></a>

### `Twist2d::omega`

```cpp
[[nodiscard]] constexpr units::AngularVelocity omega() const noexcept
```

Rotation rate, rad/s, CCW-positive — and frame-invariant, so it survives any frame rotation untouched. An AngularVelocity rather than an Angle because a RATE never wraps.

*function, declared at [`include/shulib/math/twist2d.hpp:45`](../../include/shulib/math/twist2d.hpp#L45).*

<a id="twist2d-approxequal"></a>

### `Twist2d::approxEqual`

```cpp
[[nodiscard]] bool approxEqual(const Twist2d& o, double tol = 1e-9) const noexcept
```

Component-wise ABSOLUTE comparison against a single tolerance, which is therefore read as in/s against vx/vy and as rad/s against omega — one number spanning two units, so pick it for whichever is tighter. `tol == 0` demands exact VALUE equality, which is weaker than bit-equality: -0.0 and +0.0 have different bit patterns and still compare equal, so a zero tolerance will not catch a negated stopped component. A non-finite component compares unequal to everything, itself included — inf minus inf is NaN, and NaN is `<= tol` never.

*function, declared at [`include/shulib/math/twist2d.hpp:53`](../../include/shulib/math/twist2d.hpp#L53).*

<a id="class-chassisspeeds"></a>

## `class ChassisSpeeds`

```cpp
class ChassisSpeeds
```

A COMMANDED chassis velocity: (vx, vy, ω) in in/s and rad/s — what motion asks the drivetrain to do, before any clamp, desaturation or frame rotation.  The frame is NOT part of the type: `Chassis::drive` takes a `math::Frame` alongside it with no default, `fieldToRobot`/`robotToField` map one to the other, and `IKinematics::toWheels` accepts only a BODY-frame one. A ChassisSpeeds on its own therefore says nothing about which way +X points; the parameter next to it does.  Nothing here clamps. Exceeding the drivetrain's capability is legal to construct and is bound later, by the command pipeline (§13 #5: kinematics must never clamp).

*class, declared at [`include/shulib/math/twist2d.hpp:75`](../../include/shulib/math/twist2d.hpp#L75).*

<a id="chassisspeeds-chassisspeeds"></a>

### `ChassisSpeeds::ChassisSpeeds`

```cpp
constexpr ChassisSpeeds() = default
```

A full stop: all three components exactly zero. This is what motions emit on exit.

*function, declared at [`include/shulib/math/twist2d.hpp:78`](../../include/shulib/math/twist2d.hpp#L78).*

<a id="chassisspeeds-chassisspeeds-2"></a>

### `ChassisSpeeds::ChassisSpeeds (overload 2)`

```cpp
constexpr ChassisSpeeds(units::Velocity vx, units::Velocity vy, units::AngularVelocity omega) noexcept
```

A full stop: all three components exactly zero. This is what motions emit on exit. Components are taken verbatim in canonical units — in/s, in/s, rad/s (CCW-positive). Unvalidated here; `Chassis::drive` is where finiteness becomes a precondition.

*function, declared at [`include/shulib/math/twist2d.hpp:81`](../../include/shulib/math/twist2d.hpp#L81).*

<a id="chassisspeeds-vx"></a>

### `ChassisSpeeds::vx`

```cpp
[[nodiscard]] constexpr units::Velocity vx() const noexcept
```

Commanded velocity along the frame's +X axis, in/s — forward when the frame is Body.

*function, declared at [`include/shulib/math/twist2d.hpp:85`](../../include/shulib/math/twist2d.hpp#L85).*

<a id="chassisspeeds-vy"></a>

### `ChassisSpeeds::vy`

```cpp
[[nodiscard]] constexpr units::Velocity vy() const noexcept
```

Commanded velocity along +Y, in/s — strafe-left when the frame is Body, and the component the strafe-authority clamp bounds on a drivetrain that cannot fully honour it.

*function, declared at [`include/shulib/math/twist2d.hpp:88`](../../include/shulib/math/twist2d.hpp#L88).*

<a id="chassisspeeds-omega"></a>

### `ChassisSpeeds::omega`

```cpp
[[nodiscard]] constexpr units::AngularVelocity omega() const noexcept
```

Commanded rotation rate, rad/s, CCW-positive. Frame-invariant, so a Field→Body rotation passes it through unchanged.

*function, declared at [`include/shulib/math/twist2d.hpp:91`](../../include/shulib/math/twist2d.hpp#L91).*

<a id="chassisspeeds-approxequal"></a>

### `ChassisSpeeds::approxEqual`

```cpp
[[nodiscard]] bool approxEqual(const ChassisSpeeds& o, double tol = 1e-9) const noexcept
```

Component-wise ABSOLUTE comparison against a single tolerance, read as in/s against vx/vy and as rad/s against omega — one number spanning two units, so pick it for whichever is tighter. `tol == 0` demands exact VALUE equality, which is weaker than bit-equality: -0.0 and +0.0 have different bit patterns and still compare equal, so a zero tolerance will not catch a desaturation that turned a commanded stop into a negative zero. A non-finite component compares unequal to everything, itself included — inf minus inf is NaN.

*function, declared at [`include/shulib/math/twist2d.hpp:99`](../../include/shulib/math/twist2d.hpp#L99).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 11 lines</summary>

```text

 Twist2d and ChassisSpeeds — the velocity currencies of the motion stack.

 Both carry (vx, vy, ω) with type-safe units: linear velocities are Velocity
 (in/s), the rotation rate is AngularVelocity (rad/s) — NOT an Angle, because
 a rate does not wrap. They are distinct TYPES on purpose:
   * Twist2d       — an instantaneous pose derivative (e.g. from odometry).
   * ChassisSpeeds  — a commanded chassis velocity (what motion asks the
                      drivetrain to do; FIELD frame until Chassis rotates it).
 Keeping them separate stops a measured twist being fed where a command is
 expected, and vice-versa. (master plan §5 data-flow, §6.)
```

</details>
