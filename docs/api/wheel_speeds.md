<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/kinematics/wheel_speeds.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `wheel_speeds.hpp`

WheelSpeeds — per-wheel linear *surface* speeds (in/s), in a drivetrain-defined wheel order (each IKinematics impl documents its own order).

This header declares **1** type (8 members).

Extracted from [`include/shulib/kinematics/wheel_speeds.hpp`](../../include/shulib/kinematics/wheel_speeds.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class WheelSpeeds`](#class-wheelspeeds)
  - [`kMaxWheels`](#wheelspeeds-kmaxwheels)
  - [`WheelSpeeds`](#wheelspeeds-wheelspeeds)
  - [`WheelSpeeds (overload 2)`](#wheelspeeds-wheelspeeds-2)
  - [`size`](#wheelspeeds-size)
  - [`operator[]`](#wheelspeeds-operator-index)
  - [`set`](#wheelspeeds-set)
  - [`maxMagnitude`](#wheelspeeds-maxmagnitude)
  - [`approxEqual`](#wheelspeeds-approxequal)

<a id="class-wheelspeeds"></a>

## `class WheelSpeeds`

```cpp
class WheelSpeeds
```

A drivetrain's per-wheel linear SURFACE speeds in in/s (units::Velocity, never a bare double — the units wall reaches the motor edge), indexed in that drivetrain's own wheel order. Each IKinematics implementation documents its order; this type does not know which one it is holding, so a set from one drivetrain means nothing to another. A plain value type with fixed inline capacity and no heap, because one of these is produced every control tick. The wheel COUNT is fixed at construction — there is no push or resize, so you build a set of size n and set() into it.

*class, declared at [`include/shulib/kinematics/wheel_speeds.hpp:34`](../../include/shulib/kinematics/wheel_speeds.hpp#L34).*

<a id="wheelspeeds-kmaxwheels"></a>

### `WheelSpeeds::kMaxWheels`

```cpp
static constexpr int kMaxWheels = 8
```

Hard upper bound on wheel count (see header note). Generous on purpose.

*field, declared at [`include/shulib/kinematics/wheel_speeds.hpp:37`](../../include/shulib/kinematics/wheel_speeds.hpp#L37).*

<a id="wheelspeeds-wheelspeeds"></a>

### `WheelSpeeds::WheelSpeeds`

```cpp
WheelSpeeds() = default
```

An empty set (size 0). Used as a default / accumulator seed.

*function, declared at [`include/shulib/kinematics/wheel_speeds.hpp:40`](../../include/shulib/kinematics/wheel_speeds.hpp#L40).*

<a id="wheelspeeds-wheelspeeds-2"></a>

### `WheelSpeeds::WheelSpeeds (overload 2)`

```cpp
explicit WheelSpeeds(int count)
```

A zero-initialized set of `count` wheels. `count` must be in [0, kMaxWheels].

*function, declared at [`include/shulib/kinematics/wheel_speeds.hpp:43`](../../include/shulib/kinematics/wheel_speeds.hpp#L43).*

<a id="wheelspeeds-size"></a>

### `WheelSpeeds::size`

```cpp
[[nodiscard]] int size() const noexcept
```

Wheels in the set, fixed at construction: this IS the valid index range for operator[] and set(), and 0 for a default-constructed set. For anything IKinematics::toWheels() produced it equals that drivetrain's wheelCount().

*function, declared at [`include/shulib/kinematics/wheel_speeds.hpp:51`](../../include/shulib/kinematics/wheel_speeds.hpp#L51).*

<a id="wheelspeeds-operator-index"></a>

### `WheelSpeeds::operator[]`

```cpp
[[nodiscard]] units::Velocity operator[](int i) const
```

The i-th wheel speed. Precondition: 0 <= i < size().

*function, declared at [`include/shulib/kinematics/wheel_speeds.hpp:54`](../../include/shulib/kinematics/wheel_speeds.hpp#L54).*

<a id="wheelspeeds-set"></a>

### `WheelSpeeds::set`

```cpp
void set(int i, units::Velocity speed)
```

Set the i-th wheel speed. Precondition: 0 <= i < size().

*function, declared at [`include/shulib/kinematics/wheel_speeds.hpp:60`](../../include/shulib/kinematics/wheel_speeds.hpp#L60).*

<a id="wheelspeeds-maxmagnitude"></a>

### `WheelSpeeds::maxMagnitude`

```cpp
[[nodiscard]] units::Velocity maxMagnitude() const noexcept
```

Largest |wheel speed| across the set (0 for an empty set). The quantity a uniform desaturation scales against.

*function, declared at [`include/shulib/kinematics/wheel_speeds.hpp:67`](../../include/shulib/kinematics/wheel_speeds.hpp#L67).*

<a id="wheelspeeds-approxequal"></a>

### `WheelSpeeds::approxEqual`

```cpp
[[nodiscard]] bool approxEqual(const WheelSpeeds& o, double tol = 1e-9) const noexcept
```

Element-wise comparison within `tol`, which is an ABSOLUTE tolerance in in/s — not relative, and not a norm over the set: every wheel must agree on its own. Differing sizes compare UNEQUAL rather than tripping a precondition, so it is safe to call across drivetrains. Signs matter: a reversed wheel is not approximately the forward one. A comparison for tests and assertions, not an equivalence relation — tolerance comparison is not transitive.

*function, declared at [`include/shulib/kinematics/wheel_speeds.hpp:81`](../../include/shulib/kinematics/wheel_speeds.hpp#L81).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 14 lines</summary>

```text

 WheelSpeeds — per-wheel linear *surface* speeds (in/s), in a drivetrain-defined
 wheel order (each IKinematics impl documents its own order).

 Why a fixed inline capacity (no heap):
   * cheap and allocation-free on the V5 (this is produced every ~10ms tick),
   * trivially value-typed for host tests.
 kMaxWheels is deliberately generous: a FROZEN contract (F5) should never need a
 version bump merely to gain a wheel. Current drives fit easily — tank=2, H=3,
 X/mecanum=4, swerve drive-speeds=4.

 The element type is units::Velocity, not a bare double, so the units wall (F3)
 reaches all the way to the motor edge: you cannot accidentally feed a length or
 a voltage in where a wheel speed belongs.
```

</details>
