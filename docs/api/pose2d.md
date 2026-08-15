<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/math/pose2d.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `pose2d.hpp`

Pose2d — a rigid-body pose on SE(2): position (x, y) + heading.

This header declares **1** type (6 members).

Extracted from [`include/shulib/math/pose2d.hpp`](../../include/shulib/math/pose2d.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class Pose2d`](#class-pose2d)
  - [`Pose2d`](#pose2d-pose2d)
  - [`Pose2d (overload 2)`](#pose2d-pose2d-2)
  - [`x`](#pose2d-x)
  - [`y`](#pose2d-y)
  - [`heading`](#pose2d-heading)
  - [`approxEqual`](#pose2d-approxequal)

<a id="class-pose2d"></a>

## `class Pose2d`

```cpp
class Pose2d
```

A rigid-body pose on SE(2): position plus heading. The type carries NO frame tag — nearly every Pose2d in shulib is a FIELD pose (F1: +X right, +Y away from the red driver station, heading 0 along +X, CCW-positive), but a body-frame pose uses this same type (an AprilTag's pose relative to the robot is one), so which frame a given Pose2d is in is a fact about the variable, not about the type. Position is a type-safe Length in canonical inches and heading is the WRAPPING Angle, so comparing two poses is correct across the ±180° seam rather than reporting 358° of error.

*class, declared at [`include/shulib/math/pose2d.hpp:23`](../../include/shulib/math/pose2d.hpp#L23).*

<a id="pose2d-pose2d"></a>

### `Pose2d::Pose2d`

```cpp
constexpr Pose2d() = default
```

The origin at heading 0. NOT a "no pose" sentinel — it is indistinguishable from a robot genuinely sitting at the origin, which is why the Localizer publishes a separate `Quality` (Uninitialized / DeadReckon / …) instead of overloading a default pose.

*function, declared at [`include/shulib/math/pose2d.hpp:28`](../../include/shulib/math/pose2d.hpp#L28).*

<a id="pose2d-pose2d-2"></a>

### `Pose2d::Pose2d (overload 2)`

```cpp
constexpr Pose2d(units::Length x, units::Length y, Angle heading) noexcept
```

The origin at heading 0. NOT a "no pose" sentinel — it is indistinguishable from a robot genuinely sitting at the origin, which is why the Localizer publishes a separate `Quality` (Uninitialized / DeadReckon / …) instead of overloading a default pose. Position in canonical inches; `heading` is already a wrapped Angle so nothing is normalized here. NO validation: a non-finite Length passes straight through (Angle's factories reject non-finite input, Length has no such guard), so callers that care screen it themselves — the Localizer runs std::isfinite over a corrector's proposed x/y before fusing it.

*function, declared at [`include/shulib/math/pose2d.hpp:34`](../../include/shulib/math/pose2d.hpp#L34).*

<a id="pose2d-x"></a>

### `Pose2d::x`

```cpp
[[nodiscard]] constexpr units::Length x() const noexcept
```

The x coordinate, in canonical inches. In the field frame +X points right, and heading 0 points along it — so x is the "forward" axis for a robot at heading 0.

*function, declared at [`include/shulib/math/pose2d.hpp:39`](../../include/shulib/math/pose2d.hpp#L39).*

<a id="pose2d-y"></a>

### `Pose2d::y`

```cpp
[[nodiscard]] constexpr units::Length y() const noexcept
```

The y coordinate, in canonical inches. In the field frame +Y points away from the red driver station; with CCW-positive heading that pins the whole sign convention.

*function, declared at [`include/shulib/math/pose2d.hpp:42`](../../include/shulib/math/pose2d.hpp#L42).*

<a id="pose2d-heading"></a>

### `Pose2d::heading`

```cpp
[[nodiscard]] constexpr Angle heading() const noexcept
```

The heading, as the WRAPPING Angle: already normalized to (-π, π], CCW-positive, 0 along +X. Compare it with Angle::errorTo / approxEqual — differencing radians() as raw doubles reintroduces exactly the ±180° seam bug this type exists to make impossible.

*function, declared at [`include/shulib/math/pose2d.hpp:46`](../../include/shulib/math/pose2d.hpp#L46).*

<a id="pose2d-approxequal"></a>

### `Pose2d::approxEqual`

```cpp
[[nodiscard]] bool approxEqual(const Pose2d& o, units::Length posTol = units::Length{1e-6}, double headTolRad = 1e-9) const noexcept
```

Component-wise closeness. Heading uses the SHORTEST angular error, so 179° vs -179° are 2° apart (not 358°) — comparison respects the wrap.

*function, declared at [`include/shulib/math/pose2d.hpp:50`](../../include/shulib/math/pose2d.hpp#L50).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 6 lines</summary>

```text

 Pose2d — a rigid-body pose on SE(2): position (x, y) + heading.

 Position is type-safe Length (canonical inches); heading is the wrapping
 Angle, so pose comparison is correct across the ±180° seam. Built only on
 shulib::math::Angle and shulib::units (master plan §6, frame per §7 / F1).
```

</details>
