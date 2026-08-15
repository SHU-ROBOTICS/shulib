<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/math/angle.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `angle.hpp`

Angle — a heading on SE(2). The one type that owns angle wrapping, so the "degrees into cos/sin" and "359° vs -1°" bug classes are impossible by construction.

This header declares **1** type (11 members).

Extracted from [`include/shulib/math/angle.hpp`](../../include/shulib/math/angle.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class Angle`](#class-angle)
  - [`kPi`](#angle-kpi)
  - [`Angle`](#angle-angle)
  - [`radians`](#angle-radians)
  - [`degrees`](#angle-degrees)
  - [`radians (overload 2)`](#angle-radians-2)
  - [`degrees (overload 2)`](#angle-degrees-2)
  - [`errorTo`](#angle-errorto)
  - [`operator+`](#angle-operator-plus)
  - [`operator-`](#angle-operator-minus)
  - [`operator- (overload 2)`](#angle-operator-minus-2)
  - [`approxEqual`](#angle-approxequal)

<a id="class-angle"></a>

## `class Angle`

```cpp
class Angle
```

A heading on SE(2), and the one type that owns angle wrapping — so the "degrees into cos/sin" and "359° vs -1°" bug classes cannot be written.  Stored in RADIANS, always normalized to (-π, π], CCW-positive to match the field frame. The interval is half-open at the bottom: an Angle NEVER holds -π, it holds +π instead, which is what makes the exact-antipodal case in errorTo() resolve deterministically. Degrees appear only at this API boundary; nothing downstream computes in them.  Construction goes exclusively through radians() / degrees(), which reject non-finite input, so a NaN heading is unrepresentable rather than merely unlikely. A default-constructed Angle is the zero heading (field +X). The conventions above are LOCKED (Freeze F3).

*class, declared at [`include/shulib/math/angle.hpp:35`](../../include/shulib/math/angle.hpp#L35).*

<a id="angle-kpi"></a>

### `Angle::kPi`

```cpp
static constexpr double kPi = 3.14159265358979323846
```

Pi, exact to double precision. Public so tests/callers share one constant.

*field, declared at [`include/shulib/math/angle.hpp:38`](../../include/shulib/math/angle.hpp#L38).*

<a id="angle-angle"></a>

### `Angle::Angle`

```cpp
Angle() = default
```

The zero heading.

*function, declared at [`include/shulib/math/angle.hpp:41`](../../include/shulib/math/angle.hpp#L41).*

<a id="angle-radians"></a>

### `Angle::radians`

```cpp
[[nodiscard]] static Angle radians(double r)
```

Construct from radians; input must be finite. Result is wrapped to (-π, π].

*function, declared at [`include/shulib/math/angle.hpp:44`](../../include/shulib/math/angle.hpp#L44).*

<a id="angle-degrees"></a>

### `Angle::degrees`

```cpp
[[nodiscard]] static Angle degrees(double d)
```

Construct from degrees; input must be finite. Result is wrapped to (-π, π].

*function, declared at [`include/shulib/math/angle.hpp:50`](../../include/shulib/math/angle.hpp#L50).*

<a id="angle-radians-2"></a>

### `Angle::radians (overload 2)`

```cpp
[[nodiscard]] double radians() const noexcept
```

The heading in radians, already normalized to (-π, π] — never a raw accumulated value, so it is safe to hand straight to cos/sin.

*function, declared at [`include/shulib/math/angle.hpp:57`](../../include/shulib/math/angle.hpp#L57).*

<a id="angle-degrees-2"></a>

### `Angle::degrees (overload 2)`

```cpp
[[nodiscard]] double degrees() const noexcept
```

The same heading in degrees, therefore always in (-180, 180]. For humans and telemetry; no library computation reads this.

*function, declared at [`include/shulib/math/angle.hpp:60`](../../include/shulib/math/angle.hpp#L60).*

<a id="angle-errorto"></a>

### `Angle::errorTo`

```cpp
[[nodiscard]] double errorTo(Angle target) const noexcept
```

Shortest signed rotation FROM this heading TO `target`, in (-π, π] radians. The exact-antipodal case resolves to +π (never -π) — see Freeze F3.

*function, declared at [`include/shulib/math/angle.hpp:64`](../../include/shulib/math/angle.hpp#L64).*

<a id="angle-operator-plus"></a>

### `Angle::operator+`

```cpp
[[nodiscard]] Angle operator+(Angle o) const noexcept
```

This heading rotated FORWARD (CCW) by `o`, re-wrapped. There is no separate rotation type, so the right operand is read as a rotation: 170° + 20° is -170°, not 190°.

*function, declared at [`include/shulib/math/angle.hpp:70`](../../include/shulib/math/angle.hpp#L70).*

<a id="angle-operator-minus"></a>

### `Angle::operator-`

```cpp
[[nodiscard]] Angle operator-(Angle o) const noexcept
```

This heading rotated BACKWARD by `o`, re-wrapped. It yields an Angle, not an error term — for the signed rotation a controller should act on use `o.errorTo(*this)`, which is numerically identical but returns a plain double that cannot be mistaken for a heading.

*function, declared at [`include/shulib/math/angle.hpp:74`](../../include/shulib/math/angle.hpp#L74).*

<a id="angle-operator-minus-2"></a>

### `Angle::operator- (overload 2)`

```cpp
[[nodiscard]] Angle operator-() const noexcept
```

The mirrored heading — the same rotation taken clockwise. Watch the boundary: -π is not representable, so negating +π gives back +π.

*function, declared at [`include/shulib/math/angle.hpp:77`](../../include/shulib/math/angle.hpp#L77).*

<a id="angle-approxequal"></a>

### `Angle::approxEqual`

```cpp
[[nodiscard]] bool approxEqual(Angle o, double tolRad = 1e-9) const noexcept
```

True if the shortest error to `o` is within `tolRad`. Never compares raw doubles with ==, and is correct across the wrap boundary.

*function, declared at [`include/shulib/math/angle.hpp:81`](../../include/shulib/math/angle.hpp#L81).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 15 lines</summary>

```text

 Angle — a heading on SE(2). The one type that owns angle wrapping, so the
 "degrees into cos/sin" and "359° vs -1°" bug classes are impossible by
 construction.

 Conventions (LOCKED — master plan §7 / Freeze F3):
   * Internal unit: RADIANS. Degrees appear only at this API boundary.
   * Normalization interval: (-π, π]  — half-open at -π; +π is the canonical
     boundary value, so an angle NEVER stores -π (it stores +π instead).
   * errorTo() returns the shortest signed rotation, also in (-π, π]; the
     exact-antipodal case (|Δ| == π) resolves deterministically to +π, never -π.
   * CCW-positive, matching the field frame.

 All construction goes through radians()/degrees(), which reject non-finite
 input — a NaN can never enter the type.
```

</details>
