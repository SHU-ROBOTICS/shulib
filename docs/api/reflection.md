<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/math/reflection.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `reflection.hpp`

math::Reflection — the field plane reflected across a line through the origin, or left as it is.

This header declares **3** types (19 members).

Extracted from [`include/shulib/math/reflection.hpp`](../../include/shulib/math/reflection.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class Reflection`](#class-reflection)
  - [`Reflection`](#reflection-reflection)
  - [`identity`](#reflection-identity)
  - [`across`](#reflection-across)
  - [`acrossLineAt`](#reflection-acrosslineat)
  - [`isIdentity`](#reflection-isidentity)
  - [`line`](#reflection-line)
  - [`operator==`](#reflection-operator-eq-eq)
  - [`apply`](#reflection-apply)
  - [`apply (overload 2)`](#reflection-apply-2)
  - [`apply (overload 3)`](#reflection-apply-3)
  - [`apply (overload 4)`](#reflection-apply-4)
  - [`apply (overload 5)`](#reflection-apply-5)
  - [`enum class Reflection::Line`](#enum-class-reflection-line)
    - [`None`](#reflection-line-none)
    - [`XAxis`](#reflection-line-xaxis)
    - [`Diagonal`](#reflection-line-diagonal)
    - [`YAxis`](#reflection-line-yaxis)
    - [`AntiDiagonal`](#reflection-line-antidiagonal)
  - [`struct Reflection::Point`](#struct-reflection-point)
    - [`x`](#reflection-point-x)
    - [`y`](#reflection-point-y)

<a id="class-reflection"></a>

## `class Reflection`

```cpp
class Reflection
```

The field plane reflected across a line through the origin at a multiple of 45 degrees, or the identity (the default) — the file banner's table. Exact in position, idempotent in pairs (a reflection applied twice gives positions back equal), never produces -0.0.

*class, declared at [`include/shulib/math/reflection.hpp:58`](../../include/shulib/math/reflection.hpp#L58).*

<a id="reflection-reflection"></a>

### `Reflection::Reflection`

```cpp
constexpr Reflection() noexcept = default
```

The identity.

*function, declared at [`include/shulib/math/reflection.hpp:76`](../../include/shulib/math/reflection.hpp#L76).*

<a id="reflection-identity"></a>

### `Reflection::identity`

```cpp
[[nodiscard]] static constexpr Reflection identity() noexcept
```

The identity, by name.

*function, declared at [`include/shulib/math/reflection.hpp:79`](../../include/shulib/math/reflection.hpp#L79).*

<a id="reflection-across"></a>

### `Reflection::across`

```cpp
[[nodiscard]] static constexpr Reflection across(Line line) noexcept
```

The reflection across `line` (Line::None is the identity).

*function, declared at [`include/shulib/math/reflection.hpp:82`](../../include/shulib/math/reflection.hpp#L82).*

<a id="reflection-acrosslineat"></a>

### `Reflection::acrossLineAt`

```cpp
[[nodiscard]] static Reflection acrossLineAt(Angle heading)
```

The reflection across the line through the origin at `heading` (either direction along it: -45 deg and 135 deg are the same line). PRECONDITION: `heading` is a multiple of 45 degrees (within 1e-9 deg) — any other line would need irrational coefficients and lose exactness.

*function, declared at [`include/shulib/math/reflection.hpp:87`](../../include/shulib/math/reflection.hpp#L87).*

<a id="reflection-isidentity"></a>

### `Reflection::isIdentity`

```cpp
[[nodiscard]] constexpr bool isIdentity() const noexcept
```

True for the identity (nothing is reflected).

*function, declared at [`include/shulib/math/reflection.hpp:103`](../../include/shulib/math/reflection.hpp#L103).*

<a id="reflection-line"></a>

### `Reflection::line`

```cpp
[[nodiscard]] constexpr Line line() const noexcept
```

The line (Line::None for the identity).

*function, declared at [`include/shulib/math/reflection.hpp:105`](../../include/shulib/math/reflection.hpp#L105).*

<a id="reflection-operator-eq-eq"></a>

### `Reflection::operator==`

```cpp
[[nodiscard]] friend constexpr bool operator==(const Reflection& a, const Reflection& b) noexcept
```

Two reflections are equal when they are across the same line.

*function, declared at [`include/shulib/math/reflection.hpp:108`](../../include/shulib/math/reflection.hpp#L108).*

<a id="reflection-apply"></a>

### `Reflection::apply`

```cpp
[[nodiscard]] constexpr Point apply(units::Length x, units::Length y) const noexcept
```

The point (x, y) reflected — EXACT (the coefficients are 0 / +-1), never -0.0. The identity returns it untouched.

*function, declared at [`include/shulib/math/reflection.hpp:114`](../../include/shulib/math/reflection.hpp#L114).*

<a id="reflection-apply-2"></a>

### `Reflection::apply (overload 2)`

```cpp
[[nodiscard]] constexpr Point apply(const Point& p) const noexcept
```

The point `p` reflected (as apply(x, y)).

*function, declared at [`include/shulib/math/reflection.hpp:127`](../../include/shulib/math/reflection.hpp#L127).*

<a id="reflection-apply-3"></a>

### `Reflection::apply (overload 3)`

```cpp
[[nodiscard]] Angle apply(Angle h) const
```

The heading `h` reflected: 2a - h for the line at heading a, wrapped into (-pi, pi] (F3), never -0.0. The identity returns it untouched.

*function, declared at [`include/shulib/math/reflection.hpp:131`](../../include/shulib/math/reflection.hpp#L131).*

<a id="reflection-apply-4"></a>

### `Reflection::apply (overload 4)`

```cpp
[[nodiscard]] Pose2d apply(const Pose2d& p) const
```

The pose `p` reflected: its position and its heading. The identity returns it untouched.

*function, declared at [`include/shulib/math/reflection.hpp:142`](../../include/shulib/math/reflection.hpp#L142).*

<a id="reflection-apply-5"></a>

### `Reflection::apply (overload 5)`

```cpp
[[nodiscard]] ChassisSpeeds apply(const ChassisSpeeds& s, Frame frame) const noexcept
```

A chassis velocity in `frame` reflected (the file banner): FIELD — (vx, vy) as a point, the turn rate negated; BODY — (vx, -vy, -omega) for every line. Never -0.0. The identity returns it untouched.

*function, declared at [`include/shulib/math/reflection.hpp:153`](../../include/shulib/math/reflection.hpp#L153).*

<a id="enum-class-reflection-line"></a>

## `enum class Reflection::Line`

```cpp
enum class Line : std::uint8_t
```

The line the reflection is across (the file banner's table); None = the identity.

*enum class, declared at [`include/shulib/math/reflection.hpp:61`](../../include/shulib/math/reflection.hpp#L61).*

<a id="reflection-line-none"></a>

### `Reflection::Line::None`

```cpp
None = 0
```

the identity: nothing is reflected

*enumerator, declared at [`include/shulib/math/reflection.hpp:62`](../../include/shulib/math/reflection.hpp#L62).*

<a id="reflection-line-xaxis"></a>

### `Reflection::Line::XAxis`

```cpp
XAxis = 1
```

y = 0: (x, -y, -h) — also the robot-frame mirror ("every left a right")

*enumerator, declared at [`include/shulib/math/reflection.hpp:63`](../../include/shulib/math/reflection.hpp#L63).*

<a id="reflection-line-diagonal"></a>

### `Reflection::Line::Diagonal`

```cpp
Diagonal = 2
```

y = x: (y, x, 90 deg - h)

*enumerator, declared at [`include/shulib/math/reflection.hpp:64`](../../include/shulib/math/reflection.hpp#L64).*

<a id="reflection-line-yaxis"></a>

### `Reflection::Line::YAxis`

```cpp
YAxis = 3
```

x = 0: (-x, y, 180 deg - h)

*enumerator, declared at [`include/shulib/math/reflection.hpp:65`](../../include/shulib/math/reflection.hpp#L65).*

<a id="reflection-line-antidiagonal"></a>

### `Reflection::Line::AntiDiagonal`

```cpp
AntiDiagonal = 4
```

y = -x: (-y, -x, -90 deg - h) — Override's skills mirror

*enumerator, declared at [`include/shulib/math/reflection.hpp:66`](../../include/shulib/math/reflection.hpp#L66).*

<a id="struct-reflection-point"></a>

## `struct Reflection::Point`

```cpp
struct Point
```

A field point (inches) — what a reflection maps a position to.

*struct, declared at [`include/shulib/math/reflection.hpp:70`](../../include/shulib/math/reflection.hpp#L70).*

<a id="reflection-point-x"></a>

### `Reflection::Point::x`

```cpp
units::Length x{}
```

field x, inches

*field, declared at [`include/shulib/math/reflection.hpp:71`](../../include/shulib/math/reflection.hpp#L71).*

<a id="reflection-point-y"></a>

### `Reflection::Point::y`

```cpp
units::Length y{}
```

field y, inches

*field, declared at [`include/shulib/math/reflection.hpp:72`](../../include/shulib/math/reflection.hpp#L72).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 41 lines</summary>

```text

 math::Reflection — the field plane reflected across a line through the origin, or left as it is (chunk
 MIRROR1-A, 2026-11-13; the MIRROR1 brief's §1.2 and D1/D7). PURE: it includes only math/ and units/
 (SHULIB_PRECONDITION reaches it through math/angle.hpp, which carries core/check.hpp for every math
 header). Not in the locked math/frame.hpp, whose banner allows "new vocabulary only".

 ── Why only lines at a multiple of 45 degrees ───────────────────────────────────────────────────
 A reflection across the line through the origin at heading a maps a point by the matrix
     [ cos 2a   sin 2a ]
     [ sin 2a  -cos 2a ]
 and a heading h to 2a - h. At a multiple of 45 degrees, 2a is a multiple of 90 degrees, so every
 coefficient is EXACTLY 0 or +-1: a position is reflected by swapping and negating coordinates — no
 rounding at all — and a reflected reflection gives the position back EXACTLY. Those are the only
 four lines a VEX field's symmetry uses (the two midlines and the two diagonals); any other line is a
 precondition, refused by acrossLineAt(). The four, in the field frame (F1: +x right as seen from the
 red station, +y away from it, heading 0 along +x, counterclockwise positive):
     XAxis         y = 0      (x, y, h) -> ( x, -y,       -h)   — also the ROBOT-frame mirror (D7)
     Diagonal      y = x      (x, y, h) -> ( y,  x,  90 deg - h)
     YAxis         x = 0      (x, y, h) -> (-x,  y, 180 deg - h)
     AntiDiagonal  y = -x     (x, y, h) -> (-y, -x, -90 deg - h)  — Override's SKILLS mirror (D1)

 ── What it reflects ─────────────────────────────────────────────────────────────────────────────
 A point, a Pose2d (position and heading), an Angle (2a - h, re-wrapped into F3's (-pi, pi]), and a
 ChassisSpeeds in either math::Frame: a FIELD velocity's (vx, vy) as a point and its turn rate negated;
 a BODY velocity as (vx, -vy, -omega) whatever the line — the mirror image of a robot drives the same
 speed forward, the same speed to its other side, and turns the other way.

 ── Two rules that keep its output safe to print and to compare ─────────────────────────────────
  * NEVER -0.0. Reflecting negates, and -(0.0) prints "-0.0" — robot one's x = 0 must not reach robot
    two's screen as "y = -0.0". Every component a non-identity reflection produces is normalised with
    `+ 0.0` (in round-to-nearest, -0.0 + 0.0 is +0.0 and every other value is unchanged): the idiom
    competition/toggle_visit.hpp's towardWallRadians already uses. A reflected reflection therefore
    returns positions EQUAL to the original (an input -0.0 comes back +0.0, which compares equal), not
    bit-identical.
  * THE IDENTITY IS NOT APPLIED. apply() on the identity hands back its input untouched — and a caller
    that must stay bit-identical when nothing is mirrored (the Chassis's verbs) branches on
    isIdentity() and never calls apply() at all: re-wrapping a heading through Angle::radians can move
    its last bit (the MIRROR1 brief's landmine 6).
 Headings follow F3 untouched: the result is wrapped into (-pi, pi], an exact -pi stored as +pi.

 PROS-free, allocation-free, header-only. Nothing here has run on a robot.
```

</details>
