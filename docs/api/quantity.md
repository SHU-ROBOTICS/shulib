<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/units/quantity.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `quantity.hpp`

Quantity<L, A, T, E, I> — compile-time dimensional analysis.

This header declares **1** type (13 members), **2** free functions, and **10** type aliass.

Extracted from [`include/shulib/units/quantity.hpp`](../../include/shulib/units/quantity.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class Quantity`](#class-quantity)
  - [`Quantity`](#quantity-quantity)
  - [`Quantity (overload 2)`](#quantity-quantity-2)
  - [`value`](#quantity-value)
  - [`operator+=`](#quantity-operator-plus-eq)
  - [`operator-=`](#quantity-operator-minus-eq)
  - [`operator+`](#quantity-operator-plus)
  - [`operator-`](#quantity-operator-minus)
  - [`operator- (overload 2)`](#quantity-operator-minus-2)
  - [`operator*`](#quantity-operator-star)
  - [`operator* (overload 2)`](#quantity-operator-star-2)
  - [`operator/`](#quantity-operator-slash)
  - [`operator==`](#quantity-operator-eq-eq)
  - [`operator<=>`](#quantity-operator-lt-eq-gt)
- [`operator*`](#operator-star) — *free function*
- [`operator/`](#operator-slash) — *free function*
- [`Number`](#number) — *type alias*
- [`Length`](#length) — *type alias*
- [`AngleDim`](#angledim) — *type alias*
- [`Time`](#time) — *type alias*
- [`Voltage`](#voltage) — *type alias*
- [`Current`](#current) — *type alias*
- [`Velocity`](#velocity) — *type alias*
- [`Acceleration`](#acceleration) — *type alias*
- [`AngularVelocity`](#angularvelocity) — *type alias*
- [`Power`](#power) — *type alias*

<a id="class-quantity"></a>

## `class Quantity`

```cpp
template <int L, int A, int T, int E, int I> class Quantity
```

A double that carries its DIMENSION in its type: the five integer exponents are length, angle, time, voltage, current. The stored value is ALWAYS canonical — inches, radians, seconds, volts, amperes, and combinations of them — so there is no unit tag to read and no conversion left for a caller to forget. Same-dimension arithmetic behaves normally; mixing dimensions under `+`/`-` names no overload and fails to compile, while `*` and `/` derive the result's dimension. One double wide and constexpr throughout, so the checking costs nothing at run time. A HEADING is not this type: use math::Angle, which owns wrapping (see the banner above).

*class, declared at [`include/shulib/units/quantity.hpp:38`](../../include/shulib/units/quantity.hpp#L38).*

<a id="quantity-quantity"></a>

### `Quantity::Quantity`

```cpp
constexpr Quantity() = default
```

The dimensioned zero. Never indeterminate — the stored value is 0.0 whether or not the declaration writes braces.

*function, declared at [`include/shulib/units/quantity.hpp:42`](../../include/shulib/units/quantity.hpp#L42).*

<a id="quantity-quantity-2"></a>

### `Quantity::Quantity (overload 2)`

```cpp
constexpr explicit Quantity(double canonicalValue) noexcept
```

Construct from a value ALREADY in canonical units. Explicit, so a bare double never silently becomes a dimensioned quantity — and unchecked, so a NaN, or a number that is really in degrees or milliseconds, is stored as though it were canonical. Prefer the units::literals (`24_in`, `20_ms`), which do the conversion once, where the number is written.

*function, declared at [`include/shulib/units/quantity.hpp:49`](../../include/shulib/units/quantity.hpp#L49).*

<a id="quantity-value"></a>

### `Quantity::value`

```cpp
[[nodiscard]] constexpr double value() const noexcept
```

The value in canonical units (inch / radian / second / volt / ampere, or a derived combination). The one way out of the type — everything past this point is a bare double again, so unwrap as late as possible.

*function, declared at [`include/shulib/units/quantity.hpp:54`](../../include/shulib/units/quantity.hpp#L54).*

<a id="quantity-operator-plus-eq"></a>

### `Quantity::operator+=`

```cpp
constexpr Quantity& operator+=(Quantity o) noexcept
```

Add in place. A different dimension is a different TYPE, so there is no conversion left to get wrong: `len += dt` names no overload at all.

*function, declared at [`include/shulib/units/quantity.hpp:59`](../../include/shulib/units/quantity.hpp#L59).*

<a id="quantity-operator-minus-eq"></a>

### `Quantity::operator-=`

```cpp
constexpr Quantity& operator-=(Quantity o) noexcept
```

Subtract in place. Signed — the value goes negative rather than saturating at zero.

*function, declared at [`include/shulib/units/quantity.hpp:61`](../../include/shulib/units/quantity.hpp#L61).*

<a id="quantity-operator-plus"></a>

### `Quantity::operator+`

```cpp
[[nodiscard]] friend constexpr Quantity operator+(Quantity a, Quantity b) noexcept
```

Sum, dimension unchanged — `+` is the one operator here that CANNOT derive a new type: the exponents `*` and `/` add and subtract simply carry through. A bare number is not an operand: the constructor is explicit and there is no `Quantity + double` overload (unlike `*` and `/`, which do take a dimensionless scalar), so `len + 2.0` names no overload at all — write `len + 2_in`.

*function, declared at [`include/shulib/units/quantity.hpp:68`](../../include/shulib/units/quantity.hpp#L68).*

<a id="quantity-operator-minus"></a>

### `Quantity::operator-`

```cpp
[[nodiscard]] friend constexpr Quantity operator-(Quantity a, Quantity b) noexcept
```

Difference `a - b`, signed and unclamped: `b` larger than `a` yields a negative quantity of the same dimension.

*function, declared at [`include/shulib/units/quantity.hpp:73`](../../include/shulib/units/quantity.hpp#L73).*

<a id="quantity-operator-minus-2"></a>

### `Quantity::operator- (overload 2)`

```cpp
[[nodiscard]] friend constexpr Quantity operator-(Quantity a) noexcept
```

Negation — reverses the sense of a signed quantity (a displacement, a velocity, a voltage). shulib declares no `abs` for Quantity; take `std::abs(q.value())` when a magnitude is what you want.

*function, declared at [`include/shulib/units/quantity.hpp:79`](../../include/shulib/units/quantity.hpp#L79).*

<a id="quantity-operator-star"></a>

### `Quantity::operator*`

```cpp
[[nodiscard]] friend constexpr Quantity operator*(Quantity a, double s) noexcept
```

Scale by a DIMENSIONLESS factor; the dimension is unchanged. When the factor itself carries units, use the Quantity-by-Quantity operator below — that one changes it.

*function, declared at [`include/shulib/units/quantity.hpp:84`](../../include/shulib/units/quantity.hpp#L84).*

<a id="quantity-operator-star-2"></a>

### `Quantity::operator* (overload 2)`

```cpp
[[nodiscard]] friend constexpr Quantity operator*(double s, Quantity a) noexcept
```

The same scaling with the scalar written on the left, so `0.5 * dt` and `dt * 0.5` are both spellable and identical.

*function, declared at [`include/shulib/units/quantity.hpp:87`](../../include/shulib/units/quantity.hpp#L87).*

<a id="quantity-operator-slash"></a>

### `Quantity::operator/`

```cpp
[[nodiscard]] friend constexpr Quantity operator/(Quantity a, double s) noexcept
```

Divide by a dimensionless factor; dimension unchanged. `s == 0` yields infinity by IEEE rules, not an error. There is deliberately no `double / Quantity` overload, and the asymmetry with the DOUBLED multiply above is the point rather than an oversight: this block is scalar SCALING, which leaves the dimension alone, while a scalar divided by a quantity always INVERTS it — a different operation, belonging with the Quantity-by-Quantity operators. Write `Number{1.0} / dt`, which produces the inverse dimension through exactly those.

*function, declared at [`include/shulib/units/quantity.hpp:95`](../../include/shulib/units/quantity.hpp#L95).*

<a id="quantity-operator-eq-eq"></a>

### `Quantity::operator==`

```cpp
[[nodiscard]] friend constexpr bool operator==(Quantity a, Quantity b) noexcept
```

EXACT equality of the canonical doubles — there is no tolerance anywhere in here. Two quantities computed by different routes will rarely compare equal; test `std::abs((a - b).value())` against a tolerance you chose instead.

*function, declared at [`include/shulib/units/quantity.hpp:101`](../../include/shulib/units/quantity.hpp#L101).*

<a id="quantity-operator-lt-eq-gt"></a>

### `Quantity::operator<=>`

```cpp
[[nodiscard]] friend constexpr std::partial_ordering operator<=>(Quantity a, Quantity b) noexcept
```

Orders by canonical value. `partial_ordering`, not `strong_ordering`, because doubles are: if either side is NaN the result is `unordered`, and `<`, `>`, `<=` and `>=` are then ALL false.

*function, declared at [`include/shulib/units/quantity.hpp:105`](../../include/shulib/units/quantity.hpp#L105).*

<a id="operator-star"></a>

## `operator*`

```cpp
template <int L1, int A1, int T1, int E1, int I1, int L2, int A2, int T2, int E2, int I2> [[nodiscard]] constexpr auto operator*(Quantity<L1, A1, T1, E1, I1> a, Quantity<L2, A2, T2, E2, I2> b) noexcept -> Quantity<L1 + L2, A1 + A2, T1 + T2, E1 + E2, I1 + I2>
```

Multiply two dimensioned quantities: the five exponents ADD, so the result TYPE is derived rather than declared anywhere. Velocity * Time -> Length; Voltage * Current -> Power; Number * anything -> that same dimension.

*free function, declared at [`include/shulib/units/quantity.hpp:118`](../../include/shulib/units/quantity.hpp#L118).*

<a id="operator-slash"></a>

## `operator/`

```cpp
template <int L1, int A1, int T1, int E1, int I1, int L2, int A2, int T2, int E2, int I2> [[nodiscard]] constexpr auto operator/(Quantity<L1, A1, T1, E1, I1> a, Quantity<L2, A2, T2, E2, I2> b) noexcept -> Quantity<L1 - L2, A1 - A2, T1 - T2, E1 - E2, I1 - I2>
```

Divide two dimensioned quantities: the five exponents SUBTRACT. Length / Time -> Velocity, Velocity / Time -> Acceleration, Length / Length -> Number. Dividing by a zero quantity yields infinity by IEEE rules, not an error.

*free function, declared at [`include/shulib/units/quantity.hpp:127`](../../include/shulib/units/quantity.hpp#L127).*

<a id="number"></a>

## `Number`

```cpp
using Number = Quantity<0, 0, 0, 0, 0>
```

dimensionless — a ratio, a gain, Length/Length

*type alias, declared at [`include/shulib/units/quantity.hpp:133`](../../include/shulib/units/quantity.hpp#L133).*

<a id="length"></a>

## `Length`

```cpp
using Length = Quantity<1, 0, 0, 0, 0>
```

inches; signed — a displacement, not a size

*type alias, declared at [`include/shulib/units/quantity.hpp:134`](../../include/shulib/units/quantity.hpp#L134).*

<a id="angledim"></a>

## `AngleDim`

```cpp
using AngleDim = Quantity<0, 1, 0, 0, 0>
```

radians, for RATES; a heading is math::Angle

*type alias, declared at [`include/shulib/units/quantity.hpp:135`](../../include/shulib/units/quantity.hpp#L135).*

<a id="time"></a>

## `Time`

```cpp
using Time = Quantity<0, 0, 1, 0, 0>
```

seconds; `20_ms` converts to 0.02 at the literal

*type alias, declared at [`include/shulib/units/quantity.hpp:136`](../../include/shulib/units/quantity.hpp#L136).*

<a id="voltage"></a>

## `Voltage`

```cpp
using Voltage = Quantity<0, 0, 0, 1, 0>
```

volts — what IMotor::setVoltage commands

*type alias, declared at [`include/shulib/units/quantity.hpp:137`](../../include/shulib/units/quantity.hpp#L137).*

<a id="current"></a>

## `Current`

```cpp
using Current = Quantity<0, 0, 0, 0, 1>
```

amperes — the 5th dim, for IMotor/IBattery

*type alias, declared at [`include/shulib/units/quantity.hpp:138`](../../include/shulib/units/quantity.hpp#L138).*

<a id="velocity"></a>

## `Velocity`

```cpp
using Velocity = Quantity<1, 0, -1, 0, 0>
```

in/s — Length/Time, derived, never declared

*type alias, declared at [`include/shulib/units/quantity.hpp:139`](../../include/shulib/units/quantity.hpp#L139).*

<a id="acceleration"></a>

## `Acceleration`

```cpp
using Acceleration = Quantity<1, 0, -2, 0, 0>
```

in/s^2 — Velocity/Time, i.e. Length/Time^2

*type alias, declared at [`include/shulib/units/quantity.hpp:140`](../../include/shulib/units/quantity.hpp#L140).*

<a id="angularvelocity"></a>

## `AngularVelocity`

```cpp
using AngularVelocity = Quantity<0, 1, -1, 0, 0>
```

rad/s — a yaw RATE; ChassisSpeeds' third term

*type alias, declared at [`include/shulib/units/quantity.hpp:141`](../../include/shulib/units/quantity.hpp#L141).*

<a id="power"></a>

## `Power`

```cpp
using Power = Quantity<0, 0, 0, 1, 1>
```

watts (V·A) — proof that Current composes

*type alias, declared at [`include/shulib/units/quantity.hpp:142`](../../include/shulib/units/quantity.hpp#L142).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 22 lines</summary>

```text

 Quantity<L, A, T, E, I> — compile-time dimensional analysis.

 The integer exponents of the five base dimensions live in the TYPE:
     L = length, A = angle, T = time, E = electric potential (voltage),
     I = electric current (amperes).
 (I was added 2026-06-19, additively, for IMotor/IBattery current() — F4 review.
  The original four dims and their semantics are unchanged; F3 stays intact.)
 Consequences, all enforced by the compiler:
   * adding/subtracting different dimensions FAILS TO COMPILE,
   * multiply/divide compute the resulting dimension automatically
     (Length / Time -> Velocity, Velocity / Time -> Acceleration, ...),
   * the stored value is ALWAYS canonical: inch, radian, second, volt, AMPERE. The ampere
     was missing from this list while the same header declared the current dimension and
     the Current alias, and hal/pros/battery.hpp stores units::Current{ma / 1000.0} — a
     canonical ampere, not a derived combination of the other four.

 This kills two bug classes at compile time: "degrees into cos/sin" and
 "milliseconds into a seconds-based gain". (master plan §7, §13 #3; Freeze F3.)

 Headings use the dedicated wrapping type shulib::math::Angle; the angle
 dimension here exists for RATES (e.g. angular velocity rad/s) and bookkeeping.
```

</details>
