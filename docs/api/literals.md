<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/units/literals.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `literals.hpp`

User-defined literals for shulib units. Each converts to CANONICAL units at the point of writing, so the rest of the code never sees a raw unit again: distance = 24_in; field = 1_tile; dt = 20_ms; v = 12_volt;.

This header declares **14** free functions.

Extracted from [`include/shulib/units/literals.hpp`](../../include/shulib/units/literals.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`operator""_in`](#operator-quote-quote-_in) — *free function*
- [`operator""_in (overload 2)`](#operator-quote-quote-_in-2) — *free function*
- [`operator""_tile`](#operator-quote-quote-_tile) — *free function*
- [`operator""_tile (overload 2)`](#operator-quote-quote-_tile-2) — *free function*
- [`operator""_s`](#operator-quote-quote-_s) — *free function*
- [`operator""_s (overload 2)`](#operator-quote-quote-_s-2) — *free function*
- [`operator""_ms`](#operator-quote-quote-_ms) — *free function*
- [`operator""_ms (overload 2)`](#operator-quote-quote-_ms-2) — *free function*
- [`operator""_volt`](#operator-quote-quote-_volt) — *free function*
- [`operator""_volt (overload 2)`](#operator-quote-quote-_volt-2) — *free function*
- [`operator""_deg`](#operator-quote-quote-_deg) — *free function*
- [`operator""_deg (overload 2)`](#operator-quote-quote-_deg-2) — *free function*
- [`operator""_rad`](#operator-quote-quote-_rad) — *free function*
- [`operator""_rad (overload 2)`](#operator-quote-quote-_rad-2) — *free function*

<a id="operator-quote-quote-_in"></a>

## `operator""_in`

```cpp
constexpr Length operator""_in(long double v) noexcept
```

Inches — THE canonical length unit, so `24_in` stores exactly 24.0 and nothing converts.

*free function, declared at [`include/shulib/units/literals.hpp:22`](../../include/shulib/units/literals.hpp#L22).*

<a id="operator-quote-quote-_in-2"></a>

## `operator""_in (overload 2)`

```cpp
constexpr Length operator""_in(unsigned long long v) noexcept
```

Integer spelling of `_in`, and NOT redundant with the overload above: C++ offers an integer literal only to the `unsigned long long` form, so without this one `24_in` would not compile — it is never quietly converted to the `long double` operator. That the parameter is unsigned is also why there is no negative literal: `-24_in` is unary minus applied to `Length{24.0}`, which works only because Length is a signed displacement.

*free function, declared at [`include/shulib/units/literals.hpp:28`](../../include/shulib/units/literals.hpp#L28).*

<a id="operator-quote-quote-_tile"></a>

## `operator""_tile`

```cpp
constexpr Length operator""_tile(long double v) noexcept
```

Foam field tiles, converted HERE: one tile is 24 inches, so `1_tile` is Length{24.0}. The result is an ordinary Length — nothing downstream ever knows a tile was mentioned.

*free function, declared at [`include/shulib/units/literals.hpp:31`](../../include/shulib/units/literals.hpp#L31).*

<a id="operator-quote-quote-_tile-2"></a>

## `operator""_tile (overload 2)`

```cpp
constexpr Length operator""_tile(unsigned long long v) noexcept
```

Integer spelling of `_tile`; `2_tile` is 48 inches.

*free function, declared at [`include/shulib/units/literals.hpp:33`](../../include/shulib/units/literals.hpp#L33).*

<a id="operator-quote-quote-_s"></a>

## `operator""_s`

```cpp
constexpr Time operator""_s(long double v) noexcept
```

Seconds — the canonical time unit, so `2_s` stores exactly 2.0.

*free function, declared at [`include/shulib/units/literals.hpp:37`](../../include/shulib/units/literals.hpp#L37).*

<a id="operator-quote-quote-_s-2"></a>

## `operator""_s (overload 2)`

```cpp
constexpr Time operator""_s(unsigned long long v) noexcept
```

Integer spelling of `_s`, which C++ forces to be a SEPARATE overload — a literal written with no decimal point is offered only to the `unsigned long long` form and never converted to the `long double` one above. Nothing is computed on the way in, so `2_s` is exactly 2.0, and the integer-to-double conversion stays exact far past any duration a match can run.

*free function, declared at [`include/shulib/units/literals.hpp:42`](../../include/shulib/units/literals.hpp#L42).*

<a id="operator-quote-quote-_ms"></a>

## `operator""_ms`

```cpp
constexpr Time operator""_ms(long double v) noexcept
```

Milliseconds, divided by 1000 AT THE LITERAL: `20_ms` is Time{0.02}. This is where the "internal seconds, always" rule is enforced — no millisecond value reaches a gain or a dt.

*free function, declared at [`include/shulib/units/literals.hpp:45`](../../include/shulib/units/literals.hpp#L45).*

<a id="operator-quote-quote-_ms-2"></a>

## `operator""_ms (overload 2)`

```cpp
constexpr Time operator""_ms(unsigned long long v) noexcept
```

Integer spelling of `_ms` — the form a loop period is normally written in (`10_ms`).

*free function, declared at [`include/shulib/units/literals.hpp:47`](../../include/shulib/units/literals.hpp#L47).*

<a id="operator-quote-quote-_volt"></a>

## `operator""_volt`

```cpp
constexpr Voltage operator""_volt(long double v) noexcept
```

Volts — canonical, so `12_volt` stores 12.0. It does NOT clamp: the ±12 V ceiling is applied by IMotor::setVoltage (hal::kMaxMotorVoltage), never by writing the literal.

*free function, declared at [`include/shulib/units/literals.hpp:52`](../../include/shulib/units/literals.hpp#L52).*

<a id="operator-quote-quote-_volt-2"></a>

## `operator""_volt (overload 2)`

```cpp
constexpr Voltage operator""_volt(unsigned long long v) noexcept
```

Integer spelling of `_volt` — the form the rail voltage is normally written in — and required as its own overload, since an integer literal is never offered to the `long double` operator above. It does NOT clamp either: `13_volt` builds Voltage{13.0} and only IMotor::setVoltage applies the ±kMaxMotorVoltage ceiling. Reverse is `-12_volt`, which is unary minus on Voltage{12.0} rather than a negative literal — the parameter here is unsigned.

*free function, declared at [`include/shulib/units/literals.hpp:58`](../../include/shulib/units/literals.hpp#L58).*

<a id="operator-quote-quote-_deg"></a>

## `operator""_deg`

```cpp
inline ::shulib::math::Angle operator""_deg(long double v)
```

Degrees as a WRAPPING math::Angle rather than a bare Quantity, so `90_deg` is a heading and not a scalar: it normalizes to (-π, π] on construction, which means `359_deg` stores -1°. Degrees exist only at this boundary — the value inside is radians.

*free function, declared at [`include/shulib/units/literals.hpp:64`](../../include/shulib/units/literals.hpp#L64).*

<a id="operator-quote-quote-_deg-2"></a>

## `operator""_deg (overload 2)`

```cpp
inline ::shulib::math::Angle operator""_deg(unsigned long long v)
```

Integer spelling of `_deg`. The angle literals are the only ones here that are NOT constexpr — Angle's factories run a runtime finiteness check — so `90_deg` cannot initialize a `constexpr` variable the way `24_in` can.

*free function, declared at [`include/shulib/units/literals.hpp:70`](../../include/shulib/units/literals.hpp#L70).*

<a id="operator-quote-quote-_rad"></a>

## `operator""_rad`

```cpp
inline ::shulib::math::Angle operator""_rad(long double v)
```

Radians as a wrapping math::Angle, with no conversion (`3_rad` stores 3.0) but the SAME normalization as `_deg`, so `7_rad` stores 7 − 2π ≈ 0.717. There is no literal here that builds an un-wrapped heading; use units::AngleDim for a rate or an accumulated total.

*free function, declared at [`include/shulib/units/literals.hpp:76`](../../include/shulib/units/literals.hpp#L76).*

<a id="operator-quote-quote-_rad-2"></a>

## `operator""_rad (overload 2)`

```cpp
inline ::shulib::math::Angle operator""_rad(unsigned long long v)
```

Integer spelling of `_rad`. Same type as `_deg` produces, so the two are interchangeable wherever an Angle is taken; also not constexpr (see `_deg`).

*free function, declared at [`include/shulib/units/literals.hpp:81`](../../include/shulib/units/literals.hpp#L81).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 12 lines</summary>

```text

 User-defined literals for shulib units. Each converts to CANONICAL units at
 the point of writing, so the rest of the code never sees a raw unit again:
   distance = 24_in;  field = 1_tile;  dt = 20_ms;  v = 12_volt;

 `_ms` divides by 1000 -> seconds, enforcing the "internal seconds, always"
 decision at the literal boundary. `_deg`/`_rad` build an (auto-wrapped)
 shulib::math::Angle, NOT a bare Quantity (decision §13 #3).

 Each literal has two forms: a floating (long double) and an integer
 (unsigned long long), so both `24_in` and `24.0_in` compile. The narrowing
 to double is made explicit (the build is -Wconversion -Werror).
```

</details>
