<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/distance.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `distance.hpp`

IDistance — a distance / time-of-flight sensor (pros::Distance) behind the HAL.

This header declares **1** type (8 members).

Extracted from [`include/shulib/hal/distance.hpp`](../../include/shulib/hal/distance.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class IDistance`](#class-idistance)
  - [`~IDistance`](#idistance-destructor-idistance)
  - [`IDistance`](#idistance-idistance)
  - [`IDistance (overload 2)`](#idistance-idistance-2)
  - [`IDistance (overload 3)`](#idistance-idistance-3)
  - [`operator=`](#idistance-operator-eq)
  - [`operator= (overload 2)`](#idistance-operator-eq-2)
  - [`distance`](#idistance-distance)
  - [`confidence`](#idistance-confidence)

<a id="class-idistance"></a>

## `class IDistance`

```cpp
class IDistance
```

A time-of-flight rangefinder behind the HAL, canonical at the seam: inches out of distance(), a [0, 1] confidence out of confidence(). Read confidence() FIRST — "nothing in range" arrives there as ~0 while distance() stays finite and reports a plausible- looking far wall, so a caller that trusts distance() alone acts on empty air. Consumers are the manipulation sensor-confirm and the docking distance-fallback.

*class, declared at [`include/shulib/hal/distance.hpp:20`](../../include/shulib/hal/distance.hpp#L20).*

<a id="idistance-destructor-idistance"></a>

### `IDistance::~IDistance`

```cpp
virtual ~IDistance() = default
```

Public defaulted special members on a polymorphic base: the virtual destructor makes `delete` through an `IDistance*` well-defined, and copy/move stay available so an adapter deriving from this is free to be value-like. This base carries no state of its own — hold implementations by reference or pointer, never by value.

*function, declared at [`include/shulib/hal/distance.hpp:26`](../../include/shulib/hal/distance.hpp#L26).*

<a id="idistance-idistance"></a>

### `IDistance::IDistance`

```cpp
IDistance() = default
```

*Covered by the comment on [`~IDistance`](#idistance-destructor-idistance) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/distance.hpp:27`](../../include/shulib/hal/distance.hpp#L27).*

<a id="idistance-idistance-2"></a>

### `IDistance::IDistance (overload 2)`

```cpp
IDistance(const IDistance&) = default
```

*Covered by the comment on [`~IDistance`](#idistance-destructor-idistance) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/distance.hpp:28`](../../include/shulib/hal/distance.hpp#L28).*

<a id="idistance-idistance-3"></a>

### `IDistance::IDistance (overload 3)`

```cpp
IDistance(IDistance&&) = default
```

*Covered by the comment on [`~IDistance`](#idistance-destructor-idistance) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/distance.hpp:29`](../../include/shulib/hal/distance.hpp#L29).*

<a id="idistance-operator-eq"></a>

### `IDistance::operator=`

```cpp
IDistance& operator=(const IDistance&) = default
```

*Covered by the comment on [`~IDistance`](#idistance-destructor-idistance) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/distance.hpp:30`](../../include/shulib/hal/distance.hpp#L30).*

<a id="idistance-operator-eq-2"></a>

### `IDistance::operator= (overload 2)`

```cpp
IDistance& operator=(IDistance&&) = default
```

*Covered by the comment on [`~IDistance`](#idistance-destructor-idistance) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/distance.hpp:31`](../../include/shulib/hal/distance.hpp#L31).*

<a id="idistance-distance"></a>

### `IDistance::distance`

```cpp
[[nodiscard]] virtual units::Length distance() const = 0
```

Measured distance to the nearest object (canonical inches). Only meaningful when confidence() is above the caller's threshold.

*function, declared at [`include/shulib/hal/distance.hpp:35`](../../include/shulib/hal/distance.hpp#L35).*

<a id="idistance-confidence"></a>

### `IDistance::confidence`

```cpp
[[nodiscard]] virtual double confidence() const = 0
```

Reading confidence in [0, 1]; ~0 means no usable return (treat as "no object").

*function, declared at [`include/shulib/hal/distance.hpp:38`](../../include/shulib/hal/distance.hpp#L38).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 8 lines</summary>

```text

 IDistance — a distance / time-of-flight sensor (pros::Distance) behind the HAL.
 distance() in canonical inches; confidence() in [0, 1]. The hal/pros adapter
 converts mm→inches and normalizes the raw confidence (0–63 → 0–1) exactly once.

 Callers threshold confidence() for "object present" — never trust distance() when
 confidence() is ~0 (out of range / no return). Used for manipulation sensor-confirm
 and the docking distance-fallback (master plan §M3/§M4).
```

</details>
