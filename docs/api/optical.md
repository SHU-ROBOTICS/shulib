<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/optical.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `optical.hpp`

IOptical — a color / optical sensor (pros::Optical) behind the HAL.

This header declares **1** type (10 members).

Extracted from [`include/shulib/hal/optical.hpp`](../../include/shulib/hal/optical.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class IOptical`](#class-ioptical)
  - [`~IOptical`](#ioptical-destructor-ioptical)
  - [`IOptical`](#ioptical-ioptical)
  - [`IOptical (overload 2)`](#ioptical-ioptical-2)
  - [`IOptical (overload 3)`](#ioptical-ioptical-3)
  - [`operator=`](#ioptical-operator-eq)
  - [`operator= (overload 2)`](#ioptical-operator-eq-2)
  - [`hue`](#ioptical-hue)
  - [`saturation`](#ioptical-saturation)
  - [`brightness`](#ioptical-brightness)
  - [`proximity`](#ioptical-proximity)

<a id="class-ioptical"></a>

## `class IOptical`

```cpp
class IOptical
```

A color / optical sensor behind the HAL — the seam over pros::Optical that game-object color confirmation reads (the M4 scoring-half orient, the Toggle color confirm, intake capture). Every reader's SCALE is fixed by this seam, not by the device: hue in degrees [0, 360), the other three in [0, 1]. An implementation owns whatever vendor rescaling that takes, so no caller ever divides by 255 or wonders which scale it got. Read-only — the vendor's LED, gesture and integration-time controls are not part of this seam.

*class, declared at [`include/shulib/hal/optical.hpp:22`](../../include/shulib/hal/optical.hpp#L22).*

<a id="ioptical-destructor-ioptical"></a>

### `IOptical::~IOptical`

```cpp
virtual ~IOptical() = default
```

The polymorphic-base special members, and the exact rule they follow. The virtual destructor is what makes deleting a derived sensor through an `IOptical*` legal, and user-declaring it costs the two MOVE operations: the move constructor and move assignment are then never implicitly declared, while the COPY pair still is (merely deprecated). So only the two `&&` lines below restore anything. Declaring constructors is in turn what suppresses the implicit default constructor, which is why `IOptical() = default;` has to be spelled out — without it no derived sensor could be default-constructed. None of this is an invitation to copy a live device: implementations are held by pointer, and copying one would duplicate a port, not a reading.

*function, declared at [`include/shulib/hal/optical.hpp:33`](../../include/shulib/hal/optical.hpp#L33).*

<a id="ioptical-ioptical"></a>

### `IOptical::IOptical`

```cpp
IOptical() = default
```

*Covered by the comment on [`~IOptical`](#ioptical-destructor-ioptical) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/optical.hpp:34`](../../include/shulib/hal/optical.hpp#L34).*

<a id="ioptical-ioptical-2"></a>

### `IOptical::IOptical (overload 2)`

```cpp
IOptical(const IOptical&) = default
```

*Covered by the comment on [`~IOptical`](#ioptical-destructor-ioptical) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/optical.hpp:35`](../../include/shulib/hal/optical.hpp#L35).*

<a id="ioptical-ioptical-3"></a>

### `IOptical::IOptical (overload 3)`

```cpp
IOptical(IOptical&&) = default
```

*Covered by the comment on [`~IOptical`](#ioptical-destructor-ioptical) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/optical.hpp:36`](../../include/shulib/hal/optical.hpp#L36).*

<a id="ioptical-operator-eq"></a>

### `IOptical::operator=`

```cpp
IOptical& operator=(const IOptical&) = default
```

*Covered by the comment on [`~IOptical`](#ioptical-destructor-ioptical) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/optical.hpp:37`](../../include/shulib/hal/optical.hpp#L37).*

<a id="ioptical-operator-eq-2"></a>

### `IOptical::operator= (overload 2)`

```cpp
IOptical& operator=(IOptical&&) = default
```

*Covered by the comment on [`~IOptical`](#ioptical-destructor-ioptical) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/optical.hpp:38`](../../include/shulib/hal/optical.hpp#L38).*

<a id="ioptical-hue"></a>

### `IOptical::hue`

```cpp
[[nodiscard]] virtual double hue() const = 0
```

Detected color hue in degrees, [0, 360). A color, not a heading.

*function, declared at [`include/shulib/hal/optical.hpp:41`](../../include/shulib/hal/optical.hpp#L41).*

<a id="ioptical-saturation"></a>

### `IOptical::saturation`

```cpp
[[nodiscard]] virtual double saturation() const = 0
```

Color saturation in [0, 1].

*function, declared at [`include/shulib/hal/optical.hpp:44`](../../include/shulib/hal/optical.hpp#L44).*

<a id="ioptical-brightness"></a>

### `IOptical::brightness`

```cpp
[[nodiscard]] virtual double brightness() const = 0
```

Brightness in [0, 1].

*function, declared at [`include/shulib/hal/optical.hpp:47`](../../include/shulib/hal/optical.hpp#L47).*

<a id="ioptical-proximity"></a>

### `IOptical::proximity`

```cpp
[[nodiscard]] virtual double proximity() const = 0
```

Proximity in [0, 1] (≈1 = object close to the sensor).

*function, declared at [`include/shulib/hal/optical.hpp:50`](../../include/shulib/hal/optical.hpp#L50).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 9 lines</summary>

```text

 IOptical — a color / optical sensor (pros::Optical) behind the HAL. Used for
 game-object color confirmation (master plan §M4: orientToScoringHalf, the Toggle
 color confirm, intake capture).

 hue() is a COLOR hue in degrees [0, 360) — cyclic, but NOT a heading, so it is a
 plain double, never a math::Angle (mixing a color with a spatial angle would be a
 bug). saturation()/brightness()/proximity() are normalized to [0, 1] by the
 hal/pros adapter (proximity ≈ 1 means an object is close).
```

</details>
