<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/localization/axis.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `axis.hpp`

Axis — the FIELD axis a wall measures.

This header declares **1** type (2 members) and **2** free functions.

Extracted from [`include/shulib/localization/axis.hpp`](../../include/shulib/localization/axis.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in [How it works](../guide/README.md); worked recipes live in [Recipes](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`enum class Axis`](#enum-class-axis)
  - [`X`](#axis-x)
  - [`Y`](#axis-y)
- [`axisLetter`](#axisletter) — *free function*
- [`alongAxis`](#alongaxis) — *free function*

<a id="enum-class-axis"></a>

## `enum class Axis`

```cpp
enum class Axis : std::uint8_t
```

A field axis (the F1 frame: +X to the right as seen from the red alliance station, +Y away from it). A field wall is a line `axis = constant`, so a range measured square to it says where the robot is along THAT axis and nothing about the other one — which is why the wall reset re-seats one axis at a time (`Localizer::seatAxis`).

*enum class, declared at [`include/shulib/localization/axis.hpp:24`](../../include/shulib/localization/axis.hpp#L24).*

<a id="axis-x"></a>

### `Axis::X`

```cpp
X
```

The x axis — what the LEFT and RIGHT walls (x = -70.2 / +70.2 in) measure.

*enumerator, declared at [`include/shulib/localization/axis.hpp:26`](../../include/shulib/localization/axis.hpp#L26).*

<a id="axis-y"></a>

### `Axis::Y`

```cpp
Y
```

The y axis — what the RED and BLUE walls (y = -70.2 / +70.2 in) measure.

*enumerator, declared at [`include/shulib/localization/axis.hpp:28`](../../include/shulib/localization/axis.hpp#L28).*

<a id="axisletter"></a>

## `axisLetter`

```cpp
[[nodiscard]] constexpr const char* axisLetter(Axis axis) noexcept
```

The axis's letter for a log line: "x" or "y". Exhaustive, no default — a new axis without a letter does not compile under the project's -Wswitch -Werror.

*free function, declared at [`include/shulib/localization/axis.hpp:33`](../../include/shulib/localization/axis.hpp#L33).*

<a id="alongaxis"></a>

## `alongAxis`

```cpp
[[nodiscard]] constexpr Axis alongAxis(Axis axis) noexcept
```

The OTHER axis: the one a wall on `axis` lies ALONG (the left wall measures x and runs along y). The wall reset's keep-outs are tested along it, which is why it widens them by how far the estimate may have drifted on it (`Localizer::travelSinceSeat`).

*free function, declared at [`include/shulib/localization/axis.hpp:46`](../../include/shulib/localization/axis.hpp#L46).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 13 lines</summary>

```text

 Axis — the FIELD axis a wall measures (chunk L6-A1, the wall reset, 2026-10-29).

 A LEAF header on purpose. Two headers need this one enum: the Localizer (its one-axis seat,
 `Localizer::seatAxis`, and its per-axis travel counter) and the wall reset's pure gates
 (`localization/wall_reset.hpp`). The wall reset's EXPLICIT call needs the Chassis, and the Chassis
 reaches the Localizer through motion/motion.hpp — so if `Axis` lived beside the call, the Localizer
 would have to include upward through the facade and the includes would close a cycle
 (chassis.hpp -> motion/command_pipeline.hpp -> motion/motion.hpp -> localizer.hpp -> ... ->
 chassis.hpp). The master plan's §5 layering rule forbids that include anyway: no layer includes a
 header from a layer above it. So the enum lives here, alone, below both.

 PROS-free, allocation-free, no includes beyond <cstdint>.
```

</details>
