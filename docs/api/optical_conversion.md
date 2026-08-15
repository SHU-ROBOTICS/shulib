<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/optical_conversion.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `optical_conversion.hpp`

Optical-sensor canonical conversions — the ONE place the V5 optical sensor's raw channels become shulib's canonical ranges (§7: "convert exactly once, at the edge").

This header declares **3** free functions.

Extracted from [`include/shulib/hal/optical_conversion.hpp`](../../include/shulib/hal/optical_conversion.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`opticalHueToCanonical`](#opticalhuetocanonical) — *free function*
- [`opticalUnitIntervalToCanonical`](#opticalunitintervaltocanonical) — *free function*
- [`opticalProximityToCanonical`](#opticalproximitytocanonical) — *free function*

<a id="opticalhuetocanonical"></a>

## `opticalHueToCanonical`

```cpp
[[nodiscard]] inline double opticalHueToCanonical(double raw)
```

Raw hue (get_hue(), 0–359.999, HA-116) → canonical degrees [0, 360). Clamped to the doc's own stated range so the seam contract holds even against an out-of-range raw value (a raw 360.0 must not leak through the half-open interval). A COLOR hue, never a heading — the seam's own rule (optical.hpp header).

*free function, declared at [`include/shulib/hal/optical_conversion.hpp:39`](../../include/shulib/hal/optical_conversion.hpp#L39).*

<a id="opticalunitintervaltocanonical"></a>

## `opticalUnitIntervalToCanonical`

```cpp
[[nodiscard]] inline double opticalUnitIntervalToCanonical(double raw)
```

Raw saturation or brightness (0–1.0, HA-116) → canonical [0, 1]. Identity plus the defensive clamp — one function for both channels because the vendored doc gives them the same range and meaning shape.

*free function, declared at [`include/shulib/hal/optical_conversion.hpp:48`](../../include/shulib/hal/optical_conversion.hpp#L48).*

<a id="opticalproximitytocanonical"></a>

## `opticalProximityToCanonical`

```cpp
[[nodiscard]] inline double opticalProximityToCanonical(double raw)
```

Raw proximity (get_proximity(), 0–255, HA-117) → canonical [0, 1] with ≈1 meaning close (the seam's contract — resting on HA-117's UNMEASURED larger-is-closer polarity). Drop the ÷255 and "proximity > 0.8" is true the moment anything reflects at all — capture-confirm fires on an empty intake.

*free function, declared at [`include/shulib/hal/optical_conversion.hpp:59`](../../include/shulib/hal/optical_conversion.hpp#L59).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 24 lines</summary>

```text

 Optical-sensor canonical conversions — the ONE place the V5 optical sensor's
 raw channels become shulib's canonical ranges (§7: "convert exactly once, at
 the edge"). Pure, PROS-free, host- and mutation-testable in isolation; the
 hal/pros IOptical adapter is thin glue that CALLS these.

 V5 optical sensor convention (vendored pros/optical.hpp:76-170):
  * get_hue() is a double with "a range of 0 to 359.999" (optical.hpp:79-80;
    HA-116) — already the seam's degree unit, so the conversion is a clamp
    that makes the [0, 360) contract unconditional rather than
    true-if-the-doc-is-right.
  * get_saturation() / get_brightness() are doubles with "a range of 0 to
    1.0" (optical.hpp:103-104,127-128; HA-116) — identity plus the same
    defensive clamp.
  * get_proximity() is int32 with "a range of 0 to 255" (optical.hpp:151-152;
    HA-117) — and the vendored doc documents ONLY the range: the belief that
    LARGER means CLOSER is community knowledge, not vendored text. HA-117
    flags it weak, and the bench measures it before any capture threshold is
    written against proximity().
  * Sentinels (HA-118): the double channels return PROS_ERR_F (INFINITY) on
    failure, proximity returns PROS_ERR — the adapter screens BEFORE calling
    these (T7); the finiteness preconditions here are the backstop.

 HA register: HA-116, HA-117, HA-118 (docs/hardware-assumptions.md).
```

</details>
