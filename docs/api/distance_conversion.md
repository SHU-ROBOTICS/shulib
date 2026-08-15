<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/distance_conversion.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `distance_conversion.hpp`

Distance-sensor canonical conversions — the ONE place the V5 distance sensor's millimeters become shulib's canonical inches and its raw 0–63 confidence becomes [0, 1] (§7: "convert exactly once, at the edge").

This header declares **2** free functions and **3** constants.

Extracted from [`include/shulib/hal/distance_conversion.hpp`](../../include/shulib/hal/distance_conversion.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`kDistanceNoObjectMm`](#kdistancenoobjectmm) — *constant*
- [`kDistanceConfidenceAvailableAboveMm`](#kdistanceconfidenceavailableabovemm) — *constant*
- [`kDistanceConfidenceFullScale`](#kdistanceconfidencefullscale) — *constant*
- [`distanceMmToCanonical`](#distancemmtocanonical) — *free function*
- [`distanceConfidenceToCanonical`](#distanceconfidencetocanonical) — *free function*

<a id="kdistancenoobjectmm"></a>

## `kDistanceNoObjectMm`

```cpp
inline constexpr std::int32_t kDistanceNoObjectMm = 9999
```

The vendored doc's in-band "can not detect an object" value (HA-114), named so the adapter's screen reads as the rule it implements. As a raw int32 comparison value — the sensor's real range tops out far below this.

*constant, declared at [`include/shulib/hal/distance_conversion.hpp:47`](../../include/shulib/hal/distance_conversion.hpp#L47).*

<a id="kdistanceconfidenceavailableabovemm"></a>

## `kDistanceConfidenceAvailableAboveMm`

```cpp
inline constexpr std::int32_t kDistanceConfidenceAvailableAboveMm = 200
```

Below-or-at this raw distance, get_confidence() is documented unavailable ("only available when distance is > 200mm", HA-115) — the adapter reports full confidence there instead, because a returned distance IS a detection.

*constant, declared at [`include/shulib/hal/distance_conversion.hpp:52`](../../include/shulib/hal/distance_conversion.hpp#L52).*

<a id="kdistanceconfidencefullscale"></a>

## `kDistanceConfidenceFullScale`

```cpp
inline constexpr double kDistanceConfidenceFullScale = 63.0
```

Raw confidence full-scale (HA-115): 63 = high confidence.

*constant, declared at [`include/shulib/hal/distance_conversion.hpp:55`](../../include/shulib/hal/distance_conversion.hpp#L55).*

<a id="distancemmtocanonical"></a>

## `distanceMmToCanonical`

```cpp
[[nodiscard]] inline units::Length distanceMmToCanonical(double millimeters)
```

Millimeters (get_distance(), HA-113) → canonical inches. 1 in = 25.4 mm exactly (international inch), so the scale is 1/25.4. Drop the scale and a game piece 3 inches away reads as 76 inches — every capture-confirm threshold in the manipulation layer then never fires.

*free function, declared at [`include/shulib/hal/distance_conversion.hpp:61`](../../include/shulib/hal/distance_conversion.hpp#L61).*

<a id="distanceconfidencetocanonical"></a>

## `distanceConfidenceToCanonical`

```cpp
[[nodiscard]] inline double distanceConfidenceToCanonical(double raw)
```

Raw confidence (get_confidence(), 0–63, HA-115) → canonical [0, 1]. Clamped so the seam contract ("confidence() in [0, 1]") holds even against an out-of-range raw value — clamping is defence, not policy (the controllerAxisToCanonical precedent). Drop the ÷63 and every caller's threshold saturates: 63 reads as 63.0, and "confidence > 0.5" is true for any detection however weak.

*free function, declared at [`include/shulib/hal/distance_conversion.hpp:74`](../../include/shulib/hal/distance_conversion.hpp#L74).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 32 lines</summary>

```text

 Distance-sensor canonical conversions — the ONE place the V5 distance
 sensor's millimeters become shulib's canonical inches and its raw 0–63
 confidence becomes [0, 1] (§7: "convert exactly once, at the edge"). Pure,
 PROS-free, host- and mutation-testable in isolation; the hal/pros IDistance
 adapter is thin glue that CALLS these.

 V5 distance sensor convention (vendored pros/distance.hpp:63-158):
  * get_distance() is int32 MILLIMETERS (distance.hpp:63; HA-113) — and
    "will return 9999 if the sensor can not detect an object"
    (distance.hpp:71,98 — an IN-BAND plain integer, NOT PROS_ERR; HA-114).
    9999 mm converts to a perfectly plausible-looking 393.66 inches, which is
    why the adapter maps it to confidence()==0 BEFORE any caller can mistake
    it for a wall 33 feet away. kNoObjectMm below is that magic number, named.
  * get_confidence() is int32 with "a range of 0 to 63. 63 means high
    confidence" — and it is "only available when distance is > 200mm"
    (distance.hpp:133-135; HA-115). What it returns AT OR BELOW 200 mm is
    NOT DOCUMENTED, so the adapter must not report it there (the adapter's
    close-range rule; hal/pros/distance.hpp).

 ── ADAPTER BINDING CONTRACT (the clause-by-clause list hal/pros/distance.hpp
    must honour — the imu/gps_conversion.hpp pattern) ─────────────────────────
  1. Screen PROS_ERR (device failure) BEFORE calling these — the finiteness
     preconditions here are a backstop, not the screen (T7).
  2. Check the raw distance against kNoObjectMm BEFORE converting: 9999 is a
     READING (empty intake = normal state), never a fault — map it to
     confidence 0, keep distance finite, raise nothing (T4).
  3. Report confidence 1.0 for a valid reading at or below
     kConfidenceAvailableAboveMm: the returned distance IS the detection;
     the raw confidence channel is undefined there (HA-115).

 HA register: HA-113, HA-114, HA-115 (docs/hardware-assumptions.md).
```

</details>
