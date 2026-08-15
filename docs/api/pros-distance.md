<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/pros/distance.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `distance.hpp`

ProsDistance — IDistance over pros::Distance (chunk R1b): the capture/dock-confirm rangefinder behind the HAL.

This header declares **1** type (4 members).

Extracted from [`include/shulib/hal/pros/distance.hpp`](../../include/shulib/hal/pros/distance.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class ProsDistance`](#class-prosdistance)
  - [`ProsDistance`](#prosdistance-prosdistance)
  - [`distance`](#prosdistance-distance)
  - [`confidence`](#prosdistance-confidence)
  - [`faultedReads`](#prosdistance-faultedreads)

<a id="class-prosdistance"></a>

## `class ProsDistance`

```cpp
class ProsDistance final : public IDistance
```

IDistance over pros::Distance: millimetres become canonical inches and the raw 0–63 confidence becomes [0, 1], each converted exactly once. The value of this adapter is the three screens it puts in front of PROS's sentinels. (1) A raw 9999 — PROS's plain IN-BAND "cannot detect an object", which converts to a perfectly plausible 393.66 in — reports confidence 0.0 and raises nothing: an empty intake is a normal state, and a fault every tick would cry wolf. (2) At or below 200 mm, where get_confidence() is documented as unavailable and its value undefined, confidence is 1.0, because there the returned distance IS the detection. (3) PROS_ERR is a DEVICE failure — unplugged, wrong port — and is different: distance() holds the last good finite value (initially the far no-object distance, never 0.0, so a sensor dead from boot cannot read "object touching the sensor"), confidence() is 0.0, and faultedReads() counts it. Raising a fault is the loop layer's job, not this one's. Reads are LIVE: each accessor hits the device, so distance() and confidence() are two samples, not one atomic snapshot.

*class, declared at [`include/shulib/hal/pros/distance.hpp:76`](../../include/shulib/hal/pros/distance.hpp#L76).*

<a id="prosdistance-prosdistance"></a>

### `ProsDistance::ProsDistance`

```cpp
explicit ProsDistance(std::uint8_t port)
```

`port`: 1..21.

*function, declared at [`include/shulib/hal/pros/distance.hpp:79`](../../include/shulib/hal/pros/distance.hpp#L79).*

<a id="prosdistance-distance"></a>

### `ProsDistance::distance`

```cpp
[[nodiscard]] units::Length distance() const override
```

Canonical inches. 9999 (no object) converts honestly — far and finite — with confidence() reporting 0.0 (the T4 rule); PROS_ERR holds the last good value (T7).

*function, declared at [`include/shulib/hal/pros/distance.hpp:84`](../../include/shulib/hal/pros/distance.hpp#L84).*

<a id="prosdistance-confidence"></a>

### `ProsDistance::confidence`

```cpp
[[nodiscard]] double confidence() const override
```

[0, 1]; 0.0 = no usable return (no object, or device failure). Full 1.0 at or below 200 mm, where a returned distance IS the detection (the close-range rule).

*function, declared at [`include/shulib/hal/pros/distance.hpp:97`](../../include/shulib/hal/pros/distance.hpp#L97).*

<a id="prosdistance-faultedreads"></a>

### `ProsDistance::faultedReads`

```cpp
[[nodiscard]] int faultedReads() const noexcept
```

How many DEVICE-FAILURE reads were screened (T7 observability). The 9999 no-object state is NOT counted — it is a reading, not a fault.

*function, declared at [`include/shulib/hal/pros/distance.hpp:119`](../../include/shulib/hal/pros/distance.hpp#L119).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 45 lines</summary>

```text

 ProsDistance — IDistance over pros::Distance (chunk R1b): the
 capture/dock-confirm rangefinder behind the HAL.

 BINDS:
  * get_distance() [int32 mm; HA-113] → distance()  (distanceMmToCanonical)
  * get_confidence() [int32 0–63; HA-115] → confidence()
    (distanceConfidenceToCanonical)

 THE 9999 RULE (T4 — the most important lines in chunk R1b): PROS reports
 "can not detect an object" as a PLAIN IN-BAND 9999, not PROS_ERR (vendored
 distance.hpp:71,98; HA-114). 9999 mm converts to 393.66 inches — a
 perfectly plausible-looking wall 33 feet away. This adapter maps raw 9999
 to confidence() == 0.0, IDistance's existing contract for "no usable
 return" (distance.hpp:28), which callers already threshold on. It is a
 READING, not a fault: an empty intake is a normal state, and raising a
 fault would cry wolf every tick — so no fault is raised and faultedReads()
 does NOT count it. distance() stays finite (F4 finiteness): it reports the
 honest conversion of 9999 — far, and failing every proximity threshold —
 never a held stale object (a stale "object present" is the dangerous
 direction for capture-confirm).

 THE CLOSE-RANGE RULE: get_confidence() is "only available when distance is
 > 200mm" (vendored distance.hpp:133-135; HA-115) — what it returns at or
 below 200 mm is UNDOCUMENTED. An object at 100 mm is exactly when a
 mechanism cares most, and passing the undefined raw value through could
 read "object touching the sensor, zero confidence". So: a valid reading at
 or below 200 mm reports confidence 1.0 — the returned distance IS the
 detection; the raw channel is not consulted where PROS says it does not
 exist.

 SENTINELS (T7): PROS_ERR (device failure — unplugged, wrong port) is a
 DIFFERENT state from 9999 and is screened separately: distance() holds the
 last good FINITE value, confidence() reports 0.0, faultedReads() counts it.
 The hold's INITIAL value is the far no-object distance, not 0.0 — a sensor
 dead from boot must not read "object touching the sensor". Raising a fault
 stays with the loop layer (hal/ is below diag/, the R1a T7 refinement).

 DELIBERATELY NOT BOUND: get_object_size() (0–400, -1 in-band for
 "undeterminable") and get_object_velocity() (m/s) — IDistance's seam does
 not carry them, no consumer asks for them, and each would import another
 in-band sentinel to rule on. If a consumer appears, they enter through a
 seam amendment, not through this adapter quietly growing.

 HA register: HA-113, HA-114, HA-115 (docs/hardware-assumptions.md).
```

</details>
