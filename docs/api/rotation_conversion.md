<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/rotation_conversion.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `rotation_conversion.hpp`

Rotation-sensor canonical conversions — the ONE place the V5 rotation sensor's centidegrees become shulib's canonical radians (§7: "convert exactly once, at the edge").

This header declares **2** free functions.

Extracted from [`include/shulib/hal/rotation_conversion.hpp`](../../include/shulib/hal/rotation_conversion.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`rotationCentidegToCanonical`](#rotationcentidegtocanonical) — *free function*
- [`rotationCentidegPerSecToCanonical`](#rotationcentidegpersectocanonical) — *free function*

<a id="rotationcentidegtocanonical"></a>

## `rotationCentidegToCanonical`

```cpp
[[nodiscard]] inline units::AngleDim rotationCentidegToCanonical(double centidegrees)
```

Cumulative centidegrees (get_position(), HA-11/HA-16) → canonical cumulative radians. Deliberately NOT math::Angle: odometry integrates total tracking-wheel travel, so this must never wrap (rotation.hpp:8-9). π/18000 because 1 centidegree = (1/100)° = (1/100)·(π/180) rad. Drop the scale and every tracking-wheel delta is 5730× — one inch of travel reads as 477 feet, and odometry is garbage from the first tick.

*free function, declared at [`include/shulib/hal/rotation_conversion.hpp:35`](../../include/shulib/hal/rotation_conversion.hpp#L35).*

<a id="rotationcentidegpersectocanonical"></a>

## `rotationCentidegPerSecToCanonical`

```cpp
[[nodiscard]] inline units::AngularVelocity rotationCentidegPerSecToCanonical( double centidegPerSec)
```

Centidegrees-per-second (get_velocity(), HA-105) → canonical rad/s. Same scale as position — one factor, one place.

*free function, declared at [`include/shulib/hal/rotation_conversion.hpp:44`](../../include/shulib/hal/rotation_conversion.hpp#L44).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 18 lines</summary>

```text

 Rotation-sensor canonical conversions — the ONE place the V5 rotation sensor's
 centidegrees become shulib's canonical radians (§7: "convert exactly once, at
 the edge"). Pure, PROS-free, host- and mutation-testable in isolation; the
 hal/pros IRotation adapter is thin glue that CALLS these.

 V5 rotation sensor convention (vendored pros/rotation.hpp:195-242): both
 get_position() and get_velocity() are int32 CENTIDEGREES (position cumulative,
 never wrapping — HA-11; 36000 ticks/rev — HA-16; velocity in centideg/s —
 HA-105). The sensor's reversed flag is applied by PROS itself when the sensor
 is constructed on a NEGATIVE port (rotation.hpp:46-47) — that is the "exactly
 once" of rotation.hpp:4-6, and the adapter must NOT negate again on top.

 Sentinel note (T7): PROS_ERR (= INT32_MAX) is IN-BAND for an int32
 centidegree reading — 2147483647 centideg is ~59652 revolutions, unreachable
 in a match (HA-11's arithmetic) but perfectly finite. ONLY the adapter can
 screen it, BEFORE calling these. The conversions take double so the
 PROS_ERR_F backstop still works if a future float path appears.
```

</details>
