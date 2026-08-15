<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/pros/rotation.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `rotation.hpp`

ProsRotation — IRotation over pros::Rotation (chunk R1a): the tracking-wheel pods behind the HAL.

This header declares **1** type (4 members).

Extracted from [`include/shulib/hal/pros/rotation.hpp`](../../include/shulib/hal/pros/rotation.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class ProsRotation`](#class-prosrotation)
  - [`ProsRotation`](#prosrotation-prosrotation)
  - [`position`](#prosrotation-position)
  - [`velocity`](#prosrotation-velocity)
  - [`faultedReads`](#prosrotation-faultedreads)

<a id="class-prosrotation"></a>

## `class ProsRotation`

```cpp
class ProsRotation final : public IRotation
```

IRotation over `pros::Rotation` — a tracking-wheel pod behind the HAL. It converts the sensor's int32 CENTIDEGREES to canonical radians and rad/s exactly once, at this edge, and screens the PROS_ERR sentinel by HOLDING THE LAST GOOD VALUE. That screen is the whole substance of this class: PROS_ERR is in-band for an int32 centidegree reading (INT32_MAX is ~59652 revolutions — finite, just unreachable in a match), and a faulted pod that reported zero would read as "the robot stopped", which is exactly the dead-encoder runaway the loop's cross-check exists to catch; a frozen value is what that cross-check is designed to see. ONE gap, and it is the boot case: the last-good caches are SEEDED TO ZERO and there is no have-a-value flag, so a pod that faults on its very FIRST read — unplugged, or not yet enumerated — publishes 0 rad and 0 rad/s, the exact reading the screen otherwise prevents. faultedReads() is the only way to tell that apart from a genuinely stationary robot, so check it before trusting a zero. Reversal is NOT applied here — PROS applies it when the sensor is constructed on a negative port, once, so this adapter never negates. There is deliberately no reset: odometry owns its own zero, and re-zeroing the device mid-run would step every consumer at the same instant.

*class, declared at [`include/shulib/hal/pros/rotation.hpp:72`](../../include/shulib/hal/pros/rotation.hpp#L72).*

<a id="prosrotation-prosrotation"></a>

### `ProsRotation::ProsRotation`

```cpp
explicit ProsRotation(std::int8_t port)
```

`port`: 1..21, NEGATIVE to reverse (PROS applies the reversal — once).

*function, declared at [`include/shulib/hal/pros/rotation.hpp:75`](../../include/shulib/hal/pros/rotation.hpp#L75).*

<a id="prosrotation-position"></a>

### `ProsRotation::position`

```cpp
[[nodiscard]] units::AngleDim position() const override
```

Cumulative radians (never wrapped). Sentinel-screened to last-good (T7).

*function, declared at [`include/shulib/hal/pros/rotation.hpp:78`](../../include/shulib/hal/pros/rotation.hpp#L78).*

<a id="prosrotation-velocity"></a>

### `ProsRotation::velocity`

```cpp
[[nodiscard]] units::AngularVelocity velocity() const override
```

Shaft angular velocity in rad/s, from the sensor's centidegrees/s. Sentinel-screened like position(): a faulted read returns the last good rate — but note the seed, `lastVelocity_` starts at 0 rad/s, so a pod that has never once answered reports exactly the "robot stopped" rate the screen exists to suppress; check faultedReads() before trusting a zero. Both readers are `const` and yet ADVANCE state — the last-good caches and faultedReads() — because the screen is part of the read, not commanded state; two calls in a tick may therefore differ.

*function, declared at [`include/shulib/hal/pros/rotation.hpp:95`](../../include/shulib/hal/pros/rotation.hpp#L95).*

<a id="prosrotation-faultedreads"></a>

### `ProsRotation::faultedReads`

```cpp
[[nodiscard]] int faultedReads() const noexcept
```

How many reads were screened to last-good (T7 observability).

*function, declared at [`include/shulib/hal/pros/rotation.hpp:106`](../../include/shulib/hal/pros/rotation.hpp#L106).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 39 lines</summary>

```text

 ProsRotation — IRotation over pros::Rotation (chunk R1a): the tracking-wheel
 pods behind the HAL.

 BINDS:
  * get_position() [int32 centidegrees, cumulative] → position()
    (rotationCentidegToCanonical — HA-11/HA-16)
  * get_velocity() [int32 centidegrees/s] → velocity()
    (rotationCentidegPerSecToCanonical — HA-105)

 REVERSAL — "exactly once" (rotation.hpp:4-6): a NEGATIVE ctor port reverses
 the sensor, applied by PROS itself (vendored rotation.hpp:46-47). This
 adapter therefore does NOT negate, ever — sign lives in the port number the
 robot config states, one owner, one application.

 SENTINELS (T7): IRotation has no validity channel. PROS_ERR (= INT32_MAX)
 is IN-BAND for an int32 centidegree reading — 2147483647 centideg is ~59652
 revolutions, unreachable in a match (HA-11's arithmetic), so it is screened
 as a sentinel HERE: hold the last good value, never propagate, never zero
 (a zeroed tracking wheel reads as "the robot stopped" — the exact
 dead-encoder runaway the loop's ODO_STUCK cross-check exists to catch; a
 frozen last-good value is what that cross-check is DESIGNED to see).
 faultedReads() exposes the screen count; raising a fault stays with the
 loop layer (health_monitor.hpp: raising is policy, hal/ is below diag/).

 SEED CAVEAT — an OPEN DEFECT, recorded rather than papered over: the
 last-good caches start at 0 and there is no have-a-value flag, so "never
 zero" only holds AFTER the first successful read. A pod that faults from
 the very first tick (unplugged, or not yet enumerated at boot) publishes
 0 rad / 0 rad/s — the precise reading the screen above claims to prevent,
 and the one the ODO_STUCK cross-check reads as a stopped robot. The tell
 is faultedReads() > 0 alongside a zero output. A validity flag (or a
 NaN/sentinel seed) is the fix; it is not applied here.

 DELIBERATELY NOT here: no reset()/set_position() calls — odometry owns its
 own zero (TrackingWheel reads deltas); re-zeroing the device mid-run would
 step every consumer at once.

 HA register: HA-11, HA-16, HA-105 (docs/hardware-assumptions.md).
```

</details>
