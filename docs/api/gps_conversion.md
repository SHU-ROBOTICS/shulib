<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/gps_conversion.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `gps_conversion.hpp`

GPS canonical conversions — the ONE place the VEX GPS frame becomes shulib's canonical frame (§7: "convert exactly once, at the edge").

This header declares **5** free functions and **2** constants.

Extracted from [`include/shulib/hal/gps_conversion.hpp`](../../include/shulib/hal/gps_conversion.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`kGpsDefaultNorthHeadingDeg`](#kgpsdefaultnorthheadingdeg) — *constant*
- [`kMetersToInches`](#kmeterstoinches) — *constant*
- [`gpsRmsErrorToCanonical`](#gpsrmserrortocanonical) — *free function*
- [`gpsHeadingToCanonical`](#gpsheadingtocanonical) — *free function*
- [`gpsSensorPose`](#gpssensorpose) — *free function*
- [`gpsRemoveLeverArm`](#gpsremoveleverarm) — *free function*
- [`gpsToRobotPose`](#gpstorobotpose) — *free function*

<a id="kgpsdefaultnorthheadingdeg"></a>

## `kGpsDefaultNorthHeadingDeg`

```cpp
inline constexpr double kGpsDefaultNorthHeadingDeg = 90.0
```

Default `northHeadingDeg`: 90° — VEX-North points along canonical +Y, away from red. The only other value a legal field setup produces is 270 (≡ −90°), red at the +Y wall. It has ONE owner, the robot's start pose — the same authority as the IMU's bootHeading, and it must be validated on the field (HA-09). Note what it CANNOT do: it is a ROTATION, so it can never repair a mirrored pose from a wrong position-axis binding (HA-01).

*constant, declared at [`include/shulib/hal/gps_conversion.hpp:60`](../../include/shulib/hal/gps_conversion.hpp#L60).*

<a id="kmeterstoinches"></a>

## `kMetersToInches`

```cpp
inline constexpr double kMetersToInches = 39.3700787401574803
```

Metres → inches (1 / 0.0254), the single scale behind every GPS quantity: position in gpsSensorPose() and the device's rms error in gpsRmsErrorToCanonical(). Misapplying it is silent in both directions — omit it and the corrector's R is ~39× too small, so every good fix is gated out and the GPS goes quietly dead; apply it twice and the corrector accepts lies (HA-07). Neither looks like a crash, which is why the scaling is a callable function and not a paragraph an adapter author has to remember.

*constant, declared at [`include/shulib/hal/gps_conversion.hpp:67`](../../include/shulib/hal/gps_conversion.hpp#L67).*

<a id="gpsrmserrortocanonical"></a>

## `gpsRmsErrorToCanonical`

```cpp
[[nodiscard]] inline units::Length gpsRmsErrorToCanonical(double errorMeters)
```

The device's self-reported rms position error (`pros::Gps::get_error()`, **METERS**) as a canonical Length (**INCHES**) — the one conversion behind `IGps::rmsError()`.  This is the whole of A4 register HA-07, and it is a function rather than a comment for a reason: the scale factor is 39.37, the failure is silent in both directions, and the obligation sat in this header as prose from A4 until E2 with nothing executing it. Too small an R and every good fix is gated out (the GPS goes quietly dead); too large and the corrector accepts lies. Neither looks like a crash.  Fail-loud on a sentinel, exactly like `gpsSensorPose`: `PROS_ERR_F` (== INFINITY) is a failed/off-strip read the adapter must have screened to `hasFix() == false` BEFORE asking for an error value (A4 register HA-08), and a negative rms is not a thing a device can mean. Both throw rather than propagate into the corrector's R.

*free function, declared at [`include/shulib/hal/gps_conversion.hpp:82`](../../include/shulib/hal/gps_conversion.hpp#L82).*

<a id="gpsheadingtocanonical"></a>

## `gpsHeadingToCanonical`

```cpp
[[nodiscard]] inline math::Angle gpsHeadingToCanonical( double headingDegCwFromNorth, double northHeadingDeg = kGpsDefaultNorthHeadingDeg)
```

VEX GPS heading (deg, CW from North, [0,360)) → canonical Angle (CCW from +X). canonical = (canonical heading of North) − (CW degrees turned from North), wrapped.

*free function, declared at [`include/shulib/hal/gps_conversion.hpp:91`](../../include/shulib/hal/gps_conversion.hpp#L91).*

<a id="gpssensorpose"></a>

## `gpsSensorPose`

```cpp
[[nodiscard]] inline math::Pose2d gpsSensorPose( double xMeters, double yMeters, double headingDegCwFromNorth, double northHeadingDeg = kGpsDefaultNorthHeadingDeg)
```

Canonical pose of the GPS SENSOR (no lever-arm removal yet) from a raw VEX reading. Position (meters, VEX East/North) is rotated into the canonical frame and scaled to inches; heading is converted as above.

*free function, declared at [`include/shulib/hal/gps_conversion.hpp:101`](../../include/shulib/hal/gps_conversion.hpp#L101).*

<a id="gpsremoveleverarm"></a>

## `gpsRemoveLeverArm`

```cpp
[[nodiscard]] inline math::Pose2d gpsRemoveLeverArm( const math::Pose2d& sensorPose, units::Length leverArmForward, units::Length leverArmLeft)
```

Shift a SENSOR pose to the robot CENTER by removing the lever arm. The lever arm is the GPS sensor's position in the BODY frame (forward = +X, left = +Y, per F1). The sensor sits at center + R(heading)·leverArm, so center = sensor − R(heading)·leverArm.

*free function, declared at [`include/shulib/hal/gps_conversion.hpp:120`](../../include/shulib/hal/gps_conversion.hpp#L120).*

<a id="gpstorobotpose"></a>

## `gpsToRobotPose`

```cpp
[[nodiscard]] inline math::Pose2d gpsToRobotPose( double xMeters, double yMeters, double headingDegCwFromNorth, units::Length leverArmForward, units::Length leverArmLeft, double northHeadingDeg = kGpsDefaultNorthHeadingDeg)
```

Full path: raw VEX reading → canonical robot-CENTER pose.

*free function, declared at [`include/shulib/hal/gps_conversion.hpp:137`](../../include/shulib/hal/gps_conversion.hpp#L137).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 43 lines</summary>

```text

 GPS canonical conversions — the ONE place the VEX GPS frame becomes shulib's
 canonical frame (§7: "convert exactly once, at the edge"). Pure, PROS-free, so the
 frame math is fully host-testable; the hal/pros GPS adapter is thin glue.

 VEX GPS convention: heading in DEGREES [0,360), CLOCKWISE from North (0=N, 90=E,
 180=S, 270=W) — confirmed in the vendored pros/gps.hpp. Position in METERS, (0,0) =
 field center, 4-quadrant Cartesian.

 shulib canonical (F1): inches; FIELD +X right / +Y away-from-red; BODY +X forward /
 +Y left; heading CCW-positive, +X = 0, wrapped (-π,π]. Handedness flips (CW→CCW) by
 subtraction.

 *** ASSUMPTION — position axes (VEX +X = East, +Y = North): UNVERIFIED. PROS does NOT
     document the position-axis-to-compass binding. A WRONG axis label silently MIRRORS
     the pose, and northHeadingDeg CANNOT recover it (that knob is a rotation; an axis
     swap/flip is a reflection). Bench-measure the raw→wall mapping before any scored
     run — see the field-cal oracle test in gps_conversion_test.cpp.
     A4 register HA-01 (docs/hardware-assumptions.md); R3 settles it. ***

 ONE rotation parameter, northHeadingDeg = the canonical heading VEX-North points
 toward. Default 90° (VEX-North = canonical +Y = away from red); the other canonical
 value is 270/−90° (red at the +Y wall). FIELD/ALLIANCE setup fact, ONE owner = the
 robot's start pose (same authority as the IMU bootHeading), validated on field
 (A4 register HA-09).

 ADAPTER BINDING CONTRACT (load-bearing, like the IMU's — enforce when hal/pros lands):
  * Lever arm removed HERE (gpsRemoveLeverArm), ONE owner = robot config (BODY
    forward/left INCHES — NOT the East/North meters of set_offset). Construct pros::Gps
    via the PORT-ONLY ctor; the adapter MUST NOT use set_offset(), initialize_full(),
    or the offset-taking ctors — any firmware offset makes get_position() report the
    CENTER, double-subtracting the arm (inches of silent bias). Boot-check get_offset()==(0,0).
    (A4 register HA-06; the lever-arm VALUE itself is HA-10.)
  * The adapter MUST screen PROS_ERR_F (== INFINITY: a failed/off-strip/calibrating read)
    and report IGps::hasFix() = false BEFORE calling this conversion — off-strip screening
    is the adapter's job (§13 #4; Driving Skills has no strip). Feeding a sentinel here
    THROWS by design (fail-loud backstop, NOT the off-strip path). (A4 register HA-08.)
  * rmsError() at the HAL edge MUST scale get_error() meters→inches — call
    gpsRmsErrorToCanonical() below, which exists so this is a function the adapter CALLS
    rather than a paragraph the adapter author must remember. (Until E2 this obligation
    was prose only: no code performed it and no test pinned it, which is a poor way to
    guard a silent factor of 39.37.) Skip it and the corrector's R is ~39× too small and
    good fixes get gated out; double-apply it and lies get accepted. (A4 register HA-07.)
```

</details>
