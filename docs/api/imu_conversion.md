<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/imu_conversion.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `imu_conversion.hpp`

IMU canonical conversions — the ONE place the V5 inertial sensor's frame becomes shulib's canonical frame (§7: "convert exactly once, at the edge").

This header declares **2** free functions.

Extracted from [`include/shulib/hal/imu_conversion.hpp`](../../include/shulib/hal/imu_conversion.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`imuHeadingToCanonical`](#imuheadingtocanonical) — *free function*
- [`imuYawRateToCanonical`](#imuyawratetocanonical) — *free function*

<a id="imuheadingtocanonical"></a>

## `imuHeadingToCanonical`

```cpp
[[nodiscard]] inline math::Angle imuHeadingToCanonical(double imuHeadingDegCw, math::Angle bootHeading)
```

Convert a V5 IMU heading into a canonical field heading. imuHeadingDegCw : degrees, CW-positive, measured from the calibration orientation. bootHeading     : the robot's canonical field heading AT calibration (per-robot config). Returns the canonical heading (CCW-positive, wrapped to (-π, π]). Non-finite input is rejected by Angle::degrees (a NaN can never enter the type).

*free function, declared at [`include/shulib/hal/imu_conversion.hpp:46`](../../include/shulib/hal/imu_conversion.hpp#L46).*

<a id="imuyawratetocanonical"></a>

## `imuYawRateToCanonical`

```cpp
[[nodiscard]] inline units::AngularVelocity imuYawRateToCanonical(double degPerSecCw)
```

Convert a V5 IMU yaw rate (deg/s, CW-positive) to canonical rad/s (CCW-positive). A rate does not wrap; this is the sanctioned deg/s→rad/s conversion (negate for CW→CCW).  CAVEAT (adapter): the CW-positive convention proven for heading does NOT automatically extend to the rate source. PROS exposes no CW yaw-rate scalar — get_gyro_rate() returns a raw body-axis struct whose z sign is UNDOCUMENTED. Prefer deriving yaw rate by differentiating get_rotation() (documented CW-positive) so this negate is provably correct; otherwise bench-verify get_gyro_rate().z's sign before trusting it. (A4 register HA-04; R3 settles it.)

*free function, declared at [`include/shulib/hal/imu_conversion.hpp:61`](../../include/shulib/hal/imu_conversion.hpp#L61).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 30 lines</summary>

```text

 IMU canonical conversions — the ONE place the V5 inertial sensor's frame becomes
 shulib's canonical frame (§7: "convert exactly once, at the edge"). These are
 pure, PROS-free functions so the heading math — the crux of the < 1° requirement —
 is fully host-testable; the hal/pros IMU adapter is thin glue that reads pros::IMU
 and calls these.

 V5 IMU convention (confirmed against the vendored PROS headers — pros/imu.hpp: both
 get_rotation() and get_heading() document "clockwise rotations are positive"): the
 inertial sensor reports DEGREES, CLOCKWISE-positive, from the calibration orientation.
 shulib canonical: RADIANS, CCW-positive, +X = 0, wrapped to (-π, π]. Handedness flips
 by subtraction.

 ADAPTER BINDING CONTRACT (load-bearing — fix when the hal/pros IImu adapter lands):
  * imuHeadingDegCw is CUMULATIVE CW degrees from calibration → bind to
    pros::Imu::get_rotation() (theoretically unbounded), NOT get_heading() (bounded
    [0,360), which loses the revolution continuity this cumulative contract assumes).
    (A4 register HA-03.)
  * The adapter MUST NOT call tare/tare_rotation/set_rotation/tare_heading/set_heading/
    reset after calibration — each re-zeros the sensor independently of bootHeading and
    silently invalidates the additive offset (the < 1° budget has no room to absorb it).
    (A4 register HA-05.)
  * bootHeading has ONE owner: the robot's canonical START heading (the init Pose2d
    handed to the Localizer). The offset is applied exactly ONCE, here at the HAL edge;
    no downstream consumer (odometry / Localizer / EKF) re-applies it. (A4 register HA-05.)

 ON-ROBOT VALIDATION still required: confirm the as-mounted sensor matches its own
 CW-positive doc strings (bench: a known +90° CW spin must DECREASE canonical heading
 by 90°). If a bench test ever disagrees, the sign of the subtraction is the line to flip.
 (A4 register HA-02, docs/hardware-assumptions.md; R3 settles it.)
```

</details>
