<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/pros/gps.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `gps.hpp`

ProsGps — IGps over pros::Gps (chunk R1a): the absolute-position corrector's sensor behind the HAL.

This header declares **1** type (5 members).

Extracted from [`include/shulib/hal/pros/gps.hpp`](../../include/shulib/hal/pros/gps.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class ProsGps`](#class-prosgps)
  - [`ProsGps`](#prosgps-prosgps)
  - [`pose`](#prosgps-pose)
  - [`rmsError`](#prosgps-rmserror)
  - [`hasFix`](#prosgps-hasfix)
  - [`faultedReads`](#prosgps-faultedreads)

<a id="class-prosgps"></a>

## `class ProsGps`

```cpp
class ProsGps final : public IGps
```

IGps over pros::Gps — the absolute-position corrector's sensor, converted to canonical robot-CENTER inches at this seam.  Construction is PORT-ONLY on purpose, and the constructor VERIFIES it: this adapter owns the lever-arm removal, so a firmware offset configured on the device would make the arm be subtracted twice — inches of silent, heading-dependent bias. A nonzero `get_offset()` at construction is a precondition failure; an unreadable one (device absent or calibrating) defers the check and leaves hasFix() false until a later read can settle it, and if THAT read finds an offset the device is marked no-fix for the whole run rather than throwing from a read path.  Each reader takes ONE atomic device sample (position and heading from the same status), and screens PROS_ERR_F sentinels to no-fix BEFORE any conversion runs — feeding a sentinel into the conversion helpers throws by design, and keeping that path unreachable is this class's job. While fix-less, pose() and rmsError() hold their last good values.

*class, declared at [`include/shulib/hal/pros/gps.hpp:87`](../../include/shulib/hal/pros/gps.hpp#L87).*

<a id="prosgps-prosgps"></a>

### `ProsGps::ProsGps`

```cpp
ProsGps(std::uint8_t port, units::Length leverArmForward, units::Length leverArmLeft, double northHeadingDeg = kGpsDefaultNorthHeadingDeg)
```

PORT-ONLY device construction (header). `leverArmForward`/`leverArmLeft`: the sensor's position in the BODY frame (inches — HA-10, robot config). `northHeadingDeg`: the canonical heading VEX-North points toward (HA-09, same owner as the robot's start pose).

*function, declared at [`include/shulib/hal/pros/gps.hpp:93`](../../include/shulib/hal/pros/gps.hpp#L93).*

<a id="prosgps-pose"></a>

### `ProsGps::pose`

```cpp
[[nodiscard]] math::Pose2d pose() const override
```

Canonical robot-CENTER pose. Finite always; meaningful only while hasFix() (gps.hpp:26-30). Never throws.

*function, declared at [`include/shulib/hal/pros/gps.hpp:109`](../../include/shulib/hal/pros/gps.hpp#L109).*

<a id="prosgps-rmserror"></a>

### `ProsGps::rmsError`

```cpp
[[nodiscard]] units::Length rmsError() const override
```

Canonical inches via gpsRmsErrorToCanonical (HA-07 — the ×39.37 that was prose until E2). Holds last-good while fix-less.

*function, declared at [`include/shulib/hal/pros/gps.hpp:116`](../../include/shulib/hal/pros/gps.hpp#L116).*

<a id="prosgps-hasfix"></a>

### `ProsGps::hasFix`

```cpp
[[nodiscard]] bool hasFix() const override
```

Device-level validity ONLY: this read's fields were all finite and the firmware offset is verified (0,0). It says nothing about whether the fix is ACCURATE enough to fold — that threshold belongs to the fusion corrector, which already owns it; two layers gating on rmsError() with two thresholds would fight invisibly. Like pose() and rmsError(), calling this takes a FRESH device sample, so it is not a free predicate.

*function, declared at [`include/shulib/hal/pros/gps.hpp:126`](../../include/shulib/hal/pros/gps.hpp#L126).*

<a id="prosgps-faultedreads"></a>

### `ProsGps::faultedReads`

```cpp
[[nodiscard]] int faultedReads() const noexcept
```

How many reads were screened to no-fix (T7 observability).

*function, declared at [`include/shulib/hal/pros/gps.hpp:132`](../../include/shulib/hal/pros/gps.hpp#L132).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 52 lines, click to expand</summary>

```text

 ProsGps — IGps over pros::Gps (chunk R1a): the absolute-position corrector's
 sensor behind the HAL.

 BINDS: get_position_and_orientation() — ONE atomic status read carrying
 x/y (meters) and yaw (CW-from-North degrees, HA-106), so position and
 heading in a single IGps::pose() come from the SAME device sample rather
 than three reads that could straddle an update. Plus get_error() (meters,
 HA-07) and get_offset() (the HA-06 boot check). Conversion: gpsToRobotPose()
 — sensor meters → canonical robot-CENTER inches, lever arm removed exactly
 once — and gpsRmsErrorToCanonical(), the function E2 built "so this is a
 function the adapter CALLS rather than a paragraph the adapter author must
 remember". This adapter is that author, and it calls both.

 CONSTRUCTION — PORT-ONLY, enforced (HA-06): the ctor takes the port and
 NOTHING that could reach the firmware offset. It never calls set_offset(),
 initialize_full(), or the offset-taking ctors: any firmware offset makes
 get_position() report the robot CENTER, and this adapter's own lever-arm
 removal would then subtract the arm TWICE — inches of silent,
 heading-dependent bias. The boot check verifies get_offset() == (0,0):
  * offset readable and (0,0)  → verified, normal operation;
  * offset readable and NONZERO at CONSTRUCTION → SHULIB_PRECONDITION (a
    configured device is a robot-setup error — loud at boot beats
    double-subtracted at match);
  * offset UNREADABLE (device absent/calibrating at construction) → the
    check DEFERS: hasFix() stays false until a later read CAN verify the
    offset. If the DEFERRED check then finds a NONZERO offset, the device is
    marked permanently no-fix for the run instead of throwing — pose() and
    hasFix() are read paths and MUST NOT throw (gps.hpp:27-29); the
    corrector sees a dead GPS (quality decay), which is honest and safe,
    where an inches-biased "healthy" GPS is neither. Never trusts an
    unverified device either way.

 SENTINEL SCREENING BEFORE CONVERSION (HA-08): PROS_ERR_F in any status
 field → hasFix() = false for that read, pose() returns the last good pose
 (finite by construction — gps.hpp:27-29: unspecified but finite, callers
 gate on hasFix()). Feeding a sentinel into gpsSensorPose()/
 gpsRmsErrorToCanonical() THROWS by design — that path is the fail-loud
 backstop, not the off-strip path, and this adapter's job is to make it
 unreachable.

 WHAT hasFix() MEANS HERE — device-level validity ONLY: the reads are finite
 and the offset is verified. Error-MAGNITUDE gating (is this fix worth
 folding?) deliberately stays in the fusion layer, which already owns it
 with registered thresholds (E2's corrector, HA-61..67) — two layers gating
 on the same number with two thresholds would fight, invisibly.

 DELIBERATELY NOT here: no northHeadingDeg guessing (ctor parameter, owner =
 the robot's start-pose authority, HA-09); no lever-arm value (ctor
 parameters, robot config, HA-10); no filtering, no outlier logic (E2's).

 HA register: HA-01, HA-06..HA-09, HA-106.
```

</details>
