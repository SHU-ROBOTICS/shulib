<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/drive_geometry.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `drive_geometry.hpp`

DriveGeometry — ONE description of a drive side's geometry, consumed by BOTH the drive-encoder odometry and the stall cross-check.

This header declares **1** type (5 members).

Extracted from [`include/shulib/hal/drive_geometry.hpp`](../../include/shulib/hal/drive_geometry.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`struct DriveGeometry`](#struct-drivegeometry)
  - [`wheelRadius`](#drivegeometry-wheelradius)
  - [`motorToWheelRatio`](#drivegeometry-motortowheelratio)
  - [`inchesPerRadian`](#drivegeometry-inchesperradian)
  - [`fromDiameter`](#drivegeometry-fromdiameter)
  - [`valid`](#drivegeometry-valid)

<a id="struct-drivegeometry"></a>

## `struct DriveGeometry`

```cpp
struct DriveGeometry
```

One drive side's motor-to-wheel geometry: wheel radius and external motor→wheel ratio, yielding inches of wheel travel per radian of motor output-shaft rotation. Plain data (designated initializers at the call site), validated by whoever consumes it (valid()). Instantiate one per side at the composition root and hand the SAME object to the odometry and the stall check — one representation, two consumers.

*struct, declared at [`include/shulib/hal/drive_geometry.hpp:43`](../../include/shulib/hal/drive_geometry.hpp#L43).*

<a id="drivegeometry-wheelradius"></a>

### `DriveGeometry::wheelRadius`

```cpp
units::Length wheelRadius{3.25 / 2.0}
```

Wheel RADIUS (inches). Stand-in default: half of 3.25 in (A4: HA-14).

*field, declared at [`include/shulib/hal/drive_geometry.hpp:45`](../../include/shulib/hal/drive_geometry.hpp#L45).*

<a id="drivegeometry-motortowheelratio"></a>

### `DriveGeometry::motorToWheelRatio`

```cpp
double motorToWheelRatio = 1.0
```

External gearing, motor output shaft → wheel: wheel revolutions per motor revolution (1.0 = direct drive; 0.6 = a 5:3 reduction; the cartridge is NOT included — the adapter already reports OUTPUT-shaft rotation). Stand-in default 1:1 (A4: HA-14).

*field, declared at [`include/shulib/hal/drive_geometry.hpp:49`](../../include/shulib/hal/drive_geometry.hpp#L49).*

<a id="drivegeometry-inchesperradian"></a>

### `DriveGeometry::inchesPerRadian`

```cpp
[[nodiscard]] constexpr units::Length inchesPerRadian() const noexcept
```

Inches of wheel travel per radian of motor output-shaft rotation: radius × ratio.

*function, declared at [`include/shulib/hal/drive_geometry.hpp:52`](../../include/shulib/hal/drive_geometry.hpp#L52).*

<a id="drivegeometry-fromdiameter"></a>

### `DriveGeometry::fromDiameter`

```cpp
[[nodiscard]] static constexpr DriveGeometry fromDiameter(units::Length wheelDiameter, double ratio = 1.0) noexcept
```

A geometry built from what a human measures: the wheel DIAMETER (a ruler across the tread) and the ratio (tooth counts, or 1.0 for direct drive).

*function, declared at [`include/shulib/hal/drive_geometry.hpp:58`](../../include/shulib/hal/drive_geometry.hpp#L58).*

<a id="drivegeometry-valid"></a>

### `DriveGeometry::valid`

```cpp
[[nodiscard]] bool valid() const noexcept
```

Both fields finite and strictly positive — what a consumer's precondition checks. UNSET (0) is invalid on purpose: a geometry nobody measured must refuse, never silently scale travel by zero.

*function, declared at [`include/shulib/hal/drive_geometry.hpp:66`](../../include/shulib/hal/drive_geometry.hpp#L66).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 29 lines</summary>

```text

 DriveGeometry — ONE description of a drive side's geometry, consumed by BOTH the
 drive-encoder odometry and the stall cross-check (chunk R3b Part 2, 2026-09-14).

 ── Why one type, and why it lives here ─────────────────────────────────────────────
 Converting a drive motor's shaft radians into inches of wheel travel needs two facts
 about the mechanism: the wheel's radius and the external gearing between the motor's
 output shaft and the wheel. Two consumers need that conversion — the drive-encoder
 odometry (localization/drive_encoder_odometry.hpp) and the spin-vs-motion stall check
 (motion/odo_stall_check.hpp) — and until this chunk each had a different answer: the
 stall check carried ONE scalar `wheelRadius` for every drive motor (with a comment A29
 predicted would be wrong on a geared robot), and the odometry did not exist. The R3b
 brief's ruling (first-closed-loop §4.1, session-2 §5.3): the per-side ratio lives WITH
 THE MOTORS, never in F5 kinematics (which answers "how fast should each side's wheel
 travel", symmetric and frozen), and it is represented ONCE, instantiated once per side
 at the composition root, and handed to both consumers. This header is that one type,
 in hal/ because it describes the physical motor-to-wheel mechanism the adapters sit on
 and because both consumers can include hal/ without a layering exception.

 Asymmetric sides are representable — two objects, one per side — because a real
 drivetrain has been seen with "the right side geared down a touch for traction"
 (the bench bot's legacy source, recorded in the development log on the shulib-v2
 branch). Robot two is BELIEVED symmetric; belief is not a reason to make
 asymmetry unrepresentable.

 The defaults are the tree's STAND-IN geometry (3.25 in wheel, 1:1 — A4 register HA-14),
 carried here unchanged from OdoStallCheckConfig so a config that names no geometry
 behaves exactly as before. Robot two's measured values arrive through the chassis
 table (src/chassis_table.hpp), never through these defaults.
```

</details>
