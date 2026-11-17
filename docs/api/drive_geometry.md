<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/drive_geometry.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `drive_geometry.hpp`

DriveGeometry — ONE description of a drive side's geometry, consumed by BOTH the drive-encoder odometry and the stall cross-check.

This header declares **1** type (6 members).

Extracted from [`include/shulib/hal/drive_geometry.hpp`](../../include/shulib/hal/drive_geometry.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in [How it works](../guide/README.md); worked recipes live in [Recipes](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`struct DriveGeometry`](#struct-drivegeometry)
  - [`wheelRadius`](#drivegeometry-wheelradius)
  - [`motorToWheelRatio`](#drivegeometry-motortowheelratio)
  - [`inchesPerRadian`](#drivegeometry-inchesperradian)
  - [`fromDiameter`](#drivegeometry-fromdiameter)
  - [`fromInchesPerRadian`](#drivegeometry-frominchesperradian)
  - [`valid`](#drivegeometry-valid)

<a id="struct-drivegeometry"></a>

## `struct DriveGeometry`

```cpp
struct DriveGeometry
```

One drive side's motor-to-wheel geometry: wheel radius and external motor→wheel ratio, yielding inches of wheel travel per radian of motor output-shaft rotation. Plain data (designated initializers at the call site), validated by whoever consumes it (valid()). Instantiate one per side at the composition root and hand the SAME object to the odometry and the stall check — one representation, two consumers.

*struct, declared at [`include/shulib/hal/drive_geometry.hpp:61`](../../include/shulib/hal/drive_geometry.hpp#L61).*

<a id="drivegeometry-wheelradius"></a>

### `DriveGeometry::wheelRadius`

```cpp
units::Length wheelRadius{3.25 / 2.0}
```

Wheel RADIUS (inches) — EXCEPT in a geometry built by fromInchesPerRadian(), where this field carries the whole radius × ratio PRODUCT and the ratio beside it is 1.0 (see the header's "two constructors, two provenances"). Stand-in default: half of 3.25 in (A4: HA-14).

*field, declared at [`include/shulib/hal/drive_geometry.hpp:66`](../../include/shulib/hal/drive_geometry.hpp#L66).*

<a id="drivegeometry-motortowheelratio"></a>

### `DriveGeometry::motorToWheelRatio`

```cpp
double motorToWheelRatio = 1.0
```

External gearing, motor output shaft → wheel: wheel revolutions per motor revolution (1.0 = direct drive; 0.6 = a 5:3 reduction; the cartridge is NOT included — the adapter already reports OUTPUT-shaft rotation). Stand-in default 1:1 (A4: HA-14).

*field, declared at [`include/shulib/hal/drive_geometry.hpp:70`](../../include/shulib/hal/drive_geometry.hpp#L70).*

<a id="drivegeometry-inchesperradian"></a>

### `DriveGeometry::inchesPerRadian`

```cpp
[[nodiscard]] constexpr units::Length inchesPerRadian() const noexcept
```

Inches of wheel travel per radian of motor output-shaft rotation: radius × ratio.

*function, declared at [`include/shulib/hal/drive_geometry.hpp:73`](../../include/shulib/hal/drive_geometry.hpp#L73).*

<a id="drivegeometry-fromdiameter"></a>

### `DriveGeometry::fromDiameter`

```cpp
[[nodiscard]] static constexpr DriveGeometry fromDiameter(units::Length wheelDiameter, double ratio = 1.0) noexcept
```

A geometry built from what a human measures: the wheel DIAMETER (a ruler across the tread) and the ratio (tooth counts, or 1.0 for direct drive).

*function, declared at [`include/shulib/hal/drive_geometry.hpp:79`](../../include/shulib/hal/drive_geometry.hpp#L79).*

<a id="drivegeometry-frominchesperradian"></a>

### `DriveGeometry::fromInchesPerRadian`

```cpp
[[nodiscard]] static constexpr DriveGeometry fromInchesPerRadian( units::Length inchesPerRadian) noexcept
```

A geometry built from what the ROBOT measures (chunk R3d Part 1): inches of wheel travel per radian of motor output-shaft rotation, straight from a push of a known distance along a tape. The radius and the ratio are NOT separable from that measurement and are not invented here: the product is stored in `wheelRadius` with a ratio of exactly 1.0, so inchesPerRadian() returns it unchanged (× 1.0 is exact). A calibrated geometry's `wheelRadius` is therefore NOT a radius — see the header.

*function, declared at [`include/shulib/hal/drive_geometry.hpp:90`](../../include/shulib/hal/drive_geometry.hpp#L90).*

<a id="drivegeometry-valid"></a>

### `DriveGeometry::valid`

```cpp
[[nodiscard]] bool valid() const noexcept
```

Both fields finite and strictly positive — what a consumer's precondition checks. UNSET (0) is invalid on purpose: a geometry nobody measured must refuse, never silently scale travel by zero.

*function, declared at [`include/shulib/hal/drive_geometry.hpp:98`](../../include/shulib/hal/drive_geometry.hpp#L98).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 47 lines, click to expand</summary>

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
 table (src/robots/chassis_table.hpp), never through these defaults.

 ── TWO CONSTRUCTORS, TWO PROVENANCES — and what wheelRadius then means ─────────────
 fromDiameter(d, ratio) is what a HUMAN measures: a ruler across the tread and a pair of
 tooth counts, two separable facts.

 fromInchesPerRadian(k) is what a ROBOT measures (chunk R3d Part 1, T5): pushing the
 chassis a known distance along a tape and dividing by the shaft radians it turned yields
 ONE number, and nothing in that measurement can separate the radius from the ratio. So
 the calibrated constructor stores k in `wheelRadius` with `motorToWheelRatio = 1.0`, and
 — stated here because it is the one surprising thing about this type — **a CALIBRATED
 geometry's `wheelRadius` field carries the PRODUCT, not a radius.** Both consumers
 (drive_encoder_odometry.hpp, odo_stall_check.hpp) read only inchesPerRadian(), so neither
 can tell the difference and neither needs to; only a PANEL does, and a panel showing a
 calibrated description must print "ipr", never "radius".
 Rejected: a third field or a tagged variant on a type two consumers already hold BY VALUE
 (a shape change that buys no consumer anything); and storing a guessed diameter with a
 ratio derived to match — that is HA-111's invented-constant defect class in a new coat,
 two made-up factors standing in for one measured number.
```

</details>
