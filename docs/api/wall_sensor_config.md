<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/config/wall_sensor_config.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `wall_sensor_config.hpp`

WallSensorsConfig — the robot file's SIDE DISTANCE SENSORS block.

This header declares **2** types (7 members), **3** free functions, and **9** constants.

Extracted from [`include/shulib/config/wall_sensor_config.hpp`](../../include/shulib/config/wall_sensor_config.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`kWallMountMaxOffsetInches`](#kwallmountmaxoffsetinches) — *constant*
- [`kWallMountMaxHeightInches`](#kwallmountmaxheightinches) — *constant*
- [`kWallMountMaxPositionToleranceInches`](#kwallmountmaxpositiontoleranceinches) — *constant*
- [`kWallMountMaxFacingToleranceDeg`](#kwallmountmaxfacingtolerancedeg) — *constant*
- [`kWallMountMaxBiasInches`](#kwallmountmaxbiasinches) — *constant*
- [`kWallMountMinAccuracyInches`](#kwallmountminaccuracyinches) — *constant*
- [`kWallMountMaxAccuracyInches`](#kwallmountmaxaccuracyinches) — *constant*
- [`kWallRangeFloorInches`](#kwallrangefloorinches) — *constant*
- [`kWallMinSamplesFloor`](#kwallminsamplesfloor) — *constant*
- [`struct WallSensorConfig`](#struct-wallsensorconfig)
  - [`port`](#wallsensorconfig-port)
  - [`mount`](#wallsensorconfig-mount)
- [`struct WallSensorsConfig`](#struct-wallsensorsconfig)
  - [`left`](#wallsensorsconfig-left)
  - [`right`](#wallsensorsconfig-right)
  - [`limits`](#wallsensorsconfig-limits)
  - [`note`](#wallsensorsconfig-note)
  - [`declared`](#wallsensorsconfig-declared)
- [`wallMountProblem`](#wallmountproblem) — *free function*
- [`wallSideProblem`](#wallsideproblem) — *free function*
- [`wallLimitsProblem`](#walllimitsproblem) — *free function*

<a id="kwallmountmaxoffsetinches"></a>

## `kWallMountMaxOffsetInches`

```cpp
inline constexpr double kWallMountMaxOffsetInches = 24.0
```

The farthest a side sensor's lens may sit from the robot's centre, forward or sideways (inches): a VEX U robot is at most 24 in across, so more is a typo, not a mount.

*constant, declared at [`include/shulib/config/wall_sensor_config.hpp:60`](../../include/shulib/config/wall_sensor_config.hpp#L60).*

<a id="kwallmountmaxheightinches"></a>

## `kWallMountMaxHeightInches`

```cpp
inline constexpr double kWallMountMaxHeightInches = 11.5
```

The highest a side sensor's lens may sit (inches): the field wall's top — manual drawing A14's 11.50 in (the metal perimeter; A13's portable one is 11.54). A lens above it reads past the wall (A4 register HA-208).

*constant, declared at [`include/shulib/config/wall_sensor_config.hpp:64`](../../include/shulib/config/wall_sensor_config.hpp#L64).*

<a id="kwallmountmaxpositiontoleranceinches"></a>

## `kWallMountMaxPositionToleranceInches`

```cpp
inline constexpr double kWallMountMaxPositionToleranceInches = 2.0
```

The widest position tolerance a measured mount may state (inches).

*constant, declared at [`include/shulib/config/wall_sensor_config.hpp:66`](../../include/shulib/config/wall_sensor_config.hpp#L66).*

<a id="kwallmountmaxfacingtolerancedeg"></a>

## `kWallMountMaxFacingToleranceDeg`

```cpp
inline constexpr double kWallMountMaxFacingToleranceDeg = 10.0
```

The widest facing tolerance a measured mount may state (degrees).

*constant, declared at [`include/shulib/config/wall_sensor_config.hpp:68`](../../include/shulib/config/wall_sensor_config.hpp#L68).*

<a id="kwallmountmaxbiasinches"></a>

## `kWallMountMaxBiasInches`

```cpp
inline constexpr double kWallMountMaxBiasInches = 2.0
```

The largest range bias a measured mount may state, either way (inches).

*constant, declared at [`include/shulib/config/wall_sensor_config.hpp:70`](../../include/shulib/config/wall_sensor_config.hpp#L70).*

<a id="kwallmountminaccuracyinches"></a>

## `kWallMountMinAccuracyInches`

```cpp
inline constexpr double kWallMountMinAccuracyInches = 0.05
```

The narrowest and widest measured accuracy a mount may state (inches); 0 = unmeasured.

*constant, declared at [`include/shulib/config/wall_sensor_config.hpp:72`](../../include/shulib/config/wall_sensor_config.hpp#L72).*

<a id="kwallmountmaxaccuracyinches"></a>

## `kWallMountMaxAccuracyInches`

```cpp
inline constexpr double kWallMountMaxAccuracyInches = 2.0
```

See kWallMountMinAccuracyInches.

*constant, declared at [`include/shulib/config/wall_sensor_config.hpp:74`](../../include/shulib/config/wall_sensor_config.hpp#L74).*

<a id="kwallrangefloorinches"></a>

## `kWallRangeFloorInches`

```cpp
inline constexpr double kWallRangeFloorInches = 0.79
```

The shortest `WallResetLimits::rangeMin` a robot file may type (inches): VEX's published 20-mm floor (0.787 in), rounded up (A4 register HA-204).

*constant, declared at [`include/shulib/config/wall_sensor_config.hpp:77`](../../include/shulib/config/wall_sensor_config.hpp#L77).*

<a id="kwallminsamplesfloor"></a>

## `kWallMinSamplesFloor`

```cpp
inline constexpr int kWallMinSamplesFloor = 3
```

The fewest `WallResetLimits::minSamples` a robot file may type: a median of fewer than three samples cannot outvote one outlier.

*constant, declared at [`include/shulib/config/wall_sensor_config.hpp:80`](../../include/shulib/config/wall_sensor_config.hpp#L80).*

<a id="struct-wallsensorconfig"></a>

## `struct WallSensorConfig`

```cpp
struct WallSensorConfig
```

One side's distance sensor, as the robot file declares it: its port, and its MEASURED mount.

*struct, declared at [`include/shulib/config/wall_sensor_config.hpp:83`](../../include/shulib/config/wall_sensor_config.hpp#L83).*

<a id="wallsensorconfig-port"></a>

### `WallSensorConfig::port`

```cpp
std::uint8_t port = 0
```

The sensor's smart port, 1..21; 0 = no sensor on this side (the default).

*field, declared at [`include/shulib/config/wall_sensor_config.hpp:85`](../../include/shulib/config/wall_sensor_config.hpp#L85).*

<a id="wallsensorconfig-mount"></a>

### `WallSensorConfig::mount`

```cpp
localization::SideMount mount{}
```

Where the sensor sits on the robot and how well that was measured — UNSET until measured (header): the sensor is built, and every reset with it refuses MOUNT UNSET.

*field, declared at [`include/shulib/config/wall_sensor_config.hpp:88`](../../include/shulib/config/wall_sensor_config.hpp#L88).*

<a id="struct-wallsensorsconfig"></a>

## `struct WallSensorsConfig`

```cpp
struct WallSensorsConfig
```

The robot's side distance sensors (header): one per side, and the wall reset's limits.

*struct, declared at [`include/shulib/config/wall_sensor_config.hpp:92`](../../include/shulib/config/wall_sensor_config.hpp#L92).*

<a id="wallsensorsconfig-left"></a>

### `WallSensorsConfig::left`

```cpp
WallSensorConfig left{}
```

The LEFT side's sensor (its beam out the robot's left; `facing` typically `90.0 * localization::kWallDegToRad`).

*field, declared at [`include/shulib/config/wall_sensor_config.hpp:95`](../../include/shulib/config/wall_sensor_config.hpp#L95).*

<a id="wallsensorsconfig-right"></a>

### `WallSensorsConfig::right`

```cpp
WallSensorConfig right{}
```

The RIGHT side's sensor (`facing` typically `-90.0 * localization::kWallDegToRad`).

*field, declared at [`include/shulib/config/wall_sensor_config.hpp:97`](../../include/shulib/config/wall_sensor_config.hpp#L97).*

<a id="wallsensorsconfig-limits"></a>

### `WallSensorsConfig::limits`

```cpp
localization::WallResetLimits limits{}
```

The wall reset's gate limits for this robot — PROVISIONAL library defaults unless typed (A4 register HA-211).

*field, declared at [`include/shulib/config/wall_sensor_config.hpp:100`](../../include/shulib/config/wall_sensor_config.hpp#L100).*

<a id="wallsensorsconfig-note"></a>

### `WallSensorsConfig::note`

```cpp
const char* note = ""
```

A note for a boot line and the panel, e.g. "both centred, 10.5 in up" — a stable literal.

*field, declared at [`include/shulib/config/wall_sensor_config.hpp:102`](../../include/shulib/config/wall_sensor_config.hpp#L102).*

<a id="wallsensorsconfig-declared"></a>

### `WallSensorsConfig::declared`

```cpp
[[nodiscard]] constexpr bool declared() const noexcept
```

True when either side declares a sensor (a port that is not 0).

*function, declared at [`include/shulib/config/wall_sensor_config.hpp:105`](../../include/shulib/config/wall_sensor_config.hpp#L105).*

<a id="wallmountproblem"></a>

## `wallMountProblem`

```cpp
[[nodiscard]] constexpr const char* wallMountProblem(const localization::SideMount& m) noexcept
```

The first problem with a side sensor's MEASURED mount (header), or nullptr. An UNMEASURED mount (`measured == false`) has none: it is not a config error — every reset with it refuses MOUNT UNSET.

*free function, declared at [`include/shulib/config/wall_sensor_config.hpp:117`](../../include/shulib/config/wall_sensor_config.hpp#L117).*

<a id="wallsideproblem"></a>

## `wallSideProblem`

```cpp
[[nodiscard]] constexpr const char* wallSideProblem(const localization::SideMount& m, localization::WallSide side) noexcept
```

The first problem with a MEASURED mount for the side it is declared on (header), or nullptr — run after wallMountProblem accepted it. A sensor on a side faces out of that side and sits on that side of the robot's centre line: the LEFT one's `facing` strictly between 0 and 180 degrees (typically +90) and its `left` above 0; the RIGHT one's `facing` strictly between -180 and 0 (typically -90) and its `left` below 0. The usual cause is a block copied from the other side: a facing the wrong way refuses every reset NOT SQUARE, and an offset the wrong way puts the answer about twice the offset off — refused DISAGREES, or SEATED that far wrong under a generous `maxCorrection`. An UNMEASURED mount has none.

*free function, declared at [`include/shulib/config/wall_sensor_config.hpp:172`](../../include/shulib/config/wall_sensor_config.hpp#L172).*

<a id="walllimitsproblem"></a>

## `wallLimitsProblem`

```cpp
[[nodiscard]] constexpr const char* wallLimitsProblem(const localization::WallResetLimits& l) noexcept
```

The first problem with a robot's wall-reset LIMITS (header), or nullptr. Every limits block this accepts is one localization::wallResetProblem accepts too, so a valid robot file never meets BAD REQUEST from its own limits.

*free function, declared at [`include/shulib/config/wall_sensor_config.hpp:202`](../../include/shulib/config/wall_sensor_config.hpp#L202).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 47 lines, click to expand</summary>

```text

 WallSensorsConfig — the robot file's SIDE DISTANCE SENSORS block (chunk L6-A2, 2026-11-03; the L6
 brief's D5, D6 and D7 — the team lead's answers of 2026-10-22).

 ── DATA the robot file carries, so it lives in config/ ─────────────────────────────
 The robot file is the one place a robot's facts live (VIS1's camera block is the template): one
 V5 Distance Sensor on each SIDE of the robot, facing straight out, each with its port and its
 MEASURED mount (localization::SideMount), and the robot's wall-reset gate limits
 (localization::WallResetLimits — the library's PROVISIONAL defaults unless the team typed its own).
 config/robot_config.hpp carries only the hooks: `SensorsConfig::walls`, `PortUse::WallSensor`, and
 these checks in `RobotConfig::valid()`. robot::Robot builds each declared side through the device
 provider; G3a's robot panel is to write this block like every other (it writes none yet).

 ── UNSET is not an error ───────────────────────────────────────────────────────────
 `SideMount::measured == false` (the default) is UNSET: the sensor is still built, and every wall
 reset with it refuses `MOUNT UNSET` — never the robot. So a robot boots and drives the day its
 sensors are plugged in, and the reset starts working the day someone measures the mount with a
 tape. No value is ever invented to make a block "complete" (HA-111's lesson): `valid()` judges a
 mount's numbers only once it says `measured = true`.

 ── What valid() refuses, and why each bound ────────────────────────────────────────
 A MEASURED mount: a number that is not finite; a `facing` outside (-180, 180] degrees (the likeliest
 typo: `90.0` typed alone is 90 RADIANS — every angle here is `N * kWallDegToRad`); a sensor that does
 not face out of, or sit on, its OWN side (`wallSideProblem`: the LEFT one's facing between 0 and 180
 degrees and its `left` above 0, the RIGHT one's mirrored — a block copied from the other side would
 refuse every reset NOT SQUARE, or put the answer about twice the offset off); the lens more than 24 in from the robot's centre
 forward or sideways (bigger than any robot this season may be — a typo, not a robot); a height of 0
 or less, or above 11.5 in (the field wall's top, A4 register HA-208: a lens above it reads past the
 wall, never the wall); a measured beam half-angle outside 3–30 degrees (0 = unmeasured; A4 register
 HA-205); a position tolerance of 0 or less or above 2 in, a facing tolerance of 0 or less or above
 10 degrees (nothing is measured perfectly, and a tolerance that loose is not a measurement); a range
 bias above 2 in either way; a measured accuracy outside 0.05–2 in (0 = unmeasured: VEX's published
 figures, A4 register HA-204); no provenance (who measured it, how, when).
 THE LIMITS, always: everything the wall reset itself would refuse as a bad request (so a valid robot
 file never meets BAD REQUEST from its own limits), and tighter, so a typed limit stays inside the
 span its derivation covers (A4 register HA-211): a range floor under 0.79 in (VEX's 20 mm) or a
 ceiling not above it; fewer than 3 or more than 64 samples; a window longer than its own maximum, a
 maximum past 1 s, a window past 0.6 s; a heading floor outside 0.5–10 degrees; a heading growth
 outside 0–0.1 per radian turned; an along-wall drift rate outside 0–0.5; a piece top or a beam
 margin below 0; and the rest's own ranges.

 The angle edges are compared in RADIANS against `N * kWallDegToRad` — the very value a robot file
 types — so a number typed exactly on an edge is accepted exactly (localization/wall_reset.hpp's
 wallResetProblem does the same, for the same reason).

 PROS-free, sim-free, allocation-free, constexpr-constructible (robot two's block sits in a
 constexpr table, L6-B).
```

</details>
