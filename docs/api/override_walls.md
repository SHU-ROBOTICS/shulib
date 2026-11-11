<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/localization/override_walls.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `override_walls.hpp`

The 2026-27 season's field walls for the wall reset — VEX V5RC "Override".

This header declares **15** constants.

Extracted from [`include/shulib/localization/override_walls.hpp`](../../include/shulib/localization/override_walls.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`kWallInnerFace`](#kwallinnerface) — *constant*
- [`kMetalPerimeterInnerFace`](#kmetalperimeterinnerface) — *constant*
- [`kToggleBodyHalf`](#ktogglebodyhalf) — *constant*
- [`kToggleHalfLength`](#ktogglehalflength) — *constant*
- [`kCornerKeepOut`](#kcornerkeepout) — *constant*
- [`kLoaderCentre`](#kloadercentre) — *constant*
- [`kLoaderHalfWidth`](#kloaderhalfwidth) — *constant*
- [`kStartPiecesInner`](#kstartpiecesinner) — *constant*
- [`kStartPiecesOuter`](#kstartpiecesouter) — *constant*
- [`kLoadZoneDepth`](#kloadzonedepth) — *constant*
- [`kLeftWall`](#kleftwall) — *constant*
- [`kRightWall`](#krightwall) — *constant*
- [`kRedWall`](#kredwall) — *constant*
- [`kBlueWall`](#kbluewall) — *constant*
- [`kWalls`](#kwalls) — *constant*

<a id="kwallinnerface"></a>

## `kWallInnerFace`

```cpp
inline constexpr units::Length kWallInnerFace{1783.2 / 25.4}
```

The perimeter's inner face, from the field's centre: 1783.2 mm (A10, the drawing the field elements are placed on; A13's portable perimeter agrees) = 70.205 in. A4 register HA-207.

*constant, declared at [`include/shulib/localization/override_walls.hpp:79`](../../include/shulib/localization/override_walls.hpp#L79).*

<a id="kmetalperimeterinnerface"></a>

## `kMetalPerimeterInnerFace`

```cpp
inline constexpr units::Length kMetalPerimeterInnerFace{3568.7 / 2.0 / 25.4}
```

The METAL perimeter's inner face (A14: 3568.7 mm wall to wall) = 70.25 in — 0.045 in further out than kWallInnerFace. Stated, not used by the walls below (HA-207 carries the difference inside its tolerance; telling the perimeter type at run time is out of scope). A4 register HA-207.

*constant, declared at [`include/shulib/localization/override_walls.hpp:84`](../../include/shulib/localization/override_walls.hpp#L84).*

<a id="ktogglebodyhalf"></a>

## `kToggleBodyHalf`

```cpp
inline constexpr units::Length kToggleBodyHalf{10.9}
```

Half the Toggle's BODY span along its wall (inches): |along| <= 10.9 in, inside its brackets (A8). A4 register HA-208.

*constant, declared at [`include/shulib/localization/override_walls.hpp:88`](../../include/shulib/localization/override_walls.hpp#L88).*

<a id="ktogglehalflength"></a>

## `kToggleHalfLength`

```cpp
inline constexpr units::Length kToggleHalfLength{25.99 / 2.0}
```

Half the Toggle's whole length, brackets and caps included: 25.99 in / 2 (A8). A4 register HA-208.

*constant, declared at [`include/shulib/localization/override_walls.hpp:91`](../../include/shulib/localization/override_walls.hpp#L91).*

<a id="kcornerkeepout"></a>

## `kCornerKeepOut`

```cpp
inline constexpr units::Length kCornerKeepOut{12.0}
```

How far either side of each inner corner the corner keep-out reaches (inches) — where a cone meets the other wall (it rests on the beam's spread, A4 register HA-205; the Load Zones contain it).

*constant, declared at [`include/shulib/localization/override_walls.hpp:95`](../../include/shulib/localization/override_walls.hpp#L95).*

<a id="kloadercentre"></a>

## `kLoaderCentre`

```cpp
inline constexpr units::Length kLoaderCentre{(3275.8 - 1783.2) / 25.4}
```

The Loaders' centres along the red and blue walls: 1492.6 mm (A10: 3275.8 - 1783.2) = 58.764 in either side of centre. A4 register HA-212.

*constant, declared at [`include/shulib/localization/override_walls.hpp:99`](../../include/shulib/localization/override_walls.hpp#L99).*

<a id="kloaderhalfwidth"></a>

## `kLoaderHalfWidth`

```cpp
inline constexpr units::Length kLoaderHalfWidth{4.02 / 2.0}
```

Half a Loader's width along its wall: 4.02 in / 2 (A9). A4 register HA-212.

*constant, declared at [`include/shulib/localization/override_walls.hpp:102`](../../include/shulib/localization/override_walls.hpp#L102).*

<a id="kstartpiecesinner"></a>

## `kStartPiecesInner`

```cpp
inline constexpr units::Length kStartPiecesInner{1783.2 / 25.4 - (49.82 + 3.15 / 2.0)}
```

The start pieces' inner edge along each wall: the cluster's inner Cup centred 49.82 in from the corner, a Cup 3.15 in across (A12) — 70.205 - (49.82 + 1.575) = 18.81 in. A4 register HA-212.

*constant, declared at [`include/shulib/localization/override_walls.hpp:106`](../../include/shulib/localization/override_walls.hpp#L106).*

<a id="kstartpiecesouter"></a>

## `kStartPiecesOuter`

```cpp
inline constexpr units::Length kStartPiecesOuter{1783.2 / 25.4 - (43.50 - 3.15 / 2.0)}
```

The start pieces' outer edge: the outer Cup centred 43.50 in from the corner (A12) — 70.205 - (43.50 - 1.575) = 28.28 in. A4 register HA-212.

*constant, declared at [`include/shulib/localization/override_walls.hpp:110`](../../include/shulib/localization/override_walls.hpp#L110).*

<a id="kloadzonedepth"></a>

## `kLoadZoneDepth`

```cpp
inline constexpr units::Length kLoadZoneDepth{23.11}
```

How far a VEX U Load Zone reaches from its corner along both walls: 587.1 mm = 23.11 in (A10's tape). A4 register HA-212.

*constant, declared at [`include/shulib/localization/override_walls.hpp:114`](../../include/shulib/localization/override_walls.hpp#L114).*

<a id="kleftwall"></a>

## `kLeftWall`

```cpp
inline constexpr FieldWall kLeftWall = detail::seasonWall("left wall", Axis::X, -kWallInnerFace.value(), +1, false)
```

The LEFT wall (as seen from the red station): x = -70.205, the field on its +x side. Measures x; its keep-outs lie along y. No Loaders.

*constant, declared at [`include/shulib/localization/override_walls.hpp:162`](../../include/shulib/localization/override_walls.hpp#L162).*

<a id="krightwall"></a>

## `kRightWall`

```cpp
inline constexpr FieldWall kRightWall = detail::seasonWall("right wall", Axis::X, kWallInnerFace.value(), -1, false)
```

The RIGHT wall: x = +70.205, the field on its -x side. Measures x. No Loaders.

*constant, declared at [`include/shulib/localization/override_walls.hpp:166`](../../include/shulib/localization/override_walls.hpp#L166).*

<a id="kredwall"></a>

## `kRedWall`

```cpp
inline constexpr FieldWall kRedWall = detail::seasonWall("red wall", Axis::Y, -kWallInnerFace.value(), +1, true)
```

The RED wall (behind the red alliance station): y = -70.205, the field on its +y side. Measures y; its keep-outs lie along x, its two Loaders among them.

*constant, declared at [`include/shulib/localization/override_walls.hpp:171`](../../include/shulib/localization/override_walls.hpp#L171).*

<a id="kbluewall"></a>

## `kBlueWall`

```cpp
inline constexpr FieldWall kBlueWall = detail::seasonWall("blue wall", Axis::Y, kWallInnerFace.value(), -1, true)
```

The BLUE wall: y = +70.205, the field on its -y side. Measures y; two Loaders.

*constant, declared at [`include/shulib/localization/override_walls.hpp:175`](../../include/shulib/localization/override_walls.hpp#L175).*

<a id="kwalls"></a>

## `kWalls`

```cpp
inline constexpr std::array<FieldWall, 4> kWalls{kLeftWall, kRightWall, kRedWall, kBlueWall}
```

The four walls, in the order left, right, red, blue.

*constant, declared at [`include/shulib/localization/override_walls.hpp:179`](../../include/shulib/localization/override_walls.hpp#L179).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 64 lines, click to expand</summary>

```text

 The 2026-27 season's field walls for the wall reset — VEX V5RC "Override" (chunk L6-A1, 2026-10-29).

 ── The frame, and why these walls mean anything ─────────────────────────────────────────────────
 Every coordinate here is the FIELD frame, F1 (master plan; roadmap's frame row): the origin at the
 field's centre, +X to the right as seen from the RED alliance station, +Y away from it. So the
 four walls' inner faces are the lines x = -70.205 (the LEFT wall), x = +70.205 (the RIGHT wall),
 y = -70.205 (the RED wall, behind the red station) and y = +70.205 (the BLUE wall). "Left" and
 "right" are as seen from the red station.

 These walls are meaningful ONLY when the robot's estimate is in field coordinates — that is, when
 the auton declared a FIELD start (`competition::StartPose::field(...)`). A RELATIVE-start auton has
 its origin wherever the robot started, so `kLeftWall` names nothing in its frame and a reset against
 it would seat a field number into a robot frame. The everyday call (`competition::resetFromWall`)
 REFUSES any start that is not a FIELD start; the explicit call (`chassis::resetFromWall`, Tier 3 and the host tests) cannot see
 the start mode, so its author either uses these walls under a field-frame estimate or builds an own
 line with `localization::FieldWall::line` (frame `Own`, the author's keep-outs or none).

 ── Where every number comes from (manual v2.0 — see the version note below) ────────────────────
   * The perimeter: A10 places the field elements on inner faces 3566.4 mm apart, centred at 1783.2
     mm — walls at +/-70.205 in. The two legal perimeters DIFFER: A13 (the portable perimeter) agrees,
     140.40 in inside wall to wall; A14 (the METAL perimeter) is 140.50 in (3568.7 mm), walls at
     +/-70.25 in — 0.045 in further out per wall. That difference is STATED here
     (kMetalPerimeterInnerFace), inside HA-207's tolerance, and never silently averaged: the walls
     below use A10's. (A4 register HA-207.)
   * The Toggle (A8): 25.99 in long, centred on every wall, sitting on brackets ON TOP of the wall.
     A beam below the wall top (11.50 in metal, A14; 11.54 in portable, A13) reads the wall beneath
     the Toggle's BODY — but the BODY's overhang past the inner face is read by eye, and whether a
     cone reaches above the wall top rests on an unofficial cone, so the body's span stays kept out
     until a side's `SideMount::toggleChecked` is MEASURED (kind `Toggle`). Its two BRACKETS, at
     |along| 10.9-12.995 in, hang to about 10.6 in above the floor — into the beam's band — and are
     `Fixed`: never lifted. (A4 register HA-208.)
   * The Loaders (A9, A10): 4.02 in wide, protruding 3.74 in, centred 1492.6 mm (58.764 in) either
     side of centre, on the RED and BLUE walls only (FO-1 puts the alliance stations there). `Fixed`.
     (A4 register HA-212.)
   * The corners: 12 in either side of each inner corner, where the cone meets the other wall.
     Past the wall's end the beam would land on the perpendicular wall, so the span runs on past it.
     `Fixed`.
   * The VEX U Load Zones (A10's tape, 587.1 mm = 23.11 in from each corner along BOTH walls): in VEX U
     Match Loads come in through the Loaders DURING the autonomous period too (<VUG4>) and leave a
     Loader only into its Load Zone (<SG11>), so a robot taking loads, or a dropped (nested) stack,
     may stand there — taller than the beam-height gate assumes. `Fixed` on all four walls, always.
     (A4 register HA-212.)
   * The start pieces (A12): a Cup-Pin-Cup cluster centred 1.58 in off EVERY wall at 43.50 / 46.66 /
     49.82 and 90.59 / 93.75 / 96.91 in from the corner — |along| 18.81-28.28 in to the Cups' outer
     edges (a Cup is 3.15 in across). `PieceZone`: kept out while a mount's cone is unmeasured,
     lifted by a measured one (the beam-height gate then rests on a measured cone). (A4 register
     HA-212.)
 The keep-outs are listed most specific first, so the log line names a Loader as "Loader" and not
 as the Load Zone that contains it.

 ── The manual version (re-checked 2026-10-29 by L6-A1's executor) ──────────────────────────────
 Every number is manual v2.0's (Appendix A: A8-A10, A12-A14). A search on 2026-10-29 showed a
 version table listing v2.1 (released 2026-10-08, effective 2026-10-15, "Clarification / minor
 update"), but its text could not be fetched (the manual pages answered 403) — so whether v2.1 moved
 any number here is UNVERIFIED. A manual change is a one-header change, with its citation.

 ── Two season headers, two namespaces ──────────────────────────────────────────────────────────
 The camera's season data (VIS1) lives in `shulib::vision::override2026`; these walls live in
 `shulib::localization::override2026` — season data beside the subsystem that consumes it, so the two
 never share a scope. Inside `namespace shulib::vision` an unqualified `override2026::` finds the
 camera's: write `localization::override2026::` there.

 PROS-free, allocation-free, constexpr data. Depends only on the pure wall_reset.hpp.
```

</details>
