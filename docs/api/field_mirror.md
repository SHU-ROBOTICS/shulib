<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/competition/field_mirror.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `field_mirror.hpp`

competition::FieldMirror — ONE ROUTE, TWO ROBOTS.

This header declares **1** type (3 members) and **9** free functions.

Extracted from [`include/shulib/competition/field_mirror.hpp`](../../include/shulib/competition/field_mirror.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in [How it works](../guide/README.md); worked recipes live in [Recipes](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`struct FieldMirror`](#struct-fieldmirror)
  - [`name`](#fieldmirror-name)
  - [`reflection`](#fieldmirror-reflection)
  - [`walls`](#fieldmirror-walls)
- [`mirroredSide`](#mirroredside) — *free function*
- [`reflectedWall`](#reflectedwall) — *free function*
- [`seasonImageIndex`](#seasonimageindex) — *free function*
- [`mirroredWall`](#mirroredwall) — *free function*
- [`mirrored`](#mirrored) — *free function*
- [`mirrored (overload 2)`](#mirrored-2) — *free function*
- [`isMirrored`](#ismirrored) — *free function*
- [`runReflection`](#runreflection) — *free function*
- [`seatedStart`](#seatedstart) — *free function*

<a id="struct-fieldmirror"></a>

## `struct FieldMirror`

```cpp
struct FieldMirror
```

A field's mirror (the file banner): its name for the log, its reflection, and the season's walls — the wall reset's wall is mapped to the season wall at its reflected line. A SEASON CONSTANT with static storage (an entry keeps a pointer to it): `override2026::kSkillsMirror`.

*struct, declared at [`include/shulib/competition/field_mirror.hpp:88`](../../include/shulib/competition/field_mirror.hpp#L88).*

<a id="fieldmirror-name"></a>

### `FieldMirror::name`

```cpp
const char* name = ""
```

Its name in the log ("the skills diagonal") — a stable literal, ASCII.

*field, declared at [`include/shulib/competition/field_mirror.hpp:90`](../../include/shulib/competition/field_mirror.hpp#L90).*

<a id="fieldmirror-reflection"></a>

### `FieldMirror::reflection`

```cpp
math::Reflection reflection{}
```

The reflection of the FIELD frame (never the identity — mirrored() refuses one).

*field, declared at [`include/shulib/competition/field_mirror.hpp:92`](../../include/shulib/competition/field_mirror.hpp#L92).*

<a id="fieldmirror-walls"></a>

### `FieldMirror::walls`

```cpp
std::span<const localization::FieldWall> walls{}
```

The season's walls (localization::override2026::kWalls): the image of a season wall is the season wall at its reflected line, with ITS real name and keep-outs.

*field, declared at [`include/shulib/competition/field_mirror.hpp:95`](../../include/shulib/competition/field_mirror.hpp#L95).*

<a id="mirroredside"></a>

## `mirroredSide`

```cpp
[[nodiscard]] constexpr localization::WallSide mirroredSide(localization::WallSide side) noexcept
```

The sensor on the OTHER side: LEFT <-> RIGHT. Every reflection swaps a robot's sides — robot one's right sensor along a wall is robot two's left sensor along the image wall.

*free function, declared at [`include/shulib/competition/field_mirror.hpp:100`](../../include/shulib/competition/field_mirror.hpp#L100).*

<a id="reflectedwall"></a>

## `reflectedWall`

```cpp
[[nodiscard]] constexpr localization::FieldWall reflectedWall(const math::Reflection& r, const localization::FieldWall& wall) noexcept
```

`wall` reflected GEOMETRICALLY by `r`: its line `axis = coord` mapped to the image line, the field on the image side, every keep-out span mapped along the image wall (order, kind and name kept). A wall's name and frame are kept as they are. (mirroredWall, below, is what the wall reset uses.)

*free function, declared at [`include/shulib/competition/field_mirror.hpp:112`](../../include/shulib/competition/field_mirror.hpp#L112).*

<a id="seasonimageindex"></a>

## `seasonImageIndex`

```cpp
[[nodiscard]] constexpr int seasonImageIndex(const FieldMirror& mirror, const localization::FieldWall& wall) noexcept
```

The index in `mirror.walls` of the season wall at `wall`'s reflected line (the same axis, the same inner face within 1e-6 in, the field on the same side), or -1 when none is — an author's own line, or a FIELD wall the season does not have.

*free function, declared at [`include/shulib/competition/field_mirror.hpp:142`](../../include/shulib/competition/field_mirror.hpp#L142).*

<a id="mirroredwall"></a>

## `mirroredWall`

```cpp
[[nodiscard]] constexpr localization::FieldWall mirroredWall(const FieldMirror& mirror, const localization::FieldWall& wall) noexcept
```

THE WALL RESET'S WALL under `mirror` (the file banner): a SEASON wall (frame Field) maps to the season wall at its reflected line — with that wall's REAL name and keep-outs (Override's red wall carries Loaders the right wall does not: the image is the right wall as it is, not the red wall's spans moved); an author's OWN line (frame Own), or a Field wall the season lacks, is reflected geometrically (reflectedWall), its name kept.

*free function, declared at [`include/shulib/competition/field_mirror.hpp:158`](../../include/shulib/competition/field_mirror.hpp#L158).*

<a id="mirrored"></a>

## `mirrored`

```cpp
[[nodiscard]] inline AutonEntry mirrored(const AutonEntry& entry, const FieldMirror& mirror)
```

`entry` MIRRORED by `mirror` (the file banner) — a copy that robot two's autons file registers: `registry.add(competition::mirrored(skillsRoute(), override2026::kSkillsMirror));`. It TOGGLES: an entry already mirrored by the same reflection comes back as written. `mirror` is BORROWED (a season constant with static storage). PRECONDITIONS (a programming error, thrown): `mirror` reflects (not the identity) and has a name; the entry is not already mirrored by a DIFFERENT reflection (D5). Nothing else is checked here: the registry's own check refuses a mirrored name over kMaxMirroredAutonNameChars when the entry is registered. The ROBOT is not mirrored — a part off its centre line meets the field on the other side of the reflected path (A4 register HA-259).

*free function, declared at [`include/shulib/competition/field_mirror.hpp:177`](../../include/shulib/competition/field_mirror.hpp#L177).*

<a id="mirrored-2"></a>

## `mirrored (overload 2)`

```cpp
AutonEntry mirrored(const AutonEntry& entry, const FieldMirror&& mirror) = delete
```

REFUSED AT COMPILE TIME: mirrored() handed a TEMPORARY FieldMirror. The entry keeps the mirror's ADDRESS for its whole life (the registry holds it; the runner, the wall reset and the Toggle step read it at run time), and a temporary is gone by the end of the line — a dangling pointer no warning catches. Hand it the season constant, which has static storage: `mirrored(skillsRoute(), override2026::kSkillsMirror)`. (Deleting the const rvalue form catches a non-const temporary too: it binds there before it would bind to the const reference.)

*free function, declared at [`include/shulib/competition/field_mirror.hpp:199`](../../include/shulib/competition/field_mirror.hpp#L199).*

<a id="ismirrored"></a>

## `isMirrored`

```cpp
[[nodiscard]] inline bool isMirrored(const AutonContext& ctx) noexcept
```

True while `ctx` runs a MIRRORED entry — robot two's run of the shared route. The shared route's way to send ONE robot to what lies on the field's mirror line (the file banner; the brief's D11): `if (!competition::isMirrored(ctx)) { /* robot one only */ }`.

*free function, declared at [`include/shulib/competition/field_mirror.hpp:204`](../../include/shulib/competition/field_mirror.hpp#L204).*

<a id="runreflection"></a>

## `runReflection`

```cpp
[[nodiscard]] inline math::Reflection runReflection(const AutonEntry& entry)
```

The reflection a run of `entry` uses (the file banner's table): the identity for an entry as written; for a mirrored one, its FieldMirror's reflection under a FIELD start and the ROBOT-frame mirror (math::Reflection::Line::XAxis) under a RELATIVE one. An UNSET start (refused at registration) answers the identity.

*free function, declared at [`include/shulib/competition/field_mirror.hpp:210`](../../include/shulib/competition/field_mirror.hpp#L210).*

<a id="seatedstart"></a>

## `seatedStart`

```cpp
[[nodiscard]] inline math::Pose2d seatedStart(const AutonEntry& entry)
```

Where a run of `entry` SEATS the robot — the entry's effective start, reflected by runReflection(): robot two's REAL start for a mirrored entry (where the drivers must place it), the start as written otherwise.

*free function, declared at [`include/shulib/competition/field_mirror.hpp:225`](../../include/shulib/competition/field_mirror.hpp#L225).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 68 lines, click to expand</summary>

```text

 competition::FieldMirror — ONE ROUTE, TWO ROBOTS (chunk MIRROR1-A, 2026-11-13; the MIRROR1 brief's
 D1–D11, the team lead's GO of 2026-11-13: "do what is best they should play onto one side when they
 need to when mirrored, do what you need to do").

 ── What it is for ───────────────────────────────────────────────────────────────────────────────
 In a VEX U skills run both robots start on red's side, one in each red Quadrant (the red wall's and
 the right wall's), and those two Quadrants are mirror images of each other across the field's
 diagonal (competition/override_mirror.hpp says which line and why, from the game manual). So the
 students write the skills route ONCE, in robot one's numbers, and robot two's autons file registers
 the same entry MIRRORED, in one line:

     registry.add(skillsRoute());                                                    // robot one
     registry.add(competition::mirrored(skillsRoute(), override2026::kSkillsMirror));  // robot two

 `mirrored()` copies the entry and marks it; the auton runner (match_runner.hpp) does the rest when it
 runs that entry: it seats robot two at the REFLECTED start, and opens a chassis::MirroredRun around
 the routine (or route) AND its end action, so every field number the route writes is reflected at
 the Chassis's front door (chassis.hpp's MIRRORED RUN) and every pose the route reads comes back in
 robot one's numbers. The wall reset (auton_wall_reset.hpp) and the Toggle step (toggle_visit.hpp)
 work below that door; under the entry's mirror they swap the wall (its image), the sensor side (LEFT
 <-> RIGHT) and the Toggle (its image) themselves. Sizes — timeouts, speeds, tolerances, a trigger's
 percent / inches / degrees, mechanism goals — are not field numbers and do not change.

 ── The start mode decides WHICH reflection the run uses (the brief's D7) ──────────────────────────
   FIELD start     the FieldMirror's own reflection (Override: (x, y, h) -> (-y, -x, -90 deg - h)).
   RELATIVE start  the ROBOT-frame mirror (x, y, h) -> (x, -y, -h): "every left a right". A relative
                   auton's numbers are in the frame of wherever the robot was placed; when robot two
                   is placed at the field-mirror image of robot one's spot, the field's mirror IS this
                   flip. The library cannot see where the drivers put the robot.

 ── BOTH ROBOTS RUN AT THE SAME TIME (the brief's D11) ─────────────────────────────────────────────
 A reflection leaves every point of its line where it is: whenever robot one's robot reaches the
 diagonal, robot two's reaches the same spot at the same moment. In a pure mirror the gap between the
 two robots is exactly twice robot one's clearance from the diagonal, whatever their timing — so a
 mirrored route keeps robot one's WHOLE robot at least 3 in off the diagonal (then the robots stay
 at least 6 in apart), and anything ON the diagonal (the centre goal, the two yellow Pins of the
 skills setup on it, the red/right corner's Load Zone) is for ONE robot, chosen in the shared route:

     if (!competition::isMirrored(ctx)) { /* robot one only: the centre goal */ }

 The host suite runs every mirrored entry robot two registers TOGETHER with its unmirrored self and
 fails, naming the moment and the motions, when their 15 in squares come within 6 in
 (test/mirrored_pair_check.hpp; A4 register HA-260).

 ── What the mirror cannot carry (told, never guessed) ──────────────────────────────────────────────
 The ROBOT is not mirrored: a part mounted off its centre line (a side claw, an off-centre camera,
 the two side sensors at different distances ahead of the centre) meets the field on the other side
 of the reflected path (A4: HA-259); its own left/right drive differences are not mirrored either
 (HA-260). The field is not symmetric everywhere: Override's Loaders and the Load Zones' colours do
 not mirror (override_mirror.hpp). The camera's field inputs (a vision::PieceTarget, the pose handed
 to the piece finder or the Toggle tracker) are plain numbers the library cannot tell from any other:
 they are the REAL field — feed them the localizer's pose (`ctx.chassis.deps().localizer->pose()`),
 never `ctx.chassis.pose()` inside a mirrored run (robot one's numbers), and send a real-field fix to
 a Chassis verb through `ctx.chassis.reflection().apply(...)` first.

 ── Refused by name (the brief's D5) ───────────────────────────────────────────────────────────────
   * an entry mirrored by two DIFFERENT mirrors — a precondition at mirrored() (twice by the same
     mirror gives the entry back as written: mirrored() TOGGLES);
   * a mirrored entry whose name is over kMaxMirroredAutonNameChars (25) — refused at registration by
     the registry's own check (its menu button reads "<name> SKILLS MIR" on one line);
   * the EXPLICIT wall reset (chassis::resetFromWall over a sensor handle) inside a mirrored run — a
     precondition naming the everyday form, which swaps the sensor side.
 A refusal at run time can only happen when its line runs, because ONE function serves both robots:
 the host suite runs every auton of robot two's registry on the host test plant, mirrored ones
 included, before the robot ever does.

 PROS-free, allocation-free, header-only. NOTHING HERE HAS RUN ON A ROBOT.
```

</details>
