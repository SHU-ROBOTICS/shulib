<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/competition/override_mirror.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `override_mirror.hpp`

The 2026-27 season's SKILLS MIRROR — VEX V5RC / VEX U "Override".

This header declares **2** free functions and **1** constant.

Extracted from [`include/shulib/competition/override_mirror.hpp`](../../include/shulib/competition/override_mirror.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in [How it works](../guide/README.md); worked recipes live in [Recipes](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`kSkillsMirror`](#kskillsmirror) — *constant*
- [`toggleImage`](#toggleimage) — *free function*
- [`mirroredToggle`](#mirroredtoggle) — *free function*

<a id="kskillsmirror"></a>

## `kSkillsMirror`

```cpp
inline constexpr FieldMirror kSkillsMirror{ .name = "the skills diagonal", .reflection = math::Reflection::across(math::Reflection::Line::AntiDiagonal), .walls = localization::override2026::kWalls, }
```

Override's SKILLS mirror (the file banner): the reflection across F1's y = -x — the diagonal from the red/right corner to the blue/left corner — that maps the red wall's Quadrant onto the right wall's. Robot two's autons file: `registry.add(competition::mirrored(skillsRoute(), kSkillsMirror));`. A4 register HA-258 (the event's field set up to Appendix A and Figure VURS1).

*constant, declared at [`include/shulib/competition/override_mirror.hpp:56`](../../include/shulib/competition/override_mirror.hpp#L56).*

<a id="toggleimage"></a>

## `toggleImage`

```cpp
[[nodiscard]] constexpr vision::override2026::ToggleId toggleImage(const math::Reflection& r, vision::override2026::ToggleId id) noexcept
```

The Toggle at the image of `id`'s Toggle under `r` — found GEOMETRICALLY from the season's places (vision::override2026::kTogglePlaces), so it cannot be typed wrong; Unset for an Unset id or an image that is no Toggle's place. The identity returns `id`.

*free function, declared at [`include/shulib/competition/override_mirror.hpp:65`](../../include/shulib/competition/override_mirror.hpp#L65).*

<a id="mirroredtoggle"></a>

## `mirroredToggle`

```cpp
[[nodiscard]] inline vision::override2026::ToggleId mirroredToggle(const AutonEntry& entry, vision::override2026::ToggleId id) noexcept
```

THE TOGGLE STEP'S TOGGLE for `entry` (competition/toggle_visit.hpp): `id` as written, or — for a mirrored entry — its image under the entry's FieldMirror (the Toggle is a FIELD fact, whatever the start mode: robot two placed at the image of robot one's spot faces the image Toggle). Its Quadrant colour is the same (the static_asserts below), so "ours" does not change.

*free function, declared at [`include/shulib/competition/override_mirror.hpp:90`](../../include/shulib/competition/override_mirror.hpp#L90).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 38 lines</summary>

```text

 The 2026-27 season's SKILLS MIRROR — VEX V5RC / VEX U "Override" (chunk MIRROR1-A, 2026-11-13; the
 MIRROR1 brief's D1, its §0.3 and its probe P3). Season data: next season adds its own header, and
 this one stays as the record (VIS1's and L6's rule: season data in a season header, cited).

 ── Which mirror, from the game manual (v2.0 read; v2.1's <SG1> and <VURS1>-<VURS3> re-read 2026-11-13) ─
   * In a VEX U skills run BOTH robots start "in legal starting positions for the red Alliance"
     (<VURS3>), and "All other portions of rule <SG1> apply": each robot not sharing a Quadrant with
     another robot (<SG1>d) and on its alliance's side of the Autonomous Line (<SG1>f). The
     Autonomous Line is the double diagonal tape, along F1's y = x; red's side holds exactly two
     Quadrants — the RED wall's and the RIGHT wall's (as seen from the red station). So robot one
     starts in one and robot two in the other.
   * Appendix A's field drawing (A10) makes those two Quadrants mirror images across the diagonal from
     the corner where the red wall meets the right wall to the corner where the blue wall meets the
     left wall — F1's line y = -x: (x, y, heading) -> (-y, -x, -90 deg - heading). Under it all 9 goals
     land on a goal of the same colour, all 4 Toggles on a Toggle of the same Quadrant colour (red wall
     <-> right wall, left wall <-> blue wall), red's side stays red, and the skills setup's Pins
     (Figure VURS1) land on Pins of the same colours. Every other candidate fails on the manual's own
     numbers (the brief's probe P3): the left/right mirror across x = 0 keeps 1 of 9 goal colours, the
     red/blue mirror across y = 0 keeps 1 of 9, the half turn keeps 5 of 9 and turns red's side blue
     (it is the HEAD-TO-HEAD red <-> blue swap — not this), and the other diagonal turns red's side
     blue.

 ── What it does NOT carry (manual facts — told, never refused: the library cannot see intent) ────
   * The LOADERS: all four sit on the red and blue walls (localization/override_walls.hpp); their
     images land on the right and left walls, where there are none (0 of 4). A step at a Loader has no
     mirror — Match Loads go only through red Loaders (<RSC2>d).
   * The LOAD ZONES' colours: the red/left corner's red zone lands on the blue/right corner's blue
     zone (2 of 4 keep their colour). A park in that zone does not mirror.
   * AprilTag NUMBERS: tags 1 <-> 4 and 2 <-> 3 swap (only the centre's 0 keeps its number).
   * ON THE LINE ITSELF — the centre goal (0, 0), two yellow Pins of the skills setup (about (23.5,
     -23.5) and (47, -47) in, Figure VURS1 read by pixel), and the red/right corner's Load Zone with the
     red Loader at (58.76, -70.2) beside it — each is its own image: robot two reaches it at the same
     moment as robot one. ONE robot only (competition::isMirrored(ctx), field_mirror.hpp).
   * Field elements may sit up to +-1.0 in off nominal (<T5>a): a mirrored target can land up to ~2 in
     from robot two's element where robot one's was tuned to its own. A4 register HA-258.

 PROS-free, allocation-free, constexpr data. NOTHING HERE HAS RUN ON A ROBOT.
```

</details>
