<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/localization/tag_map.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `tag_map.hpp`

TagMap — where the AprilTags are on the field, and where each of those numbers CAME FROM.

This header declares **3** types (15 members).

Extracted from [`include/shulib/localization/tag_map.hpp`](../../include/shulib/localization/tag_map.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`enum class TagProvenance`](#enum-class-tagprovenance)
  - [`Unspecified`](#tagprovenance-unspecified)
  - [`Specified`](#tagprovenance-specified)
  - [`Measured`](#tagprovenance-measured)
  - [`Invented`](#tagprovenance-invented)
- [`struct TagPlacement`](#struct-tagplacement)
  - [`id`](#tagplacement-id)
  - [`fieldPose`](#tagplacement-fieldpose)
  - [`provenance`](#tagplacement-provenance)
  - [`source`](#tagplacement-source)
- [`class TagMap`](#class-tagmap)
  - [`kMaxTags`](#tagmap-kmaxtags)
  - [`add`](#tagmap-add)
  - [`find`](#tagmap-find)
  - [`size`](#tagmap-size)
  - [`empty`](#tagmap-empty)
  - [`anyInvented`](#tagmap-anyinvented)
  - [`robotPoseFromTag`](#tagmap-robotposefromtag)

<a id="enum-class-tagprovenance"></a>

## `enum class TagProvenance`

```cpp
enum class TagProvenance : std::uint8_t
```

Where a tag's field pose came from. There is no default: see the header note.

*enum class, declared at [`include/shulib/localization/tag_map.hpp:60`](../../include/shulib/localization/tag_map.hpp#L60).*

<a id="tagprovenance-unspecified"></a>

### `TagProvenance::Unspecified`

```cpp
Unspecified = 0
```

NOT ACCEPTED by add() — the whole point of the type

*enumerator, declared at [`include/shulib/localization/tag_map.hpp:61`](../../include/shulib/localization/tag_map.hpp#L61).*

<a id="tagprovenance-specified"></a>

### `TagProvenance::Specified`

```cpp
Specified = 1
```

from a published field/game specification; `source` cites it

*enumerator, declared at [`include/shulib/localization/tag_map.hpp:62`](../../include/shulib/localization/tag_map.hpp#L62).*

<a id="tagprovenance-measured"></a>

### `TagProvenance::Measured`

```cpp
Measured = 2
```

measured on the actual competition field; `source` says how

*enumerator, declared at [`include/shulib/localization/tag_map.hpp:63`](../../include/shulib/localization/tag_map.hpp#L63).*

<a id="tagprovenance-invented"></a>

### `TagProvenance::Invented`

```cpp
Invented = 3
```

a number somebody made up; `source` says why. Legitimate, if labeled.

*enumerator, declared at [`include/shulib/localization/tag_map.hpp:64`](../../include/shulib/localization/tag_map.hpp#L64).*

<a id="struct-tagplacement"></a>

## `struct TagPlacement`

```cpp
struct TagPlacement
```

One tag's placement on the field, with its provenance attached inseparably.

*struct, declared at [`include/shulib/localization/tag_map.hpp:68`](../../include/shulib/localization/tag_map.hpp#L68).*

<a id="tagplacement-id"></a>

### `TagPlacement::id`

```cpp
int id = -1
```

the detector's AprilTag id; must be ≥ 0, so the -1 default is unaddable

*field, declared at [`include/shulib/localization/tag_map.hpp:69`](../../include/shulib/localization/tag_map.hpp#L69).*

<a id="tagplacement-fieldpose"></a>

### `TagPlacement::fieldPose`

```cpp
math::Pose2d fieldPose{}
```

Field pose; heading = the direction the tag's outward normal points (header note).

*field, declared at [`include/shulib/localization/tag_map.hpp:71`](../../include/shulib/localization/tag_map.hpp#L71).*

<a id="tagplacement-provenance"></a>

### `TagPlacement::provenance`

```cpp
TagProvenance provenance = TagProvenance::Unspecified
```

Where `fieldPose` came from. Unspecified — the default — is the one value add() refuses: "nobody said" is exactly the answer this type exists to make impossible.

*field, declared at [`include/shulib/localization/tag_map.hpp:74`](../../include/shulib/localization/tag_map.hpp#L74).*

<a id="tagplacement-source"></a>

### `TagPlacement::source`

```cpp
const char* source = nullptr
```

The citation, the measurement method, or the reason this is a guess. Must be non-empty. A static string literal: this type stores the pointer, it does not own the text.

*field, declared at [`include/shulib/localization/tag_map.hpp:77`](../../include/shulib/localization/tag_map.hpp#L77).*

<a id="class-tagmap"></a>

## `class TagMap`

```cpp
class TagMap
```

A fixed-capacity id → field-pose table, plus the one rigid-body inversion that turns a tag sighting into a robot pose. It starts EMPTY and ships empty deliberately — no published VEX AprilTag layout exists to seed it with, and an invented default would localize every team that forgot to override it against fiction (header note). Add-only, allocation-free, clock-free and HAL-free: build it once at setup, then only read it.

*class, declared at [`include/shulib/localization/tag_map.hpp:85`](../../include/shulib/localization/tag_map.hpp#L85).*

<a id="tagmap-kmaxtags"></a>

### `TagMap::kMaxTags`

```cpp
static constexpr std::size_t kMaxTags = 16
```

Enough for a VEX field's worth of tags with room to spare. Fixed so the lookup on the control path never allocates and never rehashes.

*field, declared at [`include/shulib/localization/tag_map.hpp:89`](../../include/shulib/localization/tag_map.hpp#L89).*

<a id="tagmap-add"></a>

### `TagMap::add`

```cpp
void add(const TagPlacement& placement)
```

Register a tag. Refuses, loudly and at setup time (never mid-match), an entry with no provenance, no source text, a negative id, a non-finite pose, or a duplicate id — a duplicate is the mistake most likely to survive review, because the second entry simply never wins a lookup and the map still "works".

*function, declared at [`include/shulib/localization/tag_map.hpp:95`](../../include/shulib/localization/tag_map.hpp#L95).*

<a id="tagmap-find"></a>

### `TagMap::find`

```cpp
[[nodiscard]] const TagPlacement* find(int id) const noexcept
```

The placement for `id`, or nullptr if this map does not know that tag. Linear over at most kMaxTags entries: no allocation, no branching on data the caller cannot see.

*function, declared at [`include/shulib/localization/tag_map.hpp:111`](../../include/shulib/localization/tag_map.hpp#L111).*

<a id="tagmap-size"></a>

### `TagMap::size`

```cpp
[[nodiscard]] std::size_t size() const noexcept
```

How many placements are registered, 0..kMaxTags. There is no remove and no clear, so this only ever grows.

*function, declared at [`include/shulib/localization/tag_map.hpp:122`](../../include/shulib/localization/tag_map.hpp#L122).*

<a id="tagmap-empty"></a>

### `TagMap::empty`

```cpp
[[nodiscard]] bool empty() const noexcept
```

True until the first add() — and the state a build given no field layout stays in for the whole match, which is why an empty map makes the tag corrector decline every sighting instead of quietly correcting against a guess.

*function, declared at [`include/shulib/localization/tag_map.hpp:127`](../../include/shulib/localization/tag_map.hpp#L127).*

<a id="tagmap-anyinvented"></a>

### `TagMap::anyInvented`

```cpp
[[nodiscard]] bool anyInvented() const noexcept
```

True if ANY registered tag pose is an invented number. A run anchored to invented field geometry must not read the same as one anchored to a measured field (header note).

*function, declared at [`include/shulib/localization/tag_map.hpp:131`](../../include/shulib/localization/tag_map.hpp#L131).*

<a id="tagmap-robotposefromtag"></a>

### `TagMap::robotPoseFromTag`

```cpp
[[nodiscard]] static math::Pose2d robotPoseFromTag(const math::Pose2d& tagField, const math::Pose2d& tagInRobot)
```

THE INVERSION. Given where a tag IS on the field and where it appears RELATIVE to the robot, where must the robot be?  The forward composition is `tagField = robot ∘ tagInRobot`: Tx = Rx + rx·cos(Rθ) − ry·sin(Rθ) Ty = Ry + rx·sin(Rθ) + ry·cos(Rθ) Tθ = Rθ + rθ so, solving for the robot: Rθ = Tθ − rθ                     (wrap-correct; math::Angle owns that) Rx = Tx − (rx·cos(Rθ) − ry·sin(Rθ)) Ry = Ty − (rx·sin(Rθ) + ry·cos(Rθ))  Note the ORDER: the heading must be solved FIRST, because the position term is rotated by the ROBOT's heading, not the tag's. Those two coincide exactly when rθ == 0, so a suite that only ever tested a tag "facing the same way as the robot" could not tell them apart — which is why every case in tag_map_test.cpp uses a non-zero relative heading, and none uses the origin or heading 0.  Static and pure: it is the tag map's arithmetic, not the corrector's, so it can be tested (and mutated) without constructing a corrector at all.

*function, declared at [`include/shulib/localization/tag_map.hpp:160`](../../include/shulib/localization/tag_map.hpp#L160).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 45 lines</summary>

```text

 TagMap — where the AprilTags are on the field, and where each of those numbers CAME FROM
 (master plan §8; WS5, chunk E3, tension T2).

 ── WHY THIS TYPE EXISTS AT ALL ─────────────────────────────────────────────────────────────
 A tag observation says "there is a tag with id 7 at relative pose P". That is useless on its
 own. It becomes an absolute robot pose only against knowledge of where tag 7 IS — and that
 knowledge is INPUT, not something the library can derive, measure, or reasonably guess.

 It is also the single most dangerous input in the whole localization stack, for a reason worth
 stating in full: sensor noise averages out and a fusion filter is built to absorb it, but a
 WRONG TAG POSE DOES NOT AVERAGE OUT. A map entry two inches off produces a corrector that is
 confidently two inches wrong, every time it sees that tag, with a small residual and a high
 confidence — i.e. it looks exactly like a healthy fix. It will also fight the GPS corrector,
 and the fusion policy has no way to tell which of the two is lying. There is no gate width
 that fixes this and no amount of filtering that reveals it.

 ── WHAT SHULIB DOES NOT SHIP, AND WHY ──────────────────────────────────────────────────────
 **There is NO built-in VEX field tag map in this library, and adding one would be a mistake
 until somebody can cite a published table.** Nobody on this project has a game-manual table of
 AprilTag field poses in hand; a plausible-looking default map would be invented geometry
 wearing the clothes of a specification, and every team that forgot to override it would be
 silently localizing against fiction. So the map is empty until a caller fills it, an empty map
 makes the corrector decline with `RejectedNoTagMapEntry`, and that is a loud, diagnosable
 state rather than a quiet wrong one. Obtaining the real layout is R3's job (A4: HA-68).

 ── PROVENANCE IS MANDATORY, BY CONSTRUCTION ────────────────────────────────────────────────
 `add()` REFUSES an entry that does not say where its numbers came from. Not a convention, not
 a lint: a precondition. `TagProvenance::Invented` is a completely legitimate answer — a guess
 LABELED as a guess is exactly what the A4 register exists to protect (see its own preamble) —
 but "I did not say" is not, because the difference between a specified pose and an invented
 one is invisible in the arithmetic and total in the consequences.

 `anyInvented()` exists so a run can be honest about it in telemetry: an estimator anchored to
 made-up field geometry should not read the same as one anchored to a measured field.

 ── THE FRAME ───────────────────────────────────────────────────────────────────────────────
 `fieldPose` is in the canonical field frame (F1: +X, +Y, CCW-positive), and its HEADING is the
 direction the tag's OUTWARD NORMAL points — the direction it "faces", i.e. toward a robot
 looking at it. A tag flat on the wall at x = 70 that a robot at the origin can read is facing
 180 degrees. This matches hal/vision_conversion.hpp's reduction exactly, on purpose: that
 function reports `poseInRobot.heading()` with the same meaning, so the two compose without a
 convention change in between. (A convention change in between is how the sign errors get in.)

 Fixed capacity, no allocation, no clock, no HAL: pure data plus one rigid-body inversion.
```

</details>
