<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/vision/override_elements.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `override_elements.hpp`

The 2026-27 Override season's facts the camera code needs — the Toggle's coloured faces, where the Toggles are, the Pin and the Cup, and the class table of VEX's Override AI model.

This header declares **6** types (27 members), **5** free functions, and **11** constants.

Extracted from [`include/shulib/vision/override_elements.hpp`](../../include/shulib/vision/override_elements.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`struct ToggleFace`](#struct-toggleface)
  - [`bottomIn`](#toggleface-bottomin)
  - [`bottomHiIn`](#toggleface-bottomhiin)
  - [`setBackLoIn`](#toggleface-setbackloin)
  - [`setBackHiIn`](#toggleface-setbackhiin)
  - [`topIn`](#toggleface-topin)
  - [`flatBottomIn`](#toggleface-flatbottomin)
  - [`lengthIn`](#toggleface-lengthin)
  - [`wallTopIn`](#toggleface-walltopin)
- [`kToggleFace`](#ktoggleface) — *constant*
- [`struct TogglePlace`](#struct-toggleplace)
  - [`x`](#toggleplace-x)
  - [`y`](#toggleplace-y)
  - [`inX`](#toggleplace-inx)
  - [`inY`](#toggleplace-iny)
- [`kTogglePlaces`](#ktoggleplaces) — *constant*
- [`enum class ToggleId`](#enum-class-toggleid)
  - [`Unset`](#toggleid-unset)
  - [`RightWall`](#toggleid-rightwall)
  - [`LeftWall`](#toggleid-leftwall)
  - [`BlueWall`](#toggleid-bluewall)
  - [`RedWall`](#toggleid-redwall)
- [`kRightWall`](#krightwall) — *constant*
- [`kLeftWall`](#kleftwall) — *constant*
- [`kBlueWall`](#kbluewall) — *constant*
- [`kRedWall`](#kredwall) — *constant*
- [`enum class QuadrantColour`](#enum-class-quadrantcolour)
  - [`Unset`](#quadrantcolour-unset)
  - [`Red`](#quadrantcolour-red)
  - [`Blue`](#quadrantcolour-blue)
- [`kToggleQuadrantColour`](#ktogglequadrantcolour) — *constant*
- [`toggleIndex`](#toggleindex) — *free function*
- [`quadrantColour`](#quadrantcolour) — *free function*
- [`toggleName`](#togglename) — *free function*
- [`struct PieceSize`](#struct-piecesize)
  - [`diameterIn`](#piecesize-diameterin)
  - [`heightIn`](#piecesize-heightin)
- [`kPinSize`](#kpinsize) — *constant*
- [`kCupSize`](#kcupsize) — *constant*
- [`enum class ModelClass`](#enum-class-modelclass)
  - [`RedBluePin`](#modelclass-redbluepin)
  - [`BlueYellowPin`](#modelclass-blueyellowpin)
  - [`RedYellowPin`](#modelclass-redyellowpin)
  - [`YellowPin`](#modelclass-yellowpin)
  - [`Cup`](#modelclass-cup)
- [`kModelClassCount`](#kmodelclasscount) — *constant*
- [`kOverrideClassNames`](#koverrideclassnames) — *constant*
- [`modelClassName`](#modelclassname) — *free function*
- [`modelClassSize`](#modelclasssize) — *free function*

<a id="struct-toggleface"></a>

## `struct ToggleFace`

```cpp
struct ToggleFace
```

The Toggle's FIELD-FACING face — the one that counts "when viewed from inside the Field" (the glossary) — as a leaning band, and the flat bottom face under it (header). A4 register HA-201.

*struct, declared at [`include/shulib/vision/override_elements.hpp:37`](../../include/shulib/vision/override_elements.hpp#L37).*

<a id="toggleface-bottomin"></a>

### `ToggleFace::bottomIn`

```cpp
double bottomIn = 12.55
```

The face's lower colour boundary above the floor, LOW end, inches: the arithmetic's 12.61, less the drawing's tolerance and the boundary's spread over the rounded corner (HA-201).

*field, declared at [`include/shulib/vision/override_elements.hpp:40`](../../include/shulib/vision/override_elements.hpp#L40).*

<a id="toggleface-bottomhiin"></a>

### `ToggleFace::bottomHiIn`

```cpp
double bottomHiIn = 12.65
```

The face's lower colour boundary above the floor, HIGH end, inches (HA-201).

*field, declared at [`include/shulib/vision/override_elements.hpp:42`](../../include/shulib/vision/override_elements.hpp#L42).*

<a id="toggleface-setbackloin"></a>

### `ToggleFace::setBackLoIn`

```cpp
double setBackLoIn = 0.9
```

The apex's horizontal set-back behind the lower boundary, at least, inches (the arithmetic's 1.03, less the corner's spread — HA-201).

*field, declared at [`include/shulib/vision/override_elements.hpp:45`](../../include/shulib/vision/override_elements.hpp#L45).*

<a id="toggleface-setbackhiin"></a>

### `ToggleFace::setBackHiIn`

```cpp
double setBackHiIn = 1.2
```

The apex's horizontal set-back behind the lower boundary, at most, inches (HA-201).

*field, declared at [`include/shulib/vision/override_elements.hpp:47`](../../include/shulib/vision/override_elements.hpp#L47).*

<a id="toggleface-topin"></a>

### `ToggleFace::topIn`

```cpp
double topIn = 14.39
```

The face's upper edge (the apex) above the floor, inches: the axle's 13.20 + 30.2 mm (pA8; arithmetic, HA-201).

*field, declared at [`include/shulib/vision/override_elements.hpp:50`](../../include/shulib/vision/override_elements.hpp#L50).*

<a id="toggleface-flatbottomin"></a>

### `ToggleFace::flatBottomIn`

```cpp
double flatBottomIn = 12.36
```

The FLAT bottom face's height above the floor, inches: the apex's 14.39 less pA8's 2.03 — about a quarter inch BELOW the field face's lower boundary (header; HA-201).

*field, declared at [`include/shulib/vision/override_elements.hpp:53`](../../include/shulib/vision/override_elements.hpp#L53).*

<a id="toggleface-lengthin"></a>

### `ToggleFace::lengthIn`

```cpp
double lengthIn = 25.8
```

The Toggle's length, inches: the glossary's "25.8 in (656.2 mm) long" (pA8's 25.99 includes the end caps).

*field, declared at [`include/shulib/vision/override_elements.hpp:56`](../../include/shulib/vision/override_elements.hpp#L56).*

<a id="toggleface-walltopin"></a>

### `ToggleFace::wallTopIn`

```cpp
double wallTopIn = 11.5
```

The perimeter wall's top above the floor, inches: 11.50 (the metal wall, pA14; the portable wall is 11.54, pA13).

*field, declared at [`include/shulib/vision/override_elements.hpp:59`](../../include/shulib/vision/override_elements.hpp#L59).*

<a id="ktoggleface"></a>

## `kToggleFace`

```cpp
inline constexpr ToggleFace kToggleFace{}
```

The Toggle's face geometry, as the manual and the arithmetic above give it (HA-201).

*constant, declared at [`include/shulib/vision/override_elements.hpp:62`](../../include/shulib/vision/override_elements.hpp#L62).*

<a id="struct-toggleplace"></a>

## `struct TogglePlace`

```cpp
struct TogglePlace
```

Where one Toggle is: the wall's inner face at its centre, in F1's field frame (origin at the field's centre), and the unit vector INTO the field. How far the face sits from the wall's inner face horizontally is UNVERIFIED (HA-201) — a consumer's position gate allows for it.

*struct, declared at [`include/shulib/vision/override_elements.hpp:67`](../../include/shulib/vision/override_elements.hpp#L67).*

<a id="toggleplace-x"></a>

### `TogglePlace::x`

```cpp
double x = 0.0
```

the wall's inner face at its centre: field x, inches

*field, declared at [`include/shulib/vision/override_elements.hpp:68`](../../include/shulib/vision/override_elements.hpp#L68).*

<a id="toggleplace-y"></a>

### `TogglePlace::y`

```cpp
double y = 0.0
```

the wall's inner face at its centre: field y, inches

*field, declared at [`include/shulib/vision/override_elements.hpp:69`](../../include/shulib/vision/override_elements.hpp#L69).*

<a id="toggleplace-inx"></a>

### `TogglePlace::inX`

```cpp
double inX = 0.0
```

the unit vector into the field: x

*field, declared at [`include/shulib/vision/override_elements.hpp:70`](../../include/shulib/vision/override_elements.hpp#L70).*

<a id="toggleplace-iny"></a>

### `TogglePlace::inY`

```cpp
double inY = 0.0
```

the unit vector into the field: y

*field, declared at [`include/shulib/vision/override_elements.hpp:71`](../../include/shulib/vision/override_elements.hpp#L71).*

<a id="ktoggleplaces"></a>

## `kTogglePlaces`

```cpp
inline constexpr TogglePlace kTogglePlaces[4] = { {70.2, 0.0, -1.0, 0.0}, {-70.2, 0.0, 1.0, 0.0}, {0.0, 70.2, 0.0, -1.0}, {0.0, -70.2, 0.0, 1.0}}
```

The four Toggles, one centred on each wall: pA10 puts each on its wall's centreline, 70.20 in from the field's centre (the field 140.41 in across).

*constant, declared at [`include/shulib/vision/override_elements.hpp:75`](../../include/shulib/vision/override_elements.hpp#L75).*

<a id="enum-class-toggleid"></a>

## `enum class ToggleId`

```cpp
enum class ToggleId : std::uint8_t
```

WHICH Toggle — in kTogglePlaces' order (index = the id less one). Unset (0) names none.

*enum class, declared at [`include/shulib/vision/override_elements.hpp:92`](../../include/shulib/vision/override_elements.hpp#L92).*

<a id="toggleid-unset"></a>

### `ToggleId::Unset`

```cpp
Unset = 0
```

no Toggle named (refused by every consumer)

*enumerator, declared at [`include/shulib/vision/override_elements.hpp:93`](../../include/shulib/vision/override_elements.hpp#L93).*

<a id="toggleid-rightwall"></a>

### `ToggleId::RightWall`

```cpp
RightWall
```

the right wall's (x = +70.2) — kTogglePlaces[0]

*enumerator, declared at [`include/shulib/vision/override_elements.hpp:94`](../../include/shulib/vision/override_elements.hpp#L94).*

<a id="toggleid-leftwall"></a>

### `ToggleId::LeftWall`

```cpp
LeftWall
```

the left wall's (x = -70.2) — kTogglePlaces[1]

*enumerator, declared at [`include/shulib/vision/override_elements.hpp:95`](../../include/shulib/vision/override_elements.hpp#L95).*

<a id="toggleid-bluewall"></a>

### `ToggleId::BlueWall`

```cpp
BlueWall
```

the blue alliance's wall's (y = +70.2) — kTogglePlaces[2]

*enumerator, declared at [`include/shulib/vision/override_elements.hpp:96`](../../include/shulib/vision/override_elements.hpp#L96).*

<a id="toggleid-redwall"></a>

### `ToggleId::RedWall`

```cpp
RedWall
```

the red alliance's wall's (y = -70.2) — kTogglePlaces[3]

*enumerator, declared at [`include/shulib/vision/override_elements.hpp:97`](../../include/shulib/vision/override_elements.hpp#L97).*

<a id="krightwall"></a>

## `kRightWall`

```cpp
inline constexpr ToggleId kRightWall = ToggleId::RightWall
```

The right wall's Toggle (x = +70.2) — a RED Quadrant's (HA-243).

*constant, declared at [`include/shulib/vision/override_elements.hpp:103`](../../include/shulib/vision/override_elements.hpp#L103).*

<a id="kleftwall"></a>

## `kLeftWall`

```cpp
inline constexpr ToggleId kLeftWall = ToggleId::LeftWall
```

The left wall's Toggle (x = -70.2) — a BLUE Quadrant's (HA-243).

*constant, declared at [`include/shulib/vision/override_elements.hpp:105`](../../include/shulib/vision/override_elements.hpp#L105).*

<a id="kbluewall"></a>

## `kBlueWall`

```cpp
inline constexpr ToggleId kBlueWall = ToggleId::BlueWall
```

The blue alliance's wall's Toggle (y = +70.2) — a BLUE Quadrant's (HA-243).

*constant, declared at [`include/shulib/vision/override_elements.hpp:107`](../../include/shulib/vision/override_elements.hpp#L107).*

<a id="kredwall"></a>

## `kRedWall`

```cpp
inline constexpr ToggleId kRedWall = ToggleId::RedWall
```

The red alliance's wall's Toggle (y = -70.2) — a RED Quadrant's (HA-243).

*constant, declared at [`include/shulib/vision/override_elements.hpp:109`](../../include/shulib/vision/override_elements.hpp#L109).*

<a id="enum-class-quadrantcolour"></a>

## `enum class QuadrantColour`

```cpp
enum class QuadrantColour : std::uint8_t
```

A Quadrant's colour — this header's own (header: the circular include).

*enum class, declared at [`include/shulib/vision/override_elements.hpp:113`](../../include/shulib/vision/override_elements.hpp#L113).*

<a id="quadrantcolour-unset"></a>

### `QuadrantColour::Unset`

```cpp
Unset = 0
```

no colour (an Unset Toggle's)

*enumerator, declared at [`include/shulib/vision/override_elements.hpp:114`](../../include/shulib/vision/override_elements.hpp#L114).*

<a id="quadrantcolour-red"></a>

### `QuadrantColour::Red`

```cpp
Red
```

a red Quadrant

*enumerator, declared at [`include/shulib/vision/override_elements.hpp:115`](../../include/shulib/vision/override_elements.hpp#L115).*

<a id="quadrantcolour-blue"></a>

### `QuadrantColour::Blue`

```cpp
Blue
```

a blue Quadrant

*enumerator, declared at [`include/shulib/vision/override_elements.hpp:116`](../../include/shulib/vision/override_elements.hpp#L116).*

<a id="ktogglequadrantcolour"></a>

## `kToggleQuadrantColour`

```cpp
inline constexpr QuadrantColour kToggleQuadrantColour[4] = {QuadrantColour::Red, QuadrantColour::Blue, QuadrantColour::Blue, QuadrantColour::Red}
```

Each Toggle's Quadrant colour, in kTogglePlaces' order: right RED, left BLUE, blue wall BLUE, red wall RED (header; A4 register HA-243 — reasoned from Figure Q-1 and pA15, unmeasured).

*constant, declared at [`include/shulib/vision/override_elements.hpp:121`](../../include/shulib/vision/override_elements.hpp#L121).*

<a id="toggleindex"></a>

## `toggleIndex`

```cpp
[[nodiscard]] constexpr int toggleIndex(ToggleId id) noexcept
```

The index of `id` in kTogglePlaces (0..3), or -1 for Unset or a byte outside the enum.

*free function, declared at [`include/shulib/vision/override_elements.hpp:125`](../../include/shulib/vision/override_elements.hpp#L125).*

<a id="quadrantcolour"></a>

## `quadrantColour`

```cpp
[[nodiscard]] constexpr QuadrantColour quadrantColour(ToggleId id) noexcept
```

The Quadrant colour of `id`'s Toggle — Unset for Unset or a byte outside the enum.

*free function, declared at [`include/shulib/vision/override_elements.hpp:131`](../../include/shulib/vision/override_elements.hpp#L131).*

<a id="togglename"></a>

## `toggleName`

```cpp
[[nodiscard]] constexpr const char* toggleName(ToggleId id) noexcept
```

The Toggle's name for a log line: "right wall Toggle", "left wall Toggle", "blue wall Toggle", "red wall Toggle"; "unnamed Toggle" for Unset or a byte outside the enum.

*free function, declared at [`include/shulib/vision/override_elements.hpp:138`](../../include/shulib/vision/override_elements.hpp#L138).*

<a id="struct-piecesize"></a>

## `struct PieceSize`

```cpp
struct PieceSize
```

A Scoring Object's size (the glossary).

*struct, declared at [`include/shulib/vision/override_elements.hpp:150`](../../include/shulib/vision/override_elements.hpp#L150).*

<a id="piecesize-diameterin"></a>

### `PieceSize::diameterIn`

```cpp
double diameterIn = 0.0
```

across, inches

*field, declared at [`include/shulib/vision/override_elements.hpp:151`](../../include/shulib/vision/override_elements.hpp#L151).*

<a id="piecesize-heightin"></a>

### `PieceSize::heightIn`

```cpp
double heightIn = 0.0
```

standing, inches

*field, declared at [`include/shulib/vision/override_elements.hpp:152`](../../include/shulib/vision/override_elements.hpp#L152).*

<a id="kpinsize"></a>

## `kPinSize`

```cpp
inline constexpr PieceSize kPinSize{1.6, 6.5}
```

A Pin: "approximately 1.6 in (40 mm) in diameter and 6.5 in (165 mm) tall" (the glossary).

*constant, declared at [`include/shulib/vision/override_elements.hpp:155`](../../include/shulib/vision/override_elements.hpp#L155).*

<a id="kcupsize"></a>

## `kCupSize`

```cpp
inline constexpr PieceSize kCupSize{3.15, 6.5}
```

A Cup: "approximately 3.15 in (80 mm) in diameter and 6.5 in (164.5 mm) tall" (the glossary).

*constant, declared at [`include/shulib/vision/override_elements.hpp:157`](../../include/shulib/vision/override_elements.hpp#L157).*

<a id="enum-class-modelclass"></a>

## `enum class ModelClass`

```cpp
enum class ModelClass : std::uint8_t
```

The Override AI model's classes, as api.vex.com lists them for VEXcode (read 2026-10-15: "Override: 0 redBluePin, 1 blueYellowPin, 2 redYellowPin, 3 yellowPin, 4 cup"). Whether a PROS program sees these names for these ids is A4 register HA-197 — the adapter checks them BY NAME before an id means anything (hal::ModelCheck).

*enum class, declared at [`include/shulib/vision/override_elements.hpp:163`](../../include/shulib/vision/override_elements.hpp#L163).*

<a id="modelclass-redbluepin"></a>

### `ModelClass::RedBluePin`

```cpp
RedBluePin = 0
```

a Pin, half red and half blue

*enumerator, declared at [`include/shulib/vision/override_elements.hpp:164`](../../include/shulib/vision/override_elements.hpp#L164).*

<a id="modelclass-blueyellowpin"></a>

### `ModelClass::BlueYellowPin`

```cpp
BlueYellowPin = 1
```

a Pin, half blue and half yellow

*enumerator, declared at [`include/shulib/vision/override_elements.hpp:165`](../../include/shulib/vision/override_elements.hpp#L165).*

<a id="modelclass-redyellowpin"></a>

### `ModelClass::RedYellowPin`

```cpp
RedYellowPin = 2
```

a Pin, half red and half yellow

*enumerator, declared at [`include/shulib/vision/override_elements.hpp:166`](../../include/shulib/vision/override_elements.hpp#L166).*

<a id="modelclass-yellowpin"></a>

### `ModelClass::YellowPin`

```cpp
YellowPin = 3
```

a Pin, all yellow

*enumerator, declared at [`include/shulib/vision/override_elements.hpp:167`](../../include/shulib/vision/override_elements.hpp#L167).*

<a id="modelclass-cup"></a>

### `ModelClass::Cup`

```cpp
Cup = 4
```

a Cup

*enumerator, declared at [`include/shulib/vision/override_elements.hpp:168`](../../include/shulib/vision/override_elements.hpp#L168).*

<a id="kmodelclasscount"></a>

## `kModelClassCount`

```cpp
inline constexpr std::uint8_t kModelClassCount = 5
```

How many classes the table names.

*constant, declared at [`include/shulib/vision/override_elements.hpp:171`](../../include/shulib/vision/override_elements.hpp#L171).*

<a id="koverrideclassnames"></a>

## `kOverrideClassNames`

```cpp
inline constexpr const char* kOverrideClassNames[kModelClassCount] = { "redBluePin", "blueYellowPin", "redYellowPin", "yellowPin", "cup"}
```

The table a robot file hands the adapter (`hal::AiVisionSettings::expectedClasses`): the camera must name exactly these, in this order, before an Object id is believed (A4 register HA-197). VEX's spelling; the camera's own is read at the bench — a different one fails SAFE (every object withheld, the boot line naming the first name that differed).

*constant, declared at [`include/shulib/vision/override_elements.hpp:177`](../../include/shulib/vision/override_elements.hpp#L177).*

<a id="modelclassname"></a>

## `modelClassName`

```cpp
[[nodiscard]] constexpr const char* modelClassName(std::uint8_t id) noexcept
```

The class's name as VEX spells it (`kOverrideClassNames[id]`), or nullptr for an id outside the table.

*free function, declared at [`include/shulib/vision/override_elements.hpp:181`](../../include/shulib/vision/override_elements.hpp#L181).*

<a id="modelclasssize"></a>

## `modelClassSize`

```cpp
[[nodiscard]] constexpr const PieceSize* modelClassSize(std::uint8_t id) noexcept
```

The class's physical size — a Pin's for ids 0..3, the Cup's for 4 — or nullptr for an id outside the table.

*free function, declared at [`include/shulib/vision/override_elements.hpp:187`](../../include/shulib/vision/override_elements.hpp#L187).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 28 lines</summary>

```text

 The 2026-27 Override season's facts the camera code needs — the Toggle's coloured faces, where the
 Toggles are, the Pin and the Cup, and the class table of VEX's Override AI model (chunk VIS1-A1,
 2026-10-22). DATA ONLY: which Toggle to flip, which piece to take and when are the team's (the
 competition code — the project's guardrail); nothing here decides anything. One header per season:
 next season adds its own, and this one stays as the record.

 THE SOURCE: the VEX V5RC Override Game Manual v2.0 (2026-09-03) — its Appendix A drawings (pA8 "Toggle
 Specifications", pA10 "Field Element Locations", pA13 / pA14 the perimeter walls, pA15 the Toggles'
 start orientation) and its glossary (Pin, Cup, Toggle). v2.1 was scheduled to take effect 2026-10-15;
 its PDF answered HTTP 404 on 2026-10-15, 2026-10-18 and 2026-10-22, so v2.0 is what was read. A manual
 change to any number here is a one-header change with its citation — never a silent edit.

 Every number is cited. Where a number is ARITHMETIC from a drawing rather than a stated figure, it
 says so and names its register entry: the Toggle's face geometry is A4 register HA-201 (reasoned).

 ── The Toggle's cross-section, read from pA8's end view ────────────────────────────
 A rounded triangle on an axle 13.20 in above the floor: 51.6 mm (2.03 in) from the apex to the FLAT
 bottom face, the axle 30.2 mm (1.19 in) below the apex, the corners heavily rounded; three coloured
 faces (pA15). By the shape's 120-degree symmetry each corner's middle — where one colour meets the next
 — lies 30.2 mm from the axle, so two colour boundaries are 30.2 x sqrt 3 = 52.3 mm apart: the
 glossary's "each face of the triangle is approximately 2.05 in (52.2 mm) wide". So the field-facing
 face runs from the apex (13.20 + 1.19 = 14.39 in up, over the axle) down to its lower boundary 15.1 mm
 (0.594 in) below the axle — 12.61 in up — and 26.2 mm (1.03 in) toward the field: it LEANS BACK. The
 FLAT bottom face is lower still, 21.4 mm (0.84 in) below the axle — 12.36 in up — joined to the field
 face's lower boundary by the rounded corner. (VIS1's brief, before its re-check, read 12.36 as the
 field face's lower edge; the re-check took 12.61 from the glossary; the drawing, read at VIS1-A1,
 shows both hold — 12.36 is the bottom face, not the field face's edge.)
```

</details>
