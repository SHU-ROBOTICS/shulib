<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/vision/toggle_tracker.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `toggle_tracker.hpp`

ToggleTracker — WHICH COLOUR does the Toggle in front of the robot show: RED, BLUE, YELLOW, or UNKNOWN with the reason.

This header declares **8** types (53 members), **4** free functions, and **2** constants.

Extracted from [`include/shulib/vision/toggle_tracker.hpp`](../../include/shulib/vision/toggle_tracker.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in [How it works](../guide/README.md); worked recipes live in [Recipes](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`enum class ToggleColour`](#enum-class-togglecolour)
  - [`Unknown`](#togglecolour-unknown)
  - [`Red`](#togglecolour-red)
  - [`Blue`](#togglecolour-blue)
  - [`Yellow`](#togglecolour-yellow)
- [`toggleColourWord`](#togglecolourword) — *free function*
- [`enum class ToggleWhy`](#enum-class-togglewhy)
  - [`Sure`](#togglewhy-sure)
  - [`NoRead`](#togglewhy-noread)
  - [`CameraSilent`](#togglewhy-camerasilent)
  - [`ReadNotUsable`](#togglewhy-readnotusable)
  - [`CameraFull`](#togglewhy-camerafull)
  - [`NoBand`](#togglewhy-noband)
  - [`CutByImage`](#togglewhy-cutbyimage)
  - [`MaybeBottomFace`](#togglewhy-maybebottomface)
  - [`TwoColours`](#togglewhy-twocolours)
  - [`NotHeldYet`](#togglewhy-notheldyet)
  - [`CameraFrozen`](#togglewhy-camerafrozen)
  - [`ConflictingPartialBand`](#togglewhy-conflictingpartialband)
  - [`ColourAbove`](#togglewhy-colourabove)
- [`toggleWhyWords`](#togglewhywords) — *free function*
- [`struct ToggleColourIds`](#struct-togglecolourids)
  - [`red`](#togglecolourids-red)
  - [`blue`](#togglecolourids-blue)
  - [`yellow`](#togglecolourids-yellow)
- [`kBottomDepthLoIn`](#kbottomdepthloin) — *constant*
- [`kBottomDepthHiIn`](#kbottomdepthhiin) — *constant*
- [`struct ToggleTrackerConfig`](#struct-toggletrackerconfig)
  - [`ids`](#toggletrackerconfig-ids)
  - [`holdSeconds`](#toggletrackerconfig-holdseconds)
  - [`aliveSeconds`](#toggletrackerconfig-aliveseconds)
  - [`minAspect`](#toggletrackerconfig-minaspect)
  - [`minLengthFraction`](#toggletrackerconfig-minlengthfraction)
  - [`maxLengthFraction`](#toggletrackerconfig-maxlengthfraction)
  - [`minRangeIn`](#toggletrackerconfig-minrangein)
  - [`maxRangeIn`](#toggletrackerconfig-maxrangein)
  - [`bearingWindowDeg`](#toggletrackerconfig-bearingwindowdeg)
  - [`frozenMoveIn`](#toggletrackerconfig-frozenmovein)
  - [`frozenTurnDeg`](#toggletrackerconfig-frozenturndeg)
  - [`positionToleranceIn`](#toggletrackerconfig-positiontolerancein)
  - [`maxPoseToleranceIn`](#toggletrackerconfig-maxposetolerancein)
  - [`maxFixRadiusIn`](#toggletrackerconfig-maxfixradiusin)
- [`toggleTrackerConfigProblem`](#toggletrackerconfigproblem) — *free function*
- [`enum class ToggleFaceGate`](#enum-class-togglefacegate)
  - [`Explained`](#togglefacegate-explained)
  - [`TooThin`](#togglefacegate-toothin)
  - [`Cut`](#togglefacegate-cut)
  - [`NotExplained`](#togglefacegate-notexplained)
- [`struct ToggleFaceMeasure`](#struct-togglefacemeasure)
  - [`gate`](#togglefacemeasure-gate)
  - [`aspect`](#togglefacemeasure-aspect)
  - [`sideClipped`](#togglefacemeasure-sideclipped)
  - [`range`](#togglefacemeasure-range)
  - [`impliedLength`](#togglefacemeasure-impliedlength)
- [`explainAsToggleFace`](#explainastoggleface) — *free function*
- [`struct ToggleView`](#struct-toggleview)
  - [`colour`](#toggleview-colour)
  - [`why`](#toggleview-why)
  - [`heldSeconds`](#toggleview-heldseconds)
  - [`bearing`](#toggleview-bearing)
  - [`range`](#toggleview-range)
  - [`positionChecked`](#toggleview-positionchecked)
  - [`toggleIndex`](#toggleview-toggleindex)
- [`class ToggleTracker`](#class-toggletracker)
  - [`ToggleTracker`](#toggletracker-toggletracker)
  - [`update`](#toggletracker-update)
  - [`view`](#toggletracker-view)

<a id="enum-class-togglecolour"></a>

## `enum class ToggleColour`

```cpp
enum class ToggleColour : std::uint8_t
```

A Toggle face's colour, or UNKNOWN.

*enum class, declared at [`include/shulib/vision/toggle_tracker.hpp:125`](../../include/shulib/vision/toggle_tracker.hpp#L125).*

<a id="togglecolour-unknown"></a>

### `ToggleColour::Unknown`

```cpp
Unknown = 0
```

no colour is sure (the view's `why` says why)

*enumerator, declared at [`include/shulib/vision/toggle_tracker.hpp:126`](../../include/shulib/vision/toggle_tracker.hpp#L126).*

<a id="togglecolour-red"></a>

### `ToggleColour::Red`

```cpp
Red
```

the red face

*enumerator, declared at [`include/shulib/vision/toggle_tracker.hpp:127`](../../include/shulib/vision/toggle_tracker.hpp#L127).*

<a id="togglecolour-blue"></a>

### `ToggleColour::Blue`

```cpp
Blue
```

the blue face

*enumerator, declared at [`include/shulib/vision/toggle_tracker.hpp:128`](../../include/shulib/vision/toggle_tracker.hpp#L128).*

<a id="togglecolour-yellow"></a>

### `ToggleColour::Yellow`

```cpp
Yellow
```

the yellow face

*enumerator, declared at [`include/shulib/vision/toggle_tracker.hpp:129`](../../include/shulib/vision/toggle_tracker.hpp#L129).*

<a id="togglecolourword"></a>

## `toggleColourWord`

```cpp
[[nodiscard]] constexpr const char* toggleColourWord(ToggleColour c) noexcept
```

The colour as one ASCII word for a log line or a bench page: "RED", "BLUE", "YELLOW", "UNKNOWN".

*free function, declared at [`include/shulib/vision/toggle_tracker.hpp:133`](../../include/shulib/vision/toggle_tracker.hpp#L133).*

<a id="enum-class-togglewhy"></a>

## `enum class ToggleWhy`

```cpp
enum class ToggleWhy : std::uint8_t
```

Why the tracker says what it says.

*enum class, declared at [`include/shulib/vision/toggle_tracker.hpp:144`](../../include/shulib/vision/toggle_tracker.hpp#L144).*

<a id="togglewhy-sure"></a>

### `ToggleWhy::Sure`

```cpp
Sure = 0
```

a colour, held for holdSeconds and alive

*enumerator, declared at [`include/shulib/vision/toggle_tracker.hpp:145`](../../include/shulib/vision/toggle_tracker.hpp#L145).*

<a id="togglewhy-noread"></a>

### `ToggleWhy::NoRead`

```cpp
NoRead
```

no usable read yet

*enumerator, declared at [`include/shulib/vision/toggle_tracker.hpp:146`](../../include/shulib/vision/toggle_tracker.hpp#L146).*

<a id="togglewhy-camerasilent"></a>

### `ToggleWhy::CameraSilent`

```cpp
CameraSilent
```

no usable read within aliveSeconds (unplugged, faulting, not configured, or not fed)

*enumerator, declared at [`include/shulib/vision/toggle_tracker.hpp:147`](../../include/shulib/vision/toggle_tracker.hpp#L147).*

<a id="togglewhy-readnotusable"></a>

### `ToggleWhy::ReadNotUsable`

```cpp
ReadNotUsable
```

the latest read was a fault, torn or not configured: the hold restarts

*enumerator, declared at [`include/shulib/vision/toggle_tracker.hpp:148`](../../include/shulib/vision/toggle_tracker.hpp#L148).*

<a id="togglewhy-camerafull"></a>

### `ToggleWhy::CameraFull`

```cpp
CameraFull
```

the latest read was POSSIBLY FULL (24 or more): a contradicting band may be missing

*enumerator, declared at [`include/shulib/vision/toggle_tracker.hpp:149`](../../include/shulib/vision/toggle_tracker.hpp#L149).*

<a id="togglewhy-noband"></a>

### `ToggleWhy::NoBand`

```cpp
NoBand
```

no Toggle-coloured band passed the gates

*enumerator, declared at [`include/shulib/vision/toggle_tracker.hpp:150`](../../include/shulib/vision/toggle_tracker.hpp#L150).*

<a id="togglewhy-cutbyimage"></a>

### `ToggleWhy::CutByImage`

```cpp
CutByImage
```

a Toggle-coloured band touches the image's edge (too close, or a partial band)

*enumerator, declared at [`include/shulib/vision/toggle_tracker.hpp:151`](../../include/shulib/vision/toggle_tracker.hpp#L151).*

<a id="togglewhy-maybebottomface"></a>

### `ToggleWhy::MaybeBottomFace`

```cpp
MaybeBottomFace
```

the only band could be the Toggle's BOTTOM face (too close to see the field face)

*enumerator, declared at [`include/shulib/vision/toggle_tracker.hpp:152`](../../include/shulib/vision/toggle_tracker.hpp#L152).*

<a id="togglewhy-twocolours"></a>

### `ToggleWhy::TwoColours`

```cpp
TwoColours
```

two different colours passed at once

*enumerator, declared at [`include/shulib/vision/toggle_tracker.hpp:153`](../../include/shulib/vision/toggle_tracker.hpp#L153).*

<a id="togglewhy-notheldyet"></a>

### `ToggleWhy::NotHeldYet`

```cpp
NotHeldYet
```

the colour has not held for holdSeconds

*enumerator, declared at [`include/shulib/vision/toggle_tracker.hpp:154`](../../include/shulib/vision/toggle_tracker.hpp#L154).*

<a id="togglewhy-camerafrozen"></a>

### `ToggleWhy::CameraFrozen`

```cpp
CameraFrozen
```

the camera moved or turned and the band's box did not move at all

*enumerator, declared at [`include/shulib/vision/toggle_tracker.hpp:155`](../../include/shulib/vision/toggle_tracker.hpp#L155).*

<a id="togglewhy-conflictingpartialband"></a>

### `ToggleWhy::ConflictingPartialBand`

```cpp
ConflictingPartialBand
```

a band passed beside a band of ANOTHER colour the gates could not judge

*enumerator, declared at [`include/shulib/vision/toggle_tracker.hpp:156`](../../include/shulib/vision/toggle_tracker.hpp#L156).*

<a id="togglewhy-colourabove"></a>

### `ToggleWhy::ColourAbove`

```cpp
ColourAbove
```

a box of ANOTHER Toggle colour sits directly ABOVE the band

*enumerator, declared at [`include/shulib/vision/toggle_tracker.hpp:157`](../../include/shulib/vision/toggle_tracker.hpp#L157).*

<a id="togglewhywords"></a>

## `toggleWhyWords`

```cpp
[[nodiscard]] constexpr const char* toggleWhyWords(ToggleWhy w) noexcept
```

The reason in a few ASCII words, for a log line or a bench page.

*free function, declared at [`include/shulib/vision/toggle_tracker.hpp:161`](../../include/shulib/vision/toggle_tracker.hpp#L161).*

<a id="struct-togglecolourids"></a>

## `struct ToggleColourIds`

```cpp
struct ToggleColourIds
```

The robot's three colour descriptor ids for the Toggle's faces — the colours the team taught the camera (config::CameraConfig's settings). 0 = UNSET.

*struct, declared at [`include/shulib/vision/toggle_tracker.hpp:182`](../../include/shulib/vision/toggle_tracker.hpp#L182).*

<a id="togglecolourids-red"></a>

### `ToggleColourIds::red`

```cpp
std::uint8_t red = 0
```

the red face's descriptor id, 1..7

*field, declared at [`include/shulib/vision/toggle_tracker.hpp:183`](../../include/shulib/vision/toggle_tracker.hpp#L183).*

<a id="togglecolourids-blue"></a>

### `ToggleColourIds::blue`

```cpp
std::uint8_t blue = 0
```

the blue face's descriptor id, 1..7

*field, declared at [`include/shulib/vision/toggle_tracker.hpp:184`](../../include/shulib/vision/toggle_tracker.hpp#L184).*

<a id="togglecolourids-yellow"></a>

### `ToggleColourIds::yellow`

```cpp
std::uint8_t yellow = 0
```

the yellow face's descriptor id, 1..7

*field, declared at [`include/shulib/vision/toggle_tracker.hpp:185`](../../include/shulib/vision/toggle_tracker.hpp#L185).*

<a id="kbottomdepthloin"></a>

## `kBottomDepthLoIn`

```cpp
inline constexpr double kBottomDepthLoIn = 1.0
```

The bottom face's depth, seen as a band on the bottom's plane: its far edge at least this much farther than its near edge, inches (the manual's ~2.05-in face, its rounded corners — generous; A4 register HA-201).

*constant, declared at [`include/shulib/vision/toggle_tracker.hpp:191`](../../include/shulib/vision/toggle_tracker.hpp#L191).*

<a id="kbottomdepthhiin"></a>

## `kBottomDepthHiIn`

```cpp
inline constexpr double kBottomDepthHiIn = 3.0
```

The bottom face's depth: at most this, inches (HA-201).

*constant, declared at [`include/shulib/vision/toggle_tracker.hpp:193`](../../include/shulib/vision/toggle_tracker.hpp#L193).*

<a id="struct-toggletrackerconfig"></a>

## `struct ToggleTrackerConfig`

```cpp
struct ToggleTrackerConfig
```

Every number the tracker uses — the TEAM's, from the bench. Defaults are UNSET (0) and refused.

*struct, declared at [`include/shulib/vision/toggle_tracker.hpp:196`](../../include/shulib/vision/toggle_tracker.hpp#L196).*

<a id="toggletrackerconfig-ids"></a>

### `ToggleTrackerConfig::ids`

```cpp
ToggleColourIds ids{}
```

the three descriptor ids (distinct, each 1..7)

*field, declared at [`include/shulib/vision/toggle_tracker.hpp:197`](../../include/shulib/vision/toggle_tracker.hpp#L197).*

<a id="toggletrackerconfig-holdseconds"></a>

### `ToggleTrackerConfig::holdSeconds`

```cpp
double holdSeconds = 0.0
```

T: the same colour this long, on usable reads, before it is reported

*field, declared at [`include/shulib/vision/toggle_tracker.hpp:198`](../../include/shulib/vision/toggle_tracker.hpp#L198).*

<a id="toggletrackerconfig-aliveseconds"></a>

### `ToggleTrackerConfig::aliveSeconds`

```cpp
double aliveSeconds = 0.0
```

X: a usable read at most this old, or the view is "camera silent"

*field, declared at [`include/shulib/vision/toggle_tracker.hpp:199`](../../include/shulib/vision/toggle_tracker.hpp#L199).*

<a id="toggletrackerconfig-minaspect"></a>

### `ToggleTrackerConfig::minAspect`

```cpp
double minAspect = 0.0
```

a band's width over its height, at least this (more than 1)

*field, declared at [`include/shulib/vision/toggle_tracker.hpp:200`](../../include/shulib/vision/toggle_tracker.hpp#L200).*

<a id="toggletrackerconfig-minlengthfraction"></a>

### `ToggleTrackerConfig::minLengthFraction`

```cpp
double minLengthFraction = 0.0
```

a band not cut at a side: implied length at least this x 25.8 in

*field, declared at [`include/shulib/vision/toggle_tracker.hpp:201`](../../include/shulib/vision/toggle_tracker.hpp#L201).*

<a id="toggletrackerconfig-maxlengthfraction"></a>

### `ToggleTrackerConfig::maxLengthFraction`

```cpp
double maxLengthFraction = 0.0
```

a band not cut at a side: implied length at most this x 25.8 in

*field, declared at [`include/shulib/vision/toggle_tracker.hpp:202`](../../include/shulib/vision/toggle_tracker.hpp#L202).*

<a id="toggletrackerconfig-minrangein"></a>

### `ToggleTrackerConfig::minRangeIn`

```cpp
double minRangeIn = 0.0
```

the range window's near end: lens to the face, horizontal, inches

*field, declared at [`include/shulib/vision/toggle_tracker.hpp:203`](../../include/shulib/vision/toggle_tracker.hpp#L203).*

<a id="toggletrackerconfig-maxrangein"></a>

### `ToggleTrackerConfig::maxRangeIn`

```cpp
double maxRangeIn = 0.0
```

the range window's far end, inches

*field, declared at [`include/shulib/vision/toggle_tracker.hpp:204`](../../include/shulib/vision/toggle_tracker.hpp#L204).*

<a id="toggletrackerconfig-bearingwindowdeg"></a>

### `ToggleTrackerConfig::bearingWindowDeg`

```cpp
double bearingWindowDeg = 0.0
```

a band's whole bearing interval inside +-this, degrees

*field, declared at [`include/shulib/vision/toggle_tracker.hpp:205`](../../include/shulib/vision/toggle_tracker.hpp#L205).*

<a id="toggletrackerconfig-frozenmovein"></a>

### `ToggleTrackerConfig::frozenMoveIn`

```cpp
double frozenMoveIn = 0.0
```

the LENS moved this far and the box did not move: "camera frozen?"

*field, declared at [`include/shulib/vision/toggle_tracker.hpp:206`](../../include/shulib/vision/toggle_tracker.hpp#L206).*

<a id="toggletrackerconfig-frozenturndeg"></a>

### `ToggleTrackerConfig::frozenTurnDeg`

```cpp
double frozenTurnDeg = 0.0
```

the robot TURNED this far and the box did not move: "camera frozen?"

*field, declared at [`include/shulib/vision/toggle_tracker.hpp:207`](../../include/shulib/vision/toggle_tracker.hpp#L207).*

<a id="toggletrackerconfig-positiontolerancein"></a>

### `ToggleTrackerConfig::positionToleranceIn`

```cpp
double positionToleranceIn = 0.0
```

The POSITION gate: a band counts only where a Toggle IS (override2026::kTogglePlaces), within this (plus the band's own fix radius) of the face's line, inches. 0 = the gate is OFF (every view says `positionChecked == false`) — the team turns it on only with a pose it trusts.

*field, declared at [`include/shulib/vision/toggle_tracker.hpp:211`](../../include/shulib/vision/toggle_tracker.hpp#L211).*

<a id="toggletrackerconfig-maxposetolerancein"></a>

### `ToggleTrackerConfig::maxPoseToleranceIn`

```cpp
double maxPoseToleranceIn = 0.0
```

With the position gate on: it is skipped (the view says unchecked) for a read whose pose bound is looser than this, inches.

*field, declared at [`include/shulib/vision/toggle_tracker.hpp:214`](../../include/shulib/vision/toggle_tracker.hpp#L214).*

<a id="toggletrackerconfig-maxfixradiusin"></a>

### `ToggleTrackerConfig::maxFixRadiusIn`

```cpp
double maxFixRadiusIn = 0.0
```

With the position gate on: a band whose field position is vaguer than this (its fix radius) fails it — a vague position is no evidence of WHERE, inches.

*field, declared at [`include/shulib/vision/toggle_tracker.hpp:217`](../../include/shulib/vision/toggle_tracker.hpp#L217).*

<a id="toggletrackerconfigproblem"></a>

## `toggleTrackerConfigProblem`

```cpp
[[nodiscard]] constexpr const char* toggleTrackerConfigProblem(const ToggleTrackerConfig& c) noexcept
```

The first problem with a tracker config, or nullptr. Every number must be finite; the ids set, distinct and 1..7; the hold and alive times, the window's ends, the bearing window and the two frozen thresholds positive; `minAspect` above 1; the length fractions positive and ordered; the range window ordered; the position tolerance not negative — and, with it on, the pose bound it trusts and the vaguest position it accepts set.

*free function, declared at [`include/shulib/vision/toggle_tracker.hpp:225`](../../include/shulib/vision/toggle_tracker.hpp#L225).*

<a id="enum-class-togglefacegate"></a>

## `enum class ToggleFaceGate`

```cpp
enum class ToggleFaceGate : std::uint8_t
```

How far a band got through the library's own gates (2's height rule, 3 and 4 — no team threshold except, in the tracker, the length fractions).

*enum class, declared at [`include/shulib/vision/toggle_tracker.hpp:283`](../../include/shulib/vision/toggle_tracker.hpp#L283).*

<a id="togglefacegate-explained"></a>

### `ToggleFaceGate::Explained`

```cpp
Explained = 0
```

some corner of the tolerance box explains it as a Toggle's field face

*enumerator, declared at [`include/shulib/vision/toggle_tracker.hpp:284`](../../include/shulib/vision/toggle_tracker.hpp#L284).*

<a id="togglefacegate-toothin"></a>

### `ToggleFaceGate::TooThin`

```cpp
TooThin
```

no taller than 2 x kBoxEdgePx (or an empty box): it cannot be measured

*enumerator, declared at [`include/shulib/vision/toggle_tracker.hpp:285`](../../include/shulib/vision/toggle_tracker.hpp#L285).*

<a id="togglefacegate-cut"></a>

### `ToggleFaceGate::Cut`

```cpp
Cut
```

it touches the image's top or bottom, or is cut at a side without spanning the centre

*enumerator, declared at [`include/shulib/vision/toggle_tracker.hpp:286`](../../include/shulib/vision/toggle_tracker.hpp#L286).*

<a id="togglefacegate-notexplained"></a>

### `ToggleFaceGate::NotExplained`

```cpp
NotExplained
```

no corner explains its two edges (and, when judged, its length) as a face

*enumerator, declared at [`include/shulib/vision/toggle_tracker.hpp:287`](../../include/shulib/vision/toggle_tracker.hpp#L287).*

<a id="struct-togglefacemeasure"></a>

## `struct ToggleFaceMeasure`

```cpp
struct ToggleFaceMeasure
```

What the library measures of one colour band as a Toggle's field face — numbers, never a verdict that needs a threshold the team has not typed (Bench Tests prints them beside the camera's raw records).

*struct, declared at [`include/shulib/vision/toggle_tracker.hpp:292`](../../include/shulib/vision/toggle_tracker.hpp#L292).*

<a id="togglefacemeasure-gate"></a>

### `ToggleFaceMeasure::gate`

```cpp
ToggleFaceGate gate = ToggleFaceGate::NotExplained
```

how far it got

*field, declared at [`include/shulib/vision/toggle_tracker.hpp:293`](../../include/shulib/vision/toggle_tracker.hpp#L293).*

<a id="togglefacemeasure-aspect"></a>

### `ToggleFaceMeasure::aspect`

```cpp
double aspect = 0.0
```

width over height (0 for an empty box)

*field, declared at [`include/shulib/vision/toggle_tracker.hpp:294`](../../include/shulib/vision/toggle_tracker.hpp#L294).*

<a id="togglefacemeasure-sideclipped"></a>

### `ToggleFaceMeasure::sideClipped`

```cpp
bool sideClipped = false
```

cut at the image's left or right edge (its length unmeasurable)

*field, declared at [`include/shulib/vision/toggle_tracker.hpp:295`](../../include/shulib/vision/toggle_tracker.hpp#L295).*

<a id="togglefacemeasure-range"></a>

### `ToggleFaceMeasure::range`

```cpp
Interval range{}
```

Explained: the lens's horizontal distance to the face's LOWER EDGE, inches — a bound over EVERY camera of the tolerance box that could explain it, not only the corners that do (invalid otherwise; its far end infinite when a ray at some corner cannot reach its plane and no length bounds it).

*field, declared at [`include/shulib/vision/toggle_tracker.hpp:299`](../../include/shulib/vision/toggle_tracker.hpp#L299).*

<a id="togglefacemeasure-impliedlength"></a>

### `ToggleFaceMeasure::impliedLength`

```cpp
Interval impliedLength{}
```

Explained and not side-clipped: the band's length at those ranges (its width +-2 x kBoxEdgePx over the focal lengths' extremes), inches (the Toggle is 25.8).

*field, declared at [`include/shulib/vision/toggle_tracker.hpp:302`](../../include/shulib/vision/toggle_tracker.hpp#L302).*

<a id="explainastoggleface"></a>

## `explainAsToggleFace`

```cpp
[[nodiscard]] inline ToggleFaceMeasure explainAsToggleFace(const hal::Detection& d, const CameraModel& m) noexcept
```

What the library measures of ONE colour band as a Toggle's field face, with NO team threshold: whether it can be measured, whether the image cuts it, the range at which some corner of the tolerance box explains its two edges as the face's lower edge and apex, and the length that implies — a pure helper for a bench page, which PRINTS what the tracker measures beside the camera's raw record (the tracker itself also judges the length against the team's fractions).

*free function, declared at [`include/shulib/vision/toggle_tracker.hpp:448`](../../include/shulib/vision/toggle_tracker.hpp#L448).*

<a id="struct-toggleview"></a>

## `struct ToggleView`

```cpp
struct ToggleView
```

What the tracker reports.

*struct, declared at [`include/shulib/vision/toggle_tracker.hpp:453`](../../include/shulib/vision/toggle_tracker.hpp#L453).*

<a id="toggleview-colour"></a>

### `ToggleView::colour`

```cpp
ToggleColour colour = ToggleColour::Unknown
```

a colour only when sure

*field, declared at [`include/shulib/vision/toggle_tracker.hpp:454`](../../include/shulib/vision/toggle_tracker.hpp#L454).*

<a id="toggleview-why"></a>

### `ToggleView::why`

```cpp
ToggleWhy why = ToggleWhy::NoRead
```

why

*field, declared at [`include/shulib/vision/toggle_tracker.hpp:455`](../../include/shulib/vision/toggle_tracker.hpp#L455).*

<a id="toggleview-heldseconds"></a>

### `ToggleView::heldSeconds`

```cpp
double heldSeconds = 0.0
```

how long the current colour has held on usable reads, seconds

*field, declared at [`include/shulib/vision/toggle_tracker.hpp:456`](../../include/shulib/vision/toggle_tracker.hpp#L456).*

<a id="toggleview-bearing"></a>

### `ToggleView::bearing`

```cpp
Interval bearing{}
```

The band's bearing interval (body frame, radians, left positive) — the robot's at the IMAGE's time (A4 register HA-192); valid when a band passed.

*field, declared at [`include/shulib/vision/toggle_tracker.hpp:459`](../../include/shulib/vision/toggle_tracker.hpp#L459).*

<a id="toggleview-range"></a>

### `ToggleView::range`

```cpp
Interval range{}
```

The lens's horizontal distance to the face's lower edge, inches — a bound over EVERY camera of the tolerance box that explains the band; its far end infinite when a camera next to the box's corners cannot reach the lower edge's plane and the length does not bound it.

*field, declared at [`include/shulib/vision/toggle_tracker.hpp:463`](../../include/shulib/vision/toggle_tracker.hpp#L463).*

<a id="toggleview-positionchecked"></a>

### `ToggleView::positionChecked`

```cpp
bool positionChecked = false
```

the band was found where a Toggle is (the position gate ran)

*field, declared at [`include/shulib/vision/toggle_tracker.hpp:464`](../../include/shulib/vision/toggle_tracker.hpp#L464).*

<a id="toggleview-toggleindex"></a>

### `ToggleView::toggleIndex`

```cpp
int toggleIndex = -1
```

which of override2026::kTogglePlaces, when checked; -1 otherwise

*field, declared at [`include/shulib/vision/toggle_tracker.hpp:465`](../../include/shulib/vision/toggle_tracker.hpp#L465).*

<a id="class-toggletracker"></a>

## `class ToggleTracker`

```cpp
class ToggleTracker
```

The tracker (header). Feed it every read (update); ask it at any time (view). Single-task.

*class, declared at [`include/shulib/vision/toggle_tracker.hpp:469`](../../include/shulib/vision/toggle_tracker.hpp#L469).*

<a id="toggletracker-toggletracker"></a>

### `ToggleTracker::ToggleTracker`

```cpp
ToggleTracker(const ToggleTrackerConfig& cfg, const CameraModel& model) noexcept
```

PRECONDITION: toggleTrackerConfigProblem(cfg) == nullptr, and `model` built from a mount cameraModelProblem() accepts — a consumer asks both first and refuses to construct, naming them.

*function, declared at [`include/shulib/vision/toggle_tracker.hpp:473`](../../include/shulib/vision/toggle_tracker.hpp#L473).*

<a id="toggletracker-update"></a>

### `ToggleTracker::update`

```cpp
void update(const hal::VisionFrame& f, const PoseBound& robotBound) noexcept
```

One read, seen from `robotBound` — the caller's pose and its stated bound. The pose's MOTION is used for the frozen check (the lens's travel and the robot's turn); the whole bound for the optional position gate.

*function, declared at [`include/shulib/vision/toggle_tracker.hpp:478`](../../include/shulib/vision/toggle_tracker.hpp#L478).*

<a id="toggletracker-view"></a>

### `ToggleTracker::view`

```cpp
[[nodiscard]] ToggleView view(double nowSeconds) const noexcept
```

What it believes at `nowSeconds` (the clock the reads are stamped with).

*function, declared at [`include/shulib/vision/toggle_tracker.hpp:625`](../../include/shulib/vision/toggle_tracker.hpp#L625).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 106 lines, click to expand</summary>

```text

 ToggleTracker — WHICH COLOUR does the Toggle in front of the robot show: RED, BLUE, YELLOW, or UNKNOWN
 with the reason (chunk VIS1-A2, 2026-11-03). It reports a colour ONLY when it is sure: the same band, of
 the same colour, passing every gate, on every usable read for at least `holdSeconds`, with a usable
 read within `aliveSeconds` of now. "The same band": read to read, its direction from the lens (the
 heading plus its bearing) and its range overlap the last read's, widened only by what the lens's travel
 since then explains, and (with the position gate on) it names the same Toggle — another band of the same
 colour taking over restarts the hold. Anything else is UNKNOWN, and UNKNOWN never triggers anything (the
 library's evidence rule: a consumer acts only on a colour). It reports COLOURS, never "ours": which
 colour a robot wants is its alliance's (the consumer's), not this header's.

 ── What a colour MEANS ─────────────────────────────────────────────────────────────
 The colour of the face the camera SEES where a seated Toggle's field face could be, within the stated
 tolerances — NOT the game's "set" (fully seated, a face parallel with its mounts, AND no robot
 touching it; otherwise neutral). A Toggle stuck mid-roll shows its DESTINATION face from below and can
 read as that colour, and the camera's measure of a face's lean is never fine enough to tell a seated
 Toggle from a rolled one at any mount and range VIS1's host camera model tried. So this header exposes
 no "seated" flag and no lean: seating and contact are the consumer's to establish, AT THE PUSH POINT,
 with a sensor that sees colour there (an Optical at the flipper — a distance sensor says something is
 there, never which colour). The camera's colour is the FAR-RANGE cue.

 ── The gates, in order (each turns a lookalike away) ───────────────────────────────
  1. KIND AND ID: a Colour detection with one of the three Toggle descriptor ids; anything else is
     ignored.
  2. SHAPE: a long, flat band — width over height at least `minAspect` (a standing Pin's half is tall
     and thin; a Cup is not a band) — and taller than 2 x kBoxEdgePx (a library rule, not a team
     number: a band within its edges' jitter cannot be measured; without it the bottom face's far-off
     sliver read as a far face).
  3. CUT: a band touching the image's top or bottom is a face leaving the frame — "cut by the image",
     never a colour from a partial face. A band cut at a SIDE has no measurable length, so it is judged
     only when it spans the image's centre column (a real face close in, wider than the picture, does);
     otherwise it is "cut" too (a short lookalike reaching in from the edge — a 4.5-in licence plate,
     a lying Pin — never reaches the length gate it would fail).
  4. ONE CAMERA EXPLAINS IT (camera_geometry.hpp, tested CORNER BY CORNER): the band's bottom row on
     the face's lower edge (an interval, HA-201) and its top row on the apex, at ranges that put the apex
     0.9–1.2 in behind the lower edge (the face LEANS BACK — a vertical plate does not), AND (a band not
     cut at a side) a width that is `minLengthFraction..maxLengthFraction` of the Toggle's 25.8 in at
     that range. Never by overlapping two separately widened intervals: the pitch error is common to
     both edges of one box.
  5. NOT THE BOTTOM FACE: the Toggle's BOTTOM face (a different colour) is visible from a camera below
     it and, close in, mimics the field face. A band some corner explains as the bottom face (both
     edges on the bottom's plane, 1–3 in deep) is kept only when the place its field face would occupy,
     directly above, is certainly in the image AND holds no other Toggle-coloured band; a band there
     makes this one the bottom face (dropped); that place out of frame — UNKNOWN "maybe the bottom face".
     WHAT THIS RESTS ON: an empty place is taken to mean "no field face there", so a read on which the
     field face's descriptor MISSES while its bottom face's fires lets the bottom face through — on the
     host camera model, the field face's detection removed, the bottom face's colour read sure in about one
     parking in ten (VIS1-A2's review); a field face detected but cut short by up to 6 px, never. Refusing
     every band some corner explains as a bottom face would refuse the real face at almost every distance
     from a lens below it (measured: a 6-in lens never read a colour) and STILL let a missed face's bottom
     through (16 of 600 parkings: a bottom face no corner explains as one is judged as a face). So the
     field face's descriptor must fire on every read on which its bottom face's does — A4 register
     HA-199, settled at the bench.
  6. WHERE: its bearing interval wholly inside +-`bearingWindowDeg`, and its range interval OVERLAPPING
     [`minRangeIn`, `maxRangeIn`] — the window drops bands that are certainly elsewhere (another wall's
     Toggle); the view carries both intervals, so a consumer can demand more.
  7. POSITION (optional): with a pose bound tighter than `maxPoseToleranceIn`, the band's field position
     (fix radius at most `maxFixRadiusIn`) must lie on a Toggle's face line (override2026::kTogglePlaces,
     within `positionToleranceIn`); `toggleIndex` says which.
 And, over the whole read:
  * Two DIFFERENT colours passing at once — UNKNOWN (never "pick one"); the same colour twice — the
    nearer band.
  * A passing band beside a band of ANOTHER Toggle colour the gates could not judge (cut by the image, or
    maybe a bottom face) whose bearing touches the window — UNKNOWN: that band may be the real face.
  * A band with a box of ANOTHER Toggle colour directly ABOVE it (columns overlapping; the other's top
    above the band's and its bottom within 2 x kBoxEdgePx of the band's top, or lower; whatever ITS gates
    said) — UNKNOWN. A seated field face has nothing of another Toggle colour directly above it (its
    bottom face is BELOW it); a bottom face does (its field face), and so does a Toggle mid-roll showing
    two faces. Without it, a robot TURNED 15 degrees to a RED Toggle 30 in away read the YELLOW bottom
    face "sure": off the optical axis a horizontal edge projects SLANTED, the field face's box grows
    taller than the face, gate 4 refuses it — and the thin bottom face under it passed. The cost: close
    in, the colour holds only when the robot is SQUARE to the Toggle (VIS1-A2's tests, 8-, 10- and 11-in
    lenses turned in 5-degree steps: at 14 in only square; at 20 in square, an 11-in lens to 5 degrees one
    way; at 30 in from square only (8 in) to 10–15 degrees (11 in); at 40 in 5 (8 in) to 15; at 48 in
    10 (8 in) to 15; never past 15 — the R7 test prints the table). Square up; never remove this to "get
    coverage back".
  * A read reporting 24 or more (`VisionFrame::truncated`, POSSIBLY FULL) — UNKNOWN "camera full": the
    band that contradicts a colour may be among the detections never sent.

 ── Freshness is TIME, and a frozen camera is caught only while the camera MOVES ─────
 PROS gives no frame counter (A4 register HA-192), so a camera that keeps answering with its last frame
 reads exactly like a still scene. While the CAMERA moves — its LENS (the robot's pose through the
 mount, never the robot's centre: a robot turning in place hardly moves its centre) travels more than
 `frozenMoveIn`, OR the robot turns more than `frozenTurnDeg` — a band whose box has not moved by a
 single pixel is UNKNOWN "camera frozen?"; the first read whose box moves again clears it and RESTARTS
 the hold. Parked — exactly where a consumer checks a push — nothing can tell a frozen camera from a
 still scene: that check is never the camera's.

 ── What the tracker cannot do (the host camera model's lookalikes, stated) ─────────
 A colour band the gates cannot tell from a face is reported as one (A4 register HA-199). With the
 position gate off, red bands of the wrong height or length close to a wall pass (VIS1-A2's tests: 70 of
 540 lookalike bands 30 in in front of a wall), and so does a red LED strip on the wall's top seen from
 20 in or closer. The position gate, with a pose the team trusts, turns those away (0 of the 540; the strip
 at every distance) — they are not where a Toggle's face is. It CANNOT turn away a red strip about a
 Toggle's length lying ON a Toggle's face line: a vertical plate there reads RED from about 22 in out with
 the gate off, and with it on from 22 to 30 in (named as that Toggle; farther, its position too vague for
 an 8-in fix). That is the limit a consumer plans for. A band's rows
 are taken as the face's apex and lower edge within kBoxEdgePx (A4 register HA-202: glare on the
 back-leaning face, rounded corners). And the image is OLDER than its read (HA-192): the bearing in a
 view is the robot's at the IMAGE's time — a consumer pairing it with the pose widens the bound
 (vision::boundAtImage).

 Every threshold is the TEAM's, from the bench: none has a library default, and an UNSET config is
 refused (toggleTrackerConfigProblem). PROS-free, sim-free, allocation-free; single-task. NO HARDWARE
 CLAIM: every rule here has run against the host camera model (sim/camera_plant.hpp, written with its
 own projection) and nothing else.
```

</details>
