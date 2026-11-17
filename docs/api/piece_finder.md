<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/vision/piece_finder.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `piece_finder.hpp`

PieceFinder — "take THAT piece": a route names ONE game piece by its TYPE (the AI model's class) and its EXPECTED FIELD POSITION within a TOLERANCE, and the finder answers FOUND, NONE, AMBIGUOUS, NOT IN VIEW or UNKNOWN — NEVER A GUESS.

This header declares **8** types (54 members), **7** free functions, and **1** constant.

Extracted from [`include/shulib/vision/piece_finder.hpp`](../../include/shulib/vision/piece_finder.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in [How it works](../guide/README.md); worked recipes live in [Recipes](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`enum class Posture`](#enum-class-posture)
  - [`Unsure`](#posture-unsure)
  - [`OnFloor`](#posture-onfloor)
  - [`Raised`](#posture-raised)
  - [`Lying`](#posture-lying)
- [`postureWord`](#postureword) — *free function*
- [`topFaceAllowanceIn`](#topfaceallowancein) — *free function*
- [`struct PieceSighting`](#struct-piecesighting)
  - [`classId`](#piecesighting-classid)
  - [`posture`](#piecesighting-posture)
  - [`bearing`](#piecesighting-bearing)
  - [`groundRange`](#piecesighting-groundrange)
  - [`sizeRange`](#piecesighting-sizerange)
  - [`fix`](#piecesighting-fix)
  - [`nearerThanIn`](#piecesighting-nearerthanin)
  - [`box`](#piecesighting-box)
- [`struct PieceTarget`](#struct-piecetarget)
  - [`classMask`](#piecetarget-classmask)
  - [`x`](#piecetarget-x)
  - [`y`](#piecetarget-y)
  - [`toleranceIn`](#piecetarget-tolerancein)
  - [`want`](#piecetarget-want)
- [`enum class PieceVerdict`](#enum-class-pieceverdict)
  - [`Unknown`](#pieceverdict-unknown)
  - [`Found`](#pieceverdict-found)
  - [`None`](#pieceverdict-none)
  - [`Ambiguous`](#pieceverdict-ambiguous)
  - [`NotInView`](#pieceverdict-notinview)
- [`pieceVerdictWords`](#pieceverdictwords) — *free function*
- [`enum class PieceWhy`](#enum-class-piecewhy)
  - [`Ok`](#piecewhy-ok)
  - [`NoRead`](#piecewhy-noread)
  - [`CameraSilent`](#piecewhy-camerasilent)
  - [`NotHeldYet`](#piecewhy-notheldyet)
  - [`TwoInside`](#piecewhy-twoinside)
  - [`OnTheBorder`](#piecewhy-ontheborder)
  - [`ErrorLargerThanTolerance`](#piecewhy-errorlargerthantolerance)
  - [`WrongPostureInside`](#piecewhy-wrongpostureinside)
  - [`OutOfView`](#piecewhy-outofview)
  - [`NothingThere`](#piecewhy-nothingthere)
  - [`CameraFull`](#piecewhy-camerafull)
  - [`UnmeasurableInside`](#piecewhy-unmeasurableinside)
  - [`ModelNotConfirmed`](#piecewhy-modelnotconfirmed)
  - [`ReadNotUsable`](#piecewhy-readnotusable)
  - [`NoTarget`](#piecewhy-notarget)
- [`pieceWhyWords`](#piecewhywords) — *free function*
- [`struct PieceFinderConfig`](#struct-piecefinderconfig)
  - [`holdSeconds`](#piecefinderconfig-holdseconds)
  - [`aliveSeconds`](#piecefinderconfig-aliveseconds)
  - [`minScore`](#piecefinderconfig-minscore)
  - [`lyingAspect`](#piecefinderconfig-lyingaspect)
  - [`standingAspect`](#piecefinderconfig-standingaspect)
  - [`viewMarginPx`](#piecefinderconfig-viewmarginpx)
  - [`raisedMinIn`](#piecefinderconfig-raisedminin)
- [`pieceFinderConfigProblem`](#piecefinderconfigproblem) — *free function*
- [`pieceTargetProblem`](#piecetargetproblem) — *free function*
- [`kPostureSlackIn`](#kpostureslackin) — *constant*
- [`sightPiece`](#sightpiece) — *free function*
- [`class PieceFinder`](#class-piecefinder)
  - [`kMax`](#piecefinder-kmax)
  - [`PieceFinder`](#piecefinder-piecefinder)
  - [`setTarget`](#piecefinder-settarget)
  - [`update`](#piecefinder-update)
  - [`answer`](#piecefinder-answer)
  - [`struct PieceFinder::Result`](#struct-piecefinder-result)
    - [`verdict`](#piecefinder-result-verdict)
    - [`why`](#piecefinder-result-why)
    - [`sighting`](#piecefinder-result-sighting)
    - [`inside`](#piecefinder-result-inside)
    - [`border`](#piecefinder-result-border)

<a id="enum-class-posture"></a>

## `enum class Posture`

```cpp
enum class Posture : std::uint8_t
```

How a piece stands.

*enum class, declared at [`include/shulib/vision/piece_finder.hpp:110`](../../include/shulib/vision/piece_finder.hpp#L110).*

<a id="posture-unsure"></a>

### `Posture::Unsure`

```cpp
Unsure = 0
```

cut by the image, merged with another, maybe lying toward the lens, or neither on the floor nor raised for certain

*enumerator, declared at [`include/shulib/vision/piece_finder.hpp:111`](../../include/shulib/vision/piece_finder.hpp#L111).*

<a id="posture-onfloor"></a>

### `Posture::OnFloor`

```cpp
OnFloor
```

standing on the floor: some camera puts its base, height and width at one range; no camera sees it raised or lying

*enumerator, declared at [`include/shulib/vision/piece_finder.hpp:112`](../../include/shulib/vision/piece_finder.hpp#L112).*

<a id="posture-raised"></a>

### `Posture::Raised`

```cpp
Raised
```

standing with its base above the floor (on a Goal, in a Cup, on a stack): no camera explains it on the floor

*enumerator, declared at [`include/shulib/vision/piece_finder.hpp:113`](../../include/shulib/vision/piece_finder.hpp#L113).*

<a id="posture-lying"></a>

### `Posture::Lying`

```cpp
Lying
```

on its side: its box wider than `lyingAspect`

*enumerator, declared at [`include/shulib/vision/piece_finder.hpp:114`](../../include/shulib/vision/piece_finder.hpp#L114).*

<a id="postureword"></a>

## `postureWord`

```cpp
[[nodiscard]] constexpr const char* postureWord(Posture p) noexcept
```

The posture as one ASCII word: "UNSURE", "ON-FLOOR", "RAISED", "LYING".

*free function, declared at [`include/shulib/vision/piece_finder.hpp:118`](../../include/shulib/vision/piece_finder.hpp#L118).*

<a id="topfaceallowancein"></a>

## `topFaceAllowanceIn`

```cpp
[[nodiscard]] constexpr double topFaceAllowanceIn(double heightIn, double diameterIn, double lensHeightIn) noexcept
```

How much NEARER than its near rim a standing piece's size range can read, inches: its box takes in the top face's far rim when the lens is above its top. For a piece of `heightIn` and `diameterIn` under a lens `lensHeightIn` high it is at most 2r(a - H)/H (r the radius, a the lens height, H the height — the supremum over every distance; 0 when the lens is not above the top).

*free function, declared at [`include/shulib/vision/piece_finder.hpp:132`](../../include/shulib/vision/piece_finder.hpp#L132).*

<a id="struct-piecesighting"></a>

## `struct PieceSighting`

```cpp
struct PieceSighting
```

One Object detection, interpreted (sightPiece()).

*struct, declared at [`include/shulib/vision/piece_finder.hpp:140`](../../include/shulib/vision/piece_finder.hpp#L140).*

<a id="piecesighting-classid"></a>

### `PieceSighting::classId`

```cpp
std::uint8_t classId = 0
```

the model's class (meaningful only once the model's names match)

*field, declared at [`include/shulib/vision/piece_finder.hpp:141`](../../include/shulib/vision/piece_finder.hpp#L141).*

<a id="piecesighting-posture"></a>

### `PieceSighting::posture`

```cpp
Posture posture = Posture::Unsure
```

how it stands

*field, declared at [`include/shulib/vision/piece_finder.hpp:142`](../../include/shulib/vision/piece_finder.hpp#L142).*

<a id="piecesighting-bearing"></a>

### `PieceSighting::bearing`

```cpp
Interval bearing{}
```

body frame, radians, left positive — at the IMAGE's time

*field, declared at [`include/shulib/vision/piece_finder.hpp:143`](../../include/shulib/vision/piece_finder.hpp#L143).*

<a id="piecesighting-groundrange"></a>

### `PieceSighting::groundRange`

```cpp
Interval groundRange{}
```

The lens's horizontal distance to where the box's bottom row meets the floor, inches (valid only below the horizon).

*field, declared at [`include/shulib/vision/piece_finder.hpp:146`](../../include/shulib/vision/piece_finder.hpp#L146).*

<a id="piecesighting-sizerange"></a>

### `PieceSighting::sizeRange`

```cpp
Interval sizeRange{}
```

The lens's horizontal distance to an upright thing of the class's height filling the box, inches (unclipped boxes only).

*field, declared at [`include/shulib/vision/piece_finder.hpp:149`](../../include/shulib/vision/piece_finder.hpp#L149).*

<a id="piecesighting-fix"></a>

### `PieceSighting::fix`

```cpp
FieldFix fix{}
```

where its CENTRE is (OnFloor, Raised, Lying); invalid otherwise

*field, declared at [`include/shulib/vision/piece_finder.hpp:150`](../../include/shulib/vision/piece_finder.hpp#L150).*

<a id="piecesighting-nearerthanin"></a>

### `PieceSighting::nearerThanIn`

```cpp
double nearerThanIn = 0.0
```

A box cut ONLY by the image's BOTTOM edge cannot be measured, but its base is nearer than the floor seen at the image's bottom row: the lens's horizontal distance to its centre is under this, inches (0 = no bound).

*field, declared at [`include/shulib/vision/piece_finder.hpp:154`](../../include/shulib/vision/piece_finder.hpp#L154).*

<a id="piecesighting-box"></a>

### `PieceSighting::box`

```cpp
hal::Detection box{}
```

the detection as read

*field, declared at [`include/shulib/vision/piece_finder.hpp:155`](../../include/shulib/vision/piece_finder.hpp#L155).*

<a id="struct-piecetarget"></a>

## `struct PieceTarget`

```cpp
struct PieceTarget
```

What a route asks for. Every field is the ROUTE's (the students'): none is a library default.

*struct, declared at [`include/shulib/vision/piece_finder.hpp:159`](../../include/shulib/vision/piece_finder.hpp#L159).*

<a id="piecetarget-classmask"></a>

### `PieceTarget::classMask`

```cpp
std::uint32_t classMask = 0
```

bit i = the model's class i is acceptable; 0 = UNSET

*field, declared at [`include/shulib/vision/piece_finder.hpp:160`](../../include/shulib/vision/piece_finder.hpp#L160).*

<a id="piecetarget-x"></a>

### `PieceTarget::x`

```cpp
double x = 0.0
```

the expected field position's x (F1), inches

*field, declared at [`include/shulib/vision/piece_finder.hpp:161`](../../include/shulib/vision/piece_finder.hpp#L161).*

<a id="piecetarget-y"></a>

### `PieceTarget::y`

```cpp
double y = 0.0
```

the expected field position's y (F1), inches

*field, declared at [`include/shulib/vision/piece_finder.hpp:162`](../../include/shulib/vision/piece_finder.hpp#L162).*

<a id="piecetarget-tolerancein"></a>

### `PieceTarget::toleranceIn`

```cpp
double toleranceIn = 0.0
```

how far from (x, y) it may be, inches; 0 = UNSET

*field, declared at [`include/shulib/vision/piece_finder.hpp:163`](../../include/shulib/vision/piece_finder.hpp#L163).*

<a id="piecetarget-want"></a>

### `PieceTarget::want`

```cpp
Posture want = Posture::OnFloor
```

the posture the route takes (never Unsure)

*field, declared at [`include/shulib/vision/piece_finder.hpp:164`](../../include/shulib/vision/piece_finder.hpp#L164).*

<a id="enum-class-pieceverdict"></a>

## `enum class PieceVerdict`

```cpp
enum class PieceVerdict : std::uint8_t
```

The verdict (header).

*enum class, declared at [`include/shulib/vision/piece_finder.hpp:168`](../../include/shulib/vision/piece_finder.hpp#L168).*

<a id="pieceverdict-unknown"></a>

### `PieceVerdict::Unknown`

```cpp
Unknown = 0
```

no target, no fresh usable read, the latest read not usable, FOUND not held yet, the camera full, or the model not confirmed

*enumerator, declared at [`include/shulib/vision/piece_finder.hpp:169`](../../include/shulib/vision/piece_finder.hpp#L169).*

<a id="pieceverdict-found"></a>

### `PieceVerdict::Found`

```cpp
Found
```

exactly one candidate certainly inside, none on the border — held

*enumerator, declared at [`include/shulib/vision/piece_finder.hpp:170`](../../include/shulib/vision/piece_finder.hpp#L170).*

<a id="pieceverdict-none"></a>

### `PieceVerdict::None`

```cpp
None
```

nothing of the type and posture inside or on the border

*enumerator, declared at [`include/shulib/vision/piece_finder.hpp:171`](../../include/shulib/vision/piece_finder.hpp#L171).*

<a id="pieceverdict-ambiguous"></a>

### `PieceVerdict::Ambiguous`

```cpp
Ambiguous
```

two inside, one on the border, one unmeasurable, or its error larger than the tolerance

*enumerator, declared at [`include/shulib/vision/piece_finder.hpp:172`](../../include/shulib/vision/piece_finder.hpp#L172).*

<a id="pieceverdict-notinview"></a>

### `PieceVerdict::NotInView`

```cpp
NotInView
```

the expected place is not certainly inside the picture

*enumerator, declared at [`include/shulib/vision/piece_finder.hpp:173`](../../include/shulib/vision/piece_finder.hpp#L173).*

<a id="pieceverdictwords"></a>

## `pieceVerdictWords`

```cpp
[[nodiscard]] constexpr const char* pieceVerdictWords(PieceVerdict v) noexcept
```

The verdict as ASCII words: "UNKNOWN", "FOUND", "NONE", "AMBIGUOUS", "NOT IN VIEW".

*free function, declared at [`include/shulib/vision/piece_finder.hpp:177`](../../include/shulib/vision/piece_finder.hpp#L177).*

<a id="enum-class-piecewhy"></a>

## `enum class PieceWhy`

```cpp
enum class PieceWhy : std::uint8_t
```

Why — the detail behind the verdict.

*enum class, declared at [`include/shulib/vision/piece_finder.hpp:189`](../../include/shulib/vision/piece_finder.hpp#L189).*

<a id="piecewhy-ok"></a>

### `PieceWhy::Ok`

```cpp
Ok = 0
```

FOUND

*enumerator, declared at [`include/shulib/vision/piece_finder.hpp:190`](../../include/shulib/vision/piece_finder.hpp#L190).*

<a id="piecewhy-noread"></a>

### `PieceWhy::NoRead`

```cpp
NoRead
```

no usable read yet

*enumerator, declared at [`include/shulib/vision/piece_finder.hpp:191`](../../include/shulib/vision/piece_finder.hpp#L191).*

<a id="piecewhy-camerasilent"></a>

### `PieceWhy::CameraSilent`

```cpp
CameraSilent
```

no usable read within aliveSeconds

*enumerator, declared at [`include/shulib/vision/piece_finder.hpp:192`](../../include/shulib/vision/piece_finder.hpp#L192).*

<a id="piecewhy-notheldyet"></a>

### `PieceWhy::NotHeldYet`

```cpp
NotHeldYet
```

FOUND, but not yet for holdSeconds

*enumerator, declared at [`include/shulib/vision/piece_finder.hpp:193`](../../include/shulib/vision/piece_finder.hpp#L193).*

<a id="piecewhy-twoinside"></a>

### `PieceWhy::TwoInside`

```cpp
TwoInside
```

two candidates certainly inside

*enumerator, declared at [`include/shulib/vision/piece_finder.hpp:194`](../../include/shulib/vision/piece_finder.hpp#L194).*

<a id="piecewhy-ontheborder"></a>

### `PieceWhy::OnTheBorder`

```cpp
OnTheBorder
```

a candidate could be inside and could be outside

*enumerator, declared at [`include/shulib/vision/piece_finder.hpp:195`](../../include/shulib/vision/piece_finder.hpp#L195).*

<a id="piecewhy-errorlargerthantolerance"></a>

### `PieceWhy::ErrorLargerThanTolerance`

```cpp
ErrorLargerThanTolerance
```

the candidates' own error is wider than the tolerance: drive closer, or measure the mount tighter

*enumerator, declared at [`include/shulib/vision/piece_finder.hpp:196`](../../include/shulib/vision/piece_finder.hpp#L196).*

<a id="piecewhy-wrongpostureinside"></a>

### `PieceWhy::WrongPostureInside`

```cpp
WrongPostureInside
```

a candidate of the type stands there in another posture

*enumerator, declared at [`include/shulib/vision/piece_finder.hpp:197`](../../include/shulib/vision/piece_finder.hpp#L197).*

<a id="piecewhy-outofview"></a>

### `PieceWhy::OutOfView`

```cpp
OutOfView
```

the expected place is not certainly in the picture

*enumerator, declared at [`include/shulib/vision/piece_finder.hpp:198`](../../include/shulib/vision/piece_finder.hpp#L198).*

<a id="piecewhy-nothingthere"></a>

### `PieceWhy::NothingThere`

```cpp
NothingThere
```

nothing of the type there

*enumerator, declared at [`include/shulib/vision/piece_finder.hpp:199`](../../include/shulib/vision/piece_finder.hpp#L199).*

<a id="piecewhy-camerafull"></a>

### `PieceWhy::CameraFull`

```cpp
CameraFull
```

the read was POSSIBLY FULL: a piece may be among the detections never sent

*enumerator, declared at [`include/shulib/vision/piece_finder.hpp:200`](../../include/shulib/vision/piece_finder.hpp#L200).*

<a id="piecewhy-unmeasurableinside"></a>

### `PieceWhy::UnmeasurableInside`

```cpp
UnmeasurableInside
```

a candidate of the type it cannot measure could be inside

*enumerator, declared at [`include/shulib/vision/piece_finder.hpp:201`](../../include/shulib/vision/piece_finder.hpp#L201).*

<a id="piecewhy-modelnotconfirmed"></a>

### `PieceWhy::ModelNotConfirmed`

```cpp
ModelNotConfirmed
```

the camera's AI model is not confirmed BY NAME: its ids mean nothing

*enumerator, declared at [`include/shulib/vision/piece_finder.hpp:202`](../../include/shulib/vision/piece_finder.hpp#L202).*

<a id="piecewhy-readnotusable"></a>

### `PieceWhy::ReadNotUsable`

```cpp
ReadNotUsable
```

the latest read was a fault, torn or not configured: no evidence either way

*enumerator, declared at [`include/shulib/vision/piece_finder.hpp:203`](../../include/shulib/vision/piece_finder.hpp#L203).*

<a id="piecewhy-notarget"></a>

### `PieceWhy::NoTarget`

```cpp
NoTarget
```

no target set (or one pieceTargetProblem() refuses): it knows of no piece to look for

*enumerator, declared at [`include/shulib/vision/piece_finder.hpp:204`](../../include/shulib/vision/piece_finder.hpp#L204).*

<a id="piecewhywords"></a>

## `pieceWhyWords`

```cpp
[[nodiscard]] constexpr const char* pieceWhyWords(PieceWhy w) noexcept
```

The reason in a few ASCII words.

*free function, declared at [`include/shulib/vision/piece_finder.hpp:208`](../../include/shulib/vision/piece_finder.hpp#L208).*

<a id="struct-piecefinderconfig"></a>

## `struct PieceFinderConfig`

```cpp
struct PieceFinderConfig
```

Every number the finder uses that is not the route's — the TEAM's, from the bench. Defaults are UNSET and refused: 0 for the numbers that must be positive, -1 for the two whose 0 means something (`minScore`'s "every score", `viewMarginPx`'s "no margin").

*struct, declared at [`include/shulib/vision/piece_finder.hpp:232`](../../include/shulib/vision/piece_finder.hpp#L232).*

<a id="piecefinderconfig-holdseconds"></a>

### `PieceFinderConfig::holdSeconds`

```cpp
double holdSeconds = 0.0
```

FOUND must hold this long (the same candidate), seconds

*field, declared at [`include/shulib/vision/piece_finder.hpp:233`](../../include/shulib/vision/piece_finder.hpp#L233).*

<a id="piecefinderconfig-aliveseconds"></a>

### `PieceFinderConfig::aliveSeconds`

```cpp
double aliveSeconds = 0.0
```

a usable read at most this old, seconds

*field, declared at [`include/shulib/vision/piece_finder.hpp:234`](../../include/shulib/vision/piece_finder.hpp#L234).*

<a id="piecefinderconfig-minscore"></a>

### `PieceFinderConfig::minScore`

```cpp
double minScore = -1.0
```

an Object scored below this is ignored (the raw scale: A4 register HA-198); 0 = every score; -1 = UNSET

*field, declared at [`include/shulib/vision/piece_finder.hpp:235`](../../include/shulib/vision/piece_finder.hpp#L235).*

<a id="piecefinderconfig-lyingaspect"></a>

### `PieceFinderConfig::lyingAspect`

```cpp
double lyingAspect = 0.0
```

a box wider over tall than this is LYING

*field, declared at [`include/shulib/vision/piece_finder.hpp:236`](../../include/shulib/vision/piece_finder.hpp#L236).*

<a id="piecefinderconfig-standingaspect"></a>

### `PieceFinderConfig::standingAspect`

```cpp
double standingAspect = 0.0
```

a box narrower over tall than this may stand (the posture rules decide; between the two: UNSURE)

*field, declared at [`include/shulib/vision/piece_finder.hpp:237`](../../include/shulib/vision/piece_finder.hpp#L237).*

<a id="piecefinderconfig-viewmarginpx"></a>

### `PieceFinderConfig::viewMarginPx`

```cpp
double viewMarginPx = -1.0
```

the expected place must be at least this far inside the picture, pixels; 0 = none; -1 = UNSET

*field, declared at [`include/shulib/vision/piece_finder.hpp:238`](../../include/shulib/vision/piece_finder.hpp#L238).*

<a id="piecefinderconfig-raisedminin"></a>

### `PieceFinderConfig::raisedMinIn`

```cpp
double raisedMinIn = 0.0
```

The LOWEST a raised piece's base sits above the floor on this field (a Goal's top, a stack), inches — the team's, read from the game manual's field drawings (a fact of the field, not of the bench): a standing piece is ON THE FLOOR only when no camera of the tolerance box could see it raised this much or more. A smaller number refuses more (UNSURE); a larger one than the field's lowest raise lets a raised piece read as a floor one.

*field, declared at [`include/shulib/vision/piece_finder.hpp:244`](../../include/shulib/vision/piece_finder.hpp#L244).*

<a id="piecefinderconfigproblem"></a>

## `pieceFinderConfigProblem`

```cpp
[[nodiscard]] constexpr const char* pieceFinderConfigProblem(const PieceFinderConfig& c) noexcept
```

The first problem with a finder config, or nullptr: a number that is not finite; the hold or alive time UNSET; the standing aspect UNSET or the lying one not above it; the score floor or the view margin UNSET (negative); the lowest raise UNSET.

*free function, declared at [`include/shulib/vision/piece_finder.hpp:250`](../../include/shulib/vision/piece_finder.hpp#L250).*

<a id="piecetargetproblem"></a>

## `pieceTargetProblem`

```cpp
[[nodiscard]] constexpr const char* pieceTargetProblem(const PieceTarget& t) noexcept
```

The first problem with a route's target, or nullptr: no class named, or one the season's model does not name; a position or tolerance that is not finite; the tolerance UNSET; an UNSURE posture asked for.

*free function, declared at [`include/shulib/vision/piece_finder.hpp:281`](../../include/shulib/vision/piece_finder.hpp#L281).*

<a id="kpostureslackin"></a>

## `kPostureSlackIn`

```cpp
inline constexpr double kPostureSlackIn = 0.25
```

The slack, inches, the finder allows between the ground and the size ranges of ONE camera for what the model leaves out (a box's corners off the axis's vertical plane).

*constant, declared at [`include/shulib/vision/piece_finder.hpp:302`](../../include/shulib/vision/piece_finder.hpp#L302).*

<a id="sightpiece"></a>

## `sightPiece`

```cpp
[[nodiscard]] inline PieceSighting sightPiece(const hal::Detection& d, const CameraModel& m, const PoseBound& robot, const PieceFinderConfig& cfg) noexcept
```

Interpret one Object detection, seen from `robot` (the caller's pose and its stated bound) — pure (header: the posture rules, the fix).

*free function, declared at [`include/shulib/vision/piece_finder.hpp:359`](../../include/shulib/vision/piece_finder.hpp#L359).*

<a id="class-piecefinder"></a>

## `class PieceFinder`

```cpp
class PieceFinder
```

The finder (header): set the route's target, feed it reads, ask it at any time. Single-task.

*class, declared at [`include/shulib/vision/piece_finder.hpp:553`](../../include/shulib/vision/piece_finder.hpp#L553).*

<a id="piecefinder-kmax"></a>

### `PieceFinder::kMax`

```cpp
static constexpr std::size_t kMax = hal::kMaxVisionDetections
```

The most sightings one read holds (the camera's 24).

*field, declared at [`include/shulib/vision/piece_finder.hpp:556`](../../include/shulib/vision/piece_finder.hpp#L556).*

<a id="piecefinder-piecefinder"></a>

### `PieceFinder::PieceFinder`

```cpp
PieceFinder(const PieceFinderConfig& cfg, const CameraModel& model) noexcept
```

PRECONDITION: pieceFinderConfigProblem(cfg) == nullptr, and `model` built from a mount cameraModelProblem() accepts — a consumer asks both first and refuses to construct, naming them.

*function, declared at [`include/shulib/vision/piece_finder.hpp:569`](../../include/shulib/vision/piece_finder.hpp#L569).*

<a id="piecefinder-settarget"></a>

### `PieceFinder::setTarget`

```cpp
void setTarget(const PieceTarget& t) noexcept
```

The route's target (pieceTargetProblem(t) must be nullptr: one it refuses is no target — every answer UNKNOWN "no target" until a valid one is set). A new target restarts the hold.

*function, declared at [`include/shulib/vision/piece_finder.hpp:573`](../../include/shulib/vision/piece_finder.hpp#L573).*

<a id="piecefinder-update"></a>

### `PieceFinder::update`

```cpp
void update(const hal::VisionFrame& f, const PoseBound& robot) noexcept
```

One read, seen from `robot` — the caller's pose and its stated bound, AT THE IMAGE's time (header).

*function, declared at [`include/shulib/vision/piece_finder.hpp:580`](../../include/shulib/vision/piece_finder.hpp#L580).*

<a id="piecefinder-answer"></a>

### `PieceFinder::answer`

```cpp
[[nodiscard]] Result answer(double nowSeconds) const noexcept
```

The answer at `nowSeconds` (the clock the reads are stamped with).

*function, declared at [`include/shulib/vision/piece_finder.hpp:624`](../../include/shulib/vision/piece_finder.hpp#L624).*

<a id="struct-piecefinder-result"></a>

## `struct PieceFinder::Result`

```cpp
struct Result
```

The answer at a moment.

*struct, declared at [`include/shulib/vision/piece_finder.hpp:559`](../../include/shulib/vision/piece_finder.hpp#L559).*

<a id="piecefinder-result-verdict"></a>

### `PieceFinder::Result::verdict`

```cpp
PieceVerdict verdict = PieceVerdict::Unknown
```

the verdict

*field, declared at [`include/shulib/vision/piece_finder.hpp:560`](../../include/shulib/vision/piece_finder.hpp#L560).*

<a id="piecefinder-result-why"></a>

### `PieceFinder::Result::why`

```cpp
PieceWhy why = PieceWhy::NoRead
```

the detail behind it

*field, declared at [`include/shulib/vision/piece_finder.hpp:561`](../../include/shulib/vision/piece_finder.hpp#L561).*

<a id="piecefinder-result-sighting"></a>

### `PieceFinder::Result::sighting`

```cpp
PieceSighting sighting{}
```

the one FOUND (meaningful only then)

*field, declared at [`include/shulib/vision/piece_finder.hpp:562`](../../include/shulib/vision/piece_finder.hpp#L562).*

<a id="piecefinder-result-inside"></a>

### `PieceFinder::Result::inside`

```cpp
int inside = 0
```

candidates certainly inside the tolerance

*field, declared at [`include/shulib/vision/piece_finder.hpp:563`](../../include/shulib/vision/piece_finder.hpp#L563).*

<a id="piecefinder-result-border"></a>

### `PieceFinder::Result::border`

```cpp
int border = 0
```

candidates on its border, or unmeasurable and maybe inside

*field, declared at [`include/shulib/vision/piece_finder.hpp:564`](../../include/shulib/vision/piece_finder.hpp#L564).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 92 lines, click to expand</summary>

```text

 PieceFinder — "take THAT piece": a route names ONE game piece by its TYPE (the AI model's class) and its
 EXPECTED FIELD POSITION within a TOLERANCE, and the finder answers FOUND, NONE, AMBIGUOUS, NOT IN VIEW
 or UNKNOWN — NEVER A GUESS (chunk VIS1-A2, 2026-11-03). It never returns the nearest of several, and
 never one it is not sure is inside the tolerance: "not touch one vs touch another in a route" (the team
 lead, 2026-10-14) is decided by the ROUTE's expected position, and the finder says only whether the
 piece the route named is there.

 ── The five verdicts ───────────────────────────────────────────────────────────────
  * FOUND — exactly one candidate of the named type and posture CERTAINLY inside the tolerance (its whole
    error disc inside the tolerance disc), none on the border, the same one held for `holdSeconds`.
  * NONE — nothing of that type and posture inside or on the border (it may be gone, or hidden; "wrong
    posture inside" when a candidate of the type stood there in another posture).
  * AMBIGUOUS — two inside, one on the border, one of the type it cannot measure whose BEARING could be
    inside, or its own error larger than the tolerance ("drive closer, or measure the mount tighter":
    with a mount measured to +-1 degree, two Pins a foot apart are AMBIGUOUS from about 4 ft, by design).
  * NOT IN VIEW — the expected place (its tolerance disc, at the floor AND at a Pin's height) is not
    certainly inside the picture, for every camera of the tolerance box. Tested on the eight corners of
    an octagon drawn AROUND the disc: the picture's certain region on a plane is convex (the image's
    four edges and "in front of the lens", each a half-plane, for each camera), so those eight inside
    puts the whole disc inside — a few points ON the disc would not.
  * UNKNOWN — no target set (or one pieceTargetProblem() refuses: nothing named is nothing known missing),
    no usable read, none within `aliveSeconds`, the LATEST read not usable (a fault breaks the
    picture: it is never "nothing there"), FOUND not held yet, the read POSSIBLY FULL (24 or more — a
    piece may be among the detections never sent), or the camera's AI model not confirmed BY NAME (A4
    register HA-197: an Object id means nothing until the names match, and a read whose Objects were
    withheld is never evidence that nothing is there).

 ── Where a piece is, and how it stands (the box: A4 register HA-200) ───────────────
 A candidate's BEARING comes from its box's centre; its RANGE from the row where its box's bottom meets
 the floor (the GROUND range — the piece's near rim, its centre one radius farther). Its POSTURE:
  * a box cut by the image's edge is UNSURE (a box cut ONLY by the bottom edge is bounded: its base is
    nearer than the floor seen at the picture's bottom row — `nearerThanIn`);
  * wider than `lyingAspect` it is LYING (its range from its near edge, widened by its length);
  * narrower than `standingAspect` it stands — ON THE FLOOR when SOME camera of the tolerance box makes
    the ground range of its box bottom agree with the range its known height implies (the SIZE range, one
    camera at a time: the pitch error is common to both) AND NO camera of the box could see it raised
    `raisedMinIn` or more (the team's number: the lowest a raised piece sits on this field — a Goal's
    top, a stack). Agreeing with the floor is not enough on its own: far out, or from a high lens, a Pin
    on a Goal agrees with the floor through a camera at the far edge of the tolerance box, and its floor
    fix then misses it by more than its radius (VIS1-A2's tests: 73 of 2,720 such sightings from a 10-in
    lens, 639 of 2,772 from an 18-in one) — so the raise is excluded first, shown from the whole tolerance
    box's EXACT intervals: a raise of z0 puts the bottom row's floor point a / (a - z0) times farther than
    the near rim, for a lens a high;
    AND the box is a Pin STANDING, not one LYING toward the lens (VIS1-A2's review: a Pin lying near
    end-on has a box as tall and narrow as a standing one's; it read ON THE FLOOR with a fix far from it —
    so a box narrower than `standingAspect` is never taken as upright on its aspect alone): its WIDTH agrees,
    on the same camera, with the piece's diameter at the range that camera agreed on (a Pin lying end-on
    on a Goal's top fails this: its box far too wide for a Pin that far out), and NO camera of the box could
    see it lying on the floor — a lying piece's highest point is at most a diameter up and at most its
    length's diagonal beyond its nearest point, so the box's top row would meet the plane a diameter up no
    farther than the ground range plus that diagonal (tested camera by camera at the box's corners: for one
    camera the two rows' depressions differ by a pitch-free angle, so the pitch's extremes decide whenever
    the diagonal squared is at least D(a - D) for a lens a high — a Pin under any lens below 29 in; above
    that, the whole box's extremes);
  * RAISED (on a Goal, in a Cup, on a stack) when NO camera can put it on the floor — its box bottom too
    high for its size —, also shown from the whole box's exact intervals (camera_geometry.hpp: the corners
    are not every camera, so "none" is never concluded from the corners alone);
  * anything else is UNSURE — a Pin the stated tolerances cannot place on the floor or above it, or one that
    could be lying toward the lens (close in, from a high lens, a standing Pin's box and a Pin lying end-on
    look alike: UNSURE there is the honest answer).
 The size range of a real Pin reads NEARER than its near rim when the lens is above the Pin's top (its
 box takes in the top face's far rim — camera_geometry.hpp's sizeRange()): by at most 2r(a - H)/H for a
 Pin of radius r and height H under a lens a high (in the camera's own depth; off the axis, divided by
 the cosine of the bearing), which the agreement allows for. A FIX is computed only for a posture the
 evidence decides: ON THE FLOOR from both its ground range and its size range (each a bound on the near
 rim once the floor is certain — the intersection), RAISED from its size range alone (only when every
 camera's box bottom looks down, so the lens is above its base), LYING from its ground range.
 The route says which posture it wants; a candidate of the type in another posture is never taken.

 ── What the fix is a bound on ──────────────────────────────────────────────────────
 A field position with a radius (camera_geometry.hpp's fieldFix()): the pose the CALLER states, plus the
 camera's intervals. The pose must be the robot's at the IMAGE's time (A4 register HA-192): a robot that
 moves between the image and the read is off by its speed times the image's age, so a consumer passes
 vision::boundAtImage() with the MEASURED age — and while the age is unmeasured, takes a piece only with
 the robot stopped.
 KNOWN GAP (VIS1-A2's open finding A2-F1): off the camera's axis a standing piece's TOP is nearer the lens,
 so its cross-section is the widest in pixels and sets the box's centre column — the piece's axis at the
 box's TOP row. sightPiece() takes the bearing at the box's MIDDLE row and the ground range at its BOTTOM
 row, where that column points a little farther out: both can sit just outside the true piece. On the
 host plant: a Pin 31 degrees off the axis, 11 in out, a camera measured to 0.01 in / 0.01 degree and its
 box's bottom edge a pixel low — the fix missed it by 0.02 in past its radius. With the stated
 tolerances of a real mount the radius absorbs it (no miss in ~26,000 sightings with the host plant's box
 edges jittered by 1 px; at its full 2 px plus the integer rounding a box edge can sit up to ~3 px off, past
 kBoxEdgePx, and standing floor Pins' fixes then missed by up to 1.8 in — A4 register HA-202's bound is on
 the REPORTED edge). The fix (the bearing over the box's rows, the floor range from the row's depth) is not
 built yet: no program uses the finder before drive-to-piece.

 Every number the finder uses is the TEAM's (PieceFinderConfig, from the bench) or the ROUTE's
 (PieceTarget, the students'): none has a library default, and an UNSET one is refused. PROS-free,
 sim-free, allocation-free; single-task. NO HARDWARE CLAIM: every rule here has run against the host
 camera model (sim/camera_plant.hpp) and nothing else.
```

</details>
