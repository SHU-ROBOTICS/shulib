<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/motion/curve_path.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `curve_path.hpp`

curve_path.hpp — the PURE path of the curve verbs (chunk CURVE1, 2026-11): the smooth path a curveTo / reverseCurveTo drives, built from the robot's live pose and the author's points, its speed plan, and the refusals and warnings a path earns before the rob…

This header declares **6** types (55 members), **2** free functions, and **2** constants.

Extracted from [`include/shulib/motion/curve_path.hpp`](../../include/shulib/motion/curve_path.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`kMaxCurvePoints`](#kmaxcurvepoints) — *constant*
- [`kCurveMinSpacing`](#kcurveminspacing) — *constant*
- [`struct CurvePoint`](#struct-curvepoint)
  - [`x`](#curvepoint-x)
  - [`y`](#curvepoint-y)
  - [`heading`](#curvepoint-heading)
  - [`hasHeading`](#curvepoint-hasheading)
  - [`CurvePoint`](#curvepoint-curvepoint)
  - [`CurvePoint (overload 2)`](#curvepoint-curvepoint-2)
- [`enum class CurveVerdict`](#enum-class-curveverdict)
  - [`Ok`](#curveverdict-ok)
  - [`Invalid`](#curveverdict-invalid)
  - [`StartBehind`](#curveverdict-startbehind)
  - [`TightBend`](#curveverdict-tightbend)
- [`enum class CurveInvalid`](#enum-class-curveinvalid)
  - [`None`](#curveinvalid-none)
  - [`NoPoints`](#curveinvalid-nopoints)
  - [`AtRobot`](#curveinvalid-atrobot)
  - [`TooClose`](#curveinvalid-tooclose)
  - [`NoDirection`](#curveinvalid-nodirection)
  - [`DoublesBack`](#curveinvalid-doublesback)
- [`struct CurveCheck`](#struct-curvecheck)
  - [`verdict`](#curvecheck-verdict)
  - [`why`](#curvecheck-why)
  - [`point`](#curvecheck-point)
  - [`distance`](#curvecheck-distance)
  - [`angle`](#curvecheck-angle)
  - [`tightestRadius`](#curvecheck-tightestradius)
  - [`reverse`](#curvecheck-reverse)
- [`struct CurveSample`](#struct-curvesample)
  - [`x`](#curvesample-x)
  - [`y`](#curvesample-y)
  - [`direction`](#curvesample-direction)
  - [`heading`](#curvesample-heading)
  - [`curvature`](#curvesample-curvature)
- [`class CurvePath`](#class-curvepath)
  - [`CurvePath`](#curvepath-curvepath)
  - [`build`](#curvepath-build)
  - [`built`](#curvepath-built)
  - [`reverse`](#curvepath-reverse)
  - [`authorPoints`](#curvepath-authorpoints)
  - [`segments`](#curvepath-segments)
  - [`droppedFirstDistance`](#curvepath-droppedfirstdistance)
  - [`firstDrivenPoint`](#curvepath-firstdrivenpoint)
  - [`length`](#curvepath-length)
  - [`knotS`](#curvepath-knots)
  - [`at`](#curvepath-at)
  - [`endPose`](#curvepath-endpose)
  - [`lastPoint`](#curvepath-lastpoint)
  - [`project`](#curvepath-project)
  - [`lookAheadPoint`](#curvepath-lookaheadpoint)
  - [`tightestRadius`](#curvepath-tightestradius)
  - [`tightestAt`](#curvepath-tightestat)
  - [`startAngleOff`](#curvepath-startangleoff)
  - [`speedLimitAt`](#curvepath-speedlimitat)
  - [`kSamples`](#curvepath-ksamples)
  - [`sampleCount`](#curvepath-samplecount)
  - [`sampleS`](#curvepath-samples)
  - [`sampleSpeed`](#curvepath-samplespeed)
  - [`invalidFacts`](#curvepath-invalidfacts)
  - [`kScanStep`](#curvepath-kscanstep)
  - [`kBendAhead`](#curvepath-kbendahead)
  - [`kPlanStartSpeed`](#curvepath-kplanstartspeed)
- [`checkCurve`](#checkcurve) — *free function*
- [`appendCurveCheckWords`](#appendcurvecheckwords) — *free function*

<a id="kmaxcurvepoints"></a>

## `kMaxCurvePoints`

```cpp
inline constexpr std::size_t kMaxCurvePoints = 16
```

The most points one curve takes (a longer route is two curves). A design constant — the path's fixed capacity, which keeps a curve on the auton task's stack — not a hardware belief.

*constant, declared at [`include/shulib/motion/curve_path.hpp:86`](../../include/shulib/motion/curve_path.hpp#L86).*

<a id="kcurveminspacing"></a>

## `kCurveMinSpacing`

```cpp
inline constexpr double kCurveMinSpacing = 1.0
```

The closest two knots may be (in): twice the translation settle's 0.5 in (HA-51). A headingless first point closer than this to the robot is dropped, a headed one refused; any other two consecutive knots closer than this are refused. A design constant.

*constant, declared at [`include/shulib/motion/curve_path.hpp:91`](../../include/shulib/motion/curve_path.hpp#L91).*

<a id="struct-curvepoint"></a>

## `struct CurvePoint`

```cpp
struct CurvePoint
```

One point the curve passes through, in FIELD inches: `{24_in, 12_in}` (the library picks the smooth direction) or `{48_in, 24_in, 0_deg}` (pass here with the robot's NOSE pointing this way — also on reverseCurveTo). The two constructors are the whole surface (F6-frozen as the curve verbs' parameter type); there is deliberately no default constructor — it would mean "a point at the field's origin", which nobody writes on purpose.

*struct, declared at [`include/shulib/motion/curve_path.hpp:98`](../../include/shulib/motion/curve_path.hpp#L98).*

<a id="curvepoint-x"></a>

### `CurvePoint::x`

```cpp
units::Length x
```

field x (in)

*field, declared at [`include/shulib/motion/curve_path.hpp:99`](../../include/shulib/motion/curve_path.hpp#L99).*

<a id="curvepoint-y"></a>

### `CurvePoint::y`

```cpp
units::Length y
```

field y (in)

*field, declared at [`include/shulib/motion/curve_path.hpp:100`](../../include/shulib/motion/curve_path.hpp#L100).*

<a id="curvepoint-heading"></a>

### `CurvePoint::heading`

```cpp
math::Angle heading
```

the robot's NOSE heading here; meaningful only if hasHeading

*field, declared at [`include/shulib/motion/curve_path.hpp:101`](../../include/shulib/motion/curve_path.hpp#L101).*

<a id="curvepoint-hasheading"></a>

### `CurvePoint::hasHeading`

```cpp
bool hasHeading
```

false: the library picks the smooth direction

*field, declared at [`include/shulib/motion/curve_path.hpp:102`](../../include/shulib/motion/curve_path.hpp#L102).*

<a id="curvepoint-curvepoint"></a>

### `CurvePoint::CurvePoint`

```cpp
CurvePoint(units::Length px, units::Length py) noexcept
```

A point without a heading.

*function, declared at [`include/shulib/motion/curve_path.hpp:105`](../../include/shulib/motion/curve_path.hpp#L105).*

<a id="curvepoint-curvepoint-2"></a>

### `CurvePoint::CurvePoint (overload 2)`

```cpp
CurvePoint(units::Length px, units::Length py, math::Angle h) noexcept
```

A point with the robot's nose heading `h` there.

*function, declared at [`include/shulib/motion/curve_path.hpp:108`](../../include/shulib/motion/curve_path.hpp#L108).*

<a id="enum-class-curveverdict"></a>

## `enum class CurveVerdict`

```cpp
enum class CurveVerdict : std::uint8_t
```

checkCurve()'s verdict, in the order it is decided.

*enum class, declared at [`include/shulib/motion/curve_path.hpp:113`](../../include/shulib/motion/curve_path.hpp#L113).*

<a id="curveverdict-ok"></a>

### `CurveVerdict::Ok`

```cpp
Ok = 0
```

drive it

*enumerator, declared at [`include/shulib/motion/curve_path.hpp:114`](../../include/shulib/motion/curve_path.hpp#L114).*

<a id="curveverdict-invalid"></a>

### `CurveVerdict::Invalid`

```cpp
Invalid = 1
```

refused: degenerate, or doubles back (CurveInvalid says how)

*enumerator, declared at [`include/shulib/motion/curve_path.hpp:115`](../../include/shulib/motion/curve_path.hpp#L115).*

<a id="curveverdict-startbehind"></a>

### `CurveVerdict::StartBehind`

```cpp
StartBehind = 2
```

refused: the first point more than maxStartOff off the nose (back, reversing)

*enumerator, declared at [`include/shulib/motion/curve_path.hpp:116`](../../include/shulib/motion/curve_path.hpp#L116).*

<a id="curveverdict-tightbend"></a>

### `CurveVerdict::TightBend`

```cpp
TightBend = 3
```

driven, warned: tightest bend radius < lookAhead / 2

*enumerator, declared at [`include/shulib/motion/curve_path.hpp:117`](../../include/shulib/motion/curve_path.hpp#L117).*

<a id="enum-class-curveinvalid"></a>

## `enum class CurveInvalid`

```cpp
enum class CurveInvalid : std::uint8_t
```

Why a path is Invalid.

*enum class, declared at [`include/shulib/motion/curve_path.hpp:121`](../../include/shulib/motion/curve_path.hpp#L121).*

<a id="curveinvalid-none"></a>

### `CurveInvalid::None`

```cpp
None = 0
```

not invalid

*enumerator, declared at [`include/shulib/motion/curve_path.hpp:122`](../../include/shulib/motion/curve_path.hpp#L122).*

<a id="curveinvalid-nopoints"></a>

### `CurveInvalid::NoPoints`

```cpp
NoPoints = 1
```

nothing left to drive (a lone headingless point at the robot, dropped)

*enumerator, declared at [`include/shulib/motion/curve_path.hpp:123`](../../include/shulib/motion/curve_path.hpp#L123).*

<a id="curveinvalid-atrobot"></a>

### `CurveInvalid::AtRobot`

```cpp
AtRobot = 2
```

a HEADED first point within kCurveMinSpacing of the robot

*enumerator, declared at [`include/shulib/motion/curve_path.hpp:124`](../../include/shulib/motion/curve_path.hpp#L124).*

<a id="curveinvalid-tooclose"></a>

### `CurveInvalid::TooClose`

```cpp
TooClose = 3
```

a point within kCurveMinSpacing of the knot before it

*enumerator, declared at [`include/shulib/motion/curve_path.hpp:125`](../../include/shulib/motion/curve_path.hpp#L125).*

<a id="curveinvalid-nodirection"></a>

### `CurveInvalid::NoDirection`

```cpp
NoDirection = 4
```

a headingless point whose two neighbours nearly coincide

*enumerator, declared at [`include/shulib/motion/curve_path.hpp:126`](../../include/shulib/motion/curve_path.hpp#L126).*

<a id="curveinvalid-doublesback"></a>

### `CurveInvalid::DoublesBack`

```cpp
DoublesBack = 5
```

a point's direction more than maxStartOff from an adjacent chord

*enumerator, declared at [`include/shulib/motion/curve_path.hpp:127`](../../include/shulib/motion/curve_path.hpp#L127).*

<a id="struct-curvecheck"></a>

## `struct CurveCheck`

```cpp
struct CurveCheck
```

What checkCurve() found, with the numbers its words quote. `point` is the AUTHOR's 1-based point number as written (a dropped first point does not renumber the rest); 0 = none.

*struct, declared at [`include/shulib/motion/curve_path.hpp:132`](../../include/shulib/motion/curve_path.hpp#L132).*

<a id="curvecheck-verdict"></a>

### `CurveCheck::verdict`

```cpp
CurveVerdict verdict = CurveVerdict::Ok
```

the verdict

*field, declared at [`include/shulib/motion/curve_path.hpp:133`](../../include/shulib/motion/curve_path.hpp#L133).*

<a id="curvecheck-why"></a>

### `CurveCheck::why`

```cpp
CurveInvalid why = CurveInvalid::None
```

Invalid's reason

*field, declared at [`include/shulib/motion/curve_path.hpp:134`](../../include/shulib/motion/curve_path.hpp#L134).*

<a id="curvecheck-point"></a>

### `CurveCheck::point`

```cpp
int point = 0
```

the point named (author's number), 0 = none

*field, declared at [`include/shulib/motion/curve_path.hpp:135`](../../include/shulib/motion/curve_path.hpp#L135).*

<a id="curvecheck-distance"></a>

### `CurveCheck::distance`

```cpp
double distance = 0.0
```

AtRobot / TooClose / NoDirection: the distance (in)

*field, declared at [`include/shulib/motion/curve_path.hpp:136`](../../include/shulib/motion/curve_path.hpp#L136).*

<a id="curvecheck-angle"></a>

### `CurveCheck::angle`

```cpp
double angle = 0.0
```

StartBehind / DoublesBack: the angle (rad)

*field, declared at [`include/shulib/motion/curve_path.hpp:137`](../../include/shulib/motion/curve_path.hpp#L137).*

<a id="curvecheck-tightestradius"></a>

### `CurveCheck::tightestRadius`

```cpp
double tightestRadius = 0.0
```

TightBend: the tightest bend's radius (in)

*field, declared at [`include/shulib/motion/curve_path.hpp:138`](../../include/shulib/motion/curve_path.hpp#L138).*

<a id="curvecheck-reverse"></a>

### `CurveCheck::reverse`

```cpp
bool reverse = false
```

StartBehind: measured from the robot's BACK (reverseCurveTo)

*field, declared at [`include/shulib/motion/curve_path.hpp:139`](../../include/shulib/motion/curve_path.hpp#L139).*

<a id="struct-curvesample"></a>

## `struct CurveSample`

```cpp
struct CurveSample
```

A point on the path: where, which way, how sharply it bends.

*struct, declared at [`include/shulib/motion/curve_path.hpp:143`](../../include/shulib/motion/curve_path.hpp#L143).*

<a id="curvesample-x"></a>

### `CurveSample::x`

```cpp
double x = 0.0
```

field x (in)

*field, declared at [`include/shulib/motion/curve_path.hpp:144`](../../include/shulib/motion/curve_path.hpp#L144).*

<a id="curvesample-y"></a>

### `CurveSample::y`

```cpp
double y = 0.0
```

field y (in)

*field, declared at [`include/shulib/motion/curve_path.hpp:145`](../../include/shulib/motion/curve_path.hpp#L145).*

<a id="curvesample-direction"></a>

### `CurveSample::direction`

```cpp
double direction = 0.0
```

direction of TRAVEL (rad, (-pi, pi])

*field, declared at [`include/shulib/motion/curve_path.hpp:146`](../../include/shulib/motion/curve_path.hpp#L146).*

<a id="curvesample-heading"></a>

### `CurveSample::heading`

```cpp
double heading = 0.0
```

the robot's NOSE there: direction, + pi on a reverse curve

*field, declared at [`include/shulib/motion/curve_path.hpp:147`](../../include/shulib/motion/curve_path.hpp#L147).*

<a id="curvesample-curvature"></a>

### `CurveSample::curvature`

```cpp
double curvature = 0.0
```

signed curvature of travel (1/in), + bending left

*field, declared at [`include/shulib/motion/curve_path.hpp:148`](../../include/shulib/motion/curve_path.hpp#L148).*

<a id="class-curvepath"></a>

## `class CurvePath`

```cpp
class CurvePath
```

The smooth path of one curve (file header). Built in two steps: the author's points at construction, the robot's live pose by build(). FIXED capacity, no heap, trivially copyable. Read by CurveFollow every tick and by host tools (the route preview): CurvePath, build(), length(), at(), speedLimitAt() and checkCurve() are what a tool may rely on; nothing here is frozen.

*class, declared at [`include/shulib/motion/curve_path.hpp:156`](../../include/shulib/motion/curve_path.hpp#L156).*

<a id="curvepath-curvepath"></a>

### `CurvePath::CurvePath`

```cpp
CurvePath(std::span<const CurvePoint> points, bool reverse)
```

The author's points (1..kMaxCurvePoints, every x and y finite — preconditions) and the direction: `reverse` = the robot's BACK leads (reverseCurveTo). Holds the points; builds nothing until build().

*function, declared at [`include/shulib/motion/curve_path.hpp:161`](../../include/shulib/motion/curve_path.hpp#L161).*

<a id="curvepath-build"></a>

### `CurvePath::build`

```cpp
void build(const math::Pose2d& start, const CurveConfig& cfg, double maxLinear, double maxAngular) noexcept
```

Build the path from the robot's live pose `start` (its nose heading): the near-start drop, the tangents, the refusal facts checkCurve() reads, the arc-length table and the speed plan (`maxLinear` in/s and `maxAngular` rad/s: the motion's budgets). Re-callable (rebuilds from the held points). Total: a refused path is still built finite, so a tool can draw it.

*function, declared at [`include/shulib/motion/curve_path.hpp:182`](../../include/shulib/motion/curve_path.hpp#L182).*

<a id="curvepath-built"></a>

### `CurvePath::built`

```cpp
[[nodiscard]] bool built() const noexcept
```

True once build() has run.

*function, declared at [`include/shulib/motion/curve_path.hpp:224`](../../include/shulib/motion/curve_path.hpp#L224).*

<a id="curvepath-reverse"></a>

### `CurvePath::reverse`

```cpp
[[nodiscard]] bool reverse() const noexcept
```

True for reverseCurveTo (the robot's back leads).

*function, declared at [`include/shulib/motion/curve_path.hpp:226`](../../include/shulib/motion/curve_path.hpp#L226).*

<a id="curvepath-authorpoints"></a>

### `CurvePath::authorPoints`

```cpp
[[nodiscard]] int authorPoints() const noexcept
```

The author's point count as written (before any drop).

*function, declared at [`include/shulib/motion/curve_path.hpp:228`](../../include/shulib/motion/curve_path.hpp#L228).*

<a id="curvepath-segments"></a>

### `CurvePath::segments`

```cpp
[[nodiscard]] int segments() const noexcept
```

Segments in the built path (= points driven through: a dropped first point is not one).

*function, declared at [`include/shulib/motion/curve_path.hpp:230`](../../include/shulib/motion/curve_path.hpp#L230).*

<a id="curvepath-droppedfirstdistance"></a>

### `CurvePath::droppedFirstDistance`

```cpp
[[nodiscard]] double droppedFirstDistance() const noexcept
```

The distance (in) of a DROPPED headingless first point from the robot; < 0 = none dropped.

*function, declared at [`include/shulib/motion/curve_path.hpp:232`](../../include/shulib/motion/curve_path.hpp#L232).*

<a id="curvepath-firstdrivenpoint"></a>

### `CurvePath::firstDrivenPoint`

```cpp
[[nodiscard]] int firstDrivenPoint() const noexcept
```

The author's number of the first point driven through (2 when point 1 was dropped).

*function, declared at [`include/shulib/motion/curve_path.hpp:234`](../../include/shulib/motion/curve_path.hpp#L234).*

<a id="curvepath-length"></a>

### `CurvePath::length`

```cpp
[[nodiscard]] double length() const noexcept
```

The path's length (in), along the path.

*function, declared at [`include/shulib/motion/curve_path.hpp:237`](../../include/shulib/motion/curve_path.hpp#L237).*

<a id="curvepath-knots"></a>

### `CurvePath::knotS`

```cpp
[[nodiscard]] double knotS(int i) const noexcept
```

The arc length (in) at which the path passes knot `i` (0 = the start, i = segments() the end).

*function, declared at [`include/shulib/motion/curve_path.hpp:239`](../../include/shulib/motion/curve_path.hpp#L239).*

<a id="curvepath-at"></a>

### `CurvePath::at`

```cpp
[[nodiscard]] CurveSample at(double s) const noexcept
```

The path at arc length `s` (clamped to [0, length()]): evaluated analytically from the two knots and their tangents, `s` mapped to the segment's parameter through the arc-length table.

*function, declared at [`include/shulib/motion/curve_path.hpp:245`](../../include/shulib/motion/curve_path.hpp#L245).*

<a id="curvepath-endpose"></a>

### `CurvePath::endPose`

```cpp
[[nodiscard]] math::Pose2d endPose() const
```

The robot's NOSE pose at the path's end: the last point's heading if it has one, else the path's own end direction (+ pi reversing — the robot finishes with its back along the path).

*function, declared at [`include/shulib/motion/curve_path.hpp:252`](../../include/shulib/motion/curve_path.hpp#L252).*

<a id="curvepath-lastpoint"></a>

### `CurvePath::lastPoint`

```cpp
[[nodiscard]] math::Pose2d lastPoint() const
```

The author's LAST point (x, y, and its nose heading — 0 when it has none), usable before build(): the curve publishes it as its target until the path exists.

*function, declared at [`include/shulib/motion/curve_path.hpp:260`](../../include/shulib/motion/curve_path.hpp#L260).*

<a id="curvepath-project"></a>

### `CurvePath::project`

```cpp
[[nodiscard]] double project(const math::Pose2d& pose, double fromS, double windowIn) const noexcept
```

The closest point to `pose`, searched FORWARD ONLY from `fromS` over `windowIn` of path: never backwards, never the whole path — a path that comes back near itself must not make the robot jump to another leg of it. Returns an arc length in [fromS, fromS + windowIn], clamped to the path.

*function, declared at [`include/shulib/motion/curve_path.hpp:273`](../../include/shulib/motion/curve_path.hpp#L273).*

<a id="curvepath-lookaheadpoint"></a>

### `CurvePath::lookAheadPoint`

```cpp
[[nodiscard]] CurveSample lookAheadPoint(double s, double L) const noexcept
```

The point `L` in of path past arc length `s` — past the end, on the end's line extended (along the end's direction of travel): the follower aims only at the drawn path or that line.

*function, declared at [`include/shulib/motion/curve_path.hpp:322`](../../include/shulib/motion/curve_path.hpp#L322).*

<a id="curvepath-tightestradius"></a>

### `CurvePath::tightestRadius`

```cpp
[[nodiscard]] double tightestRadius() const noexcept
```

The tightest bend's radius (in) — 1 / the largest |curvature| on the path (scanned on each segment, at most kScanStep in apart, both ends of every sample interval — so each knot's arriving side too); +infinity for a path with no bend.

*function, declared at [`include/shulib/motion/curve_path.hpp:338`](../../include/shulib/motion/curve_path.hpp#L338).*

<a id="curvepath-tightestat"></a>

### `CurvePath::tightestAt`

```cpp
[[nodiscard]] double tightestAt() const noexcept
```

Where (arc length, in) the tightest bend is.

*function, declared at [`include/shulib/motion/curve_path.hpp:342`](../../include/shulib/motion/curve_path.hpp#L342).*

<a id="curvepath-startangleoff"></a>

### `CurvePath::startAngleOff`

```cpp
[[nodiscard]] double startAngleOff() const noexcept
```

The angle (rad, [0, pi]) between the robot's nose (its back, reversing) at the start and the bearing to the first point driven through.

*function, declared at [`include/shulib/motion/curve_path.hpp:345`](../../include/shulib/motion/curve_path.hpp#L345).*

<a id="curvepath-speedlimitat"></a>

### `CurvePath::speedLimitAt`

```cpp
[[nodiscard]] double speedLimitAt(double s) const noexcept
```

The speed plan (in/s) at arc length `s` (file header): interpolated from the per-sample table build() filled; 0 at the end.

*function, declared at [`include/shulib/motion/curve_path.hpp:349`](../../include/shulib/motion/curve_path.hpp#L349).*

<a id="curvepath-ksamples"></a>

### `CurvePath::kSamples`

```cpp
static constexpr std::size_t kSamples = 17
```

The speed plan's samples, for a tool or a test: kSamples per segment, segment-major.

*field, declared at [`include/shulib/motion/curve_path.hpp:365`](../../include/shulib/motion/curve_path.hpp#L365).*

<a id="curvepath-samplecount"></a>

### `CurvePath::sampleCount`

```cpp
[[nodiscard]] int sampleCount() const noexcept
```

Sample count = segments() x kSamples (a knot is sampled twice: end of one segment, start of the next, at the same arc length and with the same speed).

*function, declared at [`include/shulib/motion/curve_path.hpp:368`](../../include/shulib/motion/curve_path.hpp#L368).*

<a id="curvepath-samples"></a>

### `CurvePath::sampleS`

```cpp
[[nodiscard]] double sampleS(int i) const noexcept
```

Sample `i`'s arc length (in).

*function, declared at [`include/shulib/motion/curve_path.hpp:370`](../../include/shulib/motion/curve_path.hpp#L370).*

<a id="curvepath-samplespeed"></a>

### `CurvePath::sampleSpeed`

```cpp
[[nodiscard]] double sampleSpeed(int i) const noexcept
```

Sample `i`'s planned speed (in/s).

*function, declared at [`include/shulib/motion/curve_path.hpp:376`](../../include/shulib/motion/curve_path.hpp#L376).*

<a id="curvepath-invalidfacts"></a>

### `CurvePath::invalidFacts`

```cpp
[[nodiscard]] const CurveCheck& invalidFacts() const noexcept
```

The refusal facts build() found (checkCurve() decides with them). Invalid or Ok.

*function, declared at [`include/shulib/motion/curve_path.hpp:382`](../../include/shulib/motion/curve_path.hpp#L382).*

<a id="curvepath-kscanstep"></a>

### `CurvePath::kScanStep`

```cpp
static constexpr double kScanStep = 0.25
```

The arc-length grid step (in) of the closest-point search, the curvature scan and the speed plan's 2 in look-ahead.

*field, declared at [`include/shulib/motion/curve_path.hpp:386`](../../include/shulib/motion/curve_path.hpp#L386).*

<a id="curvepath-kbendahead"></a>

### `CurvePath::kBendAhead`

```cpp
static constexpr double kBendAhead = 2.0
```

How far ahead (in) a bend is felt by the speed plan.

*field, declared at [`include/shulib/motion/curve_path.hpp:388`](../../include/shulib/motion/curve_path.hpp#L388).*

<a id="curvepath-kplanstartspeed"></a>

### `CurvePath::kPlanStartSpeed`

```cpp
static constexpr double kPlanStartSpeed = 12.0
```

The speed plan's value at the start (in/s), from which its forward pass rises at CurveConfig::accel per inch travelled. The host-test-plant probes' value; PROVISIONAL (A4: HA-246).

*field, declared at [`include/shulib/motion/curve_path.hpp:392`](../../include/shulib/motion/curve_path.hpp#L392).*

<a id="checkcurve"></a>

## `checkCurve`

```cpp
[[nodiscard]] inline CurveCheck checkCurve(const CurvePath& path, const CurveConfig& cfg) noexcept
```

The D3 verdicts for a built path (curve_path.hpp header), in order: Invalid (the first degenerate or doubling-back point), then StartBehind (startAngleOff() > maxStartOff — exactly maxStartOff drives), then TightBend (tightestRadius() < lookAhead / 2), else Ok. One function the follower, the documentation's examples and host tools share. Precondition: built().

*free function, declared at [`include/shulib/motion/curve_path.hpp:771`](../../include/shulib/motion/curve_path.hpp#L771).*

<a id="appendcurvecheckwords"></a>

## `appendCurveCheckWords`

```cpp
inline void appendCurveCheckWords(diag::lineformat::Line& line, const CurveCheck& c, const CurveConfig& cfg)
```

The words a CurveCheck earns, appended to `line` (ASCII, the one §18.3 formatter): the refusals' "curve not driven: ..." and TightBend's "curve: tightest bend ..." — exactly what the follower logs, so a host tool can show the same sentence. Ok appends nothing.

*free function, declared at [`include/shulib/motion/curve_path.hpp:791`](../../include/shulib/motion/curve_path.hpp#L791).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 64 lines, click to expand</summary>

```text

 curve_path.hpp — the PURE path of the curve verbs (chunk CURVE1, 2026-11): the smooth path a
 curveTo / reverseCurveTo drives, built from the robot's live pose and the author's points, its
 speed plan, and the refusals and warnings a path earns before the robot moves. No chassis, no
 scheduler, no PROS, no heap, no state beyond the path itself: a host tool (the route preview)
 builds and reads the same object the robot follows.

 ── The one rule to remember ─────────────────────────────────────────────────────────
 A point's heading is ALWAYS the way the robot's NOSE points — also on reverseCurveTo, where the
 robot travels opposite to it (its back leads). Writing the direction of TRAVEL on a reverse
 curve makes the end heading point back along the path, and the path refuses it (below).
 Do not list the robot's own spot as the first point: the path starts where the robot IS.

 ── The shape ────────────────────────────────────────────────────────────────────────
 Knots: the robot's live pose (filled by build(), at the curve's first live tick — never at
 construction: the boot window has no heading), then the author's points (at most
 kMaxCurvePoints). Each knot has a unit TANGENT in the direction of travel: the robot's heading
 at the start (+ pi reversing), a point's heading (+ pi reversing), or — for a point without
 one — the direction from the knot before it to the knot after it (Catmull-Rom); a last point
 without a heading takes the direction from the knot before it. Consecutive knots are joined by
 a cubic HERMITE segment whose tangent magnitude is the segment's chord, so the path passes
 through every point exactly, along its tangent. Nothing is stored densely: per segment, 17
 samples (u = 0, 1/16, ..., 1) of the arc length and of the speed plan, as float; every
 position, direction and curvature is evaluated from the two knots and their tangents.

 ── What is refused (checkCurve, before the robot moves) ───────────────────────────────
   Invalid      the path doubles back or is degenerate, naming the point:
                  * a headed first point within kCurveMinSpacing of the robot ("where the robot
                    already is"); a headingless one there is DROPPED instead (one Info line);
                  * a point within kCurveMinSpacing of the knot before it;
                  * a point without a heading whose two neighbours are within kCurveMinSpacing
                    of each other (an out-and-back through it has no direction — this is
                    checked BEFORE the tangent is normalised, which would divide by ~0);
                  * any point whose direction (its heading, or the one picked for it) is more
                    than maxStartOff (90 deg) from the chord arriving at it or the chord leaving
                    it — the start rule, at every point;
                  * no point left (a lone headingless point at the robot, dropped).
   StartBehind  the first point more than maxStartOff off the robot's nose (its back,
                reversing): "turn first: face()" (reversing: "... off the robot's back ...
                turn first: faceAway()").
   TightBend    driven, but warned: the tightest bend's radius is under lookAhead / 2.
 Why there is no runtime "the path reverses" check: with every direction within 90 deg of both
 chords at its knot and tangent magnitude = chord, a segment's derivative along its chord c is
   P'(u).c / |c|^2 = 6u(1-u) + (1-u)(1-3u) cos a + u(3u-2) cos b,
 linear in cos a and cos b in [0, 1], whose four corners 6u(1-u), (1-u)(1+3u), u(4-3u) and 1
 are all >= 0 on [0, 1] — so it never turns against its chord. A test property
 (test/curve_path_test.cpp CV1), not a branch.

 ── The speed plan (built once, in build()) ─────────────────────────────────────────────
 At each sample: the least of sqrt(bendAccel / K) — K the largest |curvature| over the sample
 intervals either side of it (each evaluated on its own segment, so a knot's ARRIVING side counts
 too: the interpolated plan is bounded between samples, not only at them) and within the next
 2 in of path, so a bend is felt before its apex — maxAngular / K (the robot's turn-rate limit
 never has to carry the arc), and maxLinear; 0 at the end; then a BACKWARD pass,
 v[i] = min(v[i], sqrt(v[i+1]^2 + 2 decel ds)), so the robot is already at a bend's speed when
 it reaches the bend; then a FORWARD pass from kPlanStartSpeed (12 in/s) at the start,
 v[i] = min(v[i], sqrt(v[i-1]^2 + 2 accel ds)) — the plan rises by the ground actually covered,
 which under motor lag holds the robot back until it has moved (the host-test-plant probes ran
 both passes). speedLimitAt(s) interpolates it linearly in arc length.

 Every tunable this reads is CurveConfig's (motion_config.hpp), PROVISIONAL (A4: HA-246). Its
 own constants: kMaxCurvePoints and kCurveMinSpacing (design constants), kScanStep and kBendAhead
 (the plan's grid), and kPlanStartSpeed (12 in/s, the plan's value at the start — a fixed constant
 here, PROVISIONAL, HA-246). Proven on the host test plant only.
```

</details>
