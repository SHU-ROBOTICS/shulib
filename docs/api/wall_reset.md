<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/localization/wall_reset.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `wall_reset.hpp`

The WALL RESET's pure core (chunk L6-A1, 2026-10-29; the L6 brief, the team lead's answers of 2026-10-22): the geometry that turns one side distance sensor's reading into ONE field coordinate, the gates that REFUSE rather than guess, the verdict, and the on…

This header declares **16** types (144 members), **17** free functions, and **6** constants.

Extracted from [`include/shulib/localization/wall_reset.hpp`](../../include/shulib/localization/wall_reset.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`kWallDegToRad`](#kwalldegtorad) — *constant*
- [`kPublishedCloseAccuracyInches`](#kpublishedcloseaccuracyinches) — *constant*
- [`kPublishedFarAccuracyFraction`](#kpublishedfaraccuracyfraction) — *constant*
- [`enum class WallSide`](#enum-class-wallside)
  - [`Left`](#wallside-left)
  - [`Right`](#wallside-right)
- [`wallSideWord`](#wallsideword) — *free function*
- [`enum class WallFrame`](#enum-class-wallframe)
  - [`Field`](#wallframe-field)
  - [`Own`](#wallframe-own)
- [`enum class KeepOutKind`](#enum-class-keepoutkind)
  - [`Fixed`](#keepoutkind-fixed)
  - [`Toggle`](#keepoutkind-toggle)
  - [`PieceZone`](#keepoutkind-piecezone)
- [`struct WallKeepOut`](#struct-wallkeepout)
  - [`from`](#wallkeepout-from)
  - [`to`](#wallkeepout-to)
  - [`kind`](#wallkeepout-kind)
  - [`why`](#wallkeepout-why)
- [`kMaxWallKeepOuts`](#kmaxwallkeepouts) — *constant*
- [`struct FieldWall`](#struct-fieldwall)
  - [`name`](#fieldwall-name)
  - [`axis`](#fieldwall-axis)
  - [`coord`](#fieldwall-coord)
  - [`inward`](#fieldwall-inward)
  - [`frame`](#fieldwall-frame)
  - [`keepOut`](#fieldwall-keepout)
  - [`keepOutCount`](#fieldwall-keepoutcount)
  - [`line`](#fieldwall-line)
  - [`keepOuts`](#fieldwall-keepouts)
- [`struct SideMount`](#struct-sidemount)
  - [`measured`](#sidemount-measured)
  - [`forward`](#sidemount-forward)
  - [`left`](#sidemount-left)
  - [`height`](#sidemount-height)
  - [`facing`](#sidemount-facing)
  - [`positionTolerance`](#sidemount-positiontolerance)
  - [`facingTolerance`](#sidemount-facingtolerance)
  - [`rangeBias`](#sidemount-rangebias)
  - [`rangeAccuracy`](#sidemount-rangeaccuracy)
  - [`coneHalfAngle`](#sidemount-conehalfangle)
  - [`toggleChecked`](#sidemount-togglechecked)
  - [`provenance`](#sidemount-provenance)
- [`struct WallResetLimits`](#struct-wallresetlimits)
  - [`rangeMin`](#wallresetlimits-rangemin)
  - [`rangeMax`](#wallresetlimits-rangemax)
  - [`squareWithin`](#wallresetlimits-squarewithin)
  - [`spreadMax`](#wallresetlimits-spreadmax)
  - [`minSamples`](#wallresetlimits-minsamples)
  - [`window`](#wallresetlimits-window)
  - [`windowMax`](#wallresetlimits-windowmax)
  - [`minConfidence`](#wallresetlimits-minconfidence)
  - [`stillSpeed`](#wallresetlimits-stillspeed)
  - [`stillTurn`](#wallresetlimits-stillturn)
  - [`errorBoundMax`](#wallresetlimits-errorboundmax)
  - [`keepOutMargin`](#wallresetlimits-keepoutmargin)
  - [`alongDriftRate`](#wallresetlimits-alongdriftrate)
  - [`coneHalfAngle`](#wallresetlimits-conehalfangle)
  - [`pieceTop`](#wallresetlimits-piecetop)
  - [`beamMargin`](#wallresetlimits-beammargin)
  - [`headingFloor`](#wallresetlimits-headingfloor)
  - [`headingPerTurn`](#wallresetlimits-headingperturn)
- [`struct WallResetRequest`](#struct-wallresetrequest)
  - [`wall`](#wallresetrequest-wall)
  - [`sensor`](#wallresetrequest-sensor)
  - [`maxCorrection`](#wallresetrequest-maxcorrection)
  - [`required`](#wallresetrequest-required)
  - [`name`](#wallresetrequest-name)
- [`struct WallSample`](#struct-wallsample)
  - [`distance`](#wallsample-distance)
  - [`confidence`](#wallsample-confidence)
  - [`faultMoved`](#wallsample-faultmoved)
  - [`at`](#wallsample-at)
- [`kMaxWallSamples`](#kmaxwallsamples) — *constant*
- [`kMaxWallWindowSeconds`](#kmaxwallwindowseconds) — *constant*
- [`enum class WallResetVerdict`](#enum-class-wallresetverdict)
  - [`Seated`](#wallresetverdict-seated)
  - [`NoSensor`](#wallresetverdict-nosensor)
  - [`SensorAbsent`](#wallresetverdict-sensorabsent)
  - [`MountUnset`](#wallresetverdict-mountunset)
  - [`NotLive`](#wallresetverdict-notlive)
  - [`MotionActive`](#wallresetverdict-motionactive)
  - [`BadRequest`](#wallresetverdict-badrequest)
  - [`NotFieldStart`](#wallresetverdict-notfieldstart)
  - [`NotSquare`](#wallresetverdict-notsquare)
  - [`KeepOut`](#wallresetverdict-keepout)
  - [`BeamLow`](#wallresetverdict-beamlow)
  - [`Moving`](#wallresetverdict-moving)
  - [`ReadFault`](#wallresetverdict-readfault)
  - [`NoReturn`](#wallresetverdict-noreturn)
  - [`LowConfidence`](#wallresetverdict-lowconfidence)
  - [`OutOfRange`](#wallresetverdict-outofrange)
  - [`TooFewSamples`](#wallresetverdict-toofewsamples)
  - [`Spread`](#wallresetverdict-spread)
  - [`Imprecise`](#wallresetverdict-imprecise)
  - [`Disagrees`](#wallresetverdict-disagrees)
  - [`Cut`](#wallresetverdict-cut)
- [`wallResetVerdictWord`](#wallresetverdictword) — *free function*
- [`struct WallResetReport`](#struct-wallresetreport)
  - [`verdict`](#wallresetreport-verdict)
  - [`axis`](#wallresetreport-axis)
  - [`side`](#wallresetreport-side)
  - [`required`](#wallresetreport-required)
  - [`before`](#wallresetreport-before)
  - [`measured`](#wallresetreport-measured)
  - [`applied`](#wallresetreport-applied)
  - [`maxCorrection`](#wallresetreport-maxcorrection)
  - [`median`](#wallresetreport-median)
  - [`spread`](#wallresetreport-spread)
  - [`samples`](#wallresetreport-samples)
  - [`windowSpan`](#wallresetreport-windowspan)
  - [`offSquare`](#wallresetreport-offsquare)
  - [`expectedRange`](#wallresetreport-expectedrange)
  - [`along`](#wallresetreport-along)
  - [`beamHalfWidth`](#wallresetreport-beamhalfwidth)
  - [`keepOutMargin`](#wallresetreport-keepoutmargin)
  - [`alongUncertainty`](#wallresetreport-alonguncertainty)
  - [`cone`](#wallresetreport-cone)
  - [`ceiling`](#wallresetreport-ceiling)
  - [`headingTolerance`](#wallresetreport-headingtolerance)
  - [`errorBound`](#wallresetreport-errorbound)
  - [`speedSeen`](#wallresetreport-speedseen)
  - [`turnSeen`](#wallresetreport-turnseen)
  - [`sampleIndex`](#wallresetreport-sampleindex)
  - [`sampleDistance`](#wallresetreport-sampledistance)
  - [`sampleConfidence`](#wallresetreport-sampleconfidence)
  - [`detail`](#wallresetreport-detail)
  - [`cleanFound`](#wallresetreport-cleanfound)
  - [`cleanFrom`](#wallresetreport-cleanfrom)
  - [`cleanTo`](#wallresetreport-cleanto)
  - [`faultChannel`](#wallresetreport-faultchannel)
  - [`seated`](#wallresetreport-seated)
  - [`continues`](#wallresetreport-continues)
- [`struct WallSensor`](#struct-wallsensor)
  - [`sensor`](#wallsensor-sensor)
  - [`faults`](#wallsensor-faults)
  - [`mount`](#wallsensor-mount)
  - [`absence`](#wallsensor-absence)
- [`struct WallResetLog`](#struct-wallresetlog)
  - [`seated`](#wallresetlog-seated)
  - [`refused`](#wallresetlog-refused)
  - [`last`](#wallresetlog-last)
  - [`record`](#wallresetlog-record)
- [`struct WallSensors`](#struct-wallsensors)
  - [`left`](#wallsensors-left)
  - [`right`](#wallsensors-right)
  - [`limits`](#wallsensors-limits)
  - [`log`](#wallsensors-log)
  - [`side`](#wallsensors-side)
- [`readingAccuracy`](#readingaccuracy) — *free function*
- [`axisFromReading`](#axisfromreading) — *free function*
- [`axisErrorBound`](#axiserrorbound) — *free function*
- [`headingTolerance`](#headingtolerance) — *free function*
- [`effectiveCone`](#effectivecone) — *free function*
- [`beamCeiling`](#beamceiling) — *free function*
- [`wallResetProblem`](#wallresetproblem) — *free function*
- [`struct WallAim`](#struct-wallaim)
  - [`offSquare`](#wallaim-offsquare)
  - [`expectedRange`](#wallaim-expectedrange)
  - [`along`](#wallaim-along)
  - [`beamHalfWidth`](#wallaim-beamhalfwidth)
  - [`margin`](#wallaim-margin)
  - [`drift`](#wallaim-drift)
  - [`widening`](#wallaim-widening)
  - [`cone`](#wallaim-cone)
  - [`ceiling`](#wallaim-ceiling)
  - [`keepOutWhy`](#wallaim-keepoutwhy)
  - [`cleanFound`](#wallaim-cleanfound)
  - [`cleanFrom`](#wallaim-cleanfrom)
  - [`cleanTo`](#wallaim-cleanto)
- [`keepOutActive`](#keepoutactive) — *free function*
- [`aimAt`](#aimat) — *free function*
- [`preCheck`](#precheck) — *free function*
- [`sampleVerdict`](#sampleverdict) — *free function*
- [`wallStill`](#wallstill) — *free function*
- [`describeWallReset`](#describewallreset) — *free function*
- [`evaluateWallReset`](#evaluatewallreset) — *free function*
- [`class WallSampler`](#class-wallsampler)
  - [`WallSampler`](#wallsampler-wallsampler)
  - [`step`](#wallsampler-step)
  - [`samples`](#wallsampler-samples)
  - [`maxTwist`](#wallsampler-maxtwist)
- [`appendWallResetLine`](#appendwallresetline) — *free function*

<a id="kwalldegtorad"></a>

## `kWallDegToRad`

```cpp
inline constexpr double kWallDegToRad = math::Angle::kPi / 180.0
```

Degrees to radians, as a constexpr factor: the wall reset's angle defaults are written `5.0 * kWallDegToRad` because the `_deg` literal builds a non-constexpr math::Angle.

*constant, declared at [`include/shulib/localization/wall_reset.hpp:78`](../../include/shulib/localization/wall_reset.hpp#L78).*

<a id="kpublishedcloseaccuracyinches"></a>

## `kPublishedCloseAccuracyInches`

```cpp
inline constexpr double kPublishedCloseAccuracyInches = 15.0 / 25.4
```

VEX's published accuracy of the V5 Distance Sensor below 200 mm — "approximately ±15 mm" — in inches. With kPublishedFarAccuracyFraction it gives the accuracy the reset assumes until a measured `SideMount::rangeAccuracy` replaces it: max(15 mm, 5 % of the reading) at EVERY range, never a step at 200 mm (a step would put a cliff in the precision gate). A4 register HA-204.

*constant, declared at [`include/shulib/localization/wall_reset.hpp:84`](../../include/shulib/localization/wall_reset.hpp#L84).*

<a id="kpublishedfaraccuracyfraction"></a>

## `kPublishedFarAccuracyFraction`

```cpp
inline constexpr double kPublishedFarAccuracyFraction = 0.05
```

VEX's published accuracy above 200 mm — "about 5 %" of the reading. A4 register HA-204.

*constant, declared at [`include/shulib/localization/wall_reset.hpp:87`](../../include/shulib/localization/wall_reset.hpp#L87).*

<a id="enum-class-wallside"></a>

## `enum class WallSide`

```cpp
enum class WallSide : std::uint8_t
```

Which of the robot's two side sensors a reset reads (one per side, facing straight out — the team lead's 2026-10-08 ruling for both robots).

*enum class, declared at [`include/shulib/localization/wall_reset.hpp:91`](../../include/shulib/localization/wall_reset.hpp#L91).*

<a id="wallside-left"></a>

### `WallSide::Left`

```cpp
Left
```

The sensor on the robot's LEFT side (facing +90 degrees from forward, typically).

*enumerator, declared at [`include/shulib/localization/wall_reset.hpp:93`](../../include/shulib/localization/wall_reset.hpp#L93).*

<a id="wallside-right"></a>

### `WallSide::Right`

```cpp
Right
```

The sensor on the robot's RIGHT side (facing -90 degrees from forward, typically).

*enumerator, declared at [`include/shulib/localization/wall_reset.hpp:95`](../../include/shulib/localization/wall_reset.hpp#L95).*

<a id="wallsideword"></a>

## `wallSideWord`

```cpp
[[nodiscard]] constexpr const char* wallSideWord(WallSide side) noexcept
```

The side's word in the log line: "LEFT" or "RIGHT" (printed "(LEFT sensor)" — a sensor, never a direction). Exhaustive, no default.

*free function, declared at [`include/shulib/localization/wall_reset.hpp:100`](../../include/shulib/localization/wall_reset.hpp#L100).*

<a id="enum-class-wallframe"></a>

## `enum class WallFrame`

```cpp
enum class WallFrame : std::uint8_t
```

Whose coordinates a wall is in.

*enum class, declared at [`include/shulib/localization/wall_reset.hpp:111`](../../include/shulib/localization/wall_reset.hpp#L111).*

<a id="wallframe-field"></a>

### `WallFrame::Field`

```cpp
Field
```

A SEASON wall in the field frame (F1) — localization::override2026's. Meaningful only when the estimate is in field coordinates, i.e. the auton declared a FIELD start.

*enumerator, declared at [`include/shulib/localization/wall_reset.hpp:114`](../../include/shulib/localization/wall_reset.hpp#L114).*

<a id="wallframe-own"></a>

### `WallFrame::Own`

```cpp
Own
```

An author's OWN line (`FieldWall::line`), in whatever frame the author's code uses, with the author's own keep-outs or none — the explicit, Tier-3 form's.

*enumerator, declared at [`include/shulib/localization/wall_reset.hpp:117`](../../include/shulib/localization/wall_reset.hpp#L117).*

<a id="enum-class-keepoutkind"></a>

## `enum class KeepOutKind`

```cpp
enum class KeepOutKind : std::uint8_t
```

What lifts a keep-out span (rule 3).

*enum class, declared at [`include/shulib/localization/wall_reset.hpp:121`](../../include/shulib/localization/wall_reset.hpp#L121).*

<a id="keepoutkind-fixed"></a>

### `KeepOutKind::Fixed`

```cpp
Fixed
```

Never lifted: the Toggle's brackets, the Loaders, the corners, the VEX U Load Zones.

*enumerator, declared at [`include/shulib/localization/wall_reset.hpp:123`](../../include/shulib/localization/wall_reset.hpp#L123).*

<a id="keepoutkind-toggle"></a>

### `KeepOutKind::Toggle`

```cpp
Toggle
```

The Toggle's BODY: lifted per side only by the mount's MEASURED `toggleChecked` (that sensor, at its height, was seen to read the wall beneath a real Toggle on a field).

*enumerator, declared at [`include/shulib/localization/wall_reset.hpp:126`](../../include/shulib/localization/wall_reset.hpp#L126).*

<a id="keepoutkind-piecezone"></a>

### `KeepOutKind::PieceZone`

```cpp
PieceZone
```

Where single game pieces are KNOWN to stand (the start pieces): applies while the mount's cone is UNMEASURED and lifts once a measured `coneHalfAngle` is typed, because the beam-height gate (rule 4) then rests on a measured cone.

*enumerator, declared at [`include/shulib/localization/wall_reset.hpp:130`](../../include/shulib/localization/wall_reset.hpp#L130).*

<a id="struct-wallkeepout"></a>

## `struct WallKeepOut`

```cpp
struct WallKeepOut
```

One stretch of a wall a beam must not land on, as coordinates ALONG the wall (`from` <= `to`; the along axis is the one the wall does not measure).

*struct, declared at [`include/shulib/localization/wall_reset.hpp:135`](../../include/shulib/localization/wall_reset.hpp#L135).*

<a id="wallkeepout-from"></a>

### `WallKeepOut::from`

```cpp
units::Length from{}
```

The span's lower end along the wall (inches).

*field, declared at [`include/shulib/localization/wall_reset.hpp:137`](../../include/shulib/localization/wall_reset.hpp#L137).*

<a id="wallkeepout-to"></a>

### `WallKeepOut::to`

```cpp
units::Length to{}
```

The span's upper end along the wall (inches); >= `from`.

*field, declared at [`include/shulib/localization/wall_reset.hpp:139`](../../include/shulib/localization/wall_reset.hpp#L139).*

<a id="wallkeepout-kind"></a>

### `WallKeepOut::kind`

```cpp
KeepOutKind kind = KeepOutKind::Fixed
```

What lifts it (KeepOutKind).

*field, declared at [`include/shulib/localization/wall_reset.hpp:141`](../../include/shulib/localization/wall_reset.hpp#L141).*

<a id="wallkeepout-why"></a>

### `WallKeepOut::why`

```cpp
const char* why = ""
```

The span's name in the log line ("Loader", "Load Zone", "Toggle", ...) — a stable literal.

*field, declared at [`include/shulib/localization/wall_reset.hpp:143`](../../include/shulib/localization/wall_reset.hpp#L143).*

<a id="kmaxwallkeepouts"></a>

## `kMaxWallKeepOuts`

```cpp
inline constexpr std::size_t kMaxWallKeepOuts = 12
```

The most keep-out spans one wall carries (the red and blue walls need 11).

*constant, declared at [`include/shulib/localization/wall_reset.hpp:147`](../../include/shulib/localization/wall_reset.hpp#L147).*

<a id="struct-fieldwall"></a>

## `struct FieldWall`

```cpp
struct FieldWall
```

A wall: the line `axis = coord`, with the field on its `inward` side (+1: the field is at larger coordinates, as for the left wall at x = -70.2; -1: at smaller), and the spans along it a beam must not land on.

*struct, declared at [`include/shulib/localization/wall_reset.hpp:152`](../../include/shulib/localization/wall_reset.hpp#L152).*

<a id="fieldwall-name"></a>

### `FieldWall::name`

```cpp
const char* name = ""
```

The wall's name in the log line ("left wall") — a stable literal.

*field, declared at [`include/shulib/localization/wall_reset.hpp:154`](../../include/shulib/localization/wall_reset.hpp#L154).*

<a id="fieldwall-axis"></a>

### `FieldWall::axis`

```cpp
Axis axis = Axis::X
```

The axis this wall measures.

*field, declared at [`include/shulib/localization/wall_reset.hpp:156`](../../include/shulib/localization/wall_reset.hpp#L156).*

<a id="fieldwall-coord"></a>

### `FieldWall::coord`

```cpp
units::Length coord{}
```

The wall's inner face, as a coordinate on `axis` (inches).

*field, declared at [`include/shulib/localization/wall_reset.hpp:158`](../../include/shulib/localization/wall_reset.hpp#L158).*

<a id="fieldwall-inward"></a>

### `FieldWall::inward`

```cpp
std::int8_t inward = +1
```

+1 or -1: the side of the line the field (and the robot) is on. Anything else is refused BadRequest.

*field, declared at [`include/shulib/localization/wall_reset.hpp:161`](../../include/shulib/localization/wall_reset.hpp#L161).*

<a id="fieldwall-frame"></a>

### `FieldWall::frame`

```cpp
WallFrame frame = WallFrame::Field
```

Whose coordinates the wall is in (WallFrame).

*field, declared at [`include/shulib/localization/wall_reset.hpp:163`](../../include/shulib/localization/wall_reset.hpp#L163).*

<a id="fieldwall-keepout"></a>

### `FieldWall::keepOut`

```cpp
std::array<WallKeepOut, kMaxWallKeepOuts> keepOut{}
```

The keep-out spans; the first `keepOutCount` are used.

*field, declared at [`include/shulib/localization/wall_reset.hpp:165`](../../include/shulib/localization/wall_reset.hpp#L165).*

<a id="fieldwall-keepoutcount"></a>

### `FieldWall::keepOutCount`

```cpp
std::uint8_t keepOutCount = 0
```

How many of `keepOut` are used.

*field, declared at [`include/shulib/localization/wall_reset.hpp:167`](../../include/shulib/localization/wall_reset.hpp#L167).*

<a id="fieldwall-line"></a>

### `FieldWall::line`

```cpp
[[nodiscard]] static constexpr FieldWall line(const char* name, Axis axis, units::Length coord, std::int8_t inward, std::span<const WallKeepOut> keepOuts = {})
```

An author's OWN wall line (frame `Own`) — for hand-wired Tier-3 code and tests that know their frame: the line `axis = coord`, the field on the `inward` side, and the author's keep-outs (none by default — a line with none has none: the reset then cannot tell a Loader from the wall). More than kMaxWallKeepOuts spans is a programming error (a precondition).

*function, declared at [`include/shulib/localization/wall_reset.hpp:173`](../../include/shulib/localization/wall_reset.hpp#L173).*

<a id="fieldwall-keepouts"></a>

### `FieldWall::keepOuts`

```cpp
[[nodiscard]] constexpr std::span<const WallKeepOut> keepOuts() const noexcept
```

The used keep-out spans, as a view into this wall.

*function, declared at [`include/shulib/localization/wall_reset.hpp:192`](../../include/shulib/localization/wall_reset.hpp#L192).*

<a id="struct-sidemount"></a>

## `struct SideMount`

```cpp
struct SideMount
```

One side sensor's place on the robot, in the BODY frame, MEASURED with a tape on the robot — every number with its tolerance, and a provenance string. `measured == false` (the default) makes every reset with this sensor REFUSE `MountUnset`; a placeholder "so the reset works" is the defect this project has paid for before.

*struct, declared at [`include/shulib/localization/wall_reset.hpp:202`](../../include/shulib/localization/wall_reset.hpp#L202).*

<a id="sidemount-measured"></a>

### `SideMount::measured`

```cpp
bool measured = false
```

False = UNSET: every reset with this sensor refuses MountUnset (never the robot).

*field, declared at [`include/shulib/localization/wall_reset.hpp:204`](../../include/shulib/localization/wall_reset.hpp#L204).*

<a id="sidemount-forward"></a>

### `SideMount::forward`

```cpp
units::Length forward{}
```

The lens's offset FORWARD of the robot's centre (the turning centre on a tank), inches. The fore-aft centre (0) is load-bearing for precision: the axis error from a heading error is this offset times the heading error.

*field, declared at [`include/shulib/localization/wall_reset.hpp:208`](../../include/shulib/localization/wall_reset.hpp#L208).*

<a id="sidemount-left"></a>

### `SideMount::left`

```cpp
units::Length left{}
```

The lens's offset to the LEFT of the robot's centre, inches (negative = right).

*field, declared at [`include/shulib/localization/wall_reset.hpp:210`](../../include/shulib/localization/wall_reset.hpp#L210).*

<a id="sidemount-height"></a>

### `SideMount::height`

```cpp
units::Length height{}
```

The lens centre's height above the floor, inches — decides the beam-height ceiling (rule 4).

*field, declared at [`include/shulib/localization/wall_reset.hpp:212`](../../include/shulib/localization/wall_reset.hpp#L212).*

<a id="sidemount-facing"></a>

### `SideMount::facing`

```cpp
units::AngleDim facing{}
```

The beam's direction, counter-clockwise from forward: `90.0 * kWallDegToRad` = straight out the left side, `-90.0 * kWallDegToRad` the right.

*field, declared at [`include/shulib/localization/wall_reset.hpp:215`](../../include/shulib/localization/wall_reset.hpp#L215).*

<a id="sidemount-positiontolerance"></a>

### `SideMount::positionTolerance`

```cpp
units::Length positionTolerance{0.25}
```

How well `forward` and `left` were measured (inches, +/-). PROVISIONAL default (A4: HA-211).

*field, declared at [`include/shulib/localization/wall_reset.hpp:217`](../../include/shulib/localization/wall_reset.hpp#L217).*

<a id="sidemount-facingtolerance"></a>

### `SideMount::facingTolerance`

```cpp
units::AngleDim facingTolerance{2.0 * kWallDegToRad}
```

How square the sensor was found (+/-). PROVISIONAL default (A4: HA-211).

*field, declared at [`include/shulib/localization/wall_reset.hpp:219`](../../include/shulib/localization/wall_reset.hpp#L219).*

<a id="sidemount-rangebias"></a>

### `SideMount::rangeBias`

```cpp
units::Length rangeBias{}
```

Subtracted from every reading; 0 = none measured. Typed ONLY when this sensor's readings at taped distances from a wall show a consistent offset (A4 register HA-204).

*field, declared at [`include/shulib/localization/wall_reset.hpp:222`](../../include/shulib/localization/wall_reset.hpp#L222).*

<a id="sidemount-rangeaccuracy"></a>

### `SideMount::rangeAccuracy`

```cpp
units::Length rangeAccuracy{}
```

This sensor's +/- accuracy, measured from readings at taped distances from a wall; 0 = unmeasured, and the reset assumes VEX's published max(15 mm, 5 % of the reading) — A4 register HA-204.

*field, declared at [`include/shulib/localization/wall_reset.hpp:226`](../../include/shulib/localization/wall_reset.hpp#L226).*

<a id="sidemount-conehalfangle"></a>

### `SideMount::coneHalfAngle`

```cpp
units::AngleDim coneHalfAngle{}
```

The beam's half-angle, measured (how far to the side of the beam a target is still seen); 0 = UNMEASURED: the limits' 18 degrees is used and the start-piece zones stay kept out (A4 register HA-205).

*field, declared at [`include/shulib/localization/wall_reset.hpp:230`](../../include/shulib/localization/wall_reset.hpp#L230).*

<a id="sidemount-togglechecked"></a>

### `SideMount::toggleChecked`

```cpp
bool toggleChecked = false
```

True only when this sensor, at its height, was seen to read the WALL beneath a real Toggle's BODY on a field — then the Toggle body's keep-out lifts for it (its brackets never do — A4 register HA-208).

*field, declared at [`include/shulib/localization/wall_reset.hpp:234`](../../include/shulib/localization/wall_reset.hpp#L234).*

<a id="sidemount-provenance"></a>

### `SideMount::provenance`

```cpp
const char* provenance = ""
```

Who measured what, how and when — a stable literal; printed by nothing, read by people.

*field, declared at [`include/shulib/localization/wall_reset.hpp:236`](../../include/shulib/localization/wall_reset.hpp#L236).*

<a id="struct-wallresetlimits"></a>

## `struct WallResetLimits`

```cpp
struct WallResetLimits
```

The sensor-physics and geometry gates — PROVISIONAL library defaults (A4: HA-211), overridable per robot. The ONE policy number, maxCorrection, is not here: the author states it in every request.

*struct, declared at [`include/shulib/localization/wall_reset.hpp:241`](../../include/shulib/localization/wall_reset.hpp#L241).*

<a id="wallresetlimits-rangemin"></a>

### `WallResetLimits::rangeMin`

```cpp
units::Length rangeMin{2.0}
```

The shortest bias-corrected reading accepted (inches): VEX's 20-mm floor with room for the unpublished close-range behaviour. PROVISIONAL (A4: HA-211).

*field, declared at [`include/shulib/localization/wall_reset.hpp:244`](../../include/shulib/localization/wall_reset.hpp#L244).*

<a id="wallresetlimits-rangemax"></a>

### `WallResetLimits::rangeMax`

```cpp
units::Length rangeMax{16.0}
```

The longest (inches): where the published accuracy alone (5 % = 0.8 in) leaves no room under the 1.0-in precision gate. PROVISIONAL (A4: HA-211).

*field, declared at [`include/shulib/localization/wall_reset.hpp:247`](../../include/shulib/localization/wall_reset.hpp#L247).*

<a id="wallresetlimits-squarewithin"></a>

### `WallResetLimits::squareWithin`

```cpp
units::AngleDim squareWithin{5.0 * kWallDegToRad}
```

The beam's largest angle off the wall's normal (rule 2). PROVISIONAL (A4: HA-211).

*field, declared at [`include/shulib/localization/wall_reset.hpp:249`](../../include/shulib/localization/wall_reset.hpp#L249).*

<a id="wallresetlimits-spreadmax"></a>

### `WallResetLimits::spreadMax`

```cpp
units::Length spreadMax{0.5}
```

The window's largest max - min reading (rule 7). PROVISIONAL (A4: HA-211).

*field, declared at [`include/shulib/localization/wall_reset.hpp:251`](../../include/shulib/localization/wall_reset.hpp#L251).*

<a id="wallresetlimits-minsamples"></a>

### `WallResetLimits::minSamples`

```cpp
int minSamples = 5
```

The fewest samples a window needs (rule 7); at most kMaxWallSamples. PROVISIONAL (A4: HA-211).

*field, declared at [`include/shulib/localization/wall_reset.hpp:253`](../../include/shulib/localization/wall_reset.hpp#L253).*

<a id="wallresetlimits-window"></a>

### `WallResetLimits::window`

```cpp
units::Time window{0.15}
```

The shortest window (rule 7) — the TIME is the real guard: reads inside one sensor update repeat (A4 register HA-209). Also the span rule 5 measures the speed over. Above 0 and at most kMaxWallWindowSeconds. PROVISIONAL (A4: HA-211).

*field, declared at [`include/shulib/localization/wall_reset.hpp:257`](../../include/shulib/localization/wall_reset.hpp#L257).*

<a id="wallresetlimits-windowmax"></a>

### `WallResetLimits::windowMax`

```cpp
units::Time windowMax{0.5}
```

The longest a window may run — the explicit form's frozen-wait lateness bound (at most 1 s). PROVISIONAL (A4: HA-211).

*field, declared at [`include/shulib/localization/wall_reset.hpp:260`](../../include/shulib/localization/wall_reset.hpp#L260).*

<a id="wallresetlimits-minconfidence"></a>

### `WallResetLimits::minConfidence`

```cpp
double minConfidence = 0.5
```

The lowest confidence a sample may carry (rule 6). Synthetic below 200 mm: the adapter reports 1.0 there. PROVISIONAL (A4: HA-211).

*field, declared at [`include/shulib/localization/wall_reset.hpp:263`](../../include/shulib/localization/wall_reset.hpp#L263).*

<a id="wallresetlimits-stillspeed"></a>

### `WallResetLimits::stillSpeed`

```cpp
units::Velocity stillSpeed{1.0}
```

The fastest the estimate may move during the window (rule 5): its displacement over the trailing `window`, divided by that time — never one tick's finite difference, which turns a still IMU's heading noise into phantom speed on offset tracking wheels. PROVISIONAL (A4: HA-211).

*field, declared at [`include/shulib/localization/wall_reset.hpp:268`](../../include/shulib/localization/wall_reset.hpp#L268).*

<a id="wallresetlimits-stillturn"></a>

### `WallResetLimits::stillTurn`

```cpp
units::AngularVelocity stillTurn{3.0 * kWallDegToRad}
```

The fastest it may turn during the window (rule 5), on every tick (the IMU's own rate). PROVISIONAL (A4: HA-211).

*field, declared at [`include/shulib/localization/wall_reset.hpp:271`](../../include/shulib/localization/wall_reset.hpp#L271).*

<a id="wallresetlimits-errorboundmax"></a>

### `WallResetLimits::errorBoundMax`

```cpp
units::Length errorBoundMax{1.0}
```

The largest worst-case error of the answer (rule 8) — F2's ~1.0-in pose target. PROVISIONAL (A4: HA-211).

*field, declared at [`include/shulib/localization/wall_reset.hpp:274`](../../include/shulib/localization/wall_reset.hpp#L274).*

<a id="wallresetlimits-keepoutmargin"></a>

### `WallResetLimits::keepOutMargin`

```cpp
units::Length keepOutMargin{3.0}
```

The keep-out margin (rule 3): start-pose placement + a field element's place + the beam centre's aim, summed (~2.1) and rounded up. PROVISIONAL (A4: HA-211).

*field, declared at [`include/shulib/localization/wall_reset.hpp:277`](../../include/shulib/localization/wall_reset.hpp#L277).*

<a id="wallresetlimits-alongdriftrate"></a>

### `WallResetLimits::alongDriftRate`

```cpp
double alongDriftRate = 0.10
```

The along-wall drift rate (rule 3): the keep-outs widen by this times the travel since the along axis was last set (robot two's saved scale reads ~8 % short). PROVISIONAL (A4: HA-211).

*field, declared at [`include/shulib/localization/wall_reset.hpp:280`](../../include/shulib/localization/wall_reset.hpp#L280).*

<a id="wallresetlimits-conehalfangle"></a>

### `WallResetLimits::coneHalfAngle`

```cpp
units::AngleDim coneHalfAngle{18.0 * kWallDegToRad}
```

The beam half-angle while the mount's is unmeasured (rules 3, 4) — an unofficial figure (A4 register HA-205). PROVISIONAL (A4: HA-211).

*field, declared at [`include/shulib/localization/wall_reset.hpp:283`](../../include/shulib/localization/wall_reset.hpp#L283).*

<a id="wallresetlimits-piecetop"></a>

### `WallResetLimits::pieceTop`

```cpp
units::Length pieceTop{6.5}
```

The tallest single game piece the ceiling keeps the cone above (inches): a standing Pin or Cup (A4 register HA-212). PROVISIONAL (A4: HA-211).

*field, declared at [`include/shulib/localization/wall_reset.hpp:286`](../../include/shulib/localization/wall_reset.hpp#L286).*

<a id="wallresetlimits-beammargin"></a>

### `WallResetLimits::beamMargin`

```cpp
units::Length beamMargin{0.5}
```

The clearance above that piece (inches). PROVISIONAL (A4: HA-211).

*field, declared at [`include/shulib/localization/wall_reset.hpp:288`](../../include/shulib/localization/wall_reset.hpp#L288).*

<a id="wallresetlimits-headingfloor"></a>

### `WallResetLimits::headingFloor`

```cpp
units::AngleDim headingFloor{2.0 * kWallDegToRad}
```

The heading tolerance's floor (rule 8): HA-20's drift over a run, doubled. PROVISIONAL (A4: HA-211).

*field, declared at [`include/shulib/localization/wall_reset.hpp:291`](../../include/shulib/localization/wall_reset.hpp#L291).*

<a id="wallresetlimits-headingperturn"></a>

### `WallResetLimits::headingPerTurn`

```cpp
double headingPerTurn = 0.02
```

The heading tolerance's growth per radian turned since the heading was set: HA-84's 1 %, doubled. PROVISIONAL (A4: HA-211).

*field, declared at [`include/shulib/localization/wall_reset.hpp:294`](../../include/shulib/localization/wall_reset.hpp#L294).*

<a id="struct-wallresetrequest"></a>

## `struct WallResetRequest`

```cpp
struct WallResetRequest
```

One reset, as an auton states it: the wall, the side, how far it may move the coordinate, and whether a refusal fails the step.

*struct, declared at [`include/shulib/localization/wall_reset.hpp:299`](../../include/shulib/localization/wall_reset.hpp#L299).*

<a id="wallresetrequest-wall"></a>

### `WallResetRequest::wall`

```cpp
FieldWall wall{}
```

The wall to reset against — a season wall (`localization::override2026::kLeftWall`, ...) or an author's own line (`FieldWall::line`).

*field, declared at [`include/shulib/localization/wall_reset.hpp:302`](../../include/shulib/localization/wall_reset.hpp#L302).*

<a id="wallresetrequest-sensor"></a>

### `WallResetRequest::sensor`

```cpp
WallSide sensor = WallSide::Left
```

Which side's sensor faces it.

*field, declared at [`include/shulib/localization/wall_reset.hpp:304`](../../include/shulib/localization/wall_reset.hpp#L304).*

<a id="wallresetrequest-maxcorrection"></a>

### `WallResetRequest::maxCorrection`

```cpp
units::Length maxCorrection{0.0}
```

How far this wall may move the coordinate (inches). NO DEFAULT: 0 is UNSET and refused by name (`maxCorrection UNSET: say how far this wall may move x`) — the largest drift the author has taped at this point plus ~1 in, never near the gap to anything a wrong seat could mistake for the wall.

*field, declared at [`include/shulib/localization/wall_reset.hpp:309`](../../include/shulib/localization/wall_reset.hpp#L309).*

<a id="wallresetrequest-required"></a>

### `WallResetRequest::required`

```cpp
bool required = false
```

True: a refusal FAILS the step (through `then()` the chain stops). False: a refusal leaves the estimate as it was and the auton goes on.

*field, declared at [`include/shulib/localization/wall_reset.hpp:312`](../../include/shulib/localization/wall_reset.hpp#L312).*

<a id="wallresetrequest-name"></a>

### `WallResetRequest::name`

```cpp
const char* name = nullptr
```

Optional: leads the log line when set — a stable literal.

*field, declared at [`include/shulib/localization/wall_reset.hpp:314`](../../include/shulib/localization/wall_reset.hpp#L314).*

<a id="struct-wallsample"></a>

## `struct WallSample`

```cpp
struct WallSample
```

One sample of the window, as the sampler took it.

*struct, declared at [`include/shulib/localization/wall_reset.hpp:318`](../../include/shulib/localization/wall_reset.hpp#L318).*

<a id="wallsample-distance"></a>

### `WallSample::distance`

```cpp
units::Length distance{}
```

The sensor's distance() reading — RAW (the mount's bias is removed by the gates, never here).

*field, declared at [`include/shulib/localization/wall_reset.hpp:320`](../../include/shulib/localization/wall_reset.hpp#L320).*

<a id="wallsample-confidence"></a>

### `WallSample::confidence`

```cpp
double confidence = 0.0
```

The sensor's confidence() reading, in [0, 1].

*field, declared at [`include/shulib/localization/wall_reset.hpp:322`](../../include/shulib/localization/wall_reset.hpp#L322).*

<a id="wallsample-faultmoved"></a>

### `WallSample::faultMoved`

```cpp
bool faultMoved = false
```

The read-fault count moved across the sample's two device reads: the values may be HELD (a faulted port's adapter returns its last good reading), not measured — never evidence.

*field, declared at [`include/shulib/localization/wall_reset.hpp:325`](../../include/shulib/localization/wall_reset.hpp#L325).*

<a id="wallsample-at"></a>

### `WallSample::at`

```cpp
units::Time at{}
```

When it was taken, from the window's first sample.

*field, declared at [`include/shulib/localization/wall_reset.hpp:327`](../../include/shulib/localization/wall_reset.hpp#L327).*

<a id="kmaxwallsamples"></a>

## `kMaxWallSamples`

```cpp
inline constexpr std::size_t kMaxWallSamples = 64
```

The window's sample buffer: fixed, allocation-free. At the 10-ms tick the 0.5-s windowMax is 50 samples; a faster pacer fills it and the sampler STOPS at capacity — it never wraps or overruns.

*constant, declared at [`include/shulib/localization/wall_reset.hpp:332`](../../include/shulib/localization/wall_reset.hpp#L332).*

<a id="kmaxwallwindowseconds"></a>

## `kMaxWallWindowSeconds`

```cpp
inline constexpr double kMaxWallWindowSeconds = 0.6
```

The longest `WallResetLimits::window` a reset accepts (seconds). The 64-sample buffer spans 63 ticks — 0.63 s at the project's 10-ms control tick — and a `window` longer than the buffer can span could never be met: every reset would end TOO FEW SAMPLES. 0.6 s keeps a tick of slack; past it the request is refused BadRequest, naming the field (found by the L6-A1 review).

*constant, declared at [`include/shulib/localization/wall_reset.hpp:338`](../../include/shulib/localization/wall_reset.hpp#L338).*

<a id="enum-class-wallresetverdict"></a>

## `enum class WallResetVerdict`

```cpp
enum class WallResetVerdict : std::uint8_t
```

A reset's verdict. APPEND-ONLY from L6-A1's landing (2026-10-29): a value is never renumbered or reused. `wallResetVerdictWord` gives each its word; guide 11 gives each its remedy.

*enum class, declared at [`include/shulib/localization/wall_reset.hpp:342`](../../include/shulib/localization/wall_reset.hpp#L342).*

<a id="wallresetverdict-seated"></a>

### `WallResetVerdict::Seated`

```cpp
Seated
```

The axis was re-seated.

*enumerator, declared at [`include/shulib/localization/wall_reset.hpp:344`](../../include/shulib/localization/wall_reset.hpp#L344).*

<a id="wallresetverdict-nosensor"></a>

### `WallResetVerdict::NoSensor`

```cpp
NoSensor
```

The robot declares no wall sensors (the everyday form's context has no slot — L6-A2).

*enumerator, declared at [`include/shulib/localization/wall_reset.hpp:346`](../../include/shulib/localization/wall_reset.hpp#L346).*

<a id="wallresetverdict-sensorabsent"></a>

### `WallResetVerdict::SensorAbsent`

```cpp
SensorAbsent
```

The named side's sensor does not exist (the provider refused it); the detail is the reason.

*enumerator, declared at [`include/shulib/localization/wall_reset.hpp:348`](../../include/shulib/localization/wall_reset.hpp#L348).*

<a id="wallresetverdict-mountunset"></a>

### `WallResetVerdict::MountUnset`

```cpp
MountUnset
```

The side's mount was never measured (`SideMount::measured` false).

*enumerator, declared at [`include/shulib/localization/wall_reset.hpp:350`](../../include/shulib/localization/wall_reset.hpp#L350).*

<a id="wallresetverdict-notlive"></a>

### `WallResetVerdict::NotLive`

```cpp
NotLive
```

The estimate is not live yet (the IMU still booting).

*enumerator, declared at [`include/shulib/localization/wall_reset.hpp:352`](../../include/shulib/localization/wall_reset.hpp#L352).*

<a id="wallresetverdict-motionactive"></a>

### `WallResetVerdict::MotionActive`

```cpp
MotionActive
```

A motion is still armed (a Tier-3 `async()`): the reset never moves the estimate under one.

*enumerator, declared at [`include/shulib/localization/wall_reset.hpp:354`](../../include/shulib/localization/wall_reset.hpp#L354).*

<a id="wallresetverdict-badrequest"></a>

### `WallResetVerdict::BadRequest`

```cpp
BadRequest
```

A request, mount or limit field is unusable; the detail names it.

*enumerator, declared at [`include/shulib/localization/wall_reset.hpp:356`](../../include/shulib/localization/wall_reset.hpp#L356).*

<a id="wallresetverdict-notfieldstart"></a>

### `WallResetVerdict::NotFieldStart`

```cpp
NotFieldStart
```

The auton's start is not a FIELD start (the everyday form — L6-A2).

*enumerator, declared at [`include/shulib/localization/wall_reset.hpp:358`](../../include/shulib/localization/wall_reset.hpp#L358).*

<a id="wallresetverdict-notsquare"></a>

### `WallResetVerdict::NotSquare`

```cpp
NotSquare
```

The beam is not within `squareWithin` of the wall's normal (or faces away from it).

*enumerator, declared at [`include/shulib/localization/wall_reset.hpp:360`](../../include/shulib/localization/wall_reset.hpp#L360).*

<a id="wallresetverdict-keepout"></a>

### `WallResetVerdict::KeepOut`

```cpp
KeepOut
```

The beam lands on (or its widened footprint overlaps) a keep-out span; the detail names it.

*enumerator, declared at [`include/shulib/localization/wall_reset.hpp:362`](../../include/shulib/localization/wall_reset.hpp#L362).*

<a id="wallresetverdict-beamlow"></a>

### `WallResetVerdict::BeamLow`

```cpp
BeamLow
```

The expected range, or the median's, is past the beam-height ceiling.

*enumerator, declared at [`include/shulib/localization/wall_reset.hpp:364`](../../include/shulib/localization/wall_reset.hpp#L364).*

<a id="wallresetverdict-moving"></a>

### `WallResetVerdict::Moving`

```cpp
Moving
```

The estimate moved or turned during the window.

*enumerator, declared at [`include/shulib/localization/wall_reset.hpp:366`](../../include/shulib/localization/wall_reset.hpp#L366).*

<a id="wallresetverdict-readfault"></a>

### `WallResetVerdict::ReadFault`

```cpp
ReadFault
```

A sample was read while the port faulted.

*enumerator, declared at [`include/shulib/localization/wall_reset.hpp:368`](../../include/shulib/localization/wall_reset.hpp#L368).*

<a id="wallresetverdict-noreturn"></a>

### `WallResetVerdict::NoReturn`

```cpp
NoReturn
```

A sample had no return (confidence 0 — PROS's 9999, or a held value).

*enumerator, declared at [`include/shulib/localization/wall_reset.hpp:370`](../../include/shulib/localization/wall_reset.hpp#L370).*

<a id="wallresetverdict-lowconfidence"></a>

### `WallResetVerdict::LowConfidence`

```cpp
LowConfidence
```

A sample's confidence was under `minConfidence`.

*enumerator, declared at [`include/shulib/localization/wall_reset.hpp:372`](../../include/shulib/localization/wall_reset.hpp#L372).*

<a id="wallresetverdict-outofrange"></a>

### `WallResetVerdict::OutOfRange`

```cpp
OutOfRange
```

A sample's bias-corrected range was outside [rangeMin, rangeMax].

*enumerator, declared at [`include/shulib/localization/wall_reset.hpp:374`](../../include/shulib/localization/wall_reset.hpp#L374).*

<a id="wallresetverdict-toofewsamples"></a>

### `WallResetVerdict::TooFewSamples`

```cpp
TooFewSamples
```

The window had fewer than `minSamples`, or ran shorter than `window`.

*enumerator, declared at [`include/shulib/localization/wall_reset.hpp:376`](../../include/shulib/localization/wall_reset.hpp#L376).*

<a id="wallresetverdict-spread"></a>

### `WallResetVerdict::Spread`

```cpp
Spread
```

The window's readings spread more than `spreadMax`.

*enumerator, declared at [`include/shulib/localization/wall_reset.hpp:378`](../../include/shulib/localization/wall_reset.hpp#L378).*

<a id="wallresetverdict-imprecise"></a>

### `WallResetVerdict::Imprecise`

```cpp
Imprecise
```

The answer's worst-case error is over `errorBoundMax`.

*enumerator, declared at [`include/shulib/localization/wall_reset.hpp:380`](../../include/shulib/localization/wall_reset.hpp#L380).*

<a id="wallresetverdict-disagrees"></a>

### `WallResetVerdict::Disagrees`

```cpp
Disagrees
```

The answer differs from the estimate by more than `maxCorrection`.

*enumerator, declared at [`include/shulib/localization/wall_reset.hpp:382`](../../include/shulib/localization/wall_reset.hpp#L382).*

<a id="wallresetverdict-cut"></a>

### `WallResetVerdict::Cut`

```cpp
Cut
```

The run's deadline or a disable landed in the window (the everyday form — L6-A2).

*enumerator, declared at [`include/shulib/localization/wall_reset.hpp:384`](../../include/shulib/localization/wall_reset.hpp#L384).*

<a id="wallresetverdictword"></a>

## `wallResetVerdictWord`

```cpp
[[nodiscard]] constexpr const char* wallResetVerdictWord(WallResetVerdict v) noexcept
```

The verdict's word in the log line ("SEATED", "REFUSED keep-out", ...). Exhaustive with no `default:` — a new verdict without a word does not compile (-Wswitch -Werror).

*free function, declared at [`include/shulib/localization/wall_reset.hpp:389`](../../include/shulib/localization/wall_reset.hpp#L389).*

<a id="struct-wallresetreport"></a>

## `struct WallResetReport`

```cpp
struct WallResetReport
```

What a reset did, with every number its log line prints. A VALUE: a refusal is a report, never an exception. Fields a verdict did not reach stay at their defaults.

*struct, declared at [`include/shulib/localization/wall_reset.hpp:439`](../../include/shulib/localization/wall_reset.hpp#L439).*

<a id="wallresetreport-verdict"></a>

### `WallResetReport::verdict`

```cpp
WallResetVerdict verdict = WallResetVerdict::NoSensor
```

The verdict (the FIRST rule that failed, or Seated).

*field, declared at [`include/shulib/localization/wall_reset.hpp:441`](../../include/shulib/localization/wall_reset.hpp#L441).*

<a id="wallresetreport-axis"></a>

### `WallResetReport::axis`

```cpp
Axis axis = Axis::X
```

The axis the wall measures.

*field, declared at [`include/shulib/localization/wall_reset.hpp:443`](../../include/shulib/localization/wall_reset.hpp#L443).*

<a id="wallresetreport-side"></a>

### `WallResetReport::side`

```cpp
WallSide side = WallSide::Left
```

The side whose sensor was read.

*field, declared at [`include/shulib/localization/wall_reset.hpp:445`](../../include/shulib/localization/wall_reset.hpp#L445).*

<a id="wallresetreport-required"></a>

### `WallResetReport::required`

```cpp
bool required = false
```

The request's `required` (D3): a refusal then fails the step.

*field, declared at [`include/shulib/localization/wall_reset.hpp:447`](../../include/shulib/localization/wall_reset.hpp#L447).*

<a id="wallresetreport-before"></a>

### `WallResetReport::before`

```cpp
units::Length before{}
```

The estimate's coordinate on `axis` when the reset judged it.

*field, declared at [`include/shulib/localization/wall_reset.hpp:449`](../../include/shulib/localization/wall_reset.hpp#L449).*

<a id="wallresetreport-measured"></a>

### `WallResetReport::measured`

```cpp
units::Length measured{}
```

The wall's answer for that coordinate (filled once a median exists).

*field, declared at [`include/shulib/localization/wall_reset.hpp:451`](../../include/shulib/localization/wall_reset.hpp#L451).*

<a id="wallresetreport-applied"></a>

### `WallResetReport::applied`

```cpp
units::Length applied{}
```

How far the coordinate was moved: measured - before when SEATED, 0 otherwise.

*field, declared at [`include/shulib/localization/wall_reset.hpp:453`](../../include/shulib/localization/wall_reset.hpp#L453).*

<a id="wallresetreport-maxcorrection"></a>

### `WallResetReport::maxCorrection`

```cpp
units::Length maxCorrection{}
```

The request's maxCorrection, echoed for the line.

*field, declared at [`include/shulib/localization/wall_reset.hpp:455`](../../include/shulib/localization/wall_reset.hpp#L455).*

<a id="wallresetreport-median"></a>

### `WallResetReport::median`

```cpp
units::Length median{}
```

The window's MEDIAN reading — raw, as the sensor gave it.

*field, declared at [`include/shulib/localization/wall_reset.hpp:457`](../../include/shulib/localization/wall_reset.hpp#L457).*

<a id="wallresetreport-spread"></a>

### `WallResetReport::spread`

```cpp
units::Length spread{}
```

The window's max - min reading.

*field, declared at [`include/shulib/localization/wall_reset.hpp:459`](../../include/shulib/localization/wall_reset.hpp#L459).*

<a id="wallresetreport-samples"></a>

### `WallResetReport::samples`

```cpp
int samples = 0
```

How many samples the window took.

*field, declared at [`include/shulib/localization/wall_reset.hpp:461`](../../include/shulib/localization/wall_reset.hpp#L461).*

<a id="wallresetreport-windowspan"></a>

### `WallResetReport::windowSpan`

```cpp
units::Time windowSpan{}
```

How long the window ran (first sample to last).

*field, declared at [`include/shulib/localization/wall_reset.hpp:463`](../../include/shulib/localization/wall_reset.hpp#L463).*

<a id="wallresetreport-offsquare"></a>

### `WallResetReport::offSquare`

```cpp
units::AngleDim offSquare{}
```

The beam's angle off the wall's normal, by the estimate's heading (rule 2).

*field, declared at [`include/shulib/localization/wall_reset.hpp:465`](../../include/shulib/localization/wall_reset.hpp#L465).*

<a id="wallresetreport-expectedrange"></a>

### `WallResetReport::expectedRange`

```cpp
units::Length expectedRange{}
```

The beam-centre range to the wall the ESTIMATE implies (rules 3, 4).

*field, declared at [`include/shulib/localization/wall_reset.hpp:467`](../../include/shulib/localization/wall_reset.hpp#L467).*

<a id="wallresetreport-along"></a>

### `WallResetReport::along`

```cpp
units::Length along{}
```

The beam centre's coordinate along the wall, by the estimate (rule 3).

*field, declared at [`include/shulib/localization/wall_reset.hpp:469`](../../include/shulib/localization/wall_reset.hpp#L469).*

<a id="wallresetreport-beamhalfwidth"></a>

### `WallResetReport::beamHalfWidth`

```cpp
units::Length beamHalfWidth{}
```

The beam's footprint half-width along the wall at the expected range (rule 3).

*field, declared at [`include/shulib/localization/wall_reset.hpp:471`](../../include/shulib/localization/wall_reset.hpp#L471).*

<a id="wallresetreport-keepoutmargin"></a>

### `WallResetReport::keepOutMargin`

```cpp
units::Length keepOutMargin{}
```

The keep-out margin used (rule 3).

*field, declared at [`include/shulib/localization/wall_reset.hpp:473`](../../include/shulib/localization/wall_reset.hpp#L473).*

<a id="wallresetreport-alonguncertainty"></a>

### `WallResetReport::alongUncertainty`

```cpp
units::Length alongUncertainty{}
```

The along-wall drift allowance used: alongDriftRate x travelSinceSeat (rule 3).

*field, declared at [`include/shulib/localization/wall_reset.hpp:475`](../../include/shulib/localization/wall_reset.hpp#L475).*

<a id="wallresetreport-cone"></a>

### `WallResetReport::cone`

```cpp
units::AngleDim cone{}
```

The beam half-angle used (the mount's measured one, or the limits').

*field, declared at [`include/shulib/localization/wall_reset.hpp:477`](../../include/shulib/localization/wall_reset.hpp#L477).*

<a id="wallresetreport-ceiling"></a>

### `WallResetReport::ceiling`

```cpp
units::Length ceiling{}
```

The beam-height ceiling (rule 4).

*field, declared at [`include/shulib/localization/wall_reset.hpp:479`](../../include/shulib/localization/wall_reset.hpp#L479).*

<a id="wallresetreport-headingtolerance"></a>

### `WallResetReport::headingTolerance`

```cpp
units::AngleDim headingTolerance{}
```

The heading tolerance used by the precision gate (rule 8).

*field, declared at [`include/shulib/localization/wall_reset.hpp:481`](../../include/shulib/localization/wall_reset.hpp#L481).*

<a id="wallresetreport-errorbound"></a>

### `WallResetReport::errorBound`

```cpp
units::Length errorBound{}
```

The answer's worst-case error (rule 8; filled once a median exists).

*field, declared at [`include/shulib/localization/wall_reset.hpp:483`](../../include/shulib/localization/wall_reset.hpp#L483).*

<a id="wallresetreport-speedseen"></a>

### `WallResetReport::speedSeen`

```cpp
units::Velocity speedSeen{}
```

The fastest the estimate moved during the window.

*field, declared at [`include/shulib/localization/wall_reset.hpp:485`](../../include/shulib/localization/wall_reset.hpp#L485).*

<a id="wallresetreport-turnseen"></a>

### `WallResetReport::turnSeen`

```cpp
units::AngularVelocity turnSeen{}
```

The fastest it turned during the window.

*field, declared at [`include/shulib/localization/wall_reset.hpp:487`](../../include/shulib/localization/wall_reset.hpp#L487).*

<a id="wallresetreport-sampleindex"></a>

### `WallResetReport::sampleIndex`

```cpp
int sampleIndex = -1
```

The refusing sample's index (rule 6), or -1.

*field, declared at [`include/shulib/localization/wall_reset.hpp:489`](../../include/shulib/localization/wall_reset.hpp#L489).*

<a id="wallresetreport-sampledistance"></a>

### `WallResetReport::sampleDistance`

```cpp
units::Length sampleDistance{}
```

The refusing sample's distance reading (rule 6).

*field, declared at [`include/shulib/localization/wall_reset.hpp:491`](../../include/shulib/localization/wall_reset.hpp#L491).*

<a id="wallresetreport-sampleconfidence"></a>

### `WallResetReport::sampleConfidence`

```cpp
double sampleConfidence = 0.0
```

The refusing sample's confidence (rule 6).

*field, declared at [`include/shulib/localization/wall_reset.hpp:493`](../../include/shulib/localization/wall_reset.hpp#L493).*

<a id="wallresetreport-detail"></a>

### `WallResetReport::detail`

```cpp
const char* detail = ""
```

BadRequest: the field's message; KeepOut: the span's name; SensorAbsent: the absence sentence. A stable literal (or the Robot's copied sentence), "" when none.

*field, declared at [`include/shulib/localization/wall_reset.hpp:496`](../../include/shulib/localization/wall_reset.hpp#L496).*

<a id="wallresetreport-cleanfound"></a>

### `WallResetReport::cleanFound`

```cpp
bool cleanFound = false
```

KeepOut: a clean band for the beam centre was found along the wall.

*field, declared at [`include/shulib/localization/wall_reset.hpp:498`](../../include/shulib/localization/wall_reset.hpp#L498).*

<a id="wallresetreport-cleanfrom"></a>

### `WallResetReport::cleanFrom`

```cpp
units::Length cleanFrom{}
```

KeepOut: the nearest clean band's lower end along the wall.

*field, declared at [`include/shulib/localization/wall_reset.hpp:500`](../../include/shulib/localization/wall_reset.hpp#L500).*

<a id="wallresetreport-cleanto"></a>

### `WallResetReport::cleanTo`

```cpp
units::Length cleanTo{}
```

KeepOut: the nearest clean band's upper end along the wall.

*field, declared at [`include/shulib/localization/wall_reset.hpp:502`](../../include/shulib/localization/wall_reset.hpp#L502).*

<a id="wallresetreport-faultchannel"></a>

### `WallResetReport::faultChannel`

```cpp
bool faultChannel = true
```

False: the sensor has no read-fault count ("faults n/a" — never "faults ok").

*field, declared at [`include/shulib/localization/wall_reset.hpp:504`](../../include/shulib/localization/wall_reset.hpp#L504).*

<a id="wallresetreport-seated"></a>

### `WallResetReport::seated`

```cpp
[[nodiscard]] constexpr bool seated() const noexcept
```

The axis was re-seated.

*function, declared at [`include/shulib/localization/wall_reset.hpp:507`](../../include/shulib/localization/wall_reset.hpp#L507).*

<a id="wallresetreport-continues"></a>

### `WallResetReport::continues`

```cpp
[[nodiscard]] constexpr bool continues() const noexcept
```

The auton may go on: SEATED, or REFUSED and not `required` (D3) — the value a `then()` returns.

*function, declared at [`include/shulib/localization/wall_reset.hpp:509`](../../include/shulib/localization/wall_reset.hpp#L509).*

<a id="struct-wallsensor"></a>

## `struct WallSensor`

```cpp
struct WallSensor
```

One side as a reset reads it: the sensor, its read-fault count, its mount, and why it is absent. Built by the caller (the explicit form) or by the Robot (from L6-A2).

*struct, declared at [`include/shulib/localization/wall_reset.hpp:514`](../../include/shulib/localization/wall_reset.hpp#L514).*

<a id="wallsensor-sensor"></a>

### `WallSensor::sensor`

```cpp
hal::IDistance* sensor = nullptr
```

The sensor; nullptr = absent (the reset refuses SensorAbsent with `absence`).

*field, declared at [`include/shulib/localization/wall_reset.hpp:516`](../../include/shulib/localization/wall_reset.hpp#L516).*

<a id="wallsensor-faults"></a>

### `WallSensor::faults`

```cpp
const hal::IReadFaults* faults = nullptr
```

Its read-fault count; nullptr = none (a fake, or a provider that never screens) — accepted, and the line says "faults n/a".

*field, declared at [`include/shulib/localization/wall_reset.hpp:519`](../../include/shulib/localization/wall_reset.hpp#L519).*

<a id="wallsensor-mount"></a>

### `WallSensor::mount`

```cpp
SideMount mount{}
```

Its measured mount.

*field, declared at [`include/shulib/localization/wall_reset.hpp:521`](../../include/shulib/localization/wall_reset.hpp#L521).*

<a id="wallsensor-absence"></a>

### `WallSensor::absence`

```cpp
const char* absence = "not declared"
```

Why it is absent, when `sensor` is nullptr — a stable literal or the Robot's copied sentence.

*field, declared at [`include/shulib/localization/wall_reset.hpp:523`](../../include/shulib/localization/wall_reset.hpp#L523).*

<a id="struct-wallresetlog"></a>

## `struct WallResetLog`

```cpp
struct WallResetLog
```

A robot's running tally of its resets (the Auton Tester shows it to the people testing, from L6-B) — counts and the last outcome. `record()` is how a call files one.

*struct, declared at [`include/shulib/localization/wall_reset.hpp:528`](../../include/shulib/localization/wall_reset.hpp#L528).*

<a id="wallresetlog-seated"></a>

### `WallResetLog::seated`

```cpp
int seated = 0
```

Resets SEATED so far.

*field, declared at [`include/shulib/localization/wall_reset.hpp:530`](../../include/shulib/localization/wall_reset.hpp#L530).*

<a id="wallresetlog-refused"></a>

### `WallResetLog::refused`

```cpp
int refused = 0
```

Resets REFUSED so far.

*field, declared at [`include/shulib/localization/wall_reset.hpp:532`](../../include/shulib/localization/wall_reset.hpp#L532).*

<a id="wallresetlog-last"></a>

### `WallResetLog::last`

```cpp
WallResetReport last{}
```

The last outcome (a default report until the first).

*field, declared at [`include/shulib/localization/wall_reset.hpp:534`](../../include/shulib/localization/wall_reset.hpp#L534).*

<a id="wallresetlog-record"></a>

### `WallResetLog::record`

```cpp
void record(const WallResetReport& report) noexcept
```

File one outcome: counted SEATED or REFUSED, and kept as `last`.

*function, declared at [`include/shulib/localization/wall_reset.hpp:537`](../../include/shulib/localization/wall_reset.hpp#L537).*

<a id="struct-wallsensors"></a>

## `struct WallSensors`

```cpp
struct WallSensors
```

Both sides, the robot's limits and its log — the slot the robot's context carries from L6-A2.

*struct, declared at [`include/shulib/localization/wall_reset.hpp:548`](../../include/shulib/localization/wall_reset.hpp#L548).*

<a id="wallsensors-left"></a>

### `WallSensors::left`

```cpp
WallSensor left{}
```

The LEFT side's sensor.

*field, declared at [`include/shulib/localization/wall_reset.hpp:550`](../../include/shulib/localization/wall_reset.hpp#L550).*

<a id="wallsensors-right"></a>

### `WallSensors::right`

```cpp
WallSensor right{}
```

The RIGHT side's sensor.

*field, declared at [`include/shulib/localization/wall_reset.hpp:552`](../../include/shulib/localization/wall_reset.hpp#L552).*

<a id="wallsensors-limits"></a>

### `WallSensors::limits`

```cpp
WallResetLimits limits{}
```

The robot's gate limits.

*field, declared at [`include/shulib/localization/wall_reset.hpp:554`](../../include/shulib/localization/wall_reset.hpp#L554).*

<a id="wallsensors-log"></a>

### `WallSensors::log`

```cpp
WallResetLog log{}
```

The robot's running tally.

*field, declared at [`include/shulib/localization/wall_reset.hpp:556`](../../include/shulib/localization/wall_reset.hpp#L556).*

<a id="wallsensors-side"></a>

### `WallSensors::side`

```cpp
[[nodiscard]] constexpr const WallSensor& side(WallSide which) const noexcept
```

The named side.

*function, declared at [`include/shulib/localization/wall_reset.hpp:559`](../../include/shulib/localization/wall_reset.hpp#L559).*

<a id="readingaccuracy"></a>

## `readingAccuracy`

```cpp
[[nodiscard]] inline units::Length readingAccuracy(units::Length range, const SideMount& mount) noexcept
```

PURE: the accuracy the reset assumes for a bias-corrected RANGE: the mount's measured `rangeAccuracy` when typed, else VEX's published max(15 mm, 5 % of the range) — at every range, never a step at 200 mm. A4 register HA-204.

*free function, declared at [`include/shulib/localization/wall_reset.hpp:594`](../../include/shulib/localization/wall_reset.hpp#L594).*

<a id="axisfromreading"></a>

## `axisFromReading`

```cpp
[[nodiscard]] inline units::Length axisFromReading(units::Length reading, const math::Pose2d& pose, const SideMount& mount, const FieldWall& wall)
```

PURE: the robot centre's coordinate a READING implies (the beam-centre model; the mount's bias removed). `reading` is the sensor's raw distance and must be finite (a precondition).

*free function, declared at [`include/shulib/localization/wall_reset.hpp:604`](../../include/shulib/localization/wall_reset.hpp#L604).*

<a id="axiserrorbound"></a>

## `axisErrorBound`

```cpp
[[nodiscard]] inline units::Length axisErrorBound(units::Length reading, const math::Pose2d& pose, const SideMount& mount, const FieldWall& wall, units::AngleDim headingTolerance)
```

PURE: the worst-case error of axisFromReading's answer — over the CORNERS of the tolerance box (the reading +/- its accuracy, `forward` and `left` +/- the mount's positionTolerance, `facing` +/- its facingTolerance, the heading +/- `headingTolerance`), each corner run through the exact geometry, not a linearisation. A non-finite tolerance (a NaN typed into the mount, or handed in as `headingTolerance`) gives +infinity — the worst answer, so the precision gate refuses — never a NaN silently dropped by a max (found by the L6-A1 review). A4 register HA-204 (the accuracy), HA-211 (the tolerances).

*free function, declared at [`include/shulib/localization/wall_reset.hpp:621`](../../include/shulib/localization/wall_reset.hpp#L621).*

<a id="headingtolerance"></a>

## `headingTolerance`

```cpp
[[nodiscard]] inline units::AngleDim headingTolerance(units::AngleDim turnedSinceHeadingSeed, const WallResetLimits& limits) noexcept
```

PURE: the heading tolerance the precision gate assumes: `headingFloor` + `headingPerTurn` x the |rotation| since the heading was last seeded (`Localizer::turnedSinceHeadingSeed`). A4 register HA-211 (the floor from HA-20, the growth from HA-84).

*free function, declared at [`include/shulib/localization/wall_reset.hpp:657`](../../include/shulib/localization/wall_reset.hpp#L657).*

<a id="effectivecone"></a>

## `effectiveCone`

```cpp
[[nodiscard]] constexpr units::AngleDim effectiveCone(const SideMount& mount, const WallResetLimits& limits) noexcept
```

PURE: the beam half-angle a reset uses — the mount's MEASURED one when typed (> 0), else the limits' (A4 register HA-205).

*free function, declared at [`include/shulib/localization/wall_reset.hpp:665`](../../include/shulib/localization/wall_reset.hpp#L665).*

<a id="beamceiling"></a>

## `beamCeiling`

```cpp
[[nodiscard]] inline units::Length beamCeiling(const SideMount& mount, const WallResetLimits& limits) noexcept
```

PURE: the farthest range whose cone stays above `pieceTop` + `beamMargin` all the way to the wall, from the TYPED height and cone: (height - pieceTop - beamMargin) / tan(cone), and 0 when the lens is not above that line at all. At or under it, no single standing game piece can be the nearest surface in the cone — nor the floor, which the cone meets only beyond height / tan(cone).

*free function, declared at [`include/shulib/localization/wall_reset.hpp:674`](../../include/shulib/localization/wall_reset.hpp#L674).*

<a id="wallresetproblem"></a>

## `wallResetProblem`

```cpp
[[nodiscard]] inline const char* wallResetProblem(const SideMount& mount, const WallResetRequest& request, const WallResetLimits& limits) noexcept
```

PURE: the first thing wrong with a request, its mount or the limits, as the message the line prints ("maxCorrection UNSET: say how far this wall may move x") — or nullptr when everything is usable. Rule 1's last check (BadRequest).

*free function, declared at [`include/shulib/localization/wall_reset.hpp:685`](../../include/shulib/localization/wall_reset.hpp#L685).*

<a id="struct-wallaim"></a>

## `struct WallAim`

```cpp
struct WallAim
```

Where a beam lands, by the ESTIMATE — everything rules 2-4 judge, computed once (PURE).

*struct, declared at [`include/shulib/localization/wall_reset.hpp:778`](../../include/shulib/localization/wall_reset.hpp#L778).*

<a id="wallaim-offsquare"></a>

### `WallAim::offSquare`

```cpp
units::AngleDim offSquare{}
```

The beam's angle off the wall's outward normal, in [0, pi]: above 90 degrees it faces away.

*field, declared at [`include/shulib/localization/wall_reset.hpp:780`](../../include/shulib/localization/wall_reset.hpp#L780).*

<a id="wallaim-expectedrange"></a>

### `WallAim::expectedRange`

```cpp
units::Length expectedRange{}
```

The beam-centre range to the wall the estimate implies (<= 0 if the estimate puts the lens past the wall; only meaningful while the beam points at it).

*field, declared at [`include/shulib/localization/wall_reset.hpp:783`](../../include/shulib/localization/wall_reset.hpp#L783).*

<a id="wallaim-along"></a>

### `WallAim::along`

```cpp
units::Length along{}
```

The beam centre's coordinate along the wall.

*field, declared at [`include/shulib/localization/wall_reset.hpp:785`](../../include/shulib/localization/wall_reset.hpp#L785).*

<a id="wallaim-beamhalfwidth"></a>

### `WallAim::beamHalfWidth`

```cpp
units::Length beamHalfWidth{}
```

The footprint's half-width along the wall at the expected range: the LARGER side of the exact cone footprint (a skewed beam's footprint is lopsided), applied both ways.

*field, declared at [`include/shulib/localization/wall_reset.hpp:788`](../../include/shulib/localization/wall_reset.hpp#L788).*

<a id="wallaim-margin"></a>

### `WallAim::margin`

```cpp
units::Length margin{}
```

The keep-out margin (the limits').

*field, declared at [`include/shulib/localization/wall_reset.hpp:790`](../../include/shulib/localization/wall_reset.hpp#L790).*

<a id="wallaim-drift"></a>

### `WallAim::drift`

```cpp
units::Length drift{}
```

The along-wall drift allowance (alongDriftRate x the travel since the along axis was set).

*field, declared at [`include/shulib/localization/wall_reset.hpp:792`](../../include/shulib/localization/wall_reset.hpp#L792).*

<a id="wallaim-widening"></a>

### `WallAim::widening`

```cpp
units::Length widening{}
```

beamHalfWidth + margin + drift: how far either side of `along` a keep-out must stay clear.

*field, declared at [`include/shulib/localization/wall_reset.hpp:794`](../../include/shulib/localization/wall_reset.hpp#L794).*

<a id="wallaim-cone"></a>

### `WallAim::cone`

```cpp
units::AngleDim cone{}
```

The beam half-angle used (effectiveCone).

*field, declared at [`include/shulib/localization/wall_reset.hpp:796`](../../include/shulib/localization/wall_reset.hpp#L796).*

<a id="wallaim-ceiling"></a>

### `WallAim::ceiling`

```cpp
units::Length ceiling{}
```

The beam-height ceiling (beamCeiling).

*field, declared at [`include/shulib/localization/wall_reset.hpp:798`](../../include/shulib/localization/wall_reset.hpp#L798).*

<a id="wallaim-keepoutwhy"></a>

### `WallAim::keepOutWhy`

```cpp
const char* keepOutWhy = nullptr
```

The first ACTIVE keep-out the widened footprint overlaps (its `why`), or nullptr.

*field, declared at [`include/shulib/localization/wall_reset.hpp:800`](../../include/shulib/localization/wall_reset.hpp#L800).*

<a id="wallaim-cleanfound"></a>

### `WallAim::cleanFound`

```cpp
bool cleanFound = false
```

A clean band for the beam centre exists along the wall (only computed when `keepOutWhy` is set).

*field, declared at [`include/shulib/localization/wall_reset.hpp:802`](../../include/shulib/localization/wall_reset.hpp#L802).*

<a id="wallaim-cleanfrom"></a>

### `WallAim::cleanFrom`

```cpp
units::Length cleanFrom{}
```

The nearest clean band's lower end (may be -infinity on an author's own line).

*field, declared at [`include/shulib/localization/wall_reset.hpp:804`](../../include/shulib/localization/wall_reset.hpp#L804).*

<a id="wallaim-cleanto"></a>

### `WallAim::cleanTo`

```cpp
units::Length cleanTo{}
```

The nearest clean band's upper end (may be +infinity on an author's own line).

*field, declared at [`include/shulib/localization/wall_reset.hpp:806`](../../include/shulib/localization/wall_reset.hpp#L806).*

<a id="keepoutactive"></a>

## `keepOutActive`

```cpp
[[nodiscard]] constexpr bool keepOutActive(const WallKeepOut& span, const SideMount& mount) noexcept
```

PURE: whether a keep-out span applies for this mount (KeepOutKind's rules).

*free function, declared at [`include/shulib/localization/wall_reset.hpp:810`](../../include/shulib/localization/wall_reset.hpp#L810).*

<a id="aimat"></a>

## `aimAt`

```cpp
[[nodiscard]] inline WallAim aimAt(const math::Pose2d& pose, const SideMount& mount, const FieldWall& wall, const WallResetLimits& limits, units::Length alongUncertainty)
```

PURE: where the beam lands by the estimate `pose`, and what it would land on (rules 2-4's inputs). Requires a usable request (`wallResetProblem` returned nullptr) — with an `inward` other than +/-1 the normal is undefined.

*free function, declared at [`include/shulib/localization/wall_reset.hpp:825`](../../include/shulib/localization/wall_reset.hpp#L825).*

<a id="precheck"></a>

## `preCheck`

```cpp
[[nodiscard]] inline WallResetVerdict preCheck(const math::Pose2d& pose, const SideMount& mount, const WallResetRequest& request, const WallResetLimits& limits, units::Length alongUncertainty)
```

PURE: the gates that need no reading, in order — BadRequest (wallResetProblem), NotSquare, KeepOut, BeamLow; Seated means "go on and sample". (The call checks the sensor, the mount's `measured`, liveness and motion first.)

*free function, declared at [`include/shulib/localization/wall_reset.hpp:937`](../../include/shulib/localization/wall_reset.hpp#L937).*

<a id="sampleverdict"></a>

## `sampleVerdict`

```cpp
[[nodiscard]] inline WallResetVerdict sampleVerdict(const WallSample& sample, const SideMount& mount, const WallResetLimits& limits) noexcept
```

PURE: rule 6 for ONE sample — ReadFault, NoReturn, LowConfidence, OutOfRange, or Seated when the sample is usable. The sampler stops the window at the first sample this refuses; the evaluator names it.

*free function, declared at [`include/shulib/localization/wall_reset.hpp:960`](../../include/shulib/localization/wall_reset.hpp#L960).*

<a id="wallstill"></a>

## `wallStill`

```cpp
[[nodiscard]] inline bool wallStill(const math::Twist2d& worst, const WallResetLimits& limits) noexcept
```

PURE: whether a window's worst twist is still (rule 5): the norm of (vx, vy) within `stillSpeed` and |omega| within `stillTurn`. A non-finite twist is not still.

*free function, declared at [`include/shulib/localization/wall_reset.hpp:980`](../../include/shulib/localization/wall_reset.hpp#L980).*

<a id="describewallreset"></a>

## `describeWallReset`

```cpp
[[nodiscard]] inline WallResetReport describeWallReset(const math::Pose2d& pose, const SideMount& mount, const WallResetRequest& request, const WallResetLimits& limits, units::AngleDim headingTol, units::Length alongUncertainty)
```

PURE: a report with every number the estimate alone decides — the axis, the side, `before`, the aim's numbers, the heading tolerance, the limit echoed — and verdict Seated ("nothing refused yet"). The call fills a pre-sampling refusal's report from it; evaluateWallReset starts from it.

*free function, declared at [`include/shulib/localization/wall_reset.hpp:989`](../../include/shulib/localization/wall_reset.hpp#L989).*

<a id="evaluatewallreset"></a>

## `evaluateWallReset`

```cpp
[[nodiscard]] inline WallResetReport evaluateWallReset(std::span<const WallSample> samples, const math::Pose2d& pose, const math::Twist2d& maxTwist, const SideMount& mount, const WallResetRequest& request, const WallResetLimits& limits, units::AngleDim headingTol, units::Length alongUncertainty)
```

PURE: the whole verdict from a window of samples — rules 5-9 (run preCheck first: this does not re-judge rules 2-4, but an unusable request is refused BadRequest here too, never judged). `maxTwist` is the window's worst twist: its linear part's norm (WallSampler's speed over the trailing `window`) and |omega| each the largest seen. `pose` is the estimate the answer is compared with (and seated over). Fills `measured`, `applied` (SEATED only), `median`, `spread`, `errorBound` and the refusing sample's numbers. Changes nothing.

*free function, declared at [`include/shulib/localization/wall_reset.hpp:1031`](../../include/shulib/localization/wall_reset.hpp#L1031).*

<a id="class-wallsampler"></a>

## `class WallSampler`

```cpp
class WallSampler
```

The window's sampler: one `step()` per control tick, each judging stillness from the estimate and taking ONE sample — the read-fault count read before the sample's first device read and after its second (`distance()` then `confidence()` are TWO device reads; both must be fault-free). It says the window is DONE at the first tick that refuses (moving, or a sample rule 6 refuses), once the window has `minSamples` over `window`, at `windowMax`, or at the buffer's capacity — never later. The explicit call drives it from `Chassis::waitUntil`'s predicate; the everyday form (L6-A2) from the run guard's wait. The wait's own ticks keep the estimate current.  RULE 5's SPEED is the estimate's DISPLACEMENT over the trailing `window`, per that time: on each tick, the distance from the pose of the latest earlier tick at least `window` back (or the window's first tick, while the window is younger than that), divided by the time between them, floored at `window`. A robot moving at v reads v once the window is `window` old, and a push lasting `window` is seen; WHY NOT ONE TICK'S VELOCITY (the L6-A1 review): the estimate's per-tick linear velocity is a finite difference of the fused pose, and on a robot whose tracking wheels sit off the turning centre a still IMU's heading noise (the register's HA-21, ~0.05 degrees) moves that pose a few thousandths of an inch each tick — 1.1-1.4 in/s of PHANTOM speed on the host test plant's wheel offsets, so every reset was refused MOVING while nothing moved. That jitter does not add up over `window` (it is noise about a fixed point), so the displacement reads ~0.08 in/s on average there instead (0.04-0.17 in/s over 600 still resets, ten seeds). It grows in step with the noise: ~0.33 in/s (up to ~0.7) at four times HA-21's, and it reaches the 1-in/s limit only near six times (2 of 600 still resets refused MOVING). The turn rate is judged on every tick from the estimate's own `omega` (the IMU's rate), as before. A NaN pose or turn rate counts as +infinity — the window ends MOVING at that tick (std::max alone would drop a NaN and judge the tick still).

*class, declared at [`include/shulib/localization/wall_reset.hpp:1134`](../../include/shulib/localization/wall_reset.hpp#L1134).*

<a id="wallsampler-wallsampler"></a>

### `WallSampler::WallSampler`

```cpp
WallSampler(const WallSensor& sensor, const WallResetLimits& limits) noexcept
```

Borrows `sensor` and `limits` (both must outlive the sampler); `sensor.sensor` must be set and the limits usable (wallResetProblem() returned nullptr — `window` above 0).

*function, declared at [`include/shulib/localization/wall_reset.hpp:1138`](../../include/shulib/localization/wall_reset.hpp#L1138).*

<a id="wallsampler-step"></a>

### `WallSampler::step`

```cpp
bool step(units::Time now, const math::Pose2d& pose, units::AngularVelocity turnRate)
```

One tick at time `now`, the estimate at `pose` and turning at `turnRate`: judge stillness, take one sample. Returns true when the window is DONE (the caller's wait predicate returns this).

*function, declared at [`include/shulib/localization/wall_reset.hpp:1144`](../../include/shulib/localization/wall_reset.hpp#L1144).*

<a id="wallsampler-samples"></a>

### `WallSampler::samples`

```cpp
[[nodiscard]] std::span<const WallSample> samples() const noexcept
```

The samples taken so far.

*function, declared at [`include/shulib/localization/wall_reset.hpp:1193`](../../include/shulib/localization/wall_reset.hpp#L1193).*

<a id="wallsampler-maxtwist"></a>

### `WallSampler::maxTwist`

```cpp
[[nodiscard]] math::Twist2d maxTwist() const noexcept
```

The window's worst twist so far: vx carries the largest speed judged (rule 5's: the displacement over the trailing `window`), vy is 0, omega the largest |turn rate| — the form evaluateWallReset takes.

*function, declared at [`include/shulib/localization/wall_reset.hpp:1200`](../../include/shulib/localization/wall_reset.hpp#L1200).*

<a id="appendwallresetline"></a>

## `appendWallResetLine`

```cpp
inline void appendWallResetLine(diag::lineformat::Line& line, const WallResetReport& rep, const WallResetRequest& request, const SideMount& mount, const WallResetLimits& limits)
```

PURE: append the reset's ONE log line (subsystem "WALL"; ASCII; within a Line's 384 bytes) — `[<name>: ]<wall> <axis> <WORD> (<SIDE> sensor): <the verdict's numbers>`, e.g. `left wall x SEATED (LEFT sensor): -52.21 -> -54.66 (moved -2.45, limit 3.00) | median 8.05 in spread 0.26 n 10 | bound 0.62 | faults ok`. Caller-supplied text (the names, an absence sentence) goes through the Line's sanitizer.

*free function, declared at [`include/shulib/localization/wall_reset.hpp:1231`](../../include/shulib/localization/wall_reset.hpp#L1231).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 53 lines, click to expand</summary>

```text

 The WALL RESET's pure core (chunk L6-A1, 2026-10-29; the L6 brief, the team lead's answers of
 2026-10-22): the geometry that turns one side distance sensor's reading into ONE field coordinate,
 the gates that REFUSE rather than guess, the verdict, and the one log line. Nothing here moves the
 estimate or reads a clock: the calls that do are `chassis::resetFromWall` (the explicit form over
 sensor handles — chassis/wall_reset_call.hpp) and, from L6-A2, the everyday form over an auton's
 context. Both run THESE functions, so no gate is written twice.

 ── What a wall reset is ─────────────────────────────────────────────────────────────────────────
 A robot parked beside a field wall, nose along it and square to it, with a side sensor facing it:
 the sensor's range, from the sensor's MEASURED place on the robot, says how far the robot's centre
 is from that wall — ONE coordinate (x against the left/right walls, y against the red/blue walls).
 The reset replaces that one coordinate of the estimate (`Localizer::seatAxis`) and nothing else:
 not the other coordinate, not the heading, not the estimate's trust in itself (the drift term).

 ── The rules, in the order they run (the verdict names the FIRST that fails) ───────────────────
   1. Setup — a sensor, its mount measured, the estimate live, NO motion active, the request sane
      (the call checks the first four; `wallResetProblem` the last, naming the field).
   2. Square — by the estimate's heading and the typed facing, the beam within `squareWithin` of
      the wall's normal; a beam pointing AWAY is NotSquare.
   3. Keep-out — the beam centre's coordinate ALONG the wall, widened by the beam's footprint, a
      margin and how far the estimate may have drifted along the wall since that axis was set
      (`alongDriftRate` × `Localizer::travelSinceSeat`), overlaps no active keep-out span.
   4. Beam height — the expected range is at most the CEILING: past it, the cone's lower edge dips
      below a standing game piece (6.5 in) + a margin, so the nearest surface could be a Pin or a Cup.
   5. Stillness — on every tick, the estimate's speed over the trailing `window` (its displacement
      over that time) within `stillSpeed`, and its turn rate |ω| within `stillTurn`.
   6. Each sample — no read fault between its two reads, a real return (confidence > 0), enough
      confidence, and the bias-corrected range inside [rangeMin, rangeMax].
   7. Enough, consistent, under the ceiling — ≥ minSamples over ≥ window; spread ≤ spreadMax; the
      MEDIAN's range ≤ the ceiling too (rule 4 judged the estimate; this judges what was seen).
   8. Precise — the worst-case error of the answer (`axisErrorBound`) ≤ errorBoundMax.
   9. Agrees — |answer − estimate| ≤ the author's maxCorrection. Then the axis is SEATED.
 Rules 1-4 read no sensor and cost no time; a refusal leaves the estimate exactly as it was.

 ── What no gate here can catch (never claim otherwise — guide 14 lists it too) ─────────────────
 Something TALLER than a game piece at a plausible distance in the beam passes every check: a robot
 beside ours, a nested stack against the wall, a sensor wired to the other side's port, something
 close behind a CLEAR wall panel. The defences are the author's (a tight maxCorrection; parked where
 the route says), the keep-outs, the beam-height gate and the mount. A SINGLE standing Pin or Cup is
 refused by rule 4 — so long as the typed height and cone are true (A4 register HA-205).

 ── Every number that is a belief carries its register id ──────────────────────────────────────
 The sensor's accuracy (A4 register HA-204), its cone and nearest-surface behaviour (HA-205), the
 wall returning a reading (HA-206), the update interval (HA-209), and every gate default (HA-211).
 The season's walls and keep-outs are localization/override_walls.hpp's (HA-207, HA-208, HA-212).

 PURE (the master plan's §5 layering): no chassis/, motion/, sequence/ or competition/ include — the
 Localizer includes the leaf axis.hpp, and the calls sit at the layers that own their arguments.
 Allocation-free (fixed std::arrays), no exceptions on a refusal (a refusal is a VALUE);
 SHULIB_PRECONDITION only for programming errors (a non-finite reading handed to the geometry, too
 many keep-outs handed to FieldWall::line). The angle defaults are spelled `N * kWallDegToRad`
 because the `_deg` literal is not constexpr and a robot file's block must sit in a constexpr table.
```

</details>
