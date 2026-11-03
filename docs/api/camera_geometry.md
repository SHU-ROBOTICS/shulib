<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/vision/camera_geometry.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `camera_geometry.hpp`

Camera geometry — a detection's box turned into which way it is (BEARING), how high it looks (ELEVATION), how far it is (a GROUND range from where its base meets the floor, a SIZE range from its known real height) and where it is on the FIELD — every one an…

This header declares **6** types (41 members), **2** free functions, and **4** constants.

Extracted from [`include/shulib/vision/camera_geometry.hpp`](../../include/shulib/vision/camera_geometry.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`kDegToRad`](#kdegtorad) — *constant*
- [`struct Interval`](#struct-interval)
  - [`valid`](#interval-valid)
  - [`lo`](#interval-lo)
  - [`hi`](#interval-hi)
  - [`mid`](#interval-mid)
  - [`width`](#interval-width)
  - [`contains`](#interval-contains)
- [`struct CameraOptics`](#struct-cameraoptics)
  - [`hfovDeg`](#cameraoptics-hfovdeg)
  - [`vfovDeg`](#cameraoptics-vfovdeg)
  - [`fovTolerance`](#cameraoptics-fovtolerance)
  - [`widthPx`](#cameraoptics-widthpx)
  - [`heightPx`](#cameraoptics-heightpx)
- [`kAiVisionOptics`](#kaivisionoptics) — *constant*
- [`kBoxEdgePx`](#kboxedgepx) — *constant*
- [`struct PoseBound`](#struct-posebound)
  - [`pose`](#posebound-pose)
  - [`positionTolerance`](#posebound-positiontolerance)
  - [`headingToleranceDeg`](#posebound-headingtolerancedeg)
- [`boundAtImage`](#boundatimage) — *free function*
- [`struct FieldFix`](#struct-fieldfix)
  - [`valid`](#fieldfix-valid)
  - [`x`](#fieldfix-x)
  - [`y`](#fieldfix-y)
  - [`radius`](#fieldfix-radius)
- [`struct CameraCorner`](#struct-cameracorner)
  - [`fx`](#cameracorner-fx)
  - [`fy`](#cameracorner-fy)
  - [`cx`](#cameracorner-cx)
  - [`cy`](#cameracorner-cy)
  - [`h`](#cameracorner-h)
  - [`pitch`](#cameracorner-pitch)
  - [`yaw`](#cameracorner-yaw)
- [`class CameraModel`](#class-cameramodel)
  - [`CameraModel`](#cameramodel-cameramodel)
  - [`placement`](#cameramodel-placement)
  - [`fx`](#cameramodel-fx)
  - [`fy`](#cameramodel-fy)
  - [`opticsTolerance`](#cameramodel-opticstolerance)
  - [`nominal`](#cameramodel-nominal)
  - [`ray`](#cameramodel-ray)
  - [`bearing`](#cameramodel-bearing)
  - [`elevation`](#cameramodel-elevation)
  - [`groundRange`](#cameramodel-groundrange)
  - [`sizeRange`](#cameramodel-sizerange)
  - [`fieldFix`](#cameramodel-fieldfix)
  - [`inViewForCertain`](#cameramodel-inviewforcertain)
  - [`project`](#cameramodel-project)
  - [`eachCorner`](#cameramodel-eachcorner)
  - [`rangeToPlane`](#cameramodel-rangetoplane)
- [`kMaxPitchSpanDeg`](#kmaxpitchspandeg) — *constant*
- [`cameraModelProblem`](#cameramodelproblem) — *free function*

<a id="kdegtorad"></a>

## `kDegToRad`

```cpp
inline constexpr double kDegToRad = math::Angle::kPi / 180.0
```

Degrees to radians (`math::Angle::kPi / 180`).

*constant, declared at [`include/shulib/vision/camera_geometry.hpp:73`](../../include/shulib/vision/camera_geometry.hpp#L73).*

<a id="struct-interval"></a>

## `struct Interval`

```cpp
struct Interval
```

A closed interval [lo, hi], or invalid. Every geometric answer here is one; an invalid one means nothing could be said (the producer's documentation says when).

*struct, declared at [`include/shulib/vision/camera_geometry.hpp:77`](../../include/shulib/vision/camera_geometry.hpp#L77).*

<a id="interval-valid"></a>

### `Interval::valid`

```cpp
bool valid = false
```

false: nothing could be said

*field, declared at [`include/shulib/vision/camera_geometry.hpp:78`](../../include/shulib/vision/camera_geometry.hpp#L78).*

<a id="interval-lo"></a>

### `Interval::lo`

```cpp
double lo = 0.0
```

the lower bound

*field, declared at [`include/shulib/vision/camera_geometry.hpp:79`](../../include/shulib/vision/camera_geometry.hpp#L79).*

<a id="interval-hi"></a>

### `Interval::hi`

```cpp
double hi = 0.0
```

the upper bound

*field, declared at [`include/shulib/vision/camera_geometry.hpp:80`](../../include/shulib/vision/camera_geometry.hpp#L80).*

<a id="interval-mid"></a>

### `Interval::mid`

```cpp
[[nodiscard]] constexpr double mid() const noexcept
```

The middle of the interval.

*function, declared at [`include/shulib/vision/camera_geometry.hpp:82`](../../include/shulib/vision/camera_geometry.hpp#L82).*

<a id="interval-width"></a>

### `Interval::width`

```cpp
[[nodiscard]] constexpr double width() const noexcept
```

hi - lo.

*function, declared at [`include/shulib/vision/camera_geometry.hpp:84`](../../include/shulib/vision/camera_geometry.hpp#L84).*

<a id="interval-contains"></a>

### `Interval::contains`

```cpp
[[nodiscard]] constexpr bool contains(double x) const noexcept
```

True when the interval is valid and holds `x` (both ends included).

*function, declared at [`include/shulib/vision/camera_geometry.hpp:86`](../../include/shulib/vision/camera_geometry.hpp#L86).*

<a id="struct-cameraoptics"></a>

## `struct CameraOptics`

```cpp
struct CameraOptics
```

The camera's optics — a DEVICE fact, the same for every AI Vision sensor: a library constant (`kAiVisionOptics`, A4 register HA-194), never typed by a team.

*struct, declared at [`include/shulib/vision/camera_geometry.hpp:91`](../../include/shulib/vision/camera_geometry.hpp#L91).*

<a id="cameraoptics-hfovdeg"></a>

### `CameraOptics::hfovDeg`

```cpp
double hfovDeg = 0.0
```

the horizontal field of view across the width, degrees

*field, declared at [`include/shulib/vision/camera_geometry.hpp:92`](../../include/shulib/vision/camera_geometry.hpp#L92).*

<a id="cameraoptics-vfovdeg"></a>

### `CameraOptics::vfovDeg`

```cpp
double vfovDeg = 0.0
```

the vertical field of view across the height, degrees

*field, declared at [`include/shulib/vision/camera_geometry.hpp:93`](../../include/shulib/vision/camera_geometry.hpp#L93).*

<a id="cameraoptics-fovtolerance"></a>

### `CameraOptics::fovTolerance`

```cpp
double fovTolerance = 0.0
```

relative: 0.05 = each focal length known to +-5 %, each on its own

*field, declared at [`include/shulib/vision/camera_geometry.hpp:94`](../../include/shulib/vision/camera_geometry.hpp#L94).*

<a id="cameraoptics-widthpx"></a>

### `CameraOptics::widthPx`

```cpp
int widthPx = hal::kVisionImageWidthPx
```

the report's width, pixels

*field, declared at [`include/shulib/vision/camera_geometry.hpp:95`](../../include/shulib/vision/camera_geometry.hpp#L95).*

<a id="cameraoptics-heightpx"></a>

### `CameraOptics::heightPx`

```cpp
int heightPx = hal::kVisionImageHeightPx
```

the report's height, pixels

*field, declared at [`include/shulib/vision/camera_geometry.hpp:96`](../../include/shulib/vision/camera_geometry.hpp#L96).*

<a id="kaivisionoptics"></a>

## `kAiVisionOptics`

```cpp
inline constexpr CameraOptics kAiVisionOptics{74.0, 63.0, 0.05, hal::kVisionImageWidthPx, hal::kVisionImageHeightPx}
```

The AI Vision sensor's optics as VEX publishes them — 74 x 63 degrees (VEX's knowledge-base article, read only through search snippets: the page answers 403) — with +-5 % on each focal length until the bench measures them (A4 register HA-194).

*constant, declared at [`include/shulib/vision/camera_geometry.hpp:102`](../../include/shulib/vision/camera_geometry.hpp#L102).*

<a id="kboxedgepx"></a>

## `kBoxEdgePx`

```cpp
inline constexpr double kBoxEdgePx = 2.0
```

A box edge's jitter, pixels, for every edge-derived quantity — a model object's box AND a colour band's top and bottom rows (A4 register HA-202: invented until the bench records boxes of a still object). A band no taller than two of these cannot be measured.

*constant, declared at [`include/shulib/vision/camera_geometry.hpp:108`](../../include/shulib/vision/camera_geometry.hpp#L108).*

<a id="struct-posebound"></a>

## `struct PoseBound`

```cpp
struct PoseBound
```

A robot pose with how far it may be off — the CALLER's statement (the localizer's quality, or a bound a route author types; nothing here knows a robot's drift). It must bound the pose AT THE IMAGE's time, not at the read's completion (A4 register HA-192): `boundAtImage()` widens a moving robot's bound.

*struct, declared at [`include/shulib/vision/camera_geometry.hpp:113`](../../include/shulib/vision/camera_geometry.hpp#L113).*

<a id="posebound-pose"></a>

### `PoseBound::pose`

```cpp
math::Pose2d pose{}
```

the field-frame pose (F1)

*field, declared at [`include/shulib/vision/camera_geometry.hpp:114`](../../include/shulib/vision/camera_geometry.hpp#L114).*

<a id="posebound-positiontolerance"></a>

### `PoseBound::positionTolerance`

```cpp
units::Length positionTolerance{}
```

+- radius on x and y, inches

*field, declared at [`include/shulib/vision/camera_geometry.hpp:115`](../../include/shulib/vision/camera_geometry.hpp#L115).*

<a id="posebound-headingtolerancedeg"></a>

### `PoseBound::headingToleranceDeg`

```cpp
double headingToleranceDeg = 0.0
```

+- on the heading, degrees

*field, declared at [`include/shulib/vision/camera_geometry.hpp:116`](../../include/shulib/vision/camera_geometry.hpp#L116).*

<a id="boundatimage"></a>

## `boundAtImage`

```cpp
[[nodiscard]] inline PoseBound boundAtImage(const PoseBound& atRead, double speedInPerS, double turnRateDegPerS, double maxImageAgeSeconds) noexcept
```

A bound on the pose at the IMAGE's time from a bound at the read's completion: the position tolerance grows by |speed| x age and the heading tolerance by |turn rate| x age. `maxImageAgeSeconds` is the camera's image age at read completion, MEASURED on the bench (A4 register HA-192) — a consumer with no measured age has no bound for a moving robot. A negative age, a non-finite speed, turn rate or age, or a negative or non-finite tolerance in `atRead` is NOT a bound: both tolerances of the result are infinite (every fix it touches becomes unusably wide — never a tight bound from a bad number).

*free function, declared at [`include/shulib/vision/camera_geometry.hpp:125`](../../include/shulib/vision/camera_geometry.hpp#L125).*

<a id="struct-fieldfix"></a>

## `struct FieldFix`

```cpp
struct FieldFix
```

A point on the field with a conservative error RADIUS (a bound from the intervals, not a sigma).

*struct, declared at [`include/shulib/vision/camera_geometry.hpp:145`](../../include/shulib/vision/camera_geometry.hpp#L145).*

<a id="fieldfix-valid"></a>

### `FieldFix::valid`

```cpp
bool valid = false
```

false: no position could be computed

*field, declared at [`include/shulib/vision/camera_geometry.hpp:146`](../../include/shulib/vision/camera_geometry.hpp#L146).*

<a id="fieldfix-x"></a>

### `FieldFix::x`

```cpp
double x = 0.0
```

field x, inches (the nominal point)

*field, declared at [`include/shulib/vision/camera_geometry.hpp:147`](../../include/shulib/vision/camera_geometry.hpp#L147).*

<a id="fieldfix-y"></a>

### `FieldFix::y`

```cpp
double y = 0.0
```

field y, inches (the nominal point)

*field, declared at [`include/shulib/vision/camera_geometry.hpp:148`](../../include/shulib/vision/camera_geometry.hpp#L148).*

<a id="fieldfix-radius"></a>

### `FieldFix::radius`

```cpp
double radius = 0.0
```

every position the stated tolerances allow lies within this of (x, y)

*field, declared at [`include/shulib/vision/camera_geometry.hpp:149`](../../include/shulib/vision/camera_geometry.hpp#L149).*

<a id="struct-cameracorner"></a>

## `struct CameraCorner`

```cpp
struct CameraCorner
```

One corner of the tolerance box: a concrete camera the model evaluates.

*struct, declared at [`include/shulib/vision/camera_geometry.hpp:153`](../../include/shulib/vision/camera_geometry.hpp#L153).*

<a id="cameracorner-fx"></a>

### `CameraCorner::fx`

```cpp
double fx = 0.0
```

the horizontal focal length, pixels

*field, declared at [`include/shulib/vision/camera_geometry.hpp:154`](../../include/shulib/vision/camera_geometry.hpp#L154).*

<a id="cameracorner-fy"></a>

### `CameraCorner::fy`

```cpp
double fy = 0.0
```

the vertical focal length, pixels

*field, declared at [`include/shulib/vision/camera_geometry.hpp:155`](../../include/shulib/vision/camera_geometry.hpp#L155).*

<a id="cameracorner-cx"></a>

### `CameraCorner::cx`

```cpp
double cx = 0.0
```

the principal point's column, pixels

*field, declared at [`include/shulib/vision/camera_geometry.hpp:156`](../../include/shulib/vision/camera_geometry.hpp#L156).*

<a id="cameracorner-cy"></a>

### `CameraCorner::cy`

```cpp
double cy = 0.0
```

the principal point's row, pixels

*field, declared at [`include/shulib/vision/camera_geometry.hpp:157`](../../include/shulib/vision/camera_geometry.hpp#L157).*

<a id="cameracorner-h"></a>

### `CameraCorner::h`

```cpp
double h = 0.0
```

the lens's height above the floor, inches

*field, declared at [`include/shulib/vision/camera_geometry.hpp:158`](../../include/shulib/vision/camera_geometry.hpp#L158).*

<a id="cameracorner-pitch"></a>

### `CameraCorner::pitch`

```cpp
double pitch = 0.0
```

the optical axis above the horizontal, radians

*field, declared at [`include/shulib/vision/camera_geometry.hpp:159`](../../include/shulib/vision/camera_geometry.hpp#L159).*

<a id="cameracorner-yaw"></a>

### `CameraCorner::yaw`

```cpp
double yaw = 0.0
```

the optical axis counter-clockwise from the body's +X, radians

*field, declared at [`include/shulib/vision/camera_geometry.hpp:160`](../../include/shulib/vision/camera_geometry.hpp#L160).*

<a id="class-cameramodel"></a>

## `class CameraModel`

```cpp
class CameraModel
```

The camera model: optics and a measured placement, giving pure answers with intervals. PRECONDITION: the placement has no problem (`cameraModelProblem()` returns nullptr for its config) — every consumer asks that first and refuses to construct, naming the reason.

*class, declared at [`include/shulib/vision/camera_geometry.hpp:166`](../../include/shulib/vision/camera_geometry.hpp#L166).*

<a id="cameramodel-cameramodel"></a>

### `CameraModel::CameraModel`

```cpp
CameraModel(const CameraOptics& o, const config::CameraPlacement& p) noexcept
```

The model of a camera with optics `o` mounted at `p` (both copied).

*function, declared at [`include/shulib/vision/camera_geometry.hpp:169`](../../include/shulib/vision/camera_geometry.hpp#L169).*

<a id="cameramodel-placement"></a>

### `CameraModel::placement`

```cpp
[[nodiscard]] const config::CameraPlacement& placement() const noexcept
```

The placement this model was built with.

*function, declared at [`include/shulib/vision/camera_geometry.hpp:175`](../../include/shulib/vision/camera_geometry.hpp#L175).*

<a id="cameramodel-fx"></a>

### `CameraModel::fx`

```cpp
[[nodiscard]] double fx() const noexcept
```

The nominal horizontal focal length, pixels (212.3 for kAiVisionOptics).

*function, declared at [`include/shulib/vision/camera_geometry.hpp:177`](../../include/shulib/vision/camera_geometry.hpp#L177).*

<a id="cameramodel-fy"></a>

### `CameraModel::fy`

```cpp
[[nodiscard]] double fy() const noexcept
```

The nominal vertical focal length, pixels (195.8 for kAiVisionOptics).

*function, declared at [`include/shulib/vision/camera_geometry.hpp:179`](../../include/shulib/vision/camera_geometry.hpp#L179).*

<a id="cameramodel-opticstolerance"></a>

### `CameraModel::opticsTolerance`

```cpp
[[nodiscard]] double opticsTolerance() const noexcept
```

The relative tolerance on each focal length.

*function, declared at [`include/shulib/vision/camera_geometry.hpp:181`](../../include/shulib/vision/camera_geometry.hpp#L181).*

<a id="cameramodel-nominal"></a>

### `CameraModel::nominal`

```cpp
[[nodiscard]] CameraCorner nominal() const noexcept
```

The NOMINAL camera (every tolerance at zero).

*function, declared at [`include/shulib/vision/camera_geometry.hpp:184`](../../include/shulib/vision/camera_geometry.hpp#L184).*

<a id="cameramodel-ray"></a>

### `CameraModel::ray`

```cpp
static void ray(const CameraCorner& c, double u, double v, double& bearing, double& elevation) noexcept
```

The bearing (body frame) and elevation, radians, of the ray through pixel (u, v) for camera corner `c`.

*function, declared at [`include/shulib/vision/camera_geometry.hpp:191`](../../include/shulib/vision/camera_geometry.hpp#L191).*

<a id="cameramodel-bearing"></a>

### `CameraModel::bearing`

```cpp
[[nodiscard]] Interval bearing(double u, double v) const noexcept
```

The BEARING of the point (u, v) — a box's centre — over the yaw, pitch and focal-length tolerances and +-kBoxEdgePx of jitter on the centre's column AND its row, radians (the centre of two edges that each move kBoxEdgePx moves as far, when both move the same way). Its INTERIOR point (header): the ray's forward part, cos p + yd sin p, is LARGEST — the bearing nearest the axis — at the pitch that makes the row's ray level, atan(yd); when that pitch is inside the tolerance it is evaluated too, for each corner and each jittered row.

*function, declared at [`include/shulib/vision/camera_geometry.hpp:212`](../../include/shulib/vision/camera_geometry.hpp#L212).*

<a id="cameramodel-elevation"></a>

### `CameraModel::elevation`

```cpp
[[nodiscard]] Interval elevation(double u, double v) const noexcept
```

The ELEVATION of the point (u, v), likewise, with +-kBoxEdgePx of jitter on the row AND the column, radians. Its INTERIOR point (header): the farther the column from the principal point's, the nearer level the ray, so when the jittered column reaches the principal point's that column is evaluated too — the elevation farthest from level.

*function, declared at [`include/shulib/vision/camera_geometry.hpp:241`](../../include/shulib/vision/camera_geometry.hpp#L241).*

<a id="cameramodel-groundrange"></a>

### `CameraModel::groundRange`

```cpp
[[nodiscard]] Interval groundRange(double u, double vBottom, double minDepressionDeg = 2.0) const noexcept
```

The GROUND RANGE: the horizontal distance, inches, from the LENS to where the ray through (u, vBottom) meets the floor — the range of a thing whose base is ON the floor at that row (that a model box's bottom row IS the piece's base is A4 register HA-200), over the mount's height, pitch and focal-length tolerances and +-kBoxEdgePx of jitter on the row AND the column — with the principal point's column when the jittered column reaches it (elevation()'s interior point: the ray steepest there). Invalid when any evaluated ray descends less than `minDepressionDeg`: near the horizon the range is unbounded, so it refuses rather than report an infinity.

*function, declared at [`include/shulib/vision/camera_geometry.hpp:265`](../../include/shulib/vision/camera_geometry.hpp#L265).*

<a id="cameramodel-sizerange"></a>

### `CameraModel::sizeRange`

```cpp
[[nodiscard]] Interval sizeRange(double realHeightIn, double u, double vCentre, double pxHeight) const noexcept
```

The SIZE RANGE: the horizontal distance, inches, from the LENS to an UPRIGHT thing `realHeightIn` tall whose box spans `pxHeight` rows centred on row `vCentre`, at column `u` (that a model box spans the WHOLE piece is A4 register HA-200). For each corner of the tolerance box the answer is EXACT for a vertical line: the ray through image row v, turned to the line's bearing b off the optical axis, rises tan(pitch - atan((v - cy) / fy)) x cos(b) per inch of horizontal distance, so the line's top and bottom rows put it at realHeightIn / (cos(b) x (that rise at the top - at the bottom)) — the pitch AND the line of sight's sideways angle carried exactly (a Pin 30 degrees off the axis is about 15 % farther than its axis depth). Widened over the corners, +-kBoxEdgePx on each of the two edges and on the column. The sideways factor is taken at the box's OBSERVED centre row: that row moves only when both edges move the same way, and such a box never sets an end — moving an edge changes the rise between the rows at least 5.8 times as much as the sideways factor, over the whole pitch span (measured on a dense grid, VIS1-A1's verification round). Its INTERIOR points (header): the column through the principal point, when the jittered column may be it (the line of sight square to the image: the NEAR end); and — for the FAR end — the pitch at which the product of the sideways factor and the rise between the two rows is smallest, which for a box straddling the lens's level lies INSIDE the pitch tolerance (the rise between two rows is smallest with the box symmetric about level). That product is log-convex in the pitch, and for every pitch smallest with the box shrunk by kBoxEdgePx at each edge and the column farther from the axis, so its minimum is searched (bisection on its log's slope) for that box at each focal-length corner, and taken as a bound from below — the far end never short. A real piece's box also takes in the depth of its top face, so the size range of a Pin seen from above reads somewhat NEARER than its near rim (a consumer comparing it with the ground range allows for that). Invalid when the box is too small to measure (no taller than 2 edges of jitter and a pixel) or the height is not a positive number; a CLIPPED box is the caller's to refuse (`hal::Detection::clipped()`).

*function, declared at [`include/shulib/vision/camera_geometry.hpp:315`](../../include/shulib/vision/camera_geometry.hpp#L315).*

<a id="cameramodel-fieldfix"></a>

### `CameraModel::fieldFix`

```cpp
[[nodiscard]] FieldFix fieldFix(const PoseBound& robot, const Interval& bearingI, const Interval& rangeI) const noexcept
```

The FIELD POSITION of a thing at `bearingI` (body frame, radians) and horizontal `rangeI` (inches) from the LENS, seen from the robot pose `robot`. The radius is how far the farthest point of the two intervals, at any heading of the pose's interval, lies from the nominal point — EXACT over the three together — plus the pose's position tolerance and the mount's. The farthest point is one of: a corner (each interval's end) at an end of the heading interval; a corner turned diametrically opposite the nominal point about the robot's centre, when the heading interval reaches that turn (a tolerance near or past 180 degrees); or the point of the FAR range's arc pointing straight away from the nominal point as seen from the lens, when its bearing is inside the bearing interval — at an end of the heading interval, or turned until the lens lies on the line through the robot's centre and the nominal point (with the lens behind the centre and the ray pointing straight away, that is the lens offset + range.hi + the nominal point's distance from the centre). Each is counted when the intervals reach it. Invalid when either interval is invalid or has a non-finite end, or the pose's x or y is not a finite number. A NEGATIVE or NON-FINITE position or heading tolerance is not a bound: the radius is then INFINITE (the nominal point still computed) — never a tight radius from a bad number.

*function, declared at [`include/shulib/vision/camera_geometry.hpp:391`](../../include/shulib/vision/camera_geometry.hpp#L391).*

<a id="cameramodel-inviewforcertain"></a>

### `CameraModel::inViewForCertain`

```cpp
[[nodiscard]] bool inViewForCertain(double fwd, double lft, double up, double marginPx) const noexcept
```

Whether the floor point at body-frame (`fwd`, `lft`) inches — or the point `up` inches above it — is inside the image, at least `marginPx` from every edge, for EVERY corner of the tolerance box.

*function, declared at [`include/shulib/vision/camera_geometry.hpp:477`](../../include/shulib/vision/camera_geometry.hpp#L477).*

<a id="cameramodel-project"></a>

### `CameraModel::project`

```cpp
[[nodiscard]] bool project(const CameraCorner& c, double fwd, double lft, double up, double& u, double& v) const noexcept
```

The forward projection of the body-frame point (`fwd`, `lft`, `up`) inches through corner `c` into pixel (u, v); false when the point is not in front of the lens. Used by the in-view test (the host camera model has its own, written separately — the two must never share code).

*function, declared at [`include/shulib/vision/camera_geometry.hpp:493`](../../include/shulib/vision/camera_geometry.hpp#L493).*

<a id="cameramodel-eachcorner"></a>

### `CameraModel::eachCorner`

```cpp
template <class F> void eachCorner(bool useHeight, F&& fn) const noexcept
```

Call `fn(corner)` for every corner of the tolerance box — 32 with the height tolerance, 16 without (each focal length, the pitch, the yaw and the height at each of its two extremes) — for a consumer that tests a CONSISTENCY between two quantities computed from ONE camera (the header: one camera explains it). Never overlap two intervals each widened over every corner.

*function, declared at [`include/shulib/vision/camera_geometry.hpp:520`](../../include/shulib/vision/camera_geometry.hpp#L520).*

<a id="cameramodel-rangetoplane"></a>

### `CameraModel::rangeToPlane`

```cpp
static bool rangeToPlane(const CameraCorner& c, double u, double v, double planeIn, double minAngleDeg, double& out) noexcept
```

For ONE corner: the horizontal lens distance, inches, at which the ray through pixel (u, v) meets the horizontal plane `planeIn` inches above the floor, in `out`. False when the ray meets it behind the camera or within `minAngleDeg` of level (unbounded there — refused, never an infinity).

*function, declared at [`include/shulib/vision/camera_geometry.hpp:527`](../../include/shulib/vision/camera_geometry.hpp#L527).*

<a id="kmaxpitchspandeg"></a>

## `kMaxPitchSpanDeg`

```cpp
inline constexpr double kMaxPitchSpanDeg = 55.0
```

How far the mount's pitch WITH its tolerance may reach from level, degrees: inside it every ray of every corner of the tolerance box points forward and the inputs the header calls monotone stay monotone, so every interval here is exact (VIS1-A1's verification round: a ray of the image's edge row turns backward past about 56.7 degrees). cameraModelProblem() refuses a mount that reaches past it.

*constant, declared at [`include/shulib/vision/camera_geometry.hpp:639`](../../include/shulib/vision/camera_geometry.hpp#L639).*

<a id="cameramodelproblem"></a>

## `cameraModelProblem`

```cpp
[[nodiscard]] constexpr const char* cameraModelProblem(const config::CameraConfig& cfg) noexcept
```

Why no CameraModel can be built for this robot's camera, or nullptr (then `CameraModel{ kAiVisionOptics, cfg.placement}` is valid): "no camera is declared" for port 0; the placement's own problem (`config::cameraPlacementProblem` — an UNSET mount first); or a pitch whose tolerance takes it past kMaxPitchSpanDeg (|pitch| + tolerance above 55 degrees — the geometry's own limit, not the robot file's: the camera is still built and read). Every geometry consumer asks this first and refuses to construct, naming the reason; the robot still drives (VIS1's D5).

*free function, declared at [`include/shulib/vision/camera_geometry.hpp:647`](../../include/shulib/vision/camera_geometry.hpp#L647).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 57 lines, click to expand</summary>

```text

 Camera geometry — a detection's box turned into which way it is (BEARING), how high it looks
 (ELEVATION), how far it is (a GROUND range from where its base meets the floor, a SIZE range from its
 known real height) and where it is on the FIELD — every one an INTERVAL that carries the stated error
 of everything it was computed from (chunk VIS1-A1, 2026-10-22). Pure functions over plain numbers;
 PROS-free, sim-free, allocation-free.

 ── Frames ──────────────────────────────────────────────────────────────────────────
 Image: +u right, +v DOWN, (0,0) the top-left corner, 320 x 240 (A4 register HA-193). Body (F1): +X
 forward, +Y LEFT, +Z up from the FLOOR. Bearing: counter-clockwise from the body's +X (left
 positive), radians. Elevation: above the horizontal, radians. Field: F1's (origin at the field's
 centre).

 ── The error is an INTERVAL, not a sigma ───────────────────────────────────────────
 Each input carries a stated tolerance: the mount's height, pitch, yaw and position (measured by the
 team — config::CameraPlacement), the optics' two focal lengths, EACH on its own (A4 register HA-194),
 and a box edge's jitter (A4 register HA-202). Every interval is the EXACT range of its output over that
 whole tolerance box: the output is evaluated at every CORNER of the box, where it is monotone in an
 input, and also at the INTERIOR point where it is not — a pitch that makes a row's ray LEVEL (the
 bearing of a point on the lens's level; the size range of a box straddling it) and the principal
 point's column (the elevation, ground range and size range of a point whose jittered column may be on
 it). A box edge's jitter moves a point in BOTH its coordinates, and each function carries both (the
 size range through its two edges and its column). Each function's doc names its interior points, and a
 test sweeps each against the whole box at 100 % of every tolerance. That holds while the pitch with its
 tolerance stays inside +-kMaxPitchSpanDeg — every ray of every corner then points forward, and each
 input taken as monotone stays so — which cameraModelProblem() requires. A consumer that needs one value
 takes the interval and REFUSES when it is too wide. An interval is a bound on what the stated
 tolerances allow, never a probability: a mount that moved beyond its tolerance makes it wrong, which is
 why the tolerance is typed with its provenance.

 The stated tolerances include the CALLER's PoseBound, and that bound must hold for the pose WHEN THE
 IMAGE WAS TAKEN, not when the read completed: the image's age at read completion is UNMEASURED (A4
 register HA-192), and a robot turning at 180 deg/s carries a Pin 36 in ahead about 7 in off in 66 ms.
 boundAtImage() widens a bound by the robot's speed and turn rate times that age; a consumer that has
 no MEASURED age uses field positions only with the robot stopped.

 ── One camera explains it ──────────────────────────────────────────────────────────
 Where two quantities come from ONE camera (a band's two edges; a Pin's base and its height) the pitch,
 height and focal-length errors are COMMON to both. A consumer tests their agreement CORNER BY CORNER
 through eachCorner() — "is there one camera, inside the stated tolerances, that sees this box as X?" —
 never by overlapping two intervals each widened over every corner, which double-counts the common
 error (VIS1's probes: 129 of 540 lookalike bands passed with overlapped intervals, 86 per corner).

 ── The camera's axis, and what is not modelled ─────────────────────────────────────
 The optical axis is taken square to the housing's back face, where the team measures pitch and yaw
 (A4 register HA-194); a lens mounted off that axis is a yaw or pitch to type, not a tolerance to widen.
 The optics are TWO angles, not one: 74 x 63 degrees over 320 x 240 is not square pixels (fx = 160/tan
 37 deg = 212.3 px, fy = 120/tan 31.5 deg = 195.8 px), so the two focal lengths are carried apart. The
 principal point is the image's centre; lens distortion is not modelled (HA-194's settle measures how
 far off-centre it bites); ROLL is not modelled (the mount is yaw and pitch only — re-mount a camera
 rolled more than about a degree).

 hal::CameraMount (hal/vision_conversion.hpp) encodes HA-70's LEVEL camera for the AprilTag PnP and is
 untouched: tags are not this header's.

 NO HARDWARE CLAIM: every function here has run against the host camera model (sim/camera_plant.hpp,
 written with its own projection) and nothing else.
```

</details>
