<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/vision.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `vision.hpp`

IVision / ITagSource — the AI Vision seams.

This header declares **4** types (20 members).

Extracted from [`include/shulib/hal/vision.hpp`](../../include/shulib/hal/vision.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`struct TagObservation`](#struct-tagobservation)
  - [`id`](#tagobservation-id)
  - [`poseInRobot`](#tagobservation-poseinrobot)
  - [`confidence`](#tagobservation-confidence)
- [`struct ObjectObservation`](#struct-objectobservation)
  - [`classId`](#objectobservation-classid)
  - [`bearing`](#objectobservation-bearing)
  - [`confidence`](#objectobservation-confidence)
- [`class ITagSource`](#class-itagsource)
  - [`~ITagSource`](#itagsource-destructor-itagsource)
  - [`ITagSource`](#itagsource-itagsource)
  - [`ITagSource (overload 2)`](#itagsource-itagsource-2)
  - [`ITagSource (overload 3)`](#itagsource-itagsource-3)
  - [`operator=`](#itagsource-operator-eq)
  - [`operator= (overload 2)`](#itagsource-operator-eq-2)
  - [`tags`](#itagsource-tags)
- [`class IVision`](#class-ivision)
  - [`~IVision`](#ivision-destructor-ivision)
  - [`IVision`](#ivision-ivision)
  - [`IVision (overload 2)`](#ivision-ivision-2)
  - [`IVision (overload 3)`](#ivision-ivision-3)
  - [`operator=`](#ivision-operator-eq)
  - [`operator= (overload 2)`](#ivision-operator-eq-2)
  - [`objects`](#ivision-objects)

<a id="struct-tagobservation"></a>

## `struct TagObservation`

```cpp
struct TagObservation
```

One visible AprilTag, reduced to a robot-relative planar pose.

*struct, declared at [`include/shulib/hal/vision.hpp:32`](../../include/shulib/hal/vision.hpp#L32).*

<a id="tagobservation-id"></a>

### `TagObservation::id`

```cpp
int id
```

AprilTag id within the configured family. A corrector looks this up in its map of known field placements; an id with no map entry is discarded, never guessed at.

*field, declared at [`include/shulib/hal/vision.hpp:35`](../../include/shulib/hal/vision.hpp#L35).*

<a id="tagobservation-poseinrobot"></a>

### `TagObservation::poseInRobot`

```cpp
math::Pose2d poseInRobot
```

Tag pose RELATIVE to the robot, canonical body frame (F1: +X forward, +Y left, heading CCW-positive), inches and radians. Already the PLANAR reduction: the tag's height above the camera, its pitch and its roll were discarded at the edge and are not recoverable.

*field, declared at [`include/shulib/hal/vision.hpp:39`](../../include/shulib/hal/vision.hpp#L39).*

<a id="tagobservation-confidence"></a>

### `TagObservation::confidence`

```cpp
double confidence
```

Detector confidence, [0, 1]. Not a probability that the pose is right — a corrector DIVIDES its measurement sigma by it, so larger means a tighter fix, and 0 means unusable.

*field, declared at [`include/shulib/hal/vision.hpp:42`](../../include/shulib/hal/vision.hpp#L42).*

<a id="struct-objectobservation"></a>

## `struct ObjectObservation`

```cpp
struct ObjectObservation
```

One visible classified object / color, reduced to a robot-relative bearing.

*struct, declared at [`include/shulib/hal/vision.hpp:46`](../../include/shulib/hal/vision.hpp#L46).*

<a id="objectobservation-classid"></a>

### `ObjectObservation::classId`

```cpp
int classId
```

Detected class / color descriptor id, as configured on the detector. Opaque to shulib: nothing here maps an id to a meaning — the manipulation code that asked for it owns that.

*field, declared at [`include/shulib/hal/vision.hpp:49`](../../include/shulib/hal/vision.hpp#L49).*

<a id="objectobservation-bearing"></a>

### `ObjectObservation::bearing`

```cpp
math::Angle bearing
```

Horizontal angle to the object measured from robot +X (forward), CCW-positive, wrapped to (-π, π]. A BEARING only: a bounding box carries no range, so this says which way to turn and never how far to drive.

*field, declared at [`include/shulib/hal/vision.hpp:53`](../../include/shulib/hal/vision.hpp#L53).*

<a id="objectobservation-confidence"></a>

### `ObjectObservation::confidence`

```cpp
double confidence
```

Detector confidence, [0, 1]. Carried for M4 targeting to rank candidates with; no consumer in the tree reads it yet, so nothing currently gates on a low value.

*field, declared at [`include/shulib/hal/vision.hpp:56`](../../include/shulib/hal/vision.hpp#L56).*

<a id="class-itagsource"></a>

## `class ITagSource`

```cpp
class ITagSource
```

AprilTag source (decision #7: V5 AI Vision OR a coprocessor, behind this one seam).

*class, declared at [`include/shulib/hal/vision.hpp:60`](../../include/shulib/hal/vision.hpp#L60).*

<a id="itagsource-destructor-itagsource"></a>

### `ITagSource::~ITagSource`

```cpp
virtual ~ITagSource() = default
```

The polymorphic-base boilerplate, and why it is spelled out: the destructor is virtual so deleting through `ITagSource*` is well-defined, and declaring it suppresses the implicit copy/move, which are therefore re-defaulted. The seam holds no state, so all five are trivial — an implementation is REFERENCED and never owned (RobotContext keeps a non-owning pointer, and the adapter must outlive the context).

*function, declared at [`include/shulib/hal/vision.hpp:67`](../../include/shulib/hal/vision.hpp#L67).*

<a id="itagsource-itagsource"></a>

### `ITagSource::ITagSource`

```cpp
ITagSource() = default
```

*Covered by the comment on [`~ITagSource`](#itagsource-destructor-itagsource) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/vision.hpp:68`](../../include/shulib/hal/vision.hpp#L68).*

<a id="itagsource-itagsource-2"></a>

### `ITagSource::ITagSource (overload 2)`

```cpp
ITagSource(const ITagSource&) = default
```

*Covered by the comment on [`~ITagSource`](#itagsource-destructor-itagsource) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/vision.hpp:69`](../../include/shulib/hal/vision.hpp#L69).*

<a id="itagsource-itagsource-3"></a>

### `ITagSource::ITagSource (overload 3)`

```cpp
ITagSource(ITagSource&&) = default
```

*Covered by the comment on [`~ITagSource`](#itagsource-destructor-itagsource) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/vision.hpp:70`](../../include/shulib/hal/vision.hpp#L70).*

<a id="itagsource-operator-eq"></a>

### `ITagSource::operator=`

```cpp
ITagSource& operator=(const ITagSource&) = default
```

*Covered by the comment on [`~ITagSource`](#itagsource-destructor-itagsource) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/vision.hpp:71`](../../include/shulib/hal/vision.hpp#L71).*

<a id="itagsource-operator-eq-2"></a>

### `ITagSource::operator= (overload 2)`

```cpp
ITagSource& operator=(ITagSource&&) = default
```

*Covered by the comment on [`~ITagSource`](#itagsource-destructor-itagsource) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/vision.hpp:72`](../../include/shulib/hal/vision.hpp#L72).*

<a id="itagsource-tags"></a>

### `ITagSource::tags`

```cpp
[[nodiscard]] virtual std::vector<TagObservation> tags() const = 0
```

AprilTags currently visible, each as a relative pose in the robot frame.

*function, declared at [`include/shulib/hal/vision.hpp:75`](../../include/shulib/hal/vision.hpp#L75).*

<a id="class-ivision"></a>

## `class IVision`

```cpp
class IVision
```

Object / color detection source (manipulation targeting, M4).

*class, declared at [`include/shulib/hal/vision.hpp:79`](../../include/shulib/hal/vision.hpp#L79).*

<a id="ivision-destructor-ivision"></a>

### `IVision::~IVision`

```cpp
virtual ~IVision() = default
```

Same polymorphic-base boilerplate as ITagSource, and for the same reason: a virtual destructor for delete-through-base, with copy/move re-defaulted after declaring it. It matters here that this base is stateless — decision #7 expects ONE adapter to inherit both this and ITagSource off a single V5 AI Vision sensor, and two empty bases cost that adapter nothing.

*function, declared at [`include/shulib/hal/vision.hpp:86`](../../include/shulib/hal/vision.hpp#L86).*

<a id="ivision-ivision"></a>

### `IVision::IVision`

```cpp
IVision() = default
```

*Covered by the comment on [`~IVision`](#ivision-destructor-ivision) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/vision.hpp:87`](../../include/shulib/hal/vision.hpp#L87).*

<a id="ivision-ivision-2"></a>

### `IVision::IVision (overload 2)`

```cpp
IVision(const IVision&) = default
```

*Covered by the comment on [`~IVision`](#ivision-destructor-ivision) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/vision.hpp:88`](../../include/shulib/hal/vision.hpp#L88).*

<a id="ivision-ivision-3"></a>

### `IVision::IVision (overload 3)`

```cpp
IVision(IVision&&) = default
```

*Covered by the comment on [`~IVision`](#ivision-destructor-ivision) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/vision.hpp:89`](../../include/shulib/hal/vision.hpp#L89).*

<a id="ivision-operator-eq"></a>

### `IVision::operator=`

```cpp
IVision& operator=(const IVision&) = default
```

*Covered by the comment on [`~IVision`](#ivision-destructor-ivision) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/vision.hpp:90`](../../include/shulib/hal/vision.hpp#L90).*

<a id="ivision-operator-eq-2"></a>

### `IVision::operator= (overload 2)`

```cpp
IVision& operator=(IVision&&) = default
```

*Covered by the comment on [`~IVision`](#ivision-destructor-ivision) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/vision.hpp:91`](../../include/shulib/hal/vision.hpp#L91).*

<a id="ivision-objects"></a>

### `IVision::objects`

```cpp
[[nodiscard]] virtual std::vector<ObjectObservation> objects() const = 0
```

Classified objects / colors currently visible.

*function, declared at [`include/shulib/hal/vision.hpp:94`](../../include/shulib/hal/vision.hpp#L94).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 21 lines</summary>

```text

 IVision / ITagSource — the AI Vision seams. Decision #7 (locked): the V5 AI Vision
 sensor AND a coprocessor (Pi) both sit behind ITagSource. The V5 sensor reports BOTH
 AprilTags (4 image corners + id) and classified objects/colors (bounding box + score),
 so one hal/pros adapter implements BOTH interfaces from it; a Pi adapter can too.

 CANONICAL FORMS — the raw V5 pixel corners / boxes are reduced to canonical
 robot-relative quantities in the adapter ("convert once at the edge", §7):
  * ITagSource yields each visible AprilTag as a RELATIVE POSE in the robot BODY frame
    (+X forward, +Y left — F1), a planar reduction of the tag's 6-DOF pose suitable for
    ground-plane localization. The corners→pose PnP (needs camera intrinsics + tag size)
    is a pure, host-testable function built with the M3 AprilTagCorrector; at M1 this is
    the seam the corrector reads.
  * IVision yields each object as a class id + BEARING (horizontal angle to the object,
    relative to robot +X) + confidence, for manipulation targeting (M4). The
    box-center→bearing reduction (needs camera FOV) is likewise an M4 pure function.

 Detections are returned BY VALUE: vision runs OFF the 10 ms control hot path (the
 adapter polls it at a lower rate). The consumer timestamps via IClock and owns
 staleness/latency handling (the corrector, M3) — the detection data itself is
 timestamp-free, like the GPS pose.
```

</details>
