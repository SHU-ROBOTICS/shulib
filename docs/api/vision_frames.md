<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/vision_frames.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `vision_frames.hpp`

IVisionFrames — the V5 AI Vision camera read a FRAME at a time.

This header declares **9** types (59 members), **5** free functions, and **5** constants.

Extracted from [`include/shulib/hal/vision_frames.hpp`](../../include/shulib/hal/vision_frames.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in [How it works](../guide/README.md); worked recipes live in [Recipes](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`kMaxVisionDetections`](#kmaxvisiondetections) — *constant*
- [`kMaxColourDescriptors`](#kmaxcolourdescriptors) — *constant*
- [`kMaxVisionClassNames`](#kmaxvisionclassnames) — *constant*
- [`kVisionImageWidthPx`](#kvisionimagewidthpx) — *constant*
- [`kVisionImageHeightPx`](#kvisionimageheightpx) — *constant*
- [`enum class DetectionKind`](#enum-class-detectionkind)
  - [`Colour`](#detectionkind-colour)
  - [`Code`](#detectionkind-code)
  - [`Object`](#detectionkind-object)
- [`struct Detection`](#struct-detection)
  - [`kind`](#detection-kind)
  - [`id`](#detection-id)
  - [`left`](#detection-left)
  - [`top`](#detection-top)
  - [`width`](#detection-width)
  - [`height`](#detection-height)
  - [`angleTenths`](#detection-angletenths)
  - [`score`](#detection-score)
  - [`clipped`](#detection-clipped)
- [`enum class FrameStatus`](#enum-class-framestatus)
  - [`NotRead`](#framestatus-notread)
  - [`Ok`](#framestatus-ok)
  - [`ReadFault`](#framestatus-readfault)
  - [`PartialFault`](#framestatus-partialfault)
  - [`NotConfigured`](#framestatus-notconfigured)
- [`enum class ModelCheck`](#enum-class-modelcheck)
  - [`NotAsked`](#modelcheck-notasked)
  - [`Matches`](#modelcheck-matches)
  - [`Different`](#modelcheck-different)
  - [`Unreadable`](#modelcheck-unreadable)
- [`struct VisionFrame`](#struct-visionframe)
  - [`status`](#visionframe-status)
  - [`readAt`](#visionframe-readat)
  - [`readSeq`](#visionframe-readseq)
  - [`reportedCount`](#visionframe-reportedcount)
  - [`truncated`](#visionframe-truncated)
  - [`count`](#visionframe-count)
  - [`skipped`](#visionframe-skipped)
  - [`model`](#visionframe-model)
  - [`objectsWithheld`](#visionframe-objectswithheld)
  - [`coloursWithheld`](#visionframe-colourswithheld)
  - [`detections`](#visionframe-detections)
  - [`usable`](#visionframe-usable)
- [`struct ColourDescriptor`](#struct-colourdescriptor)
  - [`id`](#colourdescriptor-id)
  - [`red`](#colourdescriptor-red)
  - [`green`](#colourdescriptor-green)
  - [`blue`](#colourdescriptor-blue)
  - [`hueRange`](#colourdescriptor-huerange)
  - [`saturationRange`](#colourdescriptor-saturationrange)
  - [`name`](#colourdescriptor-name)
  - [`provenance`](#colourdescriptor-provenance)
- [`colourDescriptorProblem`](#colourdescriptorproblem) — *free function*
- [`struct AiVisionSettings`](#struct-aivisionsettings)
  - [`colours`](#aivisionsettings-colours)
  - [`colourCount`](#aivisionsettings-colourcount)
  - [`objects`](#aivisionsettings-objects)
  - [`expectedClasses`](#aivisionsettings-expectedclasses)
  - [`expectedClassCount`](#aivisionsettings-expectedclasscount)
  - [`colourMerge`](#aivisionsettings-colourmerge)
- [`aiVisionSettingsProblem`](#aivisionsettingsproblem) — *free function*
- [`sameVisionText`](#samevisiontext) — *free function*
- [`sameAiVisionSettings`](#sameaivisionsettings) — *free function*
- [`struct CameraStatus`](#struct-camerastatus)
  - [`configured`](#camerastatus-configured)
  - [`setupProblem`](#camerastatus-setupproblem)
  - [`model`](#camerastatus-model)
  - [`modelProblem`](#camerastatus-modelproblem)
- [`cameraStatusWords`](#camerastatuswords) — *free function*
- [`class IVisionFrames`](#class-ivisionframes)
  - [`~IVisionFrames`](#ivisionframes-destructor-ivisionframes)
  - [`IVisionFrames`](#ivisionframes-ivisionframes)
  - [`IVisionFrames (overload 2)`](#ivisionframes-ivisionframes-2)
  - [`IVisionFrames (overload 3)`](#ivisionframes-ivisionframes-3)
  - [`operator=`](#ivisionframes-operator-eq)
  - [`operator= (overload 2)`](#ivisionframes-operator-eq-2)
  - [`read`](#ivisionframes-read)
  - [`status`](#ivisionframes-status)

<a id="kmaxvisiondetections"></a>

## `kMaxVisionDetections`

```cpp
inline constexpr std::size_t kMaxVisionDetections = 24
```

The camera's most detections per read (AIVISION_MAX_OBJECT_COUNT, the vendored ai_vision.h :33 — restated so this header stays PROS-free; hal/pros/ai_vision.hpp static_asserts the two agree).

*constant, declared at [`include/shulib/hal/vision_frames.hpp:48`](../../include/shulib/hal/vision_frames.hpp#L48).*

<a id="kmaxcolourdescriptors"></a>

## `kMaxColourDescriptors`

```cpp
inline constexpr std::uint8_t kMaxColourDescriptors = 7
```

Colour descriptor ids run 1..7 (ai_vision.h :86).

*constant, declared at [`include/shulib/hal/vision_frames.hpp:50`](../../include/shulib/hal/vision_frames.hpp#L50).*

<a id="kmaxvisionclassnames"></a>

## `kMaxVisionClassNames`

```cpp
inline constexpr std::uint8_t kMaxVisionClassNames = 20
```

The most class names a camera's model can name (AIVISION_MAX_CLASSNAME_COUNT, ai_vision.h :34).

*constant, declared at [`include/shulib/hal/vision_frames.hpp:52`](../../include/shulib/hal/vision_frames.hpp#L52).*

<a id="kvisionimagewidthpx"></a>

## `kVisionImageWidthPx`

```cpp
inline constexpr int kVisionImageWidthPx = 320
```

The camera's report width, in pixels (api.vex.com: 320 x 240; A4 register HA-193).

*constant, declared at [`include/shulib/hal/vision_frames.hpp:54`](../../include/shulib/hal/vision_frames.hpp#L54).*

<a id="kvisionimageheightpx"></a>

## `kVisionImageHeightPx`

```cpp
inline constexpr int kVisionImageHeightPx = 240
```

The camera's report height, in pixels (A4 register HA-193).

*constant, declared at [`include/shulib/hal/vision_frames.hpp:56`](../../include/shulib/hal/vision_frames.hpp#L56).*

<a id="enum-class-detectionkind"></a>

## `enum class DetectionKind`

```cpp
enum class DetectionKind : std::uint8_t
```

What a detection IS. OURS, not PROS's bit flags — the adapter maps each. No Tag: VIS1 turns tag detection OFF, and a tag that arrives anyway is counted in `VisionFrame::skipped`, never passed on.

*enum class, declared at [`include/shulib/hal/vision_frames.hpp:60`](../../include/shulib/hal/vision_frames.hpp#L60).*

<a id="detectionkind-colour"></a>

### `DetectionKind::Colour`

```cpp
Colour = 1
```

a colour descriptor's blob: `id` is the descriptor, 1..7 — one the settings wrote

*enumerator, declared at [`include/shulib/hal/vision_frames.hpp:61`](../../include/shulib/hal/vision_frames.hpp#L61).*

<a id="detectionkind-code"></a>

### `DetectionKind::Code`

```cpp
Code = 2
```

a colour code's merged blob, `id` 1..5 (VIS1 writes none; its adapter withholds each)

*enumerator, declared at [`include/shulib/hal/vision_frames.hpp:62`](../../include/shulib/hal/vision_frames.hpp#L62).*

<a id="detectionkind-object"></a>

### `DetectionKind::Object`

```cpp
Object = 3
```

the AI model's element: `id` is its class (meaningful only once ModelCheck::Matches)

*enumerator, declared at [`include/shulib/hal/vision_frames.hpp:63`](../../include/shulib/hal/vision_frames.hpp#L63).*

<a id="struct-detection"></a>

## `struct Detection`

```cpp
struct Detection
```

One detection, copied OUT of the camera's packed record, in IMAGE pixels: +x right, +y DOWN, (0,0) the top-left corner; the box's LEFT and TOP edges plus its size (ai_vision.h :129-130's "left edge" / "top edge" — A4 register HA-193).

*struct, declared at [`include/shulib/hal/vision_frames.hpp:69`](../../include/shulib/hal/vision_frames.hpp#L69).*

<a id="detection-kind"></a>

### `Detection::kind`

```cpp
DetectionKind kind = DetectionKind::Colour
```

what it is

*field, declared at [`include/shulib/hal/vision_frames.hpp:70`](../../include/shulib/hal/vision_frames.hpp#L70).*

<a id="detection-id"></a>

### `Detection::id`

```cpp
std::uint8_t id = 0
```

descriptor 1..7 / code 1..5 / model class 0..

*field, declared at [`include/shulib/hal/vision_frames.hpp:71`](../../include/shulib/hal/vision_frames.hpp#L71).*

<a id="detection-left"></a>

### `Detection::left`

```cpp
std::int16_t left = 0
```

px from the image's left edge

*field, declared at [`include/shulib/hal/vision_frames.hpp:72`](../../include/shulib/hal/vision_frames.hpp#L72).*

<a id="detection-top"></a>

### `Detection::top`

```cpp
std::int16_t top = 0
```

px from the image's top edge

*field, declared at [`include/shulib/hal/vision_frames.hpp:73`](../../include/shulib/hal/vision_frames.hpp#L73).*

<a id="detection-width"></a>

### `Detection::width`

```cpp
std::int16_t width = 0
```

px

*field, declared at [`include/shulib/hal/vision_frames.hpp:74`](../../include/shulib/hal/vision_frames.hpp#L74).*

<a id="detection-height"></a>

### `Detection::height`

```cpp
std::int16_t height = 0
```

px

*field, declared at [`include/shulib/hal/vision_frames.hpp:75`](../../include/shulib/hal/vision_frames.hpp#L75).*

<a id="detection-angletenths"></a>

### `Detection::angleTenths`

```cpp
std::int16_t angleTenths = 0
```

Colour/Code only: the blob's angle, tenths of a degree

*field, declared at [`include/shulib/hal/vision_frames.hpp:76`](../../include/shulib/hal/vision_frames.hpp#L76).*

<a id="detection-score"></a>

### `Detection::score`

```cpp
std::int32_t score = -1
```

Object only: the raw score, 0..65535 (its scale: HA-198); -1 otherwise

*field, declared at [`include/shulib/hal/vision_frames.hpp:77`](../../include/shulib/hal/vision_frames.hpp#L77).*

<a id="detection-clipped"></a>

### `Detection::clipped`

```cpp
[[nodiscard]] constexpr bool clipped(int marginPx = 1) const noexcept
```

True when the box comes within `marginPx` of an edge of the image (inclusive at both ends) — part of the thing may be outside the picture, so its size says nothing about its distance.

*function, declared at [`include/shulib/hal/vision_frames.hpp:81`](../../include/shulib/hal/vision_frames.hpp#L81).*

<a id="enum-class-framestatus"></a>

## `enum class FrameStatus`

```cpp
enum class FrameStatus : std::uint8_t
```

How a read went. ONLY `Ok` is evidence of anything.

*enum class, declared at [`include/shulib/hal/vision_frames.hpp:89`](../../include/shulib/hal/vision_frames.hpp#L89).*

<a id="framestatus-notread"></a>

### `FrameStatus::NotRead`

```cpp
NotRead = 0
```

default-constructed: no read happened

*enumerator, declared at [`include/shulib/hal/vision_frames.hpp:90`](../../include/shulib/hal/vision_frames.hpp#L90).*

<a id="framestatus-ok"></a>

### `FrameStatus::Ok`

```cpp
Ok
```

the count and every object read answered with a valid type

*enumerator, declared at [`include/shulib/hal/vision_frames.hpp:91`](../../include/shulib/hal/vision_frames.hpp#L91).*

<a id="framestatus-readfault"></a>

### `FrameStatus::ReadFault`

```cpp
ReadFault
```

a read failed — the count (PROS_ERR), or a read-back the adapter's once-a-second check could not make: NO detections — never a held frame

*enumerator, declared at [`include/shulib/hal/vision_frames.hpp:92`](../../include/shulib/hal/vision_frames.hpp#L92).*

<a id="framestatus-partialfault"></a>

### `FrameStatus::PartialFault`

```cpp
PartialFault
```

the count answered, an object read did not (a torn read, HA-195): not evidence

*enumerator, declared at [`include/shulib/hal/vision_frames.hpp:94`](../../include/shulib/hal/vision_frames.hpp#L94).*

<a id="framestatus-notconfigured"></a>

### `FrameStatus::NotConfigured`

```cpp
NotConfigured
```

the adapter's set-up has not succeeded yet (or was lost): nothing was read

*enumerator, declared at [`include/shulib/hal/vision_frames.hpp:95`](../../include/shulib/hal/vision_frames.hpp#L95).*

<a id="enum-class-modelcheck"></a>

## `enum class ModelCheck`

```cpp
enum class ModelCheck : std::uint8_t
```

Whether the camera's AI model is the one the robot file expects, checked BY NAME (the camera's own class-name table, ai_vision.h :371-389) — a model class id means nothing until this says Matches: class 3 is `yellowPin` in VEX's Override model and `blueRing` in its Classroom model (A4 register HA-197). Object detections are passed on ONLY while it reads Matches.

*enum class, declared at [`include/shulib/hal/vision_frames.hpp:102`](../../include/shulib/hal/vision_frames.hpp#L102).*

<a id="modelcheck-notasked"></a>

### `ModelCheck::NotAsked`

```cpp
NotAsked = 0
```

objects are off, or no read has asked yet

*enumerator, declared at [`include/shulib/hal/vision_frames.hpp:103`](../../include/shulib/hal/vision_frames.hpp#L103).*

<a id="modelcheck-matches"></a>

### `ModelCheck::Matches`

```cpp
Matches
```

every expected class name read back, in order

*enumerator, declared at [`include/shulib/hal/vision_frames.hpp:104`](../../include/shulib/hal/vision_frames.hpp#L104).*

<a id="modelcheck-different"></a>

### `ModelCheck::Different`

```cpp
Different
```

the camera names another table (another model is selected on it), or nothing

*enumerator, declared at [`include/shulib/hal/vision_frames.hpp:105`](../../include/shulib/hal/vision_frames.hpp#L105).*

<a id="modelcheck-unreadable"></a>

### `ModelCheck::Unreadable`

```cpp
Unreadable
```

the class-name read failed (PROS_ERR) — the camera may still be booting

*enumerator, declared at [`include/shulib/hal/vision_frames.hpp:106`](../../include/shulib/hal/vision_frames.hpp#L106).*

<a id="struct-visionframe"></a>

## `struct VisionFrame`

```cpp
struct VisionFrame
```

One READ of the camera — not a camera frame: the camera may update between the count read and the object reads (HA-195). What ONE poll saw, and when the poll finished.

*struct, declared at [`include/shulib/hal/vision_frames.hpp:111`](../../include/shulib/hal/vision_frames.hpp#L111).*

<a id="visionframe-status"></a>

### `VisionFrame::status`

```cpp
FrameStatus status = FrameStatus::NotRead
```

only Ok is evidence

*field, declared at [`include/shulib/hal/vision_frames.hpp:112`](../../include/shulib/hal/vision_frames.hpp#L112).*

<a id="visionframe-readat"></a>

### `VisionFrame::readAt`

```cpp
units::Time readAt{}
```

The clock when the read COMPLETED — not the exposure: the IMAGE is older by an UNMEASURED age (A4 register HA-192; a consumer pairing a frame with a pose widens the pose's bound for it — vision::boundAtImage).

*field, declared at [`include/shulib/hal/vision_frames.hpp:116`](../../include/shulib/hal/vision_frames.hpp#L116).*

<a id="visionframe-readseq"></a>

### `VisionFrame::readSeq`

```cpp
std::uint32_t readSeq = 0
```

the adapter's poll count — NOT a camera frame number (HA-192)

*field, declared at [`include/shulib/hal/vision_frames.hpp:117`](../../include/shulib/hal/vision_frames.hpp#L117).*

<a id="visionframe-reportedcount"></a>

### `VisionFrame::reportedCount`

```cpp
std::int32_t reportedCount = 0
```

what the camera said it holds (it may say more than 24)

*field, declared at [`include/shulib/hal/vision_frames.hpp:118`](../../include/shulib/hal/vision_frames.hpp#L118).*

<a id="visionframe-truncated"></a>

### `VisionFrame::truncated`

```cpp
bool truncated = false
```

POSSIBLY FULL: the camera reported `kMaxVisionDetections` OR MORE, so detections may exist that were never sent (a read of exactly 24 cannot show that the 25th was not the one that mattered — whether a camera can report more than 24 is A4 register HA-198's question; the design does not depend on the answer). Never evidence of absence.

*field, declared at [`include/shulib/hal/vision_frames.hpp:123`](../../include/shulib/hal/vision_frames.hpp#L123).*

<a id="visionframe-count"></a>

### `VisionFrame::count`

```cpp
std::uint8_t count = 0
```

valid entries in `detections`

*field, declared at [`include/shulib/hal/vision_frames.hpp:124`](../../include/shulib/hal/vision_frames.hpp#L124).*

<a id="visionframe-skipped"></a>

### `VisionFrame::skipped`

```cpp
std::uint8_t skipped = 0
```

object reads with no valid type, or a TAG (tags are off)

*field, declared at [`include/shulib/hal/vision_frames.hpp:125`](../../include/shulib/hal/vision_frames.hpp#L125).*

<a id="visionframe-model"></a>

### `VisionFrame::model`

```cpp
ModelCheck model = ModelCheck::NotAsked
```

the AI model's check at this read

*field, declared at [`include/shulib/hal/vision_frames.hpp:126`](../../include/shulib/hal/vision_frames.hpp#L126).*

<a id="visionframe-objectswithheld"></a>

### `VisionFrame::objectsWithheld`

```cpp
std::uint8_t objectsWithheld = 0
```

Object detections NOT passed on (the model not Matches, or an id past the table)

*field, declared at [`include/shulib/hal/vision_frames.hpp:127`](../../include/shulib/hal/vision_frames.hpp#L127).*

<a id="visionframe-colourswithheld"></a>

### `VisionFrame::coloursWithheld`

```cpp
std::uint8_t coloursWithheld = 0
```

Colour and Code detections NOT passed on: a descriptor this program did not write (a slot another program left set), or any code (VIS1 writes none).

*field, declared at [`include/shulib/hal/vision_frames.hpp:130`](../../include/shulib/hal/vision_frames.hpp#L130).*

<a id="visionframe-detections"></a>

### `VisionFrame::detections`

```cpp
Detection detections[kMaxVisionDetections]{}
```

[0, count) are valid; their order is UNVERIFIED (HA-198)

*field, declared at [`include/shulib/hal/vision_frames.hpp:131`](../../include/shulib/hal/vision_frames.hpp#L131).*

<a id="visionframe-usable"></a>

### `VisionFrame::usable`

```cpp
[[nodiscard]] constexpr bool usable() const noexcept
```

True only when this read is EVIDENCE (status Ok).

*function, declared at [`include/shulib/hal/vision_frames.hpp:134`](../../include/shulib/hal/vision_frames.hpp#L134).*

<a id="struct-colourdescriptor"></a>

## `struct ColourDescriptor`

```cpp
struct ColourDescriptor
```

One colour descriptor as the camera takes it (ai_vision.h :85-92): an id 1..7, an RGB, a hue tolerance in degrees (1..40) and a saturation tolerance (0.1..1). EVERY FIELD IS A MEASURED FACT the team types (from VEXcode's AI Vision Utility); the defaults are UNSET and refuse.

*struct, declared at [`include/shulib/hal/vision_frames.hpp:140`](../../include/shulib/hal/vision_frames.hpp#L140).*

<a id="colourdescriptor-id"></a>

### `ColourDescriptor::id`

```cpp
std::uint8_t id = 0
```

1..7; 0 = UNSET

*field, declared at [`include/shulib/hal/vision_frames.hpp:141`](../../include/shulib/hal/vision_frames.hpp#L141).*

<a id="colourdescriptor-red"></a>

### `ColourDescriptor::red`

```cpp
std::uint8_t red = 0
```

0..255

*field, declared at [`include/shulib/hal/vision_frames.hpp:142`](../../include/shulib/hal/vision_frames.hpp#L142).*

<a id="colourdescriptor-green"></a>

### `ColourDescriptor::green`

```cpp
std::uint8_t green = 0
```

0..255

*field, declared at [`include/shulib/hal/vision_frames.hpp:143`](../../include/shulib/hal/vision_frames.hpp#L143).*

<a id="colourdescriptor-blue"></a>

### `ColourDescriptor::blue`

```cpp
std::uint8_t blue = 0
```

0..255

*field, declared at [`include/shulib/hal/vision_frames.hpp:144`](../../include/shulib/hal/vision_frames.hpp#L144).*

<a id="colourdescriptor-huerange"></a>

### `ColourDescriptor::hueRange`

```cpp
float hueRange = 0.0F
```

degrees, 1..40; 0 = UNSET

*field, declared at [`include/shulib/hal/vision_frames.hpp:145`](../../include/shulib/hal/vision_frames.hpp#L145).*

<a id="colourdescriptor-saturationrange"></a>

### `ColourDescriptor::saturationRange`

```cpp
float saturationRange = 0.0F
```

0.1..1; 0 = UNSET

*field, declared at [`include/shulib/hal/vision_frames.hpp:146`](../../include/shulib/hal/vision_frames.hpp#L146).*

<a id="colourdescriptor-name"></a>

### `ColourDescriptor::name`

```cpp
const char* name = ""
```

the team's word for it ("toggle red"), for every log line

*field, declared at [`include/shulib/hal/vision_frames.hpp:147`](../../include/shulib/hal/vision_frames.hpp#L147).*

<a id="colourdescriptor-provenance"></a>

### `ColourDescriptor::provenance`

```cpp
const char* provenance = ""
```

who measured it, how, when, under what light — required

*field, declared at [`include/shulib/hal/vision_frames.hpp:148`](../../include/shulib/hal/vision_frames.hpp#L148).*

<a id="colourdescriptorproblem"></a>

## `colourDescriptorProblem`

```cpp
[[nodiscard]] constexpr const char* colourDescriptorProblem(const ColourDescriptor& c) noexcept
```

Why `c` is not acceptable, or nullptr. An all-zero descriptor (id, hue and saturation all 0) is UNSET and refuses; otherwise an id outside 1..7, a hue outside 1..40 or a saturation outside 0.1..1 (each inclusive, NaN refused), a null name, or a missing provenance each refuses by name. Nothing inside the vendored ranges is refused.

*free function, declared at [`include/shulib/hal/vision_frames.hpp:155`](../../include/shulib/hal/vision_frames.hpp#L155).*

<a id="struct-aivisionsettings"></a>

## `struct AiVisionSettings`

```cpp
struct AiVisionSettings
```

What the adapter writes to the camera: EXACTLY these modes — TAGS always OFF (VIS1 reads no tags, and Bench Tests station 12 leaves tags ON; a tag would take one of the 24 slots) — and these descriptors.

*struct, declared at [`include/shulib/hal/vision_frames.hpp:180`](../../include/shulib/hal/vision_frames.hpp#L180).*

<a id="aivisionsettings-colours"></a>

### `AiVisionSettings::colours`

```cpp
ColourDescriptor colours[kMaxColourDescriptors]{}
```

[0, colourCount) are written

*field, declared at [`include/shulib/hal/vision_frames.hpp:181`](../../include/shulib/hal/vision_frames.hpp#L181).*

<a id="aivisionsettings-colourcount"></a>

### `AiVisionSettings::colourCount`

```cpp
std::uint8_t colourCount = 0
```

0 => COLORS off

*field, declared at [`include/shulib/hal/vision_frames.hpp:182`](../../include/shulib/hal/vision_frames.hpp#L182).*

<a id="aivisionsettings-objects"></a>

### `AiVisionSettings::objects`

```cpp
bool objects = false
```

the AI model's game elements (ai_vision.h :73)

*field, declared at [`include/shulib/hal/vision_frames.hpp:183`](../../include/shulib/hal/vision_frames.hpp#L183).*

<a id="aivisionsettings-expectedclasses"></a>

### `AiVisionSettings::expectedClasses`

```cpp
const char* const* expectedClasses = nullptr
```

With `objects`: the class names the camera's model must report, in id order (the season's table — vision/override_elements.hpp's kOverrideClassNames). Required: an object id is opaque without it (ModelCheck). BORROWED: the array and its strings must outlive every adapter built from it (a `constexpr` table does).

*field, declared at [`include/shulib/hal/vision_frames.hpp:188`](../../include/shulib/hal/vision_frames.hpp#L188).*

<a id="aivisionsettings-expectedclasscount"></a>

### `AiVisionSettings::expectedClassCount`

```cpp
std::uint8_t expectedClassCount = 0
```

entries in expectedClasses, 1..kMaxVisionClassNames

*field, declared at [`include/shulib/hal/vision_frames.hpp:189`](../../include/shulib/hal/vision_frames.hpp#L189).*

<a id="aivisionsettings-colourmerge"></a>

### `AiVisionSettings::colourMerge`

```cpp
bool colourMerge = false
```

COLOR_MERGE (ai_vision.h :74)

*field, declared at [`include/shulib/hal/vision_frames.hpp:190`](../../include/shulib/hal/vision_frames.hpp#L190).*

<a id="aivisionsettingsproblem"></a>

## `aiVisionSettingsProblem`

```cpp
[[nodiscard]] constexpr const char* aiVisionSettingsProblem(const AiVisionSettings& s) noexcept
```

The first problem with a settings block, or nullptr: more than 7 colours; a colour's own problem; two colours on one id (the camera silently keeps the second write, ai_vision.h :340); COLOR_MERGE with no colours; NOTHING turned on (a camera configured to see nothing is a wiring mistake, not a choice); objects on with no class table, an empty one, more than 20 names, or a null or empty name.

*free function, declared at [`include/shulib/hal/vision_frames.hpp:197`](../../include/shulib/hal/vision_frames.hpp#L197).*

<a id="samevisiontext"></a>

## `sameVisionText`

```cpp
[[nodiscard]] constexpr bool sameVisionText(const char* a, const char* b) noexcept
```

Two C strings equal by CONTENT (two null pointers are equal; a null and a non-null are not).

*free function, declared at [`include/shulib/hal/vision_frames.hpp:234`](../../include/shulib/hal/vision_frames.hpp#L234).*

<a id="sameaivisionsettings"></a>

## `sameAiVisionSettings`

```cpp
[[nodiscard]] constexpr bool sameAiVisionSettings(const AiVisionSettings& a, const AiVisionSettings& b) noexcept
```

Whether two settings blocks would write the SAME thing to a camera: field by field, every string by content, the colours and class names only up to their counts (entries past a count are never written). The device provider serves a second request for a camera only with settings equal to the first's — two owners disagreeing about one camera is refused (hal/pros/device_provider.hpp).

*free function, declared at [`include/shulib/hal/vision_frames.hpp:251`](../../include/shulib/hal/vision_frames.hpp#L251).*

<a id="struct-camerastatus"></a>

## `struct CameraStatus`

```cpp
struct CameraStatus
```

What the camera's adapter knows about its own set-up and AI model, WITHOUT touching the device. The two strings are the adapter's own words, "" when there is nothing to say; they live as long as the adapter and change only on a read() (single-task).

*struct, declared at [`include/shulib/hal/vision_frames.hpp:280`](../../include/shulib/hal/vision_frames.hpp#L280).*

<a id="camerastatus-configured"></a>

### `CameraStatus::configured`

```cpp
bool configured = false
```

the set-up was written AND read back

*field, declared at [`include/shulib/hal/vision_frames.hpp:281`](../../include/shulib/hal/vision_frames.hpp#L281).*

<a id="camerastatus-setupproblem"></a>

### `CameraStatus::setupProblem`

```cpp
const char* setupProblem = ""
```

why the last set-up failed or was re-done ("" when it holds)

*field, declared at [`include/shulib/hal/vision_frames.hpp:282`](../../include/shulib/hal/vision_frames.hpp#L282).*

<a id="camerastatus-model"></a>

### `CameraStatus::model`

```cpp
ModelCheck model = ModelCheck::NotAsked
```

the AI model's check (NotAsked: objects off)

*field, declared at [`include/shulib/hal/vision_frames.hpp:283`](../../include/shulib/hal/vision_frames.hpp#L283).*

<a id="camerastatus-modelproblem"></a>

### `CameraStatus::modelProblem`

```cpp
const char* modelProblem = ""
```

e.g. "class 0 is 'blueBall', the robot file expects 'redBluePin'"

*field, declared at [`include/shulib/hal/vision_frames.hpp:284`](../../include/shulib/hal/vision_frames.hpp#L284).*

<a id="camerastatuswords"></a>

## `cameraStatusWords`

```cpp
[[nodiscard]] constexpr const char* cameraStatusWords(const CameraStatus& s) noexcept
```

The camera's state in a few ASCII words for a boot line or a bench page (the brain's screen turns non-ASCII into '?'): "NOT set up", "set up, objects off", "set up, model OK", "set up, model check pending" (the names did not read — re-asked once a second), "set up, model DIFFERENT". A caller that needs the reason prints `setupProblem` / `modelProblem` after it.

*free function, declared at [`include/shulib/hal/vision_frames.hpp:291`](../../include/shulib/hal/vision_frames.hpp#L291).*

<a id="class-ivisionframes"></a>

## `class IVisionFrames`

```cpp
class IVisionFrames
```

The camera, read a frame at a time. read() HITS THE DEVICE (one count read, then one read per detection; once a second, the PROS adapter's check of its settings and model; and, after a lost setting or a failed read, at most every half second, its set-up written again — A4 register HA-203), so it is not const, and reading at the rate a consumer needs is the consumer's job. Single-task by contract, like every device here.

*class, declared at [`include/shulib/hal/vision_frames.hpp:309`](../../include/shulib/hal/vision_frames.hpp#L309).*

<a id="ivisionframes-destructor-ivisionframes"></a>

### `IVisionFrames::~IVisionFrames`

```cpp
virtual ~IVisionFrames() = default
```

Polymorphic base: delete-through-base is well-defined; the provider owns every camera.

*function, declared at [`include/shulib/hal/vision_frames.hpp:312`](../../include/shulib/hal/vision_frames.hpp#L312).*

<a id="ivisionframes-ivisionframes"></a>

### `IVisionFrames::IVisionFrames`

```cpp
IVisionFrames() = default
```

Polymorphic base: delete-through-base is well-defined; the provider owns every camera. Default-constructible: the base has no state.

*function, declared at [`include/shulib/hal/vision_frames.hpp:314`](../../include/shulib/hal/vision_frames.hpp#L314).*

<a id="ivisionframes-ivisionframes-2"></a>

### `IVisionFrames::IVisionFrames (overload 2)`

```cpp
IVisionFrames(const IVisionFrames&) = default
```

Polymorphic base: delete-through-base is well-defined; the provider owns every camera. Default-constructible: the base has no state. Copyable at the base (stateless); an adapter decides for itself.

*function, declared at [`include/shulib/hal/vision_frames.hpp:316`](../../include/shulib/hal/vision_frames.hpp#L316).*

<a id="ivisionframes-ivisionframes-3"></a>

### `IVisionFrames::IVisionFrames (overload 3)`

```cpp
IVisionFrames(IVisionFrames&&) = default
```

Polymorphic base: delete-through-base is well-defined; the provider owns every camera. Default-constructible: the base has no state. Copyable at the base (stateless); an adapter decides for itself. Movable, same reasoning.

*function, declared at [`include/shulib/hal/vision_frames.hpp:318`](../../include/shulib/hal/vision_frames.hpp#L318).*

<a id="ivisionframes-operator-eq"></a>

### `IVisionFrames::operator=`

```cpp
IVisionFrames& operator=(const IVisionFrames&) = default
```

Polymorphic base: delete-through-base is well-defined; the provider owns every camera. Default-constructible: the base has no state. Copyable at the base (stateless); an adapter decides for itself. Movable, same reasoning. Copy-assignable, same reasoning.

*function, declared at [`include/shulib/hal/vision_frames.hpp:320`](../../include/shulib/hal/vision_frames.hpp#L320).*

<a id="ivisionframes-operator-eq-2"></a>

### `IVisionFrames::operator= (overload 2)`

```cpp
IVisionFrames& operator=(IVisionFrames&&) = default
```

Polymorphic base: delete-through-base is well-defined; the provider owns every camera. Default-constructible: the base has no state. Copyable at the base (stateless); an adapter decides for itself. Movable, same reasoning. Copy-assignable, same reasoning. Move-assignable, same reasoning.

*function, declared at [`include/shulib/hal/vision_frames.hpp:322`](../../include/shulib/hal/vision_frames.hpp#L322).*

<a id="ivisionframes-read"></a>

### `IVisionFrames::read`

```cpp
[[nodiscard]] virtual VisionFrame read() noexcept = 0
```

Read the camera NOW (A4 register HA-203: one call per detection, plus the count; once a second the PROS adapter's check, up to 28 more; and a set-up written again, up to 36 more). A failed read is a frame whose status says so, with NO detections — never the last good frame.

*function, declared at [`include/shulib/hal/vision_frames.hpp:327`](../../include/shulib/hal/vision_frames.hpp#L327).*

<a id="ivisionframes-status"></a>

### `IVisionFrames::status`

```cpp
[[nodiscard]] virtual CameraStatus status() const noexcept = 0
```

The adapter's set-up and model check as it last found them — never touches the device.

*function, declared at [`include/shulib/hal/vision_frames.hpp:329`](../../include/shulib/hal/vision_frames.hpp#L329).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 36 lines</summary>

```text

 IVisionFrames — the V5 AI Vision camera read a FRAME at a time (chunk VIS1-A1, 2026-10-18).

 ── An F4-ADDITIVE SIBLING of the frozen hal::IVision ───────────────────────────────
 hal/vision.hpp is untouched, byte for byte (F4, LOCKED 2026-06-19). This seam sits beside it, exactly
 as IDigitalOut (F11), IController (F13), IDigitalIn (F14) and IReadFaults (MECH-A1) were added beside
 the frozen set. It is NOT FROZEN (the Freeze Register says so out loud, row F15).

 ── Why a sibling and not IVision ───────────────────────────────────────────────────
 IVision yields a class id, a bearing and a confidence (hal/vision.hpp's ObjectObservation). It has no
 detection TYPE — a colour descriptor 1 and a model class 1 are the same `classId` —, no BOX (so no
 size, no elevation, no range) and no TIME. And the PROS AI Vision API has NO frame counter and NO
 timestamp (the vendored include/pros/ai_vision.h, read whole): freshness can only be TIME-based. So a
 frame carries the time its READ completed, stamped from the run's IClock, and a read count that
 counts the ADAPTER's polls — never camera frames (A4 register HA-192). The image itself is OLDER than
 that read by an age nobody has measured (HA-192 again): a consumer that pairs a frame with a pose
 widens the pose's bound for it (vision::boundAtImage).

 ── A FAILED READ IS A FRAME THAT SAYS SO, WITH NO DETECTIONS ──────────────────────
 Never the last good frame held. The F4 sensor adapters answer a PROS_ERR with their last good value
 (MECH-A1's read-fault channel exists because of it); a frame cannot, because a held frame would read
 as a fresh "nothing changed". Only `FrameStatus::Ok` is evidence of anything.

 ── What the adapter writes, as data ────────────────────────────────────────────────
 AiVisionSettings is the robot file's camera block's adapter half (config/camera_config.hpp carries
 it): the colour descriptors the team measured with VEXcode's AI Vision Utility, whether the AI model's
 OBJECTS are on, and the class names the model must report BY NAME before an object id means anything
 (ModelCheck; HA-197). Its refusals are named — an all-zero descriptor is UNSET, never a colour.

 ── status(): the set-up and the model check, through the SEAM ──────────────────────
 The Robot's boot line and a bench page report whether the camera was set up and whether its AI model
 matched — and the builder never reads a frame and must never downcast to the PROS adapter. So the
 seam carries `status()`, which touches nothing (VIS1's re-check T2).

 Allocation-free (a fixed 24-entry array — AIVISION_MAX_OBJECT_COUNT, ai_vision.h :33), PROS-free,
 sim-free. Single-task by contract, like every device here.
```

</details>
