<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/vision/toggle_camera.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `toggle_camera.hpp`

ToggleCamera — the robot's camera answer for the Toggle, as an AUTON reaches it.

This header declares **3** types (17 members), **3** free functions, and **1** constant.

Extracted from [`include/shulib/vision/toggle_camera.hpp`](../../include/shulib/vision/toggle_camera.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in [How it works](../guide/README.md); worked recipes live in [Recipes](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`struct ToggleCameraConfig`](#struct-togglecameraconfig)
  - [`tracker`](#togglecameraconfig-tracker)
  - [`feed`](#togglecameraconfig-feed)
  - [`declared`](#togglecameraconfig-declared)
- [`struct ToggleCameraCheck`](#struct-togglecameracheck)
  - [`part`](#togglecameracheck-part)
  - [`why`](#togglecameracheck-why)
- [`toggleIdsProblem`](#toggleidsproblem) — *free function*
- [`toggleCameraCheck`](#togglecameracheck) — *free function*
- [`toggleCameraProblem`](#togglecameraproblem) — *free function*
- [`kToggleCameraReasonBytes`](#ktogglecamerareasonbytes) — *constant*
- [`class ToggleCamera`](#class-togglecamera)
  - [`ToggleCamera`](#togglecamera-togglecamera)
  - [`ToggleCamera (overload 2)`](#togglecamera-togglecamera-2)
  - [`ToggleCamera (overload 3)`](#togglecamera-togglecamera-3)
  - [`operator=`](#togglecamera-operator-eq)
  - [`operator= (overload 2)`](#togglecamera-operator-eq-2)
  - [`~ToggleCamera`](#togglecamera-destructor-togglecamera)
  - [`build`](#togglecamera-build)
  - [`ready`](#togglecamera-ready)
  - [`feed`](#togglecamera-feed)
  - [`model`](#togglecamera-model)
  - [`tracker`](#togglecamera-tracker)
  - [`absence`](#togglecamera-absence)

<a id="struct-togglecameraconfig"></a>

## `struct ToggleCameraConfig`

```cpp
struct ToggleCameraConfig
```

The robot file's Toggle block (file banner): the team's tracker numbers and the camera feed's read period, UNSET by default. DECLARED as soon as any number in it is set — so a block typed half-way is declared and refused naming the missing number, never silently ignored.

*struct, declared at [`include/shulib/vision/toggle_camera.hpp:61`](../../include/shulib/vision/toggle_camera.hpp#L61).*

<a id="togglecameraconfig-tracker"></a>

### `ToggleCameraConfig::tracker`

```cpp
ToggleTrackerConfig tracker{}
```

The tracker's numbers (toggle_tracker.hpp — every one the TEAM's, from Step 0.3 at the robot).

*field, declared at [`include/shulib/vision/toggle_camera.hpp:63`](../../include/shulib/vision/toggle_camera.hpp#L63).*

<a id="togglecameraconfig-feed"></a>

### `ToggleCameraConfig::feed`

```cpp
CameraFeedConfig feed{}
```

The feed's read period (camera_feed.hpp — the TEAM's, about 0.033 s for 30 a second; A4 register HA-192).

*field, declared at [`include/shulib/vision/toggle_camera.hpp:66`](../../include/shulib/vision/toggle_camera.hpp#L66).*

<a id="togglecameraconfig-declared"></a>

### `ToggleCameraConfig::declared`

```cpp
[[nodiscard]] constexpr bool declared() const noexcept
```

True when any number in the block is set (a NaN counts as set: it is refused by name, not ignored). EVERY field of both configs is listed here: a field added to either must be added here too, or a block typed with only that field set is silently ignored (test/robot_toggle_camera_test.cpp sets each one alone).

*function, declared at [`include/shulib/vision/toggle_camera.hpp:72`](../../include/shulib/vision/toggle_camera.hpp#L72).*

<a id="struct-togglecameracheck"></a>

## `struct ToggleCameraCheck`

```cpp
struct ToggleCameraCheck
```

What is wrong with a Toggle block for a camera, if anything: `part` names the piece ("" when the sentence names it already) and `why` is the sentence, nullptr when nothing is wrong.

*struct, declared at [`include/shulib/vision/toggle_camera.hpp:92`](../../include/shulib/vision/toggle_camera.hpp#L92).*

<a id="togglecameracheck-part"></a>

### `ToggleCameraCheck::part`

```cpp
const char* part = ""
```

The piece the sentence is about, as a prefix ("the Toggle block's tracker: "), or "".

*field, declared at [`include/shulib/vision/toggle_camera.hpp:94`](../../include/shulib/vision/toggle_camera.hpp#L94).*

<a id="togglecameracheck-why"></a>

### `ToggleCameraCheck::why`

```cpp
const char* why = nullptr
```

The reason, or nullptr: the block may be built.

*field, declared at [`include/shulib/vision/toggle_camera.hpp:96`](../../include/shulib/vision/toggle_camera.hpp#L96).*

<a id="toggleidsproblem"></a>

## `toggleIdsProblem`

```cpp
[[nodiscard]] constexpr const char* toggleIdsProblem(const hal::AiVisionSettings& settings, const ToggleColourIds& ids) noexcept
```

Whether every one of the tracker's three colour ids is a colour the camera's settings WRITE (the camera fires only on a colour it was taught): nullptr, or the first id that is not.

*free function, declared at [`include/shulib/vision/toggle_camera.hpp:101`](../../include/shulib/vision/toggle_camera.hpp#L101).*

<a id="togglecameracheck"></a>

## `toggleCameraCheck`

```cpp
[[nodiscard]] constexpr ToggleCameraCheck toggleCameraCheck(const config::CameraConfig& camera, const ToggleCameraConfig& block) noexcept
```

THE ONE ORDER of the block's checks (file banner, "When the slot is READY", less the camera being BUILT, which only the Robot knows): the block declared; a camera declared and its mount measured and inside the geometry's limits (cameraModelProblem); the tracker's numbers; the feed's; the three ids among the camera's colours. Pure, total, constexpr — a robot file may `static_assert` it.

*free function, declared at [`include/shulib/vision/toggle_camera.hpp:133`](../../include/shulib/vision/toggle_camera.hpp#L133).*

<a id="togglecameraproblem"></a>

## `toggleCameraProblem`

```cpp
[[nodiscard]] constexpr const char* toggleCameraProblem(const config::CameraConfig& camera, const ToggleCameraConfig& block) noexcept
```

toggleCameraCheck()'s sentence alone (nullptr when the block may be built) — for a `static_assert`.

*free function, declared at [`include/shulib/vision/toggle_camera.hpp:154`](../../include/shulib/vision/toggle_camera.hpp#L154).*

<a id="ktogglecamerareasonbytes"></a>

## `kToggleCameraReasonBytes`

```cpp
inline constexpr std::size_t kToggleCameraReasonBytes = 224
```

The longest absence sentence the slot keeps, NUL included (a part, then a sentence of the library's).

*constant, declared at [`include/shulib/vision/toggle_camera.hpp:160`](../../include/shulib/vision/toggle_camera.hpp#L160).*

<a id="class-togglecamera"></a>

## `class ToggleCamera`

```cpp
class ToggleCamera
```

The SLOT (file banner): built EMPTY — not ready, "not built yet" — and filled once, by build(). Neither copyable nor movable: its feed holds references, and the context points at it.

*class, declared at [`include/shulib/vision/toggle_camera.hpp:164`](../../include/shulib/vision/toggle_camera.hpp#L164).*

<a id="togglecamera-togglecamera"></a>

### `ToggleCamera::ToggleCamera`

```cpp
ToggleCamera() noexcept = default
```

Not ready: "not built yet".

*function, declared at [`include/shulib/vision/toggle_camera.hpp:167`](../../include/shulib/vision/toggle_camera.hpp#L167).*

<a id="togglecamera-togglecamera-2"></a>

### `ToggleCamera::ToggleCamera (overload 2)`

```cpp
ToggleCamera(const ToggleCamera&) = delete
```

Not ready: "not built yet". Non-copyable (the context points at this object).

*function, declared at [`include/shulib/vision/toggle_camera.hpp:169`](../../include/shulib/vision/toggle_camera.hpp#L169).*

<a id="togglecamera-togglecamera-3"></a>

### `ToggleCamera::ToggleCamera (overload 3)`

```cpp
ToggleCamera(ToggleCamera&&) = delete
```

Not ready: "not built yet". Non-copyable (the context points at this object). Non-movable, same reason.

*function, declared at [`include/shulib/vision/toggle_camera.hpp:171`](../../include/shulib/vision/toggle_camera.hpp#L171).*

<a id="togglecamera-operator-eq"></a>

### `ToggleCamera::operator=`

```cpp
ToggleCamera& operator=(const ToggleCamera&) = delete
```

Not ready: "not built yet". Non-copyable (the context points at this object). Non-movable, same reason. Non-copy-assignable.

*function, declared at [`include/shulib/vision/toggle_camera.hpp:173`](../../include/shulib/vision/toggle_camera.hpp#L173).*

<a id="togglecamera-operator-eq-2"></a>

### `ToggleCamera::operator= (overload 2)`

```cpp
ToggleCamera& operator=(ToggleCamera&&) = delete
```

Not ready: "not built yet". Non-copyable (the context points at this object). Non-movable, same reason. Non-copy-assignable. Non-move-assignable.

*function, declared at [`include/shulib/vision/toggle_camera.hpp:175`](../../include/shulib/vision/toggle_camera.hpp#L175).*

<a id="togglecamera-destructor-togglecamera"></a>

### `ToggleCamera::~ToggleCamera`

```cpp
~ToggleCamera() = default
```

Not ready: "not built yet". Non-copyable (the context points at this object). Non-movable, same reason. Non-copy-assignable. Non-move-assignable. Nothing to release: the camera and the clock are borrowed.

*function, declared at [`include/shulib/vision/toggle_camera.hpp:177`](../../include/shulib/vision/toggle_camera.hpp#L177).*

<a id="togglecamera-build"></a>

### `ToggleCamera::build`

```cpp
void build(hal::IVisionFrames* camera, const hal::IClock& clock, const config::CameraConfig& cameraConfig, const ToggleCameraConfig& block)
```

Fill the slot ONCE: READY when toggleCameraCheck(cameraConfig, block) passes AND `camera` (the robot's BUILT camera, nullptr when it was not built) is non-null — the feed over `camera` and `clock` with the block's read period, the model of the config's mount, the block's tracker numbers. Otherwise NOT READY with the first reason (the checks' order, then "the camera was not built"). Reads nothing from the camera. `camera` and `clock` must outlive this object. A second call is a programming error (a precondition: a slot is filled once, at boot).

*function, declared at [`include/shulib/vision/toggle_camera.hpp:185`](../../include/shulib/vision/toggle_camera.hpp#L185).*

<a id="togglecamera-ready"></a>

### `ToggleCamera::ready`

```cpp
[[nodiscard]] bool ready() const noexcept
```

True when build() built it: the feed, the model and the tracker numbers are there.

*function, declared at [`include/shulib/vision/toggle_camera.hpp:206`](../../include/shulib/vision/toggle_camera.hpp#L206).*

<a id="togglecamera-feed"></a>

### `ToggleCamera::feed`

```cpp
[[nodiscard]] CameraFeed* feed() noexcept
```

The feed, ticked only inside the step's own waits — nullptr when not ready.

*function, declared at [`include/shulib/vision/toggle_camera.hpp:208`](../../include/shulib/vision/toggle_camera.hpp#L208).*

<a id="togglecamera-model"></a>

### `ToggleCamera::model`

```cpp
[[nodiscard]] const CameraModel* model() const noexcept
```

The model of the measured mount — nullptr when not ready.

*function, declared at [`include/shulib/vision/toggle_camera.hpp:210`](../../include/shulib/vision/toggle_camera.hpp#L210).*

<a id="togglecamera-tracker"></a>

### `ToggleCamera::tracker`

```cpp
[[nodiscard]] const ToggleTrackerConfig& tracker() const noexcept
```

The team's tracker numbers (all UNSET when not ready).

*function, declared at [`include/shulib/vision/toggle_camera.hpp:212`](../../include/shulib/vision/toggle_camera.hpp#L212).*

<a id="togglecamera-absence"></a>

### `ToggleCamera::absence`

```cpp
[[nodiscard]] const char* absence() const noexcept
```

Why it is not ready — "" EXACTLY when ready(); "not built yet" before build(). Stable for the object's lifetime once built.

*function, declared at [`include/shulib/vision/toggle_camera.hpp:215`](../../include/shulib/vision/toggle_camera.hpp#L215).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 40 lines</summary>

```text

 ToggleCamera — the robot's camera answer for the Toggle, as an AUTON reaches it (chunk AF1S-A2,
 2026-11-15; the AF1S brief's §1.3 — L6-A2's optional-slot pattern, the context's SECOND optional slot).

 ── Two pieces, one header ──────────────────────────────────────────────────────────────────────
   * ToggleCameraConfig — the robot file's TOGGLE BLOCK (config::SensorsConfig::toggleCamera): the TEAM's
     tracker numbers (vision::ToggleTrackerConfig) and the camera feed's read period
     (vision::CameraFeedConfig), every one UNSET by default, so every robot file written before it
     builds the same robot. It sits BESIDE the camera's block, not inside it: config/camera_config.hpp
     is included by the tracker's and the feed's own headers, so config::CameraConfig cannot hold their
     types (a circular include — the brief's §1.3 named CameraConfig; its re-check R9 met the same wall
     for the Quadrant colours).
   * ToggleCamera — the SLOT: ONE vision::CameraFeed over the robot's camera, the vision::CameraModel of
     its MEASURED mount, and the tracker numbers. robot::Robot fills it; the context's optional
     `toggleCamera` slot (chassis/robot_context.hpp) points at it; the everyday
     competition::checkAndFlipToggle(ctx, ...) reads it through `ctx.chassis.deps().ctx`, exactly as the
     everyday wall reset reads the wall sensors. It holds the tracker's CONFIG, not a tracker: the step
     builds a FRESH tracker for every visit (AF1S-A1), so the hold that counts always began inside it.

 ── When the slot is READY ──────────────────────────────────────────────────────────────────────
 The block DECLARED (any number in it set), toggleCameraProblem() nullptr — a camera declared; its mount
 MEASURED and inside the geometry's limits (vision::cameraModelProblem); the tracker's numbers and the
 feed's accepted (toggleTrackerConfigProblem, cameraFeedConfigProblem); the tracker's three colour ids
 among the colours the camera's settings write (an id the camera is never taught would read UNKNOWN
 for ever) — AND the camera BUILT by the provider. Otherwise NOT READY, with the FIRST reason in that
 order (absence()). A refusal never refuses the robot: an unusable Toggle block costs the skills step its
 camera (NO CAMERA, the reason in its [TGL] line), never the robot its drive — the camera's own rule
 (VIS1's D5) and the brief's mutation M-A2-2. config::RobotConfig::valid() does not check the block.

 ── Why the slot is "declared, then ready" — not "built, or null" ───────────────────────────────
 The brief has the context's slot null on any refusal. It cannot be: motion::MotionScheduler's shadow
 context copies every slot pointer when the CHASSIS is built, inside robot::Robot's member initializers,
 and the camera is built in the Robot's BODY, last (VIS1-A1's order: every provider request before it
 stays where it was). So — L6-A2's wall-sensor slot exactly — the context's pointer is decided by the
 CONFIG (the block declared), the object it points at is filled in the body, and ready() says whether it
 was built (a wall side's sensor is ABSENT inside a present slot the same way). robot::Robot's own
 toggleCamera() is the brief's "built, or null", with toggleCameraAbsence() its sentence.

 PROS-free, sim-free, allocation-free; single-task. The slot never reads the camera by itself: only the
 step's own waits tick its feed. NOTHING HERE HAS RUN ON A ROBOT.
```

</details>
