<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/config/camera_config.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `camera_config.hpp`

CameraConfig — the robot file's AI VISION CAMERA block.

This header declares **2** types (15 members) and **2** free functions.

Extracted from [`include/shulib/config/camera_config.hpp`](../../include/shulib/config/camera_config.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in [How it works](../guide/README.md); worked recipes live in [Recipes](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`struct CameraPlacement`](#struct-cameraplacement)
  - [`measured`](#cameraplacement-measured)
  - [`forward`](#cameraplacement-forward)
  - [`left`](#cameraplacement-left)
  - [`height`](#cameraplacement-height)
  - [`yawDeg`](#cameraplacement-yawdeg)
  - [`pitchDeg`](#cameraplacement-pitchdeg)
  - [`positionTolerance`](#cameraplacement-positiontolerance)
  - [`heightTolerance`](#cameraplacement-heighttolerance)
  - [`yawToleranceDeg`](#cameraplacement-yawtolerancedeg)
  - [`pitchToleranceDeg`](#cameraplacement-pitchtolerancedeg)
  - [`provenance`](#cameraplacement-provenance)
- [`finiteNumber`](#finitenumber) — *free function*
- [`cameraPlacementProblem`](#cameraplacementproblem) — *free function*
- [`struct CameraConfig`](#struct-cameraconfig)
  - [`port`](#cameraconfig-port)
  - [`settings`](#cameraconfig-settings)
  - [`placement`](#cameraconfig-placement)
  - [`note`](#cameraconfig-note)

<a id="struct-cameraplacement"></a>

## `struct CameraPlacement`

```cpp
struct CameraPlacement
```

Where the camera's LENS sits and where its optical axis points, in the body frame (+X forward, +Y LEFT, +Z up from the floor) — MEASURED by the team, typed once per robot with how well each number was measured. `measured == false` (the default) is UNSET (header).

*struct, declared at [`include/shulib/config/camera_config.hpp:38`](../../include/shulib/config/camera_config.hpp#L38).*

<a id="cameraplacement-measured"></a>

### `CameraPlacement::measured`

```cpp
bool measured = false
```

true only when every field below was measured

*field, declared at [`include/shulib/config/camera_config.hpp:39`](../../include/shulib/config/camera_config.hpp#L39).*

<a id="cameraplacement-forward"></a>

### `CameraPlacement::forward`

```cpp
units::Length forward{}
```

the lens FORWARD of the tracking centre

*field, declared at [`include/shulib/config/camera_config.hpp:40`](../../include/shulib/config/camera_config.hpp#L40).*

<a id="cameraplacement-left"></a>

### `CameraPlacement::left`

```cpp
units::Length left{}
```

the lens LEFT of the tracking centre

*field, declared at [`include/shulib/config/camera_config.hpp:41`](../../include/shulib/config/camera_config.hpp#L41).*

<a id="cameraplacement-height"></a>

### `CameraPlacement::height`

```cpp
units::Length height{}
```

the lens above the FLOOR (the field tiles), to the lens centre

*field, declared at [`include/shulib/config/camera_config.hpp:42`](../../include/shulib/config/camera_config.hpp#L42).*

<a id="cameraplacement-yawdeg"></a>

### `CameraPlacement::yawDeg`

```cpp
double yawDeg = 0.0
```

the optical axis, CCW from the robot's +X (0 = straight ahead), degrees

*field, declared at [`include/shulib/config/camera_config.hpp:43`](../../include/shulib/config/camera_config.hpp#L43).*

<a id="cameraplacement-pitchdeg"></a>

### `CameraPlacement::pitchDeg`

```cpp
double pitchDeg = 0.0
```

the optical axis above the horizontal (+ = tilted UP), degrees

*field, declared at [`include/shulib/config/camera_config.hpp:44`](../../include/shulib/config/camera_config.hpp#L44).*

<a id="cameraplacement-positiontolerance"></a>

### `CameraPlacement::positionTolerance`

```cpp
units::Length positionTolerance{}
```

+- on forward and left (a tape: about 0.25 in)

*field, declared at [`include/shulib/config/camera_config.hpp:45`](../../include/shulib/config/camera_config.hpp#L45).*

<a id="cameraplacement-heighttolerance"></a>

### `CameraPlacement::heightTolerance`

```cpp
units::Length heightTolerance{}
```

+- on height

*field, declared at [`include/shulib/config/camera_config.hpp:46`](../../include/shulib/config/camera_config.hpp#L46).*

<a id="cameraplacement-yawtolerancedeg"></a>

### `CameraPlacement::yawToleranceDeg`

```cpp
double yawToleranceDeg = 0.0
```

+- on yaw, degrees

*field, declared at [`include/shulib/config/camera_config.hpp:47`](../../include/shulib/config/camera_config.hpp#L47).*

<a id="cameraplacement-pitchtolerancedeg"></a>

### `CameraPlacement::pitchToleranceDeg`

```cpp
double pitchToleranceDeg = 0.0
```

+- on pitch, degrees (a phone level on the camera's back face: about 1; the geometry refuses a pitch it takes past 55: vision::kMaxPitchSpanDeg)

*field, declared at [`include/shulib/config/camera_config.hpp:48`](../../include/shulib/config/camera_config.hpp#L48).*

<a id="cameraplacement-provenance"></a>

### `CameraPlacement::provenance`

```cpp
const char* provenance = ""
```

who measured, how, when — and the firmware versions — required when measured

*field, declared at [`include/shulib/config/camera_config.hpp:50`](../../include/shulib/config/camera_config.hpp#L50).*

<a id="finitenumber"></a>

## `finiteNumber`

```cpp
[[nodiscard]] constexpr bool finiteNumber(double v) noexcept
```

True when `v` is a finite number (constexpr: neither NaN nor an infinity).

*free function, declared at [`include/shulib/config/camera_config.hpp:54`](../../include/shulib/config/camera_config.hpp#L54).*

<a id="cameraplacementproblem"></a>

## `cameraPlacementProblem`

```cpp
[[nodiscard]] constexpr const char* cameraPlacementProblem(const CameraPlacement& p) noexcept
```

The first problem with a placement, or nullptr: UNSET; a position that is not a number; a height outside (0, 36) in; a pitch outside (-45, 45) degrees; a yaw outside (-180, 180]; a tolerance that is zero, negative, not a number or infinite (nothing is measured perfectly — a zero would make every interval a point and every refusal impossible); no provenance.

*free function, declared at [`include/shulib/config/camera_config.hpp:62`](../../include/shulib/config/camera_config.hpp#L62).*

<a id="struct-cameraconfig"></a>

## `struct CameraConfig`

```cpp
struct CameraConfig
```

The robot's AI Vision camera: its port, what the adapter writes to it, and where it is (header).

*struct, declared at [`include/shulib/config/camera_config.hpp:92`](../../include/shulib/config/camera_config.hpp#L92).*

<a id="cameraconfig-port"></a>

### `CameraConfig::port`

```cpp
std::uint8_t port = 0
```

1..21; 0 = this robot has no camera

*field, declared at [`include/shulib/config/camera_config.hpp:93`](../../include/shulib/config/camera_config.hpp#L93).*

<a id="cameraconfig-settings"></a>

### `CameraConfig::settings`

```cpp
hal::AiVisionSettings settings{}
```

what the adapter writes (colours, objects, the class table)

*field, declared at [`include/shulib/config/camera_config.hpp:94`](../../include/shulib/config/camera_config.hpp#L94).*

<a id="cameraconfig-placement"></a>

### `CameraConfig::placement`

```cpp
CameraPlacement placement{}
```

UNSET until it is measured

*field, declared at [`include/shulib/config/camera_config.hpp:95`](../../include/shulib/config/camera_config.hpp#L95).*

<a id="cameraconfig-note"></a>

### `CameraConfig::note`

```cpp
const char* note = ""
```

e.g. "front, centred" — for a boot line and the panel

*field, declared at [`include/shulib/config/camera_config.hpp:96`](../../include/shulib/config/camera_config.hpp#L96).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 25 lines</summary>

```text

 CameraConfig — the robot file's AI VISION CAMERA block (chunk VIS1-A1, 2026-10-18).

 ── DATA the robot file carries, so it lives in config/ ─────────────────────────────
 The robot file is the one place a robot's facts live (MECH's Q1 (a)); the camera's facts are its
 port, what the library writes to it (the colour descriptors the team measured, the AI model's
 OBJECTS on or off, the class names the model must report — hal::AiVisionSettings), and where it sits
 (CameraPlacement, measured). config/robot_config.hpp carries only the hooks: `SensorsConfig::camera`,
 `PortUse::Camera`, and the camera's checks in `RobotConfig::valid()`. G3a's robot panel writes this
 block like every other.

 ── The placement is measured from the robot's TRACKING CENTRE ──────────────────────
 The point the robot's pose names (guide chapter 2, "Where does the robot's position actually
 point?"): every consumer turns the camera's bearing and range into a FIELD position by adding them to
 that pose, through this placement. On a tank with drive-encoder odometry and no tracking pods it is
 the point the robot turns about in place.

 ── UNSET is not an error ───────────────────────────────────────────────────────────
 `CameraPlacement::measured == false` (the default) is UNSET: the camera is still built and read (its
 colours and objects need no mount), and every GEOMETRY consumer refuses to construct, naming it
 (vision::cameraModelProblem). So the team can bring a camera up a piece at a time — the objects
 first, the colours once they are measured, the mount once it is measured — and no value is ever
 invented to make a block "complete" (HA-111's lesson).

 PROS-free, sim-free, allocation-free, constexpr-constructible.
```

</details>
