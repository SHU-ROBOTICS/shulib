<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/chassis/robot_context.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `robot_context.hpp`

RobotContext — the composition root / DI container: "the one object that differs across robot / sim / test".

This header declares **2** types (21 members).

Extracted from [`include/shulib/chassis/robot_context.hpp`](../../include/shulib/chassis/robot_context.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in [How it works](../guide/README.md); worked recipes live in [Recipes](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`struct RobotContextConfig`](#struct-robotcontextconfig)
  - [`clock`](#robotcontextconfig-clock)
  - [`driveMotors`](#robotcontextconfig-drivemotors)
  - [`imu`](#robotcontextconfig-imu)
  - [`gps`](#robotcontextconfig-gps)
  - [`battery`](#robotcontextconfig-battery)
  - [`telemetry`](#robotcontextconfig-telemetry)
  - [`tags`](#robotcontextconfig-tags)
  - [`vision`](#robotcontextconfig-vision)
  - [`walls`](#robotcontextconfig-walls)
  - [`toggleCamera`](#robotcontextconfig-togglecamera)
- [`class RobotContext`](#class-robotcontext)
  - [`RobotContext`](#robotcontext-robotcontext)
  - [`clock`](#robotcontext-clock)
  - [`driveMotors`](#robotcontext-drivemotors)
  - [`imu`](#robotcontext-imu)
  - [`gps`](#robotcontext-gps)
  - [`battery`](#robotcontext-battery)
  - [`telemetry`](#robotcontext-telemetry)
  - [`tags`](#robotcontext-tags)
  - [`vision`](#robotcontext-vision)
  - [`wallSensors`](#robotcontext-wallsensors)
  - [`toggleCamera`](#robotcontext-togglecamera)

<a id="struct-robotcontextconfig"></a>

## `struct RobotContextConfig`

```cpp
struct RobotContextConfig
```

The HAL handles a robot is built from. Pointers so fields are NAMED at the call site (designated initializers) and so the RobotContext can validate them non-null.

*struct, declared at [`include/shulib/chassis/robot_context.hpp:64`](../../include/shulib/chassis/robot_context.hpp#L64).*

<a id="robotcontextconfig-clock"></a>

### `RobotContextConfig::clock`

```cpp
hal::IClock* clock = nullptr
```

the single source of now; seconds, monotonic

*field, declared at [`include/shulib/chassis/robot_context.hpp:65`](../../include/shulib/chassis/robot_context.hpp#L65).*

<a id="robotcontextconfig-drivemotors"></a>

### `RobotContextConfig::driveMotors`

```cpp
std::span<hal::IMotor* const> driveMotors
```

The drive motors, in kinematic wheel order: element i is commanded with wheel speed i of the installed IKinematics, so a wrong order drives the robot the wrong way in silence. ONE entry per kinematic wheel — N physical motors on a side go behind one hal::MotorGroup (R3b Part 1). This context checks only that the span is non-empty and all-non-null; motion::MotionDeps::validate() is where the count is checked, and it demands EQUALITY with the kinematics' wheel count. This is a non-owning VIEW: the pointer ARRAY must outlive the context.

*field, declared at [`include/shulib/chassis/robot_context.hpp:73`](../../include/shulib/chassis/robot_context.hpp#L73).*

<a id="robotcontextconfig-imu"></a>

### `RobotContextConfig::imu`

```cpp
hal::IImu* imu = nullptr
```

heading and yaw rate, canonical CCW radians

*field, declared at [`include/shulib/chassis/robot_context.hpp:74`](../../include/shulib/chassis/robot_context.hpp#L74).*

<a id="robotcontextconfig-gps"></a>

### `RobotContextConfig::gps`

```cpp
hal::IGps* gps = nullptr
```

absolute fix; off-strip its hasFix() reads false

*field, declared at [`include/shulib/chassis/robot_context.hpp:75`](../../include/shulib/chassis/robot_context.hpp#L75).*

<a id="robotcontextconfig-battery"></a>

### `RobotContextConfig::battery`

```cpp
hal::IBattery* battery = nullptr
```

volts: the pipeline's ceiling, and the run bookends

*field, declared at [`include/shulib/chassis/robot_context.hpp:76`](../../include/shulib/chassis/robot_context.hpp#L76).*

<a id="robotcontextconfig-telemetry"></a>

### `RobotContextConfig::telemetry`

```cpp
hal::ITelemetrySink* telemetry = nullptr
```

every log line and DebugRecord; NullSink = off

*field, declared at [`include/shulib/chassis/robot_context.hpp:77`](../../include/shulib/chassis/robot_context.hpp#L77).*

<a id="robotcontextconfig-tags"></a>

### `RobotContextConfig::tags`

```cpp
hal::ITagSource* tags = nullptr
```

AprilTags as body-frame poses (the M3 corrector)

*field, declared at [`include/shulib/chassis/robot_context.hpp:78`](../../include/shulib/chassis/robot_context.hpp#L78).*

<a id="robotcontextconfig-vision"></a>

### `RobotContextConfig::vision`

```cpp
hal::IVision* vision = nullptr
```

object bearings for M4 manipulation targeting

*field, declared at [`include/shulib/chassis/robot_context.hpp:79`](../../include/shulib/chassis/robot_context.hpp#L79).*

<a id="robotcontextconfig-walls"></a>

### `RobotContextConfig::walls`

```cpp
localization::WallSensors* walls = nullptr
```

OPTIONAL (L6-A2, appended): the robot's side distance sensors, their limits and the reset log — nullptr = this robot declares no wall sensors, and the wall reset refuses NO SENSOR. The ONE slot the constructor does NOT require. Non-const: the everyday wall reset files its outcome in the slot's log. Must outlive the context when set.

*field, declared at [`include/shulib/chassis/robot_context.hpp:84`](../../include/shulib/chassis/robot_context.hpp#L84).*

<a id="robotcontextconfig-togglecamera"></a>

### `RobotContextConfig::toggleCamera`

```cpp
vision::ToggleCamera* toggleCamera = nullptr
```

OPTIONAL (AF1S-A2, appended): the robot's Toggle camera — one feed over the camera, the model of its measured mount, the team's tracker numbers (vision/toggle_camera.hpp); nullptr = this robot file declares no Toggle block, and the everyday Toggle step says NO CAMERA. Not required by the constructor, like `walls`. Non-const: the step ticks its feed. Must outlive the context when set.

*field, declared at [`include/shulib/chassis/robot_context.hpp:89`](../../include/shulib/chassis/robot_context.hpp#L89).*

<a id="class-robotcontext"></a>

## `class RobotContext`

```cpp
class RobotContext
```

The composition root: the ONE object that differs between the real robot, the simulator and a host test. Every layer above the HAL reaches hardware only through it, so exchanging the HAL implementations exchanges the whole robot without touching a line of motion code.  NON-OWNING throughout. It copies the config's pointers and span; it never adopts, allocates or destroys anything, so every pointee — and the array the driveMotors span views — must outlive the context. It also caches nothing: each accessor hands back the live handle, so a reading is only ever as fresh as the caller's own call.

*class, declared at [`include/shulib/chassis/robot_context.hpp:100`](../../include/shulib/chassis/robot_context.hpp#L100).*

<a id="robotcontext-robotcontext"></a>

### `RobotContext::RobotContext`

```cpp
explicit RobotContext(const RobotContextConfig& cfg)
```

Validates the whole config up front — every REQUIRED handle non-null and driveMotors non-empty (`walls` and `toggleCamera` are optional and never checked) — through SHULIB_PRECONDITION, so a mis-wired robot fails at construction naming the handle it is missing, instead of dereferencing null halfway through an auton. The count of drive motors is checked here only for emptiness; motion::MotionDeps::validate() checks it for EQUALITY against the kinematics' wheel count (the context never sees the kinematics).

*function, declared at [`include/shulib/chassis/robot_context.hpp:107`](../../include/shulib/chassis/robot_context.hpp#L107).*

<a id="robotcontext-clock"></a>

### `RobotContext::clock`

```cpp
[[nodiscard]] hal::IClock& clock() const
```

The run's clock. A reference, never null — the constructor already proved that, which is why consumers write `ctx.clock().now()` and never test a pointer.

*function, declared at [`include/shulib/chassis/robot_context.hpp:123`](../../include/shulib/chassis/robot_context.hpp#L123).*

<a id="robotcontext-drivemotors"></a>

### `RobotContext::driveMotors`

```cpp
[[nodiscard]] std::span<hal::IMotor* const> driveMotors() const
```

The drive motors, in kinematic wheel order. A VIEW of the caller's array, so it is only as alive as that array; guaranteed non-empty here, and guaranteed to EQUAL the wheel count of the installed kinematics once a motion::MotionDeps has validated the pairing.

*function, declared at [`include/shulib/chassis/robot_context.hpp:127`](../../include/shulib/chassis/robot_context.hpp#L127).*

<a id="robotcontext-imu"></a>

### `RobotContext::imu`

```cpp
[[nodiscard]] hal::IImu& imu() const
```

The IMU, already canonical: CCW-positive radians, +X = 0. Nothing above this call converts an angle. Gate trust on isReady() at boot — a calibrating IMU reports garbage that moves.

*function, declared at [`include/shulib/chassis/robot_context.hpp:130`](../../include/shulib/chassis/robot_context.hpp#L130).*

<a id="robotcontext-gps"></a>

### `RobotContext::gps`

```cpp
[[nodiscard]] hal::IGps& gps() const
```

The GPS. Check hasFix() before believing pose(): off-strip the pose is unspecified (still finite), and Driving Skills has no strip at all, so this seam is silent for a whole run.

*function, declared at [`include/shulib/chassis/robot_context.hpp:133`](../../include/shulib/chassis/robot_context.hpp#L133).*

<a id="robotcontext-battery"></a>

### `RobotContext::battery`

```cpp
[[nodiscard]] hal::IBattery& battery() const
```

The battery. Read live at each use — the command pipeline takes its voltage ceiling from it every tick and the run summary samples it at both ends; nothing here caches a volt.

*function, declared at [`include/shulib/chassis/robot_context.hpp:136`](../../include/shulib/chassis/robot_context.hpp#L136).*

<a id="robotcontext-telemetry"></a>

### `RobotContext::telemetry`

```cpp
[[nodiscard]] hal::ITelemetrySink& telemetry() const
```

The one diagnostics sink for this robot. Which sink is installed is what decides whether tracing costs anything: with NullSink the per-tick record is never even populated.

*function, declared at [`include/shulib/chassis/robot_context.hpp:139`](../../include/shulib/chassis/robot_context.hpp#L139).*

<a id="robotcontext-tags"></a>

### `RobotContext::tags`

```cpp
[[nodiscard]] hal::ITagSource& tags() const
```

The AprilTag source — V5 AI Vision or a coprocessor, indistinguishable from here. Tags arrive as body-frame poses with no timestamp; staleness is the corrector's problem.

*function, declared at [`include/shulib/chassis/robot_context.hpp:142`](../../include/shulib/chassis/robot_context.hpp#L142).*

<a id="robotcontext-vision"></a>

### `RobotContext::vision`

```cpp
[[nodiscard]] hal::IVision& vision() const
```

The object/colour detection source. A separate seam from tags() so one adapter can serve both, or either alone, without a consumer of one depending on the other.

*function, declared at [`include/shulib/chassis/robot_context.hpp:145`](../../include/shulib/chassis/robot_context.hpp#L145).*

<a id="robotcontext-wallsensors"></a>

### `RobotContext::wallSensors`

```cpp
[[nodiscard]] localization::WallSensors* wallSensors() const noexcept
```

The robot's side distance sensors (L6-A2) — nullptr when this robot declares none (the one OPTIONAL slot; the wall reset then refuses NO SENSOR). A pointer, as handed in: this context never dereferences it.

*function, declared at [`include/shulib/chassis/robot_context.hpp:149`](../../include/shulib/chassis/robot_context.hpp#L149).*

<a id="robotcontext-togglecamera"></a>

### `RobotContext::toggleCamera`

```cpp
[[nodiscard]] vision::ToggleCamera* toggleCamera() const noexcept
```

The robot's Toggle camera (AF1S-A2) — nullptr when this robot file declares no Toggle block (the second OPTIONAL slot; the everyday Toggle step then says NO CAMERA). A declared slot may still be NOT READY (its camera absent, its mount unmeasured — its own absence sentence says which). A pointer, as handed in: this context never dereferences it.

*function, declared at [`include/shulib/chassis/robot_context.hpp:154`](../../include/shulib/chassis/robot_context.hpp#L154).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 34 lines</summary>

```text

 RobotContext — the composition root / DI container: "the one object that differs across
 robot / sim / test" (master plan §5). Every layer above L0 (Chassis, Localizer, motion,
 skills) reads hardware ONLY through here, so swapping the HAL implementations
 (hal/fake ↔ hal/pros ↔ hal/sim) swaps the whole robot without touching a line of motion
 code — which is exactly the M1 Definition of Done.

 L3, PROS-free: it sees only the L0 interfaces. NOT part of the F4 freeze — it grows
 additively as M3/M4 consumers need more sensors (distance / optical / tracking-wheel
 arrays), so adding fields here later is safe.

 Built from a RobotContextConfig of NAMED pointers (designated initializers at the call
 site); the constructor validates every REQUIRED one non-null, then the accessors hand out
 references so consumers write `ctx.clock().now()` rather than juggling pointers.

 ── The first OPTIONAL slot: the wall sensors (chunk L6-A2, 2026-11-03) ─────────────
 `walls` is the one slot the constructor does NOT require: nullptr means "this robot declares no
 wall sensors", and the wall reset then refuses NO SENSOR — so every existing caller (the host
 harness, the rigs, the Robot before L6-A2) compiles and runs unchanged. It is a pointer handed
 out as a pointer (`wallSensors()`), never a reference, because null is a real answer here. This
 context only FORWARD-DECLARES its type and never dereferences it (it sees only the L0 interfaces;
 the type lives in localization/wall_reset.hpp). The pointer is NON-CONST: the everyday wall reset
 files each outcome in the slot's log. motion::MotionScheduler's shadow context carries it field by
 field like every other slot — a slot not carried there is nullptr in every `chassis.deps().ctx`.

 ── The second OPTIONAL slot: the Toggle camera (chunk AF1S-A2, 2026-11-15) ──────────────
 `toggleCamera` is the robot's camera answer for the skills Toggle step: one feed over the camera, the
 model of its measured mount and the team's tracker numbers (vision/toggle_camera.hpp). Exactly the
 wall slot's rules: nullptr means "this robot file declares no Toggle block" — the everyday step then
 says NO CAMERA — and every existing caller compiles and runs unchanged; a pointer handed out as a
 pointer (`toggleCamera()`), forward-declared and never dereferenced here; NON-CONST (the step ticks
 its feed); carried field by field by the motion scheduler's shadow context. A DECLARED block whose
 camera could not be built is a present slot that is not READY (its own absence sentence) — the
 pointer is decided by the config before the camera exists (vision/toggle_camera.hpp's banner says why).
```

</details>
