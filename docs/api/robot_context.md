<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/chassis/robot_context.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `robot_context.hpp`

RobotContext — the composition root / DI container: "the one object that differs across robot / sim / test".

This header declares **2** types (17 members).

Extracted from [`include/shulib/chassis/robot_context.hpp`](../../include/shulib/chassis/robot_context.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

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

<a id="struct-robotcontextconfig"></a>

## `struct RobotContextConfig`

```cpp
struct RobotContextConfig
```

The HAL handles a robot is built from. Pointers so fields are NAMED at the call site (designated initializers) and so the RobotContext can validate them non-null.

*struct, declared at [`include/shulib/chassis/robot_context.hpp:32`](../../include/shulib/chassis/robot_context.hpp#L32).*

<a id="robotcontextconfig-clock"></a>

### `RobotContextConfig::clock`

```cpp
hal::IClock* clock = nullptr
```

the single source of now; seconds, monotonic

*field, declared at [`include/shulib/chassis/robot_context.hpp:33`](../../include/shulib/chassis/robot_context.hpp#L33).*

<a id="robotcontextconfig-drivemotors"></a>

### `RobotContextConfig::driveMotors`

```cpp
std::span<hal::IMotor* const> driveMotors
```

The drive motors, in kinematic wheel order: element i is commanded with wheel speed i of the installed IKinematics, so a wrong order drives the robot the wrong way in silence. ONE entry per kinematic wheel — N physical motors on a side go behind one hal::MotorGroup (R3b Part 1). This context checks only that the span is non-empty and all-non-null; motion::MotionDeps::validate() is where the count is checked, and it demands EQUALITY with the kinematics' wheel count. This is a non-owning VIEW: the pointer ARRAY must outlive the context.

*field, declared at [`include/shulib/chassis/robot_context.hpp:41`](../../include/shulib/chassis/robot_context.hpp#L41).*

<a id="robotcontextconfig-imu"></a>

### `RobotContextConfig::imu`

```cpp
hal::IImu* imu = nullptr
```

heading and yaw rate, canonical CCW radians

*field, declared at [`include/shulib/chassis/robot_context.hpp:42`](../../include/shulib/chassis/robot_context.hpp#L42).*

<a id="robotcontextconfig-gps"></a>

### `RobotContextConfig::gps`

```cpp
hal::IGps* gps = nullptr
```

absolute fix; off-strip its hasFix() reads false

*field, declared at [`include/shulib/chassis/robot_context.hpp:43`](../../include/shulib/chassis/robot_context.hpp#L43).*

<a id="robotcontextconfig-battery"></a>

### `RobotContextConfig::battery`

```cpp
hal::IBattery* battery = nullptr
```

volts: the pipeline's ceiling, and the run bookends

*field, declared at [`include/shulib/chassis/robot_context.hpp:44`](../../include/shulib/chassis/robot_context.hpp#L44).*

<a id="robotcontextconfig-telemetry"></a>

### `RobotContextConfig::telemetry`

```cpp
hal::ITelemetrySink* telemetry = nullptr
```

every log line and DebugRecord; NullSink = off

*field, declared at [`include/shulib/chassis/robot_context.hpp:45`](../../include/shulib/chassis/robot_context.hpp#L45).*

<a id="robotcontextconfig-tags"></a>

### `RobotContextConfig::tags`

```cpp
hal::ITagSource* tags = nullptr
```

AprilTags as body-frame poses (the M3 corrector)

*field, declared at [`include/shulib/chassis/robot_context.hpp:46`](../../include/shulib/chassis/robot_context.hpp#L46).*

<a id="robotcontextconfig-vision"></a>

### `RobotContextConfig::vision`

```cpp
hal::IVision* vision = nullptr
```

object bearings for M4 manipulation targeting

*field, declared at [`include/shulib/chassis/robot_context.hpp:47`](../../include/shulib/chassis/robot_context.hpp#L47).*

<a id="class-robotcontext"></a>

## `class RobotContext`

```cpp
class RobotContext
```

The composition root: the ONE object that differs between the real robot, the simulator and a host test. Every layer above the HAL reaches hardware only through it, so exchanging the HAL implementations exchanges the whole robot without touching a line of motion code.  NON-OWNING throughout. It copies the config's pointers and span; it never adopts, allocates or destroys anything, so every pointee — and the array the driveMotors span views — must outlive the context. It also caches nothing: each accessor hands back the live handle, so a reading is only ever as fresh as the caller's own call.

*class, declared at [`include/shulib/chassis/robot_context.hpp:58`](../../include/shulib/chassis/robot_context.hpp#L58).*

<a id="robotcontext-robotcontext"></a>

### `RobotContext::RobotContext`

```cpp
explicit RobotContext(const RobotContextConfig& cfg)
```

Validates the whole config up front — every handle non-null and driveMotors non-empty — through SHULIB_PRECONDITION, so a mis-wired robot fails at construction naming the handle it is missing, instead of dereferencing null halfway through an auton. The count of drive motors is checked here only for emptiness; motion::MotionDeps::validate() checks it for EQUALITY against the kinematics' wheel count (the context never sees the kinematics).

*function, declared at [`include/shulib/chassis/robot_context.hpp:65`](../../include/shulib/chassis/robot_context.hpp#L65).*

<a id="robotcontext-clock"></a>

### `RobotContext::clock`

```cpp
[[nodiscard]] hal::IClock& clock() const
```

The run's clock. A reference, never null — the constructor already proved that, which is why consumers write `ctx.clock().now()` and never test a pointer.

*function, declared at [`include/shulib/chassis/robot_context.hpp:81`](../../include/shulib/chassis/robot_context.hpp#L81).*

<a id="robotcontext-drivemotors"></a>

### `RobotContext::driveMotors`

```cpp
[[nodiscard]] std::span<hal::IMotor* const> driveMotors() const
```

The drive motors, in kinematic wheel order. A VIEW of the caller's array, so it is only as alive as that array; guaranteed non-empty here, and guaranteed to EQUAL the wheel count of the installed kinematics once a motion::MotionDeps has validated the pairing.

*function, declared at [`include/shulib/chassis/robot_context.hpp:85`](../../include/shulib/chassis/robot_context.hpp#L85).*

<a id="robotcontext-imu"></a>

### `RobotContext::imu`

```cpp
[[nodiscard]] hal::IImu& imu() const
```

The IMU, already canonical: CCW-positive radians, +X = 0. Nothing above this call converts an angle. Gate trust on isReady() at boot — a calibrating IMU reports garbage that moves.

*function, declared at [`include/shulib/chassis/robot_context.hpp:88`](../../include/shulib/chassis/robot_context.hpp#L88).*

<a id="robotcontext-gps"></a>

### `RobotContext::gps`

```cpp
[[nodiscard]] hal::IGps& gps() const
```

The GPS. Check hasFix() before believing pose(): off-strip the pose is unspecified (still finite), and Driving Skills has no strip at all, so this seam is silent for a whole run.

*function, declared at [`include/shulib/chassis/robot_context.hpp:91`](../../include/shulib/chassis/robot_context.hpp#L91).*

<a id="robotcontext-battery"></a>

### `RobotContext::battery`

```cpp
[[nodiscard]] hal::IBattery& battery() const
```

The battery. Read live at each use — the command pipeline takes its voltage ceiling from it every tick and the run summary samples it at both ends; nothing here caches a volt.

*function, declared at [`include/shulib/chassis/robot_context.hpp:94`](../../include/shulib/chassis/robot_context.hpp#L94).*

<a id="robotcontext-telemetry"></a>

### `RobotContext::telemetry`

```cpp
[[nodiscard]] hal::ITelemetrySink& telemetry() const
```

The one diagnostics sink for this robot. Which sink is installed is what decides whether tracing costs anything: with NullSink the per-tick record is never even populated.

*function, declared at [`include/shulib/chassis/robot_context.hpp:97`](../../include/shulib/chassis/robot_context.hpp#L97).*

<a id="robotcontext-tags"></a>

### `RobotContext::tags`

```cpp
[[nodiscard]] hal::ITagSource& tags() const
```

The AprilTag source — V5 AI Vision or a coprocessor, indistinguishable from here. Tags arrive as body-frame poses with no timestamp; staleness is the corrector's problem.

*function, declared at [`include/shulib/chassis/robot_context.hpp:100`](../../include/shulib/chassis/robot_context.hpp#L100).*

<a id="robotcontext-vision"></a>

### `RobotContext::vision`

```cpp
[[nodiscard]] hal::IVision& vision() const
```

The object/colour detection source. A separate seam from tags() so one adapter can serve both, or either alone, without a consumer of one depending on the other.

*function, declared at [`include/shulib/chassis/robot_context.hpp:103`](../../include/shulib/chassis/robot_context.hpp#L103).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 14 lines</summary>

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
 site); the constructor validates them all non-null, then the accessors hand out
 references so consumers write `ctx.clock().now()` rather than juggling pointers.
```

</details>
