<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/motion/move_to_pose.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `move_to_pose.hpp`

MoveToPose — decoupled per-axis field-pose motion.

This header declares **2** types (12 members).

Extracted from [`include/shulib/motion/move_to_pose.hpp`](../../include/shulib/motion/move_to_pose.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`struct PoseMotionOptions`](#struct-posemotionoptions)
  - [`captureHeadingAtLive`](#posemotionoptions-captureheadingatlive)
  - [`capturePoseAtLive`](#posemotionoptions-captureposeatlive)
  - [`holdFor`](#posemotionoptions-holdfor)
- [`class MoveToPose`](#class-movetopose)
  - [`MoveToPose`](#movetopose-movetopose)
  - [`start`](#movetopose-start)
  - [`tick`](#movetopose-tick)
  - [`cancel`](#movetopose-cancel)
  - [`exitReason`](#movetopose-exitreason)
  - [`state`](#movetopose-state)
  - [`name`](#movetopose-name)
  - [`target`](#movetopose-target)
  - [`setTarget`](#movetopose-settarget)

<a id="struct-posemotionoptions"></a>

## `struct PoseMotionOptions`

```cpp
struct PoseMotionOptions
```

Internal shaping knobs for the sibling primitives (StrafeTo / HoldPose). Not part of MoveToPose's public construction surface.

*struct, declared at [`include/shulib/motion/move_to_pose.hpp:70`](../../include/shulib/motion/move_to_pose.hpp#L70).*

<a id="posemotionoptions-captureheadingatlive"></a>

### `PoseMotionOptions::captureHeadingAtLive`

```cpp
bool captureHeadingAtLive = false
```

StrafeTo: hold the first-live heading

*field, declared at [`include/shulib/motion/move_to_pose.hpp:71`](../../include/shulib/motion/move_to_pose.hpp#L71).*

<a id="posemotionoptions-captureposeatlive"></a>

### `PoseMotionOptions::capturePoseAtLive`

```cpp
bool capturePoseAtLive = false
```

HoldPose: hold the first-live pose

*field, declared at [`include/shulib/motion/move_to_pose.hpp:72`](../../include/shulib/motion/move_to_pose.hpp#L72).*

<a id="posemotionoptions-holdfor"></a>

### `PoseMotionOptions::holdFor`

```cpp
double holdFor = 0.0
```

> 0 ⇒ hold-mode exit (HoldPose)

*field, declared at [`include/shulib/motion/move_to_pose.hpp:73`](../../include/shulib/motion/move_to_pose.hpp#L73).*

<a id="class-movetopose"></a>

## `class MoveToPose`

```cpp
class MoveToPose : public IMotion
```

Drive to a FIELD-frame pose with three INDEPENDENT controllers — field-x, field-y and heading — each closing its own loop every tick and combining into one ChassisSpeeds. The robot therefore translates and rotates simultaneously; nothing in this class sequences a turn before a drive. Arrival needs BOTH criteria at once (translation distance AND heading error), so it composes two SettledUtils and one Watchdog rather than one scalar exit. StrafeTo and HoldPose are this same engine with different capture/exit options.  A MoveToPose owns no loop and no thread: the caller ticks it, having updated the Localizer first, until tick() returns something other than Running.

*class, declared at [`include/shulib/motion/move_to_pose.hpp:85`](../../include/shulib/motion/move_to_pose.hpp#L85).*

<a id="movetopose-movetopose"></a>

### `MoveToPose::MoveToPose`

```cpp
MoveToPose(const MotionDeps& deps, const math::Pose2d& target, const MotionConfig& config = {}, double timeout = 0.0)
```

Drive to `target` (FIELD frame). `timeout` seconds bounds the whole motion INCLUDING any boot wait; 0 selects config.defaultTimeout.

*function, declared at [`include/shulib/motion/move_to_pose.hpp:89`](../../include/shulib/motion/move_to_pose.hpp#L89).*

<a id="movetopose-start"></a>

### `MoveToPose::start`

```cpp
void start() override
```

Arm, or fully re-arm: the three PIDs, both settle detectors and the stall check are reset, the watchdog clock restarts, and the state drops back to WaitingForEstimate. Commands no motors. A capture-at-first-live target (StrafeTo's heading, HoldPose's pose) is re-armed too, so a re-started motion captures again from the CURRENT estimate rather than reusing the previous run's. A plain MoveToPose keeps its explicit target.

*function, declared at [`include/shulib/motion/move_to_pose.hpp:98`](../../include/shulib/motion/move_to_pose.hpp#L98).*

<a id="movetopose-tick"></a>

### `MoveToPose::tick`

```cpp
[[nodiscard]] control::ExitReason tick() override
```

One control tick, and the only member here that commands a DRIVING voltage — cancel() commands the motors too, into the shared safe state, and is in fact the only member that ever changes a brake mode (this one's stops just write 0 V). Precondition: start() has been called; the loop owner must have advanced the Localizer FIRST, since this reads the estimate as the world at time t. While the estimate is still Uninitialized it commands zero volts and makes no settle progress — but the watchdog keeps running through that wait, so a never-live estimate exits TimedOut instead of hanging. Returns Running until both criteria settle (Settled) or the watchdog fires (TimedOut, MotionTimeout raised); motors are stopped BEFORE the exit record is emitted, so the record stream ends on the true final state. After any non-Running verdict this is a no-op that returns the cached verdict. Emits AT MOST one DebugRecord per call: that cached-verdict path emits nothing, and no path emits unless the sink answers wantsRecord() — the record is built inside hal::emitRecord's lambda, so against a NullSink or any log-only sink it is never populated at all. When one is emitted its `commanded` field is the FINAL achievable command in the FIELD frame — post-clamp, so this layer's clamping is auditable from the stream.

*function, declared at [`include/shulib/motion/move_to_pose.hpp:131`](../../include/shulib/motion/move_to_pose.hpp#L131).*

<a id="movetopose-cancel"></a>

### `MoveToPose::cancel`

```cpp
void cancel() override
```

The cancel contract (motion.hpp): safe state whenever started, verdict only if still running, Idle untouched, idempotent, never raises.

*function, declared at [`include/shulib/motion/move_to_pose.hpp:244`](../../include/shulib/motion/move_to_pose.hpp#L244).*

<a id="movetopose-exitreason"></a>

### `MoveToPose::exitReason`

```cpp
[[nodiscard]] control::ExitReason exitReason() const noexcept override
```

The verdict cached by the last tick() or cancel() — Running until the first exit, then that exit reason for good. Reading it never recomputes anything and never advances the motion; only start() clears it back to Running.

*function, declared at [`include/shulib/motion/move_to_pose.hpp:273`](../../include/shulib/motion/move_to_pose.hpp#L273).*

<a id="movetopose-state"></a>

### `MoveToPose::state`

```cpp
[[nodiscard]] MotionState state() const noexcept override
```

The motion-layer state, which is also written into DebugRecord.activeCommandState every tick: Idle before start(), WaitingForEstimate through the boot window, Running while controlling, then the state matching the verdict. Finer-grained than exitReason(), which cannot tell Idle from Running.

*function, declared at [`include/shulib/motion/move_to_pose.hpp:279`](../../include/shulib/motion/move_to_pose.hpp#L279).*

<a id="movetopose-name"></a>

### `MoveToPose::name`

```cpp
[[nodiscard]] const char* name() const noexcept override
```

Always the literal "MoveToPose" — the string that identifies this motion in MotionTimeout fault text and in run result lines. The siblings override it with their own names, so a StrafeTo never reports as its base class.

*function, declared at [`include/shulib/motion/move_to_pose.hpp:284`](../../include/shulib/motion/move_to_pose.hpp#L284).*

<a id="movetopose-target"></a>

### `MoveToPose::target`

```cpp
[[nodiscard]] const math::Pose2d& target() const noexcept
```

The FIELD-frame target (after any first-live-tick capture).

*function, declared at [`include/shulib/motion/move_to_pose.hpp:287`](../../include/shulib/motion/move_to_pose.hpp#L287).*

<a id="movetopose-settarget"></a>

### `MoveToPose::setTarget`

```cpp
void setTarget(const math::Pose2d& target)
```

Retarget BEFORE start() (rebuilding a motion for a new waypoint). Precondition: not currently running.

*function, declared at [`include/shulib/motion/move_to_pose.hpp:291`](../../include/shulib/motion/move_to_pose.hpp#L291).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 46 lines, click to expand</summary>

```text

 MoveToPose — decoupled per-axis field-pose motion (chunk C1). THE HOLONOMIC
 THESIS, as code: three INDEPENDENT controllers — field-x, field-y, heading —
 each closing its own loop every tick, combined into one ChassisSpeeds. The
 robot translates and rotates SIMULTANEOUSLY and INDEPENDENTLY; there is no
 turn-then-drive sequencing anywhere in this file, and the simultaneity test
 pins that a diagonal move with a heading change happens as ONE motion. (That
 sequencing is precisely LemLib's tank-coupled behaviour this project exists
 to beat; C3's H-drive strafe FALLBACK is the only sanctioned exception, and
 it is telemetry-visible by contract.)

 ── The per-tick pipeline (frames and clamps annotated — the F1/F5 choreography) ────
   1. pose  = Localizer::pose()                                  [FIELD]
   2. errors: ex = tx−px, ey = ty−py (in); eh = heading.errorTo(target)
      — eh is F3's SHORTEST signed error, so the ±180° seam is absorbed
      BEFORE any controller sees a number.                       [FIELD/rad]
   3. three decoupled PIDs → (vx, vy) in/s, ω rad/s              [FIELD]
      heading PID is fed (setpoint 0, measurement −eh): the error the PID
      differentiates is CONTINUOUS near the target (no wrap in its input),
      and D-on-measurement stays live for a future kD.
   4–8. applyCommandPipeline (command_pipeline.hpp — extracted from here at
      C4 so the Chassis facade's drive() shares ONE choreography): |ω| clamp;
      uniform norm cap; fieldToRobot (THE one F1 rotation); body |vy| clamped
      to strafeAuthority()·maxLinearSpeed (the UPSTREAM clamp §13 #5 assigns
      to THIS layer — kinematics never clamps, F5; a meaningful bind flags
      strafeFallbackActive in the record, C3's never-silent contract);
      toWheels → desaturate → Feedforward → compensateForBattery → volts.
   9. OdoStallCheck + HealthMonitor::tick — the A3 containment wiring.
  10. settle/watchdog verdict; one lazy DebugRecord (A1 contract).

 ── Exit logic ──────────────────────────────────────────────────────────────────────
 Arrival needs BOTH a translation criterion (‖(ex,ey)‖) and a heading
 criterion (|eh|) — so this class composes TWO SettledUtils + ONE Watchdog
 rather than the 1-scalar ExitGroup (same tested primitives, 2+1 composition;
 ExitGroup's Settled-beats-simultaneous-TimedOut priority is preserved).
 TurnTo / DriveBrake, whose exit is one scalar, use ExitGroup unchanged.
 On the exit tick the motors are stopped BEFORE the record is emitted, so the
 record stream shows the true final state (zero command, exit state).

 ── Sibling shaping (PoseMotionOptions) ─────────────────────────────────────────────
 StrafeTo and HoldPose are this same engine with capture/exit options — the
 three share one pipeline so a fix lands once. Capture-at-first-LIVE-tick is
 part of the wait-for-live contract (motion.hpp): an estimate-derived target
 must never be read during the boot window.

 Gains/tolerances: MotionConfig — every default provisional until R5 (HA-50/51/52).
```

</details>
