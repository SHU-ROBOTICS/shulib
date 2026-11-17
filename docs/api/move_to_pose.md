<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/motion/move_to_pose.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `move_to_pose.hpp`

MoveToPose — decoupled per-axis field-pose motion.

This header declares **2** types (14 members).

Extracted from [`include/shulib/motion/move_to_pose.hpp`](../../include/shulib/motion/move_to_pose.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in [How it works](../guide/README.md); worked recipes live in [Recipes](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`struct PoseMotionOptions`](#struct-posemotionoptions)
  - [`captureHeadingAtLive`](#posemotionoptions-captureheadingatlive)
  - [`capturePoseAtLive`](#posemotionoptions-captureposeatlive)
  - [`holdFor`](#posemotionoptions-holdfor)
  - [`tankSteer`](#posemotionoptions-tanksteer)
- [`class MoveToPose`](#class-movetopose)
  - [`MoveToPose`](#movetopose-movetopose)
  - [`start`](#movetopose-start)
  - [`tick`](#movetopose-tick)
  - [`cancel`](#movetopose-cancel)
  - [`exitReason`](#movetopose-exitreason)
  - [`state`](#movetopose-state)
  - [`name`](#movetopose-name)
  - [`tankSteerStatus`](#movetopose-tanksteerstatus)
  - [`target`](#movetopose-target)
  - [`setTarget`](#movetopose-settarget)

<a id="struct-posemotionoptions"></a>

## `struct PoseMotionOptions`

```cpp
struct PoseMotionOptions
```

Internal shaping knobs for the sibling primitives (StrafeTo / HoldPose). Not part of MoveToPose's public construction surface.

*struct, declared at [`include/shulib/motion/move_to_pose.hpp:92`](../../include/shulib/motion/move_to_pose.hpp#L92).*

<a id="posemotionoptions-captureheadingatlive"></a>

### `PoseMotionOptions::captureHeadingAtLive`

```cpp
bool captureHeadingAtLive = false
```

StrafeTo: hold the first-live heading

*field, declared at [`include/shulib/motion/move_to_pose.hpp:93`](../../include/shulib/motion/move_to_pose.hpp#L93).*

<a id="posemotionoptions-captureposeatlive"></a>

### `PoseMotionOptions::capturePoseAtLive`

```cpp
bool capturePoseAtLive = false
```

HoldPose: hold the first-live pose

*field, declared at [`include/shulib/motion/move_to_pose.hpp:94`](../../include/shulib/motion/move_to_pose.hpp#L94).*

<a id="posemotionoptions-holdfor"></a>

### `PoseMotionOptions::holdFor`

```cpp
double holdFor = 0.0
```

> 0 ⇒ hold-mode exit (HoldPose)

*field, declared at [`include/shulib/motion/move_to_pose.hpp:95`](../../include/shulib/motion/move_to_pose.hpp#L95).*

<a id="posemotionoptions-tanksteer"></a>

### `PoseMotionOptions::tankSteer`

```cpp
bool tankSteer = false
```

TANK1: this motion may steer a tank back onto its target's line. Only the public MoveToPose constructor sets it; StrafeTo (a translation that HOLDS heading) and HoldPose (no line to steer along) leave it false, so on a tank they move exactly as before. (Their result lines are another matter: the scheduler gives a tank's StrafeTo, and a HoldPose of a spot away from the robot, TANK1 Part B's ` xt` column — a report.)

*field, declared at [`include/shulib/motion/move_to_pose.hpp:101`](../../include/shulib/motion/move_to_pose.hpp#L101).*

<a id="class-movetopose"></a>

## `class MoveToPose`

```cpp
class MoveToPose : public IMotion
```

Drive to a FIELD-frame pose with three INDEPENDENT controllers — field-x, field-y and heading — each closing its own loop every tick and combining into one ChassisSpeeds. The robot therefore translates and rotates simultaneously; nothing in this class sequences a turn before a drive. Arrival needs BOTH criteria at once (translation distance AND heading error), so it composes two SettledUtils and one Watchdog rather than one scalar exit. StrafeTo and HoldPose are this same engine with different capture/exit options.  On a TANK (strafeAuthority() exactly 0) a leg that starts within config.tankSteer.maxSteer of its target heading also steers back onto the target's LINE while it drives (TANK1 — the file header; MotionConfig::tankSteer; tankSteerStatus()), and a leg that starts farther off runs unsteered and logs one Warn. It never plans a turn: face() the point first.  A MoveToPose owns no loop and no thread: the caller ticks it, having updated the Localizer first, until tick() returns something other than Running.

*class, declared at [`include/shulib/motion/move_to_pose.hpp:118`](../../include/shulib/motion/move_to_pose.hpp#L118).*

<a id="movetopose-movetopose"></a>

### `MoveToPose::MoveToPose`

```cpp
MoveToPose(const MotionDeps& deps, const math::Pose2d& target, const MotionConfig& config = {}, double timeout = 0.0)
```

Drive to `target` (FIELD frame). `timeout` seconds bounds the whole motion INCLUDING any boot wait; 0 selects config.defaultTimeout.

*function, declared at [`include/shulib/motion/move_to_pose.hpp:122`](../../include/shulib/motion/move_to_pose.hpp#L122).*

<a id="movetopose-start"></a>

### `MoveToPose::start`

```cpp
void start() override
```

Arm, or fully re-arm: the three PIDs, both settle detectors and the stall check are reset, the watchdog clock restarts, and the state drops back to WaitingForEstimate. Commands no motors. A capture-at-first-live target (StrafeTo's heading, HoldPose's pose) is re-armed too, so a re-started motion captures again from the CURRENT estimate rather than reusing the previous run's. A plain MoveToPose keeps its explicit target. TANK1's state is re-armed too: the engage decision is made again at the next first live tick, and the yaw correction starts from 0.

*function, declared at [`include/shulib/motion/move_to_pose.hpp:133`](../../include/shulib/motion/move_to_pose.hpp#L133).*

<a id="movetopose-tick"></a>

### `MoveToPose::tick`

```cpp
[[nodiscard]] control::ExitReason tick() override
```

One control tick, and the only member here that commands a DRIVING voltage — cancel() commands the motors too, into the shared safe state, and is in fact the only member that ever changes a brake mode (this one's stops just write 0 V). Precondition: start() has been called; the loop owner must have advanced the Localizer FIRST, since this reads the estimate as the world at time t. While the estimate is still Uninitialized it commands zero volts and makes no settle progress — but the watchdog keeps running through that wait, so a never-live estimate exits TimedOut instead of hanging. Returns Running until both criteria settle (Settled) or the watchdog fires (TimedOut, MotionTimeout raised); motors are stopped BEFORE the exit record is emitted, so the record stream ends on the true final state. After any non-Running verdict this is a no-op that returns the cached verdict. Emits AT MOST one DebugRecord per call: that cached-verdict path emits nothing, and no path emits unless the sink answers wantsRecord() — the record is built inside hal::emitRecord's lambda, so against a NullSink or any log-only sink it is never populated at all. When one is emitted its `commanded` field is the FINAL achievable command in the FIELD frame — post-clamp, so this layer's clamping is auditable from the stream.

*function, declared at [`include/shulib/motion/move_to_pose.hpp:175`](../../include/shulib/motion/move_to_pose.hpp#L175).*

<a id="movetopose-cancel"></a>

### `MoveToPose::cancel`

```cpp
void cancel() override
```

The cancel contract (motion.hpp): safe state whenever started, verdict only if still running, Idle untouched, idempotent, never raises.

*function, declared at [`include/shulib/motion/move_to_pose.hpp:364`](../../include/shulib/motion/move_to_pose.hpp#L364).*

<a id="movetopose-exitreason"></a>

### `MoveToPose::exitReason`

```cpp
[[nodiscard]] control::ExitReason exitReason() const noexcept override
```

The verdict cached by the last tick() or cancel() — Running until the first exit, then that exit reason for good. Reading it never recomputes anything and never advances the motion; only start() clears it back to Running.

*function, declared at [`include/shulib/motion/move_to_pose.hpp:393`](../../include/shulib/motion/move_to_pose.hpp#L393).*

<a id="movetopose-state"></a>

### `MoveToPose::state`

```cpp
[[nodiscard]] MotionState state() const noexcept override
```

The motion-layer state, which is also written into DebugRecord.activeCommandState every tick: Idle before start(), WaitingForEstimate through the boot window, Running while controlling, then the state matching the verdict. Finer-grained than exitReason(), which cannot tell Idle from Running.

*function, declared at [`include/shulib/motion/move_to_pose.hpp:399`](../../include/shulib/motion/move_to_pose.hpp#L399).*

<a id="movetopose-name"></a>

### `MoveToPose::name`

```cpp
[[nodiscard]] const char* name() const noexcept override
```

Always the literal "MoveToPose" — the string that identifies this motion in MotionTimeout fault text and in run result lines. The siblings override it with their own names, so a StrafeTo never reports as its base class.

*function, declared at [`include/shulib/motion/move_to_pose.hpp:404`](../../include/shulib/motion/move_to_pose.hpp#L404).*

<a id="movetopose-tanksteerstatus"></a>

### `MoveToPose::tankSteerStatus`

```cpp
[[nodiscard]] TankSteerStatus tankSteerStatus() const noexcept
```

TANK1's account of the most recent tick that computed a command (all zero / false off a tank, and before the first live tick; the exit tick commands nothing, so after it this still describes the tick before): whether this motion may steer, whether this run does, and that tick's cross-track error, the predicted one the bend steered on, the bend and the yaw correction.

*function, declared at [`include/shulib/motion/move_to_pose.hpp:411`](../../include/shulib/motion/move_to_pose.hpp#L411).*

<a id="movetopose-target"></a>

### `MoveToPose::target`

```cpp
[[nodiscard]] const math::Pose2d& target() const noexcept
```

The FIELD-frame target (after any first-live-tick capture).

*function, declared at [`include/shulib/motion/move_to_pose.hpp:421`](../../include/shulib/motion/move_to_pose.hpp#L421).*

<a id="movetopose-settarget"></a>

### `MoveToPose::setTarget`

```cpp
void setTarget(const math::Pose2d& target)
```

Retarget BEFORE start() (rebuilding a motion for a new waypoint). Precondition: not currently running.

*function, declared at [`include/shulib/motion/move_to_pose.hpp:425`](../../include/shulib/motion/move_to_pose.hpp#L425).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 65 lines, click to expand</summary>

```text

 MoveToPose — decoupled per-axis field-pose motion (chunk C1). THE HOLONOMIC
 THESIS, as code: three INDEPENDENT controllers — field-x, field-y, heading —
 each closing its own loop every tick, combined into one ChassisSpeeds. The
 robot translates and rotates SIMULTANEOUSLY and INDEPENDENTLY; there is no
 turn-then-drive sequencing anywhere in this file, and the simultaneity test
 pins that a diagonal move with a heading change happens as ONE motion. (That
 sequencing is precisely LemLib's tank-coupled behaviour this project exists
 to beat.) Two sanctioned exceptions, each telemetry-visible by contract:
   * C3's strafe FALLBACK — on a drive with limited strafe authority, body |vy|
     is clamped while vx and ω keep full authority (record flag SFB);
   * TANK1's tank steering — on a drive with NO strafe authority (a tank;
     strafeAuthority() exactly 0), a plain MoveToPose whose leg STARTS within
     tankSteer.maxSteer (15°) of its target heading bends its heading setpoint
     toward the target's LINE (through the target point, along the target
     heading) by at most maxSteer, faded to 0 at the target — steering on
     where the robot will be off that line tankSteer.leadTime ahead, so the
     bend eases while it closes on the line — and cancels the
     measured yaw it did not ask for (a weaker side's), bounded by
     yawCorrectionLimit (record flag TSTR). What it is NOT: a planned turn.
     Nothing in it aims at the target POINT (tank_steer.hpp), and a leg that
     starts farther off its heading is not steered at all — it runs exactly as
     before TANK1 and logs one Warn ("turn first: face()"). StrafeTo and
     HoldPose never steer (PoseMotionOptions::tankSteer stays false).

 ── The per-tick pipeline (frames and clamps annotated — the F1/F5 choreography) ────
   1. pose  = Localizer::pose()                                  [FIELD]
   2. errors: ex = tx−px, ey = ty−py (in); eh = heading.errorTo(target)
      — eh is F3's SHORTEST signed error, so the ±180° seam is absorbed
      BEFORE any controller sees a number.                       [FIELD/rad]
   3. three decoupled PIDs → (vx, vy) in/s, ω rad/s              [FIELD]
      heading PID is fed (setpoint 0, measurement −eh): the error the PID
      differentiates is CONTINUOUS near the target (no wrap in its input),
      and D-on-measurement stays live for a future kD. On an ENGAGED tank
      leg (TANK1, above) it is fed the error to the BENT setpoint instead
      (eh_s = heading.errorTo(target heading + bend), the bend taken from the
      cross-track error predicted leadTime ahead — tank_steer.hpp), and the
      yaw correction is subtracted from its output; every other leg is fed
      −eh exactly.
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
