<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/motion/curve_follow.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `curve_follow.hpp`

CurveFollow — the curve verbs' motion (chunk CURVE1, 2026-11): the robot drives a smooth path through the author's points WITHOUT stopping at them — pure pursuit on the path curve_path.hpp builds, a speed plan that slows before each bend, two guards that st…

This header declares **5** types (37 members) and **1** free function.

Extracted from [`include/shulib/motion/curve_follow.hpp`](../../include/shulib/motion/curve_follow.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in [How it works](../guide/README.md); worked recipes live in [Recipes](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`curveYawObserverStep`](#curveyawobserverstep) — *free function*
- [`enum class CurvePhase`](#enum-class-curvephase)
  - [`Idle`](#curvephase-idle)
  - [`Waiting`](#curvephase-waiting)
  - [`Pursuing`](#curvephase-pursuing)
  - [`Ending`](#curvephase-ending)
  - [`Done`](#curvephase-done)
- [`enum class CurveGuard`](#enum-class-curveguard)
  - [`None`](#curveguard-none)
  - [`Heading`](#curveguard-heading)
  - [`Progress`](#curveguard-progress)
- [`enum class CurveHandoff`](#enum-class-curvehandoff)
  - [`None`](#curvehandoff-none)
  - [`Clean`](#curvehandoff-clean)
  - [`LastResort`](#curvehandoff-lastresort)
  - [`EndOfPath`](#curvehandoff-endofpath)
- [`struct CurveStatus`](#struct-curvestatus)
  - [`phase`](#curvestatus-phase)
  - [`s`](#curvestatus-s)
  - [`length`](#curvestatus-length)
  - [`lookX`](#curvestatus-lookx)
  - [`lookY`](#curvestatus-looky)
  - [`curvature`](#curvestatus-curvature)
  - [`speedLimit`](#curvestatus-speedlimit)
  - [`commandedSpeed`](#curvestatus-commandedspeed)
  - [`yawCorrection`](#curvestatus-yawcorrection)
  - [`headingOffPath`](#curvestatus-headingoffpath)
  - [`guard`](#curvestatus-guard)
  - [`startCheck`](#curvestatus-startcheck)
  - [`handoff`](#curvestatus-handoff)
  - [`handoffHeadingError`](#curvestatus-handoffheadingerror)
- [`class CurveFollow`](#class-curvefollow)
  - [`CurveFollow`](#curvefollow-curvefollow)
  - [`start`](#curvefollow-start)
  - [`tick`](#curvefollow-tick)
  - [`cancel`](#curvefollow-cancel)
  - [`exitReason`](#curvefollow-exitreason)
  - [`state`](#curvefollow-state)
  - [`name`](#curvefollow-name)
  - [`curveStatus`](#curvefollow-curvestatus)
  - [`path`](#curvefollow-path)
  - [`endPose`](#curvefollow-endpose)
  - [`endMotion`](#curvefollow-endmotion)

<a id="curveyawobserverstep"></a>

## `curveYawObserverStep`

```cpp
[[nodiscard]] inline double curveYawObserverStep(double estimate, double measuredRate, double commandedRate, double dt, const TankSteerConfig& cfg) noexcept
```

TANK1's yaw observer step (move_to_pose.hpp, the engaged-leg branch), written once more for the curve: the estimate moves toward the unexplained yaw rate (measured minus the rate commanded last tick, after the omega clamp, before desaturation) by dt / (yawObserverTime + dt), clamped to +-yawCorrectionLimit. The caller applies the same gate MoveToPose does (a command exists, dt > 0, the IMU is ready) — otherwise the estimate is held. Identical arithmetic, pinned bit for bit by test (motion_curve_test.cpp CV14), so a change to one is seen in the other.

*free function, declared at [`include/shulib/motion/curve_follow.hpp:127`](../../include/shulib/motion/curve_follow.hpp#L127).*

<a id="enum-class-curvephase"></a>

## `enum class CurvePhase`

```cpp
enum class CurvePhase : std::uint8_t
```

Where a curve is.

*enum class, declared at [`include/shulib/motion/curve_follow.hpp:137`](../../include/shulib/motion/curve_follow.hpp#L137).*

<a id="curvephase-idle"></a>

### `CurvePhase::Idle`

```cpp
Idle = 0
```

constructed, not started

*enumerator, declared at [`include/shulib/motion/curve_follow.hpp:138`](../../include/shulib/motion/curve_follow.hpp#L138).*

<a id="curvephase-waiting"></a>

### `CurvePhase::Waiting`

```cpp
Waiting = 1
```

started, the estimate not yet live

*enumerator, declared at [`include/shulib/motion/curve_follow.hpp:139`](../../include/shulib/motion/curve_follow.hpp#L139).*

<a id="curvephase-pursuing"></a>

### `CurvePhase::Pursuing`

```cpp
Pursuing = 2
```

following the path

*enumerator, declared at [`include/shulib/motion/curve_follow.hpp:140`](../../include/shulib/motion/curve_follow.hpp#L140).*

<a id="curvephase-ending"></a>

### `CurvePhase::Ending`

```cpp
Ending = 3
```

handed off: the end MoveToPose is running

*enumerator, declared at [`include/shulib/motion/curve_follow.hpp:141`](../../include/shulib/motion/curve_follow.hpp#L141).*

<a id="curvephase-done"></a>

### `CurvePhase::Done`

```cpp
Done = 4
```

exited (any verdict)

*enumerator, declared at [`include/shulib/motion/curve_follow.hpp:142`](../../include/shulib/motion/curve_follow.hpp#L142).*

<a id="enum-class-curveguard"></a>

## `enum class CurveGuard`

```cpp
enum class CurveGuard : std::uint8_t
```

Which guard stopped a curve.

*enum class, declared at [`include/shulib/motion/curve_follow.hpp:146`](../../include/shulib/motion/curve_follow.hpp#L146).*

<a id="curveguard-none"></a>

### `CurveGuard::None`

```cpp
None = 0
```

none fired

*enumerator, declared at [`include/shulib/motion/curve_follow.hpp:147`](../../include/shulib/motion/curve_follow.hpp#L147).*

<a id="curveguard-heading"></a>

### `CurveGuard::Heading`

```cpp
Heading = 1
```

more than lostHeading off the path for lostTime

*enumerator, declared at [`include/shulib/motion/curve_follow.hpp:148`](../../include/shulib/motion/curve_follow.hpp#L148).*

<a id="curveguard-progress"></a>

### `CurveGuard::Progress`

```cpp
Progress = 2
```

under 1 in of progress in stuckTime, outside the end zone

*enumerator, declared at [`include/shulib/motion/curve_follow.hpp:149`](../../include/shulib/motion/curve_follow.hpp#L149).*

<a id="enum-class-curvehandoff"></a>

## `enum class CurveHandoff`

```cpp
enum class CurveHandoff : std::uint8_t
```

Which hand-off started the end (the file header's (1)-(3)).

*enum class, declared at [`include/shulib/motion/curve_follow.hpp:153`](../../include/shulib/motion/curve_follow.hpp#L153).*

<a id="curvehandoff-none"></a>

### `CurveHandoff::None`

```cpp
None = 0
```

not handed off

*enumerator, declared at [`include/shulib/motion/curve_follow.hpp:154`](../../include/shulib/motion/curve_follow.hpp#L154).*

<a id="curvehandoff-clean"></a>

### `CurveHandoff::Clean`

```cpp
Clean = 1
```

(1) in the end zone, on the end's line, lined up

*enumerator, declared at [`include/shulib/motion/curve_follow.hpp:155`](../../include/shulib/motion/curve_follow.hpp#L155).*

<a id="curvehandoff-lastresort"></a>

### `CurveHandoff::LastResort`

```cpp
LastResort = 2
```

(2) inside TANK1's steering window, handoffLast of the end zone left

*enumerator, declared at [`include/shulib/motion/curve_follow.hpp:156`](../../include/shulib/motion/curve_follow.hpp#L156).*

<a id="curvehandoff-endofpath"></a>

### `CurveHandoff::EndOfPath`

```cpp
EndOfPath = 3
```

(3) the end of the path, outside the window (unsteered end, one Warn)

*enumerator, declared at [`include/shulib/motion/curve_follow.hpp:157`](../../include/shulib/motion/curve_follow.hpp#L157).*

<a id="struct-curvestatus"></a>

## `struct CurveStatus`

```cpp
struct CurveStatus
```

What a CurveFollow did on its most recent tick — for tests and Tier-3 tooling (the record stream carries only the standard fields).

*struct, declared at [`include/shulib/motion/curve_follow.hpp:162`](../../include/shulib/motion/curve_follow.hpp#L162).*

<a id="curvestatus-phase"></a>

### `CurveStatus::phase`

```cpp
CurvePhase phase = CurvePhase::Idle
```

where it is

*field, declared at [`include/shulib/motion/curve_follow.hpp:163`](../../include/shulib/motion/curve_follow.hpp#L163).*

<a id="curvestatus-s"></a>

### `CurveStatus::s`

```cpp
double s = 0.0
```

the closest point's arc length (in)

*field, declared at [`include/shulib/motion/curve_follow.hpp:164`](../../include/shulib/motion/curve_follow.hpp#L164).*

<a id="curvestatus-length"></a>

### `CurveStatus::length`

```cpp
double length = 0.0
```

the path's length (in); 0 before it is built

*field, declared at [`include/shulib/motion/curve_follow.hpp:165`](../../include/shulib/motion/curve_follow.hpp#L165).*

<a id="curvestatus-lookx"></a>

### `CurveStatus::lookX`

```cpp
double lookX = 0.0
```

the look-ahead point, field x (in)

*field, declared at [`include/shulib/motion/curve_follow.hpp:166`](../../include/shulib/motion/curve_follow.hpp#L166).*

<a id="curvestatus-looky"></a>

### `CurveStatus::lookY`

```cpp
double lookY = 0.0
```

the look-ahead point, field y (in)

*field, declared at [`include/shulib/motion/curve_follow.hpp:167`](../../include/shulib/motion/curve_follow.hpp#L167).*

<a id="curvestatus-curvature"></a>

### `CurveStatus::curvature`

```cpp
double curvature = 0.0
```

the commanded arc's curvature (1/in)

*field, declared at [`include/shulib/motion/curve_follow.hpp:168`](../../include/shulib/motion/curve_follow.hpp#L168).*

<a id="curvestatus-speedlimit"></a>

### `CurveStatus::speedLimit`

```cpp
double speedLimit = 0.0
```

the speed plan at s (in/s)

*field, declared at [`include/shulib/motion/curve_follow.hpp:169`](../../include/shulib/motion/curve_follow.hpp#L169).*

<a id="curvestatus-commandedspeed"></a>

### `CurveStatus::commandedSpeed`

```cpp
double commandedSpeed = 0.0
```

the forward speed asked for (in/s, along the travel)

*field, declared at [`include/shulib/motion/curve_follow.hpp:170`](../../include/shulib/motion/curve_follow.hpp#L170).*

<a id="curvestatus-yawcorrection"></a>

### `CurveStatus::yawCorrection`

```cpp
double yawCorrection = 0.0
```

the yaw observer's estimate (rad/s), subtracted

*field, declared at [`include/shulib/motion/curve_follow.hpp:171`](../../include/shulib/motion/curve_follow.hpp#L171).*

<a id="curvestatus-headingoffpath"></a>

### `CurveStatus::headingOffPath`

```cpp
double headingOffPath = 0.0
```

|direction of travel - path direction at s| (rad)

*field, declared at [`include/shulib/motion/curve_follow.hpp:172`](../../include/shulib/motion/curve_follow.hpp#L172).*

<a id="curvestatus-guard"></a>

### `CurveStatus::guard`

```cpp
CurveGuard guard = CurveGuard::None
```

the guard that fired, if any

*field, declared at [`include/shulib/motion/curve_follow.hpp:173`](../../include/shulib/motion/curve_follow.hpp#L173).*

<a id="curvestatus-startcheck"></a>

### `CurveStatus::startCheck`

```cpp
CurveCheck startCheck{}
```

checkCurve()'s verdict at the first live tick

*field, declared at [`include/shulib/motion/curve_follow.hpp:174`](../../include/shulib/motion/curve_follow.hpp#L174).*

<a id="curvestatus-handoff"></a>

### `CurveStatus::handoff`

```cpp
CurveHandoff handoff = CurveHandoff::None
```

which hand-off started the end

*field, declared at [`include/shulib/motion/curve_follow.hpp:175`](../../include/shulib/motion/curve_follow.hpp#L175).*

<a id="curvestatus-handoffheadingerror"></a>

### `CurveStatus::handoffHeadingError`

```cpp
double handoffHeadingError = 0.0
```

|heading error to the end heading| at the hand-off (rad)

*field, declared at [`include/shulib/motion/curve_follow.hpp:176`](../../include/shulib/motion/curve_follow.hpp#L176).*

<a id="class-curvefollow"></a>

## `class CurveFollow`

```cpp
class CurveFollow : public IMotion
```

Drive a smooth path through `points` (curve_path.hpp) and end with an ordinary MoveToPose (the file header). On any drivetrain it drives like a tank — the nose (the back, reversing) along the path, no sideways command. Owns no loop: tick it after updating the Localizer.

*class, declared at [`include/shulib/motion/curve_follow.hpp:182`](../../include/shulib/motion/curve_follow.hpp#L182).*

<a id="curvefollow-curvefollow"></a>

### `CurveFollow::CurveFollow`

```cpp
CurveFollow(const MotionDeps& deps, std::span<const CurvePoint> points, const MotionConfig& config = {}, double timeout = 0.0, bool reverse = false)
```

The author's points (1..kMaxCurvePoints, finite — preconditions, checked here, before anything moves), the motion config (copied and validated), the timeout (seconds, the WHOLE curve including any boot wait and its end; 0 = config.defaultTimeout) and the direction (`reverse`: the robot's back leads; every heading is still the NOSE's).

*function, declared at [`include/shulib/motion/curve_follow.hpp:188`](../../include/shulib/motion/curve_follow.hpp#L188).*

<a id="curvefollow-start"></a>

### `CurveFollow::start`

```cpp
void start() override
```

Arm, or fully re-arm: the path is re-built at the next first live tick (from the pose then), the end MoveToPose is discarded, the guards, the observer and the stall check reset, the watchdog restarts. Commands no motors.

*function, declared at [`include/shulib/motion/curve_follow.hpp:204`](../../include/shulib/motion/curve_follow.hpp#L204).*

<a id="curvefollow-tick"></a>

### `CurveFollow::tick`

```cpp
[[nodiscard]] control::ExitReason tick() override
```

One control tick (the file header: watchdog, end phase, guards, command). Returns Running until the end MoveToPose settles (Settled), the watchdog fires, a guard fires or the path is refused (TimedOut, MotionTimeout raised naming this motion). After any non-Running verdict a no-op returning it. At most one DebugRecord per call.

*function, declared at [`include/shulib/motion/curve_follow.hpp:230`](../../include/shulib/motion/curve_follow.hpp#L230).*

<a id="curvefollow-cancel"></a>

### `CurveFollow::cancel`

```cpp
void cancel() override
```

The cancel contract (motion.hpp): safe state whenever started, verdict only if still running, Idle untouched, idempotent, never raises. In the end phase the end MoveToPose is cancelled too — it applies the safe state and emits the exit record.

*function, declared at [`include/shulib/motion/curve_follow.hpp:377`](../../include/shulib/motion/curve_follow.hpp#L377).*

<a id="curvefollow-exitreason"></a>

### `CurveFollow::exitReason`

```cpp
[[nodiscard]] control::ExitReason exitReason() const noexcept override
```

The cached verdict (Running until the first exit).

*function, declared at [`include/shulib/motion/curve_follow.hpp:402`](../../include/shulib/motion/curve_follow.hpp#L402).*

<a id="curvefollow-state"></a>

### `CurveFollow::state`

```cpp
[[nodiscard]] MotionState state() const noexcept override
```

The motion-layer state (Running through the end phase too).

*function, declared at [`include/shulib/motion/curve_follow.hpp:404`](../../include/shulib/motion/curve_follow.hpp#L404).*

<a id="curvefollow-name"></a>

### `CurveFollow::name`

```cpp
[[nodiscard]] const char* name() const noexcept override
```

"CurveTo", or "ReverseCurveTo" — the name the result line and the MotionTimeout text use.

*function, declared at [`include/shulib/motion/curve_follow.hpp:406`](../../include/shulib/motion/curve_follow.hpp#L406).*

<a id="curvefollow-curvestatus"></a>

### `CurveFollow::curveStatus`

```cpp
[[nodiscard]] const CurveStatus& curveStatus() const noexcept
```

The most recent tick's account (phase, progress, the look-ahead, the guard, the start check, the hand-off).

*function, declared at [`include/shulib/motion/curve_follow.hpp:412`](../../include/shulib/motion/curve_follow.hpp#L412).*

<a id="curvefollow-path"></a>

### `CurveFollow::path`

```cpp
[[nodiscard]] const CurvePath& path() const noexcept
```

The path (built at the first live tick; before it, built() is false).

*function, declared at [`include/shulib/motion/curve_follow.hpp:414`](../../include/shulib/motion/curve_follow.hpp#L414).*

<a id="curvefollow-endpose"></a>

### `CurveFollow::endPose`

```cpp
[[nodiscard]] const math::Pose2d& endPose() const noexcept
```

The END pose (the robot's nose): known once the path is built; before it, the last point with its heading (0 if it has none).

*function, declared at [`include/shulib/motion/curve_follow.hpp:417`](../../include/shulib/motion/curve_follow.hpp#L417).*

<a id="curvefollow-endmotion"></a>

### `CurveFollow::endMotion`

```cpp
[[nodiscard]] const MoveToPose* endMotion() const noexcept
```

The end MoveToPose once handed off; null before.

*function, declared at [`include/shulib/motion/curve_follow.hpp:419`](../../include/shulib/motion/curve_follow.hpp#L419).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 94 lines, click to expand</summary>

```text

 CurveFollow — the curve verbs' motion (chunk CURVE1, 2026-11): the robot drives a smooth path
 through the author's points WITHOUT stopping at them — pure pursuit on the path curve_path.hpp
 builds, a speed plan that slows before each bend, two guards that stop a curve the robot has
 lost, and an END that is an ordinary MoveToPose. Chassis::curveTo / reverseCurveTo and the
 Routine steps of the same names build one; a Tier-3 caller may too (from chassis.deps()).

 ── What C1's D12 says, and the one exception this file is ─────────────────────────────
 Every other motion keeps C1's D12 (amended by TANK1): the library never plans a turn toward a
 target and never aims at a point. By the team lead's ruling of 2026-10-30 (G3a's D16 (c)) this
 motion — and only it — steers toward a point: a point a look-ahead ahead of the robot ON THE PATH
 THE AUTHOR DREW, or, in the last look-ahead, on the end's own line extended past the end. That is
 pure pursuit, LemLib's `follow` — the behaviour D12 kept out of the core, admitted inside the two
 curve verbs only. It never aims at the END point itself, never plans a turn the path does not
 contain (a first point more than 90 deg off the nose, and a path that doubles back, are refused
 before the robot moves), and never turns round to drive backwards (reverse is the author's verb).

 ── The tick (the order is load-bearing: watchdog, end phase, guards, command) ──────────────
   wait-for-live  zero volts while the estimate is Uninitialized; the watchdog runs through the
                  wait (motion.hpp). At the FIRST live tick the path is BUILT from the live pose
                  (never at construction: the boot window has no heading): the near-start drop
                  (one Info line), checkCurve() — Invalid / StartBehind exit at once, TimedOut +
                  MotionTimeout + one Warn naming the point ("curve not driven: ..."), the robot
                  never moves; TightBend logs one Warn and drives — then ONE Info line ("curve: 4
                  points, 143.4 in, tightest bend 7.4 in radius, bends at <= 21 in/s").
   1. watchdog    FIRST, every live tick: a curve that runs out of time times out as CurveTo,
                  never as its end's MoveToPose (whose own watchdog has the curve's FULL budget,
                  armed later, so it can never fire first). The one consequence, stated: on the
                  very tick the budget runs out, the curve exits TimedOut even if its end would
                  have settled on that tick — the reverse of MoveToPose's own order (Settled
                  beats a simultaneous TimedOut). One tick's difference; pinned (CV13).
   2. end phase   once handed off, the end MoveToPose ticks and its verdict is the curve's.
                  The hand-off fires on the FIRST of (path left = length - closest point):
                    (1) path left <= handoffDistance AND heading within handoffHeading of the end
                        heading AND within handoffCross of the end's line AND inside TANK1's
                        window (below) — the clean hand-off;
                    (2) path left <= handoffDistance x handoffLast AND tankLegEngages(heading
                        error) — the SAME function MoveToPose decides its steering with, on the
                        same pose, so the end is always steered (TANK1's 15 deg window). With
                        TANK1 switched off (tankSteer.maxSteer = 0) the end MoveToPose never
                        steers and never warns, so there is no window to wait for: (1) and (2)
                        drop the window, (3) never warns;
                    (3) the closest point reaches the end of the path (path left <= 0.125 in)
                        still outside that window: handed off anyway, to a MoveToPose built with
                        tankSteer.maxSteer = 0 (TANK1 off — the same unsteered line it would have
                        driven, without its "turn first: face()" Warn, which would blame the
                        author for a turn they never had to make), and ONE Warn in the curve's
                        own words ("curve end: 21.1 deg off the end heading at the end of the
                        path (steers within 15.0) -- unsteered; more run-up, a gentler last bend
                        or a lower speed into it"; on a tank only — a holonomic end strafes).
                  The end MoveToPose is built at the hand-off (std::optional, no heap) with the
                  curve's deps, its MotionConfig copy and its FULL timeout, and ticked on the same
                  tick, against the same pose. Its yaw observer starts from 0, like any moveTo's
                  (carrying the curve's estimate in was probed on the host test plant and made it
                  WORSE). So a curve ENDS like a moveTo: the same stop, the same arrival check, the
                  same per-step arrival tolerance (a step's positionTolerance reaches it through
                  the config, never as an option), and the same log.
   3. guards      (pursuit only) the direction of travel more than lostHeading (60 deg) off the
                  path's direction for lostTime (0.25 s), or less than 1 in of progress along the
                  path in stuckTime (0.75 s) while more than handoffDistance (12 in) of path is
                  left: stop (0 V), TimedOut, MotionTimeout, one Warn naming the guard ("curve
                  lost: ..."). So the HEADING guard runs on every pursuit tick, the last 12 in
                  included, until the hand-off; only the no-progress guard is off in the end zone
                  (the end is slow on purpose). From the hand-off on, only the time budget stops
                  the curve. NOT covered, stated: a side dying inside the end zone (the curve hands
                  off, and the step's time budget ends it — travel stays bounded), and a jam on a
                  robot whose odometry is its drive encoders (spinning wheels read as progress).
                  A side dying EARLY can keep making progress while it pivots: on the host test
                  plant (7 routes, either side, dead from 0.1-0.5 s) a guard stopped every such run
                  within 2.7 s of the death and 9.5 in of travel after it (CV5).
   4. command     s = the closest point, searched FORWARD only over searchWindow of path; the
                  look-ahead point lookAhead further on; in the frame of the direction of travel
                  (the robot's back's frame reversing) the arc through it has curvature
                  k = 2y / (x^2 + y^2); v = min(the speed plan at s, endSpeedGain x translation.kP
                  x path left, the last commanded speed + accel x dt); omega = v k minus TANK1's
                  yaw observer's estimate (measured minus commanded yaw rate, low-passed — the same
                  step as move_to_pose.hpp's, written once more here: curveYawObserverStep, pinned
                  equal by test; held at 0 when TANK1 is switched off, tankSteer.maxSteer = 0). The command goes through applyCommandPipeline in the BODY frame
                  with vy = 0 (the omega clamp, the uniform desaturate that keeps the arc), then
                  OdoStallCheck and the health observables, exactly as MoveToPose.

 ── Records (A1: at most one per tick, lazy; motors stopped before an exit record) ────────
 targetPose = the END pose on every live record (so the result line's ` xt` is "off the end's
 line", and `ov` is along the start-to-end chord — read ` xt` and the end error for a curve),
 errors to the end pose, `commanded` the final achievable command in the FIELD frame,
 tankSteerActive when the yaw correction is above TANK1's legibility floor. From the hand-off on,
 the end MoveToPose emits the tick's record. The record schema does not change.

 Every tunable this reads is CurveConfig's (motion_config.hpp), PROVISIONAL (A4: HA-246,
 HA-248), and TankSteerConfig's maxSteer / yawObserverTime / yawCorrectionLimit (HA-179). Its own
 constants are logic constants, not hardware beliefs: kEndOfPath (0.125 in), kProgressStep (1 in)
 and kFirstTickDt (0.01 s). It reads the pose only through the Localizer: on a robot whose odometry is its drive
 encoders, how true that pose stays in a skid-steer bend is A4 register HA-249. Proven on the host
 test plant only: nothing of it has run on a robot.
```

</details>
