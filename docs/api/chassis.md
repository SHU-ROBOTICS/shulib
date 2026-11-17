<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/chassis/chassis.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `chassis.hpp`

Chassis — the public facade every auton is written against.

This header declares **6** types (53 members) and **1** type alias.

Extracted from [`include/shulib/chassis/chassis.hpp`](../../include/shulib/chassis/chassis.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in [How it works](../guide/README.md); worked recipes live in [Recipes](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`CurvePoint`](#curvepoint) — *type alias*
- [`struct ChassisConfig`](#struct-chassisconfig)
  - [`motion`](#chassisconfig-motion)
  - [`scheduler`](#chassisconfig-scheduler)
- [`struct MotionOptions`](#struct-motionoptions)
  - [`timeout`](#motionoptions-timeout)
  - [`maxLinearSpeed`](#motionoptions-maxlinearspeed)
  - [`maxAngularSpeed`](#motionoptions-maxangularspeed)
  - [`positionTolerance`](#motionoptions-positiontolerance)
  - [`headingTolerance`](#motionoptions-headingtolerance)
  - [`validate`](#motionoptions-validate)
- [`struct TrajectoryResult`](#struct-trajectoryresult)
  - [`exit`](#trajectoryresult-exit)
  - [`completedLegs`](#trajectoryresult-completedlegs)
  - [`totalLegs`](#trajectoryresult-totallegs)
  - [`succeeded`](#trajectoryresult-succeeded)
- [`struct LegObserver`](#struct-legobserver)
  - [`fn`](#legobserver-fn)
  - [`context`](#legobserver-context)
- [`class Chassis`](#class-chassis)
  - [`Chassis`](#chassis-chassis)
  - [`Chassis (overload 2)`](#chassis-chassis-2)
  - [`Chassis (overload 3)`](#chassis-chassis-3)
  - [`operator=`](#chassis-operator-eq)
  - [`operator= (overload 2)`](#chassis-operator-eq-2)
  - [`~Chassis`](#chassis-destructor-chassis)
  - [`moveTo`](#chassis-moveto)
  - [`strafeTo`](#chassis-strafeto)
  - [`turnTo`](#chassis-turnto)
  - [`moveTo (overload 2)`](#chassis-moveto-2)
  - [`turnTo (overload 2)`](#chassis-turnto-2)
  - [`followTrajectory`](#chassis-followtrajectory)
  - [`followTrajectory (overload 2)`](#chassis-followtrajectory-2)
  - [`curveTo`](#chassis-curveto)
  - [`curveTo (overload 2)`](#chassis-curveto-2)
  - [`reverseCurveTo`](#chassis-reversecurveto)
  - [`reverseCurveTo (overload 2)`](#chassis-reversecurveto-2)
  - [`brake`](#chassis-brake)
  - [`hold`](#chassis-hold)
  - [`wait`](#chassis-wait)
  - [`drive`](#chassis-drive)
  - [`cancel`](#chassis-cancel)
  - [`waitUntil`](#chassis-waituntil)
  - [`pose`](#chassis-pose)
  - [`setPose`](#chassis-setpose)
  - [`reflection`](#chassis-reflection)
  - [`strafeAuthority`](#chassis-strafeauthority)
  - [`lastExitReason`](#chassis-lastexitreason)
  - [`lastCompleted`](#chassis-lastcompleted)
  - [`motionConfig`](#chassis-motionconfig)
  - [`deps`](#chassis-deps)
  - [`scheduler`](#chassis-scheduler)
  - [`scheduler (overload 2)`](#chassis-scheduler-2)
- [`class MirroredRun`](#class-mirroredrun)
  - [`MirroredRun`](#mirroredrun-mirroredrun)
  - [`~MirroredRun`](#mirroredrun-destructor-mirroredrun)
  - [`MirroredRun (overload 2)`](#mirroredrun-mirroredrun-2)
  - [`MirroredRun (overload 3)`](#mirroredrun-mirroredrun-3)
  - [`operator=`](#mirroredrun-operator-eq)
  - [`operator= (overload 2)`](#mirroredrun-operator-eq-2)

<a id="curvepoint"></a>

## `CurvePoint`

```cpp
using CurvePoint = motion::CurvePoint
```

One point of a curve verb (curveTo / reverseCurveTo): `{x, y}` or `{x, y, heading}` — the motion layer's motion::CurvePoint, named here so an auton never needs the motion namespace (and, in braces, never names the type at all). CURVE1, F6-additive.

*type alias, declared at [`include/shulib/chassis/chassis.hpp:308`](../../include/shulib/chassis/chassis.hpp#L308).*

<a id="struct-chassisconfig"></a>

## `struct ChassisConfig`

```cpp
struct ChassisConfig
```

Everything configurable about a Chassis, in one place. Both members are the lower layers' own config types passed through WHOLE — so an additive field there (e.g. a future per-wheel speed budget in MotionConfig, the C3 §11 flag) flows through this surface with no reshape.

*struct, declared at [`include/shulib/chassis/chassis.hpp:314`](../../include/shulib/chassis/chassis.hpp#L314).*

<a id="chassisconfig-motion"></a>

### `ChassisConfig::motion`

```cpp
motion::MotionConfig motion{}
```

gains/budgets/tolerances (HA-50/51/52)

*field, declared at [`include/shulib/chassis/chassis.hpp:315`](../../include/shulib/chassis/chassis.hpp#L315).*

<a id="chassisconfig-scheduler"></a>

### `ChassisConfig::scheduler`

```cpp
motion::MotionSchedulerConfig scheduler{}
```

fault policy mask + loop monitor

*field, declared at [`include/shulib/chassis/chassis.hpp:316`](../../include/shulib/chassis/chassis.hpp#L316).*

<a id="struct-motionoptions"></a>

## `struct MotionOptions`

```cpp
struct MotionOptions
```

Per-call knobs for the blocking verbs. 0 (the default) = "use the ChassisConfig value". Validated finite and >= 0 at each call.  FROZEN F6 NOTE (D2): the fields BELOW are frozen (name/type/meaning); the field SET is deliberately additive-open — a future knob is a new field with a 0/"config default" meaning, never a reshape of these.

*struct, declared at [`include/shulib/chassis/chassis.hpp:325`](../../include/shulib/chassis/chassis.hpp#L325).*

<a id="motionoptions-timeout"></a>

### `MotionOptions::timeout`

```cpp
units::Time timeout{0.0}
```

Watchdog bound for this motion, INCLUDING any boot wait. Typed time (D2): `{.timeout = 5_s}` / `{.timeout = 500_ms}` — a bare double does not compile, so "500 meaning milliseconds" cannot silently become 500 seconds of match time.

*field, declared at [`include/shulib/chassis/chassis.hpp:330`](../../include/shulib/chassis/chassis.hpp#L330).*

<a id="motionoptions-maxlinearspeed"></a>

### `MotionOptions::maxLinearSpeed`

```cpp
units::Velocity maxLinearSpeed{0.0}
```

Field-frame linear speed budget for this motion (in/s) — the norm cap AND the base of the strafe-authority clamp, exactly as in MotionConfig. The per-wheel budget (maxWheelSpeed) is deliberately NOT scaled with it: that is a hardware envelope, not a per-leg intent.

*field, declared at [`include/shulib/chassis/chassis.hpp:335`](../../include/shulib/chassis/chassis.hpp#L335).*

<a id="motionoptions-maxangularspeed"></a>

### `MotionOptions::maxAngularSpeed`

```cpp
units::AngularVelocity maxAngularSpeed{0.0}
```

Yaw-rate budget for this motion (rad/s).

*field, declared at [`include/shulib/chassis/chassis.hpp:337`](../../include/shulib/chassis/chassis.hpp#L337).*

<a id="motionoptions-positiontolerance"></a>

### `MotionOptions::positionTolerance`

```cpp
units::Length positionTolerance{0.0}
```

ARRIVAL TOLERANCE for this motion's position: "arrive within 2 in" is `{.positionTolerance = 2_in}`. 0 (the default) = the robot's own settle numbers (MotionConfig::translationSettle), bit for bit. A leg ends when the robot is close enough AND slow enough, and on a drive leg the SPEED bound is the one that decides (the robot is already well inside 0.5 in when it first slows under 1 in/s) — so a tolerance looser than the robot's own loosens BOTH, in proportion: 2 in means "within 2 in and slower than 4 in/s" (0.5 in / 1 in/s scaled by 2 / 0.5). A tolerance TIGHTER than the robot's own tightens only the distance. motion::loosenedSettle is the one formula (it is total: a robot whose own distance is 0 keeps its own speed bound). The leg can therefore end with the robot still rolling at up to that speed: it stops dead on the host test plant, and how far a real robot coasts after it is unmeasured (A4: HA-216). Read by every verb whose motion reads the translation settle — moveTo, driveTo / reverseTo (through moveTo), each followTrajectory leg, strafeTo, hold() (Settled iff within it when the hold window ends), and curveTo / reverseCurveTo (through the curve's end moveTo — CURVE1); turnTo / face / faceAway and brake() ignore it. Validated finite and >= 0. (APPENDED at STEP1-A, after the frozen three, so every designated initializer keeps compiling.)

*field, declared at [`include/shulib/chassis/chassis.hpp:357`](../../include/shulib/chassis/chassis.hpp#L357).*

<a id="motionoptions-headingtolerance"></a>

### `MotionOptions::headingTolerance`

```cpp
math::Angle headingTolerance{}
```

ARRIVAL TOLERANCE for this motion's heading: "within 5°" is `{.headingTolerance = 5_deg}`. A zero Angle (the default) = the robot's own MotionConfig::headingSettle, bit for bit. The same rule as positionTolerance, on the heading settle (1.15° / 0.30 rad/s on the defaults): looser loosens the distance AND the turn-rate bound in proportion (5° → 1.31 rad/s, about 75°/s), tighter tightens only the distance (a smaller rate bound could sit inside the IMU's noise and never settle — motion_config.hpp's header). Read by turnTo / face / faceAway, by every MoveToPose-engine verb above (each also settles its heading), and by curveTo / reverseCurveTo (through the curve's end moveTo — CURVE1); brake() ignores it. A math::Angle, so it is written 2_deg like every heading — and so it is wrapped: a negative value, or one that WRAPS to a negative one (359_deg stores −1°), is refused at the call, never read as a size. (APPENDED at STEP1-A.)

*field, declared at [`include/shulib/chassis/chassis.hpp:372`](../../include/shulib/chassis/chassis.hpp#L372).*

<a id="motionoptions-validate"></a>

### `MotionOptions::validate`

```cpp
void validate() const
```

Reject nonsense before anything moves: every field must be finite and >= 0. Called by each verb at the door, so a bad option value is a loud error at the call site rather than a mystery mid-motion.

*function, declared at [`include/shulib/chassis/chassis.hpp:377`](../../include/shulib/chassis/chassis.hpp#L377).*

<a id="struct-trajectoryresult"></a>

## `struct TrajectoryResult`

```cpp
struct TrajectoryResult
```

What followTrajectory did — which leg count it completed and how the last attempted leg exited. (ExitReason alone would lose WHERE the chain broke; the next thing a routine does after a failed trajectory legitimately depends on how far it got.)

*struct, declared at [`include/shulib/chassis/chassis.hpp:402`](../../include/shulib/chassis/chassis.hpp#L402).*

<a id="trajectoryresult-exit"></a>

### `TrajectoryResult::exit`

```cpp
control::ExitReason exit = control::ExitReason::Settled
```

last attempted leg's verdict

*field, declared at [`include/shulib/chassis/chassis.hpp:403`](../../include/shulib/chassis/chassis.hpp#L403).*

<a id="trajectoryresult-completedlegs"></a>

### `TrajectoryResult::completedLegs`

```cpp
int completedLegs = 0
```

legs that SETTLED (== totalLegs on success)

*field, declared at [`include/shulib/chassis/chassis.hpp:404`](../../include/shulib/chassis/chassis.hpp#L404).*

<a id="trajectoryresult-totallegs"></a>

### `TrajectoryResult::totalLegs`

```cpp
int totalLegs = 0
```

waypoints given

*field, declared at [`include/shulib/chassis/chassis.hpp:405`](../../include/shulib/chassis/chassis.hpp#L405).*

<a id="trajectoryresult-succeeded"></a>

### `TrajectoryResult::succeeded`

```cpp
[[nodiscard]] bool succeeded() const noexcept
```

True only if the last attempted leg SETTLED and every leg was completed. Note what this means for a value-initialized TrajectoryResult (0 of 0 legs, exit Settled): it reads as success. That is correct here — this verb requires at least one waypoint, so a result it produces always has legs — but any code that holds a TrajectoryResult BEFORE running one must initialize `exit` to Running instead (Routine::lastTrajectory does).

*function, declared at [`include/shulib/chassis/chassis.hpp:412`](../../include/shulib/chassis/chassis.hpp#L412).*

<a id="struct-legobserver"></a>

## `struct LegObserver`

```cpp
struct LegObserver
```

STEP1-B: what a moveTo / turnTo OBSERVER overload calls after every tick of the leg that did not end it — neither the motion's exit tick nor one the scheduler's fault policy ends (the file banner's "leg OBSERVER"): `fn(context, progress)`. A plain function pointer and a context pointer — no heap, no std::function (runner::Handler's pattern). `fn` null = no observer (the leg runs exactly as the plain verb). It runs INSIDE the scheduler's tick: no blocking verb, no async() or cancel(), and it must not throw (the banner says why).

*struct, declared at [`include/shulib/chassis/chassis.hpp:423`](../../include/shulib/chassis/chassis.hpp#L423).*

<a id="legobserver-fn"></a>

### `LegObserver::fn`

```cpp
void (*fn)(void* context, const motion::LegProgress& progress) = nullptr
```

Called once per non-exit tick with the leg's progress; null = nothing is called.

*field, declared at [`include/shulib/chassis/chassis.hpp:425`](../../include/shulib/chassis/chassis.hpp#L425).*

<a id="legobserver-context"></a>

### `LegObserver::context`

```cpp
void* context = nullptr
```

Handed back to `fn` untouched (the caller's object, borrowed for the leg).

*field, declared at [`include/shulib/chassis/chassis.hpp:427`](../../include/shulib/chassis/chassis.hpp#L427).*

<a id="class-chassis"></a>

## `class Chassis`

```cpp
class Chassis
```

The public facade every autonomous routine is written against: the blocking motion verbs, the frame-explicit manual verb, control, state, and the Tier-3 seam — over one owned MotionScheduler. FROZEN (register row F6, locked 2026-08-12); the file banner above carries the design reasoning behind every shape here, and is meant to be read before changing anything.

*class, declared at [`include/shulib/chassis/chassis.hpp:444`](../../include/shulib/chassis/chassis.hpp#L444).*

<a id="chassis-chassis"></a>

### `Chassis::Chassis`

```cpp
explicit Chassis(const motion::MotionDeps& deps, motion::ITickPacer& pacer, const ChassisConfig& config = {})
```

`deps` is the same validated bundle every motion takes; `pacer` is the seam through which the world advances during blocking verbs (host sim: step the plant; robot: delay to the tick boundary — R1/R3 build that one). All deps pointees AND the pacer must outlive the Chassis; the facade borrows, it does not own (header: construction).

*function, declared at [`include/shulib/chassis/chassis.hpp:451`](../../include/shulib/chassis/chassis.hpp#L451).*

<a id="chassis-chassis-2"></a>

### `Chassis::Chassis (overload 2)`

```cpp
Chassis(const Chassis&) = delete
```

Neither copyable nor movable: the Chassis OWNS the scheduler, which is pinned in place by its own self-referential command-id stamp, so a copy or a move would leave that stamp pointing at the wrong object. Hold a `Chassis&`; construct it once, where it will live.

*function, declared at [`include/shulib/chassis/chassis.hpp:461`](../../include/shulib/chassis/chassis.hpp#L461).*

<a id="chassis-chassis-3"></a>

### `Chassis::Chassis (overload 3)`

```cpp
Chassis(Chassis&&) = delete
```

*Covered by the comment on [`Chassis (overload 2)`](#chassis-chassis-2) — one comment documents this run of special members.*

*function, declared at [`include/shulib/chassis/chassis.hpp:462`](../../include/shulib/chassis/chassis.hpp#L462).*

<a id="chassis-operator-eq"></a>

### `Chassis::operator=`

```cpp
Chassis& operator=(const Chassis&) = delete
```

*Covered by the comment on [`Chassis (overload 2)`](#chassis-chassis-2) — one comment documents this run of special members.*

*function, declared at [`include/shulib/chassis/chassis.hpp:463`](../../include/shulib/chassis/chassis.hpp#L463).*

<a id="chassis-operator-eq-2"></a>

### `Chassis::operator= (overload 2)`

```cpp
Chassis& operator=(Chassis&&) = delete
```

*Covered by the comment on [`Chassis (overload 2)`](#chassis-chassis-2) — one comment documents this run of special members.*

*function, declared at [`include/shulib/chassis/chassis.hpp:464`](../../include/shulib/chassis/chassis.hpp#L464).*

<a id="chassis-destructor-chassis"></a>

### `Chassis::~Chassis`

```cpp
~Chassis() = default
```

*Covered by the comment on [`Chassis (overload 2)`](#chassis-chassis-2) — one comment documents this run of special members.*

*function, declared at [`include/shulib/chassis/chassis.hpp:465`](../../include/shulib/chassis/chassis.hpp#L465).*

<a id="chassis-moveto"></a>

### `Chassis::moveTo`

```cpp
control::ExitReason moveTo(const math::Pose2d& target, const MotionOptions& options = {})
```

Drive to `target` (FIELD pose): the decoupled holonomic engine — translation and rotation simultaneous and independent (C1's thesis). On a tank, a leg that starts within 15° of its target heading also steers back onto its line while it drives, and a leg that starts farther off runs unsteered and logs one Warn (header: TANK STEERING). It never plans a turn.

*function, declared at [`include/shulib/chassis/chassis.hpp:474`](../../include/shulib/chassis/chassis.hpp#L474).*

<a id="chassis-strafeto"></a>

### `Chassis::strafeTo`

```cpp
control::ExitReason strafeTo(units::Length x, units::Length y, const MotionOptions& options = {})
```

Translate to FIELD (x, y) while actively HOLDING the heading the robot has at its first live tick. On tank (authority 0) an off-line target honestly exits TimedOut (C1's drivetrain honesty) — unchanged by TANK1: strafeTo never takes moveTo's tank steering, because holding the heading it found IS its contract (on a tank, a moveTo with the current heading is the translate that steers).

*function, declared at [`include/shulib/chassis/chassis.hpp:488`](../../include/shulib/chassis/chassis.hpp#L488).*

<a id="chassis-turnto"></a>

### `Chassis::turnTo`

```cpp
control::ExitReason turnTo(math::Angle heading, const MotionOptions& options = {})
```

Rotate in place to a FIELD heading, always the short way (F3's shortest signed error; exact ±180° resolves CCW, deterministically).

*function, declared at [`include/shulib/chassis/chassis.hpp:500`](../../include/shulib/chassis/chassis.hpp#L500).*

<a id="chassis-moveto-2"></a>

### `Chassis::moveTo (overload 2)`

```cpp
control::ExitReason moveTo(const math::Pose2d& target, const MotionOptions& options, LegObserver observer)
```

STEP1-B: moveTo, WATCHED — the same motion, run the same way, with `observer` called after every tick that did not end the leg, given the distance covered along the straight line from the leg's live start to `target` (the file banner's "leg OBSERVER"). A null `observer.fn`: exactly moveTo(target, options). (F6-additive, pinned; a Routine's partway triggers ride it.)

*function, declared at [`include/shulib/chassis/chassis.hpp:512`](../../include/shulib/chassis/chassis.hpp#L512).*

<a id="chassis-turnto-2"></a>

### `Chassis::turnTo (overload 2)`

```cpp
control::ExitReason turnTo(math::Angle heading, const MotionOptions& options, LegObserver observer)
```

STEP1-B: turnTo, WATCHED — as the moveTo overload above, the progress being the angle turned toward `heading` since the turn's live start (the short way, as turnTo turns). A null `observer.fn`: exactly turnTo(heading, options). (F6-additive, pinned.)

*function, declared at [`include/shulib/chassis/chassis.hpp:524`](../../include/shulib/chassis/chassis.hpp#L524).*

<a id="chassis-followtrajectory"></a>

### `Chassis::followTrajectory`

```cpp
TrajectoryResult followTrajectory(std::span<const math::Pose2d> waypoints, const MotionOptions& options = {})
```

Chain `waypoints` as sequential moveTo legs, settling at each; stop at the first non-Settled leg (header: followTrajectory). `options` apply PER LEG (each leg is one scheduled motion with its own watchdog). Precondition: at least one waypoint. G2 boundary in the header.

*function, declared at [`include/shulib/chassis/chassis.hpp:538`](../../include/shulib/chassis/chassis.hpp#L538).*

<a id="chassis-followtrajectory-2"></a>

### `Chassis::followTrajectory (overload 2)`

```cpp
TrajectoryResult followTrajectory(std::initializer_list<math::Pose2d> waypoints, const MotionOptions& options = {})
```

Brace-list convenience: followTrajectory({a, b, c}).

*function, declared at [`include/shulib/chassis/chassis.hpp:569`](../../include/shulib/chassis/chassis.hpp#L569).*

<a id="chassis-curveto"></a>

### `Chassis::curveTo`

```cpp
control::ExitReason curveTo(std::span<const CurvePoint> points, const MotionOptions& options = {})
```

Drive a SMOOTH path from where the robot is, along its nose, through `points` (FIELD inches; 1 to 16; `{x, y}` or `{x, y, heading}` — a heading is the robot's NOSE), without stopping at them, and END like moveTo at the last point (header: curveTo). Refuses before moving — TimedOut, one Warn naming the point — a first point more than 90 deg off the nose ("turn first: face()") or a path that doubles back; stops a curve it has lost. Options as on moveTo (the timeout covers the WHOLE curve; the arrival tolerances reach its end). Preconditions, checked before anything moves: 1 to 16 points, every x and y finite.

*function, declared at [`include/shulib/chassis/chassis.hpp:585`](../../include/shulib/chassis/chassis.hpp#L585).*

<a id="chassis-curveto-2"></a>

### `Chassis::curveTo (overload 2)`

```cpp
control::ExitReason curveTo(std::initializer_list<CurvePoint> points, const MotionOptions& options = {})
```

Brace-list convenience: curveTo({{24_in, 12_in}, {48_in, 24_in, 0_deg}}).

*function, declared at [`include/shulib/chassis/chassis.hpp:591`](../../include/shulib/chassis/chassis.hpp#L591).*

<a id="chassis-reversecurveto"></a>

### `Chassis::reverseCurveTo`

```cpp
control::ExitReason reverseCurveTo(std::span<const CurvePoint> points, const MotionOptions& options = {})
```

curveTo with the robot's BACK leading the whole way. Every heading is still the robot's NOSE (a robot backing into a spot facing 0 deg ends with `0_deg`, not 180): writing the direction of travel makes the path double back, and it is refused.

*function, declared at [`include/shulib/chassis/chassis.hpp:599`](../../include/shulib/chassis/chassis.hpp#L599).*

<a id="chassis-reversecurveto-2"></a>

### `Chassis::reverseCurveTo (overload 2)`

```cpp
control::ExitReason reverseCurveTo(std::initializer_list<CurvePoint> points, const MotionOptions& options = {})
```

Brace-list convenience: reverseCurveTo({{-12_in, 30_in}, {-36_in, 36_in, 0_deg}}).

*function, declared at [`include/shulib/chassis/chassis.hpp:605`](../../include/shulib/chassis/chassis.hpp#L605).*

<a id="chassis-brake"></a>

### `Chassis::brake`

```cpp
control::ExitReason brake(const MotionOptions& options = {})
```

Stop the drivetrain (0 V under Brake) and block until the ESTIMATE certifies rest (or the watchdog fires). The controlled end-of-motion stop; cancel() is the uncontrolled one.

*function, declared at [`include/shulib/chassis/chassis.hpp:616`](../../include/shulib/chassis/chassis.hpp#L616).*

<a id="chassis-hold"></a>

### `Chassis::hold`

```cpp
control::ExitReason hold(units::Time duration, const MotionOptions& options = {})
```

Actively hold the pose the robot has at its first live tick for `duration`, driving back any disturbance with full holonomic authority; Settled iff still within tolerance when the window ends. `duration` must be finite and > 0 (HoldPose's precondition). Typed time (D2): hold(500_ms) — hold(500) does not compile, so "500 meaning milliseconds" cannot hold pose for 500 s of a 15 s auton.

*function, declared at [`include/shulib/chassis/chassis.hpp:628`](../../include/shulib/chassis/chassis.hpp#L628).*

<a id="chassis-wait"></a>

### `Chassis::wait`

```cpp
void wait(units::Time duration)
```

Wait, commanding nothing, for `duration` — then return. The world keeps advancing and the active motion (if any) keeps ticking — the same contract as waitUntil; the drive keeps whatever state the last verb left it in (after a settled motion: stopped). Deliberately DISTINCT from hold(): wait() never energizes the drive — this is the "sit still for the alliance partner" beat (D2; adopted from D1's finding that the naive waitUntil(false-pred, t) spelling logs a spurious Warn on every deliberate pause, and the Warn-free spelling needed Tier-3 plumbing). Returns void: a wait has no failure mode — a pacer that stops advancing the clock trips the scheduler's loud precondition, a programming error rather than a verdict. Warn-free and bounded by construction: the deadline predicate is time-monotone, so the internal timeout backstop is unreachable slack. `duration` must be finite and > 0 (typed: wait(2_s) / wait(500_ms)).

*function, declared at [`include/shulib/chassis/chassis.hpp:648`](../../include/shulib/chassis/chassis.hpp#L648).*

<a id="chassis-drive"></a>

### `Chassis::drive`

```cpp
void drive(const math::ChassisSpeeds& speeds, math::Frame frame)
```

Command a chassis velocity directly, in the frame the CALLER names (no default — header: drive). Pre-empts any active motion; owns one loop iteration (estimate update → shared pipeline → health → record). Precondition: all three components finite.

*function, declared at [`include/shulib/chassis/chassis.hpp:665`](../../include/shulib/chassis/chassis.hpp#L665).*

<a id="chassis-cancel"></a>

### `Chassis::cancel`

```cpp
void cancel()
```

Stop the active motion into the defined safe state (0 V + Brake); with no active motion this is the PANIC STOP and still safes the drive.

*function, declared at [`include/shulib/chassis/chassis.hpp:710`](../../include/shulib/chassis/chassis.hpp#L710).*

<a id="chassis-waituntil"></a>

### `Chassis::waitUntil`

```cpp
template <typename Pred> [[nodiscard]] motion::WaitResult waitUntil(Pred&& pred, units::Time timeout)
```

Block until `pred()` holds or `timeout` elapses (required, finite, >= 0; 0 = an honest poll) — the return says which. The active motion (if any) keeps ticking throughout; the world keeps advancing. Timing out logs one Warn and raises NO fault (a timed-out wait is a strategy branch, not a pathology). C2's verb, re-exported with typed time at the public edge (D2); the scheduler's own seconds-double signature is interior, per F3's internal-seconds convention.

*function, declared at [`include/shulib/chassis/chassis.hpp:720`](../../include/shulib/chassis/chassis.hpp#L720).*

<a id="chassis-pose"></a>

### `Chassis::pose`

```cpp
[[nodiscard]] math::Pose2d pose() const
```

The current fused FIELD pose estimate. In a mirrored run (the file banner's MIRRORED RUN): the estimate reflected — robot one's numbers, the frame the route is written in.

*function, declared at [`include/shulib/chassis/chassis.hpp:728`](../../include/shulib/chassis/chassis.hpp#L728).*

<a id="chassis-setpose"></a>

### `Chassis::setPose`

```cpp
void setPose(const math::Pose2d& pose)
```

Seed / teleport the estimated POSITION (x, y) — heading stays IMU-owned (the Localizer's structural choice). Call at auton start with the measured starting pose. In a mirrored run the position is reflected first.

*function, declared at [`include/shulib/chassis/chassis.hpp:738`](../../include/shulib/chassis/chassis.hpp#L738).*

<a id="chassis-reflection"></a>

### `Chassis::reflection`

```cpp
[[nodiscard]] const math::Reflection& reflection() const noexcept
```

MIRROR1: the reflection every FIELD number entering this Chassis passes through, and every pose() leaving it (the file banner's MIRRORED RUN) — the identity always, except inside a chassis::MirroredRun. Read-only: a MirroredRun is the one way to set it. A real-field number (a camera's fix, a localizer pose) bound for a verb inside a mirrored run goes through `reflection().apply(...)` first — a reflection is its own inverse — or the verb reflects it a second time.

*function, declared at [`include/shulib/chassis/chassis.hpp:752`](../../include/shulib/chassis/chassis.hpp#L752).*

<a id="chassis-strafeauthority"></a>

### `Chassis::strafeAuthority`

```cpp
[[nodiscard]] double strafeAuthority() const
```

Read-only passthrough of the drivetrain's sustainable lateral authority (fraction of the linear budget; F5). Routine authors budgeting lateral legs legitimately want it — the difference between a 2 s and a 3 s leg on the H-bot (C3 §11 #2, adopted).

*function, declared at [`include/shulib/chassis/chassis.hpp:758`](../../include/shulib/chassis/chassis.hpp#L758).*

<a id="chassis-lastexitreason"></a>

### `Chassis::lastExitReason`

```cpp
[[nodiscard]] control::ExitReason lastExitReason() const noexcept
```

Exit reason of the most recently finished motion (Settled on a virgin chassis — completedCount() via scheduler() says whether anything ran).

*function, declared at [`include/shulib/chassis/chassis.hpp:764`](../../include/shulib/chassis/chassis.hpp#L764).*

<a id="chassis-lastcompleted"></a>

### `Chassis::lastCompleted`

```cpp
[[nodiscard]] const motion::CompletedMotion& lastCompleted() const noexcept
```

The most recent motion boundary — id/name/exit/abortFault/times (C5's raw material; abortFault names a fault-policy cause).

*function, declared at [`include/shulib/chassis/chassis.hpp:770`](../../include/shulib/chassis/chassis.hpp#L770).*

<a id="chassis-motionconfig"></a>

### `Chassis::motionConfig`

```cpp
[[nodiscard]] const motion::MotionConfig& motionConfig() const noexcept
```

The config the verbs run under (per-call options override per motion).

*function, declared at [`include/shulib/chassis/chassis.hpp:775`](../../include/shulib/chassis/chassis.hpp#L775).*

<a id="chassis-deps"></a>

### `Chassis::deps`

```cpp
[[nodiscard]] const motion::MotionDeps& deps() const noexcept
```

The STAMPED deps bundle — build custom IMotions from THIS and their records carry command ids like the built-in verbs' do.

*function, declared at [`include/shulib/chassis/chassis.hpp:781`](../../include/shulib/chassis/chassis.hpp#L781).*

<a id="chassis-scheduler"></a>

### `Chassis::scheduler`

```cpp
[[nodiscard]] motion::MotionScheduler& scheduler() noexcept
```

The owned scheduler, for async composition / caller-paced tick() / counters. It is the SAME single motion slot the verbs use: async() here pre-empts a facade verb's motion and vice versa (one-active- motion is structural, never relaxed).

*function, declared at [`include/shulib/chassis/chassis.hpp:787`](../../include/shulib/chassis/chassis.hpp#L787).*

<a id="chassis-scheduler-2"></a>

### `Chassis::scheduler (overload 2)`

```cpp
[[nodiscard]] const motion::MotionScheduler& scheduler() const noexcept
```

The same scheduler, read-only — for counters and last-motion state from a `const Chassis&`. Identical object and identical semantics to the non-const overload; the two differ only in what they let you do.

*function, declared at [`include/shulib/chassis/chassis.hpp:791`](../../include/shulib/chassis/chassis.hpp#L791).*

<a id="class-mirroredrun"></a>

## `class MirroredRun`

```cpp
class MirroredRun
```

MIRROR1-A: ONE MIRRORED RUN on a Chassis (the file banner's MIRRORED RUN) — RAII: from its construction until its destruction the Chassis reflects every FIELD number entering it and every pose() leaving it through `reflection`; its destructor restores the identity on EVERY way out (a normal end, a cut, an abandon, a throw unwinding through it). The auton runner (competition/match_runner.hpp) opens one around a mirrored entry's routine (or route) AND its end action; a team's code never needs to. PRECONDITIONS (a programming error, thrown before anything changes): `reflection` is not the identity; no mirrored run is already open on the Chassis (runs do not nest); no motion is active (its target was given in the other frame).

*class, declared at [`include/shulib/chassis/chassis.hpp:1074`](../../include/shulib/chassis/chassis.hpp#L1074).*

<a id="mirroredrun-mirroredrun"></a>

### `MirroredRun::MirroredRun`

```cpp
MirroredRun(Chassis& chassis, const math::Reflection& reflection)
```

Open the run (the class comment's preconditions).

*function, declared at [`include/shulib/chassis/chassis.hpp:1077`](../../include/shulib/chassis/chassis.hpp#L1077).*

<a id="mirroredrun-destructor-mirroredrun"></a>

### `MirroredRun::~MirroredRun`

```cpp
~MirroredRun()
```

Close it: the Chassis's reflection is the identity again.

*function, declared at [`include/shulib/chassis/chassis.hpp:1090`](../../include/shulib/chassis/chassis.hpp#L1090).*

<a id="mirroredrun-mirroredrun-2"></a>

### `MirroredRun::MirroredRun (overload 2)`

```cpp
MirroredRun(const MirroredRun&) = delete
```

Neither copyable nor movable: a copy would close the run twice, a moved-from scope would close a run it no longer owns. Construct it where it lives (the runner: a std::optional, emplaced).

*function, declared at [`include/shulib/chassis/chassis.hpp:1093`](../../include/shulib/chassis/chassis.hpp#L1093).*

<a id="mirroredrun-mirroredrun-3"></a>

### `MirroredRun::MirroredRun (overload 3)`

```cpp
MirroredRun(MirroredRun&&) = delete
```

*Covered by the comment on [`MirroredRun (overload 2)`](#mirroredrun-mirroredrun-2) — one comment documents this run of special members.*

*function, declared at [`include/shulib/chassis/chassis.hpp:1094`](../../include/shulib/chassis/chassis.hpp#L1094).*

<a id="mirroredrun-operator-eq"></a>

### `MirroredRun::operator=`

```cpp
MirroredRun& operator=(const MirroredRun&) = delete
```

*Covered by the comment on [`MirroredRun (overload 2)`](#mirroredrun-mirroredrun-2) — one comment documents this run of special members.*

*function, declared at [`include/shulib/chassis/chassis.hpp:1095`](../../include/shulib/chassis/chassis.hpp#L1095).*

<a id="mirroredrun-operator-eq-2"></a>

### `MirroredRun::operator= (overload 2)`

```cpp
MirroredRun& operator=(MirroredRun&&) = delete
```

*Covered by the comment on [`MirroredRun (overload 2)`](#mirroredrun-mirroredrun-2) — one comment documents this run of special members.*

*function, declared at [`include/shulib/chassis/chassis.hpp:1096`](../../include/shulib/chassis/chassis.hpp#L1096).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 267 lines, click to expand</summary>

```text

 Chassis — the public facade every auton is written against (chunk C4, WS6/M2).

 ═══ STATUS: FROZEN — F6, LOCKED 2026-08-12 (chunk D2, API 2.0) ═══════════════════
 This surface is FROZEN. Every public member below — and the three public
 types ChassisConfig / MotionOptions / TrajectoryResult — changes only with
 a major API-version bump plus a migration note (include/shulib/version.hpp
 states the policy); additive extension (new verbs, new MotionOptions
 fields, new overloads) stays legal and is the intended growth path. The
 freeze is enforced STRUCTURALLY: test/f6_signature_pin_test.cpp asserts
 every frozen member's exact type at compile time and fails the build,
 naming F6, if one drifts. NOT inside F6, deliberately: Routine /
 RoutineResult / RoutineStopCause (routine.hpp — they froze at D3 as their
 OWN register row F10, because the recipe layer is a different tier and can
 version independently of the facade; see routine.hpp's banner); the
 scheduler's and deps bundle's OWN member
 surfaces behind scheduler()/deps() (those belong to C1/C2's layers); and
 the lower-layer config fields reached through ChassisConfig. The
 candidate-era reasoning for each shape is in the C4 completion record and
 the freeze rulings (what joined, what stayed out, and why) in the D2
 completion record (development log, shulib-v2 branch).

 ═══ What this class is ══════════════════════════════════════════════════════════
 The composition root of the MOTION stack: it owns the MotionScheduler and
 wraps C1's primitives in blocking verbs. It DELEGATES everywhere — motion
 logic is C1's, scheduling/fault policy C2's, kinematics C3's, the command
 choreography command_pipeline.hpp's. If a behaviour looks like it lives
 here, its test and its fix belong in the layer that owns it.

   verbs      moveTo · strafeTo · turnTo · followTrajectory · drive(speeds, Frame)
              brake · hold · wait  (C4 candidates + the D2 addition, all in F6)
              curveTo · reverseCurveTo  (chunk CURVE1, appended F6-additively)
   control    cancel (panic stop) · waitUntil(pred, timeout)
   state      pose · setPose · strafeAuthority · lastExitReason · lastCompleted
              reflection  (chunk MIRROR1, F6-additive — with the chassis::MirroredRun scope)
   Tier 3     scheduler() · deps() — the no-ceiling seam

 ═══ ONE deps source (C2's structural handoff, closed here) ══════════════════════
 The facade constructs the scheduler from the caller's MotionDeps and EVERY
 motion from scheduler.deps() — the stamped bundle whose telemetry routes
 through the command-id stamp. At C2 that plumbing was a convention a caller
 had to remember (its one named gap, §5 D10); through the facade there is no
 unstamped path: a verb CANNOT build a motion from raw deps, so every record
 of every facade motion carries its command id structurally. Tier-3 callers
 composing their own IMotion get the same guarantee by building from
 chassis.deps().

 ═══ Construction: the standalone promise (locked principle, master plan §16.2) ═══
 A code-fluent team builds a working Chassis in PLAIN C++ — value-construct a
 kinematics preset (xDrive / hDrive / TankKinematics: drivetrain is config
 data), wire the HAL + Localizer + FaultLatch + HealthMonitor into a
 MotionDeps, pick a pacer, done. No .vexbot file, no VexBuilder, no config
 file of any kind is EVER required; G1's RobotBuilder.from(profile) is an
 additional on-ramp, never the only path. (Pinned by the file-free
 construction test.)

 The facade BORROWS the deps (all pointees + the pacer must outlive it) and
 OWNS the scheduler + its MotionConfig. It deliberately does NOT own the
 Localizer or the HAL: wiring those is the builder's job (G1) or the
 caller's, and owning them here would weld the facade to one localization
 stack — Tier 3 must be able to swap estimators without losing the facade.

 ═══ API semantics the lower layers guarantee (carried, not re-implemented) ═══════
 These are facade-level API behaviour — user code WILL depend on them:

  * BLOCKING VERBS: each of moveTo/strafeTo/turnTo/followTrajectory/brake/
    hold (and curveTo/reverseCurveTo, CURVE1) is async() + waitUntilSettled() on the owned scheduler — it returns
    only when the motion exits, and CANNOT hang: the motion's own watchdog
    bounds it (C1, mutation-proven), including any boot wait; a pacer that
    stops advancing the clock trips a loud precondition (C2). The returned
    ExitReason is the motion's honest verdict (Settled / TimedOut /
    Cancelled — never Running). Deliberately NOT [[nodiscard]]: discarding
    it is a legitimate auton style (the fault latch + C5 result lines carry
    the pathology), and forcing (void) casts on every routine line would
    punish the common case.
  * WAIT-FOR-LIVE (C1): a verb issued during the boot window (estimate still
    Uninitialized) WAITS, motionless, watchdog running — a never-live
    estimate exits TimedOut rather than hanging. Budget timeouts to cover
    IMU calibration (~2 s).
  * PRE-EMPT (C2): starting a verb while a motion is active (possible via
    the Tier-3 seam or a waitUntil predicate) cancels the old motion into
    the safe state first; there is never a tick on which two motions
    command. drive() pre-empts identically — a manual command supersedes.
  * CANCEL SAFE STATE (C1/C2, HA-53): cancel() puts every drive motor at
    0 V + BrakeMode::Brake synchronously; with no active motion it is the
    PANIC STOP and still applies the safe state.
  * FAULT POLICY (C2): a fault in the configured abortFaultMask raised
    during a motion aborts it into the safe state; the verb returns
    Cancelled and lastCompleted().abortFault names the cause. Default mask:
    ODO_STUCK only (the estimate is lying). The run continues — faults log
    and recover, they never crash.
  * TURN-WHILE-DRIVE ON LIMITED-STRAFE DRIVES (C3): on an H-drive, a
    lateral-dominant leg runs authority-limited — translation proceeds at
    the achievable |vy| while vx and ω stay at full authority; rotation is
    never sequenced before translation. The mode is telemetry-visible
    (record strafeFallbackActive → TermSink " SFB"), never silent. There is
    deliberately NO polling getter for it: a live-polled bool invites
    control-flow coupling to a telemetry concept (C3 §11's recommendation,
    adopted). On tank, see TANK STEERING, next.
  * TANK STEERING (TANK1, API 2.3 — this item replaced the sentence "On
    tank, laterally-offset targets honestly exit TimedOut"): on a drive with
    NO strafe authority (strafeAuthority() exactly 0 — a tank, or an H-drive
    configured with a dead strafe wheel), moveTo keeps a leg on its LINE, the
    line through the target point along the target heading. The heading it
    servoes to is bent toward that line (0.1 rad per inch off it — counted
    where the robot will be tankSteer.leadTime = 0.06 s ahead at its heading
    and commanded speed, so the bend eases while the robot closes on the line
    instead of carrying it across — never more than
    MotionConfig::tankSteer.maxSteer = 15°, faded to 0 over the last 6 in),
    and the yaw the robot makes that the heading loop did not ask for — a
    weaker side's pull — is measured and cancelled (at most 1.5 rad/s). A leg
    that arrives with the steering off arrives with it on (host test plant,
    2026-09-28: a 24 563-leg grid, motor lags to ~0.18 s; at ~0.24 s a few
    legs starting 10–14° off their heading do not — docs/changelog.md, 2.3).
    It never aims at the target POINT and never plans a turn: a leg that
    STARTS more than maxSteer off its target heading is not steered at all —
    it runs exactly as before TANK1 and logs ONE Warn ("... -- turn first:
    face()"). Telemetry-visible like SFB: record tankSteerActive → TermSink
    " TSTR". What still exits TimedOut on a tank: a point beside the robot
    on its own heading (nothing to steer along); an offset larger than a 15°
    bend removes over the leg — no size is promised, it depends on the leg's
    length and the drivetrain's motor lag (host test plant, 2026-09-28);
    every strafeTo (holding its heading is its contract). A PathRunner leg
    with HeadingMode::Free targets the heading the robot has at leg start,
    so it steers toward an off-line point by at most 15°.
    `tankSteer.maxSteer = 0` restores the pre-TANK1 tank's motion exactly.
    Nothing of it has run on a robot. The curve verbs (below) steer toward a
    point ON their own path — the only verbs that do (C1 D12's dated note).

 ═══ drive(ChassisSpeeds, Frame) — the frame-explicit manual verb ═════════════════
 The verb a driver-control loop calls every iteration (field-centric or
 body-frame driving), and the escape hatch for direct velocity control in
 auton. The Frame parameter has NO default: the caller must say which frame
 the command is in, so silent frame confusion — the classic bug class this
 rebuild exists to prevent — is a compile error.

  * It pre-empts any active motion (above), then runs ONE loop iteration it
    owns: localizer.update() FIRST (a teleop loop that never advanced the
    estimate would rot the field rotation into exactly the frame bug),
    then the shared command pipeline, health observables, one record.
    Intended use: call it at your loop cadence; between calls nothing else
    needs ticking.
  * Frame::Field during the boot window commands ZERO volts (+ one Warn per
    window): a field-relative command needs a heading, and the boot estimate
    does not have one — rotating by garbage would move the robot in a
    garbage direction. Frame::Body works during boot (no estimate needed).
  * No stall cross-check: the driver is the supervisor in teleop, and there
    is no target to cross-check against. Health observables still tick, so
    IMU_LOST / BROWNOUT / OVER_TEMP stay live. Records carry command id 0
    (no scheduled motion) — the honest attribution.

 ═══ followTrajectory — the shape is F6, the body is deliberately minimal ═════════
 Chains the waypoints as sequential MoveToPose legs through the scheduler,
 settling at each (stop-and-settle is v1's documented motion model; blending
 is a measured Frontier item). Stops at the FIRST non-Settled leg and
 reports it — a robot that timed out mid-trajectory is lost, and chasing
 later waypoints on a lie compounds blindly.
     G2 BOUNDARY, stated honestly: no marker callbacks, no command ids on
 waypoints, no .vexbot ingestion, no profiled/curved segments — those are
 G2's PathRunner, built on this same scheduler's waitUntil primitive. This
 verb exists now because F6 freezes the VERB SET; a richer Trajectory type
 arrives as an ADDITIVE overload, never a reshape of this one.

 ═══ curveTo / reverseCurveTo — the curve follower (chunk CURVE1, F6-additive) ═════
 Flow THROUGH points instead of stopping at each: r.curveTo({{24_in, 12_in},
 {48_in, 24_in, 0_deg}}) drives one smooth path from where the robot is, along
 its nose, through (24, 12), ending at (48, 24) facing 0 deg. Pure pursuit on a
 smoothed path (motion/curve_follow.hpp, motion/curve_path.hpp), slowing BEFORE
 each bend; up to 16 points; a heading is ALWAYS the robot's NOSE (also on
 reverseCurveTo, whose back leads); do not list the robot's own spot.
  * STARTS along the nose. Refused before the robot moves (TimedOut, one Warn
    naming the point, MotionTimeout): a first point more than 90 deg off the
    nose ("turn first: face()" — the library still plans no turn), and a path
    that doubles back (points on top of each other, a heading pointing back).
  * ENDS as an ordinary moveTo: the last inches are a MoveToPose with TANK1's
    steering, judged by the same arrival check and the same per-call
    tolerances. A curve stops at its end; rolling into the next step is not
    offered.
  * STOPS a curve it has lost: 60 deg off the path for 0.25 s (checked until
    the end's hand-off, the last 12 in included), or under 1 in of progress
    in 0.75 s (not in the last 12 in, where the end is slow on purpose). From
    the hand-off on, only the timeout bounds it. The timeout covers the WHOLE
    curve and always names CurveTo.
  * C1's D12 (the library never aims at a point): by the team lead's ruling
    (2026-10-30) these two verbs, and only they, aim at a point a look-ahead
    ahead ON THE PATH THE AUTHOR DREW (or its end's line extended) — pure
    pursuit. Every other verb is unchanged, bit for bit. Proven on the host
    test plant only; the knobs are MotionConfig::curve (PROVISIONAL, HA-246).

 ═══ Options (per-call, additive-extensible) ══════════════════════════════════════
 MotionOptions carries the per-call knobs every real auton needs (a slow
 precise approach leg is table stakes). 0 means "use the config default".
 An options STRUCT (not positional parameters) is deliberate: post-freeze,
 new knobs are added fields — no signature change, no migration. STEP1-A
 (2026-11-05) took that path: positionTolerance / headingTolerance, the
 per-call ARRIVAL TOLERANCE, which loosens the settle's distance AND its speed
 bound in proportion (motion::loosenedSettle — the speed bound is what ends a
 drive leg, so a distance alone would change nothing).

 ═══ The leg OBSERVER (chunk STEP1-B, 2026-11-11; F6-additive) ═══════════════════════
 moveTo / turnTo each have a third-argument overload taking a LegObserver: a
 function pointer and a context (no heap — runner::Handler's pattern) called
 ONCE after every tick of the leg that did NOT end it, with the leg's progress
 (motion/leg_progress.hpp: the distance along the straight line from its live
 start, or the angle turned — the STEP1 brief's D7 (a)). It is how a Routine's
 partway trigger ("at 60 % of this drive, start the lift") watches a drive, and
 the Tier-3 spelling of the same thing (a recipe is never the only way). The
 overload builds EXACTLY the motion the plain verb builds and runs it inside a
 private decorator IMotion that forwards every member — name() included, so the
 records and lastCompleted() say "MoveToPose" / "TurnTo" as ever — through the
 SAME runBlocking (waitUntilSettled: no trailing pace after the exit tick; a
 waitUntil-built observer paces once more and is not bit-identical — the brief's
 probe P5). An observer that does nothing leaves the leg bit-identical to the
 plain verb. The observer is never called on the exit tick or after it — the
 motion's own exit tick, nor a Running tick the scheduler's fault policy ends
 (an abort-mask fault raised since the leg started: MotionScheduler::
 pendingAbortFault(), the scheduler's own test) — and is called with `live`
 false through the boot wait. It runs INSIDE the scheduler's
 tick: it must not call a blocking verb, async() or cancel() (the scheduler's
 own preconditions refuse them there), and it must not throw — a
 PreconditionError out of it is the scheduler's task-boundary catch (the leg
 ends Cancelled with FaultCode::Precondition), so an observer that can fail
 catches its own (the Routine's trigger does: chassis/routine.hpp, B7).

 ═══ The MIRRORED RUN (chunk MIRROR1-A, 2026-11-13; F6-additive) ════════════════════════
 One route, two robots: robot two runs robot one's auton reflected across the field's mirror line
 (Override's skills diagonal, y = -x — competition/override_mirror.hpp). A chassis::MirroredRun
 (below; the auton runner opens one for a mirrored entry, around the routine AND its end action)
 gives this Chassis a math::Reflection for exactly its lifetime, and while it is not the identity:
  * every FIELD number that ENTERS is reflected FIRST — moveTo, strafeTo, turnTo, both STEP1-B
    observer overloads, followTrajectory (each leg as it starts), curveTo / reverseCurveTo (every
    point and every stated heading, into a 16-point copy on this stack), setPose, and drive() (a
    FIELD command's vector reflected and its turn rate negated; a BODY command's sideways speed and
    turn rate negated) — so the motions, TANK1's steering, the curve's checks and the observers'
    progress all see the REAL field;
  * pose() hands the estimate back reflected — a reflection is its own inverse — so the route
    always reads robot one's numbers: a waitFor on the pose, a bearing a face() computes, a raw
    `ctx.chassis.moveTo` in a then();
  * everything else is the REAL field and is NOT reflected: lastCompleted()'s poses, deps() (the
    localizer: the wall reset and the Toggle step read it — they swap their wall, sensor side and
    Toggle themselves), scheduler(), the records and every log line. "Below the mirror".
  * Sizes are not field numbers and pass untouched: MotionOptions (timeouts, speeds, tolerances),
    hold/brake/wait, and a trigger's percent, inches and degrees (the observer's progress is a
    distance or an angle turned — a reflection keeps both).
  * Every turn sweeps the OTHER way: that is the mirror working. The one exception is F3's exact
    +-180 deg tie (math/angle.hpp: an exact antipode resolves counterclockwise) — a mirrored turnTo,
    moveTo or trajectory leg (a Routine's driveTo / reverseTo is a moveTo) whose heading change at
    its start is within 1 deg of 180 may sweep the SAME way on both robots, on ANY drivetrain. F3 is
    LOCKED and unchanged (the MIRROR1 brief's D6); such a leg logs ONE [MIR] Warn when it starts.
    Measured on the host test plant: a turn's path differs from the mirror by 0.155 in (its end by
    0.09 in); an X-drive moveTo's path by 0.307 in; a TANK moveTo's END by an amount that GROWS
    WITH THE LEG — 5.7 in on a 16 in leg, 14.2 in on a 36 in one: a tank translates only along its
    heading, so it ends beside its target, and robot two beside its own on the side opposite robot
    one's image — the two misses add (and that leg is already TANK1's unsteered one). The advice: a
    turn, two 90 deg turns; a leg, turn first (in two 90 deg turns) where it matters.
  * The robot is NOT mirrored: a part off its centre line meets the field on the other side of the
    reflected path, and its left/right drive differences are its own (A4: HA-259, HA-260 — host test
    plant, a 9 % weak side: 0.56 in, 0.44 s).
 With the identity (always, outside a MirroredRun) every verb takes its pre-MIRROR1 path — a branch,
 never "apply the identity" (re-wrapping a heading can move a bit): an unmirrored run is
 bit-identical to the merge base (test/mirror_unchanged_runs.hpp, pinned). Opening a run while one
 is open, or while a motion is active (its target would be reflected mid-flight), is a precondition.
 The driver-control loop CLEARS a mirror it finds still on, with a loud [MIR] Warn (a library
 defect: the scope is RAII) — never a refusal, which would leave the driver with no drive.

 Single-task by contract, like everything it composes. Not copyable/movable
 (it owns the scheduler, which is pinned by its self-referential stamp).
```

</details>
