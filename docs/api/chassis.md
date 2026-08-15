<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/chassis/chassis.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `chassis.hpp`

Chassis — the public facade every auton is written against.

This header declares **4** types (36 members).

Extracted from [`include/shulib/chassis/chassis.hpp`](../../include/shulib/chassis/chassis.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`struct ChassisConfig`](#struct-chassisconfig)
  - [`motion`](#chassisconfig-motion)
  - [`scheduler`](#chassisconfig-scheduler)
- [`struct MotionOptions`](#struct-motionoptions)
  - [`timeout`](#motionoptions-timeout)
  - [`maxLinearSpeed`](#motionoptions-maxlinearspeed)
  - [`maxAngularSpeed`](#motionoptions-maxangularspeed)
  - [`validate`](#motionoptions-validate)
- [`struct TrajectoryResult`](#struct-trajectoryresult)
  - [`exit`](#trajectoryresult-exit)
  - [`completedLegs`](#trajectoryresult-completedlegs)
  - [`totalLegs`](#trajectoryresult-totallegs)
  - [`succeeded`](#trajectoryresult-succeeded)
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
  - [`followTrajectory`](#chassis-followtrajectory)
  - [`followTrajectory (overload 2)`](#chassis-followtrajectory-2)
  - [`brake`](#chassis-brake)
  - [`hold`](#chassis-hold)
  - [`wait`](#chassis-wait)
  - [`drive`](#chassis-drive)
  - [`cancel`](#chassis-cancel)
  - [`waitUntil`](#chassis-waituntil)
  - [`pose`](#chassis-pose)
  - [`setPose`](#chassis-setpose)
  - [`strafeAuthority`](#chassis-strafeauthority)
  - [`lastExitReason`](#chassis-lastexitreason)
  - [`lastCompleted`](#chassis-lastcompleted)
  - [`motionConfig`](#chassis-motionconfig)
  - [`deps`](#chassis-deps)
  - [`scheduler`](#chassis-scheduler)
  - [`scheduler (overload 2)`](#chassis-scheduler-2)

<a id="struct-chassisconfig"></a>

## `struct ChassisConfig`

```cpp
struct ChassisConfig
```

Everything configurable about a Chassis, in one place. Both members are the lower layers' own config types passed through WHOLE — so an additive field there (e.g. a future per-wheel speed budget in MotionConfig, the C3 §11 flag) flows through this surface with no reshape.

*struct, declared at [`include/shulib/chassis/chassis.hpp:173`](../../include/shulib/chassis/chassis.hpp#L173).*

<a id="chassisconfig-motion"></a>

### `ChassisConfig::motion`

```cpp
motion::MotionConfig motion{}
```

gains/budgets/tolerances (HA-50/51/52)

*field, declared at [`include/shulib/chassis/chassis.hpp:174`](../../include/shulib/chassis/chassis.hpp#L174).*

<a id="chassisconfig-scheduler"></a>

### `ChassisConfig::scheduler`

```cpp
motion::MotionSchedulerConfig scheduler{}
```

fault policy mask + loop monitor

*field, declared at [`include/shulib/chassis/chassis.hpp:175`](../../include/shulib/chassis/chassis.hpp#L175).*

<a id="struct-motionoptions"></a>

## `struct MotionOptions`

```cpp
struct MotionOptions
```

Per-call knobs for the blocking verbs. 0 (the default) = "use the ChassisConfig value". Validated finite and >= 0 at each call.  FROZEN F6 NOTE (D2): the fields BELOW are frozen (name/type/meaning); the field SET is deliberately additive-open — a future knob is a new field with a 0/"config default" meaning, never a reshape of these.

*struct, declared at [`include/shulib/chassis/chassis.hpp:184`](../../include/shulib/chassis/chassis.hpp#L184).*

<a id="motionoptions-timeout"></a>

### `MotionOptions::timeout`

```cpp
units::Time timeout{0.0}
```

Watchdog bound for this motion, INCLUDING any boot wait. Typed time (D2): `{.timeout = 5_s}` / `{.timeout = 500_ms}` — a bare double does not compile, so "500 meaning milliseconds" cannot silently become 500 seconds of match time.

*field, declared at [`include/shulib/chassis/chassis.hpp:189`](../../include/shulib/chassis/chassis.hpp#L189).*

<a id="motionoptions-maxlinearspeed"></a>

### `MotionOptions::maxLinearSpeed`

```cpp
units::Velocity maxLinearSpeed{0.0}
```

Field-frame linear speed budget for this motion (in/s) — the norm cap AND the base of the strafe-authority clamp, exactly as in MotionConfig. The per-wheel budget (maxWheelSpeed) is deliberately NOT scaled with it: that is a hardware envelope, not a per-leg intent.

*field, declared at [`include/shulib/chassis/chassis.hpp:194`](../../include/shulib/chassis/chassis.hpp#L194).*

<a id="motionoptions-maxangularspeed"></a>

### `MotionOptions::maxAngularSpeed`

```cpp
units::AngularVelocity maxAngularSpeed{0.0}
```

Yaw-rate budget for this motion (rad/s).

*field, declared at [`include/shulib/chassis/chassis.hpp:196`](../../include/shulib/chassis/chassis.hpp#L196).*

<a id="motionoptions-validate"></a>

### `MotionOptions::validate`

```cpp
void validate() const
```

Reject nonsense before anything moves: every field must be finite and >= 0. Called by each verb at the door, so a bad option value is a loud error at the call site rather than a mystery mid-motion.

*function, declared at [`include/shulib/chassis/chassis.hpp:201`](../../include/shulib/chassis/chassis.hpp#L201).*

<a id="struct-trajectoryresult"></a>

## `struct TrajectoryResult`

```cpp
struct TrajectoryResult
```

What followTrajectory did — which leg count it completed and how the last attempted leg exited. (ExitReason alone would lose WHERE the chain broke; the next thing a routine does after a failed trajectory legitimately depends on how far it got.)

*struct, declared at [`include/shulib/chassis/chassis.hpp:217`](../../include/shulib/chassis/chassis.hpp#L217).*

<a id="trajectoryresult-exit"></a>

### `TrajectoryResult::exit`

```cpp
control::ExitReason exit = control::ExitReason::Settled
```

last attempted leg's verdict

*field, declared at [`include/shulib/chassis/chassis.hpp:218`](../../include/shulib/chassis/chassis.hpp#L218).*

<a id="trajectoryresult-completedlegs"></a>

### `TrajectoryResult::completedLegs`

```cpp
int completedLegs = 0
```

legs that SETTLED (== totalLegs on success)

*field, declared at [`include/shulib/chassis/chassis.hpp:219`](../../include/shulib/chassis/chassis.hpp#L219).*

<a id="trajectoryresult-totallegs"></a>

### `TrajectoryResult::totalLegs`

```cpp
int totalLegs = 0
```

waypoints given

*field, declared at [`include/shulib/chassis/chassis.hpp:220`](../../include/shulib/chassis/chassis.hpp#L220).*

<a id="trajectoryresult-succeeded"></a>

### `TrajectoryResult::succeeded`

```cpp
[[nodiscard]] bool succeeded() const noexcept
```

True only if the last attempted leg SETTLED and every leg was completed. Note what this means for a value-initialized TrajectoryResult (0 of 0 legs, exit Settled): it reads as success. That is correct here — this verb requires at least one waypoint, so a result it produces always has legs — but any code that holds a TrajectoryResult BEFORE running one must initialize `exit` to Running instead (Routine::lastTrajectory does).

*function, declared at [`include/shulib/chassis/chassis.hpp:227`](../../include/shulib/chassis/chassis.hpp#L227).*

<a id="class-chassis"></a>

## `class Chassis`

```cpp
class Chassis
```

The public facade every autonomous routine is written against: the blocking motion verbs, the frame-explicit manual verb, control, state, and the Tier-3 seam — over one owned MotionScheduler. FROZEN (register row F6, locked 2026-08-12); the file banner above carries the design reasoning behind every shape here, and is meant to be read before changing anything.

*class, declared at [`include/shulib/chassis/chassis.hpp:237`](../../include/shulib/chassis/chassis.hpp#L237).*

<a id="chassis-chassis"></a>

### `Chassis::Chassis`

```cpp
explicit Chassis(const motion::MotionDeps& deps, motion::ITickPacer& pacer, const ChassisConfig& config = {})
```

`deps` is the same validated bundle every motion takes; `pacer` is the seam through which the world advances during blocking verbs (host sim: step the plant; robot: delay to the tick boundary — R1/R3 build that one). All deps pointees AND the pacer must outlive the Chassis; the facade borrows, it does not own (header: construction).

*function, declared at [`include/shulib/chassis/chassis.hpp:244`](../../include/shulib/chassis/chassis.hpp#L244).*

<a id="chassis-chassis-2"></a>

### `Chassis::Chassis (overload 2)`

```cpp
Chassis(const Chassis&) = delete
```

Neither copyable nor movable: the Chassis OWNS the scheduler, which is pinned in place by its own self-referential command-id stamp, so a copy or a move would leave that stamp pointing at the wrong object. Hold a `Chassis&`; construct it once, where it will live.

*function, declared at [`include/shulib/chassis/chassis.hpp:254`](../../include/shulib/chassis/chassis.hpp#L254).*

<a id="chassis-chassis-3"></a>

### `Chassis::Chassis (overload 3)`

```cpp
Chassis(Chassis&&) = delete
```

*Covered by the comment on [`Chassis (overload 2)`](#chassis-chassis-2) — one comment documents this run of special members.*

*function, declared at [`include/shulib/chassis/chassis.hpp:255`](../../include/shulib/chassis/chassis.hpp#L255).*

<a id="chassis-operator-eq"></a>

### `Chassis::operator=`

```cpp
Chassis& operator=(const Chassis&) = delete
```

*Covered by the comment on [`Chassis (overload 2)`](#chassis-chassis-2) — one comment documents this run of special members.*

*function, declared at [`include/shulib/chassis/chassis.hpp:256`](../../include/shulib/chassis/chassis.hpp#L256).*

<a id="chassis-operator-eq-2"></a>

### `Chassis::operator= (overload 2)`

```cpp
Chassis& operator=(Chassis&&) = delete
```

*Covered by the comment on [`Chassis (overload 2)`](#chassis-chassis-2) — one comment documents this run of special members.*

*function, declared at [`include/shulib/chassis/chassis.hpp:257`](../../include/shulib/chassis/chassis.hpp#L257).*

<a id="chassis-destructor-chassis"></a>

### `Chassis::~Chassis`

```cpp
~Chassis() = default
```

*Covered by the comment on [`Chassis (overload 2)`](#chassis-chassis-2) — one comment documents this run of special members.*

*function, declared at [`include/shulib/chassis/chassis.hpp:258`](../../include/shulib/chassis/chassis.hpp#L258).*

<a id="chassis-moveto"></a>

### `Chassis::moveTo`

```cpp
control::ExitReason moveTo(const math::Pose2d& target, const MotionOptions& options = {})
```

Drive to `target` (FIELD pose): the decoupled holonomic engine — translation and rotation simultaneous and independent (C1's thesis).

*function, declared at [`include/shulib/chassis/chassis.hpp:264`](../../include/shulib/chassis/chassis.hpp#L264).*

<a id="chassis-strafeto"></a>

### `Chassis::strafeTo`

```cpp
control::ExitReason strafeTo(units::Length x, units::Length y, const MotionOptions& options = {})
```

Translate to FIELD (x, y) while actively HOLDING the heading the robot has at its first live tick. On tank (authority 0) an off-line target honestly exits TimedOut (C1's drivetrain honesty).

*function, declared at [`include/shulib/chassis/chassis.hpp:274`](../../include/shulib/chassis/chassis.hpp#L274).*

<a id="chassis-turnto"></a>

### `Chassis::turnTo`

```cpp
control::ExitReason turnTo(math::Angle heading, const MotionOptions& options = {})
```

Rotate in place to a FIELD heading, always the short way (F3's shortest signed error; exact ±180° resolves CCW, deterministically).

*function, declared at [`include/shulib/chassis/chassis.hpp:284`](../../include/shulib/chassis/chassis.hpp#L284).*

<a id="chassis-followtrajectory"></a>

### `Chassis::followTrajectory`

```cpp
TrajectoryResult followTrajectory(std::span<const math::Pose2d> waypoints, const MotionOptions& options = {})
```

Chain `waypoints` as sequential moveTo legs, settling at each; stop at the first non-Settled leg (header: followTrajectory). `options` apply PER LEG (each leg is one scheduled motion with its own watchdog). Precondition: at least one waypoint. G2 boundary in the header.

*function, declared at [`include/shulib/chassis/chassis.hpp:295`](../../include/shulib/chassis/chassis.hpp#L295).*

<a id="chassis-followtrajectory-2"></a>

### `Chassis::followTrajectory (overload 2)`

```cpp
TrajectoryResult followTrajectory(std::initializer_list<math::Pose2d> waypoints, const MotionOptions& options = {})
```

Brace-list convenience: followTrajectory({a, b, c}).

*function, declared at [`include/shulib/chassis/chassis.hpp:324`](../../include/shulib/chassis/chassis.hpp#L324).*

<a id="chassis-brake"></a>

### `Chassis::brake`

```cpp
control::ExitReason brake(const MotionOptions& options = {})
```

Stop the drivetrain (0 V under Brake) and block until the ESTIMATE certifies rest (or the watchdog fires). The controlled end-of-motion stop; cancel() is the uncontrolled one.

*function, declared at [`include/shulib/chassis/chassis.hpp:336`](../../include/shulib/chassis/chassis.hpp#L336).*

<a id="chassis-hold"></a>

### `Chassis::hold`

```cpp
control::ExitReason hold(units::Time duration, const MotionOptions& options = {})
```

Actively hold the pose the robot has at its first live tick for `duration`, driving back any disturbance with full holonomic authority; Settled iff still within tolerance when the window ends. `duration` must be finite and > 0 (HoldPose's precondition). Typed time (D2): hold(500_ms) — hold(500) does not compile, so "500 meaning milliseconds" cannot hold pose for 500 s of a 15 s auton.

*function, declared at [`include/shulib/chassis/chassis.hpp:348`](../../include/shulib/chassis/chassis.hpp#L348).*

<a id="chassis-wait"></a>

### `Chassis::wait`

```cpp
void wait(units::Time duration)
```

Wait, commanding nothing, for `duration` — then return. The world keeps advancing and the active motion (if any) keeps ticking — the same contract as waitUntil; the drive keeps whatever state the last verb left it in (after a settled motion: stopped). Deliberately DISTINCT from hold(): wait() never energizes the drive — this is the "sit still for the alliance partner" beat (D2; adopted from D1's finding that the naive waitUntil(false-pred, t) spelling logs a spurious Warn on every deliberate pause, and the Warn-free spelling needed Tier-3 plumbing). Returns void: a wait has no failure mode — a pacer that stops advancing the clock trips the scheduler's loud precondition, a programming error rather than a verdict. Warn-free and bounded by construction: the deadline predicate is time-monotone, so the internal timeout backstop is unreachable slack. `duration` must be finite and > 0 (typed: wait(2_s) / wait(500_ms)).

*function, declared at [`include/shulib/chassis/chassis.hpp:368`](../../include/shulib/chassis/chassis.hpp#L368).*

<a id="chassis-drive"></a>

### `Chassis::drive`

```cpp
void drive(const math::ChassisSpeeds& speeds, math::Frame frame)
```

Command a chassis velocity directly, in the frame the CALLER names (no default — header: drive). Pre-empts any active motion; owns one loop iteration (estimate update → shared pipeline → health → record). Precondition: all three components finite.

*function, declared at [`include/shulib/chassis/chassis.hpp:385`](../../include/shulib/chassis/chassis.hpp#L385).*

<a id="chassis-cancel"></a>

### `Chassis::cancel`

```cpp
void cancel()
```

Stop the active motion into the defined safe state (0 V + Brake); with no active motion this is the PANIC STOP and still safes the drive.

*function, declared at [`include/shulib/chassis/chassis.hpp:429`](../../include/shulib/chassis/chassis.hpp#L429).*

<a id="chassis-waituntil"></a>

### `Chassis::waitUntil`

```cpp
template <typename Pred> [[nodiscard]] motion::WaitResult waitUntil(Pred&& pred, units::Time timeout)
```

Block until `pred()` holds or `timeout` elapses (required, finite, >= 0; 0 = an honest poll) — the return says which. The active motion (if any) keeps ticking throughout; the world keeps advancing. Timing out logs one Warn and raises NO fault (a timed-out wait is a strategy branch, not a pathology). C2's verb, re-exported with typed time at the public edge (D2); the scheduler's own seconds-double signature is interior, per F3's internal-seconds convention.

*function, declared at [`include/shulib/chassis/chassis.hpp:439`](../../include/shulib/chassis/chassis.hpp#L439).*

<a id="chassis-pose"></a>

### `Chassis::pose`

```cpp
[[nodiscard]] math::Pose2d pose() const
```

The current fused FIELD pose estimate.

*function, declared at [`include/shulib/chassis/chassis.hpp:446`](../../include/shulib/chassis/chassis.hpp#L446).*

<a id="chassis-setpose"></a>

### `Chassis::setPose`

```cpp
void setPose(const math::Pose2d& pose)
```

Seed / teleport the estimated POSITION (x, y) — heading stays IMU-owned (the Localizer's structural choice). Call at auton start with the measured starting pose.

*function, declared at [`include/shulib/chassis/chassis.hpp:451`](../../include/shulib/chassis/chassis.hpp#L451).*

<a id="chassis-strafeauthority"></a>

### `Chassis::strafeAuthority`

```cpp
[[nodiscard]] double strafeAuthority() const
```

Read-only passthrough of the drivetrain's sustainable lateral authority (fraction of the linear budget; F5). Routine authors budgeting lateral legs legitimately want it — the difference between a 2 s and a 3 s leg on the H-bot (C3 §11 #2, adopted).

*function, declared at [`include/shulib/chassis/chassis.hpp:457`](../../include/shulib/chassis/chassis.hpp#L457).*

<a id="chassis-lastexitreason"></a>

### `Chassis::lastExitReason`

```cpp
[[nodiscard]] control::ExitReason lastExitReason() const noexcept
```

Exit reason of the most recently finished motion (Settled on a virgin chassis — completedCount() via scheduler() says whether anything ran).

*function, declared at [`include/shulib/chassis/chassis.hpp:463`](../../include/shulib/chassis/chassis.hpp#L463).*

<a id="chassis-lastcompleted"></a>

### `Chassis::lastCompleted`

```cpp
[[nodiscard]] const motion::CompletedMotion& lastCompleted() const noexcept
```

The most recent motion boundary — id/name/exit/abortFault/times (C5's raw material; abortFault names a fault-policy cause).

*function, declared at [`include/shulib/chassis/chassis.hpp:469`](../../include/shulib/chassis/chassis.hpp#L469).*

<a id="chassis-motionconfig"></a>

### `Chassis::motionConfig`

```cpp
[[nodiscard]] const motion::MotionConfig& motionConfig() const noexcept
```

The config the verbs run under (per-call options override per motion).

*function, declared at [`include/shulib/chassis/chassis.hpp:474`](../../include/shulib/chassis/chassis.hpp#L474).*

<a id="chassis-deps"></a>

### `Chassis::deps`

```cpp
[[nodiscard]] const motion::MotionDeps& deps() const noexcept
```

The STAMPED deps bundle — build custom IMotions from THIS and their records carry command ids like the built-in verbs' do.

*function, declared at [`include/shulib/chassis/chassis.hpp:480`](../../include/shulib/chassis/chassis.hpp#L480).*

<a id="chassis-scheduler"></a>

### `Chassis::scheduler`

```cpp
[[nodiscard]] motion::MotionScheduler& scheduler() noexcept
```

The owned scheduler, for async composition / caller-paced tick() / counters. It is the SAME single motion slot the verbs use: async() here pre-empts a facade verb's motion and vice versa (one-active- motion is structural, never relaxed).

*function, declared at [`include/shulib/chassis/chassis.hpp:486`](../../include/shulib/chassis/chassis.hpp#L486).*

<a id="chassis-scheduler-2"></a>

### `Chassis::scheduler (overload 2)`

```cpp
[[nodiscard]] const motion::MotionScheduler& scheduler() const noexcept
```

The same scheduler, read-only — for counters and last-motion state from a `const Chassis&`. Identical object and identical semantics to the non-const overload; the two differ only in what they let you do.

*function, declared at [`include/shulib/chassis/chassis.hpp:490`](../../include/shulib/chassis/chassis.hpp#L490).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 140 lines, click to expand</summary>

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
   control    cancel (panic stop) · waitUntil(pred, timeout)
   state      pose · setPose · strafeAuthority · lastExitReason · lastCompleted
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
    hold is async() + waitUntilSettled() on the owned scheduler — it returns
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
    adopted). On tank, laterally-offset targets honestly exit TimedOut.

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

 ═══ Options (per-call, additive-extensible) ══════════════════════════════════════
 MotionOptions carries the per-call knobs every real auton needs (a slow
 precise approach leg is table stakes). 0 means "use the config default".
 An options STRUCT (not positional parameters) is deliberate: post-freeze,
 new knobs are added fields — no signature change, no migration.

 Single-task by contract, like everything it composes. Not copyable/movable
 (it owns the scheduler, which is pinned by its self-referential stamp).
```

</details>
