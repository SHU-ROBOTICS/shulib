<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/motion/motion.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `motion.hpp`

IMotion — the contract every motion primitive implements.

This header declares **3** types (25 members) and **2** free functions.

Extracted from [`include/shulib/motion/motion.hpp`](../../include/shulib/motion/motion.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`enum class MotionState`](#enum-class-motionstate)
  - [`Idle`](#motionstate-idle)
  - [`WaitingForEstimate`](#motionstate-waitingforestimate)
  - [`Running`](#motionstate-running)
  - [`Settled`](#motionstate-settled)
  - [`TimedOut`](#motionstate-timedout)
  - [`Cancelled`](#motionstate-cancelled)
- [`applyCancelSafeState`](#applycancelsafestate) — *free function*
- [`struct MotionDeps`](#struct-motiondeps)
  - [`ctx`](#motiondeps-ctx)
  - [`localizer`](#motiondeps-localizer)
  - [`kinematics`](#motiondeps-kinematics)
  - [`faults`](#motiondeps-faults)
  - [`health`](#motiondeps-health)
  - [`validate`](#motiondeps-validate)
  - [`validatedClock`](#motiondeps-validatedclock)
- [`tickHealthObservables`](#tickhealthobservables) — *free function*
- [`class IMotion`](#class-imotion)
  - [`~IMotion`](#imotion-destructor-imotion)
  - [`IMotion`](#imotion-imotion)
  - [`IMotion (overload 2)`](#imotion-imotion-2)
  - [`IMotion (overload 3)`](#imotion-imotion-3)
  - [`operator=`](#imotion-operator-eq)
  - [`operator= (overload 2)`](#imotion-operator-eq-2)
  - [`start`](#imotion-start)
  - [`tick`](#imotion-tick)
  - [`cancel`](#imotion-cancel)
  - [`exitReason`](#imotion-exitreason)
  - [`state`](#imotion-state)
  - [`name`](#imotion-name)

<a id="enum-class-motionstate"></a>

## `enum class MotionState`

```cpp
enum class MotionState : std::uint8_t
```

Motion-layer state, the wire vocabulary for DebugRecord.activeCommandState (§18.2 — "the VOCABULARY is owned by the motion layer; once assigned, values are wire-stable like FaultCode's"). Explicit values, append-only.

*enum class, declared at [`include/shulib/motion/motion.hpp:144`](../../include/shulib/motion/motion.hpp#L144).*

<a id="motionstate-idle"></a>

### `MotionState::Idle`

```cpp
Idle = 0
```

constructed / reset; start() not yet called

*enumerator, declared at [`include/shulib/motion/motion.hpp:145`](../../include/shulib/motion/motion.hpp#L145).*

<a id="motionstate-waitingforestimate"></a>

### `MotionState::WaitingForEstimate`

```cpp
WaitingForEstimate = 1
```

started, but qualityClass() is still Uninitialized

*enumerator, declared at [`include/shulib/motion/motion.hpp:146`](../../include/shulib/motion/motion.hpp#L146).*

<a id="motionstate-running"></a>

### `MotionState::Running`

```cpp
Running = 2
```

actively controlling toward the target

*enumerator, declared at [`include/shulib/motion/motion.hpp:147`](../../include/shulib/motion/motion.hpp#L147).*

<a id="motionstate-settled"></a>

### `MotionState::Settled`

```cpp
Settled = 3
```

exited: arrived within tolerances

*enumerator, declared at [`include/shulib/motion/motion.hpp:148`](../../include/shulib/motion/motion.hpp#L148).*

<a id="motionstate-timedout"></a>

### `MotionState::TimedOut`

```cpp
TimedOut = 4
```

exited: watchdog fired (MOTION_TIMEOUT raised)

*enumerator, declared at [`include/shulib/motion/motion.hpp:149`](../../include/shulib/motion/motion.hpp#L149).*

<a id="motionstate-cancelled"></a>

### `MotionState::Cancelled`

```cpp
Cancelled = 5
```

exited: cancel() — stopped from outside (APPENDED at chunk C2 per the append-only rule; wire-stable)

*enumerator, declared at [`include/shulib/motion/motion.hpp:150`](../../include/shulib/motion/motion.hpp#L150).*

<a id="applycancelsafestate"></a>

## `applyCancelSafeState`

```cpp
inline void applyCancelSafeState(chassis::RobotContext& ctx)
```

The CANCEL SAFE STATE, defined in ONE place so every cancel path — each primitive's cancel(), the scheduler's pre-emption, its fault-policy abort, and its no-active-motion panic stop — commands the identical thing: zero volts under BrakeMode::Brake on every drive motor (rationale in the cancel contract above). Brake mode is set BEFORE the zero-volt command so the stop lands under braking semantics, never a momentary coast.  HARDWARE CLAIM, honest scope: the A2 plant does not model brake modes, so host tests prove the 0 V dynamics reach rest and pin the Brake command by state inspection — how hard a real V5 drivetrain brakes from speed is unverifiable until hardware. PROVISIONAL (A4: HA-53).

*free function, declared at [`include/shulib/motion/motion.hpp:165`](../../include/shulib/motion/motion.hpp#L165).*

<a id="struct-motiondeps"></a>

## `struct MotionDeps`

```cpp
struct MotionDeps
```

The dependencies every motion shares, as NAMED pointers (designated initializers at the call site), validated non-null by validate(). All pointees must outlive the motion. This bundle is deliberately the same set the C4 Chassis facade will own — a motion is constructible from a facade's internals with no reshaping (flagged for F6).

*struct, declared at [`include/shulib/motion/motion.hpp:177`](../../include/shulib/motion/motion.hpp#L177).*

<a id="motiondeps-ctx"></a>

### `MotionDeps::ctx`

```cpp
chassis::RobotContext* ctx = nullptr
```

clock, motors, imu, battery, telemetry

*field, declared at [`include/shulib/motion/motion.hpp:178`](../../include/shulib/motion/motion.hpp#L178).*

<a id="motiondeps-localizer"></a>

### `MotionDeps::localizer`

```cpp
localization::Localizer* localizer = nullptr
```

the fused estimate + categorical quality

*field, declared at [`include/shulib/motion/motion.hpp:179`](../../include/shulib/motion/motion.hpp#L179).*

<a id="motiondeps-kinematics"></a>

### `MotionDeps::kinematics`

```cpp
const kinematics::IKinematics* kinematics = nullptr
```

the F5 drivetrain contract

*field, declared at [`include/shulib/motion/motion.hpp:180`](../../include/shulib/motion/motion.hpp#L180).*

<a id="motiondeps-faults"></a>

### `MotionDeps::faults`

```cpp
diag::FaultLatch* faults = nullptr
```

run-scoped latch (MotionTimeout, …)

*field, declared at [`include/shulib/motion/motion.hpp:181`](../../include/shulib/motion/motion.hpp#L181).*

<a id="motiondeps-health"></a>

### `MotionDeps::health`

```cpp
diag::HealthMonitor* health = nullptr
```

the A3 pathology→fault policy

*field, declared at [`include/shulib/motion/motion.hpp:182`](../../include/shulib/motion/motion.hpp#L182).*

<a id="motiondeps-validate"></a>

### `MotionDeps::validate`

```cpp
void validate() const
```

Trip SHULIB_PRECONDITION on the FIRST null pointer, naming which one. Every motion calls this from its constructor (through validatedClock()), so a dependency the designated-initializer call site forgot is a loud contract breach at construction rather than a null dereference three ticks into an auton.

*function, declared at [`include/shulib/motion/motion.hpp:188`](../../include/shulib/motion/motion.hpp#L188).*

<a id="motiondeps-validatedclock"></a>

### `MotionDeps::validatedClock`

```cpp
[[nodiscard]] hal::IClock& validatedClock() const
```

validate(), then hand out the clock — for a member-initializer list's FIRST dependency use, so a null pointer trips the precondition rather than being dereferenced.

*function, declared at [`include/shulib/motion/motion.hpp:209`](../../include/shulib/motion/motion.hpp#L209).*

<a id="tickhealthobservables"></a>

## `tickHealthObservables`

```cpp
inline void tickHealthObservables(const MotionDeps& deps, bool odomStalled)
```

Tick the shared HealthMonitor with every observable reachable from the deps — the A3 containment wiring in ONE place (chunk C4; three copies had grown by then: MoveToPose, TurnTo, and the scheduler's idle tick, and the facade's drive() would have been a fourth). `odomStalled` stays a parameter because it is the one observable with a per-caller story: the active motion feeds its OdoStallCheck verdict; idle/teleop callers pass false — nothing (or nothing closed-loop) is commanded, so there is no spin to cross-check (the DriveBrake-exemption reasoning).

*free function, declared at [`include/shulib/motion/motion.hpp:223`](../../include/shulib/motion/motion.hpp#L223).*

<a id="class-imotion"></a>

## `class IMotion`

```cpp
class IMotion
```

The contract every motion primitive implements: one target, one tick() that reads the world and issues ONE drivetrain command, one verdict. A motion owns no loop, no task and no estimator — the loop owner advances the Localizer first, then calls tick() (the tick contract above). Implementers owe the whole of it, not just the signatures: an exit leaves the motors stopped and every later tick() is a no-op returning the cached verdict, start() fully re-arms a finished object, and cancel() works at any time and is idempotent. No motion may hang — the watchdog runs even while waiting for a live estimate.

*class, declared at [`include/shulib/motion/motion.hpp:246`](../../include/shulib/motion/motion.hpp#L246).*

<a id="imotion-destructor-imotion"></a>

### `IMotion::~IMotion`

```cpp
virtual ~IMotion() = default
```

Interface plumbing, spelled out because declaring the destructor demands all six: motions are held and destroyed through this base, and copy/move are defaulted because IMotion itself holds no state — every motion's state is in the concrete type, which is also why the scheduler passes motions by pointer, not by value.

*function, declared at [`include/shulib/motion/motion.hpp:252`](../../include/shulib/motion/motion.hpp#L252).*

<a id="imotion-imotion"></a>

### `IMotion::IMotion`

```cpp
IMotion() = default
```

*Covered by the comment on [`~IMotion`](#imotion-destructor-imotion) — one comment documents this run of special members.*

*function, declared at [`include/shulib/motion/motion.hpp:253`](../../include/shulib/motion/motion.hpp#L253).*

<a id="imotion-imotion-2"></a>

### `IMotion::IMotion (overload 2)`

```cpp
IMotion(const IMotion&) = default
```

*Covered by the comment on [`~IMotion`](#imotion-destructor-imotion) — one comment documents this run of special members.*

*function, declared at [`include/shulib/motion/motion.hpp:254`](../../include/shulib/motion/motion.hpp#L254).*

<a id="imotion-imotion-3"></a>

### `IMotion::IMotion (overload 3)`

```cpp
IMotion(IMotion&&) = default
```

*Covered by the comment on [`~IMotion`](#imotion-destructor-imotion) — one comment documents this run of special members.*

*function, declared at [`include/shulib/motion/motion.hpp:255`](../../include/shulib/motion/motion.hpp#L255).*

<a id="imotion-operator-eq"></a>

### `IMotion::operator=`

```cpp
IMotion& operator=(const IMotion&) = default
```

*Covered by the comment on [`~IMotion`](#imotion-destructor-imotion) — one comment documents this run of special members.*

*function, declared at [`include/shulib/motion/motion.hpp:256`](../../include/shulib/motion/motion.hpp#L256).*

<a id="imotion-operator-eq-2"></a>

### `IMotion::operator= (overload 2)`

```cpp
IMotion& operator=(IMotion&&) = default
```

*Covered by the comment on [`~IMotion`](#imotion-destructor-imotion) — one comment documents this run of special members.*

*function, declared at [`include/shulib/motion/motion.hpp:257`](../../include/shulib/motion/motion.hpp#L257).*

<a id="imotion-start"></a>

### `IMotion::start`

```cpp
virtual void start() = 0
```

Arm the motion: reset controllers/settle state, start the watchdog. Re-callable — a finished motion re-arms completely.

*function, declared at [`include/shulib/motion/motion.hpp:261`](../../include/shulib/motion/motion.hpp#L261).*

<a id="imotion-tick"></a>

### `IMotion::tick`

```cpp
[[nodiscard]] virtual control::ExitReason tick() = 0
```

One control tick (see the tick contract above). Precondition: start() has been called. The loop must update the Localizer BEFORE calling this.

*function, declared at [`include/shulib/motion/motion.hpp:265`](../../include/shulib/motion/motion.hpp#L265).*

<a id="imotion-cancel"></a>

### `IMotion::cancel`

```cpp
virtual void cancel() = 0
```

Stop the motion from outside (see the cancel contract above). PURE virtual ON PURPOSE — a motion type without a cancellation story is the forgettable-safety-step failure mode (A1's emitRecord lesson); every implementer must state one. Idempotent; never raises; applies the cancel safe state whenever the motion has been started.

*function, declared at [`include/shulib/motion/motion.hpp:272`](../../include/shulib/motion/motion.hpp#L272).*

<a id="imotion-exitreason"></a>

### `IMotion::exitReason`

```cpp
[[nodiscard]] virtual control::ExitReason exitReason() const noexcept = 0
```

The verdict of the most recent tick() (Running before the first tick).

*function, declared at [`include/shulib/motion/motion.hpp:275`](../../include/shulib/motion/motion.hpp#L275).*

<a id="imotion-state"></a>

### `IMotion::state`

```cpp
[[nodiscard]] virtual MotionState state() const noexcept = 0
```

The motion-layer state (the activeCommandState vocabulary).

*function, declared at [`include/shulib/motion/motion.hpp:278`](../../include/shulib/motion/motion.hpp#L278).*

<a id="imotion-name"></a>

### `IMotion::name`

```cpp
[[nodiscard]] virtual const char* name() const noexcept = 0
```

Stable short name for logs / result lines (e.g. "MoveToPose").

*function, declared at [`include/shulib/motion/motion.hpp:281`](../../include/shulib/motion/motion.hpp#L281).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 125 lines, click to expand</summary>

```text

 IMotion — the contract every motion primitive implements (chunk C1, WS6/M2).

 This is the layer that makes the library able to DRIVE: everything below it
 (control, kinematics, localization, diagnostics) is the parts of a car; a
 motion is the driver. C1 ships MoveToPose / TurnTo / StrafeTo / HoldPose /
 DriveBrake; C2's MotionScheduler runs them asynchronously; C4's Chassis facade
 wraps them in the public verbs (F6 freezes at D2 — the shapes here are what
 that facade will inherit).

 ── The tick contract (who owns the loop) ───────────────────────────────────────────
 A motion does NOT own a loop, a task, or the estimator. The loop owner (a test
 harness lambda today, C2's scheduler tomorrow) each tick:

     localizer.update();                 // the estimate advances FIRST
     const auto reason = motion.tick();  // then the motion reads it and commands

 This is the controller-first shape A2 froze into the harness (scenario.hpp
 "Loop shape") — the motion sees the world at time t, the plant/robot then
 advances to t+dt. tick() reads sensors through RobotContext + the Localizer,
 computes ONE ChassisSpeeds command, pushes it through kinematics to the motors,
 evaluates its exit criteria, and returns the verdict:

   * Running   — still working; call tick() again next loop iteration.
   * Settled   — arrived (per SettledUtil criteria); motors have been stopped.
   * TimedOut  — the watchdog fired first; motors have been stopped and
                 FaultCode::MotionTimeout raised. A motion can NEVER hang: the
                 watchdog is armed in start() and no code path disarms it.

   * Cancelled — stopped from outside via cancel() (added at C2 through the
                 documented additive path; see the cancel contract below).

 After a non-Running verdict the motion is FINISHED: further tick() calls are
 safe no-ops that return the cached verdict and leave the motors stopped.
 start() fully re-arms (a motion object is reusable, e.g. retrying a move).

 ── The cancel contract (chunk C2 — the scheduler's structural guarantee) ───────────
 cancel() is how a motion is stopped from OUTSIDE its own exit criteria: user
 abort, scheduler pre-emption (a new motion supersedes this one), or the C2
 fault policy (e.g. ODO_STUCK — the estimate is lying, and continuing to servo
 against a lie is worse than stopping). Semantics, identical in every
 primitive and pinned by test:

   * If the motion is RUNNING (incl. WaitingForEstimate — a cancel during the
     boot window is legal and common): the drivetrain is put in the CANCEL
     SAFE STATE (below) synchronously, the verdict becomes Cancelled, one
     final exit record is emitted, and the motion is FINISHED — subsequent
     tick() calls are the same safe no-ops as any other exit. This is what
     makes one-active-motion STRUCTURAL for the scheduler: a pre-empted
     motion object is inert at the object level, not merely unreferenced.
   * If the motion already EXITED (Settled / TimedOut / Cancelled): the safe
     state is still applied (cancel() means "make the drivetrain safe NOW",
     and it must be idempotent), but the verdict is PRESERVED — a motion that
     settled really did settle; rewriting history would lie to the C5 result
     line. Back-to-back cancels are therefore harmless no-ops after the first.
   * If the motion was never started (Idle): complete no-op. An unstarted
     motion has no relationship to the drivetrain yet; commanding motors from
     it would be the surprise, not the safety.
   * cancel() raises NO fault. Cancellation is a commanded, normal act; when
     the CAUSE is a fault, that fault is already latched by whoever detected
     it (the scheduler records the causal code alongside the boundary).

 The CANCEL SAFE STATE is defined once, in applyCancelSafeState() below:
 zero volts + BrakeMode::Brake on every drive motor. Why brake and not the
 alternatives: COAST lets a robot at speed keep rolling (a cancel that leaves
 the robot coasting into a wall fails the whole point); a closed-loop HOLD
 keeps servoing against the estimate — and the highest-priority cancel cause
 (ODO_STUCK) is precisely "the estimate is lying", so holding would reproduce
 the failure cancel exists to stop. Brake is estimate-independent (valid
 during the boot window, valid with dead encoders), passive, and immediate.

 ── The wait-for-live-estimate contract (A3 handoff #1 — the boot window) ───────────
 A3 proved (localizer.hpp header) that motion commanded before qualityClass()
 leaves Uninitialized acts on a pose that DOES NOT EXIST YET (boot garbage is
 held out of the fold; the published pose is frozen). The C1 contract, enforced
 by every primitive except DriveBrake:

   * While qualityClass() == Uninitialized, tick() commands ZERO volts, makes NO
     settle progress, reports state() == WaitingForEstimate, and captures no
     estimate-derived target.
   * The WATCHDOG RUNS THROUGH THE WAIT: a never-live estimate exits TimedOut
     (with MOTION_TIMEOUT raised) rather than hanging — "no motion can hang" is
     absolute and includes the wait. Callers budget timeouts to cover boot.
   * Targets derived from the current estimate (StrafeTo's held heading,
     HoldPose's captured pose) are captured at the FIRST LIVE tick, never during
     the boot window — capturing at start() would target calibration garbage.
   * Quality::Degraded does NOT gate. A robot that HAD an estimate and lost
     heading authority mid-run keeps driving on the stale estimate (the
     Localizer's own D8 choice: encoders are still good; freezing mid-run
     strands the robot mid-field). The watchdog + tolerance bound the damage.
   * DriveBrake is EXEMPT by design: zero output IS the safe boot action and
     must never be gated behind an estimate.

   Rejected alternatives: refuse-at-start (an auton legitimately starts while
   the IMU is still calibrating — the A3 survival suite's own loop waits, then
   drives); fault-immediately (boot is NORMAL, not a fault — HealthMonitor's
   boot-window-is-not-a-loss rule says exactly this).

 ── Faults & health (A3 handoff #2 lives here) ──────────────────────────────────────
 Every motion carries the fault-discipline wiring so the loop-level containment
 A3 assigned to C1 cannot be forgotten by a caller:
   * TimedOut raises FaultCode::MotionTimeout on the shared FaultLatch.
   * Each active tick runs the OdoStallCheck (spin-vs-motion cross-check — the
     ONLY defence against a frozen/dead encoder until Phase E) and ticks the
     shared HealthMonitor with every observable reachable from the deps
     (imuReady, odomStalled, odomImplausible, fixGated, batteryVolts,
     maxMotorTempC) — so ODO_STUCK / IMU_LOST / BROWNOUT / GPS_GATE_REJECT /
     MOTOR_OVER_TEMP surface during motion with no extra caller wiring.
   The active motion IS the loop at C1; when no motion is active, the loop
   owner ticks the monitor itself (C2 formalizes this ownership).

 ── Observability (A1's contracts) ──────────────────────────────────────────────────
 Every tick emits one DebugRecord through hal::emitRecord() (lazy build — a
 NullSink run never populates it). The record's `commanded` field carries the
 FINAL ACHIEVABLE command expressed in the FIELD frame (the record's pose &
 control section is field-frame by schema): post strafe-authority clamp, so the
 clamping this layer owns is auditable from the record stream.
 MotionState (below) is the wire vocabulary for DebugRecord.activeCommandState.

 ── Units ───────────────────────────────────────────────────────────────────────────
 Pid / SettledUtil / TrapezoidProfile are bare-double BY DESIGN (their headers
 say so). THIS layer owns unit consistency: every PID instance here is
 dedicated to one axis with documented units (translation: inches → in/s;
 heading: radians → rad/s), typed quantities at every boundary, and the only
 place a frame rotation happens is math::fieldToRobot / robotToField (F1).
```

</details>
