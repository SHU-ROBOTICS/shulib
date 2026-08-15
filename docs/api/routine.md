<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/chassis/routine.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `routine.hpp`

Routine — the Tier-2 recipe layer.

This header declares **3** types (36 members).

Extracted from [`include/shulib/chassis/routine.hpp`](../../include/shulib/chassis/routine.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`enum class RoutineStopCause`](#enum-class-routinestopcause)
  - [`None`](#routinestopcause-none)
  - [`MotionFailed`](#routinestopcause-motionfailed)
  - [`WaitTimedOut`](#routinestopcause-waittimedout)
  - [`ActionFailed`](#routinestopcause-actionfailed)
  - [`MechanismFailed`](#routinestopcause-mechanismfailed)
- [`struct RoutineResult`](#struct-routineresult)
  - [`ok`](#routineresult-ok)
  - [`steps`](#routineresult-steps)
  - [`completed`](#routineresult-completed)
  - [`skipped`](#routineresult-skipped)
  - [`stoppedAt`](#routineresult-stoppedat)
  - [`stoppedName`](#routineresult-stoppedname)
  - [`cause`](#routineresult-cause)
  - [`exit`](#routineresult-exit)
- [`class Routine`](#class-routine)
  - [`Routine`](#routine-routine)
  - [`Routine (overload 2)`](#routine-routine-2)
  - [`operator=`](#routine-operator-eq)
  - [`Routine (overload 3)`](#routine-routine-3)
  - [`operator= (overload 2)`](#routine-operator-eq-2)
  - [`~Routine`](#routine-destructor-routine)
  - [`startAt`](#routine-startat)
  - [`moveTo`](#routine-moveto)
  - [`driveTo`](#routine-driveto)
  - [`strafeTo`](#routine-strafeto)
  - [`turnTo`](#routine-turnto)
  - [`face`](#routine-face)
  - [`followTrajectory`](#routine-followtrajectory)
  - [`followTrajectory (overload 2)`](#routine-followtrajectory-2)
  - [`brake`](#routine-brake)
  - [`hold`](#routine-hold)
  - [`pause`](#routine-pause)
  - [`waitFor`](#routine-waitfor)
  - [`then`](#routine-then)
  - [`ok`](#routine-ok)
  - [`result`](#routine-result)
  - [`lastTrajectory`](#routine-lasttrajectory)
  - [`chassis`](#routine-chassis)

<a id="enum-class-routinestopcause"></a>

## `enum class RoutineStopCause`

```cpp
enum class RoutineStopCause
```

Why a Routine stopped early. `None` = it never stopped. Append-only: F1/F3 mechanism failures arrive as new enumerators, never as re-meanings.

*enum class, declared at [`include/shulib/chassis/routine.hpp:153`](../../include/shulib/chassis/routine.hpp#L153).*

<a id="routinestopcause-none"></a>

### `RoutineStopCause::None`

```cpp
None
```

every executed step succeeded (chain still running or finished)

*enumerator, declared at [`include/shulib/chassis/routine.hpp:154`](../../include/shulib/chassis/routine.hpp#L154).*

<a id="routinestopcause-motionfailed"></a>

### `RoutineStopCause::MotionFailed`

```cpp
MotionFailed
```

a motion step exited non-Settled (see RoutineResult::exit)

*enumerator, declared at [`include/shulib/chassis/routine.hpp:155`](../../include/shulib/chassis/routine.hpp#L155).*

<a id="routinestopcause-waittimedout"></a>

### `RoutineStopCause::WaitTimedOut`

```cpp
WaitTimedOut
```

a waitFor() deadline passed with the condition still false

*enumerator, declared at [`include/shulib/chassis/routine.hpp:156`](../../include/shulib/chassis/routine.hpp#L156).*

<a id="routinestopcause-actionfailed"></a>

### `RoutineStopCause::ActionFailed`

```cpp
ActionFailed
```

a then()-action reported failure (bool false / non-Settled)

*enumerator, declared at [`include/shulib/chassis/routine.hpp:157`](../../include/shulib/chassis/routine.hpp#L157).*

<a id="routinestopcause-mechanismfailed"></a>

### `RoutineStopCause::MechanismFailed`

```cpp
MechanismFailed
```

a then()-action returned a mechanism verdict other than Succeeded — Unconfirmed / Stalled / TimedOut / Cancelled; the stop log line names which, and the operation object itself remains the authority on the exact outcome (APPENDED at chunk F1, per the append-only rule above)

*enumerator, declared at [`include/shulib/chassis/routine.hpp:158`](../../include/shulib/chassis/routine.hpp#L158).*

<a id="struct-routineresult"></a>

## `struct RoutineResult`

```cpp
struct RoutineResult
```

What a Routine did — the whole-chain verdict, readable at any point (it is a snapshot; ask again after more steps). ExitReason alone would lose WHERE the routine broke and WHY (a wait and a watchdog are different strategy facts), exactly TrajectoryResult's argument one layer up.

*struct, declared at [`include/shulib/chassis/routine.hpp:169`](../../include/shulib/chassis/routine.hpp#L169).*

<a id="routineresult-ok"></a>

### `RoutineResult::ok`

```cpp
bool ok = true
```

no step has failed (and none skipped)

*field, declared at [`include/shulib/chassis/routine.hpp:170`](../../include/shulib/chassis/routine.hpp#L170).*

<a id="routineresult-steps"></a>

### `RoutineResult::steps`

```cpp
int steps = 0
```

steps encountered so far, including skipped ones

*field, declared at [`include/shulib/chassis/routine.hpp:171`](../../include/shulib/chassis/routine.hpp#L171).*

<a id="routineresult-completed"></a>

### `RoutineResult::completed`

```cpp
int completed = 0
```

steps that ran and succeeded

*field, declared at [`include/shulib/chassis/routine.hpp:172`](../../include/shulib/chassis/routine.hpp#L172).*

<a id="routineresult-skipped"></a>

### `RoutineResult::skipped`

```cpp
int skipped = 0
```

steps skipped after the stop

*field, declared at [`include/shulib/chassis/routine.hpp:173`](../../include/shulib/chassis/routine.hpp#L173).*

<a id="routineresult-stoppedat"></a>

### `RoutineResult::stoppedAt`

```cpp
int stoppedAt = 0
```

1-based index of the failing step; 0 = none

*field, declared at [`include/shulib/chassis/routine.hpp:174`](../../include/shulib/chassis/routine.hpp#L174).*

<a id="routineresult-stoppedname"></a>

### `RoutineResult::stoppedName`

```cpp
const char* stoppedName = ""
```

the failing step's verb/name ("" = none)

*field, declared at [`include/shulib/chassis/routine.hpp:175`](../../include/shulib/chassis/routine.hpp#L175).*

<a id="routineresult-cause"></a>

### `RoutineResult::cause`

```cpp
RoutineStopCause cause = RoutineStopCause::None
```

WHAT kind of thing stopped the chain — read this BEFORE `exit`, because only `MotionFailed` puts a real motion verdict in `exit`.

*field, declared at [`include/shulib/chassis/routine.hpp:178`](../../include/shulib/chassis/routine.hpp#L178).*

<a id="routineresult-exit"></a>

### `RoutineResult::exit`

```cpp
control::ExitReason exit = control::ExitReason::Running
```

The failing MOTION step's verdict (TimedOut / Cancelled). `Running` means "no motion verdict here" — the stop was a wait or an action, or nothing stopped (the same "none yet" convention as CompletedMotion).

*field, declared at [`include/shulib/chassis/routine.hpp:182`](../../include/shulib/chassis/routine.hpp#L182).*

<a id="class-routine"></a>

## `class Routine`

```cpp
class Routine
```

The Tier-2 recipe chain: a complete autonomous routine as a sequence of named steps, each delegating to exactly one Chassis verb, executed EAGERLY (a step runs the moment it is chained) with one built-in failure policy — stop, safe the drive, skip the rest, report. The file banner above explains every one of those choices and is meant to be read.

*class, declared at [`include/shulib/chassis/routine.hpp:190`](../../include/shulib/chassis/routine.hpp#L190).*

<a id="routine-routine"></a>

### `Routine::Routine`

```cpp
explicit Routine(Chassis& chassis, const char* name = "routine") noexcept
```

Borrows `chassis` (must outlive the Routine). `name` appears in the stop/skip log lines so a transcript names WHICH routine stopped; it must be a stable literal (it is stored, not copied).

*function, declared at [`include/shulib/chassis/routine.hpp:195`](../../include/shulib/chassis/routine.hpp#L195).*

<a id="routine-routine-2"></a>

### `Routine::Routine (overload 2)`

```cpp
Routine(const Routine&) = delete
```

Neither copyable nor movable, and there is no reset(): one chain is one run. Two handles sharing the stop-state counters would let a stopped chain's twin keep driving — the failure the whole error policy exists to prevent. Pass a `Routine&` to helpers (that is how you factor a routine into reusable steps); construct a new one for a new run.

*function, declared at [`include/shulib/chassis/routine.hpp:203`](../../include/shulib/chassis/routine.hpp#L203).*

<a id="routine-operator-eq"></a>

### `Routine::operator=`

```cpp
Routine& operator=(const Routine&) = delete
```

*Covered by the comment on [`Routine (overload 2)`](#routine-routine-2) — one comment documents this run of special members.*

*function, declared at [`include/shulib/chassis/routine.hpp:204`](../../include/shulib/chassis/routine.hpp#L204).*

<a id="routine-routine-3"></a>

### `Routine::Routine (overload 3)`

```cpp
Routine(Routine&&) = delete
```

*Covered by the comment on [`Routine (overload 2)`](#routine-routine-2) — one comment documents this run of special members.*

*function, declared at [`include/shulib/chassis/routine.hpp:205`](../../include/shulib/chassis/routine.hpp#L205).*

<a id="routine-operator-eq-2"></a>

### `Routine::operator= (overload 2)`

```cpp
Routine& operator=(Routine&&) = delete
```

*Covered by the comment on [`Routine (overload 2)`](#routine-routine-2) — one comment documents this run of special members.*

*function, declared at [`include/shulib/chassis/routine.hpp:206`](../../include/shulib/chassis/routine.hpp#L206).*

<a id="routine-destructor-routine"></a>

### `Routine::~Routine`

```cpp
~Routine() = default
```

*Covered by the comment on [`Routine (overload 2)`](#routine-routine-2) — one comment documents this run of special members.*

*function, declared at [`include/shulib/chassis/routine.hpp:207`](../../include/shulib/chassis/routine.hpp#L207).*

<a id="routine-startat"></a>

### `Routine::startAt`

```cpp
Routine& startAt(const math::Pose2d& pose)
```

Seed the pose estimate with the measured starting pose — every auton's first line (heading stays IMU-owned, exactly Chassis::setPose).

*function, declared at [`include/shulib/chassis/routine.hpp:213`](../../include/shulib/chassis/routine.hpp#L213).*

<a id="routine-moveto"></a>

### `Routine::moveTo`

```cpp
Routine& moveTo(const math::Pose2d& target, const MotionOptions& options = {})
```

Drive to a FIELD pose — translation and rotation simultaneous (C1).

*function, declared at [`include/shulib/chassis/routine.hpp:223`](../../include/shulib/chassis/routine.hpp#L223).*

<a id="routine-driveto"></a>

### `Routine::driveTo`

```cpp
Routine& driveTo(units::Length x, units::Length y, const MotionOptions& options = {})
```

Drive to the FIELD point (x, y), arriving FACING it: one moveTo whose target heading is the bearing from the live pose estimate to (x, y), computed when this step RUNS (after the steps before it). On tank, face(x, y) first so the approach is a line the drivetrain can follow (C1's honesty: tank verbs never plan turns — face() is YOUR turn).

*function, declared at [`include/shulib/chassis/routine.hpp:232`](../../include/shulib/chassis/routine.hpp#L232).*

<a id="routine-strafeto"></a>

### `Routine::strafeTo`

```cpp
Routine& strafeTo(units::Length x, units::Length y, const MotionOptions& options = {})
```

Translate to FIELD (x, y) holding the current heading (Chassis::strafeTo; on tank an off-line target honestly exits TimedOut).

*function, declared at [`include/shulib/chassis/routine.hpp:243`](../../include/shulib/chassis/routine.hpp#L243).*

<a id="routine-turnto"></a>

### `Routine::turnTo`

```cpp
Routine& turnTo(math::Angle heading, const MotionOptions& options = {})
```

Rotate in place to a FIELD heading, always the short way (Chassis::turnTo).

*function, declared at [`include/shulib/chassis/routine.hpp:249`](../../include/shulib/chassis/routine.hpp#L249).*

<a id="routine-face"></a>

### `Routine::face`

```cpp
Routine& face(units::Length x, units::Length y, const MotionOptions& options = {})
```

Rotate in place to FACE the field point (x, y): one turnTo whose target is the bearing from the live pose estimate to (x, y), computed when this step runs. "Face the goal" in field words; turnTo(atan2(...)) by hand is bit-identical.

*function, declared at [`include/shulib/chassis/routine.hpp:257`](../../include/shulib/chassis/routine.hpp#L257).*

<a id="routine-followtrajectory"></a>

### `Routine::followTrajectory`

```cpp
Routine& followTrajectory(std::span<const math::Pose2d> waypoints, const MotionOptions& options = {})
```

Chain waypoints as sequential moveTo legs (Chassis::followTrajectory — options apply PER LEG). The full TrajectoryResult stays readable via lastTrajectory(); an incomplete trajectory fails the step.

*function, declared at [`include/shulib/chassis/routine.hpp:268`](../../include/shulib/chassis/routine.hpp#L268).*

<a id="routine-followtrajectory-2"></a>

### `Routine::followTrajectory (overload 2)`

```cpp
Routine& followTrajectory(std::initializer_list<math::Pose2d> waypoints, const MotionOptions& options = {})
```

Brace-list convenience: followTrajectory({a, b, c}).

*function, declared at [`include/shulib/chassis/routine.hpp:284`](../../include/shulib/chassis/routine.hpp#L284).*

<a id="routine-brake"></a>

### `Routine::brake`

```cpp
Routine& brake(const MotionOptions& options = {})
```

Controlled stop: 0 V under Brake until the estimate certifies rest (Chassis::brake — a C4 candidate adopted into F6 at D2).

*function, declared at [`include/shulib/chassis/routine.hpp:293`](../../include/shulib/chassis/routine.hpp#L293).*

<a id="routine-hold"></a>

### `Routine::hold`

```cpp
Routine& hold(units::Time duration, const MotionOptions& options = {})
```

Actively hold the current pose for `duration` (Chassis::hold). Typed time (D2): hold(300_ms) — hold(0.3) does not compile.

*function, declared at [`include/shulib/chassis/routine.hpp:299`](../../include/shulib/chassis/routine.hpp#L299).*

<a id="routine-pause"></a>

### `Routine::pause`

```cpp
Routine& pause(units::Time duration)
```

Wait, doing nothing, for `duration` — the alliance-partner beat every real auton has. Motors keep their last state (after a settled motion: stopped); the world keeps advancing. Distinct from hold(): pause() does not energize the drive. A pure delegation to Chassis::wait (D2 — before the wait verb existed, this step carried its own Tier-3 clock-deadline plumbing; that implementation moved down to the facade where both tiers get it). A pause cannot fail, logs nothing, and never stops the chain; nonsense input (NaN, <= 0) throws through untouched, exactly like every step.

*function, declared at [`include/shulib/chassis/routine.hpp:312`](../../include/shulib/chassis/routine.hpp#L312).*

<a id="routine-waitfor"></a>

### `Routine::waitFor`

```cpp
template <typename Pred> Routine& waitFor(Pred&& pred, units::Time timeout, const char* name = "waitFor")
```

Wait until `pred()` holds, up to `timeout` (required and finite — C2's no-hang discipline). In a recipe the condition MATTERS: if the deadline passes with it still false, continuing the script would act on a state the field never reached, so the chain stops (WaitTimedOut). A wait whose timeout is a legitimate strategy branch belongs one tier down: `chassis.waitUntil(...)` directly, branching on the WaitResult.

*function, declared at [`include/shulib/chassis/routine.hpp:328`](../../include/shulib/chassis/routine.hpp#L328).*

<a id="routine-then"></a>

### `Routine::then`

```cpp
template <typename Action> Routine& then(Action&& action, const char* name = "action")
```

Run an action between motions — THE MECHANISM SEAM (the one member deliberately outside F10, filled in at chunk F1). `action` is any callable taking nothing and returning * void        — the action always succeeds, * bool        — false fails the step and stops the chain, * ExitReason  — non-Settled fails the step (so an action may wrap a facade verb and have its verdict honored), * manipulation::MechanismOutcome — a mechanism operation's verdict: ONLY Succeeded continues the chain; Unconfirmed / Stalled / TimedOut / Cancelled stop it as MechanismFailed with the outcome named in the stop line. MechanismOutcome has no bool conversion, so an Unconfirmed can never be truthy by accident — the T2 guarantee that a failed grab cannot read as success at this layer. The mechanism idiom (contract in manipulation/mechanism_op.hpp):  r.then([&] { grab.start(); (void)chassis.waitUntil([&] { return grab.tick() != Running; }, Time{2.0}); return grab.outcome(); }, "grab");  RETURN THE OUTCOME. A void lambda that runs an operation and drops its verdict "succeeds" whatever happened — the same sharp edge as dropping a direct facade call's ExitReason (guide chapter 9), owned the same way. `name` labels the step in stop/skip log lines (stable literal).

*function, declared at [`include/shulib/chassis/routine.hpp:369`](../../include/shulib/chassis/routine.hpp#L369).*

<a id="routine-ok"></a>

### `Routine::ok`

```cpp
[[nodiscard]] bool ok() const noexcept
```

True while no step has failed (and none was skipped).

*function, declared at [`include/shulib/chassis/routine.hpp:405`](../../include/shulib/chassis/routine.hpp#L405).*

<a id="routine-result"></a>

### `Routine::result`

```cpp
[[nodiscard]] RoutineResult result() const noexcept
```

The chain verdict so far (a snapshot — see RoutineResult).

*function, declared at [`include/shulib/chassis/routine.hpp:408`](../../include/shulib/chassis/routine.hpp#L408).*

<a id="routine-lasttrajectory"></a>

### `Routine::lastTrajectory`

```cpp
[[nodiscard]] const TrajectoryResult& lastTrajectory() const noexcept
```

The most recent followTrajectory step's full result — completedLegs is strategy-relevant and must not be flattened away by the chain. Before any trajectory has run it reads `exit = Running`, the project's "no verdict here yet" convention (RoutineResult::exit, CompletedMotion), so succeeded() is honestly FALSE on a virgin routine. (D3: a plain value-initialized TrajectoryResult reports `Settled` with 0 of 0 legs, which succeeded() calls SUCCESS — correct for the facade, whose verb requires at least one waypoint, and a lie here, where the member exists before any trajectory does.)

*function, declared at [`include/shulib/chassis/routine.hpp:430`](../../include/shulib/chassis/routine.hpp#L430).*

<a id="routine-chassis"></a>

### `Routine::chassis`

```cpp
[[nodiscard]] Chassis& chassis() noexcept
```

The chassis this routine drives — the mixed-tier seam, spelled out. (You can equally keep your own reference; this exists so a routine passed across a function boundary still reaches Tier 3.)

*function, declared at [`include/shulib/chassis/routine.hpp:437`](../../include/shulib/chassis/routine.hpp#L437).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 127 lines, click to expand</summary>

```text

 Routine — the Tier-2 recipe layer (chunk D1, WS12/M7 pulled forward; master
 plan §17).

 ═══ STATUS: FROZEN — F10, LOCKED 2026-08-12 (chunk D3, API 2.0) ══════════════════
 This surface is FROZEN, as its own register row (F10) rather than as part of
 F6: the facade it delegates to is a different tier and can version
 independently. Frozen: the constructor (both spellings, noexcept),
 non-copyable/non-movable, ELEVEN steps — startAt / moveTo / driveTo /
 strafeTo / turnTo / face / followTrajectory (span + brace) / brake / hold /
 pause / waitFor — the four observers ok / result / lastTrajectory / chassis,
 the types RoutineResult (all eight fields) and RoutineStopCause (append-only,
 existing values fixed), the documented error policy below, and typed time as
 a SEMANTIC (hold(0.3) must not compile). Enforced structurally:
 test/routine_signature_pin_test.cpp fails the build, naming F10, if any of
 those drifts. Changes only with a major API-version bump plus a migration
 note (include/shulib/version.hpp); ADDITIVE growth — new steps, new
 observers, new RoutineResult fields, appended RoutineStopCause enumerators —
 stays legal and is the intended path.

 NOT frozen, deliberately: **then()** — its accepted return types and its
 `name` default were left out of F10 because they were chosen before
 mechanisms existed; chunk F1 then filled the seam (a fourth accepted return
 type, manipulation::MechanismOutcome, and the MechanismFailed stop cause).
 then() STAYS unfrozen until the seam has a second real consumer (F2's
 combinators / F3's primitives) — the same build → second consumer → freeze
 path every other surface here took. Also unfrozen: the exact WORDING of the
 stop/skip log lines (the behaviour — one Warn naming routine and step, one
 Info per skipped step — is frozen; the sentence is not).
 Stated out loud because silence in a freeze reads as "frozen too" (D2 ruling
 A2's lesson). The freeze waited for a second independent consumer: D3's
 recipe cookbook (docs/cookbook/), which wrote fourteen recipes against this
 surface and needed zero changes to it. Its critique — every awkwardness
 found, with a recommendation — is the D3 completion record's centrepiece
 (development log, shulib-v2 branch).

 ═══ What this class is ══════════════════════════════════════════════════════════
 A fluent, EAGER chain over the Chassis facade, so a complete autonomous
 routine is ~10 readable lines:

     Routine r{chassis, "left-side"};
     r.startAt(Pose2d{-48_in, -24_in, 90_deg})
      .moveTo(Pose2d{-24_in, 0_in, 45_deg})
      .then([&] { intake.in(); }, "intake")  // your mechanism code (ch. 13)
      .strafeTo(-24_in, 24_in)
      .face(0_in, 48_in)
      .driveTo(0_in, 48_in)
      .hold(300_ms)
      .brake();
     if (!r.ok()) { /* strategy branch on r.result() */ }

 EAGER means each step runs (blocking) the moment it is chained — the routine
 runs exactly in the order it reads, top to bottom, and a first-year can say
 what the robot will do by reading it. The deliberate alternative (a DEFERRED
 chain that accumulates steps and executes on a terminal .run()) was rejected:
 it re-opens the read-order/run-order split when recipe and facade calls mix,
 it needs step storage (heap or an arbitrary capacity) where this class needs
 none, and it adds the worst misuse door of all — a chain built but never
 run, which compiles and does nothing. The full analysis is in the D1
 completion record (development log, `shulib-v2` branch).

 ═══ Tier discipline: this layer DELEGATES ═══════════════════════════════════════
 Every step is exactly one Chassis call (§17: tiers are strict supersets — a
 recipe is never the only way to do something). There is no motion logic
 here: no gains, no kinematics, no exit criteria, no fault policy. The two
 "field vocabulary" steps, face(x, y) and driveTo(x, y), compute ONE argument
 (the bearing from the live pose estimate to a field point — pure trig) and
 then delegate to turnTo / moveTo; that keeps C1's D12 intact on tank drives
 (the AUTHOR still writes the turn — face() IS the author's turn, said in
 field words), and a hand-written turnTo(atan2(...)) remains bit-identical
 to it by construction. Dropping from Tier 2 to Tier 3 is deleting the
 Routine object and keeping every verb.

 ═══ The error policy (decided, documented, tested — design constraint 4) ═════════
 A fluent chain makes it EASY to swallow an ExitReason, so the chain acts on
 failure whether or not anyone reads the result:

   * A step FAILS when its motion exits non-Settled, a trajectory does not
     complete every leg, a waitFor() times out, or a then()-action reports
     failure. On the first failure the chain STOPS:
       1. the drive is put in the defined safe state (Chassis::cancel — 0 V
          + Brake; idempotent, HA-53),
       2. every later step is SKIPPED (counted and visible, never executed —
          a routine that keeps driving after a failed move is chasing the
          field from a position it is not at),
       3. one Warn names the failing step; each skipped step logs at Info,
       4. ok() goes false and result() carries step index, name, cause, and
          the failing motion's ExitReason.
   * Skipping is not silent recovery: the transcript shows the stop, the
     fault latch and C5 result lines below carry the pathology, and the
     robot is parked — the failure is behaviourally loud even if the author
     never branches on it.
   * PRECONDITION THROWS ARE NOT POLICY. Nonsense input (NaN pose, negative
     timeout) throws out of the step exactly as the facade throws — a
     programming error must stay loud and early, never be converted into a
     polite "the chain stopped". A throwing step leaves the chain's counters
     untouched (the bad call never ran).

 A routine that must CONTINUE past a failure (a sweep that shrugs off one
 missed goal) branches on the verbs directly — that is one step down the
 tier ladder, not a rewrite, and mixing the two styles in one routine is
 supported and tested (steps and direct facade calls interleave freely,
 because eager execution keeps program order = field order).

 ═══ then(): the mechanism seam (filled at F1; still unfrozen — see above) ═══════
 then() accepts any callable returning void (always succeeds), bool, or
 ExitReason (failure stops the chain like any step) — and, since F1, a
 mechanism operation's MechanismOutcome, where ONLY Succeeded continues and
 an Unconfirmed grab can never read as success (the T2 guarantee; the member
 comment carries the idiom). One honest spelling note: the action is a
 CALLABLE, so a member function is passed as `then([&] { intake.in(); })` —
 `then(intake.in)` is valid C++ only for a callable data member, and the
 documentation spells the lambda form everywhere for that reason (T5).

 ═══ What a recipe deliberately CANNOT do (the documented gaps) ══════════════════
   * drive(speeds, Frame) — a recipe is a SEQUENCE; drive() is a per-
     iteration loop primitive (teleop / expert escape hatch). A routine that
     wants it calls the facade directly, mid-chain if it likes.
   * cancel() — the panic stop belongs to whoever supervises the routine,
     not to a step inside it (a step that cancels itself is just a shorter
     chain). The chain uses it internally on failure.
   * Branch on pose — recipes are position-blind between steps by design;
     `chassis.pose()` mid-chain is the supported mixed-tier idiom.

 Single-task, like everything it wraps. Borrows the Chassis (must outlive
 this object). Copying is disabled: two handles sharing stop-state counters
 would let a stopped chain's twin keep driving.
```

</details>
