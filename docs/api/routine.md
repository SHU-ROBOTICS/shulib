<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/chassis/routine.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `routine.hpp`

Routine — the Tier-2 recipe layer.

This header declares **5** types (85 members), **1** free function, **2** constants, and **6** type aliass.

Extracted from [`include/shulib/chassis/routine.hpp`](../../include/shulib/chassis/routine.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in [How it works](../guide/README.md); worked recipes live in [Recipes](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`struct OnFail`](#struct-onfail)
  - [`kind`](#onfail-kind)
  - [`target`](#onfail-target)
  - [`enum class OnFail::Kind`](#enum-class-onfail-kind)
    - [`Stop`](#onfail-kind-stop)
    - [`CarryOn`](#onfail-kind-carryon)
    - [`SkipTo`](#onfail-kind-skipto)
- [`stopChain`](#stopchain) — *constant*
- [`carryOn`](#carryon) — *constant*
- [`skipTo`](#skipto) — *free function*
- [`enum class RoutineStopCause`](#enum-class-routinestopcause)
  - [`None`](#routinestopcause-none)
  - [`MotionFailed`](#routinestopcause-motionfailed)
  - [`WaitTimedOut`](#routinestopcause-waittimedout)
  - [`ActionFailed`](#routinestopcause-actionfailed)
  - [`MechanismFailed`](#routinestopcause-mechanismfailed)
  - [`MechanismStepFailed`](#routinestopcause-mechanismstepfailed)
  - [`MechanismNotStarted`](#routinestopcause-mechanismnotstarted)
- [`struct RoutineResult`](#struct-routineresult)
  - [`ok`](#routineresult-ok)
  - [`steps`](#routineresult-steps)
  - [`completed`](#routineresult-completed)
  - [`skipped`](#routineresult-skipped)
  - [`stoppedAt`](#routineresult-stoppedat)
  - [`stoppedName`](#routineresult-stoppedname)
  - [`cause`](#routineresult-cause)
  - [`exit`](#routineresult-exit)
  - [`mechanismVerdict`](#routineresult-mechanismverdict)
  - [`failedOn`](#routineresult-failedon)
  - [`firstFailedAt`](#routineresult-firstfailedat)
  - [`firstFailedName`](#routineresult-firstfailedname)
  - [`firstFailedCause`](#routineresult-firstfailedcause)
  - [`jumped`](#routineresult-jumped)
  - [`skipPendingTo`](#routineresult-skippendingto)
- [`class Routine`](#class-routine)
  - [`Routine`](#routine-routine)
  - [`Routine (overload 2)`](#routine-routine-2)
  - [`Routine (overload 3)`](#routine-routine-3)
  - [`operator=`](#routine-operator-eq)
  - [`Routine (overload 4)`](#routine-routine-4)
  - [`operator= (overload 2)`](#routine-operator-eq-2)
  - [`~Routine`](#routine-destructor-routine)
  - [`startAt`](#routine-startat)
  - [`moveTo`](#routine-moveto)
  - [`driveTo`](#routine-driveto)
  - [`reverseTo`](#routine-reverseto)
  - [`strafeTo`](#routine-strafeto)
  - [`turnTo`](#routine-turnto)
  - [`face`](#routine-face)
  - [`faceAway`](#routine-faceaway)
  - [`followTrajectory`](#routine-followtrajectory)
  - [`followTrajectory (overload 2)`](#routine-followtrajectory-2)
  - [`curveTo`](#routine-curveto)
  - [`curveTo (overload 2)`](#routine-curveto-2)
  - [`reverseCurveTo`](#routine-reversecurveto)
  - [`reverseCurveTo (overload 2)`](#routine-reversecurveto-2)
  - [`brake`](#routine-brake)
  - [`hold`](#routine-hold)
  - [`pause`](#routine-pause)
  - [`waitFor`](#routine-waitfor)
  - [`then`](#routine-then)
  - [`onFail`](#routine-onfail)
  - [`label`](#routine-label)
  - [`move`](#routine-move)
  - [`startMove`](#routine-startmove)
  - [`home`](#routine-home)
  - [`home (overload 2)`](#routine-home-2)
  - [`startHome`](#routine-starthome)
  - [`startHome (overload 2)`](#routine-starthome-2)
  - [`run`](#routine-run)
  - [`run (overload 2)`](#routine-run-2)
  - [`startRun`](#routine-startrun)
  - [`startRun (overload 2)`](#routine-startrun-2)
  - [`set`](#routine-set)
  - [`spinUp`](#routine-spinup)
  - [`startSpinUp`](#routine-startspinup)
  - [`stop`](#routine-stop)
  - [`stop (overload 2)`](#routine-stop-2)
  - [`waitUntilDone`](#routine-waituntildone)
  - [`requires`](#routine-requires)
  - [`run (overload 3)`](#routine-run-3)
  - [`requires (overload 2)`](#routine-requires-2)
  - [`requires (overload 3)`](#routine-requires-3)
  - [`lastMechanism`](#routine-lastmechanism)
  - [`TriggerMark`](#routine-triggermark)
  - [`ArmedLeg`](#routine-armedleg)
  - [`atPercent`](#routine-atpercent)
  - [`afterDistance`](#routine-afterdistance)
  - [`afterTurning`](#routine-afterturning)
  - [`ok`](#routine-ok)
  - [`result`](#routine-result)
  - [`lastTrajectory`](#routine-lasttrajectory)
  - [`chassis`](#routine-chassis)
- [`Routine`](#routine) — *type alias*
- [`RoutineResult`](#routineresult) — *type alias*
- [`carryOn (overload 2)`](#carryon-2) — *type alias*
- [`OnFail`](#onfail) — *type alias*
- [`skipTo (overload 2)`](#skipto-2) — *type alias*
- [`stopChain (overload 2)`](#stopchain-2) — *type alias*

<a id="struct-onfail"></a>

## `struct OnFail`

```cpp
struct OnFail
```

What a step does when it FAILS — the value `r.onFail(...)` takes (the file banner's "on-failure choice"). Written with one of its three spellings: `stopChain` (the default rule: stop, safe the drive, skip the rest), `carryOn` (count it, safe the drive, run the next step) or `skipTo("x")` (count it, safe the drive, pass over every step until the one `r.label("x")` names). A plain value: build it with those three, not by hand — `r.onFail` refuses a SkipTo with no target.

*struct, declared at [`include/shulib/chassis/routine.hpp:313`](../../include/shulib/chassis/routine.hpp#L313).*

<a id="onfail-kind"></a>

### `OnFail::kind`

```cpp
Kind kind = Kind::Stop
```

which choice

*field, declared at [`include/shulib/chassis/routine.hpp:320`](../../include/shulib/chassis/routine.hpp#L320).*

<a id="onfail-target"></a>

### `OnFail::target`

```cpp
const char* target = nullptr
```

SkipTo's label — a stable literal (stored, not copied)

*field, declared at [`include/shulib/chassis/routine.hpp:321`](../../include/shulib/chassis/routine.hpp#L321).*

<a id="enum-class-onfail-kind"></a>

## `enum class OnFail::Kind`

```cpp
enum class Kind : std::uint8_t
```

Which of the three.

*enum class, declared at [`include/shulib/chassis/routine.hpp:315`](../../include/shulib/chassis/routine.hpp#L315).*

<a id="onfail-kind-stop"></a>

### `OnFail::Kind::Stop`

```cpp
Stop
```

the default rule (stopChain)

*enumerator, declared at [`include/shulib/chassis/routine.hpp:316`](../../include/shulib/chassis/routine.hpp#L316).*

<a id="onfail-kind-carryon"></a>

### `OnFail::Kind::CarryOn`

```cpp
CarryOn
```

count the failure and run the next step (carryOn)

*enumerator, declared at [`include/shulib/chassis/routine.hpp:317`](../../include/shulib/chassis/routine.hpp#L317).*

<a id="onfail-kind-skipto"></a>

### `OnFail::Kind::SkipTo`

```cpp
SkipTo
```

count the failure and pass over every step until `target`'s label (skipTo)

*enumerator, declared at [`include/shulib/chassis/routine.hpp:318`](../../include/shulib/chassis/routine.hpp#L318).*

<a id="stopchain"></a>

## `stopChain`

```cpp
inline constexpr OnFail stopChain{OnFail::Kind::Stop, nullptr}
```

The default rule, spelled out: the failing step stops the chain (`r.onFail(stopChain)`).

*constant, declared at [`include/shulib/chassis/routine.hpp:324`](../../include/shulib/chassis/routine.hpp#L324).*

<a id="carryon"></a>

## `carryOn`

```cpp
inline constexpr OnFail carryOn{OnFail::Kind::CarryOn, nullptr}
```

Count the failure, safe the drive, run the next step from where the robot is (`r.onFail(carryOn)`).

*constant, declared at [`include/shulib/chassis/routine.hpp:327`](../../include/shulib/chassis/routine.hpp#L327).*

<a id="skipto"></a>

## `skipTo`

```cpp
[[nodiscard]] inline OnFail skipTo(const char* label)
```

Count the failure, safe the drive, pass over every step until the one labelled `label` (`r.onFail(skipTo("third piece"))` ... `r.label("third piece")`). Skips go FORWARD only. `label` is stored, not copied (a string literal), and matched whole and case-sensitively. PRECONDITION: `label` not null and not empty.

*free function, declared at [`include/shulib/chassis/routine.hpp:332`](../../include/shulib/chassis/routine.hpp#L332).*

<a id="enum-class-routinestopcause"></a>

## `enum class RoutineStopCause`

```cpp
enum class RoutineStopCause
```

Why a Routine stopped early. `None` = it never stopped. Append-only: F1/F3 mechanism failures arrive as new enumerators, never as re-meanings.

*enum class, declared at [`include/shulib/chassis/routine.hpp:342`](../../include/shulib/chassis/routine.hpp#L342).*

<a id="routinestopcause-none"></a>

### `RoutineStopCause::None`

```cpp
None
```

every executed step succeeded (chain still running or finished)

*enumerator, declared at [`include/shulib/chassis/routine.hpp:343`](../../include/shulib/chassis/routine.hpp#L343).*

<a id="routinestopcause-motionfailed"></a>

### `RoutineStopCause::MotionFailed`

```cpp
MotionFailed
```

a motion step exited non-Settled (see RoutineResult::exit)

*enumerator, declared at [`include/shulib/chassis/routine.hpp:344`](../../include/shulib/chassis/routine.hpp#L344).*

<a id="routinestopcause-waittimedout"></a>

### `RoutineStopCause::WaitTimedOut`

```cpp
WaitTimedOut
```

a waitFor() deadline passed with the condition still false

*enumerator, declared at [`include/shulib/chassis/routine.hpp:345`](../../include/shulib/chassis/routine.hpp#L345).*

<a id="routinestopcause-actionfailed"></a>

### `RoutineStopCause::ActionFailed`

```cpp
ActionFailed
```

a then()-action reported failure (bool false / non-Settled)

*enumerator, declared at [`include/shulib/chassis/routine.hpp:346`](../../include/shulib/chassis/routine.hpp#L346).*

<a id="routinestopcause-mechanismfailed"></a>

### `RoutineStopCause::MechanismFailed`

```cpp
MechanismFailed
```

a then()-action returned a mechanism verdict other than Succeeded — Unconfirmed / Stalled / TimedOut / Cancelled; the stop log line names which, and the operation object itself remains the authority on the exact outcome (APPENDED at chunk F1, per the append-only rule above)

*enumerator, declared at [`include/shulib/chassis/routine.hpp:347`](../../include/shulib/chassis/routine.hpp#L347).*

<a id="routinestopcause-mechanismstepfailed"></a>

### `RoutineStopCause::MechanismStepFailed`

```cpp
MechanismStepFailed
```

a mechanism STEP (move, run, set, spinUp, home, stop, waitUntilDone — and their start forms' starts) ended other than SUCCEEDED or an UNCONFIRMED counted done; the verdict is RoutineResult::mechanismVerdict and the stop line names it. MechanismFailed keeps its F1 meaning (a then()-action's verdict). (APPENDED at chunk MECH-A4, API 2.5.)

*enumerator, declared at [`include/shulib/chassis/routine.hpp:352`](../../include/shulib/chassis/routine.hpp#L352).*

<a id="routinestopcause-mechanismnotstarted"></a>

### `RoutineStopCause::MechanismNotStarted`

```cpp
MechanismNotStarted
```

a waitUntilDone() whose START never ran: a skip jumped the step that would have started it, or that step did nothing because its goal is another robot's and the chain carried on (STEP1-A) — or, from STEP1-B, its partway trigger never fired. Never a throw, never an older step's verdict; RoutineResult::mechanismVerdict stays `Running` (nothing ran). (APPENDED at chunk STEP1-A, == 6.)

*enumerator, declared at [`include/shulib/chassis/routine.hpp:357`](../../include/shulib/chassis/routine.hpp#L357).*

<a id="struct-routineresult"></a>

## `struct RoutineResult`

```cpp
struct RoutineResult
```

What a Routine did — the whole-chain verdict, readable at any point (it is a snapshot; ask again after more steps). ExitReason alone would lose WHERE the routine broke and WHY (a wait and a watchdog are different strategy facts), exactly TrajectoryResult's argument one layer up.

*struct, declared at [`include/shulib/chassis/routine.hpp:369`](../../include/shulib/chassis/routine.hpp#L369).*

<a id="routineresult-ok"></a>

### `RoutineResult::ok`

```cpp
bool ok = true
```

no step has failed (and none skipped)

*field, declared at [`include/shulib/chassis/routine.hpp:370`](../../include/shulib/chassis/routine.hpp#L370).*

<a id="routineresult-steps"></a>

### `RoutineResult::steps`

```cpp
int steps = 0
```

steps encountered so far, including skipped ones

*field, declared at [`include/shulib/chassis/routine.hpp:371`](../../include/shulib/chassis/routine.hpp#L371).*

<a id="routineresult-completed"></a>

### `RoutineResult::completed`

```cpp
int completed = 0
```

steps that ran and succeeded

*field, declared at [`include/shulib/chassis/routine.hpp:372`](../../include/shulib/chassis/routine.hpp#L372).*

<a id="routineresult-skipped"></a>

### `RoutineResult::skipped`

```cpp
int skipped = 0
```

steps skipped after the stop

*field, declared at [`include/shulib/chassis/routine.hpp:373`](../../include/shulib/chassis/routine.hpp#L373).*

<a id="routineresult-stoppedat"></a>

### `RoutineResult::stoppedAt`

```cpp
int stoppedAt = 0
```

1-based index of the failing step; 0 = none

*field, declared at [`include/shulib/chassis/routine.hpp:374`](../../include/shulib/chassis/routine.hpp#L374).*

<a id="routineresult-stoppedname"></a>

### `RoutineResult::stoppedName`

```cpp
const char* stoppedName = ""
```

the failing step's verb/name ("" = none)

*field, declared at [`include/shulib/chassis/routine.hpp:375`](../../include/shulib/chassis/routine.hpp#L375).*

<a id="routineresult-cause"></a>

### `RoutineResult::cause`

```cpp
RoutineStopCause cause = RoutineStopCause::None
```

WHAT kind of thing stopped the chain — read this BEFORE `exit`, because only `MotionFailed` puts a real motion verdict in `exit`.

*field, declared at [`include/shulib/chassis/routine.hpp:378`](../../include/shulib/chassis/routine.hpp#L378).*

<a id="routineresult-exit"></a>

### `RoutineResult::exit`

```cpp
control::ExitReason exit = control::ExitReason::Running
```

The failing MOTION step's verdict (TimedOut / Cancelled). `Running` means "no motion verdict here" — the stop was a wait or an action, or nothing stopped (the same "none yet" convention as CompletedMotion).

*field, declared at [`include/shulib/chassis/routine.hpp:382`](../../include/shulib/chassis/routine.hpp#L382).*

<a id="routineresult-mechanismverdict"></a>

### `RoutineResult::mechanismVerdict`

```cpp
manipulation::MechanismOutcome mechanismVerdict = manipulation::MechanismOutcome::Running
```

The failing MECHANISM step's verdict — TimedOut / Stalled / Absent / NotHomed / Refused / Cancelled / an Unconfirmed NOT counted done. `Running` means "no mechanism verdict here": read `cause` first — only `MechanismStepFailed` puts one here. (APPENDED at chunk MECH-A4, API 2.5.)

*field, declared at [`include/shulib/chassis/routine.hpp:387`](../../include/shulib/chassis/routine.hpp#L387).*

<a id="routineresult-failedon"></a>

### `RoutineResult::failedOn`

```cpp
int failedOn = 0
```

steps that FAILED and the chain went on (carryOn / skipTo)

*field, declared at [`include/shulib/chassis/routine.hpp:391`](../../include/shulib/chassis/routine.hpp#L391).*

<a id="routineresult-firstfailedat"></a>

### `RoutineResult::firstFailedAt`

```cpp
int firstFailedAt = 0
```

1-based index of the first of them; 0 = none

*field, declared at [`include/shulib/chassis/routine.hpp:392`](../../include/shulib/chassis/routine.hpp#L392).*

<a id="routineresult-firstfailedname"></a>

### `RoutineResult::firstFailedName`

```cpp
const char* firstFailedName = ""
```

its verb/name ("" = none)

*field, declared at [`include/shulib/chassis/routine.hpp:393`](../../include/shulib/chassis/routine.hpp#L393).*

<a id="routineresult-firstfailedcause"></a>

### `RoutineResult::firstFailedCause`

```cpp
RoutineStopCause firstFailedCause = RoutineStopCause::None
```

Its cause — read `firstFailedAt` first (None when no step failed and went on).

*field, declared at [`include/shulib/chassis/routine.hpp:395`](../../include/shulib/chassis/routine.hpp#L395).*

<a id="routineresult-jumped"></a>

### `RoutineResult::jumped`

```cpp
int jumped = 0
```

steps a skip passed over (counted in `steps`, never run)

*field, declared at [`include/shulib/chassis/routine.hpp:396`](../../include/shulib/chassis/routine.hpp#L396).*

<a id="routineresult-skippendingto"></a>

### `RoutineResult::skipPendingTo`

```cpp
const char* skipPendingTo = ""
```

The label a skip is still looking for — "" when none is. Non-empty at the end of a run: the skip never landed, and nothing ran after its failure.

*field, declared at [`include/shulib/chassis/routine.hpp:399`](../../include/shulib/chassis/routine.hpp#L399).*

<a id="class-routine"></a>

## `class Routine`

```cpp
class Routine
```

The Tier-2 recipe chain: a complete autonomous routine as a sequence of named steps, each delegating to exactly one Chassis verb, executed EAGERLY (a step runs the moment it is chained) with one built-in failure policy — stop, safe the drive, skip the rest, report. The file banner above explains every one of those choices and is meant to be read.

*class, declared at [`include/shulib/chassis/routine.hpp:407`](../../include/shulib/chassis/routine.hpp#L407).*

<a id="routine-routine"></a>

### `Routine::Routine`

```cpp
explicit Routine(Chassis& chassis, const char* name = "routine") noexcept
```

Borrows `chassis` (must outlive the Routine). `name` appears in the stop/skip log lines so a transcript names WHICH routine stopped; it must be a stable literal (it is stored, not copied).

*function, declared at [`include/shulib/chassis/routine.hpp:412`](../../include/shulib/chassis/routine.hpp#L412).*

<a id="routine-routine-2"></a>

### `Routine::Routine (overload 2)`

```cpp
template <detail::RoutineContext Context> explicit Routine(Context& ctx, const char* name = "routine") noexcept
```

MECH-A4 (API 2.5; a NEW overload — the frozen constructor above is untouched): `Routine r{ctx, "score-three"}` in an auton — the chassis AND the robot's mechanism service, from the auton's context (competition::AutonContext; any type with these two members). Borrows both (each must outlive the Routine). A program that runs autons without a service leaves `mechanismService` null: the chassis steps run as ever, and a mechanism step is a loud precondition naming the fix. Not frozen (the file header). STEP1-A: when the context also has a `guard` (AutonContext does), the Routine borrows it for the cut rule — a step that fails once the guard is running and expired() STOPS the chain whatever its onFail choice (the file banner). A context without one builds a guard-less Routine, as Routine(Chassis&) does.

*function, declared at [`include/shulib/chassis/routine.hpp:426`](../../include/shulib/chassis/routine.hpp#L426).*

<a id="routine-routine-3"></a>

### `Routine::Routine (overload 3)`

```cpp
Routine(const Routine&) = delete
```

Neither copyable nor movable, and there is no reset(): one chain is one run. Two handles sharing the stop-state counters would let a stopped chain's twin keep driving — the failure the whole error policy exists to prevent. Pass a `Routine&` to helpers (that is how you factor a routine into reusable steps); construct a new one for a new run.

*function, declared at [`include/shulib/chassis/routine.hpp:434`](../../include/shulib/chassis/routine.hpp#L434).*

<a id="routine-operator-eq"></a>

### `Routine::operator=`

```cpp
Routine& operator=(const Routine&) = delete
```

*Covered by the comment on [`Routine (overload 3)`](#routine-routine-3) — one comment documents this run of special members.*

*function, declared at [`include/shulib/chassis/routine.hpp:435`](../../include/shulib/chassis/routine.hpp#L435).*

<a id="routine-routine-4"></a>

### `Routine::Routine (overload 4)`

```cpp
Routine(Routine&&) = delete
```

*Covered by the comment on [`Routine (overload 3)`](#routine-routine-3) — one comment documents this run of special members.*

*function, declared at [`include/shulib/chassis/routine.hpp:436`](../../include/shulib/chassis/routine.hpp#L436).*

<a id="routine-operator-eq-2"></a>

### `Routine::operator= (overload 2)`

```cpp
Routine& operator=(Routine&&) = delete
```

*Covered by the comment on [`Routine (overload 3)`](#routine-routine-3) — one comment documents this run of special members.*

*function, declared at [`include/shulib/chassis/routine.hpp:437`](../../include/shulib/chassis/routine.hpp#L437).*

<a id="routine-destructor-routine"></a>

### `Routine::~Routine`

```cpp
~Routine()
```

Neither copyable nor movable, and there is no reset(): one chain is one run. Two handles sharing the stop-state counters would let a stopped chain's twin keep driving — the failure the whole error policy exists to prevent. Pass a `Routine&` to helpers (that is how you factor a routine into reusable steps); construct a new one for a new run. STEP1-B: a Routine destroyed with a partway trigger still ARMED — its handle was discarded, and that line was the auton's last, so no later step's precondition could say so — logs ONE Warn: the triggers never fired. The same for a line that ENDED AT ITS MARK (`r.atPercent(60);` — a mark written, no start after it). Nothing when nothing is armed; a log failure is swallowed (a destructor never throws).

*function, declared at [`include/shulib/chassis/routine.hpp:443`](../../include/shulib/chassis/routine.hpp#L443).*

<a id="routine-startat"></a>

### `Routine::startAt`

```cpp
Routine& startAt(const math::Pose2d& pose)
```

Seed the pose estimate with the measured starting pose — every auton's first line (heading stays IMU-owned, exactly Chassis::setPose).

*function, declared at [`include/shulib/chassis/routine.hpp:468`](../../include/shulib/chassis/routine.hpp#L468).*

<a id="routine-moveto"></a>

### `Routine::moveTo`

```cpp
Routine& moveTo(const math::Pose2d& target, const MotionOptions& options = {})
```

Drive to a FIELD pose — translation and rotation simultaneous (C1).

*function, declared at [`include/shulib/chassis/routine.hpp:478`](../../include/shulib/chassis/routine.hpp#L478).*

<a id="routine-driveto"></a>

### `Routine::driveTo`

```cpp
Routine& driveTo(units::Length x, units::Length y, const MotionOptions& options = {})
```

Drive to the FIELD point (x, y), arriving FACING it: one moveTo whose target heading is the bearing from the live pose estimate to (x, y), computed when this step RUNS (after the steps before it). On tank, face(x, y) first so the approach is a line the drivetrain can follow (C1's honesty: tank verbs never plan turns — face() is YOUR turn); it is also what makes the leg steerable — a tank moveTo steers back onto its line only when the leg starts within 15° of its target heading, and logs "turn first: face()" when it does not (chassis.hpp: TANK STEERING). To flow through points instead of stopping at each, curveTo (CURVE1).

*function, declared at [`include/shulib/chassis/routine.hpp:491`](../../include/shulib/chassis/routine.hpp#L491).*

<a id="routine-reverseto"></a>

### `Routine::reverseTo`

```cpp
Routine& reverseTo(units::Length x, units::Length y, const MotionOptions& options = {})
```

Drive to the FIELD point (x, y) and arrive with the robot's BACK toward it: one moveTo whose target heading is the bearing from the live pose estimate to (x, y) plus π, computed when this step RUNS — driveTo turned around, and exactly a route's DriveTo waypoint with `reverse` set (runner/path_runner.hpp spells the `+ π` the same way, so a route and a recipe that name the same leg stay bit-identical). The caller's options pass through whole. The leg starts as far off its target heading as the robot's back is turned from the point, so after faceAway(x, y) it drives BACKWARDS the whole way, on any drive. Without it: a holonomic drive rotates in flight — it still arrives turned round, but its nose leads early in the leg. A tank never plans the turn (faceAway() is YOUR turn) and steers a leg back onto its line only when it starts within 15° of its target heading: from a robot FACING the point it starts 180° off, is not steered, logs "turn first: face()" — for a backwards leg that turn is faceAway() — and times out beside the point (chassis.hpp: TANK STEERING); a tank that just drove straight out from the point already has its back to it and needs no faceAway(). No robot has run a reverseTo yet (robot two has reversed only under driver control): whether a tank backs up as far as its calibration says is register entry HA-167. (Appended at chunk L9, API 2.4.)

*function, declared at [`include/shulib/chassis/routine.hpp:516`](../../include/shulib/chassis/routine.hpp#L516).*

<a id="routine-strafeto"></a>

### `Routine::strafeTo`

```cpp
Routine& strafeTo(units::Length x, units::Length y, const MotionOptions& options = {})
```

Translate to FIELD (x, y) holding the current heading (Chassis::strafeTo; on tank an off-line target honestly exits TimedOut).

*function, declared at [`include/shulib/chassis/routine.hpp:527`](../../include/shulib/chassis/routine.hpp#L527).*

<a id="routine-turnto"></a>

### `Routine::turnTo`

```cpp
Routine& turnTo(math::Angle heading, const MotionOptions& options = {})
```

Rotate in place to a FIELD heading, always the short way (Chassis::turnTo).

*function, declared at [`include/shulib/chassis/routine.hpp:533`](../../include/shulib/chassis/routine.hpp#L533).*

<a id="routine-face"></a>

### `Routine::face`

```cpp
Routine& face(units::Length x, units::Length y, const MotionOptions& options = {})
```

Rotate in place to FACE the field point (x, y): one turnTo whose target is the bearing from the live pose estimate to (x, y), computed when this step runs. "Face the goal" in field words; turnTo(atan2(...)) by hand is bit-identical.

*function, declared at [`include/shulib/chassis/routine.hpp:541`](../../include/shulib/chassis/routine.hpp#L541).*

<a id="routine-faceaway"></a>

### `Routine::faceAway`

```cpp
Routine& faceAway(units::Length x, units::Length y, const MotionOptions& options = {})
```

Rotate in place so the robot's BACK points at the FIELD point (x, y): one turnTo whose target is the bearing from the live pose estimate to (x, y) plus π, computed when this step runs — face() turned around, and the turn that goes before a reverseTo(x, y) on a tank, or on any drive whose back must lead the whole way ("back into the goal" in field words). The caller's options pass through whole (a turn in place reads the timeout and the yaw budget); turnTo(atan2(...) + π) by hand is bit-identical. (Appended at chunk L9, API 2.4.)

*function, declared at [`include/shulib/chassis/routine.hpp:556`](../../include/shulib/chassis/routine.hpp#L556).*

<a id="routine-followtrajectory"></a>

### `Routine::followTrajectory`

```cpp
Routine& followTrajectory(std::span<const math::Pose2d> waypoints, const MotionOptions& options = {})
```

Chain waypoints as sequential moveTo legs (Chassis::followTrajectory — options apply PER LEG). The full TrajectoryResult stays readable via lastTrajectory(); an incomplete trajectory fails the step.

*function, declared at [`include/shulib/chassis/routine.hpp:567`](../../include/shulib/chassis/routine.hpp#L567).*

<a id="routine-followtrajectory-2"></a>

### `Routine::followTrajectory (overload 2)`

```cpp
Routine& followTrajectory(std::initializer_list<math::Pose2d> waypoints, const MotionOptions& options = {})
```

Brace-list convenience: followTrajectory({a, b, c}).

*function, declared at [`include/shulib/chassis/routine.hpp:583`](../../include/shulib/chassis/routine.hpp#L583).*

<a id="routine-curveto"></a>

### `Routine::curveTo`

```cpp
Routine& curveTo(std::span<const CurvePoint> points, const MotionOptions& options = {})
```

Drive one SMOOTH path from where the robot is, along its nose, through `points` without stopping at them, ending like moveTo at the last (Chassis::curveTo): `{x, y}` or `{x, y, heading}`, 1 to 16 points, a heading is the robot's NOSE, never list the robot's own spot. A refused curve (first point behind the robot, a path that doubles back) or a lost one fails the step like any drive — its Warn names why. (CURVE1.)

*function, declared at [`include/shulib/chassis/routine.hpp:596`](../../include/shulib/chassis/routine.hpp#L596).*

<a id="routine-curveto-2"></a>

### `Routine::curveTo (overload 2)`

```cpp
Routine& curveTo(std::initializer_list<CurvePoint> points, const MotionOptions& options = {})
```

Brace-list convenience: r.curveTo({{24_in, 12_in}, {48_in, 24_in, 0_deg}}).

*function, declared at [`include/shulib/chassis/routine.hpp:601`](../../include/shulib/chassis/routine.hpp#L601).*

<a id="routine-reversecurveto"></a>

### `Routine::reverseCurveTo`

```cpp
Routine& reverseCurveTo(std::span<const CurvePoint> points, const MotionOptions& options = {})
```

curveTo with the robot's BACK leading (Chassis::reverseCurveTo). Every heading is still the NOSE's: backing into a spot facing 0 deg ends with `0_deg`. (CURVE1.)

*function, declared at [`include/shulib/chassis/routine.hpp:607`](../../include/shulib/chassis/routine.hpp#L607).*

<a id="routine-reversecurveto-2"></a>

### `Routine::reverseCurveTo (overload 2)`

```cpp
Routine& reverseCurveTo(std::initializer_list<CurvePoint> points, const MotionOptions& options = {})
```

Brace-list convenience: r.reverseCurveTo({{-12_in, 30_in}, {-36_in, 36_in, 0_deg}}).

*function, declared at [`include/shulib/chassis/routine.hpp:613`](../../include/shulib/chassis/routine.hpp#L613).*

<a id="routine-brake"></a>

### `Routine::brake`

```cpp
Routine& brake(const MotionOptions& options = {})
```

Controlled stop: 0 V under Brake until the estimate certifies rest (Chassis::brake — a C4 candidate adopted into F6 at D2).

*function, declared at [`include/shulib/chassis/routine.hpp:621`](../../include/shulib/chassis/routine.hpp#L621).*

<a id="routine-hold"></a>

### `Routine::hold`

```cpp
Routine& hold(units::Time duration, const MotionOptions& options = {})
```

Actively hold the current pose for `duration` (Chassis::hold). Typed time (D2): hold(300_ms) — hold(0.3) does not compile.

*function, declared at [`include/shulib/chassis/routine.hpp:627`](../../include/shulib/chassis/routine.hpp#L627).*

<a id="routine-pause"></a>

### `Routine::pause`

```cpp
Routine& pause(units::Time duration)
```

Wait, doing nothing, for `duration` — the alliance-partner beat every real auton has. Motors keep their last state (after a settled motion: stopped); the world keeps advancing. Distinct from hold(): pause() does not energize the drive. A pure delegation to Chassis::wait (D2 — before the wait verb existed, this step carried its own Tier-3 clock-deadline plumbing; that implementation moved down to the facade where both tiers get it). A pause cannot fail, logs nothing, and never stops the chain; nonsense input (NaN, <= 0) throws through untouched, exactly like every step.

*function, declared at [`include/shulib/chassis/routine.hpp:640`](../../include/shulib/chassis/routine.hpp#L640).*

<a id="routine-waitfor"></a>

### `Routine::waitFor`

```cpp
template <typename Pred> Routine& waitFor(Pred&& pred, units::Time timeout, const char* name = "waitFor")
```

Wait until `pred()` holds, up to `timeout` (required and finite — C2's no-hang discipline). In a recipe the condition MATTERS: if the deadline passes with it still false, continuing the script would act on a state the field never reached, so the chain stops (WaitTimedOut). A wait whose timeout is a legitimate strategy branch belongs one tier down: `chassis.waitUntil(...)` directly, branching on the WaitResult.

*function, declared at [`include/shulib/chassis/routine.hpp:656`](../../include/shulib/chassis/routine.hpp#L656).*

<a id="routine-then"></a>

### `Routine::then`

```cpp
template <typename Action> Routine& then(Action&& action, const char* name = "action")
```

Run an action between motions — THE MECHANISM SEAM (the one member deliberately outside F10, filled in at chunk F1). `action` is any callable taking nothing and returning * void        — the action always succeeds, * bool        — false fails the step and stops the chain, * ExitReason  — non-Settled fails the step (so an action may wrap a facade verb and have its verdict honored), * manipulation::MechanismOutcome — a mechanism operation's verdict: ONLY Succeeded continues the chain; Unconfirmed / Stalled / TimedOut / Cancelled stop it as MechanismFailed with the outcome named in the stop line. MechanismOutcome has no bool conversion, so an Unconfirmed can never be truthy by accident — the T2 guarantee that a failed grab cannot read as success at this layer. The mechanism idiom (contract in manipulation/mechanism_op.hpp):  r.then([&] { grab.start(); (void)chassis.waitUntil([&] { return grab.tick() != Running; }, Time{2.0}); return grab.outcome(); }, "grab");  RETURN THE OUTCOME. A void lambda that runs an operation and drops its verdict "succeeds" whatever happened — the same sharp edge as dropping a direct facade call's ExitReason (guide chapter 9), owned the same way. `name` labels the step in stop/skip log lines (stable literal). STEP1-A: an action that returns `ExitReason::Cancelled` or `MechanismOutcome::Cancelled` is CANCELLED, and stops the chain whatever its onFail choice (the cut rule; a `bool` false is an ordinary failure, which carryOn / skipTo may go on from).

*function, declared at [`include/shulib/chassis/routine.hpp:700`](../../include/shulib/chassis/routine.hpp#L700).*

<a id="routine-onfail"></a>

### `Routine::onFail`

```cpp
Routine& onFail(OnFail choice)
```

What the NEXT step does if it fails: `r.onFail(carryOn).driveTo(...)`, `r.onFail(skipTo("third piece")).run(...)`, `r.onFail(stopChain)` (the default, spelled out). It binds to exactly one step and is used up at that step's door — whether the step runs, is skipped after a stop, is jumped by a skip, or throws. It is not a step (the step numbers do not move). Above every choice: a CANCELLED verdict, or a guard that is running and expired, STOPS the chain (the file banner's two rules). Logs one Warn — once per Routine — when the Routine has NO run guard (a carry-on chain there cannot be stopped by the deadline), and one Warn when a skipTo's target is already BEHIND (a label this routine has passed, or the next step's own label): a skip only goes forward, so that step's failure would stop the chain. PRECONDITIONS (thrown; nothing recorded): not two onFail() in a row; a SkipTo has a non-empty target.

*function, declared at [`include/shulib/chassis/routine.hpp:747`](../../include/shulib/chassis/routine.hpp#L747).*

<a id="routine-label"></a>

### `Routine::label`

```cpp
Routine& label(const char* name)
```

Name the NEXT step, so a `skipTo("...")` before it can land there: `r.label("third piece").face(0_in, 48_in)`. Not a step of its own (the step numbers do not move); used up at that step's door. Matched whole and case-sensitively; stored, not copied — a string literal (a std::string's c_str() would dangle). Logs the "behind" Warn when an onFail(skipTo(...)) already written for the same step names this label. PRECONDITIONS (thrown; nothing recorded): not two label() in a row; `name` not null, not "".

*function, declared at [`include/shulib/chassis/routine.hpp:782`](../../include/shulib/chassis/routine.hpp#L782).*

<a id="routine-move"></a>

### `Routine::move`

```cpp
Routine& move(mechanism::PositionGoal goal, units::Time bound, const char* name = nullptr)
```

Move an arm or a lift to `goal` within `bound`, then HOLD it there (`arm["score"]`, `arm.to(120)`, `lift.to(18_in)`, `lift.clampedTo(h)`). Out of time: TIMED_OUT, and it holds where it stopped.

*function, declared at [`include/shulib/chassis/routine.hpp:848`](../../include/shulib/chassis/routine.hpp#L848).*

<a id="routine-startmove"></a>

### `Routine::startMove`

```cpp
Routine& startMove(mechanism::PositionGoal goal, units::Time bound, const char* name = nullptr)
```

move()'s START form: commands the move and returns at once; waitUntilDone() collects it.

*function, declared at [`include/shulib/chassis/routine.hpp:852`](../../include/shulib/chassis/routine.hpp#L852).*

<a id="routine-home"></a>

### `Routine::home`

```cpp
Routine& home(const mechanism::Arm& m, units::Time bound, const char* name = nullptr)
```

Find an ARM's zero by its declaration's homing method, within `bound` (then it holds there).

*function, declared at [`include/shulib/chassis/routine.hpp:856`](../../include/shulib/chassis/routine.hpp#L856).*

<a id="routine-home-2"></a>

### `Routine::home (overload 2)`

```cpp
Routine& home(const mechanism::Lift& m, units::Time bound, const char* name = nullptr)
```

Find a LIFT's zero by its declaration's homing method, within `bound`.

*function, declared at [`include/shulib/chassis/routine.hpp:860`](../../include/shulib/chassis/routine.hpp#L860).*

<a id="routine-starthome"></a>

### `Routine::startHome`

```cpp
Routine& startHome(const mechanism::Arm& m, units::Time bound, const char* name = nullptr)
```

home()'s START form, for an arm (homing while the robot drives).

*function, declared at [`include/shulib/chassis/routine.hpp:864`](../../include/shulib/chassis/routine.hpp#L864).*

<a id="routine-starthome-2"></a>

### `Routine::startHome (overload 2)`

```cpp
Routine& startHome(const mechanism::Lift& m, units::Time bound, const char* name = nullptr)
```

home()'s START form, for a lift.

*function, declared at [`include/shulib/chassis/routine.hpp:868`](../../include/shulib/chassis/routine.hpp#L868).*

<a id="routine-run"></a>

### `Routine::run`

```cpp
Routine& run(const mechanism::Power& m, units::Voltage volts, mechanism::Until until, units::Time bound, const char* name = nullptr)
```

Run a POWER mechanism at `volts` until `until` holds (seesPiece, seesPiece(grip), stopsSeeingPiece, stalls, `seesPiece.orAfter(2.5_s)`), within `bound`; its declared safe state at the end, however it ends.

*function, declared at [`include/shulib/chassis/routine.hpp:874`](../../include/shulib/chassis/routine.hpp#L874).*

<a id="routine-run-2"></a>

### `Routine::run (overload 2)`

```cpp
Routine& run(const mechanism::Power& m, units::Voltage volts, units::Time time, const char* name = nullptr)
```

Run a POWER mechanism at `volts` for `time`: SUCCEEDED when the time is up.

*function, declared at [`include/shulib/chassis/routine.hpp:879`](../../include/shulib/chassis/routine.hpp#L879).*

<a id="routine-startrun"></a>

### `Routine::startRun`

```cpp
Routine& startRun(const mechanism::Power& m, units::Voltage volts, mechanism::Until until, units::Time bound, const char* name = nullptr)
```

run()'s START form, with a done-condition.

*function, declared at [`include/shulib/chassis/routine.hpp:883`](../../include/shulib/chassis/routine.hpp#L883).*

<a id="routine-startrun-2"></a>

### `Routine::startRun (overload 2)`

```cpp
Routine& startRun(const mechanism::Power& m, units::Voltage volts, units::Time time, const char* name = nullptr)
```

run()'s START form, for a time.

*function, declared at [`include/shulib/chassis/routine.hpp:888`](../../include/shulib/chassis/routine.hpp#L888).*

<a id="routine-set"></a>

### `Routine::set`

```cpp
Routine& set(mechanism::PneumaticGoal goal, const char* name = nullptr)
```

Put a PNEUMATIC mechanism in a named position (`claw["closed"]`). Its time is the robot file's `.moveTime` (plus `.confirmWithin` when a sensor confirms it), so the step states no bound. A `grabs` position with no sensor ends UNCONFIRMED, counted done: the chain goes on.

*function, declared at [`include/shulib/chassis/routine.hpp:894`](../../include/shulib/chassis/routine.hpp#L894).*

<a id="routine-spinup"></a>

### `Routine::spinUp`

```cpp
Routine& spinUp(mechanism::SpeedGoal goal, units::Time bound, const char* name = nullptr)
```

Spin a FLYWHEEL up (`flywheel["shoot"]`, `flywheel.at(3000)`, `flywheel.clampedTo(x)`): the step SUCCEEDS at speed and the wheel KEEPS spinning until stop(), the guard or driver control.

*function, declared at [`include/shulib/chassis/routine.hpp:909`](../../include/shulib/chassis/routine.hpp#L909).*

<a id="routine-startspinup"></a>

### `Routine::startSpinUp`

```cpp
Routine& startSpinUp(mechanism::SpeedGoal goal, units::Time bound, const char* name = nullptr)
```

spinUp()'s START form (the wheel spins up while the robot drives).

*function, declared at [`include/shulib/chassis/routine.hpp:913`](../../include/shulib/chassis/routine.hpp#L913).*

<a id="routine-stop"></a>

### `Routine::stop`

```cpp
Routine& stop(const mechanism::Power& m, const char* name = nullptr)
```

Stop a POWER mechanism now — its declared safe state (a running step on it ends CANCELLED). SUCCEEDED, unless it is ABSENT this boot or ANOTHER owner holds its claim (REFUSED: the stop did nothing).

*function, declared at [`include/shulib/chassis/routine.hpp:919`](../../include/shulib/chassis/routine.hpp#L919).*

<a id="routine-stop-2"></a>

### `Routine::stop (overload 2)`

```cpp
Routine& stop(const mechanism::Flywheel& m, const char* name = nullptr)
```

Stop a FLYWHEEL now — its declared safe state. As stop(const Power&).

*function, declared at [`include/shulib/chassis/routine.hpp:933`](../../include/shulib/chassis/routine.hpp#L933).*

<a id="routine-waituntildone"></a>

### `Routine::waitUntilDone`

```cpp
template <detail::MechanismDeclaration Declared> Routine& waitUntilDone(const Declared& m, units::Time bound, const char* name = nullptr)
```

Wait (at most `bound` more) for the verdict of the step THIS routine last began on `m` — a START form's, usually; a step that already ended answers at once. A step that something OUTSIDE this routine replaced meanwhile ended CANCELLED, and so does this one. STEP1-A: when a skip JUMPED the step that would have started `m` — or that step did nothing because its goal is another robot's (a different kind or units, or a preset or position this robot's same-named mechanism lacks) — and no later step began one, the wait FAILS, counted — RoutineStopCause::MechanismNotStarted, "its start was skipped" (or "its start did nothing: another robot's declaration") — under its own onFail choice; never a throw, never an older step's verdict (mechanismVerdict stays Running, lastMechanism() is the default report). A wait never clears that mark, so a second wait answers the same. PRECONDITIONS: `bound` finite and > 0; this routine began a step on `m` (or a skip jumped one, or one did nothing as above).

*function, declared at [`include/shulib/chassis/routine.hpp:959`](../../include/shulib/chassis/routine.hpp#L959).*

<a id="routine-requires"></a>

### `Routine::requires`

```cpp
template <class D, class... Rest> requires(std::same_as<D, mechanism::Arm> || std::same_as<D, mechanism::Lift> || std::same_as<D, mechanism::Flywheel> || std::same_as<D, mechanism::Pneumatic>) Routine& run(const D&, Rest&&...)
```

`r.run(arm, ...)`: refused — run() is a power mechanism's step.

*function, declared at [`include/shulib/chassis/routine.hpp:1002`](../../include/shulib/chassis/routine.hpp#L1002).*

<a id="routine-run-3"></a>

### `Routine::run (overload 3)`

```cpp
template <class V, class... Rest> requires std::is_arithmetic_v<V> Routine& run(const mechanism::Power&, V, Rest&&...)
```

`r.run(intake, 12, 1_s)`: refused — volts carry their unit.

*function, declared at [`include/shulib/chassis/routine.hpp:1013`](../../include/shulib/chassis/routine.hpp#L1013).*

<a id="routine-requires-2"></a>

### `Routine::requires (overload 2)`

```cpp
template <class D, class... Rest> requires(std::same_as<D, mechanism::Arm> || std::same_as<D, mechanism::Lift> || std::same_as<D, mechanism::Power> || std::same_as<D, mechanism::Flywheel> || std::same_as<D, mechanism::Pneumatic>) Routine& move(const D&, Rest&&...)
```

`r.move(intake, ...)` / `r.move(arm, ...)`: refused — move() takes a TARGET.

*function, declared at [`include/shulib/chassis/routine.hpp:1020`](../../include/shulib/chassis/routine.hpp#L1020).*

<a id="routine-requires-3"></a>

### `Routine::requires (overload 3)`

```cpp
template <class D, class... Rest> requires(std::same_as<D, mechanism::Arm> || std::same_as<D, mechanism::Lift> || std::same_as<D, mechanism::Pneumatic>) Routine& stop(const D&, Rest&&...)
```

`r.stop(arm)`: refused — an arm or a lift holds, a pneumatic keeps its position.

*function, declared at [`include/shulib/chassis/routine.hpp:1031`](../../include/shulib/chassis/routine.hpp#L1031).*

<a id="routine-lastmechanism"></a>

### `Routine::lastMechanism`

```cpp
[[nodiscard]] const mechanism::StepReport& lastMechanism() const noexcept
```

The most recent mechanism step's report — its verdict, what ended it, whether an UNCONFIRMED was counted done, how long it took. Before any mechanism step: the default report (`Running`).

*function, declared at [`include/shulib/chassis/routine.hpp:1042`](../../include/shulib/chassis/routine.hpp#L1042).*

<a id="routine-triggermark"></a>

### `Routine::TriggerMark`

```cpp
template <unsigned Kinds> class [[nodiscard]] TriggerMark { public: ArmedLeg<Kinds> startMove(mechanism::PositionGoal goal, units::Time bound, const char* name = nullptr) { r_->armMove(mark_, gen_, goal, bound, name); return ArmedLeg<Kinds>{*r_, gen_}; } ArmedLeg<Kinds> startHome(const mechanism::Arm& m, units::Time bound, const char* name = nullptr) { r_->armHome(mark_, gen_, m, bound, name); return ArmedLeg<Kinds>{*r_, gen_}; } ArmedLeg<Kinds> startHome(const mechanism::Lift& m, units::Time bound, const char* name = nullptr) { r_->armHome(mark_, gen_, m, bound, name); return ArmedLeg<Kinds>{*r_, gen_}; } ArmedLeg<Kinds> startRun(const mechanism::Power& m, units::Voltage volts, mechanism::Until until, units::Time bound, const char* name = nullptr) { r_->armRun(mark_, gen_, m, volts, until, bound, name); return ArmedLeg<Kinds>{*r_, gen_}; } ArmedLeg<Kinds> startRun(const mechanism::Power& m, units::Voltage volts, units::Time time, const char* name = nullptr) { r_->armRun(mark_, gen_, m, volts, mechanism::Until::time(time), time, name); return ArmedLeg<Kinds>{*r_, gen_}; } ArmedLeg<Kinds> startSpinUp(mechanism::SpeedGoal goal, units::Time bound, const char* name = nullptr) { r_->armSpinUp(mark_, gen_, goal, bound, name); return ArmedLeg<Kinds>{*r_, gen_}; } template <class First, class... Rest> Routine& move(First&&, Rest&&...) { static_assert(detail::kTriggerRefused<First>, "a partway trigger STARTS a step: write .startMove(...) -- a blocking move cannot run " "inside a drive; collect it after the drive with r.waitUntilDone(...)"); return *r_; } template <class First, class... Rest> Routine& home(First&&, Rest&&...) { static_assert(detail::kTriggerRefused<First>, "a partway trigger STARTS a step: write .startHome(...) -- a blocking home cannot run " "inside a drive; collect it after the drive with r.waitUntilDone(...)"); return *r_; } template <class First, class... Rest> Routine& run(First&&, Rest&&...) { static_assert(detail::kTriggerRefused<First>, "a partway trigger STARTS a step: write .startRun(...) -- a blocking run cannot run " "inside a drive; collect it after the drive with r.waitUntilDone(...)"); return *r_; } template <class First, class... Rest> Routine& spinUp(First&&, Rest&&...) { static_assert(detail::kTriggerRefused<First>, "a partway trigger STARTS a step: write .startSpinUp(...) -- a blocking spinUp cannot " "run inside a drive; collect it after the drive with r.waitUntilDone(...)"); return *r_; } template <class First, class... Rest> Routine& set(First&&, Rest&&...) { static_assert(detail::kTriggerRefused<First>, "a partway trigger STARTS a step, and set() has no start form: put r.set(...) before or " "after the drive (a trigger can startMove / startHome / startRun / startSpinUp)"); return *r_; } private: friend class Routine; template <unsigned> friend class ArmedLeg; TriggerMark(Routine& r, TriggerSpec mark, std::uint32_t gen) noexcept : r_{&r}, mark_{mark}, gen_{gen} {} Routine* r_; TriggerSpec mark_; std::uint32_t gen_; }
```

STEP1-B: what atPercent() / afterDistance() / afterTurning() return — a mark waiting for its mechanism START. Its ONLY steps are the four start forms (each ARMS a trigger and returns the ArmedLeg handle); the blocking forms and set() are refused when the code is compiled. `Kinds` is the legs its mark can measure (detail::kTriggerDriveLegs / kTriggerTurnLegs). Each start form's last argument, `name`, names the TRIGGER in its lines ("its trigger 'raise' at 60 % on 'lift'"); the step those lines belong to is the drive's (a trigger counts no step of its own).

*field, declared at [`include/shulib/chassis/routine.hpp:1129`](../../include/shulib/chassis/routine.hpp#L1129).*

<a id="routine-armedleg"></a>

### `Routine::ArmedLeg`

```cpp
template <unsigned Kinds> class [[nodiscard]] ArmedLeg { public: Routine& moveTo(const math::Pose2d& target, const MotionOptions& options = {}) { static_assert((Kinds & detail::kTriggerDriveLegs) != 0, "after N degrees is a turn's mark -- a drive has no degrees turned; use afterDistance or atPercent"); return r_->armedMoveTo(gen_, target, options); } Routine& driveTo(units::Length x, units::Length y, const MotionOptions& options = {}) { static_assert((Kinds & detail::kTriggerDriveLegs) != 0, "after N degrees is a turn's mark -- a drive has no degrees turned; use afterDistance or atPercent"); return r_->armedDriveTo(gen_, "driveTo", x, y, options, false); } Routine& reverseTo(units::Length x, units::Length y, const MotionOptions& options = {}) { static_assert((Kinds & detail::kTriggerDriveLegs) != 0, "after N degrees is a turn's mark -- a drive has no degrees turned; use afterDistance or atPercent"); return r_->armedDriveTo(gen_, "reverseTo", x, y, options, true); } Routine& turnTo(math::Angle heading, const MotionOptions& options = {}) { static_assert((Kinds & detail::kTriggerTurnLegs) != 0, "after N inches is a drive's mark -- a turn has no inches; use afterTurning or atPercent"); return r_->armedTurnTo(gen_, heading, options); } Routine& face(units::Length x, units::Length y, const MotionOptions& options = {}) { static_assert((Kinds & detail::kTriggerTurnLegs) != 0, "after N inches is a drive's mark -- a turn has no inches; use afterTurning or atPercent"); return r_->armedFace(gen_, "face", x, y, options, false); } Routine& faceAway(units::Length x, units::Length y, const MotionOptions& options = {}) { static_assert((Kinds & detail::kTriggerTurnLegs) != 0, "after N inches is a drive's mark -- a turn has no inches; use afterTurning or atPercent"); return r_->armedFace(gen_, "faceAway", x, y, options, true); } [[nodiscard]] TriggerMark<Kinds> atPercent(double percent) { r_->requireLine(gen_); return TriggerMark<Kinds>{*r_, r_->percentMark(percent), r_->openMark(gen_)}; } [[nodiscard]] TriggerMark<(Kinds & detail::kTriggerDriveLegs)> afterDistance(units::Length distance) { static_assert((Kinds & detail::kTriggerDriveLegs) != 0, "after N inches is a drive's mark -- this leg already has a turn's mark (afterTurning): " "one leg is a drive OR a turn; use atPercent for a mark either can carry"); r_->requireLine(gen_); return TriggerMark<(Kinds & detail::kTriggerDriveLegs)>{*r_, r_->distanceMark(distance), r_->openMark(gen_)}; } [[nodiscard]] TriggerMark<(Kinds & detail::kTriggerTurnLegs)> afterTurning(math::Angle angle) { static_assert((Kinds & detail::kTriggerTurnLegs) != 0, "after N degrees is a turn's mark -- this leg already has a drive's mark (afterDistance): " "one leg is a drive OR a turn; use atPercent for a mark either can carry"); r_->requireLine(gen_); return TriggerMark<(Kinds & detail::kTriggerTurnLegs)>{*r_, r_->turnedMark(angle), r_->openMark(gen_)}; } private: friend class Routine; template <unsigned> friend class TriggerMark; ArmedLeg(Routine& r, std::uint32_t gen) noexcept : r_{&r}, gen_{gen} {} Routine* r_; std::uint32_t gen_; }
```

STEP1-B: what a trigger's start form returns — the leg its triggers ride, armed. Its ONLY members are the leg steps and the three arms (another trigger on the same leg; at most 4). [[nodiscard]]: a trigger with no drive after it never fires (the block's arming rules). The legs `Kinds` does not allow are refused when the code is compiled, naming the mark.

*field, declared at [`include/shulib/chassis/routine.hpp:1219`](../../include/shulib/chassis/routine.hpp#L1219).*

<a id="routine-atpercent"></a>

### `Routine::atPercent`

```cpp
[[nodiscard]] TriggerMark<detail::kTriggerAnyLeg> atPercent(double percent)
```

STEP1-B: a trigger at `percent` % of the next leg (above 0, at most 100): the share of the distance along the straight line from the leg's live start to its target — or of the angle, on a turn (the block's header, D7 (a)). Any leg. PRECONDITIONS (thrown; everything armed or pending cleared first): finite, in (0, 100]; no trigger already armed (chain a second one on the ArmedLeg handle instead).

*function, declared at [`include/shulib/chassis/routine.hpp:1293`](../../include/shulib/chassis/routine.hpp#L1293).*

<a id="routine-afterdistance"></a>

### `Routine::afterDistance`

```cpp
[[nodiscard]] TriggerMark<detail::kTriggerDriveLegs> afterDistance(units::Length distance)
```

STEP1-B: a trigger `distance` along the next leg's straight line from its live start — a DRIVE's mark (driveTo / reverseTo / moveTo; a turn after it does not compile). PRECONDITIONS: finite and > 0; no trigger already armed.

*function, declared at [`include/shulib/chassis/routine.hpp:1301`](../../include/shulib/chassis/routine.hpp#L1301).*

<a id="routine-afterturning"></a>

### `Routine::afterTurning`

```cpp
[[nodiscard]] TriggerMark<detail::kTriggerTurnLegs> afterTurning(math::Angle angle)
```

STEP1-B: a trigger once the next leg has turned `angle` toward its target — a TURN's mark (turnTo / face / faceAway; a drive after it does not compile). PRECONDITIONS: > 0 (an Angle is wrapped into (-180°, 180°], so 270_deg is -90° and refused); no trigger already armed.

*function, declared at [`include/shulib/chassis/routine.hpp:1309`](../../include/shulib/chassis/routine.hpp#L1309).*

<a id="routine-ok"></a>

### `Routine::ok`

```cpp
[[nodiscard]] bool ok() const noexcept
```

True while no step has failed (and none was skipped). (STEP1-A: a step that failed under onFail(carryOn) / skipTo counts as failed too — under the default rule failedOn is always 0, so this reads exactly as it always has.)

*function, declared at [`include/shulib/chassis/routine.hpp:1321`](../../include/shulib/chassis/routine.hpp#L1321).*

<a id="routine-result"></a>

### `Routine::result`

```cpp
[[nodiscard]] RoutineResult result() const noexcept
```

The chain verdict so far (a snapshot — see RoutineResult).

*function, declared at [`include/shulib/chassis/routine.hpp:1324`](../../include/shulib/chassis/routine.hpp#L1324).*

<a id="routine-lasttrajectory"></a>

### `Routine::lastTrajectory`

```cpp
[[nodiscard]] const TrajectoryResult& lastTrajectory() const noexcept
```

The most recent followTrajectory step's full result — completedLegs is strategy-relevant and must not be flattened away by the chain. Before any trajectory has run it reads `exit = Running`, the project's "no verdict here yet" convention (RoutineResult::exit, CompletedMotion), so succeeded() is honestly FALSE on a virgin routine. (D3: a plain value-initialized TrajectoryResult reports `Settled` with 0 of 0 legs, which succeeded() calls SUCCESS — correct for the facade, whose verb requires at least one waypoint, and a lie here, where the member exists before any trajectory does.)

*function, declared at [`include/shulib/chassis/routine.hpp:1353`](../../include/shulib/chassis/routine.hpp#L1353).*

<a id="routine-chassis"></a>

### `Routine::chassis`

```cpp
[[nodiscard]] Chassis& chassis() noexcept
```

The chassis this routine drives — the mixed-tier seam, spelled out. (You can equally keep your own reference; this exists so a routine passed across a function boundary still reaches Tier 3.)

*function, declared at [`include/shulib/chassis/routine.hpp:1360`](../../include/shulib/chassis/routine.hpp#L1360).*

<a id="routine"></a>

## `Routine`

```cpp
using chassis::Routine
```

An auton's step chain: `Routine r{ctx, "name"}`.

*type alias, declared at [`include/shulib/chassis/routine.hpp:2477`](../../include/shulib/chassis/routine.hpp#L2477).*

<a id="routineresult"></a>

## `RoutineResult`

```cpp
using chassis::RoutineResult
```

What an auton returns: `return r.result();`.

*type alias, declared at [`include/shulib/chassis/routine.hpp:2479`](../../include/shulib/chassis/routine.hpp#L2479).*

<a id="carryon-2"></a>

## `carryOn (overload 2)`

```cpp
using chassis::carryOn
```

STEP1-A: the on-failure choice's words, so a generated auton needs no namespace either — `r.onFail(carryOn)`: count the failure and run the next step.

*type alias, declared at [`include/shulib/chassis/routine.hpp:2482`](../../include/shulib/chassis/routine.hpp#L2482).*

<a id="onfail"></a>

## `OnFail`

```cpp
using chassis::OnFail
```

What `r.onFail(...)` takes.

*type alias, declared at [`include/shulib/chassis/routine.hpp:2484`](../../include/shulib/chassis/routine.hpp#L2484).*

<a id="skipto-2"></a>

## `skipTo (overload 2)`

```cpp
using chassis::skipTo
```

`r.onFail(skipTo("x"))`: count the failure and pass over every step until `r.label("x")`.

*type alias, declared at [`include/shulib/chassis/routine.hpp:2486`](../../include/shulib/chassis/routine.hpp#L2486).*

<a id="stopchain-2"></a>

## `stopChain (overload 2)`

```cpp
using chassis::stopChain
```

`r.onFail(stopChain)`: the default rule, spelled out.

*type alias, declared at [`include/shulib/chassis/routine.hpp:2488`](../../include/shulib/chassis/routine.hpp#L2488).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1">
<summary>The header’s own reasoning — 240 lines, click to expand</summary>

```text

 Routine — the Tier-2 recipe layer (chunk D1, WS12/M7 pulled forward; master
 plan §17).

 ═══ STATUS: FROZEN — F10, LOCKED 2026-08-12 (chunk D3, API 2.0) ══════════════════
 This surface is FROZEN, as its own register row (F10) rather than as part of
 F6: the facade it delegates to is a different tier and can version
 independently. Frozen: the constructor (both spellings, noexcept),
 non-copyable/non-movable, THIRTEEN steps — startAt / moveTo / driveTo /
 strafeTo / turnTo / face / followTrajectory (span + brace) / brake / hold /
 pause / waitFor, and reverseTo / faceAway, the two APPENDED at chunk L9
 (API 2.4) by exactly the additive path below and pinned the day they
 landed — the four observers ok / result / lastTrajectory / chassis,
 the types RoutineResult (all FIFTEEN fields — nine through MECH-A4, whose
 mechanismVerdict is pinned by type; six appended at STEP1-A, each pinned by
 type) and RoutineStopCause (append-only,
 existing values fixed), the documented error policy below, and typed time as
 a SEMANTIC (hold(0.3) must not compile). Enforced structurally:
 test/routine_signature_pin_test.cpp fails the build, naming F10, if any of
 those drifts. Changes only with a major API-version bump plus a migration
 note (include/shulib/version.hpp); ADDITIVE growth — new steps, new
 observers, new RoutineResult fields, appended RoutineStopCause enumerators —
 stays legal and is the intended path (F1 used it for the stop cause, L9 for
 two steps: no existing member moved either time).

 The MECHANISM STEPS (the mechanism core, chunk MECH-A4, API 2.5) took the same
 path, and what they added is in ONE delimited block below ("MECH-A4: the
 mechanism steps"): the appended stop cause MechanismStepFailed (its value
 pinned, like every enumerator's), the appended RoutineResult field
 mechanismVerdict, a SECOND constructor `Routine r{ctx, "name"}` (a new
 overload — the frozen `Routine(Chassis&, const char*)` is untouched), the
 steps move / startMove / home / startHome / run / startRun / set / spinUp /
 startSpinUp / stop / waitUntilDone, the observer lastMechanism(), and four
 refusal templates that name the right step when a team writes the wrong one.
 NOT pinned, and NOT YET RULED: whether the mechanism steps, the context
 constructor and lastMechanism() join this freeze is an OPEN decision (the
 Freeze Register's F10 row says so). Proposed at MECH-A4: not yet — every one
 of them takes the mechanism seam's types (register row F11, NOT frozen), as
 then() below does, and then() stays out of F10 for exactly that reason. The
 other reading is the additive path L9 took (its two steps pinned the day they
 landed). Until it is ruled they are unpinned: pinning them later is additive,
 un-freezing them would not be. Stated out loud for the reason the then()
 paragraph gives: silence in a freeze reads as "frozen too".

 THE ON-FAILURE CHOICE (chunk STEP1-A, 2026-11-05; the same additive path, in TWO delimited blocks
 below, "STEP1-A: the on-failure choice" — the types at namespace scope, the modifiers in the
 class — plus its state, declared last, and the policy's edits inside the existing steps, each
 marked "STEP1-A") is PINNED the day it lands, the way L9's steps were (the
 team lead's D1, 2026-11-03: a route maker generates code against it, so it must not drift): the
 two modifiers onFail(OnFail) and label(const char*), OnFail's three spellings (stopChain,
 carryOn, skipTo("x")), the appended stop cause MechanismNotStarted (== 6) and the six appended
 RoutineResult fields (failedOn, firstFailedAt, firstFailedName, firstFailedCause, jumped,
 skipPendingTo) — test/routine_signature_pin_test.cpp. The context constructor also learns the
 auton's run guard (when the context has a `guard`); it stays unpinned, as at MECH-A4.

 THE CURVE STEPS (chunk CURVE1, 2026-11; the same additive path, in ONE delimited block below,
 "CURVE1: curves") are PINNED the day they land, like L9's: curveTo / reverseCurveTo, each a span
 and a brace-list overload (four steps), each exactly one Chassis call through motionStep — a
 non-Settled curve is a MotionFailed stop (or carries on / skips under onFail), its verdict in
 RoutineResult::exit. No new stop cause: a refused start and a lost curve are motion failures
 whose Warn names them. Partway triggers (chunk STEP1-B, below) do not cover curves yet.
 PARTWAY TRIGGERS (chunk STEP1-B, 2026-11-11; the same additive path, in one delimited block in the
 class, "STEP1-B: partway triggers", plus its state, declared last, and the door's one armed check)
 are NOT pinned: the three arms atPercent / afterDistance / afterTurning and the two handles they
 return lead only to the mechanism START forms, which stay open (the team lead's ruling of
 2026-10-29; the STEP1 brief's D1). Pinning them later is additive; un-freezing them would not be.

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
 here: no gains, no kinematics, no exit criteria, no fault policy. The four
 "field vocabulary" steps — face(x, y) and driveTo(x, y), and their reversed
 twins faceAway(x, y) and reverseTo(x, y) (L9, API 2.4) — each compute ONE
 argument (the bearing from the live pose estimate to a field point — pure
 trig — plus π for the reversed pair) and then delegate to turnTo / moveTo;
 that keeps C1's D12 intact on tank drives (the AUTHOR still writes the turn
 — face() IS the author's turn, said in field words, and faceAway() is the
 same turn for backing into a point), and a hand-written turnTo(atan2(...))
 — or turnTo(atan2(...) + π) — remains bit-identical to it by construction.
 Since TANK1 (API 2.3) face() is also what makes a
 tank's driveTo leg steerable: a tank moveTo steers back onto its line only
 for a leg that starts within 15° of its target heading, and a leg right
 after face() starts within about a degree of it — the library still plans
 no turn (chassis.hpp: TANK STEERING). faceAway() does the same for a
 reverseTo leg, which starts as far off its heading as the robot's BACK is
 turned from the point: 180° from a robot facing the point (not steered),
 about 0 after driving straight out from it (steered, no faceAway() needed).
 To flow THROUGH points instead of stopping at each, a tank route writes
 curveTo / reverseCurveTo (CURVE1, the block below) — the one place the library
 steers toward a point, and only toward one on the path the author drew.
 Dropping from Tier 2 to Tier 3 is deleting the Routine object and keeping
 every verb.

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
 missed goal) writes onFail(carryOn) in front of that step (STEP1-A, below),
 or branches on the verbs directly — that is one step down the tier ladder,
 not a rewrite, and mixing the two styles in one routine is supported and
 tested (steps and direct facade calls interleave freely, because eager
 execution keeps program order = field order).

 ═══ The on-failure CHOICE (chunk STEP1-A, additive; pinned) ══════════════════════
 The policy above is the DEFAULT, unchanged bit for bit. Two modifiers, written
 in front of a step, change it for THAT step only:

     r.onFail(carryOn).driveTo(-24_in, -36_in, {.timeout = 2.5_s});
     r.onFail(skipTo("third piece")).run(intake, 12_volt, seesPiece, 2_s);
     r.label("third piece").face(0_in, 48_in);

   * onFail(carryOn): a failure is COUNTED (RoutineResult::failedOn and the
     first one's step, name and cause), Warn-logged, and the drive is safed
     exactly as a stop safes it — then the next step runs, from rest, where the
     robot is. ok() goes false; the stop fields stay clean (stoppedAt 0).
   * onFail(skipTo("x")): as carryOn, then every step until the next one
     label("x") names is PASSED OVER (counted in `steps` and `jumped`, one Info
     line each) and the auton goes on AT that step. A skip only goes FORWARD (the
     chain runs its lines as it reaches them; it never stores them): a target
     already passed — the last 32 labels are remembered, a label on a step a skip
     jumped included; one further back reads as a target that never comes — or
     the failing step's own label STOPS the chain there, said "behind it" (and Warned at the
     onFail line itself, every run, so a bench run shows it first); a target
     that never comes jumps every remaining step (RoutineResult::skipPendingTo).
   * onFail(stopChain): today's rule, spelled out. label("x") names the next
     step — it is not a step, so the step numbers do not move.
   * Each modifier binds to exactly ONE step and is used up at its door — run,
     skipped after a stop, jumped, or throwing. Two in a row, an empty or null
     name: precondition throws. Names are stored, not copied: string literals.

 TWO RULES SIT ABOVE EVERY CHOICE:
   1. THE CUT ALWAYS STOPS. A step's choice is IGNORED and the chain stops when
      its verdict is CANCELLED (a motion's exit, a mechanism step's verdict, or a
      then() action's ExitReason / MechanismOutcome — whatever cancelled it: the
      deadline, an abandon, the fault policy, a replaced step), or when the
      routine's run guard is running and expired() (the deadline passed or the
      run was abandoned); a skip still looking for its step then stops at the
      next step's door. A carry-on chain must never keep issuing steps past the
      buzzer. The guard comes from the context constructor (`Routine r{ctx,
      "x"}` in an auton); a routine with NO guard — Routine r{chassis} — keeps
      only the CANCELLED half, and Warns once at its first carryOn / skipTo.
   2. NOTHING NEW THROWS on a run-dependent path. A waitUntilDone() on a
      mechanism whose START a skip jumped answers a counted failure,
      RoutineStopCause::MechanismNotStarted ("its start was skipped"), under the
      wait's own choice — never a throw, never an older step's verdict. So does
      one whose start did nothing because its goal is another robot's (a different
      kind or units, or a preset or position this robot's same-named mechanism
      lacks) and the chain carried on past it ("its start did nothing: another
      robot's declaration").

 ═══ then(): the mechanism seam (filled at F1; still unfrozen — see above) ═══════
 then() accepts any callable returning void (always succeeds), bool, or
 ExitReason (failure stops the chain like any step) — and, since F1, a
 mechanism operation's MechanismOutcome, where ONLY Succeeded continues and
 an Unconfirmed grab can never read as success (the T2 guarantee; the member
 comment carries the idiom). One honest spelling note: the action is a
 CALLABLE, so a member function is passed as `then([&] { intake.in(); })` —
 `then(intake.in)` is valid C++ only for a callable data member, and the
 documentation spells the lambda form everywhere for that reason (T5).
 Since MECH-A4 a robot's DECLARED mechanisms need no then(): each has its own
 steps — `r.move(arm["score"], 1.5_s)`, `r.run(intake, 12_volt, seesPiece,
 3_s)` — run by the mechanism service, with their own verdict policy (the
 MECH-A4 block below). then() stays the seam for anything else.

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
