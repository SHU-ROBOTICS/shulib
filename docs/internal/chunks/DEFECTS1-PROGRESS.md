# DEFECTS1 — live progress log

> Appended continuously, in real time. Watched with `tail -f`. If this chunk is interrupted,
> this file is the honest record of exactly how far it got.
>
> **Task:** triage and resolve the 83 API defects DOCS2 reported and deliberately left in place
> (`DOCS2-API-DEFECTS.md`). Triage first — FIX / ARGUE / REJECT / DEFER — then execute.

---

## 2026-08-15 — session start

- Read `RESUMING.md`, `PROJECT-BRIEFING.md` (full), `DOCS2-COMPLETED.md`, `DOCS2-API-DEFECTS.md`
  (all 1,822 lines, all 83 items).
- `git log --oneline -5` → HEAD `d400257`, tree **clean** (0 modified files). Matches the brief's
  expected starting state.
- Position per the generated briefing block: 25 of 43 chunks complete; suite 1,121 cases /
  1,523,344 assertions / 3 skipped, green; 148 public headers; HA next free = **HA-123**.

**Plan, in order:**
1. Context gathering: Freeze Register rows, prior chunk records that may have already RULED on
   items in the list (the brief warns some of these are decisions, not defects), build state.
2. **Triage all 83 before touching code.** Each lands in exactly one of FIX / ARGUE / REJECT /
   DEFER, cited by ID, each with evidence.
3. Write the brief (`DEFECTS1-<slug>.md`) carrying the triage. Commit it.
4. Execute: small commits, one defect or one tight cluster each; every FIX gets a test that would
   have caught it; load-bearing ones mutation-proven with observed red.
5. Every behaviour change updates its `///` in the SAME commit, then regenerate `docs/api/`.
6. Annotate `DOCS2-API-DEFECTS.md` in place with each outcome.
7. All gates, both guards, ARM, release gate. Nothing pushed.

---

## Baseline verified before touching anything

```
[doctest] test cases:    1121 |    1121 passed | 0 failed | 3 skipped
[doctest] assertions: 1523344 | 1523344 passed | 0 failed |
[doctest] Status: SUCCESS!
```

Freeze Register read from `docs/roadmap.md`. Rows **F1–F5 LOCKED** (frame, accuracy, units,
**the ten HAL interface signatures**, IKinematics), **F6 LOCKED** (Chassis), **F10 LOCKED**
(Routine). F11–F14 open by design and AMENDED at DOCS2 to say documented ≠ frozen.

Note for triage: **F4's locked ten are `IClock`/`IMotor`/`IRotation`/`IImu`/`IGps`/`IDistance`/
`IOptical`/`IBattery`/`ITelemetrySink`/`IVision`+`ITagSource`.** `ILineDisplay`, `ICharSink`,
`IBlockSink`, `IDigitalOut`, `IController`, `IDigitalIn`, `IMechanism` are OUTSIDE it. That line
decides several items on its own (A9 touches `IClock` — frozen; I7 touches `ILineDisplay` — not).

## Triage fan-out launched

83 items split into 12 clusters by header/subsystem, each read against the actual code with
probes where cheap, then adversarially verified. Agents are READ-ONLY — no repo edits.
Every ruling is mine; the agents supply evidence.

| Cluster | Items |
|---|---|
| C1 hal/pros cold-start caches | D6 E5 E6 E3 A19 I9 E1 E2 |
| C2 hal/pros read semantics | D5 A16 A15 I8 A17 A18 I10 |
| C3 hal seams | A9 A11 A12 A13 A14 I7 A20 I5 D4 I6 |
| C4 diag | D2 A3 A4 A5 A6 A7 A8 E9 D3 |
| C5 control | D1 A2 I2 I3 E10 I4 E4 I1 |
| C6 scheduler | A27 A28 I19 |
| C7 motion (config/primitives) | D12 D13 D14 A31 A32 I17 |
| C8 motion (stall/geometry/context) | D15 A29 A30 I18 A1 |
| C9 localization (contracts) | D7 D8 D9 E7 I16 I14 |
| C10 localization (fusion/attribution) | A22 A23 I12 I13 I15 E8 |
| C11 math/units/kinematics/spec | D11 A26 A21 D17 I20 I21 I22 |
| C12 manipulation/sequence/tooling | A10 A24 A25 D16 D10 D18 I11 O1 |

Already suspected duplicates to confirm, not assume: **D6 ≡ E5 ≡ E6** (rotation zero-seed) and
**I9 ≡ E1** (brakeMode uncounted). And two items say in their own text that DOCS2 already fixed
them (**D3** rewrote the comment; **D10**'s root cause is the parser bug DOCS2-COMPLETED says was
fixed and pinned) — those are REJECT-as-already-resolved candidates that must be checked, not
believed.

## My own probes, run before any agent reported (so the fan-out has an independent check)

Three flagship items reproduce EXACTLY as reported, verified from scratch:

```
E9:  HealthMonitor ctor ACCEPTED brownoutRecoverVolts=+Inf
I3:  TrapezoidProfile ctor ACCEPTED maxAcceleration=inf; sample(0).accel=inf finite=0
E10: sample(NaN) pos=nan vel=nan accel=-20.000000 accelFinite=1 isDone(NaN)=0
```

`A14` also reproduces and is nastier than it reads: `ProsDigitalOut oops(1, 2);` compiles
**clean under every one of the project's strict flags** (`-Wall -Wextra -Wpedantic -Wshadow
-Wconversion -Wsign-conversion -Wdouble-promotion`), selecting the brain-ADI 2-arg constructor
with `adiPort=1, initialState=(bool)2=true`. Construction is a physical action, so that fires a
solenoid HIGH at boot on the wrong port. Every existing call site passes a real `bool`, so a
deleted non-bool overload would break none of them.

**And the first REJECT, found by probe rather than by argument — `A9`.** Its evidence makes two
claims about `IClock` and the code refutes both:

```
A9: after `ra = rb` : a.now()=5.000000 b.now()=9.000000  (sizeof IClock = 8, the vptr alone)
$ g++ -fsyntax-only  'IClock c = f;'
error: cannot allocate an object of abstract type 'shulib::hal::IClock'
```

`IClock c = someProsClock;` does **not** compile — the class is abstract. And the assignment that
does compile discards **nothing**, because the `IClock` subobject is stateless. On top of that the
header already carries the deliberate ruling: *"the defaulted copy/move set restores the move
operations that declaring a destructor suppresses, so a concrete clock stays movable. The
interface is abstract and stateless — there is no IClock value to copy."*

## Gate state at HEAD, before any edit

`self-test` `check-coverage` `check-fresh` `check-examples` `check-removability` — all **PASS**.
`briefing_status.py check` — **FAIL**, and correctly: creating `DEFECTS1-PROGRESS.md` with no
matching `-COMPLETED.md` makes the block report an interrupted chunk. That stays red for the
length of this chunk by design and closes when the completion record lands.

## I21 measured rather than argued — and the measurement kills the obvious fix

`I21` says the angle literals are the only ones in `units/literals.hpp` that are not `constexpr`.
True. The obvious fix is to make `Angle::radians`/`degrees`/`wrapRad` constexpr. Two things had
to be checked before that could be called safe, and I checked both:

1. **Does a `SHULIB_PRECONDITION` block constexpr?** No. The macro is a ternary, and the
   handler-slot call sits in the untaken branch, so a valid literal never reaches it. A probe
   with a constexpr factory guarded by `SHULIB_PRECONDITION(std::isfinite(v), …)` compiles and
   constant-evaluates. So the check would NOT have to be dropped.
2. **Does `wrapRad` survive constant evaluation?** It calls `std::remainder`, which is not
   constexpr in C++20. GCC accepts it as a builtin extension — **both** host `g++` and
   `arm-none-eabi-g++` compiled a constexpr `wrapRad` clean. **clang does not:**

   ```
   error: constexpr variable 'k' must be initialized by a constant expression
   note: non-constexpr function 'remainder' cannot be used in a constant expression
   ```

So the cheap fix makes constant evaluation of the angle vocabulary **GCC-only**, in a public
repo whose stated point is that teams outside SHU can use it — and clang is installed here and
was DOCS2's independent parser oracle. The only non-GCC-only route is re-deriving the wrap
arithmetic by hand, inside **LOCKED register row F3**, whose exact-180° → +π case is pinned by a
red-on-failure test. That is a decision, not an edit. **I21 → ARGUE**, with this measurement as
the reason.

## More independent verification, while the fan-out runs

Everything below is mine, run against HEAD, before reading any agent's report.

**A2 / A21 reproduce, and they are the SAME NaN path seen at two points:**
```
A2:  compensateForBattery(NaN,12) -> voltage=nan isnan=1 brownoutLimited=0
A21: maxMagnitude of {NaN,10,10,10} = 10.000000
A21: desaturateUniform -> out[0]=nan isnan=1 out[1]=10.000000
```
Reading the pipeline end to end changes what the right fix is. `plausibility_guard.hpp` states
the design out loud: the volt path is **FiniteGuard-shaped** — *"non-finite → Implausible + 0 V"*
— and recovery happens at the **motor edge** (`recoverWheelVoltage`, invariant 3), never at the
math helpers, on the principle that *"a diagnostic that mutates the data path is worse than the
bug it hunts"*. So the obvious fix (a throwing `SHULIB_PRECONDITION` on `desired`) is **wrong
twice over**: it contradicts that layering, and it would convert an A3-recoverable hostile-sensor
pathology into a thrown `PreconditionError` → `FAULT_ABORT`, which is the opposite of A1's
"faults log and recover, never crash".

That rules out the loud fix and leaves a much better one for **A2**. The flag is the actual lie:
`std::abs(d) > b` is **false** for NaN, so the struct reports "this value is inside the battery
envelope" about a value that is not a value. Changing it to `!(std::abs(d) <= b)` makes it true
for NaN, leaves **every finite path bit-identical**, and — importantly — does *not* swallow the
NaN, so the Implausible fault still fires downstream where it is supposed to. All four existing
assertions in `test/feedforward_test.cpp` keep passing by construction; none of them passes NaN.

For **A21** the same reasoning says the code is right and the *banner* is over-claimed:
desaturate is not "the last-line guarantee" — `recoverWheelVoltage` is. Enforcing finiteness
inside desaturate would touch **LOCKED register row F5** (whose contract names desaturate) *and*
break the recover-don't-abort rule.

**A24 / A25 collide with a deliberate, TESTED decision — and there is a fix that keeps it.**
`test/mechanism_op_test.cpp` SUBCASE *"completed verdict is PRESERVED — cancel still re-safes"*
asserts exactly the behaviour A24 calls a defect. But that test's scenario has **nobody else
holding the claim**. Guarding on `holdsClaim_ || !mech_->claimed()` closes A24/A25's hole (a
stale operation can no longer safe a mechanism a live operation owns) while leaving that test
passing unchanged. `claimed()` is already public on `IMechanism`.

**I8** confirmed by eye: `mutable units::Length lastDistance_{9999.0 / 25.4};` hardcodes both the
sentinel and the mm→inch factor, fifteen lines under `kDistanceNoObjectMm` and
`distanceMmToCanonical()` in the same class. One-line FIX.

**A1, A10, A27, A28, E7, A15 confirmed — and every one of them is already documented as a
defect in its own `///`.** That is the trap the brief named, and it is now measured rather than
anticipated: `robot_context.hpp` says *"checked only for emptiness, never against the kinematics'
wheel count"*; `mechanism.hpp` says *"A copied mechanism therefore arrives already claimed()"*;
`motion_scheduler.hpp` says *"Destruction is DEFAULTED and does not cancel"* and *"beginMotion()
does NOT clear it, so between motions it still holds the PREVIOUS motion's target"*;
`localizer.hpp` says *"`minDt` is NOT checked, and nothing checks `minDt <= maxDt`"*;
`distance.hpp` says *"Reads are LIVE … two samples, not one atomic snapshot"*. Every one of those
sentences has to be rewritten in the same commit as its fix.

**I19 is already three-quarters closed by DOCS2.** `lastCompleted()` and `completedCount()` both
carry the disagreement in writing; only `lastExitReason()` fails to name `completedCount()` as
the discriminator. And a *code* fix is not available: `lastExit_ = Settled` is what makes
`waitUntilSettled()` vacuously correct with nothing active, which is F6-documented semantics.

## A finding the list does not contain, found by checking D2's arithmetic instead of reading it

`D2` says `diag/controller_display.hpp`'s banner claims the fault-name column width is *"checked
by static math here"* while the file contains no `static_assert`, and warns that *"a 16-char
future code would silently truncate"*.

The check does not exist. **And if it did, it would be red today.** Measured against the real
`faultCodeName()` and the real `kCols`:

```
kCols=19, budget for the name after "flt " = 15
  GPS_GATE_REJECT      15
  MOTOR_OVER_TEMP      15
  MECHANISM_STALLED    17   <-- OVERFLOWS row 1
```

`MECHANISM_STALLED` was **appended at F1** and is 17 characters. The banner's "the longest fault
spellings … 15 chars" has been false since that day, and on a real robot a jammed intake paints
`flt MECHANISM_STALL` on the driver's controller — the seam truncates rather than wraps, which is
its documented behaviour, so nothing is broken except the sentence that says it cannot happen.

D2's hypothetical future 16-char code already arrived, at 17, and nobody noticed because the
promised check was never written. That is the same shape as DOCS2's own trap 6: *a gate that has
only ever run on the easy case has not been tested; it has been lucky* — except here the gate was
never written at all and the banner asserted its result.

## Already RESOLVED at HEAD — verified in the generated pages, not assumed

Four items say in their own text that they were fixed or handed over during DOCS2. All four
check out, so all four are REJECT-as-resolved:

- **D3** — `loop_monitor.hpp` now reads *"Largest dt observed since construction"*; the false
  "/reset" is gone.
- **D7 / D10 / I1** — the `///<`-continuation mis-attribution. `_strip_doc` at HEAD explicitly
  refuses a `///<` line and there is a `_continuation()` helper. The published pages are correct:
  `MechanismOutcome::Unconfirmed` and `::TimedOut` each carry their own full sentence,
  `ExitReason::Cancelled` keeps the *"never returned by ExitGroup::check()"* clause, and
  `FusionResult::audit` / `appliedConfidence` / `headingNudge` each carry their own.
- **I11 / O1** — `LogLevel`, `Localizer::Quality` and `BrakeMode` are all reflowed at HEAD to one
  enumerator per line with a `///` block each. `check-coverage` passes over the whole tree, which
  it could not if these were still one-liners. `TrackingWheel::Role` is two enumerators on two
  lines with one `///<` each, which the parser handles correctly.

## A8's obvious fix would have broken the scheduler — the bypass has an in-tree user

`A8` says `TickAttribution::PhaseScope`'s public constructor makes `phase()`'s tick-open
precondition "advisory", and proposes making the constructor private with a `friend`. Grepping
for the type before believing that:

```
include/shulib/motion/motion_scheduler.hpp:938:
    return std::optional<diag::TickAttribution::PhaseScope>{std::in_place, *att_, p};
```

**The scheduler is the bypass's only user, and it is deliberate**: `PhaseScope` is non-movable, so
the checked factory's by-value return cannot be stored in the `std::optional` the scheduler needs
(attribution is optional and must cost nothing when off). Privatising the constructor with only
`TickAttribution` as a friend would fail to compile at that line; friending `MotionScheduler`
from `diag/` would both invert the layering and hand the one real bypasser a permanent exemption.

The fix that actually closes it: give `TickAttribution` a **checked in-place factory** returning
`std::optional<PhaseScope>`, point the scheduler at that, and *then* privatise the constructor
with `TickAttribution` as the only friend. The three scheduler call sites (:974, :1013, :1021)
all sit inside `tickImpl()` under `AttributionTickGuard`, so a tick is always open there and the
added check cannot fire spuriously. `phase()` stays as it is for the ten test call sites.

## The cold-start family (D6/E5/E6) is REAL, and its stated failure scenario is NOT

This is the family the brief calls out first, so I checked its mechanism rather than its
conclusion. Two of its load-bearing claims do not survive the grep.

**Claim 1 — *"`velocity()` is the sharp one: a constant 0.0 rad/s is literally the 'the robot
stopped' reading."*** `IRotation`'s only consumer in the entire library is
`localization/tracking_wheel.hpp`, and it reads `position()` at three sites and `velocity()` at
none:

```
include/shulib/localization/tracking_wheel.hpp:74:  const double shaft = sensor_.position().value();
include/shulib/localization/tracking_wheel.hpp:88:  lastShaft_ = sensor_.position().value();
include/shulib/localization/tracking_wheel.hpp:95:  lastShaft_ = sensor_.position().value();
```
A repo-wide search for `IRotation` returns five files: the seam, the conversion helper, the PROS
adapter, the fake, and `TrackingWheel`. **`IRotation::velocity()` has no consumer at all.** The
sharpest sentence in the finding is about a value nothing reads.

**Claim 2 — *"ODO_STUCK is built to notice a FROZEN NON-ZERO value, and a frozen ZERO makes a
dead pod look like a stationary robot instead of a visible fault."*** `OdoStallCheck::update()`
compares **drive-motor** shaft deltas against **fused-pose** deltas:
`sumAbsShaftDelta += std::abs(motors[i]->position().value() - shaftBase_[i])` against
`fusedPose - base`. It never reads the rotation pod, and it works in **deltas** — so a pod frozen
at 0 and a pod frozen at 2000 rad produce the *identical* observable: zero pose travel while the
drive shafts turn. The check trips in both cases. The zero is not the thing that hides it.

**So the disposition is FIX, but for the reason the header itself gives rather than the reason
the finding gives.** The banner states a T7 policy — *"hold the last good value, never propagate,
never zero"* — and the implementation misses it in the cold-start window. A promise a class
makes about itself and does not keep is a defect whether or not the consequence is the dramatic
one. **What must NOT happen is the new comment repeating the over-claim**, which would be DOCS1's
`hal/battery.hpp` failure again: a banner teaching a model the code does not implement.

## Ruling the cold-start family — and why I do NOT follow the triage agent to REJECT

The C1 agent reached the same two refutations I did, independently, and rejected D6/E5/E6 on
them. I am overruling that to **FIX**, and the reason matters.

The finding's *headline* is a contradiction between two things in the tree, and the contradiction
is real: the banner says *"never zero"*, the caches say `{0.0}`. What the probes refute is the
finding's *explanation of the harm*. Those are different claims, and only one of them is wrong.

Worse, **the wrong explanation is in the shipped header**, in the SEED CAVEAT DOCS2 added:

> *"…publishes 0 rad / 0 rad/s — the precise reading the screen above claims to prevent, and the
> one the ODO_STUCK cross-check reads as a stopped robot."*

That last clause is false, and it is republished verbatim on `docs/api/rotation.md`. The same
over-claim is repeated in `DOCS2-COMPLETED.md`. So REJECT would close the item while leaving a
factually wrong published sentence in place — the exact `hal/battery.hpp` failure DOCS1 caught:
**a banner teaching a model the code does not implement.**

The right fix is therefore to the *promise*, not the seed: no seed value is distinguishable
downstream (TrackingWheel reads deltas; `velocity()` has no consumer), so changing `{0.0}` to
anything else changes no observable in the library. Saying so honestly is worth more than a
change that moves a number nothing reads.

**`E3` (optical) is the same shape with a real harm, and it separates cleanly.** `IOptical` has
**zero consumers outside `hal/`** — nothing in motion, localization or manipulation reads it —
but unlike rotation its channels are ABSOLUTE, not deltas, and its banner's reasoning is sound
on its own terms: *"hue 0.0 IS a color — red — so a zeroed failure would read as a confident
wrong answer."* There is no honest finite seed (NaN is forbidden by F4 at this seam), so the
promise has to be scoped rather than implemented, and the validity decision belongs to **F3**,
the chunk that writes the first consumer.

## Full baseline at HEAD, before a single edit

```
GUARD1 PASS   (PROS-free outside hal/pros/, path-anchored)
GUARD2 PASS   (core is sim-free)
ARM GATE PASS (148 headers, one TU, -Werror)
RELEASE GATE PASS  (prepare_site.py)
doc gates: self-test / check-coverage / check-fresh / check-examples / check-removability — PASS
suite: 1,121 cases · 1,523,344 assertions · 3 skipped — green
```
Anything red after this point is mine.

---

# EXECUTION

## Commit 1 — A28 + A27 (the scheduler)

**The test caught a real flaw in my own fix, which is the whole point of writing it.**
My first `~MotionScheduler()` called the full `cancel()`, mirroring F2's `WaitUnwindGuard`.
It **SIGABRTed**:

```
test/motion_scheduler_unwind_test.cpp:195: FATAL ERROR: test case CRASHED: SIGABRT
```

Motions live on the caller's stack for exactly the scheduled window, and the idiom that
*creates* A28 — construct scheduler, construct motion, leave the scope — destroys them in
**reverse**, so `active_` dangles by the time the destructor runs. `cancel()` calls
`active_->cancel()` through it. The correct destructor commands `applyCancelSafeState()`
directly and records **no** boundary, because recording one honestly requires reading an
object that may no longer exist. That constraint is now written into the header.

Two traps hit while proving it, both already on the list:

- **L2 / D3's ordering hazard fired immediately.** The first build after a `///` edit failed
  at `check-fresh`, not at the compiler — the doc gate runs first and names the wrong problem.
- **C4's stale-binary trap tried to hand me a green mutation.** My first mutation added a line,
  which moved every declaration line number, so `check-fresh` failed, the binary was never
  relinked, and running it reported the OLD green result. Gating on build success is what
  caught it. Every mutation after that was made **line-count-neutral** so the generated
  reference stays byte-identical and only the behaviour changes.

**Mutation campaign — 2/2 RED, both observed.**

| # | Mutation | Result |
|---|---|---|
| M1 | destructor's `applyCancelSafeState` made unreachable (`if (false)`) | **RED** — 1 case, 8 assertions |
| M2 | `beginMotion()`'s two pose resets removed | **RED** — 1 case, 2 assertions, and it is the A27 case by name |

Restored, rebuilt, **1,124 cases / 1,523,372 assertions / 3 skipped — green** (1,121 + 3 new).

The A28 test carries its negative control inside it: it `REQUIRE`s the drive is genuinely
energized (`> 1.0 V`) at the moment of destruction, so the post-destruction zero cannot come
from a scenario that never commanded anything.

## Commit 2 — the finiteness family (E9, I3, E10, D13, A2, I2)

Six items, one shape: a non-finite value that constructs or flows where `> 0.0` was mistaken
for a check. **`> 0.0` is satisfied by infinity**, and that single fact is behind four of them.

The interesting ruling here is the one I did **not** make. `A2`/`A21` both look like they want
a throwing `SHULIB_PRECONDITION`, and `plausibility_guard.hpp` says why that would be wrong:
the volt path is FiniteGuard-shaped, recovery lives at the **motor edge**
(`recoverWheelVoltage`, invariant 3), and *"a diagnostic that mutates the data path is worse
than the bug it hunts."* A throw there converts an A3 hostile-sensor pathology into an aborted
motion. So A2's fix is the **flag**, not the value: `!(|d| <= b)` instead of `|d| > b`, which
lands NaN on the true side, leaves every finite path bit-identical, and still lets the NaN
reach the guard that is supposed to raise on it.

`E10` needed a signature change and it is recorded rather than slipped in: **`isDone()` drops
`noexcept`**, because the precondition handler throws and a noexcept frame turns a caller bug
into `std::terminate`. version.hpp calls a noexcept change breaking; taken here because the
class has no consumer in the tree but its own test, and a follower loop spinning forever on a
NaN clock is the worse failure. Goes in the changelog.

`D13` is fixed at **both** layers — `MotionConfig::validate()` and `Watchdog`'s own constructor
— because the second is the one that makes the "a motion can never hang" claim.

**Mutation campaign — 5/5 RED, every one observed:**

| # | Mutation | Result |
|---|---|---|
| M3 | `isfinite` dropped from `brownoutRecoverVolts` | **RED** 1 case |
| M4 | `isfinite` dropped from `maxAcceleration` | **RED** 1 case |
| M5 | `sample()`'s finiteness precondition removed | **RED** 1 case |
| M6 | flag reverted to the NaN-blind `\|d\| > b` | **RED** 1 case |
| M7 | `isfinite` dropped from `defaultTimeout` | **RED** 1 case |

Restored: **1,131 cases / 1,523,404 assertions / 3 skipped — green.**

## Commit 3 — A1 + A30 (unguarded spans on the commanding path)

`A1` is the sharpest thing on the whole list: `applyCommandPipeline` indexes the drive-motor
span by **wheel** index with `std::span::operator[]`, which is unchecked, and nothing anywhere
compared the motor count to `IKinematics::wheelCount()`. Three motors with an XDrive installed
read one past the end **every tick**. What makes it a real defect rather than a hypothetical is
the asymmetry the finder spotted: every RECORD-producing loop in the tree already guards it
(`i < motors.size() && …`) — only the path that actually drives the robot did not.

The check goes in `MotionDeps::validate()`, which is the one bundle holding both the context
and the kinematics, so it fires at a motion's construction rather than three ticks into an
auton. `RobotContext` cannot make it (it has no kinematics) and `IKinematics` cannot (it has no
motors); that is exactly why it was missing.

`A30`'s empty-span half is the same family: with no motors the mean shaft delta is 0, so
`spinTravel >= minSpinTravel` is never true and the stall check reports **healthy forever**.

**Mutations 2/2 RED:** M8 (the wheel-count cross-check made vacuous) and M9 (the empty-span
precondition removed) each redden exactly one case. Restored green at **1,133 / 1,523,412 / 3**.

## Commit 4 — the claim token becomes an ownership token (A24, A25, A10)

**L4 was live here and the fix had to respect it.** `test/mechanism_op_test.cpp` already
contains SUBCASE *"completed verdict is PRESERVED — cancel still re-safes"*, which asserts
exactly the behaviour A24 calls a defect. That test is not wrong: in its scenario **nobody
else holds the claim**, and re-safing is right. So the guard is `holdsClaim_ || !claimed()`
rather than `holdsClaim_` — it closes the hole (a live owner is never disturbed) and leaves
the tested decision standing. Both pass, and a third new case pins the preserved half
explicitly so the fix can never be read as "cancel stopped re-safing".

`A10` deleted copy/move on `IMechanism`. Nothing in the tree copied one, so the change is
free — and `manipulation/mechanism_op.hpp` already deletes copy/move on both operations for
the mirror-image reason, so this is the seam finally matching a rule it had already written
down for its other half.

**Mutations 2/2 RED:** reverting either guard to unconditional reddens exactly one case.
**1,137 / 1,523,433 / 3 — green.**

## Commit 5 — diag (A3, A4, A5, A6, A7, A8, D2)

**`line_format.hpp` had no test file at all.** That is how A5 and A6 survived: the header was
covered only incidentally, through renderers that call it with the shapes they happen to use —
and both defects live exactly where no shipped renderer goes yet (a column wider than 10, a
line within two bytes of capacity). `test/line_format_test.cpp` is new.

**A MUTATION STAYED GREEN, and it was the most useful result in this commit.** M13 removed the
partial-ellipsis guard and my first A5 test did not notice. The reason is worth recording: I
had filled the line to `kCapacity - 2`, which leaves **zero** bytes free after the sanitized
text lands, and `appendRaw` with zero room writes nothing at all — so no partial marker ever
appeared. The bug needs **1 or 2** bytes free *after* the copy. Refilled to `kCapacity - 4`
with a 2-character window, and M13b is **RED**. A test that passes against the unfixed code is
not a test, and only the mutation could tell me which one I had written.

`D2` is the item that grew. The banner promised static math that did not exist; writing it
showed the arithmetic had been **wrong since chunk F1** appended `MECHANISM_STALLED` (17 chars
against a 15-char budget). Shortening a §18.4 spelling would change every TermSink line and run
summary that carries it, so the check asserts the property the display actually needs — that a
**truncated** row still names exactly one code — and it is self-extending: a second
`static_assert` proves the scan reaches past the last named enumerator, so appending a code
beyond the bound is itself a compile error.

`A8` needed the **passkey idiom**, twice over. A private constructor plus `friend` does not
work, because `std::optional`'s in-place construction does the constructing and cannot be a
friend; and the passkey TYPE has to be public (only its constructor private) or
`TickAttribution` cannot name it. The scheduler — the bypass's only user — now goes through
`phaseInPlace()` and gets the tick-open check it was skipping.

**Mutations 4/4 RED** (M12 A6, M13b A5, M14 A4, M15b A7). M15 needed `docs/api` regenerated
alongside it, because changing a default member initializer changes a *rendered signature* and
`check-fresh` blocks the build otherwise.

**1,144 / 1,523,852 / 3 — green.**

## Commit 6 — hal/pros (A11, A12, A14, D5, E1/I9, E2, I8, I10)

`A14` is the one worth reading twice. The verifier sharpened it: `ProsDigitalOut d(1, 2)`
compiles clean **only through parentheses** — brace initialisation already rejected it as a
narrowing `int → bool`. So the hole was a punctuation choice. Closed with a deleted template
overload, which is a compile-time fact and therefore asserted as one (`static_assert`), with a
negative control asserting both real constructors still work.

`D5`'s test took two attempts and the first one taught me the class. Setting an offset after
construction does nothing, because `verifyOffset` short-circuits once `offsetVerified_` is set
at boot. The *only* route to `offsetRejected_` is: unreadable at construction (so the boot
check defers rather than throwing), then readable and non-zero afterwards — a device another
program configured, discovered late. That is the scenario the header's HA-06 discussion is
about, and now the scenario the test uses.

`E2` turned out sharper than reported. The finding said the ctor leaves brake mode inherited;
the consequence nobody had stated is that `brakeMode_` is **also** the T7 fallback, so a port
left in Hold by a previous session and dying before any `setBrakeMode()` reported **Coast
forever** — the fallback contradicting the device from boot. Seeded from the device now.

`A11` is FIX-as-documentation: `flush()` already returns the answer a destructor cannot, so the
honest close is to say which failure is unreportable and point at the member that reports it.

**Mutations 3/3 RED** (M16 I9, M17 E2, M18 D5). A14 is compile-time; A12/I8/I10 are covered by
the tests above and by existing adapter tests. **1,149 / 1,523,868 / 3 — green.**

## Commit 7 — localization (E7, I14, I15/E8, I16, A22, A23, N1)

**Two of my own fixes were wrong before the tests corrected them, and both are worth recording.**

**(1) `travelDelta()` is a CONSUMING read.** My first N1 gate called it a second time to test
the delta it had just integrated — and a second call returns 0, so the gate was reading zeros
and could never fire. Caught by reading the header, not by the compiler. The deltas are now
hoisted and read exactly once, with a comment saying why.

**(2) The first N1 bound was too tight AND the wrong policy.** 12 in rejected a legitimate E3
fixture that moves 20 in in one tick, and freezing the pose broke it outright. The deeper
problem the test surfaced: `PilonsOdometry` **has no clock**, so a per-tick travel bound is
**dt-blind** — 12 in is 100 ft/s on a 10 ms tick and 5 ft/s on a 200 ms one. A gate that can
silently stop odometry accumulating on a slow loop is a worse failure than the jump it
prevents, and `plausibility_guard.hpp` already rules on exactly this: *"a diagnostic that
mutates the data path is worse than the bug it hunts."*

So N1 lands as a **visibility fix, and it is labelled as one**: the bound is generous (36 in,
unreachable at any plausible loop rate), the delta is **reported, not withheld** — the same
policy `maxTickRotation` has always had — and the phantom jump goes from *completely silent* to
raising `lastDeltaImplausible()`, which `HealthMonitor` turns into a fault. Preventing it needs
a validity channel the F4 seam does not have. Registered as **HA-123**, invented, with the
dt-blindness and the settling measurement written into the entry.

`I16`'s retype did what a retype should: it **broke two call sites at compile time**, both
passing bare doubles into a radian field.

`I15`/`E8` could not be fixed by picking a better name — `FusionResult` returns no index
identifying which proposal won. So a single proposer is attributed and several report
`"multiple"`, which is honest, where it used to print corrector[0]'s NAME beside
corrector[1]'s CONFIDENCE. A faithful per-source split is an API change and is written up.

**Mutations 2/2 RED** (M19 E7, M20b N1). M20's first spelling failed to build on
`-Werror=unused-variable`, which is the mutation runner catching itself.
**1,151 / 1,523,877 / 3 — green.**

## Commit 8 — the remaining code fixes + the doc sweep, and TWO HONEST RETRACTIONS

**`I13` is retracted and re-triaged FIX → DEFER (R4).** I applied the obvious fix — gate
`applied` on `o.dPos > 0` — and it reddened two E4 tests. They were right and I was wrong: an
accepted fix with **zero innovation** legitimately moves no position while still shrinking the
covariance, so `dPos` is not a proxy for "the update did something". The honest fix needs a
`moved` flag set where the clamp computes its scale, inside EKF internals that E4 sized against
invented noise and R4 re-measures. The defect is now written into the branch where it lives,
with the failed proxy and its reason, rather than left as a confident wrong fix.

**`D12`/`D14` are fixed but NOT test-proven, and mutation M21 stayed GREEN.** Two attempts to
build the boot-window scenario failed the same way: `MotionRig`'s localizer is seeded live in
its constructor, so neither a long `bootSettleTime` nor holding the IMU un-ready keeps
`qualityClass()` at `Uninitialized` past the old 1.5 s budget. My first test there checked that
a freshly constructed `HoldPose` had not exited — true of every `HoldPose` — and M21 is what
told me so. The test is removed and replaced by a comment naming the gap, because a test that
passes for the wrong reason is worse than none.

The doc sweep closed the sixteen items where the sentence IS the defect: `D1` `D4` `D6` `D8`
`D9` `D11` `D15` `D16` `D17` `D18` `I5` `I6` `I7` `I17` `I22` `A21`. Two are worth naming:
`D6` finally caveats the **F4 interface header**, which DOCS2 never touched while caveating both
adapter sites; and `D15`'s "halves the mean" is corrected to `(n-1)/n`, which makes the bullet's
own conclusion *stronger* than the wrong figure implied.
