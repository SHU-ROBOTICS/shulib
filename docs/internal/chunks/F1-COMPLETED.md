# F1 — the mechanism seam — completion record

> Completion record for [`F1-mechanism-seam.md`](F1-mechanism-seam.md), executed with the
> coordinator's mid-chunk corrections (which override the brief where they conflict — both are
> reflected below). Live log: [`F1-PROGRESS.md`](F1-PROGRESS.md).
> Opens Phase F; chunk 20 of 40. **The first new seam since C1.**
>
> **Do not commit** was the instruction; everything below is in the working tree.

---

## 1. What this chunk actually did

Before this chunk the library could drive to a point and could not *do* anything on arrival.
Now it has the layer every scoring verb will be built from: a device seam for mechanisms
(motor groups and pneumatic lines, each with a **declared, per-mechanism safe state**), a
bounded-operation contract above it (tickable, cancellable, watchdog-bounded, with a verdict
that can say **Unconfirmed**), hostile fakes that jam, stall and lie, and the `then()`
integration D3 left the door open for. **Nothing here has met a robot**, and nothing about
any accuracy claim moved.

| | |
|---|---|
| Suite before | 944 cases / 1,521,691 assertions *(first F1 build; pre-F1 was 915 / 1,521,419)* |
| **Suite after** | **961 cases / 1,521,812 assertions**, 3 skipped (unchanged), all green |
| New cases | **46** (four new files + guide-09d + pins) |
| Mutations | **18 executed, 18 red, 0 green, 0 build-fail, 0 skipped** (§9) |
| CI guards | GUARD1 (PROS-free) PASS · GUARD2 (no `sim/` in core) PASS |
| ARM gate | PASS, **123 headers** (was 115) |
| Doc gates | check-coverage · check-fresh · check-examples · check-removability · self-test — all PASS |
| New invented constants | **HA-92, HA-93**, both registered, cited in-tree |
| API version | **2.0 → 2.1** — the first exercise of the documented additive path (§8) |
| Freezes | **none — deliberately, and the register now says so out loud (row F11)** |

### Files

| File | What |
|---|---|
| `include/shulib/hal/digital_out.hpp` | **new** — `IDigitalOut`, the first non-motor actuation seam (no feedback, and why that is honest) |
| `include/shulib/hal/mechanism.hpp` | **new** — `IMechanism` (minimal force-safe surface + claim token), `MotorMechanism`, `PneumaticMechanism` |
| `include/shulib/hal/fake/fake_digital_out.hpp` | **new** — deterministic fake (commanded + set count) |
| `include/shulib/hal/fake/fake_mechanism.hpp` | **new** — a no-device `IMechanism` (the H2 proof; F2's park-guard double) |
| `include/shulib/manipulation/mechanism_outcome.hpp` | **new** — the verdict vocabulary (T2) |
| `include/shulib/manipulation/stall_detector.hpp` | **new** — the jam decision; thresholds REQUIRED, no defaults |
| `include/shulib/manipulation/mechanism_op.hpp` | **new** — `MechanismDeps`, `IMechanismOp`, `RunUntilConfirmed`, `ActuateAndConfirm`, `AlwaysConfirmed` |
| `include/shulib/sim/hostile/mechanism_hostility.hpp` | **new** — `JammedMotor`, `LyingSpinMotor`, `NeverConfirm`, `ConfirmAfter` (A3 pattern) |
| `include/shulib/diag/fault.hpp` | *modified* — appended `MechanismStalled = 11` |
| `include/shulib/chassis/routine.hpp` | *modified* — `then()` (the ONE unfrozen member) + appended `RoutineStopCause::MechanismFailed`; private `recordStop` gains a detail arg; banner truths updated |
| `include/shulib/diag/debug_record.hpp` | *modified* — `TickPhase::User` comment records the T6 ruling (comment only, no wire change) |
| `include/shulib/version.hpp` | *modified* — 2.1 (§8) |
| `test/mechanism_test.cpp` / `_op_test.cpp` / `_hostile_test.cpp` / `_routine_test.cpp` + `test/mechanism_test_rig.hpp` | **new** — the four suites + the shared rig (`RecordingMotor`, the tick-time convention) |
| `test/guide_examples_test.cpp` | *modified* — `guide-09d`, the compiled chapter-9 idiom |
| `test/f6_signature_pin_test.cpp`, `test/routine_signature_pin_test.cpp` | *modified* — the D2/D3 version-pin defect fixed (§8) + the appended-enumerator pin |
| `test/fault_test.cpp`, `test/blackbox_format_test.cpp` | *modified* — value/name pins + the E1 blackbox round trip for code 11 |
| `docs/internal/verify/verify-f1.sh` | **new** — the mutation harness |

**Deliberately untouched:** every pinned member of `chassis.hpp` and `routine.hpp` (both pin
suites green; the only pin edits are the version-pin fix, which is §8's finding, and the
sanctioned append pin), `RobotContext` (T7), `applyCancelSafeState`, `IMotion`, the scheduler,
and every existing test's meaning — the recipe/cookbook/guide suites run unedited except for
the additive guide-09d and its mapping comment.

---

## 2. T1 — what a "Mechanism" is (F2, F3, G1, G2, H2 and R1 inherit this)

**RULING: two levels, and an interface that is deliberately almost empty.**

**Level 1 — `hal::IMechanism`, the device seam.** Its entire virtual surface is
`applySafeState()` and `name()`. That is the answer to the brief's anti-abstraction
challenge, taken seriously: the ONE operation every mechanism supports uniformly — motors or
air — is *"put yourself in your declared safe state, now, synchronously."* F2's park guard
must walk a heterogeneous list of mechanisms it knows nothing about and force each safe at
the buzzer; that requires a common base, and **nothing else does**. Command-and-read surfaces
are NOT unified because the physics is not unified — a voltage command on a solenoid would be
a lie, and a "command a double" abstraction over both would be a worse one. Code that
commands a mechanism holds the concrete type (`MotorMechanism` — fan-out, device readback,
`maxCurrent`/`meanVelocity`; `PneumaticMechanism` — line fan-out, declared safe command);
code that only needs "make it safe" holds `IMechanism*`.

The base also carries the **one-operation claim token** (concrete, non-virtual, so no
implementation can get it wrong) — T3's structural answer, below.

*Rejected — a fat `IMechanism` with `setCommand(double)`/`read()`.* A vtable and a document
without a capability; it bakes the motor shape into the seam the pneumatic clamp then has to
fit through. *Rejected — no interface at all (concrete types only).* Then F2's park guard
cannot exist without knowing every mechanism type in the program, i.e. the library would need
to know the season's nouns, which is the C6-catalogued legacy failure. The interface is
exactly as large as the park guard's need and no larger.

**Level 2 — `manipulation/`, the bounded operation.** `IMechanismOp` +
`RunUntilConfirmed`/`ActuateAndConfirm`, season-free by construction: **what confirms is a
caller-supplied predicate** (hue for a Toggle, proximity for a cup — meaning lives above the
seam, the `hal/vision.hpp` opaque-`classId` house rule applied to actions). The scope line
against F2: F1 owns *one bounded action on one mechanism*, including time-awareness (the
watchdog, the actuation/confirm phases, the stall window) because "not hangs" and "detects a
jam" are per-operation properties; F2 owns *composition* — anything that coordinates two
operations, a motion and an operation, or a clock deadline across steps. Nothing in
`manipulation/` can express `Sequence`, `Race`, or a match timer, and that is the fence.

**A layering ruling the master plan needed:** §6 places `manipulation/` in the "Skills layer"
above the facade. That placement describes **F3's concrete primitives** (which may consume
facade verbs). The F1 *contract* layer depends only on `hal/` + `control/` + `diag/` and sits
beside `motion/` (L2) — which is what lets `routine.hpp` (L3) include the outcome vocabulary
without a layering inversion. The master plan §6 now carries this note.

**Out of scope, held:** no game semantics anywhere in `hal/` or `manipulation/` — no hues, no
"capture", no season nouns. Confirmed by grep: the only mechanism words in the library are
grammar words.

---

## 3. T2 — the verdict vocabulary (eases or worsens D1's problem; here is which)

**RULING: a separate `manipulation::MechanismOutcome` — the fourth result vocabulary — with
the mapping into the Routine layer owned and tested by the library.**

`{Running=0, Succeeded=1, Unconfirmed=2, TimedOut=3, Cancelled=4, Stalled=5}`, explicit
values, append-only. The value that forces the new vocabulary is **Unconfirmed**: *the
operation completed and the thing did not happen* — jaws closed on schedule, on a healthy
mechanism, on nothing. Not `TimedOut` (nothing timed out), emphatically not `Settled`.

*Rejected — append `Unconfirmed` to `ExitReason`.* Then `Chassis::moveTo` could
syntactically return it, which it never can; a vocabulary that admits impossible values makes
every exhaustive switch over it a lie. *Rejected — `ExitReason` + a separate `confirmed()`
flag.* The cheapest shape and the most dangerous: a caller that reads the verdict and not the
flag sees `Settled` on a failed grab — precisely the silent success the error policy exists
to prevent.

**The three guarantees that make Unconfirmed impossible to mistake for success at the
Routine layer**, each structural rather than documented:

1. `MechanismOutcome` is a scoped enum with **no bool conversion** — `if (outcome)` does not
   compile (pinned by `static_assert` in the suite). `Unconfirmed == 2` can never be truthy.
2. `then()` detects the type and maps **only `Succeeded`** to success; every other value —
   including `Running`, the tell of an operation nobody drove to completion — stops the chain
   as the appended `RoutineStopCause::MechanismFailed`, with the exact verdict named in the
   Warn line. Mutation 9 (mapping inverted) and mutation 6 (Unconfirmed → Succeeded at the
   source) both go red.
3. The residual authoring hole is stated rather than hidden: a `void` lambda that runs an
   operation and drops its verdict "succeeds" — the same sharp edge as a dropped direct-call
   `ExitReason`, documented in the same breath (guide ch. 9, the header, the cookbook note).

**For F2, what this eases and what it costs.** Costs: there are now FOUR result vocabularies
(`ExitReason`, `TrajectoryResult`, `WaitResult`, `MechanismOutcome`) and F2's combinators
span all of them — D1 §2.7's warning got heavier by one, deliberately, because each
vocabulary stays honest. Eases: `MechanismOutcome` **is** a candidate shape for D1's
"shared step-outcome vocabulary below the recipe layer" — `Running/Succeeded/TimedOut/
Cancelled` are the common step outcomes and `Unconfirmed/Stalled` are the mechanism-specific
extension. **F1 deliberately did not unify project-wide** (the brief forbade it; F2 has more
evidence). If F2 builds a step-outcome type, map into it; do not re-mean anything here.

---

## 4. The finding that gets its own section: success must not un-actuate

**Found by hand-writing the expected timeline before the code ran** — the trap countermeasure
doing its work at design time rather than in a test. The draft contract said "every exit
applies the declared safe state," mirrored from the motor case. Writing the
`ActuateAndConfirm` success timeline exposed it: a clamp whose declared safe command is
*open* would **fling its goal the instant the grab SUCCEEDED.**

The motor rationale does not transfer. A motor left energized is a hazard — energy keeps
flowing, so every exit must stop it, and Hold-at-0 V *preserves* a lift's position while
stopping the input. A solenoid consumes no energy holding a state; its commanded value IS the
completed action, and un-commanding it on completion would undo the very act the operation
exists to perform.

**RULING — the safe-state application splits by actuation physics:**

- **Motor operations** apply the declared safe state on **every** exit — Succeeded, Stalled,
  TimedOut, Cancelled. "Capture, and the intake stops; arrive, and the lift holds." Mutation
  17 (exit without safe state) turns nine cases red.
- **Discrete operations** leave the commanded state in place on **Succeeded and Unconfirmed**
  (the completed action persists; an unconfirmed one stays put, loudly reported, for the
  caller's retry/undo decision) and apply the declared safe state **only on `cancel()`** —
  the outside hammer, which is exactly the park-guard path, where the *declared* value is how
  a team says "clamp keeps the goal at the buzzer" vs "cylinder retracts inside the expansion
  limit." Mutation 15 (success applies safe state) turns five cases red.

Had this shipped as first drafted, it would have been discovered on a field, as a robot
opening its clamp every time a grab worked. The reviewer's checkpoint note independently
flagged this class of finding as record-worthy; it is the chunk's best argument for the
write-the-timeline-first rule.

---

## 5. T4 — the declared safe state (a physical claim nobody can check until there is a robot)

**RULING: the safe state is DECLARED per mechanism, at construction, and applied by
construction on every stop path.** `MotorMechanism` declares a `BrakeMode`; that is the motor
form. `PneumaticMechanism` declares a `bool` line command; that is the discrete form — the
reviewer's 3(d) is answered by generalizing the *concept* (a declared safe disposition) rather
than forcing `BrakeMode` onto air, and the test bar's "assert at motor level" has its exact
analog at `FakeDigitalOut::commanded()`.

Why per-mechanism, restated so it survives: **there is no library-wide default that is safe.**
A loaded lift at 0 V + Coast drops its stack (its declaration is Hold); a jammed intake at
Hold sits at stall current toward the ~55 °C thermal fault (its declaration is Coast or
Brake); a clamp's safe line state is a per-robot strategy fact with both polarities
legitimate. Any single default is wrong for half the robot silently, at the buzzer.
*Rejected — copy `applyCancelSafeState` (0 V + Brake for all).* That is the drivetrain's
answer to a drivetrain's question. *Rejected — a safe-state parameter on each operation call.*
The declaration belongs where the physics is known once (construction), not re-argued at
every call site; and the park guard cannot know call-site answers.

Application ordering is pinned at the device: brake mode BEFORE zero volts (mutation 14 —
the swap is a momentary coast under load — is killed by an event-order assertion on
`RecordingMotor`, which exists because voltage/mode snapshots cannot see ordering).

**The physical claim, registered:** whether `BrakeMode::Hold` at 0 V actually holds a
*loaded cascade lift* against back-drive is **HA-92** — invented, with its settling
measurement (lift on a stand, cancel under load, measure sag) and its blast radius (a parked
lift descends at the buzzer) written down. The library's provable half is narrower and is
what the tests pin: the *declaration reaches the physical motor* on every stop path, through
hostile wrappers included. The jam-signature magnitudes the hostile fake invents are
**HA-93**. Stall *thresholds* are required parameters with no library defaults — C2's
no-invented-default-constant rule — so there is no HA entry for them: there is nothing to
register until F3 chooses per-mechanism values and R4 measures them.

**The park-guard question (reviewer's 3(c)), answered so T4's promise is backed:** the guard
iterates a **caller-supplied `span<hal::IMechanism*>`**. The team lists its mechanisms where
it constructs them; the library never holds the list (T7); `IMechanism` exists precisely so
that span can be heterogeneous. The shape is already demonstrated in
`test/mechanism_test.cpp` ("a heterogeneous span can be forced safe uniformly") and stated in
guide ch. 13 so teams build the list from day one. F2 takes the span; nothing else is needed.

---

## 6. T3 — who ticks, who blocks, who arbitrates

**RULING: tick-only.** An operation owns no loop, no task, no cadence — `IMotion`'s rule,
applied unchanged. The loop owner ticks it; the **blocking idiom is the existing loop owner**:

    op.start();
    (void)chassis.waitUntil([&] { return op.tick() != MechanismOutcome::Running; }, Time{2.0});

There is deliberately **no blocking helper in the mechanism layer**. The idiom inherits every
C2 guard for free (required finite timeout, stalled-pace precondition, no-blocking-verbs-in-
predicates), it runs concurrently with an active motion, and a second loop owner that could
deadlock against the first never exists. This also answers the reviewer's 3(b) — `then()` is
a one-shot invoke, and the thing that loops is the chassis's own wait *inside* the action;
an operation returned to `then()` still `Running` (nobody drove it) stops the chain loudly
with `RUNNING` named, tested.

**The concurrency proof** (the test F2 inherits): a `MoveToPose` driving under the scheduler
while a `RunUntilConfirmed` ticks in the `waitUntil` predicate — no precondition trip, no
double tick (integer-exact arithmetic: the op's 41st tick lands on the 41st predicate call
after exactly 40 motion ticks), both progress, and the motion's final pose is **bit-identical**
(`==` on doubles) to a mechanism-free twin. Also proven: a mechanism *stall* mid-motion
latches its fault and the drive keeps driving and settles (§7's abort-mask consequence).

**Arbitration:** two mechanisms at once — required, works. Two operations on ONE mechanism —
a collision, made structural by the claim token: `start()` takes the claim or throws the
precondition naming the fix ("cancel it first — F2's pre-empt policy"); every exit releases
it. *Rejected — pre-empt-then-replace at this layer* (C2's policy): pre-emption is policy and
belongs to the policy owner; F2 composes it as cancel-then-start, which the claim's release
makes exactly one line. Un-forbidding later is additive; a silent double-drive can never be
walked back. Direct device commands (`mech.setVoltage`) are deliberately NOT claim-gated —
the same standing decision as C2's "direct Tier-3 IMotion use remains possible."

**Structurally prevented (reviewer's 3(e)):** `IMechanismOp` does not derive from `IMotion`,
so `scheduler.async(op)` — which would pre-empt and cancel the active *drive* motion — does
not compile. Pinned by `static_assert` in the suite.

---

## 7. T6 — faults vs verdicts, and `TickPhase::User`

**RULING: one code minted — `MechanismStalled = 11` ("MECHANISM_STALLED").** Stall-grade
current with a shaft that will not turn, held past the persistence window, is the robot being
unwell — a jam, a bind, a motor cooking. It is raised beside the `Stalled` verdict (both
asserted independently — E1's hole class is mutation 7, red), value-pinned, name-pinned, and
round-tripped through E1's blackbox decoder, bytes out and back.

**No `MechanismTimeout` code exists, and that is the ruling, not an omission.** The brief's
DoD reads "failure modes surface as fault codes, not hangs"; the honest narrowing is that a
mechanism operation's watchdog firing is *routinely the world declining to cooperate* — "spin
until a ring arrives, 3 s budget" timing out on a healthy, unjammed intake means no ring
came. That is strategy, and latching it would flood first-fault triage with normal outcomes,
which destroys the thing the latch exists for. The precedent followed is `waitUntil`'s
no-fault timeout, and the precedent *rejected* is `MotionTimeout` — deliberately, with the
distinction argued in the header banner: **a motion has authority over its own success** (the
drive not arriving means the robot is unwell or the estimate lies), **a mechanism's success
depends on the world**. The genuinely-unwell timeout (stuck but not stalling) is
indistinguishable from a missing ring at this layer; F3's primitives, which know what should
have happened, may mint sharper codes when they can prove them. Timeout and Unconfirmed each
log one Warn line ("MECH") — visible, not latched. Cancel and success are silent.

**The abort-mask consequence, confirmed deliberately:** code 11 lands on the
continue-degraded side of the default mask (ODO_STUCK only), so **a jammed intake does not
abort a drive** — tested end to end (the motion settles; `lastCompleted().abortFault ==
None`). A team that wants abort-on-jam sets the bit; that is C2's policy knob working as
designed. Latch capacity: value 11 fits the 32 tally slots and the `uint32_t` mask with room.

**`TickPhase::User` stays empty, with the producer named — not populated.** The ruling and
its reasoning, recorded because E1's lesson (a field nothing fills is worse than none) cuts
the other way here: the only place mechanism work is visible to the record producer today is
a `waitUntil` predicate, which runs **outside** the D-3 attribution bracket
(`beginTick`…`complete`), and crediting it would break the *pinned* sum contract (attributed
phases never exceed the tick total on the same clock). Widening the bracket means reshaping
C5's instrument for a phase whose real producer does not exist yet. The named producer is
**F2's sequencer loop** (a first-class loop owner that can bracket user work properly) and
**G2's marker dispatch** — recorded in the enum's own comment, in build-order's F1 entry, and
here. *Rejected — credit the predicate anyway* (breaks a pinned invariant to fill a slot);
*rejected — silence* (that is how `DebugRecord::fault` sat producerless from A1 to E1).

---

## 8. Two defects in earlier chunks, found and fixed here (Rule 4)

**8a. The D2/D3 version pins fenced off the additive path they were guarding** *(surfaced by
the coordinator's red-team as a blocking correction; the fix and its recording are this
chunk's).* `version.hpp` documents appended enumerators as ADDITIVE — "bump `kApiMinor` …
the intended growth path of every frozen surface" — while both freeze-pin suites hard-asserted
`kApiMinor == 0`. The first legal additive change would (and did) turn both pins red: the
pins conflated "F6/F10 are API 2.x and the version mechanism exists" (what D2/D3 meant) with
"the version is exactly 2.0" (what they wrote). **Fixed in the pins** — they now assert
`kApiMajor == 2 && kApiMinor >= 0`, each carrying a HISTORY comment so the conflation cannot
be quietly re-introduced — and exercised immediately: **API 2.0 → 2.1** for F1's additive set
(`FaultCode::MechanismStalled`, `RoutineStopCause::MechanismFailed`, `then()`'s fourth
accepted return type), reasoning recorded in `version.hpp` itself. No frozen member changed
shape; both pin suites are green; a pin firing on a *signature* line still means stop.

**8b. Guide chapter 14 claimed no cookbook existed — stale since D3 shipped one.** The "No
recipe cookbook or generated API reference site yet (D3)" bullet was written at C8 and never
updated when D3 landed both. Fixed to the true split (both exist and are build-checked;
*nothing is hosted*), with a correction note in place, because prose that admits it went
stale teaches the next reader to look.

**And one defect in the brief itself, recorded as the brief invites:** its T5 analysis found
one of the two breaks in the flagship line (`intake.in` is not a call) and missed the larger
one D1 had already ruled on (`chassis.moveTo(p)` returns `ExitReason`, which has no
`.then()` — chains belong to `Routine`). The coordinator's correction supplied the second
break; both are now stated wherever the line was quoted.

---

## 9. T5 — the flagship line, ruled: fix the prose (and it was broken twice)

**RULING: the prose is corrected everywhere; no code bends to the marketing line.** The
option "make `intake.in` literally true" (mechanisms hand out callable data members) was
rejected on three grounds: the library cannot make *team-authored* structs follow that
convention, so the flagship would only be true for code written in a special style — a trap
wearing a feature's clothes; a callable member must capture a loop owner and a timeout at
construction, coupling every mechanism struct to the chassis; and the second break makes the
point moot — **no mechanism change can give an `ExitReason` a `.then()`**. Per the
correction, no `Chassis` facade change was made or proposed.

The honest spelling — a `Routine` chain, with the operation idiom inside `then()` — is now a
**compiled, gate-checked example** (`guide-09d`, quoted verbatim in ch. 9; the
`check-examples` gate binds them line by line), which is stronger than the eight inline-prose
quotes ever were: no compiler had ever seen any of them. Every quoted place is corrected with
the double break stated: master plan §17 (the tier cell), guide ch. 9, cookbook README +
01-getting-there (`intake.release` claim; the gate-bound `Intake` struct itself untouched),
roadmap lines 214/1147/1153 (the M7 recipe-API item moves `[~] → [x]` — its stated blocker
"`.then(intake.in)` cannot exist until mechanisms do" is resolved, in both directions: the
mechanisms exist, and the spelling that could never exist is corrected), `routine.hpp`'s
banner, and the regenerated `docs/api/routine.md`. `then()`'s change is additive; every
existing use — cookbook, guide, recipe tests — runs unedited with unchanged meaning (the
legacy void/bool/ExitReason forms are re-pinned in the new suite as well).

---

## 10. T7 — ownership, and the freeze note

**RULING: `RobotContext` does not grow.** No `ctx.intake()`, no mechanism list in the
library — a fixed accessor set is this season baked into the API, the exact legacy failure C6
catalogued (five hard-coded ids, zero executors). The library owns the seam; **the team owns
the mechanisms** (constructed beside the context, listed as a span for F2's guard); G2's
registry owns id→action binding. Operations take `MechanismDeps` — named pointers
{clock, faults, telemetry} + `validate()`, the MotionDeps pattern, deliberately smaller (no
localizer: an operation never reads the estimate; no health monitor: the loop owner ticks it).
*Rejected — deps = `RobotContext*`:* hands every operation the drive motors and the IMU it
must never touch.

**G1 check, with a named handoff:** `RobotBuilder.from(profile)` can wire
`MotorMechanism`/`PneumaticMechanism` straight from `mechanisms:[{name, motors:[...],
pneumatics:[port]}]` — ports→devices, name→name — **except the declared safe state, which the
F7 schema does not carry and must** (T4: no default is safe; G1 must not invent one). The F7
schema needs a per-mechanism `safe:` field — additive, F7 is unfrozen (🎯). Recorded here and
in the F11 row's dependents column so G1 does not rediscover it.

**Freeze note (D2's lesson, discharged):** F1 freezes **nothing**, and the register now says
so out loud — **row F11**, marked 🚫 not-frozen/🚧 open by design, naming what will freeze it
(F3, the second consumer, on hardware — the same build → second consumer → freeze path F6 and
F10 took) and listing its dependents. The F4 row's locked cell is untouched; the master
plan's F4 scope note now records the "deferred additive" as landed, *outside* the freeze.

**The api-doc gate ruling (the brief's `TARGETS` question), decided deliberately:** the new
headers do **not** join `TARGETS` at F1. D3's warning — "a gate's exclusion list is where its
holes live" — is the strong counterargument and is answered head-on: the coverage gate exists
to hold *frozen* surfaces to their documentation, and gating an explicitly-unfrozen surface
pins generated reference pages to shapes F3 is expected to stress and possibly reshape —
churning committed generated docs at every adjustment teaches people to rubber-stamp the
gate, which is a worse hole than the one deferred. The exclusion is *visible* in three
places (the F11 row, this record, build-order's F1 entry), it has a **named owner and
deadline** (the F3/freeze chunk adds the headers to `TARGETS` as part of locking F11), and
the headers meanwhile carry full doc comments written to the gate's standard so joining is an
edit, not a writing project. What D3's lesson actually condemns is a *silent* exclusion; this
one is loud.

---

## 11. Test evidence

**46 new cases across four suites + the rig + guide-09d.** Every one names, in a comment,
the bug it would catch.

### The trap, in its F1 form, and the counters actually used

The fake and the operation never share a notion of "done": every timeline is hand-written
from the contract as integer tick literals (tick N commanded, tick N+k confirmed, verdict at
tick N+k) and asserted against those literals; confirmation predicates are the *test's* own
counters or clock thresholds, never a fake's completion flag; and every load-bearing
disposition is asserted at the bottom of the stack (`FakeMotor::brakeMode()` /
`commandedVoltage()` / `FakeDigitalOut::commanded()`, through hostile wrappers where the
scenario has them), never on the mechanism's or operation's self-report. `RecordingMotor`
exists because ordering claims (mode-before-volts) and never-happened claims (true-on-entry
spins nothing) cannot be seen in state snapshots at all.

**The fake can lie, and the suite uses it:** `LyingSpinMotor` reports healthy velocity and
current over a truly stopped shaft (truth kept on the frozen position channel) — the stall
detector correctly never trips, no false fault appears, and the watchdog exits at its exact
hand-computed tick with **every sensor dishonest**: the no-hang guarantee provably depends on
no sensor honesty. `ConfirmAfter{clock, 0}` is the confirm that lies *true* — over a jammed
world it produces a false `Succeeded`, asserted **as the documented trust boundary** (the
predicate is the operation's only eye on the task; F3 must confirm on real sensors) so that
if anyone ever teaches the op to second-guess its confirmation, a test forces that to be a
deliberate, argued change.

**Two timeline errors were caught by the discipline itself, before or at first run** — both
recorded in the live log as lessons: decimal dt accumulates FP dust that shifts an
ideal-arithmetic tick literal by one (fix: binary-exact `dt = 2⁻⁷` everywhere a literal
matters — the suite's one red of the whole chunk), and the jam current scaling with the
commanded voltage means tick 0 (nothing commanded yet) reads 0 A, landing the stalled-lift
trip at tick 6, not 5 (caught by re-derivation before the run).

### What the suites hold

- **Device seam** (`mechanism_test.cpp`, 9 cases): fan-out to every device; readback from the
  device (the clamp makes request ≠ applied); mode-before-volts as event order; **the T4
  asymmetry** — lift ends Hold, intake ends Coast, from identical calls; max-current /
  mean-velocity group semantics (a mean current hides a one-motor jam — argued in the case);
  the claim token; both pneumatic safe polarities; **the F2 park-guard shape** — one
  heterogeneous `span<IMechanism*>`, one verb, every declared state reached, including a
  device-free `FakeMechanism` (the H2 proof); loud construction preconditions.
- **Operation contract** (`mechanism_op_test.cpp`, 16 cases): the DoD timeline (confirmed at
  tick 8 exactly, commanded at the device every Running tick, silent on success);
  true-on-entry succeeds with zero energizing events; never-confirm exits TimedOut at tick 50
  with **no fault** and one Warn; the watchdog runs from `start()` (a dawdling caller is
  inside the same budget); **the adversarial clock** — zero-dt stretches, a 0.4 s jump
  mid-schedule, and a 10⁶ s leap, each exiting at its hand-computed tick; jam → Stalled at
  tick 5 with the latch asserted independently; spin-up transients and interrupted windows
  (reset, not paused — trip at 10, not 5); **confirm-beats-stall on the same tick** (§9's
  planning find); the cancel contract clause by clause (running / idempotent-and-re-safing /
  completed-verdict-preserved / never-started-untouched); finished-is-inert (not one device
  event after exit) and full re-arm; the claim collision (loud, non-disturbing, cleared on
  exit); the `ActuateAndConfirm` timelines — Unconfirmed at tick 50 with the predicate
  provably never consulted during actuation (21 calls, ticks 30–50), success mid-window at
  its exact tick **with the grip kept**, the eager-confirm suppression (exactly one consult,
  at the deadline), success priority at the window edge, the one-tick deploy, the
  jump-past-both-deadlines bound, cancel-forces-safe; construction preconditions; the two
  `static_assert` pins (no `IMotion` inheritance; no bool conversion) and the outcome
  value/name pins.
- **Hostile suite** (`mechanism_hostile_test.cpp`, 8 cases): the jam **liveness pin** (the
  window measurably bites, only inside itself, current scaling with command, position frozen
  at first in-window read — mutation 8's killer); jammed intake end-to-end (Stalled at tick
  37 through the wrapper, safed at the *inner* motor); stalled lift (tick 6, ends **Hold**);
  the lying device (above); the unconfirmed-grab world (dead air is invisible at the device —
  by honest design — so the verdict carries it); late confirm inside the window at its exact
  tick; the lying-true confirm boundary; **composition + ablation** — nested jam windows, a
  hand table of five probe instants, ablation differing exactly where the removed window was.
- **Integration** (`mechanism_routine_test.cpp`, 6 cases): the T3 concurrency proof
  (bit-identity, integer tick arithmetic); the abort-mask consequence (stall does not abort
  the drive; the drive settles; `abortFault == None`); the `then()` idiom end-to-end
  (Succeeded continues, 3/3 steps); **UNCONFIRMED stops the chain** — `ok()` false, cause
  `MechanismFailed`, step/skip counts exact, the transcript naming both the class and the
  verdict, the drive parked at the plant's motors, no fault; STALLED through the recipe path
  (fault latched + chain stopped + intake safed); the still-`Running` author error (loud
  stop, `RUNNING` named); the legacy then() forms unchanged in meaning.
- **Wire/pins:** `MechanismStalled` value + name pins; the E1 blackbox round trip carries
  code 11 in the full GateReason × FaultCode sweep; `RoutineStopCause::MechanismFailed == 4`
  pinned in the F10 pin file via the sanctioned append; `guide-09d` compiles and runs the
  chapter-9 listing in both worlds (confirmed → whole chain ok; never-confirmed → stopped,
  skipped, intake safed unasked).

---

## 12. Mutations

`docs/internal/verify/verify-f1.sh` — cloned from verify-e4.sh with every inherited hardening
(backup-restore never via git; INT/TERM/PIPE trap; byte-compare so an edit that changed
nothing cannot score; build-gated so a non-compiling mutation can never read green; loud
non-zero exit on any SKIP; never piped into `head`).

**18 executed: 18 RED, 0 GREEN, 0 build-fail, 0 skipped** (final campaign; log preserved in
the session scratchpad and summarized in the live log). The named eight are numbered 1–8 and
ran first: watchdog disarmed · cancel skips the safe state · cancel overwrites a completed
verdict · the safe-state declaration ignored in **both** directions (everyone-Coasts drops
the lift, everyone-Holds cooks the intake — two mutations, killed by different cases, which
is the asymmetry being real) · stall detector always healthy · Unconfirmed reports Succeeded ·
the fault raise dropped with the verdict kept (E1's hole class) · the hostile jam injection
made a no-op. Beyond the named: then()'s mapping inverted · the claim precondition defeated ·
confirm consulted during actuation · **stall checked before confirm** · the discrete deadline
pair disarmed · mode/volts order swapped · the T4 split regressed (success un-grabs) · the
claim leaked on exit · motor exits skip the safe state entirely.

**One process catch, C4's class, recorded:** mutation 8's first form (`return false;`)
BUILD-FAILED — the orphaned `now` variable tripped `-Werror` — and a build-fail proves
nothing, so it was re-formed to compile (`return now < 0.0;`, still never true) and the whole
campaign re-run: RED, 4 cases. The harness comment carries the lesson at the mutation itself.

**The campaign's one hole was found by *planning* it, and closed before it ran.** Designing
the confirm/stall order swap (mutation 12) against the then-existing suite showed **no case
could kill it**: every stall scenario had no confirmation and every confirmation no stall.
The order is load-bearing physics, not style — for a motorized clamp the current spike IS the
capture signature, so a grab routinely stalls *into* its own confirmation, and stall-first
would report a completed grab as a jam. Closed with the same-tick-success-wins case (its
comment tells this story), after which mutation 12 runs red by exactly that one case —
a sole-detector kill, the D1 pattern. Every chunk so far has found at least one hole; F1's
was found a phase earlier than usual (before the campaign executed), which is where the
project should want them found.

---

## 13. What a mechanism can and cannot do now — and what F2 may assume

**F2 will be read against this section.**

**A mechanism CAN, today, on a host, against fakes:** be composed from motors or solenoid
lines behind portable seams; declare its own safe state once and have every stop path apply
it provably down to the device; run one bounded operation at a time (structurally); spin
until a caller-defined confirmation with jam detection that must earn its verdict from
current + shaft speed; actuate-and-confirm with the pre-actuation world suppressed and
`Unconfirmed` as a first-class, unmistakable-for-success verdict; tick inside the scheduler's
wait concurrently with a live motion without perturbing it by one bit; fail loudly into a
recipe chain with the exact verdict named; and survive a device whose every sensor lies,
exiting on the watchdog at the predicted tick.

**A mechanism CANNOT, and nothing here claims otherwise:** score anything (no concrete verb
exists — F3/F′, needing hardware and the build team's still-open mechanism decisions #2/#3);
run on a robot (no `hal/pros` `IMotor`-group or ADI adapter — R1); know that Hold holds a
loaded lift (HA-92 — a guess with a settling procedure); know a real jam's current signature
(HA-93; thresholds are per-mechanism parameters with no defaults on purpose); be composed
with other operations or bounded against a match clock (F2); be fired from a path marker id
(G2) or wired from a `.vexbot` profile (G1 — which needs F7 to grow a `safe:` field, §10);
or attribute its tick time (`TickPhase::User` — F2/G2 own the producer, §7).

**What F2 may assume, by name:**
1. **The park guard's dependency exists:** take a caller-supplied `span<hal::IMechanism*>`
   and call `applySafeState()` on each — synchronous, idempotent, callable regardless of what
   any operation is doing, declared-state-correct per mechanism, proven heterogeneous
   including device-free implementations. Cancelling the operations first is polite but not
   required for device safety; it IS required to keep them from re-commanding on a later tick
   (they are inert only after exit/cancel).
2. **Pre-empt is cancel-then-start**, and it is one line: `old.cancel()` releases the claim
   synchronously and preserves history; `new.start()` claims. A `Sequence`/`Race` can hold
   `IMechanismOp&` handles — start/tick/cancel/outcome/started/name/finished is the whole
   contract, and it deliberately cannot be handed to `MotionScheduler::async`.
3. **Ticking shape:** one `op.tick()` per loop iteration, from a predicate or F2's own loop;
   post-exit ticks are safe no-ops; `start()` fully re-arms. The four-vocabulary situation
   (§3) is F2's to survey before it builds combinator verdicts — read D1 §2.7 and §3 here
   together, in that order.
4. **Fault posture:** only `MechanismStalled` latches, and it does not abort motions under
   the default mask. A sequencer deciding "retry on Stalled, move on on TimedOut, branch on
   Unconfirmed" reads the operation's outcome, not the latch.
5. **Nothing is frozen** (F11): if F2's combinators need the seam reshaped, that is a normal
   pre-freeze design conversation, not a migration.

---

## 14. Documentation contract — discharged

Guide **ch. 9** rewritten around the compiled `guide-09d` idiom with both T5 breaks stated in
plain words; **ch. 13** gained "Extension 3: building a mechanism" (the layer map updated;
the park-guard span stated as a practical instruction; the trust boundary stated); **ch. 14**'s
placeholder section replaced in the measured → measured-against → unmeasured order, leading
with what is proven against fakes and keeping "nothing has met a robot" explicit — plus the
§8b stale-prose fix. **Cookbook** README + 01-getting-there reworded (the gate-bound `Intake`
struct untouched; the false `intake.release` literal replaced by the lambda spelling it
already used two lines up). **Master plan**: §17 tier cell corrected with the double break
recorded; §6 module-map status note (including the layering ruling); the F4 scope note now
records the landed sibling. **Roadmap**: the F1 you-are-here entry (leading, as E4's does,
with what did NOT happen); row **F11**; M4's WS7 line `[~]` with evidence and its two honest
reasons; M7's recipe-API item `[x]` with the correction named; the D1 paragraph's stale
`.then(intake.in)` fixed. **HA register**: HA-92, HA-93, index rows, reconciliation clean
both ways (verified by grep, §15). **build-order**: Current position + the F1 entry's RULED
block. **Guide README** maintenance ledger: the F1 entry appended (and the examples mapping
comment updated). `docs/api/*` regenerated; the coverage gate passes with `routine.hpp`'s
new member text; the `TARGETS` ruling is §10's.

**The sentence that was hard to write** was M4's checkbox. The seam is genuinely done —
built, hostile-tested, mutation-proven — and the temptation was `[x]` on the strength of the
code. It stays `[~]` because the *line item* says "Mechanism HAL abstraction" and an
abstraction is not done until its second consumer has failed to break it; the two reasons
(unfrozen until F3; no hardware has ever moved) are on the checkbox itself. Under-claim
before over-claim, exactly because the code feels finished.

**No claim moved that this chunk does not own:** the `< 1°` sentence, the "booted, drove
nothing" distinction, and every accuracy number are untouched (verified by diff scan of the
guide and README — zero lines).

---

## 15. Verification (run, with output as observed)

```text
cmake --build build/test -j$(nproc) && ./build/test/shulib_tests | tail -4
  [doctest] test cases:     961 |     961 passed | 0 failed | 3 skipped
  [doctest] assertions: 1521812 | 1521812 passed | 0 failed |
  [doctest] Status: SUCCESS!

GUARD1 PASS          (no pros/ include anywhere in include/shulib)
GUARD2 PASS          (no shulib/sim/ include outside sim/)
ARM GATE PASS        (123 headers, arm-none-eabi-g++ -std=gnu++20 -Werror …)

doc gates:  generate → check-coverage PASS · check-fresh PASS
            check-examples PASS (361 quoted lines, all verbatim)
            check-removability PASS · self-test OK

docs/internal/verify/verify-f1.sh
  F1 mutations: 18 RED (good), 0 GREEN (HOLES), 0 build-fail, 0 SKIPPED

register reconciliation:
  grep -rn "PROVISIONAL (A4" include/ | grep "HA-9[23]"
    → hal/mechanism.hpp (HA-92) · sim/hostile/mechanism_hostility.hpp (HA-93) — both cited
  both IDs present in docs/hardware-assumptions.md with entries + index rows
```

---

## 16. Named handoffs

**→ F2 (next):** §13 in full — the span-shaped park guard, cancel-then-start pre-empt, the
four-vocabulary decision, the `TickPhase::User` producer, and the D-8 routine watchdog E1
re-homed there. The concurrency test in `mechanism_routine_test.cpp` is the one F2 extends.

**→ F3:** second consumer of the seam = the freeze trigger for F11 (and the moment the
mechanism headers join `api_doc_tool.py` TARGETS, §10). Choose per-mechanism `StallConfig`
values and confirmation sensors; `RunUntilConfirmed`/`ActuateAndConfirm` are the skeletons;
mint sharper fault codes only where a primitive can prove the pathology (§7's boundary).

**→ G1 (via F7):** the `mechanisms:[]` schema must grow a declared-safe-state field — no
default is safe (§10). Everything else wires directly.

**→ R1:** implement `IDigitalOut` over ADI digital-out; nothing else — `MotorMechanism` /
`PneumaticMechanism` run unchanged over the existing `IMotor` adapter surface.

**→ R3/R4:** HA-92 (the lift-Hold claim — walk it early, its blast radius is a dropped stack
at the buzzer) and HA-93 (jam signature; measure at several commanded voltages — the model
scales with V and thresholds must clear the spin-up/jam separation, not a table value).
