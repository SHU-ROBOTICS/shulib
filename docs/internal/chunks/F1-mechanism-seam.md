# Chunk F1 — the `Mechanism` seam + fakes

> **Phase F, chunk 1 of 2.** Predecessors: all of A, C, D and E. Successor: **F2** (the `sequence/`
> engine and the guaranteed park), which is shaped by whatever this chunk decides.
> **The first chunk since C1 to define a new seam rather than fill one in.**

**Workstream:** WS7 (manipulation) · **Milestone:** M4 · **Freezes:** **none — see §Freeze note**

---

## ⚠️ Read this before anything else: "F1" means two different things

This project has a **name collision**, and it is live in the documents you are about to read.

| "F1" | Where | What it is |
|---|---|---|
| **Chunk F1** | `build-order.md` §Phase F | **This chunk** — the mechanism seam |
| **Register row F1** | `roadmap.md` §Freeze Register | The **coordinate frame**, LOCKED 2026-06-08 |

Same for F2 (chunk = the sequence engine; register row = the accuracy targets, LOCKED), F3 (chunk =
the scoring primitives; row = units) and F4 (chunk = skills motion; row = the HAL signatures). There
is **no chunk F5** — register row F5 is the kinematics contract, LOCKED, and nothing else.

The collision is **live in the code you are about to read**: `motion/motion.hpp:126` ends with
*"the only place a frame rotation happens is `math::fieldToRobot` / `robotToField` (F1)"* — that
`(F1)` is the **coordinate frame**, not this chunk.

**Consequences you must not get wrong:**

- **Do not edit register rows F1–F5.** They are locked contracts about frames, units and interfaces
  and have nothing to do with this chunk.
- When this brief says "the F4 row", it means the **HAL-signatures freeze**. When it says "chunk F3",
  it means the **scoring primitives**.
- The register row this chunk actually touches is **F4** (HAL interface signatures) — and it touches
  it by *adding a sibling that is explicitly not part of it*, not by amending it. See T7.

---

## Why this chunk exists, and why it is *here*

The library can drive to a point and cannot **do** anything when it arrives. Every scoring verb in
the plan — `intakeUntilCapture`, `liftToLevel`, `setQuadrantToggle`, `clampActuate`,
`deployActuator` (master plan §14) — needs something to command, and there is no seam to command it
through.

Two facts make this the right moment, and both are about *ordering*, not convenience:

1. **The abstraction must be shaped by sequencing, not by hardware.** `build-order.md` §F1 says so
   in as many words: *"Defining this before any concrete mechanism exists is deliberate — the
   abstraction should be shaped by what sequencing needs, not retrofitted around whatever hardware
   happens to get built."* The build team has not settled its mechanisms (master plan §14, "build-team
   decisions to settle" #2/#3); waiting for them would let a flexwheel intake dictate a seam that a
   pneumatic clamp then has to fit through.

2. **D3 deliberately left a hole in a freeze for this chunk.** `Routine::then()` is the **one member
   excluded from F10's lock** (`routine.hpp:22-32`), because its accepted return types were chosen
   before mechanisms existed and freezing them would have committed the library to a guess. That
   exclusion has a cost — it is documented as provisional in the guide, the cookbook, the generated
   reference and the register — and F1 is what pays it off or confirms it.

**What F2 inherits from you.** F2 builds `Sequence`/`Parallel`/`Race`/`Deadline` and the guaranteed
end-of-run park — *"plausibly the highest-expected-value code in the library"* (`build-order.md`
§F2). Every one of those combinators has to interleave, pre-empt and time-bound whatever F1 hands
it. **If a mechanism operation cannot be ticked, cancelled, or bounded, F2 cannot be written** — it
would have to reach past your seam into motors, and the park guard could not stop a stuck lift. Read
every ruling below as "what does F2 need this to be".

---

## What already exists — read the actual files

| Thing | Where | The part that constrains you |
|---|---|---|
| **`IMotor`** — the device-seam template | `hal/motor.hpp` | Voltage-only by design; `current()` is documented as *"the PRIMARY capture/stall signal for manipulation sensor-confirm (M4)"*. The stall signal you need is already specified there. |
| **`BrakeMode{Coast,Brake,Hold}`** | `hal/motor.hpp:28` | Its comment already says `Hold` is *"the active position hold the guaranteed park (§M4) … depend[s] on"*. T4 is about who chooses it. |
| **`IMotion`** — the proven operation contract | `motion/motion.hpp:224-256` | `start()` / `tick() → ExitReason` / `cancel()` / `exitReason()` / `state()` / `name()`, plus the **tick contract** ("a motion does NOT own a loop"), the **cancel contract**, and the watchdog rule *"a motion can NEVER hang"*. Read all 127 lines of the banner. This is the shape to mirror — and T2 is about where mirroring stops. |
| **`applyCancelSafeState`** | `motion/motion.hpp:165` | 0 V + `Brake` on every drive motor, defined in **one** place. It is right for a drivetrain and **wrong for a loaded lift** — T4. |
| **`MotionScheduler::waitUntil(pred, timeout)`** | `motion/motion_scheduler.hpp:628` | Ticks the active motion while polling a predicate; the header says `pred` *may* call `async()`/`cancel()`. **This is the existing concurrency primitive** — T3. |
| **The scheduler's re-entrancy guards** | `motion_scheduler.hpp:583-587, 600-606, 629-634` | `inTick_` / `inWait_` / `inBoundary_` preconditions. A blocking mechanism call in the wrong place trips these. T3. |
| **The C2 fault policy** | `motion_scheduler.hpp:55-94, 194` | Abort mask defaults to **ODO_STUCK only**; everything else is continue-degraded. A new fault code lands on the "continue" side **by default** — confirm that is what you want and say so (T6). |
| **`FaultCode`** | `diag/fault.hpp:44-64` | Explicit values, **append-only**, wire-stable (E1 blackbox + F9), pinned by `test/fault_test.cpp`; `FaultLatch` has 32 code slots and the abort mask is a `uint32_t`. Last value: `Implausible = 10`. |
| **`Routine::then()`** | `chassis/routine.hpp:331-365` | The seam, **unfrozen on purpose**. Accepts a callable returning `void` / `bool` / `ExitReason`; anything else fails a `static_assert`. Everything else in that file is pinned by 37 compile-time pins. |
| **`RoutineStopCause`** | `chassis/routine.hpp:147-152` | Append-only. Its own comment: *"F1/F3 mechanism failures arrive as new enumerators, never as re-meanings."* |
| **D1 §2.7 — the three result vocabularies** | `chunks/D1-COMPLETED.md:142-152` | The warning aimed at F2, which F1 reaches first. **Read it before designing T2.** |
| **`TickPhase::User = 5`** | `diag/debug_record.hpp:108` | Reserved, in the wire schema, with **no producer**, for *"G2 markers, mechanisms"*. You are a candidate producer — T6. |
| **A3's hostility pattern** | `sim/hostile/*.hpp`, esp. `composed.hpp` | One pathology per model, composable by chaining, ablation-debuggable, magnitudes registered as HA entries. Hostile mechanism fakes follow this pattern, not a new one. |
| **The hand-written stand-in** | `test/cookbook_examples_test.cpp:84-97` | The `Intake` struct the cookbook calls *"exactly the shape a real mechanism will"* take. It is **quoted verbatim** by `docs/cookbook/01-getting-there.md` — the `check-examples` gate binds them. If your seam contradicts that claim, one of the two must change. |
| **`.vexbot` mechanism config (F7, later)** | master plan §16, line 644 | `mechanisms:[{name, motors:[...], pneumatics:[port]}]`. Whatever you build must be constructible from **that** data by G1's `RobotBuilder`, or you have made G1 impossible. |
| **The G2 registry (F8, later)** | master plan §16.3 line 675 | `runner.on("intake_in", []{ intake.in(); })`. Mechanism actions must be **bindable to a string id by the team** — `legacy-command-vocabulary.md:62` says exactly why (the mechanism set churns season to season; legacy hard-coded five ids and shipped zero executors). |

**Read first, in this order:** `motion/motion.hpp` in full (the three contracts); `hal/motor.hpp`;
`chassis/routine.hpp:102-109` and `:331-365`; `D1-COMPLETED.md` §2.7; `build-order.md` §F1–F2;
master plan §14 (the primitive list and the non-negotiable sequencer) and §17 (the tier table).

---

## Seven rulings

Each needs an explicit decision **and a named rejected alternative**, in the completion record.

### T1 — what layer is a "Mechanism", and does the interface earn its existence?

The plan calls it a *HAL abstraction* (master plan line 294, roadmap M4 line 1054). Take that
seriously enough to challenge it, because there is a real anti-abstraction case:

> An intake is two motors. A lift is two motors and a homing switch. A clamp is one solenoid.
> If `IMechanism` is a `std::span<IMotor*>` with a nicer name, **it has not earned its existence** —
> it adds a layer, a vtable and a document without adding a capability.

The case *for* a real interface, which you must weigh item by item:

- **R1** must implement it over `pros::Motor` groups and ADI digital-out — cheap either way.
- **H2** (`hal/sim`) must implement it over VexBuilder's joints, where there may be no motor at all.
- **A3-style hostility** wants to inject *"a jammed intake, an unconfirmed grab, a stalled lift"*
  (`build-order.md` §F1). Injecting those at a mechanism-level seam is honest and small; synthesising
  them by faking currents on three `FakeMotor`s is neither.
- **Air.** The master plan pairs *"pneumatic/digital-out actuation + the `Mechanism` abstraction"* as
  one deferred F4-additive (line 294), and there is **no digital-out seam in the tree** — grep
  `include/shulib/hal/`. The H-drive's primary mechanism is a **pneumatic clamp** (master plan §14
  #2). A seam shaped only around motors will have been shaped by half the hardware.

**The likely correct shape — rule it, don't inherit it:** two layers with different jobs, named
separately so nobody confuses them.

1. **`hal::IMechanism`** — the *device* seam. Commanded, readable, with a **declared safe state**
   (T4). No control logic, no clock, no loop, no game knowledge. A concrete implementation over N
   `IMotor*` ships with it (so R1 is nearly free and the composition is host-testable), and — if you
   rule air in — a minimal `hal::IDigitalOut` plus a discrete-actuator implementation.
2. **A bounded *operation*** above it (`manipulation/`, the master plan's own name for this
   directory, §6 line 214) — the thing that runs over time, can fail, and can be composed. This is
   what `then()` and F2 consume.

If you rule the interface out and ship a concrete `MotorGroup` + `IDigitalOut` instead, that is a
**defensible answer** — say so plainly, say what the master plan's "Mechanism HAL abstraction" was
answered *by*, and make sure H2 and G1 still work. What is not acceptable is an interface that exists
because the roadmap used the word "abstraction".

**Explicitly out of scope for T1:** what counts as *confirmed*. Hue for a Toggle, proximity for a
cup, a current spike for a clamp — that is season-specific and belongs to F3's primitives or the
team's code, not to an L0 device seam. If you put game semantics in `hal/`, this season is baked into
the HAL. Prefer confirmation as a **caller-supplied predicate**; if you disagree, argue it.

That is not a preference, it is the existing house rule: `hal/vision.hpp:40` carries `classId`, *"a
detected class / color descriptor id"* — an **opaque integer**, deliberately not `Cup` or `Yellow`.
The HAL reports what the device saw; meaning is assigned above it. Follow that.

**And the scope boundary you must draw explicitly:** how much of the operation layer is F1's, and how
much is F2's? The argument that *some* of it is yours is the DoD's own words — "not hangs" requires
something time-aware to bound and to detect a jam, and a purely command-and-read seam has nothing
that could raise a fault. The argument against taking more is that F2 owns composition. State where
you drew the line and why. **Scope creep into F2 is the single largest risk to this chunk**: if you
find yourself writing `Race` or a match timer, you have crossed it.

### T2 — a mechanism can finish *unconfirmed*, and `ExitReason` cannot say that

`control::ExitReason` is exactly `Running / Settled / TimedOut / Cancelled` — four enumerators,
`exit_group.hpp:25-31`, and no fifth. A mechanism has an outcome none of them can express: **the operation completed and the thing did not happen** — the jaws closed on
nothing, the grab was not confirmed. That is not `TimedOut` (nothing timed out) and it is emphatically
not `Settled`.

D1 §2.7 warned that F2's combinators will already span three result vocabularies and recommended a
shared step-outcome vocabulary might want to exist *below* the recipe layer. **F1 arrives first and
decides whether that is easy or hard.** Three shapes, all real:

- **Append to `ExitReason`.** Additive per `version.hpp`, but it means a `Chassis::moveTo` could
  syntactically return `Unconfirmed`, which it never can. A vocabulary that admits impossible values
  makes every exhaustive switch a lie.
- **A separate mechanism verdict type**, with a documented, tested mapping into `ExitReason` for
  `then()`. Costs a fourth vocabulary; keeps each one honest.
- **`ExitReason` + a separate `confirmed()` observer.** Cheapest; the risk is a caller that reads the
  verdict and not the flag — which is precisely the "silent success" failure the whole error policy
  exists to prevent.

Rule it. Whatever you choose: **an unconfirmed operation must be impossible to mistake for a
successful one at the `Routine` layer**, and `RoutineStopCause` grows append-only if it needs a new
enumerator (`routine.hpp:145-146` anticipates exactly this). Do **not** unify the vocabularies
project-wide here — that is F2's call with more evidence; record what you learned for it.

### T3 — who ticks a mechanism, and what stops two loops existing?

`IMotion`'s banner is unambiguous: *"A motion does NOT own a loop, a task, or the estimator."* That
rule is why every closed-loop test in this project is deterministic and reproducible from a seed, and
it is the same standing decision E1 re-affirmed against `build-order.md` at T1.

Apply it: **a mechanism operation does not own a loop either.** The loop owner ticks it. Today the
loop owner is the caller's own loop, or `MotionScheduler::waitUntil(pred, timeout)`, whose predicate
already runs once per iteration alongside the motion tick.

Two things must be true and **tested**, not asserted:

- **A mechanism ticking inside a `waitUntil` predicate while a motion runs must not trip
  `inTick_`/`inWait_`/`inBoundary_`, must not double-tick anything, and both must progress.** This is
  the "intake while driving" case that makes a holonomic scoring cycle worth having, and it is the
  test F2 inherits. It serves **G2** as well: `motion_scheduler.hpp:622` calls `waitUntil` *"the
  marker/callback primitive (G2's PathRunner)"*, so a mechanism that composes with it is a mechanism
  the no-code path can fire from a path marker.
- **No hang, ever.** A blocking convenience is allowed only if it cannot deadlock, cannot re-enter a
  wait that already owns the loop, and is watchdog-bounded the way C1's motions are (*armed in
  `start()`, no code path disarms it*). If you cannot make a blocking helper safe in every position a
  caller can put it, **ship only the tick form** and say why. C2's stalled-pacer guard is the
  precedent for turning an un-catchable hang into a loud precondition.

Also rule: can two operations drive the **same** mechanism at once? C2 made one-active-motion
structural via pre-empt-then-replace. Two mechanisms running at once is obviously fine and required;
two operations on one mechanism is a collision. Decide the shape and make it structural, not
documented.

### T4 — the drivetrain's safe state is the wrong safe state for a lift

`applyCancelSafeState` is 0 V + `BrakeMode::Brake`, defined once, for the drive. Copying it to
mechanisms would be a **physical mistake**:

- A **loaded lift** at 0 V drops its stack. Its safe state is `Hold`.
- A **jammed intake** commanded to `Hold` sits at stall current until the thermal fault fires
  (`MotorOverTemp`, ~55 °C, `hal/motor.hpp:65-68`). Its safe state is `Coast` or `Brake`.

So there is no library-wide default that is safe for both, and picking one silently means half the
mechanisms are wrong at the buzzer. The likely answer is a **per-mechanism declared safe state**,
declared where the mechanism is constructed and applied by *every* stop path (cancel, operation
failure, and F2's park guard) — the `applyCancelSafeState` pattern, one definition per mechanism
rather than one for all.

This is F2's park guard's dependency: it must be able to force every mechanism safe at t=match-end
regardless of what any of them is doing. If F1 does not define that, F2 reaches into motors.

**Register the physical claims** (whether a `Hold` actually holds a loaded cascade lift, what current
a jam draws) as `HA-nn` — they are unverifiable until hardware. Next free number is **HA-92**.

### T5 — `then(intake.in)` does not compile, and it is quoted in eight public places

The flagship line of the accessibility pillar is `chassis.moveTo(p).then(intake.in)` (master plan
§17, line 713). As literal C++, `intake.in` is valid **only** if `in` is a callable *data member* or
a static. If `in` is an ordinary member function — which is what the same document assumes 38 lines
earlier, `runner.on("intake_in", []{ intake.in(); })` (line 675) — then the flagship line has never
compiled and cannot, and the honest spelling is `then([&]{ intake.in(); })`.

**This was checked with a compiler, not reasoned about.** Against a `then()` carrying the real
`std::invoke_result_t` + `is_constructible_v<bool, R>` constraints, a callable data member compiles
and an ordinary member function fails:

```text
error: invalid use of non-static member function 'void IntakeMemberFn::in()'
   then(bad.in);
        ~~~~^~
```

**The two lines of the master plan contradict each other**, and the promise is repeated in
`docs/guide/09-the-recipe-api.md:232-238`, `docs/cookbook/01-getting-there.md:66`,
`roadmap.md:1147` and `:1153` (where the M7 Tier-2 checkbox is `[~]` *because* of it),
`chassis/routine.hpp:41,103,337`, and the generated `docs/api/routine.md`.

Rule one of:

- **Make it literally true** — mechanisms hand out callable action objects, so `intake.in` is a real
  member you can pass. Then the eight places become true, and `then()` may not need to change at all.
- **Fix the prose everywhere** — the idiom is `then([&]{ intake.in(); })` or an explicit action
  accessor. Then those eight places are edits you own in this chunk, including the master plan.

Do not leave it half-done: a documented example that does not compile is exactly the class of defect
the `check-examples` gate exists to prevent, and **this one is outside that gate by construction**.
The gate scans only fenced ` ```cpp ` blocks (`api_doc_tool.py:780`); every occurrence of this claim
is inline-backtick prose in a table cell or a sentence, which no compiler has ever seen. The master
plan *is* inside the gate's glob (`docs/*.md`, line 730) — so **if you make the claim true, make it a
fenced example**, and it becomes machine-checked forever.

`then()` is yours to change — it is the one unfrozen member. Every other member of `routine.hpp` is
pinned; **if a pin fires for anything except `then()`, stop and report it.** Any change to `then()`
must leave every existing use — cookbook, guide, recipe tests — meaning exactly what it means today.

### T6 — fault codes: how many, and which failures are not faults at all

The DoD is *"failure modes surface as fault codes, not hangs."* Before minting codes, draw the line
this project already draws (§18.4: *faults log and recover; they never crash*):

- A **fault** says the robot is unwell — a jam, a stall, a watchdog. Triage material.
- A **verdict** says the task did not happen — an unconfirmed grab with a healthy mechanism. That is
  strategy, not pathology, and it belongs in T2's vocabulary, not the fault latch.

Getting this backwards floods the latch with normal outcomes and destroys first-fault triage, which
is the entire reason the latch exists.

Mint **only codes this chunk can produce and prove**, appended at the end (last value:
`Implausible = 10`), and state for each: the numeric value, the `faultCodeName` spelling, its
one-line pin in `test/fault_test.cpp` (each code is pinned individually, lines 33-43), the E1
blackbox round-trip, and **the C2 abort-mask consequence**.

On that last point, the mechanics are worth knowing before you choose values: `abortFaultMask` is a
`uint32_t` **bit-indexed by the code's numeric value** (`motion_scheduler.hpp:945-959` —
`countr_zero` → `static_cast<FaultCode>(bitIdx)`), bounded by the latch's 32 tally slots. Codes 11
and 12 therefore fit with room to spare, and land on the **continue-degraded** side by default
because the default mask is ODO_STUCK only. Confirm that is what you want (a jammed intake should not
abort a drive) rather than discovering it later.

Also rule `TickPhase::User` (`debug_record.hpp:108`): it is reserved *for mechanisms*, in the wire
schema, and **has no producer — verified**; the only three references in the tree are a numeric pin,
a name pin and `tickPhaseName`. Either populate it or record why not. **E1's lesson applies
directly** — `DebugRecord::fault` had no producer from A1 until E1 while TermSink rendered it and
chapter 11 documented it, so it could never once have appeared on a real run. A field nothing fills
is worse than a field that does not exist, because nothing looks wrong.

### T7 — ownership: does `RobotContext` grow a mechanism, and who names them?

`RobotContext` says it is *"NOT part of the F4 freeze — it grows additively"* (`robot_context.hpp:9-11`),
so adding to it is legal. Whether it is *right* is the question.

Against: mechanism names and counts are season- and robot-specific. A fixed accessor set
(`ctx.intake()`) hard-codes this season into the library — and that is exactly the legacy failure C6
catalogued (five hard-coded mechanism ids, three data representations, **zero executors**;
`legacy-command-vocabulary.md:59-63`). For: motions read hardware only through `ctx`, and G1's
`RobotBuilder` must wire a robot from a profile that includes `mechanisms[]`.

The shape that looks right: **the library owns the seam; the team owns the mechanisms**; G2's
registry owns id→action binding. An operation still needs a small deps bundle (clock, fault latch,
telemetry) — mirror `MotionDeps`' named-pointer + `validate()` pattern rather than inventing a third
convention.

Whatever you rule, check it against **G1**: can `RobotBuilder.from(profile)` still wire mechanisms
from `{name, motors:[...], pneumatics:[port]}` without redesigning your seam? If not, you have moved
a problem into a chunk that cannot argue back.

**Freeze note (mandatory, D2's lesson).** F1 **freezes nothing** — the seam gets its second consumer
at F3, on hardware, and this project's proven pattern is build → second consumer → freeze (C4→D1→D2,
D1→D3). But **silence in a freeze register reads as "frozen"**: state in the F4 row (or a new row
marked 🎯) that a mechanism seam now exists, that it is **outside** F4, and what will freeze it and
when. That is exactly the omission D2 found and fixed for `Routine`.

---

## Scope

### In
1. **The device seam** per T1, in `hal/`, with its concrete implementation(s) — motor-group and, if
   ruled in, discrete/pneumatic actuation plus the minimal `hal::IDigitalOut` it needs.
2. **The bounded operation contract** per T2/T3 — tickable, cancellable, watchdog-bounded, with a
   verdict that can express *unconfirmed*.
3. **Deterministic fakes**, in the `hal/fake/` style: injectable state, no physics, no surprises.
4. **Hostile fakes**, in the A3 style: **a jammed intake, an unconfirmed grab, a stalled lift** —
   `build-order.md` names those three; composable and ablation-debuggable like `sim/hostile/`.
5. **The declared safe state** per T4, applied by every stop path.
6. **Fault codes + `TickPhase::User`** per T6.
7. **`then()` integration** per T5, with the prose reconciled wherever it is quoted.
8. **The concurrency proof** per T3 — a mechanism operating while a motion runs.

### Out
- **Concrete scoring primitives** (`intakeUntilCapture`, `liftToLevel`, `setQuadrantToggle`,
  `orientToScoringHalf`, `rotateClampToAngle`) → **chunk F3 / Phase F′**: they need hardware *and*
  the build team's final mechanism decisions.
- **Combinators, the sequencer, the guaranteed park, D-8's routine watchdog** → **chunk F2**.
- **The command-id registry** (`runner.on(...)`) → **G2**.
- **`.vexbot` mechanism ingestion / `RobotBuilder`** → **G1/G3**.
- **`hal/pros` implementations** → **R1**. Author nothing that needs a brain.
- **Air-budget accounting** → Phase F′.
- **Tuning any threshold** (stall current, confirm windows, actuation times) → **R4**. Invent,
  register `HA-nn`, label as invented.

### Explicitly rejected
- **An interface that exists because the roadmap said "abstraction."** T1 demands it earn its keep.
- **Game semantics in `hal/`** — hues, game-object names, "capture". T1.
- **A mechanism that owns a loop or a task.** T3, and the standing no-background-task decision (E1 T1).
- **One safe state for all mechanisms.** T4 — it is wrong for one of the two robots either way.
- **Minting fault codes for outcomes that are not pathologies.** T6.
- **Touching any pinned member of `routine.hpp` or `chassis.hpp`.** `then()` only.
- **Changing what an existing `then()` action means.** The cookbook, guide and tests must be
  untouched in meaning.

---

## Test requirements

Every test names, in a comment, the bug it would catch.

- **The DoD case:** a fake mechanism driven through the seam to a successful, confirmed completion.
- **Every failure mode is bounded and legible:** jam, stall, never-confirm, and cancel — each exits
  with the right verdict, raises the right fault (or correctly raises none, per T6), and **never
  hangs**. Include the adversarial clock case that C1/C2 used to prove the watchdog is real.
- **The cancel contract, mirrored from `IMotion` and tested as such:** idempotent; applies the
  declared safe state; **preserves a completed verdict** (rewriting history would lie to the C5
  result line — C2's exact ruling); a no-op on a never-started operation. Two near-identical
  contracts that quietly diverge are a bug factory; pin the mirroring.
- **The safe state is asymmetric and actually observed:** a lift-shaped mechanism ends in `Hold`, an
  intake-shaped one does not — asserted on **`FakeMotor::brakeMode()` / `commandedVoltage()`**, i.e.
  the bottom of the stack, never on the mechanism's own record of what it thinks it commanded.
- **Concurrency (T3):** a mechanism operation ticking inside `scheduler.waitUntil` while a motion is
  active — no precondition trip, no double tick, both progress, motion accuracy unchanged.
- **`then()` integration:** a mechanism action as a `Routine` step; failure stops the chain with the
  correct `RoutineStopCause`; **an unconfirmed operation cannot read as success** (T2).
- **Nothing above the seam changed:** the existing recipe/cookbook/guide cases stay green and
  unedited. If `then()` changed shape, prove old uses still mean the same thing.
- **Blackbox round trip** for any new fault code, through E1's decoder — bytes out, bytes back.
- **ARM** — the new headers compile under the gate.

### The recurring trap, in F1's form

Five chunks have been bitten by a test sharing a model with the code under test (C1, C3, C4, E2, E3).
**F1's version is the fake and the operation sharing a notion of "done".** If the fake reports
completion by the same rule the operation uses to decide completion, a broken completion check passes
green and proves nothing.

The counter, required:

- **Write the expected timeline by hand, from the contract, before running it** — tick N: commanded;
  tick N+k: confirmed; verdict at tick N+k. Assert against those literals, not against the fake's
  opinion.
- **The fake must be able to lie** (A3's whole premise): report confirmed when it is not, report
  motion when stalled. An operation that trusts its device has no failure detection, and a test that
  never lies to it cannot tell.

### Mutations

Break the operation's watchdog (disarm it); make `cancel()` skip the safe state; make `cancel()`
overwrite a completed verdict; **swap the two safe states** (`Hold` ↔ `Coast`); make the stall
detector always report healthy; make an unconfirmed completion report success; **drop the fault raise
while keeping the verdict** (E1's exact hole class); make the hostile fake's jam injection a no-op
(A3's precedent — a hostility that does not bite is worse than none).

**A mutation that stays GREEN is a hole — log it, close it with a test that fails alone, and give it
a prominent place in the record.** D3 found four, E1 two, E2 one, E3 three, E4 two. Gate the runner
on build success (a non-compiling mutation read off a stale binary looks green — C4). **Never pipe
the runner into `head`** — E2 lost a header to SIGPIPE that way; trap `PIPE` and count what you ran.

---

## Documentation

**Go slowly here.** Four stale-prose defects were found by *reading* in one session at Phase E; no
gate has an opinion about whether a sentence is still true.

- **Chapter 13 (extending the library)** — the natural home for "how to write a mechanism against the
  seam", written for someone who is not a robotics expert.
- **Chapter 14 (what it cannot do yet)** — the **"The mechanism seam is a placeholder"** section
  (lines 167-184) and, in the *next* section, the **"No mechanisms for recipes to command"** bullet
  (line 191+) are now wrong or partly wrong. Rewrite in E3's order: what is measured, what it was
  measured against, what remains unmeasured. What remains: no concrete scoring primitive, nothing on
  hardware, no sequencer.
- **Chapter 9 (the recipe API)** — lines 232-238 describe `then()` as a placeholder waiting for
  mechanisms. Whatever T5 rules, this passage changes.
- **Cookbook** — `README.md:101-103` and `01-getting-there.md:65-66` claim the hand-written `Intake`
  struct is *"exactly the shape a real mechanism will"* take. Either that is now true, or the claim
  gets corrected. The struct is quoted verbatim from `test/cookbook_examples_test.cpp:88-97`, so the
  `check-examples` gate binds them — changing one changes the other.
- **Master plan** — §17's tier table (line 713) and the registry example (line 675) per T5; the §6
  module map (line 214) if `manipulation/` now exists; the **F4 scope note** (lines 293-297) which
  lists this exact work as a deferred additive — it is no longer deferred.
- **`roadmap.md`** — the M4 WS7 checkbox *"`Mechanism` HAL abstraction"* (line 1054) flips with cited
  evidence, `[~]` if partial, naming what remains and who owns it; the **F4 register row** per T7's
  freeze note; and check whether M7's `[~]` Tier-2 items (lines 1147, 1153) move — they are marked
  partial *specifically* because `.then(intake.in)` could not exist.
- **`hardware-assumptions.md`** — every invented number, starting at **HA-92**, with source,
  confidence, settling measurement, owning chunk and blast radius.
- **`docs/api/`** — decide whether the seam header joins `TARGETS` in `tools/api_doc_tool.py:53`.
  Today the coverage gate covers only the two frozen surfaces, so **an undocumented public member in
  a new header fails nothing**. D3's lesson stands: *a gate's exclusion list is where its holes
  live.* Rule it deliberately either way.
- **`guide-maintenance.md`** — if the chapter set or the maintenance list changes.

**Do not let the seam's arrival inflate any claim.** Nothing here touches accuracy, and nothing here
has met a robot. The `< 1°` sentence and the "it booted, it drove nothing" distinction stay exactly
as they are.

---

## Definition of Done

- [ ] T1–T7 ruled explicitly, each with its rejected alternative named
- [ ] The device seam exists with its concrete implementation(s); a fake mechanism is driven through
      it end to end
- [ ] The operation contract is tickable, cancellable and watchdog-bounded — **proven not to hang**
      under an adversarial clock
- [ ] Hostile fakes cover **jammed / unconfirmed / stalled**, composable in the A3 style
- [ ] The declared safe state is per-mechanism, applied by every stop path, asserted at motor level,
      and its asymmetry is mutation-proven
- [ ] Failure modes surface as fault codes; the fault/verdict line is drawn and defended (T6)
- [ ] `TickPhase::User` populated, or its emptiness recorded with reasoning
- [ ] A mechanism operates **while a motion runs**, through the existing scheduler seam, with no
      precondition trip and no double tick
- [ ] `then()` integration works; an unconfirmed operation cannot read as success; every existing
      `then()` use unchanged in meaning
- [ ] The `intake.in` claim is either true and compiled, or corrected everywhere it appears (T5)
- [ ] No pinned member of `routine.hpp` / `chassis.hpp` changed; F6 and F10 pins green
- [ ] Nothing frozen at F1; the register says so out loud (T7)
- [ ] Expected timelines hand-written from the contract; the fake can lie
- [ ] Mutations executed with the runner gated on build success; every green logged and closed
- [ ] Guide ch. 9/13/14, cookbook, master plan, roadmap, HA register all updated; claims unscoped
      upward nowhere
- [ ] Suite green; both CI guards; ARM gate; all four doc gates

---

## Live progress log — required

`docs/internal/chunks/F1-PROGRESS.md`, created **first**, appended as work happens (`date +%H:%M:%S`).
It is watched with `tail -f` and it is what makes an interrupted chunk recoverable.

---

## Documentation contract

All of the above, plus **`F1-COMPLETED.md`**. Give **T1, T2 and T4 their own sections**: T1 is the
structural decision F2, F3, G1, G2, H2 and R1 all inherit; T2 either eases or worsens the vocabulary
problem D1 handed to F2; T4 is a physical claim about a loaded lift that nobody can check until there
is a robot, so its reasoning has to survive in writing.

**F2 will be read against this record.** State plainly what a mechanism can and cannot do now, and
what F2 may assume.

**Do not commit. Do not push.**

---

## Landmines

- **"F1" is also a locked freeze row about the coordinate frame.** Do not touch rows F1–F5.
- **Don't let the seam own a loop.** T3, and the standing decision behind every deterministic test in
  this project.
- **Don't copy the drivetrain's safe state.** T4 — it drops a loaded lift.
- **Don't put game semantics in `hal/`.** T1.
- **Don't mint a fault code for a strategy outcome.** T6 — it destroys first-fault triage.
- **Don't let the fake and the operation share a notion of "done".** The trap that has bitten five
  chunks, in its F1 form.
- **Don't edit `routine.hpp` beyond `then()`.** 37 pins are watching; a pin firing elsewhere is a
  breaking change to argue, not an edit to make.
- **Don't claim the library can score.** It can command a fake mechanism on a host. No robot has
  moved.
