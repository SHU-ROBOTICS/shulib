# Chunk C2 — `MotionScheduler`

> **Phase C, chunk 2 of 7.** Predecessor: C1 ✅ (`IMotion` + the five primitives).
> C1 could only hand-chain motions. C2 is what actually runs a routine.

**Workstream:** WS6 (Motion & scheduling) · **Milestone:** M2

---

## Why this chunk

C1 gave the library the ability to execute *one* motion. A real routine is a **chain** — and the
user's headline requirement is that a full routine hits its target with negligible accumulated error.
C1 proved the accuracy property by hand-chaining (error **flat in move count**, clean routines ending
0.00–0.24 in). C2 builds the thing that actually does the chaining in production, plus the control
verbs an auton needs: run in the background, wait for a condition, cancel.

It also owns a policy C1 explicitly deferred: **C1's motions raise faults but do not self-abort.**
Deciding what happens to a faulted motion is C2's job.

---

## What already exists

| Thing | Where | Note |
|---|---|---|
| `IMotion` tick contract, `MotionState`, `MotionDeps` | `include/shulib/motion/` | the interface you schedule |
| `MoveToPose`, `TurnTo`, `StrafeTo`, `HoldPose`, `DriveBrake` | same | the five primitives |
| `OdoStallCheck` (`ODO_STUCK`) | `motion/` | owned by every motion's tick |
| `Watchdog`, `ExitGroup`, `ExitReason` | `control/` | Settled / TimedOut / Running |
| `Localizer` + `lastOdomDeltaImplausible()` | `localization/` | estimate liveness |
| `DrivePlant`, `SimHarness`, hostile models | `sim/` | what you test against |
| `DebugRecord`, `TermSink`, `FaultCode`, `FaultLatch` | `diag/` | observability |
| `IClock` / `FakeClock` | `hal/` | **injected time — never wall-clock** |

**Read first:** `C1-COMPLETED.md` — especially §9 *"What we now know for certain, and what we do not"*
and §10 *"Deliberately left for later chunks"*, which names C2's inherited work precisely. Also
`build-order.md` Phase C and `RESUMING.md`.

---

## Scope

### In
- **`MotionScheduler`** — exactly one active motion at a time
- **`async()`** — start a motion without blocking
- **`waitUntilSettled()`** — block until the active motion exits
- **`waitUntil(pred)`** — block until an arbitrary predicate holds (the marker/callback primitive
  that G2's `PathRunner` will later need)
- **`cancel()`** — stop the active motion, leaving the drivetrain in a **defined safe state**
- **Fault policy** — what happens to a motion that raises a fault (C1's named deferral)
- **Queue/sequencing semantics** — what "start a motion while one is running" means

### Out
- `HDriveKinematics` → C3 · the `Chassis` facade → C4 · per-motion result formatting and the run
  summary → C5 · the `sequence/` combinator engine (`Sequence`/`Parallel`/`Race`/`Deadline`) → **F2**
- **Min-velocity handoff / blended chaining → Frontier.** v1 is stop-and-settle by design. Do not
  build blending. If stop-and-settle imposes a measurable *time* cost across a routine, measure and
  report it — that informs whether blending is worth pulling forward.

---

## Design constraints

### 1. One active motion — enforced, not merely documented
Two motions commanding the drivetrain simultaneously is a category of bug that must be structurally
impossible, not avoided by convention. Decide and pin the semantics of starting a motion while one is
active (reject? pre-empt? queue?) and justify the choice.

### 2. `cancel()` must leave a defined, safe state
Not "whatever the last tick happened to command." Define it — brake? hold? coast? — justify it, and
test that the drivetrain actually reaches that state. A cancel that leaves motors at their last
commanded voltage is a robot that drives into a wall.

### 3. Fault policy — C1's explicit deferral
C1's motions raise faults but keep ticking. Decide what the scheduler does: abort and brake, continue
degraded, escalate by fault severity? Consider that `ODO_STUCK` (a dead encoder) means the estimate is
*lying* — continuing to servo against a lying estimate is worse than stopping. Justify whatever you
choose; different faults may warrant different responses.

### 4. Never hang — the whole point
Every wait must be bounded. `waitUntilSettled()` on a motion whose watchdog fires must return.
`waitUntil(pred)` with a predicate that never becomes true **must not block forever** — give it a
timeout and a clear return telling the caller which happened.

### 5. Clock is injected
`IClock` throughout, never wall-clock. Determinism under a seeded scenario is a DoD item.

### 6. The scheduler is the natural home for routine-level observability
It knows motion boundaries, so it can count motions, track elapsed time, and mark transitions. Emit
what C5 will need for its per-motion result line and run summary — but **do not build C5's
formatting**. Respect A1's cost contract (`emitRecord`; `wantsRecord()`/`emit()` as a pair).

---

## Test requirements

Hold C1's escalated bar — swept and seeded, not sampled; every test names the bug it would catch.

- **Sequencing** — a chained routine runs to completion through the scheduler; results match C1's
  hand-chained baseline (**error flat in move count**; clean routines 0.00–0.24 in)
- **One-at-a-time** — starting a motion while one is active behaves per the pinned semantics; two
  motions can never both command the drivetrain
- **`async()`** — returns immediately; the motion progresses on subsequent ticks
- **`waitUntilSettled()`** — returns on settle; **also returns on timeout** rather than hanging
- **`waitUntil(pred)`** — returns when true; returns on timeout when never true; the caller can tell
  which
- **`cancel()`** — mid-motion, at the very first tick, and after settle (idempotent); drivetrain
  reaches the defined safe state in every case
- **Fault policy** — each fault class produces the decided behaviour, `ODO_STUCK` included
- **Determinism** — same seed → byte-identical routine outcome
- **Hostile** — a full routine under A3 composed hostility completes, faults appropriately, never
  diverges or hangs
- **Adversarial** — cancel during the boot window; cancel a motion already timed out; `waitUntil` with
  a predicate true on entry; zero-motion routine; back-to-back cancels; a motion started from inside
  a `waitUntil` predicate (re-entrancy — decide and pin the behaviour)

**Mutations — go well past four**, and mutate every load-bearing decision. **A mutation that stays
GREEN is a hole in the suite and the most valuable thing you can find** — C1 found two (#5 clamp, #11
battery compensation) and closed both with new tests. Record every result as observed.

---

## Definition of Done

- [ ] `MotionScheduler` with `async()` / `waitUntilSettled()` / `waitUntil(pred)` / `cancel()`
- [ ] One-active-motion enforced structurally; semantics pinned and justified
- [ ] `cancel()` leaves a defined, tested safe state in every case
- [ ] Fault policy decided, documented, and tested per fault class
- [ ] No wait can hang — every wait bounded, with a distinguishable timeout return
- [ ] A full routine through the scheduler reproduces C1's accuracy baseline
- [ ] Survives A3 composed hostility; deterministic under seed
- [ ] Any new constant carries an `HA-nn` register entry
- [ ] Suite green under strict `-Werror`; both CI guards pass; ARM gate passes

---

## Live progress log — required

`docs/internal/chunks/C2-PROGRESS.md`, appended as work happens (`date +%H:%M:%S`). Vocabulary
`START` / `DONE` / `MUTATE` / `DECIDE` / `BLOCKED` / `FOUND`. Watched live with `tail -f`.

---

## Documentation contract

All six, plus **`docs/internal/chunks/C2-COMPLETED.md`** at C1's depth (570 lines, 13 sections),
including a **"What we now know for certain, and what we do not"** section written for a reader who
was not present.

**F6 is close.** C4 builds the `Chassis` facade, D1 exercises it a second time, D2 freezes it. The
scheduler's verbs are what the facade will wrap — **flag every shape F6 will inherit from C2**, because
after D2 those signatures change only by version bump and migration.

**Do not commit.** Leave everything in the working tree for review.

---

## Landmines

- **A cancel that leaves motors energized is a robot driving into a wall.** Define the safe state.
- **Don't let two motions command the drivetrain.** Structural, not conventional.
- **Don't build blending.** Stop-and-settle is v1 by design; blending is Frontier.
- **Don't build F2's combinator engine.** Scheduler only — one motion, not `Sequence`/`Race`.
- **Continuing to servo against a lying estimate is worse than stopping** — weigh that in the fault policy.
