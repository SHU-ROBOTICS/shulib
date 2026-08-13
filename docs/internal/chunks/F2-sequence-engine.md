# Chunk F2 — the `sequence/` engine and the guaranteed end-of-run action

> **Phase F, chunk 2 of 2 — closes the phase.** Predecessor: F1 (the mechanism seam).
> **`build-order.md` calls the guarantee in this chunk "plausibly the highest-expected-value code in
> the library — it converts a failed run into 8 points."** It is also the chunk most likely to ship a
> guarantee it cannot keep, so this brief carries measurements rather than intentions.

**Workstream:** WS8 (sequencing) · **Milestone:** M4 · **Freezes:** **none**

---

## ⚠️ "F2" means two different things, and the collision is in SHIPPED CODE

| "F2" | Where | What it is |
|---|---|---|
| **Chunk F2** | `build-order.md` §Phase F | **This chunk** — the sequence engine |
| **Register row F2** | `roadmap.md` §Freeze Register | **The accuracy targets** (heading `< 1.0°` hard, ~1.0″ pose, ~0.25″ docked), LOCKED 2026-06-08 |

This is worse than the F1 case, because it is not confined to prose:

- **`include/shulib/spec/accuracy.hpp:3-4`** is titled *"the F2 accuracy targets… Freeze F2"*.
- **`test/accuracy_spec_test.cpp`** has four case names beginning **"F2 spec…"**.
- **`roadmap.md:131`** — register row **F11**'s "What depends on it" column reads **"F2's combinators +
  park guard"**, while **row F2 in the same table** is the accuracy targets.
- **`docs/legacy-command-vocabulary.md:5`** invents a *third* meaning, calling F2 *"(mechanism
  primitives)"* — which is F1/F3.

**Do not edit register rows F1–F5.** When this brief says "row F2" it means accuracy; everywhere else
"F2" is this chunk. `verify-f1.sh` already gates on the F1 version of this trap; **`verify-f2.sh` must
carry the identical gate.**

---

## Why this chunk exists, and why it is *here*

Every bound in the tree today is scoped to exactly one motion, one wait, or one mechanism operation.
**Nothing bounds a whole routine, and nothing knows when the match ends.** `Routine` has eleven steps,
four observers, **no clock and no deadline**; `MotionConfig::defaultTimeout` is 5 s (HA-51, *invented*),
so **forty default-timeout steps is a legal 200-second routine** — behaving exactly as designed — inside
a 15-second match. There is no run-scoped start time anywhere in `include/`; even `RunReporter`, the one
run-scoped object, records battery-at-start and not time-at-start.

F2 is where that stops being the author's unaided problem.

**It also collects the debt E1 named.** D-8, the routine-level watchdog, was re-homed here with its
reasoning (`docs/diagnostics-plan.md:175-184`), and E1 already saw what this brief confirms: *"a
whole-routine deadline needs an owner that outlives a motion and a policy for what to do when it
expires (park? abort? report?), which is F2's guaranteed-park question."* **D-8 and the end-of-run
action are ONE primitive with TWO policies**, not two features — treat them that way.

---

## Naming: this is the "guaranteed END-OF-RUN ACTION", not "the park guard"

Master plan §14 names it correctly — *"guaranteed end-of-run action"*. `build-order.md` and
`roadmap.md` have both drifted to *"the guaranteed park"* / *"park guard"*, which names **one team's
strategy for one season**. Names shape implementations: a brief that says "build the park guard"
produces parking.

**The library must never know where the Midfield is, what "parked" means, how long a match is, or
whether ending somewhere is even part of the plan.** It knows only that *some* caller-supplied action
fires at *some* caller-supplied instant. A team ending tucked against a goal supplies that pose; a team
raising a lift under a height limit supplies that; a team that scores to the buzzer and stops supplies
nothing. §14 wants **two** actions for our own robots (the park *and* a Toggle re-verify), so the end
action is **composable, not a single pose**.

**Fix the drift in the docs as part of this chunk.**

---

## What was MEASURED before this brief was written

Four executable probes ran against the real stack. **These are numbers, not predictions**, and several
overturn the obvious design.

| # | Finding | Evidence |
|---|---|---|
| 1 | **A deadline-cancelling pacer unwinds `waitUntilSettled()` — and ONLY that.** | A 30 s `moveTo` cut at its 2.000 s deadline, **0.0000 s latency**, `Cancelled`, motors 0 V + Brake |
| 2 | **It does NOT unwind `waitUntil(pred, timeout)`.** Its loop is `while(true)` on the predicate and its own timeout; `active_` appears in neither exit condition. | 30 s wait returned **28.0000 s late**, with **2801 cancels fired into it, all invisible** |
| 3 | Realistic lateness | one `waitFor(3_s)` → **1.01 s late**; three `pause(3_s)` → **7.03 s late, `ok=1`, `skipped=0`** — the chain sails on |
| 4 | **Motors stay 0 V + Brake through the overrun.** | ~2800 idle ticks, drive untouched. **A lost-points failure, not a runaway.** Do not let anyone downgrade it to "safe anyway" — safe and parked are different, and only one scores |
| 5 | **A deadline check INSIDE the predicate works.** | returned at 2.0100 s, **0.0000 s latency**, 0 V + Brake |
| 6 | …**and its verdict is a trap.** It returns `WaitResult::Satisfied`, which `Routine::waitFor` maps to `recordSuccess()` — **the chain keeps scoring past the buzzer** | measured |
| 7 | **`async()` from `pace()` is legal and drives the end action home.** | park reached to **0.2245 in**, no precondition trip |
| 8 | …**and it lies to the caller.** `moveTo(A)` returned **`Settled`** describing the *park*, **0 log lines at any level**; the chain continued 105 in away and ended **66 in outside** the park with `ok=true, skipped=0` | measured |
| 9 | **Cancel-at-the-deadline is INERT, and marginally counterproductive.** | with cancelling: **80.55 in** travelled, target reached **t=4.00 s**. With *no deadline logic at all*: **80.37 in**, **t=4.48 s**. Cancel/restart resets the deceleration, so it arrives *sooner*. 300 `Cancelled` boundaries bought only telemetry |
| 10 | Where the check sits inside `pace()` decides everything | step-then-cancel → **10.79 in** of post-deadline travel; cancel-then-step → **0.0000 in** |
| 11 | **A `moveTo(park)` issued after the deadline is itself cancelled after 1 tick**, and the pacer cannot run a blocking verb (`"blocking waits are not re-entrant"`) | measured |
| 12 | **Mechanisms do not preempt at all.** The motion cancel touches only `ctx.driveMotors()`. | a live op kept commanding **9 V** across the deadline |
| 13 | **`applySafeState()` survives exactly 2 device events** before a live op overwrites it | `mechanism_op.hpp:304` re-energizes |
| 14 | **The half-safe state**: the re-command restores voltage but not brake mode → **`brake=Hold, V=9.00`**, which passes any assertion checking only the mode | measured |
| 15 | **A throwing pacer is the most dangerous state found**: motors **11.4 V under Coast**, `hasActive=1`, slot pointing at a dying stack object | `Chassis::waitUntil` is a bare pass-through with **no `DetachGuard`** (`runBlocking` has one) |

---

## What already exists — read the actual files

| Thing | Where | The part that constrains you |
|---|---|---|
| `ITickPacer` — the only seam that regains control mid-motion | `motion_scheduler.hpp` | Caller-supplied, **frozen into F6's constructor**. `pace()` runs every loop iteration with `inTick_`/`inBoundary_` both false |
| `cancel()`'s guards | `motion_scheduler.hpp:663-667` | Forbids `!inTick_` and `!inBoundary_` — **not `!inWait_`**. So cancel-from-pacer is *precondition-legal today, undocumented and absent from C2's pinned re-entrancy list* |
| `MotionScheduler::tick()` | `motion_scheduler.hpp:582` | Public, "for callers running their own paced loop". **This is how F2 owns a loop without re-implementing C2** |
| `control::Watchdog` | `control/watchdog.hpp:3-6` | Its own banner says *"reusable for any bounded wait"* and already names the guaranteed park. **You need an OWNER for one, not a new timer type** |
| F1's handoff | `F1-COMPLETED.md` §13 | The guard takes a caller-supplied `span<hal::IMechanism*>`; ops are *"inert only after exit/cancel"* |
| D3's binding instruction | `D3-COMPLETED.md` §2.1 | *"if a deadline is ever added, it must be **opt-in and inert by default**. A `deadline()` that silently clamped every subsequent step's timeout would change the behaviour of existing routines — a breaking change wearing an additive costume."* |
| D1 §2.7 | `D1-COMPLETED.md:142-152` | The vocabulary warning, aimed here by name |
| `MotionOptions` | `chassis.hpp:186-203` | **`timeout = 0` means the 5 s CONFIG DEFAULT, not zero**; `validate()` **throws on a negative timeout** |

**Read first:** `motion_scheduler.hpp` in full (tick / waitUntilSettled / waitUntil / cancel / async and
the re-entrancy guards); `docs/diagnostics-plan.md:175-184`; `D1-COMPLETED.md` §2.7; `F1-COMPLETED.md`
§13; `D3-COMPLETED.md` §2.1; master plan §14.

---

## Eight rulings

Each needs an explicit decision **and a named rejected alternative**.

### T1 — what actually fires the guard

Measurement 2 kills the simple answer. A pacer decorator bounds `waitUntilSettled`-shaped verbs and
**nothing else**, and measurement 9 shows that cancelling alone is not even a partial win.

**The likely shape:** F2 owns a **supervisory loop** built on `scheduler.tick()` + `pace()`, checking
the deadline between ticks — reusing C2's tick rather than re-implementing it — **and** a pacer
decorator as a **backstop** for code written against the frozen F6/F10 surface, which is every auton
the cookbook teaches. Rule whether you ship both. Shipping only the loop leaves the actual product
surface unprotected; shipping only the pacer inherits measurement 2's 28-second hole.

**Rule 4 obligation:** the pacer position (`cancel()`/`async()` from inside `pace()`) is legal by
precondition, undocumented, untested, and **absent from C2's "Re-entrancy (decided and pinned)" list**.
If F2 relies on it, F2 adds it to that list with tests. Relying on an unstated permission is how a
later chunk breaks you by tightening a precondition.

### T2 — what expiry DOES, and how many instants there are

Two different things want to happen at two different times: **stop scoring** (early enough to still
reach the end position) and **be safe** (unconditionally, at the buzzer).

**Rule for two instants, both caller-supplied, with NO defaults.** A default lead time would be an
invented number governing whether the robot scores — and HA-51's invented 5 s default is already the
cautionary tale in this same file. The hard floor fires **even if the end action is still running**.

**Safing is safety; going somewhere is strategy.** The library may own the unconditional stop once a
deadline exists at all. It must refuse to have an opinion about where to go.

### T3 — the end action is caller-supplied and composable

Per the naming section. **The library ships no field coordinate and no concrete park.** F2's test uses
a stand-in coordinate labelled as a fixture with no field claim; `endInMidfield` is F4's, with the real
numbers. Rule this explicitly, because measurement 7 makes a `parkAt()` convenience look harmless.

### T4 — the `waitUntil` hole, and the honest boundary

Measurements 2/3/5/6 define the problem precisely. **State the lateness as a formula F2 can budget
against:** *the unexpired remainder of the wait's own timeout at the instant the deadline fires, summed
over every wait/pause step executed after that instant.*

F2's **own** wait primitives must be deadline-aware. The **frozen F10 steps cannot be** without a
breaking change — so rule what happens there and **document the limit loudly**. And note measurement 6:
a deadline folded into a predicate returns `Satisfied`, which `waitFor` reads as success. **F2's
deadline needs its own chain-halting signal, distinct from both `Satisfied` and `WaitTimedOut`.**

### T5 — the verdict vocabulary (D1 aimed this at you by name)

There are now **seven** related vocabularies, and they are stratified by knowledge horizon rather than
redundant: `ExitReason` (what a motion's criteria can render) → `MotionOutcome` (what the boundary
additionally knows: `FaultAbort`, `Superseded`) → `RoutineStopCause` (what the chain knows), with
`MechanismOutcome` a peer at the motion tier.

So a shared step-outcome type would be a **fifth tier, not a replacement**. Rule whether F2 mints one.
If it does: **map in, never re-mean**, and mirror `Superseded`'s precedent — the outer layer records
why *it* cancelled, in its own vocabulary. Constraints: several of these are wire-stable/append-only
(E1 blackbox, F9 at H1), two live inside frozen surfaces, **`WaitResult::Satisfied` does not mean
success**, and `ExitReason` is the only one with no numeric pin anywhere — if you touch it, nothing
catches a reorder.

### T6 — the guard cannot reach the operations (an F1 gap)

F1 hands the guard a `span<hal::IMechanism*>` and says ops are "inert only after exit/cancel".
**But `cancel()` lives on `IMechanismOp`, and `IMechanism` has no path to its operation** — I grepped;
it is not there. Measurements 12/13 make this concrete: `applySafeState()` alone lasts two device
events.

Rule how the guard reaches operations. F11 is unfrozen, so amending F1's seam is legal and may be
correct — **fix it at the layer that owns it (Rule 4)** rather than working around it in F2. And
measurement 14 is the test-design warning: **assert voltage AND mode together**, or a mode-only
assertion goes green on an energized lift.

### T7 — the latch: the end state must be a standing claim, not an event

Measurement 9 is decisive: cancelling repeatedly is inert and slightly *worse* than nothing.
Measurement 11 shows the end action issued after the deadline is itself cancelled after one tick.

So once the deadline passes, **subsequent motions must be refused, not cancelled** — a latch — with the
end action exempt. Rule the shape, and note measurement 10: **where the check sits inside `pace()`
changes post-deadline travel from 10.79 in to 0.0000 in.** That ordering is load-bearing and must be
pinned by a test.

### T8 — how big is F2, and what belongs to F4

`build-order.md` demands four combinators; the master plan's v1 cut-list and `roadmap.md` both say the
combinator engine may be deferred and v1 may ship as *"hand-written blocking calls + one async handle +
a wall-clock guard"*. **Rule it explicitly** — the worst outcome is silently building four combinators
while two public documents say otherwise. The DoD is one test about the end action, not about
combinators.

**And the guardrail item, which is not optional:** `buildStack` / `matchLoadCycle` / `endInMidfield` /
`strategyMode` belong to **F4**, in hardware-gated Phase F′ (`build-order.md:1297`) — but
`roadmap.md:1113` lists all four inside the same "Sequencing (WS8)" block as the engine. **A reader
working WS8 top to bottom builds season content at F2.** That content is strategy the students must
author and defend. Fix the roadmap; record the split: **F2 owns *that* a caller-supplied end action
fires on a hard schedule; F4 owns *what it is*.**

Note also that **"possession-aware time budgeting"** is a game-flavoured description of a game-agnostic
mechanism. Implementing possession as a library concept repeats the exact legacy failure C6 catalogued.

---

## Scope

### In
1. The deadline **owner** and the expiry **policy** (T1/T2) — D-8 and the end-of-run action as one
   primitive.
2. A caller-supplied, composable **end action** (T3), with the unconditional safe floor.
3. Deadline-aware **wait primitives** of F2's own (T4), and the stated limit for frozen ones.
4. The **latch** (T7).
5. Whatever combinator set T8 rules, **built on `scheduler.tick()`**, not re-implementing C2's loop.
6. Reaching mechanism **operations** (T6) — fixed at F1's layer if that is where it belongs.
7. `verify-f2.sh`, carrying the F1–F5 register-row gate.

### Out
- **Season content** — `buildStack`, `matchLoadCycle`, `endInMidfield`, `strategyMode` → **F4/F′**.
- Any field coordinate or concrete park → **F4**.
- The command-id registry → **G2**. (D1 ruled G2 does not consume the recipe chain; do not re-open it.)
- `hal/pros`, real timing → **R1/R4**.
- Freezing anything.

### Explicitly rejected
- **"The pacer alone gives a guaranteed end-of-run action."** Measurement 2.
- **Cancel-only expiry.** Measurement 9 — inert, and slightly worse than nothing.
- **A throwing pacer.** Measurement 15 — 11.4 V under Coast.
- **A default lead time, default match length, or default park pose.**
- **Extending `RobotCommands`/`Command`** as master plan §14 line 576 instructs — C6 proved it had no
  executor and C7 **deleted** it. Fix that sentence (Rule 4).
- Building four combinators silently while two public docs say v1 may be smaller (T8).

---

## Test requirements

Every test names, in a comment, the bug it would catch.

- **THE DoD TEST:** a deliberately stalled scoring loop still ends with the end action performed,
  against the plant, with the clock driven to the match limit. **This test is the entire point of the
  chunk.** Stall it several ways: a motion that never settles, a mechanism that never confirms, a
  `waitFor` whose condition never comes true, a fault cascade.
- **The measured failures, as regression tests.** Each of measurements 2/3/6/8/9/11/12/13 becomes a
  test that goes red if F2's answer regresses to the naive one.
- **Post-deadline travel is bounded and pinned** (measurement 10's ordering).
- **The end action is not reported as the caller's verdict** (measurement 8 — a silent hijack with zero
  log lines is the worst outcome measured anywhere in this campaign).
- **Mechanisms end safe AND stay safe** — assert voltage *and* brake mode, at the device, then tick the
  op again and assert it did not re-energize (measurements 13/14).
- **The latch refuses post-deadline motions** while permitting the end action.
- **`Routine` chains written against the frozen surface** get whatever bound T1 rules, measured.
- **Nothing above the seam changed**: the existing suite stays green and unedited.

### Mutations
Disarm the latch; move the deadline check to the other side of the plant step (measurement 10); make
the end action reuse the caller's verdict; drop the mechanism-op cancel and keep `applySafeState`; make
the hard floor conditional on the end action having finished; make a deadline-aware wait report
`Satisfied`; break the ordering so the end action starts before the scoring loop is latched off.

**A mutation that stays GREEN is a hole — log it, close it with a test that fails alone, and give it a
prominent place in the record.** D3 found four, E1 two, E2 one, E3 three, E4 two, F1 zero across
eighteen. Gate the runner on build success; **never pipe it into `head`**.

### The recurring trap, in F2's form
Six chunks have been bitten by a test that shared a model with the code under test. **F2's version: if
the test's notion of "the match ended" is the same clock the guard reads, a guard that never fires and
a test that never checks agree perfectly.** Drive the deadline from an independent script, and assert
against **plant ground truth** — where the robot physically is when the clock hits the limit — not
against the guard's own account of what it did.

---

## Documentation

- **Chapter 6 (how things fail)** and **12 (when things go wrong)** — the routine-level bound is new
  user-visible behaviour.
- **Chapter 9 / the cookbook** — `pause`/`waitFor`'s deadline behaviour changes or is documented as
  limited; the cookbook already ships a hand-rolled match-clock recipe reaching through the Tier-3 seam
  (`03-timing-and-partners.md:99-130`). Either F2 replaces it or that recipe stays the honest answer —
  **say which**.
- **Chapter 14** — what "guaranteed" now means and, precisely, what it does not.
- **The honesty boundary, verbatim in the guide:** F2 proves a **scheduling property** — a stalled loop
  still ends with the end action, against the plant, with the clock driven to the limit. It **cannot**
  claim the timing margin is right on a real brain: real loop rate under load and PROS call latency are
  invented register entries until R4. **And nothing preempts pure user code that never calls into
  shulib** — there are no background tasks. Write that down rather than letting "guaranteed" do
  unearned work.
- **Roadmap** — the WS8/F4 split (T8), the renaming, and `roadmap.md:131`'s "F2's combinators + park
  guard" collision.
- **Rule 4 doc fixes:** `diagnostics-plan.md:184` mis-cites the D2 completion record for what is
  D3-COMPLETED §2.1; master plan §14:576 instructs extending deleted code; the F2 naming collision in
  `spec/accuracy.hpp` and `accuracy_spec_test.cpp` case names.
- **HA entries** for anything invented, from **HA-94**.

---

## Definition of Done

- [ ] T1–T8 ruled explicitly, each with its rejected alternative
- [ ] **The DoD test passes: a deliberately stalled scoring loop ends with the end action performed**,
      against the plant, clock driven to the match limit, stalled several ways
- [ ] Measurements 2/3/6/8/9/11/12/13 each have a regression test
- [ ] Post-deadline travel bounded and the `pace()` ordering pinned
- [ ] The end action never reported as the caller's verdict; never silent
- [ ] Mechanisms end safe and STAY safe — voltage and mode, at the device, after a further tick
- [ ] The latch refuses post-deadline motions; the end action is exempt
- [ ] D-8 discharged, or its remainder named with an owner
- [ ] The pacer position added to C2's re-entrancy list with tests, if used (Rule 4)
- [ ] `Chassis::waitUntil`'s missing `DetachGuard` ruled on — fixed or recorded with its reasoning
- [ ] No season content; no field coordinate; no default lead time, match length or park pose
- [ ] Nothing frozen; the register says so out loud
- [ ] Guide + cookbook + roadmap updated; the honesty boundary written verbatim
- [ ] `verify-f2.sh` written, carrying the F1–F5 register gate
- [ ] Suite green; both guards; ARM gate; all four doc gates

---

## Live progress log — required

`docs/internal/chunks/F2-PROGRESS.md`, created **first**, appended as work happens (`date +%H:%M:%S`).

---

## Documentation contract

All of the above, plus **`F2-COMPLETED.md`**. Give **T1, T4 and T7 their own sections** — they are the
three the measurements overturned, and a future reader who does not know that will re-derive the naive
design.

**State plainly what the guarantee covers and what it does not.** This chunk's headline claim is the
one most likely to be over-quoted, and the honest version is narrower than the phrase "guaranteed".

**Do not commit. Do not push.**

---

## Landmines

- **Row F2 is the LOCKED accuracy targets.** Do not touch rows F1–F5.
- **`MotionOptions{.timeout = 0}` means 5 seconds, not zero.** Clamping an end-action leg to `remaining`
  at T−0 silently buys five more seconds — past the buzzer.
- **`MotionOptions::validate()` throws on a negative timeout.** `remaining = deadline − now` after
  expiry kills the auton at the exact instant the guard exists to save it.
- **A stalled operation's unreleased claim makes the end action's own operation THROW at `start()`**
  (`mechanism_op.hpp:184`; the claim token is `mechanism.hpp`'s). Cancel-all must strictly precede act.
- **`applySafeState()` without `cancel()` lasts two device events.**
- **Never assert brake mode alone** — `brake=Hold, V=9.00` reads safe and is energized.
- **Do not let "guaranteed" outrun the evidence.** No background task exists; user code that never calls
  into shulib cannot be preempted.
- **F2 has no second consumer inside the library** (D1 ruled G2 out; `Routine` is frozen and eager; F4
  is hardware-gated). Say so, and do not freeze anything on one consumer's evidence.
