# Chunk C2 — COMPLETED (2026-08-06)

> Completion record for [`C2-motion-scheduler.md`](C2-motion-scheduler.md) — the
> `MotionScheduler`: the thing that actually runs a routine. Everything below is **as actually
> observed** — commands run, outputs captured, all 16 mutations executed and watched (the live
> sequence is in [`C2-PROGRESS.md`](C2-PROGRESS.md)). Changes are in the working tree,
> uncommitted, pending review, per the brief.
>
> **The headline:** the library now RUNS a routine instead of having one hand-rolled around it —
> and the formalized loop is provably the SAME loop C1's accuracy baseline was measured on:
> scheduled routines reproduce the hand-chained results **bit for bit** (clean n=5 final error
> 0.228175 in, identical to the digit; hostile identical likewise), pinned by an equivalence
> test that turned out to be the ONLY detector of a loop-shape mutation (§7 M12) — the C2
> analogue of C1's simultaneity pin. The chunk also discharges C1's three named handoffs
> (fault-reactive cancellation, `activeCommandId`, monitor ownership between motions) and a
> fourth nobody restated in the brief: `core/check.hpp` had already promised that "the motion
> scheduler catches PreconditionError at the task boundary" — that catch now exists and is
> tested (§2.4).

---

## 1. What was built

| Piece | File | Role |
|---|---|---|
| `MotionScheduler` | `include/shulib/motion/motion_scheduler.hpp` *(new, 626 lines)* | The formalized loop (localizer-first, per A2/C1), ONE active slot with pinned pre-empt semantics, `async()` / `tick()` / `waitUntilSettled()` / `waitUntil(pred, timeout)` / `cancel()`, the fault policy, the task-boundary catch, per-motion boundaries + counters for C5 |
| `ITickPacer` | same | The world-advance seam: host sim steps the plant, the robot delays to the next tick boundary — the scheduler observes time, never makes it (the IClock inversion, applied to the loop) |
| `WaitResult` | same | `Satisfied` / `TimedOut` — a DISTINCT vocabulary from `ExitReason`, so "the wait timed out" can never read as "the motion timed out" |
| `CommandIdStampSink` | same | ITelemetrySink decorator that stamps `DebugRecord.activeCommandId` for every record emitted while a motion is active — id assignment made unforgettable at the sink, `wantsRecord()` forwarded (the A1 pair rule) |
| `CompletedMotion` + counters | same | Per-motion boundary data (id, name, exit, causal abort fault, start/end time) + started/settled/timedOut/cancelled/aborted counts — C5's raw material, deliberately NOT formatted |
| `IMotion::cancel()` + the cancel contract | `include/shulib/motion/motion.hpp` *(modified)* | Pure-virtual by design (a motion type without a cancellation story is the forgettable-safety-step failure mode); full semantics in-header: running → safe state + Cancelled + exit record; already-exited → safe state, verdict PRESERVED; Idle → no-op; never raises |
| `applyCancelSafeState()` | same | THE safe state, defined once: `BrakeMode::Brake` then 0 V on every drive motor, synchronous. HA-53 carries the hardware claim |
| `MotionState::Cancelled = 5` | same | Appended (wire-stable, append-only), on the `activeCommandState` wire path |
| `ExitReason::Cancelled` | `include/shulib/control/exit_group.hpp` *(modified)* | Appended via the header's own documented additive path; `ExitGroup::check()` can never return it (documented) |
| `cancel()` implementations | `move_to_pose.hpp` (serves StrafeTo/HoldPose), `turn_to.hpp`, `drive_brake.hpp` *(modified)* | Identical contract per class; boot-window cancel emits a zero-error exit record (must not invent errors against a nonexistent estimate) |
| `FaultLatch::raiseCount(code)` | `include/shulib/diag/fault.hpp` *(modified, additive)* | Per-code tally (32 slots, saturating) — the scheduler's policy needs "raised SINCE THIS MOTION STARTED", and a since-clear bitmask cannot see a RE-raise (§5 D5) |
| Scheduler rig | `test/motion_test_rig.hpp` *(modified, additive)* | `PlantPacer` (hard-capped: a defeated wait bound turns the suite RED, never hangs it) + `SchedulerRig` (rig + pacer + scheduler, motions built from `sched.deps()`) |

**New tests:** `motion_scheduler_test.cpp` (35), `motion_scheduler_routine_test.cpp` (4),
`fault_test.cpp` (+1 — the `raiseCount` owner-side pins). **40 new cases / 1,320 new
assertions** (487/858,611 → **527/859,931**; skips unchanged at 3). Grading discipline
unchanged from C1: truth from `h.truePose()`, the estimate reserved for what the code read.

**No CI edits needed:** the scheduler lives in `include/shulib/motion/`, already inside both
guard scopes; the ARM gate's generated list picked the header up automatically (86 headers).

---

## 2. How C1's handoffs were resolved (§10 of C1-COMPLETED, plus one from check.hpp)

### 2.1 Fault-reactive cancellation → decided: **abort-and-brake on a NEW ODO_STUCK; continue-degraded on everything else** (configurable)
After each Running tick the scheduler compares per-code raise counts against a snapshot taken
at `async()`. A hit in `abortFaultMask` (default: ODO_STUCK only) cancels the motion into the
safe state, logs one Warn line naming the code, and records the causal fault on the boundary
(`CompletedMotion.abortFault`). The run continues — faults log and recover, never crash.

The per-code reasoning (each pinned by test):
- **ODO_STUCK → ABORT.** The spin-vs-motion cross-check says the estimate is LYING (dead
  encoder) or the drive is physically stalled. Continuing to servo against a lying estimate is
  worse than stopping — the controller pushes full authority toward a phantom target. Measured
  A/B (§8): the abort ends the dead-encoder run at **1.51 s with a 4.35 in** truth-vs-estimate
  gap; the policy-off twin rides the watchdog to **6.01 s and 42.0 in** — a ~10× damage cut.
- **IMU_LOST → CONTINUE.** C1 pinned Degraded-does-not-gate (encoders still good; freezing
  mid-run strands the robot). Pinned: a mid-motion dropout raises once and the motion settles.
- **BROWNOUT → CONTINUE.** Firmware already cut effective volts; the ESTIMATE is still honest.
  If the pack recovers, finishing is right; if not, the motion's watchdog bounds it. Pinned:
  the C1 brownout scenario through the scheduler ends TimedOut (not Cancelled), run continues.
- **GPS_GATE_REJECT / MOTOR_OVER_TEMP / LOOP_OVERRUN → CONTINUE** — a defended attack, degraded
  authority, and degraded timing respectively; none is a lying estimate; all visible + bounded.
- The mask is configurable because this is policy, not physics — pinned: abort-on-BROWNOUT
  when configured ends the nearly-dead-pack run Cancelled at < 1 s with the cause recorded.

**Snapshot semantics, pinned both ways:** a fault latched BEFORE the motion (stale) does not
abort it; a RE-raise during it does. This is why `FaultLatch::raiseCount` exists — designing
with a raised-since-clear bitmask, motion 2 would have been blind to a persisting dead encoder
after motion 1 latched the code (found at design time, before any code; §5 D5).

### 2.2 `activeCommandId` → assigned by the scheduler, stamped at the SINK
Ids are 1-based, monotonic per scheduler lifetime, assigned at `async()`. Rather than asking
every motion to remember to copy an id into its records (forgettable), the scheduler routes
motion telemetry through `CommandIdStampSink` — a decorator that stamps every record while a
motion is active and 0 between motions. Motions are constructed from `scheduler.deps()` (the
same `MotionDeps` with telemetry re-routed); the C4 facade will make that plumbing automatic
(§11 F6 flag). The stamp forwards `wantsRecord()` — the A1 pair rule — so a NullSink run still
skips record population entirely (pinned by a records-off probe; mutation M9's home). TermSink
needed NO change: the §18.3 line now genuinely prints `cmd#1▸2` (golden-pinned).

### 2.3 Monitor ownership between motions → the scheduler owns the gaps
Idle ticks (no active motion) run `localizer.update()`, tick the HealthMonitor with every
reachable observable (`odomStalled = false` — nothing commanded, no spin to cross-check: the
DriveBrake reasoning), and emit a quiet idle record (no invented target; renders `[LOC] idle`).
Pinned: an IMU dropout BETWEEN motions still raises IMU_LOST (mutation M10 red). The scheduler
also owns a LoopMonitor ticked every scheduler tick — hostile pacing surfaces as LOOP_OVERRUN
with the EXACT slow-tick count (pinned against a jitter schedule; mutation M13-adjacent) — and
re-baselines it at `async()`/wait entry, so user code between motions is a deliberate pause,
not a phantom overrun (loop_monitor.hpp's own reset semantics).

### 2.4 The handoff nobody restated: check.hpp's task-boundary catch
`core/check.hpp` (A1) already promised: on-robot, a `PreconditionError` thrown mid-motion is
"caught by the motion scheduler at the task boundary and converted to a FAULT_ABORT exit + a
safe drivetrain state." C2's tick loop IS that boundary, so the catch now exists — TIGHT
(PreconditionError only, wrapped around `active->tick()` only; a Localizer breach has no motion
boundary to unwind to and propagates). The nasty case is tested: a motion that energizes the
drive to 5 V and THEN throws is caught, safed (0 V + Brake), latched as Precondition (raised
exactly once — the faultCount snapshot avoids double-raising when the on-robot handler already
raised), recorded as an abort boundary, and the run continues. A motion calling the scheduler
from its own tick() is the same story via the re-entrancy preconditions (§5 D8).

### 2.5 The additive `ExitReason` extension → taken: `Cancelled`
Exactly the path C1 §10 item 5 named. `ExitGroup::check()` still renders only
Settled/TimedOut/Running; cancellation is imposed from outside — documented in the enum's own
header. `-Wswitch` under `-Werror` found the one switch needing a case (TurnTo's; marked
unreachable with the reason).

---

## 3. The routine numbers through the scheduler (the accuracy DoD)

**Bit-identity first (the strongest form of "reproduces C1's baseline"):** the same waypoint
generator, seeds, and cadence as C1's hand-chained suite, run BOTH ways on identical rigs —
hand loop with motions from raw deps vs `async()`+`waitUntilSettled()` with motions from
`scheduler.deps()`. Clean AND full-hostility: final error, total time, and every per-waypoint
error **compare equal as doubles** (n=5: clean finalErr 0.228175 in / 9.56 s; hostile seed 77
finalErr 1.549 in / 13.43 s — both paths identical to the bit). The scheduler added engine,
not physics.

**Clean sweep through the scheduler** (seed 77, X-drive, graded on truth — C1's table, again):

| n | motions | final err | worst arrival | time | distance |
|---|---|---|---|---|---|
| 5 | 6 | 0.228 in | 0.229 in | 9.56 s | 128 in |
| 10 | 13 | 0.00003 in | 0.229 in | 18.4 s | 167 in |
| 20 | 26 | 0.004 in | 0.238 in | 35.0 s | 316 in |
| 40 | 53 | 0.236 in | 0.238 in | 74.8 s | 789 in |

Error **flat in move count**, identical to C1's numbers to the digit (as the bit-identity test
guarantees). **Stop-and-settle overhead through the scheduler: 1.186 s/motion** — C1 measured
1.19 hand-chained; the engine adds nothing measurable. The Frontier blending item's price
stands unchanged.

**Full composed A3 hostility through the scheduler** (seeds 11/22 — C1's exact scenario):

| n | seed | final err | worst arrival | worst heading | time | distance |
|---|---|---|---|---|---|---|
| 5 | 11/22 | 1.00 / 0.63 in | 1.00 / 0.71 in | ~0.12° | ~12 s | ~100 in |
| 10 | 11/22 | 2.61 / 0.78 in | 2.61 / 0.78 in | ~0.13° | ~24 s | ~230 in |
| 20 | 11/22 | 4.13 / 0.64 in | 4.13 / 1.02 in | ~0.39° | ~45 s | ~410 in |
| 40 | 11/22 | 3.15 / 2.35 in | 4.13 / 2.77 in | 0.64–0.85° | ~92 s | ~880 in |

Worst anywhere **4.13 in / 1.19° — C1's numbers exactly**, inside the same derived 5.0 in bound
(pessimistic physics ceiling ~12 in). Engine bookkeeping is REQUIREd to agree with physics in
every routine: started == settled == the leg count, zero cancels, zero aborts.

**Seeded determinism (DoD item):** a 3-leg hostile routine, seed 33, run twice → the per-pace
`TruthSample` streams are **byte-identical (memcmp over the full run)**, final fused pose and
clock bit-equal, fault counts equal; seed 34 diverges. No wall-clock, no shared RNG, no
iteration-order dependence anywhere in the engine.

---

## 4. Flaws found and design catches (each handled where it lives)

### 4.1 FOUND at design time (before code): a raised-since-clear bitmask cannot implement the fault policy
The first policy sketch snapshot-and-compared a per-code "raised since clear()" bitmask. That
design is BLIND to a re-raise: after motion 1 latches ODO_STUCK, the bit is set forever, so a
persisting dead encoder could never abort motion 2. Fixed by designing the additive
`FaultLatch::raiseCount()` per-code tally instead (owning A1 file, owner-side pins added).
Mutation M16 (zero-filled snapshot) and the stale-vs-reraise test guard both directions.

### 4.2 FOUND + FIXED (test suite): the cancel-physics claim was VACUOUS on the default rig plant
The shared rig plant is memoryless (kA = 0) — at 0 V it stops INSTANTLY, so "cancel → the
drivetrain reaches rest" was true by construction (observed: coast distance exactly 0.000 in
from 60 in/s — the giveaway). Re-run on a LAGGED plant (kA = 0.03, τ ≈ 0.18 s): cancel from
56.7 in/s coasts **9.7 in to rest inside 1.5 s** on the zero-volt command alone, with a > 1 in
liveness floor added so the test can never quietly go vacuous again. Honest scope stands: the
plant does not model brake torque, so this is the CONSERVATIVE (coast-only) stopping story —
real Brake mode can only be shorter; HA-53 owns the hardware claim.

### 4.3 FOUND (A2 behaviour, surfaced by the first record-stream audit): the plant emits its own per-step records
The A2 `DrivePlant` emits one truth record per step through the harness sink — UPSTREAM of the
id stamp, so those records rightly carry id 0/state 0 and interleave with the motion's stamped
records. Not a defect (truth records are deliberately not command-attributed), but the first
whole-stream audit had to learn it: motion records are discriminated by `activeCommandState !=
0`. Recorded here because C5's per-motion grouping must know the stream carries TWO producers.

### 4.4 Development honesty: four first-run test failures, all test bugs, diagnosed and fixed
(1) the id audit assumed a single record producer (§4.3); (2) a pre-empt predicate was
satisfied-on-entry, so the pre-empted-into motion never provably drove (pred re-aimed at real
progress); (3) the stale-ODO retiming — the freeze originally landed after motion 2 had
settled/slowed below the stall check's spin threshold; (4) the boot test sampled "never moved"
AFTER the predicate observed Running — by which point the first live tick has legitimately
commanded (0.6 in = exactly one 60 in/s tick); motionlessness is now asserted DURING the wait,
from inside the predicate. No scheduler defects were found by these — recorded because the
diagnosis of (4) re-derived a real contract subtlety: predicates observe state BETWEEN ticks,
one tick late by construction.

### 4.5 The mutation campaign found no suite holes — and one mutation is detectable by exactly one test shape
All 16 mutations ran RED first try (§7); C2's analogue of C1's two GREEN holes did not occur —
partly because those lessons were designed in up front (bounded-loop waits, the vacuity check
in §4.2, the designed-detector pattern). The near-miss worth recording: M12 (motion ticks
BEFORE the estimate advances — a one-tick-stale controller) is INVISIBLE to every behavioural
test (closed loops converge on a stale estimate; slower, not wrong) and is caught ONLY by the
two bit-identity equivalence tests. Had those not existed, M12 would have been the campaign's
GREEN hole. They exist precisely because the accuracy baseline was defined as "the C1 loop,
verbatim".

---

## 5. Decision log (every choice with a viable alternative)

### D1 — Start-while-active = PRE-EMPT (cancel-then-swap), not reject, not queue
The old motion is `cancel()`led (safe state + object rendered inert) BEFORE the new one is
armed; no tick exists on which both command. Rejected: REJECT (a routine that forgot one
`waitUntilSettled()` silently SKIPS a move — the robot does the wrong thing quietly, the worst
failure class); QUEUE (a queue is F2's `Sequence` combinator in disguise — explicitly fenced
out — and a latent motion firing seconds later is scarier than last-command-wins).
`async(active motion)` is defined as a RESTART (cancel + full re-arm, new id) and pinned.

### D2 — One-at-a-time is structural in TWO layers, with the honest scope stated
Layer 1: the scheduler's single slot + cancel-before-swap. Layer 2: the cancelled OBJECT cannot
re-command even if a stale caller still ticks it (post-cancel ticks are contractual no-ops that
touch no motor — audited by test with the new motion's live voltages held across a stray tick).
Honest scope: direct Tier-3 `IMotion` use that never touches a scheduler remains possible by
design; C4's facade closes that door for library users.

### D3 — The cancel safe state: 0 V + `BrakeMode::Brake`, synchronous, defined ONCE
Coast rejected (a robot at speed keeps rolling — into the wall); closed-loop hold rejected (it
keeps servoing against the estimate, and the highest-priority cancel cause — ODO_STUCK — is
precisely "the estimate is lying"; holding would reproduce the failure cancel exists to stop).
Brake is estimate-independent (valid mid-boot, valid with dead encoders), passive, immediate.
Synchronous-in-the-call is load-bearing: a cancel that depends on someone continuing to tick is
a cancel that can leave motors energized (mutation M2's 7-case red). Brake mode is set BEFORE
the 0 V write so the stop lands under braking semantics. One function (`applyCancelSafeState`),
consumed by every path incl. the scheduler's no-active-motion PANIC stop — cancel always works.

### D4 — Verdict preservation: cancel never rewrites history; Idle is untouched
A settled motion really did settle; overwriting it with Cancelled would lie to C5's result
line. So: already-exited → safe state re-applied (idempotent), verdict PRESERVED (mutation M15
red); Idle → complete no-op (an unstarted motion has no relationship to the drivetrain).
cancel() raises NO fault — cancellation is a commanded act; when the cause IS a fault, that
fault is already latched and the boundary records it (`abortFault`).

### D5 — Fault policy mechanism: per-code `raiseCount` snapshots, not a bitmask, not live conditions
§4.1's catch. Also rejected: policy-on-live-conditions (e.g. a HealthMonitor `odoStuckActive()`
accessor) — it generalizes poorly beyond the odo code and couples the scheduler to monitor
internals; the latch tally serves any code, survives cascades (multiple codes in one tick), and
gives C5 per-code totals for free. The abort check runs AFTER the motion's tick, only on a
Running verdict — a motion that exits on the same tick a fault is raised keeps its own exit.

### D6 — `waitUntil` timeout is REQUIRED and explicit; a timeout is a Warn, not a fault
Rejected: a defaulted timeout constant (hides the bound at the call site AND would have been a
new invented register entry). Every bound is caller-visible; `>= 0` and finite by precondition;
0 is an honest poll. The return is a DISTINCT enum (`WaitResult`) so wait-timeout can never be
confused with motion-timeout. No fault on timeout — "wait for the ring, else move on" is a
legitimate strategy branch, and spamming the latch would bury real root causes; one Warn line
keeps it 2am-visible. `waitUntilSettled()` carries NO timeout parameter: every motion is
watchdog-bounded (C1, mutation-proven), so the wait is bounded by construction.

### D7 — The pacer seam + the stalled-pacer guard
The scheduler cannot own time (host: plant steps; robot: delay) — `ITickPacer` is the same
inversion as IClock. The guard: a pacer that never advances the clock freezes EVERY deadline
(watchdogs read the same frozen clock) — an un-catchable hang. After `kMaxStalledPaces` (100)
consecutive non-advancing paces the scheduler fails the precondition loudly. Pure logic
constant, host-decidable — deliberately NOT a register entry (the register is for hardware
claims; register rule 1). Rejected: trusting the pacer (the one hang no bound can catch);
per-pace strictness (a single equal-time read shouldn't kill a run).

### D8 — Re-entrancy: async/cancel from a predicate ALLOWED; blocking verbs and mid-tick calls REJECTED
Allowed because pre-emption composes cleanly from between ticks (the swap never lands inside a
tick — single-command invariant held, pinned). Blocking-from-predicate is disguised recursion
with user-data-dependent depth — rejected by precondition, and RECOVERABLY (the in-wait/in-tick
flags are RAII scopes, so the scheduler is usable after the throw; pinned). async/cancel from
inside a motion's tick mutates the slot under `active_->tick()` — rejected; a motion that does
it anyway is contained by the task-boundary catch (converted to a Precondition abort, pinned).
Relaxing any of these later is additive; un-forbidding is not.

### D9 — The task-boundary catch is TIGHT and single-raise
PreconditionError only (catch(...) would hide real bugs), around `active_->tick()` only
(a Localizer breach has no motion boundary to unwind to — it propagates). The
faultCount-unchanged check makes host (throw-only handler) and robot (raise-then-throw handler)
converge on exactly one latched Precondition. Rejected: always-raise (double-counts on-robot);
never-raise (host aborts would be invisible in the latch).

### D10 — Ids stamped at the sink via `scheduler.deps()`, unconditionally
A per-motion "setCommandId" virtual was rejected as the forgettable-safety-step anti-pattern
(every current and future motion must remember to stamp). The decorator stamps every record of
every producer downstream of it, forever. The overwrite is unconditional — this scheduler is
THE id assigner (debug_record.hpp says exactly that), so an incoming nonzero id would be a bug,
not information. Cost honesty: one record copy per emit, paid only when a sink actually wants
records (pair rule forwarded; mutation M9 red). Known gap, stated: a motion constructed from
RAW deps still schedules correctly but logs id 0 — observability-only degradation, closed
structurally at C4 (§11).

### D11 — `waitUntilSettled()` on nothing = vacuously Settled, with the disambiguator
The wait's contract is "block until no motion is active" — already true, so the wait succeeded;
returning the last exit (virgin default Settled) keeps the return type honest for the normal
case. The lie-risk is handled by `completedCount()` (0 ⇒ nothing actually ran) and pinned in
the zero-motion routine test. Rejected: an optional-like return (a third state every normal
call site must unwrap for the benefit of a degenerate case).

### D12 — The scheduler emits idle records but invents NOTHING in them
Continuity for the stream between motions (pose/quality/power, `[LOC] idle`), with targetPose
and errors left at their quiet defaults — an idle tick has no target, and DriveBrake's
"targets here" convention would mislabel genuine idleness. Lazy via `emitRecord` (A1 contract).

### D13 — LoopMonitor re-baselines at every resumption point (async / wait entry)
User code runs between motions for unknowable durations; measuring it as an overrun would fault
every routine. loop_monitor.hpp's reset() exists for exactly this. Consequence accepted and
documented: overruns are counted within continuous runs, not across user-code gaps; the
caller-paced `tick()` path measures continuously (the caller owns cadence there).

### D14 — Scheduler holds `IMotion&` by reference; not copyable/movable
Reference semantics match the tree (MotionDeps pointers); ownership stays with the routine
author, lifetime documented ("must outlive its scheduled run"). The scheduler itself is
self-referential (shadow context → own stamp sink) and therefore deleted-copy/move — pinned in
place by the type system rather than by convention.

---

## 6. Test inventory (35 + 4 + 1 — every case names its bug in-file)

**motion_scheduler_test.cpp (35)** — wire pins (Cancelled == 5; faultBit ↔ FaultCode values);
async returns without advancing the world / arms the motion (bug: blocking or dead async);
move→turn→strafe chain graded on truth with id/counter bookkeeping (state leakage across
motions); **pre-empt** (old motion inert + motors braked BEFORE the new one exists to command +
the stray-tick audit with live voltages held — THE two-motions bug; M1/M2's home); restart
semantics; **the cancel matrix** — mid-motion at 57 in/s on a LAGGED plant (braked by the call,
coasts 9.7 in to rest, exit record honest: state 5 / id 1 / zero command), before-any-tick,
boot-window (StrafeTo — the capture-at-live sibling — cancelled mid-calibration: truth never
moved, scheduler stays usable), after-settle (verdict preserved; panic stop; back-to-back
cancels), already-timed-out (TimedOut preserved); **waitUntilSettled** TimedOut-not-hang
(never-live IMU, ~150 paces) and the zero-motion vacuous case (idle records invent nothing);
**waitUntil** satisfied-mid-motion (the marker primitive), never-true → deadline-exact TimedOut
+ zero faults + one Warn line, true-on-entry/poll (zero ticks, zero paces), timeout-during-motion
(verdicts independent); **re-entrancy** (async-from-pred pre-empts cleanly; cancel-from-pred
allowed; blocking-from-pred rejected AND recoverable); **task boundary** (the 5V-then-throw
motion: safed, single Precondition, run continues; the scheduler-calling motion contained);
**fault policy** (the ODO_STUCK A/B twin — §8; IMU_LOST and BROWNOUT continue; stale-vs-reraise;
the configurable mask); **idle gap** (dropout between motions still raises); **loop monitor**
(overruns == jitter-schedule slow ticks EXACTLY); **ids** (per-motion stamping with the
two-producer stream discriminated; TermSink golden `cmd#1▸2`/`cmd#1▸3`); **cost contract**
(records-off probe through the stamp); **boot through the scheduler** (motionless during the
wait, asserted from inside the predicate); **the bit-identity equivalence** (scheduled ==
hand loop, exact doubles — M12's only detector); **the stalled pacer** (frozen clock →
PreconditionError at ~100 paces, never a hang).

**motion_scheduler_routine_test.cpp (4)** — scheduled == hand-chained bit-for-bit, clean AND
hostile (loop-shape drift); the clean 5/10/20/40 sweep through the scheduler (engine-added
per-move compounding) + the settle-overhead number; the hostile sweep with C1's derived bounds
(engine-added error under hostility) + engine bookkeeping REQUIREd to match physics; seeded
determinism (byte-identical truth streams via memcmp, diverging seed proves the comparison has
teeth).

**fault_test.cpp (+1)** — raiseCount tallies per code, survives cascades, None never counts,
out-of-range reads 0, clear() resets (the C2 policy's substrate, pinned at its owner).

Honesty notes: (1) the Brake-mode half of the safe state is pinned by STATE INSPECTION — the
plant does not model brake torque (HA-53 owns the physics; the 0 V half is physical, on a
lagged plant); (2) `HoldPose`'s cancel path is exercised only via the shared MoveToPose base
(its own cancel-specific case is the StrafeTo boot-window one); (3) the idle records' TermSink
rendering is covered by the golden test only incidentally (via interleaved `[LOC]` lines) —
C5 will pin idle-line formatting when it owns formatting.

---

## 7. Mutation checks (16 — each executed: break → build → run → OBSERVE → restore → re-green)

> Restores were done from cmp-verified pristine copies, never `git checkout` (the C1 process
> lesson — the C2 work itself was uncommitted). After the campaign: all 7 touched headers
> byte-identical to pristine (cmp), no mutation markers in `include/`/`test/` (grep), suite
> re-green 527/859,931. **Zero mutations stayed GREEN.**

| # | Mutation | Observed result |
|---|---|---|
| M1 | Pre-empt drops the old motion's `cancel()` | **RED** — 2 cases / 16 assertions: the pre-empt case (old motion still Running, motors NOT safed at the swap, stray-tick audit) + async-from-pred |
| M2 | `applyCancelSafeState` drops the 0 V command (cancel leaves motors energized) | **RED** — 7 cases / 30 assertions: every cancel path incl. the never-reaches-rest physics — THE robot-into-wall bug, comprehensively guarded |
| M3 | Safe state Coast instead of Brake | **RED** — 13 cases / 56 assertions (state-inspection detection; HA-53 owns the physics) |
| M4 | Fault policy dead (`newlyRaisedAbortFault` → None) | **RED** — 3 cases / 3 fatal REQUIREs: the A/B twin rode to the 6 s watchdog, re-raise, configurable mask |
| M5 | `waitUntil` deadline check dropped | **RED** — 5 cases / 6 assertions via the CAPPED pacer — red in bounded time, NOT a hung suite (the bounded-loop shape doing its job) |
| M6 | `waitUntil` timeout reported as Satisfied | **RED** — 5 cases / 5 assertions (distinguishability pins) |
| M7 | Entry check dropped (pred only after a tick) | **RED** — 1 case / 3 assertions: the true-on-entry/poll case, its designed detector (elsewhere one extra tick is slower, not wrong — the C1-#2 pattern) |
| M8 | Id stamp dropped (decorator forwards unstamped) | **RED** — 3 cases / 260 assertions: both ids cases + the cancel-record pin |
| M9 | Pair rule broken (`wantsRecord()` hardwired true) | **RED** — 1 case / 1 assertion: the records-off probe saw emit() fire — the A1 cost-contract detector |
| M10 | Idle HealthMonitor tick dropped | **RED** — 1 case / 1 assertion: the idle-gap IMU_LOST case |
| M11 | `localizer.update()` dropped from the tick | **RED** — 29 cases / 38 assertions: essentially every C2 case (motions servo a frozen estimate) — the loop's estimate-advance is maximally load-bearing |
| M12 | Loop shape broken: motion ticks BEFORE the estimate advances | **RED** — 2 cases / 40 assertions — **ONLY the two bit-identity equivalence tests**; every behavioural test converges on a one-tick-stale estimate. The equivalence tests are the sole guard against loop-shape drift (§4.5) |
| M13 | Stalled-pace guard removed | **RED** — 1 case / 2 assertions: the FrozenPacer went to its own cap (wrong exception type + count) — red in bounded time |
| M14 | Task-boundary catch removed | **RED** — 2 cases / 21 assertions: both boundary cases (the exception escaped; motors would have stayed at 5 V) |
| M15 | Cancel verdict-preservation removed (rewrites Settled/TimedOut) | **RED** — 2 cases / 5 assertions: cancel-after-exit + cancel-already-timed-out |
| M16 | Fault-count snapshot zero-filled (stale faults look new) | **RED** — 1 case / 1 assertion: the stale-vs-reraise case (motion 1 insta-aborted on the stale latch) |

---

## 8. The fault-policy A/B numbers (what the abort is worth)

Same dead-encoder world (both tracking channels freeze at t = 1.0 s), same 40 in MoveToPose,
same 6 s timeout — the only variable is the policy:

| | exit | ended at | truth-vs-estimate gap | drivetrain state |
|---|---|---|---|---|
| default policy (abort on ODO_STUCK) | Cancelled, abortFault = ODO_STUCK | **1.51 s** | **4.35 in** | braked (0 V + Brake) |
| `abortFaultMask = 0` (C1's behaviour) | TimedOut | 6.01 s | 42.0 in | stopped by the motion's own exit |

Reading: the stall window (~0.3 s) + one tick of policy latency ends the runaway ~0.5 s after
the freeze; the un-aborted twin drives at full authority against the frozen estimate for the
remaining 5 s. The abort converts "bounded by the watchdog" (C1's containment) into "bounded by
the detection window" — a ~10× damage cut on this scenario, measured, not asserted. C1's own
frozen-encoder test (motion WITHOUT a scheduler) still passes unchanged: the raise-not-abort
behaviour remains the motion-layer contract; the POLICY lives one layer up, where it belongs.

---

## 9. What we now know for certain, and what we do not

*(Written for a reader who was not here. "Certain" = proven by a passing, mutation-guarded test
against plant ground truth, across the swept space described.)*

**Now known for certain — on the A2 plant, under A3's hostile world:**
- **The library runs routines through an engine, and the engine adds no physics.** Scheduled
  routines are BIT-IDENTICAL to the hand-chained loops C1's accuracy baseline was measured on —
  clean 5→40-move chains stay flat in move count (0.23–0.24 in, tolerance-class), full-hostility
  worst stays 4.13 in / 1.19°, settle overhead stays ~1.19 s/motion. Every C1 §3 number carries
  over verbatim, by construction, and the equivalence pin is the only thing that can see the
  loop drift (mutation M12).
- **Two motions cannot both command the drivetrain through the scheduler.** One slot; starting
  while active pre-empts through the same defined cancel path; a pre-empted/cancelled motion
  OBJECT is inert (a stray tick touches no motor — audited). Pinned semantics: pre-empt, not
  reject, not queue; async of the active motion is a restart.
- **cancel() leaves a DEFINED, reached safe state — from one call.** 0 V + BrakeMode::Brake on
  every drive motor, synchronously; on a lagged plant the drive demonstrably decays to rest
  (9.7 in coast from 57 in/s, zero further ticks required). Every cancel shape is pinned:
  mid-motion at speed, before any tick, during the boot window (truth never moves; the
  scheduler stays usable), after settle/timeout (verdict preserved), back-to-back (idempotent),
  and with no active motion at all (the panic stop always works).
- **The fault policy exists and earns its keep.** A NEW ODO_STUCK during a motion aborts it
  into the safe state within the detection window — measured against the policy-off twin:
  1.51 s / 4.35 in vs 6.01 s / 42.0 in. IMU_LOST, BROWNOUT, GPS_GATE_REJECT, MOTOR_OVER_TEMP
  and LOOP_OVERRUN deliberately do NOT abort (each pinned), the mask is configurable, a stale
  latch cannot abort a healthy motion, and a re-raise still can.
- **No wait can hang, and the caller can always tell why it ended.** waitUntil requires an
  explicit finite timeout, checks the predicate before the first tick, distinguishes
  Satisfied/TimedOut, and times out to the deadline within one tick; waitUntilSettled is
  bounded by the motion watchdog; even a pacer that freezes time dies loudly at 100 stalled
  paces instead of spinning. Every one of these bounds was broken by a mutation and observed to
  turn the suite red in bounded time.
- **A mid-motion contract breach degrades ONE motion, never the auton.** The check.hpp
  task-boundary promise is implemented and tested with a motion that energizes the drive and
  then throws: caught, safed, latched once as Precondition, recorded as an abort boundary, run
  continues.
- **The record stream finally carries command identity.** Every record of motion k is stamped
  id k at the sink (unforgettably, for every motion type), idle records carry id 0, TermSink
  prints real `cmd#N▸state` lines, and the A1 cost contract survives the stamp (records-off
  stays records-off). The stream has TWO producers (motion records + the plant's truth records
  in sim), discriminated by state.
- **The monitors have no gaps.** Between motions the scheduler ticks the HealthMonitor (a
  dropout in the gap still faults) and a LoopMonitor whose overrun count matches a hostile
  jitter schedule exactly.
- **The whole engine is deterministic under seed** — byte-identical truth streams, run to run.

**NOT yet known — and who owns finding out:**
- **Whether Brake mode stops a real robot the way the plant's coast does.** HA-53: the host
  proves the 0 V half physically and the Brake half by state inspection only; R3 benches it,
  R5 measures stop distance from speed. Until then, plan around the measured COAST bound.
- **Whether ODO_STUCK's detection thresholds false-fire on real aggressive driving.** The
  abort policy raises the stakes on HA-52: a spurious ODO_STUCK now costs a motion (loudly,
  safely) instead of a log line. R3/R4 settle the thresholds; the mask can be relaxed per-team
  meanwhile.
- **What the fault policy should do about codes we chose to ride through.** BROWNOUT-continue
  and IMU_LOST-continue are reasoned, pinned choices on modeled physics; real pack collapse and
  real IMU death may argue differently. The mask is a config field precisely so R-phase
  experience can re-legislate without an API change.
- **Scheduler-on-hardware timing.** The pacer's robot-side implementation (delay-to-boundary)
  is R1/R3 work; LoopMonitor budgets (0.015 s, an A1-era default) meet real V5 scheduling
  there. Host jitter tests prove the accounting, not the platform.
- **Everything C1 already listed** — gains/tolerances (HA-50/51), the true drift story, the
  strafe-authority reading (C3), real braking physics — unchanged by this chunk.
- **Blending remains out by design**; its measured price (~1.19 s/motion) now stands confirmed
  through the engine that would host it (Frontier).

---

## 10. Deliberately left for later chunks (named handoffs)

- **→ C3 (`HDriveKinematics`)**: unchanged from C1 — confirm D11's strafe-authority reading;
  `strafeFallbackActive` telemetry. The scheduler is drivetrain-agnostic (nothing here reads
  kinematics beyond wheelCount for records).
- **→ C4 (`Chassis` facade)**: construct BOTH the scheduler and every motion from ONE
  composition root so `scheduler.deps()` plumbing (the id stamp) is structural rather than
  remembered (§5 D10's known gap); decide whether the facade's blocking verbs simply wrap
  `async` + `waitUntilSettled`; own the robot-side `ITickPacer` (or the background-task
  alternative, which would be the first two-task design decision in the tree — flag it loudly).
- **→ C5 (results/summary)**: everything is staged — `CompletedMotion` (id/name/exit/abortFault/
  start/end), the per-exit counters, `LoopMonitor::worstDt()`, per-code `raiseCount`, stamped
  ids in the stream. C5 formats; if it needs a full per-motion HISTORY (not just the last
  boundary), add a fixed-capacity ring THERE — deliberately not built now (no consumer, no
  capacity decision).
- **→ F2 (combinators)**: `Sequence`/`Parallel`/`Race`/`Deadline` compose ON TOP of one
  scheduler slot — the one-active-motion invariant is theirs to build against, not relax.
- **→ G2 (`PathRunner`)**: `waitUntil(pred, timeout)` is its marker/callback primitive, with
  re-entrant async/cancel-from-predicate pinned as allowed.
- **→ Phase R**: the robot pacer (R1/R3); HA-53's bench measurements (R3/R5); the policy mask
  revisit with real fault statistics (R5/R6).
- **→ Frontier**: blending (measured cost stands); a possible `waitUntil` sugar for
  pose-crossing predicates once G2 shows the common shapes.

---

## 11. Freeze Register note (documentation contract #6)

**No freeze at C2.** F6 flags — the facade inherits these shapes at C4 and D2 freezes them, so
every one is named now while changing it is still free:

- **The verb set and signatures**: `async(IMotion&)` / `tick()` / `waitUntilSettled()` /
  `waitUntil(Pred&&, double timeoutSeconds)` / `cancel()`, plus the observability accessors
  (`hasActiveMotion`, `activeCommandId`, `lastExitReason`, `lastCompleted`, the five counters).
  These are the direct ancestors of the facade's public API.
- **The pre-empt semantics** (start-while-active cancels first) becomes facade-level API
  behaviour — F6's documentation must carry it, because user code WILL depend on it.
- **`WaitResult`** is a new public vocabulary the facade will re-export; two values, append-only
  if it ever grows.
- **`ExitReason::Cancelled` / `MotionState::Cancelled`(=5)**: the latter is wire-stable NOW
  (activeCommandState; F9 serializes at H1). The former is source-stable vocabulary every
  consumer switches over.
- **`ITickPacer`**: the loop-ownership seam. If C4 chooses a background task instead of
  caller-pacing, this is the seam it replaces — decide BEFORE D2, not after.
- **`MotionSchedulerConfig.abortFaultMask`**: policy surface the facade must expose (a team
  legitimately re-legislates it); the `faultBit` helper is its stable spelling.
- **`MotionDeps` via `scheduler.deps()`**: the stamped-deps pattern is the one C2 shape that is
  convention rather than structure (D10) — C4 exists to make it structural. Absorbing it
  accidentally would freeze a footgun.
- Additive changes to earlier contracts, all via documented paths: `FaultLatch::raiseCount`
  (A1 file, owner-pinned), `ExitReason` + `MotionState` appends, `IMotion::cancel()` (a new
  pure-virtual member on a contract with exactly five implementers, all in-tree — this is the
  LAST cheap moment for that kind of change; after F6 freezes, a new IMotion member is a
  version bump).

---

## 12. Verification (actually run, outputs as observed)

```text
$ cmake --build build/test && ./build/test/shulib_tests
[doctest] test cases:    527 |    527 passed | 0 failed | 3 skipped
[doctest] assertions: 859931 | 859931 passed | 0 failed |
[doctest] Status: SUCCESS!
```
(3 skipped = the two M3 acceptance stubs + the R3 GPS field-cal oracle (HA-01), unchanged.)

```text
$ <the ci.yml PROS-free guard grep, scope unchanged — motion/ already covered>
GUARD 1 PASS: core is PROS-free (incl. motion_scheduler)
$ <the ci.yml layering guard grep, scope unchanged>
GUARD 2 PASS: layering holds, core is sim-free
```

```text
$ arm-none-eabi-g++ -std=gnu++20 -Wall -Wextra -Wconversion -Wsign-conversion -Wshadow \
    -Werror -Os -mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=softfp \
    -c all_headers.cpp -o /dev/null -Iinclude        # generated list, ALL v2 headers
ARM CROSS-COMPILE: CLEAN (86 headers)
```

Working tree left uncommitted for review, per the brief. Post-mutation integrity: all touched
headers cmp-identical to pre-campaign pristine copies; the only "MUTATION" grep hits in
`include/` are pre-existing doc comments (truth_integrator, loop_monitor, motor_model) plus
this chunk's own prose references to the campaign.

---

## 13. DoD checklist (brief §Definition of Done)

- [x] **`MotionScheduler` with `async()` / `waitUntilSettled()` / `waitUntil(pred)` /
  `cancel()`** — `include/shulib/motion/motion_scheduler.hpp`, plus the caller-paced `tick()`
  the facade's non-blocking mode will need.
- [x] **One-active-motion enforced structurally; semantics pinned and justified** — single slot
  + pre-empt-with-cancel + object-level inertness (D1/D2); pinned by the pre-empt/restart/
  stray-tick cases; mutations M1/M2 red.
- [x] **`cancel()` leaves a defined, tested safe state in every case** — 0 V + Brake,
  synchronous, ONE definition (HA-53); the full cancel matrix incl. boot-window, first-tick,
  post-exit, back-to-back, panic; reaches rest on a lagged plant (the vacuity trap found and
  closed, §4.2); mutations M2/M3/M15 red.
- [x] **Fault policy decided, documented, and tested per fault class** — abort on new
  ODO_STUCK (A/B-measured ~10× damage cut), continue on IMU_LOST/BROWNOUT/GATE_REJECT/
  OVER_TEMP/LOOP_OVERRUN, configurable mask, stale-vs-reraise snapshot semantics; mutations
  M4/M16 red. Plus the check.hpp Precondition task-boundary conversion (M14 red).
- [x] **No wait can hang — every wait bounded, with a distinguishable timeout return** —
  explicit required timeouts + `WaitResult`, watchdog-bounded settle, entry-check, poll,
  stalled-pacer guard; mutations M5/M6/M7/M13 red, each observed red in BOUNDED time.
- [x] **A full routine through the scheduler reproduces C1's accuracy baseline** — bit-identical
  to the hand-chained twin (clean AND hostile), full 5/10/20/40 sweeps re-run through the
  engine with C1's numbers to the digit (§3); mutation M12 (loop drift) red via exactly these.
- [x] **Survives A3 composed hostility; deterministic under seed** — the hostile sweep (worst
  4.13 in, zero aborts, bookkeeping REQUIREd consistent) + byte-identical same-seed replay,
  diverging different-seed control.
- [x] **Any new constant carries an `HA-nn` register entry** — HA-53 (cancel safe-state
  efficacy; register 52 → 53, reconciled both directions); the stalled-pace count is a pure
  logic constant and deliberately NOT registered (register rule 1); no other constants were
  invented (waitUntil deliberately has no default timeout — D6).
- [x] **Suite green under strict `-Werror`; both CI guards pass; ARM gate passes** —
  527/859,931; both guards (scopes unchanged, motion/ already covered); 86/86 headers (§12).

Beyond the brief, at the escalated bar: the bit-identity equivalence pair (the only detector of
loop-shape drift, proven by mutation), the fault-policy A/B damage measurement, the
task-boundary catch with a motors-hot breach, 16 mutations with zero green survivors, and the
§4.2 vacuity catch that turned a would-be-hollow physics claim into a real one.
