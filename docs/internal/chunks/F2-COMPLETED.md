# Chunk F2 — COMPLETED (2026-08-13)

**The sequence engine and the guaranteed end-of-run action.** Phase F's host-provable half
closes with this chunk. Everything is in the working tree, uncommitted, per the brief.

**The headline, sized to the evidence, before anything else:** F2 proves a **scheduling
property** — a deliberately stalled scoring loop still ends with the caller's end action
performed, against the plant, with the clock driven to the match limit by an independent
script, stalled four different ways. It does NOT prove the timing margin is right on a real
brain (loop rate under load and PROS latency are unmeasured until R4), it CANNOT preempt pure
user code (no background tasks exist — a standing decision), and it CANNOT cut the frozen
F10/F6 waits (they pay their unexpired remainder — the formula is documented and pinned).
The word "guaranteed" covers exactly the first sentence and nothing more; guide chapter 14
carries the boundary verbatim, and every doc that uses the word points there.

---

## 1. What this chunk actually did

- **`include/shulib/sequence/run_guard.hpp`** (new, the only new header): `RunGuard` — a
  run-scoped deadline owner that IS an `ITickPacer` decorator; `RunGuardConfig` (two
  caller-supplied instants + a mechanism span, all validated, **no defaults**);
  `RunGuardReport` (the guard's own account, separate from every caller verdict);
  `GuardedWaitResult` (the minted sequence-tier wait verdict). D-8 and the end-of-run action
  are ONE primitive here, two policies: latch-and-report, then act.
- **Rule 4 fixes in three earlier layers** (§8): the F1 claim token grew a claimant hook, the
  F1 operations grew cancel-on-destruction + deleted copies, and C2's blocking waits grew
  unwind guards. Each fixed at the layer that owns it, each with the found defect recorded.
- **Tests:** 43 new cases across five new files (`sequence_run_guard_test.cpp`,
  `sequence_end_of_run_test.cpp`, `sequence_example_test.cpp`,
  `mechanism_claim_driver_test.cpp`, `motion_scheduler_unwind_test.cpp`), every case naming
  the bug it would catch. No existing test was weakened; the two edits to existing test files
  are the brief-ordered accuracy-spec case renames (§9) — behavior untouched.
- **Docs:** guide chapters 6, 9, 12, 14; the cookbook match-window recipe; roadmap (register
  row F12, the F11 cell, the WS8 split, you-are-here); master plan §14's dead-code sentence;
  diagnostics-plan D-8 (discharged, remainder named); legacy-vocabulary chunk attributions;
  build-order (Current position + the F2 entry's RULED block); the guide maintenance ledger.
- **`docs/internal/verify/verify-f2.sh`** — the chunk's own harness (the reviewer's
  `verify-f1.sh` untouched), carrying the F1–F5 register gate and the 14-mutation runner.

**Freezes: NONE.** Register row F12 states it out loud. F2 has no second consumer inside the
library (D1 ruled G2 out; `Routine` is frozen and eager; F4 is hardware-gated) — nothing may
freeze on one consumer's evidence, and nothing did.

**HA entries: NONE — and that is the finding, not an omission.** The design's entire point is
that every number a team could get wrong (lead time, match length, end position) is
caller-supplied. The one new constant (`kPauseBackstopSeconds`) is unreachable slack with the
same non-HA justification as `Chassis::kWaitBackstopSeconds`, documented in place. The next
free number remains **HA-94**.

---

## 2. T1 — what fires the guard *(a measurement overturned the obvious design)*

**The naive design** — a pacer that cancels at the deadline — was killed twice by the brief's
measurements before a line was written: measurement 2 (a pacer cannot unwind
`waitUntil`-shaped waits: 2,801 cancels fired into one, all invisible, 28 s late) and
measurement 9 (cancel-at-the-deadline is INERT against a retrying caller — 300 cancel
boundaries bought telemetry while the run *arrived sooner*, because cancel/restart resets the
deceleration).

**Ruled:** ONE control seam — the pacer decorator — but doing three jobs the naive version
lacked: the **cut** (cancel-from-pace unwinds `waitUntilSettled` with zero latency —
measurement 1's mechanism, now legal-by-pin), the **latch** (every post-deadline motion is
cancelled before the world advances — refusal, not a race), and the **floor**. Deadline
awareness for waits comes from a **composite predicate inside C2's own `waitUntil`**
(measurement 5's shape) with the verdict disambiguated *after* the wait (measurement 6's trap
closed). The end action runs in the caller's own call context through the frozen facade's
ordinary blocking verbs.

**The rejected alternative, named: the brief's own sketched supervisory `scheduler.tick()`
loop.** Building one would have made F2 a second loop owner duplicating what C2's waits
already do — the composite-predicate wait delivers the same zero-latency deadline (measured
0.0000 s in the probe, ± one pace in the tests) inside the existing guarded loop. F2 ships
**zero new loops**. Also rejected: `async()`-from-`pace()` to drive the end action home
(measurement 7 proved it WORKS and measurement 8 proved it LIES — the caller's cut `moveTo`
returned `Settled` describing a park it never issued, zero log lines; the worst outcome
measured in the campaign). A consequence worth stating: because no supervisory loop exists,
**`TickPhase::User` still has no producer** — F1 named "F2's sequencer loop" as a candidate
and that candidate is now retired; G2's marker path is the remaining named owner.

**Rule 4 discharge:** cancel-from-pace was precondition-legal, undocumented, untested, and
absent from C2's pinned re-entrancy list. It is now IN the list (`motion_scheduler.hpp`
banner), on `cancel()`'s own doc comment, and pinned by
`test/motion_scheduler_unwind_test.cpp`'s "cancel() from inside pace()" case — plus mutation
M13 (tighten the precondition) observed red, so a later chunk cannot break the guard silently.
The same banner records async-from-pace as measured-but-rejected, with the reason.

---

## 3. T4 — the waits, and the honest boundary *(the second overturned design)*

**What cannot work, measured:** a scheduler-level wait checks its predicate and its own
timeout — nothing else. No outside actor can end it. The frozen surfaces (`Routine::pause`,
`Routine::waitFor`, `Chassis::wait`, `Chassis::waitUntil`) therefore CANNOT be deadline-aware
without a breaking change, and F2 does not pretend otherwise.

**The lateness bound, stated as the budgetable formula and pinned by tests:** *the unexpired
remainder of the wait's own timeout at the instant the deadline fires, summed over every
wait/pause step executed after that instant.* A `Routine` normally pays ONE term — its first
post-deadline motion is refused, which stops the chain and turns every later step into an
instant skip (`report.postExpiryCancels == 1` in the compiled example is this fact);
consecutive `pause` steps each pay in full because a pause cannot fail. Two regression tests
pin the bound from both sides — including the direction nobody expects: **if the frozen wait's
lateness ever SHRINKS, the tests go red so the documentation gets rewritten rather than
silently outgrown.**

**F2's own waits** (`guard.waitFor` / `guard.pause`) are deadline-aware, phase-aware
(`endActionAt` while scoring, `hardStopAt` during the end action — so the end action can wait
toward the buzzer), and return the minted verdict where **`RunExpired` wins a tie with
`Satisfied`** — measurement 6's trap: a deadline folded into a plain predicate returns
`Satisfied`, `Routine::waitFor` maps that to success, and the chain keeps scoring past the
buzzer. The tie-break is pinned by test and by mutation M7. The latch applies to waits too:
after the deadline the predicate is never called again — **including mid-flight**, which is
exactly where the campaign found its one green mutation (§10).

**Rejected alternatives, named:** a throwing pacer (measurement 15 — 11.4 V under Coast); a
"fixed" `Routine::pause` (breaking change to F10); folding the deadline into predicates and
living with `Satisfied` (measurement 6).

---

## 4. T7 — the latch *(the third overturned design)*

**Measurement 9 is the decisive one** and deserves restating: cancelling a post-deadline
motion once per pace, with no memory, made the run *arrive at its target sooner* than doing
nothing at all (80.55 in travelled vs 80.37 in; t=4.00 s vs t=4.48 s) — cancel/restart resets
the deceleration profile. Expiry must be a **standing claim, not an event**.

**Ruled:** once `endActionAt` passes, every motion outside the end-action window is cancelled
at the next `pace()` — *before* the world advances. Measurement 10 made that ordering
load-bearing: check-then-step measured 0.0000 in of post-deadline travel, step-then-check
10.79 in. The pin is as strong as the plant allows: **exactly zero** (`posErr == 0.0`, no
epsilon) across a 40-retry storm, because the plant is memoryless and the cancel lands before
the step — mutation M2 (flip the order) goes red on it. The **end action is exempt** via the
in-end-action window (measurement 11: without the exemption its own motion dies after one
tick); the **floor exempts nothing** and re-safes every pace, so nothing a predicate
re-commands outlives one pace boundary. `run()` returning **disarms** the guard — a latch
that outlived the run would refuse the next mode's first verb (pinned by test; mutation M9).

**Refusals are counted and throttled-loud:** first refusal Warns, the rest are counted into
`RunGuardReport::postExpiryCancels` (the measured 300-boundary telemetry storm is the reason
for the throttle — and the count is the honest record of a retry loop the guard cannot end).

**Rejected alternatives, named:** cancel-only expiry (measurement 9); refusal-by-throw (a
precondition is for programming errors, and a throw from a pacer is measurement 15).

---

## 5. T2 / T3 — two instants, a callable act, and no numbers anywhere

**T2 ruled:** two caller-supplied instants — `endActionAt` (stop scoring, act) and
`hardStopAt` (be safe, unconditionally, even mid-end-action; pinned by test and mutation M6).
Both required, validated finite/ordered, measured from `run()` start; equal instants are legal
zero-runway. **No default for either** — a default lead time is an invented number governing
whether the robot scores, and HA-51's invented 5 s default is this file's cautionary tale
(landmine dodged in the tests: an end-action leg with `timeout = 0` would silently buy the
5-second config default, so the examples all carry explicit timeouts and the floor bounds
whatever remains). Safing is the library's once a deadline exists; going somewhere is
strategy and stays the caller's. *Rejected:* one instant (an end action with zero runway is
refused by the floor — measurement 11 in different clothes); any default.

**T3 ruled:** the end action is a callable accepting then()'s exact four return conventions
(`void`/`bool`/`ExitReason`/`MechanismOutcome` — no fifth convention invented), composable by
construction (§14's park + Toggle re-verify is one lambda with two acts). Its verdict lands in
`RunGuardReport` and the `SEQ` transcript — **never in any motion verdict the scoring code
saw, and never silently** (measurement 8's silent hijack is the named enemy; mutations M3 and
M4 cover both halves). *Rejected:* a `parkAt(Pose2d)` convenience — measurement 7 makes it
look harmless and it would put a pose, and therefore a season, into the library. The test
fixtures' poses are labelled as fixtures with no field claim.

---

## 6. T5 — the verdict vocabulary (D1 §2.7 answered)

D1 flagged that F2 would span three result vocabularies and asked whether a shared
step-outcome type should exist below the recipe layer. **Ruled: no shared type; one minimal
addition.** `GuardedWaitResult{Satisfied, TimedOut, RunExpired}` exists because no current
vocabulary can say "the RUN's budget expired" distinctly from "this wait's own timeout
elapsed" — `WaitResult` has no such value and adding one would re-mean a frozen-adjacent C2
type; `RoutineStopCause` is Tier-2's and wire-adjacent. The addition maps in and never
re-means: `Satisfied`/`TimedOut` correspond 1:1 with `WaitResult`'s values; `RunExpired` is
the guard's own knowledge, exactly `Superseded`'s precedent (the outer layer records why *it*
stopped things, in its own words). The count of related vocabularies goes from seven to eight;
that is the honest cost, paid for one reason: **measurement 6 proves the alternative reads
"time ran out" as "the thing I waited for arrived".** No numeric pin: the enum is not
wire-facing (blackbox/F9 untouched). *Rejected:* the fifth-tier shared step-outcome type — a
unification with one consumer would be speculative vocabulary, D1's exact warning inverted.

---

## 7. T6 — reaching the operations (an F1 gap, fixed at F1's layer)

F1 promised the guard a `span<hal::IMechanism*>`; building the guard found the promise one
capability short: the claim said THAT a mechanism was driven, not BY WHAT, so a stalled
operation was unreachable — and measurements 12/13/14 showed why unreachable is unacceptable
(`applySafeState()` survives one tick of a live op; the re-command restores voltage but not
brake mode — `brake=Hold, V=9.0`, green under any mode-only assertion; the unreleased claim
throws the end action's own `start()`).

**Ruled, at F1's layer (F11 unfrozen):** `hal::ICancellable` (one member: `cancel()`),
declared in `hal/mechanism.hpp` so the claim can carry it without an upward include;
`IMechanismOp` inherits it (its `cancel()` was already exactly this contract);
`tryClaim(ICancellable&)` registers, `claimant()` exposes, `releaseClaim()` clears; both
library operations register themselves. Cancel-all walks the span: claimant → `cancel()`
(inert + safe + released); anonymous claim → force-release + Warn + count (the guard cannot
make an unknown operation inert; releasing beats a guaranteed end-action throw, and the Warn
names the mechanism); `applySafeState()` unconditionally after either. Bare `tryClaim()`
survives untouched — F1's tests run unedited — documented as invisible-to-the-guard.
*Rejected:* a caller-supplied `span<IMechanismOp*>` (the cookbook idiom builds operations on
the stack inside `then()` lambdas — a long-lived span cannot hold them, and a forgotten op is
a silent re-energize); reshaping `tryClaim()` in place (would have edited F1's tests, and the
anonymous form is legitimately useful to third-party ops that accept the tradeoff).

**Every mechanism-safety assertion in the new suites checks voltage AND brake mode together,
at the device, and then ticks the op once more** — measurement 14 is why a mode-only check is
banned, and mutation M5 (repaint without cancel) is red against exactly this.

---

## 8. The Rule 4 ledger — defects found in earlier chunks, fixed there

1. **F1: an operation destroyed mid-flight left its mechanism claimed forever AND energized.**
   The cookbook's own idiom with a mismatched budget (the wait gives up before the op's
   watchdog) destructs a Running operation; no code path stopped it — last commanded voltage
   persists at the device, claim stuck, every later `start()` on that mechanism throws. Fixed
   with rule-of-three discipline in `mechanism_op.hpp`: destructors cancel iff
   `started && !finished` (a FINISHED discrete op is deliberately untouched — an unconditional
   cancel would un-grab every successful clamp at scope exit, the T4-persist rule), and
   copy/move deleted on both operations (the claim is a resource; the implicit copy was always
   a latent double-release). Pinned by four cases in `mechanism_claim_driver_test.cpp` and
   mutation M11.
2. **C2: both blocking waits could strand an armed motion on unwind** — measurement 15's
   11.4 V-under-Coast with the slot pointing at a dying stack object. C4's `DetachGuard` was
   the symptom patch (facade verbs only); `Chassis::waitUntil` and every direct Tier-3 wait
   had the hole. Fixed at the loop owner: `WaitUnwindGuard` in both waits cancels (safe state,
   boundary recorded, slot cleared) before the exception propagates; the facade's guard stays
   (idempotent belt-and-braces). This ruling discharges the brief's
   "`Chassis::waitUntil`'s missing DetachGuard" DoD item — fixed one layer DOWN from where the
   symptom was named, because that is where every instance of the hole lives at once. Pinned
   by three cases (throwing predicate, direct stalled-pacer, and the anti-overreach case: a
   NORMAL `waitUntil` exit must leave the active motion running) and mutation M14. The
   existing C4 facade test passes unedited over the new behavior.
3. **Docs:** master plan §14 instructed extending `RobotCommands`/`Command` — C6 proved it
   never had an executor, C7 deleted it; the sentence now records both facts and the shipped
   shape. `diagnostics-plan.md` D-8 mis-cited the D2 completion record for D3 §2.1's opt-in
   ruling — corrected inside the discharged entry. `legacy-command-vocabulary.md` called F2
   "(mechanism primitives)" — two mistakes, both named in place; its seat/settle-wiggle
   pointers now say F3/F′ with the C6-era name noted. The accuracy-spec collision: 
   `spec/accuracy.hpp`'s title and `accuracy_spec_test.cpp`'s case names now say REGISTER ROW
   F2 explicitly (the brief ordered these renames; no assertion changed).

---

## 9. Test evidence

Suite: **1017 cases / 1,522,314 assertions / 3 deliberate skips** (from 974 / 1,522,018).
43 new cases in five new files; the only edits to existing test files are the brief-ordered
accuracy-spec renames. Assertion counts flatter; the mutation table below is the measure.

**The DoD suite** (`sequence_end_of_run_test.cpp`): the trap-counter is structural — the
match limit is driven by `TallyPacer`, which counts its own paces at its own dt and reads *no
clock and no guard state*; every "it parked" claim is `posErr(h.truePose(), fixture)` (the A2
truth integrator, independent of the estimate by design) plus raw device state (voltage AND
mode, drive and mechanisms), snapshotted at the tally's own limit crossing. Four stall
shapes: a never-settling motion, a never-confirming mechanism (its op cancelled through the
claim, then ticked once more at the end and asserted inert), a never-true wait, and a
fault-abort cascade (scripted ODO_STUCK raises — scripted BECAUSE hostile-sensor cascades
corrupt the estimate by design, which would make the plant-truth park assertion meaningless;
>10 fault-aborted attempts asserted, retries refused, park still reached).

**Two honest test-design failures found on the way, recorded because they are instructive:**
the first M10-pin draft used an uncapped 300 in leg that *settled at t≈5.3 s* — the "stall"
arrived early and the test measured settle chatter between legitimate motions (0.22 in) and
briefly blamed the guard for it; and `startAt(-48,-24)` seeds the ESTIMATE while plant truth
starts at the origin, so truth-graded examples must start at the origin or configure the
plant's initial pose. Both were test bugs; the probe that exonerated the engine (refusals add
exactly 0.000000 in) is in the progress log at 06:32.

**Measured-failure regressions, mapped:** M2 → two frozen-wait-remainder tests (red in either
direction); M3 → the pause-lateness-formula tests (Routine pays one remainder + the report
agrees); M6 → the tie-break, the then()-idiom chain-halt, and DoD stall 2's
`RunExpired`-not-`Satisfied`; M8 → the verdict-honesty case (cut verb returns `Cancelled`;
`SEQ` lines asserted present); M9 → the 30-retry storm (zero travel, 31 counted); M10 → the
exact-zero ordering pin; M11 → the exemption cases; M12/13/14 → the floor/claim/stay-safe
cases and the claim-hook unit suite.

---

## 10. Mutations — 14 run, and the one that came back GREEN

Runner: `verify-f2.sh` §7 — each mutation applied to the real header by exact-match replace,
**rebuilt** (gated: a BUILD-FAIL proves nothing and is counted separately), run against the
F2-scoped suite, restored, restoration verified byte-identical (`cmp`, never `git checkout` —
the tree holds uncommitted work). PIPE trapped; nothing piped into `head`.

| # | Mutation | Result |
|---|---|---|
| M1 | latch disarmed (no post-deadline cancel) | RED (15) |
| M2 | deadline check moved AFTER the plant step | RED (2) |
| M3 | end action reuses the caller-side verdict | RED (10) |
| M4 | the guard goes silent (no verdict lines) | RED (3) |
| M5 | op-cancel dropped, applySafeState kept | RED (9) |
| M6 | hard floor conditional on end action finishing | RED (3) |
| M7 | deadline-aware wait reports Satisfied | RED (8) |
| M8 | end action starts BEFORE cancel-all | RED (1 — the stalled-claim throw) |
| M9 | run() never disarms the guard | RED (4) |
| M10 | composite predicate re-ordered pred-first | **GREEN first pass — A HOLE** → closed → RED (1) |
| M11 | mid-flight destructor cancel removed | RED (5) |
| M12 | claim stops storing the claimant | RED (11) |
| M13 | cancel() tightened to forbid the pacer position | RED (1; the process aborts — see note) |
| M14 | wait unwind guards neutered | RED (11) |

**The hole, in full, because it is the most valuable row:** M10 swapped
`expiredNow() || pred()` to `pred() || expiredNow()`. My original "predicate is not called
after expiry" test only covered a wait *started* after expiry — where `waitFor`'s pre-check
returns before the composite ever runs, masking the order entirely. The real difference is a
wait **in flight** crossing the deadline: pred-first calls the scoring predicate one more
time at a post-deadline clock — and the documented idiom puts `op.tick()` (a 9 V command)
inside that predicate, so the mutation buys one commanded device event past the buzzer.
Closed with a test that records the clock at every predicate call and asserts none ever saw
`t >= deadline`; it fails ALONE under the mutation (re-run observed RED, restored, suite
green). Final tally: **14/14 RED, 0 GREEN, 0 BUILD-FAIL, 0 SKIPPED.**

**A mechanism note on M13** for whoever reads the log: the tightened precondition throws from
`cancel()` inside `pace()`; the unwind then reaches `WaitUnwindGuard`, whose own `cancel()`
throws AGAIN during unwind → `std::terminate` (the runner counts the abort as RED — the
mutation is caught either way, but the failure mode is a crash, not a clean assertion; that
is itself evidence the precondition set is load-bearing).

---

## 11. Verification (run, with output as observed)

```text
cmake --build build/test -j$(nproc) && ./build/test/shulib_tests | tail -4
  [doctest] test cases:    1017 |    1017 passed | 0 failed | 3 skipped
  [doctest] assertions: 1522314 | 1522314 passed | 0 failed |
  [doctest] Status: SUCCESS!

GUARD1 PASS          (no pros/ include anywhere in include/shulib)
GUARD2 PASS          (no shulib/sim/ include outside sim/)
ARM GATE PASS        (124 headers, arm-none-eabi-g++ -std=gnu++20 -Werror …)

doc gates:  self-test OK · check-coverage PASS · check-fresh PASS
            check-examples PASS (386 quoted lines, all verbatim, 4 source files)
            check-removability PASS

docs/internal/verify/verify-f2.sh (final run)
  register: F1–F5 untouched; F12 added NOT-LOCKED; non-freeze stated
  scope: no season symbols; no coordinate literal in sequence/; zero-default instants
  F2 mutations: 14 RED, 0 GREEN, 0 BUILD-FAIL, 0 SKIPPED; restores byte-identical
```

Nothing committed; nothing pushed; `git status` carries the whole chunk.

---

## 12. T8 and the scope boundary — what was deliberately not built

- **No `Sequence`/`Parallel`/`Race` combinator types** — ruled explicitly (the roadmap's WS8
  block and build-order's F2 entry both carry the ruling where their old demands stood).
  The capabilities exist as idioms: eager chain, the waitUntil-predicate concurrency F1
  pinned, disjunction predicates. `Deadline` shipped as the guard. The rejected alternative
  — building four types silently against two public documents — was the brief's own named
  worst outcome.
- **No season content.** `buildStack` / `matchLoadCycle` / `endInMidfield` / `strategyMode`
  are F4's, in hardware-gated Phase F′, authored by students (G4). The roadmap no longer
  lists them inside the engine's block. "Possession-aware time budgeting" is retired as
  game-flavored (C6's legacy lesson).
- **No field coordinate, no park, no default lead time, no default match length** — enforced
  by `verify-f2.sh`'s scope gates (no `Length{`/`Pose2d{` literal may exist in `sequence/`),
  not just promised.

## 13. Named handoffs

- **→ F4 (the second consumer, and the freeze trigger for F12):** the guard's API is shaped
  by exactly one consumer (the DoD tests + examples). F4's student-authored routines will
  stress `run()`'s ergonomics, the two-instant schedule, and whether combinator types are
  actually missed. Expect amendments; nothing is frozen.
- **→ G2:** `TickPhase::User` STILL has no producer (T1 retired F1's "F2 sequencer loop"
  candidate); G2's marker path is the remaining named owner. D1's ruling that G2 does not
  consume the recipe chain is untouched.
- **→ R4:** the honesty boundary's biggest open item — real loop rate and PROS call latency —
  is what converts "the schedule fired" into "the margin was right". Until then every lead
  time is the caller's engineering margin, and guide ch. 14 says so.
- **→ whoever reopens F10:** the frozen-wait lateness is pinned by tests that go red if a
  legal cut appears — that red is an instruction to rewrite guide ch. 9, not to re-green.
