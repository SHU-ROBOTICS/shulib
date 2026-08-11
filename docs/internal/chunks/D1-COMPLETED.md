# Chunk D1 — COMPLETED (built 2026-08-11; verified same day; working tree pending review)

> **The Tier-2 recipe layer exists — and F6 is still NOT frozen.**
> `include/shulib/chassis/routine.hpp` is `Routine`: an EAGER fluent chain over the C4
> facade where a complete auton is ~10 readable lines, every step delegates to exactly one
> facade verb, and the chain's error policy is decided, documented, and mutation-pinned:
> first failure → **stop + safe (0 V + Brake) + skip the rest + report**. The recipe-vs-
> facade twin is **bit-identical, clean and hostile** — three layers (scheduler, facade,
> recipe) now provably run the same physics. D1's other product is **§2, the facade
> critique** — the D2 freeze review's input packet, written after building on the surface
> as its second independent consumer.

Suite: **686 cases / 916,026 assertions** green under strict `-Werror` (3 pre-existing
skips unchanged). Baseline entering D1: 667 / 915,605. Both CI guards pass; all **103** v2
headers ARM-cross-compile clean (102 + routine.hpp). 19 mutations run: **19 red**, 2
build-gate trips (both re-formed and re-run), 1 mutation classified structurally
equivalent and not counted. Two green holes were found **by pre-analysis** before the
campaign ran and closed with sole-detector tests — the campaign then confirmed both
(M6b, M14: each red by exactly the one new test).

---

## 1. What was built

**New header (1):**

- **`include/shulib/chassis/routine.hpp`** — `Routine` (the Tier-2 chain), `RoutineResult`
  (the whole-chain verdict: ok / steps / completed / skipped / stoppedAt / stoppedName /
  cause / exit), `RoutineStopCause` (append-only: None / MotionFailed / WaitTimedOut /
  ActionFailed). Twelve steps: `startAt` / `moveTo` / `driveTo` / `strafeTo` / `turnTo` /
  `face` / `followTrajectory` / `brake` / `hold` / `pause` / `waitFor` / `then`. Borrows
  the Chassis; owns nothing but counters; non-copyable/movable (two handles over one stop
  state would let a stopped chain's twin keep driving). No heap, no step storage — the
  eager design needs none (§3).

**Modified (6):** `test/guide_examples_test.cpp` (+3 `guide-09*` cases), `docs/guide/`
(new `09-the-recipe-api.md`; README table + maintenance notes; ch. 08 closing pointer;
ch. 14 fallen-limitation rewrite; ch. 15 +3 glossary terms), `docs/roadmap.md` (M7
checkbox `[~]` + the "you are here" D1 paragraph).

**New test file (1):** `test/chassis_recipe_test.cpp` — 16 cases + 12 static_asserts (§8).

**No facade changes. No motion-layer changes. Zero.** The brief's trap ("change the verbs
to return a chainable") was not taken; every facade awkwardness became a §2 finding
instead. No new HA register entries: the recipe layer's two constants are a numerical
degeneracy guard (bearing at zero distance) and a pure code-level backstop
(`kPauseBackstopSeconds` — §6 D10 explains why it is deliberately NOT an HA entry).

---

## 2. THE FACADE CRITIQUE — D2's input packet

> The chunk's real job: what it was like to build on `Chassis` as its second consumer.
> Each item: what was hit, the code that shows it, and a recommendation — **change at D2**
> or **accept, and why**. Ordered by how much D2 should care. Items 1–2 are the ones that
> get expensive after the freeze; the rest are accepts with reasons, which D2 should
> confirm deliberately rather than inherit.

### 2.1 Time is untyped across the whole candidate surface — CHANGE AT D2 (or never)

The misuse story stops at time. Lengths, speeds, angles are typed (`Length`, `Velocity`,
`Angle` — a bare double does not compile), but every duration on the surface is a raw
`double`: `MotionOptions.timeoutSeconds`, `hold(double seconds)`,
`waitUntil(pred, double timeoutSeconds)`. The recipe layer inherited the carve-out
(`pause(double seconds)` — kept consistent deliberately, §6 D5). The bug this permits is
exactly the class the units system was built to kill: `hold(500)` from someone thinking
in milliseconds compiles and actively holds pose for 500 s — in a 15 s auton window that
is a match-loss, and nothing flags it. `units::Time` exists, literals exist (`0.5_s`,
`500_ms`), and the C4 record's own §8 calls both speed budgets out as typed — time is the
one dimension left behind. **Recommendation: retype the three time parameters (and
`pause`) as `units::Time` at D2, before the freeze makes it a versioned migration.** Cost:
`{.timeoutSeconds = 5.0}` becomes `{.timeout = 5_s}` at every call site — trivial now
(all call sites are in-tree), a breaking migration forever after. If D2 declines, the
freeze docs must at least name the carve-out honestly.

### 2.2 There is no time-wait verb, and the workaround is Tier-3-shaped — DECIDE AT D2

Every real auton has a "wait for your alliance partner" beat. The facade cannot express
"wait N seconds" cleanly:

- `waitUntil([]{ return false; }, seconds)` — the naive spelling — **logs a Warn on every
  deliberate pause** (C2's timeout Warn) and returns TimedOut, making an intended beat
  look like a failure in the transcript and in code.
- `hold(seconds)` energizes the drive and needs a live estimate — wrong for "sit limp
  against the wall".
- The working spelling (what `Routine::pause` does) is a clock-deadline predicate:
  `waitUntil([&clock, deadline]{ return clock.now() >= deadline; }, seconds + slack)` —
  which needs the clock via **the Tier-3 seam** (`deps().ctx->clock()`) plus a backstop
  constant for the timeout that must never fire. It works, it is Warn-free, and it is
  pinned (M9/M10) — but a Tier-2 concept requiring Tier-3 plumbing and a sacrificial
  timeout is the surface telling us something.

**Recommendation: D2 should consider `wait(seconds)` (or `sleep`) as one additive facade
verb** — semantics "advance the world, command nothing, return after the duration". Cost:
one more frozen signature. If declined: bless the predicate idiom in F6's documentation so
every team doesn't rediscover the Warn problem, and accept that the recipe layer keeps
absorbing it. D1 leans add-it (it is cheap now and universally needed), but did not reshape.

### 2.3 The C2 §11 flag-6 deferral, answered: runtime `abortFaultMask` mutation — CLOSE IT

C4 deferred "runtime mask mutation — owner: D1 (if recipes need a mid-auton policy
switch)". Probed: **recipes do not need it.** A Tier-2 chain is a linear script; "ignore
ODO_STUCK during the wall-slam leg" is an expert manoeuvre, and the expert already has
construction-time config plus the Tier-3 seam. No D1 test wanted it; no recipe step
design ever reached for it. **Recommendation: D2 closes the deferral — mask is
construction-time only at F6.** C2's own Phase-R fault-statistics revisit stands untouched.

### 2.4 The C3 §11 flag-1 deferral, answered: branch-on-strafe-fallback — KEEP THE REJECTION

C3/C4 rejected a polling `strafeFallbackActive()` getter and deferred "branch on fallback
via waitUntil predicates" to D1. Probed, and worth stating precisely: **a waitUntil
predicate cannot see the fallback either** — there is deliberately no accessor, so the
record stream remains the only witness. That means branch-on-fallback is not merely
unergonomic; it is unreachable at every tier without consuming telemetry. D1's recipes
did not need it: a routine budgets lateral legs at authoring time using
`strafeAuthority()` (the passthrough C3 asked for, which was exactly enough).
**Recommendation: accept — keep the rejection at F6.** Reacting mid-leg to fallback is
re-planning, which is G2+ territory; a facade getter would still be the control-flow-
coupled-to-telemetry design C3 refused.

### 2.5 Tank ergonomics: the trig lands on every author — ACCEPT, WITH THE SUGAR WHERE IT IS

D12's honesty (tank verbs never plan turns) means every tank routine hand-computes
bearings: the guide's `tankGoTo` idiom is four lines of `atan2` per leg. Building recipes
made the cost visible — and also showed the right fix lives in the recipe layer, not the
facade: `face(x, y)` / `driveTo(x, y)` are **argument sugar** (bearing computed from the
live estimate when the step runs → one delegated `turnTo`/`moveTo`), so the author still
writes the turn, in field words, and the hand idiom stays bit-identical (pinned by the
sugar twin, M4/M5/M19). **Recommendation: accept — do NOT add `turnToFace`/`moveToPoint`
to F6.** The facade stays minimal at the freeze; the unfrozen recipe layer owns field-
vocabulary sugar and can iterate on it. (If D2 disagrees, the recipe implementations are
the reference semantics.)

### 2.6 `brake`/`hold` earned their place — ADOPT AT D2

C4 proposed them; D1 is the consumer test: **every complete recipe routine used both**
(a routine that cannot park is not complete), and without them parking would require the
Tier-3 seam — a Tier-2 cliff, which §17 forbids. The chain's own stop policy also leans
on `cancel()`'s defined safe state, reinforcing that "controlled stop" and "panic stop"
are both load-bearing vocabulary. **Recommendation: adopt both into F6.**

### 2.7 Two result vocabularies forced a third — ACCEPT FOR F6, FLAG FOR F2

The verbs return `ExitReason` except `followTrajectory` (`TrajectoryResult`) and
`waitUntil` (`WaitResult`). A chain over all of them had to normalize
(`RoutineStopCause` + the "Running = no motion verdict" convention borrowed from
`CompletedMotion`, plus a kept `lastTrajectory()` so leg accounting isn't flattened —
M13). Each individual choice upstream was right (D8's and C2's reasoning hold); the cost
only appears when one consumer spans all three, and it was absorbable in ~30 lines.
**Recommendation: accept at F6.** But F2's combinators will span the same three — whoever
builds F2 should read this section first and consider whether a shared step-outcome
vocabulary wants to exist BELOW the recipe layer.

### 2.8 The §8 tension list, answered item by item

- **"Does the recipe layer need facade-level async or result-chaining?" — NO.** The eager
  chain composes blocking verbs and loses nothing (§3 is the analysis). Async stays
  behind `scheduler()` until F2. This closes C4 §5 D2's open end for the freeze.
- **"Does `hold` want a disturbance-radius option?" — NOT YET.** No D1 routine wanted
  it; config tolerance sufficed. Additive later via `MotionOptions` if R-phase shows need.
- **"Is `TrajectoryResult` enough, or per-leg results?" — ENOUGH** for Tier 2
  (`completedLegs` is the strategy-relevant fact and the chain preserves it). Per-leg
  detail remains C5's record/result-line material.
- **"Per-verb settle-tolerance override in `MotionOptions`?" — NOT NEEDED at D1.** No
  recipe test reached for it. Freeze without; the options struct is additive post-freeze
  by design (that design choice — C4 §5 D10 — paid off here: D1 needed ZERO reshapes).

### 2.9 What was genuinely good to build on (also evidence, also for D2)

Honesty cuts both ways; these C4 shapes did exactly what their decision log claimed:
the whole-`MotionOptions` pass-through needed no reshape for any step; the Tier-3 seam
was precisely sufficient for the one thing recipes needed beneath the verbs (the clock);
blocking-verbs-returning-honest-verdicts made the twin proof and the error policy nearly
free; and the standalone construction path meant the startAt test could wire the stack by
hand in 20 lines. The critique above is real, but the surface held its second consumer's
weight with **zero changes required** — that is the headline D2 should also hear.

---

## 3. The design fork: EAGER chosen, DEFERRED rejected (the full analysis)

§17's `chassis.moveTo(p).then(intake.in)` cannot chain off `Chassis` directly (verbs
return `ExitReason`, and reshaping them was the named trap). So Tier 2 is a chain OBJECT,
and the fork was whether chaining executes now (eager) or accumulates for a terminal
`.run()` (deferred). **Eager won.** The deferred alternative, taken seriously:

**What deferred would buy:** dry-run/introspection (print the plan without driving);
a step list a sequencer could consume; error policy selectable at `.run()` time;
plausible symmetry with G2 (PathRunner "runs a plan").

**Why it lost, in order of weight:**

1. **It breaks the first-year reading test the moment tiers mix — and mixed tiers are a
   hard requirement.** `chain.moveTo(a); chassis.turnTo(b); chain.run();` *reads* a-then-b
   and *runs* b-then-a. Program order ≠ field order is precisely the confusion class
   Tier 2 exists to kill (constraint 3), and "no cliff" (constraint 1) makes mixing
   normal, not exotic. Eager keeps one invariant a beginner can hold: **the routine runs
   exactly as it reads.** (Pinned: the interop case interleaves chain steps, a direct
   verb, and a pose branch, and asserts the ordering's physical consequence.)
2. **Errors must not vanish (constraint 4) favours acting at the failure instant.** An
   eager chain stops, safes the drive, and skips — *at the moment of failure*, whether or
   not anyone reads the result. A deferred chain can implement the same policy inside
   `.run()`, but adds its own new error-vanishing mode that eager structurally cannot
   have: **a chain built and never run** — compiles, "runs", robot does nothing, no error
   anywhere. Guarding that needs `[[nodiscard]]`-plus-runtime-checks or execute-in-
   destructor (throwing/blocking in a destructor — worse). Eager has no such state.
3. **Storage.** Deferred needs type-erased steps: heap (`std::function` — first heap
   dependency in the auton path) or a fixed-capacity buffer (an invented constant, plus a
   new misuse cliff at step N+1). Eager `Routine` is a pointer and six ints.
4. **The claimed F2/G2 seam is not real.** G2's PathRunner consumes *data* (waypoints +
   command-id markers from `.vexbot`), not a C++ step list — it composes the scheduler
   directly. F2's combinators are async and live at the scheduler. Making the recipe
   chain their engine would move sequencing/ownership logic INTO the layer whose contract
   is "delegates only, adds no motion logic" (constraint 2). The mechanism seam F1/F3
   actually need is just "an action slots between motions" — `then()` provides it in
   eager form, and a mechanism action is a callable either way.
5. **Introspection/dry-run, the one genuine deferred advantage, has a better home.** A
   dry run that doesn't drive is a *simulation* concern — this project's entire test
   methodology (the A2 plant) IS the dry run, with physics. If plan-printing is ever
   wanted, it belongs in G-phase tooling over `.vexbot` data, where the plan is data
   already.

Consequence accepted with eyes open: an eager chain cannot be inspected before it runs
and cannot re-run itself (one `Routine` = one execution; construct another to retry).
Both are the right trade for the audience: §17's Tier-2 user runs their routine to see
it, exactly as they run every test in the guide.

---

## 4. The error policy (constraint 4), decided and pinned

**Policy: first failed step → (1) stop the chain, (2) safe the drive via `cancel()`
(0 V + Brake, HA-53, idempotent), (3) skip every later step — counted, each skip logged
at Info, (4) report — one Warn naming routine/step/cause, `ok()` false, `RoutineResult`
carrying index/name/cause/exit.** A step fails when: a motion exits non-Settled; a
trajectory doesn't complete all legs; `waitFor`'s deadline passes; a `then` action
returns false / non-Settled. Preconditions are NOT policy: nonsense throws through
untouched, counters unmoved (a programming error must not become a polite strategy
outcome).

Alternatives rejected:

- **Continue-and-report-at-the-end** (railway with accumulation): the robot keeps
  executing a script from a position it is not at — the brief's own words, "worse than
  one that stops". Rejected outright for the default; a team wanting continue-past-
  failure branches on direct verbs for those legs (drop-a-tier, documented and tested).
- **Stop without safing**: leaves the drive in whatever the failed motion left (a
  TimedOut motion is 0 V but NOT braked — C4 §4.5's clarification, which this policy
  turns into user-visible behaviour). Mutation M2 shows four cases watching the safe.
- **Throw on failure**: converts every strategy outcome into control flow that skips the
  rest of `autonomous()` entirely — hostile to "log and recover, never crash" (C2), and
  a timeout is not exceptional in competition.
- **`waitFor` timeout as success or as a non-event**: in a linear recipe the later steps
  ASSUME the condition; continuing would act on a field state that never arrived. One
  layer down, `WaitResult::TimedOut` remains a faultless strategy branch — the two layers
  deliberately disagree because their contracts differ, and both spellings are available.

The policy's own failure modes were mutation-hunted: skip defeated (M1), safe dropped
(M2), wrong step index (M3), verdict ignored (M7/M8), exit unrecorded (M16), transcript
silenced (M17/M18), skips miscounted (M12), `ok()` lying (M11) — every one red.

---

## 5. The numbers through the recipe layer

### 5.1 The recipe twin — three layers, one physics

C1's generator (seed 77), C2's cadence, C4's twin protocol — now with a THIRD arm:
`Routine` steps vs direct facade verbs, identical rigs. **Every waypoint time, position
error, and heading error equal to the bit, clean AND hostile**; totals identical:

| arm | clean finalErr / time | hostile finalErr / time |
|---|---|---|
| recipe (D1) | 0.228175 in / 9.56 s | 1.54884 in / 13.43 s |
| facade (C4 §3.1, unchanged) | 0.228175 in / 9.56 s | 1.54884 in / 13.43 s |

Same digits C4's twin printed against the hand-built scheduler. Consequence: **scheduler
≡ facade ≡ recipe** on the measured auton — the recipe layer added API, not physics, and
C1–C4's baselines carry over verbatim. The hostile arm also re-asserts C2's 5.0 in bound
through the chain (measured 1.55 in).

### 5.2 Clean sweep through recipe steps — flat in move count

| n | motions | finalErr | time |
|---|---|---|---|
| 5 | 6 | 0.228 in | 9.56 s |
| 10 | 13 | 0.000 in | 18.38 s |
| 20 | 26 | 0.004 in | 35.02 s |
| 40 | 53 | 0.236 in | 74.80 s |

Identical to C4 §3.2's X column to the printed digit (the twin predicted exactly this);
H spot-checked at n=10 (0.8 in bound, well inside). No growth with routine length —
nothing in the chain accumulates.

### 5.3 The complete recipe auton, all three drivetrains

8 steps, 6 motions, ~10 readable lines (`guide-09a` is the canonical listing):
**X and H**: startAt → moveTo → then(action) → strafeTo → face → driveTo → hold → brake,
final truth error < 1.0 in, zero faults, counters exact. **Tank**: startAt → face →
driveTo → then → face → driveTo → hold → brake — the author-plans-the-turn idiom in
field words, same bounds. The sugar twin additionally pins face/driveTo ≡ the hand
`atan2` idiom bit-for-bit, with a ground-truth anchor so a sign error shared by both
arms would still fail the field direction check.

### 5.4 Guarantees re-verified THROUGH the chain

- **ODO_STUCK** (hostile encoder freeze): step exits Cancelled, chain stops at step 1,
  `lastCompleted().abortFault == OdoStuck` still readable at the facade, abort prompt
  (< 2 s, not the watchdog), drive safed, truth-vs-estimate damage < 8 in, later steps
  (including a then-action) provably never ran.
- **Watchdog + boot**: never-live IMU → first step TimedOut at its budget (1.5–2.0 s
  window), robot never moved (< 1e-6 in), chain stopped, remainder skipped.
- **Hostile survival**: §5.1's hostile twin arm, bounded and bit-identical.
- **Cancel-to-safe-state**: exercised on every chain stop (M2's four watchers).

---

## 6. Decision log (every choice with a viable alternative)

### D1 — EAGER chain, not deferred
§3 carries the full analysis. The most consequential D1 decision.

### D2 — Stop/safe/skip/report as THE error policy, with drop-a-tier as the escape
§4 carries alternatives. The safe-on-stop half generalizes C4 D8's stop-at-first-failed-
leg from one verb to the whole routine, and closes the TimedOut-leaves-coast gap at the
recipe level without touching the facade's two-contract distinction.

### D3 — Chain-level outcome vocabulary: `RoutineStopCause` + kept `ExitReason`
Alternatives: reuse `ExitReason` alone (cannot say WaitTimedOut/ActionFailed without
lying); strings (unbranchable). `exit` stays `Running` for non-motion stops — the
existing "none yet" convention (`CompletedMotion`), not new vocabulary. Append-only for
F1/F3 mechanism causes.

### D4 — face()/driveTo() are recipe-layer ARGUMENT SUGAR, not facade verbs
The only arithmetic in the layer: bearing = `atan2` of the estimate-to-point delta,
computed when the step RUNS (eager makes "after the previous steps" well-defined) —
then ONE delegated verb. Honors D12 (the author writes the turn); bit-identical to the
hand idiom (pinned). Degeneracy precondition at 1e-9 in — deliberately a numerical
guard, not an intent check (a 0.1 in nudge is legitimate). Alternative — facade
`turnToFace`/`moveToPoint` — rejected at D1 and recommended against for D2 (§2.5).

### D5 — pause() rides the waitUntil predicate seam with a clock deadline
Alternatives: naive false-pred wait (Warn pollution + failure-shaped result — M9 shows
the difference observably); `hold(seconds)` (energizes the drive; needs a live
estimate); adding a facade verb (report-don't-reshape — §2.2 is the report). The
predicate is time-monotone so the timeout backstop (`seconds + 1.0`) is unreachable
slack; a stalled pacer trips the scheduler's own precondition first.

### D6 — waitFor() timeout stops the chain even though WaitResult calls it a branch
The two layers disagree on purpose: `WaitResult::TimedOut` is a faultless branch when
the CALLER holds the branch; in a linear recipe the "caller" is the next step, which
assumes the condition. Both spellings remain available (drop a tier for branchy waits),
and the recipe header says which to use when.

### D7 — then() accepts void / bool / ExitReason — the placeholder mechanism seam
Alternatives: void-only (mechanism actions couldn't fail — F1/F3 would reshape the
seam); inventing a mechanism interface now (the brief forbids it, correctly). The
ExitReason arm exists so mixed-tier glue (an action wrapping a facade verb) keeps its
verdict — pinned by the act-ok/act-bad pair.

### D8 — Preconditions pass through; the chain's counters move only on a returned step
Counting after the delegated call returns means a throw leaves the chain exactly as
before the bad call — no half-recorded step, chain still usable (pinned by the misuse
case). Converting throws to chain-stops was rejected: it would let a NaN in a pose
masquerade as a field event.

### D9 — Routine borrows the Chassis; non-copyable/non-movable; one chain = one run
Copying would fork the stop-state (a stopped chain's twin keeps driving). No `reset()`:
a match auton runs once; tests construct fresh; a smaller surface into the D-phase
freeze discussions.

### D10 — `kPauseBackstopSeconds` is NOT an HA register entry
The rule is "any invented constant gets an HA-nn entry" — for constants standing in for
unmeasured HARDWARE truth. This one is pure code-level belt-and-suspenders: no plant,
gain, or field property depends on it, and any value clearing one tick behaves
identically (M10 pins the deadline, which is the load-bearing number). Documented
in-header instead. If the reviewer disagrees, registering it is a one-line addition.

### D11 — No `.finally()` / recovery step at D1
Safe-on-stop already parks the robot, which is what a skipped `brake()` mattered for;
richer recovery is strategy = drop-a-tier. If demand appears, `.finally()` is additive.
Deliberately left OUT of the surface until someone real asks.

### D12 — startAt() is a step, not a constructor argument
A step reads in routine order ("start here, then…"), skips consistently if the chain is
already stopped, and keeps the constructor trivial. Also the honest place for the
relocalization idiom later (re-seed mid-routine against a wall).

---

## 7. Findings (each handled where it lives)

### 7.1 FOUND by pre-analysis (green hole, closed): a no-op startAt was invisible — M14

Every shared rig auto-seeds the estimate to the plant's pose (`MotionRig`'s constructor),
so `startAt` was load-bearing in NO existing test: mutation M14 (setPose dropped) would
have been GREEN across all 685 other cases. Closed with the hand-wired standalone case —
plant off-origin, estimate deliberately unseeded — where dropping the seed produces a
~31 in systematic miss. Campaign confirmed: M14 red by exactly that one case. (The same
auto-seed convenience that makes every other test terse is what hid this; the case's
comment says so for the next reader.)

### 7.2 FOUND by pre-analysis (green hole, closed): speed budgets could vanish in delegation — M6b

The twin only varies timeouts; guide-09a's capped leg still settles (faster) without its
cap. So a chain forwarding `{.timeoutSeconds = options.timeoutSeconds}` — dropping both
speed budgets — was invisible to every test. Closed with the capped-vs-uncapped leg-TIME
pin (30 in at ≤15 in/s must take > 1.5× the 60 in/s default's time). Campaign confirmed:
M6b red by exactly that one case. Note the shape of the hole: it is C4's M21 lesson
(caps are invisible to convergence-graded tests) recurring one layer up, in delegation
rather than in the clamp.

### 7.3 Build-gate trips ×2 — the C4 §4.6 failure mode, live again

M5 and M6a's first forms deleted the USE of a variable/parameter and broke `-Werror`
(unused-variable/unused-parameter). Without the build gate both would have "run" against
the stale binary and read as green. The gate tripped, both were re-formed
(use-preserving), re-run, red. The campaign runner hard-gates on build success; keep
doing that.

### 7.4 Observation: final-pose grading forgives mid-routine defects on holonomic drives

M15 (strafeTo x/y swap) left the complete-auton case GREEN on X and H: after a wrong
strafe, the later `driveTo` still reaches the final target — holonomic authority erases
the evidence. The detectors were the twin (bit-compare) and guide-09a (asserts the pose
the strafe leg itself was responsible for). The general lesson, third occurrence in this
project's records (C3 kinematics, C4 M21, now D1 M15): **end-state grading proves
convergence, not path correctness — pin the intermediate claim or compare against a twin.**

### 7.5 The pause/Warn discovery — reported, not implemented around (§2.2)

Found while designing `pause()`: the naive spelling Warn-spams deliberate beats. The
recipe layer uses the predicate idiom (legitimate, seam-intended); the finding about the
facade's missing time-wait verb went to §2.2 instead of into a facade patch. Constraint
"report, don't reshape" followed to the letter.

---

## 8. Test inventory (16 + 3 cases, 12 static_asserts — every case names its bug in-file)

**chassis_recipe_test.cpp (16):** compile-time misuse pins (bare doubles into
moveTo/strafeTo/turnTo/face/driveTo render false via concepts; Routine
non-copyable/movable); the recipe twin clean+hostile (bit-compare + hostile bound +
chain-vs-scheduler ledger agreement); clean sweep 5/10/20/40 X + H spot (flatness);
the complete auton ×3 drivetrains (8 steps / 6 motions / truth-graded / then-action
sequencing flag); the sugar twin (face+driveTo ≡ hand atan2 idiom, bit + field-direction
anchor); the policy case (stop/safe/skip/report, transcript Warn+Info counts, latch
untouched below); then-action failure + strict ordering (action sees the ARRIVED pose);
ExitReason-returning actions honored both ways; waitFor entry-true free / timeout stops
faultlessly; pause (duration window, stationary, Warn-free transcript, chain continues);
ODO_STUCK through the chain (named cause, prompt, damage-bounded, skips); never-live
boot (bounded, motionless, stops); interop (chain + direct verb + pose branch, ledger
split exact); trajectory result preserved (success and failure); startAt standalone
(§7.1); speed-budget forwarding (§7.2); misuse passthrough (throws leave the chain
untouched and usable).

**guide_examples_test.cpp (+3):** guide-09a (the canonical ~10-line recipe = chapter 8's
routine one tier up, truth-graded); guide-09b (the error policy as the chapter teaches
it, transcript line asserted); guide-09c (tank face/driveTo + the no-cliff drop-down).
Chapter 09 quotes all three verbatim; the two transcript lines in the chapter were
CAPTURED from a real run of the 09b scenario, per guide-maintenance rule 1's spirit and
rule "re-capture, don't hand-edit".

Vacuity discipline: the capped-leg pin carries its uncapped twin; the sugar twin carries
a truth anchor against shared-sign-error cancellation; the policy case requires the
robot provably NOT at the skipped target; startAt's case is the one place the seed is
load-bearing by construction.

---

## 9. Mutation campaign (19 executed: break → build-gate → run → OBSERVE → restore)

| # | Mutation | Observed |
|---|---|---|
| M1 | skipIfStopped defeated (steps run after the stop) | **RED** 8 cases / 18 asserts |
| M2 | recordStop drops the safe-state cancel | **RED** 4 / 16 |
| M3 | stoppedAt off-by-one | **RED** 8 / 17 |
| M4 | bearing atan2 args swapped | **RED** 3 / 6 (sugar twin + tank auton + guide-09c) |
| M5 | driveTo zeroes the bearing (1st form: build-gate trip, re-formed) | **RED** 3 / 6 |
| M6a | moveTo drops the whole options struct (1st form: build-gate trip, re-formed) | **RED** 5 / 27 |
| **M6b** | **speed budgets dropped, timeout kept** | **RED 1 / 1 — ONLY §7.2's new pin; pre-analysis hole confirmed** |
| M7 | then() ignores the action's verdict | **RED** 2 / 13 |
| M8 | waitFor treats TimedOut as Satisfied | **RED** 1 / 9 |
| M9 | pause() as the naive false-pred wait | **RED** 1 / 6 (Warn appears + chain stops + duration) |
| M10 | pause() waits half the requested time | **RED** 1 / 1 |
| M11 | ok() always true | **RED** 8 / 8 |
| M12 | skipped steps counted as completed | **RED** 7 / 9 |
| M13 | lastTrajectory() not stored | **RED** 1 / 4 |
| **M14** | **startAt() does not seed the estimate** | **RED 1 / 1 — ONLY §7.1's new case; pre-analysis hole confirmed** |
| M15 | strafeTo delegation swaps x/y | **RED** 2 / 13 — twin + guide-09a; the complete-auton case CANNOT see it (§7.4) |
| M16 | RoutineResult.exit stays Running on a motion failure | **RED** 6 / 6 |
| M17 | the stop Warn silenced | **RED** 2 / 3 |
| M18 | the per-skip Info silenced | **RED** 1 / 1 |
| M19 | face() computes the bearing but never turns | **RED** 3 / 10 |

Not counted: the trajectory-step mutation "branch on `exit == Settled` instead of
`succeeded()`" is **structurally equivalent** — `followTrajectory` stops at the first
non-Settled leg, so `exit == Settled ⟺ completedLegs == totalLegs`; no test can separate
them (C3's structurally-uncatchable category; recorded so nobody re-hunts it).

Post-campaign: `routine.hpp` cmp-identical to the pristine snapshot; no mutation markers
anywhere in `include/` or `test/`; final re-green **686 / 916,026**.

---

## 10. What we now know for certain, and what we do not

### Known, with evidence

1. **The recipe layer adds API, not physics** — the three-layer twin (§5.1), digits
   identical to C4's record.
2. **The error policy is real behaviour, not documentation** — 10 of 19 mutations attack
   it from different angles; all red; the robot parks braked on every stop path.
3. **The facade held a second consumer with zero changes** — and the critique (§2) is
   therefore about *fit and finish*, not load-bearing defects. Items 2.1–2.2 are the two
   with a freeze deadline.
4. **Both C4 deferrals with D1's name on them are answered** (§2.3, §2.4) — both close as
   "not needed"; D2 can close them with a citation instead of a debate.
5. **Tank recipes work without new facade surface** — face/driveTo sugar ≡ the hand
   idiom, bit-proven.
6. **Two green holes existed and are closed as sole-detector tests** (§7.1, §7.2) — the
   project's streak of "every campaign finds something" extends to D1, though this time
   both were found by pre-analysis and confirmed by execution rather than discovered red.

### NOT known, stated plainly

1. **Whether a real beginner reads a recipe correctly** — the chapter is written for one;
   none has read it (C8's honest gap, unchanged, owner M7's DoD).
2. **Anything about real hardware** — every number is the A2 plant + A3 hostility; the
   recipe layer inherits every provisional HA constant below it (HA-50…55 et al.).
3. **Whether `then()`'s placeholder shape survives real mechanisms** — F1/F3 will hold
   it to the "slots in without reshaping the chain" promise; the seam is designed for
   it, not proven by it.
4. **Whether any team wants continue-past-failure often enough to earn `.finally()` or a
   policy knob** (§6 D11, C4 §9's stop-on-failure question) — no evidence either way yet;
   the drop-a-tier idiom is the documented answer until there is.
5. **The D3 cookbook's shape** — chapter 09 is one chapter, not a cookbook; D3 owns it.

---

## 11. Deliberately left for later chunks (named handoffs)

- **→ D2 (the freeze)**: §2 is the review packet. Specifically: retype time (§2.1,
  now-or-never); decide `wait(seconds)` (§2.2); close both deferrals (§2.3, §2.4);
  adopt `brake`/`hold` (§2.6); confirm the accepts (§2.5, §2.7, §2.8). F6 remains
  UNFROZEN today — §12.
- **→ D3 (cookbook + generated docs)**: chapter 09 + the guide-09 cases are the seed
  corpus; the recipe layer's surface is stable enough to document at length AFTER D2
  settles §2.1's spelling.
- **→ F1/F3 (mechanisms)**: `then()` is the seam; its contract (callable → void / bool /
  ExitReason; failure = ActionFailed stop) is the shape mechanism actions must satisfy —
  and `RoutineStopCause` grows append-only for mechanism-specific causes.
- **→ F2 (combinators)**: read §2.7 before designing; the recipe layer's normalization
  is a symptom F2 will feel harder.
- **→ G2 (PathRunner)**: no dependency on the recipe chain (deliberately — §3 point 4);
  `followTrajectory`'s span form + scheduler waitUntil remain its floor.
- **→ M7's DoD**: a genuinely-new reader through chapter 09 (and the 10-minute flow, G4).

---

## 12. Freeze Register note (documentation contract #6)

**No freeze at D1 — explicitly.** The register's F6 row is UNCHANGED and still reads
🎯 pending **D2** ("candidate BUILT at C4 … freezes at D2 after D1's second consumer").
The second-consumer exercise the row anticipates is now done and documented (§2), which
is D2's *input*, not a freeze. The "not frozen" notices in guide chapters 10 and 14
REMAIN (ch. 14's wording updated only to say the second consumer now exists — the
warning itself stands, plus chapter 09 carries its own matching notice; the guide
README's maintenance list now names all three chapters for D2's soften sweep).

Freeze-adjacent acts, recorded: **none touched a frozen contract.** The recipe layer is
new surface in `chassis/` (F6's namespace but not F6's candidate signature set — D2
should say explicitly whether `Routine` joins the F6 freeze or stays deliberately
unfrozen through D3; D1 recommends the latter, so the cookbook can still shape it).
No F1/F3/F5 file was modified; the bearing trig lives in the recipe header.

---

## 13. Verification (actually run, outputs as observed)

```text
$ cmake --build build/test -j"$(nproc)" && ./build/test/shulib_tests | tail -4
[doctest] test cases:    686 |    686 passed | 0 failed | 3 skipped
[doctest] assertions: 916026 | 916026 passed | 0 failed |
[doctest] Status: SUCCESS!
```
(3 skipped = the two M3 acceptance stubs + the R3 GPS field-cal oracle, unchanged.)

```text
$ <guard 1: grep -rnE '#include [<"]pros/' include/shulib>
GUARD1 PASS (PROS-free)
$ <guard 2: grep -rnE --exclude-dir=sim '#include [<"]shulib/sim/' include/shulib>
GUARD2 PASS (core is sim-free)
```

```text
$ arm-none-eabi-g++ -std=gnu++20 -Wall -Wextra -Wconversion -Wsign-conversion -Wshadow \
    -Werror -Os -mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=softfp \
    -c all_headers.cpp -o /dev/null -Iinclude        # generated list, ALL v2 headers
TU includes 103 headers
ARM GATE CLEAN
```

Removability (C7/C8 property): the RESUMING four-term check
(`internal/|chunks/|RESUMING|build-order` over README, test/README, docs/*.md,
docs/guide/*.md) returns **EMPTY**. The broader C8-style grep additionally matches 5
pre-existing prose mentions of completion-record section names in non-guide public docs
(e.g. "A3-COMPLETED §3.7" in hardware-assumptions.md) — all 5 present at HEAD before D1,
none added by D1, none are links, and `docs/guide/` is clean under both greps. Flagged
here so the release-time squash review sees it once instead of rediscovering it.

Working tree left uncommitted for review, per the brief. Nothing committed, nothing
pushed. Post-mutation integrity: routine.hpp cmp-identical to its pristine snapshot.
Guide link check: every relative link target in chapter 09 resolves; chapter 09's
transcript lines captured from a real run, not hand-written.

---

## 14. DoD checklist (brief §Definition of Done)

- [x] **Recipe API implemented; a real routine reads in ~10 lines and works on all three
  drivetrains** — routine.hpp; the complete-auton case ×3 (§5.3) and guide-09a (the
  canonical listing, 7 chained steps).
- [x] **Delegates only — no motion logic duplicated** — every step is one facade call;
  the only arithmetic is bearing argument-computation (§6 D4), pinned ≡ the hand idiom;
  the three-layer twin is the proof delegation adds nothing (§5.1).
- [x] **Error policy decided, documented, tested** — §4; header docs; 10 mutations
  attack it, all red.
- [x] **Every lower-layer guarantee verified THROUGH the recipe layer** — §5.4:
  ODO_STUCK, watchdog + never-live boot, hostile bounds, cancel-safe-state; accuracy
  flat in move count (§5.2).
- [x] **No capability lost; mixed-tier usage works** — every facade verb reachable as a
  step except `drive()`/`cancel()`/pose-branching, which are DOCUMENTED deliberate gaps
  with the mixed-tier idiom as the answer (routine.hpp header; ch. 09); interop case
  pins the mixing.
- [x] **A written critique of the facade** — §2, eight findings + the tension list +
  what held up, each with a D2 recommendation.
- [x] **Guide chapter 09 written, examples compiled and quoted verbatim** —
  `docs/guide/09-the-recipe-api.md`; guide-09a/b/c; README table, ch. 08 pointer,
  ch. 14 update, glossary +3; captured transcript lines.
- [x] **F6 still NOT frozen; the register still shows it pending D2** — §12; the row is
  byte-identical to its pre-D1 state.
- [x] **Suite green; both guards pass; ARM gate passes** — §13, actual outputs;
  686 / 916,026; 103 headers.
