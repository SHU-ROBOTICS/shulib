# Chunk C5 — COMPLETED (built 2026-08-10; verified same day; working tree pending review)

> **The run is legible in real time on the terminal — M2's headline clause closes with this
> chunk.** A host-sim auton now produces the §18.3 output end to end: the §18.5 session header
> (which binary, which routine, which battery), the stamped per-tick stream, a per-motion result
> line at every boundary (structural — a routine cannot forget one), and the one-screen run
> summary. Every §18.3 surface is byte-goldened; every reported number is cross-checked against
> plant ground truth on all three drivetrains. Diagnostics-plan **D-1…D-5 all landed**, and the
> one genuinely time-sensitive act in the whole plan — **reserving the D-2/D-3 schema fields in
> `DebugRecord` before the H1/F9 freeze** — is discharged and wire-pinned.

Suite: **659 cases / 915,570 assertions** green under strict `-Werror` (3 pre-existing skips
unchanged). Baseline entering C5: 592 / 915,157. Both CI guards pass; all **102** v2 headers
ARM-cross-compile clean (89 at C4 + 13 new). 35 mutations run: 33 red, **2 green holes found
and closed**, 1 build-gate trip.

---

## 1. What was built

**New headers (13):**

| Piece | File | Role |
|---|---|---|
| `lineformat::Line` + numeric primitives | `diag/line_format.hpp` | The §18.3 formatting core, EXTRACTED VERBATIM from TermSink (A1 goldens = bit-identity proof) so all three renderers share one NaN/±Inf/compaction definition |
| `SessionInfo` + `emitSessionHeader` | `diag/session_info.hpp` | §18.5 provenance as the first lines of every run; MISSING hash is LOUD |
| `compiledBuildHash()` | `diag/build_info.hpp` | The build-system → macro → header plumbing; empty = missing, no placeholder exists |
| `MotionOutcome` + `MotionResult` + `emitResultLine` | `diag/motion_result.hpp` | §18.4's boundary vocabulary (SETTLED/TIMEOUT/CANCELLED/FAULT_ABORT/SUPERSEDED, wire-pinned) + the §18.3 result line |
| `RunSummary` | `diag/run_summary.hpp` | The summary as DATA on the sink seam (value type, bounded strings — retainable by sinks, no dangling views) |
| `LevelFilterSink` | `diag/level_filter_sink.hpp` | **D-1**: per-subsystem log levels (global + 16 overrides); records/summaries pass untouched |
| `RateLimitedSink` | `diag/rate_limit_sink.hpp` | **D-2**: per-tag token buckets; drops COUNTED + STAMPED on-wire + ANNOUNCED + SUMMARIZED; Error/Warn and summarize() exempt |
| `TickAttribution` + `tickPhaseName` | `diag/tick_attribution.hpp` | **D-3**: RAII phase timing on a separately-injected clock; last-completed-tick story; worst-phase naming |
| `PoseDeltaGuard` + command/volt audits | `diag/plausibility_guard.hpp` | **D-5**: log-and-recover beyond finiteness; `FaultCode::Implausible`; HA-56 envelope |
| `ControllerFaultDisplay` | `diag/controller_display.hpp` | **D-4** content: OK/FAULT + first fault by name + battery; rewrites only changed rows |
| `ILineDisplay` | `hal/line_display.hpp` | **D-4** seam: the V5 controller row device (3×19, HA-57); PROS glue → R1, exactly the ICharSink pattern |
| `FakeLineDisplay` | `hal/fake/fake_line_display.hpp` | Recording row device: content AND write-count assertions |
| `RunReporter` | `motion/run_reporter.hpp` | The glue: session header → result lines (as the scheduler's boundary observer) → summary; self-attaches at construction |

**Modified (9):**

- `diag/debug_record.hpp` — **THE TIME-SENSITIVE ACT**: `droppedRecords`/`droppedLines`
  (uint32, D-2) + `tickPhase[8]` (`units::Time`, D-3) + the `TickPhase` vocabulary (0–5
  defined, 6–7 spare) beside `GateReason`. Wire-pinned by test. Appended, never reshaped.
- `diag/fault.hpp` — `FaultCode::Implausible = 10` appended (the A3 MotorOverTemp recipe).
- `diag/term_sink.hpp` — rewired onto `line_format.hpp` (ADL keeps call sites verbatim);
  `summarize()` renders the §18.3 six-line block (unstamped — a run artifact, not a timed event).
- `hal/telemetry_sink.hpp` — additive `summarize(const RunSummary&)` (non-pure, no-op default,
  forward-declared — the A1 emit() recipe; decorator-forwarding rule documented loudly).
- `hal/fake/fake_telemetry_sink.hpp` — records the summary channel (bounds-checked history).
- `motion/motion_scheduler.hpp` — `CompletedMotion` +6 result fields (rule 7: added THERE, not
  shadowed); `MotionStatsSink` (record-stream aggregation) chained inside the stamp route;
  `CommandIdStampSink` also stamps tick phases; `IMotionObserver` boundary seam (+`inBoundary_`
  re-entrancy preconditions on all five verbs); optional `TickAttribution` behind
  `MotionSchedulerConfig.attributionClock` (null = off = zero cost); `PoseDeltaGuard` per tick
  (boot-window-gated); the overrun-attribution Warn line; run-level heading aggregates;
  pre-empt marked `preempted` (→ SUPERSEDED).
- `motion/command_pipeline.hpp` — the D-5 self-audit: `commandWithinCapability` post-clamp +
  `recoverWheelVoltage` per wheel volt (pass-through when healthy — bit-identity held).
- `test/CMakeLists.txt` — `SHULIB_BUILD_HASH` from `git describe --always --dirty` at configure
  (+ `CONFIGURE_DEPENDS` on `.git/HEAD`); absent git ⇒ macro undefined ⇒ the loud path.
- `.github/workflows/ci.yml` — unchanged (new headers fall inside both guards' existing scopes;
  the generated ARM list picked all 13 up automatically — the A4 no-rot property doing its job).

**New test files (9):** `session_header_test.cpp` (6), `motion_result_test.cpp` (6),
`run_summary_test.cpp` (8), `level_filter_test.cpp` (5), `rate_limit_test.cpp` (7),
`tick_attribution_test.cpp` (7), `plausibility_test.cpp` (8), `controller_display_test.cpp` (6),
`run_report_e2e_test.cpp` (13). Plus additive cases/pins in `debug_record_test.cpp` and
`fault_test.cpp`. **67 new cases**; every one names the bug it would catch, in-file.

**Register: HA-56** (D-5 physical maxima — reasoned, settle R3/R5) and **HA-57** (controller
3×19 grid — reasoned, settle R1). Reconciliation grep clean in both directions.

---

## 2. Decision log (every choice with a viable alternative)

### D1 — Result/summary/session are DIAG-level data + formatters; the glue lives in motion/
A1 predicted C5's result code would land wholly in `diag/`. The dependency reality: the result
line's data source is `CompletedMotion` and the scheduler's counters — motion-layer types that
`diag/` (a dependency LEAF, per debug_record.hpp's own rule) must never name. So the split:
`diag/` owns the VOCABULARY and FORMATTERS (`MotionResult`, `RunSummary`, `SessionInfo`, all
golden-testable with hand data — which is exactly what made the byte-exact transcript test
possible without a full rig), and `motion/run_reporter.hpp` is thin GLUE.
**Rejected:** moving `CompletedMotion` into diag/ (reshapes C2's staged type and inverts the
motion→diag dependency for no gain); a parallel result struct built by callers (shadowing —
rule 7 violation); formatting inside the scheduler (couples the loop owner to presentation).

### D2 — Result lines and the header ride log(); the summary rides a NEW seam channel
The §18.3 sample shows result lines stamped `[t=…] [MOT] …` — that IS the log() rendering, and
the FaultLatch set the precedent (owner formats structured text; sink sanitizes/frames). The
summary block is UNSTAMPED in §18.3 (a run artifact), so it cannot ride log(); it became data
on the seam — `summarize(RunSummary)`, added by the exact additive recipe A1 used for emit()
(non-pure, default no-op, forward declaration): every pre-C5 sink compiles untouched (pinned by
the additivity case). **Rejected:** formatting the summary in the reporter and pushing text
through log() line-by-line (stamps it, and freezes the terminal rendering as the ONLY consumer
— E1's SdSink could never serialize the summary structurally); per-channel seam methods for
header/result too (three new virtuals where log() already carries the exact target shape).

### D3 — `line_format.hpp` extracted VERBATIM; ADL keeps TermSink's call sites unchanged
Three renderers (ticks, result line, summary) each hand-rolling NaN tokens and %.3g compaction
is how one drifts (a result line printing libc's `-nan(0x…)` while ticks print `NaN`). The
extraction is byte-verbatim; because `Line` lives in the new namespace, TermSink's unqualified
`appendNum(line, …)` calls resolve by ADL — zero call-site churn. Proof: every A1 golden green,
unchanged, immediately after the refactor. **Rejected:** duplicating the helpers per renderer
(the drift bug by design); making the new formatters TermSink members (result lines must exist
for message-only sinks too).

### D4 — Boundary results are STRUCTURAL: an observer seam on the scheduler
The A1 emitRecord lesson, one layer up: a per-verb "remember to call reporter.motionResult()"
is a call that gets skipped. `IMotionObserver` fires at EVERY boundary — settle, timeout, user
cancel, fault abort, pre-empt — synchronously in finalize(), after `CompletedMotion` is fully
recorded and the slot is cleared. Re-entrancy is precondition-blocked (`inBoundary_` on all
five verbs): a boundary is not a place to re-plan a routine from (the e2e case proves the loud
failure). RunReporter self-attaches at construction and detaches at destruction.
**Rejected:** polling `lastCompleted()` after each verb (forgettable, and misses pre-empt
boundaries entirely — they happen inside async()); the scheduler holding a RunReporter directly
(couples it to presentation and to the reporter's sink choice); multiple observers (no consumer
needs fan-out; a composite is trivial for whoever ever does).

### D5 — Result-line quantities derive from the RECORD STREAM (`MotionStatsSink`)
Overshoot is inherently a per-tick MAX no boundary snapshot can recover, and the stream is the
one place every motion type — including future Tier-3 ones — already reports target/measured/
error uniformly (rule 7: don't re-derive what's published). The stats sink chains after the id
stamp (discriminates on the stamped id), aggregates Running ticks + the exit record once
Running was seen, and deliberately IGNORES waiting-state records (their zeros are the record
"not inventing" — aggregating them would fabricate; mutation M27 red) and idle/plant records
(id 0 — the C2 §4.3 two-producers lesson). Honest consequence, stated in-header: with NullSink
no records flow, so `hasPathData=false` and the line renders `n/a` — you cannot have free
result numbers AND zero-cost ticks. `finalPose` is boundary-read from the Localizer, ALWAYS
real, independent of the stream. **Rejected:** `IMotion::targetPose()` additions (touches C1's
frozen-adjacent contract; HoldPose/DriveBrake have no meaningful field target); scheduler-side
per-tick tracking against a target it doesn't know; a per-motion history ring (C2 already
deliberated and deferred it — still no consumer needing more than the last boundary + stream).

### D6 — Overshoot/drift semantics: projection-past-target, with a stationary-target fallback
`over` = max projection of (measured − target) onto the start→target direction, floored at 0 —
the classic overshoot, computable streaming. Below 0.1 in of start→target separation (turns,
holds, brakes) the direction is undefined and it degrades to worst-wander-from-the-point.
`drift` = |final heading error| (the §18.3 sample's `drift 0.1°` against its `90.0°→90.1°`
poses). Both defined in motion_result.hpp where the consumer reads them. **Rejected:** overshoot
as max |error| growth after first minimum (not streamable without history); drift as heading
wander range (a turn's transit would dominate; the sample clearly reads final error).

### D7 — The summary's "scored/failed" line is the MOTION LEDGER, honestly
§18.3's sketch shows `scored 6 pin · 1 cup` — game-object scoring, which is strategy-layer
knowledge the library cannot have (G-phase command registry). The M2-honest equivalent is
`motions 7 · settled 6 · timeout 1 · cancelled 0 · aborted 0`. Documented in run_summary.hpp;
scoring lands additively when G2's registry exists. Similarly `heading max/final` is the
max/final of PER-MOTION BOUNDARY drifts, not mid-tick transients (a 90° turn passes through 90°
of "error" by design; reporting that would bury how headings LANDED), and `gating rejects`
counts GPS_GATE_REJECT raises (= HealthMonitor EPISODES today; honest label, E2 refines).

### D8 — The build hash comes from the BUILD SYSTEM; missing is LOUD; `--dirty` is mandatory
Code cannot know its own commit. `SHULIB_BUILD_HASH` is a macro the consuming build defines
(test CMake: `git describe --always --dirty` at configure + `CONFIGURE_DEPENDS` on `.git/HEAD`;
R1 adds the same line to the PROS Makefile). Empty/missing ⇒ `[ERROR][SES]` FIRST + the literal
token `MISSING` in header and summary — there is no "unknown"/zeros fallback anywhere, by
design and by anti-placeholder test pins (mutation M5 red). `--dirty` because a clean-looking
hash from a modified tree IS a wrong hash. Honest scope: configure-time evaluation can lag a
dirty-flag change until reconfigure (host-suite-acceptable, documented in the CMake comment;
the robot Makefile evaluates per build). **Rejected:** a generated header per build (build
machinery for a host-test nicety); code reading `.git/` at runtime (absurd on a V5); any
fallback value (the landmine).

### D9 — D-2's cost/visibility posture: wantsRecord() forwards even when the bucket is empty
A throttled tick still pays record population so the drop can be SEEN and counted at emit().
Counting inside wantsRecord() would let a query with no call-count contract mutate state;
returning false on an empty bucket would make drops invisible — the exact D-2 failure. With
NullSink inner, wantsRecord() is false and nothing is built or counted: competition stays free
(pinned). Error/Warn lines and summarize() are throttle-EXEMPT (a throttled fault line is a
lost root cause; mutation M13 red). Line-drop episodes announce ONCE, on resume, BEFORE the
resuming line. Tag table is bounded (16 + one shared overflow bucket — bounded memory beats
per-tag fairness for hypothetical tag #17, and the sharing is documented). Defaults
(50 rec/s, 20 lines/s/tag) are terminal-bandwidth logic constants, not register entries.

### D10 — D-3's clock is a SEPARATE optional injection; attribution stamps lag one tick
Phase timing needs a clock that advances DURING a tick; the sim clock advances only between
ticks (the pacer owns the world). So `MotionSchedulerConfig.attributionClock` — null = off =
zero clock calls (the A1 cost contract, structurally; R1 passes the real μs clock; tests pass
scripted ones). Records are emitted mid-tick, before the tick's total is knowable, so records
carry the most recently COMPLETED tick's breakdown — uniformly, documented on the schema field.
The lag is exactly right for the payoff: an overrun is DETECTED at tick N+1 (its dt covers
tick N), and the last completed breakdown at that moment IS tick N — so the scheduler's
`overrun attribution: loc …ms · mot …ms · other …ms (worst mot)` line names the tick that
actually overran. `other` = total − attributed, reported as its own quantity rather than
smeared into a named phase. Exception safety: a throw through the tick body ABANDONS the
half-measured tick (guard + `abandonTick()`) — discarded, not reported, instrument re-armed.
**Rejected:** `std::chrono` directly (untestable deterministically, bare-metal-questionable);
reusing the loop clock (all-zeros in sim, silently); always-on attribution (violates the cost
contract for a dev feature).

### D11 — D-5's three invariants live where their data lives; the pose fault is ADVISORY
Pose delta: the scheduler (sees the estimate + dt each tick), episode-gated like HealthMonitor,
dt-scaled, and NOT judged during the boot window (§4.1 — found immediately by the boot suites).
It never rewrites the pose: a diagnostic that mutates the data path is worse than the bug it
hunts (principle 4); watchdogs + the fault policy bound damage, and E-phase correctors may gate
on the fault. Command capability + wheel volts: inside the ONE command pipeline as a self-audit
— pass-through when healthy (bit-identity held: the entire pre-C5 corpus, hostile suites
included, ran unchanged), REAL recovery when not (NaN volt → 0 V, over-ceiling → clamped,
before any motor). The command audit is deliberately not episode-gated: a free-function
pipeline has no state home, and a persistent pipeline regression SHOULD be loud (the latch's
saturating tally bounds it). **Rejected:** invariants as a record-auditing sink decorator
(dev-build-only protection — but D-6's flight recorder triggers on faults IN COMPETITION);
new per-invariant fault codes (one code + structured detail; the enum stays lean).

### D12 — D-4 is a ROW device seam, not ICharSink reuse; content diffs before writing
The controller LCD is three fixed rows overwritten in place — "append bytes" is the wrong verb;
`setLine(row, text)` is the device's real contract, and it is what lets content code express
"rewrite only what changed" (V5 controller writes are slow and firmware-rate-limited; steady
state costs three strncmp and ZERO device writes — write-count-pinned, mutation M30 red). The
run clock quantizes to 0.1 s (the screen provably LIVE — a frozen screen and a crashed program
must not look alike) and battery to 0.1 V (truncated toward zero: no flicker at the rounding
boundary). Row 1 carries the FIRST fault — the root cause, not the cascade. Geometry 3×19 is
HA-57. **Rejected:** ICharSink + terminal escapes (not a terminal); pushing rendered rows
through the telemetry seam (a display is not a log); rumble/priority logic (R1+ can add; C5
ships the minimal high-value content D-4 named).

### D13 — Pre-emption is SUPERSEDED, distinct from CANCELLED, at the boundary
§18.4 lists SUPERSEDED as its own exit code; `control::ExitReason` deliberately doesn't carry
it (the motion can't know WHY it was cancelled — the boundary can). `CompletedMotion.preempted`
is set only on the async() pre-empt path; the reporter maps Cancelled+fault → FAULT_ABORT,
Cancelled+preempted → SUPERSEDED, bare Cancelled → CANCELLED. A result line that blamed the
author's cancel() for the scheduler's last-command-wins would mis-teach the vocabulary the
line exists to teach. **Rejected:** extending ExitReason (wire-stable, and the motion layer
genuinely doesn't know); inferring pre-empt from "a new motion started the same tick" (fragile
bookkeeping where one honest bool suffices).

### D14 — The reporter takes the UNTHROTTLED head; one reporter + one scheduler per run
Header/result/summary are a handful of run landmarks; the recommended wiring puts D-2's
limiter on the high-rate record path and the reporter directly on the formatter — a landmark
eaten by a bucket that per-tick chatter drained would be noise control defeating its purpose.
(Through a shared throttled head, dropped landmarks ARE still counted — nothing is ever
silent.) Scheduler counters are lifetime-cumulative and the latch clears only at explicit run
boundaries, so the summary's scope is one run per scheduler+reporter — the normal auton shape,
stated in-header. Battery start/end are READ (a typed 12.6 that was actually 11.9 is the
lying-number class this chunk bans).

---

## 3. The numbers are TRUE — the ground-truth cross-checks (constraint 3)

The C1 settled-vs-truth discipline, applied to the REPORT. Every value below is as observed.

### 3.1 Clean plant, X and H, 6-leg swept routine (seed 4242/991)
Per motion: |reported finalPose − truth| , |reported drift − true heading error| , |reported
overshoot − truth-trajectory overshoot| (truth recomputed from a per-pace truth trace with the
same projection formula):

| Drivetrain | worst pos divergence | worst heading divergence | worst overshoot divergence |
|---|---|---|---|
| X | **9.1e-13 in** | 0 rad | 0 in |
| H | **5.6e-13 in** | 0 rad | 0 in |
| tank (turn-then-drive idiom, 4 legs) | **1.1e-12 in** | 0 rad | — |

With perfect sensors the estimate coincides with truth, so the bounds are pinned
NEAR-MACHINE-TIGHT (1e-9): on a clean plant, the report must EQUAL the physics — any looseness
would be the reporting path itself. (These are divergences of the REPORT from truth; the
robot's error vs its TARGET is C1–C4's story, unchanged.)

### 3.2 The overshoot check made non-vacuous — an inertial plant that REALLY overshoots
Found during bring-up: the accuracy plant is memoryless (kA = 0, deliberately hand-derivable)
and **cannot physically overshoot** — the swept comparison was 0-vs-0. Closed two ways: the
stats formula is pinned on hand-fed record streams (0.5 in exact, hold-branch 0.3 in exact,
mutation M28 red), and an INERTIAL sub-case (plant kA = 0.05 ⇒ wheel time constant ~0.3 s,
underdamped approach, while the motion's FF knows only kS/kV) makes the robot genuinely sail
past its target: **reported 3.78696 in, truth-trajectory 3.78696 in** — a real overshoot,
reported true to six digits, with final-pose divergence still < 1e-9 in.

### 3.3 Hostile (A3 FullHostility, seed 11, 5 legs)
5/5 settled; **worst believed-vs-true divergence 1.76 in** — inside the C2/C3 hostile envelope
(≤ 5 in), asserted. The report is honest to within the estimator's documented hostile bounds:
the line reports what the motion believed, and the tests quantify exactly how far belief can
drift from truth under composed hostility. The transcript stays coherent (no libc NaN
spellings anywhere, every boundary reported, ledger == counters — §18.3 tokens only).

### 3.4 What "true" means with NullSink — the honest n/a path
No records ⇒ no derived numbers ⇒ the line renders `over   n/a  drift  n/a` — NEVER a
fabricated 0.00 (mutation M3 red) — while `finalPose` stays real (boundary-read; < 1 in of the
target in the cost test) and duration/outcome are boundary facts. The summary's heading fields
render `n/a` the same way (M8 red).

---

## 4. Findings (each handled where it lives)

### 4.1 FOUND + FIXED: the pose-delta guard must not judge the boot window
The first in-vivo run of `PoseDeltaGuard` turned three boot suites red (C2 boot, C2 never-live,
C4 facade boot): while `Uninitialized` the published estimate is definitionally not a physical
trajectory (boot garbage held out of the fold, heading follows a still-calibrating IMU), and
the boot→live transition is a legitimate jump. ROOT-CAUSE fix at the scheduler call site: the
guard resets through the boot window; the first live tick re-baselines. This is exactly the
false-positive class the guard had to be proven free of before shipping — and the boot gate is
itself now mutation-pinned (M21: gate removed → the boot suites go red again).

### 4.2 FOUND (green hole, closed): the D-5 pipeline WIRING was invisible to 915k assertions — M23
Severing BOTH audit calls in `applyCommandPipeline` left the whole suite green: the audit is a
pass-through on healthy input, and every D-5 test injected at the free functions. A pipeline
regression in the field would have had no working tripwire — the exact class of protection D-5
exists for, dark. Closed with the hostile-pipeline case: NaN speeds through
`applyCommandPipeline` itself (the public Tier-3 entry; std::clamp/hypot propagate the NaN
through the clamps) must raise IMPLAUSIBLE at the pipeline's own call sites AND deliver 0 V to
every motor.

### 4.3 FOUND (green hole, closed): one mutation hiding behind another — M23a
The first closure used a combined raise-count (≥ 2), and re-running the mutation HALVES showed
the command-audit half (M23a) still green: invariant 3's four volt raises satisfied the count
without invariant 2 existing. The test now pins each invariant's wiring DISTINCTLY (structured
detail match: "command outside capability" AND "wheel volt"; exact count 1 + wheelCount).
Post-closure: M23a-only red, M23b-only red (the case aborts — FakeMotor's own precondition
rejects the NaN volt that recovery no longer stops), both-severed red. Lesson recorded: after
closing a multi-site mutation, re-run its HALVES.

### 4.4 FOUND + FIXED (by the ARM gate): a `%u` that does not compile on the robot
`uint32_t` is `unsigned int` on the host but `unsigned long` on the ARM target; the throttle
notice's `%u` failed `-Werror=format` under the A4 cross-compile gate — a defect that would
otherwise have surfaced at R1, expensively. Fixed with `%lu` + explicit cast (commented). The
gate exists for exactly this; first real catch since it was automated.

### 4.5 Process: two e2e test bugs during bring-up, both documented lessons biting
(1) The attribution case asserted on the LAST record — which was an UNSTAMPED plant record
(C2 §4.3's two-producers note, biting a C5 author): fixed by emitting stamped records from the
test motion and selecting the last stamped record. (2) The forced overrun fired before the
post-reset baseline tick, so LoopMonitor saw nothing: fixed by firing the gap before tick 3.
Neither was a scheduler defect; both are recorded because the diagnosis re-derived real
contract subtleties (stamping scope; monitor reset semantics).

### 4.6 Observed: the entire pre-C5 corpus is a zero-false-positive certificate for D-5
The pipeline audit and (boot-gated) pose guard ran under every existing suite — clean sweeps,
hostile sweeps, stall/brownout/IMU-loss injections, 915k assertions — with zero Implausible
raises outside the deliberate injections. The guards are quiet on everything the project knows
how to throw at a robot, at the HA-56 defaults.

---

## 5. Test inventory (67 new cases — every one names its bug in-file)

**session_header_test (6):** §18.5 golden bytes; MISSING is loud (ERROR first + token, and the
anti-placeholder pins: no "unknown"/"0000000"/"deadbeef" anywhere); channel semantics on a
message-only sink; empty fields render "-"; hostile field text sanitized/bounded (framing
survives an embedded newline and a 300-byte port map); THIS build carries a real git identity
(macro defined — else FAIL loudly — shape-sane, not a placeholder).

**motion_result_test (6):** MotionOutcome wire pins + spellings; the §18.3 result-line golden;
every ✗ variant + FAULT_ABORT's causal code; the n/a honesty golden; NaN/+Inf/1e300 tokens
(no libc spellings); over-long/hostile names truncated with framing intact.

**run_summary_test (8):** the §18.3 block golden, clean AND faulted/browned-out/throttled;
heading n/a; MISSING hash; non-finite tokens; RunSummary bounded-copy value semantics;
summarize() additivity + Fake summary history; ALL FOUR shipped decorators forward summarize
through a full chain (the decorator-swallows-it hazard).

**level_filter_test (5):** the D-1 core (one tag filtered, no other); override-vs-global in
both directions; in-place re-set + clearLevels; records/summaries untouched + pair rule;
loud misuse (bad tag, full table).

**rate_limit_test (7):** flood drops counted + stamped cumulatively on survivors (first-N-pass
pinned); ONE notice per episode on resume, with exact text; Error/Warn never throttled;
per-tag isolation; summarize never throttled; wantsRecord pair rule over NullSink and a
consumer; loud nonsense budgets.

**tick_attribution_test (7):** attributed + other == total EXACTLY; the slow subsystem named
(+ deterministic tie-break); repeated phases accumulate; the last completed tick stable while
the next is open; abandonTick discards/preserves/re-arms; loud bracket misuse + reset;
tickPhaseName covers the vocabulary, spares render reserved.

**plausibility_test (8):** teleport fires ONCE per episode, recovers, re-arms, fires again;
heading teleports; dt scaling (same delta: fault at 10 ms, physics at 1 s); reset forgives
deliberate teleports, dt≤0 re-baselines; command audit fires on over-budget/non-finite and
passes exact-budget clamped output; volt recovery (untouched / NaN→0 V / clamped); **the
pipeline's own wiring under hostile speeds (the M23 closure — each invariant pinned
distinctly)**; loud config nonsense.

**controller_display_test (6):** healthy content; FAULT + FIRST fault by name (not the
cascade); zero rewrites in steady state + quantum-crossing repaints only row 0; battery jitter
vs real sag; the longest fault name fits 19 columns EXACTLY + seam truncation (HA-57 pins);
fake row bounds.

**run_report_e2e_test (13):** the byte-exact TRANSCRIPT (header → ticks → result line →
summary through one TermSink — the M2-closure golden); the stats formula on hand-fed streams
(overshoot 0.5 in exact, hold branch, waiting/idle/boot exclusion); truth cross-checks X+H
(§3.1); the inertial-overshoot non-vacuity case (§3.2); tank truth; the real two-motion run
(header first, one line per boundary, IN ORDER with the tick stream, one summary, LAST, real
hash present); SUPERSEDED + FAULT_ABORT=IMU_LOST reach the terminal; summary correctness under
injected faults (first is FIRST by time, worst dt is the 30 not the 22, brownout latched,
gating episodes counted, provenance carried); flood-in-vivo (counted + stamped + summarized);
NullSink cost through the full chain (wantsRecord false, builder never runs, attribution
absent, n/a line, real finalPose); scheduler attribution in vivo (exact phase quanta, stamped
records with quiet reserved slots, the overrun line names "mot"); observer re-entrancy is a
loud precondition; the hostile coherent report (§3.3).

---

## 6. Mutation campaign (35 mutations / 37 runs — each: mutate → BUILD-GATE → run → OBSERVE → restore → cmp-verify)

| # | Mutation | Observed |
|---|---|---|
| M1 | Result-line ✓/✗ swapped | **RED** 15 asserts |
| M2 | FAULT_ABORT loses its causal code | **RED** 2 |
| M3 | n/a branch inverted (fabricate numbers with no data) | **RED** 5 |
| M4 | Missing-hash ERROR line removed | **RED** 2 |
| M5 | MISSING replaced with plausible "0000000" (the landmine) | **RED** 3 |
| M6 | Summary first-fault always "none" | **RED** 1 |
| M7 | Summary hides drop counts | **RED** 1 |
| M8 | Summary fabricates heading zeros without data | **RED** 2 |
| M9 | D-1 severity comparison inverted (would silence ERRORS) | **RED** 5 |
| M10 | D-1 override lookup dead | first form **BUILD FAIL — GATE TRIPPED, discarded**; adjusted → **RED** 3 |
| M11 | D-2 record drops uncounted (the silent drop itself) | **RED** 5 |
| M12 | D-2 wire stamps removed | **RED** 2 |
| M13 | D-2 Error/Warn exemption removed | **RED** 2 |
| M14 | D-2 resume notice removed | **RED** 1 |
| M15 | D-3 phase overwrite instead of accumulate | **RED** 1 |
| M16 | D-3 worst-phase argmax → argmin | **RED** 5 |
| M17 | Scheduler never stamps phases onto records | **RED** 2 |
| M18 | Overrun-attribution line never emitted | **RED** 1 |
| M19 | D-5 episode gate broken (fault storm) | **RED** 2 |
| M20 | D-5 pose guard dead (never fires) | **RED** 12 |
| M21 | The boot-window gate removed (judge always) | **RED** 3 — the §4.1 fix is load-bearing |
| M22 | Wheel-volt recovery dead | **RED** 4 |
| **M23** | **BOTH pipeline audit calls severed** | **GREEN — hole found** (§4.2); closed; re-run **RED** |
| **M23a** | Command-audit call alone severed | **GREEN behind M23b — second hole** (§4.3); test strengthened; re-run **RED** 2 |
| M23b | Volt-recovery call alone severed | **RED** (case aborts: FakeMotor rejects the unrecovered NaN) |
| M24 | Boundary never reads the stats (all n/a) | **RED** 4 |
| M25 | Pre-empt not marked (SUPERSEDED lost) | **RED** 2 |
| M26 | Observer callback severed (result lines vanish) | **RED** 10 |
| M27 | Waiting-state records aggregated (fabrication) | **RED** 4 |
| M28 | Overshoot hold-branch removed | **RED** 1 |
| M29 | Reporter reads LAST fault as "first" | **RED** 1 |
| M30 | Controller display always repaints | **RED** 6 |
| M31 | finalPose fabricated as origin | **RED** 6 |
| M32 | Shared %.3g compaction threshold defeated | **RED** 2 |
| M33 | Battery start never read | **RED** 1 |
| M34 | Per-motion stats reset severed (stale aggregates) | **RED** 2 |
| M35 | FAULT_ABORT collapses to CANCELLED | **RED** 1 |

Post-campaign: all 15 touched headers `cmp`-identical to pristine snapshots; the only
"MUTATION" text in `include/`/`test/` is truth_integrator's pre-existing doc comment and the
closure test's own prose. Final re-green: **659 / 915,570**. (A process note: the pristine
snapshot directory initially contained SIX stale C4-era snapshots — a restore from one would
have silently regressed the tree; they were deleted before the first mutation. Recorded
because "restore from a stale snapshot" is a campaign-invalidating failure mode the C1 lesson
(never `git checkout` mid-campaign) did not cover.)

---

## 7. Verification (actually run, outputs as observed)

```text
$ cmake -S test -B build/test        # (re-run; picks up the git hash injection)
-- shulib build hash: v0.1.1-153-g0b4948a-dirty
$ cmake --build build/test && ./build/test/shulib_tests
[doctest] test cases:    659 |    659 passed | 0 failed | 3 skipped
[doctest] assertions: 915570 | 915570 passed | 0 failed |
[doctest] Status: SUCCESS!
```
(3 skipped = the two M3 acceptance stubs + the R3 GPS field-cal oracle (HA-01), unchanged.)

```text
$ <the ci.yml PROS-free guard grep, scope unchanged — diag/ and hal/ already covered>
GUARD 1 PASS: core is PROS-free (incl. all 13 new C5 headers)
$ <the ci.yml layering guard grep, scope unchanged>
GUARD 2 PASS: layering holds, core is sim-free
```

```text
$ arm-none-eabi-g++ -std=gnu++20 -Wall -Wextra -Wconversion -Wsign-conversion -Wshadow \
    -Werror -Os -mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=softfp \
    -c all_headers.cpp -o /dev/null -Iinclude        # generated list, ALL v2 headers
TU includes 102 headers
ARM CROSS-COMPILE: CLEAN
```
(After catching the §4.4 `%u` portability bug on the first attempt — the gate's first real
catch since A4 automated it.)

Working tree left uncommitted for review, per the brief. Register reconciliation grep clean in
direction 1 (no PROVISIONAL label without an HA id); HA-56/57 point at their in-tree sources.

---

## 8. Cost (constraint 6) — measured mechanisms, honest scope

- **NullSink record path:** `wantsRecord()` is false through the ENTIRE C5 chain (stamper →
  stats → NullSink), and an instrumented builder through that chain is never invoked (the A1
  proof, re-run one level up). Consequence pinned end-to-end: a full motion under NullSink
  completes with `hasPathData=false`, an honest n/a line, and a REAL boundary pose.
- **Attribution:** off unless a clock is injected — `attribution()` is null, zero attribution
  clock calls exist to make (structural: the optional is never constructed). On = ~6 virtual
  clock calls per tick, dev-build only.
- **Per-tick additions that DO run in competition:** the pose-delta guard (one pose read, one
  hypot, two compares) and the pipeline self-audit (a hypot + ~4 compares + per-wheel
  finite/range checks). No allocation anywhere; all bounded stack.
- **Honest scope:** these per-tick costs are argued-small and structurally allocation-free,
  measured only in the sense of the mechanism proofs above — no cycle-level benchmark was run
  (A1's ARM-asm-diff standard was applied to TRACE, not repeated here). If R3's loop budget
  measurements ever show the audit mattering, `MotionSchedulerConfig` is where a switch would
  go — none exists today because none has evidence it is needed.

---

## 9. What we now know for certain, and what we do not

### Known, with evidence

1. **The F9-sensitive schema space exists and is pinned**: `droppedRecords`/`droppedLines` +
   `tickPhase[8]` (2 spare slots) + the `TickPhase` vocabulary, wire-width-asserted; H1
   inherits an exact discharge table in diagnostics-plan.md. This was the one thing that could
   not slip, and it did not.
2. **The report tells the truth.** Clean plant: reported == physical truth to ~1e-12 in on all
   three drivetrains (bounds pinned at 1e-9). A REAL 3.79 in overshoot reports true to six
   digits. Hostile: belief-vs-truth ≤ 1.76 in, inside the documented C2/C3 envelope, asserted.
   And where there is no data, the line says n/a — fabrication is mutation-pinned out (M3/M8).
3. **Boundary reporting is structural.** The observer seam fires on every boundary including
   pre-empts; severing it fails 10 assertions across suites (M26); a routine cannot forget a
   result line, and re-entrant scheduling from a boundary is a loud precondition.
4. **Silent degradation is dead on the throttle path**: drops are counted, stamped onto the
   surviving wire, announced per episode, and summarized — each of the four visibility legs
   mutation-pinned separately (M11/M12/M14/M7), with the error path and the summary channel
   throttle-exempt (M13).
5. **Overruns now have a NAME**: attribution sums exactly (attributed + other == total), the
   deliberately slow subsystem is named at unit level AND in vivo through the scheduler
   ("worst mot"), and the naming pipeline (measure → stamp → overrun line) is mutation-covered
   (M15–M18).
6. **The missing-hash path is loud and placeholder-free** — golden-pinned in header AND
   summary, with anti-placeholder string pins (M4/M5), and this suite's own build carries a
   real `git describe --dirty` identity (a test fails if the injection ever rots).
7. **D-5 catches what it claims and stays quiet otherwise**: teleports (position and heading)
   fire once per episode and re-arm; recovery is real (NaN volts never reach a motor —
   FakeMotor would refuse them, observed under M23b); and the entire 915k-assertion corpus
   runs with zero false positives at the HA-56 defaults, including every hostile suite and the
   boot windows (§4.1's gate, itself mutation-pinned).
8. **Two green holes existed and are closed** (§4.2/§4.3) — both in the D-5 wiring, both now
   single-point tests that go red alone; and the build gate caught a non-compiling mutation
   exactly as C4's process fix intended.
9. **The seam additions are genuinely additive**: message-only sinks compile untouched against
   `summarize()` (the A1 additivity proof re-run for the new channel), NullSink still costs
   nothing through the longer chain, and all four shipped decorators forward the new channel
   (pinned together).

### NOT known, stated plainly

1. **Anything about real hardware.** The controller grid (HA-57), the plausibility envelope's
   real headroom (HA-56), real serial/terminal bandwidth against the default rate budgets, and
   the real cost of the per-tick guards are all R-phase measurements. The D-4 PROS adapter and
   the robot-side `SHULIB_BUILD_HASH` Makefile line are R1 glue that does not exist yet.
2. **Real attribution numbers.** In host sim the attribution clock is scripted; "localization
   4 ms, motion 2 ms" for OUR loop on a V5 is unknown until R1 wires the μs clock. The
   instrument is proven; its first real readings are not in.
3. **Whether the result-line/summary vocabulary survives its consumers.** D1's recipes and
   G2's PathRunner are the second consumers (per-leg results? scoring lines once a command
   registry exists?). `RunSummary`/`MotionOutcome` are wire-stable but NOT frozen — H1 decides
   whether SHUL/2 v1 carries them or defers to v2 (the discharge table says both are safe).
4. **The clean-plant truth-coincidence is a property of perfect sensors**, not of the reporter:
   with real noise the estimate-vs-truth gap grows toward the hostile numbers. The hostile
   bound (≤ 1.76 in observed, ≤ 5 in asserted) is the honest field expectation, and only
   R-phase data can tighten it.
5. **The stationary-target overshoot semantics (worst wander) are a definition, not a law.**
   If tuning sessions want a different turn-quality metric (e.g. heading overshoot past the
   target angle), that is an additive field discussion for E1/H1 — the current definition is
   documented at the vocabulary, not buried in code.
6. **Rate-budget defaults are untested against a real console.** 50 rec/s + 20 lines/s/tag are
   reasoned from serial arithmetic; a real dev session at R1 may want different numbers — they
   are per-session config, deliberately not register entries.

---

## 10. Deliberately left for later chunks (named handoffs)

- **→ E1 (`SdSink` + flight recorder):** the summary channel is on the seam for the blackbox
  to serialize; D-6's ring buffer stores existing records (now carrying drop counts and phase
  slots); D-7's triage block has `firstFault`+time and the attribution story to draw on.
  Estimator introspection fields (residual/Mahalanobis/covariance) were reserved at A1 and
  remain unpopulated until the correctors exist.
- **→ H1 (F9 freeze):** freeze `DebugRecord` + `GateReason` + `TickPhase` + `FaultCode`
  exactly as the diagnostics-plan discharge table inventories them; decide `RunSummary`/
  `MotionOutcome` wire carriage (v1 or v2 — both safe, both already wire-stable).
- **→ R1:** the PROS glue set: `ICharSink` stdout adapter (A1), `ILineDisplay` controller
  adapter (HA-57 settle), the μs attribution clock, and the `SHULIB_BUILD_HASH` line in the
  robot Makefile (per-build evaluation, unlike the host suite's per-configure).
- **→ D1/D2 (facade freeze):** `RunReporter` composes via `chassis.scheduler()` — no facade
  surface changed at C5, deliberately; if D1's recipes want `chassis.reporter()` sugar or
  per-leg trajectory results, that is exactly the not-frozen window's job to surface.
- **→ R3/R5:** settle HA-56 (tighten the envelope toward measured × margin — the invariant
  gains sensitivity) and HA-57 (count real columns).
- **→ G-phase:** the summary's ledger line grows a scoring line when a command registry can
  say what a motion scored; portMap becomes generated data at G1.

---

## 11. Freeze Register note (documentation contract #6)

**No freeze occurred at C5.** Freeze-adjacent acts, recorded:

- **F9 (H1)**: the record it will freeze GREW, append-only, exactly as the A1 header's
  register note prescribes: two counters + one 8-slot array + one index vocabulary + one
  fault code. Nothing reshaped; every addition wire-pinned by test. The diagnostics-plan
  discharge table is the freeze-day inventory.
- **F4 (HAL seam)**: `summarize()` added by the documented additive recipe (non-pure, no-op
  default); the additivity test proves no implementer broke. `ILineDisplay` is a NEW additive
  seam, explicitly outside the frozen ten, like `ICharSink` before it.
- **F6 (facade)**: untouched — C5 added zero facade surface (the reporter rides the Tier-3
  scheduler seam), so the D2 freeze review inherits no accidental C5 shapes.
- **`CompletedMotion`** (C2's staged type, not frozen but consumed): extended in place with
  six additive fields per the brief's rule 7 — no consumer's existing field moved or changed
  meaning (the C2/C4 suites ran unchanged throughout).

---

## 12. DoD checklist (brief §Definition of Done)

- [x] **Session header is the first record of every run and carries a real build hash** —
  `emitSessionHeader` via `RunReporter.sessionStart`; the e2e run pins header-first and the
  live `git describe --dirty` value in the transcript; MISSING is loud, placeholder-free
  (goldens + M4/M5). *Honest note: "first" is wiring order — the reporter cannot force a
  caller to call it first; the recommended wiring is documented and the e2e pins it.*
- [x] **Per-motion result line matches §18.3; numbers verified against ground truth** —
  byte-golden (§5), truth-checked on X/H/tank incl. a genuinely-overshooting plant (§3).
- [x] **Run summary fits one screen and is correct under injected faults** — six lines,
  byte-golden; first-is-first / worst-is-worst / brownout / gating / drops all pinned against
  injected-fault runs (§5 e2e).
- [x] **D-1 through D-5 implemented; D-2 and D-3's schema fields present in `DebugRecord`** —
  all five delivered (diagnostics-plan updated per-item); the fields are wire-pinned with two
  spare slots reserved.
- [x] **Controller-display content built behind a seam; PROS glue explicitly deferred to R1** —
  `ILineDisplay` + `FakeLineDisplay` + `ControllerFaultDisplay`; HA-57 registered.
- [x] **Throttling/dropping is counted and reported — never silent** — counted, stamped
  on-wire, announced per episode, summarized; each leg mutation-pinned (M7/M11/M12/M14).
- [x] **`NullSink` cost unchanged, measured** — the builder-never-invoked proof re-run through
  the full C5 chain + structural zero-cost for attribution-off; §8 states the honest scope
  (mechanism proofs, not a cycle benchmark).
- [x] **M2's "the run is legible in real time on the terminal" clause closes** — the
  transcript golden IS the clause, end to end on one sink; roadmap boxes flipped with
  evidence. *(M2 as a whole stays open: C6 salvage + C7 cutover remain, as sequenced.)*
- [x] **Suite green under strict `-Werror`; both guards pass; ARM gate passes** — §7, actual
  outputs; 102 headers; one real portability bug caught and fixed by the gate itself.
