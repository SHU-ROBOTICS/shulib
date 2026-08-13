# shulib v2 — Build Order

> **What this is.** The [roadmap](../roadmap.md) says *what* is left, grouped by milestone. This document
> says **what order to build it in, and why that order is the correct one** — the dependency spine,
> chunked into single-session units of work.
>
> **Ordering principle: correctness, not speed.** Where the milestone numbering and the dependency
> graph disagree, the dependency graph wins and the deviation is justified in place. Schedule is
> explicitly *not* an input to this ordering.

---

## How this document is used

- Work proceeds **one chunk at a time, in order.** A chunk is a coherent unit with a single testable
  Definition of Done — sized so it can be completed and verified without leaving anything half-built.
- A chunk is **not** finished when the code compiles. It is finished when its DoD is met *and* the
  documentation contract below is discharged.
- **No chunk starts before its predecessor is documented.** Half-finished chunks are how the
  dependency spine silently rots.

### The per-chunk documentation contract

Every chunk closes with all six. This is the process, not a suggestion:

| # | Deliverable | Why it exists |
|---|---|---|
| 1 | **Roadmap checkbox flipped, with cited evidence** (file + test name + case/assertion count) | The roadmap's own rule: evidence, not vibes. `[~]` if the DoD is only partly met. |
| 2 | **"You are here" updated** in `roadmap.md` | One-glance answer to "where are we?" — stale by one chunk is stale. |
| 3 | **Design notes in the header** — *why*, not just *what* | The reasoning is the part that can't be recovered from the code later. |
| 4 | **Test evidence recorded** — case count, what each test would catch, which mutations were proven red | A suite that stays green while the code is wrong is theater. |
| 5 | **Decisions recorded** — anything chosen where an alternative was viable, and why | Prevents re-litigating settled questions three chunks later. |
| 6 | **Freeze Register updated** if the chunk froze a contract | A freeze that isn't registered isn't a freeze. |

### Watching a chunk as it happens

Every chunk keeps a **live progress log** at `docs/internal/chunks/<CHUNK>-PROGRESS.md`, appended to
as the work happens — not written at the end. Watch it from a second terminal:

```sh
tail -f docs/internal/chunks/A2-PROGRESS.md      # substitute the current chunk
```

Each entry is one line, appended immediately when the thing happens:

```
[HH:MM:SS] START   <what is being attempted>
[HH:MM:SS] DONE    <what landed, + evidence: file, test name, counts>
[HH:MM:SS] MUTATE  <mutation> -> RED/GREEN (observed)
[HH:MM:SS] DECIDE  <choice> over <alternative> because <reason>
[HH:MM:SS] BLOCKED <what, and what is being tried instead>
[HH:MM:SS] FOUND   <discovery about existing code>
```

This is a **required deliverable**, not a courtesy: a log appended in real time cannot be
retro-narrated, so it is also the honest record of what actually happened in what order. The
`-COMPLETED.md` record is written from it, not instead of it.

Other live views: `git diff --stat` for the change surface, and re-running the suite at any time —
chunks leave the tree in a buildable state as often as possible.

### Rules that hold across every chunk

1. **Freeze only after two independent consumers.** A contract that has been exercised once has been
   exercised by its author. F6 does not freeze until both a hand-written auton *and* the recipe layer
   run on it.
2. **Host-provable work is proven on the host.** Bench time is spent only on what genuinely cannot be
   established off-robot.
3. **Every chunk ships its own adversarial tests**, including at least one proven-red mutation for
   load-bearing logic.
4. **A chunk that discovers a flaw in an earlier chunk fixes it there**, rather than working around it.

### Who writes what

This document orders the **library**. The competition content that rides on it — the actual scoring
routines (Phase F) and the authored paths (Phase G) — is **strategy authored by students**, who must
be able to explain and defend it. The engine is built here; the routines are theirs. Phase F chunks
therefore deliver *primitives and the engine that runs them*, and explicitly stop short of authoring
the season's routines.

---

## Current position

**Done: PHASE A IS COMPLETE (A1–A4).** M0 complete · M1 host-side (F4 + F5 frozen host-only) ·
M2 control layer (WS4) · M2 localization tier 1 (WS5) · **Chunk A1 — `DebugRecord` + `TermSink` +
fault discipline (WS13)**, closed 2026-08-01 ([completion record](chunks/A1-COMPLETED.md)) ·
**Chunk A2 — the host plant + closed-loop sim harness**, closed 2026-08-01
([completion record](chunks/A2-COMPLETED.md)) · **Chunk A3 — hostile fakes**, closed 2026-08-02
([completion record](chunks/A3-COMPLETED.md)) — every V5 misbehaviour class injectable and
composable, three real Localizer defects found and fixed, the M2 `<1°` acceptance test live with
a measured number · **Chunk A4 — the Hardware Assumptions Register + the ARM compile gate**,
closed 2026-08-06 ([completion record](chunks/A4-COMPLETED.md)) — the debt of building without a
robot is now **inventoried, not scattered**:
[`hardware-assumptions.md`](../hardware-assumptions.md) carries **49 falsifiable entries**
(33 invented / 13 reasoned / 2 measured-elsewhere / 1 mixed), each with source, confidence,
settling measurement, owning chunk and blast radius, bidirectionally reconciled with the tree
(zero orphans, grep-verified) — this is R3's day-one checklist; and CI now **holds** the ARM
line (`arm-compile-gate` job: every v2 header, generated list, compile-only by honest scope,
proven to catch what the host build cannot).

**Phase C is COMPLETE (C1–C8, 2026-08-06 → 2026-08-11). Chunk C1 — `IMotion` + the motion
primitives — closed 2026-08-06** ([completion record](chunks/C1-COMPLETED.md), committed `7e54826`/`b3cbf39`): the
library can now be told "go to that spot". `MoveToPose` runs three DECOUPLED per-axis loops
(translate + rotate simultaneously — the holonomic thesis, mutation-proven), plus `TurnTo` /
`StrafeTo` / `HoldPose` / `DriveBrake`; every motion is watchdog-bounded, reports an
`ExitReason`, emits per-tick records, survives A3's composed hostility, and carries the two A3
handoffs (the wait-for-live-estimate contract; the `OdoStallCheck` spin-vs-motion cross-check →
`ODO_STUCK` — the only dead-encoder defence until Phase E). Full-routine accuracy measured:
clean-plant chain error FLAT in move count (0.24 in worst across 5→40-move routines — absolute
targeting does not compound); full-hostility worst 4.1 in over ~95 s attributed to M2 drift
(err-vs-time 0.028 in/s; Phase E's problem), with a 2%-miscalibration twin proving the
distance-attribution diagnostic reads exactly 2%.

**Chunk C2 — `MotionScheduler` — built and verified 2026-08-06**
([completion record](chunks/C2-COMPLETED.md), committed `1206dbe`): the library now
RUNS a routine. One active motion enforced structurally (single slot + PRE-EMPT: the old motion
is cancelled inert before the new one exists to command); `async()` / `waitUntilSettled()` /
`waitUntil(pred, timeout)` / `cancel()`; the cancel safe state is DEFINED (0 V +
`BrakeMode::Brake`, applied synchronously by the call itself — HA-53) and proven reached; C1's
deferred fault policy is decided (abort-and-brake on a NEW ODO_STUCK — servoing against a lying
estimate is worse than stopping; continue-degraded on IMU_LOST / BROWNOUT / GPS_GATE_REJECT /
MOTOR_OVER_TEMP / LOOP_OVERRUN; configurable mask; A/B-measured: the abort cuts the
dead-encoder runaway from 42 in @ 6 s to 4.4 in @ 1.5 s); every wait is bounded (explicit
timeouts, watchdog-bounded settle, a stalled-pacer guard that converts the un-catchable
frozen-clock hang into a loud precondition); the check.hpp task-boundary catch is real;
`activeCommandId` is finally assigned (a pair-rule-preserving stamping sink); the scheduler
owns HealthMonitor/LoopMonitor between motions. Scheduled routines reproduce C1's accuracy
baseline BIT-IDENTICALLY (equivalence-pinned; clean 5→40-move chains flat in count, hostile
worst 4.13 in). All three C1 handoffs discharged; `ExitReason::Cancelled` +
`MotionState::Cancelled` appended via the documented additive paths.

**Chunk C3 — `hDrive()` + the pseudo-inverse — built and verified 2026-08-06**
([completion record](chunks/C3-COMPLETED.md), committed `cad33bc`): **the 15″ H-bot
runs the same motion code as the 24″ X-bot, unmodified — the M2 DoD, measured.** The H-drive
ships as a `MatrixKinematics` preset (§13 #15) with rows derived from rigid-body kinematics
and pinned by an independent from-scratch oracle (which mutation M4 proved is the ONLY defence
against geometry sign errors — they cancel end-to-end through the shared plant, exactly as
drive_plant.hpp warned). `forward()` is the full `(AᵀA)⁻¹Aᵀ` (the M1 deferral discharged,
F5-safe): every previously-accepted table runs the historical computation VERBATIM and is
bit-identical by XOR checksum against the pre-C3 build; near-degenerate geometry is REJECTED
by a relative-Gram-determinant guard (the hole the relaxation would otherwise open). Strafe
authority = derivable speed ratio × the HA-54 traction derate (0.35 default); C1's D11 reading
CONFIRMED against the physics (the F5 "|vy|/|vx|" phrasing retired as ill-defined). The
fallback is turn-WHILE-drive — authority-limited translation with rotation free, never a
sequenced decomposition — and provably better than the crab it replaces (1.92 s vs 2.71 s over
the same 40 in lateral); `strafeFallbackActive` is populated at the one producer and renders
as TermSink " SFB" (a silent fallback fails six independent tests). Same-auton accuracy, X
next to H: clean 5→40-move chains flat in count on BOTH (X 0.228→0.236 in, H 0.225→0.238 in),
hostile worst X 4.13 in / H 4.03 in, H paying only ~1–4% extra time (the turn-while-drive
migration keeps MoveToPose legs near X speed; only explicit StrafeTo legs pay the 21 in/s
crab).

**Chunk C4 — the `Chassis` facade — built and verified 2026-08-10**
([completion record](chunks/C4-COMPLETED.md), committed `2a1cfc3`): **the public API
an auton is written against exists — and F6 is deliberately NOT frozen** (D1 exercises it as a
second consumer; D2 freezes). `chassis/chassis.hpp` wraps the stack in blocking verbs
(`moveTo`/`strafeTo`/`turnTo`/`followTrajectory` + candidate `brake`/`hold`) plus the
frame-explicit manual verb `drive(ChassisSpeeds, Frame)` — `Frame` is a required parameter, so
silent frame confusion is now a compile error. C2's structural handoff is closed: the facade
is the ONE composition root (scheduler + every motion from `scheduler.deps()`), making
command-id stamping structural — mutation-proven (a raw-deps verb fails 138 assertions). The
saturation choreography was EXTRACTED to `motion/command_pipeline.hpp` (bit-identity-pinned)
so `drive()` reuses one pipeline instead of re-deriving it; a shared health-observable helper
replaced three grown copies. The standalone promise is tested as code: a working Chassis built
in plain C++, no file, no builder. Facade routines are **BIT-IDENTICAL to the scheduler-built
twin, clean and hostile** — C1/C2/C3's accuracy baselines carry over verbatim — and the clean
5→40-move sweeps re-run green through facade verbs on X AND H, plus a NEW tank
turn-then-drive baseline (flat, ≤1.0 in). Every guarantee re-pinned THROUGH the facade:
ODO_STUCK abort, watchdog bounds, cancel safe state + panic stop, boot-window wait, IMU_LOST
continue, hostile bounds, SFB visibility. New find: a blocking verb's stack motion DANGLES in
the scheduler slot if the wait throws — the DetachGuard cancels on unwind (without it the
suite literally segfaults, observed under mutation M5).

**Chunk C5 — per-motion results + session header + D-1…D-5: closed 2026-08-10, committed
`be6d129`** ([completion record](chunks/C5-COMPLETED.md), live log in
[C5-PROGRESS.md](chunks/C5-PROGRESS.md)). The run is now legible end to end on the terminal —
§18.5 session header (build hash from `git describe --always --dirty`, MISSING is loud, never
plausible) → the stamped tick stream → a §18.3 result line at every motion boundary
(structural, via the scheduler's new observer seam; SETTLED/TIMEOUT/CANCELLED/FAULT_ABORT/
SUPERSEDED) → the one-screen §18.3 summary block, all byte-goldened. The F9-time-sensitive
schema space is RESERVED (`droppedRecords`/`droppedLines` + `tickPhase[8]` with 2 spare slots;
diagnostics-plan.md carries the discharge table), and diagnostics-plan's D-1…D-5 all landed:
per-subsystem levels, counted-stamped-announced throttling, tick-time attribution that NAMES
the overrun's consumer, the controller-screen seam (PROS glue → R1, HA-57), and the
plausibility invariants (`FaultCode::Implausible`, HA-56). Result-line numbers are
truth-checked on all three drivetrains (clean: reported == truth to ~1e-13 in; a real 3.79 in
overshoot on an inertial plant reports true to 6 digits; hostile: ≤ 1.76 in believed-vs-true).
35 mutations: 33 red, **2 green holes found and closed** (the D-5 pipeline wiring was
invisible to all 915k assertions; then its command-audit half hid behind its volt half), 1
build-gate trip. The ARM gate caught a real portability bug (`uint32_t` is `unsigned long` on
target — a `%u` that would not compile at R1).

**Chunk C6 — legacy salvage: DONE 2026-08-10, committed `d743a30`**
([completion record](chunks/C6-COMPLETED.md), live log in [C6-PROGRESS.md](chunks/C6-PROGRESS.md)).
The last look before the C7 deletion is complete: **all 34 legacy files classified** (superseded /
salvaged / discarded, each with its reason), the three roadmap salvage claims **verified in source**
(the odom new-vs-average-heading bug is at `legacy odometry.cpp:338-355` exactly as the roadmap
said; the logger's three defects confirmed by grep; `RobotCommands` turned out to have **no
executor at all** — three data representations, zero consumers, so the salvage is knowledge, not
code), and the **port list is empty by audit** (11 distinct live legacy bugs catalogued; nothing
worth carrying). Products: [`legacy-command-vocabulary.md`](../legacy-command-vocabulary.md) (7 ids
mechanically cross-checked from 4 sources, mapped to v2 or gap-flagged — `NONE` boundary markers →
G2 decision, seat/settle wiggle → F2 candidate; 7 concrete G4 importer requirements incl. the
unit-instability trap), the legacy-measured reference table in `hardware-assumptions.md` (real odom
offsets, 36000-tick provenance for HA-16, measured kS ≈ 1.9 V vs HA-45's 1.0 V placeholder), and
the diagnostics-plan C6 note (per-wheel stuck attribution → E-phase; veer triage → H2/R-phase).
**The safe-to-delete verdict is unconditional** (C6-COMPLETED §8): nothing in v2 references
`legacy/`, the irreplaceable knowledge has left the directory, and deletion removes the tree's only
copied third-party code (LemLib `pose.*`). One non-gating follow-up lodged with G4: lift the CSV
specimen from git history (`git show 357d3f2:src/legacy/autonomous_commands.csv`).

**Chunk C7 — cutover and deletion: DONE 2026-08-10, committed `3f15094`**
([completion record](chunks/C7-COMPLETED.md), live log in [C7-PROGRESS.md](chunks/C7-PROGRESS.md)).
**Phase C is COMPLETE; M2's structural clause is closed — the new tree is the only tree.** In
brief order: `src/main.cpp` rewired onto the v2 core and proven compiling with `legacy/` still
present (`make EXCLUDE_SRCDIRS=./src/legacy` — attribution preserved); then `src/legacy/` +
`include/legacy/` deleted (34 files = C6's exact classified set; recover via
`git show 691c656:<path>`); then the CI PROS-free guard broadened to ALL of `include/shulib/`
and the layering guard to the same scope minus `sim/` (both mutation-proven red); plain `make`
green — first time since June — producing `bin/hot.package.bin` + `bin/cold.package.bin`. The
HAL seams in main.cpp are shipped fakes marked TODO(R1) line by line: **the package uploads and
boots a diagnostics banner; it CANNOT drive hardware and has never run on a robot** (R3's
clause, explicitly open). Also at C7: the README rewritten as an honest front door (its
commands executed as written), and the docs reorganized — `docs/` = public documentation
(stands alone, zero references into internal, proven by removing `docs/internal/` and
re-running the link check), `docs/internal/` = the process record (this file, `RESUMING.md`,
`chunks/`, `transcripts/`) as a self-contained removable unit for the eventual squash-merge to
`main`. Suite unchanged 659/915,570/3 skips; ARM gate 102 headers clean.

**Chunk C8 — the manual: DONE 2026-08-11, committed `3437d72`**
([completion record](chunks/C8-COMPLETED.md), live log in [C8-PROGRESS.md](chunks/C8-PROGRESS.md)).
15 chapters (~22,200 words) written for someone who is not a robotics expert, with the code
examples living in `test/guide_examples_test.cpp` and **quoted verbatim** by the prose so CI
catches rot — the anti-staleness mechanism is the deliverable, not the word count. Chapter slot
**09 was deliberately left vacant** for D1. The tutorial was followed start to finish as written
(which found and fixed a wrong instruction: the build auto-discovers new test files, so "re-run
configure" was false). [`guide-maintenance.md`](guide-maintenance.md) is the companion procedure.
*(One claim in C8's record — that the broad removability grep was "empty" — was **corrected at
D1**: it returns 5 pre-existing prose mentions, none of them links. The property holds; the
stated evidence for it did not. See C8-COMPLETED §3.)*

**Chunk D1 — the Tier 2 recipe API: DONE 2026-08-11**
([completion record](chunks/D1-COMPLETED.md), live log in [D1-PROGRESS.md](chunks/D1-PROGRESS.md)).
**The facade held its second consumer with zero changes required — and F6 is still NOT frozen.**
`chassis/routine.hpp` is an **EAGER** fluent chain (each step runs the moment it is chained, so a
routine runs in the order it reads); the deferred `.run()` alternative was analyzed and rejected
because it re-opens the read-order/run-order split the moment recipe and facade calls mix, and it
adds the built-but-never-run misuse door that eager structurally cannot have. Twelve steps, each
exactly one delegated facade call; the only arithmetic in the layer is bearing argument-computation
for `face`/`driveTo`, pinned bit-identical to the hand-written `turnTo(atan2(...))` idiom AND
backed by absolute ground-truth arrival on tank, where a wrong bearing cannot reach the target.
**Chain error policy, decided and pinned:** first failure → stop → safe state (`cancel()` ⇒
0 V + Brake) → skip the rest (counted, logged) → report via `RoutineResult`; preconditions throw
through untouched, because a programming error must not be laundered into "the chain stopped".
The recipe twin is **bit-identical to the facade arm, clean and hostile** — three layers, one
physics. 19 mutations executed, 19 red, **2 green holes found by pre-analysis and closed with
sole-detector tests** (a no-op `startAt` was invisible because every rig auto-seeds the estimate;
speed budgets could vanish in delegation because the twin only varied timeouts). Guide chapter 09
fills C8's reserved slot. **D1's real product is the facade critique** (D1-COMPLETED §2, D2's
input packet): the headline is that **every duration on the frozen surface is a raw `double`** —
`hold(500)` compiles and holds pose for 500 seconds inside a 15-second match — while `units::Time`
and `_s`/`_ms` literals already exist. That is cheap to fix now and a versioned migration after D2.

**Chunk D2 — the F6 freeze: DONE 2026-08-12**
([completion record](chunks/D2-COMPLETED.md), live log in [D2-PROGRESS.md](chunks/D2-PROGRESS.md)).
**F6 is ✅ LOCKED — and for the first time a freeze in this project means something mechanical.**
Three things were wrong when the brief was written, and all three are fixed. The register row
enumerated five verbs while the header exposed twenty; it now lists the whole surface by group and
**names its exclusions**, `Routine` first and explicitly — silence in `chassis/` would have read as
"frozen". "Frozen" had no machinery behind it: the register promised changes come only by version
bump and migration, but no version identifier existed anywhere in the tree, so F1–F5 were locked
against a promise with nothing under it; `include/shulib/version.hpp` now carries API 2.0 plus a
written breaking-vs-additive policy that F7/F8/F9 will reuse. And nothing enforced a freeze at all —
`test/f6_signature_pin_test.cpp` now pins every frozen member's exact type at compile time and fails
the build **naming F6 and the member**, with remediation instructions in the message.

The docket: all nine D1 §2 items ruled, each with its rejected alternative, plus A1–A4/B1–B3/C1–C6
and a re-check of all 18 rows of C4's inherited-shape ledger (C4 wrote "anything missed here becomes
permanent at D2" — that sentence was aimed here). The headline change is **time is now typed**:
`units::Time` across the whole surface, so `hold(500)` — a match-losing 500-second hold from someone
thinking in milliseconds — no longer compiles. The retype moved no number, and could not: `Time` is
canonical seconds under F3, so `.value()` at the motion boundary is the identity; proven by staged
full-suite output diffs before any new test landed. `wait(units::Time)` was adopted as an additive
void verb, which also let `Routine::pause` become a pure delegation — restoring "every step is
exactly one Chassis call".

44 mutations executed, **two green holes found and closed**. The first is a genuine subtlety: a
`noexcept` drop on a non-overloaded member survived the exact-cast pins, because a cast that *adds*
`noexcept` is accepted — closed with compound `{ call } noexcept` requirements. The second is the
one that matters most on a field: `.hold(300_s)` where `300_ms` was meant passed the guide's own
example untouched, because the case asserted outcomes and never the clock. Closed with a
hand-computed simulated-clock bound; independently re-verified to fail alone.

**Chunk D3 — the cookbook, the generated reference, and the `Routine` freeze: DONE 2026-08-12**
([completion record](chunks/D3-COMPLETED.md), live log in [D3-PROGRESS.md](chunks/D3-PROGRESS.md)).
**Phase D is COMPLETE. `Routine` is frozen as its own register row F10**, and the documentation is
now self-maintaining in a way it has never been.

The architecture is three artifacts with three different staleness strategies, because generation
alone does not solve staleness. `tools/api_doc_tool.py` **generates** `docs/api/` from the headers —
that part cannot drift, because it *is* the headers. `docs/cookbook/` (5 chapters, 14 recipes) is
**hand-written**, because "how do I write a left-side auton that scores twice and parks" is not
derivable from signatures; its examples are compiled, run against the plant, graded on ground truth,
and quoted verbatim. And the third piece is the one that makes the promise real: a **doc-coverage
gate** that fails the build, naming the member, when a public member ships undocumented — because a
generator extracts whatever comments exist, so an undocumented member is silently absent while the
reference still *looks* complete. That is worse than a stale document: nothing looks wrong.

All four doc gates (coverage, freshness, verbatim, removability) now run at **build time**, not in a
script someone remembers to run. That distinction is the chunk's biggest find: **two of these
properties were never mechanized at all.** The verbatim rule existed only inside an internal verify
script, so a chapter's `.hold(300_ms)` → `.hold(300_s)` built clean and passed the entire suite; the
same gap meant `README.md`'s front-page example had never been compiled. Removability was equally
unenforced — a planted link into `docs/internal/` passed everything. Four green holes total, each
closed and proven red, including two of the most instructive kinds: the coverage gate initially
**accepted an empty `///`**, reproducing inside itself the exact "nothing looks wrong" failure it
exists to prevent; and the generated reference **leaked an internal path into public docs**, because
it extracts header comments and a header comment named `docs/internal/`. The lesson worth keeping:
**a gate's exclusion list is where its holes live.**

The cookbook did its job as `Routine`'s critical second consumer: **zero changes to `Routine` were
required**, an 8-item critique came out of it, and it found one real **bug** — `lastTrajectory()` on
a routine that had never run a trajectory reported a *successful* one, because a value-initialized
`TrajectoryResult` reads as Settled with 0 of 0 legs. Fixed at the layer that owns it, no surface
change, pinned. Rulings: `Routine` freezes (every critique item is additive under `version.hpp`, so
freezing forecloses nothing) as **F10, a new row** rather than an F6 amendment (different tier,
versions independently), enforced by **37 pins** including five `noexcept` drops on non-overloaded
members — D2's hole #1, deliberately not re-opened. `then()` is excluded entirely: its callable
contract is a placeholder until F1/F3 build mechanisms.

**Honest partials, both `[~]` with the remainder named:** nothing is **published** — there is no site
and no Pages config, so the output is web-portable and the publish path is written up
([docs-publishing.md](docs-publishing.md)), which is not the same as published; and **no new reader
has read the cookbook cold**, which cannot be closed without a person. An owner is named rather than
left hanging as C8 left the same clause.

*Found during verification and fixed at D3:* the freshness gate runs before any C++ compiles, so a
changed frozen signature trips "API REFERENCE IS STALE — run this command" **before** the F6/F10 pin
that names the actual problem. The pin still fires on the next build (verified), so nothing is
papered over — but a reader who obeyed only the first message would make a breaking change look
official on the way there. The freshness message now says so.

**Chunk E1 — `SdSink`, the flight recorder, and estimator introspection: DONE 2026-08-12**
([completion record](chunks/E1-COMPLETED.md), live log in [E1-PROGRESS.md](chunks/E1-PROGRESS.md)).
**Phase E opens: a run is now recoverable after the fact, with no laptop.** `diag/sd_sink.hpp` writes
a versioned, session-stamped, fixed-width binary blackbox through a new `hal::IBlockSink` seam;
`diag/blackbox_reader.hpp` reads it back. The decoder ships with the encoder because **a format
nothing can read is not a record** — and a cut file decodes up to the cut and says so, which is
exactly what a brownout leaves behind. D-6's flight recorder (RAM ring, always on, dumped only when a
fault fires) and D-7's triage block both landed.

The three tensions the brief named are ruled. **T1:** the standing no-background-task decision beat
`build-order.md`'s "off-task writes" — host determinism is what makes every closed-loop test here
reproducible from a seed, and a caller who under-flushes loses frames *visibly* via drop-and-count,
where a task's cost would have been invisible and intermittent. **T2:** a new sibling seam rather
than redefining `ICharSink`'s line contract, which is load-bearing for TermSink's goldens and would
have been invalidated silently. **T3:** half-closed and reported `[~]` — the introspection path is
complete and proven with a synthetic corrector emitting every `GateReason`, but **no real gate
exists, `gateMahalanobis` is 0 until E4, and nothing here certifies `< 1°`.**

**Two pre-existing defects, fixed in the layers that own them.** `DebugRecord::fault` **had no
producer anywhere in the tree** — TermSink has rendered ` flt=NAME` since A1 and chapter 11 has
documented it since C8, and it could never once have appeared on a real run; D-6's entire trigger
depended on it. Now stamped by the scheduler from the FaultLatch, proven with a real `LoopMonitor`
overrun rather than an injected fault. Separately, only `MoveToPose` stamped the applied-correction
fields, so the fusion story was blank for every other motion.

27 mutations, **two green holes** — both the same class, and worth naming because it generalizes:
*the suite tested that the FORMAT could carry the sink's self-description, never that the SINK filled
it in.* The end frame's counts and the header's epoch/ring/budget could each be zeroed with the whole
suite green. The single most instructive result is **mutation 1b**: moving a field in the encoder
*and* the decoder together left the round trip **perfectly green**, and only the byte-exact per-field
golden caught it — the brief predicted that class of blindness, and E1 demonstrated it rather than
arguing it. One green mutation was correctly ruled *not* a hole (an unreachable half-frame guard);
saying so beats inventing a test for dead code.

*Found during independent review and documented at E1:* "refuse, never misread" is about
**interpretation, not integrity**. Magic, version, layout, truncation and implausible values are all
caught — I corrupted a real blackbox four ways to confirm it — but there is deliberately **no
per-frame checksum**, so a bit flip landing on a merely-wrong value decodes silently. A considered
trade (a CRC costs bytes and cycles on every tick, and the card carries hardware ECC), now stated in
the reader's header so **H1's SHUL/2 cannot inherit a guarantee that was never made** — a wire over a
lossy link is exactly the consumer that would need its own frame check.

Honest gaps: nothing has touched an SD card (the `/usd/` adapter is R1's), ring depth / RAM budget /
flush cost are guesses registered as **HA-58/59/60**, and **D-8 was not delivered** — it did not fall
out for free and is re-homed to F2 with the reasoning recorded. E1 freezes nothing.

**Chunk E2 — `GpsCorrector`, the first REAL corrector: DONE 2026-08-12**
([completion record](chunks/E2-COMPLETED.md), live log in [E2-PROGRESS.md](chunks/E2-PROGRESS.md)).
The introspection path E1 built now carries decisions a real gate actually made.
`localization/gps_corrector.hpp` implements `ICorrector` behind the unchanged M2 signature, with
adaptive R from `rmsError()`, latency compensation out of an odometry history ring, a
stale-sample guard (one measurement folded once against the ~50 ms camera cadence, so correction
strength no longer depends on loop rate), high-yaw-rate rejection, a sensor-quality ceiling, and
a normalized-innovation gate whose bound WIDENS with dead-reckoned travel. Suite **794 /
1,081,382**, both guards and the ARM gate (111 headers) clean, **20/20 mutations red**.

**Three rulings, each with its rejected alternative.** **T1 — the gate is a normalized
innovation, and is named that.** A Mahalanobis distance normalises by a filter-ESTIMATED
covariance; the complementary tier has none, so `gateMahalanobis` stays 0, `RejectedMahalanobis`
is never raised, and `GateReason` gained `RejectedNormalizedInnovation` / `RejectedStaleFix` /
`RejectedSensorQuality` (append-only, wire-pinned). Rejected: calling it Mahalanobis anyway on
the grounds that an assumed isotropic S makes the scalar ratio one — rejected because the
assumption is the entire content, and one field holding both an earned and an asserted quantity
makes the difference invisible. **T2 — the accuracy claim is an aggregate, and says so.** Over 8
seeds of a 60 s hostile run, MEAN final and worst-case error are lower with the corrector
(per-seed 7/8 and 6/8). The magnitude-free claim is stronger and is asserted per seed: a known
6-inch position error is HEALED, while dead-reckoning carries it to the end of the run — which is
the actual mental-model change. Rejected: shopping for the friendlier 30 s closed-loop scenario
that does pass 8/8. **T3 — heading stays IMU-owned, provably.** The proposal carries the
PREDICTED heading, so the GPS's never leaves the corrector even if a future policy read
`fieldPose.heading()`.

**T4, a ruling the brief did not anticipate: the corrector does NOT own frame or lever arm.**
The brief assigned it "frame/lever-arm reduction"; `hal/gps.hpp`, `hal/gps_conversion.hpp` and
`fake_gps.hpp` all assign it to the HAL edge in writing, with `gps_conversion.hpp` saying **ONE
owner** and naming double-subtraction as the failure. A corrector that re-did either would be
right against `FakeGps` and silently wrong against the R1 adapter — invisible to host tests. So
E2 owed those two traps PROOF, not code: seven `[oracle]` cases pinning the conversions from the
geometry by hand, at seven headings with both lever-arm components non-zero.

**Two findings that predate the chunk.** (1) **HA-07 had no code and no test.** The metres→inches
obligation lived in `gps_conversion.hpp` as prose addressed to a future adapter author, and the
existing conversion tests pinned the scale against `kMetersToInches` imported from the header
under test — a wrong constant satisfied both sides of the `==`. Fixed where it belongs
(`gpsRmsErrorToCanonical()`), pinned against the definition of the inch. (2) **A hard 12-inch
ceiling on what any corrector can repair**, owned by `ComplementaryFusion::innovationGate` and
observed live: an estimate 29 inches out never recovered with a good GPS in view. Recorded for
E4, not patched from inside a corrector.

**The mutation hole (one green of twenty):** the Localizer substitution rule's `reason == None`
guard is dead code with ONE corrector and load-bearing with two — a silent source could stamp
`RejectedNoFix` over a tick that actually applied a fix. Latent until E3. Closed with a
two-corrector case that fails alone.

Honest gaps: nothing has seen a GPS; seven new invented constants (**HA-61…HA-67**); the accuracy
gain is modest because the modeled sensor noise and the modeled drift are the same order, both
invented; E2 bounds drift but does not recover a grossly wrong estimate. E2 freezes nothing.

*Reviewer's independent verification.* The frame and lever-arm math was re-checked against a
**from-scratch geometric oracle written for the review** — 23 assertions with hand-computed
literals: the East/North transform at two different North orientations, pure-East and negative-North
cases that catch an axis swap or a sign flip, the metres→inches scale written as `1.0 / 0.0254`
(the definition of the inch) rather than imported, and lever-arm removal at four headings with both
components non-zero, since a lever-arm sign error is invisible at heading 0. All 23 pass. That check
exists because a conversion shared between code and test cancels its own sign errors — the failure
that bit C1, C3 and C4.

That is also exactly what E2's HA-07 finding was: the pre-existing test did
`using shulib::hal::kMetersToInches;` and then asserted against `kMetersToInches`, so a wrong
constant satisfied both sides of the `==`. Confirmed by reading the test at `73b8e7f`.

*Process failure, self-reported by the chunk and worth keeping.* A `git checkout` on a file holding
uncommitted work (the brief forbids it) plus a `head`-induced SIGPIPE left `gps_conversion.hpp` with
the metres→inches multiply **deleted** mid-campaign. It was **caught by the mutation count dropping,
not by anything in the report** — which is the argument for counting what a campaign ran rather than
trusting its summary. Harness now traps PIPE and never checks out. Tree integrity re-verified at
review: the multiply is present at `gps_conversion.hpp:75` and `:101`.

**Chunk E3 — `AprilTagCorrector`: DONE 2026-08-13**
([completion record](chunks/E3-COMPLETED.md), live log in [E3-PROGRESS.md](chunks/E3-PROGRESS.md)).
**The library can now correct heading — the first absolute yaw source it has ever had.** Three new
headers (`hal/vision_conversion.hpp` for the corners→pose PnP, `localization/tag_map.hpp`,
`localization/apriltag_corrector.hpp`), plus the heading path through `correction.hpp`,
`complementary_fusion.hpp` and `localizer.hpp`. Suite **867 / 1,091,167**; ARM clean at 114 headers;
44 mutations, 43 red, three holes found and closed. **15 new assumptions, HA-68…HA-82.**

**T1 — yaw landed via the documented additive path, and the structure is the interesting part.**
`FusionResult::headingNudge` is a bounded *increment*; the Localizer accumulates it into a
**persistent bias** and publishes `imu.heading() + bias` as the tick's final write. The accumulator
is not optional: the Localizer re-reads the IMU every tick, so a nudge applied to the published pose
alone would be discarded on the next one — **the M2 red team's exact failure mode**, rediscovered
structurally rather than by accident. A consequence nobody had anticipated was ruled the same way:
the odometry delta must be re-expressed under the learned bias, or the *reported* heading improves
while position keeps accruing `bias × distance`. With no heading corrector present, both paths are
**bit-identical to pre-E3 by construction** (`==` on doubles, proven).

**T2 — shulib ships no tag map, deliberately.** Nobody on this project can cite a table of AprilTag
field poses, so inventing one and shipping it would be presenting a guess as a fact.
`TagMap::add()` refuses an entry without provenance, or with a duplicate id. **T3** — PnP is a free
function the corrector does not even include. **T4** — a `poll()`/`propose()` split, with cost
*pinned* by replacing the global allocator: **0 allocations across 20,000 ticks**, and the counter
was proved with a positive control before being trusted.

**Three green holes**, the middle one worth keeping: PnP scale derived from a single axis; the
**singular-system guard**, found by fuzzing 4M corner sets and then searching degenerate families,
where an unguarded solve cheerfully reports *"a tag 5.99 inches away"* from four collinear pixels;
and `providesHeading` not being load-bearing.

**Two entries in the record that matter more than the feature.**
*(1)* The chunk **overclaimed a bug and struck it.** It reported a "12.1013° overshoot" as a headline
finding, having read `|bias − 12| = 0.101` and **inferred the sign** — it was 11.8987, an
*undershoot*, and the tolerance was merely too tight. Measured properly, neither version overshoots
and the difference is ~0.1% in convergence rate. The code change survives on algebra, not on
evidence, and the associated mutation is recorded as **a GREEN it could not honestly close**,
because any test tight enough to catch it would pin an invented constant. Retracting a finding is
harder than reporting one, and the record is better for it.
*(2)* **A reversed corner winding is catastrophic and completely silent** — 180° of heading error
with the reprojection residual at machine zero, because a mirrored pose reprojects onto the same
four pixels. No software self-check can detect it; only a physical tag can (HA-69, R2's).

*Reviewer's independent verification.* I wrote a **from-scratch projection oracle** — the pinhole
projection derived from the header's stated frames, sharing no code with the solver — and ran the
PnP against it over **7 geometries / 70 assertions**: camera yawed both directions, off-centre
mounts, tags raised above the camera (confirming `t_y` is correctly discarded), tag headings both
ways. Every case recovers the pose I projected. **I also independently confirmed the winding
finding**: reversed corners give `valid=true`, `reprojectionError=0`, and heading `0°` where `180°`
is correct. That claim is not a caveat someone wrote defensively — it is real, and it is the single
most dangerous thing in this chunk.

*Also fixed at review:* the README's "expected output" block still showed **659 / 915,570** while the
same file claimed 867 / 1,091,167 twenty lines earlier — a beginner following the build instructions
would have concluded they had broken something. It now shows the *shape* and says the counts grow.
Fourth instance this session of prose going stale in a way no build gate can see.

**Chunk E4 — the 5-state SE(2) EKF: DONE 2026-08-13. PHASE E IS COMPLETE.**
([completion record](chunks/E4-COMPLETED.md), live log in [E4-PROGRESS.md](chunks/E4-PROGRESS.md)).
`EkfFusion` implements the unchanged `IFusionPolicy`; no caller moved. All three of E4's inherited
debts are paid or precisely scoped: the 12″ ceiling is **replaced on the EKF tier** by a
Mahalanobis gate that scales with the filter's own covariance (the same 20″ fix is refused when
fresh, ν = 19.7, and accepted after 360″ blind, ν = 2.70); `gateMahalanobis` and `covarianceTrace`
are filled from a real `S = H P Hᵀ + R` and re-read from decoded blackbox bytes; and two
disagreeing correctors are now **resolved by their stated σ** rather than merely bounded, matched
against the inverse-variance weighted mean computed independently in the test.
**The DoD line "the EKF beats the complementary filter" was NOT met and was not tuned toward** —
0.351″ vs 0.225″ mean final error over 8 seeded 60 s runs, losing 7 of 8 — so the complementary
tier stays the shipped default and both tiers stay selectable and tested. See the ruling block
under the E4 entry below.
Suite **915 cases / 1,521,419 assertions**; ARM gate 115 headers; **36 mutations, 34 red, 2
recorded GREEN with their measurements, 0 build-fail, 0 skipped**; 1 line of dead code removed;
nine new register entries
(HA-83…HA-91), all invented, all labelled.

*Reviewer's independent verification.* The covariance invariants were re-checked against a
**Cholesky decomposition written for the review** — sharing no code with the filter — over
**6 seeds × 3,000 ticks, 54,000 assertions**, driving an adversarial path of hard turns, reversals,
standstills and deliberate 40″ outliers. `P` stayed symmetric and positive-definite throughout, with
minimum Cholesky pivots between 0.09 and 1.33: comfortably positive, not marginal. Separately
confirmed by `git status`: `include/shulib/localization/` contains **one new untracked file and no
modified seam header** — the swap point was filled, not reshaped, exactly as `IFusionPolicy`'s
header promised at M2. And the `< 1°` sentence in chapter 14 has **zero diff lines**, which is the
check that mattered most: a Kalman filter arriving is precisely the moment an accuracy claim
inflates on its own.

*Also fixed at review:* the README led with "1,521,419 assertions" in bold, which flatters. 48 new
test cases produced 430,252 new assertions — the number reflects how many seeds were swept, not how
many independent things are checked. It now carries an explicit caveat pointing at mutation testing
as the measure the project actually trusts, with the reason stated: **a test that cannot fail when
the code is wrong is worse than no test, because it reads as coverage.**

**Next: F1 — Phase F (sequencing).** Phase E's estimator work is done; F1/F2 build the mechanism
abstraction and the time-budgeted sequencer on top of it.
*(Reminder: there is no Phase B — see the note under the phase table.)*

**Verified 2026-08-10 (post-C4):** host suite **592 cases / 915,157 assertions** green under
strict `-Werror` (3 deliberately skipped, unchanged); both CI guards pass (chassis.hpp and
command_pipeline.hpp fall inside existing scopes); all **89** v2 headers cross-compile clean
for ARM (the generated list picked both new headers up automatically). C4's 22 mutations: 20
observed red; **2 GREEN HOLES found and closed with new tests** (M20: teleop `drive()` loops
ran with fault observables dark; M21: the yaw-rate budget is invisible to every closed-loop
test — desaturate keeps mutated turns convergent — so only a truth yaw-rate pin can see it);
plus one process catch (a non-compiling mutation nearly misread as green off a stale binary —
the campaign runner now gates on build success).

**The governing constraint: there is no robot yet, and won't be for a while.**

**Status of the three things nothing had touched:**
1. ~~**No shulib v2 code has ever run on a V5.**~~ **Closed 2026-08-12** — a brain and a battery
   were plugged in, and the package built from `main` (`d4fac9c`) uploaded, booted, constructed
   its whole object graph on ARM, and printed its banner over USB serial, including a live
   `strafeAuthority=1.00` query through the frozen facade. Evidence: the transcript plus the
   sha256 of the exact binary. Both suspected blockers (soft-float firmware/`liblvgl.a`,
   `CXX_STANDARD`) turned out to be already resolved. **What remains honest about it:** the HAL
   was fake-backed, so it drove nothing and could not — the run proves the build/upload/boot path
   and the absence of host-only assumptions, and proves *nothing* about motion, accuracy or
   control. **This is not R3.** R3's clause is a robot that moves, and it is still open, behind
   R1's adapters.
2. ~~**There is no host sim.**~~ **Closed at A2; made HOSTILE at A3.** The plant converts voltage
   into motion behind the unmodified F4 fakes; closed loops converge, diverge, and are measured
   against exact ground truth — and since A3 the sensors LIE the way V5 hardware lies (drift,
   garbage windows, sentinels, sag, slip, latency, jitter), reproducibly from a seed. What remains
   honest about it: it proves **logic, not constants** — every hostile magnitude is provisional
   until R4 measures the real sensors (the A4 register tracks each one).
3. ~~**`make` fails** — quarantined legacy sources still `#include "shulib/util.hpp"`.~~
   **Closed at C7 (2026-08-10):** legacy deleted, `main.cpp` rewired, `make` produces the
   uploadable hot/cold package. What remains honest about it: the package has never run on a
   V5 and cannot drive one until R1's hal/pros adapters exist.

---

## The missing prerequisite (a roadmap incompleteness bug)

`roadmap.md`'s M2 Definition of Done requires *"a hand-written X-drive auton chains profiled motions
and settles within tolerance in **host sim**"* (line 379); M4's DoD requires the same (line 442).
**No task in the roadmap builds that host sim.** `hal/sim` at M6 does not fill the gap — it is a
*bridge to VexBuilder's Rapier engine*, consuming an external physics model rather than providing one,
and it is gated on a tool that hasn't shipped it.

By the roadmap's own rule — *"if something needs doing and isn't on this page, that's a bug in the
roadmap"* — this is a bug, and it is now the **critical path**.

**Why it dominates everything else:** with no robot *and* no plant, closed-loop behavior cannot be
validated anywhere, by any means. Motion, fusion, and sequencing are all closed-loop. Building them
against a plant that doesn't exist means writing them and hoping.

**The reframe:** the simulator is the robot you don't have. And for the estimator work specifically it
is *better* than a robot, because in simulation you possess ground truth — the exact pose the robot
"really" has at every tick — which you never have on a physical field. An EKF verified against known
truth over thousands of synthetic runs is more rigorously proven than one eyeballed against tape
measurements on a field.

**What simulation can never tell you** — this list *is* the Hardware Assumptions Register (Chunk A4):
real friction and wheel scrub, true sensor noise characteristics, motor thermal droop, field surface
variation, PROS call latency, and whether the GPS position-axis→compass binding assumption is correct.
Gains tuned in sim are therefore **provisional**; real tuning happens on hardware. The plant proves
*logic*, not *constants*.

---

## The order at a glance

| Phase | Theme | Chunks | Gate |
|---|---|---|---|
| **A** | Build the ground to stand on | A1–A4 | — |
| **C** | Make it move | C1–C8 | — |
| **D** | Make it usable | D1–D3 | — |
| **E** | Bound the drift (vs. synthetic truth) | E1–E4 | — |
| **F** | Sequencing | F1–F2 | — |
| **G** | No-code authoring | G1–G4 | needs VexBuilder |
| **H** | Ecosystem | H1–H3 | needs VexBuilder sim |
| **R** | **Robot arrival** | R1–R6 | **needs hardware** |
| **F′** | Scoring primitives | F3–F4 | needs hardware + final mechanisms |
| **E′** | Accuracy on the real field | E5–E6 | needs hardware + field |
| **I** | Second robot | I1–I2 | needs both robots |

**40 chunks** (C8, the manual, was added at Phase C). Freezes land at D2 (**F6**), G2 (**F8**), G3 (**F7**), H1 (**F9**).

> **There is no Phase B — deliberately.** An earlier draft's Phase B was the hardware bridge
> (`hal/pros` + on-robot validation, right after Phase A); the reversal recorded in the
> deviations table moved all of it into **Phase R**, and the lettering keeps the gap rather than
> renumbering so that early notes, briefs and commit messages citing phase letters stay true.
> A→C is not a typo; it is the reversal's fossil, kept visible on purpose.

**28 of 40 chunks need no hardware** — and **21 of those (Phases A, C, D, E, F) need nothing external
at all.** That is the great majority of the library, including its hardest and highest-value parts:
the motion layer, the estimator, the sequencer's guaranteed-park guarantee, and the accessibility
layer. Hardware-dependent work is consolidated into Phase R and the tail so that **no host chunk ever
waits on a robot**, and so the day hardware arrives there is a prepared checklist to run rather than an
open-ended exploration.

**Phases R, F′, and E′ are ordered last but authored throughout.** R1–R3 in particular are *written*
early and *run* on arrival — they need hardware to execute, not to author.

---

# Phase A — Build the ground to stand on

> **The substitute for hardware.** Without a robot, these four chunks *are* the validation
> infrastructure for everything that follows. A1 makes the system observable, A2 makes it
> closed-loop-testable at all, A3 makes those tests hostile rather than agreeable, and A4 writes down
> exactly what remains unproven so the debt is visible instead of silent.
>
> **Do not shortcut this phase to "get to the real work."** With no robot, this *is* the ground truth.
> Every chunk in C, D, and E is proven or disproven by what is built here — and an agreeable test
> harness will happily certify a broken stack.

### A1 — `DebugRecord` + `TermSink` + fault discipline
The per-tick snapshot schema (§18.2), the human-readable terminal pretty-printer (§18.3), the numeric
fault-code enum with latched first-fault, loop-overrun detection, and NaN/Inf invariant asserts that
log-and-recover rather than crash (§18.4). Also fixes the three inherited `logger.hpp` bugs
(`escapeJSONString` unapplied, dead `sendDebugMessages`, racing flush) **before** anything builds on it.

`DebugRecord` carries fields for systems that don't exist yet (gating residuals, covariance trace,
strafe-fallback). Define them now and leave them unpopulated — the schema is the thing later chunks
must not be allowed to reshape, and F9 serializes exactly this record at H1.

**DoD:** a synthetic tick stream renders the §18.3 target shape; `TRACE` is provably stripped at
compile time in a competition build; a deliberately injected NaN is caught, logged, and recovered from.

---

### A2 — The host plant + closed-loop sim harness ⟵ **the missing prerequisite**
The chunk the roadmap forgot, and the one everything downstream depends on. A host-side model that
closes the loop: **commanded voltage → wheel dynamics → kinematics → pose → sensor readings → back
into the estimator and controller.** It drives the existing fakes rather than replacing them, so
`FakeMotor` and friends keep their contracts and gain a plant behind them.

**Scope it honestly — kinematic, not dynamic.** Model voltage→velocity through the feedforward
relation, integrate pose through the existing `arcStep`, and synthesize encoder, IMU, and GPS readings
from the true pose. Do **not** pretend to model mass, motor torque curves, or friction coefficients:
those parameters are unmeasurable without a robot, and a plant tuned on invented constants produces
confident, wrong answers. The plant proves **logic**, not **constants**.

**Ground truth is the point.** The harness knows the robot's exact pose at every tick. That is what
makes E2–E4 provable: you can measure estimator error against truth directly, over thousands of runs,
which is something a physical field can never give you.

**Also delivers:** a deterministic scenario runner (seeded, reproducible, replayable) so a failing run
can be re-run exactly, and the three M0 acceptance stubs in `accuracy_spec_test.cpp` finally have a
system to run against.

**DoD:** an open-loop voltage command moves the simulated robot a predictable distance; a closed-loop
`Pid` holds a position against the plant; the plant's forward kinematics round-trip against
`MatrixKinematics`; runs are bit-reproducible from a seed.

### A3 — Hostile fakes
**The highest-leverage no-robot chunk, and currently the biggest silent risk in the project.** Today's
fakes are *agreeable*: they encode the same assumptions the production code does, so a test passes
whenever the code agrees with itself. That is a suite that certifies its own blind spots.

Upgrade the fakes and the plant to model how V5 hardware actually misbehaves:

- `PROS_ERR_F` / error sentinel returns from any sensor read
- Sensor dropout and mid-run disconnection
- IMU drift, per-boot bias, and noise; GPS no-fix, bad-fix, and off-strip
- Encoder quantization and tracking-wheel slip
- Motor voltage saturation, current limiting, and thermal droop
- Loop jitter, variable `dt`, and sensor latency
- Battery sag to brownout

**Why this matters more than any single feature:** every surprise the robot could deliver on day one
is a surprise you can deliver on purpose today. The stack that survives hostile fakes has a far
smaller gap to close when hardware arrives.

**DoD:** each degradation is independently injectable; the existing stack is run against all of them
and every resulting failure is either fixed or recorded in A4; a fault code is raised (not a crash)
for every sensor pathology.

### A4 — Hardware Assumptions Register + ARM compile gate
**The register** — a living document listing every claim about physical hardware that cannot yet be
checked, each written as a falsifiable statement paired with the exact test that will check it. Seeds
include: the GPS position-axis→compass binding (already flagged "validate-on-field" in
`gps_conversion.hpp`, with a skipped oracle at `test/gps_conversion_test.cpp:163`), IMU sign and wrap
conventions, tracking-wheel offsets and directions, motor cartridge and gearing ratios, and PROS call
latency.

This converts integration debt from invisible to **inventoried**. It becomes Phase R's checklist, so
first contact with hardware is a prepared sequence rather than an open-ended exploration.

**The ARM compile gate** — the v2 core is verified to cross-compile clean under strict flags, but
nothing *keeps* it that way: CI builds host only. Add a translation unit including every v2 header,
compiled by `arm-none-eabi-g++` in CI, so a host-only assumption can never enter the core unnoticed.
The baseline is green today; this is purely about holding it.

**DoD:** the register exists with every current assumption entered and a named test for each; CI fails
if the core stops cross-compiling.

---

# Phase C — Make it move

> Fully host-testable, because A2 built the plant these chunks close the loop against. **Every
> "settles in host sim" DoD below is achievable only because A2 exists** — that is why it comes first.
>
> **Gains tuned here are provisional.** The plant proves control *logic* — that the loop converges,
> that exits fire, that nothing hangs or diverges. Real gain values are established on hardware at R5.
> Do not treat sim-tuned constants as final, and do not spend effort polishing them.

### C1 — `IMotion` + the motion primitives
`IMotion`, then `MoveToPose` (decoupled per-axis x/y/θ — the holonomic advantage over LemLib's
tank-only coupling), `TurnTo`, `StrafeTo`, `driveBrake`, `holdPose`. Every motion reports an
`ExitReason` (the `ExitGroup` plumbing from M2 already exists) and emits a per-motion result line
through `TermSink`.

**DoD:** each primitive drives the A2 plant to its target and settles within tolerance;
mutation-checked; the run is legible on the terminal as it executes; each primitive is also run
against the A3 hostile fakes and degrades to a fault code rather than diverging.

### C2 — `MotionScheduler`
One active motion at a time; `async()` / `waitUntilSettled()` / `waitUntil(pred)` / `cancel()`. Every
motion is watchdog-bounded — a motion can never hang.

**DoD:** a chained sequence of motions runs to completion in host sim; `cancel()` mid-motion leaves
the drivetrain in a defined safe state; a deliberately stalled motion is killed by the watchdog.

### C3 — `HDriveKinematics` + the pseudo-inverse
Capped strafe authority with automatic turn-then-drive fallback (telemetry-visible via
`strafeFallbackActive`). Carries the M1 deferral: generalize `MatrixKinematics::forward()` to the full
`(AᵀA)⁻¹Aᵀ` pseudo-inverse for the H-drive's non-orthogonal off-center strafe wheel. That relaxes a
precondition only, so it is F5-safe.

**Why here:** the same motion code must drive both robots. Discovering that the H-drive needs a
different motion contract *after* the facade froze would be an F6 break.

**DoD:** the H-drive runs the C1 primitives unmodified in host sim; strafe-authority capping and the
fallback are both exercised; the pseudo-inverse is verified against the orthogonal case.

### C4 — `Chassis` facade (built, **not** frozen)
The public verbs: `moveTo` / `strafeTo` / `turnTo` / `followTrajectory` / `drive(ChassisSpeeds, Frame)`.

**Explicitly not frozen here.** F6 is the contract every auton ever written depends on; it freezes at
D2, after a second independent consumer has exercised it.

**DoD:** a hand-written X-drive auton chains profiled motions through the facade and settles.

### C5 — Per-motion results + session header
Completes WS13 for M2: the per-motion result line (target vs final · overshoot · drift · time ·
exit-reason), the end-of-run summary block, and the **session header** (git build hash, routine id,
alliance/side, port map, battery start) as the first record of every run — so runs are reproducible
and you can always confirm which binary actually ran.

**DoD:** a full host-sim run produces the §18.3 output end to end, header through summary.

### C6 — Legacy salvage
Port `RobotCommands` → the `sequence/` seed and `logger.hpp` → `io/Telemetry`. The Pilons arc math was
already re-derived cleanly at M2 (`arcStep`), which is the model: re-derive, don't copy.

**DoD:** everything worth keeping is in the new tree with tests, verified independently of `legacy/`.

### C7 — Cutover and deletion
Rewire `main.cpp` onto the new core so the ARM build compiles again, then **delete `src/legacy/` and
`include/legacy/`.** Broaden the CI PROS-free guard to all of `shulib/`.

**Deleting before hardware validation is deliberate.** The roadmap sequenced deletion *after* an
on-robot run, as a safety net. That net isn't real: `legacy/` no longer compiles (it's why `make`
fails today), C6 has already salvaged everything worth keeping, and git retains the history
permanently. Keeping broken, uncompilable code in the tree to hedge against a robot that doesn't exist
yet just guarantees two trees to reason about for months.

**DoD:** `make` succeeds and produces an uploadable package; `legacy/` is gone; the new tree is the
only tree; the CI guard covers all of `shulib/`.

> M2's *structural* DoD closes here. Its **on-robot** clause — "`main.cpp` runs entirely on the new
> core," validated on a V5 — closes at R3.

---

# Phase D — Make it usable

> **Why here, ~40 items ahead of the roadmap's M7 placement.** Two reasons, and the first is a
> correctness argument, not a convenience one.
>
> **1. The recipe API is the second consumer that validates F6 before it freezes.** A facade exercised
> only by its own author has been tested against the intent that produced it. If the recipe layer
> reveals the facade is awkward to build on, that must be discovered *before* the freeze — not after
> Phases E through I have built on it.
>
> **2. Tier 2 is the surface your own members write against.** Students must author and defend their
> competition code. Shipping the readable API at M7 means the entire season gets authored against the
> raw Tier 3 API instead. It is a thin fluent wrapper over a facade that exists at C4 — there is no
> dependency justifying its placement at the end.

### D1 — Tier 2 recipe API
`chassis.moveTo(p).then(intake.in)…` — fluent, ~10 readable lines for a working routine, hard to
misuse. A strict superset of nothing and a strict subset of Tier 3: anything expressible in recipes
must remain expressible in the full API, with no cliff between them.

**DoD:** a complete routine in ~10 lines runs in host sim; every recipe verb maps onto a facade call
with no capability lost; misuse (unit mismatch, unreachable pose) fails at compile time or is rejected
loudly.

### D2 — **Freeze F6**
Only now, with the facade exercised by a hand-written auton (C4), the recipe layer (D1), and a real
on-robot run (C7). Register the freeze; from here it changes only by version bump plus migration.

**DoD:** F6 marked LOCKED in the Freeze Register with all three consumers cited as evidence.

### D3 — Recipe cookbook + generated API docs
The cookbook and the generated API reference published to the team website. **Deliberately excludes
the "first auton in 10 minutes" guide** — that flow starts in VexBuilder and cannot honestly be
written until G4.

**DoD:** docs generate and publish; a member who has not seen the code can write a working routine
from the cookbook alone.

---

# Phase E — Bound the drift (against synthetic truth)

> M3's host-provable half. The accuracy edge, and the part that makes the award narrative true. The
> `Localizer`'s `IPoseSource` / `ICorrector` / `IFusionPolicy` seam already exists and was built
> EKF-ready — these chunks fill it in rather than reshaping it.
>
> **This phase is where having no robot is genuinely an advantage.** Every corrector and the EKF are
> *pure math over sensor streams*. Against the A2 plant you possess exact ground truth at every tick,
> so estimator error is measured directly rather than inferred from tape measurements on a field. You
> can run thousands of seeded trajectories, sweep noise parameters, and prove convergence properties
> that a physical field could never demonstrate.
>
> **What this phase cannot establish:** real sensor noise characteristics, true GPS update latency, and
> whether the field's GPS strip behaves as modeled. Those are R-phase work, and every assumption made
> here about them goes into the A4 register. The structure and correctness of the estimator are proven
> here; its noise parameters are fitted on hardware.

### E1 — `SdSink` + estimator introspection
Ordered first in the phase for the same reason A1 was ordered first overall: fusion is the hardest
thing in the project to debug, and the **no-laptop field record** is what makes a field run
diagnosable at all. Binary blackbox to `/usd/` (versioned header, session/provenance record with git
hash, fixed-width per-tick, double-buffered off-task writes, byte budget with drop-to-counter
back-pressure, flush on auton-end). Plus per-correction residual, Mahalanobis distance, accept/reject
reason, and covariance trace — **the quantities that certify `< 1.0°`, rather than asserting it.**
Latched brownout marker with the graceful-end contract.

**DoD:** a full run writes a recoverable blackbox; every gating decision is reconstructable after the
fact from the file alone.

### E2 — `GpsCorrector`
Adaptive R from `get_error()`, lever-arm and latency compensation, Mahalanobis gating, high-yaw-rate
rejection, and the **off-strip dead-reckon-only flag** — Driving Skills has no GPS strip, so this must
degrade correctly rather than trust garbage.

**DoD:** against modeled GPS noise on the A2 plant, the corrector reduces pose error versus
dead-reckoning alone and never increases it; a deliberately bad fix is gated out; off-strip degrades to
dead-reckon-only rather than trusting garbage; every gating decision is logged.

### E3 — `AprilTagCorrector`
Tags 0–4, PnP, feeding the **gated nudge** — low-R, fast, drift-cancelling, and **never a hard pose
reset** (§13 #4). The M2 red-team already caught the failure mode here (corrections not accumulating);
the seam retains nudges and converges.

**DoD:** tag-driven correction measurably reduces heading drift against known truth over a simulated
60s run; no snap is ever observable in the record; PnP is verified against synthetically-projected tags
of known pose.

### E4 — Complementary filter → 5-state SE(2) EKF
`[px, py, θ, vx, vy]` with Mahalanobis gating, consecutive-reject re-init, and process noise ∝ travel.
Swapped in behind `IFusionPolicy` — the seam was built for exactly this.

**Why after E2/E3, not before:** the EKF's job is to weigh the correctors against each other, so it
needs them to exist. The tiering decision (§3, §10) is deliberate — the simpler filter is easier to get
right *and* to explain, so it stays as the fallback.

**Ground truth makes this provable.** Generate a known trajectory, corrupt the synthesized sensor
streams with modeled noise, and verify the EKF recovers the true pose within bounds — swept across
noise levels, seeds, and trajectories. Verify the properties that matter structurally: covariance
stays positive-definite, gating rejects outliers without diverging, and consecutive-reject re-init
recovers from a hijacked estimate.

**Its noise parameters remain provisional until R4** — modeled noise is a guess about hardware. The
*structure* is proven here; the *numbers* are fitted on the robot.

**DoD:** the EKF beats the complementary filter on identical seeded runs; the swap changes nothing
above the seam; the structural invariants above hold across a parameter sweep.

> **RULED AT E4, and written here so nobody re-derives it.**
>
> **(1) "Consecutive-reject re-init" vs §13 #4 "never snap".** This entry and decision #4 could not
> both hold as written, and the conflict sat here unnoticed from the day the roadmap was drafted.
> The resolution is to read "re-init" as re-initialising the belief's **uncertainty**, not its
> **value**: on the trigger the filter resets its covariance and does not move the estimate at all.
> Never-snap then holds bit-for-bit, and the estimator still recovers — a large `P` opens the gate,
> and the following fixes walk the estimate home at the ordinary bounded rate. The event is
> declared (`GateReason::CovarianceReinit`), latched, rate-limited, and requires N consecutive
> rejections *and* a persistently large innovation. Rejected: teleporting onto the rejected fix,
> which is the snap #4 forbids and which trusts a fix the filter has just spent fifty ticks
> distrusting.
>
> **(2) "The EKF beats the complementary filter on identical seeded runs" was NOT achieved, and
> was not tuned toward.** Measured over 8 seeded 60-second hostile runs: mean final error 0.351″
> (EKF) against 0.225″ (complementary), losing on 7 of 8 seeds. The reason is E2's, one layer up —
> dead-reckoning in this simulation is already sub-inch, so the modelled GPS is noisier than the
> drift it corrects and the best move is mostly to ignore it. **The DoD line was written before
> anyone knew that**, and the honest response is to record it rather than fit the filter to the
> test. What the EKF does deliver is capability the other tier does not have at all: σ-weighted
> arbitration, recovery from a displacement past the 12-inch ceiling, and a stated uncertainty.
> The complementary tier therefore stays the shipped default. See `chunks/E4-COMPLETED.md` §T2/§T3.

---

# Phase F — Sequencing

> M4's host-provable half. The **concrete** scoring primitives need both hardware and the build team's
> final mechanism decisions, so they defer to Phase F′. The *seam* and the *engine* do not — and the
> engine happens to contain the single highest-value guarantee in the library.

### F1 — `Mechanism` HAL abstraction + fakes
The seam every scoring primitive sits behind, plus deterministic fakes for it (and hostile variants,
per A3: a jammed intake, an unconfirmed grab, a stalled lift). Defining this before any concrete
mechanism exists is deliberate — the abstraction should be shaped by what sequencing needs, not
retrofitted around whatever hardware happens to get built.

**DoD:** a fake mechanism is driven through the seam; failure modes surface as fault codes, not hangs.

### F2 — `sequence/` engine + time-budgeted sequencer
`Sequence` / `Parallel` / `Race` / `Deadline`, possession-aware time budgeting, and the
**guaranteed end-of-run action**: the +8 Midfield park and final Toggle re-verify fire on a hard
schedule *regardless of where the loop stalled*. §14 calls this non-negotiable, and it is plausibly
the highest-expected-value code in the library — it converts a failed run into 8 points.

**Fully host-provable, and worth doing early.** A stalled loop, a mechanism that never confirms, and a
motion that times out are all injectable via A3. The park guard's correctness is a scheduling property,
not a hardware one.

**DoD:** a deliberately stalled scoring loop still ends parked, verified against the plant with the
clock driven to the match limit. **This test is the entire point of the chunk.**

---

# Phase G — No-code authoring

> M5 — the accessibility pillar's Tiers 0 and 1. **Sequenced so that VexBuilder gates as little as
> possible.** shulib defines the contracts; VexBuilder implements them (§16.0). The in-memory types
> and the runtime are therefore built and tested against synthetic fixtures *first*, and file
> ingestion — the only genuinely externally-gated piece — comes last.

### G1 — `IRobotConfig` / `IRouteSource` + `RobotBuilder`
The in-memory `RobotConfig` and `Route` types and `RobotBuilder.from(profile)` → a fully wired
`Chassis`. Built against hand-written fixtures, no file parsing involved.

**This preserves the standalone promise:** a code-fluent team builds `RobotConfig` and `Route`
directly in C++ with no `.vexbot` at all. The file is the on-ramp, never a dependency.

**DoD:** a hand-built config produces a working `Chassis` in two lines; zero VexBuilder dependency.

### G2 — `PathRunner` + command-id registry → **freeze F8**
Profiled per-segment execution with marker callbacks, and `runner.on("intake_in", fn)` — the keystone
of the no-code story. The auton is **data**; the handlers are code registered once. Legacy's embedded
`code_template` C++ snippets are rejected outright: that data/code coupling is the thing this design
exists to eliminate. Unknown ids log a WARN fault and are skipped, never crash. Markers carry optional
typed args so parametric primitives don't explode into one id per value.

shulib **owns** the command-id vocabulary and exports it as a manifest for VexBuilder's picker — so
this is authored unilaterally, then handed across.

**DoD:** a synthetic `Route` executes end to end with markers firing; an unknown id degrades safely;
the manifest is exported. **F8 frozen.**

### G3 — `.vexbot` ingestion + codegen → **freeze F7**
The `robotProfile` sub-schema, the `.vexbot` → `robot_config.hpp` codegen tool (primary, on-robot: no
runtime JSON parse, no SD dependency, compile-checked), the optional SD-card runtime loader,
`inferDrivetrain()` as the documented-brittle fallback, and `schemaVersion` negotiation with additive
migration — so a `.vexbot` made next year still loads.

**The known external gap** (§16.1, cross-team asks #1 and #2): the shipped `.vexbot` v2.0.0 does not
carry `project.paths[]`, and its `electrical{}` arrays are empty because the UI was never built.
`inferDrivetrain()` covers the fallback path, but **explicit drivetrain fields are the contract.**

**DoD:** a real `.vexbot` produces a working robot and routine; a deliberately-newer schemaVersion
loads without fatality. **F7 frozen.**

### G4 — Legacy `.shupaths` importer + the 10-minute guide ⟵ **closes M5**
One-way importer mapping old `code_template` strings to command ids best-effort, flagging what it
can't. Then the "first auton in 10 minutes" guide — build → export → drag a path → run — which can
finally be written truthfully because the flow now exists.

**DoD:** a team member who cannot code builds a robot and a routine in VexBuilder, exports one
`.vexbot`, and the robot runs it. **This is the accessibility pillar delivered.**

---

# Phase H — Ecosystem

> M6. Gated on VexBuilder's Rapier sim, but H1 is not — the wire is defined unilaterally.

### H1 — `SHUL/2` wire protocol → **freeze F9**
The versioned, sequenced wire serialization of the `DebugRecord` defined back in A1, behind the same
`ITelemetrySink` seam. Defining `DebugRecord` once in chunk one and serializing it here is exactly why
A1 came first: one schema, four sinks, directly comparable bench / terminal / field / sim traces.

**DoD:** round-trip serialization verified; version negotiation handles an unknown future version.
**F9 frozen.**

### H2 — `hal/sim` adapter + record/replay
The sim adapter speaking `SHUL/2` over VexBuilder's agent socket (`server.json` discovery),
bidirectional — simulated sensors in, pose/twist/wheel-cmd/markers out — plus run record/replay and
the planned-vs-actual overlay contract.

Because the core depends only on the PROS-free HAL, **"works in the simulator" is a structural
guarantee, not a feature** — the same code already runs against `hal/pros`, `hal/fake`, and now
`hal/sim` with zero changes.

**DoD:** a real on-robot run replays in VexBuilder.

### H3 — On-brain live tuner ⟵ **closes M6**
Live PID/FF tuning on the brain mid-session, no laptop.

---

# Phase R — Robot arrival

> **Everything above is written to make this phase short and prepared.** By the time hardware exists,
> the library is complete, the A4 register lists exactly what is unproven, and R3's validation routine
> is already written and waiting. First contact is a checklist, not an exploration.
>
> **Write R1–R3's code before the robot arrives** — they need hardware to *run*, not to *author*. Slot
> the authoring in whenever convenient during Phases C–H; only the validation sessions block.
>
> **R1–R3 close M1's Definition of Done**, open since June: *identical numbers in a host test and on
> the V5, swapping only `RobotContext`.*

### R1 — `hal/pros/*` adapters
The **only** files in the tree permitted to `#include <pros/*>`. Adapters for `IClock`, `IMotor`,
`IRotation`, `IImu`, `IGps`, `IDistance`, `IOptical`, `IBattery`, `ITelemetrySink`. The already-built
and red-teamed `imu_conversion.hpp` / `gps_conversion.hpp` pure functions wire in here — the
conversions happen once, at the boundary.

**DoD:** every F4 interface has a PROS-backed implementation; the CI guard still passes; the ARM build
compiles them.

### R2 — AI Vision adapter
Vendor and extend the existing `ai_vision.hpp` wrapper into the `IVision`/`ITagSource` adapter, in
both object mode and AprilTag mode (kernel 4.2.2 provides the 4 tag families natively).

**DoD:** tags surface as robot-relative poses and objects as bearings, per F4 decision #7.

### R3 — Day-one validation ⟵ **closes M1 and M2's on-robot clause**
A minimal validation entry point — no auton, no motion. Read every sensor, command open-loop voltages,
stream `TermSink`. Then walk the **A4 Hardware Assumptions Register top to bottom**:

- **The GPS field-cal axis oracle** (`test/gps_conversion_test.cpp:163`) — bench-measure the
  position-axis→compass binding rather than assuming it. Unskip it.
- **The F5 on-V5 number-match** — the same twist produces identical wheel commands on host and robot.
- **IMU conversion truth** — canonical heading, sign, and wrap against physical rotation.
- **Tracking-wheel geometry** — measured offsets and direction signs against `PilonsOdometry`.
- **A push test** — shove the robot a measured distance; confirm odometry agrees.
- **Every remaining register entry**, marked confirmed or corrected.

**Expect corrections here, and treat them as the system working.** Each one is a defect the register
predicted and localized to the HAL seam, rather than a mystery surfacing mid-season. A correction
behind `hal/pros` does not touch the core.

**DoD:** M1's DoD met and its badge flips to ✅; every skipped hardware oracle unskipped and green;
the register fully resolved; a v2 auton runs on the robot.

### R4 — Sensor characterization → real noise parameters
Measure what only hardware can tell you: IMU per-boot bias and 60s drift rate, GPS update latency and
error distribution on and off the strip, encoder noise, and tracking-wheel slip under acceleration.
Feed these back as the EKF's noise parameters, replacing E4's modeled guesses. Add boot-time IMU cold
calibration and pitch/roll tip detection.

**Heading quality is the ceiling on F2's `< 1.0°` target** — this chunk establishes what that ceiling
actually is on your hardware.

**DoD:** measured noise parameters committed; the E4 EKF re-verified against them; documented 60s drift.

### R5 — `tools/sysid` + real gains
Offline least-squares characterization producing kS/kV/kA **constants, not code** (§10), fed by one
on-robot ramp routine. The solver and the ramp are host-authored and host-tested against synthetic
data long before this; here they meet a real robot. Then re-tune the C-phase controllers against
measured dynamics, replacing the provisional sim-tuned gains.

**DoD:** real kS/kV/kA for both robots committed as constants; predicted-vs-actual velocity error
documented; motion re-verified on hardware.

### R6 — Sim fidelity back-fit
Feed the measured parameters from R4 and R5 back into the A2 plant, then re-run the entire host suite
against the improved model. **This closes the loop that makes the whole no-robot strategy compound:**
every future chunk is developed against a plant calibrated to the real robot, and any test that
newly fails is a real defect the invented parameters had been hiding.

**DoD:** the plant reproduces a recorded real run within a documented tolerance; the full host suite is
green against the calibrated plant, with every new failure fixed rather than tolerated.

---

# Phase F′ — Scoring primitives

> Gated on hardware **and** the build team's final mechanism, lift, and role decisions — hardware
> calls, not software ones. **Ordered by the master plan's own value ranking** (§14).

### F3 — The scoring primitives
In §14's value order: `setQuadrantToggle` (index N clicks on the 3-state Toggle, Optical color confirm
— *highest value*, it is what converts yellow Pins from 0 to 10 points), `orientToScoringHalf`
(*second highest*), `intakeUntilCapture`, `liftToLevel`, `rotateClampToAngle`, `clampActuate` /
`clampConfirm`, `deployActuator` — each behind the F1 seam.

**Non-negotiable across all of them:** task-sensor confirmation on **every** grab and place — never
advance on an unconfirmed action.

### F4 — Skills motion + reference routines ⟵ **closes M4**
`fieldCentricStrafe` / `strafeTrim` (H-bot), `moveToPoseProfiled` (lift-state-aware acceleration),
`buildStack`, `matchLoadCycle`, `endInMidfield` (18″ height lockout), `strategyMode(tallTower|fastCycle)`.

**Scope boundary:** this chunk delivers primitives and the engine. The season's actual routines are
strategy authored by students on top of them.

---

# Phase E′ — Accuracy on the real field

### E5 — Calibration routines + persistence
Wheel scale and offset, GPS lever-arm, camera mount, IMU bias — measured by documented routines and
saved to SD. This is what makes the accuracy reproducible after a rebuild rather than a one-time
achievement.

**DoD:** a documented procedure any team member can run; calibration survives a power cycle.

### E6 — `alignment/DockToGoal` ⟵ **closes M3**
Visual-servo docking (AprilTag / poly-cutout) with current, distance, and pose confirmation, plus a
height-adaptive and no-tag Distance-sensor fallback. The Innovate headline: **holonomic visual-servo
docking that strafes to align** — structurally impossible on a differential drive.

**DoD:** F2 met — pose error bounded across a full 60s run with contact and spins, heading `< 1.0°`;
docking nests a 1.6″ Pin repeatably.

---

# Phase I — Second robot

### I1 — H-drive hardening end-to-end
Validation, not new core — the H-bot rides the same stack. C3 built the kinematics; this proves the
whole pipeline on the physical 15″ robot.

### I2 — Coordination seam *(stretch)*
A thin interface for the two robots to share intent. Running shulib on both bots is just two
independent instances, and *that* is already core — this chunk is only the optional layer on top.

---

## Deviations from the roadmap's milestone order

Recorded so the reordering is auditable. **Three are additions the roadmap was missing; five are
reorderings.**

| Change | Roadmap says | Build order says | Why |
|---|---|---|---|
| **+ Host plant & sim harness** | *(absent — the incompleteness bug)* | **A2, before all closed-loop work** | M2/M4 DoDs both require a "host sim" nothing builds. With no robot it is the only way to validate any closed loop |
| **+ Hostile fakes** | *(absent)* | **A3** | Today's fakes encode the same assumptions as the code, so tests certify their own blind spots |
| **+ Hardware Assumptions Register & ARM CI gate** | *(absent)* | **A4** | Makes integration debt inventoried instead of silent; becomes R3's checklist |
| **Diagnostics first** | WS13 within M2 | A1, before everything | Every later chunk is debugged through it |
| **All hardware work deferred** | `hal/pros` + cutover at end of M2 | **Phase R, last** | See the reversal note below |
| **sysid after motion, not before** | Within M2's WS4 | R5 | Real constants need a robot; sim-tuned gains are explicitly provisional until then |
| **Recipe API + F6 freeze** | M7, after M3–M6 | D1–D2, right after the facade | Second consumer validates F6 *before* the freeze; Tier 2 is the surface students author against all season |
| **Sequence engine before scoring primitives** | Both within M4 | F2 host-side; F3 deferred to F′ | The guaranteed-park guarantee is a scheduling property, provable without hardware or final mechanism designs |
| **Legacy deleted before on-robot validation** | Delete after hardware-validating | C7 | The "safety net" doesn't compile, C6 salvages first, and git keeps history |

> **Reversal, recorded honestly.** An earlier draft of this document put the hardware bridge *before*
> the motion layer, arguing that validating the HAL seam early keeps the blast radius of a conversion
> error small. That argument depended on a robot existing to validate against. With no hardware, the
> work can be *authored* but not *verified*, so front-loading it buys nothing and delays everything
> provable. The correct response to the constraint is not to reorder toward hardware — it is to build
> the substitute (A2/A3) and inventory what stays unproven (A4).

None of these change *what* gets built or any frozen contract. The three additions close a real gap;
the five reorderings each buy a correctness guarantee.

---

## Open external dependencies

Tracked here so they are never invisible. **None block Phases A, C, D, E, or F — 20 chunks.**

| # | Dependency | Owner | Blocks |
|---|---|---|---|
| 1 | A physical robot with a V5 Brain | Team | Phases R, F′, E′, I |
| 2 | `project.paths[]` in the `.vexbot` schema | VexBuilder | G3, G4 |
| 3 | Explicit drivetrain fields (`kind`/`trackWidth`/`wheelDiameter`) | VexBuilder | G3 (fallback exists, brittle) |
| 4 | Agent socket exposed for `SHUL/2` + Rapier sim | VexBuilder | H2 |
| 5 | Command-id manifest consumed by the picker | VexBuilder | G4 polish |
| 6 | Final mechanism / lift / role decisions | Build team | F3, F4 |

**The robot is the only dependency on the critical path**, and it gates the *tail* rather than the
trunk. Phases G and H are VexBuilder-gated but independently sequenced, so if VexBuilder stalls, the
library still completes through Phase F.

---

*Companion to [`roadmap.md`](../roadmap.md) (what) and the [master plan](../shulib-v2-master-plan.md) (why).
Created 2026-08-01. When a chunk closes, this file's status and `roadmap.md`'s "you are here" move together.*
