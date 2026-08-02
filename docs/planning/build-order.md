# shulib v2 — Build Order

> **What this is.** The [roadmap](roadmap.md) says *what* is left, grouped by milestone. This document
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

Every chunk keeps a **live progress log** at `docs/planning/chunks/<CHUNK>-PROGRESS.md`, appended to
as the work happens — not written at the end. Watch it from a second terminal:

```sh
tail -f docs/planning/chunks/A2-PROGRESS.md      # substitute the current chunk
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

**Done:** M0 complete · M1 host-side (F4 + F5 frozen host-only) · M2 control layer (WS4) · M2
localization tier 1 (WS5) · **Chunk A1 — `DebugRecord` + `TermSink` + fault discipline (WS13)**,
closed 2026-08-01 ([completion record](chunks/A1-COMPLETED.md)) · **Chunk A2 — the host plant +
closed-loop sim harness**, closed 2026-08-01 ([completion record](chunks/A2-COMPLETED.md)) ·
**Chunk A3 — hostile fakes**, closed 2026-08-02 with its documentation contract discharged
([completion record](chunks/A3-COMPLETED.md)) — **the seams are populated and the suite is no
longer agreeable**: every V5 misbehaviour class is injectable and composable, hostility found and
fixed three real Localizer defects, and the M2 `<1°` acceptance test is live with a measured
number. **Next: chunk A4** (A3 queued ~25 provisional magnitudes as the register's seed content).

**Verified 2026-08-02 (post-A3):** host suite **429 cases / 681,086 assertions** green under strict
`-Werror` (3 deliberately skipped: two M3 acceptance stubs + the R3 GPS field-cal oracle — the M2
stub is now LIVE); CI PROS-free guard passes (scope includes `include/shulib/diag` and all of
`include/shulib/sim` incl. `sim/hostile/`) plus the **sim-layering guard** (core may never include
`shulib/sim/`); **the v2 core cross-compiles clean for ARM** — all 77 v2 headers under the same
strict flags as host (verified directly — but not guarded by CI, which builds host only; A4 adds
the gate).

**The governing constraint: there is no robot yet, and won't be for a while.**

**Status of the three things nothing had touched:**
1. **No shulib v2 code has ever run on a V5**, and none can until hardware exists. *(Still true.)*
2. ~~**There is no host sim.**~~ **Closed at A2; made HOSTILE at A3.** The plant converts voltage
   into motion behind the unmodified F4 fakes; closed loops converge, diverge, and are measured
   against exact ground truth — and since A3 the sensors LIE the way V5 hardware lies (drift,
   garbage windows, sentinels, sag, slip, latency, jitter), reproducibly from a seed. What remains
   honest about it: it proves **logic, not constants** — every hostile magnitude is provisional
   until R4 measures the real sensors (the A4 register tracks each one).
3. **`make` fails** — quarantined legacy sources still `#include "shulib/util.hpp"`. Expected, and
   resolved in Chunk C7. *(Still true.)*

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
| **C** | Make it move | C1–C7 | — |
| **D** | Make it usable | D1–D3 | — |
| **E** | Bound the drift (vs. synthetic truth) | E1–E4 | — |
| **F** | Sequencing | F1–F2 | — |
| **G** | No-code authoring | G1–G4 | needs VexBuilder |
| **H** | Ecosystem | H1–H3 | needs VexBuilder sim |
| **R** | **Robot arrival** | R1–R6 | **needs hardware** |
| **F′** | Scoring primitives | F3–F4 | needs hardware + final mechanisms |
| **E′** | Accuracy on the real field | E5–E6 | needs hardware + field |
| **I** | Second robot | I1–I2 | needs both robots |

**39 chunks.** Freezes land at D2 (**F6**), G2 (**F8**), G3 (**F7**), H1 (**F9**).

**27 of 39 chunks need no hardware** — and **20 of those (Phases A, C, D, E, F) need nothing external
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

*Companion to [`roadmap.md`](roadmap.md) (what) and the [master plan](shulib-v2-master-plan.md) (why).
Created 2026-08-01. When a chunk closes, this file's status and `roadmap.md`'s "you are here" move together.*
