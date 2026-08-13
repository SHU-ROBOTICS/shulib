# shulib Roadmap

> **shulib** is an open, holonomic-native autonomous stack for VEX U robots — built to be more
> accurate than LemLib, cleaner than OkapiLib, and usable by a team that can't write a line of C++.

This is the in-depth, public roadmap. It is written to live on the team website and to stay **true
for the life of the project**. For the engineering rationale behind any item, see the
[master plan](shulib-v2-master-plan.md).

---

## Why this roadmap won't go stale

A roadmap goes false in three ways: work gets *discovered late* (incompleteness), items get
*reordered*, or things get *renamed*. We design against all three:

1. **Permanent workstreams.** The *what* lives in 12 workstreams (§ Workstreams) that will never be
   renamed or removed. New work slots **into** a workstream — it doesn't rewrite the roadmap.
2. **Milestones are fill-in, not rewrite.** The *when* lives in milestones M0–M8. A milestone is
   "done" only when its **Definition of Done** (a real, testable bar) is met. We move the badge, not
   the structure.
3. **A Freeze Register.** The handful of contracts that would *break* downstream work if they changed
   are listed in the [Freeze Register](#freeze-register). Once frozen, a contract changes **only via a
   version bump + migration — never a silent break.** That is the core promise that keeps everything
   below true.
4. **Frontier is expected, not a surprise.** Stretch work is bucketed as **○ Frontier** from day one,
   so shipping it later is a planned *unlock*, not a roadmap change.

If something needs doing and isn't on this page, that's a bug in the roadmap — tell the programming
chair and it gets added to a workstream.

---

## What shulib is

Most VEX motion libraries do one thing: drive a tank robot from A to B. shulib is the **whole
autonomous stack** for *holonomic* robots (X-drive, H-drive) — the part that turns wheels, the part
that knows where the robot is, and the part that decides what to do next — wrapped so that:

- **Anyone can use it.** Build a robot in our designer, drag a path, press run — no C++ required.
- **It always knows where it is.** It fuses wheel tracking, the inertial sensor, the GPS, and
  AprilTags into one drift-resistant estimate, holding **< 1° of heading error** across a full run.
- **It's provably correct.** The core runs off-robot in automated tests and inside our simulator, so
  bugs are caught before they cost a match.

shulib is one half of a two-tool ecosystem:

```text
   ┌──────────────────────────┐   .vexbot  (robot + paths)   ┌──────────────────────────┐
   │        VexBuilder        │ ───────────────────────────► │          shulib          │
   │  design robot · plan     │      (one project file)      │   run auton · localize   │
   │  paths · (sim, planned)  │ ◄─────────────────────────── │  (the library — works    │
   └──────────────────────────┘      SHUL/2 telemetry        │   standalone, no file)   │
        (visualize the robot)                                └──────────────────────────┘
```

One file carries the whole robot — its wiring *and* its autonomous routines — so a routine can never
drift out of sync with the robot it was drawn for. Each tool stands alone; together they're a single
pipeline from **CAD → code → competition**.

---

## How to read this roadmap

| Badge | Meaning |
|---|---|
| ✅ **Done** | Shipped, with its Definition of Done met |
| 🔨 **Building** | Actively in progress |
| 🎯 **Next** | Committed; the next milestone(s) up |
| 🔭 **Frontier** | On the map by design; not started, and shipping it later is an *unlock*, not a change |

**Maturity tiers** (from the [Capability Catalog](shulib-v2-master-plan.md#15-the-one-stop-shop-capability-catalog-past--present--future)):
● **Core** (proven subset, ships first) · ◐ **Tiered** (clearly reachable) · ○ **Frontier** (stretch).

We are **pre-season with a ~2–3 month dedicated build window**, so the early milestones are
deliberately foundational — we are building the thing the bug-prone old code structurally couldn't be.

### Tracking what's actually done (status discipline)

This page is only useful if it is **truthful**. Three rules keep it that way:

1. **Evidence, not vibes.** A task flips to ✅ / `[x]` **only** when its Definition of Done is backed
   by real evidence — a named test passing in CI, or a present-and-reviewed artifact. Never "done"
   from memory or intention.
2. **Three task states.** `[ ]` not started · `[~]` in progress (work exists but DoD **not** met) ·
   `[x]` done **and verified**. Half-built stays `[~]`.
3. **"You are here" is always current.** One pointer (below) marks the true frontier and is updated
   every working session, so "where are we?" has a one-glance answer. The milestone badges
   (🔨 / 🎯 / 🔭) reflect the same truth at coarser grain.

**We under-claim before we over-claim.** If there's any doubt a thing is finished, it isn't marked
finished. When status changes, the change cites its evidence.

### Testing discipline (tests that try to *break* the code)

Tests exist to **find bugs, not confirm the obvious.** A green `1+1==2` teaches nothing. The bar:

- **Every test targets a specific way the logic could be wrong** — an edge case, boundary, sign flip,
  wrap-around, off-by-one, NaN, precision loss. If we can't name the bug a test would catch, it isn't
  pulling its weight.
- **Invariants/properties over hand-picked points** — assert what must hold across the *whole* input
  space (e.g. `wrap(x)` always in `(-π,π]`; `wrap(x+2π)==wrap(x)`), swept or randomized, so edge
  cases surface themselves instead of being guessed.
- **Mutation check** — for load-bearing code, deliberately break the implementation and confirm a
  test goes **red**. A suite that stays green while the code is wrong is theater.
- **Negative tests count** — code that *must not compile* (mixing units) and inputs that *must be
  rejected* (NaN) are tested too.
- Trivial confirmations are only ever **harness self-checks** (proving the runner works) and are
  retired once real tests exist — never counted as logic coverage.

---

## Freeze Register

These are the contracts the rest of the project is built on. Each is frozen at the milestone shown;
after that, **it changes only with a `schemaVersion`/API-version bump and a migration path** — we do
not silently break them. This table is the spine of the no-staleness promise.

| # | Frozen contract | What depends on it | Frozen at | Status |
|---|---|---|---|---|
| F1 | **Coordinate frame** — origin = field center, +X right, +Y away from red, heading 0 along +X / CCW-positive | Every motion, odometry, and transform line | M0 | ✅ **LOCKED 2026-06-08** |
| F2 | **Accuracy targets** — heading **< 1.0°** (hard); ~1.0″ pose; ~0.25″ docked | All acceptance tests; estimator design | M0 | ✅ **LOCKED 2026-06-08** |
| F3 | **Units & `Angle` semantics** — internal inches + radians + **seconds**; degrees only at the API edge; one wrap type normalized to `(-π,π]`; `shortestError(a,b)==wrap(b-a)` with the exact-180° case → **+π** (not −π), pinned by a red-on-failure test | Every numeric API signature | M0 | ✅ **LOCKED 2026-06-19** |
| F4 | **HAL interface signatures** — the 10 runtime HAL interfaces `IClock`/`IMotor`/`IRotation`/`IImu`/`IGps`/`IDistance`/`IOptical`/`IBattery`/`ITelemetrySink`/`IVision`+`ITagSource`. *(The config-ingestion seam `IRobotConfig`/`IRouteSource` — decision #10 — is authored at M5 with its `RobotConfig`/`Route` schema, F7/F8; not part of this runtime-HAL freeze.)* | All three runtime targets (robot/sim/test) | M1 | ✅ **LOCKED 2026-06-19** *(freeze-reviewed by a 30-agent full-set pass + exercised by `RobotContext`; on-V5 `hal/pros` adapters pending the toolchain)* |
| F5 | **`IKinematics` contract** — twist `(vx,vy,ω)` ⇄ wheels + desaturate + `strafeAuthority()` (a **pure read-only query** = max sustainable \|vy\|/\|vx\|; the motion layer clamps, kinematics never clamps inside `toWheels()` — §13 #5) | All motion code; new drivetrains | M1 | ✅ **LOCKED 2026-06-19** *(host-validated by X-drive + tank; on-V5 number-match pending)* |
| F6 | **Public `Chassis` API** — the whole facade surface, by group: **construction** `Chassis(deps, pacer, config = {})` + non-copyable/non-movable; **blocking verbs** `moveTo` / `strafeTo` / `turnTo` / `followTrajectory` (span + brace overloads) / `brake` / `hold(Time)` / `wait(Time)` — the last three adopted at D2; **manual verb** `drive(ChassisSpeeds, Frame)`, `Frame` required; **control** `cancel()` / `waitUntil(pred, Time)`; **state** `pose` / `setPose` / `strafeAuthority` / `lastExitReason` / `lastCompleted` / `motionConfig`; **Tier-3 seam** `deps()` / `scheduler()` (both overloads); **three public types** `ChassisConfig` / `MotionOptions` (existing fields frozen; the field *set* additive-open by design) / `TrajectoryResult` (incl. `succeeded()`); **plus the documented semantics** the header carries (blocking + watchdog, pre-empt, cancel safe state, fault policy, wait-for-live, strafe-fallback visibility). Time is typed `units::Time` across the surface (the D2 retype; `hold(500)` does not compile). **Deliberately OUTSIDE F6:** `Routine` / `RoutineResult` / `RoutineStopCause` — they froze at D3 as their own row **F10** (a different tier, versioning independently); the `MotionScheduler` / `MotionDeps` member surfaces reached through `scheduler()`/`deps()` (C1/C2's layers — only the accessors freeze here); the lower-layer config fields reached through `ChassisConfig` (they belong to their layers). Enforced structurally: compile-time signature pin `test/f6_signature_pin_test.cpp` (36 pins, mutation-proven) + the version mechanism `include/shulib/version.hpp` (API 2.0, breaking-vs-additive policy written) | Every auton ever written on shulib | M2 | ✅ **LOCKED 2026-08-12** *(at D2, after D1's second consumer; the nine-item critique rulings + rejected alternatives are the D2 completion record's centrepiece)* |
| F7 | **`robotProfile` sub-schema** inside `.vexbot` — drivetrain/odometry/sensors/mechanisms/corrections | Config codegen; every robot file | M5 | 🎯 *(coordinate with VexBuilder)* |
| F8 | **`paths[]` sub-schema + command-id vocabulary** inside `.vexbot` | Every data-driven routine | M5 | 🎯 *(coordinate with VexBuilder)* |
| F9 | **`SHUL/2` telemetry wire protocol** (v1) — the wire serialization of `DebugRecord` (§18) | Sim, record/replay, tuner, VexBuilder overlay; **every sink (`TermSink`/`SdSink`/`Shul2Sink`) shares the `DebugRecord` schema** | M6 | 🎯 |
| F10 | **Public `Routine` API (the Tier-2 recipe layer)** — construction `Routine(chassis, name = "routine")` `noexcept` + non-copyable/non-movable; **eleven steps**, each returning `Routine&`: `startAt` / `moveTo` / `driveTo` / `strafeTo` / `turnTo` / `face` / `followTrajectory` (span + brace overloads) / `brake` / `hold(Time)` / `pause(Time)` / `waitFor(pred, Time, name)`; **four observers** `ok()` / `result()` / `lastTrajectory()` / `chassis()`, all `noexcept`; **two public types** `RoutineResult` (all eight fields) and `RoutineStopCause` (**append-only** — the existing enumerator *values* are pinned, because a re-meaning is invisible at every call site); **plus the documented semantics**: eager execution (a step runs when it is chained), the stop/safe/skip/report error policy, preconditions throwing through with the chain's counters untouched, `lastTrajectory()` reading `exit = Running` until a trajectory has run, and typed time as a SEMANTIC (`hold(0.3)` must not compile). **Deliberately OUTSIDE F10:** `then()` — the mechanism seam, whose accepted return types and `name` default are a placeholder chosen before mechanisms existed (F1/F3 build them), so freezing it would commit to a guess; and the exact WORDING of the stop/skip log lines (the behaviour is frozen, the sentence is not). A **separate row from F6**, not an amendment: the recipe layer is a strict client of the facade, a different tier that can version independently, and amending F6 would retroactively blur what F6 promised on its own lock date. Enforced structurally: compile-time signature pin `test/routine_signature_pin_test.cpp` (37 pins, 16-mutation-proven, every `noexcept` pin using the compound-requirement detector that D2's hole #1 taught) + `include/shulib/version.hpp` | Every Tier-2 auton ever written on shulib | M7 | ✅ **LOCKED 2026-08-12** *(at D3, after the cookbook — its second consumer — wrote fourteen recipes against it and needed zero changes to the surface; the critique and its rulings are the D3 completion record's centrepiece)* |

---

## Milestones at a glance

> **You are here:** **M1 complete; M2 control + localization complete; Phase A COMPLETE
> (2026-08-06); Phase C COMPLETE (2026-08-10) — C1–C7 all closed. M2's STRUCTURAL clause closed
> at C7: the pre-rebuild tree is deleted, `make` produces an uploadable V5 package again, and
> `src/main.cpp` wires the v2 stack alone. M2's ON-ROBOT clause is OPEN, owned by R3 — the
> library has NEVER run on a physical robot, and the hal/pros adapters that would let it drive
> one are R1's deliverable.**
> **The auton API exists and is FROZEN at BOTH tiers — F6 (`Chassis`) LOCKED at D2 and F10
> (`Routine`, the recipe layer) LOCKED at D3, both 2026-08-12, API 2.0**
> (built at C4, stressed by D1's second consumer, ruled and pinned at D2):
> `moveTo`/`strafeTo`/`turnTo`/`followTrajectory`/`drive(ChassisSpeeds,
> Frame)` + `brake`/`hold`/`wait` (adopted at D2), blocking (async+waitUntilSettled),
> watchdog-bounded, typed `units::Time` everywhere,
> `Frame` REQUIRED at the call site (frame confusion is a compile error). One composition root
> feeds scheduler and every motion, so command-id stamping is STRUCTURAL (C2's convention gap
> closed, 138-assertion mutation pin). Facade routines are BIT-IDENTICAL to the scheduler-built
> twin (clean AND hostile) — C1–C3's accuracy numbers carry over verbatim — and every guarantee
> is re-pinned THROUGH the facade (ODO_STUCK abort, cancel safe state + panic stop, boot wait,
> hostile bounds, SFB visibility). The saturation choreography now lives in ONE place
> (`motion/command_pipeline.hpp`, bit-identity-pinned) shared by motions and `drive()`; the
> file-free plain-C++ construction path is a test (the §16.2 standalone promise). The mutation
> campaign (22 run) found and closed two GREEN HOLES: teleop health observables went dark in a
> `drive()`-only loop, and the yaw-rate budget was invisible to every closed-loop test.
> **Chunk C5 (per-motion results + session header + diagnostics D-1…D-5) is DONE, 2026-08-10,
> committed `be6d129`** (completion record: development log, `shulib-v2` branch):
> M2's "the run is legible in real time on the terminal" clause closes with it, and the F9
> schema space (D-2 drop counters + D-3 tick-phase slots) is reserved ahead of H1.
> **Chunk C6 (legacy salvage — the last look before deletion) is DONE, 2026-08-10, committed
> `d743a30`** (completion record: development log, `shulib-v2` branch): all 34
> legacy files classified, the three salvage claims verified in source (and `RobotCommands`
> found to have had **no executor at all** — the salvage is knowledge, not code; **port list
> empty by audit**, 11 live legacy bugs catalogued and left behind). Products:
> [`legacy-command-vocabulary.md`](legacy-command-vocabulary.md) (7 ids from 4 sources,
> mechanically cross-checked; gaps: `NONE` → G2, seat/settle wiggle → F2; 7 G4 importer
> requirements), the legacy-measured reference table in `hardware-assumptions.md`, and
> diagnostics-plan's C6 note. **Safe-to-delete verdict: unconditional** (C6 completion record §8, development log, `shulib-v2` branch).
> **Chunk C7 (cutover and deletion — the only irreversible chunk) is DONE, 2026-08-10 — in
> the working tree pending review/commit**: `src/main.cpp` rewired onto the v2 core (proven
> compiling BEFORE the deletion, so any failure stayed attributable), `src/legacy/` +
> `include/legacy/` DELETED — 34 files, exactly C6's classified set; recover any of them via
> `git show 691c656:<path>` — `make` green for the first time since June (uploadable V5
> hot/cold package), the CI PROS-free guard broadened to ALL of include/shulib/
> (mutation-proven red on a planted violation), the README rewritten as an honest front door,
> and the docs reorganized (public documentation stands alone; the process record is
> self-contained on the development branch). Suite unchanged: 659 / 915,570 / 3 deliberate
> skips; ARM gate 102 headers CLEAN. **What C7 did NOT do: run on a robot.** The wired HAL
> seams are shipped fakes marked TODO(R1); compiling is not running; R3 owns "it works".
> **Chunk C8 (the manual) is DONE, 2026-08-11 — in the working tree pending review/commit**:
> `docs/guide/` — a 15-file user guide (orientation → concepts → setup → a full first-routine
> tutorial → the API as prose → diagnostics line-by-line with every fault code → symptom-first
> troubleshooting → extending → an honest can't-do-yet → glossary), written for a new member
> with no robotics background. Every code example compiles and runs in
> `test/guide_examples_test.cpp` (8 cases / 41 assertions, quoted verbatim by the chapters —
> the anti-rot rule), all transcripts are captured from real runs, the tutorial was followed
> start-to-finish as written (including the type-it-yourself path), and chapter slot 09 is
> reserved so D1's recipe-API chapter is an added file, not a renumbering. Suite
> 667 / 915,611 / 3 deliberate skips (assertion total varies ±6 with the configure-time build-hash
> length); both guards + ARM gate (102 headers) clean; guide↔README cross-linked; the
> public-docs removability property re-verified with the guide in place. **What C8 did NOT
> produce: the recipe cookbook and generated API reference (D3, needs D1), the
> VexBuilder-based "first auton in 10 minutes" flow (G4), and a guide validated by an actual
> new reader — it has not had one yet.**
> **Chunk D1 (the Tier-2 recipe API) is DONE, 2026-08-11 — in the working tree pending
> review/commit**: `include/shulib/chassis/routine.hpp` — `Routine`, an EAGER fluent chain
> over the facade (each step runs as it is chained, so a routine reads and runs top-to-bottom
> — the deferred/`.run()` alternative was analyzed and rejected in the D1 completion record).
> Twelve step verbs (startAt / moveTo / driveTo / strafeTo / turnTo / face /
> followTrajectory / brake / hold / pause / waitFor / then), every one a single delegated
> facade call — the recipe-vs-facade TWIN is bit-identical clean AND hostile, so C1–C4's
> accuracy numbers carry over verbatim, and the chain's error policy is decided and pinned:
> first failure → STOP + SAFE (0 V + Brake) + SKIP the rest (logged) + REPORT
> (`RoutineResult` with step index/name/cause/exit). Guarantees re-verified THROUGH the
> chain: ODO_STUCK abort (named, prompt, damage-bounded), watchdog + never-live boot, C2's
> hostile bound, mixed-tier interop (recipe steps and direct verbs interleave — no cliff).
> Guide chapter 09 written with 3 compiled examples (`guide-09a/b/c`, quoted verbatim);
> suite 686 / 916,026 green; 19 mutations run, all observed red — two of them verify holes
> found by pre-analysis and closed with sole-detector tests (a no-op `startAt` masked by
> every rig's auto-seed; speed budgets silently dropped in delegation), see the D1
> completion record. **What D1 did NOT do: freeze F6** (that was D2's, informed by D1's
> facade critique — the completion record's top section), and `.then(intake.in)` remains a
> labeled placeholder until F1/F3 build mechanisms.
> **Chunk D2 (the F6 freeze) is DONE, 2026-08-12** (completion record: development log,
> `shulib-v2` branch): all nine D1 §2 critique items ruled with rejected alternatives
> (time RETYPED to `units::Time` with full-suite output byte-identical; `wait(Time)`
> ADOPTED as an additive void verb; `brake`/`hold` ADOPTED; the runtime-mask deferral
> CLOSED; the fallback-getter rejection KEPT; sugar stays recipe-layer; three result
> vocabularies ACCEPTED + flagged for F2; all four §8 tensions confirmed no-change with
> additive paths named), all 18 C4 §2 ledger rows re-checked, and the freeze made
> STRUCTURAL: 36 compile-time signature pins that fail the build naming F6
> (mutation-proven member by member), `include/shulib/version.hpp` (API 2.0, the
> breaking-vs-additive policy the register had been promising with no machinery), the F6
> row rewritten to enumerate the real surface with `Routine`'s exclusion explicit, and
> the not-frozen notices swept atomically. The campaign (44 mutations) found and closed
> two green holes: the pin was blind to noexcept-dropping on non-overloaded members, and
> a thousand-fold `hold(300_s)` call-site mistake survived every outcome assertion —
> `guide-09a` now clock-bounds the whole first recipe (< 12 s simulated).
> **Chunk D3 (the cookbook + generated API reference) is DONE, 2026-08-12 — closes Phase D**
> (completion record: development log, `shulib-v2` branch): `docs/cookbook/` — fourteen recipes
> written as a critical consumer of `Routine`, every listing compiled and RUN against the plant
> (`test/cookbook_examples_test.cpp`, 11 cases) and quoted verbatim, with clock-asserted timing
> wherever a duration matters. `docs/api/` — a generated reference for the F6 and F10 surfaces,
> deterministic and committed, produced by `tools/api_doc_tool.py`, with **three build-time
> gates**: a doc-coverage gate that fails the build NAMING any undocumented public member (it
> found 16 on its first run, on the surface about to freeze — including `class Chassis` itself,
> `TrajectoryResult::succeeded()` and the const `scheduler()` overload), a regeneration check,
> and a C++ fidelity pin holding the rendered signatures to the headers. **`Routine` is now
> FROZEN — F10 LOCKED 2026-08-12, API 2.0**, as its own register row rather than an F6
> amendment (a different tier, versioning independently), with `then()` explicitly excluded
> until F1/F3 build mechanisms; 37 compile-time pins, all 16 pin mutations red and named.
> The cookbook's critique of `Routine` — eight awkwardnesses, each with a recommendation and
> each additively fixable — is the record's centrepiece, and it found one real BUG:
> `lastTrajectory()` on a routine that never ran a trajectory reported a SUCCESSFUL one (a
> value-initialized `TrajectoryResult` is `Settled`, 0 of 0 legs). The campaign (45 + 16
> mutations) found and closed **three green holes**, all of them enforcement that existed only
> as convention: the guide's verbatim anti-rot rule and the C7 removability property were run
> by an internal script and by **neither the build nor CI** (a chapter's `300_ms` retyped to
> `300_s` built clean and passed the whole suite), and the doc-coverage gate itself accepted an
> EMPTY `///` as documentation. All three are now build- and CI-enforced.
> **Not done, and named as such: nothing is published** — there is no website and no Pages
> configuration, so M7's "API docs publish to the website" stays `[~]`; and no genuinely-new
> reader has read the cookbook cold, so M7's human DoD clause stays open (owner: the programming
> chair, at the next new-member onboarding — the first clause with a named owner rather than a
> hope).
> **Chunk E1 (the `SdSink` blackbox + estimator introspection) is DONE, 2026-08-12 — opens
> Phase E**: the run is now recordable without a laptop. `diag/blackbox_format.hpp` defines a
> versioned, session-stamped, fixed-width binary format (v1: 256-byte header, typed frames,
> 428-byte tick record, IEEE-754 binary64 throughout — no narrowing, so a decoded record equals
> the encoded one field for field), and **the decoder ships in the same chunk**
> (`blackbox_reader.hpp`): an encoder without a decoder is not a record. `diag::SdSink`
> implements diagnostics-plan **D-6** (a RAM flight recorder, always on, that writes NOTHING
> until a fault fires and then dumps the preceding ticks) and **D-7** (a triage block, in the
> file and on the terminal, from one shared struct). Writes are **caller-paced**: the build
> order's "off-task writes" was ruled against in favour of C2/C4's standing no-background-task
> decision, because host determinism is what makes every closed-loop test here reproducible.
> The binary seam is a NEW sibling (`hal::IBlockSink`), not a redefinition of `ICharSink`.
> Suite 752 / 936,895; both guards and the ARM gate (110 headers) clean.
> **What E1 did NOT do, stated plainly:** it did not certify the `< 1°` claim and it did not
> build a real gate. The introspection PATH is complete and proven end to end, but with a
> **synthetic** corrector — E2/E3/E4 supply the real numbers, and `gateMahalanobis` stays 0
> until the EKF exists. There is also no `/usd/` adapter (R1's) and no D-8 routine watchdog
> (re-homed to F2, with the reasoning recorded rather than the work half-done). Two findings
> came out of it that predate the chunk: **`DebugRecord::fault` had no producer anywhere in the
> tree** — the guide has documented ` flt=CODE` since C8 and it could never have appeared on a
> real run — and only `MoveToPose` stamped the applied-correction fields, so the fusion story
> was blank for every other motion. Both are now stamped in the one layer that owns record
> population. The mutation campaign (27 executed) found **two green holes**, both the same
> class: the suite checked that the FORMAT could carry the sink's self-description and never
> that the SINK filled it in — the end frame's counts and the header's epoch/ring/budget could
> all be zeroed with the suite green.
> **Chunk E2 (`GpsCorrector`) is DONE, 2026-08-12 — the first REAL corrector.** Everything before
> it dead-reckoned; this is the first code in the library that can tell the estimator it is wrong,
> and the first chunk in which the §18.2 gating slots carry decisions a real gate actually made.
> `GpsCorrector` implements `ICorrector` behind the unchanged M2 signature — adaptive R from
> `rmsError()`, latency compensation from an odometry history ring, a stale-sample guard so one
> measurement is folded once rather than five times, high-yaw-rate rejection, a sensor-quality
> ceiling, and a normalized-innovation gate that WIDENS with dead-reckoned travel (without which
> a truthful fix is rejected exactly when the estimate needs it most). Suite 794 / 1,081,382;
> both guards and the ARM gate (111 headers) clean; 20/20 mutations red.
> **The chunk closes the other half of E1's T3.** A declined proposal never reaches a fusion
> policy, so a corrector-side verdict had no channel to the record at all: an off-strip GPS and
> an empty corrector list both produced `GateReason::None`. Since **Driving Skills has no strip**,
> that was the difference between a diagnosable Skills run and a mystery. `CorrectionProposal`
> gained one appended `selfAudit` field, the `Localizer` substitutes it only when the policy has
> no verdict of its own, and every corrector-side rejection is now re-derivable from the decoded
> file — residual, the sigma it was normalized by, and the verdict.
> **What E2 did NOT do, stated plainly.** It did not build a Mahalanobis gate (T1: the
> complementary tier has no filter-estimated covariance, so `gateMahalanobis` stays 0 and the
> honest `RejectedNormalizedInnovation` was appended instead); it did not touch heading (T3 — the
> GPS's heading never leaves the corrector, so the `< 1°` claim is exactly where A3 left it); and
> it did not take ownership of frame or lever-arm reduction (T4 — `hal/gps.hpp` and
> `gps_conversion.hpp` own those by written contract, so E2 pinned them with independent oracles
> rather than becoming a second subtractor). The accuracy claim is an **aggregate** across 8 seeds,
> not a per-seed sweep, and the record says so: in simulation the modeled GPS noise and the modeled
> dead-reckon drift are the same order of magnitude, both invented (HA-26, HA-20), so the measured
> gain is real but modest. Two findings came out of it that predate the chunk: **HA-07's
> metres→inches conversion had no code and no test** — prose only since A4, with the existing
> conversion tests pinning the scale against the constant imported from the header under test —
> and **`ComplementaryFusion`'s fixed 12-inch innovation gate caps what any corrector can repair**,
> observed live when an estimate 29 inches out never recovered with a good GPS in view. The first
> is fixed in the layer that owns it; the second is recorded for E4, which replaces that policy.
> The mutation campaign (20 executed) found **one green hole**: the substitution rule's
> `reason == None` guard is dead code with one corrector and load-bearing with two, so a silent
> source could stamp `RejectedNoFix` over a tick that actually applied a fix — latent until E3.
> **Chunk E3 (`AprilTagCorrector`) is DONE, 2026-08-13 — the second corrector, and the first
> ABSOLUTE YAW CORRECTION in the library's history.** A tag observation is different in kind from
> a GPS fix: it is a relative *pose*, so against a tag whose field pose is known it yields an
> absolute heading, which no other source in the tree can provide. Three new headers —
> `hal/vision_conversion.hpp` (the corners→pose PnP, a free pure function that R2's adapter will
> call and the corrector deliberately does not include), `localization/tag_map.hpp`, and
> `localization/apriltag_corrector.hpp`. Suite **867 / 1,091,167**; both guards and the ARM gate
> (114 headers) clean; doc gates all pass; **44 mutations, 43 red**.
> **The yaw path is the documented additive one and nothing frozen moved.** `FusionResult` gained
> a trailing `headingNudge` (a bounded INCREMENT, never an absolute heading — a policy that could
> return an absolute heading could snap, one that can only return an increment cannot), the
> `Localizer` folds it into a PERSISTENT heading bias and composes the published heading as
> `imu.heading() + bias` as the last write of the tick. The IMU remains the sole source of heading
> CHANGE and `PilonsOdometry` is untouched. The accumulator is not optional: the Localizer
> re-reads the IMU every tick, so a nudge applied only to the published pose would be discarded on
> the next one — verbatim the M2 red team's *corrections-not-accumulating* failure. `E3` also
> re-expresses the odometry delta under the learned bias, because otherwise the reported heading
> improves while the dead-reckoned position keeps accumulating `bias × distance` of cross-track
> error. With no heading-providing corrector, both paths are **bit-identical to pre-E3 by
> construction** (an explicit zero-bias early-out, not a floating-point argument).
> **What E3 did NOT do, stated plainly.** It does **not** claim the `< 1°` requirement is met, or
> that it is on track. What was measured is: a 4° IMU error closes to 0.5° in ~3 s and to ~1e-4°
> in 15 s, moving toward truth on every one of 1500 ticks and never past it; and no tick ever
> moved the heading by more than the documented 0.1° per-tick bound across 2000 ticks, re-read
> from a decoded blackbox as well as from live state. It was measured against **simulated truth,
> a simulated camera, an invented tag map and invented noise magnitudes** (HA-68…HA-82, fifteen
> new register entries). The single honest change to guide chapter 14 is that *the specific reason
> the requirement was listed as unachievable — nothing could correct heading at all — no longer
> applies*; everything else about that claim is unchanged and unmeasured.
> **shulib ships NO tag map, deliberately**, because nobody on this project can cite a table of
> AprilTag field poses; `TagMap::add()` refuses an entry that does not state its provenance, and
> an unknown tag id produces `RejectedNoTagMapEntry` rather than a guess. Three findings worth
> carrying: **a reversed corner winding is catastrophic and SILENT** (heading 180° out with the
> reprojection error at machine zero — no software self-check can see it, HA-69); the mutation
> harness itself had a fault that reported two mutations GREEN having never run (multi-line
> `grep -F` matches any single line), now impossible via a byte-compare; and **one green hole is
> recorded unclosed with its measurement rather than papered over** (see `E3-COMPLETED.md`).
> **Next: E4** (the 5-state SE(2) EKF — real covariance, real Mahalanobis gating, and the owner of
> `ComplementaryFusion`'s fixed 12-inch ceiling that E2 recorded and E3 confirmed still binds).
> (There is no Phase B: the original hardware phase was resequenced to Phase R when the
> execution order was planned — the lettering keeps the gap rather than papering over it.)
> **M1:** F4 (10 HAL interfaces) + F5 (kinematics) both **LOCKED & host-validated** — math/units/frame,
> `MatrixKinematics`/`xDrive()`/`TankKinematics`/desaturation, all 10 interfaces + fakes, the IMU & GPS
> canonical conversions (each **red-teamed**), and `RobotContext`. **M2 control layer (WS4) is done:**
> `Pid`, `Feedforward`+battery-comp, `SettledUtil`, `TrapezoidProfile`, `Watchdog`, `ExitGroup`.
> **M2 localization (WS5) is done — the full stack:** `arcStep` (exact SE(2) integrator — re-derived
> clean, which **caught a real legacy bug**: the old odom rotated each tick's chord by the *new* heading
> instead of the *average*), `TrackingWheel` (role-stamped), `PilonsOdometry` (IMU-owned heading + offset
> correction + trust/finiteness gate), and the fused **`Localizer`** behind an EKF-ready seam
> (`IPoseSource`/`ICorrector`/`IFusionPolicy`) with `ComplementaryFusion`'s innovation-bounded **gated
> nudge (never snaps)**, structural **IMU-owned heading**, a measurable `quality()` + dead-reckon flag,
> and GPS/AprilTag correctors **stubbed → wired at M3**. Two multi-agent red-teams (29 + 43 agents, every
> finding adversarially re-verified) hardened it — the localizer pass caught a **CRITICAL**: corrections
> weren't accumulating (fused = odom + one-tick nudge → couldn't converge to a persistent fix and snapped
> back when a corrector went quiet); now the fused state accumulates odom deltas + retains nudges, with
> convergence/persistence tests.
> **Also fixed: the on-robot ARM build** — root-caused to a stale soft-float `firmware/liblvgl.a` +
> gcc-14 standard names (NOT the toolchain, as long assumed); the kernel cold-package now links.
> **Chunk A1 (WS13 diagnostics) is done — 2026-08-01:** the complete-§18.2-schema `DebugRecord`
> (typed units, fields reserved for E2–E4 systems, F9-freeze-aware) behind an **additively extended**
> `ITelemetrySink` (`emit()` non-pure/no-op default; `wantsRecord()`+`emitRecord()` so a `NullSink`
> never even populates a record); **`TermSink`** with injected clock + injected char-sink and
> byte-pinned golden output (NaN/±Inf/1e300/control-byte/UTF-8-truncation ugly cases covered);
> **compile-time `TRACE` strip** (args provably unevaluated; stripped call = byte-identical ARM `-Os`
> code to no call); **fault discipline** — wire-pinned `FaultCode`, first-fault-latching
> crash-proof `FaultLatch`, `LoopMonitor` (inclusive `>=` dt-budget edge pinned), NaN/Inf
> log-and-recover guards with an unconditional finite-return guarantee — and the **`check.hpp`
> §18.4 policy seam** (host throws / robot routes to fault-log; call sites unchanged). The three
> legacy `logger.hpp` defects were designed out structurally (clean-room, not ported). **7 mutations
> proven red, then restored.** Full record: the A1 completion record (development log, `shulib-v2` branch).
> **Chunk A2 (the host plant + closed-loop sim harness) is done — 2026-08-01:** the roadmap's
> incompleteness bug (M2's and M4's DoDs required a "host sim" no task built) is CLOSED. Six
> headers under `include/shulib/sim/` deliver: voltage→velocity by **exact inversion of the existing
> `Feedforward` relation** (kS dead band + τ=kA/kV first-order lag; **kinematic, not dynamic** — no
> invented mass/friction constants, gains provisional until R5/R6); a ground-truth pose integrator
> that is **provably independent of `arcStep`** (RK4 on unwrapped θ; a >π-per-tick tripwire test
> goes red if truth ever reuses arcStep — the trap that would have silently nulled every Phase E
> measurement); sensor synthesis from truth into the **unmodified F4 fakes** (zero additive setters
> needed); the **first closed loops in the project** (`Pid` through the sensor path converges and
> holds; a sign-flipped gain diverges; an overdriven gain chatters at the DERIVED discrete-instability
> threshold); the **first end-to-end localization proof** (`PilonsOdometry` + `Localizer` track truth
> ≤1e-6″ over multi-second scripts, and a deliberate 2% mis-calibration is DETECTED at its predicted
> magnitude); **seeded byte-identical determinism** (SplitMix64, memcmp-pinned); and the **nine A3
> degradation seams, empty but proven live**. CI gains a layering guard (core may never include
> `shulib/sim/`). **8 mutations proven red, then restored.** Full
> record: the A2 completion record (development log, `shulib-v2` branch).
> **Chunk A3 (hostile fakes) is done — 2026-08-02:** the nine A2 degradation seams are POPULATED —
> seven `sim/hostile/` headers model how V5 hardware actually lies (IMU calibration-garbage window +
> per-boot drift + noise + dropout; GPS decimation/noise/no-fix/off-strip/bad-fix; encoder
> quantization/freeze/PROS_ERR_F sentinel breach/bump-skid; battery sag→brownout collapse + thermal
> droop; accel-triggered wheel slip; ring-buffered sensor latency; `ChainedDegradation` +
> `FullHostility` composition + a seeded `JitterSchedule`), every invented magnitude labelled
> **PROVISIONAL (A4)** in-header. **It did its job: hostility found three real Localizer defects and
> they are fixed at the source** — (1) calibration garbage poisoned the fused pose 10.8″ on a
> stationary robot (boot guard added), (2) `isReady()` outruns a latency-delayed heading stream, so a
> settle window now holds the fold after a witnessed boot (found ONLY by the composed model — the
> reducibility design paying off), (3) mid-run IMU loss misreported as `Uninitialized` (now
> `Degraded`). Fault discipline proven real: `diag::HealthMonitor` (edge-triggered episodes,
> brownout hysteresis + latched marker) + `FaultCode::MotorOverTemp` appended; every pathology in a
> 9-attack survival matrix raises its code with a finite pose on every tick. The **M2 `<1°`
> acceptance test is LIVE and measured**: worst end-of-60s heading error **0.912°** over 10 seeded
> boots under full hostility (cap 1.0° — passes; worst instantaneous 1.065° transiently over, and
> the margin at the pessimistic ±1°/min provisional drift bound is ~zero BY CONSTRUCTION: R4's
> measurement is the ceiling, Phase E's correctors buy margin). **7 mutations proven red, then
> restored.** Full record: the A3 completion record (development log, `shulib-v2` branch).
> **Chunk A4 (Hardware Assumptions Register + ARM compile gate) is done — 2026-08-06, closing
> Phase A:** every claim about physical hardware that three no-robot chunks (plus the M1/M2
> conversion layers) rest on is now **inventoried instead of scattered** —
> **[`docs/hardware-assumptions.md`](hardware-assumptions.md)**, 49 falsifiable entries
> (**33 invented / 13 reasoned / 2 measured-elsewhere / 1 mixed** — the honest cost of building
> without hardware), each with source, confidence, the specific settling measurement, its owning
> chunk (R3 conventions/geometry · R4 noise/drift · R5 gains · R6 back-fit) and its **blast
> radius if wrong** (most are contained behind the `hal/pros` seam or a single constant BY
> DESIGN; the exception worth knowing: HA-19, brownout CPU-survival, which the F2 guaranteed-park
> design presupposes). Reconciliation is bidirectional and grep-verified: every
> `PROVISIONAL (A4: HA-nn)` label in the tree maps to an entry and every header-sourced entry
> points back — zero orphans. **CI now holds the ARM line**: a second job cross-compiles every
> v2 header for the V5's Cortex-A9 from a **generated** header list under the same strict flags
> as host — a compile gate (link/run stay R1/R3), **proven** to catch a host-only construct that
> the host build is provably blind to (mutation: host GREEN, gate RED, restored). This register
> is R3's day-one checklist — first hardware contact is a prepared sequence, not an exploration.
> Full record: the A4 completion record, incl. the Phase A retrospective (development log, `shulib-v2` branch).
> *(Status line as of A4, kept for the record: next was C3; C1 closed 2026-08-06, C2 built and
> verified 2026-08-06 — completion records in the development log.)* The *what* is still this
> page; the **order** lived in the dependency-ordered execution plan — 39 chunks, written
> against the governing constraint that **there is no robot yet** — on the `shulib-v2`
> development branch.
> *Carry-overs (tracked, now placed): `hal/pros` adapters + v2 `src/main.cpp` → R1/R3; `MatrixKinematics`
> non-orthogonal pseudo-inverse → C3; `sysid` → R5; estimator-side frozen-encoder detection → E-phase
> (the loop-level cross-check shape is tested at A3).*
> *Status verified 2026-08-06 (post-C2): host suite **527 cases / 859,931 assertions green** under
> strict `-Werror` (3 deliberately skipped: two M3 acceptance stubs + the R3 GPS field-cal oracle
> = register entry HA-01); both CI guards green (scopes unchanged — the scheduler lives in
> `include/shulib/motion`, already covered); all **86** v2 headers cross-compile clean for ARM
> under the CI `arm-compile-gate` job (generated list — `motion_scheduler.hpp` is covered
> automatically). C2 ran **16 mutations, all observed red** (C2-COMPLETED §Mutations); C1's 12
> stand as recorded.*

| Milestone | Theme | DoD headline | Status |
|---|---|---|---|
| **M0** | Foundation & scaffolding | Frame frozen in a test; host-test harness + CI green; repo clean-room-ready | ✅ |
| **M1** | HAL + kinematics | Same kinematics run identically in a host test and on the V5 by swapping only `RobotContext` | 🎯 |
| **M2** | Holonomic motion + dead-reckon localizer | A hand-written X-drive auton chains profiled motions; same code runs the H-bot | 🎯 |
| **M3** | Accuracy edge (fusion + docking) | Pose bounded over a 60s run (< 1°); docking nests a 1.6″ Pin repeatably | 🎯 |
| **M4** | Skills layer + guaranteed park | Full scoring primitives; the +8 park always fires even on a stalled loop | 🎯 |
| **M5** | Autonomy authoring + `.vexbot` ingestion | A non-coder builds, exports, and runs a complete routine with zero C++ | 🎯 |
| **M6** | Ecosystem (telemetry + sim seam + tuner) | A real run replays in VexBuilder; tuning happens on-brain | 🔭 |
| **M7** | Accessibility & docs | "First auton in 10 minutes" works end-to-end; API docs publish to the website | 🔭 |
| **M8** | Second robot + coordination seam | Both robots run shulib; a thin coordination seam exists (stretch) | 🔭 |
| **Frontier** | The future (continuous) | EKF++/LIDAR, mecanum/swerve, dynamic replan, full physics round-trip | 🔭 |

The milestones below list **every task** under each. The same tasks are also grouped by permanent
workstream in [§ Workstreams](#workstreams) — two views of one backlog.

---

## The milestones (full breakdown)

### M0 — Foundation & scaffolding ✅
*Make the old bug classes structurally impossible, and stand up the machine that proves it.*

**Conventions & math (WS1)**
- [x] Freeze the coordinate frame **in a regression unit test** (F1). *Done via the
  `fieldToRobot`/`robotToField` transform below — `test/frame_test.cpp` pins +X/CCW.*
- [x] `units::Quantity<Dim>` — **the 6 canonical dims (source of truth):** length=inch, angle=radian,
  time=**second**, velocity=in/s, acceleration=in/s², voltage=volt. Literals (`_in` `_deg` `_tile`
  `_ms` `_s` `_volt`) are **convenience-only** (need not be 1:1 with dims); `_deg` builds an
  auto-wrapped `Angle`, not a bare `Quantity`. Compile-time dimensional safety (F3). *Verified
  2026-06-19: `include/shulib/units/{quantity,literals}.hpp` with `test/quantity_test.cpp` — green
  under strict `-Werror`, mutation-checked both ways (a runtime break goes red; flipping a negative
  `static_assert` fails the build).*
- [x] `Angle`/`Rotation2d` — wrap to `(−π,π]`, shortest-path `errorTo` (+180° tie-break), the **only**
  degree⇄radian boundary, non-finite rejected (F3). *Verified 2026-06-19: `include/shulib/math/angle.hpp`
  with `test/angle_test.cpp` — 10 cases / 300k assertions green under strict `-Werror`, **mutation-checked**
  (breaking the boundary fold turns the tie-break tests red).*
- [x] `Pose2d`, `Twist2d`, `ChassisSpeeds` value types. *Verified 2026-06-19:
  `include/shulib/math/{pose2d,twist2d}.hpp` with `test/pose2d_test.cpp` — green under strict
  `-Werror`, mutation-checked (naive heading compare reds the ±180° seam test); compile-time checks
  reject wrong units (a `Time` where a `Length` belongs, a measured `Twist2d` for a `ChassisSpeeds`).*
- [x] The one `fieldToRobot()` / `robotToField()` transform + its test. *Verified 2026-06-19:
  `include/shulib/math/frame.hpp` with `test/frame_test.cpp` — ~520k assertions green under strict
  `-Werror`, mutation-checked (one rotation sign flip reds the pinned-direction AND round-trip tests).
  **F1 frozen.***
- [x] Encode the accuracy targets (F2) as the first acceptance-test stubs. *Verified 2026-06-19:
  `include/shulib/spec/accuracy.hpp` (single source of truth) with `test/accuracy_spec_test.cpp` —
  value-guard + consistency invariants green; 3 system-level acceptance stubs registered & skipped
  (go live M2/M3). **M0 Conventions & math block complete.***

**Tooling, build & CI (WS11)**
- [x] Bump PROS kernel **4.1.0 → 4.2.2** (unlocks native AprilTag); `project.pros` Windows path fixed.
  *Done 2026-06-19 via `pros c apply kernel@4.2.2`: version=4.2.2; `pros/ai_vision.hpp` (4 tag families,
  `enable_detection_types`, `set_tag_family`) now in-tree; **all source compiles under 4.2.2**.*
- [ ] **On-robot toolchain** (surfaced by the bump, pre-existing): the ARM **link** fails on this Linux
  box — distro `arm-none-eabi-gcc` 13.2.1 links a **hard-float** `libgcc.a`/`libm.a` against the
  project's softfp objects (VFP-register-args mismatch). Independent of the bump/our code. Fix = use
  PROS's bundled toolchain (not Ubuntu's); verify the link on the robot build machine.
- [x] Stand up the **host-test harness** — CMake + doctest v2.4.11, strict `-Werror -Wconversion …`,
  separate from the PROS ARM Makefile. *Verified 2026-06-19: builds clean, **green on truth (exit 0)
  AND red on falsehood (exit 1)**. Evidence: `test/`, `cmake -S test -B build/test && cmake --build
  build/test && ./build/test/shulib_tests`.*
- [x] **CI** (`.github/workflows/ci.yml`): the **no-`<pros/>`-in-core guard** + strict host build
  (`-Werror`) + the test suite. *Done 2026-06-19: commands verified locally — the guard is
  mutation-checked (injecting a `pros/` include into a core header makes it exit 1), build+tests green;
  Actions runs verify on first push. **Scope:** on-robot ARM build deliberately NOT in CI (needs PROS's
  toolchain/robot — see the toolchain item) and markdown lint left out (cosmetic-warning noise); both
  addable later. Guard scope broadens to all of `shulib/` after the M2 cutover.
  *(Fulfilled at C7, 2026-08-10: the guard now covers all of `include/shulib/`.)*
  **Update (A4, 2026-08-06):** CI now also carries the ARM **compile** gate — every v2 header
  cross-compiled for Cortex-A9 from a generated list; the on-robot LINK/run remain R1/R3 as scoped.*
- [x] Regenerate `compile_commands.json` for editor tooling. *Done 2026-06-19:
  `CMAKE_EXPORT_COMPILE_COMMANDS` on; `build/test/compile_commands.json` symlinked at repo root
  (gitignored) so clangd resolves `doctest.h` + the shulib headers (fixes the editor false-positives).*
- [x] Clean-room `shulib/` layout + **legacy quarantined to `src/legacy/` + `include/legacy/`**
  (reference-only) so the new `shulib/` path is collision-free. *Done 2026-06-19: moved
  `include/shulib/{api.hpp,chassis,gui,logger.hpp,pid.hpp,pose.hpp,RobotCommands,util.hpp}` + all of
  `src/shulib/` + the loose `src/` files to `legacy/`; `include/shulib/` now holds only the verified
  core (`core/ math/ units/ spec/`); host tests stay green. Full deletion at the M2 cutover (after
  salvaging the Pilons math + `RobotCommands` + `logger.hpp`). PROS/ARM build has no `main.cpp` now —
  in-flux until the M1/M2 wiring, as planned. (The deletion happened at C7, 2026-08-10, after C6's
  salvage audit; a fresh v2 `src/main.cpp` restored the PROS/ARM build the same day.)*

**Compliance (WS12)**
- [x] **Delete `src/tracking.md`** (verbatim LLM transcript — not authored by us). *Done 2026-06-19.*
- [x] No machine-generated motion code ships — legacy `main.cpp` is quarantined; the new auton is
  written fresh at M1/M2. *Done 2026-06-19 (clean-room).*

**Definition of Done:** the frame transform and `Angle` wrap have passing host tests; CI is green and
blocks `<pros/>` in core; the kernel is on 4.2.2; `tracking.md` is gone.
**Freezes:** F1 ✅, F2 ✅, F3 ✅.

---

### M1 — HAL + kinematics foundation 🎯
*One seam for hardware/sim/test; drivetrains become interchangeable.*

**HAL (WS2)** — *F4 interfaces frozen 2026-06-19 (30-agent review + RobotContext); `hal/pros` adapters pending the toolchain*
- [x] Define all HAL interfaces (F4): `IMotor`, `IRotation`, `IImu`, `IGps`, `IVision`/`ITagSource`,
  `IDistance`, `IOptical`, `IClock`, `ITelemetrySink`, plus a battery-voltage source. *Done 2026-06-19:
  `IClock` (`hal/clock.hpp`), `IMotor` (`hal/motor.hpp`, ±12 V clamp + non-finite + cumulative-position
  contract), `IImu` (`hal/imu.hpp`, canonical heading/yawRate/calibration/tilt), `IGps`
  (`hal/gps.hpp`, canonical center pose + rmsError + hasFix), and the simple reads `IRotation`
  (cumulative non-wrapping), `IDistance` (inches + confidence), `IOptical` (hue/sat/bri/prox),
  `IBattery` (volts + capacity, for brownout comp), and `ITelemetrySink` + the zero-cost `NullSink`
  (§18 diagnostics seam; leveled messages now, per-tick `DebugRecord` rides behind it at M2), and
  `IVision`/`ITagSource` (decision #7: tags as robot-relative poses, objects as bearings; the PnP /
  bearing reductions are M3/M4 pure functions). **All 10 F4 interfaces frozen** — a 30-agent full-set
  review closed the breaking-if-deferred gaps (added `IMotor::current()`/`temperature()`/brake-mode,
  `IBattery::current()` + the 5th units dimension; flipped `isCalibrating()`→`isReady()`), then
  `RobotContext` exercised the whole set.*
- [ ] `hal/pros/*` adapters — the **only** files that include `<pros/*>`. IMU compass/CW → canonical;
  GPS frame → canonical (the conversions happen here, once). *Pure conversion math built+host-tested
  ahead of the adapters, each adversarially **red-teamed** (4-lens workflow, math verified correct):
  `hal/imu_conversion.hpp` — get_rotation()-binding / no-post-cal-tare / bootHeading-ownership /
  yaw-rate-source contracts pinned; `hal/gps_conversion.hpp` (CW-from-North→canonical, m→in rotation,
  lever-arm removal) — axis-assumption (validate-on-field), no-firmware-offset, PROS_ERR_F-screening,
  North/lever ownership, and a non-finite-lever-arm guard pinned (§7 + headers). pros glue + on-V5
  validation still pending the toolchain.*
- [ ] Vendor & extend the existing `ai_vision.hpp` wrapper into the `hal/pros` `IVision`/`ITagSource`
  adapter (object mode **and** AprilTag mode).
- [~] `hal/fake/*` deterministic doubles (injectable clock) for host tests. *Done 2026-06-19:
  `FakeClock` (monotonicity-enforcing), `FakeMotor` (real clamp/validation + injectable encoder),
  `FakeImu` (canonical injectable incl. ±180° seam), `FakeGps` (canonical pose + fix/error, safe no-fix
  default), the pure-read `FakeRotation`/`FakeDistance`/`FakeOptical`/`FakeBattery`, the recording
  `FakeTelemetrySink`, and `FakeTagSource`/`FakeVision` — each tested for contract/round-trip (the
  stateful ones also mutation-checked).*
- [x] `RobotContext` — the DI container; the one object that differs across robot/sim/test. *Done
  2026-06-19: `include/shulib/chassis/robot_context.hpp` — named-pointer config validated non-null,
  hands out references to all the HAL handles; `test/robot_context_test.cpp` mutation-checks the
  null-validation and demonstrates the M1 DoD (field→body→wheels pipeline reading the heading through
  the context; numbers identical regardless of HAL impl).*

**Kinematics (WS3)** — *complete & host-validated 2026-06-19; F5 host-frozen (on-V5 number-match pending)*
- [x] `IKinematics` contract: `toWheels`/`forward`/`desaturate`/`strafeAuthority()`/`wheelCount` (F5).
  *`include/shulib/kinematics/{kinematics,wheel_speeds}.hpp`; contract-tested
  (`test/kinematics_contract_test.cpp`) — implementable & polymorphic; `WheelSpeeds` bounds/`maxMagnitude`/
  `approxEqual` mutation-checked. **F5 validated end-to-end by the two drives below**, so it is frozen
  host-side; the on-V5 identical-numbers check rides with the ARM-toolchain item.*
- [x] `XDriveKinematics`, `TankKinematics` — host-tested with pure numbers. *X-drive is a
  `MatrixKinematics` coefficient-table preset (`xDrive()`, §13 #15 hybrid): the `matrix_kinematics.hpp`
  engine (orthogonal-column forward; rank-3 and orthogonality preconditions reject malformed tables), plus
  the `x_drive.hpp` and `tank.hpp` presets. Tests pin the physical signatures (X-drive √2-forward,
  all-wheels-equal spin, strafe⊥forward; tank forward=equal / rotation=opposite / strafe-ignored / no
  phantom vx) and swept round-trips.
  **Mutation-checked**: forward divisor swap, dropped yaw term, loosened orthogonality guard, an X-drive
  sign flip (rejected at construction), and the tank ω sign — each proven red, then restored.*
- [x] Uniform-scale wheel desaturation. *`desaturateUniform` (`desaturate.hpp`) — within-budget unchanged,
  over-budget peak-onto-limit, uniform ratio/direction preservation, all-zero no-op, non-positive budget
  rejected; wired into both drives' `desaturate()` and tested via the `MatrixKinematics` delegation path.
  Mutation-checked (inverting the scale factor goes red).*
- [ ] *(M2 carry-forward)* generalize `MatrixKinematics::forward()` to the full `(AᵀA)⁻¹Aᵀ` pseudo-inverse
  for non-orthogonal drives (H-drive's off-center strafe wheel). Relaxes a precondition only — F5-safe.

**Definition of Done:** a trivial motion's kinematics produce identical numbers in a host gtest and on
the V5, swapping only `RobotContext`. **Freezes:** F4 ✅, F5 ✅ *(both host-frozen; the on-V5 number-match + `hal/pros` adapters await the toolchain)*.

---

### M2 — Real holonomic motion + dead-reckon localizer 🎯
*Move like a holonomic robot; know roughly where you are.*

**Control & FF (WS4)** — *started 2026-06-19 (host-testable, no robot needed)*
- [x] `Pid` (derivative-on-measurement, integral clamp, output clamp, injected clock). *`control/pid.hpp`
  + `test/pid_test.cpp` (9 cases): D-on-measurement (no setpoint kick), anti-windup back-calc, output
  clamp, first-call/dt≤0 P-only (no NaN), reset, config preconditions — mutation-checked (removing the
  anti-windup clamp and loosening the `dt>0` guard each go red). Bare-double by design; the motion layer
  owns unit consistency. `control/` added to the CI PROS-free guard.*
- [x] `Feedforward{kS,kV,kA}`; **voltage/brownout compensation**. *`control/feedforward.hpp` +
  `test/feedforward_test.cpp` (7 cases): `V = kS·sign(v) + kV·v + kA·a` (typed Velocity/Acceleration
  in, Voltage out; bare gains from sysid), kS follows the velocity sign and is zero at rest;
  `compensateForBattery()` limits a desired voltage to ±battery and flags brownout-saturation (the park
  fires as the battery collapses). Mutation-checked (kS-sign and the battery clamp each go red).*
- [x] `MotionProfile` (trapezoid). *`control/trapezoid_profile.hpp` + `test/trapezoid_profile_test.cpp`
  (7 cases): signed move → (pos, vel, accel) over time; accel/cruise/decel phases, triangular
  degradation when too short to reach maxVelocity, symmetry, clamped endpoints (arrives at rest),
  zero move, monotonic position. Mutation-checked (dropping the `√` in the triangular peak and the
  `½` in the accel integral each go red). Bare doubles; per-axis in the motion layer. (S-curve later.)*
- [x] `ExitCondition`/`ExitGroup` + **`SettledUtil`** (err **and** deriv **and** time-held). *`SettledUtil`
  done (`control/settled_util.hpp` + `test/settled_util_test.cpp`, 7 cases): settles only when |error| AND
  |error-rate| are within bounds AND held for `settleTime` (window opens on the first valid-rate tick);
  break-resets the window; clock-driven. Mutation-checked (dropping the rate condition lets it settle
  while still moving → red). `ExitGroup` (`control/exit_group.hpp` + `test/exit_group_test.cpp`) composes
  `SettledUtil` + `Watchdog` and reports the `ExitReason` (Settled/TimedOut/Running) for §18 exit-codes,
  Settled taking priority over a simultaneous timeout — mutation-checked (reversing the priority reds).*
- [x] **Motion watchdog** (hard timeout — a motion can never hang). *`control/watchdog.hpp` +
  `test/exit_group_test.cpp`: clock-driven, start()/expired()/elapsed()/reset(); expires at/after the
  timeout, never before start; mutation-checked (the `>=` boundary reds). Consumed by `ExitGroup`.*
- [ ] `tools/sysid` offline kS/kV/kA least-squares → emits **constants**; one on-robot ramp routine.

**Motion (WS6)**
- [x] `IMotion`; `MoveToPose` (decoupled x/y/θ), `TurnTo`, `StrafeTo`, `driveBrake`, `holdPose`.
  *Chunk C1 (2026-08-06): `include/shulib/motion/` — `IMotion` tick contract + `MotionState`
  wire vocabulary, the 3-axis decoupled engine (simultaneity mutation-proven), the
  wait-for-live-estimate contract (A3 handoff #1) and the `OdoStallCheck` spin-vs-motion
  cross-check → ODO_STUCK (A3 handoff #2), F1/F5 choreography explicit (norm-cap → fieldToRobot
  → authority clamp → toWheels → desaturate → FF → battery comp). Evidence:
  `test/motion_primitives_test.cpp` (21), `motion_frames_test.cpp` (7 — rotation/mirror
  equivariance + the ±180° seam incl. the F3 antipodal-CCW pin), `motion_sweep_test.cpp` (7 —
  seeded sweeps with per-tick universal invariants), `motion_hostile_test.cpp` (13 — per-family
  + composed settled-vs-truth divergence), `motion_routine_test.cpp` (4 — 5/10/20/40-move
  chains: clean error FLAT in count, hostile worst 4.1 in attributed to drift-vs-time),
  `motion_stall_check_test.cpp` (9). 58 cases; suite 487/858,611; 12 mutations red.*
- [x] `MotionScheduler` — one active motion, `async()`/`waitUntilSettled()`/`waitUntil(pred)`/`cancel()`.
  *Chunk C2, 2026-08-06, committed `1206dbe`: the formalized loop (localizer-first,
  bit-identical to the C1 hand loop — pinned by an equivalence test only the loop-shape mutation
  can red), ONE active slot with pinned PRE-EMPT semantics, `cancel()` into the defined safe
  state (0 V + Brake, synchronous — HA-53), the C1-deferred fault policy (abort-and-brake on new
  ODO_STUCK, continue-degraded elsewhere, configurable mask via the new
  `FaultLatch::raiseCount`), every wait bounded (explicit `waitUntil` timeouts, watchdog-bounded
  settle, the stalled-pacer guard), the check.hpp task-boundary catch, `activeCommandId`
  stamping (`CommandIdStampSink`, pair-rule preserving), idle-gap HealthMonitor + LoopMonitor
  ownership, `ExitReason::Cancelled`/`MotionState::Cancelled` appended. Routine accuracy through
  the scheduler REPRODUCES C1's baseline verbatim (bit-identity: clean n=5 finalErr 0.228 in ==
  C1; hostile worst 4.13 in; settle overhead 1.19 s/motion). Evidence:
  `include/shulib/motion/motion_scheduler.hpp`; `test/motion_scheduler_test.cpp` (25),
  `motion_scheduler_routine_test.cpp` (4), `fault_test.cpp` (+1); suite 527/859,931; 16
  mutations, all observed red.*

**Kinematics (WS3)**
- [x] `HDriveKinematics` — capped strafe authority + automatic turn-then-drive fallback
  (telemetry-visible). *Chunk C3, 2026-08-06: shipped as the `hDrive()` MatrixKinematics preset
  (the §13 #15 hybrid backend, like `xDrive()`), rows derived from rigid-body kinematics with an
  independent from-scratch test oracle. Authority = derivable speed ratio × HA-54 traction derate
  (0.35 default — the locked master-plan number, now a registered falsifiable claim). The
  fallback's shape is turn-WHILE-drive (authority-limited translation, rotation FREE — emergent
  from C1's clamp + decoupled loops; sequenced turn-then-drive REJECTED at engine level: it would
  break StrafeTo's held heading, invent unrequested mid-field headings, and is dominated by the
  simultaneous form — measured: aligned-heading MoveToPose 1.92 s beats the pure 21 in/s crab's
  2.71 s over the same 40 in). Visible end-to-end: `strafeFallbackActive` populated at the one
  producer, TermSink renders " SFB"; silent fallback = 6 independent failing tests. Carries the
  M1 deferral: `forward()` generalized to `(AᵀA)⁻¹Aᵀ` with previously-accepted tables
  bit-identical (XOR checksums vs the pre-C3 build) and a relDet conditioning guard. C1's D11
  CONFIRMED. Evidence: `include/shulib/kinematics/h_drive.hpp`, `matrix_kinematics.hpp`;
  `test/h_drive_test.cpp` (9), `matrix_kinematics_test.cpp` (+6), `motion_hdrive_test.cpp` (11),
  `motion_hdrive_routine_test.cpp` (3); suite 556/913,561; 13 mutations (12 red, 1
  explained-green redundancy probe, doc corrected). Same-auton accuracy X vs H: clean flat in
  count (0.225–0.238 in both), hostile worst 4.13 / 4.03 in.*

**Localization, tier 1 (WS5)**
- [x] `arcStep` (exact SE(2) constant-twist integrator) + `TrackingWheel` (role-stamped) + `PilonsOdometry`
  (IMU-owned heading, offset correction, trust/finiteness gate). Re-derived clean (caught a legacy
  average-vs-new-heading bug); forward-sim-verified, mutation-checked, 5-lens red-teamed.
- [x] `Localizer` on odom + **IMU-owned heading**, correctors stubbed; quality flag. EKF-ready seam
  (`IPoseSource`/`ICorrector`/`IFusionPolicy`); `ComplementaryFusion` gated nudge (accumulates + never
  snaps); fused `Twist2d` + measurable `quality()`. 43-agent red-team (caught + fixed the no-accumulation CRITICAL).
- [ ] IMU cold-calibrate at boot; per-boot bias characterization; tip detection (pitch/roll).

**Host sim plant & closed-loop harness (WS10/WS2)** — *added 2026-08-01 by chunk A2.
This task was the roadmap's incompleteness bug: this milestone's DoD (and M4's) requires "settles
within tolerance in **host sim**", and no task on this page built that sim — by this page's own
rule ("if something needs doing and isn't on this page, that's a bug in the roadmap"). With no
robot, it is the only means of validating ANY closed-loop behavior; every "settles in host sim"
DoD in Phases C–F depends on it.*
- [x] **Host plant + deterministic scenario harness** — voltage → wheel velocity (exact inversion of
  the existing `Feedforward` relation: kS dead band, τ=kA/kV lag; **kinematic, not dynamic** — no
  invented constants, gains provisional until R5/R6) → body twist (frozen-F5 `IKinematics::forward`)
  → TRUE pose (RK4 on unwrapped θ, **provably independent of `arcStep`** — reusing arcStep for truth
  would let any arcStep error cancel invisibly out of every Phase E measurement; a >π-per-tick
  tripwire test reds that mutation) → sensors synthesized from truth into the **unmodified F4
  fakes** (zero additive setters needed) → back into the estimator/controller. Plus: ground truth
  exposed to assertions only (new CI layering guard: core may never include `shulib/sim/`); seeded
  SplitMix64 byte-identical replay; the nine A3 degradation seams (empty, documented, proven live);
  per-tick `DebugRecord` emission honoring A1's `emitRecord` cost contract; `TermSink`-watchable.
  *Evidence: `include/shulib/sim/` (6 headers) + `test/sim_motor_model_test.cpp` (9 cases),
  `test/sim_truth_test.cpp` (9 — incl. the 405-point arcStep-vs-truth agreement sweep at 1e-9″ and
  the independence tripwire), `test/sim_plant_test.cpp` (14 — analytic open-loop, X-drive AND tank),
  `test/sim_closed_loop_test.cpp` (4 — the first closed loops in the project: converge/hold via the
  sensor path; sign-flip diverges; overdriven gain chatters at the derived instability threshold),
  `test/sim_odometry_truth_test.cpp` (5 — odometry/Localizer ≤1e-6″ of truth over multi-second runs;
  a 2% mis-calibration DETECTED at its predicted 1.0″), `test/sim_scenario_test.cpp` (9 — memcmp
  determinism, seam liveness, TermSink/NullSink cost). 48 new cases / 25,320 new assertions; suite
  349/547,443 green; **8 mutations proven red** (A2 completion record: development log, `shulib-v2` branch).*
- [x] **Hostile fakes (chunk A3)** — the nine seams populated with justified V5
  misbehaviour, each independently injectable AND composable (`ChainedDegradation`/`FullHostility`),
  hostility seeded (byte-identical replay pinned UNDER full hostility, closed-loop), every invented
  magnitude labelled PROVISIONAL for the A4 register. Every pathology raises a `FaultCode` with a
  safe fallback (`diag::HealthMonitor`, edge-triggered; `MotorOverTemp` appended) — no crash, no NaN
  in the pose (finiteness REQUIREd on every tick of every attack). **Three Localizer defects found
  by hostility and fixed at the source** (boot-window poisoning; ready-vs-data-path settle window;
  mid-run-loss misreport). *Evidence: `include/shulib/sim/hostile/` (7 headers) +
  `diag/health_monitor.hpp`; `test/sim_hostile_{imu,gps,encoder,power,slip,latency}_test.cpp`
  (8+9+7+8+7+7 cases), `test/sim_hostile_survival_test.cpp` (10 — the fault-discipline matrix),
  `test/sim_hostile_composed_test.cpp` (7 — composition/determinism/ablation/catastrophic),
  `test/health_monitor_test.cpp` (10), +4 boot-guard pins in `localizer_test.cpp`, +Gaussian pins in
  `sim_scenario_test.cpp`. Suite 429/681,086 green; **7 mutations proven red** (A3
  completion record: development log, `shulib-v2` branch).*
- [~] **The M2 `<1°` acceptance test** (`accuracy_spec_test.cpp` `[acceptance][M2]`) — **unskipped
  and live at A3** against modeled IMU drift/noise: worst end-of-60s heading error **0.912°** across
  10 seeded boots (cap 1.0°). `[~]` not `[x]` because the numbers it runs against are PROVISIONAL:
  at the pessimistic ±1°/min drift bound the margin is ~zero by construction (worst instantaneous
  error touched 1.065° mid-run), so the claim "the STACK adds no heading error of its own" is proven,
  while the field claim waits on R4's measured drift (the ceiling) and Phase E's correctors (the margin).
  *(That drift bound is now register entry HA-20 — the F2 ceiling has a tracked owner.)*
- [x] **Hardware Assumptions Register + ARM compile gate (chunk A4, closing Phase A)**
  — every claim about physical hardware the no-robot build rests on, inventoried:
  **[`docs/hardware-assumptions.md`](hardware-assumptions.md)**, **49 falsifiable
  entries** (33 invented / 13 reasoned / 2 measured-elsewhere / 1 mixed), each with source
  (file:line), confidence, the specific settling measurement, owning chunk (R3/R4/R5/R6), and
  blast radius if wrong — grouped as Phase R's walk-through checklist. **Bidirectional
  reconciliation grep-verified, zero orphans**: all 35 `PROVISIONAL (A4…)` label sites carry
  register IDs; all 46 in-tree IDs have entries (3 entries are exempt non-header sources, stated
  in-register). **CI now cross-compiles every v2 header for the V5's Cortex-A9** (the
  `arm-compile-gate` job: GENERATED header list so new headers are auto-covered; compile gate
  only — link/run stay R1/R3) and the gate is **proven**, not asserted: an injected x86-only
  construct left the host build GREEN and turned the gate RED (exit 1 at the offending line),
  then was restored. *Evidence: `hardware-assumptions.md`; `.github/workflows/ci.yml`; suite
  unchanged at 429/681,086 (no new test surface, by design); the A4 completion record, incl.
  the Phase A retrospective (development log, `shulib-v2` branch).*

**Facade (WS — Chassis)**
- [x] `Chassis` public verbs (F6): `moveTo`/`strafeTo`/`turnTo`/`followTrajectory`/`drive(ChassisSpeeds,Frame)`.
  *BUILT at chunk C4 (2026-08-10); **FROZEN at chunk D2 (2026-08-12) — F6 LOCKED, API 2.0.**
  `chassis/chassis.hpp`: the five verbs plus `brake()`/`hold(Time)`/`wait(Time)` (the C4
  candidates + D2's one sanctioned addition, all adopted into F6), `cancel()`/`waitUntil()`,
  pose/setPose/strafeAuthority/lastExitReason/lastCompleted/motionConfig, and the Tier-3 seam
  (`scheduler()`/`deps()`). Time is typed `units::Time` across the surface (the D2 retype —
  `hold(500)` does not compile; full-suite output proven byte-identical through the change).
  `drive()`'s `Frame` parameter is REQUIRED (frame confusion is a compile error — now an F6
  negative pin). One composition root makes id stamping structural (C2's gap closed,
  mutation-proven); facade routines BIT-IDENTICAL to the scheduler twin (clean + hostile), so
  C1–C3 baselines carry over verbatim; file-free plain-C++ construction tested (the §16.2
  standalone promise). The freeze is enforced: `test/f6_signature_pin_test.cpp` (36 compile-time
  pins, every member mutation-proven to fail the build naming F6) + `include/shulib/version.hpp`
  (the breaking-vs-additive policy). `Routine` deliberately stays OUTSIDE F6 until D3. Evidence:
  the C4 + D2 completion records (development log, `shulib-v2` branch); suite 690/916,050;
  D2's campaign: 44 mutations executed, 2 green holes found and closed (a noexcept-blind pin;
  a clock-blind call-site case).*

**Diagnostics & observability (WS13)** — *pulled forward so M2–M3 are debuggable as built (§18)*
- [x] `DebugRecord` per-tick snapshot schema, behind the `ITelemetrySink` seam (already at M1).
  *Done at chunk A1 (2026-08-01): `diag/debug_record.hpp` carries the **complete §18.2 field set**
  in typed units — including fields for systems that don't exist yet (gating residuals, covariance
  trace, `strafeFallbackActive`), since F9 later freezes this exact record. `emit()` added to the
  seam **additively** (non-pure, default no-op — pinned by the message-only-sink test) with the
  `wantsRecord()`/`emitRecord()` null-sink cost mechanism (a `NullSink` never even POPULATES a
  record — pinned by the builder-not-invoked test). Evidence: `test/debug_record_test.cpp` (8 cases,
  schema/type/wire-value pins) + 2 additivity cases in `test/telemetry_sink_test.cpp`;
  `FakeTelemetrySink` extended to record the emit channel. Mutation-checked (always-build
  `emitRecord` goes red).*
- [x] **`TermSink`** — readable, subsystem-tagged, column-aligned **terminal stream** (the primary debug surface); levels `ERROR/WARN/INFO/DEBUG/TRACE` with a **compile-time `TRACE` strip off the hot path** (zero-cost in competition builds).
  *Done at chunk A1: `diag/term_sink.hpp` with injected `IClock` + injected `hal::ICharSink`
  (output is golden-testable, not eyeballed) — exact §18.3-shape lines pinned byte-for-byte incl.
  the ugly cases (NaN/±Inf tokens, 1e300 compaction, empty tag, control-byte sanitization,
  UTF-8-safe truncation). `diag/trace.hpp` strips `SHULIB_TRACE` at compile time: argument
  expressions provably unevaluated (side-effect-counter test) AND the stripped call compiles to
  byte-identical ARM `-Os` code vs. no call at all (asm-diff verified). Evidence:
  `test/term_sink_test.cpp` (15 cases), `test/trace_strip_test.cpp` (2) +
  `test/trace_enabled_test.cpp` (1). Mutation-checked (a stray space in `[WARN]` and an
  args-evaluating strip each go red).*
- [x] **Fault-code enum** + latched first-fault; **motion exit-reason codes** on every `IMotion`; **loop-overrun / tick-timing** detection; NaN/Inf + invariant asserts (log-and-recover, non-fatal).
  *A1 delivered everything that exists to attach to: `diag/fault.hpp` (wire-stable numeric
  `FaultCode`, values pinned; `FaultLatch` retains the FIRST fault distinctly from the cascade and
  never crashes — survives even a throwing sink), `diag/loop_monitor.hpp` (dt-budget overrun, the
  `>= `boundary pinned exactly), `diag/finite_guard.hpp` (NaN/Inf log-and-recover with an
  unconditional finite-return guarantee), and the `check.hpp` §18.4 policy seam (host throws /
  robot routes to fault-log, call sites unchanged — `test/check_policy_test.cpp`, 5 cases).
  Evidence: `test/fault_test.cpp` (7), `test/loop_monitor_test.cpp` (7),
  `test/finite_guard_test.cpp` (8); first-fault latch, overrun boundary, and NaN guard all
  mutation-checked red. **Closed at C1:** every `IMotion` now reports an `ExitReason`
  (Settled/TimedOut, never Running after exit — pinned), raises `MOTION_TIMEOUT` on its latch,
  and carries the `MotionState` vocabulary in `DebugRecord.activeCommandState`
  (`test/motion_primitives_test.cpp` exit-discipline + legibility cases).*
- [x] Per-motion result line (target vs final · overshoot · drift · time · exit-reason) + end-of-run summary block.
  *Closed at C5 (2026-08-10): `diag/motion_result.hpp` (the §18.4 boundary vocabulary
  SETTLED/TIMEOUT/CANCELLED/FAULT_ABORT/SUPERSEDED, wire-pinned) + `diag/run_summary.hpp` +
  `TermSink::summarize` (the §18.3 block, byte-goldened), fed structurally by the scheduler's
  boundary-observer seam via `motion/run_reporter.hpp` — a routine cannot forget to report a
  boundary. Numbers cross-checked against plant TRUTH on X/H/tank (clean: reported == truth to
  ~1e-13 in; an inertial-plant case proves a REAL 3.79 in overshoot reports true to 6 digits;
  hostile: believed-vs-true ≤ 1.76 in, inside the C2/C3 envelope). D-1…D-5 landed in the same
  chunk with the F9 schema space reserved (diagnostics-plan.md discharge table);
  completion record: development log, `shulib-v2` branch.*
- [x] **Session header** (git build hash + routine id + alliance/side + port map + battery start) as the first record of every run — lets us compare/reproduce runs and confirm which binary ran.
  *Closed at C5: `diag/session_info.hpp` + `diag/build_info.hpp` — the hash is BUILD-SYSTEM
  injected (`git describe --always --dirty`; the test CMake does it; the PROS Makefile gains the
  same line at R1), and a missing hash is LOUD ([ERROR][SES] + the literal token MISSING), never
  a plausible placeholder — pinned by golden. Port map is caller-authored text at C5 (the core
  HAL has no port numbers); G1's RobotBuilder generates it. Battery start is READ, not typed.*
- [x] Fix the three inherited `logger.hpp` bugs (`escapeJSONString` unapplied, dead `sendDebugMessages`, racing flush) before building on it.
  *Resolved at chunk A1 by **clean-room supersession**, per the execution plan's "Explicitly rejected"
  note (re-derive, don't copy): nothing builds on `logger.hpp`, and the replacement designs each
  defect out **structurally** — sanitization is unavoidable by construction (one sanitizing append
  is the only path for caller text into a `TermSink` line), there are no dead paths (every shipped
  path is reached by a test), and the racing flush has no analogue (no background task, no shared
  mutable buffers; the concurrency contract is explicit in every header). The legacy files stay
  quarantined, reference-only, until the C7 deletion.*

**Legacy cutover (WS11)** — *the clean-room demolition, sequenced so nothing salvageable is lost*
- [x] **Salvage before deleting:** port `RobotCommands`→`sequence/` seed, `logger.hpp`→`io/Telemetry`,
  and re-derive the Pilons arc math into the in-core `PilonsOdometry` — all complete *before* the deletion.
  *Closed at C6 (2026-08-10), each item in a better form than this 2026-06 wording imagined
  (completion record: development log, `shulib-v2` branch): the Pilons math was re-derived clean at M2
  (`arcStep` — and C6 verified the legacy bug in source, `legacy odometry.cpp:338-355` rotates the
  chord by the NEW heading; the correct average-heading form is `arc_step.hpp:99`); `logger.hpp`
  was superseded structurally at A1/C5 (C6 re-verified all three defects by grep — `escapeJSONString`
  has zero call sites, `sendDebugMessages` is called but never defined, the flush race is real);
  and `RobotCommands` proved to be UNPORTABLE-BY-VACANCY — C6 found **no executor ever existed**
  (three data representations, zero consumers, an empty `moveTo()` body), so its salvage is
  [`legacy-command-vocabulary.md`](legacy-command-vocabulary.md): every id mapped to C1/C2, Phase F′,
  or G2, the two data specimens characterized for G4's importer, and the port list **empty by
  audit** (all 34 files classified; 11 live legacy bugs catalogued and left behind). C6-COMPLETED §8
  carries the unconditional safe-to-delete verdict the next bullet's deletion acts on.*
- [~] Rewire `main.cpp` + the PROS build onto the new core → **hardware-validate on the V5** → freeze
  F6 → **delete `src/legacy/` + `include/legacy/`**. After this the new `shulib/` is the only tree.
  *Structural half closed at C7 (2026-08-10) — with the sequencing deliberately changed from this
  2026-06 wording, a recorded execution-planning deviation: the "keep legacy until hardware
  validates" safety net was not real (the old tree no longer compiled, C6 had already salvaged
  everything worth keeping, and git retains every byte — recover via `git show 691c656:<path>`).
  What is DONE: `src/main.cpp` wires the v2 stack alone, its HAL seams fake-backed and marked
  TODO(R1) at each line an adapter replaces; `make` succeeds and produces the uploadable hot/cold
  package (`bin/hot.package.bin` + `bin/cold.package.bin`); `src/legacy/` + `include/legacy/` are
  deleted (34 files); the CI PROS-free guard covers all of `include/shulib/`; the new `shulib/` is
  the only tree. The **F6 freeze clause closed at D2 (2026-08-12)** — LOCKED, enumerated,
  pin-enforced. `[~]` not `[x]` because one clause is NOT done and is owned elsewhere:
  **hardware-validation on the V5 is OPEN (R3)** — this package has never driven a robot and
  cannot until R1's hal/pros adapters exist. (It has been **booted** on a V5 brain, 2026-08-12:
  upload → boot → full object graph constructed → diagnostics banner over USB, with every
  motor and sensor still fake. That closes the "has never executed on a V5" unknown and closes
  nothing else — R3's clause is about a robot that moves.)*

**Definition of Done:** a hand-written X-drive auton chains profiled motions and settles within
tolerance in host sim; the *same* auton runs the H-bot; **the run is legible in real time on the
terminal** (per-tick state + per-motion results + a run summary); **legacy is deleted and `main.cpp`
runs entirely on the new core**. **Freezes:** F6.

*Status at C7 (2026-08-10): every host-sim clause is met and the deletion is done. The final
clause is true STRUCTURALLY — the new core is the only thing `main.cpp` wires, and the package
compiles, links, and would boot — but "runs" has never been demonstrated on a V5: that is R3's
on-robot clause, still open. **The F6 freeze closed at D2 (2026-08-12)** — LOCKED in the
register above, pin-enforced. This milestone does not close until the on-robot clause does.*

---

### M3 — Accuracy edge: fusion + docking 🎯
*Bound drift across the whole minute; score sub-inch.*

**Localization, tier 2 (WS5)**
- [x] `GpsCorrector` — **DONE at E2 (2026-08-12)**, the first real corrector.
  `include/shulib/localization/gps_corrector.hpp`; 30 cases across
  `test/gps_corrector_test.cpp` (22), `gps_corrector_blackbox_test.cpp` (8 incl. the
  two-corrector case found by mutation) plus 7 in `gps_corrector_accuracy_test.cpp` and 7 new
  `[oracle]` cases in `test/gps_conversion_test.cpp`. Suite 794 / 1,081,382. Delivered: adaptive
  R from `rmsError()`, latency compensation via an odometry history ring, a stale-sample guard
  (one measurement folded once, against the ~50 ms camera cadence), high-yaw-rate rejection, a
  sensor-quality ceiling, and the **off-strip dead-reckon-only path proven bit-identical to
  having no corrector at all**, with `RejectedNoFix` and the source's name reaching the
  blackbox. 20/20 mutations red.
  **Scoped honestly, three ways.** (1) The gate is a **normalized innovation**, NOT a
  Mahalanobis distance — the complementary tier has no filter-estimated covariance, so
  `gateMahalanobis` stays 0 and `GateReason` gained `RejectedNormalizedInnovation` (+
  `RejectedStaleFix`, `RejectedSensorQuality`, all append-only). E4's EKF earns the real one.
  (2) **Lever-arm and frame reduction stayed at the HAL edge**, which `hal/gps.hpp` and
  `gps_conversion.hpp` already own by written contract — E2 pinned both with independent
  from-scratch oracles at seven headings instead of becoming a second owner. (3) The accuracy
  claim is an **aggregate** one: over 8 seeds of a 60 s hostile run, mean final and worst-case
  error are lower with the corrector (per-seed: 7/8 and 6/8, not 8/8), because in simulation the
  modeled GPS noise (HA-26) and the modeled dead-reckon drift (HA-20) are the same order — both
  invented, both R4's to settle. The claim that does not depend on any magnitude: a known 6-inch
  position error is **healed** on all 8 seeds while dead-reckoning carries it to the end of the
  run.
  **Also fixed there, per Rule 4:** HA-07's metres→inches obligation had lived in
  `gps_conversion.hpp` as prose with **no function and no test** since A4, and the pre-existing
  conversion tests pinned the scale against the constant imported from the header under test —
  so a wrong constant satisfied both sides. Now `gpsRmsErrorToCanonical()`, pinned against the
  definition of the inch.
- [x] `AprilTagCorrector` — **DONE at E3 (2026-08-13)**, the second corrector and the first
  absolute yaw correction. `include/shulib/localization/apriltag_corrector.hpp`,
  `localization/tag_map.hpp`, `hal/vision_conversion.hpp`; **73 new cases** across
  `test/vision_conversion_test.cpp` (15), `tag_map_test.cpp` (11), `apriltag_corrector_test.cpp`
  (23), `apriltag_corrector_heading_test.cpp` (15 — convergence, never-snap, the IMU re-stamp
  ordering, two correctors), `apriltag_corrector_blackbox_test.cpp` (6) and
  `apriltag_corrector_cost_test.cpp` (3). Suite 794 → **867 / 1,091,167**. 43/44 mutations red.
  Delivered: the corners→pose PnP as a **free pure function** (R2's adapter is the caller; the
  corrector does not include it), a provenance-enforcing tag map, best-of-N tag selection by
  estimated sigma, latency compensation in **position AND heading**, a trusted-range band, a
  detector-confidence floor, yaw-rate rejection, a normalized-innovation gate with anti-lockout
  widening, and yaw correction via the documented additive `headingNudge` path into a persistent
  bias in the `Localizer`. Three appended `GateReason` values (`RejectedNoTagMapEntry`,
  `RejectedTagRange`, `RejectedObservationAge`); `correctionDTheta` and `gateResidualHeading`,
  reserved since A1, are populated for the first time.
  **Scoped honestly, four ways.** (1) **The `< 1°` requirement is NOT claimed met.** What was
  measured is convergence and boundedness against **simulated truth, a simulated camera and
  invented noise** (HA-68…HA-82). (2) **shulib ships no tag map** — nobody here can cite one, and
  a map 2″ off yields a corrector that is *confidently* 2″ wrong, which no gate or filter can
  reveal. (3) **A reversed corner winding is silent** — 180° of heading error with the
  reprojection residual at machine zero; only a physical tag can catch it (HA-69, R2's to settle).
  (4) Heading gets a trusted-range **band** rather than its own noise model, because a second
  σ would be an invented number; E4's EKF is where `R_heading` belongs.
  **`propose()` is allocation-free and PINNED, not asserted:** the cost test replaces the global
  allocator and counts **zero** allocations across 20,000 ticks (and across 5,000 full
  `Localizer::update()` ticks with the heading path live), having first proved the counter works
  by requiring `poll()` — which touches the frozen by-value `tags()` seam — to be non-zero.
- [ ] Upgrade `Localizer` complementary filter → **5-state SE(2) EKF** `[px,py,θ,vx,vy]`:
  Mahalanobis gating, consecutive-reject re-init, process noise ∝ travel.
- [~] Innovation-bounded, covariance-weighted **gated nudge** (never snap); per-tick clamp; log every
  gating decision. **Partial: the never-snap mechanism and the logging are done for POSITION (C1,
  E1, E2) and now for HEADING (E3) — bounded per-tick increments, audited on `correctionDx/Dy`
  and `correctionDTheta`, every gating decision on the record. What is still missing is
  "covariance-weighted": the complementary tier weights by an invented confidence, not by a
  filter-estimated covariance. E4 owns that half.**
- [ ] **Calibration routines + persistence** (wheel scale/offset, GPS lever-arm, camera mount, IMU
  bias) — saved to SD/config.

**Alignment (WS7)**
- [ ] `alignment/DockToGoal` — visual-servo (AprilTag/poly-cutout), current/distance/pose confirm,
  height-adaptive fallback; **Distance-sensor fallback** path for no-tag.

**Diagnostics & observability (WS13)**
- [x] **`SdSink`** binary blackbox (versioned header + session/provenance record incl. **git
  build hash**; fixed-width per-tick; byte budget + drop-to-counter back-pressure; flush on
  auton-end) — the **no-laptop field record**. **DONE at E1 (2026-08-12)**:
  `include/shulib/diag/blackbox_format.hpp` (v1 layout: 256-byte header, typed frames, 428-byte
  tick record, all IEEE-754 binary64 — no narrowing), `blackbox_reader.hpp` (**the decoder ships
  with the encoder**: a format nothing can read is not a record), `sd_sink.hpp`, and the new
  binary seam `hal/block_sink.hpp` + `hal/fake/fake_block_sink.hpp`. Evidence:
  `test/blackbox_format_test.cpp` + `test/sd_sink_test.cpp` (round trip on ground truth,
  byte-exact per-field goldens, unknown version REFUSED, budget exhaustion drops-and-counts,
  truncated file decodes up to the cut and says so). **Deviation from this line, ruled
  explicitly:** "double-buffered **off-task** writes" was NOT built — C2/C4's standing "no
  background task" decision was kept, and writes are caller-paced (E1 completion record, T1).
  **The on-robot `/usd/` adapter is R1's**: E1 ships the seam and a host fake, nothing PROS.
- [~] **Estimator introspection** in the fusion DoD: per-correction residual + Mahalanobis distance +
  accept/reject reason; per-tick covariance trace (or trust weights) — the quantities that *certify* < 1°.
  **HALF at E1:** the PATH exists and is proven end to end (corrector → `GateAudit` on the fusion
  seam → `Localizer::lastCorrection().audit` → the record's §18.2 gating slots → the file → the
  decoder), with `ComplementaryFusion` filling what it genuinely knows (verdict, innovation,
  trust weight) and a **synthetic** corrector exercising every `GateReason`
  (`test/blackbox_introspection_test.cpp`). **The other half is E2/E3/E4's**: there are no real
  correctors yet, `gateMahalanobis` stays 0 until the EKF, and nothing here certifies < 1°.
- [x] Latched **brownout** marker + graceful-end contract — **DONE at E1**: the marker latches from
  the record stream or `markBrownout()` and reaches both the triage frame and the end frame; the
  graceful-end contract is defined concretely (what is written, in what order, and what a cut file
  looks like to the reader) and the truncated case is tested, because that is the case that will
  actually occur. *(The park-still-fires half of that sentence belongs to F2's guaranteed park.)*
- [x] **Flight recorder + fault triage** (diagnostics-plan D-6/D-7) — **DONE at E1**: a RAM ring,
  always on, dumped only on the FIRST fault, triage-first so a dump cut short by the brownout that
  caused it still says what broke; the same triage data prints at run end (`diag/triage.hpp`).

**Definition of Done:** pose error stays within F2 (notably **< 1°**) across a full 60s run with
contact and spins; docking nests a 1.6″ Pin repeatably. *(Recall F2's consequence: yaw correction here
is required, not optional, to hold < 1°.)*

---

### M4 — Skills layer + guaranteed park 🎯
*The scoring verbs, and the safety net that never fails to fire.*

**Manipulation (WS7)**
- [ ] `Mechanism` HAL abstraction.
- [ ] `setQuadrantToggle` (index N clicks on the 3-state Toggle + Optical color confirm) — *highest-value primitive*.
- [ ] `orientToScoringHalf` (yellow-side-out: color-sense → pass-through vs flip) — *second highest-value*.
- [ ] `intakeUntilCapture` (counter-roller, retry, sensor-confirm).
- [ ] `liftToLevel` (homing + linkage-aware mapping + sag-comp PID hold).
- [ ] `rotateClampToAngle` (profiled, no fling), `clampActuate`+`clampConfirm`, `deployActuator` (air-budget aware).
- [ ] Task-sensor confirmation on **every** grab/place (Optical/Distance/current) — never advance on failure.

**Sequencing (WS8)**
- [ ] `sequence/` Action engine: `Sequence`/`Parallel`/`Race`/`Deadline` + match-timer park guard.
  *(v1 may ship as hand-written blocking calls + one async handle + a wall-clock guard before the full
  combinator engine.)*
- [ ] **Time-budgeted Sequencer** — possession-aware; **guaranteed end-of-run action** (the +8 Midfield
  park and final Toggle re-verify fire on a hard schedule regardless of where the loop stalled).
- [ ] `buildStack`, `matchLoadCycle`, `endInMidfield` (18″ height lockout), `strategyMode(tallTower|fastCycle)`.

**Skills motion (WS6)**
- [ ] `fieldCentricStrafe`/`strafeTrim` (H-bot), `moveToPoseProfiled` (lift-state-aware accel).

**Definition of Done:** the two reference routines (X tall-tower, H Toggle-own + park) run end-to-end
in host sim; a deliberately stalled scoring loop still ends with the robot parked in the Midfield.

---

### M5 — Autonomy authoring + `.vexbot` ingestion 🎯
*From a project file to a running robot, no hand-wiring.*

**Config ingestion (WS9)**
- [ ] `IRobotConfig` + `RobotBuilder.from(profile)` → a fully wired `Chassis`.
- [ ] **`robotProfile` sub-schema** (F7) — drivetrain/odometry/sensors/mechanisms/corrections.
- [ ] **Codegen tool** `.vexbot` → `robot_config.hpp` (config **and** routines, `inline constexpr`).
- [ ] SD-card runtime loader (optional — re-export + re-run without recompiling).
- [ ] `inferDrivetrain()` fallback until VexBuilder emits explicit drivetrain fields.
- [ ] `schemaVersion` negotiation + additive migration (future `.vexbot` files still load).

**Path authoring (WS8)**
- [ ] `io/Trajectory`: read **`project.paths[]`** from `.vexbot`; **legacy `.shupaths` importer**
  (maps old `code_template` strings → command-ids best-effort, flags the rest).
- [ ] `PathRunner` — profiled per-segment execution + marker callbacks.
- [ ] **Command-id registry** `runner.on("intake_in", fn)` (F8) — the no-code keystone.

**Definition of Done:** a team member who cannot code builds a robot + a routine in VexBuilder,
exports one `.vexbot`, and the robot runs it. **Freezes:** F7, F8.

---

### M6 — Ecosystem: telemetry, sim seam, tuner 🔭
*See the robot; tune it without a laptop; run it in simulation.*

**Telemetry & tuning (WS10/WS13)** — *the `Shul2Sink` wire on top of the `DebugRecord`; `TermSink`/`SdSink` already shipped at M2/M3 (§18)*
- [ ] **`SHUL/2`** — the versioned, sequenced **wire serialization of `DebugRecord`** behind the same `ITelemetrySink` seam (F9).
- [ ] Run **record/replay** (for VexBuilder visualize/overlay).
- [ ] On-brain **live PID/FF tuner**; *(add-on)* on-brain HUD/summary screen.

**Sim seam (WS10/WS2)**
- [ ] `hal/sim` adapter speaking `SHUL/2` over VexBuilder's agent socket (`server.json` discovery);
  bidirectional — simulated sensors in, pose/twist/wheel-cmd/markers out.
- [ ] Planned-vs-actual overlay contract (renders when VexBuilder's Rapier sim lands).

**Definition of Done:** a real on-robot run streams to and **replays in VexBuilder**; a PID gain can be
tuned on the brain mid-session. **Freezes:** F9.

---

### M7 — Accessibility & docs 🔭
*Four tiers of use, no cliff; documentation that ships itself.*

**Progressive disclosure (WS12)**
- [x] **The user guide** *(item added at C8 — pulled ahead of this milestone the same way Phase
  D pulls the recipe API)*: `docs/guide/`, 15 numbered chapters + maintenance README, written
  for a reader with no robotics background; every code example compiled and executed by
  `test/guide_examples_test.cpp` (8 cases / 41 assertions, green 2026-08-11); tutorial verified
  by following it as written; guide↔README cross-linked. Honest scope: covers the Tier-3 C++
  API only (Tier 0–2 don't exist yet), and no genuinely-new reader has used it yet — M7's own
  DoD ("a brand-new member follows the guide without help") stays open.
- [x] **Recipe cookbook** *(chunk D3, 2026-08-12)*: `docs/cookbook/` — an index plus five
  task-shaped chapters, **fourteen recipes** written as a critical consumer of the recipe API
  rather than as a demo (a two-goal side run, bail-out on a failed grab, attempt-and-continue,
  partial-trajectory recovery, an alliance-partner wait, fitting the match window, a tank
  routine, budgeting a sideways leg on a limited-strafe drivetrain, mixed-tier authoring, and
  mid-routine re-seeding). Every listing is compiled and RUN against the plant in
  `test/cookbook_examples_test.cpp` (11 cases) and quoted verbatim; where a duration matters the
  case asserts against the **simulated clock**, with bounds calibrated from measured runs.
- [~] **Generated API docs** *(chunk D3, 2026-08-12)*: `docs/api/` is generated from the headers
  by `tools/api_doc_tool.py` — deterministic, committed, and verified up to date by a build-time
  regeneration check; a **doc-coverage gate** fails the build naming any public member of the F6
  or F10 surface that has no documentation (it found **16** on its first run, on the surface
  about to be frozen); a C++ fidelity pin (`test/api_reference_fidelity_test.cpp`) holds the
  rendered signatures to the headers. Marked `[~]`, not `[x]`, for one honest reason: **nothing
  is published.** The output is web-portable markdown that renders as-is on GitHub, and the
  publish path is written down — but this repository has one CI workflow and no Pages
  configuration, so "publish to the team website" remains open. Standing up hosting is
  infrastructure, not documentation.
- [~] Tier 2 **recipe API** — `chassis.moveTo(p).then(intake.in)…` (fluent, hard to misuse).
  *Built at chunk D1 (2026-08-11): `include/shulib/chassis/routine.hpp` — an eager fluent
  chain (`Routine`) whose every step delegates to one facade verb, with a tested
  stop/safe/skip/report policy on the first failed step. 14 cases in
  `test/chassis_recipe_test.cpp` + 3 guide examples (suite 681 / 916,003, green 2026-08-11);
  the recipe-vs-facade twin is bit-identical clean AND hostile, so C1–C4 baselines carry
  over verbatim. Marked `[~]`, not `[x]`: `.then(intake.in)` itself cannot exist until
  mechanisms do — `then()` is the labeled placeholder seam, and F1/F3 own the rest.
  **FROZEN at D3 (2026-08-12) as Freeze Register row F10**, after the cookbook became its second
  independent consumer and needed zero changes to the surface; `then()` is explicitly excluded
  from that freeze for the same reason it keeps the `[~]`.*
- [ ] "Your first auton in 10 minutes" guide (build → export → drag a path → run).
- [ ] Re-derive the kept Pilons arc math into the in-core odometry (rewrite cleanly, don't copy).

**Definition of Done:** a brand-new member follows the 10-minute guide to a running auton without
help; the API reference is live on the website.

*Status of that DoD, stated plainly: both clauses are OPEN.* The 10-minute guide starts in
VexBuilder and cannot honestly be written until G4 builds it. The API reference exists, is
generated and gated, and is **not live anywhere** — there is no site. And the human clause is
still human: C8 recorded that no genuinely-new reader had used the guide, and D3 cannot close
that either — nobody read the cookbook cold. It needs a real person, and it now has a named
owner rather than a hope (see "you are here").

---

### M8 — Second robot + coordination seam 🔭 *(stretch)*
- [ ] Harden the H-drive path end-to-end (it rides the same core — this is validation, not new core).
- [ ] A **thin coordination seam** (the stretch goal): a minimal interface so the two robots can share
  intent. Running shulib on both bots is just two independent instances — *that* is core, already done;
  this milestone is only the optional coordination layer on top.

**Definition of Done:** both robots field shulib autons in a skills run; the coordination seam exists
and is documented as stretch.

---

### Frontier 🔭 *(continuous — planned unlocks, not roadmap changes)*

Each maps to an ○ cell in the [Capability Catalog](shulib-v2-master-plan.md#15-the-one-stop-shop-capability-catalog-past--present--future):
- **Drivetrain:** Mecanum + Swerve scaffolds; per-wheel slip model; auto-ID drivetrain from VexBuilder
  geometry; online wheel-radius calibration.
- **Motion:** dynamic replanning around detected obstacles; time-optimal profiling; min-velocity
  handoff chaining (v1 ships stop-and-settle).
- **Localization:** LIDAR scan-match corrector; Pi/Coral-fused absolute pose; ZUPT.
- **Control:** adaptive gains; learned feedforward from logged runs; learned-friction tables.
- **Manipulation:** vision-servo grasp; closed-loop stack verification.
- **Path/sim:** in-lib spline/Squiggles-style smoothing; full Rapier physics round-trip; live
  edit-path → re-sim; binary compiled-path artifact + schema-hash handshake.
- **Telemetry/diagnostics:** on-brain HUD/summary screen (add-on); cloud run library; auto-tune from replays.
- **Authoring:** a GUI sequence builder in VexBuilder writing `paths[]` directly.
- **Follow mode (off-field convenience + demo):** a `FollowTarget` `IMotion` that trails a **carried
  AprilTag** at a set standoff, so the robot walks itself to and from the field instead of being
  carried. *Scoped deliberately to a tag, not a person:* detecting a human is easy, but deciding
  **which** human is you (re-identification) is unreliable in a crowd — a tag is a unique id with a
  computable bearing **and** range, which makes the crowd case disappear. Reuses M3's `AprilTagCorrector`
  pipeline pointed at a different target; rides `ITagSource`, so either the V5 AI Vision (native tags
  since kernel 4.2.2) or the Pi backend serves it. **Holonomic advantage:** an X-drive strafes to keep
  the camera on the target while sidestepping, where a tank bot must turn away and lose it.
  *Generic person-following* (no tag) is a further unlock needing a Pi-side person model.
  *Obstacle avoidance* around people is a separate unlock — Distance-sensor array or LIDAR (`<VUG3>`),
  and it composes with the **Motion** row's dynamic-replanning cell above rather than duplicating it.
  **Non-negotiable if built:** a controller **dead-man** (hold-to-follow, release-to-stop) and a check
  of event rules on autonomous operation outside the field — a robot driving itself through a crowded
  venue is a real hazard. **Honest framing:** driver control already solves "don't carry the robot";
  this earns its place as an *Innovate/Design demo* that exercises the same vision + estimator stack
  the competition code depends on, not as a labor saver. **Earliest sensible slot: after M3**, once the
  tag pipeline exists.

---

## Workstreams

The same backlog, grouped by the 12 permanent capability areas. These names are stable; the roadmap
grows by adding rows here, never by renaming these.

| # | Workstream | Owns | First lands |
|---|---|---|---|
| **WS1** | Foundation & conventions | frame, units, `Angle`, `Pose2d`/`Twist2d`, the one transform | M0 |
| **WS2** | Hardware Abstraction Layer | all `I*` interfaces, `hal/pros` · `hal/fake` · `hal/sim`, `RobotContext` | M1 |
| **WS3** | Math & kinematics | `IKinematics`, X/H/Tank, desaturation, (○ mecanum/swerve, slip) | M1 |
| **WS4** | Control & feedforward | `Pid`, `Feedforward`, `MotionProfile`, `SettledUtil`, watchdog, brownout comp, sysid | M2 |
| **WS5** | Localization & fusion | Pilons odom, `Localizer`, GPS/AprilTag correctors, complementary→EKF, calibration | M2→M3 |
| **WS6** | Motion & scheduling | `IMotion`, MoveToPose/TurnTo/StrafeTo/Follow, `MotionScheduler`, skills motion | M2 |
| **WS7** | Manipulation & skills | `Mechanism` HAL, alignment/docking, the scoring primitives | M3→M4 |
| **WS8** | Autonomy authoring | `paths[]` reader + importer, `PathRunner`, command registry, Sequencer/park guard | M4→M5 |
| **WS9** | Config & hardware ingestion | `IRobotConfig`, `RobotBuilder`, `robotProfile`, codegen, versioning | M5 |
| **WS10** | Sim, telemetry & tuning | `SHUL/2`, record/replay, `hal/sim` wire, live tuner, overlay | M6 |
| **WS11** | Tooling, build & CI | kernel bump, host-test harness, CI, clean-room layout, wrapper vendoring | M0 |
| **WS12** | Docs & onboarding | tiers, recipe cookbook, generated docs site, onboarding guide | M0→M7 |
| **WS13** | Diagnostics & observability | `DebugRecord` + sinks (`TermSink`/`SdSink`/`Shul2Sink`), fault codes, exit reasons, loop-overrun, provenance (§18) | M2 |

---

## Cross-team asks (VexBuilder)

shulib *defines* the contracts; VexBuilder *produces* them. These are the things the VexBuilder side
must add for the integration to close. They are tracked here so the dependency is never invisible.

1. **Add `project.paths[]`** to the `.vexbot` schema (routines now live in the project file, not the
   retired `.shupaths`). → unblocks F8, M5.
2. **Add explicit drivetrain fields** (`kind` / `trackWidth` / `wheelDiameter`) to the robot config so
   shulib doesn't have to *infer* them from part geometry (brittle). → firms up F7, M5.
3. **Expose the agent socket for `SHUL/2`** (already discoverable via `server.json`) and, when the
   Rapier sim lands, feed simulated sensors in / render pose out. → M6.
4. **Consume the shulib command-id manifest** to populate VexBuilder's command picker, so authored
   `paths[]` only reference ids shulib actually handles. → unblocks F8, M5.

---

## Decisions still open

Almost everything is locked (see the [master plan's decision table](shulib-v2-master-plan.md#13-open-decisions--tbd)).
What remains is **hardware**, not software, and does not block the software milestones:

- **Build-team calls:** final robot roles, mechanisms, lift type (cascade vs DR6B), goal-localization
  method per robot, Toggle-defense approach. These shape the *content* of the M4 routines, not the
  *library*.

Everything in the [Open Decisions table](shulib-v2-master-plan.md#13-open-decisions--tbd) marked
"Recommended-lock" is treated as locked for roadmap purposes; it will be formally stamped at the
milestone where its workstream first lands.

---

## Principles we won't trade away

1. **Accurate first.** Every feature is measured against a numeric accuracy target (F2), not a vibe.
2. **Usable by non-coders.** If a future team can't field an auton without us, we failed.
3. **Standalone, then ecosystem.** shulib works with nothing else installed; VexBuilder makes it
   better, never required.
4. **You decide, tools execute.** The library and its tools carry out the decisions you make — they
   never make them for you.
5. **Freeze, don't break.** Contracts in the Freeze Register change only by versioned migration — so
   this roadmap stays true.

---

*shulib is built by the Seton Hall VEX U team. This roadmap is structured to stay accurate as we
build: workstreams and the Freeze Register are stable; only status badges move. Questions or want to
contribute? Talk to the programming chair.*
