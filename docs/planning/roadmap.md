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
| F6 | **Public `Chassis` API** — `moveTo`/`strafeTo`/`turnTo`/`followTrajectory`/`drive(ChassisSpeeds,Frame)` | Every auton ever written on shulib | M2 | 🎯 |
| F7 | **`robotProfile` sub-schema** inside `.vexbot` — drivetrain/odometry/sensors/mechanisms/corrections | Config codegen; every robot file | M5 | 🎯 *(coordinate with VexBuilder)* |
| F8 | **`paths[]` sub-schema + command-id vocabulary** inside `.vexbot` | Every data-driven routine | M5 | 🎯 *(coordinate with VexBuilder)* |
| F9 | **`SHUL/2` telemetry wire protocol** (v1) — the wire serialization of `DebugRecord` (§18) | Sim, record/replay, tuner, VexBuilder overlay; **every sink (`TermSink`/`SdSink`/`Shul2Sink`) shares the `DebugRecord` schema** | M6 | 🎯 |

---

## Milestones at a glance

> **You are here:** **M1 complete; M2 control + localization complete; Phase A COMPLETE
> (2026-08-06); Phase C OPEN — chunk C1 (`IMotion` + the motion primitives) built and verified
> 2026-08-06, in the working tree pending review.** The library can now be told "go to that
> spot": `MoveToPose` (decoupled per-axis x/y/θ — the holonomic thesis, mutation-proven),
> `TurnTo`, `StrafeTo`, `HoldPose`, `DriveBrake`, all watchdog-bounded, hostile-surviving, and
> honoring A3's two handoffs (wait-for-live-estimate; the ODO_STUCK spin-vs-motion cross-check).
> **Next: chunk C2 — `MotionScheduler`.** (There is no Phase B: the original hardware phase
> became Phase R — see build-order's deviations table.)
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
> proven red, then restored.** See `docs/planning/chunks/A1-COMPLETED.md` for the full record.
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
> `shulib/sim/`). **8 mutations proven red, then restored.** See
> `docs/planning/chunks/A2-COMPLETED.md` for the full record.
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
> restored.** See `docs/planning/chunks/A3-COMPLETED.md` for the full record.
> **Chunk A4 (Hardware Assumptions Register + ARM compile gate) is done — 2026-08-06, closing
> Phase A:** every claim about physical hardware that three no-robot chunks (plus the M1/M2
> conversion layers) rest on is now **inventoried instead of scattered** —
> **[`docs/planning/hardware-assumptions.md`](hardware-assumptions.md)**, 49 falsifiable entries
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
> See `docs/planning/chunks/A4-COMPLETED.md` (incl. the Phase A retrospective).
> **NEXT: chunk C2 — `MotionScheduler`** (C1 closed 2026-08-06 — see
> `docs/planning/chunks/C1-COMPLETED.md`). The *what* is still this page; the **order** lives in
> **[`build-order.md`](build-order.md)** — 39 dependency-ordered chunks, written against the governing
> constraint that **there is no robot yet**. Read it before starting any work.
> *Carry-overs (tracked, now placed): `hal/pros` adapters + v2 `src/main.cpp` → R1/R3; `MatrixKinematics`
> non-orthogonal pseudo-inverse → C3; `sysid` → R5; estimator-side frozen-encoder detection → E-phase
> (the loop-level cross-check shape is tested at A3).*
> *Status verified 2026-08-06 (post-C1): host suite **487 cases / 858,611 assertions green** under
> strict `-Werror` (3 deliberately skipped: two M3 acceptance stubs + the R3 GPS field-cal oracle
> = register entry HA-01); both CI guards green with **`include/shulib/motion` added to both
> scopes**; all **85** v2 headers cross-compile clean for ARM under the CI `arm-compile-gate`
> job (generated list — the 8 new motion headers are covered automatically). C1 ran **12
> mutations, all observed red** (two only after the suite was strengthened — the two holes they
> exposed are closed and recorded in C1-COMPLETED §Mutations).*

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
  in-flux until the M1/M2 wiring, as planned.*

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
- [ ] `MotionScheduler` — one active motion, `async()`/`waitUntilSettled()`/`waitUntil(pred)`/`cancel()`.

**Kinematics (WS3)**
- [ ] `HDriveKinematics` — capped strafe authority + automatic turn-then-drive fallback (telemetry-visible).

**Localization, tier 1 (WS5)**
- [x] `arcStep` (exact SE(2) constant-twist integrator) + `TrackingWheel` (role-stamped) + `PilonsOdometry`
  (IMU-owned heading, offset correction, trust/finiteness gate). Re-derived clean (caught a legacy
  average-vs-new-heading bug); forward-sim-verified, mutation-checked, 5-lens red-teamed.
- [x] `Localizer` on odom + **IMU-owned heading**, correctors stubbed; quality flag. EKF-ready seam
  (`IPoseSource`/`ICorrector`/`IFusionPolicy`); `ComplementaryFusion` gated nudge (accumulates + never
  snaps); fused `Twist2d` + measurable `quality()`. 43-agent red-team (caught + fixed the no-accumulation CRITICAL).
- [ ] IMU cold-calibrate at boot; per-boot bias characterization; tip detection (pitch/roll).

**Host sim plant & closed-loop harness (WS10/WS2)** — *added 2026-08-01 by build-order chunk A2.
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
  349/547,443 green; **8 mutations proven red** (see `chunks/A2-COMPLETED.md`).*
- [x] **Hostile fakes (build-order chunk A3)** — the nine seams populated with justified V5
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
  `sim_scenario_test.cpp`. Suite 429/681,086 green; **7 mutations proven red** (see
  `chunks/A3-COMPLETED.md`).*
- [~] **The M2 `<1°` acceptance test** (`accuracy_spec_test.cpp` `[acceptance][M2]`) — **unskipped
  and live at A3** against modeled IMU drift/noise: worst end-of-60s heading error **0.912°** across
  10 seeded boots (cap 1.0°). `[~]` not `[x]` because the numbers it runs against are PROVISIONAL:
  at the pessimistic ±1°/min drift bound the margin is ~zero by construction (worst instantaneous
  error touched 1.065° mid-run), so the claim "the STACK adds no heading error of its own" is proven,
  while the field claim waits on R4's measured drift (the ceiling) and Phase E's correctors (the margin).
  *(That drift bound is now register entry HA-20 — the F2 ceiling has a tracked owner.)*
- [x] **Hardware Assumptions Register + ARM compile gate (build-order chunk A4, closing Phase A)**
  — every claim about physical hardware the no-robot build rests on, inventoried:
  **[`docs/planning/hardware-assumptions.md`](hardware-assumptions.md)**, **49 falsifiable
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
  unchanged at 429/681,086 (no new test surface, by design); `chunks/A4-COMPLETED.md` (incl.
  the Phase A retrospective).*

**Facade (WS — Chassis)**
- [ ] `Chassis` public verbs (F6): `moveTo`/`strafeTo`/`turnTo`/`followTrajectory`/`drive(ChassisSpeeds,Frame)`.

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
- [ ] Per-motion result line (target vs final · overshoot · drift · time · exit-reason) + end-of-run summary block. *→ chunk C5 (needs motion data that doesn't exist yet).*
- [ ] **Session header** (git build hash + routine id + alliance/side + port map + battery start) as the first record of every run — lets us compare/reproduce runs and confirm which binary ran. *→ chunk C5.*
- [x] Fix the three inherited `logger.hpp` bugs (`escapeJSONString` unapplied, dead `sendDebugMessages`, racing flush) before building on it.
  *Resolved at chunk A1 by **clean-room supersession**, per build-order's "Explicitly rejected"
  note (re-derive, don't copy): nothing builds on `logger.hpp`, and the replacement designs each
  defect out **structurally** — sanitization is unavoidable by construction (one sanitizing append
  is the only path for caller text into a `TermSink` line), there are no dead paths (every shipped
  path is reached by a test), and the racing flush has no analogue (no background task, no shared
  mutable buffers; the concurrency contract is explicit in every header). The legacy files stay
  quarantined, reference-only, until the C7 deletion.*

**Legacy cutover (WS11)** — *the clean-room demolition, sequenced so nothing salvageable is lost*
- [ ] **Salvage before deleting:** port `RobotCommands`→`sequence/` seed, `logger.hpp`→`io/Telemetry`,
  and re-derive the Pilons arc math into the in-core `PilonsOdometry` — all complete *before* the deletion.
- [ ] Rewire `main.cpp` + the PROS build onto the new core → **hardware-validate on the V5** → freeze
  F6 → **delete `src/legacy/` + `include/legacy/`**. After this the new `shulib/` is the only tree.

**Definition of Done:** a hand-written X-drive auton chains profiled motions and settles within
tolerance in host sim; the *same* auton runs the H-bot; **the run is legible in real time on the
terminal** (per-tick state + per-motion results + a run summary); **legacy is deleted and `main.cpp`
runs entirely on the new core**. **Freezes:** F6.

---

### M3 — Accuracy edge: fusion + docking 🎯
*Bound drift across the whole minute; score sub-inch.*

**Localization, tier 2 (WS5)**
- [ ] `GpsCorrector` — adaptive R from `get_error()`, lever-arm + latency comp, Mahalanobis gate,
  high-yaw-rate rejection, **off-strip dead-reckon-only flag** (Driving Skills has no strip).
- [ ] `AprilTagCorrector` — tags 0–4, PnP; `relocalize()` **feeds the gated-nudge corrector** (low-R,
  fast, drift-canceling — *never a hard pose reset*, per §13 #4).
- [ ] Upgrade `Localizer` complementary filter → **5-state SE(2) EKF** `[px,py,θ,vx,vy]`:
  Mahalanobis gating, consecutive-reject re-init, process noise ∝ travel.
- [ ] Innovation-bounded, covariance-weighted **gated nudge** (never snap); per-tick clamp; log every
  gating decision.
- [ ] **Calibration routines + persistence** (wheel scale/offset, GPS lever-arm, camera mount, IMU
  bias) — saved to SD/config.

**Alignment (WS7)**
- [ ] `alignment/DockToGoal` — visual-servo (AprilTag/poly-cutout), current/distance/pose confirm,
  height-adaptive fallback; **Distance-sensor fallback** path for no-tag.

**Diagnostics & observability (WS13)**
- [ ] **`SdSink`** binary blackbox to `/usd/` (versioned header + session/provenance record incl. **git
  build hash**; fixed-width per-tick; double-buffered off-task writes; byte/tick budget + drop-to-counter
  back-pressure; flush on auton-end) — the **no-laptop field record**.
- [ ] **Estimator introspection** in the fusion DoD: per-correction residual + Mahalanobis distance +
  accept/reject reason; per-tick covariance trace (or trust weights) — the quantities that *certify* < 1°.
- [ ] Latched **brownout** marker + graceful-end contract (the scheduled park still fires as the battery collapses).

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
- [ ] Tier 2 **recipe API** — `chassis.moveTo(p).then(intake.in)…` (fluent, hard to misuse).
- [ ] "Your first auton in 10 minutes" guide (build → export → drag a path → run).
- [ ] Recipe cookbook.
- [ ] **Generated API docs** published to the team website (this roadmap + the capability catalog are
  already web-portable).
- [ ] Re-derive the kept Pilons arc math into the in-core odometry (rewrite cleanly, don't copy).

**Definition of Done:** a brand-new member follows the 10-minute guide to a running auton without
help; the API reference is live on the website.

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
