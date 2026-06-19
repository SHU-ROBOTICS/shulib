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
| F3 | **Units & `Angle` semantics** — internal inches + radians + **seconds**; degrees only at the API edge; one wrap type normalized to `(-π,π]`; `shortestError(a,b)==wrap(b-a)` with the exact-180° case → **+π** (not −π), pinned by a red-on-failure test | Every numeric API signature | M0 | 🔨 |
| F4 | **HAL interface signatures** — `IMotor`/`IRotation`/`IImu`/`IGps`/`IVision`/`IDistance`/`IOptical`/`IClock`/`ITelemetrySink`/`IRobotConfig`/`IRouteSource` | All three runtime targets (robot/sim/test) | M1 | 🎯 |
| F5 | **`IKinematics` contract** — twist `(vx,vy,ω)` ⇄ wheels + desaturate + `strafeAuthority()` (a **pure read-only query** = max sustainable \|vy\|/\|vx\|; the motion layer clamps, kinematics never clamps inside `toWheels()` — §13 #5) | All motion code; new drivetrains | M1 | 🎯 |
| F6 | **Public `Chassis` API** — `moveTo`/`strafeTo`/`turnTo`/`followTrajectory`/`drive(ChassisSpeeds,Frame)` | Every auton ever written on shulib | M2 | 🎯 |
| F7 | **`robotProfile` sub-schema** inside `.vexbot` — drivetrain/odometry/sensors/mechanisms/corrections | Config codegen; every robot file | M5 | 🎯 *(coordinate with VexBuilder)* |
| F8 | **`paths[]` sub-schema + command-id vocabulary** inside `.vexbot` | Every data-driven routine | M5 | 🎯 *(coordinate with VexBuilder)* |
| F9 | **`SHUL/2` telemetry wire protocol** (v1) — the wire serialization of `DebugRecord` (§18) | Sim, record/replay, tuner, VexBuilder overlay; **every sink (`TermSink`/`SdSink`/`Shul2Sink`) shares the `DebugRecord` schema** | M6 | 🎯 |

---

## Milestones at a glance

> **You are here:** **M0 — Foundation & scaffolding: COMPLETE & verified.** Math
> (`Angle`/`units`/geometry/frame, F1/F2/F3), tooling (kernel **4.2.2** + AprilTag, **CI**,
> `compile_commands.json`), legacy quarantined to `legacy/`, `tracking.md` deleted. Host suite: 26
> cases / 520k assertions, mutation-checked. *(One follow-up: the on-robot ARM **link** awaits PROS's
> toolchain — tracked.)* **Next: M1 — HAL + kinematics.** *Updated 2026-06-19.*

| Milestone | Theme | DoD headline | Status |
|---|---|---|---|
| **M0** | Foundation & scaffolding | Frame frozen in a test; host-test harness + CI green; repo clean-room-ready | 🔨 |
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

### M0 — Foundation & scaffolding 🔨
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
  addable later. Guard scope broadens to all of `shulib/` after the M2 cutover.*
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
**Freezes:** F1 ✅, F2 ✅, F3.

---

### M1 — HAL + kinematics foundation 🎯
*One seam for hardware/sim/test; drivetrains become interchangeable.*

**HAL (WS2)**
- [ ] Define all HAL interfaces (F4): `IMotor`, `IRotation`, `IImu`, `IGps`, `IVision`/`ITagSource`,
  `IDistance`, `IOptical`, `IClock`, `ITelemetrySink`, plus a battery-voltage source.
- [ ] `hal/pros/*` adapters — the **only** files that include `<pros/*>`. IMU compass/CW → canonical;
  GPS frame → canonical (the conversions happen here, once).
- [ ] Vendor & extend the existing `ai_vision.hpp` wrapper into the `hal/pros` `IVision`/`ITagSource`
  adapter (object mode **and** AprilTag mode).
- [ ] `hal/fake/*` deterministic doubles (injectable clock) for host tests.
- [ ] `RobotContext` — the DI container; the one object that differs across robot/sim/test.

**Kinematics (WS3)**
- [ ] `IKinematics` contract: `toWheels`/forward/inverse/desaturate/`strafeAuthority()` (F5).
- [ ] `XDriveKinematics`, `TankKinematics` — host-tested with pure numbers.
- [ ] Uniform-scale wheel desaturation.

**Definition of Done:** a trivial motion's kinematics produce identical numbers in a host gtest and on
the V5, swapping only `RobotContext`. **Freezes:** F4, F5.

---

### M2 — Real holonomic motion + dead-reckon localizer 🎯
*Move like a holonomic robot; know roughly where you are.*

**Control & FF (WS4)**
- [ ] `Pid` (derivative-on-measurement, integral clamp, output clamp, injected clock).
- [ ] `Feedforward{kS,kV,kA}`; **voltage/brownout compensation** (scale by measured V).
- [ ] `MotionProfile` (trapezoid).
- [ ] `ExitCondition`/`ExitGroup` + **`SettledUtil`** (err **and** deriv **and** time-held).
- [ ] **Motion watchdog** (hard timeout — a motion can never hang).
- [ ] `tools/sysid` offline kS/kV/kA least-squares → emits **constants**; one on-robot ramp routine.

**Motion (WS6)**
- [ ] `IMotion`; `MoveToPose` (decoupled x/y/θ), `TurnTo`, `StrafeTo`, `driveBrake`, `holdPose`.
- [ ] `MotionScheduler` — one active motion, `async()`/`waitUntilSettled()`/`waitUntil(pred)`/`cancel()`.

**Kinematics (WS3)**
- [ ] `HDriveKinematics` — capped strafe authority + automatic turn-then-drive fallback (telemetry-visible).

**Localization, tier 1 (WS5)**
- [ ] `PilonsOdometry` (arc math as a pure function; diagnostics off the hot path).
- [ ] `Localizer` on odom + **IMU-owned heading**, correctors stubbed; quality flag.
- [ ] IMU cold-calibrate at boot; per-boot bias characterization; tip detection (pitch/roll).

**Facade (WS — Chassis)**
- [ ] `Chassis` public verbs (F6): `moveTo`/`strafeTo`/`turnTo`/`followTrajectory`/`drive(ChassisSpeeds,Frame)`.

**Diagnostics & observability (WS13)** — *pulled forward so M2–M3 are debuggable as built (§18)*
- [ ] `DebugRecord` per-tick snapshot schema, behind the `ITelemetrySink` seam (already at M1).
- [ ] **`TermSink`** — readable, subsystem-tagged, column-aligned **terminal stream** (the primary debug surface); levels `ERROR/WARN/INFO/DEBUG/TRACE` with a **compile-time `TRACE` strip off the hot path** (zero-cost in competition builds).
- [ ] **Fault-code enum** + latched first-fault; **motion exit-reason codes** on every `IMotion`; **loop-overrun / tick-timing** detection; NaN/Inf + invariant asserts (log-and-recover, non-fatal).
- [ ] Per-motion result line (target vs final · overshoot · drift · time · exit-reason) + end-of-run summary block.
- [ ] **Session header** (git build hash + routine id + alliance/side + port map + battery start) as the first record of every run — lets us compare/reproduce runs and confirm which binary ran.
- [ ] Fix the three inherited `logger.hpp` bugs (`escapeJSONString` unapplied, dead `sendDebugMessages`, racing flush) before building on it.

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
