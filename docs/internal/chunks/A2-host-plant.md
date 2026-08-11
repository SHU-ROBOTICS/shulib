# Chunk A2 — Host plant + closed-loop sim harness

> **Phase A, chunk 2 of 39.** The chunk the roadmap forgot, and the one the rest of the project
> depends on. See [`build-order.md`](../build-order.md) § "The missing prerequisite".

**Workstream:** WS10/WS2 (adjacent) · **Milestone:** enables M2's and M4's DoDs · **Predecessor:** A1 ✅

---

## Why this chunk exists at all

`roadmap.md`'s M2 Definition of Done requires *"a hand-written X-drive auton chains profiled motions
and settles within tolerance in **host sim**"* (line 379); M4's DoD requires the same (line 442).
**No task in the roadmap builds that host sim.** `hal/sim` at M6 does not fill the gap — it bridges to
VexBuilder's Rapier engine, consuming an external physics model rather than providing one.

Today `FakeMotor` records a commanded voltage and lets a test inject an encoder value by hand. Nothing
converts voltage into motion, so **no closed loop exists anywhere in this project.** Motion, fusion,
and sequencing are all closed-loop.

**And there is no robot.** So this harness is not a convenience — it is the only means by which any
closed-loop behavior in shulib can be validated at all, until hardware exists. Every chunk in Phases
C, D, E, and F is proven or disproven by what you build here.

**The upside:** in simulation you possess **ground truth** — the robot's exact pose at every tick,
which a physical field can never give you. That is what makes Phase E's estimator work provable.

---

## What already exists (build on it, don't re-invent)

| Thing | Where | Use it for |
|---|---|---|
| `MatrixKinematics`, `xDrive()`, `tank()` | `include/shulib/kinematics/` | twist ⇄ wheels; the plant must agree with it |
| `desaturateUniform` | `kinematics/desaturate.hpp` | over-budget wheel scaling |
| `arcStep` (exact SE(2) integrator) | `localization/arc_step.hpp` | **the odometry's integrator — see Constraint 2, do NOT reuse it for truth** |
| `Feedforward{kS,kV,kA}` | `control/feedforward.hpp` | the voltage⇄velocity relation to invert |
| `Pid` | `control/pid.hpp` | the closed-loop DoD case |
| `FakeClock` (monotonic, injectable) | `hal/fake/fake_clock.hpp` | drive time; never use wall-clock |
| `FakeMotor`/`FakeImu`/`FakeGps`/`FakeRotation` | `hal/fake/` | **drive these, do not replace them** |
| `PilonsOdometry`, `TrackingWheel`, `Localizer` | `localization/` | the consumers whose error you will measure |
| `Pose2d`, `Twist2d`, units | `math/`, `units/` | typed values throughout |
| **`diag/` (new, chunk A1)** | `include/shulib/diag/` | `DebugRecord`, `TermSink`, faults — make the sim legible |

**Read first:** `docs/internal/build-order.md` (Phase A, and "The missing prerequisite"),
`docs/internal/chunks/A1-COMPLETED.md` (what A1 just built and its decision log), and the house style
in `include/shulib/localization/localizer.hpp` and `include/shulib/control/pid.hpp`.

**Conventions:** header-only under `include/shulib/`, `#pragma once`, header comments that explain
*why*. Tests are `test/<name>_test.cpp` (CMake globs them). Strict flags: `-Wall -Wextra -Wpedantic
-Werror -Wshadow -Wconversion -Wsign-conversion -Wdouble-promotion`. **PROS-free** — CI enforces it,
and add any new directory to the guard in `.github/workflows/ci.yml` as A1 did for `diag/`.

---

## Scope

### In
1. **The plant** — commanded voltage → wheel velocity → body twist → true pose, advanced per tick
2. **Sensor synthesis** — derive encoder/tracking-wheel, IMU, and GPS readings *from the true pose*
   and push them into the existing fakes
3. **Ground-truth exposure** — the harness reports the exact true pose/twist at every tick
4. **Deterministic scenario runner** — seeded, bit-reproducible, replayable runs
5. **Degradation injection seams** — the hooks A3 will populate (leave them unpopulated here)

### Out — do not build
- **Hostile degradation behaviours** (noise, dropout, `PROS_ERR_F`, slip, brownout) → **chunk A3**.
  Build the *seams*, not the behaviours.
- Motion primitives → C1 · `hal/sim` + `SHUL/2` → H2 · correctors/EKF → Phase E

### Explicitly rejected
**Do not model dynamics.** No mass, no inertia tensor, no motor torque curves, no friction
coefficients, no slip physics. **Those parameters are unmeasurable without a robot**, and a plant
tuned on invented constants produces confident, wrong answers — which is worse than an honestly
limited one, because it will be trusted.

Model voltage→velocity through the **existing feedforward relation** (`V = kS·sign(v) + kV·v + kA·a`,
inverted), with a first-order lag if you want approach realism. Document the boundary loudly in the
header: **this plant proves logic, not constants.** Gains tuned against it are provisional until R5.

---

## Design constraints (the load-bearing ones)

### 1. Drive the existing fakes — do not replace them
`FakeMotor`, `FakeImu`, `FakeGps`, `FakeRotation` keep their current contracts and gain a plant
*behind* them. F4 is frozen; the fakes are part of how it is exercised. The plant writes synthesized
readings into the fakes through their existing setters, and the code under test sees only
`RobotContext` and the F4 interfaces — exactly as it will on a robot.

If a fake genuinely lacks a setter the plant needs, add it **additively** and say so.

### 2. The truth integrator MUST be independent of `arcStep` ⚠️ **the subtlest requirement here**
`PilonsOdometry` integrates pose using `arcStep`. If the plant computes true pose with the *same*
`arcStep`, then **any error in `arcStep` cancels out and becomes invisible** — odometry would appear
perfect while both sides make an identical mistake. That would silently destroy the value of every
localization test in Phase E.

Advance the true pose by an **independent** method: fine-grained sub-stepping (many small Euler or
RK4 steps per tick), or a closed-form constant-twist solution derived separately. Then **test that
`arcStep` agrees with the independent truth to a documented tolerance** — that comparison is a real
test of `arcStep`, and it is only possible if the two are genuinely independent.

Document this reasoning in the header. It is the kind of trap that is invisible once made.

### 3. Ground truth is the product
Expose the exact pose and twist at every tick. Phase E measures estimator error directly against it.
Make it structurally impossible for code under test to *read* truth by accident — truth is for the
test harness and assertions, never an input to the estimator.

### 4. Bit-reproducible determinism
Same seed + same scenario → byte-identical results, run to run and machine to machine. No wall-clock
(use `FakeClock`), no unseeded RNG, no iteration over unordered containers in a way that affects
output. A failing run must be re-runnable exactly — this is what makes a flaky Phase E result
debuggable rather than folklore.

### 5. Build the degradation seams now, empty
A3 injects sensor noise, dropout, latency, quantization, slip, saturation, and brownout. Put the
**hooks** in now — a policy/interface the plant consults — so A3 is population, not surgery. Leave
them no-ops and document each intended use. Do not implement the behaviours.

### 6. Make the sim legible using A1
The plant should emit `DebugRecord`s and be watchable through `TermSink`. A1 exists precisely so this
chunk is observable. Respect A1's cost contract: use `emitRecord(sink, buildFn)` so a run with
`NullSink` pays nothing, and override `wantsRecord()` and `emit()` **as a pair** (A1's documented
footgun).

---

## Test requirements

Tests must **try to break the code**. Required coverage:

- **Open-loop travel** — a known voltage for a known duration moves the robot a predictable distance,
  derived analytically, not from the plant's own output.
- **Closed-loop convergence** — `Pid` drives the plant to a target and holds it; and a deliberately
  bad gain **diverges** (a plant that always converges is not modelling anything).
- **Kinematics round-trip** — plant forward kinematics agree with `MatrixKinematics::forward()`;
  `toWheels` → plant → observed twist round-trips within tolerance. Sweep, don't hand-pick.
- **`arcStep` vs independent truth** (constraint 2) — agreement within a documented tolerance across
  swept twists including near-zero ω, pure rotation, pure strafe, and large per-tick deltas.
- **Odometry error against truth** — `PilonsOdometry` fed synthesized sensors tracks truth within a
  documented bound over a multi-second trajectory. This is the first end-to-end localization proof in
  the project.
- **Determinism** — same seed, byte-identical output; different seed, different output.
- **Both drivetrains** — X-drive and tank; the plant must not be X-specific (C3 adds H-drive).
- **Degenerate inputs** — zero voltage, saturating voltage, zero `dt`, a very large `dt`, and a
  reversal, each behaving sanely rather than producing NaN or silently doing nothing.

**Mutation checks (prove red, then restore) — at minimum four**, including:
- Make the truth integrator reuse `arcStep` → the independence test must go red
- Flip a sign in the plant's wheel→body direction → the round-trip test must go red
- Break determinism (unseeded value) → the reproducibility test must go red
- Drop the feedforward `kS` term → the open-loop travel test must go red

**Record the actually-observed result of each.** A mutation you did not run does not count.

---

## Definition of Done

- [ ] An open-loop voltage command moves the simulated robot a predictable, analytically-derived distance
- [ ] A closed-loop `Pid` holds a position against the plant; a bad gain demonstrably diverges
- [ ] Plant forward kinematics round-trip against `MatrixKinematics` across a swept input space
- [ ] Truth integration is independent of `arcStep`, and `arcStep` is verified against it
- [ ] `PilonsOdometry` tracks truth within a documented bound over a multi-second run
- [ ] Runs are bit-reproducible from a seed
- [ ] Works for both X-drive and tank
- [ ] Degradation seams exist, are documented, and are no-ops
- [ ] A run is watchable via `TermSink`, and costs nothing with `NullSink`
- [ ] Full host suite green under strict `-Werror`; all v2 headers still cross-compile for ARM; CI guard passes

---

## Live progress log — required

Append to `docs/internal/chunks/A2-PROGRESS.md` **as work happens**, not at the end. The user watches
this with `tail -f`. Use `date +%H:%M:%S` for real timestamps.

```
[HH:MM:SS] START   <what is being attempted>
[HH:MM:SS] DONE    <what landed + evidence>
[HH:MM:SS] MUTATE  <mutation> -> RED/GREEN (observed)
[HH:MM:SS] DECIDE  <choice> over <alternative> because <reason>
[HH:MM:SS] BLOCKED <what, and what is being tried instead>
[HH:MM:SS] FOUND   <discovery about existing code>
```

Write the **first entry before your first edit**, and append continuously. A log written at the end is
a retro-narration and defeats the purpose.

---

## Documentation contract — all six, plus the completion record

1. **Roadmap** — A2 is not currently a roadmap task (it is the incompleteness bug). **Add it** under
   M2 as a task in the appropriate workstream, checked with cited evidence, and note that it closes a
   gap M2's and M4's DoDs both depended on.
2. **`roadmap.md` "you are here"** → A2 done, A3 (hostile fakes) next.
3. **Design notes in every header** — especially the `arcStep`-independence reasoning (constraint 2),
   the kinematic-not-dynamic boundary and why, and each degradation seam's intended use.
4. **Test evidence** — case count, what each test catches, observed mutation results.
5. **Decision log** — every choice with a viable alternative.
6. **Freeze Register** — no freeze here. Note whether anything in the harness is becoming a de-facto
   contract other chunks will depend on.

Then write **`docs/internal/chunks/A2-COMPLETED.md`** in the same shape as `A1-COMPLETED.md`: what was
built, decisions with rejected alternatives, full test inventory, mutation results as observed,
verification command outputs, discoveries about existing code, and what was deliberately deferred.

**Do not commit.** Leave everything in the working tree for review.

---

## Landmines

- **The `arcStep`-independence trap (constraint 2) is the one that would quietly ruin Phase E.** If
  you take one thing from this brief, take that.
- **Do not model dynamics.** An honestly-limited plant is worth far more than a plausible-looking one
  built on invented constants, because the plausible one gets trusted.
- **Do not let truth leak into the estimator's inputs.** The whole value is that the estimator works
  from synthesized *sensors*, not from the answer.
- **Do not implement A3's degradation behaviours.** Seams only.
- **Respect A1's null-sink cost contract** — `wantsRecord()` and `emit()` are overridden as a pair.
