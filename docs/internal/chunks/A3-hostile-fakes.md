# Chunk A3 — Hostile fakes

> **Phase A, chunk 3 of 39.** The highest-leverage no-robot chunk in the project.
> Predecessors: A1 ✅ (diagnostics), A2 ✅ (host plant + seams).

**Workstream:** WS2 (HAL) · **Milestone:** M2 · **Spec:** build-order Phase A

---

## Why this chunk exists

Today's fakes are **agreeable**: they encode the same assumptions the production code does, so a test
passes whenever the code agrees with itself. A2 said this plainly in its own honest-limits section —
*"the sensors are still agreeable until A3 populates the seams."* A suite built on agreeable fakes
certifies its own blind spots.

**With no robot, this is the closest thing to hardware you will get.** Every surprise a V5 could hand
you on day one is a surprise you can deliver on purpose today, at zero cost, reproducibly, with ground
truth available to grade against. The stack that survives hostile fakes has a far smaller gap to close
when hardware finally arrives.

**Expect this chunk to find bugs.** That is the point, not a side effect. Per build-order rule 4, a
flaw discovered in A1 or A2 gets **fixed there**, not worked around here. A3 that finds nothing has
almost certainly failed to be hostile enough.

---

## What already exists

| Thing | Where | Note |
|---|---|---|
| **9 degradation seams**, identity-default | `include/shulib/sim/degradation.hpp` | `effectiveVoltage`, `wheelMotionVelocity`, `driveEncoderPosition`, `trackingEncoderPosition`, `imuHeading`, `imuYawRate`, `imuReady`, `gps`, `batteryVoltage` — each takes `Rng&` and/or `Time now`, so stateful policies (latency, drift, thermal) work |
| `GpsTruth` struct | same file | pose + rmsError + hasFix, mirroring `IGps` |
| `DrivePlant`, `SimHarness` | `sim/drive_plant.hpp`, `sim/scenario.hpp` | the tick pipeline + run loops incl. the **variable-dt jitter seam** |
| `Rng` (SplitMix64) | `sim/rng.hpp` | seeded, cross-stdlib reproducible |
| `FaultCode`, `FaultLatch` | `diag/fault.hpp` | A1's stable numeric enum + first-fault latch |
| `FiniteGuard`, `LoopMonitor` | `diag/` | NaN/Inf log-and-recover, dt-budget overrun |
| The F4 fakes | `hal/fake/` | driven by the plant; unmodified through A2 |
| `PilonsOdometry`, `Localizer` | `localization/` | the consumers whose robustness you are testing |

**Read first:** `docs/internal/chunks/A2-COMPLETED.md` (11 decisions, the seam contracts, and its
honest-limits section), `A1-COMPLETED.md` (fault discipline + the null-sink cost contract), and
`docs/internal/build-order.md` Phase A.

**Conventions:** header-only under `include/shulib/`, `#pragma once`, headers explain *why*. Tests are
`test/<name>_test.cpp` (CMake globs). Strict `-Werror` flag set. **PROS-free**; keep both CI guards
passing (the core may never include `shulib/sim/`).

---

## Scope

### In — model how V5 hardware actually misbehaves

1. **Error sentinels** — `PROS_ERR_F` / sentinel returns from any sensor read
2. **Dropout & disconnection** — a sensor that stops responding mid-run and never returns
3. **IMU** — per-boot bias, drift over time, noise, and a not-ready/calibrating window
4. **GPS** — no-fix, bad-fix (plausible but wrong), degraded `rmsError`, and **off-strip** (Driving
   Skills has no strip at all — this path must degrade, not trust garbage)
5. **Encoders** — quantization to real tick resolution, and **tracking-wheel slip** (via
   `wheelMotionVelocity`, so encoders overcount while the robot undershoots — exactly as real slip
   behaves, per A2's decision)
6. **Motors** — voltage saturation, current limiting, thermal droop
7. **Timing** — loop jitter / variable `dt` (the harness seam exists), and **sensor latency** (a
   stateful subclass buffering true values, per `degradation.hpp`'s note — no ring buffer in the base)
8. **Battery** — sag under load down to brownout
9. **A composed "everything hostile at once" model** — realistic degradations stacked

### Out
- New estimator features (correctors, EKF) → Phase E · motion primitives → C1 · anything on-robot → R

### Explicitly rejected
**Do not invent hardware constants you cannot justify.** A2's honesty boundary still holds. Where a
magnitude is a guess (IMU drift °/s, GPS rms inches, thermal droop rate), say so **in the header** and
**add it to the A4 Hardware Assumptions Register** as a falsifiable claim R4 will measure. Model the
*shape* of the failure confidently; label the *numbers* as provisional.

Do not weaken an existing test to make it pass under hostility. If a test must change, the reason goes
in the decision log.

---

## Design constraints

### 1. Every degradation independently injectable, and composable
One at a time for diagnosis, stacked for realism. A failure under the composed model must be
reducible to the individual cause.

### 2. Pathologies raise faults — they never crash
A1 built `FaultCode` + `FaultLatch` for exactly this. **Every sensor pathology must produce a fault
code and a safe fallback**, not an abort, a NaN propagating into the pose, or a silent wrong answer.
This is the chunk that proves A1's fault discipline is real rather than decorative.

### 3. Determinism survives hostility
Same seed + same degradation model → byte-identical results. Hostility is *seeded*, never ad-hoc.
A failure found under hostile fakes must be exactly reproducible, or it is folklore.

### 4. Fix what breaks — in the chunk that owns it
When hostility exposes a flaw in `PilonsOdometry`, `Localizer`, `FiniteGuard`, or the plant, fix it
**there**. Record each in the progress log as `FOUND` and in the completion record. If something
cannot be fixed within A3's scope, say so explicitly and name the chunk that must.

### 5. Turn on the M2 acceptance stub
A2 deliberately left the M2 `<1°` acceptance stub skipped, on the grounds that asserting sub-degree
accuracy against *perfect* sensors is theater. **With modeled IMU drift and noise it becomes
meaningful — unskip it and make it real.** If the stack cannot yet hold `<1°` under hostility, that is
an honest and valuable finding: report the actual number, keep the test red-or-skipped with a
documented reason, and note that Phase E's correctors are what close it. **Do not weaken the target to
make it pass** — F2 is frozen.

### 6. Respect A1's cost contract
`emitRecord(sink, buildFn)`; override `wantsRecord()` and `emit()` as a **pair**.

---

## Test requirements

Each degradation needs (a) a test that it actually degrades — a dead seam must be impossible to ship,
as A2 proved for `effectiveVoltage` — and (b) a test that the stack **survives** it with a fault
raised and bounded error.

Specifically required:
- **Every sentinel/dropout path** → fault code raised, no NaN reaches the pose, estimate stays finite
- **IMU drift** → measured heading error over 60s, reported as a number
- **GPS off-strip** → dead-reckon-only, no garbage trusted; **bad-fix** → bounded damage
- **Slip** → encoders overcount while truth undershoots; odometry error grows in the predicted direction
- **Latency** → a stale reading does not corrupt the estimate unboundedly
- **Jitter / variable dt** → `LoopMonitor` fires; controllers stay stable
- **Brownout** → the latched marker fires and the fallback holds
- **Composed model** → the full stack completes a multi-second run with bounded, *reported* error

**Mutation checks — at least four**, including: remove a fault-raise (survival test must red), make a
degradation a no-op (its liveness test must red), break seeded reproducibility (determinism must red),
and defeat a guard so a NaN propagates (the finiteness test must red). **Record each as observed.**

---

## Definition of Done

- [ ] All 9 seams populated with justified hostile behaviours; each independently injectable
- [ ] A composed "everything hostile" model exists and a full run completes under it
- [ ] Every sensor pathology raises a fault code with a safe fallback — no crash, no NaN in the pose
- [ ] Determinism holds under hostility (same seed → byte-identical)
- [ ] Every flaw hostility exposed is **fixed in its owning chunk**, or explicitly deferred with the
      chunk named
- [ ] The M2 `<1°` acceptance stub is unskipped and either passes or reports its honest number
- [ ] Invented magnitudes are labelled provisional in-header and queued for A4's register
- [ ] Full suite green under strict `-Werror`; both CI guards pass; all headers cross-compile for ARM

---

## Live progress log — required

Append to `docs/internal/chunks/A3-PROGRESS.md` **as work happens** (`date +%H:%M:%S`). The user
watches with `tail -f`. Same entry vocabulary: `START` / `DONE` / `MUTATE` / `DECIDE` / `BLOCKED` /
`FOUND`. **Be especially generous with `FOUND`** — this chunk is expected to surface real defects, and
watching them surface is the most interesting part.

---

## Documentation contract

All six (see build-order), plus **`docs/internal/chunks/A3-COMPLETED.md`** matching A1's and A2's depth
(A2 ran 355 lines / 11 decisions). Give the flaws-found section its own prominent place, and list every
provisional magnitude destined for A4's register.

**Do not commit.** Leave everything in the working tree for review.

---

## Landmines

- **A3 that finds nothing has failed.** If the stack sails through, the fakes are not hostile enough —
  go harder before declaring done.
- **Do not weaken F2's `<1°` target** to make a test pass. Report the real number instead.
- **Do not paper over a flaw in A1/A2** — fix it at its source.
- **Label every invented magnitude.** An unlabelled guess becomes a fact nobody re-examines.
- **Hostility must be seeded.** Unreproducible failures are worse than none.
