# Chunk C1 — `IMotion` + the motion primitives

> **Phase C, chunk 1 of 7 — the chunk that makes the library able to drive.**
> Phase A complete (A1–A4). This is the largest single jump in usefulness in the project.

**Workstream:** WS6 (Motion & scheduling) · **Milestone:** M2

---

## Why this matters

Everything built so far is the parts of a car — engine, steering, speedometer, position estimate,
simulator, diagnostics. **There is no driver.** Nothing in the library can currently be told "go to
that spot." C1 is that.

It is also the first chunk where Phase A pays off directly: its DoD items are *runnable checks*
rather than aspirations, because A2 built the plant to settle against and A3 built the hostility to
survive.

---

## What already exists (all of it — build on it, invent nothing you don't need)

| Thing | Where | Use |
|---|---|---|
| `Pid` (D-on-measurement, anti-windup, injected clock) | `control/pid.hpp` | per-axis feedback. **Bare-double by design — this layer owns unit consistency** (its header says so) |
| `Feedforward{kS,kV,kA}` + `compensateForBattery()` | `control/feedforward.hpp` | velocity → voltage, brownout-aware |
| `TrapezoidProfile` | `control/trapezoid_profile.hpp` | signed move → (pos, vel, accel) over time |
| `SettledUtil` | `control/settled_util.hpp` | error AND rate AND held-time |
| `Watchdog`, `ExitGroup`, `ExitReason` | `control/watchdog.hpp`, `exit_group.hpp` | hard timeout + Settled/TimedOut/Running |
| `MatrixKinematics`, `xDrive()`, `tank()`, `desaturateUniform` | `kinematics/` | twist ⇄ wheels |
| `Localizer` (fused pose, `quality()`, dead-reckon flag) | `localization/localizer.hpp` | where the robot is |
| `fieldToRobot()` / `robotToField()` | `math/frame.hpp` | **the one** frame transform (F1) |
| `RobotContext` | `chassis/robot_context.hpp` | DI container |
| `DrivePlant`, `SimHarness`, ground truth | `sim/` | what you test against |
| Hostile models (9 families + composed) | `sim/hostile/` | what you must survive |
| `DebugRecord`, `TermSink`, `FaultCode`, `FaultLatch`, `LoopMonitor` | `diag/` | observability + faults |
| **The assumptions register** | `docs/hardware-assumptions.md` | any new invented constant gets an `HA-nn` entry |

**Read first:** `A2-COMPLETED.md`, `A3-COMPLETED.md` (especially its deferred-to-C1 items),
`build-order.md` Phase C, `RESUMING.md`.

---

## Scope

### In
- **`IMotion`** — the interface every motion implements
- **`MoveToPose`** — decoupled per-axis x/y/θ (the holonomic advantage)
- **`TurnTo`**, **`StrafeTo`**
- **`driveBrake`**, **`holdPose`**
- Every motion reports an **`ExitReason`** and emits per-tick `DebugRecord`s

### Out
- `MotionScheduler` (async/cancel/waitUntil) → **C2**
- The formatted per-motion result line, session header, run summary → **C5**
- `HDriveKinematics` → C3 · `Chassis` facade → C4 · legacy cutover → C6/C7

### Explicitly rejected
**Do not over-tune gains.** Per Phase C's opening note, gains tuned against the plant are
**provisional until R5** — the plant proves control *logic*, not *constants*. Get convergence and
correct behaviour; do not spend effort polishing numbers that a real robot will invalidate. Any gain
that becomes a shipped default needs an `HA-nn` register entry.

---

## Design constraints

### 1. Decoupled per-axis control — this is the whole holonomic thesis
`MoveToPose` runs **separate** controllers on x, y, and θ, combines them into a `ChassisSpeeds`, and
sends that through kinematics. The robot translates and rotates **simultaneously and independently** —
it does not turn-then-drive. This is precisely what LemLib structurally cannot do, and it is the
headline claim of the whole project. Prove it with a test that a diagonal move with a simultaneous
heading change happens *as one motion*.

### 2. Frames: motions target FIELD poses, kinematics eats BODY twists
Convert with `fieldToRobot()` from `math/frame.hpp` — the single transform F1 froze. Getting this
backwards is the classic bug class this project was rebuilt to make structurally impossible. Be
explicit in the code about which frame each value is in.

### 3. The motion layer owns unit consistency
`Pid` is deliberately bare-double. **This layer** wraps it in typed units and is responsible for
feeding it consistent quantities. Do not weaken `Pid`.

### 4. Kinematics never clamps — the motion layer does (F5)
`toWheels()` must not clamp internally. Use `strafeAuthority()` as a **read-only query** and clamp
here, then `desaturateUniform` for over-budget wheel speeds. C3's H-drive depends on this contract
holding.

### 5. ⚠️ Honor A3's handoffs — two things were explicitly deferred to C1
A3 recorded both; they are not optional:

- **Wait-for-live-estimate.** A3's fix means motion commanded before `quality()` leaves
  `Uninitialized` is unaccounted for. **C1 must define and enforce that contract**: a motion does not
  act on a dead estimate. Decide the behaviour (refuse / wait / fault), document it, test it.
- **The spin-vs-motion cross-check.** A3 found a *frozen encoder is invisible* to the M2 estimator —
  zero travel is plausible. Containment was assigned to C1's loop shape: compare commanded wheel
  **spin** against observed **motion**, and raise **`ODO_STUCK`** on a sustained mismatch. This is the
  only defence against a dead encoder until Phase E.

### 6. Every motion is watchdog-bounded and reports why it stopped
No motion can hang. `ExitGroup` already composes `SettledUtil` + `Watchdog` and reports
`Settled`/`TimedOut`/`Running` — use it rather than re-deriving exit logic.

### 7. Survive hostility, degrade to a fault
Under A3's hostile fakes a motion must **never diverge, hang, or NaN** — it raises a fault code and
stops safely. This is a DoD item, not a stretch goal.

### 8. Respect A1's cost contract
`emitRecord(sink, buildFn)`; override `wantsRecord()` and `emit()` as a **pair**.

---

## Test requirements

Against the A2 plant, both drivetrains (X-drive and tank), seeded and reproducible:

- **Each primitive reaches its target and settles** within tolerance, from several starting poses
- **Decoupled proof** — simultaneous translation + rotation as one motion; and a pure strafe that
  holds heading
- **Frame correctness** — a field-frame target is reached regardless of starting heading (sweep
  headings; a frame bug shows up as a heading-dependent miss)
- **`ExitReason` correctness** — settles → `Settled`; blocked → `TimedOut`; never `Running` after exit
- **Watchdog** — an unreachable target terminates rather than spinning forever
- **`holdPose`** — pushed off target (inject a disturbance through the plant), it returns
- **`driveBrake`** — stops and stays stopped
- **Wait-for-live-estimate** — a motion issued during the boot window behaves per the contract you
  defined, and is tested
- **`ODO_STUCK`** — a frozen encoder under the A3 hostile model raises it
- **Hostile survival** — every primitive under composed hostility: bounded error, fault raised, no
  divergence, no NaN, run completes
- **Degenerate** — zero-distance move, target == current pose, `dt` == 0, an unreachable target

**Mutation checks — at least four**, including: swap `fieldToRobot`/`robotToField` (frame test must
red), remove the per-axis decoupling (the simultaneous test must red), defeat the watchdog (the
unreachable-target test must red), and drop the `ODO_STUCK` cross-check (its test must red).
**Record each as observed.**

---

## Definition of Done

- [ ] `IMotion` defined; `MoveToPose`, `TurnTo`, `StrafeTo`, `driveBrake`, `holdPose` implemented
- [ ] Each drives the A2 plant to target and settles, on **both** X-drive and tank
- [ ] Decoupled per-axis motion demonstrated (translate + rotate simultaneously)
- [ ] Every motion reports a correct `ExitReason` and is watchdog-bounded
- [ ] Wait-for-live-estimate contract defined, documented, enforced, tested
- [ ] `ODO_STUCK` spin-vs-motion cross-check implemented and tested
- [ ] All primitives survive A3's composed hostility — fault, never diverge
- [ ] The run is legible through `TermSink` as it executes
- [ ] Any shipped gain has an `HA-nn` register entry
- [ ] Suite green under strict `-Werror`; both CI guards pass; ARM gate passes

---

## Live progress log — required

`docs/internal/chunks/C1-PROGRESS.md`, appended as work happens (`date +%H:%M:%S`), vocabulary
`START` / `DONE` / `MUTATE` / `DECIDE` / `BLOCKED` / `FOUND`. Watched live with `tail -f`.

---

## Documentation contract

All six, plus **`docs/internal/chunks/C1-COMPLETED.md`** at A2/A3 depth. Note in the Freeze Register
section that **F6 (`Chassis`) is approaching**: C4 builds the facade, D1 exercises it a second time,
and only D2 freezes it. Anything C1 establishes that the facade will expose should be flagged as a
shape F6 will inherit.

**Do not commit.** Leave everything in the working tree for review.

---

## Landmines

- **Frame confusion is the classic killer.** Be explicit everywhere about field vs body.
- **Don't turn-then-drive.** If the implementation ever sequences rotation before translation, the
  holonomic thesis is lost — that is the LemLib behaviour this project exists to beat.
- **Don't let kinematics clamp.** F5 assigns clamping to this layer; C3 depends on it.
- **Don't skip A3's two handoffs.** They were deferred here specifically; dropping them silently
  re-opens a known hole.
- **Don't polish gains.** They are provisional until R5. Convergence, not beauty.
