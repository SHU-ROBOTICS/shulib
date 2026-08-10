# Chunk C4 — COMPLETED (built 2026-08-10; verified same day; working tree pending review)

> **The `Chassis` facade exists — and F6 is deliberately NOT frozen.**
> `include/shulib/chassis/chassis.hpp` is the API an auton is written against:
> `moveTo` / `strafeTo` / `turnTo` / `followTrajectory` / `drive(ChassisSpeeds, Frame)`,
> plus two candidate additions (`brake`, `hold`) proposed to the D2 freeze review.
> Facade routines are **bit-identical to the scheduler-built twin, clean and hostile** —
> the facade added API, not physics — and every lower-layer guarantee is re-pinned
> **through** the public verbs. D1's recipe layer now exercises this surface as a second
> independent consumer; **only D2 freezes it** (§8 is the section D1 stresses and D2 reads).

Suite: **592 cases / 915,157 assertions** green under strict `-Werror` (3 pre-existing skips
unchanged). Baseline entering C4: 556 / 913,561. Both CI guards pass; all **89** v2 headers
ARM-cross-compile clean. 22 mutations run: 20 red, **2 green holes found and closed**.

---

## 1. What was built

**New headers (2):**

- **`include/shulib/chassis/chassis.hpp`** — the facade. Owns the `MotionScheduler` + a
  `MotionConfig`; borrows a validated `MotionDeps` + an `ITickPacer` (all pointees outlive it).
  Public surface: the five F6 verbs, candidate `brake()`/`hold()`, `cancel()` (panic stop),
  `waitUntil(pred, timeout)` (C2's verb re-exported), `pose()`/`setPose()`,
  `strafeAuthority()` (read-only passthrough), `lastExitReason()`/`lastCompleted()`/
  `motionConfig()`, and the Tier-3 seam `scheduler()`/`deps()`. Non-copyable/movable
  (the owned scheduler is pinned by its self-referential stamp).
- **`include/shulib/motion/command_pipeline.hpp`** — `applyCommandPipeline()`: the saturation
  choreography (ω clamp → uniform norm cap → the one F1 rotation → strafe-authority clamp +
  SFB flag → toWheels → desaturate → feedforward → battery ceiling → volts), **extracted
  verbatim from MoveToPose** so the facade's `drive()` reuses one pipeline instead of
  re-deriving it (C1 §11's explicit instruction). `kStrafeFallbackNoiseFraction` moved here
  unchanged. Returns `CommandOutcome{body, strafeFallback}` for the caller's record.

**Modified (6):**

- `math/frame.hpp` — **`enum class Frame { Field, Body }`** added (additive vocabulary; F1's
  conventions and the two rotations untouched). The type that makes `drive()`'s frame
  parameter compile-enforced.
- `motion/motion.hpp` — **`tickHealthObservables(deps, odomStalled)`**: the A3 health-wiring
  in ONE place (three grown copies consolidated; the facade would have been a fourth).
- `motion/move_to_pose.hpp` — tick refactored onto the pipeline (bit-identity-pinned, §3);
  old `tickHealth` removed; **new precondition: target position must be finite** (§4.4 —
  fixed at the source, both tiers inherit the rejection; `setTarget` too).
- `motion/turn_to.hpp` — its degenerate ω-only pipeline copy replaced by the shared one
  (`Frame::Body`, exact for a zero linear command).
- `motion/motion_scheduler.hpp` — `tickIdleHealth()` delegates to the shared helper.
- `test/motion_test_rig.hpp` — `ChassisRig` (MotionRig + PlantPacer + Chassis, the C4 stack),
  `chassisConfig()`, and `StallingPacer` (healthy-then-frozen — the DetachGuard's unwind path).

**New test files (3):** `chassis_facade_test.cpp` (construction, stamping, options, misuse,
unwind, brake/hold, waitUntil, panic stop), `chassis_drive_test.cpp` (frame sweeps, pipeline
clamps, pre-empt, boot, teleop health, conflicts), `chassis_routine_test.cpp` (the twin,
sweeps ×3 drivetrains, the complete auton ×3, trajectory semantics, guarantees).

No new HA register entries: **the facade invents no constants** (option sentinels and the
warn-once flag are logic; every numeric default it touches is C1's, already registered).

---

## 2. The inherited-shape ledger — every C1 §11 / C2 §11 / C3 §11 item, resolved

> The brief's hard requirement: each item **adopted / rejected with a reason / deferred with a
> named owner**. Anything missed here becomes permanent at D2.

### From C2 §11 (eight flags)

| # | Shape | Resolution |
|---|---|---|
| 1 | Verb set & signatures (`async`/`tick`/`waitUntilSettled`/`waitUntil`/`cancel` + observability accessors) as the facade's ancestors | **ADOPTED, wrapped**: each blocking verb = `async` + `waitUntilSettled` (C2 §10's question answered YES — §5 D2); `waitUntil` and `cancel` re-exported verbatim; `tick()` + counters stay reachable via `scheduler()` (Tier 3, no ceiling). `lastExitReason`/`lastCompleted` re-exported directly (C5 reads them). |
| 2 | Pre-empt semantics (start-while-active cancels first) | **ADOPTED as facade API behaviour**, documented in the header AND extended: `drive()` pre-empts identically (a manual command supersedes). Pinned by the pre-empt + conflict cases; mutation M6 red. |
| 3 | `WaitResult` re-export | **ADOPTED**: `chassis.waitUntil` returns `motion::WaitResult` unchanged — no new vocabulary invented. Deadline-window pinned; mutation M18 red. |
| 4 | `ExitReason::Cancelled` / `MotionState::Cancelled`(=5) wire-stable | **ADOPTED unchanged**: verbs return `control::ExitReason`; no facade-side enum, no re-mapping. |
| 5 | `ITickPacer` — the loop-ownership seam; "decide background-task-or-not BEFORE D2" | **DECIDED: caller-paced stays; NO background task** (§5 D3). A task would be the tree's first two-task design, unbuildable PROS-free, and would end host determinism. The robot-side pacer is R1/R3's deliverable behind the same seam. |
| 6 | `MotionSchedulerConfig.abortFaultMask` exposure | **ADOPTED at construction**: `ChassisConfig.scheduler` carries the whole `MotionSchedulerConfig` (a team re-legislates the mask by constructing with one). **Runtime mask mutation DEFERRED — owner: D1** (if recipes need a mid-auton policy switch, that is a D1 design discussion feeding the D2 review; C2's own Phase-R fault-statistics revisit stands). |
| 7 | `scheduler.deps()` stamping — convention, not structure (D10's gap) | **CLOSED STRUCTURALLY** — the chunk's core assignment. The facade is the one composition root; every verb builds from `sched_.deps()`; there is no facade code path holding raw deps. Mutation M1 (a verb using raw deps): **138 assertions red**. The Tier-3 raw-deps path still exists BY DESIGN (demonstrated in the stamping test) — the facade closes the door for its users, not for deliberate experts. |
| 8 | Additive-path items (`raiseCount`, appended enum values, `IMotion::cancel()`) | **ADOPTED as landed at C2**; C4 needed no new `IMotion` member — the brief's "last cheap moment" passed without spending it, which is itself evidence the C1/C2 contract set is sufficient for a facade. |

### From C3 §11 (five flags)

| # | Shape | Resolution |
|---|---|---|
| 1 | NO `strafeFallbackActive()` facade getter | **ADOPTED (the rejection)**: no polling getter exists. The record stream (id-stamped, TermSink " SFB") remains the surface; a live-polled bool would invite control-flow coupling to telemetry. If F′-routines ever need to BRANCH on fallback, that is a **D1 waitUntil-predicate design discussion — owner: D1**, exactly as C3 wrote. |
| 2 | `strafeAuthority()` passthrough | **ADOPTED**: `chassis.strafeAuthority()`, read-only, the F5 value verbatim (pinned: 1.0 / 0.35 / 0.0 across the three drivetrains). |
| 3 | Turn-while-drive semantics as facade-level API documentation | **ADOPTED**: chassis.hpp's header carries them (with wait-for-live and pre-empt) — "on an H-drive, lateral-dominant legs run authority-limited and flag SFB". Tested through the facade (trajectory-SFB case; mutation M17 red 8-ways). |
| 4 | `maxWheelSpeed` single shared budget — name the additive path | **NAMED FOR D2**: `ChassisConfig.motion` passes `MotionConfig` through WHOLE, so a future per-wheel budget field flows through with zero facade reshape. The freeze review checks the additive path exists — it does, structurally. **Deferred measurement — owner: R5** (C3 D13 unchanged). |
| 5 | Drivetrain-as-config | **ADOPTED**: kinematics reaches the facade as `MotionDeps.kinematics` — any `IKinematics`, value-built from plain geometry structs (`xDrive(radius)` / `hDrive(HDriveConfig)` / `TankKinematics(track)`), tested all three. No facade-side drivetrain enum, no registry — the `.vexbot` fields map onto these configs at G1 with nothing in between. |

### From C1 §11 (the facade-relevant five)

| # | Shape | Resolution |
|---|---|---|
| 1 | The `MotionDeps` bundle as the facade's held set ("if F6 wants a different composition — e.g. owning the Localizer — C4 is the time to say so") | **DECIDED: borrow exactly this set; own nothing below it** (§5 D1). Owning the Localizer would weld the facade to one odometry wiring and cap Tier 3 (custom estimators). Wiring the pieces is the caller's (standalone path) or G1's (`RobotBuilder`). |
| 2 | The `IMotion` tick contract | **ADOPTED via delegation**: the facade never ticks a motion itself — the owned scheduler does. Zero motion logic in chassis.hpp. |
| 3 | `MotionState` append-only wire vocabulary | **UNTOUCHED** — drive() records use state 0 (no scheduled motion), the honest existing value; nothing appended. |
| 4 | The saturation choreography — "the facade's drive() must reuse, not re-derive" | **ADOPTED BY EXTRACTION** (§5 D4): `command_pipeline.hpp`, one definition, three consumers (MoveToPose, TurnTo, drive()). Mutations M2/M4/M16/M17/M21/M22 each show all consumers' tests watching the one copy. |
| 5 | Wait-for-live as facade-level semantics | **ADOPTED + documented + re-tested through the facade** (boot-wait lands; never-live times out motionless; field-frame `drive()` gets its own boot gate — §5 D6). |

---

## 3. The accuracy numbers through the facade (the DoD item)

### 3.1 The bit-identity twin — the strongest statement available

The same routine (C1's waypoint generator verbatim, seed 77, C2's cadence) run through facade
verbs vs a hand-built `SchedulerRig`: **every waypoint time, error, and heading error equal to
the bit, clean AND hostile** (finalErr 0.228175 in / 9.56 s clean; 1.54884 in / 13.43 s
hostile; 6 motions each). Consequence: **C1/C2/C3's measured baselines carry over VERBATIM** —
the facade's `effectiveConfig` copy, construction order, and blocking loop add nothing to the
physics. (This is the same proof shape C2 used against C1's hand loop, one layer up.)

### 3.2 Clean sweeps through facade verbs — flat in move count, X and H

| n | X finalErr | X worst | X time | H finalErr | H worst | H time |
|---|---|---|---|---|---|---|
| 5 | 0.228 in | 0.229 in | 9.56 s | 0.225 in | 0.231 in | 10.17 s |
| 10 | 0.000 in | 0.229 in | 18.38 s | 0.000 in | 0.231 in | 19.12 s |
| 20 | 0.004 in | 0.238 in | 35.02 s | 0.004 in | 0.238 in | 36.53 s |
| 40 | 0.236 in | 0.238 in | 74.80 s | 0.238 in | 0.238 in | 78.19 s |

Identical to C3's §3 table to the printed digit (the twin predicted exactly this). Flat in
count on both; H pays ~4% time, never accuracy.

### 3.3 Hostile sweeps through facade verbs — bounded, C2/C3's bounds held

Worst across 5/10/20/40 (seed 11): **X 4.128 in** (C2's bound 5.0), **H 4.033 in** (C3's bound
6.0), heading < 0.06 rad both. Zero policy aborts, zero cancels — bookkeeping equals physics.

### 3.4 Tank — a NEW baseline (no prior tank routine existed)

C1–C3 swept tank along-axis trials but never a full multi-waypoint routine. Through the facade,
the author-planned turn-to-bearing + moveTo pattern (bearing computed from `chassis.pose()` —
the realistic auton idiom) lands **flat in count: worst arrival 0.306→0.311 in across
5/10/20/40** (final 0.24–0.29 in; 100.4 s at n=40). Asserted bound 1.0 in (settle tolerance +
the uncorrectable-lateral margin a 1.15° heading settle implies); observed ~3× inside it.

### 3.5 The complete hand-written auton, all three drivetrains

seed pose → moveTo → turnTo → lateral leg (strafeTo on X/H; author-planned turn+drive on
tank) → 2-waypoint `followTrajectory` → bounded `waitUntil` → `hold(0.3)` → `brake()`:
**X 7 motions / 7.80 s / 0.230 in final; H 7 / 7.94 s / 0.230 in; tank 10 / 10.82 s /
0.281 in** — counters exact, zero faults latched.

---

## 4. Findings (each handled where it lives)

### 4.1 FOUND + FIXED (facade): the dangling-motion unwind hazard → `DetachGuard`

A blocking verb owns its motion **on the stack** for exactly the blocking window — safe,
because `waitUntilSettled` cannot return while it is active. But if the wait **throws**
(stalled pacer precondition; an estimator breach with no motion boundary to convert it), the
stack motion dies while still referenced by the scheduler's active slot: the next verb's
pre-empt would `cancel()` a destroyed object. **Observed, not theorized**: with the guard
removed (mutation M5), the unwind test fails 10 assertions and then the suite **segfaults**
(core dumped, 545 cases skipped). The fix: `DetachGuard` cancels on unwind — slot cleared AND
drive safed before the motion object dies (declared after the motion, so it destructs first).
Why cancel-on-unwind rather than weaker alternatives: merely nulling the slot would leave the
drivetrain **energized** mid-exception (the motion was commanding volts when the throw
happened); catching-and-rethrowing per verb duplicates the guard N times; requiring
heap-allocated motions would tax every call for a corner case. Cancel is idempotent, already
the defined safe-state path, and correct even when the exception came from inside `cancel()`'s
own preconditions' domain (the `inTick_`/`inWait_` flags are unwound by their `FlagScope`s
before the guard runs). Pinned by the unwind test; mutation M5 red (violently).

### 4.2 FOUND (green hole, closed): teleop `drive()` ran with fault detection dark — M20

Nothing pinned that health observables stay live in a `drive()`-only loop — the scheduler
ticks health when IT owns the loop, but a driver-control session never touches the scheduler.
Mutation M20 (drive's health tick removed) left all 590 cases green. **A season of driver
practice with IMU_LOST/BROWNOUT/OVER_TEMP dark.** Closed with the teleop-dropout case (IMU
dies mid-drive-loop → `ImuLost` latched once, zero motions started); the mutation now fails
exactly that case.

### 4.3 FOUND (green hole, closed): the yaw-rate budget is invisible to closed-loop tests — M21

Removing the pipeline's ω clamp left all 591 cases green: `desaturate()` rescales any demand
into wheel budget, so every turn stays **convergent** — no accuracy test can see the yaw-RATE
budget, only a truth yaw-rate pin can (the C3 shared-kinematics lesson, recurring: some
mutations are structurally invisible to closed-loop grading). It also exposed that the
`maxAngularSpeed` **option** had no behavioral pin (M9's sibling would have been green too).
Closed with the truth-yaw-step case: option budget binds, config budget binds, uncapped twin
proves non-vacuity. Second-order honesty: without the clamp, a big heading error also steals
linear authority through desaturation (the commanded mix changes) — recorded in the test's
bug line.

### 4.4 FOUND + FIXED (at the source, MoveToPose): non-finite targets servo NaN for a full watchdog window

`Angle`'s named constructors reject NaN but `Pose2d` positions carried it silently — a NaN
target reached the PIDs and commanded NaN volts until the watchdog fired. Fixed **in
MoveToPose's constructor and `setTarget`** (accepts strictly less nonsense — precondition-safe;
both tiers inherit the rejection), not with a facade-only check. `followTrajectory`
additionally validates **every** waypoint before the first leg runs — a NaN at leg k must not
let legs 1..k−1 drive and then throw mid-routine (atomic input validation is a verb-level
semantic, so that part lives in the facade).

### 4.5 Three contract clarifications found by the tests (carried into §8 as API guidance)

1. **TimedOut ≠ cancel at the motors**: a timeout stops the motors (0 V) but only cancel adds
   `BrakeMode::Brake`. Two deliberate contracts (C1's exit vs HA-53's safe state) — the F6
   docs must say so, because a first test conflated them within the hour.
2. **Tank verbs never plan turns** (D12 held through the facade): an off-line `moveTo` on tank
   honestly fails; the AUTHOR writes `turnTo(bearing)` first. The auton test performs the
   idiom; F6's docs must teach it.
3. **`followTrajectory` per-leg budgets must cover the PID settle tail**, not just travel time:
   a 6-in leg needs ≈1.1 s (exponential approach at kP=3 + 0.1 s hold), and a leg's budget is
   per-leg because each leg is one watchdog. This is F6 API guidance (see §8), not a test note.

### 4.6 Process catch: a non-compiling mutation almost read as green

M1's first form broke `-Werror` (member-init reorder); the suite binary was stale and the
"result" was a fake green. Caught by checking the build log; the campaign runner now hard-gates
on build success. Recorded because the failure mode — **mutation results from a binary that
never contained the mutation** — would silently invalidate a whole campaign.

---

## 5. Decision log (every choice with a viable alternative)

### D1 — The facade BORROWS `MotionDeps` + pacer; owns only scheduler + config
Alternative: own the Localizer (C1 §11 explicitly invited it) or the whole stack. Rejected:
owning the Localizer welds the facade to one odometry wiring (PilonsOdometry + two tracking
wheels + one fusion policy) and caps Tier 3 — a custom estimator could no longer sit behind
the facade. Construction of the pieces is G1's `RobotBuilder` (file path) or the caller's
(standalone path); the facade is the composition root of the MOTION stack only.

### D2 — Blocking verbs = `async` + `waitUntilSettled`; no async verb variants at C4
Alternative: paired `moveToAsync()` etc. Rejected for now: a facade-owned async motion needs
member storage + lifetime rules (the stack-motion trick only works while blocked), and F2's
combinators are the designed home for composition. The Tier-3 seam (`scheduler()` + `deps()`)
keeps async fully expressible today. If D1's recipes need facade-level async, that is exactly
the kind of awkwardness the not-frozen window exists to surface. **No return-value
`[[nodiscard]]`** on verbs — discarding an ExitReason is legitimate auton style; the latch and
C5's result lines carry pathology (deviation from house style, deliberate and documented).

### D3 — No background task; `ITickPacer` stays the injected seam (C2 flag 5, resolved pre-D2)
Alternative: the facade spawns a control task so verbs "just work" without a pacer. Rejected:
first two-task design in the tree; unbuildable in the PROS-free core (no task primitive
exists); kills host determinism (the entire A2 methodology); and hides the loop C2 made
explicit. The robot pacer (delay-to-next-boundary) is R1/R3's deliverable behind this seam.

### D4 — The pipeline is EXTRACTED, not wrapped or duplicated
Alternative A: `drive()` re-implements the choreography (three copies, the clamp-order
divergence bug waiting). Alternative B: `drive()` constructs a throwaway motion and ticks it
(couples the manual verb to motion lifecycle + settle machinery it must not have). Chosen:
extraction to `motion/command_pipeline.hpp` with arithmetic and order VERBATIM — proven by
the suite landing bit-identical (556→556 pre/post refactor, C2's equivalence suites green),
then by the twin. TurnTo's degenerate copy also folded in (a zero linear command is exact
under the Body path).

### D5 — `Frame` is a REQUIRED parameter of `drive()`, an enum in `math/frame.hpp`
Alternatives: default to Field (silent wrong-frame for body callers); two verbs
(`driveField`/`driveBody` — doubles the frozen surface and un-types the distinction); a bool
(worse than either). The enum lives with the frame math (F1's file, additive vocabulary).
Compile-enforced: `drive(speeds)` does not compile — static_assert-pinned via concepts.

### D6 — Field-frame `drive()` during boot commands ZERO + one Warn per window
Alternatives: precondition-throw (teleop starts during calibration legally — a throw in the
driver loop is a match-loss); silently use the frozen heading (moves the robot in a garbage
direction — the exact bug class); gate Body too (absurd — the stick must work; zero-output
DriveBrake's own exemption reasoning). The warn is once per uninit window (re-armed on the
first live call), because a 100 Hz teleop loop would otherwise print 100 warns/s.

### D7 — `drive()` owns one loop iteration (localizer update FIRST), pre-empts, no stall check
Update-first: a teleop loop calling only `drive()` would otherwise never advance the estimate
— field-centric driving would rot silently (mutation M19: the estimate never leaves
Uninitialized). Pre-empt: C2's last-command-wins semantics extended to the manual verb — the
double-commander alternative is the failure class one-active-motion exists to prevent (M6).
No OdoStallCheck: there is no commanded target to cross-check and the driver is the
supervisor; health observables DO tick (found load-bearing by M20). LoopMonitor stays
scheduler-owned — teleop cadence belongs to the caller.

### D8 — `followTrajectory` returns `TrajectoryResult`; stops at the first non-Settled leg
Alternatives: return bare ExitReason (loses WHERE the chain broke — the caller's recovery
depends on it); continue past a failed leg (chasing waypoints while lost compounds blindly —
M7); a per-leg callback (that is G2's marker system; not invented early). Per-LEG timeouts
because each leg is one scheduled motion with one watchdog — a whole-trajectory budget would
need a second timeout mechanism C2 deliberately doesn't have. Waypoints validated atomically
before leg 1 (§4.4).

### D9 — `brake()` and `hold()` ARE facade verbs — proposed F6 additions (D2 decides)
The roadmap's five-verb list leaves DriveBrake/HoldPose facade-unreachable — but an auton that
cannot park is incomplete, and "anything expressible in recipes must remain expressible"
forbids a Tier-2 cliff where stopping requires the Tier-3 seam. Cost: two more signatures in
the freeze. Explicitly flagged as CANDIDATE ADDITIONS in §8 — C4 proposes, D2 disposes.
(HoldPose's explicit-pose variant stays Tier-3 — one facade verb for the common "stay here".)

### D10 — `MotionOptions` struct (timeout + two speed budgets), zero = config default
Alternatives: positional defaulted parameters (frozen forever, un-extendable without
overload explosion); per-verb config objects (heavy); nothing (real autons need slow-approach
legs — capability lost vs Tier 3). A struct's fields are additive post-freeze. 0-as-sentinel
is unambiguous because `MotionConfig::validate` forbids zero budgets. `maxWheelSpeed`
deliberately NOT scaled with the linear override (hardware envelope vs per-leg intent).

### D11 — `DetachGuard`: cancel-on-unwind (§4.1 carries the full alternatives analysis)

### D12 — Health-observable consolidation (`tickHealthObservables`)
Three copies existed (MoveToPose, TurnTo, scheduler-idle); the facade needed a fourth.
`odomStalled` stays a parameter — the one observable with a per-caller story (the motion's
cross-check verdict vs idle/teleop's structural false). Mutation M15 proved the stall verdict
stays load-bearing through the consolidation at every layer (C1+C2+C4 red together).

### D13 — The stamped bundle is the facade's ONLY deps source; Tier-3 raw deps stay legal
The structural closure (C2 D10). The stamping test demonstrates BOTH halves: facade verbs
cannot miss the stamp (M1: 138 assertions red), and a deliberate raw-deps Tier-3 motion still
runs — with id 0, the honest "nobody assigned this" value. Closing the Tier-3 hole too would
require making `MotionDeps` unforgeable, which would break the standalone promise's plain-C++
construction story. The door is closed for facade users, documented ajar for experts.

### D14 — `pose()`/`setPose()` are facade members
Alternative: force `deps().localizer->pose()`. Rejected: every auton's first line is
`setPose(start)` and routines branch on `pose()` — a Tier-2 recipe cannot reach through a
deps bundle. Heading stays IMU-owned through `setPose` (the Localizer's structural choice,
passed through untouched).

---

## 6. Test inventory (36 new cases — every one names its bug in-file)

**chassis_facade_test.cpp (12 + 11 static_asserts):** concept-based compile pins (frameless
`drive`, bare-double verbs, non-copyable); file-free standalone construction (the longhand
recipe, graded on truth); structural stamping incl. the raw-deps contrast; tank
timeout-option budget; maxLinearSpeed truth-speed cap (+ uncapped twin); maxAngularSpeed
truth-yaw cap (M21's closure); options misuse (NaN/negative, recovery after rejection);
non-finite targets/speeds/waypoints rejected before motion; atomic trajectory validation;
waitUntil semantics; brake-from-speed to true rest; hold window + honesty; panic stop;
the unwind/DetachGuard case.

**chassis_drive_test.cpp (9):** FIELD sweep over 7 headings (incl. seam-adjacent 179°); BODY
sweep over the same; lateral-axis sign pin in both frames; heading-0 coincidence pin;
H-drive authority clamp + SFB flag + tank lateral zeroing through `drive()`; record
commanded-is-field pin; drive-pre-empts-motion (+ stray-tick inertness); boot gate (field
zero + one warn, body drives, post-boot recovery, no warn re-spam); teleop health liveness
(M20's closure).

**chassis_routine_test.cpp (15):** the bit-identity twin (clean + hostile); clean sweeps X+H
via verbs; hostile sweeps X+H via verbs; the tank turn-then-drive baseline ×4 lengths; the
complete auton ×3 drivetrains; trajectory corner-is-driven (path-length proof); failing-leg
chain stop; H-trajectory SFB visibility (id-stamped); ODO_STUCK abort through the facade
(prompt, named, safe, damage-bounded); IMU_LOST continues; boot-wait lands + never-live
bounded-motionless-timeout; cancel-mid-motion through the facade (+ true rest); the
strafeAuthority passthrough pin.

Vacuity discipline: every cap/flag test carries its non-vacuous twin (uncapped run, wrong-id
contrast, provably-moving pre-state, brake modes checked ≠ Brake before the act).

---

## 7. Mutation campaign (22 — each executed: break → build-gate → run → OBSERVE → restore)

| # | Mutation | Observed |
|---|---|---|
| M1 | moveTo built from RAW deps (the forgotten convention) | **RED** 1 case / **138 assertions** (stamping) |
| M2 | Pipeline: `fieldToRobot` → `robotToField` | **RED** 41 cases / 51 — C1 frames+routines, C2, C3, C4 sweeps+twin; Body sweep & heading-0 pin stayed green exactly as the math predicts |
| M3 | Pipeline: Body input rotated as Field | **RED** 3 / 10 — **all in C4's drive suites; NO motion test can see it** (TurnTo's (0,0,ω) is rotation-invariant) — the facade sweep is the sole detector for this bug class |
| M4 | Authority clamp defeated (vyLimit=1e9) | **RED** 11 / 161 — C1 pins + C3's whole visibility suite + both C4 cases |
| M5 | DetachGuard removed | **RED** 10 assertions, then **SEGFAULT** (use-after-free real; 545 cases skipped) |
| M6 | drive() pre-empt removed | **RED** 1 / 6 |
| M7 | Trajectory chain never stops | **RED** 1 / 3 |
| M8 | Trajectory drops the last waypoint | **RED** 3 cases (corner, SFB-trajectory, the auton ×3) |
| M9 | maxLinearSpeed override ignored | **RED** 1 / 1 (the uncapped twin is the non-vacuity) |
| M10 | strafeTo + trajectory drop the timeout option | **RED** 2 / 6 |
| M11 | Field-drive boot gate removed | **RED** 1 / 7 |
| M12 | Facade cancel() delegation severed | **RED** 2 / 15 |
| M13 | hold() ignores its seconds parameter | **RED** 1 / 2 |
| M14 | brake() returns Settled without acting | **RED** 2 / 7 |
| M15 | Stall verdict dropped at the consolidated health tick | **RED** 6 / 14 across C1+C2+C4 |
| M16 | Norm cap removed (probe) | **RED** 3 / 3 — incl. C3's "X NEVER falls back" invariant (which holds only because of the cap); honest scope: accuracy suites don't need the cap |
| M17 | SFB flag forced false (the silent fallback) | **RED** 8 / 8 — C3's 6-way net + C4's two |
| M18 | waitUntil timeout stretched +3 s | **RED** 1 / 1 |
| M19 | drive() never updates the estimate | **RED** 4 / 14 |
| **M20** | **drive() health tick removed** | **GREEN — hole found**; closed with the teleop-dropout case; re-run: RED 1 / 1 |
| **M21** | **ω clamp defeated** | **GREEN — hole found** (invisible to all closed-loop tests); closed with the truth-yaw pin; re-run: RED 1 / 2 |
| M22 | Battery ceiling dropped in the pipeline | **RED** 1 / 1 (C1's weak-pack sweep — the pin migrated with the extraction) |

Post-campaign: all 6 mutated headers `cmp`-identical to pristine snapshots; zero mutation
markers in `include/`/`test/` beyond truth_integrator's pre-existing doc comment and the two
closing tests' own prose. Final re-green: 592 / 915,157. Plus the §4.6 process catch.

---

## 8. THE F6 CANDIDATE SURFACE — what D1 stresses and D2 freezes

> Everything in this section is a PROPOSAL. Nothing here is frozen. Each signature carries its
> "why this shape"; the three clarifications of §4.5 are attached as API documentation the
> freeze must carry.

```cpp
// Construction (borrows deps + pacer; ChassisConfig passes lower-layer configs through WHOLE)
Chassis(const motion::MotionDeps& deps, motion::ITickPacer& pacer, const ChassisConfig& = {});

// The five roadmap verbs
control::ExitReason moveTo(const math::Pose2d& target, const MotionOptions& = {});
control::ExitReason strafeTo(units::Length x, units::Length y, const MotionOptions& = {});
control::ExitReason turnTo(math::Angle heading, const MotionOptions& = {});
TrajectoryResult    followTrajectory(std::span<const math::Pose2d>, const MotionOptions& = {});
TrajectoryResult    followTrajectory(std::initializer_list<math::Pose2d>, const MotionOptions& = {});
void                drive(const math::ChassisSpeeds& speeds, math::Frame frame);  // frame REQUIRED

// CANDIDATE ADDITIONS (C4 proposes, D2 disposes — an auton must be able to park)
control::ExitReason brake(const MotionOptions& = {});
control::ExitReason hold(double seconds, const MotionOptions& = {});

// Control + state
void cancel();                                              // panic stop, always safe
template <Pred> [[nodiscard]] motion::WaitResult waitUntil(Pred&&, double timeoutSeconds);
[[nodiscard]] math::Pose2d pose() const;   void setPose(const math::Pose2d&);
[[nodiscard]] double strafeAuthority() const;
[[nodiscard]] control::ExitReason lastExitReason() const noexcept;
[[nodiscard]] const motion::CompletedMotion& lastCompleted() const noexcept;
[[nodiscard]] const motion::MotionConfig& motionConfig() const noexcept;

// The Tier-3 seam (the no-ceiling guarantee)
[[nodiscard]] const motion::MotionDeps& deps() const noexcept;      // STAMPED bundle
[[nodiscard]] motion::MotionScheduler& scheduler() noexcept;        // same single slot
```

**Why each shape:**

- **Blocking, returning `ExitReason`, not `[[nodiscard]]`** — the student-natural auton line
  (`chassis.moveTo(p);`), bounded by the motion watchdog so blocking can never hang; the
  return is the honest verdict for those who look; pathology reaches everyone via the latch
  and C5. Async stays behind `scheduler()` until D1 proves recipes need more (D2's call).
- **`MotionOptions` struct** — additive-extensible post-freeze (new knobs = new fields, no
  signature change). Zero-as-default is unambiguous (`MotionConfig` forbids zero budgets).
  Both speed budgets are now behaviour-pinned on ground truth (M9/M21).
- **`drive(speeds, Frame)` with a mandatory enum** — the frame is the message. A default
  would re-open the silent-confusion door this rebuild closed; two named verbs would double
  the frozen surface. Field-frame boot behaviour (zero + one warn) and the
  one-loop-iteration ownership (update → pipeline → health → record) are part of the
  contract, all mutation-pinned.
- **`followTrajectory` → `TrajectoryResult{exit, completedLegs, totalLegs}`** — failure
  location is strategy-relevant. Stops at the first non-Settled leg. **G2 boundary explicit
  in-header**: no markers, no waypoint ids, no `.vexbot`, no profiling/blending — a richer
  `Trajectory` overload arrives ADDITIVELY at G2; this span form stays.
- **`brake`/`hold` as additions** — without them, parking requires the Tier-3 seam: a Tier-2
  cliff, forbidden by §17. If D2 rejects them, delete before freeze — that is why they exist
  NOW and not after.
- **`waitUntil` re-exported with a REQUIRED finite timeout** — C2's no-hang discipline at the
  public edge; `WaitResult` distinguishes outcome from motion exit vocabulary on purpose.
- **The Tier-3 seam is part of the frozen surface** — `deps()`/`scheduler()` are the
  documented no-ceiling guarantee; freezing the facade without them would freeze users OUT of
  the lower layers.

**API documentation the freeze must carry** (from §4.5 + inherited semantics): wait-for-live
("a verb during boot waits, motionless, watchdog running"); pre-empt (incl. `drive()`);
turn-while-drive + SFB on limited-strafe drives; TimedOut-stops vs cancel-brakes; tank =
author-plans-the-turn; **per-leg trajectory budgets must cover the settle tail** (≈1.1 s for a
6-in leg at HA-50 gains — travel time alone under-budgets every leg); cancel-safe-state
(HA-53); the standalone construction recipe.

**Known tensions for D1 to probe:** does the recipe layer need facade-level async or
result-chaining that blocking verbs can't express? Does `hold` want a disturbance-radius
option? Is `TrajectoryResult` enough, or do recipes want per-leg results (C5's material)?
Should `MotionOptions` carry a per-verb settle-tolerance override (currently config-only)?
These are exactly the questions the not-frozen window exists for.

---

## 9. What we now know for certain, and what we do not

### Known, with evidence

1. **The facade adds API, not physics.** Bit-identity with the scheduler twin, clean and
   hostile; sweep tables identical to C3's to the printed digit (§3).
2. **Command-id stamping is structural through the facade.** No verb can build an unstamped
   motion (M1: 138 red); the Tier-3 raw path remains, deliberately, with id 0.
3. **The standalone promise is executable code**, not prose: a working Chassis from plain C++
   in one test file, no config artifact of any kind (the test IS the recipe).
4. **Frame confusion is now a compile error at the manual verb**, and every runtime frame
   defect in the ONE pipeline is caught from two independent directions (motion suites +
   facade sweeps) — except Body-as-Field, which ONLY the facade sweeps can see (M3): C4's
   tests are load-bearing, not duplicative.
5. **Every inherited guarantee survives the facade**: ODO_STUCK abort (prompt, named,
   damage-bounded), watchdog bounds incl. never-live boot, cancel safe state + panic stop,
   IMU_LOST continuation, hostile bounds (X 4.13 / H 4.03 in), SFB visibility (8-way net).
6. **The unwind hazard was real** — literally a segfault without the guard (M5) — and the
   guard closes it while leaving the chassis usable afterwards.
7. **Tank runs full routines through the facade** flat in count (0.31 in worst) — with the
   author-planned-bearing idiom the docs must teach.
8. **Two green holes existed and are closed**: teleop health liveness; the yaw-rate budget.
   Both are now single-point-of-failure tests that go red alone.

### NOT known, stated plainly

1. **Whether these shapes survive a second consumer.** That is D1's job — by design, nothing
   C4 can claim. §8's tension list is the honest starting point.
2. **Anything about real hardware.** Every number above is the A2 plant + A3 hostility;
   gains provisional (HA-50/51/52), brake efficacy provisional (HA-53), H geometry invented
   (HA-55). R-phase settles them.
3. **The robot-side pacer.** The seam is decided (no background task); the delay-to-boundary
   implementation and its jitter behaviour are R1/R3's.
4. **Teleop ergonomics beyond `drive()`** — curvature/arcade shaping, slew limits, driver
   preference curves are F′/D-phase questions; `drive()` is deliberately the primitive, not
   the product.
5. **Whether `followTrajectory`'s stop-on-failure is the right default for every strategy**
   (a team might want continue-on-timeout for a sweep path). The result struct makes the
   caller's own recovery possible; D1/G2 will show if a policy knob is needed.
6. **The recipe-visible result vocabulary** — C5 formats results next; if it needs more than
   `CompletedMotion` + counters, that lands there (the facade already exposes both).

---

## 10. Deliberately left for later chunks (named handoffs)

- **→ C5 (results/summary)**: everything staged as before, now facade-reachable
  (`lastCompleted()`, counters via `scheduler()`); the drive() record stream (id 0, state 0)
  is a new dev-loop surface C5 may want to label.
- **→ D1 (recipes)**: stress §8; the two deferred items with D1 ownership (runtime
  abortFaultMask mutation; branch-on-fallback via waitUntil predicates); report every
  awkwardness — that is the point of not freezing.
- **→ D2 (the freeze)**: §8 is the review packet; decide `brake`/`hold`; check §2's ledger
  (every row) before locking; carry §8's API documentation list into the frozen docs.
- **→ G1 (`RobotBuilder`)**: `from(profile)` produces exactly the borrow-set (deps + pacer +
  configs) — the facade needs zero changes for the file path to appear.
- **→ G2 (`PathRunner`)**: the additive `Trajectory` overload, markers via `waitUntil`,
  command-id registry; `followTrajectory`'s span form is the floor it builds on.
- **→ R1/R3**: the robot pacer behind `ITickPacer`; HA-53 brake efficacy; the real-IMU boot
  window vs the 2 s hostile model.
- **→ Phase E**: unchanged — the hostile-sweep error remains the drift story (facade twin
  proves the facade added none of it).

---

## 11. Freeze Register note (documentation contract #6)

**No freeze at C4 — explicitly.** The register's F6 row remains 🎯 pending **D2**, now
annotated "candidate built at C4". This chunk's entire design intent is that the surface be
exercised by D1 BEFORE it hardens; freezing here would discard the second-consumer test the
build order was rearranged to create.

Freeze-adjacent acts, recorded:

- **`math::Frame` added to frame.hpp** (F1's file): additive vocabulary only; F1's
  conventions and both rotations untouched (doc-noted in-header).
- **The pipeline extraction** touched no frozen contract: F5 consumed identically
  (strafeAuthority read-only, toWheels unclamped), bit-identity held.
- **MoveToPose's new finite-target precondition** accepts strictly less than before
  (precondition-safe by the decision-check; no accepted input changed meaning).
- **C2 §11 / C3 §11 / C1 §11: all 18 inherited items resolved in §2** — none left to be
  absorbed by accident at D2.

---

## 12. Verification (actually run, outputs as observed)

```text
$ cmake --build build/test && ./build/test/shulib_tests
[doctest] test cases:    592 |    592 passed | 0 failed | 3 skipped
[doctest] assertions: 915157 | 915157 passed | 0 failed |
[doctest] Status: SUCCESS!
```
(3 skipped = the two M3 acceptance stubs + the R3 GPS field-cal oracle (HA-01), unchanged.
Mid-refactor checkpoint, for the record: after the pipeline/health extraction and BEFORE any
facade code, the suite ran 556 / 913,561 — the exact C3 baseline, bit-stable.)

```text
$ <the ci.yml PROS-free guard grep, scope unchanged — chassis/ was already covered>
GUARD 1 PASS: core is PROS-free (incl. chassis/chassis.hpp + motion/command_pipeline.hpp)
$ <the ci.yml layering guard grep, scope unchanged>
GUARD 2 PASS: layering holds, core is sim-free
```

```text
$ arm-none-eabi-g++ -std=gnu++20 -Wall -Wextra -Wconversion -Wsign-conversion -Wshadow \
    -Werror -Os -mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=softfp \
    -c all_headers.cpp -o /dev/null -Iinclude        # generated list, ALL v2 headers
TU includes 89 headers
ARM CROSS-COMPILE: CLEAN
```

Working tree left uncommitted for review, per the brief. Post-mutation integrity: all 6
mutated headers cmp-identical to pristine snapshots; the only "MUTATION" hit in `include/` is
truth_integrator's pre-existing doc comment. Register reconciliation: zero orphaned
`PROVISIONAL (A4` labels; **no new HA entries** (the facade invents no hardware constants).

---

## 13. DoD checklist (brief §Definition of Done)

- [x] **`Chassis` with all five F6 verbs, delegating rather than re-implementing** —
  chassis.hpp holds zero motion logic, zero kinematics, zero fault policy; the one extracted
  pipeline lives in the MOTION layer and is shared, not duplicated (M2/M4/M17/M22 prove every
  consumer watches the one copy). Candidate `brake`/`hold` additions flagged for D2 (§5 D9).
- [x] **One construction path feeds scheduler and motions; id stamping structural** — the
  facade is the composition root; M1 red by 138 assertions; the Tier-3 contrast documented.
- [x] **All three drivetrains work; file-free C++ construction tested** — the complete auton
  on X/H/tank (§3.5); the standalone test is the longhand recipe.
- [x] **Every lower-layer guarantee verified THROUGH the facade** — §3.3, §6's guarantee
  block: ODO_STUCK, watchdog + never-live boot, cancel + panic stop, IMU_LOST continue,
  hostile bounds, SFB visibility — each with its facade-level mutation or hostile model.
- [x] **Routine accuracy through the facade matches prior baselines on all three** — X and H
  bit-identical-by-twin and digit-identical in the sweep tables; tank is a NEW baseline
  (0.31 in worst, flat), honestly labeled as such since no prior tank routine existed.
- [x] **Every inherited-shape item resolved** — §2: 8 (C2) + 5 (C3) + 5 (C1) = 18 items,
  each adopted / rejected-with-reason / deferred-with-owner (2 deferred: both owner D1).
- [x] **F6 explicitly NOT frozen** — register row 🎯 pending D2, annotated; §11.
- [x] **Suite green under strict `-Werror`; both guards pass; ARM gate passes** — §12,
  actual outputs; 89 headers.
