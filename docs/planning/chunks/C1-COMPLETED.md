# Chunk C1 — COMPLETED (2026-08-06)

> Completion record for [`C1-motion-primitives.md`](C1-motion-primitives.md) — `IMotion` + the
> motion primitives: the chunk that makes the library able to drive. Everything below is **as
> actually observed** — commands run, outputs captured, all 12 mutations executed and watched
> (the live sequence is in [`C1-PROGRESS.md`](C1-PROGRESS.md), 47 entries). Code and tests are
> committed as checkpoint `7e54826` after independent verification; this record completes the
> chunk's documentation contract.
>
> **The headline:** the robot can now be told "go to that spot," and the way it goes is the
> project's thesis made real — `MoveToPose` translates and rotates **simultaneously and
> independently** through three decoupled per-axis loops, proven by a test that a sequenced
> turn-then-drive implementation demonstrably fails (mutation #2). And the chunk's most valuable
> products were two test-suite HOLES that only running the mutations exposed (§7, #5 and #11).

---

## 1. What was built

| Piece | File | Role |
|---|---|---|
| `IMotion` + `MotionState` + `MotionDeps` | `include/shulib/motion/motion.hpp` *(new)* | The tick contract (start/tick→`ExitReason`; loop updates the Localizer FIRST — A2's controller-first shape), the wait-for-live-estimate contract in prose (A3 handoff #1), the wire-stable `activeCommandState` vocabulary (Idle/WaitingForEstimate/Running/Settled/TimedOut), the named-pointer dependency bundle the C4 facade will inherit |
| `MotionConfig` + `AxisGains` | `include/shulib/motion/motion_config.hpp` *(new)* | Every gain/budget/tolerance in one validated struct, all PROVISIONAL (A4: HA-50/51/52), with the saturation choreography and the noise-floor reasoning documented in-header |
| `OdoStallCheck` | `include/shulib/motion/odo_stall_check.hpp` *(new)* | **A3 handoff #2**: the windowed spin-vs-motion cross-check — mean \|Δ drive shaft\|·r vs hypot(Δx,Δy) + R·\|Δθ\| — the only defence against a dead encoder until Phase E |
| `MoveToPose` | `include/shulib/motion/move_to_pose.hpp` *(new)* | The three-axis decoupled engine (field-x PID, field-y PID, heading PID → one `ChassisSpeeds`), the full F1/F5 pipeline (norm-cap → `fieldToRobot` → authority clamp → `toWheels` → `desaturate` → FF → battery comp), exit logic, fault wiring, per-tick lazy records; also the shared core StrafeTo/HoldPose shape via `PoseMotionOptions` |
| `TurnTo` | `include/shulib/motion/turn_to.hpp` *(new)* | Pure heading loop on F3's shortest signed error; unmodified `ExitGroup` exit |
| `StrafeTo` | `include/shulib/motion/strafe_to.hpp` *(new)* | Translate holding the FIRST-LIVE-tick heading (capture-at-live is part of the wait contract) |
| `HoldPose` | `include/shulib/motion/hold_pose.hpp` *(new)* | Active three-axis disturbance rejection for `holdFor`; honest deadline verdict (§5 D8) |
| `DriveBrake` | `include/shulib/motion/drive_brake.hpp` *(new)* | Zero-volt + `BrakeMode::Brake` stop with a settle verdict built on a 5-tick vector-averaged twist (§4.2 — the noise-floor finding) |
| `Localizer::lastOdomDeltaImplausible()` | `include/shulib/localization/localizer.hpp` *(modified, additive)* | Forwards `PilonsOdometry::lastDeltaImplausible()` so the motion's HealthMonitor wiring is complete without the motion holding the odometry; raising stays policy (A3's D3 unchanged) |
| TermSink `[MOT]` discriminator fix | `include/shulib/diag/term_sink.hpp` *(modified)* | The first-consumer defect (§4.1), fixed in the owning A1 file per the rules |
| CI guards | `.github/workflows/ci.yml` *(modified)* | `include/shulib/motion` added to BOTH guard scopes (PROS-free; core-never-includes-sim). The ARM gate's generated list picks the 8 new headers up automatically |

**New tests:** `test/motion_test_rig.hpp` (the shared closed-loop rig: harness + PilonsOdometry +
ComplementaryFusion + Localizer + FaultLatch + HealthMonitor + MotionDeps, truth-graded),
`motion_primitives_test.cpp` (20), `motion_frames_test.cpp` (6), `motion_sweep_test.cpp` (6),
`motion_hostile_test.cpp` (13), `motion_routine_test.cpp` (4), `motion_stall_check_test.cpp` (9).
**58 new cases / 177,525 new assertions** (429/681,086 → **487/858,611**; skips unchanged at 3).
Every grading pose in every test comes from `h.truePose()` — the estimate is what the motion
READ; truth is what the motion DID.

---

## 2. How A3's two handoffs were resolved (they were the point)

### 2.1 Wait-for-live-estimate → **WAIT, bounded by the watchdog** (enforced by every primitive except DriveBrake)
The contract, now in `motion.hpp` and enforced + tested:
- While `qualityClass() == Uninitialized`, `tick()` commands **ZERO volts**, makes no settle
  progress, reports `WaitingForEstimate`, and captures no estimate-derived target.
- **The watchdog runs through the wait** — a never-live estimate exits `TimedOut` +
  `MOTION_TIMEOUT` rather than hanging; "no motion can hang" includes the wait itself.
- **Capture-at-first-live-tick:** StrafeTo's held heading and HoldPose's captured pose are read
  at the first LIVE tick, never at `start()` — capturing during boot would lock onto calibration
  garbage (±90° swings). Pinned: a StrafeTo issued mid-calibration holds the true post-boot 40°
  within 2°.
- **`Degraded` does NOT gate.** A robot that HAD an estimate and lost heading authority mid-run
  keeps driving on the stale estimate (the Localizer's own D8 logic; freezing mid-run strands
  the robot mid-field). Pinned: a mid-run IMU dropout raises `IMU_LOST` once, quality reads
  Degraded, and the motion still settles.
- **DriveBrake is exempt** — zero output is the safe action in every state including boot, and
  the guaranteed end-of-run park must be able to kill the drive unconditionally. Its *verdict*
  legitimately defers (§4.3); its *action* is immediate (pinned: zero volts + Brake mode from
  tick one of the calibration window).

Rejected: *refuse-at-start* (a real auton legitimately starts while the IMU calibrates — A3's
own survival loop waits, then drives) and *fault-immediately* (boot is NORMAL — HealthMonitor's
boot-window-is-not-a-loss rule says exactly this). Observed under the A3 IMU model: a MoveToPose
issued at t=0 waits 200–210 ticks (2 s calibration + the 0.1 s settle window) at provably zero
volts and zero true motion, then drives and settles; with a never-ready IMU it exits TimedOut at
its deadline having never commanded a volt.

### 2.2 `ODO_STUCK` spin-vs-motion cross-check → **`OdoStallCheck`, owned by every motion's tick**
The verdict, per 0.3 s window: `spinTravel = mean |Δ driveShaft|·wheelRadius`;
`observed = hypot(Δx,Δy) + rotationRadius·|Δheading|` (shortest-path Δθ — the ±180° seam cannot
fabricate or mask motion); stalled iff `spinTravel ≥ 1.0 in AND observed < 0.25·spinTravel`.
The verdict feeds `HealthMonitor::Observations::odomStalled` (A3's D3 wiring) from inside every
primitive's tick — the active motion IS the loop at C1, so the containment cannot be forgotten
by a caller.

The design points that took real choices: the **rotation term** makes a pure `TurnTo` immune to
false positives AND means a frozen tracking encoder cannot false-fault a turn (heading is
IMU-owned — dead position odometry can't hurt a pure rotation; pinned both ways); the **ratio**
keeps the check speed-independent with margin over A3's 70%-propulsion slip (pinned at exactly
70%); the **mean over drive wheels** keeps a single dead drive encoder detectable without
zeroing the signal. Observed against the A3 hostile model (`trackingFreezeAt`, both channels):
`lastDeltaImplausible()` **never fired** — the A3 §3.4 estimator-blindness re-confirmed at the
motion level — while the cross-check raised `ODO_STUCK` within one window and the watchdog
contained the run. Deliberate scope line, named for C2: the motion **raises the fault but does
not self-abort** — cancellation policy belongs to the scheduler (§10).

---

## 3. The chained-routine numbers (the user's headline requirement)

Hand-chained waypoint routines (MoveToPose legs, a pure `TurnTo` every 3rd waypoint — time with
~zero distance, decorrelating the regressors — a `StrafeTo` every 7th), swept over routine
length {5, 10, 20, 40} × seeds, X-drive, graded against truth at every arrival:

**Clean plant (the count-compounding pin).** With no sensor lies there is no drift clock, so any
error growth with length would be structural per-move compounding. There is none:

| n | motions | final err | worst arrival | time | distance |
|---|---|---|---|---|---|
| 5 | 6 | 0.228 in | 0.229 in | 9.6 s | 128 in |
| 10 | 13 | 0.00003 in | 0.229 in | 18.4 s | 167 in |
| 20 | 26 | 0.004 in | 0.238 in | 35.0 s | 316 in |
| 40 | 53 | 0.236 in | 0.238 in | 74.8 s | 789 in |

Error is **FLAT in move count** — 40 chained moves end as accurately as 5, all inside the 0.5 in
settle tolerance. **Directional bias:** mean signed arrival error over 75 arrivals = **(+0.006,
+0.004) in** — no systematic settling side. **Stop-and-settle overhead ≈ 1.19 s/motion** at the
provisional gains — the measured cost of the no-blending v1 scope (Frontier item, now a number).

**Full composed A3 hostility** (drift, noise, quantization, decimation, sag, slip, latency,
calibration windows — every routine's first move waits out boot):

| n | seed | final err | worst arrival | worst heading | time | distance |
|---|---|---|---|---|---|---|
| 5 | 11/22 | 1.00 / 0.63 in | 1.00 / 0.71 in | ~0.12° | ~12 s | ~100 in |
| 10 | 11/22 | 2.61 / 0.78 in | 2.61 / 0.78 in | ~0.13° | ~24 s | ~230 in |
| 20 | 11/22 | 4.13 / 0.64 in | 4.13 / 1.02 in | ~0.39° | ~45 s | ~410 in |
| 40 | 11/22 | 3.15 / 2.35 in | 4.13 / 2.77 in | 0.64–1.19° | ~92 s | ~880 in |

Worst anywhere: **arrival 4.13 in, final 4.13 in, heading 1.19°** (seed 11 — the bad-drift
boot; typical boots run 0.6–2.6 in). The asserted bound (5.0 in) is derived, not wished: at the
HA-20 worst bias (1°/min) over ~95 s and ~900 in the pessimistic physics ceiling is ~12 in;
observed sits 3× inside it.

**The three regressions** (pooled over every waypoint of every hostile routine):
- **error vs move index: 0.062 in/move** raw — but the CLEAN sweep proves count itself
  contributes ~nothing (structurally flat), so this slope is count-as-a-proxy-for-time;
- **error vs elapsed time: 0.028 in/s** — the real carrier: M2 localizer drift, **expected**
  with no correctors, and exactly what Phase E exists to shrink;
- **error vs distance travelled: 0.0031 in/in** — near-flat, consistent with calibrated wheels.

**The count-vs-time discriminator** (same ground, twice the moves): 8×30 in legs → 0.73 in mean
final; 16×15 in legs → 1.22 in. No 2× compounding — family B pays only its extra ~12 s of drift.

**The distance-attribution diagnostic, demonstrated live:** a twin run with tracking wheels
configured 2% fat (the A2 anti-agreeable trick) regresses arrival-error-vs-net-displacement at
**0.0194** — the injected 2%, recovered quantitatively — while the calibrated twin reads
**−0.00004**. That is the exact signature R3's calibration walk exists to remove (HA-12/13/14
blast radius), and the routine diagnostics demonstrably see it.

---

## 4. Flaws found (each fixed in its owning place, per rule 4)

### 4.1 FOUND + FIXED (TermSink/A1): an ACTIVE motion rendered as "[LOC] idle"
TermSink discriminated the `[MOT]` record channel on `activeCommandId != 0` — but ids are
assigned by the **C2 scheduler**, so a C1 motion running standalone has id 0, state ≠ 0, and its
whole run rendered as idle localization ticks. A defect only the first real producer could
expose. Fixed in the owning A1 file (`id != 0 || state != 0`); a genuinely idle record (both
zero) still renders `[LOC] idle`, so every existing golden-line pin held. Guarded by the C1
legibility test and re-verified by mutation #12.

### 4.2 FOUND + CHARACTERIZED (Localizer twist; handled in DriveBrake): the estimator cannot certify "stopped" below ~1.5 in/s
Probed directly: at a **physical dead stop** under composed hostility the Localizer's twist (a
raw finite difference of the fused position, documented M2 semantics) reads **0.5–1.5 in/s** of
noise — HA-21-class heading noise through the tracking-offset correction, differentiated at
100 Hz. DriveBrake's original 0.5 in/s threshold sat *below the estimator's own noise floor* and
could never settle on a hostile field. Fixed where the requirement lives: DriveBrake builds its
speed norm from a 5-tick **vector-averaged** twist (√n reduction, 50 ms verdict delay) and the
default threshold moved above the averaged floor (1.2 in/s — HA-51). Rejected: smoothing inside
the Localizer (its raw finite-difference is the documented M2 contract Phase E will measure
against). True stopped-ness stays pinned against plant ground truth (velocity exactly 0).

### 4.3 FOUND, by-design, documented: a boot-window brake's VERDICT defers ~2 s
During calibration the yaw-rate stream serves ±10 rad/s garbage, so DriveBrake's speed norm
reads ~70 in/s while the robot sits still. The zero-volt **action** is immediate (pinned tick
one); the **verdict** waits for the sensor to go live (~calibration end + ~0.2 s, pinned < 3 s).
"Stopped" is certified when provable, not asserted while the only rotation sensor screams.

### 4.4 FOUND, by-design, pinned: holding against a sustained shove IS the stall signature
HoldPose fighting a continuous push reaches equilibrium with wheels spinning against the shove
and the robot stationary — indistinguishable from a blocked drivetrain, so `ODO_STUCK` fires
before `MOTION_TIMEOUT`. Correct (same fault family: "odometry implausible / wheel stuck") and
now pinned with the reasoning in-test.

### 4.5 FOUND (mutation-9 analysis, recorded): frozen encoders during rotation produce phantom fused translation
Known A3 physics seen from a new angle: with tracking encoders frozen, a rotation's Δθ·offset
correction "removes" wheel motion the dead encoders never reported, producing ~|offset|·Δθ of
phantom fused translation — which is why the *integrated* frozen-encoder TurnTo case survives
the rotation-term mutation while the *unit* case (controlled observables) is its designed
detector. No new defect; bounded; noted for the E-phase estimator-side detector.

### 4.6 The two suite HOLES the mutation campaign found — see §7, mutations #5 and #11
Recorded there with full prominence; they are the chunk's most valuable findings.

---

## 5. Decision log (every choice with a viable alternative)

### D1 — `IMotion` is tick-shaped; the LOOP owns the Localizer update
`start()` / `tick()→ExitReason`; the loop owner runs `localizer.update()` then `motion.tick()`
— A2's documented controller-first shape, exactly what C2's scheduler will formalize. Rejected:
motions updating the Localizer themselves (double-update the tick the loop also does it;
ownership ambiguity C2 would inherit); motions owning a loop/task (single-task-by-contract, and
the scheduler is C2's whole job).

### D2 — Wait-for-live = WAIT bounded by the watchdog (not refuse, not fault) — §2.1
With capture-at-first-live-tick and the DriveBrake exemption. The deciding argument: an auton
issued during calibration is the NORMAL competition case (A3's survival suite waits the same
way), and an unbounded wait is a hang — so the watchdog spans the wait. Consequence accepted
and documented: callers budget timeouts to cover boot.

### D3 — `OdoStallCheck` is owned by every motion, and a stalled motion raises but does NOT self-abort — §2.2
Rejected: an optional check (the forgettable-safety-step lesson from A1's `emitRecord`); a new
`ExitReason::Stalled` (additive to a contract C2 consumes — the brief's "extra exit conditions
are additive later" path stays open, but cancellation POLICY is scheduler territory; named
handoff §10). The watchdog bounds the damage meanwhile — observed: full-speed runaway on a dead
encoder is capped at `timeout`, with the fault latched ~0.3 s after the freeze.

### D4 — Every motion ticks the HealthMonitor itself, with everything reachable
imuReady, odomStalled, odomImplausible (via the new additive `Localizer` forwarder), fixGated,
batteryVolts, maxMotorTempC — so ODO_STUCK / IMU_LOST / BROWNOUT / GPS_GATE_REJECT /
MOTOR_OVER_TEMP surface DURING a motion with zero caller wiring (all five observed in tests
through exactly this path). Rejected: leaving monitor wiring to the loop owner (every test and
every user re-derives it; the A3 containment becomes optional in practice). The active motion
IS the loop at C1; between motions the owner ticks it (C2 formalizes — §10).

### D5 — The pipeline order, and ONE translation gain set for both field axes
Errors → three decoupled PIDs → |ω| clamp + **norm-cap** on (vx,vy) → `fieldToRobot` (the one
F1 call) → body-vy authority clamp → `toWheels` (unclamped, F5) → `desaturate` → FF →
`compensateForBattery`. Norm-capping (not per-axis clamps) preserves commanded direction —
per-axis clamps would curve every diagonal. One shared translation gain set is load-bearing:
unequal x/y gains would make closed-loop behaviour depend on FIELD orientation — the rotation-
equivariance test (outcome rotates with the problem to 0.02 in) would red. An axis-asymmetric
ROBOT is a body-frame property and belongs in kinematics/FF, not here.

### D6 — Heading PID encoding: `pid.update(0, −errH)` with `errH = heading.errorTo(target)`
The wrap is absorbed by F3's shortest error BEFORE any controller sees a number, the PID's input
is continuous near the target, and D-on-measurement stays live for a future kD. Rejected: raw θ
subtraction (mutation #7 shows the long-way-around turns); feeding `update(errH, 0)` (kills the
derivative silently — a landmine for whoever first sets kD).

### D7 — MoveToPose composes TWO SettledUtils + ONE Watchdog instead of the 1-scalar ExitGroup
Arrival requires translation-norm AND heading criteria; ExitGroup's single scalar can't carry
both without inventing a normalized blend whose rate criterion means nothing. Same tested
primitives, 2+1 composition, ExitGroup's Settled-beats-simultaneous-TimedOut priority preserved;
TurnTo and DriveBrake, whose exits ARE one scalar, use ExitGroup unchanged (constraint 6 kept
where it applies). Mutation #10 proves the second criterion is load-bearing.

### D8 — HoldPose's deadline verdict: Settled iff currently within tolerance, else TimedOut
"Held the clock out while 10 in off" must not read as success. Rejected: always-Settled at the
deadline (lies), always-TimedOut (marks every successful hold a failure). Pinned both ways
(recovers-from-shove → Settled; pushed-through-the-end → TimedOut + MOTION_TIMEOUT).

### D9 — DriveBrake: 5-tick vector-averaged twist + a threshold above the measured floor — §4.2
Fix placed in the consumer, not the estimator.

### D10 — `DebugRecord.commanded` = the FINAL ACHIEVABLE command, expressed in the FIELD frame
The record's pose-and-control section is field-frame by schema; storing
`robotToField(bodyClamped)` keeps that honest AND makes the authority clamp auditable from
records (`fieldToRobot(commanded, heading).vy` recovers the clamped body vy — the mutation-#5
detectors are built on exactly this). Rejected: storing the raw pre-clamp demand (hides what was
actually asked of the drivetrain); storing body-frame values in a field-frame section.

### D11 — Strafe authority interpreted as |body vy| ≤ authority · maxLinearSpeed — **flagged for C3**
F5's text says "max sustainable |vy|/|vx|", which is ill-defined at vx = 0 (a pure strafe).
Physically the H-drive's strafe wheel tops out at a fraction of the main drive's speed
independent of vx, so the clamp base is the linear speed budget. Consistent with tank (0 ⇒
vy ≡ 0) and X (1.0 ⇒ no extra constraint). The fractional-authority test (0.35) pins the C1
side of the contract; **C3 must confirm this reading against the real H-drive kinematics** —
if C3 needs the literal ratio form, the clamp is one line in one place.

### D12 — Tank reachability honesty: no turn-then-drive planning, anywhere
On a strafeAuthority-0 drive the decoupled controller can reach targets along its heading line
(forward AND reverse — pinned) and cannot reach lateral offsets — StrafeTo on tank exits
TimedOut rather than hanging or lying (pinned). Building nonholonomic turn-toward-target
planning into C1 would smuggle the LemLib behaviour into the library's core; C3's H-drive
fallback is the sanctioned, telemetry-visible exception. Tank tests therefore use
kinematically-feasible targets, and this scope line is stated rather than hidden.

### D13 — No motion profiles at C1
The P-velocity-cap loop already produces trapezoid-ish motion (cruise at cap, exponential
approach), and every DoD item is provable without profiles. `TrapezoidProfile` stays ready;
`moveToPoseProfiled` is named at F4/C4 in the plan. Rejected: three per-axis profiles with
mismatched durations (re-couples the axes through time), one path profile (a trajectory
follower — a different, bigger chunk). The scaling test pins the velocity-capped time law so a
future profiled variant has a baseline.

### D14 — Provisional defaults shipped as REGISTERED claims (HA-50/51/52)
Gains, budgets, tolerances, stall thresholds, and the stand-in radii all carry
`PROVISIONAL (A4: HA-nn)` labels in-header and three new falsifiable register entries (counts
49 → 52), per Phase C's rule and A4's maintenance convention — the first post-A4 chunk to use
the pipeline. Not tuned beyond convergence, per the brief's "explicitly rejected."

---

## 6. Test inventory (what each would catch — every case names its bug in-file)

**motion_primitives_test.cpp (20)** — reaches varied field poses incl. across-the-seam
(pipeline sign/frame/FF breaks); **the decoupled proof** (turn-then-drive sequencing — ≥30%
translation must be done when the heading arrives, plus mid-flight rotation progress; mutation
#2's home); StrafeTo holds heading within 2° THROUGHOUT (translation→heading cross-coupling);
TurnTo in place on both drivetrains (a turn that translates); tank fwd/reverse with the
record-audited vy≡0 (wheel-order errors; mutation #5 detector 2); tank StrafeTo→TimedOut+audit
(settling on an unreached target); exit discipline (post-exit ticks re-command or verdicts
flap); watchdog+ODO_STUCK on total traction loss (mutations #3/#4); DriveBrake stops-and-stays
(false "stopped"); HoldPose recovers from a shove / TimedOut when pushed through the deadline
(dead hold loop / the held-the-clock-out lie); composition lands on B's ABSOLUTE target
(relative-target chaining); scaling Δt ≈ Δd/vmax (position-as-velocity miswiring ⇒ 4×);
degenerate zero-distance/sub-tolerance/dt==0/zero-tolerance/negative-tolerance/preconditions;
reuse (stale state across start()); TermSink legibility (record stream bypass; mutation #12).

**motion_frames_test.cpp (6)** — heading sweep (heading-dependent miss = any frame bug);
**rotational equivariance** (rotate the entire problem 37°/−113°/90°/180°: outcome rotates to
0.02 in, duration within 0.05 s — catches transform swaps AND hidden field-axis asymmetry);
**mirror equivariance** (chirality errors that commute with rotations but not reflections);
±180° seam short-way from both sides + crossings (long-way turns; mutation #7's home); the F3
exact-antipodal +π CCW tie-break observed in TRUE rotation, twice (nondeterministic antipodal
resolution); heading wrapped in (−π, π] every tick of a seam-crossing move (unwrapped escape).

**motion_sweep_test.cpp (6)** — 24 seeded X-drive random start→target trials with per-tick
universal invariants (termination, |V| ≤ 12 AND ≤ battery, FF-implied wheel speed ≤ budget —
mutation #8's home, wrap, per-tick truth delta ≤ vmax·dt·1.25, finiteness, bounded regression +
arrival); 12 seeded tank along-axis trials (same invariants); saturation case where the wheel
budget must BIND (desaturation missing or vacuous); **the fractional-authority (0.35) drive**
(mutation #5's closer — the C3 contract shape); **the weak-pack (8 V) ceiling** (mutation #11's
closer); abandon-A-start-B mid-flight (state leakage across motions — the C2 cancel shape).

**motion_hostile_test.cpp (13)** — the §2.1 wait-for-live quartet; the §2.2 frozen-encoder
pair (cross-check catches what the estimator provably cannot; pure TurnTo immune); **the
settled-vs-truth divergence matrix** (§8; every family + composed × 3 seeds, finiteness
REQUIREd every tick); HoldPose + DriveBrake under full composed hostility; sentinel breach
mid-motion (finite pose throughout, ODO_STUCK via the odomImplausible forwarder, gap 0.30 in
bounded); mid-run IMU dropout (Degraded does NOT gate; IMU_LOST once); brownout during a motion
(BROWNOUT first, TimedOut, run continues); thermal droop (MOTOR_OVER_TEMP through the motion's
own temperature wiring, run bounded); jitter schedule + full hostility convergence
(dt-dependence in controllers/settle logic).

**motion_routine_test.cpp (4)** — §3: the clean count-flat sweep + bias + settle-overhead; the
hostile bounded sweep + three regressions; the A/B count-vs-time discriminator; the 2%
miscalibration displacement-slope diagnostic. Bug named: per-move error compounding, drift
misattribution, scale bias invisibility, systematic settling bias.

**motion_stall_check_test.cpp (9)** — §2.2's verdict logic in isolation: fires on
spin-without-motion (mutation #4 unit home); silent on honest driving; silent on pure rotation
(mutation #9's home); catches a stall ON the ±180° seam (raw Δθ would read 358° of phantom
motion and MASK it); 70%-slip margin; verdict clears on recovery (episode re-arm); stationary
is not stuck; reset() vs teleports; config rejection.

Honesty note: two cases' "bug" is partially indirect — the plant does not model brake modes, so
DriveBrake's `BrakeMode::Brake` line is pinned by state inspection, not by physics; and the
tank vy-audit is vacuous-by-kinematics on tank itself (why the fractional-authority drive
exists — §7 #5).

---

## 7. Mutation checks (12 — each executed: break → build → run → OBSERVE → restore → re-green)

> **The two that came back GREEN are the chunk's most valuable findings.** A mutation that
> stays green is not a passing grade; it is a hole in the suite. Both were closed with new
> tests and re-run RED. Baselines shift 485 → 486 → 487 as the hole-closing tests landed.

| # | Mutation | Required? | Observed result |
|---|---|---|---|
| 1 | `fieldToRobot` → `robotToField` in the pipeline | yes | **RED** — 15 cases / 15 fatal assertions: heading sweep, BOTH equivariance cases, varied-starts, decoupled proof, composition, routines, hostile divergence, jitter, abandon-retarget |
| 2 | Turn-then-drive sequencing (zero translation while \|errH\| > 0.05) | yes | **RED** — exactly 1 case: the simultaneity proof (translation fraction ~0 at heading-arrival vs required > 0.30). Specific by design: sequencing is *slower, not wrong*, so only the simultaneity pin can see it — everything else still converges |
| 3 | Watchdog defeated (both expiry checks short-circuited) | yes | **RED** — 7 cases / 15 assertions: unreachable-target, never-live wait, tank StrafeTo, zero-tolerance, frozen-encoder, brownout, thermal. Bounded-loop test shape means a defeated watchdog reads as red, not as a hung suite |
| 4 | ODO_STUCK cross-check dropped (verdict forced false) | yes | **RED** — 3 cases / 3 assertions: the frozen-encoder A3-model case, unreachable-target, HoldPose push-equilibrium |
| 5 | **Strafe-authority clamp dropped** | extra | **GREEN on first run — A HOLE.** Blind because the clamp is structurally unobservable on tank (`toWheels` discards vy regardless) and vacuous on X-drive (authority 1.0 ⇒ limit == norm cap). Closed with (a) a fractional-authority (0.35) drive — X geometry, declared authority 0.35, the exact contract shape C3's H-drive arrives with — whose lateral move must RIDE the 21 in/s limit and never exceed it, and (b) a record audit on the tank StrafeTo attempt. **Re-run: RED — 2 cases / 150 assertions** |
| 6 | Wait-for-live gate dropped (`live` forced true) | extra | **RED** — 9 cases / 29 assertions: all three wait pins (drove during calibration; "settled" at the phantom origin; captured garbage heading) + composed/divergence/jitter/sentinel/routine cases |
| 7 | Raw heading subtraction instead of F3 `errorTo` | extra | **RED** — 2 cases / 6 assertions: the seam case observed LONG-WAY rotation on every crossing; the hostile routine's worst-heading bound blew |
| 8 | `desaturate()` dropped from the pipeline | extra | **RED** — 2 cases / 2 fatal assertions: the saturation case and the 24-trial sweep both tripped the FF-implied wheel-speed budget REQUIRE |
| 9 | Stall check's rotation term dropped | extra | **RED** — 1 case / 90 assertions (the pure-rotation unit case, every window). The integrated TurnTo case survives via the §4.5 phantom-translation physics — the unit case is the designed detector precisely because it controls its observables |
| 10 | Heading settle criterion dropped (exit on translation alone) | extra | **RED** — 3 cases / 4 assertions: the sweep's heading-arrival REQUIRE and both routine sweeps (waypoints "settled" mid-turn) |
| 11 | **`compensateForBattery` dropped** | extra | **GREEN on first run — A SECOND HOLE.** Blind because on a nominal 12.6 V pack the compensation never binds (motion demands cap ≈ 11.4 V), and the brownout path was masked by the hostile model zeroing EFFECTIVE volts regardless of the command. Closed with a weak-pack (8 V) run: every commanded voltage must respect the battery ceiling, the ceiling must genuinely engage, and the motion still completes (voltage-starved, flagged-not-faked). **Re-run: RED — 1 case / 1 fatal assertion (11.4 V commanded vs the 8 V pack)** |
| 12 | TermSink `[MOT]`-discriminator fix reverted | extra | **RED** — 1 case / 1 assertion: the legibility test saw "[LOC] idle" for an active motion again. (Process note, recorded honestly: an over-eager `git checkout` during cleanup also reverted the legitimate fix; caught, re-applied, re-verified) |

After each restoration the full suite was rebuilt and re-run green; final state: motion headers
byte-identical to pre-campaign pristine copies (diff-verified), no mutation markers in
`include/` or `test/` (grep-verified; the one "MUTATION" hit is A2's pre-existing doc comment in
`truth_integrator.hpp`), suite green **487 / 858,611**.

---

## 8. The settled-vs-truth divergence numbers (what "arrived" is worth under hostility)

A settled exit is a claim made from the ESTIMATE; these are the worst observed gaps between
that claim and ground truth at the settled instant (MoveToPose + StrafeTo on a 30 in
diagonal-with-rotation; TurnTo on a 75° turn), per family and composed:

| Hostile family | worst settled gap (truth vs estimate) | worst true miss (truth vs target) | worst heading miss |
|---|---|---|---|
| imu (calibration + drift + noise) | 0.012 in | 0.14 in | 0.63° |
| encoders (quantization) | 0.00008 in | 0.23 in | 0.74° |
| power (sag + discharge) | ~1e-13 in | 0.23 in | 0.74° |
| slip (accel-triggered) | ~1e-13 in | 0.24 in | 0.76° |
| latency (delayed channels) | **0.624 in** | 0.67 in | 0.69° |
| **composed, 3 seeds** | **0.568 in** | 0.57 in | 0.67° |
| sentinel breach (+∞, mid-motion) | 0.302 in | — (settled) | — |

Reading: quantization/power/slip cost the estimator ~nothing (A2/A3 already explained why —
tracking wheels ride through slip; sag shapes volts, not sensing); **latency is the dominant
single-family lie** (the stale-window physics A3 pinned in §3.6 — E2's compensation target),
and the composed world lands where latency + imu predict. Every value is far inside the
asserted 1.5 in / 2° bounds; per-tick finiteness was REQUIREd throughout every run.

---

## 9. What we now know for certain, and what we do not

*(Written for a reader who was not here. "Certain" = proven by a passing, mutation-guarded test
against plant ground truth, across the swept space described.)*

**Now known for certain — on the A2 plant, under A3's hostile world:**
- **The library drives.** Five primitives reach their targets and settle, on X-drive and (for
  feasible targets) tank, from arbitrary starts, seeded-swept across the field.
- **The holonomic thesis is real code, not a slogan.** Translation and rotation demonstrably
  progress simultaneously in one motion; the sequenced alternative fails the suite (mutation
  #2). A pure strafe holds heading within 2° throughout.
- **Frame handling is equivariant.** Rotate or mirror the entire problem and the outcome
  rotates/mirrors with it to 0.02 in — the strongest structural statement available that no
  frame/chirality bug survives. The ±180° seam is handled shortest-way with the F3 antipodal
  tie-break observed CCW, deterministically.
- **No motion can hang, and every exit says why.** Watchdog-bounded through every path
  INCLUDING the boot wait; ExitReason discipline pinned (never Running after exit; post-exit
  ticks are safe no-ops; motors left stopped).
- **Motion before the estimate is live cannot happen** — zero volts through the boot window,
  proven to the tick, with estimate-derived targets captured only once live.
- **A dead encoder cannot cause a SILENT runaway.** The estimator is provably blind to it
  (re-confirmed); the spin-vs-motion cross-check raises ODO_STUCK within ~0.3 s and the
  watchdog bounds the damage. Pure turns are immune to false positives.
- **Chained routines do not compound error.** 40 hand-chained moves end as accurately as 5 on
  the clean plant (0.24 in, tolerance-class, no directional bias); under full hostility the
  worst end error over ~95 s / ~900 in is 4.1 in and is attributable BY MEASUREMENT to
  time-drift (0.028 in/s), not to move count (clean-flat) or distance (0.003 in/in, and the 2%
  miscalibration diagnostic recovers an injected scale bias exactly).
- **Every A3 pathology surfaces as its fault code DURING a motion** with no caller wiring, the
  pose and volts stay finite through all of it, and a settled exit's lie is bounded and
  measured (§8, worst 0.62 in single-family / 0.57 in composed).
- **The run is legible** — one framed TermSink line per tick, `[MOT]`-tagged (after fixing the
  A1 defect that would have labeled every C1 motion "idle").

**NOT yet known — and who owns finding out:**
- **Whether any of the numbers survive a real robot.** Every gain, budget, tolerance, stall
  threshold, and radius is a registered guess (HA-45, HA-50/51/52); the plant proves logic, not
  constants. R5 tunes; R4 measures the noise floors the tolerances must clear; R6 back-fits.
- **The true drift story.** The 0.028 in/s time-slope is under a MODELED worst-case IMU
  (HA-20). Real drift may be better or worse; Phase E's correctors (and the field's GPS strip)
  are the designed answer either way. At M2, dead-reckon-only accuracy over ~90 s is
  honestly ~2–4 in, not the F2 endgame numbers.
- **The strafe-authority interpretation** (D11) awaits C3's H-drive — the fractional-authority
  test pins C1's side of the contract; C3 confirms or renegotiates (one line, one place).
- **Scheduler-grade behaviour**: cancellation, fault-reactive abort, async waits, and command
  ids are C2's; C1 proved only the by-hand shapes (abandon-and-restart, hand-chaining).
- **Anything about real braking** — the plant does not model brake modes; `BrakeMode::Brake` is
  commanded but its physics are unverified until hardware.
- **Blending / non-stop waypoint traversal** — out of scope by design; its absence now has a
  measured price (≈1.19 s/motion) for the Frontier decision.

---

## 10. Deliberately left for later chunks (named handoffs)

- **→ C2 (`MotionScheduler`)**: (1) **fault-reactive cancellation policy** — C1 motions raise
  ODO_STUCK/BROWNOUT/etc. mid-motion but deliberately do not self-abort; the watchdog bounds
  damage until `cancel()` exists (D3). (2) **`activeCommandId` assignment** (the TermSink line
  prints `cmd#0` until then). (3) **Monitor ownership between motions** — the active motion
  ticks HealthMonitor; the scheduler owns the gaps (and LoopMonitor placement). (4) **The
  formalized loop** — everything hand-chained in the routine tests (sequencing, per-move
  timeout budgeting) becomes engine code. (5) A possible additive `ExitReason` extension
  (e.g. Cancelled/Stalled) via the documented additive path.
- **→ C3 (`HDriveKinematics`)**: confirm D11's authority-clamp interpretation; the
  fractional-authority sweep test is the ready-made contract pin; `strafeFallbackActive`
  telemetry lands there.
- **→ C4 (`Chassis` facade)**: wrap `MotionDeps` + `MotionConfig` + the primitives into the
  public verbs; the profiled variant (D13) if C4's DoD wants it.
- **→ C5**: the per-motion result line + session header — the record stream already carries
  target/error/exit-state per tick; C5 formats the summary.
- **→ Phase E**: estimator-side stall detection (supersedes the cross-check's load-bearing
  role); latency compensation (the dominant settled-lie family, §8); the correctors that turn
  the 0.028 in/s drift slope into the F2 numbers; a possible smoothed-twist accessor if more
  consumers hit the §4.2 noise floor.
- **→ R3/R4/R5/R6**: settle HA-50/51/52 (and the §4.2 floor is a measurement target: real
  estimator twist noise at a dead stop).
- **→ Frontier (blending)**: min-velocity waypoint blending, now with its measured cost.

---

## 11. Freeze Register note (documentation contract #6)

**No freeze at C1.** But **F6 (`Chassis`) is approaching** — C4 builds the facade, D1 exercises
it a second time, D2 freezes it — and C1 establishes shapes the facade will inherit, flagged now
so F6's review checks them deliberately rather than absorbing them by accident:

- **The `MotionDeps` bundle** (ctx + localizer + kinematics + faults + health) is the
  constructor argument set of every primitive — the facade will hold exactly this set; if F6
  wants a different composition (e.g. owning the Localizer), C4 is the time to say so.
- **The `IMotion` tick contract** (loop-updates-localizer-first; post-exit no-op; re-armable
  `start()`) is what C2's scheduler AND the facade's blocking verbs will both build on — it
  gets its second consumer at C2, BEFORE F6.
- **The `MotionState` vocabulary** is now on the wire path (`activeCommandState`) and is
  append-only from here (F9 serializes it at H1).
- **The saturation choreography** (norm-cap → F1 rotate → authority clamp → toWheels →
  desaturate → battery comp) is the de-facto command path the facade's `drive(ChassisSpeeds,
  Frame)` verb must reuse, not re-derive — one pipeline, one place.
- **The wait-for-live contract** becomes facade-level API semantics ("moveTo issued during boot
  waits") — F6's documentation must carry it forward.
- `LocalizerConfig`/`Localizer` gained one additive accessor; `FaultCode` unchanged; F1–F5
  untouched (F5's no-clamp rule is now exercised from above by a second consumer — the motion
  layer — and held).

---

## 12. Verification (actually run, outputs as observed)

```text
$ cmake --build build/test && ./build/test/shulib_tests
[doctest] test cases:    487 |    487 passed | 0 failed | 3 skipped
[doctest] assertions: 858611 | 858611 passed | 0 failed |
[doctest] Status: SUCCESS!
```
(3 skipped = the two M3 acceptance stubs + the R3 GPS field-cal oracle (HA-01), unchanged.
Full suite wall time ≈ 0.7 s.)

```text
$ <the ci.yml PROS-free guard grep, scope now incl. include/shulib/motion>
GUARD 1 PASS: core is PROS-free (incl. motion/)
$ <the ci.yml layering guard grep, scope now incl. include/shulib/motion>
GUARD 2 PASS: layering holds, core (incl. motion/) is sim-free
```

```text
$ arm-none-eabi-g++ -std=gnu++20 -Wall -Wextra -Wconversion -Wsign-conversion -Wshadow \
    -Werror -Os -mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=softfp \
    -c all_headers.cpp -o /dev/null -Iinclude        # generated list, ALL v2 headers
ARM CROSS-COMPILE: CLEAN (85 headers)
```

Independently verified and committed as checkpoint `7e54826` (tree was left uncommitted per the
brief; the checkpoint was made by the coordinator after verification, with this record noted as
the outstanding deliverable).

---

## 13. DoD checklist (brief §Definition of Done)

- [x] **`IMotion` defined; `MoveToPose`, `TurnTo`, `StrafeTo`, `driveBrake`, `holdPose`
  implemented** — `include/shulib/motion/` (8 headers); DriveBrake/HoldPose are classes
  (`DriveBrake`, `HoldPose`) per the naming conventions of the tree.
- [x] **Each drives the A2 plant to target and settles, on both X-drive and tank** — with the
  honest tank qualification stated in D12: kinematically-feasible targets (along-heading,
  fwd/reverse) — a strafeAuthority-0 drive cannot reach lateral offsets and C1 deliberately
  contains no turn-then-drive planner; StrafeTo-on-tank times out honestly, by test.
- [x] **Decoupled per-axis motion demonstrated** — the simultaneity proof + the
  heading-holding strafe; mutation #2 red.
- [x] **Every motion reports a correct `ExitReason` and is watchdog-bounded** — exit-discipline
  + watchdog cases; mutation #3 red (7 cases).
- [x] **Wait-for-live-estimate contract defined, documented, enforced, tested** — §2.1;
  mutation #6 red (9 cases).
- [x] **`ODO_STUCK` spin-vs-motion cross-check implemented and tested** — §2.2, under the real
  A3 hostile encoder model; mutations #4 and #9 red.
- [x] **All primitives survive A3's composed hostility — fault, never diverge** — the
  divergence matrix + composed HoldPose/DriveBrake + jitter, finiteness REQUIREd per tick;
  worst settled lie 0.62 in, measured and reported (§8).
- [x] **The run is legible through `TermSink` as it executes** — one framed `[MOT]` line per
  tick, after fixing the A1 discriminator defect that would have labeled it all "idle"
  (§4.1); mutation #12 red.
- [x] **Any shipped gain has an `HA-nn` register entry** — HA-50/51/52 added (register
  49 → 52 entries, reconciliation grep-verified both directions).
- [x] **Suite green under strict `-Werror`; both CI guards pass; ARM gate passes** —
  487/858,611; both guards with `motion/` in scope; 85/85 headers (§12).

Beyond the brief, at the user's escalated bar: the full-routine accuracy suite with three-way
error attribution (§3), the settled-vs-truth divergence matrix (§8), equivariance testing,
per-tick universal invariants over seeded sweeps, and a 12-mutation campaign that found and
closed two real suite holes (§7).
