# R5a (handoff L7) — gains and noise, measured by the robot inside shulib Calibrate and kept on the SD card

> **THE TEAM LEAD'S ANSWERS (2026-09-26, ~23:10): every recommendation accepted** — "Accept all" on
> D1–D12 as written below. Builders start after tonight's claw and auton merges (his ruling of ~14:15:
> "briefs now, build after tonight"); each part still starts on its own go, at its turn in the order
> the brief gives. Committed by the coordinator with its plan entries.

> **Execution brief.** Written 2026-09-26 from a read-only pass over the tree at `a01e3a2` (branch
> `shulib-v2`), recovered after the power loss, and **re-checked the same evening** against `a01e3a2`
> and the two worktrees in flight (`t3-part2`: uncommitted on `cc513a7`; `g1-2c-final`: `80e2615` plus
> uncommitted edits). **Status: WAITS for the team lead's go and for his answers to D1–D12 below.
> Nothing is built.** Plan items: build-order **R4** (the rest capture), **R5** (the ramp and the fit),
> **E5** (persistence); handoff **L7**; the team lead's **Q7 ruling of 2026-09-26** (persist to the SD
> card at runtime), recorded below as a deviation from R5's DoD. **Four parts — A, B, C, D — each its
> own executor run, verification, documentation pass and commit, each started only on its own go.**
> **Executor model: `opus`. Do not commit.** Live log: `docs/internal/chunks/R5a-PROGRESS.md` (the id
> is D8's; rename it if the team lead picks another), **created FIRST and appended as you go**, one
> section per part. The measurement itself is a robot session (handoff S2 gains, S3 noise); every part
> of this chunk is proven on the host test plant only, and its documents must say so.

## Re-check 2026-09-26 (evening) — amendments

The checker's 15 findings (`findings-L7.json`), each re-verified against the tree, the host test
plant's model (`motor_model.hpp`, `drive_plant.hpp`, `imu_hostility.hpp`) and the worktrees' diffs.
The numbers quoted below come from a probe that mirrors `sim::MotorModel`
(`docs/internal/evidence/brief-probes-2026-09-26/R5a-probe.py`). It is not project code.

1. **(major) Equal volts plus a 20° yaw guard: REAL, fixed at the root.** On the plant, the draft's
   per-side gains curve the robot 41–52° per segment, so the draft's own DoD case could never finish.
   The guard now reads the ENCODERS, (dR − dL)/W, and is sized from the asymmetry it must tolerate.
   The IMU yaw is recorded beside it and never refuses (new **D9**; A.3; the tests use gains about 5 %
   apart in both orientations, plus a separate refusal case).
2. **(major) "Parts A and B touch no file in flight": REAL.** The in-flight list was rebuilt from both
   worktrees' actual diffs. Part B now waits for 2c's merge (`robot.hpp`, `robot_config.hpp`,
   `robot_builder_test.cpp`). Part C also waits for T3 Part 2 and SAFE-SAVE A and B1, and Part D for
   SAFE-SAVE B2 (see the reconciliation note below). Part A's doc edits are
   merged by the coordinator (D8, the in-flight section).
3. **(major) kA is fitted but nothing applies it: REAL.** The only feedforward call in the motion
   layer is the cruise overload (`command_pipeline.hpp` ~:149; `feedforward.hpp` ~:76-81). The brief
   now says "recorded, not applied" in §0, D2, `describeGains`, HA-172 and HA-174. The team lead gets
   the step segments as a decision (**D10**, recommended KEEP: without them the discrete fit is
   rank-deficient at kA = 0).
4. **(major) HA-173 and `√2·σ/dt` assume white heading noise: REAL.** `settleFloorReport` now uses the
   σ of the per-tick heading step, which is exactly what `SettledUtil` differentiates. The drift is
   reported with its σ under both noise models, plus a structure ratio. HA-173 is restated for both
   cases. "The IMU's noise structure" is added to what the plant cannot prove. The capture length
   becomes a D5/HA-170 choice.
5. **(major) The coast-down is neither bounded nor visible: REAL.** One correction to the finding: the
   plant does not pin a wheel when kA > 0. It decays it with τ = kA/kV, about 3.6 in at the test gains.
   But it models neither a V5 Coast nor a V5 Brake. Fix: a NORMAL segment end now uses 0 V + Brake
   (HA-53, HA-142) and Coast is kept for refusals and aborts. Each segment's stopping distance is
   measured, and one longer than `stopBound` refuses the next segment. The clear space is now derived
   from these numbers (new **D11**).
6. **(minor) A 10°/min bias against a 3° `Turned` bound: REAL.** `Moved` and `Turned` now come from the
   encoders, never the IMU, and a mutation covers it.
7. **(minor) The filtering of `velocity()` is not registered: REAL.** The CSV gains each side's median
   position. A mismatch case adds a first-order-filtered `velocity()`. "velocity() is current within a
   tick" is folded into HA-171.
8. **(minor) No temperature guard: REAL.** A new `TooHot` refusal fires at the start of a segment,
   above `tempGuard` (45 °C, below HA-44's 55 °C). The largest member temperature is recorded for
   every segment. In HA-172, the per-motor current limit with 12 motors plugged in is marked
   UNVERIFIED.
9. **(minor) Nothing keeps the record's CRC: REAL.** The seam is now named (C.1, D). The reader of the
   chosen drivetrain record returns that file's stored CRC. The binding is judged against the file
   the boot actually chose, which may be SAFE-SAVE's backup.
10. **(minor) Landmine 5 is wrong about the plant: REAL.** At u = 0 the plant decays with no friction
    term. Admitting u = 0 samples moves kS from 1.00 to 0.53 on the plant (probe). The landmine is
    reworded, and both mutation cases stay.
11. **(minor) D3/B.4 leave rules unstated: REAL.** The budget is now exact: 79/7 V, not "11.29 V",
    which gives 60.025 in/s. One voltage-preserving rule is stated against the config's own `wheelFf`
    and budgets. `valid()` refuses a measured characterization on a non-tank drivetrain. A
    non-positive derived budget boots on PLACEHOLDER. The `maxAngularSpeed` mutations are added.
12. **(minor) Part D omits the invented X-drive: REAL.** `src/xdrive_program.cpp` (2c) is named. It
    prints PLACEHOLDER, because GAINS measures a tank only.
13. **(minor) Some mutations survive: REAL.** Every per-side plant case now runs in both orientations.
    A stale-binding merge mutation is added. The largest mismatch that `PoorFit` misses is pinned and
    recorded in HA-171, instead of claiming that PoorFit catches mismatch in general.
14. **(minor) The documentation list is incomplete: REAL.** Added: HA-51's settle line, and HA-52 left
    unamended with the reason stated. Also guide ch.08, ch.03 (prose review), roadmap ~:1124, the
    handoff's Q7 line, and a `ci.yml` step for `sysid_refit.py self-test` (Part C, after 2c). The tool
    must be stdlib-only.
15. **(minor) Mis-citations and a coordination gap: REAL.** Command latency is unregistered, not
    HA-24/25, and HA-171 now carries it. "No pointers" is `robot_config.hpp` ~:35. SAFE-SAVE (drafted
    this evening) does own the safe save, which becomes **D12**. "R5a" is kept (D8), with a note on the
    grep.

**Found while re-checking (not in the findings):**
- (R1) The draft's `α ≤ 0 ⇒ kA = 0` test gives kA ≈ 5·10⁻⁵ instead of 0 when a kA = 0 plant fits α = +10⁻¹³.
  The probe fitted −1.2·10⁻¹³, so which sign comes out is floating-point luck. The test is now a
  resolution threshold (`alphaMin`, A.1).
- (R2) Without the step segments, the discrete fit is RANK-DEFICIENT at kA = 0 and about 40× worse
  conditioned at kA > 0. Once the ramp's start transient decays, v, u and sgn(v) are collinear. See
  D10.
- (R3) Every sample with 0 < |u| ≤ kS and |v| ≥ `minSpeed` also violates the fit's model on the plant.
  The plant's friction follows the STEADY-STATE sign (`motor_model.hpp` ~:41-44). The segments never
  produce such a sample. Landmine 5 now covers it.
- (R4) SAFE-SAVE's draft finds that the brain has no rename or delete (its HA-166 (1)–(3)). Its Part B1
  REMOVES `saveAndReadBack()` (`calibrate_program.cpp` ~:627-684), the pattern this draft's C.3 cited.
- (R5) On robot two, D7's start-screen port check would list the claw motors on ports 6 and 7 as NEW,
  unless it uses T3 Part 2's declared-ports form.

**Reconciled with the revised SAFE-SAVE brief, 2026-09-26 late evening.** That brief's D4 splits its
Part B, and its D6 lists what L7 must change. The changes:
1. **The split.** SAFE-SAVE B1 is Calibrate's save, after T3 Part 2 and G1 2c-final. B2 is the boot's
   fallback and Teleop's words, after COMP-b. L7 Part C now waits for SAFE-SAVE **A and B1**, and Part D
   for **B2**. This is fixed in R4, D8, D12, the parts table, the in-flight list and the Part C/D
   headers.
2. **The load rule.** L7 no longer adds a byte-level load rule of its own. It uses SAFE-SAVE Part A's
   `loadWithBackup(store, primary, backup, accept, …)` and `chooseEffective` → `Which { Primary,
   Backup, None }`. It saves with Part A's `safeReplace<MaxBytes>(store, primary, backup, bytes,
   accept)`, which returns a `SaveReport` (Saved / NotSaved / Unconfirmed).
3. **The save lines.** "SAFE-SAVE's `formatSaveReport` wording applies" was wrong: that formatter words
   the DRIVETRAIN record. L7 now words its own lines from the same `SaveReport`
   (`formatGainsSaveReport`, C.3), and they are spelled out there.
4. **The size.** The gains file's maximum size is stated. It is fixed by its layout (about 426
   bytes), `static_assert`ed ≤ 512, and so fits `safeReplace`'s `MaxBytes <= 1024` cap at no more than
   1088 bytes of stack (B.1). A layout past 1024 fails the build.

HA-166 is now cited as SAFE-SAVE's ONE entry (five beliefs), by belief number:
- (1) `"wb"` empties an existing file at the open;
- (2) no rename and no delete;
- (3) no read-write open;
- (4) and (5) are what the gains save rests on, as the record's does.

L7 adds no HA number for any of them. Its doc pass adds one sentence to HA-166's *Source*/*Contained*
lines (3.C).

## Decisions for the team lead (answer before Part A starts; each carries a recommendation)

**Already ruled — Q7 (2026-09-26):** gains are measured by the robot at runtime and persisted on the SD
card, not committed as constants. R5's DoD (build-order ~:1861, "real kS/kV/kA for both robots committed
as constants") is therefore changed; the deviations-table row is in §3.0.

- **D1 — Where the numbers live on the card. RECOMMENDED: a SIBLING file**
  (`shulib_characterization.bin`, its own magic `"SHCH"`, its own version, CRC-32), **bound** to the
  drivetrain record by the CRC-32 that record already stores in its last 4 bytes
  (`calibration_record.hpp` ~:142). `shulib_drivetrain.bin`, its 180-byte layout and its golden stay
  exactly as they are. *Alternative:* the drivetrain record v2 (version bump, a decoder that reads v1
  and v2). *Why the sibling:*
  1. A gains run never rewrites the one file "shulib Teleop" cannot boot without
     (`boot_source.hpp` ~:14-22). The brain cannot rename a file, and `fopen "wb"` empties the file at
     the open (SAFE-SAVE §0.2; HA-166 (1)–(2)). So every rewrite of that record is a save that can fail
     half-way, and SAFE-SAVE's backup exists only to survive that.
  2. Every program already uploaded keeps booting. A v1-only decoder refuses version 2 by name
     (`calibration_record.hpp` ~:228-235), so the v2 route forces re-uploading every slot.
  3. A damaged gains file can degrade (D4) instead of refusing.
  4. The binding makes a drivetrain re-calibration void the gains automatically. That is correct,
     because the gains are in inches per second of THAT record's scale (§6 landmine 7).

  Cost: two reads at boot and a binding rule to test. The v2 route's one advantage is a single atomic
  file, and the binding supplies that.
- **D2 — Two sides, one feedforward.** The motion layer applies ONE `control::Feedforward` to every
  wheel (`feedforward.hpp` ~:47-50; `command_pipeline.hpp` ~:142-152). **RECOMMENDED: fit and persist
  each side separately; consume their field-by-field MEAN; warn when the sides' kV differ by more than
  15 %** (a dragging side or a slipping member is a mechanical finding, not a tuning). **Only kS and
  kV of that mean reach a motor.** The pipeline calls the cruise overload `ff.calculate(v)`
  (`command_pipeline.hpp` ~:149), and `WheelSpeeds` has no acceleration channel
  (`feedforward.hpp` ~:76-81). So kA is **recorded for S2 and R6, not applied** (D10). *Alternative:*
  a per-wheel feedforward in the command pipeline. That pipeline is the hot path every motion and the
  facade share, pinned bit-identical by the C2 suites. Revisit only if S2 shows an asymmetry the PID
  cannot absorb. The 15 % warning is reachable only because D9 sizes the curve guard above it.
- **D3 — What full stick means once gains are measured.** shulib Teleop scales the sticks to
  `maxLinearSpeed` / `maxAngularSpeed` (in/s, rad/s) and the feedforward turns those into volts
  (`teleop_loop.hpp` ~:208, ~:247-249). With the placeholders, full stick forward costs **exactly
  1 + 60·12/70 = 79/7 V ≈ 11.2857 V**. A faster drivetrain measures a SMALLER kV, so the same 60 in/s
  costs fewer volts and full stick gets slower. A slower drivetrain asks for more than the rail, and
  per-wheel clamping then bends turns (it breaks the invariant `motion_config.hpp` ~:32-34 states).
  **RECOMMENDED: the builder derives the budgets from the measured gains so that each budget keeps the
  VOLTAGE it cost under the config's own `wheelFf`.** One rule, with `c` = the config's own typed (or
  default) values and `m` = the measured mean:
  - `maxLinearSpeed_m = (kS_c + kV_c·maxLinearSpeed_c − kS_m) / kV_m`;
  - `maxWheelSpeed_m` is the same rule applied to `maxWheelSpeed_c`;
  - `maxAngularSpeed_m = (kS_c + kV_c·maxAngularSpeed_c·W/2 − kS_m) / (kV_m·W/2)`, where W is the
    description's effective track width (the one the builder's `TankKinematics` is built with,
    `robot.hpp` ~:838-844).

  At every card boot `c` is the library default (`boot_source.hpp` ~:84-91), so the linear budget is
  (79/7 − kS_m)/kV_m. The rule has no special case for "typed versus default": a typed budget keeps
  its volts in the same way. The banner prints the derived budgets and what full stick costs. A
  derived budget that is not finite and > 0 boots on PLACEHOLDER naming it (B.2), never a throw.
  *Consequence to accept:* autonomous speed caps move with the robot's real capability. No auton has
  run, and S8 tunes on measured gains. *Alternative:* keep 60 in/s and 6 rad/s, check the rail
  invariant, warn, and decide after S2.
- **D4 — A gains file that is missing, damaged, or measured against a different drivetrain record.
  RECOMMENDED: boot on the placeholders and say so, naming the reason, on the screen and the serial
  line — never refuse to drive.** The placeholders are what "shulib Teleop" drove robot two with on
  2026-09-25. *Alternative:* refuse, as the drivetrain record does — rejected because a gains problem
  costs performance, not safety.
- **D5 — What the noise and timing numbers DO. RECOMMENDED: recorded and reported only.**
  - The rest capture reports:
    - the drift, with its σ under BOTH noise models and a structure ratio that says which model the
      data fits (A.4);
    - heading noise;
    - the **per-tick heading step's σ**, which is the rate noise `SettledUtil` actually sees;
    - yaw-rate noise;
    - encoder noise at rest;
    - loop timing;
    - how far each HA-51 settle floor clears the measured noise (a warning under 2×).
  - The settle-floor comparison uses the measured per-tick step, never `√2·σ/dt`. That formula holds
    only for white heading noise, which is the host plant's model and not necessarily a gyro's (HA-173).
  - Nothing is applied automatically: not to settle tolerances, and not to the EKF (the fall robots
    run `ComplementaryFusion`, `robot.hpp` ~:1144).
  - **The drift is never applied as a heading correction.** It is a per-boot bias (HA-20, new HA-173),
    so one boot's number is wrong at the next boot.
  - Boot-time bias estimation stays R4's.
  - The capture length (60 s) is an HA-170 choice. If S3 finds that the heading noise is a random walk,
    a longer capture helps only as 1/√T: 0.42 → 0.19 °/min from 60 s to 300 s at HA-22's numbers. The
    between-boot spread is then the settle, not the capture length.
- **D6 — The roadmap's `tools/sysid` (roadmap ~:1142).** The fit now runs on the brain. **RECOMMENDED:
  keep a small laptop re-fit, `tools/sysid_refit.py`** — an INDEPENDENT, **stdlib-only** implementation
  (CI installs only `python3`, `ci.yml` ~:34) with its own `self-test` — reading the raw samples
  Calibrate writes to the card after the motors stop (`shulib_ramp.csv`). It checks the brain's
  arithmetic against the brain's own data. Without another robot session, it can also test:
  - a one-tick motor-command latency (UNREGISTERED today; HA-171 now carries it);
  - battery sag per volt (HA-40);
  - whether `velocity()` lags the differenced position, now that the CSV carries position (HA-171).

  *Alternative:* no raw file and no tool.
- **D7 — Where Calibrate offers the new steps. RECOMMENDED: on the finished screen after SAVED, AND at
  program start when the card's record decodes and passes the registry check**
  (`calibration/boot_check.hpp`), so a gains run does not require pushing the robot again (which would
  also change the record and void the binding). *Alternative:* after SAVED only.
- **D8 — Chunk id and timing.** Proposed build-order id **R5a** (R5's ramp + R4's rest capture + E5's
  persistence; R4, R5 and E5 keep their other items, §4). A plain `grep R5a` also hits COMP-PROGRESS's
  mutation labels R5a1–R5a4 (~:340-412). A word grep does not (`git grep -w R5a` finds nothing today),
  so documents grep for the id as a word. Pick another id if that is unwelcome.
  **Timing, corrected against the worktrees:**
  - **A** may start on its go. Its CODE touches no file in flight. Its DOC edits (the register,
    roadmap, changelog, build-order, `docs/api`) are files 2c and T3 Part 2 also edit, so the
    coordinator merges them at commit, as with DIAG1 and COMP-a.
  - **B waits for G1 Part 2c's merge.** 2c edits `robot.hpp`, including the doc comment directly above
    `effectiveTuning()`, which B.4 rewrites. It also edits `robot_config.hpp` and
    `robot_builder_test.cpp`.
  - **C waits for B, 2c, T3 Part 2 and (per D12) SAFE-SAVE Parts A and B1.** B1 is Calibrate's save,
    which removes `saveAndReadBack()` from `calibrate_program.cpp`; Part C edits the same file.
  - **D waits for C, COMP-b and SAFE-SAVE Part B2.** B2 is the boot's fallback and Teleop's words, and
    edits `src/boot_source.*` and `src/teleop_program.cpp`, as Part D does. SAFE-SAVE's D4 already
    orders both of its B parts before L7.

  The handoff's order (§7 item 3) puts L7 after L2, L3 and L6. That is the team lead's call.
- **D9 — NEW: the curve guard (finding 1).** Both sides get the SAME volts. On a tank whose sides
  differ, the robot curves by about segmentDistance × asymmetry / W. At 36 in and W = 12.5 in, the
  draft's 20° IMU guard fires at about 12 % asymmetry, so D2's 15 % warning could never be reached.
  On the plant the draft's own test gains curve 41–52° per segment. The IMU also reads jumps of up to
  37° per tick in a hard reverse (HA-160). **RECOMMENDED:**
  - The guard reads the **ENCODERS**: the curve ψ = (dR − dL)/W from each side's median travel, with
    W = the description's effective track width.
  - It refuses (`CurveExceeded`) past **ψ_g = segmentDistance × `asymRefuse` / W**, computed at start.
    `asymRefuse` = 0.30 (HA-170) is twice D2's warning, so a robot that warns still finishes. At
    W = 12.5 in, ψ_g ≈ 49.5°, and the worst sideways excursion is segmentDistance·(1 − cos ψ_g)/ψ_g ≈
    14.6 in (the "what to clear" screen prints it from the record's W).
  - The refusal names both sides' travel and "a dragging side, a slipping member or something in the
    way".
  - The IMU's accumulated yaw is recorded beside the encoder curve for every segment. Their
    disagreement is HA-160 evidence and never a refusal.

  *Alternative:* the draft's IMU guard. Rejected: it refuses a mechanically lopsided robot as
  "curving into something", and it refuses a run when the IMU jumps.
- **D10 — NEW: keep the two STEP segments? RECOMMENDED: keep them.** kA is not applied (D2), so the
  steps' first job is not kA. On the plant they are what makes the discrete fit well-conditioned (R2).
  With ramp data only, v ≈ (u − kS)/kV − kA·r/kV² is affine in u once the start transient decays. So
  v, u and sgn(v) are collinear: RANK-DEFICIENT at kA = 0, and about 40× worse conditioned at kA > 0.
  The steps also give:
  - kA itself, which R6's plant back-fit needs for τ = kA/kV, and which a future acceleration channel
    would need;
  - HA-172's current plateau;
  - HA-160 evidence from the StepBackward, which reproduces 2026-09-25's hard reverse. With D9 the IMU
    cannot end the run there.

  Cost: about 2.6 s more powered driving, including that hard reverse. *Alternative:* drop the steps
  and fit the static form u = kS·sgn(v) + kV·v over the ramp only. kS then absorbs kA·a. At 2 V/s that
  is kA·rampRate/kV ≈ 0.25 V of the plant test's 1.0 V, and kA is not measured at all.
- **D11 — NEW: how a segment ENDS (finding 5).** A segment that ends normally used to go straight to
  0 V + Coast. On a V5 at 0 V the motor free-wheels (HA-143, unknown), so the robot rolls from up to
  about 50 in/s for an unbounded, unmeasured distance. The host plant shows only its own τ = kA/kV
  decay. **RECOMMENDED:**
  - A NORMAL segment end commands **0 V + Brake**, which reaches the motor since DEFECTS2 (HA-142).
    HA-53 claims that stop is prompt. The motors switch to Coast once the robot is at rest.
  - **Coast stays on every refusal and abort**: a person may be reaching for the robot (calibrator
    rule 6).
  - Every segment's **stopping distance** (per side, encoders, from the end tick to rest) is measured
    and reported, and goes in the CSV.
  - A stop longer than `stopBound` (12 in, HA-170) refuses before the next segment (`StopTooLong`).
  - The clear space is derived: segmentDistance + `stopBound` in front and behind, plus D9's sideways
    excursion.

  *Alternative:* keep Coast at every end, with the same measurement and the same bound. That is
  simpler, but the rolling distance is then the floor's, which is unknown.
- **D12 — NEW: the gains file's save and load (SAFE-SAVE's D6).** The brain has no rename and no
  delete, and `fopen "wb"` empties the file at the open (SAFE-SAVE §0.2; HA-166 (1)–(3)). So the draft's
  "write, flush, close, re-open, read back" erases a good gains file whenever the write fails. Its cited
  pattern (`calibrate_program.cpp` ~:627-684) is the function SAFE-SAVE Part B1 removes.
  **RECOMMENDED** (the same as SAFE-SAVE's D6):
  - **Save.** Part C saves through SAFE-SAVE Part A's
    `safeReplace<kCharacterizationBytes>(store, kCharacterizationFileName,
    kCharacterizationBackupFileName, bytes, accept)`. The backup is `shulib_characterization.bak`, and
    `accept` means the gains decoder succeeds on exactly `kCharacterizationBytes`. The result is
    Part A's `SaveReport`.
  - **Load.** The boot and Calibrate read the gains file through Part A's `loadWithBackup(store,
    primary, backup, accept, …)`: the primary if it decodes, else the backup, chosen by
    `chooseEffective` → `Which { Primary, Backup, None }`. L7 adds no load rule of its own.
  - **Words.** L7 words its own save lines from the `SaveReport` (C.3). SAFE-SAVE's
    `formatSaveReport` names the drivetrain record and what Teleop does, so it is not reused.
  - The gains line says when the backup was used.
  - **The binding** is judged against the drivetrain file the boot CHOSE (SAFE-SAVE D6's last
    paragraph). A boot from an older drivetrain backup therefore sees gains bound to a newer record
    and prints PLACEHOLDER by name, which is correct.
  - **What the save rests on.** Like SAFE-SAVE's, it rests on HA-166 (4) and (5). If either is wrong,
    both gains files can be damaged; the boot then uses the placeholders (D4) and names the refusal.
    It never refuses to drive.
  - **Order.** C waits for SAFE-SAVE A and B1; D waits for SAFE-SAVE B2.

  *Alternatives:*
  - SAFE-SAVE D6's own alternative: save safely, but the boot ignores the gains backup. The backup is
    then for a laptop only, and a torn gains save degrades to PLACEHOLDER.
  - Write in place with a read-back, and rely on D4. The cost is the lost measurement (the serial line
    still carries its numbers) and a second save discipline on one card.

## 0. Why

- **Every closed-loop command and every Teleop stick runs through placeholder gains.**
  - `MotionConfig::wheelFf {kS 1.0, kV 12/70, kA 0}` (`motion_config.hpp` ~:75-78, HA-45) feeds the
    one feedforward the pipeline applies per wheel (`command_pipeline.hpp` ~:142-152).
  - Teleop goes through the same pipeline via `chassis.drive()` (`teleop_loop.hpp` ~:249).
  - The pipeline applies only the **cruise** form, kS·sgn(v) + kV·v (~:149), so a measured kA is
    recorded, not applied (D2, D10).
  - `boot_source.hpp` ~:84-91 leaves `tuning` at the library's defaults on purpose — there has been
    nothing measured to put there.
- **The noise numbers that size the settle floors are invented** (HA-20/21/22; the derivation at
  `motion_config.hpp` ~:36-40 and HA-51). That derivation also ASSUMES white heading noise, the host
  model's shape (`imu_hostility.hpp` ~:108-114). The loop timing the hostile suites certify against is
  invented too (HA-32/34; `composed.hpp` ~:203-207).
- **The plan's own shape (R5 as written) cannot run on this team's robots.** It is an offline solver,
  fed by a ramp routine that nobody has, producing constants a person commits. The team lead's
  2026-09-18 framing ("calibrate absolutely everything it needs to know") and Q7 move the whole loop
  onto the robot:
  - the routine runs inside shulib Calibrate;
  - the fit runs on the brain;
  - the result lives on the card beside the drivetrain record;
  - the builder consumes it.
- **What exists to build on:**
  - the calibrator's tick-driven discipline and its seven rules (`drivetrain_calibrator.hpp`
    ~:78-108: one place for a sign, unwrapped yaw via `errorTo`, the median, coast on every refusal, a
    missed tick is a stop);
  - its rest definition and thresholds (~:262-280, ~:344-351);
  - the record's refuse-never-misread format (`calibration_record.hpp` ~:14-26, ~:123-147);
  - SAFE-SAVE Part A's store seam and its generic save and load, `safeReplace<MaxBytes>` and
    `loadWithBackup` (D12), once it lands;
  - the plant's motor model, which IS the feedforward law inverted with an exact exponential step
    (`motor_model.hpp` ~:21-31, ~:80-96);
  - the hostile IMU, with its drawn per-boot bias readable for assertions (`imu_hostility.hpp`
    ~:67-80, ~:148-150);
  - the jitter schedule (`composed.hpp` ~:203-231);
  - `LoopMonitor`'s inclusive budget (`loop_monitor.hpp` ~:12-20, ~:42-46);
  - `hal::medianInPlace` (`motor_group.hpp` ~:129);
  - the builder's `effectiveTuning()` (`robot.hpp` ~:1033-1041);
  - `FakeMotor::setTemperature` / `setCurrent` (`fake_motor.hpp` ~:39-40), which let tests script both.
- **What does not exist:**
  - any fitter (there is no `tools/sysid`);
  - a plant whose sides can have DIFFERENT gains (`drive_plant.hpp` ~:142: one `wheelFf` for every
    wheel, so a left/right swap is invisible today);
  - anything that reads the IMU at rest;
  - a place in `config::RobotConfig` for measured gains (~:527-542);
  - a reader that returns the drivetrain record's CRC. `RecordRead` (`boot_source.hpp` ~:230-236)
    carries the description and a reason only. SAFE-SAVE's planned `LoadedRecord` and
    `RecordFileRead` carry which file, the description, and per file opened/count/decoded/why/path,
    but no CRC either (C.1 appends it).

## Parts at a glance

| Part | What | Touches | Waits on | Size |
|---|---|---|---|---|
| **A** | the measurements as library: the fit, the ramp routine, the rest capture, tick timing, the plant's per-wheel gains | new `include/shulib/calibration/*` and `diag/tick_timing_stats.hpp`; `sim/drive_plant.hpp` (additive); `test/*` (new files + `calibration_test_rig.hpp`); docs (merged by the coordinator) | the go | ~1–1½ days |
| **B** | the card and the builder: the sibling record, the binding decision, `RobotConfig::characterization`, `Robot` consuming it (D3's budgets), the gains line | new `calibration/characterization_record.hpp`; `config/robot_config.hpp`, `robot/robot.hpp` (additive); `control/feedforward.hpp`, `sim/motor_model.hpp` (prose); `test/robot_builder_test.cpp`; docs | Part A **and G1 Part 2c merged** | ~1 day |
| **C** | shulib Calibrate runs them: GAINS and REST steps, the record's CRC, the safe save and its words, the raw CSV, the laptop re-fit (D6), the worksheet station | `src/calibrate_program.*`, `calibration/calibrate_panel.hpp`, new `calibration/characterization_store.hpp` (over SAFE-SAVE's `safeReplace` / `loadWithBackup`), one appended field in SAFE-SAVE's `record_store.hpp` (`LoadedRecord::storedCrc`), `tools/sysid_refit.py` (new), `.github/workflows/ci.yml` (one step) | Part B **and G1 Part 2c, T3 Part 2, SAFE-SAVE A and B1 landed** | ~1 day |
| **D** | every program says which gains it used | `src/boot_source.*`, `src/teleop_program.cpp`, the competition program, `src/drive_program.cpp`, `src/bench_r3a.cpp`, `src/xdrive_program.cpp` | Part C **and COMP-b, SAFE-SAVE B2 landed** | ~½ day |

Honest total: ~3½–4 days of executor plus verification. That is larger than the handoff's "~1–1½ days
host", which counted only the host halves, and the re-check added the curve guard, the stop, the
temperature guard and the noise-structure reporting. **Before B, C and D, re-check this brief against
the tree** (2c, T3 Part 2, SAFE-SAVE and COMP-b all change what those parts start from), the way G1
Part 2c's brief was re-checked.

## Files in flight — the executor must NOT edit these (read them if needed)

Rebuilt 2026-09-26 (evening) from the worktrees' diffs, not from their briefs.

- **T3 Part 2** (worktree `t3-part2`, uncommitted on `cc513a7`):
  - `include/shulib/calibration/boot_check.hpp` (`verifyAgainstRegistry` gains a `DeclaredMotorPorts`
    overload; the two-argument form stays);
  - `include/shulib/teleop/mechanism_binding.hpp`, `include/shulib/teleop/teleop_loop.hpp`;
  - `src/boot_source.{hpp,cpp}` (~+234 lines in the header), `src/teleop_program.{hpp,cpp}`,
    `src/mechanism_table.hpp`, `src/teleop_claws.hpp` (new);
  - tests: `boot_check_test`, `boot_source_test`, `mechanism_binding_test`, `mechanism_table_test`,
    `toggle_binding_test`, `teleop_claws_test` (new);
  - docs: `README.md`, `docs/api/*`, `docs/changelog.md`, cookbook 06, guide 07, 10, 11 and 14,
    `docs/hardware-assumptions.md`, `docs/roadmap.md`, and in `docs/internal/` the HANDOFF,
    PROJECT-BRIEFING, build-order, DEFECTS2-COMPLETED, the worksheet and T3-PROGRESS.
  - The characterizer INCLUDES `teleop/coupled_side_monitor.hpp` (not in flight), as the calibrator
    does at `drivetrain_calibrator.hpp` ~:148.
- **G1 Part 2c final** (worktree `g1-2c-final`: WIP commit `80e2615` on `cc513a7`, plus uncommitted
  edits):
  - library headers: `include/shulib/config/robot_config.hpp`, `include/shulib/robot/robot.hpp`
    (comments, including the one directly above `effectiveTuning()`), `include/shulib/diag/session_info.hpp`,
    `include/shulib/teleop/stick_mapping.hpp`, `include/shulib/hal/pros/device_provider.hpp`;
  - programs: `src/main.cpp`; `src/build_selection.hpp` (new, the ONE chain); `src/chassis_table.hpp`
    and `src/mechanism_table.hpp`; `src/calibrate_program.{hpp,cpp}` (it adds `calibrateInitialize`
    ~:1037); `src/bench_r3a.*`, `src/drive_program.*`, `src/fault_latch_hook.hpp`; and the new
    `src/state_screens.*`, `src/xdrive_config.hpp`, `src/xdrive_program.*`;
  - tests: `robot_builder_test`, `f6_signature_pin_test`, `routine_signature_pin_test`,
    `guide_examples_test`, `boot_source_test`, `chassis_table_test`, `mechanism_table_test`, and the
    new `xdrive_config_test` and `build_selection_test`;
  - tools and CI: `tools/src_build_gate.py`, `Makefile`, `.github/workflows/ci.yml`;
  - docs: `README.md`, `docs/api/*`, guide 07, 08 and 13, `docs/hardware-assumptions.md`,
    `docs/vexbuilder-integration.md`, `docs/roadmap.md`, and in `docs/internal/` the HANDOFF,
    G1-PROGRESS, the worksheet, PROJECT-BRIEFING, RESUMING and build-order.
- **COMP-b (next):** `src/competition_program.*`, the chain in `src/build_selection.hpp`, the gate's
  program table, the worksheet's Station K, the guide chapter "The competition program".
- **SAFE-SAVE (drafted and re-checked, awaiting its go; three parts, per its D4):**
  - **Part A** (the library): `hal/file_store.hpp`, `hal/pros/file_store.hpp`,
    `hal/fake/fake_file_store.hpp`, `calibration/record_store.hpp`, and `kRecordBackupFileName` in
    `calibration_record.hpp`; plus one sentence in `block_sink.hpp`, HA-166 and the FAQ;
  - **Part B1** (Calibrate's save; after T3 Part 2 and G1 2c-final): `src/calibrate_program.cpp`. It
    removes `saveAndReadBack()`, calls `formatSaveReport(report, BootReadsBackup::No, …)`, and edits
    the header comments, the `record file:` line and 2c-final's `calibrateInitialize` banner. It also
    edits the worksheet's Station G and Station 6;
  - **Part B2** (the boot and Teleop's words; after COMP-b as well, and after B1): `src/boot_source.*`,
    `src/teleop_program.cpp` (the source words, the LCD precedence), COMP's status screen, and one
    token in `src/calibrate_program.cpp` (`BootReadsBackup::Yes`). It also edits the worksheet's F.0,
    F.1 and G.6.
- **What each part may touch:**
  - **Part A's code** touches none of these. Its doc edits overlap T3 Part 2's and 2c's doc edits;
    the coordinator merges them at commit.
  - **Part B** edits `robot.hpp`, `robot_config.hpp` and `robot_builder_test.cpp` only after 2c has
    merged.
  - **Part C** edits `src/calibrate_program.*` after 2c and SAFE-SAVE B1. It reads `boot_check.hpp`
    after T3 Part 2. It appends ONE field to SAFE-SAVE Part A's `LoadedRecord`, once A has landed (C.1).
    If L7 C lands before SAFE-SAVE B2, B2's one-token flip in `calibrate_program.cpp` re-reads C's
    version.
  - **Part D** edits T3 Part 2's, COMP-b's and SAFE-SAVE B2's files. That is why D waits for all three.

## What the host test plant can prove, and what only S2/S3 can

- **The plant CAN prove:**
  - the fit's arithmetic: it recovers the plant's configured kS/kV/kA per side;
  - that left and right are fitted independently (once the plant carries per-wheel gains), in both
    orientations;
  - the units chain: a wrong inches-per-radian moves kV by exactly its ratio;
  - every safety rule of the routine:
    - the powered travel bound, on plant truth;
    - the encoder curve guard, and that a scripted IMU jump does NOT refuse;
    - the missed-tick stop;
    - Brake at a normal end, and Coast on every refusal;
    - the stopping distance measured against plant truth, and `StopTooLong`;
    - `TooHot` from a scripted temperature;
    - rest gating;
    - a fighting or slipping member;
  - that the rest capture recovers the hostile IMU's drawn bias and noise within bounds derived from
    the sample count, and that the per-tick step σ and the structure ratio come out right for a
    scripted white-noise heading AND a scripted random-walk heading;
  - the timing statistics, exactly;
  - the record's bytes, refusals and binding;
  - that the builder turns measured gains into exactly the volts the formula gives, and changes nothing
    when there are none.
- **The plant CANNOT prove, and the documents must not claim:**
  - that a V5 drivetrain follows the first-order law at all. **The plant's motor model IS the fitted
    model, so recovering it is circular by construction** (the A2 honesty boundary, `motor_model.hpp`
    ~:7-17);
  - the real numbers;
  - that the current limit and the firmware add nothing inside the ramp, or what the per-motor limit
    is with 12 motors plugged in (HA-172);
  - that `velocity()` is current within a tick and unfiltered (HA-171);
  - **the IMU's noise STRUCTURE** (white heading noise or integrated rate noise), its real noise,
    drift and continuity (HA-20/21/22/160/173);
  - **how far a V5 drivetrain rolls or brakes after a segment.** The plant models neither Coast nor
    Brake at 0 V; it applies only τ = kA/kV (HA-53, HA-143);
  - real temperatures and throttling (HA-44);
  - real loop timing (HA-32/34);
  - that 36 in is enough;
  - that gains transfer across surfaces and member counts (HA-174);
  - SD durability (HA-122; HA-166 (4) and (5), the beliefs the gains save rests on).

  **That is why Part A's tests include data the model does NOT describe** (§2.A, "model mismatch").
  The fit must say "poor fit" wherever it can. The largest mismatch it CANNOT catch is pinned and
  registered, not waved away.
- **The session (S2/S3) needs:**
  - robot two with a SAVED and REPRODUCED drivetrain record: R3d's Part 3 closed first, because the
    gains are in that record's units;
  - field tiles;
  - the clear space the "what to clear" screen prints: segmentDistance + `stopBound` in front and
    behind, and D9's sideways excursion to each side, beyond the robot's own footprint;
  - a charged pack, and the SD card;
  - motors cool (`TooHot` refuses above 45 °C);
  - GAINS twice, for reproducibility;
  - REST on ≥ 10 separate boots, for HA-20 and HA-173;
  - the serial log captured raw, and the CSV copied off the card.

## 1. Scope — IN

### Part A — the measurements, as library (host-only; no `src/` change)

1. **`include/shulib/calibration/feedforward_fit.hpp`** — the fit, pure and allocation-free.
   - **The model** is the DISCRETE form of the feedforward law, which the plant integrates exactly:
     `v[k+1] = α·v[k] + β·u[k] + γ·sgn(v[k])`, giving `kV = (1 − α)/β`, `kS = −γ/β`,
     `kA = −kV·dt/ln α`. No differentiated velocity is used anywhere: a derivative of an encoder is
     the noisiest number on the robot.
   - **Numerics:** accumulated ONLINE with a numerically stable method (incremental QR by Givens
     rotations is the suggestion). The normal equations are rejected: Σv² reaches ~10⁷ over a run, and
     the residual would be formed by subtracting numbers that large. The test in §2.A forces the choice
     without dictating it.
   - **Identifiability** (R2, D10) — the header says it: ramp-only data make v, u and sgn(v) collinear
     once the ramp's start transient decays, so the STEP segments are what condition α. A fit over
     ramp data alone is expected to say `NotExcited` at kA = 0.
   - **Admission** — each exclusion is counted by reason:
     - `u = 0`: a stop. Hardware reason: under Coast a V5 motor at 0 V may free-wheel (HA-143), and a
       Brake stop is braking; the law describes neither. Plant reason: the plant's own decay has no
       friction term (landmine 5).
     - `|v[k]|` or `|v[k+1]|` under `minSpeed`: the static-friction dead band.
     - a `dt` outside `nominalDt·(1 ± dtTolerance)`.
     - any member at or above `currentGuard` (HA-172).
   - **Verdicts** (`enum class FitVerdict : std::uint8_t`, append-only): `Ok`, `TooFewSamples`,
     `NotExcited` (singular), `SignMismatch` (β ≤ 0: the command and the reading disagree in sign, so
     the sign convention broke), `NonPhysical` (α ≥ 1, or kS < 0), `Implausible` (outside HA-170's
     ranges), `PoorFit` (residual RMS over the bound).
   - **kA below resolution is NOT an error.** When α < `alphaMin` = e^(−dt/τMin), with τMin = dt/5
     (HA-170), kA is reported as EXACTLY 0 with a `kABelowResolution` flag. That covers α ≤ 0, and also
     the ±10⁻¹³ a kA = 0 plant fits to, whose positive side would otherwise report kA ≈ 5·10⁻⁵ (R1).
   - **Output:** gains, samples, exclusions by reason, residual RMS (in/s), α, the voltage range the
     admitted samples covered.
2. **`include/shulib/calibration/drive_characterization.hpp`** — the DATA a run produces and Part B
   persists.
   - Per side: `{gains, samples, residualRms, voltsMin, voltsMax, members}`.
   - An optional rest summary, and the timing summaries.
   - `source` (`Unset = 0`, `Measured = 1`; append-only, persisted), and provenance text.
   - `combined()`: D2's field-by-field mean. Its kA is carried but no consumer applies it (D2).
   - `valid(why, cap)`, naming the field.
   - Fixed-width, constexpr-constructible, no pointers (`robot_config.hpp` ~:35's rule for everything
     in `RobotConfig`; embedded by value, per ruling B1, ~:18).
3. **`include/shulib/calibration/drive_characterizer.hpp`** — the ramp routine, a tick-driven state
   machine in the calibrator's image.
   - **Inputs:**
     - each side's members as `{IMotor*, sign ±1, port}`. The sign is applied ONCE, here:
       Calibrate's adapters are POSITIVE (calibrator rule 1), and a program with signed adapters
       passes +1;
     - each side's inches per radian and the effective track width W, from the description;
     - `IImu&`, `IClock&`, `IBattery&`, and a `CharacterizerConfig`.
   - **Phases:** `Idle → WaitImu → [Rest → segment → Stop] × 4 → Done | Aborted`. The segments are
     **QuasiForward** (ramp `rampRate` V/s from 0), **QuasiBackward**, **StepForward** (`stepVolts`
     held) and **StepBackward**, with both sides commanded the SAME volts each tick.
   - **Before each segment:** every member's `temperature()` below `tempGuard`, else `TooHot`, naming
     the port and the temperature.
   - **A segment ends** on the FIRST of:
     - either side's travel reaching `segmentDistance` (checked per side, never on the mean);
     - the ramp reaching `rampMax`;
     - `segmentTimeout`.

     Each is recorded as the end reason, and none is a refusal.
   - **Stop (D11):** the end tick commands 0 V + **Brake** on every member. The routine then waits
     for the calibrator's rest definition and sets Coast. The stopping distance is measured per side
     from the encoders (end tick to rest). A stop past `stopBound` ends the run as `StopTooLong`, with
     0 V + Coast, before any further segment.
   - **Refusals** (append-only, each naming the port where there is one):
     - `Aborted`, `ImuNotReady`, `RestTimeout`;
     - **`CurveExceeded`** (D9): |ψ| past ψ_g, where ψ = (dR − dL)/W is accumulated per tick from
       each side's median travel. The message names both sides' travel and "a dragging side, a
       slipping member or something in the way". **Never computed from the IMU.**
     - `Fight` (the coupled-side monitor, the calibrator's `SideMonitorConfig`);
     - `MissedTick` (a clock gap past `commandTimeout` while powered);
     - `TooHot`, `StopTooLong`;
     - `FitRefused` (with the side and the `FitVerdict`).
   - **Carried rules:**
     - re-command every tick;
     - 0 V + **Coast** on every refusal and abort, on the same tick; Brake ONLY at a normal segment end
       (D11), until rest;
     - the side's speed is `hal::medianInPlace` over its members × inches per radian;
     - the IMU yaw is accumulated per tick through `Angle::errorTo`, and RECORDED only;
     - ASCII strings with no `%`.
   - **The report** carries:
     - per segment:
       - the end reason, and samples admitted and excluded by reason;
       - travel and **stopping distance** per side;
       - battery min and mean;
       - the largest member **temperature**;
       - the **encoder curve ψ and the IMU's accumulated yaw at the segment's end, and their
         difference**;
       - **the largest tick-to-tick heading step** (the hard StepBackward reproduces the condition of
         2026-09-25's episode, so this is HA-160 evidence for free);
       - the segment's tick timing;
     - per side, the fit result;
     - the combined gains, with kA marked "recorded, not applied";
     - the side asymmetry and its warning.

     It exposes the current sample each tick (`lastSample()`: time, volts, each side's median speed AND
     median position, battery, largest member current, IMU heading), so a program can record raw data
     without the library allocating.
4. **`include/shulib/calibration/rest_capture.hpp`** — a stationary capture, unpowered.
   - **Phases:** `WaitImu → Settle (the calibrator's rest definition) → Capture (duration) → Done |
     Aborted`.
   - **Per tick:**
     - the heading, unwrapped through `errorTo`, and the **per-tick heading step Δh** (the same
       `errorTo`);
     - the yaw rate;
     - each side's median encoder speed in in/s, and each side's median travel;
     - the timestamp.
   - **Results:**
     - **drift**, as the least-squares SLOPE of the unwrapped heading against time (rad/s and °/min).
       Its σ is reported under BOTH models:
       - white heading noise: σ_line·√12/(√N·T);
       - integrated rate noise: σ_Δ·√(1.2/(dt·T)).

       Neither is picked automatically, because each overstates the other case badly;
     - the **structure ratio** ρ = σ_Δ/(√2·σ_line): ≈ 1 for white heading noise, ≪ 1 for a random walk
       (about 0.04 at HA-22's numbers). It is S3's evidence for HA-173's second half;
     - the yaw-rate channel's mean and σ, which is a second drift estimate;
     - **heading noise**: the σ of the residual ABOUT THE LINE, never about the mean, because a drift
       inflates the latter;
     - **σ_Δ/dt**, the σ of the per-tick step about its mean, divided by dt: the rate noise
       `SettledUtil` differentiates (`settled_util.hpp` ~:68);
     - the encoder speed σ per side;
     - the largest tick-to-tick heading step;
     - the tick timing.
   - **Refusals:**
     - `Moved`: either side's ENCODER travel past `moveBound`;
     - `Turned`: the ENCODER turn (dR − dL)/W past `turnBound`. **Never the IMU**, whose apparent
       rotation at rest IS the drift being measured;
     - `ImuNotReady` mid-capture;
     - `RestTimeout`;
     - `Aborted`.
   - **Plus a pure `settleFloorReport(summary, motion::MotionConfig)`:**
     - the MEASURED σ_Δ/dt against `headingSettle.maxErrorRate`;
     - encoder speed σ against `brakeSettle.maxError`;
     - each margin, and a warning under `settleMarginWarn`.

     It never uses `√2·σ_line/dt` (`motion_config.hpp` ~:36-40's derivation, valid for white noise
     only). Under a random walk that formula reads about 0.24 rad/s against the true 0.009 rad/s, and
     warns falsely against 0.30.
5. **`include/shulib/diag/tick_timing_stats.hpp`** — `TickTimingStats`: count, mean, σ, min, max, and
   counts at or over a budget (INCLUSIVE, `LoopMonitor`'s boundary) and over 2× nominal; the first tick
   only baselines, as `LoopMonitor` does. A sibling of `LoopMonitor`, which is not edited.
6. **`include/shulib/sim/drive_plant.hpp` — per-wheel gains, additive:** a per-wheel override of
   `wheelFf` whose DEFAULT leaves every existing suite bit-identical (pinned by a memcmp of a
   default-config run's per-tick truth before and after). Without it no test can tell left from right.
   `test/calibration_test_rig.hpp` gains a kA argument defaulting to 0 (existing callers unchanged).
7. **Every number** in `FitConfig`, `CharacterizerConfig` and `RestCaptureConfig` is labelled
   `PROVISIONAL (A4: HA-170)`. The rest floors the characterizer reuses keep their HA-136 labels.
   Proposed values, all invented:
   - the ramp at 2 V/s to at most 10 V; the step at 6 V;
   - 36 in and 8 s per segment;
   - `asymRefuse` 0.30 (ψ_g ≈ 49.5° at W = 12.5 in, D9); `stopBound` 12 in (D11); `tempGuard` 45 °C;
   - a current guard of 2.3 A per member; `minSpeed` 1 in/s; dt ±10 % of 10 ms; τMin = dt/5;
   - ≥ 100 samples per side; residual RMS ≤ 3 in/s;
   - kS in [0, 4] V; implied free speed `(12 − kS)/kV` in [20, 200] in/s; kA in [0, 0.2] V·s²/in;
   - an asymmetry warning at 15 %;
   - rest: 60 s, `moveBound` 0.25 in, `turnBound` 3°; a settle margin warning at 2×.

   The config's `valid(W)` refuses a config whose curve guard would fire before the asymmetry warning
   could be reached (`asymRefuse` ≤ `asymWarn`).

**Part A DoD**
- [ ] The five headers exist, PROS-free, sim-free, allocation-free, every public member `///`.
- [ ] On the plant (fixed dt, no noise, kA > 0), each side's gains equal the configured ones to 1e-6
  relative. The case uses robot two's shape (5 members a side, reversed members, a decoupled member)
  with DIFFERENT gains per side, about 5 % apart and inside the curve guard, **in both orientations**
  (L faster, then R faster).
- [ ] The rest capture recovers the hostile IMU's drawn bias (`rateBiasRadPerS()`) and its σ within
  bounds derived in the test's comment from N and T. A scripted random-walk heading gives σ_Δ/dt
  equal to the walk's σ within its sampling bound, ρ ≪ 1, and no false settle-floor warning.
- [ ] Every refusal leaves every member at 0 V + Coast on the same tick, asserted at the motors. A
  normal segment end leaves them at 0 V + Brake until rest, then Coast.
- [ ] The default plant is bit-identical (the pin); the whole existing suite green.
- [ ] Every §2.A mutation run and observed red on a fresh binary, logged.
- [ ] HA-170..174 in the register (§3.H); no `src/` file and no in-flight file touched.

### Part B — the card and the builder (host-only; no `src/` change; after G1 Part 2c has merged)

1. **`include/shulib/calibration/characterization_record.hpp`** (D1).
   - **Format:** magic `"SHCH"`, `kVersion = 1`. Fixed width, little-endian, written byte by byte.
     **The layout table in the header IS the document.** f64 fields keep their bit patterns
     (`ByteWriter::f64`).
   - **Names:** `kCharacterizationFileName = "shulib_characterization.bin"` and
     `kCharacterizationBackupFileName = "shulib_characterization.bak"` (D12), both bare (the `/usd/`
     prefix stays the adapters'). A `static_assert` checks that both have no `/` and that they differ,
     as SAFE-SAVE does for the record's pair.
   - **Size — one fixed length, `kCharacterizationBytes`, stated in the layout table and asserted.**
     Every section is always present in the bytes, and the flags say which ones hold data, so any
     other length is a LENGTH refusal. An upper-bound estimate, with every number an f64 unless noted:
     - prefix 8 (magic 4 + version 2 + payload length 2);
     - binding CRC 4 + flags 1 + source 1;
     - gains section ≈ 168: per side 62 (kS, kV, kA, residual RMS, α, voltsMin, voltsMax = 7 × 8;
       samples u32; members u8; kA-below-resolution u8), × 2 = 124; plus the ramp's timing summary 44
       (count, over-budget, over-2× as u32 = 12; mean, σ, min, max = 32);
     - rest section ≈ 144: drift, σ_white, σ_walk, ρ, heading σ, σ_Δ/dt, yaw-rate mean and σ, max
       heading step, duration (10 × 8 = 80), both sides' encoder speed σ (16), N (u32 4), and its
       timing summary 44;
     - provenance 96;
     - CRC 4.

     That is about **426 bytes** (8 + 6 + 168 + 144 + 96 + 4). The layout table fixes the exact
     number. **`static_assert(kCharacterizationBytes <= 512)`.**
   - **Stack cost.** `safeReplace<kCharacterizationBytes>` keeps two local arrays of `MaxBytes + 32`
     bytes, so this file costs at most 2 × 544 = 1088 bytes of stack (about 916 at the estimate). The
     file is well inside SAFE-SAVE's `static_assert(MaxBytes <= 1024)`, and the header states the
     cost. `loadWithBackup` reads into a `kCharacterizationBytes + 32` buffer, so a longer file is a
     LENGTH refusal (SAFE-SAVE's rule).
   - **If a later layout outgrows 1024 bytes,** SAFE-SAVE's `static_assert` fails the build; nothing
     truncates silently. The fix is then a decision, never a workaround: split the sections into two
     files, or raise SAFE-SAVE's cap with the new stack cost stated.
   - **Payload:**
     - the bound drivetrain-record CRC (u32);
     - a section-flags byte (bit 0 gains, bit 1 rest; any other bit refused);
     - the gains section: both sides' fields from A.2, and the ramp's timing;
     - the rest section: A.4's summary, and its timing;
     - provenance (96 bytes).
   - **Refusals, in THIS order, each named:** magic → version → length → CRC → flags → `valid()`. That
     is the drivetrain record's order and reasoning (`calibration_record.hpp` ~:196-205). The decoder
     answers "is this a well-formed record"; it does NOT judge the binding.
   - `boundCrcOf(recordBytes)` returns the CRC stored in a drivetrain record that DECODES (none
     otherwise).
   - `mergeSections(existing, measured, boundCrc)` replaces only the measured section (a REST run must
     not erase a GAINS run). It keeps the other section ONLY when `existing` is bound to `boundCrc`: a
     section bound to another record is in that record's units and is dropped. Pure, for Part C.
2. **`calibration::decideGains(chosenRecordCrc?, siblingBytes, siblingRead, fromBackup)`** — the boot's
   truth table. Every row has a named reason, and it never refuses (D4):
   - no chosen drivetrain record (a table boot) → PLACEHOLDER;
   - no sibling → PLACEHOLDER ("never measured");
   - sibling refused → PLACEHOLDER + the decoder's reason;
   - bound to a different CRC → PLACEHOLDER ("measured against another drivetrain record", both
     CRCs);
   - no gains section → PLACEHOLDER (the rest section still reported);
   - D3's derived budgets not finite and > 0 on the chosen description → PLACEHOLDER, naming kS and the
     budget;
   - else MEASURED, with "(backup copy)" when `fromBackup` (D12).

   `chosenRecordCrc` is the stored CRC of the drivetrain FILE THE BOOT CHOSE. That is
   `shulib_drivetrain.bak` when SAFE-SAVE's `loadRecord` returned `Which::Backup`. `siblingBytes` and
   `fromBackup` come from `loadWithBackup` over the gains pair: `fromBackup` is true when it chose
   `Which::Backup`.
3. **`config::RobotConfig` gains `calibration::DriveCharacterization characterization{}`, APPENDED
   LAST** (after `tuning`, ~:542; designated initializers need declaration order).
   - `valid()` checks it only when `source != Unset`, naming the field.
   - `valid()` **refuses a non-`Unset` characterization on any drivetrain that is not `Tank`**: the
     characterizer measures a tank's two sides, and `effectiveTuning()` passes holonomic tuning through
     (`robot.hpp` ~:1033-1041).
   - G3a's profile never carries it (the build-order G3a entry: the profile shrinks to the human-stated
     facts, and measured numbers come from the card).
4. **`robot::Robot`:** `effectiveTuning()` (~:1033-1041).
   - When the characterization is present: `tuning.motion.wheelFf = characterization.combined()`, and
     the three budgets follow D3's rule, computed by ONE pure function `deriveBudgets(typedMotion,
     measured, trackWidth)` that B.2 also uses.
   - When it is `Unset`, the function's result is EXACTLY today's.
   - **The characterization OWNS `wheelFf` and the three budgets when present.** A typed `wheelFf` is
     honoured only without one. The gains line says "overrides the config's typed wheelFf" when they
     differ. The typed budgets are not ignored: D3's rule starts from them. This resolves two owners by
     one rule, not by a refusal that would stop a typed-tuning robot booting the day a gains file
     appears.
   - New: `gainsSource()` and `describeGains(char*, cap)`, for example:
     - `gains MEASURED kS 0.84 kV 0.1312 (kA 0.021 not applied) L/R kV 3 pct; full stick 11.3 V = 79 in/s`
     - `gains PLACEHOLDER (HA-45) -- never measured: run shulib Calibrate GAINS`

     It adds the member-count note when a side has fewer members present than were measured (HA-174).
     ASCII, no `%`, bounded.
5. **Prose** in `control/feedforward.hpp` (~:9-11, ~:30-33):
   - "characterized OFFLINE by sysid — nothing in this library tunes them" becomes true to Q7: measured
     by the robot, persisted, never adapted while driving;
   - the cruise-overload note (~:76-81) says a measured kA is recorded but not applied.

   `motor_model.hpp` ~:52-53 ("what R5's sysid will measure") is corrected the same way.

**Part B DoD**
- [ ] The record's golden bytes are derived BY HAND from the layout table, with the CRC from an
  independent implementation (`python3 -c "import zlib; ..."`, the command in the PROGRESS log). They
  are never captured from the encoder.
- [ ] `decideGains`: every row tested, including the budget row and the backup flag.
- [ ] `Unset` ⇒ the builder is bit-identical: the existing pins are green, plus an explicit equality of
  `effectiveTuning` against today's.
- [ ] `Measured` ⇒ `motionConfig().wheelFf` equals the mean, and each of the three budgets equals D3's
  formula EXACTLY (from 79/7 V, never 11.29).
- [ ] A plant drive's commanded volts equal the formula exactly.
- [ ] A builder-made chassis and a hand-wired one with the same typed numbers are memcmp-equal.
- [ ] A measured characterization on an X-drive config is refused by `valid()`, naming the field.
- [ ] `describeGains` goldens for measured, placeholder, override, the backup copy and the member-count
  note.

### Part C — shulib Calibrate runs them (build-gated; after B, G1 Part 2c, T3 Part 2, SAFE-SAVE A and B1)

1. **Where offered (D7), and the record reader's CRC (the seam).**
   - The finished screen after SAVED gains **GAINS** and **REST** beside RUN AGAIN.
   - At start, a start screen offers **CALIBRATE / GAINS / REST** when both hold:
     - the card's drivetrain record decodes. It is read through SAFE-SAVE Part A's
       `calibration::loadRecord(store)` → `LoadedRecord`, over a default-constructed `ProsFileStore`,
       with `from != Which::None`. This is the primary, else the backup, exactly as B2's boot will
       choose;
     - `calibration::verifyAgainstRegistry` passes with no port MISSING. On robot two it uses T3
       Part 2's declared-ports form with the claw pivots on 6 and 7, so they are not listed as NEW
       motors (R5).

     Otherwise today's flow runs, unchanged.
   - **The chosen file's STORED CRC (the seam).** SAFE-SAVE's `LoadedRecord` carries `from`,
     `description` and the per-file `RecordFileRead primary, backup` (tried, opened, count, decoded,
     why, path), but no CRC. Part C APPENDS one field to it, `std::uint32_t storedCrc = 0;`: the u32 at
     bytes 176..179 of the file `from` names, meaningful unless `from == Which::None`. It is filled
     inside `loadRecord` from the bytes it already read. This is L7's only edit to SAFE-SAVE's header,
     it is appended last, and B2's boot inherits it for Part D. Calibrate binds a GAINS run to it.
     After SAVED, the CRC is that of the bytes just saved (`boundCrcOf`).
   - The adapters for a GAINS run from the card are made from the record's ports, POSITIVE, with the
     signs from the record (rule 1) and the record's cartridge.
2. **The screens** are new PURE views in `calibration/calibrate_panel.hpp` beside `viewFor()` (~:657).
   They use its budgets, `TouchGate`'s arming and the 5 Hz live-number rule (`calibrate_program.cpp`
   ~:30-40). They show:
   - what to clear, with the numbers computed from the config and the record's W: "N in in front and
     behind, M in to each side, stand clear";
   - the live segment, volts and travel;
   - ABORT, always present;
   - the result;
   - SAVED / NOT SAVED.
3. **Save and load (D12), in a new library header `include/shulib/calibration/characterization_store.hpp`**
   (PROS-free, allocation-free, exception-neutral like SAFE-SAVE's `record_store.hpp`; host-tested):
   - **`loadCharacterization(store)`** → `LoadedCharacterization { Which from; <the decoded
     characterization>; FileRead primary, backup }` (SAFE-SAVE Part A's
     generic per-file result: tried / opened / count / accepted / why / path). It is SAFE-SAVE Part A's
     `loadWithBackup<kCharacterizationBytes>(store, kCharacterizationFileName,
     kCharacterizationBackupFileName, accept)` → `BackupLoad`, with `accept` = the gains decoder succeeds on exactly
     `kCharacterizationBytes`, plus the decoder's reasons. It adds no load rule of its own.
   - **`saveCharacterizationSafely(store, measured, boundCrc)`** → SAFE-SAVE's `SaveReport`. It:
     - loads the existing file (`loadCharacterization`);
     - applies `mergeSections(existing, measured, boundCrc)`;
     - encodes, refusing before touching the card if the result is not `valid()` or does not encode;
     - calls `safeReplace<kCharacterizationBytes>(store, kCharacterizationFileName,
       kCharacterizationBackupFileName, bytes, accept)`.
   - **The panel** is told SAVED **if and only if** `SaveReport`'s outcome is Saved. NotSaved and
     Unconfirmed both show NOT SAVED in red, with "copy these numbers" (SAFE-SAVE's D5/D8 rule).
   - **The serial words are L7's own**, written by
     `formatGainsSaveReport(report, GainsReadAtBoot, emit)`. SAFE-SAVE's `formatSaveReport` is NOT
     used: its sentences name the drivetrain record and what Teleop does without it.
     - `GainsReadAtBoot` is `No` / `Yes`, with no default. Part C passes `No`, because no program reads
       measured gains until Part D. Part D flips it to `Yes` (one token, SAFE-SAVE's
       `BootReadsBackup` pattern).
     - Every sentence comes from the measured report. ASCII only, no `%`, and short enough for
       Calibrate's `emitf`.
     - The formatter is TOTAL: a measured combination with no named branch prints the measurement
       itself (which file the next boot would use, and whether it holds this run's bytes).
     - "unchanged" or "still on the card" is printed only when the measurement says so.
     - The lines, all with the `[R3D]` prefix:
       - **Saved:**
         - `gains file: SAVED -- shulib_characterization.bin holds this run, and the backup holds the
           same bytes`
         - or `gains file: SAVED -- shulib_characterization.bin holds this run; the backup does not
           (<an older gains file | damaged | missing>) and the next save refreshes it`
         - plus, whenever a `replace` reported a failure: `gains file: the card reported <step>
           failing on <file>, yet it reads back whole`
       - **NotSaved**, one of these, by the measurement:
         - `gains file: NOT SAVED -- nothing was written (<no card | nothing measured | would not
           encode>)`
         - `gains file: NOT SAVED -- the gains file on the card is unchanged` + `; the earlier gains
           are still on the card` (only when measured) or `; there was no earlier gains file`
         - `gains file: NOT SAVED -- the earlier gains were written back and read back whole`
         - `gains file: NOT SAVED -- shulib_characterization.bin is damaged; the earlier gains are safe
           in shulib_characterization.bak`
         - `gains file: NOT SAVED -- there was no usable gains file; an older backup copy exists`
         - `gains file: NOT SAVED -- no usable gains file is on the card`
       - **Unconfirmed:**
         - `gains file: SAVE UNCONFIRMED -- <the card is no longer in the brain | nothing on the card
           could be opened or created | the card did not read back the same way twice>; this run
           cannot say which gains file the next boot uses. Put the card back if it was pulled, copy it
           on a laptop, then read the next boot's gains line`
     - **After every outcome, exactly one use line:**
       - with `No`: `gains file: this build's programs do not read measured gains yet; they drive on
         the placeholders (HA-45)`;
       - with `Yes` after a SAVED: `gains file: Teleop and the competition program use these gains
         from their next boot, while the drivetrain record stays the one they were measured against`;
       - with `Yes` after NotSaved: the line names what the next boot uses, from the measured effective
         file — the earlier gains, the backup copy, or the placeholders (HA-45) — and ends `-- the
         robot still drives; a gains problem never refuses (D4)`;
       - with `Yes` after Unconfirmed: `gains file: the next boot uses a gains file or the
         placeholders -- the robot still drives; a gains problem never refuses (D4)`.
   - **The copy-these-numbers rule:** every NOT SAVED or UNCONFIRMED run has already printed the
     gains lines above it (5.), so the numbers are on the serial log whatever the card did.
4. **The raw samples (D6):**
   - Recorded from `lastSample()` into a fixed buffer inside HA-59's 64 KiB. A full buffer stops
     recording and says so in the file's header line; the fit does not depend on it.
   - Written to `shulib_ramp.csv` ONLY after every motor is stopped (HA-60). The columns: time,
     segment, side, volts, speed, **position**, battery, the largest member current, the IMU heading.
   - The CSV line format is a pure host-tested formatter.
   - `tools/sysid_refit.py`:
     - **stdlib only**;
     - re-fits the CSV with its own implementation and prints the same fields as the brain's serial
       line;
     - compares reported speed against differenced position (HA-171);
     - has a `self-test` built in (the pattern of `tools/src_build_gate.py self-test`);
     - is wired into `.github/workflows/ci.yml` as one step after 2c's final version of that file.
5. **The serial record** (`[R3D]`, as today):
   - the config;
   - one line per segment: end reason, samples, exclusions by reason, travel, **stopping distance**,
     encoder curve vs IMU yaw, max heading step, max temperature, timing;
   - one line per side: gains, residual, voltage range, members;
   - combined gains (kA marked not applied), asymmetry, D3's derived budgets;
   - REST's line in a fixed, greppable form (ten boots = ten lines for HA-20 and HA-173), including
     both drift σ's and ρ;
   - the settle-floor margins;
   - SAVED / NOT SAVED.

**Part C DoD**
- [ ] Both entry points work per D7; ABORT, a field disable and every refusal leave 0 V + Coast.
- [ ] Panel views host-tested; every literal checked against its budget at compile time.
- [ ] The save goes through `safeReplace<kCharacterizationBytes>`, and SAFE-SAVE's crash-point
  discipline holds for the gains pair. Its starting cards and crash enumeration are re-run with this
  file's names, length and `accept`, including u1–u3.
- [ ] `formatGainsSaveReport`: a golden for every branch in C.3 under both `GainsReadAtBoot` values,
  7-bit ASCII, no `%`, and total over every measured `SaveReport` combination the crash test produces.
- [ ] `LoadedRecord::storedCrc` equals the chosen file's bytes 176..179 for P, for K, and 0 for None.
- [ ] `src_build_gate.py self-test && check` green on the gate 2c left; ARM gate green;
  `tools/sysid_refit.py self-test` green locally AND as a CI step.
- [ ] The worksheet station written FROM the code; every document says it has not run on a robot.

### Part D — every program says which gains it used (build-gated; after C, COMP-b, SAFE-SAVE B2)

- **The boot path** (B2's, which reads the record through `loadRecord`):
  - reads the gains file through `loadCharacterization` (SAFE-SAVE's `loadWithBackup`, C.3);
  - calls `decideGains` with `LoadedRecord::storedCrc` (the drivetrain file the boot CHOSE, primary or
    backup), the gains bytes, and `fromBackup` (`from == Which::Backup` for the gains pair);
  - fills `config.characterization`.
- Calibrate's `formatGainsSaveReport` call flips to `GainsReadAtBoot::Yes` in this change.
- **What each program prints:**
  - "shulib Teleop" and the competition program print `describeGains()` on the screen and the serial
    line at boot, and the reason when PLACEHOLDER;
  - "shulib Drive" and Bench Tests print `gains: none -- open-loop sticks, no feedforward`;
  - the invented X-drive (`src/xdrive_program.cpp`, 2c) prints `gains PLACEHOLDER (HA-45) --
    holonomic: the GAINS step measures a tank only`;
  - Calibrate says what it measured.
- `test/boot_source_test.cpp` gains the gains rows.
- Re-check the brief against the tree first: which programs exist after 2c and COMP-b is their answer.

**Part D DoD**
- [ ] Every program in the build prints one gains line (the X-drive included); the placeholder line
  names its reason.
- [ ] A boot with an ABSENT member prints the member-count note.
- [ ] A boot from SAFE-SAVE's backup record binds against the backup's CRC.
- [ ] The build gate and the host suite green; docs per §3.D.

## 2. Tests (adversarial) and mutations

**Mutation recipe, every part:** edit → `touch` every test that includes the header → build
`shulib_tests` and REQUIRE `Linking CXX executable shulib_tests` in the output → run → observe red →
restore → relink → green (the stale-binary trap, HANDOFF §3). A mutation not run does not count.

### 2.A
- **Fit on synthetic data first:**
  - exact data from the discrete law (kS 0.9, kV 0.13, kA 0.02) → 1e-9;
  - kA = 0 → kA exactly 0, `kABelowResolution`, no NaN;
  - data generated with α = +1e-12 AND with α = −1e-12 → kA exactly 0 and the flag, both times (R1);
  - one voltage level only → `NotExcited`;
  - **ramp-shaped data only at kA = 0 → `NotExcited`** (R2);
  - a flipped reading sign → `SignMismatch`;
  - kS < 0 or α ≥ 1 → `NonPhysical`;
  - each HA-170 range at its edge: inside passes, one ulp outside refuses.
- **Model mismatch (the part that makes the fit honest):**
  - a quadratic drag term, a current-limited acceleration and a free-wheeling coast-down, each
    generated OFF the model. The first two must reach `PoorFit` or be excluded by the current guard.
    The third must leave the fit unchanged (u = 0 excluded).
  - **a first-order-filtered `velocity()`** (a time constant of 1–3 ticks) → pin the kA/kV bias it
    produces, or require `PoorFit`;
  - a one-tick command latency → pin and record the bias it produces;
  - **a drag sweep:** the largest quadratic drag `PoorFit` does NOT catch, and the kS/kV bias it leaves,
    pinned and written into HA-171. The documents must not say PoorFit catches mismatch in general.
- **Numerics:** 10⁵ samples with a residual ~1e-3 in/s at speeds ~80 in/s → the residual RMS equals a
  two-pass reference to 1e-9 relative (a normal-equations implementation fails this).
- **Characterizer on the plant.** kA > 0, per-wheel gains about 5 % apart (for example
  L {1.0, 0.160, 0.020}, R {1.1, 0.168, 0.024}, which curve 13–17° per segment on the plant). **EVERY
  case runs in both orientations**, L faster and R faster.
  - Recovery:
    - each side recovers its own gains;
    - reversed members with signs from the description give the same gains;
    - a decoupled member leaves the gains unchanged (the median);
    - an inches-per-radian 10 % high moves kV by exactly 1/1.1.
  - Travel, curve and stop:
    - the POWERED travel of each side ≤ segmentDistance + that tick's travel, on plant TRUTH;
    - each side's stopping distance equals plant truth;
    - a large asymmetry (≥ 1.5 × `asymRefuse`) → `CurveExceeded` with 0 V + Coast that tick, and a
      message naming both sides' travel;
    - a scripted IMU jump of 37° in one StepBackward tick, alternating in sign for 8 ticks (HA-160's
      shape) → NO refusal, with the report's max heading step and IMU-vs-encoder difference showing it;
    - a normal end → every member at 0 V + Brake until rest, then Coast;
    - a plant kA large enough that the τ decay exceeds `stopBound` → `StopTooLong` before the next
      segment.
  - Other refusals:
    - a scripted member temperature ≥ `tempGuard` → `TooHot` naming the port, and nothing powered;
    - a missed tick while powered → `MissedTick` and 0 V, while at rest → nothing;
    - ABORT in every phase;
    - a SignFlipped member → `Fight` naming the port;
    - a robot that never rests → `RestTimeout`;
    - a scripted member current above the guard → those samples excluded and counted.
  - Noise and end to end:
    - encoder hostility → a bound is measured, pinned no looser than 2× the measured error, and both
      are recorded;
    - end to end: the calibrator's description feeds the characterizer on the SAME plant, with gains
      within the pinned bound.
- **Rest capture:**
  - The hostile IMU, with a large drawn bias (e.g. `rateBiasMax` 10°/min) and a fixed seed:
    - drift within 4σ of the slope estimator (σ_b = σ·√12/(√N·T), derived in the comment);
    - heading σ within its sampling bound;
    - ρ ≈ 1 within a derived bound;
    - NO `Turned`, even though the IMU "turns" up to 10° over the capture.
  - A test-local scripted RANDOM-WALK heading (seeded; a test fixture, not a plant change):
    - σ_Δ/dt equal to the walk's σ within its bound;
    - ρ ≪ 1;
    - `settleFloorReport` does NOT warn against 0.30 rad/s at HA-22's 0.5°/s.
  - A capture straddling ±π (start at 179.9°) → the same drift.
  - A scripted push through the plant → `Moved`; a scripted turn through the plant's wheels →
    `Turned`; ready dropping mid-capture → `ImuNotReady`.
- **Timing:** a `JitterSchedule` with a fixed seed → mean and σ equal a two-pass reference to 1e-12;
  min, max and the counts EXACTLY; a tick of exactly 15 ms counts as over budget (inclusive).
- **Mutations (each red on a fresh binary):**
  - Fit:
    - drop the γ·sgn(v) column;
    - kA without the logarithm;
    - compare α against 0 instead of `alphaMin`;
    - admit dead-band samples;
    - admit u = 0 samples (red ON THE PLANT too: kS 1.00 → 0.53);
    - admit off-nominal dt;
    - remove the β sign check;
    - remove the PoorFit check;
    - swap L/R accumulation.
  - Characterizer:
    - apply the sign twice;
    - mean instead of median;
    - distance bound on the mean of the sides;
    - **distance bound checked on the LEFT side only** (red only in the R-faster orientation);
    - **curve guard computed from the IMU**;
    - **ψ_g computed with a fixed W instead of the description's**;
    - no missed-tick guard;
    - a refusal that leaves the last volts;
    - Brake instead of Coast on refusal;
    - **Coast instead of Brake at a normal end**;
    - **no stop bound**;
    - **temperature not checked**;
    - start a segment before rest;
    - current guard off;
    - forget the inches per radian.
  - Rest capture:
    - heading end-minus-start;
    - regression on the WRAPPED heading;
    - σ about the mean instead of the line;
    - **`Turned` read from the IMU** (red on the large-bias case);
    - **`settleFloorReport` using √2·σ_line/dt** (red on the random-walk case).
  - Timing and plant:
    - the baseline tick counted as dt 0;
    - over-budget with `>`;
    - the plant override applied to the wrong wheel, or the default path changed.

### 2.B
- **Record:**
  - golden bytes by hand + an independent CRC; a round trip;
  - every refusal, in order;
  - NaN and ±Inf bit patterns survive the trip and are refused by `valid()`;
  - an unknown flag bit refused;
  - a drivetrain record handed to this decoder is refused at the magic, and this record handed to the
    drivetrain decoder likewise;
  - `mergeSections` keeps the section it did not measure ONLY when that section is bound to the same
    record.
- **Boot and builder:**
  - `decideGains`, every row;
  - `RobotConfig::valid()` names a non-finite gain, and refuses a measured characterization on an
    X-drive;
  - the builder cases in the DoD, including each budget to the exact formula;
  - `describeGains` goldens, 7-bit ASCII, no `%`, within its bound.
- **Mutations:**
  - Record:
    - CRC over the wrong span;
    - the binding judged before the CRC;
    - the binding skipped;
    - an unknown flag accepted;
    - an f64 through `float`;
    - **the merge keeps a section from a sibling bound to ANOTHER record**;
    - the member count not persisted.
  - Builder and boot:
    - combined = left only;
    - `effectiveTuning` ignores the characterization;
    - the `Unset` path writes `wheelFf`;
    - D3's budget with kS added;
    - D3's budget from 11.29 instead of the typed volts;
    - **`maxAngularSpeed` not derived**;
    - **`maxAngularSpeed` derived with the wrong track width**;
    - `describeGains` says MEASURED for a placeholder;
    - `valid()` skips the characterization;
    - **`valid()` accepts it on a holonomic drivetrain**;
    - a table boot with a sibling present returns MEASURED;
    - the non-positive-budget row returns MEASURED.

### 2.C / 2.D
- **C:**
  - the panel views: GAINS/REST shown only after SAVED or a verified record; ABORT always present;
    arming after every layout change; the clear-space numbers computed from the config and W;
  - `mergeSections` through the program's path;
  - the save's crash points: SAFE-SAVE's starting cards and crash enumeration over
    `FakeFileStore`, re-run for the gains pair, with u1–u3;
  - `formatGainsSaveReport`: the goldens of C.3, both `GainsReadAtBoot` values, total over the crash
    test's reports;
  - `loadCharacterization` reads the backup only when the primary does not decode;
  - `storedCrc` for P, K and None;
  - the CSV formatter (ASCII, fixed columns, position present);
  - `sysid_refit.py self-test` goes red on a swapped column, on a dropped kS term and on a dropped
    position column.
  - **Mutations:**
    - GAINS offered with a MISSING port;
    - the merge drops the other section;
    - the formatter emits `%`;
    - the gains save bypasses `safeReplace` (a direct `replace`);
    - the binding uses a CRC other than the chosen file's;
    - **the panel says SAVED on an Unconfirmed report**;
    - **"unchanged" printed without the measurement**;
    - **the gains lines printed through SAFE-SAVE's `formatSaveReport`** (a drivetrain sentence
      appears, and the goldens go red);
    - **`storedCrc` taken from the primary when `from == Which::Backup`**.
- **D:**
  - `boot_source_test` rows for each `decideGains` outcome through the real boot decision, including
    a boot from the drivetrain BACKUP (gains bound to the newer record ⇒ PLACEHOLDER by name) and a
    gains file read from its own backup ("(backup copy)").
  - **Mutations:**
    - the sibling read under the wrong file name;
    - the gains line omitted;
    - the member-count note omitted;
    - the X-drive prints MEASURED;
    - `GainsReadAtBoot` left at `No` after Part D.

## 3. Documentation (same change as the code, or the change waits)

**3.0 — in the BRIEF's commit (the coordinator's):**
- a build-order entry for R5a under Phase R, with its parts and Q7;
- R4's, R5's and E5's entries each gain one status line naming what R5a takes from them;
- the **deviations-table row**, in build-order's four columns (`Change | Roadmap says | Build order
  says | Why`, ~:1935):

  | Change | Roadmap says | Build order says | Why |
  |---|---|---|---|
  | **+ R5a: gains and noise measured by the robot and kept on the SD card** *(2026-09-26, the team lead's Q7)* | R5: `tools/sysid` offline, "kS/kV/kA committed as constants" | the fit runs on the brain inside shulib Calibrate; the result persists beside the drivetrain record, bound to it, and the builder consumes it; a laptop re-fit (D6) cross-checks | the team lead's 2026-09-18 framing and Q7: nothing a robot can measure is typed or committed; gains depend on the drivetrain record's scale and must be re-measured with it, which a committed constant cannot follow |

- the handoff's L7 line points at the brief;
- **the handoff's Q7 line (~:492) is marked RULED 2026-09-26**;
- the status board's Q7 is closed.

**3.A** (Part A):
- `docs/hardware-assumptions.md`:
  - HA-170..174 added: index rows, entries, and the status paragraph;
  - settle lines amended, statuses unchanged:
    - HA-45: the settle path is now shulib Calibrate's GAINS step, persisted (Q7); kA is recorded,
      not applied;
    - HA-48: its extrapolation twin is HA-171;
    - HA-49: the step segments' current column;
    - HA-20/21/22: the REST step, one boot per run, ≥ 10 boots; HA-21's "σ" presumes white heading
      noise, see HA-173;
    - **HA-51**: the REST step reports the measured per-tick rate noise and encoder noise against
      these floors, with the margin;
    - **HA-53**: the GAINS step's per-segment stopping distance under Brake;
    - HA-32/34: partial, because Calibrate's loop is lighter than a motion loop;
    - HA-160: the ramp's step segments record the largest tick-to-tick step AND the IMU-vs-encoder
      difference.
  - **HA-52 is NOT amended.** Nothing L7 measures settles a stall threshold, and on these robots the
    check never reports a verdict (`independentMotionSource` is false for drive-encoder odometry,
    `robot.hpp` ~:1036-1038).
- `docs/roadmap.md`: the sysid row ~:1142 → `[~]` with the reframe; ~:1124's "bare gains from sysid"
  → "measured by the robot (Calibrate's GAINS step); kA recorded, not applied"; the R4/R5 lines.
- `docs/changelog.md` ("Added since 2.2"); build-order status; `docs/api` regenerated; the PROGRESS
  log.
- Every one of these files is also edited by 2c or T3 Part 2. The coordinator merges them.

**3.B** (Part B):
- HA-50 (D3's derived budgets, if ruled);
- `docs/guide/05-getting-there.md` ~:60-63 and ~:79-83: the feedforward constants are measured by the
  robot when a gains file exists; kA is recorded, not applied; say "host test plant" where it now says
  "simulated robot" / "simulator", if those lines are edited;
- **`docs/guide/08-your-first-routine.md` ~:198 and ~:272**: a typed `wheelFf` is overridden by a
  measured gains file at a card boot (after Part D);
- `docs/guide/14-what-it-cannot-do-yet.md` ~:56: the constants are measurable by the robot; none has
  been measured;
- changelog; roadmap/build-order; `docs/api`.

**3.C** (Part C):
- the worksheet: a NEW station at the next free letter at commit time (**L** if free; K is COMP-b's),
  written from the code. It covers:
  - the lane and its clear space (the screen's numbers);
  - the two entry points;
  - motors cool;
  - GAINS twice, and REST across boots;
  - copying the CSV, and running the re-fit;
  - the gains pair on the card (`.bin` and `.bak`), what each gains SAVED / NOT SAVED / SAVE
    UNCONFIRMED line means, and "to reset the gains, delete BOTH gains files on a laptop" (as
    SAFE-SAVE documents for the record).
- Station G's end points at the new station.
- **HA-166** (SAFE-SAVE's ONE entry, five beliefs; not renumbered or split here): one sentence under its
  *Source* and *Contained* lines. The gains pair (`shulib_characterization.bin` / `.bak`, saved by
  `safeReplace`) rests on (4) and (5) as the record's pair does. If either is wrong, a torn gains pair
  boots on the placeholders by name (D4), and never refuses to drive. Its status and claim are
  unchanged.
- `docs/faq.md`: SAFE-SAVE's answer about renaming or deleting a file names the gains pair beside the
  record's.
- `docs/guide/07-getting-set-up.md`: the programs' list gains Calibrate's new steps.
- `docs/guide/12-when-things-go-wrong.md`: each new refusal and its fix — `CurveExceeded`,
  `StopTooLong`, `TooHot` and the fit verdicts.
- the HA settle lines point at the station; roadmap/build-order.
- **Prose review** of `docs/guide/03-*.md` ~:67-69 ("on the order of a degree per minute"): unchanged
  until S3 measures, but the review is logged.

**3.D** (Part D):
- `docs/guide/11-reading-the-diagnostics.md`: the gains line and every PLACEHOLDER reason, including
  "holonomic" and "(backup copy)";
- worksheet Stations F, G and K (what the boot prints);
- ch.14; README, if it states the gains are placeholders; roadmap/build-order;
- `R5a-COMPLETED.md`, only when the DoD of every part is met. The measurement (S2/S3) is NOT part of
  this chunk's DoD, and R5 is not closed by it.

**Every part:**
- a logged prose review: grep `docs/` and `README.md` for each changed file's name and concepts, and
  read the hits BEFORE writing that it was done;
- public docs cite evidence by date, never a `docs/internal` path;
- the briefing regenerated LAST.

**3.H — the register entries (reserved HA-170..HA-174 ONLY; the coordinator renumbers at merge if
needed):**
- **HA-170 — every number in `FitConfig`, `CharacterizerConfig` and `RestCaptureConfig` is invented for
  a first run.**
  - It covers §1.A item 7's list: the ramp, the step, the segment, `asymRefuse`, `stopBound`,
    `tempGuard`, τMin, the guards and ranges, and the rest capture's 60 s and bounds.
  - It also covers D3's full-stick budget: a CHOICE, 79/7 V exactly, derived from HA-45 × HA-50.
  - And the raw-sample buffer's size, inside HA-59.
  - *Settle:* S2/S3 serial logs; each number moves with a logged line.
  - *Blast radius:* a wrong bound refuses a good run or admits a bad one, and both are loud. A
    `stopBound` or ψ_g sized wrong also changes the clear space the screen asks for.
- **HA-171 — a V5 drive side follows V = kS·sgn(v) + kV·v + kA·a closely enough that the fitted gains
  hold outside the fitted range.**
  - The gains are fitted over the ramp's range (the ~6–10 V a 36 in segment reaches). They must hold
    at the full-stick voltage and while turning.
  - The fit's two measurement channels are assumed faithful. `IMotor::velocity()` (PROS
    `get_actual_velocity`, HA-96) is assumed current within a tick, with no filter or lag of its own
    beyond it. A motor command is assumed to take effect within one tick (unregistered until now).
  - `PoorFit` is NOT a general mismatch detector. It misses a quadratic drag up to the pinned size,
    with the kS/kV bias the test records.
  - Invented; HA-48 is the plant-side twin.
  - *Settle:* S2's residual per voltage band; the laptop re-fit's reported-speed vs differenced-position
    comparison; one full-stick run whose speed is predicted from the gains.
  - *Blast radius:* Teleop's top speed and every motion's feedforward residual are off; the PID absorbs
    it, and settles slow.
- **HA-172 — inside the ramp's range, the only non-linearity is the per-motor current limit.**
  - The limit is the PROS default of 2500 mA, the vendored `motors.h` figure (~:1257) that HA-154 also
    cites.
  - **Whether VEXos lowers the per-motor limit when more than eight motors are plugged in is not
    stated in the vendored text and is UNVERIFIED.** Robot two carries 12: 10 drive and 2 claws.
  - `IMotor::current()` is assumed to report the limit within a tick, so a sample with any member at or
    above 2.3 A can be excluded.
  - Temperature is below HA-44's first throttle step, which `tempGuard` enforces.
  - Reasoned (the limit); invented (the timing, the count question, and that the firmware adds no slew
    of its own to `move_voltage`).
  - *Settle:* the step segments' current column and plateau in S2's CSV; one reading of
    `get_current_limit` on robot two.
  - *Blast radius:* current-limited samples that slip past the guard bias kS and kV, which ARE applied
    (Teleop's top speed, every motion's feedforward). A torque-limited step also inflates kA, which is
    recorded only today: it reaches the report, the CSV and R6's plant back-fit (a too-sluggish plant).
    It would reach motors only through a future acceleration channel. HA-49 is the plant-side twin.
- **HA-173 — a stationary V5 IMU's drift is a PER-BOOT bias (HA-20), and one 60 s capture may not
  separate it from noise.**
  - Per-boot bias: one rest capture measures that boot, never a property of the robot. A stored drift
    is a sample for this register, and never a correction applied at a later boot.
  - Whether one capture separates that bias from noise depends on the noise's STRUCTURE, which the host
    cannot show:
    - with WHITE heading noise (the host's hostile IMU; HA-21's 0.05°), the slope's σ = σ·√12/(√N·T)
      ≈ 0.002°/min at 60 s and 100 Hz;
    - with INTEGRATED rate noise (a gyro's heading is its integrated rate; HA-22's 0.5°/s per 10 ms
      sample), the slope's σ ≈ 0.42°/min at 60 s and 0.19°/min at 300 s. That is HA-20's typical
      drift size, so one boot cannot separate them, and only the spread across boots can. Under that
      structure `√2·σ/dt` also overstates the rate noise `SettledUtil` sees (~0.24 against
      ~0.009 rad/s).
  - Reasoned (the per-boot bias); invented (the structure).
  - *Settle:* S3, with ≥ 10 boots' REST lines: ρ says which structure holds; the spread between boots
    is compared against the within-run slope σ of that structure.
  - *Blast radius:* if the bias were stable per robot, an easy correction is left unused — a missed
    gain, not an error. If the structure is a walk, HA-51's heading-rate floor was sized on the wrong
    formula, and the REST margin line shows by how much.
- **HA-174 — gains measured once hold for the match.**
  - On the same foam tiles (not carpet or a bench top), on a charged pack (HA-47), and with the same
    drive members.
  - A member ABSENT at a later boot leaves kV roughly unchanged (motors on one train share the
    back-EMF), but raises kS roughly as 1/(members present). kS IS applied. kA rises in the same way
    but is recorded only.
  - Invented (surface); reasoned (members).
  - *Settle:* S2 on tiles and again on another surface; one run with a member unplugged.
  - *Blast radius:* low-speed creep, or a settle that stalls short. The member-count note makes the
    second half visible.

## 4. OUT (and who owns it)

- R4's other items — GPS latency and noise (HA-26..31), slip under acceleration (HA-37..39), IMU
  latency by step rig (HA-24), pitch/roll tip detection, boot-time bias estimation → **R4**.
- R5's re-tune of the PID gains (HA-50's kP) and "motion re-verified on hardware" → **R5**, after S2
  and S8.
- E5's GPS lever arm, camera mount and wheel offsets → **E5** (with L3 and a GPS).
- The plant back-fit → **R6 / P1**.
- **An acceleration channel in the command pipeline** (applying kA) — a pipeline change, pinned by the
  C2 suites; not this chunk (D2, D10).
- A per-wheel feedforward in the pipeline (D2).
- Applying noise numbers to settle floors, the EKF or heading (D5).
- An IMU-based curve refusal (D9).
- Any change to the drivetrain record, its version or its golden (D1).
- SAFE-SAVE's own work: its backup of the drivetrain record, its store seam, `safeReplace` /
  `loadWithBackup` / `loadRecord` (L7 only appends `LoadedRecord::storedCrc`, C.1),
  `formatSaveReport` and the drivetrain record's words, Teleop's BACKUP words, and HA-166's claim.
- DebugRecord, SHUL/2 and session-header fields.
- The calibration guide chapter (handoff §6 item 6) — its own documentation step.
- Robot one (Q3).
- G3a's panel reading the sibling (G3a's; the pure decoder already compiles on a host).
- A Teleop-only budget scaling (it would touch `include/shulib/teleop/*`).
- The fault policy.
- Every file in flight (above).
- The measurement itself (S2/S3), and any hardware claim.

## 5. Checks (every part; C and D add the src build gate)

- The host suite green (counts recorded); every mutation red on a fresh binary.
- `python3 tools/src_build_gate.py self-test && python3 tools/src_build_gate.py check`:
  - C and D run it on whatever gate 2c left;
  - A and B change no `src/` file, but run it once to show nothing moved.
- The ARM header gate (RESUMING's command); the two CI greps.
- `api_doc_tool` self-test / coverage / check-fresh / check-examples / check-removability;
  `doc_staleness_audit.py`.
- `tools/sysid_refit.py self-test` (C), locally and as the new CI step. The tool imports nothing
  outside the standard library.
- The briefing regenerated LAST and checked.
- Confirm nothing was committed and no in-flight file changed (`git status --short`, `git diff --stat`).

## 6. Landmines

1. **The stale-binary trap** — require the relink line on every mutation.
2. **The plant's model IS the fitted model.** Recovering it proves arithmetic, not physics; only the
   model-mismatch cases make Part A adversarial. Do not drop them because the plant "passes anyway".
3. **The plant's default kA is 0** (`drive_plant.hpp` ~:142; the rig's `calibPlant`, ~:57-62). Every kA
   case must set kA > 0, and the kA = 0 path must give EXACTLY 0 — not NaN, and not 5·10⁻⁵ from an
   α of +10⁻¹³ (R1).
4. **One `wheelFf` for every wheel** hides a left/right swap until A.6 lands; write A.6 and its
   bit-identity pin first.
5. **Stop samples are excluded for TWO reasons. Keep the exclusion even if either reason goes away.**
   - The hardware reason (HA-143): 0 V under Coast may free-wheel, which the law does not describe,
     and a Brake stop is not the law either.
   - The plant reason: the plant's friction follows the STEADY-STATE sign (`motor_model.hpp` ~:41-44,
     ~:100-106). At u = 0, and at any 0 < |u| ≤ kS, a moving wheel decays with NO friction term, so
     those samples violate the fitted model on the plant too. Admitting u = 0 moves kS from 1.00 to
     0.53 (the re-check's probe).

   The segments never produce a 0 < |u| ≤ kS sample with |v| ≥ `minSpeed`, because each starts from
   rest. Do not add a powered deceleration without re-reading this. Do not "simplify" the exclusion.
6. **One place for a sign.** Calibrate's adapters are POSITIVE and the characterizer applies the sign;
   Teleop's are SIGNED. A double application is a fight on the robot and invisible in a symmetric test.
7. **Gains are in in/s of the DESCRIPTION's scale.** A re-calibrated drivetrain changes every gain's
   units; the binding CRC is load-bearing, not bookkeeping. It is the CRC of the file the boot CHOSE.
8. **Measured gains change what full stick means** (D3) — whichever way D3 goes, the banner must say
   what full stick now costs. The constant is 79/7 V; never type 11.29.
9. **Append, never insert,** in `RobotConfig` and in every persisted enum.
10. **Never persist and apply a drift** (HA-173).
11. **The StepBackward segment reproduces 2026-09-25's episode** (a hard reverse). The IMU may jump by
    tens of degrees per tick there (HA-160). The curve guard reads the ENCODERS for exactly that reason
    (D9). Record the IMU; never let it refuse; never loosen the encoder guard to make a run pass.
12. **SD writes only at rest** (HA-60) — never inside a powered segment.
13. **Screen text:** ASCII, no `%`, the 5 Hz live-number rule (PROS 4.2.2 leaks per print,
    `calibrate_program.cpp` ~:30-40).
14. **HA numbers are 170–174 only;** amendments to older entries change settle lines, never statuses.
15. **No `docs/internal` path in a public document** (C7 removability); "host test plant", never
    "simulator".
16. **Do not edit a file in flight** — the list above, rebuilt from the diffs; read-only access is fine.
17. **kA is not applied.** No document, banner or register line may say a measured kA changes how a
    motor is driven. The pipeline calls the cruise overload (`command_pipeline.hpp` ~:149).
18. **Brake at a normal segment end ONLY, until rest; Coast everywhere else** (D11). A Brake on a
    refusal leaves a robot a person cannot push.
19. **The settle-floor comparison uses the MEASURED per-tick step**, never `√2·σ/dt`. The latter is
    right only for the host's white-noise IMU (HA-173).
20. **The brain has no rename, no delete and no read-write open; `fopen "wb"` empties the file at the
    open** (HA-166 (1)–(3); SAFE-SAVE §0.2). A host test that passes with `std::rename` proves nothing.
    The gains file goes through `safeReplace<kCharacterizationBytes>` and is read through
    `loadWithBackup` (D12). L7 writes no second save or load rule.
21. **The gains save's words are L7's own** (`formatGainsSaveReport`, C.3). Never route them through
    SAFE-SAVE's `formatSaveReport`, which names the drivetrain record and says Teleop will refuse; a
    gains problem never refuses (D4).
22. **`kCharacterizationBytes` ≤ 512 is asserted.** Any field added to the layout is checked against
    it, and against SAFE-SAVE's `MaxBytes <= 1024` cap. A layout that outgrows the cap is a decision,
    never a silent change to the cap.
