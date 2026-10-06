# R5a — progress log (live)

- **Chunk:** R5a (handoff L7) — gains and noise, measured by the robot inside shulib Calibrate and kept on
  the SD card (brief: [`R5a-gains-and-noise-on-the-sd-card.md`](R5a-gains-and-noise-on-the-sd-card.md))
- **Executor model:** opus
- Appended as the work goes, with a heading per part. Every time in a section header is a `date`
  reading taken when the section was written, not an estimate written afterwards. **Nothing in this
  chunk has run on a robot.** Every number here is from the host test plant or from synthetic data.

---

## Part A — the measurements, as library

- **Started:** Tue Sep 29 03:39:52 AM EDT 2026 (`date`).
- **Tree at start:** `9239538`, clean, in the git worktree `.claude/worktrees/r5a-a` (branch `r5a-a`) —
  NOT the main working tree.
- **Go:** the coordinator's workflow run of 2026-09-29, on the team lead's go for R5a Part A (his answers
  at the top of the brief: every recommendation D1–D12 accepted; Q7 ruled: the SD card). Do not commit;
  no `pros` command; no `/dev/ttyACM*`. Mutation backups and probes under
  `.claude/coordinator/scratch/r5a-a/` (never `/tmp`, never the repository root).
- **Scope:** Part A ONLY — `calibration/feedforward_fit.hpp`, `calibration/drive_characterization.hpp`,
  `calibration/drive_characterizer.hpp`, `calibration/rest_capture.hpp`, `diag/tick_timing_stats.hpp`
  (all new), `sim/drive_plant.hpp` (per-wheel gains, additive), `test/calibration_test_rig.hpp` (a kA
  argument defaulting to 0), the tests of §2.A and their mutations, the documents of §3.A, HA-170..174.
  NOT Part B (the card and the builder), C (Calibrate runs them) or D. No `src/` file.

### 03:39 — start
- Read `docs/internal/RESUMING.md` (the chunk loop, the standards, pace rules 1–8) and the brief in full
  (the answers at the top, the Re-check amendments 1–15 and R1–R5, the SAFE-SAVE reconciliation, D1–D12,
  §0, the Parts table, the in-flight list, §1 Part A and its DoD, §2.A, §3.A, §3.H, §4, §5, §6). This log
  created before any code.

### 03:40–03:55 — the brief's citations re-checked against the tree (`9239538`), each delta and how it is taken
The brief was written at `a01e3a2`; SRC1, T4 Part 1a, TANK1 Part A and the plan docs of 2026-09-28 landed
since. Every file:line it cites, read in the tree:
- **Held (read, unchanged in substance):** `drive_plant.hpp` :142 (the one `wheelFf`); `motor_model.hpp`
  :7-17, :21-31, :41-44, :52-53, :80-96, :100-106; `calibration_test_rig.hpp` :57-62 (`calibPlant`);
  `drivetrain_calibrator.hpp` :78-108 (the seven rules), :148 (includes `coupled_side_monitor.hpp`),
  :262-280 (rest floors), :344-351 (`commandTimeout`, `monitor`); `motion_config.hpp` :32-34 (the rail
  invariant), :36-40 (the √2·σ/dt derivation); `feedforward.hpp` :9-11, :30-33, :47-50, :76-81;
  `command_pipeline.hpp` :142-152 (:149 is now :148-150 — the cruise overload `ff.calculate(wheels[i])`);
  `imu_hostility.hpp` :67-80, :108-114; `composed.hpp` :203-231 (JitterSchedule); `loop_monitor.hpp`
  :12-20, :42-46; `motor_group.hpp` :129 (`medianInPlace`); `fake_motor.hpp` :39-40 (`setCurrent`,
  `setTemperature`); `robot.hpp` :838-844, :1033-1041, :1036-1038, :1144; `robot_config.hpp` ~:18, ~:34
  (Part B's); `teleop_loop.hpp` :208; `calibrate_panel.hpp` :657 (`viewFor`, Part C's); `ci.yml` :34.
- **Moved, adapted:**
  - `motion_config.hpp` ~:75-78 (`wheelFf`) is now **:150-153** — TANK1 added `TankSteerConfig` above
    `MotionConfig`. Part A reads the settle floors (:175-183) by NAME, so nothing depends on the line.
  - `imu_hostility.hpp` ~:148-150 (`rateBiasRadPerS()`) is now **:150-152**. Read by name.
  - `settled_util.hpp` ~:68 is **`control/settled_util.hpp`:68** (the brief gave no directory). Held.
  - `robot_config.hpp` ~:527-542 (`tuning`) is now **:543**; `teleop_loop.hpp` ~:247-249 is :249. Part B's.
  - **SRC1:** `boot_source.hpp` is `src/common/boot_source.hpp` (~:84-91 → the `tuning` note at :235-237,
    :312; `RecordRead` ~:230-236 → :374); `calibrate_program.cpp` is `src/programs/calibrate_program.cpp`
    (`saveAndReadBack` ~:627-684 → :630). Parts B–D's; Part A touches no `src/` file.
  - `docs/roadmap.md` ~:1142 (the `tools/sysid` row) is now **:1162**; ~:1124 ("bare gains from sysid")
    is **:1144**. Edited at those lines (3.A).
  - `build-order.md` R5a entry is at **:1943**, the deviations row at :2043, R4/R5/E5 status lines at
    :1920/:1933/:1991 (all written by the brief's own commit, 3.0).
- **TANK1 Part A** (landed): `MotionConfig::tankSteer` exists; `settleFloorReport` reads only
  `headingSettle` and `brakeSettle` by name, so it is unaffected. TANK1's weak side is a TEST-local
  `DegradationModel` (`motion_tank_steer_test.cpp` :100 `TankWorld`), not a plant feature — A.6's
  per-wheel override is additive beside it and must leave it bit-identical (proved below by TANK1's own
  probes).
- **What the tree has that the brief did not know:** SRC1's `[include-root]` / `[printed-path]` gate rules
  and T4 Part 1a's `EXAMPLE_SOURCE_GLOBS` touch nothing Part A adds (no `src/` file, no example quoted in
  a chapter). `docs/api` excludes `include/shulib/sim/` by design (`api_doc_tool.py` TARGET_EXCLUSIONS),
  so A.6 needs its `///` comments but generates no page; the five new calibration/diag headers each get
  a page.

### 03:55–04:05 — pristine-tree pre-checks *(`date` at entry: see the next section's header)*
- **TANK1's hash probes, on the pristine worktree** (`scratch/r5a-a/p10_hash.cpp` and
  `probe_proposed.cpp`, copied from `scratch/tank1-a/`; built with `scratch/r5a-a/b.sh` = TANK1's `b.sh`
  flags, g++ 13.3.0 `-std=c++20 -O2`, x86-64, host `lodgepc`, `-DTANK1`): `p10_pre.txt` — **all twelve
  hashes identical to TANK1's `p10_final2.txt`** (HOLO X/H clean and hostile, the OFF switch with and
  without the log, the every-verb tank script clean and hostile, the acceptance run three ways).
  `pp_pre.txt` (43 lines: the steered tank table, the hostile seeds, the acceptance runs) is this tree's
  own baseline — it differs from TANK1's `pp_final.txt` because TANK1's review changed `leadTime` after
  that file was taken; the post-change run must match `pp_pre.txt`, not TANK1's.
- **The plant's own probe** `scratch/r5a-a/plant_hash.cpp` (evidence, not project code): eight DEFAULT-
  config runs hashing per tick the full TruthSample, every wheel's true spin, every member's
  position/velocity, the IMU, GPS, both tracking encoders and the battery — tank kA 0 and 0.02, every
  MemberFault, coast-follow 0.6 and 1, decoupled drag, the mean-command path; X-drive clean, kA 0.03,
  composed hostility with a jittered dt. `plant_hash_pre.txt`:
```
TANK kA0 plain                               hash 816f00c5e27c479a
TANK kA0.02 plain                            hash f8ec879ec593d9ff
TANK kA0.02 faults                           hash 42e6e0b99bb2049e
TANK kA0.02 faults coastFollow0.6            hash 8230efc99c54886a
TANK kA0 faults coastFollow1                 hash 4f60adfd424daa25
XDRIVE kA0 clean                             hash 19eef71e5e6e9a01
XDRIVE kA0.03 clean                          hash ee4d3da311412888
XDRIVE kA0.03 hostile jitter s7              hash 77164f15f0e59642
```
  Run-to-run stable (a second run diffs empty).
- **Baseline host suite** (briefing regenerated first — it now lists R5a as open; fresh
  `cmake -S test -B build/test`, `cmake --build build/test -j8`): **`test cases: 1725 | 1725 passed |
  0 failed | 3 skipped`, `assertions: 2648032 | 2648032 passed | 0 failed`** — TANK1's final count, so
  nothing landed between.

### A.6 first (landmine 4) — the plant's per-wheel gains and their pins *(`date` 10:38:16 AM at this entry — the wall clock read 03:55 at the end of the pre-checks and 10:38 here, with only the builds below in between; recorded as read)*
- **`include/shulib/sim/drive_plant.hpp`** (additive): `DrivePlantConfig::wheelFfOverride` — an array of
  `std::optional<control::FeedforwardGains>`, one slot per wheel, EMPTY by default, appended last; the plant
  builds a `MotorModel` only for a filled slot (`wheelModel_`), and every advance goes through
  `modelFor(w)`, which returns the SAME `model_` object for an empty slot (the three advance sites: the
  original loop, the coast-follow path, and a Decoupled member's free spin — which now spins under its
  own wheel's gains). Preconditions: a filled slot past `wheelCount()` refuses; MotorModel's own checks
  validate the gains. The header's pipeline step 3 and `wheelFf`'s comment say so (and "R5 measures" →
  "Calibrate's GAINS step is built to measure, and has not yet").
- **`test/calibration_test_rig.hpp`**: `calibPlant(kappa, kS, drag, kA = 0.0)` — existing callers unchanged.
- **`test/drive_plant_per_wheel_test.cpp`** (new, P1–P6): every slot filled with `wheelFf`'s own gains is
  memcmp-identical to the default (tank and X-drive, kA 0.02); one wheel overridden moves that wheel only,
  both orientations (the other wheel bit-identical to a default plant, the overridden one to a plant whose
  `wheelFf` is the override); the default path IS `MotorModel(wheelFf)` driven by hand, bit for bit; a
  Decoupled member on an overridden wheel spins under that wheel's gains; the override makes the two
  orientations mirror images (θ_L-faster == −θ_R-faster exactly); the preconditions.
- **Bit-identity of the default:** the existing suite **1725 cases / 2 648 032 assertions, unchanged**
  (+6 cases / +1 528 assertions, all P1–P6); **all three probe sets identical to their pristine runs**
  (`plant_hash_post.txt` = `plant_hash_pre.txt`, 8/8; `p10_post.txt` = `p10_pre.txt`, 12/12;
  `pp_post.txt` = `pp_pre.txt`, 43 lines) — TANK1's side-scaling runs included.
- **Mutations** (`scratch/r5a-a/mutate.py` + `mutations.py` batch `A6`: backup under `scratch/r5a-a/bak/`,
  one exact replacement, timestamp bumped, `cmake --build build/test --target shulib_tests/fast`, the run
  REQUIRES "Linking CXX executable shulib_tests", restore byte for byte, timestamp bumped, and after the
  batch a relink and the whole suite green). (The first attempt built the plain `shulib_tests` target,
  whose doc-gate dependency refuses a stale briefing, so nothing built — five NO-BUILD rows, no
  evidence; re-run on `/fast`.) Every one RED:
  - `A6-wrong-wheel` (override lands on wheel n−1−i) — RED: P2, P4, P5.
  - `A6-override-ignored` (`modelFor` returns `model_`) — RED: P2, P4, P5. (Its first spelling did not
    compile — an unused parameter under `-Werror` — and was re-run with `(void)w;`: not counted until then.)
  - `A6-empty-slot-falls-back-to-a-filled-one` — RED: P2, P5.
  - `A6-decoupled-on-shared-model` — RED: P4.
  - `A6-default-path-numbers-changed` (`model_` built with kV·(1+1e-15)) — RED: P1, P3 (3 cases).
  - After the batch: relinked, suite green, 1731 / 2 649 560.

### RESUMED — Tue Sep 29 07:04:01 PM EDT 2026 (`date`)
- **Why:** the team lead paused all work at a usage limit (~13:50); the previous run in this worktree
  stopped after the entry above. Its LAST log entry is the A.6 batch (10:38); the work it did after that
  was never logged, so this resumption first establishes what is on disk, from the disk.
- **Checked first:** `git status` — no `src/` file and no `Makefile` change (`git diff -- src/ Makefile`
  empty), so no gate self-test plant was left behind by the killed run. HEAD still `9239538`, nothing
  committed. Every mutation backup under `scratch/r5a-a/bak/` (the five A6 ones and ten FIT ones)
  `cmp`-equals the file it was taken from — no mutation was left applied.
- **Found on disk, not in the log** (written 10:39–11:12 by the previous run): all five new headers
  (`feedforward_fit.hpp` 499 lines, `drive_characterization.hpp` 264, `drive_characterizer.hpp` 1113,
  `rest_capture.hpp` 637, `diag/tick_timing_stats.hpp` 172); five new test files
  (`feedforward_fit_test.cpp` F1–F11/M1–M6/N1–N2, `drive_characterizer_test.cpp` C1–C19,
  `rest_capture_test.cpp` R1–R9, `tick_timing_stats_test.cpp` T1–T5, `drive_characterization_test.cpp`
  D1–D4); `docs/api` regenerated with the five new pages and `mkdocs.yml`'s nav; the briefing
  regenerated (it lists R5a open and 1,787 cases). The mutation batch `A1` (FIT, ten mutations) ran at
  11:12 — every one RED, and the relinked suite green at **1,787 / 2,652,484** — its results are in
  `scratch/r5a-a/mut_A1.txt` and `mutlogs/FIT-*.log`, recorded below. Batches `A3` (characterizer),
  `A4` (rest capture) and `A5` (timing) are DEFINED in `scratch/r5a-a/mutations.py` and were NOT run
  (no log, no backup). No register, roadmap, changelog or build-order edit had been made.
- **Read in full before continuing:** every new header and every new test file (above), to check the
  work rather than trust it; the findings are the sections below.

### 19:04–19:16 — the state on disk, built and run; what the review of it found *(`date` 07:16:47 PM at this entry)*
- **Built and run as found** (briefing regenerated first; `cmake -S test -B build/test`; `cmake --build
  build/test -j8`, exit 0): **1,787 cases / 2,652,484 assertions, 0 failed, 3 skipped** — the count the
  killed run's last batch recorded, so nothing on disk was half-written.
- **A.1–A.5 as built (read in full; the numbers are the brief's §1.A item 7 unless a delta says so):**
  - `calibration/feedforward_fit.hpp` — `FeedforwardFit`: the discrete law v[k+1] = α·v[k] + β·u[k] +
    γ·sgn(v[k]) fitted by sequential Givens QR (a 3×3 R, its right-hand side and the residual sum
    accumulated per sample; never the normal equations), kV = (1−α)/β, kS = −γ/β, kA = −kV·dt/ln α;
    `alphaMin()` = e^(−1/tauMinFraction) (τMin = dt/5) → kA exactly 0 with `kABelowResolution`;
    admission by reason (`FitExclusion` NonFinite / OffNominalDt / Stop / DeadBand / OverCurrent);
    verdicts `FitVerdict` Ok..PoorFit in the brief's order (append-only); `gainsPlausible()`;
    `FitConfig` every field PROVISIONAL (A4: HA-170).
  - `calibration/drive_characterization.hpp` — `DriveCharacterization` (source, hasGains/hasRest, both
    `SideCharacterization`s, the ramp's `TickTimingSummary`, `RestSummary`, a 96-byte provenance),
    `combined()` (the field-by-field mean, constexpr), `kVAsymmetry()`, `valid(why, cap)` naming the
    field. Trivially copyable and standard-layout (static_asserts in D1). Its fields are exactly the
    brief's Part B layout estimate (per side 7 f64 + u32 + u8 + u8; rest 10 f64 + 2 f64 + u32 + timing).
  - `calibration/drive_characterizer.hpp` — `DriveCharacterizer`: Idle → WaitImu → [Rest → Driving →
    Stopping] × 4 → Done | Aborted; the eight rules in its header; `CharacterizerConfig` (+ `valid(W)`,
    `curveGuard(W)`, `sidewaysExcursion(W)`, `clearAheadAndBehind()`); `SegmentRecord`,
    `CharacterizerReport`, `CharacterizerSample` (`lastSample()`), `CharacterizerStatus`; every refusal
    through one `refuse()` = 0 V + Coast; Brake only in Stopping.
  - `calibration/rest_capture.hpp` — `RestCapture` (WaitImu → Settle → Capture → Done | Aborted), the
    shared input types (`SideMember`, `DriveSide`, `sideSpeed/sidePosition/sideTravel`,
    `driveSideValid`), `RestCaptureConfig`, `settleFloorReport()` / `SettleFloorReport`.
  - `diag/tick_timing_stats.hpp` — `TickTimingStats` (Welford; inclusive budget; strict 2×; the first
    tick baselines), `TickTimingSummary` (+ `valid()`), `TickTimingConfig`. `LoopMonitor` untouched.
- **Deltas from the brief found in the code, each with its reason (all kept — each is right):**
  1. **Rest `Moved` is judged on the encoders' TRANSLATION (the centre's travel (dL+dR)/2), not on
     "either side's travel".** With the brief's own numbers (W 12.5 in, `moveBound` 0.25 in,
     `turnBound` 3°) a turn in place moves each side 0.25 in at 2.3°, so Moved would always fire first
     and Turned could never name a turn. Split into translation and rotation, each refusal says what
     happened (`rest_capture.hpp` header §"Moved and Turned"; R4's turn-in-place subcase pins
     Turned-not-Moved).
  2. **The fit JUDGES PoorFit before Implausible** (the enum keeps the brief's order and values). When
     the law does not describe the data its gains are meaningless, and "kS out of range" would send a
     person after the wrong thing (`feedforward_fit.hpp` header §Verdicts).
  3. **A quadratic drag NEVER reaches PoorFit** (the brief: "the first two must reach PoorFit or be
     excluded by the current guard"). The current-limited case IS excluded by the guard (M1); the drag
     is not caught by PoorFit at any size — its residual stays under 0.08 in/s while kV runs to 2.4× the
     truth, and the first refusal is NonPhysical (kS < 0) at c = 0.0105. The brief anticipated this
     half ("the largest mismatch it CANNOT catch is pinned and registered"); M4 pins it and HA-171 says
     it. The whole sweep (scratch probe `drag_sweep.cpp` = the test's own generator; `drag_sweep.txt`):
     ```
     c(V/(in/s)^2)  drag@30in/s(V)  verdict          kS        kV        kA        resid(in/s)  kS-bias  kV-bias
     0.0000           0.00          OK                1.00000   0.16000   0.02000     0.0000      +0.0%     +0.0%
     0.0005           0.45          OK                0.90455   0.17678   0.01983     0.0149      -9.5%    +10.5%
     0.0010           0.90          OK                0.82354   0.19187   0.01983     0.0256     -17.6%    +19.9%
     0.0020           1.80          OK                0.68877   0.21879   0.02005     0.0400     -31.1%    +36.7%
     0.0050           4.50          OK                0.39212   0.28575   0.02141     0.0608     -60.8%    +78.6%
     0.0080           7.20          OK                0.15475   0.34326   0.02325     0.0697     -84.5%   +114.5%
     0.0100           9.00          OK                0.02366   0.37795   0.02463     0.0734     -97.6%   +136.2%
     0.0105           9.45          NON_PHYSICAL     -0.00889   0.38645   0.02503     0.0742    -100.9%   +141.5%
     0.0110           9.90          NON_PHYSICAL     -0.04126   0.39490   0.02544     0.0750    -104.1%   +146.8%
     ```
     (every 0.0005 step between is in `drag_sweep.txt`; each is monotone between its neighbours.)
  4. **A filtered `velocity()` and a one-tick command latency are pinned, not refused** (the brief:
     "pin … or require PoorFit"): τ = 1/2/3 ticks → kS +0.8/+2.0/+2.1 %, kV −0.25/−0.6/−1.0 %, kA
     +3.3/+8.6/+19 %; latency → kS +2.8 %, kV −0.4 %, kA −0.6 %; every residual under 0.05 in/s (M5, M6).
  5. **`StopTooLong` is judged when the stop reaches rest, or when `restTimeout` runs out while still
     rolling** — not at the tick the distance passes `stopBound`. The brake is held until rest (Coast
     while rolling would roll further); the refusal still comes before any further segment (the
     brief's words). The two branches are worded apart ("… and was still rolling").
  6. `CharacterizerConfig::valid(W)` also refuses ψ_g > π (a larger guard bounds nothing a segment can
     do) and a timing nominal ≠ `fit.nominalDt`.
  7. The input types (`SideMember`, `DriveSide`) and the side readings live in `rest_capture.hpp`,
     which the characterizer includes — one definition for both steps, not two.
  8. `DriveCharacterization` carries `bool hasGains, hasRest` where Part B's record has a flags byte
     (bit 0 / bit 1); Part B maps them.
- **The review found gaps in the TESTS (the code was right in each):**
  1. **Three refusal paths had no case:** the characterizer's `ImuNotReady`, `RestTimeout` AFTER a
     segment (its own message), and `FitRefused`; and `StopTooLong`'s two branches were not told
     apart. → **C20 added** (`drive_characterizer_test.cpp`, before C19 so C19 still runs last): an IMU
     that never readies (ImuNotReady at 5 s, nothing powered, 0 V + Coast); a frozen moving reading
     on port 18 during the first stop (RestTimeout naming port 18, "after a segment", braked the whole
     wait, Coast on the refusal tick); τ = 1.5 s (still rolling at `restTimeout` → StopTooLong "still
     rolling", both orientations); τ = 0.4 s with a faster drive (rolls ~14 in and rests inside
     `restTimeout` → StopTooLong at rest, `stopped` true, the distance equal to plant truth, both
     orientations); `fit.kSMax = 1.05` (the slower side Implausible, the faster Ok → FitRefused naming
     "RIGHT IMPLAUSIBLE"/"LEFT OK" by orientation, every segment ran, no `result()`).
  2. **C13 never asserted 0 V + Coast** on its RestTimeout refusal → `CHECK(o.coastViolations == 0)`.
  3. **R1 ran ONE seed, and the mutation "drift as end minus start" SURVIVED it** (batch A4's first
     run, below). Seed 3's end-minus-start error happens to be 2.05 σ_b — a one-in-ten draw for an
     estimator 32× wider than the slope's (R1's comment said "~8×"; it is √(N/6) ≈ 32×, corrected). The
     probe `scratch/r5a-a/r1_probe.cpp` (R1's rig, seeds 1–12; `r1_probe.txt`): the slope's error is at
     most 2.18 σ_b on every seed; end minus start's is 8.6–41 σ_b on every seed but 3. → **R1 now runs
     seeds 1–8** (every statistical assertion per seed; the large-bias "no Turned though the IMU turns
     5°" assertions on seed 3), and the mutation is RED (7 of 8 seeds).
  4. `tick_timing_stats_test.cpp`'s T2 header claimed "2× judged with `>=`: red", and D2 named "combined
     = LEFT only" — no batch had run either. Both run now (below).
- **The R2 numbers its comment quotes, checked** (probe `r2_probe.cpp`, R2's rig, seeds 1–8;
  `r2_probe.txt`): σ_Δ/dt 0.00864–0.00890 rad/s (the walk's 0.00873), ρ 0.030–0.062, the walk-model
  drift σ 0.420–0.433 °/min, the white formula √2·σ_line/dt 0.140–0.292 rad/s — a false warning
  against 0.30 (margin < 2) on seeds 2–8, seven of eight; the measured margin 33.7–34.7.
- **C16's measurement, as its comment cites this log for it:** four seeds of encoder quantization (the
  A3 `EncoderHostileModel`) plus a velocity jitter of 0.3 rad/s driving / 0.01 rad/s at rest (INVENTED;
  the plant has no velocity seam): worst relative error **kS 0.02599, kV 0.00925, kA 0.09693**; pinned
  at kS ≤ 0.050, kV ≤ 0.018, kA ≤ 0.19 (each under 2× its measurement). C17 (end to end on the
  calibrator's description, scale 0.986555109 both sides): kS 1.200000, kV 0.172317, kA 0.020273 = the
  plant's 1.2 / kKv/scale / 0.02/scale to 1e-6.

### 19:04–19:18 — every §2.A mutation, run on a fresh binary and observed *(`date` 07:18:03 PM at this entry)*
Runner: `scratch/r5a-a/mutate.py` + `mutations.py` (as in the A.6 section: backup under
`scratch/r5a-a/bak/`, one exact edit that must occur once, timestamp bumped, `cmake --build build/test
--target shulib_tests/fast` with the relink line REQUIRED, the named cases run, byte-for-byte restore,
timestamp bumped; after each batch a relink and the WHOLE suite). Afterwards all 42 backups `cmp`-equal
the restored files. Filter = the cases the mutation is expected to break; each log in `mutlogs/`.
```
batch A1 (fit; run 11:12 by the stopped run, results read from mut_A1.txt / mutlogs/FIT-*)
  FIT-drop-gamma-column                      RED   24 of 30 cases (C1, C10, C11, C13-C15, F1 ...)
  FIT-kA-without-log                         RED   12 of 30 (F1, C1, C14, C17)
  FIT-alpha-vs-zero                          RED   F3
  FIT-admit-deadband                         RED   17 of 30 (C1 ... F11)
  FIT-admit-stop                             RED   14 of 30 -- ON THE PLANT: C1's gains (lines 292-294), not only its Stop count
  FIT-admit-offnominal-dt                    RED   F10, F11
  FIT-no-beta-sign-check                     RED   F6
  FIT-no-poorfit-check                       RED   M3
  FIT-normal-equations-residual              RED   N1
  FIT-current-guard-off                      RED   M1, F10, C14
  after: suite 1787 / 2,652,484 green
batch A3 (characterizer; this resumption, between the 19:04 and 19:16 readings, with C20 in)
  CH-swap-LR-accumulation                    RED   6 of 20
  CH-sign-applied-twice                      RED   17 of 20 (C1-C6 ...)
  CH-mean-not-median                         RED   C1, C3 (+3)
  CH-distance-on-mean                        RED   C5
  CH-distance-left-only                      RED   C5 -- and ONLY in the R-faster orientation (re-run by hand with the
                                                   CAPTUREs printed: every failure "R faster")
  CH-curve-guard-from-imu                    RED   C7
  CH-guard-fixed-W                           RED   C6 (the W = 25 in half)
  CH-no-missed-tick-guard                    RED   C10
  CH-refusal-leaves-last-volts               RED   6 of 20
  CH-brake-on-refusal                        RED   8 of 20
  CH-coast-at-normal-end                     RED   C1, C5, C8, C20
  CH-no-stop-bound                           RED   C8, C20 (both StopTooLong branches)
  CH-temperature-unchecked                   RED   C9
  CH-segment-before-rest                     RED   C13
  CH-fit-refused-only-when-both-fail  (new)  RED   C20
  CH-forget-inches-per-radian                RED   6 of 20
  after: suite 1788 / 2,652,571 green
batch A4 (rest capture), FIRST run (R1 on one seed)
  R-drift-end-minus-start                    GREEN -- SURVIVED (see "the review found", item 3)
  R-regression-on-wrapped-heading            RED   R3
  R-sigma-about-mean                         RED   R1, R2
  R-turned-from-imu                          RED   R1 (the large-bias case), R3
  R-settle-floor-white-formula               RED   R2 (the random walk)
batch A4 RE-RUN after R1 went to eight seeds (fresh binary)
  R-drift-end-minus-start                    RED   R1 (7 assertions: 7 of the 8 seeds)
  R-regression-on-wrapped-heading            RED   R3
  R-sigma-about-mean                         RED   R1, R2
  R-turned-from-imu                          RED   R1, R3
  R-settle-floor-white-formula               RED   R2
  after: suite 1788 / 2,652,690 green
batch A5 (timing)
  T-baseline-counted-as-dt-0                 RED   T1, T3, R1
  T-over-budget-strict                       RED   T2
batch A5b (timing; T2's header claim, unrun until now)
  T-over-double-inclusive                    RED   T2
batch A2 (the characterization; D2's claim, and the kA-below-resolution rule)
  D-combined-left-only                       RED   D2, C1
  D-valid-skips-kA-resolution-rule           RED   D3
  after: suite 1788 / 2,652,690 green
batch A6 (the plant) -- the section above: 5 of 5 RED
```
**Every mutation §2.A lists has run and gone red**, plus nine it did not list — 42 in all *(count
corrected 2026-10-04 in the review round: this line said "four" and the DoD below "five"; the batches
above hold §2.A's 33 and these nine: FIT-normal-equations-residual, CH-fit-refused-only-when-both-fail,
R-refusal-leaves-motors, T-over-double-inclusive, D-combined-left-only,
D-valid-skips-kA-resolution-rule, A6-override-ignored, A6-empty-slot-falls-back-to-a-filled-one,
A6-decoupled-on-shared-model)*. The one survivor was a test weakness (one seed), fixed at the test,
and the mutation re-run red on a fresh binary.

### 19:18 — the default plant is still bit-identical, on the FINAL tree
The three probe sets rebuilt from the final headers (`b.sh`, the same flags) and re-run:
`plant_hash_final.txt` = `plant_hash_pre.txt` (8/8), `p10_final.txt` = `p10_pre.txt` (12/12, `-DTANK1`),
`pp_final.txt` = `pp_pre.txt` (43 lines, `-DTANK1`) — `cmp` silent on all three. TANK1's side-scaling
`DegradationModel` runs are among them.

### 19:18–21:43 — the documents (§3.A), and the logged prose review *(`date` 09:43:38 PM at this entry; the gates' `make` runs took most of the wall time)*
- **One more test gap, closed on the way:** REST's R5 subcases (RestTimeout, ImuNotReady) never
  asserted 0 V + Coast; `CHECK(rig.safe())` added to both. Mutation `R-refusal-leaves-motors` (batch
  `A4b`: `RestCapture::refuse()` without `safeState()`) — **RED** on R4 (the pushed 2 V left on, both
  the push and the turn subcase); after the batch the suite relinked green at **1,788 / 2,652,692**.
- **`docs/hardware-assumptions.md`:**
  - HA-170..HA-174: five index rows (between HA-161 and HA-179; no row carries the word the briefing
    counts as "settled"), five entries in a new subsection "R5a Part A's additions" after HA-180, and
    the status paragraph (158 → 163 entries, "7 of 163 settled", HA-162..169 and 175..178 still held by
    other chunks). HA-171 carries the drag sweep, the filtered-velocity and latency biases (M4–M6) and
    says PoorFit is NOT a mismatch detector. **Delta from §3.H:** HA-171's "the ~6–10 V a 36 in segment
    reaches" is written as "the range the fit admitted (`voltsMin`–`voltsMax`), about 1–6 V on the host
    test plant" — the brief's range is not what a 36 in segment reaches: the ramp runs out of lane near
    6 V at these gains (C1's plant, kV 0.16: 36 in at ~2.9 s ≈ 5.9 V), and the first admitted sample is
    just past kS (F1: voltsMin in (0.9, 1.5)).
  - Settle lines amended, statuses unchanged: HA-45 (the settle path is the GAINS step, persisted by
    Q7; kA recorded, not applied; the plant's `wheelFfOverride`), HA-48 (twin: HA-171), HA-49 (the step
    segments' current column; twin HA-172), HA-20 / HA-21 / HA-22 (the REST step; HA-21's σ presumes
    white noise → HA-173), HA-51 (`settleFloorReport`; its "~0.12 rad/s" is √2·σ/dt, white-noise only),
    HA-53 (the GAINS step's four measured Brake stops; the plant models no brake torque), HA-32 / HA-34
    (partial: Calibrate's loop is lighter than a motion loop), HA-160 (StepBackward IS the hard reverse;
    the step and the IMU-vs-encoder difference recorded; the IMU never refuses).
  - **HA-52 NOT amended** (the brief's ruling, re-checked): nothing Part A measures settles a stall
    threshold, and `independentMotionSource` is false for drive-encoder odometry, so the check never
    reports a verdict on these robots.
  - Found by the prose review (below): HA-14, HA-45 and HA-46 cited `drive_plant.hpp:135–138`,
    `:130–135`, `:141` — already drifted before this chunk, and further by its +6 lines. Re-pointed to
    `:149–151`, `:142–148`, `:154` with a dated note.
  - Reconciliation: direction 1 (`grep -rn "PROVISIONAL (A4" include/ test/ | grep -v "HA-[0-9]"`)
    prints nothing; direction 2 for the five new IDs — every file each entry's *Source* names carries
    the ID (grep per ID; HA-172's Source was reworded to cite `feedforward_fit.hpp` for the label and
    the characterizer only for what it feeds the fit, because the characterizer's header does not
    carry "HA-172").
- **`docs/roadmap.md`:** the `tools/sysid` row (now :1163) → `[~]`, struck through and reframed (the
  Q7 ruling; Part A built, host-proven only; B–D named; kA recorded, not applied); :1144's "bare gains
  from sysid" → measured by the robot in Calibrate's GAINS step, not yet on a robot, kA recorded not
  applied; the R4 line (:1220, "per-boot bias characterization") stays `[ ]` with a note that the bias
  is now MEASURABLE (the REST step) and nothing is measured. Evidence cited by date and "the
  development log on the `shulib-v2` branch", never an internal path.
- **`docs/changelog.md`:** a new "Added since 2.3 (2026-09-29)" section at the top of API 2.3.
  **Delta from §3.A** ("Added since 2.2"): TANK1 moved the API to 2.3 after the brief was written.
  No version bump (additive only).
- **`docs/shulib-v2-master-plan.md`** :268 — a dated note after "`tools/sysid` (offline … → emits
  constants)": reframed by the Q7 ruling (the prose review's find; not in §3.A's list).
- **`docs/internal/build-order.md`:** R5a's entry gains a "Status 2026-09-29: PART A BUILT" paragraph;
  R4's and R5's status lines each gain one sentence. **The `**Next:**` pointer is NOT edited** (see
  "for the coordinator"). `HANDOFF-remaining-work.md` L7 line and `REMAINING-WORK.md` row 31: one
  clause each ("Part A built 2026-09-29").
- **`docs/api`:** the five new pages, README and all-entities were regenerated by the stopped run;
  `check-fresh` passes on the final headers (no `///` comment changed since).
- **Prose review — the greps, run and their hits read BEFORE this entry was written:**
  - by file / type: `drive_plant`, `DrivePlantConfig`, `wheelFf`, `calibPlant|calibration_test_rig`,
    `feedforward_fit|FeedforwardFit`, `drive_characteriz|DriveCharacteriz`, `rest_capture|RestCapture`,
    `tick_timing|TickTiming` over `docs/` (less `docs/api/`, `docs/internal/`) and `README.md`. Hits:
    the register (the three drifted citations above, fixed; HA-137's T6b lines — no line numbers,
    true), the changelog and roadmap (this chunk's own text), `guide/08` :175/:201/:276 (a typed
    `wheelFf` — true until Part B/D, which own those lines per §3.B).
  - by concept: `sysid` (roadmap :915 — a C-phase carry-over list, historical, left; :1985 the WS4
    row, a work-stream label, left; master plan :268 — FIXED with a note; :459 the original Phase-2
    plan, historical, covered by :268's note; :544 the H-drive derate's "sysid-measured", unaffected);
    `R5 measure` (none left); `placeholder gain|feedforward constants|feedforward gains` (guide/08
    :171, :217 — true; guide/10 :59 — a list of Chassis surface, true); `LoopMonitor`
    (diagnostics-plan :71/:120, roadmap :849/:1203, guide/06 :207 — all true: LoopMonitor is
    unchanged); `per-boot|drift` (guide/08's result lines, guide/01/03/06 prose — none claims a
    measured drift); `loop timing|loop rate` (guide/14 :251/:262 "until phase R4 measures real loop
    rates" — still true, nothing measured); `√2|0.12 rad/s|0.30 rad/s` (roadmap :1113, unrelated).
  - **Left, by the brief's assignment, and stale for an older reason:** `guide/05-getting-there.md`
    :79–82 — "measured from the actual robot by driving it through a calibration script… on a robot we
    don't have yet. Today's values match the simulator's physics". "A robot we don't have yet" has
    been untrue since 2026-08-13, and "simulator" breaks the host-test-plant rule; §3.B gives these
    lines to Part B (which makes the gains measured AND consumed) — Part B should fix both.
  - Public-doc checks on this change's added lines: no "simulat…", no `internal/`, `chunks/`,
    `RESUMING`, `build-order` or PROGRESS (grep of the `+` lines of the four public files: none); the
    new headers say "host test plant" throughout (the one "simulated" in a test comment describes
    synthetic data, not the plant).

### 21:43–22:06 — every gate, on the final tree *(`date` 10:06:15 PM at this entry)*
Order: briefing regenerated → `cmake --build build/test -j8` (the doc gates run inside it) → the suite →
briefing regenerated again with the binary present → the rest.
```
suite                         [doctest] test cases: 1788 | 1788 passed | 0 failed | 3 skipped
                              [doctest] assertions: 2652692 | 2652692 passed | 0 failed |
                              (baseline 1725 / 2,648,032: +63 cases, all this chunk's; nothing else moved)
src_build_gate.py self-test   src build gate self-test: OK (189 detector cases, each against a real build)
src_build_gate.py check       src build gate: PASS (all 6 builds link; ...) -- no src/ file changed; run to show nothing moved
ARM header gate               TU includes 197 headers -- ARM compile gate: CLEAN (all v2 headers)
CI grep (PROS)                library is PROS-free (all of include/shulib except hal/pros/)
CI grep (sim layering)        layering holds: core is sim-free
api_doc_tool self-test        api_doc_tool self-test: OK
api_doc_tool check-coverage   doc coverage: 3235 public entities across 161 headers, all documented
api_doc_tool check-fresh      (silent) rc=0
api_doc_tool check-examples   doc example scan: 467 quoted lines, all verbatim (5 source files); 3 listings of whole-run files, each one unbroken run
api_doc_tool check-removab.   removability: no public doc references docs/internal/
briefing_status.py check      briefing status: generated block current
doc_staleness_audit self-test doc staleness audit self-test: OK (10 detector/filter cases)
doc_staleness_audit           doc staleness audit: clean (27 live docs, 0 numeric claims checked)
```
`git status` after the gates: only this chunk's files (the gate rebuilt `bin/`/`.d/`, both ignored);
`git diff -- src/ Makefile` empty; HEAD `9239538`; nothing committed.

### Part A DoD — where each item stands
- [x] The five headers exist, PROS-free, sim-free, allocation-free (grep: no container, `new` or
  `<vector>/<string>/<memory>/<functional>`), every public member `///` (check-coverage).
- [x] Each side's gains to 1e-6 relative on the plant, robot two's shape (5 a side, reversed members,
  a decoupled member), sides ~5 % apart, both orientations — C1.
- [x] REST recovers the hostile IMU's drawn bias and σ within derived bounds (now over 8 seeds — R1); a
  scripted random walk gives σ_Δ/dt the walk's σ, ρ ≪ 1, no false settle-floor warning — R2.
- [x] Every refusal 0 V + Coast on the same tick, asserted at the motors (characterizer: C6, C8–C13,
  C20; REST: R4–R6); a normal end 0 V + Brake until rest, then Coast (C1, C5 — `brakeViolations`,
  `coastViolations` per tick).
- [x] The default plant bit-identical (P1/P3 in the suite; the three probe sets on the final tree); the
  whole existing suite green.
- [x] Every §2.A mutation run and observed red on a fresh binary, logged (plus nine more — 42 in all;
  one survivor found and fixed at the test).
- [x] HA-170..174 in the register; no `src/` file and no in-flight file touched (see below for the
  shared documents).
**Part A is complete as far as this worktree can take it: it waits for the coordinator's
verification, merge and commit. Parts B, C and D are not started.**

### For the coordinator (merge notes)
*(Superseded on 2026-10-04 by "Merge notes, revised" at the end of the review section below — the
landed tree moved on: API 2.4, the register's rewritten status block, DIAG1 Part B's median. Kept as
written on 2026-09-29.)*
1. **Shared documents this change edits — only R5a's own lines, but they WILL meet the other
   worktrees' edits:** `docs/hardware-assumptions.md` (the status paragraph's counts "7 of 163" / "163
   entries" / "HA-162..HA-169 and HA-175..HA-178" must be re-summed with whatever SAFE-SAVE A, DIAG1 B,
   H1 A1, L9 A1 and T4 2a register — e.g. SAFE-SAVE's HA-166; the five index rows sit between HA-161
   and HA-179); `docs/changelog.md` (a new "Added since 2.3" section at the top of API 2.3 — if L9 A1
   lands API 2.4 first, this entry belongs under 2.4's "Added since"/section order, newest first);
   `docs/roadmap.md` (:1144, :1163, :1220 only); `docs/internal/build-order.md` (R5a's entry, R4's and
   R5's status lines only); `HANDOFF-remaining-work.md` (the L7 line) and `REMAINING-WORK.md` (row 31);
   `docs/shulib-v2-master-plan.md` :268 (one note). **Generated and shared:** `docs/api/README.md`,
   `docs/api/all-entities.md` and `mkdocs.yml`'s nav (five pages added) — regenerate after merging,
   do not hand-merge; `PROJECT-BRIEFING.md` — regenerate last.
2. **Not edited, left for the merge:** build-order's `**Next:**` pointer and roadmap's "You are here" —
   every worktree of this wave would otherwise edit the same sentence. R5a Part A should appear in
   both when it is committed (RESUMING pace rule 5).
3. **Read-only dependencies on files another in-flight chunk edits:** T4 Part 2a edits
   `teleop/coupled_side_monitor.hpp` and `hal/motor_group.hpp`; `drive_characterizer.hpp` uses
   `teleop::CoupledSideMonitor(SideMonitorConfig)`, `.update(double, std::span<const MemberSample>)`,
   `.reset()`, `SideVerdict::persistedCount/persistedMask`, `MemberSample{velocityRadS, present}`
   (exactly what `drivetrain_calibrator.hpp` uses), and `rest_capture.hpp` uses
   `hal::medianInPlace(double*, std::size_t)`. If T4 2a changes any of these, re-run
   `DriveCharacterizer*` and `RestCapture*` after the merge. tank1b and H1 A1 add to `diag/`; this
   chunk adds ONE new file there, `diag/tick_timing_stats.hpp` — a name clash is the only possible
   conflict.
4. **For Part B (not done here, by the brief):** `docs/guide/05-getting-there.md` :79–82 says "a robot
   we don't have yet" (untrue since 2026-08-13) and "simulator" — §3.B owns those lines.
5. **Evidence files** (scratch, not project code): `.claude/coordinator/scratch/r5a-a/` — `mutate.py`,
   `mutations.py` (batches A1–A6, A2, A4b, A5b), `mut_*.txt`, `mutlogs/`, `bak/` (42 backups, all equal
   to the restored files), the hash probes (`plant_hash.cpp`, `p10_hash.cpp`, `probe_proposed.cpp`
   and their `_pre/_post/_final.txt`), `drag_sweep.cpp/.txt`, `r1_probe.cpp/.txt`, `r2_probe.cpp/.txt`.

---

## R5a-A review findings and their disposition

- **Started:** Sun Oct  4 09:52:59 AM EDT 2026 (`date`). Worktree `.claude/worktrees/r5a-a`, HEAD still
  `9239538`, nothing committed; every file of this chunk last written 2026-09-29 (nothing touched it since).
- **Go:** the coordinator's workflow run of 2026-10-04 (the team lead's go for R5a Part A stands): the builder
  again, with the three reviewers' findings (correctness, tests, docs — 28 entries; 29, corrected
  2026-10-06). For each: re-check it,
  fix it at the root if real, or say why not with evidence; re-run every gate; do not commit.
- **Read first:** `RESUMING.md`'s standing rules; the brief (the answers at the top, the Re-check
  amendments, D1–D12, §1 Part A, §2.A, §3.A/§3.H, §6); this log; every Part A header and test file in full;
  the reviewers' probes under `scratch/r5a-a/review-*`.
- Entries below are appended as the work goes.

### RESUMED Sun Oct 4 01:17:14 PM EDT 2026 (`date`) — the run above was stopped by the usage limit
The run that wrote the four lines above was stopped partway (its last file write 11:43:52 EDT) and
logged nothing past them. (The coordinator's task text dates the stopped run 2026-09-29 22:20–01:05;
the start line above and the scratch files' own timestamps show it was this morning's, 09:52–11:43.)
**What it left on disk, reconstructed from file times and the scratch evidence** (nothing of it was
taken on trust — every result below was re-run on this afternoon's final tree):
- 09:54 the reviewers' probes rebuilt against this worktree (`scratch/r5a-a/fix/*.cpp`); 10:03 the drag
  sweep extended to c = 0.02 (`fix/drag_sweep40.txt`); 10:09–11:23 the tests and headers below edited
  (pre-edit copies of the two largest test files: `fix/*.before-review.cpp`); 11:00 the review batch
  added to `mutations.py`; 11:06 a first full mutation pass — 94 RED, ONE SURVIVOR
  (`X-rest-maxstep-signed`: no REST case had a one-way IMU jump; R14's was added) — and 11:23 a second
  pass, 95/95 RED, suite 1801/1801 (`mut_round2.txt`, `mut_round2_final.txt`).
- 11:20–11:29 a MERGE PROBE (`scratch/r5a-a/merge-probe/`: shulib-v2 with this chunk's code overlaid,
  plus four DRAFT non-finite cases that need DIAG1 Part B) — built, never run to a recorded result.
- 11:33 three public headers reworded "the R5a log" → "development log, shulib-v2 branch" (public
  pages do not name internal logs); the three hash probes re-run, identical to the pristine tree
  (`p10_r2.txt`, `pp_r2.txt`, `plant_hash_r2.txt`); 11:37 a full build, suite 1801/1801; 11:43 the src
  gate's self-test and the ARM header gate STARTED (an empty `fix/gate_selftest_r2.log`) — stopped there.

**Checks before anything else:**
- `git status`: HEAD `9239538`, nothing committed, only this chunk's files; `git diff -- src/ Makefile`
  empty.
- **No mutation left applied.** Every file `mutate.py` mutates compared with its most recent backup in
  `scratch/r5a-a/bak/`: `tick_timing_stats.hpp`, `drive_characterization.hpp` and `feedforward_fit.hpp`
  equal; `drive_plant.hpp`, `drive_characterizer.hpp` and `rest_capture.hpp` differ from theirs ONLY by
  the 11:33 rewording (`diff`: one comment sentence each). The reviewers' own backups
  (`review-behaviour/bak/`, `review-teststrength-r2/bak/`) belong to their copies of the tree, not to
  this worktree.
- **The binary rebuilt from source before any result was trusted:** every header and test of this chunk
  touched, `docs/api` and the briefing regenerated, a full build (every dependent recompiled), then the
  suite.

### The 28 findings — each re-checked, and what was done
*(Corrected 2026-10-06: the coordinator's list holds **29**. This section's numbering skipped its 19th,
"The REST Moved and Turned thresholds are untested in magnitude", whose work — R10 — was done but never
logged; see "Found by this run", item 1, in the 2026-10-06 entry below, which also maps the numbers.)*
Numbered as the coordinator listed them: correctness 1–7, tests 8–22, docs 23–28. Two pairs overlap
(1 = 23, the REST bias; 15 ⊂ 27, DIAG1 Part B) and are answered once. Every mutation named below ran on
a FRESH binary (`mutate.py` refuses a build without the relink line) and is in the final batch at the
end of this section. Probes are `scratch/r5a-a/resume/probes/*` — the reviewers' sources, rebuilt
against the FINAL tree.

**1 (= 23). A rotation under `turnBound` biases the REST drift silently; the header understated it.**
REAL. Builder delta 1 (Moved judged on the centre's travel, not "either side's travel") raised the
rotation a capture ADMITS — a turn in place from 2.29° (the design's rule refused it as Moved) to 3.0°,
a pivot about one wheel from 1.15° to 2.29° — and delta 1's entry (above, 2026-09-29) gave only its
reason, not this cost. Reproduced on the final tree (`rest_probe.txt`, an IMU with NO drift, one nudge at
30 s): a 0.467 in LEFT push (−2.14°) → Done, drift −3.2081 °/min against 1.5·θ/T = −3.2080, ρ 0.093; a
turn in place of ±0.292 in (2.67°) → Done, +4.0088 against +4.0100, ρ 0.076; ±0.350 in (3.21°) →
TURNED. Done:
- the capture RECORDS what the encoders saw — `RestSummary::encoderTurnRad` (at the last sample),
  `encoderTurnMaxRad`, `encoderMoveMaxIn` — and states `driftTurnBiasBound()` = 3·encoderTurnMaxRad/T,
  the exact least-squares bound derived in `rest_capture.hpp`'s header (a heading offset bounded by M
  moves the slope by at most M·Σ|t − t̄|/Σ(t − t̄)² = 3·M·(N − 1)/(N·T) for odd N; one step A at t0 by
  6·A·t0·(T − t0)/T³, 1.5·A/T at mid-capture). `valid()` checks the three fields (finite, ≥ 0, the final
  rotation no larger than the largest). **Part B's layout must carry them.**
- the header's "up to turnBound/T" corrected (1.5·A/T one nudge, 3·turnBound/T worst case — 4.5 and
  9 °/min at the defaults), and delta 1's cost written there; "to four digits" (the stopped run's word)
  softened to "within 0.03 %", which is what the two probe cases show (0.003 % and 0.03 %).
- HA-170's blast radius no longer says "both loud": it names this one exception; HA-173 carries the
  nudge (and its pull on ρ) in its source, settle and blast radius.
- NOT done, open for the team lead (header, HA-170): shrinking `turnBound`, or regressing the IMU
  heading MINUS the encoders' rotation (which would tie the drift to the encoder scale and the track
  width, and to scrub).
- Tests: R10 (encoder motion 1 % under each bound finishes and is recorded exactly; 1 % over refuses —
  both directions); R11 (three real nudges on the plant: drift = the step formula within 1 %, inside
  `driftTurnBiasBound()`, the recorded rotation = plant truth to 1e-6, ρ < 0.2); D3 rows; D5.
  Mutations: `MR-rest-encoder-motion-not-recorded`, `MR-rest-maxturn-signed`, `MR-dch-bias-bound-1.5`,
  `MR-dch-valid-final-turn-unchecked`.

**2. `sidewaysExcursion` left out the stop.** REAL — the formula is the brief's own (D9/D11), built as
written. Reproduced (`excursion2.txt`, final tree): a run curving 46.2° a segment with stops of
10.8–11.2 in finishes Done and sweeps 19.36–19.66 in sideways against the old 14.61 in. Done:
`sidewaysExcursion(W)` = the powered arc to the guard PLUS the whole `stopBound` (26.6 in at W = 12.5).
The reviewer's ψ' = ψ_g·(D + S)/D formula (24.7 in) was NOT taken: it assumes the stop curves like the
segment, and it need not — one side braking short bends it harder (`excursion2.txt`: a 6.3° segment,
then a stop of LEFT 13.09 / RIGHT 1.90 in); the only bound that holds for any stop's shape is its length.
Guarding the curve DURING the stop was rejected: the brake is already on, and a refusal could only
switch to Coast, which rolls further. The header's "The clear space, and what it assumes" states the
three assumptions (per segment from its start pose; a roughly constant-rate curve — a robot that turned
ψ_g at once and ran straight would go 36·sin ψ_g = 27.4 in, and the guard refuses that early turn first:
`kS 1.0 vs 2.2` → CURVE_EXCEEDED in segment 0; every stop inside `stopBound`, refused only after it
happened). Over the WHOLE run on the host test plant the excursion from the start pose stayed inside
both numbers in every case probed (run lateral ≤ 19.66 in, ahead ≤ 41.73 in). C18 pins the formula and
that the stop's share moves one for one with `stopBound`; `MR-ch-excursion-without-stop`. HA-170 and
the changelog say 26.6 in. **The brief's D9 and §1 still say 14.6 in** — not edited here (it is the
accepted plan); Part C's "what to clear" screen prints `sidewaysExcursion(W)`, so it takes the corrected
number — the coordinator decides whether the brief gets an amendment line.

**3. StopTooLong's per-side rule was unpinned.** REAL (a test gap; the code was right). C21: one side
rolls on (τ 0.45 s) while the other brakes short (τ 0.10 s), the mean under the bound — refused
StopTooLong, the reason quoting the long side, both orientations. `MR-stop-left-only`, `MR-stop-mean`.
`stop_one_side.txt` (final tree): ABORTED STOP_TOO_LONG "LEFT +2.8 in / RIGHT +13.1 in".

**4. A blocked robot was powered ~13 s against the block and finished Done, silently.** REAL — and the
brief's gap (it lists RampTop and Timeout as normal ends, "none is a refusal"). A REFUSAL (the
calibrator's `SpinNoStart` analogue, with thresholds) changes the brief's refusal set, so it is the
team lead's decision before Part C runs the step — stated in the header's "What it does NOT refuse", the
changelog and build-order's status line. Done here, the reviewer's "at minimum" (added this afternoon):
a finished run in which any segment admitted NO sample on a side sets
`CharacterizerReport::emptySegmentWarning` and leads `warning()` with that segment, side, travel and end.
`blocked.txt` (final tree; was `warning: ''`): "QuasiForward admitted no LEFT sample (travel +0.0 in,
ended on ramp top), and 3 more segment-sides likewise -- that side's gains come from its other segments
alone: something in the way, a pinned robot, or a side held at its current limit?". Two warnings now
share one 512-char buffer, joined by "; " (the empty segment first). C25 (a wall ahead; one side at its
current limit for a whole StepForward, LEFT and RIGHT; the wall with the asymmetry warning — every
subcase in both orientations) and C1/C15 additions. Mutations: `MR-ch-empty-never-warned`,
`MR-ch-empty-left-side-only`, `MR-ch-empty-last-one-named`, `MR-ch-asymmetry-overwrites-empty`,
`MR-ch-warning-buffer-one-reason`, `MR-ch-empty-flag-not-set`. **For the team lead, measured while
writing C25:** with robot two's slipping member (port 19, decoupled) a robot blocked in front is refused
FIGHT, not finished — the decoupled member spins free while its stalled mates read 0, and the
coupled-side monitor (rightly) flags them; C25 runs without the slipping member for that reason. The
exposure until the decision is bounded by `segmentTimeout` (C22 pins it).

**5. REST's Moved and Turned were tested in one direction.** REAL (a test gap). R4 pushes forward AND
backward and turns both ways, checking the reason's sign; R10 does the bounds both ways.
`MR-rest-turned-one-sign`, `MR-rest-moved-one-sign`.

**6. Velocity-reading noise biases the gains under an Ok verdict, unregistered.** REAL. M7 pins it
(40 seeds a level on the four segments' exact law, and once with the current guard removing the steps'
opening). This binary's M7 output: σ 0.5 in/s — kS +3.35 %, kV +0.78 %, kA −18.0 %, residual 0.674;
σ 1 — +11.18 / +1.35 / −45.98 %, 1.337; σ 2 — +5.85 / +5.41 / −72.40 %, 2.566; guarded σ 0.5 —
+14.11 / −0.31 / −55.61 %, 0.663; every fit Ok. The worst single runs are POSITIVE in kS and kV (a signed
probe, `noisy_fit_signed.txt`): kS +9.1 / +25.9 / +33.1 %, kV +10.9 % at σ 2. Written into the fit
header's "What PoorFit does not catch", HA-171 (entry and index row) and the changelog. **Corrected
this afternoon:** the stopped run wrote that the current guard "roughly doubles" the bias; measured,
kS ×4.2 and kA ×3.1 — "three- to fourfold" now (header, HA-171, M7). Sizing `maxResidualRms` from a bias
tolerance, or reporting a σ estimate beside the gains, is left an HA-170 choice (HA-171's settle line).

**7. ρ = 0 on a noiseless heading read as "random walk".** REAL. `RestSummary::structureRatioDefined()`
(σ_line > 0), documented in the summary, the header and HA-173 (with the nudge's pull on ρ). A derived
accessor, not a stored flag: Part B persists `headingSigmaRad`, from which it follows. R14 (quiet plant),
D5; `MR-dch-structure-ratio-always-defined`.

**8. The Timeout and RampTop ends were never exercised.** REAL. C22 (a drive whose kS, 7 V, is above the
6 V step): the ramps end RampTop within 1.5 ticks of 5 s with |lastVolts| within one tick's ramp of
10 V, the steps end Timeout at 8 s, each braked then coasted, powered no longer than the four ends allow,
the fit refused Implausible — both orientations. `X-ch-no-segment-timeout`, `X-ch-no-ramp-top`.
`stall_probe.txt` (final tree): ramp top, ramp top, timeout, timeout; 26.0 s powered; FIT_REFUSED.

**9. The volts profile and each segment's direction were unpinned.** REAL. `run()` now checks every
Driving tick's volts against the profile computed HERE from the config and the clock (±rampRate·t,
±stepVolts), and C1 asserts each segment's direction on both sides and its last volts.
`X-ch-stepbackward-drives-forward`, `X-ch-quasibackward-drives-forward`, `X-ch-step-at-rampMax`,
`X-ch-ramp-twice-rate`.

**10. The characterizer's fit dt was never off-nominal.** REAL. C23: every 25th driving tick comes
20 ms late (under `commandTimeout`) — each such sample OffNominalDt on BOTH sides exactly, each tick
over budget, the gains exact, both orientations. `X-ch-fit-dt-fixed-nominal`.

**11. Per-side scales never differed; most C cases ran one orientation.** REAL. C4 runs both scales,
LEFT only and RIGHT only, each in both orientations (that side's kV/kA move by exactly 1/1.1, nothing
else does); R12's RIGHT scale is 1.3× the LEFT's; every C case now runs both orientations except C17,
whose calibrator rig has equal sides by construction (stated in the file header).
`X-ch-right-side-uses-left-scale`, `X-rest-right-side-uses-left-scale`.

**12. The REST encoder σ was never nonzero.** REAL. R12: each member's velocity jittered, a different
spread per side; each side's σ equals the two-pass σ of its own median × its own scale to 1e-9;
`settleFloorReport` takes the larger. `X-rest-encoder-sigma-zeroed`.

**13. The REST Settle phase was untested but for "a motor that never rests".** REAL. R13: a yaw rate
reading rotation until 1.2 s delays the capture to 1.2 + restSettleTime; one moving tick mid-settle
restarts it; a rotation that never stops refuses RestTimeout "still rotating", port 0.
`X-rest-settle-ignores-yaw-rate`, `X-rest-no-settle-time`, `X-rest-settle-not-reset-on-motion`.

**14. The fields Part B persists, and the rest-gap rebaseline, were unpinned.** REAL. C1 asserts every
`SideCharacterization` field equals its fit's, `rampTiming` equals the report's timing (memcmp), and the
run timing counts exactly the driving and stopping ticks with nothing over 2× nominal.
`X-ch-result-side-fields-dropped`, `X-ch-result-ramp-timing-dropped`, `X-ch-run-timing-not-rebaselined`.

**15 (⊂ 27). Non-finite speeds untested; DIAG1 Part B changes the median.** REAL. F10 now feeds NaN and
±Inf in the speeds before and after (and a NaN dt): nine NonFinite; F11's non-finite rows leave the
gains exact. `X-fit-nonfinite-speeds-admitted`. **FOUND while answering it** (the stopped run's draft N4,
re-run this afternoon on a merge probe): a whole side's `velocity()` NaN on ONE REST tick finished the
capture Done with `speedSigma` NaN — a summary its own `valid()` refuses — because the median of five
NaN is NaN in either tree and went straight into the σ. Fixed in `RestCapture`: a side with no finite
reading on a tick is left out (of its σ for a speed; of the Moved/Turned judgement and record for a
travel) and counted, `unreadSides()`; the header's new "A side that gives no reading" (0 on a V5, whose
adapter screens a failed read first — HA-08). R15 (the RIGHT side's velocity NaN on 5 ticks, the LEFT
side's position NaN on every tick from 59.5 s to the end: Done, `valid()`, every number equal to a clean
capture's, `unreadSides()` = 55); `MR-rest-nan-speed-added-to-sigma`,
`MR-rest-nan-travel-judged-and-recorded` (its first spelling did not compile — `travelRead` unused —
and was re-spelled; a NO-BUILD is not a red), `MR-rest-unread-travel-not-counted`. The characterizer's
own whole-side NaN tick: its fit excludes it (draft N2 below); its curve and distance checks see a NaN
on that tick only, and travel is cumulative from the segment's base, so the next tick that reads judges
it — not changed. ONE member's NaN among finite ones is DIAG1 Part B's `medianInPlace` rule, not in this
base: see the merge notes.

**16. PoorFit-before-Implausible unpinned.** REAL. M3 refits its noisy data under a `kSMax` that excludes
the fitted kS: still PoorFit. `X-fit-implausible-before-poorfit`.

**17. voltsMin/voltsMax pinned only by first/last.** REAL. F12: neither extreme first or last, and an
excluded sample beyond each end. `X-fit-voltsMin-first-only`, `X-fit-voltsMax-last-admitted`.

**18. alphaMin's VALUE unpinned.** REAL. F3: `alphaMin() == e^−5` exactly; α at 0.9·e^−5 is flagged with
kA 0, at 1.1·e^−5 reported. `X-fit-alphaMin-value-x12`.

**19. lastSample(), the battery, the hottest port, the signed step, an infinite timing σ.** REAL. `run()`
checks `lastSample()` against the motors, the IMU and the pack on every driving and stopping tick; C24
(a sagging, rippling pack; every member's own current): each segment's battery min and mean are exactly
its readings; C9 names the hotter of two; C7 adds a one-way 20° IMU jump, R14 a one-way −3° one; T4 an
infinite σ. **FOUND while answering it:** a segment's battery MINIMUM included its first tick's reading
and its MEAN did not, and that tick read the IMU and the battery twice — `beginSegment()` now takes ONE
reading for the heading baseline, the battery, the temperature and the sample
(`MR-ch-battery-begin-tick-not-in-mean`). Mutations: `X-ch-sample-position-zero`, `-current-zero`,
`-heading-zero`, `X-ch-battery-mean-is-last`, `X-ch-battery-min-is-max`,
`X-ch-toohot-names-first-not-hottest`, `X-rest-maxstep-signed` (the one survivor of the stopped run's
first pass), `MR-ch-maxstep-signed`, `X-tim-valid-sigma-inf-accepted`.

**20. `valid()` gaps.** REAL. D3: +Inf in each gain and the residual, a negative voltsMin; N2: minSpeed 0
and dtTolerance 0. `X-dch-valid-{kS,kV,kA,residual}-inf-accepted`,
`X-dch-valid-voltsMin-negative-accepted`, `X-fit-valid-minSpeed-zero-accepted`,
`X-fit-valid-dtTolerance-zero-accepted`.

**21. The plant's coast-follow path unpinned under an override.** REAL. P7: an overridden DRIVEN wheel
under coast-follow 0.6 spins bit for bit as on a coast-follow plant whose `wheelFf` is the override,
the follower with it, both orientations. `X-plant-coastfollow-ignores-override`.

**22. C19 and R9 failed alone.** REAL. Every reason and warning is now checked where it is produced
(the `run()` helpers); C19 and R9 check only the names. Run alone: 1/1 each; the chunk's cases under
`--order-by=rand --rand-seed=20261004`: all green.

**24. HA-171's evidence credited to the plant; M4's "0.08 at c = 0.02".** REAL. HA-171's row and entry
say "synthetic data generated off the law" (the plant has no drag, filter, latency or noise term); M4
sweeps to c = 0.02 past the first refusal, pins 0.0813 at c = 0.02 and "never PoorFit"
(`drag_sweep40.txt` on the final tree: identical to the reviewer's).

**25. Present tense for steps no program runs.** REAL. Roadmap :1144 and :1240; HA-20's settle path,
HA-32's and HA-34's amendments, HA-49's ("once a robot session runs it… nothing has been measured
yet"); HA-160: "reproduces the condition of" that hard reverse, and "whether the 2026-09-25 episode
itself began from rest was not established".

**26. The sysid grep left out the register.** REAL — logged now. `grep -rni sysid docs/ README.md`
(less `docs/internal/chunks/`, `docs/api/`), each hit read:
- register: HA-14 :507 ("affects R5 sysid bookkeeping" — true: the GAINS step's gains are in the
  description's scale; left); HA-17 :542 (an X-drive's mount asymmetry "falls out of R5's sysid fit" —
  NOT true of the GAINS step, which is tank-only: a dated settle-path note added); HA-85 :2986 (the peak
  acceleration "falls straight out of the sysid ramp": note added — the step segments carry the run's
  hardest acceleration, a lower bound, not a full-power launch); HA-45 :3132/:3138 (generic, and the old
  `tools/sysid` settle line sits directly above Part A's own amendment; left); HA-47 :3169 (velocity at
  fixed commands across pack states "during sysid": note added — the step segments are exactly that
  measurement); HA-54 :3284/:3288 (an H-bot's sustained strafe, a measurement no R5a step makes or
  claims; left); HA-56 :3328 (top speed "during sysid": note added — the GAINS step does NOT reach top
  speed, so its implied free speed is an extrapolation);
- master plan :268 (fixed on 2026-09-29 with a note), :459 (the original Phase-2 plan, historical),
  :544 (the H-drive derate, unaffected); roadmap :915 (a C-phase carry-over list, historical), :1164
  (struck through, with R5a's reframing under it), :1987 (the WS4 label);
- `docs/internal/`: PROJECT-BRIEFING :914 (open decision #3 in the durable narrative, answered by
  build-order's ruling; not R5a's to edit), build-order :929/:933 (that ruling, historical), :1932 (R5's
  heading; its status line names R5a), :2059 and :2085 (the deviations table — R5a's row is :2059).

**27. The merge notes no longer match the landed tree.** REAL — rewritten: "Merge notes, revised"
below.

**28. Two prose slips.** REAL. The HANDOFF's L7 line: the title sentence is whole again ("…inside shulib
Calibrate.") and the Part A clause follows it. This log's mutation count: "plus four" (the batch
summary) and "plus five" (the DoD) both corrected to nine — 42 in all, the nine named.

**Found while fixing, no finding named them:**
- **doctest's `Approx` is relative only above 1** (its tolerance is ε·(scale + max(|a|, |b|)) with scale
  1): `Approx(0.02).epsilon(1e-6)` accepts ±1e-6, 5e-5 relative. Every pin in the five test files now
  goes through a `rel()` helper (scale 0). *(Not yet true when written: nine default-`Approx` checks
  remained, converted on 2026-10-06 — "Found by this run", item 2, below.)* The stopped run's comment
  says a mutation survived exactly that; which one is not recorded — the final batch below is red
  throughout.
- **A refused segment's record said it went nowhere** (0 in, 0°, 0 s — `excursion2.txt` before:
  "seg 0 none curve +0.0 deg travel +0.00"; after: "-49.9 deg travel L +22.84 R +11.95"). `refuse()`
  now keeps how far the segment got (`keepSegmentSoFar()`); C6, C10, C11, C20;
  `MR-ch-refused-segment-not-kept`.
- **The panel's travel was not reset at a segment's start**, so an abort on a segment's first tick
  would have recorded the previous segment's travel; C11; `MR-ch-status-travel-not-reset`.
- The battery's minimum and mean over different readings (19), the whole-side NaN REST tick (15), the
  "roughly doubles" wording (6), "to four digits" (1).

### RESUMED AGAIN Tue Oct 6 02:54:31 PM EDT 2026 (`date`) — the afternoon run of 2026-10-04 was stopped too
The run that wrote "The 28 findings" above was stopped after its last file write (15:44:42 EDT,
2026-10-04, the briefing). It never logged the final mutation batch, the gates, or the "Merge notes,
revised" that its entries point at. (The coordinator's task text again dates the stopped run 2026-09-29
22:20–01:05; the disk shows 2026-10-04 13:17–15:44.) **What it left on disk** (from file times and
`scratch/r5a-a/resume/`):
- 14:00–15:34 four full builds and suites (`resume/build_full*.log`, `suite_full4.log`); 14:32
  C19/R9 run alone and the chunk under `--order-by=rand` (`alone_and_rand.txt`); 14:41 the probes
  rebuilt (`resume/probes/`).
- 15:00–15:32 a MERGE PROBE on shulib-v2 `fab1ebb` (the head today, too) with this chunk's files
  overlaid, plus DIAG1 Part B's own median test and the four draft non-finite cases
  (`resume/merge-probe/`): two red runs (15:02 — C25 before its code was in the probe tree; 15:07 — R15
  before the REST NaN fix was), then 15:32 **2185 / 2185 green**. Its copies of this chunk's 13 files
  `cmp`-equal the worktree's today.
- 15:35–15:42 the FINAL mutation batch `FINAL_ALL` (`resume/mut_final2.txt`): **104 mutations, 104 RED,
  every one on a relinked binary** (`mutate.py` refuses a run without the link line); after it the
  restored tree relinked and the whole suite green at **1803 / 2,654,208**.
- 15:43–15:44 `docs/api` and the briefing regenerated; a fresh `cmake -S test -B` and a full build
  (`resume/fresh_*`, `final_*`); `final_suite.log` written at 15:44:35. Stopped there: no gate run.

**Checks before anything else (this run):**
- `git status`: HEAD `9239538`, nothing committed, only this chunk's files; `git diff -- src/ Makefile`
  empty, no untracked file under `src/` or `tools/` — no gate self-test plant left behind.
- **No mutation left applied:** each of the six files `mutate.py` mutates `cmp`-equals its latest backup
  in `scratch/r5a-a/bak/` (drive_characterizer.hpp ← `MR-ch-empty-flag-not-set`, rest_capture.hpp ←
  `MR-rest-unread-travel-not-counted`, feedforward_fit.hpp ← `X-fit-valid-dtTolerance-zero-accepted`,
  drive_characterization.hpp ← `X-dch-valid-voltsMin-negative-accepted`, tick_timing_stats.hpp ←
  `X-tim-valid-sigma-inf-accepted`, drive_plant.hpp ← `X-plant-coastfollow-ignores-override`). No
  backup exists for any test file (no mutation touches one).
- **The batch ran against ONE code state, today's:** every backup the 104 mutations took of a given file
  has the same SHA-256 (one hash per file, six files), and that state is the worktree's.
- **The binary is not trusted:** it is relinked from source below (briefing first, then a full build)
  before any suite result is used.

### 14:54–15:45 — re-verified on a fresh binary, what this run found, every gate *(`date` 03:45:15 PM at this entry)*
Evidence for this section: `scratch/r5a-a/resume2/`.
- **A fresh binary:** `docs/api` and the briefing regenerated; `cmake -S test -B build/test`; `cmake
  --build build/test --target clean`; a full build (182 objects compiled, `shulib_tests` linked, no
  warning); the suite: **1803 cases / 2,654,208 assertions, 0 failed, 3 skipped** — the count the stopped
  run's last batch recorded. The briefing regenerated again with the binary present.
- **A trap the runner caught, written down so the next run does not fall in it.** The first re-run of the
  batch after that clean build came back **104 × STALE ("no relink line") — nothing ran**
  (`mut_final3.STALE-empty-depfile.txt`; every file it touched `cmp`-equal to its backup afterwards). `clean`
  empties CMake's consolidated header-dependency file (`build/test/CMakeFiles/shulib_tests.dir/
  compiler_depend.make`: 115 bytes); only a NORMAL build refills it from the `.d` files, and `mutate.py`
  builds `shulib_tests/fast`, which then sees no header dependency and rebuilds nothing. One normal build
  refilled it (2.39 MB); a touched header then rebuilt and relinked under `/fast`, and a second `/fast`
  was a no-op. **After a `clean`, run one normal build before `mutate.py`.** `mutate.py`'s refusal to
  count a run without the link line is what turned this into a loud STALE instead of 104 false REDs or
  GREENs.
- **The final batch, re-run on today's tree** (`FINAL_ALL`, `mut_final4.txt`, 15:02–15:11): **104 of 104
  RED, each on a relinked binary** — the same ids and verdicts as 2026-10-04's `mut_final2.txt`; every
  mutated file `cmp`-equal to its backup afterwards; the relinked suite green at 1803 / 2,654,208. Every
  mutation the dispositions above name is in it (checked by script: none named and missing). By group:
  §2.A and the first round's nine (42: FIT 10, D 2, CH 16, R 6, T 3, A6 5), the review round's 53 (MR-stop
  2, MR-rest 4, MR-dch 3, MR-ch 5, `X-` 39) and the resumed afternoon's 9 (`MR-ch-empty-*` and kin 6,
  `MR-rest-nan-*` and kin 3) — 104, counted from `mutations.py`'s batches. Per mutation, cases failed:
  `mut_final4.txt`.
- **Every probe rebuilt from source and re-run** (`probes.sh`: `b.sh`'s flags; `probes_result.txt`): **13 of
  13 outputs byte-identical** to the ones the dispositions quote — `rest_probe`, `excursion`, `excursion2`,
  `stop_one_side`, `blocked`, `stall_probe`, `drag_sweep40`, `noisy_fit`, `noisy_fit_climit`
  (`-DAMAX=100.0`; one stored build log in `fix/` shows an attempt without the define, which did not
  build — the output quoted is the 100 in/s² run), `noisy_fit_signed`, and the three plant-hash sets
  against the PRISTINE tree's (`plant_hash_pre.txt` 8/8, `p10_pre.txt` 12/12, `pp_pre.txt` 43 lines):
  **the default plant is still bit-identical**, TANK1's side-scaling runs included.
- **The merge probe, refreshed:** `resume/merge-probe/tree` (shulib-v2 `fab1ebb`, today's head, with this
  chunk's 13 files overlaid — `cmp`-equal to the worktree's after today's three test files were copied in —
  plus DIAG1 Part B's own median test and the four draft non-finite cases): **2185 / 2,766,000 green**
  (`resume/merge-probe/suite4.log`).

**Found by this run:**
1. **The coordinator's list holds 29 findings, not 28.** The section above numbers 28: its 1–18 are the
   coordinator's 1–18, and its 19–28 are the coordinator's 20–29 (so its "1 = 23" is the coordinator's
   1 = 24, and "15 ⊂ 27" is 15 ⊂ 28). It has no entry for the coordinator's **19, "The REST Moved and
   Turned thresholds are untested in magnitude"** (R4's open-ended 2 V pushes refuse at ANY threshold).
   Its work WAS done by the run of 2026-10-04 — it is only unlogged. Disposition: **REAL (a test gap);
   fixed by R10** (`rest_capture_test.cpp` :575–629): the encoders read a translation, and a turn in place,
   1 % UNDER its bound (Done; the summary records exactly that motion, and `driftTurnBiasBound()` is
   3·|turn|/T) and 1 % OVER it (refused Moved or Turned by name, 0 V + Coast), each in both directions.
   `X-rest-turned-with-double-width` (Turned on 2W — about 6°) and `X-rest-moved-tenth` RED in today's
   batch.
2. **Nine checks were still on doctest's DEFAULT `Approx`** (epsilon about 1.2e-5 at scale 1), though
   each file's `rel()` comment says "every pin in this file means relative, so each goes through here",
   and the stopped run's entry above says every pin now does. Converted to `rel()`: D2's `combined()` kS,
   kV, kA and `kVAsymmetry()` (1e-12); C6's `curveGuardRad`, C18's `curveGuard` and
   `clearAheadAndBehind()` (1e-12); T3's three single-tick dt values (1e-9). The other explicit forms
   stay: R10's `.epsilon(1e-9).scale(1e-12)` and `.scale(1e-15)`, and T3's `Approx(0.0).epsilon(1e-12)`
   (a zero target, which `rel()` cannot take). Three mutations computing each value in FLOAT precision
   (batch `RESUME3`, `mut_resume3.txt`, each on a relinked binary):
   - `P-dch-combined-in-float`: RED **only** at D2 :113 — 1.05 against float's 1.0499999523, a 4.5e-8
     deviation that the old ±2.4e-5 tolerance accepted. **A real catch.**
   - `P-ch-curveguard-in-float`: RED at the converted C6 :733 and C18 :1311, and at C18 :1313, which was
     already relative. Caught before too.
   - `P-tim-dt-in-float`: RED at R1 :293 and T1 :100–101, both already relative. Caught before too. So
     the T3 conversions are consistency, not a new catch.
   - The four existing mutations that run through the edited lines re-run RED on fresh binaries
     (`mut_retouched.txt`): `D-combined-left-only`, `CH-guard-fixed-W`, `T-baseline-counted-as-dt-0`,
     `MR-ch-excursion-without-stop`. The suite after: 1803 / 2,654,208, unchanged.
3. **The HANDOFF's L7 slip (finding 29) was already in the base** (`9239538`'s line reads "…Q7 ruled: the SD
   card). inside shulib Calibrate."), from the brief's own commit, not from this chunk's insertion. It is
   fixed either way. shulib-v2 still carries the slip at its :488, so the merge takes this side's line.
4. **The draft non-finite test's header names the wrong base.** It says it was built on `237c4fe`; that was
   the MORNING probe (`scratch/r5a-a/merge-probe/`, 11:20, before `fab1ebb` existed). The afternoon probe
   that ran it green is on `fab1ebb` (`resume/merge-probe/base.txt`; its tree holds SAFE-SAVE A's
   `record_store.hpp`). Scratch, not project code; noted for the coordinator, who adds the cases at the
   merge.

**The docs dispositions re-checked against the files** (grep, hits read): HA-171's index row :280 and
entry :2159 say "synthetic data generated off the law"; HA-170's blast radius (from :2137) names the REST
exception; HA-160 :2340 says "reproduces the condition of"; the four sysid settle-path notes sit inside
HA-17 (:543), HA-85 (:2988), HA-47 (:3170) and HA-56 (:3331); the sysid hit list above matches today's
`grep -n -i sysid` line for line; every "26.6 in" and "14.6 in" (register, changelog, header, API page,
build-order) gives the corrected clear space.

**Every gate, on the final tree** (after the last edit to any header or test; the order of the
instructions: briefing, full build, suite, briefing again, the rest):
```
suite                          [doctest] test cases: 1803 | 1803 passed | 0 failed | 3 skipped
                               [doctest] assertions: 2654208 | 2654208 passed | 0 failed |
                               (baseline at 9239538: 1725 / 2,648,032 -- +78 cases, all this chunk's)
src_build_gate.py self-test    src build gate self-test: OK (189 detector cases, each against a real build)
src_build_gate.py check        src build gate: PASS (all 6 builds link; ...) -- no src/ file changed
ARM header gate                TU includes 197 headers -- ARM compile gate: CLEAN (all v2 headers)
CI grep (PROS)                 library is PROS-free (all of include/shulib except hal/pros/)
CI grep (sim layering)         layering holds: core is sim-free
api_doc_tool self-test         api_doc_tool self-test: OK
api_doc_tool check-coverage    doc coverage: 3242 public entities across 161 headers, all documented
api_doc_tool check-fresh       (silent) rc=0
api_doc_tool check-examples    doc example scan: 467 quoted lines, all verbatim (5 source files); 3 listings of whole-run files, each one unbroken run
api_doc_tool check-removab.    removability: no public doc references docs/internal/
briefing_status.py check       briefing status: generated block current
doc_staleness_audit self-test  doc staleness audit self-test: OK (10 detector/filter cases)
doc_staleness_audit            doc staleness audit: clean (27 live docs, 0 numeric claims checked)
```
`git status` after the gates: only this chunk's files (the gate rebuilt `bin/`/`.d/`, both ignored);
`git diff -- src/ Makefile` empty; HEAD `9239538`; nothing committed. The scratch TU for the ARM gate is
`resume2/all_headers.cpp`, never `/tmp` or the repository root.

### Merge notes, revised *(2026-10-06; supersedes "For the coordinator (merge notes)" above)*
1. **Where the landed tree is.** shulib-v2 is at `fab1ebb` (`git rev-parse`, 2026-10-06), eleven commits past
   this worktree's base `9239538`: `ee2e399` (plan), `a5bd83f` TANK1 B, `0f56301` COMP-t, `fa915b0` L9 A1
   (API 2.4), `ce95325` (plan), `9270ac2` DIAG1 B, `de2a4f8` (plan), `9803928` H1 A1, `7daf809` T4 2a,
   `237c4fe` COMP-b, `fab1ebb` SAFE-SAVE A. The merge probe above builds and passes this chunk on top of it.
2. **No file name clashes.** None of this chunk's new files exists on shulib-v2 (the five headers, the six
   test files, the five `docs/api` pages). `diag/` gained `crc32.hpp`, `shul2_format.hpp` and
   `shul2_reader.hpp` (H1 A1) and a changed `motion_result.hpp` (TANK1 B) — no clash with
   `tick_timing_stats.hpp`. Tests are globbed (`test/CMakeLists.txt` lists no test file by name here).
3. **`docs/changelog.md`.** This chunk's section ("### Added since 2.3 (2026-09-29) — the drive's gains and
   the IMU's noise, measured by the robot: the library half") sits at the top of API 2.3 here. On shulib-v2,
   "## API 2.4" (L9 A1) is at :17, with four "since 2.4" sections above L9's (SAFE-SAVE A 10-04, T4 2a
   10-01, H1 A1 10-01, DIAG1 B 10-01). It becomes "### Added since 2.4 (<the commit's date>) — …" at the
   top of API 2.4, newest first. It is additive only, so there is no version bump.
4. **`docs/hardware-assumptions.md`: re-sum against main's status block, never take this side's.**
   - The status line on shulib-v2 reads "7 of 166 settled". With HA-170..HA-174 it becomes **"7 of 171"**,
     or more if anything else registers first.
   - This side's sentence "R5a Part A (2026-09-29) added FIVE … The register then holds 163 entries.
     HA-162..HA-169 and HA-175..HA-178 are not in it" is rebuilt on main's paragraph. After SAFE-SAVE A's
     "with it, 166.", add R5a's "added FIVE — HA-170 to HA-174 … with them, 171". Main's "The rest of
     HA-162..HA-176 (all but HA-162, HA-165, HA-166, HA-167, HA-175 and HA-176)" gains HA-170..HA-174 in
     its "all but" list.
   - The "Held numbers" line (HA-162..HA-176, HA-181..HA-186) needs no edit: "a number that lands as a row
     may stay in its range".
   - Placement: the five index rows go between main's HA-167 and HA-175 rows. The section "### R5a Part
     A's additions" goes just before "## Group R4" (main :2600), after T4 2a's HA-176 entry, in the order
     the file adds sections.
   - The amendments to existing entries (HA-14, HA-17, HA-20/21/22, HA-32/34, HA-45/46/47/48/49, HA-51,
     HA-53, HA-56, HA-85, HA-160) are this chunk's own lines. **Main changed none of those entries**
     (each entry's text compared between `9239538` and `fab1ebb` by script; of the entries checked, only
     HA-08 differs on main — DIAG1 B's amendment — and this chunk does not touch HA-08). HA-180 differs
     here only by the `---` rule that now follows R5a's new section. Main has new text in the same file
     around these entries, so the merge must be by hunk.
5. **DIAG1 Part B's median — the one landed change that moves this chunk's behaviour.**
   `hal::medianInPlace` (shulib-v2 `motor_group.hpp` :266–279) now leaves out a non-finite reading and
   returns a fresh NaN only when none is finite. `sideSpeed`, `sidePosition` and `sideTravel`
   (`rest_capture.hpp`) go through it, and with them both steps. After the merge, ONE member's NaN is left
   out of its side's median. In this base it was sorted as a NaN, in an unspecified order. A whole side's
   NaN is still NaN: REST leaves it out and counts it (R15), and the GAINS fit counts it NonFinite.
   **Add the four draft cases N1–N4** (`scratch/r5a-a/resume/merge-probe/tree/test/
   r5a_nonfinite_draft_test.cpp`; seen green on `fab1ebb`; its header's "237c4fe" is the morning probe's
   base, see "Found" item 4). Then **re-run `DriveCharacterizer*`, `RestCapture*` and `FeedforwardFit*`**.
   DIAG1 B also changed `drivetrain_calibrator.hpp` (+159 lines). The `CalibratorConfig` defaults that
   `RestCaptureConfig` and `CharacterizerConfig` copy (restFloor, restYawRateFloor, restSettleTime,
   restTimeout, imuReadyTimeout) are unchanged; checked by `git diff`.
6. **T4 Part 2a** added `FrozenReadingDetector` to `teleop/coupled_side_monitor.hpp` and reworked
   `hal/motor_group.hpp`. `CoupledSideMonitor` as this chunk uses it is unchanged: its constructor,
   `update(double, span<const MemberSample>)`, `reset()`, `SideVerdict::persistedCount`/`persistedMask`
   and `MemberSample{velocityRadS, present}`. The merge probe compiles and passes against it.
7. **Shared plan documents** (this chunk's own lines only; locate them by text, since main moved every line
   number): `docs/roadmap.md` (the "bare gains" line, the struck-through `tools/sysid` row and its
   reframing, the R4 per-boot line); `docs/internal/build-order.md` (R5a's entry and its status paragraph,
   R4's and R5's status lines); `HANDOFF-remaining-work.md` L7 (take this side's line; see "Found" item 3);
   `REMAINING-WORK.md` row 31; `docs/shulib-v2-master-plan.md` :268's note. Main has edited every one of
   these files since `9239538` (`git diff --stat`: 1,271 lines across the seven), so expect hunks to meet.
8. **Generated files: regenerate after the merge, never hand-merge.** `docs/api/README.md`,
   `docs/api/all-entities.md` and `mkdocs.yml`'s nav (this chunk adds five pages; main's nav gained ten
   lines) via `api_doc_tool.py generate`. `PROJECT-BRIEFING.md` last, via `briefing_status.py generate`,
   with the suite binary present.
9. **Not edited here, left for the merge:** build-order's `**Next:**` pointer and roadmap's "You are here"
   (RESUMING pace rule 5). R5a Part A must appear in both when it is committed.
10. **The brief's D9 and §1 still say 14.6 in** for the clear space. The code, HA-170, the changelog and
    build-order say 26.6 in (finding 2). Whether the brief gets an amendment line is the coordinator's
    call. Part C's "what to clear" screen prints `sidewaysExcursion(W)`, so it takes the corrected number
    either way.
11. **Open for the team lead, collected from the dispositions** (none is decided here):
    - (a) refusing a segment that does not move — a `SpinNoStart` analogue; until then it is warned
      (finding 4);
    - (b) shrinking `turnBound`, or regressing the IMU heading minus the encoder rotation (finding 1);
    - (c) sizing `maxResidualRms` from a bias tolerance, or reporting a reading-σ estimate beside the
      gains (finding 6);
    - (d) tracking the run's pose for the clear space, rather than per segment (finding 2);
    - (e) on robot two, a robot blocked in front is refused FIGHT through the slipping port 19, not
      finished (finding 4's measurement).
12. **Evidence** (scratch, not project code), all under `.claude/coordinator/scratch/r5a-a/`:
    - `mutate.py` and `mutations.py` (batches through `FINAL_ALL` and `RESUME3`), `mutlogs/`, and `bak/`
      (every backup equal to the restored file);
    - `resume/` (2026-10-04: probes, merge probe, `mut_final2.txt`);
    - `resume2/` (today: the fresh build and suite, `mut_final4.txt`, `mut_resume3.txt`,
      `mut_retouched.txt`, the STALE run, `probes.sh` and `probes/`, the gate logs).

### Part A DoD, after the review round *(2026-10-06)*
Unchanged in substance from "Part A DoD" above, with the review's additions:
- 29 findings, each re-checked and dispositioned (above).
- 107 mutations observed RED on fresh binaries today: `FINAL_ALL`'s 104 and `RESUME3`'s 3.
- The suite has 1803 cases (+78 over the base).
- 13 probes reproduce, and the default plant is bit-identical.
- Every gate is green.

Five decisions are open for the team lead (merge note 11). **Part A waits for the coordinator's
verification, merge and commit. Parts B, C and D are not started.**

## The coordinator's merge — Part A (2026-10-06, 15:55–16:45 EDT)

- **The run:** the build (2026-09-29), three reviews (29 findings), the fix round — across usage-limit stops and
  session ends, resumed with continuation notes; 104 mutations red. Transport commit `118e92c` in the worktree
  `r5a-a`, squash-merged onto `fab1ebb`.
- **No code conflict.** Document conflicts resolved as this log's "Merge notes, revised" asked: the changelog (this
  part's section under `## API 2.4`, "Added since 2.4 (2026-10-06)"), the register (status 7 of 171; this part's
  sentence after SAFE-SAVE A's; the five rows between HA-167 and HA-175; its section after T4 2a's), REMAINING-WORK
  (row 31; §11 gains "R5a's five" as an open row), build-order (R5a's status, the Next line). `docs/api`, the nav and
  the briefing regenerated.
- **The four drafted cases N1–N4 ADDED** as `test/r5a_nonfinite_test.cpp` (R5a Part A x DIAG1 Part B's median). N4
  only printed what the summary did; the coordinator ran it, read the behaviour (the NaN tick left out and counted,
  both sides' σ exactly 0, the summary valid) and PINNED it with four checks. All four cases green.
- **Gates on the merged tree:** suite **2,185 | 2,185**, 3 skipped; `src_build_gate.py` self-test OK and check PASS
  on all eight builds; the ARM header gate (206 headers); both CI greps; `check-fresh`, the briefing, removability
  and the staleness audit; no plant (`.claude/coordinator/logs/r5a_merge_verify.log`).
- **Five coordinator mutations, each on a fresh binary, all RED** (`muts_r5a_a.json`, `muts_r5a_a_b.json`): the
  distance limit judged on the mean instead of each side (1 case); a hot motor still starting a segment (1); a refusal
  leaving the motors powered (4); the rest capture's turn read with the wrong sign (3); the plant ignoring a per-wheel
  override (18 — its first form left a parameter unused and did not compile, so it proved nothing until re-written).
- **Not done:** Parts B (the card and the builder), C (Calibrate runs the steps — now after CAL1, the standard
  Calibrate) and D. Nothing of R5a has run on a robot.

