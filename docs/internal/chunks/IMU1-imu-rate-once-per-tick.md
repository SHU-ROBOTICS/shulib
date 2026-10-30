# IMU1 — the IMU's turn rate is read ONCE per control tick, and every reader in that tick gets the same value

> **THE TEAM LEAD'S ANSWERS (2026-10-22)** to "Decisions for the team lead": **"Accept all (Recommended)."**
> - **D1 (a) — DECIDED, as recommended:** the robot's IMU adapter keeps its turn-rate answer for 5 ms, so every reader in
>   one control tick gets the same value; its three named limits (a stall or late wake; the tick's first read moving; a
>   reader outside the tick) stated in the code and the changelog.
> - **D2 (a) — DECIDED:** the frozen `IImu` interface is NOT touched.
> - **D3 (a) — DECIDED:** Part B, a short Bench Tests stage measures the IMU on the robot (worksheet Station N).
> - **D4 (a) — DECIDED:** IMU1 lands BEFORE Station M's floor run (its probe found the double read can turn the 40 in
>   auton from FINISHED into a 5 s timeout in one band of an unmeasured timing).
> - **Parts and sizes:** Part A ~1¼ builder-days, Part B ~½. **HA-213..HA-215** — the collision with L6 RESOLVED
>   2026-10-22: L6 was accepted first and keeps HA-204..HA-212; every IMU1 id below is renamed (204→213, 205→214, 206→215).

> **Status 2026-10-18, finished 2026-10-22 (the first draft run stopped at a session limit after §2); RE-CHECKED
> 2026-10-22 — 18 independent findings, each re-verified and dispositioned in "Re-check — amendments" below: DRAFT
> for the team lead's go — WAITS for his answers to D1–D4. Nothing is built.**
> Execution brief written by the coordinator's brief-writer from a read-only pass over `shulib-v2` at `ceb17c0`
> (main; worktrees `mech-a3` and `cal1-a2a` in flight, the VIS1 brief awaiting the team lead — and by 2026-10-22 VIS1-A1
> is building too, worktree `vis1-a1`, §6). **Probed on the host
> test plant, not argued:** the REAL `ProsImu` (today's, and a prototype of the fix) over the PROS host shim, with the
> IMU modelled the way the vendored PROS header documents it, was run through the REAL Chassis, MotionScheduler,
> Localizer and TANK1's MoveToPose — §0.4. **The probe found the defect is worse than the plan recorded:** on robot
> two TODAY, the motion record reads the rate a second time on EVERY tick (the robot's serial sink asks for records), and
> that second read moves the adapter's baseline, so it corrupts the NEXT tick's FIRST read — the Localizer's own rate,
> the one TANK1's yaw observer and `DriveBrake` use. Modelled with TANK1's own motor-lag values, that turns the 40 in
> auton from FINISHED into a 5 s TIMEOUT with 12–17° of heading wobble, for one band of an unmeasured timing (§0.4,
> probe 2). TANK1's "immune" (its completion record :67) rested on a model of the second read that left this out.
> **Size: Part A ~1¼ builder-days (~1 before the re-check; +¼ for its added test rows); Part B (D3 (a), a bench
> stage) ~½ builder-day.** HA: **HA-213..HA-215 reserved**
> (§4) — the register's Held line gains them at the plan commit (VIS1 holds HA-192..HA-203); **the L6 brief claims
> the same numbers** (the first continuation note).
> **Shape:** SER1's brief (`docs/internal/chunks/SER1-serial-lines-whole.md`) and the CAL1 / MECH-core briefs.
>
> **Continuation notes (2026-10-22 — the code is still `ceb17c0`'s; main moved to `71b4764` during the continuation, a
> documents-only plan commit; every correction below was re-opened):**
> - **HA NUMBER COLLISION — the coordinator must resolve it at the plan commit.** The L6 brief
>   (`.claude/coordinator/briefs/L6-wall-reset.md` §4, written beside this one on 2026-10-18) ALSO claims HA-204..HA-212.
>   The register's Held line (`docs/hardware-assumptions.md` :39, committed in `71b4764` during this continuation) holds
>   only VIS1's HA-192..HA-203. Whichever brief the team lead accepts first keeps HA-213; the other shifts. **Recommended: IMU1
>   becomes HA-213..HA-215** (three ids, a mechanical rename in this file; L6's nine are cited through its whole body),
>   unless IMU1 is accepted first. This brief keeps the labels HA-213/205/206 so nothing done is renumbered silently.
> - `ProsTickPacer::kTickMs` is at `hal/pros/tick_pacer.hpp` **:51**, not :52 — corrected in place (D1 (a), §1, T3).
> - The tick-pacer limit was mislabelled "the pacer's catch-up after an overrun": `pace()` RE-ANCHORS after a body that
>   overran a whole period (:60-69) and gives a full 10 ms; the short interval comes from a LATE WAKE (another task held
>   the processor) followed by an on-time one, because `delay_until` keeps the 10 ms grid (:7-14). Corrected in place.
> - Probe 3's cases (4)/(4′) reused case (3)'s schedule, so their tick 101 also woke 4 ms late; **probe 3b** separates
>   the two (§0.4). The stall's true cost under the hold is TWO of the Localizer's ticks off, not one — corrected in D1 (a).
> - "the six record producers" → **five** on the robot (§0.3 rows 4–8; the sixth, the host test plant's, reads a FakeImu).
> - Found: the defect was first recorded at DOCS2 as **A17** and DEFERRED to R4 (§0.2's last bullet) — IMU1 closes it.
> - Added: **T3 (b)** and **M11** — a held read must not extend the hold (otherwise a reader that asks every 4 ms
>   freezes the rate for ever, and nothing in T1–T11 sees it). Part B's phase measurement re-shaped (a 1 ms window
>   sees a 10 ms copy on only one tick in ten).

---

## Re-check — amendments

*(2026-10-22. Independent checkers (a "tree" lens and a "break" lens) returned 18 findings; each was re-verified by
the brief-writer — re-opened, and re-probed where it matters, every binary under `ulimit -v 4194304` + `timeout 300`,
new probes and outputs under `.claude/coordinator/scratch/brief-IMU1/revise/` — then fixed at the root in the body, or
recorded as not taken with the evidence. R1–R10 are the tree lens's, R11–R18 the break lens's; overlaps are merged
and said so. The checkers' own probes are under `scratch/brief-IMU1/check-treefreeze/` and `check-correctness/v2/`.)*

- **R1 (major) — the fingerprint as written fails: CONFIRMED, fixed (§0.4 probe 4, §2.4, §7.4, landmine 12, §9).**
  Re-run here on the checker's two builds (`check-treefreeze/build-{base,fix}`: `71b4764` and the §1 adapter + a
  5-case test TU + one comment line in `motion_tank_steer_test.cpp`). `ProsCharSink` writes serial with ONE raw
  `write()` on fd 1 (`hal/pros/char_sink.hpp` :7-21, opened), while doctest's stdout is block-buffered into a pipe,
  so where the two interleave depends on every byte printed before — the checker's 32 differing cases, not 2. With
  `stdbuf -oL` and the old tool, 6 differ; the two extra are attribution, not behaviour: doctest prints a case's
  `path:line:` header just BEFORE its `TEST CASE:` line, so it is hashed into the PREVIOUS case (the case before a
  new test file in file order — here `HealthMonitor: two brownout…`), and the `[doctest]` summary lands in the
  run's LAST case. Line numbers were hashed too, so §1.4's comment edits in `motion_tank_steer_test.cpp` would move
  12 TANK1 cases. Two more effects found HERE that the checker's tool also misses: (i) `DebugRecord: tankSteerActive
  … LAST field` prints stack addresses as DECIMAL integers, and they move with the size of the environment block
  even under `setarch -R` (`140737488344464` vs `…400` with one extra environment variable — `revise/`, two runs of
  the same binary); (ii) in 1 of 6 runs of the SAME base binary, `ProsDeviceProvider 1r` and `ProsFileStore: the
  whole SAFE SAVE…` differed (byte counts 21217→20758 and 1517→1622; five other runs identical — `revise/full-*.txt`);
  the cause is not established — both write under FIXED host temp paths (`fs::temp_directory_path() /
  "shulib_safesave_filestore"`, the provider test's `copyRoot()`), which any concurrent suite run on the laptop
  shares. **The fix:** a new tool, `scratch/brief-IMU1/tools/case_hashes_v2.py` (written and run here), masks hex
  addresses, decimal numbers of 12+ digits, every `.cpp/.hpp/.h:<line>` and the tree's path prefix (argv[1]), and
  drops the `path:line:` header lines and the `[doctest]` summary; the run is `setarch -R stdbuf -oL`. Measured with
  it: base vs fix differ in EXACTLY 4 cases — S6, the two P3-T3 source scans (`drive_program_source_test.cpp` :189
  and :228 `CAPTURE` every test file they scan, so a new test file changes them legitimately) and the adapter test's
  gyro case; base vs base (environment padded) differs only in S6 plus the intermittent pair above. **The rule now:**
  the base binary is run TWICE; a case that differs between the two base runs is NOISE (named in the log, excluded);
  base vs Part A must differ only in the named set ∪ the noise; a case outside both is re-run once on its own and,
  if it still differs, investigated. Never run beside another suite run (the fixed temp paths). Probe 4's claim was
  measured on two runs of the SAME binary only — said so in §0.4.
- **R2 (major) — the reproduction-first run cannot compile, because T3's pin names a constant today's adapter lacks:
  CONFIRMED, fixed (§1 item 4, T3, M1, §2.5, §7.2).** `ceb17c0`'s `ProsImu` has no hold constant (opened, :70-232),
  so one test naming it fails the whole TU and no red count can be logged. **Fix:** the pin moves to its OWN TU,
  `test/imu_rate_hold_pin_test.cpp`, written in the same step as the adapter change (it cannot exist before the
  constant does); `test/CMakeLists.txt` :79 GLOBs `*_test.cpp`, so the reproduction build is the tree WITHOUT that
  one file. Every other test (R0, T1–T11, T3's edge included) names no new symbol — T3's edge is written in literal
  microseconds — and is built and run against `ceb17c0`'s UNCHANGED `hal/pros/imu.hpp`. T3 (a) is stated RED on
  today's code (today re-samples at +4 999 µs — the checker is right that the old "TODAY: green" was wrong for it).
  M1 is now "the hold disabled, the constant kept" (so the pin TU still builds); §7.2's scratch run removes the pin
  TU and puts `ceb17c0`'s adapter back. Found while fixing it: §2.5 and §7.2 listed only "T4 (gyro half)" as red
  today, but T4's rotation half is red too (today every one of its 400 reads is a device read, not 100) — the lists
  now say T4, as M1's row already did.
- **R3 (minor) + R13 (a) (major) — merged: T3's exact 5 ms edge depends on floating-point rounding at the anchor:
  CONFIRMED, fixed at the root (§1 item 1, T3, M2, landmine 13).** Re-probed (`revise/`, Python, the same IEEE
  doubles): `(t + 0.005) − t < 0.005` at 704 of 1000 FakeClock-like anchors (k × 0.01 s), exactly equal at ONE (t =
  0); on µs anchors (`micros` × 1e-6) 1206 below, 1 equal, 799 above. So in double seconds a read exactly 5 ms
  later is held or re-sampled by rounding, `<` and `<=` differ only where the subtraction is exact, and M2 is
  undetectable at an arbitrary anchor. **Fix at the root, not in the test:** the hold is defined in WHOLE
  MICROSECONDS — the robot clock's own resolution (`micros()`, HA-101). The constant becomes
  `static constexpr std::uint32_t kRateHoldUs = 5000;` and the check `std::llround((now − rateAnsweredAt_).value() ×
  1e6) < kRateHoldUs`. Re-probed: `llround` recovers 4 999 / 5 000 / 5 001 µs exactly at every anchor up to 20 s
  (0 misses of 6 018). T3 (a) then tests +4 999 µs held and +5 000 µs re-sampled at three anchors, one of them not
  round, and M2 (`<` → `<=`) turns the +5 000 µs case red. The pin compares integers: `kRateHoldUs × 2 == kTickMs ×
  1000`. On the robot this is a 1 µs matter (the checker's word, and right); on the host it is what makes the edge
  testable.
- **R4 (minor) + R17 (minor) — merged: T7's "bit-identical to the oracle" holds only on a clock that stands still
  during a read, and M10 can survive T7: CONFIRMED, fixed (T7, M10's row, §2.4 item 3, §9).** The fix reads the
  clock BEFORE the device (§1.1, landmine 4) where `ceb17c0` reads it after (:180 then :185), so on a clock that
  creeps per device read each sample is stamped one `gap` earlier: the interval is mathematically the same but rounds
  differently. Re-probed (`revise/`, IEEE doubles, 400 ticks with ±0.3 ms jitter): the rate differs in its last bits
  on 3 of 399 ticks at 25 µs and 4 of 399 at 100 µs (the checkers: 2 of 400, 3 of 299). Not a defect — on the robot
  the stamp moves by the device read's latency, microseconds — but T7 as written would be red on a correct fix.
  **Fix:** T7 runs on a clock that does NOT move inside a read (the shim's µs clock stepped only between ticks, or
  `FakeClock`) — never the model's creeping clock — and §2.4/§9 now say "bit-identical on a clock that stands still
  during the device read". M10 (the first call answers a rate): a mutant that differentiates from a zero baseline
  answers 0 when the first call sees rotation 0, or comes at time 0 (then `dt <= 0` answers `lastRate_`, 0) — T7's
  FIRST call is now made at a non-zero time with a non-zero rotation (t = 0.37 s, 12.5° CW), so M10 answers a rate
  there and goes red.
- **R5 (minor) — §3.1 item 3 omits the generated API indexes a new constant makes stale: CONFIRMED, fixed (§3.1
  item 3).** `docs/api/all-entities.md` lists every public member, fields included (opened: :3438
  `ProsTickPacer::kTickMs | field`), so `kRateHoldUs` adds a row there and moves `docs/api/README.md`'s index; the
  checker's `check-fresh` on a prototype named all three pages, and `shulib_tests` depends on `doc_gates`, so the
  suite does not even build until they are regenerated. §3.1 item 3 now names `pros-imu.md`, `debug_record.md`,
  `README.md` and `all-entities.md`; they are the same two index files MECH-A3, CAL1-A2a and VIS1-A1 regenerate, so
  §6's "whichever merges second regenerates `docs/api/` from a fresh build" covers them.
- **R6 (minor) — register line numbers labelled `ceb17c0` were mostly `71b4764`'s, and the stated shift was wrong:
  CONFIRMED, fixed (§3 header note, §3.1 item 2, §3.2).** `git diff -U0 ceb17c0 71b4764` re-run: the register moved
  +2 after :150, +2 more after :2854 and +1 more after :2901 (not "+1 after ~:3200"); `REMAINING-WORK.md` +2 after
  :23, +1 after :89, +4 after :167; `HANDOFF-remaining-work.md` +2 after :45; `docs/changelog.md` +3 after :206;
  `build-order.md` +36 by its deviations table (hunks from :1660 on; the IMU1 section :1190-1196 is unmoved). Opened
  at both commits: HA-04 :422 / :424, HA-109 :865 / :867, HA-179's "The IMU read:" :2486 / :2488, HA-22 :3241 /
  :3246, the build-order deviations row :2192 / :2228, HANDOFF's "TANK1 immune" :594 / :596 (and :598 in the main
  checkout's uncommitted edits). Every register, build-order and HANDOFF citation in §3 is now marked with the
  commit it belongs to; the executor re-anchors by text anyway.
- **R7 (minor) — small citation and count errors: CONFIRMED, fixed (§0.3, §1 item 4, §2.4 item 1, landmine 11,
  §9).** Opened at `ceb17c0`: the shim's `#error` gate is `test/pros_shim/pros/imu.hpp` :29-32 (not :31-34) and
  `ImuPortState` :51-60 (not :51-61). The grep §2.4 quoted returns 11 files, not 12, and two of them
  (`guide_examples`, `robot_builder`) only NAME the provider; the compiler's dependency files of the checker's full
  build (`check-treefreeze/build-base`, `*.o.d`, re-read here) give exactly NINE test TUs whose include closure holds
  `hal/pros/imu.hpp` — §2.4 item 1 now names them, and says they are the only TUs whose cases could legitimately
  move. "`teleop/` reads only the heading" was loose: no teleop header reads an `IImu` at all —
  `teleop/field_centric.hpp` :36-39 takes the heading as an ARGUMENT, and `teleop/teleop_loop.hpp` :32 / :345 samples
  it once per tick through a `HeadingSource` it is handed (:256-261).
- **R8 (minor) — T11's stated failure mechanism did not match what it asserted: CONFIRMED, fixed (T11).** Re-read at
  `ceb17c0`: `DrivetrainCalibrator::tick()` reads the heading (`accumulateYaw()` :1673-1677, called at :679), then the
  phase's rest window (`tickSideAssign` :885 / `tickSpinTest` :934 → `tickRest` :1018-1019 → `atRest()` :1477 — the
  tick's FIRST rate read, made only while every motor is still), then `updateStatus()` (:688 → :1740 → `atRest()` —
  the LAST read), and `status().atRest` is the last read's. On today's adapter with the copy between the two reads,
  `tickRest`'s read is 0 — the defect: rest is declared and the window completes while the robot turns — but
  `updateStatus`'s read is a spike, so `status().atRest` is FALSE and the old assertion was GREEN in-gap; it went red
  only for a far phase, where the last read is 0. **Fix:** T11 asserts the OUTCOME as well as the status: while the
  robot turns at 3°/s (over `restYawRateFloor`'s 2°/s, :287) with every motor still, the rest window never completes —
  no pulse starts (every motor's commanded voltage stays 0) until `restTimeout` (:293) refuses with `RestTimeout`
  saying the robot is still rotating (:219) — AND `status().atRest` is false on every tick. TODAY: red at EVERY φ off
  the read instants — in-gap through the outcome (the window completes after `restSettleTime` and a pulse starts),
  far through the status (the last read is 0). φ is kept strictly off every read instant (HA-214's band — R12).
- **R9 (minor) — §6 left out shared documents and test files of CAL1-A2a and VIS1-A1: CONFIRMED, fixed (§6, landmine
  10).** Re-read both worktrees' `git status` (read only). `vis1-a1` also modifies `docs/changelog.md`,
  `docs/hardware-assumptions.md` (it registers HA-192..HA-203 — the Held line and count line IMU1 edits),
  `HANDOFF-remaining-work.md`, `PROJECT-BRIEFING.md`, `docs/roadmap.md`, `docs/api/README.md` + `all-entities.md`,
  `docs/guide/14-what-it-cannot-do-yet.md`, `mkdocs.yml`, and the tests `pros_adapter_fence_test.cpp` (+1 line at :14
  — landmine 10's IMU block moves from :161-175 to :162-176 — and +10 after :192), `xdrive_config_test.cpp`,
  `mechanism_devices_test.cpp` (both among §2.4's nine fingerprint TUs) and `robot_builder_test.cpp`. `cal1-a2a`
  also modifies `docs/changelog.md`, `REMAINING-WORK.md`, `build-order.md`, `HANDOFF-remaining-work.md`,
  `docs/roadmap.md`, `README.md`, `ORIENTATION.md`. No CODE file IMU1 edits is touched by any of the three worktrees
  (confirmed, as the checker found). The "whichever merges second rebases the documents" rule now covers the CAL1-A2a
  and VIS1-A1 rows with their files named, and the fingerprint's base is the merge base Part A starts from — if
  VIS1-A1 (whose test edits are inside the fingerprint's set) lands after Part A is built, the fingerprint is re-run
  against the new merge base before IMU1 merges.
- **R10 (minor) — Part B's worksheet home and its bench includes cut against existing structure: CONFIRMED, fixed (D3
  (a), §1 items 5–6, §3.3, §3.4, §4 HA-215, §6, §7 Part B, §9).** Re-read: worksheet Station 4 is "the IMU, live on
  the brain screen … Devices shows it live, so this needs no code" (`R3a-BENCH-WORKSHEET.md` :231-234), and Bench
  Tests buttons 11 and 12 each have their own station (H :1040, I :1121) — so steps that upload and run code do not
  belong in Station 4. `bench_r3a.cpp` includes neither `hal/pros/tick_pacer.hpp` (which pulls
  `motion/motion_scheduler.hpp`, its :39) nor any `localization/` header (its includes, :74-117), and STAGE 5 paces
  with `pros::Task::delay_until(&prev, 10)` directly (:1863-1870). **Fix:** Part B's worksheet home is a NEW
  **Station N — THE IMU'S TIMING** (Bench Tests button 2's STAGE 2c), placed after Station I with the other Bench
  Tests stations (the letter N is free: no `Station N` in the worksheet, the briefs or `docs/internal/`, grep
  2026-10-22); Station 4 gains one pointer line and keeps its premise. STAGE 2c paces with `delay_until(&prev, 10)`
  as STAGE 5 does, adds NO include beyond those at :74-117 (it reads the C API `pros/imu.h` and `pros/motors.h`
  already included), and the tick span's body is the no-Localizer stand-in BY PLAN, not as a fallback: the device
  reads one motion tick makes between its two IMU reads (each drive motor's position, velocity, current and voltage,
  the battery) plus one serial line through the stage's one-call path — a LOWER bound on the real span (no Localizer
  arithmetic, no record build, no motor command), said so on the line it prints.
- **R11 (major) — T8 (a) "records on == records off, bit-identical" is RED on the fixed adapter for one band of phases:
  CONFIRMED by a re-probe, fixed (T8, §2.4 item 3, §9).** `revise/t8grid.cpp` (the checker's rig, the µs-hold
  prototype `revise/fixus/`, φ ∈ {0.5, 1.5, 2.25, 2.5, 2.75, 3.25, 3.5, 3.75, 4.5, 20, 200}·gap, kA 0 and 0.02, gap
  25 and 100 µs; `out/t8grid_{today,fixus}.txt`): on the fix, the 40 in is identical at every φ; the ACCEPTANCE run
  differs at exactly φ ∈ {3.25, 3.5, 3.75}·gap — from the face→driveTo boundary on (truth on 121 of 522 ticks at kA
  0.02, 157 of 605 at kA 0) — and is identical at (2,3)·gap and far. Mechanism, as the checker said: at a
  chained-motion boundary the scheduler runs a second `update()` in the same tick, whose HEADING read is a device
  read; with records on it is the 4th device read of the tick (the record's heading read sits before it), with
  records off the 3rd, and a copy in [3,4)·gap falls between them. `heading()` is deliberately NOT held (landmine 9),
  so the later read sees the newer copy — physical, and it would happen on a robot with any extra device read in
  between, not a defect of the hold. The brief's "in-gap" was (2,4)·gap and probe 2 itself used 3.0/3.5·gap; its
  "= rec=0" compared END and time at print precision, which hid it. **Fix:** T8's in-gap φ is now (2,3)·gap
  (2.25, 2.5, 2.75·gap: today RED there — the Localizer's ω is 0 on 150–152 of the 40 in's turning ticks at kA 0 and
  the run TIMES OUT at kA 0.02; the fix bit-identical) plus far φ (20·gap, 4.5 ms; today red: the Localizer reads
  ~3 % high, so ω differs every tick). A new **T8 (a′)** at φ ∈ {3.25, 3.5, 3.75}·gap asserts what the hold DOES
  promise there: on vs off identical up to the first chained-motion boundary; after it every rate read in a tick
  equals the tick's first (T8 (b)) and both runs finish (T8 (e)). §2.4 item 3 and §9 name the exception.
- **R12 (major) — T8 (c) is RED on the fixed adapter at kA 0: the third tick's ω is exactly 0 while the truth
  turned: CONFIRMED by the same re-probe, fixed (T8 (c)).** On the µs-hold prototype at kA 0, tick index 2 reads ω =
  0 on a turning tick at every φ from 2.25·gap up (40 in and acceptance run, both gaps), records on AND off alike
  (T8 (a) holds there, so the extra reader is not the cause); at kA 0.02 no such tick. Cause: the 10 ms copy lags
  the motion's start — the copy the third tick's first read sees was taken before the plant began to turn (the
  model's `thetaAt` holds the pre-step snapshot until the next copy). It is HA-213's sample-and-hold, not IMU1's
  defect, and not a regression. **Fix:** T8 (c) exempts the first THREE ticks of each motion (each acceptance-run
  leg's first tick found from the records' `activeCommandId`), and its φ grid stays STRICTLY inside the open
  intervals, never on a read instant k·gap (on a read instant the residual of HA-214 gives 0 / double — the
  checker's older loop saw one such zero at tick 10, φ = 2·gap). TODAY: red in-gap (150–152 zero turning ticks of
  the 40 in at kA 0; 485–486 at kA 0.02).
- **R13 (major) — T3 could not pass on a correct fix as worded. (a) the exact 5 ms edge: MERGED into R3 above (fixed
  at the root — the hold in whole microseconds). (b) "no value repeated more than twice running": CONFIRMED, fixed
  (T3 (b)).** Reasoned and matching the checker's run (`check-correctness/v2/out/t3_fix.txt`: a run of 22 equal
  values on the correct fix under a continuous model, 4 under sample-and-hold; today a run of 15): a CONSTANT turn
  makes every fresh answer the same number, so the criterion fails a correct hold, and it is red on today's adapter
  too — the brief's "TODAY: green" was wrong for it. The device-read criterion (a fresh device read on every second
  read at the latest) is the one that separates: green on the fix (longest held run 1), red on M11 (a run of 50,
  `t3_m11.txt`), green today. T3 (b) now asserts only that, plus "every held read equals the latest fresh answer".
- **R14 (major) — T10 is RED on the fixed adapter across its own φ band: CONFIRMED by a re-probe, fixed (T10, M1's
  row unchanged, HA-22's amendment in §3.1 and §4).** Re-run here on the µs-hold prototype (`revise/rest_t10.cpp`, the
  checker's rig: the real `RestCapture` over the real `ProsImu`, 1°/s drift, creep per device read;
  `out/rest_t10_{today,fixus}.txt`): at gap 25 µs the fix's `yawRateSigma` is 2.242e-4 for φ ∈ (1,2)·gap against a
  clean 5.618e-7 at every other φ (0.25, 0.75, 2.25…3.75·gap) — 399×; at 100 µs 2.208e-4 against 2.231e-6. Today:
  σ 4.483e-2 in (1,3)·gap and 2.253e-4 elsewhere. Mechanism (re-opened, `rest_capture.hpp` :386-388, :417-426,
  :428-456): the settle tick's first rate read is device read 1 (:388, after no heading read); every capture tick
  reads `heading()` first (:436), so its rate read (:456) is device read 2. On the fix the capture-start tick's :456
  read is HELD (the settle read answered it), so the first FRESH capture read is the next tick's, at the 2nd device
  read: that interval spans 10 ms + gap and holds TWO copies when one lands in (1,2)·gap — one sample ×2. This is
  R15's mechanism (the tick's first read changes position between code paths); the fix cuts the effect 200× but not
  to clean. **Fix:** T10 (a) places φ in (2,3)·gap (2.25, 2.75·gap) and far (5 ms), says why (1,2)·gap is excluded,
  and bounds `yawRateSigma` ABSOLUTELY — under 1e-5 rad/s, about 1/20 of what one doubled sample adds — with the
  mean within 0.1 % (fix: 100.00 %; today 103.3 % in-band); today is red at EVERY φ under that bound (2.25e-4 even
  far). T10 (b) pins the residual at φ ∈ (1,2)·gap: exactly ONE capture sample (the first fresh one) is about twice
  the drift rate, every other sample is the drift rate, σ ≤ 3e-4 (today 4.5e-2: red). HA-22's amendment records that
  a REST capture on an IMU1 build can still carry this one doubled sample in a narrow band of phases.
- **R15 (major) — the hold keys a tick's answer to whichever read comes FIRST, and that read's position changes
  between code paths, giving a 0 / ×2 pair at each change; HA-214 named only jitter, T9's "fresh fix every tick"
  hid it, and D1 did not credit (b) with avoiding it: CONFIRMED by a re-probe, fixed (D1 (a) and (b), T9, HA-214,
  §5).** Re-opened: `GpsCorrector`'s yaw-rate gate (6) is reached only on a FRESH fix (`gps_corrector.hpp` :248-268 —
  gate (4) declines a stale one first), the Localizer measures ~20 Hz fresh against the 100 Hz loop
  (`localizer.hpp` :302-303), and `propose()` runs before the Localizer's own read (:314 before :364). Re-run here on
  the µs-hold prototype (`revise/shift_first.cpp`, the checker's rig; `out/shift_first_{today,fixus}.txt`): fresh fix
  every tick (T9's old fixture) — 300 of 300 exact; fresh every 5th tick with the copy between the gate's position
  and the Localizer's (φ 0.13 ms) — 180 exact, 60 zero, 60 double, the gate and the Localizer agreeing on every tick;
  today — 0 exact (300 "other", or 60 zero and 240 other), the gate disagreeing on every fresh tick. Not a regression,
  but a residual T9 could not see — and the SAME mechanism is R14's REST residual (settle read first, then capture
  read after `heading()`). No robot is corrected this season (§6), so its robot impact today is the REST start and
  the idle↔motion transitions only, each one pair of ticks, and only in a band one device read wide. **Fix:** HA-214's
  claim, D1 (a)'s cost and `kRateHoldUs`'s doc now name it (limit 2); T9 gains (b), a realistic cadence (fresh every
  5th tick, φ in the band) asserting the gate equals the Localizer on every fresh tick and RECORDING the residual
  counts, never asserting exactness; D1 (b) is credited — one read at a fixed point at the top of `update()` keeps
  the position constant for the Localizer, the correctors and the records. The cure that keeps (a) — the Localizer
  reading its rate at the top of `update()`, ahead of the correctors (a one-line move; byte-identical on the host,
  where the value is the same) — is named in §5 as OUT for now: no corrected robot this season, and
  `localizer.hpp` is L6-A1's file. The recommendation stays (a): what it fixes is every tick; what it leaves is a
  pair of ticks at a change.
- **R16 (major) — the hold's limits were understated: HA-215's "never for the Localizer's own first read" is false,
  an off-tick reader costs TWO Localizer ticks under the hold (one today), and the changelog's "however often it
  asks" overclaims: CONFIRMED by a re-probe, fixed (D1 (a), the plain words, §1.1, the changelog, HA-215, §5,
  §9).** Re-run here (`revise/span_reader.cpp`, `revise/fastloop.cpp`, the checker's rigs, on the µs-hold prototype;
  `out/{span_reader,fastloop}_{today,fixus}.txt`), the Localizer reading at +0.1 ms: a later reader at a FIXED +4,
  +6 or +7.5 ms every tick leaves the Localizer's read exact (300/300 at every φ — above 5 ms the Localizer then
  answers the previous tick's late read, stale by 10 ms − span: exact for the model's constant turn, late in
  general); a later reader at a RANDOM +4..+7 ms (a span jittering around 5 ms) leaves only 55 of 300 exact (34–72
  zero, 173–211 off) — so "never for the Localizer's own first read" was false; an out-of-loop read at +6.5 ms on 1
  tick in 10 costs 60 Localizer ticks of 300 (today: 30) — the next tick held from it (stale), the one after skewed;
  a team's own loop reading once per loop gets 0 on 1 490 of 2 980 reads at 1 ms, 596 zero and no exact read at
  4 ms, no exact read at 5.2 or 7 ms, exact only at 10 ms (today is as bad or worse at every period but 10 ms).
  **Fix:** a third limit in D1 (a), HA-215 and `kRateHoldUs`'s doc — a read from OUTSIDE a library tick (team code
  between routine steps, a screen task, any reader on its own rhythm) 5–10 ms after a tick's first read makes the
  next tick's first read held (stale) and the one after skewed — the one case where the hold does worse than
  today; HA-215's blast radius rewritten (a span JITTERING around 5 ms damages the Localizer's own first read on
  most ticks; a span steadily over 5 ms makes it one partial tick late); the changelog says "within one library
  tick, every read gets the library's value", and that a loop at any period other than a multiple of 10 ms gets a
  rate that is 0 on a large share of reads under HA-213 — differentiate the heading over ≥ 10 ms, or use the gyro
  once HA-04 and HA-109 are settled. Recommendation unchanged: the off-tick case is a team's choice the library can
  document; the in-tick case was the library's own every-tick defect.
- **R17 (minor) — T7's clock convention and M10's zero-rotation first call: MERGED into R4 above** (confirmed;
  fixed in T7 and M10's row).
- **R18 (minor) — Part B's gyro check was self-contradictory and its cadence unpinned: CONFIRMED, fixed (D3 (a), §1
  item 5, Station N's text in §3.3, landmine 15).** `ProsImu::yawRate()` returns the CANONICAL rate, CCW-positive
  rad/s (`hal/pros/imu.hpp` :201, `imuYawRateToCanonical`), so "ProsImu's differentiated rate (both CW deg/s raw)" was
  wrong as written, and "same sign = CW-positive" would invert HA-04's verdict if the stage compared against
  `ProsImu::yawRate()` without un-negating — the very sign D1 (d) waits on, and a wrong one drives TANK1's observer
  the wrong way (HA-179). Under Part A's hold, `ProsImu::yawRate()` read in a tight loop also refreshes at most every
  5 ms, which under HA-213's 10 ms copy alternates 0 and about double (the checker's `fast_fix.txt`; R16) — failing
  HA-109's "within ~10 %" for the wrong reason. **Fix, pinned:** the stage never calls `ProsImu::yawRate()`. Over the
  hand turn it compares ANGLES, both raw CW degrees from the C API: Δ`imu_get_rotation()` between the FIRST and LAST
  change instants its tight loop sees (≥ 2 s apart, so the 10 ms copy's quantization is under 0.5 %), beside ∫
  `imu_get_gyro_rate().z` dt over the same window (trapezoid on `micros()` stamps, z sampled every poll). Same sign ⇒
  z shares `get_rotation()`'s convention, so `GyroRateZ` and the differentiated rate agree in sign through
  `ProsImu`'s one negate — HA-04's question (whether that convention is CW is HA-02's, Station 4.2, unchanged); a
  ratio within ~10 % of 1 ⇒ HA-109's deg/s holds. Each printed line names its convention ("raw deg, PROS's
  rotation — CW-positive per HA-02"; "raw z, integrated").

**Net effect of R1–R18.** All 18 confirmed (two pairs merged: R3 with R13 (a), R4 with R17); none rejected. **No
decision changes:** D1 (a) stays recommended — its cost now names THREE limits (a stall or late wake; the tick's
first read changing position; a reader outside the tick, the one case where the hold does worse than today), and
(b) is credited with avoiding the second for the Localizer, the correctors and the records — because what (a)
fixes is the Localizer's rate on EVERY tick and what it leaves is a tick or two per cause. D2 (a), D3 (a) (its
stage re-pinned: `delay_until` pacing, a stand-in tick body by plan, angles not rates for the gyro, a new Station N)
and D4 (a) unchanged. **One design change at the root:** the hold is `kRateHoldUs = 5000`, compared in whole
microseconds (R3/R13). **No new hardware belief:** R14–R16 widen HA-214's and HA-215's claims and blast radii;
HA-213..HA-215 stay three (the L6 collision note stands). **Size:** Part A ~1 → ~1¼ builder-days (the pin TU, T8 (a′),
T9 (b), T10 (b), T11's outcome assertion); Part B ~½; both ~1¾.

---

## DECISIONS FOR THE TEAM LEAD

**D1 — WHERE the tick's one rate is read, and how every reader gets it. RECOMMENDED (a).**

- **(a) RECOMMENDED — the robot's IMU adapter answers once per tick.** `ProsImu::yawRate()` (the code behind every
  robot's IMU, `include/shulib/hal/pros/imu.hpp`) keeps its answer for 5 ms: a read less than 5 ms after the last
  answered read returns that answer and touches neither the sensor nor the differentiation sample. The 10 ms control
  tick (`ProsTickPacer::kTickMs`, `hal/pros/tick_pacer.hpp` :51) puts every read of one tick inside 5 ms of its first,
  and the next tick's first read 10 ms later — outside it. **One file of library code changes**; every reader is fixed
  at once — the Localizer, the five record producers, the two correctors' gates, the three calibration classes, and a
  team's own `imu.yawRate()` in their auton; no frozen signature moves (adapters are implementations, not contracts —
  the Freeze Register's F13 row, `docs/roadmap.md` :147); no consumer is edited; the host suite's FakeImu paths cannot
  see it. **Cost:** the adapter infers "the same tick" from TIME, not from the loop. **Three honest limits**, all
  measured on the host (probes 3 and 3b; re-check R15/R16): **(1) a stall or a late wake** — a tick that STALLS more
  than 5 ms between two of its reads gives its later readers a fresh answer, the next tick's first read is held from
  that answer, and the tick after reads low — two of the Localizer's ticks off, then exact (probe 3b (4a)); and a tick
  that wakes 5 ms or more LATE (another task held the processor) is followed by an on-time wake under 5 ms later,
  because `delay_until` keeps the 10 ms grid (`hal/pros/tick_pacer.hpp` :7-14) — that tick gets the late tick's
  answer, one tick stale (probe 3b (3c): three ticks off where a late read alone costs two). **(2) the FIRST read
  moves** — the tick's first read keys the answer, so when its position changes between ticks and the IMU's copy lands
  between the two positions, the pair of ticks around the change reads 0 and about double: a GPS or tag corrector's
  gate reads first only on a fresh-fix tick (~1 in 5, `localizer.hpp` :302-303); Calibrate's REST capture reads the
  heading before its rate where its settle step did not (`rest_capture.hpp` :388, :436, :456). Host: fresh fix every
  5th tick, copy in that band — 180 exact, 60 zero, 60 double of 300 (today: 0 exact). **(3) a reader outside the
  tick** — a read from team code between routine steps, a screen task, or any reader on its own rhythm, 5–10 ms
  after a tick's first read, makes the next tick's first read held (stale) and the one after skewed; host: one such
  read on 1 tick in 10 → 60 of 300 Localizer ticks off, where today's adapter loses 30 — the one case where the hold
  does worse than today; and a span JITTERING around 5 ms leaves only 55 of 300 exact. (1) and (2) cost a pair of
  ticks per cause and do not repeat without it; today's adapter is off on every tick (HA-214, HA-215).
- **(b) A per-tick snapshot in the Localizer that every consumer reads.** The Localizer reads once (moved to the top
  of `update()`, ahead of the correctors) and publishes the value; the records read it; the corrector gates are handed
  it (a change to the corrector seam, `ICorrector::propose`); the calibration classes, which run without a Localizer,
  each keep their own one-read rule. **Cost:** ~10 call sites in 8 headers plus a seam change, the record column's
  meaning changes (the Localizer's screened value instead of the IMU's), the chained-motion boundary needs a rule of
  its own — and the trap stays armed in the adapter: any NEW reader, including a team's `imu.yawRate()` in an auton,
  gets 0 or a spike on the robot and a right answer on the host. **Credit (re-check R15):** one read at a fixed point
  at the top of `update()` keeps the first read's position constant for the Localizer, the correctors and the
  records, so (b) has no limit (2) for them (the calibration classes keep it), and a reader outside the tick costs
  today's one tick, not (a)'s two.
- **(c) A tick counter the pacer bumps and the adapter keys on.** Exact "tick" semantics, immune to stalls.
  **Cost:** a new seam from the provider into every loop that paces (the motion scheduler, the driver loop,
  Calibrate's loop, the bench program, the competition program's two tasks), and a loop that forgets to bump it
  FREEZES the rate for the rest of the boot — a frozen reading, which is worse than today's failure, and invisible on
  the host unless every loop is tested for it.
- **(d) Switch the default rate source to the gyro (`YawRateSource::GyroRateZ`).** The gyro read keeps no per-call
  state, so every read in a tick returns the same buffered value, and it also removes the residual that (a) cannot
  (HA-214). **Cost:** its SIGN is invented (HA-04) and its UNITS are invented (HA-109), and since TANK1 a wrong sign
  makes the tank yaw observer push the WRONG way, up to its 1.5 rad/s limit (HA-179's blast radius: 16° of wobble and
  a timeout, measured on the host). It cannot be the default before the bench measures the sign. D3 (a) measures it;
  a later, separate change can then make it the default (§5).

**D2 — Does the frozen `IImu` interface's text gain the rule "every read within one control tick answers one value"?
RECOMMENDED (a).**

- **(a) RECOMMENDED — no: `IImu` (F4, LOCKED 2026-06-19, `hal/imu.hpp` :39-40) is untouched.** The rule lives where
  the state lives — `ProsImu`'s banner and its `yawRate()` doc — and in the hardware register. After (a) of D1, every
  `IImu` in the tree answers one value per tick (FakeImu is stateless; `ProsImu` holds), so no consumer needs the
  sentence to be correct.
- **(b) Add it to `IImu::yawRate()`'s documented contract.** It would bind every FUTURE implementation (a host-plant
  IMU, a team's adapter) to the rule. **Cost:** it is a semantic tightening of a frozen row: the Freeze Register's
  rule is "changes only with a version bump and a migration path" (`docs/roadmap.md` :128-130) — an API-version note
  and the register's F4 row amended, for a sentence no current consumer needs.

**D3 — Measure the IMU on the robot: a short Bench Tests stage. RECOMMENDED (a).**

- **(a) RECOMMENDED — Part B: a new stage in the Bench Tests program** (`make ROBOT=tank`, `src/programs/bench_r3a.cpp`,
  beside STAGE 2's IMU readout at :735-) that, while someone turns the robot by hand: reads the raw rotation in a tight
  loop for 2 s with `micros()` timestamps and prints the instants it CHANGES (settles HA-213: is it copied every
  10 ms?); runs 100 ticks paced with `delay_until` like STAGE 5 and prints, per tick, where the copy lands relative to
  the wake (HA-214's phase — fixed or drifting); times a stand-in for a motion tick's body between its two IMU reads
  (HA-215 — a lower bound); and compares, over the same turn, the gyro's integrated z with the change in the raw
  rotation, both raw CW degrees (HA-04's sign, HA-109's units — the measurement that D1 (d) waits for; never
  against `ProsImu::yawRate()`, re-check R18). A new worksheet **Station N** (the Bench Tests stations' pattern, like H
  and I — re-check R10) says what to do and what each number means (§3.3). **Cost:** ~½ builder-day; one more src/
  change through the seven-build gate; robot time ~10 minutes.
- **(b) No robot step now.** HA-213..206 stay reasoned/invented until something on the robot shows the rate — and
  nothing does today (§0.2: no robot program prints the record's `imuYawRate`). **Cost:** IMU1's premise and its
  residual stay unmeasured; D1 (d) stays blocked on HA-04.

**D4 — IMU1 before Station M's floor run? RECOMMENDED (a).**

- **(a) RECOMMENDED — M.6 (the floor run on the tape, HA-179/HA-180's evidence) runs on an IMU1 build;** M.0–M.5 (the
  boot, the forward check, the aborts, the wheels-up run) need not wait. Station M is the first time the library
  drives robot two on its own, and its floor run is the evidence for TANK1's constants — whose yaw observer reads the
  very rate this chunk fixes. Probe 2: with the copy phase in the Localizer-to-record gap and TANK1's own motor-lag
  values, today's code TIMES OUT the 40 in (END heading +12.44° at kA 0.02, −17.35° at kA 0.03) where the fix
  FINISHES byte-for-byte as with one reader. Whether robot two's phase lands in that gap is unmeasured (HA-214): the
  band is narrow (the gap's width over 10 ms — about 0.5 % of boots for a 50 µs gap, more for a longer one; every boot
  if the phase drifts through it). **Cost:** M.6 waits ~1¼ builder-days plus the merge.
- **(b) Station M runs first, with a caveat in its record:** a TIMEOUT with TSTR lit and the correction pinned at its
  limit on a pre-IMU1 build is not evidence against HA-179. **Cost:** a failed M.6 cannot be told apart from
  HA-179/HA-180 failing without re-running it after IMU1.

---

## The design in plain words (for the team lead)

The IMU does not report how fast the robot is turning in the way the library uses it. The library works it out: it
reads the IMU's total rotation, compares it with the last reading, and divides by the time between the two. That only
works if the two readings are a tick apart. The robot's IMU code was written for ONE reading per tick ("Call it once
per tick", its own comment), but the library reads it two or three times every tick — the position estimate reads it,
then the line the robot prints about that tick reads it again, and on a robot with a GPS or tags the GPS check reads it
first.

On the robot, the second reading comes a fraction of a millisecond after the first. The IMU only refreshes its numbers
every 10 ms (PROS's own documentation says so), so the second reading usually sees no change and reports "not
turning" — or, if the refresh happened in between, reports a turn 30 times too fast. Worse, the second reading becomes
the new starting point, so the NEXT tick's first reading — the one that matters, the one TANK1's steering correction
uses — compares against the wrong moment. For most timings that costs about 3 %; for one narrow band of timings it
makes the first reading 0 on every tick, and TANK1's correction then fights the robot instead of helping. On the
computer, the library's test clock does not move inside a tick, so every test passed and nobody saw it.

The fix: the IMU code remembers its answer for 5 ms. Anything that asks again within the same tick gets the same
answer, and nothing it does moves the starting point. The next tick, 10 ms later, gets a fresh answer. One file
changes; nothing that uses the IMU changes; a team's own code that asks for the turn rate from inside a library tick
is fixed too. It works out "the same tick" from the clock, so it has limits (D1 (a)): code that asks for the turn rate
from OUTSIDE the library's loop — between auton steps, or in a loop of its own — spoils the next two ticks' readings
(one tick today); and when the first thing to ask in a tick changes (a GPS check that runs only when a new GPS fix
arrives, Calibrate starting its still-capture), the IMU's refresh can land between the two, giving one "no turn" and
one "double turn" reading. Both are a tick or two, not every tick.

The tests are written FIRST, and they must fail on today's code: they give the computer's copy of the PROS IMU a
clock that moves between two readings in a tick and an IMU that refreshes every 10 ms, the way the real one does —
which reproduces the robot's failure (probe 1 and 2 did exactly that). Then the fix goes in and they pass. Every other
test in the library must give byte-for-byte the same results (§2.4 says how that is checked).

What IMU1 does NOT fix: if the IMU's 10 ms refresh lands right next to the moment the loop reads it, one reading per
tick still sees "no turn" on some ticks and "double the turn" on the next (§0.4, probe 3). That needs the gyro's own
rate reading (D1 (d)), which first needs its direction measured on the robot (D3).

---

## §0 Why — what exists (every line cited was OPENED at `ceb17c0`) — the probes

### 0.1 The defect, re-derived from the code

- **The adapter differentiates between CALLS.** `ProsImu::yawRate()` (`include/shulib/hal/pros/imu.hpp` :134-137)
  dispatches to `differentiatedRate()` (:179-203) by default (`YawRateSource::DifferentiateRotation`, :78-81, :95). It
  reads `get_rotation()` (:180), then `clock_->now()` (:185), and divides the rotation since the LAST CALL by the time
  since the last call (:194-201); every call moves the sample (:199-200). Its own doc says "the interval is however
  long since you last asked rather than a fixed dt … Call it once per tick" (:128-132). A call at the SAME clock
  instant repeats the previous answer (:195-196) — the branch that hides the defect on the host (below).
- **The robot's clock moves inside a tick.** `ProsClock::now()` is `pros::micros()` × 1e-6 (`hal/pros/clock.hpp`
  :54-56; HA-101, settled 2026-08-13). Two reads in one tick are microseconds apart, so `dt > 0` and :195's guard
  never fires on the robot.
- **The IMU's numbers refresh every 10 ms.** The vendored PROS header: "values are copied into the shared memory
  buffer only at 10ms intervals, setting this value to less than 10ms does not mean that you can poll the sensor's
  values any faster" (`include/pros/imu.hpp` :153-156, `set_data_rate`'s doc). So `get_rotation()` holds one value
  for 10 ms (sample-and-hold) — **documented, never measured on our brains: HA-213.** Under it, a second read
  microseconds after the first returns EXACTLY 0 unless a copy lands between the two reads, and then a spike of one
  copy's worth of turn over the gap (×10 ms/gap: ×33 for a 0.3 ms gap).
- **The second read moves the baseline, so it corrupts the NEXT tick's first read.** After tick k's second read
  at `t_k + b`, tick k+1's first read at `t_{k+1} + a` differentiates over `10 ms − (b − a)`: with no copy between a
  and b it reads ~`(b−a)/10 ms` high (3 % at 0.3 ms); with a copy between a and b, the copy was consumed by tick k's
  second read and tick k+1's interval holds NO copy — **the first read is 0, every tick, for as long as the phase
  stays in that band.** This is the part TANK1's re-check did not model (§0.2).
- **Why the host never saw it.** `FakeClock` advances only when a test or a pacer steps it, so every read inside a
  host tick happens at one instant and :195-196 repeats the answer — exactly the hold IMU1 adds. The PROS shim's clock
  is the same (`test/pros_shim/pros/rtos.hpp` :36-66: `micros()` moves only in `delay`/`delay_until`/`advanceUs`), so
  even the suites that run the REAL `ProsImu` in a loop (`test/robot_two_driver_test.cpp` :2 and :203, through
  `ProsDeviceProvider`; `test/brake_stop_stack_test.cpp` :220-) see one answer per tick. FakeImu (`hal/fake/fake_imu.hpp`
  :17) is stateless. No host test could see a second read until TANK1's P14 built one by hand.

### 0.2 What TANK1's re-check found, what it missed, and what the plan says that is not true

The finding (TANK1-PROGRESS.md :132-140, :289, :571; TANK1-COMPLETED.md :67; TANK1's brief :18 and D4 at :237-257;
build-order.md :1190-1196; REMAINING-WORK.md :127 row 32): "every read after the first in a tick returns about 0 or a
spike", six readers, and "TANK1 is immune by construction" because its observer reads the Localizer's rate
(`move_to_pose.hpp` :283-292, `loc.twist().omega()`). Re-checked here:

- **Right:** the second read is ~0 or a spike (reasoned, §0.1; nobody has measured it — HA-213).
- **Missed — the next tick's first read.** P14's model (`test/motion_tank_steer_test.cpp` :1215-1250,
  `ProsShapedImu`) answers "the FIRST `yawRate()` of each clock instant … truly" and only falsifies the later calls.
  The real adapter's first read is the one damaged by the previous tick's second read (§0.1). P14's rig also has NO
  telemetry sink ("so no record", :1260-1261), so it never had a second reader per tick.
- **Missed — on robot two the second read happens EVERY motion tick.** `hal::emitRecord` builds a record only when the
  sink wants one (`hal/telemetry_sink.hpp` :142-146). Robot two's sink is the provider's `TermSink` over the USB serial
  (`hal/pros/device_provider.hpp` :548-558, :675-678; wired into the chassis at `robot/robot.hpp` :569), and
  `TermSink::wantsRecord()` is `true` (`diag/term_sink.hpp` :106) — a `LogCopySink` defers to it (`diag/log_copy_sink.hpp`
  :126). So every motion tick reads the rate in `Localizer::update()` (`localization/localizer.hpp` :364, inside
  `MotionScheduler::tickBody()` :1072) and again in the motion's record (`move_to_pose.hpp` :543, `turn_to.hpp` :238,
  `drive_brake.hpp` :210), and every driver-control tick in `Chassis::drive()` (:428) and its record (:618).
  TANK1's observer is therefore NOT immune on robot two: its rate is the corrupted first read.
- **Not true — "Station M's `imuYawRate` column".** No robot program prints the record's `imuYawRate`: `TermSink`'s tick
  line renders target, error, command, quality and flags (`diag/term_sink.hpp` :116-165) and no IMU rate; the SD copy
  takes no records ("every per-tick record to serial only", `device_provider.hpp` :551-552); only the blackbox format
  carries the field (`diag/blackbox_format.hpp` :497, :595), and no `src/` program installs a blackbox or the SHUL/2
  sink (`Shul2Sink` is H1 Part A2's, unbuilt — `diag/shul2_format.hpp` :7). REMAINING-WORK :35 (row 4), :160 and
  build-order :1192 promise a column Station M cannot capture. What IMU1 changes at Station M is the Localizer's own
  rate — TANK1's observer and `DriveBrake`'s exit (`drive_brake.hpp` :121-130) — which is D4.
- **Found before TANK1 — DOCS2's defect A17 (added 2026-10-22).** `docs/internal/chunks/DOCS2-API-DEFECTS.md`
  :835-854 (the claim at :837, the disposition at :839): "ProsImu::yawRate() in the DEFAULT DifferentiateRotation mode is a CONSUMING read that rebases the
  differentiation sample … attaching a telemetry sink changes the yaw rate the Localizer sees", disposed "DEFECTS1 →
  DEFER. R4. The consuming-read rebase needs measured call patterns and a real loop rate before a design can be
  chosen." A17 judged the effect "not biased, only noisier"; under PROS's documented 10 ms copy (HA-213) it is
  biased — 0 on every tick in the band (§0.1). IMU1 closes A17 (its disposition gains a dated pointer, §3), and Part
  B is the "measured call patterns and a real loop rate" its deferral asked for. Its sibling A18 (`faultedReads()`
  counts reads, up to four per tick; REJECTED) changes under IMU1: a held rate read is not a device read and does not
  count (§1.1), so a dead sensor counts one rate read per tick plus its heading reads.

### 0.3 Every reader of the IMU's rate (`grep -rn "yawRate()" include src test` at `ceb17c0`, each site opened)

| # | Site | When | Reads in one tick on the robot today |
|---|---|---|---|
| 1 | `localization/localizer.hpp` :364 — the Localizer's ω (`twist()`, :453) | every `update()` | the tick's first read, unless a corrector reads first (#2/#3); a SECOND `update()` at a chained-motion boundary (TANK1-PROGRESS :132-140) reads again |
| 2 | `localization/gps_corrector.hpp` :268 — gate (6), "spinning too fast" | inside `propose()`, STEP 3, before #1 | first read on the ticks a fresh fix reaches the gate |
| 3 | `localization/apriltag_corrector.hpp` :326 — gate (6) | same | same |
| 4 | `motion/motion_scheduler.hpp` :1258 — the idle record | between motions, sink wants records | second |
| 5 | `chassis/chassis.hpp` :618 — `drive()`'s record | every driver-control tick | second |
| 6 | `motion/move_to_pose.hpp` :543 — MoveToPose's record (strafe/hold/drive legs use the engine) | every motion tick | second |
| 7 | `motion/turn_to.hpp` :238 — TurnTo's record | every turn tick | second |
| 8 | `motion/drive_brake.hpp` :210 — DriveBrake's record | every brake tick | second |
| 9 | `calibration/rest_capture.hpp` :388 (settle gate), :411 (refusal text), :456 (the REST record's rate σ) | Calibrate's REST step | :388 then :456 on the tick the capture starts (`beginCapture` :417-426 calls `tickCapture` on the same tick) — the REST record's first rate sample is a second read |
| 10 | `calibration/drive_characterizer.hpp` :1098 (`imuStill`, from :614/:699), :1031 (refusal text) | Calibrate's GAINS step | first; the refusal text a second |
| 11 | `calibration/drivetrain_calibrator.hpp` :1477 (`atRest`) — from `tickRest` (:1018-1019), `updateStatus` (:1740, every `tick()` :688) and `pushReady` (:831, the panel) — and :1049 (refusal text) | Calibrate's motor watch / spin | two or three |
| 12 | `sim/drive_plant.hpp` :378 — the host test plant's own record | host only | reads the plant's `FakeImu&` (:756), never a `ProsImu` — untouched |

No `src/` file reads the rate. No `teleop/` header reads an `IImu` at all: `teleop/field_centric.hpp` :36-39 takes the
heading as an argument, and `teleop/teleop_loop.hpp` :32 / :345 samples it once per tick through a `HeadingSource` it is
handed (:256-261) — re-check R7. Tests:
`test/pros_imu_adapter_test.cpp` :53-149, `test/motion_tank_steer_test.cpp` :1215-1338, `test/fake_imu_test.cpp`,
`test/sim_*`.

### 0.4 The probes (`.claude/coordinator/scratch/brief-IMU1/`; every binary run under `ulimit -v 4194304` + `timeout 300`)

**Setup.** `tree/` is a copy of `ceb17c0`'s `include/`, `test/pros_shim/`, `test/motion_test_rig.hpp`
(`tree/SOURCE_COMMIT`). The shim COPY gains two members on `ImuPortState` — `rotationHook` (when set,
`get_rotation()` asks it) and `rotationReads` (device reads) — the change §1 proposes for the real shim. `fix/include/`
overlays ONE file, the prototype `hal/pros/imu.hpp` (a 5 ms hold, `kRateHoldSeconds` in double seconds — re-check R3
moved the brief's hold to whole microseconds, prototyped in `revise/fixus/` and used by the re-check's probes; the clock
read moved ahead of the device read; diff in `fix/`); each probe is built twice, `out/*_today` and `out/*_fix`. **The IMU model (HA-213):**
the buffer is copied every 10 ms at phase φ; `get_rotation()` returns the rotation AT the last copy. Time moves `gap`
per device read inside a tick. φ and gap are guesses, so both are swept.

**Probe 1 — the adapter alone** (`probes/probe_adapter.cpp`; `out/probe_adapter_{today,fix}.txt`). Constant 90°/s CW
(truth −1.570796 rad/s every tick); read A at +0.1 ms (the Localizer's), read B at +0.4 ms (the record's); 200 ticks
per row.

| φ | today, A (the Localizer) | today, B (the record) | fix, A and B |
|---|---|---|---|
| 0.05 ms (copy before A) | 200 "other" (3.1 % high: the interval is 9.7 ms) | 200 × exactly 0 | 200 exact, B == A |
| 0.2 / 0.3 ms (copy between A and B) | **200 × exactly 0** | 200 spikes, −52.36 rad/s (×33) | 200 exact, B == A |
| 1 / 5 / 9.9 ms | 200 "other" (3.1 % high) | 200 × exactly 0 | 200 exact, B == A |
| any φ, ONE read per tick | 200 exact | — | 200 exact |

**Probe 2 — the real loop** (`probes/probe_loop.cpp`; `out/probe_loop_{today,fix}.txt`). The REAL Chassis,
MotionScheduler, Localizer (PilonsOdometry on the plant's FakeImu, like P14's rig, `motion_tank_steer_test.cpp`
:1262-1300) and TANK1's MoveToPose; the `ProsImu` over the shim as BOTH the context's IMU and the Localizer's (as
robot two wires it, `device_provider.hpp` :318); the right side 9 % weak; "rec=1" = the context's sink wants records
(as robot two's does), "rec=0" = no sink. A tick reads the device four times (the Localizer's heading, its rate, the
record's heading, its rate), so the Localizer's rate read is at 2·gap and the record's at 4·gap; "in-gap" φ lies between.

| run (gap 25 µs) | today, rec=0 | today, rec=1, φ in-gap | today, rec=1, φ far | fix, rec=1 (every φ) |
|---|---|---|---|---|
| 40 in, instant motors | Settled 1.89 s, END y +0.02, 0.11° | Settled 1.88 s, END y **+0.15**, 0.20°; the Localizer's ω = 0 on **152/152** turning ticks; records: 178 of 189 spikes | Settled, ≈ rec=0; records: **190 of 190 zero** | **= rec=0** (END, time); records: 3 zeros (the first ticks — "the first call returns 0", unchanged) |
| 40 in, motor lag kA 0.02 | Settled 1.46 s, END y +0.19, +0.67° | **TIMEOUT 5.01 s, END y +0.71, +12.44°**; ω = 0 on 485/485 | Settled, ≈ rec=0 (not bit-equal — the Localizer reads ~3 % high; END within 0.01°) | **Settled 1.46 s, = rec=0** |
| 40 in, motor lag kA 0.03 | Settled 2.18 s, END y +0.31, −0.43° | **TIMEOUT 5.01 s, END y +1.00, −17.35°** | Settled, ≈ rec=0 (not bit-equal — the Localizer reads ~3 % high; END within 0.01°) | **Settled 2.18 s, = rec=0** |
| acceptance run, kA 0.02 | ok, 5.22 s | **ok = 0** (stopped on its first leg, +9.20°) | ok, ≈ rec=0 | **ok, 5.22 s, = rec=0** |
| acceptance run, kA 0.03 | ok, 6.69 s | **ok = 0** (−16.76°) | ok, ≈ rec=0 | **ok, 6.69 s, = rec=0** |

(kA 0.02 / 0.03 are TANK1's own motor-lag values, its P13 — TANK1-PROGRESS :125-126.) Also measured: today, with
NO sink, the acceptance run's Localizer reads 0 on 4 ticks where the fix reads 0 on 1 — the second `update()` at each
chained-motion boundary (TANK1-PROGRESS :132-140). The same table at gap 100 µs is in the output files and agrees.
**What it shows:** the defect is reproducible on the host with the real adapter, through every layer; the fix makes a
run WITH a record reader identical to a run without one; and the failure that matters is the Localizer's, not the
record's. *(Re-check R11/R12, re-probed 2026-10-22 per tick and bitwise — `revise/t8grid.cpp`: "= rec=0" above was
END and time at print precision. Bitwise, the fix is identical records on vs off for φ in (2,3)·gap and far, but NOT
in [3,4)·gap on the acceptance run — a chained-motion boundary's second `update()` reads the heading one device read
later with records on, and a copy there falls between; this table's "in-gap" rows used 3.0 and 3.5·gap. And at kA 0
the fix reads ω = 0 on the third tick, records on and off alike — the copy lags the motion's start. T8 is now
specified around both.)*

**Probe 3 — timing hostility** (`probes/probe_timing.cpp`; `out/probe_timing_{today,fix}.txt`; 300 ticks per row,
reads at +0.1 / +0.4 ms):

| case | today | fix |
|---|---|---|
| (1) loop jitter ±0.3 ms, φ 5 ms | B ≠ A on 300/300 | B == A on 300/300; A within ±6 % (the jitter over 10 ms — differentiating against the read clock, inherent) |
| (2) φ AT the read instant, jitter ±0.05 ms, two reads | A: 154 zero, 146 high | **A: 154 exact, 73 zero, 73 double** — the residual (HA-214) |
| (2′) same, ONE read per tick | 154 exact, 73 zero, 73 double | identical — IMU1 does not cure it |
| (3) one LATE WAKE (tick 101 wakes 4 ms late, 102 back on the 10 ms grid — `delay_until`, `tick_pacer.hpp` :7-14) | 299 high, 1 low | 298 exact; the two ticks around the late wake off |
| (4) a 6 ms stall between A and B on one tick — **but see probe 3b: this case also carried (3)'s late tick 101** | B ≠ A on 299 | **B ≠ A on 1** (the stall tick — the hold's documented limit, HA-215) |

**Probe 3b — the stall and the late wake, separated (2026-10-22)** (`probes/probe_timing_stall.cpp`;
`out/probe_timing_stall_{today,fix}.txt`; built against the same `tree/` and `fix/` copies, run under the memory cap).
Probe 3's `starts` vector was not reset after case (3), so (4)/(4′) mixed a stall with a late wake; here each case
has its own schedule. "A" is the Localizer's read (+0.1 ms), "B" the record's (+0.4 ms); 300 ticks counted.

| case | today | fix |
|---|---|---|
| (4a) a 6 ms stall between A and B on tick 100, ALONE (φ 5 ms; φ 3 ms identical) | A: 299 high (+3 %), 1 zero; B ≠ A on 299 | **A: 298 exact, 1 high, 1 low; B ≠ A on 1** — the stall tick's B re-samples; tick 101's A is held from it (×1.67); tick 102's A reads low over 14 ms; then exact |
| (3b) tick 101 wakes 4 ms late, no stall | A: 299 high, 1 low | A: 298 exact, 1 low, 1 high (no hold involved — 6 ms > 5 ms) |
| (3c) tick 101 wakes 6 ms late, no stall | A: 299 high, 1 zero | **A: 297 exact, 2 high, 1 low** — tick 102 wakes 4 ms after 101 and is HELD (one tick stale) |
| (4c) the stall AND the 4 ms late wake (probe 3's case (4) as it really ran) | = probe 3 (4) | = probe 3 (4): 298 exact, 1 zero, 1 high |

**What it shows:** a late or stalled read costs the hold two or three of the Localizer's ticks, then nothing; today's
adapter is off on the same ticks AND on every other one. Both limits are HA-215's, and neither one's frequency on a
robot is known (Part B's tick-span measurement is the first look).

**Probe 4 — can "byte-for-byte unchanged" be checked?** (`tools/case_hashes.py`; `out/case_hashes_run*.txt`). The
suite's `-s` output (every passing assertion with its values) is 497 MB and takes 26 s; hashed per test case:
two plain runs differ in 224 cases (doctest prints pointer values; ASLR moves them); with `setarch -R` only the
thread-contention case `screen (S6): the owner under contention…` differs; with hex addresses masked (6+ digits) and
no `setarch`, that case and `DebugRecord: tankSteerActive (TANK1) is the struct's LAST field` (it prints decimal
addresses) differ. **`setarch -R` + the mask + excluding S6 gives a deterministic per-case fingerprint** — §2.4 uses it.
*(Re-check R1: that was measured on two runs of the SAME binary only. Between a base and a changed binary the raw
`write()` of `ProsCharSink` interleaves differently with doctest's block-buffered stdout, `path:line:` headers and the
summary are attributed to the wrong case, and line numbers move — §2.4 now runs `stdbuf -oL` and hashes with
`tools/case_hashes_v2.py`, which masks and drops all of those; measured base vs fix: exactly the 4 expected cases.)*

### 0.5 Is "about 0 or a spike" measured? No.

It is DERIVED: from the adapter's code (opened), the µs clock (HA-101, settled), and PROS's documented 10 ms copy
(HA-213, never measured on our brains). The Localizer's every-tick 0 additionally needs the copy's phase to sit in the
Localizer-to-record gap — HA-214 (invented: no one knows the phase, nor whether it drifts) and the gap's width — HA-215
(reasoned). Nothing on the robot prints the rate (§0.2), so no robot log can confirm or refute any of it; D3 (a) is
the measurement. If HA-213 is wrong in the "more often" direction (the buffer refreshes faster than 10 ms), today's
second read is a noisy short-interval derivative rather than 0, the baseline still moves, and IMU1's hold is still
right.

---

## §1 Scope — IN (D1 (a), D2 (a), D3 (a) assumed; the alternatives' scope is in their option text)

### Part A — the hold, the reproduction-first tests, the documents

1. **`include/shulib/hal/pros/imu.hpp`** — the only library CODE change.
   - A public `static constexpr std::uint32_t kRateHoldUs = 5000;` — the hold, **5 ms in whole microseconds** (the
     robot clock's own resolution, `micros()`; re-check R3/R13: in double seconds the exact edge is decided by
     rounding), with `static_assert(kRateHoldUs > 0)`.
   - `yawRate()` (:134-137): read `clock_->now()` FIRST; if an answer was given and
     `std::llround((now - rateAnsweredAt_).value() * 1e6) < kRateHoldUs`, return it — no device read, no
     `faultedReads_` change, the differentiation sample untouched; otherwise record the answer's instant and
     dispatch with `now`. **The hold covers BOTH sources** (the gyro read too): the build-order DoD is
     "every call within one tick answers the first call's value" (build-order :1196), and a record that differs from
     the Localizer's value is a lie in the evidence column whichever source made it.
   - `differentiatedRate(now)` (:179-203): takes `now` instead of reading the clock after the device. A FAULTED read
     (non-finite, :181-184) still answers last-good and still opens the hold (landmine 4); it does NOT move the
     differentiation sample (unchanged), so the next good tick differentiates over the whole interval since the last
     GOOD sample. The `dt <= 0` guard (:195-196) becomes unreachable while `kRateHoldUs > 0` — keep it, with a comment
     saying so (a precondition there would throw on the robot for a defensive branch).
   - The first call after construction still answers 0 (:186-192) — unchanged (T7).
   - Two new `mutable` members (the answered flag and instant), beside :227-230's, under the existing "state OF THE
     READ" note (:222-223).
   - **Text:** the banner's YAW RATE paragraph (:20-35) and the class doc (:83-88) rewritten: one answer per tick, ask
     as often as you like INSIDE one tick; why 5 ms (half the 10 ms tick, `hal/pros/tick_pacer.hpp` :51); the THREE
     limits (D1 (a); re-check R15/R16) — a stall over 5 ms inside a tick, or a tick that wakes under 5 ms after a late
     one (HA-215); the tick's first read changing position, with the copy between the two (HA-214); a read from
     outside the tick 5–10 ms after its first read makes the next tick stale and the one after skewed, and a loop
     whose period is not a multiple of 10 ms reads 0 on a large share of reads (HA-215, HA-213); a held read never extends the hold; the 10 ms copy it rests on — HA-213; the residual the
     hold does not cure and its cure (the gyro, once HA-04/HA-109 are measured) — HA-214; "One owner, the loop; not
     safe to read concurrently" kept (the hold adds state under the same rule). `yawRate()`'s doc (:127-133) loses
     "Call it once per tick". `faultedReads()` (:175-176) says a held read is not a device read and is not counted.
2. **`include/shulib/motion/move_to_pose.hpp`** :282-285 — comment only. The observer keeps reading `loc.twist()`,
   but the stated reason ("a second read in the same tick is ~0 or a spike") is false after IMU1. The reasons that
   remain: it is the value the estimate used this tick, finite-guarded by the Localizer (`localizer.hpp` :364-365) —
   one source of truth; and P14 pins it.
3. **`include/shulib/diag/debug_record.hpp`** :149 — `imuYawRate`'s one-line doc gains: the IMU's answer for this
   tick, the same value every reader saw that tick, the Localizer's included (one answer per tick since IMU1).
4. **Test infrastructure** (host only; the shim is `#error`-gated, `test/pros_shim/pros/imu.hpp` :29-32):
   - `test/pros_shim/pros/imu.hpp` — `ImuPortState` (:51-60) gains `double (*rotationAt)(int port) = nullptr` (when
     set, `get_rotation()` returns it after the disconnected/calibrating screens) and counters `rotationReads` /
     `gyroReads` (device reads that reached the port); the banner's "Beliefs modeled" list gains the 10 ms copy
     (HA-213). `resetImus()` (:68) already restores the defaults.
   - NEW `test/imu_buffer_model.hpp` — the HA-213 model, promoted from probe 2: the copy period (10 ms) and phase,
     the time each device read takes (`gap`), a clock that moves by `gap` per device read and returns to the plant's
     time at each tick, truth from a function (adapter tests) or from the plant's state history (loop tests), and the
     pacer that steps the plant. Its banner says it models a BELIEF (HA-213/205/206) — "host test plant", never
     "simulator".
   - NEW `test/imu_rate_once_per_tick_test.cpp` — §2.2's tests, except T3's pin. It names NO symbol the adapter
     change adds, so it builds against today's adapter (the reproduction-first run — re-check R2).
   - NEW `test/imu_rate_hold_pin_test.cpp` — T3 (c), the pin `kRateHoldUs * 2 == kTickMs * 1000`; written in the same
     step as the adapter change (it names `ProsImu::kRateHoldUs`). `test/CMakeLists.txt` :79 GLOBs `*_test.cpp`, so the
     reproduction build is the tree without this one file — no CMake edit.
   - `test/pros_imu_adapter_test.cpp` :139-149 — the gyro case reads twice at ONE instant and expects the second
     value; under the hold it advances the clock 10 ms between the two reads, and gains a same-instant check (the
     second read answers the first). No other case changes (:123-137's "same tick again" already expects the hold).
   - `test/motion_tank_steer_test.cpp` :1215-1219 and :1305-1309 — comments only: `ProsShapedImu` models the PRE-IMU1
     robot's second read and leaves out the next tick's first read (§0.2); P14 stays as the pin that the observer reads
     `loc.twist()`; IMU1's T8 runs TANK1's scenario through the real adapter. Also :1261 ("D4's, IMU1") and :1354
     ("D4's IMU1 owns it") become "fixed in `ProsImu` by IMU1"; :26 (the file's index line for P14) stays true. P14's
     assertions are unchanged — `ProsShapedImu` is its own `IImu`, not `ProsImu`, so IMU1 cannot move it (§2.4).

### Part B — the bench stage (D3 (a))

5. **`src/programs/bench_r3a.cpp`** — a new STAGE 2c, "the IMU's data cadence (HA-213/205/206; HA-04/HA-109)", after
   STAGE 2 (:735-) on the same fresh `ProsImu` + `ProsClock`, prompted "turn the robot slowly by hand, ONE way" (landmine 15):
   - **cadence:** a tight loop for 2 s reading `pros::c::imu_get_rotation()` with `pros::micros()`; print the number
     of distinct values, the intervals between changes (min / median / max µs) — HA-213 holds if they cluster at
     10 000 µs;
   - **phase:** 100 ticks paced by `pros::Task::delay_until(&prev, 10)` — as STAGE 5 paces (:1863-1870); NOT
     `ProsTickPacer`, whose header pulls `motion/motion_scheduler.hpp` into the bench program (R10); in each, poll the
     rotation from the wake until it CHANGES (at most
     9.5 ms; a tick with no change is counted and printed, never guessed — a 1 ms window would see a 10 ms copy on
     only one tick in ten); print the copy's offset from the wake (first, median, last tick, and its drift per
     second) — HA-214;
   - **tick span — a stand-in BY PLAN (R10):** Bench Tests builds no Robot and no Localizer, and STAGE 2c adds no
     include beyond those at :74-117 (no `hal/pros/tick_pacer.hpp`, no `localization/`). It times the device reads one
     motion tick makes between its two IMU reads — each drive motor's position, velocity, current and voltage (the C
     API `motor_get_*`, read only) and the battery — plus one serial line through the stage's one-call path, 100 times,
     and prints min / median / max µs, the line saying "a LOWER bound on a motion tick's span: no Localizer
     arithmetic, no record build, no motor command" — HA-215;
   - **the gyro (R18 — pinned):** never `ProsImu::yawRate()` (it answers CANONICAL CCW rad/s, :201, and under Part A's
     hold a tight loop sees it refresh at most every 5 ms). Over the one-way turn the stage compares ANGLES, both raw
     CW degrees from the C API: Δ`imu_get_rotation()` between the FIRST and LAST change instants its tight loop sees
     (≥ 2 s apart, so the 10 ms copy's quantization is under 0.5 %), beside ∫ `imu_get_gyro_rate().z` dt over the
     same window (trapezoid on `micros()` stamps, z sampled every poll). Same sign ⇒ z shares `get_rotation()`'s
     convention, so `GyroRateZ` and the differentiated rate agree in sign through `ProsImu`'s one negate (HA-04;
     whether that convention is CW is HA-02's, Station 4.2); a ratio within ~10 % of 1 ⇒ HA-109's deg/s. Each line
     names its convention ("raw deg, PROS's rotation — CW-positive per HA-02"; "raw z, integrated").
   Every line through SER1's one-call path (the stage's `emit`/`emitf`, already `serialLine`-based since SER1). It
   reads the sensor and the clock only; it commands no motor. **Inside station 2's own functions** (`tImu` →
   `reportImu`, ~:2916-2926 and ~:736-930) — never `kMenu`, `menu_grid` or `bench_r3a.hpp`, which MECH-A6 owns (the
   rule L6-B and VIS1-B already follow for stations 11 and 12); no new menu button (the 3 × 4 menu is full —
   `kMaxMenu` = 12, ~:329). Because `7 RUN ALL` calls `tImu` (~:2927-2934), STAGE 2c runs there too: it must say
   on the panel what to do before it starts reading, as STAGE 2's rotate test does.
6. **The worksheet** (`docs/internal/chunks/R3a-BENCH-WORKSHEET.md`) — a NEW **Station N — THE IMU'S TIMING** (Bench
   Tests button 2's STAGE 2c), after Station I with the other Bench Tests stations (R10: Station 4's premise is "no
   code", :231-234; buttons 11 and 12 have their own stations, H and I); §3.3 has the steps: upload `make ROBOT=tank`
   (Bench Tests), what to do (turn slowly, ONE way; about a quarter turn a second for the gyro part), what each printed
   number settles, and the pass/fail reading for each HA. Station 4 gains one pointer line to Station N.

**NOT touched by either part:** `hal/imu.hpp` (F4 — unless D2 (b)), `hal/pros/device_provider.hpp` (the constructor
call `imu_.emplace(port, math::Angle{}, clock_)` at :318 is unchanged — the hold has no constructor argument), every
reader in §0.3, `src/` (Part A), the host test plant (`sim/`).

---

## §2 The parts, the tests, the mutations, the definition of done

### 2.1 Parts and sizes (each its own executor run, verified and committed before the next)

| Part | What | Size | Compared with |
|---|---|---|---|
| **A** | §1 items 1–4: the hold, the shim hook, the model, the tests (reproduction FIRST), the mutations, §3's documents | **~1¼ builder-days** (1–1½) | SER1 (~¾–1: one header, ~20 `src/` sites, a gate check, tests, ~8 documents). IMU1 A has one ~30-line code change, but a test model and ~11 adversarial tests through the real loop (the re-check added a pin TU, T8 (a′), T9 (b), T10 (b) and T11's outcome assertion — each a few lines over the same model, ~+¼), and ~9 documents |
| **B** (D3 (a)) | §1 items 5–6: the bench stage and its worksheet station | **~½ builder-day** | TANK1 Part B (~½: one result-line column, its docs) — B is one `src/` stage plus a station, through the seven-build gate |

**Said plainly:** REMAINING-WORK's "~½–1 day (rough)" (row 32; :218) becomes ~1¼ days for A and ~1¾ with B. The growth is
the finding of §0.2 (the Localizer's own rate, through the real loop, must be tested — not only the adapter) and the
bench stage, which did not exist in the rough figure.

### 2.2 The tests (Part A) — each tries to break ONE rule; "TODAY" is what it does on `ceb17c0`'s adapter

All in `test/imu_rate_once_per_tick_test.cpp` over `test/imu_buffer_model.hpp`, unless named otherwise.

- **R0 — the model is sample-and-hold (harness self-check).** Two device reads 1 µs apart inside one copy period
  return the identical rotation; across a copy they differ by exactly one copy's turn. *Guards against:* a model so
  kind (continuous rotation) that today's adapter looks nearly right and the reproduction proves nothing. TODAY: green
  (the model is not the bug).
- **T1 — one answer per tick.** φ ∈ {before the first read, three points between the first and the last read, three
  after} — every φ strictly between two read instants, never ON one (on a read instant HA-214's residual makes the
  FIRST read 0 or double; that is T2's business to avoid, not T1's to test — re-check R12), gap ∈ {25, 100, 400} µs, 2 / 3 / 5 reads per tick, 200 ticks, constant 90°/s: every later read in a tick
  `==` the first (bit-equal). TODAY: red (probe 1: 0 or ×33).
- **T2 — the first read is the truth, however many reads follow.** Same grid, no jitter: the first read equals the
  truth within 1e-9 relative on every tick, and the first-read sequence is bit-identical with 1, 2 and 5 reads per
  tick (extra reads never move the baseline). TODAY: red (0 every tick in-band; +3.1 % elsewhere).
- **T3 — the hold's edge, and its pin.** (a) **The edge, in literal microseconds** (no new symbol — R2): on the
  shim's µs clock (`ProsClock`, stepped with `advanceUs`), answered at t: a read at t + 4 999 µs is held (no device
  read — `rotationReads` unchanged, the same value); a read at t + 5 000 µs re-samples (one device read; the rate is
  over 5 ms). At three anchors — t = 0, t = 10 000 µs and one that is not round (t = 1 234 567 µs) — because the hold
  compares whole microseconds (§1.1, R3/R13), the edge is exact at every one. TODAY: **red** (today re-samples at
  +4 999 µs — `rotationReads` moves). M2 (`<` → `<=`) turns the +5 000 µs case red. (b) **A held read never extends
  the hold:** reads every 4 ms for 200 ms — a fresh answer (one device read: `rotationReads` + 1) on every second
  read at the latest, and every held read equals the latest fresh answer. Only the device-read criterion is
  asserted (R13 (b): "no value repeated more than twice running" is FALSE for a correct hold on a constant turn —
  every fresh answer of a constant turn is the same number; the checker measured a run of 22 under a continuous
  model, 4 under sample-and-hold). TODAY: green (today re-samples every read); it exists for M11 — a hold restarted
  by every read freezes the rate for ever under any reader that asks faster than every 5 ms (the checker's M11 run:
  50 held in a row), and no other test asks that fast. (c) **The pin** — in its OWN TU,
  `test/imu_rate_hold_pin_test.cpp`, written in the same step as the adapter change because it names
  `ProsImu::kRateHoldUs`, which does not exist before (R2): `ProsImu::kRateHoldUs * 2 == ProsTickPacer::kTickMs *
  1000` (`hal/pros/tick_pacer.hpp` :51), in integers, in one TU that includes both host-shimmed headers — so a
  change to the tick moves the hold or fails.
- **T4 — a held read touches nothing.** 100 ticks × 4 rate reads (rate only — no `heading()`): `rotationReads` ==
  100; `faultedReads()` unchanged by held reads; on `GyroRateZ`, `gyroReads` == 100. TODAY: red (400 device reads
  on either source).
- **T5 — a fault inside a tick.** (a) The tick's first read is `PROS_ERR_F`: it answers the last good rate and counts
  ONE fault; a second read that tick answers the same and counts nothing. (b) The next good tick (20 ms after the last
  good sample) answers the truth (the interval runs from the last GOOD sample). TODAY: (a) red (the second read hits
  the device and counts a second fault).
- **T6 — the gyro answers once per tick too.** z changes between two reads of one tick → the second answers the
  first; the next tick (10 ms later) answers the new z. TODAY: red.
- **T7 — nothing else changed: the oracle.** A test-local copy of `ceb17c0`'s differentiation (`PreImu1Differentiator`,
  from `hal/pros/imu.hpp` :179-203, its provenance in its comment) is the oracle: with ONE read per tick, ≥ 5 ms apart,
  over a schedule with jitter (±0.3 ms), an overrun, two faulted ticks and the calibration window, the new adapter
  answers BIT-IDENTICALLY to the oracle on every tick — including "the first call answers 0". **On a clock that does
  NOT move inside a read** (the shim's µs clock stepped only between ticks, or `FakeClock`) — never the model's
  creeping clock: the fix stamps a sample before the device read and `ceb17c0` after it, and on a creeping clock the
  same interval rounds differently on ~1 % of ticks (re-check R4/R17). **The first call** is made at t = 0.37 s with
  the rotation at 12.5° CW — non-zero both — so a mutant that differentiates from a zero baseline (M10) answers a
  rate there, not 0. *The adapter-level proof that the hold changes nothing but the extra reads.*
- **T8 — the loop (probe 2, promoted).** The REAL Chassis / MotionScheduler / Localizer / MoveToPose, the `ProsImu`
  over the model as the context's IMU and the Localizer's, the right side 9 % weak, plant kA ∈ {0, 0.02, 0.03}, gap ∈
  {25, 100} µs; φ ∈ {**in-gap: 2.25, 2.5, 2.75 × gap** — strictly between the Localizer's rate read (the tick's 2nd
  device read) and the record's heading read (the 3rd) — **far: 20 × gap and 4.5 ms**}, never ON a read instant
  k × gap (HA-214's residual — R12); the 40 in (`moveTo(40, 0, 0)`) and the acceptance run
  (`competition/acceptance_run.hpp`'s four legs):
  - **(a)** a sink that wants records vs none: per-tick truth (memcmp, TANK1's `sameTruth` shape), the Localizer's ω
    and every commanded volt bit-identical — *the record's read changes nothing*. TODAY: red (in-gap kA 0.02:
    TIMEOUT vs FINISHED; far: the Localizer reads ~3 % high, ω differs on every tick — re-probed 2026-10-22,
    `revise/out/t8grid_today.txt`).
  - **(a′) the one band (a) cannot promise — φ ∈ {3.25, 3.5, 3.75} × gap** (re-check R11): at a chained-motion
    boundary the scheduler's second `update()` reads the HEADING one device read later with records on (the record's
    heading read sits before it), and a copy in [3, 4) × gap falls between the two; `heading()` is not held (landmine
    9), so the later read sees the newer copy — physical, not a defect (measured on the µs-hold prototype: the
    acceptance run differs from the face→driveTo boundary on, 121 of 522 ticks at kA 0.02; the 40 in, which has no
    boundary, is identical). Asserted there: records on vs off bit-identical UP TO the first chained-motion boundary;
    after it, (b) and (e) hold. TODAY: red (as (a)).
  - **(b)** every record's `imuYawRate` `==` the Localizer's ω on that tick (the pacer captures `loc.twist().omega()`
    per tick; records paired by `r.t`). TODAY: red (0 / spikes).
  - **(c)** the Localizer's ω is never exactly 0 on a tick whose truth turned more than 0.05 rad/s over the last
    10 ms, **after the first THREE ticks of each motion** (each acceptance-run leg's first tick found from the
    records' `activeCommandId`). The exemption is HA-213's sample-and-hold, not IMU1's: the 10 ms copy lags a
    motion's start, and on the µs-hold prototype at kA 0 the third tick (index 2) reads 0 records on AND off alike
    (re-check R12). TODAY: red in-gap (150–152 zero turning ticks of the 40 in at kA 0; 485–486 at kA 0.02).
  - **(d)** at each chained-motion boundary of the acceptance run, the second `update()`'s ω `==` the first's.
    TODAY: red (0).
  - **(e)** every run FINISHES (kA ≤ 0.03), END inside the unchanged arrival rule. TODAY: red in-gap.
- **T9 — a corrector reads first.** The Localizer with a `GpsCorrector` (a fresh on-strip `FakeGps` fix every tick)
  over the `ProsImu`, the robot turning steadily at 1.2× and at 0.8× `GpsCorrectorConfig::maxYawRate`: on every tick
  the gate rejects for yaw rate exactly when the Localizer's own ω that tick exceeds `maxYawRate` (the corrector's
  reject counter vs a count over the captured ω — the executor names the accessor it uses). TODAY: red (the gate gets
  the first read, the Localizer the second). **(b) the realistic cadence (re-check R15):** a fresh fix every 5th tick
  (the ~20 Hz sensor against the 100 Hz loop, `localizer.hpp` :302-303), the copy's phase strictly between the
  gate's read and the Localizer's: on every FRESH tick the gate's value equals the Localizer's ω that tick (asserted;
  TODAY: red — they differ on every fresh tick); the Localizer's exact / zero / double counts are RECORDED in the log,
  never asserted exact (the prototype: 180 / 60 / 60 of 300 — limit (2), HA-214).
- **T10 — Calibrate's REST record.** `RestCapture` (`calibration/rest_capture.hpp`) over the `ProsImu` with the robot
  still and the rotation drifting +0.01° per copy (1°/s). The settle tick's first rate read (:388) is the tick's 1st
  device read; every capture tick reads `heading()` first (:436), so its rate read (:456) is the 2nd — and on the
  capture-start tick (:417-426) the :456 read is held (re-check R14). (a) φ ∈ {2.25, 2.75} × gap and 5 ms (never on a
  read instant): the summary's `yawRateMean` is the drift rate within 0.1 % and its `yawRateSigma` under **1e-5
  rad/s** — an absolute bound, about 1/20 of what ONE doubled sample adds (measured on the µs-hold prototype: clean σ
  5.6e-7 at gap 25 µs, 2.2e-6 at 100 µs; one doubled sample 2.2e-4). TODAY: red at every φ (σ 4.5e-2 in-band, the
  first sample a spike of 0.01° over the gap; 2.25e-4 even far, mean 103.3 % in-band; the REST record is HA-22's
  evidence). (b) **the residual, pinned — φ ∈ {1.25, 1.75} × gap:** the first FRESH capture read is the tick after
  the start, at the 2nd device read, so its interval (10 ms + gap) holds TWO copies — exactly ONE capture sample is
  about twice the drift rate, every other is the drift rate, and σ ≤ 3e-4 (the prototype: 2.24e-4). TODAY: red (σ
  4.5e-2). Why (1,2) × gap is not held to (a)'s bound: the tick's first rate read moves from the settle position to
  the capture position when the capture starts — the hold groups reads by time, so a moved first read is one doubled
  (or zero) sample when a copy lands between the two positions (R15; HA-214's amended claim).
- **T11 — the calibrator's rest gate** (if under ~80 lines with `test/calibration_test_rig.hpp`; else logged as not
  built, with why). `DrivetrainCalibrator` in a rest window while the robot is turned by hand at 3°/s (over
  `restYawRateFloor`'s 2°/s, `drivetrain_calibrator.hpp` :287), every motor still, for every φ strictly off the read
  instants: **the rest window never completes** — no pulse starts (every motor's commanded voltage stays 0) until
  `restTimeout` (:293) refuses with `RestTimeout`, the robot still rotating (:219) — **and** `status().atRest` is false
  on every tick. TODAY: red at every φ, by two different routes (re-check R8): in-gap, `tickRest`'s read (the tick's
  FIRST rate read, :1018-1019 → :1477) is 0, so the window completes after `restSettleTime` and a pulse starts while
  the robot turns — the defect; `updateStatus`'s read (the LAST, :688 → :1740) is then a spike, so `status().atRest`
  alone would be green there. Far from the gap the last read is 0, so `status().atRest` is true — red on the
  status.

### 2.3 The mutations (each applied, the suite run, RED observed, restored; the coordinator re-runs a subset)

| # | Mutation | Must turn red |
|---|---|---|
| M1 | the hold DISABLED, the constant kept (`yawRate()` always re-samples; the pin TU still builds) — **today's behaviour; the reproduction-first run itself is `ceb17c0`'s unchanged adapter without the pin TU (R2)** | T1, T2, T3 (a), T4, T5 (a), T6, T8 (a–e), T9, T10 (T11) |
| M2 | `<` → `<=` in the hold (exact in whole microseconds — R3/R13) | T3 (a) (the +5 000 µs read) |
| M3 | the hold 5 ms → 0.3 ms (shorter than an in-tick gap) | T1 (the 400 µs rows), T8 (d) (the 100 µs acceptance rows: a boundary's second rate read comes 3 × gap = 300 µs after the tick's first; an ordinary tick's record read, 2 × gap after it, stays held even under M3) |
| M4 | the hold 5 ms → 10 ms | T3 (c) (the pin), T3 (a) |
| M5 | a held read still calls `get_rotation()` and discards it | T4 |
| M6 | a faulted read does not open the hold | T5 (a) |
| M7 | a faulted read moves the differentiation sample | T5 (b), T7 |
| M8 | the gyro path bypasses the hold | T6, T4's gyro half |
| M9 | the model's copy made continuous (the hook returns the truth at the read instant) | R0 |
| M10 | the first call answers a rate instead of 0 (from a zero baseline) | T7 (its first call at a non-zero time and rotation — R17) |
| M11 | every read, held ones included, restarts the hold (`rateAnsweredAt_ = now` before the check) | T3 (b) |

### 2.4 "Every other consumer unchanged" — how it is proven

1. **By construction:** the change is inside `ProsImu`; every host suite outside the adapter's own tests runs on
   FakeImu (stateless) or on a shim clock that never moves inside a tick (§0.1), where today's adapter already
   answered every in-tick read with one value. (Re-checked 2026-10-22, corrected by re-check R7: the test TUs whose
   include closure holds `hal/pros/imu.hpp` — read from the compiler's dependency files of a full build, not a
   grep — are exactly **nine**: `brake_stop_stack`, `driver_panel_claws`, `mechanism_devices`, `mechanism_parts`,
   `mechanism_service_evidence`, `pros_device_provider`, `pros_imu_adapter`, `robot_two_driver`, `xdrive_config`
   (`_test.cpp` each; the grep `grep -ln "ProsDeviceProvider\|hal/pros/imu.hpp" test/` returns 11 files, two of
   which — `guide_examples`, `robot_builder` — only name it). They step their clocks by whole ticks; none was seen
   reading a real `ProsImu`'s rate twice less than 5 ms apart. That is a read of the files, not a proof: the
   fingerprint below is the proof — these nine are the only TUs whose cases could legitimately move — and a case that
   differs outside the named set is INVESTIGATED and named in the log with why — never waved through.)
2. **By fingerprint (re-check R1's tool, `.claude/coordinator/scratch/brief-IMU1/tools/case_hashes_v2.py` — the
   executor copies it into its own scratch):** the base is the MERGE BASE Part A starts from (after VIS1-A1 or
   MECH-A3 lands, that tip — §6), built fresh; the base binary is run **twice** and Part A's once, each as
   `(ulimit -v 4194304; timeout 300 setarch -R stdbuf -oL ./build/test/shulib_tests -s --order-by=file) 2>&1 |
   python3 -I case_hashes_v2.py "$PWD"` from the tree's root (`stdbuf -oL` — the serial sink's raw `write()` and
   doctest's stdout must not interleave by buffer size; the tool masks addresses, 12+-digit decimals, line numbers
   and the tree's path, and drops doctest's `path:line:` headers and its summary — R1). Never beside another suite
   run on the laptop (two cases write fixed host temp paths). **NOISE** = any case that differs between the two base
   runs (seen 2026-10-22: S6 every time; `ProsDeviceProvider 1r` and `ProsFileStore: the whole SAFE SAVE…` once in
   six runs) — named in the log, excluded. **Base vs Part A must be identical for every test case except the noise
   and:** the thread-contention case `screen (S6): the owner under contention — every successful handover moves the
   generation` (nondeterministic by design); the two source scans `P3-T3: "shulib Teleop" is retired everywhere…` and
   `P3-T3: the buttons left the mechanism table…` (`test/drive_program_source_test.cpp` :189 and :228 `CAPTURE`
   every test file they scan, so IMU1's new test files change them — R1); the cases in
   `test/pros_imu_adapter_test.cpp` whose text changed (the gyro case; every other case in that file must match —
   line numbers are masked, so the gyro edit cannot move them); the new IMU1 cases (absent at the base). Every TANK1,
   holonomic, Localizer, corrector, calibration and driver-loop case is in the "must match" set (line numbers
   masked: §1.4's comment edits in `motion_tank_steer_test.cpp` move none). A case outside these that differs is run
   again on its own; if it still differs, it is investigated and named with why. (Measured 2026-10-22 on the
   checker's base and a prototype of Part A: exactly S6, the two P3-T3 scans and the gyro case differ.)
3. **On the robot's side:** T7 (one read per tick, on a clock that stands still during a read → bit-identical to the
   old adapter — R4/R17) and T8 (a) (records on == records off, for the φ the hold can promise — R11).

### 2.5 Definition of done

**Part A**
- [ ] `IMU1-PROGRESS.md` created FIRST and appended as the work goes.
- [ ] **Reproduction first:** the shim hook, the model and R0, T1–T11 (`test/imu_rate_once_per_tick_test.cpp` — NOT
      the pin TU, which cannot exist yet: R2) written and run against `ceb17c0`'s UNCHANGED `hal/pros/imu.hpp`; T1,
      T2, T3 (a), T4, T5 (a), T6, T8 (a–e), T9, T10 (T11) seen RED with their counts logged; R0, T3 (b) and T7 green
      as stated. Only then is the adapter changed and the pin TU added.
- [ ] The hold in `ProsImu` (§1.1); every test green; the whole suite green; no existing test edited beyond §1.4.
- [ ] §2.4's fingerprint: identical except the named cases — the diff of the two hash files in the log.
- [ ] M1–M11 run, each red under the named tests; the table in the log with the failing case names.
- [ ] The ARM header gate; `python3 tools/src_build_gate.py check` (the seven builds — the adapter is in every robot
      build through the provider) and `self-test`; the doc gates (§7).
- [ ] §3's documents in the same change; the prose review logged (rule 4 — grep first, then write).
- [ ] Not committed. No `pros` command; no `/dev/ttyACM*`.

**Part B**
- [ ] STAGE 2c in `bench_r3a.cpp`; `make ROBOT=tank` builds; the gate's seven builds pass with 0 disallowed warnings;
      the stage prints through SER1's one-call path only (the gate's object check).
- [ ] The worksheet station; HA-213/205/206 and HA-04/HA-109 name it as their settle.
- [ ] Nothing run on a robot by the executor; not committed.

---

## §3 Documents — in the same change as the code (pace rule 3), the prose review logged (rule 4), ≤ ~5 public each (rule 7)

Line numbers are `ceb17c0`'s unless marked "`71b4764`" — the coordinator's plan commit (VIS1's answers), which landed
DURING this brief's continuation (2026-10-22 00:59; documents only, no code) and moved (re-check R6, from `git diff -U0
ceb17c0 71b4764`): `hardware-assumptions.md` +2 after :150, +2 more after :2854, +1 more after :2901;
`REMAINING-WORK.md` +2 after :23, +1 after :89, +4 after :167; `HANDOFF-remaining-work.md` +2 after :45;
`docs/changelog.md` +3 after :206; `build-order.md` +36 by its deviations table (the IMU1 section :1190-1196 is
unmoved). **Re-anchor every edit by its text at the tip.**

### 3.1 Part A — public (three; the cap is ~5)

1. **`docs/changelog.md`** — a new section at the top of `## API 2.4`, in the file's own form (what changed / breaking or
   additive / what you must do):
   `### Fixed since 2.4 (<date>) — the IMU's turn rate answers once per control tick`
   - **What changed:** `hal::pros::ProsImu::yawRate()` keeps its answer for `kRateHoldUs` (5 ms): a read within 5 ms of
     the last answered one returns that answer and touches neither the sensor nor the differentiation sample. Before,
     every read re-differentiated over the time since the PREVIOUS read, so on a robot a second read in the same tick
     answered about 0 or a spike AND moved the next tick's starting point (the Localizer's own rate was damaged by the
     record's read — DOCS2's A17, TANK1's re-check, IMU1's probes). One new public constant, `ProsImu::kRateHoldUs`.
   - **Breaking or additive:** a BEHAVIOUR change in an adapter; **no signature changes and no version bump** —
     `hal/pros` adapters are implementations, not contracts (the Freeze Register's F13 row); the frozen `IImu` (F4) is
     untouched (D2 (a)). On the host test plant nothing changes (every host clock stands still inside a tick).
   - **What you must do:** nothing, for code that reads the rate inside a library tick: within one library tick,
     every read gets the library's value. **Two cautions (re-check R16):** a read from OUTSIDE the library's loop —
     between routine steps, or from a task of your own — 5–10 ms after a tick's first read makes the library's next
     tick read a stale rate and the one after a skewed one; and a loop of your own at any period other than a
     multiple of 10 ms gets a rate that is 0 on a large share of its reads (the IMU copies new data every 10 ms — A4
     register HA-213; on the host test plant a 4 ms loop read 0 on one read in five and never exactly). For your own
     loop, differentiate the heading over 10 ms or more yourself, or use the gyro's rate once HA-04 and HA-109 are
     settled.
   - **Not fixed:** the residual of HA-214 (the IMU's copy landing next to the read, or between two positions of a
     tick's first read) — its cure is the gyro's own rate, once the bench measures its sign (HA-04).
   - *Corrected 2026-10-29 at Part A's review (IMU1-PROGRESS.md): this template names two cautions and leaves out
     limit (1), a stall or late wake, which the team lead's D1 (a) requires in the changelog; the entry as written
     names all three limits, with the host figures IMU1's L1–L3 pin.*
2. **`docs/hardware-assumptions.md`** — §4's three new rows and entries (HA-213/205/206, or HA-213..215 — the collision
   note in the Status block) and the Held line; the count line after the newest chunk's paragraph ("IMU1 added THREE …
   With them, N" — the executor counts); and these existing entries amended:
   - **HA-179** (`71b4764` :2488-2494; `ceb17c0` :2486-2492 — "The IMU read:") — rewritten: before IMU1 the Localizer's rate on robot two was NOT the
     tick's clean first read: the record's second read (robot two's `TermSink` wants every record) moved the baseline,
     and for one band of the IMU's copy phase the Localizer read 0 on every tick — measured on the host with TANK1's own
     motor lag: a TIMEOUT of the 40 in (IMU1's brief §0.4, probe 2); since IMU1 every reader gets one value per tick,
     the corrector's gate and the chained-motion boundary included. Station M's floor run counts as evidence for HA-179
     only on an IMU1 build (D4).
   - **HA-22** (`ceb17c0` :3241-3247; `71b4764` :3246-3252) — amended: a REST capture made before IMU1 has a contaminated FIRST rate sample (the settle
     gate's read and the capture's first read share the tick — `rest_capture.hpp` :388, :417-426, :456), which inflates
     `yawRateSigma`; a REST record from a pre-IMU1 build is not HA-22's evidence (T10 measures by how much). **On an
     IMU1 build one residual remains (re-check R14):** when the IMU's copy lands between the settle read's and the
     capture read's positions in the tick (a band about one device read wide), ONE capture sample is about twice the
     drift rate — on the host test plant σ 2.2e-4 rad/s against a clean 5.6e-7, still far below HA-22's claimed 0.5°/s
     (8.7e-3 rad/s), so it cannot fake HA-22's verdict; it is named so a REST record is read knowing it.
   - **HA-04** (`71b4764` :424-438; `ceb17c0` :422) and **HA-109** (`71b4764` :867-877; `ceb17c0` :865) — Part B only: their *Settle* gains "Bench Tests button 2's STAGE 2c
     (IMU1 Part B), worksheet Station N: the gyro's z integrated over one hand turn beside the raw rotation's change
     over the same window, both raw CW degrees" (R18).
3. **`docs/api/`** — regenerated (`python3 tools/api_doc_tool.py generate`, then `check-fresh`): `pros-imu.md`
   (`ProsImu`'s class doc, `yawRate()`'s doc, `faultedReads()`'s doc, the new `kRateHoldUs` — the coverage gate requires a
   `///` on it), `debug_record.md` (`imuYawRate`'s one line), and the two generated indexes the new constant moves —
   `README.md` and `all-entities.md` (re-check R5; `shulib_tests` depends on `doc_gates`, so the suite does not build
   until they are fresh; the same two files MECH-A3, CAL1-A2a and VIS1-A1 regenerate — §6). `move_to_pose.hpp` :282-285 is a `//` comment inside a
   function — not on any page.

**Deliberately NOT touched (public), with why — the commit body says so:**
- `docs/guide/` — no chapter describes the IMU rate's per-call behaviour (grep 2026-10-22: no `yawRate`, `yaw rate`,
  `imuYawRate`, `once per tick` hit in `docs/guide/` about the IMU; `17-the-competition-program.md` :305's "once per tick"
  is the SD card). If the executor's grep at the tip finds one, it is fixed in this change.
- `docs/faq.md` :63-65 ("The yaw rate is differentiated from it") — still true.
- `docs/vexbuilder-integration.md` :559 (the `imuYawRate` field's offset in the record layout) — the field's meaning and
  place do not change.
- `docs/guide/14-what-it-cannot-do-yet.md` — HA-214's residual is an unmeasured belief; the register carries it. If the
  team lead wants it in chapter 14 too, that is a sixth public document and Part A's doc pass still fits.
- `docs/api/imu.md` — `IImu` is unchanged (D2 (a)); under D2 (b) it regenerates and `docs/roadmap.md`'s F4 row is
  amended.

### 3.2 Part A — internal

- `docs/internal/chunks/IMU1-imu-rate-once-per-tick.md` — this brief, committed at the plan commit (the coordinator's).
- `docs/internal/chunks/IMU1-PROGRESS.md` — created FIRST; every run's counts (the reproduction-first reds, the
  mutations, the fingerprint diff) go in it.
- `docs/internal/chunks/IMU1-COMPLETED.md` — at close, with its pointer moves (RESUMING rule 5).
- `docs/internal/build-order.md` — the IMU1 section (:1190-1196): its status lines; its body's "every `imuYawRate` a
  robot's record carries is about 0 today" made true (no robot program prints the field — §0.2) and the missed half
  added (the Localizer's own rate, robot two included); its DoD's last clause re-worded to D4's ruling. The deviations
  table row (`ceb17c0` :2192; `71b4764` :2228 — "TANK1 reads the Localizer's one read instead") gains "— which on robot two was itself damaged by
  the record's read (IMU1's brief §0.2)". The `**Next:**` block if IMU1 is next.
- `docs/internal/REMAINING-WORK.md` — row 32 (:127; `71b4764` :130): status and size (~1¼ / ~1¾ with Part B); row 4
  (:35; `71b4764` :37) and :160 (`71b4764` :163): "Its `imuYawRate` column does not count as evidence until IMU1 (#32) lands" →
  **"Its floor run (M.6) counts as evidence for HA-179/HA-180 only on a build with IMU1 (#32); M.0–M.5 need not wait"**
  (D4 (a)); under D4 (b): "M.6 may run first; a TIMEOUT on a pre-IMU1 build is not evidence against HA-179". The
  total's IMU1 line (:218; `71b4764` :225): "~½–1 (rough)" → "~1¼–1¾ (brief)".
- `docs/internal/HANDOFF-remaining-work.md` (`ceb17c0` :594; `71b4764` :596; :598 in the main checkout's uncommitted
  edits) — "(IMU1's, not TANK1's; TANK1 immune)" gains a dated correction:
  "— corrected 2026-10-22: not immune on robot two, whose record read damaged the Localizer's rate (IMU1's brief §0.2)".
- **History records — a dated pointer, never a rewrite** (the docs-redo rule: history records are indexed, not
  rewritten): `docs/internal/chunks/TANK1-COMPLETED.md` :67-68 ("TANK1 is immune") and
  `docs/internal/chunks/DOCS2-API-DEFECTS.md` A17 (:835-854) each gain one dated line pointing at IMU1's completion
  record. `TANK1-PROGRESS.md` and TANK1's brief are left as written (they are the record of what was believed).
- **The worksheet — Station M's evidence rule** (`docs/internal/chunks/R3a-BENCH-WORKSHEET.md`, Station M, before
  **M.6.1** at :1714). The words, for D4 (a):

  > **M.6 needs an IMU1 build.** Before IMU1 (2026-10-__), the tester's motion read the IMU's turn rate twice in
  > every tick, and on this robot the second read could spoil the next tick's first — the one TANK1's steering
  > correction uses. **A floor run counts as evidence for HA-179 and HA-180 only from a tester built after IMU1
  > landed:** before M.1's build, `git log --oneline | grep IMU1` on the laptop's tree must show it. Write M.2.2's
  > boot `hash` in M.6.4's table. A run on an older build (or a hash ending `-dirty` you cannot account for) is
  > still recorded, marked **pre-IMU1**, and a TIMEOUT on it — TSTR lit, the correction pinned at its limit — is
  > NOT evidence against HA-179. M.0–M.5 (the boot, the forward check, the aborts, the wheels-up run) need no IMU1
  > build.

  and **M.6.4**'s table gains a first row: `| build hash (M.2.2), and IMU1 in it? | | | |`. Under D4 (b) the
  paragraph's middle sentence becomes "a floor run on a pre-IMU1 build is recorded as such; a TIMEOUT on it is not
  evidence against HA-179 — re-run M.6 after IMU1 before judging HA-179 or HA-180".
  *Corrected 2026-10-29 at Part A's review (IMU1-PROGRESS.md): `git log --oneline | grep IMU1` matches two plan
  commits (`a62067b`, `73044ca`) on every pre-IMU1 tree, so it cannot tell a build without the fix. The worksheet
  checks the tree for the fix itself — `grep -c kRateHoldUs include/shulib/hal/pros/imu.hpp`, 1 or more — M.6.4's
  row records that count, and K.5 carries the same rule (§6's Robot sessions row).*
- `docs/internal/PROJECT-BRIEFING.md` — regenerated by `python3 tools/briefing_status.py generate` **after a fresh
  build of the merged tree** (never against a stale build — the 2026-10-15 rule), then `check`.

### 3.3 Part B — documents

- **Public:** `docs/hardware-assumptions.md` only — HA-213/205/206's and HA-04/HA-109's *Settle* name STAGE 2c and
  the worksheet steps; nothing is marked settled until the bench runs it.
- **Internal:** the worksheet — a NEW **Station N — THE IMU'S TIMING** *(Bench Tests button 2's STAGE 2c, robot two —
  added IMU1 Part B; HA-213/205/206, HA-04, HA-109)*, after Station I (R10): **N.1** upload Bench Tests (`rm -rf bin .d
  && make ROBOT=tank`, slot 3, as G.1's line); **N.2** run button 2, do STAGE 2 as before, then STAGE 2c's prompt: turn
  the robot slowly by hand, ONE way, for the cadence and phase parts, and about a quarter turn a second for the gyro
  part; **N.3** copy the four result blocks word for word; **N.4** what each number settles — HA-213 holds if the
  change intervals cluster at 10 000 µs (it fails "faster" if they cluster lower, and then IMU1's hold is still right,
  §0.5); HA-214's phase is a number, fixed or drifting (a drift means every boot passes through the bad band); HA-215
  holds if the stand-in tick body spans well under 5 ms (a LOWER bound — a long one is a warning, a short one is not
  proof); HA-04 holds if the integrated gyro z has the raw rotation's sign (both raw PROS degrees — R18; that the rotation
  is CW-positive is HA-02's, Station 4.2), HA-109 if the two angles agree within ~10 %. **Station 4** (:231-250) gains one line after its heading: "The IMU's TIMING — its 10
  ms copy and the gyro's sign — needs code: Station N." **Station A's table row for button 2** (:91) gains "then STAGE
  2c: turn it slowly ONE way when asked (Station N)". `IMU1-PROGRESS.md` / `IMU1-COMPLETED.md` gain Part B's record.

### 3.4 The prose review (rule 4) — the greps each part runs and logs, BEFORE writing that it was done

For every changed file, `grep -rn` over `docs/` and `README.md` for its name and its concepts, every hit read:
- Part A: `ProsImu`, `pros-imu`, `yawRate`, `yaw rate`, `imuYawRate`, `once per tick`, `differentiat`, `second read`,
  `read twice`, `immune`, `IMU1`, `kRateHoldUs`, `A17`. The hits on 2026-10-22 and their disposition are §3.1–3.2 above
  (plus `docs/roadmap.md` :241 and :1095, `docs/guide/07-getting-set-up.md` :117, `docs/changelog.md` :1723 — each
  names `ProsImu` only as an adapter, no per-call claim: no change). A NEW hit at the tip is read and dispositioned in
  the log.
- Part B: `bench_r3a`, `STAGE 2`, `IMU + ROTATE`, `Station 4`, `Station N`, `HA-04`, `HA-109`, `HA-213`..`HA-215`.

---

## §4 Hardware Assumptions — HA-213..HA-215 ONLY (the register's Held line gains them at the plan commit)

**NUMBERING — read first.** The register's next free number after its Held line (:39 at `71b4764`: HA-192..HA-203,
VIS1's) is HA-213, and **the L6 brief claims HA-204..HA-212 too** (its §4, written the same evening). The coordinator
assigns one block per brief at the plan commit; recommended: IMU1 → **HA-213..HA-215** if L6 is accepted first (a
three-id rename of this file), else L6 shifts. A fourth belief ⇒ stop and tell the coordinator.

Each in the register's form: *Claim · Source (`A4 register HA-nn` beside the code) · Confidence · Settle · Blast radius
if wrong*. **None is measured.** The premise that IS measured: HA-101 (`micros()` is µs and monotonic — settled
2026-08-13), so two reads in one tick are microseconds apart.

| # | Belief | Confidence | Cited in code by | Settled by |
|---|---|---|---|---|
| **HA-213** | The V5 IMU's readings (`get_rotation()`, `get_gyro_rate()`) are copied into the brain's shared buffer every **10 ms** (PROS's default data rate; shulib never calls `set_data_rate` — grep 2026-10-22), so between copies every read returns the SAME value: sample-and-hold | **reasoned** — the vendored PROS header says it in words (`include/pros/imu.hpp` :150-156, `set_data_rate`'s doc: "values are copied into the shared memory buffer only at 10ms intervals"); never observed on our brains | `hal/pros/imu.hpp` (the banner's YAW RATE paragraph — IMU1 A); `test/imu_buffer_model.hpp`'s banner; the shim's "Beliefs modeled" list | Part B (D3 (a)): STAGE 2c's cadence — the intervals between changes of a tight-loop read |
| **HA-214** | The phase of that 10 ms copy against the motion tick's wake is **unknown** — fixed per boot, or drifting — and when the copy lands within a read's jitter of the tick's ONE rate read, that read alternates **0 / about double** on the affected ticks (probe 3 case (2′): 73 zero and 73 double of 300 at ±0.05 ms of jitter); **or when the tick's FIRST read moves between code paths — a corrector's fresh-fix tick against a stale one, a REST capture's settle against its capture — with the copy between the two positions, the pair of ticks around each change reads 0 and about double** (re-check R15: fresh fix every 5th tick, 60 zero and 60 double of 300; R14: one doubled REST sample); IMU1's hold does not cure it. Before IMU1 the same phase decided whether robot two's Localizer read 0 on EVERY tick (copy between the Localizer's read and the record's) | **invented** (the phase, its drift; whether the IMU's copy and the RTOS's 1 ms tick share a clock) · **reasoned** (the consequence, from HA-213's model) | `hal/pros/imu.hpp` (the banner's residual sentence — IMU1 A) | Part B: STAGE 2c's phase — the change instant's offset from the wake over 100 ticks, and its drift per second |
| **HA-215** | Every rate read of one control tick falls within **5 ms** of the tick's first, consecutive ticks' first reads are **≥ 5 ms** apart, and **no reader outside the loop's tick reads the rate** — the motion task's body reads the rate early and fast (the Localizer's update, then a record — µs apart on the host plant's arithmetic), and a wake is rarely ≥ 5 ms late. So `ProsImu::kRateHoldUs` = 5 ms (half of `ProsTickPacer::kTickMs`) groups a tick's reads exactly. When it fails: a stall over 5 ms inside a tick → that tick's later readers get a fresh answer, two of the Localizer's ticks off; a wake ≥ 5 ms late → the next tick gets one stale answer (probe 3b); a read from outside the tick 5–10 ms after its first read → the next tick stale, the one after skewed (re-check R16) | **reasoned** (HA-32/HA-102's anchored 100 Hz cadence — both themselves reasoned, unmeasured under load; the tick body's span is unmeasured on a brain) | `hal/pros/imu.hpp` (`kRateHoldUs`'s doc — IMU1 A) | Part B: STAGE 2c's tick span — a stand-in, a LOWER bound (the device reads between a motion tick's two IMU reads + one serial line; R10); Bench Tests STAGE 5 (LOOP RATE) for the wake jitter, which has never run |

**Blast radius if wrong — each, in the register's words:**
- **HA-213 wrong, "faster"** (the buffer refreshes more often than 10 ms): before IMU1 the second read was a noisy
  short-interval derivative rather than 0, and the baseline still moved; IMU1's hold is still right, and HA-214's
  residual is smaller. **"Slower"** (e.g. 20 ms): a 10 ms tick's ONE read is 0 on alternate ticks even after IMU1 —
  the rate then needs differentiating over a longer window or the gyro's own rate (a new chunk). Visible at the bench
  (Part B) before any motion depends on it.
- **HA-214** — on an affected boot the Localizer's ω alternates 0 / ×2 tick to tick (mean right, noise large): TANK1's
  yaw observer (time constant 0.1 s, HA-179) averages most of it; `DriveBrake`'s exit and the calibrator's rest gate
  (`|ω|` under a floor) can pass on a 0 tick or fail on a ×2 tick; a corrector's yaw-rate gate mis-gates on ×2 ticks;
  a REST capture's `yawRateSigma` is inflated (HA-22's evidence). **Contained:** the gyro's own rate
  (`YawRateSource::GyroRateZ`, one constructor argument) once HA-04 and HA-109 are measured — §5's later change.
- **HA-215** (rewritten by re-check R16) — a tick body whose span between its first and last rate read JITTERS around
  5 ms (a slow SD write on some ticks, a long record build) damages the Localizer's OWN first read on most ticks:
  the later read is fresh on some ticks and held on others, so the next tick's first read is held or not by chance
  (host: a later reader at a random +4..+7 ms left 55 of 300 Localizer reads exact); a span STEADILY over 5 ms makes
  the Localizer answer the previous tick's late read — one partial tick late; a motion task routinely preempted by
  ≥ 5 ms gives one stale tick per preemption; a reader outside the tick (team code between routine steps, a screen
  task) costs two Localizer ticks per read (host: 60 of 300 for one read in 10 ticks; today's adapter 30).
  **Contained:** the constant (`kRateHoldUs`, pinned to the tick by T3) — or D1 (c)'s tick counter, which has no
  limit (3).

**Existing entries amended by IMU1 (no new number):**
- **HA-179** (TANK1's constants) — the "The IMU read:" paragraph rewritten (§3.1 item 2): the Localizer's rate on
  robot two was NOT clean before IMU1; Station M.6 counts only on an IMU1 build (D4).
- **HA-22** (the IMU's rate noise) — a REST record from a pre-IMU1 build is not its evidence (T10); on an IMU1 build a
  capture can carry ONE doubled sample in a narrow phase band (T10 (b), R14) — far below the claim's size.
- **HA-04** (the gyro's sign) and **HA-109** (its units) — Part B: their *Settle* gains STAGE 2c. Settled only when the
  bench runs it.
- **HA-32 / HA-102** (the 100 Hz anchored cadence) — their blast radius gains "IMU1's rate hold groups a tick's reads by
  time (HA-215); a cadence far off 100 Hz moves it".
Every new entry's code site carries `A4 register HA-nn`; the register's direction-1 grep (no `PROVISIONAL (A4` without
an HA id) stays empty.

---

## §5 OUT, and explicitly rejected

**Rejected (each is a D1/D2 option; the team lead may choose it instead — its scope is then in its option text):**
- **A per-tick snapshot in the Localizer (D1 (b))** — ~10 call sites in 8 headers plus a change to the corrector seam
  (`ICorrector::propose`), the record column's meaning changed, a rule of its own for the chained-motion boundary, and
  the trap left armed in the adapter for every NEW reader — a team's `imu.yawRate()` in an auton included — which the
  host can never show. It also edits `localization/localizer.hpp`, which L6-A1 (the one-axis wall seat) edits too (§6).
- **A tick counter the pacer bumps (D1 (c))** — a new seam into every loop that paces (six), and a loop that forgets to
  bump FREEZES the rate for the rest of the boot, invisibly on the host.
- **The gyro's rate as the default now (D1 (d))** — its sign and units are invented (HA-04, HA-109), and since TANK1 a
  wrong sign drives the tank yaw observer the wrong way to its 1.5 rad/s limit (HA-179's measured blast radius).
- **Amending the frozen `IImu` (D2 (b))** — a versioned change to F4 for a sentence no consumer needs.

**OUT (named, not done — each with where it would go):**
- **Making `GyroRateZ` the default** once Part B measures HA-04 and HA-109 — a later, separate change (~¼ builder-day
  plus a robot re-check of TANK1's observer on the gyro's rate), and the only cure for HA-214's residual. It needs
  its own go: a default source change moves every robot's rate.
- **Differentiating over a longer window, or detecting the copy instant** (another cure for HA-214) — not designed
  before Part B shows the residual exists on our brains.
- **A robot program that prints the record's `imuYawRate`** — the evidence column the plan promised (REMAINING-WORK
  row 4, build-order :1192) does not exist on any robot program (§0.2); `TermSink`'s tick line has no rate, the SD copy
  takes no records, and `Shul2Sink` (H1 Part A2) is unbuilt. IMU1 corrects the plan's words (§3.2); adding a rate to
  `TermSink`'s line is a format change with its own readers (guide 11, the worksheet's line examples) — the team lead's
  call, not IMU1's. H1 Part A2's SHUL/2 sink will carry the field, right since IMU1.
- **`heading()`, `pitch()`, `roll()`** — untouched: none keeps derivative state; a second read in a tick returns the
  same buffered value or a fresher one, and moves nothing.
- **The host test plant** (`sim/`) — `FakeImu` stays stateless and the plant's IMU is not made sample-and-hold: that
  would move every host suite's numbers. HA-213's model lives only in IMU1's new test harness.
- **`faultedReads()`'s per-read counting** (DOCS2's A18, REJECTED) — beyond what the hold changes (§1.1), unchanged.
- **Re-running TANK1's probe grids with the IMU model** (the 24 563-leg grid) — HA-179's re-run belongs to R5 with the
  measured motor lag; T8 covers the two TANK1 runs and the lag values TANK1 itself used.
- **A team's own loop, or any reader outside the library's tick** — documented (the changelog's cautions and
  `kRateHoldUs`'s doc: a loop whose period is not a multiple of 10 ms reads 0 on a large share of reads; a read
  between ticks spoils the next two — re-check R16), not designed for.
- **The Localizer reading its rate at the top of `update()`, ahead of the correctors, KEPT WITH (a)'s hold** (the
  cure for limit (2) on a corrected robot — re-check R15): a one-line move in `localization/localizer.hpp`
  (byte-identical on the host, where every read in a tick has one value). Not now: no robot is GPS- or tag-corrected
  this season (§6), and `localizer.hpp` is L6-A1's file. It belongs with the first corrected robot's chunk, and
  T9 (b) is the test that shows it working (gate == Localizer stays; the zero / double counts go to 0).
- **The calibration classes** (`rest_capture.hpp`, `drive_characterizer.hpp`, `drivetrain_calibrator.hpp`) — no edit:
  the adapter fixes their reads (§0.3 rows 9–11); T10 and T11 prove two of them.

---

## §6 Coordination — the work in flight, the shared files, who waits for whom

Re-checked 2026-10-22 against the worktrees' `git status` (read only):

| In flight | Where | Its files (code) | Shared with IMU1 | Who waits |
|---|---|---|---|---|
| **MECH-A3** | worktree `mech-a3`, on `fbaf729` | `mechanism/{service,kinds,spec,wiring}.hpp`, `sim/mechanism_plant.hpp`, `test/mechanism_*` (+ new position/speed tests, a compile refusal), `docs/guide/18-mechanisms.md` (new) | **No code file.** Docs: `docs/changelog.md`, `docs/hardware-assumptions.md` (its count line), `build-order.md`, `REMAINING-WORK.md`, `HANDOFF-remaining-work.md`, `PROJECT-BRIEFING.md`, `docs/api/README.md` + `all-entities.md` (generated indexes) | Nobody waits. Whichever merges second rebases those documents: its changelog section goes ABOVE the other's (newest first), it re-counts the register's count line, and it regenerates `docs/api/` and the briefing from a fresh build |
| **MECH-A4** (next) | not started | `chassis/routine.hpp` (mechanism steps, API 2.5), `competition/{auton_registry,match_runner}.hpp`, `teleop/teleop_loop.hpp`, `runner/…`, `version.hpp` | **No file IMU1 edits.** IMU1's T8 CONSUMES `chassis::Routine` (the acceptance run's four legs) — additive steps cannot break it; T8 writes the four legs inline (citing `competition/acceptance_run.hpp` :41-44) rather than calling `acceptanceRun(AutonContext&)`, whose context MECH-A4 grows | Nobody waits; if MECH-A4 lands first, IMU1's executor re-runs T8 on the tip (it must still compile and pass unchanged) |
| **CAL1 Part A2a** | worktree `cal1-a2a`, on `ceb17c0` | `calibration/calibrate_program.hpp` (new), `hal/pros/calibrate.hpp` (new), `calibration/calibrate_panel.hpp`, `test/calibrate_program_{rig,test}` | **No code file.** Its rig's `WarmupImu` wraps a `FakeImu` (stateless) — unaffected. The library Calibrate it builds gets its IMU from the provider → a `ProsImu` → IMU1's hold applies on the robot with no CAL1 edit. **Docs (re-check R9, its `git status`):** `docs/changelog.md`, `REMAINING-WORK.md`, `build-order.md`, `HANDOFF-remaining-work.md`, `PROJECT-BRIEFING.md`, `docs/roadmap.md`, `docs/api/README.md` + `all-entities.md` (generated), `mkdocs.yml` (IMU1 adds no page), `README.md`, `ORIENTATION.md` (IMU1 edits neither) | Nobody waits. **Whichever merges second rebases those documents** (its changelog section above the other's, `docs/api/` and the briefing regenerated from a fresh build). **CAL1 Part B's first robot Calibrate** (robot two onto the library Calibrate) should run on an IMU1 build: its REST step's `yawRateSigma` is HA-22's evidence (T10) and its rest gates read the rate (§0.3 rows 9–11) |
| **VIS1-A1** | worktree `vis1-a1`, on `ceb17c0` | `hal/device_registry`, `hal/device_provider` (+ fake and pros), `config/robot_config.hpp`, `robot/robot.hpp`, `hal/pros/device_registry.hpp`, new camera/vision headers, `test/pros_shim/pros/shim_control.hpp` (+ `ai_vision.h`), `test/{boot_source,mechanism_devices,pros_device_registry_adapter,robot_builder}_test.cpp` | **No code file IMU1 edits.** The shim directory, not a file: VIS1-A1 edits `shim_control.hpp` (an include and `resetAiVisions()` in `resetAll()`); IMU1 edits only `pros_shim/pros/imu.hpp` — `resetAll()` already calls `resetImus()` (`shim_control.hpp` :51), which restores the new members' defaults, so IMU1 needs no `shim_control.hpp` edit. `hal/pros/device_provider.hpp` constructs the `ProsImu` (:318) — IMU1 does not touch it. **Tests IMU1 depends on (re-check R9):** `pros_adapter_fence_test.cpp` (+1 line at :14, so landmine 10's IMU block moves :161-175 → :162-176; +10 after :192), and `xdrive_config_test.cpp` and `mechanism_devices_test.cpp` — two of §2.4's nine fingerprint TUs. **Docs:** `docs/changelog.md`, `docs/hardware-assumptions.md` (HA-192..HA-203: the Held line and count line IMU1 also edits), `HANDOFF-remaining-work.md`, `PROJECT-BRIEFING.md`, `docs/roadmap.md`, `docs/api/README.md` + `all-entities.md`, `docs/guide/14-what-it-cannot-do-yet.md`, `mkdocs.yml` | Nobody waits. **Whichever merges second rebases the documents** (changelog section above the other's; the register's Held and count lines re-counted; `docs/api/` and the briefing regenerated from a fresh build). **The fingerprint's base is the merge base Part A starts from;** if VIS1-A1 lands after Part A is built, §2.4's fingerprint is re-run against the new merge base before IMU1 merges |
| **VIS1-B, L6-B, MECH-A6** (later) | — | `src/programs/bench_r3a.cpp`: station 12's functions (VIS1-B), station 11's (L6-B), the menu `kMenu`/`menu_grid`/`bench_r3a.hpp` and station 13 (MECH-A6) | **IMU1 Part B edits station 2's functions only** (`tImu`/`reportImu` and a new helper beside them) — the same partition the three briefs already follow | Nobody waits; the later one re-anchors line numbers by function name. If MECH-A6's 3 × 5 menu lands first, station 2's button keeps its number (MECH-A6's brief keeps 1–12) |
| **The L6 brief** (awaiting the team lead) | — | L6-A1: `localization/localizer.hpp` (a one-axis seat), new wall headers; L6-B as above | **Under D1 (a): none** — IMU1 does not edit the Localizer. **Under D1 (b):** both edit `localizer.hpp` (IMU1 moves the rate read to the top of `update()` and publishes it; L6 adds a seat) — then the second to land rebases, and IMU1 (b) should land first (smaller) | **HA numbers collide** (§4) — the coordinator assigns at the plan commit |
| **H1 Part A2** (`Shul2Sink`, unbuilt) | — | `diag/shul2_*` | None. The first robot path that will carry `imuYawRate`; right from the day it lands if IMU1 is in | H1 A2 need not wait; its first robot log is evidence for the rate only on an IMU1 build |
| **Robot sessions** | the team | Station M (D4), Station K (K.5's floor runs re-use M.6's prediction), R5a's REST/GAINS session (S2/S3) | — | **M.6 and K.5's floor runs, and any REST capture meant as HA-22's evidence, wait for an IMU1 build** (D4 (a)); M.0–M.5 and K's other steps do not. Any GPS- or tag-corrected robot's tank `moveTo` waits (build-order's DoD) — none exists this season (robot two has neither; robot one is unbuilt) |

**IMU1's own order:** Part A, verified and committed, then Part B (Part B's worksheet text quotes Part A's constant;
its tick-span stand-in is the device reads between a motion tick's two IMU reads — R10). Part B needs no part of Part A to
compile — if the team lead wants the bench numbers first (to settle HA-213 before trusting the model), B can run first;
the brief's recommendation stays A first, because A is the fix and B only measures how big the residual is.

**What the coordinator does at the plan commit:** resolve the HA block (§4); add IMU1's block to the Held line; copy
this brief to `docs/internal/chunks/`; move REMAINING-WORK row 32 to "briefed".

---

## §7 Checks — the coordinator re-runs every one after each part (RESUMING's step 3: never the report's word)

**Every binary — the suite, a probe, a mutant — runs as `(ulimit -v 4194304; timeout 300 <command>)`** (the 2026-10-15
rule; `tools/suite_run.py` applies the same bounds once it has landed). Scratch files under
`.claude/coordinator/scratch/`, never `/tmp`, never the repository.

**Part A**
1. **The suite:** `cmake --build build/test -j"$(nproc)"`, then the suite under the cap — green, and the count of new
   cases matches the log.
2. **Reproduction first, re-seen:** in a scratch copy of the merged tree with `ceb17c0`'s `include/shulib/hal/pros/imu.hpp`
   put back AND `test/imu_rate_hold_pin_test.cpp` deleted (it names the new constant — R2), the suite shows T1, T2,
   T3 (a), T4, T5 (a), T6, T8 (a–e), T9, T10 (and T11 if built) RED with the counts the log recorded; restored,
   green.
3. **The mutations:** M1, M3, M6 and M11 at least, each in a scratch copy, each red under its named tests.
4. **The fingerprint (§2.4, re-check R1):** `(ulimit -v 4194304; timeout 300 setarch -R stdbuf -oL
   ./build/test/shulib_tests -s --order-by=file) 2>&1 | python3 -I
   .claude/coordinator/scratch/brief-IMU1/tools/case_hashes_v2.py "$PWD"`, run from each tree's root — the merge
   base TWICE (its noise set) and the merged tree once; `python3 -I …/tools/cmp_case_hashes.py` the pairs — only the
   noise and the named cases differ (S6; the two P3-T3 source scans; the adapter test's gyro case; the new IMU1
   cases). No other suite run on the laptop at the same time. The `-s` output is ~500 MB through a pipe — never to a
   file under `tools/suite_run.py`'s 256 MiB output bound.
5. **The CI guards:** the PROS-free guard and the sim-layering guard (`.github/workflows/ci.yml` :58-82) and the compile-refusal step (:121 — IMU1 adds no refusal, it must stay green); the fence
   test (`test/pros_adapter_fence_test.cpp`) is in the suite.
6. **The ARM header gate:** RESUMING's all-headers cross-compile, its `all.cpp` under the coordinator's scratch.
7. **The src build gate:** `python3 tools/src_build_gate.py check` (all seven builds, 0 disallowed warnings — the
   adapter is in every robot build through the provider) and `self-test`.
8. **The doc gates:** `python3 tools/api_doc_tool.py self-test`, `check-coverage`, `check-fresh`, `check-examples`,
   `check-removability`; `python3 tools/briefing_status.py self-test` and `check` — **`generate` only after a FRESH
   build of the merged tree** (never against a stale build); `python3 tools/doc_staleness_audit.py self-test` and the
   audit itself.
9. **Read the diff of `include/shulib/hal/pros/imu.hpp` line by line** against §1.1: the clock read before the device
   read; the hold measured from the last FRESH answer; a faulted read opening the hold without moving the sample; the
   `dt <= 0` guard kept with its comment; the banner's and docs' words (§1.1's list) present and true.
10. **The documents:** §3's list present; the prose-review greps (§3.4) logged with their hits; the pointers (rule 5).

**Part B**
1. `make ROBOT=tank` builds (from `rm -rf bin .d`); the src gate's seven builds and its object check (no stdio in
   `src/` objects — SER1's).
2. **The stage commands no motor:** `grep -n "move_voltage\|move_velocity\|move(\|setVoltage\|brake(" ` over the new
   STAGE 2c code returns nothing; it reads `imu_get_rotation`, `imu_get_gyro_rate`, `micros()`, the drive motors'
   `motor_get_*` and the battery, and paces with `pros::Task::delay_until` — nothing else; it never calls
   `ProsImu::yawRate()` (R18); the diff adds no `#include` (no `tick_pacer.hpp`, no `localization/` — R10).
3. The stage edits only station 2's functions (§1 item 5) — `git diff` of `bench_r3a.cpp` touches no `kMenu`,
   `menu_grid` or `bench_r3a.hpp`.
4. The worksheet station's steps match the stage's printed lines word for word (the executor quotes them from the
   source, not from memory).
5. Nothing claimed settled: HA-213..206, HA-04, HA-109 stay unsettled until the bench prints their numbers.

---

## §8 Landmines

1. **Write the tests against TODAY's adapter first, and see them red.** A test that only ever ran on the fixed adapter
   proves nothing about the defect. The reds and their counts go in the log BEFORE `imu.hpp` changes (§2.5).
2. **The model must be unkind (R0).** If the hook returns the TRUE rotation at the read instant (a continuous sensor),
   today's adapter looks almost right and the reproduction is hollow. Sample-and-hold, every 10 ms, at a phase that is
   swept — and R0 proves the model holds.
3. **The creeping clock is the IMU's ONLY.** T8's `ProsImu` gets its own clock (the plant's time plus `gap` per device
   read, back to the plant's time at each pace — probe 2's `CreepClock`). Moving the PLANT's or the scheduler's clock
   inside a tick changes every motion's `dt` and breaks T8 (a)'s byte-identity for the wrong reason.
4. **Read the clock BEFORE the device, and use that one `now` for both the hold check and the differentiation.** Reading
   it after the device (today's order, :180 then :185) puts the device's latency inside the answered instant; two
   `now`s let the hold and the derivative disagree about which tick a read belongs to.
5. **The hold runs from the last FRESH answer, never from a held read** (M11, T3 (b)). Restarting it on every read
   freezes the rate for ever under any reader that asks faster than every 5 ms — a frozen reading, which is worse than
   today's failure.
6. **A faulted read opens the hold but never moves the differentiation sample.** Opening it keeps a dead sensor to one
   device read and one count per tick (T5 (a)); moving the sample would make the next good tick differentiate across
   the gap from a sample that does not exist (M7 — T5 (b), T7).
7. **Keep the `dt <= 0` guard** (:195-196), with a comment that the hold makes it unreachable while `kRateHoldUs > 0`. A
   precondition there would throw on the robot for a defensive branch.
8. **The first call still answers 0** (:186-192), and a read within 5 ms of it answers 0 too (held). T7's oracle pins
   the first; nothing may "improve" it.
9. **Do not hold `heading()`.** It keeps no derivative state; holding it would delay the estimate by up to a tick for
   nothing and let heading and rate disagree across a copy. Pitch and roll likewise (§5).
10. **The fence test reads `imu.hpp` as text** (`test/pros_adapter_fence_test.cpp` :156-176 at `ceb17c0`, the IMU block
    :161-175; VIS1-A1 moves it +1 — R9): with line comments
    stripped, `tare(`, `tare_rotation(`, `set_rotation(`, `tare_heading(`, `set_heading(` and `get_heading(` must not
    appear. New `///` text is safe; a string literal or a block comment naming them is not.
11. **`ImuPortState` is an aggregate** (`test/pros_shim/pros/imu.hpp` :51-60): new members need default member
    initializers (`= nullptr`, `= 0`) so `resetImus()` (`imuPorts() = {}`, :68) restores them and every existing suite
    keeps its behaviour with the hook unset.
12. **The fingerprint is per test case, line-buffered, after `setarch -R` and `case_hashes_v2.py`'s masks** (§0.4
    probe 4, re-check R1). A whole-run hash differs on every run; ASLR moves printed pointers; without `stdbuf -oL`
    the serial sink's raw `write()` interleaves with doctest's buffered stdout differently in every changed binary;
    the old tool hashed line numbers and gave each case's `path:line:` header to the case before it. The base is run
    twice and its noise excluded. A case that differs outside §2.4's named set is investigated — a suite that steps a
    real `ProsImu`'s clock by under 5 ms between rate reads would change legitimately (only §2.4's nine TUs can), and
    is then named in the log with why.
13. **`kRateHoldUs` is pinned to the tick, not to a number** (T3 (c)): `ProsTickPacer::kTickMs` is a `std::uint32_t` of
    milliseconds and `kRateHoldUs` a `std::uint32_t` of microseconds — compare them in one integer expression with the
    conversion written out (`kRateHoldUs * 2 == kTickMs * 1000`), so a 5 ms tick (or a 20 ms one) moves the hold or
    fails the build's test. **And compare TIME in whole microseconds** (`std::llround(… * 1e6) < kRateHoldUs`), never
    `units::Time` against `units::Time{0.005}`: in doubles the exact 5 ms edge is held or re-sampled by rounding,
    depending on the anchor (re-check R3/R13), and T3 (a) and M2 then mean nothing. The pin lives in its own TU so the
    reproduction-first build (the tree without that TU) compiles against today's adapter (R2).
14. **Words.** "host test plant", never "simulator". No claim about a robot: nothing here has run on one, HA-213 is a
    sentence in PROS's header, and HA-214's phase is unknown. The "about 0 or a spike" is DERIVED, not measured (§0.5)
    — every document IMU1 touches says so where it repeats it.
15. **Part B: one direction, and ask.** STAGE 2c's gyro comparison is relative (the gyro's integrated z against the raw
    rotation's change on the SAME turn, both raw CW degrees — never `ProsImu::yawRate()`, R18), so it needs no
    declared direction — but the cadence and phase parts need a turn that keeps
    changing the reading: a robot held still shows no copies at all (the stage must say "no change seen — turn it" and
    count those ticks, never guess). The file's own 2026-08-19 lesson (`bench_r3a.cpp` ~:846-855: record what the
    operator did, do not assume it) applies to anything STAGE 2c concludes from a hand turn.
16. **Part B's busy-polling** (the phase part polls up to 9.5 ms per tick for 100 ticks) starves lower-priority tasks
    for about a second — acceptable in Bench Tests, which commands no motor, and said on the panel; never copied into a
    program that drives.
17. **Part B shares `bench_r3a.cpp` with three other chunks** (§6). Station 2's functions only; re-anchor by function
    name at the tip; never `kMenu`, `menu_grid` or `bench_r3a.hpp`.

---

## §9 Summary

**The defect.** `ProsImu::yawRate()` differentiates the IMU's rotation between CALLS. The library reads it two or three
times a tick (the Localizer; every motion's record on robot two, whose serial sink wants every record; a GPS or tag
corrector's gate first, on a corrected robot), and on the robot the clock moves microseconds between those reads while
PROS's documented 10 ms copy holds the rotation still. So the second read answers 0 or a ×33 spike — and, missed until
this brief, it moves the baseline, so the NEXT tick's first read (the Localizer's, which TANK1's observer and
`DriveBrake` use) is 3 % high, or, for one band of the copy's phase, 0 on every tick. Reproduced on the host with the
REAL adapter, Chassis, scheduler, Localizer and TANK1's `MoveToPose`: with TANK1's own motor-lag values the 40 in goes
from FINISHED to a 5 s TIMEOUT with 12–17° of wobble in that band. First recorded at DOCS2 (A17), deferred, re-found by
TANK1. Nothing of it is measured on a robot: it rests on PROS's documented 10 ms copy (HA-213) and an unknown phase
(HA-214).

**The fix (D1 (a)).** One file of library code: `ProsImu::yawRate()` keeps its answer for 5 ms (half the 10 ms tick),
so every read in a tick gets the first read's answer and nothing moves the baseline. No frozen signature, no consumer
edited; every reader inside a library tick is fixed at once — a team's own `imu.yawRate()` called from inside one
included (a reader on its own rhythm is not: R16). With one read per tick it is bit-identical to today's adapter on a
clock that stands still during the device read (T7; on the robot each sample's stamp moves by the read's latency, µs —
R4/R17); with records on, the whole 40 in and acceptance run are bit-identical to records off for every copy phase
except the narrow band where a chained-motion boundary's later heading read crosses a copy (T8 (a), R11).

| Part | What | Size (builder-days) | Compared with |
|---|---|---|---|
| **A** | the hold in `hal/pros/imu.hpp`; the shim hook; `test/imu_buffer_model.hpp`; `test/imu_rate_once_per_tick_test.cpp` (R0, T1–T11, written and seen RED on today's code first) and `test/imu_rate_hold_pin_test.cpp` (T3's pin, with the change); M1–M11; the per-case fingerprint (run with `case_hashes_v2.py`); §3's documents (3 public) | **~1¼** (1–1½) | SER1, briefed at ~¾–1 (one header, ~20 `src/` sites, a gate check, ~8 documents) |
| **B** (D3 (a)) | Bench Tests station 2's STAGE 2c (the copy's cadence and phase, a stand-in for the tick's span, the gyro's integrated z beside the raw rotation's change); worksheet Station N (new) | **~½** | TANK1 Part B (~½: one result-line column and its docs) |
| **Both** | | **~1¾** | REMAINING-WORK's rough "~½–1" (row 32) grows by the Localizer finding (§0.2) and the bench stage |

The figures are the executor's build, as SER1's was; review and fix rounds come on top (SER1, briefed at ~¾–1, landed a
week of calendar time later across two pauses and a review round). Robot time for Part B: ~10 minutes at Station N.

**Decisions for the team lead (recommendations):** **D1 (a)** the adapter holds its answer 5 ms; **D2 (a)** the frozen
`IImu` is not touched; **D3 (a)** Part B, a short Bench Tests stage that measures the IMU's cadence, phase, tick span
and the gyro's sign; **D4 (a)** Station M.6 (the floor run on the tape) runs on an IMU1 build, M.0–M.5 need not wait.

**For the coordinator:** the HA block collides with L6's (§4) — resolve at the plan commit (recommended: IMU1 →
HA-213..HA-215 if L6 goes first). No code file is shared with MECH-A3, MECH-A4, CAL1 Part A2a or VIS1-A1 (§6).

**Probes (`.claude/coordinator/scratch/brief-IMU1/`; all run under `ulimit -v 4194304` + `timeout 300`):**
1. **The adapter alone** — today: in the band the Localizer's read is 0 on 200/200 ticks and the record's a −52.36 rad/s
   spike; elsewhere the Localizer's is 3.1 % high and the record's exactly 0; with ONE read per tick, exact. The
   prototype: exact everywhere, B == A.
2. **The real loop** — today with records on, in the band: the Localizer's ω = 0 on every turning tick; the 40 in and
   the acceptance run fail with TANK1's motor lag (TIMEOUT, +12.44° / −17.35°; the acceptance run stops on its first
   leg); far from the band, records 190/190 zero. The prototype: identical to records-off in every row.
3. **Timing hostility** — jitter: fine; the residual (copy at the read instant): 73 zero / 73 double of 300 with or
   without the fix, one read or two — HA-214; a late wake and an in-tick stall: a few ticks off, then exact.
3b. **(2026-10-22) the stall and the late wake separated** — probe 3's case (4) had inherited case (3)'s late tick;
   alone, a 6 ms stall costs the fix two of the Localizer's ticks; a 6 ms late wake three (one held, stale).
4. **The fingerprint** — the `-s` output (~497 MB, ~26 s) hashed per test case is deterministic under `setarch -R` with
   hex addresses masked, except the S6 thread-contention case — for two runs of the SAME binary. Across a base and a
   changed binary it needs `stdbuf -oL` and `case_hashes_v2.py`'s masks (re-check R1); so run, base vs a Part A
   prototype differs in exactly S6, the two P3-T3 source scans and the adapter test's gyro case.
5. **(2026-10-22, the re-check — `revise/`, on a µs-hold prototype `revise/fixus/`)** — T8's phase grid bitwise (R11:
   records on == off except φ ∈ [3,4)·gap at a chained-motion boundary; R12: a third-tick zero at kA 0, records on and
   off alike); T10's REST capture (R14: one doubled sample for φ ∈ (1,2)·gap, σ 2.2e-4 vs 5.6e-7); the first read
   moving (R15: 60 zero / 60 double of 300 with a fresh fix every 5th tick); off-tick readers and own loops (R16: an
   out-of-loop read on 1 tick in 10 → 60 Localizer ticks off, today 30; a 4 ms loop never exact); the 5 ms edge in
   doubles (R3: held or re-sampled by rounding at 704 of 1000 anchors).

**UNVERIFIED — said plainly:**
- **Everything about the robot.** HA-213 (the 10 ms copy) is PROS's sentence, never observed on our brains; HA-214 (the
  copy's phase, fixed or drifting) is unknown; HA-215 (a tick's reads within 5 ms; wakes rarely ≥ 5 ms late) rests on
  HA-32/HA-102, both unmeasured under load. **Whether robot two's phase lands in the bad band — and so whether Station
  M.6 would time out on today's build — is unknown.** "About 0 or a spike" is derived, not measured (§0.5).
- **The probe's model is a model**: sample-and-hold at a swept phase, a fixed `gap` per device read (25 / 100 µs —
  guesses), the host test plant's placeholder dynamics with TANK1's two motor-lag values. The band's width on a robot
  (the real Localizer-to-record gap) is unmeasured; "about 0.5 % of boots" (D4) assumes a 50 µs gap and a fixed phase.
- **Part B's stage is designed, not prototyped**: its polling loop, its timing of a motion-shaped tick body and its
  output lines are written in this brief, not compiled; the executor proves them in the seven builds.
- **The fingerprint's "every other case byte-identical"** is a prediction from reading the nine TUs that include the
  adapter (§2.4), checked once on a prototype (re-check R1) — the executor's diff on the real Part A is the proof.
- **T11's size** (the calibrator's rest gate under ~80 lines with `test/calibration_test_rig.hpp`) is not checked; the
  brief lets the executor log it as not built, with why.
- **Line numbers** are `ceb17c0`'s (and `71b4764`'s, where marked); MECH-A3's and VIS1-A1's merges, and any later plan
  commit, move some documents' lines — the executor re-anchors by text.
- **The L6 brief's state** was read on 2026-10-22 as written on 2026-10-18 (it may also have stopped partway); only its
  §4 numbers and its file list (§6) were used here.
- **(Re-check) the fingerprint's intermittent noise:** in 1 of 6 runs of the same base binary, `ProsDeviceProvider 1r`
  and `ProsFileStore: the whole SAFE SAVE…` differed; the cause is NOT established (both write fixed host temp paths
  that a concurrent suite run would share). §2.4 handles it by running the base twice and re-running a stray case on
  its own — a procedure, not an explanation.
- **(Re-check) the residual counts of limits (2) and (3)** (R15, R16) are the host model's — a fixed `gap`, a constant
  turn, an invented reader schedule; how often a robot meets them depends on HA-214's phase and on what team code
  does, both unknown. The µs-hold the brief now specifies was prototyped (`revise/fixus/`) and run through the
  re-check's probes, not through the suite.
- **(Re-check) Part B's new Station N** — the letter was free on 2026-10-22 (grep of the worksheet, the briefs and
  `docs/internal/`); a brief accepted before IMU1 Part B lands could claim it — the executor re-checks at the tip.
