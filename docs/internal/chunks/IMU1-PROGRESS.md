# IMU1 — progress log (live)

- **Chunk:** IMU1 Part A — the IMU's turn rate is read ONCE per control tick, and every reader in that tick gets
  the same value (brief: [`IMU1-imu-rate-once-per-tick.md`](IMU1-imu-rate-once-per-tick.md); its "Re-check —
  amendments" R1–R18 OVERRIDE its body). **Part B (the bench stage, Station N) is NOT this run's.**
- **Executor model:** opus (Claude Opus 5.5, 1M context), a workflow-spawned executor.
- Appended as the work goes. Every time in a section header is a `date` reading taken when the section was
  written. **Nothing in this chunk has run on a robot; nothing here may claim it did.**

---

## Start

- **Started:** Thu Oct 29 2026 (`date` at the top of this file's creation — see the first section's stamp).
- **Tree at start:** `2ef1f3f`, clean, in the git worktree `.claude/worktrees/imu1-a` (branch `imu1-a`) — NOT
  the main working tree.
- **Go:** the team lead's "Accept all (Recommended)" of 2026-10-22 (D1 (a) the 5 ms hold in `ProsImu` with its three
  named limits; D2 (a) `IImu` untouched; D3 (a) Part B later; D4 (a) IMU1 before Station M's floor run), and his
  words of 2026-10-22/29 ("we are running out of time but that doesnt mean we should cut corners"; "go even
  faster… but be as thorough"). Do not commit; no `pros` command; no `/dev/ttyACM*`. Probes, mutation backups and
  scratch under `.claude/coordinator/scratch/imu1-a/` (never `/tmp`, never the repository root). Every test
  binary under `(ulimit -v 4194304; timeout 600 …)`.
- **In flight beside this chunk (not edited here):** CAL1 Part A2a (worktree `cal1-a2a`), VIS1-A1 (worktree
  `vis1-a1`: it edits `test/pros_shim/pros/shim_control.hpp` — IMU1 edits only `test/pros_shim/pros/imu.hpp` in the
  shim; `resetAll()` already calls `resetImus()`).

### 13:41–13:53 — the reading, the base build and the base fingerprint *(`date` 13:53)*
- Read `docs/internal/RESUMING.md` (the chunk loop, the standards, pace rules 1–8) and the brief per the team
  lead's 2026-10-22 reading ruling: its header and answers block, "Re-check — amendments" R1–R18 in full, the
  Decisions, the plain words, §0 (the defect, the readers, the probes), §1 Part A, §2.1–§2.5, §3.1/§3.2/§3.4, §4,
  §5, §6, §7 Part A, §8 and §9 in full; Part B's sections (§1 items 5–6, §3.3, §7 Part B) by heading only.
- Read the code the brief cites, at `2ef1f3f`: `include/shulib/hal/pros/imu.hpp` (whole), `test/pros_shim/pros/imu.hpp`
  (whole), `test/pros_shim/pros/rtos.hpp`, `hal/pros/clock.hpp`, `test/pros_imu_adapter_test.cpp` (whole),
  `test/motion_test_rig.hpp` (whole), TANK1's P14 rig (`test/motion_tank_steer_test.cpp` :1211-1340), `Localizer::update()`
  (`localization/localizer.hpp` :240-412), `GpsCorrector::propose()` (`gps_corrector.hpp` :198-310), `RestCapture`'s tick
  path (`calibration/rest_capture.hpp` :316-560), `DrivetrainCalibrator`'s tick / rest window / status
  (`drivetrain_calibrator.hpp` :190-198, :559-700, :880-900, :1013-1052, :1064-1110, :1470-1480, :1659-1680, :1736-1760),
  `test/calibration_test_rig.hpp` (whole), `competition/acceptance_run.hpp`, and the brief's probes
  (`.claude/coordinator/scratch/brief-IMU1/`: `revise/t8grid.cpp`, `revise/rest_t10.cpp`, `revise/shift_first.cpp`,
  `probes/probe_adapter.cpp`, the µs-hold prototype `revise/fixus/`, the fingerprint tools).
- **Deltas from the brief's base `ceb17c0` to mine `2ef1f3f`** (`git diff --stat ceb17c0 2ef1f3f` over every code file
  the brief cites): only `include/shulib/chassis/routine.hpp` (MECH-A4's mechanism steps, +486 lines, additive) and
  `test/CMakeLists.txt` (+2, the `suite_run.py` gate dependency). *(CORRECTED at the review, 2026-10-29 — this
  sentence was FALSE: `teleop/teleop_loop.hpp`, `sim/mechanism_plant.hpp`, `version.hpp` and three tools the brief
  cites changed too; the full re-check, and each delta, is in "IMU1-A review findings and their disposition",
  finding D-7, below.)* `hal/pros/imu.hpp`, the shim's `imu.hpp`,
  `pros_imu_adapter_test.cpp`, `motion_tank_steer_test.cpp`, `move_to_pose.hpp`, `debug_record.hpp`, `tick_pacer.hpp`,
  `calibration/`, `localization/`, `motion_scheduler.hpp`, `acceptance_run.hpp`, the fence test and
  `drive_program_source_test.cpp` are byte-identical to `ceb17c0` — so every code `file:line` the brief cites holds at my
  base. (`test/CMakeLists.txt` :79's GLOB is unchanged — `CONFIGURE_DEPENDS` — so a new `*_test.cpp` needs no CMake edit;
  IMU1 makes none.) Document line numbers are re-anchored by text when edited (below).
- **The base build** (`cmake -S test -B build/test`; `cmake --build build/test -j8`): the first attempt stopped at
  the doc gate because this log is a new open-chunk file the committed briefing did not name — `rm -f
  build/test/shulib_tests`, `python3 tools/briefing_status.py generate` (the binary absent, so read as unbuilt), then
  the build: 46 s, exit 0. `(ulimit -v 4194304; timeout 600 python3 tools/suite_run.py green)` → **green — 2626 cases,
  2799446 assertions** (the base).
- **The base fingerprint (§2.4, re-check R1):** the tool copied to my scratch
  (`.claude/coordinator/scratch/imu1-a/case_hashes_v2.py`, `cmp_case_hashes.py`); the base binary copied to
  `scratch/imu1-a/shulib_tests.base` and run TWICE as `(ulimit -v 4194304; timeout 600 setarch -R stdbuf -oL
  …/shulib_tests.base -s --order-by=file) 2>&1 | python3 -I case_hashes_v2.py "$PWD"` from the worktree's root →
  `fp-base-1.txt`, `fp-base-2.txt`, 2626 cases each. **Base vs base: 1 differs — `screen (S6): the owner under
  contention — every successful handover moves the generation`** (the noise set; the intermittent `ProsDeviceProvider
  1r` / `ProsFileStore` pair the brief saw once in six did not appear).

### 13:53–14:02 — the shim hook, the model, the tests; REPRODUCTION FIRST on the UNCHANGED adapter *(`date` 14:02)*
- **`test/pros_shim/pros/imu.hpp`** — `ImuPortState` gains `double (*rotationAt)(int port) = nullptr` (consulted
  after the disconnected / calibrating screens) and `rotationReads` / `gyroReads` (every call that reached the port,
  a screened one included — so a held read that still touched the device is visible even on a dead sensor; the
  brief's prototype counted only good reads); the banner's "Beliefs modeled" list gains the 10 ms copy (HA-213,
  vendored `include/pros/imu.hpp` :150-156 — re-opened: the brief's citation holds). All three have default member
  initializers (landmine 11), so `resetImus()` restores them and every existing suite keeps its behaviour.
- **NEW `test/imu_buffer_model.hpp`** — `BufferModel` (the 10 ms copy at a swept phase; a device read takes `gap` and
  the IMU's clock creeps by it, returning to the plant's at each tick — landmine 3), `ScopedInstall` (the shim hook is a
  plain function pointer, so ONE active model), `CreepClock`, `PlantHistory` (truth from the plant's per-tick state for
  the loop tests) and `SpyImu` (an `IImu` that forwards and records every `yawRate()` answer with its tick, so the
  Localizer's reads, a record's and a gate's can be told apart while ONE `ProsImu` serves them all, as robot two wires
  it). Banner: it models BELIEFS (HA-213..HA-215), "host test plant".
- **NEW `test/imu_rate_once_per_tick_test.cpp`** — R0, T1, T2, T3 (a), T3 (b), T4, T5, T6, T7, T8, T8 (a′), T9 (a),
  T9 (b), T10 (a), T10 (b), T11: **16 cases**. It names no symbol the fix adds (re-check R2). T11 BUILT (the brief's
  "if under ~80 lines with `calibration_test_rig.hpp`" — it is ~60 lines, the rig's `runWith` driving a second
  `DrivetrainCalibrator` on the `ProsImu`).
- Build: `cmake --build build/test -j8` → exit 0, no warning. `git diff --stat include/` → **empty: the adapter is
  `2ef1f3f`'s (= `ceb17c0`'s)**. Run: `(ulimit -v 4194304; timeout 600 ./build/test/shulib_tests -tc="IMU1*")` →
  **16 cases: 3 passed, 13 failed; 1353 assertions, 813 passed, 540 failed** (output kept at
  `scratch/imu1-a/repro-1.txt`). Per case, TODAY:
  - **R0 green, T3 (b) green, T7 green** — as the brief states (the model is not the bug; today re-samples every read,
    so a fast reader is never frozen; one read per tick is exactly the old adapter).
  - **T1 RED** — 63 of 63 grid rows (gap 25/100/400 µs × 2/3/5 reads × 7 phases): later reads ≠ the first on 199–200
    of 200 ticks with 2 reads, 200–398 of 400 with 3, 200–796 of 800 with 5 (0 or a spike — probe 1).
  - **T2 RED** — 84 checks: with 2 or 5 reads a tick the first read is off the truth on 199 of 199 ticks at EVERY
    phase and gap (21 rows each), and the first-read sequence differs from the 1-read run's (21 + 21); with ONE read a
    tick it is exact everywhere (`off1` never failed — the model is right, the second read is the bug).
  - **T3 (a) RED** at all three anchors (0, 10 000, 1 234 567 µs): the +4 999 µs read re-sampled (`rotationReads`
    2→… moved, the value changed), so the +5 000 µs read's checks failed too.
  - **T4 RED** — DifferentiateRotation `rotationReads` 400 (want 100), GyroRateZ `gyroReads` 400 (want 100), and
    `faultedReads()` 40 on each (want 10: four screened reads per dead tick).
  - **T5 RED** — (a) the second read on the dead tick counted a second fault (2, want 1) and reached the device (8 vs
    7); (b) the next good tick was off the truth (today's tick-2 second read had moved the sample) — the brief claimed
    only (a) red; (b) is red today too under the creeping clock, logged.
  - **T6 RED** — the second read in the tick answered the new z (and `gyroReads` 2, want 1).
  - **T8 RED** — 60 of 60 grid rows (kA 0/0.02/0.03 × gap 25/100 µs × φ {2.25, 2.5, 2.75, 20}·gap and 4.5 ms × the
    40 in and the acceptance run): (a) records on ≠ off in every row; (b) 288–1 217 reads off the tick per run;
    (c) in-gap, the Localizer's ω is EXACTLY 0 on **152 / 150** turning ticks of the 40 in at kA 0 (gap 25 / 100 µs),
    **486–487** at kA 0.02 and 0.03, 505 on the acceptance run at kA 0 — the brief's probe said 150–152 and 485–486;
    (d) all 3 boundaries mismatched in all 30 acceptance rows records off, 24 of 30 records on; (e) **24 runs did not
    finish** — every in-gap row at kA 0.02 and 0.03, both runs (the 40 in TIMES OUT; the acceptance run stops).
  - **T8 (a′) RED** — the 40 in on ≠ off in 18 of 18, the acceptance run's prefix before the first boundary ≠ in 18
    of 18; reads off the tick in 36 of 36; 24 runs unfinished (kA 0.02/0.03 — [3, 4)·gap is also between the
    Localizer's read and the record's rate read today).
  - **T9 (a) RED** — the gate and the Localizer disagree on 304–305 of 305 ticks in 3 of 4 rows (scale 1.2 at φ 80 µs
    and 5 ms; 0.8 at 80 µs — at 0.8 / 5 ms both "do not reject", 0 vs 0.8×, so they agree by accident); at 1.2 / 80 µs
    the gate never rejects at all (it reads 0).
  - **T9 (b) RED** — the gate's value ≠ the Localizer's ω on 60–61 of 61 fresh ticks, all 4 rows.
  - **T10 (a) RED** at every φ — σ **4.48e-2** rad/s in-band at gap 25 µs (1.10e-2 at 100 µs), **2.25e-4 far** (5 ms)
    against the 1e-5 bound; the mean off by more than 0.1 % in-band (the brief: σ 4.5e-2 in-band, 2.25e-4 far).
  - **T10 (b) RED** — no doubled sample (0, want 1) and σ 4.48e-2 / 1.10e-2 (> 3e-4).
  - **T11 RED at every φ** (0.5, 1.5, 2.5, 3.5, 20 gaps and 5 ms): `status().atRest` was true on 301 SideAssign ticks
    off the band; IN the band (φ 2.5·gap) the rest window COMPLETED and a pulse STARTED while the robot turned (120
    powered motor-ticks; the run ended on another refusal — code 9 naming port 11 — not `RestTimeout`) — the defect,
    by the two routes re-check R8 names.
- **One delta from the brief, in T8 (c):** each motion's first tick is found from the `Routine`'s step boundaries (the
  pacer's tick count after each blocking verb — TANK1's P14 way), not from the records' `activeCommandId`: the
  records-OFF runs have no records, and (c) is asserted on both. The two are the same instants (a blocking verb
  returns on its exit tick, and the next verb's first tick runs in that same tick).
- **One delta in T9's rig:** a GPS fix and the Localizer's own rate read are at the SAME device-read position in a
  tick (the heading read is the only IMU read before either), so with the clock creeping only per IMU read the gate's
  read and a stale tick's Localizer read would be the same instant and limit (2) could not appear. The rig wraps the
  `GpsCorrector` in `WorkSpan`, which adds `work` = 60 µs to the IMU's clock after `propose()` returns — the
  arithmetic between the gate's read and the Localizer's (the latency carry, the gate, the fusion). The band is then
  (2·gap, 2·gap + work) = (50, 110) µs at gap 25 µs; φ = 80 µs is in it, 5 ms far.

### From 14:02 — the fix, the pin, the adapter test's gyro case, the comments; green *(appended at `date` 14:48)*
- **Backup** of the unchanged adapter: `scratch/imu1-a/imu.hpp.base` (== `git show 2ef1f3f:include/shulib/hal/pros/imu.hpp`).
- **`include/shulib/hal/pros/imu.hpp`** (the only library CODE change, §1 item 1):
  - `public: static constexpr std::uint32_t kRateHoldUs = 5000;` with its `///` (why 5 ms, the pin, HA-215's limits,
    HA-213's loop caution) and `static_assert(kRateHoldUs > 0, …)`.
  - `yawRate()`: `clock_->now()` FIRST; `if (rateAnswered_ && std::llround((now - rateAnsweredAt_).value() * 1e6) <
    static_cast<long long>(kRateHoldUs)) return lastRate_;` (held — no device read, nothing moves); else
    `rateAnswered_ = true; rateAnsweredAt_ = now;` and dispatch with `now` — over BOTH sources. `long long` on both
    sides on purpose: on the ARM target `long` is 32-bit, and a `long` vs `uint32_t` comparison would be a
    sign-compare under `-Werror`.
  - `differentiatedRate(units::Time now)`: the clock read gone from it (landmine 4); a faulted read answers
    last-good, counts once, and never moves the sample (a comment says so); the `dt <= 0` guard KEPT with a comment
    that the hold makes it unreachable while `kRateHoldUs > 0` (landmine 7). The first call still answers 0.
  - Two new `mutable` members (`rateAnswered_`, `rateAnsweredAt_`) under the "state OF THE READ" note, which names
    the hold.
  - Text: the banner gains "ONE ANSWER PER CONTROL TICK" (the defect as DERIVED, why 5 ms, whole microseconds, both
    sources, a held read never extends the hold) and "THE HOLD'S THREE LIMITS" (a stall or late wake — HA-215; the
    first read moving — HA-214; a reader outside the tick, and a loop not a multiple of 10 ms — HA-215/HA-213) and
    the residual NOT cured (HA-214) with its cure (the gyro once HA-04/HA-109 are measured); "HA register:" gains
    HA-213..HA-215; the class doc says it answers once per tick and keeps "One owner, the loop; not safe to read
    concurrently" (the hold adds state under the same rule); `yawRate()`'s doc loses "Call it once per tick";
    `faultedReads()`'s doc says a held read is not counted. The fence test's banned names appear in no string
    literal or block comment (`//` and `///` only — landmine 10).
- **NEW `test/imu_rate_hold_pin_test.cpp`** — T3 (c), `ProsImu::kRateHoldUs * 2U == ProsTickPacer::kTickMs * 1000U`
  in integers, in its own TU (re-check R2), written in the same step as the adapter change.
- **`test/pros_imu_adapter_test.cpp`** (the gyro case, §1.4): the second z is now read a tick (10 ms) later, and a
  read at the same instant answers the first (`-0.785…`, held). No other case edited (:123-137's "same tick again"
  already expected the hold, as the brief said).
- **Comments only:** `include/shulib/motion/move_to_pose.hpp` :282-285 (the observer reads `loc.twist()` — the
  estimate's own value, one source of truth, pinned by P14; the "second read is ~0 or a spike" reason now past
  tense, "ProsImu now answers once per tick") — kept to the SAME line count so `docs/api/move_to_pose.md`'s line
  anchors do not move (a first wording added two lines and moved them: rewritten, the page restored).
  `include/shulib/diag/debug_record.hpp` :149 — `imuYawRate`'s doc: "the IMU's answer for this tick — the same value
  every reader saw that tick, the Localizer's included (one answer per tick since IMU1)" (a 3-line `///` block, so
  the page's later anchors move by 2 — regenerated). `test/motion_tank_steer_test.cpp`: `ProsShapedImu`'s doc (:1215)
  says it models the PRE-IMU1 second read and leaves out the next tick's first read, and that IMU1's T8 runs the
  scenario through the real adapter; the rig's note (:1261) and the boundary comment (:1354) say "fixed in ProsImu
  by IMU1"; P14's "Bug caught" (:1305) past tense; P14's assertions untouched.
- **First green run on the fix:** `-tc="IMU1*"` → 17 cases, 16 passed, **1 failed: T9 (b)'s `gateDisagrees == 0`**
  (244 / 244 / 61 disagreements). Investigated: that assertion was MINE, not the brief's, and wrong for (b) — at a
  fresh fix every 5th tick the gate is consulted only on fresh ticks, so on a stale tick "the gate rejected" is
  false whatever the Localizer's ω is. Fixed in the test (the agreement is counted on the ticks the gate READ the
  rate, i.e. fresh ticks; in (a) every tick is fresh, so (a) is unchanged); the brief's own (b) assertion
  (`freshValueDiffers == 0`) passed throughout. Then `-tc="IMU1*,ProsImu*"` → **26 cases, 26 passed, 1375
  assertions**. T9 (b)'s recorded residual (limit 2, `-s` MESSAGE lines): **φ 80 µs — exact 180, zero 60, double 60
  of 300** at scale 1.2 and 0.8 (the brief's prototype: 180 / 60 / 60); φ 5 ms — exact 300 of 300.
- **The doc gates** stopped the first build twice, as designed: `docs/api/` stale (regenerated: `pros-imu.md`,
  `debug_record.md`, `README.md`, `all-entities.md` — 4 960 → 4 961 entities, the new `ProsImu::kRateHoldUs` row),
  then the briefing (regenerated with `build/test/shulib_tests` REMOVED first, so it read "not built"; then built;
  then the suite; then regenerated with the fresh binary; then built again — the 2026-10-15 rule).
- **The whole suite:** `(ulimit -v 4194304; timeout 600 python3 tools/suite_run.py green)` → **green — 2643 cases,
  2800815 assertions** (base 2626 / 2799446: +17 cases = the 16 in `imu_rate_once_per_tick_test.cpp` + the pin).

### The documents (pace rule 3), re-anchored by TEXT at `2ef1f3f` *(appended at `date` 14:48)*
Public (§3.1 — three, the cap ~5):
- **`docs/changelog.md`** — a new section at the TOP of `## API 2.5`, above MECH-A4's 2.5 entry: `### Fixed since
  2.5 (2026-10-29) — the IMU's turn rate answers once per control tick` (what changed; the one new constant;
  breaking or additive — a behaviour change in an adapter, no version bump, F13/F4; what you must do — nothing
  inside a library tick, the two cautions of re-check R16; not fixed — HA-214's residual). **Delta from the brief:**
  it said "at the top of `## API 2.4`" — written before MECH-A4 made the API 2.5; IMU1 does not bump the API, so the
  entry heads 2.5's section as "Fixed since 2.5".
- **`docs/hardware-assumptions.md`** — the Index gains HA-213/214/215 rows (after HA-191); a NEW section "### IMU1's
  additions — the IMU's 10 ms copy and the timing the rate hold rests on (2026-10-29)" after MECH-A3's addition, the
  three entries in the register's form (Claim · Source with "A4 register HA-nn" at the code site · Confidence ·
  Settle · Blast radius), worded from the brief's §4 table as AMENDED by R14–R16; the status block gains IMU1's
  paragraph after MECH-A3's ("With them, 181") and "Status: 7 of 181 settled" (was 178). The Held line already holds
  HA-204..HA-215 (the plan commit) — unchanged ("a number that lands as a row may stay in its range"); the next free
  number stays HA-216 (the briefing reads it: "181 registered, 7 settled — next free is HA-216"). AMENDED: **HA-179**
  ("The IMU read:" rewritten, dated — robot two's Localizer rate was not clean before IMU1; T8's numbers; M.6 counts
  only on an IMU1 build), **HA-22** (dated — a pre-IMU1 REST record is not its evidence; T10's numbers; the one
  doubled sample on an IMU1 build, far below the claim), **HA-102** and **HA-32** (dated — the hold groups a tick's
  reads by time). **HA-04 and HA-109 NOT amended** — their *Settle* gains STAGE 2c, which is Part B's (§3.1 item 2
  says "Part B only").
- **`docs/api/`** — regenerated (`python3 tools/api_doc_tool.py generate`): `pros-imu.md`, `debug_record.md`,
  `README.md`, `all-entities.md` — exactly §3.1 item 3's four (re-check R5). `move_to_pose.md` NOT changed (the
  comment kept its line count).
Deliberately NOT touched (public), each re-checked at the tip: `docs/guide/` (no chapter describes the rate's
per-call behaviour — the §3.4 greps below), `docs/faq.md` :63-65 (still true), `docs/vexbuilder-integration.md`
:607 (the `imuYawRate` field's offset — meaning and place unchanged; the brief's :559 is now :607),
`docs/guide/14-what-it-cannot-do-yet.md` (HA-214's residual is an unmeasured belief the register carries — the team
lead's call), `docs/api/imu.md` (`IImu` unchanged, D2 (a)), `docs/roadmap.md` (no IMU1 row; F4/F13 rows unchanged).

Internal (§3.2):
- **`docs/internal/build-order.md`** — the IMU1 section: a new status line "2026-10-29: PART A BUILT … awaiting the
  coordinator's verification"; its body's "every `imuYawRate` a robot's record carries is about 0 today" made true
  ("would carry … no robot program prints the field today", dated to the brief's §0.2) and the missed half added
  (the baseline move; robot two's Localizer; the timeout); the DoD's last clause re-worded to D4 (a) (M.6 as
  evidence only on an IMU1 build; M.0–M.5 need not wait). The deviations-table row gains "— which on robot two was
  itself damaged by the record's read (IMU1's brief §0.2)". **The `**Next:**` block NOT moved** — IMU1 is not the
  named next chunk there (the coordinator's pointer; Part A is not committed).
- **`docs/internal/REMAINING-WORK.md`** — row 4 and the Station M bullet (:170): "Its floor run (M.6) counts as
  evidence for HA-179/HA-180 only on a build with IMU1 (#32); M.0–M.5 need not wait" (D4 (a)); row 32: "Part A
  BUILT 2026-10-29 … awaiting the coordinator's verification; Part B not started"; the totals' IMU1 line "~½–1
  (rough)" → "~1¼–1¾ (its brief)". **The plan-of-record total (~26¼–32¼) NOT recomputed** — a shared line the
  coordinator owns (it was not an exact sum of its rows before either); flagged for the coordinator.
- **`docs/internal/HANDOFF-remaining-work.md`** :603 — "(IMU1's, not TANK1's; TANK1 immune)" gains the dated
  correction ("corrected 2026-10-22: not immune on robot two, whose record read damaged the Localizer's rate (IMU1's
  brief §0.2); IMU1 Part A BUILT 2026-10-29").
- **History records — a dated pointer, never a rewrite:** `chunks/TANK1-COMPLETED.md` :67-68 and
  `chunks/DOCS2-API-DEFECTS.md` A17 (its disposition) each gain one dated pointer to IMU1's live log ("its
  completion record after Part B"). TANK1-PROGRESS and TANK1's brief left as written.
- **The worksheet, Station M** (`chunks/R3a-BENCH-WORKSHEET.md`): the brief's D4 (a) paragraph "M.6 needs an IMU1
  build" before M.6.1 (the brief's words; "IMU1 (2026-10-__)" filled as "Part A built 2026-10-29"), and M.6.4's table
  gains the first row `| build hash (M.2.2), and IMU1 in it? | | | |`. **For the coordinator:** the paragraph tells the
  team to check `git log --oneline | grep IMU1` — the commit that lands Part A must carry "IMU1" in its subject.
  *(WITHDRAWN at the review, 2026-10-29: that check matches two plan commits on every pre-IMU1 tree; the worksheet
  now greps the tree for `kRateHoldUs` — finding C-1/D-1, below.)*
- **`docs/internal/PROJECT-BRIEFING.md`** — regenerated after a fresh build (OPEN CHUNKS gains IMU1; the suite line;
  181 registered, next free HA-216).
- **`IMU1-COMPLETED.md` NOT written** — IMU1 is not closed: Part B remains (the brief's §3.2 "at close"; the same
  pattern as L9 Part A1 and H1 Part A1, which landed as parts with a live log only).

### The prose review (rule 4) — the greps RUN FIRST, every hit read, then this written *(§3.4)*
`grep -rIl --exclude-dir=api "<term>" docs README.md` (chunk records under `docs/internal/chunks/` excluded from the
file list — they are the records of what was believed; the ones IMU1 edits are named above), then
`grep -nI` on each hit file and every hit line read. Terms: `ProsImu`, `pros-imu`, `yawRate`, `yaw rate`,
`imuYawRate`, `once per tick`, `differentiat`, `second read`, `read twice`, `immune`, `IMU1`, `kRateHoldUs`, `A17`.
Hits outside IMU1's own edits, and their disposition:
- `docs/roadmap.md` :241, :1098, :1116 (`ProsImu` / `IImu`'s `yawRate` named as adapters and an interface; no
  per-call claim) and :1232 (`differentiated velocity` — the calibration fit, unrelated) — no change (the brief's
  :241 / :1095 dispositions; :1095 is now :1098).
- `docs/guide/07-getting-set-up.md` :117 (`ProsImu` in a list of adapters) — no change.
- `docs/guide/10-the-api.md` :590 and `docs/internal/MECH-PANEL-requirements.md` :148 ("once per tick" — the
  mechanism service) and `docs/guide/17-the-competition-program.md` :305 ("once per tick" — the SD card) — unrelated.
- `docs/guide/11-reading-the-diagnostics.md` :40 — `IMU1` matched `IMU10` (a port label in a sample `[SES]` line) —
  unrelated. **A NEW hit at the tip, read and dispositioned.**
- `docs/faq.md` :64-66 ("The yaw rate is differentiated from it") — still true (the brief's disposition).
- `docs/shulib-v2-master-plan.md` :319 ("source it by differentiating `get_rotation()`") — still true. A NEW hit at
  the tip (the brief's grep list did not name it), read: it states the source, not a per-call behaviour.
- `docs/vexbuilder-integration.md` :607 (`imuYawRate`'s offset in the record layout; the brief's :559) — meaning and
  place unchanged.
- `docs/changelog.md` — outside IMU1's entry: :153 (the mechanism service "once per tick"), :690 (a slow loop "once
  per EPISODE"), :770 (the fit's "no differentiated velocity"), :2018 (`ProsImu` in R1a's adapter list) — unrelated;
  TANK1's 2.3 entry (:1225-1330 region) makes no claim about the IMU read.
- `docs/hardware-assumptions.md` — outside IMU1's edits: HA-04 :449/:454 and HA-109 :902 (the differentiated rate's
  SIGN and the default source — true; their Settle is Part B's), HA-179's "Robot two's default rate (differentiated
  `get_rotation()`) has the heading's sign" (true), HA-21's noise-differentiation floor (unrelated).
- `docs/internal/build-order.md` :1204 (the 2026-09-28 status line — dated history, left) and :1206 (the chunk's
  "why" paragraph — present tense about the defect, now under a Part-A-built status line, and its "today" sentence
  corrected above).
- `docs/internal/REMAINING-WORK.md` :14 and :25 (dated plan-history lines about the brief's answers) — left.
- `docs/internal/HANDOFF-remaining-work.md` :595 (TANK1's "IMU1 is a separate small chunk" — still true) — left.
- `docs/internal/verify/verify-e2.sh` :140 and `verify-e3.sh` :235 (`yawRate` in a gate's quoted code) — unrelated.
- `pros-imu`, `read twice`: no hit outside `docs/api/`. `A17`: only `docs/changelog.md` (IMU1's entry) and the DOCS2
  record (edited above). `kRateHoldUs`: only IMU1's own edits.
- `README.md`: no hit for any term.
- The C7 removability rule: no public document IMU1 edited links into `docs/internal/` (the changelog and the
  register cite "IMU1's brief" by name only — `api_doc_tool.py check-removability` below confirms).

### The mutations M1–M11 (§2.3) — each on a FRESH binary, the WHOLE suite run, red observed, restored *(appended at `date` 14:48)*
*(SUPERSEDED 2026-10-29: this table is the `2ef1f3f` tree's, 2643 cases. The final tree's — M1–M13 on `e5a69b5` + Part
A, 2723 cases, where M9 also turns L1–L3 red — is in "IMU1-A review findings and their disposition", below.)*
Runner: `.claude/coordinator/scratch/imu1-a/mutate.py` — one same-line-count edit (asserted: the anchor occurs once
and the line count is unchanged, so `docs/api/`'s line anchors do not move), `rm -f build/test/shulib_tests`, `cmake
--build build/test -j8`, then the WHOLE suite `(ulimit -v 4194304; timeout 600 ./build/test/shulib_tests)`; the file
restored in `finally` from a backup under `scratch/imu1-a/mut/` and its timestamp bumped (`os.utime`), and the binary
removed again. After the batch: `cmp` of every restored file against its backup — identical; `grep` for every
mutant marker in the tree — none. No run was killed by the memory cap or the timeout (each 19–26 s). Base for every
row: 2643 cases, all green.

| # | The edit (in `hal/pros/imu.hpp` unless named) | Suite | Red (failing cases, observed) | The brief's "must turn red" |
|---|---|---|---|---|
| M1 | the hold disabled, the constant kept: `if (false && rateAnswered_ && …` | 14 failed | T1, T2, T3 (a), T4, T5, T6, T8, T8 (a′), T9 (a), T9 (b), T10 (a), T10 (b), T11, and `ProsImu: gyro branch` (the adapter test's same-instant check) | T1, T2, T3 (a), T4, T5 (a), T6, T8 (a–e), T9, T10 (T11) — all ✓ |
| M2 | `<` → `<=` in the hold | 1 failed | T3 (a) | T3 (a) (the +5 000 µs read) ✓ |
| M3 | `kRateHoldUs = 300` | 6 failed | T3 (c) the pin, T1 (exactly its 21 rows at gap 400 µs, of 63), T2 (its 400 µs rows), T3 (a), T8 — exactly (d) `boundaryMismatches(on)` and its reads-off-the-tick on the **100 µs acceptance rows**, T8 (a′) (the 100 µs acceptance rows) | T1 (the 400 µs rows), T8 (d) (the 100 µs acceptance rows) ✓ |
| M4 | `kRateHoldUs = 10000` | 7 failed | T3 (c) the pin, T3 (a), T3 (b), T7, T8, T8 (a′), T9 (b) | T3 (c), T3 (a) ✓ |
| M5 | a held read still calls `get_rotation()` and discards it | 3 failed | T3 (a), T4, T5 | T4 ✓ |
| M6 | a faulted read does not open the hold (`rateAnswered_ = false` on a fault) | 2 failed | T4 (its dead ticks: `rotationReads` 130, `faultedReads()` 40, want 100 and 10), T5 (a) (2 faults, 5 device reads) | T5 (a) ✓ |
| M7 | a faulted read moves the differentiation sample (`lastSampleTime_ = now` on a fault) | 2 failed | T5, T7 | T5 (b), T7 ✓ |
| M8 | the gyro path bypasses the hold (`if (GyroRateZ) return gyroRate();` before it) | 3 failed | T4, T6, `ProsImu: gyro branch` | T6, T4's gyro half ✓ |
| M9 | `test/imu_buffer_model.hpp`: the copy made continuous (`truthCwDeg(now())`) | 3 failed | R0, T8, T10 (b) | R0 ✓ |
| M10 | `hasRateSample_ = true` initially — the first call answers a rate from a zero baseline | 4 failed | T2, T3 (a), T7, T9 (a) | T7 (its first call at t 0.37 s, 12.5° CW) ✓ |
| M11 | every read restarts the hold (`rateAnsweredAt_ = now;` in the HELD branch too) | 2 failed | T3 (a), T3 (b) | T3 (b) ✓ |

- **M3 and M4 first failed to BUILD** (`check-fresh`: `docs/api/pros-imu.md` prints the constant's VALUE, so a changed
  value is a stale page — the doc gate caught them before any test ran). Re-run with the page regenerated for the
  mutant (backed up, restored after — `cmp` identical) → the rows above. Logged as found, not hidden.
- **M11's form:** the brief words it "`rateAnsweredAt_ = now` before the check" — taken literally (set before the
  comparison) every read after the first is held for ever and nearly every test goes red, which would not show T3 (b)
  doing its job. The mutant run is the brief's MEANING ("every read, held ones included, restarts the hold"): the
  held branch restarts it. Only T3 (a) and T3 (b) go red — exactly the gap the brief says T3 (b) exists for.

### The reproduction, re-seen on the final tree (§7.2)
*(SUPERSEDED 2026-10-29: the `2ef1f3f` tree's. Re-seen on the rebased final tree, T12 and L1–L3 included, in "IMU1-A
review findings and their disposition" → "The tests and their runs", below.)*
`scratch/imu1-a/swap.py repro`: `2ef1f3f`'s `include/shulib/hal/pros/imu.hpp` put back (and the three API pages it
generates, and the register — whose new entries name the pin TU, which the doc staleness audit then calls a dead
path), `test/imu_rate_hold_pin_test.cpp` removed, the briefing regenerated with the binary absent; everything else
the final Part A tree (the final test file, the shim hook, the model). `git diff --stat 2ef1f3f --
include/shulib/hal/pros/imu.hpp` → empty. Build exit 0. `-tc="IMU1*,ProsImu*"` → **25 cases: 11 passed, 14 failed;
1374 assertions, 541 failed.** Per case, the failing-assertion counts are IDENTICAL to the first reproduction's
(T1 63, T2 84, T3 (a) 12, T4 4, T5 4, T6 3, T8 234, T8 (a′) 96, T9 (a) 4, T9 (b) 7, T10 (a) 10, T10 (b) 8, T11 11) —
the T9 assertion fix (above) changed no TODAY count — plus `ProsImu: gyro branch` (1: its new same-instant check
expects the hold). R0, T3 (b), T7 green, as stated. Restored: every file `cmp`-identical to its backup, the briefing
included.

### The fingerprint (§2.4, re-check R1) — "every other consumer unchanged"
*(SUPERSEDED 2026-10-29: against `2ef1f3f`. Re-run against the new merge base `e5a69b5` below.)*
**Run 1 (the base built at the start, configured on a CLEAN tree):** `fp-base-1/2.txt` (two runs of
`shulib_tests.base`, before any IMU1 file existed on disk) vs the first Part A binary → **12 differ**: S6 (the noise);
the brief's three (`P3-T3: "shulib Teleop" is retired everywhere…`, `P3-T3: the buttons left the mechanism table…`,
`ProsImu: gyro branch…`); and eight more, each re-run on its own and investigated:
- `build hash plumbing`, `guide-08a`, `guide-08b`, `C5 e2e`, `C5 hostile` — the BUILD HASH. The base was configured on
  a clean tree (`v0.1.1-394-g2ef1f3f`), Part A after the tree went dirty (`…-dirty`: the configure re-ran when the
  new test files changed the glob). The per-case diffs show only the hash text and byte offsets moved by exactly 6
  (`241 → 247`, `38255 → 38261`, `19 → 25` hash characters). Not behaviour.
- `fence: every adapter's suppression set…`, `fence: closes BEFORE shulib code…`, `shim: every header #errors…` —
  SOURCE SCANS: they read `include/shulib/hal/pros/*.hpp` and `test/pros_shim/pros/*.hpp` from disk at run time, and
  IMU1 changed `imu.hpp` in both. Re-run on their own, both binaries in the same tree → identical (only doctest's
  summary line differs). Not behaviour.
**Run 2 (the definitive one — the base rebuilt with the SAME dirty hash):** `scratch/imu1-a/swap.py base` put every
code, test and API file IMU1 touched back to `2ef1f3f` (and the register), removed the three new test files,
regenerated the briefing with no binary, and built → `shulib_tests.base-dirty` (hash `v0.1.1-394-g2ef1f3f-dirty`;
`git diff --stat 2ef1f3f -- include test docs/api` empty; its suite green, 2799446 assertions = the base's); then
restored (every file `cmp`-identical). Run, from the worktree root, nothing else running a suite: `(ulimit -v
4194304; timeout 600 setarch -R stdbuf -oL <binary> -s --order-by=file) 2>&1 | python3 -I case_hashes_v2.py "$PWD"`
(`scratch/imu1-a/fp.sh`) — the base TWICE, Part A's final binary once:
- base vs base: **1 differs — `screen (S6): the owner under contention…`** (the noise set);
- base (either run) vs Part A: **2 differ — S6 (noise) and `ProsImu: gyro branch — get_gyro_rate().z through the same
  canonical negate (T4)`** (its text changed, §1.4); **only base: 0; only Part A: 17** (the 16 IMU1 cases and the pin).
- Every TANK1, holonomic, Localizer, corrector, calibration, driver-loop, provider and mechanism case — the nine
  include-closure TUs of §2.4 item 1 among them — is byte-identical. In this run both binaries read the SAME tree
  from disk, so the source scans (P3-T3, fence, shim) match by construction; run 1 above is where they show, and
  why.

### The gates — every one re-run on the final tree *(appended at `date` 14:52, the last lines as printed)*
*(SUPERSEDED 2026-10-29: the `2ef1f3f` tree's. Every gate re-run on the rebased final tree below.)*
- **The suite:** `(ulimit -v 4194304; timeout 600 python3 tools/suite_run.py green)` → `suite run: green — 2643 cases,
  2800815 assertions` (base 2626 / 2799446; +17 = 16 + the pin). Built from `rm -f build/test/shulib_tests`, the
  briefing generated with no binary, the build, the suite, the briefing regenerated with the fresh binary, built again.
- **The ARM header gate** (RESUMING's command; `all.cpp` under `scratch/imu1-a/`, 227 lines = 226 headers + `main`;
  `arm-none-eabi-g++ -std=gnu++20 -Wall -Wextra -Wconversion -Wsign-conversion -Wshadow -Werror -Os -mcpu=cortex-a9
  -mfpu=neon-fp16 -mfloat-abi=softfp`) → exit 0.
- **`python3 tools/src_build_gate.py self-test`** → `src build gate self-test: OK (235 detector cases, each against a
  real build)`; **`check`** → `src build gate: PASS (all 7 builds link; …)` — bench, xdrive, tank, tank-drive,
  tank-calibrate, tank-autontest, tank-competition: each 11/11 TUs, 0 disallowed warnings, hash
  `v0.1.1-394-g2ef1f3f-dirty` asserted. (Its self-test plants code in `src/programs/bench_r3a.cpp` while it runs and
  restores it: `git status --short src` empty afterwards.)
- **The CI guards** (`.github/workflows/ci.yml`): PROS-free → `library is PROS-free (all of include/shulib except
  hal/pros/)`; sim layering → `layering holds: core is sim-free`; the compile refusals → `100% tests passed, 0 tests
  failed out of 251`, `compile-refusal TUs: 251; their CTests: 251` (IMU1 adds none). The fence test is in the suite
  (green).
- **The doc gates:** `api_doc_tool.py self-test` → `OK`; `check-coverage` → `doc coverage: 4961 public entities across
  186 headers, all documented`; `check-fresh` → silent, exit 0; `check-examples` → `doc example scan: 603 quoted lines,
  all verbatim (9 source files); 6 listings of whole-run files, each one unbroken run`; `check-removability` →
  `removability: no public doc references docs/internal/`; `briefing_status.py self-test` → `OK (38 cases …)`;
  `briefing_status.py check` → `generated block current`; `doc_staleness_audit.py self-test` → `OK (16 detector/filter
  cases …)`; `doc_staleness_audit.py` → `clean (31 live docs, 0 numeric claims checked) …`.
- **§7.9, the adapter's diff read line by line:** the clock read before the device (`yawRate()` :216); the hold
  measured from the last FRESH answer (`rateAnsweredAt_` set only on the fresh path, :222); a faulted read opens the
  hold (set before dispatch) without moving the sample (`differentiatedRate` :270-278); the `dt <= 0` guard kept with
  its comment (:288-293); the banner's ONE ANSWER PER CONTROL TICK (:37) and THE HOLD'S THREE LIMITS (:66), the class
  doc, `kRateHoldUs`'s doc (:153-164), `yawRate()`'s doc and `faultedReads()`'s doc present and true.

### Where Part A stands *(for the coordinator's verification — nothing committed)*
*(SUPERSEDED 2026-10-29: as of 14:55, on the `2ef1f3f` tree. "Where Part A stands now", below, is current.)*
- **Built and gated, host only; NOTHING has run on a robot.** The DoD's Part A items: the log ✓; reproduction first
  (every brief-named TODAY-red test seen red, counts above; R0, T3 (b), T7 green) ✓; the hold, every test green, the
  suite green, no existing test edited beyond §1.4 ✓; the fingerprint (only the noise and the named case differ) ✓;
  M1–M11 each red under its named tests ✓; the ARM gate, the src gate's seven builds and self-test, the doc gates ✓;
  §3's documents and the logged prose review ✓; not committed, no `pros` command, no `/dev/ttyACM*` ✓.
- **Not done here, by design:** Part B (the bench stage, Station N, HA-04/HA-109's *Settle*); `IMU1-COMPLETED.md`
  (after Part B); the `**Next:**` pointer (the coordinator's); the plan-of-record total in REMAINING-WORK (flagged).
- **For the coordinator:** (1) *(WITHDRAWN at the review, 2026-10-29 — see "IMU1-A review findings", C-1/D-1)* the
  worksheet's new M.6 rule tells the team to look for `IMU1` in `git log --oneline` — the landing commit's subject
  must carry "IMU1"; (2) shared documents edited on my own lines only — `docs/changelog.md`
  (a new section ABOVE MECH-A4's 2.5 entry), `docs/hardware-assumptions.md` (Index rows after HA-191; a new section
  after MECH-A3's; the status paragraph after MECH-A3's "With it, 178" and "7 of 181"), `docs/api/README.md` +
  `all-entities.md` (generated: 4961), `build-order.md`, `REMAINING-WORK.md`, `HANDOFF-remaining-work.md`,
  `PROJECT-BRIEFING.md` (generated) — VIS1-A1 and CAL1-A2a edit the same files: whichever merges second rebases them
  (its changelog section above the other's, the register's count re-counted, `docs/api/` and the briefing regenerated
  from a fresh build; §6 — DONE at the review: CAL1-A2a landed first, so IMU1 rebased onto `e5a69b5`, below); (3) *(superseded at the review: the base is now `e5a69b5`, re-run below)* the fingerprint's base is `2ef1f3f` — if VIS1-A1 (whose `xdrive_config_test.cpp` and
  `mechanism_devices_test.cpp` are in §2.4's nine) lands first, re-run it against the new merge base before IMU1
  merges (`scratch/imu1-a/swap.py base` + `fp.sh` do it in place); (4) no file another in-flight chunk owns was
  touched (in the shim, only `test/pros_shim/pros/imu.hpp`; `shim_control.hpp` untouched — `resetAll()` already
  calls `resetImus()`).
- *(14:55)* After the last log edit the build relinked the binary (the doc-gate stamp moved): re-run, `suite run: green —
  2643 cases, 2800815 assertions`; `briefing_status.py check` → `generated block current`.

---

## IMU1-A review findings and their disposition *(the review round, from `date` 18:47; appended at `date` 20:25, in the verification round — the review round stopped before this section was filled and appended; see "Verification-round findings and their disposition", after it)*

Three reviewers (correctness, tests, docs) returned 16 findings; overlaps are merged below. Numbering: C-1..C-4
(correctness), T-1..T-2 (tests), D-1..D-10 (docs), in the order they were handed over. **Every one was re-checked
before it was fixed; all 16 are real; all are fixed at the root here — none is waved off.** Nothing here has run on a
robot; nothing is committed.

### First: the base moved — IMU1 rebased onto `e5a69b5` (C-3, D-3)
- **Re-checked:** `git log --oneline -6 shulib-v2` → `e5a69b5` (CAL1 Part A2a, 15:14), `d8fdb8b` (the PANEL SPRINT plan
  commit), `4691757` (MECH-A4's landing date), then my base `2ef1f3f`; `git merge-base --is-ancestor 2ef1f3f
  shulib-v2` → true. VIS1-A1 has NOT landed (worktree `vis1-a1` still on `ceb17c0`). So under §6 IMU1 merges SECOND
  and owes the rebase — now, not "if VIS1-A1 lands first" (my earlier coordinator note named only VIS1-A1: wrong).
- **Done (no commit made):** the whole uncommitted Part A backed up first (`scratch/imu1-a/prerebase/`: the patch
  against `2ef1f3f`, 1 310 lines, and a tarball of all 23 changed/new files); `git stash push -u` → `git merge
  --ff-only e5a69b5` (the `imu1-a` branch fast-forwarded — no commit of mine) → `git stash pop`. Seven files both
  sides touched: four conflicted — `docs/api/README.md`, `docs/api/all-entities.md` and `PROJECT-BRIEFING.md`
  (generated: taken from `e5a69b5`, then REGENERATED from a fresh build) and `docs/changelog.md` (resolved by hand:
  IMU1's section first under `## API 2.5`, then CAL1-A2a's whole entry unchanged, then MECH-A4's with the tip's
  dated header); three merged clean — `build-order.md`, `REMAINING-WORK.md`, `HANDOFF-remaining-work.md` (my hunk
  counts per file re-counted against the backup patch: 13, 9, 4 — equal). Every file only I touched `cmp`-identical
  to its backup. My stash entry then dropped (the stash list is shared by every worktree; the backup holds it).
- **The register recounted:** CAL1-A2a registered no HA (178 at the tip) → 181 with IMU1's three; the Held line and
  "next free HA-216" unchanged. `docs/api/`: 5 059 → **5 060** entities across 188 headers (the tip's count + the one
  constant). The briefing: OPEN CHUNKS gains IMU1; the suite line from the fresh binary.
- **A tenth test TU reaches the adapter** (C-3, D-3 — confirmed from the build's own dependency files,
  `build/test/CMakeFiles/shulib_tests.dir/*.o.d`): `brake_stop_stack`, `driver_panel_claws`, `mechanism_devices`,
  `mechanism_parts`, `mechanism_service_evidence`, **`pros_calibrate_adapter`** (new with CAL1-A2a: it includes
  `hal/pros/calibrate.hpp` → `hal/pros/device_provider.hpp` → `hal/pros/imu.hpp`), `pros_device_provider`,
  `pros_imu_adapter`, `robot_two_driver`, `xdrive_config` — plus IMU1's own two. The fingerprint re-run against
  `e5a69b5` is below; CAL1-A2a's code reads no rate itself (`calibrate_program.hpp` hands its `IImu` to the
  calibrator: `git grep 'yawRate()'` over `include src` is line-for-line identical at `2ef1f3f` and `e5a69b5`, 23
  sites).

### C-1 / D-1 (blocker) — Station M's "IMU1 build" check passed on trees without the fix
- **Re-checked, TRUE:** `git log --oneline e5a69b5 | grep IMU1` prints two plan commits (`73044ca`, `a62067b`); `git
  show <rev>:include/shulib/hal/pros/imu.hpp | grep -c kRateHoldUs` prints **0** at `2ef1f3f`, `e5a69b5`, `a62067b`
  and `73044ca`, and **7** on this tree. The words were the brief's §3.2; I copied them and added a coordinator note
  that could not have helped.
- **Fixed:** the worksheet's paragraph before M.6.1 now checks the TREE for the fix — `grep -c kRateHoldUs
  include/shulib/hal/pros/imu.hpp` must print 1 or more before M.1's build; 0 is a pre-IMU1 tree — says why not the
  git log, and gives the same test for a run written down earlier (the boot hash's `g…` commit through `git show`).
  "Before IMU1 (Part A built 2026-10-29)" → "Before IMU1 Part A"; "after IMU1 landed" → "built from a tree with IMU1
  Part A's hold in it"; the claim about the second read marked derived (landmine 14). M.6.4's row now records "the
  `kRateHoldUs` grep's count". The brief's §3.2 template gains a dated correction note (not rewritten); this log's two
  coordinator notes are marked WITHDRAWN in place. REMAINING-WORK's Station M bullet says the check is a grep for
  `kRateHoldUs`, not the git log; build-order's status line too; HA-179 names it.

### C-2 / D-2 (major) — the changelog left out limit (1)
- **Re-checked, TRUE:** `sed -n` over the entry, `grep -i 'stall\|late\|wake'` → nothing; it said "Two cautions". The
  team lead's D1 (a) requires all three limits in the code AND the changelog; the brief's §3.1 template had the gap.
- **Fixed:** "What you must do" now carries **The hold's three limits** — (1) a stall or a late wake (the Localizer's
  next two ticks off for a stall; one tick stale, three off, for a wake 5 ms or more late, where a 4 ms late wake the
  hold does not touch costs two), (2) the first read moving (→ *Not fixed*, now labelled limit (2), with T9 (b)'s
  ceiling), (3) a reader outside the tick (two Localizer ticks per read: 60 of 300, against 30 before IMU1; the 4 ms
  loop). Every figure is now pinned by a suite test (T-2) *(corrected in the verification round: every figure but the
  pre-IMU1 30, which only a run with the old adapter put back can measure — V-4, below)*. The brief's §3.1 gains a
  dated correction note.

### C-4 / T-1 (minor) — landmine 4 (clock before the device, one instant for both) had no test
- **Re-checked, TRUE:** the model's every device read takes the same `gap` and T7's clock stands still during a read,
  so an instant taken after the read is the instant before it plus a constant. The reviewers' mutants N1/N2 passed
  the whole suite (their `run2/results.jsonl`).
- **Fixed — two cases, no new symbol (they build against either adapter):** **IMU1 T12 (a)** — a fresh read whose
  device read takes 3 ms (the shim's `micros()` advanced inside `get_rotation()` through the `rotationAt` hook): a read
  at +4 999 µs from the read's START is held, at +5 000 µs fresh (asserts the read really took 3 ms); **IMU1 T12 (b)**
  — reads 10 ms apart at their start, latency alternating 0 / 3 ms, the rate exactly 100 deg/s CW on all 19
  derivatives (the first call 0). Mutations **M12** (N1: the derivative's instant re-read after the device) and
  **M13** (N2: the hold's instant stamped from a second clock read after the dispatch) added to the table below.
- **On the unchanged adapter** (the swap, below): T12 (b) RED (19 of 19 off — the pre-IMU1 adapter read the clock
  AFTER the device, landmine 4's "today's order"), T12 (a) RED (4 failed assertions: the 3 ms read answered −1.34 rad/s
  where −1.75 is right — 1° over 13 ms, the read's latency inside its interval — and, with no hold, the +4 999 µs
  read re-sampled). Both were written after the fix, at the
  review — said so in the file's header; not a reproduction-first test.

### T-2 (minor) — the host figures the documents quote were not pinned
- **Re-checked, TRUE:** the changelog's 4 ms loop, HA-215's 55 / 60 / 30 and probe 3b's costs, HA-214's 73 / 73 came
  from the brief's probes on a PROTOTYPE adapter; T9 (b) printed its counts only.
- **Fixed:** three new cases on the REAL adapter over `BufferModel` (gap 0, every read at the case's instant — the
  probes' schedules and draws reproduced exactly):
  - **IMU1 L1** (limit (1), probe 3b): a 6 ms stall on tick 100 at φ 5 and 3 ms → 298 exact, 1 high, 1 low, the later
    reader apart on 1 tick; tick 101 waking 4 ms late → 298 / 1 / 1 (the hold not involved); 6 ms late → 297 exact,
    2 high, 1 low.
  - **IMU1 L2** (limit (2)'s residual, probe 3 (2)/(2′)): the copy at the Localizer's instant, ±0.05 ms of wake
    jitter → 154 exact, 73 zero, 73 doubled, with two readers AND with one, the tick's readers never apart.
  - **IMU1 L3** (limit (3), the re-check's span and fast-loop probes, φ 3 / 5.5 / 8 ms): an off-tick read at +6.5 ms
    on 1 tick in 10 → 60 of 300 off (two per read); a reader at a random +4..+7 ms → 55 exact; steady +4 / +6 / +7.5
    ms → 300 exact; a 4 ms loop → 0 exact, zero on exactly one read in five (596 of 2 980); a 10 ms loop → 2 980 exact.
  - **T9 (b)** keeps the brief's "recorded, never asserted exact" for its residual and gains a CEILING: in the band
    zero ≤ 60 and double ≤ 60 (one each per fresh fix counted) and nothing else off; far from it, every tick exact.
  Every figure matched the brief's probe output EXACTLY on the first run. **On the unchanged adapter:** L3's off-tick
  case reads **30** at all three phases (the documents' "before IMU1, 30", now measured on the real old adapter on
  this tree), its 4 ms loop 1 788 zero; L2's one-reader case is green (identical — "IMU1 does not cure it", confirmed)
  and its two-reader case red (0 exact, 154 zero, 146 high); L1's 299 high per case (the old adapter's double read).
  The documents now say the suite pins them: the changelog ("pinned by IMU1's tests L1–L3 and T9 (b)"), HA-213 (L3),
  HA-214 (L2; T9 (b)'s ceiling), HA-215 (L1, L3, re-run on the old adapter), `ProsImu`'s banner ("the costs below
  pinned by IMU1's L1-L3"). *(The changelog's and HA-215's pre-IMU1 "30" was
  not one of them — corrected in the verification round, V-4, below.)*

### D-4 (minor) — HA-215's Source cited the pin TU, which named no HA
- **Re-checked, TRUE** (`grep -n 'HA-21' test/imu_rate_hold_pin_test.cpp` → nothing). **Fixed:** the pin's header now
  says what the pin cannot test — the belief under it, "(A4 register HA-215)". Direction 2 re-run over all three
  entries' Sources: `hal/pros/imu.hpp` HA-213/214/215; `test/imu_buffer_model.hpp` HA-213/214/215; the shim's
  `imu.hpp` HA-213; the pin HA-215 — every cited id found.

### D-5 (minor) — "about 0 or a spike" repeated as fact (landmine 14)
- **Re-checked, TRUE** at every site named, and at two more of my own (the DOCS2 and TANK1 dated pointers).
  **Fixed:** build-order's IMU1 paragraph put in the PAST tense with "derived from the adapter's code and PROS's
  documented 10 ms copy, HA-213 — never measured" (and the deviations-table row); `move_to_pose.hpp`'s comment ("would
  have been ~0 or a spike — derived, never measured"; still three lines, so `docs/api/move_to_pose.md` does not move);
  `motion_tank_steer_test.cpp`'s `ProsShapedImu` doc and P14's comment ("would return / would have been … derived,
  never measured"); HANDOFF ("would damage … derived … never measured"); the TANK1 and DOCS2 pointers; HA-179 (below).

### D-6 (minor) — two API texts stale or unconditional
- **Re-checked, TRUE.** **Fixed:** `ProsImu`'s constructor: "`clock`: read on EVERY yawRate() call, on either source
  (the one-answer-per-tick hold, IMU1), and by the differentiation path; must outlive this adapter";
  `DebugRecord::imuYawRate`: "on ProsImu since IMU1, the same value every reader saw that tick, the Localizer's
  included, within the hold's limits (a tick that stalls over 5 ms between its reads gives its later readers a fresh
  answer; ProsImu's header)". `docs/api/` regenerated (`pros-imu.md`, `debug_record.md`, and the two indexes — still
  exactly §3.1 item 3's four).

### D-7 (minor) — the log's "only routine.hpp and CMakeLists changed" was false
- **Re-checked, TRUE** — and wider than the finding. Every file the brief names (108 path tokens, `grep -oE` over the
  brief), diffed `ceb17c0..2ef1f3f` and `2ef1f3f..e5a69b5`:
  - `teleop/teleop_loop.hpp` (MECH-A4, +176/−63; cited in R7 and §0.3 at :32, :345, :256-261): :32 unchanged
    ("heading() sampled ONCE per tick"); :345 → **:343** (`const HeadingSample h = heading();`); the
    `HeadingSource` template :256-261 → **:235-239** (its second overload :388 → :448-453). Substance unchanged: no
    teleop header reads an `IImu` (`git grep 'IImu\|yawRate' -- include/shulib/teleop` → only `field_centric.hpp`
    :38's comment, at all three revisions).
  - `chassis/routine.hpp` (MECH-A4, +485/−1 — logged at the start; additive; T8 consumes it).
  - `sim/mechanism_plant.hpp` (MECH-A3/A4, +81/−9) and `version.hpp` (+22/−2, then +1/−1): named only in §6's
    coordination table as the in-flight chunks' files — no line cited, nothing IMU1 reads.
  - `tools/briefing_status.py`, `tools/doc_staleness_audit.py` (+142/−23, +147/−26) and `tools/suite_run.py` (new):
    named in §7 as gate COMMANDS, no line cited; run as such below.
  - `2ef1f3f..e5a69b5`: CAL1-A2a's `calibration/calibrate_program.hpp` (new), `calibration/calibrate_panel.hpp`,
    `hal/pros/calibrate.hpp` (new) — §6's in-flight row, now landed; no new rate reader (above); and the shim's
    `pros/motors.h` (+21/−6, CAL1's dropout knobs) — cited only in Part B's R10 text (STAGE 2c's includes), not mine.
  - Every other code file the brief cites is byte-identical across `ceb17c0`, `2ef1f3f` and `e5a69b5`, so its
    `file:line` holds. The log's false sentence is corrected IN PLACE with a pointer here.

### D-8 (minor) — D4 (a) carried to Station M only; §6 also names K.5
- **Re-checked, TRUE** (§6's Robot sessions row: "M.6 and K.5's floor runs … wait for an IMU1 build"; K.5 had no IMU1
  word; REMAINING-WORK row 4's Waits-on read "—"). **Fixed:** K.5 gains "K.5 counts with M.6's rule" (the same grep
  before K.1's build; K.2.2's hash in K.5.3's table, which gains the row); REMAINING-WORK row 4 Waits on "32 (M.6
  only)", row 6 "5; 32 (K.5 only)" and its item text, the Station K bullet; build-order's status line.

### D-9 (minor) — the HA-179 rewrite dropped TANK1's corrected-robot evidence and limit (2)
- **Re-checked, TRUE** (`git log -S "15 of 15 probe"` → `3965791`, TANK1 Part A, 2026-09-28). **Fixed:** HA-179 keeps
  TANK1's model as dated history (15 of 15 / 8 of 15, "kept as it was written 2026-09-28") and gains the residual: on
  a GPS- or tag-corrected robot the pair of ticks around each fresh/stale change reads 0 and about double — the
  observer's input included (T9 (b): 180 / 60 / 60 of 300); no steering run measured through it; the cure is the first
  corrected robot's chunk (the brief's §5); none this season. Its "Before IMU1 … NOT the tick's clean first read"
  now says DERIVED, no robot log (the brief's §0.5). build-order's status line says the same in one sentence.

### D-10 (minor) — REMAINING-WORK's plan-of-record total vs its rows
- **Re-checked, TRUE, and older than IMU1:** the table's own auto-flipper note already says the plan of record's rows
  "do not re-add exactly … the totals wait for a full re-pricing — the coordinator's". Recomputing only this line
  would contradict that note and the "Everything" line under it. **Fixed by that rule, stated:** the IMU1 row now says
  it was ~½–1, that the brief's ~¾ more is NOT added into the plan of record or the totals, and why (the same rule as
  the auto-flipper row). The rows would now add to ~26¾–33¼ — the re-pricing is the coordinator's.

### The tests and their runs (this round)
- **New at the review — written AFTER the fix, not reproduction-first** (the file's header says so; since the
  verification round its REPRODUCTION FIRST paragraph names only R0 and T1–T11): **IMU1 T12 (a)**, **T12 (b)**,
  **L1**, **L2**, **L3** in `test/imu_rate_once_per_tick_test.cpp` (21 cases now), and T9 (b)'s ceiling. Still no
  symbol the fix adds: the file builds against either adapter.
- **On Part A** (18:57, `scratch/imu1-a/r-imu1.txt`): `-tc="IMU1*,ProsImu*"` → **31 cases, 31 passed; 1 475
  assertions** (the 21, the pin, the nine `ProsImu` cases).
- **On the unchanged adapter** — `swap.py repro` (`e5a69b5`'s `hal/pros/imu.hpp`, its three API pages and the
  register put back, the pin TU removed; everything else the final tree). Run at 18:57 in the review round
  (`r-repro.txt`) and AGAIN on the final tree at 20:19 in the verification round (`vr/repro-final.txt`; `git diff
  --stat e5a69b5 -- include/shulib/hal/pros/imu.hpp` empty, build exit 0; restored, `git status --short` identical
  before and after, the adapter and its page `cmp`-identical to their finals): **30 cases: 11 passed, 19 failed;
  1 474 assertions, 586 failed** — the per-case counts of the two runs IDENTICAL: T1 63, T2 84, T3 (a) 12, T4 4,
  T5 4, T6 3, T8 234, T8 (a′) 96, T9 (a) 4, T9 (b) 11, T10 (a) 10, T10 (b) 8, T11 11; T12 (a) 4, T12 (b) 1, L1 15,
  L2 5, L3 16; `ProsImu: gyro branch` 1. R0, T3 (b), T7 green, as the brief states. T1–T11 match the first
  reproduction's counts except T9 (b), 7 → 11: its new ceiling is red on the old adapter too (its MESSAGE lines:
  in the band 240 exact and **60 "other"**, far from it 240 exact and 60 zero — at both scales). L3's off-tick case
  reads `CHECK( 30 == 60 )` at all three phases — the documents' "before IMU1, 30" (and on the verifier's own run,
  `verify/old-L.txt`, the same three).
- **The whole suite** on the final tree (the verification round): `(ulimit -v 4194304; timeout 600 python3
  tools/suite_run.py green)` → **green — 2 723 cases, 2 936 803 assertions** (19:35; again at 19:47 on the
  fingerprint's fresh rebuild). The base `e5a69b5`, measured by the fingerprint's base build below: **2 701 / 2 935 334**
  (= the committed briefing's). +22 cases = the 21 in `imu_rate_once_per_tick_test.cpp` + the pin; +1 469 assertions.

### The mutations M1–M13 on the rebased tree — each on a FRESH binary, the WHOLE suite, red observed, restored
**Run in the verification round, 19:36–19:44** — the review round's own batch was killed by SIGTERM during M9 and
left the M9 mutant in the tree (the verification round's blocker, below). `scratch/imu1-a/mutate.py`, now with
SIGTERM/SIGHUP raised as an exception (so `finally` restores) and every child in its own process group killed on
an interrupt (so an interrupted build cannot finish as an orphan and link a mutant after the restore) — that path
tested by killing a live M9 run 6 s in: restored `cmp`-identical, no binary, no orphan build. Each row: one
same-line-count edit (asserted), `rm -f build/test/shulib_tests`, a FRESH build, the WHOLE suite under `(ulimit -v
4194304; timeout 600 …)`, the file restored in `finally` and its timestamp bumped, the binary removed. Every row
exit 1, 21–34 s; **none was killed by the cap or the timeout**; no `CRASHED` in any output. After the batch: the
adapter, the model and `pros-imu.md` `cmp`-identical to their finals; `grep -rn '// M[0-9]*:' include test docs`
empty. Base for every row: **2 723 cases, all green.** M1–M8's failing sets are IDENTICAL, case for case, to the
review round's own run on this rebased tree (`mut/results.jsonl`, 19:03–19:10); M9–M13's to the verifier's
independent run (`verify/mut/results.jsonl`).

| # | The edit (in `hal/pros/imu.hpp` unless named) | Suite | Red (failing cases, observed) | The brief's "must turn red" |
|---|---|---|---|---|
| M1 | the hold disabled, the constant kept | 18 failed | T1, T2, T3 (a), T4, T5, T6, T8, T8 (a′), T9 (a), T9 (b), T10 (a), T10 (b), T11, T12 (a), L1, L2, L3, `ProsImu: gyro branch` | T1, T2, T3 (a), T4, T5 (a), T6, T8 (a–e), T9, T10 (T11) ✓ |
| M2 | `<` → `<=` in the hold | 3 failed | T3 (a), T12 (a), L3 | T3 (a) ✓ |
| M3 | `kRateHoldUs = 300` (its page regenerated for the mutant, restored after) | 10 failed | T3 (c), T1 (21 failures, all at gap 400 µs), T2, T3 (a), T8, T8 (a′), T12 (a), L1, L2, L3 | T1 (the 400 µs rows), T8 (d) ✓ |
| M4 | `kRateHoldUs = 10000` (the same) | 11 failed | T3 (c), T3 (a), T3 (b), T7, T8, T8 (a′), T9 (b), T12 (a), L1, L2, L3 | T3 (c), T3 (a) ✓ |
| M5 | a held read still calls `get_rotation()` | 4 failed | T3 (a), T4, T5, T12 (a) | T4 ✓ |
| M6 | a faulted read does not open the hold | 2 failed | T4, T5 | T5 (a) ✓ |
| M7 | a faulted read moves the differentiation sample | 2 failed | T5, T7 | T5 (b), T7 ✓ |
| M8 | the gyro path bypasses the hold | 3 failed | T4, T6, `ProsImu: gyro branch` | T6, T4's gyro half ✓ |
| M9 | `test/imu_buffer_model.hpp`: the copy made continuous | 6 failed | R0, T8, T10 (b), **L1, L2, L3** | R0 ✓ |
| M10 | `hasRateSample_ = true` initially | 4 failed | T2, T3 (a), T7, T9 (a) | T7 ✓ |
| M11 | every read restarts the hold (the held branch too) | 4 failed | T3 (a), T3 (b), T12 (a), L3 | T3 (b) ✓ |
| M12 | the derivative's instant re-read AFTER the device (the reviewers' N1) | 2 failed | T12 (a), T12 (b) | *(review-added)* T12 (b) ✓ |
| M13 | the hold's instant stamped from a second clock read after the dispatch (N2) | 1 failed | T12 (a) | *(review-added)* T12 (a) ✓ |

On the `2ef1f3f` tree M9 turned R0, T8 and T10 (b) red; L1–L3, which did not exist then, are red under it too —
they read the model's copy, so a continuous copy moves every figure they pin.

### The fingerprint, re-run against the new merge base `e5a69b5` (§2.4)
**Run in the verification round, 19:44–19:52** (`fp-e5.sh` was written in the review round and never ran). In steps,
each its own call: `swap.py base` (BASE `e5a69b5`; now with the same signal handling): every code, test and API file
IMU1 touched and the register put back to `e5a69b5`, the three new test files removed, the briefing generated with
no binary, built (exit 0) → `scratch/imu1-a/vr/shulib_tests.base-e5`, hash `v0.1.1-397-ge5a69b5-dirty`; `git diff
--stat e5a69b5 -- include test docs/api` **empty**; its suite **green, 2 701 cases, 2 935 334 assertions**; restored
(15 files; `git status --short` identical before and after; the adapter, its page and the model `cmp`-identical to
their finals). Then Part A from `rm -f build/test/shulib_tests` (the briefing generated with no binary), built, its
suite green 2 723 / 2 936 803 → `vr/shulib_tests.partA-e5`, the same hash. `fp.sh` from the worktree's root — the
base TWICE, Part A once:
- base vs base: **1 differs — `screen (S6): the owner under contention…`** (the noise set);
- base (either run) vs Part A: **2 differ — S6 (noise) and `ProsImu: gyro branch — get_gyro_rate().z through the
  same canonical negate (T4)`** (§1.4's edited case); **only base: 0; only Part A: 22** — listed by name: the 21
  `IMU1 …` cases and the pin, nothing else.
- So every other case — the ten include-closure TUs (CAL1-A2a's `pros_calibrate_adapter` among them) and
  CAL1-A2a's `calibrate_program` cases — is byte-identical. Both binaries were built in the same dirty tree, so
  the build hash and the source scans match by construction (run 1 at `2ef1f3f`, above, is where those show, and
  why). The verifier's independent run (`verify/fp-*-e5*`, built outside this worktree) found 4 differ: its two
  beyond these — `hal::pros::runCalibrate PC6` and `build hash plumbing` — are hash-only (its two binaries carried
  different hashes; `verify/pc6-*` and `verify/bh-*` differ only in the hash lines).

### The prose review (rule 4), re-run at the rebased tree — the greps first, every hit read
The greps RUN FIRST (in the verification round, before any of this was written), every hit read. Scope: the text this review round and the
verification round added or changed. `grep -rIn --exclude-dir=api -F "<term>" docs README.md` (this log excluded):
- **The withdrawn check** (`grep IMU1`, `git log --oneline`): the worksheet's own warning ("Not `git log | grep
  IMU1` …"), the brief's §3.2 template (:1050) and its dated correction (:1059 — a brief is the record of what was
  asked: corrected by note, not rewritten); every other `git log --oneline` hit is a push check or a session
  command (HANDOFF :48/:73, RESUMING :52, the briefing, RELEASING, chunk records) — unrelated. **No live document
  tells the team to look for IMU1 in the git log.**
- **`IMU1 build`, `kRateHoldUs`, `K.5`:** the worksheet's M.6 and K.5 paragraphs and their tables' rows, REMAINING-WORK
  rows 4, 6 and 32 and the Station M / Station K bullets, build-order's status line, the register (HA-22, HA-179,
  HA-213..HA-215, the status paragraph) — every one read: each check is the tree grep, "1 or more". COMP-PROGRESS
  and T4-PROGRESS (K.5 as history) — left.
- **`0 or a spike`, `about 0`, `never measured` (landmine 14):** the changelog's IMU1 entry (:29, :34 — its preamble
  says DERIVED, never measured on our brains), build-order :1210 and the deviations row (derived, said), HA-179
  (its rewrite says DERIVED; TANK1's model kept as dated history), `ProsImu`'s banner (DERIVED, :49),
  `ProsShapedImu`'s doc (derived, never measured), HANDOFF, the TANK1 and DOCS2 pointers — all say it. **One
  miss, found here and FIXED: HA-22's IMU1 amendment** said a pre-IMU1 REST capture "has a contaminated rate
  channel … 0 or a spike" with no "derived" (neither the reviewers' D-5 list nor my D-5 fix named it) — it now
  says "DERIVED from the adapter's code and PROS's documented 10 ms copy (HA-213), never measured on a robot".
  TANK1-tank-moveto-steers.md and TANK1-PROGRESS.md (history records) left. The `about 0.x s` hits (guide 11 and
  14, cookbook 04, the worksheet :561/:1735, the register :2640-2655, the changelog :1392-1396) are unrelated.
- **`three limits`, `late wake`, `limit (1)`, `measured once`:** the changelog, HA-215, HA-32/HA-102's notes,
  build-order's status line, the brief; SER1-PROGRESS ("three limits" of its own) and the R5a brief ("measured
  once") unrelated. **Fixed here:** the changelog's and HA-215's pre-IMU1 "30" (verification finding 4 — now
  "measured … with that adapter put back; no test holds that figure"); **build-order's status line** said "Its
  tests were written FIRST" and "the hold's three limits are pinned at the figures the changelog quotes" — both
  over-claimed once T12 and L1–L3 joined after the fix: it now says R0 and T1–T11 first, T12 and L1–L3 after the
  fix, every figure pinned but the 30. REMAINING-WORK row 32's status brought level with it.
- **The brief's whole term list, re-run at the rebased tree** (`ProsImu`, `pros-imu`, `yawRate`, `yaw rate`,
  `imuYawRate`, `once per tick`, `differentiat`, `second read`, `read twice`, `immune`, `IMU1`, `kRateHoldUs`,
  `A17`; chunk records excluded): the same files as at `2ef1f3f`, plus the generated briefing (`IMU1` in OPEN
  CHUNKS). CAL1-A2a's changelog entry adds no hit; the changelog's hits outside IMU1's entry are :281 (the
  mechanism service), :818, :898, :1855 (the calibrator's rest gate "requires the IMU's yaw rate under a floor" —
  true, unchanged by the hold) and :2146 (an adapter list) — unrelated. No guide chapter describes the rate's
  per-call behaviour (`grep -rniE 'imu.{0,40}rate|rate.{0,40}imu|turn rate|spin rate' docs/guide docs/cookbook
  docs/faq.md README.md` → six hits, none about it).
- **`ProsImu`'s banner** ("the costs below pinned by IMU1's L1-L3"): read — it quotes no pre-IMU1 figure; "the one
  case where the hold does worse than the adapter before it" is a comparison (L3: 60 against the measured 30), not
  a pinned cost. Left; no code changed in the verification round.
- `README.md`: no hit for any term. The removability rule: `check-removability` below.

### The gates — every one re-run on the final tree
Every gate on the final code (`include/`, `test/`, `src/` unchanged since 19:31's test-header edit; the documents
edited after are re-checked by the doc gates, re-run last):
- **The suite:** green — 2 723 cases, 2 936 803 assertions (19:35, 19:47; and the final line below). Built from `rm -f
  build/test/shulib_tests`, the briefing generated with no binary, the build, the suite, the briefing regenerated
  with the fresh binary, built again.
- **The ARM header gate** (CI's command; `scratch/imu1-a/vr/all.cpp` — "TU includes 228 headers", every
  `include/shulib` header; `arm-none-eabi-g++ -std=gnu++20 -Wall -Wextra -Wconversion -Wsign-conversion -Wshadow
  -Werror -Os -mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=softfp`) → exit 0.
- **`python3 tools/src_build_gate.py self-test`** (19:59–20:15, 16 min 44 s) → `src build gate self-test: OK (235
  detector cases, each against a real build)`, exit 0, `git status --short src` empty after; **`check`** (20:16–20:18)
  → `src build gate: PASS (all 7 builds link; …)` — bench, xdrive, tank, tank-drive, tank-calibrate, tank-autontest,
  tank-competition: each 11/11 TUs, 0 disallowed warnings, hash `v0.1.1-397-ge5a69b5-dirty` asserted; `git status
  --short src` empty after.
- **The CI guards:** `library is PROS-free (all of include/shulib except hal/pros/)`; `layering holds: core is
  sim-free`; the compile refusals (`ctest -R compile_refusal`, 19:54–19:58) → `100% tests passed, 0 tests failed out
  of 251`, `compile-refusal TUs: 251; their CTests: 251`.
- **The doc gates** (19:53, before this round's document edits; re-run after them, below): `api_doc_tool.py
  self-test` → OK; `check-coverage` → `doc coverage: 5060 public entities across 188 headers, all documented`;
  `check-fresh` → exit 0; `check-examples` → `603 quoted lines, all verbatim (9 source files); 6 listings …`;
  `check-removability` → `no public doc references docs/internal/`; `briefing_status.py self-test` → OK (38
  cases); `check` → `generated block current`; `doc_staleness_audit.py self-test` → OK (16 cases); the audit →
  `clean (31 live docs, 0 numeric claims checked)`.

### Where Part A stands now *(for the coordinator — nothing committed)*
- **Built and gated, host only; NOTHING has run on a robot.** The tree: `e5a69b5` (CAL1 Part A2a landed) + Part A,
  uncommitted, in the worktree `imu1-a`. The DoD's Part A items: the log ✓; reproduction first — R0 and T1–T11
  written and seen RED on the unchanged adapter before the fix (the first sections), re-seen on the final tree; T12
  and L1–L3 added after the fix, at the review, and seen red on the unchanged adapter too (said so in the file and
  here) ✓; the hold, every test green, the suite green (2 723 / 2 936 803), no existing test's assertions edited
  beyond §1.4's gyro case (`motion_tank_steer_test.cpp`: comments only) ✓; the fingerprint against `e5a69b5` — only
  the noise and the named case differ ✓; M1–M13 each red on the final tree, each on a fresh binary ✓; the ARM
  gate, the src gate's self-test and seven builds, the CI guards, the doc gates ✓; §3's documents and the logged
  prose review ✓; not committed, no `pros` command, no `/dev/ttyACM*` ✓.
- **Not done here, by design:** Part B (the bench stage, Station N, HA-04/HA-109's *Settle*); `IMU1-COMPLETED.md`
  (after Part B); the `**Next:**` pointer (the coordinator's); the plan-of-record total (D-10 — the coordinator's
  re-pricing).
- **For the coordinator:** (1) IMU1 is already rebased onto `e5a69b5`; if VIS1-A1 lands before it, IMU1 rebases
  again (§6): its changelog section stays on top, the register re-counted, `docs/api/` and the briefing regenerated
  from a fresh build, and the fingerprint re-run against the new merge base (`swap.py base` with `BASE` set to it,
  then `fp.sh` — VIS1-A1's `xdrive_config_test.cpp` and `mechanism_devices_test.cpp` are in the include closure).
  (2) Shared documents, my own lines only: `docs/changelog.md` (a section above CAL1-A2a's), `docs/hardware-assumptions.md`
  (Index rows after HA-191; a section after MECH-A3's; the status paragraph; HA-22/32/102/179 amended),
  `docs/api/README.md` + `all-entities.md` (generated: 5 060), `build-order.md`, `REMAINING-WORK.md`,
  `HANDOFF-remaining-work.md`, the worksheet (M.6, K.5), `PROJECT-BRIEFING.md` (generated); `test/CMakeLists.txt`
  NOT edited (its GLOB). (3) The landing commit's subject need not carry anything: the worksheet checks the tree for
  `kRateHoldUs`. (4) No file another in-flight chunk owns was touched (in the shim, only
  `test/pros_shim/pros/imu.hpp`).

---

## Verification-round findings and their disposition *(from `date` 19:31; appended at `date` 20:27)*

A fresh verifier returned four findings — one blocker, one major, two minor. **Each was re-checked before it was
fixed; all four are real; each is fixed at the root.** Three more were found by this round's own prose review and
fixed. Nothing is committed; nothing here has run on a robot.

### V-1 (blocker) — the M9 mutant and its mutant binary were left in the worktree
- **Re-checked, TRUE** (19:31, before anything else): `grep -rn '// M[0-9]*:' include test` →
  `test/imu_buffer_model.hpp:81`, `return truthCwDeg(now());` with M9's marker comment (not quoted here, so the
  marker grep stays clean); `diff` against
  `scratch/imu1-a/mut/M9.imu_buffer_model.hpp.bak` → that line only (the backup `cmp`-identical to
  `imu_buffer_model.hpp.final-pre-mut`); `build/test/shulib_tests` present, 19:10 — the M9 mutant's link;
  `mut/M9.out` ends `test case CRASHED: SIGTERM`; `mut/results.jsonl` stops at M8; `mutate.py` restored in
  `finally` but installed no signal handler, and Python's default SIGTERM ends the process without running
  `finally`. The driver started at 19:03:32 (`mut/run.err`'s creation) and was killed at 19:10:58 — 7 min 26 s in,
  so not the 600 s limit of a foreground call; most likely the end of the review round's turn, which terminates a
  background command (likely, not proven).
- **Fixed — the restore first:** the backup copied back, `touch`ed, `rm -f build/test/shulib_tests`; `cmp` with
  `imu_buffer_model.hpp.final-pre-mut` identical; the marker grep over `include test docs src tools` empty; no
  binary. The briefing was not generated while that binary existed.
- **Fixed — the root:** `scratch/imu1-a/mutate.py` and `swap.py` (both put worktree files into a temporary state
  and restore in `finally`) now (a) turn SIGTERM and SIGHUP into an exception, so `finally` runs, and (b) run every
  child in its own process group and kill the group on any exception, so an interrupted build cannot run on as an
  orphan and link a mutant binary after the restore. The copies from before the change are kept
  (`mutate.v2-nosig.py`, `swap.v1-nosig.py`).
- **The fix tested by trying to break it:** a live `mutate.py M9` killed with SIGTERM 6 s in (the mutant on disk, its
  build running) → exit 130, `KeyboardInterrupt: signal 15`, the model `cmp`-identical to before, no binary, no
  orphan compiler (`pgrep`). Every long job in this round then ran as one mutation per foreground call (each under
  40 s against the 600 s limit) or, for the 16-minute src-gate self-test and the gate's check, in the background
  with this turn kept open until each exited (`git status --short src` empty after both).

### V-2 (major) — the review round's runs, its log section and the stale sections
- **Re-checked, TRUE:** no `fp-e5-*` output; `mut/results.jsonl` M1–M8 only; the review section only a draft
  (`scratch/imu1-a/draft/review-section.md`) with seven placeholders, while this log already pointed at it four
  times; the mutation, fingerprint, gates and "where it stands" sections still the `2ef1f3f` tree's (2 643 cases).
- **Fixed:** M1–M13 all run on the final tree — M1–M8 again too, so the table is one tree's (their failing sets
  identical, case for case, to the review round's own run); the fingerprint against `e5a69b5`; the reproduction
  re-seen on the final tree; every gate; the prose review's greps re-run and logged. The draft filled and appended
  above as "IMU1-A review findings and their disposition"; the five stale sections marked SUPERSEDED in place
  (dated, pointing to it — not rewritten); M9's three new red cases (L1–L3) are in the table and said under it.
  The log's four pointers to the review section (:45-46, :237, :388, :396) now resolve.

### V-3 (minor) — the test file's header contradicted itself on reproduction-first
- **Re-checked, TRUE** (`test/imu_rate_once_per_tick_test.cpp` :3-6 say T12 and L1–L3 were written after the fix;
  :19-20 said every test there was written before it). **Fixed:** :19 now reads "every §2.2 test here (R0, T1–T11)
  was written and run against the UNCHANGED adapter before the fix" (the paragraph re-wrapped; a comment only — the
  TU recompiled, and every run above is on it).

### V-4 (minor) — the changelog's pre-IMU1 "30" stood under "pinned by IMU1's tests L1–L3"
- **Re-checked, TRUE:** the 30 comes only from runs with the old adapter put back — `CHECK( 30 == 60 )` at all three
  phases in `r-repro.txt` (18:57), the verifier's `verify/old-L.txt` and this round's `vr/repro-final.txt` (20:19);
  no suite test can hold it, because that adapter no longer exists. **Fixed at every site that quotes it:** the
  changelog ("where the adapter before IMU1 lost 30 — measured on the host test plant on 2026-10-29 with that
  adapter put back; no test holds that figure"); HA-215 (it said "re-run on the adapter before IMU1 too" — now:
  measured with that adapter put back, the same 30 at all three phases, which no suite test can hold);
  build-order's status line ("pin every figure the changelog quotes but one"). `ProsImu`'s banner quotes no
  pre-IMU1 figure — left.

### Found by this round's own prose review, and fixed
- **HA-22's IMU1 amendment** repeated "0 or a spike" without "derived" (landmine 14) — now "DERIVED from the
  adapter's code and PROS's documented 10 ms copy (HA-213), never measured on a robot".
- **build-order's status line** said "Its tests were written FIRST" and that the three limits "are pinned at the
  figures the changelog quotes" — both over-claims once T12 and L1–L3 joined after the fix; now exact.
- **REMAINING-WORK row 32** — "its review's and its verification round's findings fixed … awaiting the
  coordinator's check", level with build-order.

### Where this round's evidence is
`scratch/imu1-a/vr/`: `mut-results.jsonl` (M1–M13), `sigtest.*` (the interrupt test), `suite-1.txt`,
`fp-e5-base-{1,2}.txt` and `fp-e5-partA.txt` with `shulib_tests.{base,partA}-e5`, `repro-final.txt`, `all.cpp`,
`ctest-refusals.txt`, `srcgate-selftest.txt`, `srcgate-check.txt`, `M9.killed-1910.out` (the killed run's output,
kept); `scratch/imu1-a/mut/M*.out` (each mutant's whole-suite output). Backups of every file this round edited:
`vr/*.before-vr` and `vr/*.final`.

### The final gates, after the last document edit *(`date` 20:23–20:27, the last lines as printed)*
- `rm -f build/test/shulib_tests`; `api_doc_tool.py check-fresh` → exit 0 (no `///` changed in this round); the
  briefing generated with no binary; built (exit 0); `(ulimit -v 4194304; timeout 600 python3 tools/suite_run.py
  green)` → **`suite run: green — 2723 cases, 2936803 assertions`**; the briefing regenerated with the fresh binary
  ("2,723 cases / 2,936,803 assertions, 3 skipped — green"; OPEN CHUNKS lists IMU1; "181 registered, 7 settled —
  next free is HA-216"); built again.
- The doc gates: `api_doc_tool.py self-test` OK; `check-coverage` → 5060 public entities across 188 headers, all
  documented; `check-fresh` exit 0; `check-examples` → 603 quoted lines, all verbatim; `check-removability` → no
  public doc references `docs/internal/`; `briefing_status.py self-test` OK; `check` → generated block current;
  `doc_staleness_audit.py self-test` OK; the audit → clean (31 live docs).
- `grep -rn '// M[0-9]*:' include test docs src tools` → empty. Not committed; no `pros` command; no
  `/dev/ttyACM*`.

## The coordinator's merge verification — IMU1 Part A (2026-10-29 20:40 – 2026-10-30 08:40, main `e5a69b5` + the squash of `imu1-a`)
- **Audit before the merge:** `audit_wt.py imu1-a` — 3 backed-up files, 0 suspicious (after the verification-fix round
  restored the M9 mutant the verifier found live, V-1); `git diff --stat -- src/ tools/ Makefile` empty; no stray root file.
  The transport commit `665babc` (the branch was already rebased onto `e5a69b5` by the fix round), then `git merge --squash
  imu1-a` — **no conflicts**; the changelog's "Fixed since 2.5" sits at the top of `## API 2.5`, above CAL1-A2a's.
- **Gates on the merged tree** (`.claude/coordinator/logs/imu1_a_merge_verify.out`): host build with the doc gates; suite
  **2,723 | 2,723 passed, 3 skipped** (main's 2,701 + IMU1's 22); `src_build_gate.py` self-test OK and check **PASS, all 7
  builds**; ARM header gate over **228** headers clean; CI greps empty; api check-fresh, the briefing, removability and the
  staleness audit clean; the compile-refusal harness **251 of 251**; no plant, no stray root file.
- **The coordinator's hand mutations** (`muts_imu1_a_main.json`, on `hal/pros/imu.hpp`; each on a fresh link, the whole
  suite under a 4 GiB / 300 s cap, restored and hash-checked): **6 of 6 caught** — MI1 (the hold's edge inclusive: a read
  exactly 5 ms later still held), MI2 (no hold at all), MI3 (a held read extends the hold), MI4 (the gyro source not held),
  MI5 (a faulted read moves the derivative's sample time), MI6 (the derivative's instant read after the device) RED.
  Restored tree green.
- **The coordinator's own tool, from V-1's root cause:** `.claude/coordinator/mutate.py` had the same flaw as the builder's
  (a SIGTERM or SIGHUP skipped its `finally` restore); it now turns both into an exception, so the restore always runs.
- **Roadmap:** a "Now" paragraph for the part (the builder's run touched no roadmap line). IMU1 stays an OPEN chunk in the
  briefing until Part B (the bench stage, worksheet Station N) and its completion record.
- **Next for IMU1:** Part B, on a free slot; Station M's floor runs (M.6) and K.5 may use this build from now on.

---

## Part B — the bench stage that measures the IMU on the robot (D3 (a)) *(started `date` 15:29, 2026-10-30)*

- **Chunk:** IMU1 Part B ONLY — Bench Tests button 2's STAGE 2c (the IMU's copy cadence, its phase against a paced
  wake, a stand-in for a motion tick's span, the gyro's integrated z beside the raw rotation's change) and worksheet
  Station N. Brief: [`IMU1-imu-rate-once-per-tick.md`](IMU1-imu-rate-once-per-tick.md) — its answers block and
  "Re-check — amendments" R1–R18 (R10 and R18 are Part B's) OVERRIDE its body.
- **Executor model:** opus (Claude Opus 5.5, 1M context), a workflow-spawned executor.
- **Tree at start:** `eab0f89` (main), clean, in the git worktree `.claude/worktrees/imu1-b` (branch `imu1-b`) — NOT
  the main working tree. Part A is in it (`6798aa5`).
- **Go:** the team lead's "Accept all (Recommended)" of 2026-10-22 (D1–D4; Parts A then B). Do not commit; no `pros`
  command; no `/dev/ttyACM*`. Scratch and mutation backups under `.claude/coordinator/scratch/imu1-b/` only. Every test
  binary under `(ulimit -v 4194304; timeout 600 …)`. **Host only: nothing here has run on a robot; the stage's numbers
  come from Station N, run by the team.**
- **In flight beside this run (not edited here):** MECH-A6 (worktree `mech-a6`: `src/programs/bench_r3a.{hpp,cpp}` —
  the 3 × 5 menu, the grid, station 13; this run touches ONLY station 2's functions, each named below; whichever lands
  second rebases); VIS1-A1 (merging: the camera device layer); G3a Part 1 (merging next); L6-A1; CAL1 Part A2b; the
  STEP1 and MECH-S/TUNE1 briefs. Shared plan docs, the worksheet, the changelog and the HA register: own lines only.

### RESUMED — Tue Nov 3 2026, `date` 12:49 *(the run of 2026-10-30 stopped at ~15:40 when the team lead paused all work)*
- **What the stopped run left:** this section's header block above and nothing else — `git status` showed only this
  log modified; `git diff -- src/ Makefile` EMPTY (no gate self-test plant, no mutant, no stage code); the scratch
  directory `.claude/coordinator/scratch/imu1-b/` empty; no `build/`, `bin/` or `.d/` in the worktree. So nothing to
  restore and nothing to redo: Part B starts here from the reading.
- **Base still `eab0f89`** (`git log -1`); main has VIS1-A1 uncommitted on it and a 2-line change at the end of
  `runR3a()` in `bench_r3a.cpp` (read only, `git diff --stat` in the main tree) — neither touches station 2.
- **Fresh build first:** `rm -f build/test/shulib_tests`, `briefing_status.py generate` (the binary absent, so read
  as unbuilt), `cmake -S test -B build/test`, then the build (below).

### 12:56–13:05 — the reading, the base build *(appended at `date` 13:29)*
- **Read** (the team lead's 2026-10-22 reading ruling): `RESUMING.md`; this log whole for Part A's hand-ons (its
  "Where Part A stands now" and the coordinator's merge record: Part B, HA-04/HA-109's *Settle*, the completion
  record after Part B); the brief's header and answers block, "Re-check — amendments" R1–R18 in full, the Decisions,
  the plain words, §1 Part B (items 5–6), §2.1/§2.3/§2.5, §3.3, §3.4, §4 (HA-213..215 rows, the blast radii, the
  amended entries), §5, §6, §7 (Part A and Part B), §8 in full and §9; the rest by heading. Then the code it cites,
  at `eab0f89`: `src/programs/bench_r3a.cpp` (the banner, the includes, `emit`/`emitf`/`emitS`, the screen helpers,
  `waitForTouch`, `twoButtonPrompt`, `panelLiteralFits`, STAGE 2's `reportImu` whole, STAGE 3's `reportMotorGroup`,
  STAGE 5's `measureLoopRate`, `tImu`/`tAll`/`kMenu`), `hal/pros/imu.hpp` (Part A's hold, the gyro branch — both
  sources through one `imuYawRateToCanonical`), `hal/pros/char_sink.hpp` (`serialLine`, `serialWrapped`), the
  vendored `pros/imu.h` (`imu_get_rotation`, `imu_get_gyro_rate` → `imu_gyro_s_t`), `pros/motors.h`
  (`motor_get_position`/`_actual_velocity`/`_current_draw`/`_voltage`), `pros/misc.h` (`battery_get_voltage`),
  `pros/rtos.h` (`micros()` → `uint64_t`), `pros/error.h` (`PROS_ERR`, `PROS_ERR_F` = `INFINITY`),
  `src/robots/chassis_table.hpp` (read only: `imuPort`, `left`/`right`, `absPort`), the worksheet (Station A, 4,
  G.1, H, I), `docs/hardware-assumptions.md` (HA-04, HA-109, the IMU1 section), `test/source_pins.hpp`,
  `test/bench_menu_grid_test.cpp` (the PROS-free `src/` header precedent), `.github/workflows/ci.yml`.
- **Station N is still free** at the tip (`grep -rn "Station N"` over `docs/`: only IMU1's own Part A lines name it).
  MECH-A6 (read only, its worktree's `git diff`) adds a **Station P** at the SAME place (after Station I) and two
  `#include`s near mine (after `robots/chassis_table.hpp`, after `hal/pros/controller.hpp`) — whichever lands second
  re-places its block (§6).
- **Deltas between the brief's base `ceb17c0` and mine `eab0f89`, for Part B's files:** `bench_r3a.cpp`'s line numbers
  moved (STAGE 2 is now `reportImu` :736, `tImu` ~:2914, `tAll` ~:2928, `kMaxMenu` :330) — re-anchored by function
  name; STAGE 5 still paces with `pros::Task::delay_until(&prev, 10)`; the file still includes neither
  `hal/pros/tick_pacer.hpp` nor `localization/`; `emit()` is SER1's `serialWrapped(nullptr, line)`. Nothing else the
  brief's Part B cites moved in meaning.
- **The base, fresh:** `cmake -S test -B build/test` + `cmake --build build/test -j8` → 49 s, exit 0, 0 warnings;
  `(ulimit -v 4194304; timeout 600 python3 tools/suite_run.py green)` → **green — 2721 cases, 2937848 assertions**.
  A copy of the base tree (`git archive eab0f89`) built in `scratch/imu1-b/base-tree/` gives the base binary
  (`scratch/imu1-b/shulib_tests.base`) for the per-case comparison below.

### 13:05–13:29 — the stage, its pure half, the tests *(appended at `date` 13:29)*
- **NEW `src/programs/imu_timing.hpp`** (PROS-free, library-free, allocation-free; namespace
  `shulib::bench::imu_timing`): every number the verdicts use, each a stated choice (`kCopyPeriodUs` 10 000,
  `kBandUs` 1 000, `kMinIntervals` 20, `kPhaseWindowUs` 9 500 — the brief's, `kMinPhaseTicks` 20,
  `kFixedDriftUsPerS` 10, `kHoldUs` 5 000 = `ProsImu::kRateHoldUs` (static_asserted in both the bench TU and the
  host test), `kSpanWellUnderUs` = half of it, `kGyroMinWindowUs` 2 s — the brief's, `kGyroMinTurnDeg` 30,
  `kGyroTolerance` 0.10 — the brief's "~10 %"); `Summary`/`summarize` (lower median); `usBetween`; `ChangeWatch` (a
  change is an EXACT inequality — HA-213's same copy is the same double; a PROS_ERR_F read counted and skipped);
  `cadenceFrom` + `judgeCadence`; `TickPhase`, `unwrapNear` (step in (−P/2, +P/2]), `phaseFrom` (least-squares drift
  of the unwrapped offset) + `judgePhase` (no verdict unless 2c.1 found the 10 ms copy) + `secondsToCrossPeriod`;
  `judgeSpan` (on the LONGEST span); `GyroTurn` (the trapezoid between consecutive good polls; the window from the
  FIRST to the LAST change; a non-finite value or a clock gone backwards dropped whole; travel inside the window) +
  `judgeGyro` (the sign judged only once the magnitudes agree) + `looksLikeRadians` + `mixedTurn` (STAGE 2's rule).
- **`src/programs/bench_r3a.cpp` — the functions touched, every one (the coordinator's ask; MECH-A6 owns the rest):**
  (1) the include block, +1 line `#include "programs/imu_timing.hpp"` (:75 — outside station 2's functions: see
  the deltas); (2) a forward declaration `void imuTimingStage(std::uint8_t port);` just above STAGE 2's banner;
  (3) `reportImu()` — +4 lines at the END of its `try` block, after the HA-02 verdict: the call
  `imuTimingStage(kChassis.imuPort);` (so an adapter that REFUSED the device is not timed); (4) NEW, between
  `reportImu()` and STAGE 3's banner: `namespace imu2c` (the panel words, `panel`, `foot`, `ul`, `collectCadence`,
  `collectPhase`, `collectSpans`, `collectGyro`, `reportCadence`, `reportPhase`, `reportSpan`, `reportGyro`) and
  `imuTimingStage()`. **Untouched:** `tImu`, `tAll`, `kMenu`, `MenuItem`, `kMaxMenu`, `g_verdict`, `menu_grid`,
  `bench_r3a.hpp`, every other station's function, STAGE 5. `git diff -U0` hunks: `@@ -74,0 +75 @@`, `@@ -734,0
  +736,2 @@`, `@@ -922,0 +926,4 @@ void reportImu`, `@@ -932,0 +940,455 @@` — nothing else.
- **The stage, in order:** `rule("STAGE 2c ...")` and six lines of what it will do; `twoButtonPrompt` START / SKIP
  (SKIP logs and returns — 7 RUN ALL reaches it through `tAll → tImu → reportImu`); a fixed panel `TURN IT SLOWLY` /
  `ONE WAY - KEEP TURNING`, 2 s lead-in; **2c.1** a 2 s tight loop of `imu_get_rotation` with `micros()` read before
  each read (Part A's clock-first), the change instants into a static 512-slot buffer; **2c.2** 100 ticks on
  `pros::Task::delay_until(&prev, 10)`, each reading the rotation at the wake (the baseline) then polling until it
  changes or 9.5 ms; **2c.3** `STOP - LET GO`, 100 ticks on the same pace, each: `micros()`, the tick's first IMU
  read, every drive motor's `motor_get_position`/`_actual_velocity`/`_current_draw`/`_voltage` by `|port|` (raw, as
  STAGE 3 reads), the battery, ONE `hal::pros::serialLine("[R3A] imu2c span ...")`, then `micros()` as the instant of
  the tick's second IMU read, and that read; **2c.4** `NOW THE GYRO`, a touch, 2 s lead-in, a 3 s tight loop of
  instant + rotation + `imu_get_gyro_rate().z` into `GyroTurn`; `STOP - DONE`; then the scroll back, the header
  redrawn, and the four result blocks through `emit`/`emitf`/`emitS`. It commands no motor, writes no device, never
  calls `ProsImu::yawRate()`, adds no pacer or Localizer include. Every MEDIUM / SMALL panel literal is
  static_asserted with the file's own `panelLiteralFits` (28 / `kScreenCols`); verdict lines cut to ≤ 56 characters
  so the panel's 54-column cut keeps the verdict.
- **`make ROBOT=tank`** (from `rm -rf bin .d`): exit 0 in 8 s, `bin/hot.package.bin` 69 768 bytes; the only warnings are
  the vendored `liblvgl` header's (present at the base too) — none from `bench_r3a.cpp`'s own lines.
- **NEW `test/bench_imu_timing_test.cpp` — 32 cases** (prefix `IMU1-B`): the constants pinned to the library's hold;
  `summarize` (lower median), `usBetween` (backwards → 0, the uint32 edge exact, one past it clamped); `ChangeWatch`
  (baseline, no tolerance — `nextafter` is a change, PROS_ERR_F / NaN skipped); 2c.1 over a sample-and-hold model of
  HA-213 polled as the stage polls (10 ms at 3 phases × 3 poll steps → HOLDS; 5 ms → FASTER; 20 ms → SLOWER; still
  → NO CHANGE; a robot still for 40 % of the loop → the median still HOLDS and the long interval stays out of the
  mean; the band's ends exactly; every verdict edge; a short scratch; a backwards instant); `unwrapNear` at ±P/2
  exactly; 2c.2 (a fixed phase → drift exactly 0, FIXED; 1 µs/tick slower copy → 100 µs/s with one late wake that
  would make endpoints read ~39; a copy drifting THROUGH the wake → +10 000 µs/s unwrapped, missed ticks counted; a
  copy AT the wake → half the ticks missed, FIXED; missed ticks never in the median; no drift from one tick or one
  instant; every verdict edge — 10.0 µs/s exactly FIXED, `nextafter` DRIFTING, no verdict without the cadence);
  2c.3 (every edge; one 5.2 ms tick among 99 of 0.4 ms → OVER); 2c.4 (deg/s same sign at ±90 and 25 deg/s → AGREE;
  opposite → OPPOSITE SIGN; rad/s → SCALE OFF + radians hint; another axis → SCALE OFF, no hint; still → NO TURN;
  1.5 s of turn → TOO SHORT; the trapezoid EXACT to 1e-8 for a linear z on irregular steps with garbage z before the
  first and after the last change; dropped polls bridged, a backwards clock dropped, two polls in one µs add no area;
  travel inside the window; every verdict edge — 0.10 × 100.0 == 10.0 exactly, so 110 and 90 land ON the tolerance);
  and four SOURCE PINS over the stage's code (comments stripped): the header PROS- and library-free and the bench
  free of `tick_pacer.hpp` / `localization/`; the stage free of every motor command, device write, `yawRate(`,
  `ProsTickPacer`, `localization::` and `#include`, and reading exactly what it says; printing only through
  `emit*` and ONE `hal::pros::serialLine(`; `reportImu` calling it after the HA-02 verdict and before the catch,
  `tAll` → `tImu` → `reportImu`, and the START / SKIP prompt before the first read.
- Build: exit 0, no warning (the host's `-Wpedantic -Wshadow -Wconversion -Wsign-conversion -Wdouble-promotion
  -Werror`). `-tc="IMU1-B*"` → **32 / 32 passed, 389 assertions**. The whole suite → **green — 2753 cases** (2721 +
  32). **Per-case comparison against the base binary** (doctest's XML reporter, both run under the cap, each from its
  own tree): no case went missing, the 32 are new, and exactly five existing cases changed their ASSERTION COUNT
  (still 0 failures each) — all five scan source files and so count the two new files and the new lines:
  `P3-T3: "shulib Teleop" is retired everywhere …` (14 067 → 14 315), `P3-T3: the buttons left the mechanism table …`
  (2 315 → 2 323), `P3-T3: the library's loop is called only by …` (46 → 47), `SAFE-SAVE A scan: no rename, remove …`
  (274 → 275), `pin: no src/ code prints through stdout's FILE …` (1 292 → 1 322). Outputs:
  `scratch/imu1-b/{base,mine-1}.xml`.

### The deltas from the brief, each with its reason *(appended at `date` 13:29)*
1. **A new PROS-free header and ONE new `#include` in `bench_r3a.cpp`** — §7 Part B item 2 says "the diff adds no
   `#include` (no `tick_pacer.hpp`, no `localization/` — R10)". The coordinator's task asks for host tests of the
   stage's pure parts, and the host cannot compile `bench_r3a.cpp`; so the arithmetic lives in
   `src/programs/imu_timing.hpp` (the `bench_r3a.hpp` / `auton_tester_run.hpp` precedent). It includes only
   `<algorithm> <cmath> <cstddef> <cstdint> <limits> <span>` — R10's concern (pulling the motion scheduler or a
   Localizer into Bench Tests) is kept, and pinned by a test. **The include line is outside station 2's
   functions** (the file's include block, :75); MECH-A6 adds includes two lines below it and further down — a
   one-line, mechanical rebase for whichever lands second. **For the coordinator.**
2. **The stand-in's serial line goes through `hal::pros::serialLine()`, not `emit()`** — the brief: "one serial line
   through the stage's one-call path". `serialLine` IS SER1's one-call path; `emit()` would ALSO paint the panel
   (PROS 4.2.2's `print_at` leaks per call — R3d Part 2F-2, the file's own note — 100 times) and write the SD card,
   which a motion tick's record does not do on robot two. Every RESULT line goes through `emit`/`emitf`/`emitS`.
3. **The span is bracketed by two real IMU reads** — the brief times "the device reads … between its two IMU
   reads"; the stage makes the tick's first IMU read inside the span and stamps the instant of the second (clock
   first, as `ProsImu` does), so the span is first-read-instant to second-read-instant.
4. **START / SKIP, and a touch before 2c.4** — the brief asks that it "say on the panel what to do before it
   starts reading"; the START / SKIP prompt does, and lets a RUN ALL or an HA-02-only run pass it by (`kMenu`'s
   texts cannot change — MECH-A6's). The gyro part needs a different speed (the brief's "about a quarter turn a
   second"), so it waits for a touch and gives a 2 s lead-in rather than change instructions mid-turn on a frozen
   panel.
5. **The thresholds the brief left open, chosen and stated in the header** (each with its reason): the cadence band
   9 000–11 000 µs and ≥ 20 intervals, judged on the median (the brief: "cluster at 10 000 µs"); |drift| ≤ 10 µs/s
   as FIXED over the 1 s, the drift a least-squares slope of the UNWRAPPED offsets (the brief: "its drift per
   second"); 2c.3 judged on the max — under 2.5 ms "well under", ≥ 5 ms wrong (the brief: "well under 5 ms … a
   LOWER bound"); the gyro's 30° minimum turn and a 3 s loop (the brief: first-to-last ≥ 2 s; "within ~10 %").
   Extra printed numbers: 2c.1's in-band count and mean (a precise period), 2c.2's missed ticks, 2c.3's PROS_ERR
   motor reads, 2c.4's travel and the radians hint.
6. **The sign is judged only when the magnitudes agree** (the brief judges sign and ratio side by side): a z on the
   wrong scale or axis says nothing trustworthy about `GyroRateZ`'s sign, and `GyroRateZ` needs both. And 2c.2
   judges nothing unless 2c.1 found the 10 ms copy (its offsets are unwrapped at 10 ms).
7. **The stage runs only after a GOOD STAGE 2** (inside `reportImu`'s `try`, after the HA-02 verdict) — the brief
   says "after STAGE 2 on the same fresh ProsImu + ProsClock"; an adapter that refused the device has nothing to time.
8. **HA-04 / HA-109's *Settle* wording** — the brief's §3.1 text says "both raw CW degrees"; written "both raw PROS
   degrees" with the conventions named (the rotation CW-positive per HA-02; z's sign is the very question), as the
   stage's own lines say.

### The documents (pace rule 3) — §3.3, re-anchored by TEXT at `eab0f89` *(appended at `date` 13:34)*
Public (§3.3: `docs/hardware-assumptions.md` only — plus the roadmap's "Now" paragraph the coordinator's task asks for):
- **`docs/hardware-assumptions.md`** — HA-04's and HA-109's Index rows gain "IMU1 Part B's bench stage (Station N)"
  beside R3; their *Settle* entries gain STAGE 2c (the comparison, both raw PROS degrees, the sign judged only once
  the scale holds; HA-109's radians / other-axis readings), each "built, never run on a robot — not settled until
  the bench prints it"; the IMU1 section's opening ("not built yet" → "built 2026-11-03, never run on a robot", and
  "of HA-04 and HA-109"); HA-213's, HA-214's and HA-215's *Settle* made exact to what the stage judges (the median
  of ≥ 20 intervals in 9 000–11 000 µs; first / median / last, the missed ticks, the least-squares drift, ≤ 10 µs/s
  as fixed, nothing judged unless 2c.1 found the copy; the LONGEST stand-in span, < 2.5 ms "well under — not proof",
  ≥ 5 ms wrong); the status block's IMU1 paragraph ("built 2026-11-03, never run on a robot"). **No new HA; nothing
  marked settled** — "Status: 7 of 182 settled" and the Held line unchanged *(corrected 16:33 by the resumed run: this line said "181", Part A's figure; the register reads 182 at `eab0f89`)*. One more line, found by the prose
  review: HA-120's citation of `bench_r3a.cpp`'s registry comment ("lines 135-145 since R2a") was already stale at
  my base (the comment sits at 137–146 there) and my include moves it again — re-worded to "find it by that name"
  with the two old ranges kept as history.
- **`docs/roadmap.md`** — a "Now (2026-11-03)" paragraph after Part A's: IMU1 COMPLETE with Part B, host- and
  build-gated, never run on a robot; what STAGE 2c measures in plain words; the five beliefs unsettled until the
  team's bench run. No link into `docs/internal/` (removability gate clean).
Deliberately NOT touched (public), each re-checked: `docs/changelog.md` (Bench Tests is not library API; no entry
for a bench stage, as for R2a's stations — the changelog's two "HA-04" sentences, :118 and :125, stay true: the
gyro still waits for its sign to be measured); `docs/faq.md` :70 (HA-04 "undocumented" — still true);
`docs/guide/` (no chapter describes button 2's stages: `grep "Bench Tests\|rotate test\|STAGE 2"` → only guide 07's
program list and guide 15's word "the tester"); `docs/api/` (no `///` changed under `include/shulib` —
`check-fresh` silent, exit 0); `README.md`.

Internal:
- **The worksheet** (`chunks/R3a-BENCH-WORKSHEET.md`): NEW **Station N — THE IMU'S TIMING** after Station I (N.1
  upload, as G.1's first line; N.2 run — STAGE 2 as before, then START, the slow turn, hands off, the touch and the
  quarter-turn-a-second turn, N.2.1 what the operator did, N.2.2 a second boot; N.3 the four blocks word for word
  with a table; N.4 what each number settles; what to send back). **Every quoted string checked against the source by
  script** (the four blocks' 31 lines each matched to a format string of the stage, with the numbers made up; every
  backticked panel string found in the stage's literals — the one exception, `WHICH WAY DID YOU TURN IT?`, is STAGE
  2's own). Station 4 gains its one pointer line after the heading; Station A's row for button 2 gains "then STAGE
  2c: turn it slowly ONE way when asked (**Station N**)". **For the coordinator:** MECH-A6 inserts its Station P at
  this same point (after Station I) — whichever lands second places its station after the other's.
- **`build-order.md`** — the IMU1 section gains the status line "2026-11-03: PART B BUILT — IMU1 COMPLETE … awaiting
  the coordinator's check" (above Part A's, which I leave as it was written). The `**Next:**` block NOT moved (the
  coordinator's).
- **`REMAINING-WORK.md`** — row 32: Part A LANDED 2026-10-30 (`6798aa5`); Part B BUILT 2026-11-03; its robot step
  Station N (~10 min). The totals NOT recomputed (the coordinator's re-pricing, as Part A's D-10).
- **`HANDOFF-remaining-work.md`** NOT touched: its IMU1 sentence (:613-617, "IMU1 Part A BUILT 2026-10-29") is a dated
  pointer inside TANK1's paragraph and stays true; the plan's live status is REMAINING-WORK and build-order.
- **`IMU1-COMPLETED.md`** — NEW (IMU1's last part), in the house style of TANK1's and SER1's records.
- **`PROJECT-BRIEFING.md`** — regenerated after a fresh build (below): IMU1 leaves OPEN CHUNKS with its record.

### The prose review (rule 4) — the greps RUN FIRST (13:25), every hit read, then this written *(§3.4)*
`grep -rIl --exclude-dir=api -F "<term>" docs README.md` (the `chunks/` records excluded from the listing — history),
then each hit opened:
- `bench_r3a` → `HANDOFF-remaining-work.md` :303 (the program list: "Bench Tests … `src/programs/bench_r3a.cpp`" —
  true); `build-order.md` (my status line); `hardware-assumptions.md` :1884 (HA-120's line range — STALE, fixed, above)
  and :2691 (SAFE-SAVE's probe extension — true).
- `STAGE 2` (without "2c") → no hit outside the chunk records and the 2026-08-19 evidence logs (history, not edited).
- `IMU + ROTATE` → only the evidence logs (history).
- `Station 4` → no hit outside the worksheet (edited: the pointer).
- `Station N` → only IMU1's own lines (the register, the roadmap, build-order, REMAINING-WORK) — each now says built.
- `HA-04` → `faq.md` :70, `changelog.md` :118 and :125, the roadmap, the register — all true (dispositions above).
- `HA-109` → `changelog.md` :118, the roadmap, the register — true.
- `HA-213` / `HA-214` / `HA-215` → the register (edited), the changelog's IMU1 section (Part A's — true; it names no
  bench result), the roadmap, build-order, HANDOFF (HA-213 inside TANK1's paragraph — true).
- `imu_timing`, `STAGE 2c`, `button 2` → only my own new lines.
- And `Bench Tests` over `docs/guide`, `README.md`, `docs/*.md` → guide 07 :143 (the program list — true), guide 15
  :25 (the word "tester" — true), the register's and the roadmap's other stations — untouched by this part.

### The mutations — 37, each on a FRESH binary, the WHOLE suite, red observed, restored *(13:13–13:50; appended at `date` 13:51)*
Driver: `scratch/imu1-b/mutate_b.py` (each mutant: one exact replacement that must match once; the file backed up to
`scratch/imu1-b/backup-<id>-*`; its timestamp bumped; `cmake --build build/test -j8` — the build's own output names
every object it recompiled, recorded; then `(ulimit -v 4194304; timeout 600 ./build/test/shulib_tests
--order-by=file)`, the WHOLE suite; restored in a `finally`, SIGTERM and SIGHUP raised as an exception so an
interrupted run restores too; the restored file's sha256 checked and its timestamp bumped). **Every object depending
on the mutated file was recompiled:** the compiler's dependency files name exactly ONE host object that includes
`imu_timing.hpp` — `bench_imu_timing_test.cpp.o` — and every MB mutant's build recompiled it; no host object
compiles `bench_r3a.cpp` (the pins read it at run time), so an MP mutant needs no recompile and its run reads the
mutant text. **No mutant broke its build** (all compile under the host's `-Werror`). The first run of the driver
stopped at MB4 on a decode error in Python (a failing case printed a non-UTF-8 byte) AFTER its `finally` had
restored the file (checked: identical to its backup); the driver now decodes with `errors="replace"`, the binary was
relinked from source and the briefing regenerated around it (the doc gate had caught the mutant link — "stale
briefing" — exactly as it should), and the whole set was run again from MB1. Outputs: `scratch/imu1-b/mut-<id>.out`,
`mut-results.json`, `mut-run.txt`; failing cases from `failed_cases.py` (a case counts only with an `ERROR:` line
under its header). **37 of 37 CAUGHT, all restored** (both source files byte-identical to their pre-mutation copies
afterwards).

| # | Mutation | Red (every failing case) |
|---|---|---|
| MB1 | `summarize`: the UPPER median | summarize; (2c.2) a copy AT the wake |
| MB2 | `ChangeWatch`: a PROS_ERR_F read not screened | ChangeWatch: a PROS_ERR_F read … |
| MB3 | `ChangeWatch`: a 1e-9 tolerance | ChangeWatch: the first good read is a baseline … |
| MB4 | cadence: the band's low end excluded | (2c.1) the band's ends are IN the band |
| MB5 | cadence: the long intervals enter the in-band mean | (2c.1) a robot held still for part of the loop; the band's ends |
| MB6 | `judgeCadence`: median 9000 → FASTER | (2c.1) HA-213's verdict at every edge |
| MB7 | `unwrapNear`: no unwrap at all | unwrapNear; (2c.2) a phase that drifts THROUGH the wake |
| MB8 | `unwrapNear`: +P/2 wrapped | unwrapNear |
| MB9 | phase: drift from the ENDPOINTS | (2c.2) 1 us slower … least squares; drifts THROUGH the wake; a copy AT the wake |
| MB10 | phase: missed ticks enter the median as 0 | (2c.2) a copy AT the wake |
| MB11 | `judgePhase`: judged without the 10 ms copy | (2c.2) HA-214's verdict at every edge |
| MB12 | `judgePhase`: exactly 10 us/s → DRIFTING | (2c.2) HA-214's verdict at every edge |
| MB13 | `judgeSpan`: on the MEDIAN | (2c.3) judged on the LONGEST span |
| MB14 | `judgeSpan`: 5000 us → CLOSE | (2c.3) judged on the LONGEST span |
| MB15 | gyro: a left rectangle, not the trapezoid | (2c.4) the TRAPEZOID … exact for a ramp |
| MB16 | gyro: the integral from the loop's start | (2c.4) AGREE; OPPOSITE sign; the TRAPEZOID; a dropped poll (4 cases) |
| MB17 | gyro: a poll from before the last good one kept | (2c.4) a dropped poll … a clock gone backwards |
| MB18 | `judgeGyro`: exactly 10 % → SCALE OFF | (2c.4) HA-04/HA-109's verdict at every edge |
| MB19 | `judgeGyro`: the sign before the scale | (2c.4) RADIANS / another axis; verdict at every edge |
| MB20 | `judgeGyro`: a 2.000 s window → TOO SHORT | (2c.4) verdict at every edge |
| MB21 | gyro: the step into the first change counted as travel | (2c.4) travel inside the window |
| MB22 | `mixedTurn`: exactly a fifth → MIXED | (2c.4) travel inside the window |
| MB23 | gyro: a PROS_ERR_F z integrated | (2c.4) a dropped poll |
| MB24 | `judgeCadence`: 20 intervals → TOO FEW | (2c.1) HA-213's verdict at every edge |
| MB25 | `usBetween`: an equal pair is 1 us | usBetween |
| MB26 | `judgeGyro`: a NaN turn passes the 30° check | (2c.4) verdict at every edge |
| MP1 | the stage: `motor_move_voltage` in the stand-in | pins: STAGE 2c reads only |
| MP2 | the stage: a `ProsImu` built and `yawRate()` read in 2c.4 (R18) | pins: STAGE 2c reads only |
| MP3 | `reportImu` never calls the stage | pins: reportImu runs STAGE 2c … |
| MP4 | the stage run on the REFUSED (catch) path | pins: reportImu runs STAGE 2c … |
| MP5 | one result line through `std::printf` | pins: prints only through SER1's one-call path; AND SER1's own `pin: no src/ code prints through stdout's FILE` |
| MP6 | the bench includes `hal/pros/tick_pacer.hpp` (R10) | pins: imu_timing.hpp PROS-free … no pacer |
| MP7 | `imu_timing.hpp` includes `pros/rtos.hpp` | pins: imu_timing.hpp PROS-free … |
| MP8 | no START / SKIP prompt | pins: reportImu runs STAGE 2c … (the prompt before the first read) |
| MP9 | the stand-in's line through `emitf` (panel + SD) | pins: prints only through SER1's one-call path |
| MP10 | 2c.2 paced by `pros::delay`, not `delay_until` | pins: STAGE 2c reads only (… paced as STAGE 5) |
| MP11 | 7 RUN ALL no longer runs station 2 | pins: … and 7 RUN ALL still runs station 2 |

Not counted: `kHoldUs` moved off `ProsImu::kRateHoldUs` — a build-time refusal by design (the `static_assert` in
both `bench_r3a.cpp` and the host test), not a test verdict.

### RESUMED AGAIN — Tue Nov 3 2026, `date` 15:49 *(the run stopped at ~14:50 when the team lead dropped to ONE job at 93 % of the session limit; resumed after the reset)*
- **What the stopped run left, checked before anything else:** `git status` — the eight modified files and three new
  ones this log names; `git diff --stat -- Makefile src/ tools/ include/ test/` → ONLY `src/programs/bench_r3a.cpp`
  (+462); `grep -rn "self-test plant" Makefile src/` → EMPTY (the coordinator had already restored the Makefile the
  killed `src_build_gate.py self-test` had planted, with `git checkout`). Both mutated sources are byte-identical to
  their last mutation backups (`sha256sum`: `imu_timing.hpp` = `backup-MB26` = `5280c27d…`, `bench_r3a.cpp` =
  `backup-MP11` = `7f23e4c6…`) — no live mutant. `build/test/shulib_tests` (13:48) and `bin/` (14:07, the killed
  self-test's) are NOT trusted: the binary is removed and rebuilt from source before any result below.
- **Where it stopped:** after the 37 mutations (logged above). NOT yet done, and done below: the "every other bench
  station unchanged" proof (the object comparison had been run but not logged), the gates (the gate self-test was
  the one killed), the final review of the completion record and the roadmap paragraph, and the closing section.

### 15:50–15:56 — the resumed run's own review of the stage, before any gate *(appended at `date` 15:57)*
Read whole, against the brief's §1 item 5, R10, R18 and landmines 15–17: `src/programs/imu_timing.hpp`, the stage
(`git diff -- src/programs/bench_r3a.cpp`), the station runner (`runR3a()`'s dispatch: `kMenu[choice].run(g_found)`,
then the verdict chip and `TOUCH TO RETURN TO MENU`), `rule()`, `screenEmit()`, `setVerdict()`, `twoButtonPrompt()`,
`waitForTouch()`, and the test file's model and pins. Fresh build first: `rm -f build/test/shulib_tests`, briefing
regenerated (binary absent), `cmake -S test -B build/test`, `cmake --build build/test -j8` → exit 0 in 47 s, no
warning; `(ulimit -v 4194304; timeout 600 python3 tools/suite_run.py green)` → **green — 2753 cases, 2938525
assertions**; `-tc="IMU1-B*"` → 32 / 32, 389 assertions.
- **FOUND, and fixed — STAGE 2's results were erased from the panel the moment its HA-02 verdict printed.** Before
  Part B, `reportImu()`'s last lines (PATH, NET, the HA-02 verdict) stayed on the panel until the runner's `TOUCH
  TO RETURN TO MENU`. With STAGE 2c called at the end of `reportImu()`'s `try`, its `rule()` (which redraws the
  header and clears the scroll when the panel is live) and then `twoButtonPrompt()` (which erases the whole screen)
  ran at once — the operator never saw STAGE 2's verdict on the panel (it reached serial and the SD card only).
  That changed station 2's own, already-run behaviour, which this part must not. **Fix (station 2's functions
  only):** `imuTimingStage()`'s first act is `imu2c::foot(kColWarn, imu2c::kReadStage2)` —
  `STAGE 2 done -- read it, then TOUCH for 2c` (43 characters, added to the SMALL-line `static_assert`) — and
  `waitForTouch()`; then the rule and the prompt. **Pinned:** the pins case "reportImu runs STAGE 2c …" (renamed
  "… holds that verdict on the panel, and …") gains three checks — exactly two `waitForTouch();` in the stage, the
  first before `rule(`, and `rule(` before `twoButtonPrompt(` — and a new mutation MP12 (the hold deleted) is RED
  (below). The worksheet's N.2 step 1 says what the panel shows and to touch; Station A's **7 RUN ALL** row gains
  "then START (Station N) or SKIP STAGE 2c" (the prompt reaches RUN ALL too — a shared line, one clause, logged for
  the coordinator: MECH-A6 also edits Station A's table).
- **Checked, and kept (no change):** (1) station 2's verdict chip now ESCALATES with STAGE 2c's verdicts —
  `setVerdict()` keeps the worst severity of the whole run, so a 2c warning ("turn it and re-run") or a WRONG makes
  button 2's chip amber or red. Kept: the chip is the station's, and 2c is now part of the station; a SKIP prints no
  severity and leaves STAGE 2's chip as it was. Said here because it is a visible change to station 2's grid colour.
  (2) Many of the stage's DATA lines run past the panel's 54 columns (the longest, 2c.1's in-band line, ~83 with
  numbers) and are cut there, as STAGE 2's own PATH line (~65) always was; every verdict line keeps its verdict
  word inside 54, and Station N.3 copies the blocks from the SD card or the serial capture, which carry them whole.
  (3) The busy loops' priority assumption, the 512-change buffer's overflow line, `phaseFrom`'s replayed unwrap
  (the second pass starts from the same first offset, so the two passes unwrap identically), `unwrapNear`'s
  (−P/2, +P/2] at both ends, `judgeGyro`'s NaN-safe `!(turn >= …)`, and `usBetween`'s clamp — each re-derived; no
  defect.

### 15:53–16:01 — the stage-code mutations RE-RUN on the fixed stage, and MP12 *(appended at `date` 16:03)*
The hold changed `bench_r3a.cpp` and the pins case, so every MP mutant was re-observed on the new text. Driver
`scratch/imu1-b/mutate_b2.py` — `mutate_b.py` with MP12 added and its outputs renamed (`backup2-*`, `mut2-*.out`,
`mut2-results.json`, `mut2-run.txt`; failing cases by `failed_cases2.py`, a case counting only with an `ERROR:` line
under its header); the same discipline (one exact replacement matching once, the timestamp bumped, `cmake --build`
recording every recompiled object, the WHOLE suite under `(ulimit -v 4194304; timeout 600 …)`, restored in a
`finally` with SIGTERM / SIGHUP raised as an exception, the restored file's sha256 checked). **12 of 12 CAUGHT, all
restored** (`bench_r3a.cpp` = `backup2-MP1` = `7b125ee0…` afterwards; `imu_timing.hpp` = `5280c27d…`, unchanged):

| # | Mutation | Red (every failing case) |
|---|---|---|
| MP1 | a `motor_move_voltage` in the stand-in | pins: STAGE 2c reads only … (R18) |
| MP2 | a `ProsImu` built and `yawRate()` read in 2c.4 (R18) | pins: STAGE 2c reads only … (R18) |
| MP3 | `reportImu` never calls the stage | pins: reportImu runs STAGE 2c …, holds that verdict on the panel, … |
| MP4 | the stage run on the REFUSED (catch) path | pins: reportImu runs STAGE 2c … |
| MP5 | one result line through `std::printf` | pins: prints only through SER1's one-call path; AND SER1's `pin: no src/ code prints through stdout's FILE …` |
| MP6 | the bench includes `hal/pros/tick_pacer.hpp` (R10) | pins: imu_timing.hpp PROS-free … (R10) |
| MP7 | `imu_timing.hpp` includes `pros/rtos.hpp` (recompiled `bench_imu_timing_test.cpp`) | pins: imu_timing.hpp PROS-free … |
| MP8 | no START / SKIP prompt | pins: reportImu runs STAGE 2c … |
| MP9 | the stand-in's line through `emitf` (panel + SD) | pins: prints only through SER1's one-call path |
| MP10 | 2c.2 paced by `pros::delay`, not `delay_until` | pins: STAGE 2c reads only … |
| MP11 | 7 RUN ALL no longer runs station 2 | pins: reportImu runs STAGE 2c … |
| **MP12** | **the hold deleted — STAGE 2's results erased at once (NEW)** | pins: reportImu runs STAGE 2c …, holds that verdict on the panel, … |

**The MB set (MB1–MB26, on `imu_timing.hpp`) is NOT re-run, and its 26 verdicts stand:** the header is byte-identical
to the one they mutated (`5280c27d…`), its only host dependent is `bench_imu_timing_test.cpp.o`, and the test file
changed only by three added CHECKs inside the pins case that reads `bench_r3a.cpp` — a case none of the 26 was
caught by (`failed_cases.py`: 0 of the MB rows name it) — so every case that turned each MB red is unchanged.
**The binary after the run:** removed and relinked from source (below) before any result.

### 15:58 — "every other bench station unchanged", proven on the ARM object *(appended at `date` 16:03)*
The previous run had built both trees but not logged the comparison; re-done on the FIXED stage. `scratch/imu1-b/
base-tree` (`git archive eab0f89`) and `mine-tree` (this worktree's tree; its only difference from the worktree is
`bench_r3a.cpp`, refreshed from `backup2-MP1` = the fixed file) each built with `make ROBOT=tank` (mine: exit 0, only
`bench_r3a.cpp` recompiled; its one warning the vendored `liblvgl/core/lv_obj_style.h`'s, which the base build
shows 8 times; `bin/hot.package.bin` 69 728 bytes). `objcmp2.py` disassembles `bin/programs/bench_r3a.cpp.o` from
each (`arm-none-eabi-objdump -dr`), masks addresses, immediates fed by relocations and the compiler's numbered local
labels, and compares function by function:
`functions at base: 117 | identical: 116 | differ: 1 | new: 11 | gone: 0`.
- **The one that differs is `tImu`, and only by inlining:** at the base `reportImu` was inlined into it (738
  instructions); now `reportImu` is its own function (2 069 instructions, the stage inlined into it) and `tImu` is 6
  — `f[imuPort] == E_DEVICE_IMU`, then a tail call to `reportImu`.
- **New:** `reportImu`, `imu2c::panel`, `imu2c::foot`, `ChangeWatch::feed`, `GyroTurn::windowUs`, `summarize`,
  `phaseFrom` and four `std::sort` helpers over `std::span<uint32_t>`.
- **Every other function of the bench object — every other station's, the menu's, the grid's, STAGE 5's,
  `runR3a()` — is instruction-identical to the base.** No other object's inputs changed: against `eab0f89` the only
  `src/`/`include/`/`Makefile` change is `bench_r3a.cpp` (`git diff --stat`), and `imu_timing.hpp` is included by
  `bench_r3a.cpp` and the host test only (`grep -rln`).

### The deltas from the brief — two more, from the resumed run *(appended at `date` 16:33)*
9. **STAGE 2's results are held on the panel until a touch before STAGE 2c starts** (`STAGE 2 done -- read it,
   then TOUCH for 2c`) — the brief does not ask for it; it keeps station 2's existing behaviour (its HA-02 verdict
   readable on the panel) that calling the stage at the end of `reportImu()` would otherwise erase. One more touch
   in a station that already takes three; inside `imuTimingStage()` only.
10. **Station A's `7 RUN ALL` row gains one clause** ("then START (Station N) or SKIP STAGE 2c") — §3.3 names only
   button 2's row, but RUN ALL now meets the same prompt. A shared line (MECH-A6 edits Station A's table): one
   clause, for the coordinator.

### The prose review (rule 4), this round's additions — the greps RUN FIRST (16:32), every hit read
`grep -rIn --exclude-dir=api -F "<term>" docs README.md`, IMU1's own records and the 2026-08-19 evidence logs left
out: `TOUCH for 2c` / `read it, then TOUCH` → only the worksheet's N.2 step 1 (new, and it matches the literal);
`kReadStage2` → none; `RUN ALL` → the worksheet's menu drawing (:75, unchanged), Station A's row (:96, edited) and
N.2 step 2 (true), R2a's log (history); `STAGE 2c` and `Station N` → the roadmap, the register (HA-04, HA-109,
HA-213..215, the IMU1 section, the status line), REMAINING-WORK row 32, build-order's new status line and the
worksheet — each re-read, each true for the fixed stage (none describes the panel's sequence in a way the hold
contradicts). build-order :1208 ("Part B … not started") is Part A's dated status line of 2026-10-29, kept as
written beneath the new one. The earlier round's greps (13:25, above) re-checked at the tip with the same terms: no
new hit.

### The gates — every one re-run on the final tree *(16:05–16:33, the last lines as printed; the doc gates re-run after the last edit, below)*
- Host build from a removed binary: `rm -f build/test/shulib_tests`, briefing regenerated (unbuilt), `cmake --build
  build/test -j8` → exit 0, no warning (it recompiled `bench_imu_timing_test.cpp` and linked); `(ulimit -v
  4194304; timeout 600 python3 tools/suite_run.py green)` → `suite run: green — 2753 cases, 2938530 assertions`;
  `-tc="IMU1-B*"` → `test cases: 32 | 32 passed | 0 failed`, `assertions: 392 | 392 passed`. Briefing regenerated
  AFTER the binary existed (`python3 tools/briefing_status.py generate` → `regenerated`).
- `python3 tools/src_build_gate.py self-test` (from clean; the tree's 1 022 files hashed before, identical after —
  no plant left in `src/`, `include/`, `test/`, `tools/`, the `Makefile` or `common.mk`) → `src build gate
  self-test: OK (269 detector cases, each against a real build)` (24 min 22 s).
- `python3 tools/src_build_gate.py check` → `src build gate: PASS (all 7 builds link; …)`: bench, xdrive, tank,
  tank-drive, tank-calibrate, tank-autontest, tank-competition — each `exit 0, 11/11 TUs compiled, warnings: 8
  vendor-ignored, 0 disallowed`.
- The ARM header gate (`scratch/imu1-b/all.cpp`, 228 headers, `-Werror`) → `ARM compile gate: CLEAN`.
- The CI greps → `library is PROS-free`; `layering holds: core is sim-free`.
- `api_doc_tool.py` self-test `OK`; check-coverage `5060 public entities across 188 headers, all documented`;
  check-fresh exit 0 (silent — no `///` changed under `include/shulib`, `docs/api/` not regenerated); check-examples
  `600 quoted lines, all verbatim`; check-removability `no public doc references docs/internal/`.
- `doc_staleness_audit.py` self-test `OK`; the audit `clean (31 live docs, 0 numeric claims checked)`.
- `briefing_status.py` self-test `OK (38 cases …)`; check `generated block current`.
- Not run here: the compile-refusal harness (no file under `include/shulib` or `test/compile_refusals/` changed).

### Where Part B stands *(for the coordinator — nothing committed)*
- **Built, host- and build-gated, never run on a robot.** STAGE 2c in `bench_r3a.cpp` (station 2's functions:
  the include line, a forward declaration, 4 lines at the end of `reportImu()`'s `try`, the new `namespace imu2c`
  and `imuTimingStage()`); `src/programs/imu_timing.hpp` (new); `test/bench_imu_timing_test.cpp` (new, 32 cases);
  38 mutations RED; every other bench function instruction-identical to the base's.
- **Documents:** `docs/hardware-assumptions.md`, `docs/roadmap.md` (public); the worksheet (Station N, Station 4's
  pointer, Station A's rows for buttons 2 and 7), `build-order.md`, `REMAINING-WORK.md`, `PROJECT-BRIEFING.md`
  (generated), this log, `IMU1-COMPLETED.md` (internal). NOT touched, with why: above ("The documents").
- **For the coordinator:** (1) the include line at `bench_r3a.cpp` :75 sits two lines from MECH-A6's includes — a
  mechanical rebase for whichever lands second; (2) MECH-A6's Station P goes at the same place as Station N (after
  Station I) — the second to land places its station after the other's; (3) Station A's table rows for buttons 2
  and 7 are shared lines MECH-A6 also edits; (4) button 2's verdict chip now escalates with STAGE 2c's verdicts
  (above); (5) HA-213..215, HA-04 and HA-109 stay unsettled — the team's Station N run is the first look; the
  register's "Status: 7 of 182 settled" and its Held line are unchanged.

### The last rebuild and the doc gates after the last edit *(appended at `date` 16:37)*
The staleness audit's first run after the gate self-test said `[cases/assertions NOT derived — the test binary is
older than src/common/boot_source.hpp and 9 more, so it was not trusted]`: the self-test's restores bump the
timestamps of the files it planted in (their contents identical — hashed above). So: `rm -f build/test/shulib_tests`,
the briefing regenerated (unbuilt), `cmake --build build/test -j8` (246 objects recompiled, exit 0, no warning),
`(ulimit -v 4194304; timeout 600 python3 tools/suite_run.py green)` → `suite run: green — 2753 cases, 2938530
assertions`, the briefing regenerated from that binary (`PROJECT-BRIEFING.md`: 38 of 75 chunks, IMU1 out of OPEN
CHUNKS and in the completed list, the suite line 2,753 / 2,938,530). Then, after every document edit of this run:
api_doc_tool self-test `OK`, check-coverage `5060 public entities across 188 headers, all documented`,
check-fresh exit 0, check-examples `600 quoted lines, all verbatim`, check-removability `no public doc references
docs/internal/`; `doc_staleness_audit.py` self-test `OK` and the audit `clean (31 live docs, 0 numeric claims
checked)`; `briefing_status.py check` → `generated block current`. **Nothing committed; no `pros` command; no
`/dev/ttyACM*`; no mutant or plant left** (`git diff --stat -- Makefile common.mk include/ tools/` empty; both stage
files equal their last pre-mutation copies).

## IMU1-B review findings and their disposition *(the review round, from `date` 00:34, 2026-11-05)*

Three reviewers (correctness, tests, docs) returned 18 findings — 4 major, 14 minor, two overlapping (the
correctness lens's X4/X5 and the tests lens's Q1/Q4 are the same wiring gap). Each was re-checked against the
tree before any fix; every one was real, and each is fixed at the root below. **Base unchanged:** `eab0f89`; no
`pros` command, no `/dev/ttyACM*`, nothing committed. Pre-fix copies of the three source files:
`scratch/imu1-b/review-fix/pre-*`. **Every function touched in `bench_r3a.cpp` this round is station 2's:** the
`imu2c` namespace (`collectCadence` / `collectPhase` / `collectSpans` / `collectGyro` REMOVED — their loops moved
to the header; NEW `nowUs`, `standInBody`; `reportCadence`, `reportPhase`, `reportSpan`, `reportGyro` changed) and
`imuTimingStage()`; plus one comment paragraph above `namespace imu2c`. `reportImu`, `tImu`, `tAll`, `kMenu`, the
grid and `bench_r3a.hpp` untouched.

### Correctness
- **C-1 (minor) — HA-214's FIXED / DRIFTING assumed an unmeasured wake jitter.** Re-checked: real.
  `phaseFrom` measures each offset from the actual wake (`runPhase`'s `wake = nowUs()` after the wait), so a late
  wake enters the offset one for one, and the slope's noise is the residual spread over √Sxx — about 4 µs/s at
  20 µs of jitter, 10 µs/s at 50 µs, against a fixed 10 µs/s threshold; the header's "~6 × a few microseconds"
  was a belief (STAGE 5 has never run). **Fix:** `phaseFrom` also returns the kept offsets' min / max, the
  residual SD and the slope's standard error (`seValid` from three seen ticks); `judgePhase` reads DRIFTING only
  when |drift| > 10 µs/s AND > `kDriftSigmas` (3) × SE, FIXED only when |drift| ≤ 10 µs/s AND 3 × SE ≤ 10 µs/s,
  and anything else is the new `NotSeparable` — "no verdict: the wakes jitter too much to tell 10 us/s from none
  in 1 s" (Warn). The stage prints the spread, `drift +x +- y us/s (3 SE; ...)` and `residual SD`. I made FIXED
  symmetric too (the review asked only about false DRIFTING): a FIXED with a 3-SE band wider than 10 µs/s would
  claim what the data cannot show. **Probe** (`review-fix/probe/probe_jitter.cpp`, 400 and 100 draws per point,
  the real `runPhase` on a simulated clock): a fixed phase reads DRIFTING in 0–1 of 400 at σ = 20/30/50 µs (the
  first build's rule: ~1.5 / 11 / 34 %, the reviewer's figures), FIXED in all at σ = 2 and 5 µs, and a real
  100 µs/s drift reads DRIFTING in all at every σ up to 50 µs. **Tests:** the 3-SE and noise edges landing
  exactly (3 × 10.0 == 30.0; 3 × (10/3) == 10.0, both REQUIRE'd), the jitter probe as a host case (≤ 1 DRIFTING
  in 100 draws per σ; the first build's rule fails ≥ 30 at σ = 50 — asserted, so the case is seen to have teeth),
  a real drift through the same jitter, the residual SD and SE against a hand-worked set (SE = 80 µs/s), a
  straight line's residual exactly 0. **Docs:** worksheet N.3 (the block), its table's column, N.4's HA-214
  bullet; the register's HA-214 *Settle*.
- **C-2 (minor) — NO CHANGE / NO TURN on one change and on every read PROS_ERR.** Re-checked: real (with one
  change `cadenceFrom` has no interval and the verdict was NoChange under "changes 1"; with every read PROS_ERR_F
  nothing is stored and NoChange's text pointed at PROS's busy-read copy). **Fix:** `CadenceVerdict::NoReads` and
  `GyroVerdict::NoReads` — "every read/poll was PROS_ERR -- no finding. Check the IMU's cable and port (STAGE 1),
  and re-run." — judged FIRST; `judgeCadence` now takes the `ChangeWatch` and reads NoChange only on ZERO changes,
  so one change is TOO FEW ("%lu change(s) only", with the 20-interval minimum printed); NO CHANGE's "that is the
  finding" line prints only when PROS_ERR is 0, else "the IMU dropped reads -- check its cable; this run is no
  finding about PROS's copy"; NO TURN prints the change count. **Tests:** 400 000 PROS_ERR reads through the real
  `runCadence` → NoReads; one change → TooFew; n == 1 → TooFew (the reviewer's X2); a dead gyro loop and an
  empty turn → NoReads; one change on good polls → NoTurn. N.4 restricted to `PROS_ERR 0`.
- **C-3 (minor) — test gaps (X1 z crossing zero, X4 judgePhase's argument, X5 t1's place, X3 judgePhase's
  order).** Re-checked: real. **Fix:** a trapezoid case on z = −50 + 40t across its zero on uneven stamps, exact
  to 1e-8 against the closed form, and a genuinely both-ways turn (100 sin(πt/2) + 10t: up ~110°, back to −70°;
  `mixedTurn` true) through the real `runGyro` → AGREE; X5 is now host-tested (the loops moved — T-3); X4
  pinned (`imu2c::reportPhase(ph, it::judgePhase(ph, cadVerdict));`, and exactly one call of each judge); X3 cut
  as MR11 (NoChange after TooFew).
- **C-4 (minor) — unlogged delta: "distinct values" became a change count.** Re-checked: true (`grep -i distinct`
  found nothing). **Logged:** delta 11 below; the completion record's deltas section names it.
- **C-5 (minor) — STAGE 2c's intro painted and erased in the same instant.** Re-checked: real (`rule()` with the
  scroll live redraws the header and clears; six `emit`s paint; `twoButtonPrompt` begins with `screen_erase()`).
  **Fix:** `g_screenActive = false;` BEFORE the rule, so the rule and the intro go to serial + SD only and the
  prompt is the panel's instruction (it already carries START, slowly, ONE way, and that the panel freezes).
  Pinned (`waitForTouch(); g_screenActive = false; rule("STAGE 2c`, and no `g_screenActive = true` between the
  rule and the prompt); mutation MS1.

### Tests
- **T-1 (major) — two of the 38 "caught" mutants broke the ARM build's warning policy.** Re-checked from the
  reviewer's `review-tests/armcheck_mp.txt` (`MP3 ARM rc=0 nonvendor_warnings=1`, `MP10 ... nonvendor_warnings=1`)
  and by reading: MP3 left `imuTimingStage` (anonymous namespace) unused → `-Wunused-function`, a FAILURE under
  the src gate since G1 Part 2c; MP10 left `prev` unused. The first build's drivers (`mutate_b.py`,
  `mutate_b2.py`) ran only the host build, which never compiles `bench_r3a.cpp`. **Fix:** this round's driver
  (`review-fix/mutate_r.py`) runs `make ROBOT=tank` on EVERY mutant first and refuses (re-cut, never counted) any
  whose ARM build fails, prints a warning outside `include/liblvgl/` / `include/pros/`, or does not recompile
  `bench_r3a.cpp`; MP3 and MP10 are re-cut ARM-clean (MP3r: the call kept but guarded dead; MP10r: `delay` with
  `prev` still read) and the first cuts are run through the same check to show the refusal (table below). The
  completion record's "38 distinct, every one RED on a fresh build" is corrected (no MP mutant had been
  compiled at all). The ARM-clean MP3r SURVIVED the first build's pin (it counted only the call's text) — the
  pin now requires the call to be the try block's unconditional last statement.
- **T-2 (major) — three thresholds could change with the suite green while the panel and the documents kept the
  old numbers.** Re-checked: real (tests reached `kFixedDriftUsPerS`, `kMinIntervals`, `kMinPhaseTicks`,
  `kPhaseWindowUs` only by name; the stage printed 10, 9500, 9000..11000, 2500 as literals). **Fix:** a constants
  case pins every threshold to the value the worksheet and the register quote, and the stage now PRINTS every
  threshold from its constant (`%lu` / `%.0f`), so the printed words cannot drift from the verdict. Mutations
  MR3–MR6 (the reviewer's A1–A3, R6b).
- **T-3 (major) — the stage's wiring and verdict-to-text mapping untested; the test re-implemented the loops.**
  Re-checked: real (`cadenceInstants` / `phaseTicks` / `gyroTurn` were hand copies of `collect*`). **Fix:** the
  four loops are now templates in `imu_timing.hpp` (`runCadence`, `runPhase`, `runSpans`, `runGyro`, over
  `nowUs`, `waitTick` and the reads; still PROS-free, allocation-free); the stage instantiates them with
  `imu2c::nowUs` (= `pros::micros()`), two `delay_until` lambdas and two read lambdas, each read NAMED ONCE
  (`rotation`, `gyroZ` = `imu_get_gyro_rate(port).z`); the host helpers now call the SAME templates on a
  `SimClock` — so Q4 (a backwards span), X5 and the clock-before-device rule are host-tested (`runSpans`: span =
  40 + 300 + i exactly; `runCadence`: the instant 3402, never 3409). `judgeGyro(const GyroTurn&)` is what the
  stage calls, so Q6 (swapped angles) cannot be written in the stage, and a host case proves the overload's order
  (20° with ∫z 200 → TOO SMALL; swapped → SCALE OFF). The remaining wiring is pinned exactly (each loop's
  arguments, each judge's input, each report's call; exact read counts; `gyro.z`; no heading/yaw read; the panel
  back on before the first report — Q9). The verdict-to-text-and-chip mapping is pinned PER CASE: each `case`'s
  `emitS` lines, in order, with their `Sev::` and no others (25 cases; Q7 and B5r), and each switch's case
  count. **Not done as suggested, and why:** the reviewer offered a table of texts in the header; `emitS` carries
  `__attribute__((format(printf, 2, 3)))`, and a table of format strings would trade the compiler's argument
  check for a host test, while four cases print two chips each — the per-case pin keeps both.
- **T-4 (minor) — judgeGyro's NaN guard and looksLikeRadians' `<=` edge unexercised.** Re-checked: real. In the
  reviewed header both scale tests are `<=` comparisons (false for NaN and ∞) and `looksLikeRadians` refuses a
  non-finite integral, so the explicit `isfinite` guard was UNREACHABLE by any input — removed, with a comment;
  `judgeGyro(2, 2000000, 100.0, NaN) == ScaleOff` and `CHECK_FALSE(looksLikeRadians(0.0, 0.0))` added; the
  radians edge is tested where it lands EXACTLY (a 45° turn: |z| = r·45 ± 0.10·r·45, both REQUIRE'd exact, IN, one
  ulp outward OUT — found by searching turns 30–400° for exact landings). Mutations MR7 (a NaN-passing `!(>)`),
  MR8 (`<`), MR9 (the zero-turn guard).
- **T-5 (minor) — phaseFrom's short-scratch bound untested.** Real. A case hands a 3-slot span inside an 8-slot
  buffer of sentinels: no sentinel touched, min / median / max from the three kept. MR10.
- **T-6 (minor) — the ban list missed C++ motor commands.** Real. Added `.move(`, `.brake(`, `move_absolute`,
  `move_relative`, `pros::Motor`, `ProsMotor`, `set_brake_mode`, `adi_`, and `imu_get_heading` / `imu_get_yaw`.
  (Bare `move(` / `brake(`, the brief's grep, would also ban `std::move(`; the stage has neither — checked.)

### Documents
- **D-1 (major) — the record said T12 and L1–L3 were "written FIRST and seen RED".** Real (this log :86-87,
  :484-504). Reworded to what the log records: R0 and T1–T11 written first (all red except R0, T3 (b) and T7,
  green by design); T12 and L1–L3 added at the review, after the fix.
- **D-2 (minor) — the history pointers to IMU1's record still pointed at the live log only.** Real. One dated
  clause each in `TANK1-COMPLETED.md` and `DOCS2-API-DEFECTS.md` A17 (history not rewritten), and the record
  now says it closes A17 and corrects TANK1's "immune".
- **D-3 (minor) — button 2's dot now includes 2c's verdicts, and the worksheet never said so.** Real
  (`setVerdict` keeps the worst). N.2 step 6 says it, and Station A's row 2 gains one clause in its "Finished
  when" column (a shared table: one clause, for the coordinator).
- **D-4 (minor) — HA-04 not judged when z reads as radians.** Real, and the reason in delta 6 was wrong for that
  case. **Fixed in the code, not only the docs:** `judgeGyro` returns `RadiansSameSign` / `RadiansOppositeSign`
  (HA-109 WRONG on its units; HA-04 judged — a sign does not depend on units); `ScaleOff` now means neither scale
  (another axis), the only case where HA-04 is "not judged -- z is not about the yaw axis.". Delta 6 corrected
  below; N.4's HA-04 and HA-109 bullets and the register's HA-04 / HA-109 *Settle* say so. MR19.
- **D-5 (minor) — status words disagreed across documents.** Re-checked: the roadmap's "Now" paragraph says
  LANDED because it is written as of the merge (Part A's did the same — "Now (2026-10-29): IMU1 Part A has
  LANDED", committed at its merge); build-order, REMAINING-WORK row 32 and the record say BUILT because that is
  true now. **Fix:** the record's header names exactly what the coordinator flips at the merge (its title and
  "What is NOT done", build-order's status line, row 32); build-order's status line gains the review's date. For
  the coordinator's merge record.
- **D-6 (minor) — REMAINING-WORK §10 left out Station N.** Real. A bullet beside Stations H and I. The totals
  table's IMU1 row left to the coordinator's re-pricing (as Part A's D-10).
- **D-7 (minor) — the record left out the deltas that contradict §7.** Real. A "Deltas from the brief" section
  (the new header and include against §7.2, the loops in it, changes not distinct values, the sign's rule,
  HA-214's uncertainty, `serialLine`, the hold, the dot, Station A's RUN ALL clause).

### Found by this round itself
- **The doc gates run inside every host build.** `shulib_tests` depends on `shulib_doc_gates`, which re-runs when a
  public document changes and checks the briefing against the binary ON DISK — before the link, so at a mutant
  run, the previous mutant's link. My first two starts of the driver stopped on it (`HOST-BUILD-BROKEN`: "stale
  briefing — RED 2 cases"), because I was editing `docs/hardware-assumptions.md` meanwhile. Those binary runs were
  bounded (`suite_run.py`'s RLIMIT_AS 4 GiB, tools/suite_run.py:17), and the driver now also builds under
  `ulimit -v 4194304`. **Fix:** both starts stopped by SIGTERM — the driver's handler restored the file each time
  (sha256 of all three source files checked against `review-fix/pre-mutation-hashes.txt`: OK) — then a clean
  binary built, the briefing regenerated around it, the gates re-run green, and the full run started with no gate
  input edited until it ended (internal documents only meanwhile). The first stop's `pkill -f` also matched my own
  shell; nothing else was affected.
- Station N.3's `grep -A12` would cut a 2c.4 block of 14 lines (a mixed turn under AGREE): now `-A14`.

### The deltas from the brief — one more, and delta 6 corrected *(2026-11-05)*
11. **Changes, not distinct values** — §1 item 5: "print the number of distinct values". The stage prints the
    number of CHANGES: for a one-way turn that is distinct values − 1, and the change instants are what the
    cadence is built from; a both-ways turn can revisit a value, which a distinct count would hide.
6. *(corrected)* **The sign is judged whenever z is about the yaw axis — in deg/s or in rad/s** — and not judged
   only when z is neither (another axis says nothing about `GyroRateZ`'s sign). The first build withheld it in
   the radians case too, with a reason that holds only for the wrong axis.

### The mutations — 76, each ARM-checked, on a fresh binary, the WHOLE suite, red observed, restored *(00:51–01:40; driver `review-fix/mutate_r.py`)*
Each mutant: one or more exact replacements (each matching once); the file backed up to
`review-fix/backup-<id>-*`, its timestamp bumped; **`make ROBOT=tank` first** — REFUSED (re-cut, never counted) on a
non-zero exit, any warning outside `include/liblvgl/` / `include/pros/`, or `bench_r3a.cpp` not recompiled; then the
host build under `ulimit -v 4194304` (its output names every recompiled object — every header mutant recompiled
`bench_imu_timing_test.cpp.o`, the header's only host dependent; a stage mutant needs none, the pins read the file at
run time); then `(ulimit -v 4194304; timeout 600 ./build/test/shulib_tests --order-by=file)`, the WHOLE suite; the file
restored in a `finally` (SIGTERM / SIGHUP raised as an exception — exercised twice this round, above), its sha256
checked. Outputs: `review-fix/mut-<id>.out`, `mut-results-final.json`, `mut-run-final.txt`. **76 of 76 CAUGHT; 0
ARM-refused; 0 build-broken; all restored** (the three source files equal `review-fix/pre-mutation-hashes.txt`
afterwards). The first set's 26 header mutants (MB) were RE-RUN on the reviewed header (re-cut where its text
changed: MB9, MB12, MB18, MB19), the stage set re-cut onto the new stage, and the reviewers' survivors cut as MR / MQ
/ MS (their ids in each row). The one failing case outside `IMU1-B` is SER1's own stdout pin, on MP5 — correctly.
The first build's MP3 and MP10 as cut, through the same check (`mut-results-orig.json`): **MP3o ARM-REFUSED**
(`bench_r3a.cpp:1302:6: warning: ... imuTimingStage(uint8_t) defined but not used [-Wunused-function]` — the
review's T-1, reproduced); MP10o on the new text is ARM-clean (the lambda still reads `phasePrev`) and CAUGHT — not
counted in the 76 (the same mutation as MP10r).

| # | Mutation | Red (every failing case, names cut) |
|---|---|---|
| MB1 | summarize: the UPPER median | summarize: min / LOWER median / max, the median always a m…; (2c.2): a copy AT the wake — a late wake MISSES it (counte… |
| MB2 | ChangeWatch: a PROS_ERR_F read not screened | ChangeWatch: a PROS_ERR_F read is counted and skipped — ne…; (2c.1): an IMU that answers PROS_ERR on EVERY read is NO R… |
| MB3 | ChangeWatch: a tolerance hides a small copy | ChangeWatch: the first good read is a baseline; a change i… |
| MB4 | cadence: the band's low end excluded | (2c.1): the band's ends are IN the band, one microsecond p… |
| MB5 | cadence: long intervals enter the in-band mean | (2c.1): a robot held still for part of the loop — the MEDI…; (2c.1): the band's ends are IN the band, one microsecond p… |
| MB6 | judgeCadence: median 9000 reads FASTER | (2c.1): HA-213's verdict at every edge — the reads, the co… |
| MB7 | unwrapNear: no unwrap at all | unwrapNear: the step lands in (-P/2, +P/2] — +P/2 kept, -P…; (2c.2): a phase that drifts THROUGH the wake is unwrapped… |
| MB8 | unwrapNear: +P/2 wrapped | unwrapNear: the step lands in (-P/2, +P/2] — +P/2 kept, -P… |
| MB9 | phase: drift from the ENDPOINTS | (2c.2): a copy 1 us slower than the wake grid drifts 100 u…; (2c.2): a phase that drifts THROUGH the wake is unwrapped…; (2c.2): a copy AT the wake — a late wake MISSES it (counte…; (2c.2): a FIXED phase seen through jittery wakes is never…; (2c.2): a REAL drift through the same jitter still reads D…; (2c.2): the residual SD and the slope's SE are the textboo… |
| MB10 | phase: missed ticks enter the median as 0 | (2c.2): a copy AT the wake — a late wake MISSES it (counte… |
| MB11 | judgePhase: judged without the 10 ms copy | (2c.2): HA-214's verdict at every edge — the cadence first… |
| MB12 | judgePhase: exactly 10 us/s not FIXED | (2c.2): HA-214's verdict at every edge — the cadence first… |
| MB13 | judgeSpan: on the MEDIAN | (2c.3): HA-215 is judged on the LONGEST span, at every edg… |
| MB14 | judgeSpan: 5000 us reads CLOSE | (2c.3): HA-215 is judged on the LONGEST span, at every edg… |
| MB15 | gyro: a left rectangle, not the trapezoid | (2c.4): the integral is the TRAPEZOID between the FIRST an…; (2c.4): a z that CROSSES ZERO between polls — the trapezoi… |
| MB16 | gyro: the integral from the loop's start | (2c.4): z in deg/s with get_rotation()'s sign, a 3 s turn…; (2c.4): z with the OPPOSITE sign -> HA-04 WRONG, HA-109's…; (2c.4): the integral is the TRAPEZOID between the FIRST an…; (2c.4): a dropped poll is counted and bridged — a bad valu…; (2c.4): a z that CROSSES ZERO between polls — the trapezoi…; (2c.4): the stage's judge takes the TURN, so its four numb… |
| MB17 | gyro: a poll from before the last good one kept | (2c.4): a dropped poll is counted and bridged — a bad valu… |
| MB18 | judgeGyro: exactly 10 percent not AGREE | (2c.4): HA-04/HA-109's verdict at every edge — changes, wi… |
| MB19 | judgeGyro: the sign judged before the scale | (2c.4): z in RADIANS per second -> HA-109 WRONG, the sign…; (2c.4): HA-04/HA-109's verdict at every edge — changes, wi…; (2c.4): a NaN integral is SCALE OFF; radians' edges land E… |
| MB20 | judgeGyro: a 2.000 s window TOO SHORT | (2c.4): HA-04/HA-109's verdict at every edge — changes, wi…; (2c.4): the stage's judge takes the TURN, so its four numb…; (2c.4): a NaN integral is SCALE OFF; radians' edges land E… |
| MB21 | gyro: the step into the first change counted | (2c.4): travel inside the window — rising minus falling IS… |
| MB22 | mixedTurn: exactly a fifth MIXED | (2c.4): travel inside the window — rising minus falling IS… |
| MB23 | gyro: a PROS_ERR_F z integrated | (2c.4): a dropped poll is counted and bridged — a bad valu… |
| MB24 | judgeCadence: 20 intervals TOO FEW | (2c.1): HA-213's verdict at every edge — the reads, the co… |
| MB25 | usBetween: an equal pair is 1 us | usBetween: a backwards or equal pair is 0, a huge gap is c…; (2c.2): the residual SD and the slope's SE are the textboo… |
| MB26 | judgeGyro: a NaN turn passes the 30 deg check | (2c.4): HA-04/HA-109's verdict at every edge — changes, wi… |
| MR1 | X1: no area where z changes sign | (2c.4): a z that CROSSES ZERO between polls — the trapezoi… |
| MR2 | X2: one change / one interval reads NO CHANGE | (2c.1): HA-213's verdict at every edge — the reads, the co… |
| MR3 | A1: kFixedDriftUsPerS 10 -> 25 | IMU1-B: every threshold is pinned to the value Station N a…; (2c.2): the jitter's edge — DRIFTING needs /drift/ > 3 SE,…; (2c.2): a FIXED phase seen through jittery wakes is never… |
| MR4 | A2: kMinIntervals 20 -> 3 | IMU1-B: every threshold is pinned to the value Station N a… |
| MR5 | A3: kMinPhaseTicks 20 -> 3 | IMU1-B: every threshold is pinned to the value Station N a… |
| MR6 | R6b: kPhaseWindowUs 9500 -> 9700 | IMU1-B: every threshold is pinned to the value Station N a… |
| MR7 | A4: a NaN integral passes the deg/s scale | (2c.4): a NaN integral is SCALE OFF; radians' edges land E… |
| MR8 | R3: radians' edge excluded | (2c.4): a NaN integral is SCALE OFF; radians' edges land E… |
| MR9 | R4: radians' zero-turn guard deleted | (2c.4): a NaN integral is SCALE OFF; radians' edges land E… |
| MR10 | R9: phaseFrom's scratch bound deleted | (2c.2): a scratch smaller than the seen ticks keeps the fi… |
| MR11 | X3: judgePhase's no-change after too-few | (2c.2): missed ticks are counted, never guessed, and never…; (2c.2): HA-214's verdict at every edge — the cadence first… |
| MR12 | DRIFTING without the 3 SE test (the first build's rule) | (2c.2): the jitter's edge — DRIFTING needs /drift/ > 3 SE,…; (2c.2): a FIXED phase seen through jittery wakes is never… |
| MR13 | FIXED without the noise test | (2c.2): the jitter's edge — DRIFTING needs /drift/ > 3 SE,…; (2c.2): a FIXED phase seen through jittery wakes is never… |
| MR14 | SE: divided by Sxx, not its root | (2c.2): a FIXED phase seen through jittery wakes is never…; (2c.2): the residual SD and the slope's SE are the textboo… |
| MR15 | SE: n, not n - 2, degrees of freedom | (2c.2): the residual SD and the slope's SE are the textboo… |
| MR16 | judgeCadence: no NO READS | (2c.1): HA-213's verdict at every edge — the reads, the co…; (2c.1): an IMU that answers PROS_ERR on EVERY read is NO R… |
| MR17 | judgeGyro(turn): no NO READS | (2c.4): the stage's judge takes the TURN, so its four numb… |
| MR18 | Q6: judgeGyro(turn) swaps the angles | (2c.4): z in RADIANS per second -> HA-109 WRONG, the sign…; (2c.4): the stage's judge takes the TURN, so its four numb… |
| MR19 | radians: the sign not judged (the first build) | (2c.4): z in RADIANS per second -> HA-109 WRONG, the sign…; (2c.4): a NaN integral is SCALE OFF; radians' edges land E… |
| MR20 | runCadence: the instant read AFTER the device | (2c.1): runCadence stamps each change with the instant rea… |
| MR21 | runCadence: overflow not counted | (2c.1): runCadence stamps each change with the instant rea… |
| MR22 | runPhase: the offset at the poll's END | (2c.2): a fixed 10 ms copy -> the offset IS the phase, dri…; (2c.2): a copy 1 us slower than the wake grid drifts 100 u…; (2c.2): a copy AT the wake — a late wake MISSES it (counte… |
| MR23 | runPhase: the window past 9.5 ms | (2c.2): a phase that drifts THROUGH the wake is unwrapped…; (2c.2): a copy AT the wake — a late wake MISSES it (counte… |
| MR24 | X5: t1 after the second IMU read | (2c.3): runSpans times FIRST IMU read to the instant of th… |
| MR25 | Q4: the span measured backwards | (2c.3): runSpans times FIRST IMU read to the instant of th… |
| MR26 | runSpans: the first read's value lost | (2c.3): runSpans times FIRST IMU read to the instant of th… |
| MR27 | runGyro: the rotation fed as z | (2c.4): z in deg/s with get_rotation()'s sign, a 3 s turn…; (2c.4): z with the OPPOSITE sign -> HA-04 WRONG, HA-109's…; (2c.4): z in RADIANS per second -> HA-109 WRONG, the sign…; (2c.4): a z that CROSSES ZERO between polls — the trapezoi… |
| MP1 | a motor command in the stand-in | pins: STAGE 2c reads only — no motor command, no device wr… |
| MP2 | ProsImu::yawRate() read (R18) | pins: STAGE 2c reads only — no motor command, no device wr… |
| MP3r | MP3 re-cut ARM-clean: the call kept but dead | pins: reportImu runs STAGE 2c after STAGE 2's verdict, hol… |
| MP4 | the stage run on the REFUSED path | pins: reportImu runs STAGE 2c after STAGE 2's verdict, hol… |
| MP5 | one line through stdout's printf | pins: STAGE 2c prints only through SER1's one-call path; pin: no src/ code prints through stdout's FILE — every lin… |
| MP6 | the pacer's header pulled in (R10) | pins: imu_timing.hpp is PROS-free and library-free (R10: n… |
| MP7 | imu_timing.hpp: a PROS include | pins: imu_timing.hpp is PROS-free and library-free (R10: n… |
| MP8 | no START / SKIP prompt | pins: reportImu runs STAGE 2c after STAGE 2's verdict, hol… |
| MP9 | the stand-in's line through emitf (panel + SD) | pins: STAGE 2c prints only through SER1's one-call path |
| MP10r | MP10 re-cut ARM-clean: 2c.2 paced by delay | pins: STAGE 2c reads only — no motor command, no device wr… |
| MP11 | RUN ALL: station 2 dropped | pins: reportImu runs STAGE 2c after STAGE 2's verdict, hol… |
| MP12 | STAGE 2's results erased at once | pins: reportImu runs STAGE 2c after STAGE 2's verdict, hol… |
| MQ1 | Q1/X4: judgePhase given Holds | pins: STAGE 2c runs the header's loops with the robot's cl… |
| MQ2 | Q2: gyro x, not z | pins: STAGE 2c reads only — no motor command, no device wr… |
| MQ3 | Q3: 2c.4 reads the heading | pins: STAGE 2c reads only — no motor command, no device wr…; pins: STAGE 2c runs the header's loops with the robot's cl… |
| MQ7 | Q7: WRONG FASTER with the PASS chip | pins: each verdict prints ITS OWN words with ITS OWN sever… |
| MQ9 | Q9: the results never reach the panel | pins: reportImu runs STAGE 2c after STAGE 2's verdict, hol… |
| MQ13 | Q13/B10: a C++ Motor's brake() in 2c.3 | pins: STAGE 2c reads only — no motor command, no device wr… |
| MB5r | B5r: AGREE and OPPOSITE SIGN's lines swapped | pins: each verdict prints ITS OWN words with ITS OWN sever… |
| MS1 | the intro painted on the panel, then erased | pins: reportImu runs STAGE 2c after STAGE 2's verdict, hol… |
| MS2 | NO CHANGE's 'the finding' printed with PROS_ERR reads | pins: each verdict prints ITS OWN words with ITS OWN sever… |
| MS3 | 2c.1 runs 3 s, not 2 | pins: STAGE 2c runs the header's loops with the robot's cl… |
| MS4 | an extra IMU read (heading) in the stage | pins: STAGE 2c reads only — no motor command, no device wr… |

### RESUMED — Thu Nov 5 2026, `date` 15:37 *(the fix run stopped partway on 2026-11-05 when the team lead dropped to one job at 92 % of the session limit and then paused all work)*
- **What the stopped run left, checked before anything else:** every one of the 18 findings already dispositioned
  above, and the 76 mutations logged; it stopped inside the final gates. `git status`: the ten modified files and
  three new ones above — plus **`src/programs/zz_self_test.cpp` (untracked, 02:08), a `src_build_gate.py
  self-test` plant** (its robot-two `static_assert` detector case; tools/src_build_gate.py :2844–2884 plant there)
  that the killed self-test never removed, and that the coordinator's audit did not list. **Removed.** Every
  gate-input file the run hashed before its gates (`review-fix/pre-gate-hashes.txt`, 1 022 files) is identical
  (`sha256sum -c`: no line other than OK); the three source files equal `review-fix/pre-mutation-hashes.txt`
  (no mutant left). `bench_r3a.cpp`'s 02:06 timestamp is a self-test restore's bump (contents identical).
- **The binary relinked from source before any result** (15:38): `rm -f build/test/shulib_tests`, the briefing
  regenerated (unbuilt), `cmake -S test -B build/test`, `cmake --build build/test -j8` under `ulimit -v 4194304` →
  exit 0, 246 objects compiled, no warning (`scratch/imu1-b/resume-build.txt`); `(ulimit -v 4194304; timeout 600
  python3 tools/suite_run.py green)` → `suite run: green — 2767 cases, 3000783 assertions`; `-tc="IMU1-B*"` →
  `test cases: 46 | 46 passed | 0 failed`, `assertions: 62597 | 62597 passed`; the briefing regenerated from that
  binary.
- **The already-logged dispositions, re-verified** (none redone; none reverted):
  - *The 76 mutations* stand for the final tree: the three source files have not changed since the run (their
    sha256 equal `review-fix/pre-mutation-hashes.txt`, written before it), and `mut-results-final.json`, read
    entry by entry, holds for every one of the 76: `make ROBOT=tank` exit 0, no warning outside the vendored trees,
    `bench_r3a.cpp` recompiled, every header mutant's host build recompiled `bench_imu_timing_test.cpp.o`, at
    least one failing case, the file restored — `CAUGHT` 76 of 76 (`mut-run-final.txt`: `DONE 76 caught of 76`).
    So no mutation needed a re-run.
  - *C-1's jitter probe*, recompiled against the final header and re-run (`review-fix/probe/probe_jitter_resume`,
    100 draws per point as the file on disk stands; `probe_jitter_400`, the same at 400): a fixed phase reads
    DRIFTING 0 of 100 at every σ (2–50 µs) and **0 / 1 / 0 of 400 at σ = 20 / 30 / 50 µs** — the log's "0–1 of 400",
    reproduced; FIXED 400 of 400 at σ = 2 and 5 µs; a real 100 µs/s drift DRIFTING 400 of 400 at every σ (100 of
    100 in the 100-draw run). A 30 µs/s drift reads DRIFTING 93 of 100 at σ = 30 µs and 39 at σ = 50 µs, the rest
    NOT SEPARABLE (never FIXED) — the design's intent: no verdict, rather than a wrong one.
  - *"Every other bench station unchanged", RE-PROVEN on the final stage* (the 2026-11-03 proof predates this
    round's rewrite of station 2's functions). `scratch/imu1-b/mine-tree` refreshed with the final
    `bench_r3a.cpp` (from `review-fix/backup-MS4-bench_r3a.cpp`, sha256 `df3bc1cd…` = the worktree's file — the
    worktree's own copy was, at that moment, carrying the running gate self-test's plant, `this line is not C++`;
    caught by its hash before use) and `imu_timing.hpp`; `make ROBOT=tank` → exit 0, only `bench_r3a.cpp`
    recompiled, its one warning the vendored `liblvgl/core/lv_obj_style.h:94`'s, `bin/hot.package.bin` 71 208
    bytes (`review-fix/resume-mine-make-tank.txt`). `objcmp2.py` against the base's object: `functions at base:
    117 | identical: 116 | differ: 1 | new: 11 | gone: 0` — the one that differs is `tImu` (the inlining change
    logged at 15:58 on 2026-11-03); the 11 new are the same set (`reportImu`, `imu2c::panel`, `imu2c::foot`,
    `ChangeWatch::feed`, `GyroTurn::windowUs`, `summarize`, `phaseFrom`, four `std::sort` helpers) — the new
    `run*` templates and `judge*` are inlined into `reportImu`. Every other function — every other station's,
    the menu's, the grid's, STAGE 5's, `runR3a()` — is instruction-identical to the base's.
- **The prose review (rule 4) for this round's words — the greps RUN FIRST (15:45), every hit read:**
  `grep -rIn --exclude-dir=api -F "<term>" docs README.md`, this log and the brief left out, for the names this
  round removed (`collectCadence`, `collectPhase`, `collectSpans`, `collectGyro` → no hit anywhere) and the
  words it added (`NO READS`, `NoReads`, `not separable`, `NotSeparable`, `residual SD`, `3 SE`,
  `kDriftSigmas`, `Radians`, `SCALE OFF`, `TOO FEW`, `imu_timing`, `distinct value`, `10 us/s`, `10 µs/s`). The hits
  in the worksheet's Station N (N.3's sample block, its table header, N.4's HA-213/HA-214/HA-04/HA-109 bullets),
  the register's HA-214 *Settle*, build-order's status line and `IMU1-COMPLETED.md` were each read against the
  stage's own format strings (`bench_r3a.cpp`'s `emitS` / `emitf` lines in `imu2c`): N.3's sample lines match
  the strings word for word (`spread of those offsets`, `drift ... +- ... us/s (3 SE; ...)`, `residual SD`,
  `the wake (|drift| and 3 SE <= 10 us/s over 1 s).`, `max 731 < 2500 us`); N.4's `DRIFTS`, `no verdict --
  drift ... +- ...`, `every read was PROS_ERR -- no finding.`, `change(s) only` and `HA-04: not judged -- z is
  not about the yaw axis.` are the stage's words. The other hits (`TOO FEW MOTORS` in CAL1/R3d, `Radians` in
  G1/R3d, H1's "distinct value", VIS1's SER1 heading) are other chunks' words — unrelated. Nothing stale found.
- **The gates — every one re-run on the final tree** *(15:41–16:25)*:
  - `python3 tools/src_build_gate.py self-test` (15:41–16:09) → `src build gate self-test: OK (269 detector
    cases, each against a real build)`. Before it the tree's 1 022 gate-input files were hashed
    (`review-fix/resume-pre-gate-hashes.txt`); after it `sha256sum -c` → every one OK, no `zz_self_test*` file
    anywhere, `git status` outside `docs/` exactly `bench_r3a.cpp` modified and the two new files — no plant left.
  - `python3 tools/src_build_gate.py check` → `src build gate: PASS (all 7 builds link; ...)` — bench, xdrive,
    tank, tank-drive, tank-calibrate, tank-autontest, tank-competition: each `exit 0, 11/11 TUs compiled,
    warnings: 8 vendor-ignored, 0 disallowed` (`review-fix/resume-gate-check.txt`).
  - The ARM header gate (CI's command; `review-fix/all_headers_resume.cpp`, `TU includes 228 headers`,
    `-Werror`) → `ARM compile gate: CLEAN`. The CI greps → `library is PROS-free`; `layering holds: core is
    sim-free`.
  - The self-test's restores bump timestamps, so the binary was relinked again: `rm -f build/test/shulib_tests`,
    the briefing regenerated (unbuilt), `cmake --build build/test -j8` under `ulimit -v 4194304` → exit 0, 246
    objects, no warning; `(ulimit -v 4194304; timeout 600 python3 tools/suite_run.py green)` → `suite run: green
    — 2767 cases, 3000783 assertions`; the briefing regenerated from that binary.
  - The doc gates, after this log's last edit: below.
- **Where IMU1 Part B stands** *(for the coordinator — nothing committed)*: every one of the 18 review findings
  dispositioned (above) and re-verified at this resumption; built, host- and build-gated, never run on a robot.
  For the coordinator, unchanged from above: the include line and Station N's place beside MECH-A6's (a
  mechanical rebase for whichever lands second); Station A's row 2 clause; button 2's dot now carries 2c's
  verdicts (said in N.2 step 6); at the merge, the status words D-5 names (the record's title and "What is NOT
  done", build-order's IMU1 status line, REMAINING-WORK row 32) move to LANDED, and the totals table's IMU1 row is
  re-priced. HA-213/214/215, HA-04 and HA-109 stay unsettled until the team's Station N run. No `pros` command; no
  `/dev/ttyACM*`; no mutant or plant left.
- **The doc gates, after the last edit above** *(16:17)*: `api_doc_tool.py` self-test `OK`; check-coverage `5060
  public entities across 188 headers, all documented`; check-fresh exit 0 (silent; no `///` changed); check-examples
  `600 quoted lines, all verbatim`; check-removability `no public doc references docs/internal/`;
  `doc_staleness_audit.py` self-test `OK` and the audit `clean (31 live docs, 0 numeric claims checked)`;
  `briefing_status.py` self-test `OK (38 cases ...)` and check `generated block current`. Re-run once more after
  appending this line: the same.

## Verification-round findings and their disposition *(Part B; from `date` 16:45, 2026-11-05; appended at `date` 17:56)*

A fresh verifier returned 2 findings (1 major, 1 minor). Each was re-checked against the tree before any fix. Both
are real, and both are fixed at the root. **Base unchanged:** `eab0f89`. No `pros` command, no `/dev/ttyACM*`,
nothing committed. **`bench_r3a.cpp` is NOT changed this round** (its sha256 equals
`scratch/imu1-b/verif-fix/pre-fix-hashes.txt`), so no function of it changed, MECH-A6's included. The
"every other bench station unchanged" proof above (116 of 117 identical, `tImu` the tail call) still holds as it
stands. The changed files are `test/bench_imu_timing_test.cpp` (one new case, one case extended),
`src/programs/imu_timing.hpp` (one comment), `IMU1-COMPLETED.md`, this log and the generated briefing. Pre-fix
copies are in `scratch/imu1-b/verif-fix/pre-*`.

- **V-1 (major): the data lines C-1 added, and the SKIP path, were unpinned.** Re-checked: real. The per-case
  pin (`each verdict prints ITS OWN words`) reads only each `case`'s `emitS` lines. Every `emitf` data line of
  the four result blocks, their branch heads and their derived locals (`noise`, `d`, `z`, `ratio`) could change
  with the suite green. The SKIP branch's `g_screenActive = true;` was unguarded as well. **Shown before the
  fix (16:49–17:00):** the verifier's V12–V14 and 13 more of the same class (W1–W13, below) were run against the pre-fix suite
  (`verif-fix/mutate_w.py`, `SKIP_ARM=1`, `mut-run-before.txt`). **16 of 16 SURVIVED**, each `2767 | 2767
  passed`. So the gap was wider than the finding: every number a result block prints was unguarded, not only the
  ±.
  **Fix (not narrowed to the four lines suggested):** a new source pin, `IMU1-B pins: every number a result block
  prints is the field its words name, under the branch that prints it`. For each of `reportCadence`,
  `reportPhase`, `reportSpan` and `reportGyro`, it names, in source order, every `emitf` / `emitS` statement with
  its WHOLE argument list, every `if` / `else` head and every `const double` local. In those four functions that
  includes the suggested `const double noise = it::kDriftSigmas * ph.driftSeUsPerS;`, the drift line `...,
  ph.driftUsPerS, noise, it::kDriftSigmas, ul(it::kCopyPeriodUs));`, `if (ph.driftValid && ph.seValid) {`, the
  residual line and the spread line. Each item must be found after the one before it. The pin then requires the
  block's counts of `emitf(` + `emitS(`, `if (`, `else`, `const double ` and `?` to equal the list's own counts,
  so no unlisted print, branch or derived value can join. It also refuses any early exit, matched as tokens
  (`return;`, `return `, `for (`, `while (`, `do {`, `goto `, `throw `), because the printed words contain
  "returned" and "while a loop". The list was generated from the source by `verif-fix/gen_pins.py` (squashed
  as the test squashes). Then **every item was read by hand against its words before it went in**: each value
  is the field its label names. Examples: `min` / `max` are `ph.minUs` / `ph.maxUs`; the `+-` is `noise` =
  `kDriftSigmas` × SE, printed beside `(%.0f SE` fed `kDriftSigmas`; `z / rotation` is `z / d`; and HA-109's
  ratio is `|z / d|`. No print was found wrong; the source is unchanged. **The SKIP:** the stage-flow pin now
  requires `if (!start) { g_screenActive = true; drawHeader("STAGE 2c IMU TIMING"); screenClear(); emit("STAGE
  2c SKIPPED by the operator -- nothing measured."); return; }` exactly once. It also requires that `return;` is
  the stage's only one, after the prompt and before `it::runCadence(`.
  **Mutations — 16, counted on the fixed tree (17:02–17:16)** (`verif-fix/mutate_w.py`, a copy of `review-fix/mutate_r.py`).
  Each mutant was first built with `make ROBOT=tank`, which had to exit 0 with no warning outside the vendored
  trees and with `bench_r3a.cpp` recompiled. Then came the host test target, then the WHOLE suite under
  `(ulimit -v 4194304; timeout 600)`. The file was restored in a `finally` that also covers SIGTERM and SIGHUP,
  and its hash was checked. The results are in `mut-results-final.json` and `mut-run-final.txt`: **16 of 16
  CAUGHT; 0 ARM-refused; all restored**. The three source files equal `verif-fix/pre-mutation-hashes.txt`
  afterwards. Each mutant was a stage mutant, which needs no host recompile: the pins read the file at run
  time.

  | # | Mutation | Before (pre-fix suite) | After — red case |
  |---|---|---|---|
  | V12 | SKIP leaves the scroll off (`g_screenActive = true;` deleted) | SURVIVED | reportImu runs STAGE 2c … (the stage-flow pin) |
  | V13 | the drift line's `+-` is 1 SE (`ph.driftSeUsPerS` for `noise`) | SURVIVED | every number a result block prints … |
  | V14 | SE and residual guarded dead (`&& ph.seen > 1000`) | SURVIVED | every number … |
  | W1 | `noise` itself 1 SE (the drift line AND NOT SEPARABLE) | SURVIVED | every number … |
  | W2 | NOT SEPARABLE's `+-` is 1 SE | SURVIVED | every number … |
  | W3 | the residual SD line prints the SE | SURVIVED | every number … |
  | W4 | the offsets' spread: min and max swapped | SURVIVED | every number … |
  | W5 | 2c.1's intervals: median and max swapped | SURVIVED | every number … |
  | W6 | 2c.1's overflow line guarded dead | SURVIVED | every number … |
  | W7 | 2c.3's span line: the min printed as the median | SURVIVED | every number … |
  | W8 | 2c.4's `z / rotation` printed inverted | SURVIVED | every number … |
  | W9 | HA-109 WRONG's ratio inverted | SURVIVED | every number … |
  | W10 | an early `return` in `reportPhase` | SURVIVED | every number … |
  | W11 | the both-ways note's condition inverted | SURVIVED | every number … |
  | W12 | 2c.4: polls and dropped swapped | SURVIVED | every number … |
  | W13 | SKIP falls through into the reads (`return;` deleted) | SURVIVED | reportImu runs STAGE 2c … (the stage-flow pin) |

  **The 76 earlier mutations still stand** (`mut-results-final.json` in `review-fix/`). This round only ADDED
  assertions to the test file, and its header change is a comment, so a mutant red before is red now.
- **V-2 (minor): `kRadPerDeg`'s comment still said the radians reading is "a hint ..., never a verdict of its
  own".** Re-checked: real. It contradicts `judgeGyro`'s `RadiansSameSign` / `RadiansOppositeSign` (D-4's fix)
  and the register's HA-109 *Settle*. **Fix:** the comment now says that an integrated z within
  `kGyroTolerance` of this share of the rotation's change reads as rad/s about the yaw axis. That is
  `judgeGyro`'s `Radians*` verdicts: HA-109 WRONG on its units, with the sign (HA-04) still judged.
  `imu_timing.hpp` is under `src/programs/`, not a public header, so `docs/api` is not affected
  (check-fresh below).
- **The completion record** says it (`IMU1-COMPLETED.md`: the title's date clause, "What the reviews changed",
  47 cases, seven pins, 2,768, the 16 mutations).

### The prose review (rule 4) for this round's words *(greps RUN FIRST, 17:19, every hit read)*
`grep -rIn --exclude-dir=api -F "<term>" docs README.md src/programs/imu_timing.hpp` was run, with this log and
the brief left out. The terms were: `verdict of its own`, `never a verdict`, `a hint printed`, `46 cases`,
`2,767`, `2767`, `six source pins`, `six of them source`, `kRadPerDeg`, `Radians*`, `(3 SE`, `+- `, `SKIPPED by
the operator`. Results:
- **The stale `kRadPerDeg` comment:** gone, and no other copy of it exists. `docs/hardware-assumptions.md:918`'s
  "is its own verdict" now agrees with the code.
- **`never a verdict`:** `imu_timing.hpp:76` is TOO FEW's (correct). The others belong to other chunks (VIS1,
  L6, DOCS1, MECH, the F1 handoff, the briefing's method note), as does roadmap :692 / E2's "no verdict of its
  own". All are unrelated.
- **The counts:** only `IMU1-COMPLETED.md`, now 47 / seven / 2,768. Its `2,767` hits are the BEFORE run's
  figure, which is right.
- **The `+-`:** worksheet N.3's sample (`drift +2.9 +- 5.6 us/s (3 SE; least squares, unwrapped at 10000 us)`),
  its table header ("drift +- 3 SE") and N.4's `no verdict -- drift ... +- ...` match the strings now pinned.
- **The SKIP:** N.2 step 2's SKIP line matches the pinned `emit`.

Nothing stale remains.

### The gates — on the final tree *(17:17–17:56)*
- **Host build:** `rm -f build/test/shulib_tests`, the briefing regenerated (unbuilt), `cmake -S test -B
  build/test`, then `cmake --build build/test -j8` under `ulimit -v 4194304`. Exit 0, no warning. Then `(ulimit
  -v 4194304; timeout 600 python3 tools/suite_run.py green)` gave `suite run: green — 2768 cases, 3000912
  assertions`, and `-tc="IMU1-B*"` gave 47 of 47. The briefing was regenerated from that binary.
- **`src_build_gate.py self-test`:** `OK (269 detector cases, each against a real build)`.
- **`check`:** `PASS (all 7 builds link ...)`. bench, xdrive, tank, tank-drive, tank-calibrate, tank-autontest
  and tank-competition each gave `exit 0, 11/11 TUs compiled, warnings: 8 vendor-ignored, 0 disallowed`.
- **No plant left:** 1 471 tracked and untracked files were hashed before the self-test
  (`verif-fix/pre-gate-hashes.txt`). `sha256sum -c` afterwards: every one OK, no `zz_self_test*` file, and
  `git status` unchanged.
- **The ARM header gate:** CI's command, `TU includes 228 headers`, `-Werror`, gave `ARM compile gate: CLEAN`.
- **The CI greps:** `library is PROS-free` and `layering holds: core is sim-free`.
- **The self-test's restores bump timestamps,** so the binary is relinked again after this log's edit, and the
  doc gates are run last (below).
- **Relinked after the self-test** *(17:57)*: `rm -f build/test/shulib_tests`, the briefing regenerated
  (unbuilt), `cmake --build build/test -j8` under `ulimit -v 4194304` → exit 0, 246 objects, no warning;
  `(ulimit -v 4194304; timeout 600 python3 tools/suite_run.py green)` → `suite run: green — 2768 cases, 3000912
  assertions`; the briefing regenerated from that binary.
- **The doc gates, after the last edit above:** `api_doc_tool.py` self-test `OK`; check-coverage `5060 public
  entities across 188 headers, all documented`; check-fresh exit 0 (silent: no public `///` changed);
  check-examples `600 quoted lines, all verbatim`; check-removability `no public doc references docs/internal/`;
  `doc_staleness_audit.py` self-test `OK` and the audit `clean (31 live docs, 0 numeric claims checked)`;
  `briefing_status.py` self-test `OK (38 cases ...)` and check `generated block current`. Re-run once more after
  appending these lines: the same.
- **Where IMU1 Part B stands** *(for the coordinator; nothing committed)*: both verification findings fixed and
  logged; the stage's code unchanged this round; built, host- and build-gated, never run on a robot. Everything
  the earlier "Where IMU1 Part B stands" hands the coordinator still holds (the rebase beside MECH-A6, Station A's
  row 2 clause, the status words at the merge, the totals table's re-pricing).

## Part B — the coordinator's merge verification *(2026-11-06 merge, committed 2026-11-10)*

- **Squash** of `imu1-b` onto `5120dad` (MECH-A6, the MECH core complete). Conflicts resolved by hand: the roadmap's
  "Now" paragraphs (both kept, IMU1 Part B's dated 2026-11-06), the bench worksheet (Station P — MECH-A6 — then
  Station N — this part — both after Station I, each its own section), and the generated briefing (regenerated).
  `bench_r3a.cpp` merged cleanly beside MECH-A6's includes. Status words moved from "built" to LANDED: build-order's
  IMU1 line, REMAINING-WORK row 32, IMU1-COMPLETED.md's title and its "What is NOT done".
- **Gates on the merged tree** (`.claude/coordinator/scratch/imu1_b_merge_verify.sh`, every binary capped): host
  build clean; the suite green, 3062 cases, 3,024,635 assertions; `src_build_gate.py` self-test OK and check PASS
  (all 7 builds link); the ARM header gate CLEAN (240 headers); the api doc gates, the staleness audit and the
  briefing check clean; the compile-refusal harness 255 of 255; no plant, no stray root file.
- **The coordinator's own mutations** (`.claude/coordinator/muts_imu1_b_main.json`, `mutate.py`, every dependent
  object relinked, each file restored and hash-checked): **9 of 9 RED** — MI1 the median's lower middle, MI2 the band's
  inclusive high edge, MI3 `unwrapNear`'s half-period tie, MI4/MI5 FIXED and DRIFTING each without the standard error,
  MI6 the first step counted outside the gyro's window, MI7 the 30° edge, MI8b the mixed-turn fifth, MI9 the 5 ms span
  edge. (MI8, `sameSign` with `>= 0`, was dropped before running as equivalent under the 30° floor.) The restored
  tree relinked and ran green.
- **IMU1 is COMPLETE.** Nothing of it has run on a robot; worksheet Station N is its first look.
