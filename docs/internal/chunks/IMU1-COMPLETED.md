# IMU1 — COMPLETED (Part A landed 2026-10-30, `6798aa5`; Part B built 2026-11-03, its review's and its verification round's findings fixed 2026-11-05, LANDED 2026-11-06; host- and build-gated, NOT run on a robot)

> The IMU's turn rate is read ONCE per control tick, and every reader in that tick gets the same value; then a
> Bench Tests stage that measures, on the robot, the beliefs that fix rests on. The team lead's "Accept all
> (Recommended)" of 2026-10-22 (D1 (a) the 5 ms hold in `ProsImu` with its three named limits; D2 (a) the frozen
> `IImu` untouched; D3 (a) the bench stage, Part B; D4 (a) IMU1 before Station M's floor run). Written FROM
> [`IMU1-PROGRESS.md`](IMU1-PROGRESS.md) (Part A: the build, its review round, its verification round, the
> coordinator's merge verification; Part B: the build, a run stopped by the team lead's pause of 2026-10-30 and
> resumed 2026-11-03). Brief: [`IMU1-imu-rate-once-per-tick.md`](IMU1-imu-rate-once-per-tick.md) (its answers block
> and "Re-check — amendments" R1–R18 override its body). Executor model: `opus`, in the isolated worktrees
> `.claude/worktrees/imu1-a` and `imu1-b`. *(Written by Part B's builder before the coordinator's verification of
> Part B; the coordinator's merge record for Part B goes at the end of the live log. At that merge the
> coordinator moves this record's title and its "What is NOT done", build-order's IMU1 status line and
> REMAINING-WORK's row 32 from "built" to LANDED, so they match the roadmap's "Now" paragraph, which is
> written as of the merge.)*

## The one-sentence honest status

`ProsImu::yawRate()` keeps its answer for 5 ms (`kRateHoldUs`), so the Localizer, a record and a corrector's gate
read one value per tick — proven on the host test plant with a model of PROS's documented 10 ms copy, where the
adapter before IMU1 turned the 40 in auton into a timeout in one band of the copy's phase — and "Bench Tests"
button 2 now ends with STAGE 2c, which measures that copy's cadence and phase, a lower bound on a tick's span and
the gyro's sign and units on the robot; nothing of either part has run on a robot, and HA-213, HA-214, HA-215,
HA-04 and HA-109 stay unsettled until the team runs worksheet Station N.

## Built

- **Part A — `6798aa5` (built 2026-10-29, landed 2026-10-30):** `include/shulib/hal/pros/imu.hpp` — the hold
  (`kRateHoldUs` = 5000 µs, half `ProsTickPacer::kTickMs`, compared in whole microseconds; the clock read before the
  device; a held read touches nothing; a faulted read opens the hold but never moves the differentiation sample),
  over both rate sources; its three limits in the code and the changelog ("Fixed since 2.5"); the PROS shim's
  rotation hook; `test/imu_buffer_model.hpp`; `test/imu_rate_once_per_tick_test.cpp` (R0 and T1–T11 written FIRST against
  the unchanged adapter — all red except R0, T3 (b) and T7, green there by design; T12 and L1–L3 added at
  the review, after the fix) and `test/imu_rate_hold_pin_test.cpp` (the hold pinned to the
  tick); HA-213, HA-214, HA-215 registered; HA-22, HA-32, HA-102 and HA-179 amended. No API version change.
  It closes DOCS2's defect **A17** (deferred to R4 as "not biased, only noisier" — under PROS's documented
  10 ms copy it was biased) and corrects TANK1-COMPLETED's "TANK1 is immune" (false for robot two: the
  record's second read moved the adapter's baseline); both records carry a dated pointer here, history not
  rewritten.
- **Part B — built 2026-11-03:** Bench Tests button 2's **STAGE 2c** (`src/programs/bench_r3a.cpp`,
  `imuTimingStage()` and its `imu2c` helpers — station 2's functions only), run after STAGE 2's HA-02 verdict once
  the operator has read it (a touch), behind a START / SKIP prompt (so 7 RUN ALL reaches it too): **2c.1** the raw rotation's change instants over a 2 s
  tight loop (HA-213); **2c.2** the copy's offset from a `delay_until(10 ms)` wake over 100 ticks, and its
  least-squares drift (HA-214); **2c.3** a stand-in for a motion tick's span — the drive motors' reads, the battery
  and one serial line between two IMU reads, 100 paced ticks, a LOWER bound (HA-215); **2c.4** the gyro's z
  integrated (trapezoid on `micros()` stamps) beside the raw rotation's change between the first and last change of
  a 3 s turn, both raw PROS degrees, never `ProsImu::yawRate()` (HA-04, HA-109). It commands no motor, writes no
  device and adds no pacer or Localizer include. Its arithmetic — every verdict — AND its four collecting loops
  (templates over the clock, the wait and the reads, since the review) are PROS-free in
  `src/programs/imu_timing.hpp`, tried by `test/bench_imu_timing_test.cpp` (47 cases; seven of them source pins
  over the stage's own code, which the host cannot compile). Worksheet **Station N** (new), Station 4's pointer and Station A's row; HA-04's and
  HA-109's *Settle* (and HA-213..215's, made exact) name the stage.

## What the reviews changed

- **Part A:** the review's 16 findings, merged where they overlapped — one blocker (C-1/D-1: Station M's "IMU1
  build" check passed on trees without the fix; the worksheet now greps the tree for `kRateHoldUs`), one major
  (C-2/D-2: the changelog left out limit (1)), the base having moved (C-3/D-3: IMU1 rebased onto CAL1 Part A2a's
  `e5a69b5`) and nine minors (tests for the clock-before-device rule and for the figures the documents quote among
  them) — then the verification round's four (V-1, blocker: a mutant left live in the worktree — restored, and the
  mutation drivers now restore on SIGTERM and SIGHUP; V-2, major; V-3 and V-4, minor). All fixed at the root; the
  live log has each finding and its disposition, and the coordinator's six hand mutations were all red.
- **Part B (the review round, 2026-11-05):** three reviewers (correctness, tests, docs), 18 findings — 4 major,
  14 minor — each re-checked and fixed at the root (the live log, "IMU1-B review findings and their
  disposition"). The ones that changed what the stage says: HA-214's verdict assumed an unmeasured wake jitter —
  it now prints the offsets' spread, the residual SD and the drift's 3-standard-error band, and a drift the
  jitter could fake is "no verdict", never DRIFTING (a fixed phase under 50 µs of jitter had read DRIFTING in a
  third of runs); an IMU answering PROS_ERR on every read is "no finding", never "no change" (which pointed at
  PROS's copy); one change is "too few", not "no change"; HA-04's sign is judged when z is in rad/s too; the
  intro lines no longer flash on the panel. The ones that changed how it is proven: the four loops moved into
  the header so the host drives the stage's real loops, not copies; every threshold pinned to the number the
  documents quote, the stage printing them from the constants; the wiring and every verdict's words and
  severity chip pinned per case; two of the first build's 38 "caught" mutants (MP3, MP10) had broken the
  ARM build's warning policy and were re-cut ARM-clean — every mutant of this round is ARM-compiled first.
- **Part B (the verification round, 2026-11-05):** a fresh verifier's two findings, both fixed (the live log,
  "Verification-round findings and their disposition"). Major: the numbers the review round had added to the
  panel — the ± under "(3 SE; ...)", the residual SD — and every other data line of the four result blocks were
  unpinned (the per-case pin covered only the verdict lines), and a SKIP that left the panel's scroll off
  survived too; a new source pin now holds every print of the four blocks with its whole argument list, every
  branch head and derived local, in order, and refuses any unlisted print, branch or early exit, and the SKIP
  branch is pinned exactly. Minor: `kRadPerDeg`'s comment still called the radians reading "never a verdict of
  its own" — reworded to the `Radians*` verdicts. No change to what the stage does.

## The evidence for Part B (from the live log, "Part B")

- **Tests:** `test/bench_imu_timing_test.cpp`, 47 cases (prefix `IMU1-B`; 32 at the build, 14 added by the
  review, 1 by the verification round): the constants pinned to the library's hold AND to the numbers the worksheet and the register quote;
  the interval summary and `usBetween` at their edges; `ChangeWatch` (an exact inequality, a PROS_ERR_F read
  counted and skipped); the stage's REAL loops (`runCadence`, `runPhase`, `runSpans`, `runGyro`) driven on a
  simulated clock over a sample-and-hold model of HA-213 — 2c.1 (10 ms HOLDS, 5 ms FASTER, 20 ms SLOWER, a still
  robot NO CHANGE, a dead IMU NO READS, one change TOO FEW, a robot still for 40 % of the loop still HOLDS, the
  change instant read before the device, the overflow counted); 2c.2 (the unwrap at ±P/2 exactly; least squares
  against endpoints; a copy drifting through the wake; missed ticks never in the median; a fixed phase under
  20–50 µs of wake jitter never DRIFTING — at most 1 in 100 draws, where the first build's rule failed a third —
  and a real 100 µs/s drift under the same jitter DRIFTING every draw; the residual SD and standard error
  against a hand-worked set; a short scratch never written past); 2c.3 (the span from the first IMU read to the
  instant of the second, exactly; judged on the LONGEST span); 2c.4 (the trapezoid exact for a ramp and for a z
  that crosses zero; a both-ways turn judged; dropped polls bridged; rad/s judged with its sign, another axis
  SCALE OFF with the sign NOT judged; the judge takes the turn, so its numbers cannot be swapped); every verdict
  at its edge, each edge chosen so the double arithmetic lands on it exactly (REQUIRE'd where it is not obvious);
  and seven source pins over the stage's own code (read-only, every read named once, never `yawRate()`, no pacer or
  Localizer include, SER1's one-call path, the loops' and judges' wiring, every verdict's words and severity chip,
  the call in `reportImu` unconditional after the HA-02 verdict, STAGE 2's results held until a touch, the intro
  off the panel, 7 RUN ALL still running station 2, every result block's printed numbers and branches, the SKIP
  path). The suite: **2,768 cases, green** (main's 2,721 + 47).
- **Mutations — the review round's set, 76 distinct, every one RED** (the live log, "IMU1-B review findings and
  their disposition"): each ARM-compiled first with `make ROBOT=tank` and refused if it failed or warned outside the
  vendored trees (none was), then the host build (every header mutant recompiled `bench_imu_timing_test.cpp.o`, the
  header's only host dependent), then the whole suite under a 4 GiB / 600 s cap; restored and hash-checked. 26 on the
  first build's header set (MB, re-run on the reviewed header), 27 on this round's header code and the reviewers'
  survivors (MR), 23 on the stage's code (MP / MQ / MS / MB5r). **Corrected:** the first build's record said "38
  distinct, every one RED on a fresh build"; its 12 stage mutants had never been compiled (the host build does not
  compile `bench_r3a.cpp`), and two of them — MP3 and MP10 — broke the ARM build's warning policy, so at most 36 of
  the 38 were valid; both are re-cut ARM-clean (MP3r, MP10r) in the 76, and MP3 as first cut is shown refused.
- **Mutations — the verification round's 16, every one seen SURVIVING first, then RED:** the verifier's V12–V14
  and 13 more of the same class (W1–W13: the ± band, the residual, the spread, the cadence and span summaries,
  z / rotation and the ratio swapped or inverted, a guard made dead, an early return, the both-ways note
  inverted, the SKIP falling through). Each run first against the pre-fix suite (16 of 16 survived, 2,767 of
  2,767 cases passing) and then, ARM-compiled clean, against the fixed suite (16 of 16 caught); restored and
  hash-checked.
- **"Every other bench station unchanged":** the ARM object `bench_r3a.cpp.o` (`make ROBOT=tank`) against the
  base `eab0f89`'s, function by function with addresses masked — 116 of the base's 117 functions
  instruction-identical; the one that differs is `tImu`, only because `reportImu` (now larger) is no longer inlined
  into it (a 6-instruction tail call to it). Re-run on the review round's final stage (2026-11-05): the same — 116
  of 117 identical, `tImu` the tail call, 11 new functions.
- **Gates:** `src_build_gate.py` self-test OK (269 detector cases) and check **PASS, all 7 builds**, 0 disallowed
  warnings; the ARM header gate over 228 headers clean; the CI greps; api_doc_tool's self-test, coverage,
  check-fresh, check-examples and removability; `doc_staleness_audit.py`; `briefing_status.py check` — each as
  printed in the live log.
- **Found and fixed by the resumed run's own review:** the stage erased STAGE 2's results from the panel the moment
  its HA-02 verdict printed (a change to station 2's run behaviour); it now waits for a touch first
  (`STAGE 2 done -- read it, then TOUCH for 2c`), pinned and mutation-checked (MP12).

## Deltas from the brief (Part B; each with its reason in the live log, "The deltas from the brief")

- **A new PROS-free header, `src/programs/imu_timing.hpp`, and ONE new `#include` in `bench_r3a.cpp`** — against
  §7 Part B item 2 ("the diff adds no `#include`"). The host suite cannot compile `bench_r3a.cpp`, and the task
  asked for host tests of the stage's pure parts; the header includes only the standard library, so R10's
  concern (no motion scheduler, no Localizer in Bench Tests) holds, and is pinned.
- **The stage's four loops are in that header too** (since the review): templates the stage instantiates with
  `pros::micros()`, `delay_until` and the C API's reads, so the host drives the real loops.
- **Changes, not distinct values** — §1 item 5 asks for "the number of distinct values"; the stage prints the
  number of CHANGES (equal to distinct values − 1 for a one-way turn, and what the cadence is built from).
- **HA-04's sign is judged only when z is about the yaw axis** (deg/s or rad/s) — the brief judges sign and
  ratio side by side; a z on another axis says nothing about `GyroRateZ`'s sign, so it is "not judged" there
  (the first build also withheld it for rad/s; the review corrected that).
- **HA-214's verdict carries the drift's uncertainty** — the brief asks for "its drift per second"; the stage
  also prints the spread, the residual SD and 3 standard errors, and gives no verdict when the jitter could
  fake the drift.
- **2c.3's serial line goes through `hal::pros::serialLine()`, not `emit()`** — SER1's one call, serial only, as
  a motion tick's record is; `emit()` would also paint the panel 100 times and write the card.
- **The stage runs only after a GOOD STAGE 2**, asks START / SKIP first and, before that, holds STAGE 2's
  results on the panel until a touch (otherwise its rule and prompt erased them).
- **Button 2's status dot now includes STAGE 2c's verdicts** (`setVerdict` keeps the worst of the run) —
  the worksheet's Station A row 2 and N.2 step 6 say so.
- **Station A's `7 RUN ALL` row gained one clause** (§3.3 names only button 2's row).

## What is NOT done

- **Nothing of IMU1 has run on a robot.** HA-213 (the 10 ms copy) is PROS's sentence; HA-214 (the copy's phase) is
  unknown; HA-215 (a tick's reads within 5 ms) rests on HA-32/HA-102, unmeasured under load; HA-04 and HA-109 are
  invented. Worksheet **Station N** (~10 min, any Bench Tests session) is their first look; Station M's floor run
  (M.6) and Station K's (K.5) count as evidence for HA-179/HA-180 only on a build with Part A (`kRateHoldUs`).
- **What IMU1 does not cure** (HA-214's residual — the copy landing next to the tick's one read, or between two
  positions of a tick's first read): its cure is the gyro's own rate (`YawRateSource::GyroRateZ`) as the default — a
  later, separate change that needs its own go and STAGE 2c's HA-04/HA-109 verdicts first — or, for a GPS- or
  tag-corrected robot, the Localizer reading its rate at the top of `update()` (the brief's §5; no robot is
  corrected this season).
- **Part B's stage is designed and host-tested, not run:** its busy loops' behaviour on a brain (whether PROS's
  copy is made while a loop holds the processor), its panel layout and its timings are derived; the worksheet asks
  the team to write down anything that differs.
- *(Done 2026-11-06: the coordinator's verification and merge of Part B — the live log's "Part B — the coordinator's merge
  verification". Moved from this list at the merge.)*
