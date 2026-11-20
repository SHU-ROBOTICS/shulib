# CAL2 — progress log

Brief: [`CAL2-sides-by-coupling.md`](CAL2-sides-by-coupling.md). Executor: Claude Opus 5.5 (subagent), worktree
`.claude/worktrees/cal2` (branch `cal2`, from `9d51074`). Not committed by the executor.

## CAL2-A

- 2026-11-17 — log created before any code change. Read: RESUMING.md (standing rules), the brief in full.
- Read the calibrator (`drivetrain_calibrator.hpp`), the plant's coupled-train model (`sim/drive_plant.hpp`),
  the rigs (`calibration_test_rig.hpp`, `calibrate_golden_rig.hpp`), the panel's pulse/spin/STOPPED screens and
  robot one's logs (`robot_one_2026-11-1{6,7}.log`: 11-17 l.5698-5705 is the brief's §1 table, with the 0.6 deg
  floor crossed by FLEX at 6 V on earlier runs, l.5367/5369/5372).
- Code: `drivetrain_calibrator.hpp` — config `followThreshold` (2 deg) and `identifyYaw` (5 deg), the ramp
  3/6/9/12 V; `PulseVerdict::Followed`; `Refusal::SideConflict` and `Refusal::FollowerReversed` (appended, so no
  value moves); report `groupCount`, `groupSide[2]`, `identifyYawRad`, `PortObservation::group` / `followDeg`,
  `PulseRecord::followers` (a mask); `Step::SpinIdentify`; the heading-window rest (D4); three formatters.
  SIZE: `PortObservation` and `PulseRecord` keep their size (the new fields sit in padding: 64 B and 48 B on the
  host); the calibrator gains `group_[21]`, `groupLevel_[2]`, `identifying_`, and the rest ring (9 x 2 doubles
  + 2 size_t + 1 double) — about 200 B on the robot. Measured below.
- DELTA 1 (the brief's D1): coupling is read only when at least ONE other drive port stayed under
  followThreshold — "the robot held still". When every drive port moved, the robot moved, and a travel is no
  evidence of a shared train: the yaw decides as before. Reason: on the plant at k = 0.6 (test 25, a light
  robot whose idle side rolls along) every port "follows" port 11 and the run would end with one group;
  robot one's other side reads exactly 0.
- DELTA 2 (D1/D3): with no coupling the yaw is read at EVERY level, as before (not only at the top): an
  uncoupled drive then stops at the first level that turns the robot, i.e. it "behaves exactly as today".
  Coupling is read FIRST at each level: robot one's flex crossed its 0.6 deg floor at 6 V (11-17 log l.5367).
- DELTA 3 (D1, found by the suite): EVERY drive port is still pulsed. A grouped port is pulsed once, from its
  group's level, and must make its own group follow — the confirmation. Without it a slipping gear (robot
  two's port 18, plant `Decoupled` with drag 0.7) is dragged along like a mate (0.7 x 49 deg), joins the
  group unpulsed, and is never LEFT OUT (golden slip18's "9 of 9", test 24, T1c); and the per-port sign check
  (test 7) is lost. Robot one: 12 pulses (11 at 3/6, 12-15 at 6, 16 at 3/6, 17-20 at 6), not the brief's 4 —
  still under the 30-40 of the yaw ramp.
- DELTA 4 (union rule, D1): a pulse whose followers touch exactly ONE existing group joins it; both groups
  -> SIDE_CONFLICT naming the pulsed port; none, with two groups already -> SIDE_CONFLICT (a third group).
  A follower against its sign while the robot held still -> FOLLOWER_REVERSED naming that port.
- DELTA 5 (D3, the mixed rule): a port the yaw decided keeps its side; with ONE group the identification
  drives the group forward and the yaw-decided ports backward (they must all be one side, else SIDE_CONFLICT)
  and the group must come out on the other side; any port both grouped and yaw-decided must agree with its
  group's identified side, else SIDE_CONFLICT naming it.
- DELTA 6 (D4): the brief's `restHold` does not exist — the window is `restSettleTime` (0.5 s, the "half a
  second" the coordinator told the team lead). Its precondition tightens from >= 0 to > 0 (a window of zero
  has no rate). The heading is sampled 9 times across the window (any tick rate); a rest needs a full window.
- DELTA 7 (D2): the identification turn uses the spin's levels and start time (spinStartTime per level, then
  SPIN_NO_START naming "group A forward"); its fight monitor runs with groups A/B as the sides; after it the
  run rests, then spins as before.
- DELTA 8 (D5): the follower line prints the travels only for the LATEST pulse (`PortObservation::followDeg`
  holds one pulse; an older record prints its ports alone) — a per-record travel array would be 84 x 21 bytes
  on the opcontrol stack (the brief's landmine). The program prints every pulse on the tick it lands, so the
  serial record always carries them. The `config: pulses` line was reworded to fit the program's 240-byte line
  (the first draft cut off its end in the goldens).
- DELTA 9 (D7): the panel's chips line (the pulses screen's existing MEDIUM line) shows a group the spin has not
  named as `A 11 12 ...  B 16 ...`; the spin screen's live line says `FINDING LEFT` with `yaw +x of 5.0 deg at
  N V` during the identification turn. Both inside the existing budgets (static_assert'ed, P1 walks them).
- D6 decision: robot one's 0.6 deg / 1.0 deg/s stay. Under D4 the 1.0 deg/s is an average over 0.5 s (a
  0.5 deg change of heading); the precondition restYawRateFloor x pulseTime < pulseYawFloor (0.4 < 0.6) keeps
  its meaning. C1 and C5 run on `robotOneConfig()` (the two numbers) and pass; C6 is robot one's noise under them.
  `src/robots/` untouched.
- Tests: `test/geared_tank_rig.hpp` (robot one's drive: two trains with 10 deg backlash, 4 deg of train
  wind-up, a chassis one motor of five cannot roll, frame flex, IMU rate noise) and
  `test/calibrate_coupling_test.cpp` C1-C7. On that plant the yaw-only calibrator stops PULSE_STILL port 11
  at 12 V, "the other drive ports at most 4.0 deg" — robot one's failure — and the new one is Done.
- Existing tests moved (each read, each reason): drivetrain_calibrator 6 and 9 pin the YAW path with coupling
  off (`followThreshold` 360 deg) and check the coupled run too (6: same sides; 9: wheels up now stops at the
  identification turn, SPIN_NO_START "on blocks?"); 9's floor-from-both-sides keeps a 3-level ramp (the floor
  was measured at 9 V); 16's 0.10 s gap is 1 ns short on its last step (the clock's float sum landed past
  0.1 s once the pulses moved — the rule `> commandTimeout` is unchanged); 23 FOLLOWED and 2L + 8 pulses;
  24's port 18 starts at 6 V (its group's level) and is LEFT OUT at 12 V; 25's top level is 12 V; 26 has 14
  windows; 27 and 28 arm their IMU fault after the identification turn (it names the sides from the same
  IMU); 29 ticks a full heading window before "at rest" and 10 ticks of the hand turn before "not at rest";
  31/32 count 14 pulses, 31's 9 V line says FOLLOWED; 33's invariant is "every port before the current one is
  decided by a side or a group". IMU1 T11: atRest may read true only in the first window of the hand turn
  (the average lags a turn just begun); no pulse starts, it refuses RestTimeout "still rotating" as before.
  CalibrateProgram T1a/T-CFG the config line; T1a/T1q loop lines 4749/4748 ticks and 168 prints (14 pulses,
  not 30); T5b-1 port 9 pulsed at 3/6/9/12 V, its LEFT OUT line at 12 V; T5b-2 PULSE_STILL "at 12.0 V";
  T5b-3 wheels up stops SPIN_NO_START with coupling, PULSE_ROLLED words as before with coupling off.
- GOLDENS (all 34 re-recorded, 92 of 102 files; every moved line classified by
  `.claude/coordinator/scratch/cal2/classify.py`, nothing unclassified): the `config: pulses` line (12 V, the
  follow threshold, the heading rest) and the two intro lines; pulse lines — the 3/6 V levels of every mate
  are gone (each mate starts at its group's 9 V), the 9 V verdicts LEFT/RIGHT -> FOLLOWED, a follower line
  after each; a `groups:` line and a `sides:` line (-5.8 deg CW: A is LEFT; +5.8 CCW: A is RIGHT in the 8
  front-flipped / swapped runs; A 11 13 14 15 in the 5 runs where 12 is declared a mechanism; B without 18 in
  slip18); the `phase: SPIN` line names the identification; the `loop:` line (4762 ticks / 168 prints from 5498
  / 219). Slip18 still LEFT OUT 18 (at 12 V now) and scores 9 of 9; undeclared's port 9 is pulsed to 12 V
  (the test's 9000 mV -> 12000). Cards: the width's last byte (12.4999999999999x in: the spin starts where the
  identification turn left the heading, so the IMU sums other floats; ~5e-14 in) and the CRC, both copies.
  Screens: the mates' 3/6 V pulse screens gone, chips `A ...  B ...`, FINDING LEFT and its yaw line, and
  the live yaw of a pulse a frame apart (-5.5 -> -5.3). Every scenario's own expectations (forward score, reads)
  unchanged.
- Suite: green — 3860 cases, 3774321 assertions (`tools/suite_run.py green`).
- Docs: guide 19 (the table's "how", the candidates' ramp 3/6/9/12 V, steps 2-4 — "finding the sides", groups A/B,
  the confirmation pulse, FINDING LEFT, DONE steady — and the new STOPPED words); the changelog (top of
  `## API 2.9`: CHANGED behaviour of phase 3, the ramp, the heading rest, `restSettleTime > 0`; the additions);
  the HA register (HA-134 amended: the sign is the fallback and names the groups; HA-136: the two new numbers,
  the 12 V level, the average-rate rest; both index rows); `docs/api` regenerated.
- PROSE REVIEW (greps run, hits read): `drivetrain_calibrator|DrivetrainCalibrator|calibrate_panel|
  calibrate_program` over docs/ and README.md (minus docs/api, docs/internal) -> hardware-assumptions, changelog,
  roadmap (l.274: R3d Part 1's landing record, history — left), guide 19 (updated). `NOTHING MOVED|IT ROLLED|
  PULSE_STILL|PULSE_ROLLED|3, 6 and 9 V|3, then 6, then 9|6 and 9 V|COULD NOT SPIN` -> guide 19 l.100/126
  (fixed), changelog l.196/204 (the ROB1 history entry — left), HA register l.2642 (HA-134's 2F-1 amendment,
  history — the CAL2 amendment follows it). `ONE SIDE ONLY|SPUN WRONG WAY|NOT SETTLING|yaw rate under|
  restYawRateFloor|pulseLevelCount|pulseLevels|pushReady` -> only history entries and the HA-136 text (updated).
  `pulse` in guides 12/14/15 -> guide 12 l.318 ("a refused pulse names a port that is a mechanism": still true).
  `pulse|3, 6|9 V` in calibrate_screen/boot_check/front_check and src/robots -> robot one's facts comment
  l.478-480 ("a drift adds at most 0.4 deg to a 0.4 s pulse": still true under D4, an average) — left.
  NOT touched (the coordinator's at commit): roadmap "you are here", build-order `Next:`, the COMPLETED record.
- SIZE, measured on the ARM build (arm-none-eabi-g++ -Os, the robot's flags; sizeof read from .rodata of a probe
  object, `.claude/coordinator/scratch/cal2/size.cpp`): `DrivetrainCalibrator` 7712 -> 7928 B (+216);
  `CalibrationReport` 5616 -> 5632 (+16: groupCount, groupSide, identifyYawRad); `PortObservation` 64 -> 64;
  `PulseRecord` 48 -> 48. The program's `ProgramRun` gains two bools.
- Found by mutation M11 (SURVIVED first): the "full window" rule had two clauses that were almost the same (a full
  ring AND an old-enough oldest sample). Simplified to one — the oldest sample (`oldestSample()`) is at least
  `restSettleTime` old — and drivetrain_calibrator 29 now checks "not at rest" 0.1 s into a still robot.
- MUTATIONS (each line-count-preserving, a fresh binary each — the binary removed first, the five calibration test
  files touched, the doc gates passed, the header restored in a finally/SIGTERM/SIGHUP handler and its timestamp
  bumped; driver `.claude/coordinator/scratch/cal2/mutate.py`, filters `CAL2*,DrivetrainCalibrator*,golden*,
  CalibratePanel*,CalibrateProgram*`, under ulimit -v 4 GiB + timeout 600):
  M1 follow threshold x3 (robot one's 4 deg mates no longer follow): CAUGHT, 5 cases (C1 C2 C4 C5 C8).
  M2 group union off (every pulse a new group): CAUGHT, 141. M3 identification sign swapped: CAUGHT, 133.
  M4a drop "a drive port held still" (re-cut as `(anyStill || true)` after the first cut broke the build on an
  unused variable): CAUGHT, 3 (C3 k = 0.6, test 25, panel P10). M4b yaw fallback off: CAUGHT, 5 (C3, 6, 9 ...).
  M4c yaw before coupling: CAUGHT, 44 (C5, C8, goldens). M5 the instant rate back in atRest(): CAUGHT, 36 (C6,
  goldens). M6 no 12 V (first cut, pulseLevelCount 3, broke the doc gate — a documented default; re-cut as the
  top level one early): CAUGHT, 8 (C1, T5b-1/2, 9, 24, 25). M7 no both-groups refusal: CAUGHT, 1 (C4).
  M8 no confirmation pulse: CAUGHT, 49 (C1, C2, 24, slip18 ...). M9 no reversed-follower refusal: CAUGHT, 1 (C4).
  M10 no yaw-vs-group check: CAUGHT, 1 (C8). M11 full-window clause loosened: SURVIVED -> simplified (above);
  M11b `age >= 0`: CAUGHT, 1 (test 29). Every header restored (cmp against the pristine copy).
- GATES (final tree, after the last header change; the binary removed, briefing regenerated, docs/api regenerated,
  full `cmake --build build/test` exit 0, briefing regenerated again with the binary present):
  suite `suite run: green — 3861 cases, 3774334 assertions`; briefing check `generated block current`;
  api_doc_tool check-fresh exit 0 (silent), check-examples `987 quoted lines, all verbatim (24 source files)`,
  self-test `OK`, check-coverage `6501 public entities across 215 headers, all documented`, check-removability
  `no public doc references docs/internal/`; doc_staleness_audit `clean (37 live docs ...)`, its self-test `OK`;
  briefing self-test `OK (38 cases ...)`; ARM header gate over all 259 headers exit 0; CI greps `library is
  PROS-free ...` and `layering holds: core is sim-free`. No self-test plant (`git diff -- Makefile src/` has
  none; `git ls-files -o` in src/include/test lists only the two new test files).
- src build gate: `check` -> `src build gate: PASS (all 14 builds link or, robot one's and robot two's, refuse naming
  exactly their owed facts ...)`; `self-test` (under `timeout -s INT`; a first try cut at 40 min by my own timeout,
  rerun with 2 h) -> `src build gate self-test: OK (398 detector cases, each against a real build)` — about 43 min.
  No plant afterwards (`git diff -- Makefile src/`: none; untracked in src/include/test: the two new tests only).
- NOT covered by a dedicated test (said plainly): the one-group refusal "the yaw put port N on the other side from
  the ports before it" (`finishPulses`, yaw-decided ports on BOTH sides with one group) — the rig cannot make
  the yaw point two ways in one run; the code path is two lines and refuses into the safe state. **WRONG
  (review, 2026-11-19): the rig can (the `bias` hook, flex 0.2); C9 now tests it — see the review section.**
- STATUS: CAL2-A done by the executor, NOT committed. The coordinator owes: the COMPLETED record, the roadmap's
  "you are here" and build-order's `Next:`, and the commit.

## CAL2-A — resumed (2026-11-17 13:27 EST, `date`)

- A second executor (Claude Opus 5.5) resumes after the weekly usage limit cut the first one. Read: the log above,
  the brief (all of it, §6 included), `git status` / `git diff -- src/ Makefile` (empty: no gate self-test plant;
  untracked in src/include/test: `test/calibrate_coupling_test.cpp`, `test/geared_tank_rig.hpp` only).
- Found NOT done: the brief's §6 (added 09:10) — the calibrator still lives on the opcontrol stack
  (`std::optional<DrivetrainCalibrator> cal;` inside `CalibrateProgram::run`, calibrate_program.hpp l.731, and
  `detail::ProgramRun run{}` l.763). The first run's log never mentions it. That is this run's work, after a fresh
  build and suite to check the first run's tree.
- The first run's tree, checked fresh: the five calibration headers and the calibration tests touched (every
  dependent object recompiled), the binary removed first, `cmake --build build/test` exit 0, then
  `suite run: green — 3861 cases, 3774334 assertions` — the first run's figures exactly. Its mutation backup
  (`mut_backup_drivetrain_calibrator.hpp`) equals the live header (cmp).
- §6 MEASURED FIRST (GCC's `-fstack-usage -fcallgraph-info=su` over robot one's Calibrate TU,
  `src/programs/calibrate_program.cpp` with `-DSHULIB_ROBOT_ONE_2026 -DSHULIB_PROGRAM_CALIBRATE` and the robot's
  flags from common.mk; the CAL1 reviewer's `maxpath.py` rooted at `bench::runCalibrate`, and `through.py` for
  the deepest path THROUGH a function; all in `.claude/coordinator/scratch/cal2/stack/`). Deepest in-TU path
  (the save: `record` -> `finishDone` -> `save` -> `saveRecordSafely` -> SafeSave -> CRC), of PROS's 32,768 B:
  base 9d51074 **31,824 B**; CAL2 before §6 **32,072 B** (run() 10,280 -> 10,480: the +216 B calibrator) — past
  32 KiB before newlib's printf/file frames and VEXos's, which GCC cannot see. So the release Calibrate on robot
  one had under 1 KB for those, and the 140 B measurement build died at the end of its run, as §6 says.
  The frames: `bench::runCalibrate` 1,240; `hal::pros::runCalibrate` 16,456 (sizeof on ARM:
  `ProsDeviceProvider` **14,696** — it was 7,288 at CAL1's measure, 10-29 — `CalibrateProgram` 1,696);
  `CalibrateProgram::run` 10,480 (`DrivetrainCalibrator` 7,928, `ProgramRun` 576, `PanelView` 656).
- §6 CODE (`calibrate_program.hpp`): `detail::ProgramStorage { std::optional<DrivetrainCalibrator> cal;
  ProgramRun run{}; }` and `detail::programStorage()`, a function-static `constinit` (constant-initialised:
  no guard variable, nothing run at boot — `nm` shows the object `V ..programStorage..storage` and no
  `_ZGV` guard); `static_assert` it is trivially destructible (a task killed mid-run leaves its calibrator
  there, and the next run's `emplace` destroys it without touching its dangling references). `run()` binds
  `cal` and `run` to it (two lines; RUN AGAIN's `emplace` and `run = ProgramRun{}` unchanged). The class doc:
  "One runs at a time". Nothing else moved: the program object, the provider, the `Held` record stay where
  they were (the brief names the calibrator and the run state).
- §6 MEASURED AFTER (same method), robot one's Calibrate, deepest in-TU path by where it goes:
  the save with a card: 31,824 (base) / 32,072 (CAL2 before §6) -> **23,552 B** (`run()` 1,960);
  the save with NO card: the same frames (the no-card branch skips no function GCC sees: `loadRecord`'s
  path 31,160 / 31,408 -> 22,888; `save` -> `saveRecordSafely` as above, 23,552); the HELD screen: its view
  (`TouchGate::view`) 29,440 / 29,672 -> 21,152, its SAVE (`resolveHeld` -> save) 30,208 / 30,440 -> 21,920.
  Margin for newlib's and VEXos's frames: 944 B (base) -> 9,216 B. Robot two's Calibrate TU refuses to
  build here (its owed facts' `static_assert`, as the src gate expects), so robot one's stands for both (one
  code path). The next big object is the provider (14,696 B on runCalibrate's frame) — for the coordinator.
- §6 TEST `test/calibrate_program_stack_test.cpp` (T-STACK, new): three whole runs (the save with a card; no
  card; FRONT FLIPPED held, then USE NEW) each on a thread whose 1 MiB stack is painted first
  (`pthread_attr_setstack`); the bytes it wrote, less an empty thread's (glibc's own block), must be under
  12 KiB — measured 9,696 / 8,896 / 9,696 B on GCC 13.3 -O0 (the CI's compiler); the host calibrator is 8,168 B,
  so one on the stack makes it 17.8 KB or more. And the static storage, poisoned first (`cal.reset()`,
  `run.number = -7`), must hold THAT run's calibrator at Done and its record (`run.number` 1 / 1 / 2, `save`
  Saved / NotSaved / Saved). DELTA 10: a host bound of 12 KiB, not "less than one calibrator" (my first
  cut): the host's -O0 frames alone are 9.7 KB, more than the host calibrator — measured, then re-cut.
- §6 MUTATIONS (`.claude/coordinator/scratch/cal2/mutate2.py`, the first run's driver with the new test
  touched too; a fresh binary each, `--target shulib_tests`, filter `CalibrateProgram*`, ulimit -v 4 GiB +
  timeout 600; the header restored in finally/SIGTERM/SIGHUP, timestamp bumped, `cmp` against
  `calibrate_program.hpp.pristine_s6`: RESTORED):
  MS1 the calibrator back on the stack (`std::optional<DrivetrainCalibrator> calOnStack; ... cal = calOnStack;`):
  CAUGHT, 1 case (T-STACK). MS2 the run's record back on the stack (`ProgramRun runOnStack{}; ... run =
  runOnStack;`): CAUGHT, 1 case (T-STACK — the record check; 576 B alone would sit inside the host budget).
- Docs (§6): `hal/pros/calibrate.hpp`'s THE STACK paragraph re-measured (23,552 B, the frames, the static
  storage, the provider next) and `docs/api` regenerated (pros-calibrate, calibrate_program); the changelog's
  CAL2 entry, a "The stack" bullet under CHANGED. Guide 19 says nothing about the stack (the team never sees
  it) — unchanged.
- PROSE REVIEW (§6; greps run, hits read): `22,464|7,712|7712|one per program|10,272|opcontrol.*stack|stack.*opcontrol`
  over docs/, include/, src/, README.md -> changelog l.1569-1570 (CAL1's history entry, "22,464 B" — left),
  R3d-PROGRESS l.2327 (history — left), cookbook 6 l.108/325 and HA register l.3482 (the driver program's own
  static state — unrelated, true), the two new texts (THE STACK, the class doc). `calibrat.*stack|stack.*calibrat`
  over the HA register, the guides and the roadmap: none.
- GATES (final tree, after the last header change; the binary removed, the briefing regenerated, docs/api
  regenerated, the full `cmake --build build/test` exit 0, the briefing regenerated with the binary present):
  suite `suite run: green — 3862 cases, 3774398 assertions` (one case more: T-STACK); briefing check `generated
  block current`; api_doc_tool check-fresh exit 0 (silent), check-examples `987 quoted lines, all verbatim (24
  source files)`, self-test `OK`, check-coverage `6501 public entities across 215 headers, all documented`,
  check-removability `no public doc references docs/internal/`; doc_staleness_audit `clean (37 live docs ...)`,
  its self-test `OK`; briefing self-test `OK (38 cases ...)`; ARM header gate over 259 headers `CLEAN`; CI greps
  `library is PROS-free ...`, `layering holds: core is sim-free`. src build gate `check` -> `PASS (all 14 builds
  link or, robot one's and robot two's, refuse naming exactly their owed facts (10 refused ...))`; `self-test`
  (under `timeout -s INT 7200`) -> `OK (398 detector cases, each against a real build)`, about 50 min. No plant
  afterwards (`git diff -- Makefile src/`: empty; untracked in src/include/test: the three new test files).
- STATUS (2026-11-17 14:46): CAL2-A done by the executor, §6 included, NOT committed. The coordinator owes: the
  reviewers, the COMPLETED record, the roadmap's "you are here" and build-order's `Next:`, the commit; and on
  robot one, the run to the save (the stack is GCC's measure — newlib's and VEXos's frames are not in it).

## CAL2-A review findings and their disposition

- Started 2026-11-19 16:21 EST (`date`): a third executor (Claude Opus 5.5) takes the three
  reviewers' 17 findings (correctness 5, tests 4, docs 8; no overlap to merge). Read first: RESUMING.md's
  standing rules, the brief in full, this log, the reviewers' probes (`scratch/cal2/review-correctness/tree/
  test/probe.cpp`, `scratch/cal2/review-testgate/probes/cal2_review_probe_test.cpp`) and robot one's pulse
  lines (11-17 log l.5367-5375, l.5503-5506, l.5698-5705, l.5917-5925).
- **F1 (correctness, major) — a grouped port's confirmation could stop robot one or LEAVE OUT a working motor.
  REAL.** Robot one's mates move theirs weakly: port 12's own pulses showed others at most 0.0 / 3.6 / 3.6 / 1.2
  deg at 3 / 6 / 9 / 12 V (11-17 l.5503-5506), ports 13/14 4.8 and 2.4 (l.5370-5375); its working shafts turned
  up to 18.0 deg with nothing following (l.5373) and 24.0 deg moving the train (l.5921) — over `decoupledOwnMin`'s
  20. Reproduced on the plant (the reviewer's P1/P2): PULSE_STILL port 12; with +10 deg of own shaft, Done with
  port 12 LEFT OUT. Fix, keeping the confirmation (it is what finds slip18's slipping gear and checks each
  port's own sign): (1) a GROUPED port that reaches the top level with no follower, no turn and nothing rolled
  is **KEPT** in its group (`PulseVerdict::Kept`, appended; serial word `KEPT in its group (no follower)`), not
  refused; (2) `decoupledOwnMin` 20 -> **90 deg**: 20 sat inside robot one's measured working range; a slipping
  gear spins hundreds (the plant's port 18: 398-874 deg), a red cartridge ~200 deg in a 12 V pulse. One number
  for both paths: a yaw-path port with 20-90 deg of shaft now stops PULSE_STILL instead of being LEFT OUT (the
  safer failure). Not the brief's "pulse only ungrouped ports": slip18 would then reach the spin and stop
  SPIN_FIGHT instead of finishing without port 18, and robot one's 12 pulses already finish on the plant.
  Test C12 (the reviewer's P1/P2 plus a mate spinning 300 deg): KEPT at 0 and +10 deg, LEFT OUT at +300.
- **F2 (correctness, minor) — FOLLOWER_REVERSED when a turn rolls the other side back part-way. REAL**
  (reproduced: P3, a 0.5 V chassis, two of the idle five read -4 deg). Fix: a port that went back while the
  robot turned (|yaw| >= pulseYawFloor) is MOVED (neither follower nor still), not reversed; one that went back
  while the robot held still is refused as before. A wrong captured sign still shows: its own pulse's
  CONTRADICTION, or the spin's coupled-side monitor. Test C13 (P3, with coupling and without): Done, the yaw's
  sides.
- **F3 (correctness, minor) — "no followers" when every port turned. REAL.** `recordPulse` now stores the
  mask on every pulse (an argument, not a write after the fact — the old write also hit the last record when
  the record array was full); the follower line says `turned with it (no group): ...` when ports moved but
  made no group (the whole robot moved, or the pulse was refused). No golden prints it (no golden pulse moves
  every port). Test: C3's k = 0.6 subcase reads the line on every such pulse (at least 4).
- **F4 (correctness, minor) — the averaged rest lags a turn just begun. REAL, a design note; NOT changed in
  code.** The lag is restYawRateFloor / rate of the window (robot one, 1 deg/s against a 3 deg/s turn: a third
  of 0.5 s; a 30 deg/s hand turn: 17 ms). The reviewer's fix (the newest ~70 ms interval under "a few times"
  the floor) adds an invented factor that could bring back the flicker CAL2 removed, for a case the motors'
  check already covers on the floor (a turned tank turns its motors); for IMU-only motion the same gap already
  existed INSIDE every pulse (a turn begun 1 ms after a pulse starts was never seen before CAL2 either). Said
  plainly instead: the changelog's Rest bullet, IMU1 T11's name ("never starts a pulse") and comment.
- **F5 (correctness, minor) — C6 tested D4 only at twice the floor. REAL.** C6 now runs robot one's floor at
  0.95 deg/s (ready for more than 1000 ticks) and 1.05 deg/s (never ready). Mutation R6 (the window rate divided
  by restSettleTime, not the sample's real age): CAUGHT (C6 among 34).
- **F6 (tests, major) — three new branches untested. REAL** (S2, S3, S5 survived the full suite for the
  reviewer). Added C9 (one group, yaw ports on both sides -> SIDE_CONFLICT port 16, no turn powered), C10 (two
  groups plus port 15 by yaw -> Done, 15 LEFT, `groupSide[1]` = +1), C11 (one group of all ten -> ONE_SIDED, no
  turn powered); "never powered" is read off `status().identifying` on every tick. The log's "rig cannot"
  sentence is corrected in place (above).
- **F7 (tests, minor) — a lone reversed follower untested. REAL.** C4 gains it (port 12 alone reads back ->
  FOLLOWER_REVERSED 12 at 6 V, two pulses, no ramp to 12 V), and the old subcase has two backward ports (13 and
  14) so the FIRST is named (S7).
- **F8 (tests, minor) — `groupSide[1]` never asserted. REAL.** C1 (+1), C5 (both wirings), C8 (0 with one
  group), C10 (+1).
- **F9 (tests, minor) — the assertion drop not logged. REAL; logged here.** Base 9d51074 -> CAL2: 3,884,687 ->
  3,773,095 assertions without route_preview (-111,592, -2.9 %; the reviewer's per-case XML, re-summed by me):
  CalibratePanel P1 -67,966, P3 -26,316, additions N2 -19,006 — per-tick walks over shorter runs (14 pulses,
  4,762 ticks a run instead of 5,498); every other changed case moved by under 40, and the new ones add 1,762.
  No coverage lost: the same states are walked.
- **F10 (docs, major) — "(3, 6, 9 V)" left in the program's banner, its header, guide 19's team file. REAL;
  the first prose review never grepped `calibrate_program.hpp` and its patterns missed "(3, 6, 9 V)".** Fixed to
  "(3, 6, 9, then 12 V)": calibrate_program.hpp l.14, l.141 and the no-mechanism banner l.1221; guide 19's team
  file with `calibrate_team_file_example_test.cpp` (check-examples keeps them verbatim); T-case l.1694's
  expectation; `calibrate_program_rig.hpp`'s comment. No golden declares no mechanism, so no golden moved.
  Wider grep `3, then 6, then 9|\(3, 6, 9 V\)|3, 6 and 9 V|6 and 9 V|3, 6, 9\b` over include/ src/ test/ docs/
  README.md (minus docs/api): left only history (changelog l.3268, an API 2.x entry; the HA-134 2F-1
  amendment) and `bench_menu_grid_test.cpp`'s "stations 3, 6, 9" (unrelated).
- **F11 (docs, major) — the bench worksheet's Station G described the old phase 3. REAL; updated** (it is the
  live runbook for robot one's run): G.3 (a geared drive on blocks stops `COULD NOT SPIN` at the short turn; an
  ungeared one in the pulses after 12 V), the PUSH row (stillness = the heading held half a second), the PULSES
  row (the 3-12 V ramp until others follow, groups A/B, every motor at least once, KEPT, ten motors), the SPIN row
  (FINDING LEFT, 5 deg, 6 V, COULD NOT SPIN), LEFT OUT (90 deg at 12 V; KEPT), the stop words (12 V; SIGN
  MISMATCH's second meaning; SIDES UNCLEAR; COULD NOT SPIN) and G.7 (two lines per pulse, FOLLOWED / KEPT, the
  `groups:` and `sides:` lines), quoted from the clean golden.
- **F12 (docs, minor) — three golden moves unexplained. CHECKED.** (a) `right.inchesPerRadian`'s last byte
  (card offset 63) moved in clean and flip-again only — of 26 changed cards; both keep a record from a run
  after the first (run 2 of 2, run 3 of 3), whose push starts where the run before left the plant — which now
  includes the identification turn — so the encoders' float sums differ in the last bit (1.6250000000000324
  -> 1.625000000000033). Every single-run card keeps offset 63 (`scratch/cal2/rv_carddiff.py`). (b) The width's
  second byte (offset 72) moved in swap, typed-flip-new and typed-undecodable-flip: the same width move as
  offset 71, a carry into the next byte (~9e-14 in). (c) curved's green `10 of 10 turned` (pushReady) now comes
  after the last `drift +40.0 deg -- TOO CURVED` frame: D4 — a heading that moved in the last window is not at
  rest. Shown on fresh builds (`scratch/cal2/rv_curved_check.py`, header restored and `cmp`-checked after each):
  atRest WITHOUT the heading-rate test (motors + a full window) gives curved's push screens (run 1 and run 2's
  push, 140 lines) byte-identical to base 9d51074's; with it, the line moves. (A first try, atRest on the
  instant rate again, moved it too — but through a different cause: CAL2 also dropped the push's per-tick
  `imu_.yawRate()` read, so the adapter's first rest read is a rate over the whole push — the CAL1 B12 relation,
  CAL1-PROGRESS l.4137. Not D4's evidence; recorded so nobody repeats it.)
- **F13 (docs, minor) — the stack history overstated. REAL.** hal/pros/calibrate.hpp now pairs 31,824 B with the
  base 7,712 B calibrator and 32,072 B with CAL2's 7,928 B, and says what was seen: "a measurement build ~140 B
  over the release stopped with FATAL ERROR on robot one's opcontrol task at the save (2026-11-17)"; the same
  words in calibrate_program.hpp's ProgramStorage doc and the changelog.
- **F14 (docs, minor) — followThreshold's evidence misquoted. REAL.** The brief's table: 5-6 deg at 6 V (2-6 over
  all levels). `followThreshold`'s doc, HA-136 ("robot one's serial log, 2026-11-17; 10-23 deg in the completed
  run"; the 181-character line rewrapped), HA-134's CAL2 amendment ("2-6 deg (5-6 deg at 6 V)") and the rig's
  header comment ("2-6 deg").
- **F15 (docs, minor) — HA-134/HA-136 Source and Blast radius. REAL.** HA-134: `tickIdentify()`/`identifySides()`,
  coupling C5 and M3 in Source; "one comparison in finishPulse() and, for grouped ports, one in tickIdentify()".
  HA-136 Source: calibrate_coupling_test.cpp for followThreshold, identifyYaw, the average rest (C6 at 0.95 /
  1.05 deg/s) and decoupledOwnMin's 90 deg (C12, R2).
- **F16 (docs, minor) — overstatements. REAL.** Guide 19: "every motor is pulsed at least once" (+ KEPT and LEFT
  OUT in one line), "a robot on blocks whose sides are geared together stops at the short turn". Changelog: the
  stack moved under its own "CHANGED (`CalibrateProgram`'s stack)" heading; the Rest bullet's lag clause; KEPT,
  the reversed rule, the 90 deg, the follower line. IMU1 T11 renamed and its comments say what it now checks.
- **F17 (docs, minor, pre-existing) — "nothing has run on robot one". REAL; fixed with plain facts** (the brief
  and the logs): guide 14 (the Calibrate bullet; "Robot one has run Calibrate and stopped in its pulses", the
  claims I cannot check — census, readings, photo — dropped rather than kept), guide 19 l.15-16 and its "has not
  yet run" paragraph, README l.102-103 (the touch screen adapter's program ran on robot one in the form before
  this one). changelog l.305 (a dated entry's own claim) — history, left.
- Overlaps: none merged — the 17 findings name 17 different defects (F1 and F2 share `finishPulse`).
- SIZE: unchanged — re-measured on the ARM toolchain (the first run's `size.cpp`): `DrivetrainCalibrator` 7,928 B,
  `CalibrationReport` 5,632, `PortObservation` 64, `PulseRecord` 48 (KEPT is an enumerator; the mask was there).
- GOLDENS (this round): 33 serial logs move ONE line each — the `config:` line's `own shaft 20 deg` -> `own shaft
  90 deg` (`decoupledOwnMin`); noimu prints no config line. Nothing else moved (re-recorded to
  `scratch/cal2/rv_golden_out`, every diff line counted). T1a and T-CFG's config-line expectations likewise.
- PROSE REVIEW (this round; greps run, hits read): `LEFT OUT|decoupled|20 degrees of shaft|20° of shaft|own shaft
  20` over docs/ and README.md (minus docs/api, docs/internal) -> guide 12 l.314/318 (still true), guide 19
  (updated), changelog l.1536/3273 and HA l.2551/2645/2785 (history or the plant's slip model — true), roadmap /
  master plan / legacy vocabulary ("decoupled" motion — unrelated). `no followers|FOLLOWER_REVERSED|turned with
  it` -> only the new changelog text. `ran out of stack|overflowed|FATAL ERROR|opcontrol task died` over docs/
  include/ src/ README.md -> the three rewritten texts; the rest unrelated (median sums, a line cut). `IMU1 T11|
  never finds rest` -> CAL1-PROGRESS l.4005/4137 (history — left). `nothing has run on robot one|not run on a
  robot in that form` -> fixed (above) and changelog l.305 (history).
- MUTATIONS (this round; `scratch/cal2/rv_mutate.py`: the header backed up, one line-count-preserving
  replacement, every test/src TU naming "calibrat" touched, the binary removed, a fresh `--target shulib_tests`
  build, filters `CAL2*,DrivetrainCalibrator*,golden*,CalibratePanel*,CalibrateProgram*,IMU1*` under ulimit -v
  4 GiB + timeout 600, restored in finally/SIGTERM/SIGHUP/SIGINT with its timestamp bumped and `cmp`-checked;
  the header `cmp`-equal to its pristine copy after the batch). Every one CAUGHT:
  R1 grouped top-level no-follower refused again (KEPT off): C12. R2 the 20 deg slip bound
  (`min(decoupledOwnMin, 20 deg)` at the use; the default itself is a documented value the doc gate pins):
  C12 at +10 deg. R3 a turned robot's backward port counted reversed: C13. R4 the mask stored on FOLLOWED
  only: C3. R5 the line's word back to "followers": C3. R6 the window rate divided by restSettleTime: C6 (the
  0.95 deg/s side) and 33 others. S1 `coupled` without the reversed clause: C4 (lone backward port: refusal
  PULSE_STILL, not FOLLOWER_REVERSED). S2 the one-group yaw-both-ways refusal off: C9. S3 the group check on a
  yaw-decided port in a two-group run: C10. S4 `groupSide[1]` = +a: C1, C5, C10. S5 the ONE_SIDED refusal in
  finishPulses off: C11. S7 the LAST backward port named: C4 (14 named, 13 expected).
- SUITE AND GATES (final tree; the binary removed, docs/api and the briefing regenerated, `cmake -S test -B
  build/test`, the full `cmake --build build/test -j8` exit 0, the briefing regenerated with the binary present):
  `suite run: green — 3867 cases, 3774497 assertions` (5 cases more: C9-C13); briefing check `generated block
  current`, its self-test OK (38 cases); api_doc_tool check-fresh exit 0, check-examples `987 quoted lines, all
  verbatim (24 source files)`, self-test OK, check-coverage `6502 public entities across 215 headers, all
  documented` (one more: `PulseVerdict::Kept`), check-removability `no public doc references docs/internal/`;
  doc_staleness_audit `clean (37 live docs ...)`, its self-test OK; doc_readability check-tone OK (47 pages),
  check-nav OK (262 pages); the ARM header gate over 259 headers `CLEAN`; CI greps `library is PROS-free ...`,
  `layering holds: core is sim-free`.
- Docs touched this round: guide 19, guide 14, README.md, the changelog, the HA register (HA-134, HA-136), the
  bench worksheet's Station G, docs/api (regenerated), PROJECT-BRIEFING (regenerated), this log. Deliberately
  not touched: roadmap "you are here" and build-order `Next:` (the coordinator's, at commit); history entries
  (changelog l.305 and l.3268, the HA-134 2F-1 amendment, CAL1-PROGRESS l.4005/4137).
- src build gate (after the last code change): `self-test` under `timeout -s INT 7200` -> `src build gate
  self-test: OK (398 detector cases, each against a real build)` (in its copy, build/src-gate-selftest/);
  `check` -> `src build gate: PASS (all 14 builds link or, robot one's and robot two's, refuse naming exactly
  their owed facts (10 refused; ...))`. No plant afterwards (`git diff -- Makefile src/`: empty; untracked in
  src/include/test: the three CAL2 test files only); the calibrator header `cmp`-equal to its pristine copy.
- STATUS (2026-11-19 17:35 EST): every review finding dispositioned (16 fixed, F4 documented with its reason),
  every gate green, NOT committed. The coordinator owes: the COMPLETED record, roadmap "you are here",
  build-order `Next:`, the commit; and on robot one, the run (now with the KEPT rule and the 90 deg slip bound).

## Verification-round findings and their disposition

- Started 2026-11-19 17:49 EST (`date`): a fourth executor (Claude Opus 5.5) takes the fresh verifier's three
  findings. Read first: this log, the brief, the verifier's probes (`scratch/cal2/verify/tree/test/
  zz_verify_probe_test.cpp`: V-P1..P3, V1, V2), robot one's logs (11-16 l.44-322, 11-17 l.5289-5931).
- **V-F1 (minor) — KEPT took a grouped port whose own shaft never turned. REAL** (the probe V1: port 12 reads 0
  everywhere -> `KEPT`, Done with 12 on the left). Fix at the root: KEPT needs the port's own shaft to turn
  `movedThreshold` (5 deg, an existing number; robot one's working motors turned theirs 8.4-24 deg at every level,
  11-17 l.5367-5375, l.5503-5506, l.5698-5705, l.5917-5925); under it the port stops `PULSE_STILL` as before the
  review round. Test C12 gains own 0 / 4 / 6 deg at port 12 with no mate moving: `PULSE_STILL` port 12 at 12 V,
  `PULSE_STILL`, `KEPT`. Observed red against the round's rule: mutation V1 (below).
- **V-F2 (minor) — the forward drag made one group and ONE_SIDED. REAL** (V2: a 0.5 V chassis, two of the idle
  five read the turn's roll forward -> `ONE_SIDED: one group of 10`; yaw only: Done). Not left as a pinned
  refusal: the verifier's "better" rule, made exact. A pulse that turned the robot past the floor AGAINST the
  turn of its group's FIRST pulse (itself past the floor) leaves that group, and that group's ports are not its
  followers (`finishPulse()`, `groupYaw_[2]` — 2 bytes, set when a group is made). Each port is pulsed anyway
  (DELTA 3), so the dragged ports' own pulses take them out of A and into B (C13: two groups, 16 and 17 in B). A yaw under the floor gives no direction (a group's first pulse) and takes no port out (a
  mate's confirmation): robot one's port 11 turned it -0.48 deg at 6 V under its 0.6 floor (11-17 l.5700).
  Robot one's path: every one of its 35 logged pulses (11-16 and 11-17) turned the robot its side's way — 31 of
  ports 11-15 all negative, 4 of port 16 all positive — so the rule fires there only on a pulse whose yaw is
  against its own side; such a port then has its yaw side, checked by the identification turn (one group) or by the spin's
  coupled-side monitor (its train fights the side it was put on). Tests: C13 gains the forward drag (both
  couplings Done with the right sides; with coupling two groups, ports 16 and 17 in B); C14 (new): port 11's
  6 V pulse, then port 12's, reads +0.2 deg against the others' -0.8 — all five stay in A.
- **V-F3 (minor) — the screen adapter's "has not run on a robot"; guide 14's "every time". REAL.** The 11-16
  build e8fa3de constructs `ProsBrainScreen` in `hal/pros/calibrate.hpp` (l.129, l.160; also 9d51074, the 11-17
  builds). `brain_screen.hpp`: "inside "shulib Calibrate" this adapter ran on robot one on 2026-11-16 and
  11-17". Guide 14: "every run of the form before this one that reached the pulses stopped there" (11-16 run 1
  `PUSH_CURVED` l.192 and run 3 `ABORTED` at the push l.257 never reached them; runs 2 and 4 `PULSE_ROLLED`;
  the 11-17 yaw-only builds `PULSE_STILL` l.5377/5508/5707). The same miss in two more headers, found by the
  wider grep: `drivetrain_calibrator.hpp`'s "IT HAS NEVER FINISHED ON A ROBOT ... THIS version has never run on
  a robot at all" -> "THIS FORM HAS NOT RUN ON A ROBOT" and the form before CAL2's robot-one runs;
  `calibrate_program.hpp` l.100 gains the same sentence. `docs/api` regenerated.
- PROSE REVIEW (this round; greps run, hits read): `has not run on a robot|never run on a robot|never finished on
  a robot|NEVER FINISHED|not run on a robot|has not run|never ran|not yet run|hasn't run` over include/ docs/
  README.md src/ (minus docs/api, docs/internal), Calibrate- or screen-related hits -> the three headers above
  (fixed); guide 19 l.153 and guide 14 l.75 ("this program, in this form" — true); changelog l.21 (CAL2's own
  entry — true), l.367/1481/3250 and roadmap l.561 (dated history — left); rest_capture.hpp l.9 (R5a's capture,
  not in Calibrate — true). `every time|each time|every run` over guides 14/19, README, the changelog ->
  guide 14 (fixed); changelog l.244 (the thresholds fix's dated entry, "so every run stopped with IT ROLLED"
  — two of the four 11-16 runs never reached the pulses; history, left, for the coordinator). `KEPT in its
  group|stays in its group|moves no mate|moves none of its` over docs/ README.md include/ -> the changelog,
  guide 19 step 3, Station G (the PULSES row, the LEFT OUT paragraph, NOTHING MOVED's words) and the header's
  phase table: all now say the own shaft must turn.
- Docs touched this round: the changelog's CAL2 CHANGED bullet (KEPT needs the own shaft; the drag rule), HA-134
  (the amendment, its Source and Blast radius), HA-136 (`movedThreshold` as KEPT's floor; C12's new cases),
  guide 19 step 3, guide 14, Station G (the PULSES row, the LEFT OUT paragraph, `NOTHING MOVED`'s words), the
  three headers' run histories, `docs/api` (regenerated), PROJECT-BRIEFING (regenerated). The first HA-134 Source
  edit named this log, a docs/internal page: the doc gate's check-removability refused it inside mutation V2f's
  build (BUILD BROKE), reworded, V2f re-run (below).
- SIZE: unchanged on the ARM toolchain (`scratch/cal2/size.cpp`, `vr_size.o`): `DrivetrainCalibrator` 7,928 B
  (`groupYaw_[2]` sits in padding), `CalibrationReport` 5,632, `PortObservation` 64, `PulseRecord` 48.
- GOLDENS: none moved (the 34 compare green on the final binary). The drag rule is live on their path — a
  group's first pulse turns the golden plant past the default 2 deg floor (clean: port 11 at 9 V, -6.47 deg),
  and the sign-flipped mutant V2e reddens all 34 golden cases — and no golden pulse turns against it.
- MUTATIONS (this round; `scratch/cal2/vr_mutations.sh` through the review round's driver `rv_mutate.py`: the
  header backed up, one line-count-preserving replacement, every test/src TU naming "calibrat" touched, the binary
  removed, a fresh `--target shulib_tests` build, filters `CAL2*,DrivetrainCalibrator*,golden*,CalibratePanel*,
  CalibrateProgram*,IMU1*` under ulimit -v 4 GiB + timeout 600, restored in finally/SIGTERM/SIGHUP/SIGINT with
  its timestamp bumped and `cmp`-checked; the header `cmp`-equal to `vr_pristine_drivetrain_calibrator.hpp`
  after the batch). Every one CAUGHT:
  V1 KEPT without the own-shaft floor (the review round's rule): C12 (own 0: Done instead of `PULSE_STILL`).
  V2a the drag rule off (the code before this round): C13 forward, `ONE_SIDED: one group of 10` — the
  verifier's V2 exactly. V2b the pulsed port stays in a group its turn opposes: C13. V2c the opposed group's
  ports still followers: C13. V2d a group's first pulse under the floor given a direction: C14 (port 11 odd).
  V2e the first pulse's sign flipped: 139 cases (C5, C8-C10, C13, C14, all 34 goldens). V2f a pulse under the
  floor takes a port out: C14 (port 12 odd) — first cut BUILD BROKE on my own doc edit (above), re-run CAUGHT.
- SUITE AND GATES (final tree; the binary removed, docs/api and the briefing regenerated, `cmake -S test -B
  build/test`, the full `cmake --build build/test -j8` exit 0, the briefing regenerated with the binary present):
  `suite run: green — 3868 cases, 3774555 assertions` (one case more: C14); briefing check `generated block
  current`, its self-test OK (38 cases); api_doc_tool check-fresh exit 0, check-examples `987 quoted lines, all
  verbatim (24 source files)`, self-test OK, check-coverage `6502 public entities across 215 headers, all
  documented`, check-removability `no public doc references docs/internal/`; doc_staleness_audit `clean (37 live
  docs ...)`, its self-test OK; doc_readability check-tone OK (47 pages), check-nav OK (262 pages); the ARM header
  gate over 259 headers `CLEAN`; CI greps `library is PROS-free ...`, `layering holds: core is sim-free`.
- src build gate (after the last code change): `self-test` under `timeout -s INT 7200` -> `src build gate
  self-test: OK (398 detector cases, each against a real build)` (in its copy, build/src-gate-selftest/);
  `check` -> `src build gate: PASS (all 14 builds link or, robot one's and robot two's, refuse naming exactly
  their owed facts (10 refused; ...))`. No plant afterwards (`git diff -- Makefile src/`: empty, no
  "(self-test plant"; untracked in src/include/test: the three CAL2 test files only); the calibrator header
  `cmp`-equal to its pristine copy.
- STATUS (2026-11-19 19:04 EST): the three verifier findings fixed at the root (V-F1 KEPT's own-shaft floor;
  V-F2 the drag rule, not a pinned refusal; V-F3 the run histories, three headers more than named), every gate
  green, NOT committed. The coordinator owes: the COMPLETED record, roadmap "you are here", build-order `Next:`,
  the commit; the changelog l.244 history sentence is theirs to judge; and on robot one, the run.
