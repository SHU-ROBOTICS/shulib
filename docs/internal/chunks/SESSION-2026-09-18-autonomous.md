# Coordinator's session log — 2026-09-18, autonomous overnight session

> **Why this file exists.** The team lead (jal) went to sleep at ~05:30 on 2026-09-18 with this
> instruction: *"you can keep working and open some agents to finish faster and you can just keep
> going as long as you're safe about it … be deliberate and be safe AND DOCUMENT EVERYTHING YOU DO
> that is most important since I won't be right here."* And: *"in the morning I will hook up my PC
> to the new drivetrain to test the library motion when it is done."* This log is the record of
> every decision and action the coordinator (Claude Fable 5.1) took without him in the room, in
> order, with times (local, EDT). Chunk-level detail lives in the chunks' own PROGRESS logs; this
> file is the thread between them. A reader who was not here should be able to follow — and undo —
> everything from this file plus `git log`.

## Standing rules in force, and the one the team lead relaxed

RESUMING.md's rules 2–8 are unchanged: a brief before an executor; docs in the same commit; prose
review logged; status pointers checked; report then wait (here: report in THIS file, since nobody is
waiting); size chunks to the doc pass; no hardware claim without a log. **Rule 1 ("one thing in
flight") is relaxed for this session by the team lead's explicit words ("open some agents to finish
faster")** — with these safety limits the coordinator set for itself:

- At most ONE executor writes to the main working tree at a time. Any second executor works in an
  isolated git worktree on files disjoint from the first, and its work is integrated by the
  coordinator AFTER the main-tree chunk is verified and committed, with the doc pass redone at
  integration. Nothing is merged unverified.
- Every executor is `opus`, backgrounded, told "do not commit" and "PROGRESS log first" (RESUMING §2).
  No `fable` executor without the team lead's word on a specific run (his standing rule).
- The coordinator verifies every executor's report itself before committing (rebuild, suite, the
  five-build gate, ARM gate, CI guards, doc gates, and a read of the load-bearing code).
- Nothing is pushed. Nothing touches hardware (the robot is not on USB; the team lead does that in
  the morning). Nothing outside this repository is modified.
- Plan changes the team lead has not explicitly approved are taken under his blanket authorization
  ("you have my permission to do what you gotta do"), recorded here with the reasoning, and flagged
  for his review in the morning under **"Decisions taken without you"** below.

## The morning target, and the order it implies

By morning: robot two's NEW drivetrain (rebuilt this week — the chassis table's ports and signs are
now unverified) on the team lead's PC, and **the library's motion stack driving it for the first
time.** The shortest honest path:

1. **R3d Part 1** (the calibrator, host-only) — in flight since 2026-09-17 evening; verify, commit.
2. **R3d Part 2** (the "shulib Calibrate" program, the persisted record, boot verification, "shulib
   Teleop" built from the record with the typed table as fallback) — brief, execute, verify, commit.
   Host-only, build-gated. This is what turns the calibrator into something that can run on the
   brain, and it is what makes the stale table not matter.
3. **The morning runbook** — the worksheet's Station G, written for the team lead to follow alone:
   Bench Tests census first (the rebuilt drivetrain), then Calibrate, then Teleop from the record.
4. If time remains: **T2** (driver feel) in a worktree in parallel with step 2, since it touches
   `teleop/` only and the team lead's "clean main" goal needs it; and **G1's brief**.

Decisions on order beyond the team lead's explicit words are logged below as they are taken.

## Decisions taken without you (read these first in the morning)

*(appended as they happen)*

## Timeline

> **Timestamp correction (written ~15:00, 2026-09-18).** The entries from "~05:35" onward were
> first written with GUESSED clock times (06:10 … 17:45) that ran ahead of the real clock; at
> "17:45" the clock actually read 14:35. They have been re-derived from evidence and are marked
> `~`: the previous commit `76a5819` at 20:44:50 on 2026-09-17 and the Part 1 executor's log born
> at 20:47:49 (its launch), its measured run of 9.02 h (finished ~05:47), the T2 executor's
> measured run of 8.79 h ending at its ~14:35 report (launched ~05:47), the `date` call that read
> 05:34:34 at the end of the Part 2 brief, and the commit `17aa4eb` at 13:55:10. The start of the
> Part 1 verification is not recorded: the report reached the coordinator at a turn boundary
> whose time was not captured, and the commit is the only anchor. Lesson applied from here on:
> every entry takes its time from a `date` call in the same command, never from memory.

- **~05:15** — Session log created. State: tree at `76a5819` (the R3d brief commit) plus the Part 1
  executor's uncommitted work (`include/shulib/calibration/drivetrain_calibrator.hpp`,
  `hal/drivetrain_description.hpp`, the plant extensions, the table converter, three new test files,
  the regenerated `docs/api/`, `R3d-PROGRESS.md` at Step 3). The executor is still writing (the
  calibrator header changed at 05:28:58). No completion report yet.
- **~05:20** — Memory note saved (`autonomous-session-2026-09-18.md` in the coordinator's memory
  directory) so a later session knows this night's authorization was one-off.
- **~05:20–05:34** — Read what Part 2 needs (`main.cpp`'s competition-state functions and the
  library program's boot path, the drive program's tolerant adapter construction, the Makefile's
  PROGRAM block, the src build gate's tables, `apix.h`'s registry declaration, Station F of the
  worksheet) and the Part 1 API as it stands in the working tree (the description's fields, the
  calibrator's public surface, `tableToDescription`). **Wrote the Part 2 brief**
  (`R3d-part2-persistence-boot-programs.md`): the SD read seam, the registry seam, the record
  format with a CRC-32, the pure boot check with its honest limit (two cables swapped with each
  other are invisible to the registry and are caught at the first drive), the absent
  `MotorGroup` member, "shulib Calibrate" as a sixth build, "shulib Teleop" built from the
  record with the table as fallback, ten tests with mutations, Station G as the morning runbook.
  NOT committed yet: it references Part 1's API, which is uncommitted and may still move; it is
  re-checked and committed with (or right after) Part 1.
  **Decision taken without you #1:** the IMU port is DISCOVERED from the device registry rather
  than stated (ruling P1 in the brief) — the parent spec listed it with the typed facts only
  because the table predates the registry seam. Reversible: one ruling, one line of code.
- **~05:35–05:45** — Read the T2 seam (`teleop/stick_mapping.hpp`, its bit-identity pin, the arcade
  arithmetic in `coupled_side_monitor.hpp`, `Chassis::drive`'s frame contract, the controller seam,
  the Phase T entries and the roadmap's T2 rows, HA-112). **Wrote the T2 brief**
  (`T2-input-shaping-drive-modes.md`): continuous deadband, expo curve, slew off by default,
  arcade-body / arcade-field / tank-stick modes, field-centric with a latched fallback and a
  driver re-zero that never touches the IMU or the estimate, one-tick latency pinned; ten tests
  with mutations. Scoped to `include/shulib/teleop/` + tests + three documents; `src/` and the plan
  documents are integrated by the coordinator afterwards (brief §7).
  **Decision taken without you #2:** T2 launches NOW, in an isolated git worktree, in parallel
  with R3d — your "open some agents to finish faster", under the session's limits (disjoint files,
  integrated only after verification). It does not touch `main.cpp`, so it cannot collide with
  R3d Part 2 there.
  **Decision taken without you #3 (design):** field-centric rotates the stick in the shaper and
  always commands `Frame::Body`, rather than using the facade's `Frame::Field` — so the fallback is
  one branch and the boot window needs no special case (brief §2 item 3, rejected alternatives).
- **~05:47** — T2 executor LAUNCHED (opus, background, isolated git worktree from `76a5819`; told:
  no commit, no `src/`, no plan documents, no new HA numbers, PROGRESS log first). Two executors
  now run: R3d Part 1 on the main tree, T2 in its worktree. Disjoint files by construction.
- **~05:50** — Wrote the coordinator's verification script (scratchpad `verify.sh`: host build with
  the doc gates, the suite, the src build gate self-test + check, the ARM header gate, the two CI
  greps verbatim from `ci.yml`, api freshness, the briefing gate, the C7 removability grep, and
  the git state). One command per chunk from here on; its output goes into this log.
- **~05:55** — Read the calibrator's load-bearing phases as they stand (`tickSideAssign`,
  `finishPulse`, `tickSpinTest`, `finishSpin`, `finishDone`, `commandSpin`, `fightDuringSpin`)
  BEFORE the executor's report, so a defect would reach it early. Findings: the pulse applies the
  captured sign exactly once (T1); the side is the yaw's sign with the own-encoder contradiction
  refused by port (T3); the spin accumulates SIGNED yaw per tick and refuses a non-positive or
  disagreeing width (T8); every stop is 0 V + Coast (landmine 7); the rest wait names the port
  (its R15). One minor observation, not a defect: the spin does not check that NON-drive
  candidates stay still (the pulse does); the fight detector covers the sides. No message sent to
  the executor. Waiting for both reports; nothing else is in flight.
- **~05:47 (finished; report read at the next turn — exact time not recorded)** — **R3d Part 1 executor REPORTED** (it ran ~9 hours; 27 mutations, one green hole in
  its own test found and closed; 16 rulings beyond the brief, all in `R3d-PROGRESS.md`; the src
  build gate caught a real `-Wformat-nonliteral` defect the host build cannot see; it also found
  the stale-binary trap: the doc gates run BEFORE the link and the briefing gate runs the OLD
  binary, so a red binary blocks its own relink).
- **(start not recorded) → 13:55 — Coordinator's independent verification of Part 1** (RESUMING §3):
  `verify.sh` on the executor's tree: host build + doc gates PASS; suite **1233 cases /
  1,568,972 assertions, green**; src build gate self-test + check PASS (five builds); ARM gate
  PASS (160 headers); PROS-free and layering guards PASS; api fresh; briefing current; C7
  removability grep empty. Read the diffs of the four pre-existing files the chunk touched
  (`motor_group.hpp`: the median extracted verbatim and shared; `drive_geometry.hpp`: the
  calibrated constructor with the "wheelRadius carries the product" note; `drive_plant.hpp`: the
  two extensions with the exact-compare shortcut that keeps the fan-out path bit-identical;
  `chassis_table.hpp`: the converter and its refusal helper). **Two load-bearing mutations re-run
  by hand:** M6 (swap the yaw-sign → side rule) — 13 cases red on a fresh binary; M12 (width
  formula `dR − dL` → `dR + dL`) — first attempt used a STALE binary (the trap above: the briefing
  gate refused the relink because the M6 binary was red), redone with `--target
  shulib_tests/fast` and the link line confirmed: 12 cases red, the width-agreement and
  scrub-direction checks named. Restored, relinked, green. The coordinator's mutation recipe from
  here on: `cmake --build build/test --target shulib_tests/fast` and require the link line.
  Folded the executor's two brief corrections into the Part 2 brief (the record's field order is
  the description's DECLARATION order with `source` seventh; `describeMissingForDescription()`
  added to §1). Corrected the landing date in the roadmap and the Next block to 2026-09-18, and
  replaced the Next block's "coders start on routines" sentence with the team lead's 2026-09-18
  words (the coders wait; Phase G is the priority after Part 2) plus the parallel T2 note.
- **13:55** — **COMMITTED `17aa4eb`**: R3d Part 1 + the Part 2 brief + the T2 brief + this session
  log, after the full gate run on the exact tree (all green; suite 1233 / 1,568,972). Not pushed.
  Tree clean.
- **~13:57** — **R3d Part 2 executor LAUNCHED** (opus, background, main tree, from `17aa4eb`; told:
  no commit, PROGRESS "Part 2" section first, the stale-binary recipe, `-Wformat-nonliteral`,
  the absent member last and reported partial if it does not land clean, Station G for the
  morning). T2 continues in its worktree. Two executors in flight, disjoint files.
- **~14:35** — **T2 executor REPORTED** from its worktree
  (`.claude/worktrees/agent-aef02a3f7fb230a9d`): five new `teleop/` headers, three test files,
  17 mutations (16 red; one genuinely bit-identical `* 1.0` replaced by two that are red), 12
  rulings beyond the brief, nothing committed. It found the worktree had been handed to it at
  `c778c11` (the release tip, not this branch) and reset it to `76a5819` before starting —
  recorded in its log §0. It also found a slip in the brief's test-5 sketch ("stick UP at
  heading 90° ⇒ body LEFT" — it is RIGHT, by R(−θ)); the coordinator confirmed the derivation
  and corrected the brief's row with a note. HA-112's text widened, no new numbers minted; two
  candidate beliefs left in its log for numbering at integration.
- **14:35–14:50 — Coordinator's verification of T2 IN THE WORKTREE** (integration waits for
  Part 2, which owns the main tree): `verify.sh` on the worktree — all gates PASS; suite **1247 /
  1,563,890 green** (from 1208 / 1,563,156 at `76a5819`); src build gate five builds; ARM gate
  163 headers; guards; api fresh; briefing current; removability empty. Read `field_centric.hpp`
  in full (the rotation is R(−φ) on plain doubles with the wrap; the latch is set on any
  untrusted or non-finite heading and cleared only by `rezero()`) and the deadband/expo bodies
  (closed `<=` deadband returning literal `0.0`; the OFF cases early-return the argument). Two
  mutations re-run by hand on fresh binaries (results in the next entry).
- **~14:55** — T2 hand mutations, both on fresh binaries in the worktree: rotate by +φ instead
  of −φ ⇒ red at the odd angles and against the frozen `fieldToRobot` oracle (6 assertions
  named); drop the fallback latch ⇒ red on every latch assertion (6 named). Restored; the
  worktree's teleop/mode/driver suites green (114,453 assertions). **T2 is VERIFIED and waits
  for integration** behind R3d Part 2 (the same commit re-points `src/` and moves the plan
  documents, brief §7). Fixed an unescaped table cell in the T2 brief (row 3's `|Δ|`).
- **14:44** — Corrected this log's timestamps from evidence (see the note at the top of
  the Timeline). Then, while Part 2 runs: read what a builder must own (the facade's standalone
  promise and constructor, `MotionDeps` and its one cross-check, `RobotContext`, both graphs in
  `main.cpp`, the kinematics presets, the two odometries, the localizer's constructor, the
  adapters' constructors, the longhand standalone test, the master plan's §16.2 config slice and
  §17 tiers) and **drafted the G1 brief** (`G1-robot-config-and-builder.md`): `RobotConfig`
  embedding R3d's `DrivetrainDescription` for a tank, `Route` types with typed marker args, an
  `IDeviceProvider` seam (devices by port, provider-owned) with a fake provider in Part 1 and the
  PROS provider in Part 2, `robot::Robot` owning the PROS-free graph in the longhand order,
  bit-identity to the longhand rigs as the acceptance test, "two lines" as a literal test. Part 2
  of G1 collapses `main.cpp`. UNCOMMITTED until R3d Part 2 lands (its `main.cpp` line references
  are re-checked then).
  **Decision taken without you #4:** G1 is split in two (types + builder + host proof; then the
  PROS provider + `main.cpp` collapse + the guide), so each part's doc pass fits (rule 7).
- **16:13** — **R3d Part 2 executor REPORTED** (ran ~2.0 h): the record format (180
  bytes, CRC-32), the SD read seam, the registry seam (+ the shim's apix.h), the pure boot check
  with the pinned honest limit, `chooseBootSource`, the ABSENT group member, "shulib Calibrate" as
  the SIXTH build, "shulib Teleop" rebuilt on the description with record → table → refuse, the
  worksheet's Station G (~150 lines, every screen string copied from source); 17 mutations (two
  first read green off stale binaries, both re-run red — its log has the recipe); 29 rulings. It
  touched Part 1's calibrator twice, both logged: `accepts(Answer)` so a panel knows which
  buttons are live, and 26 em dashes in RUNTIME strings converted to `--` because LVGL's fonts
  are ASCII-only. It edited `docs/faq.md` (a sentence had become false) and `RESUMING.md`/`ci.yml`
  (five → six builds). Bit-identity: +9 assertions from one rewritten pre-existing case (the old
  "a null member throws" contract), nothing else moved.
  **Coordinator's verification:** `verify.sh` on the tree — ALL PASS; suite **1267 / 1,624,433**;
  src build gate SIX builds; ARM 168 headers; guards; api fresh; briefing; removability empty.
  Read: the boot check's verdict logic (Swapped ⇒ Refuse; IMU wrong ⇒ Refuse; policy ⇒
  Refuse/Degraded), the record's eight refusal points and the CRC, `chooseBootSource` (record
  preferred, always), the calibrate program's Done path (write → flush → close → re-open → read
  back → memcmp → decode → SAVED), `main.cpp`'s boot source and the description-built graph,
  the group's nullptr handling. **Hand mutations, fresh binaries:** Swapped treated as Missing
  ⇒ 5 assertions red naming the swapped ports; the CRC comparison skipped ⇒ the CRC-mismatch
  refusals red. Restored (diff-confirmed), green.
  **Decision taken without you #5:** committing Part 2 on the earlier full gate run plus the
  diff-confirmed restore, without a second multi-minute full run — the team lead is at the robot
  waiting for the packages; the content is byte-identical to what passed.
  Team lead is awake (~14:50 onward) and at the robot: his brain is not enumerating on USB (no
  VEX device in `lsusb`, no `ttyACM`) — cable/power, being sorted on his side. Bench Tests was
  built for him from the committed tree in a separate checkout (`.claude/worktrees/hw-17aa4eb`)
  so the executor's in-progress edits could not reach a package.
- **16:14** — **COMMITTED `5e4ce65`** (R3d Part 2 + the G1 brief + this log). The
  hardware checkout moved to it and all four packages were built there, beacons confirmed:
  Bench Tests (tank), shulib Drive (tank-drive), shulib Teleop (tank-library), shulib Calibrate
  (tank-calibrate), saved under /tmp/hw_pkgs. The brain still does not enumerate; a
  seven-minute USB watch is running at the team lead's request while he power-cycles it.
- **16:16** — **T2 integrated into the main tree** (library half): ten files copied from
  its worktree; the two shared documents three-way merged against their `76a5819` base — both
  clean; api docs, mkdocs and the briefing regenerated; the roadmap's two Phase T rows → `[~]`
  with what is and is not done; the build-order T2 entry's status line and the Next block's T2
  sentence. **Decision taken without you #6:** the `src/` re-pointing (the teleop loop, the drive
  program, the tester's DRIVE station → `DriveShaper`) is DEFERRED past the bench session, so the
  four packages built for it keep exactly the feel you drove on 2026-09-17; it is the next small
  commit after the robot has run. **Decision #7:** T2's two candidate register entries (a
  continuous deadband is preferable; a driver prefers one frame over a silent return) are NOT
  minted as HA numbers — they are driver-preference claims, already stated in HA-112's widened
  entry; the register stays a register of hardware beliefs.
- **16:24** — T2-integrated tree: `verify.sh` ALL PASS after the briefing was regenerated
  for the new count — suite **1306 / 1,625,161 green** (T2 adds 39 cases); six builds; ARM;
  guards; api fresh; removability empty. Committing T2's library half. The team lead's brain
  has still not enumerated on any port (no kernel USB event at all); he has unplugged it for now.
- **16:43** — **COMMITTED `d8aab95`** (T2's library half) and **`280dce8`** (the G1
  brief's `main.cpp` line references re-checked against the Part 2 tree). Tree clean. The
  seven-minute USB watch ended with no brain ever seen; the team lead has unplugged it — the
  cable/port question is open on his side; all four packages stay built under /tmp/hw_pkgs and
  in the hardware checkout at `5e4ce65` (T2 and G1 do not change any package: `src/` untouched).
- **16:43** — **G1 Part 1 executor LAUNCHED** (opus, background, main tree from
  `280dce8`; told: no commit, NO `src/`/Makefile/gate/worksheet changes because the hardware
  session depends on them, PROGRESS log first, the stale-binary recipe). It is the no-code
  foundation the team lead asked for first: `RobotConfig` embedding the drivetrain description,
  `Route` types, the device-provider seam with a fake provider, `robot::Robot` owning the graph,
  bit-identity to the longhand rigs as the acceptance test.
- **18:52** — **G1 Part 1 executor REPORTED** (ran ~2.3 h): `config/robot_config.hpp`,
  `config/route.hpp`, `hal/device_provider.hpp`, `hal/fake/fake_device_provider.hpp`,
  `robot/robot.hpp`; 36 new cases; 15 mutations, FOUR first-run greens each closed by a
  stronger test (a turnTo leg so swapped groups show; a 4.0 in fixture wheel so the derived
  stall geometry is not the default; a frozen member so an empty group span shows; the signed
  port asserted after both passes); 13 rulings — the one that matters: no `build()` factory (a
  non-movable `Robot` cannot be returned by value; the brief's own B3 said so), so the two lines
  are `validate()` + `Robot{cfg, devices}`. Pre-existing cases moved by exactly +15 assertions,
  all from the CI-guard mirror test (3 per new header × 5), arithmetically verified. `src/`
  byte-identical to `280dce8`. One edit outside the touch list, flagged: a table cell in
  `docs/vexbuilder-integration.md` that had become false (accepted — it is a fact). No HA
  numbers minted. **Coordinator's verification in progress:** `verify.sh` running; the
  constructor read against `TankRobot` — same objects, same order (config → kinematics → motors
  → groups → sensors → latch → monitor → geometry → odometry → fusion → corrector → localizer →
  context → deps → chassis); two hand mutations queued behind the gate run (they share the
  build directory).
- **19:02** — G1 Part 1 verification complete: `verify.sh` ALL PASS — suite **1342 /
  1,632,353 green**; six builds; ARM 178 headers; guards; api fresh; briefing; removability empty.
  Hand mutations on fresh binaries: the kinematics' wheel order handed right,left ⇒ the
  bit-identity REQUIREs fail (exit reasons differ) and the healthy group raises a disagreement;
  the GPS corrector never installed ⇒ tick counts diverge and `lastCorrection().source` is not
  "gps". Restored, relinked, green. Committing.
- **19:08** — **COMMITTED `fe8e270`** (G1 Part 1). Read what G2 needs (the recipe's
  `driveTo` arithmetic, the facade's verbs, the F10 pin test's shape, the fault/telemetry APIs,
  the integration document's §4 schema and its open `NONE` question, the legacy vocabulary) and
  **wrote the G2 brief** (`G2-path-runner-and-registry.md`): the runner over the facade with D1's
  error policy, markers on Settled only, a registry with no `std::function`, four library-owned
  built-in ids, the manifest as line-JSON through `ICharSink`, the F8 freeze (in-memory types +
  file schema reconciled — `"arc"` dropped as a named gap, `"locked"`→`"fixed"`, `"tangent"`→drive,
  args as a positional int array, `NONE` is route structure). **Decision taken without you #8:**
  G2 runs now, before G1 Part 2, because G1 Part 2 rewrites `main.cpp` and the bench session's
  packages depend on `src/` staying as committed; G2 touches no `src/`. Launching its executor.
- **19:39 — SESSION CLOSE-OUT (the team lead is rebooting into another OS).** The G2 executor was
  STOPPED mid-chunk by the coordinator (its last step: the `version.hpp` bump and test 13); its
  files are left UNCOMMITTED and UNVERIFIED in the working tree with an interruption note at the
  end of `G2-PROGRESS.md` that says exactly how to resume or discard. So the next session's
  `git status` is DIRTY ON PURPOSE — read `G2-PROGRESS.md` first, per RESUMING. Committed and
  verified tonight, in order: `17aa4eb` R3d Part 1, `5e4ce65` R3d Part 2, `d8aab95` T2 (library
  half), `280dce8` and `e9ba69f` (briefs), `fe8e270` G1 Part 1. Nothing pushed.
  **Hardware:** the brain never enumerated on USB today (no kernel event on any port — cable or a
  laptop USB controller that reset with an error at 04:54); nothing has run on the robot. The
  four packages built from `5e4ce65` live in the hardware checkout
  `.claude/worktrees/hw-17aa4eb` (detached at `5e4ce65`; `/tmp/hw_pkgs` is lost on reboot —
  rebuild with `rm -rf bin .d && make ROBOT=tank [PROGRAM=…]` there or in any checkout of
  `5e4ce65`, one program at a time, then `pros upload --slot N --name "…"`; Station G in the
  worksheet is the runbook). The T2 agent worktree (`.claude/worktrees/agent-aef02a3f7fb230a9d`)
  is integrated and can be removed with `git worktree remove --force`.
  **Next work, in order:** finish G2 (resume per its log) → G1 Part 2 (the PROS device provider
  and the `main.cpp` collapse) + T2's `src/` re-pointing → G3's brief → the bench session whenever
  the brain connects (it needs nothing from any of these).

## Resumed — 2026-09-18 20:09 (the team lead: "you can continue")

- **20:09** — "Where things stand" run: HEAD `d9d0ee3`, the tree dirty with exactly the
  seven G2 paths the close-out recorded, both side worktrees present, no brain on USB. The
  coordinator's gate script did not survive the reboot and was recreated in the scratchpad.
  **G2 RESUMED**: a fresh executor (opus, background, main tree) told to read the interrupted
  log first, treat every earlier claim as unverified, keep what is correct, finish the brief's
  DoD, and re-run EVERY mutation against the final code. One thing in flight; the standing rules
  are back in force (the overnight relaxation of rule 1 ended with that session).
- **21:30** — **G2 (resumed) REPORTED and VERIFIED.** The second executor found the first
  one's three headers and four tests complete and sound, and three defects its never-reached
  first build exposed: `version.hpp` unbumped (the F8 pin's own case red), `Route::valid()`
  refusing the very combination ruling R6 defines (`MoveToPose` + `Free`), and a tank test that
  asserted what the library deliberately does not do (a tank cannot `DriveTo` a point far off its
  heading without an authored turn — now the test pins that finding both ways). 14 mutations,
  all red on the final code, after it caught a stale-object false green (an `inline constexpr`
  table: touch EVERY including test before a fast relink). Coordinator: `verify.sh` ALL PASS —
  suite **1358 / 1,635,290**; six builds; ARM 181 headers; guards; api; briefing; removability.
  Read `run()`'s gate order, the stop path, the unknown-id path and `bearingTo`. Hand mutations
  on fresh binaries: `atan2` arguments swapped ⇒ the bit-identity REQUIRE and the reverse-heading
  checks red; markers moved ahead of the Settled gate ⇒ "fired == 0" red. Restored, green.
  **Ruling accepted (R6-SPLIT):** `valid()` now permits `MoveToPose` + `Free` and still refuses
  `Free` on `TurnTo` — it is the brief's own R6; it rewrites G1's test 2h; relaxing at the freeze
  is the additive direction. Also accepted: the manifest lines carry `minArgs`/`maxArgs`/`doc`
  (one number cannot describe `log`'s 0–2 args), and a surgical edit to
  `legacy-command-vocabulary.md` (three sentences said G2 "must decide" what G2 has now frozen).
  **Flagged for the team lead:** the master plan still spells the binding `runner.on(...)`; the
  code is `registry.on(...)`. One line, his document.
