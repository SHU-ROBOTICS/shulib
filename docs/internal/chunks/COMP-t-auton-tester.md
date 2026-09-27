# COMP-t — "shulib Auton Tester": run one of the team's autons on the bench, from RUN, through exactly the path `autonomous()` uses

> **THE TEAM LEAD'S ANSWERS (2026-09-27, ~04:55).** **D1–D9: all as recommended** ("Accept all") —
> `PROGRAM=autontest`; endActionAt 27 s / hardStopAt 29 s; the robot's configured speeds with a wheels-up
> run first; no driving between runs; all four abort edges; one appended SD file through the adapter's
> Append mode; the guide quotes the live students' file; one executor run; RUN only when the boot used the
> calibration record. **D10 is SUPERSEDED:** asked how the 40 in auton should finish on a tank that drifts
> sideways, he chose **"Fix the library first"** — chunk **TANK1** (`moveTo` on a tank steers back onto its
> line: bounded, never a planned turn, logged whenever it steers, the X-drive and H-drive byte-for-byte
> unchanged, the auton's `moveTo` line unchanged). **COMP-t is built AFTER TANK1 lands.** Before its
> builder starts, the coordinator re-makes every prediction below that says ROUTINE STOPPED / MOTION_TIMEOUT
> for a weak side (D3, D10, §0.2, M.5/M.6/M.8, tests (3b)/(3c)/(4b), the students' file's tank comment,
> guide 16) from TANK1's own tests, and dates the change here. **Slot:** 7 until T4 Part 3, then 2 (T4 N4).
> **Tree:** SRC1 is committed (`1237c91`); the flat-layout fallback is moot. Which of COMP-t and COMP-b
> lands second takes the next build number and self-test pair.

> **Execution brief (DRAFT for the team lead's go).** Written 2026-09-27 (~02:00–03:30 EDT) from a read-only
> pass over `shulib-v2` at `37706ce` (G1 complete) plus the SRC1 brief (`0fab866`) and T4's amended brief,
> and **probed**: the students' file and the tester's whole flow were compiled for the V5 (every file a real
> `-c` compile under the `src/` warning set, not `-fsyntax-only`), run on the host test plant, and built as a
> **seventh build in a scratch copy of the tree where the real `tools/src_build_gate.py check` PASSED all
> seven builds** (§0.4). **Re-checked 2026-09-27 (~04:00–05:30) against a checker's thirteen findings** (see
> "Re-check 2026-09-27 — amendments" below): every amended block was re-probed — compiled for the V5,
> run on the host test plant, and the seven builds re-run through the real gate (§0.4, probes 4–6).
> **Status: WAITS for the team lead's go and his answers to D1–D10. Nothing is built.**
>
> **The team lead's request (2026-09-27 ~01:40):** "can [you] make it so that there is a seperate auton
> program so that it doesnt need to be in comp mode to run it?" — build it NOW, beside T4 Part 1a; "Touch RUN,
> 3 s countdown, any button aborts"; logged to serial and the SD card; final slot 2 (T4's R6).
>
> **Runs in an isolated worktree** (`.claude/worktrees/comp-t`, branch `comp-t`, from the tip at launch).
> **SRC1 MUST have merged first — a hard precondition, not a preference.** The team lead ruled SRC1 lands
> "before the new code is built". At this re-check SRC1 is NOT merged: HEAD is `0fab866` (its brief only);
> its executor's change sits STAGED and uncommitted in the main tree, under verification (seen ~05:30). That
> staged change is what these paths assume: `src/{robots,team,programs,common}/`, the `src/` include root
> (`EXTRA_INCDIR:=$(SRCDIR)` ⇒ `-iquote"./src"` after `./include`; `-iquote…/src` for the host suite), and a
> new gate step 0 `[include-root]` rule that fails any `src/` include not written from `src/` and any `src/`
> path that also exists under `include/`, the shim, `test/vendor/` or `test/`. Every NEW file below is in that
> layout and every include is written from `src/` (`"team/autons.hpp"`, `"programs/…"`, `"common/…"`); none
> of the new paths collides with a file under those roots (checked). **If SRC1 is not COMMITTED at launch,
> stop and tell the coordinator.** (The first draft's "drop the folder — nothing else changes"
> fallback is WITHDRAWN: the includes, the host tests, `test/CMakeLists.txt`'s `target_sources` path, D7's
> path and §6.11 would all differ.) Existing files are cited at their `37706ce` flat path and ~line, with
> SRC1's folder in brackets; SRC1's diff moves lines in `main.cpp`, `build_selection.hpp`, the Makefile, the
> gate and `test/CMakeLists.txt`, so find each cited line there by its QUOTED text, not its number. **In parallel with T4 Part 1a:** do NOT touch `src/team/robot_two_driver.hpp`,
> `src/common/driver_controls.hpp`, `src/programs/drive_program.*`, `src/programs/teleop_program.*`,
> `src/programs/teleop_claws.hpp`, `src/robots/mechanism_table.hpp` or `include/shulib/teleop/`. **Three
> files BOTH chunks edit, in different places — whoever lands second re-reads and merges:**
> `tools/api_doc_tool.py` (the example-source list — ONE shape, §3.7.4, which T4 Part 1a item 6 also
> uses), `test/CMakeLists.txt` (`SHULIB_DOC_INPUTS` :172) and `src/boot_source.hpp` [`common/`] (T4 Part 1a
> item 7 rewords :92-94; COMP-t rewords :42-44 and :259-262). **One executor run (D8). Executor model:
> `opus`. Do not commit.** Live log:
> `docs/internal/chunks/COMP-t-PROGRESS.md`, **created FIRST** and appended as you go. **Hardware Assumptions
> numbers: HA-177 and HA-178 ONLY** (§2 R11); a third belief ⇒ stop and tell the coordinator. The team lead
> authorized the coordinator to write the auton ("no you can write the auton", 2026-09-26): the students' file
> (§3.1) and the tester's flow (§3.5) are given VERBATIM and the executor writes them as shown — any change the
> tree forces is logged with its reason, and every document quotes the RESULT, never this proposal. The probe
> copies of every new file are in `.claude/coordinator/scratch/brief-COMP-t/src1sim2/` — the re-check's
> AMENDED copies; start from them (the panel's is a sketch, §3.4). `src1sim/` is the first probe, kept as
> evidence. The library headers the re-check changed are in `…/overlay/shulib/`
> (`sequence/run_guard.hpp`, `hal/pros/block_sink.hpp`, `hal/block_char_sink.hpp`).

---

## Re-check 2026-09-27 — amendments

A checker returned thirteen findings on the draft. Each was re-verified in the tree at `37706ce`/`0fab866`,
most were re-probed (§0.4 probes 4–6), and each disposition below says where the body now carries the fix.
**Twelve are real and fixed at the root; one (1) rests on a premise that is false today, and its latent
hazard is fixed anyway.** D1–D8 are unchanged except D3 (a cross-reference) and D6 (the SD log goes through
the adapter); **D9 and D10 are new.**

1. **(major) "A typed-table boot reverses forward" — premise FALSE today; the latent hazard is REAL, fixed.**
   - **Verified false today.** Robot two's typed table cannot describe a drivetrain: track width and ratio
     are 0 (`chassis_table.hpp` :135-136), pinned by `test/chassis_table_test.cpp` :597-603 ("track width,
     external ratio (motor->wheel)"). So `tableValid` is false (`boot_source.hpp` :486) and
     `chooseBootSource` REFUSES with no record (`calibration/boot_check.hpp` :437-444) — worksheet F.0
     :529-541 says the same ("No record on the card means refused"). Neither "no card" nor "a record lost to NOT
     SAVED" reaches `UseTable` on robot two.
   - **Real, latent.** The day HA-125/126 are typed into the table, `UseTable` becomes reachable, and its
     front is the 15/16 end, the opposite of the record's (worksheet :514-521). And even on the RECORD,
     "forward" is whichever end LED the Calibrate push, so a rerun pushed from the other end flips it.
   - **Fixed (D9, R4, §3.4, M.3, M.6, the students' file).** READY shows the boot source on its own line
     (`BOOT RECORD: forward = the end that led Calibrate`, or red `BOOT TYPED TABLE: its front may be the
     OTHER end`). RUN is REFUSED unless the boot chose the RECORD (D9, recommended). M.3's push test becomes
     a STOP condition AND the definition of forward for M.6: the robot is placed with the end that made EST x
     RISE in M.3 leading. The students' file's frame comment says what "ahead" means. The guide says it too.
2. **(major) Collision with T4 Part 1a item 6 on `api_doc_tool` — REAL, fixed.**
   - **Verified.** T4 Part 1a item 6 (T4 brief, § Scope, Part 1a) widens `check-examples` to
     `src/*_driver.hpp`, calls it "the one kind of `src/` file a public document quotes whole" and keeps "a
     glob, not a list". This draft turned `EXAMPLE_SOURCE_GLOB` (`api_doc_tool.py` :1811, used :1868,
     reasoned :47) into a list and had the guide say "the one exception". Both edit the same lines, both add
     self-test cases, both touch `SHULIB_DOC_INPUTS` (`test/CMakeLists.txt` :172). Both also edit
     `boot_source.hpp` (T4 :92-94; COMP-t :42-44, :259-262).
   - **Fixed (header, §3.7.4, §6).** ONE shape, now: `EXAMPLE_SOURCE_GLOBS`, a list of GLOBS, each naming one
     KIND of file a public document may quote whole, each required to match at least one file:
     `test/*example*_test.cpp`, `src/team/autons.cpp` (COMP-t), `src/team/*_driver.hpp` (T4). Whichever
     chunk lands first creates the list with its own entry; the second appends. The phrase "the one/only
     exception" is banned (a prose-review grep); the guide README says "the team's files under `src/team/`
     that a chapter quotes (today: …), never the rest of `src/`", and the second chunk extends the list in
     the sentence. The shared files are named in the header. The coordinator carries the list shape into
     T4's item 6 (§6's callers gained).
3. **(major) The 40 in auton's likely floor result is not predicted — REAL, PROBED, fixed (D10).**
   - **Verified.** `MoveToPose` settles only when `hypot(ex, ey)` ≤ 0.5 in AND the heading is within
     0.02 rad (`move_to_pose.hpp` :176-177; `motion_config.hpp` :96-100). On a tank body `vy` is clamped to
     0 (`kinematics/tank.hpp` :76, `strafeAuthority()` 0), so a lateral error cannot be driven out: the
     facade says so ("On tank, laterally-offset targets honestly exit TimedOut", `chassis.hpp` :98). The
     heading is held by P only (kP 4, `motion_config.hpp` :83).
   - **Probed (§0.4 probe 4):** the 40 in on the host test plant with the right side's volts scaled:
     ×1.00 FINISHED at (39.77, 0.00); **×0.97 ROUTINE STOPPED at moveTo, MOTION_TIMEOUT, END y −0.93 in;
     ×0.95 y −1.56; ×0.91 y −2.86** — scoring ended at the 5 s watchdog each time, and the brake end action
     still succeeded. The acceptance run stops at step 1 from ×0.95. Robot two's sides differ by ~9 %
     (`R3d-PROGRESS.md` :2514). The plant has no inertia, so these numbers are a shape, not a prediction
     of inches — but the SHAPE is clear.
   - **Fixed (D10, D3, §0.2, M.5/M.6/M.8, tests (3b)/(3c)/(4b), the students' file, guide 16).** The
     expected first floor result is stated: **ROUTINE STOPPED at moveTo, MOTION_TIMEOUT, END |y| over
     0.5 in, the brake still run — a known tank outcome, not a tester bug.** M.6 records the result word, END
     y and the faults line. The students' file says it in one comment. The verb stays a decision (D10):
     no verb in the library settles a tank after a lateral drift, and the number is never fudged.
4. **(major) The PROS-side abort wiring was neither host-tested nor pinned — REAL, fixed at the root.**
   - **Verified.** `ProsAbortInputs::read()` lived in the program TU (probe :87-99). Tests (6) and (19) drive
     `abortCauseOf` and a SCRIPTED `IAbortInputs`, so "the release count read instead", applied where it would
     happen, survived them; so did a hard-coded `connected = true`, a dropped `E_TOUCH_ERROR` mapping and a
     dropped `competition_is_disabled()`. Mutation 3's attribution was wrong. M.4 never turned the controller
     OFF during a countdown or a run. And the pins proved `opts.pacer = &*st.guard;` exists, not that the
     Robot is built WITH `opts`.
   - **Fixed at the root (§3.2, §3.3, §3.5, R13, §5).**
     - The PROS struct is reduced by ONE host-tested template, `testerTouchFrom(t, E_TOUCH_ERROR)` (it reads
       `press_count`, `release_count`, `touch_status`, `x`, `y` by name, so the host runs it on a
       PROS-shaped struct), and `abortInputsFrom(touch, master, disabled)` builds the inputs through
       `IController`. `ProsAbortInputs::read()` is one line calling them; the program's `readTouch()` is one
       line; both are pinned exactly. Test (6b) kills the press/release swap, the hard-coded connection and
       the dropped error mapping; the disabled read is a source pin (the only guard it can have).
     - **THE CHAIN PROOF (R13), a runtime check on every boot:** one idle tick must pace BOTH the guard
       (a new additive `RunGuard::pacesTotal()`) and the watch (`AbortWatch::paces()`), or the boot is
       REFUSED. A Robot built without `opts` (the checker's mutation) paces neither; one built on the watch
       directly paces the watch only — each refused, host-tested in (13b). The exact
       `st.robot.emplace(st.boot.built.config, *st.devices, opts)` and `robot::validate(…, opts)` are pinned
       too.
     - M.4 gains two wheels-up steps: the controller switched OFF in the countdown, then mid-run ⇒ ABORTED
       `controller lost`. HA-178 gains (d), the disconnect's latency.
     - Mutation 3 is split (3a/3b) with the right tests; mutations 21–27 name the rest.
5. **(minor) Route autons diverge (the tester passes no command registry) — REAL, fixed.** §0.1's table
   gains the row "commands: nullptr — a route auton is REFUSED by name" (`match_runner.hpp` :156-159); a
   route entry's REFUSED line is a golden in (23); **B30** says the command registry is registered ONCE
   beside `registerAutons` in the students' file when the first route auton appears, and both programs pass
   it.
6. **(minor) The tester's task had no top-level catch — REAL, fixed (R14, §3.3, §3.5).** `runTesterOnce`'s
   ONE try now covers the arm, the countdown and the run (a throw in the countdown's `chassis.wait` escaped
   it before, leaving the watch armed); its handler panic-stops the drive. The task's whole body is inside a
   try: a throw anywhere else (the idle `chassis.wait`, the painter, `touch()`, a formatter) calls
   `stopTheTester()` — the PROS-free, host-tested `stopAfterProgramError()` (the drive to 0 V + Brake, every
   PRESENT mechanism to its safe state, the watch disarmed, never throws), both logs, the refusal screen —
   and the task then idles on `pros::delay` forever, never touching the Robot again. Probed (probe 5: a
   pacer layer that throws 0.5 s into the countdown ⇒ `threw`, watch disarmed, travel 0, PROGRAM ERROR).
   Landmine 8 is reworded.
7. **(minor) The "VERBATIM" `runTesterOnce` omitted the fault reads — REAL, fixed.** The verbatim block now
   reads `faultCount()` at RUN and at the end, and `lastFault()` ONLY when the count rose (a fault latched
   before RUN is not this run's — test (3c)). The fields are in `TesterRunRecord` verbatim. Probed: the
   ×0.95 run reports "faults this run: 1, last MOTION_TIMEOUT".
8. **(minor) Test (14)'s bound contradicted the probe — REAL, fixed.** The loop ends on the first tick whose
   elapsed time is ≥ 3.0 s, and floating accumulation makes that tick 301 on the plant (measured 3.0100 s).
   (14) is now `3.0 ≤ runStarted − countdownStarted ≤ 3.0 + one tick + 1e-9`, with that reason.
9. **(minor) §7's "CI greps" do not cover `src/programs/*.hpp` — REAL, fixed.** Verified: the CI guard greps
   `include/shulib` only (`ci.yml` :48-55), and the shim provides `pros/rtos.hpp` and `pros/misc.hpp`, so a
   pure header including either would still compile on the host. The claim is dropped; a source pin checks
   that the three pure headers and `src/team/autons.{hpp,cpp}` include no `pros/` header.
10. **(minor) SRC1's status overstated, the flat fallback incomplete — REAL, fixed.** Verified: HEAD
    `0fab866` adds only SRC1's brief; by the end of this re-check SRC1's change is STAGED, uncommitted, in the
    main tree. SRC1's COMMIT is now a HARD precondition (header); the fallback is withdrawn, with the reason.
    The staged change was read for what COMP-t depends on: the `-iquote` `src/` root in both builds, and its
    new gate step 0 `[include-root]` rule, which every new COMP-t include already satisfies.
11. **(minor) Stale counts and cites missing — REAL, fixed.** §3.6 now lists `main.cpp` :34 ("the four tank
    builds") and the `opcontrol()` doc :227-234; `build_selection.hpp` :109-110 (the `…=tank` prefix list
    gains `…=tank-autontest`) and :141; the Makefile's :121-127 ("`drive`, `library` and `calibrate`", "its
    fourth, fifth and sixth entries"). "four tank builds" and "fourth, fifth and sixth" join the prose-review
    greps. T4 is cited by heading, never by line (its brief is 1956 lines and still moving).
12. **(minor) The SD log bypassed the adapter that owns `/usd/` — REAL, and WORSE than reported; fixed (D6).**
    - **Verified, plus a collision the checker did not name:** SAFE-SAVE's platform scan (SAFE-SAVE §2, "The
      platform scan", ~:813-820) allows `fopen` ONLY in `hal/pros/block_sink.hpp`, `hal/pros/block_source.hpp`
      and `src/bench_r3a.cpp` — the draft's `std::fopen` in the program TU would have failed it whichever
      chunk landed second. And SAFE-SAVE's disassembly (its §0.2, ~:337-345) shows PROS sends an `O_APPEND` open to
      VEXos's `vexFileOpenWrite`; whether that CREATES an absent file is VEXos's, so the draft's
      "creating it when absent" was a belief it never handled.
    - **Fixed (D6 (a), R9, §3.7.6, R11's HA-177).** `ProsBlockSink` gains an additive
      `ProsFileMode::Append` constructor; the append open is decided in ONE host-tested function in that
      adapter, `openForAppend()`: probe with `"rb"`, open `"ab"`, create with `"wb"` ONLY when the file was
      absent — an existing log is never truncated. `hal::BlockCharSink` (T4 N5 (a)'s piece, specified there,
      built first here) turns it into the `ICharSink` the second `TermSink` needs. The program names no
      `fopen` and no `/usd/` (pinned). The adapter file is already on SAFE-SAVE's allow-list; its modes
      carry no `+`. Probed (probe 5): the four branches with a scripted opener, and two "boots" through the
      shim on a real temp directory keep both lines in order.
13. **(minor) Screen timing on RUN and RESULT — REAL, both fixed.**
    - **RUN.** A page change forces a FULL repaint, and it landed in the run's first commanded tick.
      `runTesterOnce` now spends ONE QUIET TICK after the countdown: `runStarted()` flips the page, one
      `Chassis::wait` paints it on a tick that commands nothing (and reads the abort once more), and only
      then runAuton. The scheduler resets its loop monitor at the motion's start
      (`motion_scheduler.hpp` :704-705), so the paint cannot show up as the run's `LOOP_OVERRUN`. Cost: the
      hand-off is 0.02 s instead of 0.01 s (probed).
    - **RESULT.** A touch held through an abort would lift on RESULT's armed buttons after 0.5 s. Now the
      program reads whether a finger is down (`press_count > release_count` on a good read — HA-178 (a)'s
      belief) and `showResult(rec, releaseCount, fingerDown)` holds the buttons grey until that finger's
      release has been consumed, then starts the 0.5 s arming clock. Probed (probe 5). If HA-178 (a) is wrong
      the flag reads false and the page falls back to today's 0.5 s rule. M.4 also tells the operator to
      touch the TOP half of the screen, which has no buttons on any page.
    - **Found while probing it (the draft's own defect):** READY lists 8 SMALL lines, but the probe's
      `TesterView` held 7 and its `addLine` silently DROPPED the 8th (the faults/BUILD line). The view now
      holds 8, `addLine`'s cap is `std::size(v.lines)`, and (24) fails on a dropped line (mutation 33).

**Found while re-checking (not in the checker's list; each fixed in the body):**
- **Mutation 7 SURVIVED test (15) in the draft.** Without the countdown loop's cause check, the post-loop
  check still refuses the run, so travel stays 0 and (15) stayed green; only the countdown's LENGTH shows it.
  (15) now bounds `runTesterOnce`'s return to one tick after the latch.
- **A precision exposed a real overflow.** Giving the ABSENT line's mechanism name its `%.23s` (the brief's own
  every-`%s`-a-precision rule) let GCC compute that line's real worst case, 11 + 23 + 45 + 140 = 219 > 199,
  and flag it; now `%.120s`. The stop line had the same class (211 bytes; now `%.100s`). Both caught by the V5
  compile, zero warnings after.
- **The stack statement was false.** The draft said large objects never sit on the task's stack; the painter
  holds two `TesterView`s (1216 B each) and `runOne` a `TesterRunLines` (804 B). §3.5 now states it (~4 KB of
  32 KiB).

---

## Decisions for the team lead (a recommendation for each, and the real cost of every option)

**D1 — The build switch's name. RECOMMENDED `PROGRAM=autontest`.** Define `SHULIB_PROGRAM_AUTONTEST`,
enumerator `Program::AutonTester`, variant beacon `shulib-robot-variant=tank-autontest`, serial tag `ATS`,
entry `autonTesterInitialize`, upload name "shulib Auton Tester". `make ROBOT=tank PROGRAM=autontest` reads
as what it is.
- **`PROGRAM=tester` is a TRAP, not an option.** It already exists: it is the Makefile's DEFAULT and means
  "Bench Tests" (`Makefile` :128-130, `PROGRAM?=tester`). And `testerInitialize`/`runR3a` are Bench Tests'
  names (`src_build_gate.py` :389), so the codebase's word "the tester" is Bench Tests. That is why no name
  below says just "tester".
- (b) `PROGRAM=auton`. **Cost:** it reads as "the auton program" — the competition program's job. A student
  who types it to build a match program gets the bench tool, and it uploads cleanly.
- (c) `PROGRAM=autontester`. **Cost:** none but length.

**D2 — The 40 in auton's schedule. RECOMMENDED `endActionAt = 27 s`, `hardStopAt = 29 s`** (kind Match, the
30 s period). The acceptance run is registered on the same numbers.
- **Why these.** The period is the VEX U match's 30 s autonomous (the COMP brief's C6, from the manual).
  - The hard stop sits 1 s inside it: "a hard stop that lands on [the period] is a hard stop that never had a
    tick to act" (`auton_registry.hpp` ~:40-43). The 1 s also covers the hand-off latency between the field's
    start and the guard arming (logged).
  - The 2 s between them is the end action's runway. The end action is `chassis.brake()`, 0 V under Brake
    until the estimate says still (`brakeSettle`: 1.2 in/s held 0.1 s,
    `motion_config.hpp` :107-108). It is expected well under 1 s, which is unmeasured. Its own watchdog is 5 s
    (`motion_config.hpp` :112), so a longer brake is cut by the hard stop, as designed.
  - **On the bench the schedule costs nothing.** The one moveTo is bounded by its own 5 s watchdog, and the
    brake by its 5 s: a run lasts a few seconds (1.8 s to settle on the host test plant), never 29.
  - The students will copy this entry as the pattern for real match autons, so it should be a real match
    schedule.
- (b) A short "test" schedule (e.g. 8 / 10 s). **Cost:** a pattern that throws away 20 s of the period if
  copied; it changes nothing on the bench (the watchdogs end the run first).
- (c) A thinner margin (28 / 29.5). **Cost:** 0.5 s against the field's disable, and a hand-off latency that
  has never been measured.

**D3 — The FIRST floor run's speed. RECOMMENDED (a): the robot's configured defaults, as written, with a
wheels-up run first** (Station M.5, then M.6).
- The one moveTo uses the library's PROVISIONAL defaults (HA-50): up to 60 in/s (`motion_config.hpp` :88),
  translation kP 3, settled within 0.5 in (:96-97). Robot two's free speed is about 86 in/s, if
  the reported 600 rpm cartridges and 2.75 in wheels are direct (HA-124/126). **This is the
  library's first closed-loop move on a real robot.**
- **(a)** As written. The run shows what every auton will do until R5 measures real gains. The risks are
  overshoot and wheel slip at the start (slip makes it stop SHORT: the encoders count turns the floor did not
  give). The wheels-up run, 3 m clear and the abort cover them. **Cost:** the first floor run is at full
  provisional speed.
- **Whatever the speed, the likely first floor RESULT is `ROUTINE STOPPED` (MOTION_TIMEOUT)** — a tank
  cannot drive out a sideways drift (D10). A cap (b) may make the drift smaller; it does not remove it.
- **(b)** Cap it for the first runs: `r.moveTo(math::Pose2d{40_in, 0_in, 0_deg}, {.maxLinearSpeed =
  units::Velocity{24.0}});` (there is no velocity literal). **Cost:** a line the team deletes later, and the
  first runs do not show the defaults.

**D4 — Driving between runs. RECOMMENDED no, for COMP-t.** Put the robot back on the start mark by hand.
- After a run the drive rests in BRAKE (the guard's safe state, 0 V + Brake at the motor since DEFECTS2).
  Push it slowly, or lift it; the station says so.
- Driving needs a driver file. T4 is building exactly that (Part 1a) in parallel; using Teleop's bindings now
  would bind a file T4 retires. **Revisit after T4 Part 1a:** a DRIVE page that runs T4's `run()` until a
  button, one call.
- **Cost of yes now:** a collision with T4's in-flight files, and a second driver-control path to verify.

**D5 — The abort's edges, beyond "any button". RECOMMENDED all four:**
1. **The controller disconnecting aborts** (countdown or run). With no controller the abort has no hands.
2. **RUN is refused without a connected controller.** The abort must exist before anything moves. The READY
   page says `NO CONTROLLER: RUN needs it (the abort)`.
3. **The robot being disabled aborts** (a field, a switch, a Timed Run). VEXos cuts the motors anyway; the
   abandon makes the run END cleanly on the tester's task, so the program stays usable.
4. **The sticks do NOT abort.** Only the twelve buttons and a touch. A thumb resting on a stick must not end
   a run; a button press is deliberate.
- **Cost:** none of these is in your words; each is a safety default you can strike. Striking 1 or 2 leaves
  the screen touch as the only abort when the controller drops.

**D6 — What the SD card keeps. RECOMMENDED (a): each run's summary lines, APPENDED to ONE text file across
boots** — `shulib_auton_runs.txt` at the card's root (`/usd/shulib_auton_runs.txt` on the brain), readable on
any computer. It holds the same words as serial's `[ATS]` lines: what was run, how it ended, the guard's
report, the routine's verdict, and the seat against the END estimate.
- Appended, not one file per boot (Bench Tests' and the old drive's way). A tester session spans reboots:
  upload a fix, run again. The runs before the reboot must survive. **This rests on a new belief, HA-177**
  (what the brain does with an append open), settled at Station M.7.
- **Through the adapter, never a raw `fopen` (re-check finding 12).** `ProsBlockSink` — the adapter that
  already owns the `/usd/` prefix and every mode string — gains an additive `ProsFileMode::Append`; its ONE
  host-tested decision, `openForAppend()`, opens `"ab"` and creates with `"wb"` only when an `"rb"` probe
  says the file is absent, so an existing log is NEVER truncated, whichever way VEXos answers an append on
  a missing file. `hal::BlockCharSink` (the piece T4's N5 (a) specifies) makes it an `ICharSink`. A raw
  `fopen` in the program would also fail SAFE-SAVE's platform scan. **Cost:** two small additive library
  pieces in COMP-t (~2 h with tests), which T4 Part 3 then reuses.
- No SD write happens during a run: the lines are written after the guard hands the robot back.
- **(b)** One file per boot, through the existing truncating adapter. **Cost:** a reboot erases the last
  session's runs unless the card was copied first. No new HA, no library change.
- **(a′)** Append, but name the tester's own `fopen` as an exception in SAFE-SAVE's scan. **Cost:** a second
  owner of `/usd/`, the absent-file question left open, and SAFE-SAVE's scan edited for one program.
  Rejected.
- **(c)** Also the per-tick trace (the blackbox, streamed). **Cost:** a fan-out sink through the device
  provider (library work that T4's N5 proposes for Part 3), in a binary format that needs the reader. On the
  bench, serial already carries every tick when a laptop is plugged in.

**D7 — The guide quotes the LIVE students' file. RECOMMENDED yes.** The new chapter quotes the 40 in auton
from `src/team/autons.cpp` itself. The example scan gains that ONE file as one entry of its list of globs
(§3.7.4 — the same list T4 Part 1a adds the driver file to), so a quote can never drift from what the robot
runs.
- **Cost:** a later edit to the quoted lines must update the chapter in the same commit. That is the standing
  docs rule, and the build says which line drifted.
- **(b)** A compiled snapshot in `test/guide_examples_test.cpp` ("the first auton written in the file").
  **Cost:** the guide stops showing the file the robot runs, and the example scan stays untouched.

**D8 — One executor run or two. RECOMMENDED one.** The documentation list (§6) runs well past pace rule 7's
~5 documents. COMP-b's large list was accepted as one step on the same grounds. The re-check adds two small
additive library pieces (§3.7.5-6) and about fifteen tests and seventeen mutations: a larger run, not a
different kind.
- The tester is not usable at the robot without its station (Station M) and its HA entries, and the code
  cites them. A split would ship a program nobody can run from the worksheet.
- **(b)** Part A: the code, build, tests and the docs that describe the tree. Part B: Station M, guide 16,
  README. **Cost:** one more verification and commit, and the tester waits for Part B before its first run.

**D9 (NEW, re-check finding 1) — RUN only when the boot chose the calibration RECORD. RECOMMENDED yes.**
- "Forward" is whatever the boot's description says it is. The RECORD's forward is the end that LED the
  Calibrate push (the camera, 11/20 end since 2026-09-25); the typed table's front is the 15/16 end, the
  OTHER one (worksheet, "WHICH END IS FORWARD").
- **Today robot two cannot boot from the table at all** (its track width and ratio are UNSET, pinned by
  `test/chassis_table_test.cpp`; no record ⇒ REFUSED). So the rule costs nothing now. It is there for the
  day someone types those two numbers: from then on a lost record would boot "fine" and drive the 40 in
  toward the end the operator was not told to expect, at up to 60 in/s.
- READY shows the source on its own line either way; with yes, RUN greys out and says `RUN needs the
  calibration RECORD: run Calibrate`.
- **(b)** Show it in red, but allow RUN. **Cost:** a warning line is the only thing between a flipped
  forward and a 40 in drive toward the operator.

**D10 (NEW, re-check finding 3) — The 40 in auton on a tank that drifts. RECOMMENDED (a): keep ONE
`moveTo((40, 0) facing 0)` as the team lead specified, and PREDICT its result honestly.**
- **What will likely happen on the floor.** `moveTo` arrives only when the robot is within 0.5 in of the
  point AND within ~1.15° of the heading. A tank cannot drive sideways, so any drift to one side on the way
  (robot two's sides differ by ~9 %) can never be driven out. The step then ends on its 5 s time limit:
  **`ROUTINE STOPPED` at moveTo, fault MOTION_TIMEOUT, END y beyond ±0.5 in — and the brake end action
  still runs, so the robot is safe.** The host test plant shows exactly this from a 3 % weaker side (§0.4
  probe 4). The library says so in its own words: "On tank, laterally-offset targets honestly exit
  TimedOut".
- **(a)** Keep it. The tester's job is to show what the library does on this robot, and this is the most
  useful first thing it can show. The distance is still measured (END x by the estimate against the tape),
  and M.6 records the result word and END y. **Cost:** the first RESULT is red, and the students must be
  told why before they see it (the file's comment and guide 16 say so).
- **(b)** Change the verb. **Cost:** there is no library verb that settles a tank after a sideways drift:
  `driveTo` and `followTrajectory` are `moveTo` legs, and `MotionOptions` carries no settle tolerance. It
  needs library work first.
- **Found for the plan, not COMP-t's to fix:** a tank arrival rule for `moveTo` (e.g. settle on the
  along-track error when `strafeAuthority()` is 0), or a drive-to-point verb that steers. It is the
  coordinator's plan entry (§6 item 7).
- The number stays 40 either way; nothing is fudged.

**Not asked here — the slot.** T4's **N4** asks it (T4 brief, § Decisions, "NEW decisions the amendments
raised", N4). Its recommendation (a) puts the
tester in **slot 7 until T4 Part 3 retires Teleop, then slot 2**. This brief makes (a) documents-only: **no
code string in COMP-t names a slot number** (a source pin, §5). Every document names the slot beside the name
and cites N4. Whichever way N4 goes, only the upload line and the documents change.

---

## 0. Why, and where the line is

### 0.1 The boundary with the 2026-09-25 ruling — stated, because the tester sits right against it
On 2026-09-25 the team lead stopped an auton hook inside Teleop before any code: "I want to eventually use the
actual autonomous not put it in teleop. i want to do things genuinely correct". Autons run only through the
real `autonomous()` (`src/main.cpp` :44-48, :195-199; build-order's deviations table).

**The tester is legitimate ONLY because it runs an auton through EXACTLY the library path `autonomous()` will
use.** Only the TRIGGER differs:

| | the competition program's `autonomous()` (COMP-b) | "shulib Auton Tester" (COMP-t) |
|---|---|---|
| the autons | `registerAutons()` in `src/team/autons.cpp` | the SAME call, the SAME file |
| the run | `competition::runAuton(chassis, guard, entry, …, period)` (`match_runner.hpp` :140-146) | the SAME call |
| the command registry | the one registered beside `registerAutons` once a ROUTE auton exists (B30) | `nullptr`: a route auton is REFUSED by name (`match_runner.hpp` :156-159) until B30's registry exists — the ONE argument that differs today |
| the pacer | a `sequence::RunGuard` as the Chassis's pacer (`robot::BuildOptions::pacer`, `robot.hpp` :189-200) | the SAME |
| the schedule | the entry's own `endActionAt`/`hardStopAt`, checked against the period (C6) | the SAME entry, the SAME `kAutonPeriods` |
| the start | re-seated position AND heading at the start (C2, B12, B14) | the SAME (inside runAuton) |
| the end | the guaranteed end action, the full cancel-all (B10) | the SAME (inside the guard) |
| the task | the program's OWN task (C5, B19) | the program's OWN task (R1) |
| the abandon | a watch pacer calls `guard.requestAbandon()` (B7) | the SAME layering (R3) — on a button, a touch, a controller drop or a disable |
| the trigger | the field (or Timed Run) entering autonomous | RUN on the screen, then a 3 s countdown |

**What the tester CANNOT test, and stays with COMP-b's Station K:**
- a field disable mid-auton, and the whole competition state machine;
- the auton → driver-control hand-over (B2's wait);
- whether the controller's Timed Run enables `autonomous()` (HA-128, B5);
- HA-161 (a) (PROS deletes its competition tasks);
- HA-157's disabled-brain claw level (B28);
- the first-update-at-arm path. COMP-b's estimate is first ticked INSIDE runAuton's live-wait; the tester's
  has been ticking since boot, so its `liveAfter` is 0;
- a ROUTE auton (none exists; the tester refuses one by name until B30's command registry is shared).

The tester proves the RUN path from the seat to the end action on the real robot. K proves everything around
it.

### 0.2 What the robot says today (why "40 in" will read ~43.6 in on the tape)
On 2026-09-25 the saved calibration record read `est x` 44 at a taped 48 in (`R3d-PROGRESS.md` :2507-2510):
the scale under-reads distance by ~8 %. The 40 in auton drives 40 in BY THE ESTIMATE, so on the tape it will
travel about **40 × 48/44 ≈ 43.6 in** until a careful Calibrate rerun fixes the record (the rerun R3d still
owes; SAFE-SAVE B1 is to land before it). **The number in the file stays 40. It is never fudged to make the
tape agree**; the file's banner says so (§3.1). The station writes both numbers down.

**And the likely RESULT word is `ROUTINE STOPPED`, not `FINISHED` (D10; re-check finding 3).** On
2026-09-25 robot two veered 11° right over 48 in open-loop, its right side the weak one (`R3d-PROGRESS.md`
:2507-2514). Under `moveTo` the heading is held (P only, kP 4), but any sideways drift on the way cannot be
driven out by a tank, and `moveTo` only arrives within 0.5 in of the POINT. So expect: the robot stops near
x 40 by the estimate, a little to one side; the step times out at 5 s; MOTION_TIMEOUT is this run's fault;
the brake still runs. That is the library being honest about a tank, not a tester bug. M.6 records the
word, END y and the faults line; the guide explains it before the students see it.

### 0.3 The team lead's newer request (relayed with this task) — OUT, and kept possible
"it would be nice to have the logging be visible and the whatever else on the brain screen not only visible on
the brain screen but be able to see it on the computer … maybe even you can interact with the brain screen
from the computer". That is the robot console, noted in T4's **R7** as the coordinator's plan entry after
G3a, on H1's wire. COMP-t builds none of it, but does not preclude it:
- the tester's screen is a PURE model (`TesterView`, §3.4), so a mirror can render the same frames;
- every run line goes to serial, which a computer already reads over the micro-USB;
- the pages take their touches as data (`ui::TouchSample`).

**A note for that chunk:** a RUN from the computer must still require the controller's abort. The RUN gate's
controller rule (D5.2) is the one that keeps a person's hand on it.

### 0.4 The probes (evidence, not claims — scratch copies, never the repository)
All under `.claude/coordinator/scratch/brief-COMP-t/`.

**1. ARM, the V5's compile.** `src1sim/` holds the new files in SRC1's layout, beside copies of the shared
headers with SRC1-style includes.
- The flags are `common.mk`'s: `-mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=hard -Os -mthumb -Wall -Wextra
  -Wformat=2 -Wno-psabi --std=gnu++20`.
- **A full `-c` compile, because `-fsyntax-only` misses the middle-end warnings** (`-Wformat-truncation`
  among them). The first probe used `-fsyntax-only` and passed files that a real compile then flagged.
- Result: zero warnings in our code with the §3.7 fix, with and without `SHULIB_BUILD_HASH`.

**2. The host test plant** (`host_probe2.cpp`, `-Wpedantic -Wconversion -Wsign-conversion -Werror`). The
students' file is linked and run through `runTesterOnce` with the REAL watch between the plant's pacer and the
guard:
- **Registration:** 2 entries, `40in forward` then `TEST 24in turn back`.
- **Clean run:** the countdown took 3.01 s untouched, then ROUTINE_RAN, routine OK and end action succeeded,
  in 192 paces. The estimate and the truth both read (39.77, 0.00, 0.00°): settled 0.23 in short, inside the
  0.5 in settle.
- **B in the countdown:** ABORTED; runAuton never called; **plant travel 0.00000 in**; `pacesSeen` 0.
- **The controller dropping in the countdown:** ControllerLost.
- **B 0.4–0.5 s into the run:** abandoned, end action skipped, stopped at x ≈ 24–28 (the plant is fast).
- **A disable mid-run:** Disabled.
- **A touch mid-run:** Touch.
- **A second run after an abort:** clean. The latch re-arms, and the guard drops the stale request at arm.
- **The acceptance run through the same path:** ends at (0.24, 0.12, 180.0°).
- **The formatter at its worst:** a 29-character name and run number 12345. Every line is under 199 bytes.

**3. The real gate, 7 builds.** A `git archive` of `37706ce` (flat layout) with the §3.6 wiring and the new
files: **`tools/src_build_gate.py check` PASSED all seven builds**.
- Each build reported "10/10 TUs, 0 disallowed, beacons (variant, robot) and program checked".
- `[program-table] 6 programs, 14 entry names`.
- The §3.6 self-test pair was run alone and behaved as specified: 28 FAILS naming
  `-DSHULIB_PROGRAM_AUTONTEST` and both beacons; 29 PASSES.
- `nm` of the tank-autontest package: `autonTesterInitialize` and `registerAutons` linked, no other
  program's entry.

**Not probed:**
- SRC1's include root inside the real Makefile (SRC1 builds it);
- the final painter's changed-lines path on a brain (R8);
- anything on a robot.

**The re-check's probes (2026-09-27, ~04:00–05:30; same folder):**

**4. A weak side on the host test plant** (`host_probe3.cpp`): the 40 in and the acceptance run through
`runTesterOnce`, the right wheel's volts scaled by a `DegradationModel`.

| right side | 40 in forward | acceptance run |
|---|---|---|
| ×1.00 | FINISHED, END (39.77, 0.00, 0.00°), scoring T+1.80 s | FINISHED, (0.24, 0.12, 180.0°) |
| ×0.97 | **ROUTINE STOPPED at moveTo, MOTION_TIMEOUT**, END (39.96, −0.93, −0.26°), scoring T+5.01 s, end action succeeded | FINISHED, (0.16, 0.46, 177.97°) |
| ×0.95 | ROUTINE STOPPED, END y −1.56 | ROUTINE STOPPED at step 1 (moveTo), END (23.93, −0.64) |
| ×0.91 | ROUTINE STOPPED, END y −2.86 | ROUTINE STOPPED at step 1, END (23.87, −1.17) |

The estimate and the truth agree in every row (the plant's odometry is perfect); only the arrival fails. A
kinematic plant with no inertia gives a SHAPE, not inches.

**5. The amended design on the host** (`host_probe4.cpp`, `-Wpedantic -Wconversion -Wsign-conversion
-Werror`, over `src1sim2/` and `overlay/`): **PROBE4 PASS.**
- `testerTouchFrom` on a PROS-shaped struct: press 5 / release 4 ⇒ `pressCount` 5, `fingerDown`; an error
  read ⇒ error, never a finger.
- The chain proof: guard → watch → plant PROVEN; the Chassis on the watch alone, or on the plant alone, NOT.
- The amended `runTesterOnce`, clean: `runStarted` at 3.0100 s after the countdown began, plant travel
  0.00000 at that moment, hand-off 0.020 s (the quiet tick), FINISHED, faults 0 → 0.
- ×0.95: ROUTINE STOPPED, "faults this run: 1, last MOTION_TIMEOUT", END y −1.56, end action succeeded.
- A pacer layer that throws 0.5 s into the countdown: `threw`, the watch disarmed, travel 0.00000,
  PROGRAM ERROR, and the line "… it threw (a program error): …; the drive was stopped".
- READY at its worst case: 8 lines, the longest exactly 50 (`FAULT n=99999 last GPS_GATE_REJECT  BUILD
  12:34:56`). **The first cut silently DROPPED the 8th line** (the view held 7) — the draft's own defect,
  fixed (§3.4).
- RESULT entered with a finger down: its lift over PICK acts on nothing; the buttons arm 0.5 s after the
  first frame after the lift; a tap then goes to PICK.
- `openForAppend`'s four branches with a scripted opener (`rb ab`; `rb ab`; `rb ab wb`; `rb ab` — and NO
  `wb` when the file exists). `ProsBlockSink` Append through the shim on a real temp directory, two
  constructions ("boots"): `boot 1 run 1\nboot 2 run 1\n`, CREATED then APPENDED; no card ⇒ Refused.
- `BlockCharSink`: a refused flush counted (1), nothing thrown.

**6. The V5 compile and the real gate, amended.** `src1sim2/programs/auton_tester_program.cpp` and
`team/autons.cpp`, and the three changed library headers alone, `-c` under the `src/` warning set with
`-Wunused-function`, with and without `SHULIB_BUILD_HASH`: **zero warnings** (the first amended cut had
one real `-Wformat-truncation` in the stop line, 211 bytes into 200 — fixed with `%.100s`). The amended files
flattened into the scratch tree (`tree/`): **`tools/src_build_gate.py check` PASSED all seven builds**, each
"10/10 TUs compiled, 0 disallowed, beacons (variant, robot) and program checked", `[program-table] 6
programs, 14 entry names` (logs: `gate_recheck.log`, and `gate_recheck2.log` on the FINAL copies).

### 0.5 Found while probing (each is handled in scope; each is a real finding)
1. **A latent gate failure in COMP-a's header.** `src/team/autons.cpp` is the first TU that calls
   `AutonRegistry::add()`, so `checkAutonEntry()` is emitted for the first time.
   - At `-Os`, GCC inlines `Route::valid()` into `char routeWhy[160]` (`auton_registry.hpp` :296-297) and
     flags `route.hpp` :453-456: output "between 105 and 173 bytes into a destination of size 160",
     `-Wformat-truncation`.
   - The src gate fails **EVERY build** on it, because every `src/*.cpp` compiles into every build: the
     probe's bench build FAILED with the unfixed header.
   - The ARM header gate never saw it: an inline function nothing calls is never emitted, so it is never
     analysed.
   - **Fixed at its root in COMP-a's header** (§3.7; RESUMING: "a chunk that finds a flaw in an earlier chunk
     fixes it there").
2. **An error read would have aborted the next good read.** PROS reports every touch field as −1 on
   `E_TOUCH_ERROR` (`ui/touch_panel.hpp` :236-238). A baseline taken from such a read at `arm()` makes the
   next good read's real count differ, and that reads as a touch. It is R2a's "E_TOUCH_ERROR read taken as a
   tap" class. The watch now takes its baseline from the first GOOD read (§3.2), and a test pins it.
3. **The RESULT page must take its auton from the record**, never from whatever page was up before. The
   probe's sketch indexed −1 and the registry's precondition threw.
4. **An abort latched before the guard arms costs one commanded tick.** runAuton's live-wait is satisfied
   without a tick when the estimate is already live, so the first pace comes after the moveTo's first command;
   on the plant that is 0.60 in at 60 in/s.
   - In the tester's flow this cannot happen: the countdown's last check sees any latch, and runAuton is not
     called.
   - It is B22's documented one-tick latch, and a test pins it at ≤ one tick.
5. **Two runtime strings would have lied.** The screen's cause words must fit 50 characters with the buttons'
   names: the probe's first cut overflowed at 70. That is why there are SHORT cause words (`abortCauseShort`,
   ≤ 15) and precisions (§3.4).

---

## 1. What exists (every line opened for this brief)

| Piece | Where | What matters |
|---|---|---|
| The runner | `include/shulib/competition/match_runner.hpp` | `runAuton(chassis, guard, entry, commands, telemetry, mechanisms, period)` :140-146. Order of events: refuse, arm FIRST, live-wait, re-seat, run, guard end (:7-30). The `AutonRunResult` tag, `why`, `guard`, `routine`, `armedAt`, `liveAfter`, `seated`, `imuAtSeat` (:94-125). It throws only preconditions (:137-139) |
| The registry | `…/competition/auton_registry.hpp` | `AutonEntry` (:217-251); `AutonPeriods` (:106-129), both 0 = refused; `checkAutonEntry` (:267-358); `add()` throws naming the auton (:393-397). **`routeWhy[160]` :296 — the §0.5.1 defect** |
| The acceptance run | `…/competition/acceptance_run.hpp` | `kAcceptanceRunName = "TEST 24in turn back"` :34; `acceptanceRunEntry(kind, endActionAt, hardStopAt)` :51-61 (RELATIVE, brake) |
| The selector | `…/competition/auton_menu.hpp` | the pure `AutonMenu` over `ui/`; NONE default :14-16; slots :57-68 (PREV bottom-left, NONE centre, NEXT bottom-right); non-movable :199-206; `touch(sample, drawn)` :221-259 (its OWN `ReleaseEdge`, :407) |
| The generic panel | `include/shulib/ui/touch_panel.hpp` | budgets 18/36/50 (:47-51, HA-136); `ArmingClock` :210-231; `TouchSample` (error ⇒ all −1) :235-245; `ReleaseEdge` :249-267; `hitTest` :271-279 |
| The guard | `include/shulib/sequence/run_guard.hpp` | `pace()` checks BEFORE the world advances (:304-321); `requestAbandon()` one atomic store (:332); **a request made while idle is DROPPED at the next arm — a watch must request on the STATE every tick** (:154-157, :515-517); `run()` (:452-486); `RunGuardReport` (:233-265); **`pacesSeen` counts only a RUN's paces** (:305-306), so nothing in the guard can prove the wiring before a run — hence the additive `pacesTotal()` (R13) |
| The pacer seam | `include/shulib/robot/robot.hpp` | `BuildOptions::pacer`, BORROWED for the Robot's life (:189-200); `builtMechanismAt(i)` :637, `mechanismAbsence(i)` :693, `config()` :606 |
| The idle tick | `include/shulib/chassis/chassis.hpp`; `motion/motion_scheduler.hpp` | `Chassis::wait(d)` commands nothing (:354-377); `waitUntil` checks `pred` FIRST, then `tickImpl` + `pace` (:763-792); every tick updates the localizer (`tickBody` :1010-1017) |
| The motion defaults | `motion/motion_config.hpp` | `maxLinearSpeed` 60 in/s :88; `translationSettle` 0.5 in :96-97; `defaultTimeout` 5 s :112 (all PROVISIONAL) |
| The boot | `src/boot_source.hpp` [`common/`] | `decideBootSource(tag)`; `BootOutcome`; `buildOptions()` :259-265, whose doc still says "robot::Robot reads it during construction only" (true of the ports, not of a pacer — B8); :42-44 "the host build compiles … never src/*.cpp" (§6) |
| The example program | `src/teleop_program.cpp` [`programs/`] | a function-local static of EMPTY optionals (:489-513); `initialize` (:877-953): banner, `decideBootSource`, one try, `attachFaultLatch`, session header; `paintRefusal` (:520-545) |
| The print path | `src/screen_text.hpp` [`common/`] | `printText(calibration::TextSize, …)` :75-108: the ONLY path; a literal `"%s"`. Note the type: `calibration::TextSize`, not `ui::TextSize` |
| The build | `src/build_selection.hpp`; `src/main.cpp` | the rule list :8-12; guards :69-84; `enum class Program` :89-95; ONE chain :115-139. `main.cpp`'s five switches at :134, :160, :178, :202, :236; the table :20-36, "Four programs" :30; the autonomous comments :44-48, :195-199 |
| The gate | `tools/src_build_gate.py` | builds doc :20-50, :52, :80; `VARIANTS` :275; `BUILD_ARGS` :276-283; `VARIANT_DEFINES` :284-291; `VARIANT_BEACONS` :293-300; `BUILD_ROBOT` :307-314; `BUILD_PROGRAM` :380-387; `PROGRAM_ENTRY_SYMBOLS` :388-396; cases 14/15 :1421-1445; the last case (27) ends :1925 |
| The Makefile | `Makefile` | PROGRAM block :91-148; `PROGRAM?=tester` :128; "Four programs in four slots" :118; valid values :147 |
| PROS | `include/pros/screen.h`, `misc.h`, `rtos.h/.hpp` | `screen_touch_status_s_t`: `press_count` :85, `release_count` :86, `E_TOUCH_ERROR` :74; `competition_status` bits :41-50; `competition_get_status` :75; `competition_is_disabled` :95; `usd_is_installed` :801; `Task(task_fn_t, void*, prio, stack, name)` rtos.hpp :83-84; `TASK_PRIORITY_DEFAULT` 8, `TASK_STACK_DEPTH_DEFAULT` 0x2000 words (rtos.h :63, :72) |
| The controller | `include/shulib/hal/controller.hpp`; `hal/pros/controller.hpp` | twelve `ControllerButton`s L1..A (:48-66); `pressed()` a LEVEL, `isConnected()` (:96-103); `ProsController(ControllerId)` :67 |
| The logs | `hal/pros/device_provider.hpp` :373-376; `diag/term_sink.hpp` :278; `hal/pros/char_sink.hpp`; `hal/pros/block_sink.hpp` :79-90 | the provider's sink is a TermSink over USB only; TermSink cuts a message at 200 bytes; `ProsCharSink(FILE*)` writes and flushes each line; `ProsBlockSink` opens `"wb"` TRUNCATING and OWNS the `/usd/` prefix (a bare name, HA-122) — D6 (a) adds an Append mode to it rather than a second `fopen` |
| SAFE-SAVE (briefed, not built) | `docs/internal/chunks/SAFE-SAVE-calibrate-save-and-teleop-label.md` §0.2 (~:337-345), §2 "The platform scan" (~:813-820) | PROS sends an `O_APPEND` open to VEXos's `vexFileOpenWrite`, any other write-only open to `vexFileOpenCreate`, and refuses read-write; its scan allows `fopen` ONLY in `hal/pros/block_sink.hpp`, `hal/pros/block_source.hpp` and `src/bench_r3a.cpp`, and no mode with `+` |
| The beliefs | `docs/hardware-assumptions.md` | HA-122 (fopen needs `/usd/`); HA-128 (a controller-menu start arms a pretend match); HA-147; **HA-161** (:1941-1983: PROS deletes its competition tasks, not the program's own) |
| The plan around it | T4 brief (by heading — it is still moving: § Decisions R6 and R7; "NEW decisions …" N4 and N5; § Scope Part 1a items 6 and 7); SRC1 brief §1 and §2.2; COMP brief B5, B7, B8, B9, B12, B18, B19, B20, B22, B25, B29 | T4 must NOT create the students' file or anything of the tester's (R6); SRC1's `src/team/` is "where students edit" |

---

## 2. Rulings (load-bearing; each with its reasoning)

**R1 — Everything after boot runs on the tester's OWN task; `opcontrol()` does nothing.** `initialize()`
builds everything and starts ONE `pros::Task` (the function-pointer constructor, `TASK_PRIORITY_DEFAULT`,
`TASK_STACK_DEPTH_DEFAULT` — B19's). From then on only that task touches the Robot, the Chassis, the guard, the
watch, the screen and the SD log. `main.cpp`'s `disabled()`, `competition_initialize()`, `autonomous()` and
`opcontrol()` return at once for this program.
- *Why:* HA-161 (a). A mode change deletes PROS's competition tasks mid-statement. A run inside `opcontrol()`
  would leave the scheduler's motion on a freed stack and the guard `running_`. Even an IDLE `chassis.wait()`
  on a deleted task leaves the scheduler's in-wait flag set (`FlagScope`, `motion_scheduler.hpp` :765-775),
  so every later wait throws. That is the checker's
  finding behind the team lead's C5 ruling (COMP brief :16-21).
- It also rehearses COMP-b's task shape (B19), and makes B2's rule total: one task on the Chassis for the
  program's whole life. One task paints (B20), and one task uses each sink.
- *Rejected:* the UI in `opcontrol()` with a separate auton task. It needs a cross-task channel and snapshot
  atomics, a re-sync when `opcontrol()` is re-created, and still has the deleted-in-wait hazard for the
  estimate ticking. *Rejected:* everything inline in `opcontrol()`, for HA-161 (a).
- It rests on HA-161 (b), which Station K settles; on the bench nothing changes mode (R4 refuses RUN when
  anything is connected).

**R2 — The idle loop is one-tick `Chassis::wait()`s; nothing moves until RUN and the countdown.** Each
iteration:
- `chassis.wait(10 ms)`: the localizer updates, the health checks run, one idle record is emitted, and the
  pacer chain paints. **The drive is never commanded** (`chassis.hpp` :354-367).
- one competition-status check (logged on CHANGE, B5-style);
- one touch read.

The estimate is therefore LIVE on the READY page. This is the tester's one difference from COMP-b's idle
(B19: `notify_take`), and §0.1 names what it hides. Serial carries a `[LOC] idle` record per tick, as
Teleop's drive records do.

**R3 — The pacer chain: `Chassis → RunGuard → AbortWatch → ScreenPacer → ProsTickPacer`.**
- B7's layering: the watch wraps the real pacer, the guard wraps the watch, and the Robot is built with
  `.pacer = &guard`.
- Plus the painter, inside the watch, so an abort is decided before any paint delay. Painting on the tick is
  Teleop's per-tick panel precedent.
- **Latency:** the watch latches in tick N's pace, AFTER the guard's check. The guard cuts at tick N+1's
  pace, BEFORE the world advances. So **at most one more commanded tick (10 ms) after the watch sees the
  abort** (B7). *Rejected:* the watch outside the guard (zero ticks). It deviates from runAuton's documented
  contract ("chassis MUST be the Chassis built with guard as its pacer"), and 10 ms is nothing.

**R4 — The RUN gate (`runAllowed`, PROS-free).** RUN is armed only when ALL hold:
- the estimate is live (`qualityClass() != Uninitialized`);
- a controller is connected (D5.2);
- `competition_get_status()` has none of DISABLED, AUTONOMOUS, CONNECTED;
- **the boot chose the calibration RECORD (D9; re-check finding 1)** — `TesterFacts::bootFromRecord`;
- the page's arming delay has passed.

The READY page names the first failing reason, in this order: disabled › a competition bit › not the
RECORD › no controller › not live. `touch()` re-checks the gate at the release, not only the
drawn button. *Why the status bits:* a controller-menu start arms a pretend match (HA-128, status 0x0f). The
tester then shows its pages but refuses RUN, saying "start it from the brain". It never runs an auton in any
competition mode, which keeps the boundary of §0.1 sharp.

**R5 — The abort (PROS-free core, §3.2).** A LATCH:
- armed at RUN, disarmed after the run;
- it records the FIRST cause (Disabled > ControllerLost > Button > Touch within one read) and the buttons
  down;
- it requests the abandon EVERY pace while latched and a run is live. A request made while idle is dropped
  at the next arm (run_guard.hpp :154-157), so an edge-only request would lose an abort latched in the
  countdown's last tick;
- a button is a LEVEL (a held button aborts at once; `pressed()` is a level, HA-104/HA-147);
- a touch is a change of PROS's `press_count` from the arm-time baseline. **Press, not release:** the robot
  stops when the finger LANDS. The baseline comes from the first GOOD read (§0.5.2), and error reads never
  count.
- **PROS is read in ONE line each, through host-tested pure functions (re-check finding 4).** The program's
  `readTouch()` is `testerTouchFrom(pros::c::screen_touch_status(), E_TOUCH_ERROR)`, and
  `ProsAbortInputs::read()` is `abortInputsFrom(readTouch(), master, competition_is_disabled() != 0)`
  (§3.2). Which field is the press count, what an error read means, and that the connection comes from the
  controller are decided in the pure functions, where (6b) can see them; the two one-liners are pinned.

**R6 — The countdown and the run are ONE PROS-free function, `runTesterOnce()` (§3.3), host-run on the
plant.** Its steps, ALL of 1–4 inside ONE try (re-check finding 6):
1. arm the watch (and read the fault count: the run's faults are counted from here — finding 7);
2. the countdown (one-tick waits, nothing commanded), checked every tick;
3. **one QUIET tick** (finding 13): `runStarted()` flips the page, and one `Chassis::wait` paints RUNNING
   whole on a tick that commands nothing, and reads the abort once more;
4. if nothing aborted, runAuton;
5. the handler, for `PreconditionError` and `...`: record it and panic-stop the drive (`chassis.cancel()`;
   after a runAuton throw the guard has already safed the robot on the unwind — `run_guard.hpp` :442-445;
   after a countdown throw nothing had been commanded);
6. disarm and record (the end estimate; the fault count again, and the last fault ONLY if the count rose).

The program TU only calls it. *Why:* this is the sequence that decides whether the robot moves, and the
program TU cannot be compiled on the host (`pros/screen.h`).

**R7 — The screen is a pure model over `ui/` and `competition::AutonMenu` (§3.4).**
- PICK is the AutonMenu's OWN view: the tester therefore shows the selector COMP-b's program will show.
- **Every page entry takes a FRESH `ReleaseEdge` at the current release count and restarts the arming
  clock.** PICK does it by emplacing a NEW `AutonMenu` into an optional; AutonMenu is non-movable and its edge
  is private (:199-206, :407). *Why:* a tap is never replayed on the next page, and a new page's buttons are
  dead for 0.5 s. That matters most on RESULT: an abort touch's RELEASE arrives after the result is up, and
  must hit nothing.
- **RESULT entered with a finger DOWN waits for that finger (re-check finding 13).** The 0.5 s alone was not
  enough: a touch held through an abort lifts whenever the operator lets go. `showResult(rec, releaseCount,
  fingerDown)` keeps both buttons grey until that finger's release has been consumed (it acts on nothing),
  and the arming clock starts only then. `fingerDown` is a good read with `press_count > release_count`
  (HA-178 (a)'s belief); if that belief is wrong, it reads false and the page keeps the plain 0.5 s rule.
  Only RESULT can be entered with a finger down: every other page is reached by a release.
- **No new button under the one just pressed:**
  - READY puts BACK and RUN only in the bottom row. The entry rows it is reached from are empty.
  - RESULT puts PICK bottom-left and AGAIN bottom-CENTRE, never where RUN is.
- The menu's `lockChoice()` is not used: the entry index travels in the RUN action, and no menu exists while a
  run is live. So no touch can change what runs.

**R8 — One painter, the pacer layer.**
- A frame whose BUTTONS differ (page, ids, rectangles, labels, armed, chosen — `sameButtons()`, §3.4) is
  painted WHOLE at once, so a touch is always tested against the frame on the screen.
- A text-only change is painted at most every 200 ms, and only the lines that changed (PROS leaks every
  print).
- Every string goes through `printText` (`ui::TextSize` mapped to `calibration::TextSize`).
- `g_screenPrints` is reported on each run's last log line.

**R9 — The log.** Serial carries:
- the banner and the boot line;
- the session header (Teleop's block);
- one `[ATS]` line per absent mechanism (B9);
- status changes;
- per run, the RUN line, then `testerRunLines()` — runAuton's own `[COMP]` and the guard's `[SEQ]` lines ride
  the provider's sink as always.

The SD card (D6) gets the boot line, status changes and every run's lines, through a SECOND `diag::TermSink`
over `hal::BlockCharSink` over `hal::pros::ProsBlockSink{"shulib_auton_runs.txt", ProsFileMode::Append}`,
constructed once in `initialize()` (re-check finding 12: the adapter owns `/usd/` and every mode; the program
names neither). **Never during a run.** The boot line says `SD run log ON, CREATED` or `ON, APPENDED to
/usd/shulib_auton_runs.txt` (the adapter's `path()`), or `OFF`. No card, or a log that will not open ⇒ "SD:
NO CARD" on READY and OFF on serial, never a refusal.

**R10 — B8, the first program that adds a pacer.** `robot::BuildOptions opts = boot.built.buildOptions();
opts.pacer = &*st.guard;`. `BootConfig::buildOptions()`'s doc (`boot_source.hpp` :259-262) is reworded now:
the ports are read during construction only, a caller-added `.pacer` is BORROWED for the Robot's whole life
(`robot.hpp` :195-199). COMP-b then finds it done.

**R11 — Two beliefs, registered (HA-177, HA-178).**
- **HA-177 (rewritten at the re-check, finding 12):** what the brain does with an APPEND open under `/usd/`.
  1. `fopen(path, "ab")` reaches VEXos as `vexFileOpenWrite`: newlib makes `"ab"` a write-only open with
     `O_APPEND`, and PROS's `usd_open_r` sends exactly that case to `vexFileOpenWrite` (every other
     write-only open to `vexFileOpenCreate`).
  2. On an EXISTING file, `vexFileOpenWrite` writes after its last byte and keeps every earlier byte.
  3. On an ABSENT file it may fail or create; unknown. `openForAppend()` handles both (it creates with
     `"wb"` only after an `"rb"` probe says the file is absent), so (3) changes only which word the boot line
     prints, CREATED either way.
  4. A line written and flushed survives a power-off right after it (PROS calls `vexFileSync` after every
     write it passes on).
  - *Confidence:* (1) reasoned from the vendored `libpros.a`/`libc.a`, disassembled 2026-09-26 (the same
    reading as SAFE-SAVE's HA-166 — cite HA-166 if it is registered by then, else the disassembly by date);
    (2) and (3) invented (VEXos's); (4) reasoned for the call, invented for what the sync commits. HA-122
    confirmed `fopen` under `/usd/` for `"wb"` only.
  - *Settle:* Station M.7 (a fresh card: CREATED; a power-cycle: APPENDED; all runs present, in order).
  - *Blast radius:* (2) wrong ⇒ earlier boots' runs lost from the card (serial still has them when
    captured); the robot's motion is unaffected. The one remaining truncation path: the `"rb"` probe fails
    on a file that exists AND the `"ab"` open fails too — then `"wb"` empties it. Its cost is the same log.
- **HA-178:** the tester's abort and RUN gate read the right things from PROS:
  - (a) `press_count` rises when a finger LANDS, not when it lifts, read from a task the program created;
  - (b) a controller button's level turns true within ~50 ms of the press, read from that task;
  - (c) started from the brain's screen with nothing connected and the controller on its home screen,
    `competition_get_status()` reads 0x00. There is no CONNECTED bit from the controller's own link.
  - (d) **(added at the re-check, finding 4)** a controller switched OFF reads `is_connected()` false within
    ~1 s, read from that task.
  - *Confidence:* (a) reasoned from the vendored text; (b) invented; (c) reasoned from HA-128's measurement;
    (d) invented.
  - *Settle:* Station M.2-M.4 (M.4's controller-OFF steps settle (d)).
  - *Blast radius:* (a) wrong ⇒ a touch aborts later (at release) or never, and the buttons still abort; and
    RESULT's finger-down wait falls back to the plain 0.5 s rule; (b) slow ⇒ the robot stops that much
    later; (c) wrong ⇒ RUN is refused on the bench, with the bits on the status line (fail-safe); (d) slow ⇒
    a controller that dies mid-run is noticed that much later (a touch still aborts).
- HA-161's Source gains the tester's task (it rests on (b)).

**R12 — What the tester's mechanisms do.**
- The guard safes the Robot's PRESENT mechanisms (B9). The claw's safe position is CLOSED, which is also its
  start level, and the pivots HOLD (DEFECTS2).
- The tester never commands them otherwise, so **the claw stays closed through a run and the pivots hold at
  the cut and at the end** (M.9).
- An absent one is left out and logged by name.

**R13 (NEW, re-check finding 4) — The boot PROVES the pacer chain, or refuses.** After the Robot, the registry
and the screen are built, still inside `initialize()`'s try and before the task starts,
`pacerChainProven(chassis, guard, watch)` runs ONE idle tick (`Chassis::wait`: nothing commanded; the painter
is not enabled yet) and checks that it paced BOTH the guard (`RunGuard::pacesTotal()`, additive — §3.7.5)
and the watch (`AbortWatch::paces()`). False ⇒ the boot is refused: "the tester's pacer chain is WRONG … the
abort could not stop the robot; this is a build error". *Why a runtime proof and not only pins:* the abort's
whole promise rests on this wiring, `run()` cannot check it ("the guard has no way to verify that wiring",
`run_guard.hpp` :446-450, and only warns AFTER a run), and a pin catches only the spellings it names. A Robot
built without `opts` paces neither; one built with `.pacer = &*st.watch` paces the watch only (the abandon
would be requested and never acted on); both are refused on the robot, every boot.

**R14 (NEW, re-check finding 6) — A throw never kills the tester's task silently.** `runTesterOnce` catches
everything from its arm to its end (R6). The task's whole body is inside a try; its handler calls
`stopTheTester(what)`: the PROS-free `stopAfterProgramError()` (§3.3: the drive to 0 V + Brake, every PRESENT
mechanism's `applySafeState()`, the watch disarmed, each step tried on its own, never throwing), an `Error`
line on both logs, and the refusal screen ("the tester STOPPED (a program error)" / "The drive is stopped
(0 V + Brake). Restart the program."). Then the task idles on `pros::delay(100)` for ever and never touches
the Robot again. *Why:* a C++ exception escaping a PROS task function ends in `std::terminate`, with the drive
in whatever state it was last given.

---

## 3. Scope — IN

### 3.1 The students' file — `src/team/autons.hpp`, `src/team/autons.cpp` (VERBATIM; COMP brief B18)
Compiled into every ARM build (as every `src/**/*.cpp`) and into the host suite (§3.7). No static-
initialization registration (C3): the programs call `registerAutons()` from `initialize()`. **COMP-b calls the
same function and never writes a second file.**

```cpp
#pragma once
//
// autons.hpp -- the team's autons: the one function both auton-running programs call, and the
// two periods every auton's schedule is checked against. The autons themselves are in
// src/team/autons.cpp.
//
// "shulib Auton Tester" and "shulib Competition" both call registerAutons() from initialize(),
// inside the try around building the robot: an entry the registry refuses throws a
// PreconditionError whose sentence names the auton, the program refuses its boot with that
// sentence on the screen, and nothing moves.
//
// PROS-free on purpose: the host suite compiles src/team/autons.cpp and runs every auton on the
// host test plant (test/autons_test.cpp).

#include "shulib/competition/auton_registry.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::bench {

/// The two periods an auton can run in, from the VEX U game manual (Override, v1.0, 2026-07-02):
/// a head-to-head match opens with a 30 s autonomous period (<VUT4>), and an autonomous coding
/// skills run lasts 60 s (Appendix B). An entry whose hardStopAt does not fit inside its kind's
/// period is refused by name when it is registered.
inline constexpr competition::AutonPeriods kAutonPeriods{.match = units::Time{30.0},
                                                         .skills = units::Time{60.0}};

/// Register every auton the team has written, in the order the brain's menu lists them. Called
/// ONCE, from a program's initialize(), inside its try: `registry.add()` throws a
/// PreconditionError naming the auton when an entry is refused.
void registerAutons(competition::AutonRegistry& registry);

}  // namespace shulib::bench
```

```cpp
// autons.cpp -- THE TEAM'S AUTONS for robot two: every autonomous routine the robot can run,
// and registerAutons() at the bottom, the one list the programs read.
//
// WHO RUNS THESE. "shulib Auton Tester" runs the one you pick on the brain's screen when you touch
// RUN (on the bench, no competition switch needed). "shulib Competition" runs the one picked on
// its screen when the match's autonomous period starts. Both hand the SAME entry to the SAME
// library call (competition::runAuton), so the auton you test on the bench is the auton that runs
// in the match. Only what STARTS it differs.
//
// THE FRAME. x is straight ahead of the robot, y is to its left, and angles grow counterclockwise
// (0 degrees faces +x). "Ahead" is the robot's FRONT as the calibration record defines it: the end
// that LED the push when "shulib Calibrate" ran. A RELATIVE start makes the spot where the robot
// is placed (0, 0), facing 0 degrees, at the moment the auton starts.
//
// THE DISTANCES are the robot's OWN estimate (its drive encoders and its IMU). When the tape
// disagrees with an auton, the calibration record is what is wrong, not the number written here:
// re-run "shulib Calibrate", and never change an auton's number to make the tape agree.
//
// PROVENANCE. The 40 in auton was written for the team on the team lead's go of 2026-09-26 ("no
// you can write the auton"). Every line is the team's to explain, defend and change.

#include "team/autons.hpp"

#include "shulib/chassis/routine.hpp"
#include "shulib/competition/acceptance_run.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/units/literals.hpp"

namespace shulib::bench {

using namespace units::literals;  // 40_in, 0_deg, 27_s

namespace {

// ── THE 40 IN AUTON ─────────────────────────────────────────────────────────────────────
// Drive straight forward 40 inches and stop, facing the way the robot started.
// ONE moveTo: the library drives to the POSE -- 40 in ahead, 0 in to the side, facing 0 degrees --
// and the step ends when the robot is there and still, or when the motion's own time limit ends
// it. Nothing is tuned here: the motion uses the robot's configured defaults.
// A TANK CANNOT DRIVE SIDEWAYS. If the robot drifts to one side on the way (one side of the drive
// weaker than the other does that), the drift cannot be driven out: the step ends on its time
// limit instead, the result reads ROUTINE STOPPED with the fault MOTION_TIMEOUT, and the brake at
// the end still runs. That is the library telling the truth about this robot, not a broken auton.
chassis::RoutineResult fortyInchesForward(competition::AutonContext& ctx) {
    chassis::Routine r{ctx.chassis, "40in-forward"};
    r.moveTo(math::Pose2d{40_in, 0_in, 0_deg});
    return r.result();
}

}  // namespace

void registerAutons(competition::AutonRegistry& registry) {
    // 1. THE 40 IN AUTON.
    competition::AutonEntry forty{};
    forty.name = "40in forward";                      // what the brain's menu shows
    forty.kind = competition::AutonKind::Match;       // runs in the 30 s autonomous period
    forty.routine = &fortyInchesForward;              // the function above
    forty.start = competition::StartPose::relative(); // where we place it is (0, 0), facing 0
    forty.endActionAt = 27_s;  // still running at 27 s? it is stopped, and the end action starts
    forty.hardStopAt = 29_s;   // everything stops: 2 s after 27 s, 1 s before the field's 30 s
    // endAction is left out: the end action is chassis.brake() -- 0 V under Brake until still.
    registry.add(forty);

    // 2. THE LIBRARY'S ACCEPTANCE RUN (drive 24 in, turn to 90 degrees, face the start, drive
    //    back): the test that the whole library works on this robot, on the same schedule.
    registry.add(competition::acceptanceRunEntry(competition::AutonKind::Match, 27_s, 29_s));
}

}  // namespace shulib::bench
```

**Every line, checked against the real headers.**
- `Routine{Chassis&, const char*}` (routine.hpp :195); `moveTo(const Pose2d&, const MotionOptions& = {})`
  (:223); `result()`.
- `Pose2d(Length, Length, Angle)` (pose2d.hpp :34).
- `40_in` / `0_deg` / `27_s` (units/literals.hpp :22-70; `_deg` is not constexpr, used at run time).
- `StartPose::relative()` (auton_registry.hpp :155).
- `AutonKind::Match`; `AutonRoutine = RoutineResult (*)(AutonContext&)` (:209).
- `acceptanceRunEntry(AutonKind, Time, Time)` (acceptance_run.hpp :51).
- `AutonPeriods` is an aggregate usable in a constant expression, compiled `constexpr` on host and ARM.

The name: 12 characters, within the 29-character `kMaxAutonNameChars`; `ui::screenText` passes it; it is
unique, and not "NONE".

**Re-check (2026-09-27): two comment paragraphs added, no code changed** — THE FRAME now says what "ahead"
is (finding 1), and the 40 in auton says what a tank does with a sideways drift (finding 3). Both compiled
for the V5 in `src1sim2/team/autons.cpp` (probe 6). The "time limit" is deliberately not a number in the
file: it is the robot's `defaultTimeout` (5 s, PROVISIONAL), and the guide states it with its source, so
the file cannot drift from the config.

**Order:** the 40 in first — the team's, the one run most — then the library's test.

### 3.2 The abort — `src/programs/auton_abort_watch.hpp` (PROS-free; its core VERBATIM)
The probe copy is `src1sim2/programs/auton_abort_watch.hpp` (amended at the re-check): the file banner, `AbortCause`,
`abortCauseWord()` (≤ 27 characters, for the log), `abortCauseShort()` (≤ 15, for the screen), `AbortInputs`,
`buttonsDownOf()`, `IAbortInputs` and `AbortWatch`. The load-bearing parts:

```cpp
[[nodiscard]] constexpr AbortCause abortCauseOf(const AbortInputs& in,
                                                std::int32_t pressBaseline) noexcept {
    if (in.disabled) {
        return AbortCause::Disabled;
    }
    if (!in.controllerConnected) {
        return AbortCause::ControllerLost;
    }
    if (in.buttonsDown != 0) {
        return AbortCause::Button;
    }
    if (!in.touchError && in.pressCount != pressBaseline) {
        return AbortCause::Touch;
    }
    return AbortCause::None;
}
```
```cpp
    void arm() {
        const AbortInputs now = inputs_->read();
        baselineKnown_ = !now.touchError;
        pressBaseline_ = now.pressCount;
        buttonsAtLatch_.store(0);
        cause_.store(AbortCause::None);
        armed_.store(true);
    }

    void pace() override {
        ++paces_;  // every pace, armed or not: the boot's chain proof (R13)
        if (armed_.load()) {
            AbortCause c = cause_.load();
            if (c == AbortCause::None) {
                AbortInputs in = inputs_->read();
                if (!baselineKnown_ && !in.touchError) {
                    pressBaseline_ = in.pressCount;  // the first good read IS the baseline
                    baselineKnown_ = true;
                }
                if (!baselineKnown_) {
                    in.touchError = true;  // no baseline yet: the touch says nothing this tick
                }
                c = abortCauseOf(in, pressBaseline_);
                if (c != AbortCause::None) {
                    buttonsAtLatch_.store(in.buttonsDown);
                    cause_.store(c);
                }
            }
            if (c != AbortCause::None && guard_ != nullptr && guard_->running()) {
                guard_->requestAbandon();  // every tick while latched (file banner)
            }
        }
        inner_->pace();
    }
```
- `arm()`, `disarm()` and `pace()` belong to the ONE task that runs the Chassis. `cause()`,
  `buttonsAtLatch()` and `armed()` are atomics, readable anywhere.
- Non-copyable: the guard holds it by reference.
- `attachGuard()` comes after the guard is emplaced (B7).
- `buttonsDownOf()` maps bit i to `ControllerButton` value i (L1 = bit 0 … A = bit 11), through
  `IController::pressed()`, so a `FakeController` drives it on the host.

**Added at the re-check (finding 4), VERBATIM — the ONE place PROS's reads are interpreted** (the header
gains `#include "shulib/ui/touch_panel.hpp"`; `AbortWatch` gains a plain `std::uint64_t paces_`, incremented
FIRST in `pace()`, armed or not, read by `paces()` — the pacing task only):

```cpp
/// One read of the brain's touch screen, reduced: the pages' release-edge sample, the press count
/// (the abort's touch) and whether a finger is on the glass right now.
struct TesterTouch {
    ui::TouchSample sample{};     ///< error, releaseCount, x, y -- the pages' release edge
    std::int32_t pressCount = 0;  ///< PROS's press_count: +1 when a finger LANDS (A4: HA-178 (a))
    bool fingerDown = false;      ///< a good read whose press_count is above its release_count
};

/// PROS's screen_touch_status_s_t, reduced (the one place its five fields are read). A template so
/// the host suite runs it on a struct with PROS's field names and the program on the real one;
/// `touchErrorStatus` is pros::E_TOUCH_ERROR (an error read: every field -1, nothing counts).
template <class ProsTouch>
[[nodiscard]] constexpr TesterTouch testerTouchFrom(const ProsTouch& t,
                                                    int touchErrorStatus) noexcept {
    TesterTouch r{};
    r.sample.error = static_cast<int>(t.touch_status) == touchErrorStatus;
    r.sample.releaseCount = t.release_count;
    r.sample.x = t.x;
    r.sample.y = t.y;
    r.pressCount = t.press_count;
    r.fingerDown = !r.sample.error && t.press_count > t.release_count;
    return r;
}

/// The abort's inputs from one touch read, the master controller and VEXos's disabled flag.
[[nodiscard]] inline AbortInputs abortInputsFrom(const TesterTouch& touch,
                                                 const hal::IController& master, bool disabled) {
    AbortInputs in{};
    in.controllerConnected = master.isConnected();
    in.buttonsDown = buttonsDownOf(master);
    in.touchError = touch.sample.error;
    in.pressCount = touch.pressCount;
    in.disabled = disabled;
    return in;
}
```

### 3.3 One run — `src/programs/auton_tester_run.hpp` (PROS-free; `TesterRunRecord`, `runTesterOnce`, `pacerChainProven`, `stopAfterProgramError` VERBATIM)
Constants `kTesterCountdownSeconds = 3.0`, `kTesterTickSeconds = 0.01`; the header includes
`shulib/diag/fault.hpp`. `ITesterRunObserver` (`countdownStarted(now)`, `runStarted(now)`) as the probe has it.
**Amended at the re-check** (findings 6, 7, 13): the record carries the run's fault counts; ONE try covers the
arm, the countdown, the quiet tick and the run; the handler panic-stops the drive.

```cpp
/// One run, as the tester saw it -- the RESULT page and the log lines are built from it.
struct TesterRunRecord {
    int number = 0;                       ///< run N of this boot (1-based)
    int entry = -1;                       ///< the registry index run
    bool countdownAborted = false;        ///< aborted before anything moved: runAuton never called
    AbortCause cause = AbortCause::None;  ///< the watch's latch at the end (None = nobody aborted)
    std::uint16_t buttons = 0;            ///< the buttons down when a Button cause latched
    bool threw = false;                   ///< something threw between arm and end (a program error)
    char threwWhat[160]{};                ///< what it said
    competition::AutonRunResult run{};    ///< valid unless countdownAborted or threw
    math::Pose2d endEstimate{};           ///< the estimate when the run ended
    bool endLive = false;                 ///< ... and whether it was live then
    double countdownStartedAt = 0.0;      ///< the robot clock at RUN (s)
    int faultsBefore = 0;                 ///< the fault latch's count at RUN
    int faultsAfter = 0;                  ///< ... and when the run ended
    diag::FaultCode lastFault = diag::FaultCode::None;  ///< the last fault DURING the run, or None
};
```

```cpp
/// ONE run (file banner). `chassis` MUST be the Chassis built with `guard` as its pacer, and the
/// guard's inner pacer MUST be `watch` (the COMP brief's B7 layering; pacerChainProven() checks it
/// at boot). `present` is the Robot's PRESENT mechanisms (B9). Fills and returns `rec` (the
/// caller's storage -- never a task stack). Everything from the arm to the end is inside ONE try:
/// whatever throws there (a precondition -- a programming error) is recorded, the drive is put in
/// its safe state, the watch is disarmed, and the tester's task lives on to show it.
inline TesterRunRecord& runTesterOnce(TesterRunRecord& rec, int number, chassis::Chassis& chassis,
                                      sequence::RunGuard& guard, AbortWatch& watch,
                                      const competition::AutonRegistry& registry, int index,
                                      hal::ITelemetrySink& telemetry,
                                      std::span<hal::IMechanism* const> present,
                                      ITesterRunObserver& observer) {
    SHULIB_PRECONDITION(index >= 0 && static_cast<std::size_t>(index) < registry.size(),
                        "runTesterOnce: no such auton");
    const competition::AutonEntry& entry = registry.at(static_cast<std::size_t>(index));
    hal::IClock& clock = chassis.deps().ctx->clock();
    localization::Localizer& localizer = *chassis.deps().localizer;
    const diag::FaultLatch& faults = *chassis.deps().faults;
    rec = TesterRunRecord{};
    rec.number = number;
    rec.entry = index;
    rec.faultsBefore = faults.faultCount();

    try {
        // THE COUNTDOWN: the watch ARMED, one-tick waits that command nothing. Any abort ends it
        // here, and runAuton is never called: nothing moves.
        watch.arm();
        rec.countdownStartedAt = clock.now().value();
        observer.countdownStarted(rec.countdownStartedAt);
        while (clock.now().value() - rec.countdownStartedAt < kTesterCountdownSeconds
               && watch.cause() == AbortCause::None) {
            chassis.wait(units::Time{kTesterTickSeconds});
        }
        if (watch.cause() == AbortCause::None) {
            // ONE QUIET TICK: the RUNNING page is painted whole on a tick that commands nothing,
            // so the run's first tick carries no full repaint, and the watch reads once more.
            observer.runStarted(clock.now().value());
            chassis.wait(units::Time{kTesterTickSeconds});
        }
        if (watch.cause() != AbortCause::None) {
            rec.countdownAborted = true;
        } else {
            // THE RUN: the competition program's call, on the same entry, guard, schedule and
            // mechanisms. No command registry: a ROUTE auton is refused by name (the COMP brief's
            // B30).
            rec.run = competition::runAuton(chassis, guard, entry, nullptr, telemetry, present,
                                            registry.periodFor(entry));
        }
    } catch (const PreconditionError& e) {
        rec.threw = true;
        std::snprintf(rec.threwWhat, sizeof rec.threwWhat, "%.159s", e.what());
        chassis.cancel();  // the PANIC STOP: 0 V + Brake (runAuton's guard already safed on unwind)
    } catch (...) {
        rec.threw = true;
        std::snprintf(rec.threwWhat, sizeof rec.threwWhat, "%s", "an unknown exception");
        chassis.cancel();
    }
    watch.disarm();
    rec.cause = watch.cause();
    rec.buttons = watch.buttonsAtLatch();
    rec.endEstimate = localizer.pose();
    rec.endLive = localizer.qualityClass() != localization::Localizer::Quality::Uninitialized;
    rec.faultsAfter = faults.faultCount();
    rec.lastFault = rec.faultsAfter > rec.faultsBefore ? faults.lastFault() : diag::FaultCode::None;
    return rec;
}
```

**And, for the boot and the task (R13, R14):**

```cpp
/// THE BOOT'S CHAIN PROOF: one idle tick (Chassis::wait commands nothing) must pace BOTH the guard
/// and the watch, or the abort could not reach the motion -- a Chassis built without the guard as
/// its pacer (or on the watch directly) would drive through every button. The program refuses its
/// boot when this is false. Call it before the painter is enabled, from the task that owns the
/// Chassis.
[[nodiscard]] inline bool pacerChainProven(chassis::Chassis& chassis,
                                           const sequence::RunGuard& guard,
                                           const AbortWatch& watch) {
    const std::uint64_t g0 = guard.pacesTotal();
    const std::uint64_t w0 = watch.paces();
    chassis.wait(units::Time{kTesterTickSeconds});
    return guard.pacesTotal() > g0 && watch.paces() > w0;
}

/// A throw reached the tester's task (a programming error): the drive at 0 V + Brake (the panic
/// stop), every PRESENT mechanism to its safe state, the watch disarmed. Never throws: each step
/// is tried on its own, so one that fails cannot skip the rest.
inline void stopAfterProgramError(chassis::Chassis& chassis,
                                  std::span<hal::IMechanism* const> present,
                                  AbortWatch& watch) noexcept {
    try {
        chassis.cancel();
    } catch (...) {
    }
    for (hal::IMechanism* m : present) {
        try {
            if (m != nullptr) {
                m->applySafeState();
            }
        } catch (...) {
        }
    }
    watch.disarm();
}
```

**Also here (PROS-free, golden-tested; the probe has them):**
- `testerResultWord(rec)` — ONE source for the RESULT page's big word and the first log line. First match
  wins: ABORTED (countdown) › PROGRAM ERROR › REFUSED › ABORTED (abandoned) › NOT STARTED › HARD STOP › CUT
  BY DEADLINE › FINISHED (routine OK and end action succeeded) › ROUTINE STOPPED. Each ≤ 18.
- `buttonNamesOf(mask)` — "B", "L1+R2".
- `testerRunLines(rec, entry)` — ≤ 4 lines, each < 200 bytes (TermSink's cut), ASCII:
  1. the word, the outcome, how it ended (in time / cut / `ABANDONED by <cause> <buttons>`), the end action
     (succeeded / FAILED / skipped), `HARD STOP fired`;
  2. scoring ended T+, the end action ended T+ (**"--" when it did not run**), paces, cancels after the cut,
     the **hand-off latency** (`armedAt − (countdownStartedAt + 3.0)`, B12's number), `liveAfter`;
  3. the routine (`OK n/m steps` or `STOPPED at <step name>` plus its stop cause and exit) — or runAuton's
     `why` for REFUSED / NOT_STARTED — plus the faults DURING the run (`faults this run: n, last <NAME> -- see
     the [ERROR] line`);
  4. seated (x, y, heading; the IMU read at the seat) against the END estimate (x, y, heading, LIVE / NOT
     LIVE) — "measure the tape".

  A countdown abort is ONE line: "… ABORTED in the countdown by <cause> <buttons> -- runAuton was never
  called; nothing moved". A throw is ONE line: "run N: '<name>' PROGRAM ERROR -- it threw (a program
  error): <%.120s of what>; the drive was stopped" (re-check: it may now come from the countdown too). Every
  `%s` of a variable gets a precision — the auton's name included (`%.29s`, `kMaxAutonNameChars`); the
  probes hit `-Wformat-truncation` three times without them (the re-check's stop line was the third).
  Line 3's "faults this run" comes from `faultsAfter − faultsBefore` and `lastFault` (never the latch's
  whole-boot count).

### 3.4 The screen — `src/programs/auton_tester_panel.hpp` (PROS-free; interface + rules)
The probe sketch (`src1sim2/programs/auton_tester_panel.hpp`, amended at the re-check) compiles and ran.
**The executor writes the real file from these rules:**
- `TesterPage {Pick, Ready, Countdown, Running, Result}`.
- `TesterFacts` — read by the program each tick: now, the estimate and whether it is live, controller
  connected, the competition status, the watch's cause, SD logging, **`bootFromRecord` (D9), `faultCount`,
  `lastFault`, and `buildStamp`** (the program's `"BUILD " __TIME__`, passed in so the host goldens do not
  depend on the compile time).
- `TesterView` — PICK's `competition::MenuView`, or header, bar tone, ≤ 2 LARGE and **≤ 8 SMALL lines**, and
  ≤ 2 buttons. **`addLine` never drops a line silently:** its cap is `std::size(v.lines)`, and (24) fails if
  any page emits fewer lines than this table specifies. (The first probe's view held 7 and silently dropped
  READY's 8th line — found at the re-check, probe 5.) Eight SMALL lines from y 30 at 16 px end near y 154,
  above the bottom row's y 180.
- `TesterButtonId {Back, Run, Pick, Again}`.
- `TesterAction {None | Run(entry)}`.
- `runAllowed(facts, why*)` (R4).
- `sameButtons(a, b)` (R8).
- `TesterScreen{registry}` with `showPick(releaseCount)`, `showReady(entry, releaseCount)`,
  `showCountdown(now)`, `showRunning(now)`, **`showResult(rec, releaseCount, fingerDown)`** (**takes
  `rec.entry`**; `fingerDown` holds the buttons until that finger lifts — R7), `view(facts)` and
  `touch(sample, drawn, facts)`.

| Page | Says (SMALL 50 / MEDIUM 36 / LARGE 18, every literal `static_assert`ed, every formatted line precision-bounded) | Buttons |
|---|---|---|
| PICK | the AutonMenu's own view (header `AUTON: NONE`, NONE centre, entries by row) | the menu's; choosing an entry ⇒ READY(entry); NONE ⇒ stay |
| READY | bar `AUTON: <name>` (7+29 = 36); EIGHT SMALL lines: (1) `MATCH park T+27.0s stop T+29.0s RELATIVE`; (2) `BOOT RECORD: forward = the end that led Calibrate` (49), or red `BOOT TYPED TABLE: its front may be the OTHER end` (D9); (3) `EST x 0.0 y 0.0 in, facing 0.0 deg LIVE` or `estimate NOT LIVE yet (the IMU is calibrating)`; (4) the gate's reason (R4; `RUN needs the calibration RECORD: run Calibrate` among them); (5) `RUN counts 3 s, then it moves. Step back.`; (6) `ANY BUTTON or a touch of the screen aborts.`; (7) `SD: runs appended to shulib_auton_runs.txt` or `SD: NO CARD -- this boot's runs on serial only` (no `/usd/` on the screen: on a computer the file is at the card's root); (8) `faults: none  BUILD hh:mm:ss` or `FAULT n=<n> last <%.15s NAME>  <%.14s BUILD>` (50 at the worst case, measured) | `< BACK` at `kMenuPrevSlot` ⇒ PICK (fresh menu); `RUN` at `kMenuNextSlot` — armed only by R4 |
| COUNTDOWN | bar amber; LARGE `STEP BACK  3` / `2` / `1`; MEDIUM `ANY BUTTON OR TOUCH ABORTS` | none (the watch owns touches) |
| RUNNING | bar green; LARGE `RUNNING T+12.3`; LARGE `ANY BUTTON ABORTS` (17); once latched `ABORTING: <short cause>`; the live estimate may be shown (same task) | none |
| RESULT | LARGE `testerResultWord()`; six SMALL lines for a run: `aborted by: button B` or `ROUTINE_RAN, in time`; `end action succeeded; scoring T+1.80s, 192 paces`; `START (0.0, 0.0) 0.0 deg`; `END   (39.8, 0.0) 0.0 deg` (with ` NOT LIVE` only when not live — no trailing space, the probe's had one); `faults this run: none` or red `faults this run: 1, last MOTION_TIMEOUT` (from the record's counts); `measure the tape now, then PICK or AGAIN` | `< PICK` bottom-left ⇒ PICK; `AGAIN` bottom-CENTRE ⇒ READY(same entry) — never RUN directly; both grey until a held finger lifts (R7), then 0.5 s |

Every page entry: a fresh `ReleaseEdge` at the given count and a fresh `ArmingClock`. Pages without buttons
ignore touches. `touch()` acts only on the frame drawn (its page equal to the current one). RESULT entered
with `fingerDown`: the first new release is consumed and acts on nothing, and the arming clock restarts on the
next frame (probe 5: armed 0.5 s after the first frame after the lift).

### 3.5 The program — `src/programs/auton_tester_program.{hpp,cpp}` (the flow VERBATIM)
The whole probe file is `src1sim2/programs/auton_tester_program.cpp` (amended at the re-check; zero warnings
under the `src/` warning set with `-Wunused-function`, with and without `SHULIB_BUILD_HASH`, and PASSED the
real gate as the seventh build — §0.4 probe 6).
- The helpers (`rgb`, `printSize`, `paintLine`, `paintButton`, `paintFrame`, `paintText`,
  `ScreenPacer::pace`, `logBoth`, the `TesterState` of empty optionals in borrow order) are written as there,
  with the real refusal painter wrapped like Teleop's (:520-545) — `refuse(headline, why, footer)`, the
  footer defaulting to "Nothing is powered. …" and the stop path passing its own (R14).
  **That painter is a near-copy until the shared refusal/boot words exist.** SAFE-SAVE B2's "one pure
  function" and T4's shared panel gain the tester as a caller (§6); COMP-t does not move Teleop's, because
  `teleop_program.cpp` is T4's in flight.
- `TesterState` holds `std::optional<hal::pros::ProsBlockSink> sdBlock`, `std::optional<hal::BlockCharSink>
  sdChars` and `std::optional<diag::TermSink> sdLog` (no `FILE*`), and the file name is the BARE
  `constexpr const char* kSdRunLogFile = "shulib_auton_runs.txt";` — no `/usd/` anywhere in the file (R9).
- The header declares ONE function, `autonTesterInitialize(beacon, bits)`, with no slot number anywhere.
- **PROS's reads (VERBATIM, re-check finding 4) — each ONE line through the host-tested pure functions:**

```cpp
/// One read of the brain's touch screen: the ONE place PROS's touch struct is read, reduced by
/// the host-tested testerTouchFrom() (programs/auton_abort_watch.hpp).
TesterTouch readTouch() {
    return testerTouchFrom(pros::c::screen_touch_status(), static_cast<int>(pros::E_TOUCH_ERROR));
}

// ═══ THE ABORT'S INPUTS, read from PROS every tick the watch is armed ═══════════════════
class ProsAbortInputs final : public IAbortInputs {
public:
    explicit ProsAbortInputs(const hal::IController& master) noexcept : master_{&master} {}
    [[nodiscard]] AbortInputs read() override {
        return abortInputsFrom(readTouch(), *master_, pros::c::competition_is_disabled() != 0);
    }

private:
    const hal::IController* master_;
};
```

- **The facts the screen reads (VERBATIM):**

```cpp
TesterFacts readFacts() {
    TesterState& st = state();
    const robot::Robot& r = *st.robot;
    TesterFacts f;
    f.now = r.context().clock().now().value();
    f.estimate = r.localizer().pose();
    f.estimateLive =
        r.localizer().qualityClass() != localization::Localizer::Quality::Uninitialized;
    f.controllerConnected = st.master->isConnected();
    f.competitionStatus = pros::c::competition_get_status();
    f.abortCause = st.watch->cause();
    f.sdLogging = st.sdLog.has_value();
    f.bootFromRecord = st.boot.source == calibration::BootSource::UseRecord;
    f.faultCount = r.faults().faultCount();
    f.lastFault = r.faults().lastFault();
    f.buildStamp = kBuildTimeText.data();
    return f;
}
```

- **The flow (VERBATIM):**

```cpp
// ═══ ONE RUN ═══════════════════════════════════════════════════════════════════════════
/// Tells the screen which page the run is on (runTesterOnce calls it).
class ScreenRunObserver final : public ITesterRunObserver {
public:
    void countdownStarted(double now) override { state().screen->showCountdown(now); }
    void runStarted(double now) override { state().screen->showRunning(now); }
};

void runOne(int index) {
    TesterState& st = state();
    const competition::AutonEntry& entry = st.registry->at(static_cast<std::size_t>(index));
    char line[kTesterLineBytes];
    std::snprintf(line, sizeof line,
                  "run %d: '%.29s' (%s, park T+%.2fs, stop T+%.2fs, start %s) -- RUN touched; 3 s "
                  "countdown, the abort armed",
                  st.runs + 1, entry.name, competition::autonKindWord(entry.kind),
                  entry.endActionAt.value(), entry.hardStopAt.value(),
                  competition::startModeWord(entry.effectiveStart().mode()));
    logBoth(hal::LogLevel::Info, line);
    ScreenRunObserver observer;
    const TesterRunRecord& rec = runTesterOnce(
        st.last, ++st.runs, st.robot->chassis(), *st.guard, *st.watch, *st.registry, index,
        st.devices->telemetry(),
        std::span<hal::IMechanism* const>{st.present.data(), st.presentCount}, observer);
    const TesterRunLines lines = testerRunLines(rec, entry);
    const hal::LogLevel level = rec.cause != AbortCause::None || rec.threw
                                    ? hal::LogLevel::Warn
                                    : hal::LogLevel::Info;
    for (int i = 0; i < lines.count; ++i) {
        logBoth(level, lines.line[i]);
    }
    const TesterTouch t = readTouch();
    st.screen->showResult(rec, t.sample.releaseCount, t.fingerDown);
}

/// A throw reached the tester's task (a programming error): stop everything safely, say so on
/// both sinks and the screen, and never touch the Robot again until the program restarts.
void stopTheTester(const char* what) noexcept {
    TesterState& st = state();
    stopAfterProgramError(st.robot->chassis(),
                          std::span<hal::IMechanism* const>{st.present.data(), st.presentCount},
                          *st.watch);
    try {
        char line[kTesterLineBytes];
        std::snprintf(line, sizeof line,
                      "the tester STOPPED on a program error: %.100s -- the drive is at 0 V + "
                      "Brake; restart the program",
                      what);
        logBoth(hal::LogLevel::Error, line);
        refuse("the tester STOPPED (a program error)", what,
               "The drive is stopped (0 V + Brake). Restart the program.");
    } catch (...) {
    }
}

// ═══ THE TESTER'S TASK: everything after boot, for the program's whole life ═════════════════
void testerTask(void* /*unused*/) {
    TesterState& st = state();
    try {
        chassis::Chassis& chassis = st.robot->chassis();
        st.screen->showPick(readTouch().sample.releaseCount);
        st.painter->enable();
        for (;;) {
            // ONE tick: the estimate advances, the health checks run, the frame is painted. The
            // drive is never commanded here (Chassis::wait) -- nothing moves until RUN and the
            // countdown.
            chassis.wait(units::Time{kTesterTickSeconds});
            const std::uint8_t status = pros::c::competition_get_status();
            if (status != st.lastStatus) {
                char bits[64];
                char line[kTesterLineBytes];
                const bool allowed =
                    (status & (kStatusDisabled | kStatusAutonomous | kStatusConnected)) == 0;
                std::snprintf(line, sizeof line, "competition status %s -- RUN is %s",
                              competitionBits(status, bits, sizeof bits),
                              allowed ? "allowed"
                                      : "OFF (start the tester from the brain, nothing connected)");
                logBoth(hal::LogLevel::Info, line);
                st.lastStatus = status;
            }
            const TesterAction a =
                st.screen->touch(readTouch().sample, st.painter->drawn(), readFacts());
            if (a.kind == TesterActionKind::Run) {
                runOne(a.entry);
            }
        }
    } catch (const PreconditionError& e) {
        stopTheTester(e.what());
    } catch (...) {
        stopTheTester("an unknown exception");
    }
    for (;;) {
        pros::delay(100);  // STOPPED: this task never touches the Robot again
    }
}

}  // namespace

void autonTesterInitialize(const char* variantBeacon, const char* competitionStatusBits) {
    TesterState& st = state();
    std::printf("\n[%s] booted: shulib AUTON TESTER build [%s] -- runs ONE of the team's autons\n"
                "[%s] (src/team/autons.cpp) through competition::runAuton, the competition\n"
                "[%s] program's call, when RUN is touched: a 3 s countdown, then the run. ANY\n"
                "[%s] controller button or a touch of the screen aborts. NOT RUN ON A ROBOT YET.\n"
                "[%s] competition status at boot: %s\n",
                kTag, variantBeacon, kTag, kTag, kTag, kTag, competitionStatusBits);
    std::fflush(stdout);

    st.boot = decideBootSource(kTag);
    if (!st.boot.ok) {
        refuse(st.boot.headline, st.boot.why);
        return;
    }
    // Everything is built at the graph boundary, inside ONE try; any refusal un-builds it all.
    const auto unbuild = [&st] {
        st.screen.reset();
        st.registry.reset();
        st.robot.reset();
        st.guard.reset();
        st.watch.reset();
        st.painter.reset();
        st.inputs.reset();
        st.master.reset();
        st.devices.reset();
    };
    try {
        st.devices.emplace();
        st.master.emplace(hal::pros::ControllerId::Master);
        st.inputs.emplace(*st.master);
        st.painter.emplace(st.devices->pacer());
        st.watch.emplace(*st.painter, *st.inputs);
        st.guard.emplace(*st.watch);
        st.watch->attachGuard(*st.guard);
        robot::BuildOptions opts = st.boot.built.buildOptions();
        opts.pacer = &*st.guard;  // BORROWED for the Robot's whole life (robot.hpp)
        const robot::BuildVerdict verdict = robot::validate(st.boot.built.config, *st.devices, opts);
        if (!verdict.ok) {
            unbuild();
            refuse("the library graph REFUSED to build", verdict.why);
            return;
        }
        st.robot.emplace(st.boot.built.config, *st.devices, opts);
        st.registry.emplace(kAutonPeriods);
        registerAutons(*st.registry);  // an entry the registry refuses throws, naming the auton
        // The guard safes the Robot's PRESENT mechanisms (the COMP brief's B9).
        for (std::size_t i = 0; i < st.robot->mechanismCount(); ++i) {
            hal::IMechanism* m = st.robot->builtMechanismAt(i);
            if (m != nullptr) {
                st.present[st.presentCount++] = m;
            }
        }
        st.screen.emplace(*st.registry);
        // THE CHAIN PROOF: one idle tick must pace the guard AND the watch, or no abort could
        // stop the robot. Refused, not warned: a build that cannot abort must not run an auton.
        if (!pacerChainProven(st.robot->chassis(), *st.guard, *st.watch)) {
            unbuild();
            refuse("the tester's pacer chain is WRONG",
                   "one idle tick did not pace the run guard and the abort watch -- the abort "
                   "could not stop the robot; this is a build error, not a robot fault");
            return;
        }
    } catch (const PreconditionError& e) {
        unbuild();
        refuse("the tester REFUSED to start", e.what());
        return;
    }
    attachFaultLatch(&st.robot->faults());

    // The SD run log (A4 register HA-177): ONE file, appended across boots, through the adapter
    // that owns /usd/ and every mode; no card, or a log that will not open = serial only.
    st.sdBlock.emplace(kSdRunLogFile, hal::pros::ProsFileMode::Append);
    if (st.sdBlock->isOpen()) {
        st.sdChars.emplace(*st.sdBlock);
        st.sdLog.emplace(st.devices->clock(), *st.sdChars);
    }
    // The session header (the S18.5 provenance block, serial): the hash LOUD when missing, the
    // port map generated from the config -- teleop_program.cpp's block.
    st.robot->portMap(st.portMap, sizeof st.portMap);
    diag::SessionInfo session{};
    session.buildHash = diag::compiledBuildHash();
    session.routineId = "(auton tester: one per RUN)";
    session.alliance = "";
    session.side = "";
    session.portMap = st.portMap;
    diag::emitSessionHeader(st.devices->telemetry(), session,
                            st.robot->context().battery().voltage());
    // compiledBuildHash() is a string_view: NEVER hand it to %s (the 2026-08-18 data abort).
    const std::string_view built = diag::compiledBuildHash();
    const std::string_view hash = built.empty() ? std::string_view{"MISSING"} : built;
    char line[kTesterLineBytes];
    std::snprintf(line, sizeof line,
                  "boot: shulib AUTON TESTER [%s] hash %.*s, boot source %s, %d autons, %d of %d "
                  "mechanisms present, SD run log %s %.60s",
                  variantBeacon, static_cast<int>(hash.size()), hash.data(),
                  bootSourceWord(st.boot.source), static_cast<int>(st.registry->size()),
                  static_cast<int>(st.presentCount), static_cast<int>(st.robot->mechanismCount()),
                  !st.sdLog.has_value() ? "OFF (no card, or the log would not open)"
                  : st.sdBlock->appendOpen() == hal::pros::AppendOpen::Created ? "ON, CREATED"
                                                                              : "ON, APPENDED to",
                  st.sdLog.has_value() ? st.sdBlock->path() : "");
    logBoth(hal::LogLevel::Info, line);
    for (std::size_t i = 0; i < st.robot->mechanismCount(); ++i) {
        if (st.robot->builtMechanismAt(i) == nullptr) {
            std::snprintf(line, sizeof line,
                          "mechanism '%.23s' is ABSENT -- the run guard cannot safe it: %.120s",
                          st.robot->config().mechanisms[i].name, st.robot->mechanismAbsence(i));
            logBoth(hal::LogLevel::Warn, line);
        }
    }

    st.task.emplace(&testerTask, nullptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT,
                    "auton tester");
}
```

**Checks this flow must keep:**
- The file-level `static_assert`s: the tick equals `ProsTickPacer::kTickMs`; the status bits equal PROS's
  `COMPETITION_*`.
- The run record lives in the static state (`TesterRunRecord` 592 B on the host since the re-check's fault
  fields). What IS on the task's stack, stated rather than hidden: `TesterView` (1216 B) twice in the
  painter's pace (the frame and `view()`'s local), and `TesterRunLines` (804 B) in `runOne` — about 4 KB at
  the worst moment against the task's 32 KiB (`TASK_STACK_DEPTH_DEFAULT`, 0x2000 words).
- A refused boot starts no task, and NOTHING repaints over the refusal: `main.cpp`'s other cases paint
  nothing for this program (C7).
- **(Re-check) The chain proof runs inside the try, after the screen is built and BEFORE the task starts;
  a failed proof un-builds everything and refuses (R13).** The task's whole body is inside ONE try whose
  handlers call `stopTheTester()`, then idle on `pros::delay` for ever (R14). `refuse()` takes an optional
  footer, so the stop screen does not claim "Nothing is powered" (the drive is braked).
- **(Re-check) Every `%s` of a variable carries a precision** (`%.29s` for an auton's name, `%.23s` for a
  mechanism's). Adding `%.23s` to the ABSENT line made GCC see its real bound and flag the old `%.140s`
  (11 + 23 + 45 + 140 > 199) — now `%.120s`, exactly 199 at the worst case. A precision is not decoration:
  it is what lets `-Wformat-truncation` check the line.

### 3.6 The seventh build (COMP brief B29's shape; the probe's diff PASSED the real gate)
- **`src/build_selection.hpp`:**
  - the rule list :8-12 gains `SHULIB_PROGRAM_AUTONTEST`;
  - the one-PROGRAM guard :72-75 gains `+ defined(SHULIB_PROGRAM_AUTONTEST)`;
  - a new guard: `#if defined(SHULIB_PROGRAM_AUTONTEST) && !defined(SHULIB_ROBOT_TANK_2026)` / `#error
    "PROGRAM=autontest needs ROBOT=tank -- the team's autons (src/team/autons.cpp) are robot two's, and only
    robot two boots from a calibration record"`;
  - the enum gains `AutonTester,  ///< "shulib Auton Tester" — runs one of the team's autons from RUN
    (COMP-t)` after `InventedXDrive`;
  - ONE branch before the tank default, `#elif defined(SHULIB_ROBOT_TANK_2026) &&
    defined(SHULIB_PROGRAM_AUTONTEST)`, setting `Program::AutonTester`, `RobotId::RobotTwo`,
    `"shulib-robot-variant=tank-autontest"` and nothing else;
  - the beacon comment's "the other five" (:108) becomes six, and **its prefix list :109-110 gains
    `...=tank-autontest`** (`...=tank` is a prefix of it too — the reason the match includes the NUL);
  - **:141 "the four tank builds share robot two's" becomes five** (re-check finding 11).
- **`src/main.cpp`:**
  - `#include "programs/auton_tester_program.hpp"`;
  - `initialize`: `case Program::AutonTester: shulib::bench::autonTesterInitialize(kVariantBeacon,
    bootStatusBits(bits, sizeof bits)); return;`;
  - `disabled`, `competition_initialize`, `autonomous` and `opcontrol`: `case Program::AutonTester: return;`,
    each with its one-line comment ("the tester's own task paints every state (one painter)"; "NO auton here:
    the tester runs one only from its RUN"; "everything runs on the tester's own task, started by
    initialize()");
  - the table :20-36 gains the row;
  - :30 becomes "Five programs in five slots";
  - **:34 "the four tank builds robot two's" becomes five** (re-check finding 11);
  - **the `opcontrol()` doc :227-234** gains the tester's case ("the Auton Tester returns at once: its own task
    runs everything") (re-check finding 11);
  - :44-48 and :195-199 become "…only in the competition program's autonomous() and, on the bench, the Auton
    Tester's RUN — the same library call (COMP-t) — never in any other program".
- **`Makefile`:**
  - `else ifeq ($(PROGRAM),autontest)` + `ifneq ($(ROBOT),tank)` `$(error PROGRAM=autontest needs ROBOT=tank
    — the team's autons are robot two's; '$(ROBOT)' has none)` + `override
    EXTRA_CXXFLAGS+=-DSHULIB_PROGRAM_AUTONTEST`;
  - the valid-values message :147;
  - the PROGRAM comment block (a fifth entry; "Five programs in five slots");
  - **:121-124** "`drive`, `library` and `calibrate` are only meaningful on robot two" gains `autontest`
    (its reason: the team's autons are robot two's), and **:125-127** "as its fourth, fifth and sixth entries
    (tank-drive, tank-library, tank-calibrate)" becomes "fourth to seventh (…, tank-autontest)" (re-check
    finding 11).
- **`tools/src_build_gate.py`:**
  - `VARIANTS` gains `"tank-autontest"`;
  - `BUILD_ARGS` `("ROBOT=tank", "PROGRAM=autontest")`;
  - `VARIANT_DEFINES` `("-DSHULIB_ROBOT_TANK_2026", "-DSHULIB_PROGRAM_AUTONTEST")`;
  - `VARIANT_BEACONS` `b"shulib-robot-variant=tank-autontest"`;
  - `BUILD_ROBOT` `"RobotTwo"` (no new robot beacon);
  - `BUILD_PROGRAM` `"AutonTester"`;
  - `PROGRAM_ENTRY_SYMBOLS["AutonTester"] = ("shulib::bench::autonTesterInitialize",)`;
  - the docstring's builds table, "per build (…)" :52 and the beacon list :80;
  - **a self-test pair at the next two free numbers** (28/29 at `37706ce`; after any SRC1 adds):
    - 28 — the Makefile's `override EXTRA_CXXFLAGS+=-DSHULIB_PROGRAM_AUTONTEST` line dropped ⇒ FAIL naming
      "whole token -DSHULIB_PROGRAM_AUTONTEST", "beacon … NOT in", and "tank build wearing a tank-autontest
      label";
    - 29 — the real tank-autontest build alone ⇒ PASS (cases 14/15's text, re-pointed).
- **`.github/workflows/ci.yml`** :7-8 ("six since R3d Part 2: three robot variants and the tank
  drive/library/calibrate programs") and :109-112: "seven builds", naming `ROBOT=tank PROGRAM=autontest`.
- The program reads `kChassis` / `kMechanisms` only, through the boot source; it names no per-robot table
  (step 0). `-Wunused-function` is a failure; the probe had none.

### 3.7 The library fix, B8's doc, the host build, the example scan, and the re-check's two additive library pieces
1. **`include/shulib/competition/auton_registry.hpp` :296-298 (§0.5.1).**
   - `char routeWhy[192]` — `path_runner.hpp` :196's own size for the same call.
   - The outer copy becomes `"auton '%s': its route is invalid: %.150s"`.
   - Measured: all seven builds clean. The existing substring test (`auton_registry_test.cpp` :263) still
     passes.
   - `docs/changelog.md` gets a Fixed entry.
   - `docs/api` is regenerated if `check-fresh` says so (the change is a body, not a `///`).
2. **`src/boot_source.hpp` [`common/`]:** the `buildOptions()` doc :259-262 (R10); the host-build sentence
   :42-44.
3. **`test/CMakeLists.txt`:**
   - `target_sources(shulib_tests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../src/team/autons.cpp")`, with a
     comment: the ONE `src/` `.cpp` the host builds, because it is PROS-free and it is what the robot runs;
   - `SHULIB_DOC_INPUTS` (:172) gains the file;
   - the header comment "compiles only the dependency-free core" is re-read.
4. **`tools/api_doc_tool.py` :1808-1811 (D7 (a)) — ONE shape, shared with T4 Part 1a item 6 (re-check
   finding 2).** `EXAMPLE_SOURCE_GLOB` (a string, used at :1868, reasoned at :47) becomes
   `EXAMPLE_SOURCE_GLOBS`, a LIST OF GLOBS — still globs, so the ":47 a gate you must remember to update
   rots" reasoning holds for each kind — each naming ONE KIND of file a public document may quote whole:

   ```python
   # The compiled-example sources are GLOBS, never a hand list of files: a gate you must remember to
   # update when adding a file is a gate that rots (the ARM gate's reasoning). Each glob is one KIND of
   # file a public document may quote whole, and each must match at least one file, so a moved file
   # fails loudly instead of dropping out of the scan:
   #   test/*example*_test.cpp  the compiled examples (C8);
   #   src/team/autons.cpp      the team's auton list (COMP-t): the host suite and every build compile it;
   #   src/team/*_driver.hpp    the team's driver file (T4 Part 1a item 6).
   # Never widened to the rest of src/.
   EXAMPLE_SOURCE_GLOBS = ["test/*example*_test.cpp", "src/team/autons.cpp"]
   ```

   - The scan reads every glob, and the old "no test/*example*_test.cpp found — would pass vacuously" check
     becomes per glob (naming the glob).
   - **Whichever of COMP-t and T4 Part 1a lands first creates the list with its own entry; the second
     APPENDS its glob** (and its comment line), re-reading the list, the self-test and `SHULIB_DOC_INPUTS`.
     The comment above lists all three kinds from the start, so the second change is one list element.
   - Self-test: a line present only in `src/team/autons.cpp` passes; the same line drifted fails; a glob that
     matches nothing fails, naming it.
   - `docs/guide/README.md` (:57-75): "the team's files under `src/team/` that a chapter quotes (today: the
     auton list, `src/team/autons.cpp`), never the rest of `src/`" — the second chunk adds its file to that
     parenthesis. **Never "the one exception" / "the only exception"** (a prose-review grep; T4's item 6
     wording is carried by the coordinator, §6's callers gained).
5. **`include/shulib/sequence/run_guard.hpp` — `RunGuard::pacesTotal()` (NEW, additive; R13).** A
   `std::uint64_t pacesTotal_`, incremented FIRST in `pace()` whether or not a run is live; `[[nodiscard]]
   std::uint64_t pacesTotal() const noexcept`, documented "every pace since construction, run or not (pacesSeen
   counts only a run's) — a program reads it around one idle tick to PROVE its Chassis is paced through this
   guard; the pacing task only". Probe copy: `…/overlay/shulib/sequence/run_guard.hpp` (ARM-compiled clean;
   the gate PASSED with it). `docs/changelog.md` Added; `docs/api` regenerated (a new `///`), `check-fresh`.
6. **`include/shulib/hal/pros/block_sink.hpp` — the Append mode (NEW, additive; D6 (a), R9, HA-177), and
   `include/shulib/hal/block_char_sink.hpp` — `hal::BlockCharSink` (NEW; T4 N5 (a)'s piece).** Probe
   copies under `…/overlay/shulib/hal/` (ARM-compiled clean; host-run in probe 5):
   - `enum class ProsFileMode : std::uint8_t { Truncate, Append };` and `enum class AppendOpen :
     std::uint8_t { Appended, Created, Refused };`
   - `template <class Open, class Close> std::FILE* openForAppend(path, open, close, AppendOpen& how)`: probe
     `"rb"` (existed?), open `"ab"`; on failure create with `"wb"` ONLY if the file was absent; else Refused.
     Every mode in this ONE function; no `+` (HA-166 (3)).
   - The existing `ProsBlockSink(fileName, mountRoot = "/usd/")` DELEGATES to a new `ProsBlockSink(fileName,
     ProsFileMode mode, mountRoot = "/usd/")` with `Truncate` — byte-identical behaviour for every existing
     caller (the blackbox, SAFE-SAVE's `ProsFileStore`); Append calls `openForAppend` with lambdas around
     `std::fopen`/`std::fclose` (so `fopen` stays in this file — SAFE-SAVE's scan allow-list); `appendOpen()`
     reports how it went.
   - `hal::BlockCharSink final : ICharSink` over a borrowed `IBlockSink&`: each `write()` is one block write
     then `flush()`; a refusal of either is dropped and counted (`droppedWrites()`), never thrown — exactly
     T4 N5 (a)'s specification, so T4 Part 3 reuses it (§6's callers gained).
   - The adapter fence: `block_sink.hpp` already includes `pros/misc.hpp` behind the fence
     `test/pros_adapter_fence_test.cpp` enforces; nothing new is included from PROS.
   - `docs/changelog.md` Added (both); `docs/api` regenerated; `docs/faq.md`'s `/usd/` answer (~:118-134)
     gains one sentence: "an append open is the adapter's too (`ProsFileMode::Append`), and it never empties
     an existing file".

---

## 4. OUT, and explicitly rejected
- **OUT:**
  - driving between runs (D4 — after T4 Part 1a);
  - the competition program, its watch, its task and its Station K (COMP-b);
  - the robot console (T4 R7, after G3a);
  - SD streaming of records (D6 (c));
  - a DETAILS page;
  - claw control;
  - any hardware claim (nothing here has run on a robot until Station M's log says so);
  - the slot choice (T4 N4);
  - moving Teleop's refusal painter or fault row into `common/` (T4's files);
  - **a tank arrival rule for `moveTo`, or a steering drive-to-point verb (D10)** — library work, the
    coordinator's plan entry (§6 item 7);
  - **a route auton's command registry** (B30: registered once beside `registerAutons` when the first route
    auton exists).
- **COMP-b must NOT reuse `AbortWatch`.** It aborts on ANY button: in a match, a driver's bumped button would
  end the auton. COMP-b's watch has the competition causes only. It may share the layering and the
  latch-every-tick rule (§6's B30).
- **Rejected:**
  - the UI in `opcontrol()` (R1);
  - the watch outside the guard (R3);
  - an edge-only abandon request (R5);
  - a touch on release (R5);
  - reusing `ProsBlockSink` AS IT IS (it truncates) — it gains an Append mode instead (D6 (a));
  - **a raw `fopen` in the program, or a `/usd/` literal outside the adapter** (re-check finding 12: a
    second owner of the prefix, the absent-file case unhandled, and SAFE-SAVE's scan would fail it);
  - **pins alone for the pacer wiring** (re-check finding 4): the chain is PROVEN at every boot (R13), and
    pinned as well;
  - **a warning, not a refusal, for a typed-table boot** (D9 (b));
  - **a per-auton settle tolerance to make the 40 in "finish"** (D10): `MotionOptions` has none, and loosening
    `translationSettle` robot-wide to hide a tank's drift would hide it from every auton;
  - the students' file as a header of `inline` functions — a `.cpp` is what every VEX team reads, and the
    host build can compile it;
  - a `Tester` enumerator or `PROGRAM=tester` (D1).

---

## 5. Tests (adversarial) and mutations
Host suite counts before and after; every test through the fakes, the PROS shim where an adapter is involved,
and the A2 plant (the `CompRig` shape of `competition_runner_test.cpp` :178-227, with the REAL `AbortWatch`
between `PlantPacer` and the guard).

**`test/autons_test.cpp`** (links `src/team/autons.cpp`):
- (1) `registerAutons` gives exactly 2 entries: names, kind, relative start, schedule 27/29, `endAction`
  null, `routine` non-null, `kAutonPeriods == {30 s, 60 s}`;
- (2) into a registry whose match period is 28 s, it THROWS, and the sentence names `40in forward` (the
  schedule is really checked);
- (3) the 40 in through `runTesterOnce` on the plant: ROUTINE_RAN, routine OK, end action succeeded, not
  cut, not abandoned, `pacesSeen > 0`; the truth AND the record's END estimate x within 0.5 in of 40, |y| <
  0.5, |heading| < 1.2°;
- **(3b) (re-check, finding 3) the 40 in on a plant whose RIGHT side is weak** (a `DegradationModel` scaling
  wheel 1's volts by 0.95, probe 4): `testerResultWord` is ROUTINE STOPPED; the routine stopped at step 1
  (`moveTo`) with exit TimedOut; `faultsAfter − faultsBefore == 1` and `lastFault == MotionTimeout`; END
  |y| > 0.5; the end action ran and succeeded; the drive at 0 V + Brake afterwards. This pins the PREDICTED
  floor result, and that the tester reports it as this run's fault;
- **(3c) (finding 7) a fault latched BEFORE RUN** (raise one on the latch, then a clean run): "faults this
  run" is none and `lastFault` is None — the whole-boot latch never leaks into a run's record;
- (4) the acceptance run ends within tolerance of (0, 0) facing 180°;
- **(4b)** the acceptance run on the 0.95 plant stops at step 1 (`moveTo`), and steps 2–4 are skipped;
- (5) a RELATIVE start after a pre-run drift: the plant placed at (10, −5, 30°) with an IMU offset of 17°.
  The END estimate reads (40, 0, 0) in the start frame while the truth is 40 in along the 30° heading. It
  fails without the re-seat.

**`test/auton_abort_watch_test.cpp`:**
- (6) the `abortCauseOf` truth table: every cause alone and every precedence pair; `touchError` never counts;
  an equal count is None, any other is Touch;
- **(6b) (re-check, finding 4) `testerTouchFrom` and `abortInputsFrom` on a struct with PROS's field
  names**, with `press_count` ≠ `release_count` in every case so a swap is visible: `pressCount` is
  `press_count`, `sample.releaseCount` is `release_count`, x/y copied; `touch_status == error` ⇒ `error`,
  never `fingerDown`; `fingerDown` exactly when a good read has press > release; `abortInputsFrom` takes
  the connection and the buttons from the `IController` (a disconnected `FakeController` ⇒ not connected, a
  pressed B ⇒ bit 9), `touchError` and `pressCount` from the touch, `disabled` as given;
- (7) `buttonsDownOf`: each of the 12 buttons sets its own bit; all 12 give 0x0FFF;
- (8) disarmed ⇒ `pace()` reads nothing (a counting fake) and requests nothing, and paces the inner pacer
  exactly once; **`paces()` counts every pace, armed or not** (R13);
- (9) `arm()` baselines — a touch before arm is not an abort — and clears an old latch;
- (10) **an error read at arm (−1), then a good read, aborts NOTHING, and a real press after it does**;
- (11) the latch holds when the input clears and when a second cause appears; `buttonsAtLatch` is the mask
  at the latch;
- (12) latched with the guard idle ⇒ `abandonRequested()` stays false; latched with a run live ⇒ requested
  on EVERY pace;
- (13) order: the inner pacer is called after the watch's read, once per pace;
- **(13b) (re-check, finding 4) `pacerChainProven`** over three wirings on the plant: Chassis → guard →
  watch → plant ⇒ true; the Chassis built on the WATCH (the guard skipped) ⇒ false; the Chassis on the
  plant alone (built without `opts`) ⇒ false; and none of the three commands the drive (probe 5).

**`test/auton_tester_run_test.cpp`** (`runTesterOnce` on the plant):
- (14) **(re-check, finding 8)** `3.0 ≤ runStarted − countdownStarted ≤ 3.0 + one tick + 1e-9`: the loop
  ends on the first tick whose elapsed time is ≥ 3.0 s, and floating accumulation makes that tick 301 on
  the plant (probe: 3.0100 s) — so "< 3.0 + one tick" was wrong; **then the QUIET tick (finding 13):**
  `armedAt − runStarted` is ≥ one tick and ≤ one tick + 1e-9; a recording pacer below the watch sees every
  drive motor at 0 V on every pace until the guard is running; the plant's travel is exactly 0 at
  `runStarted` AND at the guard's arming;
- (15) B at 1.5 s into the countdown ⇒ `countdownAborted`, runAuton never called (no `runStarted`, no
  `[COMP]` "arming" line, `pacesSeen` 0), travel exactly 0, **and `runTesterOnce` returns within one tick of
  the latch** (re-check: without this bound, mutation 7 — the loop without its cause check — SURVIVED (15):
  the post-loop check still refuses the run, so only the countdown's length shows it);
- (16) a button HELD at RUN ⇒ aborted at the first countdown tick;
- (17) the controller drops in the countdown ⇒ ControllerLost;
- (18) B mid-run ⇒ abandoned, end action skipped, the drive 0 V + Brake, the scheduler idle, the guard
  disarmed; the travel from the latch tick to the cut is ≤ one tick at the commanded speed;
- (19) a touch mid-run ⇒ Touch; a disable mid-run ⇒ Disabled;
- (20) after an aborted run, a second run on the SAME objects is clean;
- (21) a routine that trips a precondition (a test registry) ⇒ `threw`, the drive safe, the watch disarmed,
  and a later run works;
- **(21b) (re-check, finding 6) a throw in the COUNTDOWN** (a pacer layer below the watch that throws 0.5 s
  in, probe 5) ⇒ `threw` with its text, the watch DISARMED, `runStarted` never called, travel exactly 0, the
  word PROGRAM ERROR, the ONE line "… it threw (a program error): …; the drive was stopped"; a later run on
  the same objects is clean;
- **(21c) `stopAfterProgramError`**: the drive at 0 V + Brake (with and without an active motion); each
  PRESENT mechanism's `applySafeState()` called exactly once; the watch disarmed; it does not throw when a
  mechanism's `applySafeState()` throws, and the mechanisms after the thrower are still safed;
- (22) a latch forced BEFORE runAuton (bypassing the countdown's check) costs at most ONE commanded tick
  (B22), as measured in the probe (0.60 in at 60 in/s);
- (23) `testerResultWord` for every record shape; `testerRunLines` goldens for each shape (clean, countdown
  abort, run abort with buttons, cut, floor, refused, not started, threw — in the run AND in the countdown,
  **a ROUTE entry refused by name: "a route auton needs a command registry" (finding 5)**, and the weak-side
  ROUTINE STOPPED with "faults this run: 1, last MOTION_TIMEOUT"), each line < 200 bytes at the worst case
  (a 29-character name, run 99999, extreme numbers), 7-bit ASCII, "--" for an end action that did not run.
**`test/auton_tester_panel_test.cpp`:**
- (24) every literal within budget; every formatted line of every page within budget at the worst case, the
  length measured BEFORE `copyScreenText` so a silent cut FAILS; **and every page emits exactly the lines
  §3.4 specifies (READY 8, RESULT 6 for a run) — a DROPPED line fails** (re-check: the first probe's view
  held 7 and dropped READY's 8th silently);
- (25) the `runAllowed` truth table and its reason precedence, **including `bootFromRecord` false ⇒
  `RUN needs the calibration RECORD: run Calibrate` (D9)**; READY's BOOT line says RECORD or, red, TYPED
  TABLE;
- (26) flows: PICK ⇒ touch entry 0 ⇒ READY(0); BACK ⇒ PICK with a fresh menu (NONE live, the rest greyed
  0.5 s); RUN greyed until the arming delay passes AND R4 holds; RUN's release ⇒ `Run(0)`; RUN touched while
  not allowed ⇒ nothing; RESULT ⇒ AGAIN ⇒ READY(same); RESULT ⇒ PICK;
- (27) **no replay:** the release that chose on PICK does nothing on READY; an abort touch's release arriving
  on RESULT within 0.5 s does nothing; an error read never acts;
- **(27b) (re-check, finding 13) RESULT entered with `fingerDown`:** the held finger's release 2 s later over
  PICK, and (separately) over AGAIN, does nothing; both buttons stay grey until 0.5 s after the first frame
  after that release; then a tap acts. Entered with `fingerDown` false: the plain 0.5 s rule (a release at
  0.6 s acts);
- (28) no new button under the one just pressed, for every transition (hit boxes against the press point);
- (29) COUNTDOWN reads 3/2/1 over time; RUNNING's T+ and `ABORTING: button` once latched;
- (30) `sameButtons`: an armed flip ⇒ false, a text-only change ⇒ true; RESULT takes `rec.entry`.

**The re-check's library pieces (in their own suites):**
- **(31) the append open** (`test/pros_block_sink_adapter_test.cpp`): `openForAppend`'s four branches with a
  scripted opener — existed + `"ab"` ok ⇒ Appended; absent + `"ab"` ok ⇒ Created; absent + `"ab"` fails ⇒
  `"wb"` ⇒ Created; existed + `"ab"` fails ⇒ Refused and **NO `"wb"` call** (the log is never truncated);
  every mode string without `+`. Through the shim on a real temp directory: two constructions in Append
  ("two boots") leave both lines, in order (Created, then Appended); no card ⇒ Refused, nothing created;
  the two-argument constructor still truncates (byte-identical).
- **(32) `hal::BlockCharSink`** (a new `test/block_char_sink_test.cpp`, with `FakeBlockSink`): the bytes verbatim, one block write and one flush per
  line; a refused write or flush is counted in `droppedWrites()` and never thrown.
- **(33) `RunGuard::pacesTotal()`** (in `test/sequence_run_guard_test.cpp`): counts every pace, a run live or not;
  `pacesSeen` unchanged (the existing cases still pass).

**`test/auton_tester_source_test.cpp`** (source pins via `SHULIB_SOURCE_DIR`; what the host cannot compile).
`src/programs/auton_tester_program.cpp`:
- calls `runTesterOnce(` exactly once and never names `runAuton(`;
- **(re-check, finding 4)** contains EXACTLY `st.robot.emplace(st.boot.built.config, *st.devices, opts);` and
  `robot::validate(st.boot.built.config, *st.devices, opts)`; `pacerChainProven(` appears once, inside the
  try, AFTER the robot's emplace and BEFORE `st.task.emplace`;
- `ProsAbortInputs::read()`'s body is exactly `return abortInputsFrom(readTouch(), *master_,
  pros::c::competition_is_disabled() != 0);` and `readTouch()`'s is exactly `return
  testerTouchFrom(pros::c::screen_touch_status(), static_cast<int>(pros::E_TOUCH_ERROR));`;
  `screen_touch_status` appears ONLY there, and `press_count` / `release_count` appear nowhere in the file
  (the pure function owns the fields);
- **(finding 6)** `testerTask`'s body is `try {` … with `catch (const PreconditionError& e)` and `catch (...)`
  each calling `stopTheTester(`, followed by the `pros::delay` loop;
- **(finding 12)** no `fopen`, no `"/usd/` literal, and `ProsFileMode::Append` once;
- names no motion verb (`.drive(`, `.moveTo(`, `.turnTo(`, `.brake(`, `setVoltage(`, `move_voltage`,
  `cancel(`);
- contains `opts.pacer = &*st.guard;`, `st.guard.emplace(*st.watch)`, `st.watch.emplace(*st.painter`,
  `st.painter.emplace(st.devices->pacer())` and `attachGuard(`;
- has `registerAutons(` between the `try {` and its `catch`;
- has ONE `st.task.emplace(&testerTask` with `TASK_STACK_DEPTH_DEFAULT`;
- has no `slot` followed by a digit in any string literal (T4 N4).

**(re-check, finding 9) The pure headers:** `src/programs/auton_abort_watch.hpp`, `auton_tester_run.hpp`,
`auton_tester_panel.hpp`, `src/team/autons.hpp` and `src/team/autons.cpp` include NO `pros/` header (the CI
PROS-free grep covers `include/shulib` only, and the host shim would let `pros/rtos.hpp` or `pros/misc.hpp`
compile).

`src/main.cpp`: the AutonTester case in `disabled` / `competition_initialize` / `autonomous` / `opcontrol`
holds only `return;` and comments.

**What the gate proves (and the source pins must, since the program TU is not host-compiled):**
- step 8: the tank-autontest package links `autonTesterInitialize` and NO other program's entry, and no other
  package links it;
- step 6: the branch (variant beacon) and the robot (robot beacon);
- step 0: the chain and the tables;
- the warning policy: the whole TU, with `-Wunused-function` a failure — and the changed library headers
  (`run_guard.hpp`, `hal/pros/block_sink.hpp`, `hal/block_char_sink.hpp`) are compiled into it; the ARM
  header gate compiles each alone.

What nothing but Station M sees: the painting, PROS's scheduling of the task, HA-177, HA-178, and the chain
proof passing on the real provider's pacer (M.2: the boot is not refused).

**Mutations — each RUN on a fresh binary, observed RED, restored, logged:**
1. The watch requests only on the latch edge ⇒ (12) and (22) red.
2. `abortCauseOf` without ControllerLost ⇒ (6) and (17).
3. **(re-check, finding 4: split — the draft's attribution was wrong)**
   - 3a. Touch compared with `>` baseline+1 ⇒ (6).
   - 3b. `testerTouchFrom` reads `release_count` as the press count ⇒ (6b). ((19) scripts `AbortInputs`,
     so it could never see this.)
4. `arm()` keeps a stale baseline ⇒ (9).
5. `arm()` baselines from an error read ⇒ (10).
6. `arm()` keeps the old latch ⇒ (20).
7. The countdown loop without its cause check ⇒ (15)'s return-time bound (re-check: NOT its travel — the
   post-loop check still refuses the run, so travel stays 0).
8. runAuton called after a countdown abort (inverted `if`) ⇒ (15).
9. `disarm()` moved before runAuton ⇒ (18), (19).
10. `runAllowed` without the controller term ⇒ (25) and (26).
11. READY keeps PICK's edge ⇒ (27).
12. AGAIN placed at RUN's slot ⇒ (28).
13. `showResult` without `rec.entry` ⇒ (30) (the probe threw).
14. The students' 40 ⇒ 44 ⇒ (3).
15. `hardStopAt` 31 s ⇒ (1) throws.
16. `kAutonPeriods.match` 60 ⇒ (1).
17. The program TU calls `runAuton` directly ⇒ the source pin.
18. `main.cpp`'s tester `opcontrol` case calls `runTeleop()` ⇒ gate step 8 FAILS.
19. **`routeWhy` back to 160 ⇒ the src gate FAILS every build** (measured in the probe: bench FAILED on
    `route.hpp:454`).
20. The self-test pair itself: case 28 red with the Makefile line dropped (measured).

**Added at the re-check (each names the test or pin that kills it):**
21. `abortInputsFrom` hard-codes `controllerConnected = true` ⇒ (6b).
22. `testerTouchFrom` drops the error mapping (`error` always false) ⇒ (6b).
23. `ProsAbortInputs::read()` passes `false` for disabled (drops `competition_is_disabled()`) ⇒ the source
    pin — the ONLY guard it has: the host cannot call PROS, and no bench station can disable the robot (the
    team has no competition switch). Stated, not hidden.
24. The Robot emplaced with `st.boot.built.buildOptions()` instead of `opts` ⇒ the source pin; on the robot
    the boot's chain proof REFUSES (the host twin: (13b)'s plant-only wiring).
25. The Robot built with `.pacer = &*st.watch` (the guard skipped) ⇒ the `opts.pacer = &*st.guard;` pin and
    (13b)'s watch-only wiring; on the robot the chain proof refuses.
26. `RunGuard::pacesTotal_` incremented only while running ⇒ (33), and (13b)'s correct wiring reads false.
27. `pacerChainProven` checks only the watch ⇒ (13b)'s watch-only wiring reads true.
28. The quiet tick removed ⇒ (14) (`armedAt − runStarted` below one tick).
29. `runTesterOnce`'s try narrowed back to runAuton alone ⇒ (21b) (the throw escapes, the watch stays armed).
30. `lastFault` read without the count check ⇒ (3c).
31. RESULT ignores `fingerDown` ⇒ (27b).
32. `runAllowed` without the RECORD term ⇒ (25).
33. `addLine`'s cap back to a hard-coded 7 ⇒ (24) (the first probe did exactly this and dropped READY's 8th
    line silently).
34. `openForAppend` falls back to `"wb"` even when the file existed ⇒ (31) (the existed branch calls `"wb"`).
35. `openForAppend` without the `"rb"` probe ⇒ (31) (Appended/Created indistinguishable; the existed branch
    truncates).
36. `BlockCharSink` ignores a refused flush ⇒ (32).
37. `stopAfterProgramError` without its per-mechanism try ⇒ (21c) (a throwing mechanism leaves the next
    unsafed, or the call throws).

---

## 6. Documentation (same change — pace rule 3; prose review logged — rule 4)
1. **Worksheet `R3a-BENCH-WORKSHEET.md` — Station M "THE AUTON TESTER"** (written from the code; quotes
   EXACT). It comes after J; K is COMP-b's and L is T4's.
   - **M.0 before you touch a program:**
     - a charged battery and a FAT32 card;
     - **if you re-run Calibrate first for a true 40 in, G.0.1's rule applies: copy `shulib_drivetrain.bin`
       off the card, dated**, and do it only after SAFE-SAVE B1 is in the build;
     - 3 m clear ahead;
     - a straight tape ≥ 60 in with a START mark, a **40 in** mark and a **43.6 in** mark (today's record,
       §0.2);
     - the controller paired and ON its home screen (HA-128);
     - the robot not touched for 3 s after power-on (the IMU calibrates).
   - **M.1 upload:** `rm -rf bin .d && make ROBOT=tank PROGRAM=autontest && pros upload --slot <per N4>
     --name "shulib Auton Tester"`; start from the BRAIN's screen; READY's `BUILD` stamp is the upload
     check.
   - **M.2 boot:** the serial banner, `boot:` line and session header (boot source RECORD, 2 autons, `SD run
     log ON, CREATED` on a fresh card or `ON, APPENDED to /usd/shulib_auton_runs.txt`), and the first
     `competition status 0x00 …` line (HA-178c); the boot is NOT refused with "pacer chain is WRONG" (the chain
     proof passed on the real provider, R13); PICK shows both autons and NONE (HA-136: write down any line
     that runs off the edge).
   - **M.3 READY:**
     - EST goes LIVE; the BOOT line reads `BOOT RECORD: forward = the end that led Calibrate` (a red `BOOT
       TYPED TABLE` line, or RUN greyed with `RUN needs the calibration RECORD`, is a STOP: do not go on, fix
       the record first — D9);
     - **THE FORWARD CHECK — a STOP condition (re-check finding 1):** push the robot ~6 in toward the end
       you believe is forward (the camera end on today's record). **EST x must RISE.** If it FALLS, stop: the
       record's forward is the other end, and M.6 would drive toward you. Write down which end made x rise;
       that end is "forward" for M.5–M.8. Put the robot back;
     - turn the controller off ⇒ `NO CONTROLLER` and RUN grey; on ⇒ RUN green.
   - **M.4 the aborts, WHEELS UP (on a stand):**
     - **Every touch in M.4 goes on the TOP HALF of the screen** — no page has a button there (re-check
       finding 13);
     - RUN, then B at "2" ⇒ ABORTED, nothing moved, `button B` (HA-178b);
     - RUN, then touch the screen in the countdown ⇒ ABORTED `touch`; press and HOLD to see it abort at the
       touch-down (HA-178a); keep holding until RESULT is up, then lift: PICK and AGAIN stay GREY until 0.5 s
       after the lift (R7) — write down whether they did;
     - **RUN, then switch the controller OFF in the countdown ⇒ ABORTED `controller lost`, nothing moved**;
       switch it back on (HA-178 (d) — note how long after the switch-off the screen changed) (re-check
       finding 4);
     - RUN, let it start, then any button ⇒ the wheels stop within a blink; RESULT reads end action skipped;
       copy the `[ATS]` lines;
     - **RUN, let it start, then switch the controller OFF ⇒ the wheels stop; RESULT `aborted by: controller
       lost`** (finding 4).
   - **M.5 a wheels-up full run:** expected RESULT FINISHED, END x ≈ 40 by the encoders — on a stand the
     robot cannot turn, so no sideways error can build (D10). If it is NOT FINISHED wheels up, stop and copy
     the lines: that is not the tank's drift.
   - **M.6 the floor run on the tape:**
     - the robot on START, **the end that made EST x RISE in M.3 leading**, facing along the tape; RUN, step
       back — to the SIDE, never behind or in front;
     - **the expected result (D10, stated BEFORE the run so nobody is surprised):** `ROUTINE STOPPED`, with
       `faults this run: 1, last MOTION_TIMEOUT`, END x near 40 and END y beyond ±0.5 in, the brake still
       run. `FINISHED` is possible if the drive is even; both are results, neither is a failure of the
       tester;
     - record: the RESULT word, the END estimate (x AND y), the faults line, the TAPE along the line (expect
       ~43.6 in on today's record, ~40 after a good Calibrate rerun), how far off the line to the side, the
       veer, whether it stopped firmly, and anything else;
     - write the numbers, never "about right".
   - **M.7 the SD log (HA-177):** on a card WITHOUT `shulib_auton_runs.txt`: boot ⇒ serial `SD run log ON,
     CREATED`; two runs; a power-cycle ⇒ `ON, APPENDED to /usd/shulib_auton_runs.txt`; one more run; copy
     `shulib_auton_runs.txt` to the laptop. All three runs must be there, in order, across the reboot.
     Write down the two boot words.
   - **M.8 (optional) the acceptance run:** it ends back on the mark facing the other way — or, on the same
     cause as M.6, `ROUTINE STOPPED` at step 1 (moveTo) with steps 2–4 skipped (§0.4 probe 4). Record which.
   - **M.9 the claws:** the claw stays CLOSED through a run and the pivots hold (R12).
   - **Also:**
     - Station A's program list gains the tester (its slot per N4);
     - the DO NOT list: never run the tester on a field or a switch (RUN is refused anyway), and never
       "correct" the 40 to the tape;
     - "Before the helper leaves" gains the card copy.
2. **Guide:**
   - **`docs/guide/16-testing-an-auton-on-the-bench.md`** — new, at the end, the glossary staying 15
     (`docs/guide/README.md` :51). It covers what the tester is and what it is not (§0.1's table in prose); the team's
     file, quoted VERBATIM from `src/team/autons.cpp` (D7); the pages; RUN and the countdown; the aborts and
     the one-tick stop; the RESULT against the tape; the ~8 % note; the SD log; what only a practice match
     shows. No `docs/internal` path.
   - **Chapter 16 also carries (re-check):** "forward" is the end that led the Calibrate push, checked by
     the M.3 push before any floor run (finding 1); **what a tank does with a sideways drift** — `moveTo`
     arrives only within 0.5 in of the point, a tank cannot drive sideways, so the first RESULT is likely
     `ROUTINE STOPPED` / MOTION_TIMEOUT after the time limit (5 s, the robot's PROVISIONAL `defaultTimeout`),
     and why that is the library being honest (finding 3); the SD words CREATED / APPENDED and where the file
     is on a computer (the card's root) (finding 12); touch the top half to abort (finding 13).
   - `docs/guide/README.md`: the table row, and the maintenance paragraph's "the team's files under
     `src/team/` that a chapter quotes (today: …), never the rest of `src/`" (§3.7.4) — never "the one
     exception".
   - `15-glossary.md`: "auton tester", and "abort (the tester's)" if the chapter uses it.
   - `14-what-it-cannot-do-yet.md` and `07-getting-set-up.md`: re-read.
   - `10-the-api.md`: re-read if it mentions registering autons.
3. **`README.md`:** the `src/` list :289-295 — the tester, and `src/team/autons.cpp` "the team's autons" —
   and any program list.
4. **`docs/hardware-assumptions.md`:** HA-177 and HA-178 (R11) in the index table and the body, under a
   "COMP-t (2026-09-27) added TWO" paragraph; the header's status counts; HA-161's Source gains the tester's
   task. Public: evidence cited by date, never by a `docs/internal` path.
5. **`docs/changelog.md`:** the `auton_registry.hpp` fix (Fixed); **`RunGuard::pacesTotal()`,
   `ProsBlockSink`'s Append mode and `hal::BlockCharSink` (Added — §3.7.5-6)**. `docs/api` regenerated
   (the re-check's pieces add `///` docs), then `check-fresh`. `docs/faq.md`'s `/usd/` answer gains the
   append sentence (§3.7.6).
6. **The COMP brief — amendments, dated 2026-09-27 (COMP-t):**
   - **B5:** slot 6. It is T4 Part 0's edit; if Part 0 has not landed, COMP-t makes it in T4's words and the
     coordinator reconciles.
   - **B8:** `buildOptions()`'s doc was reworded by COMP-t (R10).
   - **B18:** the students' file IS `src/team/autons.{hpp,cpp}` (COMP-t): `kAutonPeriods` and
     `registerAutons()`, registering the 40 in auton and the acceptance run; COMP-b calls it, never a second
     file.
   - **B25:** the Makefile's comment becomes "Six programs" at COMP-b.
   - **B29:** COMP-b's is the EIGHTH build; `Program::Competition` goes after `AutonTester`; its self-test
     pair takes the next numbers after COMP-t's.
   - **NEW B30:** what COMP-b takes from COMP-t:
     - the pacer layering and the latch-every-tick rule;
     - the catch shape of `runTesterOnce` (ONE try from arm to end, a panic stop in the handler) and the
       task-level catch with `stopAfterProgramError` (R14);
     - **the boot's chain proof** (`RunGuard::pacesTotal()` around one idle tick, R13) — for its own watch;
     - `kAutonPeriods` and `registerAutons`;
     - **the command registry, when the first ROUTE auton appears: registered ONCE in the students' file
       (e.g. `registerCommands(runner::CommandRegistry&)` beside `registerAutons`), and passed by BOTH
       programs — until then the tester passes `nullptr` and refuses a route auton by name** (re-check
       finding 5);
     - the SD run log through `ProsBlockSink`'s Append mode and `hal::BlockCharSink` (COMP-b may append match
       runs to the same file, its call);
     - the pure-PROS-read pattern (`testerTouchFrom` / `abortInputsFrom`: one line in the program, the
       meaning host-tested) and the source-pin pattern.

     What it must NOT take: `AbortWatch` (buttons abort — §4), and the idle estimate ticking (B19's
     `notify_take` stays).
7. **`build-order.md`:**
   - a COMP-t entry under COMP (parts, DoD, status BRIEFED);
   - the chunk count (count the rows: SRC1 may land first);
   - the first `**Next:**` block names COMP-t beside T4 Part 1a;
   - **a deviations row, "a bench auton tester beside `autonomous()`"**: the team lead's words of 2026-09-27;
     that it does NOT reverse 2026-09-25 (same library path, only the trigger differs); and what stays with
     K;
   - **a FOUND row (re-check finding 3, D10): "`moveTo` cannot arrive on a tank after a sideways drift"** —
     the library's documented behaviour ("laterally-offset targets honestly exit TimedOut"), shown on the
     host test plant from a 3 % weaker side; a tank arrival rule or a steering drive-to-point verb is a
     plan entry for the coordinator (it touches the motion layer; R5's gains do not remove it).
8. **`docs/roadmap.md`:** the COMP / M4 rows — a bench path for the 40 in, not run — and "you are here".
9. **`HANDOFF-remaining-work.md`:** L2 gains COMP-t; Part S gains Station M before K; §7's order.
10. **`RESUMING.md` :114-121:** seven builds, `PROGRAM=autontest`.
11. **`src/team/README.md`** (SRC1's): the autons line.
12. **Comments:**
    - `src/main.cpp`, `Makefile`, `build_selection.hpp`, the gate docstring and `ci.yml` (§3.6);
    - `boot_source.hpp` [`common/`] :42-44, `test/boot_source_test.cpp` :3 and `test/xdrive_config_test.cpp`
      :3 ("never src/*.cpp" — now one PROS-free `src/` file is host-built);
    - `test/CMakeLists.txt`.
13. **`COMP-t-PROGRESS.md`** (first) and **`COMP-t-COMPLETED.md`**; the briefing regenerated LAST.

**Prose review greps** (log each and its hits): `slot 2`, `slot 7`, `four programs`, `Four programs`, `six
builds`, `four tank builds`, `fourth, fifth and sixth`, `never src/`, `one exception`, `only exception`,
`EXAMPLE_SOURCE_GLOB`, `PROGRAM=`, `autonomous()`, `auton hook`, `Teleop hook`, `runAuton`,
`registerAutons`, `acceptance run`, `40 in`, `usd/`, `fopen`, `HA-161`, `HA-177`, `HA-178`, `buildOptions`,
`pacesSeen`, `FINISHED`, `simulator`.

**Callers gained (note them in those briefs, do not edit their bodies):**
- SAFE-SAVE B2 ("one pure function" for the boot source's words);
- **SAFE-SAVE A (re-check, finding 12):** `hal/pros/block_sink.hpp` gains the Append constructor and
  `openForAppend()` — its `"wb"` line moves (the brief cites :91), and its modes are now `"wb"`, `"rb"`,
  `"ab"`, all without `+`; the platform scan's `fopen` allow-list is unchanged (the new opens are in the same
  file). `ProsFileStore` keeps calling the unchanged two-argument constructor;
- L9 Part B (the one-line boot passes `options` WHOLE, `.pacer` included);
- T4's shared panel (the fault row and the refusal painter);
- **T4 Part 1a item 6 (finding 2):** the example scan is ONE list of globs, `EXAMPLE_SOURCE_GLOBS`; T4 appends
  `src/team/*_driver.hpp` (or creates the list, if it lands first) and never calls its file "the one kind"
  or "the one exception"; the guide README sentence is extended, not replaced;
- **T4 N5 (a) (finding 12):** `hal::BlockCharSink` exists after COMP-t, to N5 (a)'s specification; Part 3
  builds only `hal::TeeCharSink` and the provider's argument.

---

## 7. Checks
- The host suite green (counts before and after).
- Every mutation red on a fresh binary.
- `src_build_gate.py self-test` and `check`: seven builds, 0 disallowed, beacons and programs.
- The ARM header gate (it now compiles the changed `run_guard.hpp`, `hal/pros/block_sink.hpp` and the new
  `hal/block_char_sink.hpp` alone).
- The CI greps as they are (the PROS-free grep covers `include/shulib` only; **it says nothing about
  `src/programs/`** — re-check finding 9). The pure `src/` headers' PROS-freedom is the source pin in
  `test/auton_tester_source_test.cpp` (§5).
- `test/pros_adapter_fence_test.cpp` still passes over `hal/pros/*.hpp` (no new PROS include).
- If SAFE-SAVE has landed first: its platform scan passes (no `fopen` outside its three files, no `+` mode).
- `api_doc_tool` self-test / coverage / check-fresh / check-examples / removability.
- `doc_staleness_audit.py`.
- The briefing regenerated LAST and checked.

## 8. Landmines
1. **`-fsyntax-only` is not a compile.** `-Wformat-truncation` lives in the middle end. Build with `make` or
   `-c`; the first probe passed a file that a real compile flagged.
2. **`printText` takes `calibration::TextSize`**, not `ui::TextSize`: map explicitly (a static_assert per
   size, with budgets equal — B20).
3. **`compiledBuildHash()` is a `std::string_view`.** Never `%s` it (the 2026-08-18 data abort); `%.*s`, with
   MISSING when empty.
4. **`PROGRAM=tester` already exists** (Bench Tests). Every new name says "auton tester", never "tester"
   alone.
5. **The guard DROPS a request made while idle** (run_guard.hpp :154-157): request on the latch, every tick.
6. **A touch error read carries −1 in every field.** Never baseline from it.
7. **`AutonMenu` is non-movable with a private edge.** A fresh one per PICK entry, in an `std::optional`.
8. **(Reworded at the re-check, finding 6.) Things CAN throw after the task starts** — a precondition in the
   idle `chassis.wait`, in the painter, in `touch()`, in a formatter. So: `registerAutons`, `validate`, the
   Robot and the chain proof are inside `initialize()`'s try; `runTesterOnce` catches EVERYTHING from its arm
   to its end; and the task's whole body is inside a try whose handlers stop the robot safely and idle for
   ever (R14). A C++ exception must never leave a PROS task function.
9. **PROS leaks every print.** Full repaint only when the buttons change; live text at most 5 Hz; count the
   prints.
10. **No slot number in any code string** (T4 N4); no `docs/internal` path in a public doc (C7).
11. **Two-task sinks.** Only the tester's task logs after boot; `main.cpp`'s cases for this program log
    nothing.
12. **Say "host test plant", never "simulator".** And NO hardware claim: every document says the tester has
    not run on a robot until Station M's log says so.
13. **(Re-check) `pacesSeen` counts only a RUN's paces.** Nothing about the guard's wiring can be learned
    from it before a run; the chain proof reads `pacesTotal()` (R13).
14. **(Re-check) A fixed-size view drops lines silently.** `addLine`'s cap is `std::size(v.lines)`, and the
    test counts each page's lines (the first probe lost READY's 8th this way).
15. **(Re-check) An append open must never become a truncating one.** `"wb"` only after `"rb"` said the file is
    absent; never a `+` mode (the brain refuses read-write, HA-166 (3)); no `fopen` and no `/usd/` in `src/`.
16. **(Re-check) A precision makes `-Wformat-truncation` SEE a line.** Adding `%.23s` to the ABSENT line
    exposed its real 219-byte worst case; size every other precision on the same line against 199.
17. **(Re-check) The first floor RESULT is likely red (D10).** Do not "fix" it in the tester, the auton or
    the settle tolerances; record it (M.6).
