# COMP — the competition program: boot the robot, pick an auton on the brain screen, run it under a guaranteed end, drive

> **THE TEAM LEAD'S ANSWERS TO THE RE-CHECK'S NEW DECISIONS (2026-09-28, ~16:15):** **ND1 — yes, after a safe
> stop:** after a program error on the auton task, the robot is stopped safely, driver control then runs
> normally, and later autons are refused until the robot is rebooted. **ND2 — refuse the auton, not the boot:**
> with no calibration record the competition program refuses every auton, and driver control still runs.
> **The coordinator's (ND3–ND6), as recommended:** ND3 (b) — COMP-b moves the shared pieces COMP-t built into
> `src/common/` as its first step and re-points COMP-t's call sites and pins, logged; ND4 — the auton task paints
> the menu and auton status, the opcontrol task paints the driver panel, the PROS phase tasks paint nothing
> (C4's "menu until the first stick input" dropped); ND5 — yes, match runs are appended to the SD log after
> FINISHED is posted; ND6 — `main.cpp` targets 300 lines, a logged overrun to 320 only after trimming. The team
> lead's go on COMP-b stands (2026-09-28, "i want to do all this"); it is built after COMP-t lands.

> **Paths moved at SRC1 (2026-09-27):** the `src/` files this brief names each moved to a folder, names unchanged — `src/common/` ← `boot_source.{hpp,cpp}`, `screen_text.hpp`, `state_screens.{hpp,cpp}`. `src/main.cpp` and `src/build_selection.hpp` stay where they were. The full mapping is in the [SRC1 brief §1](SRC1-organize-src.md); every include of a `src/` file is now written from `src/` (`#include "robots/chassis_table.hpp"`). The body below is NOT rewritten — it describes the tree it was written against; its executor re-checks every path against the tree.

> **Execution brief.** First written 2026-09-24 at `2780462`; **REVISED the same day at `8ac243f`** after a
> read-only checker compared it with the tree (sixteen findings, the load-bearing ones re-read in the source
> by the coordinator) and the team lead ruled on the two design questions it raised (below). It is the
> handoff's item **L2**. **Two parts, each its own executor run, verification and commit, each on the team
> lead's go, after G1 Part 2b lands:** **COMP-a** the library (host-proven, no `src/`), **COMP-b** the
> program (`src/`, a seventh build) and its documentation. *[superseded 2026-09-28, RC8: COMP-a LANDED
> `b5aaf95`; COMP-b comes after COMP-t (COMP-a → COMP-t → COMP-b → T4 Part 2 → Part 3) and is the EIGHTH
> build.]* **Executor model: `opus`. Do not commit.** Live
> log: `COMP-PROGRESS.md`, created FIRST.
>
> **The team lead's rulings (2026-09-24):**
> 1. **Start heading — "can we do both?": yes.** Each auton declares EITHER a FIELD start pose (x, y,
>    heading, in the field frame) OR "RELATIVE" (wherever the robot is placed becomes (0, 0) facing 0). In
>    both cases the library RE-SEATS the pose — position AND heading — at the moment the auton starts
>    (ruling C2). This also clears whatever the IMU drifted while the robot waited before the match.
> 2. **A mid-auton disable — "our own auton task".** The auton runs on a task the PROGRAM owns and PROS never
>    kills; on a disable or a mode change the program ends the routine itself, cleanly, and safes the robot
>    (ruling C5). Rejected: letting PROS kill `autonomous()` and repairing the library afterwards — the
>    checker showed a killed task leaves the scheduler's active motion dangling on a freed stack
>    (`motion_scheduler.hpp` ~:640-666, :697, :807-808), its in-tick/in-wait flags set, and the guard's
>    `running_` true, so the next phase would call through freed memory.

## Re-check 2026-09-28 — amendments RC1–RC12 (BINDING for part b; they override B1–B30 and the body where they conflict)

> A checker ran a read-only pass over **part b** (the competition program) against `b81a590`. HEAD moved to
> `e124b88` during the pass. That commit changes only the COMP-t brief's predictions, and its §6 amendments to this
> brief, B30 included, are unchanged (`git diff b81a590 e124b88`). What COMP-t will add was checked against the
> COMP-t brief (`docs/internal/chunks/COMP-t-auton-tester.md`), because COMP-t is not built yet. The checker
> returned twelve findings: three major, nine minor, no blocker. **Each one was re-verified in the tree for this
> amendment; all twelve are real** (RC12 is an estimate). The evidence is given with each.
>
> **Probes** (scratch copies only, never the repository; nothing was run on a robot):
> - The checker's `.claude/coordinator/scratch/recheck-2026-09-28/compb/comp_b_probe.cpp` covers B7, B8, B9, B11
>   as replaced, B19's task and the selector against the tree. Its V5 compile was **re-run here** with
>   `arm-none-eabi-g++` 13.2, `common.mk`'s `MFLAGS`/`CPPFLAGS`/`GCCFLAGS`, the Makefile's
>   `-Wall -Wextra -Wformat=2`, `-Wno-psabi`, `-Wunused-function`, gnu++20, `-DSHULIB_ROBOT_TANK_2026` and
>   `-iquote./include -iquote./src`. Result: exit 0, zero diagnostics (`recheck_arm.log`, empty).
> - `.claude/coordinator/scratch/recheck-2026-09-28/compb-fix/comp_b_fix_probe.cpp` is the **amended** shape from
>   RC1, RC3 and RC7:
>   - the idle loop on a finite `task_notify_take`, which logs status changes and never ticks the Chassis;
>   - FINISHED published on every path, the throw path included;
>   - a generic program-error stop with no `AbortWatch`;
>   - a refused-autons flag;
>   - an atomic screen owner;
>   - phase functions that log and never paint.
>
>   Same V5 compile: exit 0, zero diagnostics (`arm_compile.log`, empty).
>
> **Tags used below.** The dispositions are RC1–RC12. The new decisions are ND1–ND6; these names avoid T4's N1–N7
> and COMP-t's D1–D10. Superseded body text is kept and marked *[superseded 2026-09-28, RCn]*.
>
> **Second pass, the same afternoon (the re-check was resumed after an interruption).** Every citation the
> dispositions use was re-read in the tree at `e124b88` (== `b81a590` outside the COMP-t brief). Two things
> changed the first pass's conclusions:
> - **COMP-t's builder is IN FLIGHT**, not waiting. Worktree `.claude/worktrees/comp-t` (branch `comp-t`, from
>   `e124b88`, nothing committed; its log `docs/internal/chunks/COMP-t-PROGRESS.md` there says "start 14:13",
>   "the code written (nothing built yet)"). It has already written `pacerChainProven` and
>   `stopAfterProgramError` typed on `AbortWatch` (`src/programs/auton_tester_run.hpp`, verbatim from COMP-t
>   §3.3) and `testerTouchFrom` beside `AbortWatch`. So ND3's first recommendation ("amend COMP-t's brief
>   before its builder starts") has lapsed; ND3 is re-written below. Read-only observation; that worktree is
>   unreviewed and may change at COMP-t's verification.
> - **The in-flight `src/main.cpp` is 280 lines** (from 256), not the ~276 RC12 estimated. RC12 is re-written
>   and its open end becomes **ND6**.
>
> Both probes were **re-compiled in the second pass** against the tree at `e124b88`, with the same V5 flags:
> exit 0 and zero diagnostics each (`recheck2_comp_b_probe.log` and `recheck2_comp_b_fix_probe.log` in
> `.claude/coordinator/scratch/recheck-2026-09-28/`, both empty). The fix probe is written in ND1 (a)'s form.
>
> Small citation corrections from the second pass are made in place: `runAuton`'s null-mechanism refusal is at
> `match_runner.hpp` :160-166 (:140-146 is the signature); `FlagScope` is at `motion_scheduler.hpp` :933;
> COMP-t's re-predicted numbers span :27-62, not :27-38. RC1's binding text is also split from ND1 (it had
> pre-answered ND1's (c) by making FINISHED unconditional).

### Status at this re-check (RC8)
- **COMP-a LANDED** as `b5aaf95` (2026-09-26). It is host-proven and has not run on a robot. COMP-b must honour
  its deviations (COMP-PROGRESS, "Deviations from the brief" and the review fixes):
  - The selector shows TWO autons per page.
  - B20's budget static_assert lives in the host suite (`test/ui_touch_panel_test.cpp`).
  - `endActionRan` is TRUE when the abandon lands during the end action.
  - `StartPose` has a third state, UNSET. A routine entry must not be UNSET.
  - `AutonRegistry` is constructed with `AutonPeriods` (required).
  - `runAuton` also takes the period, and **REFUSES a null mechanism** rather than filtering (`match_runner.hpp`
    :160-166; the signature is :140-146). The PROGRAM filters the present mechanisms and logs each absent one by
    name (B9).
  - A `NotStarted` run brakes instead of running the entry's end action (COMP-PROGRESS :316-326).
  - `AutonRunResult::armedAt` exists, for B12's hand-off latency.
  - HA-161 was written by COMP-a.
- **The order** (build-order :2025, the team lead's rulings of 2026-09-28): COMP-a (landed) → COMP-t (next;
  after TANK1 Part A, which landed as `3965791`) → **COMP-b** → T4 Part 2 → T4 Part 3. COMP-b takes the students'
  file and the run pieces from COMP-t (B18, B30). B3's order and the header's "after G1 Part 2b" are superseded.
- **Paths (SRC1).** The program is `src/programs/competition_program.{hpp,cpp}`, as T4 :1241 already names it.
  The students' file is COMP-t's `src/team/autons.{hpp,cpp}`. The driver file is `src/team/robot_two_driver.hpp`.
  Shared pieces go per ND3.
- **Builds.** COMP-b is the **EIGHTH build** (`PROGRAM=competition`, beacon `shulib-robot-variant=tank-competition`).
  `Program::Competition` comes after COMP-t's `AutonTester`. The slot is 6 (T4 Part 0).
- **T4.** Part 1b was SKIPPED and folded into Part 3 (the team lead, 2026-09-28). So the driver file's buttons stay
  tied to `src/robots/mechanism_table.hpp` until T4 Part 3 (RC5). T4 Part 2's drive protections land AFTER COMP-b:
  COMP-b's `opcontrol()` runs without them until then, and Station K says so (B11, unchanged).
- **TANK1.** A tank `moveTo` steers back onto its line (API 2.3; TSTR on tick lines). TANK1 Part B (in flight) adds
  ` xt` to tank result lines (RC9).
- **Pending from COMP-t §6 item 6** (applied to this brief at COMP-t's landing, quoted here so the builder reads
  one list):
  - B5: slot 6 (already in).
  - B8: `buildOptions()`'s doc is reworded by COMP-t.
  - B18: the students' file IS `src/team/autons.{hpp,cpp}` (`kAutonPeriods`, `registerAutons()`, the 40 in auton
    and the acceptance run). COMP-b calls it and never writes a second file.
  - B25: the Makefile comment becomes "Six programs" at COMP-b.
  - B29: the eighth build.
  - B30, as amended below by RC1 and RC6.
- **Where the body conflicts, the B-list, B30 and this section win.** In particular §0's table, C1, C4's opcontrol
  clause, C5's `abandon()` and bounded wait, and §3's items 1-4 are superseded where marked.

### Dispositions

**RC1 (major) — REAL. A throw on the auton task must never cost driver control.**

*Evidence:*
- B2 (:45-51) defines FINISHED as the task's "last act after `run()` returned", and says `opcontrol()` waits for it
  with NO upper bound.
- B19 (:170-175) catches and "reports", and says nothing about FINISHED on the throw path.
- B30 (COMP-t :2273-2274) takes "the task-level catch with `stopAfterProgramError` (R14)". R14 (COMP-t :947-954)
  and the tester's flow (:1577-1579, `pros::delay(100); // STOPPED`) idle **for ever** after a throw.
- Composed as written, any throw on the auton task never publishes FINISHED, and `opcontrol()` paints `AUTON
  ENDING` for the rest of the match. The throw can come from:
  - a formatter or a log line;
  - the menu painter, which RC3 puts on this task;
  - `runAuton`'s own rethrow (`run_guard.hpp` :459: "throw ⇒ cancel-all + disarm + rethrow").
- Driver control after the stop is mechanically sound:
  - the guard disarms on the throw (:459);
  - the scheduler is exception-safe (`motion_scheduler.hpp`: `WaitUnwindGuard` :915, `FlagScope` :933, both
    used by the wait loops :740-741, :772-773);
  - so the Chassis is idle and callable.

*What is binding now, and what waits for ND1 (second pass).* The DEFECT is that the throw path publishes
nothing, so `opcontrol()` paints `AUTON ENDING -- driver control waits` for ever, a sentence that is false. That
is fixed whatever ND1 says: **every path out of a request publishes a TERMINAL word** (the program-error stop
first on the throw path), and **no path idles for ever without publishing one**. What `opcontrol()` does with
the throw path's word is ND1:
- ND1 (a)/(b), RECOMMENDED and written below as the default: the word is FINISHED, and driver control runs.
- ND1 (c): the word is STOPPED. `opcontrol()` then paints the program-error screen ("a program error stopped
  the auton; the drive is stopped (0 V + Brake); restart the program") and idles, never `AUTON ENDING`.
The builder builds nothing on the throw path's word until ND1 is answered.

*Amendment (B2, B19, B30), written for ND1 (a):*
- The auton task's body is ONE loop, and each request's work is inside ONE try.
- Both handlers (`PreconditionError`, `...`) do three things:
  1. run the **program-error stop**: `chassis.cancel()` (0 V + Brake), then `applySafeState()` on every PRESENT
     mechanism. Each step is tried on its own and none throws. This is COMP-t's `stopAfterProgramError` without
     the watch step; its home is ND3.
  2. log one `Error` line naming what threw;
  3. request the fault screen (RC3's owner paints it).
- **Then, on EVERY path** (normal return, refusal, abandon, throw):
  - `menu.unlock()` (its doc: "only after the auton it locked for has FINISHED", `auton_menu.hpp` :274-277);
  - **FINISHED is published**.
- **The task never idles for ever.** B30 takes R14's STOP, not R14's idle. A C++ exception never leaves the task
  function (COMP-t landmine 8).
- Whether a later `autonomous()` on the same boot runs is **ND1** (the team lead's).

*Tests:* the request body is a PROS-free function in the program's pure header, so the host can reach it; the
builder names it.
- (a) A throw OUTSIDE `runAuton` still publishes FINISHED, leaves the drive at 0 V + Brake and the present
  mechanisms safe. Examples: a throwing log sink, or a throwing formatter on the request path.
- (b) A throw inside a routine (the guard's rethrow) does the same.
- (c) A waiter shaped like `opcontrol()`'s proceeds in both cases (under ND1 (c): it sees STOPPED and paints the
  program-error screen, and never `AUTON ENDING`).
- Mutation: FINISHED (or, under ND1 (c), STOPPED) not published on the catch path ⇒ (a)–(c) red.

The compb-fix probe carries this shape.

**RC2 (major) — REAL. The HA number is HA-162, reserved. It is not "next free".**

*Evidence:*
- HA-161 is COMP-a's task-kill belief (`docs/hardware-assumptions.md` :271, settled "at COMP-b's practice-match
  station (K)").
- The register holds "HA-162..HA-178 … numbers other chunks hold — never reissued" (:84).
- COMP-PROGRESS reserved HA-161/162 for COMP (:14), left HA-162 unused, and says "the Timed Run HA (B5) is COMP-b's"
  (:254).
- L9 :28, SAFE-SAVE :1002 and DIAG1-partB :768 cite COMP-b as HA-162.
- The next free number today is HA-181, because TANK1 took HA-179 and HA-180.

*Amendment:*
- B5(2) and B23-B27's HA sentence now read: **"HA-162 ONLY, reserved: the Timed Run entry, and the only one. A
  second hardware belief ⇒ stop and tell the coordinator."**
- HA-161's *Source* gains the competition program's auton task (`src/programs/competition_program.cpp`), beside the
  tester's task, which COMP-t §6 item 4 adds.
- C5's "Station H settles it" becomes **Station K**.

*Also found (not a checker finding):* HA-161's *Settle* (register ~:1983-1988) describes a probe program that
COMP-b does not build: an `autonomous()` holding an object whose destructor logs, plus "alive N" lines. COMP-b's own
B5(1) lines can settle it without a probe build. Keep the auton running at the mode change, then read the log:
- (a) is confirmed if `autonomous()`'s "normal return" line NEVER appears. The abandon publishes FINISHED within a
  tick, so an `autonomous()` that PROS did not delete would return and log that line.
- (b) is confirmed if the auton task's status-change lines continue through the change.

COMP-b rewords HA-161's Settle to this (the claim is unchanged), and Station K's HA-161 step reads exactly those
lines. If the builder finds that the lines cannot tell the cases apart, stop and tell the coordinator.
*Second pass, two limits the step must state:*
- The routine must still be RUNNING when the mode changes. The acceptance run lasts ~6 s and the 40 in ~2 s
  (COMP-t :27-62, host test plant), so this rides on K's mid-auton-disable step (B17), not on an ordinary run
  that ended before the change. If `autonomous()` returns once FINISHED is published, an auton that finished
  early logs a normal return legitimately.
- Every one of these lines carries `millis`. A normal-return line stamped within a tick of the status-change
  line is AMBIGUOUS (PROS's deletion and the watch's abandon race), not a refutation of (a). Station K records
  it as such and settles nothing on it.

**RC3 (major) — REAL. B19's idle contradicts B5(1), and no phase has a named painter.**

*Evidence:*
- B5(1) (:59-60) logs every status change "from the auton task's idle loop". B19 (:171-172) idles on
  `notify_take(true, TIMEOUT_MAX)`, so there is no loop to log from. B30 keeps "B19's `notify_take`" (COMP-t
  :2286-2287).
- Station K's first step (B5(4)) reads those lines.
- The menu's contract (`auton_menu.hpp` :190-191): `view()` and `touch()` belong to ONE task.
- C4 (:272-275) instead gives the menu to the PROS phase tasks: two tasks, which PROS deletes on a mode change.
- B20 (:176-178) asks for one painter. B11's page (:111-112) has no menu.

*Amendment:*
- **B19:** the idle wait is `notify_take(true, <one tick, 10 ms>)` inside a loop. The loop:
  - (i) logs every competition-status change (B5(1): millis plus `competitionBits()`);
  - (ii) polls and paints the menu while it holds the screen;
  - (iii) **never ticks the Chassis.** COMP-b's estimate is first ticked inside `runAuton`'s live-wait (COMP-t
    :548-549), so B30's "no idle ticking" holds.

  The status log's latency is one tick, stated.
- **B20:** the painter for each phase is **per ND4** (recommended there and written here as the default for the
  coordinator to confirm). A task paints only while it holds the atomic screen owner, and ownership changes hands
  by compare-exchange.
  - **After the boot, and in `competition_initialize()` and `disabled()`:** the AUTON TASK polls `touch()` and
    paints `view()`.
  - **In `autonomous()`:** the auton task paints the status before `runAuton` and the result after. During a run
    nothing paints, unless the builder puts a painter in the pacer chain as COMP-t does; the builder says which.
  - **In `opcontrol()`:** the opcontrol task paints the driver panel (B11) from the moment it takes the owner.
    That is after FINISHED, or when B2's constant passes and it paints `AUTON ENDING -- driver control waits`. In
    the second case the auton task's result paint is skipped and the result is only logged.
  - **When the driver loop returns on a disable:** the owner goes back to the menu.
  - **The PROS phase tasks** (`competition_initialize()`, `disabled()`, `autonomous()`) **paint nothing.** They log
    B5(1)'s lines and hand the owner over. PROS deletes them mid-statement on a mode change (HA-161 (a)).
- **B2:** "`disabled()` commands nothing and only paints" becomes "commands nothing and **paints nothing**".
- **A refused boot (C7):** no auton task exists. The refusal that `initialize()` painted stays, because nothing
  paints after it. B5(1)'s status-change lines are then absent and the phase lines remain; say so in the guide and
  at Station K.
- **C4's** "`opcontrol()` … the menu shows until the driver's first stick input" is **ND4**.

*Tests:* the owner handover is a small PROS-free state machine, host-tested:
- a paint from a non-owner is refused;
- the handovers happen in the order above.

**RC4 (minor) — REAL. B29 names every place the eighth build must be named.** B29 replaced §3 item 3, which was
the only place that named the Makefile. Without a branch, the Makefile's :163 `$(error)` refuses
`PROGRAM=competition`. **The checklist is COMP-t §3.6** (its line numbers are at `37706ce`; find each line by its
quoted text). For COMP-b:
- **Makefile:**
  - `else ifeq ($(PROGRAM),competition)`, then `ifneq ($(ROBOT),tank)` `$(error PROGRAM=competition needs
    ROBOT=tank — …)`, then `override EXTRA_CXXFLAGS+=-DSHULIB_PROGRAM_COMPETITION`;
  - the valid-values message (:163);
  - :134 "Four programs" (becomes Six, B25);
  - :137 "only meaningful on robot two" gains `competition`;
  - :142 "fourth, fifth and sixth" (becomes "fourth to eighth").
- **`src/build_selection.hpp`:**
  - the rule list (:8-12);
  - the one-PROGRAM guard (:72-75);
  - a needs-tank guard like :82;
  - the enumerator after `AutonTester`;
  - the one branch;
  - the beacon comment: :108 "the other five" (becomes seven) and its prefix list :109-110 (gains
    `...=tank-competition`);
  - :141 "the four tank builds" (becomes six).
- **`src/main.cpp`:**
  - the include;
  - the table (:22-37), "Four programs" (:33) and "the four tank builds" (:37);
  - **:49-51 and :198-202**, "autonomous() commands NO motion in ANY build". This goes false. Reword it: *every
    build's autonomous() commands no motion except the competition program's, which runs the chosen auton on the
    program's own task through `competition::runAuton`. The Auton Tester runs the same call from its RUN*;
  - the `opcontrol()` doc (:230-237).
- **`tools/src_build_gate.py`:**
  - the docstring's builds table, :52 and :80;
  - **:173-174**, "COMP's will be the sixth". After COMP-t, `AutonTester` is the sixth enumerator and `Competition`
    the seventh;
  - the tables, and `PROGRAM_ENTRY_SYMBOLS["Competition"]`;
  - the self-test pair after COMP-t's. SRC1 took 28-30, and COMP-t takes 31/32, so COMP-b's is **33/34**.
    Re-count at start.
- **`ci.yml`** :7-8 and :110-113, and **`RESUMING.md`** :116-117 and :134-136: eight builds.

After COMP-b the counts are **8 builds, 7 `Program` enumerators, and 6 programs in slots** (B25).

**RC5 (minor) — REAL. B11 after the 2026-09-28 ruling; its documentation list is incomplete.**
- **The tie.** "Until T4 Part 1b" becomes **"until T4 Part 3"**. Part 1b was skipped and folded into Part 3 (T4
  brief :15-21; build-order :2021). The tie is `controlsMatchTable` (`teleop_program.cpp` :186, defined at
  `teleop_claws.hpp` :218), and it retires with Teleop at Part 3. COMP-b still edits neither copy.
- **The claw boot line.** The reword of `-- in shulib Teleop only` (`teleop_claws.hpp` :297) carries:
  - its three word pins, `test/teleop_claws_test.cpp` :487, :490 and :789, changed **with a log** (T4 landmine 1);
  - J.3.1's quotes (worksheet :1306-1307).

  The new wording must be TRUE FOR BOTH programs. Then, if COMP-b prints its claw boot lines through the same
  function, no second change is needed.
- **Documentation list additions** (sentences that go false when COMP-b runs the driver file):
  - `robot_two_driver.hpp` :5-7 (already named);
  - `docs/cookbook/06-driver-control.md` :268;
  - `docs/guide/07-getting-set-up.md` :125-126;
  - `src/team/README.md` :7-8;
  - `docs/roadmap.md` :353;
  - build-order's T4 status lines (:632's "NO PROGRAM RUNS IT YET", and :1479);
  - build-order's COMP DoD :1501. "a seventh build" becomes the eighth; "a separate autons file in `src/`" becomes
    COMP-t's `src/team/autons.*`; "Station H" becomes K, twice.

**RC6 (minor) — REAL. B30's pieces are typed on the tester's `AbortWatch`, and B11's "never copied" rule gives
them no home.**

*Evidence:*
- `pacerChainProven(chassis::Chassis&, const sequence::RunGuard&, const AbortWatch&)` (COMP-t :1333-1335) and
  `stopAfterProgramError(…, AbortWatch&)` (:1345-1347) are in `src/programs/auton_tester_run.hpp`.
- `testerTouchFrom` sits beside `AbortWatch` in `src/programs/auton_abort_watch.hpp` (:1205). COMP-t §4
  (:1869-1871) forbids COMP-b to reuse `AbortWatch`.
- Teleop's refusal painter (`teleop_program.cpp` :530-568) is PROS-bound (`printText`), so it cannot go into a
  PROS-free `driver_panel.hpp`. COMP-t already carries its own `refuse()`, because moving Teleop's painter is OUT
  for COMP-t (§4).
- `clawRowText`/`faultRowText` stay in `programs/teleop_claws.hpp` until T4 Part 3 moves them (T4 :1319-1320). No
  gate rule forbids another program including that header; the checker's probe did, and compiled.

- *Second pass:* COMP-t's in-flight worktree already holds these pieces as its brief lists them
  (`auton_tester_run.hpp` :161-167 and :173-188; `auton_abort_watch.hpp` :117-129), and its log says its
  refusal painter is "wrapped like Teleop's" — a second copy of Teleop's, as COMP-t §4 allowed.

*Amendment:* the home of each piece is **ND3**. Until it is ruled, the builder copies nothing.

**RC7 (minor) — REAL. Do not build the phase screens on `benchStateScreen`.**

*Evidence:* `benchStateScreen` (`src/common/state_screens.cpp`):
- calls `screen_erase()` (:31), which would wipe the menu or a refusal;
- prints "This program runs ONLY in DRIVER CONTROL, enabled." (:36) and "Unplug any competition switch…" (:42),
  both false here;
- logs under `[R3A]` (:25);
- bypasses `printText` by design (`state_screens.hpp` :27-33), against B20.

*Amendment (B3):*
- COMP-b reuses ONLY `competitionBits()`.
- The Competition cases in `main.cpp`'s `disabled()` and `competition_initialize()` forward to the program's own
  phase functions. Those functions log B5(1)'s line and hand the owner over (RC3).
- `benchStateScreen` is never called for this program.

**RC8 (minor) — REAL. The body is stale.** The status block above covers COMP-a, the order, the paths and the
builds. The body is corrected where marked:
- the header's "a seventh build … after G1 Part 2b";
- §0's rows (the chassis takes `options.pacer`, `robot.hpp` :562; the driver loop is the driver file's; HA-161
  exists);
- C1's `RunGuard guard{devices.pacer()}` becomes B7's `guard{watch}`;
- C5's `RunGuard::abandon()` and its "bounded, ≤ 100 ms" wait become B1's `requestAbandon()` and B2's unbounded
  wait;
- §3 items 1-4 (paths, the students' file, slot 6 not 5, Station K);
- landmine 6 (K);
- §5 ("button bindings (T3)" becomes the driver file, T4).

Two more fixes:
- **B6:** "no auton is ever started from `opcontrol()` or another program" gains **"except the Auton Tester, which
  runs one through the same `runAuton` path from its RUN (the team lead, 2026-09-27; COMP-t §0.1)"**.
- **B8:** "`st.boot.buildOptions()`" becomes **`st.boot.built.buildOptions()`** (`BootOutcome::built`,
  `boot_source.hpp` :370; the door named at :50). It is not in the checker's list; the checker's probe already
  spells it this way.

**RC9 (minor) — REAL. Station K comes after COMP-t's Station M, and after TANK1.**
- **B4:** K runs only after F, J **and M** have passed on the same build line:
  - M.3's forward check passed. It is the only station that proves "forward" before an auton drives (COMP-t
    :2157-2160).
  - M.6's floor result is recorded, whatever it was.
- **B17:** K's expected results cite **COMP-t's re-predicted table and its thresholds** (COMP-t :27-62), as
  host-test-plant numbers:
  - the acceptance run FINISHES down to ×0.724 on a weak side;
  - at robot two's ~9 %: FINISHED (0.37, 0.06, 177.28°) at T+6.14 s;
  - the 40 in finishes to ×0.723.

  K is anchored on M.6/M.8's measured result, and quotes TSTR and ` xt` (TANK1 Part B) **as the code prints them
  at COMP-b's merge**.
- **B5(3):** add **J.9.6** (worksheet :1496-1503) to the places that re-read the rule. J.9.6 stays "forbidden" for
  slot 2: the exception is slot 6 at Station K only.

**RC10 (minor) — REAL. B21's targets have moved.**
- **HANDOFF :617:** "impossible until L2 (or a minimal auton hook) exists; R3b closes on that later run" becomes:
  *"the first library-commanded `moveTo` runs at COMP-t's Station M.6 (the auton tester, through the same
  `runAuton` path); R3b and M1 close at S8 (a student-authored routine through the competition program)."*
  - S8 (HANDOFF :632-634) is unchanged.
  - Whether M.6 alone may close R3b is not decided here.
- **HANDOFF L2 (:345-352), beyond B21's two strings:**
  - "`opcontrol()`: `runDriverControl` … and the bindings (L5)" becomes the driver file (B11 as replaced);
  - "`disabled()`/`competition_initialize()`: state screens" becomes the program's own screens (RC3, RC7).
- **build-order :640's "minimal auton hook"** is inside a block "kept as history". Leave it unedited.

**RC11 (minor) — REAL. The guide chapter's number and example rule.**
- The chapter is **17**. COMP-t's chapter 16 comes first, and the glossary stays 15 (guide README :49-51).
- Listings about registering an auton quote `src/team/autons.cpp` **live, as unbroken runs**. COMP-t appends the
  file to `EXAMPLE_SOURCE_GLOBS` and `WHOLE_RUN_SOURCE_GLOBS` (`api_doc_tool.py` :1831, :1853). Those listings
  **link to chapter 16** rather than re-explaining the file.
- Program-only examples still compile in `test/guide_examples_test.cpp`: the start modes, the schedule, the
  selector, the refused boot.
- §3 item 4 is amended to match.

**RC12 (minor) — REAL, and tighter than the checker estimated. `main.cpp` ≤ 300 is likely to be crossed.**
- `main.cpp` is 256 lines at `b81a590`.
- *[first pass: COMP-t adds about +20, giving about 276 — superseded by the measurement below]* **Measured
  (second pass):** COMP-t's in-flight `src/main.cpp` is **280** lines (+24: an include, a three-line table row,
  "Five programs", a case in each of the five switches, four of them with a one-line comment, and three more
  lines in the `opcontrol()` doc). It is unreviewed and may move at COMP-t's verification.
- COMP-b adds the same shape: an include, a table row, and five forwarding cases. Its Competition cases in
  `disabled()`, `competition_initialize()` and `autonomous()` cannot share the tester's `return` (RC3/RC7: they
  forward to the program's phase functions). Its `autonomous()`/`opcontrol()` docs are reworded (RC4). That is
  about +18-24, giving about **298-304**.
- The bound is G1 Part 2's target, "stated so 'cleaner' is a number" (G1 brief :94, DoD item 5 :429). The gate
  does not enforce it (`tools/src_build_gate.py` has no line count). What it protects is enforced separately:
  G1 item 5's grep (no `MotorGroup`, `Localizer`, … in `main.cpp`).

*Amendment (B3):* the builder counts `main.cpp` at start (after COMP-t) and at the end, keeps every case comment
to one line, and moves prose to the program's header. What happens if it is still over 300 is **ND6** (it was "stop
and tell the coordinator" in the first pass; a stop the brief can predict is better ruled before the run).
Raising G1's bound is not the builder's call.

### Decisions (new, 2026-09-28)
None of these was ruled before. Each has a recommendation. Nothing is built on an open one.

- **ND1 (the team lead) — after a program error on the auton task, does driver control still run? And do later
  autons on the same boot?**
  - **(a) RECOMMENDED:** driver control runs, after the program-error stop and FINISHED (RC1). Later
    `autonomous()` calls on this boot are REFUSED, with a sentence on the screen and in the log: *"restart the
    program"*.
    - Why driver control: an auton bug must not cost the match's driver period. The Chassis is idle and callable
      after the stop (RC1's evidence).
    - Why refuse later autons: after an unknown throw the program's own state is unproven.
  - **(b)** Driver control runs, and later autons run too. A deterministic bug throws again and is caught again,
    safely.
  - **(c)** R14 as written: the program stops for the boot. The robot is dead for the rest of the match. Even
    then the task publishes STOPPED, and `opcontrol()` paints the program-error screen rather than `AUTON
    ENDING` (RC1's binding part).
- **ND2 (the team lead) — does COMP-t's D9 apply to the competition program?** D9 said RUN only from the
  calibration RECORD, because the typed table's front is the 15/16 end, not the record's camera end.
  - **RECOMMENDED:** yes. `autonomous()` REFUSES the auton (not the boot) when the boot came from the typed table.
    Driver control still runs, and the screen and log say why.
  - It costs nothing today: robot two's table has no track width, so it cannot boot from the table (COMP-t D9).
  - **(b)** Run it with a red warning.
- **ND3 (the coordinator) — where the shared pieces live.**
  - *[first pass, lapsed — COMP-t's builder started at 14:13 and has written the pieces verbatim (second pass,
    header):]* ~~RECOMMENDED (a), if COMP-t's builder has not started: amend COMP-t's brief now so it builds the
    generic halves in `src/common/`, PROS-free … COMP-t's VERBATIM listings and source pins then name those
    files.~~
  - **(a′)** A course correction to COMP-t's in-flight builder now, before its verification. It changes §3.3's
    VERBATIM listings mid-run, so COMP-t's brief, pins and tests must be re-read for the change as well, and it
    adds a design change to a run that is already writing code. Not recommended.
  - **(b) RECOMMENDED now:** COMP-b does the move, at the second consumer, as the first step of its run and with
    a log. It builds the generic halves in `src/common/`, PROS-free:
    - the chain proof, templated over any watch with `paces()`;
    - `stopAfterProgramError(chassis, present)` with no watch argument (the tester keeps a one-line wrapper
      that also disarms its own watch);
    - the touch reducer under a neutral name returning `ui::TouchSample` plus the press count (the tester's
      `TesterTouch` stays the tester's).

    COMP-t's call sites and source pins are re-pointed, each change logged. That is about +¼ day for COMP-b, on
    files COMP-t has landed and verified.
  - **(c)** Near-copies, against B11's "never copied". Not recommended.
  - **The refusal painter:** a PROS-bound `src/common/` pair (a PROS-free header and a PROS-bound `.cpp`, as
    `state_screens` is). COMP-b and the tester use it (under (b), COMP-b re-points the tester's copy, which its
    log says is "wrapped like Teleop's"). Teleop keeps its own until T4 Part 3 deletes Teleop, so Teleop's
    pinned file is not touched. That copy retires with Teleop; say so.
  - **The claw and fault rows:** they stay in `programs/teleop_claws.hpp` until T4 Part 3 moves them (T4
    :1319-1320). COMP-b includes that header as an interim step, stated in both files.
- **ND4 (the coordinator) — who polls and paints in each phase, and C4's `opcontrol()` menu.**
  - **RECOMMENDED:** the assignment written into B20 by RC3.
    - The auton task paints the menu in every non-driver phase, and the auton status in `autonomous()`.
    - The opcontrol task paints the driver panel after the handover.
    - The PROS phase tasks paint nothing.
  - **Drop C4's "menu until the driver's first stick input".** B11's page has no menu. B6 forbids starting an auton
    from `opcontrol()`. With nothing attached, `autonomous()` never runs, so a choice made there is unused. A field
    or a Timed Run brings the robot up disabled first (HA-128), and the menu is up then.
  - The auton pick under a Timed Run stays B6's: the team lead rules after Station K measures it.
- **ND5 (COMP-b's call, per B30) — does COMP-b append match runs to `shulib_auton_runs.txt`?**
  - **RECOMMENDED:** yes, through COMP-t's Append mode and `BlockCharSink`, which drop and count and never throw.
  - **Binding either way:** the auton task writes only AFTER it has published FINISHED. The write must never delay
    driver control.
  - Host test: FINISHED is published before the first block write; a fake block sink records the order.
  - Mutation: the write moved before FINISHED ⇒ red.
- **ND6 (the coordinator) — `main.cpp` over G1's 300-line target (RC12).** Measured: 280 lines in COMP-t's
  in-flight tree; COMP-b's dispatch adds about +18-24.
  - **RECOMMENDED:** keep 300 as the target, and allow COMP-b a stated overrun up to **320**, logged with its
    reason, only after the trimming rules have been applied (one-line case comments; prose in the program's
    header). The reason: every program costs about twenty lines of dispatch (an include, a table row, five
    three-line cases; COMP-t's cost 24), and the bound's purpose, no robot logic in `main.cpp`, is held by G1
    item 5's grep, which stays. The new number goes into the G1 brief's :94 as a dated note. T4 Part 3 retires
    Teleop's cases, which gives some of it back.
  - **(b)** 300 stays hard. The builder moves `main.cpp`'s per-program prose (the "WHAT RUNS" list) into the
    program headers until it fits. This loses the one place a reader sees every program's status.
  - **(c)** First-pass behaviour: the builder stops and asks mid-run.

### The honest size of COMP-b now
- **Scope:** one executor run, the largest of COMP's parts:
  - the program: task, watch, phase functions, menu painter and screen owner, B5 logging, the program-error path,
    refusals;
  - the eighth build across six files;
  - host tests: the watch against a fake status source, RC1's throw cases and mutation, the owner state machine,
    source pins, and the self-test pair 33/34;
  - the documentation: guide chapter 17, Station K (the Timed Run measurement, HA-161's settle from the program's
    own lines, B28's claw step, the mid-auton disable), HA-162, B21's held pass, and RC5/RC10's additions. The
    team lead accepted it as one step.
- **Size, an estimate by analogy:** *[first pass: COMP-t's probe program is 1 567 lines across five files;
  expect about 900–1 300 lines of `src/` and 400–600 of tests — the test figure is superseded:]* COMP-t's
  in-flight tree (second pass; unbuilt, unreviewed) holds **2 045 lines of program and students' file** and
  **3 521 lines of tests** (seven new test files plus a rig). COMP-b has less screen (it reuses COMP-a's menu and
  the driver file's page) but more concurrency to prove (the phase handovers, the watch, the throw path, the
  owner). Expect about **1 000–1 500 lines of `src/`, and 1 500–2 500 of tests**. That is about **2 executor-days
  plus about 1 day of verification**, unchanged. The documentation list is the longest of any COMP part.
  HANDOFF L2's "~1 day" for all of COMP is stale.
- **ND3 changes it:** *[(a) lapsed, second pass]* (b), now recommended, adds about ¼ day to COMP-b. ND6 (a) adds
  nothing; (b) adds an hour of prose moves.
- **Blocking before the go:** ND1 (the throw path's word) and ND4 (who paints) change the program's core
  shape, and ND3 changes which files it opens. ND2, ND5 and ND6 each change one branch or one step.
- **Nothing in it has run on a robot** until Station K's log says so.

---

## Re-check 2026-09-26 — amendments B1–B27, B28 added at T3 Part 1's merge and B29 at G1 Part 2c final's (BINDING; they override the text below where they conflict)

> A read-only re-check of this brief against the tree at `b622d9b` plus G1 Part 2b (landed `7f9b9a1`),
> DEFECTS2 (landed `a8540e6`) and the T3 briefs (`9325490`), the vendored PROS headers and the 2026-09-25
> robot evidence; the load-bearing items re-verified by the coordinator. **The team lead's rulings
> (2026-09-26):** COMP-a starts NOW in an isolated worktree, in parallel with T3 / G1 Part 2c / DIAG1 (COMP-b
> after 2c); the auton-pick question under a Timed Run (B6) is decided AFTER the measurement; COMP-b's large
> documentation list is accepted as one step and it CARRIES the held 2026-09-25 documentation pass (B21).

- **B1 (was blocker A1) — abandon is a LATCH, never a disarm.** `RunGuard::requestAbandon() noexcept`
  (ADDITIVE to F12) stores an atomic flag and returns. From the next check on, exactly as after
  `endActionAt`, `pace()` cuts any active motion BEFORE the world advances, and `waitFor()` / `pause()` /
  `expired()` return RunExpired (the cause goes in the report). **`run()`**, not abandon, sees the flag when
  scoring returns: the FULL cancel-all (B10), SKIP the end action, disarm at its normal exit.
  `RunGuardReport` gains `bool abandoned` (`endActionRan` false then). Tests: abandon during a motion, inside
  `guard.waitFor`, inside `guard.pause`, during the end action — later guard calls return RunExpired without
  throwing, later motion starts are cut, the scheduler ends idle and callable, the guard disarms only when
  `run()` returns. Mutation: disarm inside `requestAbandon()`.
- **B2 (was blocker A2) — one task on the Chassis.** No task but the auton task touches the Chassis,
  scheduler, localizer, pacer or mechanisms until the auton task has published FINISHED (an atomic, its last
  act after `run()` returned; the guard's cancel-all already left drive and mechanisms safe). `opcontrol()`
  waits for it with NO upper bound, checking every tick; after a stated program constant (reasoned in code)
  it paints `AUTON ENDING -- driver control waits` and logs once, naming the cause. `disabled()` commands
  nothing and ~~only paints~~ **paints nothing** *[amended 2026-09-28, RC3: it logs B5(1)'s line and hands the
  screen owner to the menu; the auton task paints]*. The guide states the worst case (a frozen wait delays driver control by its
  remainder; use `guard.pause()`/`guard.waitFor()` in autons). *[Amended 2026-09-28, RC1: a TERMINAL word is
  published on EVERY path after a request — a normal return, a refusal, an abandon AND a throw (after the
  program-error stop) — so the unbounded wait can only be a routine frozen outside the guard's waits, never a
  caught error. The word is FINISHED on every path under ND1 (a)/(b), the recommended default; under ND1 (c)
  the throw path's word is STOPPED, and `opcontrol()` paints the program-error screen and idles.]*
- **B3 — order and 2c.** COMP starts after G1 Part 2c for COMP-b (the order 2b → DEFECTS2 → T3 → 2c → COMP);
  COMP-a (library only) may run earlier in a worktree. §3.3's "if 2c has not yet replaced it" is settled:
  COMP adds only forwarding branches to 2c's dispatcher `main.cpp` (≤ 300 lines) and builds its phase screens
  on `src/state_screens.*`; it does not touch the gate's (removed) warning differential.
  *[superseded 2026-09-28: the ORDER is COMP-a (landed) → COMP-t → COMP-b → T4 Part 2 → T4 Part 3 (RC8,
  build-order :2025); the phase screens are NOT built on `state_screens` — COMP-b reuses only
  `competitionBits()`, and `main.cpp`'s Competition cases forward to the program's own phase functions, which
  log and hand the screen owner over (RC7, RC3); ≤ 300 lines still holds, counted at start and end, and an
  overrun is the coordinator's call (RC12).]*
- **B4 — Station K**, never H (H/I are R2a's sensors, J is T3's claws). K runs only after Stations F (drive)
  and J (claws) passed on the same build line. *[Amended 2026-09-28, RC9: AND after COMP-t's Station M — M.3's
  forward check passed and M.6's floor result recorded, on the same build line.]*
- **B5 — the controller Timed Run is the bench start mode** (the team owns no competition switch). COMP-b:
  (1) logs every competition-status CHANGE (millis + decoded bits) from the auton task's idle loop, and one
  line on entry to / normal return from each of `competition_initialize()` / `disabled()` / `autonomous()` /
  `opcontrol()` *[the status-change lines need B19's idle LOOP — a finite `notify_take`, RC3]*; (2) a NEW HA
  ~~(next free at commit, after DIAG1's HA-160 — e.g. HA-161)~~ **HA-162 ONLY, reserved: the Timed Run entry
  (and the only one); a second hardware belief ⇒ stop and tell the coordinator** *[amended 2026-09-28, RC2:
  HA-161 is COMP-a's task-kill entry; HA-162 is COMP-b's reservation, cited by L9, SAFE-SAVE and DIAG1-partB]*: "a controller Timed
  Run enables autonomous() after a disabled window of __ s, for __ s, then __" — UNMEASURED, settled by
  Station K; HA-128 cross-referenced, still settled for what it measured; (3) the worksheet's "never start
  from the controller's menu" rule gains ONE exception, written at A.1, G.0.5, **J.9.6 (re-read; it stays
  forbidden for slot 2 — RC9)** and the DO NOT list: slot 6
  "shulib Competition" (slot 5 is held for the team's old-code fallback — T4 Part 0, 2026-09-27) at Station K, started with Timed Run; afterwards turn the controller off, wait ten
  seconds, see the menu return before starting any other slot; (4) Station K's first step is this
  measurement — if Timed Run never enters autonomous(), K records that and tries "Match" from the same menu;
  (5) C6's periods never come from this measurement.
- **B6 — the auton pick under a Timed Run** (the team lead: decide after measuring): if Station K finds no
  disabled window before autonomous(), how an auton is chosen on the bench goes to the team lead (options: a
  remembered last choice on the SD card, a switch, practice only at an event). Nothing is built for it before
  his ruling; no auton is ever started from `opcontrol()` or another program *[amended 2026-09-28, RC8:
  except the Auton Tester, which runs one through the same `runAuton` path from its RUN — the team lead's go of
  2026-09-27, COMP-t §0.1]*.
- **B7 — the pacer layering.** A program-side `CompetitionWatch` pacer wraps `devices.pacer()`; its `pace()`
  reads `competition_get_status()`, and on a disable or a change out of autonomous while a run is live calls
  `guard.requestAbandon()` (an atomic store, no re-entry), then paces (≤ one tick of latency, stated).
  Storage: the program's static state of empty optionals, declared provider, watch, guard{watch},
  robot{.pacer = &guard}; the watch gets the guard's address after the guard is emplaced. Host-tested with a
  fake status source.
- **B8 — 2b's one-door boot.** `initialize()`: the precondition handler; `st.boot = decideBootSource(kTag)`;
  on `!ok` paint the refusal and stop; emplace provider, watch, guard; then inside ONE try:
  `robot::BuildOptions opts = st.boot.buildOptions(); opts.pacer = &*st.guard;` *[RC8: spelled
  `st.boot.built.buildOptions()` in the tree — `BootOutcome::built`, `boot_source.hpp` :370; the config is
  `st.boot.built.config`]* → `validate` → `emplace`;
  `attachFaultLatch(&robot.faults())` BEFORE the auton task exists; the beacon arrives from main.cpp's
  branch; no `SHULIB_RUNS_*` outside main.cpp. `BootConfig::buildOptions()`'s doc is reworded (a pacer a
  caller adds is borrowed for the Robot's life) or `buildOptions(motion::ITickPacer&)` is added.
- **B9 — the guard's mechanisms are the Robot's PRESENT mechanisms** (after T3): no nullptr (RunGuardConfig
  throws on one) — filtered and each absent one logged by name; from the Robot, never from an AutonEntry.
  Test: one pivot absent ⇒ the auton runs and the present pivot is safed at the cut and the end.
- **B10 — abandon's cancel-all is run()'s FULL cancel-all:** the drive, then every listed mechanism
  (claimant cancelled, anonymous claim force-released, safe state applied). Test: an abandon during a
  mechanism operation leaves no mechanism `claimed()`, and a T3 binding drives that pivot on the next press.
  (T3 also frees leftover claims at driver-control entry — the team lead's ruling. Its consequence
  for the claw, noted at T3 Part 2's review: the release comes before the toggle's first `enter()`,
  so at the FIRST driver-control start the toggle sends its initial level even over a claim — a
  claw an auton left claimed and OPEN closes when driver control starts, as an unclaimed one
  already did. Whether that is wanted after an auton is the team lead's call — asked in
  T3-PROGRESS P2.11; COMP-b re-checks the answer before it wires the claw.)
- **B11 — opcontrol() IS the driver file** *(REPLACED by T4 Part 1a, 2026-09-27, the team lead's rulings
  R1–R7 of that night; until then it read "opcontrol is Teleop's driver control": T3's bindings overload with
  the SAME bindings as Teleop and a `DriverLoopConfig` naming Teleop's constant)*.
  - The program holds a `RobotTwoDriver` (`src/team/robot_two_driver.hpp`) in its static state.
  - `initialize()` calls `st.driver.bind(*st.robot, why, sizeof why)` inside the ONE try, right after
    the Robot's emplace. False ⇒ the refusal screen with `why`, and nothing further built. The catch
    calls `st.driver.unbind()` before it resets the Robot.
  - `opcontrol()`, after B2's wait, calls `st.driver.run(r, <pacer>, disabled, panel)`, and nothing else
    drives.
  - The re-zero, the feel and Coast-on-release are the driver file's constants, never restated here.
  - The pacer is COMP-b's choice (the provider's, or the guard, which passes through outside a run —
    say which).
  - **The driver-control screen is built from T4's shared PURE rows.**
    - Its rows and the controller-screen lines come from PURE, PROS-free functions in one shared `src/`
      header (proposed `src/common/driver_panel.hpp`), host-tested.
    - COMP's page is the header, the claw row, the fault row and the controller row. The PROTECTION row
      joins at T4 Part 2.
    - The new "shulib Drive" composes its MAIN and DETAILS pages from the same rows (T4 Part 3, the team
      lead's R5), and a later robot console can stream them (the team lead, 2026-09-27). The program only
      paints.
    - The claw row is `clawRowText(t, robot, st.driver.bindings().anyReleasedLeftoverClaim())`.
    - The controller screen's ONE chooser moves here, and so does any piece COMP needs that is still in
      `teleop_program.cpp`. They MOVE to the shared header; they are never copied. SAFE-SAVE B2 later adds
      its `BACKUP` word to that chooser. *[2026-09-28, RC6: the refusal painter is PROS-bound and cannot go in
      a PROS-free header, and B30's pieces are typed on the tester's `AbortWatch` — each piece's home is ND3;
      until ruled, nothing is copied.]*
  - The claw-port-in-record refusal stays in the shared boot source.
  - Teleop's claw boot line ends `-- in shulib Teleop only`, which is false once this program binds the
    claws: reword it and re-quote worksheet J.3.1 in the same change. *[2026-09-28, RC5: with its three word
    pins `test/teleop_claws_test.cpp` :487, :490, :789, changed WITH A LOG (T4 landmine 1); the new wording
    true for BOTH programs; and the documentation list gains cookbook 06 :268, guide 07 :125-126,
    `src/team/README.md` :7-8, roadmap :353, build-order :632/:1479 (T4 status) and :1501 (COMP's DoD).]*
  - B10's question is ANSWERED by the team lead's ruling: the first driver-control entry closes the claw
    whatever autonomous left, and the driver file's T2b pins it. Station K's B28 step still observes it
    on the robot.
  - The driver file's header caller sentence gains this program.
  - A host source pin proves the wiring (T4's pattern).
  - The drive protections (T4 Part 2, which lands after this) reach opcontrol() through
    runDriverControl with no wiring. Until then this program's driver control has no mid-match absence
    rule and no cuts, and Station K says so.
  - *(As landed at T4 Part 1a, for COMP-b's builder: the shared claw row is
    `clawRowText(const MechanismTable&, const robot::Robot&, bool freed)` in
    `src/programs/teleop_claws.hpp`, byte-identical to Teleop's; the driver file's `bindings()` is a
    `DriverBindings` (`src/common/driver_controls.hpp`) with `holds()`, `toggles()`, `holdFor(name)`,
    `toggleFor(name)` and `anyReleasedLeftoverClaim()`; T2b is `test/robot_two_driver_test.cpp`'s
    "T4 1a T2b" case. Until T4 ~~Part 1b~~ **Part 3** *[RC5: Part 1b SKIPPED and folded into Part 3, the team
    lead 2026-09-28]* the buttons are also in `src/robots/mechanism_table.hpp`, tied at
    compile time (`controlsMatchTable`, `teleop_program.cpp` :186) — COMP-b edits neither copy.)*
- **B12 — re-seat only a LIVE estimate.** `runAuton` arms the guard FIRST (start time as close to
  `autonomous()` entry as the hand-off allows; the latency logged), then inside the guarded run waits with
  `guard.waitFor(localizer.qualityClass() != Uninitialized)`, THEN re-seats position and heading, then runs
  the routine; `reseatHeading`'s precondition: the estimate is live. Test with the plant's IMU still
  calibrating; mutation: re-seat before the wait.
- **B13 — the re-seat test must be able to fail:** the plant's IMU must DISAGREE with the declared start
  (e.g. plant at 90°, a Degradation `imuHeading()` subtracting 90°; for the relative start a ≥10° pre-auton
  drift) — state both numbers; the "drop the re-seat" mutation must go red.
- **B14 — `reseatHeading` is a CALLER seed**, the heading twin of `setPose`: it re-baselines exactly as
  `setPose` does (clears `hasLast_`), and `runAuton` calls it on the same tick as `setPose` (or one
  `Localizer::reseat(Pose2d)` does both); precondition: no active motion. COMP-a rewords `localizer.hpp`
  ~:35-39 and `complementary_fusion.hpp` ~:86-90 to "no CORRECTOR can …", logging the decision-consistency
  check. Tests: an EkfFusion localizer re-seated by 90° shows no heading process-noise spike; a re-seat
  raises no ODO_STUCK.
- **B15 — each AutonEntry declares its KIND (Match or Skills)**; the period follows from the program's two
  constants (30 s, 60 s); the selector shows the kind; the registry refuses at REGISTRATION (inside the try,
  logged by name) an unset schedule or a `hardStopAt` beyond the kind's period; `runAuton` keeps its own
  checks. The guide recommends a margin inside the period.
- **B16 — routes:** a route entry's start pose IS its route's Start waypoint (field mode); the registry
  refuses a route entry that also declares a different StartPose; a route with no Start takes the entry's.
  `runAuton` returns the RunGuardReport plus EITHER the routine's RoutineResult OR the route's RunReport (a
  tagged result).
- **B17 — Station K is written against robot two as it is** (the 2026-09-25 evidence): boots from the record
  (no record ⇒ refused); +x is the record's front, the CAMERA end; the expected end pose both as the estimate
  and on the tape (with the record as saved, ~8 % short — or the Calibrate reproduction run first,
  recommended); an ODO_STUCK during a motion aborts it (DIAG1's valued line lands before K; K records
  `stoppedAt` and the cause); the field/switch case stays UNRUN at the bench; the mid-auton-disable step is
  written from K's own Timed Run measurement. *[Amended 2026-09-28, RC9: the expected results cite COMP-t's
  re-predicted table after TANK1 (the acceptance run FINISHES on the host test plant to ×0.724 weak — at ~9 %:
  (0.37, 0.06, 177.28°) T+6.14 s; the 40 in to ×0.723), as host-test-plant numbers anchored on M.6/M.8's
  measured result, and quote TSTR and ` xt` (TANK1 Part B) as the code prints them at COMP-b's merge.]*
- **B18 — the test routine** is the library's acceptance run, not season content: a PROS-free header,
  host-tested on the plant, registered from the students' file, which otherwise ships EMPTY apart from the
  registration pattern — the executor authors no season auton.
- **B19 — the auton task:** created ONCE in `initialize()`, after the precondition handler and
  `attachFaultLatch`; the function-pointer `pros::Task` constructor (no heap), `TASK_PRIORITY_DEFAULT`,
  `TASK_STACK_DEPTH_DEFAULT`; idles on ~~`notify_take(true, TIMEOUT_MAX)`~~ **`notify_take(true, <one tick>)`
  in a loop that logs every status change (B5(1)), polls and paints the menu while it owns the screen, and
  never ticks the Chassis** *[RC3]*; its body catches `PreconditionError`
  and `...`, safes through the guard's unwind, and reports *[RC1: the handlers run the program-error stop
  (drive 0 V + Brake, present mechanisms safe, each step alone, never throwing), log an Error line, request the
  fault screen, and THEN — on every path — unlock the menu and PUBLISH FINISHED (STOPPED on the throw path under
  ND1 (c)); the task never idles for ever without publishing]*;
  waits between tasks use `notify_take` with a
  timeout or an atomic checked every tick, never `task_join`; a second `autonomous()` on the same boot runs the
  choice current then *[RC1: after a program error on this boot, per ND1's answer]*.
- **B20 — one painter, one set of budgets:** `ui/`'s text budgets are static_asserted equal to calibration's;
  the program prints only through `src/screen_text.hpp`'s `printText`; an atomic "screen owner" handed over
  explicitly (menu → auton status → driver panel); the auton task's fault screen is a request the owner paints.
  *[Amended 2026-09-28, RC3/ND4: a task paints only while it holds the owner (compare-exchange); the auton task
  paints the menu (after the boot, `competition_initialize()`, `disabled()`) and the auton status; the opcontrol
  task paints the driver panel after the handover (after FINISHED, or when B2's constant passes); the owner
  returns to the menu when the driver loop returns on a disable; the PROS phase tasks paint nothing.
  `src/screen_text.hpp` is `src/common/screen_text.hpp` (SRC1). COMP-a put the budget static_assert in the host
  suite (its deviation 2).]*
- **B21 — documentation:** COMP-b's list is accepted as one step (the team lead), and COMP-b CARRIES the
  held 2026-09-25 documentation pass, removing every "or a minimal auton hook" (build-order, HANDOFF) and
  correcting HANDOFF L2 ("after G1 Part 2b", "Left / Right / Skills / None"). *[Amended 2026-09-28, RC10:
  HANDOFF :617 reworded around the stations (the first library-commanded `moveTo` at COMP-t's M.6; R3b and M1
  close at S8, unchanged); L2's `opcontrol()` line (the driver file, B11) and "state screens" (the program's own,
  RC3/RC7) corrected too; build-order :640's hit is in a block kept as history — left alone.]*
- **B22 — the one-tick latch:** a routine that starts a motion after `requestAbandon()` commands one tick
  before `pace()` cuts it; test: at most one commanded tick per refused start, zero plant travel; stated in
  the guide.
- **B23-B27 (notes):** re-anchor every citation when COMP starts (or cite by quoted text) — `misc.h`'s enum is
  at ~:41-50 and C4 needs `competition_is_field` / `is_switch`; the task-kill HA names `teleop_loop.hpp`
  ~:19-23 and T3's static-state rule as the other things resting on the belief; an auton name ≤ kMediumCols −
  7 = 29 ASCII characters, no `%`, also within the selector button's label budget; since DEFECTS2 every guard
  stop and the default `brake` end action really brake at the motor and the claw pivots HOLD at every
  cancel-all; AutonContext's mechanisms list is for the guard's safing only (a lookup by name comes when a
  routine first commands a mechanism); the Makefile comment becomes "Five programs …", the guide chapter's
  number stated, COMP's new HA entries take the next free numbers at commit time (after HA-160).
  *[superseded 2026-09-28: the Makefile comment becomes "Six programs" (COMP-t's pending B25); the guide
  chapter is **17** (RC11); the HA is **HA-162 ONLY**, reserved — never "next free", which is HA-181 today
  (RC2); the task-kill HA is HA-161 (COMP-a), whose Source gains this program's auton task and whose Settle is
  reworded to the program's own B5(1) lines (RC2).]*
- **B28 — Station K carries HA-157's second half** *(added at T3 Part 1's merge, 2026-09-26)*: whether a
  3-wire level the program writes while the brain is DISABLED reaches the valve. Station K starts the
  program through the controller's Timed Run on purpose (B5), which brings the brain up disabled (HA-128),
  so it is the one station that can answer it; Station J never starts from the controller's menu. Station
  K's claw step: leave the claw at the OTHER level from its start level, start the program that way, and
  record whether the claw moves at the start, at the enable, when driver control begins, or never
  (HA-157's settle text). COMP-b writes the step.

- **B29 — the seventh build under G1 Part 2c final's ONE chain** *(added at 2c final's merge, 2026-09-27; it
  REPLACES §3 item 3, which still names `SHULIB_RUNS_*` and the retired differential)*: COMP-b adds the program by a
  `Program::Competition` enumerator; ONE branch of `src/build_selection.hpp`'s chain (`#elif
  defined(SHULIB_ROBOT_TANK_2026) && defined(SHULIB_PROGRAM_COMPETITION)` setting `kProgram =
  Program::Competition`, `kRobotId = RobotId::RobotTwo` and `kVariantBeacon =
  "shulib-robot-variant=tank-competition"`, nothing else) plus its `#error` guards; a case in EACH of `src/main.cpp`'s
  five entry-point switches; in `tools/src_build_gate.py` `VARIANTS`, `BUILD_ARGS`, `VARIANT_DEFINES`,
  `VARIANT_BEACONS`, `BUILD_PROGRAM`, `BUILD_ROBOT` (`RobotTwo` — no new robot beacon) and
  `PROGRAM_ENTRY_SYMBOLS`, and a self-test pair like cases 14/15. The define is read in
  `src/build_selection.hpp` ONLY, and the program reads `kChassis` / `kMechanisms` only — never a per-robot table
  by name (step 0 refuses it). `-Wunused-function` is a gate FAILURE. **The slot and opcontrol() — decided by T4** (the team lead's rulings of 2026-09-27, R6; T4 Part 0): slot 6
  "shulib Competition"; `opcontrol()` per B11 as T4 Part 1a replaces it. Whichever of COMP-b and COMP-t (the
  auton tester, briefed as the seventh build) lands second takes the next build and the next self-test pair.
  *[Settled 2026-09-28, RC8/RC4: COMP-t lands first; COMP-b is the EIGHTH build, `Program::Competition` after
  `AutonTester`, self-test pair 34/35 (as COMP-t landed: case 33 is its tables-agree check). The list above omits the Makefile branch and
  several comments — RC4 lists every place; COMP-t §3.6 is the checklist.]*


### Amendments dated 2026-09-28 — COMP-t, "shulib Auton Tester", built first (BINDING on COMP-b)

> COMP-t (the auton tester, `docs/internal/chunks/COMP-t-auton-tester.md` §6 item 6) landed first and built
> pieces this brief assumed COMP-b would build. **B5 is unchanged here** — its slot wording is T4 Part 0's
> (slot 6 "shulib Competition", slot 5 held). The amendments below override B8, B18, B25 (in B23-B27's
> notes) and B29 where they conflict, and add B30.

- **B8 (amended):** `BootConfig::buildOptions()`'s doc was **reworded by COMP-t** (R10): the ports are read
  during construction only; a pacer a caller adds (`opts.pacer = &guard;`) is BORROWED for the Robot's whole
  life. COMP-b finds it done and adds no `buildOptions(ITickPacer&)` overload. The tester's `initialize()`
  (`src/programs/auton_tester_program.cpp`) is the worked example of B8's order, including the boot's chain
  proof (B30).
- **B18 (amended):** **the students' file IS `src/team/autons.{hpp,cpp}` (COMP-t)** — `kAutonPeriods` (the
  manual's 30 s / 60 s) and `registerAutons(competition::AutonRegistry&)`, registering the 40 in auton (written
  on the team lead's go of 2026-09-26, "no you can write the auton") and the acceptance run, both MATCH,
  endActionAt 27 s, hardStopAt 29 s, RELATIVE start. COMP-b CALLS `registerAutons()` from its `initialize()`,
  inside its try, and never writes a second file. The host suite compiles `src/team/autons.cpp` and runs every
  auton in it (`test/autons_test.cpp`).
- **B25 (amended, in B23-B27's notes):** the Makefile's program comment says "Five programs in five slots"
  since COMP-t; COMP-b makes it **"Six programs"**.
- **B29 (amended):** COMP-t took the seventh build (`tank-autontest`, `Program::AutonTester`, beacon
  `shulib-robot-variant=tank-autontest`, self-test cases 31/32). **COMP-b's is the EIGHTH build;
  `Program::Competition` goes after `AutonTester` in the enum; its self-test pair takes the next two numbers
  after COMP-t's (34/35 as of COMP-t's review: case 33 is COMP-t's check that the gate's own tables agree — a build dropped from `VARIANTS` alone).** `tank-autontest` joins the list of beacons `…=tank` is a prefix of.
- **B30 (NEW) — what COMP-b takes from COMP-t, and what it must not:**
  - **Take:** the pacer layering and the latch-every-tick rule (the watch requests `requestAbandon()` on its
    LATCHED cause every pace a run is live — a request made while idle is dropped at the arm); the catch shape
    of `runTesterOnce` (ONE try from the arm to the end, a panic stop `chassis.cancel()` in each handler, and a
    `catch (const std::exception&)` that keeps the throw's own words — COMP-t's review) and
    the task-level try whose handler calls `stopAfterProgramError()` (the drive to 0 V + Brake, every PRESENT
    mechanism safed, each step tried on its own) and then idles for ever (R14) *[RC1, with the team lead's ND1 (2026-09-28): COMP-b takes R14's STOP only — after the safe stop it posts FINISHED and its auton task lives on, so driver control still runs and later autons are refused until a reboot; R14's idle-for-ever is NOT taken]*; **the boot's chain proof** —
    `RunGuard::pacesTotal()` (added by COMP-t) and the watch's own pace count around ONE idle tick, the boot
    REFUSED when either did not move (R13) — AND the watch attached to that guard (`AbortWatch::attachedTo`,
    COMP-t's review: an unattached watch paces and latches but asks nobody to abandon, and the run drove 14 in
    past the button) — for COMP-b's own watch, with the same link check; the Robot's PRESENT mechanisms handed
    to `runAuton` and to the last-resort stop (source-pinned, and test (18b) safes them through the run);
    `kAutonPeriods` and `registerAutons()`;
    the SD run log through `ProsBlockSink`'s Append mode and `hal::BlockCharSink` (ND5, answered: COMP-b
    appends its match runs to the same `shulib_auton_runs.txt`, only after FINISHED is posted); the pure-PROS-read pattern (one line in the program,
    the meaning in a host-tested template such as `testerTouchFrom` / `abortInputsFrom`) and the source-pin
    pattern (`test/auton_tester_source_test.cpp`).
  - **Where the shared pieces live (RC6; ND3 (b), the coordinator's, 2026-09-28):** `pacerChainProven`,
    `stopAfterProgramError` and `testerTouchFrom` are typed on, or live beside, the tester's `AbortWatch` in
    `src/programs/`; COMP-b's FIRST step moves them into `src/common/` with generic versions and re-points
    COMP-t's call sites and pins, logged (~+¼ day).
  - **The command registry, when the first ROUTE auton appears:** registered ONCE in the students' file (e.g.
    `registerCommands(runner::CommandRegistry&)` beside `registerAutons`) and passed by BOTH programs. Until
    then the tester passes `nullptr` and a route auton is REFUSED by name ("a route auton needs a command
    registry to fire its markers") — the one argument that differs from COMP-b's call today.
  - **Must NOT take:** `bench::AbortWatch` — it aborts on ANY button, and in a match a driver's bumped button
    would end the auton; COMP-b's watch has the competition causes only. Nor the idle estimate ticking: B19's
    `notify_take` idle stays COMP-b's.

---

## 0. What exists (read these; every line was re-checked at `8ac243f`)

*[superseded in part 2026-09-28, RC8 — the table describes the tree before COMP-a: the chassis now takes
`options.pacer` when set (`robot.hpp` :562, COMP-a's `BuildOptions::pacer`, :189-200); the boot source and
screen text are `src/common/boot_source.*` and `src/common/screen_text.hpp` (SRC1); `opcontrol()` runs the
DRIVER FILE (`src/team/robot_two_driver.hpp`, B11 as replaced), which calls the library loop — it is not
"this loop, as in Teleop"; the task-kill HA entry EXISTS (HA-161, COMP-a). Re-anchor every other citation by
its quoted text (B23).]*

| Piece | Where | What matters |
|---|---|---|
| The robot from data | `robot/robot.hpp` | `validate(cfg, devices, BuildOptions)`, `Robot{cfg, devices, BuildOptions}`; **the chassis takes `devices.pacer()` at ~:488** and keeps it for life (`chassis.hpp` ~:239-246), while `BuildOptions` is documented "read during construction only" (~:136-138, :427-429) — see C1 |
| The on-robot devices | `hal/pros/device_provider.hpp` | the IMU is made with **boot heading 0** (~:71-73) — see C2 |
| The boot source | G1 Part 2b's shared `src/boot_source.hpp`/`.cpp` (2b, amended at `6456beb`, re-checked at `8ac243f`) | returns its outcome; COMP boots exactly as shulib Teleop does |
| The screen text | 2b's shared `src/` screen-text header (`printText` moved out of `calibrate_program.cpp`) | the one safe print path: ASCII, no `%` (PROS re-reads printed text as a format), bounded, a literal `"%s"`; repaint on change (every print leaks) |
| The driver loop | `teleop/teleop_loop.hpp` (`runDriverControl`, `DriverLoopConfig`, `HeadingSample`) | opcontrol() is this loop, as in Teleop |
| The guaranteed end | `sequence/run_guard.hpp` (**F12 unfrozen**) | `RunGuard(ITickPacer& inner)` (~:240); `run(chassis, config, scoring, endAction)` (~:375-377); `RunGuardConfig{endActionAt, hardStopAt, mechanisms}` (~:163-194) whose fields DEFAULT to 0.0 and whose `validate()` THROWS on them (~:167-189); `RunGuardReport{…, pacesSeen}` (~:198-218) — `pacesSeen` counts only during `run()` and resets at arm (~:258-259, :446); a throw safes and RETHROWS (~:366-369); inert when idle; the guard's `waitFor`/`pause`/`expired` (~:92-107) are the only waits its deadline can cut |
| Routines | `chassis/routine.hpp` (**F10 frozen**): `startAt` calls `setPose`, which IGNORES heading (`localizer.hpp` ~:469-481); `RoutineResult` (~:169-183) | an auton function returns the `RoutineResult`, not a bare `ExitReason` (C3) |
| Routes | `runner/path_runner.hpp` (`PathRunner`, takes `const CommandRegistry&`, ~:164-165), `runner/command_registry.hpp` (`CommandRegistry`, ~:116) (**F8 frozen**) | a route auton needs a registry (non-null precondition) |
| The heading composition | `localization/localizer.hpp`: the published heading is `imu.heading() + headingBias_` (~:16, :35, :253-254), odometry rotated into that frame when the bias is non-zero (~:245-247), the bias kept by `setPose` (~:469-481) | the start-heading re-seat reuses this, rather than touching the IMU (C2) |
| A pure screen model | `calibration/calibrate_panel.hpp` | its RECTANGLES, hit boxes, budgets, `TouchSample` and arming delay are generic (~:73-139, :161-176, :329, :913-923); its `TouchGate` is typed to the calibrator (~:958, :974-975) and caps at 3 buttons (~:234) — see C4 |
| Competition state | `include/pros/misc.h`: `competition_get_status` (~:34-36, :75), `competition_is_disabled` (~:95) | the vendored headers say NOTHING about task killing; the belief lives only in code comments today (~`main.cpp`:447-450) — an HA entry is owed (C5) |

---

## 1. Rulings

**C1 — The builder accepts a pacer, borrowed for the Robot's lifetime.** `robot::BuildOptions` gains
`motion::ITickPacer* pacer = nullptr` (ADDITIVE; `nullptr` = `devices.pacer()`, every existing test
bit-identical). COMP-a REWRITES `BuildOptions`' and the constructor's doc comments: `absentDrivePorts` is
read during construction only, but **`pacer` is borrowed for the Robot's whole life** (the chassis keeps it).
Spelled `robot::BuildOptions{.absentDrivePorts = …, .pacer = &guard}` (designated initializers in
declaration order). The competition program constructs `RunGuard guard{devices.pacer()}` BEFORE the Robot;
outside a run the guard passes `pace()` through. *[superseded 2026-09-28, RC8: B7's layering —
`guard{watch}`, the watch wrapping `devices.pacer()`; the chain proved at boot per B30.]*

**C2 — The start pose is re-seated when the auton starts, position AND heading (ruling 1).** An auton
declares `StartPose::field(x, y, heading)` or `StartPose::relative()` (= (0, 0), heading 0). At the start of
the run — never at boot, because the choice is made after the Robot exists — `runAuton` sets the position
(`setPose`) AND the heading through a NEW, ADDITIVE `Localizer::reseatHeading(math::Angle)`: it sets
`headingBias_` so the published heading equals the given one at this tick, reusing the composition the
AprilTag corrector already relies on (the IMU stays the sole source of heading CHANGE). `Routine::startAt`
(F10, frozen) is NOT changed — it still ignores heading; the guide says so and says `runAuton` does the
re-seat. **HA-05 is amended** (the one owner of the start heading is the chosen auton's start pose, applied
at autonomous start via the localizer's bias, not the IMU's boot heading); the amendment and its reasoning
go in the register. Host tests: a field start at 90° drives a 90°-rotated routine to the right place on the
plant; relative mode after a simulated pre-match drift zeroes it; GPS and tag corrections still compose.

**C3 — An auton is DATA the students' code registers; it gets a context, and returns a `RoutineResult`.**
`competition::AutonRegistry` (fixed capacity, no heap, no `std::function`): each entry has a NAME (ASCII,
unique, ≤ the MEDIUM budget so "AUTON: <name>" fits one MEDIUM line), EITHER
`chassis::RoutineResult (*)(AutonContext&)` OR a `const config::Route*`, a `StartPose` (C2), its end
action (default: `brake`), and its schedule. `AutonContext` carries `chassis::Chassis&`, the `RunGuard&`
(its `waitFor`/`pause` are the waits the deadline can cut), the `hal::ITelemetrySink&`, and a
`std::span<hal::IMechanism* const>` (empty today; the guard safes whatever is listed). Registration is an
explicit call from `initialize()` into the students' file — **never at static-initialization time** (the
file is compiled into every build). The library ships ONE labelled test routine (drive 24 in, turn 90°,
drive back — the R3b/M1 closing run); the season's autons are the students' (the guardrail).
*[superseded in part 2026-09-28, RC8: the students' file is COMP-t's `src/team/autons.{hpp,cpp}` —
`kAutonPeriods` and `registerAutons()`, which register the 40 in auton and the acceptance run (B18 as COMP-t
amends it; B30). COMP-b calls `registerAutons()` and writes no second file. `AutonContext`, `AutonEntry` and
`AutonRegistry` are as COMP-a landed them (the status block's deviations: `StartPose` UNSET, `AutonPeriods`).]*

**C4 — The selector is a PURE, host-tested model with its OWN generic touch gate.** Build the generic parts
(rectangles, hit boxes, budgets, `TouchSample`, the arming delay, a button-id-agnostic gate) in
`include/shulib/ui/`, and `competition::AutonMenu` on them. **`calibrate_panel.hpp` is NOT changed** (the
pending bench session's Calibrate runs on it); moving it onto `ui/` is a later, separate change. The menu:
one button per entry, paged when more than fit, NONE always present and the DEFAULT; the choice is one
atomic store, readable from any task; a touch cannot change it once the auton has started. Which task polls
the screen, per case, is stated in the program: a field or competition switch attached
(`competition_initialize()` / `disabled()`), nothing attached (`opcontrol()` starts at once — the menu shows
until the driver's first stick input), and HA-128's pretend match from the controller menu.
*[superseded in part 2026-09-28, RC3: the PROS phase tasks do NOT poll or paint the menu — `view()`/`touch()`
belong to ONE task (`auton_menu.hpp` :190-191), and PROS deletes its phase tasks on a mode change. The poller
and painter per phase is ND4 (recommended: the auton task, in every non-driver phase). The `opcontrol()`
clause "the menu shows until the driver's first stick input" is ND4's to keep or drop (recommended: drop).
COMP-a built the menu and the generic `ui/` pieces as ruled here; they are landed.]*

**C5 — The auton runs on the program's OWN task; the program ends it (ruling 2).** `autonomous()` starts or
signals a program-owned `pros::Task` that PROS does not kill, then waits; the auton task runs `runAuton`. The
pacer under the guard checks the competition state every tick: on a disable or a mode change it ends the
run through a NEW `RunGuard::abandon()` (ADDITIVE to F12, unfrozen) — cancel into the safe state, skip the
end action (a disabled robot's motors are off; the park cannot run), unwind the routine normally on its own
stack so nothing dangles, disarm. `opcontrol()` and `disabled()` wait (bounded, e.g. ≤ 100 ms) for the auton
task to report finished before commanding anything; if it has not, they force the drive safe and say so.
An exception inside the auton is caught ON the auton task: the robot is safed, a fault screen and a log line
say what broke. **Register an HA entry** for the belief that PROS kills its competition tasks
(`autonomous`, `opcontrol`, `disabled`, `competition_initialize`) on a mode change — Station H settles it.
*[superseded in part 2026-09-28 — the ruling itself (the program's own task; the program ends the routine) is
unchanged. What changed: `RunGuard::abandon()` landed as the latch `requestAbandon()` (B1), and `run()` does
the cancel-all (B10). "`opcontrol()` and `disabled()` wait (bounded, ≤ 100 ms) … force the drive safe" is
replaced by B2: only `opcontrol()` waits, with NO upper bound, for the task's terminal word; `disabled()`
commands and paints nothing (RC3). The exception clause is RC1's: the program-error stop, an Error line, the
fault screen through the screen owner, then the terminal word per ND1. The HA entry EXISTS as HA-161 (COMP-a);
**Station K**, not H, settles it (B4, RC2), from the program's own B5(1) lines (RC2's "also found").]*

**C6 — The schedule is stated, never defaulted, and bounded by the period.** From the official Override
Game Manual v1.0 (2026-07-02; the team lead asked the coordinator to look it up): **VEX U Head-to-Head —
a 30 s Autonomous Period (`<VUT4>`), then 90 s of driver control (`<VUT5>`); VEX U Autonomous Coding
Skills — 60 s, two robots; Driving Skills — 60 s** (Appendix B's Match table). The program states these as
its two periods (match 30 s, skills 60 s); each auton states its own `endActionAt`/`hardStopAt` inside
its period. `runAuton` detects an
UNSET schedule (the fields' 0.0 defaults) and refuses with a sentence BEFORE calling `run()` (which would
throw); the program also refuses any entry whose `hardStopAt` exceeds the period it runs in.
*[2026-09-28, RC8: the two periods are stated ONCE, as `kAutonPeriods` in COMP-t's students' file
(`src/team/autons.hpp`; B18/B30), and `AutonRegistry` takes them at construction (COMP-a's deviation); COMP-b
reads them from there and states no second copy.]*

**C7 — Nothing runs from a boot the library refused.** The refusal screen stays in every phase;
`autonomous()` logs why and does nothing; `opcontrol()` idles. *[2026-09-28, RC3: a refused boot creates no
auton task, so nothing paints over the refusal and B5(1)'s status-change lines are absent (the phase lines
remain). ND2, if answered as recommended, adds a narrower refusal: a boot from the typed table REFUSES the auton
only, and driver control still runs.]*

---

## 2. Scope — COMP-a (the library; no `src/`)

*[2026-09-28, RC8: COMP-a LANDED as `b5aaf95` (2026-09-26), host-proven, not run on a robot. This section is
history; what landed differs from it where the re-check's status block lists COMP-a's deviations (e.g.
`RunGuard::abandon()` is the latch `requestAbandon()`, B1).]*

1. `BuildOptions::pacer` + its doc rewrite (C1). Tests: default bit-identical; a guard passed in sees every
   `pace()` of a motion (use a COUNTING inner pacer — `pacesSeen` counts only during `run()`); a routine is
   cut at `endActionAt` on the plant.
2. `Localizer::reseatHeading` (C2) + HA-05's amendment. Tests as in C2, plus: the bias composes with a later
   tag/GPS correction; `setPose` after a re-seat keeps it.
3. `include/shulib/competition/auton_registry.hpp` (C3): `StartPose`, `AutonContext`, `AutonEntry`,
   `AutonRegistry` (preconditions: unique ASCII names within budget, exactly one of routine/route, a route
   entry requires a registry at run time, a schedule set or "unset").
4. `include/shulib/ui/` generic panel pieces + `include/shulib/competition/auton_menu.hpp` (C4).
5. `include/shulib/competition/match_runner.hpp` — `runAuton(chassis::Chassis&, sequence::RunGuard&,
   const AutonEntry&, const runner::CommandRegistry*, hal::ITelemetrySink&,
   std::span<hal::IMechanism* const>, units::Time period)`: refuse an unset schedule or one past the period
   (C6); re-seat the start pose (C2); run under `guard.run(...)`; return the `RunGuardReport` plus the
   routine's `RoutineResult`. `RunGuard::abandon()` (C5), additive.
6. **Tests (adversarial, on the host plant):** the end action FIRES when the routine stalls in `guard.waitFor`
   (the deadline cuts it) — and a separate case pins how late a wait the guard CANNOT cut ends; the hard stop
   forces safe if the end action hangs; `abandon()` mid-motion leaves the scheduler idle and callable, the
   guard disarmed, and the drive safe, with the routine unwound (no dangling); a throwing routine ⇒ safe;
   a route entry runs bit-identically to the same legs as a routine; the start-pose cases (C2); the menu
   (NONE default, every drawn button maps to an entry, paging hides none, no change once started, no new
   button under the one just pressed, ASCII/no-`%`/budget checks); the registry's preconditions. Mutations
   RUN and observed red: skip the end action on a cut; arm an unset schedule; drop the heading re-seat; drop
   `abandon()`'s cancel; drop the default NONE; allow a touch to change the choice after start; drop the
   `BuildOptions::pacer` forwarding.
7. **Docs (COMP-a):** `docs/api` (regenerated); `docs/changelog.md` (additive: `BuildOptions::pacer`,
   `Localizer::reseatHeading`, `RunGuard::abandon`, the new headers); HA-05's amendment and the new
   task-kill HA entry; the Freeze Register's F12 row (a second consumer — the freeze is the team lead's
   call; state the evidence, do not freeze); `build-order.md` + `roadmap.md` pointers.

## 3. Scope — COMP-b (the program, a seventh build)

*[superseded in part 2026-09-28, RC8: the EIGHTH build (after COMP-t's seventh). Where an item below conflicts
with B1–B30 or the 2026-09-28 re-check, those win; each item carries its correction.]*

1. `src/competition_program.cpp`/`.hpp` (`namespace shulib::bench`): `initialize()` — the precondition
   handler, `ProsDeviceProvider`, the shared boot source, `RunGuard guard{devices.pacer()}`, `validate(...)`
   then the Robot ONCE into `std::optional` with `{.absentDrivePorts = …, .pacer = &guard}` (the `try` wraps
   both), the session header, the students' registration call (C3), the auton task created (C5), the menu
   (C4); `competition_initialize()` / `disabled()` — C5's wait, the state screen with the choice and the boot
   source; `autonomous()` — start the auton task, log the chosen auton by name (the session header was
   printed before the choice), wait; `opcontrol()` — C5's wait, then `runDriverControl` as in Teleop.
   *[superseded 2026-09-28, RC8: the files are `src/programs/competition_program.{hpp,cpp}` (SRC1; as T4
   :1241 names it), with the shared pieces placed per ND3. The pacer layering is B7's (`guard{watch}`, the
   watch wrapping `devices.pacer()`), proved at boot by B30's chain proof; the build is B8's
   (`st.boot.built.buildOptions()`, then `.pacer = &*st.guard`) and B11's `st.driver.bind(...)` in the same
   try. `competition_initialize()`/`disabled()` log and hand the screen owner over; they paint nothing and
   never call `benchStateScreen` (RC3, RC7). `autonomous()` signals the auton task and waits; B2's terminal
   word ends the wait (RC1). `opcontrol()` waits per B2 and then runs the DRIVER FILE (`st.driver.run(...)`,
   B11 as replaced), not `runDriverControl` directly.]*
2. The students' autons file in `src/` with the one labelled test routine (C3). *[superseded 2026-09-28, RC8:
   the file is COMP-t's `src/team/autons.{hpp,cpp}` (B18, B30). COMP-b calls `registerAutons()`, writes no
   second file, and authors no auton.]*
3. **The seventh build, everywhere it must be named:** `src/main.cpp`'s variant checks (the one-PROGRAM
   `#error`, the needs-`ROBOT=tank` check, the `SHULIB_RUNS_*` macros, the beacon chain — beacon
   `shulib-robot-variant=tank-competition`), the Makefile's PROGRAM validation and its "Four programs"
   comment (~:102), `tools/src_build_gate.py` (`VARIANTS`, `BUILD_ARGS`, `VARIANT_DEFINES`, `BEACONS`, its
   header table, a self-test pair like its cases 14/15, and — if G1 Part 2c has not yet replaced it — the
   variant differential's list), `.github/workflows/ci.yml`'s comment. Slot **5 "shulib Competition"** (slot 6 — B5 as amended).
   *[superseded: B29 replaced this item (G1 Part 2c final, 2026-09-27), and RC4 (2026-09-28) lists every place
   the EIGHTH build is named, the Makefile branch included, with COMP-t §3.6 as the checklist. The slot is
   **6**; slot 5 is HELD for the team's old-code fallback (T4 Part 0; worksheet :28-34).]*
4. Docs (COMP-b): a guide chapter **"The competition program"** from the code (registering an auton, the two
   start modes, the selector, the schedule and the end action, a mid-auton disable, a refused boot), every
   example COMPILED in `test/guide_examples_test.cpp` and quoted verbatim; README; worksheet **Station H**
   (a practice match: the selector, a competition-switch or field run including a mid-auton disable, what the
   screen and the log say, the test routine's expected end pose; it settles the task-kill HA entry);
   `build-order.md` + `roadmap.md` (M4's guaranteed-park line: reaches a real program — still unrun);
   `COMP-COMPLETED.md`.
   *[superseded in part 2026-09-28: the chapter is **17**, and its registration listings quote
   `src/team/autons.cpp` live as unbroken runs and link to chapter 16 (RC11); program-only examples still
   compile in `test/guide_examples_test.cpp`. The station is **K**, not H (B4), run after F, J and M (RC9),
   with the Timed Run as its start mode (B5; the team owns no competition switch), and it settles HA-161 from
   the program's own lines (RC2). The HA is **HA-162 only** (RC2). The list also carries B21's held pass
   (with RC10) and RC5's additions.]*

## 4. Landmines
1. `-Wformat-nonliteral`/`-Wformat-truncation` are errors in `src/`; one print path (2b's shared header,
   now `src/common/screen_text.hpp` — SRC1);
   no `%` on screen; ASCII; y < 236.
2. The auton task must never command the drive after `opcontrol()` has started it — C5's handshake, tested
   on the host where it can be (the pure parts) and read line by line where it cannot. *[2026-09-28: the
   handshake is B2's terminal word (FINISHED, or STOPPED under ND1 (c)) published on EVERY path (RC1).]*
3. The Robot, the provider, the guard and the auton task live for the program; construct once, from
   `initialize()`.
4. Assert `pacesSeen > 0` after a run, in the log (C1). *[2026-09-28: and prove the chain at boot, before the
   first run, with `RunGuard::pacesTotal()` around one idle tick (B30, COMP-t's R13), for COMP-b's own watch.]*
5. The IMU calibrates ~2 s after boot; motions wait for a live estimate — an auton started early is slow, not
   wrong; the guide says so.
6. No hardware claim: the program is UNRUN until Station ~~H~~ **K**'s log says otherwise *[RC8; B4]*.

## 5. Not decided here
Which routines exist (the students'), the
mechanisms and their end states (handoff Q1), button bindings (T3), field-centric for the driver (off, as
Teleop). *[2026-09-28, RC8: the button bindings, the feel and field-centric (OFF) are the DRIVER FILE's
(`src/team/robot_two_driver.hpp`, T4 Part 1a; its heading rule is `driverHeading()` in
`src/common/driver_controls.hpp`); the buttons stay tied to `src/robots/mechanism_table.hpp` until T4 Part 3
(RC5). None of them is COMP-b's to decide.]*
