# COMP-t — COMPLETED (built 2026-09-28, landed 2026-09-29; host- and build-gated; NOT run on a robot)

> "shulib Auton Tester" and the students' auton file — the team lead's request of 2026-09-27 ("a seperate
> auton program so that it doesnt need to be in comp mode to run it"), his answers D1–D9 as recommended,
> D10 superseded by TANK1 ("Fix the library first"), slot 7 until T4 Part 3 then 2 (T4 N4). Written FROM
> [`COMP-t-PROGRESS.md`](COMP-t-PROGRESS.md) (the live log: every step, the deviations with their reasons,
> the mutation table, every gate line). Brief: [`COMP-t-auton-tester.md`](COMP-t-auton-tester.md) (its
> "Re-predicted after TANK1 Part A" note governs the weak-side numbers). Executor model: Opus 5.5 (1M
> context), one run (D8), in the isolated worktree `.claude/worktrees/comp-t` from `e124b88`. **Nothing was
> committed by the executor; the coordinator verifies and commits.**

## The one-sentence honest status

A bench program runs one of the team's autons through exactly the call the competition program's
`autonomous()` will make — `competition::runAuton` on a Chassis paced by a `RunGuard`, the same entry,
schedule, re-seat and guaranteed end — started by RUN and a 3 s countdown instead of the field, abortable
by any button, a touch, the controller dropping or a disable; it and the students' file are proven on the
host test plant and in all seven V5 builds, and **none of it has run on a robot** (worksheet Station M is
its first run).

## Built

- **The students' file** — `src/team/autons.hpp`, `src/team/autons.cpp`: VERBATIM as the brief's §3.1 gave
  them (byte-compared by script; no change was forced). `kAutonPeriods` (30 s / 60 s), `registerAutons()`:
  the 40 in auton (`r.moveTo(math::Pose2d{40_in, 0_in, 0_deg})`) and the library's acceptance run, both
  MATCH, `endActionAt` 27 s, `hardStopAt` 29 s, RELATIVE, the brake as end action. The ONE `src/` `.cpp` the
  host suite compiles (`test/CMakeLists.txt`'s `target_sources`).
- **The program** — `src/programs/auton_tester_program.{hpp,cpp}`: §3.5's program **as amended by the
  review (deviation 7), no longer the brief's text**. Still byte-identical to §3.5's blocks (checked by
  script, block by block): the PROS reads, `readFacts`, `ScreenRunObserver`, `stopTheTester`. Changed by
  the review: `runOne` (now `runOne(int index, std::int32_t runPressCount)` — the abort watch's baseline),
  `testerTask` (holds RUN's touch read in a local and passes its press count to `runOne`; a
  `std::exception` handler) and `autonTesterInitialize` (the chain refusal's words; the boot line split into
  `testerBootLines()`'s three). It stands on three PROS-free, host-tested headers: `auton_abort_watch.hpp` (the watch, `abortCauseOf`, `testerTouchFrom`,
  `abortInputsFrom`), `auton_tester_run.hpp` (`runTesterOnce`, `pacerChainProven`, `stopAfterProgramError`,
  `testerResultWord`, `testerRunLines`) and `auton_tester_panel.hpp` (the pages, `runAllowed`, `sameButtons`).
  The boot PROVES its pacer chain or refuses (R13); RUN only from a RECORD boot (D9); one task owns the
  robot (R1) and its whole body is one try (R14).
- **The seventh build** — `make ROBOT=tank PROGRAM=autontest` → `Program::AutonTester`, beacon
  `shulib-robot-variant=tank-autontest`, tag `ATS`: `src/build_selection.hpp` (the chain's branch, the
  guards), `src/main.cpp` (a case in each of the five entry points — four of them only `return;`),
  `Makefile`, `tools/src_build_gate.py` (every table, the docstring, self-test cases 31/32), `ci.yml`.
- **The library (additive; no version bump)** — `RunGuard::pacesTotal()`; `ProsBlockSink`'s Append mode
  (`ProsFileMode`, `AppendOpen`, `openForAppend()`, `appendOpen()`; the two-argument constructor delegates
  with Truncate, byte for byte); `hal::BlockCharSink` (NEW header). **Fixed at its root:** COMP-a's
  `checkAutonEntry` `routeWhy[160]` → 192 and `%.150s` — a `-Wformat-truncation` in every V5 build the first
  time a `src/` file registers an auton (mutation 19 reproduced it: bench FAILED on `route.hpp:454`).
- **The example scan** — `tools/api_doc_tool.py`: `src/team/autons.cpp` APPENDED to `EXAMPLE_SOURCE_GLOBS`
  and to `WHOLE_RUN_SOURCE_GLOBS` (the coordinator's ruling); its self-test gained the auton kind.
- **Beliefs** — HA-177 (an append open on the card) and HA-178 (the abort's and the RUN gate's PROS reads),
  registered; HA-161's Source gains the tester's task.

## The review (2026-09-29): 23 findings, their fixes, and what they changed

Three reviewers (safety, tests, docs) returned 23 findings; every one was real, and 22 are fixed — D10
(the briefing counts this record as a 33rd chunk) is NOT: it is the coordinator's call ("For the
coordinator", below). The live log's "COMP-t review findings and their disposition" has each with its
proof. The code findings were fixed at their root, each with a test whose mutation was observed RED on a
fresh binary:
- **Mechanisms safed, proven:** test (18b) sends FakeMechanisms through `runTesterOnce` (end of a clean run,
  B mid-run with a claimant cancelled, B during the end action, a countdown abort touching none); the
  program's collection of `present` and its two hand-offs are source-pinned.
- **The chain proof covers the watch→guard link** (`AbortWatch::attachedTo`), and an abort the guard never
  acted on reads a red **`ABORT IGNORED`**, never FINISHED.
- **An abort during the brake reads `end action CUT`**, never FAILED.
- **`RunGuard` fixed at its root:** a throwing safe state no longer skips later mechanisms or reaches
  `std::terminate` through the unwind (changelog FIXED; F12 open by design, no bump).
- **Throws keep their words** (`catch (const std::exception&)` in `runTesterOnce` and the task), and each
  handler's panic stop is proven by a throw outside every wait with the drive live — (21e)(b); the stopped
  pass's version could not see it, because the scheduler's F2 unwind stops a throw through a wait first.
- **A touch after RUN's read always aborts** (`AbortWatch::arm(pressBaseline)`, armed with RUN's own read).
- **The boot line is three lines** (`testerBootLines()`, every field bounded): at this repository's real
  `git describe` the one line was 206 bytes and lost the SD path.
- **Pins** on `readFacts`, the chain-proof refusal, RESULT's finger and the repaint rule; PICK in
  `sameButtons`' test; a real NOT STARTED run; the ×0.65 golden whole; the adapter's close observed.
- **Test (1b) runs EVERY registered auton** — the docs' claim, made true.
- **The gate** checks its own tables agree (self-test case 33).

The docs findings: guide 16 (the `rm -rf bin .d`, the competition program not built yet, `startAt`,
ABORT IGNORED, CUT, the top-half reason, the anchor, the SD words, what the tests do with a new auton,
STEP BACK), the worksheet (M.2.2 the CREATED check, M.7 rebuilt around the existing file — never format the
card — M.6.5 lift don't shove, M.4's reason and ABORT IGNORED STOP, M.9's note), mkdocs nav, chapter 9,
cookbook 1, README, HA-177/HA-178, the COMP brief's B29/B30.

## Deviations from the brief (each forced; each logged in the live log with its measurement)

1. The throw line's `what` is `%.88s`, not `%.120s`: with the brief's `%.29s` name and a 5-digit run the
   `%.120s` line is 231 bytes, past TermSink's 199 (mutation A1 measured the cut).
2. `TesterRunLines::cut`, and `TesterView::droppedLines` / `unsafeLines` — the "never silently" rules made
   countable, so tests (23) and (24) fail on a cut instead of reading a truncation as fine.
3. The panel formats into twice the widest budget: a budget-sized buffer let `snprintf` itself cut an
   over-budget line before the view could count it — **mutation A2 SURVIVED (24) until this fix**.
4. A REFUSED run's first two lines say so instead of a hand-off computed from an `armedAt` of 0.
5. The refusal painter wraps its footer over two SMALL rows (the probe's default footer was 55 characters
   on one 50-character line) — both footers `static_assert`ed.
6. The source-pin test's file constant was first named `kProgram` — the gate's one-chain rule FAILED the
   tree on it (a chain constant declared outside the chain); renamed `kProgramFile`.
7. **From the review (2026-09-29), each forced by a measured defect:** VERBATIM blocks changed —
   `runTesterOnce` (a `runPressCount` parameter and `watch.arm(runPressCount)`: a touch in the gap never
   aborted; a `std::exception` handler: its text was lost), `pacerChainProven` (`&& watch.attachedTo(guard)`:
   an unattached watch drove 14.22 in past B with a green FINISHED), and the program's flow (`runOne`'s
   baseline, the task's `std::exception` handler, the chain refusal's words, and the boot line split into
   `testerBootLines()`'s three: 206 bytes at a real hash). `src/team/autons.{hpp,cpp}` are still
   byte-identical to §3.1. The brief's verbatim text is the coordinator's to amend.

## Evidence

- **Host suite:** 1,725 cases / 2,648,032 assertions before → **1,780 / 2,649,362** at the build → **1,796 /
  2,649,923** after the review's fixes (2026-09-29), 3 skipped, 0 failed.
- **The re-predicted weak-side table, re-run on the BUILT code** (`.claude/coordinator/scratch/comp-t/
  repredict_built/`): byte-identical to the re-prediction's `table_tank1.txt` and `sweep_tank1_0001.txt` —
  the 40 in FINISHES to ×0.723 (T+3.70 s) and first stops at ×0.722; the acceptance run to ×0.724 / ×0.723;
  ×0.95: FINISHED (39.70, 0.01, 0.06°) T+1.85 s; ×0.65: ROUTINE STOPPED at moveTo, MOTION_TIMEOUT, END
  (39.38, −0.93, 0.56°). Host test plant only.
- **The review's mutations (2026-09-29):** R01–R36 plus R07b, R14b, R33b — every one RED on a fresh binary
  and restored byte-exact (`scratch/comp-t/review-fix/mut/results*.jsonl`); R26–R28 and R33 survived the
  first version of their tests and were closed by strengthening them.
- **Mutations:** all 37 of the brief's (1–37, 3 split as 3a/3b; 18 and 19 through the gate's own build and
  verdict, 20 as self-test cases 31/32) and 10 of the executor's (A1–A10) — every one observed RED on a fresh
  binary and restored byte-exact; A2 survived once and closed a real hole (deviation 3). **The whole set was
  re-run on the FINAL tree by the second pass** (45 host mutations 45 RED; 18 and 19 RED through the gate;
  every file hashed before and after) — the live log's last section.
- **Gates, final lines on the final tree** (the live log has every build's line):
  - `src_build_gate.py check`: "src build gate: PASS (all 7 builds link; …)" — each of bench, xdrive, tank,
    tank-drive, tank-library, tank-calibrate and **tank-autontest** "exit 0, 10/10 TUs compiled, warnings: 7
    vendor-ignored, 0 disallowed, hash asserted, beacons (variant, robot) and program checked"; step 0:
    `[chain-table] 7 builds, 3 robots`, `[program-table] 6 programs, 14 entry names`, `[include-root] 31
    source files`, `[printed-path]` clean.
  - `src_build_gate.py self-test`: "OK (194 detector cases, each against a real build)" → **"OK (198 detector
    cases …)"** after the review (case 33); `check` re-run after the review: PASS, all 7 builds, 0 disallowed.
  - The ARM header gate: all 193 headers in one TU, and `run_guard.hpp`, `hal/pros/block_sink.hpp`,
    `hal/block_char_sink.hpp`, `competition/auton_registry.hpp` alone — PASS.
  - The CI greps: the library PROS-free outside `hal/pros/`; the core sim-free.
  - `api_doc_tool`: self-test OK; coverage "2950 public entities across 157 headers, all documented";
    check-fresh exit 0; check-examples "503 quoted lines, all verbatim (6 source files); 5 listings of
    whole-run files, each one unbroken run"; removability clean.
  - `doc_staleness_audit.py`: self-test OK (10 cases); run clean. `briefing_status.py check`: current.

## The coordinator's merge verification (2026-09-29, 21:36–22:15 EDT)

The run's transport commit `393fbd8` (worktree `comp-t`) was squash-merged onto `a5bd83f` (TANK1 Part B, which
landed while this chunk was in review): seven conflicts, all in shared documents, each resolved to the merged
state (the live log lists them). On the merged tree: the suite **1,808 | 1,808**, 2,652,625 assertions, 3
skipped; `src_build_gate.py` self-test OK (198 cases) and check PASS on all seven builds; the ARM header gate
over all 193 headers; both CI greps; `check-fresh`, removability, the staleness audit and the briefing — all
clean, and no plant left in `src/`, `include/`, `test/` or the Makefile. **Ten coordinator mutations on the
safety paths, each on a fresh binary, all RED:** no button aborts; button A never aborts; the chain proof blind
to which guard the watch is attached to, or passing with no pace through the chain; an existing log truncated
when the append open fails; RUN without the calibration record (D9); a controller drop not aborting; RUN on a
field connection outside autonomous; a throwing mechanism skipping the later ones in `RunGuard`; a program
error leaving the mechanisms unsafed. The table, with the tests that caught each, is in the live log's last
section. **D10 is settled** (below), and the brief's changed VERBATIM blocks are named in its header note.

## Documentation (the same change)

Guide chapter 16 (NEW) — the students' file quoted as two unbroken listings, held by `check-examples`; the
guide README (row, maintenance paragraph), glossary, chapters 7 and 14; README's `src/` list; the register
(HA-177, HA-178, the header's counts, HA-161's Source); the changelog (API 2.3, "Added and fixed since 2.3");
the FAQ (the append sentence); `docs/api` regenerated (`block_char_sink.md` NEW; `run_guard.md`,
`pros-block_sink.md`, `auton_registry.md`'s anchors, the index pages, `mkdocs.yml`); the roadmap ("Now", the
F12 row's note, a `[~]` M4 row); `build-order.md` (Next, COMP's DoD and status, the deviations row's FOUND
sentence); `HANDOFF-remaining-work.md` (L2, L2t, S1b, §7, the program list); `RESUMING.md` (seven builds);
`src/team/README.md`; the worksheet's Station M (and Station A, the DO NOT list, "Before the helper
leaves"); the COMP brief's amendments (B8, B18, B25, B29, NEW B30 — B5 untouched, T4 Part 0's); gained-caller
notes atop the SAFE-SAVE, L9 and T4 briefs; `guide-maintenance.md` rule 1 (the team files a chapter quotes —
now the auton file too) and HANDOFF S1 (the first uploaded program that commands a `moveTo` is the tester, at
S1b), both found by the second pass's prose review; comments in `src/main.cpp`, the Makefile, `build_selection.hpp`,
the gate, `ci.yml`, `boot_source.hpp` / `.cpp`, `teleop_program.{hpp,cpp}`, `xdrive_program.{hpp,cpp}`,
`fault_latch_hook.hpp`, `test/boot_source_test.cpp`, `test/xdrive_config_test.cpp`, `test/CMakeLists.txt`.
**Deliberately not touched:** historical records (other chunks' PROGRESS/COMPLETED logs, dated status
paragraphs such as the roadmap's R3d Part 2 "six builds"), and TANK1's own sections (TANK1 Part B is in
flight in another worktree).

## For the coordinator

- **Merge with TANK1 Part B**: both chunks add a section at the top of `docs/changelog.md`'s API 2.3, and both
  may touch the Next block of `build-order.md` and the roadmap's "Now"; COMP-t touched no `diag/` file and no
  result-line code.
- The Next block and the roadmap were written as they should read AFTER this change is committed.
- The self-test pair is 31/32, and case 33 is the review's (a build dropped from `VARIANTS` alone); COMP-b's
  pair is now 34/35 (B29 amended).
- **The briefing's position counts this record** ("33 of 63"), while `build-order.md` calls COMP-t a part of
  COMP, not a chunk (the review's finding D10). `briefing_status.py` counts every `-COMPLETED.md` (R1a and
  R1b are counted the same way). Your call before committing: a 64th row, or a record name the tool skips.
  **SETTLED at the merge (2026-09-29):** the tool skips it. A record or log named `<CHUNK>-<part>` with a
  lower-case letter in the part is a PART (`PART_NAME`, `chunk_of()`): its record is not a completed chunk and
  its log keeps the chunk open, so the briefing still reads 33 of 63 with COMP open; `R1a`, `R2a` and `R5a`
  have no hyphen and stay chunks. The rule has a `self-test` (17 cases), run by the doc gates and CI.
- **The brief's VERBATIM blocks** that the review changed (deviation 7) need amending in the brief, as do
  B30's copy notes (done in the COMP brief: the chain proof's link check, the `std::exception` handler, the
  PRESENT mechanisms). **Done at the merge:** the brief's header note names the three changed blocks and says
  the tree governs them; §3.1 is unchanged.
- `RunGuard`'s throw-path fix is library code from an earlier chunk (F2/COMP-a), fixed where the flaw
  was, per the standing rule; it changes nothing unless an adapter throws.
- Station M's first floor run is the first time an uploaded program commands a `moveTo`; whether that
  closes R3b is a ruling against R3b's DoD, not claimed here.
- `docs/internal/ORIENTATION.md`'s "Where it stands (2026-09-24)" is a dated snapshot, stale on counts older
  than COMP-t (Calibrate and Teleop have run since; the claws are in Teleop) and silent on the tester; a
  refresh is outside COMP-t's §6 list — left for the coordinator. **Done at the merge**, re-read against the
  logs: Calibrate first ran on robot two 2026-09-24 and the EARLIER, hand-wired Teleop drove it from the record
  2026-09-25; today's Teleop has never run (HANDOFF's program list says so), so "Teleop has run" holds only for
  the earlier version, and the page says which.
- Eleven files this chunk changed carry OLDER "simulator" wording (none added here — the live log's prose
  table); not swept, as it is not this chunk's prose.
