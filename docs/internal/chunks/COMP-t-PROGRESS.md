# COMP-t — live progress log ("shulib Auton Tester" and the students' auton file)

> Appended in real time by the executor. The brief is
> [`COMP-t-auton-tester.md`](COMP-t-auton-tester.md) (the team lead's answers D1–D9 as recommended, D10
> superseded by TANK1, slot 7 until T4 Part 3; the "Re-predicted after TANK1 Part A (2026-09-28)" note
> OVERRIDES the older predictions). **Executor model: Opus 5.5 (1M context)**, one executor run (D8).
> Worktree `.claude/worktrees/comp-t`, branch `comp-t`, from `e124b88` (SRC1, T4 Part 1a and TANK1 Part A
> committed there). **Nothing is committed by the executor.** TANK1 Part B is being built in parallel in
> another worktree (it touches `diag/` and the result-line code); any file it names that COMP-t must touch is
> logged here.

## 2026-09-28 14:13 EDT — start

- Read `RESUMING.md` (standing rules: docs in the same change; prose review logged, grep BEFORE claiming;
  adversarial tests; every mutation on a fresh binary, observed red, restored; backups under
  `.claude/coordinator/scratch/comp-t/`, never `/tmp`).
- Read the brief in full (2389 lines).
- Tree: `git status` clean at `e124b88`.
- **Baseline (before any change), measured 14:20:** `python3 tools/briefing_status.py generate` (the new
  PROGRESS file made the committed briefing stale — the doc-gate stamp FAILED on exactly that, as it
  should), then the host build and `./build/test/shulib_tests`: **1,725 cases / 2,648,032 assertions, 3
  skipped, 0 failed**. Briefing regenerated again after the suite existed.

## 2026-09-28 ~14:25–15:05 — the code written (nothing built yet)

- **Library (additive, each as briefed):** `include/shulib/competition/auton_registry.hpp` — `routeWhy[192]`
  and the outer copy `%.150s` (§0.5.1, with the reason in a comment); `include/shulib/sequence/run_guard.hpp` —
  `RunGuard::pacesTotal()` (incremented FIRST in `pace()`, run or not; `<cstdint>` added; banner note);
  `include/shulib/hal/pros/block_sink.hpp` — `ProsFileMode`, `AppendOpen`, `openForAppend()` (the ONE decision:
  `rb` probe, `ab`, `wb` only when absent), the three-argument constructor, the old constructor DELEGATES with
  `Truncate`, `appendOpen()`; banner section "THE APPEND MODE"; `include/shulib/hal/block_char_sink.hpp` — NEW,
  `hal::BlockCharSink` (one write + one flush per line, dropped and counted). `write()` is declared `noexcept`
  (the overlay's was not; ICharSink's contract says "MUST NOT throw" and the fake block sink's calls are
  noexcept, so the stronger spelling is free).
- **The students' file:** `src/team/autons.{hpp,cpp}` copied from §3.1 by line extraction
  (`sed -n '965,998p'` / `'1000,1078p'` of the brief, fences dropped) — `diff` against
  `comp-t-repredict/fix2/students/team/` (the copy the re-prediction ran) is EMPTY for both files.
  sha256 `c2f54410…` (cpp), `73fe9b4a…` (hpp). **No change forced so far.**
- **The pure headers:** `src/programs/auton_abort_watch.hpp` (the probe's `src1sim2/` copy; banner extended
  with WHAT ABORTS and the COMP-b note; a line-number cite replaced by a quoted cite), `auton_tester_run.hpp`
  (§3.3's verbatim `TesterRunRecord`, `runTesterOnce`, `pacerChainProven`, `stopAfterProgramError`; the
  result word, `buttonNamesOf`, and `testerRunLines` written to §3.3's line spec — see "Deviations" below),
  `auton_tester_panel.hpp` (written from §3.4's rules; the probe's sketch as the start).
- **The program:** `src/programs/auton_tester_program.{hpp,cpp}` — §3.5's verbatim `readTouch`,
  `ProsAbortInputs`, `readFacts`, `ScreenRunObserver`, `runOne`, `stopTheTester`, `testerTask`,
  `autonTesterInitialize`; the painter helpers as the probe; the refusal painter wrapped like Teleop's.
- **Wiring:** `src/build_selection.hpp`, `src/main.cpp`, `Makefile`, `tools/src_build_gate.py` (tables,
  docstring, self-test cases **31/32** — SRC1 took 28–30), `.github/workflows/ci.yml`, `test/CMakeLists.txt`
  (`target_sources` + `SHULIB_DOC_INPUTS`), `tools/api_doc_tool.py` (APPENDED `src/team/autons.cpp` to
  `EXAMPLE_SOURCE_GLOBS` AND to `WHOLE_RUN_SOURCE_GLOBS` per the coordinator's ruling; self-test fixture gains
  the auton file and three cases). Comments: `src/common/boot_source.hpp` (the host-build sentence and
  `buildOptions()`'s doc, R10), `test/boot_source_test.cpp` :3, `test/xdrive_config_test.cpp` :3.

### Deviations from the brief so far (each forced, with its reason)
1. **The throw line's `what` precision is `%.88s`, not §3.3's `%.120s`.** §3.3 also requires `%.29s` on the
   auton's name; with both, the line's worst case (a 29-character name, run 99999) is 4+5+3+29+2+13+32+120+23
   = 231 bytes, past TermSink's 199. `%.88s` makes it exactly 199. The full text still reaches serial on the
   fault latch's `[ERROR]` line (the precondition handler raises it) and the screen shows it wrapped.
2. **`TesterRunLines` gains `bool cut`** — set when any `snprintf` would have written past 199 — so test (23)
   can fail on a cut line instead of reading a 199-byte truncation as "under 200".
3. **`TesterView` gains `droppedLines` and `unsafeLines` counters** (a line past eight; a line that fails
   `ui::screenText` BEFORE the safety copy cuts it) — the "never silently" rules of §3.4 made countable, so
   (24) measures before `copyScreenText`.
4. **A REFUSED run's lines 1–2 say so** ("runAuton refused it before the guard armed; nothing moved" / "the
   guard never armed") instead of printing a hand-off latency computed from an `armedAt` of 0.
5. **The refusal painter's default footer is wrapped over two SMALL rows** — the probe printed
   "Nothing is powered. The full text is on the serial log." (55 characters) as ONE SMALL line, which the
   50-character budget would cut. Both footers are `static_assert`ed to fit two rows.

## ~15:05–15:38 — tests written; the first builds

- **A trial V5 build** (`rm -rf bin .d && make ROBOT=tank PROGRAM=autontest`, before any test existed):
  EXIT 0, every warning in the log the vendored `lv_obj_style.h` one (ignored by path) — our code clean.
- **Tests (NEW):** `test/auton_tester_rig.hpp` (the program's chain on the plant: Chassis → RunGuard →
  REAL AbortWatch → a probe layer in the painter's place → PlantPacer; the watch's inputs go through
  `abortInputsFrom(testerTouchFrom(...))` on a PROS-field-named struct, so every run test exercises the PROS
  mapping), `test/autons_test.cpp` (1)–(5), (3b), (3b′), (3c), (4b); `test/auton_abort_watch_test.cpp`
  (6)–(13b); `test/auton_tester_run_test.cpp` (14)–(23), (21b), (21c); `test/auton_tester_panel_test.cpp`
  (24)–(30), (27b); `test/auton_tester_source_test.cpp` (the source pins); `test/block_char_sink_test.cpp`
  (32). **Added to existing suites:** `test/pros_block_sink_adapter_test.cpp` (31: the four branches, two
  boots through the shim, no card, the two-argument constructor still truncates),
  `test/sequence_run_guard_test.cpp` (33).
- **First host build:** `docs/api` stale (expected: new `///` in three headers) → `api_doc_tool.py generate`
  (auton_registry.md line anchors, run_guard.md, pros-block_sink.md, NEW block_char_sink.md, README,
  all-entities, mkdocs.yml nav). Two compile errors in my tests (`-Wsign-conversion` in the literal
  scanner; `-Wmissing-field-initializers` on designated initializers of a struct with a `std::string`) —
  fixed.
- **First suite run: 3 failures, all in the tests, each read and fixed at the root:** (29) called
  `showCountdown` with no entry (the program never does — the countdown always follows READY; the model's
  `registry.at(-1)` threw, which is the right loud failure); (32) my hand-counted byte length was 58, the
  string is 57; (33) asserted `wait(0.02)` = 2 paces, but floating accumulation made it 3 — re-graded
  against the plant's own pace count (the guard's inner pacer), which is what "every pace" means.
- **Suite now: 1,780 cases / 2,649,347 assertions, 3 skipped, 0 failed** (baseline 1,725 / 2,648,032: +55
  cases).

## ~15:39 — the re-predicted table re-run on the BUILT code

`.claude/coordinator/scratch/comp-t/repredict_built/`: the coordinator's `comp-t-repredict/repredict.cpp`,
UNCHANGED, rebuilt by a new `build.sh` against THIS worktree — its own `src/team/autons.cpp`,
`src/programs/*.hpp`, `include/`, `test/` (no probe copy, no overlay) — at the same flags (`-O2`, the
warning set, `-Werror`).
- `./rp_built table > table_built.txt`: **byte-identical to `comp-t-repredict/table_tank1.txt`** (sha256
  `2346495921ff…` both). Every cell of the brief's after-TANK1 table holds on the built code.
- `./rp_built sweep 0.001 > sweep_built_0001.txt`: **byte-identical to `sweep_tank1_0001.txt`** (2 004 lines).
  The 40 in finishes to ×0.723 (T+3.70) and first stops at ×0.722; the acceptance run finishes to ×0.724 and
  stops from ×0.723 — as the brief says.
- `./rp_built verbose 1 0.65` / `verbose 1 1.0`: the BUILT `testerRunLines` and RESULT page (the brief's
  goldens plus §3.3's additions): ×0.65 line 3 reads `run 1: routine STOPPED, 0 of 1 steps done, stopped at
  moveTo (motion TIMEOUT); faults this run: 1, last MOTION_TIMEOUT -- see the [ERROR] line` (142 bytes);
  line 4 `… END estimate (39.38, -0.93) facing 0.6 deg LIVE -- measure the tape`; RESULT `END   (39.4, -0.9)
  0.6 deg` / `faults this run: 1, last MOTION_TIMEOUT`. ×1.00: `run 1: routine OK, 1 of 1 steps done; faults
  this run: none`; `END   (39.8, 0.0) 0.0 deg` (no trailing space).

## ~15:40–15:48 — mutations (each on a fresh binary, observed, restored)

Harness: `.claude/coordinator/scratch/comp-t/mut/harness.py` + `spec.json` (the exact old/new text of every
mutation) → `results.jsonl`. Per mutation: back the file up in `mut/` (`m<id>_<file>.orig`), replace ONE exact
text (asserted to occur once), bump the mtime, `make shulib_tests/fast` (the test binary rebuilt — never a
stale one), run the named cases, restore byte-exact from the backup (asserted), bump the mtime. Gate-level
ones through the gate's OWN `run_build()`/`verdict()` (`mut/gate_mut.py`). After all of them, a full rebuild
and the whole suite: green. `git status` after: only this chunk's changes (the two borrowed files,
`motion_config.hpp` and `match_runner.hpp`, show no diff).

| # | mutation (the brief's numbering) | killed by | observed |
|---|---|---|---|
| 1 | the watch requests only on the latch EDGE | (12), (22) | RED 2/2 |
| 2 | `abortCauseOf` without ControllerLost | (6), (17) | RED 2/2 |
| 3a | touch compared `> baseline + 1` | (6) | RED |
| 3b | `testerTouchFrom` reads `release_count` as the press count | (6b) | RED |
| 4 | `arm()` keeps a stale baseline | (9) | RED |
| 5 | `arm()` baselines from an error read | (10) | RED |
| 6 | `arm()` keeps the old latch | (20) | RED |
| 7 | the countdown loop without its cause check | (15) (the return-time bound) | RED |
| 8 | runAuton after a countdown abort (inverted `if`) | (15) | RED |
| 9 | `disarm()` moved before runAuton | (18), (19) | RED 2/2 |
| 10 | `runAllowed` without the controller term | (25), (26) | RED 2/2 |
| 11 | READY keeps PICK's edge (`showReady(choice, edgeCount_)`) | (27) | RED |
| 12 | AGAIN at RUN's slot | (28) | RED |
| 13 | `showResult` without `entry_ = rec.entry` | (30) | RED |
| 14 | the students' 40 → 44 | (3) | RED |
| 15 | `hardStopAt` 31 s | (1) | RED (registerAutons threw) |
| 16 | `kAutonPeriods.match` 60 | (1) | RED — a BUILD failure: (1)'s `static_assert` (the constexpr-ness check) fires first |
| 17 | the program calls `runAuton` | pin "runs autons ONLY through runTesterOnce" | RED |
| 18 | main.cpp's tester `opcontrol` case calls `runTeleop()` | the gate, tank-autontest build | RED: "build tank-autontest is the AutonTester program, but bin/hot.package.elf links the Teleop program's shulib::bench::runTeleop … (step 8)" |
| 19 | `routeWhy` back to 160 | the gate, bench build | RED: "disallowed warning -Wformat-truncation= at include/shulib/config/route.hpp:454" — §0.5.1 reproduced |
| 20 | the self-test pair | `src_build_gate.py self-test` cases 31/32 | see the gate section below |
| 21 | `abortInputsFrom` hard-codes `controllerConnected = true` | (6b) | RED |
| 22 | `testerTouchFrom` drops the error mapping | (6b) | RED |
| 23 | `ProsAbortInputs::read()` passes `false` for disabled | pin "PROS's two reads" (its ONLY guard) | RED |
| 24 | Robot emplaced with `buildOptions()` instead of `opts` | pin "built WITH opts" | RED |
| 25 | `.pacer = &*st.watch` | pin "built WITH opts" | RED |
| 26 | `pacesTotal_` incremented only while running | (33), (13b) | RED 2/2 |
| 27 | `pacerChainProven` checks only the watch | (13b) | RED |
| 28 | the quiet tick removed | (14) | RED |
| 29 | `runTesterOnce`'s try narrowed back to runAuton | (21b) | RED |
| 30 | `lastFault` read without the count check | (3c) | RED |
| 31 | RESULT ignores `fingerDown` | (27b) | RED |
| 32 | `runAllowed` without the RECORD term | (25) | RED |
| 33 | `addLine`'s cap a hard-coded 7 | (24) | RED |
| 34 | `openForAppend` falls back to `"wb"` when the file existed | (31) | RED |
| 35 | `openForAppend` without the `"rb"` probe | (31) ×2 | RED 2/2 |
| 36 | `BlockCharSink` ignores a refused flush | (32) | RED |
| 37 | `stopAfterProgramError` without its per-mechanism try | (21c) | RED |
| A1 | (mine) the throw line back to §3.3's `%.120s` | (23) worst case | RED — the deviation's reason, measured |
| A2 | (mine) READY's EST line at `%.2f` (51 chars at the worst case) | (24) | **SURVIVED at first** — see below; RED after the fix |
| A3 | (mine) TANK1 off (`maxSteer` 0) | (3b), (4b) | RED 2/2 — (3b)/(4b) really depend on the steering |
| A4 | (mine) the rig's weak side never wired | (3b) (its TSTR clause) | RED |
| A5 | (mine) runAuton's heading re-seat removed (COMP-a's code) | (5) | RED |
| A6 | (mine) the SD log opened truncating | pin "the SD run log" | RED |
| A7 | (mine) RUN not re-checked against `runAllowed` at the release | (26) | first run a BUILD failure (unused `f`); re-run with `(void)f`: RED |

**A2 SURVIVED — a real hole in (24), fixed at the root.** The panel formatted each line into a buffer the size
of the SMALL budget + 1 (`char text[51]`), so a 51-character line was cut by `snprintf` itself BEFORE
`setLine()` could measure it — the silent cut (24) exists to catch. The format buffer is now twice the widest
budget (`char text[2 * ui::kSmallCols + 1]`, with the reason in a comment), so an over-budget line reaches
`setLine()` whole and is counted in `unsafeLines`. Re-run: A2 RED. (The same class cannot hide in
`testerRunLines`: its `cut` flag reads `snprintf`'s return value, not the buffer.)

## ~15:50 — the gate's first `check`: FAIL, on MY test (fixed)

`python3 tools/src_build_gate.py check` — all seven builds compiled and linked clean ("10/10 TUs compiled,
warnings: 7 vendor-ignored, 0 disallowed, hash asserted, beacons (variant, robot) and program checked", the
seventh `[tank-autontest]` among them; `[chain-table] 7 builds, 3 robots`; `[program-table] 6 programs, 14
entry names`; `[include-root] 31 source files`; `[printed-path]` clean) — but step 0's one-chain rule
FAILED: `test/auton_tester_source_test.cpp:170 declares one of THE ONE CHAIN's constants … const
std::string kProgram = "";` — my source-pin test named its file-path constant `kProgram`, one of the three
names only the chain may declare (a host test that declares one is an ODR hazard the rule exists to catch).
Renamed `kProgramFile`. The gate caught a real (if harmless here) violation of its own rule — as designed.
The self-test was already running when this was read; it is re-run after the fix (its "the real tree is
clean" cases read the tree as it is).

*(Times corrected 16:00: the headings above first carried estimated times that ran ahead of the clock;
they now follow the scratch files' mtimes — the trial V5 build 15:04, `table_built.txt` 15:39,
`results.jsonl` 15:45, `gate_check.log` 15:50.)*

## ~15:52–16:00 — the self-test's first run, the documentation, and the gate re-run

- The first `self-test` (launched right after `check`, before its log was read) stopped at case 5 — "the
  gate FAILS on the real, unplanted tree" — on the same `kProgram` name; `git status` afterwards shows no
  planted file left behind. **Re-run after the fix**, below.
- **Documentation written** (docs only while the gate ran): guide chapter 16 (NEW), the guide README (row +
  the maintenance paragraph — "the team's files under `src/team/` that a chapter quotes (today: … the team's
  auton file, `src/team/autons.cpp`, which chapter 16 quotes …)", never "the one exception"), the glossary
  (`abort (the auton tester's)`, `auton tester`), chapter 7 (the repo layout names `autons.cpp` and the
  tester), chapter 14 (a bullet: the program that will close the first loop exists and has not run); README's
  `src/` list; `docs/hardware-assumptions.md` (HA-177, HA-178 — index rows, bodies, the header's counts 158 →
  160 and "HA-162..HA-176 are not in it", HA-161's Source gains the tester's task); `docs/changelog.md`
  (API 2.3, "Added and fixed since 2.3"); `docs/faq.md` (the append sentence); the COMP brief (B8, B18, B25,
  B29 amended; B30 NEW; B5 untouched — T4 Part 0's); `build-order.md` (the Next block's head, COMP's DoD and a
  LANDED status, the deviations row's FOUND sentence pointing at TANK1's row); `roadmap.md` ("Now" and a new
  `[~]` M4 row); `HANDOFF-remaining-work.md` (L2 gains COMP-t, Part S gains S1b = Station M before K, §7's
  order, the program list "seven builds"); `RESUMING.md` (seven builds, `PROGRAM=autontest`);
  `src/team/README.md` (the autons); the worksheet — Station M (M.0–M.9 and what to send back), Station A's
  fifth program, the DO NOT list, "Before the helper leaves".
- **Stale "six builds" in `src/` comments** (found by the prose-review grep): `src/common/boot_source.cpp`,
  `src/programs/teleop_program.cpp` (×3), `src/programs/xdrive_program.cpp` (×3), `src/common/fault_latch_hook.hpp`
  (×2) → seven (edited after the self-test ended, before the re-run).

## ~16:10 — every VERBATIM block of the brief checked against the files (a script, not by eye)

Each ```` ```cpp ```` block of the brief was extracted by line and searched for as ONE contiguous substring of
the file it belongs to: §3.2's `abortCauseOf` (1127) and the added `testerTouchFrom`/`abortInputsFrom` (1192)
→ `auton_abort_watch.hpp`; its `arm()` and `pace()` excerpt (1145 — two functions with other members between
them in the class) → each function verbatim; §3.3's `TesterRunRecord` (1236), `runTesterOnce` (1256) and
`pacerChainProven`/`stopAfterProgramError` (1327) → `auton_tester_run.hpp`; §3.5's PROS reads (1440),
`readFacts` (1462) and the whole flow — `ScreenRunObserver`, `runOne`, `stopTheTester`, `testerTask`,
`autonTesterInitialize` (1485, 213 lines) → `auton_tester_program.cpp`; §3.1's two files → byte-equal to
`src/team/autons.hpp` / `autons.cpp`. **All verbatim.** The program differs from the probe copy
(`src1sim2/programs/auton_tester_program.cpp`) only in the banner, `using calibration::TextSize;`, the
budget `static_assert`, the refusal literals and their `static_assert`s, and the refusal painter (wrapped like
Teleop's, the footer over two rows — deviation 5).

## ~16:05 — the gate `check` re-run after the rename: PASS, seven builds

`gate_check2.log`: `[one-chain]` clean ("by ONE chain setting program, robot and beacon in every branch …");
`[chain-table] 7 builds, 3 robots`; `[program-table] 6 programs, 14 entry names`; `[include-root] 31 source
files in src/ and 4 folders`; `[printed-path]` clean; each of the seven builds "exit 0, 10/10 TUs compiled,
warnings: 7 vendor-ignored, 0 disallowed, hash asserted (v0.1.1-350-ge124b88-dirty), beacons (variant,
robot) and program checked"; **"src build gate: PASS (all 7 builds link; …)"**. The ARM header gate (one TU
of all 193 headers under `include/shulib`, the V5 flags, `-Werror`): PASS; `run_guard.hpp`,
`hal/pros/block_sink.hpp`, `hal/block_char_sink.hpp` and `competition/auton_registry.hpp` each ALONE: PASS.
The CI greps: "library is PROS-free (all of include/shulib except hal/pros/)", "layering holds: core is
sim-free". `ci.yml`'s comment now says the grep covers nothing in `src/` and names the source pin that does.

## ~16:15 — the prose review (rule 4: the greps RUN, the hits READ, before this was written)

Scope of every grep: `README.md docs src include tools test/CMakeLists.txt Makefile .github mkdocs.yml`,
`docs/api/` excluded (generated). "All" counts every hit; "live" leaves out `docs/internal/chunks/` (briefs and
logs — history) and `docs/internal/{evidence,verify,transcripts}/`. Each live hit was read.

| grep | all | live | what the live hits needed |
|---|---|---|---|
| `slot 2` | 82 | 18 | Teleop's slot today — true until T4 Part 3; the tester's "slot 7, then slot 2" (guide 16, HANDOFF L2t) is right; nothing stale |
| `slot 7` | 22 | 11 | all the tester's, all "until T4 Part 3" (main.cpp's table, Makefile, roadmap, guide 16, HANDOFF, worksheet) |
| `four programs` | 10 | 1 | build-order's T4 R5 row quotes the team lead — left; worksheet G.1 "upload all four programs" is Station G's own scope — left |
| `Four programs` | 5 | 0 | Makefile and main.cpp now "Five programs in five slots"; the COMP brief's old §3 line is overridden by B25 (amended) |
| `six builds` | 76 | 5 | FIXED in `src/`: boot_source.cpp, teleop_program.cpp ×3, xdrive_program.cpp ×3, fault_latch_hook.hpp ×2 → seven; HANDOFF's program list → seven; the roadmap's R3d Part 2 paragraph, build-order's dated Next histories and the gate's "six builds at 2c" are history — left |
| `four tank builds` | 8 | 0 | main.cpp :40 and build_selection.hpp now "five" |
| `fourth, fifth and sixth` | 3 | 0 | the Makefile now "fourth to seventh" |
| `never src/` | 3 | 0 | boot_source.hpp, boot_source_test.cpp, xdrive_config_test.cpp reworded (one PROS-free `src/` file is host-built) |
| `one exception` / `only exception` | 16 / 4 | 4 / 0 | none about the example scan (vexbuilder's `heading`, sd_sink's fault dump, the changelog's H-drive and xt notes) — the guide README says "the team's files under `src/team/` that a chapter quotes (today: …)" |
| `EXAMPLE_SOURCE_GLOB` | 8 | 1 | api_doc_tool's own history comment ("until then this was one glob, EXAMPLE_SOURCE_GLOB") — correct |
| `PROGRAM=` | 174 | 84 | every program list that enumerates the programs now has `autontest` (Makefile, gate docstring, ci.yml, RESUMING, main.cpp's table, README via guide 16, guide 07, HANDOFF); the rest name one program each |
| `autonomous()` | 141 | 92 | main.cpp :52-55 and :213-216 reworded as briefed; `teleop_program.hpp` and `xdrive_program.hpp`'s "autonomous routines run only in the competition program's autonomous()" comments gained "and, on the bench, the Auton Tester's RUN, through the same library call"; the printed strings (xdrive_program.cpp's runtime line) left — they state the ruling, and changing a printed string changes a package for no reader |
| `auton hook` / `Teleop hook` | 8 / 1 | 3 / 0 | HANDOFF S1's "until L2 (or a minimal auton hook) exists" — S1b now follows it and names Station M; build-order's history; the tester's own header says it is NOT one |
| `runAuton` | 107 | 48 | read; guide 03's re-seat sentences, changelog, roadmap — true; the new text agrees |
| `registerAutons` | 38 | 15 | all new (the file, the program, guide 07/16, README, roadmap, build-order, api_doc_tool) |
| `acceptance run` | 61 | 17 | README's competition row, the changelog, the register's HA-179 — true |
| `40 in` | 182 | 58 | guide 04 ("a 40 in leg finish with one side up to 25 % weak") agrees with the built table (×0.75 finishes, ×0.723 is the limit); the register's HA-179 numbers are TANK1's |
| `usd/` / `fopen` | 184 / 91 | 80 / 30 | the FAQ gained the append sentence; the register's HA-122/HA-177; no `fopen` or `/usd/` literal in the tester (pinned) |
| `HA-161` | 34 | 10 | the register's Source gained the tester's task; main.cpp and the tester's headers cite it |
| `HA-177` / `HA-178` | 33 / 33 | 16 / 12 | all new, consistent (register, FAQ, changelog, guide 16/14, the code's comments) |
| `buildOptions` | 43 | 8 | boot_source.hpp's doc reworded (R10); build-order's B8 mention is the COMP brief's, amended |
| `pacesSeen` | 28 | 15 | guide 12 and the changelog describe the run's own count — unchanged meaning; `pacesTotal()` added beside it |
| `FINISHED` | 90 | 19 | guide 16, roadmap, worksheet M — all "expected on the host test plant", none a robot claim |
| `simulator` | 59 | 33 | **no line this chunk added says it** (`git diff \| grep '^+' \| grep -ci simulator` = 0; the new files' only hit is this log naming the grep). Eleven files this chunk changed carry OLDER hits (README, changelog, guide 07/14, HANDOFF, build-order, roadmap, the L9 brief, api_doc_tool, docs/api, the briefing) — pre-existing prose, not this chunk's to sweep; recorded for the coordinator |

## ~16:10–16:25 — the last changes, and every gate's FINAL line on the final tree

- **Last changes before the final gates:** the two header comments (`teleop_program.hpp`, `xdrive_program.hpp`,
  the prose review's `autonomous()` row); exact goldens added to (23) for the shapes a real run does not produce
  here — an abort by two buttons mid-run, the deadline cut, the hard floor, NOT STARTED, a throw inside the run —
  each checked RED under a mutation (`mut/spec_goldens.json`: A8 the "HARD STOP fired" suffix dropped → (23)
  red; A9 an end-action time printed for an end action that never ran → (18), (23) red; A10 the whole-boot fault
  count printed as this run's → (23) red). HANDOFF's L2t and `src/` paragraph brought to "landed".
- **`python3 tools/src_build_gate.py check`** (`scratch/comp-t/gate_check_final.log`):
      [one-chain] the build switches are read in src/build_selection.hpp only (src/, include/shulib/, test/ searched, CMake included), by ONE chain setting program, robot and beacon in every branch; no allowlisted file; no selection name a macro; 7 per-robot names named only in the tables, test/ and static_asserts
      [chain-table] 7 builds, 3 robots, matching src/build_selection.hpp's chain
      [program-table] 6 programs, 14 entry names, matching src/main.cpp
      [include-root] 31 source files in src/ and 4 folders (common/, programs/, robots/, team/): every include of one written from src/, no path a second root could answer
      [printed-path] every src/ path in a string literal under src/ and include/shulib/ names a file or folder that exists
      [bench     ] exit 0, 10/10 TUs compiled, warnings: 7 vendor-ignored, 0 disallowed, hash asserted (v0.1.1-350-ge124b88-dirty), beacons (variant, robot) and program checked
      [xdrive    ] exit 0, 10/10 TUs compiled, warnings: 7 vendor-ignored, 0 disallowed, hash asserted (v0.1.1-350-ge124b88-dirty), beacons (variant, robot) and program checked
      [tank      ] exit 0, 10/10 TUs compiled, warnings: 7 vendor-ignored, 0 disallowed, hash asserted (v0.1.1-350-ge124b88-dirty), beacons (variant, robot) and program checked
      [tank-drive] exit 0, 10/10 TUs compiled, warnings: 7 vendor-ignored, 0 disallowed, hash asserted (v0.1.1-350-ge124b88-dirty), beacons (variant, robot) and program checked
      [tank-library] exit 0, 10/10 TUs compiled, warnings: 7 vendor-ignored, 0 disallowed, hash asserted (v0.1.1-350-ge124b88-dirty), beacons (variant, robot) and program checked
      [tank-calibrate] exit 0, 10/10 TUs compiled, warnings: 7 vendor-ignored, 0 disallowed, hash asserted (v0.1.1-350-ge124b88-dirty), beacons (variant, robot) and program checked
      [tank-autontest] exit 0, 10/10 TUs compiled, warnings: 7 vendor-ignored, 0 disallowed, hash asserted (v0.1.1-350-ge124b88-dirty), beacons (variant, robot) and program checked
    src build gate: PASS (all 7 builds link; policy: errors fail, vendor warnings path-ignored, no category allowed, every other warning fails; defines token-exact; one chain picks program and robot; variant and robot beacons asserted; programs asserted; src/'s include root unambiguous; every printed src/ path exists)
    CHECK EXIT 0
- **`python3 tools/src_build_gate.py self-test`** (`gate_selftest_final.log`): **"src build gate self-test: OK
  (194 detector cases, each against a real build)"** — 189 before this chunk by SRC1's record (not
  re-measured at this chunk's baseline), + case 31's four expectations (the FAIL, the define named, the
  tank-autontest beacon absent, the tank beacon present) + case 32's one (the real build PASSES) = 194.
- **The ARM header gate:** one TU of all 193 `include/shulib` headers at the V5 flags with `-Werror` — PASS;
  `sequence/run_guard.hpp`, `hal/pros/block_sink.hpp`, `hal/block_char_sink.hpp`,
  `competition/auton_registry.hpp` each alone — PASS.
- **The CI greps:** "library is PROS-free (all of include/shulib except hal/pros/)"; "layering holds: core is
  sim-free".
- **`api_doc_tool`:** self-test OK; check-coverage "2950 public entities across 157 headers, all documented";
  check-fresh exit 0 (silent on success); check-examples "503 quoted lines, all verbatim (6 source files); 5
  listings of whole-run files, each one unbroken run" (chapter 16's two listings of `autons.cpp` among them);
  check-removability "no public doc references docs/internal/".
- **`doc_staleness_audit.py`:** self-test "OK (10 detector/filter cases)"; run "clean (28 live docs …)".
- **`briefing_status.py`:** regenerated before the full build and again after the suite existed; `check`:
  "generated block current".
- **The host suite (final):** **1,780 cases / 2,649,362 assertions, 3 skipped, 0 failed** (baseline 1,725 /
  2,648,032).
- **A rule slip, recorded:** the explicit `api_doc_tool` re-runs above captured their output in a transient
  `/tmp/x.<pid>` file (removed at once) — the standing rule puts probe files under
  `.claude/coordinator/scratch/comp-t/`, never `/tmp`; no probe or evidence file was written there.
- `git status`: only this chunk's changes; nothing committed; no planted file left by any mutation or gate
  self-test (the two borrowed files the mutations touched, `motion_config.hpp` and `match_runner.hpp`, show no
  diff).

## 2026-09-28 16:47 EDT — a second executor pass: VERIFY, then report (nothing re-written blind)

The first pass ended at 16:31 with its COMPLETED record, but no final report reached the coordinator. This
pass is the same task relaunched. It takes nothing above on trust: it re-checks the build against the brief
by script, re-runs the suite and every gate on the tree as it stands, re-runs the re-predicted table on the
built code, and re-runs the mutation harness on the FINAL tree (the first pass's mutations ran before its
last test and comment edits of 16:10–16:25). Everything it finds is appended below. `git status` at 16:47:
this chunk's 46 modified and 19 new files, nothing else; only `PROJECT-BRIEFING.md` is newer than the
COMPLETED record (the briefing regenerated last, as required).

### 16:53–17:10 — what the second pass re-checked, and found

- **The build against the brief, by script** (`scratch/comp-t/verify2/`): `src/team/autons.hpp` and `.cpp`
  are byte-identical to §3.1 (lines 966–997 and 1001–1077 of the brief, fences dropped; sha256 `73fe9b4a…` /
  `c2f54410…`, every line ≤ 99 characters); every other VERBATIM block is one contiguous substring of its
  file — `abortCauseOf`, `arm()`, `pace()`, `testerTouchFrom`/`abortInputsFrom` (auton_abort_watch.hpp);
  `TesterRunRecord`, `runTesterOnce`, `pacerChainProven`/`stopAfterProgramError` (auton_tester_run.hpp);
  `readTouch`/`ProsAbortInputs`, `readFacts` and the 213-line flow (auton_tester_program.cpp).
- **The re-predicted table on the FINAL tree:** the coordinator's `comp-t-repredict/repredict.cpp`,
  unchanged, rebuilt by `repredict_built/build.sh` against this worktree (`verify2/repredict/rp_final`):
  `table` and `sweep 0.001` byte-identical to `table_tank1.txt` (`2346495921ff…`) and `sweep_tank1_0001.txt`
  (`1971915625e4…`, 2 004 lines). One word change per sweep: the 40 in FINISHES to ×0.723 (T+3.70 s, END
  (39.55, ∓0.20)) and first STOPS at ×0.722, either side; the acceptance run finishes to ×0.724 and stops
  from ×0.723.
- **Every mutation re-run on the final tree** (the first pass's ran before its last edits, and the re-runs
  of A2 and A7, A8–A10 and gate mutations 18/19 had left no result file): `verify2/mut/spec_all.json` — the
  first pass's exact texts, A2/A7 as re-run, A8–A10 — each asserted to occur exactly once in the final
  files; the same harness, backups under `verify2/mut/`. **45 of 45 RED** (17:02–17:04; results.jsonl): the
  brief's 1–17, 21–37 (3 as 3a/3b) and A1–A10; 16 is a BUILD failure — (1)'s `static_assert` on
  `kAutonPeriods`, "static assertion failed: kAutonPeriods is a constant expression with the manual's 30 s
  and 60 s". Final rebuild exit 0. **Gate mutations through the gate's own `run_build()`/`verdict()`**
  (`verify2/mut/gate_results.jsonl`): 18 RED — "build tank-autontest is the AutonTester program, but
  bin/hot.package.elf links the Teleop program's shulib::bench::runTeleop — this build runs Teleop code
  (step 8)"; 19 RED — "disallowed warning -Wformat-truncation= at include/shulib/config/route.hpp:454 … writing
  53 bytes into a region of size between 41 and 109". 20 is self-test case 31 (below). Every file of the
  tree hashed before and after (1 030 files): identical but for this pass's own doc edits.
- **D7's scan, mutated** (`check-examples`, backups under `verify2/`): chapter 16's `40_in` → `44_in` ⇒
  exit 1, "NOT VERBATIM … (block 1)"; a whole line added to `autons.cpp` inside the quoted part ⇒ exit 1,
  "NOT ONE UNBROKEN RUN … a listing of src/team/autons.cpp"; both restored (sha256 back to `c2f54410…`).
- **The host suite** (up-to-date binary, 16:57): 1,780 cases / 2,649,362 assertions, 3 skipped, 0 failed.
- **Gates re-run on the final tree:** `src_build_gate.py check` PASS, all seven builds (17:05–17:07,
  `verify2/gate_check.log`); the ARM header gate (193 headers in one TU; `run_guard.hpp`,
  `hal/pros/block_sink.hpp`, `hal/block_char_sink.hpp`, `competition/auton_registry.hpp` alone) PASS; the
  two CI greps; `api_doc_tool` self-test / check-coverage (2950 entities, 157 headers) / check-fresh /
  check-examples (503 lines, 6 source files, 5 whole-run listings) / check-removability; the staleness
  audit's self-test and run.
- **Prose review re-run** (`verify2/prose_greps.sh`; the live hits of `six builds`, `four programs`, `one
  exception`, `auton hook`, `buildOptions`, `slot 7`, `slot 2`, `Five programs`, `HA-177`, `HA-178`,
  `FINISHED` and `EXAMPLE_SOURCE_GLOB` read) plus a wider sweep the brief's list does not name (`_driver.hpp`,
  `seven builds`, `six programs`, `check-examples`). **Found and fixed (the first pass had missed them):**
  1. `docs/internal/guide-maintenance.md` rule 1 named only T4's driver file among the team files a chapter
     may quote — now the auton file too (chapter 16), each listing one unbroken run, never the rest of
     `src/`. (`EXAMPLE_SOURCE_GLOB` with `-F` also matches the plural; word-bounded it is live only in
     `api_doc_tool.py`'s history comment, as the first pass said.)
  2. `HANDOFF-remaining-work.md` S1: "no uploaded program commands a `moveTo` … impossible until L2 (or a
     minimal auton hook) exists" — false after this change; now: no program S1 runs does; the first that
     does is the Auton Tester at S1b, and whether its floor runs close R3b is a ruling for when its log
     exists.
  3. `build-order.md`'s COMP status said "7 of the executor's" mutations — written before A8–A10; now all 10,
     re-run on the final tree.
  4. `COMP-competition-program.md`: a blank line before the `---` that follows the new B30 list.
  **Left, for the coordinator:** `docs/internal/ORIENTATION.md`'s "Where it stands (2026-09-24)" (a dated
  snapshot, stale on older counts too — Calibrate and Teleop have run since, the claws are in Teleop) says
  "six programs … plus two others"; refreshing it is outside COMP-t's §6 list.
- **`src_build_gate.py self-test` on the final tree** (17:11–17:21, `verify2/gate_selftest.log`): **"src build
  gate self-test: OK (194 detector cases, each against a real build)"** — case 31 is the brief's mutation 20
  (the Makefile's `override EXTRA_CXXFLAGS+=-DSHULIB_PROGRAM_AUTONTEST` dropped ⇒ FAIL naming the define, the
  tank-autontest beacon absent, a tank build wearing its label), case 32 the real build alone ⇒ PASS. The tree
  hashed after it: nothing planted left behind.
- **The host suite from a CLEAN build** (`rm -rf build/test`, the briefing regenerated first; 17:22,
  `verify2/build_final.log`: zero warnings; the doc-gate target in the build — coverage, freshness, verbatim
  examples, removability, briefing, staleness — passed): **1,780 cases / 2,649,362 assertions, 3 skipped, 0
  failed** (`verify2/suite_final.log`), the same as before the mutations. Then the briefing regenerated with
  the suite built, and `briefing_status.py check` run. **Nothing committed.** The second pass ends here; its
  report goes to the coordinator.

## 2026-09-29 10:56 EDT — a third pass: the reviewers' findings (builder again)

Three reviewers (safety, tests, docs) returned 22 findings on the tree the second pass left. This pass
re-checks each against the code and the reviewers' probes (`scratch/comp-t/review-safety-2/`,
`review-gatebuild/`, `review-docs/`), fixes the real ones at the root, re-runs every mutation it adds on a
fresh binary (backups under `scratch/comp-t/review-fix/`), re-runs every gate, and appends "COMP-t review
findings and their disposition" below. Nothing is committed.

### 2026-09-29 13:57 EDT — the third pass RESUMED (it was stopped at ~13:50 by the usage-limit pause)

The pass above was stopped mid-way and logged nothing past its header. What the resumption found, and
did, before anything else (`date` read 13:56:34 EDT):
- **A mutation plant was left in `src/`.** The stopped pass's harness (`scratch/comp-t/review-fix/mut/`,
  `spec.json` R01–R36) had finished R01–R12 (all RED, `results.jsonl`) and was INSIDE R13 when it was
  killed: `src/programs/auton_tester_program.cpp` still had the task's `catch (const std::exception& e)`
  handler deleted. Restored byte-exact from the harness's own backup (`R13_auton_tester_program.cpp.orig`,
  `cmp` clean), its timestamp bumped. Then the WHOLE tree was checked against the harness's
  `tree_before.sha256` (taken at 13:52:04, before its first mutation): `sha256sum -c` clean — nothing
  else planted. The Makefile's diff is this chunk's own (PROGRAM=autontest), no gate self-test plant.
- **What the stopped pass had already done** (read from the diffs against its `review-fix/bak/pre-review_*`
  backups, not from memory): the code and test fixes for the safety and tests findings — `run_guard.hpp`,
  `auton_abort_watch.hpp`, `auton_tester_run.hpp`, `auton_tester_panel.hpp`, `auton_tester_program.cpp`,
  `tools/src_build_gate.py`, and 18 new or changed test cases in seven test files. NOT started: every
  documentation finding (guide 16 and the worksheet are byte-identical to their pre-review backups),
  and R13–R36 of its mutation list. `src/team/autons.{hpp,cpp}` are still byte-identical to §3.1
  (sha256 `73fe9b4a…` / `c2f54410…`).
- This resumption keeps those fixes (each re-checked below against its finding), re-runs R13–R36 on fresh
  binaries with the same harness, does the documentation findings, re-runs every gate, and writes the
  disposition section. Nothing is committed.

## COMP-t review findings and their disposition (third pass, resumed; mutations 14:04 and 14:08 EDT by the result files' times; written while the gate ran)

Three reviewers (safety, tests, docs) returned 23 findings; overlaps are merged below (safety 4(b) and
tests 3 share the `std::exception` handler). Each was re-checked against the code before anything moved.
**Mutation evidence** (`scratch/comp-t/review-fix/mut/`, the harness unchanged in method: one exact edit, a
fresh `shulib_tests` build, the named cases, a byte-exact restore, the timestamp bumped): `results.jsonl`
(the stopped pass: R01–R12, all RED, on source byte-identical to the final — each `R*.orig` compared) and
`results2.jsonl`/`results3.jsonl` (this resumption: R07b, R13–R36, R14b, R33b). **Final: every mutation
RED.** R07 and R14 were first killed only by `-Werror=unused-parameter`, so each was re-run in a form that
compiles (R07b `return (void)guard, guard_ != nullptr;`, R14b `(void)runPressCount; watch.arm();`) — a test
kills both. **R26, R27, R28 and R33 SURVIVED the stopped pass's tests** — two real holes in its own fixes,
closed here (rows T3 and T4). The tree was hashed before each harness run and checked after (clean, bar
the worksheet this pass was editing in parallel).

| # | Finding | Verdict | What was done | Proof |
|---|---|---|---|---|
| S1 | Nothing checks mechanisms are safed at a run's end / on an abort (M1, M2, M4 survived) | **Real (major)** | Test (18b): FakeMechanisms in `present` through `runTesterOnce` — a clean run and B mid-run safe every one (B also cancels a registered claimant and releases its claim), B during the end action safes every one, a countdown abort touches none. Source pin: `st.present[st.presentCount++] = m;` and the span `{st.present.data(), st.presentCount}` handed to the run and to the last-resort stop. | R01 (`present.first(0)`) RED (18b); R02 (the lost `++`) and R03 (an empty span in runOne) RED on the pin |
| S2 | The chain proof misses the watch→guard link; an ignored abort reads a green FINISHED | **Real** | `AbortWatch::attachedTo(guard)`; `pacerChainProven` also requires it (the refusal's words now name "or the watch is not attached"). `abortIgnored(rec)` → the red word `ABORT IGNORED`, log line 1 `IGNORED the abort by <cause>`, RESULT line 1 `abort IGNORED: <cause>`. Tests (13b) extended, (20b) new. **A deviation from two verbatim blocks** (`pacerChainProven`, the flow's refusal words) — forced: the reviewers showed the unattached chain drives 14.22 in past B with a green FINISHED. | R04 (drop `attachedTo`) RED (13b)+(20b); R05 (no ABORT IGNORED) RED; R06 (no `attachGuard`) RED on the pin; R07b RED |
| S3 | An abort during the end action reads "end action FAILED" | **Real** | Log line 1 `end action CUT by the abort`, RESULT line 2 `end action CUT`; guide 16 "skips the end action — or cuts it short"; worksheet M.4.4 says what CUT means. Test (18c) new, (23) golden added. | R08, R09 RED |
| S4a | A throwing safe state → `std::terminate` through the guard | **Real — fixed at its root, in RunGuard** (the standing rule: a flaw in an earlier chunk is fixed there) | `safeMechanisms()` tries each mechanism, keeps the FIRST exception and rethrows it after the sweep; `RunUnwindGuard`'s destructor swallows its repeat (a second throw from a destructor during unwinding is terminate). Library behaviour change on the throw path only; changelog FIXED item; roadmap F12 row noted (open by design, no bump). Two tests in `sequence_run_guard_test.cpp`. | R10 (no try in the unwind) RED; R11 (rethrow inside the loop) RED |
| S4b | `catch (...)` drops a `std::exception`'s text | **Real** | `catch (const std::exception& e)` before `catch (...)`, copying `what()`, in `runTesterOnce` and in the task. **Verbatim-block deviation** (runTesterOnce, the flow) — forced: a student's `std::out_of_range` read "an unknown exception". Test (21d). | R12, R13 (the task's handler, pinned), R29 RED |
| S4c | A touch between RUN's read and `arm()` never aborts | **Real** | `AbortWatch::arm(pressBaseline)`; `runTesterOnce(..., runPressCount)` arms with the press count of the very read whose release touched RUN; the flow keeps that read (`TesterTouch t = readTouch()`) and passes `t.pressCount`. Fail-safe: the count only rises, so an earlier baseline can only abort sooner. **Verbatim-block deviation** (runTesterOnce's signature, runOne, the idle loop) — forced. Tests (9b), (15b); pin. | R14b, R15 (a fresh read), R16 RED |
| T1 | The run gate's inputs, the chain-proof refusal, RESULT's finger, the repaint rule — 6 mutations survived | **Real (major)** | Whitespace-collapsed body pins: `readFacts`, the chain-proof block through `return;`, `showResult(rec, t.sample.releaseCount, t.fingerDown)`, `!paintedOnce_ \|\| !sameButtons(next, painted_)`. (Moving `bootFromRecord`'s derivation into a pure function was not done: the pin kills P1, and the brief's `readFacts` is verbatim.) | R17–R22 (P1–P6) all RED |
| T2 | `sameButtons` on PICK untested (H4, H4b) | **Real** | Test (30) gains PICK frames: 0.0 s (grey) vs 0.6 s (armed) must differ; then each menu-button field changed on a copy (the chosen highlight among them) must differ; a header-only change must not. | R23, R24, R25 RED |
| T3 | `catch (...)` and both panic stops never exercised; route/NOT LIVE only on made-up records | **Real — and the stopped pass's (21e) did NOT close it** | (21d) a `std::runtime_error` and a non-standard throw, each recorded. (22b) a real NOT STARTED run (the IMU never ready) → `endLive` false, NOT LIVE. **(21e) rewritten here:** its pacer-layer throw is made safe by the scheduler's own F2 `WaitUnwindGuard` BEFORE the handler runs, so deleting a handler's `chassis.cancel()` survived (R26–R28, measured). Half (b) throws from the observer's `runStarted()` — outside every wait — with the drive left at 6 V under Coast; only the handler's stop can save it. (H3, the RouteRan branch: R30.) | R26, R27, R28 RED after the rewrite; R29, R30, R31 RED |
| T4 | The boot line overflows 199 bytes at a real hash and loses the SD path | **Real** | `testerBootLines()` — THREE lines (identity+hash; boot source+counts; SD state+path), every field bounded (beacon `%.40s`, hash capped at `diag::kMaxHashBytes`, boot word `%.11s`, path `%.60s`), a `cut` flag. **Verbatim-flow deviation** — forced (206 bytes at this repo's own describe). Worksheet M.2.2/M.7, guide 16, HA-177 quote the new lines. Test (23b); **its worst case was lengthened here** (a 64-byte beacon fitted even unbounded — R33 survived). | R32 RED; R33, R33b RED after the lengthening |
| T5 | Gate: a build dropped from VARIANTS alone passes | **Real** | `build_table_problems()` (VARIANTS against every per-build table, no duplicates) and a chain beacon must belong to a build in VARIANTS; self-test case 33. COMP brief B29: COMP-b's pair is now 34/35. | self-test (this pass's run, below) |
| T6 | (23)'s ×0.65 line 2 checked by its start only; the adapter's close unobserved (H13) | **Real** | (23) asserts the whole line 2 (`… end action ended T+5.13s, 513 paces, 0 cancels …`); the adapter's own close observed through the process's open descriptors (`/proc/self/fd`): on an existing file, exactly ONE more while the sink lives (its `"ab"` handle — the `"rb"` probe closed before the constructor returns), none after it. | R34 (brake settle 0.1→0.15) RED; R35 (a no-op close) RED |
| D1 | Docs say the suite runs every auton; it ran two | **Real (major)** — made TRUE | Test (1b): EVERY registered entry through `runTesterOnce` on the clean plant (never REFUSED / PROGRAM ERROR / NOT STARTED / HARD STOP / ABORTED / ABORT IGNORED, the end action run and succeeded, the drive safe, guard and watch let go); (1)'s count keeps `== 2` with a message telling the student to pin the new one. Guide 16 "Writing the next auton" says what both tests do. The ten claims stand, now true. | R36 (a third, route, auton) RED on (1b) |
| D2 | Guide 16's build command can upload the previous program | **Real (major)** | `rm -rf bin .d && make ROBOT=tank PROGRAM=autontest`, with why. | — (the reviewer's measurement) |
| D3 | M.7 cannot be done as written; invites erasing the record | **Real (major)** | M.2.2 is now the CREATED check; M.7.1 power-cycles, expects APPENDED, one run, copies the file, every run since M.2 in order; M.7.2 (optional) deletes ONLY the log, never formats, keeps `shulib_drivetrain.bin`. HA-177's Settle and guide 16's SD section say the same. | — |
| D4 | Chapter-4 anchor single dash | **Real** | `#honesty-about-limits--what-limited-strafe-does-to-your-commands` (the master-plan `17-accessibility--…` link shows the rule). | — |
| D5 | Chapter 16 missing from mkdocs nav | **Real** | Added after the Glossary (survives `api_doc_tool generate`, checked). | — |
| D6 | No reset step between floor runs | **Real** | M.6.5 LIFT back onto START, never shove a braked drive; M.8 points at it. | — |
| D7 | "No page has a button there" false for PICK | **Real** | Guide 16 and M.4: "no button on the countdown, running or RESULT page is there". | — |
| D8 | Guide 16 describes the competition program in the present tense | **Real** | A paragraph: not built yet, will call the same `registerAutons()`; "(once it is built)", "will show". The verbatim students' file untouched. | — |
| D9 | startAt "every auton's first line" contradicts the 40 in auton; README :195 | **Real** | Guide 16 (a registered auton's start is its entry's; no `startAt`), chapter 9's table row and cookbook 1 qualified for autons run by `runAuton`; README "has steered". | — |
| D10 | The briefing counts COMP-t as a 33rd chunk | **Real, not mine to decide** | Left for the coordinator: the tool counts every `-COMPLETED.md` (R1a/R1b are counted the same way); either a 64th row or a record name the tool skips. Noted in the COMPLETED record. | — |
| D11 | A test comment cites HA-166 | **Real — already fixed by the stopped pass** | Now cites the vendored-libpros disassembly of 2026-09-26, "SAFE-SAVE's belief, not yet in the register". | grep `HA-166` in test/: none |
| D12 | Guide 16 misquotes `STEP BACK 3` | **Real** | "shows STEP BACK with a count of 3, 2, 1" (no backticks: the screen prints two spaces). | — |

**Verbatim blocks the review changed (for the coordinator's brief amendment; each forced, reasons above):**
`runTesterOnce` (a `runPressCount` parameter, `watch.arm(runPressCount)`, a `std::exception` handler);
`pacerChainProven` (`&& watch.attachedTo(guard)`); the program's flow (`runOne(int, std::int32_t)`, the
idle loop keeps its `TesterTouch t`, the task's `std::exception` handler, the chain refusal's words, the
boot line → `testerBootLines()`'s three lines). `arm()`, `abortCauseOf`, `pace()`, `testerTouchFrom` /
`abortInputsFrom`, `readTouch`/`ProsAbortInputs`, `readFacts`, `stopAfterProgramError` and
`TesterRunRecord` are unchanged. `src/team/autons.{hpp,cpp}` are byte-identical to §3.1.

**Prose review (rule 4 — greps RUN and hits READ before this line):** `SD run log`, `boot line`, `end action
FAILED`, `skips the end action`, `unknown exception`, `ABORT IGNORED`, `attachGuard|attachedTo`,
`pacerChainProven`, `31/32|cases 31`, `no page has a button`, `STEP BACK`, `every auton|every registered
auton`, `safeMechanisms`, `touch baseline`, `PROGRAM=autontest` over README, `docs/` (not `docs/api`, not
other chunks' logs) and `src/team/README.md`. Fixed from the hits: HA-177's Settle (the old M.7), HA-178's
Source (the RUN-read baseline). Read and left: the COMP brief's "cases 31/32" (COMP-t's own pair, still
right), guide 07 and `src/team/README.md`'s "runs every auton" (now true), the roadmap/HANDOFF/RESUMING
build mentions (unchanged facts).

**A standing-rule slip, logged:** this pass wrote five `api_doc_tool` stderr captures to the session
scratchpad under `/tmp`; deleted at once (nothing read from them — the exit codes were read directly).

### every gate re-run on the final tree (the review's fixes in) — by the logs' own times: suite 14:24, gate `check` 14:26, `self-test` 14:35 EDT; this section written at 18:29 (`date`)

- Host suite (full build, the briefing regenerated first; the doc-gate target passed in it): **1,796 cases /
  2,649,923 assertions, 3 skipped, 0 failed** (`review-fix/suite3.log`).
- `src_build_gate.py check`: **PASS, all 7 builds link**, each "exit 0, 10/10 TUs compiled, warnings: 7
  vendor-ignored, 0 disallowed, hash asserted (v0.1.1-350-ge124b88-dirty)"; `[chain-table] 7 builds, 3 robots`
  (`review-fix/gate_check3.log`). `self-test`: **"OK (198 detector cases, each against a real build)"**
  (`gate_selftest3.log`). After it: `src/`, `include/`, `test/` hash-identical to before, and the tracked
  `src/`+Makefile diff the chunk's own (11 files, +102/−43) — no plant left.
- ARM header gate: 193 headers in one TU PASS; `run_guard.hpp`, `hal/pros/block_sink.hpp`,
  `hal/block_char_sink.hpp`, `competition/auton_registry.hpp` alone PASS. CI greps: PROS-free outside
  `hal/pros/`, sim-free core — clean.
- `api_doc_tool`: `generate` (run_guard's `///` changed), self-test OK, check-coverage 2950 entities / 157
  headers, check-fresh 0, check-examples 503 lines verbatim (6 files; 5 whole-run listings unbroken),
  check-removability clean. `doc_staleness_audit.py` self-test OK and run clean. `briefing_status.py`
  regenerated with the suite built, `check` current.
- **Nothing committed.** Left for the coordinator: D10 (the briefing's count), the brief's verbatim-block
  amendments, the ORIENTATION snapshot (unchanged from the second pass).

## Verification-round findings and their disposition (2026-09-29, 18:50–19:20 EDT by `date`; the gate logs' own times)

A fresh verifier checked the review round's fixes and returned ONE finding (minor, two parts, both in
internal records, both introduced by the review round). Re-checked before anything moved:

| # | Finding | Verdict | Fix | Proof |
|---|---|---|---|---|
| V1a | COMPLETED "Built" calls the program "§3.5's flow VERBATIM" while deviation 7 lists the review's changes to that flow | **Real** | "Built" now says §3.5's program AS AMENDED by the review (deviation 7), names the blocks still byte-identical to §3.5 (the PROS reads, `readFacts`, `ScreenRunObserver`, `stopTheTester`) apart from the changed ones (`runOne(int index, std::int32_t runPressCount)`; `testerTask` holding RUN's touch read and passing its press count, and its `std::exception` handler; `autonTesterInitialize`'s chain-refusal words and the boot line split into `testerBootLines()`'s three) | `verif-round/blocks.py` (each brief block tested as one contiguous substring of `auton_tester_program.cpp`): VERBATIM — PROS reads (brief 1441–1457), `readFacts` (1463–1480), `ScreenRunObserver` (1486–1492), `stopTheTester` with its doc comment (1521–1541); CHANGED — the whole flow (1486–1698), `runOne` (1494–1519), `testerTask` (1542–1580), `autonTesterInitialize` (1584–1698). `diff` of the brief's `runOne`/`testerTask` against the file's (`verif-round/brief_*.txt` vs `prog_*.txt`): exactly the parameter, the call's extra argument, the local `TesterTouch t`, and the added `catch (const std::exception& e)` |
| V1b | `build-order.md`'s COMP-t status says "23 findings, all real, all fixed" while D10 is deliberately left open | **Real** | Now "23 findings, all real; 22 fixed, and D10 — the briefing counting this record as a 33rd chunk — left for the coordinator". The COMPLETED record's review section says the same (it said "every one was real" and then only "the code findings were fixed", which left D10's state to the reader) | `PROJECT-BRIEFING.md:108` still reads "33 of 63 chunks complete" after regeneration; D10's row above (:469) and "For the coordinator" unchanged |

Nothing in `src/`, `include/`, `test/` or `tools/` changed in this round; no `///` comment changed, so
`docs/api` was not regenerated (check-fresh confirms it current).

**Prose review (rule 4 — greps RUN and hits READ before this line):** `verbatim` over the chunk's record,
`build-order.md`, `HANDOFF-remaining-work.md`, `RESUMING.md`, `PROJECT-BRIEFING.md`, the changelog, the roadmap,
guide 16, the COMP brief and the program's sources; `all fixed|all real|23 findings|findings, all` over
`docs/`, `src/` and README; `as the brief gave|§3.5|every finding|findings fixed|all 23` over `docs/`; every
added line of this chunk's `docs/` diff mentioning the review. Fixed from the hits: COMPLETED :28 (V1a),
COMPLETED's review paragraph and `build-order.md` :1503 (V1b). Read and left: COMPLETED :23 and
`build-order.md` :1503's "VERBATIM as the brief gave it" (both about `src/team/autons.{hpp,cpp}` —
byte-identical to §3.1, re-hashed by the second pass and unchanged since); the Next line (:632), the
roadmap's "Now" and COMP-t item, HANDOFF's L2t/:375 and the changelog's API 2.3 note (none claims the
program's text is the brief's, none claims every finding fixed); this log's first-pass (:44) and second-pass
(:340) "verbatim" lines (dated entries, true when written; the review section's "Verbatim blocks the review
changed" above supersedes them); other chunks' "23 findings" (F1/F2/G1/DOCS1 — unrelated).

### every gate re-run after the verification round's edits

- Host build: `cmake -S test -B build/test` exit 0 (the briefing regenerated first); `cmake --build
  build/test -j8` exit 0, zero warnings (the doc-gate target passed in it) (`verif-round/build.log`).
  Suite: **1,796 cases / 2,649,923 assertions, 3 skipped, 0 failed** (`verif-round/suite.log`).
- `src_build_gate.py self-test`: **"OK (198 detector cases, each against a real build)"**, exit 0
  (`verif-round/gate_selftest.log`). `check`: **PASS, all 7 builds link** — bench, xdrive, tank,
  tank-drive, tank-library, tank-calibrate, tank-autontest each "exit 0, 10/10 TUs compiled, warnings: 7
  vendor-ignored, 0 disallowed, hash asserted (v0.1.1-350-ge124b88-dirty), beacons (variant, robot) and
  program checked"; `[chain-table] 7 builds, 3 robots` (`verif-round/gate_check.log`, finished 19:16).
  After both: `src/`, `include/`, `test/` and `Makefile` sha256-identical to before the gate
  (`verif-round/tree_before.sha` = `tree_after.sha`) — no plant left.
- ARM header gate: 193 headers in one TU at the V5 flags with `-Werror` — CLEAN; `run_guard.hpp`,
  `hal/pros/block_sink.hpp`, `hal/block_char_sink.hpp`, `competition/auton_registry.hpp` alone — PASS.
  CI greps: "library is PROS-free (all of include/shulib except hal/pros/)", "layering holds: core is
  sim-free".
- `api_doc_tool`: self-test OK; check-coverage "2950 public entities across 157 headers, all documented";
  check-fresh exit 0; check-examples "503 quoted lines, all verbatim (6 source files); 5 listings of
  whole-run files, each one unbroken run"; check-removability "no public doc references docs/internal/".
  `doc_staleness_audit.py` self-test OK (10 cases) and run clean. `briefing_status.py` regenerated after the
  suite binary existed (byte-identical to before) and `check` current.
- **Re-run after this section was written (19:18):** the host build relinked `shulib_tests`, exit 0 — the
  files newer than the first build are the ones the src gate's self-test plants into and restores
  (`src/main.cpp`, `src/build_selection.hpp`, `test/CMakeLists.txt`, the tables, …: `find -newer`), their
  sha256 unchanged; the suite on that binary: 1,796 cases /
  2,649,923 assertions, 3 skipped, 0 failed (`verif-round/suite2.log`); every `api_doc_tool` check, the
  staleness audit and `briefing_status.py check` passed again, the briefing regenerated byte-identical
  (`verif-round/docgates2.log`).
- **No plant from before this round:** `run_guard.hpp` (18:45) and `auton_tester_run.hpp` (18:46) were
  touched before this round began (the verifier's run, by the times); every file the review round mutated
  (R01–R36, R07b, R14b, R33b) is byte-identical to its saved `review-fix/mut/R*.orig` — the source the
  review round's final gates ran on.
- **Nothing committed.** Still left for the coordinator: D10 (the briefing's count), the brief's
  verbatim-block amendments, the ORIENTATION snapshot.

## 2026-09-29 21:36–22:15 EDT — the coordinator's merge and verification (by `date`)

- **The transport commit and the merge.** The worktree's whole run — the build and its second pass, the
  three reviews, the fix (resumed from this log after the usage-limit pause of ~13:50), a fresh verifier
  (all 23 dispositions held; one new minor) and its fix — was committed in the worktree as
  `393fbd8` and squash-merged onto main at `a5bd83f` (TANK1 Part B, committed while this chunk was in
  review). Seven conflicts, every one in a shared document, each resolved to the merged state and read
  after: the T4 brief (both notes kept); the COMP brief (main's self-test pair 34/35 kept, with this
  chunk's B8/B18/B25/B29/B30 amendments and the re-check's RC1/ND1, ND5 and RC6/ND3 folded in);
  `build-order.md`'s Next line (one head: COMP-t landed, TANK1 complete, the speedups' builds) and its COMP
  DoD and status (this chunk's DoD, the Station H → K note, the landing date, COMP-b's re-check status);
  HANDOFF (the size line, with this chunk's L2t bullet); `docs/api` regenerated (`check-fresh` 0);
  PROJECT-BRIEFING regenerated.
- **D10 settled — a part is not a chunk.** `tools/briefing_status.py` gains `PART_NAME` and `chunk_of()`:
  a record or log named `<CHUNK>-<part>`, the part holding a lower-case letter (`COMP-t`), belongs to its
  chunk — its record is not a completed chunk, and its log keeps the chunk open. The briefing therefore
  still reads **33 of 63** (32 chunks and R2a, as before this chunk), with COMP open. `R1a`, `R2a` and
  `R5a` are chunk names with no hyphen and never match. The rule has its own `self-test` (17 cases: names
  both ways, and a scratch records directory where a part's record is left out, a part's log keeps its
  parent open and the parent's own record closes it), run by the doc gates in `test/CMakeLists.txt` and by
  CI; four mutations of the rule (the part pattern narrowed to a letter first; part records counted;
  part logs listed under their own names; `chunk_of` returning the name) each turned it red.
- **Documents the merge owed** (this chunk's "For the coordinator"): the COMP-t brief carries a header note
  naming the three VERBATIM blocks the review changed (deviation 7) and that §3.1 is unchanged;
  `ORIENTATION.md` brought up to date (seven programs, the tester, what has run on robot two since
  2026-09-24 with the logs that say so, the next steps; its counts dated 2026-09-29); `REMAINING-WORK.md`
  (COMP-t landed; Station M ready; the totals less COMP-t's day and TANK1 B's half).
- **Gates on the merged tree** (`.claude/coordinator/logs/*.comptmain.log`): host build exit 0 and the suite
  **1,808 | 1,808 passed**, 2,652,625 assertions, 3 skipped; `src_build_gate.py` self-test OK (198 detector
  cases, each against a real build) and check **PASS** (all 7 builds link, 0 disallowed warnings; step 0:
  `[chain-table] 7 builds, 3 robots`, `[program-table] 6 programs, 14 entry names`, `[include-root] 31
  source files`); the ARM header gate over all 193 headers exit 0; both CI greps empty; `api_doc_tool check-fresh` exit 0; `briefing_status.py check` current;
  removability empty; the staleness audit clean. No plant: `git diff -- src/ Makefile include/ test/`
  empty after the gates and after the mutations.
- **Ten coordinator mutations, each on a fresh binary** (`.claude/coordinator/muts_compt.json`; the
  project's recipe: edit, touch every test, fast link, whole suite, restore, hash-check):

  | # | the mutation | caught by (first failing checks) |
  |---|---|---|
  | CT1 | `buttonsDownOf` reads no button | 11 cases red — `auton_abort_watch_test.cpp` :234-235, :250 |
  | CT1b | button A (the last enumerator) never aborts | `auton_abort_watch_test.cpp` :250, :253 (`0x0FFF`) |
  | CT2a | the chain proof ignores which guard the watch is attached to | `auton_abort_watch_test.cpp` :526, :528; `auton_tester_run_test.cpp` :504 |
  | CT2b | the chain proof passes with no pace through the chain (`>` → `>=`) | `auton_abort_watch_test.cpp` :504, :506 |
  | CT3 | an existing log truncated with `"wb"` when the append open fails | `pros_block_sink_adapter_test.cpp` :257-259 |
  | CT4 | RUN allowed without the calibration record (D9 dropped) | `auton_tester_panel_test.cpp` :303-305 |
  | CT5 | a controller drop does not abort | `auton_abort_watch_test.cpp` :98, :114, :130 |
  | CT6 | RUN allowed on a field connection outside autonomous | `auton_tester_panel_test.cpp` :310-311, :330 |
  | CT7 | a throwing mechanism skips the mechanisms after it (`RunGuard::safeMechanisms`) | `sequence_run_guard_test.cpp` :241, :274 |
  | CT8 | a program error leaves the mechanisms unsafed (`stopAfterProgramError`) | `auton_tester_run_test.cpp` :613-615 |

  Every one linked (one link line each), every file restored byte-exact (hash-checked), and the restored
  tree relinked and passed (1,808 | 1,808). 21:52:15–21:57:37.
- **After the self-test was added** (`tools/briefing_status.py`, `test/CMakeLists.txt`'s doc gates, `ci.yml`,
  HANDOFF §3's command set): the host build exit 0 with the doc gates printing "briefing status self-test: OK
  (17 cases …)"; the suite 1,808 | 1,808 again; the briefing regenerated and current; `src_build_gate.py check`
  PASS on all seven builds (the gate reads `test/CMakeLists.txt`); `git diff -- src/ Makefile include/` empty
  (`.claude/coordinator/logs/comptmain2.log`, 22:11).
- **Committed by the coordinator** with every document above in the same commit. The worktree `comp-t` is
  removed and its branch kept (`393fbd8` is cited here). Nothing of it has run on a robot: worksheet Station M
  is its first run.
