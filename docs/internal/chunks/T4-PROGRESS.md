# T4 — progress log (live)

- **Chunk:** T4 — the driver program (brief: [`T4-driver-program.md`](T4-driver-program.md))
- **Executor model:** opus
- Appended as the work goes, with a heading per part. Every time in a section header is a `date`
  reading taken when the section was written, not an estimate written afterwards. **Nothing in this
  chunk has run on a robot.**

---

## Part 1a — the driver file

- **Started:** Sun Sep 27 05:15:29 AM EDT 2026 (`date`).
- **Tree at start:** `9e86a3c` (Part 0 landed there), clean, in the git worktree
  `.claude/worktrees/t4-1a` (branch `t4-1a`) — NOT the main working tree.
- **Go:** the coordinator's workflow run, 2026-09-27, on the team lead's go for T4 Part 1a. Do not
  commit; no `pros` command; no `/dev/ttyACM*`. Mutation backups under
  `.claude/coordinator/scratch/t4-1a/` (never `/tmp`).
- **Scope:** Part 1a ONLY (§1 "Part 1a — the driver file"). Part 0 is already in `9e86a3c`. Parts
  1b, 2 and 3 are not touched, nor anything COMP-t or TANK1 owns (`src/team/autons.*`, the auton
  tester, `include/shulib/motion/*`). No new build, no new HA entry.

### 05:15 — start
- Read `docs/internal/RESUMING.md` (the chunk loop, the standards, pace rules 1-8) and the brief in
  full (the header's rulings and answers, § Decisions, §0-§8). This log created before any code.

### 05:16–05:21 — required reading, and the baseline *(`date` 05:21:30 at this entry)*
- Read, in the tree at `9e86a3c`: `src/programs/teleop_claws.hpp` (whole), `src/programs/teleop_program.cpp`
  (:1-230, :540-620, :720-1000), `src/programs/xdrive_program.cpp` (:1-76, :225-351), `src/robots/mechanism_table.hpp`
  (whole), `src/common/boot_source.hpp` (:1-240), `include/shulib/teleop/mechanism_binding.hpp` (the rules and
  the declarations), `include/shulib/teleop/teleop_loop.hpp` (:85-337), `test/teleop_claws_test.cpp` (whole — the
  fixture Part 1a's test reuses), `test/CMakeLists.txt`, `tools/api_doc_tool.py` (the header, `do_check_examples`,
  the self-test's frame), `tools/src_build_gate.py`'s header (step 0's per-robot-name rule: a robot's table may be
  named only in the table files, in `test/`, and inside a `static_assert`), `docs/cookbook/06-driver-control.md`,
  COMP's B10/B11/B28/B29, the build order's T4 entry and first `**Next:**`, the roadmap's T4 row, HANDOFF's
  L5/L5b/§7, G1-PROGRESS's owed heading-helper item (:1729-1731, :2007-2009) and G1-COMPLETED's follow-ups (:83-86).
- Read the coordinator's §0.4 probe v2 (`.claude/coordinator/scratch/brief-T4/v2/driver_controls.hpp`,
  `host_probe_v2.cpp`) — read-only, a SKETCH: its shared refusal format under a pragma is not acceptable
  (landmine 13), and the real file is written from it, not copied.
- **Anchors re-checked against the tree (SRC1 changed no line count):** `headingSample()` teleop_program.cpp:559
  and xdrive_program.cpp:237-248 (the "Written twice … the merge as owed" comment :237-241); `clawRowText`
  teleop_claws.hpp:385; the three T3 comments teleop_claws.hpp:21-25, mechanism_table.hpp:493-497,
  boot_source.hpp:91-94 (the brief says :92-94; the sentence starts on :91); `EXAMPLE_SOURCE_GLOB`
  api_doc_tool.py:1811, used :1868, reasoned :47; `SHULIB_DOC_INPUTS` test/CMakeLists.txt:172-178.
- **Baseline, host** (`build/test`, a fresh configure at `9e86a3c` + this log; the briefing regenerated first,
  because this log makes T4 an open chunk and the build runs the briefing gate):
  `[doctest] test cases: 1659 | 1659 passed | 0 failed | 3 skipped`,
  `assertions: 2467051 | 2467051 passed | 0 failed` — the same as the committed briefing's.

### 05:21–05:59 — the code and the tests *(`date` 05:59:17 at this entry)*
- **`src/common/driver_controls.hpp` (NEW, PROS-free).** Written from the §0.4 sketch, not copied:
  - `HoldControl` / `ToggleControl`; five constexpr predicates, one property each. `controlButtonsDistinct`
    compares use by use (not by enumerating the twelve buttons), so it holds for any button byte;
    `controlsNameTable` also refuses one name taken by a hold AND a toggle.
  - `DriverBindings::bind()` in FOUR passes: (1) resolve every control to a DECLARED mechanism of its kind
    (nothing built yet, so a refusal constructs nothing), (2) coverage, (3) build a binding per BUILT mechanism
    (an absent one skipped), the toggle's initial level `config::toLineValue(mechanisms[i].pneumaticStart)`,
    (4) `teleop::validBindings` last. Capacity `config::kMaxMechanisms` (8) per kind, checked first. Every
    refusal its OWN literal format with a precision on every `%s` (landmine 13; the sketch's shared format under
    a pragma is gone). Two refusals the sketch did not have, each host-pinned: a null control name, and more
    controls than a robot can declare (the sketch's sentence had no numbers).
  - `holdFor(name)` / `toggleFor(name)` match the BOUND mechanism's own `name()`; `anyReleasedLeftoverClaim()`.
  - `driverHeading()` — Teleop's `headingSample()` logic, byte-identical expression.
- **`src/team/robot_two_driver.hpp` (NEW)** — §2, with exactly three differences (diffed against the brief's
  §2 text, lines 1388-1529, saved as `.claude/coordinator/scratch/t4-1a/section2_verbatim.hpp`):
  1. the caller sentence in its "At 1a" form (§1 Part 1a item 2, verbatim);
  2. **"change it HERE and nowhere else" is FALSE at 1a**, because item 3's interim tie makes a button change
     need the table too (and the re-zero, Teleop's `kRezeroButton`). Reworded to what is true until 1b: "To
     change what a button does, change it HERE -- and, until T4 moves the buttons out of
     src/robots/mechanism_table.hpp, there too (the re-zero also in "shulib Teleop"): `make` refuses a build in
     which they disagree." — 1b restores §2's "and nowhere else" when the table's copy goes;
  3. the protections paragraph is Part 2's, so it is not in the 1a file; and `run()`'s doc drops "one that takes
     a fourth argument is also handed the drive protections' state" (§2's own note: "`run()`'s doc names the
     optional fourth argument (Part 2)").
  **No code line differs from §2.**
- **The interim tie:** `controlsMatchTable(holds, toggles, table)` in `src/programs/teleop_claws.hpp`; the two
  `static_assert`s in `src/programs/teleop_program.cpp` right under the re-zero asserts. The same two in
  `test/robot_two_driver_test.cpp` — the second written against `kTeleopRezeroButton`, because Teleop's
  `kRezeroButton` is in that TU's anonymous namespace (it is static_assert-tied to `kTeleopRezeroButton` there,
  so the host form checks the same fact). Both name `kRobotTwoMechanisms` only inside a `static_assert`
  (landmine 10).
- **`clawRowText(t, robot, bool freed)`** is the shared form; the `TeleopClaws&` form is a one-line forwarder
  (`return clawRowText(t, robot, claws.anyReleasedLeftoverClaim());`) — no call site, no 1l pin changed.
- **ONE heading rule:** `headingSample()` DELETED from `teleop_program.cpp` and `xdrive_program.cpp`; both call
  `driverHeading(r)`. The X-drive's "Written twice … the merge as owed" comment replaced by a note that the merge
  was made; its now-unused `localization/localizer.hpp` include removed. Teleop's `kFieldCentric` comment said
  "the heading source below" — re-pointed to `driverHeading()`.
- **The three T3 comments** reworded to the rule (the first entry in a run SENDS the start level; nothing moves
  unless something commanded the line in between; after an autonomous the ruling closes it), each saying what it
  said until 1a: `teleop_claws.hpp` :21-25, `mechanism_table.hpp` :493-497, `boot_source.hpp` :91-94.
  `mechanism_table.hpp`'s WHO READS IT gains the driver file, and `kTeleopRezeroButton`'s doc the second tie.
- **Tests.** `test/driver_controls_test.cpp` (PROS-free, 11 cases) and `test/robot_two_driver_test.cpp` (through
  the shim, 14 cases). Two things found while making them pass, both TEST-side:
  - a fresh `FakeBattery` reads 0 V, and the drive then commands 0 V under a BROWNOUT fault (probe
    `.claude/coordinator/scratch/t4-1a/probe/fake_drive_probe.cpp`) — the fake rigs now set 12.8 V;
  - REASONED (from `teleop_program.cpp`'s own comment on the latch), to be checked by the `ArcadeField`
    mutation below: T5 might not see field-centric ON without a re-zero — field-centric LATCHES robot-centric
    when its first sample is untrusted (tick 0 is sampled before the estimate is live), and only a re-zero
    clears it. So the script presses Y at ticks 5-6 (in ArcadeBody a press only pre-arms the zero; the direct
    run presses it too).
- **Suite:** `test cases: 1684 | 1684 passed | 0 failed | 3 skipped`, `assertions: 2469298 | 2469298 passed`
  (+25 cases, +2,247 assertions against the baseline).

### 06:00–06:14 — the quoting gate, the documents, and the first gate pass *(`date` 06:13:08 read mid-section)*
- **`tools/api_doc_tool.py`**: `EXAMPLE_SOURCE_GLOB` (one string) became `EXAMPLE_SOURCE_GLOBS`, a LIST, one
  glob per KIND of file a public document may quote whole: `test/*example*_test.cpp` and `src/team/*_driver.hpp`
  (T4 lands first, so T4 creates the list; COMP-t appends `src/team/autons.cpp`). Each glob must match at least
  one file, or the scan FAILS naming it. The reasoning is at the list (and the header's DOCS2 paragraph points
  at it); it says it is never widened to all of `src/`, and neither entry is called "the one/only exception".
  A NEW self-test case builds a fixture tree and runs the real check: a line only a driver file holds passes; a
  drifted driver quote fails; a line only a program file holds fails; a kind whose glob matches nothing fails.
- **`test/CMakeLists.txt`**: `SHULIB_DOC_INPUTS` gains `../src/team/*_driver.hpp`, so an edit to the driver
  file re-runs the verbatim scan; the check-examples comment names the new kind.
- **Documents** (each prose-reviewed below):
  - `docs/cookbook/06-driver-control.md` — the closing section "A whole driver file: robot two's": the
    controls, the feel, `bind()` and `run()` quoted VERBATIM (every line checked by `check-examples`), what each
    part is for, that the binder is `src/common/driver_controls.hpp`, that no program runs it yet and nothing of
    it has run on a robot; the intro now says the last section quotes a project file.
  - `docs/cookbook/README.md` — the index row, and the two sentences that said every listing is in
    `test/cookbook_examples_test.cpp` (false for the new section).
  - `docs/guide/README.md` — the verbatim-scan paragraph: "a compiled example test, or one of the team's files
    under `src/team/` that a chapter quotes (today: robot two's driver file …) — never the rest of `src/`".
  - `docs/guide/07-getting-set-up.md` :123-126 — said `src/team/` is "empty until the driver program and the
    autonomous list land": FALSE now; says its first file is the driver file, run by no program yet.
  - `test/README.md` :24-25 — the build's gate list said "verbatim in a compiled example test" only.
  - `docs/internal/chunks/COMP-competition-program.md` — B11 REPLACED by §4 Part 1a's text, verbatim, plus a
    header saying what it replaced and an "as landed" note naming the landed API for COMP-b's builder (the brief:
    "every document quotes the result, never the proposal").
  - `docs/internal/build-order.md` — T4's status line and Part 1a's bullet; the first `**Next:**` (T4 Part 1a
    landed; COMP-b next on it) with a landed paragraph.
  - `docs/roadmap.md` — the "Now" paragraph and T4's row, `[ ]` → `[~]` with the evidence named.
  - `docs/internal/HANDOFF-remaining-work.md` — L5 (the buttons stated twice until 1b; the re-zero tie), L5b
    (Part 1a landed), §7's order.
  - `docs/internal/chunks/G1-COMPLETED.md` — its follow-up "the heading-trust clause has no host test" now says
    the owed shared helper was made (T7), the fault-latch attach still has none.
  - `src/team/README.md` — "It is empty until then" was false; names the driver file.
  - *(Added by the review round, 07:46: `README.md` :296-301 was edited too — the `team/` line said
    "empty until …", and `common/` gained the binder — and this list left it out. The count below is
    therefore TWELVE hand-edited documents, not eleven; see "Part 1a review findings", finding 7.)*
  - **Pace rule 7:** that is eleven documents, not five. The brief's own list is five (cookbook 06, COMP B11,
    build-order, roadmap, HANDOFF); item 6's guide README sentence is the brief's too; the other five are
    sentences the prose review found FALSE after this part (cookbook README ×2, guide 07, test/README,
    src/team/README) and G1-COMPLETED's follow-up the brief names ("say so in G1-COMPLETED's follow-ups").
    Flagged for the coordinator, not split.
- **Deliberately NOT touched** (each is a later part's, and nothing in it is false at 1a): guide 10 :177-214
  (Part 1b: "shulib Teleop" still drives through the loop on the table's buttons — true), guide 14 (Part 1b/2),
  HA-153's source text (Part 1b: Teleop still binds R1/R2/L1/L2 through `teleop_claws.hpp` — true), worksheet
  Station J (Part 1b), `teleop_loop.hpp`'s banner (Part 1b; no library header changed, so `docs/api` is
  unchanged — `check-fresh` passes), the parked T3-COMPLETED draft (1b, the coordinator's), T3's build-order line
  on where macros will live (§8 names no part for it; left for the coordinator), no HA entry (HA-175/176 are
  Part 2's), no changelog entry (no library declaration changed).
- **Gates, first pass (06:05-06:14):** host suite 1684/1684; `src_build_gate.py self-test` OK (189 detector
  cases) and `check` PASS (six builds, 8/8 TUs each, 0 disallowed, `[one-chain] … 7 per-robot names named only
  in the tables, test/ and static_asserts`); ARM header gate exit 0 over 191 headers; the two CI greps; api_doc
  self-test / check-coverage / check-fresh / check-examples (460 quoted lines, 5 source files) /
  check-removability; briefing check; `doc_staleness_audit.py` self-test and audit clean.

### 06:15–06:38 — mutations *(`date` stamps in `.claude/coordinator/scratch/t4-1a/mutation-results.txt`)*
- Runner: `.claude/coordinator/scratch/t4-1a/mutate.py` (backups under `…/scratch/t4-1a/backup/`). Each
  mutation: exact one-occurrence edits, `check-examples` run on its own, the suite built through CMake's
  doc-gate-free `shulib_tests/fast` target (the binary's mtime compared before/after: every runtime row below
  says `fresh binary=True`), the named cases run, every file restored byte-for-byte AND re-stamped.
- **Runs 1-3 are VOID as evidence, and why** (kept in the results file, marked): (1) the restore used
  `shutil.copy2`, which put back the backup's OLD mtime, so `make` kept objects compiled from mutated sources —
  the restored tree then failed ten unrelated suites until a `--clean-first` rebuild (06:25, 1684/1684 green);
  (2) runs 1-2 went through the full build, where the doc gate stopped every edit of a line the cookbook quotes
  before the compile (itself a finding: the verbatim scan now guards the driver file); (3) run 3 went through
  the full build too, whose briefing gate reads the suite's own pass/fail from the binary, so it failed each
  build for the previous mutation's red. Fixed in the runner (re-stamp on restore; `check-examples` alone, then
  `shulib_tests/fast`); **run 4 (06:36-06:37) is the evidence**, every mutation from scratch.
- **Run 4, every row observed** (brief §3's table, then the additions):

  | Mutation | Seen |
  |---|---|
  | swap R1/R2 in `kHolds[0]`, driver file only | compile: `the driver file and the mechanism table disagree about a button -- change both until T4 Part 1b` (the tie); and `check-examples` red on the quoted line |
  | the same swap in BOTH files (the tie satisfied) | T1 red: `with[k].mv[6] == 12000` (13 failures) |
  | swap the two holds' names, driver only / in both (the table's buttons swapped to match) | the tie / T1 red (50 failures) |
  | toggle initial level `!toLineValue(start)` | T2 red (`claw->initialState() == start`, `lineA()[1] == s`) |
  | initial level = `p->commanded()` | T2b red on the bind-after-set case only (`initialState() == start`, `lineA().back() == s`) |
  | a hard-coded `false` initial level | T3 red (`lineA()[1] == closed`) |
  | initial level from `pneumaticSafe` | T2c red (`lineA()[1] == open`) |
  | `holdFor()` returns the first hold | T1 red (`seven->forwardButton() == L1`, port 7's -4000 mV) |
  | `toggleFor()` returns null | T2 red (`REQUIRE(claw != nullptr)`) |
  | `anyReleasedLeftoverClaim()` returns false | T9 red (the flag and the FREED row) |
  | no null check before the hold `emplace` | T4: SIGSEGV |
  | no null check before the toggle `emplace` | T4b: SIGSEGV |
  | an undeclared name treated as absent | T10 red (`CHECK_FALSE(d.bind(…))`, the sentence) |
  | the kind check dropped | T10 red: the sentence differs (the hold falls to `mechanismAt()` == null and is skipped; coverage then refuses with ITS sentence) |
  | the coverage check dropped | T10 red on "intake" |
  | `kFeel.mode = ArcadeField` | T5 red (328 of 514 assertions: `viaDriver[k].mv[p] == direct[k].mv[p]`) |
  | … AND T5's re-zero press removed | **T5 PASSES** — the latch hypothesis (05:59) confirmed: without the press, field-centric latches robot-centric from tick 0 and the mutation hides; the press is load-bearing |
  | `releaseBrakeMode = Brake` | T6 red (432 failures: mV, `stopped`, `coast`) |
  | drop each `driverHeading()` term | T7 (a), (b), (c) red, one each (driver_controls_test :419/:432/:445) |
  | drop `unbind()` at the top of `bind()` | T8 red (the second `bind()` refuses: its own leftovers make every button used twice) |
  | `run()` handing `{}` for the toggles | T2 red (`log[0].lineWrites == 2U`) |
  | drop `validBindings()` from `DriverBindings::bind()` | driver_controls_test's run-time clash red (11 failures) |
  | a word reading the wrong entry: the shared claw row reads `mechanismAt(0)` for every pivot | the row pins red — 5 cases, listed by a re-run at 06:50: T4 1a T4 (`CLAW 6 FWD 20C \| 7 ABSENT …`) and T3's 1e, 1g, 1h and 1n. The brief's table says "the J.3.1 pin"; J.3.1 quotes the BOOT lines, which Part 1a does not touch — the shared words 1a changes are the claw ROW (J.3.3's), so the mutation was run there |
  | the shared row ignores `freed` | 1h and the new shared-row case red |
  | the heading rule copied back into `xdrive_program.cpp` | the source pin red (`ruleCopies == 1U`, `ruleFile`) |
  | the tie compares holds only / forward only | the tie case red (1 / 2 failures) |
  | `controlButtonsDistinct` skipping adjacent uses (forward vs its own back) | red (`forwardIsBack`, `onL2`) |
  | `holdVoltsValid` as `v <= 0 \|\| v > 12` (NaN passes) | red on NaN |
  | `controlsNameTable`'s hold KIND rule dropped | red — **after a test fix**: in runs 1-2 it SURVIVED, because every wrong-kind case also named a mechanism another control named, so the "twice" rule refused it anyway. Added two cases where only the kind rule can refuse (a hold on "wing", a toggle on "intake") |
  | the toggle KIND rule dropped (new) | red on the toggle-on-"intake" case |

  **Compile mutations** (each the named message seen; each also turns `check-examples` red, because the
  cookbook quotes the edited line):

  | Mutation | Messages |
  |---|---|
  | `rezeroButton = Button::A` | `a mechanism's button is the re-zero button` + the re-zero tie |
  | `"claw pivot 66"` | the naming message + `one of robot two's mechanisms has no control` + the tie |
  | `13_volt` | `a pivot's volts are not in (0, 12]` + the tie |
  | port 7's line removed | `one of robot two's mechanisms has no control` — **the ONLY one** |
  | a hold naming `"claw"` | the naming message + coverage + the tie |
  | R1 twice | `a button does two jobs (or a pivot's forward is its back)` + the tie |
  | `.tickPeriod = 20_ms` | `one tick is not the brain's` (only) |
  | `kHolds[0].forward = Button::X`, table unchanged | the interim tie's message ONLY (host) |

  **On the ARM build, fresh `bin/`/`.d/`, `make ROBOT=tank PROGRAM=library` (06:22):** the same `Button::X`
  edit → `src/programs/teleop_program.cpp:186: static assertion failed: the driver file and the mechanism table
  disagree about a button -- change both until T4 Part 1b`; `rezeroButton = Button::X` (no control uses X, so the
  driver file's own checks pass) → `teleop_program.cpp:190: … the driver file's re-zero button and shulib
  Teleop's kRezeroButton disagree …`. Restored byte-exact, `bin/`/`.d/` removed.

  **The quoting gate** (06:23, `tools/api_doc_tool.py` backed up and restored): the driver glob dropped →
  self-test FAILS (two cases) and `check-examples` on the real tree FAILS (the cookbook's block 5); the list
  widened with `src/*/*.cpp` → self-test FAILS ("never all of src/"); a glob matching nothing tolerated →
  self-test FAILS; the cookbook's quoted `10_ms` edited to `20_ms` → `check-examples` FAILS naming block 6.
- **Stated EQUIVALENT, not counted (brief §3):** a per-run `DriveShaper` and a constant heading lambda —
  unobservable while `kFeel` is ArcadeBody with no slew (the heading is ignored and the zero offset is not in
  `DriveShaperState`); a field-centric test through the driver file is owed the day field-centric becomes
  selectable. `validBindings()` is unreachable for the REAL set (the compile-time checks refuse all it would);
  it is reached and pinned through `DriverBindings` directly (the M20 row).
- After run 4: every file restored and re-stamped; `shulib_tests/fast` rebuilt; the suite 1684/1684,
  2,469,300 assertions (+2 from the two kind-rule checks).

### 06:39–06:55 — the prose review (logged BEFORE any claim of it), the final gates, and what is left
- **The prose review — greps RUN and hits READ** (outputs saved: `.claude/coordinator/scratch/t4-1a/prose_review_1.txt`,
  `_2.txt`, `_3.txt`), over `docs/`, `README.md`, `test/README.md`, and `src/` / `include/` / `test/` for the
  code terms *(CORRECTED by the review round: the `src/team` grep in `_2.txt` was NOT run over
  `docs/internal/` — it has no `docs/internal/` hit at all — and three live "empty until …" sentences
  survived there and in the roadmap; see "Part 1a review findings", finding 1, and `prose_review_4.txt`)*:
  - the brief's three terms: `headingSample` (hits only in internal history and the new source pin — no live
    copy); "cannot move when driver control" and its variants ("not move when driver", "moves when driver
    control", "claw does not move") — only the three reworded comments quoting their OLD words, the brief, and
    cookbook 06 :194 ("Give the toggle the same starting level, or the claw moves when driver control first
    starts" — advice for a program building its own toggle, still TRUE, kept); "which button" — every hit true.
  - the changed files' concepts: `check-examples` / "verbatim in a compiled" / `*example*` (→ fixed
    `test/README.md` :24 and the guide README; the cookbook chapters' "compiled and run in
    test/cookbook_examples_test.cpp" lines are about their own recipes and stay true); `src/team` (→ fixed guide
    07 :123-126, and, found by the third grep, `README.md` :296-297 "empty until the driver program … land" —
    FALSE, fixed, with `common/`'s list gaining the binder); `clawRowText` / `teleop_claws` / `TeleopClaws` (guide
    07 :136 and HA-153's source text still true at 1a — Teleop still binds through `teleop_claws.hpp`; both are
    Part 1b's); heading trust (guide 10 :185, true); `EXAMPLE_SOURCE` / `api_doc_tool` (guide README :148, roadmap
    :523 and :1856 — about the generator, true); `kTeleopRezeroButton` / `kRezeroButton` /
    `pneumaticStartCommand` (no public hit).
- **Final gates (06:38-06:55; the src gate 06:38-06:49, the rest 06:50-06:55), on the tree as left:**
  - host: `[doctest] test cases: 1684 | 1684 passed | 0 failed | 3 skipped`, `assertions: 2469300 | 2469300
    passed | 0 failed` (baseline 1659 / 2,467,051: +25 cases, +2,249 assertions);
  - `src_build_gate.py self-test`: `OK (189 detector cases, each against a real build)` (finished 06:48:41; `check` 06:49:42 — after every edit
    under src/ and test/); `check`: `src build gate: PASS (all 6 builds link; …)`, each build 8/8 TUs, 0
    disallowed, `[one-chain] … 7 per-robot names named only in the tables, test/ and static_asserts`,
    `[include-root] 24 source files in src/ and 4 folders`, `[printed-path]` clean;
  - ARM header gate: exit 0 over 191 headers; CI greps: PROS-free, sim-free;
  - `api_doc_tool.py` self-test OK, check-coverage (2908 entities, 155 headers), check-fresh exit 0 (no library
    header changed, so `docs/api` needed no regeneration), check-examples (460 quoted lines, 5 source files),
    check-removability clean; `briefing_status.py check` current (regenerated before the build and after the
    binary: `Suite: 1,684 cases / 2,469,300 assertions … green`, open chunks now include T4, 13 `[~]`);
    `doc_staleness_audit.py` self-test OK and clean.
- **Nothing committed; no `pros` command; no `/dev/ttyACM*`. `bin/` and `.d/` removed after the ARM mutations
  (the gate removes and rebuilds them itself).**

### For the coordinator (Part 1a)
1. **Driver file vs §2:** three differences, all header prose, no code line (above, 05:59). The second is
   forced by the tree: §2's "change it HERE and nowhere else" is false while the interim tie stands; 1b puts it
   back.
2. **B11 in COMP's brief** is the §4 text verbatim, plus a provenance header and an "as landed" note (the landed
   API names and T2b's case name). Take or trim them at review.
3. **The verbatim scan now guards the driver file:** an edit to (or the removal of) a line the cookbook quotes
   fails the build's doc gate until the cookbook is re-quoted (seen on every compile mutation). *(Corrected by
   the review round: as first landed, a line ADDED to the file passed — membership only. The whole-run rule,
   `WHOLE_RUN_SOURCE_GLOBS`, now fails that too; see "Part 1a review findings", finding 5.)* That is the design (item 6), but it
   is also a new step for whoever edits a button — COMP-b, 1b and G3a's generator (its marked region is exactly
   the quoted WHICH BUTTON block; G3a's DoD "the driver file's region, byte for byte" should expect to re-quote
   the cookbook, or the cookbook should quote something G3a does not rewrite).
4. **COMP-t lands second on the shared list:** it appends `src/team/autons.cpp` to `EXAMPLE_SOURCE_GLOBS`, adds
   it to `SHULIB_DOC_INPUTS`, and extends the guide README's "(today: robot two's driver file …)". `src/team/README.md`,
   guide 07 :123-126 and README :296-297 now name the driver file as the first file, and "the autonomous list"
   as next — COMP-t re-reads all three. *(Review round: so do the roadmap's SRC1 paragraph, HANDOFF's `src/`
   paragraph and build-order's first `**Next:**` — six in all. And the tool now has a second list,
   `WHOLE_RUN_SOURCE_GLOBS`; whether `src/team/autons.cpp` joins it is COMP-t's call — finding 5 below.)*
5. **The mutation runner's two traps** (runs 1-3 void): restoring with `copy2` leaves stale objects, and the
   full build's briefing gate fails for a mutation's red. Any later executor's runner should re-stamp restored
   files and build `shulib_tests/fast` (or run the suite on a clean rebuild).
6. **Pace rule 7:** eleven documents (above) — *corrected by the review round to twelve (README.md was
   missing from the list), and fourteen after it (the brief's 1b instructions and `guide-maintenance.md`)*;
   none of the extra ones could wait without leaving a false sentence.
7. **Found, not T4 1a's:** `teleop_claws.hpp`'s refusal sentence (`TeleopClaws::bind`) and T3's test 1i remain
   until 1b retires them; `docs/guide/07` :133-136 lists `src/common/` without the new binder (not false —
   the list was never exhaustive — Part 1b's guide-07 edit can add it).

---

## Part 1a review findings and their disposition

- **Round:** the coordinator's workflow run handed back the review's seven findings, 2026-09-27; the first
  `date` reading of the round was 07:26:54 AM EDT. Same worktree, same rules: nothing committed, no `pros`, no
  `/dev/ttyACM*`. Everything this round wrote outside the tree is under `.claude/coordinator/scratch/t4-1a/`:
  `mutate_review.py` (the runner: exact one-occurrence edits, backups in `backup-review/`, `shulib_tests/fast`
  with the binary's mtime compared, restore byte-compared AND re-stamped), `mutation-results-review.txt`,
  `prose_review_4.txt`, `review-round-*.log`.
- **No `src/` or `include/` file changed this round.** The code the review looked at was right in every case;
  what was missing was tests that could see it, a gate rule, and true sentences.

### Finding 1 (major) — three live "empty until …" sentences; the logged review claimed more than it ran
- **Re-checked: REAL.** `grep -rn "empty until"` over ALL of `docs/` found the three the review named:
  `docs/roadmap.md` :380, `HANDOFF-remaining-work.md` :283, `build-order.md` :632 (inside the first `**Next:**`,
  a few sentences after its own "T4 PART 1a LANDED" — a paragraph contradicting itself). And
  `prose_review_2.txt` has NO `docs/internal/` line at all: the `src/team` term was not grepped over
  `docs/internal/`, though this log said "over `docs/`". It also printed roadmap :379, the line above the
  stale one. **Six sentences said "empty until" when Part 1a started, not three:** README.md, guide 07 and
  `src/team/README.md` (fixed in the first pass) and these three. The first pass's report said three.
- **Fixed:** the roadmap's SRC1 paragraph (:379-381: "its first file is robot two's driver file (T4 Part 1a),
  and the auton list comes with the auton tester"); HANDOFF :282-286 (the same, naming the file, and
  `src/common/`'s list gains "a driver file's binder and checks", as README.md's does); build-order :632 (the
  SRC1 clause: "SRC1 left it empty; its first file is T4 Part 1a's driver file, and COMP-t's auton list comes
  next"). The claim at 06:39 above is corrected in place, with a pointer here.
- **The grep, RE-RUN and read** (`prose_review_4.txt`, 07:47, over all of `docs/` with `docs/internal/`,
  `README.md`, `test/README.md`, `src/`, `include/` and `tools/`):
  - "empty until": no live sentence about `src/team/` left. The other hits are this log's own history lines,
    R3d-PROGRESS :2247 and E3-PROGRESS :145 (history, other subjects), and three library comments about
    other things (`hal/pros/motor.hpp` :365, `localization/tag_map.hpp` :24, `complementary_fusion.hpp` :286).
  - `src/team`: 91 hits, every one read. The live ones are true; SRC1's brief (:24 "empty today") and log are
    history written before T4; COMP-t's brief is a plan for its own part.
  - The concept's other spellings ("where students edit", "the team's own code", "team folder", "team/ holds
    README.md only") and `src/common/`'s contents lists (README.md :300-301 and HANDOFF :285-286 name the binder; the
    roadmap :382 and guide 07 :127 list no contents): true.
  - Found by the same re-run, not named by the review, both false since Part 1a: build-order :632 said the
    cookbook "quotes the file whole" (it quotes four parts) — reworded; `docs/internal/guide-maintenance.md`
    :31-35 said `check-examples` wants every line "in one of those test files" and that the source set "is a
    glob" — reworded (finding 5).

### Finding 2 — "a refusal leaves NOTHING bound" was unpinned after a successful bind (RV-M1)
- **Re-checked: REAL.** RV-M1 (`unbind()` moved from the top of `bind()` to just before the build step,
  after every refusal), fresh binary, 07:30:59: 25/25 cases, 2,249/2,249 assertions — it SURVIVED.
- **Fixed in the tests.** `driver_controls_test`'s "refuses, by name" case binds robot two cleanly before EVERY
  refusal (`refusedAfterABind`) and checks `holds()`, `toggles()`, `holdFor()` and `toggleFor()` are empty
  after each; the null-`why` refusal too. T10 binds the driver file to robot two over the FAKE provider
  (`FakeRig`, which keeps no shim state, so it stays alive while the refusing robot is built on the shim)
  before each of its refusals.
- **After:** RV-M1 turns the "refuses, by name" case and T10 red (07:42:23, 2 cases, 50 assertions), and T10
  run alone red too (07:43:08, 8 failures at :751/:752/:769/:770/:785/:786 …). Fresh binary each time.

### Finding 3 — a toggle naming an undeclared mechanism reached no test (RV-M7)
- **Re-checked: REAL.** RV-M7 (the toggle branch's refusal replaced by `toggleAt[t] = i; continue;`)
  SURVIVED, 07:31:01, 25/25.
- **Fixed, two ways.** `driver_controls_test`: toggles `{"claw", A}, {"wing", B}` on robot two → false,
  "a control names 'wing', which the robot does not declare", nothing bound. T10, through the driver file and
  the shim: a robot with no claw (the table's `pneumaticCount` 0) → "a control names 'claw', which the robot
  does not declare", nothing bound. That is the form a real robot would hit. The run-time case also gained
  the two it lacked: an undeclared HOLD name ("claw pivot 66") and a hold on the claw (the wrong kind), and
  both coverage refusals.
- **After:** RV-M7 red in both files (07:42:25, `driver_controls_test` :418/:419; 07:43:11, T10 :795-797).

### Finding 4 — RV-M2 (the tie's hold loop starting "matched") and RV-M3 (the capacity check as `>=`)
- **Re-checked: REAL**, both SURVIVED (07:31:03, 07:31:05).
- **RV-M2 fixed:** the tie case gains pivot 7's entry gone (`motorCount` 1), and pivot 7 declared only as a
  PNEUMATIC entry of the same name — both `CHECK_FALSE`. After: red at :837 and :843 (07:42:27).
- **RV-M3 fixed:** a new case, "capacity is EIGHT of each kind". Eight holds all naming pivot 6 → the
  coverage sentence ("the robot declares 'claw pivot 7', and no control names it"); eight toggles all naming
  the claw on A → the loop's own sentence ("button A drives both mechanism 'claw' and mechanism 'claw' -- one
  button, one job", observed first, then pinned); nine → the capacity sentence. Eight holds can never ALL bind
  (16 buttons; a controller has 12), so the pin is WHICH sentence refuses them. After: red at :461 and :466
  (07:42:29).

### Finding 5 — `check-examples` checked membership only
- **Re-checked: REAL.** 07:34:56: `{"intake", Button::X, Button::B, 12_volt},` added to `kHolds` →
  `check-examples` "460 quoted lines, all verbatim", exit 0. Restored, `cmp`-identical, re-stamped.
- **Fixed at the root, in the tool as well as the words.** `WHOLE_RUN_SOURCE_GLOBS = ["src/team/*_driver.hpp"]`:
  a listing ALL of whose lines are lines of one such file must also be ONE UNBROKEN RUN of that file's
  non-blank lines — nothing skipped, nothing reordered (blank lines ignored on both sides; it may start and
  stop anywhere). Each entry must also be an `EXAMPLE_SOURCE_GLOBS` entry, or the scan fails. `EXAMPLE_SOURCE_GLOBS`
  itself is UNCHANGED (item 6's shape, shared with COMP-t). The membership failure message now says "Fix the
  SOURCE first — the example test, or the team's file under src/team/ — then re-quote"; the new failure names
  the listing and the file and says to re-quote.
- The cookbook's `bind()`/`run()` listing skipped the three `///` lines between the two functions, so it was
  re-quoted with each function's own comment — an unbroken run of the file's :87-104. `check-examples`: 467
  quoted lines (was 460), 3 listings of whole-run files.
- **Observed:** the intake line → FAILS naming the WHICH BUTTON block (block 5; 07:36:39); a
  `{"wing", Button::B},` toggle → FAILS, the same block; each restored `cmp`-identical. The self-test gained
  six cases: an unbroken run passes (with blank lines on both sides); an added line, a reordered listing and a
  skipped line each fail; an example test is not a whole-run kind; a whole-run kind not in
  `EXAMPLE_SOURCE_GLOBS` fails. **The rule mutated** (07:36:59; the tool backed up and restored `cmp`-identical):
  G1 the new check never fails, G2 the rule applied to the example tests too, G4 order ignored, G5 the
  EXAMPLE_SOURCE_GLOBS guard dropped → each turns the self-test red; G3 the file's blank lines kept → the
  self-test red AND `check-examples` red on the real cookbook (block 7).
- **Words:** cookbook 06 (:264-266) says each listing is an unbroken run and what fails; guide README and
  test/README say it in a clause; the tool's and `test/CMakeLists.txt`'s comments say "quote from" / "part by
  part", never "quote whole"; this log's coordinator item 3; guide-maintenance.md; build-order :632.
- **Measured on the way:** there are 49 ```` ```cpp ```` listings in the public docs. The 46 that lie wholly
  inside one example test are ALL unbroken runs already, and the other 3 are the driver file's. So adding the
  example tests to the rule would need no chapter re-quoted. The tool's comment records that as a separate
  decision; it is not made here, because it is not this finding's and it changes a gate the DOCS chunks own.

### Finding 6 — the 1a-only header wording had no instruction telling 1b to undo it
- **Re-checked: REAL.** The brief's per-part list (§1 Part 1a item 2) says only that 1b "adds" the Teleop
  clause. A 1b executor would leave "there too … `make` refuses …" in place after the tie is gone.
- **Fixed in the brief** (the coordinator's to take or drop — a plan document, amended only to carry out what
  1a already did): the 1b bullet gains a sub-bullet that quotes 1a's clause and tells 1b to delete it and
  restore §2's "change it HERE and nowhere else". §4's Part 1b list gains "the interim sentences Part 1a wrote":
  that clause, cookbook 06's "both files" bullet, and HANDOFF L5's "ALSO stated … until T4 Part 1b".

### Finding 7 — small doc-accuracy slips
- **(a) REAL.** README.md was edited (:296-301) and left off the documents list. It is added there with a
  correction: the first pass hand-edited TWELVE documents, not eleven.
- **(b) REAL.** The re-zero is stated in three places: the driver's `kLoop.rezeroButton`, the table's
  `kTeleopRezeroButton` and Teleop's `kRezeroButton`, tied pairwise. Cookbook 06's bullet now says "— and
  the re-zero button in three, because "shulib Teleop" also names its own".
- **(c) REAL.** Cookbook README :4-6 now says the recipes run "on the host test plant", and that the driver
  file "runs on the host through a stand-in for PROS". The same paragraph's "against the simulated robot"
  (:26) became "on the host test plant" (the vocabulary rule).

### Documents this round, and the count
- **Touched:** `docs/roadmap.md`, `HANDOFF-remaining-work.md`, `build-order.md`, cookbook 06, cookbook README,
  guide README, `test/README.md`, `docs/internal/guide-maintenance.md` (new to the part), the T4 brief (new to
  the part: the 1b instructions), this log, and the briefing (regenerated). Code and tools:
  `tools/api_doc_tool.py`, `test/CMakeLists.txt` (comments only), `test/driver_controls_test.cpp`,
  `test/robot_two_driver_test.cpp`. `docs/api` is unchanged (no header changed; `check-fresh` passes).
- **Pace rule 7:** Part 1a has now hand-edited FOURTEEN documents: the first pass's twelve, plus
  guide-maintenance.md and the brief.

### Gates, on the tree as left (07:48-07:58)
- **Host** (a fresh configure; the briefing regenerated before the build and again after the binary): `[doctest]
  test cases: 1685 | 1685 passed | 0 failed | 3 skipped`, `assertions: 2469415 | 2469415 passed | 0 failed`
  (07:57; the first pass left 1,684 / 2,469,300: +1 case, the capacity case, and +115 assertions).
- **`src_build_gate.py self-test`:** `OK (189 detector cases, each against a real build)` (07:48:57-07:53:53);
  **`check`:** `src build gate: PASS (all 6 builds link; …)`, each build 8/8 TUs, 0 disallowed, `[include-root]
  24 source files in src/ and 4 folders`, `[printed-path]` clean (07:54:56). No `src/` file changed this round.
- **ARM header gate:** exit 0 over 191 headers. **CI greps:** PROS-free, sim-free (07:48:59).
- **`api_doc_tool.py`:** self-test OK (with the six new cases); check-coverage 2908 entities / 155 headers;
  check-fresh exit 0 (`docs/api` unchanged); check-examples `467 quoted lines, all verbatim (5 source files);
  3 listings of whole-run files, each one unbroken run`; check-removability clean. **`briefing_status.py
  check`:** current. **`doc_staleness_audit.py`:** self-test OK, audit clean.
- **The runner's last act** (07:56:45-07:56:51) re-ran RV-M1/M7/M2/M3 once more, to name the failing cases:
  RV-M1 and RV-M7 → T10 AND "DriverBindings refuses, by name"; RV-M2 → the interim-tie case; RV-M3 → the
  capacity case. Every file restored `cmp`-identical and re-stamped; the host build and suite above ran AFTER it.
- **Nothing committed; no `pros` command; no `/dev/ttyACM*`.** `bin/` and `.d/` are the src gate's own outputs
  (git-ignored).

### For the coordinator (the review round)
1. **A gate rule the brief did not ask for:** `WHOLE_RUN_SOURCE_GLOBS` (finding 5). `EXAMPLE_SOURCE_GLOBS` is
   unchanged, so COMP-t's append still works as briefed. Whether `src/team/autons.cpp` should also be a
   whole-run kind is COMP-t's call. The same holds for the example tests (46/46 of their listings already
   pass, so it would cost no re-quote). Take the rule, or drop it and keep only the corrected words.
2. **The brief was amended** (finding 6): two additions telling Part 1b to restore §2's "and nowhere else" and
   to remove 1a's three interim sentences. It is a plan document, so this is yours to accept.
3. **The first pass's report said three "empty until" sentences; there were six.** Its prose review did not
   grep `src/team` over `docs/internal/`. Both are corrected above, and the grep was re-run (`prose_review_4.txt`).
4. **Pace rule 7:** fourteen hand-edited documents for Part 1a (twelve plus guide-maintenance.md and the brief).

## Part 1a — the coordinator's merge and verification (2026-09-27, 08:00–08:15)

**The rulings on the review round's three questions:**
1. **`WHOLE_RUN_SOURCE_GLOBS` is KEPT.** The rule closes a real hole the builder demonstrated (a control
   ADDED to the driver file passed `check-examples`, so a public page could show a driver file that is not
   the file). A listing of a file the team edits must be the file, unbroken. It is a tooling rule, so it is
   the coordinator's call; the COMP-t brief is told (its header): `src/team/autons.cpp` joins the list too,
   for the same reason, when COMP-t appends its example entry.
2. **The brief amendment (Part 1b restores §2's "and nowhere else" and removes 1a's three interim
   sentences) is ACCEPTED** — it keeps every sentence true at every commit, which is the brief's own rule.
3. **Fourteen hand-edited documents** is over pace rule 7's "about five"; accepted, because the review's
   major finding was precisely a document the first pass had not reached.

**The merge:** the t4-1a worktree (the build, the review round, the fixes) as the transport commit
`4d9bbcf`, squash-merged onto `9e86a3c` with no conflicts.

**Verification on the merged tree (08:01–08:09):** suite **1685 | 1685 passed | 0 failed | 3 skipped**,
2,469,415 assertions; `src_build_gate.py` self-test **OK (189 detector cases)** and check **PASS** on all six
builds (8/8 TUs each, 0 disallowed, beacons and programs checked, include root unambiguous, printed paths
real); the ARM header gate clean (191 headers); both CI greps empty; `check-fresh` 0; the briefing current
(regenerated before the build and after the suite binary); removability empty; the staleness audit clean.

**Five coordinator mutations, each on a fresh binary or a fresh build, each restored and hash- or
`cmp`-checked:**

| # | Mutation | Observed |
|---|---|---|
| H1 | `driverHeading()` hands the loop a heading of 0 (the trust flag untouched) | RED: `driver_controls_test.cpp:536` and `robot_two_driver_test.cpp:557` |
| H2 | `RobotTwoDriver::run()` passes the loop NO hold bindings | RED: 5 cases, 58 assertions (`robot_two_driver_test.cpp:283` — pivot 6 not braked, not driven at 12 V) |
| H3 | `RobotTwoDriver::run()` passes the loop NO toggle bindings | RED: 5 cases, 25 assertions (`robot_two_driver_test.cpp:370` — the claw's line never written) |
| G1 | the table's port-7 forward moved L1 → X, the driver file untouched; a fresh `make ROBOT=tank PROGRAM=library` | **make exit 2**: `teleop_program.cpp:186` "the driver file and the mechanism table disagree about a button -- change both until T4 Part 1b" |
| G2 | pivot 6's line cut from the cookbook's driver-file listing | **`check-examples` exit 1**, naming the listing and saying to re-quote it; clean again after the restore |

**Not done here:** no program runs the driver file yet (COMP-b is its first caller); nothing of it has run
on a robot.

## Part 2a — the drive protections, the LIBRARY half

- **Started:** Tue Sep 29 03:39:51 AM EDT 2026 (`date`).
- **Tree at start:** `9239538`, clean, in the git worktree `.claude/worktrees/t4-2a` (branch `t4-2a`) — NOT the
  main working tree.
- **Executor model:** opus.
- **Go:** the coordinator's workflow run, on the team lead's go for Parts 2 and 3 (2026-09-28, "i want to do all
  this") and his approval (2026-09-28 ~17:35, speedup B) to SPLIT Part 2: **Part 2a now** (the library half:
  §1 Part 2 items 1-5 and 7, their tests and library documents), **Part 2b after COMP-b** (item 6 — the `src/`
  words, the PROTECTION row on the competition program's panel, `perTick`'s fourth argument in COMP — and item
  8, the re-read of COMP-b's files). Do not commit; no `pros`; no `/dev/ttyACM*`. Mutation backups and probes
  under `.claude/coordinator/scratch/t4-2a/` only.
- **Answers in force:** C (a) — the velocity-exactly-0 check DROPPED; D (a) — guard G4 (majority-or-tie, and a
  repeat needs position AND velocity unchanged); N7 (c) — refuse mid-run only when 3+ drive motors are dead in
  total; N1 (a) — the over-current cut ON, one switch off, a push check required; N2 (a) as amended; N3 —
  HA-175 (the absence rule) and HA-176 (the over-current cut) only.
- **In flight beside this build (their files are not mine):** comp-t, tank1b, DIAG1 Part B, SAFE-SAVE Part A,
  L9 Part A1, H1 Part A1, R5a Part A. Shared plan docs: my own lines only.

### 03:40 — start
- Read `docs/internal/RESUMING.md` and the brief in full (header rulings and answers, the 2026-09-28 re-check
  amendments, § Decisions, §0-§8). This heading appended before any code.

### 03:40–04:05 — required reading, the anchors, the baseline *(`date` 04:05 at this entry)*
- Read in the tree at `9239538`: `include/shulib/teleop/teleop_loop.hpp` (whole), `hal/motor_group.hpp` (whole),
  `teleop/coupled_side_monitor.hpp` (whole), `teleop/drivetrain_degradation.hpp` (whole), `chassis/chassis.hpp`
  `drive()`/`cancel()`, `motion/motion.hpp` (`MotionDeps`, `tickHealthObservables`), `robot/robot.hpp` (the
  constructor, `describe()`, `MotorWiring`, `wireMotors`, `makeSideGroup`, `makeGroupArray`, `appendAbsent`),
  `hal/fake/fake_motor.hpp`, `sim/drive_plant.hpp` (`MemberFault`, `setMemberFault`, `writeMember`),
  `test/robot_builder_test.cpp` (the `PlantWorld`/`BuiltRobot` fixture :150-262), `test/teleop_loop_test.cpp`
  (the fixture), the OLD `src/programs/drive_program.cpp` (whole — the words and the order the protections move
  from), `src/programs/teleop_program.cpp` (the boot lines :250-300, the loop call :975-1000),
  `src/team/robot_two_driver.hpp` (header), worksheet Station F (whole), `version.hpp`, the changelog's head, the
  Freeze Register. Read the re-check's probes (`.claude/coordinator/scratch/recheck-2026-09-28/slip_probe.cpp` and
  its results) — the G4 rule is implemented from their semantics: `frozen = suspect && nOk >= nSus` over the
  present, unlatched members; a repeat needs position AND velocity bit-identical.
- **Every anchor in the brief's re-anchored table (`e124b88`) re-checked at `9239538` by its quoted text — ALL
  MATCH, no delta:** `teleop_loop.hpp` preconditions :213-219, claim release :223-240, disable :262-269, `drive()`
  :291, release :294, `perTick` :307, overloads :316/:328; `chassis.hpp` `drive()` :419-457; `hal/pros/motor.hpp`
  `position()` :233, `velocity()` :247, `current()` :259; `device_provider.hpp` `telemetry()` :376, `TermSink` :467;
  `motor_group.hpp` "N = 5" :37, "later chunk" :82-90, `isPresent` :291, `member(i)` :299-305;
  `coupled_side_monitor.hpp` "five motors" :8-9, `SideMonitorConfig` :67-73, "the drive program" :15/:40/:65/:171/:195,
  `MemberAbsenceDetector` :197-252; `drivetrain_degradation.hpp` THE REASONING :19-29, DOMAIN :31-37, caller :38-40,
  :45, constants :46/:50; `robot.hpp` `makeSideGroup` :922 (called :513-514), `describe()` :747;
  `teleop_program.cpp` `kDeadPortLine1/2` :274-279, `loopCfg` + the call :988-998.
- **The tree beside the brief:** COMP-b has NOT landed (no `src/common/driver_panel.hpp`, no
  `src/programs/competition_program.cpp`) — so the PROTECTION row, the chooser's words, COMP's `perTick` and item
  8's re-read are Part 2b's, as the split says. SRC1's layout, Part 1a's driver file and TANK1 Part A (API 2.3) are
  in the tree as the brief describes.
- **Baseline, host** (a fresh `cmake -S test -B build/test` at `9239538` + this log; the briefing regenerated
  first): `[doctest] test cases: 1725 | 1725 passed | 0 failed | 3 skipped`, **`assertions: 2648032 | 2648032
  passed`** — the bit-identity pin's "before" (the committed briefing says the same).
- **HA numbers:** the register's status paragraph says HA-162..HA-178 are held by briefed chunks and next free is
  HA-181; HA-175 and HA-176 are T4's (N3). No other number is taken.

### 04:05 — the scope of 2a, decided before any code (each a delta from the unsplit brief, logged)
1. **Part 2b's, NOT built here:** item 6 in full — the shared panel's PROTECTION row and chooser words, COMP's
   `perTick`, the driver file's protections paragraph and `run()`'s doc sentence (so P2-18, its source pin, and
   the cookbook's re-quote of the driver-file listing), Station K's push check (Station K is COMP-b's, not yet
   written) — and item 8, the re-read of COMP-b's files. §3's "shared panel (host)" tests are 2b's.
2. **In 2a because the library change makes them FALSE (the coordinator's rule: "any src/ sentence your change
   makes FALSE must be corrected in this part"):** Teleop's `kDeadPortLine1/2` (Teleop reaches the protections
   through `runDriverControl`'s defaulted field, so "AT BOOT ONLY" and "a port that dies AFTER boot is still
   shulib Drive's" go false the moment this lands) — with the literal diff check the brief gives; and worksheet
   Station F's dead-port paragraph, its intro's "shulib Drive (slot 1) stays the program that notices a port dying
   mid-run", and its fault row's "the drive CONTINUES" for MOTOR_GROUP_DISAGREE (a fight now also CUTS for 1 s).
   The Teleop source pin (its `loopCfg` names no `protection`) is 2a's: it is what makes Teleop's protections true.
3. **Decisions in force:** C (a) — no velocity-exactly-0 signature beside the FROZEN rule (`MemberAbsenceDetector`
   stays in the library unchanged, with its tests; P2-19b is not built, and the mutation rows marked "C (b) only"
   do not apply); D (a) — G4; N7 (c) — `evaluateRuntimeDegradation`; N1 (a) — `overCurrentCuts = true`.

### 04:05–04:50 — the library code *(`date` 04:50 at this entry)*
- **`include/shulib/hal/motor_group.hpp`:** `markAbsent(i)` (noexcept, refuses out of range / a boot hole / a
  second mark / the last present member), `runtimeAbsentMask()`, `setMonitorExcluded(mask)` /
  `monitorExcludedMask()` (bits past the range, on holes and on runtime-absent members dropped), labels (a third,
  defaulted constructor argument `std::span<const int>`, `label(i)`). Every read goes through one private
  `readable(i)` (not a hole, not marked); `setVoltage`/`setBrakeMode` still skip only holes, so a marked member is
  still commanded. `member(i)` keeps its precondition on a BOOT hole only. Header: "N = 5" → four per side (record of
  2026-09-25), the "WHY BOOT, AND NOT RUNTIME … a later chunk" paragraph rewritten, and a new section "A MEMBER THAT
  DIES MID-RUN".
- **`include/shulib/teleop/coupled_side_monitor.hpp`:** `MemberReading`, `FrozenVerdict`, `FrozenReadingDetector`
  (G4: a repeat needs position AND velocity bit-identical — `std::bit_cast`, so a held reading is a repeat and 0.0 vs
  -0.0 is not; FROZEN = SUSPECT and `answering >= suspect` over the present, unlatched members; latched after
  `persistTicks` FROZEN ticks; precondition `0 < suspectReads < persistTicks`). Prose: THE HAZARD's "five motors per
  side … the other four" → several (five in the typed table, four on the record); "the drive program" → the old
  "shulib Drive" / the driver loop's protections (:15, :40, :65); `MemberAbsenceDetector`'s doc now says it is the
  faulted-read detector for a caller that HAS the count, and why the library does not run it (C (a)). :171
  (`tankSideVolts`) left for Part 3, as the brief says.
- **`include/shulib/teleop/drivetrain_degradation.hpp`:** `evaluateRuntimeDegradation(left, right)` (N7 (c): only the
  total cap; an EMPTY side is legal there — a one-group chassis — while a negative count or more answering than
  expected refuses). THE REASONING kept as reasoned (for five) and a new paragraph on four per side; DOMAIN; the
  callers (the boot check, the library's protections, the old Drive until Part 3); `kMinAnsweringPerSide`'s doc.
- **`include/shulib/teleop/drive_protection.hpp` (NEW):** `DriveProtectionConfig` (the six fields, their two
  static_asserts), `ProtectionMode`, `kMaxProtectedGroups` (4), `kProtectionTextBytes`, `DriveProtectionState`
  (trivially copyable, static_assert), `validProtectionConfig()`, `DriveProtection` (enter / beforeDrive /
  commandAllowed / afterDrive / onDisable / state / config; destructor clears every mask).
- **`include/shulib/teleop/teleop_loop.hpp`:** `DriverLoopConfig::protection` appended LAST; the entry precondition
  beside `tickPeriod` and `validBindings`, BEFORE the claim release; one `DriveProtection` built and `enter()`'d after
  the entry safe state and the release mode; each tick `beforeDrive()` → `drive(allowed ? command : zero)` →
  `afterDrive()` → 6b …; `onDisable()` after `cancel()` on the disable; `perTick`'s fourth argument by `if constexpr
  (std::is_invocable_v<…>)`. Header: the tick order, a THE DRIVE PROTECTIONS section, the banner's callers (Teleop by
  its own call, the X-drive, the driver file, COMP-b's `opcontrol()` "will").
- **`include/shulib/robot/robot.hpp`:** `MotorWiring` gains `leftLabels`/`rightLabels` (port magnitudes, filled in
  `wireMotors`); `makeSideGroup` passes them; the header, `describe()`'s and `appendAbsent()`'s docs say a runtime
  mark is named too.
- **Deltas from the brief found while building (each with its reason):**
  1. **The frozen-reason words are formatted from the group's own window** — "reads frozen for %s while its side
     answered", the `%s` from `persistTicks` × the loop's tick ("250 ms" at the defaults, pinned) — rather than a
     literal "250 ms" in the detector: a group built with another window must not print a false duration. The
     detector has no `reason()`; the protections' line carries it.
  2. **`DriveProtectionState::dead` lists EVERY dead drive port**, boot holes and runtime marks, not only the
     runtime-dead: the verdict it sits beside counts both, and a PROTECTION row reading `DEGRADED dead` with an
     empty list (a boot hole) would be false.
  3. **The REFUSED line drops the policy reason's own leading "REFUSE: "** (the old program printed "REFUSED:
     REFUSE: …"), and its duration is the true one, "0 V for the rest of the run, every later start of driver
     control included" — the old "0 V until the field disables" was false for the library (the marks persist, P2-7).
  4. **No disable summary for bare drive motors** (the X-drive): the brief asks for one line per GROUP. A first draft
     added a "wheels" line; it put a new TEL line into three pre-existing tests' drives (a ChassisRig of bare
     motors), and it is not in the brief — removed.
  5. **`kMaxProtectedGroups` = 4** (a tank's two, or a four-wheel holonomic with a group per wheel) — the object lives
     on the driver-control task's stack (about 5 KB), so its capacity is bounded; a chassis with more groups is
     refused by the entry check, before anything moves. Not in the brief; stated.
  6. **Two `-Wformat-truncation` findings** (the host build's `-Werror`): the entry REFUSE line could exceed 200
     bytes at its worst case, so it is two lines (the refusal with the census; then `  dead: … -- restart the
     program once the robot is fixed`), and `dead` lists print with a precision. The api-doc parser also refused
     `1 << 30` (read as a template bracket), so the counter caps are written `0x40000000  // 2^30`.
- **The unchanged suite on the new library code** (`docs/api` regenerated, the briefing regenerated before the build):
  `1725 | 1725 passed | 0 failed | 3 skipped`, **`assertions: 2648037`** (was 2,648,032). The +5 was traced, not
  assumed — a scratch build of `9239538` (`git archive` into `.claude/coordinator/scratch/t4-2a/base/`) run with
  doctest's XML reporter, compared case by case: exactly two cases differ, both explained and neither a drive
  change — "only hal/pros includes <pros/>" 522 → 525 (the in-tree CI guard checks each header: one more header),
  and "T4 1a T9: a claim left on pivot 6 …" 15 → 17 (its last loop CHECKs every sink line of the second entry, which
  now includes the two groups' disable summary lines). Every other case's count is identical: the drive is
  bit-identical wherever nothing is dead, fighting or over current.

### to 10:54 — the tests *(`date` 10:54:18 at this entry)*
- **`test/drive_protection_test.cpp` (NEW, 25 cases):** a `World` — robot two's SHAPE (four a side, the record's
  ports L -16 +17 -18 -20, R +11 -12 +13 +15, drive-encoder odometry) built as a `robot::Robot` over the
  `FakeDeviceProvider` and driven through `runDriverControl` with a `FakeController` — in two modes: the HOST TEST
  PLANT (members hung on its coupled train, faults from `setMemberFault`; currents by hand, the plant synthesizes
  none, which the file says) and HAND-STEPPED (`HandSide`: the re-check's `slip_probe` model — train lag, BLUE's 300
  counts with a quarter-count phase per member, whole-RPM velocity reports with a lag, a wall, a slipping member —
  run through the REAL loop, monitor and detector). Cases: P2-1, P2-2+P2-14, P2-14 (three-argument form), P2-3, P2-4
  (seven subcases), P2-4b, P2-5, P2-6, P2-6b, P2-6c, P2-6d, P2-7 (N7 (c), and the next entry), P2-8, P2-9, P2-11,
  P2-12, P2-13 (the X-drive), P2-15, P2-16, P2-16b, P2-17 (the builder's labels; and a HAND-MADE unlabelled tank,
  `UnlabelledTank`, for "member i"), P2-19, P2-19c, P2-20 (a counting `IMotor`/`IDigitalOut` decorator on every drive
  port, the pivot and the claw's line), P2-21 (drag 0 and 0.8), P2-21b, P2-22 (the disable return, and a throw), and
  the source pins (Teleop's `loopCfg` and the driver file's `kLoop` name no `protection`).
- **`test/motor_group_test.cpp`:** P2-10 — `markAbsent` (out of every read, still commanded, refusals without a
  throw, `member(i)` for a mark vs a hole, labels) and `setMonitorExcluded` (fed `present = false` and nothing else;
  dropped bits; a zero mask bit-identical over 30 evaluations).
- **`test/coupled_side_monitor_test.cpp`:** P2-10b — `FrozenReadingDetector` alone, each rule at its boundary (4 vs 5
  reads; own speed 0.50 vs 0.51; mates 1.00 vs 1.01; a repeating mate not counted; D (a)'s velocity-changed read is
  no repeat; the guard 1/2/3/4 suspect of four; a tie on a 2-of-4 side; the latch at 24 vs 25 and latched for life;
  a whole side alike; the precondition).
- **`test/drivetrain_degradation_test.cpp`:** `evaluateRuntimeDegradation` — robot two's 2/4 on one side drives
  mid-run (and the boot rule still refuses it), the third dead anywhere refuses, five a side, an empty side legal,
  nonsense refused.
- **The tick numbers, and where they come from (each derived before it was pinned, then observed):** on the plant,
  tick 0 reads the robot at rest and a stick change first shows one tick later (the plant synthesizes in `pace()`).
  So the fight's first cut is tick 25 (reads moving from 1), cut #2 at 151 (the re-arm at 125, the first moving read
  at 127, its 25th at 151); an over-current set from tick 10 cuts at 34 (the current is set by hand, no lag); P2-6's
  member is FROZEN at the reversal + 5 and ABSENT at + 29 (the rulings probe's number); P2-5's at-rest death is
  FROZEN at the first moving read (101) and ABSENT at 125; P2-6c's is ABSENT at 51, inside the cut (24-124). The
  first run of the new file had four wrong EXPECTATIONS, each a test error, not a code one, fixed at the test: P2-2's
  mode on the re-arm tick itself (the 100th zero tick ends re-armed); P2-2's at-the-cut count (400 ticks hold three
  cuts, not two); P2-15 run past the fight's second cut (151); P2-12's 60th cut (every 126 ticks from 25, so 7500
  ticks, not 7590). And P2-9 was VACUOUS on its first form — one slipping member left the two middle readings equal
  (gap 0, so "moves by no more than the gap" was a check of 0 ≤ 0): it now slips two members at different ticks, so
  the middle two differ and the median really moves at the mark (both pinned: `gap > 0`, the move `> 0`).
- **P2-18 (the driver file's paragraph) and the shared panel's tests are Part 2b's** (the scope entry above).
- **Suite:** `1768 | 1768 passed | 0 failed | 3 skipped`, `assertions: 2656087`.

### RESUMED — Tue Sep 29 07:02 PM EDT 2026 (`date`), a new executor run (opus) in the same worktree
- **Why a resumption:** the team lead paused all work at a usage limit (2026-09-29 ~13:50); the run above stopped
  partway. The coordinator's continuation order: read this log, check no gate self-test plant was left in `src/` or
  the `Makefile`, build and run what is there, and continue — no finished work redone, none reverted unless wrong.
- **What the tree held at resumption (`git status`, file times, the scratch dir), checked before any edit:**
  - `src/` and the `Makefile`: the ONLY `src/` change is `src/programs/teleop_program.cpp` (the two literals and the
    header comment this log's scope entry names); `Makefile` and `tools/` unchanged — **no self-test plant was left**.
    *(Corrected 2026-10-01, the review of 2026-09-30: the 04:05 scope entry names the two literals only. The header
    comment — `teleop_program.cpp` :23-24, "the dead-port policy at boot … a port that dies mid-run is
    teleop::runDriverControl's since T4 Part 2" — is this part's own addition, made for the same reason as the
    literals: the old words "applied once, by the boot check" went false with this part.)*
  - The literal diff check (`git diff -U0 -- src/programs/teleop_program.cpp | grep '^[-+].*"'`): exactly four
    hits, `kDeadPortLine1`'s two lines and `kDeadPortLine2`'s two, before and after — the only literal changes.
  - **Work the log above does not record (done after its 10:54 entry, before the pause):**
    - a mutation run, 10:59-11:10, by `.claude/coordinator/scratch/t4-2a/mutate.py` (results in
      `mutation-results.txt` beside it) — every mutation's backup is under `scratch/t4-2a/backup/`; **every source
      header in the tree is byte-identical to its pre-mutation backup** (`cmp` against the newest backup of each of
      the five headers; all backups of one file share one md5), so no mutation was left in the tree;
    - one test-side change between the mutation passes (`test/coupled_side_monitor_test.cpp`, 11:08; the filtered
      case count went 102 → 103);
    - two prose-review greps (`scratch/t4-2a/prose_review_1.txt`, `_2.txt`, 11:11-11:12);
    - public-document edits at 13:50-13:52, stopped mid-way by the pause: guide 10, 11, 12, 14, the FAQ, cookbook
      06, the changelog. **Not yet touched:** `docs/hardware-assumptions.md` (HA-175/176, HA-132/133), worksheet
      Station F, the plan documents' own lines.
  - Each of these is re-checked below before it is logged as done: the mutations are re-read and re-run where the
    record is not enough, the document edits are read whole against the code.

### 19:02–19:10 — the tree as the pause left it, built and run *(`date` 19:10)*
- Briefing and `docs/api` regenerated, `cmake -S test -B build/test`, full build (exit 0, no warning), suite:
  **`1769 | 1769 passed | 0 failed | 3 skipped`, `assertions: 2656098`** — the 10:54 entry's 1768/2656087 plus the one
  P2-10b case the previous run added at 11:08 (a member NOT PRESENT for a tick starts over), written for mutation
  M40 below. The code headers are byte-identical to their pre-mutation backups (above), so this is the code the
  mutation pass ran against.

### The mutation pass — the previous run's (10:59–11:10), recorded here for the first time, re-read against the tree
- **Runner:** `.claude/coordinator/scratch/t4-2a/mutate.py` (scratch, not project code). Per mutation: back the file up
  under `scratch/t4-2a/backup/`, one exact replacement asserted to occur once, the depend step, then `make
  shulib_tests/fast` in a SCRATCH build dir (`scratch/t4-2a/mbuild`, configured from this worktree's `test/`) — the
  run is refused ("NOT REBUILT") when no object recompiled, which is what voided a first pass
  (`mutation-results-VOID-not-rebuilt.txt`, kept, not counted) — then the Part 2 cases, the verdict, restore,
  `cmp`-verify, and a timestamp bump so the next build recompiles. Results verbatim in `mutation-results.txt`.
- **Every row of the brief's §3 Part 2 mutation table, each observed RED on a fresh binary** (the red cases named):

  | # | Mutation (the brief's row) | Red |
  |---|---|---|
  | M1 | the FROZEN exclusion applied AFTER `drive()` | P2-6, P2-6b |
  | M2 | SUSPECT after `persistTicks` reads | 13 cases, P2-6 and seven P2-10b among them (the runner names at most 8) |
  | M3 | the mates' fastest counting a member whose position repeats | P2-6c; P2-10b (a repeating mate) |
  | M4 | SUSPECT only through the mates (own-speed branch dropped) | P2-6c, P2-6d; P2-10b ×4 |
  | M5 | FROZEN without the other-member guard | P2-21, P2-21b; P2-10b ×3 (the whole-side case among them) — **not P2-19**: delta 7 below *(corrected 2026-10-01 from "delta 3", the review of 2026-09-30)* |
  | M6 | a FROZEN member's current counted | P2-4b |
  | M7 | over-current counted at or below the 1 V command floor | P2-4 (the released-stick case), P2-4b |
  | M8 | over-current judged on the side's `totalCurrent()` | P2-1, P2-4, P2-4b, P2-16, P2-16b |
  | M9 / M9b | the fight / the over-current judged on the first group only | P2-3, P2-13 / P2-4 (RIGHT), P2-13 (M9 read `groups_[0]` of the X-drive's empty span and crashed the run after P2-3 had already failed: 39 cases ran) — *corrected 2026-10-01 (the review of 2026-09-30): BOTH forms read `groups_[0]` of the X-drive's EMPTY span, undefined behaviour, so their "P2-13" is a crash and not a catch, and the runner's 102-105-case filter judged every row; the faithful forms XM9f/XM9bf (`g < std::min(1, groups_.size())`) are RED on the full suite (P2-3 / P2-4) — the reviewers' run and this round's final-tree pass, below* |
  | M10 | the config's entry check dropped | P2-20 |
  | M11 | over-current `>=` | P2-4 (2.40 A) |
  | C1 / M12 | `overCurrentTicks` 24 — as the DEFAULT: the build fails, `static assertion failed: the 250 ms over-current window equals the fight monitor's window by choice (HA-132): change both, with a reason`; as the comparison (`>= ticks - 1`) | build / P2-4, P2-4b, P2-6c |
  | M13 | over-current streaks not reset at the re-arm | P2-4 |
  | M14 | `cutTicks` 99 | P2-2, P2-3, P2-4, P2-12, P2-13, P2-15, P2-16b, P2-17 |
  | M15 | no 0 V write on the deciding tick | P2-2, P2-3, P2-15 |
  | M16 | the cut handing `drive()` the stick command | P2-2, P2-15, P2-16b |
  | M17 | an absent member's current counted | P2-4 (the absent case), P2-4b — on its SECOND form (below) |
  | M18 | `markAbsent` leaves the member in the monitor | P2-4b, P2-5, P2-6, P2-6b, P2-6c, P2-6d, P2-10 |
  | M19 | `markAbsent` leaves it in the median | P2-9, P2-10 |
  | M20 | `markAbsent` stops commanding it | P2-7, P2-10 |
  | M21 | `markAbsent` allows the last member | P2-10 |
  | M22 | the REFUSE not re-applied at entry | P2-4, P2-7, P2-11 |
  | M23 | the per-side floor applied mid-run under N7 (c) | P2-7 (the 2/4 case), P2-17 |
  | M24 | the frozen-reading rule dropped | 9 cases: P2-4b, P2-5, P2-6, P2-6b, P2-6c, P2-6d, P2-7, P2-9 named |
  | M25 | the log bound off by one | P2-12 |
  | M26 | the fourth `perTick` argument never handed the real state | 12 cases: P2-2+P2-14, P2-3, P2-4, P2-4b, P2-5, P2-6c, P2-7, P2-11 named |
  | M27 | `overCurrentCuts` ignored | P2-4 (the warn case) |
  | M28 | the guard as item 4 first wrote it ("some other member not SUSPECT") | P2-21, P2-21b; P2-10b (the guard's boundary) |
  | M29 | the repeat on position alone | P2-19, P2-19c, P2-21, P2-21b; P2-10b (velocity changed) |
  | M30 | no mask clear in the destructor | P2-22 |
  | M31 | the peak line only on the disable path | P2-1, P2-16b |
  | M32 | the config check after the claim release | P2-20 (its first form did not compile — an unused variable — and is not counted; the second form ran red) |
  | M33 | `member(i)` throwing for a runtime-absent member | P2-10 |

  **Beyond the table, each also red:** M34 `setMonitorExcluded` keeping bits on holes and marks (P2-10); M35 the
  runtime cap off by one (P2-7 and both runtime-degradation cases); M36 the peak line on an EQUAL peak (P2-1); M39
  the latch at `persistTicks - 1` (P2-10b's latch case, P2-4b, P2-5, P2-6, P2-6b, P2-6c, P2-6d, P2-7); M41 the census counting absent members as
  answering (P2-10b's two-gone tie); **C2** `frozenSuspectReads` default 25 — the build fails, `static assertion
  failed: the frozen-reading rule must decide before the fight monitor can cut on a dead member's stale velocity`.
- **Not applicable (C (a) dropped the velocity-0 signature):** the two "C (b) only" rows. **Part 2b's:** "the
  defaults' paragraph drifted" (P2-18 pins the driver file's paragraph, which is 2b's).
- **Survivors, each resolved:**
  - **M17's first form** (only the streak's `isPresent` dropped) survived: EQUIVALENT by construction —
    `readCurrents()` already zeroes a non-present member's current and `overCurrentAmps > 0` is an entry
    precondition, so the streak's own guard is defence in depth. Strengthened to drop BOTH (the read and the
    streak): red, above.
  - **M40** (a non-present member's repeat not reset) survived the first pass; the previous run added the P2-10b
    case "a member NOT PRESENT for a tick starts over" (11:08) and it went red (11:10).
  - **M37** (no mask clear at `enter()`) survived both of the previous run's passes: through `runDriverControl` it
    is unobservable (the first `beforeDrive()` re-sets every mask before the first `drive()`). This session added
    **P2-22b** (below), which pins `enter()`'s clear on `DriveProtection` directly — its public contract for a
    program with its own loop: **red at 19:14:45.**
  - **M38** (no `resetDisagreement()` at the re-arm) survived: **EQUIVALENT, as the brief's §3 states** (the monitor
    resets every streak on each tick the side is commanded 0 V, and a cut is 100 such ticks). Kept for the old
    program's stated intent.

### 19:10–19:20 — tests added this session, each with the mutation that turns it red (observed)
- **P2-22b** `enter()` clears an exclusion mask it finds (and construction touches nothing) — **M37 red** (above).
- **P2-21c THE KNOWN GAP** (decision D (a)'s accepted cost, the re-check's S10), pinned on purpose: a LEFT side
  already at 2 of 4 (ports 16 and 17 marked before the run), its slipping member (port 18) running while its last
  live mate (port 20) stalls at a wall — a tie, so port 20 is marked at **tick 360** (1.6 s into the push; observed,
  pinned), the third dead, and the drive REFUSED to the end; the slipping member and the RIGHT side are never marked.
  HA-175 and guide 12 state this cost; the test keeps the statement true. **M42** (a STRICT-majority guard,
  decision D (d)) **red at 19:15:18**: P2-21c, P2-7 (its 2-of-4 tie), and P2-10b's two tie cases — so the guard's
  tie is pinned from both sides.
- **The source pins now cover the X-drive** (`src/programs/xdrive_program.cpp`'s `kLoopConfig` names no
  `protection`): `teleop_loop.hpp`'s banner says every caller gets the defaults "since none names the field", and
  the X-drive is one. **M44** (by hand, a source pin — the binary reads the file at run time): `.protection =
  {.overCurrentCuts = false}` added to `kLoopConfig` → both X-drive checks red; restored from
  `backup/M44__xdrive_program.cpp`, `cmp`-verified, timestamp bumped, `git diff` empty.
- Suite after: **`1771 | 1771 passed | 0 failed | 3 skipped`, `assertions: 2656801`.**

### 19:20–21:40 — documents, the gate's finding, the prose review *(`date` 21:40 at this entry)*
- **The documents the pause left half-done, each read whole against the code before it was counted:** guide 10's
  "The drive protections — in the loop, for every program", guide 12's new symptom "The drive stops for a second,
  or a motor is reported dead", guide 14's status sentence, guide 11 (a `MOTOR_GROUP_DISAGREE` row the fault table
  never had, and the driver-control note under the TRAVEL reading), the FAQ's held-reading note, cookbook 06's
  protections paragraph, and the changelog's entry "Added and changed since 2.3" (no API bump: `teleop/` and
  `hal::MotorGroup` are in no Freeze Register row, `version.hpp` bumps `kApiMinor` for a frozen surface only, and
  IMotor (F4) is untouched — the Freeze Register re-read to confirm). Every quoted log line was checked against
  its format string in `drive_protection.hpp` (`[WARN]`/`[ERROR]` against `term_sink.hpp`'s level words).
- **Corrected in them:** the examples named **port 13 as a LEFT port** (guide 10's peak line, guide 12's
  over-current line, two comments in `drive_protection.hpp`) — the brief's examples came from the typed table,
  where 13 was LEFT; on robot two's record 13 is RIGHT (L 16 17 18 20, R 11 12 13 15). Now port 17 / 18, and the
  `dead` list's example is in group order ("18 13"). Cookbook 06's paragraph was moved one paragraph down: it had
  split the snippet from the paragraph describing that snippet's behaviour.
- **Written this session:** `docs/hardware-assumptions.md` — HA-175 and HA-176 (index rows, in number order, and a
  "T4 Part 2 added TWO" section after TANK1's, with claim / source / confidence / settle / blast radius / contained,
  worded for C (a) and D (a) as the brief's superseding text gives, plus the known gap P2-21c now pins), HA-132's
  consumers and blast radius and HA-133's blast radius AMENDED, the status paragraph (160 entries; HA-162..174 and
  177..178 held elsewhere), and HA-123's held-velocity sentence given the driver-loop note the FAQ and guide 11
  carry. Worksheet **Station F** (still Teleop's): its intro's "shulib Drive (slot 1) stays the program that notices
  a port dying mid-run", the L/R row's amber (a mark mid-run shows `3/4 present`; a cut shows `+0.0 V`), the fault
  row's "the drive CONTINUES", the dead-port paragraph (rewritten from the code: the ABSENT and REFUSED lines, what
  the screen does and does not show), a new "What a CUT looks like", F.2.7's disable summary lines, and what to send
  back. The **COMP brief**'s two sentences that Part 2a makes false (:112-113 and B11's last bullet — "Until then
  this program's driver control has no mid-match absence rule and no cuts"): struck through and replaced with what
  holds (logged for the coordinator below). The plan documents' own T4 lines: build-order's T4 status and Part 2
  bullet and the Next block's "T4 Part 2" words, the roadmap's T4 row, HANDOFF L5b, REMAINING-WORK item 9.
- **Headers touched again (comments only):** `drivetrain_degradation.hpp` (`SideCount`'s "the chassis table lists"
  → the description's ports; `DriveVerdict`'s "What the drive program is allowed to do" → at boot or mid-run),
  `motor_group.hpp` (its fault-policy paragraph said "the driver must be TOLD, not stopped" — in driver control a
  fight now CUTS for 1 s: one sentence added). **`teleop_program.cpp`'s `kDeadPortLine2`** reworded again for
  accuracy — "a drive port whose readings freeze 250 ms is dropped" became "a drive port reading frozen while its
  side turns is dropped after 250 ms" (186 bytes, inside the static_assert's 200; the literal diff check still gives
  exactly `kDeadPortLine1/2`'s four lines).
- **THE GATE'S FINDING (the first `src_build_gate.py` run of this part, 21:11-21:25):** `check` FAILED in all six
  builds — `disallowed warning -Wformat-truncation= at include/shulib/teleop/drive_protection.hpp:783 — 'snprintf'
  output may be truncated before the last format character` (two TUs a build); the self-test failed cases 4, 5,
  11, 13 and 15 on the same warning, and restored every plant (`git diff -- src/ Makefile` after it: Teleop's two
  literals only). The host build never saw it: only the ARM build at `-Os`, with `runDriverControl` instantiated by
  a program, inlines `overCurrentWords()` far enough to size it — a scratch probe
  (`scratch/t4-2a/arm_probe.cpp`, the gate's flags, an opaque disable predicate so the loop is not folded away)
  reproduced it: *"output between 20 and 98 bytes into a destination of size 96"* (`DriveProtectionState::lastCut`).
  **Fixed at the format** (`"CUT: %.16s%.12s over %.3g A for %.23s"`, with a comment saying why): realistic words
  are far inside those bounds, so no pinned line changes (the suite, unchanged: 1771/2656801). The probe is clean
  after the fix; at `-Wformat-truncation=2`, stricter than the gate's policy, it lists more "may truncate" sites —
  every one a bounded `snprintf` into a 200-byte line or a 96-byte field, stated, not changed.
- **Gates after the fix** (21:25-21:34): `src_build_gate.py self-test` — `OK (189 detector cases, each against a
  real build)`; `check` — `PASS (all 6 builds link …)`, 0 disallowed warnings in every build; `git diff --stat --
  src/ Makefile` after both: `src/programs/teleop_program.cpp` only.
- **Prose review — run, then read, before this line was written** (outputs kept: `scratch/t4-2a/prose_review_3.txt`,
  21:15, the brief's terms over `README.md`, the live `docs/`, the worksheet, `src/` and `include/shulib/`;
  `prose_review_4.txt`, 21:37, every changed file's name and concept — `drive_protection`, `DriveProtection`,
  `teleop_loop`, `runDriverControl`, `DriverLoopConfig`, `perTick`, `motor_group`, `MotorGroup`,
  `markAbsent|runtimeAbsent|setMonitorExcluded`, `coupled_side_monitor`, `CoupledSideMonitor|FrozenReading`,
  `drivetrain_degradation`, `evaluate(Runtime)Degradation`, `teleop_program`, `MOTOR_GROUP_DISAGREE`, HA-132,
  HA-133, HA-175, HA-176, `port 13`, `over-current`; plus the previous run's two, 11:11-11:12). **Every hit read.
  Fixed from them:** HA-123's held-velocity sentence, `motor_group.hpp`'s fault-policy paragraph, the two
  degradation doc comments, the port-13 examples (above). **Read and left, each with its reason:** history records
  (roadmap :217-235, :291, :1448-1456; build-order's older status lines; `drive_program.*`, true of the old program
  until Part 3); `motor_group.hpp` :47-50 and `coupled_side_monitor.hpp` :181 ("the drive program" — still true, and
  the brief lists both for Part 3's sweep); `boot_check.hpp` :22 and `stick_mapping.hpp` :104 (Part 3's list);
  `src/robots/chassis_table.hpp` :228/:233 ("five motors a side" — the TYPED table IS five a side) and
  `bench_r3a.cpp` :1336 (Bench Tests' own reasoning); `teleop_claws.hpp` :384 and worksheet :1355 (a PIVOT's frozen
  temperature — mechanism motors are outside the protections, §5); "Part 1b" in `src/` (Part 3 rewrites them);
  `drive_encoder_odometry.hpp` :43's "whether it sees a member whose velocity is held too is not established"
  (still true of the MONITOR, the sentence's subject; the file is DIAG1 Part B's in flight — not touched here).
  **Not T4's, and in flight elsewhere:** `sim/drive_plant.hpp` :221-222 "robot two has five per side" (the
  re-check's item 13 listed it; Part 2 edits no plant, and R5a Part A owns `sim/drive_plant` now) — for the
  coordinator.
- **The literal diff check, re-run at the end:** `git diff -U0 -- src/programs/teleop_program.cpp | grep
  '^[-+].*"'` — nine lines, every one a line of the two literals: `kDeadPortLine1` (2 old, 2 new) and
  `kDeadPortLine2` (2 old, 3 new). Nothing else in the file's strings changed. (The resumption entry's "four
  hits" counted the four literal LINES, each shown before and after — eight diff lines then; nine now, because the
  reworded `kDeadPortLine2` takes three lines. The same two literals.)

### Deltas from the brief found in the resumed session (each with its reason; 1-6 are the 04:05-04:50 entry's)
7. **The brief's mutation table names P2-19 for "FROZEN without the other-member guard"; under D (a) it is P2-21,
   P2-21b and P2-10b's whole-side case instead (M5).** The velocity-identical repeat means a stalled side's
   decaying reports keep changing together, so no member of a live side at a wall repeats while a mate still
   reports motion — P2-19 stays green without the guard. The guard's evidence is where D (a) put it: the slipping
   member (P2-21/21b) and the detector's whole-side case (P2-10b), both red under M5.
8. **Three tests beyond §3:** P2-22b (`enter()`'s mask clear, the only way to see it — M37), P2-21c (decision D's
   known gap, pinned so HA-175's and guide 12's statement of it stays true — M42), and the X-drive in the source
   pins (the loop's banner names it among the callers that take the defaults — M44).
9. **The over-current words take a precision on each `%s`** — the ARM gate's `-Wformat-truncation` (above). The
   brief's landmine 13 asks it of `src/`; the library header needed it too.
10. **The examples' port numbers** follow robot two's RECORD (13 is RIGHT), not the typed table the brief's examples
    were written from.
11. **COMP's brief edited** (two sentences; §4 Part 2's "COMP: B11's last bullet is replaced by what now holds"),
    in this part because Part 2a lands BEFORE COMP-b and the sentences said the opposite. **Station K's push check
    is not written**: Station K does not exist until COMP-b writes it (below, for the coordinator).
12. **`DriveProtection` measured, not only estimated:** `sizeof` = **5440 bytes on ARM** (5528 on the host), the
    state 360 — the header's "about 5 KB" stands. Its per-tick read cost on the brain stays unmeasured (HA-32;
    landmine 26), as the header says.

### For the coordinator (merge notes; nothing here is committed)
- **Base and drift:** this worktree is `9239538`; `shulib-v2` has gained `ee2e399` (plan) and `a5bd83f` (TANK1 Part
  B) since. Files both this part and `a5bd83f` edit: `docs/changelog.md`, `docs/hardware-assumptions.md`, guides
  11, 12 and 14, `docs/internal/{PROJECT-BRIEFING,HANDOFF-remaining-work,REMAINING-WORK,build-order}.md`, and the
  generated `docs/api/README.md` / `all-entities.md` — each edit here is in its own paragraph, row or section;
  regenerate `docs/api`, `mkdocs.yml` and the briefing after the merge.
- **Shared or other chunks' files touched:** `docs/internal/chunks/COMP-competition-program.md` (COMP-b's brief:
  :112-117 and B11's last bullet, struck through and replaced, dated); worksheet Station F (Teleop's station — F's
  intro, rows 5 and 9, the dead-port paragraph, a new "What a CUT looks like", F.2.7, what to send back); the
  plan docs' T4 lines only; the register's status paragraph (every chunk edits it — the counts are this part's
  alone: 158 → 160, HA-175/176).
- **The changelog:** the entry is "Added and changed since 2.3" at the top of `## API 2.3`, no bump. L9 Part A1
  takes API 2.4 in flight; if it lands first, this entry moves above or below its heading by the version rule
  (it extends no frozen surface either way).
- **Station K's two lines** (what a CUT / FROZEN / dead port looks like in the competition program, and THE PUSH
  CHECK, REQUIRED before it drives a match, reading the `drive: LEFT peak …` line and any `at the cut:` lines):
  §4 Part 2 gave them to Station K, which COMP-b writes. Since Part 2a lands first, COMP-b's `opcontrol()` cuts
  on over-current from its first build — so the push check must exist before COMP-b's program drives a match:
  COMP-b writes it with the station, or Part 2b adds it before any match. Your call.
- **Not T4's, stale, in flight elsewhere:** `include/shulib/sim/drive_plant.hpp` :221-222 ("robot two has five per
  side" — four on the record; R5a Part A owns `sim/drive_plant` now). `localization/drive_encoder_odometry.hpp` :43
  (DIAG1 Part B's) is still true and was left.
- **Order lines left as they are:** the COMP brief's :22, :104 and :564 say "COMP-b → T4 Part 2 → T4 Part 3", true
  of Part 2b; REMAINING-WORK's "Where things stand" paragraph and the build-order's Next pointer beyond T4's own
  words are yours.
- **Part 2b's list** (unchanged by this part): item 6 — the PROTECTION row and the chooser's words in COMP-b's
  shared panel (host tests: every mode's words; REFUSED > CUT > fault; budgets), COMP's `perTick`, the driver
  file's protections paragraph and `run()`'s doc sentence, P2-18 and its "2.5 A" mutation, cookbook 06's "How it
  runs" re-quote — and item 8's re-read of COMP-b's files. The `DriveProtectionState` a panel reads is in the tree
  (`teleop/drive_protection.hpp`), pinned by P2-14.

### 21:45–22:10 — the final tree: every gate, and a spot re-run of the mutations *(`date` 22:10 at this entry)*
- Briefing and `docs/api` regenerated, `cmake -S test -B build/test`, full build (exit 0, no warning): **`[doctest]
  test cases: 1771 | 1771 passed | 0 failed | 3 skipped`, `assertions: 2656801 | 2656801 passed`** (the pre-change
  baseline, logged at 04:05: 1725 / 2648032). Briefing regenerated after the binary existed.
- **`src_build_gate.py self-test`: `OK (189 detector cases, each against a real build)`; `check`: `PASS (all 6
  builds link …)`, every build `0 disallowed`** — six builds, the tree's count (COMP-t's and COMP-b's are not in
  this worktree). `git diff --stat -- src/ Makefile` after both: `src/programs/teleop_program.cpp` only.
- ARM header gate (RESUMING's command, 193 headers, `drive_protection.hpp` among them): exit 0. CI greps: `library
  is PROS-free (all of include/shulib except hal/pros/)`; `layering holds: core is sim-free`.
- `api_doc_tool.py` self-test `OK`; check-coverage `2999 public entities across 157 headers, all documented`;
  check-fresh exit 0; check-examples `467 quoted lines, all verbatim (5 source files); 3 listings of whole-run files,
  each one unbroken run`; check-removability `no public doc references docs/internal/`; `briefing_status.py check`
  `generated block current`; `doc_staleness_audit.py` self-test `OK`, audit `clean`. Every tool exit 0.
- **Spot re-run on the FINAL tree** (after the format fix and the comment edits; the runner above, fresh binaries):
  M1, M5, M6, M15, M28, M29 — **all six RED**, the same cases as the first pass, plus P2-21c under M29 (the pinned
  tick moves when a repeat needs the position alone). Every header restored and `cmp`-verified.
- **Nothing committed** (`git status`: 33 modified, 3 new — the change set; HEAD still `9239538`). A rebuild after
  the spot re-run's restores (their timestamps bumped): suite again `1771 | 1771 passed`, `2656801`; briefing current.

## T4-2a review findings and their disposition

- **Started:** Wed Sep 30 03:54 AM EDT 2026 (`date`). **Executor:** opus, the builder again, in the same worktree
  (`.claude/worktrees/t4-2a`, branch `t4-2a`, HEAD `9239538`). Do not commit.
- **Input:** three reviewers' findings (correctness, tests, docs) — 27 items, two of them duplicates (the sub-1 ms
  tick is both a correctness and a tests finding) and one overlapping (HA-175 claim (b) is in the correctness
  lens's major finding and the docs lens's). Each is re-checked below before its disposition is written.
- **The tree at the start, checked before any edit** (the team lead's note: an earlier session hit its limit, so
  nothing is assumed): `git status` — the change set this log's Part 2a entries describe (33 modified, 3 new), no
  other file; every changed file's mtime is at or before 22:08:44 on 2026-09-29 (this log's last entry), so no
  later session edited the worktree; the reviewers mutated only their private copy
  (`scratch/t4-2a/review-testgate/tree`, `mut.py` :4-5), and every source and test file of the change is
  byte-identical between that copy and this worktree (`cmp`, twelve files) — no reviewer mutation was left anywhere.

### RESUMED — Wed Sep 30 11:14 AM EDT 2026 (`date`), a new executor run (opus) in the same worktree
- **Why:** the fix run started at 03:54 was stopped partway by the account's usage limit. It logged only the entry
  above; its edits are in the tree. The coordinator's continuation order: read this section, check the tree, relink
  the test binary from source, then finish every finding, re-verify the logged ones, re-run every gate, log it.
- **What the stopped run left, checked before any edit** (`git status`, file times, and a `diff` of every file it
  touched against the reviewers' copy `scratch/t4-2a/review-testgate/tree`, which the 03:54 entry found identical
  to this worktree):
  - **Six files edited 04:19-04:22, nothing after:** `teleop_loop.hpp`, `coupled_side_monitor.hpp`,
    `drive_protection.hpp`, `drivetrain_degradation.hpp`, `diag/fault.hpp` (newly in the change set: 34 modified
    now, 3 new) and `test/drive_protection_test.cpp`. Its only scratch output is `scratch/t4-2a/fix/probes/`
    (`wallslip.cpp`, its binary and `wallslip-mine.txt`, 03:55): a re-run of the wall-slip finding through the real
    loop. The diffs are saved as `scratch/t4-2a/fix2/killed-run-*.diff`. Every edit is a deliberate fix, read
    whole; each is dispositioned below with the finding it answers.
  - **No mutation and no gate plant was left:** `backup/` holds nothing newer than 22:05:38 on 2026-09-29 (the
    stopped run backed nothing up, so it ran no mutation); the newest backup of each header equals the reviewed
    copy byte for byte, or differs from it only by the comment edits this log records at 19:20-21:40
    (`motor_group.hpp`'s driver-control paragraph, `drivetrain_degradation.hpp`'s two doc comments);
    `xdrive_program.cpp` equals its M44 backup; `git diff --stat -- src/ Makefile` shows only
    `src/programs/teleop_program.cpp` (12 lines, the two literals and the comment logged at 21:40).
  - **A header the stopped run cites has no test yet:** its new `coupled_side_monitor.hpp` text names P2-21d and
    P2-19d, and its `HandSide` gained `flicker`, `dither` and `tauVOf`, but no case uses them. Finding 1 below
    writes the cases.
- **Relinked from source before any suite result was trusted:** the briefing regenerated; `cmake -S test -B
  build/test`; `cmake --build build/test --clean-first -j8` — **stopped by the build's own doc gate**: `docs/api`
  was stale for the `///` comments the stopped run changed (`fault.md` first). `api_doc_tool.py generate`, then the
  same clean build: exit 0, no warning, binary 12:05:26. The briefing regenerated with the binary present. Suite:
  **`1771 | 1771 passed | 0 failed | 3 skipped`, `assertions: 2656807`** — 22:10's 2,656,801 plus the stopped run's
  one new subcase (P2-4 driving in reverse: two REQUIREs, the 24-tick case's one CHECK, the 25-tick case's REQUIRE
  and two CHECKs = 6).

### RESUMED AGAIN — Thu Oct 1 01:42 AM EDT 2026 (`date`), a new executor run (opus) in the same worktree
- **Why:** the 11:14 run was stopped too, and so was a third run after it; neither logged past the entry above.
  This run's order is the same: read this section, check the tree, relink, finish, re-verify, every gate, log.
- **What the two unlogged runs left, from file times and the scratch directory (checked before any edit; no
  process of theirs still runs — `ps`, 01:42):**
  - **The 11:14 run, after its 12:06 entry:** a re-run of the wall-slip probe (`scratch/t4-2a/fix2/probes/`,
    14:11, the same source as the 03:54 run's — the same numbers); the four test files edited 15:09-15:10;
    `drivetrain_degradation.hpp` 15:16; a filtered build and run (`fix2/build2.log`, `p2run1.log`: 63 cases, all
    passed, 15:14); `fix2/arm/` probe copies, 15:16. Then stopped.
  - **A third run, 01:25-01:36 today:** saved that run's test diff (`fix2/run2-test.diff`), regenerated `docs/api`
    and the briefing, built and ran the FULL suite (`fix3/suite0.log`, 01:28: `1788 | 1788 passed`, `assertions:
    2658921`), compiled the two ARM probes at -O1/-O2/-O3/-Os on ARM and the host (`fix3/arm/matrix.txt`: the host
    at -O3 flagged `drive_protection.hpp:619`, the REFUSED line), bounded that line's `%s` (01:35:52), and re-ran
    the matrix clean (`matrix2.txt`, 01:36). Then stopped.
  - **No mutation and no gate plant was left:** `scratch/t4-2a/backup/` holds nothing newer than 2026-09-29
    22:05:38, so neither run mutated through the runner; every changed file was diffed against the reviewers'
    copy (`scratch/t4-2a/review-testgate/tree`, identical to this worktree at 03:54) and every difference read —
    each is a deliberate fix (the header and test edits dispositioned below), none a mutation; `git diff --stat --
    src/ Makefile`: `src/programs/teleop_program.cpp` only, byte-identical to the reviewed copy. The binary on disk
    (01:27:59) predates the 01:35:52 header edit: it is relinked below before any result is trusted.
- **Relinked from source** (the clock then read 08:40 — the session was held between 01:42 and 08:39 and nothing
  touched the worktree meanwhile, `find -newermt`): `docs/api` and the briefing regenerated; `cmake -S test -B
  build/test`; `cmake --build build/test --clean-first -j8` exit 0, no warning (`scratch/t4-2a/fix4/build0.log`);
  suite **`1788 | 1788 passed | 0 failed | 3 skipped`, `assertions: 2658921`** — the third run's 01:28 numbers (its
  01:35 edit changed one format precision, no count); the briefing regenerated with the binary present.
- **08:41-09:11, two tests added and one header comment hedged** (each dispositioned below with its finding):
  P2-20c gains the tick OVER 1000 s (the old ceiling, the other half of the correctness lens's finding); new
  **P2-5c** pins a FROZEN member's current out of `DriveProtectionState::peakAmps` AND out of the peak line's
  stretch (the reviewers' S9 survived their run unlisted; its twin in `trackStretches` had no mutation at all);
  `drive_protection.hpp`'s new reversal sentence said every member "holds near its current limit" as a fact — now
  "is expected to … (believed to be the PROS default 2.5 A, HA-154 — unmeasured)". `docs/api` regenerated (the
  line numbers moved), full build exit 0: **`1789 | 1789 passed`, `assertions: 2658984`**.
- **09:11:43 — the final-tree mutation pass started** (`scratch/t4-2a/fix4/mut4.py`, results in
  `fix4/mutation-results.txt`): every mutation of the executor's list (minus M9/M9b's undefined-behaviour forms,
  replaced by the reviewers' faithful XM9f/XM9bf) and of the reviewers' (minus S20, whose `tickMsOf` is gone), plus
  this round's T1-T4 (the tick) and S9c — 86, each target asserted to occur exactly once in the final tree, each on
  a FRESH binary in its own build dir (`fix4/mbuild`), the FULL suite (no filter: the reviewers' finding on the
  executor's 102-105-case filter), restored from `fix4/backup/`, `cmp`-verified, timestamp bumped.

### The 27 findings, each re-checked and dispositioned *(written 09:25-, 2026-10-01; the 11:14 and 01:25 runs' edits included, each read whole against the finding)*
"Red" below means observed on a FRESH binary, the FULL suite, in this round's final-tree pass (09:11-, results
and the table further down). Duplicates: 2 and 7 are one defect; 17 overlaps 1.

1. **(correctness, major) Guard G4 at a wall beside the slipping port 18 holds only while every stalled member
   reads alike — REAL. The code is decision D (a) exactly; what falls short is D (a)'s evidence, so the guard is NOT
   changed here — a ruling for the team lead, which the coordinator brings (REMAINING-WORK §11, "D, again").**
   - *Re-checked:* `coupled_side_monitor.hpp` :407-437 (every live non-suspect member counts as answering;
     `guardHolds = nAnswering >= nSuspect`); the test model's stalled members were identical by construction
     (`HandSide`'s one `w`, one `tauV`, one truncation). The reviewer's probe source, re-run through the real loop
     here (`scratch/t4-2a/fix/probes/`, `fix2/probes/wallslip-now.txt`): its numbers exactly — a 1-RPM flicker
     marks 16 and 17 at 360; a dithering encoder 17 and 20 at 360; a 30 ms report lag 16 at 244; currents at
     2.5 A with the cut on, 350; two pushes, port 20 at 1060 and REFUSED; without the slipper, nobody.
   - *Done:* `HandSide` gained `flicker`, `dither` and `tauVOf` (one stalled member perturbed once the wall has
     held 5 ticks, never the slipper); **P2-19d** (a PLAIN wall, each perturbation: nobody marked, no cut; the
     longest live FROZEN spell pinned, 0 / 0 / 9) and **P2-21d** (THE WIDER GAP pinned as it is: the three rows
     and the two-push REFUSED — the reviewer's proposed case). The wider gap is stated where the narrower one was:
     HA-175 (the index row's and the long form's claim (b) carry the exception; a third load-bearing belief with
     the reviewer's random-flicker counts, re-counted from their output file: 20/20 refused at p 0.5, 11/20 at
     0.3, 8/20 at 0.2, 0/20 at 0.1; the settle reads the push check's push for a `went ABSENT` line and leaves
     "REQUIRED before a match" to the team lead; the blast radius's "known gaps", each obvious fix's cost, and
     "fixing port 18's gear removes the trigger"), guide 12's symptom, `coupled_side_monitor.hpp`'s WHAT IT
     CANNOT SEE, `DriveProtectionState::frozen`.
   - *Not done, with reasons:* no guard change (it amends D (a); HA-175 says what each alternative costs); the
     2.5 A variant (350) and the random flicker are probed, not pinned (the same mechanism, and a pinned test
     has no randomness).
   - *Red:* P2-21d under XM42 (a strict majority) — the guard cannot change without this case saying so (the
     exec half of the table below).
2. **+ 7. (correctness minor, tests minor — one defect) A tick under 1 ms, or over 1000 s, passed the loop's check
   and then threw after the entry had commanded the robot — REAL; fixed at the root.**
   - *Re-checked:* the reviewed tree's `DriveProtection` needed a whole millisecond (`tickMsOf`: 0 outside
     1 ms..1e6 ms) while `runDriverControl`'s precondition — finite and > 0, unchanged since `9239538` — accepted
     both; the constructor runs after the claim release, the entry safe state, the toggles' `enter()` and the
     release mode.
   - *Fix* (the reviewer's third option): the tick kept in seconds (`tickSeconds_`); every duration printed from
     it (`durationText`: "250 ms", "1 s", "12.5 ms" at 0.5 ms, "50000 s" at 2000 s); the constructor's
     precondition is now exactly the loop's, which `teleop_loop.hpp`'s doc says ("together, every precondition of
     the DriveProtection the loop builds"). Rejected: refusing such ticks at entry — it narrows a contract the loop
     has had since G1 Part 2a, for the sake of a duration's wording.
   - *Tests:* P2-20c (0.5 ms and 2000 s — the latter added this session: both run, the cut on tick 1024, every
     line in true units). *Red:* T1 (the reviewed tree's 1 ms..1000 s precondition back), T4 (the 1000 s ceiling
     alone), T2 (a sub-second duration rounded to whole ms), T3 (never whole seconds: 11 cases).
3. **(correctness, minor) `warnOverCurrent`'s unbounded `%s` — REAL; fixed, and the matrix the reviewer suggested
   found one more.** Each `%s` bounded as `overCurrentWords()`'s (`%.16s%.12s … %.23s`). The third run compiled
   both scratch probes (three- and four-argument `perTick`, bindings, toggles, the switch off) with
   arm-none-eabi-g++ 13.2.1 and host g++ 13 at -O1/-O2/-O3/-Os: the host at -O3 flagged the mid-run REFUSED line
   (`REFUSED: %s -- …`, 122 fixed bytes) — now `%.68s`, the longest reason the mid-run rule gives ("motor counts
   make no sense (negative, or more answering than listed)", 68 characters, so 199 worst case; re-counted here).
   The matrix re-run on the final tree is in the gates entry below.
4. **(correctness, minor) HA-176 never tried a full-stick reversal or a launch from rest — REAL (a hardware belief
   no test can reach: the plant synthesizes no current).** Documented: HA-176's claim and index row (both named as
   ordinary driving believed to hold the limit for less than 250 ms), its Source (what each host test covers, and
   that none can show this), its Settle (after the push: a reversal at top speed and a launch from rest, read the
   peak line and any `CUT:` line; a cut is the team lead's before a match, N1), its blast radius (a healthy drive
   cut mid-manoeuvre); `drive_protection.hpp`'s over-current paragraph; guide 12's "One to rule out"; the
   worksheet's "What a CUT looks like". Station K's push check is not written (COMP-b's station): what it needs is
   in the merge notes below.
5. **(tests, major) Every protection driven forward only — REAL; fixed.** P2-4's reverse subcase (S1), **P2-5b**
   (F7: a death at rest, the robot backing away), P2-6d mirrored (F8), **P2-16c** (S2: a reverse push's peak line),
   P2-10b's negative rows at both floors. *Red:* S1, S2, F7, F8.
6. **(tests, major) No robot with a boot hole was driven through the protections — REAL; fixed.** **P2-23** (the
   reviewer's probe L): `absentDrivePorts = {18}`, a RIGHT fight — CHECK_NOTHROW, the DEGRADED entry line,
   `ABSENT (dead at boot)` at the cut, `dead at boot` in the summary, `state.dead` "18". *Red:* S21, S22, S23.
8. **(tests, minor) P2-20 reached 4 of the entry check's clauses — REAL; fixed.** P2-20's table: overCurrentAmps
   NaN/0/-1/inf, overCurrentTicks 0/-1, cutTicks 0, maxCutsLogged 0, frozenSuspectReads 0/25 — each refusal
   must NAME its field, and nothing is commanded; **P2-20b** calls `validProtectionConfig` directly (five groups,
   four, a null group, a null and a zero-capacity `why`). *Red:* S10-S15 — S15 by a SIGSEGV inside P2-20b itself:
   the dropped clause IS the null check, so the mutant dereferences null in the case written for it (the
   observable a null dereference has; M9's crash, by contrast, hit an unrelated case).
9. **(tests, minor) "The protections never read a runtime-absent member" unpinned — REAL; fixed:** P2-11's three
   line checks (probe G's). *Red:* S5, S6.
10. **(tests, minor) `DriveProtectionState` mostly unpinned — REAL; fixed.** P2-4b pins `frozen` on each of its 25
    FROZEN ticks (the marking tick included) and no other, `dead`, `groupCount`, `peakAmps`, the verdict before
    and after; P2-5 the same on the way; **P2-7b** (a third death latching DURING a cut: Refused, `cutTicksLeft`
    0, no re-arm). *Red:* S7, S8, S9b. **Beyond the finding:** the reviewers' S9 (`peakAmps` counting a FROZEN
    member) also survived their run, unlisted, and the peak line's stretch has the same rule with no mutation at
    all (S9c here); **P2-5c** pins both (a FROZEN member's current rising while it is FROZEN — what a LIVE member
    wrongly FROZEN at a wall can read; a dead port's held current cannot). *Red:* S9, S9c.
11. **(tests, minor) The X-drive's re-arm (S4) — REAL; fixed:** **P2-13b** (the stick held through the re-arm:
    cut #2 at 24 + 100 + 25), with P2-13's rig factored into `XDriveRig`. *Red:* S4.
12. **(tests, minor) `first()` and a vacuous brake-mode check — REAL; fixed** in `motor_group_test.cpp` (every
    member its own mode; a FIRST member marked: 33 °C not 80, Coast not Hold). *Red:* G1 (and the control G2).
13. **(tests, minor) `perTick`'s command during a cut (S16), `describe()` after a mark (R1) — REAL; fixed:**
    **P2-14b** (both `perTick` forms: the stick's command while the drive reads 0 V), **P2-17b** (`DEGRADED: absent
    18,13` after two marks). *Red:* S16, R1.
14. **(tests, minor) Other boundary survivors — REAL; fixed:** F1 (P2-10b: a report alternating 0.0/-0.0 is no
    repeat; a held NaN is one), F2 (P2-10b at exactly 4 vs 5 repeats), D2/D3 (the RIGHT side's clauses by verdict
    AND reason), S3 (**P2-4c**, through `DriveProtection`'s public calls, since the loop's pipeline never lands on
    exactly 1.0 V), S17 (**P2-2b**), S19 (**P2-2c**), S25 (**P2-24**, a hand-made four-group chassis). *Red:* all.
15. **(tests, minor) M9/M9b were undefined behaviour; the verdicts were judged on a filter — REAL (the log's
    claim).** The M9 row is corrected in place (dated): both forms read `groups_[0]` of an EMPTY span, so their
    "P2-13" was a crash, not a catch. This round's pass runs the faithful XM9f/XM9bf instead (red: P2-3 / P2-4)
    and EVERY mutation on the FULL suite; M17's first form and M38 are re-judged there (the table below).
16. **(docs, minor) `DriveVerdict`'s enumerators false for the mid-run verdict — REAL; fixed**
    (`drivetrain_degradation.hpp`: each worded for both rules; `docs/api` regenerated).
17. **(docs, minor) HA-175 claim (b) said "never" — REAL; fixed with 1** (the exception in claim (b), index row and
    long form; the gap worded one way everywhere — the slipping member "keeps turning while its last live mate
    stalls": HA-175, guide 12, `coupled_side_monitor.hpp`).
18. **(docs, minor) `fault.hpp` still said "told, not stopped" — REAL; fixed** (the driver-control qualifier;
    `docs/api` regenerated); the CamelCase spelling is in this round's prose review (below).
19. **(docs, minor) `teleop_loop.hpp`'s "on EVERY way out" — REAL; fixed** (it names the task-deletion path and
    points at `drive_protection.hpp`).
20. **(docs, minor) P2-19's BUG CAUGHT named a mutation that leaves it green — REAL; fixed** (re-pointed to M29,
    with D (a)'s reason and where the guard's own evidence is).
21. **(docs, minor) "LANDED 2026-09-29" in an uncommitted tree; the brief not annotated for the split — REAL.**
    The brief is annotated: a dated PART 2 SPLIT paragraph in its header, rows 2a and 2b in its parts table, and
    notes on D2's cost line and landmine 21 (the cost "`opcontrol()` unprotected until Part 2" no longer exists).
    The dates: an uncommitted change cannot know its landing day, so they are set at the commit — each is listed,
    by file and line, in the merge notes below (`fa915b0` set L9's status line the same way).
22. **(docs, minor) DIAG1 Part B rewrites the same reads — REAL** (`diag1b`'s `motor_group.hpp`: +215/-34, read
    loops on `m != nullptr`, and "a member that is ALWAYS unreadable is a dead port … decided at boot"). Another
    worktree's files: not edited; the merge notes say what the second to land must do.
23. **(docs, minor) COMP-t landed after the builder's last entry — REAL, and L9 Part A1 (API 2.4) has landed
    since.** The merge notes below are rewritten for `fa915b0`.
24. **(docs, minor) Unhedged hardware statements — REAL; fixed:** guide 12 (the current limit "believed … register
    entry HA-154, not measured"; "refuses again — at boot, if those ports are dead then, or once more than two are
    dead again"), the worksheet ("pushing a wall is expected to do that … HA-176, invented"), HA-176's blast
    radius ("is believed to sit at its current limit — the PROS default 2.5 A …; HA-154, unmeasured"); and the
    same kind of sentence in this round's own new reversal paragraph in `drive_protection.hpp`, hedged before it
    was ever reviewed.
25. **(docs, minor) Worksheet Station F row 5's "red = … frozen" — REAL; fixed**, checked in the code first:
    Teleop's row is red on `disagreeingMembers() > 0` (`teleop_program.cpp` :723, the monitor's PERSISTED count);
    a cut's zero command leaves the monitor un-evaluated, which zeroes every streak on the next tick
    (`coupled_side_monitor.hpp` :127-135); the panel recomputes every 20 ticks (`teleop_program.cpp` :314). Row 5
    now says red is a
    flash at most, and a FROZEN motor shows as amber `3/4 present` once marked.
26. **(docs, minor) Prose-review and log accuracy — REAL; fixed.** :677 and the M5 row corrected in place, dated;
    the build added ELEVEN P2-10b cases, not twelve (13 now, with this round's two); prose review 5 (below) ran
    the bare terms the earlier reviews narrowed, and the CamelCase spelling.
27. **(docs, minor) Guide 10's banner and cookbook 06 — REAL; fixed.** Guide 10: one paragraph — `runDriverControl`
    is in no Freeze Register row (re-read: F6 is the `Chassis` facade, F13 the controller seam, open), so it
    changes without a major bump and with a changelog entry, as the protections did; it names the entry by title,
    not by version, so the merge onto API 2.4 cannot stale it (the register's pointer was changed the same way).
    Cookbook 06: `panel` is named as the loop's `perTick`, its three-parameter signature written out, then the
    fourth.

### Prose review 5 — run, then read, before this entry *(09:19; `scratch/t4-2a/fix4/prose_review_5.txt`, 515 lines)*
- **Terms:** the three the review named — the bare words `cut`/`cuts`/`CUT` and `runtime`, and `MotorGroupDisagree`
  — over `README.md`, the live `docs/` (`docs/api` is generated from the headers, which are in the grep), the
  worksheet, `src/` and `include/shulib/`. (prose_review_3's narrower forms — "runtime absence", "runtime dead",
  "no cuts", "re-arm" — were chosen because the bare words hit every log-truncation and run-guard sentence; that
  choice was not logged then, and it missed `fault.hpp`.)
- **Every hit read. Nothing further was stale:** `MotorGroupDisagree` (8) — history (changelog :676, roadmap :1453,
  `version.hpp` :71), the raise in `health_monitor.hpp` :250 (its comment says nothing of the driver), `fault.hpp`
  (fixed, finding 18), `motor_group.hpp` :64 (qualified at 2a's :72-75). `runtime` (94) — PROS "the V5 runtime",
  the SD runtime loader, F4's "runtime interfaces", this change's own words, the old program's (true until Part 3).
  `cut` (370) — TermSink's 200-byte cut, the run guard's cut, the blackbox reader's truncation, the bench tester's
  permanent cut-out (Station D), the OLD "shulib Drive"'s cuts (Station E, true until Part 3), and this change's own
  text, each re-read against the code. The worksheet (37): Stations D and E (their own programs), F (this change's,
  re-read).

### For the coordinator — the merge notes, rewritten for `shulib-v2` at `fa915b0` *(2026-10-01; they replace the 21:40 entry's where they differ; nothing here is committed)*
*(SUPERSEDED where they differ by the verification round's notes for `9803928` (16:50, "T4-2a verification-round findings and their disposition"): base drift, the register's count 164 not 163, the push check's owner, DIAG1's loops re-pointed by THIS merge. Annotated 2026-10-01 19:55.)*
- **The decision this round raises (finding 1):** guard G4's gap at a wall beside the slipping port 18 is wider than
  decision D (a) weighed — REMAINING-WORK §11's new row "D, again", with the evidence in HA-175 and P2-21d. The
  tree implements D (a) as ruled and pins the gap; no guard change was made. A ruling that changes the guard
  re-pins P2-21d/P2-21c (both go red under any guard change — XM42 shows it) and rewords HA-175's known gaps.
- **Base drift:** this worktree is `9239538`; `shulib-v2` is now `fa915b0` (`ee2e399` plan, `a5bd83f` TANK1 Part B,
  `0f56301` COMP-t, `fa915b0` L9 Part A1). Files both sides changed: the changelog, the register, guides 11, 12 and
  14, the FAQ, the plan documents, the worksheet, the COMP brief, `teleop_program.cpp` and `xdrive_program.cpp`
  (COMP-t's edits there are comments only — "six builds" → "seven" — so the source pins hold), and the generated
  `docs/api`, `mkdocs.yml` and briefing (regenerate them after the merge, never merge them by hand).
- **The changelog:** `fa915b0` opened `## API 2.4` (L9's `reverseTo`/`faceAway`). This change bumps nothing, so its
  entry moves UNDER `## API 2.4`, ABOVE L9's entry, retitled "Added and changed since 2.4 (<date>) — the drive
  protections, inside the driver loop"; `version.hpp` stays 2.4. The guide-10 and register pointers to the entry
  name it by its title, not its version (this round), so they survive the move.
- **The register:** `shulib-v2` holds 161 entries (HA-167 from L9, HA-177/178 from COMP-t) and a "Held numbers"
  line (HA-162..HA-176, HA-181..HA-186; "a number that lands as a row may stay in its range"). At the merge: the
  index rows in number order (…167…, 175, 176, 177, 178…); this part's status sentence goes after L9's and ends
  "with them, 163" (replacing "the register then holds 160 entries. HA-162..HA-174, HA-177 and HA-178 are not in
  it"); `shulib-v2`'s "The rest of HA-162..HA-176 is not in it" becomes "HA-162..HA-174"; the briefing's count and
  next-free number regenerated (`briefing_status.py` reads the held line).
- **The builds:** COMP-t's autontest is the seventh. `src_build_gate.py self-test` and `check` on the merged tree
  run seven builds (this worktree ran six); the over-current words' and the REFUSED line's bounded `%s` were
  chosen against the probe matrix at every optimisation level, not against the six builds' TUs alone.
- **The dates (finding 21) — set each to the commit date at the commit, as `fa915b0` did for L9's status line:**
  the STATUS lines — `build-order.md` :632 (the Next block's "Part 2a … LANDED 2026-09-29"), :1484, :1490;
  `roadmap.md` :1749; `HANDOFF-remaining-work.md` :555-556; `REMAINING-WORK.md` :41. The "since" and section
  dates — the changelog heading (:19), the register (:84, :1190, :2098), the worksheet (:519) and the COMP brief's
  two struck-through annotations (:113, :662) — follow the chunk's date in `fa915b0`'s practice (it kept L9's
  "2026-09-29" in both the changelog and the register); keep or move them, consistently. The four code headers'
  "(chunk T4 Part 2, 2026-09-29)" (`motor_group.hpp` :102, `coupled_side_monitor.hpp` :273, `teleop_loop.hpp`
  :61, `drive_protection.hpp` :4) date when the code was written, as every chunk's header does.
- **DIAG1 Part B (the `diag1b` worktree) rewrites the same `MotorGroup` reads (finding 22).** Its
  `motor_group.hpp` (+215/-34, read 2026-10-01) loops on `m != nullptr` / `members_[i] != nullptr`. Its READ
  loops — `temperature()` (:363 there), `evaluateDisagreement()`'s samples (:410), `first()` (:477, which
  `brakeMode()` reads), `medianOf()` (:489, where `nonFiniteReads()` is counted) and `sumCurrent()` (:515) — must
  test `readable(i)`, and the samples must also honour the exclusion mask, as 2a's do; its COMMAND loops
  (`setVoltage` :305, `setBrakeMode` :318) and the constructor's count (:285) keep skipping boot holes only, by
  2a's design (a marked member is still commanded). Whichever of T4 2a and DIAG1 Part B lands SECOND does that
  re-pointing — P2-10 ("out of every read: the median, the mean current, the sum, the max temperature"), the
  `first()` subcases and P2-5's median check go red if one is missed. Two of its
  texts go false at T4 2a: its header's "a member that is ALWAYS unreadable is a dead port, and that is the absent
  member's business, decided at boot" (:53-55 there — runtime marks exist since T4 Part 2), and its brief's
  "Runtime member-absence … a later chunk, per motor_group.hpp ~:89-90" (`DIAG1-partB-median-nonfinite.md` :695;
  2a deleted that paragraph). Not edited here (another worktree's files).
- **COMP-b (the `comp-b` worktree, base `0f56301`) read the COMP brief WITHOUT 2a's replacement of B11's last
  bullet.** Its own log says so (`COMP-PROGRESS.md` :971-979 there). Sentences of it that go false the moment both
  are in: `docs/roadmap.md` :1724 there ("driver control has no drive protections until T4 Part 2") and
  `docs/guide/17-the-competition-program.md` :298 there ("no cut when a drive side's motors fight each other, and
  no rule for a …"). Its `driver_panel.hpp` :15 and `competition_screen.hpp` :556 ("T4 Part 2 adds its PROTECTION
  row") stay true of Part 2b. Correct them in whichever merge comes second.
- **Station K's push check** (Part 2b's, or COMP-b's with the station — your call, as before), now with this
  round's additions: on the ground, full stick into a wall for 3 s, let go, read the `drive: LEFT peak …` line and
  any `CUT:` / `at the cut:` lines; THEN a full-stick reversal at top speed and a full-stick launch from rest,
  each let go after it, read the same (HA-176's settle); and read the whole log for any `went ABSENT` line — a
  false DEAD at the wall, port 18 included (HA-175's settle; whether this is REQUIRED before a match is the team
  lead's, with "D, again").
- **Unchanged from the 21:40 notes:** the COMP brief's two sentences (:112-117 and B11's last bullet) struck
  through and replaced; worksheet Station F (Teleop's); the plan documents' T4 lines only; `sim/drive_plant.hpp`
  :221-222 ("five per side", R5a Part A's file now); the order lines left for you.

### 09:11:43-09:36:32 — the final-tree mutation pass: 86 mutations, 84 red, 2 survivors (both equivalent) *(`scratch/t4-2a/fix4/mutation-results.txt`, full output of each run under `fix4/runs/`)*
- **How:** `fix4/mut4.py` — each target asserted to occur exactly once in the FINAL tree (all 86 did: none skipped),
  the file backed up under `fix4/backup/`, the depend step and `make shulib_tests/fast` in this pass's own build dir
  (`fix4/mbuild`, configured from this worktree's `test/`; a run in which no object recompiled is VOID — none was),
  the FULL suite (1789 cases), the file restored, `cmp`-verified and its timestamp bumped. **After the pass:** every
  backup of each mutated file is byte-identical to every other and to the worktree's file (`md5sum`: 52 backups of
  `drive_protection.hpp`, 17 of `coupled_side_monitor.hpp`, 8 of `motor_group.hpp`, 5 of `teleop_loop.hpp`, 3 of
  `drivetrain_degradation.hpp`, 1 of `robot.hpp` — one checksum each); `git status` 39 entries (the 38 of the
  morning plus the T4 brief this round annotated).
- **The lists:** this round's T1-T4 and S9c; the reviewers' list (`review-testgate/mut.py`, minus S20 — its
  `tickMsOf` is gone, T1-T4 take its place); the executor's (`mutate.py`'s `M`, prefixed X, minus M9/M9b's
  undefined-behaviour forms — XM9f/XM9bf instead — plus M17's first form as XM17a).
| # | Mutation | Verdict (cases failed) | Red cases |
|---|---|---|---|
| T1 | the constructor's tick precondition back to a whole ms .. 1000 s (the reviewed tree's) | RED (1) | P2-20c |
| T2 | durationText rounds a sub-second duration to whole ms | RED (1) | P2-20c |
| T3 | durationText never prints whole seconds | RED (11) | P2-2 + P2-14, P2-3, P2-2b, P2-2c, P2-4, P2-12, P2-13, P2-13b, P2-16b, P2-17, P2-20c |
| T4 | the constructor refuses a tick over 1000 s only (the old ceiling alone) | RED (1) | P2-20c |
| S9c | the peak line's stretch counts a FROZEN member's current | RED (1) | P2-5c |
| S1 | over-current: the side's command floor judged SIGNED (no abs): a reverse push never counts | RED (1) | P2-4 |
| S2 | peak line: the stretch judged SIGNED (no abs): a reverse push never gets a peak line | RED (1) | P2-16c |
| S3 | over-current: the command floor inclusive (>= 1 V) | RED (1) | P2-4c |
| S4 | re-arm: the BARE (X-drive) over-current streaks not cleared | RED (1) | P2-13b |
| S5 | cut(): a runtime-ABSENT member READ at the cut (its ABSENT branch dropped) | RED (1) | P2-11 |
| S6 | onDisable(): a runtime-ABSENT member summarized as a live one | RED (1) | P2-11 |
| S7 | rejudge(): a refusal during a cut leaves cutTicksLeft_ running (not zeroed) | RED (1) | P2-7b |
| S8 | beforeDrive(): DriveProtectionState::frozen never filled | RED (3) | P2-4b, P2-5, P2-5c |
| S9 | readCurrents(): DriveProtectionState::peakAmps counts a FROZEN member's stale current | RED (1) | P2-5c |
| S9b | readCurrents(): DriveProtectionState::peakAmps never updated at all | RED (2) | P2-4b, P2-5c |
| S10 | entry check: overCurrentTicks 0 accepted (<= 0 -> < 0) | RED (1) | P2-20 |
| S11 | entry check: the maxCutsLogged clause dropped | RED (1) | P2-20 |
| S12 | entry check: the frozenSuspectReads <= 0 clause dropped (the detector then throws AFTER the entry's commands) | RED (1) | P2-20 |
| S13 | entry check: overCurrentAmps only finite (0 or negative accepted) | RED (1) | P2-20 |
| S14 | entry check: the kMaxProtectedGroups clause dropped | RED (1) | P2-20b |
| S15 | entry check: the null-group clause dropped | RED (1) | P2-20b |
| S17 | the fight: the LAST persisted group named, not the first (break dropped) | RED (1) | P2-2b |
| S19 | the whole-side fight words never chosen | RED (1) | P2-2c |
| S16 | perTick handed the ZERO command during a cut (doc: 'command is always the driver's request') | RED (1) | P2-14b |
| F1 | sameBits replaced by == (0.0 == -0.0 a repeat; a NaN never one) | RED (1) | P2-10b |
| F2 | the mates' fastest: a mate AT suspectReads repeats still counted (< -> <=) | RED (1) | P2-10b |
| F3 | the FROZEN streak not reset on a non-frozen tick (accumulates) | RED (2) | P2-21c, P2-21d |
| F4 | a LATCHED member still takes part in every rule (re-latched every tick) | RED (1) | P2-10b |
| F7 | the mates' fastest judged SIGNED (no magnitude): mates turning BACKWARD are no evidence | RED (2) | P2-10b, P2-5b |
| F8 | the own-speed branch SIGNED (no magnitude): a member frozen at a NEGATIVE speed not suspect by it | RED (2) | P2-10b, P2-6d |
| F9 | seen_ dropped from the repeat (a first read compared with a stale/zero previous) | RED (3) | P2-10b |
| G1 | first() returns a runtime-MARKED member (brakeMode() and temperature()'s seed read it) | RED (1) | P2-10 |
| G2 | CONTROL: temperature()'s loop includes marked members | RED (1) | P2-10 |
| S25 | census(): only groups 0 and 1 counted (a 3- or 4-group chassis's other groups ignored) | RED (1) | P2-24 |
| S21 | cut(): the boot-hole branch dropped -- member(i) on a boot hole THROWS at the first cut | RED (1) | P2-23 |
| S22 | beforeDrive(): a boot hole READ (member(i) on a hole THROWS on tick 0) | RED (1) | P2-23 |
| S23 | rejudge(): the dead list names runtime marks only (a boot hole left out -- the builder's own delta 2) | RED (1) | P2-23 |
| XM9f | M9 faithful: the fight judged on the first group only, without reading past an empty span | RED (2) | P2-3, P2-23 |
| XM9bf | M9b faithful: the over-current judged on the first group only, no UB | RED (1) | P2-4 |
| R1 | describe()/appendAbsent names BOOT holes only (a runtime mark not named) | RED (1) | P2-17b |
| D2 | runtime rule: more answering than listed on the RIGHT accepted | RED (1) | runtime degradation |
| D3 | runtime rule: a negative RIGHT count accepted (right.answering >= 0 and right.expected >= 0 dropped) | RED (1) | runtime degradation |
| XM1 | the FROZEN exclusion applied AFTER drive() | RED (2) | P2-6, P2-6b |
| XM2 | SUSPECT after persistTicks reads (the first proposal's length) | RED (19) | P2-10b, P2-6, P2-6b, P2-6c, P2-6d, P2-7 (N7 (c)), P2-7b, P2-19d, P2-21d, P2-22 |
| XM3 | the mates' fastest counting a member whose position repeats | RED (4) | P2-10b, P2-6c, P2-7b |
| XM4 | SUSPECT only through the mates (the own-speed branch dropped) | RED (8) | P2-10b, P2-6c, P2-6d, P2-7b |
| XM5 | FROZEN without the other-member guard | RED (6) | P2-10b, P2-21, P2-21b, P2-21d |
| XM6 | a FROZEN member's current counted | RED (1) | P2-4b |
| XM7 | over-current counted at or below the 1 V command floor | RED (3) | P2-4, P2-4c, P2-4b |
| XM8 | over-current judged on the side's totalCurrent() | RED (8) | P2-1, P2-4, P2-4c, P2-4b, P2-5c, P2-16, P2-16b, P2-20c |
| XM10 | the loop's config entry check dropped | RED (1) | P2-20 |
| XM11 | over-current >= instead of > | RED (1) | P2-4 |
| XM12 | the over-current window 24 (the comparison) | RED (6) | P2-4, P2-4c, P2-4b, P2-6c, P2-7b, P2-20c |
| XM13 | over-current streaks not reset at the re-arm | RED (1) | P2-4 |
| XM14 | cutTicks 99 | RED (15) | P2-2 + P2-14, P2-3, P2-2b, P2-2c, P2-4, P2-7b, P2-15, P2-12, P2-13, P2-13b, P2-16b, P2-17, P2-21c, P2-21d, P2-20c |
| XM15 | no 0 V write on the deciding tick | RED (4) | P2-2 + P2-14, P2-14b, P2-3, P2-15 |
| XM16 | the cut handing drive() the stick command | RED (6) | P2-2 + P2-14, P2-14b, P2-15, P2-16b, P2-21c, P2-21d |
| XM17 | an absent member's current counted (read, AND in the streak) | RED (2) | P2-4, P2-4b |
| XM18 | markAbsent leaves the member in the monitor | RED (9) | P2-4b, P2-5, P2-5b, P2-5c, P2-6, P2-6b, P2-6c, P2-6d, P2-10 |
| XM19 | markAbsent leaves it in the median | RED (2) | P2-9, P2-10 |
| XM20 | markAbsent stops commanding it | RED (4) | P2-7 (N7 (c)), P2-21c, P2-21d, P2-10 |
| XM21 | markAbsent allows the last member | RED (1) | P2-10 |
| XM22 | the REFUSE not re-applied at entry | RED (6) | P2-4, P2-7 (N7 (c)), P2-11, P2-21c, P2-23, P2-24 |
| XM23 | the per-side floor applied mid-run under N7 (c) | RED (6) | P2-7 (N7 (c)), P2-7b, P2-17, P2-21c, P2-21d, P2-24 |
| XM24 | the frozen-reading rule dropped | RED (15) | P2-4b, P2-5, P2-5b, P2-5c, P2-6, P2-6b, P2-6c, P2-6d, P2-7 (N7 (c)), P2-7b, P2-9, P2-21c, P2-19d, P2-21d, P2-22 |
| XM25 | the log bound off by one | RED (1) | P2-12 |
| XM26 | the fourth perTick argument never handed the real state | RED (22) | P2-2 + P2-14, P2-14b, P2-3, P2-2b, P2-2c, P2-4, P2-4b, P2-5, P2-5c, P2-6c, P2-7 (N7 (c)), P2-7b, P2-11, P2-15, P2-12, P2-13, P2-13b, P2-21, P2-21c, P2-21d, P2-20c, P2-23 |
| XM27 | overCurrentCuts ignored | RED (1) | P2-4 |
| XM28 | FROZEN guard 'some other member not SUSPECT' (item 4 as first written) | RED (4) | P2-10b, P2-21, P2-21b, P2-21d |
| XM29 | the repeat on position alone | RED (9) | P2-10b, P2-19, P2-19c, P2-21, P2-21b, P2-21c, P2-19d, P2-21d |
| XM30 | no mask clear in the destructor | RED (1) | P2-22 |
| XM31 | the peak line only on the disable path (no peak line at a stretch's end) | RED (4) | P2-1, P2-5c, P2-16b, P2-16c |
| XM32 | the config check after the claim release | RED (1) | P2-20 |
| XM33 | member(i) throwing for a runtime-absent member | RED (1) | P2-10 |
| XM34 | setMonitorExcluded keeps bits on holes and marks | RED (1) | P2-10 |
| XM35 | the runtime cap off by one (>= 2 refuses) | RED (7) | P2-7 (N7 (c)), P2-7b, P2-21c, P2-21d, P2-24, runtime degradation |
| XM36 | the peak line on an EQUAL peak (>= for >) | RED (1) | P2-1 |
| XM37 | no mask clear at enter() | RED (1) | P2-22b |
| XM38 | EQUIVALENT? no resetDisagreement() at the re-arm | **SURVIVED** |  |
| XM42 | a STRICT-majority guard (decision D (d)) in place of majority-or-tie | RED (6) | P2-10b, P2-7 (N7 (c)), P2-21c, P2-21d |
| XC1 | COMPILE: overCurrentTicks default 24 (the static_assert) | RED at build | the static_assert: "the 250 ms over-current window equals the fight monitor's window by choice (HA-1…" |
| XC2 | COMPILE: frozenSuspectReads default 25 (the static_assert) | RED at build | the static_assert: "the frozen-reading rule must decide before the fight monitor can cut on a dead m…" |
| XM39 | the latch at persistTicks - 1 | RED (13) | P2-10b, P2-4b, P2-5, P2-5b, P2-5c, P2-6, P2-6b, P2-6c, P2-6d, P2-7 (N7 (c)), P2-7b, P2-21c, P2-21d |
| XM40 | a non-present member's repeat not reset | RED (1) | P2-10b |
| XM41 | the whole-side census counting absent members as answering | RED (2) | P2-10b, P2-21d |
| XM17a | M17 first form: only the streak's isPresent dropped | **SURVIVED** |  |
- **The two survivors, each EQUIVALENT, with the reason checked against the final code:**
  - **XM38** (no `resetDisagreement()` at the re-arm): the monitor zeroes every member's streak on each tick its
    side is not evaluated (`coupled_side_monitor.hpp` :127-135 — a command at or below 1 V), and a cut is 100 such
    ticks, so the re-arm always finds the streaks at 0 — the brief's §3 says the same. Kept for the old program's
    stated intent.
  - **XM17a** (only the streak's `isPresent` dropped): `readCurrents()` stores 0.0 for a member that is not
    present (`drive_protection.hpp` :648-651) and `overCurrentAmps > 0` is an entry precondition — which is no
    longer an unpinned premise: S13 (that clause dropped) is red on P2-20's 0 A and -1 A rows. So `0.0 >
    overCurrentAmps` is false for every config that can run. XM17 (the read AND the streak) is red.
- **Two notes on reds:** S15 is red by a SIGSEGV inside P2-20b — the case written for it — because the dropped
  clause is the null check itself; T3 (11 cases) shows every whole-second duration in the suite is pinned in words.

### 09:37-09:46 — every gate on the final tree *(`scratch/t4-2a/fix4/gates.sh`; one log per gate under `fix4/gates/`; `date` 09:47 at this entry)*
- `docs/api` regenerated and the briefing regenerated; `cmake -S test -B build/test`; full build exit 0, no warning
  (the mutation pass's restores had bumped five headers' timestamps: they recompiled); suite **`1789 | 1789 passed
  | 0 failed | 3 skipped`, `assertions: 2658984 | 2658984 passed`** (22:10's 1771 / 2,656,801 plus the 18 cases of
  this review round: sixteen in `drive_protection_test.cpp` — P2-2b, P2-2c, P2-4c, P2-5b, P2-5c, P2-7b, P2-13b,
  P2-14b, P2-16c, P2-17b, P2-19d, P2-20b, P2-20c, P2-21d, P2-23, P2-24 — and two P2-10b); the briefing regenerated
  with the binary present (`1,789 cases / 2,658,984 assertions`, 193 headers, 160 registered in this worktree's
  base).
- `api_doc_tool.py` self-test `OK`; check-coverage `2999 public entities across 157 headers, all documented`;
  check-fresh exit 0; check-examples `467 quoted lines, all verbatim (5 source files); 3 listings of whole-run
  files, each one unbroken run`; check-removability `no public doc references docs/internal/`;
  `briefing_status.py check` `generated block current`; `doc_staleness_audit.py` self-test `OK (10 detector/filter
  cases)`, audit `clean (27 live docs …)`. CI greps: `library is PROS-free (all of include/shulib except
  hal/pros/)`; `layering holds: core is sim-free`. ARM header gate (RESUMING's command): 193 headers, `CLEAN`.
- **The snprintf matrix** (`fix4/arm/matrix.sh`: both probes, arm-none-eabi-g++ 13.2.1 with common.mk's own
  flags and host g++ 13.3, -O1/-O2/-O3/-Os, -Wall -Wextra -Wformat=2): **0 warnings in all 16 compiles**. The
  same script on the REVIEWED copy (`review-testgate/tree`) reproduces finding 3 first — `drive_protection.hpp`
  :853:88 at -O1/-O2/-O3 on both compilers, and the REFUSED line :614:47 at host -O3 (`fix4/arm/
  matrix-reviewed-tree.txt`) — so the matrix can see what it is for.
- **`src_build_gate.py self-test`: `OK (189 detector cases, each against a real build)`** (366 s); **`check`: `PASS
  (all 6 builds link …)`, every build `0 disallowed`** — six, this worktree's base (COMP-t's seventh is in the merge
  notes). After both: `git diff --stat -- src/ Makefile` shows `src/programs/teleop_program.cpp` only (12 lines) and
  no untracked file under `src/`, `Makefile` or `tools/` — no plant left. The literal diff check (`git diff -U0 --
  src/programs/teleop_program.cpp | grep '^[-+].*"'`): the same nine lines as at 21:40, `kDeadPortLine1/2` only.
- **Nothing committed.** `git status`: 36 modified, 3 untracked — the 21:40 change set (33 + 3), plus
  `diag/fault.hpp` and its `docs/api/fault.md` (finding 18) and the T4 brief (finding 21). HEAD `9239538`.

### What this round leaves open, for the coordinator *(the merge notes above carry the how)*
*(SUPERSEDED by the verification round's list at the end of this log (the register's count is 164; the ruling covers both wall gaps). Annotated 2026-10-01 19:55.)*
- **A ruling:** "D, again" (finding 1) — the wider gap at a wall beside port 18. The tree pins it as built.
- **At the merge:** the changelog under API 2.4, the register's count (163) and status sentence, the seven-build
  gate, the dates, DIAG1 Part B's read loops, COMP-b's two sentences, Station K's push check (now with the
  reversal, the launch and the `went ABSENT` read).
- **Nothing of Part 2a has run on a robot**; every number it uses is invented (HA-132, HA-175, HA-176).

## T4-2a verification-round findings and their disposition

- **Started:** Thu Oct 1 10:37 AM EDT 2026 (`date`). **Executor:** opus, the builder again, in the same worktree
  (`.claude/worktrees/t4-2a`, branch `t4-2a`, HEAD `9239538`). Do not commit.
- **Input:** a fresh verifier's three findings (one major — the wall gap without a slipping member; two minor — the
  Part 2b sequencing lines against `ce95325`/`de2a4f8`, and the tick precondition pinned on its accepting side only).
  Each is re-checked below before its disposition is written.
- **The tree at the start, checked before any edit:** `git status` — 39 entries (36 modified, 3 untracked), the
  09:37-09:46 entry's change set exactly; the only worktree file newer than that entry is this log (09:48:01). The
  verifier worked in its own copies (`scratch/t4-2a/verify/ptree`, `verify/tree`); nothing of theirs is in this
  worktree.

### RESUMED — Thu Oct 1 02:18 PM EDT 2026 (`date`), a new executor run (opus) in the same worktree
- **Why:** the coordinator paused this round at 10:37, right after the entry above (the team lead: "just go slower
  rather than stop"). Order: check the worktree for leftover test code, rebuild, then the three findings.
- **Leftovers, checked before any edit:** `git status` 39 entries, the same set; no worktree file newer than this
  log's 10:37:12 entry (`find -newermt`); `diff -rq` of `include/`, `src/` and `test/` against the verifier's
  `verify/ptree` (which the verifier found md5-equal to this worktree): identical — no mutation or probe was left
  in the tree. This round's scratch is `scratch/t4-2a/fix5/`.
- **`shulib-v2` moved again since the verifier ran:** `9803928` (H1 Part A1, 10:37:19) is on top of the verifier's
  `de2a4f8`. Finding 2's merge notes are written for `9803928`.
- **Rebuilt from source before any result was trusted** (14:28): the briefing regenerated; `cmake -S test -B
  build/test`; `cmake --build build/test --clean-first -j8` exit 0, no warning (`fix5/build0.log`); suite **`1789 |
  1789 passed | 0 failed | 3 skipped`, `assertions: 2658984`** — the 09:37 numbers exactly; the briefing
  regenerated with the binary present.

### 14:30-16:48 — the three findings, each re-checked, then fixed or answered
1. **(major) The wall gap without a slipping member — REAL, and the root is in the rule's evidence, not only in the
   words.**
   - *Reproduced, through the test file's own `World` and `HandSide` included unchanged* (`fix5/probes/
     wallgrid_test.cpp`, built against this worktree; output `wallgrid-out2.txt`): the verifier's numbers exactly
     — port 16's report at 30 ms beside mates at 80 / 100 / 120 / 150 / 200 ms: not marked (9 ticks), not marked
     (17), marked at 244, 244, 244; 10 vs 100 at 229; 20 vs 120 at 238; 50 vs 200 at 257 (and 1 cut); 80 vs 200 at
     276 (and 1 cut); 50 vs 150 24 ticks, one short; every lag equal (80, 150, 200 ms): 0 ticks. Beyond theirs:
     ports 17 and 20 in port 16's place mark at 244 too; HALF stick marks at 30 vs 150 (242) though not at 30 vs
     120 (20 ticks); every member drawing 2.5 A at the wall, the over-current cut ON, still marks at 244 (3
     over-current cuts as well); two fast members of four (`multifast-out.txt`) are a 2-2 tie and BOTH are marked
     at 244; three fast are 3-1, nobody marked, 2 fight cuts.
   - *The rule, measured:* the probe logs t0 (the member's first exact 0 report after the wall) and the last
     tick any LEFT mate's report reads above 1 rad/s. Every row marks exactly when that gap is 29 ticks or more,
     and marks on tick t0 + 29: 5 reads to SUSPECT, then 25 FROZEN (the 25th latches).
   - *Why — the root:* `FrozenReadingDetector`'s mates' fastest counts a mate whose `repeat` is below
     `suspectReads`, and under decision D (a) `repeat` needs position AND velocity unchanged. So a mate whose
     ENCODER has stopped while its lagging REPORT still decays has `repeat` 0 and is evidence that the train
     turns. The header's own reason for the filter ("a member whose own encoder has stopped is no evidence that
     the train turns", `coupled_side_monitor.hpp` :297-299 before this round) and the brief's (item 4; the FROZEN
     rule's first wording, "a side-mate whose position IS changing", brief :966) no longer described the code.
     The code is decision D (a) exactly as it was probed and ruled: `slip_probe.cpp` :78-86 uses the same
     `repeat` for both. So the gap is the ruled design's, not a slip in the build.
   - *Why the code is NOT changed here:* every change that closes it changes ruled behaviour, and D's guard is
     the team lead's (REMAINING-WORK §11, "D, again", which the verifier's own fix routes to him).
   - *The variant, probed so the ruling has evidence* (`fix5/mut5.py` V1, run as a mutation: a position-only
     repeat counter, and the mates' filter on it — a mate counts only while its encoder moved within the last 5
     reads; FULL suite on a fresh binary, then the wall grid against the mutated header, `fix5/runs/V1.log`,
     `V1-probe.txt`):
     - Only P2-19d and P2-19e go red — the pins of this gap. Every dead-port case (P2-5 to P2-7b, P2-9, P2-10b,
       P2-21 to P2-21d, P2-22 to P2-24) and P2-19c's coast-down are unchanged.
     - At a plain wall nobody is marked at any spread in the grid, any slot, either stick.
     - But the same push gets ONE 1 s fight CUT where it got none, from 30 vs 100 ms up (and 10 vs 80, 20 vs
       100), at either stick, with the motors under 2.4 A. The early-settling member, no longer FROZEN, stays in
       the fight monitor, under a quarter of its mates' reported speed for 25 ticks while they still read above
       1 rad/s; the FROZEN exclusion had hidden it. Worked through for 30 vs 100: under a quarter from about 6
       ticks after the wall, the mates above the floor to about 38 ticks after it.
     - With every motor at 2.5 A the over-current cuts come first (3, as with equal lags) and V1 adds none.
     - The slipping-member rows (P2-21d's three) are unchanged: the slipping member's own encoder is evidence
       under either rule.
     - It is the brief's first wording of the evidence ("a side-mate whose position IS changing") put back
       beside D (a)'s repeat, which stays for SUSPECT itself.
   - *Done:*
     - **P2-19d** keeps its title true ("marks NOBODY") and gains three pinned rows: 30 vs 100 ms (17 ticks),
       50 vs 150 ms (24 — one short of the mark, the latch's boundary at a plain wall) and half stick at 30 vs
       120 ms (20). Its BUG CAUGHT now names the mechanism: the mates' lagging REPORTS, not "nothing turns".
     - **P2-19e** (new, PINNED ON PURPOSE like P2-21c/d) — THE REPORT-LAG GAP: 30 vs 120 ms marks port 16 at 244
       (the verifier's row), 10 vs 100 at 229, half stick 30 vs 150 at 242; each with that member only, one
       `WARNING: port 16 (LEFT) went ABSENT at runtime` line, no cut, no REFUSE, `Degraded`, and the member still
       commanded after its mark (tick 400).
     - `OddMember` gained `fastTau`, `matesTau` and `stick` (defaults = the old behaviour, so P2-21d is
       unchanged). P2-21d's comment says the slipping member lowers the spread needed.
     - The words, everywhere the narrower claim stood (the verifier's list, re-found by grep):
       - `coupled_side_monitor.hpp`: THE MATES' FASTEST now says what it counts and that the gap comes from it;
         FROZEN carries "for as long as its members' reports settle alike"; WHAT IT CANNOT SEE leads with the
         any-wall case (the 29-tick rule, the model's numbers, a tie of two); "fixing port 18's gear ends the
         ties and the smaller spread, NOT the report-lag case"; `FrozenVerdict::suspectMask`'s doc.
       - `drive_protection.hpp`: `DriveProtectionState::frozen` (the exception is any wall, smaller differences
         beside a slipping mate) and THE FROZEN RULE's summary in ONE TICK.
       - HA-175: the index row (claim, source, settle "on any drive port"), claim (b) (the exception is a wall,
         any wall), the Source list (P2-19d's new rows, P2-19e), the third belief (load-bearing at EVERY wall;
         no V5 report lag measured, the model's 80 ms invented; the 29-tick rule), the settle (any port), and the
         known gaps (the any-wall case first, with its numbers; V1's measured cost beside the two older
         alternatives; "the trigger of the two slipping-member gaps … NOT the any-wall one").
       - Guide 12's "A motor reported dead that is not", guide 14's known case, guide 10's rule sentence (with a
         pointer to guide 12), the changelog's rule bullet (a "Known gap" sentence), the roadmap row,
         build-order's status line, HANDOFF's annotation, and REMAINING-WORK's "D, again" (both gaps, V1's
         measured trade, "fixing port 18's gear ends (2), not (1)"; the push check's log read for `went ABSENT`
         on ANY port).
       - The T4 brief's item 4: a dated note under the mates' rule saying it no longer describes the code, with
         the probe line and the pointer to "D, again".
       - The rule's "a moving side-mate" became "a side-mate whose readings still change" (HA-175, guide 10,
         the changelog, `drive_protection.hpp`, `suspectMask`), because at a wall the mate is not moving; its
         report is.
   - *Red, observed on a fresh binary:* V1 → P2-19d and P2-19e (above); the final-tree pass below re-runs the
     whole list against them.
2. **(minor) The Part 2b sequencing lines against `ce95325` / `de2a4f8` — REAL.** `git log` agrees with the
   verifier's times (`ce95325` 08:41:29, `9270ac2` 10:13:29, `de2a4f8` 10:14:00, and since then `9803928` 10:37:19).
   - *Fixed in the worktree, so a clean merge cannot land a contradiction* — each line re-worded to the plan of
     `ce95325` + `de2a4f8` (2b's items are Part 3's LATE step; Part 3's early step runs beside COMP-b), each true
     on this worktree's own and after the merge:
     - the T4 brief: the split paragraph (now "2b FOLDED INTO PART 3"; the order Part 1a → 2a → Part 3, early
       and late steps), row 2b ("FOLDED INTO PART 3 … with Part 3, as its late step"), D2's note and landmine 21's
       note;
     - REMAINING-WORK: §2's heading; row 9 rewritten as `shulib-v2`'s row 9 with 2a's landing in it; row 10
       RESTORED to the base text (Part 3's row is the coordinator's, and `shulib-v2` re-wrote it);
     - build-order: the status line (2b folded) and the Part 2 bullet, and the Next block's T4 clause;
     - HANDOFF :555-564 (folded; the verification's gap); roadmap :1749-1754 ("Parts 1b and 2b folded into Part
       3 (2b's items are its late step); Part 3 PLANNED"); the COMP brief :116 and B11's last bullet (Part 3's
       late step; and the push check now names the reversal, the launch and the `went ABSENT` read).
   - *Checked by a merge simulation onto `9803928`* (`fix5/merge-sim/`: `git merge-file`, base `9239538`, no
     commit): the T4 brief, HANDOFF, the roadmap and the COMP brief merge CLEAN, and a grep of each merged file
     for "Part 2b" finds only the fold's wording (the T4 brief :5-12 is `ce95325`/`de2a4f8`'s own, then this
     change's :58-62, :783, :3136; HANDOFF :585; the COMP brief :116, :676-678).
   - *The merge notes* are rewritten below for `9803928`: base drift, the register count 164, the push check's
     owner, DIAG1's loops re-pointed by THIS merge.
3. **(minor) The tick precondition pinned only on its accepting side — REAL.**
   - *Re-checked:* every direct construction in the suite used `Time{0.01}`, and `runDriverControl`'s identical
     check runs first, so no case could reach the class's own refusal; the verifier's E1 survived 1789/1789.
   - *Fixed:* **P2-20d** constructs `DriveProtection` directly with a tick of 0, -0, -0.01, NaN, +inf and -inf.
     Each must be a `PreconditionError` whose message is the precondition's own words ("DriveProtection: the tick
     period must be finite and > 0"); the refused construction must leave a group's exclusion mask as it found
     it (no object, so no destructor ran) and write no line. The smallest and the largest finite positive ticks
     (`denorm_min`, `max`) construct.
   - *Red, observed on a fresh binary, FULL suite* (`fix5/mutation-results.txt`, 15:22-15:23): **E1** (the check
     dropped, `true || …`): P2-20d only, all six rows; **E1b** (the finiteness clause dropped, `!(t <= 0)`):
     P2-20d, the NaN and +inf rows; **E1c** (`>= 0`): P2-20d, the zero and negative-zero rows.
- **One more, found while reading the reds:** a red row printed `row.name := 0x5c7a…` — doctest prints a captured
  `const char*` as an ADDRESS unless `DOCTEST_CONFIG_TREAT_CHAR_STAR_AS_STRING` is set (it is not, in this
  suite). Five captures in `drive_protection_test.cpp` (a file new in this change: P2-19d, P2-19e, P2-21d,
  P2-20d's rows, P2-20's `field`) now capture a `std::string`, so a red names its row.
- **The suite now:** **`1791 | 1791 passed | 0 failed | 3 skipped`, `assertions: 2659058`** (16:45) — 1789 plus
  P2-19e and P2-20d; +74 assertions: P2-19d's three rows × 6, P2-19e's three × 10, P2-20d's six × 4 and 2.

### For the coordinator — the merge notes, for `shulib-v2` at `9803928` *(16:50-; they REPLACE the 09:48 notes, "rewritten for `shulib-v2` at `fa915b0`", wherever they differ; nothing here is committed)*
- **The ruling this round widens (finding 1):** "D, again" now covers TWO gaps — (1) at ANY wall, a motor whose
  speed report settles well before its mates' is marked dead (P2-19e; no slipping gear needed), and (2) the
  slipping-member gaps of the review (P2-21d, P2-21c) — with the encoder-evidence variant's measured trade
  (REMAINING-WORK §11; HA-175's known gaps). The tree implements D (a) as ruled and pins both; no guard change.
  A ruling that changes the guard or its evidence re-pins P2-19d/P2-19e (V1 turns exactly those two red) and,
  for a guard change, P2-21c/P2-21d (XM42).
- **Base drift:** this worktree is `9239538`; `shulib-v2` is `9803928` — `ee2e399` (plan), `a5bd83f` (TANK1 B),
  `0f56301` (COMP-t), `fa915b0` (L9 A1), `ce95325` (plan: 2b folded into Part 3), `9270ac2` (DIAG1 B), `de2a4f8`
  (plan: Part 3 starts early), `9803928` (H1 A1). Twenty files changed on both sides. A `git merge-file`
  simulation (`scratch/t4-2a/fix5/merge-sim/`, base `9239538`, theirs `9803928`, after this round's edits):
  - **merge clean:** the FAQ, guides 12 and 14, the T4 brief, the COMP brief, the worksheet, HANDOFF, the
    roadmap, `mkdocs.yml` and `teleop_program.cpp`. Each was grepped after the merge for "Part 2b": only the
    fold's wording is left (the T4 brief's :5-12 are `ce95325`/`de2a4f8`'s own lines; this change's are :58-64,
    :783 and :3143 — re-measured at 19:20 after the 18:15 run's four brief notes; HANDOFF :585; the COMP brief
    :116 and :676-678);
  - **conflict, regenerate, never hand-merge:** `docs/api` (README 2, all-entities 1, motor_group 26), the
    briefing (1), and `mkdocs.yml` too, though it merged clean;
  - **conflict, hand-merge:** the changelog (1), guide 11 (1), the register (3), build-order (2),
    REMAINING-WORK (1), `motor_group.hpp` (4). How, each:
- **`motor_group.hpp` — T4 2a lands AFTER DIAG1 Part B, so THIS merge re-points DIAG1's read loops** (finding
  22's "whichever lands second"). Take DIAG1's bodies — `temperature()` (NaN seed, `any`, `isUsableReading`),
  `medianOf()` (the `nonFiniteReads_` count), `sumCurrent()` (its `CurrentSum`) — and replace each loop's
  `m != nullptr` test with T4's index loop on `readable(i)`, so a runtime-marked member is left out of every read
  as P2-10 demands ("out of every read: the median, the mean current, the sum, the max temperature"); `first()`
  keeps T4's `readable(i)` loop, its comment merged ("…the constructor's precondition and markAbsent()'s refusal
  of the last member guarantee there is one … brakeMode() read a DEVICE rather than a hole. (temperature() used
  it as its seed until DIAG1 Part B; it no longer does.)"). `evaluateDisagreement()` merges clean to T4's (fed
  only when `readable(i)` and not excluded — DIAG1 did not touch it), as does T4's deletion of the old "later
  chunk" paragraph. The command loops (`setVoltage`, `setBrakeMode`) and the constructor's count keep skipping
  boot holes only (a marked member is still commanded). **One DIAG1 sentence goes false at this merge:** its
  "A READING THAT IS NOT A NUMBER" bullet (:54-56 there) — "A member that is ALWAYS unreadable is a dead port, and
  that is the absent member's business, decided at boot." Suggested: "…decided at boot — and, since T4 Part 2,
  mid-run in the driver loop when its readings repeat bit for bit while its side turns (A4 register HA-175)". The
  merged suite is the check: P2-10, the `first()` subcases, P2-5's median check and DIAG1's
  `median_nonfinite_test.cpp` must all pass. DIAG1's brief line ("Runtime member-absence … a later chunk, per
  `motor_group.hpp` ~:89-90", `DIAG1-partB-median-nonfinite.md` :695) is history in a landed brief; leave it, or
  annotate it, your call.
- **The changelog:** `## API 2.4` now holds H1's "Added since 2.4 (2026-10-01)", DIAG1 Part B's "Changed since
  2.4 (2026-10-01)" and L9's entry. This change bumps nothing: its entry moves to the TOP of `## API 2.4`, above
  H1's, retitled "### Added and changed since 2.4 (<the commit's date>) — the drive protections, inside the
  driver loop"; `version.hpp` stays 2.4. Guide 10 and the register name it by its title, so they survive.
- **Guide 11 (1 hunk):** both rewrote the `L=` / `R=` paragraph's end. Take DIAG1's text, then append this
  change's parenthetical "(In driver control a member whose readings are held — position AND velocity repeating
  while its side turns — is caught by the library's drive protections instead: marked ABSENT after 250 ms with a
  `TEL` line naming the port, Chapter 12.)".
- **The register (3 hunks): 162 entries on `shulib-v2`, 164 with this change.** (a) The status sentences: keep
  `shulib-v2`'s (COMP-t's, L9's, DIAG1 Part B's) and put this change's AFTER DIAG1's, ending "with them, 164" —
  drop this change's "the register then holds 160 entries. HA-162..HA-174, HA-177 and HA-178 are not in it…",
  which `shulib-v2`'s "The rest of HA-162..HA-176 is not in it…" replaces and stays true of. (b) The index rows in
  number order: …161, 165, 167, 175, 176, 177, 178, 179…. (c) The long forms: keep both sides' sections (L9's
  HA-167 and DIAG1's HA-165, then this change's HA-175/176 section, or by date — consistently). The held line
  ("HA-162..HA-176, HA-181..HA-186; a number that lands as a row may stay in its range") needs no edit;
  regenerate the briefing (`briefing_status.py` reads it).
- **build-order (2 hunks):** (a) the Next block — `shulib-v2`'s (COMP-t … H1 Part A1 LANDED …; "now COMP-b …,
  then T4 Part 2 (the protections) and Part 3 …") with T4 2a's landing put in: "T4 Part 2a LANDED <date> (the
  drive protections inside `runDriverControl`, host- and build-gated, NOT run on a robot — [live log](chunks/
  T4-PROGRESS.md), "Part 2a"); now COMP-b …, then T4 Part 3 (with Part 2b's items, its late step — Part 3's
  early step can start now, beside COMP-b) …". This change's own Next-block clause (re-worded this round to the
  fold) is there to copy from. (b) The T4 status lines: keep BOTH — this change's "Status 2026-09-29" line (date
  it at the commit) and then `ce95325`'s "Status 2026-10-01" line, whose "Part 2a … is in its review round"
  becomes "landed" (the coordinator's line).
- **REMAINING-WORK (1 hunk, rows 9-10):** take THIS change's row 9 (it is `shulib-v2`'s row 9 with 2a's landing
  written in: "✅ landed <date> … Part 2b … FOLDED INTO PART 3 … Part 3's late step") and `shulib-v2`'s row 10
  (this change restored row 10 to the base text, so `shulib-v2`'s re-write wins; the hunk only groups them). §2's
  heading and §11's "D, again" merge clean.
- **HANDOFF :555's base sentence** ("**Part 2 is ~2–2½ days after COMP-b; Part 3 is ~3–3½ days after Part 2.**")
  predates the split and the fold, and is not this change's line; this change's annotation after it now says
  both. Re-word or leave it, your call.
- **The builds:** COMP-b has not landed, so the merged tree still has SEVEN builds (H1's merge: "PASS on all seven
  builds"); this worktree's gate ran six (its base predates COMP-t). Run `src_build_gate.py self-test` and
  `check` on the merged tree.
- **The dates — set each to the commit date at the commit** (unchanged list, re-found by grep this round):
  build-order :632, :1484, :1490; roadmap :1749; HANDOFF :555-556; REMAINING-WORK :41; the changelog heading
  (:19); the register (:84, :1190, :2098); the worksheet (:519); the COMP brief (:113, :662). The four code
  headers' "(chunk T4 Part 2, 2026-09-29)" (`motor_group.hpp` :102, `coupled_side_monitor.hpp` :273,
  `teleop_loop.hpp` :61, `drive_protection.hpp` :4) date when the code was written; keep them.
- **COMP-b (`comp-b` worktree, base `0f56301`, not landed):** unchanged from the 09:48 notes — its roadmap :1724
  ("driver control has no drive protections until T4 Part 2") and guide 17 :298 go false once both are in;
  whichever merge comes second corrects them. Its `driver_panel.hpp` :15 and `competition_screen.hpp` :556 ("T4
  Part 2 adds its PROTECTION row") stay true of Part 2b's items, now Part 3's late step — which re-reads COMP-b's
  files (item 8) anyway.
- **Station K's push check** — Part 3's late step (Part 2b's items since 2026-10-01) or COMP-b's with the station,
  your call, as before: on the ground, full stick into a wall for 3 s, let go, read the `drive: LEFT peak …` line
  and any `CUT:` / `at the cut:` lines; then a full-stick reversal at top speed and a full-stick launch from rest,
  each let go after it, read the same (HA-176's settle); and read the whole log for any `went ABSENT` line on ANY
  drive port — a false DEAD at the wall (HA-175's settle and both of its wall gaps). Whether the push is REQUIRED
  before a match for this reason too is "D, again"'s.
- **Unchanged from the earlier notes:** the COMP brief's two sentences (:112-117 and B11's last bullet) struck
  through and replaced (both now name Part 3's late step); worksheet Station F (Teleop's); the plan documents'
  T4 lines only; `sim/drive_plant.hpp` :221-222 ("five per side", R5a Part A's file now); the order lines left
  for you.

### RESUMED AGAIN — Thu Oct 1 06:15 PM EDT 2026 (`date`), a new executor run (opus) in the same worktree
- **Why:** the 14:18 run stopped after the merge notes above (the log's last write 16:52:50) and after writing
  `fix5/prose_review_6.txt` (16:53, a grep dump) — before reading it, before a final-tree mutation pass, before
  the gates. This run finishes those, re-verifies the three dispositions, and does not redo them. Scratch:
  `scratch/t4-2a/fix6/`.
- **Leftovers, checked before any edit:** `git status` 39 entries, the same set as at 10:37 and 14:18;
  `shulib-v2` still `9803928`, so the merge notes above stand as written (one line re-measured, below). No
  worktree file but this run's own first edits (below) was newer than this log (`find -newer`, 18:50). Every file the round's mutation tooling backed
  up (`fix5/backup/`: E1, E1b, E1c on `drive_protection.hpp`, V1 on `coupled_side_monitor.hpp`) diffed against
  the worktree: the only differences are the round's own COMMENT edits made after those runs (THE FROZEN
  RULE's summary and `DriveProtectionState::frozen`; THE MATES' FASTEST, FROZEN and WHAT IT CANNOT SEE) — no
  mutated line (no `posRepeat_`, no `true ||`, the tick precondition as `std::isfinite(...) && ... > 0.0`).
  `mut5.py` restores with `cmp` and exits on a failed restore; its results file shows every run ended.

### 18:15-19:46 — the stopped run's open items, finished *(`date` 19:46 at this entry)*
- **The three dispositions, re-verified against the tree (not redone):**
  - *Finding 1:* every place the verifier listed now states the any-wall case — `coupled_side_monitor.hpp`
    THE MATES' FASTEST / FROZEN / WHAT IT CANNOT SEE, `drive_protection.hpp` :20-23 and
    `DriveProtectionState::frozen`, HA-175's index row, claim (b), the third belief and the known gaps, guide
    12's "A motor reported dead that is not", guide 14, the roadmap row, REMAINING-WORK §11 "D, again"; P2-19e
    pins 30 vs 120 ms at tick 244 (and 10 vs 100 at 229, half stick 30 vs 150 at 242); P2-19d's comment names
    the mates' lagging REPORTS. No "nothing turns", "removes the trigger" or "removes both" is left
    (`fix5/prose_review_6.txt`). The final-tree pass below turns V1 red on P2-19d and P2-19e again.
  - *Finding 2:* the merge simulation re-run onto `9803928` after this run's edits (`fix6/merge-sim/`, `git
    merge-file`, base `9239538`, no commit): the same counts as at 16:48 — clean: the FAQ, guides 12 and 14,
    the T4 brief, the COMP brief, the worksheet, HANDOFF, the roadmap, `mkdocs.yml`, `teleop_program.cpp`;
    conflicts: `docs/api` README 2, all-entities 1, motor_group 26, the briefing 1, the changelog 1, guide 11 1,
    the register 3, build-order 2, REMAINING-WORK 1, `motor_group.hpp` 4. "Part 2b" in the merged brief: :5-12
    (`ce95325`/`de2a4f8`'s own), then this change's :58-64, :783, :3143 — the merge notes' one line re-measured
    to that; HANDOFF :585, the COMP brief :116 and :676-678, the roadmap :1773, unchanged.
  - *Finding 3:* P2-20d as logged; E1, E1b and E1c red again in the pass below, each on P2-20d alone.
- **Prose review 6, READ now** (the stopped run wrote `fix5/prose_review_6.txt` at 16:53 and logged nothing of
  it; 374 lines, 17 terms — P2-19d, P2-19e, P2-20c, P2-20d, P2-21d, "report lag", "settles sooner/well", "removes
  the trigger/both", "nothing turns/stays SUSPECT", "marks nobody", "known gap", "D, again", "moving (side-)mate",
  "read/settle alike", "tick period", "finite and > 0", the slipping terms). Every hit read. **Stale, and fixed
  this run:**
  1. `coupled_side_monitor.hpp` :349 — "All three are wider than decision D weighed": false of the middle bullet
     (a side already at two of four, P2-21c), which is the cost D ACCEPTED; the round's rewrite of WHAT IT
     CANNOT SEE introduced it (the 15:22 backup says only the full-side case was wider). Now "The first and the
     third are wider than decision D weighed, and the team lead's to rule on (the second is the cost D
     accepted)". A `//` comment, but it moved the declarations below it one line, so `docs/api`'s line anchors
     were regenerated (the doc gate caught them: the first build at 18:51 was red at `shulib_doc_gates`).
  2. build-order :1484 (this change's status line) — "while it or a moving mate says the train turns": the
     round re-worded the rule to "a side-mate whose readings still change" everywhere else and missed this one
     (at a wall the mate's REPORT moves, not the mate). Re-worded the same.
  3. HA-175's Source — "P2-19 (a live train at a wall marks nobody)": true only of P2-19's case, every report
     lagging alike. Now "(a live train at a wall, every report lagging alike, marks nobody)"; P2-19's title
     says "its reports lagging ALIKE" (no document quotes the title; grep).
  4. The T4 brief's unqualified "a live train at a wall marks nobody / is never a death", in four places a
     Part 3 executor reads: N2 (a)'s FROZEN line (:972), its Cost bullet (:1010), §3's P2-19 (:2542) and §5
     landmine 28 (:3150). Each gains a dated note (only while the side's reports settle alike; P2-19e; HA-175's
     known gaps; "D, again"), the body not rewritten — as the round did under item 4 (:1731).
  - *True as they stand:* the brief's history and plans (:362, :515, :807, :955, :2436-2438, :2497 — mates that
    really move, or the re-check's record of its day; :2936/:2943 the register rows it proposed, superseded by
    the register); P2-5's "a moving mate" (`drive_protection_test.cpp` :814, :843 — the mates really move);
    `coupled_side_monitor.hpp` :353 and its test :425 (a coast-down's mates move); every "tick period" and
    "finite and > 0" hit (the library's other preconditions, `loop_monitor`'s budget, the X-drive's comment);
    every slipping-term hit outside this change (R3d's calibrator, HA-130/137, guide 11's cause column); the
    other chunks' "known gap"s.
- **Prose review 7 — this run's own words** (`fix6/prose_review_7.txt`, 60 lines: "All three are wider",
  "first and the third", "lagging/decay alike", "never/not a death", "moving (side-)mate", "marks nobody",
  "landmine 28", "item 4's note", "D, again"): each hit read; every pointer resolves (landmine 28 is §5's 28;
  the item-4 note is §1 Part 2 item 4's, :1731; "D, again" is REMAINING-WORK §11, :175). Nothing further.
- **Rebuilt from source** (18:51-18:53): the briefing regenerated; `cmake -S test -B build/test`; `cmake --build
  build/test --clean-first -j8` — red at the doc gate (item 1's anchors), `api_doc_tool.py generate`, the
  briefing again, the clean build exit 0, no warning (`fix6/build1.log`); suite **`1791 | 1791 passed | 0
  failed | 3 skipped`, `assertions: 2659058`** (16:45's numbers; a title and comments changed, no check).
- **The final-tree mutation pass, 18:57:17-19:18:37** (`fix6/mut6.py`; `fix6/mutation-results.txt`; each run's
  output under `fix6/runs/`): **90** — fix4's 86 (its own T1-T4 and S9c, the reviewers' list minus S20, the
  executor's prefixed X minus M9/M9b, plus XM9f/XM9bf and XM17a) and fix5's V1, E1, E1b, E1c — the same
  procedure (target asserted once, backup under `fix6/backup/`, the depend step and `make shulib_tests/fast` in
  `fix6/mbuild` (configured from this worktree's `test/`, its baseline 1791/1791), VOID when nothing recompiled
  (none was), the FULL suite, restore + `cmp` + timestamp bump). A `--check` mode ran first and again after:
  every target present exactly once, no mutated text in the tree. **86 RED, 2 red at the build (XC1, XC2 — the
  static_asserts), 2 survivors: XM38 and XM17a** — the 09:11 pass's two, both equivalent as that entry says.
  Compared case by case with the 09:11 and 15:22 passes: **no mutation lost a red case**; T1 and T4 now also
  fail P2-20d; XM1 P2-19d; XM2, XM24, XM26 and XM29 P2-19e; XM39 both. **After the pass:** the 90 backups
  (55 of `drive_protection.hpp`, 18 of `coupled_side_monitor.hpp`, 8 of `motor_group.hpp`, 5 of
  `teleop_loop.hpp`, 3 of `drivetrain_degradation.hpp`, 1 of `robot.hpp`) each md5-equal to the worktree's file;
  `git status` 39 entries.
- **Every gate on the final tree, 19:28:59-19:45:51** (`fix6/gates.sh`, one log per gate under `fix6/gates/`,
  summary `fix6/gates-summary.txt`; a first start at 19:27 was stopped by hand during its build, before any gate
  that plants, because its 10-minute limit was too short for the src gate's self-test): `api_doc_tool.py
  generate` (no change); the briefing; configure; build exit 0; **suite 1791 | 1791 passed, 2659058
  assertions**; the briefing again with the binary present; api_doc_tool self-test OK, check-coverage (2999
  entities, 157 headers, all documented), check-fresh, check-examples (467 quoted lines verbatim, 5 source
  files; 3 whole-run listings unbroken), check-removability; `briefing_status.py check` (generated block
  current); `doc_staleness_audit.py` self-test OK and audit clean (27 live docs); the CI greps (PROS-free,
  sim-free); the ARM header gate over every `include/shulib` header (CLEAN); the snprintf matrix (16 builds, 0
  warnings); `src_build_gate.py self-test` **OK, 189 detector cases** (842 s — the machine shared with other
  builds) and `check` **PASS, all 6 builds**; `src/` + Makefile: only `teleop_program.cpp`, its two boot
  strings, as before. No plant left: `git status` 39 entries, `git diff --stat -- src/ Makefile tools/` the
  one file. After this entry was written, the gates that read documents ran again (19:52): check-fresh,
  check-examples, check-removability, check-coverage, `briefing_status.py check`, the staleness audit — each
  exit 0.
- **This run's edits, all in the worktree, nothing committed:** `coupled_side_monitor.hpp` (the comment, item
  1), `docs/api/coupled_side_monitor.md` (regenerated anchors), `test/drive_protection_test.cpp` (P2-19's
  title), `docs/hardware-assumptions.md` (HA-175's Source sentence), `docs/internal/build-order.md` (:1484),
  the T4 brief (four dated notes), this log (this entry, the merge notes' one line, and a SUPERSEDED note under
  the 09:48 merge notes and the 09:48 open list, whose "163" and `fa915b0` the verifier flagged); the briefing
  regenerated.

### What the verification round leaves open, for the coordinator *(the 16:50 merge notes carry the how)*
- **A ruling:** "D, again", now covering BOTH wall gaps — at ANY wall (P2-19e, the verification's) and beside
  port 18 (P2-21d, the review's) — with the encoder-evidence variant's measured trade (REMAINING-WORK §11,
  HA-175's known gaps). The tree implements D (a) as ruled and pins both; no guard change.
- **At the merge:** as the 16:50 notes say (base `9803928`; the register's count 164; DIAG1 Part B's read loops
  re-pointed by THIS merge; the changelog under API 2.4; seven builds; the dates; the push check's owner).
- **Nothing of Part 2a has run on a robot**; every number it uses is invented (HA-132, HA-175, HA-176).

## The coordinator's merge — Part 2a (2026-10-01, 20:15–21:00 EDT)

- **The run:** the build (2026-09-29), three reviews, the fix round, a fresh verifier and its fix pass (fix2) —
  cut by usage-limit stops and a session end, and resumed each time with continuation notes. Transport commit
  `621379d` in the worktree `t4-2a`, squash-merged onto `9803928`.
- **One CODE conflict, resolved by hand:** `include/shulib/hal/motor_group.hpp`, where DIAG1 Part B (a reading that
  is not a number is left out and counted) and this part (a member marked ABSENT at runtime is not read) both
  rewrote the group's read loops. Every read loop — `temperature()`, `medianOf()`, `sumCurrent()` — now walks
  `readable(i)` (neither a boot hole nor runtime-ABSENT) AND keeps DIAG1's finite-only test and its
  `nonFiniteReads()` count; the command loops (`setVoltage`, `setBrakeMode`) still skip only boot holes, as both
  sides intended. Document conflicts: the changelog (this part's section moved under `## API 2.4`, "Added and
  changed since 2.4 (2026-10-01)"), the register (status 7 of 164; this part's sentence after DIAG1's; the rows in
  numeric order — 165, 167, 175, 176, 177, 178; its section after COMP-t's and L9's), guide 11 (DIAG1's paragraph
  kept, this part's driver-control sentence added after it), build-order (the Next line; the T4 status lines, this
  part's re-dated to its landing, 2026-10-01), REMAINING-WORK (row 9 landed 2026-10-01; row 10 kept as ruled on
  2026-10-01). `docs/api` and the briefing regenerated.
- **"D, again" ANSWERED the same evening** (the team lead: "Encoder check"): a side-mate counts as evidence that
  the train turns only while its ENCODER moved in the last 5 reads. It is built in Part 3's early step; until then
  guard G4 is as this part built it (REMAINING-WORK §11).
- **Gates on the merged tree:** suite **1,958 | 1,958**, 3 skipped; `src_build_gate.py` self-test OK (198) and check
  PASS on all seven builds; the ARM header gate; both CI greps; `check-fresh`, the briefing (self-test and check),
  removability and the staleness audit — all clean; no plant (`.claude/coordinator/logs/t42a_merge_verify.log`).
- **Eight coordinator mutations, each on a fresh binary, all RED** (`.claude/coordinator/muts_t4_2a.json`):
  `markAbsent` dropping the last member (1 case); `readable()` ignoring runtime marks (15); guard G4 needing a strict
  majority (6); exactly two dead stopping the drive mid-run (7); the over-current count including a frozen member
  (1); a cut not stopping the drive (8); and two aimed at the merge itself — `temperature()` (1) and the current sum
  (1) reading a member marked ABSENT.

## Part 3 — the new "shulib Drive"; "shulib Teleop" retired (with Part 2b's items folded in) — THE EARLY STEP

- **Started:** Thu Oct  1 08:57:57 PM EDT 2026 (`date`).
- **Tree at start:** `7daf809` (T4 Part 2a merged), clean, in the git worktree `.claude/worktrees/t4-3` (branch
  `t4-3`) — NOT the main working tree.
- **Executor model:** opus.
- **Go:** the coordinator's workflow run, on the team lead's go for Parts 2 and 3 (2026-09-28, "i want to do all
  this"; his A–D answers of 2026-09-28), his rulings of 2026-10-01 (2b folded into Part 3 — `ce95325`; Part 3
  started early, in two steps — `de2a4f8`), and his answer to "D, again" (2026-10-01, "Encoder check"; the
  coordinator's merge entry above). Do not commit; no `pros`; no `/dev/ttyACM*`. Probes, backups and scratch under
  `.claude/coordinator/scratch/t4-3/` only.
- **THIS IS THE EARLY STEP.** It builds only what needs nothing of COMP-b's files (COMP-b is in flight in its own
  worktree); each item is logged below as built early or left for the LATE step, with why. The late step runs
  after the coordinator merges COMP-b into main and tells this worktree.
- **First, before the program work: the "D, again" encoder check** (RULED 2026-10-01): in
  `FrozenReadingDetector`, a side-mate counts as evidence that the train turns ONLY while its ENCODER (position)
  moved in the last 5 reads, never on its speed report alone.

### 20:58–21:02 — required reading, and the baseline *(`date` 21:01:53 at the baseline)*
- Read RESUMING.md, the T4 brief WHOLE (the header notes — the 2b fold and the early start —, the 2026-09-28
  re-check amendments, § Decisions, §0–§8), this log's Parts 1a and 2a (the review rounds, the verification and the
  coordinator's merge entry, "D, again" ANSWERED), REMAINING-WORK §11, the fix5 probes of the verification
  (`scratch/t4-2a/fix5/mut5.py` V1, `probes/wallgrid_test.cpp`, `runs/V1-probe.txt` — the encoder-evidence variant
  the ruling chose), and COMP-b's worktree READ-ONLY (`src/common/driver_panel.hpp`, `screen_paint.hpp`,
  `screen_touch.hpp`, `src/programs/competition_screen.hpp`; its `git status`).
- **Baseline, host** (the briefing regenerated, `cmake -S test -B build/test`, full build, at `7daf809`):
  `[doctest] test cases: 1958 | 1958 passed | 0 failed | 3 skipped`, `assertions: 2709569 | 2709569 passed`.

### 21:02–21:25 — "D, again": THE ENCODER CHECK (RULED 2026-10-01), built first *(`date` 21:25 at this entry)*
- **`include/shulib/teleop/coupled_side_monitor.hpp`:** `FrozenReadingDetector` keeps a second counter,
  `encoderRepeat_` (the consecutive reads whose POSITION alone repeated), and the mates' fastest counts a mate only
  while `encoderRepeat_[j] < suspectReads` — its encoder moved within the last 5 reads. SUSPECT's own count
  (`repeat_`, position AND velocity, decision D (a)) is unchanged; so are the guard, the latch and every
  declaration (no public API changes: the counter is private; `reset()` and a member not present clear it). It is
  the verification's V1 variant exactly (`scratch/t4-2a/fix5/mut5.py`). Prose: THE MATES' FASTEST, FROZEN, a new
  WHY THE ENCODER CHECK paragraph (the ruling, the gap it closes, its measured cost), and WHAT IT CANNOT SEE
  rewritten (the any-wall case closed; port 18's two gaps kept until its gear is fixed); `FrozenVerdict::suspectMask`.
  `drive_protection.hpp`: ONE TICK's FROZEN sentence and `DriveProtectionState::frozen`.
- **A probe first** (`scratch/t4-3/probes/wallgrid3_test.cpp` — the verification's wall grid, through the test
  file's own World/HandSide, under the new rule; output `wallgrid3-out.txt`): every PLAIN-wall row marks nobody and
  FREEZES nobody; every spread from 30 ms beside 100 ms (or 10 beside 80) gets exactly ONE FIGHT cut (ticks 224-240,
  per row); 30 beside 80, a flicker, a dither and every lag alike cut nothing; at 2.3 A each the same one fight cut;
  at 2.45 A and 2.5 A each the over-current cuts come first (three, at 224, 349, 474) and no fight cut; the three
  slipping-member rows (P2-21d's) are unchanged. The same as V1's run of 2026-10-01 row for row.
- **Tests, re-pinned and added (logged one-for-one):**
  - `test/drive_protection_test.cpp` **P2-19d** re-pinned: was "marks NOBODY … each longest live FROZEN spell pinned
    (0, 0, 9, 17, 24, 20)"; now three rows (flicker, dither, 30 beside 80) that mark nobody, FREEZE nobody (0) and cut
    nothing. Its 30/100, 50/150 and half-stick 30/120 rows moved to P2-19e (each now one fight cut).
  - **P2-19e** re-pinned: was "THE REPORT-LAG GAP … marked ABSENT" (244, 229, 242); now "THE REPORT-LAG SPREAD,
    CLOSED by the encoder check" over the verification's whole grid (17 spread rows incl. ports 17 and 20 and half
    stick, plus three every-lag-alike rows): nobody marked or FROZEN, no `went ABSENT`, mode `Driving`, and per row
    either exactly ONE 1 s FIGHT cut at its pinned tick (0 V on the deciding tick and the 100 after, driving again on
    the 101st, the cut line naming the early-settling port, no over-current line) or none; plus the 2.3 A row (one
    fight cut) and four over-the-limit rows (2.45 A, 2.5 A ×2, every lag alike at 2.5 A): cuts at exactly 224, 349,
    474, all over-current, no FIGHT. `OddMember` gained `wallAmps`; `checkOneFightCut()` is new.
  - P2-21d's comment re-pointed (the no-slip case now marks nobody); P2-21c and P2-21d unchanged and green (port 18's
    gaps stay, as ruled).
  - `test/coupled_side_monitor_test.cpp`, two new P2-10b cases: THE ENCODER CHECK at its boundary (a mate whose
    encoder held 4 reads is still evidence, at 5 it is not, while its report keeps changing above 1 rad/s), and at a
    wall whose reports settle unevenly (nobody suspect, FROZEN or latched over 60 reads).
- **Suite:** `1960 | 1960 passed`, `assertions: 2711704` (+2 cases).
- **Mutations** (`scratch/t4-3/mut/mutate.py`: back up, one exact replacement asserted once, the `shulib_tests`
  target rebuilt in the scratch build dir `mut/mbuild` — VOID when nothing recompiled —, the FULL suite, restore,
  `cmp`, timestamp bump; results `mut/mutation-results.txt`, each run under `mut/runs/`). *The first run's red LISTS
  were over-counted by the runner (it took every `TEST CASE:` line, and doctest prints one for a case that only
  logged); the summaries were right; the lists were re-parsed from the saved runs with the fixed parser, which reads
  only blocks holding an ERROR:*

  | # | Mutation | Red (FULL suite, fresh binary) |
  |---|---|---|
  | EC1 | the encoder check reverted (a mate is evidence while ANY reading changes) | 4 cases: both new P2-10b cases, P2-19d, P2-19e |
  | EC2 | the encoder window off by one (`<=`) | 2: the new boundary case, and the old "EXACTLY suspectReads" case |
  | EC3 | the encoder window one short (`< suspectReads - 1`) | 1: the new boundary case |

  *Stated EQUIVALENT, not counted:* clearing `encoderRepeat_` for a member not present (and in `reset()`) — a
  member's next read after either has `seen_` false, which already makes its encoder repeat 0. A mates' filter on
  BOTH counters — `repeat_ <= encoderRepeat_` always (a repeat needs the position to repeat), so it is the same rule.
- **Documents for the ruling** (the prose review below): HA-175's index row and long form (claim (b), the source,
  the third belief, the settle — the push check's log read for `CUT: FIGHT` too —, the known gaps: the any-wall
  case CLOSED with its cost, the other changes weighed), guide 12 (the FIGHT cut's "two to rule out", and "a motor
  reported dead that is not"), guide 10 (the rule's sentence), guide 14 (the known case), the roadmap's T4 row,
  HANDOFF's T4 annotation. The changelog's entry is written with Part 3's library pieces (below).

### 21:25–22:58 — item 4′: the SD copy's library pieces *(`date` 22:58 at this entry)*
- **`include/shulib/diag/log_copy_sink.hpp` (NEW, `diag::LogCopySink`)**, under decision A (a): `log()` and
  `summarize()` to the primary, then the copy; `wantsRecord()` and `emit()` to the PRIMARY only, as a pair. PROS-free,
  allocation-free, two borrowed references; `primary()` / `copy()` read them back.
- **`include/shulib/hal/pros/device_provider.hpp`:** a NEW constructor `ProsDeviceProvider(const char*
  serialCopyFile, ProsFileMode copyMode = Truncate, const char* copyMountRoot = "/usd/")` — a null name is the
  default provider exactly; given a BARE name the provider owns `ProsBlockSink` → `hal::BlockCharSink` (COMP-t's,
  reused, no new file) → a second `TermSink` over the same clock → `LogCopySink` over the serial TermSink and that
  one, which `telemetry()` returns. `serialCopyOpen()`, `serialCopyPath()`, `serialCopyDroppedLines()`,
  `serialCopyAppendOpen()` read the copy's state. The default constructor and the registry constructor are
  untouched. Header: a new section AN OPTIONAL COPY OF THE LOG; NO HARDWARE CLAIM's program list re-pointed (it named
  "shulib Teleop").
- **Deltas from the brief (item 4′), each with its reason:**
  1. **The provider's argument is a NEW constructor, not a defaulted argument on the old one** — `ProsDeviceProvider()`
     stays byte-for-byte today's, which is the brief's "default nullptr: today's provider" with nothing to compare.
  2. **A third optional argument, `copyMountRoot`** (default "/usd/"): not in the brief. Without it the copy cannot
     be host-tested on the SAME join+open+write path (`ProsBlockSink`'s own mountRoot exists for exactly that).
  3. **Four read-only accessors** (`serialCopyOpen/Path/DroppedLines/AppendOpen`): not in the brief's piece list
     (§5 calls N5 (a)'s list exhaustive). ProsBlockSink's header makes the composition root say a refusing card ONCE
     (the T5 rule); with the file owned by the provider, the program can only say it through these. Additive.
- **Tests:** `test/log_copy_sink_test.cpp` (NEW, LC-1..LC-6 — LC-6 drives 200 REAL `Chassis::drive()` ticks
  through `motion_rig::ChassisRig` with the copy sink as the context's telemetry: 200+ record lines on the primary,
  none on the copy) and `test/pros_device_provider_test.cpp` 1n-1q through the shim and a scratch "card" (the copy
  holds exactly the serial LOG lines and the summary and no record; no card: every line refused and counted, nothing
  thrown; Append keeps an earlier boot's bytes and creates a missing file; a non-bare name is a precondition).
- **Suite:** `1970 | 1970 passed`, `assertions: 2711767`.
- **Mutations** (the same runner; the scratch dir rebuilt green first — `1970 | 1970` — after the new test files,
  the stale-binary trap; the first run at 22:01 was VOID: the worktree's briefing was stale after the new files and
  every mutation build stopped at the doc gate; then LC2/PV1 removed lines (the doc gate's anchors) and PV2 left a
  parameter unused (-Werror) — each re-written line-for-line and re-run; all logged in the results file):

  | # | Mutation | Red |
  |---|---|---|
  | LC1 | `emit()` forwarded to the copy (the brief's named mutation) | 5: LC-1, LC-5, LC-6, 1n, 1o |
  | LC2 | `summarize()` not forwarded to the copy | 5: LC-2, LC-4, LC-5, 1n, 1o |
  | LC3 | `wantsRecord()` answered from the copy | 1: LC-3 |
  | LC4 | the copy written before the primary | 1: LC-4 |
  | PV1 | the provider's `telemetry()` ignores the copy | 3: 1n, 1o, 1p |
  | PV2 | the copy's mode ignored (always Truncate) | 1: 1p |
  | PV3 | the copy's drop count never reported | 1: 1o |

### RESUMED — Sun Oct 4 08:06 AM EDT 2026 (`date`), a new executor run (opus) in the same worktree
- **Why:** the previous run of this step was STOPPED partway. Its last entry here is 22:58 (2026-10-01, item 4′'s
  library pieces); the tree shows it went on WITHOUT logging: files modified 2026-10-02 01:17–02:10 (the table,
  `teleop_claws.hpp`, the driver file, a new `src/programs/drive_panel.hpp`, `drive_program.{hpp,cpp}`, `main.cpp`,
  `build_selection.hpp`, the Makefile, the gate, `ci.yml`, Calibrate's and the X-drive's literals,
  `teleop_program.{hpp,cpp}` deleted — `git rm`, staged) and again 2026-10-02 16:33–16:37 (three test files:
  `mechanism_table_test.cpp`, `boot_source_test.cpp`, `robot_two_driver_test.cpp`) — a second stopped run, also
  unlogged. Its scratch: `scratch/t4-3/edits/*.py` (the edit scripts), `base/build5.log` (the doc-staleness gate red
  on twelve `teleop_program.cpp` paths), `base/fast1.log` (290 compile errors in the test files the table cleanup
  broke), `base/arm-drive{1,2}.log` (two `PROGRAM=drive`-shaped ARM builds that LINKED).
- **Checked first:** no mutation left applied — each of the ten backups under `scratch/t4-3/mut/backup/` (EC1-3,
  LC1-4, PV1-3) is byte-identical (`cmp`) to its file in the tree. The gate's self-test plants were checked by reading
  the whole `git diff -- src/ Makefile` (below). The test binary (`build/test/shulib_tests`, 2026-10-01 21:58)
  predates every program edit, so no suite result is trusted until it is rebuilt from source.
- **Plan:** read the unlogged work whole against the brief, log it here item by item (what it built, every delta),
  finish what is unfinished, then the full gates. Nothing of it is reverted unless wrong (logged with why).

### 08:06–08:58 — the stopped runs' unlogged work, read whole and logged; what this run finished *(`date` 08:57 at this entry)*
**Read against the brief, file by file (`git diff` of every changed file; the edit scripts in `scratch/t4-3/edits/`).
Nothing of it was reverted; three stale sentences were fixed (below).** The early/late split, item by item:

| Item | Built EARLY (this tree) | Left for the LATE step, and why |
|---|---|---|
| "D, again" (the encoder check) | all (21:02–21:25 above) | — |
| 4′ library pieces | `diag::LogCopySink`, the provider's copy constructor (21:25–22:58 above) | — |
| 4′ the SD copy per program | "shulib Drive" passes the bare `"drive_log.txt"` (`drive_program.cpp`, `kLogCopyFile`; Truncate); its boot says where the copy is, or that it did not open, ONCE (`log copy:` line) | the competition program's copy, Append (Decision B) — its file is COMP-b's. **Not given a copy, logged for the coordinator:** the Auton Tester (the brief names Drive and COMP only; it keeps COMP-t's own SD run log), Calibrate and the X-drive (neither in the brief's list) |
| 1 the new program | `src/programs/drive_program.{hpp,cpp}` REPLACED: tag `DRV`, `driveInitialize/driveAutonomous/runDrive`, `kDriveAutonomousReason` (46 chars, the brief's own sentence, checked <= 50); the boot in Teleop's order (banner + build stamp + run status; the boot source; the provider WITH the copy, `validate`, `emplace` and `st.driver.bind()` in ONE try, `unbind()` first in its catch; `attachFaultLatch`; the session header; the boot lines MOVED from Teleop without `clawBannerLine()`, the driver line and the protections line GENERATED from `RobotTwoDriver::kFeel` / `kLoop.protection`); ONE function-local static; `DriveScreen` (the 4-argument `perTick`) builds every row of the pages each 200 ms. **NEW `src/programs/drive_panel.hpp`**: the pure page model — MAIN / DETAILS / MORE as `competition::MenuLine` rows (the type COMP-b's painters paint), `DrivePager` (release edge, arming delay, a press held across a flip refused), the source rows, the state words and the DETAILS/MORE rows, all moved from Teleop | **painting** the visible page and the refusal SCREEN (`src/common/screen_paint.{hpp,cpp}`, COMP-b), **the touch read** feeding `DrivePager` (`src/common/screen_touch.hpp`), **the controller row** and **the controller's three lines** with the ONE chooser and the line writer (`src/common/driver_panel.hpp`), and **the PROTECTION row's words** (Part 2b's, written INTO `driver_panel.hpp`) — every one is a COMP-b file. Until then MAIN's two rows are empty and a refusal is said on serial only (the file's header, "EARLY STEP"); this tree is never uploaded as it is |
| 2 what the old program deletes / moves | all of the brief's table: the open-loop volts, the typed-table refusals, the bare `ProsMotor`s, its own `ProsBlockSink`, its panel and constants, `AllStopGuard`, the protections (already the library's, Part 2a). Teleop's pieces MOVED, never copied: tag/stamps, `logWrapped`, the boot literals (`kProvisionalLine` lost "feel different from shulib Drive"; `kDeadPortLine1` lost "the same policy shulib Drive uses"), `sourceRows()` with its three static_asserts, `qualityName/modeWord/fieldCentricWord`, the panel's rows (split into MAIN/DETAILS/MORE), `logBootLines`, the entry points | Teleop's `paintRefusal`/`refuse` painter: **deleted, not moved** — COMP-b made the shared PROS-bound refusal painter (ND3 (b), `paintRefusalScreen`), so the late step wires "shulib Drive" to THAT one (the brief's re-check item 7 note) |
| 3 Teleop retired | `teleop_program.{hpp,cpp}` deleted (`git rm`); `main.cpp` (five switches; Drive's own `autonomous()` sentence; the build table and run-status notes); `build_selection.hpp` (`Program::Teleop`, the tank-library branch and its `#error` gone; the count guard; a RETIRED section; `PROGRAM=drive`'s `#error` in the brief's words); the Makefile (`PROGRAM=library` → the brief's `$(error)` text exactly; drive's error; the valid values; the programs comment); the src gate (below); `ci.yml`'s two comments; Calibrate's save line and the X-drive's `kRobotTwo` in the brief's exact words (45 chars); the Auton Tester's slot 7 → 2 in the Makefile, `main.cpp` and gate comments (no code string names a slot — COMP-t's pin) | the claw row, fault row, LCD word and claw boot lines MOVING from `teleop_claws.hpp` into COMP-b's `driver_panel.hpp`, COMP-b's include and calls re-pointed, `teleop_claws.hpp` deleted and `test/teleop_claws_test.cpp` becoming the shared panel's test (COMP-b's files); `main.cpp`'s FINAL form after COMP-b's eighth build |
| 5 Part 1b folded | `mechanism_table.hpp` lost `forward/reverse/volts/toggle`, `kTeleopRezeroButton` and checks 4-7 (renumbered 1-5: count, completeness, ports, disjoint, lines); WHAT IT DECLARES / WHY std::optional / COMPILE-TIME CHECKS / WHO READS IT rewritten; provenance strings keep the team lead's words + "(buttons: src/team/robot_two_driver.hpp since T4)"; no port/sign/cartridge/safe mode/line/polarity/position changed. `teleop_claws.hpp`: `TeleopClaws`, `controlsMatchTable`, the forwarder and `clawBannerLine()` gone; the two boot lines take the driver file's controls (`std::span<const HoldControl>` / `<const ToggleControl>`) and find each entry's buttons BY NAME (`holdControlFor`/`toggleControlFor`). The driver file's header: §2's "change it HERE and nowhere else" restored (its caller sentence names "shulib Drive" and the competition program) | the same for the competition program (its call sites of the two boot lines take the controls — COMP-b's file); the driver file's PROTECTIONS paragraph (Part 2b item 6) |
| C5 (2026-10-02) | — | all of it (the competition program's registration path; the tester's if shared) — COMP-b's files |

**Deltas from the brief (each with its reason):**
1. **The BUILT boot line's end** `-- in shulib Teleop only` (COMP-b's tree: `-- in shulib Teleop and shulib Competition`) became
   `-- buttons: src/team/robot_two_driver.hpp`, not "-- in shulib Drive and shulib Competition": the brief allows a
   reword only where the words name Teleop, and this one did. A list of programs went stale the day COMP-b added one;
   where the buttons live is what a reader of the line needs since they left the table, and it stays true when a
   program is added. The late step resolves COMP-b's version to this one and re-quotes worksheet J.3.1.
2. **A third page, MORE** (DETAILS' second page): the brief's own fallback (":887 If that does not fit one page,
   DETAILS gets a second page (MORE), with R5's four on its first"). DETAILS holds R5's four + the command, estimate,
   mode and battery (9 SMALL rows, the most above the button row); MORE the two groups, the travel and the
   protections' counters, last cut and peaks.
3. **The page model is its own file, `src/programs/drive_panel.hpp`**, not rows added to the shared panel header:
   MAIN/DETAILS/MORE are this program's alone; what is shared (the PROTECTION row, the controller row, the chooser)
   stays the shared header's and is handed in as rows. Rows are `competition::MenuLine` so COMP-b's painters paint
   them with no adapter.
4. **The gate's NEW pair takes the RETIRED cases' numbers (12-13)** rather than new numbers, so 14+ keep theirs; 12
   also plants the retired switch in `build_selection.hpp` (as an `#error` guard, legal there for a LIVE switch) and in
   the host CMake, and checks steps 5 and 6 name the retired define and beacon — additive to the brief's pair.
5. **Case 19's four `SHULIB_PROGRAM_LIBRARY` plants re-pointed to the LIVE `CALIBRATE`/`AUTONTEST`** (each keeps its
   file and directive form; none copies `#ifndef SHULIB_PROGRAM_DRIVE`), and case 27/28's to `CALIBRATE`/`AUTONTEST`
   — a retired switch planted there would test the RETIRED list (case 12), not the rule.
6. **Case 26's re-pointed mutation** names `kMechanisms` in the new file's `motorClawBootLine(kMechanisms, ` (the
   brief: "a line the new file holds that names kMechanisms"); **case 30's** dead-path plant is the new file's
   `"TYPED TABLE (src/robots/chassis_table.hpp)"` boot-line literal; **case 29's** include is
   `"robots/mechanism_table.hpp"`; **case 28's** `CLAWS_PLANT` retired onto `DRIVE_PLANT`'s
   `#include "common/boot_source.hpp"` (its file is deleted in the late step).
7. **`kMaxMechanisms` stays 6** (it was 6 at the base; the "five" bound was argued from the buttons, and its comment now
   says the driver file's checks refuse what a controller cannot carry).

**Fixed in the unlogged work (stale sentences, this run):** `mechanism_table.hpp` "a swapped button in this file turns
the suite red" (the buttons left it) and "check 9 refuses a line declared twice" (now check 5); `drive_program.cpp`'s
`driveAutonomous()` "the state screen already says why" (the refusal is said at boot, on serial, until the late
step); `robot_two_driver_test.cpp`'s header "no program runs it yet".

**This run's code (08:06–08:58):**
- `test/robot_two_driver_test.cpp` — the one compile error the 16:37 run left (`kinds.pneumatics[1].toggle`).
- `test/teleop_claws_test.cpp` — RE-POINTED, never weakened (the brief's P3-T4/P3-T5): the fixture `RobotTwoClaws` binds
  through the driver file's binder (`DriverBindings`) with the driver file's controls and its re-zero (Y); every word
  pin UNEDITED except the BUILT line's end (delta 1; 1g, 1n — three pins); `clawBannerLine`'s two pins in 1g and one in
  1n RETIRED with the function (J.3.2's sentence; only Teleop printed it); 1d re-pointed (the re-zero is
  `RobotTwoDriver::kLoop.rezeroButton`; the same two refusal sentences, the library's `validBindings`); 1i
  RETIRED → `robot_two_driver_test` T10 + `driver_controls_test` "refuses, by name" (the by-name refusals), and its
  "swapped" sub-case turned into what is now CORRECT: a robot built with the pivots in the other order binds, R1
  still drives port 6, each boot line names its own pivot's buttons (NEW, in 1i's place); 1m RETIRED →
  `driver_controls_test` "a hold bind() cannot build (13 V) THROWS" (which says it carries 1m's intent); 1k re-pointed
  (the table's buttons moved into the case's own controls; every check kept, by name); 1l RETIRED with
  `teleop_program.cpp` → `test/drive_program_source_test.cpp` (below), its `boot_source.cpp` pin kept there.
  **NEW P3-T5 case:** the boot lines read the CONTROLS — a copy with R1/R2 swapped prints `R2 forward / R1 reverse`;
  an ABSENT pivot names its buttons from the controls (`L2/L1 do nothing`); controls in the other order with 6 V on
  pivot 7 print each pivot's own; an entry no control names prints `?` and 0 V; a toggle on B prints B.
- `test/drive_protection_test.cpp` — the Part 2 source pin read `teleop_program.cpp`'s `loopCfg`: re-pointed to
  "shulib Drive" (its CODE names no `DriverLoopConfig`, calls no `runDriverControl(`, runs `st.driver.run(`, and every
  `.protection` it names is a READ of `RobotTwoDriver::kLoop.protection`). `test/driver_controls_test.cpp` — the
  heading-rule pin's Teleop line → "shulib Drive" names `driverHeading(` nowhere (it reaches it only through the
  driver file).
- **NEW `test/drive_program_source_test.cpp` (P3-T3, 8 cases):** the boot's ONE try (the exact emplace → bind →
  refusal → catch-unbind sequence; latch < header < boot lines); the loop only through `st.driver.run(` in
  `runDrive()` with `competition_is_disabled` only in its lambda, the refused path idling before any `*st.robot`;
  `driveAutonomous()` with no `chassis()`; none of the old program (`mapSticks(`, `tankSideVolts(`, `ProsMotor`,
  `ProsBlockSink`, `clawBannerLine(`, the chassis-table include, `kChassis`) and the copy reached only by the bare
  `"drive_log.txt"`; the claw boot lines from `kMechanisms` + the driver file's controls, the exit line;
  `main.cpp`'s three Drive cases and no Teleop; Teleop's names in NO code and "shulib Teleop" / "(slot 2)" in NO
  string literal under `src/` and `include/shulib/`; `kTeleopRezeroButton`, `controlsMatchTable`, "until T4 Part 1b"
  in no code under `src/` or `test/`, no mechanism table setting a button field; `runDriverControl(` called only by
  the driver file and the X-drive, a binding named only in `driver_controls.hpp`. *(Its helpers are its own: COMP-b's
  `test/source_pins.hpp` is not in this tree. The late step re-points this file — and `auton_tester_source_test.cpp`
  if COMP-b has not — to it: one copy.)*
- **NEW `test/drive_panel_test.cpp` (P3-T2, 10 cases):** MAIN holds exactly the bar, PROTECTION, controller, DETAILS;
  DETAILS opens with R5's four (status, two source rows, claw, fault) then cmd/est/mode/battery, MORE left and BACK
  right; MORE's six rows and BACK; all 17 rows on exactly one page, placed SMALL at x 8 / y 32+16i above the buttons;
  every row's words and tone (connected/not, DEGRADED, fell back, odometry or not, a group with a BOOT hole AND a
  runtime mark → `dead 17 18`, each protection mode's word and colour, the last cut, the peaks with NaN and 3 and 99
  groups); every row screen-safe at the extremes (1e12, NaN, a full dead list, a stray `%`); the source rows; the
  pager (a pre-program tap never replayed, a PRESS alone never flips, an error read, a release off every button, the
  same sample twice, a reflexive second tap inside the arming delay consumed and never replayed, MORE/BACK, a press
  held across a flip refused even after the delay, the hit box's pad). The LCD chooser's order and the PROTECTION
  row's states are the shared header's (COMP-b's `driver_panel_test.cpp` + Part 2b's) — late.
- **Suite (fast build, full binary, no filter):** `1983 | 1983 passed | 0 failed | 3 skipped`,
  `assertions: 2723200 | 2723200 passed` (was 1970 at 22:58; +13 = `drive_panel_test` +10, `drive_program_source_test`
  +8, `teleop_claws_test` 14 → 13 (1l and 1m retired, P3-T5 added, 1i replaced in place), `robot_two_driver_test`
  15 → 14 (the tie case), `mechanism_table_test` 19 → 16 (the three button/volts cases, retired by the 16:33 run
  with their map in the file)).

### RESUMED AGAIN — Sun Oct 4 09:53 AM EDT 2026 (`date`), a third executor run (opus) in the same worktree
- **Why:** the run above was STOPPED after its 08:58 entry. The tree shows what it did after that entry
  WITHOUT logging it: at 09:09 a sweep of the LIBRARY header comments that named "shulib Teleop" or the old
  "shulib Drive" (`boot_check.hpp`, `calibration_record.hpp`, `robot_config.hpp`, `session_info.hpp`,
  `device_registry.hpp`, `block_source.hpp`, `robot.hpp`, `motor_group.hpp`, `drive_protection.hpp`,
  `stick_mapping.hpp`, `coupled_side_monitor.hpp`, `pros/motor.hpp`, `drivetrain_degradation.hpp`,
  `teleop_loop.hpp` — comments only), and at 09:16 a scratch reflow helper (`scratch/t4-3/tools/reflow.py`)
  that was never run: several of those re-worded comment lines ran to 116-163 characters.
- **Checked first:** no mutation left applied — the seven LC/PV backups are byte-identical to their files;
  the three EC backups differ from `coupled_side_monitor.hpp` ONLY in the 09:09 comment lines (read with
  `diff`: the "old shulib Drive" re-wordings at :17-19, :181-182, :206-208, :214-215 — no code line). No gate
  plant in `src/`, `tools/` or the Makefile (the whole `git diff` read). The briefing regenerated; the host
  build reconfigured; the FULL build (`base/build6.log`) stopped at the doc-staleness gate (twelve
  `src/programs/teleop_program.cpp` paths in `README.md`, guides 07/08/10/14, `hardware-assumptions.md`,
  `build-order.md`, `roadmap.md` — the documents are this run's work below) before linking, so the binary was
  RELINKED from source without the doc gates (`scratch/t4-3/fastbuild.sh`, `base/fast7.log`, 10:04):
  `[doctest] test cases: 1983 | 1983 passed | 0 failed | 3 skipped`, `assertions: 2723200 | 2723200 passed` —
  the 08:58 entry's counts, now on a fresh binary.
- **The 09:09 sweep, checked line by line (kept; re-wrapped, and three sentences fixed):** every re-worded sentence is true of
  the tree (the Auton Tester DOES boot from the record — `auton_tester_program.cpp` calls the shared boot
  source); the overlong lines were re-wrapped (`calibration_record.hpp`, `robot_config.hpp`, `pros/motor.hpp`,
  `robot.hpp`, `coupled_side_monitor.hpp` — `tankSideVolts`' doc —, `stick_mapping.hpp`, `teleop_loop.hpp`,
  `motor_group.hpp`); and THREE sentences it left or made false were fixed: `stick_mapping.hpp`'s header still
  said "the drive program plus the bench tester's DRIVE station still call `mapSticks()` … Both callers move to
  `DriveShaper` when src/ is re-pointed" (the new program calls neither; only the station does); `motor_group.hpp`'s
  section title "ONE evaluator, shared with the drive program" (now "with the tester's DRIVE station", which
  runs `CoupledSideMonitor` — `bench_r3a.cpp` :1601); `device_provider.hpp`'s NO HARDWARE CLAIM listed
  "shulib Calibrate" among the programs that construct a provider — it constructs none (grep: only Drive, the
  Auton Tester and the X-drive do; the competition program joins in the late step) — and left out the X-drive.
  `docs/api` regenerated (17 pages changed by these comments, plus the new `log_copy_sink.md`).

### 10:05–11:15 — this run: the comment sweep finished, the documents of what the early step built, Part 3's mutations, the gates *(`date` 11:10 at the mutations' end)*

**The comment sweep — the brief's RULE, not only its list** (`grep -rn 'shulib Teleop\|teleop_program\|Teleop\|drive
program\|shulib Drive\|slot 2' src/ include/shulib/`, plus `slot 7`): 114 hits at this run's start, every one read.
- **Re-worded (this run), `src/`:** `common/screen_text.hpp` :7-9, :27 (Teleop's sanitizer use → past; the words'
  table → `drive_panel.hpp`); `common/fault_latch_hook.hpp` :10-12 (Drive's graph, Teleop's past); `common/state_screens.hpp`
  :14-15 ("Bench Tests", "shulib Drive", "shulib Calibrate" share them — NOT the Auton Tester, which paints its own
  states, `main.cpp`'s cases — and Teleop did), :33 (`kDriveAutonomousReason`); `common/driver_controls.hpp` :260
  (the heading rule's callers); `programs/xdrive_program.cpp` :8, :31, :97-98 (the re-zero is the driver file's Y),
  :115-116 (the check `drive_program.cpp` makes as `driverLineMatchesFeel()`), :238-239; `robots/xdrive_config.hpp` :20;
  `programs/calibrate_program.cpp` :10 (the brief's words: Drive (slot 1) and the competition program boot from the
  file), :47-48, :107-108, :256-257; `programs/calibrate_program.hpp` :7 ("one function, as the OLD Drive's was" — the
  new one has three entry points); `programs/bench_r3a.hpp` :15-16; `programs/bench_r3a.cpp` :28, :150-152, :241-242,
  :1641; `robots/chassis_table.hpp` :4-8 (its readers since T4 Part 3: the tester and the boot source's fallback,
  `tableToDescription()`), :30-34, :81, :154-155, :264-265 (`tableSignedPort` is the tester DRIVE station's now —
  `bench_r3a.cpp` :1559), :453.
- **Kept as TRUE history (no edit):** every "the old "shulib Drive"" / "(retired at T4 Part 3)" sentence the earlier
  runs wrote; `boot_source.hpp` :59 (the 2026-09-25 boots happened inside Teleop); `xdrive_program.cpp` :5, :14
  (`runTeleopLoop`, a deleted function, named as deleted); `teleop_loop.hpp` :176 (the feel Teleop was driven with
  on 2026-09-25); `drivetrain_degradation.hpp` :3, :6 ("a drive program" in the general sense, and the instruction's
  history); `drive_protection.hpp` :8 (the ruling's own words, glossed at :11); `bench_r3a.hpp` :171 ("slot 2" is a
  screen slot there); `teleop_claws.hpp`'s header (the previous run's — the file moves in the late step);
  `drive_program.{hpp,cpp}`/`drive_panel.hpp`'s "moved from Teleop" notes; `main.cpp`/`build_selection.hpp`/Makefile/
  gate/`ci.yml`'s retirement notes.
- **Left for the LATE step (COMP-b rewrote the very lines; a word changed here would conflict and be re-done):**
  `common/boot_source.hpp` :9-12 (names `src/programs/teleop_program.cpp`; COMP-b's version names Teleop, the tester
  and "shulib Competition"); `common/fault_latch_hook.hpp` :16 ("the teleop program included"; COMP-b's "all eight
  builds"); `programs/auton_tester_program.cpp` :186, :363, :590 (each names `teleop_program.cpp`; COMP-b rewrote that
  file's refusal painter and these comments' neighbours). And EVERY "seven builds" / COMP-b's "eight builds" comment
  (`boot_source.cpp` :12, `fault_latch_hook.hpp` :16-17, `xdrive_program.cpp` :35, :37, `auton_tester_program.cpp`
  :49): this tree has SIX builds and the merged tree SEVEN, so the late step writes "seven" once.

**Documents of what the early step built (this run; the prose review below):**
- **Dead paths (the staleness gate's twelve):** `README.md` :244-257 (Drive is the worked example; "built the same
  way as robot two's"); guide 07 :135-143 (the worked example; the claw words' file and that their buttons are the
  driver file's); guide 08 :53, :220, :313-318 ("every program in `src/` that builds a robot builds it this way" — it
  said "two programs", stale since COMP-t; the auton tester named); guide 10 :199-204 (the honest status: the driver
  file drives through the loop, "shulib Drive" runs it); guide 14 :376-381 (the same; the raw mapping is the tester
  station's only — "until a driver has compared the two on the robot" now names that baseline), :396-397 (the driver
  file gives the claws); `hardware-assumptions.md` HA-112, HA-124, HA-131 (dead path), HA-132's consumers, HA-141's
  settle, HA-143's release feel, the claws' group text, HA-153..HA-159's program text (Teleop → "shulib Drive" / the driver file;
  HA-158's and HA-159's "slot 2" → slot 1), the exclusions table; `build-order.md` :1440 and `roadmap.md` :337, :1806 —
  DATED notes, kept as history with "(deleted at T4 Part 3)" on the line (the staleness audit's own rule for a path
  recorded as deleted).
- **Item 5's folded 1b documents done here:** cookbook 06's "Watch out for" bullet ("change a button in both files —
  and the re-zero in three") → "in the driver file and nowhere else"; HANDOFF L5 :448-452 (1a's "ALSO stated … tied in
  Teleop until Part 1b") and :456-460 (the re-zero paragraph named `kTeleopRezeroButton` and `teleopInitialize()`);
  HA-153's source text. guide 07's claw-wiring sentence (above).
- **N4 (the tester to slot 2):** guide 16 :50, :55, :58-61 (the upload line, the warning, and a brain uploaded
  before may still hold it in slot 7). `faq.md` :218 (Drive checks the record against the ports).
- **The changelog** (`docs/changelog.md`, a new section at API 2.4's top): `diag::LogCopySink`, the provider's copy
  constructor and its four accessors, and the encoder check as a BEHAVIOUR CHANGE; no version bump (an
  implementation of F4's `ITelemetrySink`, not a change to it; `teleop/` and the adapters are in no register row).
  `hal::TeeCharSink` was NOT built — Decision A (a) replaced it with `LogCopySink` (the brief's 4′).
- **Plan documents (own lines only):** HANDOFF :109-115 (the typed table is no fallback for Drive — it drove the OLD
  Drive; Calibrate must succeed before "shulib Drive" can run at all), :348, :670-671; build-order's T4 entry gains
  a "Status 2026-10-04: Part 3 IN PROGRESS" line (what the early step built, what the late step does).
- **Left for the LATE step (COMP-b rewrote the same lines, or the words describe what only the late step wires):**
  the WORKSHEET's whole Part 3 pass (§4 Part 3: Station F rewritten — F.0's OLD/NEW check, F.1's MAIN/DETAILS/MORE
  and the LCD, F.5, F.6, F.7's `drive_log.txt` —, E's pointer, J moved to slot 1 with J.3.1 re-quoted and J.3.2
  retired, G.1 and G.6, A's map, the DO NOT list, the tester's slot-7 lines, 1b's J.4 and F row 4): its screen
  sections describe the painting, the PROTECTION row and the chooser the late step wires, and COMP-b rewrote 258 of
  its lines (Station K, A's map, J.3.1) — it is written once, from the merged code; the README's `src/` layout block
  (:294-306) and guide 07 :124-126 and cookbook 06 :280-283 ("no program runs this file yet" — COMP-b re-worded all
  three); `RESUMING.md` :114-141 and `ORIENTATION.md` :91-136 (COMP-b re-worded both; seven builds after the merge);
  HANDOFF's program list :277-287 and L5b (COMP-b touched :276, :285, :567-602); REMAINING-WORK :47, :151 ("Station J
  … on shulib Teleop"); the first `**Next:**` line of `build-order.md` (the coordinator's pointer; it still says "D,
  again" open).

**Test added (a survivor, below):** `test/teleop_claws_test.cpp` P3-T5 gains two checks — a toggle list whose only
entry names ANOTHER mechanism prints `?`, and one with the claw's toggle SECOND prints `B`.

**Part 3's mutations** (`scratch/t4-3/mut/mutate.py` + `mutations_p3.py`; the scratch build dir re-configured and
FULLY rebuilt first — `1983 | 1983`, the new test files' stale-binary trap; FULL suite each; restore + `cmp` +
timestamp bump; `mut/mutation-results.txt`). `src/programs/drive_program.cpp` and `src/main.cpp` are compiled by no
host TU (no screen shim): their source pins read the file at RUN time (`SHULIB_SOURCE_DIR`), so those runs recompile
nothing and run the fresh binary built from the unmutated tree — marked SOURCE-ONLY, not VOID:

| # | Mutation | Red (FULL suite) |
|---|---|---|
| DP1 | the program's `run` swapped for a direct `runDriverControl` (named) | 3: P3-T3 "drives ONLY through run()", P3-T3 "the loop is called only by the driver file and the X-drive", Part 2's source pin |
| DP2 | `bind` moved OUTSIDE the try (named) | 1: P3-T3 "binds ONCE, inside the try" |
| DP3 | `unbind()` not first in the catch | 1: the same |
| DP4 | the refused path returns instead of idling | 1: P3-T3 "drives ONLY through run() … the refused path idles" |
| DP5 | the provider built without the copy's file name | 2: P3-T3 "none of the OLD Drive survives … the SD copy is the provider's", "binds ONCE" |
| DP6 | `driveAutonomous()` reaches `chassis()` | 1: P3-T3 "drives ONLY …; autonomous commands nothing" |
| DP7 | `competition_is_disabled()` outside the lambda | 1: the same |
| MN1 | a leftover `#include "programs/teleop_program.hpp"` in `main.cpp` (named) | 2: P3-T3 "Teleop retired everywhere", "main.cpp's Drive cases"; and the ARM build: `make ROBOT=tank PROGRAM=drive` on a clean `bin/.d` exits 2, `src/main.cpp:77:10: fatal error: programs/teleop_program.hpp: No such file or directory` |
| MT1 | `kTeleopRezeroButton` left in `mechanism_table.hpp` (named) | 1: P3-T3 "the buttons left the mechanism table" |
| PG1 | the page flips on a PRESS (named) | 1: P3-T2 "flips on the RELEASE edge … never on a press alone" |
| PG2 | a press held across a flip honoured | 1: P3-T2 "a press held ACROSS a flip does nothing" |
| PG3 | the arming delay ignored | 1: P3-T2 "flips on the RELEASE edge … once its arming delay has run" *(first form left `armedNow` unused — a -Werror BUILD failure, not a red; re-written to keep the clock observed and arm every touch)* |
| PG4 | the press count not re-latched at a flip | 1: P3-T2 "held ACROSS a flip" |
| DV1 | MAIN gains the fault row | 2: P3-T2 "MAIN holds exactly …", "every row on exactly ONE page" |
| DV2 | R5's claw and fault rows dropped from DETAILS | 2: P3-T2 "DETAILS opens with R5's four", "every row on exactly ONE page" |
| CL1 | a motor claw's boot line pairs its control by TABLE INDEX | 2: P3-T5, 1i's replacement (the pivots in the other order) |
| CL2 | a boot line prints the table's OLD buttons (R1/R2 hard-coded) — the named "reads the buttons from anywhere but the controls", in the only form that still compiles | 4: P3-T5, 1g, 1i, 1n |
| CL3 | a pneumatic claw's boot line takes the FIRST toggle | **SURVIVED** (robot two has one toggle; the no-control case passed an EMPTY list) → the two P3-T5 checks above → re-run on a fresh full build: **RED**, 1: P3-T5 |

*Stated, not run here:* "CUT ranked below a fault in the chooser" — the chooser is COMP-b's (`driver_panel.hpp`),
the late step's; "LogCopySink forwarding emit()" is LC1 (above, red). **P3-T1's plants run for real inside the gate's
self-test** (206 cases, each against a real build, PASS): the retired pair (case 12 — `SHULIB_PROGRAM_LIBRARY` planted
in `src/` files, `build_selection.hpp` and the host CMake, and its define and beacon in steps 5/6; case 13 — `make
ROBOT=tank PROGRAM=library` stops at the `$(error)` naming the retirement), the NUL match (case 10: a tank build
wearing the tank-drive label), step 8 (case 20: Drive's and the tester's calls swapped), and every re-pointed plant
(cases 16, 19, 21, 24, 26-30, logged in the 08:58 entry), each through `_replace`'s exactly-once assertion.

**Gates on this tree (11:15; the final run is the entry below):** host build with the doc gates PASS; suite
`1983 | 1983 passed | 0 failed | 3 skipped`, `assertions: 2723202` (+2: CL3's checks); `src_build_gate.py
self-test` OK (206) and `check` PASS (6 builds, 0 disallowed warnings); the ARM header gate over all 198 headers
exit 0; both CI greps clean; `api_doc_tool` check-fresh / check-examples (503 lines) / self-test / check-coverage
(3217 entities, 162 headers) / check-removability; `doc_staleness_audit.py` clean; `briefing_status.py` self-test
(24) and check.

**The prose review (run and read BEFORE this entry):** the brief's Part 3 terms over `README.md`, `docs/*.md`,
`docs/guide/`, `docs/cookbook/`, `docs/internal/*.md`, `src/`, `include/shulib/` (chunk records excluded: they are
records) — "shulib Teleop", "PROGRAM=library", "tank-library", "teleop_program", "teleop_claws", "drive_log",
"runDrive", "adapters, NOT the motion stack" (no live hit), "slot 7", "kTeleopRezeroButton" / "controlsMatchTable" /
"TeleopClaws" (no live code hit; the docs' hits are the HANDOFF history above), "both files" / "there too" (fixed:
cookbook 06, HANDOFF L5), "seven builds" / "eight builds" (the comments above, the late step's), "Part 1b" (history
and the plan's fold notes); and the library pieces' terms — "LogCopySink", "log_copy_sink", "serialCopy",
"TeeCharSink" (no hit), "encoder check", "D, again", "reads frozen", "settles well before". Every hit not edited is
true history, the plan's own record, or on the LATE list above.

### THE EARLY STEP'S HAND-OVER — exactly what the LATE step must do (after the coordinator merges main, with COMP-b, into this worktree)
1. **Resolve the merge** in `src/main.cpp` (COMP-b's Competition case in all five switches + this tree's Drive case
   with its own `autonomous()` sentence, no Teleop), `src/build_selection.hpp` (COMP-b's competition branch and
   `SHULIB_PROGRAM_COMPETITION` in the count guard + this tree's retirement), the Makefile (COMP-b's `competition` +
   the retired `library`; the axis comment's entry count), `tools/src_build_gate.py` (COMP-b's eighth build and its
   self-test pairs + the RETIRED list and the re-pointed cases; tank-library stays out of every table), `ci.yml`'s
   comments, `src/programs/teleop_program.{hpp,cpp}` (COMP-b edited, this tree deleted: it stays DELETED — first read
   COMP-b's 52 changed lines there and confirm each moved into `src/common/` or is Teleop-only), `teleop_claws.hpp`,
   `test/teleop_claws_test.cpp`, the driver file's header, and every shared document (README, guides 07/14/16,
   cookbook 06, the changelog, the register, HANDOFF, build-order, roadmap, RESUMING, ORIENTATION, REMAINING-WORK,
   the briefing). Re-run the suite before trusting it (a fresh FULL build: COMP-b adds test files).
2. **Part 2b item 8 first** — re-read and log COMP-b's `driver_panel.hpp` (its rows, the chooser's order and whether
   it has a PROTECTION slot, where the claw/fault rows and the refusal painter live), `competition_program.cpp` (its
   `perTick`, source pins, boot lines, any "AT BOOT ONLY" / "not in this graph" words), Station K, the build list,
   `version.hpp` and the changelog, `teleop_loop.hpp`'s banner, and its reword of `-- in shulib Teleop only`.
3. **Part 2b item 6:** the PROTECTION row and the chooser's new words (`REFUSED see brain`, `CUT see brain`, row
   1's `dead <ports>`) in `src/common/driver_panel.hpp` at D4's place, pure and host-tested (P3-T2's chooser order,
   every pair, BACKUP if SAFE-SAVE B2 landed; the mutation "CUT ranked below a fault"); COMP's page gains the row;
   `competition_program.cpp`'s `perTick` takes the fourth argument (COMP-b's source pins re-pointed one-for-one); any
   COMP-b word carrying Teleop's dead-port claims made true; the driver file's PROTECTIONS paragraph (§2, verbatim)
   and `run()`'s doc, cookbook 06's "How it runs" re-quote, its one protections sentence and the loop recipe's fourth
   argument; P2-18; Station K's two lines and THE PUSH CHECK unless COMP-b wrote them.
4. **Item 1's wiring:** "shulib Drive" paints the visible page through COMP-b's painters (`screen_paint.hpp`, a row
   repainted only when it changed, at most 5 Hz), the refusal SCREEN through `paintRefusalScreen` with
   `kDriveRefusedWord` / `kDriveRefusalFooter` (and repaints it on the refused opcontrol path), the touch read
   (`screen_touch.hpp`) into `DrivePager`, MAIN's PROTECTION and controller rows, and the controller's three lines
   through the ONE chooser and line writer; remove the "EARLY STEP" notes (`drive_program.cpp`'s header and
   `refuse()`/`DriveScreen` docs, `drive_panel.hpp`'s header).
5. **Item 3's last part:** the claw row, the fault row, `kLcdClawAbsent` and the claw boot lines MOVE from
   `teleop_claws.hpp` into `driver_panel.hpp`; COMP-b's include and calls re-pointed (its boot-line calls take the
   driver file's controls — item 5 for the competition program); `teleop_claws.hpp` deleted;
   `test/teleop_claws_test.cpp` becomes the shared panel's test (word pins UNEDITED); the gate's `CLAWS_PLANT` (case
   28) and P3-T3's pins re-pointed; the BUILT line's end resolved to `-- buttons: src/team/robot_two_driver.hpp`
   (delta 1 of the 08:58 entry; COMP-b's is `-- in shulib Teleop and shulib Competition`) with worksheet J.3.1
   re-quoted; P3-T3's helpers onto COMP-b's `test/source_pins.hpp` (one copy).
6. **Decision B:** the competition program's provider gets `("comp_log.txt", hal::pros::ProsFileMode::Append)` (or
   the name the late step logs), its one `log copy:` boot line, and source pins like Drive's.
7. **`main.cpp`'s FINAL form** and every build count: SEVEN builds (bench, xdrive, tank, tank-drive, tank-calibrate,
   tank-autontest, tank-competition); "seven" in each comment COMP-b wrote "eight" (`boot_source.cpp` :12,
   `fault_latch_hook.hpp` :16-17, `xdrive_program.cpp` :35/:37, `auton_tester_program.cpp` :49, the Makefile,
   `ci.yml`, RESUMING); the slot map 1 Drive · 2 Auton Tester · 3 Bench Tests · 4 Calibrate · 6 Competition.
8. **The C5 ruling (2026-10-02, COMP-PROGRESS "C5"):** the competition program registers through a check that does
   not throw; each refused entry is named on the screen and in the log and kept off the menu; every other auton and
   driver control keep working (the students' file untouched); the same for the Auton Tester if it shares the
   registration path (log the choice); tests that FAIL on the unchanged code; chapter 17, Station K and the COMP
   brief's B15/B18 notes brought to the new rule.
9. **The documents this early step left** (its 10:05 entry, "Left for the LATE step"): the worksheet's whole Part 3
   pass; README's `src/` layout block; guide 07 :124-126; cookbook 06 :280-283; guide 10's honest status naming the
   competition program; RESUMING and ORIENTATION; HANDOFF's program list and L5b (and its note that R5a Part D and
   L9 Part B now have "shulib Drive" and COMP as callers); REMAINING-WORK :47 and :151; the first `**Next:**`; the
   comments `boot_source.hpp` :9-12, `fault_latch_hook.hpp` :16, `auton_tester_program.cpp` :186/:363/:590; any
   "slot 7" COMP-b wrote for the tester (guide 17, Station K/M); SAFE-SAVE A's `formatSaveReport` sentences if it has
   landed (landmine 25).
10. **For the coordinator to rule, not built:** an SD copy for the Auton Tester (COMP-t's file, landmine 23; it
    keeps COMP-t's own SD run log), Calibrate (constructs no provider) and the X-drive (refuses at boot on every
    robot) — the brief's 4′ names only Drive and COMP.
11. Then `T4-COMPLETED.md`, the briefing LAST, ONE review round, the fresh verification, the commit.

### THE EARLY STEP STOPS HERE — the final gates on this tree *(`date`: Sun Oct  4 11:33:00 AM EDT 2026)*
- **Host:** the briefing regenerated, the FULL build (doc gates included) exit 0, the binary relinked 11:15:
  `[doctest] test cases: 1983 | 1983 passed | 0 failed | 3 skipped`, `assertions: 2723202 | 2723202 passed`
  (the baseline at `7daf809` was 1958 / 2709569); the briefing regenerated again after the binary.
- `src_build_gate.py self-test`: OK (206 detector cases, each against a real build); `check`: PASS (all 6 builds
  link, 0 disallowed warnings, every beacon, step 8).
- The ARM header gate over all 198 `include/shulib` headers: exit 0. The CI greps: PROS-free and sim-free.
- `api_doc_tool.py` check-fresh exit 0, check-examples (503 lines, 5 whole-run listings), self-test OK,
  check-coverage (3217 entities, 162 headers), check-removability; `doc_staleness_audit.py` clean and its
  self-test OK (10); `briefing_status.py` self-test OK (24) and check current.
- **Nothing committed** (`HEAD` is `7daf809`; the two staged deletions are `git rm`'s, uncommitted). No mutation
  or gate plant left applied (every runner restore `cmp`-verified; the gate's `_replace` asserts byte-exact
  restores). The tree waits for the coordinator's merge of main (with COMP-b) and the LATE step.

## Part 3 — THE LATE STEP (main with COMP-b and SAFE-SAVE A merged in; Part 2b's items; the competition program's SD copy; C5; `main.cpp`'s final form)

- **Started:** Sun Oct  4 11:44 AM EDT 2026 (`date`), in the worktree `.claude/worktrees/t4-3` (branch `t4-3`), on the
  early step's transport commit `6abcac8`.
- **Executor model:** opus. Do not commit; never abort the merge; no `pros`; no `/dev/ttyACM*`. Scratch, probes and
  backups under `.claude/coordinator/scratch/t4-3/` only.
- **The merge:** the coordinator ran `git merge --no-commit --no-ff fab1ebb` (main: COMP-b `237c4fe` + SAFE-SAVE
  Part A `fab1ebb`) before this run; `MERGE_HEAD` is set. 16 paths left conflicted (`git diff --name-only
  --diff-filter=U`): `.github/workflows/ci.yml`, `Makefile`, `docs/api/README.md`, `docs/api/all-entities.md`,
  `docs/api/calibration_record.md`, `docs/changelog.md`, `docs/internal/HANDOFF-remaining-work.md`,
  `docs/internal/PROJECT-BRIEFING.md`, `docs/roadmap.md`, `src/build_selection.hpp`, `src/main.cpp`,
  `src/programs/teleop_claws.hpp`, `src/programs/teleop_program.cpp` (modify/delete), `src/team/robot_two_driver.hpp`,
  `test/teleop_claws_test.cpp`, `tools/src_build_gate.py`.
- **The coordinator's rulings for this step:** (1) the SD log copy goes to "shulib Drive" and the competition
  program ONLY, as item 4′ names them — NOT the Auton Tester (it keeps COMP-t's own SD run log), NOT Calibrate (it
  constructs no provider), NOT the X-drive (it refuses at boot on every robot) — the coordinator's ruling of
  2026-10-04 on the early step's open question (its hand-over item 10); (2) the team lead's C5 ruling of 2026-10-02
  ("drop just that auton"; COMP-PROGRESS, the coordinator's merge entry): one auton entry the registry refuses
  must NOT refuse the competition program's boot.
- **Read first:** RESUMING; the T4 brief whole (header notes, the 2026-09-28 amendments, § Decisions, §1 Part 2 items
  6 and 8, §1 Part 3 items 1-5 with 4′, §2, §3's Part 3 tests, §4's Part 3 documents, §5-§7); this log's Part 3
  EARLY STEP entries and its HAND-OVER list; COMP-PROGRESS's C5 entries (:1078-1081, :1218-1223, :1388).

### 11:44–12:20 — the merge's 16 conflicts, resolved one by one *(`date` 12:20 at this entry)*
Each to the state BOTH changes intend: COMP-b's eighth build kept, the library build (Teleop) gone, the tester on
slot 2 — so SEVEN builds (bench, xdrive, tank, tank-drive, tank-calibrate, tank-autontest, tank-competition) and
five programs in five slots (1 Drive, 2 Auton Tester, 3 Bench Tests, 4 Calibrate, 6 Competition). Staged with
`git add` / `git rm` only (that is how a conflict is marked resolved); nothing committed, the merge not aborted.

| Path | Resolution, and why |
|---|---|
| `src/main.cpp` (4 hunks) | the build table: this tree's Drive/tester(slot 2)/Calibrate rows + COMP-b's Competition row + the RETIRED library note; "Five programs in five slots (1 Drive, 2 Auton Tester, 3 Bench Tests, 4 Calibrate, 6 Competition)" and "the five tank builds" (COMP-b: six/six — it counted Teleop); WHAT RUNS: this tree's history paragraph (Teleop and the old Drive gone) + COMP-b's autonomous() bullet (it is true: Competition runs a routine); the precondition doc: COMP-b's "refuses on screen" (Drive paints its refusal screen from this step); autonomous()'s doc: COMP-b's sentence minus "or teleop program" (no such program) plus the host-plant sentence this tree kept. Every switch case auto-merged: COMP-b's Competition case in all five, this tree's Drive cases, no Teleop. |
| `src/build_selection.hpp` (4) | the switch list and the count guard: COMPETITION in, LIBRARY out (it is on the RETIRED list, and the RETIRED section above says why); the beacon sentence "the other six — and the RETIRED `...=tank-library` — are NOT", the prefix list with `tank-competition`; "the five tank builds". COMP-b's competition `#error` and chain branch auto-merged. |
| `Makefile` (2) | COMP-b's `competition` description (its log copy named — Decision B, below) beside this tree's tester-on-slot-2 lines; "Five programs in five slots"; the ROBOT=tank-only list = drive, calibrate, autontest, competition ("any of the four"); the gate's "fourth to seventh entries (tank-drive, tank-calibrate, tank-autontest, tank-competition)"; the valid-values message without `library`, with `competition`. The `library` `$(error)` and COMP-b's `competition` block auto-merged. |
| `.github/workflows/ci.yml` (2) | "seven since T4 Part 3 retired the tank library program … drive/calibrate/autontest/competition"; the step comment's history + "seven builds since T4 Part 3, which RETIRED ROBOT=tank PROGRAM=library". |
| `tools/src_build_gate.py` (7) | the docstring: both histories (library RETIRED at Part 3; the sixth PROGRAM value, COMPETITION, at COMP-b); COMP-b's tank-competition block + this tree's RETIRED paragraph; `check`'s build list without tank-library, with tank-competition; the beacon paragraph "the other six — the RETIRED `…=tank-library` too — must NOT be"; step 8's "measured for … the eighth at COMP-b, and for the new tank-drive at T4 Part 3"; step 0's program-table history + this tree's RETIRED SWITCH paragraph; the self-test list with both "cases 34-35" and "cases 12-13 since T4 Part 3"; `VARIANTS` = the seven. Every table (BUILD_ARGS, VARIANT_DEFINES, VARIANT_BEACONS, BUILD_ROBOT, BUILD_PROGRAM, PROGRAM_ENTRY_SYMBOLS) and COMP-b's cases 34-35 auto-merged correctly (read). |
| `src/programs/teleop_program.cpp` (modify/delete) | **kept DELETED** (`git rm`). COMP-b's 52 changed lines there, read whole: (1) "all seven/eight builds" comment counts — Teleop's only; (2) a note that a file's string literals can survive `--gc-sections` in another build's package (words, never code) — Teleop's storage comment; (3) `refreshLcd` re-pointed to the shared chooser `controllerScreenLines()` and the one-line-per-write `ControllerLineWriter` (`src/common/driver_panel.hpp`, COMP-b's review) — **carried to the surviving program that needs it: "shulib Drive"'s LCD is written that way (item 1's wiring, below)**, and Teleop's test-1l LCD pins (COMP-b re-pointed them) move with it to `test/drive_program_source_test.cpp`. |
| `src/programs/teleop_claws.hpp` (2) | this tree's form (the boot lines take the driver file's controls; the BUILT line's end `-- buttons: src/team/robot_two_driver.hpp`, delta 1 of the early step, over COMP-b's `-- in shulib Teleop and shulib Competition`, which names a retired program). The file itself MOVES into `src/common/driver_panel.hpp` below (item 3's last part). |
| `src/team/robot_two_driver.hpp` (1) | this tree's caller sentence (§2's final form: "shulib Drive" on its own, and the competition program's opcontrol(); "change it HERE and nowhere else") over COMP-b's interim one ("so will the driver program … until T4 moves the buttons"). |
| `test/teleop_claws_test.cpp` (3) | this tree's form (the controls-taking calls; the BUILT pins' end as above; 1l RETIRED with its pointer) over COMP-b's (its 1l LCD pins on `teleop_program.cpp` — carried to Drive, above). |
| `docs/changelog.md` (1) | both 2026-10-04 sections, this chunk's first (it lands after SAFE-SAVE A). |
| `docs/internal/HANDOFF-remaining-work.md` (1) | this tree's T4 annotation ("D, again" ANSWERED and BUILT) with COMP-b's path on the SAFE-SAVE B2 line. |
| `docs/roadmap.md` (1) | this tree's T4 row (the encoder check built; Part 3 IN PROGRESS) — COMP-b's said "Part 3 PLANNED". |
| `docs/api/README.md`, `all-entities.md`, `calibration_record.md`, `docs/internal/PROJECT-BRIEFING.md` | generated: markers dropped, then REGENERATED (`api_doc_tool.py generate`, `briefing_status.py generate`), never hand-merged. |
| also, not conflicted | `test/build_selection_test.cpp` :6 "The other seven branches" (COMP-b) → "The other six" (seven builds, the host takes one). |

**Gates on the merged tree, before anything new (12:00–12:20):** briefing regenerated; `cmake -S test -B build/test`;
the FULL build (doc gates included) exit 0; suite `2128 | 2128 passed | 0 failed | 3 skipped`, `assertions: 2774535`;
`api_doc_tool` check-fresh 0, check-examples (520 lines, 6 files, 6 whole-run listings), self-test OK, check-coverage
(3339 entities, 165 headers), check-removability; `doc_staleness_audit.py` clean + self-test OK (10);
`briefing_status.py` self-test OK (24), check current after regenerating on the fresh binary; the ARM header gate
over all 202 headers exit 0 (`scratch/t4-3/late/armhdr.sh`); both CI greps clean. **`src_build_gate.py check`: FAIL —
the merge's one SEMANTIC conflict**, in every build: `src/programs/competition_program.cpp` :709/:712 call
`motorClawBootLine(kMechanisms, *st.robot, i)` / `pneumaticClawBootLine(…)` — COMP-b's call sites of the OLD
signature, which this tree's item 5 changed to take the driver file's controls (`late/gate-check1.log`). Step 0
itself passed whole (one chain, 7 builds, 3 robots, 6 programs / 17 entry names, the include root, the printed
paths). That call site is item 5's "COMP-b's call sites re-pointed in the same change" — done next, as part of the
claw words' move; the self-test is run once the tree compiles (a known-red compile would fail its every real-build
case for the same reason).

### RESUMED — Sun Oct 4 01:17 PM EDT 2026 (`date`), a new executor run (opus) in the same worktree
- **Why:** the run above was STOPPED after its 12:20 entry. The tree shows what it did after that entry WITHOUT
  logging it (files modified 12:21–12:35): the claw words moved into `src/common/driver_panel.hpp` with Part 2b's
  PROTECTION row and chooser words (`addProtectionFacts`, `protectionRowText`, the chooser's REFUSED / CUT and line
  1's `dead <ports>`); `test/teleop_claws_test.cpp` renamed (`git mv`) to `test/driver_panel_claws_test.cpp`; the
  include re-pointed in `driver_panel_test.cpp` and `robot_two_driver_test.cpp`; a NEW `src/common/log_copy_line.hpp`
  (the ONE `log copy:` boot line, its test not yet written); `competition_screen.hpp` (the PROTECTION row FIRST on
  the driver page); `competition_program.cpp` (the four-argument `perTick`, `kCutSeconds`, `addProtectionFacts`, the
  provider built with `("comp_log.txt", Append)`, the `log copy:` line, the claw boot lines with the driver file's
  controls, the C5 registration `OnRefusal::DropEntry` and one Warn line per dropped auton); and a library change for
  C5 — `competition::OnRefusal` (`Throw` default, `DropEntry`), `AutonRegistry`'s constructor argument,
  `refusedCount()`, `refusal(i)`, `onRefusal()`, `kMaxAutonRefusals`, and `AutonMenu`'s red status line naming each
  dropped entry in turn. Its scratch: `scratch/t4-3/late/backup/` (PRE-EDIT copies, taken before each edit — not
  mutation backups) and `late/panel_parts.py` with its text pieces.
- **Checked first:** no mutation left applied — every runner backup under `scratch/t4-3/mut/backup/` is byte-identical
  to `6abcac8`'s file (`cmp`; the EC and PV backups differ only in the 09:09/09:53 comment lines, read with `diff`),
  and every file a mutation touched is unchanged since `6abcac8` (`git diff 6abcac8 --stat`: empty for
  `coupled_side_monitor.hpp`, `device_provider.hpp`, `mechanism_table.hpp`; `drive_program.cpp`, `drive_panel.hpp`,
  `log_copy_sink.hpp` byte-identical to their backups). The `late/backup/` copies are the merged index's files (`git
  show :path | cmp`), i.e. what the 12:21–12:35 edits started from. No gate plant: the whole `git diff -- src/
  Makefile tools/` (index and working tree) read; every hunk is a logged resolution or the unlogged work above.
  `pp_pre_t.txt` / `pp_post_t.txt` (empty, at the repository root) come from main (`git ls-tree fab1ebb`), not from
  this chunk — for the coordinator.
- **Kept, every piece of it** (read whole below, against the brief): nothing of the unlogged work is reverted; each
  item is logged in this run's entries with what was finished around it.

### 12:21–15:00 — item 8 (re-read), item 6 (Part 2b), item 3's last part, item 1's wiring, Decision B, C5 *(`date` 15:01 at this entry; the 12:21–12:35 work is the stopped run's, read whole and kept)*

**Item 8 — COMP-b's files re-read (the brief's §1 Part 2 item 8), and what each held:**
- `src/common/driver_panel.hpp` (COMP-b): the controller's three lines (`controllerScreenLines()`), the ONE chooser
  (`controllerWord()`: AUTON ENDING > the last fault > no controller > DEGRADED > CLAW ABSENT > ok — no PROTECTION
  slot, no REFUSED/CUT), the writer (`ControllerLineWriter`, COMP-b's review), `PanelRow` and the controller row. It
  INCLUDED `programs/teleop_claws.hpp` as an interim step for `kLcdClawAbsent`; the claw row, the fault row,
  `anyClawAbsent()` and the claw boot lines stayed in `teleop_claws.hpp` (the COMP re-check's ND3). The refusal
  painter is `src/common/screen_paint.{hpp,cpp}`'s `paintRefusalScreen()` (ND3 (b)); Teleop kept its own.
- `src/programs/competition_program.cpp`: a THREE-argument `perTick` (`DriverPanel::operator()`), pinned WHOLE by
  `test/competition_source_test.cpp` P14 with `view()` and `refreshLcd()`; boot lines from
  `competitionBootLines()`; the claw boot lines through `teleop_claws.hpp`'s OLD signature (P11); its registry
  built to THROW (B15/B18). No "AT BOOT ONLY" / "not in this graph" / "runtime absence" / "dead-port" word anywhere
  in COMP-b's `src/` files (grep over `src/common/` and `src/programs/competition_*`: none) — nothing to make true.
- Worksheet Station K (COMP-b): K.5 drives the program's `opcontrol()`; COMP-b did NOT write THE PUSH CHECK or the
  protections line (grep `push check\|PUSH CHECK\|CUT\b` in Station K: none) — they are this step's (below).
- The build list: eight at main (COMP-t the 7th, COMP-b the 8th); the self-test's pairs 31-32 (COMP-t) and 34-35
  (COMP-b) — merged with this tree's 12-13 retirement (the 11:44 entry).
- `include/shulib/version.hpp` and the changelog: API 2.4 (L9 Part A1); SAFE-SAVE A added `calibration/record_store.hpp`
  and `hal/file_store.hpp` under 2.4 with no bump (additive). The rule the file states (additive = no bump while the
  register row is untouched) is followed below for C5's registry change.
- `teleop_loop.hpp`'s banner: names the driver file (COMP's `opcontrol()`), "shulib Teleop" and the X-drive as
  callers — Teleop's clause is this chunk's to retire (the comment sweep, below).
- COMP-b's reword of `-- in shulib Teleop only`: `-- in shulib Teleop and shulib Competition` (RC5), with three
  word pins. Resolved to this tree's `-- buttons: src/team/robot_two_driver.hpp` at the merge (the 11:44 entry).

**Item 6 (Part 2b) — built:**
- `src/common/driver_panel.hpp`: the PROTECTION row (`protectionRowText(state, cutSeconds)`: `drive OK` green; `drive
  OK -- frozen 13` / `drive OK -- N over-current warning(s)` amber; `DEGRADED dead 18 13[ frozen 14]` amber; `CUT 1 s
  #2: <the log's words without "CUT: ">` red; `REFUSED: N drive motors dead -- 0 V for the run` red; cut at the SMALL
  budget, every %s with a precision — this run added the precisions the stopped run left bare); the chooser's
  REFUSED > CUT words at D4's place (above the last fault, below AUTON ENDING: a state in force outranks a latched
  history); line 1's `dead <ports>` in place of the heading while any is dead; `addProtectionFacts()` (it only ever
  SETS a word, so the boot's own DEGRADED is never cleared). `ControllerFacts` gained `refused`, `cut`, `deadPorts`.
- `src/programs/competition_screen.hpp`: the driver page's PROTECTION row FIRST (it explains a stopped drive), the
  four COMP-b rows one lower; `CompetitionDriverFacts::protection`.
- `src/programs/competition_program.cpp`: `perTick` takes the FOURTH argument (`DriverPanel::operator()`, `view()`,
  `refreshLcd()`), `kCutSeconds` from `RobotTwoDriver::kLoop` (never retyped), `addProtectionFacts()`.
- **COMP-b's pins re-pointed one-for-one** (logged in each case's comment): P2's try anchor (the provider WITH the
  copy) and its register anchor (the DropEntry registry); P9's `ProsFileMode::` count 1 → 2 (both Append); P11 (the
  claw lines from the driver file's controls, the shared header's include, no `teleop_claws`); P14's three bodies
  (the fourth argument, the PROTECTION row, `addProtectionFacts`) and its `, true);` count 2 → 3 (the dropped autons'
  lines); `competition_screen_test` S11/S12 (five rows, the PROTECTION row first; every COMP-b word pin unchanged).
- Still to do (below, after the gate run): the driver file's PROTECTIONS paragraph (§2) and `run()`'s doc, P2-18,
  cookbook 06's re-quote, its protections sentence and the loop recipe's fourth argument; Station K's two lines and
  THE PUSH CHECK.

**Item 3's last part — the claw words MOVED (the stopped run's 12:21-12:23 work, read whole):** every function of
`teleop_claws.hpp` that survived the early step (`clawCartridgeWord`, `clawStopWord`, `adiLetter`, `ClawLine`,
`holdControlFor`, `toggleControlFor`, `motorClawBootLine`, `pneumaticClawBootLine`, `clawTemperatureText`,
`kNoClawsRow`, `clawRowWorstLength` with its two asserts, `ClawRow`, `clawRowText`, `anyClawAbsent`,
`kLcdClawAbsent`, `faultRowText`) is in `src/common/driver_panel.hpp`, its body byte-identical (`diff` of the two
function regions: comments only — the interim notes dropped); `teleop_claws.hpp` deleted (`git rm`);
`test/teleop_claws_test.cpp` RENAMED (`git mv`) to `test/driver_panel_claws_test.cpp`, its include and header
re-pointed, EVERY word pin unedited (P3-T4); `driver_panel_test.cpp` and `robot_two_driver_test.cpp` include the
shared header. COMP-b's include and its two calls re-pointed (the calls take `RobotTwoDriver::kHolds` /
`kToggles` — item 5 for the competition program).

**Item 1's wiring — "shulib Drive" (this run):** `src/programs/drive_program.cpp`: the refusal SCREEN through
`paintRefusalScreen()` (`kDriveRefusedWord`, `kDriveRefusalFooter`, the BUILD time; its serial line `[DRV] REFUSED:
…`), painted at boot and again on the refused `opcontrol()` before it idles; `DriveScreen` reads the touch every
tick (`readTouch()`, the one-line reduction) into `DrivePager` (a flip repaints at once), rebuilds every row at most
every 200 ms (MAIN's PROTECTION and controller rows from the shared header), paints only what
`driveRepaint()` marks (NEW in `drive_panel.hpp`, pure: whole for the first paint, a new page or bar; else the rows,
bar texts and buttons that changed — a row's text OR tone), and writes the controller's lines through the ONE
chooser and the writer every 5 ticks (`addProtectionFacts` included); `st.lcd` (`ProsLineDisplay`); the disable
line gains the screen-print count (Teleop's). The "EARLY STEP" notes are gone from `drive_program.cpp` and
`drive_panel.hpp`. Built and linked: `make ROBOT=tank PROGRAM=drive` exit 0, no warning from `src/`
(`late/arm-drive1.log`).

**Decision B — the competition program's SD copy (the stopped run's, kept):** `st.devices.emplace("comp_log.txt",
ProsFileMode::Append)`; the boot says where it is ONCE through the NEW shared `src/common/log_copy_line.hpp`
(`logCopyBootLine()` — Truncated / Created / Appended / NotOpen, a Warn when it did not open). This run re-pointed
"shulib Drive"'s own `log copy:` literal to the same function (two programs typing it would drift) and wrote its
test (`test/log_copy_line_test.cpp`, LCL1-2). **The coordinator's ruling of 2026-10-04 is applied:** the copy goes
to "shulib Drive" and the competition program ONLY — not the Auton Tester (COMP-t's own SD run log stays its record),
not Calibrate (it constructs no provider), not the X-drive (it refuses at boot on every robot).
**Found, for the coordinator (no HA number taken):** in the competition program the provider's sink is written by
TWO tasks — the auton task's idle loop (`logStatusChange()`, the menu's choice line) and the opcontrol task's driver
loop (the protections' events) — so around a disable both can log at once. Serial (stdout) had that exposure since
COMP-b; the copy adds the card file under the same sink (`hal::BlockCharSink` over `ProsBlockSink`, whose
`droppedWrites()` counter is a plain integer). Whether PROS's newlib serializes two tasks' writes to one `FILE*` is
not measured here and no register entry states it. The program header says so honestly (THE LOGS); a lock, or a
register entry, is the coordinator's call.

**C5 — "drop just that auton" (the team lead's ruling of 2026-10-02):**
- **Why a LIBRARY change** (a delta from §5's "Part 3 changes library TEXT only" — the ruling post-dates the brief):
  the students' `registerAutons()` calls `registry.add()` entry by entry, and the file must stay untouched; while
  `add()` throws, the first refused entry aborts the function and every later entry is lost, whatever the program
  does around it (a try would keep the boot but lose the rest). So `AutonRegistry` gains ONE additive choice: the
  constructor's `OnRefusal` argument (`Throw`, the default — B15 unchanged for every other caller; `DropEntry` —
  nothing registered for a refused entry, `check()`'s sentence kept, `add()` returns), `refusedCount()`,
  `refusal(i)`, `onRefusal()`, `kMaxAutonRefusals` (8). `AutonMenu`'s status line names each dropped entry in RED, in
  turn, 2 s apiece, while the choice is open (`kMenuRefusedPrefix`, `kMenuRefusalTurnSeconds`). No existing
  declaration changed; the default is bit-identical (the whole suite).
- **The competition program:** `st.registry.emplace(kAutonPeriods, competition::OnRefusal::DropEntry)` and the
  boot's lines through the NEW shared `src/common/dropped_autons.hpp` (`sayDroppedAutons()`: one summary line, then
  `DROPPED: <sentence>` per entry, and the count past the kept sentences) — on serial and the SD run log. The
  stopped run's own two-literal version had a `-Wformat-truncation` (`%.191s` into 150 bytes — `late/arm-drive1.log`)
  and is replaced by the shared function.
- **The Auton Tester — the SAME, logged as the choice:** it registers through the same `registerAutons()`; one bad
  entry refused its whole boot, so a student fixing one auton could test none of the others. It now builds its
  registry with `DropEntry` and says the same lines (`logBoth`, serial + its SD run log); its PICK page IS
  `AutonMenu`'s view, so the red line comes with it.
- **Tests that fail on the unchanged code** (each needs the new API, or pins a call the unchanged programs lack):
  `auton_registry_test` "C5 DropEntry …" ×2, `auton_menu_test` "C5 the selector names each DROPPED entry …",
  `dropped_autons_test` DA1-DA2, `competition_source_test` P18, `auton_tester_source_test` "the tester's registry
  DROPS …". The mutations are run below.

**Other tests this run (P3-T2/P3-T3, Part 2b's shared panel):** `driver_panel_test` D6 (the PROTECTION row, every
mode, worst case), D7 (the chooser's full order, all 21 pairs — the brief's "CUT ranked below a fault" mutation's
test), D8 (`addProtectionFacts`: sets only, line 1's dead list cut at 19); `drive_panel_test` "driveRepaint …";
`drive_program_source_test` three new cases (the shared painters only — refusal, repaint plan, one touch read; the
MAIN rows and the controller lines from the shared header with the protections' state; the log copy line) and the
refused path's repaint; `competition_source_test` P17 (the log copy) and P19 (the fourth argument, `kCutSeconds`).
**`test/source_pins.hpp` is now the ONE copy of the pin helpers:** the drive pins' own copy (written in the early
step) is gone — its three extras MOVED there (digit separators; `bodyOf` skipping braces in literals; `squeeze()`,
`sourcesUnder()`). Re-pointed also: `drive_protection_test`'s Part 2 source pin skips `rows_.protection` (the MAIN
page's ROW, words not config).
- **Suite (fresh binary, the new test files configured):** `2146 | 2146 passed | 0 failed | 3 skipped`,
  `assertions: 2776313` (was 2128 at the merge's gate run).

### RESUMED — Tue Oct 6 02:59 PM EDT 2026 (`date`), a new executor run (opus) in the same worktree
- **Why:** the run above was STOPPED after its 15:01 entry. The tree shows what it did after that entry WITHOUT
  logging it (files modified 15:03–15:44, Oct 4): P2-18 (`test/robot_two_driver_test.cpp`) and the driver file's
  PROTECTIONS paragraph and `run()`'s doc (`src/team/robot_two_driver.hpp`, §2 verbatim); cookbook 06's re-quote
  and its protections sentence; the "seven builds" comments (`boot_source.cpp`, `fault_latch_hook.hpp`,
  `xdrive_program.cpp`); `boot_source.hpp` :9-12, `screen_paint.hpp`'s painter note, `mechanism_table.hpp` :114,
  the gate's case-28 comment; C5's words in `src/team/autons.hpp`, `src/team/README.md`, `test/autons_test.cpp`
  (2b), chapter 17, guide 16 and the COMP brief's B15/B24 notes; README's layout block, guides 07/10; the
  worksheet's Part 3 pass (815 changed lines); RESUMING, ORIENTATION, HANDOFF, REMAINING-WORK, build-order, roadmap,
  the changelog, the register; `docs/api` and the briefing regenerated. Its last build (`late/build3.log`, 15:44)
  passed the doc gates and linked; no suite run, no gate self-test, and NO late-step mutation was run (no runner
  under `late/`).
- **Checked first:** the merge is still open (`MERGE_HEAD` = `fab1ebb`, `HEAD` = `6abcac8`), no path unmerged
  (`git diff --name-only --diff-filter=U`: empty). No gate plant or mutation left applied: no `SHULIB_PROGRAM_LIBRARY`
  outside `build_selection.hpp`'s RETIRED comment, no `"../"` / `"./"` include in `src/`, and every `src/`,
  `include/`, `test/`, `tools/` hunk since the 15:01 entry read whole (comment, document or test text only — the
  list above). `pp_pre_t.txt` / `pp_post_t.txt` are main's (`fab1ebb`), as logged.
- **Kept, every piece of it**, each item checked against the code in this run's entries below; nothing reverted
  unless an entry says why.

### 14:59–15:50 (Oct 6) — the stopped run's 15:03–15:44 work, read whole against the code and the brief; the fresh binary *(`date` 15:50 at this entry)*
- **The fresh binary first:** the briefing regenerated; `cmake -S test -B build/test`; `autons.cpp` touched so the
  link is fresh; the FULL build (doc gates included) exit 0 (`late/build4.log`); suite
  `2148 | 2148 passed | 0 failed | 3 skipped`, `assertions: 2776343` (`late/suite2.log`; +2 since the 15:01 entry:
  P2-18 and autons (2b)).
- **Item 6's remaining pieces — DONE by the stopped run, checked here:** the driver file's PROTECTIONS paragraph is
  §2's VERBATIM (read line by line against the brief :2113-2118) and `run()`'s doc gains §2's fourth-argument clause
  (`src/team/robot_two_driver.hpp` :31-36, :102-103); **P2-18** (`test/robot_two_driver_test.cpp`, the last case):
  the paragraph read from the file and every number checked against the config `kLoop` runs with — "2.4 A" and "250
  ms" against `overCurrentAmps` × `overCurrentTicks` × the tick, the fight's window against
  `SideMonitorConfig{}.persistTicks`, "1 s" against `cutTicks` × the tick (and pinned WHOLE, so a 99-tick cut is not
  hidden by `%.0f`), "more than two" against `kMaxDeadTotal` (N7 (c): the per-side floor's "three" is not in the
  paragraph), "the rest of the run" present and "until the field disables" absent; cookbook 06's "How it runs"
  listing RE-QUOTED (:264-267, `check-examples` green) and one protections sentence with its numbers (:281-287); the
  loop recipe's fourth argument was already Part 2a's (cookbook :68-73).
- **Station K (the worksheet, COMP-b's station):** K.0.4 (the wall for K.8), K.2.2 (the log copy's line and C5's
  DROPPED lines), K.2.3 (J.3.1's re-quoted claw line end), K.2.4's ONE line on what a CUT / a FROZEN port / a dead
  port looks like in this program, K.2.5, K.5.4, K.7.2 (`comp_log.txt`), and **K.8 THE PUSH CHECK** (N1; the
  re-check's item 11: the PEAK line read after letting go, never the disable line; HA-176's launch and reversal), "What
  to send back" 6–8 and the end-of-session list. Every quoted word was grepped in `src/` and `include/shulib/` (the
  15:50 check below).
- **The worksheet's Part 3 pass (§4 Part 3) — DONE by the stopped run, read whole here:** the header's program list
  and the FINAL slot map (with the stale-brain instruction: re-upload the tester to slot 2, delete slot 7); **Station
  E** a pointer; **Station F REWRITTEN** for the new "shulib Drive" — the forward note, **F.0.1 OLD or NEW**
  (`adapters, NOT the motion stack` / `[R3B]` vs `[DRV]`), F.0.2's refusal screen and its four headlines, **F.1**
  MAIN / DETAILS / MORE row by row and the controller's three lines, the serial-only lines, the WARNING and CUT
  paragraphs moved from E and reworded for the library, F.2 wheels up, F.3 → J, F.4 the ground, **F.5** the feel
  against Bench Tests' 10 DRIVE with its four differences, **F.6** (i) the pulled cable (HA-175), (ii) THE PUSH
  CHECK (HA-176), (iii) no staged fight, **F.7** the disable line and the card's `drive_log.txt`; **Station G** G.0.4,
  G.1 (three uploads; the RETIRED `PROGRAM=library` note), G.4's slip note, G.6 "shulib Drive" from the record;
  **Station J** moved to slot 1 (header, J.0.1, J.3.1 re-quoted with `[DRV]` and `-- buttons:
  src/team/robot_two_driver.hpp`, **J.3.2 RETIRED** with its reason, J.3.3 on DETAILS, J.4's "where the buttons live",
  every "slot 2" → "slot 1", J.9); **Station M** slot 2 (the stale slot-7 note); the DO NOT list. **Checked against
  the code:** 34 quoted strings grepped in `src/`/`include/shulib/` (the refusal word and footer, the four headlines,
  `This is NOT the old shulib Drive`, the DETAILS run-status row, MORE's row formats (`drive_panel.hpp` :320-386), the
  controller's `LIB`/`hdg`/`dead` lines, the log copy's two forms, the disable and peak lines, the CUT / re-arm /
  `went ABSENT` / `degradation re-evaluated` / `REFUSED:` lines (`drive_protection.hpp` :360-908), the dead-port boot
  lines, the driver line, `opcontrol(): robot two's driver file is live`): every one present.
- **C5's documents (the computed task's list) — DONE by the stopped run, checked here:** chapter 17 (the Covers line,
  the schedule section's DROP paragraph, the boot lines' order as `competitionInitialize()` logs them — the absent
  mechanisms, the dropped autons, the log copy, the claws —, the driver page's five rows with the PROTECTION row's
  table and the chooser's order, the SD card's two files, the refusals list without "an auton refused at
  registration" and its new "not a refusal" paragraph, "What it does not do yet"'s protections bullet); Station K
  K.2.2; the COMP brief's B15 and B18 (amended) notes (:704-709, :810-811 — the old text kept, the new rule in a dated bracket); guide 16 :128-132; `src/team/autons.hpp`
  :7-11, :28-31 and `src/team/README.md` :7-9; `test/autons_test.cpp` (2) re-worded and **(2b) NEW** — the REAL
  students' file into a DropEntry registry with a 28 s match period: both entries dropped, in order, by name,
  nothing thrown; with the game manual's periods nothing dropped.
- **The plan documents (own lines only) — DONE by the stopped run, checked here:** RESUMING :114-140 (seven builds;
  the RETIRED note), ORIENTATION :94-106/:138-139, HANDOFF's program list :278-292 and L5b's new bullets :612-627,
  REMAINING-WORK row 10 and §10's Station J line, build-order's T4 status and the first `**Next:**` line, roadmap
  :1724-1727 (COMP-b's row) and :1794-1799 (T4's), the changelog's C5 entry; the register's HA-153 test path
  (renamed file) and HA-176's K.8. **One more line here:** build-order's first `**Next:**` still said "the team lead's
  "D, again" open" in T4 Part 2a's clause — now "ANSWERED 2026-10-01 — the encoder check, built in Part 3" (the
  briefing regenerated).
- **Comments (the stopped run's; read):** "seven builds" in `boot_source.cpp` :12, `fault_latch_hook.hpp` :17-18,
  `xdrive_program.cpp` :35/:37/:39, `auton_tester_program.cpp` :51; `boot_source.hpp` :9-14 (the hand-over's
  :9-12), :87-96; `screen_paint.hpp` :16-19 (three programs, one painter); `mechanism_table.hpp` :114;
  `record_store.hpp` :668-670 and :740-743 (SAFE-SAVE A's two COMMENTS that named Teleop — no sentence of
  `formatSaveReport` names a drive program, landmine 25 holds); the gate's case-28 comment; `auton_tester_program.cpp`
  :189, :520 (its :316 quotes a past comment — kept as history). **This run re-wrapped** three comment lines the
  stopped run left long (`auton_tester_program.cpp` :189-193, `competition_program.cpp` :80-82, `main.cpp`
  opcontrol()'s doc :258-262) — words unchanged.

### 15:50–16:25 (Oct 6) — the late step's MUTATIONS, two survivors closed *(`date` 16:25 at the last run's end)*
**The runner:** the early step's (`scratch/t4-3/mut/mutate.py`, the late ids in `mut/mutations_late.py`, every id
`L…`): back up, ONE exact replacement asserted to occur once, rebuild in the scratch build dir `mut/mbuild`, the FULL
suite, restore, `cmp`, timestamp bump; results appended to `mut/mutation-results.txt`, each run under `mut/runs/`.
**The stale-binary trap, met and logged:** `mbuild` held the EARLY step's binary (`1983` cases); it was re-configured
from this tree and FULLY rebuilt (203 TUs) and run green (`2148 | 2148`) before any mutation (its first build
stopped at the doc gate: this run's own build-order line had made the briefing stale — regenerated). **Seven
BUILD FAILED verdicts were not reds and were re-run** (logged in the results file): five because `docs/api` carries
header LINE NUMBERS (a mutation that adds or drops a line in `include/shulib/` fails `check-fresh` before any test
compiles; and once a mutation run leaves `mbuild`'s last suite red, the briefing's generated suite line goes stale
for the next doc-gate run) and two because a re-typed line left a parameter or a local unused (`-Werror`) — re-run
through `mut/mutate_fast.py`, the same runner building `shulib_tests/fast` after the depend step (no doc gates, as
`scratch/t4-3/fastbuild.sh`), with LRG4 / LPR2 / LDR3 re-written so every name stays used. Every verdict below is a
FULL suite on a fresh binary. After the runs, every mutated file `cmp`-identical to its backup and its target present
exactly once (43 of 43).

| # | Mutation | Red (FULL suite) |
|---|---|---|
| LRG1 | C5: DropEntry keeps the sentence but still THROWS | 6: `auton_registry_test` C5 ×2, `auton_menu_test` C5, `autons_test` (2b), DA1, DA2 |
| LRG2 | C5: a dropped entry's sentence not kept | 5: the same less DA2 |
| LRG3 | C5: `refusedCount()` stops counting at the kept 8 | 2: C5 "each dropped sentence in order … counted past the cap", DA2 |
| LRG4 | C5: the constructor's `OnRefusal` ignored (always Throw) | 6: as LRG1 |
| LRG5 | `refusal(i)` past `refusedCount()` reads its slot instead of `""` | **SURVIVED — EQUIVALENT, not counted:** a slot at or past `refused_` is never written (`add()` writes slot `refused_`, then counts it), so it is the zero-initialised `""` either way; the guard is kept as the contract's statement |
| LMN1 | the menu's status line never names a dropped entry | 1: `auton_menu_test` "C5 the selector names each DROPPED entry …" |
| LMN2 | the status line names only the FIRST dropped entry | 1: the same |
| LMN3 | the dropped entry's line amber, not red | 1: the same |
| LMN4 | a dropped entry's line hides LOCKED once the auton started | 1: the same |
| LDA1 | only the first dropped entry's `DROPPED:` line said | 2: DA1, DA2 |
| LDA2 | the "N more" line missing when ONE more was dropped than kept | **SURVIVED** (DA2 drops three past the cap) → **NEW DA3** (exactly the cap: no "more" line; one past: `DROPPED: 1 more, …`) → re-run on a fresh binary: **RED**, 1: DA3 |
| LDA3 | a line emitted when nothing was dropped | 2: DA1, DA2 |
| LCP1 | the competition program's registry built to THROW (the unchanged code's form) | 2: `competition_source_test` P18, P2 |
| LCP2 | the competition program never says which autons it dropped | 2: P14, P18 |
| LAT1 | the Auton Tester's registry built to THROW (the unchanged code's form) | 1: `auton_tester_source_test` "the tester's registry DROPS …" |
| LAT2 | the Auton Tester never says which autons it dropped | 1: the same |
| LCH1 | **CUT ranked BELOW a fault in the chooser** (the brief's named P3-T2 mutation) | 1: `driver_panel_test` D7 (every pair) |
| LCH2 | REFUSED ranked below CUT | 2: D7, D8 |
| LCH3 | REFUSED ranked above AUTON ENDING | 1: D7 |
| LPR1 | the PROTECTION row's CUT amber, not red | 2: D6, `competition_screen_test` S11 |
| LPR2 | the PROTECTION row's cut length typed in (1 s), not the config's | **SURVIVED** (every D6 check handed in 1 s) → **D6 gains a 0.5 s and a 2 s cut** → re-run: **RED**, 1: D6 |
| LPR3 | the row keeps the log's `CUT: ` | 2: D6, S11 |
| LPR4 | the DEGRADED row drops a frozen member | 1: D6 |
| LPR5 | the Driving row never says a reading froze | 1: D6 |
| LPR6 | the REFUSED row counts one side's dead, not the total | 2: D6, S11 |
| LAF1 | `addProtectionFacts` CLEARS the boot's own DEGRADED | 1: D8 |
| LAF2 | `addProtectionFacts` drops the dead ports | 1: D8 |
| LAF3 | `addProtectionFacts` says CUT while merely DEGRADED | 1: D8 |
| LCL1 | line 1's dead list not cut at 19 columns | 1: D8 |
| LDR1 | `driveRepaint` ignores a row whose TONE changed | 1: `drive_panel_test` "driveRepaint …" |
| LDR2 | `driveRepaint` never repaints whole on a page flip | 1: the same |
| LDR3 | `driveRepaint` forgets a row that went away | 1: the same |
| LCS1 | the PROTECTION row second on the competition program's driver page | 1: S11 |
| LCP3 | the competition program's cut length RETYPED (`kCutSeconds = 1.0`) | 1: P19 |
| LCP4 | Decision B: its copy TRUNCATED each boot | 3: P17, P2, P9 |
| LCP5 | its controller never told of REFUSED / CUT / dead | 2: P14, P19 |
| LDP1 | "shulib Drive"'s refused `opcontrol()` does not repaint the refusal | 2: `drive_program_source_test` (drives ONLY through run(); paints through the shared painters only) |
| LDP2 | "shulib Drive"'s cut length RETYPED | 1: `drive_program_source_test` (MAIN rows and controller lines …) |
| LDP3 | "shulib Drive"'s controller never told of REFUSED / CUT / dead | 1: the same |
| LLC1 | the log copy line's CREATED and APPENDED swapped | 1: `log_copy_line_test` LCL1 |
| LLC2 | a copy that did not open said as INFO | 1: LCL1 |
| LPP1 | **the driver file's paragraph says "2.5 A"** (the brief's named P2-18 mutation) | 1: P2-18 |
| LPP2 | the paragraph's stop shortened to "until the field disables the robot" | 1: P2-18 |

**Stated, not run:** the re-pointed `drive_protection_test` source pin (it skips `rows_.protection`, the MAIN page's
ROW, as it already skipped `rows_.protections`; a re-point, not a new claim); the claw words' move (P3-T4: every word
pin of `teleop_claws_test` unedited in `driver_panel_claws_test.cpp` — the early step's CL1-CL3 ran on the same
functions, byte-identical bodies).

**Tests added by this run:** `dropped_autons_test` **DA3** (above); `driver_panel_test` **D6** two checks (a 0.5 s
and a 2 s cut). **Suite (fresh FULL build with the doc gates, `late/build5.log`):** `2149 | 2149 passed | 0 failed |
3 skipped`, `assertions: 2776350` (`late/suite3.log`).

### 16:25–17:20 (Oct 6) — a claim corrected, the prose review, `T4-COMPLETED.md` *(`date` 17:20 at this entry)*
- **A claim of the 15:01 entry was NOT true, and is made true here:** it said the competition program's header
  "says so honestly (THE LOGS)" — that more than one task writes the provider's sink. The header said nothing of
  it (grep: no hit). THE LOGS now says it (`competition_program.cpp` :66-71): the auton task, the PROS phase tasks
  and the opcontrol task's driver loop all log through the one sink, so two can write at once — on serial since
  COMP-b, into the card copy since this part — and that PROS's serialization of two tasks' writes to one `FILE*`
  is measured nowhere and stated in no register entry (left for the coordinator; no HA number taken). A comment:
  the gate is re-run below.
- **The prose review (run and its hits read BEFORE this entry):** the brief's §4 Part 3 terms over `README.md`,
  `docs/*.md`, `docs/guide/`, `docs/cookbook/`, `docs/internal/*.md`, the worksheet, `src/` and `include/shulib/`
  (generated `docs/api/` excluded) — "eight builds" / "all eight" (0 / only other eights: motors, RoutineResult's
  fields, the record's starts) and "eighth build" (COMP-b's ordinal at its landing — history in build-order, the
  roadmap's COMP row, HANDOFF L2 and the gate's docstring), "six builds" (dated history), "slot 7" (guide 16's
  stale-brain note, dated plan history, the Makefile's "slot 7 until then"), "teleop_claws" / "teleop_program" (each
  hit "(deleted at T4 Part 3)", "moved from", or a quoted past comment), "PROGRAM=library" / "tank-library" (the
  RETIRED notes, dated history, the gate's RETIRED list and case 12), "kTeleopRezeroButton" (HANDOFF L5's history),
  "controlsMatchTable" / "TeleopClaws" / "clawBannerLine" (no live hit), "Six programs" (0), "shulib Teleop" /
  "Teleop" over `src/` and `include/shulib/` (every hit past tense or history: the boards' "(Teleop's)" origins,
  `runTeleopLoop` named as deleted, the 2026-09-25 runs, the ruling's own words in `drive_protection.hpp` :8,
  `stick_mapping.hpp`'s generic "teleop"), "drive_log" / "comp_log" (12 / 12, each the file named where it is
  written or read), "runDrive", "adapters, NOT the motion stack" (F.0.1's OLD-program tell, `drive_program.hpp` :12,
  Bench Tests' own DRIVE rule), "both files" / "there too" (other meanings: the record's two files, guide 16's
  schedule), "AT BOOT ONLY" / "not in this graph" / "no drive protections" (0), "runtime absence" (the library's
  own header), "five per side" (`sim/drive_plant.hpp` :222, the A2 plant's capacity — not T4's), "Part 1b" (the
  plan's fold notes), "TeeCharSink" (0); and C5's: "refuses its boot" / "refused at registration" / "add() throws"
  (no hit states the old rule: the Auton Tester's boot refusal on a failed chain proof, the boot source's, the
  registry's own banner, the changelog's REGISTRATION sentence — still true: an entry IS refused at registration,
  and dropped); "OnRefusal" / "DropEntry" / "PROTECTION row" / "push check" / "log copy" (each hit true of the tree).
- **`docs/internal/chunks/T4-COMPLETED.md`** written from this log (the brief's §4 Part 3): what T4 delivered by
  part, Part 3's two steps, the deltas from the brief with reasons, the tests and the mutations that proved them, the
  documents, what is NOT done or claimed (nothing has run on a robot; the review round, the verification and the
  commit are still to come; B2's BACKUP word; port 18's gaps), the coordinator's two items, and the gate lines. It
  says BUILT, not landed: the briefing lists T4 among the completed records the moment the file exists, as it did
  for COMP-b (whose executor wrote `COMP-COMPLETED.md` before its review round).

### THE LATE STEP STOPS HERE — the final gates on the merged tree *(`date`: Tue Oct  6 05:30:31 PM EDT 2026)*
- **Host:** the briefing regenerated, `cmake -S test -B build/test`, the FULL build with the doc gates exit 0
  (`late/build6.log`): `[doctest] test cases: 2149 | 2149 passed | 0 failed | 3 skipped`, `assertions: 2776350 |
  2776350 passed` (`late/suite4.log`); the briefing regenerated again after the binary, and `check` current.
- `src_build_gate.py self-test`: **OK (211 detector cases, each against a real build)**; `check`: **PASS — all 7
  builds link** (bench, xdrive, tank, tank-drive, tank-calibrate, tank-autontest, tank-competition), 0 disallowed
  warnings, every beacon and program asserted, one chain, the include root, every printed path
  (`late/gate-selftest4.log`, `late/gate-check4.log`). Run AFTER the last `src/` edit (the THE LOGS comment).
- The ARM header gate over all 202 `include/shulib` headers: exit 0. The CI greps: PROS-free and sim-free.
- `api_doc_tool.py` check-fresh exit 0, check-examples (521 quoted lines, 6 files, 6 whole-run listings), self-test
  OK, check-coverage (3348 entities, 165 headers), check-removability; `doc_staleness_audit.py` clean and its
  self-test OK (10); `briefing_status.py` self-test OK (24) and check current.
- **The merge is still OPEN and nothing is committed** (`MERGE_HEAD` = `fab1ebb`, `HEAD` = `6abcac8`); no
  path unmerged. The four NEW files of the late step are marked intent-to-add (`git add -N`:
  `src/common/dropped_autons.hpp`, `src/common/log_copy_line.hpp`, `test/dropped_autons_test.cpp`,
  `test/log_copy_line_test.cpp`) so a commit cannot leave them behind. No mutation or gate plant left applied (43 of
  43 mutated files `cmp`-identical to their backups; every file the gate's self-test touched differs from the index
  only by this step's own logged edits). The tree waits for the ONE review round, the coordinator's fresh
  verification and the commit.

## T4-3 review findings and their disposition

- **Started:** Wed Oct  7 11:51 PM EDT 2026 (`date`), a new executor run (opus) in the worktree
  `.claude/worktrees/t4-3` (branch `t4-3`), the merge still open (`HEAD` `6abcac8`, `MERGE_HEAD` `fab1ebb`, no path
  unmerged). Do not commit; no `pros`; no `/dev/ttyACM*`. Scratch, probes and backups under
  `.claude/coordinator/scratch/t4-3/fix1/` only.
- **The task:** three reviewers (correctness, tests, docs) returned 17 findings (two overlap: T4-COMPLETED.md untracked,
  named by the tests lens and the docs lens). Each is re-checked below, then fixed at the root or answered with
  evidence; every gate re-run on the final tree.
- **Checked first:** no file under the worktree (outside `build/`, `bin/`, `.d/`) is newer than the late step's stop
  (2026-10-06 17:30; `find -newermt`: none), so no reviewer's probe or mutation touched this tree (their copies are
  under `scratch/t4-3/review-*/`).

### RESUMED — Thu Oct 8 09:24 AM EDT 2026 (`date`), a new executor run (opus) in the same worktree
- **The stopped run (Wed Oct 7 23:51 – Thu Oct 8 ~01:02) logged nothing past "Checked first" above.** What it did was
  reconstructed, not assumed: the two reviewers' copies of this tree (`scratch/t4-3/review-testgate/wt` and
  `review-correctness/r2/wt`, taken 2026-10-07 23:20 / 23:48 — `diff -rq` between them: EMPTY, so either is the late
  step's tree) diffed against the worktree: 48 files changed, saved whole as `scratch/t4-3/fix2/stopped-run.diff`
  (4,030 lines) and READ WHOLE. Its scratch, `scratch/t4-3/fix1/`: `pre/` (its backups of 13 files before editing),
  `build1-3.log`, `suite1.log` (2168 | 2167 passed — the new e2e case counted its lines in the wrong sink; fixed in the
  test at 00:45:56: the latch logs to the rig's `faultSink`), `suite2.log` (**2168 | 2168 passed**, 00:46),
  `gate-check1.log` (the src gate **PASS, all 7 builds link**, 00:48 — after its last code edit), four prose-review
  grep dumps (`pr_*.txt`), and a mutation runner with NO mutation defined and none run (`fix1/mut/backup/` and
  `runs/` empty).
- **No mutation or plant left applied:** every difference from the reviewers' baseline is one of the stopped run's
  intended fixes (read whole, each tied to a finding below); the late step's own 43 mutation backups were already
  `cmp`-verified at its stop. Relinked from source: `cmake -S test -B build/test`, the briefing regenerated, the FULL
  build with the doc gates exit 0 (`fix2/build1.log`), the suite **2169 | 2169 passed | 0 failed | 3 skipped**,
  `assertions: 2776607` (`fix2/suite1.log` — 2149 + the stopped run's 19 new cases + this run's LCL5).
- Backups, probes, mutation runs and greps of THIS run: `scratch/t4-3/fix2/` only.

### 09:24– (Oct 8) — the seventeen findings, each re-checked, then fixed at the root or answered
*(F1–F17 in the order the reviewers gave them; the tests lens's and the docs lens's "T4-COMPLETED.md untracked" are one
finding, F7 = F17's first half. Mutation ids (FX, RS, SP, SC, SA, RV) are in the table at the end of this entry.)*

**F1 (major) — the SD copy got a line and a flush on EVERY overrunning tick of an auton. REAL; fixed at the root.**
Re-checked in the code before the fix: `LoopMonitor::tick()` raised `LOOP_OVERRUN` per overrunning tick,
`FaultLatch::raise()` logged one Error line per raise, the Robot's latch logs to `devices.telemetry()` — the
provider's `LogCopySink` — and `BlockCharSink` writes and flushes per line; the scheduler's `overrun attribution:`
line (D-3) went the same way, per tick. The reviewer's probe shape (50 lines in a 1 s turn at a 20 ms tick) is now
host test **1r** (`pros_device_provider_test`: the REAL provider with the competition program's Append copy and
the REAL Robot through the shim) — it pins ONE `fault=LOOP_OVERRUN` line on the card while the monitor and the
latch still count every slow tick, and the 10 ms control says nothing. **The fix (the reviewer's first option, the
root):** a slow loop is one EPISODE — `LoopMonitor` logs only an episode's first slow tick (`raise()`), the rest
through the new `FaultLatch::raiseRepeat()` (latched, counted, tallied, stamped exactly as `raise()`, NO line — except
the run's FIRST fault, which is always said); a tick under the budget, or `reset()`, ends the episode;
`overrunEpisodes()` counts them; the scheduler's attribution line fires once per episode with it (every later slow
tick's phases ride its records' `tickPhase`). Why not the other two options: buffering the copy until a request's
terminal word would hold a CUT or a dead port off the card until the auton ends (a reboot mid-match loses it), and a
rate limit drops lines silently — the episode rule loses no information (`n=` carries the silent count). **Stated,
not hidden:** an INTERMITTENT overrun (slow ticks between healthy ones) is a new episode each time, so a loop slow
every few ticks still says each one — exactly as before, no worse (loop_monitor.hpp, HA-60, the changelog say so;
a hysteresis that merges near episodes is the coordinator's option if Station K's copy shows it). The per-call
guards (`finite_guard.hpp`, the plausibility guard's command and wheel-volt audits) still log each offending call
— each is a defect, not a healthy run's state; `fault.hpp`'s header says so (its earlier sentence "the rest of the
library's monitors were already edge-triggered" was too broad and is corrected). Tests: `fault_test` ×3
(raiseRepeat counts with no line; a first-fault repeat IS said, also after `clear()`; a throwing sink), 
`loop_monitor_test` (a stretch logs one line; a healthy tick and `reset()` each end it; AT the budget counts),
`run_report_e2e_test` "C5 e2e (T4-3)" (two slow ticks: one LOOP_OVERRUN line, one attribution line, two counted),
1r. Mutations FX1–FX8. Documents: HA-60's second-consumer note (the copy IS a card write in the tick; "at least
VISIBLE" now says the copy's own overrun would be said once), the changelog (a BEHAVIOUR CHANGE entry; `raiseRepeat`
and `overrunEpisodes` added), guide 11 (the fault line and the attribution line, once per slow stretch),
`diagnostics-plan.md` D-3, chapter 17 and worksheet K.7.2 (no line per slow tick).

**F2 (minor) — a "shulib Drive" boot refused AFTER the provider truncated `drive_log.txt` to 0 bytes with no reason.
REAL; fixed.** The header's "everything from the provider's construction on is in the SD copy too" was false for the
three refusals after the provider (the validate verdict, the driver file's bind, a `PreconditionError`). Now each goes
through `refuseBuilt()`: the refusal is said THROUGH the provider's sink first (`src/common/log_copy_line.hpp`'s
`logBootRefusal()`: `REFUSED: <headline> -- <why>` at ERROR, broken at spaces into pieces TermSink never cuts, each
later piece starting `... `, and a closing line if a word longer than a piece stopped it), THEN the graph released
(binding first), then the screen painted with no serial line of its own (`.logToSerial = false`, so serial does not
say it twice). The competition program had the same gap (its four refusals after the provider; `comp_log.txt` never
said why a boot refused) and gets the same `refuseBuilt` lambda. A refusal decided BEFORE the provider (the boot
source's) is serial-only and the card keeps the previous boot's file — the header now says that too. Tests: LCL4,
**LCL5** (this run: a word longer than a piece stops the refusal and SAYS so — the closing line was unpinned),
P3-T3 (T4-3) (the order: log, unbind, release, paint without the serial line; every refusal after the provider through
it), P2's re-pinned refusals in `competition_source_test`. Mutations FX16–FX18, SP1–SP3, SC1. Documents: worksheet
F.1's refusal paragraph and F.7.2, chapter 17's log-copy paragraph.

**F3 (minor) — "did NOT open (no SD card?)" with the card in. REAL; fixed.** The copy's open state is now mapped by
one shared function, `logCopyOpenFrom(open, cardIn, append, created)` (`log_copy_line.hpp`), and each program asks
the card ONCE at its boot line (`pros::c::usd_is_installed()`, HA-122's belief (1), confirmed on hardware
2026-08-18): `NoCard` says `did NOT open: NO SD CARD`, `NotOpen` says `did NOT open, though an SD card is in (full?
write-protected? a folder of that name?)`; "refused so far" became "lost so far" (it now also counts the F5 guard's
lines). Tests: LCL1 re-pinned, LCL2 over the new state, **LCL3** (the mapping, both programs' modes), the two source
pins (`usd_is_installed` asked once in each). Mutations FX14b, FX15b (FX14/FX15 were red only at the BUILD — an
unused parameter under `-Werror` — which proves nothing of the tests, so each was re-aimed with the parameter
voided), FX19, SP5, SC3. Documents: worksheet F (the serial-only paragraph) and K.2.2, chapter 17.

**F4 (minor) — the Auton Tester's behaviour changed (C5's drop, PICK's red line) and documents credited it to the
team lead. REAL as to the credit; the behaviour is kept and put to him.** COMP-COMPLETED.md :114 records the ruling
for the competition program's boot; the tester's drop came from the late step's computed task ("the same for the
Auton Tester if it shares the registration path" — the brief's re-check table :1900 says the same), i.e. the
COORDINATOR's extension. The code is sound and a student fixing one auton can still test the others, so it is kept;
every credit is corrected to say whose it is: guide 16, `REMAINING-WORK.md` row 10 (now ASKS the team lead to confirm
or reject it), `src/common/dropped_autons.hpp`'s header, `src/team/autons.hpp`, `auton_registry.hpp`'s banner (it said
the team lead ruled "for the programs that run the team's autons"), `auton_tester_program.cpp`'s comment, HANDOFF's
Part 3 line, build-order's T4 status and its Next block, `T4-COMPLETED.md` (the C5 bullet and delta 7, which now says
the tester's behaviour CHANGED although the brief's §5 lists COMP-t's program OUT). **Correction to the record:** the
review lens's premise that the tester's behaviour is unchanged was not true of this tree; T4-COMPLETED says so.

**F5 (minor) — several tasks write one `FILE*` on the card; `BlockCharSink`'s counter unsynchronised. REAL (the late
step's own flag, confirmed); the CARD half fixed, the serial half left to the coordinator.** `diag::LogCopySink` now
takes a TRY-lock (`std::atomic_flag`) for the copy half of `log()` and `summarize()`: a line that finds another task
mid-copy goes to the primary (serial), is left off the card and counted (`contendedLines()`, added into the
provider's `serialCopyDroppedLines()`); the flag is released by a scope guard even past a throwing copy. **Added
by this run (10:12):** a count said only on the boot line would hide a gap that opens mid-run (the stopped run's
chapter 17 and K.7.2 sentence — "the boot line's `lost` count includes them" — was true only of lines lost before
the boot line, i.e. never of these), so the next line or summary that reaches the copy is preceded there by ONE Warn
line, `[DIA] copy: <n> line(s) left off here while another task was writing it -- serial has them`
(`LogCopySink::kGapTag`; the mark it counts from is touched under the guard only); chapter 17 and K.7.2 now say
that. (The gate run started at 10:11 was stopped for it, before its build step, and re-run whole.) So only one
task at a time ever touches the card's `FILE*` and `BlockCharSink`'s counter, and NO task waits on another's card
write (a blocking mutex would let one task's slow card write stall the driver loop — the reason it is a try-lock).
The sink is now neither copied nor moved (it holds the flag). Serial (the primary TermSink over stdout) has had the
same two-task exposure since COMP-b and is unguarded: whether PROS's newlib serializes two tasks' writes to stdout is
a belief no register entry states — **left for the coordinator (a new HA number is his), unnumbered here**, as the
register's T4 Part 3 status sentence says. The guard's own atomicity rests on the compiler's ARMv7 exclusive
load/store pair for `atomic_flag` on the brain's one core (a C++ guarantee, never run there). Tests: LC-7 (a copy that
re-enters the sink mid-line, exactly where a preempting task would: the primary gets both, the copy one, both the
line and the summary counted, the guard released after, the gap line once on the copy and never on the primary),
LC-8 (the gap said before a summary too; a second gap counts only its own), 1s (the provider's sum). Mutations
FX9–FX13, FXG1–FXG5. Documents: the
changelog, chapter 17 and K.7.2 (two lines at once: the card takes one), the competition program's THE LOGS.

**F6 (minor) — "DROPPED: none is on the menu" read as an empty menu. REAL; fixed:** `N auton(s) REFUSED at
registration and DROPPED -- the dropped ones are NOT on the menu (each named below); every other auton and driver
control still run -- fix src/team/autons.cpp` (181 bytes at 16 drops, under TermSink's 200). DA1 re-pinned; DA2 holds
the length. Mutation FX21. Documents: chapter 17, worksheet K.2.2.

**F7 / F17 (first half) — `T4-COMPLETED.md` was untracked. REAL; fixed:** `git add -N` (the stopped run did it;
`git status` shows ` A docs/internal/chunks/T4-COMPLETED.md`). The hand-over's list of NEW files is now FIVE:
`src/common/dropped_autons.hpp`, `src/common/log_copy_line.hpp`, `test/dropped_autons_test.cpp`,
`test/log_copy_line_test.cpp`, `docs/internal/chunks/T4-COMPLETED.md` — all intent-to-add (this round added no file).

**F8 (minor) — "shulib Drive"'s rows, partial paint and generated boot lines unpinned (RS1–RS10 survived). REAL;
fixed at the root, the reviewer's first option.** The decisions moved out of `drive_program.cpp` (compiled by no host
test) into `src/programs/drive_panel.hpp`, pure and host-tested: `driveSourceRows()` (the source rows and their tone),
`DriveLiveFacts` + `driveLiveRows()` (every live row from plain facts, the protections' state and the two groups),
`driveLineToPaint()` (what a partial repaint paints; a row that went is erased), `driveProtectionsLine()` (the
boot's protections line, generated from the configs and checked against `DriveProtectionConfig{}` and
`SideMonitorConfig{}` word for word, and against a changed config), `driveDegradedBootLine()`, `driveBootLevel()`.
The program now only READS the robot into `DriveLiveFacts` and calls them; its calls are pinned WHOLE
(`refreshRows()`'s body squashed, as P14 pins the competition program's). Tests: `drive_panel_test` P3-T2 (T4-3) ×5,
`drive_program_source_test` P3-T3 (T4-3) ×2. Mutations RS1–RS12 (the review's ten re-aimed at the moved code, plus
the cut length and the estimate's x/y), SP4.

**F9 (minor) — `refusal()`'s cap guard unpinned; the check passed on undefined behaviour. REAL; fixed:** the check
is now by ADDRESS — `refusal(kMaxAutonRefusals)` must be the same pointer as `refusal(99)` (the guard's one `""`)
and must not point inside the registry. Mutation RV1.

**F10 (minor) — `AutonMenu`'s turn clamp unpinned. REAL; fixed:** a new case drops `kMaxAutonRefusals + 1` entries
and walks two whole cycles at half-turn steps: every status line is `REFUSED auton 'late …`, never the bare prefix,
the kept eight each named, the ninth never. Mutation RV2.

**F11 (minor) — three smaller unpinned choices. REAL; each fixed:** (a) D6 — frozen AND warnings at once names the
FROZEN port (`drive OK -- frozen 13`, amber); (b) `static_assert(kMenuRefusalTurnSeconds == 2.0)` in the menu test,
naming the documents that say 2 s; (c) D8 — a facts object that starts REFUSED and CUT keeps both through
`addProtectionFacts` and the controller says `REFUSED see brain`. Mutations RV3, RV5 (red at the BUILD — the
`static_assert`, which IS the pin), RV14.

**F12 (major) — HA-161's and HA-143's sources still pointed at the old program. REAL; fixed.** HA-143: "Calibrate,
the old shulib Drive (until T4 Part 3) and the tester's DRIVE station". HA-161 half (a): the old
`drive_program.cpp` text named as gone at T4 Part 3, and the belief re-pointed to where it is written now —
`calibrate_program.cpp` :115-116, the new `drive_program.cpp`'s storage note (:250-251, quoted), and
`src/common/driver_controls.hpp`'s Storage note (:49-50) — each quote checked against the file. **The adjacent stale
citation the reviewer noted is fixed at its root too** (the register is shared, but the sentence is the one this
fix rewrote): `src/main.cpp`'s "X-drive loop" text has not existed since G1 Part 2c (grep over `src/`: no hit); the
X-drive's loop is the library's `runDriverControl` since then, whose step 1 states the belief — the Source now says
so, dated. The register's status paragraph also gained its T4 Part 3 sentence (F17).

**F13 (major) — the prose review skipped six brief terms over `docs/`. REAL; run in full now (below).**

**F14 (minor) — REMAINING-WORK §11's answered cell said "below 2.5 A"; the build cuts under the 2.4 A limit. REAL;
corrected in the cell (T4's own row), dated and marked as a correction:** "one 1 s fight cut while no motor reads
above the 2.4 A over-current limit (above it the over-current cuts come first and the fight adds none)" — the
comparison is strict (`ampsNow > overCurrentAmps`, `drive_protection.hpp` :728), P2-19e pins 2.3 A (one fight cut)
and 2.45 A (the three over-current cuts, no fight cut). The RULING (the encoder check) is untouched — the wrong
number was the consequence sentence written with it, and the computed task carried the same words; for the
coordinator to know. HA-175, the changelog and the monitor's header already said "under 2.4 A".

**F15 (minor) — chapter 17 and F.6.2 stated HA-154 as fact. REAL; fixed:** "is believed to sit near its 2.5 A limit
(HA-154, not measured)" in both.

**F16 (minor) — the dropped-autons line claimed driver control in the tester; `autons.hpp` pointed at itself.
REAL; fixed:** `sayDroppedAutons(registry, driverControlRuns, emit)` — the competition program passes `true`, the
tester `false` (`every other auton still runs`); the header says why. `autons.hpp`: "Fix the entry in
src/team/autons.cpp". Tests: DA4 (the flag changes that clause and nothing else), the two source pins. Mutations
FX20, SC2, SA1.

**F17 (minor) — status lines that go false at the commit; small slips. REAL; fixed or listed.** Fixed: guide 14 (the
claws since T3 Part 2, in the driver file since T4), the monitor header (P2-19e only; P2-19d pins the rows that cut
nothing), REMAINING-WORK §10 ("written for shulib Teleop until then; it has never been run" — Station J never ran),
chapter 17 (a line logged during an auton OR driver control is a card write in that tick), the register's T4 Part 3
status sentence (no entry added; the entries brought up to the tree; the unnumbered serial belief left for the
coordinator). The status lines: their dates now say 2026-10-04/06 and the review (build-order's Next and :1490,
the roadmap's T4 row, HANDOFF's Part 3 line, REMAINING-WORK rows 10 and the plan table, `T4-COMPLETED.md` :1 and
:24); flipping each to LANDED with the commit is the coordinator's at the merge — listed under "For the coordinator"
at the end of this entry.

**THE PROSE REVIEW, RUN IN FULL (F13), and the record corrected.** *Correction first:* the late step's 16:25 entry
listed its terms and said each hit was true of the tree, but its list never ran six of the brief's §4 Part 3 terms
over `docs/` — "shulib Drive", "drive_program", "slot 1", "slot 2", "typed table", "Auton Tester" (nor "Teleop"
over `docs/`; the early step's comment sweep ran "shulib Drive" and "slot 2" over `src/` and `include/shulib/`
only). The builder's report that "every remaining hit is true of the tree or is dated history" was therefore not
earned, and it was false: the reviewer's seven lines below were stale. Run now, the hits READ before this entry
(`scratch/t4-3/fix2/pr/hits.py` prints every occurrence with its context; the dumps are `pr/hits_*.txt`):
over `README.md` and every `docs/**/*.md` except the generated `docs/api/` and the chunk records (other chunks'
briefs, logs and completion records are dated history), the worksheet included — "shulib Drive" 161, "drive_program"
17, "slot 1" 31, "slot 2" 31, "typed table" 29, "Auton Tester" 49, "Teleop" 172; and over `src/` + `include/shulib/`
the four terms no step had run there — "drive_program" 40, "slot 1" 8, "typed table" 31, "Auton Tester" 44 (every
hit true: the new program named where it is, or history marked so). **Made true here** (each a clause T4 made
false; nothing else in a shared plan document touched):
- HANDOFF (line numbers as of this entry) :170 (the only driver-control program with a hardware history: the OLD
  shulib Drive), :269 (the stopped run: `stick_mapping.hpp` is the bench tester's DRIVE station's), :272-275 (the
  stopped run: `runDriverControl`'s callers), :360 (the G1 ruling's "shulib Drive … keep `mapSticks`": the OLD one),
  :540 (the boot source that builds the claws: shared since T4 Part 3, not Teleop's), :553 (the record the programs
  boot from: the tester too), :561 (the "table" label: the boot source's), :622-634 (the Part 3 line: dates, the
  review and its fixes, C5's credit), :657 (the stopped run: T2's finish is the tester's station only), :701 (the
  stopped run: rebuild "shulib Drive", not Teleop), and :68 (the stopped run: the hardware checkout's OLD Drive).
- build-order :632 (the Next block's T4 sentence: dates, the review, the tester's drop credited), :1440 (T2's
  2026-09-26 status: the NEW Drive drives through `DriveShaper`), :1490 (T4's status: dates, the review's fixes, C5's
  credit), :1508 (the stopped run: COMP's boot "exactly as shulib Drive does since T4 Part 3").
- the roadmap :1761 and :1766 (the stopped run: the T2 rows), :1794 (T4's row: dates, the review).
- REMAINING-WORK :46 and :233 (the stopped run), :150 (Station J never ran), :179 (F14).
- the register :3774 (the port map: "every program built on `robot::Robot` … ("shulib Teleop" and the X-drive)" —
  Teleop then; Drive, the tester and the competition program since T4 Part 3; the table read by the bench tester and
  the boot source), and F12's two Sources.
- the worksheet :363 (Station C's DISAGREES: the NEW "shulib Drive" boots from the record, not the table — the OLD
  one refused on a table disagreement).
- `auton_registry.hpp`'s banner and `auton_tester_program.cpp`'s comment (F4's credit).
**Read and left, each true or dated:** every guide hit (07, 08, 10, 14, 16, 17 name the new program where it is);
the changelog's dated entries; ORIENTATION (:95-105 already say "the OLD shulib Drive" and "today's"); RESUMING
:116-140 (already T4's); the worksheet's stations (the slot map, E's pointer, F, G, J, M say the new program; G.6.5
and J.3.2 are marked RETIRED); `faq.md` :250 (the new Drive DOES check its record against the ports first);
HANDOFF :12-16 (the 2026-09-24 audit note), :464 (T4's "not yet merged" — true until the commit; listed below),
:799-803 (§7's dated order); build-order's "Previously" blocks, :1442 (T2's 2026-09-18 status, superseded by :1440),
:1512-1520 (COMP-t's landing), :1544 (G1's), the deviations table (dated rows); the roadmap's dated history boxes
(:219-:346, :1773 T3's row) — and its "You are here" "Now (2026-09-28)" paragraph (:348-:362: "slot 7 until the
driver program replaces Teleop", "and so will the driver program … "shulib Teleop" retires when that lands"), which
is the coordinator's status pointer: true until the commit, listed below; PROJECT-BRIEFING's narrative (dated
"Current as of 2026-09-24", not gated); vexbuilder-integration :64 (G1's dated landing). **Another chunk's document,
flagged and NOT edited:** `docs/internal/MECH-PANEL-requirements.md` :104 and :280 say the controller screen's three
rows are "used by the fault display in Drive and Teleop" — Teleop is gone; since T4 Part 3 they are "shulib Drive"'s
and the competition program's (through the ONE chooser, `src/common/driver_panel.hpp`).

**THE MUTATIONS (this run; `scratch/t4-3/fix2/mut/`: `mutate_fix.py`, `mutations_fix.py`, `mutations_fix2.py`,
every result in `mutation-results.txt`, every run's output in `runs/`).** Each: the file backed up, one exact
replacement asserted to occur once, a FRESH build of the suite in the scratch build dir (VOID if nothing recompiled,
except the SOURCE-ONLY pins, which read the file at run time), the FULL suite, the file restored, `cmp`-verified and
its timestamp bumped. The scratch build was first rebuilt whole from the current tree and run: 2169 | 2169 passed
(`fix2/mut/suite0.log`; again after the gap line, 2170 | 2170, `suite1.log`). **57 run, 57 RED, 0 survived, 0 void**
(Thu Oct 8 09:47–10:21):

| Id | What it breaks | Red in |
|---|---|---|
| FX1 / FX3 | a healthy tick / `reset()` never ends the episode | `loop_monitor_test` (the stretch case) |
| FX2 | every slow tick logs again | the stretch case, C5 e2e (T4-3), **1r** |
| FX8 | `overrunEpisodes()` never counts | the stretch case, both C5 e2e attribution cases |
| FX7b (FX7 re-aimed) | the attribution line on every slow tick | C5 e2e (T4-3) |
| FX4 | a repeat that is the run's FIRST fault silent | `fault_test` (first since `clear()`) |
| FX5 | `raiseRepeat()` logs | `fault_test` ×2, the stretch case, C5 e2e (T4-3), 1r |
| FX6 | `raise()` silent after the first | `fault_test` ×2, the stretch case, DIAG1 D2/F3, D-5 |
| FX9 / FX12 | `log()` / `summarize()` copy with no guard | LC-7 |
| FX10 | the guard never released | LC-4, LC-5, LC-7, 1n, 1o |
| FX11 | a kept-off line not counted | LC-7 |
| FXG1 / FXG3 | no gap line before a line / before a summary | LC-7, LC-8 / LC-8 |
| FXG2 / FXG4 / FXG5 | the gap line repeated before every line / on the primary / re-counting the first gap | LC-7, LC-8 / LC-7, LC-8 / LC-8 |
| FX13 | the provider's lost count leaves out the contended lines | 1s |
| FX14b, FX15b (re-aimed) | a card in said as NO CARD; a Truncate copy mapped as Append | LCL3 |
| FX16 / FX17 / FX18 / FX19 | the refusal below ERROR / a continuation unmarked / an over-long word silent / the old no-card guess | LCL4+LCL5 / LCL4 / **LCL5** / LCL1 |
| FX20b (re-aimed) / FX21 | the tester's line claims driver control / "none is on the menu" back | DA4 / DA1, DA4 |
| RS1–RS4, RS5b, RS6–RS12 | the review's RS1–RS10 re-aimed at `drive_panel.hpp`, the cut length typed in, the estimate's x/y swapped | the five P3-T2 (T4-3) cases, each its own |
| SP1–SP5 *(source-only)* | Drive: the refusal logged after the release / said twice / one refusal bypassing `refuseBuilt` / the groups swapped / the card never asked | P3-T3 (T4-3) ×2, P3-T3 (the binding, the painters, the copy line) |
| SC1–SC3, SA1 *(source-only)* | the competition program: the refusal after `unbuild()` / the driver-control clause dropped / the card never asked; the tester's flag | P2, P18, P17; the tester's DropEntry pin |
| RV1 / RV2 / RV3 / RV14 | the review's survivors | C5 DropEntry (the address check) / the new C5 menu case / D6 / D8 |
| RV5 | the turn 3 s while the documents say 2 s | the BUILD: the menu test's `static_assert`, which IS the pin |

Five first runs were red only at the BUILD under `-Werror` (an unused parameter or variable — FX7, FX14, FX15, FX20
and RS5): that proves nothing about the tests, so each was re-aimed with the name voided, built, and observed red in
the suite (the `b` ids). The one red-at-the-build counted as red is RV5, because there the assertion IS the test.

**What this round leaves for the coordinator** *(also in `T4-COMPLETED.md`'s "For the coordinator")*:
1. **At the merge, flip each status line to LANDED with the commit** — build-order's first `Next:` (:632) and T4's
   status (:1490); the roadmap's T4 row (:1794) and its "You are here" "Now (2026-09-28)" paragraph (:348-:362
   *— the range CORRECTED at the verification round: the whole paragraph, :348-:393; see its V4*);
   HANDOFF :464 and :622; REMAINING-WORK §2 row 10 (:46) and the plan table (:233); `T4-COMPLETED.md`'s title, Part 3
   row and "Not done here"; then the briefing.
2. **The team lead's confirm-or-reject of the Auton Tester's drop** (REMAINING-WORK §2 row 10; F4).
3. **REMAINING-WORK §11's answered cell was corrected** (F14): the computed task's "below 2.5 A" was the same wrong
   number; the build's line is the 2.4 A over-current limit.
4. **Serial's two-task exposure** (stdout, since COMP-b): a register entry (a new HA number is the coordinator's) or
   a lock. The card copy no longer depends on it (F5).
5. **`MECH-PANEL-requirements.md` :104 and :280** name "Drive and Teleop" (another chunk's document; not edited).
6. **An intermittent overrun** still logs each slow stretch — a hysteresis is the coordinator's option if Station K's
   copy shows it (F1).
7. **HA-161's `src/main.cpp` "X-drive loop" citation** (stale since G1 Part 2c, before T4) was corrected in the same
   Source sentence F12 rewrote — for awareness, since the register is shared.
8. The FIVE new files are intent-to-add (F7). Nothing is committed; the merge is still open (`MERGE_HEAD` `fab1ebb`).

### THE REVIEW-FIX ROUND STOPS HERE — every gate on the final tree *(`scratch/t4-3/fix2/gates.sh`; one log per gate under `fix2/gates/`; Thu Oct 8 10:21:33–10:37:12 EDT 2026)*
- `api_doc_tool.py generate`, `cmake -S test -B build/test`, the briefing regenerated, the FULL build with the doc
  gates exit 0; the suite **2170 | 2170 passed | 0 failed | 3 skipped**, `assertions: 2776627 | 2776627 passed`; the
  briefing regenerated after the binary.
- `api_doc_tool.py` self-test OK; check-coverage (3356 public entities, 165 headers, all documented); check-fresh
  exit 0; check-examples (521 quoted lines, 6 files, 6 whole-run listings); check-removability; `briefing_status.py`
  self-test OK (24) and check current; `doc_staleness_audit.py` self-test OK (10) and clean.
- The CI greps: PROS-free, sim-free. The ARM header gate over all **202** `include/shulib` headers: CLEAN.
- `src_build_gate.py self-test`: **OK (211 detector cases, each against a real build)** (637 s); `check`: **PASS —
  all 7 builds link** (bench, xdrive, tank, tank-drive, tank-calibrate, tank-autontest, tank-competition), 0
  disallowed warnings, every beacon and program asserted, one chain, the include root, every printed path — run after
  the last `src/` and `include/` edit.
- **No plant or mutation left applied:** every file the gate's self-test touched is byte-identical to the reviewers'
  baseline copy except `drive_program.cpp`, which is byte-identical to its pre-mutation copy of 09:49 (the stopped
  run's fixes); the 57 mutations each restored and `cmp`-verified by the runner; a grep for every mutation's
  replacement text over `src/`, `include/` and `test/`: no hit.
- **The merge is still OPEN and nothing is committed** (`MERGE_HEAD` `fab1ebb`, `HEAD` `6abcac8`); the five new files
  intent-to-add. The tree waits for the coordinator's fresh verification and the commit.

## T4-3 verification-round findings and their disposition

- **Started:** Thu Oct  8 11:22 AM EDT 2026 (`date`), a new executor run (opus) in the worktree
  `.claude/worktrees/t4-3` (branch `t4-3`), the merge still open (`HEAD` `6abcac8`, `MERGE_HEAD` `fab1ebb`). Do not
  commit; no `pros`; no `/dev/ttyACM*`. Backups, probes, mutation runs and greps of this run:
  `.claude/coordinator/scratch/t4-3/vfix/` only.
- **The task:** a fresh verifier returned four minor findings (V1–V4 below, in its order). Each is re-checked, then
  fixed at the root or answered with evidence; every gate re-run on the final tree.
- **Checked first:** no file under the worktree (outside `build/`, `bin/`, `.d/`) is newer than the review-fix round's
  stop (10:37:12) except this log and `T4-COMPLETED.md` (both 10:38:08, that round's closing entries), so no
  verifier probe or mutation touched this tree (its copies are under `scratch/t4-3/verify/`).
- Backups of every file this round edits, taken before its first edit: `scratch/t4-3/vfix/pre/` (17 files).

### 11:22–12:06 (Oct 8) — the four findings, each re-checked, then fixed at the root or stated *(`date` 12:06 at this entry)*
*(V1–V4 in the verifier's order. Mutation ids (VF, V5, VS, VD) are in the table at the end of this entry.)*

**V1 (minor) — a task PROS deletes while it holds the card copy's guard ends the copy for the boot, silently, and
three documents said otherwise. REAL; the behaviour KEPT as the chosen failure, now stated everywhere and pinned.**
Re-checked: the guard (`std::atomic_flag`) is released only by `CopyHeld`'s destructor; HA-161 (a) (reasoned) says
PROS deletes a phase task mid-statement and a deleted task runs no destructor; "shulib Drive" logs
`opcontrol(): field DISABLED ...` straight after `st.driver.run(...)` returns on `competition_is_disabled()`, and the
competition program logs `driver control ended` and `normal return` straight after its loop ends — both the moment
(a) says PROS deletes the task. `noteGap()` runs only under the guard, so after such a deletion no gap line is ever
written. The verifier's probe (`scratch/t4-3/verify/probes/held_guard_probe.cpp`, the REAL sink) shows 0 card lines
and 5 counted. The claims it names were false for this case: `log_copy_sink.hpp`'s "so one bad line can never leave
the guard held for the run", chapter 17's and K.7.2's "the card's next line says so", and the changelog's "so the gap
is visible where it happened".
**Why it is KEPT, not recovered — evidence, not convenience:**
1. **The proposed recovery rests on a belief no register entry holds.** That means recording the holder's task
   handle and treating the guard as free once `pros::c::task_get_state()` reports the task deleted. The vendored
   `include/pros/rtos.h` :124-125 does list `E_TASK_STATE_DELETED` ("deleted using task_delete") and
   `E_TASK_STATE_INVALID` ("does not point to a current or past task"). Nothing in it says what the call returns
   once PROS has built its NEXT phase task, possibly in the deleted one's memory. In upstream FreeRTOS, a task
   deleted by ANOTHER task has its TCB freed at once, and `eTaskGetState` reads through the handle. That is
   reasoning from FreeRTOS, not PROS's vendored kernel (not in this tree). A guard built on that call would make it
   a new hardware belief. The computed task's rule applies: an executor who meets a new hardware belief stops and
   logs it for the coordinator, which is done here.
2. **Releasing would be the WORSE failure.** The deleted writer may have stopped inside newlib's `fwrite` or the
   card's driver (`ProsBlockSink::write` is `std::fwrite`; `flush()` is `std::fflush`, `block_sink.hpp` :193-200).
   That can leave the FILE half-updated and any lock below it held (unmeasured). A released guard hands that
   state to the next writer, which could block there — the driver loop of the next period included. A held guard
   turns it into counted, lost card lines, and no task waits. The verifier's balance paragraph says the same. The
   header now says it as the reason.
3. **The way past it is a design change, not a fix here:** one card-writer task that alone touches the file, fed by
   the others. It removes the case, and with it the card write inside a tick (HA-60). It adds a task, a queue and a
   priority — the coordinator's call, listed below.
**Done:**
- **Stated** in `include/shulib/diag/log_copy_sink.hpp`: a new banner section, "A writer that never comes back"; the
  class `///` comment; `contendedLines()`; and `CopyHeld`'s comment, whose false claim is corrected.
- **Stated** in `hal/pros/device_provider.hpp`: its copy paragraph and `serialCopyDroppedLines()`.
- **Stated** in both programs' headers. `drive_program.cpp`'s ORDER AT BOOT said "everything from the provider's
  construction on is in the SD copy too"; it now names the copy's two exceptions, and that the boot line's count is
  as of the boot only. A comment at Drive's disable line says it races the deletion.
- **Stated** in the register: a consequence line under HA-161 ("if (a) is RIGHT"), and its status note.
- **Stated** in the student documents: chapter 17 (what a card copy that ends at a disable means; read serial);
  worksheet F.7.2 and K.7.2, each with a new question (a card copy that stopped at a disable while serial ran on?
  at which line? — the first sighting of how often); the changelog.
- **Pinned** by host test **LC-9**, the probe's shape with REAL threads. A copy whose first line does not return
  until the test lets it go (a deleted task); 500 lines and a summary from another thread.
  - All of that finishes within 10 s (no blocking guard).
  - None reaches the copy, and no gap line.
  - 501 are counted; the primary got every line and the summary.
  - Then, as clean-up and contrast, the holder returns, and the next line is preceded by `copy: 501 line(s) left off
    here ...`.
  - The test file gained `<atomic>`, `<chrono>`, `<cstdint>`, `<future>`, `<mutex>` and `<thread>`. The suite
    already links threads (`competition_screen_test` S6).
- Mutations VF1–VF4.

**V2 (minor) — LC-8 claimed the gap line comes BEFORE a summary but did not check the order. REAL; fixed as
suggested.** `ReenteringCopy` now also records ONE ordered list, `events`: a line's message, or `<summary>`
(`kSummaryMark`) for a summary. LC-8 asserts it is exactly `one`, the gap line, `<summary>` — so the gap line comes
IMMEDIATELY before the summary. The verifier's mutation V5 (`copy_->summarize(summary); noteGap();`) is now RED in
LC-8 (it SURVIVED at 11:07, `verify/mut/run1.out`).

**V3 (minor) — `refuseBuilt()`'s no-provider branch painted with its serial line OFF. REAL (unreachable today, as
the verifier says); fixed as suggested.** Both programs read `const bool logged = st.devices.has_value();` BEFORE the
release, log only when `logged`, and paint with `!logged`: Drive `paintRefusal(!logged)`, the competition program
`refuse(headline, why, !logged)`. So a refusal is said on serial exactly once on either path. (Read after the
release, the flag would be true always, and every refusal said twice — VS2/VS4 pin that.) The comments say why.
Re-pinned: P3-T3 (T4-3) (`drive_program_source_test`: the body whole, `paintRefusal(!logged)` once,
`paintRefusal(false)` never) and P2 (`competition_source_test`: the lambda whole, `refuse(headline, why, false)`
never). Mutations VS1–VS4.

**V4 (minor) — (a) the hand-over's range for the roadmap's "Now (2026-09-28)" paragraph stopped at :362; (b) "(each
named below)" over-claimed past eight drops. Both REAL; fixed.**
- **(a)** Read in the roadmap now: the paragraph is ONE block-quote paragraph from :348 to :393 (the `>` blank lines
  are :347 and :394). Its clauses that go false at the T4 commit:
  - :349-:350, the tester in slot 7 "until the driver program replaces Teleop";
  - :360-:362, "and so will the driver program … "shulib Teleop" retires when that lands";
  - :368-:375, "shulib Teleop" still COASTS on release, and T3's sentence ("shulib Teleop" now drives robot two's
    claws … the panel's two fault rows became one).

  SRC1's sentences (:385-:393) are dated history, true. `T4-COMPLETED.md`'s "For the coordinator" now gives
  :348-:393 with the three clauses named. The 10:38 entry's item 1 (above) is annotated as corrected, rather than
  rewritten.
- **(b)** The wording is now EXACT, not merely softened. `sayDroppedAutons()` says `(each named below)` while every
  dropped entry is named, and `(the first 8 named below)` past the registry's eight kept sentences. The last line
  already counted the rest.
  - The longest line is now 196 bytes, under TermSink's 200. That is the 10-digit count, the driver-control clause
    and "the first 8" (`python3` arithmetic, this run); at 16 drops it is 188.
  - Re-pinned: DA2 (11 drops: the whole line), DA3 (AT the cap `(each named below)`; one past `(the first 8 named
    below)` and no "each named").
  - Documents: chapter 17 and K.2.2 add the past-eight wording beside their quote.
  - Mutations VD1–VD3.

**THE PROSE REVIEW for this round — run, then read, before this entry.** The terms were the changed files' names
and their concepts: `log_copy_sink`, `LogCopySink`, `contendedLines`, `serialCopyDroppedLines`, `refuseBuilt`,
`logToSerial`, `dropped_autons`, `sayDroppedAutons`, "named below", `comp_log`, `drive_log`, "card copy / card's copy
/ SD copy / log copy", "one writer". They ran over `README.md` and `docs/**/*.md`, leaving out the generated
`docs/api/`, this log and other chunks' records. The dumps are `scratch/t4-3/vfix/pr/hits_*.txt` (12, 20, 2, 4, 0,
0, 9, 2, 6, 18, 17, 71 and 6 lines), plus `DROPPED` / "NOT on the menu" / "serial line of its own" / "REFUSED:
<headline>" over the changelog, the guides, the FAQ and the worksheet.
- **Made true:** every place named under V1 (and `T4-COMPLETED.md`). Chapter 17 :200-:203 and K.2.2 (V4 b). Worksheet **F.7.2**, which
  the verifier did not name: it listed "the disable lines" as on the card, the very lines that race the deletion,
  and now states the case and asks the same question as K.7.2.
- **Read and left, each true:**
  - the changelog's provider entry (`serialCopyDroppedLines()` = the card's refusals plus the lines the guard kept
    off — the post-deletion lines are among the latter) and its C5 entry (the menu's red line, unchanged);
  - HANDOFF :633 ("one writer at a time");
  - the register's HA-60 second-consumer note;
  - `T4-COMPLETED` :63 and :124;
  - guide 16 :128-133 (it quotes `DROPPED: auton '<name>': ...` only, never the summary line);
  - worksheet F.1 :499-503: the `[ERROR][DRV] REFUSED:` path is the logged one, and the no-provider path is
    unreachable with today's constant file name;
  - chapter 17's "three refusals, and each says why on the screen and on serial" — truer now.

  Unrelated "one writer" hits (`vexbuilder-integration.md` 5.2.3, H1's capture rule, MECH-PANEL :670) are another
  subject.

**THE MUTATIONS (this round; `scratch/t4-3/vfix/mut/`: `mutate_fix.py` — the review round's runner, extended to
apply a LIST of replacements in one file, each asserted once — and `mutations_vfix.py`; results in
`mutation-results.txt`, every run's output in `runs/`).**
- **The procedure:** each file backed up; the replacement(s) asserted once; a FRESH build of the suite in the
  scratch build dir; the FULL suite; the file restored, `cmp`-verified and its timestamp bumped. A run is VOID if
  nothing recompiled, except the four SOURCE-ONLY pins, which read the file at run time.
- **The baseline:** the scratch build was first rebuilt whole from this tree (202 TUs) and run: **2171 | 2171
  passed** (`vfix/mut/suite0.log`).
- **The result:** **12 run, 12 RED, 0 survived, 0 void** (Thu Oct 8 11:46:48–11:52:39). Afterwards a grep for every
  mutation's replacement text over `include/` and `src/` found nothing.

| Id | What it breaks | Red in |
|---|---|---|
| VF1 | the guard taken back after 100 contended lines (a "the holder must be dead by now" rule) | LC-9 |
| VF2 | a BLOCKING guard (waits up to 15 s for the holder; `<chrono>` added by the same mutation) | LC-9 |
| VF3 | the gap said on the copy by a writer that does NOT hold the guard | LC-7, LC-8, LC-9 |
| VF4 | a contender frees the held guard for the next line | LC-7, LC-8, LC-9 |
| V5 | the verifier's: the gap line written AFTER the summary | LC-8 |
| VS1 / VS2 *(source-only)* | Drive: the no-provider refusal painted with serial off / `logged` read after the release | P3-T3 (T4-3) |
| VS3 / VS4 *(source-only)* | the competition program: the same two | P2 |
| VD1 | "the first N" said AT the cap (`>=`) | DA1, DA3, DA4 |
| VD2 | the verifier's state back: "each named below" past the cap | DA2, DA3 |
| VD3 | the dropped count where the named count belongs | DA2, DA3 |

**What this round leaves for the coordinator** *(also in `T4-COMPLETED.md`'s "For the coordinator")*:
1. **The roadmap's "Now (2026-09-28)" paragraph is :348-:393** (V4 a). Its T4-false clauses are :349-:350,
   :360-:362 and :368-:375; flip them with the other status lines at the merge.
2. **The card copy that ends at a deletion** (V1) is kept as the chosen failure. The two ways past it are yours:
   - a guard that recovers — this needs a NEW belief, that `task_get_state()` reads DELETED for a deleted task's
     handle and never a later task's state, and it re-exposes the next writer to what the deleted one left
     half-done;
   - or one card-writer task, which removes the case and the in-tick card write.

   F.7.2 and K.7.2 now collect the first sighting.
3. **The same deletion can land mid-SERIAL-write** (the post-loop lines go to the primary first). Whether newlib
   holds a lock on stdout that a deleted task would keep is the same unmeasured belief as item 4 of the 10:38 list
   (serial's two-task exposure). Named here so the entry or lock you choose covers both.
4. Everything else in the 10:38 list stands. Nothing is committed; the merge is still open (`MERGE_HEAD` `fab1ebb`);
   the five new files are intent-to-add; this round added no file.

### 12:10–13:17 (Oct 8) — the first gate run was RED at the `src/` gate; the root fixed; every gate re-run GREEN
- **The red, 12:24-12:27** (`scratch/t4-3/vfix/gates-run1-1210/`, kept): the `src/` gate's self-test (case 35) and its
  `check` both failed on ONE disallowed warning in all seven builds. It was `-Wformat-truncation=` at
  `src/common/dropped_autons.hpp` :64: V4 (b)'s `(%s)` clause took a 48-byte buffer, so GCC bounded the 200-byte line
  at more than 200. Every gate before it was green (the suite **2171 | 2171**).
- **The cause was mine:** the file's own banner rule says "a precision on every %s", and V4 (b) broke it. **The
  fix:** the clause is printed `%.23s` — "the first 8 named below" is 23 characters. A
  `static_assert(competition::kMaxAutonRefusals < 10, ...)` guards it: a two-digit cap would be cut by that precision,
  so it is refused at the build and names DA2.
  - The longest line, at a 10-digit count with the driver-control clause, is 196 bytes.
  - A scratch probe (`vfix/da_probe.cpp`) compiled with `arm-none-eabi-g++` and the gate's warning flags. The new
    header was CLEAN; the old `(%s)` shape, as the control, reproduced the warning.
- **Mutations after the fix** (`vfix/mut/run2.out`, 12:30-12:32): VD1-VD3 re-run on the changed header, all RED as
  before. **VD4** is new: the precision `%.23s` cut to `%.16s` ("the first 8 name"). It is RED in DA2 and DA3. That
  makes **13 mutations this round, 13 RED, 0 survived, 0 void**. The restored scratch binary is green: **2171 |
  2171, assertions 2776646**, stable over two runs. The +2 assertions over 12:11 are the `static_assert`'s two
  string literals, which a source pin's literal scan counts.
- **The gates, re-run whole on the final tree** (`scratch/t4-3/vfix/gates.sh`; one log per gate under
  `vfix/gates/`; Thu Oct 8 12:56:16–13:16:47 EDT 2026):
  - `api_doc_tool.py generate`, `cmake -S test -B build/test`, the briefing regenerated, the FULL build with the doc
    gates: exit 0.
  - The suite: **2171 | 2171 passed | 0 failed | 3 skipped**, `assertions: 2776646 | 2776646 passed`. The briefing
    regenerated after the binary (its only change is the suite line, 2,171 / 2,776,646).
  - `api_doc_tool.py`: self-test OK; check-coverage (3356 public entities, 165 headers, all documented);
    check-fresh exit 0; check-examples (521 quoted lines, 6 files, 6 whole-run listings); check-removability.
  - `briefing_status.py`: self-test OK (24) and check current. `doc_staleness_audit.py`: self-test OK (10) and clean.
  - The CI greps: PROS-free, sim-free. The ARM header gate over all **202** `include/shulib` headers: CLEAN.
  - `src_build_gate.py self-test`: **OK (211 detector cases, each against a real build)** (816 s). `check`: **PASS —
    all 7 builds link**, 0 disallowed warnings in each. It ran after the last `src/` and `include/` edit.
- **No plant or mutation left applied:** every file the self-test touched (`Makefile`, `mkdocs.yml`, `src/main.cpp`,
  `src/build_selection.hpp`, `test/CMakeLists.txt`, `test/build_selection_test.cpp`, `test/chassis_table_test.cpp`,
  the tables, `calibrate_program.cpp`, `bench_r3a.cpp`, `boot_source.hpp`, `robot_config.hpp`, the vendored
  `doctest.h` and `lv_obj_style.h`) is byte-identical to the verifier's copy of the tree (`scratch/t4-3/verify/wt`).
  The two exceptions are this round's: `drive_program.cpp`, which differs from its 11:2x backup by exactly this
  round's four edits, and `PROJECT-BRIEFING.md`'s suite line. Every mutation was restored and `cmp`-verified by
  the runner. A grep for each replacement text over `src/`, `include/` and `test/` hits only this round's own tests
  and comments, which quote two of them on purpose.
- **This entry, and `T4-COMPLETED.md`'s count (12 → 13 mutations), were written after the gate run.** The doc gates
  that read them were re-run after — see the next line.
- **13:19-13:20, after the two document edits above:** `briefing_status.py check` current; `doc_staleness_audit.py`
  clean; `api_doc_tool.py` check-fresh, check-removability and check-examples exit 0
  (`scratch/t4-3/vfix/gates-final-docs/`). No `src/`, `include/` or `test/` file changed after 12:56.
- **THE VERIFICATION-FIX ROUND STOPS HERE.** The merge is still OPEN and nothing is committed (`MERGE_HEAD` `fab1ebb`,
  `HEAD` `6abcac8`). The five new files are intent-to-add; this round added none. The coordinator's list is the
  12:06 entry's "What this round leaves for the coordinator", together with `T4-COMPLETED.md`'s "For the
  coordinator".

### The coordinator's merge verification — 2026-10-08 13:20–14:30 (`date`), main `ebf1a89` + the squash of `t4-3`
- **Audit before the merge:** no gate plant or mutant in the worktree's added lines (the only `self_test` text is
  the gate's own cases in `tools/src_build_gate.py`); `MERGE_HEAD` `fab1ebb` as logged; the worktree committed as
  the transport commit `a82b992`, then `git merge --squash t4-3` onto main.
- **Seven conflicts, all documents:** the changelog (T4's entry placed first, dated 2026-10-08, the newest of the
  "since 2.4" entries, then R5a's), the register (R5a's and T4's status paragraphs both kept; the index keeps
  HA-170..HA-174 and takes T4's HA-175 row), REMAINING-WORK (the milestone chain with T4 landed), build-order's
  `Next:` (main's line with T4's facts, now LANDED), the briefing and `docs/api/` (regenerated, not hand-merged).
  The two empty `pp_*.txt` files stay deleted (main removed them, `2b07bda`).
- **Gates on the merged tree** (`.claude/coordinator/logs/t4p3_merge_verify.out`): host build with the doc gates,
  suite **2,253 | 2,253 passed, 3 skipped**; `src_build_gate.py` self-test OK and check **PASS, all 7 builds**; ARM
  header gate over **207** headers clean; both CI greps empty; `api_doc_tool` check-fresh, the briefing's self-test
  and check, the removability grep and the staleness audit clean; no plant after the gates.
- **The coordinator's hand mutations** (`muts_t4p3.json`, `muts_t4p3_b.json`; each on a fresh link, the whole
  suite, restored and hash-checked): **22 RED** — the encoder check (T43-1..3), `OnRefusal::DropEntry` (T43-5..7),
  the menu's refusal line (T43-8..10), `LogCopySink`'s per-tick filter, guard, gap line, count and summary
  (T43-11..15), the dropped-autons lines (T43-16b/16c, T43-17), the PROTECTION row (T43-18/19), the copy's boot
  word (T43-20), and the two programs' wiring through their source pins (T43-21/22). **One SURVIVED, equivalent:**
  T43-4 deletes `encoderRepeat_[i] = 0` on a member that leaves — redundant, because `seen_[i] = false` on the same
  path zeroes it on the member's first read back, and an absent member is never read as a mate. T43-16's first
  anchor matched twice after fix2; re-anchored as 16b/16c.
- **The team lead confirmed the Auton Tester's drop** (2026-10-08: "yes auton tester can work that way too"); every
  line that asked him (build-order, REMAINING-WORK row 10, HANDOFF, this record, `auton_registry.hpp`,
  `dropped_autons.hpp`) now says so. The hand-over's status lines were flipped to LANDED (build-order's `Next:` and
  T4 status, the roadmap's T4 row and a new "Now (2026-10-08)" paragraph over the 2026-09-28 one, HANDOFF, the record's
  title and "Not done here"), and MECH-PANEL :104/:280's "Drive and Teleop" corrected.
- **Left for the coordinator, taken:** the serial log's two-task exposure — the vendored newlib's per-FILE locks are
  no-ops (`__retarget_lock_*` = `bx lr` in `firmware/libc.a`, linked); PROS's `ser_write_r` takes a mutex per device
  write; stdout's buffer is shared and unlocked. It is registered separately (the next commit) and a fix is planned.
