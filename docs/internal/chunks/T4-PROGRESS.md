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
