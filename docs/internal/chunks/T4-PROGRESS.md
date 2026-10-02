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
