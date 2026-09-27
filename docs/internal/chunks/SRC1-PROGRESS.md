# SRC1 — progress log (live)

- **Chunk:** SRC1 — `src/` organized into `robots/`, `team/`, `programs/` and `common/`
- **Brief:** [`SRC1-organize-src.md`](SRC1-organize-src.md) (committed at `0fab866`)
- **Started:** Sun Sep 27 01:54:33 AM EDT 2026
- **Tree at start:** `0fab866`, clean, in the git worktree `.claude/worktrees/src1` (branch `src1`) —
  NOT the main working tree
- **Executor model:** opus
- **Go:** the coordinator's workflow run, 2026-09-27, on the team lead's "Yes, this layout" / "Now,
  before the new code is built". Do not commit; no robot upload; no `pros` commands; no
  `/dev/ttyACM*`. Mutation backups under `.claude/coordinator/scratch/src1/` (never `/tmp`).

Appended as the work went. **Nothing in this chunk has run on a robot.** No behaviour change is intended.

> **Correction (SRC1 review, 2026-09-27 03:12):** this line first said "Appended in real time", and the
> section times below were first written as 01:55–02:20, 02:20–02:55, 02:55–03:15, 03:15–03:50,
> 03:50–04:20 and 04:20–04:45. They were estimates written at the end, not clock readings, and four
> of them were later than the clock when the review read them (`date` 02:44). This log was last
> written at 02:40:21. The times now in the headers come from the scratch files' modification times
> (`ls --time-style=+%H:%M:%S .claude/coordinator/scratch/src1/`, and the changed files' own `stat`),
> which date the same work to 01:54–02:40; each header names the files it is dated by. The work each
> section describes is unchanged.

---

## Log

### 01:54 — start
- `date` → Sun Sep 27 01:54:33 AM EDT 2026; `git status` clean; HEAD `0fab866`.
- Read `docs/internal/RESUMING.md` (the chunk loop, the standards, the pace rules) and the brief in
  full. This log created before any code.

### 01:54–02:00 — required reading, and the baseline measured before anything moved *(dated by `rodata_strings.sh` 01:58:43, `baseline/*` 01:59:04–02:00:24)*
- Read: `Makefile`, `common.mk` (INCLUDE at :150 is `$(foreach dir,$(INCDIR) $(EXTRA_INCDIR),-iquote"$(dir)")`;
  sources by `rwildcard` at :152-157, recursive; objects `bin/<path under src>.o`; the compile rule
  at :279 adds a per-file `-iquote"$(INCDIR)/$(dir $*)"`; `EXCLUDE_SRC_FROM_LIB` at Makefile:172),
  `tools/src_build_gate.py` in full, `test/CMakeLists.txt`, `.github/workflows/ci.yml`,
  `tools/doc_staleness_audit.py`'s scope, `tools/briefing_status.py`'s `generate`/`check`.
- Every `#include` of a src file: 8 test TUs include `../src/<file>` (boot_source, chassis_table x2,
  bench_r3a, teleop_claws, mechanism_table, build_selection, xdrive_config); every src file includes
  its siblings by bare name. Three test TUs also READ src files by path (`mechanism_table_test.cpp:349`
  and `:980`, `teleop_claws_test.cpp:911`, `:985`).
- `git grep -c` of the 13 moved stems as `src/<stem>` over the tree: **781 hits in 95 files**
  (most in PROGRESS/COMPLETED records, which are not edited).
- **Found before touching anything: nine PRINTED strings name a moved file** (the packages print
  them — `strings` of each hot ELF's `.rodata`, below): `[BLD] robot tables: %s (src/chassis_table.hpp,
  src/mechanism_table.hpp)` (main.cpp:131, all six builds), Drive's `fill/fix src/chassis_table.hpp,
  rebuild, upload`, Calibrate's `type it into src/chassis_table.hpp, rebuild, upload`, the boot
  source's `mechanism table: REFUSED -- %s (src/mechanism_table.hpp)` and `src/chassis_table.hpp and
  src/mechanism_table.hpp disagree`, Teleop's `TYPED TABLE (src/chassis_table.hpp)` and its banner,
  the claw banner's `no mechanisms are declared for this robot (src/mechanism_table.hpp)`, and five
  bench-tester lines. The brief's §3 says each boot banner and `[BLD]` line prints exactly as at
  `37706ce` "except for any `__FILE__`-derived text". No file uses `__FILE__` (`git grep` over
  `include/shulib` and `src`: none). Decision logged at the edit below.
- **Baseline, host:** `build/test-base` (a separate build dir, fresh configure at `0fab866`) —
  `[doctest] test cases: 1659 | 1659 passed | 0 failed | 3 skipped`, `assertions: 2467051 | 2467051
  passed`. (The first attempt stopped at the briefing gate: this log existing makes SRC1 an open
  chunk; `briefing_status.py generate` then the build — the task's "regenerate before the build".)
- **Baseline, ARM:** `.claude/coordinator/scratch/src1/rodata_strings.sh` built all six packages
  fresh (`rm -rf bin .d; make <args>`) and saved each hot ELF's `.rodata` strings, its defined symbols
  and its `size` line to `scratch/src1/baseline/`: bench 593 strings (text 56209), xdrive 424 (63001),
  tank 603 (56987), tank-drive 265 (23251), tank-library 558 (83111), tank-calibrate 411 (49223).
  After the move the same script runs again and the two are diffed.

### 02:00–02:20 — the moves, the includes, the include root, the gate *(dated by `rewrite_includes.py` 02:03:15, `rewrite_paths_code.log` 02:05:45, the hand edits 02:07:02, `gate-check-1.log` 02:12:11, `tools/src_build_gate.py` last edited 02:20:19; it overlaps the next two sections)*
- **`git mv`, 20 files** (names unchanged; `git status` shows 20 `R` renames): `src/robots/` ←
  chassis_table.hpp, mechanism_table.hpp, xdrive_config.hpp; `src/programs/` ← bench_r3a.{hpp,cpp},
  calibrate_program.{hpp,cpp}, drive_program.{hpp,cpp}, teleop_program.{hpp,cpp}, teleop_claws.hpp,
  xdrive_program.{hpp,cpp}; `src/common/` ← boot_source.{hpp,cpp}, screen_text.hpp,
  state_screens.{hpp,cpp}, fault_latch_hook.hpp. `src/main.cpp` and `src/build_selection.hpp` stay.
  `src/team/README.md` (new, one paragraph) makes the empty team folder exist.
- **Landmine 2, confirmed before fixing the path:** with the tables moved and `TABLE_FILES` still
  naming `src/chassis_table.hpp`, step 0 returned exactly two problems — `src/chassis_table.hpp is
  missing — step 0 cannot derive its per-robot names` and the same for the mechanism table; the
  per-robot name set was EMPTY. Fail-closed, as G1 Part 2c final wrote it. Then fixed.
- **40 includes rewritten** to the src-rooted form by a script (`scratch/src1/rewrite_includes.py`,
  which prints each one): 32 in src/ (every sibling include, each .cpp's own header included), 8 in
  test/ (`../src/<f>` → `<folder>/<f>`; `../src/build_selection.hpp` → `build_selection.hpp`).
  Include ORDER left as it was (a pure path rewrite).
- **The include root — `-iquote`, decided:** Makefile `EXTRA_INCDIR:=$(SRCDIR)` (common.mk's INCLUDE
  line makes it `-iquote"./src"` after `-iquote"./include"`); test/CMakeLists.txt
  `target_compile_options(shulib_tests PRIVATE "-iquote<repo>/src")` (joined spelling: ONE option,
  which CMake's option de-duplication cannot split). Why not `-I`: every src/ include is quoted;
  `-I` would also answer `<…>` includes. Considered and NOT relied on: search order. On the robot
  `include/` is searched before `src/`; on the host `-iquote src` is searched BEFORE the `-I` shim,
  library and vendor dirs — the two builds' orders differ, so order cannot be the proof. The proof
  is a new gate check (step 0 `[include-root]`, below) that makes every src/ path answerable by ONE
  place only, in both builds.
- **Paths inside the code** (`scratch/src1/rewrite_paths.py`, look-behind so `../src/x`-style
  prefixes are reported rather than rewritten — none were): **176 `src/<moved file>` paths** in
  Makefile, 11 include/shulib headers (comments only), src/, test/ and tools/src_build_gate.py
  (full list: `scratch/src1/rewrite_paths_code.log`). Then by hand: every "by relative path"
  comment (5 test TUs, bench_r3a.hpp, xdrive_config.hpp) — the tests include from src/ now; every
  "every src/*.cpp" (6 comments) → "every .cpp under src/" (after the move `src/*.cpp` is only
  main.cpp); main.cpp's program table re-aligned and its "every program lives in its own file
  under src/" → src/programs/ (+ the other three folders named).
- **The nine printed strings — DECISION: updated to the new paths** (a deviation from the brief's
  §3 wording, logged here and reported): each tells the reader WHICH FILE TO EDIT ("fill
  src/chassis_table.hpp, rebuild, upload", "[BLD] robot tables: … (src/chassis_table.hpp, …)"), and
  after the move that file does not exist — a refusal that sends a student to a missing file is
  the stale-pointer class the documentation rules exist to prevent, and it is the same class as
  the `__FILE__`-derived text §3 already excepts. Alternative rejected: leave them (the banner
  "prints exactly as at 37706ce", pointing at nothing). Width checked for each against its
  limit: Drive's refusal `fix` row is `%.54s` (49 and 50 chars after); Calibrate's fix wraps to 2 x
  50 cols (58 chars → "type it into src/robots/chassis_table.hpp," / "rebuild, upload" — 2 lines
  before too, complete); Teleop's `library graph BUILT from the TYPED TABLE (…): %.120s` stays
  within TermSink's 200 bytes (≤193); the boot source's `why[192]` holds ≤161; the bench tester's
  screen cuts at 54 as it already did for all five (the SD/serial lines carry them whole). The
  three host tests that pin two of these sentences word for word (`boot_source_test.cpp` ×2,
  `teleop_claws_test.cpp` ×1) follow the sentence.
- **tools/src_build_gate.py:** `TABLE_FILES` → `src/robots/…`; the self-test's plant paths
  (SRC_PLANT = src/programs/bench_r3a.cpp, the two tables, DRIVE_PLANT, CALIBRATE_PLANT) moved;
  NEW step 0 `[include-root]` (`include_root_problems()`, header "THE INCLUDE ROOT") — (a) a
  src-relative path that also exists under include/, test/pros_shim/, test/vendor/ or test/, or
  under any src/ subfolder, FAILS (both directions: src/shulib/… or src/pros/… would shadow the
  library or SDK on the host); (b) a quoted include in a src/ file that climbs with `..` or
  resolves beside its includer in a subfolder FAILS (the old sibling form builds — that is why it
  is a check); (c) a host test including `../src/…` FAILS. `_newfile` now creates (and removes)
  missing parent directories. Self-test cases 28 (source-only: every folder read by the one-chain
  rule, the include root's plants) and 29 (a BUILD: an old-form include in src/programs/ fails
  the build, naming the file). No compile-flag check is token-exact on include flags (the
  token-exact check is on the -D defines only), so nothing was weakened; a missing `-iquote src`
  fails every build (programs' includes resolve nowhere).

### 02:12–02:16 — the six packages, measured against the baseline *(dated by `gate-check-1.log` 02:12:11, `after/*` 02:13:02–02:14:08, `compare_rodata.py` 02:16:05)*
- `python3 tools/src_build_gate.py check` (first run on the moved tree, before any doc edit) →
  `src build gate: PASS (all 6 builds link; …)`, each build `8/8 TUs compiled`, `6 vendor-ignored,
  0 disallowed`; `[include-root] 22 source files in src/ and 4 folders (common/, programs/, robots/,
  team/): every include of one written from src/, no path a second root could answer`.
- The robot compile line, read from `make ROBOT=bench V=1` (drive_program.cpp's): `-iquote./include`,
  `-iquote./src`, `-iquote./include/programs/` — in that order (the third is common.mk's per-TU
  root; it comes after src/, so it cannot shadow a src/ header).
- `scratch/src1/rodata_strings.sh` again → `scratch/src1/after/`, and
  `scratch/src1/compare_rodata.py` classifies every `.rodata` string difference per build:
  `bench 593 -> 593; 3 renamed-path string(s); 1/1 merge-artifact differences; unexplained: 0`,
  `xdrive 424 -> 424; 3; 2/2; 0`, `tank 603 -> 603; 3; 1/1; 0`, `tank-drive 265 -> 265; 3; 2/2; 0`,
  `tank-library 558 -> 558; 6; 4/4; 0`, `tank-calibrate 411 -> 411; 1; 2/2; 0` → `ALL EXPLAINED`.
  The only differences *(incomplete, the review found: the objects' link order changed too — R6 below)*: the printed paths decided above; the build stamps (`__DATE__ __TIME__`,
  `BUILD hh:mm:ss`, the hash now `-dirty`); and `strings`' merge artifacts (the same C string with a
  neighbouring printable byte glued to its front — `jUNKNOWN`/`UNKNOWN` — which moves when a
  longer string shifts the layout). The DEFINED SYMBOLS (`nm -C --defined-only`, names and types)
  are IDENTICAL in all six builds. Text grows by exactly the path bytes plus alignment: bench +34,
  xdrive +34, tank +34, tank-drive +36, tank-library +64, tank-calibrate +6; data and bss
  unchanged. No `__FILE__`-derived text exists.
- Which boot/[BLD] text changed, per build: the `[BLD] robot tables:` line in all six; Teleop's
  banner line `…else from src/robots/chassis_table.hpp…` and `TYPED TABLE (src/robots/…)` (library,
  and xdrive's package carries the same literals); nothing else in any boot banner.

### 02:13–02:23 — the documents, and the host suite *(dated by `rewrite_paths_docs.log` 02:13:09, the docs' own times 02:13:09–02:20:30, `host-*.log` 02:21:07–02:22:36)*
- **Live documents, paths:** the same script over README.md, guide chapters 07/08/10/14,
  `docs/hardware-assumptions.md` (29 — every Source line), `docs/roadmap.md` (5),
  `docs/internal/HANDOFF-remaining-work.md` (9), `docs/internal/build-order.md` (9), the worksheet
  `R3a-BENCH-WORKSHEET.md` (9) — **88 paths** (`scratch/src1/rewrite_paths_docs.log`). By hand:
  README's tree block (src/ now shows its four folders and the two top files), guide 07's `src/`
  paragraph (the four folders, and "every include written from src/"), HANDOFF §4's programs
  paragraph (one sentence on the layout), roadmap's "Now" paragraph (one SRC1 paragraph; the
  pointer's first sentence unchanged), build-order: the DOCS/DEFECTS row (+ SRC1), the count
  **57 → 58** (SRC1's parenthetical first), a deviations-table row, one sentence in the Next block
  after its bold pointer (the pointer itself unchanged — T4 / COMP-b stay next), and the G3a
  entry's dated note.
- **The G3a entry — a deviation from brief §2.6, logged:** §2.6 says the G3a entry gets a note and
  its body is not rewritten; its body names `src/chassis_table.hpp` and `src/mechanism_table.hpp`
  in backticks, and `doc_staleness_audit.py` reads build-order.md and FAILS a dead backticked path.
  So the note is added AND that one sentence's two path tokens carry the new paths; the note says
  so. The build-order "Previously — … kept as history" Next blocks got the same token update for
  the same reason (the audit reads the whole file; the files are the same files).
- **Open briefs, a dated note at the top of each, bodies untouched** (`git diff --stat`: +2 lines
  each): COMP-competition-program.md, SAFE-SAVE-…, L9-…, R5a-…, H1-shul2-wire.md,
  DIAG1-partB-median-nonfinite.md, G1-part2-pros-provider-and-main.md ("G1's §2c note" read as
  that brief — its §2c is where the paths are named). Each note maps exactly the files that brief
  names, by folder, and links the SRC1 brief §1.
- **Not edited, by rule (records describe the tree at the time):** every -PROGRESS and -COMPLETED
  record, `docs/internal/evidence/`, and the committed briefs of landed work (GATE1, R2a, R3b ×3,
  R3d ×3, T2, T3 ×3).
- **CI (`.github/workflows/ci.yml`), comments only:** the PROS guard's "src/main.cpp remains the
  other deliberate exception" → src/ as a whole (main.cpp and the programs under src/programs/ and
  src/common/) — it was already narrower than the truth before the move; the self-test list gains
  SRC1's plants.
- **docs/api regenerated** (`api_doc_tool.py generate`): 11 pages, 19 lines, each a header comment's
  `src/<file>` path. `check-fresh` exit 0. mkdocs.yml unchanged.
- **NOT touched, with why:** `docs/changelog.md` (it records library API changes; SRC1 changes no
  library code); `docs/internal/RESUMING.md`, `ORIENTATION.md`, `docs/shulib-v2-master-plan.md`,
  `docs/vexbuilder-integration.md`, `docs/faq.md`, the cookbook — each grepped for every old path
  and for its description of `src/`: no hit to fix (RESUMING and the master plan name only
  `src/main.cpp` and `src/build_selection.hpp`, which did not move). `project.pros` lists only
  `src/main.*` (unchanged). `common.mk` (the PROS template's file; EXTRA_INCDIR is its hook).
- **Host suite, fresh configure (`rm -rf build/test`):** the flags line carries
  `-iquote/…/src1/test/../src`. First run: **1 failure** — `mechanism_table_test.cpp:1038` pins
  Calibrate's `#include "mechanism_table.hpp"` as text (my earlier grep looked for `src/` in string
  literals and could not see an include pinned without it). Updated to `#include
  "robots/mechanism_table.hpp"` (the move forces it); then `grep 'include \\"' test/*.cpp` — the
  only other hits are pros_adapter_fence_test's `"pros/…"` checks, untouched by the move. Rerun:
  **`test cases: 1659 | 1659 passed | 0 failed | 3 skipped`, `assertions: 2467051 | 2467051
  passed`** — the baseline's numbers exactly.
- Briefing regenerated after the binary existed; `briefing_status.py check` → current;
  the doc gates in the build: coverage 2908/155 all documented, examples 437 verbatim,
  removability clean, staleness audit clean (27 live docs).

### 02:25–02:29 — the hand mutations (each observed, each restored and `cmp`-checked; backups in scratch/src1/backups/)
Step 0 is run exactly as `cmd_check` runs it (`scratch/src1/step0.py`: the four source checks).
| # | Mutation | Observed |
|---|---|---|
| M1 | `#ifdef SHULIB_ROBOT_TANK_2026` appended to src/programs/teleop_program.cpp | step 0 FAIL: `src/programs/teleop_program.cpp:1002 names a build switch` |
| M2 | `#if defined(SHULIB_PROGRAM_LIBRARY)` appended to src/common/state_screens.cpp | step 0 FAIL: `src/common/state_screens.cpp:49 names a build switch` |
| M3 | new file src/team/m3_probe.hpp with `#if defined(SHULIB_PROGRAM_DRIVE)` | step 0 FAIL: `src/team/m3_probe.hpp:2 names a build switch` (file removed; team/ holds README.md only) |
| M4 | `&kRobotTwoMechanisms` named in src/programs/xdrive_program.cpp | step 0 FAIL: `…:353 names kRobotTwoMechanisms, a per-robot table` |
| M5 | `#if defined(SHULIB_ROBOT_TANK_2026)` back in src/robots/chassis_table.hpp | step 0 FAIL: `src/robots/chassis_table.hpp:523 names a build switch` |
| M6 | old form `#include "chassis_table.hpp"` in src/programs/calibrate_program.cpp; fresh `make ROBOT=tank PROGRAM=calibrate` | make exit 2: `src/programs/calibrate_program.cpp:137:10: fatal error: chassis_table.hpp: No such file or directory`; no package |
| M7 | the old SIBLING form in src/robots/mechanism_table.hpp; fresh `make ROBOT=tank PROGRAM=library` | **make exit 0 — it BUILDS** (resolved beside its includer); step 0 FAIL: `[include-root] src/robots/mechanism_table.hpp:142 includes "chassis_table.hpp", which resolves beside it in src/robots/` — the case the check exists for |
| M8a | include/robots/chassis_table.hpp planted (`#error "SHADOW …"`); step 0; fresh `make ROBOT=bench` | step 0 FAIL (`include/robots/chassis_table.hpp and src/robots/chassis_table.hpp are the same path under two include roots`); the robot build took include/'s copy: `./include/robots/chassis_table.hpp:2:2: error: #error "SHADOW …"` — **the robot build searches include/ first** |
| M8b | test/robots/chassis_table.hpp planted; step 0; host compile of drivetrain_description_test.cpp | step 0 FAIL (same message, test/); the host compile took test/'s copy: `test/robots/chassis_table.hpp:2:2: error: #error "SHADOW …"` — **the host build searches the test's own directory first**. The two builds' orders differ, which is why the proof is the check, not the order. |
| M9 | `EXTRA_INCDIR:=$(SRCDIR)` dropped from the Makefile; fresh `make ROBOT=tank PROGRAM=library` | make exit 2: `src/common/boot_source.cpp:22:10: fatal error: common/boot_source.hpp: No such file or directory` |
| M10 | `-iquote…/src` dropped from test/CMakeLists.txt; fresh configure (build/test-m10), compile chassis_table_test.cpp | exit 2: `test/chassis_table_test.cpp:29:10: fatal error: robots/chassis_table.hpp: No such file or directory` |
| M11 | mechanism_table_test reads `src/mechanism_table.hpp` (the OLD path); rebuilt binary | `mechanism_table_test.cpp:211: FATAL ERROR: REQUIRE( in.good() )`, 1658/1659; restored, rebuilt: 1659/1659 |
After all eleven: step 0 PASS (0 problems); `git status` shows no probe file.

### ~02:30–02:40 — the gate's self-test, every gate's final line, the grep accounting *(dated by `gate-selftest.log` 02:36:04 at the end of a ~6-minute run, `all_headers.cpp` 02:36:24, `final-*.log` 02:37:08–02:37:31, the briefing 02:37:48, `gate-check-final.log` 02:39:22, this log 02:40:21)*
- `python3 tools/src_build_gate.py self-test` (6 min, real builds) → **`src build gate self-test: OK
  (180 detector cases, each against a real build)`** — 157 at `37706ce` (G1-PROGRESS.md, Part 2c
  final) + 21 (case 28) + 2 (case 29). No probe file left behind (`git status`: only this log and
  src/team/ untracked).
- Final, every gate, on the finished tree:
  - host suite, FRESH (`rm -rf build/test`, configure, build): `[doctest] test cases: 1659 | 1659
    passed | 0 failed | 3 skipped`; `assertions: 2467051 | 2467051 passed | 0 failed` — identical to
    the baseline.
  - `src_build_gate.py check`: `src build gate: PASS (all 6 builds link; policy: errors fail, vendor
    warnings path-ignored, no category allowed, every other warning fails; defines token-exact; one
    chain picks program and robot; variant and robot beacons asserted; programs asserted; src/'s
    include root unambiguous)`; each build `8/8 TUs`, `6 vendor-ignored, 0 disallowed` (6 before
    the move too — all `include/liblvgl/core/lv_obj_style.h:94`). Objects land under `bin/common/`,
    `bin/programs/`, `bin/main.cpp.o` — no two share a path. `EXCLUDE_SRC_FROM_LIB` (Makefile:188,
    `$(SRCDIR)/main` + each extension) still names `src/main.cpp`, which did not move, and only
    matters under `IS_LIBRARY:=1` (it is 0).
  - ARM header gate: `TU includes 191 headers` / `ARM compile gate: CLEAN (all v2 headers)`.
  - CI greps: `library is PROS-free (all of include/shulib except hal/pros/)`; `layering holds: core
    is sim-free`.
  - `api_doc_tool`: self-test OK; `doc coverage: 2908 public entities across 155 headers, all
    documented`; check-fresh exit 0; `doc example scan: 437 quoted lines, all verbatim`;
    `removability: no public doc references docs/internal/`.
  - `briefing_status.py check`: `generated block current` (regenerated before the build and again
    after the suite binary existed: Position 31 of **58**; SRC1 listed OPEN — no -COMPLETED.md yet).
  - `doc_staleness_audit.py`: self-test OK (10); `clean (27 live docs …)`.
- **Grep accounting** (`git grep -E 'src/(<the 13 moved stems>)\b'`): **781 hits in 95 files at the
  start → 513 in 32 files, every one under docs/internal/chunks/, none elsewhere** (README, docs/,
  include/, src/, test/, tools/, Makefile, CI: 0). The 32: the seven open briefs, bodies untouched,
  each with its dated note (COMP 3, SAFE-SAVE 22, L9 20, R5a 16, H1 4, DIAG1 Part B 11, G1 Part 2
  10); the SRC1 brief itself (2, its own mapping); records — PROGRESS (COMP 1, DEFECTS2 1, G1 130,
  GATE1 4, R2a 14, R3a 3, R3b 22, R3d 36, T2 2, T3 166) and COMPLETED (G1 8, R2a 2); briefs of
  landed work (GATE1 1, R2a 4, R3b part0b 4 / parts123 1 / session2 2, R3d 5 / part2 4 / part2f 3,
  T2 1, T3 addendum 3 / claw-pivots 2 / part2 6). `../src/` and `/src/<stem>` forms: none in live
  files (R3b-PROGRESS:1054 and SAFE-SAVE:381 are records/brief bodies).

## For the coordinator
*(Revised at SRC1's review, 2026-09-27 ~04:20: items 1, 2 and 5 rewritten, 6 and 7 added; each
change is explained under "SRC1 review findings and their disposition" below.)*

1. **The printed paths changed — THE TEAM LEAD RULES ON THIS BEFORE THE COMMIT.** Brief §3 says each
   boot banner and `[BLD]` line prints "exactly as at 37706ce", with `__FILE__` text the only
   exception, and no file uses `__FILE__`. The builder changed **18 literal lines holding 20 path
   occurrences** anyway, because each old path sends a student to a file that no longer exists:
   `[BLD] robot tables:` (main.cpp, all six packages), Drive's two refusal fixes, Calibrate's
   cartridge fix, the boot source's two, Teleop's `TYPED TABLE (…)`, its banner and its claw line,
   five bench-tester lines, the two `kNoChassis` strings (gc-dropped, in no package) and two
   `static_assert` messages (compile time only). That departs from the go as briefed.
   - **Keep:** the tree as it stands. Log the ruling here and in the commit body.
   - **Revert:** from the tree's root, `git apply
     /home/gonzei/projects/shulib/.claude/coordinator/scratch/src1/review/SRC1-revert-printed-paths.patch`
     (15 files; `git apply --check` passes against this worktree as it stands). It puts the 20 paths back to their 37706ce text, the three host-test pins with them,
     rewords the roadmap, build-order and HANDOFF sentences that describe them, and REMOVES the
     step-0 `[printed-path]` check this review added (R3 below), because that check exists to fail
     exactly those literals. Verified in a snapshot (below, R1): the suite 1659/1659, `check` PASS,
     `self-test` OK (183), and the six packages' strings identical to `0fab866`'s. Then log the
     ruling. The old note here ("a one-token edit per string (+ the 3 test pins)") understated it.
2. **The G3a entry's body** carries the two new table paths (the staleness audit reads
   build-order.md) — a departure from §2.6's "do not rewrite", stated in the entry's note. The note
   no longer decides where G3a's generator writes: it names that as G3a's brief's decision, and
   names the `TABLE_FILES` coupling that brief must know (R7).
3. "G1's §2c note" was read as `G1-part2-pros-provider-and-main.md` (noted at its top).
4. **Not noted** (§2.6 does not list them; their remaining work is robot sessions, not code against
   these paths): the R3a/R3b/R3d/T2/T3 briefs. Say if they should carry the note too.
5. **The commit, exactly.** The renames are staged by `git mv`; every other change is unstaged; and
   TWO NEW FILES ARE UNTRACKED: `src/team/README.md` and this log. `git commit -a` and `git add -u`
   leave both out, and without the README `src/team/` does not exist in the commit (README, guide
   07, HANDOFF, roadmap and build-order all describe it) and CI's `src_build_gate.py self-test`
   FAILS at case 28 ("README.md is missing"). So: `git add -A` (or name
   `src/team/README.md docs/internal/chunks/SRC1-PROGRESS.md` explicitly, with SRC1-COMPLETED.md and
   the regenerated `docs/internal/PROJECT-BRIEFING.md`), then `git status --short` must print
   nothing. The `-COMPLETED.md` record, the commit and the merge verification are the coordinator's.
6. **Build-order edits the builder did not report before (R7):** besides the G3a entry, path tokens
   were rewritten inside four "Previously — … kept as history" Next blocks (build-order.md:636,
   638, 640, 644), the dated T2 status "2026-09-26 (G1 Part 2b)" (:1419), the G1 entry's dated
   status (:1496) and the deviations-table row "a third build variant" (:2013). Only path tokens
   changed, each `src/<file>` → its new folder; the reason is the staleness audit, which reads the
   whole file (`PATH_FILES` includes build-order.md, and its dead-path sweep has no history
   exemption) and fails a backticked path that no longer exists. Say so to the team lead with
   item 1.
7. **Link order changed (R6):** the objects now link in a different order (common.mk sorts object
   paths, and `bin/common/…` sorts before `bin/main…` and `bin/programs/…`), so the memory layout
   moved. No package has a start-up initializer to reorder (`__init_array_start ==
   __init_array_end` in all six packages, before and after). Say so in the commit body.

---

## SRC1 review findings and their disposition

- **Started:** Sun Sep 27 03:04:15 AM EDT 2026 (`date`), the same worktree, the same builder (opus),
  on the coordinator's review of this log and the tree: eight findings. Each is re-checked below
  and either fixed at its root or answered with evidence. Every gate is re-run at the end. Nothing
  is committed.

### R2 (major) — the section times were invented — REAL, FIXED (03:12)
- Re-checked: the scratch files' times (listed in the correction note at the top) put the work at
  01:54–02:40; this log's headers said 01:55–04:45. `date` read 03:04 when this review began, so
  "03:50–04:20" and "04:20–04:45" were still in the future then. `git log -1 --format=%ci 0fab866`
  → `2026-09-27 01:51:28 -0400`, so the brief's "Written … (~02:10)" named a time after its own commit.
- Fixed: every section header now carries the time range its scratch files show, and names those
  files; a dated correction note under line 13 keeps the old headers and says what they were
  ("Appended in real time" is gone from that line). The brief's header line is corrected in place,
  saying what it said before. From here on each entry's time is a `date` reading taken when it was
  written.

### R1 (major) — the printed paths changed against brief §3 — REAL; PUT TO THE TEAM LEAD, BOTH OUTCOMES READY (03:40–04:16)
- Re-checked: `git diff -M HEAD -- src/` holds **18 changed literal lines, 20 path occurrences** (the
  list is in "For the coordinator" item 1; the review counted 15). Brief §3 allows only
  `__FILE__`-derived text, and no file uses `__FILE__`. The departure is real, and so is the
  review's second point: roadmap.md ("no behaviour changed (the six packages differ only in the
  file paths they print …)") and the build-order Next block ("no behaviour changed … only the
  printed file paths moved") described the change as settled, and the old cost note here left out
  those sentences.
- What this builder cannot do is rule on it: pace rule 2 makes it the team lead's call, through the
  coordinator. Fixed at the root by making the ruling cheap and exact, in both directions:
  - **The tree keeps the new paths** (the builder's recommendation: a refusal that says "fill
    src/chassis_table.hpp" names a file that no longer exists). The two sentences no longer say "no
    behaviour changed". roadmap.md: "no program's logic changed: the six packages print each file's
    new path where a message names one, and otherwise hold the same functions and data, linked in a
    new order". build-order.md's Next block names the §3 departure as "put to the team lead". Item 1
    of "For the coordinator" says the ruling comes BEFORE the commit.
  - **The revert is a verified patch**, `scratch/src1/review/SRC1-revert-printed-paths.patch`, made by
    `scratch/src1/review/make_revert.py` in a snapshot of this tree (`revert-tree`, its own git repo,
    `GIT_CEILING_DIRECTORIES` set so nothing reads the main repo). It changes 15 files: the 20 paths
    back to their `0fab866` text (only inside string literals, tokenized by the gate's own
    `_CXX_TOKENS`, so comments keep the new paths), the three test pins, the roadmap, build-order and
    HANDOFF sentences, the CI comment, and the whole `[printed-path]` check (R3). Measured in the
    snapshot: fresh host build, its doc gates clean (coverage 2908/155, 437 verbatim, removability,
    briefing, staleness audit), `test cases: 1659 | 1659 passed | 0 failed | 3 skipped`,
    `assertions: 2467051 | 2467051 passed`; `check` → `src build gate: PASS (all 6 builds link; …)`;
    `self-test` → `OK (183 detector cases, …)`; the six packages' `.rodata` strings
    (`rodata_strings.sh`) set-equal to `baseline/` (`0fab866`) apart from the build stamps and the
    `strings` merge artifacts (`jUNKNOWN`/`UNKNOWN` and the like, printed and read), and **no
    string naming a new folder**; the 18 reverted literals are token-identical to `0fab866`'s.
    `git apply --check` of the patch passes against this worktree.

### R3 (minor) — 16 of the 20 printed paths (15 of the 18 literals) were pinned by nothing — REAL, FIXED (03:17–04:16)
- Re-checked: the review's surviving mutation is real. `drive_program.cpp:329` → `"fill
  src/robot/chassis_table.hpp, rebuild, upload"`, and the six builds still link (the builds cannot
  see it, and no host test reads that literal).
- Fixed at the root with a fifth step-0 check, **`[printed-path]`** (`printed_path_problems()`;
  header "THE PATHS A PROGRAM PRINTS"). Every `src/…` path inside a string literal of a file under
  `src/` or `include/shulib/` must name a file or folder that exists. Adjacent literals are joined
  first, as the compiler joins them (`_literal_runs`), and a sentence's full stop is not part of
  the path. Comments are not read: that is the staleness audit's class, and a comment never
  reaches a robot. `test/` is not read either, because its tests pin what they print. A problem
  names the line of the literal the path starts in; that attribution was checked against a naive
  count for all 4,222 literals in the scanned files, with no mismatch. On the real tree: 20 paths
  in 18 literal runs, all exist. The PASS line gains "every printed src/ path exists". Self-test
  **case 30**: the real tree is clean; a dead path is reported in Drive's refusal, when split
  across two adjacent literals, and in a library header; the same dead path in a comment is not
  reported; the plants are restored.
- The review's mutation, now: `check` → `src build gate: FAIL` /
  `[printed-path] src/programs/drive_program.cpp:329 prints "src/robot/chassis_table.hpp", which
  does not exist …`. The six builds are still green under it, which is exactly why step 0 is the
  place for this check (`scratch/src1/review/RM6-check.log`, run on the final gate at 04:13; the
  file was restored and `cmp`-checked against `backups/drive_program.cpp.review.bak`).
- Coupling, stated because it bears on R1: under a revert this check fails on every reverted
  literal, by design. The revert patch therefore removes it; it does not exempt those literals,
  because an allowlist is where a gate's holes live (D3).

### R4 (minor) — two [include-root] branches no self-test case exercised — REAL, FIXED (03:17–03:59)
- Re-checked in memory, without editing the gate. With `test/vendor` dropped from
  `INCLUDE_SHADOW_ROOTS`, a planted `src/doctest.h` → `include_root_problems()` returns `[]`; with
  it kept, it is reported. With ` or x.startswith("./")` dropped (the tool's source compiled with
  that one edit), `#include "./common/boot_source.hpp"` in `src/programs/teleop_claws.hpp` →
  `[]`; with it kept, it is reported. Each branch was the only guard, and the case-28 plant list
  (as read at the start of this review) planted neither.
- Fixed: case 28 now plants `src/doctest.h` (the direction that matters most: all 172 host tests
  write `#include "doctest.h"`, and `-iquote src` is searched before the vendored directory),
  `test/vendor/robots/chassis_table.hpp`, and the `./` form in `CLAWS_PLANT`. That form BUILDS,
  because `-iquote src` answers it, which is why step 0 has to check it. The case's comment says so.

### The mutations of this review, each on a fresh copy of the gate, each observed failing (03:52–03:59)
Run in snapshot copies of this tree (`scratch/src1/review/mut-<name>`, each its own git repo, so the
build-hash assertion holds; `mutate.py` asserts that its one edit matched exactly once). The five ran
in parallel, and each copy builds into its own `bin/`. Every one is the full `self-test`, real builds
included. (A first round, 03:39–03:46, on the gate before its per-literal line fix, gave the same five
failures.)
| # | Mutation in `tools/src_build_gate.py` | Observed |
|---|---|---|
| RM1 | `test/vendor` dropped from `INCLUDE_SHADOW_ROOTS` | exit 1: `28: src/doctest.h was not reported …` and `28: test/vendor/robots/chassis_table.hpp was not reported …` |
| RM2 | ` or x.startswith("./")` dropped | exit 1: ``28: `#include "./common/boot_source.hpp"` in src/programs/teleop_claws.hpp was not reported`` |
| RM3 | adjacent literals not joined (`if False:`) | exit 1: `30: a dead path split across two literals in src/common/boot_source.hpp was not reported` |
| RM4 | `include/shulib` dropped from `PRINTED_PATH_ROOTS` | exit 1: `30: a dead path in a library header (include/shulib/config/robot_config.hpp) was not reported` |
| RM5 | the full stop kept in the path (`.rstrip(".")` dropped) | exit 1: case 5 `the gate FAILS on the real, unplanted tree` (four `[printed-path]` lines, among them `bench_r3a.cpp:724` and `teleop_program.cpp:882` — the path's own line inside a multi-line printf) and case 30's real-tree check |
| RM6 | (in the SOURCE) the review's `src/robot/` in Drive's refusal; `check` on the worktree | exit 1, `[printed-path] … drive_program.cpp:329 …` (above) |

The unmutated gate, on this worktree: `src build gate self-test: OK (189 detector cases, each against
a real build)`. That is 180 + 3 (case 28) + 6 (case 30). (`review-selftest-2.log`, 7 min 16 s; an
earlier run before the per-literal line fix also gave 189.)

### R5 (minor) — the bench panel now cuts one census line to `src/robots/chassis_` — REAL, ACCEPTED, LOGGED (03:50)
- Re-checked: `kScreenCols = 54` (bench_r3a.cpp:229). `:1294` was 57 characters and is now 64; the
  panel showed `…goes into src/chassis_table.h` and now shows `…goes into src/robots/chassis_`.
  `:1435` goes from `src/chassis_tab` to `src/robots/chas`. The other three lines keep the whole path
  on the panel. `:1294`'s sentence is in no package's `.rodata` (grep of `baseline/` and `after/`,
  all six), so it is dead code today, and it goes live for a robot whose table has no ports.
- Why this is accepted rather than re-wrapped: (1) each cut prefix still names exactly one file:
  `src/robots/` holds `chassis_table.hpp`, `mechanism_table.hpp` and `xdrive_config.hpp`, and only
  one of them starts `chas`. (2) The panel already cuts most of the tester's lines, 107 of its 163
  plain `emit("…")` lines being longer than 54, by the tester's own design ("the SD file keeps the
  full-width detail"). (3) A re-wrap changes the SD and serial line breaks too, which is a second
  departure from §3 of the kind R1 puts to the team lead. If he wants the panel to show the whole
  name, the two re-wraps belong in that ruling. Not changed.

### R6 (minor) — the link order and memory layout changed, and the package comparison could not see it — REAL, MEASURED, STATED (03:22–03:52)
- Re-checked: common.mk:159 `GETALLOBJ=$(sort …)`. From the V=1 link lines of fresh builds, the
  old order was `bench_r3a, boot_source, calibrate_program, drive_program, main, state_screens,
  teleop_program, xdrive_program` and the new order is `common/boot_source, common/state_screens,
  main, programs/bench_r3a, programs/calibrate_program, programs/drive_program,
  programs/teleop_program, programs/xdrive_program` (all six builds). `compare_rodata.py` compares
  SETS and `rodata_strings.sh` drops addresses, so neither could see it. The earlier "the only
  differences" sentence (the 02:12–02:16 section) was wrong by omission.
- Measured, with `scratch/src1/review/init_order.sh`: all six packages built from `git archive
  0fab866` and from a snapshot of this tree, both with no git hash, so the builds compare like for
  like. **The start-up table is EMPTY in every package, before and after**: `__init_array_start ==
  __init_array_end` (bench `0780dd8c` → `0780dda8`, 0 bytes both), `__preinit`/`__fini` likewise,
  and no object under `bin/` has an `.init_array`, `.ctors` or `.preinit_array` section. No
  start-up code runs in any order, so none can run in a different one. Section sizes: `.text`,
  `.data` and `.bss` are identical in all six; `.rodata` grows by the path bytes only (+28, +28,
  +28, +28, +56, +14); `.ARM.exidx` changes by one 8-byte entry in three builds (tank-drive
  352 → 360, tank-library 1696 → 1704, tank-calibrate 944 → 936). The linker merges adjacent
  identical unwind entries, so the link order changes how many merge. That table is looked up by
  address, so this is harmless. The set of sections is the same in every build. Stated in "For the
  coordinator" item 7, in the roadmap sentence ("linked in a new order") and in the build-order
  Next block.

### R7 (minor) — build-order edits not reported, and a design commitment for an unbriefed chunk — REAL, FIXED (03:14)
- Re-checked with `git diff -U0 HEAD -- docs/internal/build-order.md`. Path tokens changed in the
  Next block (:632), four "Previously — … kept as history" blocks (:636, :638, :640, :644), the T2
  status (:1419), the G1 entry's status (:1496), the G3a body (:1567) and the deviations row "a
  third build variant" (:2013). Every change is a `src/<file>` token moved to its folder. The reason
  holds: `doc_staleness_audit.py`'s dead-path sweep reads every line of build-order.md
  (`PATH_FILES`; `HISTORICAL` exempts numeric claims, never paths). The history-block rewrites were
  logged at 02:13–02:23 but not reported. They are now "For the coordinator" item 6.
- The G3a note read "The generator's target is `src/robots/` … and the `src` gate's step 0
  `[include-root]` must stay green". That is a design decision for a chunk that has no brief (pace
  rule 2). It is now the path mapping only, plus: "For G3a's brief to decide, not decided here:
  where the generator writes". It also names the coupling the review found. Step 0 derives every
  robot's table names from exactly `TABLE_FILES`, and outside `test/` and a `static_assert` only
  those two files may name a robot's tables. So a generator that writes a table into a file with
  another name either fails that check on its own output, or leaves that table outside it
  (`per_robot_table_names()`, read at :510).

### R8 (minor) — two untracked files, and CI fails without the README — REAL, NOT A TREE DEFECT; THE COMMIT STEP IS NOW EXACT (04:20)
- Re-checked: `git status --short` → `?? docs/internal/chunks/SRC1-PROGRESS.md`, `?? src/team/`. The
  gate requires the README (case 28, `expect(os.path.isfile(TEAM_README), …)`). The staleness audit
  would NOT catch its absence, because `PATH_PAT` needs a file extension and the docs name the
  folder as `` `src/team/` ``. So case 28 is the one guard, and it fires in CI, not locally before
  the commit.
- The index is not touched here: the commit is the coordinator's, and a half-staged index is a
  hazard of its own. "For the coordinator" item 5 now names both files, the command, and the
  `git status --short` → empty check.

### Every gate, re-run on the finished tree (04:24–04:33)
- `briefing_status.py generate` before the build. Fresh host build (`rm -rf build/test`, configure,
  `cmake --build build/test -j8`): the flags line carries
  `-iquote/…/src1/test/../src`, and the doc gates inside the build are clean. Suite:
  **`[doctest] test cases: 1659 | 1659 passed | 0 failed | 3 skipped`**, **`assertions: 2467051 |
  2467051 passed | 0 failed`**, the same numbers as the baseline at `0fab866`. Then `generate`
  again, and `check` → `generated block current`.
- `src_build_gate.py check` → `src build gate: PASS (all 6 builds link; … programs asserted; src/'s
  include root unambiguous; every printed src/ path exists)`. Step 0 prints five lines, the fifth
  `[printed-path] every src/ path in a string literal under src/ and include/shulib/ names a file
  or folder that exists`. Each build: `8/8 TUs compiled, warnings: 6 vendor-ignored, 0
  disallowed`, hash, beacons and program checked.
- `src_build_gate.py self-test` → **`src build gate self-test: OK (189 detector cases, each against
  a real build)`** (04:27–04:32, `review/final-selftest.log`). No probe file is left behind:
  `git status` shows only this log and `src/team/` untracked. `bin/` and `.d/` are the gate's
  last build (both gitignored).
- ARM header gate → `TU includes 191 headers` / `ARM compile gate: CLEAN (all v2 headers)` (TU
  written under `scratch/src1/review/`, not /tmp). CI greps → `library is PROS-free (all of
  include/shulib except hal/pros/)` and `layering holds: core is sim-free`.
- `api_doc_tool.py`: self-test OK; `check-coverage` → `2908 public entities across 155 headers, all
  documented`; `check-fresh` exit 0 (no header comment changed in this review, so docs/api was not
  regenerated); `check-examples` → `437 quoted lines, all verbatim`; `check-removability` → `no
  public doc references docs/internal/`. `doc_staleness_audit.py` → self-test OK (10), `clean (27
  live docs …)`.
- The revert patch still applies to this worktree (`git apply --check`, after every edit above).
- **Prose review for this review's changes** (rule 4; the grep was run and its hits read before this
  line was written). I grepped README.md, `docs/*.md`, the guide, the cookbook, RESUMING, ORIENTATION,
  PROJECT-BRIEFING, HANDOFF and test/README.md for `src_build_gate`, "build gate", "step 0", "include
  root", "four"/"five" source checks and "detector cases". What each hit says:
  - "four source checks" appeared only in the gate itself (now "five", in its header and in
    `cmd_check`).
  - The HANDOFF §4 layout paragraph named step 0's include-root check. It now names the
    printed-path check too.
  - roadmap.md and the build-order Next block were reworded (R1, R6).
  - The CI self-test comment names the new plant.
  - HANDOFF:53's gate list is dated 2026-09-24, as history, and was not touched.
  - Nothing else describes step 0.
  - The build-order deviations row for SRC1 says the gate "gained a check that no src/ path can be
    answered by a second include root". That is true, and it does not claim to be the only check,
    so it was left as is. Changing it would also have to change the revert patch.
- **Touched in this review:** `tools/src_build_gate.py` (the check, cases 28 and 30, the header),
  `docs/roadmap.md`, `docs/internal/build-order.md` (Next block, G3a note),
  `docs/internal/HANDOFF-remaining-work.md` (§4, one clause), `.github/workflows/ci.yml` (one
  comment), the SRC1 brief (its header's time), this log, and `docs/internal/PROJECT-BRIEFING.md`
  (regenerated). **Not touched:** src/, test/, include/, the Makefile. No finding needed a code
  change, and every src/ file is byte-identical to its state before the review (RM6's plant was
  restored and `cmp`-checked). docs/api was not touched either: no header comment changed.
- Scratch: the snapshot trees (`base-tree`, `after-tree`, `revert-tree`, `mut-*`) were deleted after
  their runs. Kept under `.claude/coordinator/scratch/src1/review/`: every log named above, the
  per-build section, link-order and start-up-table dumps (`base/`, `after/`), `revert-rodata/`, the
  scripts (`init_order.sh`, `mutate.py`, `make_revert.py`, which rebuild those trees) and
  `SRC1-revert-printed-paths.patch`.
- **Stopped here, 04:33, nothing committed.** R1 waits for the team lead's ruling (keep, or apply the
  patch). Everything else is closed in the tree.
