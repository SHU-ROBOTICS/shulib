# SRC1 — `src/` organized into `robots/`, `team/`, `programs/` and `common/`

> **Execution brief.** Written 2026-09-27 (before 01:51, when `0fab866` committed it; this line said
> "~02:10", a time after that commit, until SRC1's review corrected it) by the coordinator from the team lead's request and
> answers. **The team lead's go:** "the src folder is getting very crowded is there something we can do
> about this?", then "Yes, this layout" and "Now, before the new code is built" (the driver program T4
> and the auton tester COMP-t are briefed but not built; their builders start on this layout). **One
> executor run in an isolated worktree off `37706ce`. Executor model: `opus`. Do not commit.** Live log:
> `docs/internal/chunks/SRC1-PROGRESS.md`, created FIRST. No Hardware Assumptions entry is expected; if
> one is, stop and say so.

## 0. Why
`src/` holds 22 files at `37706ce` and is about to gain the driver file, its helpers, the auton tester,
the students' auton list and the competition program. A reader cannot tell the team's own code from a
program's plumbing, or what a robot IS from what a program DOES. Each folder gets one job.

## 1. The layout (the team lead's choice — exactly this)
```
src/
  main.cpp              the PROS entry points: hand each one to the build's program   (stays)
  build_selection.hpp   the one chain: which program, which robot                     (stays)
  robots/               what each robot IS
    chassis_table.hpp  mechanism_table.hpp  xdrive_config.hpp
  team/                 THE TEAM'S CODE — where students edit (empty today: T4 adds
                        robot_two_driver.hpp, COMP-t adds the auton list; add a one-paragraph
                        README.md saying what belongs here and what does not)
  programs/             one program each
    bench_r3a.{hpp,cpp}  calibrate_program.{hpp,cpp}  drive_program.{hpp,cpp}
    teleop_program.{hpp,cpp}  teleop_claws.hpp  xdrive_program.{hpp,cpp}
  common/               what the programs share
    boot_source.{hpp,cpp}  screen_text.hpp  state_screens.{hpp,cpp}  fault_latch_hook.hpp
```
File NAMES do not change (only their folders), so history and searches keep working.

## 2. Scope — IN
1. **`git mv` every file** to its folder (history preserved). No code change beyond what the move forces.
2. **Includes are written from `src/`**: `#include "robots/chassis_table.hpp"`, `#include
   "common/boot_source.hpp"`, the same form in every folder and in `main.cpp`. Give the ARM build the
   include root it needs (the Makefile's include flags — e.g. `-iquote"./src"` beside the existing
   `-iquote"./include"`) and the host test build the same (`test/CMakeLists.txt`), so the eight test
   files that include `../src/...` include `"robots/..."` / `"common/..."` / `"programs/..."` instead.
   Decide and log whether `-iquote` or `-I`, and prove a same-named header elsewhere cannot shadow one.
3. **`tools/src_build_gate.py`**: every path it names (the table files it reads the per-robot names
   from, the planted files in its self-test cases — e.g. `src/bench_r3a.cpp`, `src/chassis_table.hpp`,
   `src/mechanism_table.hpp` — and any list of `src/` files); confirm step 0 walks `src/`'s subfolders
   (a switch or a per-robot name planted in `src/programs/`, `src/common/` and `src/team/` must fail it);
   step 8's program table and `main.cpp`'s switches are unchanged. If the gate checks compile flags
   token-exact, account for the new include flag deliberately (not by weakening a check).
4. **The Makefile / `common.mk`**: `src/` is searched recursively already (`common.mk` ~:74 `rwildcard`,
   ~:154); objects land under `bin/<folder>/` — confirm, and confirm `EXCLUDE_SRC_FROM_LIB` (~:172,
   `src/main`) still means what it meant. The CI workflow's comments and any path it names.
5. **Live documents** (they describe the tree NOW) — every `src/<file>` path updated to its new folder:
   the guides, README, the worksheet, `docs/hardware-assumptions.md` (Source lines), `vexbuilder-
   integration.md`, the cookbook, FAQ, the master plan where it names today's files, `build-order.md` /
   `roadmap.md` / HANDOFF status text, `RESUMING.md`, `ORIENTATION.md`, code comments in `include/`,
   `src/`, `test/` and `tools/`. Grep for every old path (`src/<name>` for all 22 names) and account for
   every hit.
6. **Open briefs** (work not yet done: COMP (B-part b), SAFE-SAVE, L9, R5a, H1, DIAG1 Part B, G1's §2c
   note, the build-order G3a entry): do NOT rewrite their bodies — add one dated note at the top of each:
   "Paths moved at SRC1 (2026-09-27): src/X → src/robots/X, …" (the full mapping once, in this brief §1,
   linked). Their executors re-check against the tree anyway.
7. **Historical records** (every `-PROGRESS.md`, `-COMPLETED.md`, the evidence logs, committed briefs of
   finished chunks) are NOT edited: they describe the tree at the time.
8. **`build-order.md`**: an SRC1 entry (the DOCS/DEFECTS row: "reactive — the team lead's request"),
   the chunk count 57 → 58, a deviations-table row.

## 3. Tests and checks (nothing about behaviour may change)
- The host suite: the SAME case count as at `37706ce` (1659), all passing — every test that read a
  `../src/` path now reads the moved file.
- `src_build_gate.py self-test` OK (the case count may grow by the new planted-path cases) and `check`
  PASS on all six builds, variant and robot beacons and programs asserted, 0 disallowed warnings.
- **Mutations, each observed failing then restored:** a build switch planted in `src/programs/`, in
  `src/common/` and in `src/team/` → step 0 FAILS; a per-robot table name planted in a program file
  under `src/programs/` → step 0 FAILS; an `#if` planted back in `src/robots/chassis_table.hpp` →
  FAILS; a stale `#include "chassis_table.hpp"` (old form) left in one program → the build FAILS.
- The ARM header gate, both CI greps, `check-fresh`, removability, the staleness audit, the briefing
  regenerated before the build and again after the suite binary exists.
- The six packages: each links, and each boot banner and `[BLD]` line prints exactly as at `37706ce`
  except for any `__FILE__`-derived text (list those, if any).

## 4. OUT
Renaming any file (e.g. `bench_r3a` → `bench_tests`); any behaviour change; retiring Teleop (T4's);
the team folder's contents (T4, COMP-t); anything under `include/` except comments that name a `src/`
path.

## 5. Landmines
1. PROS object paths: two files with the same name in different folders would collide in `bin/` — none
   do today; the `team/README.md` must not be picked up by a source glob.
2. The gate reads the per-robot names FROM the table files (G1 Part 2c final): moving the tables without
   updating that path makes step 0 check nothing — the gate is written to FAIL in that case; confirm it
   does before fixing the path.
3. `api_doc_tool` checks examples quoted verbatim from files it names — a moved file's quote path.
4. Do not touch `.claude/` (the coordinator's) or any other worktree.
