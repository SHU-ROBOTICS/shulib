# SRC1 — COMPLETED (landed 2026-09-27; host- and build-gated)

> `src/` organized into `robots/`, `team/`, `programs/` and `common/` — the team lead's layout ("the src
> folder is getting very crowded is there something we can do about this?", then "Yes, this layout" and
> "Now, before the new code is built"). Written FROM [`SRC1-PROGRESS.md`](SRC1-PROGRESS.md) (the build,
> the review round and its fixes) and the coordinator's merge verification. Brief:
> [`SRC1-organize-src.md`](SRC1-organize-src.md). Executor model: `opus`, in the isolated worktree
> `.claude/worktrees/src1`.

## The one-sentence honest status

Twenty files moved with `git mv` (names unchanged) into four folders with one job each; every include is
written from `src/` under an unambiguous include root that the gate now enforces; every build compiles the
same translation units and the suite is identical (1659 cases, 2,467,051 assertions) — the only behaviour
a user can see is that the boot lines and messages that name a `src/` file now name its new path.

## Built

```text
src/  main.cpp  build_selection.hpp
      robots/    chassis_table.hpp  mechanism_table.hpp  xdrive_config.hpp
      team/      README.md (the team's code goes here: T4's driver file, COMP-t's auton list)
      programs/  bench_r3a.*  calibrate_program.*  drive_program.*  teleop_program.*
                 teleop_claws.hpp  xdrive_program.*
      common/    boot_source.*  screen_text.hpp  state_screens.*  fault_latch_hook.hpp
```

- **Include root:** `-iquote ./src` for the robot build (`Makefile`'s `EXTRA_INCDIR`) and the host build
  (`test/CMakeLists.txt`); 40 includes rewritten (32 in `src/`, 8 in `test/`).
- **The gate (`tools/src_build_gate.py`):** `TABLE_FILES` points at `src/robots/` (step 0 was confirmed to
  fail closed before the path was fixed); a new `[include-root]` check fails a `src/` path any second root
  could answer, a `..` or sibling-form include, and a test including `../src/`; a new `[printed-path]`
  check fails a `src/` path inside a string that names nothing. Self-test 157 → 189 detector cases.
- **Documentation:** every live document's `src/` paths (README's tree, guide 07/08/10/14, the register,
  the roadmap, HANDOFF, the build order, the worksheet, the CI comments, 11 `docs/api` pages); the seven
  open briefs carry a dated path-mapping note at the top, bodies untouched; historical records untouched.

## The coordinator's ruling at the merge

**The printed paths changed** (18 literal lines, 20 paths — the `[BLD]` line, boot and refusal messages,
the bench tester's census lines), against the brief's §3 ("print exactly as at `37706ce`"). The builder
was right and the brief was wrong: each line tells a student which file to edit, and the old files no
longer exist; the new `[printed-path]` check keeps every printed path real. The ruling is the
coordinator's, logged; a tested revert patch exists in the coordinator's scratch folder if the team lead
prefers the old text. One census line on the bench tester's 54-column panel now cuts a file name
(`src/robots/chassis_`) — accepted and logged (107 of its 163 lines already exceed 54 columns).

## The coordinator's merge verification

The src1 worktree (the build, then the review round and its fixes) as the transport commit `47e34d8`,
squash-merged onto `0fab866` with no conflicts. **Verification on the merged tree (2026-09-27, 04:38–04:49):**
suite **1659 | 1659 passed | 0 failed | 3 skipped**, 2,467,051 assertions (identical to `37706ce`);
`src_build_gate.py` self-test **OK (189 detector cases)** and check **PASS** on all six builds (8/8 TUs
each, 0 disallowed, variant and robot beacons and the program checked, `src/`'s include root
unambiguous, every printed `src/` path existing); the ARM header gate clean (191 headers); both CI
greps empty; `check-fresh` 0; the briefing current; removability empty; the staleness audit clean.
**Two coordinator mutations, each restored and `cmp`-checked:**

- **G-a:** a build switch in a NEW file under `src/team/` (`zz_probe.hpp`) → `check` **FAILS** at step 0
  naming `src/team/zz_probe.hpp:2` and the one-chain rule.
- **G-b:** the `[BLD]` line's printed path pointed at a folder that does not exist
  (`src/main.cpp:134`, `src/robot/chassis_table.hpp`) → `check` **FAILS** naming `src/main.cpp:134` and the
  missing file. The first aim at this mutation replaced the file's FIRST match, a comment (`:26`), and
  the gate rightly passed: comments are outside the `[printed-path]` rule by design. Logged as a
  mis-aimed mutation, not a gate gap.

The builders' own sets: M1–M11 (the build) and RM1–RM6 (the review round), all caught (`SRC1-PROGRESS.md`).

## What is NOT done

- Nothing here needs a robot, and nothing new has run on one.
- Briefs of finished chunks (R3a, R3b, R3d, T2, T3) keep their old paths — they describe the tree at the
  time; the open ones carry the mapping note.
