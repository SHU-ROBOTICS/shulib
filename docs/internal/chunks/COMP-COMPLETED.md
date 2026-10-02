# COMP — COMPLETED (COMP-a landed 2026-09-26, COMP-t landed 2026-09-29, COMP-b landed 2026-10-02; host- and build-gated; NOT run on a robot)

> The competition program — the handoff's L2, briefed 2026-09-24 and built in three parts, each on the team
> lead's go: **COMP-a** the library half (`b5aaf95`), **COMP-t** "shulib Auton Tester" and the team's auton file
> (`0f56301`, its own record [`COMP-t-COMPLETED.md`](COMP-t-COMPLETED.md)), and **COMP-b** "shulib Competition",
> the program a team uploads for a match. Written FROM [`COMP-PROGRESS.md`](COMP-PROGRESS.md) (the live log —
> COMP-a's entries, the coordinator's merge record, then "Part b": every step, the review fixes, the deviations
> with their reasons, the mutation table, every gate line). Brief:
> [`COMP-competition-program.md`](COMP-competition-program.md) (its header ND1–ND6 and the 2026-09-28 re-check
> RC1–RC12 govern COMP-b). COMP-b's executor: Opus 5.5 (1M context), in the isolated worktree
> `.claude/worktrees/comp-b` from `0f56301`, over three runs (the first two STOPPED at the session limit; the
> log records what each left and how the next resumed). **Nothing was committed by the executor; the
> coordinator verifies, merges and commits — COMP-b is "built", not "landed", until then.**

## The one-sentence honest status

The competition program exists and is proven on the host and in the V5 build — its decisions run in the host
suite on the plant with the real watch, the program file's functions that feed and act on them are pinned
whole or by their order (since the review), its boot proves its pacer chain or refuses, and the eighth build
links clean — but **no part of it has run on a robot**: worksheet Station K (the controller's Timed Run,
measured first — HA-162) is its first run, and the field and the competition switch stay unrun at the bench.

## What COMP-b built

- **First, ND3 (b) — COMP-t's shared pieces MOVED to `src/common/`** and the tester re-pointed:
  `guarded_run.hpp` (the boot's chain proof over any `WatchLike` watch; the program-error stop with no watch),
  `screen_touch.hpp` (the one touch read), `screen_paint.{hpp,cpp}` (the palette, the selector's painters, the
  refusal screen), `auton_run_log.hpp` (the SD run log's name and words), the status bits in
  `state_screens.hpp`. The tester's behaviour is proven unchanged: its tests unchanged in meaning (one pin
  rewritten to follow the run log's name through the move); seven mutations on the moved code red through the
  tester's own tests; the gate line unchanged but for the TU count; and — re-measured on the FINAL tree at the
  review round, because two later moves (the SD run log, `sameMenuButtons`) post-date STEP 1's package
  snapshot — the tester's V5 package against `0f56301`'s, both hash-less with the compile time pinned: its 766
  printable read-only strings identical but for PROS's link timestamp, and function by function only the
  move's own differences (the painters out and in, their call sites, the SD-log word chosen through the shared
  table, two linker veneers). The controller's ONE chooser moved from Teleop to `src/common/driver_panel.hpp`
  (B11), proven equal to Teleop's pre-move lines byte for byte; since the review the lines are WRITTEN through
  one shared writer too (one line at a time, line 2 first, at most one per 0.1 s) — in Teleop as well.
- **The program:** `src/programs/competition_program.{hpp,cpp}` over three PROS-free headers —
  `competition_watch.hpp` (the watch: DISABLED or LEFT AUTONOMOUS, latched, the abandon requested every tick a
  run is live), `competition_run.hpp` (one request: the choice LOCKED, the refusals — NONE, a typed-table boot
  (ND2), after a program error (ND1) — `competition::runAuton` under the guard inside ONE try, the
  program-error stop, and the terminal word FINISHED on EVERY path; the report and the SD write only after it
  (ND5); opcontrol's unbounded wait; the driver loop's end, and — since the review — the auton task waiting
  until that loop has LEFT (or its heartbeat stopped for 0.1 s) before it serves; a request served after its
  period reading the status once and running nothing (TOO LATE); the words and lines), and `competition_screen.hpp`
  (the atomic screen owner, each painter's tracker, the menu phase's rule for a touch, every page). The auton
  task idles one tick at a time, logs every status change, paints the menu, never ticks the Chassis (RC3).
- **The eighth build** (`make ROBOT=tank PROGRAM=competition`, beacon `…=tank-competition`, slot 6): the
  Makefile branch, `src/build_selection.hpp`, `src/main.cpp`'s five forwards (280 → 298 lines, inside ND6's
  300), `tools/src_build_gate.py` (its tables and self-test cases 34/35), `ci.yml` and `RESUMING.md`.

## Tests and mutations

New: `test/competition_watch_test.cpp` (W1–W8), `test/competition_run_test.cpp` (R1–R24: every registered auton
through the program's request, a disable and the period's end mid-auton, NONE, ND1, ND2 (and R6b, its reachable
states), RC1 (a)–(c) and the stop before the word (R18), ND5's order, B9's present mechanisms, the first tick after
a long idle, this run's faults only (R19), TOO LATE (R20) and a refusal's precedence over it, the driver loop
leaving first (R21: through a watcher that has watched the heartbeat since boot, as the program's has), each program
error's reason (R22), the levels, stop words, boot and phase lines (R23), Station M's predictions through the
request (R24)), `test/competition_screen_test.cpp` (S1–S14: the owner — RC3's "a paint from a non-owner is
refused" and the handovers in order, a field match end to end, AUTON ENDING, the owner under contention; the menu
phase over every input; every page and tone), `test/competition_source_test.cpp` (P1–P16; P16 the handshake's
store-then-load pair, whole), `test/driver_panel_test.cpp` (D1–D5; D5's alternation in both directions), `test/guarded_run_test.cpp` (G1–G6, the moved pieces), `test/competition_rig.hpp`, `test/source_pins.hpp`
(COMP-t's pin helpers, moved), `guide-17a/b/c`. **Mutations:** 48 on the program and its pins, every one RED on a
fresh build (four first forms did not compile and one first form removed nothing — each re-run correctly; one
survivor exposed an untested counter, now tested and red), plus the move's seven; then the review round's 54 —
the reviewers' 31 survivors and the correctness reviewer's two (in three forms) re-run on the fixed tree, and 20
on the fixes — all RED; then the verification round's 11 — the verifier's seven survivors (V01–V06, V11), first
observed GREEN on this tree, and four more relaxed orders of the pair (X01–X04) — all RED; the tables are in the
live log.

## Gates (the final tree, after the verification round — the live log's last entry has each line)

- Host suite: **1,886 cases / 2,701,750 assertions, 3 skipped — green** (a fresh binary; the start was 1,808,
  the reviewed tree 1,871, after the review round 1,885).
- `src_build_gate.py check`: "PASS (all 8 builds link …)" — `[tank-competition] exit 0, 12/12 TUs compiled,
  warnings: 8 vendor-ignored, 0 disallowed, hash asserted, beacons (variant, robot) and program checked`;
  `[chain-table] 8 builds, 3 robots`; `[program-table] 7 programs, 19 entry names`.
- `src_build_gate.py self-test`: "OK (203 detector cases, each against a real build)" (cases 34/35 among them).
- Doc gates: coverage "2956 public entities across 157 headers, all documented"; examples "520 quoted lines,
  all verbatim (6 source files); 6 listings of whole-run files, each one unbroken run"; check-fresh exit 0;
  removability clean; staleness audit clean; briefing current. ARM header gate exit 0 (193 headers); the CI
  greps: PROS-free, sim-free.

## Documentation

Guide chapter 17 (new), chapters 7, 11, 12, 14, 16, the guide README, the glossary, cookbook 06, the README,
the changelog (no API change; the review's additions), the register (HA-162 new — the ONLY new HA; HA-161 and
HA-178 amended, HA-161's settle anchored on the next phase function's entry line at the review), the worksheet
(Station K new; A.1, the slot map, G.0.5, J.3.1, J.9.6, M's preface, Teleop's controller rows, DO NOT), HANDOFF
(B21, RC10, L5b), build-order, roadmap (M4's guaranteed park reaches a real program — unrun), RESUMING, `ci.yml`,
REMAINING-WORK, ORIENTATION, `mkdocs.yml`, and (at the review) the master plan's and the diagnostics plan's
"competition build" sentences. The prose-review greps, the review's 29 findings and the verification round's three
with their dispositions, and what was deliberately left alone are in the live log.

## The coordinator's merge verification — COMP-b (2026-10-02, 01:10–01:45 EDT)

COMP-b's run — the build, three reviews (29 findings, 27 distinct), the fix round, a fresh verifier (three test
gaps, one major) and its fix pass, across two usage-limit stops and a session end — was committed in the worktree as
`9468a12` and squash-merged onto `7daf809`, which had gained L9 Part A1, DIAG1 Part B, H1 Part A1 and T4 Part 2a since
COMP-b's base `0f56301`. **No code conflict:** T4 Part 2a's protections reach COMP-b's `opcontrol()` through
`teleop::runDriverControl`, as B11 said they would, and the merged suite is green with them on. Document conflicts
resolved by hand: the register (status 7 of 165; COMP-b's HA-162 sentence after T4 Part 2a's; the rows in numeric
order), HANDOFF (T4's status kept, with COMP-b's SAFE-SAVE B2 note), the roadmap (the driver file run by the competition
program's `opcontrol()` since 2026-10-02), build-order's Next line; `docs/api` and the briefing regenerated (35 of 63
records, COMP complete). On the merged tree: the suite **2,036 | 2,036**, 3 skipped; `src_build_gate.py` self-test OK
(198) and check PASS on **all eight builds**; the ARM header gate (197 headers); both CI greps; `check-fresh`, the
briefing, removability and the staleness audit — all clean; no plant. **Six coordinator mutations on the safety
paths, each on a fresh binary, all RED:** FINISHED never published (15 cases); a throw not latching the program error
(6); a throw skipping the safe stop (3); driver control running on into autonomous (1); autons allowed without the
calibration record (1); the menu left locked after a run (4).

**The team lead's C5 ruling (2026-10-02): "Drop just that auton"** — one auton entry the registry refuses no longer
refuses the boot: it is named, left off the menu, and every other auton and driver control still work. Built in T4
Part 3's late step (the competition program's files), not here; until then the rule above ("Not done") stands as built.

## Not done, and why

- **Everything on a robot** — Station K: the Timed Run's behaviour (HA-162), HA-161's settle, B28's claw step,
  the practice match, the mid-auton disable. A field or a competition switch stays unrun at the bench.
- **The drive protections in driver control** — T4 Part 2 (they reach `opcontrol()` through
  `runDriverControl` with no wiring here).
- **Route autons** — refused by name until the team's commands are registered beside the autons (B30).
- **The auton pick under a Timed Run with no disabled window (B6)** — the team lead's, after Station K.
- **One refused auton entry refuses the whole boot (B15/B18 as built)** — the review asked whether ND1's reason
  should keep driver control instead; the team lead's to decide (the live log, "For the coordinator (this
  round)").
- **The pointers** (build-order `Next:`, the roadmap's "You are here") — the coordinator's, at merge.
