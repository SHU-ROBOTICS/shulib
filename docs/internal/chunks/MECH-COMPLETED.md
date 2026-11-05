# MECH — the mechanism core — COMPLETED *(landed 2026-11-06 with its sixth and last run, MECH-A6; host- and build-gated, NOT run on a robot)*

> The mechanism core: a team DECLARES each mechanism once — what it is (a power roller, an arm, a lift, a flywheel,
> a pneumatic), in its own units — WIRES it separately (where each role is plugged in), and the library builds,
> checks, runs, drives, routes and direction-checks it. Six executor runs, each its own go, strictly in order
> (the brief's §2): A1–A6 landed — **A6 on 2026-11-06**, after its review, fix round and fresh verifier and
> the coordinator's merge verification (MECH-PROGRESS.md's last section). Written FROM [`MECH-PROGRESS.md`](MECH-PROGRESS.md) (every run's steps,
> deltas, mutations, review and verification rounds, and the coordinator's merge record for A1–A5) at A6's fix
> round, as the brief's §4.1 asks ("at A6's close `MECH-COMPLETED.md`") and RESUMING's pace rule 5 requires (a chunk
> without its record is not closed). Brief: [`MECH-core-mechanism-library.md`](MECH-core-mechanism-library.md) (its
> "Re-check 2026-10-04 — amendments" override its body; the team lead's go of 2026-10-08, "Accept all, go", took
> N1–N13 as recommended). **Two lines are the coordinator's at the merge:** A6's landing commit below, and the
> status pointers that flip with it (the roadmap's MECH rows `[~]` → `[x]`, build-order's MECH entry and first
> bold `Next:`, REMAINING-WORK row 12) — main has moved since A6's base `81eb666`, so they are written against the
> tree the merge makes, not this worktree's.

## The one-sentence honest status

Every mechanism a robot declares is now built, bound by role, run by ONE service, driven by an auton step, a
driver-file control or a route marker, and direction-checked on the bench page — all of it proven on the host test
plant and in all seven builds, **none of it on a robot**: robot two's claws (two Power pivots on ports 6 and 7, a
pneumatic claw on 3-wire A) are declarations whose ports, directions and open level are still REPORTED or GUESSED
(HA-153, HA-159), and every number the core starts from is invented (HA-182..HA-186).

## The six runs

| Run | What it built | Landed |
|---|---|---|
| **MECH-A1** — the binding and the devices | `mechanism/kinds.hpp` (the declarations), `spec.hpp` (every mistake refused at build BY NAME), `wiring.hpp` (`wire(...)`, the roles), `devices.hpp` and the read-fault seam (`hal::IReadFaults`), the route ids derived from the declarations, the compile-refusal harness in CI | 2026-10-15, `fd36e07` |
| **MECH-A2** — the service, power, pneumatic, the guard's cut | the mechanism SERVICE (one per program, ticked by every wait), power with its done-conditions and pneumatic with its positions, the evidence and read-fault rules; the run guard's deadline and abandon CUT a claimed mechanism (a behaviour change) | 2026-10-15, `98392b2` |
| **MECH-A3** — position and speed | arms, lifts (spool, linkage, measured) and flywheels: every homing, the move and hold loops, gravity terms, both limit rules and the soft stop, the stall watch, "at speed"; the starting values (HA-182) and `.tuning` | 2026-10-22, `43ac361` |
| **MECH-A4** — autons, the driver loop, the direction judgement, the route binder | a mechanism step on one line of an auton (API 2.5), the driver file's controls, every mechanism's route ids bound in one call, `judgeDirection` / `judgePneumatic` (the verdict naming the robot-file line; BackDriven, A4-D28) | 2026-10-29, `46c874c` |
| **MECH-A5** — robot two and the programs | robot two's claws as declarations in `src/robots/mechanism_table.hpp`, ONE service in every program that boots it, `src/common/driver_file.hpp` and robot two's three-line driver file, the per-robot namespace rule in the src gate | 2026-10-30, `81eb666` |
| **MECH-A6** — station 13 and the direction check page | Bench Tests' **station 13 MECHANISMS** (`src/programs/bench_r3a.{hpp,cpp}`): every declared mechanism in its declared units, powered one at a time behind six gates, the direction check run on the robot, `[R3A] mech` RECORD lines; the menu 3 × 5 (N5) | **LANDED 2026-11-06** (built 2026-10-30; review fix round 2026-11-03; verified 2026-11-06) |

## What is proven, and where

- **The host suite** — every run's adversarial cases, each naming the bug it catches; at A6's fix round **2,775 |
  2,775 passed, 3 skipped** (A6's two files: `test/bench_menu_grid_test.cpp`, `test/bench_mech_station_test.cpp`).
  The core's behaviours run on the **host test plant**'s invented models (HA-183), never a real mechanism.
- **The compile-refusal harness** (`ctest -R compile_refusal`, CI since A1): every named build refusal is a
  translation unit that must FAIL with its own reason — 255 of 255 at A6.
- **The ARM header gate** (every `include/shulib` header, −Werror, Cortex-A9) and **the src build gate** (all seven
  builds link, zero disallowed warnings; its self-test planting each detector into a real build) — at every run.
- **Mutations** — every §3 mutation of every run RUN on a fresh build of every dependent object, observed red and
  restored; each review round's survivors became tests (MECH-PROGRESS, each run's mutation entry and its
  "review findings and their disposition").

## What has NOT run — the honest limits

- **Nothing in the core has run on a robot.** Worksheet **Station P** (Bench Tests' station 13) is the first run of
  the bench page and the first powered look at robot two's claws; **Station J** (their first run from the
  controller, "shulib Drive") and **Station K** (the competition program) follow.
- **Open register entries the core owns** (none settled): **HA-182** (every starting number, the bench page's own
  among them), **HA-183** (the host test plant's mechanism models), **HA-184** (a VEX limit switch reads 1 when
  pressed, and an unplugged line reads released — Station P.7.1 when a robot first declares a switch), **HA-185**
  (robot two's claw moves within 0.5 s), **HA-186** (a rotation sensor is absolute within one turn at power-up and
  after a program restart — Station P.7.2 when a robot first declares one); and robot two's own **HA-153** (the
  pivots' ports and direction) and **HA-159** (the claw's line and its open level), each Station P's to check first.
- **"MECH on hardware" is the freeze trigger** (the brief's Q26): the mechanism seam (roadmap register **F11**) and
  the sequence engine (**F12**) stay open by design until the core's mechanisms have run on a robot and a second
  real consumer has stressed them. Nothing here froze anything.
- **The bench page's numbers are invented for a first run** (its 3 V start, its 1 s announcement and 0.4 s check,
  its "did not move" and "drawing" thresholds, its cuts — HA-182's page entries), and the screen says so.

## Deltas from the brief — the index

Each run's deltas, each with its reason, are in [`MECH-PROGRESS.md`](MECH-PROGRESS.md) under "Deltas from the
brief (MECH-A*n*)": A1 and A2 inside their run sections; A3, A4 (A4-D28: the ninth finding, BackDriven; A4-D29:
every role-naming finding has a generated-file sentence), A5 and A6 under their own headings — and A6's review
round added its own (A6-D18 onward, in "MECH-A6 review findings and their disposition").

## Handed on (not the core's to finish)

- **CAL1 Part B** maps the core's mechanism list into Calibrate's facts (N9; A5's merge record).
- **VIS1-B, L6-B and IMU1 Part B** plan their stations around A6's `bench_r3a.*` edits (MECH-PROGRESS, A6's
  "every function touched"); the IMU1 brief's "the 3 × 4 menu is full" is stale since A6 (logged for the
  coordinator, not edited here).
- **G3a** (the robot file and its generator) adopts the core's declarations and refusals (the brief's §6).

## Documents

The core's public chapter is [guide 18](../../guide/18-mechanisms.md) (its bench page section since A6); guide 10
(the provider's read faults), guide 14 (the honest limits), the cookbook's driver-control recipes, the changelog's
`## API 2.5` entries, the register's HA-182..HA-186; internally the brief, `MECH-PROGRESS.md`, the bench worksheet
(Station P; Station J's claws), build-order's MECH entry, the roadmap's MECH rows, REMAINING-WORK row 12 and the
HANDOFF.
