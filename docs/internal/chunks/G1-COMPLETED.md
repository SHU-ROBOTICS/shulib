# G1 — COMPLETED (landed 2026-09-18 → 2026-09-27; host- and build-gated, NOT run on a robot in its final form)

> A robot described as DATA and built by the library, and every program on the robot started through
> that one front door. Written FROM [`G1-PROGRESS.md`](G1-PROGRESS.md) (every part's live log: Part 1;
> Parts 2a, 2b, 2c, 2c's review round, "Part 2c final" with the team lead's rulings and its review round)
> and the coordinator's merge verifications, not instead of them. Briefs:
> [`G1-robot-config-and-builder.md`](G1-robot-config-and-builder.md) (Part 1) and
> [`G1-part2-pros-provider-and-main.md`](G1-part2-pros-provider-and-main.md) (Part 2: 2a / 2b / 2c,
> revised after the 2026-09-24 audit; 2b's amendments A1–A19). Executor model: `opus` throughout.

## The one-sentence honest status

`config::RobotConfig` describes a robot as data, `robot::Robot` builds the whole graph from it over a
device provider (the fakes on the host, `hal::pros::ProsDeviceProvider` on a brain), "shulib Teleop" and
the invented X-drive's program boot through that front door, and `src/main.cpp` is a 253-line
dispatcher whose one build chain in `src/build_selection.hpp` picks each build's program AND robot —
all host- and build-gated, and **no version of 2b's or 2c's programs has run on a robot yet** (the
hand-wired Teleop they replaced drove robot two on 2026-09-25).

## Built, part by part

- **Part 1 — `fe8e270` (2026-09-18):** `config::RobotConfig` and `config::Route` (plain data), the
  `IRobotConfig` / `IRouteSource` seams, `hal::IDeviceProvider`, and `robot::Robot{config, devices}` — a
  robot from data, host-proven.
- **Part 2a — `2019ada` (2026-09-24):** `hal::pros::ProsDeviceProvider` (registry-checked, a port refused
  by name), absent drive ports taken from the boot check (not re-derived by the builder),
  `Robot::portMap()`, and `teleop::runDriverControl` (PROS-free, host-proven).
- **Part 2b — `7f9b9a1` (2026-09-26):** "shulib Teleop" rebuilt on `robot::Robot` over the provider, from
  the shared boot source `src/boot_source.{hpp,cpp}` (SD record → typed table → refuse), driving through
  the library loop with the `DriveShaper` default profile; `main.cpp` 1,340 → ~720 lines.
- **Part 2c and its final run — this commit (2026-09-26/27):** `src/main.cpp` a dispatcher; the invented
  X-drive a `constexpr RobotConfig` (`src/xdrive_config.hpp`) built on `robot::Robot`
  (`src/xdrive_program.cpp`), refusing at boot on every robot that exists; the state screens moved to
  `src/state_screens.*`; `runTeleopLoop` gone. **The team lead's rulings of 2026-09-26 (~13:15):** ONE
  chain, in `src/build_selection.hpp`, reads the build switches and sets the program, the robot
  (`kRobotId`) and the variant beacon in every branch; `src/chassis_table.hpp` and
  `src/mechanism_table.hpp` choose by that constant with no `#if` (every robot's table under its own name,
  so the host tests read the real ones); a robot beacon (`[BLD] robot tables: …`) derived from
  `kRobotId` and asserted per build; no program may name another robot's table outside a
  `static_assert` (the review's major finding); and `-Wunused-function` is a gate FAILURE. The src build
  gate: step 0 (the one-chain rule over `src/`, `include/shulib/`, `test/` and the host CMake; the chain
  table against the gate's per-build tables; the per-robot-name rule), step 6's robot half, step 8's
  program check from each package's symbol table; 157 detector cases in its self-test.

## Deviations (each logged with its alternative in `G1-PROGRESS.md`)

1. `-Wunused-function` is a failure — the team lead's ruling overrides the Part 2 brief §2c item 4.
2. The robot is named by a second, derived beacon rather than folded into the variant beacon.
3. The chain's robot constant is `kRobotId`, not the ruling's example `kRobot` (the library documents
   `inline constexpr RobotConfig kRobot{…}` as the way to type a robot).
4. Step 0 reads `include/shulib/`, `test/` and `test/CMakeLists.txt` as well as `src/`.
5. The X-drive build carries `kNoChassis` (an unset table), not the bench bot's.
6. The per-robot-name rule enforced by the gate, not by a namespace (a namespace only makes uses easier to
   find).

## The coordinator's merge verifications

- Parts 1, 2a, 2b: their commits' messages and the log's verification sections.
- **Part 2c final (2026-09-27, this commit):** the g1-2c-final worktree (2c squashed onto `cc513a7`, then the
  rulings run — cut off by the 2026-09-26 afternoon power loss and resumed at 20:20 — and its review round)
  as the transport commit `63b8cd2`, squash-merged onto `f6274e1` (T3 Parts 1+2, DIAG1 Part A, COMP-a, the
  next-wave briefs). Conflicts, all documentation: `README.md` and guide 07 (2c's text, plus T3's
  `src/teleop_claws.hpp`), HANDOFF §7's status sentence, `docs/api/robot.md` and the briefing
  (regenerated). The merge list's stale sentences fixed in T3 Part 2's files (`teleop_program.{hpp,cpp}`,
  `boot_source.cpp`, `teleop_loop.hpp`, HANDOFF). The COMP brief gained **B29** (the seventh build under the
  one chain; re-checked against T4); the G3a entry gained the table-shape constraint; the Part 2 brief's
  §2c item 4 is annotated as superseded. **Verification on the merged tree:** suite **1659 | 1659 passed |
  0 failed | 3 skipped**, 2,467,051 assertions; `src_build_gate.py` self-test **OK (157 detector cases)**
  and check **PASS** on all six builds (8/8 TUs each, 0 disallowed, no category allowed; variant and robot
  beacons and the program checked); the ARM header gate clean (191 headers); both CI greps empty;
  `check-fresh` 0; the briefing current; removability empty; the staleness audit clean. Two coordinator
  mutations: robot two's selection returning the bench bot's table → host RED; a program naming
  `kRobotTwoChassis` (`src/bench_r3a.cpp:158`, the review's surviving edit) → `src_build_gate.py check`
  **FAILS** at step 0 naming the file, the line and the rule. The builders' own sets: 31 runs (final) and
  R1–R7 + G1–G8 (review round), all caught.

## What is NOT done

- **Nothing of 2b's or 2c's programs has run on a robot.** "shulib Teleop" as rebuilt (and with T3's
  claws) meets robot two at worksheet Stations F and J; every package now prints `[BLD] robot tables: …`
  as its first boot line.
- The four sentences `main.cpp` paints itself (the DISABLED, FIELD CONTROL, tester and Calibrate
  AUTONOMOUS reasons) are not tied to a program by any check (2c's review; logged).
- Two X-drive details have no host test because `src/*.cpp` is not host-compiled (its fault-latch
  attach and its heading-trust clause; logged). *(T4 Part 1a, 2026-09-27: the heading-trust clause has
  one now — the owed shared helper, ruling R8, was made: Teleop and the X-drive both call
  `driverHeading()` in `src/common/driver_controls.hpp`, and `test/driver_controls_test.cpp` flips each of
  its three terms alone (T7). The fault-latch attach still has none.)*
- The next program to join the chain is the competition program (COMP-b, amendment B29) and the driver
  program (T4, being briefed).
