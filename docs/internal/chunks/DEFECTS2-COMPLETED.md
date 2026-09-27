# DEFECTS2 — COMPLETED (landed 2026-09-26; host- and build-gated, NOT run on a robot)

> Brake modes now reach the V5 motor. Written FROM [`DEFECTS2-PROGRESS.md`](DEFECTS2-PROGRESS.md) (the
> executor's live log, including its review round) and the coordinator's merge verification below, not
> instead of them. Brief: [`DEFECTS2-brake-modes-reach-the-motor.md`](DEFECTS2-brake-modes-reach-the-motor.md)
> (committed `9325490`). Executor model: `opus`, in the isolated worktree `.claude/worktrees/defects2-lib`
> (branch `defects2-lib`), in parallel with T3's library half — the team lead's "spawn up some agents make
> sure they dont conflict" (2026-09-26).

## The one-sentence honest status

A Brake or Hold that THIS program commands now stops the V5 motor through PROS's `brake()` instead of a
mode-ignoring `move_voltage(0)`, Coast is byte-for-byte unchanged, and both driver loops keep the drive
COASTING on stick release — proven against a PROS shim that now models `brake()`, and **never yet on a
robot**.

## Built

- **`hal/pros/motor.hpp` (`ProsMotor`)** — the stop is decided in one place (`stopAtZero()`), on the
  millivolts actually SENT, keyed on `commandedMode_` (the mode this program set through `setBrakeMode`),
  never on the device-seeded `brakeMode_` (the constructor's T7 fallback); one `brake()` per transition
  into zero; a refused mode write is re-sent and a stop is believed only when both calls were accepted;
  `setBrakeMode` re-issues the stop if already at zero, so call order never matters; Coast keeps
  `move_voltage(0)`.
- **`teleop/teleop_loop.hpp`** — `DriverLoopConfig::releaseBrakeMode` (default Coast, the team lead's
  ruling), applied to every drive motor at loop entry and after every `drive()`; a disable still cancels
  into Brake.
- **`src/main.cpp`** (at merge, by the coordinator) — the X-drive's `runTeleopLoop` sets Coast on the drive
  at entry and after every `drive()` too (the executor's merge finding 1); 2c removes that loop.
- **The PROS shim** — `Motor::brake()`, a per-port control state, `brakeCalls`, refusable mode writes.
- **Tests** — adapter a1-a11 (+a9b), stack s1-s8 (`test/brake_stop_stack_test.cpp`): mechanisms on
  ports 6/7 under Hold, a full robot's cancel over the real provider, the loop's release mode, the
  seeded-mode trap; 21 executor mutations red.
- **Docs** — HA-142..HA-146 new; HA-53, HA-92, HA-98 amended; the include/ comments made true; the
  changelog ("Fixed since 2.2", no version bump — no signature changed); the worksheet's F.2.3/F.2.7 (the
  release coasts; a disable is a poor test of Brake, HA-145); build-order and roadmap pointers; docs/api
  and the briefing regenerated.

## The coordinator's merge verification (2026-09-26 ~04:40)

Squash-merged from the transport commit `ac383f6` onto `9325490`; four conflicts resolved (the driver-loop
header's doc paragraph — both sides kept; the register's HA-141 row — G1 Part 2b's kept, HA-142..146
added; `docs/api/teleop_loop.md` and the briefing — regenerated). Every gate re-run on the merged tree:
suite **1464 | 1464 passed | 0 failed | 3 skipped**, 2,387,533 assertions; `src_build_gate.py` self-test
OK and check PASS on all six builds (0 disallowed, beacons); the ARM header gate (184); both CI greps
empty; `check-fresh` 0; the briefing current (regenerated AFTER the build — the stale-binary trap caught
it first); removability empty. The adapter's `stopAtZero()` read in full. Two hand mutations on fresh
binaries, restored with `cmp`: keying the stop on the device-seeded `brakeMode_` → 1 red (the
seeded-mode trap); never calling `brake()` → 17 red, 76 assertions.

## What is NOT done

- **Nothing here has run on a robot.** HA-143 (what 0 V does physically), HA-144 (whether a repeated
  `brake()` under Hold re-captures the position), HA-145 (Hold across a disable) stay open; the claws'
  Station J and COMP's first auton stop are where they settle.
- The library default `DriverLoopConfig::rezeroButton = L1` (a claw pivot's button) — owed with T3/COMP.
  *(Update, T3 Part 2, 2026-09-26: the default is still L1, and it is now GUARDED rather than moved:
  `teleop::validBindings()` — `runDriverControl`'s precondition before its first tick, since T3
  Part 1 — refuses any binding on the loop's re-zero button, so the collision cannot run silently;
  "shulib Teleop" names Y (the team lead's ruling), `static_assert`-tied to the mechanism table's
  `kTeleopRezeroButton` and checked against every binding button of both tables at compile time,
  and validates its set against it in `teleopInitialize()`. COMP inherits the same rule through the
  same loop. A program with no bindings still gets L1, harmlessly.)*
