# DEFECTS2 — brake modes never reached the motor: make HOLD and BRAKE real at the V5

> **Execution brief.** Written 2026-09-26 (early morning) at tree `b622d9b`, while G1 Part 2b was being
> built in the working tree. Found by the read-only research for T3 (the claw pivots on ports 6 and 7,
> which must HOLD their angle on release) and **re-verified by the coordinator against the vendored
> headers and the tree** before this brief was written. **The team lead's rulings (2026-09-26):** fix
> BOTH Hold and Brake; in shulib Teleop the drive COASTS when the sticks are released (the 2026-09-25
> feel); this is its own step, right after G1 Part 2b: **2b → DEFECTS2 → T3 → 2c → COMP**. It starts
> only after Part 2b is committed, on the team lead's go. **One executor run. Executor model: `opus`. Do
> not commit.** Live log: `DEFECTS2-PROGRESS.md`, created FIRST.

---

## 0. The defect, with its evidence

- **PROS:** `include/pros/motors.h` ~:253-254 (`motor_move_voltage`): *"This function will not respect
  brake modes, and simply sets the voltage to the desired value."* The same note is on `motor_move`
  (~:52). Only `motor_brake()` (~:80-86) *"stops the motor using the currently configured brake mode …
  sets motor velocity to zero, which will cause it to act according to the set brake mode."*
  `set_brake_mode` alone changes nothing the motor does.
- **shulib:** `hal::pros::ProsMotor::setVoltage` only ever calls `move_voltage` (`hal/pros/motor.hpp`
  ~:122-127); `setBrakeMode` only calls `set_brake_mode` (~:142-145); nothing in `include/`, `src/` or
  `test/` calls `brake()` or `move_velocity`. Every stop in the library is "set a mode, then 0 V":
  `MotorMechanism::applySafeState()` (`hal/mechanism.hpp` ~:240-245), the motion cancel safe state
  `applyCancelSafeState()` (`motion/motion.hpp` ~:166-171, "0 V + Brake"), `DriveBrake` every tick
  (`motion/drive_brake.hpp` ~:112-115), the teleop loop's disable path (`teleop/teleop_loop.hpp` ~:137,
  `chassis.cancel()`), and therefore `RunGuard`'s forced-safe paths.
- **On the robot, then:** a mechanism declared `BrakeMode::Hold` goes limp at release; the drive's
  "0 V + Brake" COASTS. `hal::IMotor`'s own contract (`hal/motor.hpp` ~:28-40) already promises the
  opposite ("HOLD — actively drives back to the position the shaft held when the command went to zero").
- **Why no test saw it:** the host plant models 0 V under Brake/Hold as pinned (`sim/drive_plant.hpp`);
  `FakeMotor` echoes the mode; the PROS shim stores `brake` as a field unrelated to how the motor is being
  driven and has no `brake()` (`test/pros_shim/pros/motors.hpp` ~:41-43, :81, :171-176) — its own header
  says it tests the adapter against OUR BELIEF about PROS.

## 1. Scope — IN

1. **The fix, in the adapter only (`hal/pros/motor.hpp`, `ProsMotor`):** when the millivolts actually
   sent round to **0** and the mode **THIS PROGRAM last commanded through `setBrakeMode`** is **Hold or
   Brake**, call `motor_.brake()`. **Key it on a separate "commanded by this program" record, NEVER on
   `brakeMode_`**: the constructor SEEDS `brakeMode_` from the device (`motor.hpp` ~:106-117, the T7
   fallback), so a drive motor an earlier program left in Hold would otherwise `brake()` at centred sticks
   in a program that never asked for it. Otherwise
   instead of `move_voltage(0)`; **Coast keeps `move_voltage(0)` exactly as today** (Calibrate, shulib
   Drive and the tester's DRIVE station all set Coast explicitly and their measured behaviour, HA-137,
   rests on today's 0 V path). In `setBrakeMode`, if the last command was a zero stop, re-issue the stop
   under the new mode so the order of the two calls never matters (Hold→0 V and 0 V→Hold both hold). Key
   the decision on the millivolts SENT (`motorVoltageToMillivolts` rounds, `motor_conversion.hpp` ~:67),
   never the raw double. `commandedVoltage()` stays the applied 0 V.
2. **The ctor's inherited mode stays as it is:** the constructor commands no mode and seeds the T7
   fallback `brakeMode_` from the device's read-back (~:106-117) — keep that; it is why item 1 keys on a
   separate commanded record. A motor this program never gave a mode keeps `move_voltage(0)` at zero,
   exactly as today. (Pre-existing, fix while in the file: `brakeMode()` counts faulted reads, ~:152, while
   the class and member docs, ~:84-88 and ~:214-215, say it does not — make the docs match the code.)
3. **Teleop coasts on stick release (the team lead's ruling).** After the fix, a cancel leaves the drive in
   Brake and a released stick would then brake. The driver loop must guarantee the ruled release mode for
   the whole of driver control: `teleop::DriverLoopConfig` gains a release brake mode (additive; the struct
   is unfrozen), default **Coast**, applied to the drive motors at loop entry and wherever a library path
   inside the loop could have left another mode; on DISABLE the loop still cancels into Brake (that stop
   SHOULD brake). shulib Teleop passes Coast. The X-drive's `runTeleopLoop` (still in `main.cpp` until
   2c) gets the same guarantee or a stated reason why not.
4. **The PROS shim learns the truth it was missing:** `Motor::brake()` with a call counter, and a per-port
   CONTROL STATE (driven by voltage / stopped under the brake mode), so a test can tell "0 V, mode ignored"
   from "stopped, mode applied". The shim's header says what it now models and what it still cannot.
5. **Tests (adversarial):** adapter — Hold at 0 V ⇒ `brake()` and the stopped state; Brake likewise;
   **Coast at 0 V ⇒ still `move_voltage(0)`, no `brake()`** (the control); a non-zero command after a hold
   leaves the stopped state; both call orders (mode then 0 V, 0 V then mode); a sub-millivolt command;
   **a port SEEDED in Hold by the shim (the device left in Hold, `setBrakeMode` never called) ⇒
   `setVoltage(0)` does NOT call `brake()`** (the seeded-mode trap).
   A `MotorMechanism` built on a real `ProsMotor` over the shim: `applySafeState()` with Hold reaches
   `brake()` at the bottom of the stack. The motion cancel safe state over `ProsMotor`s: Brake reaches the
   motors. The driver loop: a cancel followed by released sticks leaves the drive in Coast at the adapter;
   a disable still brakes. **Mutations** (each run on a fresh binary, observed red, restored): revert to
   plain `move_voltage(0)`; drop the Coast branch (Coast would brake); drop the re-issue in
   `setBrakeMode`; key on the raw double; **key on the device-seeded `brakeMode_` instead of the commanded
   record** (the seeded-Hold test must go red).
6. **The host plant stays honest:** state in `sim/drive_plant.hpp` (comment only, unless a test needs
   more) that its pinned-at-0 V model is now what the adapter produces, and until this fix it was not.

## 2. OUT
The claw pivots themselves (T3). Current-limit / stall / thermal monitoring of mechanism motors (T3
decides; register the belief here). An `IMotor::stop()` verb (a pure virtual breaks every implementer and
F4; a defaulted virtual is silently skipped by wrappers — rejected). Any change to Calibrate, shulib Drive
or the tester's behaviour (they use Coast; the Coast control test proves them unchanged). Any hardware
claim: **nothing here has run on a robot** until a worksheet station's log says otherwise.

## 3. Documentation, in the same commit
- `docs/hardware-assumptions.md`, next free numbers (HA-142 onward — re-check at commit time): **new** —
  (a) `move_voltage`/`move` ignore the brake mode, only `brake()` applies it (reasoned: vendored text);
  (b) what the motor physically does at `move_voltage(0)` (UNKNOWN; `IMotor` asserts coast and HA-137
  depends on it); (c) whether a repeated `brake()` under Hold resets the held position (invented); (d) Hold
  while the brain is disabled (UNKNOWN); **amend** HA-53 (the drive's "0 V + Brake" coasted until this fix),
  HA-92 (its premise that the declaration reaches the motor was false at the wire), HA-98 (brake mode is
  device state that persists across programs).
- `include/` comments made true: `hal/pros/motor.hpp`'s class doc, `manipulation/mechanism_op.hpp` ~:63
  ("Hold + 0 V IS the hold"), `hal/motor.hpp` ~:38 where it claims more than PROS documents, the shim's
  header; `docs/api/` regenerated (the freshness gate).
- `docs/changelog.md` — a user-visible behaviour fix; state the `kApiMinor` decision explicitly under
  `include/shulib/version.hpp`'s policy (no signature changes; IMotor already promised this).
- The worksheet (`R3a-BENCH-WORKSHEET.md`): how the fix will be seen — the drive's library stops now brake
  (a disable during driver control; COMP's first auton stop), Teleop still coasts on stick release; the
  loaded-hold proof belongs to T3's claw station.
- `build-order.md` — a DEFECTS2 entry and a deviations-table row (a reactive chunk, +1 to the count, and
  the order 2b → DEFECTS2 → T3 → 2c → COMP with the team lead's rulings), the first `Next:`; `roadmap.md`
  where it states the stop behaviour or the "You are here"; the briefing (regenerated LAST).
- **Prose review** (grep BEFORE claiming it): `move_voltage`, `brake mode`, `BrakeMode::Brake`, "0 V +
  Brake", "0 V + Hold", `applySafeState`, "holds", "coast" across `docs/`, `README.md`, `include/`, `src/`,
  `test/` and `tools/` comments; fix what is false in the files this chunk edits; list the rest.

## 4. Checks and DoD
- [ ] Host suite green (counts before/after); every mutation above red on a fresh binary (`touch
      test/*.cpp` before a `/fast` rebuild; the relink line required).
- [ ] `tools/src_build_gate.py` self-test + check: six builds PASS, 0 disallowed warnings, beacons.
- [ ] ARM header gate, CI greps, `check-fresh`, `check-removability`, `doc_staleness_audit.py`, briefing.
- [ ] §3's documentation; `DEFECTS2-PROGRESS.md`; `DEFECTS2-COMPLETED.md` written by the coordinator.

## 5. Landmines
1. **Coast must not move.** Four programs' measured behaviour sits on `move_voltage(0)` under Coast.
2. **A stopped motor re-commanded every tick:** a live operation re-sends its voltage each tick (F2's rule,
   `mechanism.hpp` ~:53-68); a `brake()` issued every tick under Hold may reset the held position (HA new
   (c)) — prefer issuing the stop on the transition INTO zero, and say what the code does.
3. **Wrappers:** `MotorGroup`, the hostile fakes and the test wrappers forward `setVoltage` /
   `setBrakeMode`; the fix lives below them, in the adapter, so every wrapper inherits it — do not add
   stop logic above the adapter.
4. **Frozen surfaces:** F4 (`IMotor`) is not changed in signature; the adapter is an implementation, not a
   contract.
5. `-Wformat-nonliteral` / `-Wformat-truncation` are errors in `src/`; ASCII only in runtime strings.
