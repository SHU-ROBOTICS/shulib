# T3 — the claw pivots: button bindings and mechanisms built from the robot's config (robot two, ports 6 and 7)

> **Execution brief.** Written 2026-09-26 (early morning) at tree `b622d9b` while G1 Part 2b was being built,
> from three read-only research reports (the library seams; the program / boot / Calibrate side; the motor
> ground truth) and a critic, with the load-bearing facts re-verified by the coordinator. **The team lead's
> order (2026-09-26): "the 6 and 7 thingies actually needs to be done and put in ... the driver program and
> it needs to be done asap with actual library code"; "we need to grind and get the claws working ASAP".**
> Order: **G1 Part 2b → DEFECTS2 (brake modes reach the motor) → T3 → G1 Part 2c → COMP.** T3 starts only
> after DEFECTS2 is committed (the pivots' HOLD depends on it), on the team lead's go. **One executor run.
> Executor model: `opus`. Do not commit.** Live log: `T3-PROGRESS.md`, created FIRST.

---

## 0. The request, and the team lead's rulings

Robot two has two **separate** claw PIVOT motors (the team lead, 2026-09-26): **port 6** — forward while **R1**
is held, backward while **R2** is held; **port 7** — forward while **L1** is held, backward while **L2** is held.
**RED** cartridge each, **full power (12 V)** while held, and on release each pivot **HOLDS its angle**. Driver
control only (the auton does not need them now; routines may command the same mechanisms later).

**Rulings (2026-09-26):**
- **R1 — The stick rule, amended.** build-order's T3 entry calls "any binding is interruptible by stick input"
  NON-NEGOTIABLE (~:1439-1441; handoff L5 repeats it). For a pivot the driver uses WHILE driving, that rule would
  make the claw unusable. **It applies to chassis macros only.** For a mechanism button binding, **releasing the
  button is the stop**: the mechanism reaches its declared safe state in the same tick, asserted at the motors.
  Recorded as a deviation (a deviations-table row), never reinterpreted silently.
- **R2 — Both buttons of one pivot held ⇒ the pivot HOLDS** (neither wins); releasing one lets the other drive
  (level semantics).
- **R3 — Teleop's panel is full** (12 rows; a 13th fails its `static_assert`): **merge the two fault rows** and
  give the freed row to the claws.
- **R4 — One T3 chunk:** the claws in shulib Teleop AND `robot::Robot` building mechanisms from
  `config::MechanismConfig` (handoff L9's piece), so COMP inherits the claws through the same config. Chassis
  macros (the stick-interrupt case) are later.
- **Stated to the team lead without objection:** the claws live in shulib Teleop (and COMP later), NOT shulib
  Drive (slot 1); names "claw pivot 6" and "claw pivot 7"; Calibrate never touches a declared claw port; each
  pivot's temperature is shown; a missing claw motor at boot ⇒ **Teleop drives on**, that pivot ABSENT and said so.

## 1. What exists (read these; committed APIs — 2b changed only comments under `include/`)

| Piece | Where | What matters |
|---|---|---|
| Mechanism | `hal/mechanism.hpp` | `MotorMechanism(span<IMotor* const>, BrakeMode safe, name)` BORROWS span and name; `setVoltage` fans one voltage to all motors (so **two pivots = two single-motor mechanisms**); `applySafeState()` = mode then 0 V; the claim token; F1's "applySafeState() without cancel() is undone one tick later" (~:53-68); open-ended "while held" behaviour is NOT an `IMechanismOp` — command directly, own the stop |
| Controller | `hal/controller.hpp` | button LEVELS (`pressed`), `isConnected()`, per-consumer `ButtonEdge` |
| Driver loop | `teleop/teleop_loop.hpp` | `runDriverControl(...)`: disable ⇒ `chassis.cancel()` + return (PROS deletes the task on a field disable — no destructors); axes/buttons read only while connected; `perTick` after `drive()`; `DriverLoopConfig::rezeroButton` DEFAULTS TO **L1** (the port-7 forward button) — Teleop uses A (check at 2b's commit) |
| Config | `config/robot_config.hpp` ~:331-352 | `MechanismConfig{name[24], motorPorts[4], motorCount, pneumaticPorts[2], pneumaticCount}` — no cartridge, no safe mode; Robot builds nothing from it; `validate()` requests every motor with the DRIVETRAIN's cartridge |
| Provider | `hal/pros/device_provider.hpp` | `motor(signedPort, cartridge)` registry-checked; the same port again with a different sign/cartridge is a PRECONDITION (throws on the robot) |
| Brake modes | DEFECTS2 (landed before T3) | after it, a program-commanded Hold at 0 V calls PROS `brake()` — the hold is real at the motor; its HA entries name what is still unmeasured |
| Calibrate | `src/calibrate_program.cpp` ~:1114-1129, :1174-1195, :626-633 | takes EVERY registry motor as a drive candidate: BLUE gearing written, set Coast, possibly pulsed — a claw on 6/7 would be touched |
| Fakes / shim | `hal/fake/fake_controller.hpp` (buttons read false while disconnected — hides a missing connected check), `test/mechanism_test_rig.hpp` (`RecordingMotor`: ordered Voltage/Brake events), `test/pros_shim/pros/motors.hpp` (per-port signed mV, call counters; `brake()` after DEFECTS2) | an end-to-end wiring test through `ProsDeviceProvider` + the shim with the REAL `src/` claw table is possible |

## 2. Scope — IN

**Library (PROS-free, host-tested):**
1. **`include/shulib/teleop/mechanism_binding.hpp`** — a non-copyable, non-movable hold-to-run binding over one
   `hal::MotorMechanism&`: a forward button, a reverse button, a STATED voltage (0 < V ≤ 12, finite, no default).
   A pure state machine over two button LEVELS:
   - forward only ⇒ `setVoltage(+V)` every tick; reverse only ⇒ `setVoltage(-V)` every tick;
   - neither, both (R2), or disconnected ⇒ `applySafeState()` **exactly once on the transition**, then NO command
     until a button drives again (re-safing every tick would starve a future operation and may re-capture the hold
     point — DEFECTS2's HA);
   - an outside stop (`makeSafe()`, the loop's disable path) ⇒ INERT until that button is released, then re-arms
     (an outside stop is never undone by the next tick); disconnect = release, and a still-held button drives again
     after reconnect;
   - `reset()` + `makeSafe()` at loop entry; a button already held at the first tick drives at once;
   - it takes **no claim** and **never commands a mechanism someone else has `claimed()`** (a claim registered by a
     binding would survive a PROS task deletion and break a later operation or RunGuard's cancel-all).
   Every rule stated in the header and pinned by a test.
2. **`runDriverControl` overload** taking `std::span<teleop::MechanismBinding* const>` (the old signature forwards
   with an empty span — one body; existing tests keep pinning the no-binding path): entry ⇒ `makeSafe()` every
   binding before tick 1; read each binding's buttons only while connected; tick the bindings after
   `chassis.drive()`, before `perTick`; on `disabled()` ⇒ `chassis.cancel()`, then `makeSafe()` every binding,
   then return. A construction-time check that no binding button equals the loop's re-zero button.
3. **Mechanisms from config (handoff L9's piece):** `MechanismConfig` gains a **cartridge** and a **safe brake
   mode**, each with an UNSET representation (NOT a value-initialised `BrakeMode` — its first enumerator is Coast,
   an invented default that drops a loaded pivot); `valid()` refuses an unset value when `motorCount > 0`, placed
   AFTER the port-claim loop (existing collision tests keep their messages); `robot::Robot` builds one
   `MotorMechanism` per configured mechanism (requesting its motors with the MECHANISM's cartridge), names it,
   exposes `mechanism(name)` and a `std::span<hal::IMechanism* const>` for RunGuard; a mechanism motor the
   provider refuses ⇒ that mechanism is ABSENT (an absent-mechanism path, never a whole-build refusal — the
   dead-port rule's spirit), reported by name; a mechanism port that is also a drive port ⇒ `valid()` refuses.
   Robot's bit-identity pin for configs with no mechanisms stays byte-for-byte.

**Program (`src/`):**
4. **`src/mechanism_table.hpp`** — a PROS-free sibling of `chassis_table.hpp` (which stays "the ONE typed input
   about a drivetrain"), selected by the same `SHULIB_ROBOT_TANK_2026` `#if`; other variants declare none. Per
   pivot: signed port (+6, +7), RED, safe mode Hold, forward/reverse buttons, 12 V, the name, provenance "team lead
   2026-09-26". A value is never invented.
5. **shulib Teleop:** the boot source puts the table's mechanisms into the `RobotConfig`; `Robot` builds them;
   the bindings are constructed and VALIDATED inside `teleopInitialize`'s existing `try` (unique buttons,
   forward ≠ reverse, none equal to the re-zero button, the voltage range) and emplaced in the static
   `TeleopState` — **nothing that can throw runs after driver control starts** (`opcontrol`/`runTeleop` have no
   `try`); a claw port that the calibration record lists as a DRIVE port ⇒ refuse Teleop by name ("recalibrate").
   Boot lines per pivot (built / ABSENT with the provider's own sentence); the claw row on the panel ("CLAW 6
   FWD|REV|HOLD|ABSENT <temp>C | 7 ...") at the panel's existing cadence, after merging the two fault rows (R3);
   Teleop's boot banner says the claws are in Teleop, not shulib Drive.
6. **shulib Calibrate:** read the mechanism table and drop declared claw ports from the candidate list BEFORE any
   `ProsMotor` is built — log `port 6: claw pivot (declared) -- not a candidate, not touched`; no gearing written,
   no Coast, no pulse. (Must land before the owed Calibrate reproduction run.) The boot check's "NEW motors not in
   the calibration" line names declared claw ports as claw pivots, not as unknown motors.

**Tests (adversarial; each with its mutations RED on a fresh binary):**
7. The research's list, all of it: (1) wiring through `ProsDeviceProvider` + the shim with the REAL `src/` table —
   R1 alone ⇒ port 6 +12000 mV and port 7 untouched, R2 ⇒ port 6 −12000, L1 ⇒ port 7 +12000, L2 ⇒ port 7 −12000,
   the drive ports receiving exactly what they receive with no button (kills swapped buttons, swapped pivots, a
   wrong sign, a wrong voltage); (2) release at a `RecordingMotor` ⇒ exactly [Brake Hold, Voltage 0] once, then
   no event over 50 released ticks; (3) both held ⇒ safe once, nothing energised, releasing one drives the other;
   (4) R1 + L2 together ⇒ +12 V on 6 and −12 V on 7 in the same tick; (5) a press read at tick k is commanded in
   tick k; (6) disconnect with a LYING controller double (buttons true while `isConnected()` false) ⇒ safe, no
   command while disconnected, resume on reconnect; (7) disable while held ⇒ both pivots safe and
   `chassis.cancel()` run inside the disable tick, zero commands after; (8) entry safe before tick 1; (9) an
   outside `makeSafe` while held ⇒ inert until release, re-arms on release-then-press; (10) a mechanism claimed by
   someone else is never commanded; (11) construction refusals (duplicate button, forward == reverse, a binding
   button == re-zero, V ≤ 0, V > 12, NaN) — at CONSTRUCTION, not in the loop; (12) the drive is unaffected: the
   same stick script with and without bindings gives identical drive volts every tick; (13) Calibrate's candidate
   filter as a pure function; (14) config: unset cartridge/safe mode refused when motorCount > 0, a mechanism port
   colliding with a drive port refused, Robot builds and names the mechanisms, an absent mechanism motor leaves the
   drive built and the mechanism ABSENT, the no-mechanism bit-identity pin unchanged.

## 3. OUT
Chassis macros and the stick-interrupt rule's implementation (later). Angle presets (`rotateClampToAngle`, F3).
A stall cut-off (12 V passes straight through, as ordered; temperature is shown; a MEASURED cut-off can come
later). Claws in shulib Drive. Any hardware claim — **nothing here has run on a robot**.

## 4. Documentation, in the same commit
`docs/hardware-assumptions.md` — new: robot two's claw pivots are RED on ports 6 and 7 (reported by the team
lead; nothing has seen them yet); Hold holds a LOADED pivot (to measure at Station J, building on DEFECTS2's
entries); a pivot stalled at 12 V against its stop heats (the 2.5 A default ceiling; timing invented); a cable
swapped between a claw and a drive motor is invisible to the registry (the coupled-side monitor catches the drive
half). `R3a-BENCH-WORKSHEET.md` **Station J** (J is free): Station F passes first; the boot lines to expect; for
each pivot which physical way R1/L1 moves it (a flip goes only in the table's sign); the load test — release,
measure sag over 30 s, read the temperature; both buttons; a controller unplug; the claws are in slot 2 (Teleop),
not slot 1 (Drive). `build-order.md` — the T3 entry (R1's amended rule, R4's scope), deviations rows (the order
change and R1), the first `Next:`; `roadmap.md` Phase T rows and "You are here"; `HANDOFF-remaining-work.md` L5,
L9, Q1 (answered in part: two RED pivots on 6 and 7) and §7; `docs/guide/10-the-api.md` and `14`; a cookbook
recipe whose example is COMPILED in `test/cookbook_examples_test.cpp` and quoted verbatim; `docs/changelog.md`
("Added since 2.2": the binding, the loop overload, `MechanismConfig`'s fields, `Robot`'s mechanisms — no version
bump; F11/F12/F13 stay unfrozen); `docs/vexbuilder-integration.md` (the mechanisms schema); a note in the COMP
brief that its practice-match station becomes K (H/I/J are taken) and its RunGuard mechanisms span is now
non-empty; `docs/api/` regenerated; the briefing LAST. **Prose review** (grep before claiming): `MechanismConfig`,
`mechanism(`, `binding`, `interruptible`, `R1`, `L1`, "no mechanism", "builds no mechanism", "claw".

## 5. Checks and DoD
- [ ] Suite green (counts before/after); every test's mutations RED on a fresh binary.
- [ ] `tools/src_build_gate.py` self-test + check — six builds, 0 disallowed warnings, beacons (the new `src/`
      header compiles in every build; no `SHULIB_RUNS_*` outside `main.cpp`).
- [ ] ARM header gate, CI greps, `check-fresh`, `check-removability`, `doc_staleness_audit.py`, briefing.
- [ ] §4's documentation; `T3-PROGRESS.md`; `T3-COMPLETED.md` by the coordinator after the commit.

## 6. Landmines
1. **Two pivots = two single-motor `MotorMechanism`s** — one two-motor mechanism would move both claws on R1.
2. **Nothing that throws after driver control starts** — validate and request everything in `teleopInitialize`.
3. **PROS deletes the opcontrol task on a disable** — bindings, mechanisms and their `IMotor*` arrays live in the
   static `TeleopState`, never on `runTeleop`'s stack; no claim (§2 item 1).
4. **Safe ONCE on release** — a per-tick re-safe starves operations and may creep the hold point.
5. **The re-zero default is L1** — the construction check catches a caller that keeps it.
6. **Calibrate must not touch a claw** — the filter runs before any `ProsMotor` exists.
7. **An unset safe mode must be unrepresentable as Coast** in `MechanismConfig`.
8. **The panel's row budget** — merge the fault rows (R3); keep the pitch; every row within its budget.
9. `-Wformat-nonliteral` / `-Wformat-truncation` errors in `src/`; ASCII only; only a literal `"%s"` to the screen.
