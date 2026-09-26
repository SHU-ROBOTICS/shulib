# T3 Part 2 — the claws wired into shulib Teleop

> **Execution brief (T3's final part).** Written 2026-09-26 ~05:20; re-checked against the tree at
> `c845041` (T3 Part 1 landed) and amended with Part 1's merge items at ~13:25. **Runs in an isolated
> worktree, `.claude/worktrees/t3-part2` (branch `t3-part2`, from `c845041`)**, in parallel with the
> coordinator merging DIAG1 and COMP-a on the main tree and with G1 Part 2c's final run in another
> worktree (the team lead's rulings of ~13:15: ONE `#if` chain in `src/` picks the program AND the robot,
> the two tables pick by that constant — so **do not restructure how `src/mechanism_table.hpp` or
> `src/chassis_table.hpp` choose the robot, and do not touch `src/main.cpp`**; 2c's run owns those). The
> coordinator has committed T3 Part 1 (the library half from `.claude/worktrees/t3-lib` — hold and toggle
> bindings, the `runDriverControl` bindings overload, `MechanismConfig` motor + pneumatic mechanisms,
> `robot::Robot` building them, the provider's digital out — and the program-side prep from
> `.claude/worktrees/t3-prog-prep` — `src/mechanism_table.hpp` with the pivots and the pneumatic claw,
> Calibrate skipping declared claw ports, worksheet Station J, HA-153..159). Read FIRST: the T3 brief
> (`docs/internal/chunks/T3-claw-pivots-bindings-and-mechanisms.md`), its addendum
> (`T3-addendum-pneumatic-claw.md`), and `T3-PROGRESS.md` (every earlier run's log). DEFECTS2 has landed
> (`a8540e6`): a Hold this program commands now stops the motor through PROS's `brake()`. **Executor model:
> `opus`. Do not commit.** Log under a heading "Part 2: Teleop wiring" in `T3-PROGRESS.md`.

## 1. Scope — IN
1. **The boot source puts the table's mechanisms into the `RobotConfig`** (`src/boot_source.hpp`'s pure
   core converts `src/mechanism_table.hpp`'s entries into `MechanismConfig` — the pivots as single-motor
   mechanisms, the claw as a pneumatic mechanism; every field converted, nothing defaulted), so
   `robot::Robot` builds them. The motors come from `kMechanisms.motors[0 .. motorCount)` (a
   `bench::MotorMechanismEntry`: `*entry.safeMode`, `*entry.forward`, `*entry.reverse`, `volts`), the
   claw from `kMechanisms.pneumatics[0 .. pneumaticCount)` (`pneumaticPorts[0] = adiLine`,
   `pneumaticCount = 1`; the start and safe values are LEVELS — `bench::pneumaticStartCommand(e)` /
   `pneumaticSafeCommand(e)` — never the positions; the toggle on `*e.toggle`). **The 3-wire output
   is driven the moment it is constructed (HA-119): it must be constructed at the START level, or the
   claw twitches at boot** — a test pins the first level the shim's line ever sees (T3-PROGRESS E2-2).
   The library builds the line at the config's `pneumaticStart` and the toggle binding sends ITS
   OWN initial level on the first driver-control entry (B2's decision 3): both come from the ONE
   table value (`pneumaticStartCommand(e)`), and a test pins them equal — two different levels would
   move the claw when driver control starts. The boot report and the panel read the claw through
   `robot.builtMechanismAt(i)` / `robot.pneumaticMechanism(name)` (the coordinator ratified B2's
   typed lookups at T3 Part 1's merge; `mechanismAt(i)` / `mechanism(name)` are the MOTOR lookups),
   and `mechanismAbsence(i)` gives an absent one's sentence. A declared mechanism port that the loaded calibration record lists as a
   DRIVE port ⇒ Teleop REFUSES by name ("port N is both a claw and a drive motor in the record --
   recalibrate"). The boot check's "NEW motors not in the calibration" line names declared claw ports as
   claw pivots, not unknown motors.
2. **shulib Teleop's bindings:** two hold bindings (port 6: R1/R2; port 7: L1/L2; 12 V) and one toggle
   binding (the claw: A; starts CLOSED), built and VALIDATED inside `teleopInitialize`'s `try` from the
   table, emplaced in the static `TeleopState` (never on `runTeleop`'s stack); `runTeleop` uses the
   bindings overload of `runDriverControl`. **Nothing that can throw runs after driver control starts.**
   A mechanism the provider refused ⇒ that binding is inert and the pivot/claw is ABSENT, said on a boot
   line (with the provider's own sentence), on the panel and on the controller if a line has room; the
   drive still runs.
3. **The re-zero button moves A → Y** (`kRezeroButton`), with `static_assert(kRezeroButton ==
   bench::kTeleopRezeroButton)` tying it to the table's re-zero constant, the existing "never
   L1/L2/R1/R2" assert extended to "never a binding button", and its comment ("No ruling picks it:
   A ...") replaced by the team lead's ruling of 2026-09-26. In the same change, rewrite the
   `kTeleopRezeroButton` doc comment in `src/mechanism_table.hpp` (~:140-148, "NOT YET TIED TO TELEOP
   ... Until then the two DISAGREE") to say the two are tied and where the assert sits (E2-1).
4. **The claim ruling (the team lead, 2026-09-26):** `runDriverControl` RELEASES any leftover claim on each
   bound mechanism at loop entry (a claim left by a deleted autonomous task must never lock the driver out),
   and says so — one log line per released claim, and a word on the panel. Library change in
   `include/shulib/teleop/` with its tests (a mechanism claimed before the loop ⇒ released at entry, then
   driven by its binding; a mechanism claimed DURING the loop by something else ⇒ still never commanded).
5. **The panel:** merge the two fault rows into one (ruling R3) and give the freed row to the claws:
   `CLAW 6 FWD|REV|HOLD|ABSENT <temp>C | 7 ... | CLAW OPEN|CLOSED|ABSENT` (or the closest that fits the
   row's budget from its x — every row within budget, compile-time checked for literals, redraw only on
   change, only a literal `"%s"` to the screen, ASCII). Pivot temperatures from the motors.
6. **Tests:** the T3 brief's test 1 — wiring through `ProsDeviceProvider` + the PROS shim with the REAL
   `src/` table: R1 alone ⇒ port 6 +12000 mV and port 7 untouched, R2 ⇒ port 6 −12000, L1 ⇒ port 7 +12000,
   L2 ⇒ port 7 −12000, the drive ports receiving exactly what they receive with no button; release ⇒ each
   pivot `brake()` under Hold once (DEFECTS2's shim); A ⇒ the claw's line flips once per press; the
   FIRST level port A's shim line ever sees is the start level (no boot twitch), and a disable or a
   controller drop leaves the line where it is; the boot
   conversion of the table into `MechanismConfig` exactly; the drive-port collision refusal; the re-zero Y.
   Mutations RED on a fresh binary: swap R1/R2 in the wiring; drop the claim release; convert a pivot with
   Coast; start the claw open; hand the library the claw's POSITION instead of its level (or the safe
   level as the start); construct the 3-wire output at the opposite level before setting the start.
7. **Docs:** fill Station J's every `<<FROM TELEOP AT MERGE>>` from the code (quotes EXACT — J.3.1,
   J.3.2, J.3.3, J.4.6, J.7.1 (strike "and the serial line" if Teleop prints none on a controller
   drop), J.9.1, J.9.3); worksheet Station F's re-zero button (A → Y) and anything quoting the panel's
   fault rows; every statement that the re-zero is A (`docs/roadmap.md` ~:1690, `build-order.md`
   ~:1409, HANDOFF ~:396, `include/shulib/teleop/teleop_loop.hpp` `DriverLoopConfig`'s comment ~:90
   and the generated `docs/api/teleop_loop.md` — regenerate, never hand-edit; `grep -rn "re-zero"`
   and `"rezero"` to find any other); `DEFECTS2-COMPLETED.md`'s last "NOT done" bullet (the L1
   default) — say what guards it now; HANDOFF :111 (Calibrate
   takes every motor EXCEPT declared mechanism ports), L5, L9, Q1 (answered in part, the pneumatic
   claw included), §7; build-order T3
   status + the first `Next:` (G1 Part 2c, then COMP); roadmap Phase T rows + "Now"; guide 14 (driver
   control: the claws); the changelog if the library changed (item 4); the register's status line counts
   after HA-159; `docs/api` and the briefing regenerated LAST.

## 2. OUT
Chassis macros; angle presets; a stall cut-off; claws in shulib Drive (slot 1); any hardware claim —
**nothing here has run on a robot** until Station J's log says so.

## 3. Checks
Host suite green (counts before/after); every mutation RED on a fresh binary; `src_build_gate.py`
self-test + check (six builds, 0 disallowed, beacons); ARM header gate; CI greps; `check-fresh`;
`check-removability`; `doc_staleness_audit.py`; the briefing regenerated LAST and checked.
