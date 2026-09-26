# DEFECTS2 — progress log (live)

- **Chunk:** DEFECTS2 — brake modes never reached the motor: make HOLD and BRAKE real at the V5
- **Brief:** `DEFECTS2-brake-modes-reach-the-motor.md` (committed by the coordinator at merge; this
  run read it from the coordinator's scratchpad, deliberately outside the repo)
- **Started:** Sat Sep 26 03:11:59 AM EDT 2026
- **Tree at start:** `b622d9b`, clean, in the git worktree `.claude/worktrees/defects2-lib` (branch
  `defects2-lib`) — NOT the main working tree, where another executor is finishing G1 Part 2b
- **Executor model:** opus
- **Go:** the coordinator's workflow run, 2026-09-26, on the team lead's request to run parallel
  agents that do not conflict. Do not commit; no robot upload; no `pros` commands; no `/dev/ttyACM*`.
- **This run is PHASE 1 — the library half.** IN: brief §1 items 1, 2, 4, 5, 6 and item 3's LIBRARY
  half (`teleop::DriverLoopConfig` gains the release brake mode, default Coast; `runDriverControl`
  applies it); §3's documentation for the files touched; HA numbers reserved for this run:
  **HA-142..HA-146 only** (a parallel chunk holds HA-147..HA-152). OUT (the coordinator, at merge):
  every file under `src/` (Teleop's "pass Coast" wiring lands after G1 Part 2b is committed; the
  X-drive's `runTeleopLoop` in `main.cpp`), the worksheet, `build-order.md`, `roadmap.md`,
  `HANDOFF-remaining-work.md`. `teleop_loop.hpp` is also edited by a parallel T3 builder — this
  run's edit there is small, additive and local.

Appended in real time. **Nothing in this chunk has run on a robot.**

---

## Log

### 03:12 — start
- `date` → Sat Sep 26 03:11:59 AM EDT 2026; `git status --short` empty; HEAD `b622d9b`.
- Read `docs/internal/RESUMING.md` (the chunk loop, the standards, the pace rules) and the brief in
  full. This log created before any code.

### 03:12–03:40 — required reading, and the defect re-verified (not taken from the brief)
- **The vendored PROS text, read myself:** `include/pros/motors.h:52` (`motor_move`) and `:253-254`
  (`motor_move_voltage`): *"This function will not respect brake modes, and simply sets the voltage
  to the desired value."* `:82-86` (`motor_brake`) and `include/pros/motors.hpp:260-284`
  (`Motor::brake()`): *"Stops the motor using the currently configured brake mode. This function sets
  motor velocity to zero, which will cause it to act according to the set brake mode."* The mode
  enum's own words (`motors.h:860-868`, `abstract_motor.hpp:37-42`): coast *"coasts when stopped"*,
  brake *"brakes when stopped"*, hold *"actively holds position when stopped"* — every one of them
  about a motor that is STOPPED, which a 0 mV voltage command is not.
- **shulib, read myself:** `hal/pros/motor.hpp:122-127` `setVoltage` only ever calls
  `move_voltage`; `:142-145` `setBrakeMode` only calls `set_brake_mode`. `grep -rn
  "\.brake()\|move_velocity\|motor_brake" include/shulib src test` (shim excluded) → every hit is the
  Chassis/Routine `brake()` VERB (the DriveBrake motion — `chassis_recipe_test.cpp`,
  `f6_signature_pin_test.cpp`, `path_runner.hpp:303`, …); no caller of `pros::Motor::brake()` or
  `move_velocity` anywhere. Every library stop is "mode, then 0 V":
  `hal/mechanism.hpp:240-245` (`applySafeState`), `motion/motion.hpp:166-171`
  (`applyCancelSafeState`), `motion/drive_brake.hpp:112-115` (every tick),
  `teleop/teleop_loop.hpp:136-138` (disable → `chassis.cancel()`). The defect is real as stated.
- **Item 2's pre-existing doc defect, confirmed:** `brakeMode()` DOES count a screened read
  (`motor.hpp:152`, `faultedReads_ += 1` — DEFECTS1 I9/E1, pinned by
  `pros_motor_adapter_test.cpp:175-190`), while the class doc (`:84-88`) and `faultedReads()`'s doc
  (`:211-215`) still say it does not. The code is right (a test pins it); the docs get fixed.
- Also read: `hal/motor.hpp` (the `BrakeMode` docs at :28-41 — "voltage 0 is COAST, NOT a hold";
  Hold "drives back to the position the shaft held when the command went to zero", which is more
  than PROS says), `hal/motor_conversion.hpp:62-70` (`lround` to the millivolt), `hal/motor_group.hpp`
  (forwards `setVoltage`/`setBrakeMode` to present members — so the fix below it reaches every
  group), `motion/motion_scheduler.hpp:655-667` (the destructor relies on `setBrakeMode`/`setVoltage(0)`
  never throwing — the fix must add no throwing path there), `chassis/chassis.hpp` (`drive()`
  pre-empts an active motion with `sched_.cancel()` → Brake; `deps().ctx->driveMotors()` is the
  loop's route to the drive motors), `sim/drive_plant.hpp` (0 V under Brake/Hold is pinned; the
  plant reads the mode only for coast-follow), the PROS shim (`test/pros_shim/pros/motors.hpp` — no
  `brake()`, `brake` a free-floating field), `test/pros_motor_adapter_test.cpp`,
  `test/teleop_loop_test.cpp`, `test/pros_device_provider_test.cpp` 1k (a `robot::Robot` over the
  REAL provider through the shim — the composition the stack tests below reuse), the HA register
  (HA-53, HA-92, HA-98, HA-137, HA-141 and the status line / index / reconciliation sections).

### 03:13–03:25 — baseline, clean tree `b622d9b`, fresh build dir `build/test`
- The first build needed the briefing regenerated (this log opens DEFECTS2 in the generated block):
  `python3 tools/briefing_status.py generate`. Then `cmake -S test -B build/test` + a full build →
  **`test cases: 1425 | 1425 passed | 0 failed | 3 skipped`, `assertions: 2386871 | 2386871 passed`.**
- `python3 tools/src_build_gate.py check` → **PASS**, all six builds (bench / tank / tank-drive /
  tank-library / tank-calibrate `2 allowed (-Wunused-function), 3 vendor-ignored, 0 disallowed`;
  xdrive `0 allowed`; `4/4 TUs compiled`, hash asserted, beacon checked).

### 03:40 — the design (before code)
**The adapter (`ProsMotor`), three new private records beside the existing ones:**
1. `commandedMode_` (`std::optional<BrakeMode>`) — the mode THIS PROGRAM last gave through
   `setBrakeMode`. Empty until the first call. The stop decision keys on THIS, never on `brakeMode_`
   (which the ctor seeds from the device for the T7 fallback — a drive motor an earlier program
   left in Hold would otherwise `brake()` at centred sticks in a program that never asked).
2. `atZero_` — the last `setVoltage` SENT 0 mV (the rounded `motorVoltageToMillivolts` value, not
   the double: a 0.0004 V command is a 0 mV wire command and must be treated as the stop).
3. `stoppedUnder_` (`std::optional<BrakeMode>`) — the wire's last act was `brake()` under this
   mode, and nothing has replaced it since.

- `setVoltage(v)`: mV ≠ 0 → `move_voltage(mV)`, clear `stoppedUnder_` (a non-zero command leaves
  the stop). mV = 0 → `stopAtZero()`: commanded Hold/Brake → `brake()` UNLESS already stopped under
  that same mode (landmine 2: issue the stop on the TRANSITION into zero, never every tick — a
  repeated `brake()` under Hold may re-capture the held position, HA-144); Coast or no commanded
  mode → `move_voltage(0)` on every call, EXACTLY as today.
- `setBrakeMode(m)`: `brakeMode_ = m` (fallback, as today), `commandedMode_ = m`,
  `set_brake_mode(m)` (as today); then, if `atZero_`, re-issue the stop under the new mode: Hold/Brake
  → `brake()` unless already stopped under m; Coast → `move_voltage(0)` ONLY if the motor is
  currently in a `brake()` stop (leaving it for the 0 V state Coast has always meant). So Coast →
  Coast at zero sends exactly today's wire traffic (`set_brake_mode` only) — the three programs
  that set Coast explicitly today (Calibrate, shulib Drive, the tester's DRIVE station:
  `drivetrain_calibrator.hpp:1456-1474`, `src/drive_program.cpp:166`, `src/bench_r3a.cpp:1407`)
  see no change.
- **Added while designing:** the `brake()` record is set only when PROS answers 1. `brake()`
  returns `PROS_ERR` on a port that does not answer (vendored `motors.hpp:271-273`); recording a stop
  that never happened would make the adapter skip the stop when the port comes back. So a failed
  `brake()` is retried by the next zero command.
- Order independence: mode-then-0 V and 0 V-then-mode both end in the same wire state.
- `commandedVoltage()` stays the applied 0 V. No throwing path added (the scheduler's destructor).

**The driver loop:** `DriverLoopConfig::releaseBrakeMode`, default `BrakeMode::Coast`. Applied to
every drive motor (`chassis.deps().ctx->driveMotors()`) at loop entry and again after EVERY
`chassis.drive()`. Why every tick rather than only when `drive()` pre-empted a motion: `drive()`'s
pre-emption (`sched_.cancel()` → Brake) is the one library path TODAY that can leave another mode
inside the loop, but a check keyed on that is a coupling to the facade's internals that a later
change would silently break; the unconditional re-assert is the invariant "every tick the loop
completes leaves the drive in the release mode", and it costs one `set_brake_mode` per motor per
tick (the adapter adds nothing else when the mode is unchanged) — `DriveBrake` already sends one
every tick. After `drive()`, not before, so a pre-emption inside this tick's `drive()` is undone in
the same tick (before, it would brake the drive for one 10 ms tick first). The disable path still
cancels into Brake — and with the adapter fixed, that stop now brakes.

### 03:45–04:20 — code and tests written
- **The shim** (`test/pros_shim/pros/motors.hpp`): `Motor::brake()` (PROS_ERR on a dead port,
  like every other command), a per-port `shim::MotorControl` (`Voltage` / `Stopped`),
  `stoppedUnder` (the mode AT the brake() call — so a test sees whether a stop was re-issued under
  a new mode) and `brakeCalls`. `move_voltage` now also sets `control = Voltage`.
  `lastVoltageMv` deliberately stays "the last move_voltage argument" (not zeroed by brake()): a
  test that asks only "is the wire at 0 mV?" is asking the question the defect hid behind. The
  header now states what it models and the five things it still cannot (HA-143..HA-146, plus a
  set_brake_mode sent while Stopped).
- **The adapter** (`include/shulib/hal/pros/motor.hpp`): as designed above — `setVoltage` keys on
  the rounded millivolts, `stopAtZero()` is the one place the stop is decided, `setBrakeMode`
  re-issues at zero. Item 2's docs: the class doc and `faultedReads()` now say FIVE screened reads
  are counted, brakeMode() included (the code, pinned since DEFECTS1, unchanged).
- **The loop** (`include/shulib/teleop/teleop_loop.hpp`): `DriverLoopConfig::releaseBrakeMode`
  (trailing field, default Coast — additive, designated initializers unaffected); a local
  `applyReleaseMode` lambda called once at entry and once right after `chassis.drive()` (step 6b).
  The edit is three hunks inside the function plus the field and its doc — nothing moved or
  reformatted (T3's parallel builder edits this file too).
- **Tests:** 10 adapter cases appended to `test/pros_motor_adapter_test.cpp` (DEFECTS2 a1–a10) and
  a new `test/brake_stop_stack_test.cpp` (s1–s7: MotorMechanism + RunUntilConfirmed + MotorGroup
  over real ProsMotors on ports 6/7; `chassis.cancel()` over a `robot::Robot` on the REAL provider
  through the shim, all ten ports; the loop's default; a pre-emption then released sticks → Coast
  in the same tick, then a disable → Brake; a disable while driving → Brake; the entry guarantee
  observed from the heading source before tick 0's drive(); a Hold release mode → one brake() per
  port across seven released ticks). Every assertion reads the shim's control state, never an
  adapter's `brakeMode()`.
- `python3 tools/api_doc_tool.py generate` (pros-motor.md, teleop_loop.md), then a full build:
  **`test cases: 1442 | 1442 passed | 0 failed | 3 skipped`, `assertions: 2386980 | 2386980
  passed`** (+17 cases = 10 + 7). Pre-existing suites untouched and green — including every
  teleop/mechanism/cancel test on the fakes and the plant.

### 04:20–04:35 — mutations, every one on a FRESH binary, every one observed RED
Runner: a scratchpad script (outside the repo) that, per mutation, replaces an exact-once string,
`touch test/*.cpp`, `make -C build/test shulib_tests/fast`, REQUIRES `Linking CXX executable
shulib_tests` in the output, runs the FULL suite, lists the failing cases, and restores the file
byte for byte. All fourteen relinked. The brief's five are M1, M2, M3, M4, M5; the rest were added
for the transition rule, the refused-brake retry, the Coast re-issue, the stop record, and the loop.

| # | Mutation | Suite | Red cases |
|---|---|---|---|
| M1 | revert to plain `move_voltage(0)` (`applyMode = false`) | 14 failed | a1 a2 a4 a5 a6 a7 a8 a9, s1 s2 s4 s5 s6 s7 |
| M2 | drop the Coast branch (Coast brakes) | 5 failed | a3 (the control) a5, s1 s4 s6 |
| M3 | drop the re-issue in `setBrakeMode` | 4 failed | a5 a7, s4 s6 |
| M4 | key on the raw double (`volts.value() == 0.0`) | 1 failed | a6 |
| M5 | key on the device-seeded `brakeMode_` | 1 failed | a7 (the seeded-mode trap) |
| M6 | `brake()` on every zero (no transition rule) | 3 failed | a8, s4 s7 |
| M7 | record the stop whatever `brake()` returned | 1 failed | a9 |
| M8 | Coast re-issue skipped when leaving a `brake()` stop | 3 failed | a5, s4 s6 |
| M9 | a driving command does not clear the stop record | 2 failed | a4 a6 |
| L1 | loop: drop the per-tick release re-assert | 1 failed | s4 |
| L2 | loop: drop the entry application | 1 failed | s6 |
| L3 | loop: ignore `cfg.releaseBrakeMode` (hard-coded Coast) | 1 failed | s7 |
| L4 | loop: default release mode Brake | 4 failed | s3 s4 s6, `G1 2a loop 4b` |
| L5 | loop: re-assert BEFORE `drive()` instead of after | 1 failed | s4 |

Restored, fresh relink, full suite: **1442 | 1442 passed | 0 failed | 3 skipped**. The two
narrowest kills (M4 → a6 only; M5 → a7 only) are the brief's two named traps; each has exactly the
one test built for it, which is the point of writing them. (L4 also reddens a PRE-EXISTING G1 2a
test: its "not braked while driving" check sees the Brake default — an independent witness.)

### 04:35–05:30 — documentation (§3, for the files this run touches)
- **`docs/hardware-assumptions.md`** — five NEW entries, only the reserved numbers: HA-142
  (`move_voltage`/`move` ignore the mode; only `brake()` applies it — reasoned, vendored text),
  HA-143 (what a motor physically does at `move_voltage(0)` — invented/UNKNOWN), HA-144 (a repeated
  `brake()` under Hold re-captures the held position — invented; the adapter issues one per
  transition, so the claim being true costs nothing, and the entry names the opposite risk: a hold
  that LAPSES without a refresh), HA-145 (Hold/Brake across a VEXos disable and re-enable —
  invented/UNKNOWN; the adapter's stop record cannot see a disable), HA-146 (leaving the stop: the
  next non-zero `move_voltage` replaces `brake()`; a per-tick `set_brake_mode` is harmless —
  reasoned). Index rows + a section after HA-141. **Amended:** HA-53 (the cancel coasted on a V5
  until DEFECTS2; its Source sentence "the A2 plant does not model brake modes" corrected — it reads
  the mode for coast-follow since 2F-1, it models no brake TORQUE), HA-92 (the declaration reached
  only the IMotor seam and the device's mode register, not the motor), HA-98 (the brake mode is
  device state too; why the stop keys on the commanded mode). Status line "7 of 141" → "7 of 146"
  with a DEFECTS2 sentence (the parallel chunk's HA-147..152 will move it again at merge).
- **A finding written into HA-142/HA-53/HA-145 while writing the settle steps:** a DISABLE is a
  confounded test of the fix. The loop's disable path issues its Brake stop only once the brain
  already reports disabled; whether VEXos is by then cutting motor output is HA-145's open question.
  The clean first look at HA-142 is an ENABLED stop (COMP's first `chassis.brake()`, or a
  driver-loop build with `releaseBrakeMode = Brake`) against a Coast release. The brief's worksheet
  line ("a disable during driver control" shows the fix) should be written with that caveat — for
  the coordinator's worksheet pass.
- **Every in-tree tag the new entries cite exists** (`grep -rln "A4 register HA-14[2-6]\b" include
  test`): HA-142 in `hal/mechanism.hpp`, `hal/motor.hpp`, `hal/pros/motor.hpp`,
  `manipulation/mechanism_op.hpp`, `sim/drive_plant.hpp`, `motion/motion.hpp`; HA-143 and HA-144 in
  `hal/motor.hpp` + `hal/pros/motor.hpp`; HA-145 in `hal/pros/motor.hpp`; HA-146 in
  `hal/pros/motor.hpp` + `teleop/teleop_loop.hpp`. (Three tags had first been line-split and did not
  grep; fixed.)
- **`include/` comments made true:** `hal/pros/motor.hpp` (header "THE STOP"; class doc; item 2 —
  five screened reads counted, brakeMode() included); `hal/motor.hpp` (the `BrakeMode` docs now
  quote PROS's own words per mode and mark the rest as belief — which position Hold holds, that
  Brake does not return; `setBrakeMode`'s doc no longer says "setVoltage(0) (= coast)"
  unconditionally); `manipulation/mechanism_op.hpp` ("Hold + 0 V IS the hold" — on a V5 only since
  DEFECTS2); `hal/mechanism.hpp` (T4 banner: the declaration did not reach a V5 before);
  `motion/motion.hpp` (`applyCancelSafeState`'s hardware note); `sim/drive_plant.hpp` (item 6:
  the pinned Brake/Hold wheel is what the adapter produces only since DEFECTS2 — comment only; no
  test needed more); the shim's header. **Wording discipline:** every "Hold went limp / the cancel
  coasted" is stated as reasoned from PROS's text, never as observed (two first drafts said it
  flatly; corrected).
- **`docs/changelog.md`** — "Fixed since 2.2 (2026-09-26)": what changes on a robot, what does not
  (Coast, and a motor never given a mode), the new `releaseBrakeMode`, the corrected docs, the
  shim note for contributors. **The `kApiMinor` decision, stated there: NO bump** — no signature
  changed; IMotor (F4) already promised Hold, F6's cancel safe state is unchanged in the contract,
  the adapter is outside every freeze, and `DriverLoopConfig` is unfrozen with a default that keeps
  the release behaviour — a fix that makes an implementation match its frozen contract is neither
  the policy's BREAKING nor its ADDITIVE.
- `docs/api/` regenerated (`motor.md`, `pros-motor.md`, `teleop_loop.md`, `mechanism.md`,
  `mechanism_op.md`, `motion.md`, README/all-entities: 2,592 → 2,593 entities, the new field).
- **A gate caught me:** the first full build after the register edit FAILED `check-removability` —
  HA-143 cited `docs/internal/chunks/R3d-part2f-…` from a public doc. Reworded to "the development
  log". (My earlier grep of that build's output filtered for "STALE|FAIL" and missed the word
  "VIOLATION"; since then every build is checked by its exit status.)

### 05:30–05:50 — prose review (every grep RUN and its hits READ before this entry)
Greps over `docs/`, `README.md`, `include/shulib`, `src/`, `test/`, `tools/`:
`move_voltage`; `0 V + Brake` / `0 V + Hold` / `Hold + 0 V` / `0 V under Brake` / `0 V and Brake` /
`zero volts under`; `BrakeMode::Brake` / `BrakeMode::Hold`; `brake mode`; `applySafeState` (public
docs); `coast` / `hold` / `brake mode` in the guide, cookbook, FAQ, READMEs, roadmap, master plan,
vexbuilder doc; `= coast|is coast|0 V is|setVoltage(0) alone|not model brake`; the shim's
descriptions in public docs.
- **Fixed (false, in files this run edits):** `hal/motor.hpp` ("voltage 0 is COAST, NOT a hold";
  "This is where setVoltage(0) alone leaves a motor"; Hold/Brake claims beyond PROS's text);
  `hal/pros/motor.hpp` (item 2's two comments; the BINDS list lacked brake()); HA-53's "the A2 plant
  does not model brake modes"; `teleop_loop.hpp`'s "The two numbers" (now three fields).
- **Now TRUE because of the fix, left as written:** every IMotor-level "0 V + Brake" description of
  the cancel safe state (`chassis.hpp:84,333,427`, `motion_scheduler.hpp:48,584,795`,
  `routine.hpp:291,514`, `path_runner.hpp:34,325`, `route.hpp:134`, `teleop_loop.hpp:19`, the test
  comments, `docs/vexbuilder-integration.md:287,303`, `docs/guide/09-the-recipe-api.md:105,127,152`,
  `docs/guide/10-the-api.md:232`, `docs/cookbook/02-when-a-step-fails.md:47`, `docs/faq.md:26`) —
  they describe the command sequence at the seam, which is unchanged and now applied at a V5.
- **Listed, not edited (outside this run's files, each true or now-true; for the coordinator):**
  - `docs/guide/14-what-it-cannot-do-yet.md:346` — "brake mode is commanded correctly, but what a
    real V5 brake does ... is HA-53". It was FALSE until DEFECTS2 (commanded, never applied) and is
    true now; worth a "(since DEFECTS2 — before it the mode never reached the motor)" at merge. NOT
    edited here: the main working tree has G1 Part 2b's uncommitted edits to this same file.
  - `include/shulib/motion/drive_brake.hpp:5-7` — "with BrakeMode::Brake so real hardware resists
    rather than coasts — the A2 plant does not model brake modes": the first half was false on a V5
    until DEFECTS2 and is true now; "does not model brake modes" is slightly stale since R3d Part
    2F-1 (it reads the mode for coast-follow; it models no brake torque). `:111` "every tick
    (idempotent)" stays true at the seam (at the device: one `brake()`, then `set_brake_mode` only).
  - `include/shulib/motion/move_to_pose.hpp:118` — "this one's stops just write 0 V": true at the
    seam; on a V5 that zero now BRAKES if an earlier library stop left Brake commanded (the sticky
    mode — changelog bullet added).
  - `src/main.cpp:449,1330` — "cancel() (0 V + Brake)"; `src/` is out of this run.
  - `docs/roadmap.md:442,1133`, `docs/internal/build-order.md:261`, `PROJECT-BRIEFING.md:296` — the
    seam-level description; out of this run (roadmap/build-order are the coordinator's).
  - History (`docs/internal/chunks/*-PROGRESS/COMPLETED`, R1a/F2/G1/G2 briefs) — records, not edited.
- **Nothing found that claims the shim models brake()/stops** in public docs (guide 07:129, 13:231,
  14:217, FAQ:203, README:272 describe the shim generically).

### 05:50–06:10 — final gates (all run in the worktree, on the final tree)
- Host suite, fresh relink (`touch test/*.cpp`, `/fast`, relink line seen): **1442 | 1442 passed |
  0 failed | 3 skipped; assertions 2386980 | 2386980 passed** (baseline 1425 / 2386871: +17 cases,
  +109 assertions — 10 adapter cases, 7 stack cases).
- Full `cmake --build build/test` (the doc gates run inside it): exit 0 — api_doc_tool self-test OK;
  coverage 2593 entities / 148 headers, all documented; check-fresh clean; examples 408 lines
  verbatim; removability clean; briefing current; doc staleness audit clean.
- `python3 tools/src_build_gate.py self-test` → OK (33 detector cases); `check` → **PASS**, six
  builds, the same warning profile as the baseline (bench/tank/tank-drive/tank-library/
  tank-calibrate `2 allowed (-Wunused-function), 3 vendor-ignored, 0 disallowed`; xdrive `0
  allowed`) — the adapter compiles and links against the REAL vendored `pros::Motor::brake()`.
- ARM header gate (all 184 `include/shulib` headers, `arm-none-eabi-g++ -std=gnu++20 -Wall -Wextra
  -Wconversion -Wsign-conversion -Wshadow -Werror -Os …`): exit 0. CI greps (no `<pros/` outside
  `hal/pros`; no `shulib/sim/` outside `sim`): both print nothing.
- Briefing regenerated LAST (`python3 tools/briefing_status.py generate`, then `check` → current).

### What this run did NOT do (by the split, or not reached)
- **OUT by the split (the coordinator, at merge):** shulib Teleop passing Coast (`src/`, after G1
  Part 2b commits — the library default is already Coast, so a Teleop that states nothing coasts);
  the X-drive's `runTeleopLoop` in `src/main.cpp` (brief item 3's "same guarantee or a stated
  reason"). **Read at `src/main.cpp:453-473` — it needs the guarantee, and not only for a
  pre-emption:** it sets no brake mode anywhere, so its released sticks coast only while no
  library stop has ever set Brake on those ProsMotors. The graph is built once and PROS calls
  `opcontrol()` again after a disable, so after the FIRST disable (its own `cancel()`), or after an
  autonomous that ended in `brake()`/`cancel()`, every later stick release in that loop would
  BRAKE — the sticky mode. In the committed tree `runTeleopLoop` is ALSO today's shulib Teleop's
  loop (`main.cpp:1328`, until 2b moves it onto `runDriverControl`, which carries the guarantee).
  Smallest fix at merge: `setBrakeMode(Coast)` on `chassis.deps().ctx->driveMotors()` at the loop's
  entry (no pre-emption can happen inside it — nothing in it starts a motion); or move it onto
  `runDriverControl` at 2c. The
  worksheet (with the disable-confound caveat above); `build-order.md` (entry, deviations row,
  `Next:`); `roadmap.md`; `HANDOFF-remaining-work.md`; the brief itself; `DEFECTS2-COMPLETED.md`.
- **Listed for the coordinator from the prose review:** `docs/guide/14-what-it-cannot-do-yet.md:346`
  (2b is editing that file in the main tree), `include/shulib/motion/drive_brake.hpp:5-7`.
- **Merge notes:** `docs/hardware-assumptions.md`'s status line ("7 of 146") and index will collide
  with the parallel chunk's HA-147..152 additions — both add rows after HA-141 and both edit the
  status sentence. `teleop_loop.hpp`: this run's hunks are the header's step 6b, one `#include`,
  the `DriverLoopConfig` doc + field, the `cfg` bullet, and three lines in the function body (the
  lambda + entry call, and the call after `drive()`). The briefing and `docs/api/` must be
  regenerated after the merge.
- **Nothing here has run on a robot.**

---

## Review findings and their disposition (04:15–04:45, the same run's builder, second pass)

A reviewer reported eight findings (one major, seven minor). I re-checked each one against the
tree before changing anything. For the two surviving-mutation claims, I first ran the reviewer's
mutations on the UNFIXED tree, each on a fresh binary with the relink line seen
(`d2_mutate_prefix.py`, a scratchpad copy of the runner):

| Mutation (unfixed tree) | Suite | Red |
|---|---|---|
| M10 `atZero_ = atZero_ \|\| millivolts == 0` (the reviewer's first form) | 1440/1442 | a4 a6 |
| M10b `atZero_` set only inside the zero branch (the reviewer's second form) | **1442/1442** | **none: survived** |
| L6 re-assert only when `drive()` pre-empted a motion | **1442/1442** | **none: survived** |

The reviewer's first form was already caught. A stuck-true record sends the NEXT DRIVING command
into the stop branch, and a4 and a6 notice that. The second form is the real hole, and so is L6.

**F1 (major): a zero record that sticks after the first stop. REAL (in its second form); FIXED.**
With M10b, every `setBrakeMode(Brake/Hold)` after one stop re-issues a stop over a RUNNING motor.
Under a Brake or Hold release mode, the loop's per-tick re-assert would stop the drive dead on every
driving tick. Code unchanged; the tests were missing:
- **a11** (new, three subcases): Hold, zero (stopped), 6 V, then `setBrakeMode` Hold / Brake / Coast.
  In each case the port stays under Voltage at 6000 mV, with `brakeCalls == 1` and no extra
  `move_voltage`.
- **s7 extended:** released ticks 3..7 are held, checked EVERY tick (one `brake()` per port). The
  sticks then push again on ticks 8..10, and every port must be under Voltage at ≠ 0 mV with still
  one `brake()`. The disable comes WHILE DRIVING and gives the second stop per port. Its comment
  now also says the re-assert's `set_brake_mode(hold)` reaches the held motor every tick (F5).
- Mutated again after the tests: M10b is red (a11, s7) and M10 is red (a4, a6, a11, s7).

**F2 (minor): the "after EVERY drive()" invariant was not pinned. REAL; FIXED (test).** Added
**s8**, with two subcases. `perTick` calls `chassis.cancel()` at tick 3 with no motion active (the
test checks `hasActiveMotion()` is false, and that the panic stop really braked all ten ports).
(a) The sticks were already released: from tick 4 on, every port is `zeroVoltsWithMode(coast)`.
(b) The robot is driving at the panic stop and the sticks are released at tick 5: the release
coasts. L6 is now red (s8), and L1 is red on s4 and s8.

**F3 (minor): a refused `set_brake_mode()` was believed. REAL; FIXED (code + shim + test).**
`brake()` stops under the device's REGISTER, not under `commandedMode_`. I confirmed the reviewer's
scenario against the code: the mode write is lost to a dropout; the port comes back; one zero goes
out as `brake()` under the register's old Coast; the stop is recorded as Hold; and every later zero
sends nothing. The fix lives in `ProsMotor`:
- A new `modeOnDevice_` records whether the port answered 1 to the last mode write.
- `stopAtZero()`'s Brake/Hold branch re-sends a refused mode write BEFORE `brake()`, and records the
  stop only when BOTH calls were accepted.
- `brake()` is sent even when the mode write was refused. The stop happens under whatever mode the
  register holds and is not believed. That beats leaving the motor on the last voltage it was
  driven at, and the next zero retries both calls.
- When the port accepts, the wire traffic is exactly what it was before this pass. a8 still counts
  ten `set_brake_mode`s, and a9b's control subcase counts one.

`disconnected` refuses every call, so the shim gained `MotorPortState::refuseSetBrakeMode` (refuse
the next N mode writes, nothing else). It models a dropout that ends between two calls, and ENODEV
is the only failure the vendored text documents for `set_brake_mode` (`motors.hpp` ~:1139).
**a9b** has three subcases:
- The dropout spans `setBrakeMode()`. One zero after the port comes back re-sends the mode and holds,
  and a repeated zero then sends nothing.
- The mode write is refused twice but `brake()` is accepted. The motor is stopped under Coast and NOT
  believed; the next zero stops it under Hold; the zero after that sends nothing.
- The control: an accepted write is never re-sent.

Four mutations, all red on a9b: M11 (the pre-review code, restored), M12 (believe the stop whatever
the mode write returned), M13 (drop the re-send), M14 (`setBrakeMode` ignores the write's result).
The limit that remains is named in THE STOP header and in the changelog: a caller that sends its
zero ONCE (`applySafeState()` at an operation's exit, T3's binding) gets no retry if the port is
down at that moment. Every refused command has had that limit, `move_voltage(0)` included.

**F4 (minor): HA-143's settle step compared against a DISABLE. REAL; FIXED.** HA-143 now compares
against an ENABLED Brake stop (COMP's `chassis.brake()`, or a `releaseBrakeMode = Brake` build), and
says why not a disable (HA-142 and HA-145). The loop header's step 1 gained two lines that point to
HA-145 ("whether that stop … lands in time and outlasts the cut is A4 register HA-145 (unknown)"),
and HA-145's Source names that tag. The reviewer suggested leaving this to the merge. I made it
now, as a pure addition below step 1. That is outside the "field + where it is applied" scope for
this file, so I checked it first: a trial 3-way merge (`git merge-file` over read-only copies of
base `b622d9b`, this file and t3-lib's file, in the scratchpad) merges step 1 cleanly. The only
conflicts are the three regions under F6, the same with or without this addition. The coordinator
can drop it if they want the stricter scope.

**F5 (minor): a per-tick same-mode `set_brake_mode()` to a motor STOPPED under Hold was
unregistered. REAL (a register gap, and a false line in the shim header); FIXED IN THE REGISTER,
NOT IN CODE.**
- **HA-144 widened** to two halves: a repeated `brake()` (unchanged), and a same-mode
  `set_brake_mode()` written to a motor already held. The claim, source, confidence, settle step
  (T3's claw station: one hold with a per-tick `set_brake_mode(hold)` test build, the same sag test)
  and blast radius were all updated, plus its index row.
- **HA-146 narrowed** to a RUNNING motor (title, claim and row), pointing at HA-144.
- **The shim header was wrong:** it said "the adapter does not rely on either answer (it re-issues
  brake())". That is true only for a CHANGED mode. For the same mode it relies on the write
  changing nothing, and the header now says so.

I considered the reviewer's alternative and rejected it: skip the wire write in `setBrakeMode()`
when the mode is unchanged and `stoppedUnder_` already equals it. Reasons, recorded in HA-144:
- It drops the per-tick re-write, which heals a register changed underneath (a second adapter on
  the port, a replugged motor, HA-98).
- It changes `DriveBrake`'s wire traffic, which is what it has been since C1.
- It guards nothing a program does today: the ruled release is Coast, T3's claws take their safe
  state once, and `DriveBrake` stops under Brake, which holds no position.

If T3's claw station shows that a same-mode write re-targets the hold, that suppression is the fix.
It belongs in `setBrakeMode()`, with its own mutation and a check against the T7 fallback.

**F6 (minor): `teleop_loop.hpp` conflicts with T3. REAL (textual and in numbering); A MERGE ITEM,
with a correction to my own report.**
- **My earlier report was wrong in one respect:** this run's edit to the file did not only add lines.
  It also REWROTE three things: the `DriverLoopConfig` doc lead ("The two numbers…/State BOTH" →
  "What the loop needs…/State the first TWO"), the function doc's order list ("…`drive(Body)`,
  `perTick`…" → "…`drive(Body)`, the release mode, `perTick`…"), and the `cfg` bullet. The merge
  notes above list the `cfg` bullet but miss the order list. The full list of hunks now: header step
  1 (+2 lines, F4), header step 6b (+7), one `#include`, the struct doc lead, the field + its doc,
  the order list, the `cfg` bullet, and in the body the lambda + entry call and the call after
  `drive()`.
- **The trial merge** (above) gives exactly three conflict regions:
  1. my `cfg` bullet against T3's new `bindings` bullet and its rewritten `disabled` bullet;
  2. my entry block (the lambda + `applyReleaseMode()`) against T3's "0. Every binding starts…"
     block, both after `hal::ButtonEdge rezeroEdge{}`;
  3. my "6b" body lines against T3's "6b. The mechanism bindings" loop.
- **Recipe:** keep both sides of each region, release mode first:
  - (1) my `cfg` bullet, then T3's `bindings` and `disabled` bullets.
  - (2) the release-mode lambda and entry call, then T3's binding reset/makeSafe loop. The two are
    independent.
  - (3) `// 6b.` release re-assert right after `drive()` (it must undo that `drive()`'s pre-emption
    in the same tick), then T3's loop renumbered `// 6c. The mechanism bindings…`.
  - Then, by hand, where git sees no conflict: in T3's header paragraph, change "between steps 6 and
    7" to "between steps 6b and 7 (as step 6c)". Add the release mode to its "three things are
    added" list, or say it is unaffected by bindings. Add "the bindings" after "the release mode" in
    the function doc's order list.
  - The re-assert stays in T3's single loop body. T3's no-span overload forwards to it, so there is
    one body and s4–s8 cover both entry points.

**F7 (minor): `setVoltage`'s doc said `commandedVoltage()` is "0 V for a stop". REAL; FIXED.** It
now says the value is the clamped volts, UNROUNDED, so a sub-millivolt command reads back as itself
(0.0004 V, which a6 pins) although the wire sent a stop.

**F8 (minor): the per-confidence counts; HA-98's row wording. REAL; FIXED (note), RECOUNT DEFERRED.**
The Counts sentence sums to 122, a drift the status paragraph already admits for HA-124..141. The
DEFECTS2 sentence of that paragraph (lines this run already owns) now says the counts predate
HA-142..146 too, and that a recount is due once HA-147 onward is in. A recount now would collide
with the parallel chunk's edit of the same paragraph and would be wrong again at merge. The HA-98
row now reads "— and the BRAKE MODE (added to this row by DEFECTS2) —".

**Also corrected while here:**
- HA-142's Source cited "a1–a10 / s1–s7"; it now cites a1–a11 (a9b included) / s1–s8, and names
  a9b's reason (brake() stops under the register).
- HA-144's s7 count went from "seven" to "five released driver-loop ticks".
- The changelog gained a "stop the port refused is retried, never believed" bullet and the shim's
  new knob.
- Prose greps for "0 V for a stop", "does not rely on either answer", "a1–a10", "s1–s7", "seven
  released" and "since DEFECTS2, the BRAKE MODE" now hit nothing outside this log.

### The full mutation set, re-run on the final code (`d2_mutate_r1.py`; every one relinked, every one RED)
The runner's M5 and M7 patterns were rewritten for the new `stopAtZero()` body, with the same
meaning.

| # | Mutation | Suite | Red cases |
|---|---|---|---|
| M1 | revert to plain `move_voltage(0)` | 17 failed | a1 a2 a4 a5 a6 a7 a8 a9 a9b a11, s1 s2 s4 s5 s6 s7 s8 |
| M2 | drop the Coast branch | 6 failed | a3 a5, s1 s4 s6 s8 |
| M3 | drop the re-issue in `setBrakeMode` | 5 failed | a5 a7, s4 s6 s8 |
| M4 | key on the raw double | 1 failed | a6 |
| M5 | key on the device-seeded `brakeMode_` | 1 failed | a7 |
| M6 | `brake()` on every zero | 5 failed | a8 a9b, s4 s7 s8 |
| M7 | record the stop whatever `brake()` returned | 1 failed | a9 |
| M8 | Coast re-issue skipped when leaving a stop | 4 failed | a5, s4 s6 s8 |
| M9 | a driving command does not clear the stop record | 2 failed | a4 a6 |
| M10 | `atZero_` OR-accumulated | 4 failed | a4 a6 a11, s7 |
| M10b | `atZero_` set only in the zero branch | 2 failed | a11, s7 |
| M11 | the pre-review stop (no re-send; believed regardless) | 1 failed | a9b |
| M12 | believe the stop whatever the mode write returned | 1 failed | a9b |
| M13 | drop the mode re-send | 1 failed | a9b |
| M14 | `setBrakeMode` ignores the write's result | 1 failed | a9b |
| L1 | drop the per-tick release re-assert | 2 failed | s4 s8 |
| L2 | drop the entry application | 1 failed | s6 |
| L3 | ignore `cfg.releaseBrakeMode` | 1 failed | s7 |
| L4 | default release mode Brake | 5 failed | s3 s4 s6 s8, `G1 2a loop 4b` |
| L5 | re-assert BEFORE `drive()` | 1 failed | s4 |
| L6 | re-assert only after a pre-emption | 1 failed | s8 |

Restored byte for byte (`git status` shows only this run's files). After a fresh relink the suite is
**1445 | 1445 passed | 0 failed | 3 skipped; assertions 2387052**. That is +3 cases (a9b, a11, s8)
and +72 assertions over the first pass.

### Final gates after the review pass (worktree, final tree)
- `python3 tools/api_doc_tool.py generate` (the new setVoltage/setBrakeMode docs reach
  `docs/api/pros-motor.md`). Then `python3 tools/briefing_status.py generate` (suite 1,442 → 1,445).
- Full `cmake --build build/test` (the doc gates run inside it): **exit 0**.
  - api_doc_tool self-test OK.
  - Coverage: 2593 entities / 148 headers, all documented.
  - Examples: 408 lines, verbatim.
  - Removability clean; briefing current; doc staleness audit clean.
  - The build before the briefing regeneration FAILED, as it should have: the briefing still showed
    the 1,442 count.
- Fresh relink (`touch test/*.cpp`, `/fast`, `Linking CXX executable shulib_tests` seen): **1445 |
  1445 passed | 0 failed | 3 skipped; assertions 2387052 | 2387052 passed**.
- `check-fresh` exit 0. `check-removability` exit 0. `briefing_status.py check` shows the block is
  current. `doc_staleness_audit.py` is clean.
- `python3 tools/src_build_gate.py self-test`: OK (33 detector cases). `check`: **PASS**, six
  builds with the same warning profile as the baseline (bench / tank / tank-drive / tank-library /
  tank-calibrate: 2 allowed `-Wunused-function`, 3 vendor-ignored, 0 disallowed; xdrive: 0 allowed).
  All six report 4/4 TUs, hash asserted, beacon checked. The new `set_brake_mode` re-send compiles
  against the REAL vendored `pros::Motor`.
- ARM header gate (all 184 `include/shulib` headers; `arm-none-eabi-g++ -std=gnu++20 -Wall -Wextra
  -Wconversion -Wsign-conversion -Wshadow -Werror -Os …`): exit 0. Both CI greps print nothing.
- Still not committed, and nothing touched outside this worktree. No `pros` command was run and
  `/dev/ttyACM*` was not touched. **Nothing here has run on a robot.**
