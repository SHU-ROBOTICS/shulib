# COMP — the competition program: boot the robot, pick an auton on the brain screen, run it under a guaranteed end, drive

> **Execution brief.** Written 2026-09-24 at tree `2780462` while the team lead was away from the
> robot, so it is ready for his go the moment G1 Part 2b lands. It is the handoff's item **L2**
> (`HANDOFF-remaining-work.md` §5), which the plan had no entry for; this brief's commit adds the
> `build-order.md` entry and the deviations row. **Two parts, each its own executor run, verification
> and commit, each on the team lead's go:** **COMP-a** the library (host-proven, no `src/`), **COMP-b**
> the program (`src/`, a seventh build) and its documentation. **Executor model: `opus`. Do not
> commit.** Live log: `COMP-PROGRESS.md`, created FIRST.
>
> **Why it matters now.** It is the last piece between "the library exists" and "a student codes an
> auton and runs it on the robot": every `autonomous()` in `src/` commands nothing today, so R3b and
> M1 cannot close and no routine a student writes can run in a match. The team lead's goal (2026-09-24):
> "finish CORRECTLY this library in time to code an auton ... but WE AREN'T GOING TO RUSH".

---

## 0. What exists to build on (read these)

| Piece | Where | What matters |
|---|---|---|
| The robot from data | `robot/robot.hpp` | `validate(cfg, devices, BuildOptions)`, `Robot{cfg, devices, BuildOptions}`, `chassis()`, `localizer()`, `faults()`, `describe()`, `portMap()`; **the chassis is built with `devices.pacer()`** (`robot.hpp` ~:488) — see ruling C1 |
| The on-robot devices | `hal/pros/device_provider.hpp` (`ProsDeviceProvider`, G1 Part 2a) | registry-checked adapters by port; `pacer()`; the IMU calibrates once on first request |
| The boot source | G1 Part 2b (`bootConfig()`: the SD record → verify → the typed table → refuse; the absent ports) | COMP boots EXACTLY as "shulib Teleop" does — the same function, not a copy (ruling C4) |
| The driver loop | `teleop/teleop_loop.hpp` (`runDriverControl`), `teleop/driver_control.hpp` (`DriveShaper`, `DriverProfile`) | opcontrol() is this loop, unchanged |
| The guaranteed end | `sequence/run_guard.hpp` (`RunGuard`, `RunGuardConfig{endActionAt, hardStopAt, mechanisms}`, `RunGuardReport`, **F12 unfrozen**) | `RunGuard` IS an `ITickPacer` wrapping the real one; `run(chassis, config, scoring, endAction)` cuts scoring at `endActionAt`, runs the end action, forces safe at `hardStopAt`, and on a throw safes and RETHROWS. **The chassis must be built with the guard as its pacer** or its guarantee never applies (`pacesSeen == 0` warns) |
| Routines | `chassis/routine.hpp` (`Routine`, **F10 frozen**) and `runner/path_runner.hpp` (`PathRunner`, `CommandRegistry`, **F8 frozen**) | an auton is either a function that runs a `Routine` on the chassis, or a `config::Route` run by `PathRunner` |
| A pure screen model | `calibration/calibrate_panel.hpp` (R3d Part 2F-1) | the pattern to reuse: a PROS-free `viewFor()` + `hitTest()` + `TouchGate` (release edge, arming delay, error reads ignored), constexpr ASCII / no-`%` / budget checks; the program only renders |
| The screen facts | the Part 2F investigations and `src/calibrate_program.cpp`'s `printText` | three text sizes; budgets LARGE 18 / MEDIUM 36 / SMALL 50 (derived, not measured); PROS re-reads screen text as a format (no `%`) and leaks every print (redraw on change); usable height 240; touches on release |
| Competition state | PROS `competition_get_status()`, `competition_is_disabled()`, the five entry points in `src/main.cpp` | PROS runs `autonomous()`/`opcontrol()` as tasks it KILLS on a mode change |

---

## 1. Rulings

**C1 — The builder accepts a pacer, so the chassis can be guarded.** `robot::BuildOptions` gains
`motion::ITickPacer* pacer = nullptr` (ADDITIVE; `nullptr` = `devices.pacer()`, today's behaviour, every
existing test bit-identical). The competition program constructs `RunGuard guard{devices.pacer()}`
BEFORE the `Robot` and passes `&guard`; outside a run the guard passes `pace()` straight through, so
driver control is unaffected. Rejected: the provider handing out a guard (the provider owns hardware,
not match policy); a second chassis for autonomous (two facades over one drivetrain).

**C2 — An auton is DATA the students' code registers; the library ships the mechanism, not the
season.** `competition::AutonRegistry` (fixed capacity, no heap, no `std::function`): each entry has a
NAME (ASCII, ≤ the LARGE budget, unique), EITHER a routine function
`control::ExitReason (*)(chassis::Chassis&)` OR a `const config::Route*` (run by `PathRunner` with the
program's `CommandRegistry`), a start pose, its OWN end action (a function; the default is `brake`), and
its OWN schedule (`endActionAt`, `hardStopAt`). The registration file is the STUDENTS'
(`src/autons.cpp`, owned by them, like `drive.cpp` in other libraries); the library's copy holds ONE
clearly labelled test routine for the first on-robot auton (drive 24 in forward, turn 90°, drive back —
the R3b/M1 closing run), never season strategy (the guardrail: students write and defend competition
code).

**C3 — The selector is a PURE, host-tested model; the program renders it.** `competition::AutonMenu`:
`viewFor(menu, …)` → the buttons (one per entry, paged when more than fit; NONE always present and the
DEFAULT), `hitTest`, the same `TouchGate` discipline as the calibrate panel (release edge, arming delay,
error reads ignored). The choice persists from `initialize()` through `competition_initialize()` and
`disabled()` until `autonomous()` reads it; it is shown on every state screen ("AUTON: LEFT SIDE"). A
touch cannot change it once `autonomous()` has started. Rejected: selecting with the controller
buttons only (the brain screen is where the team lead looks; the controller is the driver's).

**C4 — COMP boots exactly as shulib Teleop does.** The boot source (record → verify → table →
refuse, and the absent ports) is ONE function both programs call. **This depends on G1 Part 2b placing
`bootConfig()` in a shared `src/boot_source.cpp`/`.hpp` rather than inside `teleop_program.cpp`** —
proposed as a one-line amendment to the G1 Part 2 brief in this brief's commit; if 2b lands without
it, COMP-b moves it first.

**C5 — Every phase starts safe.** PROS kills the autonomous task when the mode changes; a routine
killed mid-motion must not leave a command on the motors. `autonomous()`, `opcontrol()` and
`disabled()` each begin with `chassis.cancel()` (0 V + the configured stop), and `autonomous()` runs
under the `RunGuard`, whose `hardStopAt` is set inside the period so the guard, not the task kill, is
what stops the robot.

**C6 — The match schedule is stated, never defaulted.** `RunGuardConfig` has no defaults by design.
The VEX U autonomous period's length for Override and the skills run's length are stated by the team
lead from the game manual (UNKNOWN until he does — the program refuses to arm a run whose schedule
is unset, with a sentence saying so). The end action's lead ("park needs ~X s") is the auton's own.

**C7 — Nothing runs from a boot the library refused.** If `bootConfig()` or `validate()` refuses, the
refusal screen stays up in every phase, `autonomous()` does nothing and logs why, `opcontrol()` idles.

---

## 2. Scope — COMP-a (the library; no `src/`)

1. `robot::BuildOptions::pacer` (C1) + tests: default bit-identical; a guard passed in sees every
   `pace()` of a motion (`pacesSeen > 0`) and cuts a routine at `endActionAt` on the plant.
2. `include/shulib/competition/auton_registry.hpp` (C2): `AutonEntry`, `AutonRegistry` (add, find by
   name, count; preconditions: unique ASCII names within budget, exactly one of routine/route, a valid
   start pose, a schedule with `hardStopAt >= endActionAt > 0` or "unset" which C6 refuses to arm).
3. `include/shulib/competition/auton_menu.hpp` (C3): the pure selector model and view, reusing the
   calibrate panel's geometry constants and `TouchGate` discipline (factor the shared parts into
   `include/shulib/ui/` ONLY if that is genuinely simpler than depending on `calibration/` — say why).
4. `include/shulib/competition/match_runner.hpp` — `runAuton(Robot-like chassis, RunGuard&, const
   AutonEntry&, CommandRegistry*, TelemetrySink&) → report`, PROS-free: sets the start pose, runs the
   entry under `guard.run(...)`, returns the `RunGuardReport` plus the routine's own verdict; NONE runs
   nothing and says so; an unset schedule refuses (C6).
5. **Tests (adversarial, on the host test plant):** the end action FIRES when the routine stalls
   (a `waitFor` that never comes true); the hard stop forces safe even if the end action hangs; a
   routine that throws ⇒ the drive is safe and the throw propagates (the program shows a fault); a
   route entry runs through `PathRunner` bit-identically to the same legs as a routine; the menu:
   NONE is the default, every drawn button maps to an entry, paging never hides an entry, a touch
   after autonomous started changes nothing, the double-tap hazard (no new button under the one just
   pressed), ASCII/no-`%`/budget checks; the registry's every precondition. Mutations RUN and observed
   red: skip the end action on a cut; arm with an unset schedule; drop the default NONE; let a touch
   change the choice during autonomous; drop the `BuildOptions::pacer` forwarding.
6. **Docs (COMP-a):** `docs/api` (regenerated), `docs/changelog.md` (additive), the Freeze Register's
   F12 row (RunGuard's second consumer — whether that triggers its freeze is the team lead's call;
   state the evidence, do not freeze), `build-order.md` + `roadmap.md` pointers.

## 3. Scope — COMP-b (the program, a seventh build)

1. `src/competition_program.cpp`/`.hpp` (`namespace shulib::bench`): `initialize()` — the precondition
   handler, `ProsDeviceProvider`, `bootConfig()` (C4), `RunGuard guard{devices.pacer()}`,
   `validate(...)` then the `Robot` constructed ONCE into `std::optional` storage with
   `{.absentDrivePorts, .pacer = &guard}`, the session header from `robot.portMap()`, the menu shown;
   `competition_initialize()`/`disabled()` — the state screen with the chosen auton and the boot
   source; `autonomous()` — C5, then `runAuton(...)` with the chosen entry, the report logged;
   `opcontrol()` — C5, then `runDriverControl` exactly as shulib Teleop.
2. `src/autons.cpp` (the STUDENTS' file, C2) with the one labelled test routine.
3. The seventh build: `make ROBOT=tank PROGRAM=competition`, its beacon, `tools/src_build_gate.py`
   updated (VARIANTS, beacons, and — if G1 Part 2c has not yet replaced it — the variant
   differential's list), the Makefile's PROGRAM validation, `.github/workflows/ci.yml` comment. Slot:
   **5 "shulib Competition"**.
4. Docs (COMP-b): a guide chapter **"The competition program"** written from the code (registering an
   auton, the selector, the schedule and the end action, what happens on a refused boot), every example
   COMPILED in `test/guide_examples_test.cpp` and quoted verbatim; README; the worksheet — a new
   **Station H** (a practice match: the selector, a field-control or competition-switch run, what the
   screen and the log should say, the test routine's expected end pose); `build-order.md` + `roadmap.md`
   (M4's guaranteed-park line: the guarantee now reaches a real program — still unrun);
   `COMP-COMPLETED.md` at the end.

## 4. Landmines
1. `-Wformat-nonliteral`/`-Wformat-truncation` are errors in `src/`; only a literal `"%s"` reaches a
   screen print; no `%` on screen; ASCII only; y < 236.
2. PROS kills the autonomous/opcontrol tasks on a mode change — nothing in `autonomous()` may rely on
   running to its end; the guard's `hardStopAt` must sit inside the period.
3. The `Robot`, the provider and the guard are non-movable; construct once, from `initialize()`, into
   program-lifetime storage (a throwing function-local static is re-attempted).
4. The guard must be the chassis's pacer (C1) — assert `pacesSeen > 0` after a run in the log.
5. The IMU is calibrating for ~2 s after boot; motions wait for a live estimate (the watchdog runs
   through the wait) — an auton started early is slow, not wrong; say so in the guide.
6. No hardware claim: the program is UNRUN until Station H's log says otherwise.

## 5. What this brief does NOT decide
The schedule numbers (C6), which routines exist (the students'), the mechanisms and their end
states (handoff Q1), button bindings (T3), field-centric for the driver (off by default, as Teleop).
