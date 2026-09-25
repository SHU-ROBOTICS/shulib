# COMP — the competition program: boot the robot, pick an auton on the brain screen, run it under a guaranteed end, drive

> **Execution brief.** First written 2026-09-24 at `2780462`; **REVISED the same day at `8ac243f`** after a
> read-only checker compared it with the tree (sixteen findings, the load-bearing ones re-read in the source
> by the coordinator) and the team lead ruled on the two design questions it raised (below). It is the
> handoff's item **L2**. **Two parts, each its own executor run, verification and commit, each on the team
> lead's go, after G1 Part 2b lands:** **COMP-a** the library (host-proven, no `src/`), **COMP-b** the
> program (`src/`, a seventh build) and its documentation. **Executor model: `opus`. Do not commit.** Live
> log: `COMP-PROGRESS.md`, created FIRST.
>
> **The team lead's rulings (2026-09-24):**
> 1. **Start heading — "can we do both?": yes.** Each auton declares EITHER a FIELD start pose (x, y,
>    heading, in the field frame) OR "RELATIVE" (wherever the robot is placed becomes (0, 0) facing 0). In
>    both cases the library RE-SEATS the pose — position AND heading — at the moment the auton starts
>    (ruling C2). This also clears whatever the IMU drifted while the robot waited before the match.
> 2. **A mid-auton disable — "our own auton task".** The auton runs on a task the PROGRAM owns and PROS never
>    kills; on a disable or a mode change the program ends the routine itself, cleanly, and safes the robot
>    (ruling C5). Rejected: letting PROS kill `autonomous()` and repairing the library afterwards — the
>    checker showed a killed task leaves the scheduler's active motion dangling on a freed stack
>    (`motion_scheduler.hpp` ~:640-666, :697, :807-808), its in-tick/in-wait flags set, and the guard's
>    `running_` true, so the next phase would call through freed memory.

---

## 0. What exists (read these; every line was re-checked at `8ac243f`)

| Piece | Where | What matters |
|---|---|---|
| The robot from data | `robot/robot.hpp` | `validate(cfg, devices, BuildOptions)`, `Robot{cfg, devices, BuildOptions}`; **the chassis takes `devices.pacer()` at ~:488** and keeps it for life (`chassis.hpp` ~:239-246), while `BuildOptions` is documented "read during construction only" (~:136-138, :427-429) — see C1 |
| The on-robot devices | `hal/pros/device_provider.hpp` | the IMU is made with **boot heading 0** (~:71-73) — see C2 |
| The boot source | G1 Part 2b's shared `src/boot_source.hpp`/`.cpp` (2b, amended at `6456beb`, re-checked at `8ac243f`) | returns its outcome; COMP boots exactly as shulib Teleop does |
| The screen text | 2b's shared `src/` screen-text header (`printText` moved out of `calibrate_program.cpp`) | the one safe print path: ASCII, no `%` (PROS re-reads printed text as a format), bounded, a literal `"%s"`; repaint on change (every print leaks) |
| The driver loop | `teleop/teleop_loop.hpp` (`runDriverControl`, `DriverLoopConfig`, `HeadingSample`) | opcontrol() is this loop, as in Teleop |
| The guaranteed end | `sequence/run_guard.hpp` (**F12 unfrozen**) | `RunGuard(ITickPacer& inner)` (~:240); `run(chassis, config, scoring, endAction)` (~:375-377); `RunGuardConfig{endActionAt, hardStopAt, mechanisms}` (~:163-194) whose fields DEFAULT to 0.0 and whose `validate()` THROWS on them (~:167-189); `RunGuardReport{…, pacesSeen}` (~:198-218) — `pacesSeen` counts only during `run()` and resets at arm (~:258-259, :446); a throw safes and RETHROWS (~:366-369); inert when idle; the guard's `waitFor`/`pause`/`expired` (~:92-107) are the only waits its deadline can cut |
| Routines | `chassis/routine.hpp` (**F10 frozen**): `startAt` calls `setPose`, which IGNORES heading (`localizer.hpp` ~:469-481); `RoutineResult` (~:169-183) | an auton function returns the `RoutineResult`, not a bare `ExitReason` (C3) |
| Routes | `runner/path_runner.hpp` (`PathRunner`, takes `const CommandRegistry&`, ~:164-165), `runner/command_registry.hpp` (`CommandRegistry`, ~:116) (**F8 frozen**) | a route auton needs a registry (non-null precondition) |
| The heading composition | `localization/localizer.hpp`: the published heading is `imu.heading() + headingBias_` (~:16, :35, :253-254), odometry rotated into that frame when the bias is non-zero (~:245-247), the bias kept by `setPose` (~:469-481) | the start-heading re-seat reuses this, rather than touching the IMU (C2) |
| A pure screen model | `calibration/calibrate_panel.hpp` | its RECTANGLES, hit boxes, budgets, `TouchSample` and arming delay are generic (~:73-139, :161-176, :329, :913-923); its `TouchGate` is typed to the calibrator (~:958, :974-975) and caps at 3 buttons (~:234) — see C4 |
| Competition state | `include/pros/misc.h`: `competition_get_status` (~:34-36, :75), `competition_is_disabled` (~:95) | the vendored headers say NOTHING about task killing; the belief lives only in code comments today (~`main.cpp`:447-450) — an HA entry is owed (C5) |

---

## 1. Rulings

**C1 — The builder accepts a pacer, borrowed for the Robot's lifetime.** `robot::BuildOptions` gains
`motion::ITickPacer* pacer = nullptr` (ADDITIVE; `nullptr` = `devices.pacer()`, every existing test
bit-identical). COMP-a REWRITES `BuildOptions`' and the constructor's doc comments: `absentDrivePorts` is
read during construction only, but **`pacer` is borrowed for the Robot's whole life** (the chassis keeps it).
Spelled `robot::BuildOptions{.absentDrivePorts = …, .pacer = &guard}` (designated initializers in
declaration order). The competition program constructs `RunGuard guard{devices.pacer()}` BEFORE the Robot;
outside a run the guard passes `pace()` through.

**C2 — The start pose is re-seated when the auton starts, position AND heading (ruling 1).** An auton
declares `StartPose::field(x, y, heading)` or `StartPose::relative()` (= (0, 0), heading 0). At the start of
the run — never at boot, because the choice is made after the Robot exists — `runAuton` sets the position
(`setPose`) AND the heading through a NEW, ADDITIVE `Localizer::reseatHeading(math::Angle)`: it sets
`headingBias_` so the published heading equals the given one at this tick, reusing the composition the
AprilTag corrector already relies on (the IMU stays the sole source of heading CHANGE). `Routine::startAt`
(F10, frozen) is NOT changed — it still ignores heading; the guide says so and says `runAuton` does the
re-seat. **HA-05 is amended** (the one owner of the start heading is the chosen auton's start pose, applied
at autonomous start via the localizer's bias, not the IMU's boot heading); the amendment and its reasoning
go in the register. Host tests: a field start at 90° drives a 90°-rotated routine to the right place on the
plant; relative mode after a simulated pre-match drift zeroes it; GPS and tag corrections still compose.

**C3 — An auton is DATA the students' code registers; it gets a context, and returns a `RoutineResult`.**
`competition::AutonRegistry` (fixed capacity, no heap, no `std::function`): each entry has a NAME (ASCII,
unique, ≤ the MEDIUM budget so "AUTON: <name>" fits one MEDIUM line), EITHER
`chassis::RoutineResult (*)(AutonContext&)` OR a `const config::Route*`, a `StartPose` (C2), its end
action (default: `brake`), and its schedule. `AutonContext` carries `chassis::Chassis&`, the `RunGuard&`
(its `waitFor`/`pause` are the waits the deadline can cut), the `hal::ITelemetrySink&`, and a
`std::span<hal::IMechanism* const>` (empty today; the guard safes whatever is listed). Registration is an
explicit call from `initialize()` into the students' file — **never at static-initialization time** (the
file is compiled into every build). The library ships ONE labelled test routine (drive 24 in, turn 90°,
drive back — the R3b/M1 closing run); the season's autons are the students' (the guardrail).

**C4 — The selector is a PURE, host-tested model with its OWN generic touch gate.** Build the generic parts
(rectangles, hit boxes, budgets, `TouchSample`, the arming delay, a button-id-agnostic gate) in
`include/shulib/ui/`, and `competition::AutonMenu` on them. **`calibrate_panel.hpp` is NOT changed** (the
pending bench session's Calibrate runs on it); moving it onto `ui/` is a later, separate change. The menu:
one button per entry, paged when more than fit, NONE always present and the DEFAULT; the choice is one
atomic store, readable from any task; a touch cannot change it once the auton has started. Which task polls
the screen, per case, is stated in the program: a field or competition switch attached
(`competition_initialize()` / `disabled()`), nothing attached (`opcontrol()` starts at once — the menu shows
until the driver's first stick input), and HA-128's pretend match from the controller menu.

**C5 — The auton runs on the program's OWN task; the program ends it (ruling 2).** `autonomous()` starts or
signals a program-owned `pros::Task` that PROS does not kill, then waits; the auton task runs `runAuton`. The
pacer under the guard checks the competition state every tick: on a disable or a mode change it ends the
run through a NEW `RunGuard::abandon()` (ADDITIVE to F12, unfrozen) — cancel into the safe state, skip the
end action (a disabled robot's motors are off; the park cannot run), unwind the routine normally on its own
stack so nothing dangles, disarm. `opcontrol()` and `disabled()` wait (bounded, e.g. ≤ 100 ms) for the auton
task to report finished before commanding anything; if it has not, they force the drive safe and say so.
An exception inside the auton is caught ON the auton task: the robot is safed, a fault screen and a log line
say what broke. **Register an HA entry** for the belief that PROS kills its competition tasks
(`autonomous`, `opcontrol`, `disabled`, `competition_initialize`) on a mode change — Station H settles it.

**C6 — The schedule is stated, never defaulted, and bounded by the period.** From the official Override
Game Manual v1.0 (2026-07-02; the team lead asked the coordinator to look it up): **VEX U Head-to-Head —
a 30 s Autonomous Period (`<VUT4>`), then 90 s of driver control (`<VUT5>`); VEX U Autonomous Coding
Skills — 60 s, two robots; Driving Skills — 60 s** (Appendix B's Match table). The program states these as
its two periods (match 30 s, skills 60 s); each auton states its own `endActionAt`/`hardStopAt` inside
its period. `runAuton` detects an
UNSET schedule (the fields' 0.0 defaults) and refuses with a sentence BEFORE calling `run()` (which would
throw); the program also refuses any entry whose `hardStopAt` exceeds the period it runs in.

**C7 — Nothing runs from a boot the library refused.** The refusal screen stays in every phase;
`autonomous()` logs why and does nothing; `opcontrol()` idles.

---

## 2. Scope — COMP-a (the library; no `src/`)

1. `BuildOptions::pacer` + its doc rewrite (C1). Tests: default bit-identical; a guard passed in sees every
   `pace()` of a motion (use a COUNTING inner pacer — `pacesSeen` counts only during `run()`); a routine is
   cut at `endActionAt` on the plant.
2. `Localizer::reseatHeading` (C2) + HA-05's amendment. Tests as in C2, plus: the bias composes with a later
   tag/GPS correction; `setPose` after a re-seat keeps it.
3. `include/shulib/competition/auton_registry.hpp` (C3): `StartPose`, `AutonContext`, `AutonEntry`,
   `AutonRegistry` (preconditions: unique ASCII names within budget, exactly one of routine/route, a route
   entry requires a registry at run time, a schedule set or "unset").
4. `include/shulib/ui/` generic panel pieces + `include/shulib/competition/auton_menu.hpp` (C4).
5. `include/shulib/competition/match_runner.hpp` — `runAuton(chassis::Chassis&, sequence::RunGuard&,
   const AutonEntry&, const runner::CommandRegistry*, hal::ITelemetrySink&,
   std::span<hal::IMechanism* const>, units::Time period)`: refuse an unset schedule or one past the period
   (C6); re-seat the start pose (C2); run under `guard.run(...)`; return the `RunGuardReport` plus the
   routine's `RoutineResult`. `RunGuard::abandon()` (C5), additive.
6. **Tests (adversarial, on the host plant):** the end action FIRES when the routine stalls in `guard.waitFor`
   (the deadline cuts it) — and a separate case pins how late a wait the guard CANNOT cut ends; the hard stop
   forces safe if the end action hangs; `abandon()` mid-motion leaves the scheduler idle and callable, the
   guard disarmed, and the drive safe, with the routine unwound (no dangling); a throwing routine ⇒ safe;
   a route entry runs bit-identically to the same legs as a routine; the start-pose cases (C2); the menu
   (NONE default, every drawn button maps to an entry, paging hides none, no change once started, no new
   button under the one just pressed, ASCII/no-`%`/budget checks); the registry's preconditions. Mutations
   RUN and observed red: skip the end action on a cut; arm an unset schedule; drop the heading re-seat; drop
   `abandon()`'s cancel; drop the default NONE; allow a touch to change the choice after start; drop the
   `BuildOptions::pacer` forwarding.
7. **Docs (COMP-a):** `docs/api` (regenerated); `docs/changelog.md` (additive: `BuildOptions::pacer`,
   `Localizer::reseatHeading`, `RunGuard::abandon`, the new headers); HA-05's amendment and the new
   task-kill HA entry; the Freeze Register's F12 row (a second consumer — the freeze is the team lead's
   call; state the evidence, do not freeze); `build-order.md` + `roadmap.md` pointers.

## 3. Scope — COMP-b (the program, a seventh build)

1. `src/competition_program.cpp`/`.hpp` (`namespace shulib::bench`): `initialize()` — the precondition
   handler, `ProsDeviceProvider`, the shared boot source, `RunGuard guard{devices.pacer()}`, `validate(...)`
   then the Robot ONCE into `std::optional` with `{.absentDrivePorts = …, .pacer = &guard}` (the `try` wraps
   both), the session header, the students' registration call (C3), the auton task created (C5), the menu
   (C4); `competition_initialize()` / `disabled()` — C5's wait, the state screen with the choice and the boot
   source; `autonomous()` — start the auton task, log the chosen auton by name (the session header was
   printed before the choice), wait; `opcontrol()` — C5's wait, then `runDriverControl` as in Teleop.
2. The students' autons file in `src/` with the one labelled test routine (C3).
3. **The seventh build, everywhere it must be named:** `src/main.cpp`'s variant checks (the one-PROGRAM
   `#error`, the needs-`ROBOT=tank` check, the `SHULIB_RUNS_*` macros, the beacon chain — beacon
   `shulib-robot-variant=tank-competition`), the Makefile's PROGRAM validation and its "Four programs"
   comment (~:102), `tools/src_build_gate.py` (`VARIANTS`, `BUILD_ARGS`, `VARIANT_DEFINES`, `BEACONS`, its
   header table, a self-test pair like its cases 14/15, and — if G1 Part 2c has not yet replaced it — the
   variant differential's list), `.github/workflows/ci.yml`'s comment. Slot **5 "shulib Competition"**.
4. Docs (COMP-b): a guide chapter **"The competition program"** from the code (registering an auton, the two
   start modes, the selector, the schedule and the end action, a mid-auton disable, a refused boot), every
   example COMPILED in `test/guide_examples_test.cpp` and quoted verbatim; README; worksheet **Station H**
   (a practice match: the selector, a competition-switch or field run including a mid-auton disable, what the
   screen and the log say, the test routine's expected end pose; it settles the task-kill HA entry);
   `build-order.md` + `roadmap.md` (M4's guaranteed-park line: reaches a real program — still unrun);
   `COMP-COMPLETED.md`.

## 4. Landmines
1. `-Wformat-nonliteral`/`-Wformat-truncation` are errors in `src/`; one print path (2b's shared header);
   no `%` on screen; ASCII; y < 236.
2. The auton task must never command the drive after `opcontrol()` has started it — C5's handshake, tested
   on the host where it can be (the pure parts) and read line by line where it cannot.
3. The Robot, the provider, the guard and the auton task live for the program; construct once, from
   `initialize()`.
4. Assert `pacesSeen > 0` after a run, in the log (C1).
5. The IMU calibrates ~2 s after boot; motions wait for a live estimate — an auton started early is slow, not
   wrong; the guide says so.
6. No hardware claim: the program is UNRUN until Station H's log says otherwise.

## 5. Not decided here
Which routines exist (the students'), the
mechanisms and their end states (handoff Q1), button bindings (T3), field-centric for the driver (off, as
Teleop).
