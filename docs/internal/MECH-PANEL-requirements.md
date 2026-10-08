# Mechanisms and the robot panel — the requirements so far (PLANNING, not a brief)

> **Status: PLANNING with the team lead, 2026-09-28.** This collects everything he has asked for about
> mechanisms and the robot panel, so it can be checked in one place. It is **not an execution brief**: he
> asked that no brief be written "before we are 100% confident that we have everything down". When he says it
> is complete, it becomes the input to the briefs (MECH for the mechanisms; an amendment of G3a's brief for the
> panel; G3's for `.vexbot`).
>
> **Revised 2026-09-28 after an independent check** of this list against his words, the accepted idea list,
> the plan and the code. Every finding is folded in: the per-tick mechanism service (§3.0), honest PTO wording
> (P7, §3.7), the frozen surfaces touched (§6), honest sizes (§7), and 29 questions that are his (§5). Lines
> marked **Proposed** are design defaults a brief would otherwise have to guess; he can overrule any of them.
> Sizes are unmeasured ranges with their reasoning; a brief measures them. [`REMAINING-WORK.md`](REMAINING-WORK.md)
> lists where each piece sits in the plan; its MECH and panel sizes predate this revision (§7 has the new ones).
> Kept current by the coordinator.

## 0. What he said (verbatim, in order)

1. "I feel like it shouldnt matter what mechanism a robot has? We should be able to make any mechanism that we
   need to that is important to the rework of this library"
2. "also mechanisms should be port agnostic, swapping around cables is a normal thing and is done frequently
   so we shouldnt make the mechanisms be made with an exact port in mind when they are created but they are
   bounddd to ports after they are created if that makes sense"
3. "I also want mechanisms the ability to add take away etc in the panel so that when you make a robot it is
   super simple"
4. "also for the future of the panel you should be able to import vexbots for people that design the bot in
   vexbuilder first anddd export something for vexbuilder to be able to use to get started"
5. On the idea list: "the rest of them are also very good so lets plan for all for the first version and we
   will make it PTO ready" — and on a power take-off: "I dont think weve used a PTO before ... if it is helpful
   and not hardware then yes lets do it".
   **His acceptance, as he gave it, is conditional on "not hardware".** The coordinator answered that a PTO is
   hardware (a gearbox that shifts motors between the drive and a mechanism). So, by his own condition, no PTO
   support is built and the design stays PTO-ready. What "PTO-ready" must mean in the code is his to choose
   (§5 Q4).
6. "there is more to plan about the mechnisms so maybe dont write the brief now before we are 100% confident
   that we have everything down"
7. On the sizes, choosing between everything in the first version and the core first: "lets do this so we have
   it, we can do the rest after the panel" — **ruled (2026-09-28, ~17:15): the CORE first** (the four control kinds —
   power, position, speed, pneumatic —, port binding, auton steps and presets, with what they need: the mechanism
   service, the provider accessors, the host-plant models), **before the panel; the rest after the panel** (the
   coordinator places it before G3, so the robot-file format is locked with VexBuilder once). See §7.2.

## 1. The principles (these decide every detail below)

- **P1 — any mechanism.** The library supports any mechanism a VEX robot can have. What a given robot has
  (the handoff's Q1) decides only that robot's SETTINGS; it never holds up library work. The plan's old gate
  (build-order F3/F4 "needs hardware + final mechanisms") applies only to season-specific STRATEGY, which is
  the students' code built from the library's pieces (the plan changes this needs are drafted in §7.3).
- **P2 — port-agnostic.** A mechanism is defined by what it IS — its ROLES ("arm motor 1, reversed", "limit
  switch", "piece sensor") — never by a port. Ports are BOUND to roles afterwards, in a separate wiring step.
  Re-cabling changes only the binding. A motor's direction belongs to the role (how it is mounted), not the
  port. The drivetrain already works close to this: Calibrate discovers the drive ports and directions on the
  robot, and the robot boots from that record. The typed drive table still exists as the fallback; whether it
  stays in the robot file is §5 Q21. This changes the published VexBuilder schema and G3a's robot file (§4.10).
- **P3 — one description, used everywhere.** The same mechanism is used in autons (plain steps), the driver
  file (buttons) and the robot file / panel (data), and is generated, never written twice.
- **P4 — every action is bounded and says how it ended.** A time limit on everything; a verdict at the end
  (reached target / time up / stalled / a sensor saw or stopped seeing a piece / refused by an interlock /
  not homed / cancelled), logged like a drive motion. **Nothing sits at stall current beyond its stated cap**
  (a squeezing claw and robot two's Hold-on-release pivots hold at a capped current, §3.1, §5 Q16).
- **P5 — safety carries over.**
  - One action per mechanism at a time (the existing claim).
  - At the end of an auton, every mechanism goes to its declared safe state. The guard's cancel-all reaches
    every running behaviour, because the mechanism service is a claimant (§3.0).
  - Driver control takes back anything an auton left running (T3 Part 2's release at loop entry).
  - When two controls act on one mechanism, a stated **arbitration rule** decides (§3.3, §5 Q8).
  - **The build refuses** the same button AND gesture used twice, and two controls of the same KIND on one
    mechanism. It no longer refuses an arm that has a stick, presets and a cycle button together.
  - **Proposed:** a mechanism with no driver control is allowed when it is marked auton-only, so a forgotten
    control is still caught (§5 Q8). Today's `controlsCoverTable` check refuses it outright.
- **P6 — the team owns the numbers.** Ports, gearing, limits, presets, thresholds, timings and strategy are the
  team's. Gains ship with starting values and are MEASURED on the robot later, never guessed into a match.
  No threshold is invented as a default (HA-51's invented 5 s default is the cautionary tale).
- **P7 — PTO later, not PTO-built.** No PTO support is built (§0.5). The design must not make a later PTO
  harder than it has to be. It is **not** PTO-ready for free today. Three rules refuse a port shared by the
  drive and a mechanism:
  - `RobotConfig::valid()`;
  - G3a's generator ("a claw port that is a drive port");
  - the VexBuilder contract §3.2 ("may not share a port with the drivetrain").

  The motor group also has no ownership hand-over. T4 Part 2's planned `markAbsent` (briefed, not yet built)
  is not one: it latches a member absent for the group's whole life and keeps commanding it. Whether PTO-readiness is built now or listed as
  that later change is his choice (§3.7, §5 Q4).
- **P8 — the robot file is the `.vexbot` profile.** The file the panel edits is exactly the block a `.vexbot`
  carries, so import and export are one format going both ways. That holds only if VexBuilder keeps the
  fields it does not understand (§4.10, §5 Q20).

## 2. What exists today (so nothing is rebuilt)

| Piece | Where | What it does |
|---|---|---|
| Mechanism devices | `hal/mechanism.hpp` (F1) | `MotorMechanism` (a group of motors on one shaft, commanded by VOLTAGE only; reads velocity; no position reader at this level), `PneumaticMechanism` (every line set to ONE level), `IMechanism` (`applySafeState()` — synchronous, "no further tick required" — + `name()`), the declared safe state, the one-operation claim with an optional claimant the guard can cancel |
| Bounded operations | `manipulation/mechanism_op.hpp` (F1) | `RunUntilConfirmed`, `ActuateAndConfirm` (with an honest `Unconfirmed` verdict), the tick contract (no own loop), the blocking idiom `chassis.waitUntil([&]{ return op.tick() != Running; }, bound)`; an open-ended behaviour is "deliberately NOT an operation" |
| Stall detection | `manipulation/stall_detector.hpp` | the stall signal with required thresholds |
| The end-of-run guard | `sequence/run_guard.hpp` (F2) | `RunGuard` IS the tick pacer's decorator: it checks the deadlines, cancels every claimant, runs the end action. **F2 retired the parallel/race/deadline combinators**: "intake while driving" exists only as the waitUntil-predicate idiom, and "there are no background tasks" |
| Motor reads | `hal/motor.hpp` (F4, LOCKED) | voltage-only command; reads position, velocity, current, temperature (seeded 20 °C, last good value on a failed read); no current-limit setter |
| Control pieces | `control/` | PID, trapezoid profile, feedforward — position and speed control reuse them |
| The IMU's pitch | `hal/imu.hpp` | `pitch()` exists; its sign on the robot is unmeasured (HA-110) |
| Mechanisms as config | `config/robot_config.hpp` | `MechanismConfig` (motor or pneumatic, never both; ports; cartridge; safe mode), `kMaxMechanisms`; a mechanism port that is a drive port is refused |
| The robot builds them | `robot/robot.hpp` | `mechanism(name)`, `pneumaticMechanism(name)`, `builtMechanismAt(i)`; an absent one is ABSENT and the drive runs |
| Driver buttons | `teleop/mechanism_binding.hpp`, `teleop/teleop_loop.hpp`, `src/common/driver_controls.hpp`, `src/team/robot_two_driver.hpp` | hold-to-run and toggle, bound BY NAME (already port-free); `runDriverControl` releases a leftover claim at loop entry; compile-time checks refuse a button used twice and a mechanism with no control |
| Autons | `chassis/routine.hpp` (F10, LOCKED), `competition/auton_registry.hpp` (COMP-a) | EAGER steps (each runs when chained; a deferred chain was rejected); `then()` (unfrozen) already accepts a `MechanismOutcome`; `AutonContext` holds the mechanisms for the guard's safing only — no lookup by name |
| Route markers | `runner/path_runner.hpp` (G2, F8) | fire only AFTER a leg settles, never partway; a timed-out leg fires none (ruling R2) |
| The controller screen | `diag/controller_display.hpp` | all three rows used by the fault display in Drive and Teleop; row 0 is rewritten ~10 times a second |
| Robot two's mechanisms | `src/robots/mechanism_table.hpp` | the claw pivots on ports 6 and 7 (power only, Hold on release), the pneumatic claw on line A (with ports — P2 changes this) |
| Sensor adapters | `hal/pros/{distance,optical,rotation,digital_in}.hpp` | readings with their sentinels screened. The device provider hands out **rotation** today (tracking wheels); **distance, optical and digital-in are not handed out** |

**Missing:** the per-tick mechanism service, position control, homing, travel limits, speed control, cycle
mechanisms, generic done-conditions, auton steps for mechanisms and an auton's way to reach them, the
provider's distance/optical/digital-in accessors, a current-limit seam, a controller output (rumble), presets
and the other driver controls, port binding separate from the definition, and the mechanism pieces of the
panel.

## 3. Mechanisms — the first version (everything below is IN)

### 3.0 The mechanism service (NEW — Part A; most of this list depends on it)

Several behaviours must run on EVERY tick, including the time inside a blocking drive verb and every tick of
driver control:
- a position hold with its gravity term, and speed control;
- a cycle's re-wind, and interlocks "checked every tick";
- jam reverse, piece counting and colour sorting;
- heat derating, and the lift height that tip protection reads;
- any mechanism step running beside a drive.

Nothing does this today. F2 built no parallel engine, and an open-ended behaviour is "deliberately NOT an
operation". The requirement:

- **One mechanism service, ticked once per control tick.**
  - **In autons, from the pacer seam.** The service is a pacer decorator composed with `RunGuard`, in this
    order: the guard checks its deadlines first (its check-then-step order is pinned by test), then the
    service ticks, then the world steps. A cut at the deadline therefore cancels before the service could
    re-command.
  - **In driver control, from `runDriverControl`.** It ticks at a stated place in the loop: after the
    bindings read the buttons, before `perTick`. At loop entry the T3 Part 2 ruling releases a leftover claim
    WITHOUT calling `cancel()`, so the service must drop its auton-era state at entry. It must never go on
    believing it holds a claim.
- **A claimant.** Every mechanism the service drives is claimed with the service registered as the
  `ICancellable`. The guard's cancel-all then stops it (inert + safe + claim released). A stale tick can never
  re-command a mechanism the guard made safe; `mechanism.hpp` measured that "half-safe" failure. Interlocks
  are evaluated inside the service, so no command path bypasses them.
- **Re-entrancy against C2's `waitUntil`.**
  - An operation has exactly ONE ticker: the service or a caller's `waitUntil` predicate, never both. The
    claim records which.
  - The service's tick never calls a blocking verb, `waitUntil` or `scheduler.cancel()` (C2's re-entrancy
    list).
  - A predicate may read the service's state but never tick it.
  - **Exactly once per tick**, even where both seams run in one loop (a driver loop pacing through the same
    pacer): the service ticks from one seam or the other, never both.
- **Stated honestly in the guide:** the service is ticked only inside a shulib motion, wait or the driver
  loop. A student's own busy loop starves it, the same limit `RunGuard` states ("nothing preempts pure user
  code").
- **Run while driving comes from the service** (moved here from the old Part C): an intake started by an
  auton step keeps running during the next `driveTo` (§3.4).
- Sized on its own (§7).

### 3.1 Kinds of control

- **Power** — run at a set voltage (intakes, rollers, conveyors, claw pivots). Exists as `setVoltage`; gains
  the done-conditions, limits and protection below.
- **Position** — go to an angle or height and hold it: arms, lifts, tilters, claw pivots with an encoder.
  - Units the team thinks in: degrees (from the motor encoder through the gearing, or a rotation sensor), inches
    (a spool's diameter, or a lift's geometry).
  - **Non-linear lifts. Proposed:** a single arm by its length and pivot height, and a four-bar or a double
    reverse four-bar by the same formula per stage (a parallel linkage keeps the end level). Any other shape
    (a chain-bar, an odd linkage) uses a **lookup table**: pairs of (height in inches, angle in degrees), at
    least two, strictly increasing, interpolated linearly. The build refuses a table that is not monotonic.
  - **Smooth moves:** accelerate, cruise, slow down (the existing trapezoid profile), not a slam to the target.
  - **Gravity compensation:** an arm needs more power held out flat than straight up (a term that follows the
    cosine of the angle from horizontal); a linear lift needs a constant term. The angle at which the arm is
    horizontal is a setting, or is measured by auto-tuning (§3.6). **Loaded gains:** a piece-sensor role
    switches to a second gain set when holding a piece. The switch is bumpless: the output does not jump.
  - **Holding and low-power hold:** at the target it holds. The low-power hold holds with just enough current
    instead of full Hold heating the motor all match. There are two ways to build it (§5 Q15):
    - **Firmware.** Hold plus a reduced per-motor current limit (PROS `motor_set_current_limit`, default
      2500 mA). It needs a new seam beside the LOCKED `IMotor` (§6). It is synchronous, so it survives a
      disable and the end of an auton with nothing ticking: that IS its safe state.
    - **Software.** A gravity term plus a small P on voltage, ticked by the service. It cannot be the
      synchronous safe state (`mechanism.hpp`: "a safe state that depends on someone continuing to tick can
      leave things energized"). At a disable or the end of an auton, the mechanism falls to its declared safe
      state (full Hold or Coast), and that must be stated per mechanism.
  - **Travel limits — two rules:**
    - **Refused before a match:** a CONSTANT target outside the limits (a preset, a literal auton target, a
      macro's step) is refused at build where the target is a compile-time constant, otherwise at boot. The
      refusal names the mechanism, the target and the limit.
    - **A runtime soft stop:** every command while running is clamped at the limits. That covers stick
      control, computed targets (a preset plus an offset, a non-linear conversion) and encoder drift; a power
      command pushing past a limit is cut at the limit.
    - **Proposed:** before homing, the limits are unknown. Position moves are refused (a NOT_HOMED verdict),
      and stick control runs at a reduced, team-set power with the controller saying so.
  - **Homing:** find zero with a limit switch, or by driving gently until it stalls. The seam question F14 was
    built on (switch or stall) is answered "both" by P1 (to confirm, §5 Q26). An absolute rotation sensor
    with a stored offset needs no homing. **When homing happens in a match** matters because, under field
    control, the robot boots while DISABLED and nothing can move before autonomous. The library supports
    every moment; the auton or driver file says which one a mechanism uses:
    1. **at boot, without motion** — a limit switch already pressed, or the arm declared to rest on its hard
       stop at boot;
    2. **at the start of autonomous** — costs match time, which the auton pays and the log shows;
    3. **at driver-control start** — if the auton did not home;
    4. **on request** — a button or an auton step.

    Which applies to robot two's pivots is §5 Q6. They are power-only today, so homing matters only once they
    gain positions.
- **Speed** — spin at an RPM and report when it is up to speed (flywheels, some intakes). **Proposed:**
  shulib's own loop on voltage (feedforward plus PI in the service; `IMotor` is voltage-only by design). "At
  speed" means within a team-set band for a team-set time.
- **Pneumatic** — open/closed (exists), plus **three positions** from two lines (e.g. a hood: down, middle,
  up). This needs a new pneumatic kind: today's `PneumaticMechanism` sets every line to ONE level.
  **Proposed:** the team states each position's two line levels and the safe position per mechanism, with no
  default (the `pneumaticSafe` precedent: only the team knows the plumbing). A line state no position names
  is refused. Any position may go to any other (§5 Q13).
- **Cycle** — a catapult or puncher: wind down until a limit switch or rotation sensor says loaded, fire,
  re-wind — one control kind instead of every team hand-writing it. Slip gear or latch, and whether one
  mechanism may mix a motor and a pneumatic, are §5 Q12 (`valid()` and the contract refuse a mixed mechanism
  today).

### 3.2 Conditions, protection and interlocks

- **Done-conditions** (any combination, always with a time limit): reached the target; time elapsed; stalled;
  a sensor sees a piece / stops seeing one; a count reached.
- **Piece counting. Proposed:** a count goes up on each rising edge of a piece sensor at the entry (with a
  team-set debounce time) and optionally down at a second sensor at the exit. It resets at program start, on
  an explicit reset step or button, and when the exit sensor empties it. "3 loaded" is a done-condition
  ("until count ≥ 3") and a controller-screen field; it drives nothing by itself.
- **Colour sorting** — the optical sensor sees a wrong-colour piece and the mechanism rejects it.
  - **Proposed:** the optical sensor's LED is switched on at a team-set brightness in the adapter's
    construction config. `IOptical` is read-only and LOCKED (§6).
  - **Proposed:** a reject is one of three settings: reverse the intake for a time; fire a pneumatic eject
    line; flick a separate motor for a time. Each fires after a team-set delay from the sensor, measured on
    the bench page.
  - Where "our colour" comes from is §5 Q11. The auton menu has no alliance choice today.
- **Jam auto-reverse. Proposed:** on a stall (the stall detector, with the team's thresholds), the intake
  backs off for a team-set time at a team-set voltage and retries. After a team-set number of tries it stops
  in its safe state with a JAMMED verdict and one log line.
- **Interlocks** — rules between mechanisms, written as data and checked every tick by the service: "don't fire
  the catapult while the claw is open", "the intake stops while the lift is up".
  - **Proposed vocabulary:** a condition is one of "mechanism X is above/below position p", "pneumatic X is
    in state s", "mechanism X is running" or "sensor Y sees a piece". Conditions combine with AND only in the
    first version.
  - **Required by P5 and the guard's own contract** ("safety is not negotiable, going somewhere is"): an
    interlock never blocks safing. The guard's cancel-all, the
    end action's safe states and a disable bypass every interlock.
  - What happens on a violation, whether the driver gets an override, and an interlock that names an ABSENT
    mechanism are §5 Q17.
- **Current limits and heat.**
  - Each mechanism gets a current cap. Firmware or software is §5 Q15. A per-motor firmware cap needs a new
    seam beside `IMotor` (§6).
  - How per-motor caps interact with the brain's own current sharing when more than 8 motors run is not in
    the vendored PROS docs and is unmeasured; robot two already runs 10 (8 drive + 2 pivots). **An HA is to
    be registered** for it, and one for the firmware's own thermal limiting.
  - **Heat, proposed:** the warn and derate thresholds are team settings with no invented default. The one
    threshold that is not ours is the firmware's over-temperature flag (`motor_is_over_temp`).
  - A temperature from a port that never answered reads 20 °C (`ProsMotor` seeds it). The warning therefore
    shows "temp unknown" when the reads have failed (`faultedReads()`), never "cool".
  - The driver is warned on the controller (§3.3's budget).
- **Tip protection** — with a lift above a team-set height, the drive's acceleration is limited. The IMU's
  `pitch()` exists; its sign on the robot is unmeasured (HA-110). Whether this applies in autons as well as
  driver control, and whether pitch triggers anything or is only logged, are §5 Q14. In autons it touches the
  motion layer: a `MotionOptions` field, F6-additive (§6).

### 3.3 Driver control (in the driver file, beside today's hold and toggle)

- **Presets** — a button sends a mechanism to a position ("B → arm 90°") (Part A). **Proposed:** presets are
  named data in the robot file ("arm: score = 90°"), which autons use too (§3.4).
- **Preset cycling** — one button steps through positions (low, mid, high).
- **Stick control** — an arm on a stick, holding wherever it is let go, clamped by the soft stop (§3.1).
  **Proposed:** any axis the drive does not use; the build refuses an axis the drive uses. On robot two's tank
  the free axes are left X and right Y; arcade uses left Y and right X. Which axis drives which mechanism is
  the team's control scheme.
- **Long-press and double-tap** as separate actions. **Proposed:** their timings are starting values in the
  driver file's FEEL block, tuned by the drivers, never a fixed library number. The cost is stated for the
  drivers: a button with a double-tap must wait the double-tap window before its single tap acts, and a
  button with a long-press cannot act on the press itself. Which gestures may share a button is §5 Q8.
- **Arbitration** when several controls act on one mechanism (an arm with a stick, presets and a cycle button)
  is §5 Q8. **Proposed:** the most recent input wins. The stick overrides a running preset the moment it
  leaves its deadband, and a new preset overrides a stick hold.
- **Rumble** when a piece is captured or a motor overheats.
- **The controller screen** shows mechanism state: an arm's angle, pieces held, a temperature warning.
- **The controller's output budget** (a requirement, from PROS's own notes and the tree):
  - All three rows are already used by the fault display in Drive and Teleop. Which rows show mechanism state,
    on which controller, is §5 Q10.
  - PROS drops text AND rumble updates faster than 50 ms apart over VEXnet (10 ms wired). The fault display's
    row 0 alone already writes ~10 times a second.
  - So there is **one controller-output arbiter per controller**: text and rumble share one budget of at
    least 50 ms between writes, and a rumble pattern is at most 8 characters of `.`, `-` and space.
  - The column count is unresolved (19 believed, 15 in the PROS docs, HA-107). It is measured at a robot
    session before any mechanism row is designed.
  - Whether mechanisms and their state can live on the **partner controller** is §5 Q9. VEX U has two
    drivers, and `RobotTwoDriver` binds the master only.
- **Macros — two kinds, two rules:**
  - **Chassis macros** (a turn to face the goal, a short drive): any stick input takes control back within one
    tick, and releasing the button cancels. This is T3's non-negotiable and ruling R1, which applies the
    stick rule to chassis macros only.
  - **Mechanism macros** ("score" = lift to height, wait, open the claw, lower): NOT stopped by the drive
    stick, because you drive to the goal while the lift rises (T3 R1). What stops one is §5 Q7.
  - **Proposed:** a macro is written as data: a named list of the same eager mechanism steps autons use
    (move, run-until, set, wait), stored in the robot file's controls block and generated into the driver
    file. There is no branching in the first version.
  - This absorbs T3's open "chassis macros and angle presets" (the macros to Part C, the presets to Part A).

### 3.4 Autons

- **Every mechanism step is ONE eager call** carrying its target, its done-condition and its bound. F10 froze
  EAGER execution and rejected a deferred chain, whose worst failure is "a chain built but never run, which
  compiles and does nothing". A step runs the moment it is written.
  - Illustrative, names to be settled in the brief: `r.moveMechanism(arm, 90_deg, 1.2_s)`,
    `r.run(intake, 12_volt, Until::piece(clawSensor), 1.5_s)`, `r.set(claw, Closed)`,
    `r.spinUp(flywheel, 3000_rpm, 2_s)`, `r.home(lift, 1_s)`.
  - The earlier `r.run("intake", 12_volt).until(piece).within(1.5_s)` is withdrawn: `run()` could not act
    until `within()` arrived.
  - New steps are F10-additive (a minor API bump), and new stop causes are appended to `RoutineStopCause`.
  - The steps take the mechanism as an argument, so `Routine`'s frozen constructor (a `Chassis&` only) is
    untouched.
  - Until the steps land, `then()` (unfrozen) already runs a mechanism operation inside a routine.
- **How an auton reaches its mechanisms:** `AutonContext` gains a mechanism lookup. It is additive to COMP-a's
  struct, which is not frozen. It lands after COMP-t and COMP-b have shipped the students' auton file, so the
  autons written before it keep compiling and gain the lookup. Today's `AutonContext` holds the mechanisms for
  the guard's safing only, and its own comment says a routine looks mechanisms up "from the Robot", which
  nothing hands it.
- **Names checked at build or at run** — generated typed handles (`mech::arm`; a renamed or removed mechanism
  fails the build) or strings (`"arm"`; a missing name ends the step with an ABSENT verdict at run time) — is
  §5 Q18.
- **Run while driving** (Part A, from the service): a START step is eager too. It starts the action and
  returns, its bound still applies, the next `driveTo` runs while the service ticks it, and the auton can wait
  on it later with its own bound.
- **Wait on sensors** — "drive forward until the claw has a piece, at most 30 in". **Proposed:** a step built
  on the scheduler's existing async + `waitUntil` (Tier 3) with the drive's own distance and time bounds. That
  needs no F6 change; the step is F10-additive.
- **Triggers partway through a drive** — "at 60 % of this drive, raise the lift". **Corrected premise:** route
  markers fire only AFTER a leg settles, never partway, and a timed-out leg fires none (G2 ruling R2). So
  this is NEW for routes AND for plain routines.
  - It needs a progress signal from the motion: the fraction of the leg's starting distance covered. It comes
    either from Tier 3 (async + `waitUntil` on distance travelled) or from a new `MotionOptions` field (F6's
    field set is additive-open).
  - For routes it needs a new optional marker field. That is F8-additive but a cross-team change for
    VexBuilder's picker.
  - Whether routes are in scope, and what happens when the leg times out or is cancelled before 60 %, are §5
    Q19. **Proposed:** it never fires in that case, the R2 precedent ("a marker that fires at the wrong place
    is ... strictly worse than a marker that does not fire at all").
- **Robot two's claws in autons** — the first proving case. A claw with no sensor can only be confirmed by
  TIME, and the plan's F3 rule says "never advance on an unconfirmed action" (§5 Q2).

### 3.5 Port binding (P2)

- **Roles, not ports**, in every definition; the binding is separate: role → smart port or 3-wire line.
  Where the binding lives is §5 Q1; three requirements depend on the answer (§3.9).
- **Checked at every boot, on the SMART ports**, against the brain's device list (the registry-checked device
  provider):
  - the right KIND on every bound smart port (motor, rotation, distance, optical, IMU);
  - no port bound twice, and no mechanism on a drive port (unless §5 Q4 allows a declared-shared port);
  - a missing device is ABSENT: its buttons do nothing and the drive still runs;
  - every refusal names the role and the port.
- **The 3-wire lines (A–H) cannot be checked.** The brain's registry cannot see what is plugged into them
  (HA-158; the device provider does not ask). A limit switch or a solenoid gets no kind check and no presence
  check. **Their only confirmation is the bench page and the direction check (§3.6)**, and the boot line says
  "not checkable" beside each one.
- **The provider's missing accessors are Part A work:** `distance(port)`, `optical(port)` and `digitalIn(line)`
  on the device provider, answered by every implementer (the fake and the PROS one). Rotation is provided
  today.
- **A boot line lists every role's port**, so a swapped cable is visible in the log.
- **The dangerous case** — two cables of the SAME kind swapped between mechanisms: nothing looks wrong at boot.
  The direction check and guided binding (§3.6) are how that is caught.
- **3-wire expanders** (more than eight lines): whether the first version supports them is §5 Q5.

### 3.6 Setup and diagnostics

- **A mechanism page in Bench Tests** — jog each mechanism and see its encoder, current, temperature and limit
  switches live; fire each solenoid line on the operator's confirm. It is the way to confirm wiring after
  re-cabling, and the only confirmation a 3-wire line has.
- **Direction check** — "press R1: did the arm go up?"; if not, one tap flips that role's direction. Where the
  flip is saved depends on §5 Q1 (§3.9).
- **Guided port binding on the brain** — "wiggle the arm": the program sees which port moved and binds it;
  nothing is typed. What it can and cannot detect:
  - **Smart motors and smart sensors:** yes. Turning a motor or a rotation sensor by hand moves its encoder,
    and a hand in front of a distance or optical sensor changes its reading. Nothing is powered, so it works
    while disabled.
  - **3-wire inputs (a limit switch):** yes — press it, and the line that changed is the one.
  - **3-wire outputs (a solenoid): no.** A solenoid gives nothing to detect, and binding one by trial means
    firing unknown lines, which could fire the wrong cylinder. **Proposed:** solenoid lines are bound only in
    the robot file (the panel or the compiled map). The bench page fires a bound line only on the operator's
    confirm, with the robot safe. Under §5 Q1 (b), where the robot file carries no ports, a solenoid role is
    bound on the brain by the operator choosing its letter from a list, never by firing lines to find it.
  - Guided binding only makes sense if the binding can be saved on the brain (§5 Q1 (b) or (c)).
- **Auto-tuning** — the robot measures an arm's gains itself, holding at a few angles. **Proposed:** it
  measures the gravity term, the static friction and the velocity term. Holding at three or more angles also
  fits the horizontal angle, so "horizontal" is measured rather than typed. Starting values for the position
  loop follow from those. Where the results are stored, and whether it waits for R5a (the same idea for the
  drive), is §5 Q24.
- **A health log per match** — cycles, stalls and peak temperatures per mechanism, to the SD card. Whether it
  is the same file as T4 Part 3's SD copy of the log is §5 Q23.
- **Mechanisms in the host test plant** — arm, lift and flywheel models, so an auton that uses them is tested
  on the laptop before the robot. **The arm and lift models are Part A**: position control cannot be tested
  adversarially without them. The flywheel model comes with speed control (Part B).

### 3.7 PTO-ready (P7)

No PTO support is built (§0.5). What "ready" means is his choice (§5 Q4):
- **(a) Ready now.**
  - A port may be DECLARED shared by the drive and one mechanism, and exactly one owner commands it at a time.
  - The motor group can hand a member over AND take it back, and it stops commanding a member it does not
    own. That is unlike `markAbsent`, which is permanent and keeps commanding.
  - The validator, the generator and the VexBuilder contract accept the declared-shared port.
  - A later "shift" step (drive ↔ mechanism, only when both are stopped) is then one added piece.
  - Cost: ~1½–2½ days in Part A.
- **(b) A named later change.** Nothing is built now. A PTO later changes the three rules listed in P7 and adds
  the hand-over to the motor group. The design keeps those four places known and nothing else depends on
  "a port is never shared".

### 3.8 What stays out

- Season strategy (the plan's F3/F4 verbs — `setQuadrantToggle`, `buildStack`, `matchLoadCycle`,
  `strategyMode` …) is the students' code built from these pieces; the library may ship them as EXAMPLES.
- **F4's generic items are placed, not dropped (proposed; his go with §7.3):**
  - `fieldCentricStrafe` / `strafeTrim` (H-bot) go to the holonomic track: R3c on the spring X-drive and I1's
    H-drive hardening.
  - `moveToPoseProfiled` (lift-state-aware acceleration) is absorbed by tip protection (§3.2).
  - `endInMidfield`'s 18″ height lockout becomes an interlock (§3.2) plus the students' end action.
- The sensors ruled for after G3 (2026-09-28): the AI Vision adapter, the distance-sensor wall reset, driving
  to a piece. **To confirm with the team lead (§5 Q3):** the sensors a MECHANISM needs (a limit switch, a
  rotation sensor, a distance or optical sensor as a piece sensor or colour sorter) come with the mechanisms,
  not after G3.

### 3.9 What depends on where bindings live (§5 Q1)

Three requirements cannot be stated without Q1's answer. Each one's behaviour under each option:

| Requirement | (a) compiled wiring map | (b) SD record written on the brain | (c) both — the record overrides the compiled default |
|---|---|---|---|
| The panel's wiring page: a role "left unassigned to bind on the brain" (§4.3) | not offered: every role needs a port in the robot file, or the build refuses it | the robot file carries no ports; the page shows the last record read from a card, read-only | offered: an unassigned role is bound on the brain; the page sets the compiled default and shows the card's record read-only, warning where they disagree |
| The direction check's flip "and saves" (§3.6) | cannot be saved on the brain: the screen shows the one change to make in the robot file, then a rebuild | written into the record with SAFE-SAVE's safe save | written into the record, which overrides the compiled direction. The calibration record's hazards apply to flips too — a stale flip after a motor is re-mounted, another robot's card — so the record is bound to the robot's identity (the drivetrain record's CRC, as R5a does) and the boot line marks each flipped role |
| The export's "mechanisms and wiring" (§4.8) | roles and their compiled ports | roles only (no wiring exists off the brain) | roles and the compiled defaults, noted as "a card may override" |

## 4. The robot panel — the first version (everything below is IN)

The panel is the optional VS Code extension over the robot file (G3a). Nothing needs it; everything it does,
the robot file and the build can do without it. Its job: making a robot is super simple, with nothing typed
in C++.

1. **New robot** — name, drivetrain kind, motor cartridge. *(The front is NOT asked — the team lead, 2026-10-06: "itll be
   hard to say which way is the front through the panel ... maybe we just get it from the calibrate?" The end that LEADS
   Calibrate's push is the front, carried by the calibration record, with a warning when a recalibration flips it — CAL1,
   the standard Calibrate.)* Everything measurable
   (track width, wheel size, motor directions) is measured by Calibrate and SHOWN, never typed. G3a's brief
   generates a TANK only (the X-drive is a C++ config), so which kinds the first version offers is §5 Q22.
   It creates the robot's build too, the gap found 2026-09-28 in G3a's brief as written. It writes these
   files, in MARKED REGIONS only (G3a's D2). D2 plans regions in the two table files and the driver file only;
   the `Makefile`, `build_selection.hpp` and gate regions are new:
   - the robot file;
   - the `Makefile`'s `ROBOT=` value and its build switch;
   - `src/build_selection.hpp`'s one chain: the robot's `RobotId` and its beacon through `robotBeacon()`;
   - the robot's two tables, as new named tables in `src/robots/chassis_table.hpp` and
     `src/robots/mechanism_table.hpp`. They go in those two files and no third, because the `src` gate's
     `TABLE_FILES` reads exactly those two;
   - the `src` gate's lists in `tools/src_build_gate.py`;
   - a starter driver file `src/team/<robot>_driver.hpp` with its WHICH BUTTON region.
2. **Mechanisms** — add, remove, duplicate, reorder (the order is the boot line's and the panel's).
   - **Add** asks for a template and a name. The template fills the settings below; every number the team must
     own is left EMPTY and refused until stated (P6), never invented:

     | Template | Kind | Safe state | Filled | Left for the team |
     |---|---|---|---|---|
     | Intake | power | Coast | jam auto-reverse on; hold-to-run in/out | cartridge; stall thresholds; an optional piece-sensor role |
     | Arm | position (degrees) | Hold | gravity term (cosine); homing by stall; smooth moves | cartridge; gearing; travel limits; presets; the horizontal angle (or auto-tune it) |
     | Lift | position (inches) | Hold | gravity term (constant); homing by switch if a switch role is present, else stall | cartridge; spool diameter or geometry/table; travel limits; tip-protection height |
     | Claw | pneumatic | — | a toggle control | open level, start level, safe level (no default, today's rule); an optional piece sensor (§5 Q2) |
     | Flywheel | speed | Coast | "at speed" done-condition; a toggle control | cartridge; target RPM; the at-speed band and time |
     | Catapult | cycle | stated by the team | wind until loaded, fire, re-wind | cartridge; the loaded sensor role; slip gear or latch (§5 Q12); the safe state (a loaded catapult's depends on its mechanism) |
     | Conveyor | power | Coast | piece counting at an entry sensor; jam auto-reverse on | cartridge; the entry (and optional exit) sensor roles; the debounce time |
     | Blank | chosen by the team | stated by the team | nothing | everything |

     The team changes what differs.
   - **Remove** also removes that mechanism's buttons, interlocks, presets and the macro steps that name it; the
     panel lists them before it saves. A student's C++ auton that names the mechanism is not the panel's to
     edit:
     - under typed handles (§5 Q18 (a)), the build fails and names each auton that used it, and the panel
       warns before saving by searching `src/team/`;
     - under strings (§5 Q18 (b)), the auton's step ends ABSENT at run time, and the panel's warning is the
       only early notice.
   - Every setting of §3.1–3.2 is editable, with the reason shown for any value the build would refuse.
3. **Wiring** — each role gets a port. Whether it can instead be "left unassigned to bind on the brain"
   depends on §5 Q1 (§3.9). Live warnings: a port used twice, a drive port, the wrong kind of device on a smart
   port (from the last record), a role left unbound. 3-wire lines are marked "not checkable on the brain".
4. **Controls** — buttons per mechanism: hold, toggle, preset, preset cycling, stick, long-press, double-tap,
   macros. Live warnings: the same button and gesture used twice, two controls of one kind on a mechanism, the
   re-zero button, a stick axis the drive uses, a control for a mechanism that no longer exists, and a
   mechanism with no control that is not marked auton-only.
5. **Calibration** — the SD card's calibration record shown read-only (the measured drive: ports, directions,
   track width, scale), with a warning when it disagrees with the robot file (for robot two today: the record's
   front is the camera end, the typed table's the 15/16 end).
6. **Save** — writes the robot file and generates the C++ (the tables, the driver file's button block); if the
   build would reject something, the panel names it before saving. One validator: the panel never has its own
   copy of a rule.
7. **Import a `.vexbot`** — for a team that designs in VexBuilder first (G3). Today's `.vexbot` (v2.0.0)
   carries empty electrical arrays and no robot profile, so an import fills only what is there until VexBuilder
   writes the profile (the integration doc's asks #5 and #2). A field the file lacks is taken from the robot
   file, never defaulted (§4.10).
7b. **Routes, drawn in the panel — NEW 2026-10-08, the team lead's ruling** ("i honestly think that it should be able to and
   then vex not only CAN but can just take in that route that was done in the panel and add physics sim to it"): a field
   view where a team draws a route (waypoints, headings, reversed legs) and drops MECHANISM STEPS on it, picked from the
   robot's own route ids (below); it saves in the shared, frozen route format (F8, the `paths[]` VexBuilder reads), so
   VexBuilder can take the same route and add its physics simulation. One validator, as everywhere: a marker naming a
   route id the robot does not have, or a malformed leg, is refused before saving. **No simulation in the panel**
   (the team lead, 2026-10-08, reversing the same day's preview: "it can be misconstrued between that and vex and for them
   to be accurate they basically have to be both the same ... simulating it in the panel is not good and should be kept to
   vexbuilder") — VexBuilder is the ONE place a route is simulated; shulib's host test plant stays inside the library's
   tests.
   **Placement (ruled the same day):** its own panel part RIGHT AFTER the mechanism pages (G3a Part 6), before the
   panel's extension for the rest; ~4–6 builder-days (rough).
   **Mechanism route ids (ruled 2026-10-08, built in the MECH core):** every mechanism publishes its actions as route
   ids AUTOMATICALLY (an arm's presets as `arm_stow`, `arm_score`; an intake's `intake_run`, `intake_until_piece`), in
   shulib's manifest that VexBuilder's marker picker reads — a coder no longer registers each id by hand.
8. **Export a starter `.vexbot`** — the robot profile, its mechanisms and (per §3.9) its wiring, an empty path
   list, and shulib's command list (VexBuilder's command picker can already read it, ask #4), so a team can
   start in the panel and continue in VexBuilder. It needs two NEW asks of VexBuilder: open a project shulib
   started, and keep unknown profile fields (§4.10).
9. **Later, in the same extension:** the robot console (H4 — the brain's screen and log on the computer,
   remote touches).
10. **The robot file against VexBuilder's schema and G3a's brief** (P2, P8):
    - The published contract (`vexbuilder-integration.md` §3.1) and G3a's brief both put ports INSIDE each
      mechanism (`motors: [{port, reversed}]`). G3a's brief also keeps the drive's ports typed "as wired".
    - P2 needs **a revision of the contract's §3.1**: roles in the mechanism (`{role, reversed}`) and a separate
      binding block (role → port). VexBuilder is building against the contract, so it gets a note. Before G3
      freezes F7 the change costs little; after, it costs a `schemaVersion` migration.
    - P8 makes every MECH field (kinds, limits, presets, interlocks, macros, controls) part of the profile, so a
      panel → VexBuilder → import round trip could silently drop mechanism settings. **A NEW ask:** VexBuilder
      keeps `robotProfile` fields it does not understand, byte for byte (§5 Q20).
    - Whether the drive's ports stay typed in the robot file is §5 Q21.
    - G3a's brief (its robot-file table, the byte-for-byte differential) is amended to the roles-plus-binding
      shape.

## 5. Open questions (the team lead's)

> **THE TEAM LEAD'S ANSWERS (2026-09-29, ~03:45–03:55)** — the core's questions, asked after his approval of
> speedup C (2026-09-28 ~17:35), and his word that **the list is complete: "It's complete, write the brief"**.
> - **Q1 — where bindings live: (a) ONLY the robot file** (edit the wiring, rebuild, upload). Consequences, carried
>   into the brief: no SD binding record; the direction check on the brain REPORTS a wrong direction and names the
>   robot-file line to change (it cannot save a flip); guided binding (in the rest) becomes "tell me which port
>   moved", the port then written into the robot file; the panel's wiring page has no "bind on the brain" option.
> - **Q2 — a sensorless claw grab: (c) time now, a sensor later** — counted done after a set time and reported as
>   `Unconfirmed`; a distance sensor confirms it once mounted.
> - **Q3 — mechanism sensors come WITH the mechanisms** (limit switch, rotation, distance/optical as a piece sensor;
>   the provider accessors in the core). The after-G3 ruling covers the AI Vision adapter, the wall reset and piece
>   targeting only.
> - **Q4 — PTO-ready: (b) name the later change** — nothing extra built; the plan records the three rules a PTO
>   would change (§3.7).
> - **Q5 — 3-wire expanders: (a) not in the first version.**
> - **Q6 — homing for robot two's pivots: not sure yet** — decided once the pivots gain positions; the library
>   supports every homing moment (§3.1).
> - **Q8 — several controls on one mechanism: (a) the most recent input wins**; a mechanism may be marked auton-only
>   (the gestures sub-question follows the recommendation: short + long on one button, a double-tap only on a button
>   with no hold-to-run — in the rest, with MECH C).
> - **Q13 — three-position pneumatics: (a) the team states each position's levels and the safe position.**
> - **Q18 — names in autons: (a) generated typed handles, checked at build.**
> - **Q25 — sequence against T4 Part 3:** answered by speedup C (2026-09-28 ~17:35): the core's brief is written
>   while T4 builds and its LIBRARY half builds alongside T4 Part 3 (a stated file split; its `src/` half waits).
> - **Q28 — the F3/F4 re-scope: YES** (the drafted deviations row and dependency #6 amendment, §7.3).
> - **Q29 — anything missing: the list is complete.**
> - **Q30 — the bench page and the direction check: INTO THE CORE** (+~1½–2½ days).
> - **Not asked, carried to the brief as recommended for his go:** Q26 (MECH on hardware as the F11–F14 freeze
>   trigger). The rest's questions (Q7, Q9–Q12, Q14–Q17, Q19–Q24, Q27) wait for the rest (after the panel).
> - **The front (asked for the panel):** "the front will be different depending on the robot" — the design already
>   makes the front a per-robot, human-stated setting (never a library default). Robot two's own front — its
>   calibration record and how the team drives use the camera end; its typed table says the 15/16 end — is a
>   robot-two setting to confirm with him at the panel brief.


Each has its options and, where the evidence supports one, a recommendation. None is decided here.

### Scope and hardware

1. **Where port bindings live** (§3.5, §3.9): (a) a compiled wiring map (edit one line, rebuild, upload);
   (b) a binding record on the SD card written ON THE BRAIN by a guided step, no laptop at an event; (c) both,
   the record overriding the compiled default. The drive's calibration record vs its typed table is the
   precedent for (c), and it has hazards: a stale record, and a record from another robot. The design must
   refuse both. **Recommend (c):** guided binding, which is on his accepted list, only makes sense under (b)
   or (c), and (b) loses the build's port checks and the panel's wiring page. Cost: ~1–2 days in Part D for
   the record and its refusals, after SAFE-SAVE's Part A.
2. **A claw grab with no sensor** (§3.4): (a) confirmed by TIME, with an honest `Unconfirmed` verdict (the auton
   decides whether that stops it — F3's "never advance on an unconfirmed action" relaxed for a sensorless
   mechanism); (b) a distance sensor on the claw (build-team hardware); (c) (a) now, (b) when a sensor is
   mounted. **Recommend (c):** `ActuateAndConfirm` already tells `Succeeded` from `Unconfirmed`, so a
   time-only grab can say honestly that it was not confirmed.
3. **Mechanism sensors with the mechanisms, not after G3** (§3.8): (a) with MECH (the limit switch, rotation,
   distance and optical as piece sensor or colour sorter; the provider accessors are Part A); (b) after G3 with
   the rest. **Recommend (a):** homing, counting, colour sorting and the sensor done-conditions cannot be built
   or tested without them. The after-G3 ruling named the AI Vision adapter, the wall reset and piece targeting
   only.
4. **What "PTO-ready" means** (P7, §3.7): (a) built now: a declared shared drive+mechanism port, one owner at a
   time, a motor group that hands a member over and back (~1½–2½ days); (b) a named later change to three
   rules plus the motor group. **Recommend (b)** unless the build team plans a PTO this season: the team has
   not used one, and building the hand-over before a PTO exists designs against a guess (the
   build → consumer → freeze lesson of F6/F10/F11).
5. **3-wire expanders** (§3.5): (a) not in the first version: the brain's eight lines only, as the provider
   is today; (b) supported: a line becomes (expander port, line). **Recommend (a):** robot two uses one line of
   eight, and (b) is additive later.
6. **Homing in a match, for robot two's pivots** (§3.1): (i) at boot without motion (the pivot declared to
   rest on its hard stop at boot); (ii) a stall-home at the start of autonomous (costs match time); (iii) at
   driver-control start; (iv) on request only; (v) a rotation sensor with a stored offset, so no homing at all.
   The library supports every one. **No recommendation:** it depends on where the pivots rest at boot, which
   only the team knows, and it matters only once they gain positions (they are power-only today).

### Driver control

7. **What stops a mechanism macro** (§3.3), given T3 R1 keeps the drive stick for chassis macros only: (a) a
   second press of its button; (b) any control of a mechanism the macro moves; (c) a dedicated cancel button;
   (d) releasing its button (a hold-to-run macro); (e) the drive stick, only for a macro that also drives.
   **Recommend (a) + (b):** never the drive stick, since you drive while the lift rises. (c) optional.
   ("Or button" in the earlier draft was not his; it is dropped unless he adds it.)
8. **Several controls on one mechanism** (P5, §3.3):
   - **Arbitration:** (a) the most recent input wins (the stick overrides a running preset once past its
     deadband; a preset overrides a stick hold); or (b) a fixed priority (stick > preset > cycle).
     **Recommend (a).**
   - **Gestures:** may one button carry a short press, a long-press and a double-tap? **Recommend:** short +
     long on one button; a double-tap only on a button with no hold-to-run, because of the delay it forces on
     the single tap.
   - **Auton-only mechanisms:** may a mechanism have no driver control? **Recommend: yes, when marked
     auton-only**, so today's check keeps catching a forgotten control.
9. **The partner controller** (§3.3): (a) the master only, as today; (b) mechanisms on the partner only;
   (c) each control names its controller, default master, and a mechanism's state shows on the controller
   that holds its controls. **Recommend (c):** VEX U has two drivers, and the seam already builds a partner
   controller.
10. **The controller rows** (§3.3): (a) faults keep all three rows, and mechanism state shows only when no
    fault is latched; (b) one row for mechanism state and two for faults; (c) mechanism state only on the
    partner controller. **Recommend (b):** T3 R3 already merged two fault rows into one on the brain's panel to
    free a row for the claws. HA-107 (the column count) is measured first.

### Kinds and protection

11. **Colour sorting** (§3.2): where "our colour" comes from: (a) an alliance choice added to the auton menu
    and carried into driver control; (b) a driver button that flips it; (c) both, the menu setting it and the
    button overriding. **Recommend (c).** Reject methods: **recommend all three** (reverse, a pneumatic eject, a
    flick) as settings of one control, since they share one shape: an action for a time after a delay.
12. **Cycle mechanisms** (§3.1): (a) slip gear (motor only); (b) a latch (pneumatic or motor); (c) both. And a
    pneumatic latch with a motor wind: (i) allow one mechanism to mix a motor and a pneumatic (changes
    `valid()` and the contract's "never both"); (ii) a cycle control that spans two mechanisms by name.
    **Recommend (c) with (ii):** it keeps "one mechanism is motors OR pneumatics" and changes no schema rule.
13. **Three-position pneumatics** (§3.1): (a) the team states each position's line levels and the safe position
    per mechanism, with no default; (b) one fixed library table. **Recommend (a):** only the team knows the
    plumbing (the `pneumaticSafe` precedent).
14. **Tip protection** (§3.2): (a) driver control only; (b) autons too (a `MotionOptions` field, F6-additive).
    And pitch: (i) logged only; (ii) a trigger that cuts acceleration further. **Recommend (b) with (i)**, driven
    by lift height, until HA-110 (pitch's sign) is measured; then (ii) as a later step.
15. **Current caps and the low-power hold** (§3.1, §3.2): (a) firmware: the per-motor current limit through a
    new seam beside the LOCKED `IMotor`; (b) software, ticked by the service; (c) both, firmware as the floor.
    **Recommend (a):** only the firmware limit holds with nothing ticking, which the synchronous safe-state rule
    in `mechanism.hpp` requires. (c) if a mechanism later needs a finer hold.
16. **May a gripping claw hold at a capped current** (P4)? (a) yes, at its stated cap, with the heat warning
    watching it; (b) no, it must drop to a lower hold after a set time. **Recommend (a)**; it is also how robot
    two's Hold-on-release pivots already behave, at the firmware's default limit of 2500 mA (how well Hold
    keeps a loaded claw up is unmeasured, HA-156).
17. **Interlocks** (§3.2):
    - **On a violation:** (a) refuse the new command, logged, with the auton step ending INTERLOCKED; (b) stop
      the other mechanism; (c) wait until clear, bounded. **Recommend (a).**
    - **A driver override:** none in the first version, or a hold-to-override button? **Recommend none:** it
      is cheap to add later and unsafe to take away.
    - **An interlock naming an ABSENT mechanism:** treated as violated (refuse, plus a boot warning naming the
      interlock) or as satisfied? **Recommend violated.**

### Autons

18. **Mechanism names in autons** (§3.4): (a) generated typed handles — a renamed or removed mechanism fails
    the build; (b) strings — a missing name ends the step ABSENT at run time, and a typo is found on the robot;
    (c) both. **Recommend (a):** it catches the mistake before the robot and gives the panel's Remove a build
    error naming each auton.
19. **Triggers partway through a drive** (§3.4): (a) plain routines only now, routes with G3 and VexBuilder;
    (b) routes too, now (a new optional marker field, a cross-team change). And a leg that times out or is
    cancelled before its trigger: (i) never fires; (ii) fires at the end. **Recommend (a) with (i)**, the
    R2 precedent.

### The robot file, VexBuilder and the panel

20. **Ask VexBuilder to keep shulib-only profile fields byte for byte** (§4.10): (a) yes, sent with the §3.1
    revision; (b) no — export is one-way, and an import re-reads only the fields VexBuilder owns. **Recommend
    (a)**, with (b)'s rule as the safety net until VexBuilder confirms: a field missing from an imported file is
    taken from the robot file, never defaulted.
21. **Do the drive's ports stay typed in the robot file** (P2, §4.10)? (a) yes, as G3a's brief has them (the
    fallback before a record exists; the panel warns on disagreement); (b) no — the record is the only source,
    and a robot with no record does not drive until Calibrate runs; (c) optional. **Recommend (a) for the
    first version:** G3a's brief and its byte-for-byte differential rest on the typed table. The disagreement
    on robot two's front shows the cost of two sources, which the panel's warning covers.
22. **Drivetrain kinds the new-robot page offers** (§4.1): (a) tank only, as G3a's brief generates; (b) tank and
    X; (c) tank, X and H. **Recommend (b):** an X-drive config exists as C++ to generate from. No H robot
    exists (I1 comes later). (b) adds ~1–1½ days to the generator.

### Diagnostics

23. **The health log's file** (§3.6): (a) the same SD file as T4 Part 3's log copy, as its own line kind; (b) its
    own file. **Recommend (a):** one writer path, and fewer files on a card the brain cannot rename or delete
    from (SAFE-SAVE's finding).
24. **Auto-tuning** (§3.6): stored (a) like R5a (an SD file bound to the robot's record through SAFE-SAVE's safe
    save), or (b) in the robot file (shown in the panel, applied by a rebuild). And (i) after R5a, reusing its
    ramp, fit and storage, or (ii) before it. **Recommend (a) with (i):** one tuning store, not two.

### Plan and freezes

25. **MECH's sequence against T4 Part 3** (§7.2): (a) MECH A starts after T4 Part 3 lands; (b) a stated file
    split: MECH A's library half runs during Part 3, and its `src/` half waits. (b) is parallel running, an
    exception to the one-thing-in-flight rule that needs his explicit go. **Recommend (a):** Part 3 moves the
    buttons out of `mechanism_table.hpp`, the file MECH A rewrites.
26. **The freeze triggers** (§6):
    - F11 (the mechanism seam), F12 (the guard), F13 (driver input) and F14 (digital input) all freeze on
      "MECH on hardware". **Recommend yes.**
    - F12 alone has alternatives: keep "the students' first season routines on a robot", or COMP-b on the
      robot. **Recommend MECH on hardware for F12 too:** the service is composed with the guard and is its
      claimant, so MECH is a real second consumer of it.
    - F14's open question (switch or stall for homing): record it answered "both" (P1)? **Recommend yes.**
27. **G3's scope** (§7.1): (a) the plan of record: import, the SD-card runtime loader, `inferDrivetrain()`,
    `schemaVersion` negotiation with migration, the F7 freeze (~5½–8 days); (b) re-scoped: import,
    `schemaVersion` negotiation and the F7 freeze (~2½–4 days). Under (b), `inferDrivetrain()` is dropped (its
    input, VexBuilder's electrical arrays, is empty, and Calibrate discovers the drive on the robot) and the
    runtime loader is decided with Q1. **Recommend (b).**
28. **The F3/F4 re-scope in the plan** (§3.8, §7.3): his go on the drafted deviations row, the dependency #6
    amendment, and the placement of F4's generic items.
29. **Anything still missing** — the list is open until he says it is complete.
30. **(NEW, 2026-09-28, after the core-first ruling) The bench page and the direction check in the CORE?** §3.5
    makes them the ONLY confirmation for a 3-wire binding and for two same-kind cables swapped between
    mechanisms — and both sat in MECH D, which the ruling puts after the panel, so the core would ship port
    binding with nothing to confirm it on the robot. (a) Move the mechanism bench page and the direction check
    into the core (~1½–2½ days more); guided binding, auto-tuning and the health log stay in the rest.
    (b) Keep them in the rest; until then, a swapped cable shows only in the boot line. **Recommend (a).**

## 6. Surfaces touched (what MECH changes that is frozen, locked or published)

| Surface | Status | What this list needs | How, and its cost |
|---|---|---|---|
| F4 `IMotor` | LOCKED | a per-motor current limit (caps, a firmware low-power hold) | a NEW sibling seam beside it, implemented by `ProsMotor` and `FakeMotor` — the `IDigitalOut` / `IController` / `IDigitalIn` precedent — so F4's row is untouched and there is no F4 bump |
| F4 `IOptical` | LOCKED, read-only | the LED for colour sorting | set in `ProsOptical`'s construction config (the adapter, not the seam); no F4 change |
| F4 `IImu` | LOCKED | pitch for tip protection | none: `pitch()` exists (sign HA-110) |
| The device provider | G1's; not a register row | distance, optical and digital-in devices | three new accessors every implementer answers (Part A) |
| F13 `IController` | unfrozen, input-only | rumble and text for mechanisms | a NEW controller-output seam (the text half exists as `ILineDisplay`), under one arbiter per controller (§3.3) |
| F11 the mechanism seam | unfrozen | three positions; a position reader on `MotorMechanism`; the service as claimant | a new pneumatic kind; a reader (`IMotor` has position); additive, since the seam is open by design |
| F6 `Chassis` | LOCKED; `MotionOptions`' field set additive-open | tip protection in autons; possibly a progress field for mid-drive triggers | new `MotionOptions` fields — additive, a minor API bump |
| F8 routes | LOCKED | mid-leg markers, only if routes are in scope (§5 Q19) | a new optional marker field — additive under F8, but a cross-team change for VexBuilder's picker |
| F10 `Routine` | LOCKED | mechanism steps; new stop causes | new steps and appended `RoutineStopCause` enumerators — additive, a minor bump; the constructor untouched |
| COMP-a `AutonContext` | not frozen | a mechanism lookup | a new member — additive; lands after COMP-t/COMP-b |
| `RobotConfig::valid()`, G3a's generator, the contract §3.2 | code, a brief, published | only if §5 Q4 is (a): a declared-shared drive+mechanism port | an exception in all three |
| The VexBuilder contract §3.1 | published, not frozen | roles plus a binding block; the drive-ports decision | a revision and a note to VexBuilder; a NEW ask to keep unknown fields (§4.10) |
| F7 `robotProfile` | freezes at G3 | — | **freezing F7 at G3 freezes the whole mechanism vocabulary** (kinds, limits, presets, interlocks, macros, controls) with VexBuilder. MECH's vocabulary must be settled before G3; later growth needs a `schemaVersion` migration |
| F11–F14 freeze triggers | open by design | — | today: F11 "F3's scoring primitives, on hardware", F12 "F4's routines", F13 "T3", F14 "F3's homing". The F3/F4 re-scope orphans F11, F12 and F14 (F3/F4 no longer name library pieces), and F13's trigger moves with T3's macros into MECH C. **Proposed: MECH on hardware as the trigger for F11–F14** (F12 because the service is the guard's claimant); F14's switch-or-stall question recorded as answered "both" (§5 Q26) |
| T3's ruling R1 | ruled | — | unchanged: the stick interrupts chassis macros only (§3.3) |

The API version takes minor bumps only (F6, F8 if routes, F10, COMP-a). Nothing here needs a major bump.

## 7. Where it goes in the plan, and how big (proposed; his to confirm)

### 7.1 Sizes — unmeasured ranges, with the reasoning

Each item is priced against the plan's own comparable chunks: T4 Part 2 (the drive's protections inside
`runDriverControl`) ~2–2½ days; SAFE-SAVE ~2–2½; R5a ~3–4 plus robot sessions; G3a's brief Part A 3–3½ (a
~2 000–2 400-line tool) and Part B 2½; the A2 plant was its own chunk; TANK1 Part A ~1¼ for one behaviour. The
earlier 14–21 days left the service, the provider work and the plant models unsized, and its low ends were
below those comparables. The ranges narrow once §5 is answered (Q1, Q4, Q15, Q19, Q22–Q24, Q27 each move them).

| Part | What | Builder-days |
|---|---|---|
| **MECH A — the core** | the mechanism service ~1½–2½; roles + the compiled binding + boot checks + the boot line + the provider's three accessors ~1½–2½; position control (units, non-linear, profile, gravity, hold, both limit rules, every homing moment) ~2½–4; power with done-conditions ~½–1; three-position pneumatics ~½–1; auton steps (eager, the `AutonContext` lookup, stop causes, handles) + run while driving ~1–2; presets ~½; the host test plant's arm and lift models ~1–2 | **~9–15½** (+1½–2½ if §5 Q4 is (a)). Large enough that its brief may split it: A1 = service, binding, provider, power, pneumatics, plant; A2 = position, homing, limits, auton steps, presets |
| **MECH B — kinds and protection** | speed control + the flywheel model ~1½–2½; cycle ~1–1½; piece counting ~½–1; colour sorting ~1–1½; jam reverse ~½–1; interlocks ~1–2; loaded gains ~½; current caps (the sibling seam) + low-power hold + heat ~1½–2½; tip protection ~1–1½ | **~8½–14** |
| **MECH C — driver and auton extras** | preset cycling ~½; stick control ~½–1; long-press and double-tap ~1–1½; the controller-output seam + arbiter + rumble + the mechanism row ~1–1½; macros (data, the two stop rules, arbitration) ~1½–2½; mid-drive triggers for routines ~1–2 | **~5½–9** (+~1 if routes are in scope, §5 Q19) |
| **MECH D — setup and diagnostics** | the bench page ~1–1½; the direction check ~½–1; guided binding ~1½–2½; the SD binding record and its refusals ~1–2 (§5 Q1 (b)/(c) only); auto-tuning ~3–4 + robot sessions; the health log ~½–1½ (low if §5 Q23 is (a)) | **~7½–12½** under (c); ~6½–10½ under (a) |
| **MECH, all four** | | **~30½–51** (was ~14–21) |
| **G3a, amended — the generator** | the brief's Part A 3–3½; growth: the whole mechanism vocabulary with every refusal the C++ checks make and the agreement test ~3–5; new-robot creation (the regions of §4.1) ~1½–2½ | **~7½–11** (+1–1½ for X-drive generation, §5 Q22 (b)) |
| **G3a, amended — the panel** | the brief's Part B 2½; growth: the new-robot page, mechanisms with eight templates, wiring, controls of every kind, every setting with its refusal reason ~3–5 | **~5½–7½** |
| **G3a, amended, together** | | **~13–18½** (+1–1½ with X; was ~9–12) |
| **G3** | §5 Q27: (a) the plan of record ~5½–8; (b) re-scoped ~2½–4 | **~2½–8** (was ~1, which ignored its plan-of-record scope) |
| **Export** | the starter `.vexbot` and its round-trip tests, plus VexBuilder's side | **~1–2** |

### 7.2 Placement

**Ruled 2026-09-28 (~17:15), superseding the placement below where they differ:** the CORE — MECH A (§7.1) plus
speed control and the flywheel model from MECH B — is built after T4 Part 3 and **before the panel (G3a)**; the
REST — the remainder of MECH B, MECH C and MECH D, and the panel's extension for them — **after the panel**, and
(the coordinator's placement, open to his objection) before G3, so F7 locks the whole vocabulary once. Sizes: the
core ~10½–18 days; the rest ~19–32; the panel's extension ~2–3½ (all rough). The §5 questions the CORE needs
answered before its brief: Q1 (where bindings live — the core carries the compiled binding; guided binding and
the SD record are in the rest), Q2, Q3, Q4, Q5, Q6, Q8 (presets beside hold/toggle), Q13, Q18, Q25, Q26, Q28,
Q29, Q30; the others can wait for the rest.

- **MECH A after T4 Part 3 lands** (§5 Q25). T4 Part 3 moves the buttons out of `src/robots/mechanism_table.hpp`
  and makes the driver file their one owner. MECH A rewrites that table into roles and a binding and extends
  the driver file's controls, so running them together collides on files. The alternative is a stated file
  split (MECH A's library half first). That is parallel running, an exception to the one-thing-in-flight rule
  that needs his explicit go. It is not assumed here.
- Then MECH B, C and D, each its own go. **Before G3a**, so the panel and the robot file are designed around
  the mechanism vocabulary. G3a's generator needs at least MECH A–C's vocabulary settled.
- G3a's brief still says its Part A "runs after T4 Part 1b". Part 1b was folded into Part 3 on 2026-09-28, so
  the brief's ordering note must read "after T4 Part 3 and MECH A–C" when it is amended.
- G3 after G3a; export after G3.

### 7.3 The plan changes the F3/F4 re-scope needs (drafts for his go, §5 Q28)

- **A deviations-table row (draft):** "**F3/F4 re-scoped; MECH added** *(2026-09-28, the team lead: "I feel like
  it shouldnt matter what mechanism a robot has? ...")* | F3 the scoring primitives, F4 skills motion + reference
  routines, Phase F′ gated on hardware and the final mechanism decisions | **MECH A–D: the generic mechanism
  library, not gated; F3/F4 = the students' season strategy (examples optional); F4's generic items placed
  (§3.8)** | P1: what a robot has decides its settings, never the library's work"
- **Open dependency #6 (draft amendment):** "Final mechanism / lift / role decisions — Build team — gates the
  season strategy (the students') and each robot's MECH settings, not MECH's library work."
- **The Phase F′ banner** gains one line pointing at this re-scope.
- **The freeze register's F11–F14 triggers**, as §6 and §5 Q26 propose.
