# HANDOFF — everything left to finish shulib (written 2026-09-23)

> **Who this is for:** the next Claude session that picks up shulib, and the team lead (jal) who
> hands it over. **Read it in full before doing anything.** It was written so that nothing has to be
> guessed. Where something is genuinely unknown, it says UNKNOWN and says who decides.
>
> **Authority order when documents disagree:** the git tree (code + `git log`) → `docs/internal/RESUMING.md`
> (the working protocol) → `docs/internal/build-order.md` (the plan and its first bold `Next:` block)
> → `docs/roadmap.md` ("You are here") → this file. If this file is wrong about the tree, the tree wins
> and this file must be corrected in the same commit that notices it.

---

## 0. Read these first, in this order

1. `docs/internal/RESUMING.md` — the chunk loop and the non-negotiable standards. Follow it exactly.
2. `docs/internal/chunks/SESSION-2026-09-18-autonomous.md` — the last working session: every commit,
   every decision taken without the team lead ("Decisions taken without you" #1–#8 plus the evening
   entries), the hardware outcome, the lessons (the stale-binary trap, below).
3. This file.
4. The first bold `**Next:**` block in `docs/internal/build-order.md`.
5. The brief for the next chunk: `docs/internal/chunks/G1-part2-pros-provider-and-main.md`.

---

## 1. The state of the tree at hand-off

- **Repo:** `/home/gonzei/projects/shulib`, branch **`shulib-v2`**. Main branch is `main` (a squash of
  `release/v2` with `docs/internal/` dropped — never measure `main` by commit distance; see
  `build-order.md`). **Nothing from 2026-09-17 onward has been pushed.** Push only when jal asks.
- **HEAD at writing:** `d34bbf9` (plus the commit that adds this file). **Tree clean. Nothing in flight.**
- **Suite:** 1358 test cases / 1,635,266 assertions, 3 skipped, all green (since the
  2026-09-24 build-identity fix; the 1,635,290 recorded before it included six assertions from a
  stale `-dirty` build hash — see `C5-COMPLETED.md` D8's correction).
- **Gates, all green at HEAD:** host suite (the doc gates run inside the test build: coverage,
  freshness, verbatim examples, removability, briefing, staleness); `python3 tools/src_build_gate.py
  self-test && check` (**six** builds: `bench`, `xdrive`, `tank`, `tank-drive`, `tank-library`,
  `tank-calibrate`); the ARM header gate (181 headers); the two CI greps (no `pros/` include outside
  `include/shulib/hal/pros/`; no `shulib/sim/` include outside `sim/`); the C7 removability grep.
- **Side worktrees that exist:** `.claude/worktrees/agent-aef02a3f7fb230a9d` (T2's, already integrated —
  safe to remove with `git worktree remove --force <path>`), `.claude/worktrees/hw-17aa4eb` (a
  detached checkout at `5e4ce65` used to build hardware packages; `/tmp` copies of packages did not
  survive the reboot — rebuild from here or from HEAD).
- **Commits of the last working sessions (newest first):** `d34bbf9` close-out · `ab7f5b1` G1 Part 2
  brief · `225d0ac` master plan spells `registry.on`/`runner.run` · `cf7a943` G2 (runner, registry,
  manifest, **F8 frozen**) · `fe8e270` G1 Part 1 (robot from data) · `d8aab95` T2 library half (driver
  feel) · `5e4ce65` R3d Part 2 (calibrate program, SD record, boot check) · `17aa4eb` R3d Part 1
  (the self-calibrator).

### What that means in plain words

- **The library has NEVER moved a robot.** Every closed-loop claim in the repo is proven on the
  **host test plant** only. "Host test plant" = `include/shulib/sim/` + the test rigs: a laptop-only,
  non-visual kinematic model that knows the robot's true pose so tests can grade routines. It is
  **not** a visual simulator and **not** VexBuilder's planned physics simulator (which does not exist).
  Say "host test plant", never "the simulator", when talking to jal — he found the word confusing.
- **"shulib Drive"** (adapters + open-loop volts from the sticks, no motion stack) **has driven robot
  two once** (jal's report, 2026-09-17, no log). Nothing else has run.
- **The robot's brain has never enumerated on this laptop's USB** (all of 2026-09-18: no kernel USB
  event on any port). Suspected: a charge-only cable, or a laptop USB controller that logged
  "xHC error in resume … Reinit" at 04:54 that morning. jal is getting a known data cable. **Coding on
  the robot starts when it enumerates** (`pros lsusb` shows a V5 System Port and a User Port).

---

## 2. The robots and the season (facts the next session must not re-derive)

- **VEX U, game "Override".** Two robots. **Both fall robots are TANK drives** (jal's ruling,
  2026-09-14). An X-drive is built in spring 2027; all holonomic validation waits for it.
- **Robot two** ("2026 tank chassis"): 5 coupled motors per side on one gear train, 4 wheels per side.
  Ports as last measured (before a REBUILD that made them unverified): LEFT 11 12 13 14 15, RIGHT 20 19
  18 17 16 (back → front); signs LEFT −11 +12 −13 +14 −15 | RIGHT +20 −19 +18 −17 +16; blue cartridges;
  2.75 in wheels; IMU port 2; radio port 1; front = the end where ports 15/16 sit. Port 18 travels ~20 %
  short on a push (HA-130, unexplained). **The drivetrain was REBUILT around 2026-09-18** — trust none
  of these until the census + MOTOR WATCH (or the calibrator) re-measure them. Source of the typed
  values: `src/chassis_table.hpp` (the typed fallback; the calibrator's SD record is preferred).
- **Robot one:** also tank. Its ports are **UNKNOWN** — jal/the build team must state motor + IMU ports;
  the calibrator discovers the rest.
- **Brain:** VEXos 1.1.5, PROS kernel 4.2.2. **Never start a program from the controller's own menu**
  (it arms a pretend field-control match, HA-128) — use the brain's screen (Run) or
  `pros v5 run <slot> <system port>`. USB ports renumber — resolve with `pros lsusb` every time
  (HA-129). Read program output raw: `stty -F <user port> raw -echo; timeout N cat <user port> > file`,
  recover text with `strings`. The screen is resistive (firm presses). Check battery first. An SD
  card (FAT32) in the brain is required for the calibration record and for logs.
- **No tracking wheels / rotation sensors on the fall robots** (jal's decision): odometry is the drive
  motors' own encoders + IMU (`DriveEncoderOdometry`), to be corrected by GPS and/or AprilTags and by a
  wall reset (item A6 below). GPS was named the priority purchase.
- **AI Vision sensors:** jal has two (arriving ~2026-09-23). Recommended mounting (given to jal in
  chat, repeat to the builders if asked): **one per robot, on the chassis (never on a lift or intake
  that moves), front-facing, level, rigid, nothing in its field of view at any mechanism position,
  on a known smart port**; measure its forward offset, sideways offset and height from the robot's
  centre (inches) and whether it is level or tilted. Pieces are detected by the sensor's own onboard
  detection (colour signatures or the season's game-element model) — **pieces do NOT need tags**.
  Tags are only for localization, and only help if the competition field has AprilTags — **UNKNOWN,
  jal must confirm.** Whether the built-in model recognizes Override pieces is **UNKNOWN** until the
  sensor is on the bench.
- **Competition:** jal wants auton "locked in" for a competition the weekend of 2026-09-26/27. His
  words: "we will complete the entire library by then or at least try without rushing." **Do not
  rush verification to hit it; do not promise it.** Report honestly what is and is not proven.

---

## 3. Standing rules (jal's, all in force — `RESUMING.md` "Pace and documentation rules")

1. **One thing in flight.** One chunk, one executor. (He relaxed this ONCE, for the unattended night of
   2026-09-18; it is back in force unless he says otherwise again.)
2. **A brief waits for jal's go.** Write it, commit it, show him its scope + documentation impact list,
   STOP. Never write a brief and launch its executor in the same turn.
3. **Docs move in the same commit as the code, or the commit waits.** The commit body lists documents
   touched AND deliberately not touched, with why.
4. **Prose review is a logged step:** for every changed source file, grep `docs/` and `README.md` for
   its name and concepts, read the hits, fix what is stale, record what was checked.
5. **Status pointers checked before every commit:** `docs/roadmap.md` "You are here" and the FIRST
   bold `**Next:**` block in `build-order.md` must be true for the tree after the commit.
6. **Report, then wait.** After each step say what changed and which docs moved, and stop.
7. **Size chunks so the doc pass fits** (~5 documents); split into parts otherwise.
8. **Never claim a hardware result without the log or jal's report.** Under-claim before over-claim.

Also: **executors are the Agent tool, `subagent_type: general-purpose`, model `opus`,
`run_in_background: true`, told "do not commit" and "create the PROGRESS log first and append as you
go".** Never use a `fable` model for an executor unless jal says yes for that specific run. **Verify
every executor yourself** before committing (all gates + read the load-bearing code + re-run the
load-bearing mutations by hand). **Commit trailer:** use the attribution line the current session's
system reminder gives (it changed when the model changed). Other memory rules: tests must try to
break the code (mutation checks actually run and observed red); document everything; do the thorough
thing, not the easy thing; students author competition routines (the library delivers primitives —
never write the season's routines for them).

### The stale-binary trap (learned three times — do not relearn it)

`cmake --build build/test` runs the doc gates BEFORE linking, and the briefing gate runs the OLD
binary. A red (mutant) binary therefore blocks its own relink, and the next run silently reports the
stale result. **Mutation recipe:** edit → `touch` EVERY test file that includes the edited header
(an `inline constexpr` table leaves stale objects otherwise) → `cmake --build build/test --target
shulib_tests/fast -j"$(nproc)"` → **require `Linking CXX executable shulib_tests` in the output** →
run → observe red → restore → relink → green. Regenerate the briefing (`python3
tools/briefing_status.py generate`) only AFTER the last restore relinks green.

### The coordinator's verification command set

```sh
cmake --build build/test -j"$(nproc)" && ./build/test/shulib_tests | tail -6
python3 tools/src_build_gate.py self-test && python3 tools/src_build_gate.py check
find include/shulib -name '*.hpp' | sed 's|^include/||' | LC_ALL=C sort | awk '{print "#include \""$0"\""}' > /tmp/all.cpp
echo "int main(){return 0;}" >> /tmp/all.cpp
arm-none-eabi-g++ -std=gnu++20 -Wall -Wextra -Wconversion -Wsign-conversion -Wshadow -Werror -Os -mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=softfp -c /tmp/all.cpp -o /dev/null -Iinclude
grep -rnE '#[[:space:]]*include[[:space:]]*[<"]pros/' include/shulib | grep -v '^include/shulib/hal/pros/'   # must print nothing
grep -rnE --exclude-dir=sim '#[[:space:]]*include[[:space:]]*[<"]shulib/sim/' include/shulib                 # must print nothing
python3 tools/api_doc_tool.py check-fresh && python3 tools/briefing_status.py check
grep -rn "internal/\|chunks/\|RESUMING\|build-order" docs/*.md docs/guide docs/cookbook docs/api README.md  # must print nothing
```

---

## 4. What already exists (so nothing is rebuilt by mistake)

Motion core (C1–C5): `IMotion` primitives (`MoveToPose`, `TurnTo`, `StrafeTo`, `HoldPose`,
`DriveBrake`), `MotionScheduler`, the `Chassis` facade (**F6 frozen**), per-motion results and the session
header. Recipe API `chassis/routine.hpp` (**F10 frozen**; `then()` deliberately unfrozen). Cookbook +
generated API reference + doc gates (D3). Localization: `Localizer`, `ComplementaryFusion`, `EkfFusion`,
`GpsCorrector`, `AprilTagCorrector` (synthetic images only), `PilonsOdometry`, `DriveEncoderOdometry`,
the `IOdometry` seam. Diagnostics: `TermSink`, `SdSink` blackbox + reader, fault latch, health monitor,
loop monitor. Mechanisms: `hal/mechanism.hpp` (`IMechanism`, `MotorMechanism`, `PneumaticMechanism`),
`manipulation/mechanism_op.hpp` (`RunUntilConfirmed`, `ActuateAndConfirm`, `StallDetector`) — **seam
only, F11 unfrozen, never on hardware.** Sequencing: `sequence/run_guard.hpp` (`RunGuard`, the guaranteed
end-of-run action; F12 unfrozen). Adapters (`hal/pros/`): motor, IMU, GPS, rotation, distance, optical,
digital in/out, controller, battery, clock, char sink, block sink + source (SD), line display, tick
pacer, device registry — **no AI Vision adapter.** Drivetrain: `hal::MotorGroup` (median reads, ABSENT
members), `hal::DriveGeometry`, `hal::DrivetrainDescription`. Calibration: `calibration/
drivetrain_calibrator.hpp`, `calibration_record.hpp` (180-byte, CRC-32), `boot_check.hpp`. Driver
control: `teleop/stick_mapping.hpp` (raw, used by shulib Drive), `coupled_side_monitor.hpp`,
`drivetrain_degradation.hpp`, and T2's `input_shaping.hpp`, `slew_limiter.hpp`, `drive_modes.hpp`,
`field_centric.hpp`, `driver_control.hpp` (**not wired into any program yet**). Data layer:
`config/robot_config.hpp`, `config/route.hpp` (**F8 frozen**), `hal/device_provider.hpp` + fake,
`robot/robot.hpp` (`robot::validate` + `robot::Robot`), `runner/command_registry.hpp`,
`runner/path_runner.hpp`, `runner/manifest.hpp`.

Programs in `src/` (six builds, `make ROBOT=… PROGRAM=…`): **Bench Tests** (`make ROBOT=tank`, slot 3,
the measuring tool; `src/bench_r3a.cpp`), **shulib Drive** (`PROGRAM=drive`, slot 1,
`src/drive_program.cpp`), **shulib Teleop** (`PROGRAM=library`, slot 2, built from the SD record else the
typed table; the library teleop loop in `src/main.cpp`; **never run**), **shulib Calibrate**
(`PROGRAM=calibrate`, slot 4, `src/calibrate_program.cpp`; **never run**), plus `bench` and `xdrive`
(invented ports, HA-111). Upload: `rm -rf bin .d && make ROBOT=tank PROGRAM=<p> && pros upload --slot N
--name "<name>"` — one program at a time (make is incremental across variants).

---

## 5. EVERYTHING LEFT — the full list

Each item: **what** it is · **why** · **what exists to build on** · **what "done" means** ·
**what it needs** · **documentation it owes** · **rough size**. IDs match `build-order.md` where a
chunk exists; `NEW-` marks work the plan does not yet have an entry for — **add an entry to
`build-order.md` (and the deviations table) in the brief's commit before building it.**

### Part A — library code, done on the laptop (host-proven, build-gated; no robot needed)

**A1. G1 Part 2 (+ T2's wiring): the PROS device provider, the library teleop loop, `main.cpp` as a
dispatcher.** — **BRIEF WRITTEN, WAITING FOR JAL'S GO:** `docs/internal/chunks/G1-part2-pros-provider-and-main.md`
(read it; it is the spec). jal approved the direction ("we haven't tested teleop before so we might as
well rewrite") but has NOT said go.
- What: `hal/pros/device_provider.hpp` (`ProsDeviceProvider`: adapters made on first request by port,
  one `try` each, refusal names the port); dead-port tolerance moved INTO `robot::validate()` as an
  opt-in (`BuildOptions{tolerateDeadPorts}` → absent group members via `teleop::evaluateDegradation`) —
  **without it the rewrite would make Teleop stricter than today's program**; `teleop/teleop_loop.hpp`
  (`runDriverControl`, PROS-free, the disabled predicate injected, `DriveShaper` inside); shulib Teleop
  moved to `src/teleop_program.cpp` built as provider → `bootConfig()` (SD record → typed table →
  refuse) → `robot::Robot` → the loop; the X-drive graph as a config constant (HA-111 label kept);
  `src/main.cpp` ≤ 300 lines, a dispatcher only.
- Rulings already made in the brief: only Teleop changes feel (shulib Drive and the tester keep
  `mapSticks` — they have hardware history); field-centric ships selectable but OFF; the registry
  check and the builder's device pass both stay and name a port once.
- Done: brief §5 DoD; six builds green, beacons unchanged; Teleop still says UNRUN everywhere.
- Docs owed (brief §6): README wiring paragraph, guide ch. 08 (two-line builder path, compiled in
  `test/guide_examples_test.cpp`, quoted verbatim), ch. 10, ch. 14, roadmap rows, build-order, HA-112
  and HA-111 source lines, worksheet panel table if a row changes, briefing.
- Size: ~1 day executor + verification.

**A2. NEW-COMP: the competition program.** Does not exist anywhere in the plan or the tree.
- What: the program uploaded for a match. `initialize()`: boot the robot (IMU calibration, port check
  against the SD record, status screen, refusal by name). A **brain-screen auton selector** (touch
  buttons: e.g. Left / Right / Skills / None; the choice survives until `autonomous()`; defaults to
  None). `autonomous()`: run the chosen routine under `sequence::RunGuard` with the match duration and
  the guaranteed end-of-run action (the park); both a `chassis::Routine` function and a
  `runner::PathRunner` route must be selectable. `opcontrol()`: `runDriverControl` with the driver's
  profile and the bindings (A5). `disabled()`/`competition_initialize()`: state screens. Handoff:
  anything running when a phase ends is cancelled into the safe state.
- Why: every piece exists except the thing that ties them into a match; today's library program
  commands NOTHING in `autonomous()` (deliberately).
- Builds on: A1's provider/loop/`Robot`, `RunGuard`, `Routine`, `PathRunner`, `CommandRegistry`.
- Done: a seventh build (`PROGRAM=competition`, a beacon, the gate updated), the selector and the
  phase logic host-tested with fakes (phase transitions, cancellation, the park firing on a stalled
  loop), routines registered by the STUDENTS' code (the library provides the registration API; it
  ships only an example routine in the cookbook, not season content).
- Docs owed: a guide chapter "The competition program" (written from the code, examples compiled and
  quoted verbatim), README, roadmap (M4's guaranteed-park line), build-order entry + Next, the
  worksheet (a new station for a practice match).
- Size: ~1 day.

**A3. R2: the AI Vision adapter.**
- What: `hal/pros/vision.hpp` implementing `hal::IVision` (object observations) and `hal::ITagSource`
  (tag observations) — the seams are in `include/shulib/hal/vision.hpp` (`TagObservation`,
  `ObjectObservation`, `ITagSource`, `IVision`); conversion helpers `hal/vision_conversion.hpp` already
  exist. Mount pose (forward/left offset, height, tilt) as config (`SensorsConfig` gains a vision entry;
  `RobotConfig` stays additive). Support BOTH detection modes (colour signature and the built-in
  game-element model) since which works is UNKNOWN. Register every belief about the PROS AI Vision API
  (units, frame, latency, sentinel values, the tag family) as HA entries — check the vendored
  `include/pros/` headers for the actual API; if the vendored PROS 4.2.2 has no AI Vision API,
  STOP and report (the sensor may need a newer kernel — that is a platform decision for jal).
- Done: host-tested through a shim extension exactly like the other adapters (`test/pros_shim/`),
  wired into `robot::Robot` when configured (the `AbsentVision`/`AbsentTagSource` path unchanged when
  not), the AprilTag corrector installable from config when a tag map is given.
- Needs on the robot later: C6.
- Docs owed: HA entries, roadmap (M3/R2), build-order, guide ch. 14, the worksheet (a vision station:
  what the screen shows for a piece and a tag).
- Size: ~1 day host; the bench check is C6.

**A4. NEW-TARGET: piece targeting.**
- What: a motion or helper that turns `ObjectObservation`s (bearing, apparent size/distance, class)
  into a drive toward the nearest piece of a chosen class, with a timeout and a "lost it" exit
  reason; an intake-while-approaching composition with the mechanism op. Pieces need no tags.
- Builds on: A3, the motion primitives, F1's mechanism ops.
- Done: host-tested with a fake vision source producing synthetic observations from the plant's truth
  (the same pattern E3 used for tags); the approach settles on the piece within a stated bound.
- Docs owed: cookbook recipe, guide, roadmap, HA entries for the distance-from-box-size belief.
- Size: ~1 day.

**A5. T3: button bindings and driver macros.** Build-order entry exists ("T3 — Button bindings and
driver macros").
- What: bind a button (hold/press/toggle) to a mechanism operation or a chassis action; **stick input
  takes control back within one tick; releasing the button cancels and the mechanism ends in its
  declared safe state** (asserted at the motors). Per-consumer `ButtonEdge` (HA-104).
- The binding mechanism is the library's; which button does what is the team's.
- Docs owed: guide chapter section on driver control, cookbook, roadmap Phase T rows, build-order.
- Size: ~½–1 day.

**A6. NEW-WALLRESET: re-seat the position estimate against a wall or goal.**
- What: a primitive "drive slowly toward a known wall until the distance sensor reads X (or the drive
  stalls against it), then set the estimate's coordinate on that axis from the known wall position
  and the sensor offset." Uses `hal::IDistance` (adapter exists) and the localizer's `setPose` for one
  axis only (never touch heading — IMU-owned).
- Why: with drive-encoder odometry the estimate drifts with every skid; this is the cheapest,
  field-independent correction a tank auton has.
- Done: host-tested on the plant with a synthetic distance channel derived from truth; a routine step
  `resetAgainstWall(...)` usable from `Routine` via `then()` or as a runner built-in (adding a built-in
  id is additive under F8).
- Docs owed: cookbook recipe, guide, HA entries (sensor offset, wall positions), roadmap.
- Size: ~½ day.

**A7. NEW-SELFCAL-GAINS: gains and noise as self-run calibration routines** (R5 and R4's on-robot half,
re-framed as routines inside shulib Calibrate — proposed to jal 2026-09-18, he agreed with "calibrate
absolutely everything it needs to know"; **confirm the framing with him in the brief**).
- What: a voltage-ramp routine per side that fits feedforward kS/kV/kA (replacing HA-45's
  placeholders); a stationary IMU drift/noise capture; loop-timing capture. Persisted in the SD record
  (a record v2 with its version bump and a decoder that still reads v1, or a sibling record — decide in
  the brief) and consumed by the builder as the tuning.
- Done: host-proven on the plant (the fitted gains recover the plant's configured gains); the record
  round-trips; Teleop and the competition program use measured gains when present and say which.
- Docs owed: HA-45/50/51/52 and the noise entries' settle lines, roadmap R4/R5, guide, worksheet station.
- Size: ~1–1.5 days host; the measurement is C2/C3.

**A8. F3: the scoring primitives.** Build-order entry exists. **BLOCKED on B1 (the mechanism decisions).**
- What: intake-until-capture, lift-to-level, clamp-and-confirm, outtake, homing (limit switch or stall —
  jal/builders decide), each a bounded, confirmed `IMechanismOp` over `MotorMechanism`/
  `PneumaticMechanism`, registered with names so routines read `intake.untilCapture(1500_ms)`.
- Done: host-tested against fakes/hostile fakes; then C4 on hardware.
- Docs owed: a mechanism chapter in the guide, cookbook recipes, HA entries for every threshold,
  roadmap M4, F11 freeze decision (F3 is its second consumer).
- Size: ~2 days once decided.

**A9. NEW-ERGONOMICS: the small API additions the readable routine depends on.**
- `driveTo(..., {.reverse = true})` on the facade/recipe — **F6/F10 are frozen**: this must be an
  ADDITIVE option (new field in `MotionOptions` or a new overload), a `kApiMinor` bump, pins updated
  deliberately, a migration note — follow `include/shulib/version.hpp`'s policy exactly.
- A named-mechanism lookup on `robot::Robot` (`robot.mechanism("intake")`) from `MechanismConfig`.
- `Robot::fromCard()`-style one-liner (may be A1's `bootConfig()` exposed; decide in A1/A2).
- Docs owed: the register rows touched, version notes, guide, cookbook.
- Size: ~½ day.

**A10. The remaining plan chunks (lower priority for the season):**
- **G3 — `.vexbot` ingestion + codegen → freeze F7** (entry exists). A host tool `.vexbot` →
  `robot_config.hpp` + routes as `inline constexpr`, optional SD loader, `schemaVersion` negotiation.
  jal said "we aren't really going to talk about the VexBuilder app" — build it only after A1–A9 unless
  he says otherwise. ~1.5 days.
- **G4 — legacy `.shupaths` importer + the ten-minute guide** (closes M5). The importer is small; the
  guide needs VexBuilder's UI, which is not shulib's. ~1 day shulib side.
- **H1 — SHUL/2 telemetry wire → freeze F9** (live streaming to a laptop). ~1 day.
- **T2 finish:** re-point shulib Drive and the tester's DRIVE station at `DriveShaper` AFTER a driver
  has compared feels on hardware (ruling M4 of A1's brief).

### Part B — decisions only jal and the build team can make (code waits on these)

- **B1. The mechanisms on each robot:** intake type, lift type and levels, clamp (yes/no, pneumatic or
  motor), every motor port and sign, every pneumatic port, and how the lift homes (limit switch vs
  stall — owed since 2026-08-13). **Blocks A8, A5's meaning, and every scoring routine.**
- **B2. Sensor placement:** AI Vision per §2 (one per robot, chassis, front, level, offsets measured);
  GPS — bought? where mounted? (lever arm offsets); distance sensors for the wall reset — where, facing
  which way.
- **B3. Robot one's ports** (motors and IMU). The calibrator does the rest.
- **B4. Does the competition field have AprilTags?** If not, vision's match value is piece detection
  only.
- **B5. The auton routines themselves** are the STUDENTS' to author (the guardrail: students must be
  able to write and defend competition code). The next session teaches, reviews and supplies primitives;
  it does not write the season's routines.

### Part C — sessions with the robots, in order (none can be done at a desk; each produces evidence)

Rules for every session: brain on USB via a data cable (`pros lsusb`), SD card in, battery checked, the
program started from the brain screen (never the controller menu), the user port captured raw to a file,
**everything the robot printed recorded in the relevant `-PROGRESS.md` as evidence**, no claim without it.

- **C1. The first drive** (closes R3d and R3b, flips M1 only with evidence). Worksheet **Station G**
  (`docs/internal/chunks/R3a-BENCH-WORKSHEET.md`, line ~597) is the runbook: Bench Tests census + MOTOR
  WATCH on the rebuilt drivetrain → shulib Calibrate wheels-up (expect the yaw-floor refusal — that is
  correct) → Calibrate on the floor (push front-first, answer the prompts, tape push, spin — 3 m clear,
  a second person at the battery) → `SAVED` read back → a second Calibrate run for reproduction → shulib
  Teleop from the record, wheels up then floor → a few inches of `moveTo`. Then write R3b-COMPLETED.md
  and R3d-COMPLETED.md from the log, move the roadmap pointer and the Next block, flip only evidenced
  checkboxes, update `docs/guide/14-what-it-cannot-do-yet.md` ("It has never driven a robot" becomes
  false only when the log shows it drove).
- **C2. Real gains** (A7's ramp on the floor), persisted.
- **C3. Real noise** (IMU at rest, encoders rolling, loop timing, battery sag under stall).
- **C4. Mechanisms on hardware** (after A8), one log per primitive.
- **C5. Robot one** through C1–C3.
- **C6. Vision on the bench** (after A3): what the sensor reports for a piece and a tag, latency, frame
  conventions; settle A3's HA entries.
- **C7. Absolute references on the field:** GPS with the field strip (if bought), tags (if present), the
  wall reset against a real wall.
- **C8. The first field autonomous** — a student-authored routine run through the competition program
  (A2), graded by where the robot ends up; then robot one; then both.
- **C9. Retune** constraints/gains after field runs.

### Part D — back on the laptop after the sessions

- **D1. R6 — back-fit the host test plant** with the measured numbers (R4/R5) and re-run the whole
  suite; every newly failing test is a real defect the invented numbers hid — fix each where it lives.
- **D2. Settle the Hardware Assumptions Register:** every HA entry the sessions touched gets its settle
  line answered with the log reference, or its number moved.

### Part E — later, not this season's critical path

Continuous path following (pure pursuit / blended legs instead of stop-and-settle); the on-brain live
tuner (H3); visual-servo docking (E6); holonomic validation on the spring X-drive (R3c's other half,
I2); the VexBuilder sim seam and replay (H2); the robot-to-robot coordination seam (I2).

---

## 6. DOCUMENTATION — as important as the code (jal, 2026-09-23: "documentation is also incredibly important for when it finishes")

**Per chunk (every item above):** the brief, the live `-PROGRESS.md`, the `-COMPLETED.md` record, the
generated API reference (`python3 tools/api_doc_tool.py generate` — the freshness gate fails otherwise),
`///` on every public member (the coverage gate fails otherwise), guide/cookbook examples COMPILED in a
test file and quoted verbatim (the verbatim gate), the Hardware Assumptions Register for every invented
or reasoned number, the roadmap row/"You are here", the build-order status + Next block, the prose
review logged, and the commit body's touched/not-touched list. None of this is optional.

**When the library is finished, the documentation that must exist for a coder on the team** (check each
exists and was written FROM WHAT ACTUALLY HAPPENED, not from intent):
1. **README.md** — an honest front door: what it is, what is proven on hardware and what is not, the
   two-line robot, a short routine, links.
2. **A "first hour" guide for a new student coder:** clone, build, upload each program, run the
   calibrator, drive, write and run a first routine on the robot. Followed start to finish by someone
   who did not write it (C8's rule; D3's still-open partial).
3. **The competition program chapter** (A2): selector, phases, the park, how routines are registered.
4. **The mechanism chapter** (A8): each primitive, its confirmation and failure modes, homing.
5. **The driver-control chapter** (A1, A5, T2): profiles, modes, field-centric and its fallback,
   bindings.
6. **The calibration chapter:** what the calibrator measures, the record on the card, boot
   verification, what to do when it refuses (each refusal message and its fix).
7. **The vision chapter** (A3, A4): mounting, detection modes, targeting, tags if used.
8. **The accuracy chapter:** odometry, wall reset, GPS/tags, what drift to expect (measured numbers).
9. **A field-session runbook** (the worksheet's stations consolidated for competition days: pre-match
   checks, reading the brain screen, reading a log off the SD card).
10. **Troubleshooting** (`docs/guide/12-when-things-go-wrong.md` updated with every real failure seen).
11. **`docs/guide/14-what-it-cannot-do-yet.md`** — kept exactly true at every commit.
12. **The generated API reference** published (the publish path is written up in
    `docs/internal/docs-publishing.md`; publishing itself is still open).
13. **The Hardware Assumptions Register** with every settled entry citing its log.
14. **A release** through `docs/internal/RELEASING.md` / `tools/release.py` when jal asks (squash to
    `main`, `docs/internal/` dropped — so no public doc may reference `docs/internal/`).

---

## 7. The recommended order for the next session

1. Ask jal for the **go on A1** (show him the brief's scope and doc list; he has read it once). On go,
   launch the executor per §3, verify, commit.
2. The moment the brain enumerates: **C1** — stop other work and do the first drive (worksheet Station
   G). Nothing is worth more.
3. Brief → go → build, one at a time: **A2**, **A3**, **A6**, **A7**, **A5**, **A9**, then **A4**.
   Ask jal for **B1–B4** early so **A8** is not the thing that stalls.
4. Sessions C2, C3, C6 as the robot is available; C4 after A8; C5 for robot one; C7–C9 on the field.
5. D1, D2, then the documentation set in §6 checked item by item; A10 when jal wants it.

**Do not** tell jal the whole library can be finished by the 2026-09-26 weekend. Tell him, with evidence,
what is proven at each step.
