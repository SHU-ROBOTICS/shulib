# HANDOFF — everything left to finish shulib (written 2026-09-23, corrected 2026-09-24)

> **Who this is for:** the next Claude session that picks up shulib, and the team lead (jal) who
> hands it over. **Read it in full before doing anything.** It was written so that nothing has to be
> guessed. Where something is genuinely unknown, it says UNKNOWN and says who decides.
>
> **Authority order when documents disagree:** the git tree (code + `git log`) → `docs/internal/RESUMING.md`
> (the working protocol) → `docs/internal/build-order.md` (the plan and its first bold `Next:` block)
> → `docs/roadmap.md` ("You are here") → this file. If this file is wrong about the tree, the tree wins
> and this file must be corrected in the same commit that notices it.
>
> **Corrected 2026-09-24 by an audit of this file against the tree** (162 findings; the page is in
> the team lead's claude.ai artifacts as "shulib Handoff Audit"). The corrections that change what
> the next session DOES: the first drive can close R3d but **not** R3b or M1 (no program commands a
> `moveTo`); **shulib Teleop refuses without a SAVED calibration record** (the typed table has no
> track width or ratio); the hardware checkout holds **only the shulib Drive package**; the G1 Part 2
> brief had four defects and was revised into three parts before its go; robot one's ports do **not** need typing
> (the calibrator discovers them). **The item IDs were renamed** — they were A1–A10 / B / C1–C9 /
> D1–D2, which collide with the plan's own chunk IDs (build-order's C6 is legacy salvage, not a
> vision session): library work is now **L1–L10**, questions for jal and the builders **Q1–Q7**,
> robot sessions **S1–S9**, laptop work after the sessions **P1–P2**.

---

## 0. Read these first, in this order

1. `docs/internal/RESUMING.md` — the chunk loop and the non-negotiable standards. Follow it exactly.
2. `docs/internal/chunks/SESSION-2026-09-18-autonomous.md` — the last working session: every commit,
   every decision taken without the team lead (decisions #1–#8 are inline in its Timeline; its
   "Decisions taken without you" section now carries an index to them), the hardware outcome, the
   lessons (the stale-binary trap, below). Its end carries a corrections section from the audit.
3. This file.
4. The first bold `**Next:**` block in `docs/internal/build-order.md`.
5. The brief for the next chunk: `docs/internal/chunks/G1-part2-pros-provider-and-main.md` (revised
   2026-09-24 after the audit — read its revision note at the top).

---

## 1. The state of the tree

- **Repo:** `/home/gonzei/projects/shulib`, branch **`shulib-v2`**. `main` is a **single-parent tree
  snapshot** of `release/v2` (made with `git commit-tree`, not a merge or squash), and `release/v2` is
  `shulib-v2` merged in with `docs/internal/` dropped — `docs/internal/RELEASING.md` is the procedure.
  Never measure `main` by commit distance.
- **Pushed:** `origin/shulib-v2` is `6ceb244` (2026-09-17 19:58, "shulib Drive drove robot two").
  **Everything after it is unpushed** — `git log --oneline origin/shulib-v2..HEAD` lists it. Push only
  when jal asks.
- **Suite:** 1358 test cases / 1,635,266 assertions, 3 skipped, all green (since the 2026-09-24
  build-identity fix `05ad50a`; the 1,635,290 recorded before it included six assertions from a stale
  `-dirty` build hash — see `C5-COMPLETED.md` D8's correction).
- **Gates, all green (re-run 2026-09-24):** host suite (the doc gates run inside the test build:
  coverage, freshness, verbatim examples, removability, briefing, staleness); `python3
  tools/src_build_gate.py self-test && check` (**six** builds: `bench`, `xdrive`, `tank`,
  `tank-drive`, `tank-library`, `tank-calibrate`); the ARM header gate (181 headers); the two CI
  greps; `python3 tools/api_doc_tool.py check-removability`.
- **Open chunks** (a `-PROGRESS.md` with no `-COMPLETED.md`): **G1** (Part 1 landed; Part 2 briefed
  in three parts — Part 2a landed 2026-09-24, host-only; 2b and 2c open),
  **R3a** (bench measurements; its DoD's register groups are unsettled), **R3b** (host pieces landed;
  closes on an on-robot auton), **R3d** (Parts 1–2 landed; Part 3 is the bench session), **T2**
  (library half landed; `src/` wiring is in G1 Part 2, and `T2-COMPLETED.md` is owed at the end).
  None is interrupted: the tree is clean. G2 was closed on 2026-09-24 (`G2-COMPLETED.md`).
- **Side worktrees:** `.claude/worktrees/agent-aef02a3f7fb230a9d` (T2's, integrated — remove with
  `git worktree remove --force <path>` and then `git branch -D worktree-agent-aef02a3f7fb230a9d`,
  its leftover branch); `.claude/worktrees/hw-17aa4eb` (detached at `5e4ce65`; **its `bin/` holds ONE
  package — shulib Drive**: each `rm -rf bin .d && make …` overwrote the one before, so Bench Tests,
  Calibrate and Teleop must be rebuilt).
- **Commits since the last push, newest first:** run `git log --oneline origin/shulib-v2..HEAD` —
  this file used to list a selection by hand and dropped four (`d9d0ee3`, `e9ba69f`, `280dce8`,
  `76a5819`) without saying so.

### What that means in plain words

- **The library's motion stack has NEVER driven a robot.** Every closed-loop claim in the repo is
  proven on the **host test plant** only. "Host test plant" = `include/shulib/sim/` + the test rigs: a
  laptop-only, non-visual kinematic model that knows the robot's true pose so tests can grade
  routines. It is **not** a visual simulator and **not** VexBuilder's planned physics simulator
  (which does not exist). Say "host test plant", never "the simulator", to jal.
- **What HAS run on hardware, with logs in the repo:** the v2 core booted on a brain (2026-08-12,
  `docs/internal/evidence/first-boot-2026-08-12.md`); the R1a bench session — eight motors commanded
  at +2.0 V through `ProsMotor`, sensors read, seven unit-scale beliefs settled (2026-08-13,
  `R1a-BENCH-SESSION.md`); Bench Tests on the old bot, a log written to the SD card (read back on a laptop), the IMU
  heading sign confirmed (2026-08-17..19, `evidence/r3a-session-2026-08-19*.log`, HA-02, HA-122);
  Bench Tests on robot two — the census, IMU on port 2, battery and controller reads, sign captures,
  ten motors constructed at 0 V (2026-09-10, `R3b-PROGRESS.md`). **Unlogged:** "shulib Drive" drove
  robot two open-loop through the library's adapters once (jal's report, 2026-09-17). **Nothing has
  run since the drivetrain was rebuilt (~2026-09-18).**
- **The robot's brain has not enumerated on this laptop's USB since 2026-09-18** (no kernel USB
  event on any port; suspected a charge-only cable, or the laptop's USB controller, which logged an
  error at 04:54 that morning). jal is getting a known data cable. **The moment `pros lsusb` shows a
  V5 System Port and a User Port, other work stops and S1 runs.**

---

## 2. The robots and the season (facts the next session must not re-derive)

- **VEX U, game "Override".** Two robots. **Both fall robots are TANK drives** (jal's ruling,
  2026-09-14). An X-drive is built in spring 2027; all holonomic validation waits for it.
- **Robot two** ("2026 tank chassis"): 5 coupled motors per side on one gear train, 4 wheels per side.
  Ports as last measured (before a REBUILD that made them unverified): LEFT 11 12 13 14 15, RIGHT 20 19
  18 17 16 (back → front); signs LEFT −11 +12 −13 +14 −15 | RIGHT +20 −19 +18 −17 +16; IMU port 2;
  radio port 1; front = the end where ports 15/16 sit — all MEASURED 2026-09-10. **Beliefs, not
  measurements:** blue cartridges (read off a motor), 2.75 in wheels (reported, not rulered). Port 18
  travels ~20 % short on a push (HA-130, unexplained). **The drivetrain was REBUILT around
  2026-09-18** — trust none of these until the census + MOTOR WATCH (or the calibrator) re-measure
  them.
- **The typed table is NOT a Teleop fallback.** `src/chassis_table.hpp` carries ports, signs and the
  cartridge but `trackWidthIn = 0.0` and `externalRatio = 0.0`, so `tableToDescription()` yields an
  invalid description and **shulib Teleop refuses to start without a SAVED calibration record** on
  the card. The table does drive shulib Drive and Bench Tests (they need only ports and the
  cartridge). So on the robot: Calibrate must succeed before Teleop can run at all.
- **Robot one:** also tank. **Its ports do not need typing** — shulib Calibrate takes every motor the
  brain's registry reports as a candidate and decides which drive, their signs and their sides by the
  push; it finds the IMU port from the registry too. Only three facts are human-stated: the
  cartridge, the kind (tank) and the front. **But today Calibrate builds only for `ROBOT=tank` and
  takes the cartridge from robot two's table** — robot one needs either its own variant or a stated
  cartridge before it can be calibrated (Q3).
- **Brain:** VEXos 1.1.5, PROS kernel 4.2.2. **Never start a program from the controller's own menu**
  (it arms a pretend field-control match, HA-128) — use the brain's screen (Run) or
  `pros v5 run <slot>` (check `pros v5 run --help` for the port argument at the session; the tree
  records the command in use but not that argument form). USB ports renumber — resolve with
  `pros lsusb` every time (HA-129). Read program output raw: `stty -F <user port> raw -echo; timeout
  N cat <user port> > file`, recover text with `strings`. The screen is resistive (firm presses).
  Check the battery first. An SD card (FAT32) in the brain is required for the calibration record
  and for logs.
- **No tracking wheels / rotation sensors on the fall robots** (jal's decision): odometry is the drive
  motors' own encoders + IMU (`DriveEncoderOdometry`), to be corrected by GPS and/or AprilTags and by a
  wall reset (L6). A consequence: **the odometry stall cross-check has no independent source on these
  robots and never reports**, so nothing detects a drive stall from odometry. GPS was named the
  priority purchase.
- **AI Vision sensors:** jal **has two** (his words, 2026-09-23). Recommended mounting (repeat to the
  builders if asked): **one per robot, on the chassis (never on a lift or intake that moves),
  front-facing, level, rigid, nothing in its field of view at any mechanism position, on a known smart
  port**; measure its forward offset, sideways offset and height from the robot's centre (inches) and
  whether it is level or tilted. Pieces are detected by the sensor's own onboard detection (colour
  signatures or the season's game-element model) — **pieces do NOT need tags**. Tags are only for
  localization — and **the Override field HAS them** (nine goals, ids 0–4 with 1–4 duplicated; see the
  game bullet below and Q4). Whether the built-in model recognizes Override pieces is **UNKNOWN** until the sensor is on
  the bench. **The vendored PROS 4.2.2 already has the AI Vision API** (`include/pros/ai_vision.h`),
  so the adapter (L3) is not blocked on a kernel upgrade.
- **The game, from the official manual** (Override Game Manual **v1.0, released 2026-07-02**,
  `content.vexrobotics.com/docs/2026-2027/override/files/v5rc-override-1.0.pdf`; read 2026-09-24):
  **VEX U Head-to-Head: a 30 s Autonomous Period (`<VUT4>`) then a 90 s Driver Controlled Period
  (`<VUT5>`); VEX U Autonomous Coding Skills: 60 s, two robots; VEX U Driving Skills: 60 s** (the Match
  table, Appendix B). **AprilTags are ON the field** (Appendix A, sheet A16 "AprilTag numbering/locations"):
  one on each of the nine goals, numbered 0 (centre) and 1–4 — **each of 1–4 appears TWICE**, mirrored,
  so a tag's number alone does not say which goal it is; the tag FAMILY is not in the manual's text
  (R2a's station cycles the four). **`localization::TagMap` refuses duplicate ids** (`tag_map.hpp`
  ~:92-104) and the AprilTag corrector looks placements up by id alone (`apriltag_corrector.hpp`
  ~:349, :394) — **it cannot represent this field as written**; R2b/E3 must allow several placements
  per id and choose by the current estimate (nearest, gated). **GPS field code strips are OPTIONAL per
  event** (the manual lists them among the modifications an event may make) — ask each event.
- **Mechanisms (jal, 2026-09-24):** the builders are still deciding; **the team will write its own
  custom mechanisms on the library's building blocks** ("using our library we will just make our own
  custom ones"). So L8 is re-scoped from season-named primitives the library implements to generic,
  well-documented building blocks (motor and pneumatic mechanisms, confirm-by-sensor or by-time, homing
  by limit switch or stall, safe states, bindings) plus a guide chapter and one worked example — to be
  briefed; pneumatics included (`PneumaticMechanism`, `ProsDigitalOut`, `ActuateAndConfirm` exist,
  host-only).
- **Competition:** jal has a competition the weekend of 2026-09-26/27 (his words, 2026-09-23). **Do
  not rush verification to hit it; do not promise the whole library by then.** Report honestly what
  is and is not proven at each step. As of 2026-09-24: no program can run an autonomous in a match
  (L2 does not exist), and the only driver-control program with a hardware history is shulib Drive.
- **VexBuilder:** jal, 2026-09-23: *"We are not working on the VexBuilder app. I want to code whatever
  I want on the real robots through our own library."* shulib's own no-code data layer (G1, G2) stays;
  work that exists only to serve VexBuilder's files (G3, G4) waits until after L1–L9 unless he says
  otherwise.

---

## 3. Standing rules

**RESUMING.md's "Pace and documentation rules", exactly as numbered there** (this list used to put
a different rule under number 8 and drop the close-out rule):

1. **One thing in flight.** One chunk, one executor. (Relaxed ONCE, for the unattended night of
   2026-09-18; back in force.)
2. **A brief waits for jal's go.** Write it, commit it, show him its scope + documentation impact
   list, STOP. Never write a brief and launch its executor in the same turn.
3. **Docs move in the same commit as the code, or the commit waits.** The commit body lists documents
   touched AND deliberately not touched, with why.
4. **Prose review is a logged step:** for every changed source file, grep `docs/` and `README.md` for
   its name and concepts, read the hits, fix what is stale, record what was checked. **Run the grep
   before writing that it was run** (the 2026-09-24 audit caught a commit body claiming a review the
   grep had not covered).
5. **Status pointers checked before every commit:** `docs/roadmap.md` "You are here" and the FIRST
   bold `**Next:**` block in `build-order.md` must be true for the tree after the commit.
6. **Report, then wait.** After each step say what changed and which docs moved, and stop.
7. **Size chunks so the doc pass fits** (~5 documents); split into parts otherwise.
8. **Session close-out.** Before a session ends: the tree is clean, or the PROGRESS log says exactly
   where work stopped; the four "Where things stand" commands give a true answer.

**jal's further standing words:** never claim a hardware result without the log or his report —
under-claim before over-claim; documentation is as important as the code; say "host test plant",
not "simulator".

Also: **executors are the Agent tool, `subagent_type: general-purpose`, model `opus`,
`run_in_background: true`, told "do not commit" and "create the PROGRESS log first and append as you
go".** Never use a `fable` model for an executor unless jal says yes for that specific run. **Verify
every executor yourself** before committing (all gates + read the load-bearing code + re-run the
load-bearing mutations by hand). **Commit trailer:** the attribution line the current session's
system reminder gives (it changed when the model changed; RESUMING says the same). Tests must try to
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

There is no tracked `verify.sh` — earlier sessions kept one in their scratchpad. These commands are
the set:

```sh
cmake --build build/test -j"$(nproc)" && ./build/test/shulib_tests | tail -6
python3 tools/src_build_gate.py self-test && python3 tools/src_build_gate.py check
find include/shulib -name '*.hpp' | sed 's|^include/||' | LC_ALL=C sort | awk '{print "#include \""$0"\""}' > /tmp/all.cpp
echo "int main(){return 0;}" >> /tmp/all.cpp
arm-none-eabi-g++ -std=gnu++20 -Wall -Wextra -Wconversion -Wsign-conversion -Wshadow -Werror -Os -mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=softfp -c /tmp/all.cpp -o /dev/null -Iinclude
grep -rnE '#[[:space:]]*include[[:space:]]*[<"]pros/' include/shulib | grep -v '^include/shulib/hal/pros/'   # must print nothing
grep -rnE --exclude-dir=sim '#[[:space:]]*include[[:space:]]*[<"]shulib/sim/' include/shulib                 # must print nothing
python3 tools/api_doc_tool.py check-fresh && python3 tools/briefing_status.py check
python3 tools/api_doc_tool.py check-removability   # the real C7 gate; stronger than a hand grep
```

---

## 4. What already exists (so nothing is rebuilt by mistake)

Motion core (C1–C5): `IMotion` primitives (`MoveToPose`, `TurnTo`, `StrafeTo`, `HoldPose`,
`DriveBrake`), `MotionScheduler`, the `Chassis` facade (**F6 frozen**; its verbs are `moveTo`,
`turnTo`, `strafeTo`, `hold`, `drive`… — **there is no facade `driveTo`**; `driveTo` is `Routine`'s),
per-motion results and the session header. Recipe API `chassis/routine.hpp` (**F10 frozen**; `then()`
deliberately unfrozen). Cookbook + generated API reference + doc gates (D3). Localization:
`Localizer`, `ComplementaryFusion`, `EkfFusion`, `GpsCorrector`, `AprilTagCorrector` (synthetic images
only), `PilonsOdometry`, `DriveEncoderOdometry`, the `IOdometry` seam. Diagnostics: `TermSink`, `SdSink`
blackbox + reader, fault latch, health monitor, loop monitor. Mechanisms: `hal/mechanism.hpp`
(`IMechanism`, `MotorMechanism`, `PneumaticMechanism`), `manipulation/mechanism_op.hpp`
(`RunUntilConfirmed`, `ActuateAndConfirm`) and `manipulation/stall_detector.hpp` (`StallDetector`) —
**seam only, F11 unfrozen, never on hardware.** Sequencing: `sequence/run_guard.hpp` (`RunGuard`, the
guaranteed end-of-run action; F12 unfrozen). Adapters (`hal/pros/`, 18 headers): motor, IMU, GPS,
rotation, distance, optical, digital in/out, controller, battery, clock, char sink, block sink +
source (SD), line display, tick pacer, device registry, and (G1 Part 2a, 2026-09-24) the device
provider `ProsDeviceProvider` — **no AI Vision adapter.** Drivetrain:
`hal::MotorGroup` (median reads, ABSENT members), `hal::DriveGeometry`, `hal::DrivetrainDescription`.
Calibration: `calibration/drivetrain_calibrator.hpp`, `calibration_record.hpp` (180-byte, CRC-32),
`boot_check.hpp`. Driver control: `teleop/stick_mapping.hpp` (raw, used by shulib Drive),
`coupled_side_monitor.hpp`, `drivetrain_degradation.hpp`, and T2's `input_shaping.hpp`,
`slew_limiter.hpp`, `drive_modes.hpp`, `field_centric.hpp`, `driver_control.hpp`, and (G1 Part 2a)
`teleop_loop.hpp` — `runDriverControl`, the loop over them (**none of it wired into any program
yet**). Data layer: `config/robot_config.hpp` (`MechanismConfig` exists as data; **nothing
constructs a mechanism from it yet**), `config/route.hpp` (**F8 frozen**; `Waypoint::reverse` exists),
`hal/device_provider.hpp` + fake (members: motor, imu, gps, rotation, clock, battery, telemetry,
pacer — **no distance, vision or digital-out yet**), `robot/robot.hpp` (`robot::validate` +
`robot::Robot`; since G1 Part 2a also `BuildOptions` for the boot check's absent drive ports,
`portMap()` and `driveOdometry()`), `runner/command_registry.hpp`, `runner/path_runner.hpp`, `runner/manifest.hpp`.

Programs in `src/` (six builds, `make ROBOT=… PROGRAM=…`): **Bench Tests** (`make ROBOT=tank`, slot 3,
the measuring tool; `src/bench_r3a.cpp`), **shulib Drive** (`PROGRAM=drive`, slot 1,
`src/drive_program.cpp`), **shulib Teleop** (`PROGRAM=library`, slot 2, built from the SD record — the
typed table cannot describe the drivetrain, see §2; the library teleop loop in `src/main.cpp`;
**never run**), **shulib Calibrate** (`PROGRAM=calibrate`, slot 4, `src/calibrate_program.cpp`;
**never run**), plus `bench` (the measured bench bot's tester) and `xdrive` (the only build with
invented ports, HA-111). **Every `autonomous()` commands nothing.** Upload: `rm -rf bin .d && make
ROBOT=tank PROGRAM=<p> && pros upload --slot N --name "<name>"` — one program at a time (each build
overwrites `bin/`).

---

## 5. EVERYTHING LEFT — the full list

Each item: **what** · **why** · **what exists to build on** · **what "done" means** · **what it
needs** · **documentation it owes** · **rough size**. Where a chunk exists in `build-order.md` its id
is given in brackets; `NEW` marks work the plan has no entry for — **add an entry to `build-order.md`
(and the deviations table) in the brief's commit before building it.**

### Part L — library code, done on the laptop (host-proven, build-gated; no robot needed)

**L1. [G1 Part 2 + T2's wiring] The PROS device provider, the library teleop loop, `main.cpp` as a
dispatcher.** — **BRIEF REVISED 2026-09-24 INTO PARTS 2a / 2b / 2c. PART 2a LANDED 2026-09-24,
HOST-ONLY** (jal's go that morning; `G1-PROGRESS.md` "Part 2a"); **2b and 2c each wait for his go.**
*(This line read "PART 2a WAITS FOR JAL'S GO" until 2a landed; the paragraph below is kept as it was
written before the go.)*
`docs/internal/chunks/G1-part2-pros-provider-and-main.md`. jal approved the direction ("we haven't
tested teleop before so we might as well rewrite") but has NOT said go. The 2026-09-24 audit found
four defects that would make an executor fail: `Robot`'s constructor re-runs `validate()` without
options (a tolerated dead port would throw inside it); only `ProsMotor` throws on a dead port (an
empty IMU/GPS/rotation port would come back live); the `src` build gate's variant differential
requires the X-drive helpers' `-Wunused-function` warnings in `main.cpp` (moving them fails all five
tank builds); and test 7 forbids `screen_print` in a `main.cpp` that §2.6 says keeps the state
screens. Plus smaller gaps (no tick-period source for the loop, the degradation policy's 3-per-side
floor, the panel reading private odometry, the fault hook across translation units, tank-stick mode's
right stick, a read-back-failing motor becoming tolerated). The revision fixes each (the builder now
takes the boot check's list of ABSENT ports instead of re-running the dead-port policy) and splits
the work into three parts so each doc pass fits: **2a** library only (provider, absent ports,
`Robot::portMap()`, the loop), **2b** shulib Teleop rebuilt on the library, **2c** `main.cpp` as a
dispatcher, the X-drive as data, the build gate's differential redesigned.
- What: `hal/pros/device_provider.hpp` (`ProsDeviceProvider`, registry-checked); absent drive ports
  in the builder, from the boot check; `teleop/teleop_loop.hpp` (`runDriverControl`, PROS-free); shulib Teleop moved to
  `src/teleop_program.cpp` built as provider → `bootConfig()` (SD record → typed table → refuse) →
  `robot::Robot` → the loop; the X-drive graph as a config constant; `src/main.cpp` a dispatcher.
- Rulings already made: only Teleop changes feel (shulib Drive and the tester keep `mapSticks`);
  field-centric ships selectable but OFF; the registry check and the builder's device pass both stay
  and name a port once.
- Done / docs owed: per the revised brief's DoD and documentation lists.
- Size: ~1–1.5 days executor + verification, across its parts.

**L2. [COMP] The competition program.** BRIEFED 2026-09-24 (`docs/internal/chunks/COMP-competition-program.md`, two parts, each on jal's go, after G1 Part 2b); it did not exist in the plan or the tree before that (the G1 Part 2
brief names it as a future consumer of the builder's dead-port tolerance).
- What: the program uploaded for a match. `initialize()`: boot the robot (IMU calibration, port check
  against the SD record, status screen, refusal by name). A **brain-screen auton selector** (touch
  buttons: e.g. Left / Right / Skills / None; the choice survives until `autonomous()`; defaults to
  None). `autonomous()`: run the chosen routine under `sequence::RunGuard` with the match duration and
  the guaranteed end-of-run action (the park); both a `chassis::Routine` function and a
  `runner::PathRunner` route must be selectable. `opcontrol()`: `runDriverControl` with the driver's
  profile and the bindings (L5). `disabled()`/`competition_initialize()`: state screens. Handoff:
  anything running when a phase ends is cancelled into the safe state.
- **A wiring problem to solve in the brief:** `RunGuard` only guarantees its end-of-run action when
  it IS the chassis's pacer (`run_guard.hpp`, "`chassis` MUST be the one constructed with THIS guard
  as its pacer"), but `robot::Robot` builds its `Chassis` with `devices.pacer()` and has no way to
  take another. Either the provider hands out a guard-wrapped pacer or `Robot` gains an option to
  take a pacer — decide in the brief; either is additive.
- Why: every piece exists except the thing that ties them into a match. **It is also what lets R3b
  and M1 close**: R3b's DoD is "a v2 tank auton runs on the robot under the library's own steering",
  and no program can command one today.
- Builds on: L1's provider/loop/`Robot`, `RunGuard`, `Routine`, `PathRunner`, `CommandRegistry`.
- Done: a seventh build (`PROGRAM=competition`, a beacon, the gate updated), the selector and the
  phase logic host-tested with fakes (phase transitions, cancellation, the park firing on a stalled
  loop), routines registered by the STUDENTS' code (the library provides the registration API; it
  ships only an example routine in the cookbook, not season content).
- Docs owed: a guide chapter "The competition program" (written from the code, examples compiled and
  quoted verbatim), README, roadmap (M4's guaranteed-park line), build-order entry + Next, the
  worksheet (a new station for a practice match), the changelog.
- Size: ~1 day.

**L3. [R2] The AI Vision adapter.**
- What: `hal/pros/vision.hpp` implementing `hal::IVision` (object observations) and `hal::ITagSource`
  (tag observations) — the seams are in `include/shulib/hal/vision.hpp`. **The vendored PROS 4.2.2 has
  the API** (`include/pros/ai_vision.h` / `.hpp`); register every belief about it (units, frame,
  latency, sentinel values, the tag family) as HA entries. Mount pose (forward/left offset, height,
  tilt) as config (`SensorsConfig` gains a vision entry; `RobotConfig` stays additive). Support BOTH
  detection modes (colour signature and the built-in game-element model) since which works is
  UNKNOWN.
- **What is NOT there yet, found by the audit:** `hal/vision_conversion.hpp` holds only the tag PnP
  (`tagCornersToRobotPose`), which needs camera intrinsics (fx, fy, cx, cy) the config does not carry;
  there is **no object box → bearing helper** (the header assigns it to M4); and `IDeviceProvider` has
  no `vision()` member — each is an additive piece of this chunk.
- Done: host-tested through a shim extension exactly like the other adapters (`test/pros_shim/`),
  wired into `robot::Robot` when configured (the `AbsentVision`/`AbsentTagSource` path unchanged when
  not), the AprilTag corrector installable from config when a tag map is given.
- Needs on the robot later: S6.
- Docs owed: HA entries, roadmap (the R2 checkbox sits under **M1** in the roadmap, not M3), build-order,
  guide ch. 14, the worksheet (a vision station: what the screen shows for a piece and a tag).
- Size: ~1 day host; the bench check is S6.

**L4. [NEW-TARGET] Piece targeting.**
- What: a motion or helper that turns `ObjectObservation`s into a drive toward the nearest piece of a
  chosen class, with a timeout and a "lost it" exit reason; an intake-while-approaching composition
  with the mechanism op. Pieces need no tags.
- **A constraint found by the audit:** `hal::ObjectObservation` carries only `classId`, `bearing` and
  `confidence` — "a BEARING only: a bounding box carries no range" — and it is part of F4, LOCKED
  2026-06-19. The box size exists only at the PROS layer. Range-from-box-size therefore needs an
  additive sibling type (or an adapter-level estimate) under F4's additive rules — decide in the
  brief; a bearing-only approach (turn to it, drive until the intake confirms) needs none.
- Builds on: L3, the motion primitives, F1's mechanism ops.
- Done: host-tested with a fake vision source producing synthetic observations from the plant's truth
  (the pattern E3 used for tags); the approach settles on the piece within a stated bound.
- Docs owed: cookbook recipe, guide, roadmap, HA entries for any distance-from-box-size belief.
- Size: ~1 day.

**L5. [T3] Button bindings and driver macros.**
- What: bind a button (hold/press/toggle) to a mechanism operation or a chassis action; **stick input
  takes control back within one tick; releasing the button cancels and the mechanism ends in its
  declared safe state** (asserted at the motors). Per-consumer `ButtonEdge` (HA-104).
- The binding mechanism is the library's; which button does what is the team's.
- Docs owed: guide chapter section on driver control, cookbook, roadmap Phase T rows, build-order.
- Size: ~½–1 day.

**L6. [NEW-WALLRESET] Re-seat the position estimate against a wall or goal.**
- What: a primitive "drive slowly toward a known wall until the distance sensor reads X, then set the
  estimate's coordinate on that axis from the known wall position and the sensor offset." Never touch
  heading (IMU-owned).
- **Three gaps found by the audit:** (1) there is **no one-axis setter** — `Localizer::setPose` moves
  x and y together (and re-seeds both), and `Chassis::setPose` (F6-frozen) takes a full pose; a
  one-axis re-seat is an additive member (on the facade it is an F6 additive → a minor bump). (2)
  **Nothing detects a drive stall on these robots** (the odometry stall check has no independent
  source), so the stop is the distance reading, with motor current/velocity as a secondary stop the
  chunk must build. (3) `IDeviceProvider` has no `distance()` member (the adapter exists).
- Why: with drive-encoder odometry the estimate drifts with every skid; this is the cheapest,
  field-independent correction a tank auton has.
- Done: host-tested on the plant with a synthetic distance channel derived from truth; a routine step
  usable from `Routine` via `then()` or as a runner built-in (adding a built-in id is additive under F8).
- Docs owed: cookbook recipe, guide, HA entries (sensor offset, wall positions), roadmap, changelog.
- Size: ~1 day (it was ½ before the gaps were found).

**L7. [R4 + R5 + E5, reframed] Gains and noise as self-run calibration routines** inside shulib
Calibrate. **The plan already has this work** — R4 (IMU bias/drift), R5 (the on-robot ramp routine),
E5 (calibration routines + persistence to SD) — so it is not new; what is new is the framing jal
agreed to on 2026-09-18 ("calibrate absolutely everything it needs to know"): **R5's DoD says the gains
are committed as constants, and this persists them to the SD card at runtime instead.** That departure
is a plan change for jal to confirm in the brief (Q7).
- What: a voltage-ramp routine per side that fits feedforward kS/kV/kA (replacing HA-45's
  placeholders); a stationary IMU drift/noise capture; loop-timing capture. Persisted in the SD record
  (a record v2 with its version bump and a decoder that still reads v1, or a sibling record — decide in
  the brief) and consumed by the builder as the tuning.
- Done: host-proven on the plant (the fitted gains recover the plant's configured gains); the record
  round-trips; Teleop and the competition program use measured gains when present and say which.
- Docs owed: HA-45/50/51/52 and the noise entries' settle lines, roadmap R4/R5, build-order R4/R5/E5,
  guide, worksheet station.
- Size: ~1–1.5 days host; the measurement is S2/S3.

**L8. [F3] The scoring primitives.** **BLOCKED on Q1 (the mechanism decisions).**
- What (build-order's list, in its value order): `setQuadrantToggle` (index N clicks on the 3-state
  Toggle with an Optical colour confirm — highest value), `orientToScoringHalf`, `intakeUntilCapture`,
  `liftToLevel` (homing inside it: limit switch or stall — jal/builders decide), `rotateClampToAngle`,
  `clampActuate`/`clampConfirm`, `deployActuator` — each a bounded, confirmed `IMechanismOp` over
  `MotorMechanism`/`PneumaticMechanism`, registered with names. Task-sensor confirmation on every grab
  and place.
- Done: host-tested against fakes/hostile fakes; then S4 on hardware.
- Docs owed: a mechanism chapter in the guide, cookbook recipes, HA entries for every threshold,
  roadmap M4. **F11 does not freeze here** — its trigger is F3's primitives run **on hardware** (S4).
- Size: ~2 days once decided.

**L9. [NEW-ERGONOMICS] The small API additions the readable routine depends on.**
- Reverse driving: **the facade has no `driveTo`** (it is `Routine`'s, F10-frozen), and the data path
  already has it (`Waypoint::reverse`, which the runner honours). So the work is an ADDITIVE option on
  `Routine::driveTo` — a `kApiMinor` bump, pins updated deliberately, a changelog entry — following
  `include/shulib/version.hpp`'s policy exactly.
- A named-mechanism lookup on `robot::Robot` (`robot.mechanism("intake")`): **bigger than it looked** —
  `Robot` constructs no mechanism from `MechanismConfig` today, and the config lacks what a mechanism
  needs (its safe state / brake mode, the motors' cartridge); the provider lacks a digital-out member.
- A `Robot::fromCard()`-style one-liner (may be L1's `bootConfig()` exposed; decide in L1/L2).
- Docs owed: the register rows touched, the changelog, guide, cookbook.
- Size: ~1 day.

**L10. The remaining plan chunks (lower priority for the season):**
- **[G3] `.vexbot` ingestion + codegen → freeze F7.** Serves VexBuilder's files — after L1–L9 unless
  jal says otherwise (§2). ~1.5 days.
- **[G4] legacy `.shupaths` importer + the ten-minute guide** (closes M5). ~1 day shulib side.
- **[H1] SHUL/2 telemetry wire → freeze F9.** ~1 day.
- **T2 finish:** re-point shulib Drive and the tester's DRIVE station at `DriveShaper` AFTER a driver
  has compared feels on hardware (the G1 Part 2 brief's ruling M4) — then **`T2-COMPLETED.md`**, which
  `T2-PROGRESS.md` owes.

### Part Q — decisions only jal and the build team can make (code waits on these)

- **Q1. The mechanisms on each robot:** intake type, lift type and levels, clamp (yes/no, pneumatic or
  motor), every motor port and sign, every pneumatic port, and how the lift homes (limit switch vs
  stall — owed since 2026-08-13). **Blocks L8, L5's meaning, and every scoring routine.**
- **Q2. Sensor placement:** AI Vision per §2 (one per robot, chassis, front, level, offsets measured);
  GPS — bought? where mounted? (lever arm offsets); distance sensors for the wall reset — where, facing
  which way.
- **Q3. Robot one's cartridge**, and whether it gets its own build variant (Calibrate builds only for
  robot two's `ROBOT=tank` today). Its ports are discovered by the calibrator.
- **Q4. ANSWERED 2026-09-24:** yes — AprilTags on the nine goals, ids 0–4 with 1–4 duplicated (§2).
  The tag FAMILY is still unknown (R2a's station finds it on a printed tag or the field).
- **Q5. The auton routines themselves** are the STUDENTS' to author (the guardrail: students must be
  able to write and defend competition code). The next session teaches, reviews and supplies
  primitives; it does not write the season's routines.
- **Q6. The `MotionDeps` version ruling (found 2026-09-24).** R3b Part 1 (`3cf03d0`) tightened the
  motor-count check from "at least as many motors as wheels" to "exactly as many" with no major bump.
  By `version.hpp`'s policy that is breaking (the old docs made the looser rule valid, and `MotionDeps`
  feeds the frozen `Chassis` constructor). Bump to 3.0 with a migration note, or rule it a defect fix?
  The changelog's API 2.2 entry states the migration and says the ruling is open.
- **Q7. L7's framing:** gains persisted to the SD card at runtime (the 2026-09-18 framing) or
  committed as constants (R5's DoD as written)?

### Part S — sessions with the robots, in order (none can be done at a desk; each produces evidence)

Rules for every session: brain on USB via a data cable (`pros lsusb`), SD card in, battery checked, the
program started from the brain screen (never the controller menu), the user port captured raw to a file,
**everything the robot printed recorded in the relevant `-PROGRESS.md` as evidence**, no claim without it.
**Build the packages on this laptop from the committed tree** — do not `git pull` (the remote is
behind) — and, if an executor is working in the main tree, build in a separate worktree at the commit
you mean so its in-progress edits cannot reach a package.

- **S1. The first drive** — worksheet **Station G** (`docs/internal/chunks/R3a-BENCH-WORKSHEET.md`,
  from line ~597) is the runbook: rebuild Bench Tests, Calibrate and Teleop (the hardware checkout
  holds only Drive) → Bench Tests census + MOTOR WATCH on the rebuilt drivetrain → shulib Calibrate
  wheels-up (expect the yaw-floor refusal — that is correct) → Calibrate on the floor (push
  front-first, answer the prompts, tape push, spin — 3 m clear, a second person at the battery) →
  *(Superseded in part by R3d Part 2F-1, 2026-09-24: the calibrator now takes ONE 48 in push — no
  separate tape push, no distance question — and a wheels-up run now refuses at the first port as
  "rolled / not on the floor" after a 3 → 6 → 9 V ramp, not at the yaw floor. Station G was rewritten
  from the new program at 2F-2 — the wheels-up run is now OPTIONAL; follow Station G, not this
  line.)* →
  `SAVED` read back → a second Calibrate run for reproduction → shulib Teleop from the record, wheels
  up then floor. **What it closes:** R3d (on the reproduction run). **What it cannot close:** R3b and
  M1 — no uploaded program commands a `moveTo`, so the "few inches of `moveTo`" this file used to list
  is impossible until L2 (or a minimal auton hook) exists; R3b closes on that later run. Then write
  `R3d-COMPLETED.md` from the log, move the roadmap pointer and the Next block, flip only evidenced
  checkboxes. **The "never driven a robot under its own control" sweep** (`docs/internal/guide-maintenance.md`'s "R3 — first
  real robot run" row) happens when the log shows the library's MOTION STACK drove — every instance
  in ONE commit: `README.md`, `docs/README.md`, guide ch. 01, 07, 14 and the guide README, the master
  plan's status line, `docs/vexbuilder-integration.md`. Teleop driving from the record is the library
  driving open-loop through its own graph — say exactly that, not more.
- **S2. Real gains** (L7's ramp on the floor), persisted.
- **S3. Real noise** (IMU at rest, encoders rolling, loop timing, battery sag under stall).
- **S4. Mechanisms on hardware** (after L8), one log per primitive — F11's freeze trigger.
- **S5. Robot one** through S1–S3.
- **S6. Vision on the bench** (after L3): what the sensor reports for a piece and a tag, latency, frame
  conventions; settle L3's HA entries.
- **S7. Absolute references on the field:** GPS with the field strip (if bought), tags (if present), the
  wall reset against a real wall.
- **S8. The first on-robot autonomous** — a student-authored routine run through the competition
  program (L2), graded by where the robot ends up. **This is where R3b and M1 close.** Then robot one;
  then both.
- **S9. Retune** constraints/gains after field runs.

**R3a is also open** (its DoD at build-order's R3a entry: the register's groups settled or re-planned);
S1–S3 feed it, and its `-COMPLETED.md` is written when that DoD is met.

### Part P — back on the laptop after the sessions

- **P1. [R6] Back-fit the host test plant** with the measured numbers (R4/R5) and re-run the whole
  suite; every newly failing test is a real defect the invented numbers hid — fix each where it lives.
- **P2. Settle the Hardware Assumptions Register:** every HA entry the sessions touched gets its settle
  line answered with the log reference, or its number moved.

### Part E — later, not this season's critical path

Continuous path following (pure pursuit / blended legs instead of stop-and-settle); the on-brain live
tuner (H3); visual-servo docking (E6); holonomic validation on the spring X-drive (R3c's holonomic
half, and I1 — H-drive hardening end-to-end); the VexBuilder sim seam and replay (H2); the
robot-to-robot coordination seam (I2).

---

## 6. DOCUMENTATION — as important as the code (jal, 2026-09-23: "documentation is also incredibly important for when it finishes")

**Per chunk (every item above):** the brief, the live `-PROGRESS.md`, the `-COMPLETED.md` record, the
generated API reference (`python3 tools/api_doc_tool.py generate` — the freshness gate fails otherwise),
`///` on every public member (the coverage gate fails otherwise), guide/cookbook examples COMPILED in a
test file and quoted verbatim (the verbatim gate), the Hardware Assumptions Register for every invented
or reasoned number, **the changelog entry for any user-visible change** (`docs/changelog.md`: one
section per API version, written when the change lands — API 2.2 was written six days late), the
roadmap row/"You are here", the build-order status + Next block, the prose review logged, and the commit
body's touched/not-touched list. None of this is optional.

**When the library is finished, the documentation that must exist for a coder on the team** (check each
exists and was written FROM WHAT ACTUALLY HAPPENED, not from intent):
1. **README.md** — an honest front door: what it is, what is proven on hardware and what is not, the
   two-line robot, a short routine, links.
2. **A "first hour" guide for a new student coder:** clone, build, upload each program, run the
   calibrator, drive, write and run a first routine on the robot. Followed start to finish by someone
   who did not write it (the user guide chunk C8's rule; D3's still-open partial).
3. **The competition program chapter** (L2): selector, phases, the park, how routines are registered.
4. **The mechanism chapter** (L8): each primitive, its confirmation and failure modes, homing.
5. **The driver-control chapter** (L1, L5, T2): profiles, modes, field-centric and its fallback,
   bindings.
6. **The calibration chapter:** what the calibrator measures, the record on the card, boot
   verification, what to do when it refuses (each refusal message and its fix).
7. **The vision chapter** (L3, L4): mounting, detection modes, targeting, tags if used.
8. **The accuracy chapter:** odometry, wall reset, GPS/tags, what drift to expect (measured numbers).
9. **A field-session runbook** (the worksheet's stations consolidated for competition days: pre-match
   checks, reading the brain screen, reading a log off the SD card).
10. **Troubleshooting** (`docs/guide/12-when-things-go-wrong.md` updated with every real failure seen).
11. **`docs/guide/14-what-it-cannot-do-yet.md`** — kept exactly true at every commit.
12. **The generated API reference, published.** The site **is live** — docs.shurobotics.com, published
    from `main` by `.github/workflows/pages.yml` (`docs/internal/docs-publishing.md`); it shows what
    `main` has, so it lags `shulib-v2` until the next release.
13. **The Hardware Assumptions Register** with every settled entry citing its log.
14. **The changelog**, one section per API version.
15. **A release** through `docs/internal/RELEASING.md` / `tools/release.py` when jal asks (`release/v2`
    = `shulib-v2` merged with `docs/internal/` dropped; `main` = a `git commit-tree` snapshot of it —
    so no public doc may reference `docs/internal/`).

---

## 7. The recommended order

1. **The G1 Part 2 brief was revised and committed on 2026-09-24** (three parts). **Part 2a landed
   the same day** (host-only). Show jal Part 2b's scope and documentation list and **wait for his go
   (L1).** On go, launch the executor per §3, verify, commit — then 2c, with its own go.
2. **The moment the brain enumerates: S1** — stop other work and do the first drive (Station G).
   Nothing is worth more. It needs nothing from L1.
3. Brief → go → build, one at a time: **L2** (it is also what lets R3b/M1 close), **L3**, **L6**,
   **L7**, **L5**, **L9**, then **L4**. Ask jal for **Q1–Q7** early so **L8** is not the thing that
   stalls.
4. Sessions S2, S3, S6 as the robot is available; S4 after L8; S5 for robot one; S7–S9 on the field.
5. P1, P2, then the documentation set in §6 checked item by item; L10 when jal wants it.

**Do not** tell jal the whole library can be finished by the 2026-09-26 weekend. Tell him, with evidence,
what is proven at each step.
