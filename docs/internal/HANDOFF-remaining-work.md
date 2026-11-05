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
- **Pushed:** `origin/shulib-v2` is `ceb17c0` (CAL1 Part A1), pushed 2026-10-18 on jal's "Push now" — every
  commit up to it, those since `bdcbc7c` included, is on GitHub. *(Corrected 2026-10-18: this line said
  `6ceb244` (2026-09-17 19:58, "shulib Drive drove robot two") and "everything after it is unpushed" — the
  tree moved past both.)* Anything newer is unpushed — `git log --oneline origin/shulib-v2..HEAD` lists it.
  Push only when jal asks.
- **Suite:** 1358 test cases / 1,635,266 assertions, 3 skipped, all green (since the 2026-09-24
  build-identity fix `05ad50a`; the 1,635,290 recorded before it included six assertions from a stale
  `-dirty` build hash — see `C5-COMPLETED.md` D8's correction).
- **Gates, all green (re-run 2026-09-24):** host suite (the doc gates run inside the test build:
  coverage, freshness, verbatim examples, removability, briefing, staleness); `python3
  tools/src_build_gate.py self-test && check` (**six** builds: `bench`, `xdrive`, `tank`,
  `tank-drive`, `tank-library`, `tank-calibrate`); the ARM header gate (181 headers); the two CI
  greps; `python3 tools/api_doc_tool.py check-removability`.
- **Open chunks** (a `-PROGRESS.md` with no `-COMPLETED.md`): **G1** (Part 1 landed; Part 2 briefed
  in three parts — Part 2a landed 2026-09-24, host-only; Part 2b landed 2026-09-26, build-gated, this
  version of "shulib Teleop" unrun; Part 2c landed 2026-09-26, build-gated — G1's completion record
  is written when 2c is merged),
  **R3a** (bench measurements; its DoD's register groups are unsettled), **R3b** (host pieces landed;
  closes on an on-robot auton), **R3d** (Parts 1–2 landed; Part 3 is the bench session), **T2**
  (library half landed; its `src/` wiring landed with G1 Part 2b, unrun — `T2-COMPLETED.md` is owed
  once a driver has compared the feel on the robot).
  None is interrupted: the tree is clean. G2 was closed on 2026-09-24 (`G2-COMPLETED.md`).
- **Side worktrees:** `.claude/worktrees/agent-aef02a3f7fb230a9d` (T2's, integrated — remove with
  `git worktree remove --force <path>` and then `git branch -D worktree-agent-aef02a3f7fb230a9d`,
  its leftover branch); `.claude/worktrees/hw-17aa4eb` (detached at `5e4ce65`; **its `bin/` holds ONE
  package — shulib Drive** (the OLD one, replaced at T4 Part 3): each `rm -rf bin .d && make …`
  overwrote the one before, so Bench Tests, Calibrate and the programs since must be rebuilt — "shulib
  Teleop" no longer among them: T4 Part 3 retired it, and `PROGRAM=library` is now a `make` error).
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
- **The typed table is NOT a fallback for the programs that boot from the record.** `src/robots/chassis_table.hpp`
  carries ports, signs and the cartridge but `trackWidthIn = 0.0` and `externalRatio = 0.0`, so
  `tableToDescription()` yields an invalid description and **"shulib Drive" (since T4 Part 3; "shulib
  Teleop" until then) refuses to start without a SAVED calibration record** on the card — as does
  every program that boots robot two from the record. The table drives Bench Tests only now (it needs only
  ports and the cartridge; the OLD "shulib Drive" drove from it until T4 Part 3 replaced it). So on
  the robot: Calibrate must succeed before "shulib Drive" can run at all.
- **Robot one:** also tank. **Its ports do not need typing** — shulib Calibrate takes every motor the
  brain's registry reports as a candidate — EXCEPT a port `src/robots/mechanism_table.hpp` declares as a
  mechanism motor (robot two's claw pivots on 6 and 7, since T3: never a candidate, never touched) —
  and decides which drive, their signs and their sides by the
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
  (L2 does not exist), and the only driver-control program with a hardware history is shulib Drive
  (the OLD one — T4 Part 3 replaced it with a new program of the same name, which has never run).
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

### The stale-binary trap (learned five times — do not relearn it)

`cmake --build build/test` runs the doc gates BEFORE linking, and the briefing gate runs the OLD
binary. A red (mutant) binary therefore blocks its own relink, and the next run silently reports the
stale result. **Mutation recipe:** edit → `touch` EVERY test file that includes the edited header
(an `inline constexpr` table leaves stale objects otherwise) → `cmake --build build/test --target
shulib_tests/fast -j"$(nproc)"` → **require `Linking CXX executable shulib_tests` in the output** →
run it BOUNDED — `(ulimit -v 4194304; timeout 300 ./build/test/shulib_tests)`, never bare (a
mutant can loop and allocate without end: on 2026-10-15 one took the desktop down) → observe red → restore →
relink → green. Regenerate the briefing (`python3
tools/briefing_status.py generate`) only AFTER the last restore relinks green. **Two more forms (2026-09-29):**
(1) a test file ADDED after the last full (non-`/fast`) build is not in make's dependency database, so a
restored header does not rebuild it and the next mutation links the previous mutant through it — after
adding a test file, run one FULL build before any mutation (L9 Part A1's first mutation pass, discarded);
(2) a mutation run KILLED mid-way (a usage-limit stop) restores its source but leaves the last mutant
LINKED — relink from source before trusting any suite result (H1 Part A1: two golden cases red on the
M26 mutant's binary, 1,781 of 1,781 green after the relink).

### The coordinator's verification command set

There is no tracked `verify.sh` — earlier sessions kept one in their scratchpad. These commands are
the set:

```sh
cmake --build build/test -j"$(nproc)" && python3 tools/suite_run.py green   # the BOUNDED run (tools/suite_run.py)
python3 tools/src_build_gate.py self-test && python3 tools/src_build_gate.py check
find include/shulib -name '*.hpp' | sed 's|^include/||' | LC_ALL=C sort | awk '{print "#include \""$0"\""}' > /tmp/all.cpp
echo "int main(){return 0;}" >> /tmp/all.cpp
arm-none-eabi-g++ -std=gnu++20 -Wall -Wextra -Wconversion -Wsign-conversion -Wshadow -Werror -Os -mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=softfp -c /tmp/all.cpp -o /dev/null -Iinclude
grep -rnE '#[[:space:]]*include[[:space:]]*[<"]pros/' include/shulib | grep -v '^include/shulib/hal/pros/'   # must print nothing
grep -rnE --exclude-dir=sim '#[[:space:]]*include[[:space:]]*[<"]shulib/sim/' include/shulib                 # must print nothing
python3 tools/api_doc_tool.py check-fresh && python3 tools/briefing_status.py self-test && python3 tools/briefing_status.py check
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
guaranteed end-of-run action; F12 unfrozen; since MECH-A2 its deadline and an abandon also cut a claimed
mechanism). Adapters (`hal/pros/`, 21 headers): motor, IMU, GPS,
rotation, distance, optical, digital in/out, controller, battery, clock, char sink, block sink +
source (SD), line display, tick pacer, device registry, (G1 Part 2a, 2026-09-24) the device
provider `ProsDeviceProvider`, (SAFE-SAVE Part A, 2026-10-04) the SD store `ProsFileStore` built
over the block sink and source, (CAL1 Part A1, 2026-10-15) the brain's touch screen
`ProsBrainScreen`, (CAL1 Part A2a, 2026-10-29) the standard Calibrate's shell
(`hal/pros/calibrate.hpp`: `ProsCalibrateIo` and the five entry functions — not an adapter), and (VIS1-A1,
2026-10-30) the AI Vision camera `ProsAiVision` — read a FRAME at a time, its colours and AI model only; **no
AprilTag adapter.** Drivetrain:
`hal::MotorGroup` (median reads, ABSENT members), `hal::DriveGeometry`, `hal::DrivetrainDescription`.
Calibration: `calibration/drivetrain_calibrator.hpp`, `calibration_record.hpp` (180-byte, CRC-32),
`boot_check.hpp`, `calibrate_panel.hpp` (the calibrator's screens and touch), (SAFE-SAVE Part A) `record_store.hpp`
(the safe save), (CAL1 Part A1) `calibrate_screen.hpp` (the painter) and `front_check.hpp` (`compareFronts`), and
(CAL1 Part A2a) `calibrate_program.hpp` (the standard Calibrate, `CalibrateProgram` — host-only; no program builds
it yet), and (R5a Part A) `drive_characterization.hpp`, `drive_characterizer.hpp`, `feedforward_fit.hpp` and
`rest_capture.hpp` (the GAINS and REST measurements; Calibrate runs them in R5a Part C). Driver control: `teleop/stick_mapping.hpp` (raw, used by the bench tester's DRIVE
station; the OLD shulib Drive used it until T4 Part 3),
`coupled_side_monitor.hpp`, `drivetrain_degradation.hpp`, and T2's `input_shaping.hpp`,
`slew_limiter.hpp`, `drive_modes.hpp`, `field_centric.hpp`, `driver_control.hpp`, and (G1 Part 2a)
`teleop_loop.hpp` — `runDriverControl`, the loop over them (**called by robot two's driver file —
which "shulib Drive" and the competition program run, since T4 Part 3; "shulib Teleop" called it from
G1 Part 2b until then — and by the invented X-drive since Part 2c; none has run with it**). Data layer: `config/robot_config.hpp` (`MechanismConfig` exists as data, and since T3
`robot::Robot` builds each one — since MECH-A1, 2026-10-08, through `robot::MechanismParts`, with its sensor roles), `config/route.hpp` (**F8 frozen**; `Waypoint::reverse` exists),
`hal/device_provider.hpp` + fake (members: motor, imu, gps, rotation, digitalOut (T3), and since MECH-A1
distance, optical, digitalIn and the read-fault pair; clock, battery, telemetry, pacer — **no vision yet**); the
mechanism core's vocabulary `mechanism/{kinds,spec,wiring,route_ids,devices}.hpp` (MECH-A1) and its service
`mechanism/{until,service}.hpp` (MECH-A2: power and pneumatic mechanisms run through it; MECH-A3: position and speed
too — arms, lifts and flywheels move, hold, home and spin; host-only, no program built one until MECH-A5), and what calls the
service (MECH-A4, built 2026-10-22, its review's fix round 2026-10-27, its verification round 2026-10-29; LANDED 2026-10-29): the Routine's mechanism steps (`chassis/routine.hpp`, API 2.5),
`runAuton`'s and `runDriverControl`'s service overloads, `mechanism/{driver_inputs,direction_check}.hpp` and
`runner/mechanism_routes.hpp` (MECH-A5, LANDED 2026-10-30, built 2026-10-29: robot two's three programs build
the service and run its claws on it — `src/robots/mechanism_table.hpp`, `src/common/driver_file.hpp`; MECH-A6, LANDED
2026-11-06 (the core complete): Bench Tests' station 13 runs the direction check on the robot — `src/programs/bench_r3a.{hpp,cpp}`,
worksheet Station P), `robot/robot.hpp` (`robot::validate` +
`robot::Robot`; since G1 Part 2a also `BuildOptions` for the boot check's absent drive ports,
`portMap()` and `driveOdometry()`), `runner/command_registry.hpp`, `runner/path_runner.hpp`, `runner/manifest.hpp`.

Programs in `src/` (seven builds since T4 Part 3, `make ROBOT=… PROGRAM=…`): **Bench Tests** (`make ROBOT=tank`,
slot 3, the measuring tool; `src/programs/bench_r3a.cpp`), **shulib Drive** (`PROGRAM=drive`, slot 1,
`src/programs/drive_program.cpp` — since T4 Part 3 the team's driver file on `robot::Robot`, built from the SD
record (the typed table cannot describe the drivetrain, see §2), with the library's drive protections, the
claws and "shulib Teleop"'s diagnostics as its DETAILS page; **never run** — the OLD "shulib Drive" it replaced
drove robot two open-loop on 2026-09-17, and the "shulib Teleop" it retired (`PROGRAM=library`, slot 2, now an
`$(error)`) drove it from the record on 2026-09-25), **shulib Calibrate** (`PROGRAM=calibrate`, slot 4, `src/programs/calibrate_program.cpp`;
**first ran on robot two 2026-09-24** — run 1 stopped on `SPIN_DISAGREE`, run 2 finished with no
serial capture; its recorded reproduction run is still owed), **shulib Auton Tester** (`PROGRAM=autontest`,
slot 2 — slot 7 until T4 Part 3 —, `src/programs/auton_tester_program.cpp`; COMP-t, 2026-09-28 — runs one of the team's
autons from `src/team/autons.cpp` on the bench through `competition::runAuton`; **never run**), **shulib
Competition** (`PROGRAM=competition`, slot 6 — slot 5 stays the team's old-code fallback —
`src/programs/competition_program.cpp`; COMP-b, 2026-09-30 — THE program for a match: the auton chosen on the
brain's menu runs in `autonomous()` on the program's own task through the same `competition::runAuton`, a
disable or the end of the period ends it, then driver control runs the team's driver file; **never run**,
worksheet Station K), plus `bench` (the measured bench bot's tester) and `xdrive` (the only build with
invented ports, HA-111 — since G1 Part 2c `src/robots/xdrive_config.hpp` on `robot::Robot`, refusing at boot
on any robot that exists). `src/build_selection.hpp` chooses, in one chain, which program AND which robot's tables a build is,
and `src/main.cpp` only hands each PROS entry point to that program (G1 Part 2c and its final run).
Since SRC1 (2026-09-27, the team lead's layout) `src/` is one folder per job: `src/robots/` (what
each robot IS — the two tables, the X-drive's config), `src/team/` (the team's own code, where
students edit — its first file is robot two's driver file, `src/team/robot_two_driver.hpp` (T4
Part 1a), and its second the team's autons, `src/team/autons.{hpp,cpp}` (COMP-t), the one `src/`
`.cpp` the host suite compiles), `src/programs/` (one
program each) and `src/common/` (what the programs share — the boot source, the state screens,
the screen-text path, the fault-latch hook, a driver file's binder and checks); every include of a `src/` file is written from `src/`
(`#include "robots/chassis_table.hpp"`), and the `src` gate's step 0 fails any path a second
include root could answer and any `src/` path a program prints that names no file (so a message
that tells a student which file to edit cannot point at a moved one).
**Every `autonomous()` commands nothing.** Upload: `rm -rf bin .d && make
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
HOST-ONLY** (jal's go that morning; `G1-PROGRESS.md` "Part 2a"). **PART 2b LANDED 2026-09-26,
BUILD-GATED — "shulib Teleop" on `robot::Robot`; THIS VERSION HAS NOT RUN ON A ROBOT** (jal's answers
of 2026-09-26: switch to the feel layer, status pointers only, the documentation list accepted; his
go on 2b CONFIRMED to the executor by the coordinator on 2026-09-26; `G1-PROGRESS.md` "Part 2b").
**PART 2c LANDED 2026-09-26, BUILD-GATED** (built in an isolated worktree in parallel with T3,
DIAG1 and COMP-a — jal's "start all three"; `G1-PROGRESS.md` "Part 2c"): `src/main.cpp` a
dispatcher, the invented X-drive a `constexpr RobotConfig` on `robot::Robot`, the state screens
moved out, the build gate's differential retired for the beacon over one chain and a program check
from each package's symbol table. **Its final run** (the team lead's rulings of 2026-09-26 ~13:15)
moved the one chain into `src/build_selection.hpp`, picking the program AND the robot's tables (no
`#if` in either table; a robot beacon asserted per build; no program may name another robot's table),
and made `-Wunused-function` a gate failure. **G1 is complete; nothing of it has run on a robot.**
*(This line read "PART 2a WAITS FOR JAL'S GO" until 2a landed, and "2b and 2c each wait for his go"
until 2b landed; the paragraph below is kept as it was written before the go.)*
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
  `src/programs/teleop_program.cpp` (deleted at T4 Part 3) built as provider → `bootConfig()` (SD record → typed table → refuse) →
  `robot::Robot` → the loop; the X-drive graph as a config constant; `src/main.cpp` a dispatcher.
- Rulings already made: only Teleop changes feel (shulib Drive and the tester keep `mapSticks` — the
  OLD shulib Drive; the new one, since T4 Part 3, drives through `DriveShaper` by robot two's driver file);
  field-centric ships selectable but OFF; the registry check and the builder's device pass both stay
  and name a port once.
- Done / docs owed: per the revised brief's DoD and documentation lists.
- Size: ~1–1.5 days executor + verification, across its parts.

**L2. [COMP] The competition program.** **Status 2026-09-30:** COMP-a LANDED (`b5aaf95`, host-proven only); COMP-t (the auton tester, L2t) LANDED (`0f56301`); **COMP-b BUILT 2026-09-30 in its worktree — host- and build-gated (the eighth build), NOT run on a robot; its first run is worksheet Station K** (`COMP-PROGRESS.md`, "Part b"; the coordinator's merge records its landing). *(2026-09-28: COMP-b's brief RE-CHECKED — RC1–RC12, all amended; jal answered ND1/ND2 at ~16:15, and ND3–ND6 were taken as recommended.)* BRIEFED 2026-09-24 (`docs/internal/chunks/COMP-competition-program.md`, two parts, each on jal's go; COMP-b after COMP-t — the order was re-ruled 2026-09-28, superseding the brief's first "after G1 Part 2b"); it did not exist in the plan or the tree before that (the G1 Part 2
brief names it as a future consumer of the builder's dead-port tolerance).
- What: the program uploaded for a match. `initialize()`: boot the robot (IMU calibration, port check
  against the SD record, status screen, refusal by name). A **brain-screen auton selector** (as built:
  the team's registered autons by name, two per page, from `src/team/autons.cpp`, and NONE — always
  present and the default; the choice is LOCKED when the auton starts). `autonomous()`: run the chosen routine under `sequence::RunGuard` with the match duration and
  the guaranteed end-of-run action (the park); both a `chassis::Routine` function and a
  `runner::PathRunner` route must be selectable (as built: a `Routine` runs; a route is refused by name
  until the team registers its commands — neither program passes a `CommandRegistry` yet). `opcontrol()`: the team's DRIVER FILE
  (`src/team/robot_two_driver.hpp`, T4 Part 1a — the COMP brief's B11 as replaced), after the auton
  task's terminal word. `disabled()`/`competition_initialize()`: one log line on entry and on return,
  and the screen handed to the program's own menu, which its auton task paints (RC3/RC7 — never the
  state screens). Handoff: anything running when a phase ends is cancelled into the safe state.
- **A wiring problem to solve in the brief — SOLVED at COMP-a** (`robot::BuildOptions::pacer`, which
  "shulib Competition" sets to its guard): `RunGuard` only guarantees its end-of-run action when
  it IS the chassis's pacer (`run_guard.hpp`, "`chassis` MUST be the one constructed with THIS guard
  as its pacer"), and `robot::Robot` built its `Chassis` with `devices.pacer()` with no way to
  take another.
- Why: every piece exists except the thing that ties them into a match. **It is also what lets R3b
  and M1 close**: R3b's DoD is "a v2 tank auton runs on the robot under the library's own steering";
  since COMP-t (the tester) and COMP-b (this program) two programs can command one — neither has run on
  a robot.
- Builds on: L1's provider/loop/`Robot`, `RunGuard`, `Routine`, `PathRunner`, `CommandRegistry`.
- Done: the EIGHTH build (`PROGRAM=competition`, a beacon, the gate updated — COMP-t took the seventh), the selector and the
  phase logic host-tested with fakes (phase transitions, cancellation, the park firing on a stalled
  loop), routines registered by the STUDENTS' code (the library provides the registration API; it
  ships only an example routine in the cookbook, not season content).
- Docs owed: a guide chapter "The competition program" (written from the code, examples compiled and
  quoted verbatim), README, roadmap (M4's guaranteed-park line), build-order entry + Next, the
  worksheet (a new station for a practice match), the changelog.
- Size: ~1 day. *(Stale since the briefs, corrected 2026-09-28: COMP-a has landed; COMP-t is ~1 day (rough),
  since its brief states no size; COMP-b is ~2¼ executor-days plus ~1 day of verification (its re-check,
  "The honest size of COMP-b now"). The practice match is worksheet Station K.)*
- **COMP-t — the auton tester, a part of L2 — LANDED 2026-09-29, host- and build-gated, NOT run on a
  robot** (`docs/internal/chunks/COMP-t-COMPLETED.md`): "shulib Auton Tester" (`make ROBOT=tank
  PROGRAM=autontest`, the seventh build; slot 7 until T4 Part 3, then 2) runs one of the team's autons
  on the bench from RUN on the brain's screen through the SAME `competition::runAuton` call
  "shulib Competition"'s `autonomous()` makes — only the trigger differs; any button, a touch, a controller drop or a
  disable aborts. It created the students' auton file `src/team/autons.{hpp,cpp}` (the 40 in auton,
  the acceptance run) that COMP-b calls, never copies (COMP brief B18/B30, amended 2026-09-28); COMP-b
  is the EIGHTH build. Its first run on robot two is worksheet Station M — the first time an uploaded
  program commands a `moveTo`; whether that log closes R3b is to be judged against R3b's DoD when it
  exists.

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
- The binding mechanism is the library's; which button does what is the team's. **The team lead's
  plan (2026-09-26): the claw pivots on ports 6 and 7, on R1/R2 and L1/L2** — after the brake-mode
  fix (DEFECTS2), which makes Hold reach the motor.
- **DONE FOR THE CLAWS (T3, 2026-09-26 — Part 1 the library and the claw table, Part 2 the Teleop
  wiring; host- and build-gated, NOTHING RUN ON A ROBOT):** hold-to-run and toggle bindings
  (`teleop/mechanism_binding.hpp`), `runDriverControl`'s bindings overloads, and "shulib Teleop"
  driving robot two's claws — R1/R2 port 6, L1/L2 port 7 (12 V while held, HOLD on release, both
  buttons of one pivot ⇒ hold), A toggles the pneumatic claw on 3-wire A (CLOSED at the start, kept
  through a disable). The team lead's rulings amended the stick rule: stick-interrupt applies to
  chassis MACROS only; for a mechanism button, releasing the button is the stop. And a claim an
  autonomous operation left on a bound mechanism is RELEASED by `runDriverControl` when driver
  control starts, and logged (`[WARN][TEL] driver control: RELEASED a claim ...`) — his ruling,
  T3 Part 2. **Still open here: chassis macros** (a button running a chassis motion the sticks can
  interrupt) and angle presets. Station J is where the claws are first seen. **Since T4 Part 3 (which
  took over Part 1b; landed 2026-10-08)** robot two's buttons are stated ONLY in its
  driver file (`src/team/robot_two_driver.hpp`, L5b): they left `src/robots/mechanism_table.hpp` with
  `kTeleopRezeroButton`, its button checks and Part 1a's interim tie; the macros, when they come, get their
  line in its WHICH BUTTON block and their binder in `src/common/driver_controls.hpp` (T4 brief §8; that file
  was deleted at MECH-A5, 2026-10-29 — a driver file's shared plumbing is `src/common/driver_file.hpp` since).
- **The re-zero default, and what guards it now:** `teleop::DriverLoopConfig::rezeroButton` still
  DEFAULTS to `L1` (pivot 7's forward button). What stops the collision running, since MECH-A5
  (2026-10-29): a driver file whose controls take its `kLoop`'s re-zero button does not BUILD
  (`src/common/driver_file.hpp`'s `static_assert`, the refusal harness's n57), and
  `runDriverControl`'s service overload refuses it again before its first tick
  (`mechanism::DriverInputs::avoids()`); T3's `teleop::validBindings()` still guards the superseded
  bindings' overloads (until MECH-A5 robot two's driver file went through it, its `bind()` validating
  the whole set). Robot two's driver file names **Y** in its `kLoop` (the team lead's ruling: A became
  the claw's toggle) ("shulib Teleop" named Y itself, beside the table's `kTeleopRezeroButton`, until
  T4 Part 3 retired both). The default itself
  was NOT moved or made a sentinel — a program with no bindings still gets L1, harmlessly.
- Docs owed (for the macros): guide chapter section, cookbook, roadmap Phase T rows, build-order.
- Size: the macros ~½–1 day.

**L6. [NEW-WALLRESET] Re-seat the position estimate against a wall or goal.** *(2026-10-22: superseded by its brief,
`chunks/L6-wall-reset.md` — answered "Accept all"; the plan below is kept as history: a SIDE sensor re-seats against a
wall the robot parked beside, it does not "drive toward" one.)*
- What: a primitive "drive slowly toward a known wall until the distance sensor reads X, then set the
  estimate's coordinate on that axis from the known wall position and the sensor offset." Never touch
  heading (IMU-owned).
- **Three gaps found by the audit:** (1) there is **no one-axis setter** — `Localizer::setPose` moves
  x and y together (and re-seeds both), and `Chassis::setPose` (F6-frozen) takes a full pose; a
  one-axis re-seat is an additive member (on the facade it is an F6 additive → a minor bump). (2)
  **Nothing detects a drive stall on these robots** (the odometry stall check has no independent
  source), so the stop is the distance reading, with motor current/velocity as a secondary stop the
  chunk must build. (3) `IDeviceProvider` had no `distance()` member when this was written; since
  MECH-A1 (2026-10-08) it has one (a mechanism's piece sensor), which this chunk can ask for its own port.
- Why: with drive-encoder odometry the estimate drifts with every skid; this is the cheapest,
  field-independent correction a tank auton has.
- Done: host-tested on the plant with a synthetic distance channel derived from truth; a routine step
  usable from `Routine` via `then()` or as a runner built-in (adding a built-in id is additive under F8).
- Docs owed: cookbook recipe, guide, HA entries (sensor offset, wall positions), roadmap, changelog.
- Size: ~1 day (it was ½ before the gaps were found).

**L7. [R4 + R5 + E5, reframed] Gains and noise as self-run calibration routines inside shulib
Calibrate.** **BRIEFED 2026-09-26 as chunk R5a** (`chunks/R5a-gains-and-noise-on-the-sd-card.md`; every recommendation accepted, Q7 ruled: the SD card). **Part A BUILT 2026-09-29 (reviewed 2026-10-04) — the measurements as library, host-proven only; no program runs them; nothing measured** (`chunks/R5a-PROGRESS.md`); Parts B–D not started. **The plan already has this work** — R4 (IMU bias/drift), R5 (the on-robot ramp routine),
E5 (calibration routines + persistence to SD) — so it is not new; what is new is the framing jal
agreed to on 2026-09-18 ("calibrate absolutely everything it needs to know"): **R5's DoD says the gains
are committed as constants, and this persists them to the SD card at runtime instead.** That departure
is a plan change for jal to confirm in the brief (Q7).
- What: a voltage-ramp routine per side that fits feedforward kS/kV/kA (replacing HA-45's
  placeholders); a stationary IMU drift/noise capture; loop-timing capture. Persisted in the SD record
  (a record v2 with its version bump and a decoder that still reads v1, or a sibling record — decide in
  the brief) and consumed by the builder as the tuning.
- Done: host-proven on the plant (the fitted gains recover the plant's configured gains); the record
  round-trips; "shulib Drive" (Teleop until T4 Part 3 retired it) and the competition program use
  measured gains when present and say which.
- Docs owed: HA-45/50/51/52 and the noise entries' settle lines, roadmap R4/R5, build-order R4/R5/E5,
  guide, worksheet station.
- Size: ~1–1.5 days host; the measurement is S2/S3. *(Stale, corrected 2026-09-28: R5a's brief puts it at
  ~3½–4 executor-days across parts A–D, plus verification and the robot sessions.)*

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

**L9. [NEW-ERGONOMICS] The small API additions the readable routine depends on.** **BRIEFED 2026-09-26** (`chunks/L9-rest-reverse-driving-and-one-line-boot.md`: `reverseTo` / `faceAway`, API 2.3 *[2026-09-28: TANK1 Part A took 2.3, so L9 Part A takes 2.4]*; the boot as library code; T3 already delivered the mechanism lookup).
- Reverse driving: **the facade has no `driveTo`** (it is `Routine`'s, F10-frozen), and the data path
  already has it (`Waypoint::reverse`, which the runner honours). So the work is an ADDITIVE option on
  `Routine::driveTo` — a `kApiMinor` bump, pins updated deliberately, a changelog entry — following
  `include/shulib/version.hpp`'s policy exactly. *(L9 Part A1, 2026-09-29, built as the brief's D1 (a) —
  two new steps, `reverseTo` / `faceAway`, API 2.4 — not an option on `driveTo`; "which the runner honours" had
  been shown for a reversed APPROACH (arriving at bearing + π) on an X-drive only, tank along-axis reverse legs
  were already tested at C1, and L9 adds the first reversed approach on a tank — through `faceAway`, steered by
  TANK1 — both as a recipe and as a route, bit-identical; [live log](chunks/L9-PROGRESS.md).)*
- A named-mechanism lookup on `robot::Robot` (`robot.mechanism("intake")`): **DONE at T3 Part 1
  (2026-09-26)** — `MechanismConfig` gained a cartridge, a safe brake mode and a pneumatic START and
  SAFE level (each with an UNSET value `valid()` refuses); `Robot` builds a `MotorMechanism` or a
  `PneumaticMechanism` per configured mechanism (ABSENT, never a whole-build refusal, when the
  provider refuses one) with typed lookups `mechanism(name)` / `pneumaticMechanism(name)` and the
  RunGuard span; the provider gained `digitalOut()`. Since T3 Part 2 the boot source ("shulib Teleop"'s
  then; `src/common/boot_source.cpp`, shared by "shulib Drive", the Auton Tester and the competition
  program, since T4 Part 3 retired Teleop) converts `src/robots/mechanism_table.hpp` into those configs,
  so robot two's claws are built from it.
  Host-tested only.
- A `Robot::fromCard()`-style one-liner (may be L1's `bootConfig()` exposed; decide in L1/L2).
- Docs owed: the register rows touched, the changelog, guide, cookbook.
- Size: ~1 day. *(Stale, corrected 2026-09-28: the brief puts it at Part A ~½–1 day plus Part B ~1 day, so
  ~1½–2 days.)*

**L9b. [SAFE-SAVE, NEW 2026-09-26, reactive] Calibrate's save can never erase a good record; the boot falls
back to a verified backup.** BRIEFED (`chunks/SAFE-SAVE-calibrate-save-and-teleop-label.md`; every
recommendation accepted). The brain cannot rename or delete a file, so today a failed save can erase the
record the programs boot from ("shulib Drive" since T4 Part 3, Teleop until then; the Auton Tester and
the competition program too) while its NOT SAVED text says the old one survived. **Until Part B1 is in the
build: copy `shulib_drivetrain.bin` off the card on a laptop, dated, before ANY Calibrate run** (worksheet
G.0.1). Part A (the library) now; B1 (Calibrate's save) after T3 Part 2 and 2c-final, before the next
Calibrate rerun; B2 (the boot's fallback, and the boot-source words Teleop said — "shulib Drive"'s and
the competition program's since T4 Part 3) after COMP-b. **Status 2026-09-29: Part A BUILT
in the `safesave-a` worktree, host- and build-gated, awaiting the coordinator's verification** (the store
seam, `ProsFileStore`, `FakeFileStore`, `calibration/record_store.hpp`, HA-166; `chunks/SAFE-SAVE-PROGRESS.md`)
— no program uses it yet, so the copy-first rule above still stands. The boot source's "table" label
(Teleop's when it was fixed) was already fixed at G1 Part 2b (D3) — a regression pin only. ~2–2½ days in all.

**DIAG1 Part B** (the median's non-finite handling) LANDED 2026-10-01 — **DIAG1 is COMPLETE**
(`chunks/DIAG1-COMPLETED.md`; brief `chunks/DIAG1-partB-median-nonfinite.md`); host test plant only.

**L2t. [COMP-t, NEW 2026-09-27] The auton tester.** **LANDED 2026-09-28, host- and build-gated, NOT run on a
robot** (`chunks/COMP-t-COMPLETED.md`; brief `chunks/COMP-t-auton-tester.md`; jal: D1–D9 as recommended).
"shulib Auton Tester" runs a registered auton on the bench through the same `runAuton` path `autonomous()` uses
— touch RUN, a 3 s countdown, any button aborts — and created the students' auton file
`src/team/autons.{hpp,cpp}` with the 40 in auton. Slot 7 until T4 Part 3, then slot 2. Built after TANK1 Part A,
its predictions re-made from TANK1 (the re-predicted table re-run on the built code: byte-identical). Its first
robot run is worksheet Station M (Part S, S1b).

**L2u. [TANK1, NEW 2026-09-27, reactive] `moveTo` on a tank steers back onto its line.** **COMPLETE
2026-09-29** (`chunks/TANK1-COMPLETED.md`): Part A LANDED 2026-09-28 (`3965791`, API 2.3); Part B (the ` xt`
column, no API bump) LANDED 2026-09-29. Host- and build-gated, NOT run on a robot (COMP-t's Station M is the
first). BRIEFED (`chunks/TANK1-tank-moveto-steers.md`; jal's go ~08:17, D1–D4 as recommended); API 2.3 (L9 Part A
then takes the next free minor, 2.4). **IMU1** (the IMU's yaw rate read once per tick) is
a separate small chunk from its D4. On
today's tank `moveTo` nothing corrects a sideways drift, so the 40 in auton on a robot with one side a few
percent weak ends MOTION_TIMEOUT (COMP-t's probe; robot two's sides differ by ~9 %). jal: "Fix the library
first" — a bounded correction that never plans a turn, logged when it steers, the X-drive and H-drive
byte-for-byte unchanged, the auton line unchanged. HA-179/180. **Part A LANDED 2026-09-28 as `3965791`**
(`chunks/TANK1-PROGRESS.md`, Part A and the coordinator's merge record: the steering, TSTR, the Warn, 40 new test
cases after the 2026-09-28 review added the bend's lead, API 2.3, HA-179/180); its build found that the
Localizer reads the IMU rate twice at every chained-motion boundary (IMU1's, not TANK1's; TANK1 immune — corrected
2026-10-22: not immune on robot two, whose record read would damage the Localizer's rate (IMU1's brief §0.2;
derived from the adapter's code and PROS's documented 10 ms copy, HA-213 — never measured); IMU1 Part A BUILT
2026-10-29, `chunks/IMU1-PROGRESS.md`). **Part B LANDED
2026-09-29** (`chunks/TANK1-PROGRESS.md`, Part B, its review findings and the coordinator's merge record: translating
tank result lines end with ` xt`, the final cross-track error; X/H lines byte-identical).

**L5b. [T4, NEW 2026-09-26] The driver program.** BRIEFED (`chunks/T4-driver-program.md`); Part 0 (the slots
and the plan) landed 2026-09-27. **Part 1a LANDED 2026-09-27, host- and build-gated, NOT run on a robot; run
by "shulib Competition"'s `opcontrol()` since COMP-b (built 2026-09-30, not run on a robot)** (`chunks/T4-PROGRESS.md` "Part 1a"): robot two's driver file `src/team/robot_two_driver.hpp`
and its generic half `src/common/driver_controls.hpp` (deleted at MECH-A5, 2026-10-29) (the binder by name, the five checks, the ONE heading rule
— Teleop and the X-drive called it then; the driver file and the X-drive since Part 3); the buttons also
stayed in `src/robots/mechanism_table.hpp`, tied at compile time in Teleop, until Part 3 moved them into the
driver file (Part 1b was folded into Part 3 on jal's ruling of 2026-09-28: Teleop was never rewired); COMP-b's B11 replaced ("`opcontrol()` IS the driver file"). Parts (COMP-b) → 2 (the old drive's
protections, moved into the library) → 3 (the new "shulib Drive" replaces the old one; "shulib Teleop" retires;
the buttons leave the mechanism table). jal's answers:
N1 (a) the over-current cut ON with an off switch and a required push check; N2 (a) as amended; N4 (a) the
auton tester in slot 7 until Part 3; N5 (a) an optional SD copy of every library program's log; N7 (c)
driving stops mid-match only when 3+ drive motors are dead in total. N3 and N6 as the brief recommends.
**Re-checked 2026-09-28** (Parts 2 and 3, against `b81a590`: thirteen findings plus a fourteenth, major, all
amended); jal's answers of ~16:15, all as recommended: C drops the velocity-exactly-0 check, D adds guard G4,
A limits the SD copy to events and summaries, and B has the competition program keep an appended SD copy.
**Part 2 is ~2–2½ days after COMP-b; Part 3 is ~3–3½ days after Part 2.** *(2026-09-29: Part 2 was SPLIT on
jal's approval of 2026-09-28 ~17:35. **Part 2a — the library half — LANDED 2026-10-01, host- and build-gated, NOT
run on a robot** (`chunks/T4-PROGRESS.md` "Part 2a"): `runDriverControl` drops a drive motor whose readings
freeze (250 ms), refuses at 3+ dead in total, and cuts for 1 s on a fight or 2.4 A for 250 ms, for EVERY caller —
so COMP-b's `opcontrol()` has them from its first build; HA-175/176 registered. **Part 2b** (the PROTECTION row,
COMP's `perTick`, the driver file's paragraph, Station K's push check unless COMP-b writes it, the re-read of
COMP-b's files) was FOLDED INTO PART 3 on jal's ruling of 2026-10-01 — Part 3's late step, after COMP-b. Part
2a's review (2026-09-30) and its verification (2026-10-01) found guard G4's gap at a wall wider than decision D
weighed — at ANY wall a motor whose speed report settles well before its mates' is marked dead (P2-19e), and
beside the slipping port 18 less is needed (P2-21d) — pinned as built, stated in HA-175, jal's to rule
(`REMAINING-WORK.md` §11, "D, again"); and the push check gains a full-stick reversal and a launch from rest
(HA-176), its log read for `went ABSENT` on any port. **"D, again" ANSWERED 2026-10-01: the encoder check**
— a side-mate counts as evidence only while its ENCODER moved in the last 5 reads; BUILT in Part 3's early step
(`chunks/T4-PROGRESS.md` "Part 3"): the any-wall case marks nobody now (P2-19e re-pinned), and a hard wall push
whose reports settle unevenly gets one 1 s fight cut instead; port 18's gaps (P2-21c/d) stay until its gear is
fixed.)*
- **Part 3 LANDED 2026-10-08 — T4 COMPLETE** (built in both steps 2026-10-04/06; reviewed 2026-10-07; the
  review's and the fresh verification's findings fixed 2026-10-08; verified by the coordinator) (`chunks/T4-PROGRESS.md` "Part 3"): the new "shulib Drive" (MAIN / DETAILS / MORE through COMP-b's
  painters, the touch page flip, the controller's lines through the ONE chooser), "shulib Teleop" retired
  (`PROGRAM=library` an `$(error)`; seven builds), the Auton Tester in slot 2, the buttons in the driver file
  only, Part 2b (the PROTECTION row and the chooser's REFUSED / CUT / dead words, COMP's four-argument
  `perTick`, the driver file's protections paragraph, Station K's push check, K.8), the SD log copy for
  "shulib Drive" (`drive_log.txt`) and the competition program (`comp_log.txt`, appended), and jal's C5
  ruling (2026-10-02: one refused auton entry is DROPPED, never the boot — `competition::OnRefusal::DropEntry`,
  in the competition program; the Auton Tester does the same — the coordinator's extension of the ruling,
  confirmed by jal 2026-10-08). The review's fixes (2026-10-08): a slow loop says
  `LOOP_OVERRUN` once per EPISODE (`FaultLatch::raiseRepeat`, `LoopMonitor::overrunEpisodes`), the SD copy has
  one writer at a time, a boot refused after the provider is built says why in the copy, and "shulib Drive"'s
  row building and generated boot lines moved into `src/programs/drive_panel.hpp`'s host-tested functions.
- For SAFE-SAVE B2's re-check: its `BACKUP see brain` word goes into the shared chooser
  (`src/common/driver_panel.hpp`), at its place after CLAW ABSENT (`teleop_program.cpp` is deleted).
- For SAFE-SAVE A/B1's re-check: `formatSaveReport`'s sentences name no drive program (SAFE-SAVE A wrote them
  that way); its two COMMENTS that named Teleop were re-worded at T4 Part 3.
- R5a Part D's and L9 Part B's callers are now "shulib Drive" and the competition program (they named Teleop);
  their briefs are re-read at their go, not edited now.

**L10. The remaining plan chunks (lower priority for the season):**
- **[G3a — NEW 2026-09-26, before G3] The robot profile, its generator and an optional VS Code robot
  panel** (jal: "lets do it for before implementing .vexbot"; build-order's G3a entry). ~3–5 days.
  *(Stale, corrected 2026-09-28: its draft brief of 2026-09-27 puts it at ~5½–6 days; it waits for jal's go
  and his answers to D1–D6.)* *(2026-10-29: amended for the MECH core; D1–D6 answered 2026-09-29 and D6′–D10 2026-10-29, all as
  recommended — the brief is `chunks/G3a-robot-profile-generator-panel.md`: two lanes, Parts 1–6 ~17¾–23¾
  builder-days, ~11–15½ to a robot made and driven in the panel, everything before G3 ~24¾–34¼ (rough); by jal's ordering ruling of 2026-10-29 its
  foundation — the robot file and its generator — starts when MECH-A5 merges, beside VIS1, L6 and the
  auto-flipper, and DOCS3 follows the panel; build-order's G3a entry.)* *(2026-10-29, later: jal's PANEL SPRINT —
  the panel by 2026-11-21; Part 1 building beside MECH-A5, the builds overlapping; the camera's later runs,
  ALLY1, L4 and AF1 after the panel.)* *(2026-10-30: the evening amendment answered — the feel, drive tuning, the
  full route maker (+ STEP1), the wall and camera pages, release-ready; D16 a true curve follower, CURVE1, after
  the route maker; publisher `shulib`; MECH-S and TUNE1 ruled, briefs next.)*
- **[G3] `.vexbot` ingestion into G3a's generator → freeze F7.** Serves VexBuilder's files. ~1 day
  after G3a.
- **[G4] the ten-minute guide** (closes M5). The legacy `.shupaths` importer was DROPPED 2026-09-26
  (jal: "shupaths probably isn't going to be a thing"). ~½ day shulib side.
- **[H4 — NEW 2026-09-27, after G3a] The robot console:** the log and the brain's screen on the computer
  over USB, remote touches back, on H1's wire; a bench tool, inert under field control.
- **[PKG1 — NEW 2026-09-27, LAST] The PROS template and the starter project** (jal: "after everything like
  after everything"); G4's guide is written from it, so G4 goes last with it.
- **[H1] SHUL/2 telemetry wire → freeze F9.** BRIEFED 2026-09-26 (`chunks/H1-shul2-wire.md`; parts A1, A2, B; in this wave, last when slots are short). ~1–2 days. *(Stale,
  corrected 2026-09-28: the brief gives A1 ~1–1½, A2 ~1 and B ~½, so ~2½–3 days.)*
- **T2 finish:** re-point the tester's DRIVE station at `DriveShaper` AFTER a driver has compared
  feels on hardware (the G1 Part 2 brief's ruling M4) — the new "shulib Drive" already drives through
  it (robot two's driver file, since T4 Part 3), so the station is all that is left — then
  **`T2-COMPLETED.md`**, which `T2-PROGRESS.md` owes.

### Part Q — decisions only jal and the build team can make (code waits on these)

- **Q1. The mechanisms on each robot:** intake type, lift type and levels, clamp (yes/no, pneumatic or
  motor), every motor port and sign, every pneumatic port, and how the lift homes (limit switch vs
  stall — owed since 2026-08-13). **Blocks L8, L5's meaning, and every scoring routine.**
  **ANSWERED IN PART (the team lead, 2026-09-26), for ROBOT TWO:** two claw PIVOTS, RED, on ports 6
  and 7 (R1/R2 and L1/L2, 12 V, holding on release), and a PNEUMATIC claw on 3-wire port A (A
  toggles it, CLOSED at program start, keeps its state when disabled) — all REPORTED, none seen by a
  census yet (A4 register HA-153, HA-159); the pivots' physical directions, a loaded Hold, the
  claw's single/double solenoid and its polarity are UNMEASURED (worksheet Station J). Still open:
  every other mechanism on robot two (intake? lift?), everything on robot one, and the homing.
- **Q2. Sensor placement — PARTLY ANSWERED 2026-10-08** (the team lead, for the auto-flipper question: "we are going to
  have two distance sensors one on each side and the ai vision sensor in the front"): the AI Vision camera faces FORWARD;
  two distance sensors, one on EACH SIDE (facing left and right). Still open: which robot(s), the measured mount offsets
  and heights (lever arms), and whether a GPS is bought and where it sits.
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
  committed as constants (R5's DoD as written)? **RULED 2026-09-26 (jal): ON THE SD CARD** — Calibrate measures and saves, programs load at boot; the brief is `chunks/R5a-gains-and-noise-on-the-sd-card.md` (chunk R5a).

### Part S — sessions with the robots, in order (none can be done at a desk; each produces evidence)

Rules for every session: brain on USB via a data cable (`pros lsusb`), SD card in, battery checked, the
program started from the brain screen (never the controller menu), the user port captured raw to a file,
**everything the robot printed recorded in the relevant `-PROGRESS.md` as evidence**, no claim without it.
**Build the packages on this laptop from the committed tree** — do not `git pull` (the remote is
behind) — and, if an executor is working in the main tree, build in a separate worktree at the commit
you mean so its in-progress edits cannot reach a package.

- **S1. The first drive** — worksheet **Station G** (`docs/internal/chunks/R3a-BENCH-WORKSHEET.md`,
  from line ~597) is the runbook: rebuild Bench Tests, Calibrate and "shulib Drive" (Teleop until T4
  Part 3 retired it; the hardware checkout holds only the OLD Drive) → Bench Tests census + MOTOR WATCH on the rebuilt drivetrain → shulib Calibrate
  wheels-up (expect the yaw-floor refusal — that is correct) → Calibrate on the floor (push
  front-first, answer the prompts, tape push, spin — 3 m clear, a second person at the battery) →
  *(Superseded in part by R3d Part 2F-1, 2026-09-24: the calibrator now takes ONE 48 in push — no
  separate tape push, no distance question — and a wheels-up run now refuses at the first port as
  "rolled / not on the floor" after a 3 → 6 → 9 V ramp, not at the yaw floor. Station G was rewritten
  from the new program at 2F-2 — the wheels-up run is now OPTIONAL; follow Station G, not this
  line.)* →
  `SAVED` read back → a second Calibrate run for reproduction → shulib Drive from the record (shulib
  Teleop until T4 Part 3), wheels
  up then floor. **What it closes:** R3d (on the reproduction run). **What it cannot close:** R3b and
  M1 — no program S1 runs commands a `moveTo`, so the "few inches of `moveTo`" this file used to list
  is not in S1. The first uploaded program that commands one is "shulib Auton Tester" (COMP-t, part of
  L2; built 2026-09-28, not yet run) at S1b below — whether its floor runs close R3b is a ruling
  against R3b's DoD when its log exists, not claimed here. Then write
  `R3d-COMPLETED.md` from the log, move the roadmap pointer and the Next block, flip only evidenced
  checkboxes. **The "never driven a robot under its own control" sweep** (`docs/internal/guide-maintenance.md`'s "R3 — first
  real robot run" row) happens when the log shows the library's MOTION STACK drove — every instance
  in ONE commit: `README.md`, `docs/README.md`, guide ch. 01, 07, 14 and the guide README, the master
  plan's status line, `docs/vexbuilder-integration.md`. "shulib Drive" driving from the record (Teleop
  until T4 Part 3) is the library driving open-loop through its own graph — say exactly that, not more.
- **S1b. The 40 in auton on the bench — worksheet Station M** (COMP-t, "shulib Auton Tester"; after
  S1's Calibrate rerun if the team wants a truer 40 in — copy the record off the card first, G.0.1):
  the forward check (EST x must RISE — a STOP condition), the aborts wheels up, a wheels-up full run,
  then three floor runs on the tape with serial captured (the RESULT word, END x and y, the faults
  line, the TSTR count, the tape along and beside the line — HA-179, HA-180), the SD log across a
  reboot (HA-177), and the abort/RUN-gate reads (HA-178). It comes BEFORE COMP-b's practice match
  (Station K), which still owns the field disable, the Timed Run and the hand-over to driver control.
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
4. **The mechanism chapter** (L8): each primitive, its confirmation and failure modes, homing. *(Begun:
   the guide's chapter 18, MECH-A3, 2026-10-15 — the kinds, homing, limits, jam signatures, tuning; MECH-A4,
   2026-10-22, the auton steps, the driver controls, the route markers and the direction check's judgement; A6
   (built 2026-10-30, in review) added the bench page's section.)*
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
   the same day** (host-only); **Part 2b landed 2026-09-26** (build-gated; this version of "shulib
   Teleop" has not run on a robot). **jal's order of 2026-09-26: the brake-mode fix (DEFECTS2 —
   Hold and Brake reach the V5 motor; Teleop coasts on stick release), then T3 (the claw pivots on
   ports 6/7, R1/R2 and L1/L2 — L5), then G1 Part 2c, then COMP,** each briefed, shown to him and
   started only on his go. **DEFECTS2, T3 (both parts — the claws now driven by "shulib Teleop") and G1
   Part 2c have landed, host- and build-gated, not run on a robot — G1 is COMPLETE. SRC1 (`src/`
   organized) has landed, and so has T4 Part 1a (the driver file, 2026-09-27; the competition program
   runs it since COMP-b).
   TANK1 Part A (`moveTo` on a tank steers back onto its line — jal: "Fix the library first") and COMP-t
   (the auton tester, with the 40 in auton, 2026-09-28) have landed, host- and build-gated, not run on a
   robot; TANK1 Part B (the ` xt` column) is being built alongside. Next: COMP-b on COMP-t and T4 Part 1a
   (it takes the students' auton file and the run pieces from COMP-t — B18/B30), then T4 Parts 2 and 3
   (Part 1b folded into 3, jal 2026-09-28). The tester's first robot run is Station M (S1b). The sensors (L3 the AI Vision adapter, L4 piece targeting, L6 the wall reset) come after G3
   (jal, 2026-09-28). jal pushes the branch
   once the auton is complete, before G3a.** *(Status 2026-09-28: TANK1 Part A landed `3965791`; COMP-t and
   TANK1 Part B are building; COMP-b's and T4 Parts 2–3's briefs are re-checked and carry jal's answers of
   ~16:15. The readable list of everything left is [`REMAINING-WORK.md`](REMAINING-WORK.md).)* *(G1 Part 2c was built in parallel with T3 on his "start all three"; its re-check of the
   brief against the tree is `G1-PROGRESS.md` 2c.1.)* The claws' first sight is worksheet Station J
   (after Station F).
2. **The moment the brain enumerates: S1** — stop other work and do the first drive (Station G).
   Nothing is worth more. It needs nothing from L1.
3. Brief → go → build, one at a time: **L2** (it is also what lets R3b/M1 close), **L3**, **L6**,
   **L7**, **L5**, **L9**, then **L4**. Ask jal for **Q1–Q7** early so **L8** is not the thing that
   stalls.
4. Sessions S2, S3, S6 as the robot is available; S4 after L8; S5 for robot one; S7–S9 on the field.
5. P1, P2, then the documentation set in §6 checked item by item; L10 when jal wants it.

**Do not** tell jal the whole library can be finished by the 2026-09-26 weekend. Tell him, with evidence,
what is proven at each step.
