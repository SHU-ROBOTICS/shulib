# Everything left — the full list, in order

> **Kept current by the coordinator at every milestone** (the team lead, 2026-09-28: "please keep this list up
> to date and add more details to it"). The status board (the shulib Status Board artifact) carries an
> easy-to-read version. The plan of record is [`build-order.md`](build-order.md) and
> [`HANDOFF-remaining-work.md`](HANDOFF-remaining-work.md); this list is the readable view of them. Where this list
> and the plan disagree, the stale side is corrected in the same commit: the plan when a ruling has not reached it
> yet, this list otherwise. Anything the plan does not carry yet is tagged **NEW 2026-09-28 — PROPOSED, planning with the
> team lead, not yet in the plan** until he rules and the plan is amended.
>
> **Sizes** are builder-days (one executor) taken from the chunk's brief where a brief exists. Every size without
> a brief is marked **(rough)**. **Verification is in no row.** The reviewers, the fixes and a fresh verifier are
> counted once, under the total. **Robot-session-bound** means the item closes on evidence from the robot rather
> than on builder time, so it is not in the total. **Last updated: 2026-10-02 (COMP-b's commit, on `7daf809`).**

**Where things stand:** 34 of the phase table's 63 chunks are complete (COMP on 2026-10-02), and R2a (half of R2) has
also landed; 29 chunks are open. Landed most recently: L9 Part A1 (backing into a point in a recipe, 2026-09-30); DIAG1
Part B (a reading that is not a number left out, completing DIAG1), H1 Part A1 (the SHUL/2 wire's read side) and T4
Part 2a (the drive protections) on 2026-10-01; and **COMP-b, the competition program** (2026-10-02, completing COMP).
Being built: T4 Part 3's early step (since 2026-10-01). In review or fixes: SAFE-SAVE Part A and R5a Part A. The MECH
core's brief is being written; G3a's amendment waits for your D6′–D10.
Nothing new has run on the robot since 2026-09-25. **The 40 in auton is ready for you on the bench** (#4).

---

## 1. The auton — in progress (the team lead pushes the branch after this, before G3a)

| # | Item | State | Waits on | Size |
|---|---|---|---|---|
| 1 | **TANK1 Part A**: a tank's `moveTo` bends back toward its line (≤ 15°, a 0.06 s look-ahead) and cancels the yaw a weak side makes. X/H drives unchanged | ✅ landed `3965791` (API 2.3) | — | — |
| 2 | **COMP-t**: "shulib Auton Tester" (pick an auton, touch RUN, 3 s countdown, any button aborts; it runs the same `runAuton` path `autonomous()` uses) plus the students' auton file `src/team/autons.{hpp,cpp}` with the 40 in auton | ✅ **landed 2026-09-29** (host- and build-gated, seven builds; the coordinator's ten safety mutations all red; **never run**) | — | — |
| 3 | **TANK1 Part B**: the sideways-miss number (` xt`) on every tank result line; no API bump | ✅ landed 2026-09-29 — **TANK1 COMPLETE** | — | — |
| 4 | **You run the 40 in on the bench**: worksheet Station M. Wheels up first, then the floor and the tape, with serial captured. Its `imuYawRate` column does not count as evidence until IMU1 (#32) lands | **ready — needs you at the robot** (upload with `rm -rf bin .d` first; copy the calibration record off the card first) | — | ~1 hour |
| 5 | **COMP-b**, the competition program: `autonomous()` runs the chosen auton on the program's own task under the guard; `opcontrol()` runs the driver file; the brain-screen auton picker; slot 6; the eighth build | ✅ **landed 2026-10-02 — COMP complete** (host- and build-gated, NOT run on a robot; Station K is its first run). Your C5 ruling (one bad auton entry is dropped, never the boot) is built in T4 Part 3's late step | — | — |
| 6 | **Station K**: a practice match with the controller's Timed Run (does Timed Run start `autonomous()`? HA-162). B6 is decided after it (§11) | needs you at the robot | 5 | ~1 hour |
| 7 | **R3b + M1** close when "a v2 tank auton runs on the robot under the library's own steering" (R3b's DoD) and M1's DoD holds. Both are judged against Station M's and Station K's logs. They also settle HA-18, the threshold half of HA-52 and the "drivable" half of HA-112 | open since June | 4, 6 | robot-session-bound |

## 2. The driver program (T4) — Part 2a before COMP-b; Part 3, with Part 2b's items, beside it and after it

| # | Item | State | Waits on | Size |
|---|---|---|---|---|
| 8 | **Part 1a**: `src/team/robot_two_driver.hpp` (which button does what, the feel) + `src/common/driver_controls.hpp` | ✅ landed `8d89df7` | — | — |
| — | **Part 1b** was SKIPPED on 2026-09-28 by your ruling and folded into Part 3, so Teleop is never rewired | — | — | — |
| 9 | **Part 2**: the old "shulib Drive"'s protections move into the library for every program. Driving continues with dead drive motors (it stops only when 3+ are dead). The FIGHT and OVER-CURRENT cuts are added; over-current is on by default, with an off switch and a push check before any match | **Part 2a** (the library half) ✅ **landed 2026-10-01** (host- and build-gated, NOT run on a robot; COMP-b's `opcontrol()` gets it from its first build); **Part 2b** (its `src/` wiring) FOLDED INTO PART 3 on your ruling of 2026-10-01 — Part 3's late step | — | — |
| 10 | **Part 3**: the new "shulib Drive" replaces the old one (slot 1). "shulib Teleop" retires. The buttons leave the mechanism table (Part 1b's work). Each program can keep an optional SD copy of its log (events only). `main.cpp` reaches its final form. **Since 2026-10-01 it starts with Part 2b's wiring** (the PROTECTION row, `perTick`'s fourth argument, the re-read of COMP-b's files). **Since 2026-10-01 it starts EARLY**: the items that need nothing of COMP-b are built beside it, the rest after it lands, then one review round | brief re-checked (as above); your A/B answered | Part 2a (early step); COMP-b (late step) | ~3½–4 days |
| 11 | **Stations F (drive) and J (claws) re-run on the new "shulib Drive"**: T4's DoD, and nothing of T4 is claimed on hardware until these logs exist | needs you at the robot | 10 | ~1 hour (rough) |

## 3. Mechanisms (MECH) — NEW 2026-09-28 — the order is ruled, the list is COMPLETE (2026-09-29), the core's brief is being written

The requirements are in [`MECH-PANEL-requirements.md`](MECH-PANEL-requirements.md). That file is planning, not a
brief; the team lead declared the list complete on 2026-09-29, and the core's brief is being written. In short: **any
mechanism is described as data, bound to ports after it is created, and used the same way in autons, the driver
file and the panel.** PTO-readiness is one of its principles (P7); how it will be met is still open.

**The team lead's ruling (2026-09-28, ~17:15):** "lets do this so we have it, we can do the rest after the panel"
— **the mechanism CORE is built first, before the panel; everything else comes after the panel** (and before G3,
so the robot-file format is locked with VexBuilder once, with the whole vocabulary — the coordinator's placement,
open to his objection).

| # | Part | What | When | Size (rough, unmeasured) |
|---|---|---|---|---|
| 12 | **MECH core** | the four control kinds — **power** (with done-conditions), **position** (degrees or inches, non-linear lifts, smooth moves, gravity compensation, travel limits, homing by switch or stall), **speed** (flywheels, "at speed"), **pneumatic** (incl. three positions); **port binding** (roles instead of ports, the wiring binding validated every boot, one boot line per role); **auton steps** (and the auton's mechanism lookup, run while driving); **presets** in the driver file; the per-tick mechanism service everything depends on; the device provider's distance / optical / digital-in accessors; arm, lift and flywheel models in the host test plant; robot two's claws as the proving case | after T4 Part 3, **before the panel** | ~10½–18 days (+1½–2½ if PTO support is built now) |
| 13 | **MECH, the rest** — protection and more kinds | cycle mechanisms (catapult/puncher); piece counting; colour sorting; jam auto-reverse; interlocks; loaded gains; low-power hold; current caps and heat derating with a driver warning; tip protection | **after the panel** | ~7–11½ days |
| 14 | **MECH, the rest** — driver and auton extras | preset cycling, stick control with hold-where-released, long-press and double-tap, rumble, mechanism state on the controller screen, driver-interruptible macros (would absorb T3's open chassis macros and angle presets, #16); triggers partway through a drive | **after the panel** | ~5½–9 days (+~1 if routes are in scope) |
| 15 | **MECH, the rest** — setup and diagnostics | a mechanism page in Bench Tests; the direction check; guided port binding on the brain (and the SD binding record, if bindings live there); auto-tuning of an arm's gains; a per-match health log | **after the panel** | ~6½–11½ days |
| 15b | **The panel's extension for the rest** | the panel gains what the rest adds (templates such as Catapult and Conveyor, interlocks, the extra controls, the bench-page links) | after the rest | ~2–3½ days (rough) |

The sizes are the requirements' re-priced ranges (§7.1), still unmeasured until a brief is written. Its §5 holds
29 questions; the ones the CORE needs answered before its brief are marked there. The core starts **after T4
Part 3 lands** (Part 3 moves the buttons out of the mechanism table the core rewrites; running them together would
be an exception needing his explicit go).

### Plan chunks MECH would change (they stay open in the plan until he rules)

| # | Chunk in the plan | What is open | Waits on | Size | The proposal |
|---|---|---|---|---|---|
| 16 | **T3**: chassis macros and angle presets | a button that runs a chassis motion the sticks take back within one tick; angle presets (T3's DoD). The claws' part has landed | — | ~½–1 day (rough, HANDOFF L5) | NEW 2026-09-28 — PROPOSED, planning with the team lead, not yet in the plan: absorbed into MECH C |
| 17 | **F3**, the scoring primitives | `setQuadrantToggle`, `orientToScoringHalf`, `intakeUntilCapture`, `liftToLevel`, `rotateClampToAngle`, `clampActuate`/`clampConfirm`, `deployActuator`, each bounded and confirmed over the F1 seam | Q1 (the mechanism decisions) | ~2 days once decided (rough, HANDOFF L8) | NEW 2026-09-28 — PROPOSED, planning with the team lead, not yet in the plan: the generic pieces move into MECH; the rest is the students' season strategy (examples optional) |
| 18 | **F4**, skills motion + reference routines (closes M4) | `fieldCentricStrafe`/`strafeTrim`, `moveToPoseProfiled`, `buildStack`, `matchLoadCycle`, `endInMidfield`, `strategyMode` | F3; hardware + final mechanisms | not sized | NEW 2026-09-28 — PROPOSED, planning with the team lead, not yet in the plan: re-scoped as for F3 |

## 4. The robot panel and `.vexbot`

| # | Item | What | Waits on | Size |
|---|---|---|---|---|
| 19 | **G3a**, as briefed | the robot file (the `.vexbot` profile block itself); its generator into the C++ tables and the driver file's button region, byte for byte; the freshness gate; one validator; the optional VS Code panel (a form over the file, with the calibration record shown read-only) | T4 Part 3; the team lead's push; his go and his answers to D1–D6 (§11) | ~5½–6 days (its draft brief, 2026-09-27) |
| 20 | **G3a, the amended panel**: NEW 2026-09-28 — PROPOSED, planning with the team lead, not yet in the plan | a **new robot** (which creates its build too); **mechanisms** (add / remove / duplicate / reorder, with templates: Intake, Arm, Lift, Claw, Flywheel, Catapult, Conveyor, Blank); **wiring** (roles → ports, or "bind on the brain", with live warnings); **controls** (every MECH C control, with live warnings); **save** = generate + the build's own checks | MECH's design | grows G3a to ~13–18½ days (rough: the generator ~7½–11, the panel ~5½–7½; +1–1½ with X-drive generation) |
| 21 | **G3**, `.vexbot` ingestion → **freeze F7** | **import** a `.vexbot` into G3a's generator; the optional SD-card runtime loader; `inferDrivetrain()` (the brittle fallback); `schemaVersion` negotiation with migration; freezes F7. DoD: a real `.vexbot` produces a working robot and routine | G3a; VexBuilder asks #1 (`paths[]`), #5 (the profile) and #2 (drivetrain fields). Today's `.vexbot` carries none of them | ~5½–8 days for the plan-of-record scope (rough; HANDOFF's ~1 day ignored it); ~2½–4 if re-scoped as proposed (import, `schemaVersion`, the F7 freeze — MECH-PANEL-requirements §5 Q27) |
| 22 | **Export**: NEW 2026-09-28 — PROPOSED, planning with the team lead, not yet in the plan | the panel writes a starter `.vexbot` (profile, mechanisms, wiring, an empty path list, shulib's command list), so a team can start in the panel and continue in VexBuilder | G3; a new VexBuilder ask (open a project shulib started) | ~1–2 days (rough) |
| 23 | **H4**, the robot console | the brain's screen and log on the computer over USB, and remote touches, in the same extension. A bench tool, off under field control | G3a; H1 | not sized |

## 5. Sensors — after G3 (your ruling, 2026-09-28)

| # | Item | Waits on | Size |
|---|---|---|---|
| 24 | **The AI Vision adapter** (R2b): objects and AprilTags from the camera on port 3 | Stations H and I (Bench Tests 11/12) run on the robot first | ~1 day (rough) |
| 25 | **Wall reset with the distance sensors** (L6): drive toward a wall until the sensor reads a set distance, then correct that axis | Q2 (where the sensors face and sit) | ~1 day (rough) |
| 26 | **Drive to a game piece** (L4): turn toward the piece the camera sees, then drive until the claw or a sensor confirms | 24 | ~1 day (rough) |

After the adapter, **S6** puts vision on the bench (robot sessions below). NEW 2026-09-28 — PROPOSED, planning with
the team lead, not yet in the plan: the sensors a MECHANISM needs (a limit switch, rotation, or a distance or optical
sensor used as a piece sensor or colour sorter) would come with MECH rather than after G3 (§11).

## 6. Library chunks already briefed ("accept all", 2026-09-26) — builders start after the auton merge

| # | Item | Size |
|---|---|---|
| 27 | **DIAG1 Part B**: the odometry skips bad (non-finite) readings — ✅ **landed 2026-10-01; DIAG1 complete** | — |
| 28 | **SAFE-SAVE**: a failed Calibrate save can never erase the good record, and the boot falls back to a verified backup. **Part B1 lands before the next Calibrate rerun** (the plan's order). Until B1 is in the build, copy `shulib_drivetrain.bin` off the card before ANY Calibrate run (worksheet G.0.1) | ~2–2½ days |
| 29 | **L9**: reverse driving. **Part A1 ✅ landed 2026-09-30** (`reverseTo`, `faceAway`; API 2.4, because TANK1 took 2.3). Left: Part A2 (cookbook 04c and the rest of the documentation) and Part B, which makes the boot library code and comes after COMP-b | ~1¼–1½ days left (A2 ~¼–½, B ~1) |
| 30 | **H1**: the SHUL/2 telemetry wire (A1 read side, A2 write side, B freezes F9). The robot console uses it. **Part A1 ✅ landed 2026-10-01** | ~1½ days left (A2 1, B ½) |
| 31 | **R5a**: the drive's gains and noise, measured by the robot and kept on the SD card. Parts A–D: C waits for SAFE-SAVE A and B1; D waits for COMP-b and SAFE-SAVE B2 | ~3½–4 days + robot sessions |

Subtotal: ~10–12½ days.

**Planned, not briefed:**

| # | Item | State | Its DoD's constraint | Size |
|---|---|---|---|---|
| 32 | **IMU1**: read the IMU's turn rate once per tick. On a robot, a second read in the same tick returns about 0 or a spike. Found by TANK1's re-check; the team lead's D4 (a) | planned 2026-09-27; no brief yet | must land **before a robot record's `imuYawRate` is read as evidence** (Station M's records are the first) and before any GPS- or tag-corrected robot runs a tank `moveTo` | ~½–1 day (rough) |

## 7. Chunks that close on the robot (robot-session-bound; not in the total)

| # | Item | What closes it | Waits on | Size |
|---|---|---|---|---|
| 33 | **R3a**, platform validation | the register's groups settled or re-planned: (a) settled or corrected; (b) settled in part, with the rest named; (c)/(d) left open, naming the hardware each needs. Its `-COMPLETED.md` is written when that holds | every session feeds it (S1–S3) | robot-session-bound |
| 34 | **R3c, the absolute-reference half**, this season, on a tank | GPS and AprilTags on a field (HA-01/06/07/09/10/31/106, HA-68/69/70) | a GPS or the camera mounted; the field (S7) | robot-session-bound |
| 35 | **R4's remainder** | GPS latency and error, encoder noise and tracking-wheel slip, turned into the EKF's noise parameters with E4 re-verified; boot-time IMU cold calibration and tip detection (code). R5a took the rest capture | robot sessions | robot-session-bound (the code is not sized) |
| 36 | **R5's remainder + S9 (retune)** | re-tune the C-phase controllers against the measured dynamics; motion re-verified on hardware; retune again after field runs | R5a's measurements; field runs | robot-session-bound |
| 37 | **R6 / P1**: back-fit the host test plant | the measured numbers (R4, R5a) go into the plant and the whole suite is re-run; every new failure is fixed where it lives | R4's and R5a's sessions | not sized (it depends on what fails) |
| 38 | **P2**: settle the register | every HA entry a session touched gets its settle line, citing the log | each session | robot-session-bound (written with each session) |
| 39 | **I1**: robot one, also a tank (the 2026-09-14 ruling) | a second chassis table plus R3d's calibration; no new core | robot one; Q3 (its cartridge); S5 | robot-session-bound |

## 8. Other open pieces

| # | Item | Size |
|---|---|---|
| 40 | **T2's close-out**: a driver compares the feel on the robot (T4 Part 3 covers most of it), then `T2-COMPLETED.md` | ~½ day (rough) |
| 41 | **Documentation, the big passes**: the finished-library set of guides, a field-day runbook, the API reference and a release | ~2–3 days (rough) |

## 9. Last of all

| # | Item | Waits on | Size |
|---|---|---|---|
| 42 | **PKG1**: a PROS template and a starter project, so a team starts from a template instead of a clone of this repository | every other chunk | not sized |
| 43 | **G4**: the 10-minute guide, written from the starter. DoD: a team member who cannot code builds a robot and a routine in VexBuilder, exports one `.vexbot`, and the robot runs it | PKG1; VexBuilder asks #1 (`paths[]`) and #5 (the profile) | ~½ day (rough; shulib's side) |

## 10. Robot sessions (you at the robot), in order

- **Station J, the claws**, on "shulib Teleop" (its package is ready now).
- **Station M, the 40 in on the bench** (ready: COMP-t landed 2026-09-29). If the floor run's log shows the library's motion stack
  drove, the **"never driven a robot under its own control" documentation sweep** follows: every instance in one
  commit (HANDOFF S1). Station M's `imuYawRate` does not count as evidence until IMU1 (#32) lands.
- **Station K, Timed Run** (after COMP-b). Then R3b and M1 are judged against Station M's and Station K's logs (#7),
  and you rule on B6 (§11).
- **Stations F and J re-run on the new "shulib Drive"** (after T4 Part 3; T4's DoD).
- **A recorded Calibrate rerun**, which fixes the ~8 % distance error and closes R3d. It waits for **SAFE-SAVE
  B1** (the plan's order). If a Calibrate run happens before B1, copy the record off the card first (G.0.1).
- **Stations H and I (Bench Tests 11/12)**: the distance sensors and the camera, ~10 minutes, any time before
  R2b.
- **S6, vision on the bench** (after R2b): what the camera reports for a piece and a tag, its latency and frame
  conventions. It settles R2b's HA entries.
- **R5a's sessions (S2 gains, S3 noise)**, after R5a Part C. The host test plant is then re-fitted to the real
  numbers (#37).
- **Mechanism sessions.** In the plan this is S4, after F3, and it is F11's freeze trigger. With MECH (proposed):
  directions, homing, holding under load and tuning.
- **Robot one** (S5, after Q3; I1). Then **S7**, absolute references on the field (GPS, tags, the wall reset),
  and **S9**, the retune after field runs.

## 11. Your decisions still open

| Who / when | Decision | Recommendation |
|---|---|---|
| COMP-b | **ND1**: after an error in the auton's task, does driver control still run? | ✅ **answered 2026-09-28: yes, after a safe stop**; later autons refused until reboot |
| COMP-b | **ND2**: the tester only runs from the calibration record (D9); does the competition program? | ✅ **answered: refuse the auton, not the boot**; driver control still runs |
| After Station K | **B6**: the auton pick under a Timed Run | you rule after Station K measures it (COMP brief B6) |
| T4 Part 2 | **C**: keep or drop the "velocity exactly 0" dead-motor check | ✅ **answered: dropped** |
| T4 Part 2 | **D**: the frozen-reading rule's guard (so a slipping port 18 at a wall does not get live motors marked dead) | ✅ **answered: guard G4** |
| T4 (Part 2a's review, 2026-09-30, and its verification, 2026-10-01) | **D, again**: G4's gap at a wall is WIDER than D weighed, two ways. **(1) At ANY wall** (the verification): the mates' lagging speed REPORTS still count as the train turning, so a motor whose report settles to 0 well before theirs is marked dead — a live motor (host model: 30 ms beside 120 ms at full stick; pinned by P2-19e). **(2) Beside the slipping port 18** (the review): one stalled motor whose readings flicker, or settle sooner, gets one or two live motors marked, and a second push can stop the drive for the run (P2-21d). Register HA-175, its "known gaps". Keep G4 with the gaps stated; change the evidence or the guard (HA-175 says what each costs) — probed on the host test plant: counting a mate as evidence only while its ENCODER moved in the last 5 reads (the brief's first wording: "a side-mate whose position IS changing") closes (1) at every probed spread and moves no other test, but that push then gets one 1 s fight CUT while the motors stay under 2.4 A (at 2.5 A each the over-current cuts come first and it adds none), and (2) stays; and/or make the wall push before a match REQUIRED with its log read for `went ABSENT` on any port | ✅ **answered 2026-10-01: the encoder check** — a side-mate counts as evidence the train turns only while its ENCODER moved in the last 5 reads; built in T4 Part 3 (its early step). A hard wall push then gets one 1 s fight cut below 2.5 A; port 18's gap (2) stays until its slipping gear is fixed |
| COMP-b (its review, 2026-10-01) | **C5**: one auton entry the registry refuses (e.g. a hard stop past the period) refused the whole BOOT, driver control included | ✅ **answered 2026-10-02: drop just that auton** — named on screen and in the log, left off the menu; every other auton and driver control still work; built in T4 Part 3's late step |
| T4 Part 3 | **A**: what the SD log copy holds | ✅ **answered: events and summaries only** |
| T4 Part 3 | **B**: does the competition program keep an SD copy? | ✅ **answered: yes, appended** |
| MECH | the core's questions (Q1–Q6, Q8, Q13, Q18, Q25, Q28–Q30) | ✅ **answered 2026-09-29 ~03:50**: bindings only in the robot file; a sensorless claw grab by time, reported unconfirmed; mechanism sensors with the mechanisms; PTO: name the later change; no 3-wire expanders yet; the most recent input wins; three-position levels stated by the team; typed names checked at build; F3/F4 re-scoped; the bench page and direction check in the core; **the list is complete — write the core's brief**. Homing for robot two's pivots: not sure yet |
| G3a | its six decisions (D1–D6); robot two's front | ✅ **D1–D6 answered 2026-09-29: all as recommended.** The front: "the front will be different depending on the robot" — per robot, already the design; robot two's own front (record: camera end; typed table: 15/16 end) to confirm at the amended panel brief |
| Q2 / Q3 / Q6 | sensor placement; robot one's cartridge; the `MotionDeps` version question | open |
| Q1 | the mechanisms each robot has. **In the plan it blocks F3 and every scoring routine**, and it is answered in part for robot two (the claws). NEW 2026-09-28 — PROPOSED, planning with the team lead, not yet in the plan: with MECH, Q1 would decide only the robots' **settings**, never library work | open |

## Later, not this season

- The VexBuilder link with run replay (H2) and an on-brain tuner (H3).
- The holonomic half of R3c on the spring X-drive (2027).
- Robot-to-robot coordination (I2, a stretch goal).
- The rest of E5 (GPS lever arm, camera mount) and visual docking (E6).
- Smooth path following.
- Power take-off support.

## The total, honestly

Sizes are from the briefs where one exists, and rough ranges otherwise (MECH, G3a's growth and G3 are priced in
[`MECH-PANEL-requirements.md`](MECH-PANEL-requirements.md) §7.1 against the plan's own comparable chunks).

| Group | Builder-days |
|---|---|
| 1. the auton: (COMP-b landed) | 0 |
| 2. T4 Part 3 with Part 2b's items (Part 2a built) | ~3½–4 |
| 3. the plan chunks MECH would change: T3's macros and presets, F3 (rough); F4 not sized | ~2½–3 |
| 4. the panel as planned: G3a as briefed (5½–6), G3 at its plan-of-record scope (~5½–8, rough); H4 not sized | ~11–14 |
| 5. sensors (rough) | ~3 |
| 6. the briefed library chunks | ~8–9½ |
| 6. IMU1 (rough) | ~½–1 |
| 8–9. T2's close-out, documentation, G4 (rough); PKG1 not sized | ~3–4 |
| **The plan of record** | **~31¾–38¼** |
| + the MECH core, before the panel (ruled 2026-09-28; rough, unmeasured) | +10½–18 |
| + G3a amended for the core (new robot, mechanisms with templates, wiring, controls): 5½–6 → ~11½–16 (rough) | +5½–10½ |
| + the rest of MECH, after the panel (ruled; rough) | +19–32 |
| + the panel's extension for the rest (rough) | +2–3½ |
| − G3 re-scoped, ~5½–8 → ~2½–4 (proposed, requirements §5 Q27) | −(3–4) |
| + Export (proposed; rough) | +1–2 |
| − T3's macros and F3, which MECH's rest would absorb | −(2½–3) |
| **Everything, as now planned** | **~62¾–99¼** |

**Not in these numbers:** H4, PKG1 and F4 (not sized); the robot-session-bound items (§7, #7 and #11) and every
robot session; and **verification**. Each result is reviewed, fixed and checked by a fresh verifier before it
merges; the one brief that sizes this, COMP-b's, puts it at about half again (~1 day on ~2).

**Calendar time.** More builders do not shorten the chain of steps that each wait on the one before, so the chain
sets the pace. The milestones along it:

| Milestone | The chain to it | Builder-days in sequence | Weeks (five-day), with verification |
|---|---|---|---|
| **The 40 in on the bench** | COMP-t ✅ (2026-09-29) — waits on your Station M run | — | now |
| **The 40 in through the real `autonomous()`** | COMP-b ✅ (2026-10-02) — waits on Station K | — | now |
| **The driver program finished, Teleop retired, `main.cpp` final** | T4 Part 3 (with Part 2b's items; early step running) | ~3½–4 | ~1 |
| **Mechanisms (the core) in the library** | + the MECH core | ~14¼–22¼ | ~4½–7 |
| **The panel: make a robot with its mechanisms, no C++** | + G3a amended | ~25¾–38¼ | ~8–12 |
| **Everything** | + the rest of MECH (its three parts side by side where their files allow) and the panel's extension, then G3 → export → sensors → documentation → PKG1 → G4 | ~45¼–69¾ | ~14–21 |

These hold only if the decisions and robot sessions keep pace and the builders are not paused. The work grew on
2026-09-28 because the team lead added generic mechanisms (MECH), the panel's new pages and export, and because
the re-checks re-sized COMP-b, T4 Parts 2–3 and G3; the core-first ruling changes the ORDER (the panel arrives
sooner), not the total. Estimates, not promises.
