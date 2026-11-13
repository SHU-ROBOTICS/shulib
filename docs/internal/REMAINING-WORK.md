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
> than on builder time, so it is not in the total. **Last updated: 2026-10-22 (the L6 and IMU1 briefs: your "Accept all" — L6 ~6–7½ in four runs, D2 amending locked decision #4; IMU1 ~1¾, before Station M's floor run; MECH-A3 landed); before that 2026-10-18 (VIS1's brief: your "Accept all", D1–D11 as recommended — VIS1 ~6½–8½ in three runs, VIS1-A1 next; ALLY1, our alliance colour, added as its own chunk; SER1's two calls answered; the branch pushed at `ceb17c0`).**

**Where things stand:** 36 of the phase table's 71 chunks are complete (CAL1 added 2026-10-06, MECH, DOCS3, SER1, VIS1 and AF1 2026-10-08, ALLY1 2026-10-18) (COMP on 2026-10-02, T4 on 2026-10-08, SER1 on 2026-10-15), and R2a (half of R2) has
also landed; 34 chunks are open. Landed most recently: L9 Part A1 (backing into a point in a recipe, 2026-09-30); DIAG1
Part B (a reading that is not a number left out, completing DIAG1), H1 Part A1 (the SHUL/2 wire's read side) and T4
Part 2a (the drive protections) on 2026-10-01; and **COMP-b, the competition program** (2026-10-02, completing COMP).
SAFE-SAVE Part A (the safe save's library) landed 2026-10-04, and R5a Part A (the gains-and-noise measurements as
library) 2026-10-06. **T4 Part 3 landed 2026-10-08, completing T4**: the new "shulib Drive", Teleop retired, the
Auton Tester on slot 2, the SD log copy, one bad auton dropped (never the boot), `main.cpp` final, seven builds. The
**MECH core is building** (A1 and A2 landed 2026-10-15, A3 2026-10-22, A4 2026-10-29; A5 2026-10-30; A6 2026-11-06 — THE CORE IS COMPLETE (`MECH-COMPLETED.md`); the runs overlap, your speedup; CAL1 maps the core's list after A5, its N9 fallback); CAL1's brief has your answers (its Part A1 landed 2026-10-15); VIS1's brief has your
answers (2026-10-18), and L6's and IMU1's (2026-10-22). **Your ordering ruling of 2026-10-29:** the panel's
foundation (G3a: the robot file and its generator) starts right after MECH-A5 lands, beside VIS1, L6 and the
auto-flipper; their pages plug in as each lands; DOCS3 follows the panel. G3a's brief has your answers (D6′–D10,
2026-10-29, all as recommended): two lanes after MECH-A5. **Its evening amendment is answered too (2026-10-30, D11–D20):** the driving feel, drive tuning, a full route maker (with the library chunk STEP1), the wall and camera pages; D16 a true curve follower (its own chunk, CURVE1, after the route maker); publisher `shulib`. String-lift safety (MECH-S) and live tuning (TUNE1) are ruled, briefs next. **The first competition is NOV 21 (your builder, 2026-11-05; it was written as the 23rd), both robots identical, one built by
Nov 14; your order (2026-11-05): the CASCADE first (current cap, live tuning, its levels), then the autons, then the auto-flipper,
then curves; snap detection and L4 after the 21st.** **Your PANEL SPRINT (2026-10-29, later): the panel by
2026-11-21** — MECH-A5 and G3a Part 1 building now, the builds overlapping, the wall reset kept; the camera's
later runs, our colour, driving to a piece and the auto-flipper after the panel; robot one made in the panel
with a hand-written backup.
Nothing new has run on the robot since 2026-09-25. **The 40 in auton is ready for you on the bench** (#4).

---

## 1. The auton — in progress (the team lead pushes the branch after this, before G3a)

| # | Item | State | Waits on | Size |
|---|---|---|---|---|
| 1 | **TANK1 Part A**: a tank's `moveTo` bends back toward its line (≤ 15°, a 0.06 s look-ahead) and cancels the yaw a weak side makes. X/H drives unchanged | ✅ landed `3965791` (API 2.3) | — | — |
| 2 | **COMP-t**: "shulib Auton Tester" (pick an auton, touch RUN, 3 s countdown, any button aborts; it runs the same `runAuton` path `autonomous()` uses) plus the students' auton file `src/team/autons.{hpp,cpp}` with the 40 in auton | ✅ **landed 2026-09-29** (host- and build-gated, seven builds; the coordinator's ten safety mutations all red; **never run**) | — | — |
| 3 | **TANK1 Part B**: the sideways-miss number (` xt`) on every tank result line; no API bump | ✅ landed 2026-09-29 — **TANK1 COMPLETE** | — | — |
| 4 | **You run the 40 in on the bench**: worksheet Station M. Wheels up first, then the floor and the tape, with serial captured. Its floor run (M.6) counts as evidence for HA-179/HA-180 only on a build with IMU1 Part A (#32); M.0–M.5 need not wait | **ready — needs you at the robot** (upload with `rm -rf bin .d` first; copy the calibration record off the card first) | 32 (M.6 only) | ~1 hour |
| 5 | **COMP-b**, the competition program: `autonomous()` runs the chosen auton on the program's own task under the guard; `opcontrol()` runs the driver file; the brain-screen auton picker; slot 6; the eighth build | ✅ **landed 2026-10-02 — COMP complete** (host- and build-gated, NOT run on a robot; Station K is its first run). Your C5 ruling (one bad auton entry is dropped, never the boot) is built in T4 Part 3's late step | — | — |
| 6 | **Station K**: a practice match with the controller's Timed Run (does Timed Run start `autonomous()`? HA-162). B6 is decided after it (§11). Its floor runs (K.5) count with M.6's rule — only on a build with IMU1 Part A (#32) | needs you at the robot | 5; 32 (K.5 only) | ~1 hour |
| 7 | **R3b + M1** close when "a v2 tank auton runs on the robot under the library's own steering" (R3b's DoD) and M1's DoD holds. Both are judged against Station M's and Station K's logs. They also settle HA-18, the threshold half of HA-52 and the "drivable" half of HA-112 | open since June | 4, 6 | robot-session-bound |

## 2. The driver program (T4) — Part 2a before COMP-b; Part 3, with Part 2b's items, beside it and after it

| # | Item | State | Waits on | Size |
|---|---|---|---|---|
| 8 | **Part 1a**: `src/team/robot_two_driver.hpp` (which button does what, the feel) + `src/common/driver_controls.hpp` | ✅ landed `8d89df7` | — | — |
| — | **Part 1b** was SKIPPED on 2026-09-28 by your ruling and folded into Part 3, so Teleop is never rewired | — | — | — |
| 9 | **Part 2**: the old "shulib Drive"'s protections move into the library for every program. Driving continues with dead drive motors (it stops only when 3+ are dead). The FIGHT and OVER-CURRENT cuts are added; over-current is on by default, with an off switch and a push check before any match | **Part 2a** (the library half) ✅ **landed 2026-10-01** (host- and build-gated, NOT run on a robot; COMP-b's `opcontrol()` gets it from its first build); **Part 2b** (its `src/` wiring) FOLDED INTO PART 3 on your ruling of 2026-10-01 — Part 3's late step | — | — |
| 10 | **Part 3**: the new "shulib Drive" replaces the old one (slot 1). "shulib Teleop" retires. The buttons leave the mechanism table (Part 1b's work). Each program can keep an optional SD copy of its log (events only). `main.cpp` reaches its final form. **Since 2026-10-01 it starts with Part 2b's wiring** (the PROTECTION row, `perTick`'s fourth argument, the re-read of COMP-b's files). **Since 2026-10-01 it starts EARLY**: the items that need nothing of COMP-b are built beside it, the rest after it lands, then one review round. ✅ **LANDED 2026-10-08, completing T4** (host- and build-gated, NOT run on a robot — #11 is its robot check). Both steps built 2026-10-04 and 2026-10-06; reviewed 2026-10-07; the review's 17 findings and the fresh verification's 4 fixed 2026-10-08: the new "shulib Drive", Teleop retired (seven builds), the tester in slot 2, the SD log copy for "shulib Drive" and the competition program only (the coordinator's ruling, 2026-10-04), Part 2b's PROTECTION row and push check, and your C5 ruling (one bad auton entry DROPPED, never the boot, in the competition program — and the same in the tester, the coordinator's extension of your ruling, **confirmed by you 2026-10-08**) | brief re-checked (as above); your A/B answered | — | — |
| 11 | **Stations F (drive) and J (claws) re-run on the new "shulib Drive"**: T4's DoD, and nothing of T4 is claimed on hardware until these logs exist | needs you at the robot | 10 | ~1 hour (rough) |

## 3. Mechanisms (MECH) — NEW 2026-09-28 — the order is ruled, the list is COMPLETE (2026-09-29), the core's brief has your go (2026-10-08) and run A1 is building

The requirements are in [`MECH-PANEL-requirements.md`](MECH-PANEL-requirements.md). That file is planning, not a
brief; the team lead declared the list complete on 2026-09-29, and the core's brief has your go (2026-10-08; run A1 building). In short: **any
mechanism is described as data, bound to ports after it is created, and used the same way in autons, the driver
file and the panel.** PTO-readiness is one of its principles (P7); how it will be met is still open.

**The team lead's ruling (2026-09-28, ~17:15):** "lets do this so we have it, we can do the rest after the panel"
— **the mechanism CORE is built first, before the panel; everything else comes after the panel** (and before G3,
so the robot-file format is locked with VexBuilder once, with the whole vocabulary — the coordinator's placement,
open to his objection).

**The team lead's ruling (2026-10-29):** "Start the panel's FOUNDATION (G3a: the robot file and its generator into C++) right after MECH-A5 lands — beside VIS1, L6 and the auto-flipper — instead of waiting for them and DOCS3? The panel's sensor and auto-flipper pages plug in as those land; DOCS3 then documents everything, panel included. The auto-flipper stays fully in." — "Start it after A5 (Recommended)". **G3a's foundation
starts when MECH-A5 merges, beside MECH-A6, VIS1, L6, ALLY1, AF1 and L4; their panel pages plug in as each
lands; DOCS3 (#19b) follows the panel.** The core's last run (A6) is the only part of it beside the panel.

**The team lead's PANEL SPRINT (2026-10-29, later the same day):** "WE MUST FINISH THE PANEL BY THEN" — "by the 21st". **MECH-A5 starts now, before CAL1 lands (the brief's own fallback), with G3a Part 1 beside it (D6′ (b)); each later run's build starts when the previous BUILD ends; the wall reset keeps building; VIS1's later runs, ALLY1, L4 and AF1 wait until the panel lands; up to five jobs.** Nothing is skipped.

| # | Part | What | When | Size (rough, unmeasured) |
|---|---|---|---|---|
| 12 | **MECH core** | the four control kinds — **power** (with done-conditions), **position** (degrees or inches, non-linear lifts, smooth moves, gravity compensation, travel limits, homing by switch or stall), **speed** (flywheels, "at speed"), **pneumatic** (incl. three positions); **port binding** (roles instead of ports, the wiring binding validated every boot, one boot line per role); **auton steps** (and the auton's mechanism lookup, run while driving); **presets** in the driver file; the per-tick mechanism service everything depends on; the device provider's distance / optical / digital-in accessors; arm, lift and flywheel models in the host test plant; robot two's claws as the proving case; **every mechanism publishes its actions as route ids automatically** (ruled 2026-10-08) | after T4 Part 3, **before the panel** — **brief ✅ GO 2026-10-08** (N1–N13 as recommended; [brief](chunks/MECH-core-mechanism-library.md)); **A1 LANDED 2026-10-15** (the binding and the devices — [live log](chunks/MECH-PROGRESS.md)); **A2 LANDED 2026-10-15** (the service, power, pneumatic, the guard's cut); **A3 LANDED 2026-10-22** (built 2026-10-15: position and speed: moves, holds, homing, the soft stop, the stall watch on every behaviour, the starting values — HA-182); **A4 BUILT 2026-10-22** (auton steps, the driver loop, the direction judgement, the route binder — API 2.5; its review's fix round done 2026-10-27 and its verification round 2026-10-29; LANDED 2026-10-29 with your three rulings — the steps' freeze OPEN for now, the driver-start home's exception and U5's stricter rule ACCEPTED); **A5 LANDED 2026-10-30** (built 2026-10-29; robot two and the programs: its claws as declarations, one mechanism service in every program that boots it, its boot lines and route ids, its driver file as three lines of controls — claws exactly as before; no API change; HA-185); **A6 LANDED 2026-11-06 — the core is COMPLETE** (Bench Tests' station 13 MECHANISMS: each mechanism in its own units, powered one at a time behind six gates, the direction check on the robot; the menu 3 × 5; no API change; not run on a robot — worksheet Station P is its first run), with G3a Part 1 built beside A5 and in its review's fixes (your PANEL SPRINT, 2026-10-29) | ~19¼–26 (its brief; six runs A1–A6) |
| 13 | **MECH, the rest** — protection and more kinds | cycle mechanisms (catapult/puncher); piece counting; colour sorting; jam auto-reverse; interlocks; loaded gains; low-power hold; current caps and heat derating with a driver warning; tip protection | **after the panel** | ~7–11½ days |
| 14 | **MECH, the rest** — driver and auton extras | preset cycling, stick control with hold-where-released, long-press and double-tap, rumble, mechanism state on the controller screen, driver-interruptible macros (would absorb T3's open chassis macros and angle presets, #16); triggers partway through a drive | **after the panel** | ~5½–9 days (+~1 if routes are in scope) |
| 15 | **MECH, the rest** — setup and diagnostics | ~~a mechanism page in Bench Tests; the direction check~~ (moved into the core by its brief, 2026-10-08: Bench Tests' MECHANISMS station and its direction check); guided port binding on the brain (and the SD binding record, if bindings live there); auto-tuning of an arm's gains; a per-match health log | **after the panel** | ~6½–11½ days |
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
| 18b | **CAL1, the standard Calibrate** — NEW 2026-10-06, your request | Calibrate moves into the library so every team (and the panel) gets the same, correct Calibrate; a team states only drive kind, motor cartridge and its mechanisms' ports. It absorbs SAFE-SAVE B1 (the safe save) and adds your FRONT rule: the end that leads the push is the front, with a warning when a recalibration flips it. Robot two's Calibrate becomes a few lines calling it | **brief ✅ answered 2026-10-08** (D1–D5 as recommended, D6 (a) — [brief](chunks/CAL1-standard-calibrate.md)); **Part A1 ✅ landed 2026-10-15** (the seams: the brain's screen as a seam, the painter, the front check — host-only, no program uses them yet; [live log](chunks/CAL1-PROGRESS.md)); A2 SPLIT into A2a and A2b (your call, 2026-10-08); **Part A2a ✅ landed 2026-10-29** (built 2026-10-18–22) (the standard program with no typed reference: the program in the library, its PROS shell, the front check's and the watch's holds, the safe save — host-only, no program uses it yet); **Part A2b LANDED 2026-11-03** (built 2026-10-30: your D6 (a) — a team's typed drive ports checked when the card holds no record —, the program proven through the real PROS shell in 34 recorded scenarios, the team's 21-line file and guide chapter 19, HA-187); **Part B LANDED 2026-11-11 — CAL1 COMPLETE** (built 2026-11-05; robot two's "shulib Calibrate" IS the standard Calibrate: a 27-line file over facts taken from its own tables; the WHICH END question is gone — the end that leads the push is the front; host-only, not yet run on a robot; lands at the coordinator's merge, and CAL1 is then complete — [record](chunks/CAL1-COMPLETED.md)) | ~6¼–6¾ days (its brief: A1, A2a, A2b, B; B1's ~1 included) |
| 19 | **G3a**, as briefed | the robot file (the `.vexbot` profile block itself); its generator into the C++ tables and the driver file's button region, byte for byte; the freshness gate; one validator; the optional VS Code panel (a form over the file, with the calibration record shown read-only) | MECH-A5 merging (your ruling, 2026-10-29) and each part's go — D1–D6 answered 2026-09-29, D6′–D10 2026-10-29 ([brief](chunks/G3a-robot-profile-generator-panel.md)) | ~5½–6 days (its draft brief, 2026-09-27); *its answered brief (2026-10-29) sizes rows 19 and 20 together — row 20* |
| 18d | **VIS1, the AI Vision camera's colours and objects** — NEW 2026-10-08, your auto-flipper (its objects by your ruling of 2026-10-14) | the camera adapter (R2b's, pulled forward, objects and colours only — no tags), a frame type beside the frozen interface, pixels turned into directions and distances from the measured mount, the Toggle colour tracker, the piece finder (ONE named piece by its type and field position, or none — never a guess), one camera feed for every consumer, the host camera model, Station 12 with colours | **brief ✅ GO 2026-10-18 (all 11 as recommended)** — your "Accept all" ([brief](chunks/VIS1-ai-vision-colours.md)); three runs, each its own go: **VIS1-A1** (the camera as a device) ✅ **LANDED 2026-10-30**; **VIS1-A2** (what the camera means) when A1's build ends; **VIS1-B** (robot two and the bench page) after MECH-A5, before MECH-A6; Step 0 at the robot (its first four steps need no new code — the next bench session) | ~6½–8½ days (its brief: A1 ~2½–3½, A2 ~2¼–2¾, B ~1¾–2¼) |
| 18e | **AF1, the Toggle auto-flipper** — NEW 2026-10-08, your request; both robots, a motor arm | an automatic flip that never fights the driver, never stops on the opponent's colour, always gets off the Toggle, and never starts what it cannot finish; the alliance colour from ALLY1 (#18f — taken out of AF1 on 2026-10-18, VIS1's D8); a period clock; a host Toggle model; a TOGGLE bench station. Supersedes F3's `setQuadrantToggle` | after MECH-A4 and ALLY1 (#18f), before DOCS3; the Optical-at-the-flipper question after Step 0 *(before the 21st again — your ruling of 2026-11-03, "Everything incl. flipper")* | ~8¾–11½ days (rough; it still counts the alliance colour, now ALLY1's, until AF1's brief re-sizes it) |
| 18f | **ALLY1, our alliance colour** — NEW 2026-10-18, VIS1's D8 (your "Accept all") | the alliance colour — RED / BLUE / SKILLS, chosen before the match on the brain-screen picker, unset until chosen (no arming without it) — kept from autonomous into driver control: the library half (`competition/alliance.hpp`, the picker's colour row), then its `src/` half (the menu row in the competition program and the Auton Tester, a driver override button) after MECH-A5. L4 needs it to choose "our" piece, AF1 for the Toggle's target colour; it was inside AF1 (#18e) until D8 | after MECH-A4, **before L4 (#26) and AF1 (#18e)**; its own brief first *(before the 21st again — your ruling of 2026-11-03, "Everything incl. flipper")* | ~1–1½ days (rough, VIS1's brief D8) |
| 18c | **SER1, the serial log's lines kept whole across tasks** — NEW 2026-10-08 (HA-189), your go with your fix | your fix re-ruled to ONE-CALL WRITES (unbuffered stdout would crash the brain when `pros terminal` connects): every shulib serial line is one direct write that PROS's own lock keeps whole; the build gate bans stdout printing in `src/`; two new register entries (HA-190, HA-191). ✅ **LANDED 2026-10-15** (built 2026-10-08; reviewed, fixed and verified 2026-10-15 — [live log](chunks/SER1-PROGRESS.md), [record](chunks/SER1-COMPLETED.md)). **✅ Both your calls ANSWERED 2026-10-18:** (1) **"Use Station K's K.7.3"** — HA-189's dedicated bench test (two tasks printing numbered lines, every number once) is not built; worksheet K.7.3 is accepted, in writing, as HA-189's robot test, and no Bench Tests station is added (the 3 × 4 grid stays full). (2) **"Accept pieces"** — your ONE-CALL WRITES ruling, by its letter, was one `write()` per line whatever its length; the sink offers a call at most 512 bytes (`kSerialWriteMaxBytes`), so a text longer than that through a team's own sink goes out as several whole pieces (one call past PROS's ~2 KB could hang its writer, HA-191) — every line shulib prints is still ONE call | the bench: worksheet K.7.3, HA-189's robot test (your ruling, 2026-10-18) — HA-189 stays unsettled until it runs. (The coordinator's verification, the whole serial test under ASan + UBSan included, was done at the merge, 2026-10-15.) | ~¾–1 day |
| 19b | **DOCS3, the documentation redo** — NEW 2026-10-08, your ruling | every current document re-organised so each reader has one front door (driving, coding, mentoring, maintaining), rewritten where unclear, every claim re-checked against the code, the site's navigation and the API reference's layout redone; the history records (briefs, logs, completion records, changelog entries, HA numbers) indexed, not rewritten. Until then each chunk keeps its own documents current | **after the panel** (your ruling of 2026-10-29: it documents everything, panel included; it was after the MECH core, before the panel) | ~5–8 days (rough, until its brief) |
| 20 | **G3a, the amended panel**: NEW 2026-09-28 — in the plan since its brief was answered (2026-10-29) | a **new robot** (which creates its build too); **mechanisms** (add / remove / duplicate / reorder, with templates: Intake, Arm, Lift, Claw, Flywheel, Catapult, Conveyor, Blank); **wiring** (roles → ports, or "bind on the brain", with live warnings); **controls** (every MECH C control, with live warnings); **save** = generate + the build's own checks | MECH-A5 merging; each part's go | grows G3a to ~13–18½ days (rough: the generator ~7½–11, the panel ~5½–7½; +1–1½ with X-drive generation); *its answered brief (2026-10-29): Parts 1–6 ~17¾–23¾ (~11–15½ in two lanes to a robot made and driven in the panel); with Parts R, S and AF ~24¾–34¼* |
| 20c | **STEP1, the route maker's library half** — NEW 2026-10-30 (G3a's D13–D15) — **brief ✅ GO 2026-11-03** ([brief](chunks/STEP1-route-steps-library.md), D1–D8 accepted; STEP1-A ~3–4 and STEP1-B ~3–4 days; in the auton library, ahead of the panel) — **STEP1-A LANDED 2026-11-11** (built 2026-11-05; the arrival tolerance; stop / carry on / skip to a label; the deadline always stops; "DONE, n FAILED"; [live log](chunks/STEP1-PROGRESS.md)) — host- and build-gated, NOT run on a robot; in review — **STEP1-B LANDED 2026-11-13 (API 2.8)** (built 2026-11-11; triggers partway through a drive: `atPercent` / `afterDistance` / `afterTurning`; the leg observer; D8's check functions; HA-217..HA-218; same live log) — host- and build-gated, NOT run on a robot; in review | per-step arrival tolerance and on-failure (stop / skip to a named step / carry on; the deadline always stops), partway triggers (a drive that ends early never fires one); STEP1-A and STEP1-B, briefed from G3a's R.13 | STEP1-A before G3a Part R1's build; STEP1-B before R1-T (the brief's §6) | ~6–8 days (rough; the brief's re-check — was ~5–6½) |
| 20d | **CURVE1, a true curve follower** — NEW 2026-10-30 (G3a's D16 (c), your answer) | a pursuit- or boomerang-style controller on a tank, a smooth path form, its verb and Routine step, F8's `"arc"` (additive); re-opens C1's D12 ("never aim at the point") for the follower only; its own brief. **CURVE1-A LANDED 2026-11-12 (API 2.7)** (built 2026-11-11; pure pursuit, `curveTo` / `reverseCurveTo` + Routine steps, host test plant only — `chunks/CURVE1-PROGRESS.md`); CURVE1-B (F8's `"arc"`) after the 21st | Station M, then Station C on robot two (the two tuning numbers; `kUseCurves` stays false until it passes) | CURVE1-A built; CURVE1-B ~¾–1¼ days |
| 20e | **MECH-S, a string lift's safety** — NEW 2026-10-29, your ruling — **brief ✅ GO 2026-11-03** ([brief](chunks/MECH-S-TUNE1-string-lift-safety-live-tuning.md), D1–D13 accepted; the lift is on ROBOT ONE, built, STRINGS BOTH WAYS — the snap watch is amended for that before it builds); **MECH-S-A (the current cap) LANDED 2026-11-10** (built 2026-11-05) — host-only, HA-219..HA-223 | a per-mechanism current cap (pulled forward from MECH's rest, #13), snapped-string detection by motor load by default (a slack signature: below X A while above Y rpm, or while holding off the bottom, for Z s — thresholds from the bench) with an output rotation sensor optional and stronger | its brief; before the panel lands | not sized |
| 20f | **TUNE1, live tuning on the robot** — NEW 2026-10-29, your ruling — **brief ✅ GO 2026-11-03** (with MECH-S's: "shulib Tune" in slot 7); **TUNE1-A (live tuning's library) LANDED 2026-11-11** (built 2026-11-10); **TUNE1-B1 ("shulib Tune", slot 7, ROBOT-generic) LANDED 2026-11-11** (built 2026-11-11) — host-only, no HA id; TUNE1-B2 (the copy-back) after the panel | change a mechanism's gains, speeds and limits on the brain, watch the response, copy the numbers into the panel | its brief; before the panel lands | not sized (rough ~2–3) |
| 20b | **G3a Part R, routes in the panel** — NEW 2026-10-08, your ruling | draw a route on the field (waypoints, headings, reversed legs), drop mechanism steps from the robot's own route ids, save in the shared route format VexBuilder reads; **a visual of the route** (the path, headings, the robot's outline at each waypoint, the mechanism markers, a step-through — geometry only), and **VexBuilder simulates it — the panel never does** (your rulings, 2026-10-08: one simulator, so the two can never disagree) | after G3a's Parts 6 and 3b in the panel lane (its D6′, 2026-10-29); the route ids come from the MECH core — *2026-10-30: now Parts R1 (the route tool) and R2 (the page), every auton step with care per step, mechanism timing, red/blue mirroring and a timeline; STEP1 (#20c) first* | ~4–6 days (rough) |
| 21 | **G3**, `.vexbot` ingestion → **freeze F7** | **import** a `.vexbot` into G3a's generator; the optional SD-card runtime loader; `inferDrivetrain()` (the brittle fallback); `schemaVersion` negotiation with migration; freezes F7. DoD: a real `.vexbot` produces a working robot and routine | G3a; VexBuilder asks #1 (`paths[]`), #5 (the profile) and #2 (drivetrain fields). Today's `.vexbot` carries none of them | ~5½–8 days for the plan-of-record scope (rough; HANDOFF's ~1 day ignored it); ~2½–4 if re-scoped as proposed (import, `schemaVersion`, the F7 freeze — MECH-PANEL-requirements §5 Q27) |
| 22 | **Export**: NEW 2026-09-28 — PROPOSED, planning with the team lead, not yet in the plan | the panel writes a starter `.vexbot` (profile, mechanisms, wiring, an empty path list, shulib's command list), so a team can start in the panel and continue in VexBuilder | G3; a new VexBuilder ask (open a project shulib started) | ~1–2 days (rough) |
| 23 | **H4**, the robot console | the brain's screen and log on the computer over USB, and remote touches, in the same extension. A bench tool, off under field control | G3a; H1 | not sized |

## 5. Sensors — the distance sensors and the camera's OBJECTS now BEFORE THE PANEL'S SENSOR PAGES (your ruling, 2026-10-14), beside the panel's foundation (your ruling, 2026-10-29); AprilTags and GPS stay after G3 (2026-09-28)

| # | Item | Waits on | Size |
|---|---|---|---|
| 24 | **The AI Vision adapter** (R2b): **its OBJECTS part (pins, cups) is now in VIS1 (#18d), before the panel** (your ruling, 2026-10-14); its AprilTags part (and the field's duplicate tag ids) stays after G3 | Stations H and I (Bench Tests 11/12) run on the robot first | AprilTags: ~1 day (rough) |
| 25 | **Wall reset with the side distance sensors** (L6): a robot parked square along a field wall re-seats ONE coordinate (x or y) from the side sensor facing that wall — never the heading, never in the background — inside gates that refuse rather than guess — **BEFORE THE PANEL'S WALL-RESET PAGE** (your ruling, 2026-10-14; beside the panel's foundation since 2026-10-29) — **brief ✅ GO 2026-10-22** ("Accept all", D1–D13; D2 amends locked decision #4) | **L6-A1 LANDED 2026-11-03** (built 2026-10-29; the library core: `Localizer::seatAxis`, the gates, the season's walls, the explicit call, the host field plant — host- and build-gated, NOT run on a robot); **L6-A2 LANDED 2026-11-10** (built 2026-11-03–05; the robot file's side sensors, the Robot building them, the everyday call `competition::resetFromWall(ctx, …)` — FIELD starts only — host- and build-gated, NOT run on a robot); B before MECH-A5; C (optional) after MECH-A4 | ~6–7½ days (its brief; four runs) |
| 26 | **Drive to a game piece** (L4): turn toward the piece the camera sees, then drive until the claw or a sensor confirms — **BEFORE THE PANEL'S CAMERA PAGE, after VIS1** (your ruling, 2026-10-14; beside the panel's foundation since 2026-10-29) | VIS1's objects; ALLY1 (#18f — our colour, to choose "our" piece; VIS1's D8, 2026-10-18) *(before the 21st again — your ruling of 2026-11-03, "Everything incl. flipper")* | ~1 day (rough) |

After the adapter, **S6** puts vision on the bench (robot sessions below). NEW 2026-09-28 — PROPOSED, planning with
the team lead, not yet in the plan: the sensors a MECHANISM needs (a limit switch, rotation, or a distance or optical
sensor used as a piece sensor or colour sorter) would come with MECH rather than after G3 (§11).

## 6. Library chunks already briefed ("accept all", 2026-09-26) — builders start after the auton merge

| # | Item | Size |
|---|---|---|
| 27 | **DIAG1 Part B**: the odometry skips bad (non-finite) readings — ✅ **landed 2026-10-01; DIAG1 complete** | — |
| 28 | **SAFE-SAVE**: a failed Calibrate save can never erase the good record, and the boot falls back to a verified backup. **Part A (the library) ✅ landed 2026-10-04.** **Part B1 (Calibrate's save) is now part of CAL1** (2026-10-06, row 18b); B2 (the boot's fallback) remains. B1 is in robot two's Calibrate since CAL1 Part B (built 2026-11-05); before the FIRST run of that new Calibrate, copy both files off the card (worksheet G.0.1) | ~½ day left (B2) |
| 29 | **L9**: reverse driving. **Part A1 ✅ landed 2026-09-30** (`reverseTo`, `faceAway`; API 2.4, because TANK1 took 2.3). Left: Part A2 (cookbook 04c and the rest of the documentation) and Part B, which makes the boot library code and comes after COMP-b | ~1¼–1½ days left (A2 ~¼–½, B ~1) |
| 30 | **H1**: the SHUL/2 telemetry wire (A1 read side, A2 write side, B freezes F9). The robot console uses it. **Part A1 ✅ landed 2026-10-01** | ~1½ days left (A2 1, B ½) |
| 31 | **R5a**: the drive's gains and noise, measured by the robot and kept on the SD card. **Part A (the library) ✅ landed 2026-10-06**; B (the card and the builder), C (Calibrate runs them — waits for SAFE-SAVE B1) and D (waits for SAFE-SAVE B2) remain | ~2½–3 days left + robot sessions |

Subtotal: ~10–12½ days.

**Planned, not briefed:**

| # | Item | State | Its DoD's constraint | Size |
|---|---|---|---|---|
| 32 | **IMU1**: read the IMU's turn rate once per tick (the adapter keeps its answer for 5 ms, so every reader in a tick gets one value). Found by TANK1's re-check; its probe found the double read can turn the 40 in auton into a 5 s timeout in one timing band | **brief ✅ GO 2026-10-22** ("Accept all", D1–D4); **Part A BUILT 2026-10-29**, its review's and its verification round's findings fixed the same day, LANDED 2026-10-30 (`6798aa5`); **Part B BUILT 2026-11-03, LANDED 2026-11-06 — IMU1 COMPLETE** — Bench Tests button 2's STAGE 2c, the IMU's timing (host- and build-gated, never run on a robot — [record](chunks/IMU1-COMPLETED.md), [live log](chunks/IMU1-PROGRESS.md)); its robot step is worksheet **Station N** (~10 min, any Bench Tests session) | must land **before Station M's floor run** (D4) and before any GPS- or tag-corrected robot runs a tank `moveTo` | ~1¼ + ~½ days (its brief) |

## 7. Chunks that close on the robot (robot-session-bound; not in the total)

| # | Item | What closes it | Waits on | Size |
|---|---|---|---|---|
| 33 | **R3a**, platform validation | the register's groups settled or re-planned: (a) settled or corrected; (b) settled in part, with the rest named; (c)/(d) left open, naming the hardware each needs. Its `-COMPLETED.md` is written when that holds | every session feeds it (S1–S3) | robot-session-bound |
| 34 | **R3c, the absolute-reference half**, this season, on a tank | GPS and AprilTags on a field (HA-01/06/07/09/10/31/106, HA-68/69/70) | a GPS or the camera mounted; the field (S7) | robot-session-bound |
| 35 | **R4's remainder** | GPS latency and error, encoder noise and tracking-wheel slip, turned into the EKF's noise parameters with E4 re-verified; boot-time IMU cold calibration and tip detection (code). R5a took the rest capture | robot sessions | robot-session-bound (the code is not sized) |
| 36 | **R5's remainder + S9 (retune)** | re-tune the C-phase controllers against the measured dynamics; motion re-verified on hardware; retune again after field runs | R5a's measurements; field runs | robot-session-bound |
| 37 | **R6 / P1**: back-fit the host test plant | the measured numbers (R4, R5a) go into the plant and the whole suite is re-run; every new failure is fixed where it lives | R4's and R5a's sessions | not sized (it depends on what fails) |
| 38 | **P2**: settle the register | every HA entry a session touched gets its settle line, citing the log | each session | robot-session-bound (written with each session) |
| 39 | **I1**: robot one, also a tank (the 2026-09-14 ruling) | a second chassis table plus R3d's calibration; no new core | robot one; Q3 (its cartridge); S5 — *for 2026-11-23 (the sprint): made in the panel if the tool lane lands ~Nov 16 and tests well, the students' hand-written table as the backup* | robot-session-bound |

## 8. Other open pieces

| # | Item | Size |
|---|---|---|
| 40 | **T2's close-out**: a driver compares the feel on the robot (T4 Part 3 covers most of it), then `T2-COMPLETED.md` | ~½ day (rough) |
| 41 | **Documentation, the big passes** (after DOCS3, #19b, which follows the panel since 2026-10-29; these cover only what came after DOCS3): the finished-library set of guides, a field-day runbook, the API reference and a release | ~2–3 days (rough) |

## 9. Last of all

| # | Item | Waits on | Size |
|---|---|---|---|
| 42 | **PKG1**: a PROS template and a starter project, so a team starts from a template instead of a clone of this repository | every other chunk | not sized |
| 43 | **G4**: the 10-minute guide, written from the starter. DoD: a team member who cannot code builds a robot and a routine in VexBuilder, exports one `.vexbot`, and the robot runs it | PKG1; VexBuilder asks #1 (`paths[]`) and #5 (the profile) | ~½ day (rough; shulib's side) |

## 10. Robot sessions (you at the robot), in order

- **Station J, the claws**, on "shulib Drive" (slot 1) since T4 Part 3 (it was written for "shulib Teleop" until then; it has never been run).
- **Station M, the 40 in on the bench** (ready: COMP-t landed 2026-09-29). If the floor run's log shows the library's motion stack
  drove, the **"never driven a robot under its own control" documentation sweep** follows: every instance in one
  commit (HANDOFF S1). Its floor run (M.6) counts as evidence for HA-179/HA-180 only on a build with IMU1 Part A
  (#32; the worksheet's check is a grep for `kRateHoldUs`, not the git log); M.0–M.5 need not wait.
- **Station K, Timed Run** (after COMP-b; its K.5 floor runs count with M.6's rule). Then R3b and M1 are judged against Station M's and Station K's logs (#7),
  and you rule on B6 (§11).
- **Stations F and J re-run on the new "shulib Drive"** (after T4 Part 3; T4's DoD).
- **A recorded Calibrate rerun**, expected to fix the ~8 % distance error (not yet measured), and it closes R3d.
  Unblocked by CAL1 Part B (2026-11-05: SAFE-SAVE B1's safe save is in robot two's Calibrate build — row 28);
  **copy BOTH files off the card before the first run of the new Calibrate** (worksheet G.0.1).
- **Stations H and I (Bench Tests 11/12)**: the distance sensors and the camera, ~10 minutes, any time before
  R2b.
- **Station N (Bench Tests button 2's STAGE 2c, ~10 minutes, any Bench Tests session)**: the IMU's 10 ms copy,
  its phase against the tick, a lower bound on a tick's span, and the gyro's sign and units (HA-213/214/215,
  HA-04, HA-109). A first look, not a settlement; making the gyro's own rate the default waits on it (IMU1, #32).
- **VIS1's Step 0** (its [brief](chunks/VIS1-ai-vision-colours.md) §2.0, answered 2026-10-18): measure the camera's
  mount, read which AI model the camera carries, teach the three Toggle colours in VEXcode's AI Vision Utility, and
  hold a Toggle where the arm will push it. These first four steps need no new code — the next bench session. Their
  answers settle where the camera can sit and feed your Optical-at-the-flipper decision.
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
| T4 (Part 2a's review, 2026-09-30, and its verification, 2026-10-01) | **D, again**: G4's gap at a wall is WIDER than D weighed, two ways. **(1) At ANY wall** (the verification): the mates' lagging speed REPORTS still count as the train turning, so a motor whose report settles to 0 well before theirs is marked dead — a live motor (host model: 30 ms beside 120 ms at full stick; pinned by P2-19e). **(2) Beside the slipping port 18** (the review): one stalled motor whose readings flicker, or settle sooner, gets one or two live motors marked, and a second push can stop the drive for the run (P2-21d). Register HA-175, its "known gaps". Keep G4 with the gaps stated; change the evidence or the guard (HA-175 says what each costs) — probed on the host test plant: counting a mate as evidence only while its ENCODER moved in the last 5 reads (the brief's first wording: "a side-mate whose position IS changing") closes (1) at every probed spread and moves no other test, but that push then gets one 1 s fight CUT while the motors stay under 2.4 A (at 2.5 A each the over-current cuts come first and it adds none), and (2) stays; and/or make the wall push before a match REQUIRED with its log read for `went ABSENT` on any port | ✅ **answered 2026-10-01: the encoder check** — a side-mate counts as evidence the train turns only while its ENCODER moved in the last 5 reads; built in T4 Part 3 (its early step). A hard wall push then gets one 1 s fight cut while no motor reads above the 2.4 A over-current limit (above it the over-current cuts come first and the fight adds none — *corrected 2026-10-08 at T4 Part 3's review: this cell said "below 2.5 A", but the build's line is the 2.4 A over-current limit; P2-19e pins 2.3 A (one fight cut) and 2.45 A (the over-current cuts, no fight cut)*); port 18's gap (2) stays until its slipping gear is fixed |
| COMP-b (its review, 2026-10-01) | **C5**: one auton entry the registry refuses (e.g. a hard stop past the period) refused the whole BOOT, driver control included | ✅ **answered 2026-10-02: drop just that auton** — named on screen and in the log, left off the menu; every other auton and driver control still work; built in T4 Part 3's late step |
| R5a Part C (its review, 2026-10-04) | **R5a's five**: (a) refuse a GAINS segment that does not move (today it only warns); (b) shrink `turnBound`, or fit the IMU heading minus the encoder rotation; (c) size `maxResidualRms` from a bias tolerance, or report a reading-σ beside the gains; (d) track the whole run's pose for the clear space; (e) robot two's slipping port 18 makes a blocked robot refuse as a fight | open — they shape Part C (Calibrate runs the steps, after CAL1); the coordinator brings them with Part C's brief |
| T4 Part 3 | **A**: what the SD log copy holds | ✅ **answered: events and summaries only** |
| T4 Part 3 | **B**: does the competition program keep an SD copy? | ✅ **answered: yes, appended** |
| MECH | the core's questions (Q1–Q6, Q8, Q13, Q18, Q25, Q28–Q30) | ✅ **answered 2026-09-29 ~03:50**: bindings only in the robot file; a sensorless claw grab by time, reported unconfirmed; mechanism sensors with the mechanisms; PTO: name the later change; no 3-wire expanders yet; the most recent input wins; three-position levels stated by the team; typed names checked at build; F3/F4 re-scoped; the bench page and direction check in the core; **the list is complete — write the core's brief**. Homing for robot two's pivots: not sure yet |
| G3a | its six decisions (D1–D6); robot two's front | ✅ **D1–D6 answered 2026-09-29: all as recommended.** ✅ **The front, answered 2026-10-06:** never asked in the panel — the end that leads Calibrate's push is the front, with a flip warning (built in CAL1). ✅ **D6′–D10 answered 2026-10-29: all as recommended** — two lanes after MECH-A5 (on a budget warning the panel lane yields first), robot two's typed table flipped to the camera end in Part 1 (D7: Bench Tests, the boot's cross-check line and CAL1's typed check read it), a new robot's five programs (D8), the shared marked regions with clashing ids refused (D9), three required facts (D10) |
| Q2 / Q3 / Q6 | sensor placement — **Q2 partly answered 2026-10-08:** AI Vision in the FRONT, one distance sensor on EACH SIDE (which robot, the measured offsets and the GPS still open); robot one's cartridge; the `MotionDeps` version question | Q2 partly; Q3, Q6 open |
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
| 2. T4 (Part 3 landed 2026-10-08) | 0 |
| 3. the plan chunks MECH would change: T3's macros and presets, F3 (rough); F4 not sized | ~2½–3 |
| 4. the panel as planned: G3a as briefed (5½–6), G3 at its plan-of-record scope (~5½–8, rough); H4 not sized | ~11–14 |
| 5. sensors (rough) | ~3 |
| 6. the briefed library chunks | ~6–7½ |
| 6. IMU1 (its brief, 2026-10-22: Part A ~1¼, Part B ~½). *2026-10-29: this row was ~½–1 (rough); the brief's ~¾ more is NOT added into the plan of record or the totals below — the same rule as the auto-flipper row's note: this table's sums wait for the coordinator's full re-pricing* | ~1¼–1¾ |
| 8–9. T2's close-out, documentation, G4 (rough); PKG1 not sized | ~3–4 |
| **The plan of record** | **~26¼–32¼** |
| + the MECH core, before the panel (its brief, 2026-10-08) | +19¼–26 |
| + G3a amended for the core (new robot, mechanisms with templates, wiring, controls): 5½–6 → ~11½–16 (rough) | +5½–10½ |
| + the rest of MECH, after the panel (ruled; rough) | +19–32 |
| + the panel's extension for the rest (rough) | +2–3½ |
| − G3 re-scoped, ~5½–8 → ~2½–4 (proposed, requirements §5 Q27) | −(3–4) |
| + Export (proposed; rough) | +1–2 |
| + routes in the panel (ruled 2026-10-08; rough; no simulation — VexBuilder's) | +4–6 |
| + DOCS3, the documentation redo (ruled 2026-10-08, after the core and before the panel; after the panel since 2026-10-29; rough) | +5–8 |
| + SER1, the serial log's lines kept whole (HA-189; 2026-10-08) | +¾–1 |
| + the Toggle auto-flipper, VIS1 + AF1, before the panel (2026-10-08; beside it since 2026-10-29; rough, from its research; ~2½–3 of it planned work moved earlier, netted here). *2026-10-18: VIS1's answered brief now sizes VIS1 at ~6½–8½ (this line priced it at ~4¾–6½ — colours only, before the objects of 2026-10-14) and adds ALLY1, our alliance colour, at ~1–1½ (#18f; taken out of AF1, whose ~8¾–11½ is not re-sized until its brief). Neither change is added into this line or the totals below: this table's own sums do not re-add exactly (the plan of record's six non-zero rows add to ~26–32½ against the ~26¼–32¼ stated), so the totals wait for a full re-pricing — the coordinator's, from the briefs* | +12½–18 |
| − T3's macros and F3, which MECH's rest would absorb | −(2½–3) |
| **Everything, as now planned** | **~88¼–134** |

**Not in these numbers:** H4, PKG1 and F4 (not sized); the robot-session-bound items (§7, #7 and #11) and every
robot session; and **verification**. Each result is reviewed, fixed and checked by a fresh verifier before it
merges; the one brief that sizes this, COMP-b's, puts it at about half again (~1 day on ~2).

**Calendar time.** More builders do not shorten the chain of steps that each wait on the one before, so the chain
sets the pace. The milestones along it:

| Milestone | The chain to it | Builder-days in sequence | Weeks (five-day), with verification |
|---|---|---|---|
| **The 40 in on the bench** | COMP-t ✅ (2026-09-29) — waits on your Station M run | — | now |
| **The 40 in through the real `autonomous()`** | COMP-b ✅ (2026-10-02) — waits on Station K | — | now |
| **The driver program finished, Teleop retired, `main.cpp` final** | T4 Part 3 ✅ (2026-10-08; with Part 2b's items) — its robot check is #11 (Stations F and J on the new "shulib Drive") | — | now |
| **Mechanisms (the core) in the library** | the MECH core (its brief: six runs A1–A6; CAL1 beside it, landing before A5) | ~19½–26¼ | ~6–8 |
| **The Toggle auto-flipper (library, host-tested)** | VIS1 beside the core; AF1 after MECH-A4 | ~23–30 | ~7–9 |
| **The panel: make a robot with its mechanisms, no C++** | *THE SPRINT (2026-10-29, later): target 2026-11-21, routes a stretch; MECH-A5 and G3a Part 1 from 2026-10-30, the builds overlapping — the tool lane's builds A5 → 5 → 2 → 4 ~12¼–17 builder-days plus the last part's verification, ~Nov 11–16 if nothing stops the builders. The plan below it is the morning's:* the MECH core through A5 (~17¼–23), then G3a in two lanes to a robot made and driven in the panel (~11–15½, its answered brief of 2026-10-29) — beside A6, the auto-flipper, the wall reset (L6) and driving to a piece (L4), whose pages plug in as each lands (your ruling, 2026-10-29; it was ~42–57 builder-days, ~12½–17 weeks, with them and DOCS3 in the chain) | ~28–38½ | ~8½–11½ |
| **Everything** | + DOCS3 (after the panel since 2026-10-29), the rest of MECH (its three parts side by side where their files allow) and the panel's extension, then G3 → export → sensors → documentation → PKG1 → G4 (re-derived 2026-10-29: the panel's ~30–41½, DOCS3's ~5–8, and the same ~13½–24¾ after them as before; it was ~55½–81¾, ~17–25 weeks; not re-derived for G3a's answered sizes) | ~49–74 | ~14½–22 |

These hold only if the decisions and robot sessions keep pace and the builders are not paused. The work grew on
2026-09-28 because the team lead added generic mechanisms (MECH), the panel's new pages and export, and because
the re-checks re-sized COMP-b, T4 Parts 2–3 and G3; the core-first ruling changes the ORDER (the panel arrives
sooner), not the total. The core grew again on 2026-10-08, when its brief sized it at ~19¼–26 (the rough
range was 10½–18): it carries the 66 build refusals, the read-fault channel, the stall signatures and Bench Tests'
MECHANISMS station that the rough figure did not. Estimates, not promises.
