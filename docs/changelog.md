# Changelog

> **What this is.** Every user-visible change to shulib, newest first, one section per API
> version — each entry says **what changed**, **whether it is breaking or additive**, and
> **what you must do about it**. The version policy itself lives in
> [`include/shulib/version.hpp`](https://github.com/SHU-ROBOTICS/shulib/blob/main/include/shulib/version.hpp):
> the major number moves only for a breaking change to a frozen surface, always with a
> migration note; the minor number moves for additive growth. The
> [Freeze Register](roadmap.md#freeze-register) says which surfaces are frozen at all.
>
> This file starts existing partway through the 2.1 line — the project ran to API 2.1 with
> the version history recorded only in `version.hpp`'s comments, which an outside team on
> 2.0 had no reason to read. Entries below the line marked *(reconstructed)* were written
> after the fact from those comments and the project records; everything above it was
> written when the change landed.

## API 2.9

### 2026-11-17 — PREVIEW in the route builder: the robot drives your route on the field, from the real library

> Written when it was built (chunk RMP1, run RMP1-A2u). **No library change — no API change:** nothing under
> `include/shulib/` or `src/` moved; this is the route builder's page. **Host and browser only: PREVIEW is the host
> test plant's run, not the robot's.**

**What you can now do (additive):**

- **PREVIEW** in the [route builder](route-builder.md) (**Ctrl+Enter**): the real library runs the route on the
  page — in the background, with **Cancel** — and the robot plays it on the field, its path solid beside the dashed
  plan, under a timeline you can scrub (play, pause, step to the step before or after, the speed). Each step in the
  list says how it went — done, failed (with the library's own line), through a wall, cut, skipped — and the steps
  that used a number robot one's facts file has not measured yet are marked PLACEHOLDER. PREVIEW plays robot one;
  robot two's mirrored run is not in it yet.
- How: [guide chapter 22, step 6](guide/22-making-your-first-route.md#6-preview-watch-the-library-drive-it).

**What changed:** robot one's cascade has no levels yet (its facts, typed 2026-11-15), so the route builder's
Mechanism menu now says **"No levels yet: record them at Bench Tests station 13"** and offers what the cascade can
do — home it, wait until it is done — instead of an empty list of levels. A move to a level in an older project file
is red, with the same words.

**What you must do:** nothing.

### 2026-11-16 — the route builder's Controls tab: the driver's buttons and the driving feel, by clicking

> Written when it was built (chunk RMP1, run RMP1-B). **No library change — no API change:** nothing under
> `include/shulib/` moved. One line of `src/common/driver_file.hpp` is new (below). **Host and browser only: no map
> or feel it writes has run on a robot here.**

**Why:** the team's non-coders set the driver's buttons, and "a small push of the stick slow, a full push fast" —
until now by typing C++.

**What you can now do (additive):**

- **The Controls tab** of the [route builder](route-builder.md): a picture of the V5 controller — click a free button,
  pick a mechanism and what it does (run while held with one button or two, a toggle, a level or a position on one
  press, a home); what the build refuses is refused in plain words with the build's own name beside them, and what is
  unfinished or would leave the button map wrong is asked for.
- **Driving feel:** the drive mode, and for each stick direction the dead zone and the curve, with a live graph of the
  library's own formula (the same numbers as the library's C++, checked on the host at 9 630 points); the ramp, what
  the drive does when the sticks are let go, the re-zero button.
- **Export** writes the driver file's three marked parts — which button does what (`kControls`), the button map at
  the top of the file, the feel (`kFeel`) — in Edge or Chrome with the shulib folder connected; in Firefox it hands
  back the updated driver file you pick; or it copies them.
  How: [guide chapter 22](guide/22-making-your-first-route.md#the-drivers-buttons-and-the-driving-feel).
- **The top bar fits every laptop screen** (fixed at the merge, 2026-11-17): from 1101 px up, PREVIEW and Export
  stay whole and apart; at 1600 px and below the folder button keeps only its icon (its words are on the Export page
  too), and at 1366 px and below the "Route builder" words and the "Robot one" chip are not drawn.

**CHANGED (a driver file that compiled can now stop the build — none in this repository does):** `DriverFile`
refuses an out-of-range feel by name (`the feel is out of range`): a `.deadband` below 0 or at 1 and above, or an
`.expo` outside 0 to 1. Outside those the curve overshoots full speed or turns the stick around.

**What you must do:** nothing, unless your driver file's feel is out of range (fix the number the message names).
The driver files carry the `control words` and `feel` markers from robot two's update; until then the tab offers
Copy only. "shulib Drive" (and the competition program) now print the feel the driver file runs on their own boot
line, `driver feel: …`, made from its `kFeel` — so a feel other than the library's default builds.

### 2026-11-16 (built 2026-11-16) — robot two is robot one's twin; the calibration record names its robot; one skills route for both robots

> Written when it was built (chunk ROB1, run BT — the brief's G1-B and G1-T together). **Added (no version
> bump):** one optional fact and two functions in the calibration library — a layer no row of the
> [Freeze Register](roadmap.md#freeze-register) freezes; a Calibrate that states no robot name writes exactly
> what it always did, and the record's format is unchanged. **Host- and build-gated only: nothing has run on
> either robot.**

**Why:** the team lead, 2026-11-13: "robot 1 and robot 2 are the same build and just the auton routes are
different". And two robots of one build, wired the same, pass each other's port check — so a card moved
between them would boot the other robot's calibration.

**ADDED (library, `calibration/`):**
- **`CalibrateFacts::recordName`** (optional, last) — stated, Calibrate starts the record's provenance with
  `robot=<name> `; when the card's record names another robot or none, the card line and the front check each
  add one line saying so. A name that is not 1-24 lower-case letters, digits or `-` is a facts problem
  (`FactsList::RecordName`).
- **`calibration::isRecordName(name)`** and **`calibration::recordRobotName(provenance)`** (with
  `kRecordNameKey`, `kRecordNameMax`) — the rule, and the name a record carries (empty for an older record).
- **`calibration::kProvenanceKept`** (40) — the calibrator's cut of the provenance, named (it was a literal);
  the record name sits within it.

**BOTH ROBOTS (the team's files and the programs, not the library):**
- **The boot refuses a record that names another robot, or none**, before the port check and with no fall-back
  to the typed table: `the card's record is ANOTHER ROBOT'S` / `the card's record names NO ROBOT`, each with
  "run shulib Calibrate (slot 4) on THIS robot". Each robot's Calibrate writes its own name (`robot-one`,
  `robot-two`). [Guide chapter 19](guide/19-calibrating-your-robot.md#two-robots-one-card-each-the-record-names-its-robot).
- **`src/team/skills_route.{hpp,cpp}`** — the team's skills route, written ONCE in robot one's numbers and
  compiled into both robots: robot one registers it as written, robot two MIRRORED (one line). It is empty — the
  route is the students'; the route maker writes only between its `BEGIN` and `END` lines.
- Each driver file carries `[control words …]` and `[feel …]` markers around its header map and its feel.
- The dropped-autons line and the Auton Tester's banner name the BUILD's robot's autons file (they chose it on
  the claw words, so robot two's would have named robot one's file); on both robots a second line says the
  shared skills route is fixed in `src/team/skills_route.cpp`.
- **A mirror swaps the robot's SIDES, not its mechanisms:** robot one's right side toggle's job is robot two's
  LEFT toggle in the mirrored route, so a side-toggle step picks the side with
  `competition::isMirrored(ctx) ? leftToggle : rightToggle` ([cookbook chapter 9](cookbook/09-one-route-two-robots.md)).
- The route preview hatches robot two's piston steps (their stroke times are owed in its facts file) and lists
  every fact robot two still owes under PLACEHOLDER, as it does robot one's.
- The route preview's stamp hashes the robot's autons file together with `src/team/skills_route.cpp`, so an edit to
  the shared route shows in it; the route builder describes robot two's cascade levels from robot two's own facts.

**ROBOT TWO — robot one's twin** ([guide chapter 21](guide/21-adding-your-robot-by-hand.md#robot-two-robot-ones-twin)):
- `src/robots/robot_two_facts.hpp` (new): robot one's 35 facts, each placeholder robot one's value; every one
  OWED but Calibrate's two thresholds (robot one's, typed for the same drive). Every robot-two build refuses, one line per owed fact its program needs
  (`robot two: OWED -- … (src/robots/robot_two_facts.hpp)`), until they are typed.
- Its mechanisms, driver file and autons list are robot one's (its region word for word; the builder's map);
  its chassis table is robot one's design (the cartridge UNSET until read off robot two). `test/robot_twin_test.cpp`
  fails the suite when the two robots differ anywhere else. Its claws (two pivots on ports 6 and 7, a pneumatic
  claw on A) and its measured drive table are retired — the host suite keeps them as its example claw robot.

**What you must do:** fill the [facts form](robot-facts-form.md) again for robot two; run shulib Calibrate once
on each robot (a card written before today names no robot and is refused at boot). A button or an auton changed
on one robot is changed on both.

**The src build gate:** robot two's six builds join the facts proof (each refuses naming exactly its own owed
facts; all twelve robot builds link in the gate's typed copy), and `self-test` runs in a copy of the tree with
robot two's facts typed there — the repository is never planted.

### 2026-11-16 — the pair check refuses a tank step that drives AND turns around

> Written when it was built (chunk MIRROR1, run B). **A stricter host check (no API change):** a mirrored tank
> route that passed before can now fail the suite — see What you must do. The host test suite (`test/`), and one
> comment each in `chassis/chassis.hpp` and `competition/field_mirror.hpp`. Host test plant only.

**Why:** on a tank, one step that drives AND turns about 180° ends beside its target, and robot two misses on
the other side: on a 36 in drive each robot is about 7 in off, and robot two ends 14.2 in from robot one's mirror
image. The team lead, asked whether to block it so students write turn, then drive (2026-11-15): "Block it".

**Changed:** on a TANK the pair check (`test/mirrored_pair_check.hpp`) now fails a mirrored route — a RELATIVE one
too — whose robot two logs the `[MIR]` half-turn line for a leg: a `driveTo`, `reverseTo`, `moveTo` or trajectory
leg within 1° of 180° that drives at least 0.5 in. It names the step and the steps to write:

```text
auton 'one step': motion 2 (MoveToPose to (-36.0, -55.7)) at 1.54 s turns 180.0 deg while it drives, which a tank cannot mirror -- turn first, then drive: face(-36.0, -55.7), then driveTo(-36.0, -55.7).
```

Not refused: a `turnTo`, and a drive that can strafe (an X-drive, an H-drive). **What you must do:** if robot
two's route fails this way, write the steps the sentence gives — turn first (`face`, or `faceAway` before a
`reverseTo`), then drive ([cookbook chapter 9](cookbook/09-one-route-two-robots.md)).

### 2026-11-16 (built 2026-11-15, reviewed and fixed to 2026-11-16) — the Toggle flip from an auton: the robot file's Toggle block, the robot's camera for the step, and the everyday call

> Written when it was built (chunk AF1S, run A2). **No version bump** (the coordinator, at the merge: every change is additive and no frozen row is touched).
> **Additive:** every name below is new, or a member appended with a default to a type no row of the
> [Freeze Register](roadmap.md#freeze-register) freezes, or a verdict appended to the step's own list; a robot
> file with no Toggle block builds exactly the robot it did, and its boot line is byte for byte what it was.
> **Host- and build-gated, NOT run on a robot:** no Toggle, no flipper and no camera reading of a real Toggle
> exist in this project.

**Why:** the skills Toggle step (API 2.8's entry of 2026-11-13, "the Toggle flip for a SKILLS run, as one route
step") could be called only over handles a hand-wired program passes. A team's skills route now calls it from
its auton, with the robot's own camera.

**Added:**

- **The robot file's Toggle block** — `config::SensorsConfig::toggleCamera` (type `vision::ToggleCameraConfig`,
  `vision/toggle_camera.hpp`): the Toggle tracker's numbers and the camera's read period, beside the camera's
  own block, every number UNSET by default. `vision::toggleCameraCheck` (its answer a
  `vision::ToggleCameraCheck`) / `toggleCameraProblem` check it, in one order, at build (`static_assert`) as well
  as at boot; `vision::toggleIdsProblem` is its colour-id check alone. `RobotConfig::valid()` does not check it.
- **The robot's camera for the step** — `robot::Robot` builds a `vision::ToggleCamera` (one `CameraFeed` over the
  camera, the `CameraModel` of its measured mount, the tracker's numbers) when the block is declared, a camera is
  declared and built, its mount is MEASURED (and inside the geometry's limits), the tracker's and the read
  period's numbers are accepted and the three colour ids are colours the camera is taught; otherwise it is not
  built, with the first reason (at most `vision::kToggleCameraReasonBytes`). **A Toggle block the step cannot
  use never refuses the robot** — it boots and drives. `Robot::toggleCamera()` / `toggleCameraAbsence()`. It
  asks the provider for nothing more.
- **The context's second optional slot** — `chassis::RobotContextConfig::toggleCamera` /
  `RobotContext::toggleCamera()` (appended, nullptr by default: a robot file with no Toggle block), carried by the
  motion scheduler's shadow context like the wall sensors' slot, so `ctx.chassis.deps().ctx` reaches it.
- **The everyday call** — `competition::checkAndFlipToggle(ctx, request)` and `competition::lookAtToggle(ctx,
  request)`: the explicit form's visit, every rule the same but the two below, over the auton's context — the
  camera slot, the mechanism service, the run guard, the entry and the telemetry. No slot, or one not built, is
  NO CAMERA naming why (after NOT SKILLS). An optional third argument, `competition::TogglePoseBound`, states how far the estimate
  may be off, so the block's position check can run under a FIELD start (without it it is skipped; under any
  other start the step drops it and says `wall and position not checked (relative start)`). In a mirrored auton
  it visits the image Toggle, as the explicit form does.
- **`ToggleVisitVerdict::SpotsTooClose`** (appended) — the everyday call's answer when the request's
  `lookFrom - pushFrom` is twice the Toggle block's `frozenMoveIn` or less (two numbers from two files): nothing
  read or moved, the route's `onFail` applies. It is checked after NOT SKILLS and NO CAMERA and before FLIPPER
  NOT READY, so a match auton still says NOT SKILLS. **`competition::toggleVisitBlockProblem(request, block)`** checks
  that pair at build, in a `static_assert`. (The explicit form, where one caller holds both numbers, still
  treats it as a programming error.)
- `competition::ExplicitToggleCamera::absence` (appended): NO CAMERA's words for a camera the caller knows is
  absent.
- **The boot line**, with a Toggle block declared: the camera's words end `, toggles ready` or
  `, toggles OFF (<why>)`; the camera's own words keep their length. (The brain's programs print the line cut
  to 220 characters; the visit's `[TGL]` line carries the reason, cut to 100 characters, and
  `toggleCameraAbsence()` whole.)

**What you must do:** nothing, unless you want the checked flip from an auton: give the robot file a camera
block (its port, the colours taught on the Toggle's real faces, a MEASURED mount — neither of the team's robot
files has one yet) and a Toggle block ([guide chapter 20](guide/20-the-camera.md#the-robot-files-toggle-block)),
measured on each robot that runs the route, and write the visit as
[cookbook chapter 8](cookbook/08-skills-toggles.md) shows. The plan for the first competition stays the team's
own blind two-push flip until this step has flipped a real Toggle.

### 2026-11-17 — Calibrate on a drive with many motors per side: a robot's own pulse thresholds

> Found on robot one's first Calibrate runs (2026-11-16): on its 10-motor drive one pulsed motor drags its four
> side-mates and turned the robot only ~1.0 deg at 9 V, under the 2.0 deg the calibrator needs, so every run
> stopped with IT ROLLED. **Added (no version bump — an additive member in an unfrozen header):**

- `CalibrateFacts::calibrator` — the calibrator's thresholds for THIS robot (default: `CalibratorConfig`'s). Calibrate
  builds its calibrator with them; `factsProblem()` refuses, before anything is powered, a pulse floor that the drift
  of a still-looking robot could cross during a pulse.
- Robot one's ledger types two chosen values (0.6 deg pulse floor, 1.0 deg/s rest rate) and its chassis table passes
  them on; robot two, its twin, types the same two. Every other robot keeps the defaults.

**What you must do:** nothing, unless your drive has many motors per side and Calibrate stops with IT ROLLED —
then lower `pulseYawFloor` (and `restYawRateFloor` with it) in your robot's facts.

### 2026-11-16 (built 2026-11-15) — the docs by task: a new menu, a front page that asks "What do you want to do?", and limits as plain facts — no API change

> Written when it was built (chunk DOCS3, run N1). **Documentation only** — nothing under `src/` or `include/`
> changed. **What you must do:** nothing; every page keeps its file name and its address.

- **The site's menu is by task** — Robot, Routes, Mechanisms, Troubleshoot, Learn, API reference, Project —
  and shows titles, not chapter numbers.
- **The front page asks "What do you want to do?"** and links each answer, with a picture of a new robot's
  first week drawn from guide 21's "The order on the robot".
- **New landings:** "How it works" (the guide's ideas in order, and every page by its old chapter number),
  "Recipes: which to read" (the cookbook), and "How these pages stay true" (the maintenance rules).
- **Limits are plain facts where they apply;** the status disclaimers that opened ten pages are gone (what
  has run on a robot is listed in guide 14).
- **Links:** every "Chapter N" on a reader page is a link; every glossary term has its own link
  (`guide/15-glossary.md#pose`); on the site, every register row does too (`hardware-assumptions.md#ha-136`).
- **New build checks:** every public page is in the menu exactly once (`tools/doc_readability.py check-nav`);
  the tone rule (`check-tone`); a link to a missing anchor fails the site build.

### 2026-11-16 (built 2026-11-14, reviewed and fixed to 2026-11-16) — the route builder: build the skills route by clicking, export the C++ (in the browser, on Windows and Linux)

> Written when it was built (chunk RMP1, its first run, RMP1-A1). **No library change — no API change:** nothing
> under `include/shulib/` or `src/` moved; this is a tool beside the library. **Host and browser only: no route it
> writes has run on a robot here.**

**Why:** the team's non-coders write the skills route — and until now that meant typing C++.

**What you can now do (additive):**

- **The route builder** — a web page on the docs site ([its page](route-builder.md), the "Route builder" tab), and a
  one-file download for offline use. Place robot one's start on the real field; add drives (turning to face the
  point first on a tank), turns, curves (the library's own curve, drawn and checked), cascade levels, homes,
  pneumatic positions, timed motor runs, waits; give any step "if it fails: stop / carry on / skip ahead". The page
  marks in plain words — with the library's own sentence beside them where it has one — every step the robot would
  refuse (a move before the cascade is homed, a wall in the way, a curve the library will not drive), with one-click
  fixes.
- **Export** writes the route as commented C++ into the ONE marked region of `src/team/skills_route.cpp` (Edge or
  Chrome, the shulib folder connected; keeping the file's own line endings, never overwriting a hand edit without
  asking), or copies it (any browser). **Save** writes the route into `src/team/route_maker.json`.
- How to use it: [guide chapter 22](guide/22-making-your-first-route.md).

**What you must do:** nothing. Robot two's update adds `src/team/skills_route.cpp`; until it is in your copy of the
repository, the route builder offers Copy only.

### 2026-11-15 — the route preview previews robot one

> Written when it was built (chunk RPV1, the switch to robot one). **No API change**: nothing a robot program
> compiles changed, and `src/` is untouched. **Host test plant only — nothing here has run on a robot.**

**CHANGED (a host tool):** `python3 tools/route_preview.py "<auton name>" --robot one` now runs that auton from
`src/team/robot_one_autons.cpp` on robot one's own mechanisms (each on a fake built from its own declaration, the
cascade included), on the same stand-in drive. Every fact robot one's facts file (`src/robots/robot_one_facts.hpp`)
still OWES is a placeholder (the preview uses it wherever it needs that number), so the page and the terminal list
each one under PLACEHOLDER at the top; a fact leaves the list once it is typed, a value typed as "chosen, not
measured" included (the page does not mark it). Robot one's pages go to `build/route_preview/one/` (the two
robots share auton names). Until now `--robot one` said robot one had no build and drew nothing.

**What you must do:** nothing. [Chapter 16](guide/16-testing-an-auton-on-the-bench.md#before-the-bench-previewing-a-route)
shows the command.

### 2026-11-15 (built 2026-11-15) — robot one's real mechanisms and its first typed facts; a ONE-button hold for a power mechanism

> Written when it was built (chunk ROB1, run C2 — the team lead: "we need to have the bot coded tonight").
> **Added (no version bump):** one new word in the mechanism core's driver controls — a layer no row of the
> [Freeze Register](roadmap.md#freeze-register) freezes; every existing driver file compiles and runs exactly as
> before (a two-button hold is unchanged). **Host- and build-gated only: nothing has run on robot one.**

**Why:** robot one was built by 2026-11-15, and its builder read its ports, cartridges, gearing and 3-wire letters
off it and gave its button map. Its build had refused every program until those facts were typed and every
non-drive motor declared.

**ADDED (library, `mechanism/driver_inputs.hpp`):**
- **`whileHeld(powerMechanism, button, volts)`** — a ONE-button hold-to-run for a POWER mechanism: held, it runs
  ONE way (+volts; which way the motor turns is its declaration's `.motors` direction); let go, it stops. It takes
  no second button, so the button beside it stays free for another control. `Control::oneWay` marks it; the
  boot line reads `<name>: hold-to-run L1 (one way), 12 V`. On an arm, a lift, a pneumatic or a flywheel it is
  refused by name (`whileHeld_with_one_button_is_for_a_power_mechanism_an_arm_or_a_lift_needs_two`).

**ROBOT ONE (the team's files, not the library):**
- **Typed** in `src/robots/robot_one_facts.hpp` from the builder's answers ("the team's builder, 2026-11-15, read
  off the robot"): the cascade's motors on ports 10 and 1, facing each other (port 10 taken as Forward —
  station 13 confirms), BLUE, direct to the spool (typed 1:1); the side toggles' ports 9 and 2 and their GREEN
  cartridge; the claw on 3-wire A, the front toggle on B, the ramp on C; the cascade's levels as NONE (manual
  for now); and the non-drive motors {10, 1, 9, 2}. **Chosen, not measured:** the cascade's starting cap 2.0 A
  and stall signature 1.5 A below 5 rpm for 0.3 s, and the flipper's kind (pneumatic: the front toggle) — the
  coordinator; the side toggles' starting cap 1.5 A, each piston's starting stroke time 0.5 s, and the cascade's
  own **homing** signature, 0.45 A (chapter 18's starting value for a gentle push — the 1.5 A jam signature
  could never end a gentle home, so the two are separate facts). The drive table's cartridge: BLUE.
- **Declared** in `src/robots/mechanism_table.hpp`: two side toggles (`Power`), the claw, the front toggle (the
  `Pneumatic` named `"flipper"`) and the ramp, each piston starting in the position its solenoid holds unpowered.
- **The driver file is the builder's map**: R1 / R2 the cascade while held, A the claw, B the front toggle, X the
  ramp, L1 / L2 the right / left side toggle while held, Y the re-zero, the d-pad free; the cascade homes at the
  first driver start (the one line marked `[delete-able]`). Its three numbers nobody gave (the cascade's
  12 V, the side toggles' 12 V, the home's 3 s) are marked "chosen, not measured".
- **Still owed** (each refuses by name): which port is the right side toggle (Stage A — Bench Tests and
  Calibrate wait for this one only); the cascade's two rows, hold volts, homing push and unhomed volts, each side
  toggle's direction, and which line level opens each piston (Stage B).
- The Drive screen's mechanism row prints a COMPACT form when a list's full row cannot fit the screen's 50
  characters (robot one's six mechanisms): `MECH 10H34 9B31 2B30 CLAW:C FLIP:O RAMP:U` — chapter 21 has the legend.

**What you must do:** nothing — robot two's files and its driver file are unchanged, and it behaves as before
(its packages carry the library's new one-button branch, never taken by its two-button holds). A driver file
that wants a one-way control on one button writes `whileHeld(roller, Button::L1, 12_volt)`. Robot one: type
the right side toggle's port, then build Bench Tests and Calibrate ([Chapter 21](guide/21-adding-your-robot-by-hand.md)).

### 2026-11-15 — shulib Tune: the READY button shows its whole label

**Fixed:** Tune's first page drew its READY button as "READY - I AM CL" — the label box held 15 characters and the label has 18. The box now holds 23, and a test checks every one of the page's 24 button labels fits. **What you must do:** nothing; rebuild shulib Tune.

### 2026-11-15 (built 2026-11-13, reviewed and fixed to 2026-11-15) — one route, two robots: robot two runs robot one's skills route MIRRORED across the field's skills diagonal

> Written when it was built (chunk MIRROR1, run A). **API 2.9** — numbered by the coordinator at the merge. **Added (nothing to do):** every name below is new; the
> Chassis (Freeze Register row F6) gains a read-only `reflection()` and the `chassis::MirroredRun` scope —
> F6-additive, every existing verb unchanged unless a mirrored run is open (an unmirrored run is bit-identical,
> pinned) — and the Routine (F10) does not change at all. The version number is the release's to set.
> **Host- and build-gated, NOT run on a robot:** robot two as robot one's twin does not exist yet.

**Why:** the first competition (2026-11-21) is SKILLS ONLY, and both robots — the same build — run the skills
route at the same time, one in each red Quadrant (the game manual's `<VURS2>`, `<VURS3>`, `<SG1>`). Those two
Quadrants are mirror images across the field's diagonal from the red/right corner to the blue/left corner, so
the students write the route ONCE and robot two runs it mirrored — one registration line instead of a second,
hand-reflected copy to keep equal through every edit. The team lead's go (2026-11-13): "do what is best they
should play onto one side when they need to when mirrored".

**Added:**

- `math::Reflection` (`math/reflection.hpp`) — the field reflected across a line through the origin at a
  multiple of 45° (exact in position, a reflected reflection gives the position back, never `-0.0`), or the
  identity: a point, a `Pose2d`, an `Angle` (`2a - h`, re-wrapped), a `ChassisSpeeds` in either frame.
- `competition::mirrored(entry, mirror)`, `competition::isMirrored(ctx)`, `competition::FieldMirror`,
  `mirroredWall`, `mirroredSide`, `runReflection`, `seatedStart` (`competition/field_mirror.hpp`), and
  `competition::override2026::kSkillsMirror` — Override's skills mirror, `(x, y, h) -> (-y, -x, -90° - h)`, with
  `toggleImage` / `mirroredToggle` (`competition/override_mirror.hpp`; the season's walls and Toggles checked to
  close under it at compile time). Robot two's autons file:
  `registry.add(competition::mirrored(skillsRoute(), competition::override2026::kSkillsMirror));`.
  `mirrored()` handed a TEMPORARY `FieldMirror` does not compile (a deleted overload): the entry keeps the
  mirror's address, so it must be the season constant.
- `AutonEntry::mirror` (appended, default none) and `kMaxMirroredAutonNameChars` (25 — the button reads
  `<name> SKILLS MIR`); `AutonRunResult::mirrored`.
- `Chassis::reflection()` and `chassis::MirroredRun` (`chassis/chassis.hpp`): while a mirrored run is open every
  FIELD number entering the Chassis is reflected (`moveTo`, `strafeTo`, `turnTo`, both observer overloads,
  `followTrajectory`, `curveTo` / `reverseCurveTo`, `setPose`, `drive`) and `pose()` hands the estimate back
  reflected; `deps()`, `scheduler()`, `lastCompleted()`, the records and the log stay the REAL field.
- `competition::runAuton` seats a mirrored entry at the REFLECTED start (a RELATIVE one by the robot-frame
  mirror, every left a right) and runs its routine AND its end action inside one mirrored run, closed on every
  way out. Its `COMP` lines say `FIELD MIRRORED` and name robot one's start; the result screens say
  `START MIR (...)`; the menu says `<name> SKILLS MIR`.
- The wall reset's everyday call swaps the wall (its image) and the sensor side (LEFT <-> RIGHT) for a mirrored
  auton, its `WALL` line noting `(LEFT sensor, mirrored from red wall RIGHT)`; the Toggle step visits the image
  Toggle (the same Quadrant colour).
- A `[MIR]` Warn when a mirrored `turnTo`, `moveTo` or trajectory leg (a `driveTo` / `reverseTo` is a `moveTo`)
  starts within 1° of a half turn, on any drivetrain — F3's exact-180° tie turns both robots counterclockwise;
  its advice names the verb's fix (a turn: two 90° turns; a leg: turn first, in two 90° turns). And one when
  the driver-control loop finds a mirror still on: it CLEARS it (a library defect — never a refusal).
- Refused by name: the explicit wall reset inside a mirrored run (a precondition naming the everyday call); an
  entry mirrored by two DIFFERENT mirrors (a precondition at `mirrored()`); a mirrored name over 25 characters
  (at registration).
- **Both robots at once:** the host suite runs every mirrored entry robot two registers TOGETHER with its
  unmirrored self on the host test plant and fails — naming the moment, both positions and the drive motion each
  robot was in, robot one's with its target in the route's own numbers — when their 15 in squares come within
  6 in (`test/mirrored_pair_check.hpp`). The guide's rule: keep
  robot one's whole robot 3 in off the diagonal; what lies ON it (the centre goal, two yellow Pins, the red/right
  Load Zone) is for ONE robot, through `isMirrored(ctx)`.

**Proved on the host test plant:** robot two's path is robot one's reflection to 1.1e-12 in pace by pace (a tank,
20 steps of every verb, a trigger of each kind on the same pace; a 16-point curve) and 6.2e-13 in (an X-drive);
the exact-180° tie the one divergence, each pinned with its number — a turn 0.155 in; an X-drive `moveTo` 0.307 in
along the way; a TANK `moveTo` at its END by more the longer the leg — 5.7 in on a 16 in leg, 14.2 in on a 36 in
leg (a `driveTo` straight behind, 5.0 in on 14 in) — while the same drive after turning first in two 90° turns
mirrors exactly; an unmirrored run through `runAuton` bit-identical to the merge base (tank and X-drive, every
verb, hashed).
**Unverified on a robot:** A4 registers HA-258 (the event's field is the manual's mirror), HA-259 (robot one's
build is left-right symmetric where a route uses it) and HA-260 (the robots' own asymmetry, and 3 in a side).

**Pages:** [guide chapter 2](guide/02-the-field-and-coordinates.md#the-fields-mirror-two-robots-one-route),
[chapter 17](guide/17-the-competition-program.md#two-robots-one-route-a-mirrored-auton), chapter 11's `MIR`, `COMP`,
`WALL` and `TGL` lines, chapter 14, the glossary, and [cookbook chapter 9](cookbook/09-one-route-two-robots.md).

## API 2.8

### 2026-11-14 (built 2026-11-12, reviewed and fixed to 2026-11-14) — the route preview: one command draws a whole auton from the team's own file, on the host test plant

> Written when it was built (chunk RPV1, part A — its first two parts together). **No API change**: nothing a
> robot program compiles changed, no library header a robot build includes moved, and `src/` is untouched.
> **Host test plant only — nothing here has run on a robot.** [Chapter 16](guide/16-testing-an-auton-on-the-bench.md#before-the-bench-previewing-a-route)
> teaches it.

**ADDED (a host tool):**
- **`python3 tools/route_preview.py "<auton name>"`** — runs that auton from `src/team/autons.cpp` through the
  same library call the competition program makes, on the host test plant, under the test suite's own bounds,
  and writes ONE offline page (`build/route_preview/<name>.html`) plus the same step table on the terminal: the
  field to scale, the path the plant's robot really drove, every motion (drive, curve, strafe, turn, brake, hold) and
  mechanism step numbered where it began, with its times, mechanism steps that ran while the robot drove as
  their own stretch of path, a refused wall reset in amber, failures in red with the library's own sentence,
  numbers nobody has measured dashed as PLACEHOLDER, and the robot's outline checked against the field —
  THROUGH a wall or a Loader fails the command, a TOUCH is a note. `--place=X,Y,DEG` places a RELATIVE
  auton; `--list` names them all; an auton the program refuses is drawn REFUSED with its sentence; an auton
  that loops forever after a stop is stopped and drawn; one that throws is drawn to the throw, which is
  placed inside a step, during a wait, or after a step. Exit codes 0 / 1 / 2 / 3 (guide chapter 16; 3 is
  the tool's own failure). `--robot two` (the default); `--robot one` says robot one has no build yet (robot one's build landed the same week; switching the preview to it is queued — done 2026-11-15: "the route preview previews robot one"). The
  page's step numbers count from below (every motion and mechanism step — not pauses, waits, `then(...)`
  steps or `startAt`); the library's own quoted lines carry the code's real numbers.

**ADDED (host test infrastructure, outside the API reference):** `hal::fake::FakeTelemetrySink::setOnLine` — an
opt-in hook after each stored line (default none: every existing test unchanged). The Auton Tester's host rig
(`test/auton_tester_rig.hpp`) takes any robot's mechanism list through a new last parameter whose default is
the rig exactly as it was (`test/fakes_by_kind.hpp` builds any list's fakes, an arm, lift or flywheel moving
on its declaration's own numbers, with an assumed time constant where a declaration states no `kA` — every
flywheel, whose declaration has none). `test/README.md` documents both.

**What you must do:** nothing. To preview a route, see chapter 16.

### 2026-11-13 (built 2026-11-11) — robot one in the code: `make ROBOT=one`, its facts file, six builds that refuse by name until its facts are typed

> Written when it was built (chunk ROB1, its first run), and brought up to date when it was integrated onto curves
> (2026-11-13: robot one gets "shulib Tune" too). **No library change — no API change:** nothing under
> `include/shulib/` moved; this is the team's repository growing a second robot. **Host- and build-checked only:
> robot one does not exist in this project's records, and nothing has run on it.**

**Why:** the team's second competition robot — robot one, reported to be a tank like robot two (HA-252, not yet
seen) with a 4-stage string cascade, built ~Nov 14 —
had no build at all, and every robot-one task waited on one. Its builder's facts (ports, gearing, heights) arrive
later; nothing waits on them.

**What you can now do (additive):**

- `make ROBOT=one` (Bench Tests, slot 3) and `make ROBOT=one PROGRAM=calibrate|drive|autontest|competition|tune`
  (slots 4, 1, 2, 6, 7) — robot two's six programs, for robot one ("shulib Tune" because robot one's cascade
  makes it a robot with mechanisms: `one` is in the Makefile's `MECHANISM_ROBOTS`). Robot two's builds
  (`ROBOT=tank`) are unchanged. On robot one a Tune COPY is not pasted into the declaration: its hold volts,
  homing push, stall signature and cap are typed into the facts file
  ([chapter 21](guide/21-adding-your-robot-by-hand.md)); its gains, profile and a narrowed range have no fact yet.
- **Robot one's facts file**, `src/robots/robot_one_facts.hpp`: every number about its mechanisms that nobody has
  measured is written once there, **OWED**, with a placeholder the host suite runs on. Robot one's build of every
  program that needs an owed fact **fails**, one line per fact, naming it
  (`static assertion failed: robot one: OWED -- the cascade's motor ports …`); Bench Tests and Calibrate need
  only "what is plugged where", so they build first and measure the rest — every other program (Tune
  included: it homes the cascade and moves it to its levels), one added later included, needs "how it moves"
  too. A typed fact must carry its own value and its source. [Chapter 21](guide/21-adding-your-robot-by-hand.md) explains it; the
  [robot facts form](robot-facts-form.md) is what the builder fills in.
- Robot one's cascade (`src/robots/mechanism_table.hpp`, its own region below robot two's): a two-motor Lift on a
  measured shape with a row at every level, homed by stalling DOWN under its current cap — every number from the
  facts file. Its flipper is declared once the builder names its kind; all three forms are ready (chapter 21).
- `src/team/robot_one_driver.hpp` — a DEFAULT controller map, the team's to change — and
  `src/team/robot_one_autons.cpp` — the 40 in as a SKILLS auton and the acceptance run; **the skills route is the
  team's to write**.
- **`make` starts fresh by itself when `ROBOT=` or `PROGRAM=` changes** (a stamp in `.d/`): it used to recompile
  nothing and re-link the previous program, which is why the guide said `rm -rf bin .d`. The `rm` is now
  harmless; a refused build leaves no other robot's or program's package to upload (a refused rebuild of the
  same robot and program keeps that build's last good package).
- The programs run THIS build's driver file and autons (`src/common/this_robot.hpp`) and say THIS robot's words
  (`src/common/robot_words.hpp`): robot one's screens and boot lines say "mechanism" / `MECH`, never "claw";
  robot two's words are byte for byte what they were.
- The src build gate runs **fourteen** builds: robot one's six on the real tree must each link or refuse naming
  exactly its owed facts, and in a throwaway copy of the tree with every fact typed all six must link and pass
  every check robot two's builds pass.

**What you must do:** nothing, on robot two. On robot one: fill in the facts form; the coordinator types it.

### 2026-11-13 (built 2026-11-11) — the Toggle flip for a SKILLS run, as one route step: look, flip only when it is not its Quadrant's colour, look again (the explicit form)

> Written when it was built (chunk AF1S, run A1). **Added (no version bump):** every name below is new and lives
> in a layer no row of the [Freeze Register](roadmap.md#freeze-register) freezes. **Host- and build-gated, NOT
> run on a robot:** no Toggle, no flipper and no camera reading of a real Toggle exist in this project.

**Why:** the team lead's rulings of 2026-11-10 — the first competition (2026-11-21) is SKILLS only, and its
reliable form of the Toggle flip is a route step: the route drives to each Toggle, the front camera checks its
colour, and the robot flips it only if it is not already ours. In a skills run "ours" is each Toggle's own
Quadrant's colour (the manual's `<RSC3>`). The plan for the 21st stays the team's blind two-push flip written with
the mechanism steps they have; this step replaces it once it has flipped a real Toggle.

**Added:**

- `competition::checkAndFlipToggle(camera, chassis, guard, service, entry, request, sink)`
  (`competition/toggle_visit.hpp`) — the step over explicit handles; returns a `ToggleVisitReport` (a value:
  the verdict, the colours before and after, the pushes, each flipper step's `StepReport`, the time) whose
  `outcome()` a route's `then()` takes. Its rules, each tested on the host test plant: refused unless a SKILLS auton, a usable camera,
  the flipper ready AT `back` (never moved there by the step) and, with a FIELD start, the robot facing the
  named Toggle's wall; a LOOK for a sure colour; at the spot only when the band's WHOLE range interval lies
  inside `lookFrom ± offBy` (containment — never overlap); nothing more when it is already ours; begun only when
  the run's time left covers every push it may need; drive in WITHOUT blocking, push, `back` on ANY verdict,
  settle, drive out; look again counting only a picture from after the retract once the camera has moved; a
  second push only on the third colour, never a third. FLIPPED means "shows our colour after a FULL stroke" —
  every success line ends `seating not checked`; a short stroke that reads ours is `OursAfterShortStroke`, a
  failure. A camera that shows itself FROZEN on the drive in (the lens moves, the picture does not) stops the
  drive there and nothing is pushed (`FrozenBeforePush`) — with the camera's position check on as well; when
  the drive in would reach the Toggle before the frozen picture is older than `maxImageAge` can explain, the
  step HOLDS before anything touches it (at most `maxImageAge` a push, counted in `worstCaseSeconds`); a drive in or out that does not arrive is `DriveFailed`;
  a pneumatic flipper whose `flipBound` or `backBound` is shorter than its move time plus its confirm window (the
  most its slot waits) is refused as not ready. The
  report says whether the camera's position check ran (`positionChecked`). ONE `[TGL]` line per visit, each
  shape pinned byte for byte by a test.
- `competition::lookAtToggle(...)` (the look alone; it moves nothing), `competition::worstCaseSeconds(request)`
  (the time check's own sum), `competition::toggleVisit({...})` (a request checked AT BUILD — a problem stops the
  build by name), `toggleVisitRequestProblem` / `toggleVisitProblemOf`, `bandAtLookSpotProblem`,
  `towardWallRadians`, `oursFor`, `toggleVisitWord`, and the types `ToggleVisitRequest`, `FlipperGoal` (an Arm's
  or a Lift's preset, a Pneumatic's position, or PASSIVE — a fixed bar the drive itself strokes),
  `ToggleVisitVerdict`, `ToggleVisitReport`, `ExplicitToggleCamera`.
- `vision::override2026::ToggleId`, the handles `toggles::kRightWall` / `kLeftWall` / `kBlueWall` / `kRedWall`,
  `QuadrantColour`, `kToggleQuadrantColour` (the right and red walls' RED, the left and blue walls' BLUE — a
  reading of the manual's Figure Q-1 and pA15), `toggleIndex`, `quadrantColour`, `toggleName`
  (`vision/override_elements.hpp`, data only).
- Test-only: `sim::TogglePlant` (`sim/toggle_plant.hpp`) — the host Toggle: three faces in a roll order, a
  stroke that rolls one face (two when over-rolling, none when sticky, stuck part-way when jammed), a passive
  stroke, a seat time, contact, and the game's colour (yellow while touched or unseated); it draws its true
  faces into the host camera model.

**What it cannot catch** ([guide chapter 14](guide/14-what-it-cannot-do-yet.md#the-toggle-flip-in-a-skills-route-what-it-cannot-catch)):
a Toggle stuck part-way reads as set; a lookalike the camera's checks cannot tell from a face; our other robot
flipping the Toggle between the look and the push; a pneumatic flipper's stroke (nothing senses the rod — a
blocked cylinder counts as a full stroke); where the flipper lands (the route's driving). No team auton can call
it yet: the robot's camera reaches an auton in the next part. Guide
[chapter 20](guide/20-the-camera.md#checking-and-flipping-a-toggle-in-a-skills-route) explains the step;
[chapter 11](guide/11-reading-the-diagnostics.md#the-toggle-visit-tgl) reads its line.

**Hardware Assumptions:** HA-231, HA-232, HA-234 and HA-235 (AF1's, reused) and HA-243..HA-245 (new) registered;
AF1's HA-228..HA-230, HA-233 and HA-236 and ALLY1's/L4's HA-237..HA-242 held. **None is measured.**

### 2026-11-13 (built 2026-11-03, reviewed and fixed to 2026-11-12) — what the AI Vision camera means: the Toggle's colour (or UNKNOWN, with the reason), one named piece (or none), and one read per period for both

> Written when it was built (chunk VIS1, part A2 of three: what the camera means; the device was part A1,
> robot two's camera and the bench page are part B). **No version bump — every change is ADDITIVE:** three
> new headers and two new members of `vision::CameraModel`, a class in no row of the
> [Freeze Register](roadmap.md#freeze-register) (row F15: the camera's frames and `vision/*`, NOT FROZEN).
> **No program uses them yet** — the automatic flip, driving to a piece and our alliance colour are later
> chunks — so every program builds and drives exactly as before. **Host only — no AI Vision sensor has run
> this code;** every number below is the host test plant's camera model, looking through a true camera set
> inside the stated tolerances. [Guide chapter 20](guide/20-the-camera.md) is the whole story.

- **`vision::CameraFeed`** (`vision/camera_feed.hpp`) — the program's own loop calls `tick()` once per pass;
  the feed reads the camera at most once per `minReadPeriodSeconds` (the team's number; UNSET refused),
  never twice in one call, and hands the SAME frame to every consumer through `latest()` — the latest READ,
  a fault frame included, never the last good one held. `stats()` counts the reads by status and times
  each one with the loop's clock (the last and the slowest — what HA-203 needs from the bench). No task, no
  allocation.
- **`vision::ToggleTracker`** (`vision/toggle_tracker.hpp`) — `update(frame, poseBound)` / `view(now)` →
  `ToggleView {colour, why, heldSeconds, bearing, range, positionChecked, toggleIndex}`: RED, BLUE or
  YELLOW only when one band of that colour passed every gate (kind and id; a long, flat band taller than
  its edges' jitter; not cut by the picture — a side-clipped band judged only when it spans the centre
  column; ONE camera of the tolerance box explains its two edges as the face's lower edge and top edge
  with the face's lean, and its length as a Toggle's; not the Toggle's bottom face; inside the bearing and
  range windows; optionally, on a Toggle's face line) on every usable read for `holdSeconds` — the SAME
  band throughout (its direction and range from the lens agreeing read to read) — with a usable read
  within `aliveSeconds`; otherwise UNKNOWN with one of twelve reasons (two colours, another
  colour cut or maybe a bottom face beside it, another colour directly above, camera full, frozen, silent,
  not held yet, ...). A colour is the face the camera SEES, never a "set" Toggle. Every threshold is the
  team's (`toggleTrackerConfigProblem` refuses an UNSET one by name). `ToggleView::range` is a bound over
  every camera of the tolerance box. `explainAsToggleFace()` gives a bench page the library's measurements
  of one band with no team threshold. Telling the bottom face from the field face rests on the camera
  detecting the field face whenever it detects the bottom face (HA-199, for the bench).
- **`vision::PieceFinder`** (`vision/piece_finder.hpp`) — a route's `PieceTarget {classMask, x, y,
  toleranceIn, want}` (refused by `pieceTargetProblem` when unset) and the team's `PieceFinderConfig`
  (`pieceFinderConfigProblem`); `answer(now)` → FOUND (exactly one candidate of the type and posture whose
  whole error disc is inside the tolerance, none on the border, held), NONE, AMBIGUOUS, NOT IN VIEW (the
  whole tolerance disc, at the floor and at a Pin's height, not certainly in the picture) or UNKNOWN (no
  target set among its reasons) — never the nearest of several. `sightPiece()` interprets one Object: ON THE
  FLOOR, RAISED, LYING or UNSURE, its bearing, ranges and field fix; a box narrow enough to stand is ON THE
  FLOOR only when its width is a Pin's at the range it agreed on and no camera could see it lying toward
  the lens. `minScore` and `viewMarginPx` mean something at 0, so their UNSET is −1.
  **`PieceFinderConfig::raisedMinIn`** — the lowest a raised piece sits on the field (a Goal's top), read
  from the manual's field drawings — is a team number this part added beyond its brief: agreeing with the floor is
  not enough far out or from a high lens (a Pin on a Goal agreed through a camera at the tolerance box's
  far edge, and its fix missed it), so ON THE FLOOR also needs no camera of the box able to see it raised
  that much.
- **`vision::CameraModel::groundRangeAt()` / `sizeRangeAt()`** — the ground and size ranges for ONE camera
  of the tolerance box, for a consumer that checks a consistency one camera at a time; `groundRange()` and
  `sizeRange()` are now their hulls (unchanged answers).
- **What you must do:** nothing — no existing call changes. A consumer that wants the camera builds a feed,
  a tracker and a finder from the team's numbers and ticks the feed from its own loop (chapter 19).
- **Tested** (host only; the suite 2,815 → 2,944 cases): the tracker's gates against their lookalikes
  through the host camera model; the bottom face never reported, including with the robot turned −30..+30
  degrees (390 parkings, 0 wrong colours) and with the Toggle's real lower shape (2,288 parkings, 0 wrong) —
  and, the field face's detection removed, reported (the stated dependency, pinned); the mount table D9's
  advice rests on (a 10-in level lens reports from ~70 in to ~12 in; a lens at 12¼ in level never); floor
  Pins across the picture never RAISED, Goal Pins and lying Pins never ON THE FLOOR, every fix holding the
  true centre (true cameras at the tolerance box's edges, box edges jittered by 1 px), and each term of a
  fix's far end but the off-axis factor shown needed by a near-exactly measured camera; the decision
  thresholds' edges landed on exactly (the hold, alive, aspect, window, colour-above, bottom-face, frozen,
  length, position-gate and finder in/border/out edges); every config refusal by its words. Mutations:
  see the run's record — the survivors are named there, each judged. **Open:** a Pin about 30 degrees off
  the camera's axis, close in, seen by a near-exactly measured camera with its box's bottom edge a pixel
  low (inside the stated jitter) — its field position missed it by 0.02 in past the stated radius, because
  the box's centre column is the Pin's TOP's, not its middle's; the finder's arithmetic for it is not fixed
  yet (no program uses the finder before drive-to-piece).

### 2026-11-13 (built 2026-11-11) — triggers partway through a drive: "at 60 % of this drive, start raising the lift"

> Written when it was built (chunk STEP1-B, the route maker's library half, part B). **API 2.8** — every
> change is ADDITIVE, on the additive paths of F6 and F10, so the minor goes up (numbered by the coordinator at the merge). **Every 2.6
> program compiles and runs exactly as before:** nothing that exists changed shape or behaviour — a drive
> with no trigger takes the same path it always took, and the new observer overloads, unused, leave a
> leg bit-identical to the plain verb (every pace's pose, the exit tick and the record compared exactly,
> on two host test plants). **Host only — nothing here has run on a robot;** how soon after its mark a
> real mechanism starts (A4: HA-217) and how close the estimate's "arrived" is to the tape (HA-218) are
> unmeasured. [Chapter 9](guide/09-the-recipe-api.md#partway-through-a-drive-triggers) teaches it;
> [cookbook 7](cookbook/07-mechanisms.md#start-the-lift-partway-through-a-drive) has the recipe.

**ADDED (Freeze Register row F10's additive path — NOT pinned: the trigger words stay OPEN, like the
mechanism START forms they lead to, the team lead's D1):**

- **`Routine::atPercent(p)`**, **`afterDistance(d)`**, **`afterTurning(a)`** — a mark on the NEXT leg (one
  of six: `driveTo`, `reverseTo`, `moveTo`, `turnTo`, `face`, `faceAway`; `strafeTo` and
  `followTrajectory` take no trigger yet), followed by one of the four START forms (`startMove`, `startHome`, `startRun`, `startSpinUp`) and
  then the leg: `r.atPercent(60).startMove(lift["high"], 5_s).driveTo(40_in, 0_in)`. Up to four per leg,
  each on a different mechanism. "60 %" is the distance covered along the straight line from the leg's
  live start to its target (the angle turned, on a turn); `afterDistance` is a drive's mark,
  `afterTurning` a turn's — the wrong kind does not compile, nor does a blocking step after a mark. A
  trigger fires ONCE, the first tick its mark is reached — never before the leg is live, never on the
  tick that ends it (a fault abort's included), never once the run guard's time is up (an arrival on
  the deadline tick included). A leg that ARRIVES fires each trigger still
  waiting whose mark is ON the leg (no further than its length plus its arrival tolerance); a leg that
  ends any other way fires none. An unfired trigger's later `waitUntilDone` answers
  `MechanismNotStarted` ("the trigger never fired"), never a throw or an older verdict. A trigger is part
  of its drive step (it counts no step); a START refused when it fires (ABSENT, REFUSED, NOT_HOMED) never
  interrupts the drive — the drive step then fails, under its own `onFail` choice, naming the trigger
  refused FIRST (a START's `name`, if given, names the trigger in that line). A trigger written
  wrong throws on its own line, every run (the mechanism's own checks, below). Two handle types,
  `Routine::TriggerMark<Kinds>` and `Routine::ArmedLeg<Kinds>`, both `[[nodiscard]]`; a handle acts
  only for the line that made it (kept and used later, it is refused); a line that ends at its mark or
  without its drive is refused by the next call, and a Routine destroyed with one still pending logs
  one Warn.
- **`mechanism::PowerSlot::checkRun`**, **`PositionSlot::checkMoveTo`** / **`checkHome`**,
  **`SpeedSlot::checkSpinUp`** — each start command's precondition list, moved out of the command
  unchanged (same checks, same order, same messages) so a trigger runs the SAME list where it is
  written. The commands call them first; behaviour is unchanged.
- **`motion::LegProgress`**, **`motion::legProgressAlong`**, **`motion::turnProgress`**
  (`motion/leg_progress.hpp`, pure) — a leg's progress as a trigger reads it. `LegProgress` stays open
  to new fields.

**ADDED (Freeze Register row F6, the additive path; pinned the day they landed):**

- **`Chassis::moveTo(pose, options, LegObserver)`** and **`Chassis::turnTo(heading, options,
  LegObserver)`**, and **`chassis::LegObserver`** (`fn` and `context`) — the same leg as the plain
  verb, with an observer called after every tick that did not end it (nor one the scheduler's fault
  abort ends). The Tier-3 spelling of what a trigger does; it runs inside the tick (no blocking, no
  cancel, no throw).

**ADDED (outside F6 — the scheduler's own surface, reached through `scheduler()`):**
**`motion::MotionScheduler::pendingAbortFault()`** — the abort-mask fault raised since the active motion
started (the one its fault policy cancels it for at the end of the running tick), or `None`. Read-only;
the observer overloads ask it so a watcher is never called on a tick the fault abort ends.

**CHANGED (comment only):** `runner/mechanism_routes.hpp` says a step that starts mid-leg is now a
Routine's partway trigger; route markers stay blocking (F8 unchanged).

**What you must do:** nothing.

## API 2.7

### 2026-11-12 (built 2026-11-11) — curves: `curveTo` / `reverseCurveTo`, the robot flows through points instead of stopping at each

> Written when it was built (chunk CURVE1, part A). **API 2.7** — numbered by the coordinator at the merge
> (STEP1-B, built beside it, takes the next). Every change is
> ADDITIVE, on the additive paths of the frozen rows F6 and F10: **every 2.6 program compiles and runs
> exactly as before** — a six-leg route of today's verbs on a tank (TANK1 live) and on an X-drive hashes
> byte-identical (records, log lines, true pose) to the tree before this change. **Host test plant only —
> nothing here has run on a robot;** the curve's tuning (look-ahead, bend grip), its guards and its
> run-up rule are register entries HA-246..HA-250. [Chapter 5](guide/05-getting-there.md#flowing-through-points-curves)
> teaches it; [cookbook 1](cookbook/01-getting-there.md#flow-through-points-a-curve-and-its-fallback) has
> the recipe and its fallback.

**ADDED (Freeze Register row F6, the facade's additive path; pinned the day they land):**

- **`Chassis::curveTo(points, options)`** and **`Chassis::reverseCurveTo(points, options)`**, each with a
  `std::span<const CurvePoint>` and a brace-list overload: one smooth path from where the robot is,
  along its nose (its back, reversing), through 1 to 16 points, without stopping at them — **pure
  pursuit** (aim along the arc to the point 8 in ahead on the path) with TANK1's yaw observer, a speed
  plan that slows BEFORE each bend (`sqrt(bendAccel / curvature)`, capped by the turn-rate budget,
  braking back from every bend and from the end), and an END that is an ordinary `MoveToPose` (TANK1's
  steering, the same arrival check, STEP1-A's per-step tolerances, the same log). Refused before the
  robot moves (`TimedOut`, `MOTION_TIMEOUT`, one Warn naming why): a first point more than 90° off the
  nose ("turn first: face()"), and a path that doubles back (points within 1 in, a headingless point
  between two that coincide, any direction more than 90° from the chord into or out of it). Stopped
  when the robot loses the path: 60° off it for 0.25 s (checked until the end's hand-off, the last 12 in
  included), or under 1 in of progress in 0.75 s (not in the last 12 in). The timeout covers the WHOLE
  curve and always names `CurveTo` / `ReverseCurveTo`.
- **`chassis::CurvePoint`** (= `motion::CurvePoint`) — `{x, y}` or `{x, y, heading}`; **a heading is the
  robot's NOSE, also backwards.** Its two constructors and four members are pinned; there is
  deliberately no default constructor.

**ADDED (Freeze Register row F10, the Routine's additive path; pinned):** the four steps
**`Routine::curveTo`** / **`Routine::reverseCurveTo`** (span + brace list), in one block, each exactly
one Chassis call: a refused or lost curve is a `MotionFailed` stop — or carries on / skips under
`onFail`; the deadline always stops it.

**ADDED (not frozen):** `motion/curve_path.hpp` — the PURE path (`CurvePath`, `checkCurve`,
`appendCurveCheckWords`, `kMaxCurvePoints`, `kCurveMinSpacing`), callable from a host tool with no robot
(the route preview draws the planned curve with it); `motion/curve_follow.hpp` — the motion
(`CurveFollow`, `curveStatus()`, `curveYawObserverStep`); **`MotionConfig::curve`** (`CurveConfig`,
14 PROVISIONAL knobs, appended last — designated initializers keep compiling; `validate()` checks it).

**On the host test plant** (2026-11-12, measured on the built code after its review): the brief's 17
drivable routes × a side 0 / 9 / 15 % weaker × motor lag 0 / 0.12 / 0.18 s settle 16 16 12 / 16 16 16 /
13 17 16 per cell, every miss within 0.9 in, with **no guard trip** in 153 runs and no "turn first" Warn;
the speed plan holds the bend grip everywhere on the path, into every point included; a dead side stops
within 2.7 s (dying at 0.1–0.8 s, either side);
**a sideways miss up to 6 in is removed from 2½ ft of run-up, up to 1 ft from 3 ft** (nose within 20°,
a side up to 9 % weaker, lag to 0.18 s — every one of 180 runs within 0.5 in and 1.15° of the end pose;
closer than that, the curve runs out its timeout, at most 1.5 in off in this test: write `face` +
`driveTo` + `turnTo` there). The library never backs up and retries on its own.

**What you must do:** nothing. To use curves, write them behind a flag that stays `false` until the curve
has passed on your robot (`constexpr bool kUseCurves = false;` — the cookbook recipe), and give each a
`.timeout` near its real time. TANK1's off switch (`tankSteer.maxSteer = 0`) also switches off the curve's
yaw correction and its wait for the end's steering window. A robot file that lists `MotionConfig` members
positionally (none in this repository does) would see one more member at the end.

## API 2.6

### 2026-11-11 (built 2026-11-11) — "shulib Tune": live tuning of a robot's mechanisms on the brain screen

> Written when it was built (chunk TUNE1-B1). **No version bump — every change is ADDITIVE and nothing in
> it is a frozen surface:** three new headers and a new program. **No other program changes:** the match
> programs (Competition, Drive, the Auton Tester) and every other build link no live-tuning code — the src
> build gate now checks it from each package's symbols. **Host only — nothing here has run on a robot.**
> [Chapter 18](guide/18-mechanisms.md#the-tune-program--tuning-on-the-brain-screen) teaches it;
> [Chapter 11](guide/11-reading-the-diagnostics.md) reads its lines.

- **`mechanism/tune_program.hpp` (new)** — `mechanism::TuneProgram`, the PROS-free program: it makes ONLY
  the robot's mechanisms (never the drive), and only after READY; binds one service over a `LiveTuning` copy
  of the table; a TEST A / TEST B move per mechanism (presets, or a quarter and three quarters of the
  range; a flywheel's speeds; a power run), bounded by its plan's time ×2 + settle + 0.5 s (at most 10 s),
  and its report from the samples (`tuneReportFrom()`: overshoot, settle, peak, final error, the hold
  current over the second after it settled, the stall-above-hold and cap-above-peak warnings); HOME; the
  left stick as a jog (a fresh push only); UNDO, RESET, COPY (a block for every mechanism changed — and EXIT
  copies whatever no COPY carried yet); every mechanism released on ANY controller button, the controller
  disconnecting, a field or switch, a disable or STOP; READY refused without a controller, with a field or a
  button held; a mechanism's first motion behind a second confirm; every current cap it changed written back
  on RESET, on leaving that mechanism's page (its numbers COPIED first) and on EXIT (a RESET, copied first,
  when the cap cannot go back alone); an UNDO whose next undo would change ANOTHER mechanism's cap waits
  (nothing changes; UNDO it on that mechanism's page). `copyChanged()`, `uncopied()`, `copiedAtExit()`,
  `resetAtExit()`, `undoTrackedDepth()`.
  `tunePositionBound()`, `tuneTargets()`, `tuneTestLabel()`, `TuneIo`, `TuneDevices`, the `kTune*` constants.
- **`mechanism/tune_panel.hpp` (new)** — the screen as a pure model: `tuneView()` builds every page (CLEAR,
  TUNE, CONFIRM, REPORT, STRING CHECK — empty until the snapped-string watch —, EXITED, STOPPED) from
  `TuneFacts`; `tuneHit()`; `TuneView`, `TuneReport` (with why a hold is n/a: `holdEnded`, `holdStopped`),
  `TuneButton`, `TunePage`.
- **`hal/pros/tune.hpp` (new)** — the PROS shell: `ProsTuneShell` (the provider with its SD copy
  `/usd/tune_log.txt` APPENDED across boots, the screen, the master controller, the status),
  `tuneInitialize()`, `runTune()`.
- **The program (`src/programs/tune_program.{hpp,cpp}`)**: `make ROBOT=<a robot with mechanisms>
  PROGRAM=tune`, slot 7 — the first ROBOT-generic program row (the Makefile's `MECHANISM_ROBOTS`; a robot
  with none is refused by `make`). The src build gate gains the `tank-tune` build (eight in all), a `[tune-row]`
  check and step 8's live-tuning half, with self-test cases 42-49.

### 2026-11-11 (built 2026-11-10) — live tuning's library: a mechanism's numbers changed while the robot runs, inside the build's own rules, undone, and copied back out as the lines its robot file needs

> Written when it was built (chunk TUNE1-A — the library half of live tuning; "shulib Tune", the program
> that puts it on the brain screen, is the next part; the copied numbers are written into the robot file
> by hand). **No version bump — every change is ADDITIVE and nothing in it is a frozen surface:**
> a new header, new members on the mechanism service and its position slot, and one new member on
> `control::Pid` (C1's layer, not in the Freeze Register). **Nothing any program does changes:** no
> program includes the new header yet, and nothing else calls the new members. **Host only — nothing
> here has run on a robot.** [Chapter 18](guide/18-mechanisms.md#or-on-the-robot-live-tuning) teaches it;
> [Chapter 11](guide/11-reading-the-diagnostics.md) reads its lines.

**ADDED:**

- **`mechanism::LiveTuning`** (`mechanism/live_tuning.hpp`) — a COPY of a robot's mechanism table that the
  mechanism service runs (`table()`, then `attach(service)`, which checks the service runs this copy):
  `apply(i, Tunable, value)` and `step(i, Tunable, ±1, fine)` change one number inside the rules (the
  build's own check first, then the live bounds it lacks — a number the robot file states past one may
  step back toward it —, then the slot's state — between steps, between ticks, a travel limit never past
  a held target nor, holding nothing, past where it is) and have the service re-derive it; `undo()` (16 deep) and `reset(i)`
  (the robot file's numbers, bit for bit); `copyBlock(i, sink)` — the `TUNE` lines to write back into the
  declaration, the smallest set that builds to exactly what ran (a patch, member by member; a number the
  file states at exactly its worked-out value is left out). A current cap changed live stays in the
  motors after the program unless it is reset or undone first (HA-220). With it: `Tunable`, `StepRule`,
  `Parameter` / `parametersOf()` / `parameterOf()` / `offered()`, `tunableValue()` / `setTunable()` /
  `tunableUnit()`, `steppedValue()`, `tuningRefusal()` / `tuningProblem()`, `LiveRefusal` /
  `liveRefusalText()`, `Applied`, and the constants `kLiveUndoDepth` (16), `kLiveKvMostTimesStarting` (2 —
  invented), `kLiveFirstKaSeconds` (0.01 s) and `kCopyNumberChars` (24).
- **`MechanismService::rederive(i, what)`** and **`canRederive(i)`** — the slot re-reads what it copied from
  its spec at bind (the PIDs' gains, BUMPLESS; its stall and homing detectors; its limits in degrees and
  the soft stop's acceleration; its current cap, re-written — or written back to the motor's 2.5 A default
  when a live cap is removed), with one `MECH` Info line; refused (false, nothing said) inside a tick,
  while that mechanism's step runs, or — a position mechanism — when its spec's travel limits fail
  `takesRange` (read once, before anything is re-derived; a caller that wrote the spec puts the old one
  back).
- **`PositionSlot::holding()`, `heldButtonMoving()`, `takesRange(candidate)`** — whether it holds a
  target, whether a held button moves it, and whether new travel limits may be given to it now (never
  past its held target, never under a held button, and, holding nothing, never past where it is).
- **`control::Pid::setGains(kP, kI, kD)`** and **`Pid::config()`** — new gains with no jump of their own:
  the integral TERM is kept (the accumulator rescaled), the derivative's baseline kept.

**What you must do:** nothing.

### 2026-11-11 (built 2026-11-05) — care on every auton step: an arrival tolerance, and a per-step choice to stop, carry on, or skip ahead to a named step

> Written when it was built (chunk STEP1-A, the route maker's library half, part A — partway triggers
> are part B). **API 2.6** — every change is ADDITIVE, on the additive paths of the frozen rows F6 and F10, so the
> minor goes up (numbered by the coordinator at the merge). **Every 2.5 program compiles and runs exactly as before**, with ONE thing to do
> (below): a `Routine` with no `onFail` follows the default rule bit for bit — seven golden chains (every
> chassis step form, five failure kinds, and robot x's mechanism steps) captured from the base tree read byte-identical,
> and every existing test passes unchanged. **Host only — nothing here has run on a robot;** how far
> robot two coasts after a leg that ends at a loosened speed bound is register entry HA-216, unmeasured.
> [Chapter 9](guide/09-the-recipe-api.md#choosing-what-a-failure-does-onfail-and-label) teaches it;
> [cookbook 2](cookbook/02-when-a-step-fails.md#carry-on-past-a-missed-piece) has two recipes.

**ADDED (pinned the day it landed, Freeze Register row F10 — the team lead's D1):**

- **`Routine::onFail(OnFail)`** and **`Routine::label(const char*)`** — modifiers written in front
  of ONE step: `r.onFail(carryOn).driveTo(...)` counts a failure, safes the drive and runs the next
  step; `r.onFail(skipTo("third piece")).run(...)` does the same, then passes over every step until
  `r.label("third piece")`'s, and goes on there (forward only: a target already passed, or the step's
  own label, stops the chain, said "behind it" — and Warned on the `onFail` line itself, every run; a
  target that never comes jumps the rest). `onFail(stopChain)` is the default, spelled out. A label is
  not a step: the step numbers do not move. Two in a row, or an empty name, throw.
- **`chassis::OnFail`** and its three spellings **`stopChain`**, **`carryOn`**, **`skipTo("x")`** —
  also in `mechanism::words`, so a robot's auton file needs no namespace.
- **`RoutineStopCause::MechanismNotStarted`** (`== 6`) — a `waitUntilDone()` whose start a skip
  jumped: a COUNTED failure ("its start was skipped") under the wait's own choice. Before this, that
  wait threw out of the auton, or answered an older step's verdict at once. The same answer ("its
  start did nothing: another robot's declaration") for a wait whose start named another robot's
  mechanism (a different kind or units, or a preset or position this robot's same-named mechanism
  lacks) and was carried on past.
- **Six `RoutineResult` fields:** `failedOn`, `firstFailedAt`, `firstFailedName`, `firstFailedCause`,
  `jumped`, `skipPendingTo`. The nine existing fields keep their meaning ("where the chain STOPPED"):
  under `carryOn` / `skipTo`, `stoppedAt` stays 0 unless a stop happens. `ok()` is false once a step
  has failed, carried on or not.
- **Two rules above every choice.** (1) **The deadline always stops:** a CANCELLED verdict (a
  motion's, a mechanism step's, or a `then()` action's `ExitReason` / `MechanismOutcome`), any failure
  once the run guard is running and expired, and a skip still pending then, all STOP the chain
  whatever the step chose. A `Routine r{ctx, ...}` built from an `AutonContext` now borrows its run
  guard for this (any context with a `guard` member); a Routine built from a bare chassis has only
  the CANCELLED half and Warns once at its first `carryOn` / `skipTo`. (2) **Nothing new throws** on a
  run-dependent path.

**ADDED (Freeze Register row F6, the field set's additive path; both types pinned):**

- **`MotionOptions::positionTolerance`** (a `units::Length`) and **`MotionOptions::headingTolerance`**
  (a `math::Angle`, so `5_deg`) — an ARRIVAL TOLERANCE per call. 0 = the robot's own settle numbers,
  bit for bit. Looser than the robot's own loosens the distance AND the speed bound in proportion
  (2 in → within 2 in and slower than 4 in/s), because on a drive leg the speed bound is what ends it:
  on the host test plant the 40 in leg takes 1.80 s at the robot's own 0.5 in — and 1.80 s at 1, 2 or
  4 in with the distance alone — but 1.35 s at 2 in with both; tighter tightens the distance only.
  `brake()` ignores both. Refused at the call: non-finite, negative, or an angle that wraps negative
  (`359_deg`).
- **`motion::loosenedSettle(robot, tolerance)`** — that formula, pure and total (a robot whose own
  settle distance is 0 keeps its speed bound; never a division by 0).

**CHANGED — the brain's result word (the Auton Tester and the competition program):** a routine that
carried on past failed steps to its end, with its end action succeeded, reads **`DONE, n FAILED`**
(`DONE, 9+ FAILED` for ten or more) in AMBER — no longer `ROUTINE STOPPED`. Every DONE word fits the
driver page's unchanged 15-character slot. The serial and SD log gain one line in that case (and when a
skip never landed), naming the first failed step; under the default rule every line is unchanged.

**What you must do:** one thing, as at 2.1 and 2.5's first entry: a `switch` over `RoutineStopCause`
that lists every value needs a `MechanismNotStarted` case (under `-Wswitch -Werror` it stops compiling
without one — the Auton Tester's and the competition program's each gained one). Nothing else.

## API 2.5

### 2026-11-11 (built 2026-11-05) — the WHICH END question removed: the end that leads the push is the front; robot two runs the standard Calibrate

> Written when it was built (chunk CAL1, Part B — the last of its four parts). **BEHAVIOUR CHANGE in an
> unfrozen layer — no version bump:** `calibration/` is in no row of the
> [Freeze Register](roadmap.md#freeze-register), so removing and renaming its members is not a breaking change
> to a frozen surface (the policy in `include/shulib/version.hpp`). **Host only — robot two's Calibrate has not
> run on a robot in this form** (the calibrator inside it ran on robot two on 2026-09-24).

**Why:** the team lead's ruling of 2026-10-06 — the robot's FRONT is never asked or typed: the end that LEADS
Calibrate's push is the front, with a flip warning. The standard Calibrate (2.5's "the standard Calibrate"
entry below) already never asked; robot two's own program still did. This part moves robot two onto the
standard Calibrate and takes the question out of the library.

**BEHAVIOUR CHANGE (removed and renamed — `calibration/`):**

- `calibration::Answer::FrontFirst` and `Answer::BackFirst` are **removed** (`Answer` is `Done, Abort`; it is
  never stored, so nothing on a card changes). `DrivetrainCalibrator`'s DONE goes straight on to the pulses:
  the end that led the push is the front, and the captured signs stand as captured. A push led with the
  other end gives every port the other sign AND the other side — the robot facing the other way — which is
  why a program checks the result against the card's record (`calibration/front_check.hpp`).
- `calibration::CalibrationReport::backFirst` is **removed**.
- `calibration::PanelScreen::WhichEnd` and `calibration::FrontRule` (with `viewFor()`'s and
  `TouchGate::view()`'s `rule` parameter) are **removed**: every screen is the standard Calibrate's — four
  steps (`1/4 IMU` .. `4/4 SPIN`), `PUSH FRONT FIRST`, the front rule while the IMU warms up.
- `calibration::PanelButtonId::Front` is **renamed `SaveHeld`** (a held screen's USE NEW / SAVE) and
  `PanelButtonId::Back` is **removed**; `kFrontBox` / `kBackBox` are **renamed `kChoiceLeftBox` /
  `kChoiceRightBox`** (the same rectangles, in the middle of the screen).
- In `calibration::panel_text`: `kHeaderImu`, `kHeaderPush`, `kHeaderPulses`, `kHeaderSpin` now hold the
  four-step words, and `kHeaderImu4` .. `kHeaderSpin4` are removed; `kNextTouchDone` is `"touch DONE -- then
  stand clear"` and `kNextTouchDoneClear` is removed; `kPushForward`, `kWhichEnd`, `kHeaderWhichEnd`,
  `kLabelFront` and `kLabelBack` are removed (`kPushFrontFirst` stays).
- `DrivetrainCalibrator` reads the IMU's yaw rate on every tick of the push (nothing of the push uses it):
  the PROS IMU adapter's default rate is the derivative of the rotation between fresh reads, so without it
  the first rest tick after DONE read a rate averaged over the whole push, and a robot turned by hand read
  as still for that one tick. The WHICH END step used to take that stale read. On the host test plant the
  one visible change is a curved push reaching rest one tick sooner.

**What you must do:** a program that answered `Answer::FrontFirst` / `BackFirst` answers nothing now —
delete the call (DONE is the last answer); a program that names `PanelButtonId::Front` uses `SaveHeld`
(`Back` is gone), `kFrontBox` / `kBackBox` use `kChoiceLeftBox` / `kChoiceRightBox`, and a `viewFor()` /
`TouchGate::view()` call drops its `FrontRule` argument. No program outside this repository is known to
use them. A team on the standard Calibrate (chapter 19's 21-line file) changes nothing.

**Robot two (this repository's programs, not library API):** `make ROBOT=tank PROGRAM=calibrate` — slot 4,
the `tank-calibrate` beacon — is now the standard Calibrate: `src/programs/calibrate_program.cpp` is a 27-line
glue (it was the 1,397-line program), and robot two's facts are DERIVED from its two tables by
`src/robots/calibrate_facts.hpp` (Tank, its cartridge, one declared port per mechanism motor, named by its
mechanism; no typed drive ports, because its typed table is not one the drive program could drive from —
its track width and gear ratio are unset). Every one of the 34 recorded runs of 2.5's standard Calibrate
was unchanged by deriving the facts and removing the question. Against robot two's OLD program, the same
seven comparable runs on one host test plant differ only in lines that each have a named reason — G
(robot two's words become the standard program's), F (the front is the end that leads the push: no WHICH
END, steps 1/4..4/4, the front check), S (the safe save and its words), W (the declared-port watch, and a
declared port named by its mechanism), M (the words that say an undeclared motor may be a mechanism), T
(timing on the 10 ms plant grid) — with none left over; the saved records are byte-identical (`one`,
`slip18`, `curved`) or differ only in a scale's last bits (`clean`), and the new program also writes the
backup. One more (the standard Calibrate's screen, every team's): after DONE the PULSES screen's ABORT
stays in its usual place at the bottom right until the first pulse powers a motor, and only then covers
the whole button row — so a reflexive second tap where DONE was lands on nothing, as the screen's own
rule says no button may (since 2.5 the PULSES' ABORT had covered the row from the first frame, and the
second tap aborted the run). The PULSES screen is drawn once more when it widens: one more `screen:`
line per run, and the loop line's screen-print count one higher.

### 2026-11-10 (built 2026-11-05) — a mechanism's current cap: the most each motor may draw, written into the motors at boot and enforced by the motors themselves — how hard a string lift's string is pulled

> Written when it was built (chunk MECH-S-A, the first part of a string lift's safety — the slack watch
> that sees a snapped string, and live tuning on the brain, are later parts). **No version bump — every
> change is ADDITIVE and nothing in it is a frozen surface:** a new HAL seam BESIDE the locked `IMotor`
> (F4 is untouched, byte for byte — the `IDigitalOut` / `IReadFaults` precedent), a new optional member on
> the four motor kinds, three refusals APPENDED to the build check's list, a new member at the end of
> `MechanismSpec` and of `mechanism::Devices`. **One thing a team's own code may meet:** `IDeviceProvider`
> gains one more accessor every implementer must answer (`currentLimit(port)`) — a provider written
> outside the library must add it (VIS1-A1's `aiVision` is the precedent). **Every 2.5 program compiles
> and runs exactly as before while every mechanism motor reads the 2.5 A default:** a mechanism with no
> cap is only READ at boot (nothing written), and robot two states no cap (its boot is byte for byte a
> provider's that answers no limit at all — `mechanism_current_cap_test.cpp`); the host test plant's
> limiter is bypassed at the default (a reversal's samples are 2.5's, byte for byte). **Host only —
> nothing here has run on a robot;** whether a real motor holds its limit this way is HA-219, never
> measured. [Chapter 18](guide/18-mechanisms.md#current-caps--how-hard-a-string-is-pulled) teaches it;
> [Chapter 11](guide/11-reading-the-diagnostics.md) reads its lines.

**ADDED:**

- **`hal::IMotorCurrentLimit`** (`hal/motor_current_limit.hpp`) — a motor's own current limit:
  `setCurrentLimit(units::Current)` (true when the device accepted the write) and `currentLimit()` (READ
  BACK from the device, never the request's echo), and `hal::kDefaultMotorCurrentLimit` (2.5 A, the V5's
  documented default). Implemented by `hal::pros::ProsMotor` (mA on the wire through the new
  `hal::motorCurrentLimitToMilliamps`; a refused read holds the last good one and counts in
  `faultedReads()`) and `hal::fake::FakeMotor` (remembers the limit, enforces nothing; knobs to refuse,
  to keep another number, to lose power; counts writes and reads). A V5 motor holds its limit in WHOLE
  milliamps: a cap is sent rounded to the nearest mA, and the service compares every read-back in mA.
- **`IDeviceProvider::currentLimit(port)`** — the limit of the motor the provider made on `port`;
  nullptr when none was made there. Looks up, never makes. Every implementer answers it:
  `ProsDeviceProvider`, `FakeDeviceProvider` (and its new `bindMotor(port, FakeMotor&)`, which binds a
  bound fake's limit too).
- **`.currentCap = {.amps = …}`** on `Power`, `Arm`, `Lift` and `Flywheel` (`mechanism::CurrentCap`,
  after `.stall` in each kind's member order) → `MechanismSpec::currentCapAmps`. Per motor; optional;
  never a default; a cap written as 0 is no cap, the same as leaving it out.
- **Three refusals, appended (71, 72, 73):** `ACurrentCapMustBeAbove0AndAtMost25A` (a written cap below
  0 or above 2.5 A, a NaN included), `AStallSignatureMustBeBelowTheCurrentCap` (any written stall or
  homing signature at or above the cap — it could never trip) and `ACurrentCapMustBeAtLeastOneMilliamp`
  (a cap above 0 but under 0.001 A — it would be sent as 0 mA), named at build as
  `a_current_cap_must_be_above_0_and_at_most_2_5_A`,
  `a_stall_signatures_amps_must_be_below_the_current_cap_it_could_never_trip` and
  `a_current_cap_must_be_at_least_0_001_A_a_motor_holds_whole_milliamps`.
- **The mechanism service** writes a stated cap into every motor of the mechanism at bind and reads it
  back (one Info line, per motor and in all; a Warn per motor that would not take it, naming the port
  and both numbers; a Warn for a signature not MORE than 0.2 A below the cap, judged in whole milliamps as the motor reads — HA-223), reads every uncapped
  mechanism motor's limit and writes NOTHING (an Info line when one reads other than 2.5 A — HA-220,
  HA-221), and re-checks each capped motor once a second inside its tick (one back at the default is
  written again with one Warn; one at any other number is warned about once and never written). Two
  library constants: `kCapSignatureMarginAmps` (0.2 A) and `kCapRecheckSeconds` (1 s). No verdict
  changes; nothing is refused at boot.
- **`mechanism::Devices::currentLimits`** (each motor's limit, filled by `robot::MechanismParts`) and
  `mechanism::kDevicesMaxMotors`.
- **The host test plant** (`sim::ArmPlant`) honours motor 0's limit when it is below 2.5 A: the
  limit's TORQUE, re-solved at the end speed of each step, the Hold brake at 0 V included.

**Hardware Assumptions:** HA-219..HA-223 registered (HA-224..HA-227 stay held for the slack watch).

### 2026-11-10 (built 2026-11-03–05) — the wall reset from an auton: the robot file's side distance sensors, built by the Robot, and the everyday call over the auton's context

> Written when it was built (chunk L6-A2, the wall reset's second part — robot two's sensors are Part
> B). **No version bump — every change is ADDITIVE and nothing in it is a frozen surface:** a new
> header, new members APPENDED (with defaults) to `config::SensorsConfig`, `config::PortUse`,
> `chassis::RobotContextConfig` and `robot::Robot`, none of which is in the
> [Freeze Register](roadmap.md#freeze-register) (`RobotContext` is "NOT part of the F4 freeze" by its own
> banner). **Every 2.5 program compiles and runs as before, with one visible change: the boot line**
> (below). **Host only — nothing here has run on a robot;** robot two's robot file does not declare its
> side sensors yet (Part B), so on robot two a FIELD-start auton's wall reset refuses `NO SENSOR` until
> it does (a RELATIVE-start auton's refuses `not a field start` first, on every robot).
> [Chapter 3](guide/03-knowing-where-you-are.md#re-seating-one-axis-against-a-wall) teaches it,
> [cookbook 5](cookbook/05-mixing-tiers.md#re-seat-x-against-a-wall) is the recipe.

**ADDED:**

- **`competition::resetFromWall(ctx, request)`** (`competition/auton_wall_reset.hpp`) — THE EVERYDAY CALL an
  auton writes, through `then()`:
  `r.then([&] { return competition::resetFromWall(ctx, {.wall = localization::override2026::kLeftWall,
  .sensor = localization::WallSide::Left, .maxCorrection = 3_in}).continues(); }, "left wall x");`. It
  refuses — reading nothing, taking no time — when the auton's start is not a FIELD start
  (`not a field start`, the line naming the mode: RELATIVE, or UNSET for a route whose Start states no
  heading; the season's walls are field coordinates, so **a wall reset needs a FIELD start**) or when the
  robot declares no wall sensors (`no sensor`); otherwise it runs every check of the explicit call over
  the robot's own side sensor, mount and limits, and samples through the RUN GUARD's deadline-aware wait —
  so a deadline or a disable that lands mid-window cuts it at once (`cut`, nothing seated), where the
  explicit call's frozen wait would run on up to `windowMax` late. Every outcome is filed in the robot's
  wall-reset log (seated and refused counts, the last report — what the Auton Tester will show, Part B);
  ONE `WALL` line goes to the auton's own telemetry. Calling it outside a live run is a precondition (an
  `AutonContext` lives for one guarded run).
- **The robot file's side sensors — `config::WallSensorsConfig`** (`config/wall_sensor_config.hpp`), as
  **`RobotConfig::sensors.walls`**: per side (`left`, `right`) the sensor's `port` (0 = none, the
  default) and its MEASURED `mount` (`localization::SideMount` — UNSET until measured, which is NOT a
  config error: the robot boots and every reset with that sensor refuses `MOUNT UNSET`), the robot's
  `limits` (`localization::WallResetLimits`, the PROVISIONAL defaults unless typed) and a `note`.
  `config::PortUse::WallSensor` (APPENDED; "a wall sensor"), and `RobotConfig::valid()` refuses, naming
  the side and the port: a port past 21; both sides on one port; a port claimed by anything else, naming
  both ("port 4 is claimed by the LEFT drive and by a wall sensor"); a MEASURED mount with a problem
  (`config::wallMountProblem` — a number that is not finite, a `facing` outside (−180°, 180°] (`90.0`
  typed alone is 90 radians), the lens more than 24 in from the centre, a height outside (0, 11.5] in —
  the wall's top —, a measured cone outside 3–30°, a tolerance of 0 or past 2 in / 10°, a bias past 2
  in, a measured accuracy outside 0.05–2 in, no provenance; then `config::wallSideProblem` — a sensor
  that does not face out of, or sit on, its own side: the usual sign of a block copied from the other
  side); and limits
  with a problem (`config::wallLimitsProblem` — everything the reset itself would refuse as a bad
  request, so a valid robot file never meets one, plus tighter spans: `rangeMin` at least 0.79 in,
  3–64 samples, `headingFloor` 0.5–10°, `headingPerTurn` 0–0.1, `alongDriftRate` 0–0.5).
- **`robot::Robot` builds the side sensors** — LAST in the constructor's body, after the camera, through
  the provider's `distance(port)` and `readFaults(port)` (no provider change): **`wallSensor(side)`**
  (nullptr when absent), **`wallSensorAbsence(side)`** ("" exactly when present; "not declared" for port
  0; the provider's own sentence, COPIED, for a refused one), **`wallSensors()`** (both sides, the limits
  and the reset log). **A refused side sensor never refuses the robot** — a cable out must not cost a
  match — and neither does a provider that throws making one. `validate()` does not ask for them.
- **`chassis::RobotContextConfig::walls`** — the context's first OPTIONAL slot (`localization::WallSensors*`,
  nullptr by default — then the wall reset refuses `NO SENSOR`), with **`RobotContext::wallSensors()`**.
  The constructor still requires every other slot. The Robot hands it in when its config declares a
  side; the motion scheduler's shadow context (what `chassis.deps().ctx` is) carries it.

**CHANGED (visible):**

- **The boot line** (`Robot::describe()`, printed by the drive and X-drive programs as `robot::Robot:
  ...`) gains the side sensors' words between `tags ABSENT` and the camera's: `walls ABSENT` (no side
  declared — every program's line today), `walls L4 R5`, `walls L4 R5(ABSENT)` for a refused side,
  `walls L4 R5 MOUNT UNSET` when a declared side's mount is unmeasured. Robot two's line grows from 179
  to 192 bytes; the drive program already splits a long line at a space, so nothing is cut.
- **The `not a field start` WALL line** now names the start mode: `start RELATIVE -- the season's walls
  are FIELD coordinates: declare a FIELD start` (`start UNSET` for an unset one).
- **The `cut` WALL line** has a second form for a call made after the run had already ended (no sample
  taken): `the run's deadline or a disable came before the window began; nothing changed` — the call came
  too late, not the window.

**What you must do:** a test that pins the boot line's exact text adds ` walls ABSENT` after
`tags ABSENT` (the library's own pins were updated). To use the wall reset from an auton: declare the
side sensors in the robot file (ports, and each mount measured with a tape — Chapter 10), give the auton a
FIELD start, park nose-along the wall and square (Chapter 3's recipe), include
`shulib/competition/auton_wall_reset.hpp` and `shulib/localization/override_walls.hpp` in the auton
file (the shipped `src/team/autons.cpp` includes neither), and call `competition::resetFromWall(ctx, …)`
through `then()`.

### 2026-11-06 (built 2026-10-30) — the mechanism core, part 6 (its last): Bench Tests' station 13 shows every mechanism in its own units, powers one at a time behind six gates, and runs the direction check on the robot — no API change

> Written when it was built (chunk MECH-A6, the last of the mechanism core's six parts) and corrected at
> its review round (2026-11-03: the 31 findings of three reviewers — what follows is the page after it; its
> verification round follows). **No version bump and no library change:** nothing under `include/shulib/`
> moved. It is the bench program's (`src/programs/bench_r3a.{hpp,cpp}`) — and the first program to run the
> core's direction check. **Nothing here has run on a robot.**

- **Station 13 MECHANISMS** in Bench Tests (slot 3): every mechanism the robot's file declares, built
  through `robot::MechanismParts` from the same projection every program boots from — no drivetrain, no
  mechanism service, nothing homed — shown one at a time:
  - **its readings, live, in its declared units**: each motor's angle since power-up, speed, current and
    temperature (`unknown` when the read failed); an arm's or lift's position through its gearing and
    shape (a gearing typed wrong is a wrong angle on the screen); a flywheel's output rpm; the angle
    sensor raw and as mounted, and the number `absoluteSensor(at, reads)` takes (the RAW reading in
    [0, 360)); a limit switch's level — a home switch or a switch piece sensor — asked for pressed,
    released and unplugged (HA-184's settle); the piece sensor's reading and whether it sees a piece; a
    pneumatic's position read back from its lines at every visit; more lines than the panel holds on a
    second page (tap the readings);
  - **power behind six gates**: only a built mechanism; a CLEAR question before anything is made (its YES
    says whether making a line moves a cylinder), a confirm before a motor mechanism's first volt and
    before EVERY pneumatic fire; L1 as a dead-man (let go, and what the page powered is safe that tick — a
    confirm on the screen closes at once — and what it never powered is never touched); a jog only while
    its arrow is held; a ceiling from 3 V for each mechanism that R1 raises to 12, never past a power
    mechanism's declared `maxVolts`; a cut on a fight between a mechanism's motors, on 2.4 A for 250 ms, on
    its own stall signature and on a 3 s jog; everything safe on the way out. Every number is invented for
    a first run (HA-182's page entries);
  - **the direction check**, from rest: "ONLY '<name>' should move" said with nothing driven for 1 s (a
    mechanism still moving in its last 0.4 s abandons it), one + jog for 0.4 s, `mechanism::judgeDirection`
    over what the encoders saw — a fight, a dead motor, a jam, a back-drive, a reversed sensor reported at
    once, otherwise the three answers; a pneumatic fired and "which position did it go to?"
    (`judgePneumatic`) — the verdict naming the robot-file line, nothing saved. RECORD writes `[R3A] mech`
    lines to serial and the SD log — each mechanism's own latest verdict, and every line one serial write
    (a long one in tagged `(cont.)` pieces).
- **The menu is 3 × 5** (it was 3 × 4): thirteen buttons, room for fifteen; stations 1–12 keep their
  numbers, their logged names and their places; the buttons are 36 px tall again, their second line in the
  small font. Buttons 10 and 13 carry the red stripe: the two stations that power motors.
- The station's logic is PROS-free and host-tested over the library's own parts on a fake provider (the
  gates one by one, the check's flow, the readings, the RECORD lines, the line lengths on robot two's real
  table), with every gate's mutation seen red; the steps only the bench program runs (the CLEAR before
  anything is made, the dead-man watched inside each confirm) are pinned by reading its source.

### Added and fixed since 2.5 (2026-11-03; built 2026-10-29–30) — the wall reset's library core: one axis of the estimate re-seated against a field wall from a side distance sensor; cookbook 05's re-seed recipe keeps y

> Written when it was built (chunk L6-A1, the first of the wall reset's three parts — the robot file's
> side sensors and the everyday call an auton writes are Part A2, robot two's sensors Part B). **No
> version bump — every change is ADDITIVE and nothing in it is a frozen surface:** new headers, and a
> new member and two new read-only accessors on `localization::Localizer`, which is outside every
> freeze (`Localizer::reseatHeading`, 2.4's COMP-a, is the precedent). **Every 2.5 program compiles and
> runs exactly as before:** the Localizer's estimate, quality and drift term are bit-for-bit
> unchanged by the two new counters (the whole suite is the evidence, and a 1500-tick trace of pose,
> twist, quality, drift term and heading bias over one drive — through the boot fold, a `setPose` and
> a `reseatHeading` — is identical to 2.5's under both fusion tiers). **Host only — nothing here has
> run on a robot,** and no distance-sensor reading of the team's robot exists yet;
> [Chapter 10](guide/10-the-api.md#the-wall-reset--one-axis-against-a-field-wall) teaches it and
> [Chapter 14](guide/14-what-it-cannot-do-yet.md#the-wall-reset-what-it-cannot-catch) says what it
> cannot catch.

**ADDED:**

- **`chassis::resetFromWall(chassis, sensor, request, limits)`** (`chassis/wall_reset_call.hpp`) — the
  EXPLICIT call over sensor handles a hand-wired robot (or a host test) builds: with the robot parked
  beside a field wall, nose along it, it samples that side's distance sensor for a short window (≥ 5
  samples over ≥ 0.15 s, at most 0.5 s), and either SEATS that one coordinate of the estimate (x against
  the left/right walls, y against the red/blue walls) or REFUSES, saying which check failed — the setup
  (sensor, measured mount, a live estimate, **no motion armed**, a usable request), the beam square to the
  wall, the beam off the season's keep-outs, the beam-height ceiling, stillness (the estimate's
  displacement over the last `window`, ≤ 1 in/s, and its turn rate, ≤ 3°/s), every sample (no read
  fault between its two device reads, a real return, enough confidence, inside 2–16 in), the window's
  count, time, spread and median, the answer's worst-case error, and its agreement with the estimate
  within the author's `maxCorrection` (which has NO default: 0 is refused by name). ONE log line either
  way (tag `WALL`, [Chapter 11](guide/11-reading-the-diagnostics.md#the-wall-reset-wall) reads it); commands nothing to the
  drive; a refusal leaves the estimate exactly as it was. Returns a `WallResetReport` — `continues()` is
  what a `then()` returns (a refusal fails the step only when the request says `required`).
- **`localization/wall_reset.hpp`** — the pure core both forms run: `WallSide`, `WallFrame`,
  `KeepOutKind`, `WallKeepOut`, `FieldWall` (and `FieldWall::line(...)` for an author's own line),
  `SideMount` (the MEASURED mount: `measured = false` refuses every reset; optional measured
  `rangeBias`, `rangeAccuracy`, `coneHalfAngle`, `toggleChecked`), `WallResetLimits` (the PROVISIONAL gate
  defaults — HA-211), `WallResetRequest`, `WallSample`, the APPEND-ONLY `WallResetVerdict` and
  `wallResetVerdictWord`, `WallResetReport`, `WallSensor`, `WallSensors`, `WallResetLog`, the geometry
  (`axisFromReading`, `axisErrorBound` — the worst case over the tolerance box's corners, accuracy max(15
  mm, 5 %) or a measured one, never a step at 200 mm — `headingTolerance`, `beamCeiling`), the gates
  (`preCheck`, `sampleVerdict`, `evaluateWallReset` — which refuses an unusable request itself, naming
  the field), `WallSampler` and the line's text (`appendWallResetLine`). A `window` over 0.6 s
  (`kMaxWallWindowSeconds` — more than the 64-sample buffer spans) is refused by name; a non-finite
  tolerance makes the error bound infinite (refused), never 0; and a non-finite pose or turn rate ends
  the sampler's window at that tick as MOVING.
- **`localization/override_walls.hpp`** — the 2026-27 season's four walls in
  `localization::override2026` (`kLeftWall`, `kRightWall`, `kRedWall`, `kBlueWall`, inner faces at ±70.205
  in, FIELD coordinates), each with the stretches a beam must not land on: the Toggle body (lifted only by a
  measured `toggleChecked`), its brackets, the Loaders (red and blue walls), the corners, the VEX U Load
  Zones (always), and the start pieces (until the beam's spread is measured). Manual v2.0's numbers (v2.1,
  2026-10-08, exists and could not be read: whether it moved any is unverified — HA-207).
- **`localization/axis.hpp`** — `localization::Axis` (X, Y), `axisLetter`, `alongAxis`: a leaf both the
  Localizer and the wall reset include.
- **`Localizer::seatAxis(axis, value)`** — moves ONE axis of the estimate and nothing else: not the other
  axis, the heading, the heading bias, the drift term, the quality, nor the odometry (unlike `setPose`,
  which would let one measured axis pass as a fix of both). The next tick re-bases (dt 0), which the EKF
  tier needs. Preconditions: a live estimate, a finite value, and — by contract — no motion active. And
  two read-only counters the wall reset's gates use: **`Localizer::travelSinceSeat(axis)`** (path length
  since that axis was last set) and **`Localizer::turnedSinceHeadingSeed()`** (rotation since the heading
  was last seeded) — both counted in STEPS (`kTravelCounterStepInches`, 0.5 in of the odometry's path;
  `kTurnCounterStepRadians`, 1° of the IMU's heading), so a robot standing still reads ~0 however long
  it stands (summed tick by tick, a still IMU's heading jitter read hundreds of degrees of "turning" a
  minute and refused a skills run's late resets) — at the IMU jitter the register believes (HA-21): on
  the host test plant that holds up to three times the jitter and falls off steeply past it (HA-211). **Locked decision #4** (never snap) carries the exception the team lead accepted on
  2026-10-22: a caller-invoked ONE-axis wall reset, with no motion active, bounded by the author's
  `maxCorrection`; no sensor ever seats the heading.
- **The host test plant's field and side sensors** (`sim/field_plant.hpp`, for test authors — see
  `test/README.md`): `sim::FieldModel` (boxes with heights, opaque or clear, the floor; the season's field
  from its own copy of the drawing numbers) and `sim::SideSensorPlant` (a cone of rays from a TRUE mount,
  nearest hit or a blend, the code strip, noise, update hold, latency, drop-outs, a frozen mode, read
  faults).

**FIXED:**

- **Cookbook 05's re-seed recipe overwrote the OTHER axis** ("Re-seed the estimate mid-routine"): it wrote
  `0_in` for y after a wall at x = 36 in, which measures x only — right only for a robot that happened to
  stand on y = 0, which its compiled test's robot did. It now keeps y (`c.chassis.pose().y()`), says that
  `startAt` clears the drift term for both axes, and points at the wall reset for a sensor-measured axis;
  its test drives along y = 12 in.

**What you must do:** nothing. To use the reset on a hand-wired robot, measure each side sensor's mount
and pass it with the sensor; on the team's robots, wait for the robot file's side sensors (Part A2).

### Added since 2.5 (2026-10-30; built 2026-10-18–22, reviewed and fixed 2026-10-22–30) — the AI Vision camera as a device: a frame at a time, its colours and AI model written and checked, and pixels turned into bearings and ranges that carry their error

> Written when it was built (chunk VIS1, part A1 of three: the camera as a device; its Toggle colour
> tracker, piece finder and camera feed are part A2, robot two's camera and the bench page part B). **No
> version bump — every change is ADDITIVE:** seven new headers; an appended enumerator
> (`hal::DeviceKind::AiVision`), an appended field (`config::SensorsConfig::camera`), an appended `config::PortUse::Camera`
> and new members on surfaces the [Freeze Register](roadmap.md#freeze-register) does not freeze
> (`hal::IDeviceProvider`, `config::RobotConfig` and `robot::Robot` are in no row of it); the F4 seams
> untouched — `hal::IVision` stays byte for byte, and the camera's frames are a SIBLING seam, row F15, NOT
> FROZEN. Every config that was valid stays valid and builds the same robot: the camera's port defaults to
> 0, no camera. **No program declares a camera yet** (robot two's table gains it in part B), so every
> program builds and drives exactly as before. **Host only — no AI Vision sensor has been read by this
> code.**

**Why:** the team lead's rulings of 2026-10-08 and 2026-10-14 put the camera before the robot panel —
its colours for the Toggle auto-flipper (AF1), its OBJECTS (Pins, Cups: VEX's Override AI model) for
driving to a named piece (L4). Both rest on one adapter, one frame and one geometry, built here.

**ADDED:**

- **`hal::IVisionFrames`** (`hal/vision_frames.hpp`) — the camera read a FRAME at a time: `read()` (hits
  the device: the count, then each detection) and `status()` (touches nothing). A `hal::VisionFrame` holds
  up to 24 `hal::Detection`s (a colour blob, a colour code or an AI-model object: its kind, id and box in
  image pixels, +y down), the time the read COMPLETED (`readAt` — PROS gives no frame counter and no
  timestamp, and the IMAGE is older than the read by an unmeasured age), the adapter's poll count
  (`readSeq`, never a frame number), and how the read went: **only `FrameStatus::Ok` is evidence.** A
  failed read is a frame that SAYS it failed, with NO detections — never the last good frame held over
  (the scalar sensor adapters hold their last good value; a held frame would read as a fresh "nothing
  changed"). An Object's raw score is kept whole (0..65535; its scale is unknown). A read of 24 or more is `truncated` — "possibly full", never evidence that nothing else is
  there. `hal::AiVisionSettings` is what is written to the camera (the colour descriptors a team measures
  with VEXcode's AI Vision Utility, `hal::ColourDescriptor`; whether the AI model's objects are on; the
  class names the model must report), refused by name by `aiVisionSettingsProblem()` — an all-zero
  descriptor is UNSET, never a colour. `hal::CameraStatus` and `cameraStatusWords()` give the set-up and
  the model check in ASCII words ("set up, model OK", "set up, model DIFFERENT", "NOT set up", ...).
- **`hal::pros::ProsAiVision`** (`hal/pros/ai_vision.hpp`) — the adapter over PROS's C API. It WRITES the
  colours and the detection modes at construction — exactly the modes asked for, TAGS off, through the
  all-modes mask — and reads each back (PROS's setters report success whatever the camera did); writes
  them again after a read the camera does not answer and whenever the once-a-second check finds one
  changed; reads `NotConfigured` until a set-up holds, retried at most every 0.5 s. The once-a-second check
  reads the modes and EVERY colour back and re-asks the AI model, so a match is never more than a second
  old (a camera swapped while nothing read it is caught before its objects pass); a check the camera does
  not answer is a failed read, counted like a failed count read — never a "lost setting". It checks the camera's AI model BY
  NAME before an object id means anything — class 3 is `yellowPin` in VEX's Override model and `blueRing`
  in its Classroom model — reading each expected id's name into a zeroed buffer and never asking for a
  count: the vendored PROS 4.2.2 returns 1 for every id, whatever its header says (read in its
  `libpros.a`). Until the names match, every Object is WITHHELD (counted); colours still pass — but only
  the colours this program wrote: a blob from a descriptor another program left on the camera, and any
  colour code, is withheld too (counted, `coloursWithheld`). A torn read (an object with no valid type, or
  a tag) is a `PartialFault`. It never calls PROS's camera reset, white
  balance, tag-family, USB-overlay or colour-code calls. It is an `hal::IReadFaults` too.
- **`config::CameraConfig`** (`config/camera_config.hpp`) — the robot file's camera block: the port (0 = none),
  the settings, the MEASURED mount (`config::CameraPlacement`: forward, left, height, yaw, pitch, each with
  how well it was measured, and who measured it) and a note. An UNSET mount is not a config error: the
  camera is built and read, and only the geometry refuses (`vision::cameraModelProblem()` names why), so a
  team can bring a camera up a piece at a time.
- **`vision::CameraModel`** (`vision/camera_geometry.hpp`) — a box turned into a BEARING, an ELEVATION, a
  GROUND range (from where its base meets the floor), a SIZE range (from its known height) and a FIELD
  position with a radius — every one an INTERVAL, the EXACT range of the answer over every combination of
  the stated tolerances (the mount's, the optics' — VEX's 74 x 63 degrees, two focal lengths, never a
  square pixel, each with its own tolerance — and a box edge's jitter): computed at the corners of that
  box and at the few points inside it where an answer turns back (a box on the lens's level, the
  principal point's column), and the fix radius over the bearing, the range and the heading together. A
  ground range near the horizon is refused rather than infinite; a mount whose pitch WITH its tolerance
  reaches past 55 degrees (`vision::kMaxPitchSpanDeg`) is refused by the geometry, not by the robot file.
  The size range is exact for an upright line anywhere in the picture (its sideways angle and the
  camera's pitch carried); a Pin's box also takes in its top face, so its size range reads a little nearer
  than its near rim. A pose bound that is not a bound (a negative or non-finite tolerance) gives an
  INFINITE fix radius, never a tight one. `vision::boundAtImage()` widens a robot's pose bound by
  its speed and turn rate times the image's measured age; with no measured age, field positions hold only
  with the robot stopped.
- **`vision::override2026`** (`vision/override_elements.hpp`) — the season's data, every number cited to
  the Game Manual v2.0: the Toggle's leaning face (its lower edge 12.55–12.65 in up, the apex 14.39 in,
  the flat bottom face 12.36 in), the four Toggles' places, the Pin's and the Cup's sizes, and the
  Override model's class names.
- **`robot::Robot::camera()` / `cameraAbsence()`** — a config with a camera gets it built, LAST, through
  the provider. **A refused camera never refuses the robot:** `camera()` is nullptr and `cameraAbsence()`
  says why ("port 3: a vision sensor (expected an AI Vision sensor)"), and the robot builds and drives —
  a camera cable out at an event must never cost the match.
- **`config::PortUse::Camera`** — the camera's port is claimed like every other: a motor typed on it is
  refused naming both ("port 3 is claimed by the LEFT drive and by the AI Vision camera").
- **For test authors** (outside the reference — `test/README.md` documents them):
  `hal::fake::FakeVisionFrames` (a settable camera, its own read-fault count), the host PROS shim's `pros/ai_vision.h`
  (it answers as the vendored binary does where the binary and the header differ, and starts adversarial:
  station 12's tags on, a stray colour, VEX's Classroom model), and `sim::CameraPlant`, the host camera
  model (Toggles, Pins, Cups and lookalikes seen through a TRUE camera set apart from the believed one,
  with jitter, drops, a frozen or unplugged or torn camera; written with its own projection, so a sign
  error in the library's cannot cancel out).

**CHANGED (for an implementer, or a reader of the census words):**

- **`hal::IDeviceProvider` has a new pure virtual, `aiVision(port, settings)`**, and `readFaults(port)`
  answers the camera too. MAKING the camera WRITES its settings. A port outside 1..21, a port holding
  anything but an AI Vision sensor — the OLDER Vision sensor included, by name — and a settings problem are
  refused, naming the port; a second request with EQUAL settings returns the same camera and writes
  nothing, one with different settings is refused ("one owner"). Code that only CALLS a provider is
  unaffected; a class that IMPLEMENTS one must add it (the two in the library have). **Migration:** make
  the camera or refuse it naming the port, and answer `readFaults(port)` with it once made.
- **`hal::DeviceKind::AiVision`** — the brain's registry answer for an AI Vision sensor, appended.
  **A BEHAVIOUR CHANGE in words:** until now the registry called the AI Vision sensor `Vision`, the older
  Vision sensor's kind, so a census line said "VISION" for it and no check could refuse an older Vision
  sensor on the camera's port; it now says **"AI-VISION"** — so "shulib Calibrate"'s device census
  should list robot two's port 3 that way, and so would the boot check's refusal for a drive or IMU port
  found holding the camera, IF the brain's registry reports an AI Vision sensor as `E_DEVICE_AIVISION`
  (unmeasured: no census so far printed a word that tells the two apart) — and `Vision` means the older
  sensor only. A `switch` over `DeviceKind` needs one more case.
- **The robot's boot line** (`robot::Robot::describe()`) ends with the camera's words where it ended
  "vision ABSENT": "camera ABSENT" with none declared — every program's line today —, "camera port 3
  ABSENT (why)" refused, or "camera port 3 set up, model OK" and the other states, the reason in
  parentheses, ASCII only. A test or a tool that matches the old words must take the new ones.
- **`config::RobotConfig::valid()`** checks the camera when its port is not 0: the port, its claim, its
  settings, and a mount that is set. A config without a camera is unaffected. **What you must do:**
  nothing.

**TESTED:** every refusal by name on both sides of each range (a hue of 1 and 40 accepted, 0.99 and 40.01
refused); the adapter over the shim — the set-up written and read back, a write that "succeeds" and changes
nothing caught, the retry's 0.5 s and the read-back's 1 s each landed on exactly, each colour field compared
on its own, only the binary's exact failure record taken for a camera that did not answer (a wiped slot is a
changed setting), descriptor 7 written and held, a dead camera's `PROS_ERR` (2,147,483,647) never read as a
count, 23 complete and 24 possibly full, a torn read and a tag not evidence, the Classroom model withholding
every object, one wrong name of five, a camera that names nothing, a model re-plugged; the provider's refusals
for every kind the brain reports; the geometry against hand anchors and, over 400 placements, against the host
camera model with a true camera moved to the edge of every tolerance (every interval held the truth); every
interval swept against the WHOLE tolerance box at 100 % of each tolerance, the pitch inside it included (each
held the sweep's extremes and reached them), and the fix radius against a sweep of bearing, range and heading;
an image 66 ms old while turning 180 deg/s shown to put a Pin outside the fix computed without
`boundAtImage()`, and inside it with; the Robot's camera words in every state, and a refused camera with the
robot still driving; each load-bearing rule's mutation run and seen to fail a test. Twelve beliefs about the
camera and the field are new in the [register](hardware-assumptions.md): HA-192..HA-203 — none measured.

**What you must do:** nothing, unless you implement `hal::IDeviceProvider` or match the census or boot
line words (above).

### 2026-10-30 (built 2026-10-29) — the mechanism core, part 5: robot two's programs run its claws on the core — no API change

> Written when it was built (chunk MECH-A5, the fifth of the mechanism core's six parts). **No version
> bump and no library change:** nothing under `include/shulib/` moved but its comments. **Robot two's claws
> behave exactly as before on every input** — R1 / R2 run the port-6 pivot forward / back at full power
> while held and put it in Hold when let go, L1 / L2 the port-7 pivot, A opens and closes the pneumatic
> claw — host-proven tick for tick against the bindings they replace. **Nothing here has run on a robot.**

What a library user can see in it is the core used end to end in real programs — the pattern for a
team's own:

- **The robot file declares, and wires separately** (`src/robots/mechanism_table.hpp`): robot two's two
  claw pivots are `Power` declarations and its claw a `Pneumatic` (closed = `Low`, open = `High` — a
  guess, HA-159 — and a move time of 0.5 s, a guess, **HA-185, NEW**), inside the namespace `robot_two`,
  then ONE `mechanisms(wire(...), ...)` list; the selection of a build's list is one function with a
  case per robot. The build refuses a mechanism wired to its own robot's drive ports or IMU port —
  every role, sensors included.
- **Every program that boots robot two binds ONE mechanism service** over that list and the Robot's
  built devices. The two that run autons (the competition program and the Auton Tester) put it in
  their pacer chain between the run guard and their watch, and **prove the chain at boot**: one idle
  tick must reach the guard, the service and the watch — the refusal names which link is missing.
  "shulib Drive" runs no auton and has no guard: its driver loop ticks the service itself.
- **The boot prints each mechanism**, tag `MECH`: what it is declared, one line per role (where it is
  plugged in, and whether the brain found it), what the driver file's control does with it, and what it
  lacks (`'claw pivot 6': no stall signature -- held against its stop it is cut only by the driver
  letting go (HA-154)`); the starting-values line for a position or speed mechanism.
- **The route ids are registered** in each auton program (`runner::MechanismRoutes::registerInto`), and
  its boot says how many: `ROUTES: 6 mechanism route ids published (claw_pivot_6_run .. claw_open); 26 of
  the registry's 32 left for the team's own` — so a route auton's markers can fire the claws.
- **Robot two's driver file is three lines of controls** (`whileHeld(clawPivot6, Button::R1,
  Button::R2, 12_volt)`, the same for pivot 7, `toggle(claw, Button::A)`), the feel and the loop's two
  numbers; the plumbing every driver file shares is one template the programs hold, which refuses at
  build a control on the loop's re-zero button or a loop tick that is not the brain's, and at boot a
  robot mechanism no control names. [Cookbook chapter 6](cookbook/06-driver-control.md) shows it.
- **The team's autons** build every routine on the context (`Routine r{ctx, "40in-forward"}`) and name
  robot two's mechanisms through its namespace; the host suite runs every registered auton with the
  service live. [Guide chapter 16](guide/16-testing-an-auton-on-the-bench.md#the-claws-in-an-auton)
  shows a claws auton and a claws route, each step's verdict asserted by a host test.

**What you must do:** nothing — no library surface changed. **Hardware assumptions:** HA-185 is NEW
(the claw's 0.5 s move time — invented, timed at the bench worksheet's Station J.9.8); HA-159's flip is
now "swap `Low` and `High` on the claw's `.positions` line". See the
[hardware assumptions register](hardware-assumptions.md).

### Fixed since 2.5 (2026-10-29) — the IMU's turn rate answers once per control tick

> Written when it was built (chunk IMU1 Part A; the team lead's "accept all" of 2026-10-22). **No
> version bump — a BEHAVIOUR change inside an adapter and one new constant:** `hal::pros` adapters are
> implementations, not contracts (the [Freeze Register](roadmap.md#freeze-register)'s F13 row), no
> signature changed, and the frozen `IImu` (F4) is untouched. **Host and build gated only — nothing
> here has run on a robot**: the defect is DERIVED from the adapter's code and PROS's documented 10 ms
> data copy (A4 register HA-213, never measured on our brains) and reproduced on the host test plant;
> whether robot two's copy timing lands in the bad band (HA-214) is unknown.

**FIXED — a second read of the IMU's turn rate in one tick answered about 0 or a spike, and spoiled the
next tick's first read.** `hal::pros::ProsImu::yawRate()` (every robot's IMU) differentiated the IMU's
rotation between CALLS, and the library reads the rate two or three times a tick — the Localizer; every
motion's record when the sink wants records (robot two's serial sink wants every one); a GPS or tag
corrector's gate; the calibration classes. On a robot those reads are microseconds apart while PROS
copies the IMU's numbers only every 10 ms, so the second read answered about 0 (or a spike when a copy
landed between the two) AND moved the differentiation baseline: the NEXT tick's first read — the
Localizer's, which TANK1's yaw observer and `DriveBrake` use — read about 3 % high, or, for one band of
the copy's timing, 0 on every tick. On the host test plant, with TANK1's own motor-lag values, that band
turned the 40 in auton from FINISHED into a 5 s TIMEOUT. Now `yawRate()` keeps its answer for
`ProsImu::kRateHoldUs` (5 ms, half the 10 ms tick): a read within 5 ms of the last fresh answer returns
that answer and touches neither the sensor nor the differentiation baseline. Every reader in one tick
gets the same value — the Localizer, every record (whose `imuYawRate` is now the Localizer's value), a
corrector's gate, the calibration classes, and your own `imu.yawRate()` from inside a library tick. The
hold covers the gyro source (`YawRateSource::GyroRateZ`) too. `faultedReads()` no longer counts a held
read: a dead sensor counts one rate read per tick. First recorded at DOCS2 (A17), re-found by TANK1.

**Added:** `hal::pros::ProsImu::kRateHoldUs` (`std::uint32_t`, 5000 µs) — the hold, in whole
microseconds, pinned to `ProsTickPacer::kTickMs` by a test.

**Breaking or additive:** a behaviour change in an adapter; no signature changes and no version bump.
On the host test plant nothing changes for any existing program: every host clock stands still inside a
tick, where the old adapter already answered every in-tick read with one value — the suite's every
other case gives byte-for-byte the same results.

**What you must do:** nothing, for code that reads the rate inside a library tick: within one library
tick, every read gets the library's value. **The hold's three limits** (`ProsImu`'s header; A4 register
HA-215 — the hold infers "the same tick" from the clock, not from the loop; the figures are the host
test plant's, pinned by IMU1's tests L1–L3 and T9 (b)):

- **(1) A stall or a late wake.** A tick that stalls more than 5 ms between two of its reads (a slow SD
  write, a long record) gives its later readers a fresh answer, and the Localizer's next two ticks read
  off — the first held from that answer, the second low; a tick that wakes 5 ms or more late is followed
  by an on-time wake less than 5 ms after it, which gets the late tick's answer, one tick stale (three
  of the Localizer's ticks off, where a 4 ms late wake, which the hold does not touch, costs two). Then
  nothing — neither repeats without its cause.
- **(2) The tick's first read moving** — see *Not fixed*, below.
- **(3) A reader outside the tick.** A read from OUTSIDE the library's loop — between routine steps, or
  from a task of your own — 5–10 ms after a tick's first read makes the library's next tick read a stale
  rate and the one after a skewed one: two of the Localizer's ticks per such read (one such read every
  10 ticks: 60 of 300 ticks off, where the adapter before IMU1 lost 30 — measured on the host test plant
  on 2026-10-29 with that adapter put back; no test holds that figure). And a loop of your own at any
  period other than a multiple of 10 ms gets a rate that is 0 on a large share of its reads (the IMU
  copies new data every 10 ms — A4 register HA-213; a 4 ms loop read 0 on one read in five and never
  exactly). For your own loop, differentiate the heading over 10 ms or more yourself, or use the gyro's
  rate once HA-04 and HA-109 are settled.

**Not fixed** (limit (2)): the residual of A4 register HA-214 — the IMU's copy landing next to the tick's
one rate read, or between two positions of a tick's first read (a corrector's gate on a fresh-fix tick
against the Localizer on a stale one; Calibrate's REST capture starting) — gives a pair of ticks 0 and
about double (a GPS fix every 5th tick, with the copy between the gate's read and the Localizer's: at
most 60 zero and 60 double of 300 ticks). Its cure is the gyro's own rate, once the bench measures its
sign (HA-04).

### Added since 2.5 (2026-10-29; built 2026-10-18–22, reviewed and fixed 2026-10-22–29; its typed drive ports 2026-11-03, built 2026-10-30) — the standard Calibrate: a team states its drive kind, its cartridge and its mechanism motors, and every team runs the same Calibrate

> Written when it was built (chunk CAL1, Parts A2a and A2b of four: A2a the standard program, with the
> card's record as the front check's reference; A2b, 2026-10-30, a team's TYPED drive ports as the
> reference when the card holds no usable record — the team lead's D6 (a) — the guide's
> [chapter 19](guide/19-calibrating-your-robot.md) with a team's whole Calibrate file, and the program
> run through its real PROS shell in 34 recorded scenarios; robot two moves onto it in Part B). **No
> version bump — every change is ADDITIVE:** two new headers, and additions to
> `calibration/calibrate_panel.hpp` whose defaults are today's screens (a host test renders every view
> of four runs both ways and requires them equal, field for field); `calibration/` is in no row of the
> [Freeze Register](roadmap.md#freeze-register). **No program uses it yet:** robot two's "shulib
> Calibrate" is still its own program, and paints, prints and saves exactly as before. **Host only —
> this program has not run on a robot in this form** (the calibrator inside it ran on robot two on
> 2026-09-24).

**Why:** the team lead asked on 2026-10-06 for Calibrate to ship WITH the library, so every team — and
the VS Code panel, which reads the record Calibrate saves — runs the same, correct Calibrate. This part
moves robot two's program into the library over the seams the previous part added, so the whole
program runs in the host test suite on the host test plant, and adds what his rulings asked for: the
end that LEADS the push is the robot's front, with a check against the card's record before anything
is saved; the safe save; and a watch of the motors a team says are not drive motors.

**ADDED:**

- **`calibration::CalibrateProgram`** (`calibration/calibrate_program.hpp`) — the standard Calibrate,
  PROS-free: `CalibrateProgram(facts, devices, buildStamp).run()` from `opcontrol()`. It finds the
  motors and the IMU from the brain's registry (never a typed port list), drops every DECLARED mechanism
  port before any device is made, refuses by name — before any device is asked for — facts it cannot
  use, makes one motor per candidate port through the robot's `hal::IDeviceProvider`, runs
  `calibration::DrivetrainCalibrator` with the operator answering on the touch screen (the push, the
  pulses, the spin), prints the serial record robot two's program prints, and offers RUN AGAIN. Every
  exit leaves every motor at 0 V and coasting; the competition status is polled every tick, and the end
  of driver control — the field's disable, or the start of autonomous, with or without a field — ends
  the run cleanly, its words naming which. **`CalibrateProgram::kPushLogMinSeconds`** (0.25 s) and
  **`kPushLogKeepaliveSeconds`** (5 s) are the push's live serial line's two intervals: a changed count
  at most that often, an unchanged one at least that often.
  - **The front is the end that leads the push** — the screen says PUSH FRONT FIRST, and the
    calibrator's old WHICH END question is answered by the program itself, never painted.
  - **The FRONT CHECK**, before anything is saved: the card's record, read the way the boot reads it
    (the record file, else its backup), against this run's, with `calibration::compareFronts()`. A card
    with no record: the first record, saved with the note `FIRST RECORD -- the end you pushed first is
    FRONT` (painted only when the save succeeds — under NOT SAVED it would name a record the card does not
    hold; a card that READ as holding none: a good record misread at the check makes an unchecked first
    record — the safe save's two-fault limit, stated in the header; no third read hides it). Files that do not decode: saved, each file named with its reason (`EMPTY (0 bytes read)` for
    an empty one), note `FRONT NOT CHECKED`. CANNOT TELL: saved, the same note. **FRONT FLIPPED: NOTHING
    is written** until the operator touches USE NEW (saved as pushed — right when the drive cables were
    swapped between the sides, or the front was changed on purpose) or RUN AGAIN (nothing saved — right
    when the back led by mistake); the records cannot tell those apart, so the screen says what the
    records say, never what the operator did. No SD card: no check (nothing can be saved), and the
    words say there is no card.
  - **The WATCH of the declared mechanism motors**: each is READ as the push begins and again at DONE —
    never commanded, never configured (on the brain, PROS's C reads by port and no motor object; the
    belief that such a read writes nothing to the motor is register entry
    [HA-187](hardware-assumptions.md)). One that
    turned as far as the calibrator's own threshold for a drive port **holds the save** on MECHANISM
    PORT TURNED, with its travel beside the drive's and whether the card's record drives with it: SAVE
    (a mechanism the push turns: kept declared, note `PORT N KEPT AS A MECHANISM`) or RUN AGAIN (a
    drive motor declared by mistake: fix the facts). **Every declared port that turned is its own
    question**, one screen each in the facts' order, and nothing is saved until the last is answered
    SAVE (note `N PORTS KEPT AS MECHANISMS`) — so an intake the push legitimately turns can never stand
    in for a drive motor declared by mistake. After a USE NEW the first of them follows; each new
    question's boxes are greyed again for the arming delay, so a reflexive second tap cannot answer it.
  - **The save is the safe save** (`calibration::saveRecordSafely()`, a verified backup first; the
    words `calibration::formatSaveReport()`'s): SAVED is said only when the card measurably holds the
    new record, and a failed save never erases a good one.
  - **A motor the facts do not list is a drive candidate**, and one the push turns is POWERED (3, 6,
    9 V) like a drive motor: the program says so in its banner when nothing is declared, names every
    candidate the push did not turn ("never powered -- if it is a mechanism, declare it"), and adds the
    same advice to a LEFT OUT line and a pulse refusal.
  - **The TYPED drive ports as the reference (A2b — the team lead's D6 (a)).** A team whose drive
    program already types its drive ports may state them; with a usable record on the card they change
    nothing (not a line, a draw or a byte). With NO usable record on a present card — no file, or files
    that do not decode — the front is checked against them instead: the `card :` line and the check's
    lines say "your typed drive ports", and list them; SAME with every port agreeing saves with the note
    `FRONT MATCHES YOUR TYPED DRIVE PORTS`; SAME with any port differing (a sign, a side, a port in one
    list only) saves with `FRONT MATCHES, SOME TYPED PORTS DIFFER -- see log` and one serial line per
    port; CANNOT TELL saves with `FRONT NOT CHECKED`; FRONT FLIPPED holds the save in the typed ports'
    own words (a wrong push → RUN AGAIN; the front led → USE NEW, then `FIX YOUR TYPED DRIVE PORTS`, a
    serial line and the note). A MECHANISM PORT TURNED screen's evidence names the typed ports when they
    were the reference (`your typed ports DRIVE with port N`). The words that say what a discarded run
    leaves follow the CARD's contents, never the reference. They are compared, never copied into a
    record; with no SD card nothing is checked, typed or not.
- **What a team states — `calibration::CalibrateFacts`**: the drive kind (`config::DrivetrainKind`;
  only `Tank` is measured, anything else is refused `NOT A TANK`), the drive motors' cartridge (never
  defaulted: unset refuses), and `mechanisms` — EVERY motor that is not a drive motor
  (`calibration::MechanismPort`, a port and a name; up to `kMaxMechanismPorts`). Optional: the robot's
  name and a stated IMU port (a cross-check; the registry decides). **`calibration::factsProblem()`**
  (constexpr, so a build can `static_assert` its facts) names the first unusable entry — a NAMED entry
  with port 0, a port past 21, a port listed twice — as a `calibration::FactsProblem`;
  **`calibration::declaredMechanism()`** finds a port's entry. **(A2b)** The optional, LAST member
  **`CalibrateFacts::typedDrive`** — a **`calibration::TypedDrivePorts`** (`left[21]`, `right[21]`,
  signed ports in a chassis table's own spelling, 0 = an unused slot) — written
  `.typedDrive = {.left = {-11, 12, -13, 14, -15}, .right = {20, -19, 18, -17, 16}}`, or from a
  `config::RobotConfig` with **`calibration::typedDrivePortsFrom(kRobot.drivetrain.tank.description)`**;
  **`calibration::hasTypedDrive()`** (any entry on either side) and
  **`calibration::typedDriveDescription()`** (the two sides as a description, zero slots skipped, the
  order kept). `factsProblem()` checks the typed lists after the mechanisms — a port outside 1..21, a
  port listed twice on one side or once on each (by |port|), only one side typed — and
  **`FactsProblem::list`** (a **`calibration::FactsList`**: `Mechanisms`, `TypedLeft`, `TypedRight`)
  says which list `entry` indexes; the refusal names `typedDrive.left[i]` or `.right[i]` and its value.
  A file that names the other members is unchanged by it.
- **What the program needs from its platform**: **`calibration::CalibrateIo`** — four calls (one serial
  line, the competition status, a plain wait, the watch's read) — and **`calibration::CalibrateDevices`**
  (the provider, the registry, the `hal::IFileStore` card, the `hal::IBrainScreen`, the IO); the
  competition bits **`kCompetitionDisabled`/`Autonomous`/`Connected`** and **`competitionWords()`**;
  **`calibrateBootBanner()`** for `initialize()` and **`calibrateStateScreen()`** for the other three
  entry points.
- **The robot's shell** (`hal/pros/calibrate.hpp`): **`hal::pros::ProsCalibrateIo`** (each serial line
  ONE tagged write, `[R3D] …`, through `serialWrapped()`; the watch's three C reads, converted to
  degrees by the units and gearing the motor reports) and the five entry functions a team's file
  calls — **`calibrateInitialize()`**, **`calibrateDisabled()`**, **`calibrateConnected()`**,
  **`calibrateAutonomous()`** and **`runCalibrate(facts)`**, which composes the PROS registry, device
  provider, SD store, screen and IO on its own frame (its stack on the opcontrol task is stated in the
  header: 22,464 B of 32 KiB as GCC measures it) — and **`hal::pros::calibrateStateScreen(state, why)`**,
  the competition-state screen the three state functions share, over the brain's own screen. Include
  it in ONE translation unit (its default build stamp is that unit's).
- **`calibration/calibrate_panel.hpp`, additions only:** `FrontRule` (`AskedAfterPush`, today's;
  `ByPush`, the standard Calibrate's: steps 1/4..4/4, PUSH FRONT FIRST, the front rule while the IMU
  warms up, never the WHICH END view), `FrontChoice`, `HoldReason`, `FinishFacts` (the hold, its two
  evidence lines, the finished screen's one NOTE row — a two-row calibrator warning is then cut to one
  ending ` ...`, the full text on serial), the appended `PanelScreen::Held`, `SaveOutcome::Held` and
  `TouchAction::SaveHeld`, and two defaulted parameters on `viewFor()` and `TouchGate::view()`. A held
  screen's left box is today's FRONT FIRST button relabelled USE NEW or SAVE (no new button id — today's
  programs switch over them); `TouchGate` reads it as `SaveHeld`, never an answer, and restarts its
  arming delay when the held question changes — its reason, or `FinishFacts::holdPort`, the port a
  MECHANISM PORT TURNED screen asks about. Every new word is a `calibration::panel_text` literal checked
  at compile time against its line's budget: the four step headers (`kHeaderImu4`, `kHeaderPush4`,
  `kHeaderPulses4`, `kHeaderSpin4`), the two held headers (`kHeaderFlipped`, `kHeaderMechTurned`) and
  `kHeaderSavedNew`; `kPushFrontFirst`, `kNothingSavedYet`, `kNextTouchDoneClear` and `kFrontRule`; the
  FRONT FLIPPED lines (`kFlipWhat`, `kFlipAgain`, `kFlipNew`, `kFlipWarn`) and the MECHANISM PORT TURNED
  lines (`kMechNotOnCard`, `kMechSave`, `kMechAgain`); the two box labels (`kLabelNew`, `kLabelSave`); the
  NOTE literals (`kNoteFirst`, `kNoteNotChecked`); and `kWarningCut`, the ` ...` that ends a cut warning.
  **(A2b)** `FinishFacts::typedPorts` (default false): the FRONT FLIPPED view then paints the typed
  ports' three lines — `kFlipWhatTyped`, `kFlipNewTyped`, `kFlipWarnTyped` — in place of the card's; and
  the three typed NOTE literals, `kNoteTypedSame`, `kNoteTypedPartial` and `kNoteFixTyped`.

**What you must do:** nothing. A team that wants the standard Calibrate writes the 21-line file
[chapter 19](guide/19-calibrating-your-robot.md) shows (the typed drive ports are one optional line).
No program in this repository runs it yet: robot two moves onto it in CAL1's Part B.

**For contributors:** the host PROS shim's per-port motor state (`test/pros_shim/pros/motors.hpp`)
gained three dropout knobs — `failPositionReads`, `failUnitsReads` and `failGearingReads` — each
failing the next N calls of ONE of `pros/motors.h`'s three C reads while the other two answer (a
dropout between two reads of one port), so a test reaches each of a read's guards alone. **(A2b)**
`test/calibrate_golden_test.cpp` runs the standard Calibrate through its REAL PROS shell — the PROS
adapters over that shim, robot two's shape on the host test plant — in 34 scenarios, each held to
committed goldens in `test/goldens/calibrate/` (the serial log, the screen's draw log, the card's two
files): a change to what the program says, draws or saves fails it, naming the first differing line.
Set `SHULIB_GOLDEN_OUT=<dir>` to have every scenario's actual files written there as well, to read and
copy in by hand — nothing regenerates itself. `test/calibrate_team_file_example_test.cpp` holds the
team's 21-line file, compiled and run. In passing, `calibrate_panel.hpp`'s chips line copies exactly
the characters it built (`"%.*s"`): the line drawn is unchanged, and GCC's `-Wformat-truncation` at
`-O1` no longer fires on it.

### Added (2026-10-29; built 2026-10-22) — the mechanism core, part 4: a mechanism step on one line of an auton, presets on the driver's buttons, every mechanism's route ids run, and the direction check's judgement

> Written when it was built (chunk MECH-A4, the fourth of the mechanism core's six parts). **Minor bump,
> 2.4 → 2.5** — the recipe layer ([Freeze Register](roadmap.md#freeze-register) row F10) and the
> command vocabulary (row F8), both FROZEN, grow by the paths their freezes name: a new `Routine`
> constructor (a new overload — the frozen `Routine(Chassis&, const char*)` is untouched), new steps,
> an APPENDED `RoutineStopCause` value and an APPENDED `RoutineResult` field; an APPENDED
> `CommandEntry` field and a new `CommandRegistry::on()` overload. Whether the mechanism steps, the
> context constructor and `lastMechanism()` join F10's freeze is **not yet decided** (the register's
> F10 row gives both readings): they are not pinned meanwhile. Nothing that existed changed shape or
> behaviour: **every 2.4 program runs exactly as before**, and compiles as before unless it `switch`es
> over every `RoutineStopCause` (see *What you must do*). No program builds a mechanism service yet — robot two's programs and its
> driver file move onto it in the core's next part. Everything was measured on the host test plant's
> invented models (HA-183), with the library's invented starting values (HA-182). **Host only — nothing
> here has run on a robot.** [Chapter 18](guide/18-mechanisms.md) teaches all of it.

**ADDED:**

- **The mechanism STEPS — one line each** (`chassis/routine.hpp`): `r.move(arm["score"], 1.5_s)` (a
  preset, `arm.to(120)`, `lift.to(18_in)`, or a computed `lift.clampedTo(h)`), `r.home(lift, 1.5_s)`,
  `r.run(intake, 12_volt, seesPiece, 3_s)` (any done-condition, or a time: `r.run(clawPivot6, 12_volt,
  0.4_s)`), `r.set(claw["closed"])` (its time is the robot file's `.moveTime`), `r.spinUp(flywheel["shoot"],
  2_s)`, `r.stop(intake)` / `r.stop(flywheel)`, and the START forms `startMove`, `startHome`, `startRun`,
  `startSpinUp` — each returns at once, so the next drive runs while the mechanism works — collected by
  `r.waitUntilDone(intake, 1_s)`. Each step runs the moment it is chained, through the robot's mechanism
  service, and every one states its time limit. **The verdict policy:** SUCCEEDED continues; UNCONFIRMED
  COUNTED done (no sensor exists to confirm it — robot two's sensorless claw) continues; every other
  verdict (TIMED_OUT, STALLED, NOT_HOMED, ABSENT, REFUSED, CANCELLED, an UNCONFIRMED not counted) STOPS
  the chain — the drive safed, the rest skipped — naming the verdict in the stop line
  (`routine 'score' STOPPED at step 1 (intake): mechanism step FAILED (TIMED_OUT) — skipping the rest;
  drive safed`), in the new `RoutineResult::mechanismVerdict` and in the new `Routine::lastMechanism()`. A
  START form stops the chain only when its start is ABSENT, REFUSED or NOT_HOMED. A step on a mechanism
  this robot does not have is ABSENT, said (`'claw' is not on this robot (its wiring list does not have
  it): the step did nothing` — or, for another robot's same-named mechanism, that the one here is not the
  one the step names; or, when the program never bound its service, `'arm': the mechanism service is not
  bound (the program bound no robot list): the step did nothing` — the drive steps still run). A blocking
  step whose wait sees the service NEVER tick stops the chain saying the
  service is not in the chassis pacer's chain — a program's wiring fault found at its first blocking
  step. `waitUntilDone` collects only a step THIS routine began (none: a precondition; one replaced from
  outside: CANCELLED). After its step a mechanism keeps its behaviour — an arm HOLDS, a flywheel keeps
  spinning — until another command, the end of the guarded run, or driver control.
- **`Routine r{ctx, "name"}`** — a second constructor taking the auton's context: the chassis AND the
  mechanism service. A routine built from `ctx.chassis` (the frozen constructor) runs its chassis steps
  as ever; a mechanism step on it is a loud precondition naming the fix.
- **`RoutineStopCause::MechanismStepFailed`** (APPENDED, `== 5`) — a failing mechanism step's cause.
  `MechanismFailed` keeps its meaning: a `then()` action's mechanism verdict.
- **The wrong step for a kind is refused BY NAME**, at your line: `r.run(arm, …)` ("r.run(...) is for a
  POWER mechanism -- move an arm or a lift with r.move(...)…"), `r.run(intake, 12, 1_s)` ("volts are
  written with their unit: 12_volt, not 12"), `r.move(intake, …)` / `r.move(arm, …)` ("r.move(...) takes
  a TARGET…"), `r.stop(arm)` ("r.stop(...) is for a power mechanism or a flywheel…").
- **`competition::AutonContext::mechanismService`** (APPENDED, default `nullptr` — a positional
  `AutonContext{chassis, guard, telemetry, mechanisms, entry}` still compiles) and a **`runAuton`
  overload taking the `mechanism::MechanismService`**: the guard's span is the service's built
  mechanisms, the routine's context carries the service, and — first, before any refusal — the
  service's `enterAutonomous()`: whatever a deleted driver-control task left running (a held intake at
  12 V) stops before the auton, and a claim another owner left is released and said by name. The
  service must be in the chassis's pacer chain under the guard.
- **The driver file's mechanism controls** (`mechanism/driver_inputs.hpp`): `controls(kRobotXMechanisms,
  …)` — ONE list, checked at build against the robot's list — of `whileHeld(arm, Button::R1,
  Button::R2, 6_volt)` (a power mechanism: run while held, its safe state on release; an arm or a lift:
  jog while held, HOLD HERE on release), `toggle(claw, Button::A)` (a two-position pneumatic) /
  `toggle(flywheel["shoot"], Button::X)` (a flywheel's NAMED speed on / off), `preset(arm["score"],
  Button::Right, 1.5_s)` (a position, a speed, or `preset(hood["up"], Button::Up)` with no time limit),
  `homeOnPress(lift, Button::Left, 1.5_s)`, `homeAtDriverStart(lift, 1.5_s)` and `autonOnly(hood)`. The
  build refuses, BY NAME: a button doing two jobs (`button_Right_does_two_jobs`), a mechanism the robot
  lacks, a hold outside (0, 12] V, a preset or homing with no time limit, two controls of one kind on a
  mechanism, a toggle on anything but a two-position pneumatic or a flywheel's named speed, homing a
  never-homed mechanism, **a switch- or stall-homed arm or lift held or preset with no way to home it in
  driver control** (`homeAtDriverStart` or `homeOnPress`; the declaration's `.unhomedVolts` answers for a
  mechanism that is only HELD — a hold moves it unhomed but never homes it, so a preset beside it needs a
  homing control; its buttons would be dead after any restart off its home), a never-homed one held with
  no `.unhomedVolts` or preset at all, a
  mechanism with no control unless marked `autonOnly`, and more than 24 controls. At run time
  `mechanism::DriverInputs` (static storage, bound once at boot) runs them: **the most recent input
  wins** — a held button that sees something newer speak (a preset, a toggle, a stall) commands nothing
  until it is released; a toggle's, a preset's or a homing button's press must be SEEN (a button down at
  entry, or still down when a dropped controller comes back, is not a press, and a drop itself presses
  nothing); a hold's button is a level — down at entry, it drives at once; a drop releases every hold;
  both buttons of a hold = neither. **While a `homeAtDriverStart` home runs, that mechanism's own
  controls wait** (a hold held during it does nothing until released after it; a preset or homing press
  during it is dropped) — a touch would otherwise cancel the home and leave its buttons dead. A flywheel
  toggle's step is bounded by
  `kToggleAtSpeedWindowSeconds` (5 s, HA-182 — its logged verdict only; the wheel keeps spinning).
- **A `teleop::runDriverControl` overload taking the service and its `DriverInputs`**: at entry the
  service's `enterDriverControl()` then the inputs' `enter()`; each tick, after the drive and BEFORE your
  panel, the inputs command the service and the service ticks — EXACTLY once per tick, whichever pacer
  the loop is handed; on a disable the service's `leaveDriverControl()` (motor mechanisms safe, a
  pneumatic keeps its level — [HA-157](hardware-assumptions.md): whether the brain keeps a 3-wire
  output's level while disabled is unmeasured). It refuses, before anything is commanded, a T3 binding beside the
  service (one driver path per mechanism) and a control on the re-zero button. Every drive protection
  is unchanged.
- **The route binder** (`runner/mechanism_routes.hpp`): `runner::MechanismRoutes routes{chassis, service,
  kRobotTwoMechanisms, sink}; routes.registerInto(registry);` registers every id the robot's list derives
  (`arm_score`, `intake_until_piece`, `claw_closed`) in ONE call, each with its doc. A marker runs
  exactly the step an auton's line runs, through a `Routine`, and BLOCKS at its waypoint until the
  step's verdict; SUCCEEDED and a counted UNCONFIRMED continue the route, any other verdict stops it
  (`MarkerFailed`). Its args are the route author's: the time limit in ms (`arm_score [1500]`), and a
  power id's volts within ±12 and not 0 (`intake_until_piece [12, 3000]`); a marker without them is
  REFUSED with one line naming the id and what it needs, and the route stops, nothing commanded.
- **A doc per command id** (F8, "new registry members"): `runner::CommandEntry::doc` and
  `CommandRegistry::on(id, handler, argCount, doc)`; the manifest publishes a registered id's doc where
  it published `""` (robot two's: `{"id":"claw_pivot_6_run",…,"doc":"claw pivot 6: run; args[0] =
  volts (-12..12), args[1] = how long, ms"}`). A doc with a `"` or a `\` is refused at registration. **A
  registry with no docs writes exactly the bytes it wrote before.**
- **The direction check's judgement** (`mechanism/direction_check.hpp`; its bench page is the core's last
  part): `judgeDirection(spec, wiring, observation, minMoveRad, drawAmps)` and `judgePneumatic(...)` turn
  a jog's encoder readings and the person's THREE answers (its + way / the other way / it did NOT move,
  or something ELSE moved) into a finding and the exact place to change — the declaration's line for a
  direction, the `wire(...)` line for a cable on another mechanism's port, a generated robot file's JSON
  path (the value to write there, then "regenerate, rebuild and upload") when it states one. A motor
  against its command is a FIGHT only while a mate moves with its own; when every motor that moved went
  against it the mechanism was BACK-DRIVEN (gravity, a load) and no line is named. It REPORTS; it saves
  nothing.
- **The words** an auton and a driver file need, beside the robot file's: `Routine`, `RoutineResult`,
  `AutonContext`, `Button`, `DriverProfile`, `DriverLoopConfig`, `ArcadeBody`, `ArcadeField`,
  `TankSticks`, and the control words — all from `using namespace shulib::mechanism::words;`.

**CHANGED (documentation only; no behaviour of a 2.4 program):**

- **REFUSED** (`manipulation::MechanismOutcome::Refused`) now has a second meaning beside "another owner
  holds the claim": a route marker whose args are wrong. Both say nothing was commanded.
- **T3's bindings are superseded, not removed**: `teleop::MechanismBinding`, `teleop::ToggleBinding` and
  the `runDriverControl` overloads that take them stay, unchanged, for a program with no mechanism
  service; a robot whose mechanisms are declared drives them through the service overload.

**What you must do:** a `switch` over `RoutineStopCause` that lists every value needs one more `case`,
`MechanismStepFailed` — under `-Wswitch` as an error (shulib's own build) it stops compiling without
one, and shulib's competition program and Auton Tester each gained exactly that line. Nothing else:
every other 2.4 program compiles and behaves the same (`kApiMinor` is now 5 and `kApiVersionString`
`"2.5"`). To use the steps, an auton builds its routine from its context:
`Routine r{ctx, "score-three"}`, in a program that hands `runAuton` its mechanism service — robot two's
programs do from the core's next part.

## API 2.4

### Added and changed since 2.4 (2026-10-22; built 2026-10-15) — the mechanism core, part 3: arms, lifts and flywheels move, hold, find their zero and spin; every one watched for a jam

> Written when it was built (chunk MECH-A3, the third of the mechanism core's six parts), and revised by
> its review before it landed (2026-10-22). **No version bump — every change is ADDITIVE in its
> signatures:** new classes and members of the mechanism service, three constants, two functions beside
> the starting values, a boot-line formatter, one new field of a goal (`PositionGoal::units`), and one
> APPENDED build refusal (`spec.hpp`'s own append-only rule). **One precondition LIFTED** (`bind()` takes an `Arm`, a
> `Lift` and a `Flywheel`) **and one NEW build refusal** (below). The auton steps that call these
> behaviours, the driver controls and the bench page are the core's next parts; **no program builds a
> service yet**. Everything was measured on the host test plant's invented models (HA-183), with the
> library's invented starting values (HA-182). **Host only — nothing here has run on a robot.**
> [Chapter 18](guide/18-mechanisms.md) teaches all of it.

**ADDED:**

- **The POSITION behaviour** (`mechanism::PositionSlot`, an `Arm` or a `Lift`; `MechanismService::position(i)`,
  `find(arm)`, `find(lift)`, `find(arm["score"])`): `moveTo(position, timeLimit)` — a smooth trapezoid in
  the output's degrees (a lift's speed in inches turned into degrees by its STEEPEST slope — exact for a
  spool, a linkage's level slope, a measured table's steepest segment — so the plan is never faster in
  inches than stated anywhere on its travel, and never asks the motor for more than it has near a
  linkage's top), P + D on the plan with feedforward and
  the declaration's gravity hold, then a FRESH hold loop (P + I + D, its integral clamped to 3 V) when the
  plan ends; REACHED is judged in the mechanism's own units (a degree tolerance converted at a measured
  table's end or a linkage's top was 0), for the settle time: SUCCEEDED, and it goes on HOLDING. Out of
  time it is TIMED_OUT and HOLDS WHERE IT STOPPED — never pushing on toward a target it could not reach.
  `home(timeLimit)` finds its zero by the declaration's method; `holdHere()`, `jog(volts)` (the driver's
  hold-to-run) and `release()`; `homed()`, `position()`, `target()`, `softStops()`, `jointDeg()`.
  **Before its zero is found** a move is NOT_HOMED and a push runs at most `.unhomedVolts` (0: nothing
  moves it). **Homing with no motion at boot** — `restsAtStop`, `absoluteSensor` (the reading's turn
  chosen nearest the middle of the travel; a reversed sensor's mounting applied), a home switch already
  pressed on a FRESH read — and by a homing step (`homeBySwitch`: until the switch reads pressed on a fresh
  read; `homeByStall`: until its own signature trips — and one that never trips says what the push drew
  when stopped (no faster than its signature's speed; never the behaviour before the push, never a swing
  it brakes) against what its signature needs: `(it never stalled against its stop: it drew at most 0.62
  A when stopped, its homing signature needs 2.00 A)` — or, if it never got that slow, `it never slowed
  to its homing signature's speed -- did it reach its stop?`). A home switch whose read fails is never read as
  pressed (`Slot::homeSwitchPressed()`, the read-fault rule's second half). **Every (re-)home logs where its
  zero went** — a Warn when a re-home moves it past the "reached" tolerance (homing into a wedged piece sets
  a wrong zero; now it says so). `home()` on a mechanism that rests at its stop keeps its BOOT zero (it
  never re-zeroes where it stands). **The limits:** a COMPUTED target past one is clamped and logged ONCE;
  a push toward one is the driver's own until it could no longer stop before the limit at the move's
  acceleration, then taken over for the rest of that push — the move loop brings it to the limit and a
  fresh hold HOLDS it there, never pushing toward the limit harder than the push (one soft stop per push;
  turned away, it is the driver's again at once); no hold target is ever past a limit. A goal finds this
  robot's mechanism only in the goal's own units and, for a named preset, only at the same preset value
  (`PositionGoal::units`, set by the `Arm` and `Lift` words; a goal built by hand without it finds
  nothing).
- **The SPEED behaviour** (`mechanism::SpeedSlot`, a `Flywheel`; `MechanismService::speed(i)`,
  `find(flywheel)`, `find(flywheel["shoot"])` — a named speed only at this flywheel's own value):
  `spinUp(rpm, timeLimit)` — feedforward plus P, with an integral that acts only within 10 % of the target
  (frozen outside it, clamped to 3 V); "at speed" is SUCCEEDED and it KEEPS
  spinning; out of time it is TIMED_OUT and still spins. A computed speed past the free speed is clamped
  and logged once. `stop()`, `spinning()`, `rpm()` (the angle sensor's when one is wired), `targetRpm()`,
  `atSpeed()`.
- **The stall watch on EVERY behaviour** of a position and speed mechanism: the move, the hold after
  it, a push, a homing push by switch, a spin before and after "at speed" (a homing push by stall keeps
  its own signature). A trip is STALLED, `MECHANISM_STALLED` once, the declared safe state, the claim
  released — and, when it ends a behaviour AFTER its step, the command number bumped (how the driver
  controls, the core's fourth part, will ignore a held button until it is released).
- **The library's own numbers, named** (HA-182 — invented, provisional): `mechanism::kHoldIntegralLimitVolts`
  (3 V), `kSpeedIntegralWindow` (0.10), `kSpeedIntegralLimitVolts` (3 V) — LIBRARY constants for every
  mechanism, not `.tuning` fields and not on the boot line; `positionStartingValues(spec)` and
  `speedStartingValues(spec)` (what a spec runs with no `.tuning`); and `mechanism::startingValuesLine()`, ONE
  boot line per position or speed mechanism naming the numbers its loop runs with and which of them are
  starting values — `arm: starting values kP 0.250 kI 0.300 kD 0.0040 kV 0.0500, cruise 180.0 deg/s in
  0.25 s, reached within 1.50 deg for 0.10 s -- PROVISIONAL (HA-182); .tuning overrides` (a number stated by
  `.tuning` is marked "(tuned)"; the ramp by its TIME, so a stated speed alone keeps an unmarked 0.25 s
  ramp; a flywheel's `kV` and `kP` per output rad/s, its `kI` per output rad). The programs print it from
  the core's fifth part on.
- **The host test plant** (`sim/mechanism_plant.hpp`, test-only): an angle sensor MOUNTED on the joint
  (`RotationMount` — absolute within one turn at power-up, HA-186's belief, either way round), a limit
  switch (`SwitchMount`, at a chosen pressed level), and a stop BELOW the joint (`setWedgeDeg` — a piece
  wedged under the arm).

**CHANGED:**

- **`MechanismService::bind()` takes an `Arm`, a `Lift` and a `Flywheel`** — part 2 refused them (a
  precondition naming this part). It now checks their devices instead: a built position or speed
  mechanism carries its angle sensor exactly when it declares one, and a position one its home switch
  exactly when it homes by switch (a refused sensor or switch makes the mechanism ABSENT at boot).
- **A NEW build refusal, the 70th of `spec.hpp`'s:** absolute homing over a travel of a whole turn of the
  sensor or more (`position_absolute_homing_needs_a_travel_shorter_than_one_turn_of_its_angle_sensor`) — a
  reading within one turn cannot tell the turns apart (a spool lift's sensor turns several times over its
  travel). A robot file with such a mechanism stops compiling, by that name; none in this tree has one.
  The robot panel's validator mirrors the list by sentence (G3a).
- **A measured-table `Lift`'s STARTING speed and ramp** (`Lift::spec().position.maxVelocity` and
  `maxAcceleration`, so `startingValuesLine()`'s cruise) now come from the table's STEEPEST segment, not
  its first — the same or faster: 2.5 times for a table whose last segment is 2.5 times as steep as its
  first. The core's first part scaled them by the first segment; the move plan turns the speed back into
  degrees by the steepest one, so a starting move is 75 % of the output's free speed everywhere on the
  travel and never more. A stated `.tuning` speed or ramp is unchanged; a spool's or a linkage's starting
  values are unchanged.

**TESTED** (host): 144 moves over a deliberately mismatched grid with the starting
values, never tuned: 143 SUCCEEDED, every SUCCEEDED true at its verdict, the one miss a plant slower than
its own cartridge (TIMED_OUT); a profile the plant cannot follow TIMES OUT; the hold under a piece's load
against a control nobody ticks; the hold's integral (3.20° of steady error without it); a blocked move
STALLED, and one whose signature is too slow TIMED_OUT and holding where it stopped at 0.37 A; both limits'
soft stop on a slow, a medium and a fast arm and on a lift, pushed at 4 to 12 V with a plant up to twice as
heavy (one soft stop per push, never pushing harder than the button, held there for 20 s; a push ended by a
release, a move or a STALLED leaves none behind); "reached" at a
table's end and a linkage's top, and only after an unbroken settle; a linkage's and a table's plan within the
motor's speed; a tuned kS and kA, a lift's gravity term, both integral clamps and the speed integral's freeze;
every homing moment, a dead home switch, a wedged piece, a homing signature its push cannot reach (its line
counting only the push's own current when stopped — never a pinned jog before it, never a swing it brakes); an
absolute sensor mounted either way, wrapped through 0, above and below the middle of its travel; 27
flywheels, the integral window's ablation, a reverse spin, a jam before and after "at speed". 107 mutations of
these rules, the two reviews' own among them, were each run on the final code, on a fresh build of every file
they reach: 104 turned a test red, and the other three change nothing the code can do (each shown equivalent).

### Added since 2.4 (2026-10-15; built 2026-10-08–15) — the standard Calibrate's seams: the brain's screen as a seam, the painter that draws on it, and the front check

> Written when it was built (chunk CAL1, Part A1 of four: the seams; the standard Calibrate program is
> its Parts A2a and A2b, and robot two moves onto it in Part B). **No version bump — every change is
> ADDITIVE:** five new headers, and no existing header changed; `hal::IBrainScreen` is a new device
> seam beside `hal::ILineDisplay`, not one of the frozen F4 interfaces, and `calibration/` is in no row
> of the [Freeze Register](roadmap.md#freeze-register). **No program uses any of it yet**, so every
> program paints, prints and saves exactly as before. **Host only — nothing here has run on a robot.**

**Why:** the team lead asked on 2026-10-06 for Calibrate to ship WITH the library, so every team —
and the VS Code panel, which reads the record Calibrate saves — gets the same, correct Calibrate.
Today's "shulib Calibrate" is robot two's program and calls PROS's screen inline, so the host suite
can run only the pure pieces it calls (`viewFor`, `TouchGate`, the calibrator). These seams let the
library's Calibrate draw through an interface the host suite records, and its front check be a
library function.

**ADDED:**

- **`hal::IBrainScreen`** (`hal/brain_screen.hpp`) — the V5 brain's own touch screen as a seam:
  `fillRect`, `outlineRect`, `eraseRect`, `eraseAll`, `print(size, x, y, pen, eraser, text, cols,
  padTo)`, `touch()` (a `hal::ScreenTouch`: an error flag, the release count and the last place) and
  `prints()`. Beside it, **`hal::screenSafeLine()`**, the screen's sanitizer for the library: `'%'` and
  control bytes become `?`, a UTF-8 en or em dash becomes `-`, any other non-ASCII character becomes ONE
  `?`, and the text is cut at `cols`, then padded to `padTo`, never past the cut. Every `print()` passes
  its text through it, so no caller can hand VEXos a format character. Its rules are, byte for byte,
  those of the robot programs' own `printText()` (`src/common/screen_text.hpp`), which keeps its own
  copy until Part B makes it call this one; a host test feeds both copies the same 5,008 texts at every
  size and five pad widths (75,120 comparisons) and requires the same bytes. (Bench Tests keeps its own
  sanitizer at its 54-column width.)
- **`hal::pros::ProsBrainScreen`** (`hal/pros/brain_screen.hpp`) — the seam over PROS's screen API, in
  the order "shulib Calibrate" makes the calls today: the pen or the eraser set, then the draw; a print
  sets the eraser, then the pen, then prints the sanitized line through a literal `"%s"`; a touch read
  PROS answers with `E_TOUCH_ERROR` (every field −1) is an error, never a tap. It counts its prints,
  because PROS 4.2.2's print is believed to leak its buffer on every call (read from the vendored
  `libpros.a` at R3d Part 2F, never measured — like the belief that PROS re-reads printed text as a
  format string, which the sanitizer guards against; neither is in the assumptions register yet).
- **`hal::fake::FakeBrainScreen`** (`hal/fake/fake_brain_screen.hpp`) — the host's screen: it records
  every call as one line, in exactly the words the host PROS shim records for the same draw through
  `ProsBrainScreen` (a host test holds the two equal, call for call), sanitizes as the adapter does,
  counts prints, and plays back the touch as PROS reports it: a press moves the place, a release the
  release count, and an error read is PROS's — every field −1, never the last tap (a second host test
  holds its touch reads equal to the adapter's, read for read, so a reader that took its baseline
  from an error read would see the same phantom tap here as on the robot).
- **`calibration::CalibratePainter`** and the program's words, **`calibration::screen_text`**
  (`calibration/calibrate_screen.hpp`) — the palette and the painter "shulib Calibrate" uses today,
  re-expressed through the seam with the same rules: a change of screen, or of a line's place or size,
  repaints everything; a colour change is painted at once; a live number is repainted at most every
  `kLiveRepaintSeconds` (0.2 s — PROS is believed to leak each print); an in-place change is padded
  over the old text, never erased; a button that vanished is erased. Also the program's own screens
  (starting, cannot start, not driver control, the competition-state screen) and their words: every
  word the library itself paints is checked at compile time against its line's budget
  (`screen_text::kAll`) — the competition-state screen's names for its three states and their reasons
  among them (`kStateName*`, `kStateWhy*`; its name for `competition_initialize()` is
  `FIELD CONNECTED`, because the `FIELD CONTROL CONNECTED` robot two's other programs paint is five
  characters past the LARGE budget). Text a caller passes — a refusal's reason and fix, the status
  bits, the words a program hands the state screen — is cut at its budget by the sanitizer, and a
  screen that cut or dropped any of it says so on its last line: the full text is on the serial log.
  The header's build time is cut at `kBuildTimeCols` (14 characters, `BUILD hh:mm:ss`) and its word
  at `kHeaderCols` (24), the widths the header's layout is checked for. Two kinds of cut are NOT
  said on screen, because they are clamps that keep the layout whole: the build time's, and
  `render()`'s of a view's header word, lines and button labels (the views `viewFor()` builds are
  checked to fit; `paintCannot()` does say a header word it cut). The words that named robot
  two are general here (no slot numbers, no "Bench Tests": a team's project may have neither).
- **`calibration::compareFronts`** (`calibration/front_check.hpp`) — did a new calibration put the
  robot's FRONT at the other end? A record carries each drive port's side and sign relative to the
  push, so a push led by the other end gives every port the other side AND the other sign. Port by
  port over the ports both descriptions list: same side and sign agree, other side and other sign are
  flipped, one without the other is a re-mounted motor or a moved cable. `FrontVerdict::Flipped` only
  when the flipped ports are a strict majority, `Same` when the agreeing ones are, else `Undetermined`
  (a tie included) — never a guess. **What it cannot tell:** a robot whose LEFT and RIGHT drive cables
  were swapped between the sides gives the same record as one pushed from the other end, so FLIPPED
  says what the records say, never what the operator did. The earlier description may be the card's
  record or, with no usable record on the card, the drive ports a team typed (the team lead's decision
  of 2026-10-08); the function needs neither to be a valid description, and a side whose count runs
  past 21 decides nothing.
- **The host PROS shim** (test infrastructure, never on a robot): a `pros/screen.h` that records every
  draw (each of PROS's five text formats apart) and scripts the touch in PROS's two steps, press and
  release; the competition reads of `pros/misc.h`; three C-API motor reads of
  `pros/motors.h` (position, encoder units, gearing) that write nothing and count their calls; an
  advance hook in `pros/rtos.hpp`, called at every wait (`delay()`, and a `Task::delay_until()` that
  blocks) so a host test can step the host test plant with a program's own waits — it changes nothing
  while none is installed; and a host `main.h` that declares PROS's five entry points.

**What you must do:** nothing. The standard Calibrate that uses these comes in the next parts.

### Added and changed since 2.4 (2026-10-15; built 2026-10-08–15) — the mechanism core, part 2: one service runs power and pneumatic mechanisms; the run guard's deadline now cuts a claimed mechanism

> Written when it was built (chunk MECH-A2, the second of the mechanism core's six parts). **No
> version bump — every change is ADDITIVE in its signatures:** three new headers, three APPENDED values
> of `manipulation::MechanismOutcome` (its own append-only rule), one appended field of
> `sequence::RunGuardReport` (the run guard is row F12 of the
> [Freeze Register](roadmap.md#freeze-register), NOT FROZEN). **One BEHAVIOUR CHANGE**, below. In this
> part the service runs POWER and PNEUMATIC mechanisms only — the position and speed behaviours (an
> `Arm`, a `Lift`, a `Flywheel`) are the core's next part, and the auton steps and driver controls that
> call the service come after that; until then a program calls the service's slots directly. Until
> that next part, `bind()` REFUSES a list that holds an `Arm`, a `Lift` or a `Flywheel` (a precondition
> naming the part that brings them) — it never skips one silently *(part 3, above, lifted it)*. **Host
> only — nothing here has run on a robot.**

**ADDED:**

- **Done-conditions** (`mechanism/until.hpp`) — `mechanism::Until`, the words `seesPiece`,
  `stopsSeeingPiece` and `stalls` (from `using namespace shulib::mechanism::words;`), `seesPiece(grip)` /
  `stopsSeeingPiece(grip)` for ANOTHER mechanism's sensor (one that declares none FAILS THE BUILD, named
  `that_mechanism_has_no_piece_sensor`), `Until::time(...)`, `seesPiece.orAfter(2.5_s)` ("or after
  2.5 s, whichever comes first" — SUCCEEDED by time), and `|` (either ends the step; two times keep the
  earlier; two DIFFERENT mechanisms' sensors in one condition are refused at the step — the step's own
  sensor beside a name, `seesPiece | stopsSeeingPiece(intake)`, is one sensor when the step runs on
  the intake itself). A declaration written without `.name` is on no robot's list: `seesPiece(m)` on
  one fails the build (`a_mechanism_has_no_name`), and a hand-made `Until` naming one is refused at the
  step. A condition never replaces a step's time limit — every step still states one.
- **The mechanism service** (`mechanism/service.hpp`) — `mechanism::MechanismService`: one object, ticked
  once per control tick, that runs every mechanism's behaviour, so a mechanism step goes on while the robot
  drives. It is a `motion::ITickPacer` a program puts in the chassis's pacer chain (Chassis → `RunGuard` →
  `MechanismService` → the world), and the driver loop's stated place is `tickFromDriverLoop()` — exactly
  ONE tick per control tick either way. `bind()` makes one SLOT per mechanism of the robot's list over the
  devices `robot::MechanismParts` built (the same devices the guard's span lists); a slot is the mechanism's
  registered claimant, so the guard reaches every running behaviour. A `PowerSlot` runs at a voltage until
  its conditions or its time limit (`run()`), or as the driver's hold-to-run (`drive()` / `stop()`); a
  `PneumaticSlot` goes to a named position, every line at its level at once, and gives its verdict after the
  robot file's move time (`set()`). The most recent command wins; a mechanism ANOTHER owner has claimed is
  never commanded (the step says REFUSED). The mode changes: `enterDriverControl()`, `enterAutonomous()` and
  `leaveDriverControl()` — a claim left by an operation whose task PROS deleted is released WITHOUT calling
  into it, counted and said (its motor then made safe, like any other). None of the three puts a pneumatic in
  its safe state: `leaveDriverControl()` (a disable) and `enterAutonomous()` send it nothing, and
  `enterDriverControl()` only its START position, on the first entry (T4's ruling: the claw starts closed) —
  so through a disable the service leaves a pneumatic at its level ([HA-157](hardware-assumptions.md)). The
  run guard is another matter: `run()`'s cancel-all at the end of every guarded auton still puts every LISTED
  mechanism, a pneumatic included, in its safe state, as before, and its new cut (below) puts a pneumatic
  whose step has not given its verdict yet in its safe position too. A service destroyed while a slot holds
  its claim ends that behaviour first (CANCELLED, the safe state, the claim released), so no claimant is
  left pointing into it — its devices, clock, fault latch and sink must outlive it. A piece sensor is read
  as POSITIVE evidence: a reading that may be a held value (a distance near with no confidence, or ANY
  sample taken while the sensor's read-fault count moved — a pulled cable reads exactly like "no object")
  satisfies neither `seesPiece` nor `stopsSeeingPiece`. Every step ends with ONE `[MECH]` log line
  ([Chapter 11](guide/11-reading-the-diagnostics.md) reads it field by field).
- **Three verdicts**, appended to `manipulation::MechanismOutcome` — `Absent = 6` (`ABSENT`: the
  mechanism was not built at boot), `NotHomed = 7` (`NOT_HOMED`: a position move before the mechanism
  found its zero — produced from the core's next part on) and `Refused = 8` (`REFUSED`: another owner
  holds the mechanism's claim). Each is a step that moved NOTHING. A `switch` over the enum that lists
  every value needs three more `case`s (none in the library or its programs did).
- **`sequence::RunGuardReport::mechanismCuts`** — how many claimed mechanisms the guard's latch
  cancelled after the deadline or an abandon (the change below).
- **The host test plant's mechanisms** (`sim/mechanism_plant.hpp`, test-only — the library cannot
  include it) — `sim::ArmPlant` (one joint driven by N fake motors: a gravity load, hard stops, an
  obstacle, a jam, the brake modes at 0 V; a lift and a roller are the same joint) and
  `sim::FlywheelPlant`. Every constant is invented: [HA-183](hardware-assumptions.md).

**BEHAVIOUR CHANGE (an auton run by `sequence::RunGuard`) — the guard's deadline and an abandon now CUT a
claimed mechanism** (the team lead's ruling, 2026-10-08). A mechanism step waits in `Chassis::waitUntil`, a
wait the guard cannot cut, so before this an intake step still running when the end action was due ran on to
its OWN time limit: measured 1.01 s late on a 2 s step and 2.02 s late on a 5 s one — the hard stop then
fired during scoring and the end action (the park) failed; after a disable it ran up to 2.52 s on, which the
competition program's driver control waits out. Now, in the two places the guard already cuts the drive's
motion, it also calls `cancel()` on every listed mechanism whose claim has a REGISTERED claimant (a
mechanism-service slot, or an F1 operation started with one): the step ends CANCELLED and its wait sees it
at the next pace — one pace (0.01 s) after the deadline; after a disable, two (0.02 s: the guard reads the
request at its next pace) — nothing commanded after the cut. The DEADLINE cut never runs inside the end
action (the end action's own steps are exempt from the deadline, as its motions are); an ABANDON (a disable)
cuts claimed mechanisms everywhere, the end action included. A mechanism nobody has claimed is not re-made
safe every pace (a repeated Hold re-captures where it holds and lets a loaded shaft creep); `run()`'s
cancel-all still makes every listed mechanism safe, as before. The run's first cut is logged once (`SEQ`:
`mechanism 'intake' cut at the deadline -- ...`) and every cut is counted in `mechanismCuts`. **What it
changes for you:** an F1 operation (`RunUntilConfirmed`, `ActuateAndConfirm`) with a registered claimant,
still running at the deadline, now ends CANCELLED there instead of running to its own timeout. **What you
must do:** nothing, unless an auton RELIED on an operation outliving the deadline — the library's whole
suite passed with the cut, and none did. An operation claimed ANONYMOUSLY (no claimant) is not reached until
`run()`'s cancel-all, as before.

**TESTED** (every case against the REAL guard, the REAL `MechanismParts` over the fake provider, and the
host test plant): exactly one service tick per control tick through all three pacers a driver loop can be
handed; the guard's reach leaves nothing that re-commands on the next tick; every done-condition alone and
OR-ed; a declared piece sensor MISSING this boot runs the step's whole time with the motor running and ends
UNCONFIRMED, counted done — never a false SUCCEEDED; a piece sensor whose reads FAIL, through the REAL PROS
distance and optical adapters over the PROS shim, never read as an answer; the deadline cuts a 2 s and a 5 s
step one pace after it, the disable two paces after it; and a mechanism left at 12 V by a driver task PROS
deleted runs 0 of 269 auton paces with `enterAutonomous()` (all 269 without it). Every rule's mutation was
run and turned its test red. One hardware belief is new in the [register](hardware-assumptions.md): HA-183
(the host test plant's mechanism models stand in for real ones).

### Fixed and added since 2.4 (2026-10-15; built 2026-10-08) — the serial log keeps every line whole when two tasks print: one `write()` per line

> Written when it was built (chunk SER1, the team lead's ruling "one-call writes", 2026-10-08). **No
> version bump — a fix inside an adapter, and additions:** `hal::pros::ProsCharSink` is an adapter, not a
> frozen contract (`hal::ICharSink` is deliberately outside the [Freeze Register](roadmap.md#freeze-register)'s
> F4), its constructor taking a `FILE*` keeps its signature, and every other name below is new. **Host and
> build gated only — nothing here has run on a robot**; the fix is reasoned from the vendored C library and
> can be settled only at the bench (A4 register HA-189, whose dedicated two-task test is still owed). *(2026-10-18:
> the team lead accepted the bench worksheet's practice-match serial check — the competition program's own lines,
> read around every mode change — as HA-189's robot test in place of that dedicated test; HA-189 stays unsettled
> until it runs.)*

**FIXED — two tasks printing at once could mix or lose each other's serial lines** (A4 register HA-189,
reasoned from the vendored `libc.a`/`libpros.a`). Every task on the V5 shares ONE stdout buffer, and the
C library's locks on it are empty stubs, so two tasks inside `printf`/`fwrite` at once (the competition
program's auton task and PROS's phase tasks, around every mode change) could interleave or drop bytes —
and `ProsCharSink`'s old banner said the opposite. Now every line shulib sends is ONE direct `write()` on
the serial's descriptor, which PROS's own serial lock keeps whole: `ProsCharSink::write()` no longer
goes through stdout's buffer at all, and every line the programs print at boot goes through the new
functions below. One case nothing on our side can cover is new in the register: a task PROS deletes in
the middle of that one write is believed to leave PROS's serial lock held, so every later serial line of
that boot would wait for ever (HA-190, reasoned, not seen — the guide's
[chapter 17](guide/17-the-competition-program.md) says when that can happen).

**Added** (`include/shulib/hal/pros/char_sink.hpp`):
- `hal::pros::serialLine(format, ...)` — one printf-style line, formatted on the caller's stack, given
  exactly one trailing newline, sent with one `write()`. At most `kSerialLineBytes` (256) bytes with its
  newline; a longer result is CUT, visibly (`kSerialCutMarker`, `...[cut]`).
- `hal::pros::serialWrapped(tag, text)` and `serialWrapped(tag, {part, part, ...})` — a text of any
  length as `[tag] ` lines of at most 256 bytes, one `write()` each, broken at spaces and never cut;
  later pieces begin `[tag] ... `. `kSerialTagBytes` (16) caps the tag.
- `hal::pros::writeWhole(fd, bytes, writeFn)` and `hal::pros::PosixWrite` — the write loop underneath:
  the remainder after a short write, a stop (never a spin) on 0 or −1, and no call offered more than
  `hal::pros::kSerialWriteMaxBytes` (512) bytes — a longer text goes in pieces of that size, so no shulib
  call can reach PROS's ~2 KB limit on one write (A4 register HA-191). Every line shulib prints (at most
  384 bytes) is still one call.
- `ProsCharSink()` — a default constructor for the robot's serial (fd 1) that names no `FILE` at all.
  `hal::pros::ProsDeviceProvider` uses it, unchanged in its source.

**BEHAVIOUR CHANGE (`ProsCharSink(std::FILE*)`)** — it writes that FILE's DESCRIPTOR directly, flushes
the FILE once when it is built (so earlier text goes first), and now also refuses a FILE with no
descriptor (a contract breach, loud on the host; a null FILE it refused already). Anything written into
the FILE through stdio AFTER the sink exists may come out after the sink's lines. **What you must do:**
build the sink before anything else writes that FILE, and stop writing it through stdio (a test that
handed it `tmpfile()` and read the file back still works).

**BEHAVIOUR CHANGE (`ProsCharSink::write()`)** — a text longer than 512 bytes now goes out in 512-byte
pieces, one `write()` each, in order; before, it went in one write of any length, which on the brain is
believed to wait for ever past about 2 KB (HA-191). Nothing shulib prints is that long (a diagnostic line
is at most 384 bytes); a team's own sink text that is longer can now have another task's line land
between two pieces — keep a line that must stay whole under 512 bytes.

**BEHAVIOUR CHANGE (the programs' serial output)** — the bytes of every boot line are unchanged, except
a line longer than 255 characters: it now arrives as `[TAG] ` pieces, the later ones `[TAG] ... `
("shulib Drive"'s run-status sentence on every boot; a long refusal reason; a claw's ABSENT line).

**FIXED (the API reference)** — the documentation tool had published four entries under names it misread,
and its coverage gate counted them as documented: `MenuButton::label` and `MenuLine::text`
(`competition/auton_menu.hpp`) and `Wrapped::line` (`calibration/calibrate_panel.hpp`) appeared as `1]`,
and `Handler::fn` (`runner/command_registry.hpp`), a pointer-to-function field, as a function named after
its return type. Their headings and link anchors changed to the real names (`#menubutton-label`,
`#menuline-text`, `#wrapped-line`, `#handler-fn`); the declarations are unchanged. A conversion operator
(`operator T()`) is now named `operator T` — it was named after the type it converts to. The gate now FAILS
any name it cannot read, or one that is only a keyword.

**What you must do** (a team's own code that prints): print a line with
`shulib::hal::pros::serialLine("[TAG] ...", ...)`, a long text with `serialWrapped("TAG", text)` — never
`printf`, `puts`, `putchar` or a `std::cout << ...` chain from code that runs while another task prints,
and never more than about 2 KB in one `write()` of your own (HA-191; shulib's calls stay under 512). The
project's src build gate now fails a build whose `src/` code names stdout or stderr, or calls any
printer that writes them — every one the vendored C library defines (the list is checked against the
library's own symbol table on every run) and `std::cout`, `std::wcout` and their family
(`tools/src_build_gate.py`, step 9), so code in `src/team/` follows the rule or does not build.
[Chapter 11](guide/11-reading-the-diagnostics.md) has the rule and its why.

### Added since 2.4 (2026-10-08, later; its review round 2026-10-15) — the mechanism core, part 1: any mechanism declared once and wired separately, checked at build; its route ids; every role built at boot

> Written when it was built (chunk MECH-A1, the first of the mechanism core's six parts). **No
> version bump — every change is ADDITIVE:** eight new headers; appended fields, an appended enum
> value and new members on surfaces the [Freeze Register](roadmap.md#freeze-register) does not freeze
> (`config::RobotConfig`, `robot::Robot` and `hal::IDeviceProvider` are in no row of it; the mechanism
> seam is row F11's, still NOT FROZEN); the F4 sensor seams untouched (the read-fault count is a
> SIBLING interface, as `IDigitalIn` was). Every config that was valid stays valid and builds the same robot: the new
> config fields are all 0 by default, which is the form every config had. **This part is the
> vocabulary, the build checks and the devices — nothing RUNS a mechanism through it yet** (the
> service, the auton steps and the driver controls are the core's next parts, and robot two's table
> moves to these words in a later one). **Host only — nothing here has run on a robot.**

**ADDED:**

- **The five declarations** (`mechanism/kinds.hpp`) — `Arm` (an angle in degrees, held), `Lift` (a
  height in inches — a `spool`, a `linkage` or a `measured` table), `Flywheel` (an output rpm, "at
  speed"), `Power` (run at a voltage) and `Pneumatic` (two or three named positions, each the line
  levels the team states). A declaration says what a mechanism IS — never a port — and is the HANDLE an
  auton names (`arm["score"]`, `arm.to(120)`, `lift.to(18_in)`, `flywheel["shoot"]`, `claw["closed"]`).
  The words come from one `using namespace shulib::mechanism::words;`: `gears(12, 60)` / `direct` (the
  gearing as TOOTH COUNTS, motor's gear first — a bare number does not compile), `Forward` /
  `Reversed`, `rotationSensor()`, `distanceSensor(2_in)`, `opticalSensor(0.5)`, `limitSwitch()`,
  `cosine(...)` / `constantGravity(...)` / `noGravity`, `restsAtStop` / `homeBySwitch` / `homeByStall`
  / `absoluteSensor` / `neverHomed`, `within(...)`, `High` / `Low`, `grabs` / `releases`, `Red` /
  `Green` / `Blue`, and one `Hold` / `Brake` / `Coast` that reads as either a mechanism's safe state or
  a drive brake mode. Each declaration records the file and line it was written on.
- **One mechanism as data, and THE build check** (`mechanism/spec.hpp`) — `MechanismSpec` and
  `specRefusal()`: 69 reasons, one property each, each with a sentence (`refusalText`) and a function
  named for it in `mechanism::refused`, which is what the compiler prints. Among them: a jam signature
  (`.stall`) is REQUIRED on an `Arm`, a `Lift` and a `Flywheel`, and every signature written must be
  able to trip — amps above 0 and below the motor's 2.5 A current limit, `belowMotorRpm` above 0,
  `lasting` above 0; a homing push must point at its home end; a lift needs its `.shape`; a homing
  push and a gravity hold within the motor's 12 V either way, `.unhomedVolts` from 0 to 12; a negative
  settle time, at-speed lasting or `.confirmWithin` each its own reason; a range check refuses a value
  that is not a number.
- **The wiring** (`mechanism/wiring.hpp`) — `mechanisms(wire(arm, motors(1)), wire(claw, lines('A')),
  ...)`: a `MechanismTable`, checked at build — every port in range and of the right kind (a NUMBER for
  a smart port, a LETTER for a 3-wire one), at most four motors and two lines, each sensor role written
  once, a port or a line used twice refused BY NAME (`smart_port_4_is_used_twice`), the declaration's
  own check, every pneumatic position stating one level per wired line; `entryProblem()` (the
  per-mechanism wiring rules as data — `mechanisms()` holds every entry to them, one built by hand
  included); `wiringProblem()` (the same rules at run time, plus a drive's ports and the IMU's);
  `toMechanismConfig()` (THE projection to `config::MechanismConfig` — a Reversed motor role is a
  negative port); `declaredMotorPorts()`; `roleLine()` (one boot line per role).
- **Route ids** (`mechanism/route_ids.hpp`) — every mechanism's actions as the ids a route marker
  names, DERIVED from the list, never typed: an arm's or a lift's `<name>_<preset>` and
  `<name>_home`; a flywheel's `<name>_<preset>` and `<name>_stop`; a power mechanism's `<name>_run`,
  `<name>_until_piece`, `<name>_until_gone` (with a piece sensor) and `<name>_stop`; a pneumatic's
  `<name>_<position>`. An id past 23 characters, two alike, a built-in, or more than 32 fail the build
  at the list. (Running one is a later part.)
- **The devices** — `hal::IReadFaults` (`hal/read_faults.hpp`): how many reads a sensor adapter has
  screened; `ProsDistance`, `ProsOptical` and `ProsDigitalIn` implement it. `robot::MechanismParts`
  (`robot/mechanism_parts.hpp`): every role of every mechanism built over a device provider — the
  actuators, then an angle sensor (refused: the mechanism is ABSENT), a home switch, a piece sensor
  (refused: it runs WITHOUT one) — with one `mechanism::RoleFact` per role and one `mechanism::Devices`
  per mechanism (`mechanism/devices.hpp`, the read-fault counts included). `robot::Robot` owns one and
  hands it out (`mechanismParts()`); its other mechanism accessors are unchanged.
- **`hal::MotorMechanism::meanPosition()`**, and a PER-LINE form of `hal::PneumaticMechanism` (one
  declared safe level per line, `setLevels()`, `commandedLevel(i)`, `perLine()`, `safeLevel(i)`) — a
  three-position mechanism on two lines has a safe POSITION its `applySafeState()` applies. The
  single-level form is unchanged.
- **`config::MechanismConfig`** appends the sensor roles (`rotationPort`, `distancePort`,
  `opticalPort`, `switchLines[2]`) and per-line levels (`pneumaticSafeByLine`, `pneumaticStartByLine`);
  `config::PortUse::MechanismSensor` is appended.

**CHANGED (for an implementer, or a config that uses the new fields only):**

- **`hal::IDeviceProvider` has five new pure virtuals** — `distance(port)`, `optical(port)` (a piece
  sensor; refused on a port holding anything else), `digitalIn(line)` (a limit switch; refused only
  outside 1..8 — one 3-wire line is ONE direction: an input on a line made as an output, or the
  reverse, is a precondition), and the read-fault pair `readFaults(port)` / `lineReadFaults(line)`.
  Code that only CALLS a provider is unaffected; a class that IMPLEMENTS one must add them (the two in
  the library have). **Migration:** answer each — make the sensor or refuse it naming the port, and
  return the count of what you made (nullptr where you made nothing, or your sensors never screen a
  read).
- **`config::RobotConfig::valid()`** also claims each mechanism's sensor ports (a calibration record
  that drives one is refused naming both: "port 9 is claimed by the LEFT drive and by a mechanism
  sensor") and its limit-switch lines (against the pneumatic lines), and refuses two piece sensors on
  one mechanism (a distance, an optical or a limit switch: a piece sensor is one device) and a
  half-stated per-line level set. A config without the new fields is unaffected. **What you must do:**
  nothing.

**TESTED:** every refusal at run time, one property per case (both sides of every two-sided rule),
the check order pinned; every refusal a robot file can reach AT BUILD by a translation unit that must
fail to compile naming its reason (all but two "used twice" fallbacks no list can reach) — every place
that calls one a robot file can reach, and where two layers could both
refuse, the line GCC must cite (`test/compile_refusals/`, `ctest -R compile_refusal`, run by CI, which also fails when the
harness finds no tests or fewer than its translation units); the wiring mirror both ways (every
hand-built entry the build refuses, the run-time check refuses); robot two's claws written in the
new words project to EXACTLY the config today's boot builds, field by field. Two beliefs about the
hardware are new in the [register](hardware-assumptions.md): HA-184 (a limit switch reads 1 when
pressed; an unplugged line reads released) and HA-186 (a rotation sensor starts at its shaft's
absolute angle).

### Added and changed since 2.4 (2026-10-08) — an SD copy of a program's log; the frozen-reading rule's encoder check; one refused auton entry dropped, not the boot; a slow loop said once per episode

> Written when it was built (chunk T4 Part 3, the library pieces of both its steps, and its review's
> fixes of 2026-10-08). **No version bump — every change is ADDITIVE or a rule inside an unfrozen
> layer:** `diag::LogCopySink` is a new implementation of `hal::ITelemetrySink`, whose interface (F4) is
> untouched; `ProsDeviceProvider` is an adapter, not a frozen contract, and its two existing
> constructors are byte-for-byte unchanged; `teleop/`, `competition/` and `diag/`'s latch and loop
> monitor are in no row of the [Freeze Register](roadmap.md#freeze-register) (F9 freezes the
> `DebugRecord` wire, whose fault codes and numbers are unchanged); no existing declaration in any of
> them changed, and `AutonRegistry`'s new constructor argument defaults to what it did before. **Host
> only — nothing here has run on a robot.**

**BEHAVIOUR CHANGE (driver control) — the frozen-reading rule counts a side-mate's ENCODER, not its
speed report** (the team lead's ruling, 2026-10-01). `teleop::FrozenReadingDetector` — the rule the
driver loop's protections drop a dead drive motor by — now counts a side-mate as evidence that the
train turns only while that mate's POSITION moved within the last 5 reads; a mate whose encoder has
stopped is no evidence, however fast its lagging speed report still reads. Before, a mate counted
while any of its readings changed, so at a wall — every encoder stopped, the speed reports still
decaying — a live motor whose report settled first could be marked ABSENT (the known gap the entry
below names). Now a plain wall push marks nobody; when one motor's report settles well before its
mates', it costs at most one 1 s FIGHT cut instead — the fight monitor sees it lag (pinned at every
report-lag spread probed, under the 2.4 A limit; at 2.45 A and 2.5 A per motor the over-current cuts
come first). Nothing else changed: the 5 reads, the 250 ms latch, the majority-or-tie guard and a member's
OWN repeat (position AND velocity) are as before. **Still a known gap:** beside a drive gear that
slips (robot two's port 18) at a wall, the rule can still mark a live motor (A4 register HA-175).
**What you must do:** nothing.

**BEHAVIOUR CHANGE (diagnostics) — a slow loop is said once per EPISODE, not once per tick** (the
T4 Part 3 review, 2026-10-08). `diag::LoopMonitor` still raises `LOOP_OVERRUN` on EVERY tick at or past
its budget — `overrunCount()`, the latch's `faultCount()` and `raiseCount()`, and every record's fault
stamp count each one, as before — but only the FIRST tick of an overrun episode logs its `fault=`
line; the episode ends at the first tick back under the budget, or at `reset()`, and the next slow
tick is said again (its `n=` carries the ticks counted silently). The scheduler's `overrun
attribution:` line follows the same rule (once per episode, beside it). Why: a loop that stayed slow
logged an Error line every tick — 50 a second at a 20 ms tick — and the programs' SD copy of the log
(below) turned each into a card write and a flush INSIDE that already-late tick. The per-tick `dt` is
still on every record. A loop slow only now and then (a slow tick between healthy ones) is a new
episode each time, so it still logs each one, as before. **What you must do:** nothing — read `n=` (or
`overrunCount()`) for how many ticks a stretch lasted, not the number of lines.

**Added:**

- **`diag::FaultLatch::raiseRepeat(code, subsystem, detail)`** — raise a fault as the continuation of
  an episode whose opening `raise()` was already logged: latched, counted and tallied exactly as
  `raise()`, with no line — except when it is the first fault since construction or `clear()`, which
  is always logged (the root cause is never silent). **`diag::LoopMonitor::overrunEpisodes()`** — how
  many overrun episodes began (each said once).

- **`include/shulib/diag/log_copy_sink.hpp`** (new): `diag::LogCopySink(primary, copy)` — a
  decorator that hands every `log()` line and every `summarize()` block to the primary sink and then
  to the copy, and every per-tick record (`wantsRecord()`/`emit()`) to the primary ONLY: a copy of a
  program's events and summaries, never a line per tick (a record line every 10 ms would be a card
  write and a flush every 10 ms, A4 register HA-60). PROS-free, allocation-free; `primary()` and
  `copy()` read the two sinks back. The COPY has one writer at a time (the T4 Part 3 review): a line or
  summary that arrives while another task is mid-copy reaches the primary, is left off the copy — never
  waited for, so one task's card write never stalls another's loop — and is counted,
  `contendedLines()`; the next line or summary that does reach the copy is preceded there by ONE Warn
  line, `copy: <n> line(s) left off here while another task was writing it -- serial has them`
  (`LogCopySink::kGapTag`, `DIA`), so the gap is visible where it happened — once the writer that
  held the copy returns. A writer that never returns (a task PROS deletes mid-line, A4 register
  HA-161 (a), reasoned — e.g. at a disable, right as a driver program logs that its loop ended) keeps
  the guard: the copy ENDS for the rest of that boot, every later line counted and on the primary,
  with no gap line. That is the chosen failure — a released guard would hand the next writer, the
  driver loop included, whatever the deleted one left half-done on the card (unmeasured), where it
  could block; the header says why and what would remove it. Neither copied nor moved (it holds the
  guard).
- **`hal::pros::ProsDeviceProvider(serialCopyFile, copyMode = Truncate, copyMountRoot = "/usd/")`**
  — a new constructor: given a BARE file name, the provider keeps a copy of its serial log on the SD
  card (a `ProsBlockSink` in the mode asked — `Truncate`, one file per boot, or `Append`, kept across
  boots — under COMP-t's `hal::BlockCharSink`, a second `diag::TermSink` over the same clock, and a
  `diag::LogCopySink` over the serial sink and that one, which `telemetry()` then returns). A null
  name is the default provider exactly. A missing or full card leaves serial unchanged and is
  COUNTED, never thrown: `serialCopyOpen()`, `serialCopyPath()`, `serialCopyDroppedLines()` (every
  line the copy lost: the card's refusals plus the lines the one-writer guard kept off) and
  `serialCopyAppendOpen()` say what happened, for a program's one boot line. The copy starts when the
  provider is constructed.
- **`competition::OnRefusal` and `AutonRegistry(periods, onRefusal = OnRefusal::Throw)`** — what
  `add()` does with an entry `check()` refuses (the team lead's ruling of 2026-10-02: "drop just that
  auton"). `Throw`, the default, is unchanged: a `PreconditionError` carrying `check()`'s sentence.
  `DropEntry` registers NOTHING for that entry, keeps its sentence, and returns, so a registration
  function's later `add()` calls still register — one bad auton no longer costs every other one, or a
  program's boot. `refusedCount()` counts every drop; `refusal(i)` gives the first
  `competition::kMaxAutonRefusals` (8) sentences, each beginning `auton '<name>': `; `onRefusal()`
  reads the choice back. **`AutonMenu`** names each dropped entry on its status line, in red,
  `REFUSED auton '<name>': ...` (`kMenuRefusedPrefix`), one at a time for `kMenuRefusalTurnSeconds`
  (2 s) each, while the choice is open (LOCKED still wins once the auton starts) — a menu over a
  registry that dropped nothing is unchanged. **What you must do:** nothing; a program that wants the
  drop asks for it (shulib's competition program and auton tester do).

### Added since 2.4 (2026-10-06) — the drive's gains and the IMU's noise, measured by the robot: the library half

> Written when it was built (chunk R5a, Part A of four). **No version bump — every change is
> ADDITIVE:** five new headers, and one field appended to the host test plant's config; nothing
> frozen changed, and no existing header's behaviour moved (a default host test plant is
> bit-for-bit what it was — checked against a hash of eight default runs taken before the change).
> **Nothing below has run on a robot, and no program runs it yet** — it is proven on the host test
> plant and on synthetic data only. The team lead ruled on 2026-09-26 that the drive's
> feedforward gains are measured BY THE ROBOT, inside shulib Calibrate, and kept on the SD card —
> never typed or committed as constants. This is the measuring half; the card, the robot builder
> consuming the gains, Calibrate's new GAINS and REST steps and every program's gains line are
> R5a's later parts. Until they land every drive runs on the placeholder gains, exactly as before.

**ADDED:**

- **`calibration::FeedforwardFit`** (`calibration/feedforward_fit.hpp`) — kS, kV and kA of ONE
  drive side, fitted on the brain from that side's own run, one sample per tick. It fits the
  DISCRETE form of the feedforward law, v[k+1] = α·v[k] + β·u[k] + γ·sgn(v[k]), by incremental QR
  (never the normal equations), so no differentiated velocity is used anywhere. Every excluded
  sample is counted by its reason (`FitExclusion`: not a number, an off-nominal tick, a stop at
  0 V, the dead band, a member at the current guard); the result carries a `FitVerdict` (`Ok`,
  `TooFewSamples`, `NotExcited`, `SignMismatch`, `NonPhysical`, `Implausible`, `PoorFit`), the
  residual and the voltage range fitted over. A kA below the fit's resolution is reported as
  exactly 0, flagged. **What `PoorFit` does not catch is stated in the header and measured:** a
  quadratic drag never reaches it (it biases kS down and kV up instead — at a drag of 0.45 V at
  30 in/s, kS −9.5 % and kV +10.5 %), and neither does a filtered `velocity()`, a one-tick command
  latency, or noise on the speed reading (which biases kA low and kS high under an `Ok` verdict —
  at 1 in/s of noise, kA −46 % and kS +11 % on average); each bias is pinned by a test and recorded
  in the register (HA-171).
- **`calibration::DriveCharacterizer`** (`calibration/drive_characterizer.hpp`) — the GAINS step:
  a tick-driven routine that drives the robot through two voltage ramps and two held steps, forward
  and back, both sides at the same volts, fitting each side separately. Each segment ends on the
  FIRST of either side's travel reaching 36 in, the ramp reaching 10 V, or 8 s; it then stops at
  0 V + BRAKE until the robot is at rest (then Coast), measuring each side's stopping distance.
  It refuses — always to 0 V + COAST on the same tick, so a person can push the robot — on a curve
  past its guard (read from the ENCODERS, never the IMU, which only records), a fighting member,
  a missed tick while driving, a motor at 45 °C before a segment, a stop longer than 12 in, a robot
  that never rests, an IMU that never readies, a fit that is not `Ok`, or `abort()`; each refusal
  names its port or its sides. The report carries every segment's numbers (travel, stopping
  distance, battery, temperature, the encoder curve against the IMU's yaw, the largest heading
  step, the loop timing) — a segment a refusal ended keeps how far it got — and `lastSample()`
  exposes each tick's raw reading. `CharacterizerConfig::clearAheadAndBehind()` /
  `sidewaysExcursion(W)` give the lane it needs: 48 in ahead and behind, and 26.6 in to each side at
  a 12.5 in track width (the arc of a segment curving up to the guard plus the whole 12 in stop
  bound — the stop is not curve-guarded; the header states what both numbers assume). It does NOT
  refuse a segment that cannot move (something in the way): that segment ends on its ramp's top or
  its 8 s timeout, the run finishes on the other segments, and `warning()` names the segment and
  side that admitted no sample (`CharacterizerReport::emptySegmentWarning`); whether to refuse it
  instead is an open decision before Calibrate runs the step.
  **A measured kA is recorded, not applied** — the motion layer applies the cruise form
  `Feedforward::calculate(v)` only.
- **`calibration::RestCapture`** (`calibration/rest_capture.hpp`) — the REST step: the robot sits
  still and unpowered for 60 s while the IMU's drift (the slope of the unwrapped heading, with its
  σ under both white-heading-noise and random-walk models, and a structure ratio saying which the
  data fits), heading noise, yaw-rate noise, the per-tick heading step, the encoders' noise at rest
  and the loop timing are measured. It refuses a robot that MOVES or TURNS by its encoders — never
  by the IMU, whose apparent rotation at rest is the drift being measured — and records the
  rotation it admitted under its 3° bound, with the most that rotation can have moved the drift
  (`RestSummary::driftTurnBiasBound()`: a hand on the robot turns the IMU too). A side that gives
  no finite reading on a tick is left out of that tick and counted (`RestCapture::unreadSides()`),
  so one bad tick cannot spoil the capture's numbers. Also
  **`settleFloorReport()`**: how far each settle floor of a `MotionConfig` clears the measured noise
  (a warning under 2×) — it changes nothing. **The drift is a per-boot number and is never applied
  as a correction** (HA-173).
- **`calibration::DriveCharacterization`** (`calibration/drive_characterization.hpp`) — what the
  two steps measure, as fixed-width data (each side's gains and the facts behind them, the rest
  summary, the timing, provenance), with `combined()` (the two sides' mean — what R5a Part B will
  hand the motion layer) and `valid()` naming the offending field.
- **`diag::TickTimingStats`** (`diag/tick_timing_stats.hpp`) — a loop's dt statistics (count, mean,
  σ, min, max, ticks at or over the 15 ms budget, ticks over twice the period), a sibling of
  `LoopMonitor`, which is unchanged.
- **Host test plant:** `sim::DrivePlantConfig::wheelFfOverride` — per-wheel gains, so a host test
  can tell a drivetrain's LEFT from its RIGHT. Empty by default, appended last (every designated
  initializer still compiles), and a default plant is bit-identical to before.

Five new register entries, HA-170 to HA-174, in the [Hardware Assumptions Register](hardware-assumptions.md).

### Added since 2.4 (2026-10-04) — the calibration record's safe save: a verified backup, a load that falls back to it, and a report that measures the card

> Written when it landed (chunk SAFE-SAVE, Part A — the LIBRARY; "shulib Calibrate" switches to it in
> Part B1, and the boot's fallback and Teleop's words are Part B2). **No version bump — every change is
> ADDITIVE:** four new headers and one new constant; nothing frozen changed shape, and the calibration
> record's layout, version (1) and golden bytes are unchanged. **Nothing below has run on a robot**, and
> no program uses it yet.

**Why:** on the brain there is no rename, no delete and no read-write open (HA-166, read from the
vendored PROS and newlib libraries), and `fopen "wb"` reaches VEXos's create call, which is taken to empty
an existing file at the OPEN (reasoned from those libraries, not measured). So "shulib Calibrate"'s save —
open the record with `"wb"`, write, read back — is taken to erase the good record whenever it fails after
the open, and it then printed that the earlier record was "still on the card". A rename-based fix cannot
be built on this platform (it would pass every host test and fail on the robot).

**ADDED:**

- **`hal/file_store.hpp`** — `hal::IFileStore`: `cardPresent()`, `read(name, into)` → `StoreRead`
  (opened, count, the path used), `replace(name, bytes)` → `StoreReplace` (`ReplaceStep` `None` /
  `NoCard` / `Open` / `Write` / `Flush`, the path used). Deliberately no rename, remove, stat or
  read-write open. A sibling of `IBlockSink`/`IBlockSource`, not a change to them. Not `noexcept`: a
  name that is not bare throws `PreconditionError`; every refusal by the card is a returned value. And
  `hal::sameFileOnCard(a, b)`: whether FAT's name rule makes two names ONE file (it ignores letter case
  and trailing dots and spaces) — the rule the brain's card is taken to follow, not yet measured (HA-166
  (6)).
- **`hal/pros/file_store.hpp`** — `hal::pros::ProsFileStore`: builds a `ProsBlockSource` or
  `ProsBlockSink` per call, each closed before the call returns; no `fopen`, mode string or `/usd/` of
  its own. `ProsFileStore(root)` for host tests against a real directory.
- **`hal/fake/fake_file_store.hpp`** — `hal::fake::FakeFileStore`: named files in memory, with reported
  faults (open refused, short write, false flush, short read), silent faults (a cut, a flipped byte, a
  write that never landed, a read that returns a flipped byte), and a crash at any operation that leaves
  the file absent, empty, a prefix, all new, or torn — with the card then gone or silent — plus an
  operation log.
- **`calibration/record_store.hpp`** — `chooseEffective()` (the load rule: the record file if it
  decodes, else the backup, else none); `loadWithBackup<MaxBytes>()` and `safeReplace<MaxBytes>()`, the
  generic byte-level load and save (for any small file with a backup — up to 1024 bytes, stated stack);
  `acceptRecord()`, `saveRecordSafely()`, `loadRecord()` → `LoadedRecord` for the drivetrain record;
  `SaveReport` (`SaveResult` `Saved` / `NotSaved` / `Unconfirmed`, what each step did, and what both
  files hold when re-read at the end); `formatSaveReport(report, BootReadsBackup, emit)`, the serial
  words, every sentence derived from what the save re-read. The save: read both files (the record file
  twice when its first read does not decode), copy the old record to the backup and read it back, THEN
  replace the record and read it back (restoring the old one if that fails), THEN refresh the backup,
  THEN re-read both files. `SAVED` only when the record file re-reads as the new record; `SAVE
  UNCONFIRMED` when the card is gone, refuses every open, or does not read back the same way twice. It is
  designed so that a failure part-way leaves the record in effect before or the new one — given HA-166
  (4) and (5), and short of two faults in one save that begin with a misread of its first reads that its
  checks cannot see (the header states the limit). A store's read count is never trusted past the
  buffer, and a read that did not open counts as no bytes.
- **`calibration::kRecordBackupFileName`** = `"shulib_drivetrain.bak"`, beside `kRecordFileName`, with a
  compile-time check that both names are bare and name two different files under FAT's name rule.

**What you must do:** nothing — no existing call changes. To reset a card on purpose once the programs
use the backup, delete BOTH files. **Until "shulib Calibrate" saves through this (Part B1), copy
`shulib_drivetrain.bin` off the card before every Calibrate rerun** — its NOT SAVED can still mean the
record was erased.

**Hardware assumptions:** HA-166 is NEW (the six beliefs about the SD card the design rests on — three
reasoned from the vendored binaries, two invented, and FAT's name rule, whose VEXos half is invented);
HA-122 gains two cross-references (no status change).
See the [hardware assumptions register](hardware-assumptions.md).

### Added and changed since 2.4 (2026-10-01) — the drive protections, inside the driver loop

> Written when it landed (chunk T4 Part 2, its library half). **No version bump:** nothing here
> extends or changes a FROZEN surface — `teleop/` and `hal::MotorGroup` are not in the
> [Freeze Register](roadmap.md#freeze-register), and `version.hpp` bumps `kApiMinor` for additive
> extensions of a frozen one — and **`hal::IMotor` (F4) is untouched**: a dead port is recognised
> from what the seam already reports, not from a new validity channel. Every previously-compiling
> program compiles unchanged. **Host test plant and hand-made readings only — nothing here has run
> on a robot, and every number is invented** (A4 register HA-132, HA-175, HA-176).

**BEHAVIOUR CHANGE (driver control):** `teleop::runDriverControl` now runs the old "shulib Drive"'s
three protections on every call, from `DriverLoopConfig::protection`'s defaults unless a program
states otherwise (the team lead's ruling R4, 2026-09-27):

- **A drive member whose readings FREEZE is dropped.** A member of a `hal::MotorGroup` whose position
  AND velocity repeat bit for bit for 5 reads while it, or a side-mate whose readings still change,
  says the train turns — with at least as many of its side answering as not — is kept out of the
  group's fight monitor at once, and after 250 ms marked ABSENT for the rest of the run (out of every
  read, still commanded), with one `[WARN][TEL]` line naming the port. **Known gap:** at a wall it can
  mark a LIVE motor — one whose speed report settles well before its mates', or, beside a slipping
  gear, one that reads unlike its stalled mates (A4 register HA-175, its known gaps).
- **More than two dead drive motors in total stop the drive for the rest of the run** — 0 V, one
  `[ERROR][TEL]` line, and every later start of driver control refused until the program is started
  again (the marks live in the groups). One or two dead drive on, degraded. The BOOT rule is unchanged.
- **A fight, or an over-current, CUTS the drive for 1 s** — a group's own monitor persisting (the same
  verdict that raises `MOTOR_GROUP_DISAGREE`), or one member above 2.4 A for 250 ms while its side is
  driven above 1 V: both sides to 0 V in that tick, a cut line and an `at the cut:` line per member,
  then a re-arm. A drive motor in no group (the X-drive) gets the over-current cut only.
- **New log lines, tag `TEL`:** the cut, re-arm, absence and refusal lines above; a peak line at the end
  of a driven stretch whose highest member current is a new high for the entry; and, on the disable,
  one summary line per group (each member's peak current and temperature, and the cut count).
- A drive with nothing dead, fighting or over current commands exactly what it did before: the whole
  pre-existing suite runs unchanged (two cases count more assertions: one checks each header, one checks
  every line a disable writes).

**What you must do:** nothing. A robot that must push against a wall or a robot in a match can make
the over-current cut a logged, counted warning with
`DriverLoopConfig::protection.overCurrentCuts = false`; the fight cut has no switch. Measure your
robot's push — and a full-stick reversal at top speed and a full-stick launch from rest, which may
hold the limit too — before deciding (HA-176).

**Added:**

- **`teleop/drive_protection.hpp`** (new): `teleop::DriveProtectionConfig` (`overCurrentAmps` 2.4,
  `overCurrentTicks` 25, `cutTicks` 100, `maxCutsLogged` 50, `overCurrentCuts` true,
  `frozenSuspectReads` 5), `teleop::ProtectionMode`, `teleop::DriveProtectionState` (trivially
  copyable, for a panel), `teleop::kMaxProtectedGroups` (4), `teleop::kProtectionTextBytes`,
  `teleop::validProtectionConfig()`, and `teleop::DriveProtection` (the loop builds one per call; a
  program never needs to).
- **`teleop::DriverLoopConfig::protection`** — appended LAST, so every designated initializer still
  compiles; checked before the first tick, before anything is commanded (a bad one is a
  `PreconditionError` and nothing moves).
- **`runDriverControl`'s `perTick` may take a fourth argument**, `const DriveProtectionState&`
  (detected at compile time; the three-argument form is unchanged).
- **`hal::MotorGroup`:** `markAbsent(i)` (noexcept; refuses — never throws — out of range, a boot hole,
  a second mark and the last present member), `runtimeAbsentMask()`, `setMonitorExcluded(mask)` /
  `monitorExcludedMask()`, and labels — a third, defaulted constructor argument
  (`std::span<const int>`, one port number per slot) and `label(i)`. `presentCount()` and `isPresent()`
  now count a runtime mark too; `absentMask()` stays the boot holes; `member(i)` returns a marked
  member and still refuses a boot hole. `robot::Robot` labels its groups with the description's port
  numbers, and `describe()` names a runtime mark as absent.
- **`teleop::FrozenReadingDetector`**, `teleop::MemberReading`, `teleop::FrozenVerdict`
  (`teleop/coupled_side_monitor.hpp`) — the frozen-reading rule, pure and allocation-free.
  `teleop::MemberAbsenceDetector` is unchanged (the library does not run it: it needs a faulted-read
  count IMotor does not have).
- **`teleop::evaluateRuntimeDegradation(left, right)`** (`teleop/drivetrain_degradation.hpp`) — the
  mid-run rule: only the total cap (more than two dead refuses); `evaluateDegradation`, the boot rule,
  is unchanged.

### Added since 2.4 (2026-10-01) — the SHUL/2 wire format and its reader

> Written when it was built (chunk H1 Part A1 — the READ side of the SHUL/2 telemetry wire; its
> writer, the sink, is Part A2, and the freeze of register row F9 is Part B). **No version bump —
> every change is ADDITIVE:** nothing frozen changed shape, and the wire carries its own version in
> every Hello (F9 is not frozen yet, so its layouts may still move until Part B). **Host only —
> these bytes have never crossed a real link**, and no program writes them yet.

**ADDED:**

- **`include/shulib/diag/crc32.hpp`** — `diag::crc32Ieee`, the bitwise CRC-32/IEEE (the same number
  `zlib.crc32` gives), moved here from the calibration record so the wire can share it.
- **`include/shulib/diag/shul2_format.hpp`** — namespace `diag::shul2`, the SHUL/2 v1.0 format:
  BINARY (not the "JSON wire" older documents named), little-endian binary64, a 14-byte envelope with
  a CRC on every frame; five frame types — Hello, Tick, Log, Summary, End — where a Tick and a Summary
  ARE the SD blackbox's own bytes. The envelope constants, `FrameType`, `kSignature`, `WidthRow` and
  `kWidthTable`, `ReaderProfile` and `kThisReader`, `Verdict` and `negotiate()` (another major is
  refused; a newer minor is read, its appended bytes ignored; an older minor is read at its own
  widths), `ScanResult` and `scanFrame()` (the one function any reader — a file reader or a live
  socket loop — is built on), `encodeFrame`/`encodeHello`/`encodeLog`/`encodeEnd`,
  `decodeHello`/`decodeLog`/`decodeEnd`/`decodeTick`/`decodeSummary`, `WireLevel` with
  `wireLevel()`/`logLevelFromWire()`, and `utf8Prefix()`.
- **`include/shulib/diag/shul2_reader.hpp`** — `Shul2Reader`, a reader over a whole capture that
  delivers only verified frames of accepted sessions, resynchronises one byte on after any damaged
  frame, names what it could not read (`CaptureStatus`: EMPTY, NO_FRAME, NO_SESSION, REFUSED, OK) and
  counts every skipped byte and frame by name (`ReaderCounters`, `SessionReport` — including sequence
  gaps and, when an End arrived, the frames the link itself lost). It BORROWS the capture: keep the
  buffer alive as long as the reader, and never hand it a temporary.
- The format is written out for other tools — every layout, value table and rule — in
  [`vexbuilder-integration.md`](vexbuilder-integration.md) §5.2.

**FIXED:**

- **`blackbox::decodeTick` now clamps `wheelCount` to 8** (`DebugRecord::kMaxWheels`) and raises
  `corrupt` when the byte is above it — before, a damaged byte of 9–255 decoded as-is, and any
  consumer that looped to `wheelCount` would index past the 8-wide wheel arrays.
  **`blackbox::encodeTick` writes it clamped to [0, 8]** (before, a count above 255 wrapped). No
  file changes: every writer in the tree already wrote at most 8.

**CHANGED (same behaviour):** `calibration::crc32Ieee` is now a one-line forwarding function to
`diag::crc32Ieee` — the same spelling and the same number, so the calibration record and every
caller are unchanged.

**What you must do:** nothing.

### Changed since 2.4 (2026-10-01) — a motor group leaves out a reading that is not a number; the calibrator names a port with no reading

> Written when it landed (DIAG1 Part B). **No version bump:** `hal::MotorGroup`,
> `hal::medianInPlace` and the drivetrain calibrator are not in the
> [Freeze Register](roadmap.md#freeze-register) (F4 freezes `IMotor`'s signatures, which are
> untouched), every signature that existed still compiles (in a build that does not assume every
> number is finite — CHANGED, below), and a group whose every reading is a number reads exactly as
> before, bit for bit — except the one overflow under CHANGED, below. **Host test plant only —
> nothing here has run on a robot**, and on a V5 none of it is expected to fire: the motor adapter
> replaces a failed read with the last good one before any group or the calibrator sees it
> ([HA-08](hardware-assumptions.md), amended). It defines what the library does when a reading
> does get through — a future adapter, a nested group, a host test.

**BEHAVIOUR CHANGE (a member reading NaN, +Inf or -Inf — never on a V5 through `ProsMotor`):**

- **`hal::MotorGroup`'s `position()` and `velocity()` leave such a member out of that read,**
  the way they leave out an absent member: the median of the present members whose reading is
  finite, and a fresh NaN only when none is. The rule is per read — the member's next finite
  reading counts again — and `presentCount()` / `absentMask()` never change for it. Before, the
  median sorted every value: a NaN broke `std::sort`'s ordering (undefined behaviour — on the host
  library, 21 members with 1–3 NaN gave a WRONG FINITE number on 18,030 of 20,000 draws), and an
  infinity was an ordinary outlier until infinities of one sign made up half the members or more
  (a single one on a side of one or two members): the side then read ±Inf, or NaN where both
  signs met in the middle. On a drive-encoder robot, one bad member of a side no longer makes the
  side NaN (which ended the motion on `ODO_STUCK` — observed on the host test plant: a scheduled
  `moveTo` with one of five members reading NaN mid-motion used to abort on the next tick, and now
  finishes identical, bit for bit, to the clean run).
- **Leaving members out can STEP the side's reading, and step it back when they return** — the
  membership step ([HA-165](hardware-assumptions.md), new, invented). For one member left out it
  is at most half a gap between the members' positions beside the middle one; for several it can
  be as large as the members' spread — at worst, the member farthest from the side's median read
  alone. The odometry takes the step as travel: under its 36 in travel bound the pose moves
  silently, past it the tick is `TRAVEL` and raises `ODO_STUCK`. That now includes a side whose
  infinite members used to reach the middle: it read infinite, which froze the tick
  (`NONFINITE+TRAVEL`, `ODO_STUCK`); it now reads the rest's median and integrates.
- **`current()` and `totalCurrent()` sum only the finite readings, and `current()` divides by
  how many that was; `temperature()` is the max over the finite readings.** A member reading
  +Inf °C no longer raises `MOTOR_OVER_TEMP` on its own, and a NaN in the FIRST member no longer
  makes the temperature NaN (it used to, and a NaN anywhere else did not — the answer depended on
  member order). With no finite reading, all three return NaN — `totalCurrent()` included: an
  empty sum would read 0 A, a side drawing nothing. A side whose readings are all finite reads
  exactly as before, bit for bit — 0 A at rest included. `temperature()` now reads each member
  once.
- **The drivetrain calibrator no longer makes a candidate whose push reading is not finite into a
  drive port.** Before, a NaN push reading was already "not moved" — silently — while a ±Inf one
  passed the moved test and became a drive port with a sign read off PROS's failure sentinel; on
  the host test plant that run was then refused `PULSE_CONTRADICTION` at that port, and one whose
  motor recovered after FRONT could finish with it on the WRONG side. Now one predicate decides
  "moved on the push" for the classification, `pushSummary()`, `pushReady()` and the live
  `movedNow`, and the candidate is NAMED: its PUSH TABLE line reads `NO READING` where it read
  `still` (a NaN) or `MOVED` (±Inf) — `push port 13 travel +inf deg NO READING sign +0` — and
  `warning()` gains `port 13 gave no reading at DONE -- not counted as a drive port; if it drives
  the robot, check it and run again`. The Done screen shows the first two 50-column lines of it
  (up to `check it and`); the program's serial summary prints `warning()` only beside a port LEFT
  OUT as slipping, so there the PUSH TABLE line is what names the port. If that port does drive
  the robot and recovers, the first pulse on its side back-drives it and the run is refused
  `NON_DRIVE_MOVED` naming it, with a reason that says so — `port 13 moved (… rad/s) while only
  port … was pulsed -- it gave no reading at DONE, so it was not counted as a drive port` (a
  port that READ a number and did not move keeps the old reason). Its `PortObservation`'s
  `pushTravelRad` is exactly +0.0, as every port's with no sign now is: it was sign × reading —
  a NaN for a port with no reading, and -0.0 for a still port read slightly backward (the one
  difference on an all-finite run, and nothing reads it: the scale reads drive ports only, and
  the PUSH TABLE prints the reading). The spin's side travel uses the same median, so a member
  reading NaN there is left out too.
- **The `ODO_STUCK` line:** a drive side whose every member is `+Inf` now prints `L=NaNin`
  (the fresh NaN), where it printed `L=+Infin`. The odometry's verdict is the same.
- **The stall check (`motion::OdoStallCheck`), on a drivetrain with an independent motion source
  (tracking wheels — the check's default):** a drive side with no finite reading when a window
  closes now gives that window NO verdict, nor the next (its new baseline is the NaN), where a
  side all `+Inf` or all `-Inf` read as an infinite spin travel and raised `ODO_STUCK`. A side
  with only SOME members infinite reads the others' median instead of `±Inf`, so its window is
  judged on its real travel. (Robot two has no independent source; its check gives no verdict at
  all — [HA-131](hardware-assumptions.md).)

**CHANGED (a weaker post-condition):**

- **`hal::medianInPlace(v, n)` REORDERS `v`; its contents afterwards are unspecified** (it used to
  sort `v` in place). It leaves out a value that is not finite, as above, returns a fresh quiet NaN
  when none is, and takes the midpoint of an even count with `std::midpoint` — bit-identical to
  the old `0.5 * (a + b)` wherever that did not overflow, and finite where it did (a middle pair
  whose sum overflowed, which gave `+Inf` or `-Inf`). Every caller in the library passes a scratch
  array and never reads it again. **Migration:** if you read `v` after the call, sort it yourself.

**CHANGED (a build constraint):**

- **A build that assumes every number is finite is refused.** `hal/motor_group.hpp` stops the
  compile with an `#error` under `-ffinite-math-only`, `-ffast-math` or `-Ofast`: under those flags
  the compiler may fold `std::isfinite` to `true` (measured: `arm-none-eabi-g++` 13.2.1 does), which
  would put every NaN this change leaves out back into the median, with no warning. No build in
  this repository sets any of them. **Migration:** remove the flag.

**ADDED:**

- `hal::MotorGroup::nonFiniteReads()` — how many member readings the group has left out for not
  being finite, cumulative, counted with the exclusion's own test (it mirrors
  `ProsMotor::faultedReads()`). An observable only: nothing raises on it.
- `hal::isUsableReading(double)` — THE test (`std::isfinite`): the median excludes with it, the
  counter counts with it, and the calibrator's push rule is built on it.

**Migration:** nothing to change, unless you read `medianInPlace`'s array after the call or build
with `-ffast-math` (both CHANGED, above). To see whether a group ever left a member out, read
`nonFiniteReads()`.

### Added — backing into a point in a recipe: `Routine::reverseTo` and `Routine::faceAway` (L9 Part A1, 2026-09-29)

> Written when it landed (chunk L9, Part A1). **Minor bump, 2.3 → 2.4** — two new steps on the
> FROZEN recipe layer ([Freeze Register](roadmap.md#freeze-register) row F10), its first new steps
> since it locked, added by the path the freeze itself names (new steps, pinned the day they land);
> nothing that existed changed shape or behaviour. **Host test plant only — nothing here has run on
> a robot.**

**ADDED:**

- **`Routine::reverseTo(x, y, options)`** — back into a field point: arrive with the robot's back to
  it. One `moveTo` whose target heading is the bearing from the live pose estimate to the point plus
  180°, computed when the step runs. It is `driveTo` turned around, and exactly a route's `"drive"`
  waypoint with `reverse` set: a route and a recipe that back into the same point drive identically,
  bit for bit.
- **`Routine::faceAway(x, y, options)`** — rotate in place so the robot's back points at a field
  point: `face` turned around, one `turnTo`. After it, a `reverseTo` to the same point drives
  backwards the whole way, on any drive.
- Both pass your options through whole, stop the chain under their own names (`(reverseTo)`,
  `(faceAway)`) when their motion fails, and throw — like `face` and `driveTo` — when the point is
  where the robot already is (that message now names all four steps).
- **A `reverseTo` leg starts as far off its heading as the robot's back is turned from the point.**
  On a tank, write `faceAway` first — the turn is yours to write, as `face` is before `driveTo` — and
  the backwards leg starts within about a degree of its heading, so it is steered back onto its line
  like any tank leg since 2.3. From a robot *facing* the point, a `reverseTo` with no `faceAway`
  starts 180° off: it is not steered, it logs `tank moveTo not steered: it starts 180.0 deg off its
  target heading (steers within 15.0 deg) -- turn first: face()` — for a backwards leg that turn is
  `faceAway` — and, measured on the host test plant, the robot swings round as it drives and times out
  beside the point: almost 8 in beside it at the end of a 43 in leg, still about an inch at the end
  of a 6 in one. A tank that just drove straight out from the point already has its back to it, so
  backing straight back needs no `faceAway`. A holonomic drive arrives turned round without
  `faceAway`, because it rotates in flight — so its nose leads early in the leg; write `faceAway`
  first if the back must lead the whole way.

**What you must do:** nothing — every 2.3 program compiles and behaves the same (`kApiMinor` is now
4 and `kApiVersionString` `"2.4"`). Whether a tank backs up exactly as far as its calibration says —
the scale comes from one unpowered push — has not been measured: HA-167 in the
[Hardware Assumptions Register](hardware-assumptions.md).

## API 2.3

### 2026-09-30 — the competition program runs the competition library in a match — no API change

> Written when it was built (chunk COMP-b). **No version bump and no library change:** nothing under
> `include/shulib/` moved. **Nothing here has run on a robot.**

"shulib Competition" (`src/programs/competition_program.cpp`, `make ROBOT=tank PROGRAM=competition`,
slot 6) is the first program whose `autonomous()` runs an auton: the one chosen on the brain's
`competition::AutonMenu` runs through `competition::runAuton` — the call the bench's "shulib Auton
Tester" makes — on a task the program owns, under a `sequence::RunGuard` whose inner pacer is the
program's own watch, which calls `RunGuard::requestAbandon()` on every tick after the robot is
disabled or leaves autonomous. [Guide chapter 17](guide/17-the-competition-program.md) describes the
program; what it shows a library user is the pattern for those three pieces in a real program: run
the auton on a task PROS does not delete, end it through the abandon latch rather than a killed task,
publish a terminal word on every path out of a run (a throw included) before driver control touches
the robot, and prove the pacer chain at boot (one idle tick must reach the guard and the watch, and the
watch must be attached to the guard). **Hardware assumptions:** HA-162 is NEW (what the controller's
Timed Run does after it starts a program disabled — unknown, measured first at the program's bench
station), and HA-161's settle now reads the program's own log lines. See the
[hardware assumptions register](hardware-assumptions.md).

Its review (2026-10-01) added three more parts of that pattern, before anything ran: the auton task
does not start a run while the driver loop still holds the robot — it waits until the loop has left,
or until the loop's once-a-tick heartbeat has stopped for 0.1 s — so the two never share the robot
even if PROS keeps `opcontrol()` running at a change to autonomous; a request served after its period
has already ended reads the competition status once before anything moves and runs nothing (`TOO
LATE`); and the controller's three lines are written one at a time, at most one every 0.1 s, the word
line first, because PROS does not apply faster text updates. That last change applies to "shulib
Teleop" too: both programs write through the same writer.

### Added and fixed since 2.3 (2026-09-28) — an append mode for the SD card, a text sink over a block sink, a guard pace count; a registry warning fixed

> Written when it landed (chunk COMP-t — "shulib Auton Tester", the bench program that runs one of the
> team's autons from the brain's screen through the competition program's own `competition::runAuton`
> call). **No version bump:** every change is additive or a function body — no signature moved, no
> existing behaviour changed except what happens when a mechanism's adapter THROWS during the run
> guard's safing (the `RunGuard` fix below, found by the chunk's review), and none of these headers
> is in the [Freeze Register](roadmap.md#freeze-register) (`RunGuard` is row F12, open by design).
> **Host tests only; nothing here has run on a robot.**

**ADDED:**

- **`hal::pros::ProsBlockSink(fileName, ProsFileMode mode, mountRoot = "/usd/")`**, with
  **`ProsFileMode { Truncate, Append }`**, **`AppendOpen { Appended, Created, Refused }`**,
  **`appendOpen()`** and the free function **`hal::pros::openForAppend(path, open, close, how)`**.
  Append keeps every earlier byte of the file and writes after them — one log across reboots. The
  decision is made in `openForAppend()` alone: ask with `"rb"` whether the file exists, open `"ab"`,
  and create with `"wb"` ONLY when the file was absent, so an existing file is never emptied
  whichever way the brain answers an append on a missing file (that is a belief —
  [HA-177](hardware-assumptions.md)). No mode has a `+`. **The two-argument constructor is unchanged:
  it delegates with `Truncate` and still opens `"wb"`, byte for byte** — the blackbox's `SdSink` and
  every other existing caller see no difference.
- **`hal::BlockCharSink`** (`hal/block_char_sink.hpp`) — an `ICharSink` over a borrowed `IBlockSink`:
  each `write()` is one block write and one flush; a refusal of either is dropped and counted
  (`droppedWrites()`), never thrown. A `diag::TermSink` over it puts serial's exact lines on the SD
  card.
- **`sequence::RunGuard::pacesTotal()`** — every `pace()` since construction, whether or not a run
  was live (`RunGuardReport::pacesSeen` still counts only a run's, as before). A program reads it
  around one idle tick to PROVE its `Chassis` is paced through the guard — the wiring `run()` cannot
  verify; the Auton Tester refuses its boot when that proof fails.

**FIXED:**

- **`competition::checkAutonEntry` — a `-Wformat-truncation` in every V5 build the first time a
  program registers an auton.** Its buffer for a route's refusal sentence was 160 bytes; at `-Os` GCC
  inlines `Route::valid()` into it and sees a sentence of up to 173 bytes. The buffer is now 192 (the
  size `runner::PathRunner` uses for the same call) and the outer copy is bounded (`%.150s`).
  Nothing called `add()` from a V5 source until the team's auton file did, so the warning — which
  the robot build treats as an error — had never been emitted. No behaviour change on the host.
- **`sequence::RunGuard` — a mechanism whose safe state (or whose claimant's `cancel()`) throws no
  longer skips the mechanisms after it, and no longer ends the program in `std::terminate`.** Before,
  the throw left the safing loop at once, so every mechanism listed after the thrower kept its last
  command; and when it happened at a run's end, the unwind path's own cancel-all threw a second time
  from a destructor — `std::terminate` (measured: exit 134), so no handler ever ran. Now every
  mechanism is attempted, the FIRST exception is rethrown after the sweep (a broken adapter stays
  loud), and the unwind path drops its repeat. The drive was never affected (the scheduler's cancel
  runs first), and no shipped adapter throws there (`ProsMotor::setVoltage(0)` and `setBrakeMode`
  have no throwing path) — so nothing changes unless an adapter is broken.

**What to do:** nothing, unless you write to the SD card from a program: then open a log you want to
keep across reboots with `ProsFileMode::Append`, and never with a raw `fopen` (the adapter owns the
`/usd/` prefix and every mode).

### Changed — `moveTo` on a tank steers back onto its line (TANK1, 2026-09-27)

> Written when it landed (chunk TANK1, Part A; its Part B — the result line's ` xt` column —
> landed with change 3 below, 2026-09-28). **Minor bump, 2.2 → 2.3** — the team lead's ruling
> (2026-09-27): one sentence the FROZEN `Chassis` header carries ([Freeze Register](roadmap.md#freeze-register)
> row F6, its strafe-fallback semantics) said that on a tank,
> laterally-offset targets exit `TimedOut`, and after this change some of them arrive. That relaxes a stated LIMIT — the same
> direction as 2.2's relaxed `Route::valid()` rule — and no signature moved, so the bump is minor;
> but tank programs move differently, so it is listed as a behaviour change with a one-line way
> back. **Tank only: the X-drive and H-drive are unchanged, byte for byte** (every record, log
> line and motion checked with the change on and off) — with one exception by design: an H-drive
> configured with NO strafe authority (`strafeTractionDerate` 0, a dead strafe wheel) cannot move
> sideways either, reports authority 0, and is steered — and reported, ` xt` included — like a
> tank. **Host test plant only — nothing here has run on a robot.**

**BEHAVIOUR CHANGE (tank drives — a drivetrain whose `strafeAuthority()` is exactly 0):**

- **A tank `moveTo` leg that starts within 15° of its target heading now steers back onto its
  line while it drives.** The line runs through the target point along the target heading (for a
  recipe `driveTo`, the line from where the step started to the point). The heading the leg servoes
  to is bent toward the line — 0.1 rad per inch off it, never more than 15°, fading to nothing over
  the last 6 in; the inches counted are where the robot WILL be 0.06 s ahead at its heading and
  commanded speed, so the bend eases while the robot is already closing on the line instead of
  carrying it across — and the turn the robot makes that the heading loop did not ask for (a
  weaker side's pull) is measured and cancelled, never more than 1.5 rad/s. It never aims at the
  target POINT and never plans a turn: `face()` is still your turn. The three changes you can see:
  1. **Legs that already arrived take slightly different paths.** Any engaged tank leg with a
     cross-track error or a yaw disturbance moves differently from before, including legs that
     used to arrive: on the host test plant the library's acceptance run on an even drive ended at
     (0.24, 0.12, 180.00°) in 5.42 s before and ends at (0.23, 0.04, −179.97°) in 5.44 s now. Legs
     that used to time out on a weak side now finish: the 40 in leg with one side 3 %, 5 % or 9 %
     weak timed out 0.93 / 1.56 / 2.86 in off its line and now finishes within 0.02 in of it.
     **A leg that arrived before still arrives** — checked on the host test plant over 24 563 legs
     that start up to 14° off their target heading toward points up to 60 in away, with motor lags
     from none to about 0.18 s: none that settles with the change switched off times out with it
     on. (A review of this change caught a draft that failed that check — a leg starting 10–15°
     off its heading and turned toward its line crossed the line near its target and timed out
     half an inch beside it; steering on where the robot will be, above, removed every one.) The
     limit, stated: with about 0.24 s of motor lag, 24 of the 1 867 such legs that arrive with the
     change off time out with it on — at that lag the heading loop itself is at its limit, which
     R5's measured tuning is for.
  2. **An un-faced tank leg logs one Warn.** A tank `moveTo` leg that starts more than 15° off its
     target heading is not steered at all — it runs exactly as before — and logs, once, at its
     first live tick: `[WARN][MOT] tank moveTo not steered: it starts 26.6 deg off its target
     heading (steers within 15.0 deg) -- turn first: face()`.
  3. **Translating tank result lines end with an ` xt` column** (TANK1 Part B, 2026-09-28): the
     final cross-track error — how far the leg finished off its line, in inches, + to the left of
     the target heading — appended after the duration. On the host test plant the 40 in leg
     with its right side 9 % weak now ends `… drift  0.1°   1.89s xt   0.02"` (with the left side
     weak, `xt  -0.02"`: a leg the steering brings home can finish a hair to either side of its
     line). It is on the line of every tank motion that set out to go somewhere, whatever its
     outcome (a `moveTo`/`driveTo` leg, each `followTrajectory` leg, a `strafeTo`, and a Tier-3
     `HoldPose` built with a spot away from the robot); turns, `brake` and the `hold` verb (it
     holds where the robot is) never carry it, and no X-drive or H-drive line does — except an
     H-drive with no strafe authority, which is treated as a tank (above).
     Every earlier byte of the line is unchanged, so a reader that finds the fields by position
     still finds them; one that expects a tank's line to END at the duration will meet the extra
     column. With no record stream (`over n/a`) the column is left out.
     [Chapter 11](guide/11-reading-the-diagnostics.md#per-motion-result-lines) reads it.
- The tick log line of a steered tick carries the new flag ` TSTR` (after ` SFB`); ` SFB` lights
  on far fewer tank ticks than before, because the sideways error it reports is being steered out.
- `strafeTo`, `hold`, `turnTo`, `face`, `brake` and `drive()` move exactly as before on a tank;
  of their result lines, only `strafeTo`'s changes — it gains ` xt` (change 3).

**ADDED:**

- `motion::TankSteerConfig` (`crossGain`, `maxSteer`, `fadeDistance`, `leadTime`,
  `yawObserverTime`, `yawCorrectionLimit`; every value PROVISIONAL — HA-179 in the
  [Hardware Assumptions Register](hardware-assumptions.md)) and `MotionConfig::tankSteer`,
  appended last (every designated initializer still compiles); `MotionConfig::validate()` refuses
  an invalid one.
- `motion/tank_steer.hpp` — the pure geometry: `TankLineError`, `tankLineError()`,
  `tankLeadCross()`, `tankSteerAngle()`, `tankLegEngages()`, `TankSteerStatus`,
  `kTankSteerNoiseFraction`.
- `MoveToPose::tankSteerStatus()` — whether a motion may steer, whether this run does, and the last
  commanding tick's cross-track error, the predicted one its bend steered on, the bend and the yaw
  correction.
- `diag::DebugRecord::tankSteerActive` (appended last) — on the SD-card blackbox as tick-flags bit
  2 (the file format version stays 1: every earlier writer wrote that bit as 0, so an older file
  reads it as false, which is true of it); on the terminal as ` TSTR`.
- `PoseMotionOptions::tankSteer` (Tier 3; false unless you build a plain `MoveToPose`).
- The ` xt` column's data (Part B), each appended last: `motion::CompletedMotion::hasCrossTrack`
  and `crossTrack` (`Chassis::lastCompleted()` carries them); `diag::MotionResult::hasCrossTrack`
  and `crossTrack`, which `diag::emitResultLine` renders; and `MotionStatsSink::hadDirection()`
  (did the motion set out to translate — judged on its first live target) and
  `MotionStatsSink::crossTrack()`.

**Migration:** nothing to change to keep compiling. To get the old tank back exactly, set
`maxSteer` to 0 — `ChassisConfig.motion.tankSteer.maxSteer = 0` (on a robot built from a
`RobotConfig`, its `tuning.motion.tankSteer.maxSteer`): a tank's `moveTo` then moves, records,
logs and ends exactly as it did at 2.2, bit for bit, with no Warn — except the ` xt` column on its
result lines, a diagnostic kept whatever `maxSteer` is (with the steering off it reads the drift
the steering exists to remove: ` xt  -2.86"` on the 40 in leg with its right side 9 % weak,
` xt   2.86"` with its left side weak).
The documented limits of the steering —
what it removes and what still times out, and what motor lag does to it — are in the guide's
[chapter 14](guide/14-what-it-cannot-do-yet.md#motion-quality-boundaries).

## API 2.2

### Changed since 2.2 (2026-09-26, later) — driver control releases a leftover claim; the boot check names declared mechanism motors

> Written when it landed (T3 Part 2 — robot two's claws wired into "shulib Teleop"). **No version
> bump:** `teleop::runDriverControl`, the mechanism bindings and `calibration::verifyAgainstRegistry`
> are not in the [Freeze Register](roadmap.md#freeze-register), and every signature that existed
> still compiles and means what it meant, except the one behaviour change below. **Host test plant
> only; nothing here has run on a robot.**

**BEHAVIOUR CHANGE (the team lead's ruling, 2026-09-26):**

- **`teleop::runDriverControl` RELEASES a claim still on a bound mechanism when it starts.** An
  operation autonomous was running when the field ended autonomous never releases its own claim —
  PROS deletes that task wherever it is — and a binding never commands a claimed mechanism, so the
  driver was locked out of that mechanism for the whole of driver control ("Added since 2.2 —
  mechanism buttons in driver control", below, told a program to release it itself). Now the loop
  does it: after its set check (a refused set still touches nothing) and before the entry safe
  state and each toggle's `enter()`, every hold
  binding's and toggle's mechanism that is still claimed is released — the claimant's `cancel()` is
  NOT called, because it may have lived on the deleted task's stack — and the loop logs one Warn
  line per release on the chassis's own sink, tag `TEL`: `driver control: RELEASED a claim left on
  mechanism '<name>' at entry (an operation that outlived its task) -- its buttons work from the
  first tick`. A claim taken DURING the loop is untouched: the binding still defers to it.
  **What it moves, for a toggle:** the release comes before the toggle's first `enter()`, so at the
  FIRST driver-control start the toggle's initial level now goes out even when an operation had
  the mechanism claimed — a claw an autonomous operation left claimed at the OTHER level moves to
  the initial level (a claw that starts closed closes) the moment driver control starts, a
  movement nobody pressed a button for. Before, a claimed claw stayed where the operation left it;
  an unclaimed one already moved (the toggle's rule since it was added).
  **Migration:** a program that released the claim itself before the loop may keep doing so (the
  loop then finds nothing and says nothing) or drop it. A program that starts an operation on a
  bound mechanism BEFORE calling the loop, meaning to tick it from the per-tick hook, loses that
  claim at the loop's start: start the operation from the hook instead. The dead operation object
  still believes it holds the claim, so do not `cancel()` it later — that would safe the mechanism
  under whoever holds it then. If your autonomous leaves a toggle's mechanism where driver control
  must find it, state that level as the toggle's initial level — the toggle has no "keep what is
  there" start.

**ADDITIVE:**

- **`MechanismBinding::releaseLeftoverClaim()` / `releasedLeftoverClaim()`**, and the same pair on
  **`ToggleBinding`** — what the loop calls at entry, and what a panel reads afterwards ("a claim
  was released when driver control started").
- **`calibration::DeclaredMotorPorts`** and a three-argument **`verifyAgainstRegistry(desc,
  registry, declared)`**: motor ports the PROGRAM declares as mechanism motors, and the words for
  them. A registry motor on one of them that the description does not list is expected — counted in
  `BootVerdict::declaredPorts` / `declaredCount` and named in its own clause after the NEW motors
  (`NEW motors not in the calibration: 14; claw pivots (declared, not drive motors): 6 7`), never
  as a NEW motor. The two-argument form is unchanged, word for word.

### Added since 2.2 (2026-09-26) — the competition library: an auton registry, a selector, a runner, abandon, and the start-heading seed

> Written when it was built (chunk COMP-a, the LIBRARY half of the competition program; the
> program itself — `src/`, the seventh build — is COMP-b). **No version bump — every change is
> ADDITIVE:** nothing frozen changed shape. `RunGuard` (Freeze Register row F12) is not frozen;
> `Localizer` and `robot::BuildOptions` are outside every freeze; `Routine::startAt` (F10) and
> `Chassis::setPose` (F6) are untouched and still seed position only. **Nothing below has run on a
> robot** — it is proven on the host test plant only.

**ADDED:**

- **`robot::BuildOptions::pacer`** (default `nullptr` = the provider's pacer, so every existing
  graph is byte-for-byte unchanged). A competition program passes its `sequence::RunGuard` here —
  the guard reaches a running motion only through the pacer, and a Chassis takes its pacer once.
  **Unlike `absentDrivePorts`, the pacer is BORROWED FOR THE ROBOT'S WHOLE LIFE** (the Chassis
  keeps it): spell it `robot::BuildOptions{.absentDrivePorts = ..., .pacer = &guard}` with the
  guard constructed first and living as long as the Robot.
- **`sequence::RunGuard::requestAbandon()`** and `abandonRequested()`, plus
  `RunGuardReport::abandoned` — the third way a run ends (the first two are the deadline and the
  hard stop). One atomic store, safe from any task and from inside a pacer; from the next check on
  every motion is cut before the world advances and every guard wait returns `RunExpired`; when
  scoring returns, `run()` does the full cancel-all, SKIPS the end action, and disarms. It is a
  latch, never a disarm: the routine unwinds normally, on its own stack. A request made while no
  run is live is dropped when the next run arms. **What you must do:** nothing — no existing call
  changes; `RunGuardReport` gained a field.
- **`localization::Localizer::reseatHeading(math::Angle)`** — the heading's twin of `setPose`: the
  published heading becomes the given one on this tick (through the same bias the tag corrector
  moves; the IMU stays the only source of heading CHANGE), and the next tick re-baselines as after
  `setPose`. Preconditions: the estimate is live, and no motion is active. It is a CALLER seed —
  no sensor and no corrector can reach it. **What you must do:** nothing; `setPose` still ignores
  heading.
- **`include/shulib/competition/`** (new directory):
  `auton_registry.hpp` — `AutonRegistry`, `AutonEntry` (a name, MATCH or SKILLS, a routine OR a
  route, a FIELD or RELATIVE `StartPose`, `endActionAt`/`hardStopAt`, an optional end action),
  `AutonPeriods` (the program's two periods — required, never defaulted), `checkAutonEntry`;
  every unset or impossible field is refused at REGISTRATION, by name. A ROUTE entry's start is
  its route's `Start` waypoint — and that waypoint's heading is read only when it is `Fixed`: a
  `Free` Start states no heading, so the entry must declare a FIELD start at the same (x, y) or
  it is refused (a default 0 is never seated by accident).
  `match_runner.hpp` — `runAuton(...)`: refuses before arming, arms the guard, waits (with the
  guard's own wait) for a live estimate, re-seats position AND heading on one tick, runs the
  routine or the route, and returns a TAGGED `AutonRunResult`. A run whose estimate never went
  live (`NotStarted`) brakes at its end — the entry's own end action is never run in a frame
  that was never seated.
  `auton_menu.hpp` — `AutonMenu`, the brain-screen selector as a pure, host-tested model: NONE
  always present and the default, paged, one atomic choice that `lockChoice()` freezes.
  `acceptance_run.hpp` — `acceptanceRun`, the one routine the library ships: drive 24 in, turn
  90 degrees, face the start and drive back; not season content.
- **`include/shulib/ui/touch_panel.hpp`** (new directory) — the generic pieces of a screen model:
  the text budgets (held equal to the calibrate panel's at compile time), rectangles and padded hit
  boxes, `screenText`/`copyScreenText` (printable ASCII, no `%`), the release edge, the arming
  clock. `calibration/calibrate_panel.hpp` is unchanged.

**RE-WORDED (a guarantee stated more precisely, not weakened):** `localizer.hpp` said "there is no
code path … that ASSIGNS a heading"; it now says no CORRECTOR and no fusion policy can, and names
the one caller seed. Never-snap — what a sensor may do to a live estimate — is unchanged. The
claims that followed from the old sentence now name the seed too: the heading bias is exactly
zero, and the published heading is the raw IMU bit for bit, only on a tree with no
heading-providing corrector AND no `reseatHeading()` seed — so **`Localizer::headingBias()` is no
longer a pure drift measure once a competition auton has started** (it carries the start-heading
seed). `Localizer::Quality::Uninitialized`'s doc now says what the code has always done: the
estimate is never live on the FIRST `update()`, only from the second (behaviour unchanged).
`config::Route`'s docs (F8; wording only) and the VexBuilder verb table say that a `Start`'s
heading is the start heading a competition auton is seated to when the Start is `Fixed`.

**Hardware assumptions:** HA-05 is AMENDED (the start heading's one owner is the chosen auton's
start pose, applied at autonomous start through the localizer's bias — never the IMU's boot
heading); HA-161 is NEW (the belief that PROS kills its competition tasks on a mode change — the
reason the competition program will run its auton on a task of its own). See the
[hardware assumptions register](hardware-assumptions.md).

### Changed since 2.2 (2026-09-26) — an `ODO_STUCK` line names the check that fired and its values

> Written when it landed (chunk DIAG1). **No version bump — the decision, under
> [`version.hpp`](https://github.com/SHU-ROBOTICS/shulib/blob/main/include/shulib/version.hpp)'s
> policy:** every surface this touches — `localization::IOdometry`,
> `diag::HealthMonitor::Observations`, the fault line's text — is outside the
> [Freeze Register](roadmap.md#freeze-register), and the one frozen promise nearby (F6's fault
> policy: `ODO_STUCK` aborts the active motion by default) is unchanged. The odometry's
> VERDICT — which ticks are implausible — is bit-identical; only the words on the line grew.
> **Nothing below has run on a robot.** It exists because robot two raised
> `implausible odometry tick` twice on 2026-09-25 and the line could not say why — and the next
> one could end an autonomous.

**CHANGED — what you will see:**

- **The `implausible odometry tick` fault line says which check fired, with the numbers.**
  `fault=ODO_STUCK n=1 FIRST implausible odometry tick why=ROT dth=1.745rad h0=0.012 h1=1.757
  L=0.000in R=0.000in` — `why=` is one or more of `NONFINITE`, `ROT` (the heading moved more than
  π/2 in one tick), `TRAVEL` (a side or pod moved more than 36 in in one tick); `dth`, `h0`, `h1`
  are the tick's heading change and its start/end IMU headings in radians; `L=`/`R=` are the two
  drive sides' travel in inches (`fwd=`/`lat=` on a tracking-wheel robot). One line per episode,
  with the FIRST tick's values. `why=UNREPORTED` means the odometry gave no account; `+STALL`
  means the stall cross-check rose on the same tick (the stall-only line is unchanged). Read
  field by field in the guide's
  [Chapter 11](guide/11-reading-the-diagnostics.md#reading-an-odo_stuck-line).
- **What you must do:** nothing, unless you parse fault lines — the text after
  `implausible odometry tick` is new.

**ADDED:**

- **`diag/odom_tick_audit.hpp`** — `diag::OdomTickAudit` (the trust gate's account of one tick:
  three flags, five values, `tripped()`), `diag::OdomTravelLabels`, `diag::appendOdomTickAudit()`
  and `diag::kOdomAuditDetailMaxBytes` (140 — what the fault latch's line can carry whole).
- **`localization::IOdometry::lastTickAudit()`** — a new **pure virtual**. Both of the library's
  odometries implement it, and their verdict is now read FROM the audit, so the two cannot
  disagree. **What you must do:** if you wrote your own `IOdometry`, add it — returning
  `diag::OdomTickAudit{}` compiles and makes your lines read `why=UNREPORTED`; filling it from your
  own gate makes them say why. This is source-breaking for such a class; it is not a frozen
  surface, which is why it carries no version bump.
- **`localization::Localizer::lastOdomTickAudit()`** (forwards it) and
  **`diag::HealthMonitor::Observations::odomAudit`** (appended LAST, so every designated
  initializer still compiles; left default, the line reads `why=UNREPORTED`).
  `motion::tickHealthObservables` fills it, so `drive()`, every motion and the idle tick carry it
  with no change to your program.

### Added since 2.2 (2026-09-26) — mechanism buttons in driver control, and mechanisms built from the config

> Written when it landed. **No version bump:** nothing here grows a FROZEN surface — the mechanism
> seam (F11), the controller seam (F13) and the robot profile (F7) are all still unfrozen, `Chassis`
> (F6) and `Routine` (F10) are untouched, and `robot::Robot`, `config::RobotConfig` and
> `teleop::runDriverControl` are not in the [Freeze Register](roadmap.md#freeze-register), and
> neither is `hal::IDeviceProvider`. Some config rules are new and REFUSE configs they used to
> accept, and the device provider gained a member every implementer must answer — see below.
> **Everything here is proven on the host test plant only, and nothing has moved a mechanism on a
> robot.** (When this was written no program used it; since T3 Part 2, later the same day, "shulib
> Teleop" drives robot two's claws with it — still not run on a robot.) The pneumatic half (the toggle, the provider's 3-wire lines,
> pneumatic mechanisms from the config) landed the same day, from T3's addendum.

**ADDITIVE:**

- **`teleop::MechanismBinding`** (`teleop/mechanism_binding.hpp`) — two controller buttons driving
  one `hal::MotorMechanism`, hold-to-run: the forward button alone drives it at a STATED voltage
  (no default; in (0, 12] V) every tick, the reverse button alone at minus that, and neither, both,
  or a disconnected controller puts it in its declared safe state ONCE, on the transition — then it
  commands nothing until a button drives again. Releasing the button is the stop. An outside stop
  through the binding (`makeSafe()`) while a button is held leaves it inert until the driver lets
  go; a mechanism that something else has claimed is never commanded — including one an operation
  left claimed when the end of autonomous deleted its task, so a program whose autonomous commands
  a bound mechanism releases that claim before driver control. *(Superseded later the same day:
  `runDriverControl` now releases such a claim itself when it starts — "Changed since 2.2 (2026-09-26,
  later) — driver control releases a leftover claim", above.)*
  `teleop::validBindings(bindings, rezeroButton, why, cap)` checks a whole set (a button used
  twice, a button that is also the re-zero button, a mechanism bound twice) and names the problem;
  `teleop::controllerButtonWord()` prints a button.
- **`teleop::ToggleBinding`** (same header) — ONE button flipping one `hal::PneumaticMechanism`:
  each press flips the level the line was last commanded, exactly once (holding does nothing more);
  a press must be SEEN — a button already down when driver control starts, or when a dropped
  controller comes back, is not a press, so a press made while the controller was gone is not
  replayed (provided the controller's connected flag and its buttons come back together — not yet
  measured); the initial level is STATED (no default) and sent once, at the binding's first loop
  entry; and a disconnect, a reconnect or a field disable commands NOTHING — the claw keeps what it
  holds. It takes no claim and never commands a claimed mechanism.
  `teleop::validBindings(bindings, toggles, rezeroButton, why, cap)` checks hold bindings and
  toggles as ONE set (the hold-only form is the same check with no toggles).
- **`teleop::runDriverControl` takes the bindings** — a new overload with a
  `std::span<teleop::MechanismBinding* const>` after the loop config. Each binding is reset and made
  safe before the first tick, ticked after `drive()` with its buttons read only while the controller
  is connected, and made safe again on a field disable (after the drive's own stop). It refuses,
  before the first tick, a set `validBindings()` refuses. The old signature is the same loop with
  no bindings. **Watch the default:** `DriverLoopConfig::rezeroButton` defaults to L1, so a program
  that binds L1 must name another re-zero button. A second overload also takes a
  `std::span<teleop::ToggleBinding* const>` after the hold bindings: each toggle is entered before
  the first tick, ticked with its button read only while connected, and **left untouched on a
  field disable** (the hold bindings are made safe; the claw is not).
- **`config::MechanismConfig` gains `cartridge` and `safeMode`** (`config::SafeBrakeMode` —
  `Coast`, `Brake`, `Hold`, with the byte 0 meaning UNSET and no default), plus
  `config::safeBrakeModeWord()`, `config::isSafeBrakeMode()` and `config::toBrakeMode()` — and
  **`pneumaticSafe` and `pneumaticStart`** (`config::PneumaticLevel` — `Low`, `High`, the byte 0
  UNSET, no default), plus `config::pneumaticLevelWord()`, `config::isPneumaticLevel()` and
  `config::toLineValue()`.
- **`hal::IDeviceProvider::digitalOut(line, initialState)`** — the brain's own 3-wire ports, 1..8
  for A..H (`hal::kBrainAdiLines`). Making a line DRIVES it to the stated level (PROS's own
  constructor does), so the level is an argument; the same line again is the same object, never
  driven again; the same line at another level is a precondition. `hal::pros::ProsDeviceProvider`
  makes a `ProsDigitalOut` and can refuse only a line outside 1..8 — the brain's registry cannot
  see a solenoid, so nothing can tell it one is missing. `hal::fake::FakeDeviceProvider` drives its
  lines the same way and adds `bindDigitalOut()`, `fakeDigitalOut()`, `refuseLine()`/`allowLine()`,
  `digitalOutRequestCount()`, `digitalOutMade()` and `digitalOutInitialState()`.
- **`robot::Robot` builds the mechanisms** — one `hal::MotorMechanism` per mechanism with motors,
  its motors asked for with the MECHANISM's cartridge and signed ports, its declared safe mode, its
  config name. `Robot::mechanism(name)`, `mechanismAt(i)`, `mechanismCount()`,
  `mechanismAbsence(i)`, and `mechanisms()` — the `hal::IMechanism*` span an end-of-run guard
  walks. **A mechanism motor the device provider will not make leaves that mechanism ABSENT**, with
  the provider's sentence, and the robot still builds and drives; `validate()` does not ask for
  mechanism motors. **Since the addendum it also builds one `hal::PneumaticMechanism` per
  mechanism with 3-wire lines**: each line made at the config's start level (on a brain, the moment
  the Robot is built is the moment the claw takes it), the declared safe level, the config name —
  `Robot::pneumaticMechanism(name)` and `pneumaticMechanismAt(i)` find it (`mechanism(name)` stays
  the MOTOR lookup), and `mechanisms()` carries both kinds. A line the provider will not make leaves
  that mechanism ABSENT; `validate()` never asks for a line (asking would drive it). A robot with no
  mechanisms builds exactly the graph it built before.
- **`Robot::builtMechanismAt(i)`** — the `i`-th declared mechanism if it was built, of EITHER kind
  (a `hal::IMechanism*`). Because the other lookups are typed, `mechanismAt(i)` is null for a built
  pneumatic claw; a boot line or panel row that reports each declared mechanism asks
  `builtMechanismAt(i)`, and `mechanismAbsence(i)` is empty exactly when it is non-null — for an
  index past the declared mechanisms it now says "no mechanism is declared at this index" rather
  than the empty string that means "built".

**Behaviour you should know changed, without a signature moving:**

- **`RobotConfig::valid()` (and so `robot::validate()` and `Robot`'s constructor) now refuses a
  mechanism WITH motors whose `cartridge` or `safeMode` is unset**, a mechanism name with no NUL
  inside its 24 bytes, and two mechanisms with the same name — each by name. Port collisions are
  checked first and keep their messages. **Migration:** give every mechanism that has motors a
  `.cartridge` and a `.safeMode`; a purely pneumatic mechanism needs neither.
- **…and, since the addendum, a mechanism with 3-wire lines whose `pneumaticSafe` or
  `pneumaticStart` is unset, a mechanism with BOTH motors and lines, and a 3-wire line listed twice**
  (by two mechanisms, or twice by one) — each by name. **Migration:** give every pneumatic
  mechanism its two levels; split a mixed mechanism into a motor one and a pneumatic one.
- **`hal::IDeviceProvider` has a new pure virtual, `digitalOut()`.** Code that only CALLS a
  provider is unaffected; a class that IMPLEMENTS one must add it (the two in the library have).
  **Migration:** answer it — make the line at the stated level, or return nullptr and say why in
  `lastRefusal()`.

### Fixed since 2.2 (2026-09-26) — Brake and Hold now reach a V5 motor; the driver loop's release mode

> Written when it landed (chunk DEFECTS2). **No version bump — the decision, under
> [`version.hpp`](https://github.com/SHU-ROBOTICS/shulib/blob/main/include/shulib/version.hpp)'s
> policy:** no signature changed anywhere; `hal::IMotor` (Freeze Register row F4) already PROMISED
> what this fix delivers ("Hold is the active position hold the guaranteed park relies on"), and
> `Chassis::cancel()`'s documented safe state (F6: "0 V + Brake") is unchanged in the contract — it
> is the V5 adapter, an implementation outside every freeze, that now keeps them. The one new name,
> `teleop::DriverLoopConfig::releaseBrakeMode`, is on an unfrozen struct and its default keeps the
> stick-release behaviour a robot showed before. A behaviour change that makes an implementation
> match its frozen contract is neither the policy's BREAKING (the documented semantics did not
> move) nor its ADDITIVE (nothing frozen grew). **Nothing below has run on a robot**; the beliefs
> it rests on are A4 register HA-142–HA-146 in the
> [hardware assumptions register](hardware-assumptions.md).

**FIXED — what changes on a robot:**

- **Every Brake and Hold stop now applies its mode at the motor.** PROS applies a brake mode only
  through `pros::Motor::brake()`; `move_voltage()` "will not respect brake modes". Until now
  `hal::pros::ProsMotor` sent every zero as `move_voltage(0)`, so — by PROS's own documentation;
  nobody has watched it happen — on a V5 a mechanism declared `BrakeMode::Hold` went LIMP when its
  operation ended, and the drive's cancel safe state ("0 V + Brake") COASTED, through every path
  that uses it: `Chassis::cancel()` and its panic
  stop, a pre-emption, a fault abort, the `brake()` verb (`DriveBrake`), `RunGuard`'s forced-safe
  paths, and the driver loop's disable. Every host test passed throughout, because the fakes echo
  the mode and the host test plant pins a wheel at 0 V. Now a zero command (the millivolts SENT
  round to 0) under a Brake or Hold that the program set with `setBrakeMode()` goes out as
  `brake()` — once, on the way into the stop; the mode and the zero may come in either order.
  **Coast is unchanged**, byte for byte on the wire: a zero under Coast is still `move_voltage(0)`,
  so "shulib Calibrate", "shulib Drive" and the tester's DRIVE station behave exactly as before.
- **A motor your program never gave a mode is unchanged too.** The adapter keys the stop on the
  mode THIS program commanded, never on the mode the device remembers from an earlier program
  (which it still reads for `brakeMode()`'s fallback): a drive motor left in Hold by another
  program does not start holding at centred sticks.
- **A stop the port refused is retried, never believed.** `brake()` stops under whatever mode the
  motor's register holds, so if a port drops out as a mode or a stop is sent (PROS answers with an
  error), the adapter records no stop, re-sends the mode before its next `brake()`, and tries again
  at the next zero command. A caller that sends its zero only once — a mechanism operation's exit —
  gets no retry if the port is down at that instant, as with any refused command.
- **A mode, once set, governs every later zero on that motor — so it now matters on a robot.**
  The library's Brake stops (a cancel, the `brake()` verb, a pre-emption, a fault abort) leave the
  drive motors in Brake, and a later zero command that sets no mode of its own — a motion's settle
  stop, the zero volts a motion commands while it waits for the estimate — now BRAKES too, where
  before DEFECTS2 every such zero was a plain 0 V. That is `hal::IMotor`'s contract working as
  written, and it is why the driver loop now re-asserts its own release mode every tick (below).
- **What you must do:** nothing, unless your program relied on a Brake or Hold stop coasting —
  then set `BrakeMode::Coast` where you meant it. Expect a mechanism declared Hold to draw holding
  current at rest now (watch its temperature — HA-92), and expect the drive to brake on a cancel.

**ADDED:**

- **`teleop::DriverLoopConfig::releaseBrakeMode`** (default `BrakeMode::Coast`, the team lead's
  ruling of 2026-09-26). `runDriverControl` puts it on every drive motor at entry and again after
  every `drive()` — because a `drive()` that pre-empts a motion cancels it into Brake, and with the
  fix above that Brake would now make the next released stick brake the drive. The disable path is
  not governed by it: a disable still cancels into Brake. **What you must do:** nothing to keep
  coasting on release; set `.releaseBrakeMode = hal::BrakeMode::Brake` (or `Hold`) to make a
  release stop the drive.

**CORRECTED DOCUMENTATION:**

- `ProsMotor::faultedReads()` counts a screened `brakeMode()` read — it has since DEFECTS1 — but
  its comment and the class comment still said it did not. They now say five screened reads are
  counted.
- The `hal::BrakeMode` comments (`hal/motor.hpp`) now quote what PROS actually says each mode does
  and mark the rest as belief: which position Hold holds, and that Brake does not return to where
  it stopped, are not in PROS's text.

**For contributors:** the host PROS shim (`test/pros_shim/pros/motors.hpp`) gained `Motor::brake()`
and a per-port CONTROL STATE (`shim::MotorControl::Voltage` / `Stopped`, and the mode a stop was
issued under), so a test can tell "0 V, mode ignored" from "stopped, mode applied". Assert a stop
through that state, never through an adapter's `brakeMode()`, which read Hold all through this
defect. `MotorPortState::refuseSetBrakeMode` refuses the next N mode writes alone, for a dropout
that ends between two calls.

### Removed since 2.2 (2026-09-24, later the same day) — the calibrator's three retired names

> Written when it landed. **No version bump:** `calibration::DrivetrainCalibrator` is not in the
> [Freeze Register](roadmap.md#freeze-register); the calibration record's format and the boot check
> are unchanged. The section below retired these names and said they would be removed once the
> "shulib Calibrate" program no longer used them; that program now paints the screen model
> (`calibrate_panel.hpp`'s `viewFor()`) instead, so they are gone. **The rewritten program has not
> run on a robot.**

**REMOVED — what a caller must change:**

- **`Phase::ScalePush`** — the calibrator never entered it since the change below. **Migration:**
  delete the `case`; a `switch` over `Phase` now has seven values.
- **`Answer::Distance24`, `Answer::Distance48`, `Answer::Distance72`** — never accepted since the
  change below (there is one push, of `CalibratorConfig::pushDistance`). **Migration:** delete the
  distance buttons; the answers are `Done`, `FrontFirst`, `BackFirst` and `Abort`.
- **`PortObservation::tapeDeltaRad`** — always 0 since the change below. **Migration:** read
  `pushTravelRad` (a drive port's travel over the one push, sign-corrected) or `pushDeltaRad` (as
  read).

### Changed since 2.2 (2026-09-24) — the drivetrain calibrator rebuilt after its first night on a robot; a pure model of its screen

> Written when it landed. **No version bump:** `calibration::DrivetrainCalibrator` is not in the
> [Freeze Register](roadmap.md#freeze-register), and nothing frozen moved — the calibration record's
> format is unchanged (`kRecordVersion` is still 1) and so is the boot check. **The calibrator's
> public surface DID change**, so this section says what a caller must do. Why it changed: on
> 2026-09-24 the calibrator ran five times on a real robot and never finished — its one 3 V
> single-motor pulse was too weak for a five-motor gear train, its screen offered DONE before
> anything was pushed, and its host tests had passed on a test plant that could not fail the way the
> robot did. **Everything below is proven on the host test plant only; none of it has run on a
> robot.**

**CHANGED — what a caller must change:**

- **One push instead of two.** The operator pushes the robot forward `CalibratorConfig::pushDistance`
  (48 in) along a tape ONCE, then answers FRONT or BACK; that push gives the drive ports, their signs
  and each side's scale. There is no distance question: `Answer::Distance24/48/72` are **retired** —
  `accepts()` never accepts them and `answer()` refuses one as out of order — and `Phase::ScalePush`
  is **retired** (never entered). `PortObservation::tapeDeltaRad` is **retired** (always 0); read
  `pushTravelRad`. The three retired names still compile so the current "shulib Calibrate" program
  builds; **they will be removed** when that program is rewritten.
  **Migration:** drop the distance buttons and the `ScalePush` case; offer DONE only when the new
  `pushReady()` is true (see below).
- **`CalibratorConfig::pulseVolts` and `spinVolts` are gone**, replaced by ramps:
  `pulseLevels` / `pulseLevelCount` (3, 6, 9 V) and `spinLevels` / `spinLevelCount` (3, 6 V). New
  fields: `restYawRateFloor`, `decoupledOwnMin`, `pushDistance`, `spinStartTime`, `spinStartYaw`,
  `spinWrongWay`. A config whose ramp is not strictly increasing inside (0, 12] V, or whose
  `restYawRateFloor × pulseTime` is not under `pulseYawFloor`, is refused at construction.
  **Migration:** a config that set `pulseVolts = X` sets `pulseLevels[0] = X` (and the count), or
  keeps the defaults.
- **A drive port whose motor turns but moves nothing (a slipping gear) is LEFT OUT** of the
  description with a warning (`warning()`, `report().excludedPorts`), and the run can still finish:
  a description may now name fewer ports than the push moved. The boot check already treats such a
  present-but-unrecorded motor as a warning.
- **`status().prompt`** may now point into a buffer the calibrator owns (the push instruction names
  the configured distance); it is valid until the calibrator's next `tick()`, `answer()` or
  `abort()` — copy it if you keep it longer. Every prompt is at most 44 characters.
- **Behaviour you should know changed, without a signature moving:** rest between pulses now also
  requires the IMU's yaw rate under a floor; the side pulse ramps and stops at the first level that
  turns the robot; the spin refuses at once if it turns the WRONG way (15°), and steps up once and
  then refuses if it cannot start turning (instead of stalling 20 s); the spin-disagreement refusal
  says "pct" instead of the percent sign (PROS re-reads screen text as a format); the copy
  constructor and copy assignment are now explicitly deleted (they were already ill-formed).

**ADDITIVE:**

- **`calibration::Refusal`**, `refusal()` and `refusalPort()` — a code (and the port, where there is
  one) for every way a run can stop, so a screen can explain a stop without parsing prose;
  `refusalName()` for logs.
- **`pushReady()`** — the DONE gate, built from the same predicates DONE is judged on.
- **Numbers in `CalibratorStatus`** (moved count, push travel and far-enough, at-rest, drift, pulse
  ordinal/port/level, live yaw, spin direction/turns/level, the side counts, left-out count, seconds
  in the phase) and **a serial record** in the report — one `PulseRecord` per pulse at every level,
  one `SpinRecord` per spin direction — with `formatPushRecord()`, `formatPulseRecord()` and
  `formatSpinRecord()` printing each as one ASCII line.
- **`calibration/calibrate_panel.hpp`** — the calibration screen as a pure, host-tested model:
  `viewFor()`, `hitTest()`, `TouchGate`, `wrapWords()`, `stoppedWords()`, with compile-time checks
  that every piece of text fits its (estimated) line budget, is ASCII and carries no percent sign.
  No program uses it yet.

### Added since 2.2 (2026-09-24) — the on-robot device provider, dead ports in the builder, the port map, the driver-control loop

> Written when it landed. **No version bump, and none owed:** everything here is additive, and none
> of it grows a FROZEN surface — `IDeviceProvider` and `robot::Robot` are not in the Freeze
> Register, `Chassis` (F6), the route and runner (F8), `Routine` (F10), the HAL interfaces (F4) and
> the kinematics (F5) are untouched, and `motion::ITickPacer` gained nothing. **Nothing on a brain
> uses any of it yet** — no program in `src/` changed; "shulib Teleop" moves onto it in a later
> step — and everything below is proven on the host only: the provider through the host's
> stand-in for the PROS headers, the builder and the loop on the host test plant.

**ADDITIVE:**

- **`hal::pros::ProsDeviceProvider`** (`hal/pros/device_provider.hpp`) — the on-robot
  `IDeviceProvider`: real adapters made on first request, one per port, owned for the life of the
  program, with no heap. **Before it constructs anything it asks the brain's port registry what is
  plugged in**, and refuses an empty or wrong-kind port with a sentence naming the port, what it
  found and what it expected (`port 2: a rotation sensor (expected an IMU)`); a motor that is
  present but rejects its configuration is refused with the adapter's own reason. Asking again for
  a port returns the same object; asking with a different sign, cartridge or GPS lever arm is a
  precondition violation. `imu()` starts the IMU's calibration once, itself. (New hardware belief:
  HA-141, [register](hardware-assumptions.md).)
- **Dead drive ports in the builder:** `robot::BuildOptions{ .absentDrivePorts = … }` — port
  numbers the boot check found dead — accepted by `robot::validate(cfg, devices, options)` and
  `robot::Robot{cfg, devices, options}`. Each becomes an ABSENT member of its side's
  `hal::MotorGroup` and is never asked for; the list is refused, naming the port, if a port in it
  is not a drive port, if the drivetrain is not a tank, or if it would empty a side. The builder
  applies no dead-port policy of its own. **Default options build exactly what 2.2 built.**
  `Robot::describe()` adds `DEGRADED: absent …` when a port is absent (only then).
- **The session header's port map, generated:** `Robot::portMap(out, cap)` and the free function
  `robot::writePortMap(config, out, cap)` — the text `diag::SessionInfo::portMap` carries, in the
  same format the programs in `src/` print by hand today, bounded and always terminated.
- **`Robot::driveOdometry()`** — the drive-encoder odometry, for a panel's cross-check row
  (`nullptr` on a tracking-wheel robot).
- **`teleop::runDriverControl`** (`teleop/teleop_loop.hpp`) — the driver-control loop, PROS-free:
  a disable check that stops the drive into its safe state and returns, all four stick axes read
  only while the controller is connected, `DriveShaper`, the chassis's speed budgets,
  `drive(…, Frame::Body)`, a re-zero button with its own edge, a per-tick hook for a panel. The
  tick period, the disabled check and the heading source are arguments
  (`teleop::DriverLoopConfig`, `teleop::HeadingSample`).
- **`hal::fake::FakeDeviceProvider::motorRequestCount(port)`** — counts every request, refused ones
  included (test infrastructure).

**Behaviour you should know changed, without a signature moving:**

- **`teleop::FieldCentric::rezero()` (and so `DriveShaper::rezero()`) no longer accepts a heading
  that is not a finite number.** It used to store it as the zero and clear the fallback latch, after
  which the next trusted tick produced a NaN command that `Chassis::drive` refuses by precondition.
  Now the previous zero is kept and the fallback latches, so the controls stay robot-centric until
  the driver re-zeroes on a real heading. Every finite heading behaves exactly as before.
  **Migration:** none.

### 2026-09-18 — routines as data, a robot built from data, driver feel, and self-calibration; F8 frozen

> *Written 2026-09-24, late.* The bump to 2.2 landed on 2026-09-18 without this section, against
> this file's own promise. It covers everything user-visible between 2.1 (2026-08-15) and 2.2,
> reconstructed from the commits and the headers' own notes. **None of it has run under the
> library's motion stack on a robot** — every closed-loop claim below is proven on the host test
> plant.

**Nothing frozen changed shape.** Every change is additive except one tightened precondition,
listed first because it is the one a compiler will not show you.

**Behaviour you should know changed, without a signature moving:**

- **`motion::MotionDeps` now requires the drive-motor count to EQUAL the kinematics' wheel count**
  (it accepted *more* motors than wheels before). The old rule let eight raw motors sit on a
  two-wheel tank kinematics with six of them never commanded — no voltage, no brake mode, no
  fault. **Migration:** put the N motors on one side behind one `hal::MotorGroup` and hand the
  group to the context as that wheel's motor. A program that passed extra motors now refuses at
  construction with a sentence saying exactly this. **Under the version policy this is a breaking
  change** — the old documentation made "at least as many motors as wheels" valid, a program
  relying on it no longer runs, and `MotionDeps` is what the frozen `Chassis` constructor takes —
  and it shipped (2026-09-14) without a major bump or a migration note. Whether to bump, or to
  rule it a defect fix as the 2.1 entry did for unfrozen surfaces, is an open ruling for the
  project lead; this entry will say which when it is made.

**ADDITIVE:**

- **Routines as data (the change that bumped the minor and LOCKED register row F8):**
  `runner::PathRunner` runs a `config::Route` through the facade's verbs, `runner::CommandRegistry`
  binds a marker's command id to your code (`registry.on("intake_in", fn)`), and
  `runner::writeManifest` exports the id vocabulary for a path editor. `config/route.hpp` gains
  the id rules and the four library-owned built-ins (`wait_ms`, `brake`, `hold_ms`, `log`);
  `Route::valid()` gains rules and **relaxes one** — a `MoveToPose` may now use
  `HeadingMode::Free` ("keep the heading you have"). Relaxing is the additive direction: every
  route valid under 2.1 is valid under 2.2. On a tank, a route must carry its own turns — the
  facade never plans a turn you did not ask for.
- **A robot described as data and built from it:** `config/robot_config.hpp` (`RobotConfig`),
  `hal/device_provider.hpp` (`IDeviceProvider`, devices by port) with a host fake, and
  `robot/robot.hpp` (`robot::validate()` and `robot::Robot`, which owns the whole graph). The PROS
  device provider was not built at 2.2, so no program on a brain used this path. *(It has existed
  since 2026-09-24 — the entry above — and still no program on a brain uses the path.)*
- **Coupled drive motors:** `hal::MotorGroup` (N motors as one, median reads, a member may be
  ABSENT), `hal::DriveGeometry`, and `diag::FaultCode::MotorGroupDisagree = 12` — appended, per the
  append-only rule — raised when one member of a group persistently disagrees with its mates.
  `MotionDeps` gains `motorGroups` so the fault fires in `drive()` and in every motion.
- **Odometry from the drive motors:** `localization::IOdometry` (the seam) and
  `DriveEncoderOdometry`. `Localizer`'s constructor now takes an `IOdometry&` rather than a
  `PilonsOdometry&` — every existing call still compiles, because `PilonsOdometry` implements it.
  `MotionConfig`'s stall check gains per-wheel geometry and an `independentMotionSource` flag
  (off when the odometry IS the drive encoders, since a check against itself proves nothing).
- **Drivetrain self-calibration:** `hal::DrivetrainDescription`,
  `calibration::DrivetrainCalibrator` (the robot measures its own sides, signs, scales and track
  width), `calibration/calibration_record.hpp` (a 180-byte, CRC-checked record for the SD card)
  and `calibration/boot_check.hpp` (a record verified against the ports before any adapter
  exists); `hal::IBlockSource` and `hal::IDeviceRegistry` with their PROS adapters and fakes.
- **Driver feel:** `teleop/` — `input_shaping.hpp` (continuous deadband, expo), `slew_limiter.hpp`,
  `drive_modes.hpp` (arcade body/field, tank sticks), `field_centric.hpp` (with a latched fallback
  on an untrusted heading), `driver_control.hpp` (`DriveShaper`), plus `stick_mapping.hpp`,
  `coupled_side_monitor.hpp` and `drivetrain_degradation.hpp`. **No program uses the shaper yet.**
- **Absent devices:** `hal::AbsentGps`, `AbsentTagSource`, `AbsentVision` — for a robot without
  them, never polled, with no fake in a competition build.

## API 2.1

### 2026-08-15 — 59 defect fixes, mostly additive, with five surface changes to know about

An audit chunk (DEFECTS1) triaged the 84 API defects the documentation pass had reported and
left in place, and fixed 59 of them. **Most are invisible to a caller doing nothing wrong** —
added preconditions, corrected counters, corrected comments. Five change something a compiler
or a caller can see, and one of those is breaking under the version policy.

**BREAKING (unfrozen surfaces, no consumer in the tree):**

- **`control::TrapezoidProfile::isDone()` is no longer `noexcept`**, and both it and `sample()`
  now REJECT a non-finite `t`. `sample(NaN)` used to return a *partially* finite state —
  position and velocity NaN, acceleration a perfectly finite `-aMax` — so a caller screening
  only `acceleration` passed it and forwarded a plausible-looking down-ramp; `isDone(NaN)`
  returned false, so a follower loop terminating on it spun forever. The `noexcept` drop is
  required because the precondition handler throws and a `noexcept` frame would turn a caller
  bug into `std::terminate`. **Migration:** screen your clock, or catch `PreconditionError`.
  This class has no consumer in the library but its own test.
- **`hal::IMechanism` is now non-copyable and non-movable.** It holds the claim token as value
  state, so a copied mechanism arrived already `claimed()`, with `claimant()` pointing at an
  operation registered against the original. **Migration:** construct a mechanism where it lives
  and hand out `IMechanism&` / `IMechanism*` — which is what the header already told you to do.
- **`localization::EkfFusion::state()` and `covariance()` are no longer `noexcept`**; both now
  bounds-check. They are observability accessors, and an out-of-range index was silent UB.

**ADDITIVE, but you may notice:**

- **`diag::MotionOutcome::Unset = 5`**, appended and value-pinned, is the new default for
  `MotionResult::outcome`. An unpopulated result line used to render `✓ SETTLED` — the one
  value meaning success — for a motion that never happened; it now renders `✗ UNSET`.
- **`ProsDigitalOut(smartPort, adiPort)` with the boot state forgotten no longer compiles.**
  It used to select the brain-ADI constructor with `initialState = (bool)2 = true` and fire a
  solenoid HIGH at boot on the wrong port. **Migration:** state the boot level, which the
  required third argument always intended.

**Behaviour you should know changed, without a signature moving:** a `MotionScheduler` destroyed
with a motion still armed now forces the drive to its safe state instead of leaving the motors
at their last command; `MotionConfig` and `Watchdog` reject an *infinite* timeout (`> 0.0` is
satisfied by infinity, and an infinite watchdog can never expire); `Localizer` rejects a `minDt`
that is non-positive or not below `maxDt`; and a stale, finished mechanism operation can no
longer `cancel()` a mechanism a different live operation now owns.

Full triage with evidence for every item, including the 15 findings that did **not** hold and
the 6 that need a decision before they can be fixed, is in the development record.


### 2026-08-14 — the API reference now covers the whole public API — no API change

No library code changed: not a signature, not a default, not a behaviour. This entry is here
because what the documentation *promises* changed, and because one of those promises is
enforced by a build gate you will meet if you contribute.

**The reference went from 2 documented types to every public entity in every shipped header** —
117 pages covering **1,625** types, members, nested types, free functions, namespace-scope
constants and type aliases, plus an [A–Z index](api/all-entities.md) of all of them. The
generator's target list is now a glob over `include/shulib/`, so a header added later is covered
the moment it exists rather than when someone remembers to list it.

**The coverage gate grew with it.** A public entity anywhere under `include/shulib/` with no
`///` documentation now fails the build, naming the entity, its file and its line. If you are
adding to a shipped header, that is the one new obligation: say what it is *for*.

**Being documented does not freeze anything.** The distinction matters and is easy to
mis-read: a change to a *frozen* signature (`Chassis` = F6, `Routine` = F10) fails a
compile-time pin that names the register row, and costs a major version bump plus a migration
note. A change to a documented-but-unfrozen seam costs one comment edit and a regeneration.
Freeze Register rows F11–F14 previously said those seams were deliberately ungated for exactly
this fear; all four are **amended** rather than silently overridden, and their freeze triggers
are unchanged.

**Three locked contracts were not in the reference at all**, and nobody could have known from
reading it: the generator was structurally blind to types with a base-class list, enums with an
explicit underlying type, and everything declared at namespace scope — so the coordinate frame
(`math::Frame` and its two conversions), the accuracy targets (`spec/accuracy.hpp`) and the
units vocabulary (`units::Length`, `Time`, `Voltage`, …) produced no output whatsoever. They are
there now.

**The site is easier to move around**, which matters more at 117 pages than at 2: the top-level
areas are tabs, subsystem groups collapse, each section's overview is its own landing page, and
each header's design commentary folds when it runs long. The nav is generated from the same list
as the pages and byte-checked, because a page missing from the nav was measured to publish
*unreachable*, with exit code 0 and an INFO line.

**Breaking:** nothing. **What you must do:** nothing, unless you contribute to a header — then
write a `///` on anything public you add.

### 2026-08-14 — the mechanism-sensor adapters land (`hal/pros/`, second half) — additive

The other half of the hardware binding: PROS-backed adapters for the mechanism seams —
`ProsDistance`, `ProsOptical`, `ProsDigitalOut`, `ProsDigitalIn`, and `ProsBlockSink` (the
SD-card device behind the blackbox). Each applies its unit conversion exactly once at the
edge, through two new pure conversion headers (`distance_conversion.hpp`,
`optical_conversion.hpp`).

New HAL seam, **not frozen**: `IDigitalIn` (one member — the raw level of a digital input
line; no debouncing, no edge state, no validity channel, each by documented ruling) with
`hal::fake::FakeDigitalIn`. It is an additive sibling outside the F4 freeze, exactly as
`IDigitalOut` and `IController` were; the Freeze Register records the non-freeze out loud
(row F14). It was built ahead of its consumer — the lift-homing switch-or-stall question is
still open — and the register row says that too.

Three behaviours worth knowing before you use these:

- **The distance sensor's "no object" is 9999 mm, in-band.** PROS reports an empty field of
  view as a plain reading that converts to a plausible-looking 393.66 inches — not an error.
  `ProsDistance` maps it to `confidence() == 0.0` (the seam's documented "no usable return"
  channel) and keeps `distance()` finite and far. Threshold `confidence()`, always — the FAQ
  entry "Why does my distance sensor read 393 inches?" walks it.
- **Constructing a `ProsDigitalOut` PHYSICALLY DRIVES the line.** PROS actuates the port at
  construction and defaults it LOW; on a pneumatic that moves the cylinder at boot. The
  adapter refuses the default — the initial state is a required constructor argument, and it
  must agree with the owning `PneumaticMechanism`'s declared safe state (a test pins the
  pattern).
- **A missing SD card does not stop the robot.** `ProsBlockSink` constructs successfully with
  no card, refuses every write (`write()` returns false from the first call — the blackbox's
  drop-and-count design absorbs it), and reports the fact once through `isOpen()`.

**Breaking:** nothing. Every frozen surface (F3/F4/F6/F10) is untouched; the adapters are new
files implementing existing interfaces, and `IDigitalIn` is additive.

**What you must do:** nothing, unless a mechanism uses these sensors — then construct the
`Pros*` adapters and read the seam headers' design notes (each names which PROS call it
binds, which conversion it applies, and which Hardware-Assumptions entries its beliefs rest
on — HA-113 onward, none yet measured on hardware). Scope, unchanged from R1a: these
are host-tested against the programmable PROS stand-in, and **the library has still never
driven a robot.**

### 2026-08-13 — first hardware validation of the adapters — no API change

No code changed. Recorded here because it changes what the library's claims are worth.

The `hal/pros/` adapters were run against a physical V5 brain and a real robot for the first time.
Every unit conversion they perform was checked against a turning wheel:

| Conversion | Expected | Measured |
|---|---|---|
| motor degrees → radians | 57.2958 | 57.296 |
| motor RPM → rad/s | 9.5493 | 9.549 |
| motor mA → amps | 1000 | 1000.0 |
| battery raw → volts | — | 13039 → 13.04 V |
| battery capacity → [0,1] | — | 91.0 → 0.91 |
| `micros()` per 1000 ms | 1000000 | 999784 |

Seven previously-guessed hardware assumptions are now measured observations rather than reasoning.
The battery unit was the weakest of them — PROS's vendored headers document no unit for
`battery_get_voltage()` at all.

**What this does not mean:** the library still has never driven a robot. Nothing closed a control
loop, nothing followed a path, and no wheel turned under motion control. These measurements
establish that the platform layer reads and commands real hardware correctly, and nothing more.

### 2026-08-13 — the PROS hardware adapters land (`hal/pros/`) — additive

The library's first hardware binding: header-only adapters under
`include/shulib/hal/pros/` implement the frozen HAL interfaces over real V5 devices —
`ProsClock`, `ProsMotor`, `ProsRotation`, `ProsImu`, `ProsGps`, `ProsBattery`,
`ProsCharSink`, `ProsLineDisplay`, `ProsController`, plus `ProsTickPacer` (the real
10 ms tick over the PROS scheduler). Each adapter applies its unit conversion exactly
once, at the edge, through new pure conversion headers (`motor_conversion.hpp`,
`rotation_conversion.hpp`, `controller_conversion.hpp` — joining the existing IMU and
GPS ones).

New HAL seam, **not frozen**: `IController` (normalized [-1, 1] axes, button levels, a
positive `isConnected()` signal, master/partner support) with `ButtonEdge` for
per-consumer press detection and `hal::fake::FakeController` for tests. The Freeze
Register records the non-freeze out loud (row F13).

**Breaking:** nothing. Every frozen surface (F3/F4/F6/F10) is untouched; the adapters are
new files implementing existing interfaces.

**What you must do:** nothing, unless you want your robot code on real hardware — then
construct the `Pros*` adapters instead of fakes (the shipped `src/main.cpp` is the worked
example, and guide chapter 7 walks it). Scope: these adapters are
host-tested against a programmable stand-in for PROS, and **the library has still never
driven a robot** — the beliefs behind every conversion are catalogued in the
[Hardware Assumptions Register](hardware-assumptions.md) (HA-94 onward) and get their
first reality check on a bench, not in a test suite.

### 2026-08-13 — a guaranteed end of run *(reconstructed)* — additive

`sequence/run_guard.hpp` is new: `RunGuard`, plus `RunGuardConfig`, `RunGuardReport` and
`GuardedWaitResult`. It is an `ITickPacer` **decorator** — construct it around the real pacer
and hand it to the chassis — and it is completely **inert until `run()`**.

At the instant you name, it cuts the active motion with zero latency, then **refuses every
later motion**. The refusal is a *latch*, not a pause: cancel-only expiry was measured inert
and marginally counterproductive, so once the deadline passes nothing new starts. Cancel-all
runs strictly before your end action, and an unconditional `hardStopAt` floor fires regardless.

**The library refuses to know your strategy.** No field coordinate, no park pose, no default
lead time, no default match length — both instants and the end action are yours. A team that
ends somewhere else, or does not park at all, is not fighting the library.

**Four limits, stated because they bind:** it proves a *scheduling* property, not a timing
margin on a real brain (loop rate and PROS call latency are still invented register entries);
**nothing preempts pure user code** — there are no background tasks anywhere in this library, so
a `while (true)` loop that never lets a call return keeps the CPU and your end action waits;
the frozen `pause`/`waitFor`/`waitUntil` surfaces cannot see the deadline and pay their full
remainder (a tested limit, not a hidden one); and **"zero travel after the deadline" is a
simulator result** — the host plant is memoryless, so on a robot with mass the claim is
*"no new commanded motion"*. Budget lead time for a robot that is still moving when the command
stops.

**Breaking:** nothing. New header, new types; no frozen surface touched.

**What you must do:** nothing, unless you want the guarantee — then wrap your pacer. Note that
the routine-level watchdog (diagnostics item D-8) is the same primitive under a second policy.

### 2026-08-13 — mechanism seam growth *(reconstructed from `version.hpp`)* — additive, 2.0 → 2.1

The mechanism layer grew through the documented additive paths: `RoutineStopCause`
gained an appended `MechanismFailed` enumerator, `Routine::then()` accepts a fourth
return type (`manipulation::MechanismOutcome`), and `FaultCode` appended
`MechanismStalled`. No frozen member changed shape.

**What you must do:** nothing. If you `switch` exhaustively over `RoutineStopCause` or
`FaultCode`, add the new cases.

## API 2.0

> The four entries below all landed **after** the 2.0 freeze and **before** F1 opened 2.1. None
> of them moved the version number, and that is the policy working rather than an oversight:
> `version.hpp` tracks the growth of *frozen* surfaces, and these four added entirely new
> headers and types outside `Chassis` and `Routine`. Nothing previously compiling changed
> meaning. They are reconstructed here because they are user-visible surface an outside team
> would otherwise meet only by reading the source.

### 2026-08-13 — a Kalman tier, which is **not** the default *(reconstructed)* — additive

`localization/ekf_fusion.hpp` adds `EkfFusion` and `EkfFusionConfig` — a 5-state SE(2) extended
Kalman filter behind the unchanged `IFusionPolicy` seam. It can do three things the simpler
filter cannot: weigh two disagreeing correctors against each other by their stated accuracy
(matched against the inverse-variance weighted mean to better than 0.2%), recover from a
displacement the fixed gate makes permanent, and state its own uncertainty as a number.

**`ComplementaryFusion` remains the shipped default, and the measurement is why.** Raced over
eight seeded 60-second runs the EKF finished *less* accurate — 0.351″ vs 0.225″ mean final
error, losing on 7 of 8 seeds. It lost the comparison its own definition-of-done asked it to
win, and that result was kept rather than tuned away. In this simulation dead-reckoning is
already sub-inch and the modelled GPS is noisier than the drift it corrects, so the right move
is mostly to ignore the sensor — which a blunt fixed gain does slightly harder. **Anyone
quoting this as an accuracy improvement is quoting it wrong.** Every noise parameter in it
(HA-83…HA-91) is invented, so real sensor characterization could flip the result either way.

**Breaking:** nothing. **What you must do:** nothing — you get the complementary filter unless
you ask for the EKF.

### 2026-08-13 — absolute heading from AprilTags *(reconstructed)* — additive

`localization/apriltag_corrector.hpp` adds `AprilTagCorrector` and `AprilTagCorrectorConfig`;
`localization/tag_map.hpp` adds `TagMap`, `TagPlacement` and `TagProvenance`. This is the only
source of **absolute heading** in the library — a tag whose field position is known tells the
robot which way it is actually pointing, and it proposes with `providesHeading = true`.

**shulib ships no tag map, deliberately.** Nobody on the project can cite a table of AprilTag
field poses, so inventing one would be a confidently wrong answer rather than a missing one.
`TagMap::add()` refuses a placement whose provenance is `Unspecified` — `Invented` is a
perfectly legitimate answer, *unstated* is not — and an unknown tag id makes the corrector
decline with `RejectedNoTagMapEntry` rather than guess.

**Two things worth knowing before you trust it:** a map that is two inches off produces a
corrector that is *confidently* two inches wrong, and unlike noise that error does not average
out (HA-68). And a **reversed corner winding is catastrophic and silent** — it mirrors the tag,
puts the recovered heading 180° out, and leaves the solver's own reprojection error reading
machine zero. No software self-check can see it; only a physical tag can (HA-69).

**Breaking:** nothing. **What you must do:** nothing, unless you add a camera — then you must
supply the tag map, with provenance.

### 2026-08-12 — GPS position correction *(reconstructed)* — additive

`localization/gps_corrector.hpp` adds `GpsCorrector` and `GpsCorrectorConfig`, the first real
`ICorrector`: it bounds position drift while the field strip is in view. It corrects **position
only** — `providesHeading` stays false, so the IMU keeps owning heading.

The accuracy claim is sized to the evidence rather than rounded up: over eight seeded 60-second
runs the corrected estimate was better on 7 of 8 on final error and 6 of 8 on worst-case. A
shorter scenario that passed 8 of 8 was found and **deliberately not adopted**, as scenario
shopping.

**Driving Skills has no GPS strip**, so `hasFix()` is false for that entire event and the
estimator dead-reckons — a normal operating state, not a fault. The FAQ entry
"Why does `hasFix()` go false in Driving Skills?" is the full version.

**Breaking:** nothing. **What you must do:** nothing, unless you have a GPS.

### 2026-08-12 — the SD-card blackbox *(reconstructed)* — additive

`diag/sd_sink.hpp` adds `SdSink` (an `ITelemetrySink` writing through the `IBlockSink` device
seam) with `SdSinkConfig`, `SdSinkStorage` and `SdSinkBuffers`; `diag/blackbox_format.hpp` adds
the versioned, session-stamped, fixed-width binary format; `diag/blackbox_reader.hpp` adds
`BlackboxReader` and `ReadStatus`. **A run becomes recoverable after the fact, with no laptop.**

Two design points that are the whole value: the **decoder ships in the same chunk as the
encoder** — a format nothing can read is not a record — and it is held to the format by
byte-exact goldens rather than by agreeing with the encoder, so the two cannot drift into
agreeing on something wrong. And the default posture is a **RAM flight recorder** that costs the
card nothing until a fault fires, at which point it writes the triage block **first** and the
preceding ticks after it — in that order, because the fault may be the brownout that cuts the
write short.

**Breaking:** nothing. **What you must do:** nothing. If you want it, give `SdSink` a block
device; the V5 adapter for that arrived later (see the 2026-08-14 entry) and a missing card is
handled rather than fatal.

### 2026-08-12 — the v2 facade freezes *(reconstructed)* — the 2.0 baseline

The public `Chassis` API froze (Freeze Register row F6), followed the same day by the
`Routine` recipe layer (row F10). From this point, routines written against either tier
do not need rewriting: signatures and documented behaviour change only with a major
version bump and a migration note, enforced by compile-time signature pins that fail the
build if a frozen member drifts.

**Breaking, relative to everything before it:** shulib v2 is a ground-up rebuild; the
legacy v1 tree was deleted when `src/main.cpp` was rewired onto the v2 core. There is no
v1→v2 migration path — v1 was never released beyond the team.

**What you must do:** new users start at the [guide](guide/README.md); the
[cookbook](cookbook/README.md) and the [API reference](api/README.md) are the day-to-day
documents.
