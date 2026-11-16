# MIRROR1 — ONE ROUTE, TWO ROBOTS: robot two runs robot one's auton MIRRORED across the field's skills diagonal — every field number reflected at the Chassis's front door for exactly that run, the start pose seated reflected, the wall reset's wall and sensor side swapped, the Toggle step's Toggle swapped, everything else unchanged, and a step that cannot be mirrored refused by name

> **THE TEAM LEAD'S ANSWER (2026-11-13 ~03:30): "do what is best they should play onto one side when they need to when mirrored ,
> do what you need to do"** — GO on MIRROR1-A with D1–D11 as recommended after the re-check, and the coordinator's calls on
> the open points: **D11** — each robot plays its OWN side: the shared route keeps robot one's whole robot ≥ 3 in off the
> diagonal; anything ON the diagonal (the centre goal, the two yellow Pins there, the corner Load Zone) is taken by ONE robot
> through `isMirrored(ctx)`; the host pair test FAILS, naming the step, under a 6 in gap. **D2** — robot two's starter autons
> file registers the shared skills route MIRRORED (one line, plumbing like robot one's starter registering the 40 in auton;
> marked the team's to change; it lands with ROB1-BT or MIRROR1-A, whichever merges second); the ROUTE itself stays the
> students' (G4).

> **Status 2026-11-13 (written ~01:00–01:42; re-checked ~02:00–03:00 — "Re-check — amendments" below; *[R13] the first
> print said "~01:00–04:00" and §6 "re-checked ~03:30", times that had not yet happened*): DRAFT for the team lead's
> go. WAITS for his answers to D1–D11 below, then for a go for the one run, MIRROR1-A. Nothing is built.** Written by
> the coordinator's brief writer from a READ-ONLY pass.
> **Base:** every `file:line` below was OPENED at main **`d33d25d`** (CURVE1-A) unless marked **@8df58f3** — STEP1-B
> was COMMITTED as `8df58f3` (API 2.8) while this was being written (`git log`), and it moved `routine.hpp` and
> `chassis.hpp` lines; the main worktree's INDEX now carries **VIS1-A2's squash, staged** (camera files, guide
> chapter 20; no file this brief changes). Worktrees read, never edited (§6.1): `af1s-a1`, `rob1-int`, `rpv1-a`,
> `rob1-a`, `g3a-p1`. **Probed on the host test plant, not argued:** four probes in
> `.claude/coordinator/scratch/brief-MIRROR1/` (§0.4) — *[R1, R4] six since the re-check: P5 (the two robots at once)
> written by this writer from the checker's probe, P6 the checker's trigger probe re-run —* over `tree/` = `git archive 8df58f3`, every binary under
> `(ulimit -v 4194304; timeout 300 …)`. The manual read for this brief: v2.0's text (`scratch/autoflip/v20.txt`) and its
> drawings rendered fresh (`scratch/brief-MIRROR1/manual/`: `a16-111.png` AprilTags, `vurs1_field.png` the VEX U skills
> field, `p87-087.png`), plus `scratch/autoflip/img/a-105.png` (A10) and `a-106.png` (A11); v2.1's `<SG1>` and `<VURS1>`–
> `<VURS3>` re-read ONLINE on 2026-11-13 (manuals.vex.com/v5/override/v2.1/game/ and /vex-u/): unchanged in substance;
> v2.1's Appendix A lists the same sheets and states no change (its drawings were not re-measured).
>
> **THE TEAM'S SITUATION AND THE TEAM LEAD'S WORDS THIS BRIEF HOLDS (BINDING), as given to this writer (2026-11-13):**
> - First competition **Nov 21, SKILLS ONLY; BOTH robots run the skills route.**
> - The team lead: **"robot 1 and robot 2 are the same build and just the auton routes are different"**,
>   **"IT IS LITERALLY THE SAME ROBOT BUILD JUST A MIRRORED ROUTINE"**, **"we can do everything including the second
>   robot do not sell yourself short"**, and **"i really want the route maker and/or panel so that it is easier to code
>   the bots for non coders"**.
> - Robot one is in the code (ROB1-A, integrating onto main now as ROB1-INT); robot two becomes the same build with its
>   own name and autons file (**ROB1-BT** — the G1 brief's "G1-T, the twin", `briefs/G1-robot-one-and-twin.md` :225-233,
>   :558-561). Routes are written in code (`src/team/*autons*.cpp`, `chassis::Routine`: moveTo / driveTo / turnTo /
>   face, curveTo / reverseCurveTo (CURVE1-A, API 2.7), onFail / label / skipTo (STEP1-A), triggers partway through a
>   drive (STEP1-B, API 2.8 — committed `8df58f3` during this writing), mechanism steps r.move / r.set / r.home, the wall
>   reset `competition::resetFromWall` (L6-A2), the checked Toggle step `competition::checkAndFlipToggle` (AF1S-A1, built,
>   merging after VIS1-A2)); the host route preview RPV1-A (`tools/route_preview.py`, building) draws a route on the field.
> - The panel (G3a, branch `g3a-p1`, unmerged; its brief: R1 the route tool, R2 the ROUTES page, **D17 the mirrored pair,
>   "the field's mirror axis not yet read"**, `docs/internal/chunks/G3a-robot-profile-generator-panel.md` :812-841) is
>   **AFTER the 21st**. **Code changes stop ~Nov 18. Max 3 build jobs at a time** (the usage limit).
> - **G4: the students author the routes; a tool may turn THEIR choices into code, it never writes the route for them.**
>   MIRROR1 writes no route. It makes the library run the students' ONE route on the second robot, reflected.
>
> **Five things in the task as given to this writer that the tree, the manual and the probes do not support —
> corrected here** *(the fifth added at the re-check, [R1]):*
> 1. **"Relative moves (driveTo distances) unchanged, turn-by-angle negated."** The library has **no relative move and no
>    turn-by-angle.** `driveTo(x, y)` drives to a FIELD POINT (`routine.hpp` :447-455: "Drive to the FIELD point (x, y)"),
>    `turnTo(h)` turns to a FIELD HEADING (:489-494), `face`/`faceAway`/`reverseTo` take field points, and the Chassis
>    has no relative verb (`chassis.hpp` :379-604 at `d33d25d`; **@8df58f3 :419-668** *[R13]*). So **every number a route writes is a field coordinate and IS
>    reflected.** The only numbers a mirror leaves alone are SIZES: a trigger's percent, inches or degrees (STEP1-B),
>    the arrival tolerances, speeds, timeouts, mechanism goals. Under the field's mirror a heading `h` becomes
>    **−90° − h**, not −h (−h is the RELATIVE-start mirror, D7).
> 2. **"Reflected about the field's mirror axis."** The axis is a **DIAGONAL**, not a midline: the reflection across the
>    line **y = −x** (from the corner where the RED wall meets the RIGHT wall to the corner where the BLUE wall meets the
>    LEFT wall). The usual VEX "mirror" (left ↔ right across x = 0) puts robot two's route on the WRONG goals — the red
>    goals land on black ones (probe P3: 1 of 9 goals keeps its colour). D1.
> 3. **The skills field is NOT symmetric everywhere.** The goals, Toggles, pins and tape are; **the Loaders and two Load
>    Zones' colours are not** (P3: 0 of 4 Loaders land on a Loader). A mirrored route cannot take Match Loads where robot
>    one takes them. And **the ROBOT is not mirrored**: a part mounted off its centre line lands on the other side of the
>    reflected path. D4, §8.
> 4. **"Robot two's autons file registers robot one's routine"** as written today would FAIL the robot build gate:
>    ROB1-A's gate step 8 refuses any robot package that links another robot's autons registration or carries its
>    file's path (`rob1-int/tools/src_build_gate.py` :384-386). *[R13]* **The STRINGS half of step 8 would most likely
>    catch it first** — robot two's packages may not carry `b"src/team/robot_one_autons.cpp\0"`
>    (`ROBOT_FOREIGN_STRINGS` :1709-1713), which a function defined in that file brings into robot two's package through
>    `__FILE__` — beside the symbols half (`ROBOT_FOREIGN_SYMBOLS` :1703-1708). D2 recommends a shared route file both
>    robots register — no gate change.
> 5. *[R1]* **The two robots run AT THE SAME TIME** (`<VURS2>`, v20.txt :3556-3557: both robots in the one skills
>    match). A reflection leaves every point of its line where it is, so **wherever robot one touches the diagonal,
>    robot two touches the same spot at the same moment.** The diagonal runs through the centre goal, two of the skills
>    setup's yellow Pins and the red/right corner's Load Zone. The first print never said so; its own probe route
>    crashes the two robots (P5). D11.
>
> **Executor model: `opus`. Do not commit. Live log `docs/internal/chunks/MIRROR1-PROGRESS.md`, created FIRST.** Say
> "host test plant", never "simulator" — in code, comments, tests and docs. **No hardware claim anywhere:** nothing here
> has run on a robot; robot two as robot one's twin does not exist yet. **Hardware Assumptions: HA-258..HA-260 ONLY**
> (§4; next free at this writing is HA-258 — `rob1-int/docs/internal/PROJECT-BRIEFING.md`; a fourth belief ⇒ stop and
> tell the coordinator; RMP1, briefed beside this, expects none). **Memory rule (2026-10-15):** every binary built —
> probes, the suite, any test — runs as `(ulimit -v 4194304; timeout 300 …)`; the suite only through
> `python3 tools/suite_run.py`.
>
> **Shape:** the house style of `docs/internal/chunks/L6-wall-reset.md`.

---

## Re-check — amendments (2026-11-13 ~02:00–03:00; a checker's 13 findings, each re-verified by this writer)

Every finding was re-opened at its cited `file:line` or re-run; **all 13 are REAL** (2 major, 11 minor) and all 13 are
fixed in the body, each place marked *[Rn]*. Nothing was dismissed.

| # | Finding (severity) | Re-verified how | Disposition — where fixed |
|---|---|---|---|
| **R1** | The two robots run at the same time; a mirrored pair meets on the diagonal (major) | `<VURS2>` v20.txt :3556-3557 read. The checker's `check/probe_meet_chk.cpp` RE-RUN: centres 0.32 in apart at pace 1927, 4.09 in with the 9 % weak side — confirmed. **New probe P5** (`probe_meet.cpp`, `out/p5_meet.txt`) adds the two robots' 15 in squares: on P1's route they already TOUCH at pace 1014 (10.1 s, in the curve), before the centres meet; a route G of the same 14 step kinds kept on robot one's side keeps them **8.43 in** apart = 2 × robot one's clearance from the diagonal (4.21 in) — in a pure mirror the gap is exactly twice the clearance, whatever the timing; with the weak side 8.75 / 4.26. **Figure VURS1 by pixel:** four yellow Pins sit ON the diagonal (within 3 px ≈ 0.4 in), two on red's side at ≈ (23.5, −23.5) and ≈ (47, −47) in | **REAL. FIXED:** header item 5; **NEW D11** (the rule, a suite check of the pair at the same moment, `competition::isMirrored(ctx)` so ONE robot takes what lies on the diagonal, both robots drawn together); D1 bullet; D4 row; plain words; §1.2; T11; MM15–MM16; Step 0.5; HA-258/HA-260 widened (no fourth HA); §6.4–§6.5; §8.13; §9 |
| **R2** | Size and "merged Nov 15–16" not checked against the 3-slot queue or robot two's bench needs (major) | NOTES :993 (build order ROB1-BT, MIRROR1, RMP1, ROB1-C, AF1S-A2 → DOCS3-A), :996 (running: ROB1-INT, RPV1-A, this brief; RMP1 brief next). STEP1 brief :24 and CURVE1 brief :61/:937: ~3–4 each. The peers' measured calendar pace from NOTES (§6.1b). `rob1-int/src/team/robot_one_autons.cpp` :50 "THE SKILLS ROUTE GOES HERE"; ROB1-A's 17 owed facts | **REAL. FIXED:** MIRROR1-A re-sized **~3–4 builder-days** (+~½–1 review/merge) with its reasons; **§6.1b slot plan** naming every queued run; the **five bench conditions** beside the date (D9, §9); D9 (b) restated — it saves the build risk only, not the bench path |
| **R3** | D5's "exactly three" misses the camera functions' field inputs (minor) | `vision/piece_finder.hpp` (staged on main) :73-76 (the caller states the pose), :158-164 (`PieceTarget{x, y}`); `toggle_tracker.hpp` :475-479; ALLY1-L4 brief :348-349; NOTES :948 (L4 after the 21st) | **REAL. FIXED:** D4 row; D5 says what is REFUSED and what is only DOCUMENTED; §6.6 note for L4/ALLY1; §8.17 |
| **R4** | Evidence cited to probes that did not test it (minor) | `probe_path.cpp`: no trigger, no observer, no `strafeTo`, no `drive()`, tank only — confirmed by grep; `probe_field.py`: no pins. The checker's `check/probe_trigger_chk.cpp` RE-RUN: 24 in drive / 63° turn / exact 180° turn — progress equal to ≤ 3.82e-14, 60 % on the same tick in each. The VURS1 crops (`check/crosses.png`) viewed | **REAL. FIXED:** D1 and D4 re-cite (P6 = the checker's trigger probe; the figure read for the pins); §0.4 says P1 is tank-only with no `strafeTo`/`drive()`/trigger (T2 covers them); plain words no longer say "every kind" |
| **R5** | The 180° tie also affects moveTo and a curve's end on a holonomic drive (minor) | `motion/move_to_pose.hpp` :216, :386, :586 and `curve_follow.hpp` :271, :601 @8df58f3 — all `errorTo`; robot one believed a tank (`rob1-int/src/robots/chassis_table.hpp` :210, HA-252) | **REAL. FIXED:** D6 names MoveToPose and the curve end; the `[MIR]` Warn covers a mirrored moveTo / trajectory leg whose heading change is within 1° of 180°; T2's X-drive route avoids exact 180° heading changes, T3 adds one pinned; §8.2 |
| **R6** | `CurvePoint` has no default constructor (minor) | `motion/curve_path.hpp` :93-110 @8df58f3 | **REAL. FIXED:** §1.2 names the storage; §8.14 |
| **R7** | Cookbook page 08 is AF1S-A2's (minor) | `af1s-a1/test/cookbook_examples_test.cpp` :13-14, :878; no brief claims 09 (grep of every brief) | **REAL. FIXED:** cookbook **09**, test prefix `cookbook-09a`; §3, §6.2, §7 |
| **R8** | Guide 11 missing; log lines' budget and characters unchecked (minor) | Guide 11 :246, :512-527 (the WALL lines pinned byte for byte by `test/wall_reset_line_test.cpp`); `match_runner.hpp` :239 `char line[160]`; the sink cuts a message at **200 bytes** (`diag/term_sink.hpp` :285); the first print's sample COMP line had an em dash and measures **~220** characters at the worst case (a 25-character name, ±9999.99) | **REAL. FIXED:** D8 (two ASCII COMP lines, each under 160 and 200; the WALL suffix inside the sensor's parenthesis, so a cut never drops it); guide 11 in §3 and §6.2; T8 measures every new line's worst case |
| **R9** | T8's worst case weaker than the screens' own; reflected zeros print "-0.0" (minor) | `auton_tester_panel_test.cpp` :224 (−999.9), `competition_screen_test.cpp` :455 (−9999.9); "START MIR (-9999.9, -9999.9) -179.9 deg" = 39 characters; `printf("%.1f", -0.0)` = "-0.0" (checked); `toggle_visit.hpp` :674's `+ 0.0` | **REAL. FIXED:** T8 reuses −999.9 / −9999.9 and pins a zero coordinate and a mirrored RELATIVE start; `math::Reflection` adds `+ 0.0`; T1's "bit-exact" becomes "equal, never −0.0"; MM17; §8.15 |
| **R10** | The shared vectors file assumes tooling that does not exist (minor) | `git ls-tree 8df58f3 test/`: no `fixtures/`, `test/vendor` = `doctest.h` only, no `package.json`; goldens are read through `SHULIB_SOURCE_DIR` (`calibrate_golden_test.cpp` :270) | **REAL. FIXED:** a plain text file `test/goldens/mirror/skills_mirror_vectors.txt`, one vector per line, ~20 lines of `std::ifstream` + `std::sscanf`; the node test set-up is RMP1's (D10, §1.4, §6.5) |
| **R11** | Who writes robot two's registration line, under G4 (minor) | The memory's G4 rule: the students author competition code; the only exception given is the 40 in auton | **REAL. FIXED:** D2 asks it; recommended: the STUDENTS type the one line from cookbook 09; no run writes it without the team lead's explicit go (§6.3) |
| **R12** | The driver-control check is a hard stop, and loosely placed (minor) | `teleop/teleop_loop.hpp` :241 (`detail::driverLoop`, the one body; called at :459 and :478, the other two overloads forwarding through them at :491 and :502); `competition_program.cpp` :12-14 (opcontrol waits for the auton task's FINISHED) | **REAL. FIXED:** the check lives in `detail::driverLoop` and is NOT a precondition: a loud `[MIR]` Warn and the leaked mirror CLEARED, so the driver always drives the real field (§1.2, T4, §8.9) |
| **R13** | Citation slips (minor) | `routine.hpp` @8df58f3: :1052 is a comment, `bearingTo` :1368-1369; `chassis.hpp` @8df58f3 :419-668; `toggle_visit.hpp` :775-784 (the WRONG WALL check) vs :668-675 (`towardWallRadians`); `rpv1-a/test/route_preview_run.hpp` :120-130 already holds robot two's row; `ROBOT_FOREIGN_STRINGS` :1709-1713 | **REAL. FIXED** in place (header item 4, D3, §0.2, §6.4) — and the first print's status times, which named hours not yet reached, corrected (header) |

---

## DECISIONS FOR THE TEAM LEAD (a recommendation for each; nothing below decides them silently)

**D1 — Which mirror the field has (read from the manual, not assumed). RECOMMENDED (a).**
- **(a) RECOMMENDED: the reflection across the field's DIAGONAL from the red/right corner to the blue/left corner.** In
  the library's field frame (F1: +x to the right as seen from the red station, +y away from it; `override_walls.hpp`
  :5-10, roadmap :135): **(x, y, heading) → (−y, −x, −90° − heading).** Why this one, from the manual:
  - In a VEX U skills run **both robots start "in legal starting positions for the red Alliance"** (`<VURS3>`, v20.txt
    :3560-3562; v2.1 the same), and `<SG1>` still applies: each robot **"not sharing a Quadrant with another Robot"**
    (:1067) and **"contacting the Field tiles and Field Perimeter on their Alliance's side of the Autonomous Line"**
    (:1069). The Autonomous Line is the DOUBLE diagonal tape (glossary :4230; A11's tape sheet, `img/a-106.png`), which in
    F1 runs along y = x; red's side holds exactly two Quadrants (the glossary's triangles, :4584-4591): **the red wall's
    and the right wall's.** So robot one starts in one, robot two in the other.
  - The diagonal reflection maps those two Quadrants onto each other and keeps red's side red. On A10's goal table
    (`img/a-105.png`) **all 9 goals land on a goal of the same colour**, **all 4 Toggles land on a Toggle of the same
    Quadrant colour** (red wall ↔ right wall, left wall ↔ blue wall) — probe P3 — and the VEX U skills setup's pins
    land on pins of the same colours (Figure VURS1, v20.txt :3574, rendered `manual/vurs1_field.png`). *[R4] The pins
    were NOT in P3 (it has none): they were read off the figure — by eye, the four four-Pin crosses identical and each
    the other's image across the diagonal (the checker's crops `check/crosses.png`, viewed by this writer); by pixel,
    the single yellow Pins' centres (§0.4, P5's note).*
  - Every other candidate fails on the manual's own numbers (P3): **left/right across x = 0** keeps 1 of 9 goals' colour
    and does not keep red's side red; **red/blue across y = 0** 1 of 9; **the half turn** keeps 5 of 9 and turns red's
    side blue — it IS the head-to-head red ↔ blue swap (A16's AprilTag numbers pair under it, 9 of 9 — the lead the G3a
    brief named, :832-835); **the other diagonal** turns red's side blue.
  - **What it does NOT carry (manual facts, D4):** the Loaders (all four on the red and blue walls, `override_walls.hpp`
    :34-35; their images land on the right and left walls, where there are none — 0 of 4), the Load Zones' colours (the
    red/left corner's red zone lands on the blue/right corner's blue zone — 2 of 4 keep their colour), and the AprilTag
    NUMBERS (tags 1 ↔ 4, 2 ↔ 3 — only the centre's 0 keeps its number). Field elements may sit up to ±1.0 in off nominal
    (`<T5>`a, :2960) — HA-258.
- **(b) Left/right across x = 0** — what "mirrored" usually means in VEX. **Cost:** on Override it sends robot two to
  black goals where robot one scores red (P3); not this field's symmetry.
- **(c) The half turn.** **Cost:** robot two would start on the BLUE side — illegal in skills (`<VURS3>`).
- **Please confirm with the builder and drivers:** robot one starts in the red wall's Quadrant and robot two in the right
  wall's (as seen from the red station), or the other way round — the mirror is the same either way.
- *[R1]* **What this mirror means with BOTH robots on the field at once** (`<VURS2>`): the diagonal is the seam between
  the two robots' halves, and a reflection leaves it where it is — so whenever robot one's robot reaches the diagonal,
  robot two's reaches the same spot at the same moment. **A mirrored route is safe only while robot one's whole robot
  stays on its own side of the diagonal with room to spare** — for a 15 in robot its CENTRE at least 7.5 in (square
  on) to 10.6 in (corner on) from the line, plus the margin D11 sets (3 in recommended). On the diagonal itself sit
  the centre goal, two yellow Pins of the skills setup and the red/right corner's Load Zone: a pure mirror can take
  none of them without a crash; ONE robot can, by a one-line choice in the shared route (D11).

**D2 — How a route says it is mirrored, and how robot two's autons file reads. RECOMMENDED (a).**
- **(a) RECOMMENDED: the AUTON ENTRY says it** (`competition::AutonEntry`, the row a team registers,
  `auton_registry.hpp` :249-284 — NOT frozen, roadmap :135-149 has no competition row), and the students' route lives
  ONCE in a shared file both robots register. Robot one's file: `registry.add(skillsRoute());` — robot two's file,
  **one line per mirrored auton**:
  ```cpp
  // Robot two is robot one's build. It runs robot one's skills route MIRRORED across the field's diagonal:
  // the red wall's Quadrant and the right wall's are mirror images of each other (game manual, Appendix A).
  registry.add(competition::mirrored(skillsRoute(), competition::override2026::kSkillsMirror));
  ```
  where `skillsRoute()` is the students' function in `src/team/skills_route.cpp` that builds the entry (name, kind,
  routine, start, schedule) — the route itself is THEIRS (G4). `mirrored(entry, mirror)` copies the entry and marks it;
  the auton runner does the rest when it runs that entry (D3). Neither robot's package links the other's registration,
  so ROB1-A's gate step 8 (`src_build_gate.py` :384-386 in `rob1-int`) holds unchanged. The file layout is ROB1-BT's
  to build (§6.3); MIRROR1 supplies the call, the recipe and the guide.
  - *[R11]* **A question for you (G4): who types robot two's one line?** It is competition code — it CHOOSES which
    route robot two runs — and the only exception you have given to "the students write competition code" is the
    40 in auton. **RECOMMENDED: the students type it**, from cookbook 09's recipe (one line, quoted and compiled by the
    suite); robot two's starter autons file (ROB1-BT) carries a COMMENT naming the recipe and guide 17's section, and
    no run writes the line itself. **The other way:** you give an explicit go, like the 40 in's, for the run that lands
    second of ROB1-BT and MIRROR1-A to write it. Either way the ROUTE stays the students' alone.
  - **(a′) One line for ALL of robot one's autons:** `competition::registerMirrored(registry, &registerRobotOneAutons,
    kSkillsMirror);`. **Cost:** robot two's package then links robot one's registration — gate step 8 must be re-ruled
    for the twin in ROB1-BT; and robot one's bench autons (the 40 in, the acceptance run) come mirrored too. Not for the
    21st; additive later.
- **(b) A Routine option** (`Routine r{ctx, "skills", mirrored}`). **Cost:** the routine function would need to know
  which robot runs it — an auton routine takes only its context (`auton_registry.hpp` :241, a plain function pointer),
  so robot two needs a wrapper function per auton; and the park (the entry's end action) and any Chassis call inside a
  `then()` would run robot one's numbers on robot two (D3 (b)).
- **(c) A per-robot setting in robot two's robot file** ("everything this robot runs is mirrored"). **Cost:** it hides
  the mirror from the autons file a student reads, and mirrors robot two's own bench tests and any robot-two-only route.
- **(d) A wrapper around the routine** (`competition::mirroredRoutine<&skills>`). **Cost:** the start pose must then be
  mirrored separately in the entry — two places that must agree; (a) does both in one call.

**D3 — Where the reflection is applied. RECOMMENDED (a).**
- **(a) RECOMMENDED: at the Chassis's front door, for exactly the mirrored run.** The auton runner (`match_runner.hpp`
  :193-277) seats robot two at the reflected start, then switches the Chassis into a MIRRORED RUN for the routine AND its
  end action (the park), and switches it back on every way out (a normal end, the deadline's cut, an abandon, a throw).
  While it is on, every FIELD number that enters the Chassis is reflected — `moveTo`, both STEP1-B observer overloads,
  `strafeTo`, `turnTo`, `followTrajectory`, `curveTo` / `reverseCurveTo`, `setPose`, `drive` — and `pose()` hands back
  the estimate reflected (*[R13]* `chassis.hpp` **@8df58f3**: `moveTo` :419, `strafeTo` :432, `turnTo` :442, the
  observer overloads :453 / :464, `followTrajectory` :477 / :506, `curveTo` :522 / :528, `reverseCurveTo` :536 / :542,
  `drive` :602, `pose()` :663, `setPose` :668). So the students' route sees robot one's numbers everywhere it looks,
  including a `waitFor` that reads the pose and a raw `ctx.chassis.moveTo` inside a `then()`.
  **No change to the Routine at all** (F10 untouched — it only calls the Chassis and reads `chassis_->pose()` in
  `bearingTo`, *[R13]* `routine.hpp` **:1368-1369 @8df58f3** (:1051-1052 at `d33d25d`), and logs). The wall reset and the Toggle step work BELOW the front door (they read
  the localizer and drive the scheduler directly: `wall_reset_call.hpp` :84; `af1s-a1/.../toggle_visit.hpp` :851,
  :1065), so they get the real field and only their wall / sensor side / Toggle id are swapped (D4). The competition
  program runs the auton on its own task that PROS never deletes and unwinds the routine on that task's stack
  (`src/programs/competition_program.cpp` :6-21), so the switch-back always runs, and opcontrol() starts driver control
  only after the auton task's FINISHED (:12-14), which comes after the switch-back; *[R12]* belt and braces, the driver
  loop itself CLEARS a mirror still on, with a loud `[MIR]` Warn — never a refusal that would leave the driver with no
  drive (§1.2).
- **(b) Inside the Routine's steps only.** **Cost:** the park and every raw Chassis call in a `then()` or a `waitFor`
  would use robot one's numbers on robot two — silently wrong, and the park would drive across the field; and every
  field step's body in F10 plus STEP1-B's six armed legs would change.

**D4 — What is mirrored and what is NOT. RECOMMENDED: as the table.**

| What the route writes | Under the mirror (skills diagonal, FIELD start) | Why / evidence |
|---|---|---|
| The entry's FIELD start `(x, y, h)` | seated at `(−y, −x, −90° − h)`; the screen shows robot two's real start (D8) | the runner's re-seat, `match_runner.hpp` :233-237 |
| `startAt(pose)`, `setPose` | position reflected (heading ignored, as today) | `localizer.hpp` :618-632 |
| `moveTo(pose)`, `followTrajectory(poses)` | every position and heading reflected | P1 A: path = reflection to 8.8e-13 in (a tank); *[R5]* on a holonomic drive an EXACT 180° heading change rotates counterclockwise on both robots: D6 |
| `driveTo` / `reverseTo` / `face` / `faceAway` `(x, y)` | the point reflected; the bearing is computed from the reflected pose, so it comes out reflected | P1 A |
| `strafeTo(x, y)` | the point reflected | *[R4]* NOT in P1 (a tank; P1 has no `strafeTo`) — reasoned, T2 on the X-drive rig |
| `turnTo(h)` | `−90° − h`; every turn sweeps the OTHER way | P1 A; the exact-180° tie: D6 |
| `curveTo` / `reverseCurveTo` points | every point and every stated heading reflected; the curve bends the other way | P1 A (steps 8, 9) |
| `brake`, `hold(t)`, `pause(t)`, `waitFor(pred, t)`, `then(...)`, `onFail` / `label` / `skipTo` | unchanged; inside them every Chassis number is robot one's | they write no field number |
| STEP1-B triggers `atPercent` / `afterDistance` / `afterTurning` | unchanged — a share, a distance along the leg and an angle turned are the same sizes in a mirror | `leg_progress.hpp` :13-18 @8df58f3; *[R4]* **P6** (the checker's `check/probe_trigger_chk.cpp`, re-run by this writer — NOT P1, which has no trigger): a 24 in drive, a 63° turn and an exact 180° turn reflected by hand, the leg observer's progress equal to ≤ 3.82e-14 and 60 % reached on the same tick in each, the tie included |
| Mechanism steps (move / set / home / run / spinUp / stop / waitUntilDone and the start forms) | unchanged | no field number — but see "the robot" below |
| `MotionOptions` (timeouts, speeds, tolerances) | unchanged | sizes |
| `drive(speeds, Field)` / `drive(speeds, Body)` in a `then()` | field vector reflected and turn rate negated / sideways speed and turn rate negated | a reflection reverses turning; *[R4]* not in P1 — T2 |
| `competition::resetFromWall(ctx, {wall, sensor, …})` | the wall → its mirror image (red ↔ right, left ↔ blue), sensor LEFT ↔ RIGHT; `maxCorrection`, `required`, `name` unchanged | P2: SEATED on the right wall with the LEFT sensor |
| `checkAndFlipToggle(…, request)` (AF1S) | the Toggle → its mirror image (right ↔ red wall, left ↔ blue wall), the Quadrant colour kept; distances and times unchanged | P4 |
| The end action (the park) | mirrored (it runs inside the mirrored run) | D3 |
| A config::Route entry (F8, through PathRunner) | mirrored the same way (PathRunner drives only through the Chassis, `path_runner.hpp` :233-331) | — |
| The log and the screens | the REAL field (robot two's numbers), marked MIRRORED (D8) | they read below the front door |
| `chassis.deps()`, `chassis.scheduler()`, `chassis.lastCompleted()`'s poses | NOT mirrored — the real field ("below the mirror", the guide says so) | escape hatches; `lastCompleted()` returns a reference to the scheduler's record (`chassis.hpp` :622 at `d33d25d`, *[R13]* :686 @8df58f3) |
| *[R3]* The camera's field inputs: `vision::PieceTarget{x, y}` (`piece_finder.hpp` :158-164, staged on main), the robot pose a caller hands the piece finder (:73-76) or a `ToggleTracker::update` (`toggle_tracker.hpp` :475-479), and a camera FIX the route drives to | **NOT mirrored, and NOT refused — documented.** The camera functions read the REAL field: feed them the localizer's pose (`ctx.chassis.deps().localizer->pose()`, as AF1S's step does) and real-field targets — never `ctx.chassis.pose()` inside a mirrored run (robot one's numbers). A real-field fix goes into a Chassis verb through `ctx.chassis.reflection()` first (a reflection is its own inverse), or the front door reflects it a second time | no program uses the finder before the 21st (drive-to-piece, L4, after the 21st — NOTES :948); §6.6 |
| *[R1]* **Both robots at once** (`<VURS2>`) | robot two is at the mirror image of robot one AT THE SAME MOMENT; a point ON the diagonal is its own image | P5: the probe route crashes (squares touching from 10.1 s); a route kept on robot one's side keeps them 2 × its clearance apart — D11 |

**What the mirror cannot carry (told plainly, not refused — the library cannot see it):** a step at a **Loader**
(robot two's mirrored spot has none — Match Loads only through red Loaders, `<RSC2>`d, v20.txt :2025); the **Load Zone**
in the red/left corner becomes the BLUE one in the blue/right corner; **anything mounted off the robot's centre line**
(a side claw, an off-centre camera, a flipper to one side) meets the field on the other side of the reflected path —
HA-259; the robot's own left/right drive differences (HA-260). The preview shows the first two; the bench shows the rest.
*[R1]* **And what lies ON the diagonal** — the centre goal (0, 0), the two yellow Pins of the skills setup on the single
tape from the red/right corner to the centre (≈ (23.5, −23.5) and ≈ (47, −47) in, Figure VURS1 by pixel), and the
red/right corner's Load Zone with the red Loader at (58.76, −70.2) beside it — is the SAME place for both robots at the
same moment: a pure mirror cannot take it at all without a crash (D11).

**D5 — A step that cannot be mirrored. RECOMMENDED (a).**
- **(a) RECOMMENDED: refused BY NAME where it runs, never silently passed.** *[R3] The first print said "exactly three
  things cannot be mirrored" — wrong: the camera functions' field inputs cannot be mirrored either (D4's row); they are
  DOCUMENTED, not refused, because they are plain numbers the library cannot tell from any other. And a route that
  brings the two robots together is caught by the suite (D11), not here.* Exactly three things are REFUSED:
  (1) the EXPLICIT wall reset `chassis::resetFromWall(Chassis&, const WallSensor&, …)` (`wall_reset_call.hpp` :170) —
  it is handed a sensor DEVICE, and the mirror needs the other side's: a precondition naming the everyday form;
  (2) an entry mirrored twice by two DIFFERENT mirrors (twice by the same one is the original — `mirrored` toggles):
  refused at the `mirrored(...)` call; (3) a mirrored entry whose name is over 25 characters — refused at registration
  by the registry's own path, because its button must read "<name> SKILLS MIR" on one line (D8). A refusal at run time
  can only happen when the line runs, because ONE function serves both robots; so **the host suite runs every auton of
  robot two's registry on the host test plant** (as it runs robot two's today, `test/autons_test.cpp`), and robot two's
  mirrored autons are exercised before the robot ever is.
- **(b) Pass them through unmirrored.** **Cost:** a wall reset against robot one's wall from robot two's spot is refused
  "not square" by L6's own gates (P2: all three wrong forms) — safe, but the reset is lost without a word about why.
- **(c) Refuse at compile time.** Not possible: the same compiled function runs mirrored on one robot and not on the
  other; only the entry knows.

**D6 — A turn of exactly 180°. RECOMMENDED (a).**
- **(a) RECOMMENDED: keep F3's rule as it is** (an exact ±180° turns counterclockwise, `angle.hpp` :11-12, roadmap :137,
  LOCKED) **and say it**: one `[MIR]` Warn line when a mirrored turn starts within 1° of 180° ("robot two may sweep the
  same way as robot one, not the mirror way"), and the guide's advice — when the sweep matters, write two 90° turns.
  Measured (P1 C): with a first turn of exactly 180° robot two turns counterclockwise like robot one — mid-turn the
  headings differ by up to 179°, the path by 0.155 in, the end by under 0.1 in. Any turn not EXACTLY 180° is mirrored
  exactly (P1 A's 14 steps). On a robot an exact tie never happens: the IMU's noise makes a 180° turn a coin toss for
  EACH robot already.
  *[R5] Not only TurnTo:* **MoveToPose's heading term uses the same rule** (`motion/move_to_pose.hpp` :216, :386, :586
  @8df58f3, `errorTo`), and so does **the curve follower's end heading** (`curve_follow.hpp` :271, :601). On a
  HOLONOMIC drive, a `moveTo` (or a trajectory leg) whose heading change is exactly 180° rotates counterclockwise on
  both robots; a curve's end heading meets the tie only if the robot arrives exactly opposite its stated heading. So
  the `[MIR]` Warn covers a mirrored `turnTo` AND a mirrored `moveTo` / trajectory leg whose heading change at its
  start is within 1° of 180°. **Robot one is believed a tank** (`rob1-int/src/robots/chassis_table.hpp` :210,
  HA-252), where a moveTo's heading follows its travel — nothing changes for the 21st.
- **(b) Mirror the tie too** (robot two turns clockwise at an exact 180°). **Cost:** TurnTo, MoveToPose, the curve's
  end AND STEP1-B's turn progress (`leg_progress.hpp` :97-103 @8df58f3, the same tie) must change together — one
  without the others and an `afterTurning` trigger on that turn never fires; it changes nothing measurable on a robot;
  F3 is LOCKED.

**D7 — A RELATIVE start. RECOMMENDED (a).**
- **(a) RECOMMENDED: mirrored in the robot's own frame — (x, y, h) → (x, −y, −h): every left becomes a right.** A
  RELATIVE auton's numbers are in the frame of wherever the robot was placed (`auton_registry.hpp` :157-180); when the
  drivers place robot two at the mirror image of robot one's spot, the field's diagonal mirror IS this robot-frame flip
  (P1 D: exact to 2.3e-13 in). "40 in forward" mirrors to itself. The wall reset still refuses a RELATIVE start
  (`auton_wall_reset.hpp` :112), as today.
- **(b) Refuse a mirrored RELATIVE entry.** **Cost:** robot one's bench autons (relative) could not be mirrored.

**D8 — How the brain screens and the preview show a mirrored auton. RECOMMENDED (a).**
- **(a) RECOMMENDED:** the menu button reads **"<name> SKILLS MIR"** (the label is "<name> <KIND>" in a 36-column line,
  `auton_menu.hpp` :17, :99-102, :419 — so a mirrored name is at most 25 characters, D5); the Auton Tester's and the
  competition screen's START line reads **"START MIR (55.7, 36.0) -90.0 deg"** — robot two's REAL start, which is where
  the drivers must place it (`auton_tester_panel.hpp` :593-594, `competition_screen.hpp` :492-494); the END line and the
  log are the real field; the runner's COMP lines name the mirror and both poses. *[R8] The first print's sample was
  one line with an em dash, ~220 characters at the worst case — over `match_runner.hpp` :239's 160-byte buffer and the
  sink's 200-byte cut (`diag/term_sink.hpp` :285). Now TWO ASCII lines:* the existing re-seat line with the mode word
  `FIELD MIRRORED` (`auton 'skills': re-seated FIELD MIRRORED (55.70, 36.00) in facing -90.0 deg; the IMU read ...` —
  147 characters at the worst case: a 25-character name, ±9999.99, −179.9) and a second, mirrored runs only:
  `auton 'skills': MIRRORED across the skills diagonal -- robot one's start (-36.00, -55.70) facing 0.0 deg`
  (130 at the worst case). The `[WALL]` line's mirrored note goes INSIDE the sensor's parenthesis —
  `right wall x SEATED (LEFT sensor, mirrored from red wall RIGHT): ...` — so a cut at 200 bytes never drops it; every
  new line is ASCII ("--", never a dash character) and its worst case measured by T8. **RPV1:** a mirrored entry
  previews with no RPV1 change (its run goes through the Auton Tester's sequence and `competition::runAuton`,
  `rpv1-a/test/route_preview_run.hpp` :17 — RPV1-A is still building, its lines move); its page adds a "MIRRORED from
  robot one's route" banner, robot one's path faded beside it, and *[R1]* BOTH robots drawn at the same moment, as a
  small RPV1 follow-up (§6.4).
- **(b) A separate "MIRRORED" screen line.** **Cost:** the result screens' line budget is used (the executor confirms
  before choosing (b)); a longer name budget.

**D9 — The run, and when. RECOMMENDED (a).**
- **(a) RECOMMENDED: ONE run, MIRROR1-A — *[R2] re-sized* ~3–4 builder-days + ~½–1 day review and merge — launched
  at the first free slot after your go,** beside ROB1-BT (no shared code file, §6.2 — re-checked at launch). *[R2] The
  first print said ~2–3; its peers STEP1-B and CURVE1-A were each sized ~3–4 (STEP1 brief :24, CURVE1 brief :61), and
  MIRROR1-A, though it has no new controller, is as WIDE: ~10 library and screen files, ~12 Chassis functions behind
  F6's signature pin, a scope with six ways out, a runner-level hash, 5 new test files and ~6 changed, 17 mutations,
  ~5 pages and guide 11's lines, plus D11's pair check (~¼–½).* The Toggle swap goes in whichever lands SECOND of MIRROR1-A and AF1S-A1's merge (one function
  and one test — ~¼ day if it is a separate MIRROR1-B; AF1S-A1 is expected to merge first, today). Robot two's one
  line: the students' (D2's question). **Honest dates (§6.1b, §9):** the peers' builds ran ~5–8 hours of agent time
  each, spread over ~1½–2 calendar days by usage stops and pauses (CURVE1-A 11-11 ~10:00 → 11-12 ~22:40; STEP1-B
  11-11 ~01:50 → 11-13 ~01:00; ROB1-A 11-11 ~03:40 → 11-12 ~23:00 — NOTES). So: your go on Fri 11-13 → launched that
  day → built ~Nov 14–15 → **merged ~Nov 15–16**. A go on Nov 14, or a full-day usage stop, puts the merge ~Nov 17,
  at the freeze edge.
  **What robot two needs on the BENCH besides MIRROR1-A — the date above is not the long pole:** (1) ROB1-BT merged
  (robot two is robot one's build in the code — it waits for ROB1-INT's merge and for your builder to say robot two is
  rebuilt, G1 brief :558-561); (2) the builder's facts typed — robot one's 17 owed facts (ROB1-A's five robot-one
  builds refuse until then) and robot two's own, once it is the twin; (3) the students' robot-one skills route
  written and run on robot one (`rob1-int/src/team/robot_one_autons.cpp` :50 still reads "THE SKILLS ROUTE GOES
  HERE"); (4) robot two's one line typed (D2); (5) the pair kept apart (D11) — the suite says so the moment the line is
  in. **Robot two's mirrored route on the bench Nov 16–18 only if (1)–(4) are done by then.**
- **(b) After the 21st; for the 21st, robot two's route hand-reflected** (a copy of robot one's function with every
  number put through the table in §0.3, the wall reset's wall and sensor and the Toggle swapped by hand). **Cost:** two
  copies of the route to keep equal by hand; a missed number is a wrong route. *[R2] In the light of the bench list:
  (b) removes only MIRROR1-A's build risk — it needs the same (1)–(3) first, it keeps D11's crash risk with no suite
  check to catch it, and every edit of robot one's route before the freeze must be re-reflected by hand. Worth it only
  if MIRROR1-A has not merged by ~Nov 17.*

**D10 — The route maker page (RMP1, briefed beside this) and a mirrored route. RECOMMENDED (a).**
- **(a) RECOMMENDED: RMP1 exports robot one's route ONCE plus robot two's one registration line (D2) — never a second,
  reflected copy of the numbers** — and DRAWS the mirrored path with the same formula, checked against one shared table
  of test vectors. *[R10] The first print named `test/fixtures/skills_mirror_vectors.json` "read by the C++ suite and
  by RMP1's `node --test`" — but there is no `test/fixtures/`, no JSON reader (`test/vendor` holds only `doctest.h`)
  and no `package.json` at `8df58f3`.* Now: a plain TEXT file, **`test/goldens/mirror/skills_mirror_vectors.txt`**,
  one vector per line (`FIELD x y h -> x' y' h'` or `RELATIVE …`, `#` comments), read by the C++ suite in ~20 lines
  (`std::ifstream` + `std::sscanf`, located through `SHULIB_SOURCE_DIR` as `calibrate_golden_test.cpp` :270 locates
  its goldens) and by RMP1 by splitting on whitespace. **RMP1's own test set-up (node, `package.json`) is RMP1's to
  choose and own** — MIRROR1-A adds no JavaScript tooling. *[R1]* RMP1 also draws BOTH robots at the same moment and
  flags where they come within D11's gap (§6.5).
- **(b) RMP1 exports a reflected copy.** **Cost:** two copies drift the first time robot one's route is edited; only
  worth it as the fallback if MIRROR1-A misses the freeze (D9 (b)).

**D11 — *[R1, NEW at the re-check]* The two robots run AT THE SAME TIME: keeping a mirrored pair apart. RECOMMENDED
(a).**
- **The fact:** in a VEX U skills match both robots run together (`<VURS2>`, v20.txt :3556-3557). A reflection leaves
  every point of its line where it is, so whenever robot one's robot touches the diagonal, robot two's touches the same
  spot at the same moment — and robot two's square is robot one's square reflected, so **in a pure mirror the gap
  between the two robots is exactly twice robot one's clearance from the diagonal, whatever their timing** (P5: 8.43 in
  = 2 × 4.21 in; with a 9 % weak side 8.75 / 4.26). On the diagonal sit the centre goal, two yellow Pins of the skills
  setup (≈ (23.5, −23.5), ≈ (47, −47)) and the red/right corner's Load Zone. **Measured:** P1's own 14-step route —
  the first print's evidence — CRASHES the two robots: their 15 in squares touch from 10.1 s (pace 1014, in the curve)
  and their centres come within 0.32 in at 19.3 s (P5; the checker's probe).
- **(a) RECOMMENDED — four parts:**
  1. **The rule, in the guide and the cookbook:** a mirrored route keeps robot one's whole robot at least **3 in** from
     the diagonal — then the robots are at least 6 in apart at every moment, however their timing drifts. For a 15 in
     robot: its centre at least 10.5 in from the line when square to it, 13.6 in when corner-on.
  2. **A SUITE CHECK on the host test plant** for every MIRRORED entry robot two registers (beside T9's runs): robot
     one's run (the entry as written) and robot two's run (mirrored), each on its own plant, compared pace by pace —
     **FAIL**, naming the time, both positions and the step each robot was on, when their 15 in squares (RPV1's
     `kPreviewFootprintIn`, `rpv1-a/test/route_preview_model.hpp` :48 — "both robots are 15 in") come closer than
     **`kMirrorGapIn` = 6 in** (a named constant the team may change). In a pure mirror this IS part 1's rule. A
     RELATIVE mirrored entry is skipped by name (its field positions are wherever the drivers put it).
  3. **ONE robot only, for what lies on the diagonal:** `competition::isMirrored(ctx)` — true in robot two's mirrored
     run — so the students' ONE route can send only one robot to the centre goal, a diagonal Pin or the corner Load
     Zone: `if (!competition::isMirrored(ctx)) { /* robot one only */ }`. The two runs then differ in time, and part
     2's check compares them at the same moment — **at the planned timing only**; a real robot's timing drifts (P1 B:
     0.44 s over 21 s from a 9 % weak side), so the guide advises keeping the other robot far from the spot while
     one robot takes it.
  4. **Both robots drawn together** — RPV1's follow-up and RMP1 draw the two robots at the same moment, the 6 in band
     along the diagonal, and the first moment their squares come within the gap (§6.4, §6.5).
- **(b) A start delay for robot two** (`mirrored(entry, mirror, delay)`). **Cost:** it is no longer a mirror in time;
  it does not stop a meeting at a place robot one is still at (the centre goal); robot two loses the delay from its
  60 s. Additive later if the bench asks for it.
- **(c) Tell only** (the guide and the preview), no suite check. **Cost:** the first crash would be found on the field.
- **(d) A check on the robot itself.** Rejected: robot two cannot see robot one; its own distance from the diagonal is
  the right number only for a pure mirror and wrong after a one-robot branch — false alarms, or false comfort.
- **Please decide:** the 6 in gap (3 in each side), or another number; and whether a route that fails the pair check
  should turn the suite red (recommended — it is a crash) or only print a warning.

---

## The design in plain words (for the team lead)

Robot one and robot two start in the two red Quadrants — the red wall's and the right wall's. The game manual's field
drawing makes those two halves of red's side mirror images across the diagonal that runs between them: the same goals,
the same colours, the same Toggles, the same pins. So the students write the skills route ONCE, for robot one, and robot
two's autons file says, in one line, "run that route mirrored".

When robot two runs it, the library seats robot two at the mirror image of robot one's start and puts the drive in
"mirrored" mode for that one run: every spot and heading the route names is reflected before the robot drives to it,
and every position the route reads back is reflected the other way, so the route always sees robot one's numbers. On
the host test plant robot two's path came out as the exact reflection of robot one's, to a billionth of an inch, over 14
steps of the kinds a tank route uses — straight drives, turns, faces, backing up, curves forwards and backwards (probe
P1; *[R4]* sideways moves and raw drive commands were not in it — the build's tests cover them). A wall reset
uses the mirror wall and the robot's OTHER side sensor (robot one's right sensor on the red wall becomes robot two's left
sensor on the right wall — P2: it seated, moving the estimate the mirrored inch). The Toggle step goes to the mirror
Toggle, whose colour is the same (P4). Mechanism steps, triggers, waits and time limits do not change.

What it cannot do: robot two has no Loader where robot one's is, so a route that takes Match Loads at a Loader needs
its own steps for robot two; and the ROBOT is not mirrored — a claw on robot one's left side is still on robot two's
left, which in the mirrored world is the wrong side. If the robot is built symmetric left to right, the mirror works; if
not, the bench will show which steps need robot two's own numbers. Nothing here has run on a robot.

*[R1, added at the re-check]* **The two robots are on the field AT THE SAME TIME, and that changes how the route must
be written.** The diagonal is the line between robot one's half and robot two's half, and the mirror leaves it where it
is: whenever robot one drives onto the diagonal, robot two drives onto the same spot at the same moment — a crash. The
test route this brief first used to prove the mirror does exactly that (the two robots touch 10 seconds in). So the
rule is: **robot one's whole robot stays at least 3 inches on its own side of the diagonal.** Then the two robots are
always at least 6 inches apart, even if one runs a little slower than the other. The centre goal, two yellow Pins and
the corner Load Zone sit right on the diagonal, so only ONE robot can go for each — the shared route says which with
one line ("robot one only"). The library's tests run the two robots together on the computer and fail, naming the
step, if they ever come within 6 inches; the route preview will draw both robots together so the students can see it.

---

## 0. Why — what exists, what the manual gives, and the probes

### 0.1 Why this chunk, and why now
Both robots run the skills route on Nov 21 (the header). Without MIRROR1 the team writes robot two's route as a second
copy with every number reflected by hand, and keeps the two equal by hand through every edit before the freeze — the
error this library exists to prevent. With it, robot two's file is one line and robot one's route is the only route.

### 0.2 What exists in the tree (opened at `d33d25d`; @8df58f3 where marked)
- **Every route number is a field coordinate.** `Routine::driveTo` (`routine.hpp` :447-455), `reverseTo` (:472-480),
  `strafeTo` (:483-486), `turnTo` (:489-494), `face` (:497-506), `faceAway` (:512-520), `followTrajectory` (:523-546),
  `curveTo` / `reverseCurveTo` (:552-575), `startAt` (:424-431); the field-vocabulary steps compute their bearing from
  `chassis_->pose()` (`bearingTo`, :1051-1058; *[R13]* :1368-1375 @8df58f3). Every step is exactly one Chassis call (banner :105-130), and
  `chassis.pose()` mid-chain is the supported idiom (:232). STEP1-B's armed legs call the Chassis's observer overloads
  (`chassis.hpp` :453-474 @8df58f3) and measure progress as sizes (`leg_progress.hpp` :8-18 @8df58f3).
- **The Chassis's field verbs** (`chassis.hpp`): `moveTo` :379, `strafeTo` :392, `turnTo` :402 ("always the short way …
  exact ±180° resolves CCW", :399-401), `followTrajectory` :413 (validates every waypoint, then one MoveToPose per leg),
  `curveTo` :458 / `reverseCurveTo` :472 (1–16 points, `CurvePoint` = `motion::CurvePoint`, :235), `drive(speeds, frame)`
  :538 (`math::Frame::{Field, Body}`, `math/frame.hpp` :36-39), `pose()` :599, `setPose` :604; below the front door:
  `lastCompleted()` :622, `deps()` :633, `scheduler()` :639. *[R13] The same @8df58f3 (the base): `moveTo` :419,
  `strafeTo` :432, `turnTo` :442 (its comment :441), observers :453 / :464, `followTrajectory` :477, `curveTo` :522,
  `reverseCurveTo` :536, `CurvePoint` :262, `drive` :602, `pose()` :663, `setPose` :668, `lastCompleted()` :686,
  `deps()` :697, `scheduler()` :703 / :707.* F6 is FROZEN with an additive path and a signature pin
  (roadmap :140; `test/f6_signature_pin_test.cpp`).
- **The frame and the angle rules (LOCKED):** F1 — +X right, +Y away from red, heading 0 along +X, CCW positive
  (roadmap :135; `pose2d.hpp` :17-18); F3 — `(-π, π]`, an exact antipode resolves +π (roadmap :137; `angle.hpp` :7-13).
- **The entry and the runner:** `StartPose` FIELD / RELATIVE (`auton_registry.hpp` :157-200), `AutonContext` carries
  the entry (:215-247), `AutonEntry` (:249-284, appendable — its own banner asks for unset fields to be refused);
  `runAuton` seats position AND heading on one tick (`match_runner.hpp` :233-237), then runs `entry.routine(ctx)` (:251) or
  a route through `PathRunner` (:255), and the park `entry.endAction(ctx)` (:275) inside the guard.
- **The wall reset:** the everyday call refuses a non-FIELD start first (`auton_wall_reset.hpp` :112), finds the named
  side's sensor (:126), and samples through `chassis::detail::runWallReset`, which reads the LOCALIZER, never the
  Chassis's pose (`wall_reset_call.hpp` :84). `WallSide` Left/Right (`wall_reset.hpp` :92-97), `FieldWall` (:153-190 —
  an author's own line `FieldWall::line`), `WallResetRequest` (:300-316). The season walls (`override_walls.hpp`
  :160-176): every wall carries the same keep-outs except the Loaders, which sit only on the red and blue walls and lie
  INSIDE the Load Zone keep-out every wall carries (58.76 in from centre vs the zones' 47.1 in, :97-114, :152-153).
- **The Toggle step (AF1S-A1, worktree `af1s-a1`, merging after VIS1-A2):** `ToggleId` RightWall / LeftWall / BlueWall /
  RedWall in `kTogglePlaces`' order (`vision/override_elements.hpp` :92-131 there), the Quadrant colours RED, BLUE, BLUE,
  RED (:121, HA-243), `ToggleVisitRequest::toggle` (`toggle_visit.hpp` :222), the WRONG WALL check toward the named
  Toggle's wall (*[R13]* :775-784; the direction it checks against, `towardWallRadians`, :668-675), the estimate read
  from the localizer (:851) and the drive in/out as a scheduler-level
  MoveToPose (:1065) — below the front door; the explicit form takes `ctx.entry` (:1490).
- **The screens:** the menu label "<name> <KIND>" (`auton_menu.hpp` :419, budget static_asserts :99-102); the START lines
  (`auton_tester_panel.hpp` :593-594; `competition_screen.hpp` :492-494). The competition program's auton task unwinds on
  its own stack (`competition_program.cpp` :6-21).
- **The two robots' files (worktree `rob1-int`):** robot one's starter autons say "THE SKILLS ROUTE GOES HERE"
  (`src/team/robot_one_autons.cpp`); a build's autons are chosen by `autonsFor(kRobotId)` (`src/common/this_robot.hpp`
  :58-71); gate step 8 refuses a robot package linking another robot's registration or carrying its file paths
  (`tools/src_build_gate.py` :384-386; symbols :1703-1708, *[R13]* strings :1709-1713). The twin (ROB1-BT) re-makes robot two from robot one's region (G1 brief :558-561; HA-256 "the twin
  is structurally identical").
- **What does NOT exist:** any reflection in the library (`git grep -i 'mirror\|reflect' d33d25d -- include` finds only
  unrelated uses — a CRC's "reflected polynomial", calibration's "mirror-mounted" motors, vendor headers); G3a's R.8 named two flips — the midline and the half turn — and left the field's to be read
  (:812-835); neither is the skills one (D1).

### 0.3 What the manual gives (v2.0 read; v2.1's SG1 and VURS re-read online 2026-11-13)
- **Who starts where:** `<RSC2>`a (:2020-2024, V5RC: the Quadrant adjacent to the red station), replaced for VEX U by
  `<VURS3>` (:3560-3562: both robots in red's legal starting positions, "All other portions of rule <SG1> apply") with
  `<SG1>`d (:1067, not sharing a Quadrant) and f (:1069, on their Alliance's side of the Autonomous Line).
- **The field:** A10 (`img/a-105.png`) — the goals at 23.11 / 46.66 / 70.20 / 93.75 / 117.30 in from the drawing's
  lower-left corner; red goals at (23.11, 46.66) and (46.66, 23.11) — mirror images across the drawing's
  lower-left-to-upper-right diagonal, as is every other goal pair. With the red station on the drawing's left
  (A10's red tape, FO-1), F1 x = 70.205 − Y_d, y = X_d − 70.205, so that diagonal is F1's **y = −x**.
- **The transform table (the guide will carry it):** heading 0 → −90, 90 → 180, 180 → 90, −90 → 0, 45 → −135, −45 →
  −45 (the axis itself), 135 → 135 (P3). A point on the axis (x = −y) maps to itself.
- **Tolerance:** `<T5>`a — field elements within ±1.0 in of nominal (:2957-2960) — HA-258.

### 0.4 The probes (`.claude/coordinator/scratch/brief-MIRROR1/`, run by this writer 2026-11-13; every binary under `ulimit -v 4 GiB` + `timeout`)
Built by `build.sh` against `tree/` (`git archive 8df58f3`) with the suite's warning flags. The reflections are the
PROBE's own formulas, independent of any future library code.
- **P1 — the path IS the reflection** (`probe_path.cpp`, `out/p1_path.txt`). A tank on the motion rig's host test
  plant runs a 14-step route (driveTo, face, driveTo, turnTo 45°, moveTo, faceAway, reverseTo, curveTo with an end
  heading, reverseCurveTo with an end heading, a two-leg followTrajectory, face, hold, pause, driveTo) from (−36, −58,
  0°); robot two runs the same route with every number reflected by hand from the reflected start. *[R4] P1 is a TANK
  only, and has NO `strafeTo`, NO `drive()` and NO STEP1-B trigger or leg observer — "every kind" in the first print
  was wider than what ran; those are T2's (the suite, the X-drive rig), and the triggers are P6's.* Pace by pace over
  2,063 paces: **max |reflect(robot one) − robot two| = 8.8e-13 in and 3.8e-12°**; both 14 of 14 steps, the same
  2,063 paces. A mirrored mirror of four sample poses returns the original within 1.4e-14.
  **B — a 9 % weak RIGHT side on both robots** (the robot is not mirrored): both finish 14 of 14; robot two's path
  strays at most **0.56 in** from the reflected path of robot one, its time differs by **0.44 s** (21.33 s vs 21.77 s),
  the ends agree within 0.02 in. **C — the seam:** a first `turnTo` of exactly 180° from the seated heading: both robots
  turn COUNTERCLOCKWISE (robot two at −21.25° at pace 20 where the reflection is −158.75°); the paths stay within
  0.155 in, the ends within 0.09 in (D6). **D — RELATIVE:** the route from (0, 0, 0) mirrored as (x, −y, −h): exact to
  2.3e-13 in, 14 of 14 (D7).
- **P2 — the wall reset on the mirror wall** (`probe_wall.cpp`, `out/p2_wall.txt`), on the robot built from a robot
  file with side sensors (`test/wall_robot_rig.hpp`), through `competition::runAuton`, FIELD starts, the estimate 1 in
  off. Robot one, along the red wall, RIGHT sensor: `red wall y SEATED (RIGHT sensor): -54.70 -> -55.71 (moved -1.01)`.
  Robot two at the reflected pose (55.705, 36, −90°), the right wall with the LEFT sensor: `right wall x SEATED (LEFT
  sensor): 54.70 -> 55.69 (moved 0.98)`. Robot two's three WRONG requests — the unmirrored one (red wall, RIGHT), the
  mirror wall with the sensor not swapped (right wall, RIGHT), the wall not mirrored with the sensor swapped (red wall,
  LEFT) — each `REFUSED not square` (90°, 180°, 90° off), nothing seated.
- **P3 — which transform is the field's** (`probe_field.py`, `out/p3_field.txt`): A10's goals, the four Toggles with
  AF1S-A1's Quadrant colours, the Loaders, the Load Zones, A16's tag numbers, red's side — against six candidate
  transforms. Only the y = −x reflection keeps 9/9 goal colours, 4/4 Toggle colours, red's side red, and swaps the two
  red Quadrants; it keeps 0/4 Loaders, 2/4 Load Zone colours, 1/9 tag numbers (D1).
- **P4 — the Toggle id under the mirror** (`probe_toggle.cpp`, `out/p4_toggle.txt`, against AF1S-A1's header copied to
  `af1s/`): right wall (RED) ↔ red wall (RED), left wall (BLUE) ↔ blue wall (BLUE); AF1S's WRONG WALL direction for the
  image Toggle equals the reflected direction for every Toggle (one pair as −180° vs 180°, the same heading — §8.3).
- ***[R1]* P5 — the two robots AT THE SAME TIME** (`probe_meet.cpp`, `out/p5_meet.txt`; built from the checker's
  `check/probe_meet_chk.cpp`, whose numbers it reproduces: centres 0.32 in apart at pace 1927, 4.09 in with the weak
  side). It adds each robot's 15 in SQUARE (RPV1's footprint) and its distance from y = −x. **P1's route (A):** the
  squares TOUCH from pace 1014 (10.1 s, step 8's curve toward (12, −18)) and robot one's square crosses the diagonal by
  up to 23 in — a crash twice over; **B** (9 % weak side) the same. **Route G** (the same 14 step kinds, every point
  kept on robot one's side — `moveTo (−2, −30, 45°)`, a curve to (12, −32, −30°), the last drive to (10, −44)): 14 of
  14, the squares at least **8.43 in** apart = **2 × 4.21 in**, robot one's least clearance from the diagonal — the
  pure-mirror identity (a point of one square and a point of its image are at least the sum of their distances from
  the line apart); with the weak side **8.75 / 4.26 in**, both 14 of 14. (Case D, RELATIVE, is not a field meeting:
  both runs start at their own (0, 0, 0).)
- ***[R4]* P6 — the triggers** (the checker's `check/probe_trigger_chk.cpp`, RE-RUN by this writer 2026-11-13): a
  STEP1-B leg observer on a tank, robot one's leg and its reflection by hand — a 24 in drive, a 63° turn and an exact
  180° turn: 408 paces in all for each robot (154, 110, 144 ticks per leg on both); the progress fraction equal to ≤ 3.82e-14; 60 % reached on the same tick (32, 23, 32).
- ***[R1, R4]* Figure VURS1 by pixel** (`manual/vurs1_field.png`, 1138 px square, a few lines of PIL over its yellow
  blobs): the four single yellow Pins at pixels (199, 936), (383, 753), (749, 387), (931, 204) — each within 3 px
  (≈ 0.4 in) of the drawing's lower-left-to-upper-right diagonal, i.e. ON F1's y = −x: two on red's side at
  ≈ (23.5, −23.5) and ≈ (47, −47) in, two on blue's. The four Pin crosses lie on the Autonomous Line, identical, each
  pair the other's image (the checker's crops, viewed).

---

## 1. Scope — IN (every new and changed file)

### 1.1 The run
**MIRROR1-A — one run** (D9). Base: main at its tip when launched (at least `8df58f3`, STEP1-B). If AF1S-A1 has merged
by then, the Toggle swap is in it; otherwise it is **MIRROR1-B** (~¼ day) after AF1S-A1 merges — or AF1S-A2 carries it.
*[R11]* Robot two's registration line is the STUDENTS' unless the team lead rules otherwise (D2's question, §6.3).
*[R2]* **Build order inside the run, with a cut line** (the DOCS3-A brief's device): (1) the reflection, the Chassis's
mirrored run, the runner, the registry, `isMirrored` and the pair check with their tests — the part robot two cannot
run a mirrored route without; (2) the wall reset's and the Toggle's swaps; (3) the screens' MIR words; (4) the pages.
If a usage stop leaves (1)–(2) done and the freeze near, the coordinator may merge them with (3)–(4) following as a
docs-and-screens run; (1) alone does not merge (a wall reset in a mirrored run would be refused "not square", P2).

### 1.2 Library files
- **`include/shulib/math/reflection.hpp` — NEW, PURE.** `math::Reflection`: the identity, or a reflection across a line
  through the origin at a heading that is a multiple of 45° (so the coefficients are exactly 0 / ±1 and the
  reflection of a reflection is exact in position). Applies to a point, a `Pose2d`, an `Angle` (`2α − h`, re-wrapped), and
  a `ChassisSpeeds` in either `math::Frame`. Precondition on any other line. Includes only math/ and units/. (Not in
  the locked `math/frame.hpp`, whose banner allows "new vocabulary only".) *[R9]* Every result it produces is
  normalised with `+ 0.0` (the idiom `toggle_visit.hpp` :674 already uses), so a reflected zero is `0.0`, never `-0.0`
  — robot one's x = 0 must not print as robot two's "y = -0.0" on a screen or in a log.
- **`include/shulib/chassis/chassis.hpp` — CHANGED, F6-additive.** A mirrored run: a scope type (suggested
  `chassis::MirroredRun`, RAII) that sets the Chassis's reflection and restores the identity on destruction; opening one
  while one is open, or while a motion is active, is a precondition; a read-only `reflection()`. While not identity:
  every field verb listed in D3 reflects its arguments FIRST (so the observer overloads' targets, the curve's checks and
  TANK1's steering all see the real field), `pose()` reflects back, `curveTo` copies at most 16 reflected points into
  local storage (a span outside 1..16 passes through untouched so today's precondition fires) — *[R6] NOT a plain
  `std::array<CurvePoint, 16>`, which does not compile: `CurvePoint` deliberately has no default constructor
  (`motion/curve_path.hpp` :93-110, F6-pinned). Build the array from `CurvePoint`'s own two constructors (an
  `index_sequence` helper constructing slot i as the reflected point i, or a placeholder `CurvePoint{0_in, 0_in}`
  past n, with only the first n handed on as the span), or reflect each point as the curve's path is built; never add
  a default constructor* — `followTrajectory` reflects each leg as it starts, and a `turnTo` — *[R5]* or a `moveTo` /
  trajectory leg — whose heading change at its start is within 1° of 180° logs the one `[MIR]` Warn (D6). **When
  identity, each verb takes today's code path untouched — a branch, never "apply the identity"** (re-wrapping a heading
  can move bits, §8.6). New members pinned in `test/f6_signature_pin_test.cpp`; the F6 row gains one sentence.
- **`include/shulib/competition/field_mirror.hpp` — NEW.** `competition::FieldMirror`: its name ("the skills
  diagonal"), its `math::Reflection`, and the season's two image functions (a wall → the season wall at the reflected
  line, with its real name and keep-outs, or the geometric reflection for an author's own line; a Toggle id → its
  image). `competition::mirrored(entry, mirror)` (D2, D5's toggle and refusal). *[R1]* `competition::isMirrored(ctx)`
  — true while `ctx.entry` carries a mirror — so the shared route can send ONE robot to what lies on the diagonal
  (D11 (a) 3).
- **`include/shulib/competition/override_mirror.hpp` — NEW, season data** (`competition::override2026::kSkillsMirror`),
  citing `<VURS3>`, `<SG1>`d/f and A10 as §0.3 does; `static_assert`s that the reflection maps each season wall onto a
  season wall and (once AF1S-A1 is in) each Toggle onto a Toggle of the same Quadrant colour; a test that the reflected
  season walls' kept-out spans equal the real image walls' (the Loader spans differ only inside the Load Zones).
- **`include/shulib/competition/auton_registry.hpp` — CHANGED (not frozen).** `AutonEntry` APPENDS `mirror` (default:
  none — every existing initializer compiles); `checkAutonEntry` refuses a mirrored name over 25 characters (D8);
  `effectiveStart()` unchanged (the runner reflects).
- **`include/shulib/competition/match_runner.hpp` — CHANGED.** For a mirrored entry: the re-seat writes the REFLECTED
  start straight into the localizer (position and heading, one tick, as today — FIELD by the entry's mirror, RELATIVE
  by the robot-frame flip, D7); `result.seated` is robot two's real start; one `chassis::MirroredRun` wraps the routine
  (or route) AND the end action; the COMP lines name the mirror and both poses (D8 — *[R8]* the re-seat line's mode
  word `FIELD MIRRORED` in its existing 160-byte buffer, plus one second line for robot one's start; both ASCII and
  under the sink's 200-byte cut); `result.endEstimate` and every pose the record keeps are read after the scope
  closes or from the localizer — the real field. Unmirrored: byte-identical (§2.2 T10).
- **`include/shulib/competition/auton_wall_reset.hpp` — CHANGED.** Under `ctx.entry`'s mirror: the request's wall →
  its image, its sensor → the other side, then today's checks; the `[WALL]` line names the REAL wall and sensor and
  carries the mirrored note *[R8]* INSIDE the sensor's parenthesis — `(LEFT sensor, mirrored from red wall RIGHT)` —
  handed to `chassis::detail::runWallReset`'s line through a defaulted parameter (the explicit form's lines and
  every unmirrored line byte-identical; `test/wall_reset_line_test.cpp`'s pins unchanged, a mirrored case added).
- **`include/shulib/chassis/wall_reset_call.hpp` — CHANGED.** The explicit form refuses (a precondition naming
  `competition::resetFromWall(ctx, …)`) when `chassis.reflection()` is not identity (D5).
- **`include/shulib/competition/toggle_visit.hpp` (AF1S-A1's) — CHANGED, in whichever lands second (§1.1).** The
  request's Toggle id → its image under `entry.mirror` before rule 1; the `[TGL]` line names the real Toggle.
- **`include/shulib/competition/auton_menu.hpp` — CHANGED.** A mirrored entry's label "<name> <KIND> MIR".
- **`include/shulib/teleop/teleop_loop.hpp` — CHANGED (one check at the loop's entry).** *[R12] The first print made
  it a precondition in `runDriverControl` — a refusal that, if ever hit in a match, leaves the driver with NO drive
  for the whole driver period; and it named the overloads (:454-487), not the one body.* Now: in
  **`detail::driverLoop`** (:241 — every overload reaches it: :459, :478, and through them :491, :502), before the
  first tick: if `chassis.reflection()` is not identity, ONE `[MIR]` Warn line (`an auton's mirror was still on at
  driver control -- cleared; the driver drives the real field (a library defect: please report it)`) and the mirror
  CLEARED through a detail-level call on the Chassis — the driver always drives the real field. It cannot happen in
  the competition program (opcontrol starts only after the auton task's FINISHED, which comes after the scope
  closes — `competition_program.cpp` :12-14); the check is for a defect, and T4 pins both the line and the clearing.
- **`include/shulib/version.hpp` — `kApiMinor` 9**, its paragraph (F6-additive; nothing to do).

### 1.3 Robot program files (`src/`)
- `src/programs/auton_tester_panel.hpp` and `src/programs/competition_screen.hpp` — the START line's "MIR" (D8), the
  worst-case width re-checked by their screen tests. No other `src/` file: *[R11]* robot two's line is the students'
  (D2's question); the starter file and its comment are ROB1-BT's.

### 1.4 Tests (NEW unless noted) and the shared vectors
`test/reflection_test.cpp`, `test/chassis_mirror_test.cpp`, `test/mirrored_auton_test.cpp`,
`test/mirrored_wall_reset_test.cpp`, the Toggle case in `test/toggle_visit_rules_test.cpp` (CHANGED, after AF1S-A1),
*[R10]* `test/goldens/mirror/skills_mirror_vectors.txt` (one vector per line) + its ~20-line reader in
`reflection_test.cpp` (D10), *[R1]* `test/mirrored_pair_check.hpp` (the pair check: two runs, 15 in squares, the gap,
the verdict naming time, positions and steps — one function the suite, RPV1's follow-up and any later tool share) +
`test/mirrored_pair_test.cpp` (T11) + the pair check over robot two's registered mirrored entries beside T9's runs
in `test/autons_test.cpp` (CHANGED), *[R8]* a mirrored case in `test/wall_reset_line_test.cpp` (CHANGED), the
menu/screen cases in their existing tests (CHANGED), the cookbook recipe `cookbook-09a` in
`test/cookbook_examples_test.cpp` (CHANGED; *[R7]* 08 is AF1S-A2's), the F6 pin (CHANGED), compile refusals none
(every refusal is a run-time sentence or a precondition — D5 (c)).

### 1.5 The API number
F6 grows additively (a new type and members; every verb unchanged unless a mirrored run is open) and F10 does not
change at all: **minor bump to 2.9** (STEP1-B took 2.8 at `8df58f3`). If another F6/F10 run merges first, the next free
minor. Changelog "API 2.9 — Nothing to do".

---

## 2. Step 0, tests, mutations, definition of done

### 2.0 Step 0 at the robot — no new code (the coordinator puts these to the builder)
0.1 Where each robot starts (D1's question). 0.2 Every part robot one's route uses at the field: its sideways offset
from the robot's centre line (intake, flipper, cascade, camera's `left`, both side sensors' `forward`) — HA-259.
0.3 Both side sensors' mounts measured (L6's D6 — robot two will use the side robot one never does). 0.4 After
MIRROR1-A and ROB1-BT: robot two's mirrored route in the Auton Tester, START MIR read against where the drivers placed
it, END against the tape. *[R1]* **0.5 Both robots run the pair TOGETHER on a field at least once before the 21st** —
first with the drive speed turned down, someone at each robot's stop — watching the diagonal: the event must not be
the first time the two run at once (HA-260).

### 2.1 The run
| Run | Contents | Size | Waits for |
|---|---|---|---|
| **MIRROR1-A** | §1.2–§1.4, §3 | *[R2]* **~3–4 builder-days** (was ~2–3) + ~½–1 day review/merge — at the peers' measured pace ~1½–2 calendar days with stops (D9) | the team lead's go on D1–D11; a free slot (max 3; §6.1b) |
| MIRROR1-B (only if AF1S-A1 merges after A) | the Toggle swap + its test | ~¼ | AF1S-A1's merge (or AF1S-A2 carries it) |

### 2.2 Tests (ADVERSARIAL — each tries to break ONE rule)
- **T1 the reflection, against an INDEPENDENT formula** (the test's own rotate–flip–rotate with `cos`/`sin`, never the
  0/±1 shortcut) over a seeded sweep of 10,000 poses including headings ±180°, 0, −0.0, the axis itself, and |x|,|y| up
  to 1e4: positions exact, headings within 1e-12 rad, compared WRAPPED; a reflected reflection equals the original
  (positions EQUAL — *[R9]* not bit-exact: `+ 0.0` turns an input −0.0 into 0.0, which compares equal — and no result
  is ever −0.0, checked with `std::signbit`); a line not a multiple of 45° refused; `ChassisSpeeds` in both frames;
  *[R10]* every line of `skills_mirror_vectors.txt`, and a malformed line refused by the reader by line number.
- **T2 every Chassis verb mirrored = the reflected route** (P1's shape, in the suite): the same route run unmirrored
  from S and mirrored from the reflection of S, pace by pace ≤ 1e-9 in / 1e-9 rad on a tank AND on the X-drive rig
  (strafeTo moves there; on the tank both time out identically); every verb in D3 including both observer overloads
  with a STEP1-B trigger of each kind firing on the same pace in both runs; `drive` in both frames; `pose()` reflected
  back; `setPose`. *[R5]* The X-drive route has NO `moveTo` / trajectory leg with an exact 180° heading change (it would
  rotate counterclockwise on both robots and fail the 1e-9 check by design — that case is T3's).
- **T3 the seam pinned:** an exact-180° first turn — robot two turns counterclockwise, the path within 0.2 in of the
  reflection, the end within 0.1 in, the one `[MIR]` line; a 179.9° and a 180.1° turn mirrored exactly with no `[MIR]`
  line below 179°. *[R5]* And on the X-drive rig, a `moveTo` with an exact 180° heading change: both robots rotate
  counterclockwise, the one `[MIR]` line; its divergence MEASURED first and pinned with its number (as P1 C measured
  the turn's).
- **T4 the scope:** cleared after a normal end, the deadline's cut, an abandon, a throw from the routine and a throw from
  the end action (each followed by an unmirrored run that must be byte-identical to a fresh one); nesting refused; opening
  with a motion active refused; *[R12]* the driver loop, handed a Chassis with a mirror left on, logs the one `[MIR]`
  Warn, CLEARS it, and its field-relative drive command on the first tick equals a fresh unmirrored loop's — not a
  refusal.
- **T5 the runner:** a mirrored FIELD entry seats (−y, −x, −90° − h) in position AND heading; RELATIVE seats (0,0,0) and
  mirrors (x, −y, −h); a route entry mirrored through PathRunner; the park mirrored; `mirrored(mirrored(e, k), k)` runs
  byte-identical to `e`; a second, different mirror refused by name; a 26-character mirrored name refused by name.
- **T6 the wall reset on the robot rig** (P2's shape): robot one's request on robot two SEATS the right wall with the
  LEFT sensor, its correction the reflection of robot one's; the `[WALL]` line names the real wall and side; an own
  wall line reflected geometrically; the explicit form under a mirrored run is a precondition naming the everyday form.
- **T7 the Toggle** (after AF1S-A1): `kRedWallVisit` on robot two visits the right wall's Toggle, "ours" RED both times;
  its WRONG WALL check passes with the reflected heading; the four images keep their Quadrant colours.
- **T8 the screens and the lines:** "<name> SKILLS MIR" fits at 25 characters; *[R9] the screens' OWN worst cases,
  not a weaker one:* "START MIR (-999.9, -999.9) -179.9 deg" on the Auton Tester (`auton_tester_panel_test.cpp` :224's
  values) and "START MIR (-9999.9, -9999.9) -179.9 deg" (39) on the competition screen (`competition_screen_test.cpp`
  :455's) fit; the START MIR text PINNED for a start with a zero coordinate (robot one at x = 0 → robot two's y prints
  "0.0", never "-0.0") and for a mirrored RELATIVE start ("START MIR (0.0, 0.0) 0.0 deg"); *[R8]* the two COMP lines
  at their worst case (147 and 130 characters measured here; the test measures its own) under 160 and under the
  sink's 200-byte cut, every mirrored `[WALL]` line form at its worst case under 200 with the mirrored note intact,
  the `[MIR]` lines, all ASCII; every unmirrored label and line byte-identical.
- **T9 robot two's mirrored autons run in the suite** (from ROB1-BT): every entry of robot two's registry runs on the
  host test plant, as robot two's do today — *[R1]* and every MIRRORED entry also passes the pair check (T11's
  function) against its unmirrored self.
- **T10 NOTHING ELSE CHANGES:** CURVE1's `curve_cv16` hash (`test/curve_unchanged_route.hpp` :1-11, pinned in
  `motion_curve_test.cpp`) unchanged; a NEW runner-level hash — every auton of today's robot two registry and the
  acceptance run through `runAuton`, every log line and the true pose after every pace — MEASURED at MIRROR1-A's merge
  base before any edit (the CV16 method) and pinned.
- ***[R1]* T11 the pair at once** (`test/mirrored_pair_test.cpp`, over the check in `test/mirrored_pair_check.hpp` — the
  check returns a verdict; the test asserts the verdict, so a planted crash is a PASSING test of the check, and only a
  registered team route that crashes turns the suite red): (i) P1's route — FAILS, naming 10.1 s, the curve step and
  both positions; (ii) P5's route G — PASSES with the least gap equal to 2 × robot one's least clearance from the
  diagonal (to 1e-6 in) and ≥ 6 in; (iii) the same with a 9 % weak side on both plants — PASSES; (iv) a route whose
  robot-one-only branch (`!isMirrored(ctx)`) visits the centre goal while robot two waits on its own side — PASSES;
  the same branch timed so robot two crosses the centre while robot one is there — FAILS; (v) a RELATIVE mirrored
  entry — skipped, by name; (vi) the gap constant at 6 in, and one route 5.9 in apart FAILS while 6.1 in PASSES.

### 2.3 Mutations (each must turn the named test RED; the coordinator re-runs them)
MM1 the field heading as −h (the relative rule) → T1, T2. MM2 `pose()` not reflected back → T2 (bearings). MM3 the
scope's destructor emptied → T4. MM4 `drive(Body)` keeps the turn rate → T2. MM5 the wall mapped, the side not → T6.
MM6 the Toggle images crossed (right ↔ left) → T7. MM7 curve headings not reflected → T2. MM8 the observer overload
not reflected → T2 (trigger pace). MM9 `mirrored` does not toggle back → T5. MM10 the runner seats the unreflected
start → T5. MM11 RELATIVE uses the diagonal → T5. MM12 the identity case "applies" the identity (re-wraps headings) →
T10. MM13 only the first followTrajectory leg reflected → T2. MM14 the end action outside the scope → T5.
*[R1]* MM15 the pair check compares robot two with the REFLECTION of robot one instead of robot one (a check that can
never fail) → T11 (i). MM16 `isMirrored` always false → T11 (iv). *[R9]* MM17 the `+ 0.0` normalisation dropped →
T1, T8.

### 2.4 Definition of done
Suite green through `tools/suite_run.py`; every robot build links (`tools/src_build_gate.py`); the ARM header gate; the
compile-refusal harness count unchanged; MM1–MM17 red; the docs checks (§7); the progress log's prose review; no
hardware claim; HA-258..260 registered with their code sites.

---

## 3. Documents — in the same change as the code (pace rule 3), ≤ ~5 substantial public pages
- **Guide 02 (the field)** — a section "The field's mirror": the manual's reason (both robots red, two red Quadrants),
  the transform table of §0.3 with a worked start pose, what the mirror cannot carry (Loaders, the red/left Load Zone,
  the robot itself); *[R1]* what lies ON the diagonal (the centre goal, the two yellow Pins, the corner Load Zone) and
  the 6 in band, drawn.
- **Guide 17 (the competition program)** — after "The two start modes" (:57): "Two robots, one route: a mirrored
  auton" — robot two's one line, what flips (D4's table), START MIR / "<name> SKILLS MIR", the `[MIR]` line, what reads
  the real field (`deps()`, `scheduler()`, `lastCompleted()`, *[R3]* the camera's inputs), the refusals; *[R1]* **"Both
  robots at once"**: the 3 in rule, the pair check's failure sentence and how to read it, `isMirrored(ctx)` for a
  one-robot step and why its timing then matters.
- ***[R7]* Cookbook 09 (NEW) "One route, two robots"** (08 is AF1S-A2's — `af1s-a1/test/cookbook_examples_test.cpp`
  :13-14) — write once, run mirrored: the shared function, robot one's line, robot two's line, a wall reset and a
  Toggle visit inside, *[R1]* one `isMirrored` step, quoted verbatim from `cookbook-09a` in `cookbook_examples_test.cpp`
  and run (and passing the pair check); the cookbook README's index gains its row.
- ***[R8]* Guide 11 (reading the diagnostics)** — the `[MIR]` lines (the 180° Warn, the driver loop's cleared-mirror
  Warn), the `[WALL]` line's mirrored note (its section :512-527 quotes lines pinned by `test/wall_reset_line_test.cpp`
  — the mirrored line joins them, pinned the same way), the two mirrored COMP re-seat lines; the tag list (:246) gains
  `MIR`.
- **Changelog** — API 2.9. **Guide 14** — three lines (Loaders and side-mounted parts; the 180° tie; *[R1]* a pure
  mirror cannot take what lies on the diagonal). **Glossary** — "mirror (a mirrored auton)". **API pages**
  regenerated. Internal: `MIRROR1-PROGRESS.md`, `REMAINING-WORK.md`, `build-order.md`, the HA register,
  PROJECT-BRIEFING (generated).

---

## 4. Hardware Assumptions — HA-258..HA-260 ONLY
| ID | Belief | Kind | Code site | Settled by |
|---|---|---|---|---|
| **HA-258** | The event's field is set up to Appendix A *[R1] and Figure VURS1*, so the red wall's and the right wall's Quadrants are mirror images across y = −x (goals, Toggles, pins, tape) — each element within `<T5>`a's ±1.0 in, so a mirrored target can land up to ~2 in from robot two's element where robot one's was tuned to its own — *and the centre goal, two yellow Pins and the red/right Load Zone sit ON the diagonal (VURS1 by pixel, ≈ ±1 in)* | reasoned (manual drawings A10, A11, VURS1) | `override_mirror.hpp`'s `kSkillsMirror` | tape the two red goals and two Toggles from their walls on the event's field |
| **HA-259** | Robot one's build is left-right symmetric for every part a route uses at the field; a part off the centre line lands twice its offset from the reflection of robot one's (the camera's `left`, the two side sensors' `forward` included) | unknown | `competition::mirrored`'s doc; the guide | Step 0.2 |
| **HA-260** | The drivetrain's own left/right differences are NOT mirrored: robot two's arrivals differ from the reflection of robot one's by its own asymmetry (host test plant, 9 % weak side: 0.56 in, 0.44 s — P1 B) — *[R1] and each robot's real position near the diagonal stays within half the pair check's gap (3 in) of its plan, so a route that passes the 6 in check keeps the two robots apart on the field* | invented magnitude (the host plant's; the 3 in is a chosen margin, unmeasured) | `chassis.hpp`'s mirrored-run doc; *[R1]* `kMirrorGapIn` in `test/mirrored_pair_check.hpp` | Step 0.4 on both robots; *[R1]* Step 0.5 (both together) |

---

## 5. OUT, and explicitly rejected
- The head-to-head red ↔ blue swap (the half turn; ALLY1 and G3a's D17 after the 21st — this brief's P3 settles its
  axis for them). Any mirror other than a 45°-multiple line.
- Mirroring the robot (swapping its mechanisms' sides, its motors): the robot is one physical build.
- Steps at Loaders: refused? No — the library cannot know a step is "at a Loader"; told (D4, §8.8), never guessed.
- Mirroring a route by rewriting C++ source text (fragile: expressions, helpers, variables); RMP1's model-level copy
  is the fallback only (D10 (b)).
- `registerMirrored` for all of robot one's autons (D2 (a′)) — later, with ROB1-BT's gate re-ruling.
- Changing F3's tie (D6 (b)); any change to the Routine (F10).
- A replay of a real run beside the plan, or overhead tracking (the team lead, 2026-11-13: after the 21st).
- *[R1]* A collision check ON the robot (D11 (d), rejected); a start delay for robot two (D11 (b), additive later).
- *[R3]* Mirroring the camera's field inputs (a piece target, a caller's pose): documented, not done — drive-to-piece
  (L4) maps its own target when it is built (§6.6).

---

## 6. Coordination — every in-flight run (re-checked 2026-11-13 ~02:10 — *[R13] the first print said "~03:30", an hour not yet reached*; every worktree read only)

### 6.1 In flight
| Run | State | Files it shares with MIRROR1-A | Rule |
|---|---|---|---|
| STEP1-B | COMMITTED `8df58f3` (API 2.8) | `chassis.hpp`, `routine.hpp` | MIRROR1-A bases on it; reflects its observer overloads (§1.2) |
| VIS1-A2 | squash STAGED on main | none (camera, `robot.hpp`, guide 20) | none |
| AF1S-A1 | done, merges after VIS1-A2 | `toggle_visit.hpp`, `override_elements.hpp` | the Toggle swap goes in whichever lands second (§1.1) |
| ROB1-INT | running (`rob1-int`) | none in `include/`; `src/programs/*_program.cpp` (MIRROR1 touches `*_panel.hpp` / `*_screen.hpp` only) | MIRROR1-A re-reads `src/programs/` at its tip if ROB1-INT merged |
| RPV1-A | running (`rpv1-a`) | `test/auton_tester_rig.hpp` (MIRROR1 does not edit it); *[R1]* its 15 in footprint constant (`route_preview_model.hpp` :48) — the pair check uses the same size | none; §6.4 — if RPV1-A has merged, the pair check includes its constant; if not, it defines its own `15.0` with a comment, and the second to merge makes it one |
| RMP1 brief | next slot after this one | — | D10; the shared vectors file; D11's drawing |
| *[R2]* ROB1-BT (G1-B + G1-T) | queued FIRST (NOTES :993); waits for ROB1-INT's merge and the builder's word that robot two is rebuilt | robot files and `src/team/` (MIRROR1 touches neither) | ~1¼–1¾ builder-days (+~1 verification; G1 brief §2.1) |
| *[R2]* RMP1 build | queued after MIRROR1 | the vectors file (reads it) | unsized until its brief is written |
| *[R2]* ROB1-C (G1-C) | queued | none | ~½ (+~¼) |
| *[R2]* AF1S-A2 | queued; after AF1S-A1, L6-A2, VIS1-A2 merged | `toggle_visit.hpp` (the everyday form — reads `ctx.entry`'s mirror) | ~1¼–1½ (+ verification) |
| *[R2]* DOCS3-A1 | queued after the robot-code runs (the team lead, 11-12/11-13: documentation is the priority in QUALITY, may pass the freeze — docs change no robot code) | the guides (§6.2); quotes cookbook 09 | ~4¼–5¼ + ~1 review |

### 6.1b *[R2]* The three slots from now to the ~Nov 18 freeze (an honest plan, not a promise)
Measured pace (NOTES, 11-11 to 11-13): a run sized ~3–4 builder-days took ~5–8 hours of agent time, spread over
~1½–2 calendar days by usage-limit stops and the team lead's pauses; a local merge took ~2 hours (STEP1-B) to a day
(ROB1-A needed its own integration run). Three 5-wide stops in ~30 h on 11-11 are why it is 3 slots now.

| When (2026) | Slot 1 | Slot 2 | Slot 3 |
|---|---|---|---|
| **Fri 11-13 now (~02:00)** | ROB1-INT (running) | RPV1-A (running since 11-12 late; ~2½ + ~1) | this brief's re-check |
| Fri ~03:00 | ROB1-INT | RPV1-A | the RMP1 brief (~2–3 h) |
| Fri daytime — **the team lead's go on D1–D11** | ROB1-INT ends → merged → **ROB1-BT** (if the builder has said robot two is rebuilt) | RPV1-A | free from ~06:00 → **MIRROR1-A at the go** |
| Sat 11-14 | ROB1-BT | RPV1-A ends → merged → **RMP1 build** (at its go) | MIRROR1-A |
| Sun 11-15 | ROB1-BT ends → **ROB1-C** (~½) | RMP1 build | MIRROR1-A ends → merged (local, ~½ day) → **AF1S-A2** |
| Mon 11-16 | ROB1-C ends → **DOCS3-A1** | RMP1 build | AF1S-A2 |
| Tue–Wed 11-17–18 | DOCS3-A1 (may run past the freeze) | RMP1 ends → merged | AF1S-A2 ends → merged; **code freeze ~Nov 18** |

**What waits, and what slips first:** nothing above has slack for a full-day usage stop. One stop pushes MIRROR1-A's
merge to ~Nov 17 and AF1S-A2's to the freeze; the coordinator then chooses between AF1S-A2 and the RMP1 build for the
last code slot (the team lead's call — both are his priorities). The RPV1 follow-up (the banner, both robots drawn,
~¼–½) has no slot before the freeze in this plan unless RMP1's drawing covers it (§6.4). DOCS3-A1 is docs-only and
may finish after the freeze (the team lead, 11-13 ~00:40).

### 6.2 Shared documents
The changelog, guides 14/15/17, *[R8]* guide 11, *[R7]* the cookbook README's index and page 09, the HA register,
`version.hpp`'s paragraph, the generated API pages — merged serially by the coordinator; the second to merge re-reads
the first. Beside these, AF1S-A1 brings guide 11's TGL section (`af1s-a1/docs/guide/11-reading-the-diagnostics.md`
:484) and AF1S-A2 writes cookbook 08.

### 6.3 ROB1-BT (robot two = robot one's build, its own name and autons file)
MIRROR1-A gives it `competition::mirrored` and `kSkillsMirror`. **The file layout D2 (a) recommends:** the students'
skills route in `src/team/skills_route.{hpp,cpp}` (a function returning the entry), robot one's file
`registry.add(skillsRoute())`, robot two's `registry.add(competition::mirrored(skillsRoute(), kSkillsMirror))` — gate
step 8 unchanged (no robot package links the other's REGISTRATION; both link the shared route by design — ROB1-BT
confirms step 8's symbol list does not name the shared route, and that the shared file's `__FILE__` path is not on
`ROBOT_FOREIGN_STRINGS` :1709-1713). G4: ROB1-BT and MIRROR1-A write NO route content — the shared file starts with
the "THE SKILLS ROUTE GOES HERE" comment robot one's starter file carries today. *[R11]* **Robot two's line is the
STUDENTS'** (D2's question): robot two's starter file carries only a comment naming cookbook 09 and guide 17's
section; no run writes the line unless the team lead gives that go explicitly.

### 6.4 RPV1
A mirrored entry previews mirrored with no RPV1 change (it calls `runAuton`). *[R13] The first print said robot two
"gets" a row — it HAS one already* (`rpv1-a/test/route_preview_run.hpp` :120-130; robot one's is :132-144, at this
reading — RPV1-A is still building): after ROB1-BT, robot two's row is UPDATED to its new build (its mechanism list,
its registration, its autons file). A small RPV1 follow-up (~¼–½ day) adds the "MIRRORED from robot one's route"
banner and robot one's path faded beside it, and *[R1]* draws BOTH robots' 15 in squares at the same moment, the 6 in
band along the diagonal, and a mark at the first moment the pair check fails — calling the same
`test/mirrored_pair_check.hpp`, so the picture and the suite can never disagree. The page already draws the walls,
Toggles, Loaders, Load Zones and start pieces (`rpv1-a/test/route_preview_render.hpp` :49-51), so a mirrored step that
robot one ends at a Loader is visibly at a bare wall; it does NOT draw the goals — adding A10's nine goals in their
colours (P3's table) is the one drawing that would show the mirror landing on the right goals (same follow-up).

### 6.5 RMP1 (D10)
RMP1 exports robot one's route once and robot two's ONE line (written by the student who presses export — G4, D2);
its drawing of the mirrored path uses `(−y, −x, −90° − h)` (FIELD) and `(x, −y, −h)` (RELATIVE), checked against
*[R10]* `test/goldens/mirror/skills_mirror_vectors.txt` (one vector per line; MIRROR1-A writes it; RMP1 reads it by
splitting lines; if RMP1 lands first, it writes the file with this table and MIRROR1-A's T1 reads it). **RMP1's own
test set-up — node, a `package.json`, `node --test` — is RMP1's to choose and own; none exists on main today.**
*[R1]* RMP1 also draws both robots at the same moment as the student scrubs through the route, the 6 in band along
the diagonal, and flags every moment the squares come within the gap; its estimate is a planning aid — the suite's
pair check (the real library on the host test plant) is the judge.

### 6.6 G3a (after the 21st) and ALLY1
G3a's R.8 / D17 should be corrected at the plan commit: the skills pair of ONE alliance is the y = −x reflection (this
brief); the head-to-head red ↔ blue pair is the half turn (A16's tags, P3) — not the midline. Its route file's
`"mirror": true` then generates robot two's line (D2). AF1S-A2 (the everyday Toggle form) reads `ctx.entry`'s mirror.
*[R3]* **L4 (drive-to-piece, after the 21st) and ALLY1:** a route's `vision::PieceTarget{x, y}` is written in robot
one's numbers, so `driveToPiece` maps its target through `ctx.entry`'s mirror (and hands the finder the LOCALIZER's
pose, never `ctx.chassis.pose()`), and drives to the real-field fix through `chassis.reflection()` (an involution) or
below the front door. ALLY1-L4's brief :348-349 says mirroring the piece's position "is G3a D17's job" — that is the
head-to-head colour pair (the half turn); the SKILLS mirror is this brief's, and L4 must take it when it is built.

### 6.7 DOCS3-A
The beginner track's skills-route walkthrough ends with robot two's one line (after MIRROR1-A merges); DOCS3-A quotes
the cookbook recipe rather than writing its own.

---

## 7. Checks — the coordinator re-runs every one (never the report's word)
```sh
cmake --build build/test -j"$(nproc)"
python3 tools/suite_run.py green
(cd build/test && timeout 1200 ctest -R compile_refusal --output-on-failure) | tail -4
# the ARM header gate — every v2 header in ONE TU (the new math/reflection.hpp and competition season header included)
find include/shulib -name '*.hpp' | sed 's|^include/||' | LC_ALL=C sort | awk '{print "#include \""$0"\""}' > <scratch>/all.cpp
echo "int main(){return 0;}" >> <scratch>/all.cpp
arm-none-eabi-g++ -std=gnu++20 -Wall -Wextra -Wconversion -Wsign-conversion -Wshadow -Werror \
  -Os -mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=softfp -c <scratch>/all.cpp -o /dev/null -Iinclude
python3 tools/src_build_gate.py check
python3 tools/api_doc_tool.py self-test && python3 tools/api_doc_tool.py check-coverage \
  && python3 tools/api_doc_tool.py check-fresh && python3 tools/api_doc_tool.py check-examples
python3 tools/doc_staleness_audit.py && python3 tools/briefing_status.py check
# ── the brief's own ──
git diff <base> -- include/shulib/chassis/routine.hpp                      # EMPTY: F10 untouched (D3)
git diff <base> -- include/shulib/math/frame.hpp include/shulib/math/angle.hpp   # EMPTY: F1/F3 untouched
grep -nE '#include' include/shulib/math/reflection.hpp | grep -vE 'shulib/(math|units)/|<'   # empty: PURE
grep -rn -i 'simulator' include/shulib/math/reflection.hpp include/shulib/competition/field_mirror.hpp \
  include/shulib/competition/override_mirror.hpp docs/cookbook/09-*.md test/*mirror*   # empty  ([R7]: page 09)
LC_ALL=C grep -nP '[^\x00-\x7F]' include/shulib/competition/match_runner.hpp include/shulib/competition/auton_wall_reset.hpp \
  include/shulib/chassis/chassis.hpp include/shulib/teleop/teleop_loop.hpp | grep -E 'snprintf|log\(|"' # [R8] review: no non-ASCII in a log string
grep -rnoE 'HA-2(5[89]|60)' include src test | sort -u     # every id has its register row, every row a code site
```
Spot checks by hand: T10's runner hash measured at the base BEFORE the first edit (the log shows the order); MM3 and
MM12 re-run, seen red, restored.

---

## 8. Landmines
1. **Heading conventions.** F1 is the MATH convention (0 along +x, counterclockwise positive, `angle.hpp` :13) — not a
   compass. The field mirror is `−90° − h`; `−h` is the RELATIVE (robot-frame) mirror; `180° − h` is the WRONG
   left/right mirror. Never let a test check one with the other's formula.
2. **Turn direction under a reflection.** Every turn sweeps the other way, by construction — that is the mirror working.
   The one exception is the exact 180° tie (D6): both robots go counterclockwise — *[R5]* in TurnTo, AND in
   MoveToPose's heading term (`move_to_pose.hpp` :216, :386, :586) and the curve's end heading (`curve_follow.hpp`
   :271, :601), which rotate in place on a holonomic drive. A test that compares a mirrored X-drive `moveTo` with an
   exact 180° heading change pace by pace against the reflection fails by design (T2 avoids it; T3 pins it).
3. **The 180° seam.** −180° and +180° are one heading; `Angle` stores +π (`angle.hpp` :27-29). A reflected 90° gives
   −180° → stored +180°. AF1S's `towardWallRadians` returns −π for the left wall (`atan2(-0.0, -1.0)`, P4). Compare
   headings WRAPPED, never raw radians.
4. **A sensor side swap.** Robot two uses the side sensor robot one's route NEVER uses. Its mount must be measured (an
   unmeasured mount refuses MOUNT UNSET, L6's D6), and if the two sensors sit at different distances ahead of the
   centre, robot two's beam lands elsewhere along the wall — the keep-outs may refuse where robot one's passed (HA-259).
5. **A relative start.** The library cannot see where the drivers put the robot: a mirrored RELATIVE auton is right
   only when robot two is placed at the mirror image of robot one's spot. A FIELD start's START MIR line tells them where.
6. **The identity must be today's code, not a reflection that does nothing.** Re-wrapping a heading through
   `Angle::radians` can move a last bit (−0.0, ±π); T10's hash is the guard.
7. **Two frames at once.** Inside a mirrored run `chassis.pose()` is robot one's numbers and the log, the screens,
   `deps().localizer->pose()` and `lastCompleted()` are robot two's real field. Every log line a student reads names
   real coordinates; the guide says so in its first paragraph on the mirror.
8. **Loaders and Load Zones.** A route that takes Match Loads, or parks in the red/left corner's Load Zone, does not
   mirror (D4). The preview shows the bare wall; the library does not refuse — it cannot know intent.
9. **The mirror switch must never outlive the auton.** If it did, driver control would steer reversed. The scope is
   RAII, the competition program's auton task unwinds on its own stack, opcontrol waits for the auton task's FINISHED,
   and — *[R12]* never a refusal, which would leave the driver with no drive for the whole period — the driver loop
   CLEARS a leaked mirror with a loud `[MIR]` Warn (`detail::driverLoop`, T4).
10. **A mirrored run started with a motion active** would reflect a target mid-flight — refused (T4).
11. **Season data in a library header.** `kSkillsMirror` is Override's; next season adds its own header (VIS1's and
    L6's rule: season data in a season header, cited).
12. **Gate step 8.** Registering robot one's REGISTRATION from robot two's file fails the robot build gate (header
    correction 4 — most likely first through the strings half, robot one's file path); the shared route file avoids it.
13. ***[R1]* THE TWO ROBOTS RUN AT THE SAME TIME.** A reflection leaves its line where it is: robot one on the
    diagonal = robot two on the same spot at the same moment. In a pure mirror the gap between the robots is exactly
    twice robot one's clearance from the diagonal (P5) — so the check is cheap and exact; after an `isMirrored`
    branch it holds only at the planned timing. P1's own route — this brief's first evidence — crashes. Never write
    an example route, test route or cookbook route that crosses the diagonal unless it is the test that the pair
    check FAILS it.
14. ***[R6]* `CurvePoint` has no default constructor** (`curve_path.hpp` :93-110, F6-pinned): `std::array<CurvePoint,
    16>` does not compile. Build the reflected points from its own constructors (§1.2); never add a default one.
15. ***[R9]* −0.0.** Reflection negates, and `-(0.0)` prints "-0.0". `math::Reflection` adds `+ 0.0` to every result
    (as `toggle_visit.hpp` :674 does); T1 checks `std::signbit`, T8 pins a zero coordinate on the screen.
16. ***[R8]* Log line budgets.** `match_runner.hpp`'s COMP buffer is 160 bytes (:239); the terminal sink cuts any
    message at 200 (`term_sink.hpp` :285); WALL lines live in a 384-byte Line but still pass through that sink. Put
    the mirrored note where a cut cannot reach it, measure every new line's worst case, write ASCII only ("--").
17. ***[R3]* The camera reads the real field.** A piece target written in the route is robot one's; a pose for the
    finder must be the localizer's, not `chassis.pose()`; a camera fix driven to through a Chassis verb must go through
    `chassis.reflection()` first. Nothing uses the finder before the 21st; L4 must take this when it is built.

---

## 9. Summary
*(re-written at the re-check, 2026-11-13 ~02:45 — [R1], [R2], [R4], [R11])*
- **What:** the students write the skills route once; robot two's autons file says, in one line (typed by the students
  — D2's question), "run it mirrored"; the library reflects every field number at the Chassis for that run, seats robot
  two at the reflected start, swaps the wall reset's wall and sensor side and the Toggle step's Toggle, refuses by name
  the three things it cannot mirror, and documents the camera's field inputs it does not.
- **The mirror (manual, D1):** the reflection across the diagonal from the red/right corner to the blue/left corner,
  `(x, y, h) → (−y, −x, −90° − h)` — the only transform that keeps all 9 goals, all 4 Toggles and red's side; not the
  usual left/right mirror. Loaders and one Load Zone pair are not symmetric; the robot is not mirrored.
- **Both robots at once (D11, new):** the two robots run together, so robot one must keep its whole robot 3 in off the
  diagonal — then the pair is 6 in apart whatever the timing; what lies ON the diagonal (the centre goal, two yellow
  Pins, the corner Load Zone) is for ONE robot, chosen with `isMirrored(ctx)`; the suite runs the pair together on the
  host test plant and fails, naming the step, under 6 in; the previews draw both robots.
- **Evidence (host test plant):** robot two's path = the reflection of robot one's to 8.8e-13 in over 14 steps of the
  kinds a tank route uses (P1; no sideways move, raw drive or trigger in it — the triggers are P6's, equal to 3.8e-14
  on the same tick; the rest is T2's); the wall reset seats on the mirror wall with the other sensor (P2); the
  exact-180° tie is the one divergence (0.155 in; it also applies to a holonomic moveTo); P1's own route crashes the
  two robots at 10.1 s, a route kept on robot one's side holds them 8.4 in apart (P5).
- **Runs and sizes:** MIRROR1-A **~3–4 builder-days + ~½–1 day review/merge** (API 2.9, F6-additive, F10 untouched;
  was ~2–3); MIRROR1-B ~¼ only if AF1S-A1 merges after A (not expected); the RPV1 follow-up ~¼–½ (no slot before the
  freeze unless RMP1's drawing covers it).
- **Honest dates against the ~Nov 18 freeze (§6.1b):** the team lead's go on Fri 11-13 → launched that day → built
  ~Nov 14–15 → merged **~Nov 15–16** (the peers' measured pace: ~1½–2 calendar days with stops). One full-day usage
  stop → ~Nov 17, the freeze edge. **Robot two's mirrored route on the bench Nov 16–18 ONLY IF** ROB1-BT has merged,
  the builder's facts are typed, the students' robot-one route is written and runs on robot one, and robot two's line
  is typed — the library run is not the long pole. Fallback (D9 (b)) only if MIRROR1-A has not merged by ~Nov 17.
- **Unverified:** everything on a robot (HA-258..260) — including whether 3 in a side is enough margin for the pair
  (Step 0.5); v2.1's Appendix A drawings and Figure VURS1 re-measured (only v2.1's text was read; the Pins on the
  diagonal are from v2.0's figure by pixel, ≈ ±1 in); the twin's real build symmetry (Step 0.2); RMP1's size (its brief
  is not written); whether the builder has rebuilt robot two (ROB1-BT waits on it).
