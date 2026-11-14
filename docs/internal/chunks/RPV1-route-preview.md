# RPV1 — the ROUTE PREVIEW on the host: one command runs a named auton from the team's REAL `src/team/autons.cpp` through the REAL library on the host test plant and writes ONE self-contained page — the field to scale, the robot's path, every step numbered where it started and ended with its time, the outline, mechanism steps and wall resets marked, every failed / skipped / cut step in its own colour with the library's own sentence, the total against the 60 s skills budget — plus the same table on stdout

> **THE TEAM LEAD'S ANSWER (2026-11-11 ~00:20): "Go: A0+B1 before 21st (Recommended)"** — A0 and B1 as ONE run, queued after
> the checked Toggle step, curves and robot one's build; A1, A2 and B2 after the 21st. D1–D13 otherwise as recommended.

> **Status 2026-11-10 (late evening), RE-CHECKED ~23:30: DRAFT for the team lead's go. WAITS for his answers to
> D1–D13 below, then for a go per run (RPV1-A0 first — §2.1). Nothing is built.** The checker's 13 findings and what
> was done with each are in **"Re-check — amendments"** right below this header; every body passage they changed is
> marked **[RE-CHECK Rn]**.
> Written by the coordinator's brief writer from a READ-ONLY pass over branch `shulib-v2`. **[RE-CHECK R7] Base
> re-stamped: main's tip is `16c01e0` (L6-A2, COMMITTED 2026-11-10 23:12).** The pass was first made at `d684f34`
> (IMU1-B); between the two, of the files this brief cites only `motion/motion_scheduler.hpp` (+4 lines;
> `setBoundaryObserver` still :928), `localization/override_walls.hpp`, `docs/hardware-assumptions.md` and
> `docs/roadmap.md` changed (`git diff --stat d684f34 16c01e0`), and the lines cited in them were re-opened at
> `16c01e0` and still hold — so every `file:line` below is **`16c01e0`'s**. Citations once marked "(L6-A2, staged)"
> are `16c01e0`'s too (`auton_wall_reset.hpp` :9-12, :25-29, :87 re-opened). The main worktree's INDEX now carries the
> **MECH-S-A squash, staged** (54 files; §6). Worktrees in flight were read, never edited (§6). **Probed on the host
> test plant, not argued:** six probe programs over `git archive d684f34` (and one over the L6-A2 index) in
> `.claude/coordinator/scratch/brief-RPV1/` (§0.4), plus the checker's four (`checker/out/c_*.txt`); every binary ran
> under `(ulimit -v 4194304; timeout 300 …)`.
>
> **THE TEAM LEAD'S WORDS THIS BRIEF HOLDS (BINDING), 2026-11-10, as given to this writer:**
> - "I really need to be able to plan whole routes" — offered (a) a standalone planner page + preview, (b) code +
>   preview only, (c) the panel's full route maker now; **he chose (b) "Code + preview only".**
> - **NO PANEL before the 21st. SKILLS ONLY at this competition** (one 60-second skills auton). The auto-flipper
>   (now AF1-S) and curves (CURVE1) are kept. **First competition 2026-11-21; code changes stop ~Nov 18; robot one
>   (the cascade robot; both robots identical, 15") is built ~Nov 14.** Both robots run the skills route (AF1-S's
>   answers block, 2026-11-10).
> - So routes are WRITTEN IN CODE (`chassis::Routine` in `src/team/autons.cpp`) and the team needs to SEE a whole route
>   before and between robot sessions. **G4: the team writes the autons; the tool only runs and draws them.**
>
> **Two things the coordinator told the team lead that the probes do not support — both corrected here:**
> 1. **"The times are lower bounds."** They are not. On this very plant a motor that LAGS settles the 40 in drive
>    SOONER (1.40 s vs 1.80 s ideal) and the acceptance run in 4.74 s vs 5.44 s; a 9 % weak side makes them LATER
>    (1.89 s; 6.14 s) — probe P10, agreeing with STEP1's P3 (`docs/internal/chunks/STEP1-route-steps-library.md`
>    :378). The ideal plant's time is the library's own speed budget acting on a perfect robot: **neither a floor nor a
>    ceiling.** The caption says so, and D5 offers a "spread" line that shows it.
> 2. **"About 1 build, ~2–3 days."** **[RE-CHECK R5, R2]** A first part that previews ROBOT TWO's routes is about that:
>    **RPV1-A0, ~1½ builder-days + ~½ day review, no change to anything a robot program compiles.** But it CANNOT
>    preview robot one (the cascade robot): on the suite's rig a step on a mechanism robot two does not have STOPS the
>    route at that step (checker probe C4). Robot one needs a second build, **RPV1-B1 (~1 builder-day + ~½ review)**,
>    and robot one's own file from G1. Exact step numbers need a small library change (D2/D13, the Routine says nothing
>    when a step succeeds, P8) and real wall resets need a robot built from its file with side sensors (P9) — both are
>    later parts. **Honest size of everything: ~5 builder-days + ~2½ review**, of which A0 + B1 (~2½ + ~1) is what
>    matters before the 21st (§2.1, §9). Calendar dates are ranges that allow for 5-hour-limit stops (§9).
>
> **Executor model: `opus`. Do not commit. Live log `docs/internal/chunks/RPV1-PROGRESS.md`, created FIRST** (a heading
> per run). Say "host test plant", never "simulator" — in code, comments, the page and the docs. **No hardware claim
> anywhere:** nothing here runs on a robot. **Hardware Assumptions: none expected** (§4). **Memory rule (2026-10-15):**
> every binary built — the tool, probes, the suite, any test — runs as `(ulimit -v 4194304; timeout 300 …)`; the suite
> only through `python3 tools/suite_run.py`. **The robot builds:** RPV1 changes NOTHING in `src/`. **[RE-CHECK R3,
> R12e]** RPV1-A0, A1 and B1 change nothing a robot program compiles (A0's one `include/` edit is an opt-in hook on the
> host-only fake log sink, `hal/fake/fake_telemetry_sink.hpp`, which no file under `src/` includes). Only **RPV1-A2**
> (the step listener, D13) changes a library header the robot programs compile: they are recompiled and behave the
> same (nothing attaches a listener on a robot); the host plant's runs were proven bit-identical with and without it
> (P11) — the robot binaries are NOT claimed identical. The team lead decides that change and its timing in D13.
>
> **Shape:** the house style of `docs/internal/chunks/L6-wall-reset.md` and `STEP1-route-steps-library.md`.

---

## Re-check — amendments (2026-11-10 ~23:30; the checker's 13 findings, each re-verified by this writer)

Each finding was re-opened at `16c01e0` (or in the checker's probe logs, `.claude/coordinator/scratch/brief-RPV1/checker/out/`)
before it was accepted. **All 13 are real; one carries an arithmetic slip that does not change the conclusion (R1).**

| # | Finding (checker's severity) | Re-verified | Disposition — where the body changed |
|---|---|---|---|
| **R1** | The field check calls a normal touch "LEAVES THE FIELD", fails the command and turns the suite red (major) | **REAL.** The Loaders protrude 3.74 in, the Toggle brackets and body overhang 1 in (`sim/field_plant.hpp` :20-29, :127); driving up to a Loader or squaring on a wall overlaps them by design. **One correction:** a start typed at y = −62.7 puts a 15 in square's edge at −70.2000, **0.0047 in INSIDE** the face (−70.2047), not over it — but the same start turned 0.1° reaches 7.5·(cos 0.1° + sin 0.1°) = 7.513 in and overlaps by 0.008 in, so the conclusion stands: "touch" is razor-thin at every wall start. | **Fixed.** Two cases: **THROUGH** (the outline past a wall's inner face by more than an allowance, proposed 1 in) red, rest faded, exit 1; **TOUCH** (a wall, Loader or Toggle within the allowance) an amber note, exit 0. CI fails only on THROUGH. The 15 in square, the point it is centred on and the allowance are now the team lead's decision **D12**. D6, D8, §1.1 field check, RP11, new RP29, RM23. If D8 (a) + D12 (a) are chosen the footprint is a belief code acts on → an HA row (§4). |
| **R2** | RPV1-A cannot preview robot one's route; RPV1-B arrives at the freeze (major) | **REAL.** `checker/out/c_other.txt`: a step on `'intake'` → "is not on this robot … the step did nothing", the routine STOPS at step 2 (`routine.hpp` :621, :646, :660, :680 `notOnThisRobot`); the rig binds robot two's list only (`test/auton_tester_rig.hpp` :153-171, :300-303). | **Fixed.** D3 says plainly that the first part previews robot two only and a robot-one route stops at its first cascade step. RPV1-B split: **B1** = any robot's own mechanism list on fakes by kind (a lift or arm on `sim::ArmPlant` built from its declaration, as `test/mechanism_auton_rig.hpp` already runs robot x's lift in the program's pacer chain), still on the stand-in drive, **independent of L6-B**, built and tested against robot x's fixture lift BEFORE G1, switched to robot one's list the day G1 lands; **B2** = real wall sensors after L6-B. The sentence "it can land after the Nov 18 freeze and still serve the 21st" is **removed** — routes are code and freeze ~Nov 18. D3, D10, §1.2, §2.1, §9. |
| **R3** | An auton that loops forever ends as PROGRAM ERROR `std::bad_alloc` after ~2.6 GB, not at the 300 s timeout (major) | **REAL.** `checker/out/c_forever.txt`: 16,777,216 log lines, 2.6 GB, bad_alloc at 12.6 s. Each skip after a stop logs a line (`routine.hpp` :821-838) and the fake sink keeps every line (`hal/fake/fake_telemetry_sink.hpp` :33-35). **A second form remains that the checker did not run:** a loop of steps that SUCCEED without letting time pass (`for(;;) r.then([]{ return true; })`) logs nothing and allocates nothing — that one really is caught only by the 300 s timeout. The robot's behaviour ("sits until the field disables it; its park never runs") is REASONED: the guard checks its floor only inside `pace()` (`sequence/run_guard.hpp` :344-351), and a loop with no pace never reaches it. | **Fixed.** A0 adds an opt-in hook to the host-only fake sink (default off: every existing test unchanged); the tool trips it when more than 10,000 lines arrive with no pace in between (a real 60 s run logs ~40 lines: C3 37, P2 under 300 even lifted), writes the page from what was recorded with "after step 39 the auton keeps looping without letting time pass; on the robot it would sit there until the field disables it, and its park would never run", prints the table, and ends with `std::_Exit(1)` (the hook may run inside library code that must not throw). The A2 listener's log gets the same no-pace cap. Landmine 10 rewritten for both forms. §1.1, RP27, RM22. |
| **R4** | Verdicts of START mechanism steps attach to the wrong step (major) | **REAL.** `checker/out/c_start.txt`: `startRun(clawPivot6, …)` is step 1; its verdict is logged at T+1.02 while step 2 (a drive) is open. The start forms exist at `16c01e0` (`routine.hpp` :572-685). | **Fixed.** `MECH` lines are attributed by the mechanism's NAME, not by the open step: each started mechanism is drawn as its own span along the path, from where it began to where its verdict arrived, with that verdict; the start step is a "started" mark; a `waitUntilDone` step shows the verdict it collected. §1.1, §1.3, RP28, RM21. |
| **R5** | Size and dates: RPV1-A jumps the queue silently, edits `routine.hpp` beside STEP1-B and CURVE1, and is not the smallest first part (major) | **REAL.** NOTES :963 lists builds AF1S-A1, TUNE1-B1, STEP1-B, CURVE1, RPV1, G1; the brief never said which it moves. Calendar history: STEP1-A launched 11-06 ~00:30, done 11-11 ~00:xx (NOTES :949, :965 — with a 5-hour-limit stop and a shutdown, :954); L6-A2 needed a week. | **Fixed.** A real smallest first part, **RPV1-A0** (~1½ builder-days, nothing a robot compiles changes; contents §1.1). Then **B1** (robot one), **A1** (spread, lifted run, CI check, extra flags), **A2** (the library listener — after STEP1-B and CURVE1 both merge, so `routine.hpp` is edited in sequence, never three-way; after the 21st unless both are in by ~Nov 16), **B2** (real wall sensors after L6-B). The queue position is **D11**, the library change's timing **D13**; dates are ranges (§9). |
| **R6** | D9's lifted run states a false total for a "cycle until time" route; D5's spread is meaningless on a cut route (major) | **REAL.** `checker/out/c_cycle.txt`: real run CUT at 57.02 s; lifted 590/595 s run CUT again at 590.02 s (59,316 paces, 418 ms, 77 MB). A cut run's `scoringEnded` is about its park time, so a spread of cut runs reads ~57 s three times; the example "weak right side 61.0 s" cannot happen on a skills entry. | **Fixed.** D9: if the lifted run is cut again, the page says "this route keeps going until its park time (it is meant to be cut)", draws the dashed part only to T+120 s, and never prints a "whole route" total. D5: on a cut route the spread shows how far each version got ("cut after step 52 / 55 / 49"); the impossible example is replaced. RP12, RP19, new RP30. |
| **R7** | The brief's base is stale (minor) | **REAL.** `git log`: `16c01e0` L6-A2 committed; the index now carries MECH-S-A (54 files). | **Fixed.** Header re-stamped to `16c01e0` (with the list of cited files that moved); §6 rows for L6-A2 (committed), MECH-S-A (staged; its `service.hpp` accessors move — `lastStep` :272, `running` :274, `stepsBegun` :279 in the index — and it touches `test/calibrate_program_rig.hpp` and `test/mechanism_position_rig.hpp`, NOT `auton_tester_rig.hpp`), STEP1-A (done, in the merge queue). |
| **R8** | The commit stamp does not show "-dirty" after an edit to `autons.cpp` (minor) | **REAL.** `test/CMakeLists.txt` :29-73: `git describe --always --dirty` runs at configure time, re-run only when `.git/HEAD` / `logs/HEAD` move; its own comment says a dirty flag "can lag until the next reconfigure". | **Fixed.** The launcher computes `git describe --always --dirty --abbrev=7` and a short SHA-256 of `src/team/autons.cpp` at RUN time and passes both to the program; the page and stdout print both. Landmine 14 rewritten; RP34, RM24. |
| **R9** | RP5's "STEP1-A's captures re-used" contain no truth-stream bytes (minor) | **REAL.** `.claude/worktrees/step1-a/test/routine_golden_chains.hpp` :10: "It reads only what both trees have: the nine frozen RoutineResult fields and the RTN lines". | **Fixed.** RP5 (now in A2): A2 captures its OWN FNV-1a hashes of the truth stream and the sink's records over the golden chains from the merged tree BEFORE adding the seam (as P11 did), and compares against them. |
| **R10** | RP20's recorder cap cannot be reached; the plant's 200,000-pace cap fires first (minor) | **REAL.** `test/motion_test_rig.hpp` :148-160: `PlantPacer` throws "PlantPacer: pace cap exceeded — a wait failed to end" at 200,000 paces; the recorder sits above it. | **Fixed.** The recorder's own cap is dropped; the plant's cap is the bound and its sentence is the one shown. RP20 rewritten around a frozen `pause(3000_s)`. **Not verified:** whether the guard's hard floor ends a frozen pause first (it is checked in `pace()`, `run_guard.hpp` :350-351; a frozen wait keeps pacing) — the executor probes it before writing RP20, and RP20 asserts whichever sentence ends the run. |
| **R11** | Robot two's placeholder numbers are not marked in RPV1-A (minor) | **REAL.** The claw's `moveTime = 0.5_s` is "PROVISIONAL (HA-185)" (`src/robots/mechanism_table.hpp` :124). | **Fixed.** A0 marks every step whose time comes from a PROVISIONAL declaration number with the hatched "PLACEHOLDER number" note and its HA number. §1.1, RP33. |
| **R12** | Smaller gaps (a)–(e) (minor) | **(a)** REAL — a helper's own `Routine` restarts at step 1. **(b)** REAL — a dropped entry is not registered; its name is only inside `refusal(i)`'s "auton '<name>': …" (`competition/auton_registry.hpp` :98, :458-460, at most 8 kept). **(c)** REAL — `route_preview_main.cpp` is not in the suite. **(d)** REAL — check-examples scans only ```` ```cpp ```` blocks (`tools/api_doc_tool.py` :2073). **(e)** REAL — the header's wording could be read as "the robot binaries are identical". | **Fixed.** (a) steps numbered per routine, the routine's name shown (RP31, RM25). (b) the lookup reads the refusal sentences' quoted names too; a refused "skills" draws a REFUSED page with the sentence, exit 1; more than 8 refusals → "more autons were refused than the registry keeps" (RP32). (c) the logic lives in `test/route_preview_run.hpp` (`int runPreview(args, out, err)`), `main` only calls it; RP10 calls it. (d) the claim is dropped; the launcher's `self-test` instead parses every `route_preview.py` line in guide 16 with its own argument parser. (e) header reworded. |
| **R13** | Some decisions are not put to the team lead; the D blocks are not plain words (minor) | **REAL.** | **Fixed.** D1–D10 rewritten in plain words (the file names and internal terms stay in §1); **D11** (the queue position), **D12** (the robot's outline and what counts as touching), **D13** (the library change in the freeze week) added, each with a recommendation. |

**Found by this writer while re-checking:** (i) STEP1-A is DONE and third in the merge queue (NOTES :965), so A0 can read its stop/skip/carry-on lines if it has merged — A0 does not depend on it. (ii) CURVE1's brief holds HA-246..HA-250 (`briefs/CURVE1-curve-follower.md` :39, its A14), so RPV1's first HA number, if one is needed, is **HA-251** (§4). (iii) If the suite fails on the outline (D8 (a) + D12 (a)), "the robot is a 15 in square centred on the point the plant tracks" becomes a belief that code acts on, so it needs a register row (§4).

---

## DECISIONS FOR THE TEAM LEAD (a recommendation for each; nothing below decides them silently)

**[RE-CHECK R13] Rewritten in plain words; D11–D13 added. The file names and internal terms are in §1.**

**D1 — How the team runs it. RECOMMENDED (a).**
- **(a) RECOMMENDED — one command:** `python3 tools/route_preview.py "skills"` (the auton's name as the brain's menu
  shows it). It builds the preview program on the laptop (about 5 seconds the first time), runs it with the same
  memory and time limits the test suite uses, prints the step table, and says where the page was written
  (`build/route_preview/skills.html`). Open that file in any browser; it needs no internet.
- **(b) Two commands** (build, then run). Works, but a student must remember both and the limits.
- **(c) Make it part of the test suite.** The suite would have to write files and be told a name. Not recommended.

**D2 — How the picture numbers the steps. RECOMMENDED (a). [RE-CHECK R5 — the recommendation changed]**
Today the library says nothing when a step SUCCEEDS; it only speaks when a route stops ("STOPPED at step 12 …") and for
each step it then skips. From the outside the preview can see every drive, turn and mechanism command, but not a
`pause`, a `then(...)` action (a wall reset is one) or a `startAt` (probe P8).
- **(a) RECOMMENDED for the first version — count what can be seen, and quote the library's own lines.** The picture
  numbers drives, turns and mechanism steps in order ("drive 3", "claw 2") and prints the library's own "STOPPED at
  step 12" and "step 13 skipped" sentences word for word, which carry your code's REAL step numbers. **Cost:** when a
  route has pauses or `then(...)` steps, the picture's numbers and your code's numbers differ, and the page says so in
  one line at the top.
- **(b) Add the small library change that makes them match exactly (D13) in the first version.** Delays the first
  version until STEP1-A merges, and changes a file every robot program compiles in the week before the freeze.
- **(c) Read the auton's source text.** Not workable: loops and helpers break it.

**D3 — Which robot it previews, and where it starts. RECOMMENDED (a). [RE-CHECK R2]**
- **(a) RECOMMENDED.** **The first version previews ROBOT TWO only** — its real claw pivots and claw — on the test
  suite's stand-in drive (a 12 in tank with made-up gains; robot two's own track width is not in the code yet), with no
  wall sensors (every wall reset is drawn "refused: no sensor", which is also what robot two's program does today).
  **It cannot preview robot one (the cascade robot):** a route step on a mechanism robot two does not have STOPS the
  route right there ("'cascade' is not on this robot"), so a robot-one route would be drawn stopped at its first
  cascade step. **Robot one needs the second build (B1)** — any robot's own mechanisms, the cascade on a pretend lift
  that moves exactly as its declared numbers say — and robot one's file from G1. B1 can be built and tested before
  G1 lands; it switches to robot one the day G1 lands. Real wall resets come last (B2, after L6-B gives robot two its
  sensors). The page's first line always says which robot, which drive, and what is pretend.
  **The start:** a route declared at a FIELD position starts there on the picture. A RELATIVE route needs a place:
  `--place -48,-48,90` (x in, y in, heading in degrees); without it the robot is put at the field's centre facing 0°
  and the page says so in red at the top.
- **(b) Build robot one's preview first.** It cannot show anything until G1 lands, so robot two's preview would wait
  for nothing.
- **(c) Refuse a RELATIVE route without `--place`.** Safer, but the 40 in auton could no longer be previewed with one
  word.

**D4 — The output. RECOMMENDED (a).**
- **(a) RECOMMENDED — ONE web page that works offline** (the field picture, the step table, the caption; nothing
  fetched from the internet), **plus the same table as plain text in the terminal** (for CI). `--svg` also saves the
  bare picture for slides (in A1).
- **(b) The picture only.** No table beside it.
- **(c) Always two files.** Two files to keep in step for nothing.

**D5 — What the page shows. RECOMMENDED (a). [RE-CHECK R6]**
- **(a) RECOMMENDED. Always:** the field to scale (walls, Toggles, Loaders, Load Zones and the start pieces — the only
  field parts shulib knows; the page says anything else is not drawn); the robot's path, coloured step by step; each
  step's number where it started and ended, with its time from the auton's start; where each drive was AIMED beside
  where it stopped; the robot's outline at the start and end of every drive and turn, front marked; every mechanism
  step and wall reset marked where it ran, with its result word; a mechanism started in the background drawn as its
  own stretch along the path, from where it began to where it finished; failed, skipped and cut steps each in their own
  colour with the library's own sentence; a timeline from 0 to 60 s (30 s for a match auton) with the park and stop
  times; the result word; a "PLACEHOLDER number" note on every step whose time comes from a number nobody has measured
  yet (robot two's claw: 0.5 s, HA-185); and the CAPTION: *"IDEAL host test plant, not the robot: no wheel slip, no
  battery sag, motors that answer at once. It shows what the library would do with this code on a perfect robot. The
  times are not a promise either way — time it on the robot."*
  **Then (A1): a "spread" line** — the same route re-run with sluggish motors and with robot two's ~9 % weak side, so
  nobody reads one number as exact. On a route that finishes: "ideal 5.44 s · sluggish motors 4.74 s · weak right side
  6.14 s" (the acceptance run, probe P10). On a route that is cut at its park time it says how far each got instead: "all three cut at the park time —
  ideal after step 52 · sluggish motors after step 55 · weak right side after step 49" (an illustration; not run).
- **(b) No spread line.** Simpler; the one time reads as more certain than it is.
- **(c) The bare path only.** Not what was asked.

**D6 — A step that fails, and a robot that hits the field. RECOMMENDED (a). [RE-CHECK R1]**
- **(a) RECOMMENDED.** A step that fails or refuses is drawn red where it happened, with the library's own sentence. A
  wall reset that refuses but lets the route go on is AMBER ("refused — the position was not changed"), never green.
  Skipped steps never ran, so they are listed in grey under the stop, not drawn on the field. A step cut by the park
  time is amber; what never ran after it is listed.
  **The field:** the host test plant has no walls (a 200 in drive goes straight through, P6), so the preview checks the
  robot's outline against the field itself, with TWO levels:
  - **THROUGH** — the outline goes more than the allowance (D12; proposed 1 in) past a wall's inner face: red, "the
    host test plant has no walls; on the robot it hits the left wall here at 4.12 s in step 3", everything after it
    faded ("the robot would not get here"), and the command reports failure.
  - **TOUCH** — the outline reaches a wall, a Loader or a Toggle, but not THROUGH: an amber note ("touches the Loader
    here — the plant has no Loader"), and the command still succeeds. Squaring on a wall or driving up to a Loader is
    normal, so it must not count as a failure.
  - Driving over a start piece: a grey note (the plant does not push pieces).
- **(b) Stop drawing at the first THROUGH.** Hides how far off the rest of the route is.
- **(c) Make the plant's robot stop at the wall.** That would be invented physics. Not recommended.

**D7 — One auton by name, or all. RECOMMENDED (a).**
- **(a) RECOMMENDED — by its exact name.** No name: it lists every auton and draws nothing. A wrong name: "no auton
  named 'skils' in src/team/autons.cpp — registered: '40in forward', 'TEST 24in turn back'", nothing drawn (it never
  guesses — it must never draw a different auton than you asked for). An auton the program itself REFUSES (for
  example a stop time past 60 s) is found by its name too and drawn as a REFUSED page with the program's sentence.
  `--all` (in A1) writes one page per auton and an index page.
- **(b) Always all of them.** Slower to find the one you want.

**D8 — Whether the test suite checks the skills routes. RECOMMENDED (a). [RE-CHECK R1]** *(in A1)*
The test suite has to pass before ANY library change is merged, so whatever this check fails on can hold up library
merges until a route is fixed.
- **(a) RECOMMENDED — fail only on things the robot would really hit:** a skills route whose outline goes THROUGH a
  wall (D6), a route that crashes the program, or a route that loops without letting time pass (on the robot it would
  freeze until the field disables it). A route cut at its park time is reported with its margin but does NOT fail
  (a "keep cycling until time" route is legal). TOUCHES never fail. **Cost:** a student route with one of those three
  defects holds up library merges until it is fixed — usually a one-line fix.
- **(b) As (a), and also fail when a route cannot finish before its park time.** Catches a plainly too-long route,
  but a deliberate "keep cycling until time" route turns everything red with no way to say "meant to be cut".
- **(c) Report only, never fail.** Nothing is held up, and nothing forces anyone to read it.

**D9 — Where 60 s falls on the path. RECOMMENDED (a). [RE-CHECK R6]** *(in A1)*
A skills route can never run past 60 s: the program refuses a stop time past 60 s and the route is cut at its park
time. So "where 60 s falls" only means something on the route as WRITTEN.
- **(a) RECOMMENDED.** When the real run is cut, the preview re-runs the route with the clock limit lifted and draws
  what would have come next DASHED, with a red "60 s" mark where the clock reaches 60 s, up to 120 s at most. If the
  route finishes there: "the whole route would take 129.0 s". If it is cut AGAIN (a "cycle until time" route): "this
  route keeps going until its park time (it is meant to be cut)" — and no total is printed.
- **(b) The real run only.** Says "cut at 57 s, 89 steps never ran" but not where they would have gone.

**D10 — The parts and their order. RECOMMENDED (a). [RE-CHECK R2, R5]**
- **(a) RECOMMENDED — five small parts, each its own go:**
  1. **A0** — robot two's routes: the picture, the table, the field check, the plain caption (~1½ builder-days).
  2. **B1** — any robot's own mechanisms, so robot one's routes preview once G1 lands (~1 builder-day).
  3. **A1** — the spread, the dashed rest of a cut route, the suite check, `--all` / `--svg` (~1 builder-day).
  4. **A2** — the library change that makes the step numbers exact (D13; ~½ builder-day).
  5. **B2** — real wall resets, after L6-B gives robot two its wall sensors (~1 builder-day).
  Before the 21st, A0 and B1 are the ones that matter; A1 if there is room; A2 and B2 most likely after.
- **(b) One build with everything.** Nothing in the team's hands until it all lands — after robot one's route is
  written.
- **(c) A0 only.** Robot one's route — the cascade, the top priority — is never previewed.

**D11 — Where the first part goes in the build queue. RECOMMENDED (a). [RE-CHECK R5]**
The builds queued now are, in order: the Toggle step (AF1S-A1), the mechanism tuning (TUNE1-B1), the route-step
triggers (STEP1-B), curves (CURVE1), then this preview, then robot one's build (G1).
- **(a) RECOMMENDED — A0 right after the Toggle step**, ahead of TUNE1-B1, STEP1-B and CURVE1. It changes no library
  file, so it collides with none of them; and the routes are written between ~Nov 14 and the ~Nov 18 freeze — a preview
  that arrives after that has missed its use. **Cost:** if every build slot is busy, whichever of TUNE1-B1, STEP1-B or
  CURVE1 is waiting for a slot starts about a day and a half later. B1 goes right after A0 for the same reason.
- **(b) Keep its listed place, after curves.** In the team's hands ~Nov 16–17 at best — after most of the route is
  written.
- **Either way:** robot one's build (G1) should not wait behind A1 — the robot-one preview (B1) is only useful once G1
  has landed.

**D12 — The robot's outline for the field check. RECOMMENDED (a). [RE-CHECK R1]**
The preview needs a shape to check against the walls. It only knows where the plant's robot's tracked point is.
- **(a) RECOMMENDED — a 15 in square centred on the tracked point** (your 2026-10-08 word: both robots are 15 in),
  with a **1 in allowance** before a wall overlap counts as THROUGH. Said on every page: the real frame may be smaller,
  the tracked point may not be the robot's middle, and a claw or lift that reaches past the square is not checked.
- **(b) The builder measures each robot's frame and where its tracked point sits**, and the check uses those. More
  exact; needs his numbers before it is right.
- **(c) A different allowance** (for example ½ in or 2 in). Smaller catches more near misses and flags more normal
  wall squaring; larger misses real hits.

**D13 — When the library change for exact step numbers goes in. RECOMMENDED (a). [RE-CHECK R5, R12e]**
The change is small and does nothing on a robot (nothing listens there; on the laptop runs are proven identical with
and without it). But it is in a file every robot program compiles, and STEP1-B and CURVE1 are changing that same file
before the 21st.
- **(a) RECOMMENDED — after BOTH STEP1-B and CURVE1 have merged, and only if that is by ~Nov 16;** otherwise after the
  competition. Until then the picture numbers what it can see (D2 (a)).
- **(b) In the first part (A0).** Exact numbers sooner; a third change to the same file in the freeze week.
- **(c) Never.** The picture's numbers keep differing from the code's whenever a route has pauses or `then(...)`.

---

## The design in plain words (for the team lead)

You write a route the way you do today, in `src/team/autons.cpp`, and register it with a name. You type one command
with that name. The command builds a small program on the laptop that contains YOUR file and the SAME library code
the robot runs, and runs your auton on the host test plant — the pretend robot the 3,000-test suite already drives every
auton on. While it runs, the program writes down where the plant's robot really is every 10 ms (not where the robot
THINKS it is — the plant knows the truth), which drive, turn or mechanism step is running (D2), every mechanism
command, every wall reset, and
every line the library logs. When the auton ends, it draws one page: the field, the path, each step numbered where it
started and ended with its time, the robot's outline at the end of each drive, and a timeline against the 60 s.

It runs your auton exactly the way the competition program does (the same `runAuton` call, the same deadline guard, the
same park at your park time). So what it draws is what the library would make a perfect robot do — including a step
that times out, a mechanism step that refuses, a wall reset that refuses, a cut at the deadline. What it cannot tell
you: how YOUR robot differs from a perfect one — wheel slip, a weak side, a tired battery, a lift that is slower than
its numbers say, a robot that hits a field element the plant does not have. That is why the page says "IDEAL" at the
top and shows a spread of times, and why a route still has to be run on the robot. **[RE-CHECK R2]** The first part
previews robot two only; robot one's routes need the second part (B1) and robot one's file (G1).

It never writes or changes your code (G4): it runs it and draws it. It cannot hurt the robot: it changes nothing in
`src/`; the first parts change nothing a robot program compiles, and the one later library change (D13) does nothing
on a robot. **[RE-CHECK R12e]**

---

## §0 Why, what exists, and the probes (every line cited was OPENED at `d684f34` and re-checked at `16c01e0` — header, [RE-CHECK R7])

### 0.1 Why this chunk, and why now

- The team lead, 2026-11-10: "I really need to be able to plan whole routes" → "Code + preview only"
  (`.claude/coordinator/NOTES.md` :957). The panel (G3a) — which held the only planned route drawing (G3a's R.2 "the
  ROUTES page", `docs/internal/chunks/G3a-robot-profile-generator-panel.md` :565-583) — is out before the 21st (NOTES
  :955). So routes are code, and nothing draws code.
- The skills route is ~60 s of steps written between robot one's build (~Nov 14) and the freeze (~Nov 18). Robot time is
  the scarcest resource in those four days; a route whose 40th step would leave the field, or that needs 129 s, should
  be found on a laptop, not on the field.
- Queue position: after CURVE1's brief; builds AF1S-A1, TUNE1-B1, STEP1-B, CURVE1, RPV1, G1 (NOTES :963). **[RE-CHECK
  R5]** Where A0 goes in that order is the team lead's call — D11 (recommended: right after AF1S-A1).

### 0.2 What exists in the tree

**The host run every auton already gets.** `test/CMakeLists.txt` :90 compiles `src/team/autons.cpp` into the suite;
`test/autons_test.cpp` :150-185 (1b) runs EVERY registered entry through `tester_rig::TesterStack::run` →
`bench::runTesterOnce` (`src/programs/auton_tester_run.hpp` :120-186) → `competition::runAuton(... mechanisms,
registry.periodFor(entry))` (:161; `include/shulib/competition/match_runner.hpp` :333-341, the service overload). The
stack (`test/auton_tester_rig.hpp` :270-338) is the tester program's chain — `Chassis → RunGuard → MechanismService
(REAL, robot two's list) → AbortWatch → ProbeLayer → PlantPacer` (:9) — on `motion_rig::MotionRig` (a 12 in tank,
`PilonsOdometry` with tracking wheels, gains kS 1.2 / kV 0.17, `test/motion_test_rig.hpp` :45-58, :83-112). Its
`ProbeLayer::inner` is a public pointer set in the constructor body (:307) — the recorder goes there, no fork (P1).
**`registerAutons`** (`src/team/autons.hpp` :26-33, `kAutonPeriods` match 30 s / skills 60 s; `autons.cpp` :74-89:
"40in forward" and the acceptance run "TEST 24in turn back", `competition/acceptance_run.hpp` :34).

**Does any rig record the pose each tick? No** (the task's question). `PlantPacer` keeps the cumulative true distance
and the previous pose only (`motion_test_rig.hpp` :148-170); `ProbeLayer` counts paces and two travel snapshots
(`auton_tester_rig.hpp` :175-225); `FakeTelemetrySink` keeps every `DebugRecord` (the ESTIMATE, `measuredPose`, while a
motion records) and every log line — **with no time stamp** (`hal/fake/fake_telemetry_sink.hpp` :27-31). The truth is
one call away: `SimHarness::sample()` (`sim/scenario.hpp` :186-192, a memcmp-able `TruthSample`). The recorder samples
it on every pace and notes `sink.size()`, which dates every log line to its pace (P1).

**Step boundaries.** `Routine::skipIfStopped` is the door of every step; `recordSuccess` / `recordStop` the two record
paths; only the stop (Warn) and the skips (Info) log (`chassis/routine.hpp` :821-877); `pause` is a frozen
`chassis_->wait` (:432-439); `then(...)` runs any action (:489-). STEP1-A (in flight) centralises the door as
`enterStep(name)` and adds carry-on / jump paths (`.claude/worktrees/step1-a/include/shulib/chassis/routine.hpp` :1043,
:1104, :1123, :1138, :1157, :1225 — six counter sites). **Motion boundaries** are already observable:
`MotionScheduler::setBoundaryObserver` (:928) with `CompletedMotion` (id, name, exit, start/end time, final and target
pose — `motion/motion_scheduler.hpp` :568-591); the slot is FREE under `runTesterOnce` (P1). **Mechanism steps:**
`MechanismService::size()/slotPtr(i)` and each slot's `stepsBegun()`, `running()`, `lastStep()` (`mechanism/service.hpp`
:231-242, ~:1726-1734 at `16c01e0`; **[RE-CHECK R7]** in MECH-S-A's staged index `lastStep` :272, `running` :274,
`stepsBegun` :279 — the executor re-reads at launch), and the `MECH` verdict lines, which carry the mechanism's NAME
in quotes (the attribution key, [RE-CHECK R4]). **The start forms** (`startMove` / `startHome` / `startRun` /
`startSpinUp`, collected by `waitUntilDone`: `routine.hpp` :572-685) let a mechanism run while the robot drives — its
verdict arrives while a LATER step is open (checker probe C2).

**The run's own account.** `AutonRunResult` (`match_runner.hpp` :114-145): `outcome`, `why`, `guard`
(`RunGuardReport`, `sequence/run_guard.hpp` :269-306: `scoringCut`, `floorFired`, `scoringEnded`, `endActionEnded`),
`routine` (`RoutineResult`), `armedAt` — the instant every T+ time is measured from. `runAuton`'s refusals
(`match_runner.hpp` :168-191); the registry's DROP mode, the programs' mode (`competition/auton_registry.hpp` :101,
:406, :459 `refusal(i)`).

**The field.** `sim::FieldModel::override2026()` (`sim/field_plant.hpp` :141; banner :1-50): the four walls at
±70.2047 in, each Toggle's body and brackets, the Loaders on the red and blue walls, A12's start pieces on every wall —
as boxes with heights, independent of the library's copy; `localization/override_walls.hpp` (the library's season
walls; the Load Zones' 23.11 in, `kLoadZoneDepth`). The frame is F1 (origin at the centre, +X right from the red
station, +Y away from it — `docs/roadmap.md` :135). **The drive plant has no walls** (`sim/drive_plant.hpp`: no wall,
field or collision term; P6).

**Robot two's tables.** `src/robots/mechanism_table.hpp` :113-132 (pivots 6/7 Power, Red, Hold; the claw Pneumatic on
'A', `moveTime 0.5_s` PROVISIONAL HA-185); `src/robots/chassis_table.hpp` :124-142 (`kRobotTwoChassis`: Blue, wheel
2.75 in REPORTED, **track width UNSET**, HA-125). On the robot the tuning is the library's defaults untouched
(`src/common/boot_source.hpp` :132-137, :208): `maxLinearSpeed` 60 in/s (`motion/motion_config.hpp` :164, HA-50),
`wheelFf` kS 1.0 / kV 12/70 (:154, HA-45). The suite's rig caps at the same 60 in/s; its plant saturates at ~63 in/s.
**Robot one has no build** (`src/build_selection.hpp` :113-117: BenchBot, RobotTwo, InventedXDrive; AF1-S's finding,
NOTES :963).

**Wall resets (L6-A2, committed `16c01e0` — [RE-CHECK R7]).** The everyday call `competition::resetFromWall(ctx, …)`
is written as a `then(...)` step and logs ONE `[WALL]` line (`competition/auton_wall_reset.hpp` :9-12, :87); it reads
the robot's wall-sensor slot, "none → REFUSED `no sensor` (… the host test plant's tester rig, which builds no Robot,
has none)" (:25-29); a Robot built from a config that declares sensors over the field plant is
`test/wall_robot_rig.hpp` (`WallWorld`, `WallWorldPacer`). **Mechanism numbers are provisional by design** (`mechanism/kinds.hpp` :40-48:
the controllers' gains, the smooth move's speed, the "reached" tolerance are STARTING values, HA-182); the
mechanism plant's constants are invented (HA-183; `sim/mechanism_plant.hpp` :1-40).

### 0.3 What this brief corrects in the framing it was given

1. **"Lower bounds"** — false on this plant (P10; header item 1). The caption and D5's spread line replace it.
2. **"About 1 build"** — the first part for robot two (A0) is; robot one (C4 → B1, [RE-CHECK R2]), exact step numbers
   (P8 → A2, D13) and real wall resets (P9 → B2) are not.
3. **"The 60 s line on the path"** — a valid skills run never reaches 60 s (the registry refuses a stop past the period,
   P7b; the guard cuts at the park time, P2). The line lives on the lifted-schedule run (D9, P2b).
4. **"Reuse the rigs that run the acceptance run"** — done for A0 (TesterStack, unchanged; B1 generalises it in place
   with today's list as the default, [RE-CHECK R2]). But that rig is NOT robot
   two's drive (a stand-in tank on tracking wheels) and builds no Robot (no wall sensors); the page must say so (D3).

### 0.4 The probes (`.claude/coordinator/scratch/brief-RPV1/`; `tree/` = `git archive d684f34`; `tree-l6a2/` = the staged index via `git checkout-index`; `tree-seam/` = a scratch copy with the step-listener seam, now part A2; logs in `out/`; the checker's C1–C4 in `checker/`)

| # | What it ran | What it found | Log |
|---|---|---|---|
| **P1** | `probe_preview.cpp`: a recorder pacer put between `TesterStack::probe` and the plant by re-pointing `probe.inner`; the scheduler's boundary observer attached; the team's two autons | Recorded true path **= `PlantPacer::distance` exactly** (39.770423 in; 47.320884 in; diff 0). The boundary-observer slot is **free** before and still ours after `runTesterOnce`. 40in: 495 samples, guard live T 3.53–5.44 (the tester's 50 idle ticks + 3 s countdown come first; `armedAt` 3.52). **4.2 ms and 5.3 ms** per run (-O0); probe build 5.1 s, 330 MB | `out/p_all_O0.txt` |
| **P2** | a SKILLS auton of 160 steps (40 × drive 40 in, pause 0.5 s, claw set, pivot run 0.4 s), park 57 s, stop 59 s | **CUT BY DEADLINE**; `scoringEnded` **57.36 s** (a frozen pause/claw step overran the 57 s cut by 0.36 s); stopped at step 71 'claw', 70 done, 89 never ran. **28.6 ms** (-O0), 18.6 ms (-O2); RSS +11.5 MB (the fake sink's 12,219 `DebugRecord`s × 456 B ≈ 5.6 MB; samples 0.68 MB) | `out/p_all_O0.txt`, `out/p_skills_O2.txt` |
| **P2b** | the same, schedule lifted (skills period 600 s, park 590 s) | FINISHED; **the whole route takes 129.0 s**; T+60 lands on a pace (`armedAt + 60` vs the nearest sample: 4×10⁻¹² s — compare within half a tick, landmine 6). 56.8 ms; whole probe binary peak **31.8 MB RSS, 0.11 s** | `out/p_all_O0.txt` |
| **P5** | a FIELD-start auton (-48, -60, 90°): (a) plant placed at the declared start; (b) plant left at the rig's (0,0,0); (c) the RELATIVE 40in placed at (-48,-48,90°) | (a) truth starts at the start; estimate = truth at the end. **(b) the estimate ends at (-0.23, -23.98) — "right" — while the truth ends at (36.0, -47.8, -90°): the picture would be WRONG and look right** → the plant MUST be placed at the declared start (RP2). (c) truth ends (-48.00, -8.23, 90°) = the placement ∘ the relative 39.77 in | `out/p_all_O0.txt` |
| **P6** | a RELATIVE auton driving 200 in from the field's centre | FINISHED at **x = 199.8 in**: the plant has no walls → the preview must detect leaving the field itself (D6) | `out/p_all_O0.txt` |
| **P7** | (a) an auton that throws `std::runtime_error` after its first step; (b) a skills entry with stop 61 s; (c) an entry with no start | (a) `PROGRAM ERROR`, `threw` with the text, the path up to the throw (19.77 in) recorded; `routine` counters unset (the throw unwound it). (b) DROP-mode registry: refused, "auton 'too long': hardStopAt 61.00 s is past its SKILLS period of 60.00 s -- the field disables the robot first". (c) "auton 'no start': has no start pose -- declare FIELD(x, y, heading) or RELATIVE" | `out/p_all_O0.txt` |
| **P8** | a 7-step auton: moveTo, pause, claw set, pivot run, turnTo, then(action), moveTo | `RoutineResult.steps` = 7, but from below: **3 motions + the end action's DriveBrake, and 2 mechanism steps** — the pause and the action leave no trace. Numbering from below would call step 5 "4" → D2 | `out/p_all_O0.txt` |
| **P9** | (staged L6-A2) a FIELD-start auton with `resetFromWall(ctx, {kLeftWall, Left, 3_in})` on TesterStack | `[WALL] left wall x REFUSED no sensor (LEFT sensor): this robot declares no wall sensors`; the step counts done (`continues()`), word FINISHED → D6 (amber, never green), D3 / B2 | `out/p_wall.txt` |
| **P10** | the team's two autons: ideal; motor lag kA 0.02; robot two's ~9 % weak right side; both | 40in **1.80 / 1.40 / 1.89 / 1.46 s**; acceptance **5.44 / 4.74 / 6.14 / 5.19 s** — lag is FASTER here, a weak side SLOWER: not a lower bound (header) | `out/p_bounds.txt` |
| **P11** | `tree-seam/`: a listener on the Chassis (prototype; the brief puts it on the scheduler, same call sites), events at the door / success / stop / skip; the same 3 autons with and without it, and the seam-less `tree/` | Events **equal the Routine's own numbers**: mixed 1–7 all SUCCEEDED with times; "stops": step 3 FAILED (cause 1), 4 and 5 skipped; long: 231+ events, stop at 71. **FNV-1a of the whole truth stream identical** in all three builds (64c18c40…, 6d5a1f6c…, a2552b7d…) | `out/p_seam.txt`, `out/p_noseam.txt` |
| **C1** *(checker)* | `for(;;){ r.moveTo(A); r.moveTo(B); }`, skills 57/59 s | cut at step 39, then every step skipped with no pace, one log line each: **16,777,216 lines, 2.6 GB, `std::bad_alloc` after 12.6 s**, reported as PROGRAM ERROR with the counters lost → [RE-CHECK R3] | `checker/out/c_forever.txt` |
| **C2** *(checker)* | `startRun(clawPivot6, 12 V, 1 s)`, `moveTo`, `waitUntilDone` | the pivot's verdict is logged at T+1.02 while the DRIVE (step 2) is open → attribute by mechanism [RE-CHECK R4] | `checker/out/c_start.txt` |
| **C3** *(checker)* | `while(!ctx.guard.expired()){ moveTo; claw; moveTo; claw; }` real and lifted 590/595 s | real CUT at 57.02 s; lifted CUT AGAIN at 590.02 s (59,316 paces, 418 ms, 77 MB) → no "whole route" total [RE-CHECK R6] | `checker/out/c_cycle.txt` |
| **C4** *(checker)* | a step on `'intake'` (not on robot two) | "'intake' is not on this robot …": step ABSENT, the routine STOPS at step 2 → robot one cannot be previewed on robot two's list [RE-CHECK R2] | `checker/out/c_other.txt` |

Not probed (reasoned, said where used): the Robot-built world's cost (B2; `WallWorld` is the suite's, L6-A2 committed);
a placeholder lift's times (no Lift on any robot today); the HTML's size (estimated from P2b: ~13k samples × ~16 bytes
≈ 210 KB of path text).

---

## §1 Scope — IN (the recommended answer to every decision assumed; [RE-CHECK R5] split into five parts)

**[RE-CHECK R5, R2]** The scope was one RPV1-A and one RPV1-B. It is now five parts, each its own go, in this order:
**A0 → B1 → A1 → A2 → B2** (D10, D11, D13). What each carries:

| Part | Carries | Library files a robot compiles? |
|---|---|---|
| **A0** | the tool and launcher with their caps; the recorder; motions and mechanism steps numbered from below plus the Routine's own stop / skip / (STEP1-A's, if merged) carry-on and jump lines quoted word for word (D2 (a)); mechanism spans attributed by name; wall marks; the field drawing with the caption, the robot line and the PLACEHOLDER notes; THE FIELD CHECK (THROUGH / TOUCH); the no-pace-loop stop; the stdout table; exact-name lookup including refused entries; the registry's refusals; the run-time commit stamp | **none** (one opt-in hook on the host-only fake sink) |
| **B1** | any robot's own mechanism list on fakes by kind, still on the stand-in drive; robot x's fixture lift until G1 lands, robot one's list after; `--robot two\|one` | none |
| **A1** | the spread (D5), the lifted run (D9), the suite check (D8), `--all`, `--svg`, `--outline every:Ns`, `--estimate`, `--no-spread` | none |
| **A2** | the step listener (D2 (b) / D13) and exact step numbers | **`motion/step_listener.hpp` (new), `motion/motion_scheduler.hpp`, `chassis/routine.hpp`** |
| **B2** | the robot built from its own files with its declared wall sensors over the field plant (wall resets SEAT or refuse for real) | none |

### 1.1 RPV1-A0 — robot two's routes, the picture, the table, the field check

**`include/shulib/hal/fake/fake_telemetry_sink.hpp` — CHANGED, host only [RE-CHECK R3]:** an opt-in observer called
after each stored line (`void setOnLine(void (*fn)(void*, std::size_t lines), void* ctx) noexcept`; default none, so
every existing test is unchanged). No file under `src/` includes this header (grep), so no robot build sees it. If
`api_doc_tool.py` covers the fakes, its page is regenerated. *If the reviewer prefers not to touch a fake, the only
other route is a `TesterStack` whose sink type is a parameter — a bigger change to a rig 3,000 tests use; the brief
recommends the hook.*

**`test/route_preview_run.hpp` — NEW (host only; allocates). [RE-CHECK R12c] ALL the program's logic lives here**, as
`int runPreview(std::span<const std::string_view> args, std::ostream& out, std::ostream& err)` — argument parsing,
lookup, the run, the page, the exit code — so the suite calls it (RP10 and the rest); `route_preview_main.cpp` only
forwards `argv`.
- `Recorder` — an `ITickPacer` inserted at `TesterStack::probe.inner` (P1): on every pace it forwards, then stores
  `SimHarness::sample()`, the estimate, the scheduler's active command id, each mechanism slot's `stepsBegun()` /
  `running()`, the guard's `running()`, and `sink.size()`. **[RE-CHECK R10] No cap of its own:** the plant's
  `PlantPacer` already throws "PlantPacer: pace cap exceeded — a wait failed to end" at 200,000 paces
  (`motion_test_rig.hpp` :148-160), below any recorder cap; the page shows that sentence as the run's end.
- **The no-pace-loop stop [RE-CHECK R3]:** the sink's hook compares `sink.size()` with the size the recorder noted at
  the last pace; more than **10,000** lines with no pace in between (a real 60 s run logs about 40: C3 37 lines; the
  590 s lifted C3 run 286) means the auton loops without letting time pass. The hook then writes the page from what
  was recorded — the path up to the last pace, the first 200 of the looping lines and the count — with *"after step N
  the auton keeps looping without letting time pass; on the robot it would sit there until the field disables it, and
  its park would never run"*, prints the table, and ends the process with `std::_Exit(1)` (it may run inside library
  code that must not throw; nothing is unwound). A loop of steps that SUCCEED without pacing logs nothing and is caught
  only by the launcher's 300 s timeout (landmine 10).
- `MotionLog` — the boundary observer (`CompletedMotion`: name, target, final pose, exit, start/end time).
- `PreviewModel run(const AutonRegistry&, std::size_t index, const PreviewOptions&)` — builds a `TesterStack` with
  `cfg.plant.initialPose` = the entry's FIELD start or `--place` (RELATIVE) (P5), attaches the recorder, the hook and
  the observer, calls `TesterStack::run`, detaches, and folds everything into the model, all times T+ from `armedAt`:
  - **steps, numbered from below (D2 (a)):** each motion (from `MotionLog`) and each mechanism step (a slot's
    `stepsBegun()` rising) in order, with its start/end T+ and pose; plus every `RTN` line quoted word for word at the
    pace it dates to — the stop, each skip, and (if STEP1-A has merged) its carry-on and jump lines — which carry the
    code's REAL step numbers. The page's top line: "numbers count drives, turns and mechanism steps; pauses and
    `then(...)` steps are not counted — the library's own lines below give your code's step numbers".
  - **[RE-CHECK R12a] per routine:** the `RTN` lines carry the routine's name (`routine 'forever' STOPPED …`); a second
    `Routine` (a helper's own) starts its own numbering, shown with its name ("helper 'score' · step 2").
  - **[RE-CHECK R4] mechanism spans, attributed by NAME:** every `MECH` line is attached to the mechanism it quotes, not
    to the step open when it arrived. A mechanism commanded by a blocking step is a mark where it ran; one STARTED in
    the background (`startRun` / `startMove` / `startSpinUp` / `startHome`) is its own span along the path, from the
    pose where its slot's `stepsBegun()` rose to the pose where its verdict line arrived, labelled with that verdict; a
    `waitUntilDone` shows the verdict it collected. A failed background move is red on ITS span, never on the drive
    step that happened to be open.
  - **[RE-CHECK R11] PLACEHOLDER numbers:** every step whose time comes from a declaration number marked PROVISIONAL —
    today robot two's claw, `moveTime 0.5_s` (HA-185, `mechanism_table.hpp` :124) — carries the hatched note
    "PLACEHOLDER number — this time is the declaration's 0.5 s (HA-185); Station J times the real one". The list of
    such numbers is a table in the tool keyed by mechanism name, each with its HA number and a test that reads the
    `// PROVISIONAL (HA-…)` comment out of `src/robots/mechanism_table.hpp` so the table cannot drift.
  - wall marks (`WALL` lines; a refusal is amber whatever the step's count says), the result word
    (`bench::testerResultWord`), `armedAt`, the cut (`scoringCut`, `scoringEnded`), the end action's span, the throw
    text, and THE FIELD CHECK (below).
- **THE FIELD CHECK [RE-CHECK R1]:** the outline (D12 (a): a 15 in square centred on the plant's pose,
  `kPreviewFootprintIn`, and an allowance `kThroughAllowanceIn` = 1 in — both named constants the page prints) on
  every sample:
  - **THROUGH** — any corner more than the allowance past a wall's inner face (±70.2047 in): the first such sample is
    marked red with its step, T+ and wall ("the host test plant has no walls; on the robot it hits the left wall
    here"), everything after it faded, exit 1.
  - **TOUCH** — any overlap, within the allowance, with a wall's inner face or a `FieldModel::override2026()` box that
    is not a game piece (a Loader, a Toggle's brackets or body): an amber note per element per contact ("touches the
    Loader here — the plant has no Loader"); exit unchanged.
  - start pieces: a grey note per first contact.
  The page says: "the square is a drawing assumption — the real frame may be smaller, its tracked point may not be its
  middle, and a claw or lift reaching past it is not checked".
- the registry is built in **DROP mode with `kAutonPeriods`** — the programs' own (`autons.hpp` :7-13).
  **[RE-CHECK R12b] A refused entry is not registered**, so the lookup reads the refusal sentences too: each begins
  "auton '<name>': " (`auton_registry.hpp` :458-460); a name found there draws a REFUSED page with the sentence, exit 1
  (P7b, P7c). The registry keeps at most 8 sentences (`kMaxAutonRefusals`, :98); when `refusedCount()` exceeds 8 and
  the name is found nowhere, the answer is "no auton named 'X' — and N more autons were refused than the registry
  keeps the names of; fix the refusals listed first", exit 2.
- a run that throws: `TesterRunRecord::threw` / `threwWhat` (P7a) — the path to the throw, "after step N", PROGRAM
  ERROR, exit 1.
- **[RE-CHECK R8] the stamp:** the launcher passes `--stamp "<git describe --always --dirty --abbrev=7>"` and
  `--autons-hash <first 12 hex of SHA-256(src/team/autons.cpp)>`, both computed at RUN time; the page and stdout print
  both. Without them (the program run by hand) the page says "commit not stamped".

**`test/route_preview_render.hpp` — NEW:** `renderText` (the stdout table, fixed columns), `renderSvg` (the field at
5 px/in, +Y UP: SVG y = −Y; the red station at the bottom; walls, Toggles, Loaders, Load Zones, start pieces from
`FieldModel::override2026()` and `override_walls.hpp`; the path as polylines, one per step, from the recorded samples
ONLY; markers, spans, outlines, the THROUGH mark and faded rest, TOUCH notes), `renderHtml` (the SVG inline, the table,
the caption, the robot line, the stamp, a few lines of inline script for "hover a row, highlight its step"; NO external
URL, NO wall-clock time — two runs of the same tree are byte-identical; every string from the program HTML-escaped).

**`test/route_preview_main.cpp` — NEW** (not `*_test.cpp`, so not globbed into the suite): `return
runPreview(args, std::cout, std::cerr);`. Flags in A0: `NAME`, `--list`, `--place X,Y,DEG`, `--out FILE`,
`--stamp`, `--autons-hash`. Exit codes: **0** drawn, nothing wrong (TOUCH notes allowed); **1** drawn, and the auton
was refused, threw, went THROUGH a wall, or looped without letting time pass; **2** usage (unknown name — lists the
names, writes nothing).

**`test/CMakeLists.txt` — CHANGED:** `add_executable(shulib_route_preview route_preview_main.cpp
../src/team/autons.cpp)` linked to `shulib_test_flags` (`test/CMakeLists.txt` :100-101). Built by the default target,
so CI compiles it. It does NOT depend on the doc-gate step.

**`test/route_preview_test.cpp` — NEW:** A0's cases (§2.2).

**`tools/route_preview.py` — NEW, standard library only:** configures `build/test` if absent, builds
`shulib_route_preview`, computes the stamp, runs it with `RLIMIT_AS` 4 GiB, `RLIMIT_FSIZE` 256 MiB, `RLIMIT_CPU` 600 s,
no core, 300 s wall time (imported from `tools/suite_run.py` by path so they cannot drift), passes the exit code
through, and on a timeout says "the auton did not finish in 300 s of laptop time — a loop of steps that never lets
time pass?". `self-test` proves each bound, the passthrough, the stamp, and **[RE-CHECK R12d]** parses every
`route_preview.py` command line in guide 16's section with its own argument parser (an unknown flag fails it).

### 1.2 RPV1-B1 — any robot's own mechanisms (robot one) [RE-CHECK R2: split from the old RPV1-B]

**`test/auton_tester_rig.hpp` — CHANGED IN PLACE (reuse, not a fork):** `TesterStack` takes the mechanism list and its
devices as a parameter whose DEFAULT is today's (`kRobotTwoMechanisms`, `RobotTwoFakes`), so every existing test that
builds it is byte-for-byte unchanged (RP35 proves it). A new `FakesByKind` builds devices for any list: Power → fake
motors; Pneumatic → fake lines; Arm / Lift → fake motors stepped by `sim::ArmPlant` whose constants are taken FROM the
declaration (so it moves exactly as its declared numbers predict — the plant's job here is to show the declared
timing, not to test the controller); Flywheel → `sim::FlywheelPlant` likewise. The plants are stepped in the chain
the way `test/mechanism_auton_rig.hpp` already steps robot x's lift (`Chassis → RunGuard → MechanismService → World`).
Tested against **robot x's fixture lift** (`test/mechanism_fixtures.hpp`) before G1 lands; switched to robot one's list
(`--robot one`) the day G1 lands — re-reading how G1 shapes robot one's autons file (one file with both, or one each).
Every step of a mechanism whose numbers are the library's STARTING values (`mechanism/kinds.hpp` :40-48, HA-182) — the
cascade, until TUNE1 tunes it — carries the hatched "PLACEHOLDER numbers — this time is what its declaration predicts;
TUNE1 measures the real one". Still the stand-in drive; still no wall sensors.
**Changed:** `route_preview_run.hpp` (`--robot`), tests RP24–RP26, RP35, the guide section.

### 1.3 RPV1-A1 — the spread, the lifted run, the suite check, the extra flags

- **the lifted run (D9) [RE-CHECK R6]:** when `scoringCut`, a copy of the entry with park/stop at 590/595 s into a
  registry whose skills period is 600 s. If it FINISHES: its samples after the real cut drawn dashed to its end, the
  T+60 point found within half a tick, "the whole route would take X s". If it is CUT AGAIN: "this route keeps going
  until its park time (it is meant to be cut)", the dashed part drawn only to T+120 s, no total. Either way the dashed
  part stops at T+120 s ("…continues").
- **the spread (D5) [RE-CHECK R6]:** the same entry on `plantConfig()` with `wheelFf.kA = 0.02` and on
  `WeakSide{1, 0.91}` (the rig's own knobs, `auton_tester_rig.hpp` :81-89, :287-291). A finished run gives its time; a
  cut run gives "cut after step N" (steps completed by the cut). Only those are kept.
- **the suite check (D8 (a)):** `test/route_preview_skills_check_test.cpp` runs every SKILLS entry of
  `registerAutons` and fails on THROUGH, a throw, or the no-pace loop — never on TOUCH or a cut (it prints the margin).
  The no-pace loop inside the suite cannot `_Exit` the suite: in the check, the hook THROWS a named exception
  instead. That is safe at the skip site — C1's `std::bad_alloc` came out of exactly that `push_back` and was caught by
  `runTesterOnce` and reported (`checker/out/c_forever.txt`) — and RP27b proves the named one is caught the same way and
  that the check reads it as the loop, not as an ordinary crash. A loop whose lines come from a noexcept caller would
  terminate instead; if RP27b finds one, the check runs each skills entry in a child process. Today no skills entry is
  registered: the case says so with a MESSAGE and passes; RP21 proves it bites with a fixture registry.
- `--all` (one page per auton and an index), `--svg`, `--outline every:Ns`, `--estimate`, `--no-spread`.

### 1.4 RPV1-A2 — the step listener (exact step numbers; D13)

As the old §1.1 described it, unchanged in design: `include/shulib/motion/step_listener.hpp` (NEW, a leaf,
PROS-free, allocation-free: `StepPhase { Began, Succeeded, Failed, FailedCarriedOn, Skipped, Jumped }` append-only;
`StepEvent { routine, index, step, label, phase, cause }`; `IStepListener::onStep(const StepEvent&) noexcept`; must
not throw, must not call a scheduler verb, nothing attached on a robot); `motion_scheduler.hpp` gains
`setStepListener` / `stepListener()` beside `setBoundaryObserver` (:928); `routine.hpp` calls a private `notifyStep`
at the door and at each counter site, on the merged tree **after STEP1-B and CURVE1** (D13), so it is edited once,
in sequence. The tool's `StepLog` keeps each event with the clock, **with the same no-pace cap as the sink [RE-CHECK
R3]** (10,000 events with no pace → the same stop); the page's numbers become the code's, and the "numbers count
drives…" line goes away. Placed on the scheduler, not on the frozen `Chassis` (F6); not pinned. Bit-identity: RP5
(re-captured, [RE-CHECK R9]).

### 1.5 RPV1-B2 — the robot as its program boots it (after L6-B)

`test/route_preview_world.hpp` — NEW: a world built like `WallWorld` (`test/wall_robot_rig.hpp`) from the robot's OWN
files: `robot::Robot` from `bench::bootConfigFrom(<its typed description with a STAND-IN track width, named on the
page>, …, <its mechanism list>)` plus the robot file's `walls` block (L6-B adds robot two's), over
`FakeDeviceProvider`; the field and side-sensor plants; B1's `FakesByKind`; the run is `competition::runAuton` (the
competition program's call — no countdown, no abort watch). Wall resets SEAT or refuse for real. Tests RP22, RP23.

### 1.6 What the preview shows for STEP1-B, CURVE1 and AF1-S steps (they land later)

- **STEP1-B's trigger partway through a drive:** visible from below already (a mechanism slot's `stepsBegun()` rising
  while a motion is active); the page draws a TICK on the path at the pose of the firing pace, labelled with the
  mechanism and the verdict ("at 60 % of drive 7: lift → SUCCEEDED") — the same name attribution as a start form
  [RE-CHECK R4] — and under the drive "trigger never fired: the drive ended early" when STEP1-B's own line says so.
  With A2's listener, `StepPhase::TriggerFired` (appended by A2, which comes after STEP1-B, D13) carries the trigger's
  own words.
- **CURVE1's curve:** a curve is a motion — numbered like any drive; the page draws the path the plant DROVE (the
  truth) and the curve's end target (×). Drawing the curve's PLANNED points too needs CURVE1's path to be readable by
  the tool (a later field; not in RPV1).
- **AF1-S's Toggle step** (AF1S-A1): a step like any other, its log lines attached; on the tester rig the camera is a
  `FakeVision` with no frames, so it is drawn as the step's own "could not see" answer (a camera world is OUT, §5).

---

## §2 Tests, mutations, definition of done

### 2.1 Runs and sizes (each its own go; verified and committed before the next)

| Run | Base | Builder-days | Then |
|---|---|---|---|
| **RPV1-A0** *[RE-CHECK R5]* | main's tip at launch (STEP1-A merged or not — A0 reads its lines if it is) | **~1½** (tool core ¾, field check + spans ¼, launcher + CMake + sink hook ¼, tests + mutations ¼… docs within) | review + fix ~½ |
| **RPV1-B1** *[RE-CHECK R2]* | main after A0 merges (not tied to L6-B); robot one's list after G1 | **~1** (+ ~1 hour when G1 lands) | review ~½ |
| **RPV1-A1** | main after A0 (and B1) merge | **~1** | review ~½ |
| **RPV1-A2** | main after STEP1-B AND CURVE1 merge (D13) | **~½** | review ~½ |
| **RPV1-B2** | main after L6-B (robot two's walls) | **~1** | review ~½ |
| **Total** | | **~5** | **~2½** — before the 21st, A0 + B1 = ~2½ + ~1 |

### 2.2 Tests — ADVERSARIAL: each tries to break ONE rule

**[RE-CHECK R5] Which part carries which:** **A0:** RP1, RP2, RP3, RP6, RP8, RP9, RP10, RP11, RP13, RP14, RP15, RP16,
RP17, RP18, RP27, RP28, RP29, RP31, RP32, RP33, RP34. **B1:** RP24, RP25, RP26, RP35. **A1:** RP7, RP12, RP19, RP20,
RP21, RP27b, RP30. **A2:** RP4, RP5 (and RP31 re-run on exact numbers). **B2:** RP22, RP23. Where a case below says
"step N", in A0 it means the library's own `RTN` number quoted on the page, or the from-below count where the case
says so.

- **RP1 the picture's path IS the plant's poses.** For the acceptance run and P2's skills auton: every recorded sample
  equals `SimHarness::sample()` taken by an independent tap at the same pace (memcmp); the summed path equals
  `PlantPacer::distance` exactly (P1); the SVG is parsed back (the polyline points) and each point equals the sample
  mapped by the field transform within 0.01 px. Under an `OffsetImu` (`autons_test.cpp`'s, +5°) the drawn path is the
  TRUTH, not the estimate.
- **RP2 a FIELD start is drawn at the declared start** — the first sample equals the entry's start (P5a), and the
  estimate and truth agree at the end.
- **RP3 a RELATIVE start is drawn at `--place`** — the 40in auton placed at (-48,-48,90°) ends at (-48.00, -8.23)
  ±0.01 (P5c); without `--place`, the red "placed at the centre" line is on the page and in stdout.
- **RP4 the numbers are the Routine's.** For STEP1-A's golden chains (`test/routine_golden_chains.hpp`, in flight) and
  P8's chain: the listener's indices run 1..`steps` with no gap, `Failed` sits at `stoppedAt` with `stoppedName`,
  `Skipped` count = `skipped`, `Jumped` = `jumped`, `FailedCarriedOn` = `failedOn` and the first at `firstFailedAt`.
- **RP5 no listener ⇒ bit-identical. [RE-CHECK R9]** STEP1-A's golden chains hold only the nine `RoutineResult`
  fields and the `RTN` lines (`routine_golden_chains.hpp` :10), not the truth stream. So A2 FIRST captures its own
  FNV-1a hashes of the whole truth stream and of the sink's records for each golden chain on the merged tree WITHOUT
  the seam (as P11 did), commits them as constants with the command that made them, then asserts the same hashes with
  no listener and with one attached.
- **RP6 a failed step is drawn as failed.** A chain whose step 3 times out: step 3 red at its pose, its sentence = the
  `RTN` stop line verbatim; steps 4–5 in the grey "never ran" list and NOT on the path; result word ROUTINE STOPPED.
- **RP7 the 60 s line is at the right place.** P2's auton: the tick's pose equals the lifted run's sample at
  `armedAt + 60` (within half a tick) and it sits in the step that was open then; for a MATCH entry the line is at 30 s.
- **RP8 a refused start.** (a) a skills entry with stop 61 s → the page says REFUSED with the registry's sentence (P7b),
  no path, exit 1; (b) an entry with no start (P7c) the same; (c) `runAuton`'s own refusal (a route entry with no
  registry is not reachable from the file — a fixture through the same code) → REFUSED, nothing moved.
- **RP9 an auton that throws** (P7a) → PROGRAM ERROR with its text, the path to the throw, "after step 1" (or the open
  step's number when it threw inside one), exit 1.
- **RP10 an unknown name** → exit 2, every registered name listed, NO file written; a name differing only in case is
  unknown too. **[RE-CHECK R12c]** Tested by calling `runPreview(...)` from `route_preview_run.hpp` (the suite cannot
  call `main`); `route_preview_main.cpp` is one line, checked by the coordinator's §7 command.
- **RP11 going THROUGH a wall. [RE-CHECK R1]** P6's drive → THROUGH at the FIRST sample whose outline CORNER passes
  70.2047 + 1 in (not the centre — 7.5 in earlier; not at mere contact), its T+ and step, everything after faded,
  exit 1.
- **RP12 the cut.** P2's auton → CUT BY DEADLINE at `scoringEnded` 57.36 s, step 71 amber with its sentence, 89 listed as
  never ran; the lifted run dashed after the cut to T+120 s, "the whole route would take 129.0 s".
- **RP13 mechanism steps** are drawn at the pose where they BEGAN with their verdict word from the `MECH` line
  (UNCONFIRMED counted done in its own colour, not green).
- **RP14 a wall reset that refuses** (L6-A2's call, committed `16c01e0`; P9) → amber, "no sensor", never green, though the step is
  counted done.
- **RP15 the caption and the robot line** are on every page: "IDEAL host test plant", the stand-in drive's words, the
  commit; the page contains no `http`, no `src=`/`href=` to anything outside itself, and not the word "simulator".
- **RP16 the stdout table equals the page's table** (both rendered from one model; compared cell by cell).
- **RP17 determinism** — two runs of the same tree write byte-identical pages.
- **RP18 escaping** — an auton named `a<b & "c"` (printable ASCII the registry accepts) appears escaped; no raw `<b`.
- **RP19 the spread [RE-CHECK R6]** — for a route that finishes, the three times on the page equal three separate runs'
  `scoringEnded` (P10's numbers for the team's autons); for C3's cycle route (cut in all three), the page shows "cut
  after step N" for each, N equal to each run's completed count, and NO time.
- **RP20 a wait that never ends [RE-CHECK R10]** — a fixture auton with a frozen `pause(3000_s)` (the guard cannot cut a
  frozen pause). The executor FIRST probes what ends it (the plant's 200,000-pace cap, "PlantPacer: pace cap exceeded —
  a wait failed to end", or the guard's hard floor at the stop time — not verified, R10) and the case asserts that
  sentence on the page, a bounded memory, and no out-of-memory.
- **RP21 the CI check bites** — a fixture registry with a skills auton that goes THROUGH a wall → the check's case is
  red for it; one that throws → red; one that loops with no pace → red (RP27b); one that only TOUCHES a Loader → green;
  one cut at its park time → green with the margin printed. *[RE-CHECK R1]*
- **RP27 the loop with no pace [RE-CHECK R3]** — C1's `for(;;){ r.moveTo(A); r.moveTo(B); }` (skills 57/59 s) ends
  through the no-pace stop, NOT `std::bad_alloc` and not the 300 s timeout: the page exists with "after step 39 the
  auton keeps looping without letting time pass", exit 1, in under 5 s and under 200 MB (`runPreview` run in a forked
  child by the case, since the stop calls `_Exit`). **RP27b** (A1): in the suite check the same route is caught as the
  loop (the named exception), and the check's case is red for it.
- **RP28 a background mechanism's verdict [RE-CHECK R4]** — C2's `startRun(clawPivot6, 12 V, 1 s)` then a 40 in
  `moveTo`, `waitUntilDone`: the pivot's SUCCEEDED is on the pivot's own span (from its start pose to its T+1.02 pose),
  and the drive step carries NO mechanism verdict; a variant whose background move FAILS draws its span red and leaves
  the drive green.
- **RP29 a TOUCH is not a failure [RE-CHECK R1]** — (a) a FIELD start with its back on the red wall (y = −62.7, heading
  90.1°) → a TOUCH note at T+0.00, exit 0; (b) a drive up to a Loader's face → "touches the Loader", exit 0; (c) the
  same drive 1.5 in further → THROUGH, exit 1; (d) a route that hugs the wall 0.1 in inside → no note at all.
- **RP30 a route cut twice [RE-CHECK R6]** — C3's cycle route: the lifted run is cut again; the page says "this route
  keeps going until its park time (it is meant to be cut)", draws dashes only to T+120 s, and contains no "whole route"
  total.
- **RP31 two routines [RE-CHECK R12a]** — an auton whose helper builds its own `Routine` named "score" and stops at its
  step 2: the page shows "score · step 2" with the helper's sentence, and the main routine's numbering is not reset by
  it.
- **RP32 a refused entry by name [RE-CHECK R12b]** — `"too long"` (stop 61 s) is found by its name among the refusals
  and drawn REFUSED with the registry's sentence, exit 1 (not "no auton named", exit 2); with 9 refused fixtures and the
  name in the 9th, the "more autons were refused than the registry keeps" sentence, exit 2.
- **RP33 the PLACEHOLDER note [RE-CHECK R11]** — every claw step on the page carries "PLACEHOLDER number … (HA-185)";
  no pivot step does; the tool's table of provisional numbers equals the `// PROVISIONAL (HA-…)` comments read out of
  `src/robots/mechanism_table.hpp`.
- **RP34 the run-time stamp [RE-CHECK R8]** — `runPreview` given `--stamp abc1234-dirty --autons-hash 0123456789ab`
  prints both on the page and stdout; without them, "commit not stamped"; the launcher's `self-test` shows the hash
  changing when a scratch copy of `autons.cpp` changes with no reconfigure.
- **RP35 the rig's default is unchanged [RE-CHECK R2]** (B1) — a `TesterStack` built with the new parameter left at
  its default runs the team's two autons and P2's skills auton with the truth stream's FNV-1a and the records
  identical to the hashes captured before the change; robot x's lift on `FakesByKind` moves in its declared time.
- *(RPV1-B2)* **RP22** a robot-two world with L6-B's walls block: the left wall reset SEATS with its correction drawn;
  **RP23** the same auton on the tester rig and the boot-built world draw the same steps (drive geometry differs — the
  times are compared only for order). *(B1)* **RP24** a placeholder lift step (robot x's lift) carries the hatched note; **RP25** a robot with no file
  (robot one before G1) → "robot one has no build yet", exit 2; **RP26** `--robot one` picks robot one's list (after
  G1; before it, the fixture list).

### 2.3 Mutations — each applied, the suite run through `suite_run.py`, RED observed, restored, logged

| # | Mutation | Must turn red |
|---|---|---|
| RM1 | the recorder samples BEFORE `inner->pace()` (one tick stale) | RP1 |
| RM2 | the path drawn from the ESTIMATE | RP1 (OffsetImu) |
| RM3 | SVG y not flipped | RP1, RP2 |
| RM4 | the plant not placed at the FIELD start | RP2 |
| RM5 | `--place` heading applied with the wrong sign | RP3 |
| RM6 | Began reported with `steps_` instead of `steps_ + 1` | RP4 |
| RM7 | the listener called from inside a paced wait (a behaviour change) | RP5 |
| RM8 | skipped steps given the stop's pose and drawn on the path | RP6 |
| RM9 | T+ measured from the clock's zero, not `armedAt` (3.52 s off, P1) | RP6 (A0: step 1 begins at T+0.00), RP7, RP12 |
| RM10 | a Throw-mode registry | RP8 |
| RM11 | the throw drawn as FINISHED | RP9 |
| RM12 | a case-insensitive / nearest-name match | RP10 |
| RM13 | the field check on the centre, not the outline | RP11 |
| RM14 | the wall at 72 in | RP11 |
| RM15 | a refused wall reset drawn by its step's verdict (green) | RP14 |
| RM16 | the caption dropped | RP15 |
| RM17 | a wall-clock time stamp in the page | RP17 |
| RM18 | no HTML escaping | RP18 |
| RM19 | the CI check reads the result word only (misses a THROUGH on a FINISHED run) | RP21 |
| RM20 | the launcher's RLIMIT_AS not applied | the launcher's `self-test` |
| RM21 *[R4]* | `MECH` lines attributed to the step open when they arrive | RP28 |
| RM22 *[R3]* | the no-pace stop disabled (threshold never reached) | RP27 (ends in `bad_alloc` or the case's own time limit, not the page) |
| RM23 *[R1]* | every overlap counted as THROUGH (allowance 0) | RP29 (a), (b) |
| RM24 *[R8]* | the stamp taken from `SHULIB_BUILD_HASH` (configure time) | RP34 |
| RM25 *[R12a]* | one step counter across all routines | RP31 |
| RM26 *[R6]* | a cut-again lifted run reported with its `scoringEnded` as the "whole route" total | RP30 |

**[RE-CHECK R5] Which part applies which:** A0 — RM1–RM5, RM8, RM9, RM10–RM18, RM20–RM25; B1 — the RP35 hash
check made to fail by changing the default list; A1 — RM19, RM26 and RP12/RP19's own; A2 — RM6, RM7.

### 2.4 Definition of done — every run

The suite green through `python3 tools/suite_run.py green` with the new cases counted; every mutation of its run RED then
restored, logged with the case that went red; the ARM header gate green (A2 only — the new header and the two edited
ones in one TU with the robot's flags; A0 and B1 also run it once to show nothing a robot compiles moved); `api_doc_tool.py` check-coverage / check-fresh / check-examples /
check-removability, `doc_staleness_audit.py`, `briefing_status.py` green; **the src gate's 7 builds green and `git diff
<base> -- src/` EMPTY** (and for A0, A1, B1, B2 `git diff <base> -- include/` touches only `hal/fake/` — [RE-CHECK R5]);
`python3 tools/route_preview.py "40in forward"` and `"TEST 24in turn back"` exit 0 and their pages open (the log names
the file sizes); the prose review logged (§3.3); RPV1-PROGRESS.md current; nothing committed.

---

## §3 Documents — in the same change as the code (pace rule 3), the prose review logged (rule 4), ≤ ~5 public each (rule 7)

### 3.1 RPV1-A0 (public) — A1 and A2 add to the same section and the changelog [RE-CHECK R5]
1. **`docs/guide/16-testing-an-auton-on-the-bench.md` — NEW section "Before the bench: previewing a route"** (chapter 16
   is about running an auton; a new chapter would race G3a's renumbering to 20). In plain words: the one command; what
   the page shows; how to read a red, amber and grey step; THROUGH vs TOUCH; why the numbers count drives, turns and
   mechanism steps (until A2); the dashed rest-of-route, the 60 s tick and the spread line (A1);
   **what it does NOT tell you** — the robot's real times (slip, a weak side, the battery, mechanisms slower than their
   numbers), anything the field has that shulib does not draw, pieces the robot pushes, whether a wall reset will seat
   on robot two (it cannot until B2 and L6-B), whether the start is legal, robot one's routes (not until B1 and G1); "the
   preview is a plan, the robot is the test". **[RE-CHECK R12d]** Its command lines are in ```` ```bash ```` blocks,
   which check-examples does NOT scan (`api_doc_tool.py` :2073 reads only ```` ```cpp ````); the launcher's `self-test`
   parses each `route_preview.py` line there with its own argument parser instead.
2. **`docs/guide/15-glossary.md`** — "route preview", "host test plant" (amended to mention the preview).
3. **`docs/guide/14-what-it-cannot-do-yet.md`** — one entry: the preview's drive is a stand-in; it has no walls; no
   camera world.
4. **`docs/changelog.md`** — the tool, and the fake sink's hook (host only). (A2 adds the step listener: additive; "the
   coordinator numbers the minor at merge"; "the robot programs are recompiled and behave the same".)
5. **`test/README.md`** — the tool's files (and A1's CI check).
(Generated by `api_doc_tool.py` if it covers the fakes: `docs/api/fake_telemetry_sink.md` or its entry. A2:
`docs/api/step_listener.md`, `motion_scheduler.md`, `all-entities.md`, `README.md`.)

### 3.2 RPV1-B1 and B2 (public)
Guide 16's section: `--robot` and robot one, the PLACEHOLDER hatching (B1); real wall resets (B2); changelog;
test/README.

### 3.3 The prose review's greps (each run runs them and logs the hits it read)
`simulator` (must be absent from every new file and the page), `lower bound` (must be absent — §0.3), `exact` /
`guarantee` near "time", `robot two` (every claim about the drive says "stand-in"), `internal/|chunks/|RESUMING`
(none in public docs), `RC[0-9]|RP[0-9]|RM[0-9]` (no internal tag in a public doc or a header).

---

## §4 Hardware Assumptions — none expected, ONE conditional [RE-CHECK R1, writer's finding (iii)]

The tool's numbers are already registered or are drawing facts: the stand-in drive (HA-45 gains, HA-50 speed budget,
HA-51 settle), robot two's unset track width (HA-125), the mechanism plants (HA-183) and starting gains (HA-182), the
claw's move time (HA-185).
**The conditional row:** the outline ("the robot is a 15 in square centred on the point the plant tracks", with a 1 in
allowance) is only a drawing fact while nothing fails on it. If the team lead picks **D8 (a) and D12 (a)**, the suite
FAILS on it, so it becomes a belief code acts on and gets a register row in A1 (where the suite check lands): the size
is the team lead's stated 15 in (2026-10-08) and the start-size rule, NOT a measured frame; the tracked point is assumed
to be the frame's middle; a mechanism that extends past it is not checked; the allowance is a choice. With D12 (b) the
row records the builder's measured numbers instead.
**The number:** at `16c01e0` the register's Held line ends at HA-227 (`docs/hardware-assumptions.md` :39); AF1's
HA-228..236, ALLY1+L4's HA-237..242 and AF1-S's HA-243..245 are reserved in briefs; **CURVE1's brief holds HA-246..250**
(`briefs/CURVE1-curve-follower.md` :39 and its A14, which already names RPV1's as HA-251). So RPV1's row would be
**HA-251** — the coordinator confirms at A1's launch. If the executor finds any other belief that needs a row, it stops
and tells the coordinator.

---

## §5 OUT, and explicitly rejected

- **A planner page / any drawing → code** (the team lead chose "code + preview only"); any write to `src/team/` (G4).
- **The panel's ROUTES page, mirroring, the F8 route file** (G3a, after the 21st).
- **A camera world** (VIS1's `sim/camera_plant.hpp` driving AF1-S's step on the page) — a later RPV1-C if wanted.
- **Contact physics**: walls that stop the robot, pieces that move, robots that collide (D6 (c) rejected).
- **A "legal start" check** (the skills starting rules) and **scoring** (no points are counted).
- **Robot timing**: the page never claims a robot time; "lower bound" is withdrawn (§0.3).
- **Route autons (F8 `entry.route`)**: drawn from below (legs and markers) without step numbers; the team writes
  `Routine` code.
- **A brain-screen step display** using the listener (possible later; not asked).
- **A fuzzy name match** (D7, RM12).

---

## §6 Coordination — in flight while this was written (read only, 2026-11-10 ~23:30, re-checked at `16c01e0`; NOT edited) [RE-CHECK R7]

| Run | Where | What it touches that RPV1 meets | Rule |
|---|---|---|---|
| **L6-A2** | **COMMITTED `16c01e0`** | `motion/motion_scheduler.hpp`, `competition/auton_wall_reset.hpp`, `robot/robot.hpp`, `test/wall_robot_rig.hpp` | in the base; A0's wall marks (RP14) use the everyday call; B2 uses `wall_robot_rig.hpp`'s pattern |
| **MECH-S-A** | **staged on main** (54 files; merging first, NOTES :965) | `mechanism/service.hpp` (accessors move: `lastStep` :272, `running` :274, `stepsBegun` :279 in the index), `sim/mechanism_plant.hpp`, `hal/fake/fake_motor.hpp`, `test/calibrate_program_rig.hpp`, `test/mechanism_position_rig.hpp` — NOT `auton_tester_rig.hpp` | A0 reads slots only; B1's `FakesByKind` and its `ArmPlant` use re-read the merged tip |
| **CAL1-B** | `cal1-b`, done, second in the merge queue | Calibrate, `src/` | none |
| **STEP1-A** | `.claude/worktrees/step1-a`, **DONE, third in the merge queue** (NOTES :965) | `chassis/routine.hpp` (door `enterStep`, carry-on, jump), `chassis.hpp`, `motion_config.hpp`, `src/programs/*`, `test/routine_golden_chains.hpp` | A0 does not wait for it; if merged at A0's launch, A0 quotes its carry-on / jump lines too. A2 is written on its merged `routine.hpp`; RP4 uses its golden chains, RP5 re-captures hashes (R9) |
| **TUNE1-A** | `tune1-a` | `mechanism/*` | B1's placeholder note names TUNE1 as what measures the real numbers |
| **VIS1-A2** | `vis1-a2` | `vision/*` | none (no camera world, §5) |
| **AF1-S** (brief done) / AF1S-A1 (next build) | — | a new Toggle step; may touch `auton_tester_rig.hpp` | A0 and B1 re-read the rig at launch; the Toggle step is drawn like any step (§1.6) |
| **CURVE1** (briefed; holds HA-246..250) | — | a curve verb in `routine.hpp` / `chassis.hpp` | A0/A1/B1 do not touch `routine.hpp`; A2 comes AFTER CURVE1 (D13), so no "whichever lands second" |
| **STEP1-B** (queued) | — | `routine.hpp`, `mechanism/service.hpp`, triggers | A2 comes AFTER STEP1-B (D13) and appends `StepPhase::TriggerFired` at its fire path; until then the trigger is drawn from below (§1.6) |
| **G1** (brief in flight, `w4fb9rems`) | — | robot one's files and how its autons are registered | B1 is built against robot x's fixture lift and switched to robot one's list the day G1 lands |
| **L6-B** (queued) | — | robot two's walls block | B2 waits for it |

Shared files: A0 writes `test/CMakeLists.txt` (one target; the globbed suite list is untouched),
`include/shulib/hal/fake/fake_telemetry_sink.hpp` (the hook), guide 14/15/16, changelog, test/README. B1 writes
`test/auton_tester_rig.hpp` (a defaulted parameter). A2 writes the three library files of §1.4. **No `src/` file,
ever** (§7 checks it).

---

## §7 Checks — the coordinator re-runs every one after each run (RESUMING's step 3: never the report's word)

```bash
# the suite, bounded
python3 tools/suite_run.py green
# the ARM header gate — every v2 header in ONE TU (A2: step_listener.hpp, motion_scheduler.hpp, routine.hpp)
# A0/A1/B1/B2: include/ touched only under hal/fake/ [RE-CHECK R5]
git diff --stat <RUN_BASE> -- include/ | grep -v 'hal/fake/'   # must print only the summary line
# the CI guards, api_doc_tool (coverage, fresh, examples, removability), doc_staleness_audit, briefing_status
# the src gate: 7 builds, and src/ untouched
python3 tools/src_build_gate.py && git diff --stat <RUN_BASE> -- src/   # must print nothing
# ── the brief's own ──
(ulimit -v 4194304; timeout 300 python3 tools/route_preview.py "40in forward"); echo "exit $?"      # 0
(ulimit -v 4194304; timeout 300 python3 tools/route_preview.py "TEST 24in turn back"); echo "exit $?" # 0
(ulimit -v 4194304; timeout 300 python3 tools/route_preview.py "no such auton"); echo "exit $?"     # 2, names listed, no file
(ulimit -v 4194304; timeout 300 ./build/test/shulib_route_preview "40in forward"); echo "exit $?"   # 0, page says "commit not stamped" (main forwards to runPreview, R12c)
# [RE-CHECK R3] C1's loop (a scratch autons copy, never src/): ends through the no-pace stop, exit 1, page written
# (A1) the lifted run, spread, CI check and --all:
(ulimit -v 4194304; timeout 300 python3 tools/route_preview.py --all); ls -la build/route_preview/
cmp build/route_preview/40in\ forward.html <(second run's copy)        # RP17 by hand
grep -c 'http\|simulator' build/route_preview/*.html                    # 0
grep -n 'IDEAL host test plant' build/route_preview/*.html              # every page
python3 tools/route_preview.py self-test
# a mutation subset re-run by the coordinator — A0: RM1, RM9, RM13, RM15, RM21, RM22, RM23; A1: RM19, RM26; A2: RM6 + RP5
```

---

## §8 Landmines

1. **Times start at `armedAt`, not 0.** The tester's 50 idle ticks and 3 s countdown come first (`armedAt` 3.52, P1); the
   competition program has no countdown. Every T+ is `t − armedAt` (RM9).
2. **Place the plant at the FIELD start** — or the picture is wrong while the estimate looks right (P5b, RM4).
3. **The plant has no walls** (P6). Never "fix" that by clamping (D6 (c)); detect and say.
4. **A refused wall reset is a counted-done step** (P9): draw the `WALL` verdict, not the step's (RM15).
5. **The final `COMP` line is logged after the last pace** (P1: it dates to "after the end") — attach it to the result,
   not to the last step.
6. **The 60 s tick:** compare clocks within half a tick (P2b: 4×10⁻¹² s of float drift).
7. **The listener must not change a run** — no pace, no log, no scheduler verb inside `onStep`; `noexcept`; RP5/RM7.
8. **The tester rig is a test header** (`test/auton_tester_rig.hpp`); the tool depends on it, and a change to the rig
   changes the tool — the tool's own suite cases catch it. Never copy the rig into the tool (the task's "reuse, not
   fork").
9. **The outline is 15 in square** — a claw or lift that extends past it after the start is not checked; say so.
10. **[RE-CHECK R3, R10] Three ways a route never ends, three different stops.** (a) A wait that never ends (a frozen
    `pause(3000_s)`) paces on: the plant's 200,000-pace cap ends it with its own sentence (`motion_test_rig.hpp`
    :148-160) — or the guard's floor, not verified (RP20); the recorder has NO cap of its own. (b) A loop AFTER a stop
    or cut (`for(;;){ r.moveTo(A); r.moveTo(B); }`) does not pace but LOGS a skip line per step: without the no-pace
    stop it grows the fake sink to 16.7 million lines and dies of `std::bad_alloc` at ~2.6 GB (C1) — the hook stops it
    at 10,000 lines (RP27). (c) A loop of steps that SUCCEED without pacing (`for(;;) r.then([]{ return true; })`)
    neither paces nor logs: only the launcher's 300 s timeout catches it, and its sentence says so. The `DebugRecord`
    channel also grows with the run (P2: ~5.6 MB per 60 s), bounded by (a).
11. **An auton that reads the clock or the guard behaves differently on the lifted run** — the dashed part is labelled
    "schedule lifted".
12. **`claw["open"]` is checked at compile time** — a position name computed at run time does not compile (found
    writing P2's fixture); fixtures use literals.
13. **Names are exact and may contain spaces** — the launcher passes them as one argument; RP10.
14. **[RE-CHECK R8] The commit stamp is taken at RUN time, never from `SHULIB_BUILD_HASH`.** That macro is set at
    configure time and re-run only when `.git/HEAD` / `logs/HEAD` move (`test/CMakeLists.txt` :29-73, whose own note
    says a dirty flag "can lag until the next reconfigure"), so an edited `autons.cpp` could be stamped clean. The
    launcher passes `git describe --always --dirty --abbrev=7` and a short SHA-256 of `src/team/autons.cpp` (RP34).
15. **"Robot two" is only its mechanisms in A0** — every sentence about the drive says "stand-in" (§3.3's grep).
16. **[RE-CHECK R2] A step on a mechanism the robot does not have STOPS the route** (`notOnThisRobot`, C4) — on A0,
    every cascade step of a robot-one route does that. The page's top line says "previewing ROBOT TWO's mechanisms:
    a step on any other mechanism stops the route here, as it would on robot two" whenever that is the stop.
17. **[RE-CHECK R4] A mechanism's verdict can arrive during a LATER step** (the start forms, C2; STEP1-B's triggers).
    Attribute `MECH` lines by the quoted name, never by the open step (RM21).
18. **[RE-CHECK R1] Touching is normal.** A wall start is within 0.01 in of the wall face; loading means touching a
    Loader. Only THROUGH (beyond the allowance) is a failure (RM23).

---

## §9 Summary [RE-CHECK — rewritten]

**What it is.** One command — `python3 tools/route_preview.py "skills"` — runs that auton from the team's own
`autons.cpp` through the real library on the host test plant, the way the competition program runs it, and writes one
offline page: the field to scale, the path, every drive, turn and mechanism step numbered where it started and ended
with its time, outlines, mechanism and wall marks (a mechanism running in the background as its own stretch of path),
failures in their colours with the library's own sentences, touches and wall hits, placeholder numbers marked, the
total against 60 s — plus the same table in the terminal. Later parts add the spread of times, the dashed rest of a cut
route with its 60 s mark, a suite check, exact step numbers, robot one, and real wall resets.

**What the probes settled.** The suite's tester rig takes a recorder without a fork and records the path exactly (P1); a
60 s auton costs ~30 ms and ~12 MB (P2); the plant must be placed at a field start (P5); the plant has no walls (P6);
the Routine is silent on success (P8); a 1-pointer listener gives exact numbers and changes no host run (P11); wall
resets refuse on the tester rig (P9); ideal times are NOT lower bounds (P10). **The checker's probes added:** a route
that loops after a stop exhausts memory (C1); a background mechanism's verdict lands in a later step (C2); a "cycle
until time" route is cut even with the clock lifted (C3); a robot-one step stops the route on robot two's list (C4).

**The parts, sizes and honest dates** (today Tue Nov 10, late; robot one built ~Sat Nov 14; freeze ~Wed Nov 18;
competition Sat Nov 21). Builder-days are effort; calendar time has run 2–4× that lately (STEP1-A: launched Nov 6,
done Nov 11, with a 5-hour-limit stop and a shutdown between — NOTES :949, :954, :965), so every date is a range.
- **A0 — robot two's routes. ~1½ builder-days + ~½ review.** With D11 (a), launched ~Nov 11: in the team's hands
  **Nov 13 at best, Nov 14–15 with one stop**. With D11 (b): ~Nov 16–17.
- **B1 — any robot's mechanisms (robot one). ~1 builder-day + ~½ review**, right after A0: ready on robot x's fixture
  lift **~Nov 14–16**. **Robot one's own route previews the day G1 lands** (+ about an hour). G1's date is G1's brief's
  to give; if G1 lands Nov 17–18, robot one's skills route gets at most a day of previews before the freeze. **The
  sentence "RPV1-B can land after the freeze and still serve the 21st" is withdrawn** — routes are code and freeze too.
- **A1 — spread, dashed rest of route, suite check, extra flags. ~1 + ~½.** ~Nov 16–17 if a slot is free; otherwise
  after the 21st. Nothing in it is needed to SEE a route.
- **A2 — exact step numbers (the library change). ~½ + ~½.** After STEP1-B and CURVE1 both merge; after the 21st
  unless both are in by ~Nov 16 (D13).
- **B2 — real wall resets. ~1 + ~½.** After L6-B; most likely after the 21st.
- **Total ~5 builder-days + ~2½ review**, against the "~1 build, 2–3 days" the team lead heard. **A0 alone matches that
  promise for robot two; robot one needs B1 and G1 as well.**

**The decisions, as I would put them to the team lead (plain words):**
- **D1** One command with the auton's name; it builds and runs everything and tells you where the page is.
  *(Recommended.)*
- **D2** First version: the picture counts drives, turns and mechanism steps, and prints the library's own "STOPPED at
  step 12" lines, which carry your code's real numbers. Exact numbering comes later (D13). *(Recommended.)*
- **D3** First version previews robot two only, on the suite's pretend drive. **A robot-one route stops at its first
  cascade step there** — robot one needs the second part (B1) and robot one's file (G1). *(Recommended.)*
- **D4** One page that works offline, plus the table in the terminal. *(Recommended.)*
- **D5** The page shows everything listed above, with "PLACEHOLDER" on times nobody has measured; later, a spread of
  times (or, for a route cut at its park time, how far each version got). *(Recommended.)*
- **D6** Failures red with the library's sentence; refused wall resets amber; skipped steps listed. Going THROUGH a wall
  is red and fails the command; just TOUCHING a wall, Loader or Toggle is an amber note and does not. *(Recommended.)*
- **D7** One auton by exact name; a wrong name lists the right ones and draws nothing; a refused auton is found by name
  and shown with the reason. *(Recommended.)*
- **D8** The test suite fails only if a skills route goes THROUGH a wall, crashes, or loops without time passing — and
  that would hold up library merges until fixed. A route cut at its park time never fails it. *(Recommended; options:
  stricter, or report only.)*
- **D9** When a route is cut, draw where the rest would go with the 60 s mark, up to 120 s; if it would be cut again,
  say it is meant to be cut, and give no total. *(Recommended.)*
- **D10** Five small parts: robot two first (A0), robot one second (B1), the extras third (A1), exact numbers and real
  wall resets last (A2, B2). *(Recommended.)*
- **D11** Put the first part right after the Toggle step, ahead of the tuning, trigger and curve builds — it touches no
  library file, and routes are written Nov 14–18. Costs those builds about a day and a half if all slots are busy. And
  don't let robot one's build wait behind the extras. *(Recommended.)*
- **D12** For the wall check, treat the robot as a 15 in square around its tracked point, with 1 in of slack before it
  counts as going through. Or give us each robot's measured frame. *(Recommended: the 15 in square for now.)*
- **D13** The library change for exact step numbers waits until the trigger and curve changes have both merged, and
  then only if that is by ~Nov 16; otherwise after the competition. *(Recommended.)*

**What stays unverified:**
- **The calendar.** Every date above assumes slots free up as NOTES :965 expects and at most one limit stop; G1's own
  date is unknown, and it decides when robot one's route can be previewed.
- **The no-pace-loop stop** (R3) is designed, not built: the hook's threshold (10,000 lines) is reasoned from C1/C3's
  line counts; RP27b's "the named exception is caught like C1's `bad_alloc`" is reasoned from C1, not run; a loop whose
  lines come from a noexcept caller would terminate instead (RP27b would find it).
- **What ends a frozen 3000 s pause** — the plant's pace cap or the guard's floor (R10) — not run.
- **The robot's behaviour on a no-pace loop** ("sits until the field disables it; its park never runs") — reasoned from
  `run_guard.hpp` :344-351, not run on a robot (nothing here runs on a robot).
- **B1's cost** — `FakesByKind` and stepping `ArmPlant` from a declaration in `TesterStack`'s chain is reasoned from
  `test/mechanism_auton_rig.hpp`, not probed; robot one's list and how its autons are registered depend on G1.
- **B2's cost** (the robot-built world) — not probed.
- **Whether reviewers accept the hook on the host fake sink** — the alternative (a sink-typed `TesterStack`) is bigger.
- **The page's size and how it reads on a phone** (estimated ~0.3 MB); whether CURVE1's or STEP1-B's final shapes need
  more than one appended `StepPhase` (A2 re-reads both).
- **The TOUCH arithmetic of R1** was checked by hand (7.5·(cos 0.1° + sin 0.1°) = 7.513 in), not by a probe; RP29 (a)
  runs it.
