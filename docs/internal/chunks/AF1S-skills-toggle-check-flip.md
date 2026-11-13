# AF1-S — the Toggle flip for a SKILLS run: one route step that looks at a Toggle with the front camera, flips it with our flipper ONLY if it is not already its Quadrant's colour, and looks again after — every doubt does nothing and is reported, the route's onFail decides what comes next; and a MECHANISM COVERAGE CHECK for everything the team may build for the skills run on 2026-11-21

> **THE TEAM LEAD'S ANSWERS (2026-11-10, late evening)** — on the questions put to him from D8, D9 and D11:
> - **The Toggle plan: "Both: blind + checked (Recommended)"** — build the checked step (AF1S-A1, then AF1S-A2 the camera
>   in autons) AND have the team's blind two-push flip ready in the skills route as the 21st's fallback; switch to the
>   checked step only if it has flipped a real Toggle by ~Nov 18. **D11 confirmed:** the flipper acts on the camera's
>   word alone in the checked step (his 11-03 "just the ai vision sensor"); "flipped" means "shows our colour after a full
>   stroke", never "set".
> - **Which robot runs skills: "both robots but they will be structurally similar so building one will be like building
>   the other"** — BOTH robots run the skills route (VEX U, <VURS2>); robot one's build (G1) must make the second robot a
>   near-copy (its own ports and measured numbers only).
> - **D9's order: "No, keep the queue"** — the route-preview brief (RPV1) stays ahead of robot one's brief; robot one's brief
>   follows it.
> - **D9 3 (a bottom limit switch on the cascade):** answered by the team's builder on 2026-11-05 — NO limit switch exists;
>   the cascade homes by stalling gently DOWN into its bottom stop under MECH-S-A's cap (a route's first `startHome`).
> - **Still owed by the builder:** D7 (robot one's flipper: motor arm, pneumatic or passive bar) and D3 (standing at the red
>   station, is the Toggle on the wall to your right the red one, on the event's field?).
> - Every other decision (D1, D2, D4–D6, D10) as recommended after the re-check.

> **Status 2026-11-10 (evening; RE-CHECKED late — 12 findings, see "Re-check — amendments"): DRAFT for the team lead's go — WAITS for his answers to D1–D11 below. Nothing is
> built.** Written by the coordinator's brief writer from a READ-ONLY pass over branch `shulib-v2` at **`d684f34`**
> (main's tip, IMU1-B, committed today). The main worktree's INDEX carries the **L6-A2 squash, staged and
> uncommitted** (`include/shulib/{chassis/robot_context,chassis/wall_reset_call,competition/auton_wall_reset,
> config/robot_config,config/wall_sensor_config,localization/override_walls,localization/wall_reset,
> motion/motion_scheduler,robot/robot}.hpp` and tests): every `file:line` below is **`d684f34`'s** (`git show
> d684f34:<path>`) unless it says **(L6-A2, staged)** — then it is the index's (`git show :<path>`), because AF1-S
> builds on L6-A2's optional-slot pattern. **VIS1-A2 is IN FLIGHT** (worktree `.claude/worktrees/vis1-a2`, base
> `9d77c9d`, uncommitted: `include/shulib/vision/{camera_feed,toggle_tracker,piece_finder}.hpp` NEW,
> `camera_geometry.hpp` changed) — its lines are cited **(VIS1-A2, in flight)** and may move before it merges.
> **STEP1-A is IN FLIGHT** (worktree `step1-a`, uncommitted: `chassis/routine.hpp`'s onFail / label / skipTo) — cited
> **(STEP1-A, in flight)**. In flight and NOT edited (every worktree read only): MECH-S-A (`mech-s-a`), TUNE1-A
> (`tune1-a`, on MECH-S-A's snapshot `2bc30ab`), STEP1-A, L6-A2 (staged on main), CAL1-B (`cal1-b`), VIS1-A2; and
> RPV1 and CURVE1, briefed beside this one (their scripts `.claude/coordinator/drafts/brief-{rpv1,curve1}.js` read;
> no brief of theirs on disk yet).
>
> **THE TEAM LEAD'S RULINGS THIS BRIEF HOLDS (BINDING), 2026-11-10, as given to this writer:**
> - **NO PANEL before the first competition (2026-11-21; code changes stop ~Nov 18; robot one — both robots
>   identical, 15" — built ~Nov 14).**
> - **SKILLS ONLY at this competition** (ALLY1, the alliance colour, after the 21st); **curves kept**; **routes written
>   in code with a host route preview (RPV1, briefed beside you).**
> - Asked **"would it work with just one ai vision sensor in the front?"**, he was told: in a skills run nobody else is
>   on the field, so no Toggle changes unless our robot changes it; the reliable skills version is **"the route drives
>   to each Toggle (a known spot — Toggles are centred on every wall, on top of it:
>   include/shulib/localization/override_walls.hpp :27-31, :86-91), the camera checks its colour, and the robot flips
>   it only if it is not already ours, as a normal route step"**; the full automatic driver-control flipper (the AF1
>   brief, `.claude/coordinator/briefs/AF1-toggle-auto-flipper.md`, answered **"Accept all" 2026-11-03** with the
>   camera-only ruling **"just the ai vision sensor"** — **NO Optical sensor is available**) comes **AFTER the 21st**.
> - **He answered: "yes lets do that. I just need to have all potential mechanisms be able to be built for the skills
>   run on the 21st. lets do it. go full speed ahead".**
> - (Recorded in `.claude/coordinator/NOTES.md` :956-958, 2026-11-10; the 11-03 AF1 answer at :934.)
>
> **One correction to what he was told, found while writing (D3, §0.3):** "nobody else is on the field" holds for an
> OPPONENT, but **a VEX U skills run may use BOTH of our robots** (`<VURS2>`, v20.txt :3556) — so our OTHER robot can
> change or touch a Toggle. The look-first check covers that; nothing else changes.
>
> **Executor model: `opus`** (never `fable` unless the team lead says so). **Do not commit.** Live log
> `docs/internal/chunks/AF1S-PROGRESS.md`, created FIRST (a heading per run). Say "host test plant", never
> "simulator". **No hardware claim anywhere: nothing here has run on a robot.** No Toggle, no flipper, no robot one and
> no camera Toggle reading exist in this project's records: robot one has **no build** (no `RobotId`, no chassis or
> mechanism table — `src/build_selection.hpp` :113-117 lists BenchBot, RobotTwo, InventedXDrive only), robot two has
> **no flipper** (`src/robots/mechanism_table.hpp` :107-133: two claw pivots and a pneumatic claw), and VIS1's Step 0
> (the camera on a real Toggle) has not run. **Hardware Assumptions** (§4): five of AF1's reserved HA-228..HA-236 are
> REUSED (they mean the same thing here), **three new beliefs take HA-243..HA-245** (after ALLY1+L4's reserved
> HA-237..HA-242); a fourth new belief ⇒ stop and tell the coordinator. **Memory rule (2026-10-15):** every binary —
> probes, the suite, any test — runs as `(ulimit -v 4194304; timeout 300 …)`; the suite only through
> `python3 tools/suite_run.py`. **G4 (the project's guardrail):** the team writes the skills route — which Toggles to
> visit, in what order, from where, and what happens when a visit fails; the library provides the step, its checks and
> ONE example (the cookbook recipe).
>
> **The manual.** VEX V5RC *Override* Game Manual **v2.0** (2026-09-03), local text
> `.claude/coordinator/scratch/autoflip/v20.txt`, Appendix A rendered as `scratch/autoflip/img/`. **v2.1 EXISTS
> (2026-10-08, effective 2026-10-15) and was read for this brief on 2026-11-10** (manuals.vex.com/v5/override/v2.1/:
> its change log and Section 3 — earlier attempts got 403/404): v2.1 re-words `<RSC3>` for the new "Pin Half" term and
> **keeps the Toggle rule unchanged in substance** — "for a Pin Placed in a Quadrant Goal, yellow Pin Halves are Owned
> if the Toggle in that Quadrant matches the Quadrant's colour"; **`<SC4>`, `<SG6>`, the VURS rules, the Quadrants and
> Appendix A's Toggle starting orientation have NO v2.1 change** (the starting-orientation drawing dates from v0.2).
> v2.2 is scheduled for 2026-12-03 — after the 21st. **The executor re-checks `<RSC2>`, `<RSC3>`, `<SC4>`, `<SG6>`,
> `<VURS2>`, `<VURS3>`, Figure Q-1 and pA15 against the newest manual before AF1S-A1 writes a rule into a comment,
> and says which version it used.**
>
> **AF1-S PUSHES ON THE CAMERA'S WORD ALONE *(re-check R1)*.** That rests on the team lead's 2026-11-03 camera-only
> ruling ("just the ai vision sensor", `NOTES.md` :934), which RELAXES VIS1's accepted AF1 contract — rules 2 and 3 of
> `docs/internal/chunks/VIS1-ai-vision-colours.md` §6.5 (:1715-1722: "a colour is the face the camera SEES, not a set
> Toggle"; "before any push, the face's colour must be corroborated AT THE PUSH POINT by a sensor that sees colour
> there"). **The amendment that was to record that relaxation (`drafts/brief-af1-camera.js`) is SHELVED until after
> the 21st (`NOTES.md` :958) and has NOT been written** — so D11 asks the team lead to confirm the relaxation for
> AF1-S by name, and the plan commit records it. What it costs is said plainly in D11: the camera cannot see whether a
> Toggle is SEATED.

---

## Re-check — amendments (2026-11-10, late; newest first)

A read-only checker returned 12 findings (3 major, 9 minor). Each was re-verified by this writer: the checker's probes
P5 and P6 were rebuilt and re-run against VIS1-A2's worktree as it stands (outputs byte-identical to the checker's —
`scratch/brief-AF1S/recheck/out_p5.txt`, `out_p6.txt`), and two new probes were written and run
(`recheck/p7_spot_and_stuck.cpp` + `p7b_allowance.cpp` → `out_p7.txt`, `out_p7b.txt`; `recheck/p8_after_rule.cpp` →
`out_p8.txt`; every binary under `ulimit -v 4 GiB` + `timeout 300`). **Eleven are REAL and fixed in the body (each
place marked *(re-check Rn)*); one is PARTLY real** (R11's line numbers were right for the cited file). VIS1-A2 is
still IN FLIGHT (its fourth run): its banner has moved since the first pass — the tracker lines cited in this section
are as of this re-check.

| # | Finding (severity) | Disposition | What changed |
|---|---|---|---|
| **R1** | The camera cannot see whether a Toggle is SEATED: one stuck part-way through its roll reads as the colour it is rolling TO, so the step would say FLIPPED on a Toggle that scores yellow (major) | **REAL.** VIS1-A2's own banner, `toggle_tracker.hpp` :10-18 (re-check): "A Toggle stuck mid-roll shows its DESTINATION face from below and can read as that colour … seating and contact are the consumer's to establish, AT THE PUSH POINT, with a sensor that sees colour there". P5(c) re-run: stuck 20–100° short of seated, read RED "sure" in **58 of 108** cases (lens 8/10/12 up, looked at from 14–24 in); P7(b): at lens 8 and 10 every roll from 50° to 100° reads RED from the look spot, and the band sits AT the spot (the spot check does not catch it). A second look from a different range would not help (P5(c): the same RED from 14 to 24 in at lens 10). The brief never said so, and never said it depends on the shelved camera-only relaxation | Header: the camera-only dependence. **New D11** (the relaxation, and the stuck Toggle in plain words). Rule 6: `Flipped` only after a FULL STROKE (the push SUCCEEDED, or the passive drive reached `pushFrom`); a short stroke that then reads ours → new verdict **`OursAfterShortStroke`** (Unconfirmed). Every `AlreadyOurs` / `Flipped` line ends `seating not checked`. HA-232 extended (a full stroke seats it — the camera cannot check). New test **T14** (the plant's Toggle jammed part-way, after a full and after a short stroke) + mutation MA23. "What it cannot catch", D4, D5, guide 14 |
| **R2** | The "at the spot" check by OVERLAP passes a robot parked ~8–10 in too far (major) | **REAL.** P6 re-run: the tracker's range interval is 5–18 in wide; with `lookFrom` 16, `offBy` 3 an overlap test accepts a TRUE 11–27 in at lens 10 (P7). Containment (the WHOLE interval inside the window) with `offBy` = the interval's larger half-width at the spot + 1.5 in accepts true 14–17 in at lens 10 (look 16), 19–21 at lens 8 (look 20), 12–15 at lens 10 tilted 10° (look 14) — **but 22–33 in at lens 12 (look 24): there the range barely changes with distance** (P7b) | Rule 2: **containment**, not overlap; **no recommended `offBy`** (the team's from Step 0.4: the interval's larger half-width at the spot plus the route's arrival allowance); with a FIELD start, ALSO the pose's own distance to the Toggle's wall. D2, HA-244, T5, MA9 corrected; plain words: **the camera catches only gross arrival errors; the push spot's accuracy is the route's odometry** |
| **R3** | A blocking Chassis verb for the drive in and out starves the feed and tracker; a camera frozen at the look spot returns there and reads the stale colour "sure" (major) | **REAL.** `chassis.hpp` :295-300, :557-566 (`moveTo` → `runBlocking` → `waitUntilSettled`): nothing of the step's runs during it. The tracker's frozen check compares the lens with where the band's box last moved (`toggle_tracker.hpp` :521-534 (re-check)); in-and-out returns to that spot. P5(a) re-run: fed only parked → after = YELLOW "sure" (stale) at lens 10 level, 10 tilted, 12 tilted; fed through the drive → "camera frozen?". P8: fed on EVERY read, a freeze at any of five points (look spot, half-way in, push spot before/after the roll, half-way out) never gives a colour (30 frozen cases, 0 accepted) | Rule 5 spelled out: `motion::MoveToPose` started with `chassis.scheduler().async(m)`, waited through `guard.waitFor` whose predicate ticks the feed and the tracker, cancelled on every exit (as `Chassis::DetachGuard`, :536-555); the tracker fed on EVERY read from the look before to the look after. New mutation **MA24** (the drive through a blocking verb) that T3 must turn red; a §7 grep |
| **R4** | D2 (b) "look and push from one spot is a setting of (a)" contradicts the rules; NO CHANGE is unreachable where the camera names the colour at the push spot (minor) | **REAL.** P5(b) re-run: lens 12 up tilted 10°, push at 10 in: the hold never breaks, so "hold began after the retract" never holds → always UNKNOWN AFTER on a sticky Toggle with a live camera. P8: under that rule NO CHANGE was reachable in 3 of 5 mounts | D2 (b) marked **NOT SUPPORTED** (refused by the request). **Rule 6 re-written** (P8): the after-colour counts from a sure view whose IMAGE is after the retract's end (R8) once the lens has travelled more than `frozenMoveIn` since the retract's end, the tracker fed on every read (R3), the excursion `lookFrom − pushFrom` more than **twice** `frozenMoveIn` (so a camera frozen anywhere in it is moved past the threshold). P8: 60 cases, **0 wrong colours accepted** under the old rule or the new; NO CHANGE reachable **5 of 5** (was 3 of 5). MA6 kept, re-aimed. Landmine 2 corrected |
| **R5** | P3's "0 pushes on ours" holds by construction (it assumes correct colours); a third-colour lookalike while the true face is ours would push it away (minor) | **REAL.** `p3_skills_rule.py` sets seen = truth. VIS1-A2 records 70 of 540 lookalike bands passing with the position gate off (`toggle_tracker.hpp` :75-80 (re-check)) | P3's claim re-worded "the RULE, given correct colours". T1 gains a lookalike scene (true face ours, its band failing a gate, a BLUE lookalike passing in the window) with the honest outcome pinned; R2's containment and the position gate are what refuse it. D6 (c) re-worded: the position gate RECOMMENDED right after a wall reset (a pose the team trusts) |
| **R6** | The wall check's sign: facing the wall is the OPPOSITE of the inward normal (minor) | **REAL.** `kTogglePlaces`' `inX/inY` are "the unit vector INTO the field" (`override_elements.hpp` :64-76) | D6 and rule 1: "heading within ±45° of the direction TOWARD the wall (minus the inward normal)": right wall 0°, blue wall +90°, left wall 180°, red wall −90° (F1, heading 0 along +X, CCW-positive — `pose2d.hpp` :18) |
| **R7** | The retract-now threshold leaves out the drive out: the robot can still touch the Toggle at 0:00 (minor) | **REAL.** `<SC4>`b: a touched Toggle counts yellow; Step 0.5 has not shown the push spot is clear | Retract-now threshold = `backBound + settle + driveBound + margin`; after it fires the step drives out at once; verdict **`RetractedForTime`** (TimedOut). T6 updated (the plant's touch flag clear before the deadline) |
| **R8** | "Hold began after the retract" uses the READ's time; the image is older than its read (minor) | **REAL.** The tracker stamps with `f.readAt` (:515, :530 (re-check)); its banner and HA-192: the image is older by an unmeasured age | Rule 6: a read counts only when `readAt − maxImageAge` is after the retract's end. **`maxImageAge` added to the request** — it was ALSO missing for rule 2's `boundAtImage`; UNSET is refused; Step 0.2 measures it (HA-192's station 12 change timer, no new code) |
| **R9** | A circular include for the Quadrant colours; `FlipperGoal`'s construction; D1's example would be refused (minor) | **REAL.** `toggle_tracker.hpp` includes `override_elements.hpp`; `ToggleColour` lives in the tracker | §1.2: its own data-only `enum class QuadrantColour {Unset, Red, Blue}`, mapped to `vision::ToggleColour` in `toggle_visit.hpp`. `FlipperGoal`: implicit constructors from `mechanism::PositionGoal` and `mechanism::PneumaticGoal` (`kinds.hpp` :447, :476), default-constructed = PASSIVE. D1's example completed (every field) |
| **R10** | The dates assume a free slot and G1 in parallel; all five slots are full, G1 has no brief, MECH-S-A is done (minor) | **REAL.** `NOTES.md` :960-961: RUNNING (5) TUNE1-A, STEP1-A, CAL1-B, this brief, VIS1-A2 (cont3); QUEUE the CURVE1 brief, then the RPV1 brief, then TUNE1-B1 / STEP1-B / L6-B; MECH-S-A DONE, merging after L6-A2. G1 never briefed | Honest dates re-written (§9); MECH-S-A "done, merging after L6-A2" (§0.6, §6.1, D9); D8/D9: **G1's brief before both AF1S runs** and which queued item yields; **the blind flip is THE plan for the 21st**, said plainly |
| **R11** | P1's summary over-generalises the turned cases; line numbers drifted; autons.cpp's forms are comments (minor) | **PARTLY REAL.** P1: the turned rows held only for the lens 10 up level — re-stated per mount (out_p1.txt unchanged). RC18's lines: **:1270, :1841 are right** for `docs/internal/chunks/G3a-robot-profile-generator-panel.md` at `d684f34` (`git show d684f34:… \| grep -n RC18`); the checker's :1245, :1816 are the coordinator's copy `.claude/coordinator/briefs/G3a-…` — both now cited by path. autons.cpp :19-29: the forms are COMMENTS ("No auton here steps a mechanism yet"), tested in `test/mechanism_auton_steps_test.cpp` :86-89 | §0.4 P1 row, D2, landmine 3 per mount; D9 / §0.6 cites by path; §0.6 claws row |
| **R12** | One homing method per declaration; the cheaper skills option (restsAtStop) not offered (minor) | **REAL**, and one better option the checker did not name: **`homeBySwitch` with a bottom limit switch** (the builder's own recommendation, `NOTES.md` :946) zeroes at boot with NO motion when the switch is pressed (`service.hpp` :768-769) AND can re-home on demand by a move | G3 re-written as a three-way choice, one per declaration: `restsAtStop(0_in)` / `homeByStall` / `homeBySwitch` (recommended if the switch is fitted) |

---

## DECISIONS FOR THE TEAM LEAD (a recommendation for each; nothing below decides them silently)

**D1 — The step's shape and name. RECOMMENDED (a).**
- **(a) One library call the route runs as a step: `competition::checkAndFlipToggle(ctx, {...})`**, written in the
  route the way the wall reset already is (L6-A2's form, `competition/auton_wall_reset.hpp` :8-11 (L6-A2, staged)):
  ```cpp
  r.onFail(skipTo("blue wall")).then([&] {
      return competition::checkAndFlipToggle(ctx, {
          .toggle = toggles::kRedWall,        // WHICH Toggle — the team's (G4); "ours" comes from it (D3)
          .flipper = flipper["flip"], .back = flipper["stow"],   // FlipperGoal converts from a preset (R9)
          .lookFrom = 16_in, .offBy = 6_in, .pushFrom = 4_in,    // Step 0.4/0.5 (offBy: R2 — no default)
          .lookWithin = 0.5_s, .flipBound = 0.8_s, .backBound = 0.8_s, .settle = 0.2_s,
          .driveBound = 0.6_s, .margin = 0.3_s, .maxImageAge = 0.1_s,  // Step 0.1/0.2/0.6 — every one UNSET-refused
      }).outcome();
  }, "red wall Toggle");
  ```
  *(re-check R9: the example is now complete — every number in it is a placeholder for Step 0's, none measured.)*
  It looks, decides, pushes and retracts, backs off, looks again, at most twice — all inside ONE step, so every
  safety rule is the library's and the same in every route. It answers a VALUE (§1.1's report: what it saw before and
  after, how many pushes, why it stopped) and `outcome()`: **SUCCEEDED** when the Toggle shows its Quadrant's colour
  at the end (already, or after a FULL stroke — *re-check R1*; "seating not checked" in the line, D11), **CANCELLED** when the period's end cut it, otherwise a failure the route's
  `onFail` decides about (STEP1-A: `carryOn`, `skipTo("x")`, or the default stop). **No change to the frozen Routine**
  (`then()` already takes a `MechanismOutcome` — `routine.hpp` :488-516; probe P4 compiles the form).
- **(b) A new Routine member `r.checkAndFlipToggle(...)`.** **Cost:** a Routine change (F10-additive and pinned the
  day it lands, as L9's steps were), and the Routine would have to reach the camera and the season's Toggles — the
  layering L6 rejected for the same reason (its L6-C, "OPTIONAL, NOT recommended"). Saves the team one lambda line.
- **(c) Only a LOOK call (`competition::lookAtToggle`), and the team writes the flip with today's steps**
  (`r.move(flipper["flip"], …)` …) and an `if`. **Cost:** every safety rule (never push ours, every push paired, the
  time check, at most two pushes, the look after) moves into the students' route, written once per visit. (a) also
  exposes the look call for a team that wants it, so (c) is a subset of (a), never the recommended path.

**D2 — Where the robot stands to look, and how far off it may be. RECOMMENDED (a).**
- **(a) The route parks the robot at a LOOK SPOT square to the Toggle's wall, centred on it, about 14–24 in back
  (lens to face — the team's number from Step 0.4); the step then drives itself straight IN to the push spot, flips,
  and straight back OUT to the look spot to look again.** The camera cannot name the colour close in: on the host
  camera model (probe P1, VIS1-A2's own tracker) a lens 10 in up and level names it from **12 in out** and not nearer
  ("cut by the image" at 4–6 in, "maybe the bottom face" at 8–10 in); 6 in up, from 18 in; 8 in up, from 14 in;
  12 in up, from 24 in; tilted up 10°, from 8–12 in — and **only when the robot is near square**, by mount
  *(re-check R11 — the first pass gave the lens-10 row as if it held for all)*: turned 3°, lens 10 level 14–48 in,
  lens 12 level 24–48, but lens 6 level loses 30 in and lens 8 level loses 18 in; turned 5°, lens 10 level 16–48,
  lens 6 or 8 level only 30/36–48; turned 8°, 30–48 at best (except the 12-in lens, tilted: 10–48 at every turn)
  (VIS1's own warning, `toggle_tracker.hpp` :56-62 (VIS1-A2, in flight, re-check)). A flipper reaches the face from a
  few inches — so look and push are two different spots, and the step owns the short straight move between them.
  **"At the spot" is checked by the camera, coarsely** *(re-check R2)*: the WHOLE range interval of the band the
  tracker names must lie inside `[lookFrom − offBy, lookFrom + offBy]` (containment — never overlap: the interval is
  5–18 in wide, and an overlap test accepted a robot 8–11 in too far, P6/P7), and its bearing interval inside the
  tracker's window; with a FIELD start the pose's own distance to the Toggle's wall must also be within `offBy` of
  where `lookFrom` puts it. Otherwise **NOT AT THE SPOT** — nothing pushed, nothing driven. **`offBy` has no default:**
  Step 0.4 sets it to the larger half-width of the interval the bench page prints AT the look spot, plus the route's
  own arrival allowance. On the host camera model (P7b, +1.5 in allowance) that accepts a true 14–17 in at lens 10
  (look 16), 19–21 at lens 8 (look 20), 12–15 at lens 10 tilted (look 14) — **but 22–33 in at lens 12 (look 24):
  with the lens near the Toggle's height the range hardly changes with distance.** Said plainly: **the camera catches
  only gross arrival errors; where the flipper lands is the route's odometry** (HA-244).
- **(b) Look and push from ONE spot** (no drive inside the step). **NOT SUPPORTED** *(re-check R4 — the first pass
  called it "a setting of (a)", which the rules refuse)*: the look after a push must be made after the lens has moved
  more than `frozenMoveIn` since the retract (rule 6 — the only way to see a frozen camera), so the request refuses an
  excursion `lookFrom − pushFrom` of twice `frozenMoveIn` or less.
- **(c) The step drives to the Toggle from anywhere.** **Cost:** a route-planning feature inside a step (obstacles,
  pieces, the other robot) — the route's job (G4), and RPV1 draws routes, not hidden drives. NOT recommended.

**D3 — What "ours" is in a skills run. RECOMMENDED (a) — and ONE question for the builder.**
- **The rule (v2.0 `<RSC3>`a-iii-1, v20.txt :2047-2049; unchanged in substance in v2.1):** in Skills a Toggle helps
  only when it shows **ITS OWN QUADRANT's colour** — "yellow Pins Placed in a red Quadrant with a Toggle that is set to
  blue are not Owned and will not earn any points". There is no alliance in Skills. **So "ours" is per Toggle: a red
  Quadrant's Toggle wants RED, a blue Quadrant's wants BLUE.** Every Toggle starts **YELLOW** facing the field with
  its Quadrant's colour facing OUT (pA15, `img/a-110.png`: "Red Alliance-side Toggles: Yellow facing up and into the
  field. Red facing out of the field"; the same for blue).
- **Which Toggle is which** (Figure Q-1, `img/gl-122.png`, read 2026-11-10): Red 1 lies against the red alliance's
  wall, Red 2 against the wall on the red station's RIGHT; Blue 1 against the blue alliance's wall, Blue 2 against the
  red station's LEFT. In this library's field frame F1 (`override_walls.hpp` :6-11: +X to the right as seen from the
  red station, the RED wall at y = −70.2): **RED wall → RED, RIGHT wall (x = +70.2) → RED, BLUE wall → BLUE, LEFT
  wall (x = −70.2) → BLUE.** This is a READING of a picture — HA-243, reasoned.
- **(a) The team names WHICH Toggle (`toggles::kRedWall`, `kRightWall`, `kBlueWall`, `kLeftWall`); the library
  looks up its Quadrant colour from one season table** (§1.2, cited to Q-1 and pA15). "Ours" is never typed, so it
  cannot be typed wrong; a route that names the wrong Toggle is the visible mistake, and with a FIELD start the step
  also checks the robot faces that Toggle's wall (D6).
- **(b) The team types the colour per visit** (`.want = Red`). **Cost:** one typo flips a Toggle to the colour that
  scores nothing — and nothing can catch it.
- **The question for the builder (Step 0.7):** on the field you will compete on, standing at the RED station, is the
  Toggle on the wall to your RIGHT the red-Quadrant one (it starts with RED facing out)? One look settles HA-243.
- **What a wrong-coloured Toggle costs in Skills, said plainly:** the Toggle scores only through yellow Pin Halves
  Placed in ITS Quadrant (10 points each, v2.1 `<RSC3>`); a Toggle of the other colour is worth exactly what yellow is
  (nothing) — except in the deepest tie-break (`<T22>` 9-ii, v20.txt :3324: "Toggles set to a color other than
  yellow"). So a flip stopped half-way (yellow → the other colour) **costs nothing** in Skills (probe P3) — unlike a
  head-to-head match, where it would hand the opponent the Quadrant.

**D4 — What happens on UNKNOWN (and every other doubt). RECOMMENDED (a).**
- **(a) Nothing is pushed and the report says why; the route's `onFail` decides what comes next** (the team's choice:
  `carryOn` to the next goal, `skipTo` a later re-visit, or the default stop). The doubts that do nothing: the colour
  UNKNOWN with the tracker's reason (no read, camera silent, read not usable, camera full, no band, cut by the image,
  maybe the bottom face, two colours, not held yet, camera frozen?, another colour beside or above — `toggle_tracker.hpp`
  :124-140 (VIS1-A2, in flight)); the robot not at the spot; the wrong wall (FIELD start); not enough time left; no
  camera on this robot; not a Skills auton; the flipper not homed or absent; a stroke that stopped short before the
  colour turned ours (*re-check R1* — the Toggle may be stuck part-way). With no `onFail` written, a failed visit
  stops the route as any failed step does today (`routine.hpp` :111-131). **What "does nothing on a doubt" cannot
  cover (D11):** a Toggle stuck part-way through its roll that reads as its destination colour is NOT a doubt to the
  camera — it reads "sure".
- **(b) Push anyway, since every Toggle starts yellow.** **Cost:** breaks the library's positive-evidence rule, and
  our other robot (D3) may already have set it — a blind push would roll it AWAY from ours. NOT recommended. (A team
  that wants a blind flip writes it with today's steps — the fallback in D10.)
- **(c) Wait longer for a sure colour.** Already inside (a): the look waits up to `lookWithin` (the team's number) for
  a sure colour before it says UNKNOWN.

**D5 — Whether it re-checks after the flip, and retries. RECOMMENDED (a).**
- **(a) Always look again after every push (backing out to the look spot to do it); a SECOND push only when the first
  changed the face to the THIRD colour (neither the start nor ours); never a third push; no automatic retry.** This is
  AF1's rule 5 (AF1 brief §1.1, amended re-check #2) kept word for word, with "the target" = the Toggle's Quadrant
  colour. Probe P3 — **the RULE, given correct colours** *(re-check R5: P3 feeds the true face as the seen colour, so
  "no push on ours" holds there by construction; a lookalike can still mislead the camera — T1's lookalike scene)* —
  (84 skills cases — both roll orders × both Quadrant colours × every start × seven troubles): **no push
  ever made while the true face was ours, never a third push, no Toggle owned before and not after.** And FLIPPED is
  said only after a FULL stroke, with "seating not checked" *(re-check R1, D11)*. From yellow:
  FLIPPED in every no-trouble case; a sticky Toggle → NO CHANGE (one push, stop); a jam on push 2 → STILL NOT OURS;
  UNKNOWN or a frozen camera after push 1 → UNKNOWN AFTER (stop). A visit that ends short of ours fails, and the
  route's `onFail` may send the robot back later (a second call is a fresh visit).
- **(b) Push the predicted number of times, then look once.** Needs the roll direction MEASURED (Step 0.1 — AF-H4,
  HA-231): on the research's inference **every skills Toggle needs TWO pushes from yellow** (the Quadrant colour is
  the hidden outward face: yellow → the other colour → ours; probe P3's "pushes needed": 2 on the research's order, 1
  on the other). **Gain:** about one back-out and one look per Toggle (~1–1½ s, guessed). **Cost:** if the
  first push over-rolls (two faces), the second rolls it AWAY from ours — the very case (a)'s look catches. Offer it
  AFTER Step 0 shows one face per stroke, every time — not by default.
- **(c) Retry a failed visit automatically.** **Cost:** hidden time in a 60-s run and a sticky Toggle pushed again
  and again. The route can call the step twice; the library will not decide that.

**D6 — Does it need the robot's FIELD position (a field start), or does it work from a RELATIVE start?
RECOMMENDED (a).**
- **(a) It works from BOTH; a FIELD start adds one check.** The look-spot check is the camera's own range and bearing
  (D2), the in-and-out drive is along the robot's own heading, and the time check is the run guard's — none needs the
  field frame. With a **FIELD start** (`competition::StartPose::field(...)`, `auton_registry.hpp` ~:171) the step ALSO
  refuses a robot that does not face the named Toggle's wall (heading within ±45° of the direction TOWARD the wall —
  minus `kTogglePlaces`' inward normal *(re-check R6: the first pass said "the inward normal", the wrong sign)*: right
  wall 0°, blue wall +90°, left wall 180°, red wall −90°, in F1 with heading 0 along +X, CCW-positive, `pose2d.hpp`
  :18 — the four walls are 90° apart) — **WRONG WALL**, nothing pushed — the one check that catches a route naming the wrong
  Toggle. With a RELATIVE start the line says `wall not checked (relative start)`. **Recommendation for the skills
  route itself: a FIELD start** — a skills run starts at a known place (`<RSC2>`a / `<VURS3>`, v20.txt :2021-2022,
  :3560-3562: the red Quadrant start), so a FIELD start costs the team one measured pose, and it also lets L6-A2's wall
  reset work in the same route (it refuses a RELATIVE start — `auton_wall_reset.hpp` :20-26 (L6-A2, staged)).
- **(b) FIELD starts only.** **Cost:** refuses every route written like today's autons (`src/team/autons.cpp` uses a
  RELATIVE start, :80).
- **(c) Also use the tracker's POSITION gate** (`toggleIndex`, `toggle_tracker.hpp` :391-397 (VIS1-A2, in flight)) to
  prove WHICH Toggle the band is. Needs a pose the team trusts to ~a few inches (robot two's drift is unmeasured — L6's
  reason to exist); left to the tracker's own config (`positionToleranceIn` 0 = off), not required by the step.
  *(re-check R5)* **Recommended where the route has just run L6-A2's wall reset** (a pose the team trusts): it is the
  gate that turns lookalikes away (VIS1's P4b: 0 of 540 passed with it, 70 without).

**D7 — Which flippers the step drives. RECOMMENDED (a).**
- **(a) A MOTOR flipper (an `Arm` or a `Lift` — two presets, "flip" and "stow"), a PNEUMATIC one (two named
  positions), and a PASSIVE one (no mechanism: the in-and-out drive itself is the stroke).** The first is the team
  lead's 2026-10-08 description (a motor arm); the builder's note of 2026-11-05 says "**passive toggle mechanical**"
  (`NOTES.md` :946) — which may mean robot one's flipper is a fixed bar. One step serves all three: the motor and
  pneumatic forms command the flipper through the robot's mechanism service, as `r.move` / `r.set` do; the passive
  form drives in to `pushFrom` (where the bar has rolled the Toggle) and out. **Cost over motor-only:** ~¼ day (two
  more overloads, their tests).
- **(b) Motor arm only** (AF1's v1). **Cost:** if robot one's flipper is passive or pneumatic, the step cannot serve
  it before the 21st.
- **The question for the builder (Step 0.1):** what is robot one's Toggle flipper — a motor arm, a pneumatic, or a
  passive bar? Its answer picks which form the route uses; the library is ready for all three.

**D8 — The runs, and what is cut first. RECOMMENDED (a).**
- **(a) Two library runs and the team's route:**
  - **AF1S-A1 — the step, over EXPLICIT handles** (~2¾–3¼ builder-days — *re-check: +¼*): the decision and its report, the in-and-out
    drive, the time check, the three flipper forms, the season's Quadrant table, the host Toggle model coupled to the
    host camera model and the mechanism plant, the adversarial tests. **Starts on your go and a free slot — after G1's brief (R10)**, on VIS1-A2's build
    snapshot if VIS1-A2 has not merged (TUNE1-A's precedent: a snapshot base, re-checked at merge).
  - **AF1S-A2 — the robot's camera answer reachable from an auton** (~1¼–1½): the robot file's Toggle-tracker block,
    the Robot building the camera feed and the tracker, the auton context's second OPTIONAL slot (L6-A2's pattern),
    the everyday `checkAndFlipToggle(ctx, …)`, the cookbook recipe. **Starts when L6-A2 and VIS1-A2 have merged**
    (it edits the same four files L6-A2 did — §6.2).
  - **Robot one's own entries** (the camera block, the tracker numbers, the flipper declaration) go in robot one's
    hand-written build (the coverage check's G1 — its own chunk) from Step 0's numbers; **the skills route is the
    team's** (an evening, from the recipe).
  - *(re-check R10)* **G1 has no brief yet, and its brief comes BEFORE both AF1S runs** in the queue (G1 blocks every
    mechanism on robot one; AF1S blocks only the checked flip). All five slots are full tonight and the queue is the
    CURVE1 brief, then the RPV1 brief (`NOTES.md` :960-961) — **recommended: the G1 brief takes the slot after the
    CURVE1 brief, so the RPV1 brief moves one place later**; AF1S-A1 after it. Your call.
  - *(re-check R10)* **For the 21st the PLAN is the blind flip** (the fallback below, written by the team into the skills
    route with today's steps). The camera-checked step is unlikely to have flipped a real Toggle on robot one before
    the ~Nov 18 freeze (§9's dates); it is switched in only if it has.
- **(b) One run.** **Cost:** nothing reviewable until A2's dependencies land; one slip loses all of it.
- **Cut in this order if it slips:** (1) D5 (b)'s option; (2) the pneumatic and passive forms if Step 0.1 shows a
  motor arm (or the motor form if it shows a bar); (3) AF1S-A2's recipe page (the example test stays); (4) AF1S
  itself — **the fallback for Nov 21 needs NO new code**: the team writes the flip as today's steps (`r.move(flipper
  ["flip"], …)` and `r.move(flipper["stow"], …)`, twice per Toggle if Step 0.1 measures two faces from yellow), with
  NO camera check — honest only because in Skills every Toggle starts yellow and only our robots touch it. **Never
  cut:** never a push on ours or on UNKNOWN, every push paired, the time check, at most two pushes, the look after.

**D9 — The mechanism gaps' fixes and their order** (the coverage check, §0.6 — every gap with its smallest fix).
**RECOMMENDED (a).**
- **(a) In this order:** **G1 robot one's build plumbing FIRST** (a new robot id, chassis table, mechanism table,
  driver file, autons — hand-written; RC18's "its own chunk, the coordinator's", `docs/internal/chunks/G3a-robot-
  profile-generator-panel.md` :1270, :1841 at `d684f34` — the coordinator's copy `.claude/coordinator/briefs/G3a-…`
  has the same words at :1245, :1816 *(re-check R11)*) — without it NOTHING
  of robot one, the cascade included, can be built, driven, tuned (TUNE1) or tested at all; **then the cascade's
  declaration from Step 0's facts** (no new library code — G2–G4 below are answered by words that exist: two or more
  motors per lift, a homing word chosen from three (G3, *re-check R12*), a two-point `measured({...})` shape for a
  cascade's multiplied travel, constants `lift.to(h)` beyond six named levels); **then AF1S**; **G8 (more than six
  NAMED levels) only if the team asks for more than six buttons' worth.** **MECH-S-A (the current cap) is DONE and
  merges after L6-A2** *(re-check R10 — the first pass said "building")*; TUNE1-A (live tuning) is building.
- **(b) AF1S before G1.** **Cost:** a finished step with no robot to run it on.
- **What I need from you:** (1) which robot runs the skills route — robot one (a new build), robot two rebuilt, or
  BOTH (`<VURS2>` allows both in a VEX U skills run); (2) a go for G1's BRIEF (it has none — *re-check R10*) in the
  next free slot after the CURVE1 brief, ahead of the RPV1 brief; (3) whether robot one's cascade has a bottom limit
  switch (the builder recommended one, 11-05) — it picks the cascade's homing word (G3).

**D10 — The register numbers. RECOMMENDED (a).**
- **(a) Reuse AF1's HA-231, HA-232, HA-234, HA-235, HA-236** (AF-H4 roll direction, AF-H5 seat time, AF-H7 a disable
  does not retract, AF-H8 a stall bounds a push, AF-H9 the flipper's facts — the same beliefs), leave **HA-228..230
  and HA-233** (the Optical and the field timer) to AF1 after the 21st, and take **HA-243..HA-245** for three new
  beliefs (§4). The register's Held line (`docs/hardware-assumptions.md` :39 at `d684f34`) still ends at HA-227: the
  plan commit owes AF1's HA-228..236 and ALLY1+L4's HA-237..242 to it as well. If RPV1's or CURVE1's brief reserves
  numbers first, these shift by a mechanical rename (labels AFS-H1..H3 kept).

**D11 — Pushing on the camera's word alone, and what that cannot see: a Toggle stuck part-way. *(NEW, re-check R1.)*
RECOMMENDED (a).**
- **What you ruled on 11-03** ("just the ai vision sensor") lets the flipper act on the camera's colour alone. The
  camera chapter's rules said a colour must be confirmed AT THE PUSH POINT by a sensor that sees colour there
  (VIS1 §6.5 rules 2–3) — the write-up of your relaxation was shelved until after the 21st, so it needs your word
  again, for this step by name.
- **What the camera cannot see, said plainly:** whether the Toggle has SETTLED flat. A Toggle that sticks part-way
  through its roll shows the face it is rolling TO, and the camera reads that colour "sure" (the host camera model:
  58 of 108 stuck positions read as the destination colour, from every look spot tried). The game counts that Toggle
  as yellow (`<SC4>`a). So "FLIPPED" means "it shows our colour after a full stroke", never "it is set".
- **(a) Accept it, and make the report honest:** FLIPPED only after a FULL stroke (the flipper reached its flip
  position, or a passive robot reached the push spot) — a stroke that stopped short and then reads ours is reported
  as `OursAfterShortStroke` and FAILS (the route's onFail decides); every success line ends `seating not checked`;
  Step 0.1 checks on a real Toggle that one full stroke always seats it (HA-232). **Cost:** ~⅛ day inside A1.
- **(b) Look a second time from a different distance.** Rejected: the stuck Toggle read the same wrong colour from
  14 to 24 in (P5(c), P7(b)) — it would not catch it.
- **(c) A colour sensor at the push point.** None is available (your ruling); revisit with AF1 after the 21st.

---

## The design in plain words (for the team lead)

- **What the route says.** "Go to the look spot in front of the red-wall Toggle, square to the wall; check and flip
  it; if that fails, skip to the blue wall." One step, one line of log, one answer.
- **What the robot does in that step, in order.**
  1. **Is this allowed?** A Skills auton (in a match auton "ours" is the alliance colour — ALLY1, after the 21st —
     so it refuses); a camera on this robot; the flipper present and homed; with a field start, facing the named
     Toggle's wall. Any no → it does nothing and says which.
  2. **Look.** Up to `lookWithin` (the team's number, about half a second) for the camera to name the colour, held
     steady. UNKNOWN → nothing, and the reason. The band must be where the Toggle should be: at the look distance
     within the team's tolerance, straight ahead — else "not at the spot" *(re-check R2: a coarse check — it catches a
     robot parked well off, not a couple of inches; where the flipper lands is the route's driving)*.
  3. **Already ours?** Red-Quadrant Toggle showing RED, or blue showing BLUE → **done, no push** ("already ours").
  4. **Enough time?** The run guard knows how long the 60 s run has left. The step pushes only if the time left covers
     everything this visit may still need — drive in, push, retract, settle, drive out, look, for every push it may
     still make, plus a margin — using the team's measured upper limits, not typical times. Else nothing ("not now —
     3.1 s left, needs 6.4 s").
  5. **Flip.** Drive straight in to the push spot, push, retract (EVERY push is followed by its retract, whatever
     the push's verdict — a stall, a timeout), drive straight back out. A passive flipper: the drive in and out IS the
     push.
  6. **Look again** — the camera watched on every frame the whole way in and out *(re-check R3)*, counting only
     pictures taken AFTER the retract, once the robot has backed off far enough to show a frozen camera
     *(re-check R4, R8)*. Ours after a full stroke → **FLIPPED** (with "seating not checked" — D11); ours after a
     stroke that stopped short → fails ("maybe stuck part-way"). The same colour as before → **no change** (a sticky Toggle, or a camera that froze) — stop.
     The third colour → one more push (if the time still covers it), then a last look. **Never a third push.**
- **What it never does.** It never pushes a Toggle that reads ours. It never pushes on a guess. It never leaves the
  flipper out on purpose. It never starts a push that could still be out when the run ends. It never drives anywhere
  but straight in to the push spot and back to where the route left it. It never decides what the route does next.
- **What it cannot catch — said plainly.**
  - ***(re-check R1)* Whether the Toggle is SEATED.** A Toggle stuck part-way through its roll shows the colour it is
    rolling to and reads "sure" (D11); the game counts it yellow. The step says FLIPPED only after a full stroke, and
    every success line says `seating not checked`. The same stuck Toggle can read "already ours" before a push.
  - **A camera that freezes while the robot stands still looks exactly like a still scene** (probe P2: frozen while
    parked, the Toggle flipped under it, the tracker still said the OLD colour, "sure"). That is why the camera is
    read on EVERY frame through the drive in and out (*re-check R3*: fed only while parked, a camera frozen at the look
    spot came back to the same spot and still read stale "sure" — P5(a)), and the look after counts only pictures
    taken after the retract once the robot has backed off past the frozen check's distance: moving, the tracker
    catches the frozen camera (P2; P8: a freeze at any of five points, 0 colours accepted). The cost: a frozen camera
    after a REAL flip reads as "unknown after" — the visit fails although the Toggle is right. That is the safe
    direction. **The look BEFORE is parked:** a camera that froze on the approach with this Toggle in view shows the
    colour of a moment earlier — harmless in Skills unless our other robot changed it in that moment.
  - **Something else of a Toggle colour where the face should be** (a red or blue piece held high, our other robot's
    plate) can read as a face; the tracker's gates turn most away (VIS1-A2's 70 of 540 lookalike bands still passed
    with the position gate off — `toggle_tracker.hpp` :75-80 (re-check)). In Skills no opponent is there; our other
    robot is. *(re-check R5)* If the real face's band is turned away and a lookalike of the THIRD colour passes, the
    step pushes a Toggle that was already ours (T1 pins it); the spot check (R2) and the position gate after a wall
    reset (D6 (c)) are what refuse it.
  - **It cannot see that the robot is touching the Toggle** (then the Toggle counts yellow, `<SC4>`b). The step backs
    out after every push; whether the robot's frame touches the Toggle at the push spot is Step 0.5's question.
  - **It cannot count the Pins and Cups the robot holds** (`<SG6>`: holding more than one Pin or one Cup, a robot may
    not interact with Toggles). The route must not visit a Toggle then — the recipe says so.
  - **It knows nothing that is not measured.** Every number is the team's from Step 0; with one unset the step does
    nothing and names it.
- **How long it takes — GUESSES, nothing measured.** On the host test plant's slow test arm (40 rpm at the joint) a
  push and its retract took 0.88 s + 1.45 s (AF1's probe P4). With a quicker flipper: drive in ~0.5 s, push and
  retract ~1–1½ s, settle ~0.2 s, back out ~0.5 s, look ~0.3–0.5 s → **~3 s for a Toggle that needs one push, ~5–6 s
  for one that needs two** — and on the research's guess of the roll direction every skills Toggle needs TWO from
  yellow. Four Toggles could cost ~20–25 s of a 60-s run. **Which Toggles are worth a visit is the team's strategy**:
  a Toggle helps only if yellow Pins end up in that Quadrant's Goals.
- **What changes for the drivers:** nothing. This is auton-only. The driver-control automatic flipper (AF1) comes
  after the 21st and will reuse this step's rules.

---

## 0. Why — what exists, what the rules give, and the probes

### 0.1 Why this chunk, and why now
- The team lead's 2026-11-10 rulings (header): skills only; the camera-checked flip as a route step for the 21st; the
  full driver-control flipper after. A Toggle is the only way a yellow Pin Half scores in a Quadrant Goal in Skills
  (v2.1 `<RSC3>`; v2.0 :2045-2049), at 10 points each — twice a red or blue half.
- Nothing in the library does it today: the Routine can move a flipper (`r.move`, `routine.hpp` :568) but cannot ask
  the camera anything, and no auton can reach the camera at all (`AutonContext`, `auton_registry.hpp` :215-237, has no
  camera; `Robot::camera()` exists, `robot/robot.hpp` :754-768, but an auton never sees the Robot).
- "All potential mechanisms buildable for the 21st" (his words) needs the coverage check of §0.6 — and it finds that
  the largest gap is not a mechanism kind at all: **robot one has no build** (G1).

### 0.2 What exists in the tree (every line OPENED; main = `d684f34`)
| Piece | Where (opened) | What it gives AF1-S / what it lacks |
|---|---|---|
| The Routine | `chassis/routine.hpp` :111-131 (the error policy: a failed step stops the chain, safes the drive), :140-152 (`then()` the seam), :257 (`class Routine`), :488-516 (`then()`: void / bool / ExitReason / `MechanismOutcome` — only `Succeeded` continues), :554-558 (THE HONEST LIMIT: the service ticks only inside a shulib motion, wait or pause), :568-697 (the mechanism steps) | The step rides `then()` with a `MechanismOutcome` (D1 (a)); no Routine change |
| STEP1-A (in flight) | `step1-a` worktree, `routine.hpp`: the on-failure choice banner (+:153-197 of its diff), `OnFail{Stop, CarryOn, SkipTo}`, `carryOn`, `skipTo("x")`, `label("x")`; **the cut rule** — a CANCELLED verdict (a `then()` returning `MechanismOutcome::Cancelled` included) or an expired guard STOPS the chain whatever the choice; the context constructor borrows `ctx.guard` | The route's `onFail` decides after a failed visit (D4); `outcome() == Cancelled` at the period's end stops the route (D1) |
| The run guard | `sequence/run_guard.hpp` :408-410 (`remaining()`), :436-448 (`waitFor`: deadline-aware, RunExpired wins a tie, the active motion keeps ticking — over the scheduler's `waitUntil`, so the pacer chain, the mechanism service included, ticks) | The time check (no period clock needed in an auton — AF1's `PeriodClock` is NOT needed); every wait inside the step goes through it, so a deadline cuts the step at once |
| The auton's context | `competition/auton_registry.hpp` :110-114 (`AutonKind {Unset, Match, Skills}`), :130-150 (`AutonPeriods`: the skills period is the PROGRAM's constant), :158-183 (`StartPose`: field / relative), :215-237 (`AutonContext`: chassis, guard, telemetry, mechanisms, entry, `mechanismService`) | `ctx.entry.kind` (refuse a Match auton), `ctx.entry.effectiveStart()` (the wall check), `ctx.guard`, `ctx.mechanismService` |
| L6-A2's everyday call (staged) | `competition/auton_wall_reset.hpp` :1-48 (the banner: a value report, `continues()`, the guard's wait, one line), :105-129 (the call); `chassis/robot_context.hpp` :17-41, :64-68, :126-128 (the FIRST optional slot, a forward-declared pointer); `motion/motion_scheduler.hpp` :668 (the scheduler's SHADOW context carries the slot — one line); `robot/robot.hpp` :626 (built last), :1256-1320 | **The template AF1S-A2 copies** (a second optional slot, `toggleCamera`, the same three places) |
| The mechanism service | `mechanism/service.hpp` :169-180 (`StepReport`), :727 (`PositionSlot`), :779 (`moveTo`), :826 (`moveToPreset`), :937-944 (`homed()`, `position()`), :1439 (`PneumaticSlot`), :1454 (`set`), :1571 (`MechanismService : motion::ITickPacer`), :1640 (`pace()`), :1783-1810 (`find(PositionGoal)`, `find(PneumaticGoal)`) | The push and the retract are two commands to one slot and their verdicts, exactly as `r.move` / `r.set` do (`routine.hpp` :1022-1034); nothing added to `service.hpp` (STEP1-B and MECH-S edit it — §6) |
| The camera feed (in flight) | `vision/camera_feed.hpp` (VIS1-A2, whole file, 125 lines): one read per period for every consumer, `tick()`, `latest()` (a fault frame included), `minReadPeriodSeconds` the team's, UNSET refused | The step ticks the feed inside its own waits; nobody else reads the camera in an auton |
| The Toggle tracker (in flight) | `vision/toggle_tracker.hpp` (VIS1-A2): :1-75 (the banner: a colour only when sure; the gates; **"Parked — exactly where a consumer checks a push — nothing can tell a frozen camera from a still scene"** :47-52; the lookalikes :62-67), :105-110 (`ToggleColour`), :124-140 (`ToggleWhy`), :176-207 (`ToggleTrackerConfig`, every number the team's), :385-398 (`ToggleView`: colour, why, heldSeconds, bearing, range, positionChecked, toggleIndex), :407 (`update(frame, PoseBound)`), :546 (`view(now)`) | **What AF1-S depends on:** `ToggleColour`, `ToggleWhy` and its words, `ToggleView`'s colour / why / heldSeconds / bearing / range, `update` and `view`, `ToggleTrackerConfig` and its `…Problem`; `CameraFeed`; `vision::CameraModel` and `PoseBound` (`camera_geometry.hpp` :113-125, VIS1-A1, landed). **NOT** `piece_finder.hpp` |
| The camera as a device | `robot/robot.hpp` :128-140 (built last, a refused camera never refuses the robot), :754-768 (`camera()`, `cameraAbsence()`); `config/camera_config.hpp` :38-48 (`CameraPlacement`, measured and its tolerances), :92-96 (`CameraConfig`) | AF1S-A2 builds the feed and tracker beside it |
| The season's Toggles | `localization/override_walls.hpp` :6-11 (F1), :27-33 (the Toggle on every wall, centred, on top of it), :86-91 (its body and half length), :160-176 (the four walls); `vision/override_elements.hpp` :35-62 (the face's geometry, HA-201), :64-76 (`kTogglePlaces[4]`: right, left, blue, red — NO Quadrant colour) | §1.2 adds the Quadrant colour of each, beside `kTogglePlaces` |
| The mechanism words | `mechanism/kinds.hpp` :717-806 (`Arm`, presets `arm["flip"]` checked at build), :811-914 (`Lift`), :1075-1170 (`Pneumatic`, `claw["open"]`) | The flipper is any of them — no new kind (D7) |
| The host plants | `sim/camera_plant.hpp` (VIS1-A1, landed: `addToggle` with a field face and a bottom face, `addRolledToggle`, `CameraState::{Normal, Unplugged, Frozen, Torn}`), `sim/mechanism_plant.hpp` (`ArmPlant`), `sim/field_plant.hpp` (L6-A1) | AF1S-A1's `sim/toggle_plant.hpp` couples them (§1.4) — AF1's planned plant, re-pointed from an Optical to the camera |
| The team's autons | `src/team/autons.cpp` :1-34 (who runs them; the mechanisms by declaration), :62-66 (the 40-in auton: `Routine r{ctx, …}`), :80 (`StartPose::relative()`) | The recipe's route lives in a file like this (G4) |

### 0.3 What the rules give (v2.0 read, v2.1's changes read 2026-11-10 — header)
- **Skills target per Toggle = its Quadrant's colour** (`<RSC3>`a-iii-1, v20.txt :2047-2049; v2.1 the same in "Pin
  Half" words). Quadrants: Figure Q-1 (`img/gl-122.png`), each a triangle against one wall, holding that wall's Toggle
  (glossary, :4584-4586: "Each Quadrant contains two Goals and one Toggle"). **Start state:** yellow into the field,
  the Quadrant's colour out (pA15, `img/a-110.png`).
- **Who may touch which, when:** in Skills "Robots may move freely about the Field after the start of the Match"
  (`<RSC2>`f, :2028) — all four Toggles; **both our robots may play** (`<VURS2>`, :3556-3557); the robot starts in the
  red Quadrant (`<RSC2>`a, `<VURS3>` :3560-3562); holding more than one Pin or one Cup, no Toggle (`<SG6>`, :1169-1175: a robot holding more than one Pin or one Cup "cannot interact with Toggles");
  push only (`<GG9>`); no damage (`<R18>`a).
- **When it counts:** at the end of the Skills match (`<SC1>`'s timing applies — `<RSC1>`, :2007-2008 "standard
  rules apply"); **set** = fully seated AND not touched by a robot (`<SC4>`, :949-956; no v2.1 change). So the flip
  must be finished, the flipper back and the robot off the Toggle before 0:00.
- **One minute** (Section 3 overview, :1994-1996). The program's skills period is `AutonPeriods::skills`.
- **The roll direction is UNVERIFIED** (AF1's research, `scratch/autoflip/design.md` §A2: a push from the field side
  rolls the top away; red-side Y→B→R, blue-side Y→R→B — so from yellow **every** skills Toggle needs two pushes,
  passing through the other colour). The rule does not depend on it (P3); the time each visit costs does (AF-H4,
  HA-231).

### 0.4 The probes (`.claude/coordinator/scratch/brief-AF1S/`, run by this writer 2026-11-10; every binary under `ulimit -v 4 GiB` + `timeout 300`; C++ built against `.claude/worktrees/vis1-a2/include` and its `test/camera_scene_rig.hpp` — VIS1-A2's own rig, IN FLIGHT)
| Probe | What | Result |
|---|---|---|
| **P1** `p1_look_spot.cpp` → `out_p1.txt` | VIS1-A2's tracker on the host camera model: a seated RED-faced Toggle with a YELLOW bottom face, the TRUE camera moved inside the stated tolerances (the rig's), 0.6 s of reads at lens-to-face 4–48 in; lens 6/8/10/12 in up × level / 10° up × turned 0/3/5/8/12° | **Never a wrong colour.** Square, the colour is named from: 6 in up level **18 in**; 8 in up **14**; 10 in up **12**; 12 in up **24**; tilted 10° up 8–12 in. Nearer: "cut by the image" (4–6 in) or "maybe the bottom face" (8–10 in, lens 10). **Turned 5°**: 16–36 in; **turned 8°**: 30–48 in (except the 12-in lens, which holds from 10–14 in when turned — the bottom face leaves its view). *(re-check R11: the turned figures above are the lens-10-level
row only. Per mount, turned 3°: lens 6 level 18–24 + 36–48 (30 lost), lens 8 level 14–16 + 20–48 (18 lost), lens 10
level 14–48, lens 12 level 24–48; turned 5°: lens 6/8 level 30/36–48 only, lens 10 level 16–48, lens 10 tilted 18 +
24–48; turned 8°: 30–48 at best, lens 12 tilted 10–48.)* ⇒ D2: look and push are two spots; the look spot must be square. NO WALL in the model (it hides the bottom face from a low camera on a real field — `camera_plant.hpp`'s NOT MODELLED) |
| **P2** `p2_frozen_and_pairs.cpp` → `out_p2.txt` | (a) every field/bottom colour pair, lens 10 in level, 12–36 in, turned 0/2/4°; (b) a camera that FREEZES while parked, the Toggle flipped yellow → blue under it | (a) **126 cells, 114 sure, 0 wrong.** (b) parked 16 in live: YELLOW sure; **frozen, parked, Toggle now BLUE: YELLOW "sure"** (the stale frame); **after backing off 4 in: UNKNOWN "camera frozen?"**; recovered, at 20 in: BLUE sure ⇒ the look after a flip must be made MOVING and count only reads after the retract (§1.1 rule 6) |
| **P3** `p3_skills_rule.py` → `out_p3.txt` | the check-and-flip rule with "ours" = the Quadrant colour: both roll orders × both Quadrant colours × start Y/R/B × none, sticky, over-roll, jam on push 2, UNKNOWN after push 1, frozen after, no time for push 2 | **84 cases: 0 pushes on a true face of ours, 0 third pushes, 0 Toggles owned before and not after** — *the RULE given correct colours (re-check R5: the probe sees the true face, so the first holds by construction; lookalikes are T1's)*. From yellow: no trouble FLIPPED ×4; sticky NO CHANGE ×4; over-roll FLIPPED ×4; jam on 2 FLIPPED ×2 / STILL NOT OURS ×2; unknown after 1 → UNKNOWN AFTER ×4; frozen → UNKNOWN AFTER ×4; no time → FLIPPED ×2 / NO TIME FOR PUSH 2 ×2. Pushes needed from yellow: 1 or 2 by the order — **2 for every Toggle on the research's order** |
| **P4** `p4_then_forms.cpp` → `out_p4.txt` | the route's forms against main's `routine.hpp` | a report with an explicit `bool` and a `MechanismOutcome` accessor: **both pass `then()`** (compiles, g++ 13, `-std=c++20`) |
| **P5** (the checker's, re-run identical) `check/p5_step_geometry.cpp` → `recheck/out_p5.txt` | (a) a camera frozen at the look spot, the Toggle flipped, the robot in and back out — fed only parked vs on every read; (b) a sticky Toggle, a live camera: does a new hold begin after the retract?; (c) a Toggle stuck 0–120° through its roll toward RED, read from 14–24 in | (a) fed only parked: stale YELLOW **"sure"** at three mounts; fed through the drive: "camera frozen?" (R3). (b) lens 12 up tilted, push at 10 in: the hold never breaks (R4). (c) stuck 20–100°: RED "sure" in **58 of 108** (R1) |
| **P6** (the checker's, re-run identical) `check/p6_range.cpp` → `recheck/out_p6.txt` | the range interval's width at the look spot | 5–18 in wide (lens 10 at 16 in: [11.7, 19.6]) (R2) |
| **P7 / P7b** (re-check) `recheck/p7_spot_and_stuck.cpp`, `p7b_allowance.cpp` → `out_p7.txt`, `out_p7b.txt` | the at-the-spot rule: overlap ±3 vs containment vs midpoint, true distance 8–34 in; a stuck Toggle's interval | overlap ±3 at lens 10 (look 16) accepts true **11–27 in**; containment (+1.5 in allowance) **14–17**; lens 8 (look 20) 19–21; lens 10 tilted (look 14) 12–15; **lens 12 (look 24) 22–33** — the range is ill-conditioned there. A stuck Toggle's band sits AT the spot at lens 8 and 10 (R1, R2) |
| **P8** (re-check) `recheck/p8_after_rule.cpp` → `out_p8.txt` | rule 6 old ("hold began after the retract") vs new (a picture after the retract, the lens moved more than `frozenMoveIn` since, fed on every read): 5 mounts × flipped/sticky × a freeze at none or one of five points | **60 cases: 0 wrong colours accepted by either rule**; a sticky Toggle with a live camera reaches NO CHANGE **5 of 5** under the new rule, 3 of 5 under the old (R4) |

### 0.5 Found while reading (each handled in scope or named)
- **"Nobody else is on the field" — except our other robot** (`<VURS2>`): handled by looking first (D3, D4).
- **The look and the push cannot be one spot** for most camera mounts (P1): handled by the step's own in-and-out
  drive (D2).
- **A frozen camera is invisible parked** (P2, and VIS1-A2's own banner): handled by rule 6 (the look after is made
  moving, after the retract).
- **No auton can reach the camera** (`AutonContext`, `RobotContext`): AF1S-A2's optional slot.
- **`kTogglePlaces` has no Quadrant colour**: §1.2.
- **A Match-kind auton has no "ours" until ALLY1**: the step refuses `NOT SKILLS` (ALLY1 adds the Match target after
  the 21st — §6.5).
- **The builder's "passive toggle mechanical" (2026-11-05)**: D7's passive form, and Step 0.1's question.
- **Robot one has no build** (`build_selection.hpp` :113-117; Makefile :95-101 has ROBOT=bench/xdrive/tank only):
  the coverage check's G1.

### 0.6 THE MECHANISM COVERAGE CHECK — every mechanism the team may build for the skills run, against the core AS IT IS ON MAIN (`d684f34`)
Read: `mechanism/kinds.hpp`, `spec.hpp`, `wiring.hpp`, `service.hpp`, `driver_inputs.hpp`, `chassis/routine.hpp`;
robot two's `src/robots/mechanism_table.hpp` :107-133 as the worked example. "Today" = what a team can write and run
on the host test plant at `d684f34`, with nothing in flight. **Honest: no hardware has run any of it** — robot two's
claws were uploaded, never driven by the core on a robot; robot one has no build.

**The four questions for every mechanism:** DECLARE (a declaration in the team's words, checked at build) · BIND (to
ports, in `wire(...)` — re-cabling changes only that line) · CONTROLLER (a driver-file control) · AUTON (a route step).

| Mechanism | DECLARE today | BIND today | CONTROLLER today | AUTON STEP today | Gap → smallest fix (size) |
|---|---|---|---|---|---|
| **The 4-stage string CASCADE lift** (strings both ways, stall-homed down into its bottom stop, preset LEVELS by stack height) | **YES** — `Lift` (`kinds.hpp` :811-914): `.motors = {Forward, Reversed}` (one to four motor roles, :126-150; `kMaxMotorRoles` 4, `spec.hpp` :61), `.cartridge`, `.gearing = gears(a, b)`, `.shape`, `.safe`, `.stall` (REQUIRED, :200), `.range` in inches, `.gravity = constantGravity(...)`, `.home = homeByStall(0_in, -3_volt, {...})` (:303-309), `.presets` (six named, `kMaxPresets` 6, `spec.hpp` :65) | **YES** — `wire(cascade, motors(8, 9))` (two to four ports, `wiring.hpp` :623-664), optional `angleSensor(p)` (:716) | **YES** — `whileHeld(cascade, L1, L2, 12_volt)` (held: jog, soft-stopped once homed; released: HOLD there, gravity included), `preset(cascade["L3"], Button::Up, 1.5_s)`, `homeOnPress(...)`, `homeAtDriverStart(cascade, 2_s)` (`driver_inputs.hpp` :8-34, :103-110) | **YES** (on the host test plant; the forms are tested in `test/mechanism_auton_steps_test.cpp`, used by no auton yet) — `r.home(cascade, 2_s)` / `r.startHome(...)` (homing while the first leg drives), `r.move(cascade["L3"], 1.5_s)`, `r.move(cascade.to(17.5_in), …)` (constants, unlimited), `r.move(cascade.clampedTo(h), …)` (a height computed from the stack count, clamped and logged), `r.startMove` + `r.waitUntilDone` (`routine.hpp` :568-697) | **G2 — the cascade's travel per spool turn.** `spool(d)` is "string on a spool" (:373-376): carriage inches = string inches. A CASCADE-rigged lift moves its top stage a MULTIPLE of the string pulled (continuous rigging: 1×). **Fix today, no code:** `.shape = measured({{0_in, 0}, {H_in, D}})` — two measured pairs (top-stage height at two output angles) make the exact linear map (`measured`, :391-402, up to 8 pairs, interpolated — and it absorbs any non-linearity a real cascade has). A one-word `spool(d, stages)` would be ~¼ day and one new belief — NOT recommended (the measurement is needed anyway). **G3 — homing "at boot" vs "on demand" — ONE method per declaration (`.home` is a single field), so the team picks** *(re-check R12 — the first pass offered only the stall)*: **(i) `restsAtStop(0_in)`** — ready at program start with NO motion, trusting the lift starts all the way down (it does at a skills start); `r.home` on it only keeps that zero (`service.hpp` :866-874) — **it can never re-home by a move**, so a restart with the lift up leaves a wrong zero. **(ii) `homeByStall(0_in, -3_volt, {...})`** — a MOVE into the bottom stop, so it cannot run at program start (the robot is disabled then); the route's FIRST line `r.startHome(cascade, 2_s)` (homes while the first leg drives) and `homeAtDriverStart` for driver skills; survives a restart, re-homes on demand. **(iii) `homeBySwitch(0_in, -3_volt)` + `homeSwitch('B')` in `wire(...)`** — **zeroed at program start with no motion when the switch is pressed** (`service.hpp` :768-769) AND re-homed on demand by a move; needs the bottom limit switch the builder recommended (11-05, `NOTES.md` :946). **Recommended: (iii) if the switch is fitted; else (ii).** No code in any case. **G4 — "levels by stack height":** six NAMED levels fit buttons; any number of constants (`to(h)`) or computed heights (`clampedTo`) fit autons. **G8** (only if the team wants >6 NAMED): raise `kMaxPresets` (~¼–½ day; `config::MechanismConfig`, the route ids and G3a's mirror read it — after the 21st unless asked). **The cap:** MECH-S-A adds `.currentCap` — **DONE, merging after L6-A2** *(re-check R10)*. **Live tuning:** TUNE1-A (building) — reaches only robots with a build (G1). **The string snap watch:** MECH-S-B, after the 21st (his ruling of 11-05) |
| **The FLIPPER** (motor arm) | **YES** — `Arm` with presets `{"stow", 0}, {"flip", 95}`, `restsAtStop(0)` or `homeByStall`, a stall signature (bounds the push's force, `<R18>`a) | **YES** — `wire(flipper, motors(10))` | **YES** — `whileHeld` / `preset(flipper["flip"], …)` | **YES — the blind flip** (`r.move(flipper["flip"], 0.6_s).move(flipper["stow"], 0.6_s)`). **NO — the camera-checked flip** | **This brief (AF1S-A1/A2).** No new kind |
| **A pneumatic flipper** | **YES** — `Pneumatic`, two positions, `.moveTime` the team's | **YES** — `lines('C')` | **YES** — `toggle(...)`, `preset(flipper["out"], …)` | **YES** blind (`r.set(...)`); camera-checked: this brief (D7) | AF1S-A1's pneumatic overload (~in A1) |
| **A passive flipper** (a fixed bar, "passive toggle mechanical") | Nothing to declare (no motor, no line) | — | — (the driver drives into the Toggle) | Blind: a drive step; camera-checked: this brief's passive form (D7) | AF1S-A1's passive form (~in A1); Step 0.1 settles whether the bar rolls exactly one face per stroke (AF-H4) |
| **Claws** — robot two's pivots (Power on 6/7) and its pneumatic claw (line A) | **YES** — `src/robots/mechanism_table.hpp` :113-128 (two `Power`, one `Pneumatic`, `grabs`) | **YES** — :129-132 | **YES** — `whileHeld(clawPivot6, R1, R2, 12_volt)` (`src/team/robot_two_driver.hpp`) | **YES** — `r.run(clawPivot6, 12_volt, 0.4_s)`, `r.set(claw["open"])` — *(re-check R11)* documented in COMMENTS at `autons.cpp` :19-29 ("No auton here steps a mechanism yet") and tested in `test/mechanism_auton_steps_test.cpp` :86-89; no auton uses them | None in the core. A claw pivot WITH positions (hold an angle) is an `Arm` — exists. Robot one's claws: G1 |
| **Rollers / intakes** (none this competition) | **YES** — `Power`, optional piece sensor (`distanceSensor(2_in)`, `opticalSensor(0.5)`, `limitSwitch()`, :175-193), optional stall | **YES** — `wire(intake, motors(11, 12), pieceSensor(13))` | **YES** — `whileHeld(intake, A, B, 12_volt)` | **YES** — `r.run(intake, 12_volt, seesPiece, 3_s)`, `seesPiece.orAfter(2.5_s)`, `stalls`, `stopsSeeingPiece` | None |
| **A flywheel / shooter** (not this game's, listed for "all") | **YES** — `Flywheel` (:928) | **YES** | **YES** — `toggle(flywheel["shoot"], X)` | **YES** — `r.spinUp`, `r.stop` | None |
| **Pneumatics** (any cylinder) | **YES** — two or three named positions on one or two lines (`kMaxPneumaticPositions` 3, `kMaxLineRoles` 2, `spec.hpp` :63-67); a third cylinder group is a second `Pneumatic` | **YES** — `lines('A')`, `lines('B', 'C')` | **YES** — `toggle`, `preset` | **YES** — `r.set(...)` | None. `.moveTime` has no default (measure it) |
| **A passive toggle** (mechanical, no code) | Nothing to write | — | — | — | None in code; a route visiting a Toggle with it uses the passive form above |
| **Any mechanism on ROBOT ONE** | **NO** — robot one has no mechanism table, no chassis table, no `RobotId` (`build_selection.hpp` :113-117), no Makefile `ROBOT=` value (:95-101), no driver file, no autons file, no boot path; every program after the tester is `ROBOT=tank`-only (MECH-S/TUNE1 chunk, re-check #10) | **NO** | **NO** | **NO** | **G1 — robot one's hand-written build plumbing** (RC18, `docs/internal/chunks/G3a-robot-profile-generator-panel.md` :1270, :1841 at `d684f34` — :1245, :1816 in the coordinator's copy: "its own chunk, the coordinator's"; never briefed, never launched — *re-check R10/R11*). Rough **~1½–2½ builder-days** (this writer's estimate, from robot two's files: a `RobotId`, its chassis table and mechanism table regions, `boot_source`'s case, the Makefile's `ROBOT=` branch, the src build gate's new builds, a driver file and an autons file, their tests) — **the largest gap for the 21st, and it blocks EVERY other row on robot one**. If "both identical" means robot two is rebuilt as the cascade robot, G1 shrinks to rewriting robot two's two tables (~½–1 day) — D9's question |
| **A mechanism whose numbers are unknown until robot one exists** | **Refused at BUILD, by name** (`spec.hpp` :19-24: an UNSET field is never invented; a stall signature, a range, a home, the gearing are REQUIRED) | — | — | — | **G6 — by design, no code:** the robot file declares a mechanism only when Step 0 has measured it; until then it is simply not in `mechanisms(...)` and the robot builds and drives without it. Tighten the dates instead: robot one's facts at the bench on ~Nov 14, typed the same day (D9) |
| **Two motors on one lift, mirrored** | **YES** — `{Forward, Reversed}`; each motor's direction checked on the robot by Bench Tests station 13's direction check page (MECH-A6, `5120dad`) — `ROBOT=tank` only until G1 | | | | None in the core; G1 for robot one |

**Summary of the check:** every mechanism KIND the team named can be declared, bound, driven from the controller and
stepped in an auton with what is on main TODAY — **except the camera-checked Toggle flip (this brief)** — and **none of
it can exist on robot one until robot one has a build (G1).** The cascade needs no new library word (G2–G4 are
answered by `measured`, a homing word chosen from three (G3 — *re-check R12*), and constants); the current cap (MECH-S-A) is DONE and merging after L6-A2, live tuning (TUNE1-A) is building *(re-check R10)*.

---

## 1. Scope — IN (every new and changed file, by run)

### 1.1 `include/shulib/competition/toggle_visit.hpp` — NEW: the step (AF1S-A1 the explicit form; AF1S-A2 the everyday form)
- **The request** (every number the TEAM's from Step 0; 0 = UNSET and refused by name — the core's rule, no library
  default):
  ```cpp
  struct ToggleVisitRequest {
      vision::override2026::ToggleId toggle{};   // WHICH Toggle (§1.2) — "ours" comes from it
      FlipperGoal flipper{}, back{};             // two presets of ONE Arm/Lift, or two positions of ONE Pneumatic,
                                                 // or both default = a PASSIVE flipper (D7). FlipperGoal converts
                                                 // IMPLICITLY from mechanism::PositionGoal and PneumaticGoal
                                                 // (kinds.hpp :447, :476) — so `.flipper = flipper["flip"]` (R9)
      units::Length lookFrom{}, offBy{}, pushFrom{};  // lens to face at the look spot; its tolerance; at the push spot
      units::Time lookWithin{}, flipBound{}, backBound{}, settle{}, driveBound{}, margin{};
      units::Time maxImageAge{};                 // the camera's image age at its read (HA-192, Step 0.2) — rule 2's
                                                 // boundAtImage and rule 6's "a picture after the retract" (R8)
      std::uint8_t maxPushes = 2;                // 1 or 2; never 3 (a 3 is refused)
  };
  ```
  `pushFrom > lookFrom`, `offBy ≥ lookFrom`, **an excursion `lookFrom − pushFrom` of twice the tracker's
  `frozenMoveIn` or less** (*re-check R4*: the look after must be able to see a frozen camera — so "look and push from
  one spot" is refused, D2 (b)), a flipper whose two goals name two mechanisms or the same preset → refused (consteval
  where the request is built as a constant — the core's build-check idiom; a run-time PRECONDITION otherwise).
- **The verdicts** (`ToggleVisitVerdict`, appended-only like every enum the route shares): `AlreadyOurs`, `Flipped`
  — the two that SUCCEED — and `NotSkills`, `NoCamera`, `FlipperNotReady` (absent / not homed / `pneumatic`
  absent), `WrongWall` (FIELD start), `UnknownBefore` (+ the tracker's `ToggleWhy`), `NotAtSpot` (+ range and bearing),
  `NotEnoughTime` (+ left / needed), `NoChange`, `StillNotOurs`, `UnknownAfter` (+ why), `NoTimeForSecondPush`,
  `FlipperStuck` (a retract that did not reach `back`), `Cut` (the guard's deadline or a disable);
  *(re-check)* **`OursAfterShortStroke`** (R1: ours after a push that STALLED or TIMED OUT, or a passive drive that
  did not reach `pushFrom` — maybe stuck part-way) and **`RetractedForTime`** (R7: the retract-now rule fired).
- **The report** (`ToggleVisitReport`, a VALUE): verdict, `toggle`, `ours` (the target colour), `before` / `after`
  (`vision::ToggleColour`), `whyBefore` / `whyAfter`, `pushes`, the camera's range and bearing at the look,
  `secondsLeftAtStart`, `secondsNeeded`, `seconds` (the step's own time), each flipper step's `StepReport`.
  `ok()` — AlreadyOurs or Flipped; **`outcome()` → `manipulation::MechanismOutcome`**: `Succeeded` (ok), `Cancelled`
  (Cut — STEP1-A's cut rule then stops the route whatever its onFail), `Stalled` (FlipperStuck, or a push that
  stalled and the visit ended short), `TimedOut` (NotEnoughTime, NoTimeForSecondPush, RetractedForTime),
  `Unconfirmed` (every other doubt, OursAfterShortStroke included). The words used in the log and by RPV1: `toggleVisitWord(verdict)`.
- **`checkAndFlipToggle(ExplicitToggleCamera&, chassis::Chassis&, sequence::RunGuard&, mechanism::MechanismService*,
  const competition::AutonEntry&, const ToggleVisitRequest&, hal::ITelemetrySink&)`** — AF1S-A1, over explicit
  handles (the host tests, Tier 3); **`checkAndFlipToggle(AutonContext&, const ToggleVisitRequest&)`** — AF1S-A2, the
  everyday form, reading the camera slot through `ctx.chassis.deps().ctx->toggleCamera()` exactly as L6-A2 reads the
  wall sensors. Also exposed: **`lookAtToggle(...)`** (steps 1–2 only, never moves anything — D1 (c)'s building block)
  and **`worstCaseSeconds(request)`** (the time check's own sum — RPV1's budget, §6.6).
- **THE RULES, in order — each one a test (§2.2):**
  1. **Allowed?** `ctx.entry.kind == Skills` else `NotSkills`; a camera slot (feed + tracker) else `NoCamera`; the
     flipper's slot found, built and (position kinds) homed and AT `back` within its reached tolerance — else
     `FlipperNotReady` (**it never moves a flipper to `back` to make it ready** — the route does that); with a FIELD
     start, the estimate's heading within ±45° of the direction TOWARD the named wall — minus `kTogglePlaces`'
     inward normal (*re-check R6*; right 0°, blue +90°, left 180°, red −90°, F1 CCW-positive) — else `WrongWall`. Nothing read,
     nothing commanded, the clock unmoved.
  2. **Look** — ticking the feed and feeding the tracker inside `guard.waitFor` (so the service and the active
     motion tick, and a deadline cuts it — `run_guard.hpp` :436-448), up to `lookWithin`, for a view that is
     `Sure`. The pose handed to the tracker is the estimate with its stated bound (`PoseBound`), widened for the
     image's age (`boundAtImage`, with the request's `maxImageAge`). Not sure → `UnknownBefore` + the reason. Sure
     but the band's range interval is not WHOLLY INSIDE `[lookFrom − offBy, lookFrom + offBy]` (*re-check R2*:
     containment, never overlap — P6/P7), or its bearing interval is not wholly inside the tracker's window, or (FIELD
     start) the estimate's lens-to-wall distance along the wall's normal is more than `offBy` from `lookFrom` →
     `NotAtSpot`. The hold that counts must have begun inside this step (the tracker is fed from the look's start).
  3. **Ours?** `before == ours` → `AlreadyOurs`. Nothing pushed.
  4. **Time.** `need(n) = n × (⌈driveBound⌉ + ⌈flipBound⌉ + ⌈backBound⌉ + ⌈settle⌉ + ⌈driveBound⌉ + ⌈lookWithin⌉ +
     2 ticks) + margin`, ⌈x⌉ rounded UP to the scheduler's tick — AF1's re-checked formula (AF1 brief §1.1, P6: 0
     failures in 768 timelines), with the drive in and out and the look after each push added. Push 1 starts only
     when `guard.remaining() ≥ need(maxPushes)` (a visit that may need two pushes is not started unless both fit);
     push 2 only when `remaining() ≥ need(1)`. Else `NotEnoughTime` / `NoTimeForSecondPush`, ONE line.
  5. **Flip — every push paired.** Drive straight IN by `lookFrom − pushFrom` along the current heading —
     *(re-check R3)* **never a blocking `Chassis` verb** (`moveTo` → `runBlocking` → `waitUntilSettled`,
     `chassis.hpp` :295-300, :557-566: nothing of the step runs during it, so the camera goes unread and a camera frozen
     at the look spot comes back to it reading stale "sure" — P5(a)). Instead:
     `motion::MoveToPose m{chassis.deps(), target, chassis.motionConfig(), driveBound.value()};
     chassis.scheduler().async(m);` with `target` = the estimate moved `lookFrom − pushFrom` along its heading, the
     heading unchanged, then `guard.waitFor(pred, driveBound)` whose predicate ticks the feed, feeds the tracker every
     new read (with the pose and its bound), runs the retract-now check, and is satisfied when
     `chassis.scheduler().completedCount()` has grown; a scope guard cancels the motion on EVERY exit (as
     `Chassis::DetachGuard`, :536-555), so the motion never dangles in the scheduler's slot. The drive back out is the
     same with the sign reversed (on a tank a straight reverse leg — the executor pins it on the tank rig). Then
     command `flipper`; on its verdict —
     ANY verdict, SUCCEEDED / TIMED_OUT / STALLED — command `back` on the next tick; wait `backBound`; `settle`.
     A retract that does not reach `back` → `FlipperStuck`: **the robot does NOT drive out** (a flipper caught on the
     Toggle would drag it — `<GG9>`); one Warn line naming it; the route's onFail decides (the recipe shows a
     `skipTo` to a recovery step). If the time left falls to `backBound + settle + driveBound + margin` while the
     flipper is out (*re-check R7*: the drive out is included — the robot must be OFF the Toggle at 0:00, `<SC4>`b),
     `back` is commanded AT ONCE (the push is replaced — the most recent command wins), the robot drives out as soon
     as `back` is reached, and the visit ends `RetractedForTime`. Then drive straight OUT by the same distance. A PASSIVE flipper: the drive in and out IS the stroke (no mechanism command).
  6. **Look again — moving, and only after the retract** — *re-written at re-check (R3, R4, R8; P8)*. The tracker is
     fed on EVERY read from the look before, through the drive in, the push, the retract and the drive out (R3). The
     after-colour counts only from a SURE view on a read whose IMAGE is after the retract's end
     (`readAt − maxImageAge ≥ retractEnd` — R8: the read's time is not the picture's, HA-192) **and** after the lens
     has travelled more than `frozenMoveIn` since the retract's end (the excursion is more than twice `frozenMoveIn`,
     so a camera frozen at ANY point of it has been moved past the frozen check's distance and reads "camera frozen?").
     The first pass's test — "the hold BEGAN after the retract" — is dropped: where the camera still names the colour
     at the push spot (a high, tilted lens) the hold never breaks, and NO CHANGE was unreachable (P5(b); P8: 3 of 5
     mounts → 5 of 5 now, 0 wrong colours accepted either way). For a PASSIVE flipper "the retract's end" is the
     moment the drive in ends. Up to `lookWithin` after reaching the look spot.
     `after == ours` after a FULL stroke (the push `Succeeded`; a passive drive that reached `pushFrom`) → `Flipped`,
     the line ending `seating not checked`; `after == ours` after a short stroke → `OursAfterShortStroke` (*re-check
     R1, D11*: the camera cannot tell a seated Toggle from one stuck part-way). `after == before` → `NoChange` (a sticky Toggle and a camera frozen since before the
     push look the same; both stop). Not sure → `UnknownAfter` + why. **Push 1 and `after` the THIRD colour** →
     rule 4's push-2 time check, then rule 5 once more, then rule 6; push 2 and not ours → `StillNotOurs`. **Never a
     third push.**
  7. **One `[TGL]` line per visit** (subsystem `TGL`, ASCII, ≤ 160 bytes, Info for the two successes, Warn otherwise),
     e.g. `red wall Toggle: YELLOW -> RED after 2 pushes, 5.8s (ours RED)`, `red wall Toggle: not at the spot -- band
     at 19.8-29.6 in, look spot 16 +- 6 in`, `red wall Toggle: UNKNOWN before (camera frozen?) -- nothing pushed`,
     `red wall Toggle: not now -- 3.1s left, needs 6.4s`; every `AlreadyOurs` / `Flipped` line ends
     `seating not checked` (*re-check R1*), e.g. `red wall Toggle: YELLOW -> RED, 2 pushes, 5.8s, seating not checked`. The service's own `[MECH]` lines for each push and retract
     appear beside it. And `wall not checked (relative start)` when it applies.
- PROS-free, sim-free, allocation-free, single-task; it never runs in the background and never reads the camera
  outside its own waits.

### 1.2 `include/shulib/vision/override_elements.hpp` — CHANGED: each Toggle's Quadrant colour (AF1S-A1)
- `enum class ToggleId : std::uint8_t { Unset = 0, RightWall, LeftWall, BlueWall, RedWall }` in `kTogglePlaces`'
  order (:75-76), named handles `toggles::kRedWall` … for the route, and `kToggleQuadrantColour[4] = {Red (right),
  Blue (left), Blue (blue wall), Red (red wall)}`, cited to Figure Q-1 and pA15 and to **A4 register HA-243**
  (reasoned — a reading of a picture). DATA ONLY, like the rest of the file (:5-7). *(re-check R9)* The table's type
  is this file's OWN `enum class QuadrantColour : std::uint8_t { Unset = 0, Red, Blue }` — NOT
  `vision::ToggleColour`, which lives in `toggle_tracker.hpp`, a header that INCLUDES this one (a circular include
  otherwise); `toggle_visit.hpp` maps `QuadrantColour` to `ToggleColour` (a test pins both entries). VIS1-A2 does not change this file
  (`diff` against `d684f34`: identical); the second of VIS1-A2 and AF1S-A1 to land re-reads it.

### 1.3 The robot's camera answer for autons (AF1S-A2) — L6-A2's pattern, a second OPTIONAL slot
- `config/robot_config.hpp` — CHANGED: `CameraConfig` gains `toggle` (a `vision::ToggleTrackerConfig` and a
  `vision::CameraFeedConfig`, both UNSET by default — a default member initializer, so every existing config compiles),
  validated by `toggleTrackerConfigProblem` / `cameraFeedConfigProblem` only when set.
- `robot/robot.hpp` — CHANGED: when the camera was built AND its placement is measured AND the Toggle block is set, the
  Robot builds a `ToggleCamera {CameraFeed, ToggleTracker, CameraModel}` (static storage, after the wall sensors);
  any refusal leaves the slot null with a sentence (`toggleCameraAbsence()`), never refuses the robot; the boot
  line's camera words add `toggles ready` / the reason.
- `chassis/robot_context.hpp` — CHANGED: the second optional slot `ToggleCamera* toggleCamera = nullptr`
  (forward-declared, as `WallSensors` is at :39-41 (L6-A2, staged)) and `toggleCamera()`.
- `motion/motion_scheduler.hpp` — CHANGED: ONE line, the shadow context carries it (as `.walls` at :668 (L6-A2,
  staged)) — L6's landmine: without it every everyday call says `NoCamera` (M-A2-1).

### 1.4 `include/shulib/sim/toggle_plant.hpp` — NEW: the host Toggle (AF1S-A1)
AF1's planned plant (AF1 brief §1.7), re-pointed from the Optical to the camera: three faces in a configurable roll
order (both of the research's); a stroke rolls ONE face when the flipper passes `rollDeg` outward (none on a short
stroke; TWO in "over-roll"; none when "sticky"; *(re-check R1)* in "jammed part-way" it stops at a set angle
(e.g. 50–100° of its 120° roll), drawn with `addRolledToggle`, and never seats — `scoredColour()` yellow); a PASSIVE mode where the robot's true position passing `pushFrom`
is the stroke; a seat time; contact while the flipper is past `contactDeg` or the robot's front is within
`touchIn` of the wall; `scoredColour()` = the field face only when seated and untouched, else yellow (`<SC4>`). It
writes the TRUE field and bottom face colours into the host camera model (`sim::CameraPlant::addToggle`, cleared and
re-added on each change; mid-roll via `addRolledToggle`) and couples to `sim::ArmPlant` through the flipper's true
angle. Ground truth for the tests only (the CI's sim-layering guard).

### 1.5 Tests (§2.2) and the cookbook example (AF1S-A1 the test; AF1S-A2 the page)
- NEW: `test/toggle_visit_rules_test.cpp` (the explicit form, rules 1–7), `test/toggle_visit_scenes_test.cpp`
  (end to end on the host plants: a tank `ChassisRig`, the camera plant, the arm plant, the Toggle plant),
  `test/sim_toggle_plant_test.cpp`, `test/toggle_visit_ctx_test.cpp` (A2: through a real `robot::Robot` and
  `runAuton`), `test/robot_toggle_camera_test.cpp` (A2), compile refusals (`test/compile_refusals/t*_…`).
- CHANGED: `test/cookbook_examples_test.cpp` (the recipe's route compiled and run on the host plants);
  `test/boot_source_test.cpp`, `test/robot_camera_test.cpp` (A2: the boot line's camera words).

### 1.6 NOT changed
`chassis/routine.hpp` (D1 (a)); `mechanism/*` (no new kind, no new member — D7); `teleop/*`; `src/` (robot one's
entries are G1's, the route is the team's); `include/shulib/version.hpp` (no frozen surface grows — the L6 brief's
§1.8 rule).

---

## 2. Step 0, the runs, their tests and mutations, and the definition of done

### 2.0 STEP 0 at the robot — what the team measures (needs NO new code; robot one ~Nov 14, robot two now)
- **0.1 The flipper.** What is it — a motor arm, a pneumatic, or a passive bar (D7)? Motor: its port, cartridge, gear
  teeth (motor first), which way + turns it, its stow angle on its stop, its flip angle, its far stop, a stall
  signature (amps, rpm, time — held against a Toggle by hand); pneumatic: its line, which level pushes, its move time.
  **On a real or mock Toggle at field height** (axle 13.20 in, on brackets above an 11½-in wall — AF1's Step 0.2):
  does ONE stroke roll exactly ONE face? Which face follows which, mounted as a RED-side and as a BLUE-side Toggle
  (pA15) — so how many pushes from yellow (AF-H4, HA-231)? How long until it sits flat (`settle`, HA-232)? *(re-check
  R1)* **Does every FULL stroke leave it seated — twenty strokes, each checked by eye for a Toggle stuck part-way?**
  (the camera cannot see it — D11); and what a stroke that stalls on a stuck Toggle looks like in the `[MECH]` line. Times:
  push, retract (worst of five each → `flipBound`, `backBound`). With the flipper all the way out the robot fits
  24 × 24 in (`<SG2>`a).
- **0.2 The camera's mount** (VIS1's Step 0, `docs/internal/chunks/VIS1-ai-vision-colours.md` :1340-1373): lens
  forward, left, height to the lens centre, yaw, pitch, each with its tolerance → the robot file's camera placement
  (`measured = true`). *(re-check R8)* **The image's age at its read → `maxImageAge`** (HA-192's Step 0.2: Bench
  Tests station 12's change timer as it stands, no new code — wave a card, read the lag; take the worst of ten).
- **0.3 The colours, taught on the REAL Toggle's faces** (red, blue, yellow descriptors in the AI Vision utility) under
  light like the event's; and the tracker's numbers (hold, alive, aspect, length fractions, range window, bearing
  window, frozen thresholds — `toggle_tracker.hpp` :176-207 (VIS1-A2)).
- **0.4 The look spot.** Square to a Toggle, centred, step the robot back 4 → 36 in: where does the camera name each
  face, and how far turned does it still (P1 predicts ~12–24 in and within ~3°, by mount)? Pick `lookFrom` in the
  middle of that band. *(re-check R2)* **`offBy`** = the larger half-width of the RANGE INTERVAL the page prints at
  the look spot (P7: ~4–6½ in on the host model) + the route's arrival allowance; then park 3 in too far and 3 in too
  near and see `NotAtSpot` — if a lens near the Toggle's height cannot tell them apart (the host model's 12-in lens
  could not), rely on the route's odometry and say so in the route. Bench Tests station 12 prints what the tracker says (after VIS1-B — `ROBOT=tank` only until
  G1).
- **0.5 The push spot.** With the flipper at stow, square: the lens-to-face distance where the flipper (or the bar)
  rolls the Toggle → `pushFrom`. At the push spot with the flipper at stow, **does any part of the robot touch the
  Toggle** (`<SC4>`b)? Does the flipper at stow block the camera's view of the Toggle from the look spot?
- **0.6 The drive in and out.** Ten in-and-out moves of `lookFrom − pushFrom` (more than twice `frozenMoveIn` —
  R4): the worst time → `driveBound`; how far the robot ends from where it started, and where the flipper lands on the
  face (HA-244).
- **0.7 The field** (D3's question): on the event's field, which walls' Toggles start with RED facing out (HA-243).

### 2.1 The runs — two, each its own go, each verified and committed on its own
| Run | What (§1) | Files it writes | Waits on | Size (builder-days, rough) | Public docs |
|---|---|---|---|---|---|
| **AF1S-A1 — the step** | §1.1 (explicit form, `lookAtToggle`, `worstCaseSeconds`), §1.2, §1.4, §1.5's A1 tests | NEW `competition/toggle_visit.hpp`, `sim/toggle_plant.hpp`, tests; CHANGED `vision/override_elements.hpp` | your go; VIS1-A2's BUILD ended (a snapshot base if not merged — re-checked at merge) | **~2¾–3¼** *(re-check: +¼ — R1's full-stroke rule, the plant's jammed mode and T14; R3's async drive and MA24; R4/R8's after rule)* (the rules and the report ~1–1¼; the in-and-out drive and the time check ~½; the three flipper forms ~¼; the Toggle plant and its coupling to the camera and arm plants ~½–¾; tests and mutations included in each; docs ~¼) | changelog; guide 19 (the camera chapter, VIS1-A2's — a section "Checking and flipping a Toggle in a skills route"); guide 14 (what it cannot catch); guide 11 (the `[TGL]` line); the HA register |
| **AF1S-A2 — the camera for autons** | §1.1's everyday form, §1.3, the recipe | CHANGED `config/robot_config.hpp`, `robot/robot.hpp`, `chassis/robot_context.hpp`, `motion/motion_scheduler.hpp` (one line), `competition/toggle_visit.hpp`; tests | AF1S-A1 merged; **L6-A2 merged**; **VIS1-A2 merged** | **~1¼–1½** (the config block ~¼, the Robot's build and its absence words ~½, the slot and the shadow line ~¼, the everyday form ~¼, the recipe and its test ~¼) | changelog; a cookbook recipe (§3); guide 10 (the API page's list) |
| **The team's route** | the skills route, from the recipe | `src/team/…` (robot one's file — G1) | AF1S-A2 merged; G1; Step 0 | the students: an evening | — |
| **AF1S: A1 + A2** | | | | **~4–4¾** *(re-check: A1 +¼)*; verification ~half again: **~6–7 slot-days** | |
Sized against AF1-A's re-checked pieces (the action ~2–2¾ — this step is smaller: no driver arbitration, no new-visit
rule, no arm button, no period clock; the Toggle plant ~1–1¼ there, less here because the camera plant exists) and
L6-A2 (~1¾–2, the same slot pattern). **None of these is a measured duration.**

### 2.2 Tests (ADVERSARIAL — each tries to break ONE rule) and the mutations each must turn red
Every mutation is RUN and seen RED, then restored; the executor logs each in `AF1S-PROGRESS.md`. Every end-to-end test
uses the host Toggle plant as the oracle (its true face and `scoredColour()`), never the step's own report.

**AF1S-A1**
| # | Test (the rule it tries to break) | Mutations it must turn RED |
|---|---|---|
| T1 | **Never a push on OUR colour**: a RED-Quadrant Toggle showing RED (and a BLUE one showing BLUE) → `AlreadyOurs`, the flipper's command number unchanged, the drive never commanded; after push 1, `after == ours` → no push 2 (the plant's over-roll case); across seeded scenes (noise, both roll orders, sticky, over-roll, our other robot flipping it at a scripted time) the plant counts flipper contacts while its TRUE seated face is ours — **0**; *(re-check R5)* **a lookalike while the true face is ours**: the RED face's band turned away by a gate, a BLUE band passing inside the window → the outcome PINNED as it is (without the position gate, the step may push; with containment (R2) refusing the band's range, or with the position gate after a wall reset, `NotAtSpot` / `UnknownBefore`, nothing pushed) — the test names which, so a change in it is seen | MA1 the target compared against `before`'s band INDEX, not the colour; MA2 `AlreadyOurs` decided from the Quadrant table of the WRONG Toggle (an off-by-one in `ToggleId`) — the right-wall Toggle then wants BLUE; MA3 push 2 without the look |
| T2 | **Never a push on UNKNOWN**: each of the tracker's reasons alone (no read, silent, not usable, full, no band, cut, maybe bottom, two colours, not held, frozen, beside, above) → `UnknownBefore` with that reason, nothing commanded | MA4 `Unknown` treated as "not ours" (a push); MA5 the hold ignored (one read taken as sure) |
| T3 | **A stale frame**: the camera unplugged mid-look (alive passes) → `UnknownBefore` "camera silent"; a FROZEN camera while parked, the Toggle flipped under it by the plant (P2) → the look AFTER a push says `UnknownAfter` "camera frozen?" and never `Flipped` from the stale frame; *(re-check R3, P5(a))* a camera FROZEN at the look spot before the drive in, the Toggle flipped under it → `UnknownAfter` "camera frozen?" (fed on every read through the drive in and out); *(re-check R4, R8, P8)* a freeze at each of: half-way in, the push spot before and after the roll, half-way out → never a colour accepted; a sticky Toggle, a LIVE camera that names the colour at the push spot (lens 12 up, tilted 10°, push at 10 in) → `NoChange` (not `UnknownAfter`) | MA6 the after-look accepting a view whose picture predates the retract's end (+ `maxImageAge`) — the high tilted lens names the PRE-roll colour at the push spot and a flipped Toggle reports `NoChange`; MA7 the back-off skipped (parked after-look: frozen invisible); **MA24** *(re-check R3)* the drive in and out through a blocking `chassis.moveTo` (the tracker fed only while parked — P5(a)'s stale YELLOW "sure" → `NoChange` on a flipped Toggle); MA25 *(R8)* `maxImageAge` ignored in rule 6 (a read stamped just after the retract, its picture before it, counted) |
| T4 | **Two Toggles / two colours**: two Toggle-coloured bands of different colours in the window → `UnknownBefore` "two colours"; a second Toggle's band at another wall outside the bearing window → ignored, the named one judged | MA8 the step choosing the nearer band itself instead of the tracker's verdict |
| T5 | **The robot off the spot** *(re-check R2)*: the band's range interval reaching 0.01 in outside `[lookFrom − offBy, lookFrom + offBy]` → `NotAtSpot`, nothing driven; wholly inside → proceeds; **a robot parked 8 in too far (lens 10 up, `lookFrom` 16, true 24 in — P7: the interval [16.7, 29.8] OVERLAPS the window) → `NotAtSpot`**; with a FIELD start, a pose 2 × `offBy` off the look spot → `NotAtSpot`; turned 8° at 16 in → `UnknownBefore` (P1) — never a push; a FIELD start facing the wrong wall (heading 60° off the named wall's normal) → `WrongWall` before any read; a RELATIVE start → proceeds and the line says `wall not checked` | MA9 *(re-check R2)* the range check by OVERLAP instead of containment, or dropped (the robot 8 in too far drives in short and pushes air — the plant: no roll); MA10 the wall check using the inward normal itself (the first pass's own wording — every correct visit refused, a robot facing away accepted) |
| T6 | **The period ending**: `remaining()` just under `need(2)` → `NotEnoughTime`, ONE line, nothing commanded; just over → proceeds; push 1 started exactly at the bound with every stroke at its bound → push 2 still fits and the flipper is at `back` and the robot off the Toggle before the deadline (the plant); a slow plant whose push is still out when `remaining()` reaches `backBound + settle + driveBound + margin` → `back` at once, the drive out, `RetractedForTime`, and **the plant's touch flag clear before the deadline** (*re-check R7*); the guard's deadline landing mid-look → `Cut`, `outcome() == Cancelled`, the clock not past the deadline + one tick | MA11 `need(1)` for the first push (a two-push visit started that cannot finish); MA12 the retract-now rule dropped (the flipper out at 0:00 — `scoredColour()` yellow); MA12b *(R7)* its threshold without `driveBound` (the robot still touching at 0:00 — `scoredColour()` yellow); MA13 waits through `chassis.waitUntil` instead of `guard.waitFor` (the cut late by up to `lookWithin`) |
| T7 | **A flipper that stalls**: the push STALLED (the plant's sticky Toggle, an arm held) → `back` commanded on the next tick, the look after, `NoChange`; the retract STALLED → `FlipperStuck`, `outcome() == Stalled`, **the drive NOT commanded out**; TIMED_OUT on either likewise | MA14 `back` only after a SUCCEEDED push; MA15 the robot backing out with the flipper stuck out |
| T8 | **At most two pushes, the second only on the third colour** (P3's 84 cases as a table over the plants): FLIPPED exactly where P3 says, never a third push; push 1 → the start colour again → `NoChange`, no push 2 | MA16 a third push allowed; MA17 push 2 on "no change" |
| T9 | **Not allowed**: a Match-kind entry → `NotSkills`; no camera slot → `NoCamera`; the flipper absent, not homed, or not at `back` → `FlipperNotReady` and **never moved to `back`**; each with the clock unmoved and nothing read | MA18 the Skills check dropped; MA19 a flipper not at `back` moved there by the step |
| T10 | **The three forms**: an Arm, a Lift and a Pneumatic flipper each flip the plant's Toggle and log their `[MECH]` lines; the PASSIVE form flips it with the drive alone and commands no mechanism; refused at build: two goals of two mechanisms, the same preset twice, `maxPushes` 3 | MA20 the pneumatic form waiting no move time |
| T11 | **The report and the line**: ONE `[TGL]` line per visit, ≤ 160 bytes, ASCII, Info only for AlreadyOurs/Flipped; `outcome()` for every verdict as §1.1 maps it; under STEP1-A, `onFail(carryOn)` goes on after `NoChange`, `onFail(skipTo("x"))` lands at `x`, and a `Cut` stops the route whatever the choice | MA21 `Cut` mapped to `Unconfirmed` (a carry-on route keeps driving past the buzzer) |
| T12 | **The host Toggle plant itself** (`sim_toggle_plant_test`): one roll per full stroke, none on a short one, two in over-roll, none when sticky, the passive stroke, the seat time, `scoredColour()` yellow while touched or unseated, the camera plant shows the true faces after each change | — (a plant mutation must turn T1/T8 red: the roll on the inward stroke) |
| T13 | **`worstCaseSeconds`** equals rule 4's `need(maxPushes)` for the request, on the tick grid | MA22 the drive terms dropped |
| T14 | *(re-check R1)* **A Toggle stuck part-way — what the step says, pinned against the plant**: after a FULL stroke the plant jams at 50°, 70° and 100° of the roll (it reads the destination colour — P5(c)) → `Flipped` with `seating not checked` in the line, while `scoredColour()` is YELLOW — the test PINS this honest limit (it is the documented outcome, guide 14); after a push that STALLED or TIMED OUT and then reads ours → `OursAfterShortStroke`, `outcome() == Unconfirmed`; before any push, a jammed Toggle reading ours → `AlreadyOurs` with `seating not checked` | MA23 the stroke check dropped (a stalled push that then reads ours → `Flipped`); MA23b `seating not checked` missing from a success line |

**AF1S-A2**: `robot_toggle_camera_test` (the slot built only with a camera, a measured placement and the block set;
each refusal leaves it null with its sentence and the robot still drives; the boot line's words); `toggle_visit_ctx_test`
(through a real `Robot` and `runAuton`: the slot reaches `ctx.chassis.deps().ctx->toggleCamera()` — **M-A2-1 the
shadow context's line dropped → every call `NoCamera`**; a RELATIVE-start skills auton flips; a Match auton refuses);
`cookbook_examples_test` (the recipe's route: three Toggles, one already ours, one sticky with `onFail(skipTo(...))`,
run on the plants, the result's counts pinned). Mutations: M-A2-1; M-A2-2 a refused tracker config refusing the robot;
M-A2-3 the slot built with an UNMEASURED placement.

### 2.3 Definition of done — every run (the MECH core's §2.1, adapted)
- [ ] `AF1S-PROGRESS.md` created FIRST and appended as the run goes; nothing committed.
- [ ] Every new public member documented (the API doc gate); every belief carries `A4 register HA-nn` (§4's ids only).
- [ ] The host suite green through `python3 tools/suite_run.py green`; every listed mutation RUN and seen RED, logged
      with the failing test's name, then restored.
- [ ] The ARM header gate clean; the CI's PROS-free and sim-layering guards pass; `python3 tools/src_build_gate.py
      check` (A2 changes headers `src/` includes).
- [ ] The doc gates: `api_doc_tool.py` check-fresh / coverage / examples / removability; `briefing_status.py check`
      (against a fresh build); `doc_staleness_audit.py`.
- [ ] §3's documents changed IN THE SAME CHANGE, and the prose review logged.
- [ ] Every status line under-claims: "host- and build-gated, NOT run on a robot"; `[~]` until a robot has flipped
      a Toggle in a skills route and the camera agreed.

---

## 3. Documents — in the same change as the code (pace rule 3), the prose review logged (rule 4), ≤ ~5 public each (rule 7)
- **AF1S-A1 (five, + `docs/api/`):** `docs/changelog.md` ("Added (no version bump): `competition::checkAndFlipToggle`
  (explicit form), `lookAtToggle`, `worstCaseSeconds`, `vision::override2026::ToggleId` and the Quadrant colours");
  **guide 19** (the camera chapter — a section: the look spot and why it is not the push spot (P1's table in words),
  the rules, what it cannot catch, the numbers Step 0 measures); **guide 14** (*re-check R1* FIRST: a Toggle stuck
  part-way reads as set — "FLIPPED" is not "set"; a frozen camera parked; lookalikes; a touching robot; the pieces
  held; *R2* the spot check is coarse); **guide 11** (the `[TGL]` line, word by word, and what to do for each); the **HA
  register** (§4).
- **AF1S-A2 (five):** changelog; **a cookbook recipe "A skills route that visits the Toggles"** — a FIELD-start
  skills auton that homes the cascade while driving (`r.startHome`), visits three Toggles in the team's chosen
  order, each `r.onFail(skipTo("next"))`, never while holding more than one Pin or one Cup (`<SG6>`), backs off, and
  ends; every number marked "Step 0"; compiled and run by `cookbook_examples_test`; **guide 10** (the API list);
  guide 15 (glossary: look spot, push spot, Quadrant colour); the internal worksheet (Step 0.1–0.7 as a station).
- **Internal, every run:** `AF1S-PROGRESS.md`; `build-order.md`'s AF1 entry and **Next** block (AF1-S before the
  21st, AF1-A..D after); `REMAINING-WORK.md` row 18e split; `roadmap.md`; `PROJECT-BRIEFING.md` (generated).
- **The plan commit (the coordinator's, on the team lead's go, BEFORE AF1S-A1):** this brief to
  `docs/internal/chunks/AF1S-skills-toggle-check-flip.md` with his answers at the top; the HA register's Held line
  gains AF1's HA-228..236, ALLY1's HA-237..242 and these HA-243..245; the deviations table records the 11-10 rulings.
- **The prose review's greps** (each run): `simulator` (none new); `internal/|chunks/|RESUMING|build-order` in public
  docs (none); hardware claims ("tested", "measured", "works on the robot") about the flip (none without a cited
  session).

---

## 4. Hardware Assumptions — five of AF1's reused, three new (HA-243..HA-245)
*Claim · Confidence · Code site · Settled by.* **None is measured.**

| # | Belief | Confidence | Cited in code by | Settled by |
|---|---|---|---|---|
| **HA-231** (AF1's AF-H4, reused) | One flipper stroke rolls a Toggle exactly one face, always the same way round, and the retract does not roll it back; the research's per-side order (red-side Y→B→R, blue-side the mirror) — so from yellow every skills Toggle needs TWO pushes | **reasoned** (pA15 + geometry, design.md §A2) | `toggle_visit.hpp` rule 6 (the rule holds either way — P3; the TIME does not) | Step 0.1, on a red-side and a blue-side mounting |
| **HA-232** (AF-H5) | The Toggle seats (`<SC4>`a) within `settle` after the retract — *(re-check R1, extended)* **after every FULL stroke** (never left stuck part-way), because the camera cannot check it: a Toggle stuck part-way reads its destination colour "sure" (VIS1-A2's banner `toggle_tracker.hpp` :10-18; P5(c): 58 of 108) | **invented** | `ToggleVisitRequest::settle`; rule 6's full-stroke condition; the `seating not checked` words | Step 0.1 (twenty full strokes, each checked by eye) |
| **HA-234** (AF-H7) | At the run's end a V5 motor in its declared safe brake mode does not move the flipper toward `back` (so a flipper out at 0:00 stays out — why rule 4 exists) | **reasoned** (the service safes, it does not retract — AF1's P3) | rule 4, rule 5's retract-now | a timed run with the flipper out at the end |
| **HA-235** (AF-H8) | A push stopped by a sticky Toggle is cut by the flipper's stall signature (or `flipBound`) before it damages the Toggle (`<R18>`a) | **invented** | the flipper's `.stall`, `flipBound` | Step 0.1 with the Toggle held |
| **HA-236** (AF-H9) | The flipper is on the ports, at the angles and of the kind Step 0.1 reports — now ROBOT ONE's | **reported** (when it is) | robot one's mechanism table (G1) | Step 0.1; Bench Tests station 13 (after G1) |
| **HA-243** (AFS-H1) | Each Toggle's Quadrant colour in F1: RED wall and RIGHT wall (x = +70.2) RED; BLUE wall and LEFT wall BLUE (Figure Q-1 read with pA15's start orientation); and every Toggle starts the skills run yellow-in, Quadrant-colour-out | **reasoned** (two drawings, read 2026-11-10; v2.1 changed neither) | `override_elements.hpp`'s `kToggleQuadrantColour` | Step 0.7 on the event's field |
| **HA-244** (AFS-H2) | The step's straight drive in and out of `lookFrom − pushFrom` (~10 in) puts the flipper within its reach of the face, and back within `offBy` of the look spot, on a square robot — **because the route arrived within its allowance: the camera's spot check refuses only GROSS arrival errors** *(re-check R2: the first pass claimed a default `offBy` of 3 in "small enough to refuse a robot parked at the wrong distance" — false by overlap (P6/P7); by containment the host model refuses ~2 in at lens 8–10 but not ~9 in at lens 12 (P7b); no default now)* | **invented** | `ToggleVisitRequest::{offBy, pushFrom, driveBound}` | Step 0.4, 0.5, 0.6 |
| **HA-245** (AFS-H3) | On a REAL field the camera names the Toggle's colour from the look spot as the host camera model predicts (P1: ~12–24 in, within ~3° of square, for a lens 6–12 in up): the wall below the Toggle hides its bottom face from a low lens (the model draws no wall), the taught colours hold under event light, and the flipper at stow is not in the picture | **invented** (the camera's own beliefs HA-192..HA-203 cover the device; this is the look spot's geometry on our robot) | `ToggleVisitRequest::lookFrom`; guide 19's section | Step 0.3, 0.4 |

HA-228..230 (the Optical) and HA-233 (the field timer) stay AF1's, unused here. VIS1's HA-192..HA-203 cover the
camera device (frame rate and freshness HA-192, FOV HA-194, lookalikes HA-199, the face's geometry HA-201).

---

## 5. OUT, and explicitly rejected
- **The driver-control automatic flipper** (AF1-A..D) — after the 21st, the team lead's ruling. It will reuse this
  step's rule 6 and the Toggle plant. **What of AF1-A is NOT needed for skills:** the automatic action in the driver
  loop (`mechanism/auto_action.hpp`), its arm button, the tick order that lets the driver win, `DriverInputs::engaged`
  / `touches`, the `teleop_loop` overload, the period clock (`competition/period_clock.hpp` — the run guard's
  `remaining()` replaces it in an auton), the Optical colour check and `Arm::pieceSensor` (no Optical), the new-visit
  rule (one call is one visit), the policy seam (the route IS the policy; "ours" is the season's table), the screens'
  words (D13/D14). **What IS reused:** the positive-evidence rule, rule 3 (every push paired), rule 5 (the check —
  FLIPPED only on the target, push 2 only on the third colour, never a third), the time formula (P6), the host Toggle
  plant, AF-H4/H5/H7/H8/H9. "The driver always wins" is moot in auton.
- **ALLY1** (the alliance colour; head-to-head "ours") — after the 21st. Until then a Match auton refuses (`NotSkills`).
- **The panel** (G3a) — out before the 21st (the team lead, 2026-11-10).
- **Driving to the Toggle from anywhere** (D2 (c)); **a blind push on UNKNOWN** (D4 (b)); **automatic retries**
  (D5 (c)); **a Routine member** (D1 (b)).
- **Robot one's build plumbing (G1)** — needed, but its own chunk (D9).
- **Counting held pieces** (`<SG6>`) — the route's.

---

## 6. Coordination — shared files, who waits for whom, and what the others get

### 6.1 In flight (re-checked 2026-11-10 evening, again late — R10; NOT edited)
- **The slots, late 11-10** (`NOTES.md` :960-961): RUNNING (5) TUNE1-A, STEP1-A, CAL1-B, this brief, VIS1-A2 (cont3,
  its fourth run); QUEUE: the CURVE1 brief (moved first at the team lead's "when are we doing curves"), the RPV1
  brief, then TUNE1-B1 / STEP1-B / L6-B. **G1 has no brief and no place in the queue** — D8/D9 ask for one.
- **VIS1-A2** (`vis1-a2`, base `9d77c9d`, uncommitted): NEW `vision/{camera_feed,toggle_tracker,piece_finder}.hpp`,
  CHANGED `camera_geometry.hpp`, `src/programs/bench_r3a.cpp`, guide 19. **AF1S-A1 depends on its API** (§0.2's
  list); if A1 starts before VIS1-A2 merges it builds on a snapshot of VIS1-A2's build and re-checks every name at the
  merge. Its base predates L6-A1, so its tree lacks `sim/field_plant.hpp` — AF1S needs neither's absence.
- **STEP1-A** (`step1-a`, base `ef6162e`): `chassis/routine.hpp`'s onFail / label / skipTo and the cut rule. AF1S
  edits no Routine line; its tests T11 run the onFail forms **only once STEP1-A has merged** (before that, T11 pins the
  default stop).
- **L6-A2** (staged on main): AF1S-A2 copies its slot pattern into the SAME four files (`robot_context.hpp`,
  `robot.hpp`, `robot_config.hpp`, `motion_scheduler.hpp`) — **A2 starts after L6-A2 commits** and re-reads them at
  the tip.
- **MECH-S-A** (`mech-s-a`) — **DONE, merging after L6-A2** (`NOTES.md` :960 — *re-check R10*):
  `mechanism/{kinds,spec,service,devices}.hpp`, `hal/…motor…` — AF1S touches none; a flipper's `.currentCap` simply
  works once it lands.
- **TUNE1-A** (`tune1-a`): `mechanism/service.hpp`, `mechanism/live_tuning.hpp` — none shared.
- **CAL1-B** (`cal1-b`): `src/build_selection.hpp`, `src/robots/mechanism_table.hpp`, `src/main.cpp`, the Makefile —
  **G1 (robot one's plumbing) edits the same files**: G1 waits for CAL1-B or re-reads its tip.

### 6.2 Shared files
| File | Who else writes it | Rule |
|---|---|---|
| `vision/override_elements.hpp` | none in flight (VIS1-A2's copy is identical to `d684f34`'s) | AF1S-A1 appends; ALLY1-B / L4 re-read |
| `config/robot_config.hpp`, `robot/robot.hpp`, `chassis/robot_context.hpp`, `motion/motion_scheduler.hpp` | L6-A2 (staged), VIS1-B (its camera block is in `src/`, not here), G1 (no) | AF1S-A2 after L6-A2 commits |
| `docs/changelog.md`, guides 10/11/14/15/19, the HA register, `mkdocs.yml`, `docs/api/`, `PROJECT-BRIEFING.md` | every run | merged serially; the generated files regenerated, never hand-merged |

### 6.3 VIS1-B and robot one
VIS1-B gives ROBOT TWO its camera block and Bench Tests station 12's colours; robot one gets its camera block in G1.
Step 0.3–0.4 use station 12 — on robot one only after G1 (every program after the tester is `ROBOT=tank`-only).

### 6.4 MECH-A6 (landed), TUNE1, MECH-S
The flipper's direction and its presets are checked on station 13's direction page and tuned live (TUNE1-A) — both
on robot two today, robot one after G1.

### 6.5 ALLY1 and AF1 after the 21st
ALLY1 adds a Match target (the alliance colour) to the step — the `NotSkills` refusal becomes "no colour chosen" for a
Match auton; AF1-A builds the driver-loop action over this step's rules and plant.

### 6.6 RPV1 (the route preview) and CURVE1
- **How the preview draws a check-and-flip:** the step's in-and-out drive is an ordinary motion (drawn as path, its
  times measured); the push and retract are mechanism steps (marked); the step's verdict is drawn with its word
  (`toggleVisitWord`). The preview's field must carry the AF1S Toggle plant and the camera plant for the step to
  LOOK; without them the step answers `NoCamera` and is drawn as failed (honest, and the route's `onFail` path is what
  gets drawn). **For the 60-s budget the preview should add `worstCaseSeconds(request)`** for each visit — the ideal
  plant flips in the minimum time, the robot will not. RPV1's brief should take the Toggle plant as an OPTIONAL scene
  (`--toggles yellow|ours|sticky`), so the team can preview "every Toggle needs two pushes" and "this one is sticky".
- **CURVE1:** a curve that ends at a look spot must end SQUARE to the wall within ~3° (P1) — the recipe ends the
  approach with `face(...)` or `turnTo(...)` before the visit.

### 6.7 What the other chunks get from AF1-S
AF1 (after the 21st) gets rule 6, the plant, the time formula; ALLY1 the Match target seam; RPV1 the word and
`worstCaseSeconds`; the team a recipe.

---

## 7. Checks — the coordinator re-runs every one after each run (never the report's word)
Every binary under the memory rule. `<scratch>` is the coordinator's scratch folder — never `/tmp`.
```sh
cmake --build build/test -j"$(nproc)"
python3 tools/suite_run.py green
ctest --test-dir build/test -R compile_refusal --no-tests=error --output-on-failure | tail -3
# the ARM header gate — every v2 header in ONE unit
find include/shulib -name '*.hpp' | sed 's|^include/||' | LC_ALL=C sort | awk '{print "#include \""$0"\""}' > <scratch>/all.cpp
echo "int main(){return 0;}" >> <scratch>/all.cpp
arm-none-eabi-g++ -std=gnu++20 -Wall -Wextra -Wconversion -Wsign-conversion -Wshadow -Werror \
  -Os -mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=softfp -c <scratch>/all.cpp -o /dev/null -Iinclude
# the CI guards
grep -rnE '#[[:space:]]*include[[:space:]]*[<"]pros/' include/shulib | grep -v '^include/shulib/hal/pros/'   # empty
grep -rnE --exclude-dir=sim '#[[:space:]]*include[[:space:]]*[<"]shulib/sim/' include/shulib              # empty
python3 tools/src_build_gate.py check
python3 tools/api_doc_tool.py self-test && python3 tools/api_doc_tool.py check-coverage \
  && python3 tools/api_doc_tool.py check-fresh && python3 tools/api_doc_tool.py check-examples \
  && python3 tools/api_doc_tool.py check-removability
python3 tools/doc_staleness_audit.py && python3 tools/briefing_status.py check
# ── the brief's own ──
git diff --stat -- include/shulib/chassis/routine.hpp include/shulib/mechanism/ include/shulib/teleop/ include/shulib/version.hpp   # empty (§1.6)
git diff --stat -- src/ Makefile tools/          # A1, A2: empty
grep -nE '#include.*(optical|period_clock|auto_action)' include/shulib/competition/toggle_visit.hpp   # empty: camera only, no AF1-A piece
grep -nE '#include.*competition/' include/shulib/vision/*.hpp include/shulib/mechanism/*.hpp          # empty: layering
grep -nE 'override_elements|toggle_tracker|vision/' include/shulib/sim/toggle_plant.hpp               # empty: the plant stays independent of the library's vision code
grep -n 'waitUntil' include/shulib/competition/toggle_visit.hpp | grep -v '^[^:]*:[0-9]*:[[:space:]]*//'   # empty: every wait is the guard's (MA13)
grep -nE '\.(moveTo|strafeTo|turnTo|moveThrough|holdPose|brake)\(|runBlocking|waitUntilSettled' include/shulib/competition/toggle_visit.hpp | grep -v '^[^:]*:[0-9]*:[[:space:]]*//'   # empty: no blocking motion inside the step (R3, MA24)
grep -n 'seating not checked' include/shulib/competition/toggle_visit.hpp   # present: every success line says it (R1)
grep -n 'toggleCamera' include/shulib/motion/motion_scheduler.hpp    # A2: exactly the shadow context's one line
grep -rnoE 'HA-(23[1-6]|24[3-5])' include src test | sort -u          # every id has its register row, and every row a code site
grep -rn 'PROVISIONAL (A4' include/shulib | grep -v 'HA-[0-9]'       # empty
grep -rln 'internal/\|chunks/\|RESUMING\|build-order' docs/*.md docs/guide docs/cookbook README.md   # empty
```
Spot checks, by hand: **A1** — rule 5's "every push paired" in the source (one place commands `back`, after ANY
verdict) and re-run **MA6** (the after-look from before the retract: T3 red — P2's stale "sure") and **MA11** (`need(1)`
for push 1), and *(re-check)* **MA24** (the blocking drive: T3 red — P5(a)), **MA9** (overlap: T5 red — a robot 8 in
too far), **MA23** (a short stroke reported `Flipped`: T14 red); **A2** — re-run **M-A2-1** (the shadow line) and a
refused tracker block that leaves the robot driving.

---

## 8. Landmines
1. **The camera can be frozen and look perfect while parked** (P2; `toggle_tracker.hpp` :47-52). Never accept the
   after-look from a parked robot, never count a read from before the retract (MA6, MA7). Never remove the back-off
   "to save time".
2. **The look spot is not the push spot** (P1). `pushFrom == lookFrom` — or any excursion of twice `frozenMoveIn` or
   less — is REFUSED by the request *(re-check R4: the first pass said it gave `UnknownBefore`; at a high tilted lens
   it would instead never reach NO CHANGE)* — the guide says why.
3. **Square matters** (P1, VIS1's ColourAbove rule): turned 5° at 12–14 in, the colour is UNKNOWN at every mount but
   the 12-in tilted one; at lenses 6 and 8 in up even 3° loses a distance *(re-check R11)*. The route must
   end its approach square (`face`/`turnTo`), especially after a CURVE1 curve.
4. **"Ours" is per Toggle in Skills, not per alliance.** Never let a Match "ours" leak in (ALLY1 is after the 21st);
   the step refuses a Match auton.
5. **Two pushes through the other colour** (HA-231): in Skills a cut between them costs nothing (D3), but it costs
   ~2–3 s more per Toggle. Budget it.
6. **A stuck flipper must not be dragged** (rule 5): the step does not back out with the flipper out; the recipe
   shows the recovery step (`back` again with a longer bound, then back off).
7. **The run's end safes, it does not retract** (HA-234): rule 4 is the only thing keeping the flipper off the
   Toggle at 0:00. Never weaken `need(n)` to "typical" times — bounds only.
8. **`then()` drops a verdict if the lambda returns nothing** (`routine.hpp` :483-487): the recipe ALWAYS returns
   `.outcome()`; a `void` lambda "succeeds" whatever happened.
9. **The service ticks only inside a shulib wait** (`routine.hpp` :554-558): every wait in the step is
   `guard.waitFor` — a `pros::delay` would freeze the flipper at its last command.
10. **The shadow context** (L6's landmine): AF1S-A2's slot must reach `motion_scheduler.hpp`'s shadow context, or
    every everyday call says `NoCamera` (M-A2-1).
11. **A route that names the wrong Toggle under a RELATIVE start flips it to the wrong colour, said nowhere but in
    the line's `wall not checked`.** The recipe uses a FIELD start (D6).
12. **`<SG6>`**: the robot holding more than one Pin or one Cup may not interact with a Toggle — a violation the
    library cannot see. The recipe's order of steps respects it.
13. **The roll direction, the seat time, the stall's force — all unmeasured** (HA-231/232/235). The step is safe
    whichever way they come out; only the time budget changes.
14. **VIS1-A2's names may change before it merges.** A1 re-checks every name it uses at VIS1-A2's merge (§0.2's
    list is the contract).
15. **G1 is the real critical path.** A finished AF1S on a robot that cannot be built is worth nothing on the 21st.
16. *(re-check R1)* **"FLIPPED" is not "set".** The camera cannot see a Toggle stuck part-way; never word a report,
    a guide line or a status line as if the step confirms the Toggle is set.
17. *(re-check R3)* **Never drive inside the step with a blocking `Chassis` verb** — the camera goes unread and a
    frozen camera reads "sure" (MA24). Every motion is `async` + `guard.waitFor` with the feed ticked in the
    predicate, cancelled on every exit.
18. *(re-check R2)* **The spot check is containment, never overlap**, and it is coarse: never present it to the team as
    what makes the push land.

---

## 9. Summary

**What AF1-S is:** one route step for a skills auton — `competition::checkAndFlipToggle(ctx, {...})`, run through
`r.then(...)` — that, at a look spot the route parked the robot on, square in front of a named Toggle: checks it is a
Skills auton, the camera and flipper are ready and (with a field start) the robot faces that Toggle's wall; LOOKS with
the front AI Vision camera (VIS1-A2's tracker: a colour only when sure); does NOTHING if the Toggle already shows its
Quadrant's colour (the skills "ours", `<RSC3>`); checks the run guard's time covers the whole visit; drives straight in,
pushes, retracts (every push paired), drives straight out, LOOKS AGAIN while moving and only after the retract; pushes
a second time only if the first turned it to the third colour; never a third. Every doubt — UNKNOWN, a stale or frozen
camera, two colours, not at the spot, the wrong wall, not enough time, a stuck flipper — does nothing more and is
reported in one `[TGL]` line; the route's `onFail` (STEP1-A) decides what comes next; the period's end cuts it as
CANCELLED. It serves a motor arm, a pneumatic and a passive flipper. No Routine change, no mechanism change, no
version bump.

**The decisions (each with a recommendation; re-checked late 11-10 — changed ones marked):** **D1** one call through
`then()`, returning a report and an outcome (not a Routine member; not a look-only call) · **D2** *(changed, R2/R4)*
look from a spot ~14–24 in back, square; the step drives in and out itself (look and push from ONE spot is not
supported); "at the spot" checked by the camera by CONTAINMENT, `offBy` the team's from Step 0.4 (no default) — a
COARSE check: where the flipper lands is the route's driving · **D3** "ours" = each Toggle's Quadrant colour, from one
season table, the team names the Toggle — **builder: confirm which side wall is red (Step 0.7)** · **D4** UNKNOWN does
nothing; onFail decides · **D5** always look again; a second push only on the third colour; never a third; no automatic
retry · **D6** *(sign fixed, R6)* works from any start; a FIELD start adds the wall check and is recommended for the
skills route; the position gate recommended after a wall reset (R5) · **D7** motor, pneumatic and passive flippers —
**builder: which is robot one's (Step 0.1)?** · **D8** *(changed, R10)* two runs (A1 the step, A2 the robot's camera
for autons) — **but for the 21st the PLAN is the team's blind flip** written with today's steps; G1's brief goes ahead
of both AF1S runs · **D9** *(changed, R10/R12)* robot one's build (G1) FIRST — it needs a BRIEF first — then the
cascade's declaration (no new library word; its homing one of three words), then AF1S — **you: which robot runs the
skills route; a go for G1's brief ahead of the RPV1 brief; is there a bottom limit switch on the cascade?** · **D10**
reuse five of AF1's HA ids, three new (HA-243..245) · **D11** *(NEW, R1)* the flipper acts on the camera's word alone
(your 11-03 ruling — confirm it for this step); the camera cannot see whether a Toggle is SEATED, so "FLIPPED" means
"shows our colour after a full stroke", every success line says `seating not checked`, and a short stroke that then
reads ours FAILS.

**The coverage check, in one line:** every mechanism kind the team named — the cascade (two+ motors, stall homing,
levels), the flipper, claws, rollers, pneumatics, a passive toggle — can be declared, wired, driven from the controller
and stepped in an auton with what is on main today, **except the camera-checked flip (this brief)**; **none of it can
exist on robot one until robot one has a build (G1, ~1½–2½ builder-days, never briefed)**; the cascade's multiplied
travel is a two-point `measured({...})` shape, its levels are presets
(six named) or constants (any number); its homing is one of three words — `homeBySwitch` if a bottom switch is fitted, else `homeByStall` + the route's first `startHome`, or `restsAtStop` (ready at boot, never re-homes) *(re-check R12)*; the current cap (MECH-S-A) is DONE and merging after L6-A2, live tuning (TUNE1-A) is building *(re-check R10)*.

**Sizes (builder-days, rough, none measured):** **AF1S-A1 ~2¾–3¼** *(re-check: +¼ for R1's full-stroke rule, its
plant mode and T14, R3's async drive and MA24, R4/R8's after rule)* **· AF1S-A2 ~1¼–1½ — ~4–4¾** (~6–7 slot-days with
verification); **G1 ~1½–2½** (its own chunk; a brief and a check first, ~½ slot-day); the team's route an evening.

**Honest dates** *(re-written at re-check R10)* (today Tue 2026-11-10, late; robot one built ~Sat Nov 14; code
freeze ~Wed Nov 18; competition Sat Nov 21; one builder-day per slot per calendar day, verification half again — the
plan's own arithmetic, never measured). **All five slots are full tonight** (TUNE1-A, STEP1-A, CAL1-B, this brief,
VIS1-A2's fourth run) and two briefs are queued ahead of anything new (CURVE1, then RPV1). Best case:
- **~Nov 11:** this brief's slot goes to the CURVE1 brief; the next slot to free (STEP1-A, CAL1-B or TUNE1-A, each in
  its fix or verify round) takes the **G1 brief** (~½ slot-day with its check), your go ~Nov 11–12.
- **G1** (~2¼–3¾ slot-days with verification) waits for CAL1-B's merge (the same files) → **merges ~Nov 14–16**,
  robot one's facts typed in as they arrive (~Nov 14).
- **AF1S-A1** (~4–5 slot-days) needs a further free slot and your go on this brief → at the earliest ~Nov 12 →
  **merges ~Nov 16–17**. **AF1S-A2** (~2–2¼) only after A1, L6-A2 and VIS1-A2 have all merged → **~Nov 18–19**.
- Then Step 0 on robot one with the camera (station 12 reaches robot one only after G1), the team's route, and a first
  checked flip on a real Toggle: **~Nov 19–20 at best — after the ~Nov 18 freeze.** With the usage stops of the last
  two weeks (one job at a time twice since 11-03), later still.
- **Said plainly: the camera-checked step will almost certainly NOT be ready to use on the 21st.** "VIS1-A2 near its
  end tonight" was a hope, not a fact (it is on its fourth run).
- **So the recommendation for the 21st:** the team writes the **blind flip** into the skills route with today's steps
  (`r.move(flipper["flip"], …)` then `r.move(flipper["stow"], …)`, once or twice per Toggle as Step 0.1 shows), which
  is honest in Skills because every Toggle starts yellow and only our robots touch it; **G1 gets the next brief slot
  after CURVE1's** (it blocks every mechanism on robot one, the cascade first); AF1S-A1 is built when a slot is free
  that nothing on the 21st's path needs — it then serves robot one after the 21st and is the base AF1 builds on.

**Probes** (`.claude/coordinator/scratch/brief-AF1S/`): P1 the look spot (12–24 in, square within ~3°, never a wrong
colour); P2 a frozen camera is invisible parked and caught moving; 126 colour-pair cells, 0 wrong; P3 the rule over 84
skills cases — the rule given correct colours (0 pushes on ours, 0 third pushes, 0 worse endings); P4 the route's form
compiles against main's `then()`; *(re-check)* P5/P6 the checker's (re-run identical): a stuck Toggle reads its
destination colour (58 of 108), a camera fed only parked reads stale "sure", the range interval is 5–18 in wide; P7/P7b
containment vs overlap (overlap accepted a robot 8–11 in too far); P8 the new after rule (0 wrong of 60, NO CHANGE
reachable 5 of 5).

**UNVERIFIED (each a §4 entry, a Step 0 question, or a coordination fact to re-check):**
- which side wall is a RED Quadrant on the event's field (HA-243 — read from Figure Q-1 and pA15);
- what robot one's flipper is, its numbers, the roll direction and seat time (HA-231/232/235/236) — no flipper exists;
- whether the camera names the colour from the look spot on a real field (HA-245 — P1 is the host camera model, with
  no wall and no lighting);
- the in-and-out drive's repeatability and `offBy` (HA-244) — and whether the camera's range on robot one's real
  mount tells a robot 3 in off from one on the spot at all (P7b: not at the host model's 12-in lens);
- *(re-check R1)* whether one full stroke always SEATS a Toggle (HA-232) — the camera cannot check it on the field;
- *(re-check R8)* the camera's image age (HA-192 — `maxImageAge`, unmeasured);
- *(re-check R3/R4)* the after rule (P8) and the async drive are host-test-plant results on VIS1-A2's in-flight
  tracker; P8 freezes the camera for good — a camera that freezes and recovers is the tracker's own tested path;
- every time in the plain-words section (guesses; AF1's 40-rpm test arm is the only number);
- VIS1-A2's final API (in flight), STEP1-A's final onFail forms (in flight), L6-A2's final slot pattern (staged);
- whether robot one runs the skills route alone, robot two rebuilt, or both (D9) — and G1's size;
- whether v2.2 (2026-12-03) changes anything — after the 21st;
- **nothing in AF1-S has run on a robot; nothing in robot one's mechanisms exists in code.**
