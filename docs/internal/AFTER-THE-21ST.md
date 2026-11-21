# After the 21st — everything left after the first competition

> Compiled by the coordinator, 2026-11-20 ~16:30 EST, on the team lead's question: "can you tell me everything that
> needs to be done post comp?". Read from the coordinator's NOTES (`.claude/coordinator/NOTES.md`, **N:** below),
> the briefs (`.claude/coordinator/briefs/`, **BR/**), this folder's plan files, every open chunk's `-PROGRESS.md`
> (**CH/** = `docs/internal/chunks/`) and the coordinator's memory of the team lead's rulings. Every item has a
> source; the surprising ones were re-read by the coordinator before this was written.
> [`REMAINING-WORK.md`](REMAINING-WORK.md) (last full update 2026-10-22) and [`build-order.md`](build-order.md) do not
> carry November's chunks (RMP1, RPV1, ROB1, CAL2, S13UX, SETUP1) yet: **this file is the current list** until they
> are brought up to date (item 10.4).
>
> Items still running on 2026-11-20 (RMP1-L names, RMP1-M Mechanisms tab, RMP1-N camera view, the camera test, a
> run with Watch the robot, a Timed Run) move here only if they do not finish before the competition.

## The order (the coordinator's recommendation, for the team lead to rule)

1. **Mechanisms made easy (SETUP1 + add/remove).** One setup program in place of Bench Tests, Calibrate and station 13
   that measures each mechanism and saves it; the route builder's Mechanisms tab then adds and removes mechanisms
   with no C++ typed. The team lead's most repeated ask (11-20: "we really need to make the mechanism stuff easier").
2. **The camera, used.** The camera's data first (the test on White Bot), then AprilTags (see, drive to), the Pin
   intake check, and camera-corrected position.
3. **The cascade's levels and its heights re-taped.** The builder's "MOST IMPORTANT" (11-05); White Bot's cascade
   has no levels yet.
4. **Black Bot ready.** Its 5 owed wiring facts, its own Calibrate run and its mechanism setup (or SETUP1's).
5. **Head-to-head matches:** our alliance colour (ALLY1), driving to a piece (L4), red/blue route pairs.
6. **The full auto-flipper (AF1).**
7. **The names everywhere, the docs and the videos:** White Bot / Black Bot on the brain screens and in the docs;
   the docs redo (DOCS3); the two videos re-recorded; team files' comments cut (READ1).
8. **The robot panel (G3a)** — its shell undecided (it may fold into the route builder's web app).
9. **Library leftovers**, robot checks owed and decisions owed (sections 7–9).

## 1. Setup and mechanisms

| Item | State | Ruled | Source |
|---|---|---|---|
| **SETUP1** — one guided setup program (Bench Tests + Calibrate + station 13); mechanism records on the SD card read at boot; the route builder reads them; Calibrate's annoying steps removed | sketch brief (D1–D6); full brief not written; first ask the team which steps annoy them and time a setup as it is | the team lead 11-20: "Yes, after the 21st"; "one calibrate that does everything" | BR/SETUP1-guided-setup.md:3-14, :26-51; N:1262, :1285 |
| Fold the Auton Tester into Competition | to weigh in SETUP1's full brief | — | BR/SETUP1:13-14 |
| **Add and remove mechanisms from the route builder** (ports, kinds, limits; a new robot) | not started; over G3a Part 1's robot file and generator (built, unmerged, worktree `g3a-p1`) | RMP1-M D4: after the 21st, with SETUP1 | BR/RMP1-M-mechanisms-tab.md:42-43; N:1061 |
| **Cascade preset levels** (heights per stack) | none typed: the cascade is manual | the builder 11-05: "MOST IMPORTANT" | src/robots/robot_one_facts.hpp:537-541; N:1050, :1080 |
| **Re-check the cascade's height rows** — 11-17's tape: 20 in at 3,759° above the bottom stop. A trace on 11-20 read 11.6 in at the top stop, but that run's homing still pushed the wrong way, so the rows were left as taped. Run it fully up in Drive: the `[CAS]` log says its height at the top stop | open | — | src/robots/robot_one_facts.hpp:530-534; N:1296-1297 |
| MECH "the rest": protection and more kinds (~7–11½ d), driver and auton extras (~5½–9 d), setup and diagnostics (~6½–11½ d), their panel pages (~2–3½ d) | not started | the team lead 09-28: after the panel | REMAINING-WORK.md:80-83; MECH-PANEL-requirements.md:566, :773-780 |
| MECH-S-B — the string slack/snap watch (B1 ~2¾–3¾ d, B2 ~1¼–1¾ d); amend for a lift strung both ways first | not built | the team lead 11-05: kept, after the 21st (the builder had said "ignore snapping") | N:927, :947-948; CH/MECH-S-PROGRESS.md:3 |
| TUNE1-B2 — Tune copies its numbers back to the robot file (~1¼–1¾ d) | not built; needs G3a Part 5/6 | after the panel | CH/TUNE1-PROGRESS.md:3 |
| A distance sensor as the lift's height | skipped for the 21st; not re-placed | — | coordinator memory (first competition) |

## 2. The camera (AI Vision, White Bot port 7)

| Item | State | Ruled | Source |
|---|---|---|---|
| **The camera test** — Drive's `[CAM]` lines while pointing it at an AprilTag, a Pin, a Pin in the intake, the Toggles; find the AprilTag family (Bench Tests station 12 switches families) | Drive prints them since 13:50 11-20; no line read yet | today if the robot is plugged in | N:1298 |
| Type White Bot's camera port **7** (the facts still say owed, placeholder 3) and its mount; Black Bot's too | owed (nothing reads it yet: Drive finds the camera on any port) | — | src/robots/robot_one_facts.hpp:658-661; src/robots/robot_two_facts.hpp:238-241 |
| **AprilTags: see them and drive to them** (R2b's AprilTag part ~1 d rough; TagMap's duplicate-id fix) | not built | the team lead 11-20; "not for tomorrow" | N:1298; REMAINING-WORK.md:125; HANDOFF-remaining-work.md:441-460 |
| **Pins: see them; check what is in the intake** | not built | the team lead 11-20 | N:1298 |
| Camera-corrected position: the AprilTag corrector, absolute references on a field (R3c; the GPS strip optional per event) | plan items | — | REMAINING-WORK.md:156; HANDOFF-remaining-work.md:792 |
| The camera view in the route builder (RMP1-N, building 11-20); the full live camera page (picture + overlay) with the panel | RMP1-N in flight | — | BR/RMP1-N-camera-view.md; N:999-1005 |
| VIS1-B — the robots' camera block and Bench Tests station 12 (~1¾–2¼ d), re-targeted from the old robot two; VIS1 Step 0 at the robot (mount, model, colours) | not built | after the 21st | CH/VIS1-PROGRESS.md:3; REMAINING-WORK.md:195-198 |
| L4 — drive to a game piece (L4-A ~3¼–4¼ d, L4-B ~1½–2 d) | brief drafted with ALLY1, never answered | deferred 11-05 | BR/ALLY1-L4-alliance-colour-drive-to-piece.md:3-8, :115-116 |

## 3. Autons, matches and motion

| Item | State | Source |
|---|---|---|
| **ALLY1 — our alliance colour** (A ~1¾–2¼ d, B ~1½–2 d); red/blue head-to-head route pairs | brief never answered; OUT for the 21st (skills only) | N:956; BR/ALLY1-L4:115; BR/RMP1-route-maker-page.md:1127, :1447 |
| **AF1 — the full driver-control auto-flipper** (A–D ~8½–11 d + D ~1–1¼ d); AF1S's rest (White Bot's Toggle block, Station T, a real Toggle flipped in skills; which wall's Toggle is red — the builder) | answered 11-03; moved after the 21st 11-10 (the brief's header still says DRAFT: stale) | N:934, :958, :963; CH/AF1S-PROGRESS.md:3 |
| **The team's v3 skills route into the repo** (today only in the `jal-test` worktree) | waits on "both robots at the same time?" | N:1150, :1291 |
| **The claw** — "the claw wasn't working" in the route on 11-19; the debug (Drive's A, air, port A, active level) never recorded an outcome | **check tonight** (driver skills may use it) | N:1208-1213 |
| White Bot's port 11 gear slips (~10 % less travel on that motor) | the builders | N:1124, :1128, :1214 |
| CURVE1-B — routes as data (~¾–1¼ d); Station C (the curve session) | after the 21st | CH/CURVE1-PROGRESS.md:3 |
| MIRROR1 — both robots run the mirrored route together (never yet); the toggle-side swap question | open | build-order.md:1797; N:1097, :1104 |
| L6-B (side sensors + the WALL page ~1½–2 d), L6-C (~½–¾ d) — written for the old robot two; White Bot declares no wall sensors | re-target first | CH/L6-PROGRESS.md:3 |
| RPV1 A1 (~1 d), A2 (~½ d, may be covered by RMP1's step attribution), B2 (~1 d) | — | CH/RPV1-PROGRESS.md:3 |
| L9 A2 (~¼–½ d), B (~1 d) | — | CH/L9-PROGRESS.md |
| T3 — chassis macros and angle presets (~½–1 d; proposed into MECH's driver extras); T2 close-out (~½ d) | — | REMAINING-WORK.md:94, :167 |
| R5a B, C, D (~2½–3 d + robot sessions) and its five decisions; SAFE-SAVE B2 (~½ d) | — | REMAINING-WORK.md:138, :141, :219 |
| Replay a real run in PREVIEW (~1 d; Watch the robot covers part); an overhead-camera tracking rig (~3–5 d) | the team lead 11-13: "another day after the 21st" | N:994 |

## 4. Tools: the route builder, VS Code, the panel

| Item | State | Source |
|---|---|---|
| The route builder's Route tab, the rest (RMP1-A1b: partway triggers, wall-reset and Toggle steps, the mirror band, the time bar; ~2–3 d) | not built | BR/RMP1-route-maker-page.md:515, :557 |
| The route builder polish run (usability session, motion, accessibility, live preview while editing; ~2–2½ d) | after the 21st | BR/RMP1-route-maker-page.md:517, :1673-1679 |
| The Controls tab one step at a time | proposed | N:1178 |
| Small follow-ups: RMP1-H's four PREVIEW fixtures + two pictures; RMP1-I's `send-ok.png`, the "line missing" check, the Windows CI rows, the VS Code task on the team laptop; RMP1-E's "keep going" highlight | owed | CH/RMP1-PROGRESS.md:3254-3258, :4440-4446, :4479-4482, :4642-4644, :4925-4928 |
| **The robot panel (G3a)**: Part 1 built, unmerged (`g3a-p1`, 4691757); Parts 2–7, T, S, AF to build; the shell (VS Code extension, the route builder's web app + local helper, or desktop) to decide | OUT for the 21st (11-10) | N:955, :998, :1028-1029, :1061; build-order.md:1946-1953 |
| RMP1-P's follow-ups (Leave gently): on other verbs (D4); a B Back up backing out straight BEFORE its turn, or turning gently — the team lead's to rule; an arm's or lift's releasing position (the library marks only a piston's `grabs` / `releases`); 5 in/s and the 0.5 s time margin never watched on White Bot; the WebAssembly engine tests' memory cap (they fail under `ulimit -v` 4 GiB: run them in a `systemd-run` scope) and RMP1-E's preview fixture re-recorded (stale since 11992a7) | owed | CH/RMP1-P-leave-gently.md (D4); CH/RMP1-PROGRESS.md, "RMP1-P review findings and their disposition" |
| The store listing (Marketplace + Open VSX, publisher "shulib") — only if the shell is a VS Code extension; the hosting decision | after the shell | N:834-836, :1028-1030 |
| H1 A2 (~1 d), B (~½ d); H4 the robot console; G3 `.vexbot` (~2½–8 d); a starter `.vexbot` (~1–2 d); PKG1, G4 last | — | REMAINING-WORK.md:117-119, :140, :174-175 |

## 5. The robots

| Item | State | Source |
|---|---|---|
| **Black Bot: 5 wiring facts owed** (both toggle directions, the claw's, flipper's and ramp's levels) — its programs refuse to build until typed; its camera port and mount | the builders' answers | src/robots/robot_two_facts.hpp:204-216, :238-241 |
| Black Bot: its own Calibrate run (its drive table is UNSET), station 13, Station R2 | owed | CH/CAL2-COMPLETED.md:34; src/robots/chassis_table.hpp:112-124 |
| White Bot: the second Calibrate run's SAVED photo; a run to the save on the brain's stack (CAL2) | owed | CH/CAL2-COMPLETED.md:33; N:1207, :1213 |
| White Bot: station 13's cap "NOT SET" line never seen on the robot; `ProsDeviceProvider`'s 14,696 B on Calibrate's stack | owed | CH/S13UX-COMPLETED.md:22; CH/CAL2-COMPLETED.md:35 |

## 6. Docs and videos

| Item | State | Source |
|---|---|---|
| **White Bot / Black Bot on the brain screens, the C++ names and the docs** | after the 21st (the team lead's "What people see, today" covered the route builder, VS Code, the upload words) | BR/RMP1-L-white-bot-black-bot.md:3-6 |
| **DOCS3 — the docs redo**: N2, P1a (guide 18 split first), A1 (the beginner track), P1b-1/2, P1c-1/2, A2, P3a/b/c — ~33–37 days + ~10 review in all; visuals required, no disclaimers | not started | CH/DOCS3-PROGRESS.md:453-495 |
| Re-record the Controls video (c15–c17) and the make-a-route video (m23, m29, m30) | owed | N:1259, :1267; CH/RMP1-PROGRESS.md:4851, :4925 |
| READ1 — cut team files' comments (src/team, src/robots, examples) to a short header + code | never ran | N:1053-1057, :1155 |
| "Never driven a robot under its own control" — README.md:87 and guide 14:16 still say it; White Bot ran 8 of 8 on 11-17 | owed | N:1137 |
| A doc check refusing `../assets/` in raw HTML under docs/guide | TODO | N:1238 |

## 7. Library chunks still open (the briefing's list)

AF1S (White Bot's Toggle block, Step 0) · CURVE1 (B, Station C) · DOCS3 (11 runs) · H1 (A2, B) · L6 (B, C) · L9 (A2,
B) · MECH-S (B1, B2) · MIRROR1 (the both-robots run) · R3a (the register's groups at the robot) · R3b (the DoD:
White Bot's 11-17 runs may satisfy part; the judgement is owed) · R3d (White Bot's three Calibrate runs agreed within
0.05 %; its completion record is owed) · R5a (B, C, D) · RMP1 (A1b, polish, follow-ups) · ROB1 (facts as they arrive)
· RPV1 (A1, A2, B2) · SAFE-SAVE (B2) · STEP1 (a robot run + its record) · T2 (close-out) · T3 (macros) · TUNE1 (B2) ·
VIS1 (B). Sources: docs/internal/PROJECT-BRIEFING.md:112 and each chunk's progress file.

## 8. Robot checks owed

- RMP1-K's six real-brain checks: the user-port terminal (Linux, Windows), `pros user-script tools/brain_slots.py`
  (pros-cli 3.5.6 and the extension's frozen copy), `--description` on a real upload, the Windows `pros lsusb`
  parse, the step lines over real serial, one real Check on White Bot (CH/RMP1-PROGRESS.md:5625-5631).
- A Timed Run on shulib Competition (slot 2): Station K, HA-162; then ruling B6 (REMAINING-WORK.md:46, :214).
- Worksheet stations never formally run: N (IMU timing), H and I (distance sensors, camera), T (Toggles), K.7.3; M
  and F largely overtaken by White Bot's 11-17..20 sessions, J's claws belonged to the retired robot two.
- Robot sessions S2–S9, then re-fitting the host test plant (P1) and settling the register (P2: 7 of 227 settled).

## 9. Decisions owed by the team lead

MIRROR1-B delta 4 (N:1097) · the mirrored toggle-side swap (N:1104) · AF1S-A2 finding 1 (CH/AF1S-PROGRESS.md:799) ·
L9's "turn first: face()" warning · R5a's five (REMAINING-WORK.md:219) · B6 (:214) · Q6, the `MotionDeps` version
(HANDOFF-remaining-work.md:738) · the rest of Q2, sensor placement (:726) · the freeze line for MECH's auton steps
(N:773-776) · COMP-b's routine-stop line, 214 B against 200 (N:854-855) · the panel's shell (item 4) · SETUP1's
full brief.

## 10. Infrastructure and the plan

1. Make the tests clean on GCC 15, then drop CI's ubuntu-24.04 pin (N:1023).
2. Move the pages workflow's actions off Node 20 (N:1012).
3. Builders as `claude -p` on the Console key, with the team lead's OK (N:1078).
4. Bring `REMAINING-WORK.md` and `build-order.md` up to date: November's chunks, the 11-10 "no panel, skills only"
   ruling, AF1's answers, stale between-parts markers (MIRROR1, RMP1), stale "nothing has run on a robot" lines.

## Ruled out or declined (not pending)

"shulib Live" (no new brain programs, N:1285) · CURVE1's back-up-and-re-approach (N:968) · G4's `.shupaths` importer
· an Optical sensor for AF1 (camera only, N:934) · the 2.0 V kS drive test (reverted, N:1215).
