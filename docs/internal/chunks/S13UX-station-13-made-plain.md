# S13UX — Station 13, made plain (robot two measures its mechanisms without the coordinator)

> Brief by the coordinator, 2026-11-19 ~16:10. **The team lead's go, 2026-11-19: "put all resources into getting
> everything done"** — he has handed robot two to the builders ("i will leave everything to the builders"). One run,
> **S13UX-A**. Program code only: `src/programs/bench_r3a.{hpp,cpp}` (Bench Tests) — the library (`include/`) is FROZEN
> since Wed Nov 18. Competition: Saturday Nov 21. Base: main `ee5a1f5`.

## 1. Why (robot one, 2026-11-17 — the team lead at station 13; log `.claude/coordinator/robot_logs/robot_one_2026-11-17.log`)

Robot two must go through station 13 before its Drive, Auton Tester, Competition and Tune can build: each mechanism's
direction check, the cascade's bottom-stop and top-stop RECORDs (Stage B rows), which way each side toggle runs, which
line level opens each piston. The builders will do it ALONE on Friday. On robot one the team lead said: "literally
nothing worked", "I'm confused what it's asking me", "the screen is confusing", "we need to restart this it is so
confusing". What actually went wrong, in order:

1. The controller showed FIELD CONNECTED (status 0x0f) while not linked to the brain — every press did nothing; re-pairing
   fixed it. The station never said the controller was the problem.
2. Nothing moves unless **L1 is held** (the dead-man), and UP/DOWN jog — not said where the eye is.
3. The ceiling starts at **3 V**; R1 raises it (6/9/12 V). At 3 V the cascade barely lifts. Guide 21 says "never press R1
   on the cascade" because **station 13 does not apply the cascade's current cap** (`kCascadeCapAmps`, 2.0 A).
4. A jog is cut at **3 s**; after ANY cut the operator had to leave the station and re-enter — nobody knew.
5. The panel's lines are engineering words ("Six gates", "HA-182", "INVENTED", "did-not-move 0.250 rad").
6. RECORD writes every mechanism's lines to serial; the number the coordinator types (the cascade's `output … deg`)
   is not shown where the operator looks.

## 2. Decisions

- **D1 — Plain words on the panel.** Every line station 13 paints is an instruction or a reading a student understands.
  Engineering detail (HA numbers, "INVENTED", thresholds, radians) goes to the serial/SD log only — never removed from
  the log. Every painted line inside the existing budgets (LARGE 18 / MEDIUM 36 / SMALL 50) and the paint-only-what-
  changed rule (PROS leaks every print — `g_screenPrints`).
- **D2 — One "what to do now" line, always,** derived from the station's state — e.g. controller not connected → "Pair
  the controller to the brain (it shows no link)"; the controller reports a field/competition link → "Controller says
  FIELD -- unplug the field cable or re-pair"; L1 not held → "Hold L1 to power the cascade"; L1 held → "UP raises, DOWN
  lowers the cascade (L1 held)"; jogging → "Moving UP -- 1.8 s of 3 s"; a jog that did not move → "It did not move at
  3 V -- press R1 for 6 V" (subject to D5); a cut → what cut it and the ONE action that continues (D4). Find how the
  station reads the controller's link and the competition status today (station 10's and Drive's code do); a state the
  code cannot tell apart is said as what it can tell.
- **D3 — A lift's measuring guide on its page.** For a `Lift`, the reading lines show the steps of guide 21's "The order
  on the robot" step 3, numbered, the current one marked: bottom stop → RECORD; each level (when any) → tape → RECORD; the
  top stop (D5's rule) → RECORD. After a RECORD the panel shows, large, what was recorded for the selected mechanism in
  the words the coordinator types from (e.g. "RECORD 2: cascade output 3759.0 deg") — the serial line format unchanged.
  Other kinds (toggles, pistons) get one plain line each saying what to do on them (run briefly / which way; fire / which
  position).
- **D4 — A jog-time cut is not a fault.** The 3 s jog cap's cut re-arms when L1 is RELEASED and HELD again (a
  deliberate act), and the panel says exactly that. A FAULT cut (two motors fighting, 2.4 A for 250 ms, the stall
  signature) keeps today's rule; the panel says it plainly with the one way on (today: DONE, then open 13 again).
- **D5 — The declared current cap at station 13.** Apply each motor mechanism's DECLARED current cap (the cascade's
  `kCascadeCapAmps`, the toggles' `kSideToggleCapAmps`) while station 13 powers it, IF that is reachable from
  `src/` with no `include/` change (find how the competition/drive programs apply it — the mechanism parts or the
  service). Then R1 is safe on the cascade under its cap: update guide 21's "never press R1" note to the capped rule and
  keep "never power into the top stop" as "approach the top stop at 3–6 V, stop when it stops rising". **If D5 needs an
  `include/` change, do NOT make it: implement D1–D4, leave guide 21's note as it is, and report D5 as blocked by the
  freeze with the exact change it needs.**
- **D6 — Nothing else changes:** the six gates and their order, L1 as the dead-man, the cut thresholds, the RECORD
  serial lines (the coordinator types from them), the menu, every other station.

## 3. Tests (ADVERSARIAL — each seen RED against a mutant)

The station's pure parts live in `src/programs/bench_r3a.hpp` (`mech_station`) and are host-tested
(`test/bench_r3a_*` — find them). At least: D2's line for every state (no link, field link, L1 up, L1 held, jogging
with its seconds, did-not-move, each cut kind) — every line within 50 columns; D3's guide for a lift with no levels and
with levels, the current step advancing on RECORD, the large RECORD line's number equal to the serial line's; D4: a
jog-cap cut re-arms on L1 release + hold, a fault cut does NOT re-arm that way, a jog-cap cut with L1 never released does
not re-arm; D5 (if built): the cap applied to the cascade's and toggles' motors while powered, and absent mechanisms
untouched. Mutations (each RED, restored): D4's fault/jog distinction; the re-arm needing the release; D2's
field-link line; D3's number; D5's cap (if built). Gates: the suite through `python3 tools/suite_run.py green`, the src
build gate `check` AND `self-test` (under `timeout -s INT`), the doc gates.

## 4. Docs

Guide 21: "The order on the robot" step 3 and the station 13 notes (what the operator sees now), "What robot one does
not do yet" (D5's note); the Bench Tests worksheet / guide pages that describe station 13 (grep "station 13",
"13 MECHANISMS" over `docs/`); `docs/internal/chunks/S13UX-PROGRESS.md` (create it FIRST, `## S13UX-A`). Short and plain
(the team, 2026-11-15: "soooooo many comments").

## 5. Landmines

- `src/programs/bench_r3a.cpp` is 4,276 lines: read station 13's parts (from `runMechanisms` / `mech13::` and the live
  page loop ~line 3500 onward) and what they call — not the whole file.
- The src build gate's self-test plants faults INTO `src/` and restores them: never leave a plant (`git diff -- Makefile
  src/` for '(self-test plant'; `git ls-files -o --exclude-standard src include test`).
- In flight beside you (do not edit their files): RMP1-C (`tools/route_maker/`, `src/team/skills_route.*`,
  `src/team/*autons*`), CAL2-A (`include/shulib/calibration/`), the coordinator's robot-two facts
  (`src/robots/robot_two_facts.hpp`, `src/robots/chassis_table.hpp`).
- Never run pros or touch /dev/ttyACM* (the coordinator uploads).
