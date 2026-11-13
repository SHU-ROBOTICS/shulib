# CURVE1 — a TRUE CURVE FOLLOWER for the tank: the robot drives a smooth path through the author's points without stopping at them — pure pursuit on a smoothed path, a speed limit in the bends, an end that is an ordinary `moveTo`, guards that stop a curve the robot has lost, the verbs `curveTo` / `reverseCurveTo` and their Routine steps; F8's `"arc"` after the 21st

> **THE TEAM LEAD'S ANSWER (2026-11-11 ~02:00): "Go (Recommended)"** — D1–D13 as recommended after both re-checks: pure
> pursuit; `curveTo` / `reverseCurveTo`; the end hand-off to `MoveToPose`; the run-up rule (6 in from 2½ ft, 1 ft from 3 ft);
> the back-up-and-re-approach REJECTED (his "add both only if it really does make it better", 2026-11-10). CURVE1-A starts
> when STEP1-A merges and a slot frees; the skills route is written with today's steps first and legs switch to curves where
> the robot session passes.

> **Status 2026-11-10: DRAFT for the team lead's go, RE-CHECKED the same night (eleven findings, every one real —
> "Re-check — amendments" below; the body is corrected in place and each change is marked *(re-check An)*). WAITS for
> his answers to D1–D12 below, then for a go per run (CURVE1-A, then — after the 21st, by D10 — CURVE1-B). Nothing is
> built.** *(re-check A12: the trunk is now `16c01e0` (L6-A2); every cited line in this brief was re-opened there —
> only `motion_scheduler.hpp`'s moved, refreshed in §0.2.)* **AMENDED again late on 2026-11-10 — the sideways miss
> (the team lead's "add both only if it really does make it better"): the run-up rule ADDED (guide text, CV23, an
> optional robot check); the back-up-and-re-approach REJECTED on the numbers (§5). No decision changed; the size
> unchanged.** *(amended 2026-11-10 late)* **The amendment RE-CHECKED (five findings, all real — "Re-check of the
> amendment" B1–B5 at the end of the amendment section): the run-up rule is now TWO numbers that hold at 0.18 s of
> motor lag too — 2½ ft of run-up for a miss up to 6 in, 3 ft up to 1 ft; the back-up's rejection restated honestly
> (it lands exactly more often than turn-drive-turn; rejected for blind reversing and the freeze, not for success).**
> **STEP1-A is BUILT and waiting to merge, third in the queue (MECH-S-A → CAL1-B → STEP1-A, NOTES :965); the
> dates below still hold.** *(amendment re-check B5)*
> Execution brief written by the coordinator's brief writer from a READ-ONLY pass over `shulib-v2` at **`d684f34`**
> (IMU1 Part B; the main worktree carries L6-A2's squash STAGED and uncommitted — `git status` at the start of this
> pass — which touches `motion_scheduler.hpp`'s shadow context only among the files this brief cites). Worktrees read,
> never edited: **`step1-a`** (STEP1-A — in its fix round at that pass; DONE since, waiting to merge third, *amendment
> re-check B5* — `MotionOptions` gains `positionTolerance` /
> `headingTolerance`, `motion_config.hpp` gains `loosenedSettle`, `chassis.hpp` :167, :231, :264 there). In flight
> while this was written (`.claude/coordinator/NOTES.md` :955-962): TUNE1-A, STEP1-A, CAL1-B (done, merging after
> L6-A2 and MECH-S-A), the AF1-S brief, VIS1-A2; the RPV1 brief (the host route preview) is queued next, beside this
> one. **Line numbers are `d684f34`'s.**
>
> **The team lead's words this brief holds (BINDING).**
> - **2026-10-30, G3a's D16 — (c), "A TRUE CURVE FOLLOWER" (not the recommendation)**: "a pursuit- or
>   boomerang-style controller on a tank, a smooth path form, its end behaviour, the facade verb, the Routine step
>   and F8's `"arc"` (additive). It RE-OPENS C1's D12 ('never aim at the point') — by this answer, for the curve
>   follower only. ~3½–5½ builder-days (rough, no precedent here) plus robot tuning"
>   (`docs/internal/chunks/G3a-robot-profile-generator-panel.md` :6-11; its decision block R.7a :778-805).
> - **2026-11-03: BEFORE THE 21st**, ahead of the panel, within the usage limits — told curves would not exist by the
>   21st: "this is extremely important though can we not get this done??" → "Everything incl. flipper"
>   (`docs/internal/build-order.md` :2402, the deviations table).
> - **2026-11-05 (the team's builder, sent by the team lead): the first competition is NOV 21**; both robots
>   identical; one built "by the 14th"; robot test week Nov 14–21; **code changes stop ~Nov 18**; the order is the
>   cascade → the autons → the auto-flipper → CURVE1 (`build-order.md` :2401).
> - **2026-11-10: NO PANEL before the 21st; SKILLS ONLY at this competition; "Yes, keep curves"; routes are written
>   in CODE, and a host ROUTE PREVIEW (RPV1) draws them** (`NOTES.md` :955-957, :961: "when are we doing curves" put
>   this brief next in the queue). **Nothing here may depend on the panel.**
> - Both robots are **tanks** this season; robot two's drive is five coupled motors per side, BLUE cartridges, 2.75 in
>   wheels reported, track width UNSET (`src/robots/chassis_table.hpp` :92-137).
>
> **Executor model: `opus`. Do not commit. Live log `docs/internal/chunks/CURVE1-PROGRESS.md`, created FIRST** (a
> heading per run). Say "host test plant", never "simulator". **No hardware claim anywhere:** the library's motion
> stack has never driven a robot under its own control — Station M (the 40 in on the bench) has not run
> (`docs/internal/REMAINING-WORK.md` :176-181); TANK1's steering "has only been proven on the host test plant"
> (`docs/guide/14-what-it-cannot-do-yet.md` :443; `chassis.hpp` :126: "Nothing of it has run on a robot."). **Hardware Assumptions: HA-246..HA-250 ONLY** (§4; after STEP1's HA-216..218, MECH-S/TUNE1's
> HA-219..227, AF1's HA-228..236, ALLY1+L4's HA-237..242 and AF1-S's HA-243..245 — the last three blocks are held by
> briefs, not yet in the register's Held line, `docs/hardware-assumptions.md` :39; a sixth belief ⇒ stop and tell the
> coordinator; if RPV1's brief reserves first, a mechanical rename, labels CV-H1..H5 kept). **Memory rule
> (2026-10-15):** every binary built — probes, the suite, any test — runs as `(ulimit -v 4194304; timeout 300 …)`;
> the suite only through `python3 tools/suite_run.py`.
>
> **Sizes (rough; one precedent, TANK1, sized by analogy):** **CURVE1-A ~3–4 builder-days** (the follower, the path,
> the guards, the verbs, the Routine steps, the tests and documents); **CURVE1-B ~¾–1¼** (F8's `"arc"`, after the
> 21st by D10). **~3¾–5¼ in all** — inside D16's ~3½–5½. **Plus robot sessions no host test plant settles** (§2.0).
>
> **Is Nov 21 reachable? Honestly: the CODE, probably, if CURVE1-A starts by ~Nov 12–13 BESIDE STEP1-B (not after
> it); a TUNED curve on the robot, only just — Station M and Station C must BOTH be done on the robot by ~Nov 18 (the
> code freeze), a window of about Nov 17–18.** *(re-check A5.)* §9 gives the dates and the fallback the team should
> write anyway.

---

## Amendment 2026-11-10 (late) — the sideways miss

**The team lead's words (verbatim, 2026-11-10).** Told "a tank can't slide sideways: if the robot ends up a few
inches off to the side of a point, a drive can't fix that sideways error", he asked: **"what can we do to fix?"**
Told that a one-point `curveTo` arcs in onto the target's line (D4's hand-off only once within 0.3 in and 5°), and
offered two additions — (1) MEASURE the run-up a curve needs to remove a sideways miss; (2) a BACK-UP-AND-RE-APPROACH
option for a robot too close to arc in (back up a stated distance, curve in again, at most once, reported) — he
answered: **"add both only if it really does make it better"**.

**The outcome in one line: (1) is ADDED as evidence and a rule the team writes by (guide text, one test that pins
it, one optional robot check — no new library behaviour); (2) was prototyped and is REJECTED — it is no faster than
what the team already has, and it reverses blind for up to 3 ft.** *(Re-check B1/B2: the rule is now 2½ ft for a
miss up to 6 in, 3 ft up to 1 ft — it holds at 0.18 s of lag, which the first table did not test; the back-up DOES
land exactly more often than turn-drive-turn (100 vs 88 %) — it is rejected for its blind reversing and the freeze,
and for time against the fix the guide recommends, not because success was a tie.)* Every existing decision D1–D12 stands; the
CURVE1-A size is unchanged (the additions are ~⅛ day, inside ~3–4); nothing here can slip the build.

**The probes** (`.claude/coordinator/scratch/brief-CURVE1/amend/`, every binary under `ulimit -v 4 GiB` + `timeout`;
the tree untouched; `README.txt` there lists every file). `curve_core.hpp` = `recheck/rc.cpp`'s follower with the
R-A defaults built in (re-verified: `recheck_verify.txt` is byte-identical to `recheck/g15w12.txt`'s first cell),
the hand-off case recorded. `runup.cpp` drives, on the same host test plant (tank 12 in, the weak RIGHT side, motor
lag as `kA`), graded on the TRUE pose: **`curve`** (a one-point `curveTo` to the pose; refused by D3/A3 as specified),
**`moveTo`**, **`fmf`** (today's fix: `face(point)` + `driveTo(point)` + `turnTo(end heading)`, 3 s per step),
**`curveP1`** / **`curveP2H3`** (STEP1-A's `{.positionTolerance = 1_in}` and `{2_in, 3_deg}`, emulated through the
settle config as STEP1-A's D2 scales it), **`raPre`** (the re-approach decided at the START: too close for its
sideways miss ⇒ reverse straight along its own heading far enough for the run-up, then the curve, once),
**`raPost:R`** (the re-approach on FAILURE: the curve reaches its path's end not lined up, or its end `moveTo` has
not settled 1 s after the hand-off ⇒ reverse straight R in and curve again, once, only with ≥ 2.5 s left), and
**`revCurve:Df`** (the author's own version with today's verbs: `faceAway(spot)` + `reverseTo(spot)` + `curveTo`, the
spot D in back on the target's line). The target is (0, 0, 0°); the robot starts **D in back along the target's
line** (6, 9, 12, 15, 18, 21, 24, 30, 36), **S in to the side** (±1, 2, 4, 6, 9, 12 — both sides, because the right
side is the weak one), nose **0°, ±10°, ±20°** off the target's heading; weak ×1.00 / ×0.95 / ×0.91 (TANK1's 5–9 %)
and ×0.85, lag τ 0 and 0.12 s; a 6 s budget. The CLOSE matrix: D 2–6 in, S ±2–6 in. **4,320 runs per method on the
main matrix, 2,000 on the close one.** Files: `matrix1.sh` → `p1_*.txt`; `matrix2.sh` → `p2_*.txt`, `pc_*.txt`;
`matrix3.sh` (the author's version); `summarize1.py`, `summarize1b.py` → `p1_summary_*.txt`, `p1_need_*.txt`;
`compare.py` → `p2_compare.txt`, `pc_compare.txt`; `paired.py` → `paired.txt`; `truth.py` → `truth.txt`; `cv23.sh`.

**Probe 1 — the run-up a one-point curve needs** (`p1_need_weak5to9.txt`, `p1_need_weak15.txt`,
`p1_need_lookahead.txt`). The smallest D from which EVERY run settles (both sides, every heading in the row, every
cell named — ">36" = none in the matrix):

| Sideways miss S (in) → | 1 | 2 | 4 | 6 | 9 | 12 |
|---|---|---|---|---|---|---|
| nose square to the end heading — weak 0–9 %, lag 0/0.12 s | 12 | 18 | 21 | 24 | 30 | 30 |
| nose within 10° | 18 | 21 | 24 | 30 | 30 | 36 |
| nose within 20° — lag 0/0.12 s only (the first draft's "rule", ~~21/24/30/30/36/36~~ — SUPERSEDED, B1) | 21 | 24 | 30 | 30 | 36 | 36 |
| nose within 20°, a 15 % weaker side | 21 | 30 | 30 | 36 | 36 | >36 |
| nose within 20°, with `{.positionTolerance = 1_in}` | 15 | 18 | 18 | 18 | 21 | 21 |
| nose within 20°, look-ahead 6 in (C.2's "faster" step) | 15 | 18 | 21 | 21 | 21 | 18 |
| nose within 20°, look-ahead 10 in (C.2's "steadier" step) | 30 | 36 | 36 | >36 | >36 | >36 |

*(Re-check B1 — the 1 in grid, lag 0 / 0.06 / 0.12 / **0.18 s**, weaker side 0 / 3 / 5 / 7 / 9 %, nose every 5° to
±20°, D 18–40 in by 1 in then 42–60; `xcheck/fine_all.txt`, 145,800 runs; `fix2/need.py`, `fix2/need2.py`,
`fix2/variants.sh` → `fix2/v_*.txt`, D 8–48 by 1 in. The smallest D from which every run settles on the TRUE pose,
in inches:)*

| Sideways miss S (in) → | 1 | 2 | 4 | 6 | 9 | 12 |
|---|---|---|---|---|---|---|
| nose within 20°, lag 0–0.12 s | 21 | 23 | 27 | 29 | 32 | 34 |
| nose within 20°, lag **0.18 s** | 27 | 28 | 28 | 28 | 26 | 24 |
| nose square, lag 0–0.18 s | 18 | 18 | 20 | 25 | 28 | 30 |
| **nose within 20°, lag 0–0.18 s → THE RULE: 2½ ft (30 in) up to 6 in; 3 ft (36 in) up to 12 in** | **27** | **28** | **28** | **29** | **32** | **34** |
| the same, a 12–15 % weaker side (`xcheck/weak15_all.txt`) | 26 | 27 | 29 | 32 | 36 | 40 |
| the same, `{.positionTolerance = 1_in}` (graded within 1 in) | 14 | 16 | 17 | 19 | 20 | 23 |
| look-ahead 6 in, lag 0–0.12 s (**at 0.18 s it never settles from any D up to 60 in**) | 15 | 17 | 20 | 20 | 19 | 18 |
| look-ahead 10 in, lag 0–0.18 s | 33 | 33* | 35 | 37 | 40 | 43 |

\* one run (2 in, nose 20° away, 0.18 s) also timed out 0.50 in off from 5 ft back — at the arrival's edge.
The in-between grid starts at 18 in, so an "18" there means "18 or less"; the last three rows use `fix2/v_*.txt`
(D from 8 in, weaker side 0 / 5 / 9 %).

**Inside the two-number rule, 0 of 68,040 in-between-grid runs fail** (weaker side 0–9 %, lag 0–0.18 s;
`fix2/rule_new_fine.txt`); at a 12–15 % weaker side 16 of 20,412 end 0.50–0.52 in off (a hair outside the arrival).
The worst need, 29 in at 6 in and 34 in at 12 in, leaves a 1 in and a 2 in margin — host numbers, so the guide tells
the team to add a few inches where there is room. The first draft's 2 ft step failed 149 in-between runs at 0.18 s
(misses of 2 in or less, the nose 15–20° turned away from the line, 21–27 in back, all timing out 0.51–0.63 in off).

- **Closer than the rule, the curve misses — but safely.** It hands over off the end's line, TANK1's short leg cannot
  remove the rest (L2), and the step times out at its budget — **every such run uses its WHOLE `.timeout`**: never a
  guard trip; **from 6 in or more back** the robot ends **≤ 3.5 in** off (3.7 in at 15 %; median 1.9 in from 6 in
  back, 1.1 in from 12 in, 0.7 in from 18 in, ≤ 0.6 in from 24 in; `fix2/inside.txt`). **From 2–6 in out, a
  curve fixes nothing: 0 of 1,500** (`pc_compare.txt`) **and ends about as far off as it started to the side — up to
  4.1 in (4.2 in at 15 %); 206 of the 1,984 driven close runs end further from the point than their sideways start**
  (e.g. 3 in back, 4 in to the side → 4.17 in off) *(re-check B3)*. **D3 refused 12 of those 1,500** — only at D = 2 in, S = 6 in
  with the nose 20° away from the point (the point 91.6° off the nose); never on the main matrix.
- **Inside the rule the curve is the fastest fix:** inside the two-number rule (B1) on the main matrix, **600 of 600
  end within 0.5 in and 1.15° of the TRUE pose (`fmf` 555 of 600), median 2.14 s — 0.56 s saved per pose** against
  `fmf` on the same runs (0.08 s lost at worst, 1.58 s saved at best; at a 15 % weaker side 195 of 200, 0.80 s saved;
  `fix2/inside.txt`). *(The first draft's rule: 780 of 780, 0.58 s — superseded, B1.)*
- **A plain `moveTo` is not a fix:** 18 % of the main matrix (a square 1 in miss needs 18 in; nothing at 20° off).
- **The run-up is set by the look-ahead** (above): Station C's choice moves the rule, so the guide gives it for the
  look-ahead the robot file holds (§3.1) and C.5b checks one row on the robot.
- **Found on the way (not CURVE1's to fix — for the coordinator):** today's fix, `face` + `driveTo` + `turnTo`,
  arrives every time by its steps' own verdicts (4,320 of 4,320), but its LAST turn in place on a weak side moves the
  robot's centre: on the TRUE pose **12 % of runs at a 0–9 % weaker side end 0.5–1.0 in from the point** (55 % at
  15 %, to 1.55 in) — while `face` + `driveTo` alone end ≤ 0.44 in (`p1_fm.txt`). `turn_to.hpp` and guide 14 do not
  say so. For a look spot (AF1-S needs the heading within ~3°, which holds) this rarely matters; for a pose that must
  be exact to 0.5 in, it does.

**Probe 2 — the back-up-and-re-approach, against what the team already has** (`p2_compare.txt`, `pc_compare.txt`,
`paired.txt`, `truth.txt`; weak 0–9 %, lag 0/0.12 s; "settled" = the steps' own verdict, "true" = within 0.5 in and
1.15° of the TRUE pose; times include failures at the 6 s they cost):

| Method | Main matrix (D 6–36): settled / true / time median, worst | Close (2–6 in out, 2–6 in to the side) | Reverses blind |
|---|---|---|---|
| a plain `moveTo` (times out) | 18 % / 18 % / 6.01 s | 0 % | — |
| one-point `curveTo` (+ STEP1-A's `onFail(carryOn)`: the route goes on ≤ 3.5 in off) | 52 % / 52 % / 2.77 s (2.03 s when it settles) | 0 % | — |
| curve + `{.positionTolerance = 1_in}` | 75 % / — / 2.07 s | 3 % | — |
| curve + `{2_in, 3_deg}` | 90 % / — / 1.75 s | 27 % | — |
| **today's `face` + `driveTo` + `turnTo`** | **100 % / 88 % / 2.72 s, 4.26 s** | **100 % / 82 % / 3.15 s, 4.06 s** | ≤ 0.5 in |
| **re-approach at the start (`raPre`)** | **100 % / 100 % / 3.02 s, 4.00 s** | **100 % / 100 % / 3.30 s, 3.91 s** | **up to 32–36 in** |
| re-approach on failure (`raPost`, R = 12 / 18 / 24 in) | 72 / 81 / 86 % / 3.98–4.41 s | 10 / 53 / 82 % / 4.42–6.01 s | 12–24 in |
| the author's own: **`faceAway`** + `reverseTo` (30 in) + `curveTo` | 99.9 % / 99.9 % (3,236 of 3,240) / 3.84 s, 6.01 s | 100 % / 100 % / 3.91 s, 5.06 s | 27–29 in |
| the same WITHOUT `faceAway` (`reverseTo` is not steered when the robot does not face away; the curve never runs) | — / **45.1 %** / 2.51 s, worst end **34.9 in** off | **49.5 %** | — |
| *the guide's own advice: a curve inside the rule, `face` + `driveTo` + `turnTo` otherwise (B2)* | — / 89.2 % / 2.54 s, 4.26 s | 82.4 % / 3.15 s | ≤ 0.5 in |

*(Re-check B2 — every method on the TRUE pose, same runs, `fix2/truth2.py` → `fix2/truth2.txt`:)* at a weaker side
0–9 %: **`raPre` 100 % (3.02 s median)**, `fmf` 87.8 % (2.72 s), the guide's mix 89.2 % (2.54 s), the author's
`faceAway` + `reverseTo` + `curveTo` 99.9 % (3.84 s); close starts: `raPre` 100 % (3.30 s), `fmf` 82.4 % (3.15 s), the
author's 100 % (3.91 s). **At a 15 % weaker side: `raPre` 98.8 %, `fmf` 44.6 %, the mix 49.4 %, the author's 95.4 %
(4.22 s); close: `raPre` 100 %, `fmf` 13.4 %, the author's 100 % (4.25 s).** Paired against `fmf` on the same runs:
`raPre` is **+0.26 s slower (median) on the runs where it backs up**, **+1.44 s at worst**, 0.94 s faster at best; on
the close matrix **+0.16 s median, +1.45 s worst**. Against the author's own back-up, `raPre` is **0.6–0.8 s faster
(median)** and reverses a little further (main up to 31.9 in, close 18–36.2 in, against 26.8 / 24.1–28.6 in).

**The verdict: REJECTED** (§5 OUT, with these numbers). The rule was "adopt ONLY if clearly better on success AND
time for the cases a skills route meets, and its risks bounded":
- **success** — *(re-check B2: restated; the first draft called it "a tie by the steps' own verdicts", which hid the
  true pose)* **on where the robot really ends up, `raPre` WINS**: 100 % against `fmf`'s 88 % and the guide's mix's
  89 % at a weaker side up to 9 %; **99 % against 45 % at 15 %**; close starts 100 % against 82 % (13 % at 15 %).
  `fmf` and the mix still arrive by every step's own verdict; their miss is `fmf`'s last turn in place on a weak side
  (0.5–1.0 in, 1.55 in at 15 %). So on success alone it IS better for a pose that must be exact to 0.5 in;
- **time** — NOT better than what the team already writes: slower than `fmf` on the median wherever it acts (+0.26 s;
  +1.44 s at worst) and than the guide's mix (3.02 against 2.54 s); **faster than the student-written back-up by
  0.6–0.8 s** (3.02 against 3.84 s main, 3.30 against 3.91 s close); the on-failure form wastes a failed approach
  first (median 4.0–4.4 s) and succeeds less. The adoption test needed success AND time — it passes one;
- **risk** — NOT bounded: it reverses up to 3 ft on a robot that cannot see behind it, toward a wall (±70.2 in), a
  piece, the other robot or a field start's keep-out, and the library cannot know which; the 60 s run pays its 1.45 s
  worst case with no say from the author.
- **The freeze** — it would be a new behaviour on a frozen verb's end, CV tests, a Warn and a `CurveConfig` field,
  added before the ~Nov 18 freeze; the CURVE1 build must not slip for an option.
- **So, honestly:** the library back-up would land exactly MORE often than turn-drive-turn and would be ~¾ s faster
  than the student-written back-up; it is rejected because it reverses up to 3 ft blind on the library's own
  judgement, and because building it now puts the build at risk before the freeze. **The trade, if the team lead
  wants it later:** after the 21st it could return as an EXPLICIT, opt-in step the author places (the author, not the
  library, judging the space behind) — not proposed here.
- **What the team gets instead, with no new code:** the rule (plan the run-up); `face` + `driveTo` + `turnTo` when
  the robot is closer (exact by its own verdict; on a weak side its last turn can leave the robot 0.5–1 in off the
  point in ~1 run in 8); and, where the AUTHOR knows the space behind is clear and the pose must be exact to 0.5 in on
  a weak side, **`faceAway`** + `reverseTo` + `curveTo` — written by the student, visible in the route, slower
  (3.8–3.9 s median), 3,236 of 3,240 on the main matrix and every close run at a side up to 9 % weaker (95.4 % at
  15 %) on the host test plant. **Without the `faceAway` it fails about half the time** (45–50 %: `reverseTo` is not
  steered unless the robot faces away, and the curve never runs; worst end 34.9 in off) *(re-check B4)*.

**Body changes (each marked *(amended 2026-11-10 late)*):** the status line; D3 (where D3 refused); D4 (a) (the
sideways miss and the rule; the case-3 Warn's words); the plain words; §1.2 "The end" (the Warn's words; no new
behaviour, said so); §2.0 C.5b (optional, ~10 min); §2.1 (size note); §2.2 **CV23** (the rule, pinned) with
mutations **M27** (look-ahead 8 → 12) and **M28** (tangent length ½ × chord); §3.1 items 1, 2, 4 and guide 14 (item
3); §4 HA-246 (the rule folded in — no new HA); §5 OUT (the re-approach, rejected); §9; the decisions (item 13).
**`cv23.sh` → `cv23.txt` / the CV23 numbers:** at the shipped defaults, the three rule cells (D, S) = (24, ±2),
(30, ±6), (36, ±12), nose 0/±10/±20°, ×1.00/×0.91 × τ 0/0.12: **40/40 true each**; look-ahead 12 in: 27, 19 and
18 of 40 (RED); tangent ½ × chord: 40, 38 and 26 of 40 (RED on two cells). The hand-off numbers (0.3 in → 3 in,
5° → 15°, the ungated last resort) do NOT move these cells (the robot is lined up long before) — they stay pinned by
CV6/CV8, not here. The too-close side (D 12, S ±4, nose 0/±20°): 22 of 24 time out at the budget (6.01 s), worst
miss 1.52 in, no guard trip. *(Superseded by B1/B4 below: CV23's cells and lags are re-sized — `fix2/cv23b.sh` →
`fix2/cv23b.txt`.)*

### Re-check of the amendment (2026-11-10, after its checker; every number re-run by this writer)

New probe files under `amend/fix2/` (every binary under `ulimit -v 4 GiB` + `timeout`; the tree untouched):
`need.py` / `need2.py` (the need tables from the checker's `xcheck/fine_all.txt` and `weak15_all.txt` — its
`runup.cpp` and `curve_core.hpp` are byte-identical to this amendment's, checked with `diff`), `variants.sh` →
`v_p1.txt`, `v_la6.txt`, `v_la10.txt` (the guide's other rows on a 1 in grid, lag 0/0.12/0.18), `rule_new.py` →
`rule_new_fine.txt`, `truth2.py` → `truth2.txt`, `inside.py` → `inside.txt`, `cv23b.sh` → `cv23b.txt`,
`tooclose.txt`, `cv23_margin.txt`.

| # | Finding (severity) | Verdict | What changed |
|---|---|---|---|
| **B1** | The rule's first two steps fail at 0.18 s of motor lag, a lag the brief tests everywhere else (major) | **REAL.** `xcheck/rule.py` re-run: **149** in-between-grid runs inside the first rule fail at τ 0.18 (25–38 per weak cell), all misses ≤ 2 in with the nose 15–20° turned away, 21–27 in back, timing out 0.51–0.63 in off; the worst need at 0.18 s is 26–28 in for every miss; neither robot's lag is measured (HA-246) | **Fixed — the checker's simplest fix.** THE RULE IS TWO NUMBERS: **a miss up to 6 in needs 2½ ft (30 in) of run-up; up to 1 ft, 3 ft (36 in)** — nose within 20°, a side up to 9 % weaker, lag 0–0.18 s, the shipped look-ahead 8 in. **0 of 68,040** in-between-grid runs inside it fail (`rule_new_fine.txt`). The first draft's 2 ft step is gone everywhere (D4, the plain words, §1.2, C.5b, CV23, §3.1 items 1–2, HA-246, §9, decision 13). The other rows re-measured at 0.18 s too: `positionTolerance = 1_in` → 1¾ ft up to 6 in, 2 ft up to 1 ft (was "1¼–1¾ ft"); look-ahead 6 in **never settles at 0.18 s** (said so); look-ahead 10 in → 3¼ / 3⅔ ft; a 12–15 % weaker side → 2¾ / 3⅓ ft. The rule costs the team 6–9 in more run-up for a 1–2 in miss than the first draft said — the price of holding at an unmeasured lag. **C.5b now runs the hardest corner: 2½ ft back, 6 in to the side, the nose 20° turned AWAY from the line** |
| **B2** | The rejection and decision 13 call success "a tie" by each step's own verdict; on the TRUE pose the back-up wins, and beats the guide's alternative (major) | **REAL.** `truth2.txt` reproduces every number: `raPre` 100 % (3.02 s), `fmf` 87.8 % (2.72 s), the guide's mix 89.6 % under the first rule / 89.2 % under B1's (2.52 / 2.54 s); 15 %: 98.8 / 44.6 %; close at 15 %: 100 / 13.4 %; the author's `faceAway` + `reverseTo` + `curveTo` 3.84 / 3.91 s against `raPre`'s 3.02 / 3.30 s, 95.4 % at 15 % against 98.8 % | **Fixed.** REJECTED kept — for its blind reversing and the freeze; **"success is a tie" dropped** in the verdict above, §5 and decision 13, which now say plainly: on where the robot really ends up, the library back-up would land exactly more often than turn-drive-turn (100 against 88 %; 99 against 45 % at a 15 % weaker side), and ~¾ s faster than the student-written back-up; it is slower than what the guide recommends (2.54 s) and reverses up to 3 ft blind on the library's judgement |
| **B3** | "Up to 3½ in off" understates a too-close curve's miss; it can end further to the side than it started (minor) | **REAL.** `pc_curve.txt`: 1,984 driven close runs, all timing out at the budget; worst 4.13 in (4.17 at 15 %); **206** end further from the point than their sideways start; 3½ in is the main matrix's (6 in or more back) | **Fixed** in the amendment's bullet, D4, §1.2, the plain words, guide 05's text and decision 13: "from 6 in or more back it ends up to 3½ in off; from closer, about as far to the side as it started — and it always uses its whole `.timeout`" |
| **B4** | CV23's cells sit on a hair-thin margin (minor) | **REAL in kind.** Under B1 the cells move: (30, ±2), (30, ±6), (36, ±12) against a worst need of 28 / 29 / 34 in (margins 2 / 1 / 2 in); the true end on the passing cells is ≤ 0.50 in and 1.07° — the 0.5 in is the arrival tolerance itself (a settled run ends inside it by construction), so the thin part is the RUN-UP margin | **Fixed both ways the checker offered:** CV23 asserts the rule's own cells, now **× τ 0/0.12/0.18 s (60 runs a cell; host 60/60 each, `cv23b.txt`)**, and LOGS (does not assert) the smallest passing D per cell on a 1 in grid; its header says **a red CV23 means re-measuring the rule and updating guide 05 in the same commit — never tuning the follower to pass**. M27 (look-ahead 12): 47, 30, 31 of 60 — RED; M28 (tangent ½ × chord): 60, 58, 46 of 60 — RED on two cells; the hand-off 0.3 → 3 in now costs one run (59/60 at (30, ±2)) — still not relied on, CV6/CV8 pin it. Too-close side at τ 0–0.18: 30 of 36 time out at the budget, worst 1.52 in (`tooclose.txt`) |
| **B5** | The header's STEP1-A status is out of date (minor) | **REAL** (NOTES :965: STEP1-A DONE; merge queue MECH-S-A → CAL1-B → STEP1-A) | **Fixed** in the header, §6.1, §6.2, §9 and D12's dependency line: STEP1-A is built and merges third. The dates hold: CURVE1-A still needs its MERGED tip and a slot behind AF1S-A1, TUNE1-B1 and STEP1-B (start ~Nov 12–13, merged ~Nov 16–17, the freeze ~Nov 18). The run-up additions stay ~⅛ day (B1 changes numbers in text and one test's cells, no library behaviour) |

---

## Re-check — amendments (2026-11-10, after the checker; every number below re-run by this writer)

**New probes** (`.claude/coordinator/scratch/brief-CURVE1/recheck/`, every binary under `ulimit -v 4 GiB` + `timeout`;
the tree untouched): `rc.cpp` = `probe_curve.cpp` + a heading-gated last-resort hand-off (`hlh=`, `hard=`) + `seed`
(carries the curve's yaw estimate into the member through a SHADOW copy of `move_to_pose.hpp` in `recheck/inc/` — a
probe-only method, never proposed for the tree); `mx.sh` / `sum.sh` (the 17 drivable routes × 9 cells, `off150`
excluded because D3 refuses it); `refusals.py` / `refusals_all.py` (the proposed path refusals over 32 shapes);
`sizes3.cpp` (the trimmed layout). The checker's own probes (`check_probes/`) were re-run where cited.
**"R-A"** below = the recommended defaults as amended: look-ahead 8 in, grip 60 in/s², hand-off 12 in / 5° / 0.3 in,
last resort at 4 in gated by the member's steering window, search window 12 in (`mx.sh rc g15w12 … hlh=15 hard=0.01
win=12`).

| # | Finding (severity) | Verdict | Disposition — what changed in the body |
|---|---|---|---|
| **A1** | The end hand-off often comes > 15° off the end heading: the member `MoveToPose` is not steered and logs "turn first: face()" (major) | **REAL.** `R_matrix.txt` is byte-identical to P17; re-counted: **25 of 153** healthy runs hand off > 15° (to 34.9°), each with the member's Warn; 4 of them time out | **Fixed — the checker's option (a), refined.** The 4 in last-resort hand-off now fires only when the heading is inside the member's OWN steering window — judged by `tankLegEngages()` (`tank_steer.hpp` :106-109), the function `MoveToPose` itself uses, so the two can never disagree. Otherwise the follower keeps pursuing (along the end's line, extended) until it is inside, or until it reaches the end of the path; THERE it hands off regardless, to a member built from a config copy with `tankSteer.maxSteer = 0` ("switches it off", `motion_config.hpp` :202) — the same unsteered line the member would have driven, minus the member's misleading "turn first: face()" Warn; the curve logs its own words. **R-A: hand-offs > 15° 25 → 2** (both `bend90r12` under 0.12 / 0.18 s lag); **settled 139 → 140 of 153**; the worst end heading of a settled run 1.01° → **0.74°**, position unchanged (0.50 in). **Option (b) TESTED AND REJECTED:** seeding the member's observer with the curve's estimate made it WORSE (136/153 alone, 137 with the gate) — the member's observer keeps starting from 0, on evidence. **NOT fixed, stated:** at a 15 % weaker side with NO lag, `bend90r18`, `bend90r12`, `snake` and `revS` end 0.51–0.58 in off and time out WITH hand-offs ≤ 15°; a longer hand-off (16 in: 139), a later last resort (6 in: 131) and the seeded observer do not help — TANK1's own short-leg limit (L2); Station C and STEP1-A's `positionTolerance` are the answers. Body: D4, §1.2 "The end", CV6, CV8, L14, the plain words |
| **A2** | The path builder needs more refusals: NaN at an out-and-back free point, a cusp from a heading pointing back, and D2's own reverse example is wrong (major) | **REAL.** Re-run: `outback` "dumped core"; `loopmid` lost at 0.81 s, 32.7 in from the end; `revEx180` hand-off 72°, tightest bend 0.1 in, one Warn; `revEx0` settles (6.5 in, 6.8°, no Warn); `dupNear` (0.001 in apart) accepted | **Fixed.** `checkCurve` gains an **`Invalid`** verdict — refused before the robot moves, the Warn naming the point: (i) a point within 1 in of the point before it (A3); (ii) a free point whose two neighbours are within 1 in of each other (it has no direction — the NaN); (iii) ANY point's direction — its heading, or the one the library picks — more than 90° from the chord arriving at it or the chord leaving it (D3's start rule, applied everywhere). With (iii) and tangent length = chord, the cubic Hermite's derivative never turns against its chord: P′(u)·c / \|c\|² = 6u(1−u) + (1−u)(1−3u)·cos a + u(3u−2)·cos b, linear in cos a, cos b ∈ [0, 1], whose four corners are 6u(1−u), (1−u)(1+3u), u(4−3u) and 1 — all ≥ 0 — so "the derivative reverses" needs no runtime check: it is a TEST property (CV1). `refusals_all.py`: all 17 drivable routes and the Station C shapes pass, `hairpin6` → TightBend, `off150` → StartBehind; `outback`, `outbackNear`, `loopmid`, `revHead`, `dupNear`, `revEx180`, `nearAheadH` → Invalid. **The heading convention is written down: a heading is ALWAYS the robot's NOSE, also on `reverseCurveTo`**; D2's example corrected to `0_deg`. Body: D2, D3, §1.1, CV1, CV12, L15 |
| **A3** | A first point at (or within an inch of) the robot gives a random refusal or a false warning (major) | **REAL.** `nearAhead` (0.36 in away) accepted with a 0.1 in "tightest bend" → a false TightBend Warn on a clean run; `nearSide` sits on the 90° boundary; a point 0.3 in behind would be refused | **Fixed.** One design constant, `kCurveMinSpacing` = **1 in** (twice the translation settle's 0.5 in, HA-51). A HEADINGLESS first point within 1 in of the live start is DROPPED with one Info line ("curve: point 1 is 0.4 in from the robot -- dropped; do not list the start"); a HEADED one is refused (Invalid: "point 1 is where the robot already is"); the same 1 in between consecutive points; a curve left with no point is refused. `refusals.py`: `nearAhead` / `nearSide` / `nearBehind` dropped → Ok, tightest 20.0 in (no false Warn); `nearAheadH` refused. CV12 tests both sides of the boundary (0.99 / 1.01 in); the cookbook says "do not list the start". Body: D2, §1.1, CV12, §3.1 |
| **A4** | `static_assert(sizeof(CurveFollow) <= 4096)` cannot hold (major) | **REAL.** `sizes2`: ~5,536 B | **Fixed by trimming AND an honest budget.** No separate point copy (the path's knot array holds the points from construction; the start knot is filled at the first live tick); no stored coefficients (evaluated from knots + tangents); `float` arc table and speed plan (A6) at the same 17 samples per segment; the member `MoveToPose` in a `std::optional`, emplaced at the hand-off (A1 picks its config then). `sizes3.cpp`: CurvePath 3,144 + member 1,504 + own MotionConfig 536 + state ~170 ≈ **5.35 KB on the host** (~16 % of the 32 KiB task stack). **`static_assert(sizeof(CurveFollow) <= 6144)`** on the host, from that stated count; the ARM `-fstack-usage` frame (§2.3) stays the REAL gate, flagged above 8 KB. Body: §1.1, §1.2 "Size", CV21, L4, HA-250 |
| **A5** | The schedule and robot scope are out of date (major) | **REAL.** NOTES :963 (11-10 ~23:30, after this brief's ~23:00 pass): "BOTH ROBOTS run skills", robot one has NO build (G1), the build queue is AF1S-A1, TUNE1-B1, STEP1-B, then CURVE1; the freeze (~Nov 18, `build-order.md` :2401) covers Station C's tuned numbers and any `kUseCurves` line | **Fixed.** D12 rewritten: CURVE1-A needs STEP1-A MERGED (CV17, the `onFail`), NOT STEP1-B — it runs BESIDE STEP1-B, the second to merge re-reading the other's blocks (+~¼ day) and taking the next `kApiMinor` (7 or 8); waiting for STEP1-B's tip lands it ~Nov 19–21, after the freeze. Station C on robot TWO only (the robot built "by the 14th"); robot one uses robot two's numbers as a recorded belief (HA-246) and runs curves only after its own short check (C.1), if built in time. **`kUseCurves` defaults to `false`**; the team lead decides each flip from the Station C log (the students write the line, G4), **before the freeze** — unless he rules that the two tuning numbers and that one line may change after it (asked in D12). The plain truth for the team lead: **a curve the team can trust in the skills run needs Station M AND Station C done on the robot by ~Nov 18.** Body: header, D12, §2.0, §2.1, §6.1, §6.2, HA-246, §9, the decisions |
| **A6** | The specified speed plan never brakes INTO a bend (unlike the probe), and L11's ω cap is missing from the formula (major) | **REAL.** `probe_curve.cpp` :143-161 runs a backward pass; §1.1's `speedLimitAt` had only the bend, the end and the cap. The checker's `R_nobwd.txt` re-summarised: **135 vs 139 of 153** settled; the 0.18 s-lag cells lose the snake, the field S and a bend | **Fixed.** The speed plan is a per-sample table (17 samples per segment, `float`) built ONCE at the first live tick: the bend limit √(bendAccel ÷ the largest \|κ\| within the next 2 in), **v ≤ maxAngularSpeed ÷ \|κ\|** (L11), the call's and the robot's `maxLinearSpeed`, 0 at the end — then a BACKWARD pass so the robot is already at the bend's speed when it reaches the bend, slowing at no more than `decel`. `speedLimitAt(s)` interpolates it (the preview reads the same table). CV19 extended. Body: §1.1, CV19, L11 |
| **A7** | D3 calls the 6 in hairpin "the one route the robust defaults miss" (minor) | **REAL** (P14 and R-A: at ×0.85, no lag, two tight bends, the snake and `revS` also miss; at 0.18 s lag the 36 in bend) | **Fixed** in D3, tied to A1's residual |
| **A8** | The plain-words times are the fast setting's (minor) | **REAL** (`p15_times.txt`) | **Fixed.** Recommended defaults, no lag: **2.64–5.24 s as curves against 4.01–11.23 s stop-and-turn — 47–67 % of the time** (with 0.12 s lag 51–68 %); the fast setting (2.27–4.26 s) is the after-Station-C upside. Body: D1, the plain words, §9 |
| **A9** | The guards are off in the last 12 in (a side dying there runs to the full timeout), and a jam cannot be detected on robot two (minor) | **REAL.** `rc … dead=1.4/1.9/2.2`: `scurve` and `bend90r36` TimedOut at the full 6 s, total travel 48–55 in on 54–55 in paths (bounded); `fieldS` (whose deaths fall outside its end zone) stopped 1.0–1.2 s after. Robot two's odometry is its drive encoders: wheels spinning against a wall read as progress and `OdoStallCheck` gives no verdict (`motion_config.hpp` :193-197) | **Fixed.** D6 states the end-zone residual (the step's timeout ends it; travel stays bounded) and drops "a jam" for encoder odometry; CV5 gains a death inside the last 12 in (TimedOut at the budget, bounded travel); **Station C no longer pulls motor cables** (impractical, and it tests device loss and the fault policy, not the curve's guards): the dead-side half of HA-248 is host-only, and every Station C log is read for a FALSE "curve lost". Body: D6, the plain words, CV5, §2.0, HA-248 |
| **A10** | D7 misstates why C1's D12 was decided, and "never off the drawn path" contradicts the look-ahead past the end (minor) | **REAL** (`C1-COMPLETED.md` :270-271: "would smuggle the LemLib behaviour into the library's core"; pure pursuit IS LemLib's `follow`, master plan :147) | **Fixed.** §0.2 and D7 quote the real reason; the dated note says plainly that the team lead's ruling admits that LemLib behaviour inside the two curve verbs only; D7 says the follower aims only at the drawn path or the end's line extended. Body: D7, §0.2, the plain words |
| **A11** | D11 misreads G3a: the panel may not simulate, and a TypeScript panel cannot call the C++ builder (minor) | **REAL** (G3a :592-593, the 10-08 ruling: "Any simulation OUT … VexBuilder is the one simulator") | **Fixed.** D11 (later part): the panel ports the pure builder to TypeScript, pinned against golden vectors a C++ test writes (or a WASM build, its own decision); drawing the PLANNED geometry is not a simulation, but bend SPEEDS are dropped from the panel's drawing (G3a: the step-through "never shows speed"). Nothing changes before the 21st. Body: D11 |
| **A12** | Smaller inconsistencies (minor) | **REAL**, item by item | `CurvePoint` is F6-FROZEN as a verb parameter type (its two constructors pinned, CV22); `CurvePath`, `checkCurve` and `CurveConfig` stay open (§1.1). **A default constructor NOT added:** after A4 the fixed array holds the path's own knot type, so nothing needs one, and it would freeze a third constructor meaning "a point at the field's origin". CV21: `is_trivially_copyable_v` kept for what it proves (a cheap copy), "no pointer member" moved to §7's reading. CV16's baseline is the MERGE BASE. CV6: 17 routes. D1 (b): boomerang hand-offs 4.8–26.3° (P7). §1.4 adds STEP1-A's "Read by" comment edit (curve verbs named, no new field). `motion_scheduler.hpp` at `16c01e0`: the result block :1154-1176, `hasCrossTrack` :1158-1159, `overshoot()` :470 |
| **A13** | *(found by this writer)* The probe's closest-point window was 40 in; the brief specifies 12 in | **REAL gap in the evidence** | Re-run at 12 in (`g15w12`): **identical, 140/153** — `searchWindow` 12 in stands, now measured |
| **A14** | *(found by this writer)* HA numbers: RPV1's brief (:543) takes "HA-246 — unless CURVE1 reserves first" | no collision | CURVE1 keeps HA-246..250; RPV1's first is HA-251 if it needs one — the coordinator writes both at the plan commit |

**Per cell, R-A (17 drivable routes; ×1.00 / ×0.91 / ×0.85 weak right side × lag τ 0 / 0.12 / 0.18 s):** 16 16 14 /
16 16 16 / 13 17 16 settled; 0 guard trips in 153; the worst heading off the path's direction on a healthy run 24.6°
(the guard is 60°). These replace P14's "14–17 of 18" wherever the body quotes the recommended defaults.

---

## DECISIONS FOR THE TEAM LEAD (a recommendation for each; nothing below decides them silently)

The team lead already chose WHAT (a true curve follower, before the 21st, re-opening "never aim at the point" for it).
These twelve are the HOW questions that choice leaves open. Every number quoted is the host test plant's (§0.4), never
a robot's.

**D1 — The controller. RECOMMENDED (a) pure pursuit.**
- **(a) RECOMMENDED — pure pursuit on a smoothed path, with TANK1's yaw observer.** Every 10 ms the robot finds where
  it is along the path, picks the point on the path a fixed distance AHEAD of that (the "look-ahead", 8 in by
  default), and drives the arc that would carry it through that point; the speed comes from a speed plan along the
  path (slower in tight bends, slowing to the end). TANK1's yaw observer (`move_to_pose.hpp` :274-294) cancels a
  weaker side, as it does for `moveTo`. It is what LemLib's `follow` is (master plan :147), the method most VEX
  teams know, and it has ONE main knob. On the host test plant it is the only law that settled every route on a
  lag-free plant at every weak side tried (×1.00, ×0.91, ×0.85: 18/18 each, P14, the fast setting), and with the
  robust defaults it settles **13–17 of the 17 drivable routes** at every weak side and motor lag tried (R-A,
  *re-check A1*), where the others fall to 2–13. Its curves take **47–67 % of the time** of today's stop-and-turn
  legs on the same routes (*re-check A8*).
- **(b) Boomerang** (LemLib's `moveToPose`: chase a "carrot" point set back from the target along its heading).
  **Cost:** it knows only the LAST pose, so it cannot follow the author's middle points at all (31–39 in off the drawn
  route on the multi-point routes, P16); on the single-pose curves it settled 0 of 6 with this engine's end (P7) —
  it reaches its hand-off 4.8–26.3° off the end heading, and TANK1's short leg does not finish from there: every run
  times out 0.55–2.35 in from the end (*re-check A12*). A route of several boomerangs is "rolling
  through points" (G3a's D16 (b)), which the team lead did not choose.
- **(c) A path-tangent follower — TANK1's bend carried along the curve.** The heading setpoint is the path's own
  direction at the closest point plus TANK1's bounded bend toward the path; it never aims at any point, so C1's D12
  would not even need re-opening. **Cost:** it tracks a little tighter on a clean plant (worst 0.51 in off the path
  against pursuit's 0.71, P16) but falls apart with a weak side or motor lag — 11/18 at a 15 % weaker side with no
  lag, 5/18 with lag, against pursuit's 18/18 and 16/18 (P16, P14).
- **(d) A timed trajectory with a tracking law (RAMSETE-style).** **Cost:** it needs the drive's measured speed
  constants (kS/kV/kA — R5, never measured, HA-50) to be right, a time plan for the whole path, and much more code:
  not before the 21st.

**D2 — How the author writes a curve. RECOMMENDED (a).**
- **(a) RECOMMENDED — a list of points, each with or without a heading; the curve starts where the robot IS.**
  ```cpp
  r.curveTo({{24_in, 12_in}, {48_in, 24_in, 0_deg}});        // through (24, 12), end at (48, 24) facing 0°
  r.reverseCurveTo({{-12_in, 30_in}, {-36_in, 36_in, 0_deg}});  // from (0, 24) facing 0°, backing through (-12, 30)
                                                                // to (-36, 36), still FACING 0° (its back leads)
  ```
  **A heading is ALWAYS the way the robot's NOSE points — also on `reverseCurveTo`**, where the robot travels
  opposite to it. *(re-check A2: the first draft wrote `180_deg` here — the direction of TRAVEL — which on the host
  test plant made a 0.1 in bend, a 72° hand-off and the robot turning round at the end; `0_deg` settles cleanly. A
  heading that points back along the path is now refused, below.)* **Do not list the robot's own spot as the first
  point** (a habit from other tools): a point without a heading within 1 in of the robot is dropped with an Info line,
  one with a heading is refused (*re-check A3*).
  A point with a heading means "pass here pointing this way"; a point without one means "pass here, the library
  picks the smooth direction". The library joins the robot's live position and heading (read when the step runs,
  like `driveTo`'s bearing, `routine.hpp` :312-320) and the points into one smooth path (cubic Hermite segments; a
  point with no heading takes the direction from the point before it to the point after it). Up to **16 points**
  per curve (a longer route is two curves). The end point's heading is the heading the robot finishes at; with no
  heading there, it finishes along the path's own direction.
- **(b) Every point must carry a heading.** **Cost:** two numbers more per point, and a wrong middle heading makes an
  S-wiggle nobody drew.
- **(c) Lines and circular arcs (radius, angle).** **Cost:** a person draws points, not radii; it would not match the
  route maker's later points or F8's waypoints.

**D3 — How a curve STARTS. RECOMMENDED (a).**
- **(a) RECOMMENDED — from the robot's own heading; a curve whose first point is BEHIND the robot is refused.** The
  path leaves the robot along its nose (backwards for `reverseCurveTo`), so a curve never begins with a turn in
  place. If the first point lies more than **90°** off that direction, the step fails at once (it does not drive) with
  one Warn: "curve not driven: its first point is 150 deg off the robot's heading (at most 90) -- turn first:
  face()". Probed: starts 30°, 60° and 90° off settle (P2); 150° makes the path loop on itself (its tightest bend
  0.5 in, P1) — the robot followed it 2.7–5.5 in wide. A path whose tightest bend is tighter than half the look-ahead
  (4 in at the default) is DRIVEN but warned about once ("tightest bend 4.0 in -- tighter than the look-ahead can
  follow; expect it to cut the corner"): the robust defaults miss the 6 in hairpin (4 in radius) everywhere, and at a
  15 % weaker side with no lag also two tight bends, the snake and the backwards S (each ends 0.51–0.58 in off — the
  end `moveTo`'s short-leg limit, D4 and *re-check A1*), and at 0.18 s of lag the 36 in bend (*re-check A7*).
  **And a path that doubles back is refused before the robot moves** (*re-check A2/A3*, `Invalid`, one Warn naming the
  point): a point within 1 in of the point before it; a point without a heading whose two neighbours are within 1 in of
  each other (an out-and-back through one point has no direction); any point whose direction — its heading, or the one
  the library picks — is more than 90° from the way the path arrives at it or leaves it (the same 90° as the start).
  On the host test plant every one of the 17 drivable routes passes these and every broken shape the checker built is
  refused (`refusals_all.py`). *(amended 2026-11-10 late:* on the sideways-miss probe — a one-point curve from 2–36 in
  back, 1–12 in to the side, nose up to 20° off — D3 refused only 16 of 6,320 starts (every cell), all at 2 in out, 6 in
  to the side, the nose 20° away from the point; the refusal is right: no curve from there fixes anything, see D4.)
- **(b) Turn in place to face the path first.** That is a turn the library plans for the author — the very thing D12
  forbids and TANK1 refused; `face()` is the author's turn.
- **(c) Never refuse.** A curve that starts behind the robot drives a loop.

**D4 — How a curve ENDS. RECOMMENDED (a).**
- **(a) RECOMMENDED — the last stretch IS an ordinary `moveTo` to the end pose.** When the robot is within 12 in of
  the end along the path, within 5° of the end heading and within 0.3 in of the end's line, the curve hands over to
  the library's own `MoveToPose` (with TANK1's steering) for the last inches — or, if it never lines up, at 4 in from
  the end **once its heading is inside TANK1's 15° steering window** (judged by the same function `MoveToPose`
  uses). *(re-check A1: the first draft handed off at 4 in regardless; in 25 of 153 healthy host runs that came
  15–35° off the end heading, so the end ran UNSTEERED and logged TANK1's "turn first: face()" — wrong words for a
  curve the author drew correctly.)* If the robot reaches the end of the path still outside the window (2 of 153 host
  runs, a 12 in bend under motor lag), the curve hands over anyway, to an unsteered end, and says so in its own words
  ("curve end: 21.1 deg off the end heading at the end of the path (steers within 15.0) -- unsteered; more run-up, a
  gentler last bend or a lower speed into it" — *"more run-up" amended 2026-11-10 late*). So a curve ENDS like a `moveTo`: the same stop, the same arrival check (0.5 in and
  1.15° on the defaults, HA-51), the same per-step arrival tolerance STEP1-A adds (`{.positionTolerance = 1_in}`
  works on a curve too), TANK1's steering in all but those rare ends, and the same log. **The heading at the end is
  judged** — AF1-S needs the robot square to a wall within ~3° before it looks at a Toggle (AF1-S brief §6.6).
  Probed (R-A): the **140 settled runs of 153 end within 0.50 in and 0.74° of the end pose**, true pose. The 13 that
  do not are the 6 in hairpin (D3's warning), the 36 in and 12 in bends under 0.18 s of lag, and four routes at a
  15 % weaker side with no lag that stop 0.51–0.58 in off — just outside the 0.5 in arrival; carrying the curve's
  weak-side estimate into the end was tried and made it worse (136/153).
  **A sideways miss — what a curve fixes, and from how far** *(amended 2026-11-10 late; the amendment's probe 1)*. A
  tank cannot slide sideways; a one-point `curveTo` to the pose removes a sideways miss by arcing onto the end's line
  — but only with enough RUN-UP, because its last inches are TANK1's short `moveTo`, which removes little sideways
  (L2). **The rule (shipped look-ahead 8 in, nose within 20° of the end heading, a weaker side up to 9 %, motor lag
  0–0.18 s): a miss up to 6 in needs the curve to start 2½ ft back along the end's line; up to 1 ft, 3 ft**
  *(amendment re-check B1: the first draft's "2 ft for up to 2 in" failed at 0.18 s of lag)* (the amendment's B1
  table; worst need 29 / 34 in; a 12–15 % weaker side needs 2¾ / 3⅓ ft; `{.positionTolerance = 1_in}` brings it to
  1¾ / 2 ft; look-ahead 10 in to 3¼ / 3⅔ ft; look-ahead 6 in needs only ~1⅔ ft but never settles at 0.18 s of lag).
  Inside the rule it ended within 0.5 in and 1.15° of the TRUE pose in every host run (0 of 68,040 failed) and beat
  today's `face` + `driveTo` + `turnTo` by 0.56 s (median) per pose. Closer, it misses SAFELY: it always uses the
  step's WHOLE budget, no guard trip, and the result line's ` xt` shows the miss (guide 11, §3.1) — from 6 in or more
  back it ends ≤ 3.5 in off; from 2–6 in out it fixes nothing (0 of 1,500) and ends about as far off as it started to
  the side (≤ 4.2 in) *(amendment re-check B3)*. **Closer than the rule, the team's fix is today's `face` + `driveTo` +
  `turnTo`** (every host run arrived by its steps' own verdicts; median 2.7–3.2 s, worst 4.3 s at a weaker side up to 9 %, 4.7 s at 15 %; on the TRUE pose its last turn leaves ~1 run in 8 0.5–1 in off at a weaker side up to 9 %). **The library adds NO back-up-and-re-approach**
  — prototyped and rejected (§5: it lands exactly more often than that fix but is slower than it, and it reverses
  blind up to 3 ft on the library's judgement, before the freeze; *amendment re-check B2*).
- **(b) Stop at the end point; do not judge the heading.** **Cost:** pure pursuit leaves the robot 2.5–10.7° off the
  end heading (P1, "ppend": a 12 in bend ends 10.7° off) — the next straight leg or turn has to fix it, and a look
  at a Toggle would be crooked.
- **(c) Roll through the end at speed into the next step** ("motion chaining"). That is G3a's D16 (b), not chosen;
  the master plan cut it from v1 (`docs/shulib-v2-master-plan.md` :515). OUT (§5).

**D5 — How fast it takes the bends, and who tunes it. RECOMMENDED (a).**
- **(a) RECOMMENDED — a sideways-grip limit: bend speed = √(grip ÷ bend tightness).** One number,
  `curve.bendAccel` (in/s², how hard the robot may be pushed sideways in a bend), sets the speed in every bend; the
  robot also slows smoothly to the end, and never exceeds the call's `maxLinearSpeed` or the robot's own. The
  library ships PROVISIONAL defaults — **look-ahead 8 in, bend grip 60 in/s² (about 0.16 g, gentle)** — chosen for
  ROBUSTNESS on the host test plant, not speed: they settle 13–17 of the 17 drivable routes under every weak side and
  motor lag tried (R-A, *re-check A1*), and cost ~15–20 % more time than the fastest setting (look-ahead 6 in,
  120 in/s²), which is better on a crisp robot and collapses on a laggy one (5–8 of 18 at a 0.18 s lag, P14, P15).
  The robot slows BEFORE each bend, not in it: the speed plan is worked backwards from every bend and from the end,
  at no more than the slowing rate, and never asks for more turn rate than the robot's limit (*re-check A6*: the
  first draft's formula braked only for the end; without the backward pass the host matrix drops 139 → 135). **Nobody can pick the right
  numbers off the robot**: the plant has no wheel slip and robot two's motor lag has never been measured. **The team
  tunes them at the robot** — the builder/driver with the coordinator's worksheet — in ONE session (Station C, §2.0),
  after Station M has passed: look-ahead first, then the bend grip, typed into the robot file's motion config.
- **(b) One fixed slow speed for every curve.** Safe, simple, slow on gentle bends — wastes the time a curve is for.
- **(c) A bend-speed field in every step's options (`MotionOptions`).** **Cost:** a field every other verb ignores (a
  silent no-op there — the reason STEP1's D1 (b) was rejected); the call's `maxLinearSpeed` already slows one curve.

**D6 — When the robot has LOST the curve: stop. RECOMMENDED (a).**
- **(a) RECOMMENDED — two guards, then stop and fail the step.** The curve stops (motors to 0 V), fails the step like a
  timed-out drive (one Warn naming the guard and its numbers; the `MotionTimeout` fault, as every motion that did
  not arrive raises), when the robot's heading is **more than 60° off the path's direction for 0.25 s**, or it has
  **made less than 1 in of progress along the path in 0.75 s** (outside the last 12 in, where the `moveTo` end is
  slow on purpose). Why: probed with one side going DEAD mid-curve, the follower without guards drove circles for the
  rest of its time budget — 43–84 in of travel, ending 11–106 in from where it should be (P7); with the guards it
  stopped 0.8–2.2 s after the side died, every time (P10). Across 153 healthy runs (17 routes — the start-behind one is
  refused by D3 — × 3 weak sides × 3 motor lags) neither guard fired once (P14; R-A again 0). The deadline still always
  stops (STEP1's D3). **What the guards do NOT cover** (*re-check A9*): (1) the last 12 in, where they are off
  because the end is slow on purpose — a side that dies there leaves the robot creeping until the step's own time
  budget ends it (host: TimedOut at the full 6 s, total travel 48–55 in on 54–55 in paths — bounded, but the whole
  budget); (2) **a jam on robot two**: its position comes from its drive encoders, so wheels spinning against a wall
  read as progress, and the library's spin-vs-motion check gives no verdict on such a robot (`motion_config.hpp`
  :193-197) — only the step's time budget stops it. Give a skills curve a `.timeout` near its real time.
- **(b) Only the time budget.** **Cost:** a dead side (an unplugged motor cable, a tripped breaker) drives the robot in
  circles across the field until the budget ends.
- **(c) Distance off the path only (e.g. 6 in).** **Cost:** a pivoting robot stays within 6–12 in of the path; it
  fired at 3.1–3.3 s in two of three dead-side runs and never in the third (P8).

**D7 — What re-opening C1's D12 covers, and what it does not. RECOMMENDED (a).**
- **(a) RECOMMENDED — the follower only, and only toward its own path.** C1's D12 (`docs/internal/chunks/C1-COMPLETED.md`
  :267-273): the library never plans a turn toward a target, because "building nonholonomic turn-toward-target
  planning into C1 would smuggle the LemLib behaviour into the library's core"; TANK1 amended it (:275-281): bounded
  steering toward the author's LINE is allowed, turning toward the POINT is not. *(re-check A10: the first draft
  recast that reason as "a surprise turn"; the record's reason is keeping LemLib's behaviour out of the core.)* Pure
  pursuit IS LemLib's `follow` (master plan :147), so the dated note under D12 (not a rewrite) says it plainly: **by
  the team lead's ruling of 2026-10-30, that LemLib behaviour is admitted into the library — inside the two curve
  verbs only.**
  **COVERS** — inside `curveTo` / `reverseCurveTo` only, the robot steers toward a point a look-ahead ahead of itself
  ON THE PATH THE AUTHOR DREW — or, in the last look-ahead, on the end's own line extended past the end; so it turns
  where the drawn path turns, including into the path's first bend.
  **DOES NOT COVER** — `moveTo`, `driveTo`, `reverseTo`, `strafeTo`, `turnTo`, `face`, `faceAway`,
  `followTrajectory`, `hold`, `brake` and the route runner are unchanged, bit for bit (a test pins it); the follower
  never aims at the END point itself, nor at anything but the drawn path and the end's line extended (*re-check
  A10*); no turn the path does not contain (a start more than 90° off is refused, D3; a path that doubles back is
  refused, D3); no turning round to drive backwards (reverse is the author's verb); the last inches are TANK1's
  `moveTo` with its rules (D4).
- **(b) A general re-opening** (every verb may aim at points). Not what the team lead said ("for the curve follower
  only"), and it would change every existing leg.

**D8 — The names. RECOMMENDED (a).**
- **(a) RECOMMENDED — `curveTo(points)` and `reverseCurveTo(points)`, the same words on the `Chassis` and on the
  `Routine`.** They read like `driveTo` / `reverseTo`, which the team already writes (L9). Each takes the step's
  usual options (`{.timeout = 3_s, .maxLinearSpeed = 40}` — and STEP1-A's tolerances), returns like `moveTo`, and the
  Routine steps join the chain like any drive step (a failure stops the chain, or carries on under STEP1-A's
  `onFail`). The point type (`CurvePoint`) never needs to be named in an auton: `{x, y}` and `{x, y, heading}` in
  braces.
- **(b) `followCurve` / `followPath`.** Fine words, but unlike the steps the team uses.
- **(c) One verb with a direction argument (`curveTo(points, Backward)`).** One more thing to get wrong in a long
  line.

**D9 — Other drivetrains. RECOMMENDED (a).**
- **(a) RECOMMENDED — the curve works on any drivetrain, driving like a tank** (nose along the path), and changes
  nothing else on any drivetrain. Both robots are tanks this season; an X-drive or H-drive bench robot can still run a
  team's skills route on the host test plant and the bench.
- **(b) Tanks only** (an X-drive refuses). **Cost:** a test route can no longer run on the bench bot; nothing gained.

**D10 — F8's `"arc"` (routes as DATA). RECOMMENDED (a): after the 21st.**
- **(a) RECOMMENDED — its own small run, CURVE1-B, after the 21st.** Nothing reads a route written as data before the
  21st: there is no panel, and the skills routes are written in code (the team lead, 2026-11-10). CURVE1-B appends
  `WaypointMotion::Arc = 7` (F8's additive path, `docs/roadmap.md` :142), `"arc"` to the file vocabulary
  (`docs/vexbuilder-integration.md` :281-290 promises exactly that), and teaches the route runner to drive a run of
  consecutive Arc waypoints as ONE curve. ~¾–1¼ builder-days.
- **(b) In CURVE1-A, now.** **Cost:** +~¾–1 day on the critical path, with a frozen-surface change (F8's pins, the
  manifest, the file schema) that nobody uses before the 21st.

**D11 — What the route preview draws now, and the route maker later. RECOMMENDED (a).**
- **(a) RECOMMENDED.** **Now (RPV1):** the preview runs the auton through the real library on the host test plant,
  so a curve is drawn as the robot DROVE it — and CURVE1-A also ships the pure path builder (`motion/curve_path.hpp`,
  no robot needed) so the preview can draw the PLANNED curve beside the driven one, the bends' speed limits, and the
  same refusal/warning words (start behind the robot; a path that doubles back; a bend tighter than the look-ahead).
  **Later (the panel's route maker, G3a Part R — after the 21st):** the author drags points and heading arrows; the
  panel (a TypeScript VS Code extension, which cannot call a C++ header) draws the PLANNED curve with a TypeScript PORT
  of the pure builder, pinned to the C++ one by golden vectors a C++ test writes (or a WebAssembly build of the same
  header — its own decision then), and writes `r.curveTo({…})` into the auton file and an `"arc"` waypoint run into
  the export (after CURVE1-B). Drawing the planned geometry is not a simulation, but the panel does NOT colour bends
  by speed: G3a's 10-08 ruling keeps every simulation out of the panel and its step-through "never shows speed" (G3a
  :592-593). *(re-check A11: the first draft said the panel would call "the SAME pure builder" under "the panel's
  own one-simulator rule" — a misreading; that rule forbids the panel simulating at all.)* Nothing here changes
  before the 21st.
- **(b) Let each tool draw its own curve.** Two geometries that will drift apart (the port in (a) is pinned against
  that).

**D12 — The runs, their order and the date. RECOMMENDED (a).**
*(re-check A5 — rewritten against the 11-10 ~23:30 queue, NOTES :963: BOTH robots run skills; robot one has no build
yet (G1); the builds queued are AF1S-A1, TUNE1-B1, STEP1-B, then CURVE1-A on its go.)*
- **(a) RECOMMENDED — CURVE1-A on STEP1-A's MERGED tip, BESIDE STEP1-B (not after it), in the first slot free after
  the three builds queued ahead of it; Station C on robot two only after Station M passes; `kUseCurves` OFF until
  Station C passes; the team lead decides each flip before the freeze.** CURVE1-A needs STEP1-A (its tolerances and
  `onFail` reach a curve with no new code, CV17; *amendment re-check B5: STEP1-A is BUILT and waits to merge third,
  after MECH-S-A and CAL1-B, NOTES :965*) but NOT STEP1-B (it adds no trigger to curves, §5). STEP1-B is
  ~3–4 builder-days and has not started: waiting for its tip would land CURVE1-A ~Nov 19–21, after the ~Nov 18
  freeze. Beside it, both edit `chassis.hpp`, `routine.hpp`, `version.hpp` and the F6/F10 pins in delimited blocks;
  **the second to merge re-reads the other's blocks** (the MECH and L6 briefs' rule, ~¼ day) and takes the next
  `kApiMinor` (STEP1-A takes 6; CURVE1-A takes 7 or 8 — assigned at the merge). With a slot by ~Nov 12–13 the code
  lands ~Nov 16–17 (§9). **Robot two** (built "by the 14th") gets Station C after Station M — in practice
  **~Nov 17–18**, one or two days. **Robot one** uses robot two's two numbers as a recorded belief (HA-246) and runs
  curves only if, once built, it passes C.1's short check itself (~20 min); otherwise its skills route drives
  straight legs. **The flip:** `kUseCurves` starts `false` (today's `face` + `driveTo` legs, no new code); the team
  lead decides each `true` from the Station C log read with the coordinator, and a student writes the line (G4).
  **Deadline: the ~Nov 18 freeze** — Station C's two numbers in the robot file and every `kUseCurves` line are code
  changes. **Asked of the team lead:** may those two numbers and that one line per robot change after the freeze, on
  the robot, during the test week? If not, a curve that is not trusted by ~Nov 18 does not run on the 21st.
- **(b) CURVE1-A on STEP1-B's merged tip.** No re-read of STEP1-B's blocks, but it lands ~Nov 19–21: after the
  freeze, so no tuned curve on the 21st.
- **(c) CURVE1-A after the 21st.** Safe for the competition, but not what the team lead asked for twice.

---

## The design in plain words (for the team lead)

- **What it does.** Today a tank route stops at every point: turn to face the next point, drive straight, stop, turn
  again. A curve lets the robot flow through the points without stopping. On the host test plant the same five
  routes took **2.6–5.2 s as curves (the gentle settings it ships with) against 4.0–11.2 s as today's stop-and-turn
  legs — about half to two-thirds of the time** (P11, P15); the faster settings Station C may allow took 2.3–4.3 s.
  *(re-check A8: the first draft quoted the fast settings' times.)* How much a skills run saves on the robot is
  unknown until Station C.
- **What the auton says.** `r.curveTo({{24_in, 12_in}, {48_in, 24_in, 0_deg}});` — "from here, pass through (24, 12),
  and finish at (48, 24) facing 0°". `reverseCurveTo` is the same, backwards. Options as on any drive:
  `{.timeout = 3_s, .maxLinearSpeed = 40}`.
- **How it drives.** The library draws one smooth path from where the robot is (and the way it faces) through the
  points. Every 10 ms it finds the point on the path 8 in ahead of the robot and steers along the arc to it — so it
  turns where the path turns, and cuts very tight bends a little. It slows in bends (how much: one grip number the
  team tunes), slows to the end, and for the last few inches does an ordinary `moveTo`, so it stops, lines up and is
  judged exactly like any drive.
- **When it refuses or stops — and says why.** It will not start a curve whose first point is behind the robot ("turn
  first: face()"), nor a path that doubles back on itself (a heading pointing back the way the path came, two points
  on top of each other) — it names the point. It warns about a bend tighter than it can follow. It stops and fails
  the step if the robot has lost the path — pointing more than 60° off it for a quarter second, or making no progress
  for three quarters of a second (a dead side) — except in the last 12 inches, where only the step's time budget
  stops it; a robot pushing against a wall with its wheels spinning also looks like progress to robot two, so only
  the time budget stops that too. The time budget and the match deadline always stop it. *(re-check A2, A9.)*
- **What it never does.** It aims only at points on the path you drew (or that path's end line, past the end) —
  never at the end point itself — never plans a turn the path does not contain, and changes nothing about any other
  step. *(re-check A10.)*
- **A sideways miss** *(amended 2026-11-10 late)*. A tank can't slide sideways, but a curve can arc back onto the
  line — if it starts far enough back: **2½ ft back fixes a miss up to 6 in, 3 ft up to a foot** (robot pointing
  within 20° of where it should end; this holds even with slow-reacting motors — *amendment re-check B1*). Closer than
  that it misses — from 6 in or more back by up to 3½ in, from closer by about as much as it started to the side —
  and it always runs until its time is up. Closer than that, use today's turn-drive-turn (`face`, `driveTo`,
  `turnTo`) — every step said it arrived in every test, in about 3 s, though on a weaker side about 1 run in 8 ended
  half an inch to an inch off the exact spot. The library will NOT back up and try again on its own: tested, it did
  land on the exact spot more often, but it was slower than turn-drive-turn and it drives backwards blind for up to
  3 ft (*amendment re-check B2*).
- **One rule to remember.** A heading is always where the robot's NOSE points, also when it drives backwards; and
  don't list the robot's own spot as the first point. *(re-check A2, A3.)*
- **What only the robot can tell us.** The right look-ahead and bend grip for robot two (its motor lag and its grip are
  unmeasured); whether its wheels skid in a fast bend; whether its odometry (drive encoders, no tracking wheels) stays
  as true in a curve as on a straight. One robot session (Station C) after Station M — both by ~Nov 18; robot one
  borrows robot two's numbers and checks them briefly once built. *(re-check A5.)*

---

## 0. Why — what exists, and the probes (every line cited was OPENED at `d684f34`)

### 0.1 Why this chunk, and why now
The team lead asked for curves three times (D16 (c) on 2026-10-30; "extremely important" on 2026-11-03; "Yes, keep
curves" and "when are we doing curves" on 2026-11-10). Skills is 60 s (G3a's D18) and every point of today's routes
costs a stop and a turn: the guide states the cost — "Stop-and-settle only: trajectories settle at every waypoint …
Measured cost: about 1.2 s per motion" (`docs/guide/14-what-it-cannot-do-yet.md` :437-439).

### 0.2 What exists in the tree
- **`moveTo` is decoupled per axis** (`include/shulib/motion/move_to_pose.hpp` :3-25): three PIDs, x / y / heading; on a
  tank the body's sideways command is clamped to 0 (`command_pipeline.hpp`, step 4 — "THIS layer clamps"), so a tank
  `moveTo` drives along its heading and steers only by TANK1's bend.
- **TANK1** (`motion/tank_steer.hpp` :1-53; `motion_config.hpp` :65-139; `move_to_pose.hpp` :218-306): a leg that
  starts within `maxSteer` = 15° (:100) of its target heading bends its heading setpoint toward the target's LINE by
  ≤ 15°, faded over the last 6 in (:105), steering on the cross-track error predicted `leadTime` = 0.06 s ahead
  (:116); a yaw observer cancels a weaker side, ≤ 1.5 rad/s (:127). "Nothing here ever aims the robot AT the point.
  That is the difference from a pursuit or boomerang controller" (`tank_steer.hpp` :11-14). A leg starting more than
  15° off is not steered and logs "turn first: face()" (`move_to_pose.hpp` :480-489). HA-179, invented
  (`docs/hardware-assumptions.md` :422).
- **`followTrajectory` stops at every point** (`chassis.hpp` :150-160, :326-355): sequential `MoveToPose` legs,
  "stop-and-settle is v1's documented motion model; blending is a measured Frontier item … a richer Trajectory type
  arrives as an ADDITIVE overload, never a reshape of this one."
- **The facade is frozen, additively open** (`chassis.hpp` :5-22: "additive extension (new verbs, new MotionOptions
  fields, new overloads) stays legal and is the intended growth path"; F6, `docs/roadmap.md` :140). **`MotionOptions`**
  (:206-239) — "the field SET is deliberately additive-open". `effectiveConfig` (:571-580) maps options onto a copy of
  the config — the seam STEP1-A extends (`step1-a` ~:614-637).
- **The Routine** (`chassis/routine.hpp`): `driveTo` computes its bearing from the LIVE estimate "when this step RUNS"
  (:304-320) — the pattern a curve's start follows; `reverseTo` (:322-344, L9, API 2.4) is the reverse-naming
  precedent; `followTrajectory` (:385-410) fails the step on an incomplete trajectory; F10 (`roadmap.md` :144):
  thirteen pinned steps, L9's appended `reverseTo` / `faceAway` "by the additive path this row allows, pinned the day
  they landed".
- **F8** (`roadmap.md` :142): `WaypointMotion` values `Start`=1 … `Hold`=6 frozen, "new enumerators AT THE END" are
  additive; "`motion: "arc"` … is a **named gap** rather than an omission (no facade verb performs an arc, and F8 must
  not freeze something the library cannot do — it arrives additively with the roadmap's blending item)";
  `config/route.hpp` :155-178; `docs/vexbuilder-integration.md` :281, :286-290.
- **The master plan**: LemLib's motion is "Boomerang `moveToPose`, pure-pursuit `follow` — heading coupled to travel
  direction" (`docs/shulib-v2-master-plan.md` :147); v1 cut "min-velocity motion chaining (stop-and-settle first)"
  (:515); the capability catalogue's "Tiered" row lists "In-lib spline/`Squiggles`-style smoothing;
  velocity-constrained profiles" (:685). No locked decision (§13, :540-560) forbids a curve verb; decision #5 (the
  strafe clamp, the motion layer's job) is untouched — a curve commands no sideways speed.
- **C1's D12 and why** (`docs/internal/chunks/C1-COMPLETED.md` :267-273): "Building nonholonomic turn-toward-target
  planning into C1 would smuggle the LemLib behaviour into the library's core"; TANK1's dated amendment :275-281. The
  reason, in the record's own words, is keeping LemLib's behaviour out of the library's core — and pure pursuit IS
  LemLib's `follow`. The team lead's ruling admits it anyway, for the curve follower only; D7 words the dated note so
  it says exactly that, and keeps every other verb as D12 left it. *(re-check A10: the first draft recast the reason
  as "a surprise turn".)*
- **The scheduler's result line** (`motion_scheduler.hpp` :1154-1176 at `16c01e0`, *re-check A12*): name, verdict, a
  target, overshoot along the start→target line, the ` xt` column on a tank (`hasCrossTrack`, judged
  `strafeAuthority() == 0.0`, :1158-1159).
  A motion publishes its target in each record; a curve publishes its END pose (§1.2) — so ` xt` is "off the end's
  line" and `ov` is along the start-to-end chord (a landmine, §8 L7).
- **The tick and the stack.** A blocking verb's motion lives on the calling task's stack (`chassis.hpp` :295-299:
  `motion::MoveToPose m{…}; return runBlocking(m);`); PROS gives a task 32 KiB (`hal/pros/calibrate.hpp` :36-42,
  measured there at ~70 % for Calibrate). `sizeof(MoveToPose)` is 1 376 B on the host (P12).
- **STEP1** (`docs/internal/chunks/STEP1-route-steps-library.md`): STEP1-A appends two tolerances to `MotionOptions`
  and `loosenedSettle` to `motion_config.hpp` (:393-410), bumps `kApiMinor` 5 → 6 (:434); STEP1-B adds `LegProgress`
  (:481-487, "stays open to new fields (CURVE1 may add one)", :72-73) and observer overloads, `kApiMinor` 6 → 7
  (:552); its coordination table: "CURVE1 (after the route maker) | `chassis.hpp` `MotionOptions` | after STEP1-A"
  (:857), "it also edits `MotionOptions` (after STEP1-A, which therefore keeps its two fields at the end of the
  struct)" (:820-821). **This brief needs NO new `MotionOptions` field** (D5 (c)) — it edits `chassis.hpp` for the
  verbs only.
- **`boot_source_test.cpp` pins `MotionConfig`** — member count 13 (:244-245), `sizeof` 416 (:264), `ChassisConfig`
  464 (:268), and `requireDefaultTuning()` compares every leaf (:271-…). A new `MotionConfig::curve` member trips all
  three by design (TANK1's §0.5 item 1 found the same).

### 0.3 What the field and the season give
Skills only at this competition; 60 s for skills (G3a's D18). The field is ±70.2 in wall to wall (L6's
`override_walls.hpp`). Robot two: blue cartridges, 2.75 in wheels reported — about 86 in/s free speed at 600 rpm if
direct drive (not confirmed, `chassis_table.hpp` :107-123); track width UNSET (Calibrate discovers it). The
library's speed budget is 60 in/s (`motion_config.hpp` :164), PROVISIONAL (HA-50).

### 0.4 The probes (`.claude/coordinator/scratch/brief-CURVE1/`, run by this writer 2026-11-10; every binary under `ulimit -v 4 GiB` + `timeout`)
**What they are.** `probe_curve.cpp` — prototype followers built ONLY from the tree's own parts (the command
pipeline, the Localizer, `SettledUtil`, and the real `MoveToPose` for the end), driven on the host test plant's tank
(`TankKinematics` 12 in, `motion_rig::plantConfig()`, the same rig TANK1's probes used), compiled against the tree
read-only (`build.sh`). Laws: `pp` (pure pursuit + yaw observer), `ppnoobs` (no observer), `ppend` (pursuit to the
very end, position-only arrival), `tan` (D1 (c)), `boom` (D1 (b), carrot lead 0.6, the end handed to `MoveToPose`
inside 7.5 in). Hostility: a weaker RIGHT side (×0.91, ×0.85 — TANK1's grid), motor lag as the plant's `kA`
(τ = kA/kV: 0.12 and 0.18 s — TANK1's re-check range), a side going dead mid-curve, a cancel mid-curve.
`probe_through.cpp` — the same routes TODAY's way, through the real `Routine` (`face` + `driveTo` per point, then
`turnTo`). `probe_sizes.cpp` — object sizes. `matrix.sh` — every route × weak side × lag, one summary line per cell.
**Eighteen routes:** a straight 48 in; an S (`scurve`); a lane change; 90° bends of 36, 18 and 12 in (tightest radii
22.7, 11.4, 7.6 in); hairpins of 18, 9 and 6 in half-width (radii 10.9, 5.7, 4.0 in); starts 30°, 60°, 90°, 150° off
the first chord; a 143 in snake; a 159 in field S; the S driven backwards (`revS`); an S ending with no heading
(`endfree`); a path that crosses itself (`crossing`). Grades are against the plant's TRUE pose.

| # | What | Result | File |
|---|---|---|---|
| **P1** | first pass: hand-off when 8 in from the end along the END's line, heading ≤ 15° | **wrong rule, found by the probe**: on a hairpin the end's line passes through the start, so the hand-off fired at t = 0 with the robot 180° off (no motion, TIMEOUT); on bends it fired 17–42° off the end heading → TANK1 "not steered" → TIMEOUT. The hand-off must be judged by progress ALONG THE PATH. `ppend` (no hand-off): heading at arrival **2.5–10.7° off** (D4 (b)); `off150`'s path has a 0.5 in bend — a loop (D3) | `out/p1_clean.txt` (re-run after the fix; the first pass is reproduced by reverting the rule) |
| **P2** | pursuit look-ahead 6 / 8 / 12 in, lag-free | **L 6: 17/17**, worst 0.71 in off the path (bar `off150`); **L 8: 16/17** (the 6 in hairpin TIMEOUT, 1.32 in); **L 12: 12/17**, 2.87 in — a long look-ahead cuts tight bends | `out/p2_pp_lookahead.txt` |
| **P3** | the laws × weak side × lag (hand-off ≤ 15°, ≤ 0.3 in off the end's line) | lag-free: `pp` 17/17 at every weak side; `ppnoobs` 16/17 at ×0.85; `tan` 15/17 at ×0.91, **11/17 at ×0.85**. At τ 0.12 s every law drops (pp 9/17 at ×1.00): the failures are at the END | `out/p3_matrix.txt` |
| **P4** | the hand-off at ≤ 5° instead of 15° | pp at τ 0.12: **15/17, 17/17, 17/17** (×1.00/×0.91/×0.85); τ 0.18: 6–8/17 | `out/p4_handoff.txt` |
| **P5** | why τ 0.18 fails | the path is followed within 1–2 in, then the `moveTo` end leaves 0.5–2.6 in (TANK1's own short-leg limit, `tank_steer.hpp` :24-36); on the long routes the 6 in look-ahead oscillates (6.5 in off) | `out/p5_lag018.txt` |
| **P6, P13, P14** | *(the counts below include `off150`, which D3 refuses, and the ungated last resort — R-A supersedes them for the recommended defaults, re-check A1)* candidate defaults: **R** = look-ahead 8, grip 60, hand-off 12 in / 5° / 0.3 in (last resort at 4 in), end speed ×0.6, guards on; **F** = look-ahead 6, grip 120, hand-off 8 in | **R: 14–17 of 18 in EVERY cell** (×1.00/.91/.85 × τ 0/.12/.18), worst 2.5 in off the path; misses the 6 in hairpin (radius 4 < half the look-ahead, D3's warning) and, at ×0.85 with no lag, two tight bends, the snake and `revS`. **F: 18/18 lag-free at every weak side, 15–18 at τ 0.12, 5–8 at τ 0.18** (6.5 in off the path). P13 found the no-progress guard false-alarming 10 times inside the slow end zone → the guard is off in the last 12 in and the 4 in last-resort hand-off was added; P14: **0 false alarms in 153 runs** (17 routes × 9 cells) | `out/p6_end.txt`, `out/p13_defaults.txt`, `out/p14_defaults2.txt` |
| **P7** | adversarial: a dead side, a weak one, a cancel, the global-search mutation, boomerang | **dead right side from 0.8 s, no guard: drives circles for the whole 6 s budget, 43–84 in travelled, ends 11.7–106 in from the end**; one motor of five out (×0.8): 4/4 settle; cancel at 1.0 s: stops at once (prototype); closest-point search over the WHOLE path did NOT fail in closed loop on hairpins or the crossing path (the robot stays on its own leg) — so the mutation needs a pure-function test (§2.2 CV9); boomerang single-pose curves: **0/6 settle** (0.55–2.35 in, 13–26° at its hand-off), pursuit 6/6 | `out/p7_adversarial.txt`, `out/p8_guard_crossing.txt` |
| **P8–P10** | the "lost the path" guard | distance-only (6 in for 0.2 s): fired at 3.1–3.3 s in two dead-side runs, never in the S (D6 (c)); heading-off-path maxima on HEALTHY runs: 13.9° lag-free, 22.2° at τ 0.12, 47.3° at τ 0.18 → 60° keeps a margin; **heading 60°/0.25 s + progress < 1 in/0.75 s: a dead side stopped 0.8–2.2 s after it died, 5/5** | `out/p8_guard_crossing.txt`, `out/p9_headguard.txt`, `out/p10_progress_guard.txt` |
| **P11** | the time a curve saves (lag-free / τ 0.12) | today's `face`+`driveTo`: snake 11.23 / 9.55 s, field S 10.42 / 8.64, hairpin 18 6.69 / 5.46, 90° bend 4.01 / 3.73, S 5.07 / 3.83 — **curve (F): 4.26 / TIMEOUT, 4.19 / 5.10, 3.05 / 3.13, 2.27 / 2.15, 2.51 / 2.28**; with R ~15–20 % slower (P15): snake 5.24, field S 4.86 | `out/p11_through_points.txt`, `out/p15_times.txt` |
| **P12** | sizes on the host (x86-64; the V5 is 32-bit, smaller) | `MoveToPose` 1 376 B, `TurnTo` 1 008, `MotionConfig` 416, `ChassisConfig` 464, `MotionOptions` 24 (STEP1-A: +16), `Routine` 128 | `out/p12_sizes.txt` |
| **P17** | R's full matrix, the END of every settled run | **145 settled of 162 runs (162 = 18 routes × 9 cells): worst 0.50 in, worst 1.01° from the end pose** (true pose) — *SUPERSEDED by R-A (re-check A1): 140 of the 153 drivable runs, 0.50 in, 0.74°; 25 of P17's runs handed off outside TANK1's window* | `out/p17_R_full.txt`, `out/p17_summary.txt` |
| **R1–R6** *(re-check)* | `recheck/`: the gated hand-off, the seeded observer, hand-off variants, the 12 in window, deaths in the end zone, the path refusals, the trimmed sizes | **R-A: 140/153 settled (16 16 14 / 16 16 16 / 13 17 16 per cell), hand-offs > 15° 25 → 2, worst settled end 0.50 in / 0.74°, 0 guard trips, worst heading off the path 24.6°**; seeded observer 136–137 (rejected); hand-off 16 in: 139; last resort 6 in: 131; window 12 in = 40 in (140); deaths at 1.4–2.2 s inside the end zone: TimedOut at 6 s, travel bounded; refusals: 17/17 drivable routes pass, 7 broken shapes refused; CurveFollow ≈ 5.35 KB host | `recheck/*.txt`, `refusals_all.py`, `sizes3.cpp` |
| **P16** | `tan` and `boom` with the same end as R/F | `tan`: 18/18 lag-free ×1.00, **11/18 at ×0.85**, 5–12/18 with lag; `boom` on multi-point routes ignores the middle points (31–39 in off the drawn route) | `out/p16_tan_boom_fair.txt` |

**Not probed (reasoned, and said where used):** robot two's drive-ENCODER odometry (the rig uses the plant's tracking
wheels; HA-249); IMU noise and the composed hostility models; wheel SLIP in a bend (the plant has none; HA-247); the
ARM per-tick cost and stack (HA-250, measured by the ARM gate's `-fstack-usage` in §2.3); an X-drive or H-drive
curve (D9; the test suite covers it, §2.2 CV20). **The prototype is NOT the design**: it uses `std::vector`, `new`
and dense samples; §1.1 bounds memory.

---

## 1. Scope — IN (D1 (a), D2 (a), D3 (a), D4 (a), D5 (a), D6 (a), D7 (a), D8 (a), D9 (a), D10 (a), D11 (a), D12 (a) assumed)

### 1.1 CURVE1-A — `include/shulib/motion/curve_path.hpp` — NEW, PURE (no chassis, no scheduler, no PROS, no heap)
- `struct CurvePoint` — `units::Length x, y; math::Angle heading; bool hasHeading;` with two constructors,
  `CurvePoint(Length x, Length y)` (no heading) and `CurvePoint(Length x, Length y, math::Angle heading)`, so `{24_in,
  12_in}` and `{48_in, 24_in, 0_deg}` brace-initialise in a `std::initializer_list<CurvePoint>` (D8). Finite x, y is a
  precondition of the verbs (like `followTrajectory`'s atomic check, `chassis.hpp` :333-341). **`heading` is the
  robot's NOSE on both verbs** (reverse included — the header says so in its first lines; *re-check A2*).
  **`CurvePoint` is F6-FROZEN** as the verbs' parameter type — its members and its two constructors pinned (CV22);
  NO default constructor (*re-check A12*: nothing needs one after A4, and it would freeze a third constructor meaning
  "a point at the field's origin").
- `inline constexpr std::size_t kMaxCurvePoints = 16;` — a design constant (a route longer is two curves), not a
  hardware belief. `inline constexpr double kCurveMinSpacing = 1.0;` (in) — a design constant, twice the translation
  settle (*re-check A3*): a headingless first point closer than this to the live start is DROPPED (one Info line),
  a headed one refused; consecutive points closer than this refused.
- `class CurvePath` — FIXED capacity, **trimmed** (*re-check A4*): the knots — the start (filled at the first live
  tick) + ≤ 16 points, held here FROM CONSTRUCTION (no second copy anywhere), as the path's own knot type; per-knot
  unit tangents; the cumulative length at each knot; and, per segment, **17 samples as `float`** of the arc length
  AND of the speed plan (≤ 16 × 17 × 4 B × 2 ≈ 2.2 KB). NO stored coefficients: the cubic Hermite (tangent magnitude
  = the segment's chord) is evaluated from the two knots and their tangents. Built in two steps — the points at
  construction (`CurvePath(std::span<const CurvePoint>, bool reverse)`), then
  `void build(const math::Pose2d& start, const CurveConfig&, double maxLinear, double maxAngular) noexcept` at the
  first live tick — a free point's tangent is the direction from its neighbour before to its neighbour after
  (Catmull–Rom); a free LAST point's is from the point before; `reverse` flips the start tangent (the robot's back
  leads) and every heading's travel direction (heading + π).
  Members: `length()`, `at(double s) → {x, y, heading, curvature}` (evaluated ANALYTICALLY from the knots, tangents
  and the arc-length table — no dense sample buffer), `project(pose, double fromS, double windowIn) → double s` (the
  closest point, searched FORWARD ONLY from `fromS` over `windowIn` of arc — never backwards, never the whole path: a
  path that comes back near itself must not make the robot jump to a later or earlier leg), `lookAheadPoint(double s,
  double L)` (past the end: on the end's tangent, extended), `tightestRadius()`, `startAngleOff()` (the angle between
  the robot's heading (or its back) and the bearing to the first point), `speedLimitAt(double s)` — **the speed plan**
  (*re-check A6*), interpolated from the per-sample table `build` fills ONCE: at each sample the least of √(bendAccel
  ÷ the largest |κ| within the next 2 in), **maxAngular ÷ |κ|** (L11: the robot's turn-rate limit is never asked to
  carry the arc), and `maxLinear` (the call's, else the robot's); 0 at the end; then a **BACKWARD pass**
  v[i] = min(v[i], √(v[i+1]² + 2·decel·Δs)) so the robot is already at a bend's speed when it reaches the bend —
  what the probes ran (`probe_curve.cpp` :143-161); without it the host matrix drops 139 → 135 of 153.
- `[[nodiscard]] CurveCheck checkCurve(const CurvePath&, const CurveConfig&) noexcept` — the D3 verdicts, with the
  numbers and the point's index: `Ok`; `Invalid` (refuse, *re-check A2/A3*: a point within `kCurveMinSpacing` of the
  one before it; a free point whose two neighbours are within `kCurveMinSpacing` of each other; any point's direction
  more than `maxStartOff` (90°) from the chord arriving at it or the chord leaving it; a headed first point at the
  robot; no point left); `StartBehind` (refuse: `startAngleOff > 90°`); `TightBend` (warn: `tightestRadius <
  lookAhead / 2`). The order: `Invalid`, then `StartBehind`, then `TightBend`. The free-point check runs BEFORE its
  tangent is normalised (the probe divided by zero there). One function the follower, the docs' examples and RPV1
  share (D11). (Why no runtime "derivative reverses" check: with every direction within 90° of both chords and tangent
  length = chord, P′(u)·c/|c|² = 6u(1−u) + (1−u)(1−3u)·cos a + u(3u−2)·cos b ≥ 0 on [0, 1] — the amendments' A2 —
  so it is a TEST property, CV1.)
- **What the preview reads** (D11): `CurvePath`, `build`, `length`, `at`, `speedLimitAt`, `checkCurve`. Nothing else
  is promised. **`CurvePath`, `checkCurve` and `CurveConfig` are NOT frozen** (outside F6, like `MotionConfig`); only
  `CurvePoint`, as the verbs' parameter type, is (*re-check A12*).

### 1.2 CURVE1-A — `include/shulib/motion/curve_follow.hpp` — NEW: `class CurveFollow : public IMotion`
- Constructed `(const MotionDeps&, std::span<const CurvePoint>, const MotionConfig&, double timeout, bool reverse)`;
  copies the points (≤ 16) into its `CurvePath`'s knots — the ONLY copy (*re-check A4*); preconditions: 1–16 points,
  every x/y finite (refused at the door, all before any motion). `name()` = `"CurveTo"` / `"ReverseCurveTo"` (the result line and the `MotionTimeout` text say
  which).
- **Wait-for-live** (`motion.hpp`'s contract, as `MoveToPose` :185-209): zero volts while the estimate is
  Uninitialized; the watchdog runs through the wait; the path is BUILT at the first live tick from the live pose
  (never at construction: the boot window has no heading — the same reason TANK1 decides at the first live tick,
  `move_to_pose.hpp` :218-228). At that tick: the near-start drop (one Info line, *re-check A3*), then `checkCurve` →
  `Invalid` or `StartBehind` exits at once (TimedOut + `MotionTimeout` "curve not driven: …" naming the point, one
  Warn, motors 0 V — D3), `TightBend` logs one Warn and drives; and ONE Info line:
  `curve: 4 points, 143.4 in, tightest bend 7.4 in radius, bends at <= 21 in/s` (the §18.3 formatter,
  `diag/line_format.hpp`, ASCII).
- **Pursuit phase, each tick:** `s = project(pose, s, window)` (window = 12 in of arc, a `CurveConfig` field);
  look-ahead point at `s + lookAhead`; in the robot's frame (its back's frame for reverse) the arc's curvature
  κ = 2·y / (x² + y²); forward speed v = min(`speedLimitAt(s)`, `endSpeedGain × translation.kP × remaining arc`, the
  last commanded speed + `accel × dt`); ω = v·κ minus the yaw observer's estimate — the SAME observer TANK1 uses
  (measured − commanded yaw rate, low-passed over `tankSteer.yawObserverTime`, clamped to
  `tankSteer.yawCorrectionLimit`), read from `loc.twist()` like TANK1's (`move_to_pose.hpp` :274-294; P14 of TANK1:
  never a second IMU read). The command goes through `applyCommandPipeline` in the BODY frame with vy = 0
  (`command_pipeline.hpp`: ω clamp, norm cap, the strafe clamp, inverse kinematics, the uniform desaturate — which
  scales both sides alike, so the arc's curvature survives a saturated tick), then `OdoStallCheck` and
  `tickHealthObservables` exactly as `MoveToPose` :340-343.
- **The observer's code is NOT extracted from `MoveToPose`** in this run: it is ~10 lines, and moving it would touch
  a class whose records TANK1's tests pin bit for bit. It is written once more in `curve_follow.hpp`, citing
  `move_to_pose.hpp`, and a test pins that the two agree on the same inputs (CV14). (A later shared helper is a
  refactor for its own run.)
- **Guards (D6), pursuit phase only:** heading more than `lostHeading` (60°) off the path direction at `s` for
  `lostTime` (0.25 s), or progress along the path < 1 in over `stuckTime` (0.75 s) while more than `handoffDistance`
  (12 in) of path remain → exit TimedOut, `MotionTimeout` raised naming the guard ("curve lost: heading 71 deg off the
  path for 0.25 s" / "curve lost: 0.4 in of progress in 0.75 s"), one Warn, motors 0 V.
- **The end (D4)** — *re-check A1, rewritten:* the hand-off fires on the FIRST of
  1. remaining arc ≤ `handoffDistance` (12 in) AND heading within `handoffHeading` (5°) of the end heading AND
     ≤ `handoffCross` (0.3 in) off the end's line (the clean hand-off);
  2. remaining arc ≤ `handoffDistance × handoffLast` (⅓ → 4 in) AND `motion::tankLegEngages(errH, cfg.tankSteer)`
     — the SAME function `MoveToPose` decides its steering with (`tank_steer.hpp` :106-109), on the same pose, so
     the member is always steered (the last resort; until then the follower keeps pursuing, along the end's line
     extended once the look-ahead passes the end);
  3. the closest point reaches the end of the path (remaining arc ≤ 0.125 in — what the probe measured: its closest
     0.25 in sample was the last; an exact 0 could be approached forever while the speed fades with the remaining
     arc) while still outside the window — the
     follower hands off anyway, with ONE Warn in the curve's own words ("curve end: 21.1 deg off the end heading at
     the end of the path (steers within 15.0) -- unsteered; more run-up, a gentler last bend or a lower speed into
     it" — *"more run-up" amended 2026-11-10 late*: on the sideways-miss probe a curve started too close for its
     sideways miss is the commonest way to reach case 3).

  *(amended 2026-11-10 late)* **No re-approach.** The end adds NOTHING for a robot too close to arc in — no back-up,
  no second attempt (prototyped and rejected, §5). A too-close curve ends as above: the member times out at the
  step's WHOLE budget — ≤ 3.5 in off from 6 in or more back, about its starting sideways miss (≤ 4.2 in) from closer,
  on the host test plant (*amendment re-check B3*) — its ` xt` in the result line; the author's fixes are the
  run-up rule and `face` + `driveTo` + `turnTo` (D4, §3.1).

  The MEMBER is a `std::optional<MoveToPose>` (no heap) EMPLACED at the hand-off (*re-check A4*: the config is chosen
  then) with the same deps, the follower's `MotionConfig` copy and its FULL timeout, through the public constructor
  (TANK1 on) — except in case 3, where it is built from a copy with `tankSteer.maxSteer = 0` (documented as
  switching TANK1 off, `motion_config.hpp` :202): the same unsteered line the member would have driven, WITHOUT its
  "tank moveTo not steered … turn first: face()" Warn (`move_to_pose.hpp` :480-489), which would blame the author
  for a turn they never had to make. No member Warn can therefore appear inside a curve (CV6 pins it). The member's
  yaw observer starts from 0 as on any `moveTo` — carrying the curve's estimate in was probed and made the matrix
  WORSE (136/153), so it is not done. From the hand-off on the follower returns the member's verdict. The member's
  watchdog has the FOLLOWER's full timeout, so the follower's own watchdog — checked first every tick — always fires
  first: a curve that runs out of time times out as `CurveTo`, never as `MoveToPose` (CV13). The member's records
  ride the same stamped deps (one command id). `curveStatus()` reports which of 1–3 fired and the heading then.
- **The end pose**: the last point's heading if it has one; else the path's own direction at its end (+π for reverse:
  the robot finishes with its back along the path). Published as `targetPose` in every record, so the result line's
  ` xt` is "off the end's line" (§0.2).
- **Records (A1: at most one per tick, lazy, motors stopped before the exit record):** `targetPose` = the end pose;
  `errorX/Y/Heading` to the end pose; `commanded` the final achievable command in the FIELD frame (as `MoveToPose`
  :346-358); `tankSteerActive` lights when the yaw observer's correction exceeds `kTankSteerNoiseFraction ×
  maxAngularSpeed` (the same legibility floor, `tank_steer.hpp` :122-132) — the record schema does NOT change (F9's
  candidate, the blackbox encoding — no new field).
- **`curveStatus()`** (tests and Tier-3, like `tankSteerStatus()` `move_to_pose.hpp` :406-418): phase (waiting /
  pursuing / ending / done), `s`, path length, the look-ahead point, curvature, speed limit, the observer's estimate,
  which guard fired, the start check's verdict.
- **`cancel()`**: the shared safe state (`applyCancelSafeState`), the member `MoveToPose` cancelled too if it was
  started, the verdict only if still running — the cancel contract, idempotent (`move_to_pose.hpp` :362-388).
- **Size** (*re-check A4*): **`static_assert(sizeof(CurveFollow) <= 6144)`** on the host, from a stated count
  (`recheck/sizes3.cpp`, a mirror of this layout): `CurvePath` ~3,144 B + the optional member `MoveToPose` ~1,504 B
  (its own `MotionConfig` copy grows by `CurveConfig`) + the follower's `MotionConfig` copy ~536 B + state ~170 B ≈
  **5.35 KB**. The first draft's ≤ 4 KB could not hold (the checker's mirror: ~5.5 KB). The real gate is the ARM
  `-fstack-usage` frame (§2.3), flagged above 8 KB — the stack rule, §8 L4.

### 1.3 CURVE1-A — `include/shulib/motion/motion_config.hpp` — CHANGED (its layer's, outside F6)
- `struct CurveConfig` — `lookAhead` 8.0 in, `bendAccel` 60.0 in/s², `accel` 200.0 in/s² (speeding up), `decel`
  120.0 in/s² (slowing to the end), `searchWindow` 12.0 in, `handoffDistance` 12.0 in, `handoffHeading` 5° (rad),
  `handoffCross` 0.3 in, `handoffLast` ⅓, `endSpeedGain` 0.6, `lostHeading` 60° (rad), `lostTime` 0.25 s,
  `stuckTime` 0.75 s, `maxStartOff` 90° (rad) — every one `PROVISIONAL (A4: HA-246)` (the guards' three: HA-248), with
  a `valid()` (finite; lengths, accelerations and times > 0; angles in (0, π); `handoffLast` in (0, 1]; `lookAhead` <
  `handoffDistance` is NOT required — say why in the comment). *(re-check A13: the first probes searched a 40 in
  window; `searchWindow` 12 in was re-run and gives the identical 140/153.)* The last-resort hand-off's heading window
  is NOT a `CurveConfig` field: it is `tankSteer.maxSteer`, read through `tankLegEngages()`, so the two can never
  disagree (*re-check A1*).
- `MotionConfig` APPENDS `CurveConfig curve{};` LAST (designated initializers keep compiling, as TANK1's `tankSteer`
  :201-205); `validate()` descends into `curve.valid()` (TANK1's reasoning :217-218: nothing else owns it).
- STEP1-A's `loosenedSettle` is untouched; the curve reads `translationSettle` / `headingSettle` only through its
  member `MoveToPose`, so STEP1-A's tolerances reach a curve's end with no new code (CV17).

### 1.4 CURVE1-A — `include/shulib/chassis/chassis.hpp` — CHANGED (F6-additive)
- `using CurvePoint = motion::CurvePoint;` in `shulib::chassis`.
- Four verbs, after `followTrajectory` (:357-362): `control::ExitReason curveTo(std::span<const CurvePoint>, const
  MotionOptions& = {})`, its `std::initializer_list<CurvePoint>` overload, and the same two for `reverseCurveTo`. Each:
  `options.validate()`; every point finite (atomic, before any motion); `motion::CurveFollow m{sched_.deps(), points,
  effectiveConfig(options), options.timeout.value(), reverse}; return runBlocking(m);` — the blocking verb's shape
  (:295-299). **No `MotionOptions` field** (D5 (c)).
- The banner gains a `═══ curveTo / reverseCurveTo ═══` section (what it does, D3/D4/D6/D7 in two sentences each),
  the verbs line (:31) gains the two names, and the TANK STEERING item (:99-124) one sentence ("the curve verbs steer
  toward a point ON their path — the only verbs that do, C1 D12's dated note").
- **STEP1-A's two field comments** (`positionTolerance` / `headingTolerance`, `step1-a` worktree `chassis.hpp`
  :230-262 — each lists the verbs that read it): `curveTo` / `reverseCurveTo` are added to both lists ("through the
  curve's end `moveTo`"). A comment edit only — no new field (*re-check A12*).

### 1.5 CURVE1-A — `include/shulib/chassis/routine.hpp` — CHANGED (F10-additive), one delimited block "CURVE1: curves — BEGIN/END"
- `Routine& curveTo(std::span<const CurvePoint>, const MotionOptions& = {})`, its `initializer_list` overload, and the
  same two for `reverseCurveTo` — each `motionStep("curveTo", …)` (:804-811): a non-Settled exit is a
  `MotionFailed` stop (or carries on / skips under STEP1-A's `onFail`), its verdict in `RoutineResult::exit`. No new
  `RoutineStopCause` (a refused start and a lost curve are motion failures with a `MotionTimeout` fault and a Warn
  that names them).
- The banner's tank paragraph and `driveTo`'s comment (:304-311) gain one sentence: "to flow through points instead of
  stopping, `curveTo`".

### 1.6 CURVE1-A — `include/shulib/version.hpp` — `kApiMinor` → the next free number at the merge (STEP1-A takes 6,
STEP1-B 7 — `STEP1-route-steps-library.md` :434, :552; CURVE1-A takes 7 if it merges before STEP1-B, else 8 — it now
runs BESIDE STEP1-B, D12, so either; the trunk is at 5, `16c01e0`), with its history paragraph (F6 + F10 additive: four
verbs, four steps, one type alias).

### 1.7 CURVE1-A — tests (§2.2)
**NEW** `test/curve_path_test.cpp` (pure), `test/motion_curve_test.cpp` (the host test plant; *amended 2026-11-10 late:* it also holds CV23, the run-up rule), `test/routine_curve_test.cpp`
(the facade, the Routine, the run guard); **CHANGED** `test/f6_signature_pin_test.cpp` (the four verbs' exact types, the
alias), `test/routine_signature_pin_test.cpp` (the four steps, non-`noexcept`, returning `Routine&`),
`test/boot_source_test.cpp` (`memberCount<MotionConfig>` 13 → 14, a `memberCount<CurveConfig>` pin, the two `sizeof`
pins re-measured, `requireDefaultTuning()` compares every `CurveConfig` leaf — §8 L5), `test/cookbook_examples_test.cpp`
(the new recipe, §3.1), `test/CMakeLists.txt`.

### 1.8 CURVE1-A — `src/team/autons.cpp` — NOT touched by the library run
The skills route is the team's (G4 guardrail: students author competition code). The cookbook recipe (§3.1) shows the
`kUseCurves` fallback pattern; the coordinator offers it to the team at Station C (§2.0).

### 1.9 CURVE1-B (after the 21st, D10) — F8's `"arc"`
- `config/route.hpp`: `WaypointMotion::Arc = 7` APPENDED (pinned in `test/f8_signature_pin_test.cpp`, :108-…'s pattern);
  `waypointMotionWord` → "Arc"; `valid()` rules for the NEW value only (a new enumerator's rules are not a
  tightening): an Arc waypoint is legal at index ≥ 1; a run of consecutive Arc waypoints is one curve of ≤ 16 points
  ending at its last; `headingMode` Fixed = a point with a heading, Free = without; `reverse` legal on an Arc run (all
  of a run the same, else refused naming the index); markers only on a run's LAST waypoint (the runner fires markers
  after a leg; a middle-of-curve marker would fire late — refused by name, until partway triggers reach curves).
- `runner/path_runner.hpp`: gathers the run, calls `curveTo` / `reverseCurveTo`; the bit-identity test G2 used (data vs
  `Routine`, `roadmap.md` :142) for an arc route.
- `runner/manifest.hpp` only if it lists motion words (read first); `docs/vexbuilder-integration.md` §4.1.3 :281 the
  row moves from DROPPED to `"arc"`; the F8 register row; `version.hpp` next minor.

---

## 2. Step 0, the runs, their tests and mutations, and the definition of done

### 2.0 STEP 0 and Station C at the robot — what a student measures (the coordinator writes the worksheet with the plan commit)
- **C.0 — Station M first.** No curve session before the 40 in `moveTo` has run on the robot and its log shows the
  library's motion stack driving (REMAINING-WORK :176-181). The curve's end IS that `moveTo`.
- **C.1 — The two numbers, at half speed** (`{.maxLinearSpeed = 30}`), on a clear field, a person on the
  controller's disable: an S (`{{24_in, 12_in}, {48_in, 24_in, 0_deg}}`) and a 90° bend (`{{36_in, 36_in, 90_deg}}`),
  each three times, the serial log saved. Read: SETTLED or not, the end's ` xt` and heading, the TSTR count. Then
  full speed.
- **C.2 — Look-ahead** 8 → 6 in if every run settles and tracks (the look of it: does it swing wide on the bend?);
  8 → 10 in if the robot wobbles side to side on a straight stretch.
- **C.3 — Bend grip** 60 → 90 → 120 in/s² while the robot holds the path through the 12 in bend without the back
  stepping out (HA-247).
- **C.4 — Odometry on a curve** (HA-249): tape a mark at the end pose; after the S at full speed, measure the robot
  against the mark and compare with the log's estimate.
- **C.5 — A hairpin** (`{{36_in, 18_in, 90_deg}, {0_in, 36_in, 180_deg}}`) and the backwards S from (0, 0) facing
  0° (`r.reverseCurveTo({{-24_in, -12_in}, {-48_in, -24_in, 0_deg}})` — the end heading is the NOSE, *re-check A2*).
- **C.5b — The run-up, OPTIONAL, ~10 min, only if C.1–C.5 left time** *(amended 2026-11-10 late; HA-246)*: tape an
  end pose; place the robot **2½ ft back along its line, 6 in to the side, its nose turned about 20° AWAY from the
  line** (the rule's hardest corner — where every host failure of the first draft's rule sat, *amendment re-check
  B1*); one-point `curveTo` to the pose, three times — the rule says it settles; then 1 ft back, 2 in to the side,
  square, three times — the rule says it misses (TIMEOUT at the full `.timeout`, ` xt` above 0.5 in). Read SETTLED
  or not and the end's ` xt`. A robot that misses from 2½ ft means the guide's rule is too short for it: the
  coordinator adds the measured distance to the robot file's comment and the team plans a longer run-up. Skipping it
  costs nothing but the check.
- **C.6 — The guards, healthy side only** (HA-248; *re-check A9*): every C run's log is read for a FALSE "curve lost"
  (there must be none). **No motor cables are pulled** — impractical on a five-motor side, and it exercises device
  loss and the fault policy, not the curve's guards; the dead-side half of HA-248 stays host-only (CV5).
- **C.7 — Record the two numbers** in the robot file's motion config with the date and "Station C" as provenance —
  **before the ~Nov 18 freeze** unless the team lead rules otherwise (D12).
- **The fallback** the team writes with every curve: `constexpr bool kUseCurves = false;` — OFF until the team lead
  turns it on from the Station C log (*re-check A5*: doing nothing must be the safe choice) — and
  `if (kUseCurves) r.curveTo({…}); else r.face(…).driveTo(…)…;` (the cookbook recipe). One flag per robot's skills
  file.
- **Robot one** (*re-check A5*; no build yet, G1): it runs robot two's two numbers as a recorded belief (HA-246), and
  its own `kUseCurves` turns on only after it passes C.1 itself (the S and the bend, three times each, ~20 min) once
  built. **Who and when, in one line:** Station M then Station C on robot two ~Nov 17–18; the team lead decides each
  `kUseCurves`; a student writes it; all by the freeze.

### 2.1 The runs — each its own go, each verified and committed on its own
| Run | What | Size (rough) | Starts | Merges before |
|---|---|---|---|---|
| **CURVE1-A** | §1.1–§1.7 + the documents of §3.1 | **~3–4 builder-days** (TANK1 Part A was sized 1¼ and took ~1 calendar day plus a review that found a loop-damping blocker, `tank_steer.hpp` :24-36: the follower ~1¼–1½, the path ~½–¾, guards + end ~½, verbs/steps/pins/version ~½, tests ~¾–1, docs ~½; *re-check: the refusals and the gated end add ~¼, inside the range*; *amended 2026-11-10 late: CV23 and the run-up guide text add ~⅛, inside the range — no new library behaviour, so no slip*) | on STEP1-A's merged tip, BESIDE STEP1-B, in the first free slot after AF1S-A1, TUNE1-B1 and STEP1-B launch (D12, *re-check A5*) | the ~Nov 18 freeze, for Station C — realistically merged ~Nov 16–17 |
| **CURVE1-B** | §1.9 | ~¾–1¼ | after the 21st, after STEP1-B | — |

### 2.2 Tests (ADVERSARIAL — each tries to break ONE rule) and the mutations each must turn red
Grade against the plant's TRUE pose (the rig's rule, `motion_test_rig.hpp` :5-8). Tank = `TankKinematics` 12 in; the
weak side = TANK1's `TankWorld` (`test/motion_tank_steer_test.cpp` :117-144) — re-use it, do not re-invent it.

| # | Test (what it tries to break) | Must hold | Mutation(s) that must turn it RED |
|---|---|---|---|
| **CV1** | the path through its own points (pure) | `at(0)` = the start pose and heading; every point passed within 1e-6 in; a point WITH a heading is passed at that heading (≤ 1e-6 rad); a free middle point's tangent ∥ (next − previous); `length()` within 0.1 % of a 4 000-sample polyline; *(re-check A2)* on every ACCEPTED path of the test set (the 17 drivable routes and 200 seeded random point lists that pass `checkCurve`), P′(u)·chord ≥ −1e-9 at 64 samples per segment — the path never doubles back | M1 tangent magnitude ×0 (a corner); M2 the free tangent from the previous point only |
| **CV2** | a straight curve is a `moveTo` | one point straight ahead, `{48, 0, 0°}`: SETTLED; the end within 0.5 in and 1.15°; lateral excursion ≤ 0.2 in | M3 the look-ahead's sign (y → −y): the robot curls away |
| **CV3** | a curve OFF ITS START HEADING | starts 30°, 60°, 90° off the first chord: SETTLED, ≤ 1.5 in off the path; 91° and 150°: refused at the first live tick — zero distance driven, TimedOut, ONE Warn containing "turn first: face()", `MotionTimeout` raised, elapsed ≤ 2 ticks after live; *(re-check A2)* the `Invalid` shapes (CV12) refused the same way, zero distance, the Warn naming the point | M4 `maxStartOff` check removed (150° drives a loop — the test measures the distance travelled); M5 `>` → `>=` at exactly 90.0° (the boundary pinned: 90° drives) |
| **CV4** | a HAIRPIN | the 18 in and 9 in hairpins (radii 10.9, 5.7 in): SETTLED, ≤ 1.5 in off the path, heading at the end ≤ 1.15°; the 6 in hairpin (radius 4.0 < look-ahead/2): exactly ONE `TightBend` Warn, and then either SETTLED or TimedOut — never a guard trip, never > 3 in off the path | M6 the bend speed limit dropped (curvature term → ∞ speed): off-path > 3 in on the 9 in hairpin |
| **CV5** | a DEAD MOTOR / side | right side to 0 V from 0.8 s into the S, the 90° bend and the field S: the curve exits TimedOut within 2.5 s of the death with "curve lost" (heading or progress), motors at 0 V on the exit tick, total distance travelled after the death ≤ 30 in; one motor of five lost (×0.8 from the start): SETTLED; *(re-check A9)* the right side dead from INSIDE the last 12 in (the S at 1.9 s, the 36 in bend at 2.2 s): exits TimedOut at the step's budget (no guard — they are off there, stated), total travel ≤ the path length + 6 in, motors 0 V on the exit tick | M7 heading guard removed AND M8 progress guard removed (each alone, then both: both removed ⇒ the robot drives for the whole budget — RED on the 2.5 s bound) |
| **CV6** | no FALSE ALARM, and the end is STEERED | *(re-check A1, A12)* the **17** drivable routes (P14's set less `off150`, which D3 refuses) × weak ×1.00/×0.91/×0.85 × τ 0/0.12/0.18 s: zero guard trips; **no "turn first" / "not steered" Warn in any curve's log**; hand-offs outside TANK1's 15° window counted and pinned (R-A: 2 of 153, none lag-free); settle counts pinned per cell from the library's own run, none below R-A's (16 16 14 / 16 16 16 / 13 17 16) minus 1 | M9 progress guard ON in the end zone (P13: fires in the slow approach); M10 `lostTime` 0.25 → 0; M10b the last resort ungated (4 in regardless: the "turn first" Warns come back, 25 of 153 on the host) |
| **CV7** | a DEADLINE MID-CURVE | `competition::runAuton` / `RunGuard` with the park at T = 1.0 s while a `curveTo` runs: the curve is cancelled on the cut (STEP1's P6: one tick AT the deadline, then the cut), motors 0 V + Brake on that tick, `RoutineResult` stopped at the curve step with `exit = Cancelled`, the result line names `CurveTo`; under STEP1-A's `onFail(carryOn)` the deadline STILL stops (STEP1's D3) | M11 `cancel()` skips the member `MoveToPose` when it is in the end phase (cut at T inside the last 12 in — a second test with the deadline there) |
| **CV8** | THE END HEADING | every settled route ends ≤ 0.5 in and ≤ 1.15° (the default settle) against the TRUE pose; with STEP1-A's `{.headingTolerance = 3_deg}` the end settles sooner and still ≤ 3°; a free last point ends along the path's own end direction (≤ 1.15°); *(re-check A1)* the 12 in bend under τ 0.18 s reaches case 3 (the end of the path outside the window): ONE curve Warn "curve end: … unsteered", the member built with `maxSteer = 0` (its `tankSteerStatus().eligible` false) | M12 the hand-off at 15° instead of 5° (P3/P4: ends off by 0.5–0.9 in, TIMEOUT under lag τ 0.12) |
| **CV9** | the closest point never JUMPS (pure) | on the crossing path and a 3 in-wide hairpin, a pose ON the later leg but 0.1 in nearer the earlier leg: `project` returns the later leg's `s` (forward-only window); a pose behind the current `s`: `s` never decreases | M13 the search over the whole path (P7: closed loop does NOT catch it — this pure test must) |
| **CV10** | reverse | `revS` and a reversed 90° bend: SETTLED, ≤ 1.5 in off the path; the robot's BACK leads the whole way (body vx ≤ 0 on every pursuit tick); a reverse curve whose first point is in FRONT of the robot is refused (D3, mirrored) | M14 the reverse flag ignored in `build` |
| **CV11** | wait-for-live | issued during the boot window: zero volts every wait tick, the path built at the FIRST live tick from the live pose (a pose seeded before live and moved before live → the path starts at the moved pose), a never-live estimate exits TimedOut | M15 the path built in the constructor |
| **CV12** | the INPUT edges | 0 points, 17 points, a NaN coordinate in point 3 (refused BEFORE any motion — the first two points never drive), *(re-check A2/A3)* consecutive points 0.99 in apart (refused by name) and 1.01 in apart (accepted); a HEADINGLESS first point 0.99 in from the live start (dropped, one Info line, the curve drives from the next point — no TightBend Warn) and 1.01 in (kept); a HEADED first point 0.99 in from the start (refused: "where the robot already is"); a single headingless point at the robot (refused: nothing left); `outback` (start → (24, 12) → (0, 0, 180°): refused — no NaN, no crash), `loopmid` ((24, 0, 180°) then (0, 24, 180°)), `revHead` ((36, 0, 180°) forward), the reverse example with `180_deg` (refused) and with `0_deg` (accepted, SETTLED); a 1-point curve, a 16-point curve (accepted) | M16 the finiteness check moved after the first leg; M16b the 90° chord rule applied to HEADED points only (`outbackNear`-style free points pass and the path doubles back — CV1's property turns RED); M16c the free-point neighbour check after the normalisation (NaN) |
| **CV13** | the deadline belongs to the CURVE | a curve with `{.timeout = 1.5_s}` that cannot finish: exits TimedOut at 1.5 s ± 1 tick, `MotionTimeout` names `CurveTo` (never `MoveToPose`), even when the end phase began | M17 the member `MoveToPose` armed with a timeout SHORTER than the remaining budget |
| **CV14** | the observer is TANK1's | the curve's observer step and `MoveToPose`'s, fed the same (measured, commanded, dt, ready) sequence of 200 ticks: identical estimates (bits) | M18 the observer's clamp removed in the curve only |
| **CV15** | a WEAK side and LAG | the S, the 36 in 90° bend and the field S at ×0.85 and at τ 0.12 s: SETTLED (R-A; *re-check A1*: the 18 in and 12 in bends do NOT settle at ×0.85 with no lag — they stay out of this test and in CV6's pinned counts) | M19 the observer off (P3: `ppnoobs` drops at ×0.85) |
| **CV16** | NOTHING ELSE CHANGES | every existing motion test green unedited; a 6-leg tank `moveTo`/`driveTo`/`reverseTo`/`turnTo` route and the same on the X-drive: records (blackbox-encoded), log lines and truth bit-identical to the run's MERGE BASE (*re-check A12*; TANK1's `requireIdentical` / `requireSameRecords` / `requireSameLog`) | M20 `MoveToPose` given the curve's look-ahead (any edit to its tick) |
| **CV17** | STEP1-A's tolerance reaches the end | `{.positionTolerance = 2_in}`: the curve ends when within 2 in and slower than 4 in/s (STEP1-A's D2 rule), measured | — (STEP1-A's own) |
| **CV18** | the records and the result line | one record per tick; `targetPose` the end pose on every record; the exit record's command zero; `tankSteerActive` only above the floor; the result line `CurveTo SETTLED … xt` | M21 the exit record emitted before the motors stop |
| **CV19** | the speed plan | on a 90° bend of radius 11.4 in at bendAccel 60: the commanded forward speed inside the bend ≤ √(60 × κ⁻¹) + 1 in/s; never above `maxLinearSpeed` (and the call's); monotone slowing over the last 12 in; *(re-check A6)* on a 72 in straight into a 2.5 in-radius bend: the robot is ALREADY at the bend's speed when it reaches the bend, having slowed at ≤ `decel` (+ one tick's rounding) — measured along the path — and the planned v·\|κ\| ≤ `maxAngularSpeed` at every sample (pure, on `speedLimitAt`) | M22 the ±2 in look-ahead in `speedLimitAt` dropped (speeds into the bend's start); M22b the backward pass removed (arrives at the tight bend too fast); M22c the ω cap removed (the 2.5 in bend's planned v·\|κ\| exceeds the limit) |
| **CV20** | other drivetrains (D9) | the S on the X-drive and the H-drive: SETTLED, body vy commanded 0 on every pursuit tick, SFB never lights | — |
| **CV21** | the size | *(re-check A4, A12)* `static_assert(sizeof(CurveFollow) <= 6144)` on the host (the stated count, §1.2); `std::is_trivially_copyable_v<CurvePath>` (a cheap, heap-free copy — it does NOT prove "no pointer"; that is read in §7) | — |
| **CV22** | the pins | F6: the four verbs' exact types, `CurvePoint`'s two constructors; F10: the four steps (L9's patterns, each seen failing the build under its own mutation the day it lands) | M23–M26: a parameter type changed, a `noexcept` added, the `initializer_list` overload removed, the return type changed |
| **CV23** | *(amended 2026-11-10 late)* THE RUN-UP RULE the guide states (a one-point curve removes a sideways miss) | *(amendment re-check B1, B4: re-sized to the two-number rule and the 0.18 s lag)* at the SHIPPED `CurveConfig`, a one-point `curveTo` to (0, 0, 0°) from (D, S) = (−30, ±2), (−30, ±6), (−36, ±12) in, nose 0°/±10°/±20°, weak ×1.00/×0.91 × τ 0/0.12/**0.18** s (60 runs a cell): every run SETTLED and within 0.5 in and 1.15° of the TRUE end pose (host probe: 60/60 each, `fix2/cv23b.txt`; the run-up margins against the measured worst need are only 2 / 1 / 2 in). The test also LOGS, not asserts, the smallest passing D per cell on a 1 in grid (host: 28, 29, 34 in). The too-close side, (−12, ±4) nose 0°/±20°, τ 0/0.12/0.18: never a guard trip, no "turn first" / "not steered" Warn, at most the one "curve end … unsteered" Warn, the exit at the step's `.timeout` ± 1 tick when it does not settle, the end ≤ 2 in off (host: 30 of 36 time out, worst 1.52 in). The test's header names guide 05's rule and says: **a red CV23 means RE-MEASURING the rule and updating guide 05 in the same commit — never tuning the follower to pass**; a later default change that moves the rule changes the guide in the same commit | M27 `lookAhead` 8 → 12 in (host: 47, 30 and 31 of 60 — RED); M28 the Hermite tangent length ½ × chord (host: 60, 58 and 46 of 60 — RED). *(The hand-off thresholds are not relied on here — 0.3 → 3 in costs one run of 60 at (−30, ±2); they are CV6/CV8's.)* |

**Mutations**: each in a FRESH build (TANK1 §2.5's lesson: `motion_config.hpp` reaches 20+ test files), the coordinator
re-runs two by hand (§7).

### 2.3 Definition of done — CURVE1-A (the MECH core's §2.1, adapted)
The suite green through `tools/suite_run.py`; every mutation above RED and restored; the ARM header gate
(`-Werror`, every header in one TU) green; **the ARM stack check**: `arm-none-eabi-g++ -fstack-usage` over a TU that
calls `chassis.curveTo` with 16 points — the frame of `Chassis::curveTo` + `CurveFollow::tick` recorded in the live log
(HA-250's first evidence; flag anything over 8 KB — *re-check A4*: the motion alone is ~5.35 KB on the host); `tools/src_build_gate.py check`; the API doc tool's five checks;
`briefing_status.py check`; the prose review logged; the live log's last entry says what was NOT done; no hardware
claim anywhere.

---

## 3. Documents — in the same change as the code (pace rule 3), the prose review logged (rule 4), ≤ ~5 public each (rule 7)

### 3.1 CURVE1-A (public)
1. `docs/guide/05-getting-there.md` — a section "Flowing through points: curves" (what it does; the two lines; how it
   starts, ends and stops; the tuning; what it never does; "proven on the host test plant only").
   *(amended 2026-11-10 late)* — and a sub-section **"A sideways miss: what a curve fixes, and from how far"**, in
   these words (the writer may smooth them; the numbers stay): "A tank cannot slide sideways. If the robot is a few
   inches to the side of where it must end, a `moveTo` will not fix it; a one-point `curveTo` to the end pose will —
   if it starts far enough back, because it needs room to arc onto the end's line before its last few inches, which
   are a `moveTo`. With the shipped settings (look-ahead 8 in), the robot pointing within 20° of the end heading:
   **a miss up to 6 in needs 2½ ft of run-up along the end's line; up to 1 ft, 3 ft.** This holds with motors that
   react slowly (up to 0.18 s of lag on the host test plant) and a side up to 9 % weaker; it leaves only an inch or
   two to spare, so add a few inches where there is room." [*amendment re-check B1*: the worst need was 27–29 in up
   to 6 in, 32–34 in up to 12 in, lag 0–0.18 s; the first draft's "2 ft for up to 2 in" failed at 0.18 s.] "A
   noticeably weaker side (12–15 %) needs about 2¾ ft and 3⅓ ft. `{.positionTolerance = 1_in}` (when 1 in is good
   enough) brings it to 1¾ ft and 2 ft. If the robot file's look-ahead was changed at Station C, the distances change
   with it: look-ahead 10 in needs about 3¼ ft and 3⅔ ft; look-ahead 6 in needs only about 1⅔ ft but does not settle
   at all on slow-reacting motors. Closer than the rule, the curve gets close but not within half an inch: it runs
   until its `.timeout` is used up, then stops — up to 3½ in off from 6 in or more back, and from closer about as far
   to the side as it started — and the result line's ` xt` shows by how much. **Closer than the rule, write today's
   turn-drive-turn: `face(x, y)`, `driveTo(x, y)`, `turnTo(heading)`** — every step said it arrived in every host
   test, in about 3 s (worst ~4½ s). One thing to know: on a robot with a weaker side, its last turn in place can
   shift the robot up to about 1 in (1½ in on a much weaker side) — about 1 run in 8 ended more than half an inch
   from the exact spot.
   The library never backs up and tries again on its own: it cannot see behind the robot. If YOU know the space
   behind is clear and the pose must be exact, write it: **`faceAway(x0, y0)` FIRST** — without it this fails about
   half the time on the host test plant, because the backward drive is not steered unless the robot already faces
   away — then `reverseTo(x0, y0)` to a spot 2½ ft back on the end's line, then the `curveTo`. It is slower (about
   4 s), and it arrived in all but 4 of 3,240 host tests with a side up to 9 % weaker (about 95 % with a 15 % weaker
   side). All of these are host test plant numbers, not robot numbers." *(amendment re-check B3, B4)*
2. `docs/cookbook/01-getting-there.md` — a recipe: the S, the backwards curve, and the `kUseCurves` fallback (`false`
   until Station C); compiled by `test/cookbook_examples_test.cpp`. Two rules in bold (*re-check A2, A3*): **a heading
   is the robot's NOSE, also backwards**; **do not list the robot's own spot as the first point**. *(amended
   2026-11-10 late)* A third, one line: **a curve into a pose that must be exact starts 2½ ft back along that pose's
   line for a miss up to 6 in, 3 ft up to 1 ft (guide 05's run-up rule); closer, `face` + `driveTo` + `turnTo`.**
   *(amendment re-check B1)*
3. `docs/guide/14-what-it-cannot-do-yet.md` :437-439 — "Stop-and-settle only" becomes: curves exist (CURVE1); a curve
   stops at its END; rolling through points is still not offered. *(amended 2026-11-10 late)* Plus one item: "A tank
   cannot remove a sideways miss in a short distance — not with `moveTo`, not with a curve started too close (guide
   05's run-up rule). The library does not back up and retry by itself."
4. `docs/guide/11-reading-the-diagnostics.md` — the `curve:` Info line (and the near-start drop), the two "curve lost"
   Warns, the refusals (`StartBehind`, every `Invalid` wording), the "curve end: … unsteered" Warn, the
   result line's `CurveTo` / `ReverseCurveTo` (and what `ov` means for a curve, §8 L7). *(amended 2026-11-10 late)*
   And: "a `CurveTo` that TIMED OUT with ` xt` above 0.5 in reached its end off the end's line — usually too little
   run-up for the sideways miss it started with (guide 05)".
5. `docs/changelog.md` — the entry (API 2.x, additive), with the probe numbers as "host test plant".
Plus the generated `docs/api/` pages (`curve_path.md`, `curve_follow.md`, `chassis.md`, `routine.md`,
`motion_config.md`; the doc tool regenerates them, not counted), `docs/guide/15-glossary.md` (look-ahead, bend grip,
pure pursuit — one line each), `docs/guide/10-the-api.md`'s verbs list.

### 3.2 Internal, CURVE1-A
`docs/roadmap.md` — F6 and F10 rows (one sentence each: the verbs/steps appended, API 2.x), the "Path & trajectory"
items; `docs/internal/chunks/C1-COMPLETED.md` — D12's dated note, D7 (a)'s text verbatim, under TANK1's (never a
rewrite); `docs/hardware-assumptions.md` — HA-246..250 (§4); `docs/internal/REMAINING-WORK.md` row 20d; the briefing
(`PROJECT-BRIEFING.md`) through `briefing_status.py`; `docs/internal/build-order.md` (the plan commit's, not the run's).

### 3.3 The plan commit (the coordinator's, on the team lead's go, BEFORE CURVE1-A)
This brief → `docs/internal/chunks/CURVE1-curve-follower.md` with an answers block; REMAINING-WORK row 20d (its "after
the route maker" → "before the 21st, after STEP1-A; CURVE1-B after the 21st"); the HA register's Held line gains
HA-246..250 (with the owed AF1, ALLY1+L4 and AF1-S blocks); `build-order.md` the 11-10 rows; the worksheet's Station C.

### 3.4 The prose review's greps (each run runs them and logs the hits it read)
`never aims\|never plans a turn\|aim at the point\|D12\|stop-and-settle\|settling at each\|no curve\|no arc\|"arc"\|named gap\|blending`
over `include/shulib docs/guide docs/cookbook docs/roadmap.md docs/vexbuilder-integration.md` — every hit read: true
as written, or amended to name the curve verbs as the one exception (D7). `simulator` in every touched file: empty.

---

## 4. Hardware Assumptions — HA-246..HA-250 ONLY (the register's Held line gains them at the plan commit)

| HA | Label | Belief | Status | Code site | Settled by |
|---|---|---|---|---|---|
| **HA-246** | CV-H1 | `CurveConfig`'s look-ahead (8 in), bend grip (60 in/s²), accelerations (200 / 120 in/s²), the hand-off (12 in, 5°, 0.3 in, last at ⅓ inside TANK1's window) and the end speed (×0.6) let robot two follow a curve and settle at its end within the default rule — **and robot one, on robot two's numbers, until its own C.1** (*re-check A5*: both robots run skills; "structurally similar", NOTES :963) — *(amended 2026-11-10 late)* **and the run-up rule guide 05 states (a one-point curve removes a sideways miss of up to 6 / 12 in from 2½ / 3 ft back, nose within 20°; on the host test plant it holds over motor lag 0–0.18 s — *amendment re-check B1*) holds on the robot within ~6 in** | **invented** (host test plant, R-A: 13–17 of the 17 drivable routes under every weak side and lag tried; the run-up rule: the amendment's probe 1, CV23; the plant has no slip and neither robot's lag is measured) | `motion_config.hpp` `CurveConfig`; each robot file's motion config | Station C.1–C.3, C.5 on robot two (the run-up: C.5b, optional); C.1 on robot one once built |
| **HA-247** | CV-H2 | robot two does not slide sideways in a bend taken at √(bendAccel ÷ curvature) — its tyres hold 60 in/s² (≈ 0.16 g) of sideways push | **reasoned** (VEX tyres commonly grip well above that; never measured here; the plant has no slip) | `CurveConfig::bendAccel` | Station C.3 |
| **HA-248** | CV-H3 | a healthy robot two never points more than 60° off its path for 0.25 s nor stalls under 1 in of progress for 0.75 s outside the end zone, and a dead side does both within ~2 s | **invented** (P9/P10/P14: 0 false alarms in 153 host runs; 47° worst at τ 0.18 in P9, 24.6° in R-A) | `CurveConfig::lostHeading/lostTime/stuckTime` | the HEALTHY half: every Station C run's log (C.6, no false "curve lost"); the dead-side half: host tests only (CV5) — no cable is pulled on the robot (*re-check A9*) |
| **HA-249** | CV-H4 | robot two's odometry (drive encoders + IMU heading, no tracking wheels) is as true on a curve as on a straight — the skid of eight wheels in a turn does not overcount distance enough to move the end by more than ~1 in on a 60 in curve | **reasoned** (a skid-steer drive scrubs in a turn; the IMU owns the heading; never measured) | `curve_follow.hpp` (reads the Localizer only) | Station C.4 |
| **HA-250** | CV-H5 | a `curveTo` fits the auton task: its motion (≤ 6 KB host by `static_assert`, ~5.35 KB counted — *re-check A4*; about the same on the 32-bit V5, whose doubles are also 8 B) on the default 32 KiB stack, and its per-tick work (one forward search over 12 in of arc, a few Hermite evaluations) well inside the 10 ms tick | **reasoned** (sizes P12; the ARM `-fstack-usage` frame in §2.3 is the first evidence; the loop monitor's overrun count at Station C) | `curve_follow.hpp`, `curve_path.hpp` | §2.3 + Station C's log |

---

## 5. OUT, and explicitly rejected
- **Rolling through points / motion chaining** (G3a's D16 (b); master plan :515) — a curve stops at its end. Two curves
  in a row stop between them; one curve through all the points does not.
- **The panel and its route maker** (G3a Part R) — no panel before the 21st (2026-11-10). D11 says what it will draw.
- **F8's `"arc"` in CURVE1-A** (D10 (b)) — CURVE1-B, after the 21st.
- **Partway triggers on a curve** (STEP1-B's `atPercent` on `curveTo`) — after STEP1-B lands, additive (a curve's
  `LegProgress` is "fraction of the path's length"); meanwhile a mechanism's START form before the curve and its
  `waitUntilDone` after it do the job.
- **A holonomic curve with an independent heading** (an X-drive facing one way while curving another) — the C1
  thesis's natural follower; not this season (both robots are tanks).
- **A timed trajectory / RAMSETE, S-curve profiles, time-optimal plans** (D1 (d)) — need R5's measured constants.
- **Extracting TANK1's observer into a shared helper** (§1.2) — a refactor for its own run.
- **Carrying the curve's yaw estimate into the end `moveTo`** (*re-check A1*) — probed and WORSE (136–137 of 153
  against 139–140); it would also need a new method on `MoveToPose`, which D7 keeps unchanged.
- **Guards inside the last 12 in, and jam detection on encoder odometry** (*re-check A9*) — the end zone is slow on
  purpose (P13's false alarms) and robot two cannot tell spinning wheels from motion; the step's time budget is the
  bound, stated in D6.
- **A new `ExitReason` or `RoutineStopCause` for "lost" / "refused"** — `TimedOut` + `MotionTimeout` + a Warn that names
  it; a new enumerator ripples through every `switch` on a frozen type.
- **A new `DebugRecord` field** (the look-ahead point, the progress) — the record schema and blackbox encoding are
  untouched; `curveStatus()` and the Info line carry them.
- **Automatic turn-to-face before a curve** (D3 (b)) — the author's `face()`.
- **Boomerang** (D1 (b)) — and its "carrot" for single poses; `moveTo` with TANK1 stays the single-pose verb.
- **A back-up-and-re-approach for a robot too close to arc in — CONSIDERED, REJECTED** *(amended 2026-11-10 late;
  the amendment's probe 2, `amend/p2_compare.txt`, `pc_compare.txt`, `paired.txt`, `truth.txt`)*. Prototyped two
  ways on the host test plant — decided at the start from the run-up rule (`raPre`: reverse straight along the
  robot's heading far enough, then the curve, once) and on failure (`raPost`: reverse 12 / 18 / 24 in after a curve
  that did not line up, then curve again, once, only with ≥ 2.5 s left) — against a plain `moveTo`, `face` +
  `driveTo` + `turnTo`, STEP1-A's `onFail` and wider tolerances, on 4,320 runs (D 6–36 in, S ±1–12 in, nose 0/±10/±20°,
  four weak sides × two lags) and 2,000 close ones (2–6 in out, 2–6 in to the side). *(amendment re-check B2: graded
  on the TRUE pose, `fix2/truth2.txt`.)* At a weaker side up to 9 %, `raPre` ended within 0.5 in and 1.15° of the
  true pose in **100 %** of runs (main and close) against `face` + `driveTo` + `turnTo`'s **88 %** (close 82 %) and the
  guide's own mix's **89 %** (a curve inside the rule, turn-drive-turn otherwise) — all three "arrive" by their steps'
  own verdicts; turn-drive-turn's misses are its last turn in place on a weak side. At a 15 % weaker side: **99 %
  against 45 %** (close 100 % against 13 %). **On success, then, the back-up IS better.** On time it is not: **3.02 s
  median / 4.00 s worst** against turn-drive-turn's **2.72 / 4.26 s** and the mix's **2.54 s** (close: 3.30 / 3.91
  against 3.15 / 4.06); paired on the same runs **+0.26 s slower (median) where it backed up, +1.44 s at worst**. It is
  0.6–0.8 s FASTER than the student-written back-up (`faceAway` + `reverseTo` + `curveTo`: 99.9 % / 3.84 s main,
  100 % / 3.91 s close, 95.4 % at 15 %), and reverses a little further (up to 32–36 in against 27–29 in). `raPost`
  arrived 72–86 % (close 10–82 %), median 4.0–4.4 s. **Rejected because** the adoption test was "clearly better on
  success AND time, risks bounded": it is better on success and not on time against what the guide recommends, and
  the risk is not bounded — the robot cannot see behind it, and up to 3 ft of blind reversing, on the library's own
  judgement, toward a wall (±70.2 in), a piece, the other robot or a start's keep-out is a hazard the library cannot
  judge, inside a 60 s run it can cost 1.45 s of. It would also have been a new behaviour on a frozen verb's end, CV
  tests, a Warn and a `CurveConfig` field before the freeze — the CURVE1 build must not slip for an option. (After
  the 21st, an EXPLICIT back-up step the author places could be briefed on its own; not proposed here.)

---

## 6. Coordination — shared files, who waits for whom, and what the consumers get

### 6.1 In flight while this was written (re-checked 2026-11-10 ~23:00; NOT edited — every worktree read only)
TUNE1-A, **STEP1-A** (`step1-a`: `chassis.hpp`, `routine.hpp`, `motion_config.hpp`, `version.hpp` (not yet bumped in the
worktree), the F6/F10 pins, `boot_source`-adjacent tests), CAL1-B (merging), the AF1-S brief, VIS1-A2. Queued: the RPV1
brief, TUNE1-B1, STEP1-B, L6-B, AF1-S's build (NOTES :961-962).
*(re-check A5, NOTES :963-964, 11-10 ~23:30 and after: the AF1-S brief is DONE; L6-A2 is COMMITTED (`16c01e0`); MECH-S-A
then CAL1-B merge next; the RPV1 brief is running and the **G1 brief** (robot one and its twin) follows it; the BUILD
queue is **AF1S-A1, TUNE1-B1, STEP1-B, then CURVE1-A** (after its go), RPV1, G1, AF1S-A2. BOTH robots run skills;
robot one has no build yet.)* *(amendment re-check B5, NOTES :965, 11-11 ~00:xx: **STEP1-A is DONE**; the merge queue
is MECH-S-A → CAL1-B → STEP1-A.)*

### 6.2 Shared files — who writes them, who waits, who re-reads
| File | Also edited by | Rule |
|---|---|---|
| `chassis/chassis.hpp` | STEP1-A (built, merging third — B5), STEP1-B (queued, ahead of CURVE1-A) | CURVE1-A starts on STEP1-A's MERGED tip and runs BESIDE STEP1-B (D12 (a), *re-check A5*); whichever merges second re-reads the other's block and takes the next `kApiMinor` |
| `chassis/routine.hpp` | STEP1-A, STEP1-B (its trigger handle lists the leg kinds: "drive legs" `driveTo`/`reverseTo`/`moveTo`) | CURVE1-A does NOT add curves to STEP1-B's leg kinds (§5); STEP1-B, if second, leaves `curveTo` out of its kinds and says so |
| `motion/motion_config.hpp` | STEP1-A (`loosenedSettle`) | append-only on both sides; CURVE1-A's `curve` member LAST |
| `version.hpp`, F6/F10 pins | STEP1-A/B, MECH-S/TUNE1 (version only, by their brief) | the number is assigned at the merge |
| `test/boot_source_test.cpp` | — (STEP1-A adds no `MotionConfig` member) | CURVE1-A updates the pins (§8 L5) |
| `docs/changelog.md`, guide 10/11/14/15, roadmap, HA register, generated pages | every run | merged serially by the coordinator |

### 6.3 TANK1's steering
Untouched (CV16). The curve REUSES `MoveToPose` (its end) and duplicates the observer's ten lines (CV14 pins them
equal). Any later change to `TankSteerConfig` reaches the curve's end and its observer — HA-179's blast radius grows by
the curve; say so in HA-179's row (one sentence, the plan commit).

### 6.4 STEP1 (A: tolerances and on-failure; B: partway triggers)
- A curve is a drive step under STEP1-A's `onFail`, `label`, `positionTolerance`, `headingTolerance` — no new code
  (CV17). The deadline always stops (CV7).
- STEP1-B's `LegProgress` "stays open to new fields (CURVE1 may add one)" — CURVE1-A adds NONE (§5); the curve's
  progress is `curveStatus().s / length`, ready for STEP1-B's successor.

### 6.5 RPV1 (the host route preview, briefed beside this one)
RPV1 runs `autons.cpp` through the real library on the host test plant: a `curveTo` is drawn as driven, with its
step time against 60 s. CURVE1-A gives it `motion/curve_path.hpp` (§1.1) for the PLANNED path, its bend speeds and
`checkCurve`'s words. If RPV1 lands first, it draws curves when CURVE1-A lands with no change of its own beyond
recognising the step name. RPV1's brief should NOT reserve HA-246..250.

### 6.6 AF1-S (the skills Toggle check-and-flip)
AF1-S's §6.6: "a curve that ends at a look spot must end SQUARE to the wall within ~3°". A curve's end is a `moveTo`
judged at 1.15° (D4 (a), CV8) — satisfied without a following `face()`; with a loosened `headingTolerance`, keep it
≤ 3° on look spots (the cookbook says so).

### 6.7 What the other chunks get from CURVE1
The team: two verbs and a fallback recipe. RPV1: the pure path. G3a Part R (after the 21st): the same pure path and
`"arc"` (CURVE1-B). STEP1-B's successor: a progress number.

---

## 7. Checks — the coordinator re-runs every one after each run (RESUMING's step 3: never the report's word)
Every binary under the memory rule (2026-10-15). `<scratch>` is the coordinator's scratch folder — never `/tmp`.
```sh
cmake --build build/test -j"$(nproc)"
python3 tools/suite_run.py green
ctest --test-dir build/test -R compile_refusal --no-tests=error --output-on-failure | tail -3
# the ARM header gate — every v2 header in ONE TU
find include/shulib -name '*.hpp' | sed 's|^include/||' | LC_ALL=C sort | awk '{print "#include \""$0"\""}' > <scratch>/all.cpp
echo "int main(){return 0;}" >> <scratch>/all.cpp
arm-none-eabi-g++ -std=gnu++20 -Wall -Wextra -Wconversion -Wsign-conversion -Wshadow -Werror \
  -Os -mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=softfp -c <scratch>/all.cpp -o /dev/null -Iinclude
# the ARM stack frame of a 16-point curveTo (§2.3; HA-250) — read the .su lines for curveTo / CurveFollow::tick
arm-none-eabi-g++ -std=gnu++20 -Os -mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=softfp -fstack-usage -c <scratch>/curve_tu.cpp -o <scratch>/curve_tu.o -Iinclude
# the CI guards
grep -rnE '#[[:space:]]*include[[:space:]]*[<"]pros/' include/shulib | grep -v '^include/shulib/hal/pros/'   # empty
grep -rnE --exclude-dir=sim '#[[:space:]]*include[[:space:]]*[<"]shulib/sim/' include/shulib              # empty
python3 tools/src_build_gate.py check
python3 tools/api_doc_tool.py self-test && python3 tools/api_doc_tool.py check-coverage \
  && python3 tools/api_doc_tool.py check-fresh && python3 tools/api_doc_tool.py check-examples \
  && python3 tools/api_doc_tool.py check-removability
python3 tools/doc_staleness_audit.py && python3 tools/briefing_status.py check
# ── the brief's own ──
git diff --stat -- include/shulib/motion/move_to_pose.hpp include/shulib/motion/tank_steer.hpp include/shulib/motion/command_pipeline.hpp   # empty (D7: nothing else changes)
git diff --stat -- src/ tools/ Makefile include/shulib/config/route.hpp include/shulib/runner/                                      # CURVE1-A: empty
grep -nE '#include.*(chassis|sequence|competition|runner)/' include/shulib/motion/curve_path.hpp      # empty: PURE
grep -nE 'std::vector|new |std::unique_ptr|std::function|malloc' include/shulib/motion/curve_{path,follow}.hpp | grep -vE '^[^:]+:[0-9]+:[[:space:]]*//'   # empty: no heap
grep -n 'MotionOptions' include/shulib/motion/curve_follow.hpp                                         # empty: the motion never sees options
grep -rn 'probe_curve\|CurveProbe\|brief-CURVE1' include/shulib test                                  # empty (no scratch code leaked)
grep -rn 'PROVISIONAL (A4' include/shulib | grep -v 'HA-[0-9]'                                       # empty
grep -rnoE 'HA-(24[6-9]|250)' include src test | sort -u                                              # every id has its register row and a code site
grep -rn -i 'simulator' docs/guide/05-getting-there.md docs/cookbook/01-getting-there.md include/shulib/motion/curve_*.hpp   # empty
```
Plus the spot checks — three mutations re-run by hand (seen RED, restored, the suite green again): **M13** (the
whole-path search — CV9), **M7+M8** (both guards out — CV5) and **M10b** (the ungated last resort — CV6, *re-check
A1*); and READ: `CurveFollow::tick`'s order — watchdog FIRST, then the end phase, then the guards, then the command
(CV13's rule in the source); the last-resort hand-off calls `tankLegEngages` (not a copied 15°); `CurvePath` and
`CurveFollow` hold no pointer member besides the deps (CV21 cannot prove it, *re-check A12*); `checkCurve`'s free-point
neighbour check runs before any division (*re-check A2*); the speed table is filled by a backward pass and capped by
`maxAngular ÷ |κ|` (*re-check A6*).

---

## 8. Landmines
- **L1 — The hand-off judged on the END's line.** P1 found it: on a hairpin the end's line runs through the start, so
  "8 in from the end along its line" is true at t = 0. Judge progress ALONG THE PATH (remaining arc), and the heading
  and the cross-track to the end's line together (D4 (a)).
- **L2 — A short-leg `moveTo` cannot remove a sideways offset.** TANK1's own measured limit (`tank_steer.hpp` :24-36):
  over the last few inches a 15° bend removes little. The hand-off must wait until the robot is ON the end's line
  (≤ 0.3 in) and nearly aligned (≤ 5°) — P3→P4 is the difference between 9/17 and 15–17/17 at τ 0.12 s.
- **L3 — The no-progress guard in the end zone.** The end is slow on purpose; P13: 10 false alarms. Off in the last
  `handoffDistance`, and the last-resort hand-off at ⅓ of it so the follower can never creep forever.
- **L4 — The stack.** The motion lives on the auton task's 32 KiB stack (`chassis.hpp` :295-299; `calibrate.hpp`
  :36-42). A sample buffer of ~600 points × 6 doubles (the prototype's) is ~29 KB — a stack overflow on the brain with
  no message. Analytic evaluation + `float` arc-length and speed tables; the points held ONCE (in the path); the
  member `MoveToPose` in an `std::optional`; `static_assert(sizeof(CurveFollow) <= 6144)` from the stated count
  (~5.35 KB host — *re-check A4*: the first draft's ≤ 4 KB could not hold); the ARM `-fstack-usage` frame logged
  (§2.3), flagged above 8 KB.
- **L5 — `boot_source_test.cpp`'s pins** (:244-245, :264, :268): `MotionConfig` 13 → 14 members and new sizes —
  extend `requireDefaultTuning()` with EVERY `CurveConfig` leaf; never just bump the number (the pin exists so a
  config a core sets cannot survive unread).
- **L6 — The closest point must only move forward**, over a bounded window. Searching the whole path makes a robot on
  a hairpin's return leg or at a crossing jump legs — and closed-loop tests did not catch it (P7). CV9 is a pure test
  for that reason.
- **L7 — The result line's `ov` for a curve** is along the START-TO-END chord (`MotionStatsSink::overshoot()`,
  `motion_scheduler.hpp` :470) — meaningless for a hairpin. Not fixed here (the scheduler is shared); guide 11 says "for a curve,
  read ` xt` and the end error; `ov` is along the chord".
- **L8 — The member `MoveToPose`'s watchdog.** Armed at the hand-off with the follower's FULL timeout, so it can never
  fire first (CV13); armed with "the remaining budget" it would race the follower's own and name the wrong motion.
- **L9 — The curve's start is the LIVE pose at the first live tick** — not at the call, not at construction (CV11).
  `Routine::driveTo`'s bearing has the same rule (:304-320).
- **L10 — `reverseCurveTo`'s geometry flips TWICE** (the start tangent is the robot's back; the robot-frame look-ahead
  uses heading + π; the commanded vx is negative; ω = v·κ keeps its sign). A sign error drives the arc mirrored —
  CV10's "body vx ≤ 0 on every tick" and its off-path bound catch both.
- **L11 — Desaturation preserves the arc** (uniform scale, `command_pipeline.hpp` steps 5–6), the ω clamp does NOT:
  at `maxAngularSpeed` (6 rad/s) a tight bend at speed is under-turned. The bend speed limit keeps v·κ under the clamp
  on every default (check: 60 in/s² ÷ 4 in radius → ω ≈ 3.9 rad/s); the speed plan also caps v ≤ maxAngular ÷ |κ|
  — now IN §1.1's formula (*re-check A6*: the first draft named the cap here but left it out of the formula, and left
  out the backward pass that brakes INTO a bend).
- **L12 — Don't extend the probe into the library.** `probe_curve.cpp` is a measuring tool (vectors, `new`, dense
  samples, a string law switch). Write the library from §1, and quote the probe only as evidence.
- **L13 — "Never aims at the point" sentences.** Several headers and guide pages say it in absolute words
  (`tank_steer.hpp` :11-14, `move_to_pose.hpp` :21-23, `chassis.hpp` :114-118). They stay TRUE for their verbs; the
  curve verbs are the stated exception (D7) — add the exception where a reader would otherwise be misled (§3.4), do
  not weaken the sentences.
- **L14 — The end hand-off outside TANK1's window** (*re-check A1*). A last resort that fires on distance alone hands
  the member a leg more than 15° off: unsteered, and its Warn tells the author to `face()` first — for a curve they
  drew correctly (25 of 153 host runs). Gate it with `tankLegEngages`; at the very end of the path, build the member
  with `maxSteer = 0` and speak in the curve's words. Do NOT "fix" it by seeding the member's observer (probed: worse).
- **L15 — The reverse heading** (*re-check A2*). The heading of every point is the robot's NOSE, on both verbs; the
  travel direction of a reverse curve is heading + π. Writing the TRAVEL direction (the first draft's own example)
  makes the end heading point back along the path — now refused by the 90° chord rule, which is why that rule covers
  EVERY point, headed or free.

---

## 9. Summary
- **What:** `curveTo` / `reverseCurveTo` — pure pursuit on a smoothed path through the author's points, slower in
  tight bends, ending with an ordinary `moveTo`, refusing a curve that starts behind the robot, and stopping a curve it
  has lost. On the host test plant: 47–67 % of the time of today's stop-and-turn routes on the shipped settings
  (P11, P15); they settle 13–17 of the 17 drivable routes under every weak side and motor lag tried, the end steered
  by TANK1 in all but 2 of 153 runs (R-A); a dead side stops in ≤ 2.2 s instead of driving circles (P10) — except in
  the last 12 in, where the step's time budget stops it. Broken paths (doubling back, points on top of each other, a
  first point at the robot) are refused before the robot moves. *(re-check A1–A9.)*
- **A sideways miss** *(amended 2026-11-10 late, the team lead's "what can we do to fix?")*: a one-point curve
  removes a sideways miss up to 6 / 12 in from 2½ / 3 ft back (look-ahead 8 in, nose within 20°, motor lag 0–0.18 s;
  host test plant, CV23 pins it — *amendment re-check B1*), 0.56 s faster per pose than `face` + `driveTo` +
  `turnTo`; closer, that turn-drive-turn is the fix (every step's verdict arrived, ~3 s; ~1 in 8 ends 0.5–1 in off
  the exact spot on a weak side). A library back-up-and-re-approach was prototyped and REJECTED (§5): it lands
  exactly more often (100 % against 88 %) but is slower than turn-drive-turn and reverses up to 3 ft blind on the
  library's judgement, before the freeze (*amendment re-check B2*). Found on the way, for the coordinator: `turnTo` in place on a weak
  side shifts the robot up to ~1 in (12 % of host runs end > 0.5 in off the point after face-drive-turn).
- **What it changes:** four facade verbs and four Routine steps (additive, pinned), one config block, two new
  headers; nothing about any other verb (pinned). C1's D12 gains a dated note: the curve verbs, and only they, steer
  toward a point ON their own path.
- **Runs:** CURVE1-A ~3–4 builder-days, before the 21st; CURVE1-B (F8's `"arc"`) ~¾–1¼, after the 21st. HA-246..250.
  *(amended 2026-11-10 late: the run-up rule adds ~⅛ day — CV23 and guide text — inside the range; no new library
  behaviour, no new HA, the dates below unchanged.)*
- **The date, honestly** (*re-check A5*). CURVE1-A needs STEP1-A MERGED (built; third in the merge queue, after MECH-S-A and CAL1-B — *amendment re-check B5*) and a slot; the build
  queue puts AF1S-A1, TUNE1-B1 and STEP1-B ahead of it (NOTES :963). It runs BESIDE STEP1-B, not after it (waiting
  for STEP1-B's tip lands it ~Nov 19–21, past the freeze). Its build and review take ~2½–3½ calendar days when nothing
  stops (TANK1 Part A: go 09-27 → merged 09-28, `TANK1-PROGRESS.md` :17, :574) — but this month's runs have been cut
  by the 5-hour limit most days (NOTES :955-962). **Start ~Nov 12–13; best case merged ~Nov 16; likely ~Nov 17; the
  freeze is ~Nov 18.** On the robot: Station M must pass first on robot two (the motion stack has never driven a
  robot; robot two is built "by the 14th"), then ONE Station C session — **in practice a window of ~Nov 17–18**.
  Robot one (no build yet) borrows robot two's numbers and runs curves only after its own short check. **So: the code
  by the 21st is reachable; a curve the team can TRUST in the skills run needs Station M and Station C both done on
  the robot by ~Nov 18 — tight, and the first thing to slip.** The team writes the skills route with today's verbs
  first, `kUseCurves = false`, and the team lead turns legs to curves only where Station C passes — then a late or
  failed CURVE1 costs time, never the run.
- **Needs from the team lead:** answers to D1–D12 (D12 now includes one question: may the two tuning numbers and the
  `kUseCurves` lines change after the ~Nov 18 freeze?); the go for CURVE1-A on STEP1-A's tip, beside STEP1-B; a slot.
  *(amended 2026-11-10 late: the sideways-miss answer, decision item 13, asks him nothing new — it reports.)*

---

### The decisions, as I would put them to the team lead *(re-check: rewritten after the checker's findings)*

1. **How it steers (D1):** pure pursuit — the robot keeps chasing a point 8 inches ahead of it on your path. It's what
   LemLib uses for paths and it held up best in our tests, including with a weak side. *Recommended.* (Boomerang can't
   follow middle points; the other two broke down with a weak side or slow motors.)
2. **How you write one (D2):** `r.curveTo({{24_in, 12_in}, {48_in, 24_in, 0_deg}});` — points, each with or without a
   heading; the curve starts where the robot is. Up to 16 points. A heading is always where the robot's nose points,
   even driving backwards, and you don't list the robot's own spot. *Recommended.*
3. **How it starts, and what it refuses (D3):** along the robot's nose. Before moving it refuses, and names the point,
   if the first point is behind the robot or the path doubles back on itself (a heading pointing back the way the path
   came, or two points on top of each other). *Recommended.*
4. **How it ends (D4):** the last few inches are a normal `moveTo`, steered and judged like any drive — it now waits
   until the robot is lined up within 15° before handing over, so it never blames you with "turn first" for a curve
   you drew right. *Recommended.* In tests 140 of 153 runs stopped within half an inch and under 1°; the misses are a
   very tight hairpin, two tight bends with a 15 % weaker side, and slow-motor cases.
5. **Speed in bends (D5):** one "grip" number sets how fast it takes bends; it slows down BEFORE each bend. The
   library ships gentle settings; the team tunes two numbers on robot two in one session after the 40-inch test.
   *Recommended.*
6. **If it loses the path (D6):** it stops and fails the step (pointing 60° off the path, or no progress for ¾ s).
   In our tests a dead side otherwise drove circles until time ran out. Two gaps, stated: in the last 12 inches, and
   for a robot pushing a wall with spinning wheels, only the step's time limit stops it — so give each curve a
   realistic `.timeout`. *Recommended.*
7. **"Never aim at the point" (D7):** that rule existed to keep LemLib-style path chasing out of the library. Your
   ruling lets it in — only inside the two curve steps, and only toward the path you drew; every other step stays
   exactly as it is. *Recommended.*
8. **Names (D8):** `curveTo` and `reverseCurveTo`, like `driveTo` / `reverseTo`. *Recommended.*
9. **Other drivetrains (D9):** works on any drive, driving like a tank. *Recommended.*
10. **Routes as data, `"arc"` (D10):** after the 21st — nothing uses it before then. *Recommended.*
11. **Drawing (D11):** the route preview draws curves now from the real library. The panel, later, uses a TypeScript
    copy of the same maths, checked against the C++ one; it won't show speeds (your no-simulation rule for the panel).
    *Recommended.*
12. **When (D12):** start right after STEP1-A merges, running alongside STEP1-B instead of waiting for it (waiting
    would miss the freeze). Honest dates: the code probably lands Nov 16–17; trusting it in the skills run needs the
    40-inch test AND one curve session on robot two by about Nov 18 — a one-to-two-day window. The skills routes start
    with today's straight legs (`kUseCurves = false`) and you switch a leg to a curve only where the robot session
    passed; robot one borrows robot two's numbers after a short check of its own. **One question for you:** may the
    two tuning numbers and the curve on/off lines change on the robot after the Nov 18 freeze? If not, any curve not
    proven by the 18th does not run on the 21st. *Recommended.*
13. **Your question — "what can we do to fix?" a sideways miss** *(amended 2026-11-10 late; you said "add both only
    if it really does make it better", so we tested both)*.
    - **The run-up — ADDED (as a rule and a test, no new robot behaviour).** A curve CAN fix a sideways miss, if it
      starts far enough back to arc onto the line: **2½ ft back fixes up to 6 inches off, 3 ft up to a foot** (robot
      pointing within 20° of the end, a side up to 9 % weaker, even with slow-reacting motors; tested on the host
      test plant over 145,800 starts). *(A checker caught that my first, finer table — "2 ft for up to 2 inches"
      — failed with slow-reacting motors, which we haven't measured on either robot yet; the two-number rule holds
      there too, with an inch or two to spare, so add a few inches where there's room.)* Inside that, it got there
      every time and was about half a second faster per spot than turn-drive-turn. Closer than that, it misses —
      from 6 inches or more back by up to 3½ inches; from closer, by about as much as it started to the side — and it
      always uses its whole time limit. So the team's rule is: **plan 2½–3 ft of straight-ish run-up before a spot
      that must be exact; if the robot is closer, use turn-drive-turn (`face`, `driveTo`, `turnTo`) — every step
      said it arrived in every test, in about 3 seconds.** The guide and the cookbook say exactly this, a test keeps
      the numbers honest (if it ever fails, the rule gets re-measured — the follower isn't tuned to pass it), and
      one optional 10-minute check at the robot session tries the hardest case on robot two.
    - **Back up and try again — NOT added, and here is the honest trade.** We built it and raced it on the same
      6,320 starts. Measured on where the robot REALLY ends up, it did better: it landed within half an inch every
      time, against 88 % for turn-drive-turn (and 99 % against 45 % on a robot with a much weaker side). But it was
      slower than turn-drive-turn — about a quarter-second on average where it backed up, up to 1½ seconds at worst
      — and it backed up as much as 3 ft blind, on its own decision: the robot can't see behind it, and on a field
      with walls, pieces and the other robot, the library can't know if that space is clear. Building it now would
      also add new behaviour days before the Nov 18 freeze. Your test was "better on getting there AND on time";
      it passes the first and not the second, so it's out. If a student KNOWS the space behind is clear on a
      particular spot, they can write the back-up themselves — **`faceAway` first** (without it, it fails about half
      the time), then `reverseTo`, then the `curveTo`; about ¾ s slower than the library version would have been,
      but visible in the route and their call. If you want a library back-up later, it could come back after the
      21st as a step the author places on purpose.
    - **One thing we noticed:** turn-drive-turn's last turn in place can shift a robot with a weaker side by up to
      about an inch, even though every step says it arrived. Fine for looking at a Toggle; worth knowing for a spot
      that must be exact to half an inch. Not part of curves — the coordinator will raise it separately.
    - **Nothing else changes:** every recommendation above stands, and the build is the same size and dates.
