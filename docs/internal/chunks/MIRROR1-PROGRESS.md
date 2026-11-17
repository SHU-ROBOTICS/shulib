# MIRROR1 — progress log (live, appended as the work goes)

<!-- release: between parts — MIRROR1-A landed 2026-11-15 (API 2.9); MIRROR1-B (the team lead's D6: refuse an exact-180° tank leg in the pair check) is queued (declared by the coordinator 2026-11-15) -->

Brief: [`MIRROR1-mirrored-routine.md`](MIRROR1-mirrored-routine.md) (the coordinator's copy of record; its answers block
and "Re-check — amendments" R1–R13 override its body).

## MIRROR1-A

- **Executor:** an `opus` builder subagent, launched by the coordinator's workflow, in the git worktree
  `.claude/worktrees/mirror1-a` (branch `mirror1-a`, from `5f6633b` = main with AF1S-A1). Do not commit.
- **Go:** the team lead, 2026-11-13 (the brief's answers block): "do what is best they should play onto one side when
  they need to when mirrored, do what you need to do" = GO on D1–D11 as recommended after the re-check; D11 each robot
  plays its own side; D2 the coordinator's call (below).
- **Base facts read at the start (2026-11-13 ~10:51):** main is API 2.8; AF1S-A1 (the checked Toggle step,
  `competition/toggle_visit.hpp`) IS on the base, so the Toggle swap is in this run (no MIRROR1-B). The API minor is the
  coordinator's at the merge — this run writes the changelog entry at the top of `## API 2.8` and the version-history
  paragraph here, and does NOT edit `version.hpp`'s number.

### Log

*(Correction ~11:55: the first print of three headings below named clock times not yet reached (12:00–13:10) —
guessed, not read; re-dated from the scratch files' modification times.)*

#### 2026-11-13 ~10:59 — T10 measured at the base, BEFORE any library edit (the CV16 method)
- Wrote `test/mirror_unchanged_runs.hpp` (self-contained, no MIRROR1 header): every auton of today's robot two
  registry (`registerAutons`: the 40 in auton, the acceptance run) from the origin, plus a FIELD-start SKILLS auton of
  its own ("every verb": driveTo / face / driveTo with a trigger of each kind, turnTo, moveTo, faceAway, reverseTo,
  curveTo, reverseCurveTo, followTrajectory, strafeTo, a waitFor reading `pose()`, a `then()` driving `drive()` in
  both frames and calling `setPose`, face, hold, pause, driveTo — `onFail(carryOn)` from the curve on so a leg the
  weak side cannot finish skips no verb after it — and a park end action), each through `competition::runAuton`
  on the program's chain (Chassis -> RunGuard -> MechanismService over robot two's real list -> a hashing world),
  on a 9 % weak-side tank and on an X-drive; records (blackbox bytes), every log line, the true pose after every
  pace and the result folded into FNV-1a.
- Diagnosed first (`scratch/mirror1-a/t10/probe_t10_diag.cpp`): without carryOn the tank's reverseCurveTo timed out
  and skipped 8 verbs — so carryOn added; after it, the tank runs 17 steps (15 completed: the reverse curve and the
  strafe time out on a tank, by design) and the X-drive 17 of 17.
- Measured with the header compiled against the base (`git rev-parse HEAD` = 5f6633b, `git status`: only the brief,
  this log and the new header untracked; `scratch/mirror1-a/t10/base_state.txt` holds the header's sha256
  3a720a08…): **tank 0.91 → 0x560480cf92adc886** (twice, identical), **X-drive → 0xe015d28a01816830** — at the
  suite's flags (no -O, `out_base.txt`) and again at -O1 (`out_base_O1.txt`), identical. (`out_O0.txt`/`out_O1.txt`
  are the FIRST draft's, before carryOn — superseded.)

#### ~11:00–11:15 — part (1) of the build order: the reflection, the Chassis's mirrored run, the runner, the registry
- NEW `include/shulib/math/reflection.hpp` (`math::Reflection`, four lines at multiples of 45°, `+ 0.0` on every
  output, the identity never applied). Includes only math/ and units/ — `SHULIB_PRECONDITION` reaches it through
  `math/angle.hpp` (which carries `core/check.hpp`), so the brief's §7 purity grep stays empty.
- `chassis.hpp`: `reflection()`, `chassis::MirroredRun` (RAII; refuses the identity, nesting, a live motion), every
  field verb reflected at the door with a branch (identity = the old path), `pose()` reflected back, a 16-slot
  curve copy built from `CurvePoint`'s own constructor (an `index_sequence` of placeholders — R6), the `[MIR]` Warn on
  a mirrored turnTo / moveTo / trajectory leg within 1° of a half turn, `detail::clearLeakedMirror` (a friend) for
  the driver loop. Banner section "The MIRRORED RUN". F6 pins added (`f6_signature_pin_test.cpp`).
- NEW `competition/field_mirror.hpp` (`FieldMirror`, `mirroredSide`, `reflectedWall`, `seasonImageIndex`,
  `mirroredWall`, `mirrored()` — toggles; a different reflection is a precondition —, `isMirrored(ctx)`,
  `runReflection`, `seatedStart`) and NEW `competition/override_mirror.hpp` (`kSkillsMirror`, `toggleImage` found
  geometrically from `kTogglePlaces`, `mirroredToggle`, static_asserts: the walls and the Toggles close under the
  mirror, every Toggle keeps its Quadrant colour). The quick header TU compiled with the static_asserts PASSING the
  first time.
- `auton_registry.hpp`: `AutonEntry::mirror` (appended, a `const FieldMirror*`, forward-declared — the registry
  does not include the season), `kMaxMirroredAutonNameChars` = 25, the name refusal in `checkAutonEntry`.
- `match_runner.hpp`: a precondition on a leaked mirror; the reflected re-seat; `FIELD MIRRORED` in the re-seat
  line AND (a delta, below) the arming line; the second COMP line; one `std::optional<chassis::MirroredRun>` opened
  after the re-seat, reset after `guard.run` (and by the unwind); `AutonRunResult::mirrored`.
- Wall reset: `appendWallResetLine` (localization/wall_reset.hpp) and `runWallReset` gain a defaulted `sideNote`;
  the explicit form refuses inside a mirrored run; the everyday form swaps wall and side after rule 1.
- Toggle: `ToggleVisit` holds the request BY VALUE with its real Toggle (`realRequest`).
- Menu label `" MIR"`; both result screens' `START MIR`; the driver loop clears a leaked mirror with one `[MIR]` Warn.
- Build: the doc gate first refused 5 undocumented entities (the deleted `MirroredRun` members, the forward
  declaration) — documented; then the stale API pages — regenerated (`api_doc_tool.py generate`; mkdocs.yml gains
  the three new pages' nav lines, generated). The WHOLE existing suite then ran green through
  `tools/suite_run.py green` with the library changes and no new test: 3627 cases.
- T1 (`reflection_test.cpp`, 6 cases) green after fixing my own test's wrong -0.0 check (it asserted
  "non-negative" where it meant "never -0.0"). The shared vectors file `test/goldens/mirror/skills_mirror_vectors.txt`:
  17 FIELD + 5 RELATIVE vectors, worked by hand.
- T2/T3/T4-scope (`chassis_mirror_test.cpp`): tank 2083 paces, max |reflect(one) − two| **1.09e-12 in, 5.7e-14 rad**;
  X-drive 1529 paces, **6.2e-13 in, 8.7e-14 rad**; triggers on the same paces. T3: the exact-180 turn as P1 C (a
  Routine turnTo + driveTo): path **0.155 in**, end **0.091 in** — P1 C's numbers reproduced; the X-drive exact-180
  moveTo MEASURED at **0.307 in** of path and **3.087 rad** of heading apart mid-leg, pinned (0.30–0.315). My first
  draft of the seam used a raw `moveTo` with the current heading (end 0.131 in): replaced by the probe's own route.

#### ~11:15–11:45 — parts (2)–(4): the swaps, the screens, the tests, the pages
- **T10 re-measured (~11:20) on a CLEAN base tree.** To make MM12 (the identity "applied") observable, the
  every-verb route now writes a `-0.0` heading (`turnTo(Angle::radians(-0.0))`) and a `-0.0` position (a `moveTo`) on
  purpose — an identity that re-wraps or adds `+ 0.0` turns them into `+0.0` and the records' bytes show it. Measured
  by compiling the probe against `git archive 5f6633b` unpacked under `scratch/mirror1-a/base-tree/` with the new
  header copied in (header sha256 44cf39c1…): **tank 0.91 → 0x0fff1e8756c26e85, X-drive → 0x61e0c4294783616a**; the
  same probe against the MIRROR1 tree gives the same two numbers. (The ~10:59 numbers above are the first draft's,
  superseded.) After the mutation runs (~11:50) the header's file comment was reworded ("Everything here is the host test
  plant" — the brief's §7 word grep over `test/*mirror*` must be empty); `diff` against the measured copy shows that
  one comment line only.
- Tests, every one written to break ONE rule (the file comments name the bug each catches):
  `test/reflection_test.cpp` (T1, 6 cases), `test/chassis_mirror_test.cpp` (T2 tank + X-drive, the triggers firing,
  T3 ×3, T4 scope, T4 curve copy, T4 driver loop — 9 cases), `test/mirrored_auton_test.cpp` (T10, T5 ×7, T4 runner ×2,
  T8 ×3 — 15 cases), `test/mirrored_wall_reset_test.cpp` (T6, 5 cases), `test/mirrored_pair_check.hpp` +
  `test/mirrored_pair_test.cpp` (T11, 7 cases), T7 in `test/toggle_visit_rules_test.cpp`, T8 in
  `test/auton_tester_panel_test.cpp`, `test/competition_screen_test.cpp` and `test/wall_reset_line_test.cpp`, T9 as
  `autons (1d)` in `test/autons_test.cpp`, `cookbook-09a` in `test/cookbook_examples_test.cpp`; the F6 pins; the
  vectors file.
- Measured on the way (each pinned with its number): T11 (i) the brief's P1 route FAILS at **10.02 s** (5.76 in apart),
  both robots in **motion 8 (CurveTo)**, robot two at robot one's reflection to 1e-6, and the squares later TOUCH
  (least gap 0) — the brief's "10.1 s" is P5's touch moment on the bare probe; (ii) route G **8.43 in**, = 2 × 4.21 in
  to 1e-6; (iii) with the 9 % weak side **8.71 in**; (iv) robot one alone to the centre **47.97 in**, the branch timed
  wrong FAILS at 6.40 s; (vi) 5.9 FAILS, 6.1 PASSES, exactly 6.0 (hand-built tracks) PASSES and the next double below
  FAILS. The worst mirrored COMP lines **142 and 128 characters** (the brief estimated 147 and 130); the worst `[MIR]`
  line **184**; the worst mirrored `[WALL]` line's note closes at **byte 178** of 200. Cookbook 09's pair: 20.47 in.
- Fixed in my own tests along the way (none a library defect): the T4 follow-up first aimed at a target a tank
  cannot reach (moveTo (24, 6, 30°)) — replaced by (30, 4, 8°); the park first aimed at a lateral offset a tank cannot
  settle on — replaced by a straight park; T6 first handed robot two's entry the ALREADY-reflected start (the runner
  reflected it again) — the mirrored entry carries robot one's start, as a team writes it; the cookbook's wall seat
  first ran under the wall rig's 30 s MATCH period (refused) — it now runs `runAuton` with the 60 s skills period.
- Pages (pace rule 3, in this change): guide 02 "The field's mirror: two robots, one route" (the manual's reason, the
  table, the worked start, a drawing, what the mirror cannot carry, what lies ON the line and the 6 in band); guide
  17 "Two robots, one route: a mirrored auton" (the one line, the table, the screens and lines, below the mirror, the
  refusals, "Both robots at once" with the pair check's failure sentence, what it cannot carry); guide 11 (the
  seventh pinned `WALL` line, the `COMP` mirrored lines, both `[MIR]` lines, the `TGL` real-Toggle note); guide 14
  ("A mirrored auton: what the mirror cannot carry", three bullets); guide 10 (`reflection()`, the explicit wall
  reset's refusal, the everyday call's swap); guide 16 (the 25-character mirrored name); guide 20 (the image Toggle;
  the camera's real-field inputs); the glossary ("mirror (a mirrored auton)"); guide README (chapters 2 and 17's
  rows); cookbook 09 (NEW) + the cookbook README's index + mkdocs.yml's cookbook nav line (mine); the changelog (top
  of `## API 2.8`); roadmap F6 row (one sentence + the pin count 67 → 69); HA register (HA-258..HA-260 rows, the
  checklist section, the additions paragraph, the status count 223 → 226); REMAINING-WORK row 18e-M; build-order's
  MIRROR1 status block; API pages regenerated (`api_doc_tool.py generate`; mkdocs.yml's generated nav gains
  Reflection, Field mirror, Override mirror); PROJECT-BRIEFING regenerated (the tool).

#### ~11:45–11:50 — mutations (30 run, every one RED; one first cut re-cut)
Driver `scratch/mirror1-a/mut/mutate.py` (spec `spec.json`, outputs `run_all.log`, `run_named.log`): per mutation the
file backed up under `scratch/mirror1-a/mut/backup/`, ONE exact replacement (the old text must occur exactly once), a
FRESH small binary (the object directory emptied, every TU recompiled: `test_main.cpp`, `src/team/autons.cpp` and the
named tests' files), the named tests run under `ulimit -v 4 GiB` + `timeout 600`, the file restored in `finally` AND on
SIGTERM/SIGHUP/SIGINT with its mtime bumped. After the runs every spec's old text is present once and no mutant text
remains (checked by script), and `git diff -- Makefile src/` carries no `(self-test plant`.

| # | Mutation | Red (the test cases that failed) |
|---|---|---|
| MM1 | the field heading as -h (AntiDiagonal's 2a = 0) | T1 vectors, T1 sweep, T2 tank, T2 X-drive |
| MM2 | `pose()` not reflected back | T2 tank, T2 X-drive |
| MM3 | `~MirroredRun` emptied | T4 MirroredRun scope; T4 the runner's every way out |
| MM4 | `drive(Body)` keeps the turn rate | T2 tank, T2 X-drive |
| MM5 | the wall mapped, the side not | T6 seats the right wall LEFT; T6 own line |
| MM6 | the Toggle images crossed right <-> left (in `mirroredToggle`, the function the step calls — `toggleImage`'s static_asserts would stop the build first, so the mutation is placed where only T7 can see it) | T7 |
| MM7 | curve headings not reflected | T2 tank, T2 X-drive |
| MM8 | the moveTo observer overload not reflected | T2 tank, T2 X-drive |
| MM9 | `mirrored()` does not toggle back | T5 twice = e; T5 different mirror |
| MM10 | the runner seats the unreflected start | T5 FIELD seat; T5 park; T5 route entry |
| MM11 | RELATIVE uses the diagonal | T5 RELATIVE |
| MM12 | the identity re-wraps a turnTo heading (`+ 0.0`) | T10 |
| MM12b | the identity normalises a moveTo position (`+ 0.0`) | T10 |
| MM13 | only the first followTrajectory leg reflected | T2 tank, T2 X-drive |
| MM14 | the end action outside the scope | T5 park |
| MM15 | the pair check compares robot two with the REFLECTION of robot one | T11 (i), (ii), (iii), (iv), (vi), the registry walk |
| MM16 | `isMirrored` always false | T11 (i)–(iv), cookbook-09a |
| MM17 | the `+ 0.0` normalisation dropped (AntiDiagonal) | T1 ×3, the tester panel's START MIR, the competition screen's START MIR |
| MX1 | the driver loop does not clear a leaked mirror | T4 driver loop |
| MX2 | runAuton's leaked-mirror precondition removed | T4 runAuton refuses |
| MX3 | the explicit wall reset's precondition removed | T6 explicit |
| MX4 | the menu label loses MIR | T8 menu; cookbook-09a |
| MX5 | the tester's START line loses MIR | panel T8 |
| MX6 | the mirrored name budget = the plain one (29) | T5 26-character name |
| MX7 | a second, different mirror silently toggles | T5 different mirror |
| MX8 | the half-turn Warn's window 2° | T3 ×3 |
| MX9 | the Toggle step keeps robot one's Toggle — FIRST CUT BROKE ITS OWN BUILD (`-Werror=unused-parameter`): not counted, re-cut as MX9b | — |
| MX9b | the same, re-cut (the parameter still read) | T7 |
| MX10 | the WALL line drops the mirrored note | T6 ×2; T8 WALL line |
| MX11 | a reflected wall's keep-out spans not mapped along the image wall | T6 season walls |

#### The API number — for the coordinator (the number is the coordinator's at the merge; `version.hpp` NOT edited)
F6 grows additively (`reflection()`, `chassis::MirroredRun`, pinned); F10 untouched (`git diff 5f6633b --
include/shulib/chassis/routine.hpp` is empty); everything else is in unfrozen competition/ and math/ headers. The
history paragraph, ready to paste under `kApiMinor` (the brief's §1.5 says "2.9"; if another F6/F10 run merges
first, the next free minor):

```
/// 9 = chunk MIRROR1-A (2026-11-1x; built 2026-11-13), one route, two robots, by the additive path of
/// register row F6: `Chassis::reflection()` and the RAII scope `chassis::MirroredRun` (pinned) — while a
/// mirrored run is open every FIELD number entering the Chassis is reflected and `pose()` reflected back;
/// the new headers math/reflection.hpp (`math::Reflection`, pure), competition/field_mirror.hpp
/// (`mirrored` — a temporary FieldMirror refused at compile time —, `isMirrored`, `FieldMirror`) and
/// competition/override_mirror.hpp (`kSkillsMirror`);
/// `AutonEntry::mirror` and `AutonRunResult::mirrored`, appended. No frozen member changed shape, no
/// enumerator was added or moved, F10 untouched; every 2.8 program compiles and runs exactly as before (an
/// unmirrored run through runAuton hashes byte-identical, tank and X-drive). Nothing to do.
/// docs/changelog.md, API 2.9, lists it.
```
The changelog entry sits at the top of `## API 2.8` (the task's rule); the merge moves it under a new `## API 2.9`
heading with the bump, as STEP1-A's and CURVE1-A's were.

#### Prose review (pace rule 4) — the greps were RUN and their hits READ before this entry was written
For every changed source file, `grep -rln '<name>' docs/*.md docs/guide docs/cookbook README.md test/README.md`:
- `chassis.hpp` → README.md :222 (an include line: unchanged meaning), guide 09 :57 and guide 10 :38 (pointers to the
  header's commentary: still true — the banner gained "The MIRRORED RUN"), roadmap (F6 row — amended), the register,
  the changelog (entry written). Guide 10's "Reading state" paragraph gained `reflection()` (it lists the state
  members; without it a reader would not know `pose()` can be robot one's numbers).
- `match_runner` → the register, the changelog: no stale text. `auton_registry`, `auton_menu` → the changelog only.
- `auton_wall_reset` / `wall_reset_call` → cookbook 05 (the everyday call's recipe — unmirrored, still exact), guide
  03 :340 (the includes an auton needs — unchanged), guide 10 :468-473 and :676-689 (both forms described — each
  gained one sentence: the explicit form's refusal in a mirrored run, the everyday form's swap and its line).
- `toggle_visit` → guide 20 :316+ (the step's section — gained the mirrored paragraph), guide 11 (TGL — gained the
  real-Toggle note), test/README.md (the host Toggle: unchanged), the register, the roadmap: no stale text.
- `teleop_loop` → guide 10, the register, the changelog: no sentence says the loop refuses or ignores a chassis
  state at entry, so nothing stale; the new clear is documented in guide 11 (`MIR`) and the changelog.
- `auton_tester_panel` → the register only; `competition_screen` → no public hit. The START line's words are in
  guide 17 (`START MIR`) and guide 11.
- New headers `reflection.hpp`, `field_mirror.hpp`, `override_mirror.hpp` → guide 02, the register, the changelog.
Concept greps: `re-seated` (guide 11 and 17 carry the mirrored form; the cookbook/guide 03 hits are an auton's
comment), `29 characters` (guide 16 :126 — gained "25 for a mirrored entry"), `<name> <KIND>` (no public hit),
`mirror` (every public hit read: guide 13, 18, 19, the master plan, the VexBuilder doc and roadmap use the word in an
unrelated sense — "mirrored clause", a mirror-mounted motor, "mirrors migrateProject" — none contradicts this chunk),
guide README (chapters 2 and 17's rows gained the topic).
Deliberately NOT touched: `version.hpp` (the number is the coordinator's), `test/README.md` (no new sim plant or rig
convention), guide 09 (the Routine is unchanged — F10 untouched), guide 16 beyond the name budget (the tester runs a
mirrored entry exactly as any other), README.md (no feature list there names autons' registration).
The C7 removability property: no public doc added names `docs/internal/`, `chunks/`, `-PROGRESS` (the build's
check-removability passed).

#### ~12:00–12:40 — every gate, final lines (the tree as it stands, NOT committed)
- `python3 tools/suite_run.py green` → `suite run: green — 3673 cases, 3877357 assertions` (3627 at the base).
- `python3 tools/src_build_gate.py self-test` → `src build gate self-test: OK (306 detector cases, each against a real
  build)`; `check` → `src build gate: PASS (all 8 builds link; ...)` (each of bench, xdrive, tank, tank-drive,
  tank-calibrate, tank-autontest, tank-competition, tank-tune: exit 0, 12/12 TUs, 0 disallowed warnings).
  `git diff -- Makefile src/ | grep -c '(self-test plant'` → 0.
- ARM header gate (all 258 headers in one TU, `-Werror`, cortex-a9 softfp) → exit 0.
- `ctest -R compile_refusal` → `100% tests passed, 0 tests failed out of 280` (unchanged: no refusal added — every
  MIRROR1 refusal is a run-time sentence or a precondition, D5 (c)).
- CI greps: PROS-free PASS; core never includes sim PASS.
- `api_doc_tool.py` self-test OK; check-coverage `6434 public entities across 214 headers, all documented`;
  check-fresh exit 0; check-examples `905 quoted lines, all verbatim (16 source files); 6 listings of whole-run files,
  each one unbroken run`; check-removability `no public doc references docs/internal/`.
- `doc_staleness_audit.py` self-test OK; the audit `clean (34 live docs, 0 numeric claims checked)`.
- `briefing_status.py check` → `generated block current` (after the regenerate with the fresh binary: 3,673 cases,
  258 headers, HA next free HA-261, F6 pins 69).
- The brief's §7: `git diff 5f6633b -- routine.hpp` EMPTY; `-- math/frame.hpp math/angle.hpp` EMPTY;
  `reflection.hpp`'s purity grep EMPTY; the word grep over the new headers, cookbook 09 and `test/*mirror*` EMPTY
  (after the boilerplate rewording above); the non-ASCII grep's hits are all pre-existing COMMENT lines of
  chassis.hpp / teleop_loop.hpp (arrows, dashes, degree signs) — none a log string, none MIRROR1's; every
  HA-258..HA-260 id has its register row and a code site.

#### Deltas from the brief (each with its reason)
1. **`version.hpp` not edited** — the task's rule: the API minor is the coordinator's at the merge (paragraph above).
2. **`FieldMirror` = {name, reflection, the season's walls}** — not "the season's two image functions": a wall's image
   is the GEOMETRIC reflection matched against the season's wall list (the real image wall returned, with its own
   keep-outs; an own line reflected geometrically), and a Toggle's image is found geometrically from
   `kTogglePlaces` in override_mirror.hpp (`toggleImage`). One formula, no per-season function to get wrong; the
   static_asserts prove the season's walls and Toggles close under the mirror; the generic header carries no season
   type.
3. **`AutonEntry::mirror` is a `const FieldMirror*`, forward-declared** — the registry does not include the season.
4. **`mirrored()` compares mirrors by REFLECTION, not by object** — a copy of the same mirror toggles back; a different
   reflection is the precondition the brief names; an identity "mirror" is refused too.
5. **The ARMING line says `MIRRORED` too** (the brief named the re-seat line and the second line) — a mirrored run
   that never starts is otherwise unnamed in the log.
6. **`runAuton` refuses a Chassis whose mirror leaked** (a precondition; not in the brief) — without it an UNMIRRORED
   auton would run reflected.
7. **`localization/wall_reset.hpp` changed** (not in §1.2's list) — `appendWallResetLine` gains the defaulted
   `sideNote`, the only way to put the note INSIDE the sensor's parenthesis (R8); every existing line byte-identical
   (L6's pins unchanged; guide 11's quote test now expects seven lines, its name extended).
8. **`ToggleVisit` holds the request BY VALUE** (`realRequest`) — it held a reference; the swap needs a copy.
9. **`MirroredRun` also refuses the identity.**
10. **The `[MIR]` half-turn Warn on every drivetrain** — the brief gave the holonomic reason; on a tank a moveTo whose
    heading changes by 180° is TANK1's unsteered case and still rotates by `errorTo`, so the tie applies there too.
11. **T2's tank route is route-G-like (kept off the diagonal, 20 steps, a trigger of each kind)**, not P1's route,
    which crosses the diagonal (landmine 13).
12. **T3:** 179.9° and 180.1° turns are mirrored EXACTLY and ARE warned (within 1°); 178.9° is not; the edge at exactly
    179.0° is not claimed (its comparison sits at floating precision). The X-drive exact-180° moveTo MEASURED 0.307 in
    and pinned 0.30–0.315.
13. **T4's "an unmirrored run byte-identical to a fresh one"**: after a cut, an abandon or a throw the plant is
    wherever it stopped, so a fresh stack cannot share its state. Pinned instead: the mirror off; the follow-up
    RELATIVE run's every record of its leg aims BIT FOR BIT at the written target (a leaked mirror aims at a
    reflection); it seats unmirrored and ends there. (The never-leaked case IS byte-identical: T10's hash.)
14. **T10's route writes `-0.0` on purpose** (MM12's sensitivity) and was re-measured on a clean `git archive`.
15. **T11 (i) fails at 10.02 s** (the first pace under 6 in), not the brief's 10.1 s (P5's TOUCH on a bare chassis);
    the squares touch later in the same run (least gap 0).
16. **T11 (vi)'s 5.9 / 6.1 in routes are robot one parked square-on to the line** (a pause); the exact-6.0 edge is on
    hand-built tracks (it lands exactly; a real run's gap goes through a √2).
17. **The pair check names the robot's DRIVE MOTION ("motion 8 (CurveTo)")**, not the Routine's step number — the
    Routine exposes no step index while it runs, and the scheduler's command id and the motion's name do.
18. **T9 is `autons (1d)`**: today's robot two registry has no mirrored entry, so it checks none and says so; the
    registry walk itself is proven on a planted registry (T11's last case) and by cookbook-09a.
19. **Cookbook 09 has no Toggle-visit listing** — the Toggle step has only its explicit form, which no team auton can
    call yet (guide 14 says so; AF1S-A2 brings the everyday form). The swap is stated in prose and proven by T7;
    AF1S-A2 adds the visit line to page 09's route.
20. **Guide 17's section is a `##` after "The schedule, refused by name"**, not inside "Registering an auton" after
    "The two start modes" — so the start-mode and schedule subsections stay together; its anchors are the ones the
    other pages link.
21. **D2 (the coordinator's call): no `src/team` line** — robot two as robot one's twin is not on this base (src/team
    holds robot two's current file, `autons.cpp`); cookbook 09 shows the line, quoted and compiled; the coordinator
    adds it at ROB1-BT's merge.
22. **RPV1 hook:** no RPV1 file touched. The hook is `mirror_pair::checkMirroredPair(entry, options, &tracks)`
    (`test/mirrored_pair_check.hpp`): both robots' TRUE pose and motion for every pace, from the same function the
    suite judges with — RPV1's follow-up can draw both robots and the 6 in band from it. A mirrored entry already
    previews mirrored with no RPV1 change (RPV1 runs `runAuton`).
23. **`kRobotSquareIn = 15.0` is the pair check's own** (RPV1-A's `kPreviewFootprintIn` is not on this base) — the
    second to merge makes them one constant.
24. **HA register count 223 → 226** (a shared line; ROB1-INT's HA-251..HA-257 change it too — the second to merge
    recounts).
25. **Test files beyond §1.4's list**: `test/mirror_unchanged_runs.hpp` (T10's fixture), the T8 cases in the two
    screen tests and the menu case in `mirrored_auton_test.cpp`.

## MIRROR1-A review findings and their disposition

*(The builder again, on the coordinator's computed task: three reviewers' 26 findings — correctness 5, tests 7, docs
14; overlaps deduped below into 24 items — each RE-CHECKED before its disposition. Started 16:01 (read from `date`).
Scratch for this pass: `.claude/coordinator/scratch/mirror1-a/fix/`. Correction at the resumption, 23:4x: the first
print said "25 findings — docs 13"; the task's list holds 26, 14 of them docs — recounted.)*

Each finding was re-checked at its cited `file:line` (or re-run) BEFORE its disposition; overlaps are taken
together. **25 of 26 REAL and fixed at the root here; 1 (merge coordination) is the coordinator's at the merge
and is logged for it below.** Nothing was dismissed. *(This pass was STOPPED partway — the session ended; the
laptop restarted ~23:28 — and RESUMED at 23:32: see "Resumption" at the end of this section.)*

**1. The exact-180° tie on a TANK leg (correctness, MAJOR) + the docs saying "holonomic only" (docs).** REAL.
Re-checked: `warnHalfTurn` has no drivetrain check (delta 10 said why) while the banner, `mirroredLeg`'s comment,
guide 17, guide 14 and the changelog said "a `moveTo` / trajectory leg on a holonomic drive" and quoted the
TURN's 0.15 in. Measured on the host test plant with T3's own rig (two robots, robot one from (-36, -58, 0)):
a TANK `moveTo(-20, -58, 180°)` — path and END **5.705 in** from robot one's reflection (robot one ends
(-20.0, -60.9)); a Routine `driveTo(-50, -58)` (a point straight behind; a `driveTo` IS a `moveTo` whose heading
is the bearing) **4.988 in**; at pace 10 robot one is at +29.4° (counterclockwise from 0) and robot two at
-60.6° (counterclockwise from -90 too — its mirror would be -119.4°). The reviewer's numbers, reproduced. The
same drive after **turning first in two 90° turns** mirrors to **6.0e-13 in**, no `[MIR]` line. And the tank
leg is ALREADY TANK1's unsteered one — both robots log `tank moveTo not steered ... -- turn first: face()`.
**Fixed:**
- `chassis.hpp`: the banner's MIRRORED RUN bullet and `mirroredLeg`'s comment say ANY drivetrain, name a
  `driveTo` / `reverseTo` as a `moveTo`, and give the three measured sizes (a turn 0.155 in; an X-drive leg
  0.307 in of path; a TANK leg ~5.7 in at its END) with the advice per verb. *[Corrected in the verification
  round (V1, below): 5.7 in is the 16 in leg's; the size GROWS WITH THE LEG — 14.2 in on a 36 in leg.]*
- **The `[MIR]` line's advice now names the verb's own fix** (the reviewer's "the advice does not address a
  moveTo"): a `turnTo` keeps `-- write two 90 deg turns where it matters` byte for byte; a `moveTo` /
  `followTrajectory leg` reads `-- turn first, in two 90 deg turns, where it matters`. `warnHalfTurn` takes a
  `turn` flag from its two callers (no string compare). The longest line ("followTrajectory leg") is **194
  bytes**, under the 200-byte cut — T8 pins it whole and its size.
- **T3 tank case (NEW)** `MIRROR1 T3 tank: an exact 180 deg moveTo / driveTo ENDS inches from the mirror, said
  once; turning first mirrors exactly`: pins 5.65 < end < 5.75 (moveTo) and 4.94 < end < 5.04 (driveTo), both
  robots sweeping counterclockwise, one `[MIR]` line (the leg's advice, exact), TANK1's Warn on both robots, and
  the advice route at ≤ 1e-9 with no line. The X-drive T3 case now pins its whole `[MIR]` line.
- Docs: guide 17's exception paragraph rewritten as three sizes and two fixes (a turn: two 90° turns; a tank
  leg: turn first) plus "on a robot the IMU makes a leg that close to 180° a coin toss for each robot" (its
  tank bullet does NOT say "both our robots": robot one's drivetrain is still the brief's D6 belief — a
  parenthesis saying so was removed at ~16:44, first logged at the resumption); guide 14's
  bullet; guide 11's `MIR` item (both advice forms); cookbook 09's watch-out; the changelog (the Warn's scope
  and advice; the proved-on-the-plant numbers). API page regenerated.
- **For the coordinator / the team lead:** the brief's D6 says "Robot one is believed a tank, where a moveTo's
  heading follows its travel — nothing changes for the 21st". The measurement says otherwise for a tank LEG
  whose heading change is ~180° (TANK1 already warns on such a leg, so a well-written tank route turns first
  and is exact). The brief is the coordinator's copy of record: NOT edited here.

**2. The pair check names "motion N (Kind)", not the step (correctness minor + docs minor).** REAL. Delta 17
stands (the Routine exposes no step index to an outside observer; F10 untouched — `git diff 5f6633b --
routine.hpp` still empty), but the sentence was hard to act on. **Fixed at the root of "find the line":**
each `Pace` now carries the active motion's TARGET from its own latest LIVE record (`DebugRecord::targetPose`,
searched among the newest 8 records because other emitters' id-0 records interleave — a probe,
`fix/rec_probe.cpp`, showed `lastRecord()` alone never matched), and robot one's step name prints it in the
route's own numbers: `motion 8 (CurveTo ending at (12.0, -18.0))`, `motion 5 (MoveToPose to (-6.0, -6.0))`,
`motion 1 (TurnTo to 90.0 deg)`; a hold or brake has none; robot two's stays bare. The file banner's new
"Naming the step" section says what the number counts (drive motions; each followTrajectory leg; a then() or
Toggle step counts the drives it runs; a wall reset, pause, waitFor or mechanism step none) and that it is NOT
the Routine's "step N". **Tests:** T11 (i) pins the WHOLE sentence (as guide 17 quotes it); T11 (iv) pins
`motion 5 (MoveToPose to (-6.0, -6.0))`; NEW `MIRROR1 T11 the step names: ...` lands every form on hand-built
tracks (turn, curve, reverse curve, drive, strafe, hold, between motions, before the first, a motion with no
live record yet, the bare form, the run over). **Docs:** guide 17's sample line and its "Read it as"
paragraph (the target, and the motion-count sentence the docs reviewer asked for); the changelog.
**For the coordinator:** delta 17 (the motion, not the Routine's step index) still needs your — or the team
lead's — acceptance against the answers block's "naming the step"; the target makes the step findable.

**3. `mirrored()` accepts a temporary `FieldMirror` (correctness minor).** REAL — the reviewer's probe compiled
silently. **Fixed:** `AutonEntry mirrored(const AutonEntry&, const FieldMirror&&) = delete;` (documented: the
entry keeps the ADDRESS; a const-rvalue deletion catches a non-const temporary too, which binds there first).
**Test:** NEW `MIRROR1 T5 mirrored() refuses a TEMPORARY FieldMirror at compile time; a named one binds` — a
`requires`-concept in a template (a non-dependent requires-expression outside a template would make the program
ill-formed, not false) with four `static_assert`s (a named const / non-const mirror binds; a temporary and a
const temporary are refused). Mutation KD (the deletion removed) — see the table: the build of the test file
stops AT THE TEST'S OWN static_assert, by its message. Not added to the compile-refusal harness: that harness
is the mechanism core's (its banner: "one TU per BUILD-TIME refusal of the mechanism core"); the count stays
280.

**4. T7 never ran the Toggle step inside an open `MirroredRun` (correctness minor).** REAL (T7 called
`checkAndFlipToggle` through the rig, the Chassis's mirror never open). **Fixed:** T7's robot-two block runs
the same visit again with `chassis::MirroredRun{scoped.chassis, kSkillsMirror.reflection}` open (runAuton's
configuration) and asserts the same Toggle, "ours", wall check, verdict, the same stroke count and the
byte-identical `[TGL]` line. Mutation KT7 (the step's `estimate()` read through `chassis_.pose()` — the front
door — instead of the localizer) turns ONLY the new scoped assertions red (the table gives the lines): the old
T7 could not see it.

**5. T10's fixture banner said "two FIELD-start autons" (correctness minor).** REAL. Rewritten with finding 8
below (the fixture now names exactly what it runs).

**6. K1: a 16-point mirrored curve could run UNREFLECTED, suite green (tests, MAJOR).** REAL — the "16
accepted" case only checked `CHECK_NOTHROW`. **Fixed:** NEW `MIRROR1 T4 a mirrored 16-point curve (the most a
curve takes) IS reflected: the path pace by pace` — exactly 16 points (`REQUIRE(== kMaxCurvePoints)`), a stated
heading on the last, on robot one unmirrored and robot two mirrored: the path within **1.2e-13 in** pace by
pace (≤ 1e-9 asserted), robot two ending at **(46.44, 9.88)** — within 3 in of the reflection of the last point
(46, 10), over 50 in from the point itself — no `[MIR]` line. K1 re-run: the table.

**7. The mutation log's table named tests that did not go red; the driver's parser over-reported (tests,
MAJOR).** REAL — re-checked against `mut/results-115014.json`: MM7 listed 2 / doctest 1 failed, MM16 5 / 2, MX8
3 / 1 (a passing case that prints a MESSAGE gets a `TEST CASE:` header too). The first table's verdicts (all
RED) stand; its NAMED cases were wrong in those three rows. **Fixed at the root:** a new driver
(`fix/mut/mutate2.py`) splits doctest's output into its `=====` blocks and counts a case red only when its
block carries an `ERROR:` line, keeps each red case's failing LINE numbers, and checks its count against
doctest's own `N failed` (a mismatch is a verdict of its own, never hidden). It keeps the first driver's
rules (one exact replacement, a FRESH small binary — the object directory emptied, every TU recompiled — the
4 GiB cap and the 600 s timeout, the restore in `finally` and on SIGTERM/SIGHUP/SIGINT, the mtime bumped).
**Every mutation was RE-RUN on the final tree** with it — the table at the end of this section (run at the
resumption, on the final tree) REPLACES the first table's
"Red" column (the first table is left above as it was written, as the record of that run).
**And a test made stronger where the corrected parse showed a gap:** with MM7 (curve headings not reflected),
T2 X-drive stayed green — its route's curve stated an end heading of **-45°, which lies ALONG the mirror line**
(-90 - (-45) = -45: its own reflection), so a mutant that skips reflecting curve headings changes nothing there.
The X-drive route's curve now ends at **-30°** (`test/chassis_mirror_test.cpp`, the X-drive route's `curveTo`;
its comment says why), and the re-run turns T2 X-drive red under MM7 too (the table). *(Made in the stopped
pass at ~16:33, after its own mutation run; first logged at the resumption.)*

**8. T10 hashed the team's own autons file and robot two's mechanism list (tests minor).** REAL — and the
brief's §2.2 T10 asked for "today's robot two registry", so this is a brief-level choice made better rather
than a builder slip. A library test that a student's edit (or ROB1-BT's merge, or the D2 line) turns red
pins the wrong thing. **Fixed:** `test/mirror_unchanged_runs.hpp` now OWNS everything it runs — a copy of
the 40 in auton as the base had it, the library's `acceptanceRunEntry(Match, 27 s, 29 s)`, the every-verb
FIELD auton, and a copy of robot two's mechanism list at the base (two `Power` pivots and the pneumatic claw)
on its own fakes; it includes no `src/` file (no `team/autons.hpp`, no `robots/mechanism_table.hpp`) *[FALSE as
written — through `auton_tester_rig.hpp` (for `WeakSide`) it pulled in `robots/mechanism_table.hpp` and two
`programs/` headers; made TRUE in the verification round, V2]*. Banner
rewritten (finding 5: it says three autons, which). **Re-measured** with the new header against a FRESH clean
`git archive 5f6633b` (`fix/base-5f6633b/`, no `math/reflection.hpp` in it — checked) and against the tree,
each twice (`fix/t10_probe.cpp`): **tank 0.91 → 0x0fff1e8756c26e85, X-drive → 0x61e0c4294783616a** — the SAME
two numbers as before, as they must be for an exact copy (the pins in `mirrored_auton_test.cpp` unchanged;
its comment rewritten). Sensitivity checked: the fixture's 40 in moved to 41 in gives 0x36b6144b012a3e80 /
0xceee43eae0d7254d. The review's two independence probes re-run through the driver (`expect: green`): E1 (a
student's 40 → 41 in in `src/team/autons.cpp`) and E2 (a third auton) — see the table.

**9. K2: the half-turn window pinned only between 0.1° and 1.1° (tests minor).** REAL. **Fixed:** the T3 window
case now runs 179.9, 180.1, **179.05, 180.95** (0.95° from the half turn, warned) and **178.95**, 178.9 (1.05° and
1.1°, not warned) — each mirrored exactly; 0.05° either side of the 1° edge, far above floating precision (the
edge itself is still not claimed). K2 (0.2°) and MX8 (2°) re-run: the table.

**10. K5: the NO SENSOR refusal in a mirrored auton could name robot one's wall (tests minor).** REAL. **Fixed:**
NEW `MIRROR1 T6 a mirrored reset on a robot with NO wall sensors: refused naming the real wall, side and note`
— the wall robot built from a file declaring no wall sensors (`robotWith({})`, the slot null): the mirrored
entry's one line is exactly `right wall x REFUSED no sensor (LEFT sensor, mirrored from red wall RIGHT): this
robot declares no wall sensors`, the report's side LEFT; unmirrored, L6's line byte for byte.

**11. K6: `mirrored()`'s "has a name" precondition untested (tests minor).** REAL. **Fixed:** T5's refusal case
adds a mirror named `""` and one named `nullptr` (both the skills reflection): each refused with the
precondition's exact sentence.

**12. K8: `acrossLineAt`'s 1e-9° tolerance untested (tests minor).** REAL. **Fixed:** T1 adds 45 ± 1e-8°
(refused) and 45 ± 1e-10° (the Diagonal) — the degrees round trip moves a value near 45 by ~1e-14°.

**13. HA-258's settle labelled "MIRROR1 Step 0.1" (docs minor).** REAL (the brief's Step 0.1 is "where each
robot starts"). **Fixed** in the summary row and the checklist entry: "On the event's field: tape ...".

**14. Guide 17's numbered list broke at its code fences (docs minor).** REAL by inspection (a column-0 fence
ends a list item; Python-Markdown then restarts the second list at 1). **Fixed:** both fences and the "Read it
as" paragraph indented 3 spaces into their items. `check-examples` still reads the indented `cpp` block — proved
by drifting one line of it (`pause(4_s)` → `5_s`): the gate failed naming guide 17's block 5; restored.
(Not rendered through mkdocs here: the site's `pymdownx.superfences` — which nests a fence in a list item —
is not installed in this environment, and python-markdown 3.5.2's plain `fenced_code` does not nest fences at
all, so it is no judge of either form. The 3-space form is the house precedent: guide 08 :19-23 nests a
` ```sh ` fence in item 1 exactly so.)

**15. Landmine 7 — the first paragraph on the mirror did not say the log and screens are robot two's real
field (docs minor).** REAL. **Fixed:** "What the program does with a mirrored entry" now ends: everything the
robot REPORTS — the log, the screens, the localizer, `lastCompleted()` — is robot two's real field; only what
the route reads through the Chassis is robot one's (linking "Below the mirror").

**16. Guide 17's table lacked two D4 rows (docs minor).** REAL. **Fixed:** `startAt(pose)` / `setPose(pose)` (the
position reflected; the heading stays the IMU's) and an entry that runs a `config::Route` (mirrored the same way —
the path runner drives only through the Chassis; T5's route case proves it).

**17. Guide 17 stated which Quadrant each robot starts in as fact (docs minor).** REAL (D1 / Step 0.1 leave it
open). **Fixed:** "one robot in each of red's two Quadrants — here robot one in the red wall's and robot two in
the right wall's, though the other way round works the same, the mirror is its own inverse."

**18. The pair check's footprint belief (a 15 in square CENTRED on the tracking centre) was in no HA (docs
minor).** REAL. The brief says "a fourth belief ⇒ stop and tell the coordinator"; R1's precedent widened HA-260
for the pair check, so **HA-260 is WIDENED here, not a fourth number** — the row, the checklist claim (with the
d-in reasoning), its confidence ("the footprint's place about the tracking centre unmeasured"), its settle
(taped with Step 0.2: how far each robot's front and back reach from its tracking centre) and the additions
paragraph; the pair check's banner names the belief. **For the coordinator:** accept the widening, or reserve
a number and say which.

**19. `kMirrorGapIn` lacked the register's PROVISIONAL label (docs minor).** REAL. **Fixed:** `PROVISIONAL (A4:
HA-260) — a chosen margin, unmeasured.`

**20. Quoted numbers unpinned; guide 11's pin list incomplete (docs minor).** REAL, all three. **Fixed:** (1)
T11 (i) pins guide 17's sentence whole (finding 2's form); (2) cookbook-09a checks `leastGapIn >= 20.0` (the
page's "at least 20 in apart"; measured 20.47); (3) guide 11's list names `test/mirrored_auton_test.cpp` and
`test/chassis_mirror_test.cpp` for the `COMP` and `MIR` lines.

**21. `test/README.md` not updated for the new shared helpers (docs minor).** REAL — my first prose review's
"no new rig convention" was wrong: the pair check IS a shared helper. **Fixed:** three entries — the pair check
(what it runs, the 6 in PROVISIONAL gap, the verdict-not-assertion rule, who calls it), the vectors file (who
reads it), and the T10 fixture (library-owned; re-measure on a clean archive of the base).

**22. The prose review's record missed `localization/wall_reset.hpp` and one `chassis.hpp` hit (docs minor).**
REAL. Greps RUN and hits READ before this line: `grep -rn 'wall_reset\.hpp' docs/*.md docs/guide docs/cookbook
README.md test/README.md` (excluding `auton_wall_reset.hpp` / `wall_reset_call.hpp`, logged before) → the
changelog :754 (L6's entry — the pure core: still true; the defaulted `sideNote` is in MIRROR1's entry),
roadmap :1853 (L6's row), the register's L6 entries (:4050–:4215, sources: unaffected); `appendWallResetLine` /
`sideNote` → the changelog :763 (L6's, still true). `chassis.hpp` → `docs/legacy-command-vocabulary.md` :48,
:211 — the LEGACY chassis header (`include/legacy/...`), unrelated. None stale.

**23. Cookbook 09 did not say where the shared route must NOT live (docs minor).** REAL as a merge item (ROB1-A's
gate step 8 is not on this base). **Fixed what is true now:** "a file of its OWN that both robots' builds
compile — not inside either robot's autons file, so each robot's program carries only its own
registrations." **At ROB1-BT's merge (the coordinator's):** name `src/team/skills_route.{hpp,cpp}` in cookbook
09 and guide 17, and give the gate's reason.

**24. Merge coordination (docs minor) — the coordinator's, at the merge; logged, nothing here to change.**
(1) Beyond the shared lines the first report named, ROB1-INT also changes guide 16, the guide README,
roadmap.md, REMAINING-WORK, build-order and PROJECT-BRIEFING — re-read each at the merge. (2) shulib-v2 moved
past this base: d7b290a (guide 02 — the reviewer's `git merge-file` found 0 conflicts), 52313b7/055ab0b (the new
`docs/README.md` front page: its "Write an autonomous route" list needs a "Run the skills route on robot two,
mirrored" line → cookbook 09, pace rule 3), a5cad04 (the release preflight: this log needs a
`MIRROR1-COMPLETED.md` or the between-parts marker; build-order keeps MIRROR1 at `[~]` until robot two has
run). (3) Also noticed here: `test/chassis_mirror_test.cpp`'s T2 routes name robot two's mechanisms
(`clawPivot6` / `clawPivot7` from `robots/mechanism_table.hpp`) for their triggers, as does
`mirrored_pair_check.hpp`'s `Robot` (`kRobotTwoMechanisms`): if ROB1-BT renames robot two's list, these stop
COMPILING (loud, not a silent re-pin) — the second to merge re-reads them.

#### Resumption — 2026-11-13 23:32 EST (read from `date`), finished 2026-11-14 01:05 (read from `date`)
The pass above was STOPPED partway: its file times run 16:01–17:03 on 11-13; the session ended and the laptop
restarted at ~23:28. Resumed by the builder on the coordinator's continuation task. Found, before anything else:
- **Its dispositions 1–24 were written and their edits APPLIED** (each re-checked here at its file — the test
  cases, the deleted overload, the T10 fixture, the pair check's targets, the docs' lines, HA-258 / HA-260, test
  README); **NOT written: its mutation table and its gate record** (drafted only in scratch,
  `fix/log_part3.md`, never landed here, and that draft's table came from a run older than the final tree —
  below).
- **No mutant and no plant left:** all 40 files under `fix/mut/backup/` are byte-identical (`cmp`) to their tree
  files — nothing needed restoring; `git diff -- Makefile src/ include test | grep -c '(self-test plant'` → 0.
  The `src/` / `Makefile` / `test/vendor/doctest.h` / `include/liblvgl` mtimes of 16:22–16:56 were the src
  gate self-test's touches: `git diff --quiet` against HEAD on each → identical.
- **Two edits the stopped pass made AFTER its 16:17 mutation run and never logged** (found by diffing against
  the reviewers' pre-fix copy, `review-tests/tree/`, and the pass's own guide-17 backup, `fix/g17.bak`): the T2
  X-drive curve's end heading -45° → -30° (16:33; now logged under item 7) and guide 17's "(both our robots)"
  dropped (16:44; now logged under item 1). Every other file changed after 16:17 is generated (API pages,
  mkdocs.yml's generated nav, the briefing) or this log.
- **The suite binary (16:18) predated the 16:33 test edit — a stale link.** Removed; the briefing regenerated
  (reading it as unbuilt); a full build (303 of 304 objects recompiled, 0 warnings); the briefing regenerated;
  `suite_run.py green` → 3678 cases.

**Re-verified on fresh binaries (nothing taken from the stopped pass's word):**
- **Every mutation RE-RUN on the final tree** (the table below; the stopped pass's 16:17 run predates the
  -30° edit, so it is superseded and not quoted). All 39 code mutations CAUGHT (38 RED; KD stops its build AT
  the test's own `static_assert`, by its message); E1 and E2 GREEN as expected.
- **K2 and MX8 re-run singly** with their output kept, to name the angles behind their one failing line (the
  window case CAPTUREs `deg`).
- **T10's two hashes re-measured** with the fixture compiled against a FRESH `git archive 5f6633b`
  (`fix/resume/base/`, no `math/reflection.hpp` in it), run twice: **0x0fff1e8756c26e85 / 0x61e0c4294783616a** —
  the pins; the tree side is the suite's own T10, green.
- **Finding 14's `check-examples` drift probe re-observed** on the current guide 17 (edited after the logged
  probe): `pause(4_s)` → `5_s` in the indented listing → `NOT VERBATIM ... guide/17 (block 5)`, exit 1;
  restored byte-identical (`cmp`), the gate passing again.
- **The measured numbers, printed by the fresh binary** (`-s`): the tank exact-180° `moveTo` path and end
  **5.705 in**, robot one ending (-20.0, -60.85); the tank `driveTo` straight behind **4.988 in**; turned first
  in two 90° turns **6.0e-13 in**; the X-drive `moveTo` **0.307 in** of path, **0.018 in** at the end; the
  16-point curve **1.2e-13 in**, robot two ending (46.44, 9.88); cookbook 09's pair **20.47 in**; T11 (i)'s
  sentence whole, as guide 17 quotes it (also checked by script: the guide's quote == the test's literal).

**One test made stronger here (logged as a delta of this resumption):** the T3 tank case computed the review's
controls — robot one alone with its target 0.5° either side of the tie, and each mirrored — but only PRINTED
them. Guide 17's "a coin toss for EACH robot, which is the same few inches on a tank" *[that wording corrected in
the verification round, V1: not "a few inches" — it grows with the leg]* and "the tie alone
causes it" rested on numbers nothing asserted. Now pinned: each control mirrors to **≤ 1e-9 in** (measured
1.2e-12 / 5.9e-13) and robot one's end moves **5.78–5.88 in** between the two (measured **5.831 in**). Rebuilt
fresh (303 TUs), suite green (3678 cases, 3877430 assertions); mutation **KC** (below) turns the two new
control assertions red. The coin pin is a measurement of robot one's UNMIRRORED run — no mirror mutation can
move it, and none is claimed.

**Still for the coordinator / the team lead (unchanged by the resumption):** (1) D6's premise vs. the measured
tank tie (item 1); (2) delta 17 — the pair check names the drive motion and its target, not the Routine's step
index (item 2); (3) HA-260 widened for the footprint belief rather than a fourth number (item 18); (4) the merge
items (23, 24, D2). *[(1)'s size corrected in the verification round, V1: the end error GROWS WITH THE LEG,
5.7 in on a 16 in leg and 14.2 in on a 36 in leg — the report's "ends 5.7 in from the mirror" understated it]*

#### The mutations, RE-RUN on the final tree at the resumption (23:37–23:40, `fix/mut/mutate2.py`, `spec2.json`, `run_resume.log`, `results-234043.json`)
40 entries: the first run's 29 (MX9's broken first cut left out; its re-cut MX9b kept), 9 new mutations for the
review's findings (K1, K2, K5, K6, K8, KD, KT7, KT, KA) and the review's two independence probes (E1, E2,
expected GREEN). The stopped pass HAD run this spec at 16:16–16:17 (`results-161749.json`), but
`test/chassis_mirror_test.cpp` changed at 16:33 (the -30° curve, item 7) — so that run is not this tree's and is
superseded, never quoted. Before the run: every spec's old text present exactly once, no mutant text present,
the 15 files' sha256 saved (`tree_before_resume.sha`). Each build a FRESH small binary (`test_main.cpp`,
`src/team/autons.cpp` and the named test files; the object directory emptied, every TU recompiled), each run
under `ulimit -v 4 GiB` + `timeout 600`; a case is red only when ITS doctest block carries an `ERROR:` line, and
every row's red count EQUALS doctest's own `N failed` (no mismatch in any row). After the run: `sha256sum -c`
of all 15 files OK; driver exit 0.

| # | Mutation | Red — the cases that FAILED (their failing lines; the filter's other cases passed) |
|---|---|---|
| MM1 | the field heading as -h | T1 vectors (160), T1 10,000 poses (251), T2 tank (287–293), T2 X-drive (304–309) — 4 of 9 |
| MM2 | `pose()` not reflected back | T2 tank, T2 X-drive — 2 of 3 |
| MM3 | `~MirroredRun` emptied | T4 MirroredRun scope (567–583); T4 the runner's every way out (429, 459) — 2 of 6 |
| MM4 | `drive(Body)` keeps the turn rate | T2 tank, T2 X-drive — 2 of 3 |
| MM5 | the wall mapped, the side not | T6 seats right/LEFT (135); T6 no sensor (159, 161); T6 own line (195, 197) — 3 of 6 |
| MM6 | the Toggle images crossed (in `mirroredToggle`) | T7 (310–346: the unscoped block, the scoped block and the image loop) — 1 of 1 |
| MM7 | curve headings not reflected | T2 tank (287–293), **T2 X-drive (304–309)** — 2 of 3 (the X-drive red since its curve ends at -30°, item 7) |
| MM8 | the moveTo observer overload not reflected | T2 tank, T2 X-drive — 2 of 3 |
| MM9 | `mirrored()` does not toggle back | T5 twice = e (304, 309, 317); T5 different mirror (337) — 2 of 8 |
| MM10 | the runner seats the unreflected start | T5 FIELD seat (185–195); T5 park (215–221); T5 route entry (294, 295) — 3 of 8 |
| MM11 | RELATIVE uses the diagonal | T5 RELATIVE (246–258) — 1 of 8 |
| MM12 | the identity re-wraps a turnTo heading | T10 (166, 167: both hashes) |
| MM12b | the identity normalises a moveTo position | T10 (166, 167) |
| MM13 | only the first trajectory leg reflected | T2 tank, T2 X-drive — 2 of 3 |
| MM14 | the end action outside the scope | T5 park (215–221) — 1 of 8 |
| MM15 | the pair check compares with the reflection | T11 (i), (ii), (iii), (iv), (vi), the registry walk — 6 of 8 |
| MM16 | `isMirrored` always false | T11 (iv) (258, 261, 270); cookbook-09a (1117, 1118) — 2 of 9 |
| MM17 | the `+ 0.0` dropped | T1 vectors (163), T1 10,000 poses (254), T1 ChassisSpeeds (341, 342), panel T8 START MIR (798), screen T8 START MIR (684) — 5 of 8 |
| MX1 | the driver loop keeps a leaked mirror | T4 the driver loop (701–720) |
| MX2 | runAuton's leaked-mirror precondition removed | T4 runAuton refuses (511) |
| MX3 | the explicit wall reset's precondition removed | T6 explicit (243) — 1 of 6 |
| MX4 | the menu label loses MIR | T8 the menu (597); cookbook-09a (1109) — 2 of 2 |
| MX5 | the tester's START line loses MIR | panel T8 (788, 798, 803) |
| MX6 | the mirrored name budget 29 | T5 26-character name (379, 385) — 1 of 8 |
| MX7 | a different mirror silently toggles | T5 different mirror (333) — 1 of 8 |
| MX8 | the half-turn window 2° | the T3 window case only (402) — 1 of 4; re-run singly at 00:22 with its output kept (`resume/angles_MX8.out`): **178.95° and 178.9° warned** |
| MX9b | the Toggle step keeps robot one's Toggle | T7 (310–326) |
| MX10 | the WALL line drops the mirrored note | T6 seats (145); T6 no sensor (161); T6 own line (197); T8 the mirrored WALL line (324, 340) — 4 of 7 |
| MX11 | an image wall's keep-outs not mapped | T6 season walls (228) — 1 of 6 |
| **K1** | the curve copy's bound `<` (16 points unreflected) | **T4 16-point curve** (630 the path, 631 robot two's end not at the reflection) — 1 of 4. Survived the review's whole suite |
| **K2** | the half-turn window 0.2° | **the T3 window case** (402) — 1 of 4; re-run singly at 00:22 (`resume/angles_K2.out`): **179.05° and 180.95° unwarned**. Survived before |
| **K5** | NO SENSOR refusal: the request as written, no note | **T6 no sensor** (159 the side, 161 the line) — 1 of 6. Survived before |
| **K6** | the name half of `mirrored()`'s precondition removed | **T5 refusal case** (347 the nameless, 351 the null-named mirror) — 1 of 8. Survived before |
| **K8** | `acrossLineAt`'s tolerance 1e-5° | **T1 acrossLineAt** (296, 297: 45 ± 1e-8° accepted) — 1 of 6. Survived before |
| **KD** | the deleted overload removed | **BUILD FAILED AT THE TEST'S OWN ASSERTIONS**: `mirrored_auton_test.cpp:371` `static assertion failed: a temporary FieldMirror must be refused (it would dangle)` and `:372` (the const temporary). The failure IS the test's `static_assert`, named by its message (not a -Werror or unused-variable break), so it counts as CAUGHT |
| **KT7** | the Toggle step reads its pose through `chassis_.pose()` | **T7 — only the NEW scoped block** (326 verdict, 327 strokes, 329 the TGL line; the unscoped block's 310–315 passed): the old T7 could not see it |
| **KT** | the failure sentence loses robot one's target | T11 (i) (206, the whole sentence); T11 (iv) (272, `motion 5 (MoveToPose to (-6.0, -6.0))`) — 2 of 8 |
| **KA** | the `[MIR]` advice swapped turn <-> leg | T3 exact-180 turn (383); T3 X-drive moveTo (439); T3 tank (518, 530); T8 the `[MIR]` lines (577, 580) — 4 of 5 |
| **E1** (probe, expect GREEN) | a student's 40 in auton → 41 in in `src/team/autons.cpp` | **GREEN** — T10 no longer pins the team's file (the review's E1 turned it red) |
| **E2** (probe, expect GREEN) | a student adds a third auton | **GREEN** — likewise |
| **KC** (00:25, `spec_resume.json`) | a mirrored leg's heading left unreflected (`mirroredLeg` returns robot one's heading) | T3 tank (513–541, and **552, 553: the NEW control assertions** — 0.5° off the tie no longer mirrors); the coin pins 554–555 passed, as they must (robot one runs unmirrored) |

#### Every gate on the final tree (the resumption, 2026-11-14 00:26–01:05; NOT committed)
Run LAST, on the tree with the T3 control pins in it (only this log changed after; the doc gates below read
it unchanged). The src gate ran alone (its self-test plants in `src/`, `include/` and `test/`): no build,
mutation or other gate beside it.
- `src_build_gate.py self-test` (00:26–00:53) → `src build gate self-test: OK (306 detector cases, each against a
  real build)`; `git diff -- Makefile src/ include test | grep -c '(self-test plant'` → **0**; `check`
  (00:53–00:58) → `src build gate: PASS (all 8 builds link; ...)` — bench, xdrive, tank, tank-drive,
  tank-calibrate, tank-autontest, tank-competition, tank-tune: each exit 0, 12/12 TUs, 9 vendor-ignored, 0
  disallowed. (Also run once before the control pins, 23:49–00:21: the same lines.)
- The suite binary relinked from source after the src gate (removed; the briefing regenerated reading it as
  unbuilt; `cmake -S test -B build/test`; a full build, 303 TUs, 0 warnings; the briefing regenerated) →
  `suite run: green — 3678 cases, 3877430 assertions` (3673 at the review; +5 cases: T3 tank, T4 16-point
  curve, T5 temporary FieldMirror, T6 no wall sensors, T11 the step names).
- ARM header gate (all **258** headers in one TU, `-Werror`, cortex-a9 softfp) → exit 0.
- CI greps: PROS-free PASS; core never includes the sim plant PASS.
- `api_doc_tool.py` self-test OK; check-coverage `6435 public entities across 214 headers, all documented` (+1:
  the deleted `mirrored(const AutonEntry&, const FieldMirror&&)`, documented); check-fresh exit 0;
  check-examples `905 quoted lines, all verbatim (16 source files); 6 listings of whole-run files, each one
  unbroken run`; check-removability `no public doc references docs/internal/`.
- `briefing_status.py` self-test OK; check → `generated block current`.
- `doc_staleness_audit.py` self-test OK; the audit `clean (34 live docs, 0 numeric claims checked)`.
- `ctest -R compile_refusal` → `100% tests passed, 0 tests failed out of 280` (unchanged — finding 3's refusal is
  pinned by the test's own `static_assert`s, not the harness: item 3).
- The brief's §7: `git diff 5f6633b -- routine.hpp math/frame.hpp math/angle.hpp` EMPTY; `reflection.hpp`
  includes only math/ and units/; the word grep over the new headers, cookbook 09 and `test/*mirror*` EMPTY.
- `git diff --stat -- src Makefile` → the two screen headers only (`auton_tester_panel.hpp`,
  `competition_screen.hpp`: START MIR), as before the review; no `version.hpp` edit.

## Verification-round findings and their disposition

*(The builder again, on the coordinator's computed task: a fresh verifier's two findings — one major, one minor —
each RE-CHECKED at its cited lines and re-measured on a fresh build before its disposition. Started 01:31 on
2026-11-14 (read from `date`). Scratch for this round: `.claude/coordinator/scratch/mirror1-a/vround/`. The round
STOPPED at ~02:43 on 2026-11-14 — the session ended during its second src gate self-test, before this section
reached the live log (its draft was `vround/log_section.md`) — and was RESUMED on 2026-11-15 at 18:12 EST (read
from `date`), scratch `vround2/`: see "Resumption" at the end of this section, which found a src gate self-test
plant left in the tree.)*

**Both REAL; both fixed at the root. Nothing dismissed.**

**V1 (major). The tank exact-180° tie was told as ~5.7 in ("up to 5.7 in", "by inches", "the same few inches");
on the host test plant it GROWS WITH THE LEG.** REAL. Item 1 above pinned the reviewer's ONE geometry (a 16 in
leg), and the docs then gave that number as the effect's size — guide 14 even as a bound. Re-measured with T3's
own `tieRun` (the test file included whole, `vround/tie_scan.cpp`, built fresh from the tree, run under the
4 GiB cap + `timeout 600`; `vround/tie_scan.out`; rebuilt from the final tree at the resumption,
`vround2/scan/tie_scan.out`, IDENTICAL). Robot one starts at (-36, -58) facing 0. Each leg is either a `moveTo` to
(-36 + L, -58) facing 180°, or a Routine `driveTo` to (-36 - L, -58), straight behind. Each leg's time budget is
4 + L/8 s; the T3 legs' own 4 s and 6 s give the same ends: 16 in 5.7052, 36 in 14.1992 both ways. Each leg logs
exactly ONE `[MIR]` line on robot two:

| leg L (in) | `moveTo` end off the mirror (in) | ÷ L | robot one's own miss of its target (in) | robot one's coin, ±179.5° (in) | `driveTo` behind, end (in) |
|---|---|---|---|---|---|
| 4 | 1.423 | 0.36 | 0.712 | 1.455 | — |
| 6 | 2.135 | 0.36 | 1.067 | 2.182 | 2.135 |
| 8 | 2.846 | 0.36 | 1.423 | 2.909 | — |
| 12 | 4.273 | 0.36 | 2.136 | 4.367 | — |
| 14 | — | — | — | — | **4.988** (pinned since item 1) |
| **16** | **5.705** (pinned since item 1) | 0.36 | 2.853 | **5.831** (pinned since the resumption) | — |
| 20 | 7.143 | 0.36 | 3.572 | 7.300 | 7.143 |
| 24 | 8.804 | 0.37 | 4.402 | 8.997 | 8.804 |
| 30 | 12.149 | 0.40 | 6.074 | 12.405 | 12.149 |
| 34 | — | — | — | — | 13.637 |
| **36** | **14.199** (pinned HERE) | 0.39 | 7.100 | **14.530** (pinned HERE) | — |
| 42 | 15.422 | 0.37 | 7.711 | 15.836 | — |
| 46 | 15.969 | 0.35 | 7.985 | 16.443 | — |
| 52 | 16.534 | 0.32 | 8.267 | 17.100 | — |
| 60 | 16.975 | 0.28 | 8.488 | 17.671 | — |

The verifier's six numbers reproduced exactly: 2.13 (6 in), 4.99 (the 14 in `driveTo`), 5.71 (16 in), 13.64 (the
34 in `driveTo`), 14.20 (36 in) and 15.97 (46 in). So did its 36 in controls: each leg 0.5° off the tie mirrors
to 1.2e-12 / 4.8e-13 in, and robot one's end moves 14.53 in between them. In the scan, every control at every
length mirrors to ≤ 2.4e-11 in.

**The mechanism, found by the scan and now pinned:** the end lies EXACTLY twice robot one's own miss of its
target off the mirror (to 4.4e-8 in on 16 in, 2.8e-13 in on 36 in).
- The leg is unsteered (TANK1's `turn first: face()` Warn), so the tank slides sideways while it sweeps.
- Robot one ends beside its target. Robot two sweeps the same way, so it ends as far beside its own target, on
  the side opposite robot one's image. The two misses add.
- Robot one's miss grows with the leg: 0.71 in at 4 in, 7.99 in at 46 in, 8.49 in at 60 in. That is 0.17–0.20
  of the leg, so the end error (twice it) is about a THIRD of the leg up to ~46 in. It then levels off
  (0.32 of the leg at 52 in, 0.28 at 60 in).
- Nothing beyond 36 in is pinned, and no page quotes it.
- A `driveTo` straight behind is the same leg, number for number.

*(Correction at the resumption: the stopped round's draft said robot one's miss was "about a third of the
leg". That is the END's ratio; the miss is half of it (0.17–0.20 of the leg). Corrected before this section reached the
log. No page or test carried the slip.)*

**Fixed:**
- **Test (T3 tank).** The case is renamed `... ENDS off the mirror by more the longer the leg, said once;
  turning first mirrors exactly`. It is now 31 assertions (was 20), and its banner says why. It adds:
  - the same leg 36 in long (to (0, -58) facing 180°, 6 s): 14.15 < end < 14.25 (MEASURED 14.199), one `[MIR]`
    line on robot two and none on robot one, with the leg's exact line;
  - the 2× relation on BOTH legs: |end − 2 × robot one's miss| < 1e-6 (MEASURED 4.4e-8 / 2.8e-13);
  - the 36 in controls, each ≤ 1e-9 in (MEASURED 1.2e-12 / 4.8e-13), and robot one's coin at 14.48–14.58 in
    (MEASURED 14.530).
- **Docs. Every number they quote is pinned:** 5.7, 14.2, 5.0, 5.8 and 14.5 in. "The two misses add" rests on
  the 2× relation. "Roughly a third" rests on the two pinned legs' ratios, 0.36 and 0.39.
  - Guide 17's tank bullet now gives the mechanism and the two legs, and says a `driveTo` is the same leg.
  - Guide 17's coin paragraph says the coin is as large as the tie: 5.8 in on the 16 in leg, 14.5 in on the
    36 in. The pair check runs the plan exactly, so either robot can end that far from where the check placed
    it. On the 36 in leg that is more than twice the 6 in the check keeps.
  - Guide 14's bullet now reads "by more the longer the leg — 5.7 in on a 16 in leg, 14.2 in on a 36 in leg";
    "up to" is gone.
  - Cookbook 09's watch-out now reads "by more the longer the drive: 14 in on a 36 in drive"; "by inches" is
    gone.
  - `chassis.hpp`'s MIRRORED RUN banner now reads "a TANK moveTo's END by an amount that GROWS WITH THE LEG —
    5.7 in on a 16 in leg, 14.2 in on a 36 in one", and says which side each robot ends on. The API page was
    regenerated.
  - The changelog's "Proved" paragraph was corrected to match.
- **This log.** Markers were added at item 1's "Fixed" bullet, at the resumption's coin sentence and at its
  "Still for the coordinator (1)", each pointing here. The earlier entries stay as written, as the record.

**For the coordinator / the team lead — decision 1, CORRECTED.**
- My report said "an exact-180° tank leg ends 5.7 in from the mirror". That is the 16 in leg's number only.
- On the host test plant the end error is about a third of the leg: 5.7 in on 16 in and 14.2 in on 36 in (from
  1.4 in at 4 in up to 17.0 in at 60 in).
- On a robot, each robot's end is a coin toss of the same size (5.8 in / 14.5 in), and the pair check cannot
  predict which way it falls.
- A typical skills drive of 24–48 in puts the error at 9–16 in. That is more than the pair check's 6 in margin.
- The brief's D6 ("Robot one is believed a tank ... nothing changes for the 21st") should be re-decided on THIS
  scaling, not on 5.7 in.

Unchanged:
- The `[MIR]` Warn fires on every such leg (robot two).
- TANK1's `turn first: face()` Warn fires on both robots.
- The advice still holds: turn first, in two 90° turns. That route mirrors EXACTLY (pinned, 6.0e-13 in).

One option for that decision, NOT built here (D6 is not mine to change): the pair check could FAIL a mirrored
entry whose robot two logs a `[MIR]` leg line on a tank. Then a shared route could not carry such a leg to the
event unseen.

**V2 (minor). T10's fixture said "no src/ file is included", but it pulled three in through
`auton_tester_rig.hpp`.** It included the rig only for `WeakSide`, and the rig brings in `robots/mechanism_table.hpp`,
`programs/auton_abort_watch.hpp` and `programs/auton_tester_run.hpp`. REAL: I read `test/auton_tester_rig.hpp:41-43`.
The hash was never affected: nothing from those headers was used, and E1/E2 stayed green. But the stated reason —
self-contained, so it compiles against the base — held only because those files exist at the base. A rename in
robot two's list at ROB1-BT's merge would have reached the fixture.

**Fixed by making the claim TRUE.** This is the verifier's first option. I chose it over rewording because the
fixture is the library's frozen pin, and a dependency on robot two's list is exactly what item 8 set out to
remove. The precedent is `test/curve_unchanged_route.hpp:47`, CV16's fixture, which carries its own `WeakSide`
for the same reason.
- `mirror_t10::WeakSide` is a copy of `tester_rig::WeakSide`, its body byte for byte as the base had it
  (`git show 5f6633b:test/auton_tester_rig.hpp`, lines 81–89).
- The rig include is dropped. `shulib/sim/degradation.hpp` and `shulib/sim/rng.hpp` are included directly.

**Proved** (`vround/t10/`, and again from scratch at the resumption, `vround2/t10/`):
- The T10 probe compiles with `src/` OFF the include path (no `-iquote src`). `g++ -MM` lists **0** `src/`
  dependencies against the tree, and 0 against a FRESH `git archive 5f6633b` (which has no
  `math/reflection.hpp` — checked; all 120 of its library paths are the archive's own). Both link.
- The CONTROL: the fixture as it was before this round, compiled the same way, FAILS with
  `auton_tester_rig.hpp:41:10: fatal error: programs/auton_abort_watch.hpp: No such file or directory`.
- The two hashes, re-measured twice each on the base and on the tree, are **0x0fff1e8756c26e85 /
  0x61e0c4294783616a** — the pins, unchanged, as an exact copy must leave them.
- Mutation **KW** (the copy ignores its `scale`) turns T10's TANK line red. The X-drive runs at scale 1.0, so
  its line rightly stays green. The copy is live.
- MM12 / MM12b are red; E1 / E2 are green (the table below).

The fixture's banner, the test README's T10 entry ("includes no `src/` file, not even through a rig") and a
marker at item 8 now say so.

**Prose review (this round; the greps run and their hits read at the resumption, 2026-11-15 ~18:13, BEFORE
this line was written).** Changed sources: `include/shulib/chassis/chassis.hpp` (the banner),
`test/chassis_mirror_test.cpp` and `test/mirror_unchanged_runs.hpp`.

V1's concepts: `grep -rn -i "5\.7 in\|few inches\|by inches\|exact 180\|exact-180\|exact half turn\|half turn"`
over `docs/` and `README.md`. Hits read:
- Current and corrected here: guide 14 :517-522, guide 17 :157-177, cookbook 09 :120-123, and the changelog
  :61 and :76-78.
- Guide 11 :261-269 (the `[MIR]` line) is current. It says the sweep moves where a tank leg ENDS and sends the
  reader to chapter 17 for the numbers; it quotes no size.
- Guide 10 :143 (a `turnTo` tie resolves counterclockwise) and roadmap :137 (F3's lock) are current.
- Unrelated hits: guide 2 :194 (the head-to-head half turn, another mirror), guide 9 :421 (a curve), and the
  "a few inches" in guides 1, 3, 5, 16, 20 and cookbook 04 (other subjects).
- The brief, `MIRROR1-mirrored-routine.md` (D6, §0.4, :973), is the coordinator's copy of record. I did not
  edit it. Its D6 premise is decision 1 above.

V2's concepts: `grep -rn "mirror_unchanged_runs\|auton_tester_rig\|WeakSide\|no src/ file"` over `docs/`,
`README.md` and `test/`. Hits read:
- `test/README.md` :169-175 is updated.
- `mirrored_auton_test.cpp` :153-161 ("a student's edit to src/team or src/robots never moves it") is current,
  and now cannot break the fixture's compile either.
- `match_runner.hpp` :68 and `docs/api/match_runner.md` :352 ("measured at the merge base, pinned") are current.
- The rig's own users (`competition_rig.hpp`, `auton_tester_run_test.cpp`, `auton_abort_watch_test.cpp`,
  `robot_two_claws_example_test.cpp`) are untouched; they still use the rig.

Stale beyond the pages this round changed: none.

#### The mutations, this round (`vround/mut/mutate3.py` — `fix/mut/mutate2.py` with its scratch directory moved; `spec_vround.json`)
Each mutation gets a FRESH small binary: the object directory is emptied and every TU recompiled
(`test_main.cpp`, `src/team/autons.cpp` and the named test file). Each runs under `ulimit -v 4 GiB` +
`timeout 600`, and the file is restored in `finally` and on SIGTERM/SIGHUP/SIGINT. A case counts as red only when
its doctest block carries an `ERROR:` line, and the red count must equal doctest's own. Before the run the files'
sha256 were saved; after it, `sha256sum -c` was OK.

The spec ran twice, with the same verdicts and the same failing lines both times (read back from both JSON files
at the resumption):
- at 01:41 (`results-014127.json`), before the banner's last comment edit;
- on the FINAL tree at 02:21 (`results-022140.json`, `run2.log`).

KA was re-run at the resumption, at 18:15 on 2026-11-15 (`vround2/mut/mutate4.py`, `spec_ka.json`,
`results-181515.json`). It ran on a FRESH binary, `sha256sum -c` was OK after, and the driver exited 0.

T3 tank's NEW assertions are at lines 533–537 (the 36 in leg), 541–542 (the 2× relation) and 584–587 (the 36 in
controls and coin).

| # | Mutation | Red — the cases that FAILED (their failing lines) |
|---|---|---|
| KC (re-run) | a mirrored leg's heading left unreflected (`mirroredLeg` returns robot one's heading) | T3 tank (518, 520, **534**, **541, 542**, 550, 561, 563, 574, 575, **584, 585** — the 36 in leg's end, the 2× relation and the 36 in controls among them) — 1 of 1 |
| **KH** | the tie resolved the MIRROR way on a leg (robot two's target nudged +1e-6 rad, so it sweeps clockwise) | T3 tank (518 robot two no longer counterclockwise, 519 the 16 in end, **533 the 36 in end**, **541, 542 the 2× relation**, 549 the `driveTo`, 561, 574, 575, 584, 585 — the 1e-6 rad perturbs the exact-mirror bounds too) — 1 of 1 |
| **KW** | the T10 fixture's `WeakSide` copy ignores its `scale` | T10 166 (the TANK hash) only — the X-drive runs at scale 1.0, so 167 rightly passed: the copy is live |
| MM12 (re-run) | the identity re-wraps a `turnTo` heading | T10 166, 167 |
| MM12b (re-run) | the identity normalises a `moveTo` position | T10 166, 167 |
| E1 (probe, expect GREEN) | a student's 40 in auton → 41 in in `src/team/autons.cpp` | GREEN |
| E2 (probe, expect GREEN) | a student adds a third auton | GREEN |
| KA (re-run at the resumption) | the `[MIR]` advice swapped turn <-> leg | T3 exact-180 turn (383); T3 X-drive `moveTo` (439); T3 tank (523 the 16 in leg's line, **537 the NEW 36 in leg's line**, 552 the `driveTo`'s); T8 the `[MIR]` lines (577, 580) — 4 of 5 |

Not caught by a mirror mutation, and none claimed:
- 586–587: robot one's 36 in coin is a measurement of its UNMIRRORED run, as the 16 in coin's was.
- 535–536: the 36 in leg's `[MIR]` counts check the same Warn code that the 16 in leg's 521–522 already pin.

#### Resumption — 2026-11-15 18:12 EST (read from `date`)
The builder resumed on the coordinator's continuation task. Before anything else, it found:
- **A src gate self-test plant LEFT IN THE TREE: `src/team/shared_helpers.hpp`.** It was untracked, 106 bytes,
  mtime 2026-11-14 02:43:13. It came from the gate's case "a team file of NO robot names robot two's namespace",
  and its content was byte for byte the plant text at `tools/src_build_gate.py:3059-3061`.
  - Cause: the stopped round's SECOND self-test (started 02:22:58, `vround/srcgate2.pid`) was killed mid-case
    when the session ended, so `_newfile`'s `__exit__` never ran.
  - **The standing check cannot see this kind of plant.** `git diff -- Makefile src/ | grep '(self-test
    plant'` misses it twice over: the file is untracked, and a `_newfile` plant's text carries no marker.
  - Removed: moved to `vround2/LEFTOVER-selftest-plant-shared_helpers.hpp` as evidence.
  - Every other plant target of the gate (the paths at `src_build_gate.py:2072-2088`) equals HEAD: `git status`
    is silent on them.
  - `git ls-files -o --exclude-standard src include test Makefile` now lists only MIRROR1's 11 new files.
  - For the coordinator: `git ls-files -o --exclude-standard src include test` belongs beside the plant grep
    after any interrupted self-test.
- **The mutation tooling left nothing applied.** Every file the stopped round's driver mutated is
  `cmp`-identical to its backup in `vround/mut/backup/`: `chassis.hpp` (KC, KH, MM12, MM12b),
  `mirror_unchanged_runs.hpp` (KW) and `src/team/autons.cpp` (E1, E2). `sha256sum -c vround/mut/tree_before2.sha`
  is OK, and `autons.cpp` equals HEAD.
- **All the stopped round's edits were present.** Its section above was still a draft with two placeholders
  (the KA row and the gates). Its markers at items 1 and 8 and at the resumption were already in the live log.
- **The suite binary was a stale link.** It was built at 02:20:02, before `chassis.hpp`'s restore bump at
  02:21:35. I relinked it from source before trusting any suite result (the gates below).

Re-verified on fresh builds, 18:14–18:15; nothing was taken on the stopped round's word:
- the tie scan, rebuilt from the tree, gave output identical to the stopped round's;
- V2's `-MM` and link checks, the old-fixture control, and the hashes, twice each on the tree and on a FRESH
  `git archive 5f6633b` (`vround2/t10/base`);
- the mutation table, read back from both JSON files;
- KA, run (above).

Only this log changed after that.

#### Every gate on the final tree (the resumption, 2026-11-15 18:15–18:52 EST, read from `date`; NOT committed)
Outputs: `vround2/srcgate_times.txt`, `srcgate_selftest.log`, `srcgate_check.log`, `gates.sh` → `gates.out`.
- **The src gate ran ALONE first** — no build, mutation or other gate beside it, because its self-test plants in
  `src/`, `include/` and `test/`.
  - `self-test` (18:15–18:41): `src build gate self-test: OK (306 detector cases, each against a real build)`.
  - Then `git diff -- Makefile src/ include test | grep -c '(self-test plant'` → **0**, and
    `git ls-files -o --exclude-standard src include test Makefile` → **11**, MIRROR1's own (listed again at the
    end).
  - `check` (18:41–18:45): `src build gate: PASS (all 8 builds link; ...)`. Each of bench, xdrive, tank,
    tank-drive, tank-calibrate, tank-autontest, tank-competition and tank-tune: exit 0, 12/12 TUs, 9
    vendor-ignored, 0 disallowed.
- **The suite binary was relinked from source after the src gate.** The binary was removed and the briefing
  regenerated reading it as unbuilt. Then `cmake -S test -B build/test` and a full build: 303 TUs, 0 warnings,
  binary at 18:46:59. Then the briefing was regenerated.
  - `suite run: green — 3678 cases, 3877441 assertions`. That is +11 assertions on the resumption's 3877430,
    exactly T3 tank's 31 vs 20. No case was added: the round strengthened an existing case.
- **T3 tank and T10 alone, with `-s`:** 2 cases, 33 assertions, green. The printed measurements:
  - the 16 in end 5.70519 and the 36 in end 14.1992 in; robot one's miss 2.8526 / 7.09961 in;
  - end − 2 × miss = -4.35e-8 / 2.77e-13; the `driveTo` 4.98825 in; turned first 6.01e-13 in;
  - the coins 5.83086 / 14.5301 in, with controls 1.208e-12 / 5.948e-13 (16 in) and 1.206e-12 / 4.766e-13 (36 in).
- `ctest -R compile_refusal` → `100% tests passed, 0 tests failed out of 280`.
- ARM header gate (all **258** headers in one TU, `-Werror`, cortex-a9 softfp) → exit 0.
- CI greps: PROS-free PASS; core never includes the sim plant PASS.
- `api_doc_tool.py`:
  - self-test OK;
  - check-coverage `6435 public entities across 214 headers, all documented`;
  - check-fresh exit 0;
  - check-examples `905 quoted lines, all verbatim (16 source files); 6 listings of whole-run files, each one
    unbroken run`;
  - check-removability `no public doc references docs/internal/`.
- `briefing_status.py` self-test OK; check → `generated block current`.
- `doc_staleness_audit.py` self-test OK; the audit `clean (34 live docs, 0 numeric claims checked)`.
- The brief's §7:
  - `git diff 5f6633b` of `routine.hpp` and of `math/frame.hpp` + `math/angle.hpp` is EMPTY (0 lines each);
  - `reflection.hpp` has 0 includes outside math/ and units/;
  - the word grep over the new headers, cookbook 09 and `test/*mirror*` finds 0, and so does the same word in
    this change's added lines;
  - the non-ASCII grep's 9 hits are all `//` / `///` comment lines (`chassis.hpp` :97, 101, 119, 174, 211,
    358, 626; `teleop_loop.hpp` :144, 218), none a log string;
  - HA-258 / 259 / 260 each have code sites (in 1 / 3 / 5 files).
- The tree:
  - `git diff --stat -- src Makefile` → the two screen headers only (`auton_tester_panel.hpp`,
    `competition_screen.hpp`: START MIR), as before;
  - `version.hpp` is untouched;
  - untracked files in `src include test Makefile` are MIRROR1's 11 only (`field_mirror.hpp`,
    `override_mirror.hpp`, `math/reflection.hpp`, the 7 test files and the golden vector file).
- After this section was appended (the log was the only file changed after the gates), I re-ran the gates that
  read the docs: the results are the next lines.
- **Re-run after the append (18:52:57, read from `date`):**
  - check-examples: the same line;
  - check-removability: `no public doc references docs/internal/`;
  - check-fresh: exit 0;
  - `briefing_status.py check`: `generated block current`;
  - `doc_staleness_audit.py`: `clean (34 live docs, 0 numeric claims checked)`.

  Nothing else changed after that. The worktree is NOT committed.

## MIRROR1-A — the coordinator's merge verification *(2026-11-15)*

- **Squash-merged from `mirror1-a`** (transport `76b8acd`, base `5f6633b`) onto `9befdb3` (ROB1-A, the homing fix and
  RPV1-A landed since). Conflicts resolved by hand: the guide index (chapter 16 keeps RPV1-A's preview words, chapter 17
  gains "two robots, one route"), the register (both additions — ROB1-A's HA-252..255 and this run's HA-258..260; **7 of
  230** settled; HA-260 WIDENED as the fix round proposed — accepted), the changelog (this run's entry under a new
  `## API 2.9` heading), the briefing (regenerated).
- **API 2.9** (F6-additive): `version.hpp`'s `kApiMinor = 9`, `"2.9"`, the history paragraph from this log.
- **Merge item 23:** the pair check's `kRobotSquareIn` and RPV1-A's `kPreviewFootprintIn` held equal by a
  `static_assert` in `mirrored_pair_test.cpp` (one robot size; both 15 in).
- **Delta 17** (the pair check names the motion, not the Routine's step index): ACCEPTED by the coordinator — the
  motion and its target make the step findable.
- **D2:** robot two's one mirrored line is NOT added — robot two is not robot one's twin on main yet; it lands at
  ROB1-BT's merge with `src/team/skills_route.{hpp,cpp}` named in cookbook 09 and guide 17 (item 23 of the review).
- **D6 RE-DECIDED by the team lead (2026-11-15):** "Block it" — a tank leg that drives AND turns ~180° in one step (the
  fix round measured: it ends about a third of the leg beside its target, and the mirror misses the other way) is to be
  REFUSED by the host pair check, naming the step, so students write turn-then-drive. A small follow-up run (MIRROR1-B)
  before the Nov 18 freeze.
- **The resumption's leftover plant:** the fix2 resumption found and removed an UNTRACKED self-test plant
  (`src/team/shared_helpers.hpp`); the coordinator now also runs `git ls-files -o --exclude-standard src include test`
  before every transport commit (main and every worktree clean on 2026-11-15).
- **Gates on the merged tree** (`.claude/coordinator/scratch/mirror1_a_merge_verify.sh`, log
  `.claude/coordinator/logs/mirror1a_merge_verify.out`, 18:58–20:10): the suite green (**3,752 cases**);
  `src_build_gate.py` self-test OK (377 cases) and check **PASS — all 14 builds**; the ARM header gate exit 0; `ctest`
  **289 of 289**; the API reference regenerated for 2.9 (`docs/api/version.md`) and fresh; the briefing current; the
  staleness audit clean; no plant, no untracked file under src/include/test, no stray root file.
- **The coordinator's own mutations** (`muts_mirror1_a_main.json`, log `.claude/coordinator/logs/muts_mirror1_a_main.out`):
  **3 of 3 caught** — MM-C1 the skills mirror (y = −x) applied as a 180° rotation: caught AT COMPILE TIME by the
  library's own season checks (`competition/override_mirror.hpp` :98-119 — six static_asserts, "the skills mirror maps
  the RIGHT wall onto the RED wall" first; rebuilt by hand to confirm the failure is those assertions); MM-C2 a heading
  reflected as −h (RED, 25 cases); MM-C3 `isMirrored` always false (RED, 3 cases). The restored tree relinked and ran
  green.
- **MIRROR1-A LANDED (API 2.9).** Host test plant only; nothing has run on a robot. Next: MIRROR1-B (D6: refuse an
  exact-180° tank leg in the pair check), robot two's mirrored line at ROB1-BT's merge.

## MIRROR1-B

- **Executor:** an `opus` builder subagent, launched by the coordinator's workflow, in the git worktree
  `.claude/worktrees/mirror1-b` (branch `mirror1-b`, from `3c29b73` = main at API 2.9 with ROB1-C2). Do not commit.
  Scratch: `.claude/coordinator/scratch/mirror1-b/`.
- **Go:** the team lead's D6 ruling, 2026-11-15. Asked: "Should the code block a route step that tells a tank to
  drive somewhere AND turn all the way around in the same step ... so students must write it as turn, then drive?"
  Answer: **"Block it"**.
- **The job:** the host PAIR CHECK (`test/mirrored_pair_check.hpp`) FAILS a mirrored route on a TANK in which any leg
  logs the `[MIR]` half-turn line. It names the step (motion and target, as delta 17's failures do) and gives the
  fix in one plain sentence: turn first, then drive (two steps). An X-drive is not blocked (T3 pins its 180° moveTo).
  A `turnTo` that logs the line is not a leg and is not blocked.

### Log

#### 2026-11-16 02:15 EST (read from `date`) — started

- Read: RESUMING's standing rules; the brief's header, answers block, Re-check R1–R13, D6, D11, §2.2 T3/T11, §2.3,
  §6, §8; this log's V1 ("decision 1, CORRECTED") and the coordinator's merge record.
- Read the code: `test/mirrored_pair_check.hpp` (whole), `test/mirrored_pair_test.cpp` (whole),
  `chassis.hpp`'s MIRRORED RUN banner and `warnHalfTurn` (:942-981), T3 in `test/chassis_mirror_test.cpp`, the
  pair check's other users (`test/autons_test.cpp` (1d), `test/cookbook_examples_test.cpp` cookbook-09a).

#### ~02:16–02:27 (read from `date`) — the baseline, then the check

- **Baseline (before any edit):** host build from `3c29b73` (71 s). The pair cases `MIRROR1 T11*`, `cookbook-09a*`
  and `autons (1d)*`: 10 cases, 81 assertions, green; their output kept (`scratch/mirror1-b/base_pair_tests.out`).
- **The check** (`test/mirrored_pair_check.hpp`): each `Pace` records how many log lines the run has so far, so a
  log line can be placed in the run. `findHalfTurnLeg(track, reflection)` reads robot two's lines for the FIRST
  `[MIR] mirrored <verb> turns N deg: within 1 deg of a half turn` line whose verb is not `turnTo`. Its leg is the
  first live motion after that line. The step is named like the gap failures name robot one's (delta 17), its
  target from robot two's own record reflected BACK through the entry's mirror — the route's own numbers.
  `checkMirroredPair` runs it only when `kin->strafeAuthority() == 0.0` (the library's own test for a tank,
  `motion_scheduler.hpp`), sets `ok = false` and puts the leg's sentence first; a gap failure too is kept after it.
- **Measured FIRST** (a print-only probe case, then removed), each through `checkMirroredPair`:

  | route (robot one's numbers) | drivetrain | `[MIR]` lines on robot two | verdict | robot two's end off the mirror (in) | least gap (in) |
  |---|---|---|---|---|---|
  | `moveTo((0, -58), 180°)` from (-36, -58, 0) — T3's 36 in leg | tank | 1 (moveTo) | REFUSED, motion 1 | **14.1992** (T3: 14.199) | 61.63 |
  | `driveTo(-36, -58)` from (0, -58, 0) — straight behind | tank | 1 (moveTo) | REFUSED, motion 1 | 14.1992 | 61.36 |
  | the same as `face` then `driveTo` | tank | 1 (turnTo) | PASSES | **0.0453** | 60.81 |
  | `turnTo(180°)`, `driveTo(-50, -58)`, trajectory leg to (-36, -58, 0) | tank | 2 (turnTo, then followTrajectory leg) | REFUSED, **motion 3** at 2.80 s | 4.874 | 111.7 |
  | `moveTo((-20, -58), 180°)` — T3's X-drive leg | X-drive | 1 (moveTo) | PASSES | 0.0179 | 89.38 |
  | `moveTo((-20, -58), 179.0°)` | tank | 1 (moveTo, "179.0 deg") | REFUSED | 2.8e-13 | 93.38 |
  | `moveTo((-20, -58), 178.9°)` | tank | 0 | PASSES | 4.4e-13 | 93.41 |
  | `moveTo((-20, -58), 179.5°)` | tank | 1 (moveTo) | REFUSED | 1.2e-12 | 93.26 |
  | `moveTo((-20, -58), 180°)` — T3's 16 in leg | tank | 1 | REFUSED | **5.70519** (T3: 5.705) | 89.19 |
  | into (-4, -4) beside robot two, then straight back to (-30, -50) | tank | 1 | REFUSED (motion 3, 3.11 s) AND the gap (5.67 in at 1.99 s) | — | 0 |

  Notes: a half-turn LEG times out on both robots (robot one's Routine fails too — it ends beside its target). A
  first try of the last route put the leg FIRST: its Routine stopped at the timed-out leg, so the crash never
  came — re-cut with the crash first. **179.0° sits on the `[MIR]` window's edge** (T3 does not claim the edge: it
  is at floating precision); on this plant the line fires, so the check refuses it — pinned as measured.
- **The pair cases unchanged:** after the change the same 10 cases / 81 assertions, every printed line IDENTICAL
  to the baseline (`after_pair_tests.out`; `diff` differs only in the skipped count).

#### ~02:27–02:35 (read from `date`) — the tests, the pages, the mutations

- **Tests** (all measured above first, then pinned): `test/mirrored_pair_test.cpp` T12 (i)–(vi), 6 cases;
  `test/cookbook_examples_test.cpp` `cookbook-09b`, 1 case (the page's two listings, run through the check;
  its failure sentence pinned whole — guide 17 and the changelog quote it).
- **Pages:** guide 17 (the tank bullet, the coin paragraph — it said the check "cannot tell" such a leg; on a tank it
  now refuses it — and item 4 of "Both robots at once"); cookbook 09 (the 180° bullet split; the refused and the
  two-step listings; its "cases" line); the changelog (top of `## API 2.9`); `test/README.md` (the pair check's
  entry). No library header changed: `test/` only.
- **Mutations** (`scratch/mirror1-b/mut/mutate.py`, `spec.json`, `spec_recut.json`; logs `MB*.build.log` /
  `MB*.test.log`; results `results-*.json`). Each: the binary DELETED, the suite target rebuilt (the build log
  checked for every dependent object — `mirrored_pair_test`, `cookbook_examples_test`, `autons_test`,
  `mirrored_auton_test`; for MB10 `chassis_mirror_test` too), the cases `MIRROR1-B*, cookbook-09*, MIRROR1 T11*,
  autons (1d)*` (MB10: and `MIRROR1 T3*`) run under `(ulimit -v 4194304; timeout 600)`, the file restored in a
  `finally` (SIGTERM / SIGHUP / SIGINT restore too) with its timestamp bumped. Every restored file compared with its
  backup: identical. **Every one RED** (the RED cases read from each log's `ERROR:` blocks):

  | # | mutation (in `test/mirrored_pair_check.hpp` unless said) | RED |
  |---|---|---|
  | MB1 | the tank test inverted (`!= 0.0`): the X-drive refused, the tank passed | T12 (i)–(vi), cookbook-09b |
  | MB2 | the tank test dropped: every drivetrain refused | T12 (iv) |
  | MB3 | the refusal not counted (`ok` left true) | T12 (i), (ii), (iii), (v), cookbook-09b |
  | MB4 | a turn's `[MIR]` line refused as if it were a leg | T12 (ii), (iii), cookbook-09b |
  | MB5 | the leg named one pace early | T12 (iii), (vi), cookbook-09b |
  | MB6 | the target not reflected back — **BROKE ITS OWN BUILD** (`back` unused, `-Werror`): re-cut as MB6b | — |
  | MB6b | the same, `back` kept in use: robot two's own numbers printed ("(58.0, 36.0)") | T12 (i), (ii), (iii), (v), (vi), cookbook-09b |
  | MB7 | the half-turn sentence HIDES a gap failure | T12 (vi) |
  | MB8 | only the run's first `[MIR]` line looked at (a turn's line stops the search) | T12 (iii) |
  | MB9 | the log lines not placed in the run (`Pace::lines` never set) | T12 (i), (ii), (iii), (v), (vi), cookbook-09b |
  | MB10 | LIBRARY, `chassis.hpp`: the `[MIR]` line reworded ("of half a turn") — the check must not go blind silently (140 objects rebuilt) | T12 (i), (ii), (iii), (v), (vi), cookbook-09b, and T3's three pinned-line cases |

  The existing T11 cases and `autons (1d)` stay green under every mutation, as they should: none of their routes
  has a half-turn leg.

#### Prose review (pace rule 4) — the greps were RUN and their hits READ before this entry was written (~02:40)

Changed sources: `test/mirrored_pair_check.hpp`, `test/mirrored_pair_test.cpp`, `test/cookbook_examples_test.cpp`.
Each grep over `docs/` (not `docs/internal/`), `README.md` and `test/README.md`:

- `grep -rn -i 'pair check'`: guide 17 :171, :177 (updated here); guide 14 :533 (the gap at the planned timing —
  current); hardware-assumptions :551, :2333, :2338 (HA-260, the gap — current); `test/README.md` :156 (updated);
  the changelog (this run's entry).
- `grep -rn -i 'half turn\|half-turn\|exact 180\|exact-180\|turn first\|turns around\|5\.7 in\|14\.2 in\|14 in on'`:
  - updated here: guide 17 :157-180 (the tank bullet; the coin paragraph said the check "cannot tell which way a real
    robot will sweep" — on a tank it now refuses the leg), :254-268 (item 4); cookbook 09 :120-138.
  - current, NOT touched: guide 14 :525-530 and guide 11 :261-269. Both are true: the library still sweeps both
    robots the same way at the tie, and their advice (turn first) is the refusal's fix. Neither says the pair check
    now refuses the leg. Left so this run stays on guide 17 and cookbook 09 while DOCS3-N1 sweeps every public
    page — **for the coordinator:** one clause each would complete them ("on a tank the pair check refuses such a
    leg — chapter 17").
  - unrelated: guide 02 :198 (the head-to-head half turn, another mirror); TANK1's `turn first: face()` in guides
    04, 05, 09, 10, 11, 12, 14, 16 and cookbooks 01, 04; guide 10 :143 and roadmap :137 (F3's tie — current);
    the changelog's MIRROR1-A entry :130-146 (the record, unchanged).
- `grep -rn 'mirrored_pair_check\|checkMirroredPair\|PairVerdict\|comparePair'` (and `include/`, `test/`):
  - `test/autons_test.cpp` (1d): its name and comment speak of the 6 in only — still true. It asserts
    `v.ok || !v.checked`, and the refusal sets `ok` false, so (1d) enforces the ruling over robot two's registry
    with no edit. Not touched (robot two's autons test; ROB1-BT's area).
  - `test/mirrored_auton_test.cpp` :5, :26: uses `Robot` — unchanged.
  - `include/shulib/competition/field_mirror.hpp` :43-45 (and its API page :238): "fails ... when their 15 in squares
    come within 6 in" — true, silent on the tank leg. NOT touched: a library header edit would take this run out of
    `test/` (API page regeneration). For the coordinator.
  - hardware-assumptions :2339 (`kMirrorGapIn`) and the changelog :119 (MIRROR1-A's entry): current.
- The word grep (the host test plant's forbidden other name) over the changed files: one hit, changelog :3344, an old
  entry — not this run's.
- No log string added (the check's sentence is a test verdict, not a log line); its text is ASCII (checked).

#### Deltas from the task as given (each with its reason)

1. *[The fix round, items 3 and 4: 179.0 is no longer pinned; a "leg" that drives nowhere is not refused.]* **Refused = whatever logs the `[MIR]` LEG line: within 1° of 180°, not only the exact tie.** The ruling's own
   test ("any leg logs the `[MIR]` half-turn line"). On a robot the IMU makes a leg that close a coin toss as large
   as the tie (V1: 14.5 in on a 36 in leg), so the plan's exactness does not make it safe. **179.0° sits on the
   line's edge** (T3 does not claim it); on this plant the line fires and it is refused — pinned as measured, with
   178.9° (passes) and 179.5° (refused) either side of it.
2. **Both failures named, the leg's first.** A route with a crash AND a half-turn leg gets the leg's sentence, then
   the gap's — never one hiding the other (T12 (vi), MB7).
3. **The step is named from robot two's own record, reflected back** — not robot one's motion at the same pace —
   so the name stays right after an `isMirrored` branch or with a weak side. The form is delta 17's ("motion N
   (MoveToPose to (x, y))"); a `driveTo`, `reverseTo` and a trajectory leg are all `MoveToPose`.
4. *[CLOSED in the fix round, item 2: on a tank a RELATIVE entry's leg is now refused.]* **A RELATIVE mirrored entry is still skipped whole**, so a tank half-turn leg in one is NOT refused. The skip is
   MIRROR1-A's design (T11 (v)); D6's ruling did not touch it. **Open for the coordinator / the team lead.**
5. **`cookbook-09b` added to `test/cookbook_examples_test.cpp`** (a shared file): the page's two listings must
   compile and run (`api_doc_tool check-examples`). My lines only: its mapping line and the case after 09a's.
6. **Guide 17's coin paragraph rewritten** (MIRROR1-A's words): it said the check "cannot tell which way a real
   robot will sweep" — on a tank it now refuses that leg.
7. **No library header changed** — `test/` and pages only; the API stays 2.9 with no number to move.

#### Resumption — 2026-11-16 08:53 EST (read from `date`)

The run above was STOPPED by the 5-hour usage limit at ~03:03, inside the src gate's self-test.

- **A self-test plant was LEFT in `src/`.** `git diff -- src/ Makefile` showed one line in `src/build_selection.hpp`:
  `#if defined(SHULIB_ROBOT_XDRIVE_INVENTED_BROKEN)`. That is the src gate's self-test case 17
  (`tools/src_build_gate.py` :3097-3104), planted at 03:03:45 (the file's time) and never restored: the run was
  killed inside it. It carries no `(self-test plant` marker, so the marker grep alone would have missed it.
  **Restored** with `git checkout -- src/build_selection.hpp` and its time bumped. After that: `git diff -- src/
  Makefile include/` EMPTY; the marker grep over the whole diff 0; `git ls-files -o --exclude-standard src include
  test` EMPTY.
- **What was finished, checked:** the code, the tests, the pages, the mutations and the prose review above. The
  three test files' times are before the mutations (02:21, 02:25) and `test/mirrored_pair_check.hpp` and
  `chassis.hpp` are byte-identical to their mutation backups, so the mutation verdicts stand for this tree. Only
  cookbook 09 and this log changed after them (02:54).
- **What was not finished:** every gate. The suite binary (02:36) predates the plant's restore, so it is rebuilt
  from source before any result is read. The gates follow, the src gate alone first.

#### The src gate, alone (08:55–10:58, read from `date`; driver `scratch/mirror1-b/resume/srcgate.sh`, `srcgate2.sh`)

- **First try (08:55–10:10):** the self-test hit its own 3600 s timeout at 09:55:17 (rc 124, no verdict). The
  machine ran two other worktrees' self-tests beside it (`integrate`, `docs3-n1`; load ~14). The kill landed
  between plants: right after it, `git diff -- src Makefile include` EMPTY, no untracked file under src, include,
  test or the Makefile, and `test/` held only this run's four files. **No verdict is claimed from that try.**
- `check` (09:55–10:10, on that clean tree): `src build gate: PASS (all 14 builds link or, robot one's, refuse naming
  exactly their owed facts (4 refused; every one links in the gate's typed copy); ...)`.
- **Second try (10:10–10:58):** the self-test alone again, its timeout 10800 s. **rc 1: every case passes but two,
  53 and 55** (`resume/srcgate2_selftest.log`, five failure lines, all of them those two cases). Both expect robot
  one's Bench Tests build (`one`) to REFUSE its owed Stage A facts. It no longer does: the coordinator typed the
  last one (`kRightTogglePort = 9`) at ROB1-C2's merge, and that record says Bench Tests and Calibrate "now LINK"
  and that the self-test was "NOT yet run on the merged tree". This run's `check` shows the same: `[one] exit 0`.
  **Not this run's:** MIRROR1-B changes nothing under `src/`, `include/`, `tools/` or the Makefile. The ARM builds
  in those two cases do not compile `test/`. So 53 and 55 fail the same way on `3c29b73`. **For the coordinator
  (ROB1's area, `tools/src_build_gate.py` :4236-4251, :4271-4297):** the two cases need a robot-one program that
  still owes facts (`one-drive` refuses its 9 Stage B facts). Not edited here.
- After it: no plant (the marker grep outside this log 0), `git diff -- src Makefile include` EMPTY, no untracked file
  under src, include, test or the Makefile.

#### Every other gate on the final tree (10:59–11:08, read from `date`; `resume/gates.sh` → `resume/gates.out`; NOT committed)

- **The suite binary rebuilt from source after the src gate.** The binary was removed and the briefing regenerated
  reading it as unbuilt. Then `cmake -S test -B build/test` and a full build: 313 TUs, 0 warnings, binary at
  11:00:51. Then the briefing was regenerated.
  - `suite run: green — 3769 cases, 3882542 assertions` (the base: 3,762 cases — +7, T12 (i)–(vi) and
    `cookbook-09b`).
  - The pair cases alone, with `-s` (`resume/pair_cases.log`): 17 cases, 142 assertions, green. Every printed
    number as measured above: 14.1992 in (i), (ii); 0.0453205 in (two steps); motion 3 at 2.80 s (iii); 89.38 in
    (iv); 179.0 and 179.5 refused, 178.9 passes (v); motion 3 at 3.11 s with the gap's sentence after it (vi);
    `cookbook-09b`'s sentence whole.
- `ctest -R compile_refusal` → `100% tests passed, 0 tests failed out of 295`.
- ARM header gate (all **258** headers in one TU, `-Werror`, cortex-a9 softfp) → exit 0.
- CI greps: PROS-free PASS; core never includes the sim plant PASS.
- `api_doc_tool.py`: self-test OK; check-coverage `6439 public entities across 214 headers, all documented`;
  check-fresh exit 0; check-examples `926 quoted lines, all verbatim (18 source files); 7 listings of whole-run
  files, each one unbroken run`; check-removability `no public doc references docs/internal/`. No `///` comment
  changed under `include/`, so `docs/api` was not regenerated (check-fresh agrees).
- `briefing_status.py` self-test OK; check → `generated block current` (only its suite line moved: 3,769 cases).
- `doc_staleness_audit.py` self-test OK; the audit `clean (35 live docs, 0 numeric claims checked)`.
- Words: the forbidden word in this change's added lines: 1, this log's own prose-review line (reworded after);
  non-ASCII in the check's added code lines (comments excluded): 0.
- The tree: `git diff -- include src Makefile` EMPTY; the marker grep's one hit is this log's own sentence above;
  no untracked file under src, include, test or the Makefile; `version.hpp` untouched. Changed: the 4 test files
  and 5 docs (changelog, cookbook 09, guide 17, the briefing's suite line, this log).

## MIRROR1-B review findings and their disposition

*(The fix round, 2026-11-16 ~12:20–, read from `date`. Three reviewers — correctness, tests, docs — returned 21
findings; deduplicated below to 15 items. Each was re-checked against the tree before acting. Scratch:
`.claude/coordinator/scratch/mirror1-b/fix1/`. Host test plant only.)*

**1 (major; correctness #1, docs #1) — the fix "turn first, then drive" is refused again for T12 (i)'s own leg.
REAL. Fixed at the root.** T12 (i)'s leg is a `moveTo` to a point AHEAD that ends facing back. In the library that
is a reverse leg: `reverseTo(x, y)` IS `moveTo(point, bearing + pi)` (`routine.hpp` :516-522). Taken literally
(`turnTo(180)`, then `driveTo`), the advice gives a `driveTo` to a point now BEHIND: refused again (measured, as
the reviewers did: motion 2 at 1.45 s, 179.3 deg). The check now writes the steps for the leg's own shape, in the
route's numbers, from robot two's leg start (the pace before it; `Track::start` for a first leg) and its target:
- end heading within 90 deg of the drive's direction (it drives forwards): `face(x, y), then driveTo(x, y)`;
- otherwise (it backs up): `faceAway(x, y), then reverseTo(x, y)`;
- either way, `then turnTo(h)` when the end heading is more than 1 deg off the drive's own (a reflection keeps
  that angle, so robot two's shape is robot one's).
The phrase stays the ruling's: "turn first, then drive" / "turn first, then back up". WHAT is blocked does not
change. "Drive there, then turn" also passes (the reviewers' probe A); the check gives `faceAway` / `reverseTo`
because that is what the library calls such a leg, and it keeps "turn first". **For the coordinator / the team
lead:** the sentence no longer reads "write it as two steps: turn first, then drive." — it names the steps.
- Pinned: T12 (i) runs the given fix (PASSES; robot two 0.0453 in off the mirror, the `faceAway`'s own exact
  half turn, as in (ii)) and the literal reading (REFUSED again, given `face` / `driveTo` for THAT shape). T12 (xi):
  a leg to a point ahead and to the left, ending facing back, gets three steps; they PASS (3.0e-13 in).
- Mutations MC9 (the shape inverted) and MC10 (the closing `turnTo` never added): both RED.
- Docs: guide 17 item 4, cookbook 09 and the changelog show the `face` form and say a step that ends facing back
  gets `faceAway`, then `reverseTo`.

**2 (major; correctness #2, docs minor) — a RELATIVE mirrored entry skipped the refusal (delta 4). REAL. Fixed.**
The skip's reason (no field positions) holds for the GAP, not for the leg. On a tank, a RELATIVE mirrored entry
now runs robot two ALONE from its seated start (the origin), with `runReflection(entry)` (the robot-frame mirror)
to name the step. A leg found: `ok` false and `checked` TRUE, so `autons (1d)`'s `v.ok || !v.checked` fails it
with no edit. No leg: skipped by name exactly as before (T11 (v) unchanged). On a drive that can strafe it is
skipped whole, as before. Pinned: T12 (vii) (refused, its sentence whole, robot two only in `tracks`; its fix
skipped; an X-drive skipped). Mutations MC6 (skipped whole again), MC7 (refused but unchecked), MC8 (named
through the FIELD mirror): all RED. Docs: guide 17 items 3 and 4, the changelog, `test/README.md`, the check's
banner, `autons (1d)`'s comment. Delta 4 is CLOSED.

**3 (major/minor x3; correctness #3, tests #1, docs) — T12 (v) pinned 179.0 deg on a floating-point edge. REAL.
Fixed.** T3 deliberately does not claim the edge. T12 (v) now pins samples off it on both sides, as T3 does:
179.05 and 180.95 REFUSED (the line prints "179.1"), 178.95 and 181.05 PASS, and 179.5 refused, 178.9 passes. Each
one mirrors to <= 1.2e-12 in. The comment records 179.0 as measured (refused) and NOT claimed, and 181.0 (the same
distance the other way, margin -4.4e-16) as passing. **For the coordinator / the team lead:** the brief's "a tank
leg at 179 deg" sits exactly on the `[MIR]` window's edge, where 179.0 and 181.0 disagree. The reviewers' G9 (the
window's `>=` made `>` in `chassis.hpp`) went red only on 179.0; with the edge unclaimed it would pass T12, as it
passes T3 — T3's policy, kept.

**4 (minor; correctness #4) — a leg that turns 180 deg without driving was refused "while it drives". REAL.
Fixed.** The ruling is about a step that tells a tank to "drive somewhere AND turn all the way around". A `moveTo`
whose target is within the library's own arrival tolerance of where it starts drives nowhere: it only turns, like
a `turnTo`. It is no longer refused: `kInPlaceIn` = `MotionConfig{}.translationSettle.maxError` = 0.5 in (the
robots' own config here). Pinned, T12 (ix): in place passes (robot two exactly on the mirror), 0.4 in passes (0.14 in
off), 0.6 in and 1 in are refused (0.21, 0.36 in). Every one logs its `[MIR]` moveTo line. Mutations MC11 (the rule
dropped) and MC12 (widened to 1 in): both RED. **For the coordinator:** the run's task said "any leg logs the `[MIR]`
line"; this keeps the ruling's own words instead (a leg that drives). The changelog says "drives at least 0.5 in".

**5 (minor; correctness #5, docs "three different fixes") — `face` + `driveTo` was the docs' only fix, wrong for a
refused `reverseTo`; guide 17 gave two turn forms. REAL. Fixed.** The changelog now says "turn first (`face`, or
`faceAway` before a `reverseTo`), then drive". Guide 17's tank bullet no longer adds "in two 90° turns, that route
mirrors exactly"; item 4 says the `face`'s own `[MIR]` line is expected. One form everywhere. NOT changed: the
library's `[MIR]` Warn text ("turn first, in two 90 deg turns, where it matters", `chassis.hpp` :977-980). It is a
log string that T3 pins and guide 11 quotes. **For the coordinator:** "where it matters" no longer fits a tank leg.

**6 (major; docs) — the changelog, cookbook 09 and the check's banner said robot one's miss was 14.2 in. REAL.
Fixed.** T3 pins robot two's end at twice robot one's miss (`chassis_mirror_test.cpp` :542), 14.15–14.25 in on the
36 in leg, so robot one misses by about 7.1 in. All three now say: each robot about 7 in off, on opposite sides of
the mirror; robot two 14.2 in from robot one's image. The run's task text carried the same mix-up ("about a third of
the leg BESIDE its target"); MIRROR1-A's V1 had corrected it once.

**7 (minor; correctness #6, docs prose review) — descriptions left stale outside the change. REAL. Fixed, but two
pages are left for the coordinator.**
- `test/autons_test.cpp` (1d): its comment and its title now name the half-turn leg. Its title now ends
  "— 6 in apart, no tank half-turn leg". No other file names that title (grep).
- `include/shulib/competition/field_mirror.hpp` :43-46: one clause, "On a TANK it also fails one step that
  drives AND turns about 180 deg ... turn first, then drive". Comment only.
- `include/shulib/chassis/chassis.hpp` :256-257 (the MIRRORED RUN banner; the first round's grep hit its API
  page, `docs/api/chassis.md` :1057, and the round's hit list left it out): one clause, "on a TANK, always: the
  host suite's pair check refuses such a leg". Comment only. `docs/api` regenerated (`chassis.md`,
  `field_mirror.md`: the clauses and the moved line numbers).
- **NOT touched, for the coordinator (DOCS3-N1 is sweeping these pages; this run may edit only its own lines in
  guide 17 and cookbook 09):**
  - guide 11 :266-268 ("and on a tank, turn first before such a leg") wants "— the suite's pair check refuses
    it ([Chapter 17](17-the-competition-program.md#both-robots-at-once))";
  - guide 14 :530 ("— turn first, in two 90° turns, then drive") wants "(on a tank the suite's pair check
    refuses such a leg, Chapter 17)".
  Both pages stay true as they are. Pace rule 3: the commit body lists them as deliberately not touched, or the
  merge carries the two clauses.

**8 (minor; tests G1) — naming the LAST leg survived. REAL. Fixed:** T12 (viii), two half-turn legs (the first
carries on after its timeout; the turn between logs a TURN's line too — measured), the first named. MC1 (the
search run from the end): RED.

**9 (minor; tests G2, G4, G5) — three defensive branches never run. REAL. Fixed:** T12 (xii), planted tracks:
- (a) a leg line after the run's last pace is refused, never quietly. The sentence claims no time: the old
  branch would have printed "at 0.00 s". It now reads "a leg logged after its run's last pace ... -- turn first,
  then drive.".
- (b) a pace between motions after the line is not the leg.
- (c) a leg whose first pace has no target takes the target from its next pace.
- (d) a `turnTo`'s line is never a leg, by its verb.
MC2, MC3, MC4 and MC14 (the time branch swapped): all RED.

The first pass of this round's mutations found one SURVIVOR, MB4 (a turn's `[MIR]` line taken for a leg). It is
equivalent in a real run: a TurnTo's record target is the robot's own spot (`turn_to.hpp` :225, "turn in place"),
so the new in-place rule skips the turn too. Case (d) plants that target 20 in away; MB4 re-run: RED.

**10 (minor; tests G8) — the tank boundary was pinned only at strafe authority 0 and 1. REAL. Fixed:** T12 (x),
an H-drive (`motion_rig::hBotKinematics()`, authority 0.35). Its half-turn `moveTo` logs its `[MIR]` line and is
NOT refused; robot two ends 0.027 in off the mirror. The banner says any drive that can strafe is not refused (the
library's own test for a tank, an authority of EXACTLY 0). MC5 (`< 1.0`): RED.

**11 (minor; tests, docs) — the quoted sentence had no gate, and a test comment said cookbook 09 quotes it. REAL.
Fixed.** `cookbook-09b` now reads guide 17 (`source_pins::readFile`) and requires the sentence whole, on its own
line. The comment is corrected: guide 17 quotes the sentence; the changelog's copy is the record and is
deliberately NOT gated (a later change to the sentence must not rewrite history). MD1 (one number changed in guide
17's quote): RED.

**12 (minor; tests) — `autons (1d)` checks zero mirrored entries. TRUE; the claim is now qualified.** Robot two's
registry registers 0 mirrored entries today. Its mirrored line waits for ROB1-BT. So (1d) enforces the ruling over
**0 entries** until ROB1-BT lands it. Its staying green under every mutation proves nothing yet. The planted cases
(T12, `cookbook-09b`) are what carry the ruling now.

**13 (minor; tests) — the report said MB5 turned T12 (i) red; it did not. TRUE.** MB5 turned (iii), (vi) and
`cookbook-09b` red in the first round, as this log already said. The report's table was wrong. The first round's
`results-*.json` `failing_cases` listed every case that printed a MESSAGE; its verdicts were read from the ERROR
blocks, but those JSON files are not evidence. This round's driver (`fix1/mut/mutate.py`) takes failing cases from
the ERROR blocks only and lists the failing lines.

**14 (minor; docs) — the changelog entry did not say breaking or additive. REAL. Fixed:** "**A stricter host check
(no API change):** a mirrored tank route that passed before can now fail the suite". It also now says one comment
each changed in `chassis.hpp` and `field_mirror.hpp`.

**15 (minor; docs) — the status docs and the brief of record do not carry D6's ruling. TRUE; the coordinator's, at
the merge (pace rules 2 and 5), NOT edited here:**
- `build-order.md` :1790-1791 and `REMAINING-WORK.md` :107 still read "MIRROR1-A BUILT ... awaiting its review".
- The brief's D6 (a) ("nothing changes for the 21st"), §2.1 and §9 still define MIRROR1-B as the Toggle swap.
- This log's :16 says "(no MIRROR1-B)".
- The between-parts marker at :3 says "exact-180°". The build refuses a 2° window (that rule is item 3 here).

#### The check after the round (`test/mirrored_pair_check.hpp`)

`findHalfTurnLeg` finds robot two's FIRST leg line and skips a `turnTo`'s line and a leg that drives nowhere
(item 4). It names the step in robot one's numbers and builds the fix (`halfTurnFix`, item 1). `halfTurnSentence`
writes the one sentence. `checkMirroredPair` runs it on a tank for a FIELD entry (after the gap, the leg's sentence
first) and for a RELATIVE one (robot two alone). `Track::start` records the plant's true start. No library code
changed; two library comments did (item 7).

#### Mutations (`fix1/mut/mutate.py`, `spec.json`; logs `fix1/mut/<id>.build.log` / `.test.log`; `results-124932.json`, `run2.log`)

Each mutation ran the same way:
- the suite binary DELETED and the suite target rebuilt;
- the build log checked for every dependent object (`mirrored_pair_test`, `cookbook_examples_test`,
  `autons_test`, `mirrored_auton_test`; MB10 also `chassis_mirror_test`, 140 objects);
- the cases `MIRROR1-B*, cookbook-09*, MIRROR1 T11*, autons (1d)*` run (MB10 also `MIRROR1 T3*`; MD1
  `cookbook-09*`), under `(ulimit -v 4194304; timeout 600)`;
- the file restored in a `finally`, SIGTERM/SIGHUP/SIGINT restoring too, its time bumped.
After the run, each restored file was compared with its pre-run copy (`fix1/orig/`): identical. **25 of 25 RED**
(MB4 after case (d), above). The RED cases are read from the ERROR blocks:

| # | mutation | RED |
|---|---|---|
| MB1 | the tank test inverted | T12 (i)–(xi), `cookbook-09b` |
| MB2 | the tank test dropped (every drive refused) | T12 (iv), (vii), (x) |
| MB3 | a FIELD refusal not counted (`ok` left true) | T12 (i), (ii), (iii), (v), (viii), (ix), (xi), `cookbook-09b` |
| MB4 | a turn's line taken for a leg | T12 (xii) — after (d); SURVIVED before it (equivalent in a real run, above) |
| MB5 | the leg named one pace early | T12 (i), (iii), (vi), (xii), `cookbook-09b` |
| MB6 | the target not reflected back | T12 (i), (ii), (iii), (v), (vi), (viii), `cookbook-09b` |
| MB7 | the leg's sentence hides a gap failure | T12 (vi) |
| MB8 | a turn's line stops the search | T12 (i), (iii) |
| MB9 | the log lines not placed in the run | T12 (i), (ii), (iii), (v), (vi), (vii), (viii), (ix), (xi), `cookbook-09b` |
| MB10 | LIBRARY: the `[MIR]` line reworded | T3 (tank, X-drive, the turn), T12 (i), (ii), (iii), (v), (vi), (vii), (viii), (ix), (xi), `cookbook-09b` |
| MC1 | the LAST leg named (G1) | T12 (viii) |
| MC2 | a line after the last pace goes quiet (G4) | T12 (xii) |
| MC3 | a pace between motions taken for the leg (G2) | T12 (xii) |
| MC4 | the leg's first pace used as it is (G5) | T12 (xii) |
| MC5 | the tank test widened to `< 1.0` (G8) | T12 (x) |
| MC6 | a RELATIVE entry skipped whole again | T12 (vii) |
| MC7 | a RELATIVE refusal left unchecked | T12 (vii) |
| MC8 | a RELATIVE leg named through the FIELD mirror | T12 (vii) |
| MC9 | the leg's shape inverted | T12 (i), (ii), (iii), (vi), (vii), (xi), (xii), `cookbook-09b` |
| MC10 | the closing `turnTo` never added | T12 (xi) |
| MC11 | the in-place rule dropped | T12 (ix) |
| MC12 | the in-place rule widened to 1 in | T12 (ix) |
| MC13 | the run's start not recorded | T12 (i), (ii), (ix), (xi) |
| MC14 | the sentence's time branch swapped | T12 (i), (iii), (vi), (vii), (xii), `cookbook-09b` |
| MD1 | DOC: guide 17's quote off by one number | `cookbook-09b` |

T11 and `autons (1d)` stay green under every mutation: none of their routes has a half-turn leg, and (1d) has 0
entries (item 12).

#### Prose review (pace rule 4) — RUN and READ before this entry (~12:50)

Over `docs/` (not `docs/internal/`), `README.md`, `test/README.md` and `include/`:
- `grep -rn -i 'pair check'` hits:
  - guide 17 :171 — updated;
  - guide 14 :533 — the gap at the planned timing: current;
  - hardware-assumptions :551, :2333, :2338 — HA-260: current;
  - the changelog — this entry;
  - `docs/api/chassis.md` :1061 — this round's clause, regenerated;
  - `test/README.md` :156 — updated.
- `grep -rn -i 'RELATIVE mirrored\|relative.*skipped'` hits:
  - guide 17 :250 — updated ("its gap is skipped");
  - guide 11 :254 and `match_runner.hpp` :59 / `docs/api/match_runner.md` :343 — the re-seat line's mode word:
    unrelated;
  - `test/README.md` :164 — updated.
- `grep -rn -i 'half turn\|half-turn\|turn first, then\|14\.2 in\|5\.7 in\|14 in on\|about 7 in\|turns around'`
  hits:
  - updated here: guide 17 :157-172 and :252-267, cookbook 09 :123-140, the changelog :19-39;
  - current, NOT touched (item 7, for the coordinator): guide 14 :525-530, guide 11 :261-269;
  - unrelated: guide 02 :198; cookbook 05 :154, :190 ("about 7 in from the wall"); guide 09 :421 (TANK1's
    unsteered leg — current); the changelog's MIRROR1-A entry :132, :148 (the record).
- Hits in `docs/api/*.md` for 'half turn\|pair check': `chassis.md` :1061 (this round) and `override_mirror.md`
  :84 (the head-to-head half turn: unrelated).
- New names (`HalfTurnLeg`, `halfTurnFix`, `halfTurnSentence`, `kInPlaceIn`, `Track::start`) live in `test/`
  only.

#### Resumption — 2026-11-16 22:50 to 2026-11-17 00:20 EST (read from `date`)

The round above was STOPPED by the coordinator's pause inside the src gate's self-test (its driver
`fix1/gates/gates.sh` started it at 13:13; nothing ran after it). Its entry was written before that, so its
gates section was never filled. Scratch for this part: `fix1/gates2/`, `fix1/mut2/`.

- **A self-test plant was LEFT in `src/` again.** `git diff -- src/` showed `src/programs/bench_r3a.cpp` with two
  appended lines, `void ser1SelfTestCard(std::FILE* f) ...`: the src gate's self-test case 38
  (`tools/src_build_gate.py` :3972-3974), written at 13:53:18 (the file's time) and never removed — the run was
  killed inside it. It has no `(self-test plant` marker, like case 17's at the first resumption. **Restored** with
  `git checkout -- src/programs/bench_r3a.cpp`, its time bumped. After: `git diff -- src Makefile` EMPTY, the marker
  grep outside this log 0, no untracked file under src, include, test or the Makefile.
- **Re-checked against the tree:** the 15 items' code, tests and pages are there as written above, and the
  round's pre-mutation copies (`fix1/orig/`) are byte-identical to the tree's `mirrored_pair_check.hpp`,
  `chassis.hpp` and guide 17.
- **A gap in the round's own record, closed.** The first mutation pass (`fix1/mut/results-124932.json`) ran
  before T12 (xii)'s case (d) was added (`mirrored_pair_test.cpp`'s time, 12:49:53, is after it; 229 assertions
  then, 230 now). Only MB4 was re-run after (d). A RED verdict cannot turn green when a CHECK is added, but the
  verdicts were not measured on the final tree. **So every mutation was run again on the final tree**
  (`fix1/mut2/`, the same driver and spec; MD1's dependent objects set to none, since a page is read at run time).
  **25 of 25 RED**, each failing case read from the ERROR blocks, every set the same as the table above, MB4
  through (xii). Every restored file is byte-identical to its pre-run copy (`fix1/mut2/pre/`).
- **One wording fix (docs only).** Guide 17 item 4 and cookbook 09 said "A step that ends facing back gets
  `faceAway`, then `reverseTo`". Both refused shapes end facing back from where they started. The one that backs up
  ends facing **the way it came**, so both now say that. Prose review: `grep -rn -i 'facing back\|ends facing'` over
  `docs/` (not `docs/internal/`), `README.md` and `test/README.md` → these two lines (changed), and guide 17 :168
  ("facing back (180° from robot one's start)", MIRROR1-A's leg — exact, unchanged).

#### Every gate on the final tree (the resumption; NOT committed)

- **src gate** (`fix1/gates2/srcgate.sh` → `srcgate.out`; three other worktrees' self-tests ran beside it):
  - `check` (22:52–23:09): `src build gate: PASS (all 14 builds link or, robot one's, refuse naming exactly their
    owed facts (4 refused; ...))`.
  - `self-test` (23:09–00:03): **rc 1. Every case passes but 53 and 55**, the same five failure lines as the first
    resumption's (`diff` of the two logs' failure lines: none). Not this run's: both expect robot one's Bench Tests
    to refuse owed Stage A facts, and it links since ROB1-C2. **On main, `531c3fd` (RPV1-S1) has already changed
    the two cases** ("src gate self-test cases 53/55 updated now that Bench Tests links"); this branch's base,
    `3c29b73`, is before it. After the merge onto main they are main's cases.
  - After it: `git diff -- src Makefile` EMPTY, the marker grep outside this log 0, nothing untracked.
- **The rest** (`fix1/gates2/gates.sh` → `gates.out`, 00:12–00:20): the binary removed, the briefing regenerated
  as unbuilt, a full build (0 warnings), the briefing regenerated.
  - `suite run: green — 3775 cases, 3882630 assertions`;
  - the pair cases (`MIRROR1-B*, cookbook-09*, MIRROR1 T11*, autons (1d)*`) with `-s`: 23 cases, 230 assertions,
    green;
  - `ctest -R compile_refusal`: 295 of 295;
  - the ARM header gate (258 headers in one TU): rc 0; the CI greps: PROS-free PASS, core sim-free PASS;
  - `api_doc_tool`: self-test OK; check-coverage (6439 entities, all documented); check-fresh rc 0;
    check-examples (926 quoted lines, all verbatim); check-removability;
  - `briefing_status.py` self-test OK, check `generated block current`; `doc_staleness_audit.py` self-test OK,
    audit clean;
  - the words: the forbidden name in added lines 0; non-ASCII in the check's added code lines 0.
- **The tree:** `version.hpp` untouched; `include/` changed in two comments only (item 7). Changed: the 4 test
  files, `test/README.md`, the 2 library comments and their 2 API pages, the changelog, cookbook 09, guide 17, the
  briefing's generated block and this log.

#### Open for the coordinator / the team lead (from this round, in one place)

1. The refusal sentence now names the steps for the leg's shape ("turn first, then drive: face(x, y), then
   driveTo(x, y)" or "turn first, then back up: faceAway(x, y), then reverseTo(x, y)", and a `turnTo` after when
   the end heading is neither). It is no longer the fixed "write it as two steps: turn first, then drive." (item 1).
2. The brief's "a tank leg at 179°" sits exactly on the `[MIR]` window's edge: 179.0 is refused, 181.0 passes.
   Neither is claimed (item 3, T3's policy).
3. A `moveTo` that drives under 0.5 in is not refused: it only turns (item 4). This follows the ruling's words, "a
   step that drives".
4. The library's `[MIR]` leg text, "turn first, in two 90 deg turns, where it matters", no longer fits a tank leg
   (item 5; a library string that T3 pins).
5. Guide 11 :266-268 and guide 14 :530 each want one clause (item 7; DOCS3-N1's pages).
6. The status docs, the brief of record and this log's marker (item 15).
7. `autons (1d)` checks 0 mirrored entries until ROB1-BT (item 12).

## MIRROR1-B — the coordinator's merge *(2026-11-17)*

- **Squash-merged from `mirror1-b`** (transport `a71f16d`) onto the integrate worktree after the Calibrate fix and AF1S-A2.
  Conflicts: the changelog (MIRROR1-B's entry at the top of `## API 2.9`; test-only, no API change), the briefing
  (regenerated), `cookbook_examples_test.cpp`'s mapping comment and includes (both runs' lines kept).
- **The team lead's D6 ruling, implemented:** the pair check refuses a tank leg that turns within 1 deg of a half turn in
  one step, naming it, "turn first, then drive". **Open for the team lead (delta 4):** a RELATIVE mirrored entry is still
  skipped whole by the pair check (MIRROR1-A's design), so a half-turn leg inside one is not refused.
- Gated together with the Calibrate fix and AF1S-A2 (one gate run on the final tree), then the coordinator's mutations.
- **Gates on the final tree** (with the Calibrate fix and AF1S-A2): suite green (**3,819 cases**); src gate self-test OK and check PASS (14 builds); ARM; doc gates. **The coordinator's mutation MF-2** (every robot treated as a non-tank — D6 undone): **RED, 10 cases**. MIRROR1-B LANDED.
