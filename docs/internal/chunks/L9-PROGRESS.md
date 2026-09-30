# L9 — live progress log

> Appended in real time. The brief: [`L9-rest-reverse-driving-and-one-line-boot.md`](L9-rest-reverse-driving-and-one-line-boot.md)
> (the team lead's answers at its top: every recommendation accepted; its Re-check amendments override
> its body).

## Part A1 — `Routine::reverseTo` and `Routine::faceAway` (2026-09-29)

**Executor:** Claude Opus 5.5 (1M context), a subagent of the coordinator, in the git worktree
`.claude/worktrees/l9-a1` (branch `l9-a1`, from `9239538`). Not committed (the coordinator verifies and
commits). Scope: Part A1 only (D8 (a)) — the two steps, the version, the pins, tests A1–A10 and A12,
guide 09, the changelog, the roadmap, HA-167, the conditional VexBuilder sentence and the internal
documents. NOT A2 (cookbook-04c) and NOT Part B (the boot). Probes and scratch files:
`.claude/coordinator/scratch/l9-a1/` (outside the repository).

### Step 0 — log created, standing rules read

- `docs/internal/RESUMING.md` read in full (the chunk loop, the standards, the pace rules 1–8).
- The brief read in full, lines 1–765.

### Step 1 — baseline, and the brief's citations re-checked against the tree *(`date` 03:47 EDT)*

- **Baseline host suite** at `9239538` + this log (briefing regenerated first; fresh
  `cmake -S test -B build/test`, full build `Linking CXX executable shulib_tests` seen, rc 0):
  `test cases: 1725 | 1725 passed | 0 failed | 3 skipped`, `assertions: 2648032 | 2648032 passed`
  (= TANK1's merged count; nothing landed in a test since). Output kept in the scratch dir
  (`baseline_build.txt`).
- **Every file:line the brief cites, re-opened on the tree** (the brief was written at `a01e3a2`;
  SRC1, T4 Part 1a, TANK1 Part A and the 2026-09-28 plan docs landed since). Deltas and how each is
  handled:
  1. **API version — 2.4, not 2.3.** `version.hpp` is at `kApiMinor = 3` / `"2.3"` (TANK1's note
     ~:75-91). L9 Part A1 takes 2.4 (build-order's L9 entry and the handoff say so). Every "2.3" in
     the brief's §1 item 4/5, A9 and §3 becomes "2.4"; the `## API 2.3` changelog heading already
     exists (TANK1's), so L9 adds `## API 2.4` ABOVE it; A9's floor is `kApiMinor >= 4`; mutation 8
     is "left at 3 / "2.3"", mutation 9 is "4 with "2.3"".
  2. **`routine.hpp` line numbers moved** (TANK1 grew two doc comments): the F10 banner is still
     ~:6-20; the field-vocabulary paragraph is ~:63-77 (TANK1 added the "Since TANK1 (API 2.3)
     face() is also what makes a tank's driveTo leg steerable" sentence); `driveTo` ~:231-246;
     `face` ~:260-270; `bearingTo` ~:447-460. The shapes the brief relies on (`skipIfStopped` then
     `runMotion`, two name literals per field step) are unchanged. The field-vocabulary paragraph's
     TANK1 sentence is kept and extended to the two new steps.
  3. **`version.hpp`**: H1's block ~:38-41 unchanged (not touched); the notes are ~:52-91 now (1, 2,
     and TANK1's 3); the Q6 sentence is ~:70-74, unchanged and still open — the 4 note must not
     resolve it.
  4. **`path_runner.hpp`**: ~:20-27 (bearing duplicated, "FROZEN — F10's pins fail the build if it
     is touched"), ~:187-188 (DriveTo doc), ~:236-245 / ~:241 (the `+ π`) — all exactly as cited.
  5. **`routine_signature_pin_test.cpp`**: G1 Part 2c-final HAS landed (its lines are ~:385-388,
     the session-header sentence); the "kApiMinor is deliberately unpinned" comment A9 rewords is now
     ~:390-391 and the TEST_CASE ~:392-398. F10DriveTo ~:100-105 as cited. D4's shared-file
     concern is moot (2c-final is merged; nothing else in flight names this file).
  6. **`f8_signature_pin_test.cpp`** ~:473-476 / ~:488-489: as cited.
  7. **`chassis_recipe_test.cpp`** ~:186-206, ~:358-394, ~:782-806 (now ~:788-804), ~:811-830: as
     cited.
  8. **`motion_test_rig.hpp`** ~:47-51 and ~:148-170: as cited; 36 test files include it (brief: 35).
     `RecordingPacer` stays in `path_runner_test.cpp`; the new test file gets its own local pacer
     (the brief's first option) so the rig's 36 includers are not touched.
  9. **`path_runner_test.cpp`** ~:80-134 (`RecordingPacer`, `requireIdentical`), ~:197-233
     (`tankAcceptanceRoute`), ~:714-750 (runner 10): as cited. `startPose()` is (−24, −12) at 90°.
  10. **`move_to_pose.hpp` ~:196-220** (the brief's D2 reasoning about a tank's projection): the file
      is TANK1's now; the claim still holds (the pipeline clamps body vy on a tank; TANK1 changes
      only ω), but the tank path now has an ENGAGE WINDOW decided once at the first live tick
      (`tankLegEngages`, ~:216-226, `|errH| <= maxSteer` = 15°) and ONE Warn for a leg outside it
      (`warnNotSteered`, ~:474-487). **This changes A1–A4's expectations** — see Step 2.
  11. **`angle.hpp`** ~:9-12 (antipode → +π): as cited.
  12. **`test/CMakeLists.txt` ~:77**: the glob is `*_test.cpp` with `CONFIGURE_DEPENDS` — a new
      `test/routine_reverse_test.cpp` is picked up without editing CMake.
  13. **`api_reference_fidelity_test.cpp`** ~:141-197: as cited (D3 routine page ~:141-172, the
      index ~:183-213).
  14. **The stale-binary set**: `chassis/routine.hpp` is included by 15 test files (brief: 14 —
      `motion_tank_steer_test.cpp` is new since) and by the three `competition/` headers.
  15. **Docs**: guide 09's stability notice is ~:10-25 (brief ~:10-20); the steps table ~:96-109;
      the sugar paragraph ~:111-122 (TANK1 added the steerable-leg sentences); "Tank drives" is
      ~:199-236 (brief ~:194-222). The roadmap F10 row is ~:144 ("eleven steps", "37 pins"); F8 ~:142
      ("API **2.2**, carrying the G2 additive note"). The register: T3's additions ~:1659, COMP-a's
      ~:1942, TANK1's ~:1994-2079 (Group R3 ends ~:2081); index rows HA-161 ~:271 then HA-179 ~:272;
      the status sentence chain ends with TANK1's ~:79-84. VexBuilder §4.1.1 is ~:240-255 (the `"drive"`
      row ~:249). Handoff L9 bullet ~:489-493; stale-binary recipe ~:208-216; command set ~:218-233.
  16. **HA-167's sources**: `sim/motor_model.hpp` now models kA (motor lag, R5a-era) — V = kS·sign(v) +
      kV·v + kA·a, ~:20-28 — still odd-symmetric and still no backlash term, so HA-167's reasoning
      holds; re-cited at the new lines. `drivetrain_calibrator.hpp`'s push/BACK-first lines are
      ~:27-35 and ~:50-58.
  17. **SRC1**: no `src/` path is involved in Part A1. T4 Part 1a: nothing in Part A1 touches
      `api_doc_tool.py` or `src/team/`.

### Step 2 — the library code, then the steps MEASURED on the built code

- **Code** (`include/shulib/chassis/routine.hpp`): `reverseTo(x, y, options)` placed right after
  `driveTo`, `faceAway(x, y, options)` right after `face`, each in its twin's shape —
  `skipIfStopped("<name>")` first, then `runMotion("<name>", <one Chassis call>)`, so two name literals
  per step — and each computing ONE argument, `bearingTo(x, y) + math::Angle::radians(math::Angle::kPi)`
  (the `+ math::Angle::radians(math::Angle::kPi)` is `path_runner.hpp` ~:241's, character for
  character); `bearingTo` shared, its precondition message now names all four field steps. The F10
  banner says THIRTEEN steps (the two appended at L9, API 2.4, and pinned); the field-vocabulary
  paragraph covers four steps, keeps the hand-written-idiom sentence (now with its `+ π` form) and
  TANK1's steerable-leg sentence, and adds that faceAway() does the same for a reverseTo leg.
  `version.hpp`: `kApiMinor = 4`, `"2.4"`, a `4 =` note in the 1/2/3 notes' style; the Q6 sentence
  untouched and still open; ~:38-41 untouched. `path_runner.hpp`: comments only (~:24-29 — F10 grows
  only additively, which is why `bearingTo` is still duplicated, and the `+ π` is reverseTo's; ~:187-191
  — `reverse` is exactly `Routine::reverseTo`, with the tank's authored `TurnTo`, and HA-167).
- **Measured on the built code** (probe `scratch/l9-a1/measure.cpp`, `a4.cpp`, built by `b.sh` — TANK1's
  flags: g++ -std=c++20 -O2, host test plant, this worktree's headers; output `measure.txt`, `a4.txt`).
  Geometry: the brief's rule — start (−24, −12) at 90° (path_runner_test's `startPose()`), point (0, 24),
  bearing 0.98279372324732905 rad; `b + π` = −2.1587989303424644, `b − π` = −2.158798930342464 (the
  brief's probe re-run here: identical output, DIFFER at this geometry, SAME at the others).
  - **A1 (tank, faceAway(4 s, ≤4 rad/s) then reverseTo(8 s, ≤40 in/s))**: Settled both; 3.54 s; truth
    (−0.128, 23.808), 0.231 in from the point, heading 0.00002 rad from b + π. The hand-written oracle
    (`turnTo(Angle::radians(atan2 + kPi))`, then `moveTo` at the re-read bearing + kPi) is
    **bit-identical** (truth per tick by memcmp, every volt, tick count). **TANK1:** the faceAway leg ends
    0.0131 rad (0.75°) off b + π, so the reverseTo leg starts inside the 15° window — no "not steered"
    Warn; it is steered.
  - **A2 (the reverseTo leg, 208 ticks)**: heading within 0.0131 rad (0.75°) of b + π on every tick;
    backward displacement along the leg's start heading 43.025 in of a 43.267 in leg; forward movement
    along the instantaneous heading 0.0000 in; at most 0.064 in off the line.
  - **The reversed leg IS steered (TANK1 adaptation, new)**: one side weak, faceAway then reverseTo:
    L ×0.95 3.41 s (0.30 in, xt −0.018), L ×0.91 3.62 s (0.36 in, xt −0.048), R ×0.95 3.40 s (xt 0.033),
    R ×0.91 3.59 s (0.37 in, xt 0.055) — all Settled, no Warn. With `maxSteer = 0` (TANK1's off switch)
    every one TIMES OUT at the 8 s budget: xt +1.889 / +3.319 (L ×0.95/×0.91), −1.430 / −2.755 (R).
  - **A4 (tank FACING the point, reverseTo alone, 8 s)**: exactly ONE Warn, `tank moveTo not steered: it
    starts 180.0 deg off its target heading (steers within 15.0 deg) -- turn first: face()` (the leg is
    not steered: 180° > 15° *[corrected in the review round, R3: this start is NOT an exact antipode in
    doubles — `errorTo` returns π − 4.4e-16 by the wrap's rounding; the +π rule is not reached]*); then the
    RTN stop line. The robot
    swings round through 180° (unwrapped +3.1416 rad) while it drives — within 0.02 rad of b + π by
    1.43 s — covers the whole 43.27 in along the line (path 49.3 in), stops moving at 3.39 s **7.807 in
    to the side of the point** with its heading exactly b + π, and sits there: TimedOut at 8.01 s; the
    same end at a 20 s budget. **Landmine 7 (the antipode rule):** the OUTCOME does not depend on it,
    only the SIDE does — started 0.001 rad either side of facing, it turns CCW / CW (+3.1406 / −3.1406)
    and ends +7.808 / −7.808 in off the line (0.05 rad: ±7.849); at the exact antipode `errorTo`'s +π
    turns it CCW, to the +7.807 side *[corrected, R3: "exactly facing" turns CCW by ROUNDING, not by the
    rule — a mutant that disables the rule leaves it CCW; A4d is the start where the rule decides. And a
    positive offset in this cross product is the RIGHT of the line of travel]*. So a reversed leg on a tank does NOT
    settle without a preceding turn *[corrected, R1: when it starts FACING the point — one that starts
    with its back to the point is steered and settles]* — the brief's condition for the VexBuilder
    sentence (§3) is met (that sentence is conditioned on a leg that starts facing the point).
  - **A5 (X-drive, reverseTo alone from 90°)**: Settled 2.07 s, 0.231 in, heading 0.0016 rad from
    b + π, no Warn at all.
  - **A12**: faceAway uncapped 1.35 s, ≤2 rad/s 1.93 s, ≤1 rad/s 3.05 s (×2.26); the reverseTo leg
    uncapped 1.86 s, ≤20 in/s 2.94 s, ≤15 in/s 3.57 s (×1.92).
  - **A6**: faceAway given 0.1 s — stopped at step 2, `faceAway`, TimedOut, `routine 'starved' STOPPED at
    step 2 (faceAway): motion TIMEOUT`; reverseTo given 0.3 s — stopped at step 3, `reverseTo`, TimedOut.
- **What TANK1 changed in the brief's A1–A4 (logged adaptations):** A1/A3 are bit-identity oracles
  between two arms that make the SAME facade calls, so TANK1 cannot move their verdict; their anchors
  hold as measured. A1 gains two checks the brief did not have: the reverse leg starts inside the
  window (the estimate's heading after faceAway within 0.05 rad of b + π, measured 0.0131) and no
  "not steered" Warn. A NEW case (A1b) pins that the reversed leg IS steered: with either side 9 %
  weak it settles, and with `maxSteer = 0` it times out — the only observable that separates "steered"
  from "not steered" on a clean plant, where the bend is ~0. A4 is no longer "MEASURE what happens" but
  TANK1's documented un-faced case: one Warn naming 180.0 deg, not steered, then (measured) a timeout
  about 7.8 in beside the point — pinned as the docs will state it. A2's band is unchanged (the steering
  bends the heading SETPOINT by at most the cap and only when the leg is off its line; on a clean plant
  the measured band is 0.75°, set by faceAway's settle tolerance, not by TANK1).

### Step 3 — the tests

- **`test/routine_reverse_test.cpp` (NEW, 9 cases)** — every case on the geometry rule (start (−24, −12)
  at 90°, point (0, 24)), graded on the plant's truth; a LOCAL recording pacer (`ReverseRecorder`, in
  `RecordingPacer`'s shape: sample after the tick, before the plant steps, hard-capped) and a local
  one-side-weak `DegradationModel`, so `motion_test_rig.hpp` (36 includers) is untouched.
  - **A8** (compile-time): `reverseTo`/`faceAway` refuse `(double, double)` and `(int, int)`, accept
    `(Length, Length)` and `(Length, Length, MotionOptions)`; `faceAway(Angle)` does not compile (an angle
    is not a point) — the last one is an addition to the brief's list.
  - **A1** (the oracle, tank): recipe `faceAway(4 s, ≤4 rad/s).reverseTo(8 s, ≤40 in/s)` vs a fresh hand
    arm spelled `Angle::radians(atan2 + kPi)` (independent of the library's `Angle + Angle` *[corrected,
    R6: NOT independent in its arithmetic — the two spellings do the same floating-point work; the arm is
    independent because it reads the pose and does the trigonometry itself]*), per-tick
    truth memcmp + every volt + clock; anchors < 1.0 in / < 0.10 rad / +x / +y; plus the TANK1 checks
    (leg starts < 0.05 rad off b + π; no "not steered" Warn; no fault).
  - **A1b** (NEW — the TANK1 adaptation): after faceAway the reversed leg is steered — each side ×0.91 weak
    settles (< 1.0 in, < 0.10 rad, no Warn); with `maxSteer = 0` it stops at `reverseTo`, TimedOut,
    > 1.5 in off (measured 3.32 / 2.76).
  - **A2**: per-tick band < 10° (measured 0.74°), backward ≥ leg − 1 in (43.029 of 43.267), forward
    < 0.1 in (measured 0) — the forward bound tightened from the brief's 1 in on the measurement.
  - **A4** (re-derived against TANK1): three starts (exactly facing, ±0.001 rad): MotionFailed/TimedOut
    at `reverseTo`, exactly one "not steered" Warn naming 180.0 (179.9 a hair off) and "(steers within
    15.0 deg) -- turn first"; ends turned round (< 0.10 rad of b + π) after swinging > 3.0 rad, 7.0–8.5 in
    from the point (measured 7.807/7.808), on the side it turned toward; the exact antipode turns CCW
    (errorTo's +π — landmine 7, pinned); drive safed. *[Corrected, R3: a CCW swing ends on the RIGHT of the
    line of travel, not "the side it turned toward"; and the exactly-facing start is CCW by the wrap's
    rounding, not the rule — landmine 7 was NOT pinned here; A4d pins it.]*
  - **A5** (X-drive): reverseTo alone from 90° (2.55 rad from b + π) arrives < 1.0 in, < 0.10 rad, no Warn.
  - **A6** ×2: starved reverseTo (0.3 s) — stoppedAt 3, `reverseTo`, MotionFailed/TimedOut, the Warn line
    `'starved-reverse' STOPPED at step 3 (reverseTo): motion TIMEOUT`, then faceAway/reverseTo/driveTo
    SKIPPED with one Info each naming them, no motion started, the robot unmoved, no time passed, drive
    safed; starved faceAway (0.1 s) — stoppedAt 2, `faceAway`, the Warn naming `(faceAway)`, the reverseTo
    skipped and named, one motion only.
  - **A7**: at the robot's own position, at a NaN point, with a NaN or negative timeout: each step throws
    `PreconditionError`; counters, motions and clock untouched; the chain still backs in afterwards.
  - **A12**: faceAway ≤1 rad/s takes > 1.5× its uncapped time (measured 3.05 / 1.35 s); the reverseTo leg
    ≤15 in/s > 1.5× (3.57 / 1.86 s).
- **A3 in `test/path_runner_test.cpp`** (beside runner 10, reusing `RecordingPacer` / `requireIdentical`):
  `L9 A3` — tank: route [Start, TurnTo(atan2 + kPi computed at authoring), DriveTo reverse] vs recipe
  [startAt, turnTo(same angle), reverseTo]; X-drive: [Start, DriveTo reverse] vs [startAt, reverseTo];
  bit-identical (memcmp truth, exact volts, ticks) plus arrival anchors. (The brief says `"G2 runner 10"`'s
  neighbourhood; named `L9 A3` so a transcript says whose it is.)
- **A9 in `test/routine_signature_pin_test.cpp`** — ADDED only: `F10ReverseTo`, `F10FaceAway`
  (member-pointer casts), two `SHULIB_F10_NOT_NOEXCEPT`, `F10TerseSpellingsReversed` (**delta:** the brief
  says "both terse spellings in `F10TerseSpellings`"; that would edit an existing pin's concept, and
  landmine 1 / A9 say "no existing PIN is edited" — so they are a NEW concept, pinned on their own; the
  old pin is byte-identical), `F10ConstReverseTo` / `F10ConstFaceAway` (the non-const pin extended by two
  NEW pins, same reason), and `CHECK(kApiMinor >= 4)` with F8's wording ("F10 grew additively at 2.4 …
  past TANK1's 3"). The one existing-TEXT edit: the "kApiMinor is deliberately unpinned" comment (~:390)
  now says it is not pinned to a value and has a floor. 2c-final's lines (~:385-388) untouched. Pins: 38 +
  12 = 50 before → 43 + 14 = 57 after (`briefing_status.py`'s counter).
- **A10 in `test/api_reference_fidelity_test.cpp`**: `docs/api/routine.md` renders both signatures;
  `Routine::reverseTo` / `Routine::faceAway` in the index list.
- **Suite** (docs/api + briefing regenerated, fresh configure for the new file, full build with the doc
  gates, `Linking CXX executable shulib_tests`): **1735 | 1735 passed | 0 failed | 3 skipped, 2 649 676
  assertions** (baseline 1725 / 2 648 032: +10 cases — 9 new file + A3; A9/A10 extend existing cases).

### Step 4 — mutations, FIRST pass THROWN OUT (a stale object of my own making)

- Driver `scratch/l9-a1/mutate.py` (TANK1's `mutate3.py` mechanics: backup, exact one-match replacement,
  touch every TU whose depfile names the mutated file, `--target shulib_tests/fast`, REQUIRE `Linking
  CXX executable shulib_tests` or — for the must-not-compile mutations — record the compiler's errors,
  whole suite, restore + sha256 + mtime bump). 28 mutations.
- **The pass is NOT evidence** (kept as `scratch/l9-a1/mut_pass1_contaminated/`): M8 (the version left at
  3) went red on `L9 A1` — which cannot see the version. Cause, found: `test/routine_reverse_test.cpp` was
  added AFTER the last full build, and CMake's Makefile generator consolidates a TU's depfile into
  `compiler_depend.make` only in a NON-`/fast` build; every build since was `/fast`, so make did not know
  that object depends on `routine.hpp`. Restoring `routine.hpp` (mtime bumped) therefore did not rebuild
  it, and the next mutation of a DIFFERENT file (M8, M9 on `version.hpp`; M12 on `path_runner.hpp`) linked
  the PREVIOUS mutant's inline `Routine` code from that object (M7b's `- π` → A1 red). HANDOFF's
  stale-binary recipe covers a header the depfiles know; it does not cover a brand-new TU whose deps make
  has never consolidated. The routine.hpp mutations of pass 1 were each rebuilt from their own mutant
  (every one touches all 16 TUs, this file included), but the pass is re-run whole rather than
  cherry-picked.
- **Fix** (`mutate2.py`): (1) a full, non-`/fast` build first — confirmed afterwards that
  `compiler_depend.make` lists `routine_reverse_test.cpp.o: … include/shulib/chassis/routine.hpp`; (2)
  after every restore, re-touch every TU that was touched for the mutation, so the next build recompiles
  each from the RESTORED source whatever make believes. Baseline before pass 2: 1735 / 1735 green.

### Step 5 — mutations, SECOND pass (the evidence)

- `scratch/l9-a1/mutate2.py`, output `scratch/l9-a1/mut2/<id>/` (`result.txt`, `build.txt`, `suite.txt`),
  summary `scratch/l9-a1/mut2_all.txt`. **25 mutations: 20 linked FRESH (`Linking CXX executable
  shulib_tests` in every one's `build.txt`) and went RED on the whole suite; the 5 that must not compile
  failed the build as required; 25 of 25 restored sha256-ok, mtime bumped, their TUs re-touched.** No
  existing (pre-L9) test went red on any L9 mutation except where the brief said it must (M12 → G2
  runner 10; M9 → the F6/F8 version twins).

| # | mutation | observed |
|---|---|---|
| M1 | `reverseTo` drops the `+ π` | RED: A1, A1b, A2, A3, A4, A5, A7, A12 |
| M2 | `faceAway` drops the `+ π` | RED: A1, A1b, A2, A7, A12 |
| M3a | `reverseTo`'s bearing from the field ORIGIN | RED: A1, A1b, A2, A3, A4, A5, A7, A12 |
| M3b | `faceAway`'s bearing from the field ORIGIN | RED: A1, A1b, A2, A7, A12 |
| M4 | `reverseTo`'s stop AND skip names `"driveTo"` | RED: A1b, A4, A6 ×2 |
| M4a | … the SKIP name only | RED: A6 ×2 |
| M4b | … the STOP name only | RED: A1b, A4, A6 (reverseTo) |
| M5 | `skipIfStopped` removed from `reverseTo` | RED: A6 ×2 |
| M5b | `skipIfStopped` removed from `faceAway` | RED: A6 (reverseTo — its skipped faceAway turned) |
| M6 | `faceAway` = an in-place `moveTo` | RED: A1, A1b |
| M7a | `reverseTo`: `bearing - math::Angle::radians(kPi)` | RED: A1, and A3's **X-drive** subcase (the tank subcase stays green: its reverse bearing is read at the pose the authored turn left, and there the `- π` target either equals the `+ π` one bit for bit or its 1-ulp difference never reaches a volt — not separated further) |
| M7b | `faceAway`: `- π` | RED: **A1 only** (the 1-ulp turn target changes the turn's volts and so its truth) |
| M8 | `kApiMinor` left at 3 / `"2.3"` | RED: F10 pin (the new `>= 4` floor) |
| M9 | `kApiMinor` 4 with `"2.3"` | RED: F6, F8 and F10 pins (the token check) |
| M10a | `reverseTo` `noexcept` | BUILD FAILS: `routine_signature_pin_test.cpp:211: static assertion failed: F10 FREEZE VIOLATION: Routine::reverseTo GAINED noexcept. A change to a frozen signature's noexcept shape is BREAKING IN EITHER DIRECTION (include/shulib/version.hpp): it changes the member's type, and it turns a precondition throw into std::terminate. Accidental? Revert. Intended? Bump kApiMajor, write the migration note, update Freeze Register row F10, THEN this pin.` |
| M10b | `faceAway` `noexcept` | BUILD FAILS: the same message naming `Routine::faceAway` |
| M11a | a `reverseTo(double, double)` overload | BUILD FAILS on A8's negative pins (`routine_reverse_test.cpp:204-205: static assertion failed`) |
| M11b | a `faceAway(double, double)` overload | BUILD FAILS on A8's negative pins (`:208-209`) |
| M11c | `reverseTo`'s options default removed | BUILD FAILS: A8 (`:206`) and `routine_signature_pin_test.cpp:380: F10 FREEZE VIOLATION: the frozen signature of the defaulted spellings reverseTo(x, y) and faceAway(x, y) changed …` |
| M12 | `path_runner.hpp`: DriveTo's `+ π` dropped | RED: G2 runner 10 AND A3's **tank** subcase (`report.ok()` — the un-turned reverse leg now times out) — so A3 also guards F8's `reverse` on a tank |
| M13 | `faceAway`'s runMotion name `"face"` | RED: A6 (starved faceAway) only |
| M13b | `faceAway`'s skip name `"face"` | RED: A6 (starved reverseTo — its skipped step 4) |
| M14a | `reverseTo` forwards only `.timeout` | RED: A1 (its capped options), A12 |
| M14b | `faceAway` forwards only `.timeout` | RED: A1, A12 |
| M15 | (added) `reverseTo` drives FORWARDS facing the point, then turns round at the end | RED: A1, A1b, A2, A3, A4, A7, A12 |

- **Mutation 7 (the brief's probe), answered:** at the geometry rule's bearing the `- π` mutant is NOT
  equivalent in either step — A1 sees it in `faceAway` (a 1-ulp turn target) and in `reverseTo` (tank);
  A3 sees it on the X-drive, where the bearing is read at the exact seeded pose; A3's tank subcase does
  not (its reverse bearing is read at the post-turn pose; see the table).
- After the last restore: full build WITH the doc gates (`Linking CXX executable shulib_tests`), suite
  **1735 | 1735 passed | 0 failed | 3 skipped, 2 649 676 assertions**.

### Step 6 — documents, and the prose review (pace rule 4), grepped and read BEFORE this entry

- **Documents written (Part A1):** `docs/guide/09-the-recipe-api.md` (stability notice: the additive path
  used twice, 2.4's two steps; two rows in the steps table; the sugar paragraph covers four steps; a
  "Backing into a point" paragraph in "Tank drives" with A4's measured outcome and HA-167);
  `docs/changelog.md` (a NEW `## API 2.4` above TANK1's `## API 2.3`, one entry: ADDED, the tank idiom,
  "What you must do: nothing", host test plant only); `docs/roadmap.md` (F10 row: thirteen steps, the
  two appended at L9 / 2.4, the pin count recounted — 57 = 43 + 14, "37" stale since F1, said so; F8 row:
  "API 2.2 at the lock"); `docs/hardware-assumptions.md` (HA-167: index row in numeric order, the entry
  under "L9 (2026-09-29) added ONE — HA-167" at the end of Group R3 after TANK1's, one appended status
  sentence); `docs/vexbuilder-integration.md` §4.1.1 (the `"drive"` row: on a tank, author a `"turn"`
  first — A4 showed a reversed leg does not settle without it, the brief's condition); `docs/api/`
  regenerated (`routine.md`, `path_runner.md`, `version.md`, `route.md`, `all-entities.md`,
  `README.md`); internal: `build-order.md` (the L9 entry's status line), `HANDOFF-remaining-work.md`
  (one parenthetical sentence on the L9 reverse bullet), this log; the briefing regenerated last.
- **Greps (brief §3), over `README.md` and `docs/` (generated `docs/api/` and chunk records excluded),
  every hit read:**
  - `driveTo` — guide 04 :104, guide 09, cookbook 01/02/03/04/05 listings and prose, master plan :161
    (LemLib's `driveToPoint`), roadmap, changelog: all still true; none says a recipe cannot back up.
  - `face(` — guides 04/09/10/11/12/14, cookbook 04, changelog (TANK1's Warn quotes): true of forward legs.
    **Two were incomplete for a backwards leg and are fixed:** guide 11 ~:213 (the Warn's "Fix: `face()`
    the point" now adds `faceAway()` for a `reverseTo`) and guide 12 ~:78 ("Face the point first" gains
    the same). Guide 10 ~:348 and guide 14 ~:362 only quote what the log says — true, unchanged.
  - `reverse` / `backwards` / `backward` / `back up` / `back into` / `backing` — vexbuilder §4.1.1 (edited),
    guide 04 :15 ("can drive forward and backward" — true), master plan :708/:741 (the data field — true),
    the rest unrelated (a claw constant, a `/usd/` prefix, HA text about other things).
  - `kApiMinor`, `2.2`, `2.3`, `API 2.3` — roadmap :693 (history); every "since API 2.3" is TANK1's and still
    true; nothing public claims the CURRENT version is 2.3; F8's "API **2.2**" reworded (above).
  - `F10` — guide 09 :11, guide 10 :24, guide 14 :193, guide README :124/:147, diagnostics-plan :188-190,
    changelog history, roadmap history: all true (frozen, additive growth legal).
  - `37` — roadmap F10 row (fixed); roadmap :531 ("37 compile-time pins", the D3 history entry) and
    build-order :335 (history) stay, as the brief says.
  - `eleven steps` / `ELEVEN` / `Twelve` — roadmap :486 ("Twelve step verbs", history, stays); cookbook 01
    :113/:131 count ONE recipe's steps (not F10's) — true.
  - The changed sources' names (`routine.hpp`, `path_runner.hpp`, `version.hpp`, `route.hpp`) — HA source
    lines, PROJECT-BRIEFING :243 ("FROZEN (F10), except then()" — true), MECH-PANEL-requirements :102-103
    (true), build-order history.
  - `include/`: `routine.hpp is FROZEN` — 0 hits left (path_runner's banner reworded); "exactly
    `Routine::driveTo`" — path_runner :191, now with its reverseTo half. **Found while reading, fixed:**
    `config/route.hpp`'s `Waypoint::reverse` doc still said the flag was "advisory data … the library's
    verbs are holonomic and do not read it today" — false since G2 (the runner reads it); rewritten
    (it is now exactly `Routine::reverseTo`, the tank's authored `TurnTo`), and `WaypointMotion::DriveTo`'s
    doc names `reverseTo` for the reversed form. Comment-only; `docs/api/route.md` regenerated.
  - `simulat` in anything added: `git diff | grep '^+' | grep -i simulat` — empty. "host test plant" used.
- **Glossary:** no new term (the two names are API, in the reference). **Links** in every changed document
  resolved; `check-removability` (below) confirms no public link into `docs/internal/`.
- **Deliberately not touched:** the cookbook (Part A2 owns cookbook 04 / its README / `cookbook-04c`);
  guide 04 (its tank bullet is about forward legs and links chapter 9); guide 10 / guide 14 (they quote
  the Warn — true); `motion/move_to_pose.hpp`'s Warn text (see "for the coordinator"); `chassis.hpp`
  (TANK STEERING describes moveTo generally — true for reverse legs; not F10's); the brief (the
  coordinator's record — this log supersedes its version numbers).

### Step 7 — the gates on the finished tree *(`date` 10:58 EDT when this entry was written — Steps 2–6 carry no time: only Step 1's 03:47 and this one are `date` readings; the src build gate alone ran well over ten minutes)*

- **Host suite** (briefing regenerated, full build WITH the doc gates, `Linking CXX executable
  shulib_tests`): `[doctest] test cases: 1735 | 1735 passed | 0 failed | 3 skipped`, `assertions:
  2649676 | 2649676 passed | 0 failed`.
- `python3 tools/src_build_gate.py self-test`: `src build gate self-test: OK (189 detector cases, each
  against a real build)`. `check`: `src build gate: PASS (all 6 builds link; …)` — every build exit 0, 8/8
  TUs, 0 disallowed warnings. `git diff -- src/`: 0 lines.
- **ARM header gate** (RESUMING's command, 192 headers + main, scratch TU): rc 0; `chassis/routine.hpp`,
  `version.hpp`, `runner/path_runner.hpp`, `config/route.hpp` each alone: OK.
- **CI greps** (ci.yml's two): both print nothing.
- **api_doc_tool:** `self-test: OK`; `doc coverage: 2937 public entities across 156 headers, all
  documented` (TANK1's 2 935 + the two steps); `check-fresh` rc 0; `doc example scan: 467 quoted lines,
  all verbatim (5 source files); 3 listings of whole-run files, each one unbroken run`; `removability: no
  public doc references docs/internal/`.
- `doc_staleness_audit.py`: `clean (27 live docs, 0 numeric claims checked)`.
- **Briefing** regenerated after the suite binary existed, then `briefing_status.py check`: `generated
  block current`.
- **Not committed. No `pros` command. No `/dev/ttyACM*`.** Part A2 and Part B not started. No
  `L9-COMPLETED.md`; `roadmap.md`'s "You are here" and `build-order.md`'s `**Next:**` NOT moved (the
  coordinator's reconciliation) — only the L9 entry's status line.

### For the coordinator

1. **TANK1's Warn says `turn first: face()` for a reverseTo leg too.** For a backwards leg the turn it
   needs is `faceAway()` — following the Warn literally (calling `face()`) leaves the leg 180° off again.
   Not changed here: the text is `move_to_pose.hpp` ~:485 (TANK1's; DIAG1 Part B is in that file), it is
   pinned verbatim by TANK1's P5 (`motion_tank_steer_test.cpp` ~:624, ~:668), and quoted in ten documents
   and COMP-t's brief. Documented around it instead (guide 9, guide 11, guide 12, changelog, `reverseTo`'s
   doc). A one-line wording fix (e.g. `turn first: face() (faceAway() to back in)`) is a small follow-up
   for whoever owns the file — the team lead's call.
2. **Shared-document edits** (self-contained blocks, merge by hand if a neighbour moved): `changelog.md`
   (a new `## API 2.4` section ABOVE `## API 2.3`: any chunk landing after this with a "since 2.3" entry
   re-heads it "since 2.4" under it); `hardware-assumptions.md` (HA-167's index row between HA-161 and
   HA-179; its entry after HA-180 in Group R3; ONE status sentence after TANK1's, saying "with it, 159" —
   order-dependent if DIAG1 B / SAFE-SAVE / H1 land register entries first; and TANK1's next sentence
   minimally reworded "HA-162..HA-178 are not in it" → "The rest of HA-162..HA-178 is not in it", since
   HA-167 now is); `roadmap.md` (F10 and F8 rows only); `build-order.md` (L9 entry, one status line — and,
   since the review round, the deviations table's L9 row, R14);
   `HANDOFF-remaining-work.md` (one parenthetical on the L9 reverse bullet); `docs/api/all-entities.md`
   and `docs/api/README.md` (regenerate at merge); the briefing (regenerate at merge).
3. **Files another in-flight chunk may also touch:** `docs/guide/11-reading-the-diagnostics.md` ~:213-215
   and `docs/guide/12-when-things-go-wrong.md` ~:78-79 (tank1b's xt column may edit both; my edits are
   one clause each); `include/shulib/config/route.hpp` (comment-only, ~:166-168 and ~:277-285 — no
   in-flight chunk named it); `version.hpp` ~:92-102 (H1 edits ~:38-41, a different hunk).
4. **The stale-binary trap has a fourth form** (Step 4): a TU added after the last NON-`/fast` build is
   not in make's dependency database, so a restored header does not rebuild it and the next mutation of a
   different file links the previous mutant from it. HANDOFF's recipe could add: "after adding a test file,
   run one full build before any mutation" — not edited here (HANDOFF's stale-binary section is shared).
5. Numbers the brief quoted that were stale: API 2.3 → 2.4 (TANK1); F10's "37 pins" (really 50 before,
   57 after); `routine.hpp`'s includers 15 (brief 14); `motion_test_rig.hpp`'s 36 (brief 35).

## L9-A1 review findings and their disposition

- **Round:** the coordinator's workflow run handed back three reviewers' findings (lenses: correctness, tests,
  docs — 17 in all, overlapping), 2026-09-29; the first `date` reading of the round was 21:38:22 EDT. Same
  builder, same worktree, same rules: nothing committed, no `pros`, no `/dev/ttyACM*`. Everything this round
  writes outside the tree is under `.claude/coordinator/scratch/l9-a1/fix/` (probes, the mutation runs, the
  prose-review greps). The reviewers' own probes (`scratch/l9-a1/review-*/`) were re-read, and every claim
  below that this round relies on was re-measured here, not taken from them.
- **Deduplicated** into fifteen items (R1–R15), each naming the findings it answers. Every behaviour a new
  test or a reworded sentence rests on was measured on the built code first: `fix/probe.cpp` → `fix/probe.txt`
  (this worktree's headers, the host test plant), `fix/a4zero.cpp` → `a4zero.txt`, and the antipode probe
  against a copy of `include/` with the rule disabled → `fix/probe_mut_antipode.txt`.
- **Suite after the test changes** (fresh full build WITH the doc gates, `Linking CXX executable
  shulib_tests`): **1741 | 1741 passed | 0 failed | 3 skipped, 2 649 745 assertions** (before this round:
  1735 / 2 649 676; +6 cases — A2b, A4b, A4c, A4d, A12b, and A3 split in two).
- **Mutations this round** (`fix/mutate3.py`, pass 2's mechanics; output `fix/mut3/<id>/`, summary
  `fix/mut3_all.txt`): 17 run — 13 linked FRESH (`Linking CXX executable shulib_tests` in each `build.txt`)
  and went RED as below; the 4 that must not compile failed the build NAMING F10; 17 of 17 restored sha256-ok,
  mtime bumped, their TUs re-touched. Cited per item.

### R1 (docs major + correctness minor + docs minor, and part of the build-order finding) — the un-faced outcome was stated too generally
- **Re-checked: REAL.** The docs said a `reverseTo` with no `faceAway()` "starts 180° off", "is not steered"
  and "stops almost 8 in beside the point" as a rule. TANK1 decides from `errH` at the leg's first live tick
  (`move_to_pose.hpp` ~:214-226), and the target heading is bearing + π, so the start offset is how far the
  robot's BACK is turned from the point. Measured (`probe.txt` §3, §7, §2): back turned 0° / 10° → Settled
  (0.233 / 0.258 in), no Warn; 20° / 45° / ±90° / 135° / 180° → one Warn naming exactly that number, TimedOut;
  facing the point, the miss grows with the leg — 1.067 / 2.136 / 4.402 / 7.100 / 7.807 / 8.634 in at 6 / 12 /
  24 / 36 / 43.3 / 72 in, every one TimedOut; driving out to the point then `reverseTo` back: Settled 1.85 s,
  0.230 in, no Warn, 0 in nose-first (a 9 % weak side either way: 1.94 s, 0.364 in). The reviewers' numbers
  reproduce.
- **Fixed — the sentences** (each now says what the tests pin, nothing more): `routine.hpp` (banner: the leg
  starts as far off as the back is turned — 180° from a robot facing the point, about 0 after driving straight
  out; `reverseTo`'s doc: the same, plus the tank's timeout "beside the point" with no number, and "a tank that
  just drove straight out from the point … needs no faceAway()"); `docs/api/routine.md` regenerated; the
  changelog's 2.4 entry (a bold rule, then the facing case with the leg lengths — "almost 8 in beside it at the
  end of a 43 in leg, still about an inch at the end of a 6 in one" — and the drive-out case); guide 9's
  "Backing into a point" paragraph (the same, in the guide's words); the build-order status line.
  `vexbuilder-integration.md` §4.1.1 was already conditioned ("a reversed leg that starts facing the point") —
  unchanged. The test file's banner said "A reverseTo with no faceAway() starts 180° off" — reworded.
- **Fixed — the pins** (the brief's rule: pin exactly the claim the docs make):
  - `L9 A4` gains the 6 in leg (TimedOut, one Warn naming 180.0, 0.8–1.3 in off; measured 1.067).
  - `L9 A4b` (new): drive out, then `reverseTo` straight back, clean and with either side 9 % weak — arrives
    < 1.0 in, heading < 0.10 rad of the outbound bearing, no "not steered" Warn (on an eligible tank leg, no
    Warn = TANK1 engaged), forward travel < 0.1 in (measured 0).
  - `L9 A4c` (new): back turned 10° → Settled, < 1.0 in, no Warn; 20° → one Warn "starts 20.0 deg off"; ±90°
    → "starts 90.0 deg off". Their verdicts past the Warn are not claimed, so not pinned.
  - Evidence they bite: M1 (reverseTo drops the + π) and M15 (drives forward, turns at the end) turn A4b and
    A4c RED; M18 (reverseTo plans its own turn first — the library making the author's turn) turns A4, A4c and
    A4d RED (no Warn, it arrives).

### R2 (correctness minor) — on the X-drive, `reverseTo` alone arrives turned round but does not back in the whole way
- **Re-checked: REAL.** `probe.txt` §4, A5's geometry: `reverseTo` alone — 4.746 in of travel along the
  robot's own nose, heading band 2.55 rad; after `faceAway` — band 0.0129 rad, backward 43.030 of 43.267 in,
  forward 0 in. The docs said "Drive BACKWARDS" unconditionally and "a holonomic drive needs no faceAway".
- **Fixed — the sentences:** `reverseTo`'s doc now opens "Drive to the FIELD point (x, y) and arrive with the
  robot's BACK toward it", says `faceAway` first makes it backwards the whole way on any drive, and that a
  holonomic drive alone "rotates in flight — it still arrives turned round, but its nose leads early in the
  leg"; `faceAway`'s doc: the turn before a `reverseTo` on a tank "or on any drive whose back must lead the
  whole way"; `route.hpp`'s `Waypoint::reverse` ("Back into this leg's point", and the authored `TurnTo` on a
  tank "or on any drive whose back must lead the whole leg") — `docs/api/route.md` regenerated; the changelog
  and guide 9 (the table row: "arrives with its back to it … backwards the whole way after a `faceAway`").
- **Fixed — the pins:** `L9 A5` gains "the nose leads early" (forward travel along the nose > 1.0 in; measured
  4.746) — M18 turns A5 RED on exactly that check (`mut3/M18/suite.txt`, :638) and nothing else in A5; `L9 A2b`
  (new) is A2 on the X-drive after `faceAway` (band < 10°, backward ≥ leg − 1 in, forward < 0.1 in, arrives) —
  M1, M2 and M15 turn it RED.

### R3 (correctness minor) — A4's "exact antipode" never reached the antipode rule; landmine 7 was not pinned
- **Re-checked: REAL**, and one more error found beside it. `a4zero.txt`: at A4's exactly-facing start the raw
  heading error is −3.1415926535897936 (just past −π); `std::remainder` folds it to +3.1415926535897927 =
  π − 4.4e-16, so `wrapRad`'s `w <= -kPi` branch never runs — CCW by rounding. Mutant AP (`w <= -kPi` →
  `w < -kPi` in `angle.hpp`, 140 TUs rebuilt, fresh link): `L9 A4` stays GREEN, as the reviewer found.
  **Also wrong, found while fixing:** A4's comment said a CCW swing "ends left of the line" and Step 3 said "on
  the side it turned toward". The cross product in the test is positive to the RIGHT of start → point, so a CCW
  swing ends on the RIGHT (confirmed on the new geometry: CCW ends at y = −4.900, the right of a robot facing
  −x). The check itself (`(rotated > 0) == (cross > 0)`) was right; its words were not.
- **Fixed:** A4's comments and its offset-0 line now say "CCW by the wrap's rounding (not the rule)" and name
  the side correctly; Step 2 and Step 3 above are corrected in place with pointers here. **`L9 A4d` (new)** is a
  start where the rule really decides: heading exactly +π, the point straight ahead on −x at the robot's own y,
  so the bearing is atan2(+0, −36) = +π, the target is 0 and the raw error is exactly −π (REQUIREd, not
  assumed; `errorTo` must return +π). Measured: CCW, +3.1416 rad, ends 7.100 in right of the line, TimedOut;
  with the rule disabled (`probe_mut_antipode.txt`) the same start swings CW, −3.1416 rad, to the other side.
  **Mutant AP turns A4d RED** (and six unit tests that pin the rule in `angle`/IMU/TermSink tests — the rule was
  guarded there; what was missing was the behaviour through `reverseTo`). A4d is deliberately NOT on the geometry
  rule (at bearing π, `b + π` and `b − π` are the same bits): it exists for the rule, not the steps.

### R4 (tests major) — surviving mutant N1: `reverseTo` could drop `maxAngularSpeed`
- **Re-checked: REAL.** On the tank after `faceAway` a yaw cap never binds; only a leg that turns in flight can
  see it. `probe.txt` §5, X-drive `reverseTo` alone from 90°: 2.07 s uncapped, 3.09 s at 1 rad/s (×1.49 — too
  close to 1.5), 3.86 s at 0.75, 5.45 s at 0.5 (×2.63).
- **Fixed:** `L9 A12b` (new) — that leg at ≤0.5 rad/s must take > 1.5× its uncapped twin. **N1 RED on A12b**
  (and only A12b). `MotionOptions` has three fields; `reverseTo` now has each forwarding pinned (timeout A6,
  maxLinearSpeed A12/A1, maxAngularSpeed A12b). `faceAway` reads two: a turn in place commands no translation
  (`turn_to.hpp` hands the pipeline a pure ω), and a 0.01 in/s `maxLinearSpeed` on `faceAway` is bit-identical
  to none on both drives (`probe.txt` §6) — an equivalent mutant, said in A12b's comment rather than "tested".
  **Not fixed (D1's, predates L9, for the coordinator):** D1's options case covers only `moveTo`'s
  maxLinearSpeed; no step's `maxAngularSpeed` forwarding other than `reverseTo`'s is pinned.

### R5 (tests minor) — surviving mutant N11: bearing computed before `skipIfStopped`
- **Re-checked: REAL** (`probe.txt` §8: on a stopped chain all four field steps aimed at the parked estimate
  are skipped; on a clean chain the same call throws).
- **Fixed:** `L9 A6` (starved reverseTo) now aims `reverseTo`, `faceAway`, `face` and `driveTo` at the parked
  estimate after the stop — `CHECK_NOTHROW` each, then their four skip lines (steps 7–10) and the counters (7
  skipped of 10). `face` and `driveTo` share `bearingTo` and the order, and theirs had no pin since D1, so they
  are pinned here too (a D1 gap closed in L9's file, said in the case's comment). **N11a / N11b / N11c / N11d**
  (each step's bearing moved above its skip check) each turn A6 RED at its own `CHECK_NOTHROW` (:674 / :675 /
  :676 / :677, "THREW … no defined bearing").

### R6 (tests minor) — comments claimed to catch mutants that are equivalent by construction
- **Re-checked: REAL.** `Angle::radians(r)` is `Angle{wrapRad(r)}` and `operator+` is `Angle{wrapRad(a + b)}`
  with `Angle::radians(kPi)` exactly `kPi` (`angle.hpp` :44-46, :70), so `Angle::radians(b + kPi)`,
  `b + Angle::radians(kPi)` and `b + Angle::degrees(180.0)` are the same floating-point work (the reviewer's
  20 M-bearing probe: 0 differ).
- **Fixed:** A3's banner now lists what it catches — a `- π` (the X-drive case, at this geometry), a `reverse`
  read by nobody (both cases), motion logic in `reverseTo` — and says the other spellings are the same code; A1's
  hand-arm comment now says its independence is reading the pose and doing the trigonometry, not its spelling;
  Step 3's A1 line corrected in place. For the coordinator: the brief's landmine 2 ("spelled exactly … or A3's
  memcmp fails") is true only of a `- π`.

### R7 (tests minor) — surviving mutant N9: "the message now names all four steps" was unpinned
- **Re-checked: REAL** (A7 checked only "no defined bearing"; the try/catch would pass silently if nothing threw).
- **Fixed (pinned, not dropped — the changelog's claim is true and useful):** A7 throws from BOTH new steps and
  checks the message contains "no defined bearing", "reverseTo" and "faceAway", with a `threw` flag so a step
  that stopped throwing fails. **N9** (the pre-L9 "face/driveTo" wording) **RED on A7** (:778, :779, both steps).

### R8 (tests minor) — A3's two subcases shared one TEST_CASE with REQUIREs
- **Re-checked: REAL** — doctest ends the case at the first failing REQUIRE, so the X-drive subcase never ran
  when the tank one failed; the first round's report line "M12 (tank part only)" was that artifact (this log's
  Step 5 row said only that the tank subcase was red, which was accurate).
- **Fixed:** two TEST_CASEs, `L9 A3 (tank)` and `L9 A3 (X-drive)`, sharing two small helpers (the Start and the
  reversed DriveTo waypoint). Re-run on fresh binaries: **M12 → RED on runner 10 AND on both A3 cases**; **M1
  → both**; **M7a → A3 (X-drive) and A1, A3 (tank) GREEN** — as Step 5 explained, now readable per case.

### R9 (docs minor) — the roadmap's "L9's additions mutation-proven" claimed four pins never seen failing
- **Re-checked: REAL.** First round: M8/M9 (version), M10a/b (`noexcept`), M11c (terse). The member-type pins
  and the two negative const pins were never mutated.
- **Fixed by running them:** **M16a / M16b** (`reverseTo` / `faceAway` take their options BY VALUE — the
  member's type changes, A8 still compiles) → build fails, `routine_signature_pin_test.cpp:167` / `:175`: `F10
  FREEZE VIOLATION: the frozen signature of Routine::reverseTo(Length, Length, const MotionOptions&) ->
  Routine& (appended at L9, API 2.4) changed …` (and `faceAway`'s); **M17a / M17b** (a declared `const`
  overload of each) → `:409` / `:414`: `F10 FREEZE VIOLATION: the frozen signature of reverseTo staying
  non-const (a const chain cannot secretly advance) changed …` (and `faceAway`'s). The roadmap row now says
  each of L9's seven pins was seen failing the build under its own mutation.

### R10 (docs minor) — "No robot has backed up under this library yet" was false
- **Re-checked: REAL** (HA-160: robot two reversed under the library's graph, driver control, 2026-09-25 — the
  `guide-maintenance.md` :96 trap).
- **Fixed:** "No robot has run a reverseTo yet (robot two has reversed only under driver control)", in
  `reverseTo`'s doc; `docs/api/routine.md` regenerated. The other hardware sentences (guide 9, changelog, the
  register) were already scoped and are unchanged.

### R11 (docs minor) — HA-167 said the calibration was "measured going FORWARDS"
- **Re-checked: REAL** — the push can be BACK-first (`drivetrain_calibrator.hpp` :53-58).
- **Fixed:** the lead paragraph ("measured in ONE unpowered push, one end first — never under power, and never in
  the other direction"); the title and index row ("the way its calibration measured it"); claim (a) (the one
  scale holds under power in BOTH directions; which one a reversed leg exercises depends on which end went
  first); Confidence ("never measured under power, in either direction"); the Source ("whose doc comment names
  HA-167" — it read as a file position); the status sentence. The same "going forwards" phrase was in
  `reverseTo`'s doc, `path_runner.hpp`'s DriveTo doc, the changelog, guide 9 and the test banner — each now says
  "as far as its calibration says" (the changelog adds "the scale comes from one unpowered push"). The brief's
  HA-167 title is the coordinator's record — not edited; this is a logged departure from it.

### R12 (docs minor) — guide 9's stability notice and "the two that back in"
- **Re-checked: REAL.** **Fixed:** the enumerator sentences now follow the `MechanismFailed` clause they belong
  to, then the 2.4 steps clause; "plus 180° for the two turned-around twins" (`faceAway` only turns).

### R13 (docs minor) — three departures from the brief's documentation instructions were not logged
- **(a) HA-167's settle venue.** The brief: "COMP-b's Station K, or S8". The entry says "the first on-robot
  autonomous session — COMP-t's auton tester …, or COMP-b's Station K". "S8" is the handoff's milestone "The
  first on-robot autonomous" (`HANDOFF-remaining-work.md` :649), not a station; the register names settle
  venues by chunk and station ("COMP-t Station M.6", "COMP-b's practice-match station (K)") and never by the
  handoff's S-numbers (grepped: no `S8` anywhere in `hardware-assumptions.md`), so the entry says S8's content in
  words. COMP-t's auton tester (briefed after this brief was written) is the first program that runs an auton
  on the robot, so it is named first; Station K is kept. *(A first draft of this line claimed "no current plan
  document names S8" — the grep, run before this entry was final, found HANDOFF :649; corrected before
  closing.)*
- **(b) No "L9's addition" heading.** The brief: "under an 'L9's addition' heading … placed as T3's additions
  are". The entry has a bold lead paragraph instead, as the four latest single-chunk additions in Group R3 do
  (G1 Part 2a, DEFECTS2, COMP-a, TANK1); T3's `###` heading covers a seven-entry set, and a heading for one
  entry would add a one-entry section to the page's outline. Placement (end of Group R3, after TANK1's) is as
  the brief intends.
- **(c) The idiom sentence was extended.** Brief §1 item 4: carry the hand-written-idiom sentence "unchanged".
  It now reads "a hand-written turnTo(atan2(...)) — or turnTo(atan2(...) + π) — remains bit-identical to it by
  construction": the original clause is intact, and the insertion covers the two new steps the paragraph now
  describes (true, and pinned by A1).

### R14 (docs minor) — two build-order lines
- **The status line** repeated the over-general claim — reworded (R1). It still says "NOT YET VERIFIED OR
  COMMITTED", which is true of this tree; **the coordinator rewrites it to the committed state at commit** (pace
  rule 5).
- **The deviations row** said "API 2.3" — it is L9's own row, so the same bracketed note the L9 entry and the
  handoff bullet carry was added: "*[2026-09-28: TANK1 Part A took 2.3, so L9 Part A takes 2.4]*". Logged here
  for the coordinator's merge.

### R15 (correctness minor) — TANK1's Warn tells a `reverseTo` author to call `face()`
- **Re-checked: REAL** (`move_to_pose.hpp` :478-487 — the Warn cannot know the leg's intent). **Not changed
  here, deliberately:** the text is TANK1's, pinned verbatim by its P5, quoted in ten documents, and the file is
  in flight in DIAG1 Part B; every L9 document says the turn for a backwards leg is `faceAway()`. It stays
  "For the coordinator" item 1 — the team lead's call, for whoever owns `move_to_pose.hpp` next.

### Resumption — the round was stopped by the account's usage limit *(`date` 2026-09-30 02:57:15 EDT)*

The run that wrote R1–R15 above was stopped by the usage limit. Its last log write was 2026-09-29 22:15. File
times show it running again briefly at about 01:03–01:05 on 2026-09-30 before it stopped for good (an
inference from mtimes; nothing it wrote says so). Same builder role, same worktree, same rules. This
resumption's own files are under `.claude/coordinator/scratch/l9-a1/fix/resume/`.

- **What the stopped run left (checked before anything was trusted):**
  - **No mutation left applied.** Every one of `mutate3.py`'s 17 backups (`fix/mut3/<id>/*.orig`: `routine.hpp`
    ×15, `path_runner.hpp`, `angle.hpp`) is `cmp`-identical to the worktree's file.
  - **No self-test plant left.** `git diff -- src/ Makefile` is empty. The two plant targets the stopped
    self-test had reached are byte-identical to HEAD (`src/programs/bench_r3a.cpp`, mtime 01:04:28, and
    `include/liblvgl/core/lv_obj_style.h`, 01:04:41). No other plant target is modified, there is no
    `include/robots/` (the directory the `_newfile` cases create), and nothing is untracked or ignored except
    this log, the new test file, `.d/`, `bin/` and `build/`. No gate or build process was running in this
    worktree.
  - **What it had not finished.** Its doc gates had run and passed at 01:04–01:05 (`fix/doc_gates_final.txt`,
    every rc 0). The src build gate self-test it started at 01:03 (`fix/srcgate_selftest.txt`) is EMPTY: it
    was killed before it printed. So the src gate never ran on the final tree, the final gates were never
    logged, and this section's prose review was run (`fix/prose_review.txt`, 22:13) but never logged.
  - **The test binary** was linked at 22:17:32, after the last mutation's restore. Per the stale-binary rule it
    was not trusted: every object was wiped (`--target clean`) and rebuilt (below).
- **Re-verified WITHOUT re-running, because the evidence already came from fresh binaries.**
  - Each live mutant's `fix/mut3/<id>/build.txt` shows every touched TU recompiled and exactly one `Linking
    CXX executable shulib_tests`. That is 16 TUs for `routine.hpp` and 140 for `angle.hpp`. N1 recompiled 140,
    the set AP's restore re-touched. M12 and M15 recompiled 17, the previous restore's re-touches included.
  - Every failing line each mutant reported is the check this section names, at the same line in the CURRENT
    file: A5 :638 (M18); A6 :674–:677 (N11a–d); A7 :778–:779 (N9); A4d :584 / :595 / :598 (AP); A12b :863 (N1).
    Every mutant run counted 1741 cases, which is the suite as it stands.
  - The three mutated sources are byte-identical to their backups.
  - The probes (`fix/probe.txt` §1–§8, `a4zero.txt`, `probe_mut_antipode.txt`) were re-read. They carry every
    number R1–R5 quote, and they agree with the reviewers' figures as quoted in the findings.
  - One number in a new comment was the correctness reviewer's, not this round's: A5's "35.8 in" (how far into
    the leg the heading comes within 10° of b + π). The reviewer's `review-correctness/p1.cpp` was REBUILT
    against the final headers (`fix/resume/p1_final`). Its output is byte-identical to the reviewer's
    `p1.txt`: 35.84 in of the 43.27 in leg, 4.746 in nose-first, the back leading 76.8 % of the path, and
    100.0 % after `faceAway`.
- **Found while re-verifying, and fixed: A4d's banner still put its CCW swing "left of the line".** This is the
  error R3 corrected in A4's comment, and it survived in the new case's own banner (`routine_reverse_test.cpp`
  :567). The banner contradicted A4d's own check (:596–:598: a CCW swing ends RIGHT of the line of travel) and
  the probe's end at y = −4.900, which is +y and the right of a robot travelling down −x. It is reworded to
  "counter-clockwise, ending RIGHT of the line of travel (+y here)", with both measured ends: y = −4.900 and,
  with the rule disabled, y = −19.100 (`probe_mut_antipode.txt`).
  - The change is to a comment only, and it was kept to the same five lines, so the file is 865 lines as before
    and every line number cited above is unchanged.
  - A grep for "left of the line", "ends left" and "turned toward" over the L9 test, header and public-doc text
    now finds only this log's corrected-in-place history notes. The other hits are TANK1's text (its xt sign
    convention, and its "turned toward its line"), which is unrelated.
- **The prose review for this round (pace rule 4).** The greps were re-run on the final tree and every hit was
  read BEFORE this entry. Output: `fix/resume/prose_p1_p3.txt`, `prose_p4_p8.txt`, `prose_p9_p12.txt`.
  - (P1) Every "180°" near `reverseTo` / `faceAway` in the headers, public docs and tests is either the
    definition (the bearing + 180°) or is conditioned on a robot FACING the point (`routine.hpp` :84, :268;
    guide 9 :236; the VexBuilder row; build-order :1245). The other hits are unrelated: AprilTag corner
    windings and latency.
  - (P2) Every "almost 8 in" / "~7.8 in" carries its 43 in leg (guide 9 :240, changelog :47, build-order
    :1245, the A4 comment).
  - (P3) Every X-drive claim says the nose leads early without `faceAway`: `routine.hpp` :265, `route.hpp`
    :282, guide 9 :241-242, changelog :49-50, build-order :1245.
  - (P4) No "going forwards" / "measured forwards" is left. Each backing-up sentence reads "as far as its
    calibration says": guide 9, changelog, `routine.hpp`, `path_runner.hpp` and the test banner.
  - (P5) The one hardware-history sentence L9 wrote is scoped ("No robot has run a reverseTo yet (robot two has
    reversed only under driver control)"). Every other hit is the standard "never driven a robot under its own
    control" or a scoped "nothing here has run on a robot".
  - (P6) The roadmap's F10 row claims mutation evidence for exactly the seven pins that were mutated (M10a/b,
    M11c, M16a/b, M17a/b).
  - (P7) Every side-of-line sentence in the L9 text now says RIGHT for a CCW swing.
  - (P8) Every public quote of the Warn's `turn first: face()` about a BACKWARDS leg adds `faceAway()` (guide 9
    :238, changelog :45, guide 11 :213-215, guide 12 :78-79). The rest (guide 4 :104, guide 9 :127, guide 10
    :348, guide 14 :362, cookbook 4 :36, changelog :103, `routine.hpp` :248) are about forward legs and are
    true. Cookbook 4 is Part A2's.
  - (P9) The test comments claim independence only for reading the pose and doing the trigonometry by hand
    (A1 :247-250). The A3 banner says the equivalent spellings are the same code (`path_runner_test.cpp`
    :759-762).
  - (P10) "Names all four steps" (changelog :39) is pinned by A7 :777-779.
  - (P11) HA-167's index row, status sentence and entry agree.
  - (P12) The generated `docs/api/routine.md` :367, `route.md` :596 and `path_runner.md` :300 carry the
    reworded comments; `check-fresh` agrees.
  - Nothing stale was found beyond A4d's banner.

### The gates on the final tree (the review round) *(`date` readings as given; output under `fix/resume/`)*

Every gate was run in sequence, so the src gate's self-test, which plants faults in `src/`, `include/` and
`test/`, never overlapped a host build or the ARM gate. The stopped run had let the self-test overlap its doc
gates.

- **Host build from nothing (03:11:40–03:12:17).** The briefing was regenerated, then `cmake -S test -B
  build/test`, then `--target clean` (0 objects left, no binary), then a full build: 177 of 177 TUs compiled,
  one `Linking CXX executable shulib_tests`, no warning, and the build's doc gates green.
  - Suite (03:12:31): `1741 | 1741 passed | 0 failed | 3 skipped`, `2649745 | 2649745 passed`.
  - That build already carried a first wording of A4d's fix, which was six lines. It was then re-cut to five, so
    no cited line number moved. After that (03:15), `routine_reverse_test.cpp.o` alone was recompiled and
    relinked. The suite gave the same 1741 / 2 649 745, and the briefing was regenerated after the binary
    existed. The 16 L9 cases are all registered.
- **The documentation gates, run as CI runs them (after 03:16):**
  - `api_doc_tool self-test: OK`.
  - `doc coverage: 2937 public entities across 156 headers, all documented`.
  - `check-fresh` rc 0.
  - `doc example scan: 467 quoted lines, all verbatim (5 source files); 3 listings of whole-run files, each one
    unbroken run`.
  - `removability: no public doc references docs/internal/`.
  - `briefing_status.py check`: `generated block current`.
  - `doc_staleness_audit.py self-test: OK (10 detector/filter cases)`; the audit itself: `clean (27 live docs,
    0 numeric claims checked)`.
- **The CI greps (03:21):** both print nothing (`library is PROS-free (all of include/shulib except
  hal/pros/)`, `layering holds: core is sim-free`).
- **The ARM header gate (03:21:07):** RESUMING's command, with the TU in the scratch dir: 192 headers, rc 0.
  `chassis/routine.hpp`, `version.hpp`, `runner/path_runner.hpp` and `config/route.hpp` each also compile
  alone, rc 0.
- **`python3 tools/src_build_gate.py self-test` (03:23:00–03:33:00):** `src build gate self-test: OK (189
  detector cases, each against a real build)`.
  - The tree was snapshotted just before it: the `git diff` sha256, `git status --porcelain --ignored`, and the
    new test file's sha256.
  - Afterwards the tracked diff and the status list were IDENTICAL, the test file was unchanged, there was no
    `include/robots/`, and `git diff -- src/ Makefile` was 0 lines. The log differed, by this section's edits.
- **`python3 tools/src_build_gate.py check` (03:33:35–03:35:30):** `src build gate: PASS (all 6 builds link;
  …)`.
  - Each of bench, xdrive, tank, tank-drive, tank-library and tank-calibrate: exit 0, 8/8 TUs, 6
    vendor-ignored warnings, 0 disallowed; hash, beacons and program asserted.
  - `[one-chain]`, `[chain-table]`, `[program-table]`, `[include-root]` and `[printed-path]` all hold.
  - The tracked diff was still identical afterwards.
- **The host suite once more, after every gate (03:36:38–03:38:06).**
  - The self-test's byte-exact restores had bumped the mtime of the vendored `doctest.h`, so all 177 TUs
    recompiled: a second build from nothing. It linked once, with no warning, and the build's doc gates were
    green.
  - Suite: `1741 | 1741 passed | 0 failed | 3 skipped`, `2649745 | 2649745 passed`.
  - The briefing was regenerated after the binary existed; `check`: `generated block current`.
- **Not committed. No `pros` command. No `/dev/ttyACM*`.** Part A2 and Part B are not started. There is no
  `L9-COMPLETED.md`, and the status pointers were not moved: `build-order.md`'s L9 status line still says
  "NOT YET VERIFIED OR COMMITTED", which the coordinator rewrites at commit (R14).

### 2026-09-30 03:40–04:20 EDT — the coordinator's merge and verification of Part A1 (by `date`)

- **The run.** The build (2026-09-29), three reviews (17 findings, deduplicated here as R1–R15), and the fix
  round — stopped by the account's usage limit after all fifteen dispositions were logged but before its final
  gates, then resumed at 03:11 with a continuation note: it found no mutation and no gate plant left applied,
  rebuilt every object from nothing, corrected one comment (A4d's), and ran every gate on the final tree
  (03:11–03:41; the section above). Transport commit `2cb8c66` in the worktree `l9-a1`.
- **The merge** onto `0f56301` (COMP-t, the auton tester, and TANK1 Part B, both landed while Part A1 was
  built): three hand-resolved conflicts. `include/shulib/version.hpp` — both notes kept, `kApiMinor = 4`,
  "2.4". `docs/hardware-assumptions.md` — the status paragraph (COMP-t's sentence, then this part's, the
  running total 161 — "with it, 159" was written before HA-177/178 existed — and the held-number sentence
  now also naming HA-181, G3a's, and HA-182..HA-186, the mechanism core's); the index rows in numeric order
  (HA-167 before HA-177/178); the detail sections in landing order (COMP-t's, then this part's); the status
  line "7 of 161". `docs/api` (README, all-entities, version) and the briefing taken from main and
  regenerated. Auto-merged and read: the changelog (`## API 2.4` above 2.3, COMP-t's "since 2.3" entry under
  2.3, where it belongs), guides 9, 11 and 12, the roadmap's F8/F10 rows, build-order's deviations row.
- **Hand edits at the merge:** build-order's L9 status line (LANDED 2026-09-30) and its Next line;
  REMAINING-WORK (row 29, the totals, the status paragraph); HANDOFF's stale-binary trap gains this part's
  form (a test file added after the last full build) and H1 Part A1's (a killed mutation run leaves the last
  mutant LINKED) — "For the coordinator" item 4, done.
- **Held numbers — the register and the briefing tool.** The register gains a one-line **Held numbers** list
  (HA-162..HA-176, HA-181..HA-186), and `tools/briefing_status.py` names the next free number above every row
  AND every held one: the briefing read "next free is HA-181" while G3a's amended brief held it (G3a's
  re-check, #12); it now reads HA-187. Its `self-test` grows to 24 cases; three mutations of the rule (held
  numbers ignored, a range read one short, the line matched without its bold label) each turned it red.
- **Gates on the merged tree:** the suite **1,824 | 1,824** (2026-09-30 03:48), 3 skipped; `src_build_gate.py`
  self-test OK (198 cases) and check PASS on all seven builds; the ARM header gate over all 193 headers; both
  CI greps empty; `check-fresh`, the briefing's self-test and check, removability and the staleness audit
  clean; no plant after the gates or after the mutations (`.claude/coordinator/logs/l9_merge_verify.log`).
  After the held-numbers change: the briefing's self-test (24 cases), regenerated, `check` current.
- **Eight coordinator mutations, each on a fresh binary** (`.claude/coordinator/muts_l9a1.json`):

  | # | the mutation | caught by |
  |---|---|---|
  | LA1 | `reverseTo` drives forwards (no `+ π`) | 14 cases red |
  | LA2 | `faceAway` faces the point (no `+ π`) | 6 cases |
  | LA3 | a stopped chain still runs `reverseTo` | 2 cases |
  | LA4 | a stopped chain still runs `faceAway` | 1 case |
  | LA5b | `reverseTo` drops the caller's options | 5 cases (`routine_reverse_test.cpp` :261-262, :657) |
  | LA6b | `faceAway` drops the caller's options | 4 cases (:261-262, :718) |
  | LA7 | `reverseTo`'s lines named `driveTo` | 3 cases |
  | LA8 | the API minor left at 3 | 3 cases |

  LA5 and LA6's first form did not compile (an unused parameter is an error), so they proved nothing and were
  re-run as LA5b/LA6b, which reference the options and ignore them. Every file restored byte-exact; the
  restored tree relinked and passed.
- **An independent read-only audit of the merge** (two agents: every hunk of both sides present; no
  cross-document contradiction): started, then STOPPED unreported at the team lead's usage warning (91 %) —
  to re-run against this commit at the next session; the gates and mutations above stand on their own.
- **Left open:** "For the coordinator" item 1 (TANK1's Warn says `turn first: face()` for a backwards leg) —
  the team lead's call, brought to him with the other open decisions; Parts A2 and B not started. Nothing of
  L9 has run on a robot.
