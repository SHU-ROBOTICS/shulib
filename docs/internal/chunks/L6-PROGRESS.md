# L6, the wall reset — progress log (live)

- **Chunk:** L6, the wall reset with the side distance sensors (brief:
  [`L6-wall-reset.md`](L6-wall-reset.md)) — three runs and an optional fourth (A1, A2, B; C optional), each its
  own go (the brief's D1, §2.1).
- **Executor model:** opus.
- Appended as the work goes, a heading per run. Every time in a section header is a `date` reading taken when
  the section was written, not an estimate written afterwards. **Nothing in this chunk has run on a robot.**
  Say "host test plant", never "simulator".

---

## L6-A1

- **Started:** see the first entry's `date`.
- **Tree at start:** `d8fdb8b` (main; the panel sprint's plan commit), clean, in the git worktree
  `.claude/worktrees/l6-a1` (branch `l6-a1`) — NOT the main working tree.
- **Go:** the team lead, 2026-10-22: "Accept all (Recommended)" — D1–D13 as recommended; D2 AMENDS locked
  decision #4 for a caller-invoked ONE-AXIS seat while no motion is active. His sprint ruling of 2026-10-29
  ("Sprint, keep the wall reset") keeps L6 building beside MECH-A5 and G3a Part 1. The coordinator's workflow run
  launched L6-A1 ONLY. Do not commit; no `pros` command; no `/dev/ttyACM*`. Mutation backups and probes under
  `.claude/coordinator/scratch/l6-a1/` (never `/tmp`, never the repository root).
- **Scope (§2.1's L6-A1 row):** NEW `localization/{axis,override_walls,wall_reset}.hpp`,
  `chassis/wall_reset_call.hpp`, `sim/field_plant.hpp`, five test files; CHANGED `localization/localizer.hpp`
  (`seatAxis` + two counters), `hal/distance.hpp` (comment sentences only), `test/cookbook_examples_test.cpp`
  (D11). NOT: L6-A2 (`config/wall_sensor_config.hpp`, `robot/robot.hpp`, `chassis/robot_context.hpp`,
  `competition/wall_reset.hpp`), L6-B (`src/`), L6-C, `version.hpp`.


### 14:39 — start
- Read `docs/internal/RESUMING.md` (standing rules, pace rules 1-8) and, per the team lead's 2026-10-22 reading ruling, the brief's header and answers block, "## Re-check — amendments" (RC1-RC48) in full, the Decisions (D1-D13) in full, the plain words, §1.1-§1.3, §1.7-§1.9, §1.12 (for the explicit/everyday split), §2.1-§2.3, §3.1-§3.4, §4, §5, §6 and §7-§9 in full. This log created before any code.

### 18:16 (2026-10-29, `date`) — RESUMED after the stop (~15:10) and the reboot (17:58)
- The coordinator stopped this build at ~15:10 (the team lead at 92 % of the session limit); the laptop rebooted
  at 17:58; resumed the same evening by a fresh executor (opus). The log above held only the 14:39 start entry —
  the work between 14:39 and ~15:01 was NOT logged by the stopped run; it is recorded here from the tree.
- **Audit of the tree at resumption** (`git status`, `git diff`): 7 paths — `docs/internal/PROJECT-BRIEFING.md`
  (regenerated against an UNBUILT binary: "Suite: not built" — to be regenerated again after a fresh build),
  `include/shulib/localization/localizer.hpp` (+136/−7: `seatAxis`, `travelSinceSeat`, `turnedSinceHeadingSeed`,
  the banner's sentence), and NEW `localization/{axis,override_walls,wall_reset}.hpp`,
  `chassis/wall_reset_call.hpp`, this log. `git diff -- src/ Makefile` EMPTY — no gate self-test plant left by a
  killed run. No mutation backups in the scratch folder (only `baseline_build{,2}.log`, `cmake_config.log`,
  `probe/inc.cpp`). NOT yet written: `sim/field_plant.hpp`, every test file, `hal/distance.hpp`'s two comment
  sentences, cookbook 05 / its compiled twin (D11), every public document, every mutation.
- Re-read the four new headers and the localizer diff in full before continuing (nothing reverted; deltas
  from the brief's listing are logged below as found).
- `build/test/shulib_tests` existed (stale, from before the stop): removed before any result is trusted.

### 18:38 — the resumed tree builds and stays green; the plant and the first four test files
- **Fresh build first** (`rm -f build/test/shulib_tests`, `cmake -S test -B build/test`, `cmake --build`): the
  doc gate asked for `api_doc_tool.py generate` (the stopped run had not regenerated `docs/api/` for the new
  headers — `axis`, `override_walls`, `wall_reset`, `wall_reset_call` pages and `localizer.md`'s three new members)
  and a briefing regeneration (230 headers); both done (the briefing against an ABSENT binary — "not built").
  Then `python3 tools/suite_run.py green` (under the 4-GiB cap): **green — 2626 cases, 2799840 assertions** — the
  Localizer's counters and `seatAxis` change no existing observable (the base reads 2,626 cases).
- **The dev build from here** (`scratch/l6-a1/b.sh`): `rm -f build/test/shulib_tests` → `briefing_status.py
  generate` (so it reads "not built", never a stale or mutant link) → `cmake --build build/test --target
  shulib_tests`. The test target depends on the doc gates, which run `briefing_status.py check` — so the
  briefing must match an absent binary at every build; it is regenerated against the FINAL fresh binary last.
- NEW `include/shulib/sim/field_plant.hpp` (§1.7): `sim::FieldModel` (boxes with heights, opaque or see-through,
  the floor; `override2026()` from the plant's OWN copy of the drawing numbers — walls ±70.2047, 11.5 in tall, the
  Toggle body on top with a 1-in overhang, its brackets to 10.6 in, Loaders on the red/blue walls, A12's Cup-Pin-Cup
  clusters on every wall) and `sim::SideSensorPlant` (the TRUE mount; a 7 × 7 cone kept inside the circle, nearest
  hit or `blend`; the code strip's per-colour bias/confidence; bias, seeded noise, update hold, k-step latency,
  drop-outs, FROZEN, read faults through a `FakeReadFaults` with ProsDistance's PROS_ERR behaviour; 9999 / conf 0
  past 78.74 in). Includes nothing from `localization/`. Host and ARM flags: compiles clean.
- NEW tests: `test/localizer_seat_axis_test.cpp` (T6 both tiers, T7, T16's counters), `test/wall_reset_geometry_test.cpp`
  (T1, T2, T11), `test/wall_reset_gates_test.cpp` (T3, T4, T5, T9, T10, T6's refused half), `test/sim_field_plant_test.cpp`
  (T14), and a shared rig `test/wall_reset_rig.hpp` (the tank ChassisRig + both side-sensor plants stepped by the
  pacer — **a delta: the brief lists five test files; the rig header is the project's rig pattern**, e.g.
  `mechanism_position_rig.hpp`). All 41 L6 cases green after three test fixes, each a test-side mistake, not a
  library one: (1) T6's "a 10-in drive moves by the odometry's 10 in only" — under EkfFusion the estimate lags the
  odometry by ~0.02 in on any drive (seated or not), so the case now compares a seated rig with a TWIN seated to its
  own value (the same dt-0 re-base, no jump): the two differ by exactly the seat at every tick; (2) T14's
  see-through case read the FLOOR through a 7 × 7 cone from 10.5 in (the lowest ray meets it at 33.98 in) — it now
  uses the centre ray; (3) T6's refused case shifted the estimate 5 in AWAY from the wall, which the beam-height
  gate refuses before sampling (BeamLow) — it now shifts 4 in TOWARD the wall so the window runs and the
  agreement gate refuses.

### 18:59 — the mutations: 36 run, 36 RED, every one restored
- Driver `scratch/l6-a1/mut/driver.py` (definitions `mut/mutations.json`, built by `mut/make_muts.py`): each mutation
  backs up its files to `scratch/l6-a1/mut/bak/`, applies exact-once replacements that keep the line count (so
  `docs/api/`'s line numbers and the doc gates stay valid), builds through `b.sh` (`rm -f build/test/shulib_tests`, the
  briefing regenerated against the ABSENT binary, `cmake --build build/test --target shulib_tests` — every object that
  depends on the mutated file recompiled), runs `-tc='L6*,cookbook-05b*'` under `ulimit -v 4194304` + `timeout 600`,
  and restores in a `finally` (byte-exact — sha-256 checked — then the file's mtime bumped). Localizer mutations
  rebuilt ~37-57 s (the header is everywhere); the others ~8-11 s. Every red below is a TEST ASSERTION (no build failure
  was counted; none occurred). M18 is deleted by RC1; M25-M28, M31, M40, M41 are L6-A2's.

| # | Mutation | Verdict | Failing cases (the L6 suite's) |
|---|---|---|---|
| M1 | a wall's inward sign ignored (aimAt) | RED (3 failed) | L6 T1 the aim's along coordinate is where the centre ray hits the wall; L6 T1 the inversion is exact: every wall, side, offset, skew and bias; L6 T11 an own wall line in a rotated frame gives the same seat as the season wall |
| M2 | the range used without the cos (beam off normal) | RED (2 failed) | L6 T1 the inversion is exact: every wall, side, offset, skew and bias; L6 T2 the error bound bounds: 20 000 draws inside the tolerances |
| M3 | the lever arm rotated by -heading | RED (16 failed) | L6 T1 the inversion is exact: every wall, side, offset, skew and bias; L6 T10 then() and D3: seated continues, refused continues unless required; L6 T11 an own wall line in a rotated frame gives the same seat as the season wall; L6 T16 the along drift beside a Toggle bracket: seated wrong without it, refused with it; L6 T2 the error bound bounds: 20 000 draws inside the tolerances; L6 T5 a sensor with no read-fault count is accepted and says faults n/a; L6 T8 RC31 a caller-paced tick after a seat over 2.25 in raises exactly one IMPLAUSIBLE; L6 T8 RC34 the start cluster in the beam: 8/9 in up refused beam low, 10.5/11 in up seated; L6 T8 RC41 with toggleChecked the Toggle body lifts and its brackets do not; L6 T8 S1 a leg along the left wall, the estimate drifted 2.5 in: SEATED within 0.1 in; L6 T8 S11b a 2-deg heading error with the sensor at the fore-aft centre: within 0.05 in; L6 T8 S12 a mount typed 0.5 in wrong seats 0.5 in off; L6 T8 S2 drifted past maxCorrection: REFUSED disagrees, untouched; L6 T8 S5b/S5c pieces under the beam: a lying Cup, a lying Pin, a standing Pin — SEATED; L6 T8 S7a/S7b a sensor mounted backwards: refused in the open; a robot 5 in away seats wrong; L6 T9 one call, one WALL line, no SCH line; the drive never commanded; the window bounded |
| M4 | the bias added, not subtracted | RED (1 failed) | L6 T1 the inversion is exact: every wall, side, offset, skew and bias |
| M5 | the mount-position term dropped from the error bound | RED (1 failed) | L6 T2 the error bound bounds: 20 000 draws inside the tolerances |
| M6 | the accuracy model replaced by the 200-mm STEP | RED (1 failed) | L6 T2 the accuracy is max(15 mm, 5 %) everywhere, never a step; a measured one replaces it |
| M6b | a measured rangeAccuracy ignored | RED (1 failed) | L6 T2 the accuracy is max(15 mm, 5 %) everywhere, never a step; a measured one replaces it |
| M6c | the heading tolerance held at the floor (the turn counter ignored) | RED (1 failed) | L6 T2 the heading tolerance grows with the rotation since the heading seed |
| M7 | the square gate's comparison flipped | RED (34 failed) | L6 T10 then() and D3: seated continues, refused continues unless required; L6 T11 an own wall line in a rotated frame gives the same seat as the season wall; L6 T16 a reset right after the perpendicular axis was seated is not over-refused; L6 T16 the along drift beside a Toggle bracket: seated wrong without it, refused with it; L6 T16 the along drift: a Loader's estimate 4 in off seats wrong without it, refused with it; L6 T3 beam height: an expected range under the ceiling with a median past it is BeamLow; L6 T3 beam height: at the ceiling passes, +0.01 in refuses BeamLow; the cone used; L6 T3 keep-out kinds: PieceZone lifts with a measured cone, Toggle with the flag, Fixed never; L6 T3 keep-out: a footprint that overlaps a span refuses, even with the centre outside it; L6 T3 square: 4.9 deg passes, 5.1 deg refuses NotSquare; a beam facing away is NotSquare; L6 T4 a refused aim costs no time and reads nothing; MotionActive cancels nothing; L6 T5 a sensor with no read-fault count is accepted and says faults n/a; L6 T6 a refused reset leaves the Localizer bit-identical; L6 T8 RC31 a caller-paced tick after a seat over 2.25 in raises exactly one IMPLAUSIBLE; L6 T8 RC34 an unmeasured cone keeps the start pieces out; a 3-in lens is beam low; L6 T8 RC34 the start cluster in the beam: 8/9 in up refused beam low, 10.5/11 in up seated; L6 T8 RC36 a beam inside a Load Zone is kept out, the cone measured or not; L6 T8 RC37 a clear panel with an object close behind seats wrong by the gap (known limit); L6 T8 RC41 with toggleChecked the Toggle body lifts and its brackets do not; L6 T8 S1 a leg along the left wall, the estimate drifted 2.5 in: SEATED within 0.1 in; L6 T8 S10a the beam on the Toggle: REFUSED keep-out; L6 T8 S11b a 2-deg heading error with the sensor at the fore-aft centre: within 0.05 in; L6 T8 S12 a mount typed 0.5 in wrong seats 0.5 in off; L6 T8 S2 drifted past maxCorrection: REFUSED disagrees, untouched; L6 T8 S3 parked 8 deg off parallel: REFUSED not square; L6 T8 S4a'/S4b' beside a Loader: keep-outs ON refuse; OFF seats 3.7 in wrong (known limit); L6 T8 S5b/S5c pieces under the beam: a lying Cup, a lying Pin, a standing Pin — SEATED; L6 T8 S5d a 4-in lens with a standing Pin in front: REFUSED beam low; L6 T8 S6 a read fault mid-window: REFUSED read fault; L6 T8 S7a/S7b a sensor mounted backwards: refused in the open; a robot 5 in away seats wrong; L6 T8 S8 a clear wall panel with nothing behind it: REFUSED; L6 T8 S9 sampled while driving: REFUSED moving; L6 T9 one call, one WALL line, no SCH line; the drive never commanded; the window bounded; L6 T9 the window ends at windowMax by the clock, never later (M23) |
| M8 | the footprint widening dropped | RED (3 failed) | L6 T16 a reset right after the perpendicular axis was seated is not over-refused; L6 T16 the along drift: a Loader's estimate 4 in off seats wrong without it, refused with it; L6 T3 keep-out: a footprint that overlaps a span refuses, even with the centre outside it |
| M9 | the speed read once at the start of the window, not every tick | RED (1 failed) | L6 T3 stillness: 0.99 / 1.01 in/s and 2.9 / 3.1 deg/s; a push mid-window is seen |
| M10 | a 9999 (no-return) sample skipped instead of refusing | RED (2 failed) | L6 T3 each sample: no return, confidence 0.50 / 0.49, range 2.0 / 1.99 and 16.0 / 16.01; L6 T9 the window ends at windowMax by the clock, never later (M23) |
| M11 | the spread gate removed | RED (1 failed) | L6 T3 enough and consistent: 5 / 4 samples, the window's time, spread 0.5 / 0.515625 |
| M12 | the disagreement compared on the OTHER axis | RED (22 failed) | L6 T10 then() and D3: seated continues, refused continues unless required; L6 T16 a reset right after the perpendicular axis was seated is not over-refused; L6 T16 the along drift beside a Toggle bracket: seated wrong without it, refused with it; L6 T16 the along drift: a Loader's estimate 4 in off seats wrong without it, refused with it; L6 T3 beam height: an expected range under the ceiling with a median past it is BeamLow; L6 T3 enough and consistent: 5 / 4 samples, the window's time, spread 0.5 / 0.515625; L6 T3 precise and agrees: the bound at its limit, the correction at its limit; L6 T3 stillness: 0.99 / 1.01 in/s and 2.9 / 3.1 deg/s; a push mid-window is seen; L6 T3 the value used is the median, not the mean; L6 T5 a sensor with no read-fault count is accepted and says faults n/a; L6 T5 the read-fault rule: a count that moves inside the last sample refuses ReadFault; L6 T8 RC31 a caller-paced tick after a seat over 2.25 in raises exactly one IMPLAUSIBLE; L6 T8 RC34 the start cluster in the beam: 8/9 in up refused beam low, 10.5/11 in up seated; L6 T8 RC37 a clear panel with an object close behind seats wrong by the gap (known limit); L6 T8 RC41 with toggleChecked the Toggle body lifts and its brackets do not; L6 T8 S1 a leg along the left wall, the estimate drifted 2.5 in: SEATED within 0.1 in; L6 T8 S11b a 2-deg heading error with the sensor at the fore-aft centre: within 0.05 in; L6 T8 S12 a mount typed 0.5 in wrong seats 0.5 in off; L6 T8 S4a'/S4b' beside a Loader: keep-outs ON refuse; OFF seats 3.7 in wrong (known limit); L6 T8 S5b/S5c pieces under the beam: a lying Cup, a lying Pin, a standing Pin — SEATED; L6 T8 S7a/S7b a sensor mounted backwards: refused in the open; a robot 5 in away seats wrong; L6 T9 one call, one WALL line, no SCH line; the drive never commanded; the window bounded |
| M13 | an UNSET maxCorrection (0) taken as infinite | RED (1 failed) | L6 T3 maxCorrection UNSET (0) is refused by name; NaN and negative are refused |
| M14 | the pre-check run after sampling (a window read before refusing) | RED (2 failed) | L6 T4 a refused aim costs no time and reads nothing; MotionActive cancels nothing; L6 T8 S5d a 4-in lens with a standing Pin in front: REFUSED beam low |
| M15 | the read-fault count read once per window | RED (2 failed) | L6 T5 the read-fault rule: a count that moves inside the last sample refuses ReadFault; L6 T8 S6 a read fault mid-window: REFUSED read fault |
| M16 | the count compared only around distance() | RED (1 failed) | L6 T5 the read-fault rule: a count that moves inside the last sample refuses ReadFault |
| M17 | the seat clears the drift term (setPose's behaviour) | RED (4 failed) | L6 T6 seatAxis(X) moves x and nothing else — ComplementaryFusion; L6 T6 seatAxis(X) moves x and nothing else — EkfFusion; L6 T6 seatAxis(Y) moves y and nothing else — ComplementaryFusion; L6 T6 seatAxis(Y) moves y and nothing else — EkfFusion |
| M18b | the seat calls odom_.setPose (the dropped shift restored) | RED (4 failed) | L6 T6 seatAxis(X) moves x and nothing else — ComplementaryFusion; L6 T6 seatAxis(X) moves x and nothing else — EkfFusion; L6 T6 seatAxis(Y) moves y and nothing else — ComplementaryFusion; L6 T6 seatAxis(Y) moves y and nothing else — EkfFusion |
| M19 | hasLast_ left true by the seat | RED (5 failed) | L6 T6 after a seat, a 10-in drive moves the seated axis by the odometry's 10 in only; L6 T6 seatAxis(X) moves x and nothing else — ComplementaryFusion; L6 T6 seatAxis(X) moves x and nothing else — EkfFusion; L6 T6 seatAxis(Y) moves y and nothing else — ComplementaryFusion; L6 T6 seatAxis(Y) moves y and nothing else — EkfFusion |
| M20 | seatAxis's live check removed | RED (1 failed) | L6 T7 seatAxis refuses a not-live estimate and a non-finite value |
| M21 | the median replaced by the mean | RED (1 failed) | L6 T3 the value used is the median, not the mean |
| M22 | a second log line per sample | RED (1 failed) | L6 T9 one call, one WALL line, no SCH line; the drive never commanded; the window bounded |
| M23 | the window bound (windowMax) ignored | RED (1 failed) | L6 T9 the window ends at windowMax by the clock, never later (M23) |
| M24 | continues() ignoring required | RED (1 failed) | L6 T10 then() and D3: seated continues, refused continues unless required |
| M29 | the plant's wall at 70.0 instead of 70.205 | RED (14 failed) | L6 T1 the inversion is exact: every wall, side, offset, skew and bias; L6 T14 the plant's Override field agrees with override_walls.hpp to 0.01 in; L6 T16 a reset right after the perpendicular axis was seated is not over-refused; L6 T2 the error bound bounds: 20 000 draws inside the tolerances; L6 T8 RC34 the start cluster in the beam: 8/9 in up refused beam low, 10.5/11 in up seated; L6 T8 RC37 a clear panel with an object close behind seats wrong by the gap (known limit); L6 T8 RC41 with toggleChecked the Toggle body lifts and its brackets do not; L6 T8 S1 a leg along the left wall, the estimate drifted 2.5 in: SEATED within 0.1 in; L6 T8 S11b a 2-deg heading error with the sensor at the fore-aft centre: within 0.05 in; L6 T8 S12 a mount typed 0.5 in wrong seats 0.5 in off; L6 T8 S2 drifted past maxCorrection: REFUSED disagrees, untouched; L6 T8 S4a'/S4b' beside a Loader: keep-outs ON refuse; OFF seats 3.7 in wrong (known limit); L6 T8 S5b/S5c pieces under the beam: a lying Cup, a lying Pin, a standing Pin — SEATED; L6 T9 one call, one WALL line, no SCH line; the drive never commanded; the window bounded |
| M30 | the recipe's 0_in restored (the cookbook listing AND its compiled twin) | RED (1 failed) | cookbook-05b: re-seeding the estimate in the middle of a routine |
| M32 | the motion check dropped | RED (1 failed) | L6 T4 a refused aim costs no time and reads nothing; MotionActive cancels nothing |
| M33 | the along drift term dropped | RED (3 failed) | L6 T16 a reset right after the perpendicular axis was seated is not over-refused; L6 T16 the along drift beside a Toggle bracket: seated wrong without it, refused with it; L6 T16 the along drift: a Loader's estimate 4 in off seats wrong without it, refused with it |
| M34 | seatAxis zeroing BOTH counters | RED (4 failed) | L6 T16 the along drift beside a Toggle bracket: seated wrong without it, refused with it; L6 T16 travelSinceSeat: grows by path length, setPose zeroes both, seatAxis its own only; L6 T6 seatAxis(X) moves x and nothing else — ComplementaryFusion; L6 T6 seatAxis(X) moves x and nothing else — EkfFusion |
| M35 | the turn counter summing SIGNED dtheta | RED (1 failed) | L6 T16 turnedSinceHeadingSeed sums \|dtheta\| through the wrap; seeds restart it |
| M36 | the beam-height gate removed (rule 4) | RED (2 failed) | L6 T3 beam height: at the ceiling passes, +0.01 in refuses BeamLow; the cone used; L6 T8 S5d a 4-in lens with a standing Pin in front: REFUSED beam low |
| M37 | the median's ceiling check removed (rule 7) | RED (1 failed) | L6 T3 beam height: an expected range under the ceiling with a median past it is BeamLow |
| M38 | the piece zones' rule inverted | RED (4 failed) | L6 T16 the along drift beside a Toggle bracket: seated wrong without it, refused with it; L6 T3 keep-out kinds: PieceZone lifts with a measured cone, Toggle with the flag, Fixed never; L6 T8 RC34 an unmeasured cone keeps the start pieces out; a 3-in lens is beam low; L6 T8 RC34 the start cluster in the beam: 8/9 in up refused beam low, 10.5/11 in up seated |
| M39 | toggleChecked lifting the Toggle's brackets too | RED (2 failed) | L6 T16 the along drift beside a Toggle bracket: seated wrong without it, refused with it; L6 T8 RC41 with toggleChecked the Toggle body lifts and its brackets do not |

- **M19 is RED under BOTH tiers here, not the EKF only** (a delta from T6's text, logged): T6 pins RC30's per-tick
  quality — the dt-0 re-base tick reads Degraded 0.100 — and with `hasLast_` left true that tick has a healthy dt
  and reads 0.200, so the complementary tier fails ONLY that pin. Under `EkfFusion` the bounce-back reproduces RC1's
  probe R1 exactly: the filter's velocity reads **167.625 in/s** and the estimate sits at -40.32 for a seat to -42 —
  **1.68 in of the 2-in seat bounced back** — and the twin drive case loses ~2.8 of its 3 in.

### 19:12 — D11, the remaining code, the documents, the prose review, the deltas
- **D11 (cookbook 05's re-seed recipe):** `docs/cookbook/05-mixing-tiers.md` "Re-seed the estimate mid-routine" now
  keeps y (`r.startAt(Pose2d{36_in, c.chassis.pose().y(), c.chassis.pose().heading()})`), says `startAt` clears the
  drift term for both axes, points at guide 10's wall reset, and "the simulator's ground truth" became "the host test
  plant's ground truth". Its compiled twin `test/cookbook_examples_test.cpp` ("cookbook-05b") drives along y = 12 in
  (T15) and pins y unchanged and the drift term 0. M30 (both the listing and the twin back to `0_in`) is RED above.
- **`hal/distance.hpp`** (F4, frozen): the two comment sentences gained "the wall reset (L6)" — §7's comment-only
  check prints nothing.
- NEW `test/wall_reset_example_test.cpp` ("guide-10w" — guide 10's listing compiled; `check-examples` matches it):
  **a delta** — a seventh test file, so guide 10's listing is compiled without editing the shared
  `guide_examples_test.cpp`.
- **Documents (§3.1's six for L6-A1, plus the generated and test-author ones):** `docs/changelog.md` (an "Added and
  fixed since 2.5" section at the top — no version bump, §1.8); guide 10 (NEW section "The wall reset — one axis
  against a field wall": the explicit call (compiled listing), the request, the mount, the limits, the report and its
  verdict words, what one call does, `seatAxis` and why it is not `setPose`, the two counters, RC31's advisory
  IMPLAUSIBLE beside the explicit call, the frozen wait's lateness); guide 11 (NEW section "The wall reset (WALL)": six
  REAL lines printed by the host test plant through a scratch probe, `scratch/l6-a1/probe/lines.cpp` — word by word;
  §1.2's 21-row verdict table with every RC tag and every internal "Step 0.n" reference rewritten for a public page;
  "Choosing `maxCorrection`"; one bullet in the warnings list pointing at it); guide 14 (NEW section "The wall reset:
  what it cannot catch"); cookbook 05 (D11 only); `docs/hardware-assumptions.md` (below); `test/README.md` (the host
  field plant, for test authors); `docs/api/` + `mkdocs.yml` regenerated (`api_doc_tool.py generate`).
- **The HA register: EIGHT registered — HA-204..HA-209, HA-211, HA-212** (index rows, the preamble's note, the
  detailed entries after SER1's, the status line 178 → 186). **HA-210 is NOT registered:** §4's "Cited in code by" puts
  it in L6-B (`src/robots/chassis_table.hpp`'s provenance), and the task registers only the ids §4 assigns to L6-A1 —
  it stays in the Held line. HA-113, HA-114, HA-115 and HA-180 each gained one blast-radius sentence (§4's last
  paragraph). The Held line (HA-204..HA-215) is unchanged (its rule: a landed number may stay in its range). No new
  hardware belief beyond §4's was needed — nothing to stop for.
- **Plan documents:** `docs/roadmap.md` (a `[~]` L6 item in the M3 localization list, "host- and build-gated, NOT
  run on a robot"), `REMAINING-WORK.md` row 25's status cell, `build-order.md`'s L6 entry (a status line). **NOT
  edited, for the coordinator:** `build-order.md`'s first bold `Next:` (the one long shared line — left to the merge,
  where the coordinator restates it); the master plan's Distance row (:412 "wall cross-check" → "wall reset (L6): one
  axis against a field wall, from a side sensor") — §3.3 gave it to the PLAN commit, which did not make it (decision
  #4's amendment at :550 DID land); a plan change is the coordinator's (pace rule 2).
- **The prose review (§3.4), each grep run and its hits read:**
  1. `never snap|never-snap|nothing that reads a sensor|caller seed|relocalize` over `include/` and the public docs —
     ~50 hits read. STALE and FIXED: guide 15's glossary "snap" ("shulib never does it, in position or in heading" —
     the wall reset now seats one position coordinate from a sensor: reworded to the correctors, plus the one
     caller-invoked exception) and guide 12 :149 ("visible teleports should not happen" — a wall reset's seat is a
     deliberate jump its WALL line reports: one sentence added). **A delta:** guides 12 and 15 are L6-B's / L6-A2's
     documents in §3.1; A1 touches one sentence in each because A1's code made them stale (rule 4). Read and still
     true: the localizer's banner (:43-47, plus the stopped run's new paragraph :48-53), its class doc (:177-178),
     `complementary_fusion.hpp`, `ekf_fusion.hpp` (:58 is about the heading — no sensor seat exists for it),
     `gps_corrector.hpp`, `correction.hpp`, guide 03 :118 ("Corrections never snap" — about correctors; A2 adds the
     guide 03 section), guide 11's CLMP and heading-audit lines, guide 14 :103 (yaw), the master plan (decision #4's
     amendment present at :550-554; :409/:627/:638/:645 are the tag/feature relocalize — unchanged by D2), the roadmap.
  2. `wall reset|re-seed|reseat|startAt` — every hit read: guide 09's `startAt` row, guide 16/17's "no `startAt` in
     a registered auton", cookbooks 02/03/04 (start seeds, true), cookbook README's row (still true), guide 14 :440
     ("a wall reset would see it" — true, now host-gated), and the new sections. Nothing stale.
  3. `distance sensor|side sensor|IDistance` — the claim §3.4 names (`hal/distance.hpp` :7-9, :18-19) fixed above;
     guide 03 :92-94 ("a one-dimensional position fix … cannot tell a wall from a robot passing by" — true, and A2's
     section expands it); the faq's 393-inch entry, guide 06's in-band impostor, guide 13 :172 — true.
  4. `about −70|70 in|140` — guide 02 :17-18 "about −70 to +70" is true ("about"), not touched (the brief withdrew it).
  5. `HA-20[4-9]|HA-21[0-2]` — both directions: the code cites HA-204..209, 211, 212, each with its register row;
     every registered row has a code site; HA-210 is cited nowhere and registered nowhere (held for L6-B).
- **§7's own greps:** no `seatAxisProto|l6proto|l6field|PROBE_`; no `setPose|reseatHeading|headingBias` outside comments
  in the two wall-reset headers (a trailing "never setPose" comment on the seat's line was reworded so the grep is
  empty); `field_plant.hpp` names nothing in `localization/` (two banner comments reworded so the grep is empty); the
  pure headers include nothing from chassis/motion/sequence/competition; `maxCorrection{0.0}` is its only default
  (and the report's echo field); no `PROVISIONAL (A4` without an HA id; C7 removability empty; **no RC tag** in a
  guide, cookbook, header or compiled listing (four header comments that cited RC35/RC41/RC47 reworded);
  "simulator" absent from guide 03 and cookbook 05.
- **The brief's file:line citations re-checked at `d8fdb8b` (landmine 18 — only the ones A1's code and documents rest
  on):** holding — scheduler :761/:798/:830 (`loopMonitor_.reset()`), :840 (the timed-out line), :1110 (the pose
  guard), :660 (the shadow context); `plausibility_guard.hpp` :63/:69/:116; `line_format.hpp` :49 (384);
  `ekf_fusion.hpp` :442 (resync)/:450 (`ux = px − lastX_`); `drive_encoder_odometry.hpp` :219 and
  `pilons_odometry.hpp` :203 (`prevHeading_`); `src/robots/chassis_table.hpp` :124; the master plan :547 and :412;
  `auton_registry.hpp` :215; `run_guard.hpp` :95; the localizer's :155 (driftHorizon) and :204 (kMaxCorrectors).
  **Moved:** the localizer's STEP 2 (:266-281 → :246-265 at the base, `update()` at :240); `hal/pros/distance.hpp`'s
  hold (:70-76 → `faultedReads_ += 1` at :93/:106/:117); `calibration/drivetrain_calibrator.hpp` :274 → :166
  (`kDegreesToRadians`); the master plan's `relocalize(featureSource)` :639 → :645; `routine.hpp`'s `then()`
  ~:426-462 → :489; `auton_registry.hpp` `effectiveStart()` :266-275 → :274. None changes what the brief concluded.

### 19:12 — every delta from the brief's listing and test list (kept, each with its reason)
*The stopped run wrote the four headers and the Localizer change before 15:01 without logging; this resumed run read
them in full, kept them, and records here where they differ from §1.2/§1.3's listing (the brief: "the executor may
rename only with the coordinator's agreement" — nothing in the listing was RENAMED; everything below is ADDED or
re-shaped behind the same names).*
- **`WallSample` gained `at`** (the sample's time from the window's first) — rule 7's "≥ `window`" is then judged in
  the PURE evaluator, from the samples alone.
- **`WallResetReport` gained the line's every number:** `side`, `maxCorrection` (echoed), `windowSpan`, `offSquare`,
  `beamHalfWidth`, `keepOutMargin`, `alongUncertainty`, `cone`, `headingTolerance`, `speedSeen`, `turnSeen`,
  `sampleIndex` / `sampleDistance` / `sampleConfidence`, `detail` (BadRequest's field message, KeepOut's span name,
  SensorAbsent's sentence), `cleanFound` / `cleanFrom` / `cleanTo` (RC46's "nearest clean span"), `faultChannel`
  ("faults n/a"). The listing's fields all kept.
- **Added pure helpers** (each documented, each tested): `readingAccuracy` (HA-204's max), `effectiveCone`,
  `wallResetProblem` (BadRequest's message — "maxCorrection UNSET: say how far this wall may move x"), `WallAim` /
  `aimAt` (rules 2-4's numbers computed once), `keepOutActive` (the kinds), `sampleVerdict` (rule 6), `wallStill`,
  `describeWallReset`, `WallSampler` (one tick at a time — the per-sample read-fault rule lives here), `appendWallResetLine`,
  `wallSideWord`, `axisLetter`, `alongAxis`, `WallSensors::side()`, `WallResetLog::record()`. §1.2's "one sampler (a
  template over the wait)" is `chassis::detail::runWallReset<Wait>` over the pure `WallSampler`: the explicit form
  passes `chassis.waitUntil`; A2's everyday form will pass the guard's `waitFor` (a `false` return = CUT, already
  handled — Cut is unreachable from the explicit form).
- **`preCheck` runs BadRequest first**, then NotSquare, KeepOut, BeamLow; the call checks sensor → mount → live → motion
  before it (rule 1's order). In the explicit form a null sensor is **`SensorAbsent`** (its handle's `absence` sentence);
  `NoSensor` (no slot at all) is the everyday form's — T4's "NoSensor" case is SensorAbsent here.
- **`axisErrorBound`** evaluates the 32 corners of (reading ± accuracy) × (forward ± tol) × (left ± tol) × (facing ±
  tol) × (heading ± tolerance) through the exact geometry; T2's 20 000 draws found no draw past it (worst/bound ≥ 0.5,
  so not loose).
- **The keep-outs' order**, most specific first (Toggle body, brackets, Loaders, start pieces, corners, Load Zones), so
  the log names a Loader "Loader", not the Load Zone that contains it (T8 S4a' pins "Loader at along 58.8").
- **The Loader centre:** the header computes 58.764 in from A10's (3275.8 − 1783.2) mm; the brief writes 58.77 (the
  plant keeps 58.77 — the two agree to 0.006 in, T14). **The manual version** (the brief's header asks): a search on
  2026-10-29 showed v2.1 listed (released 2026-10-08, "Clarification / minor update"); its text answered 403, so the
  numbers are v2.0's and whether v2.1 moved any is UNVERIFIED — said in `override_walls.hpp`'s banner.
- **The window stops at the first refusing sample** (no later sample can change a rule-6 verdict). So T9's "every
  sample 9999 returns within windowMax + one tick" returns at its FIRST sample (pinned: ≤ one tick), and **M23 is
  restated** against a window that cannot fill (`minSamples` 64 > the 51 samples 0.5 s holds at 10 ms): it must end
  at `windowMax` by the clock (0.50 s), and with the bound ignored it ran to the buffer's 64 samples — RED.
- **`kWallResetWaitBackstopSeconds`** (1 s past `windowMax` on the wait's own timeout) — unreachable by construction
  (the predicate ends the window at `windowMax`), a code-level backstop, not an HA entry; T9 pins that no `SCH` line
  is ever written.
- **T8's scenarios on THIS plant** (an independent model; P2's differed): S7a and S8 refuse **OUT OF RANGE**, not NO
  RETURN — this plant models the floor, and the cone's lower rays meet it ~34 in out (P2's floorless model said no
  return); refused either way. S7b seats **2.0 in** wrong at a 7-in parking (P2's 2.96 was at 8 in — the error is the
  parking distance minus the robot's 5 in). S5b/S5c run 10.5 in up (P2's 9 in is now past RC34's ceiling at that
  reading). RC37's clear panel parks 6 in off: gaps 1 / 2 in SEAT ~1 / ~2 in wrong (pinned ±0.15), 3.5 / 4 in refuse
  DISAGREES; a 3-in gap under `maxCorrection` 3 sits ON the agreement edge and the noise decides — not pinned.
  RC41's mid-Toggle SEATED case uses a lens 10 in up with a true and typed 12° cone: at 10.5 in with ±18° this plant's
  1-in overhang (HA-208's invented part) is inside the cone's top rays — which is exactly why only a MEASURED
  `toggleChecked` lifts the body.
- **T16's plant half, re-based (RC36 meets RC33):** with RC36's Load Zones on the season red wall, every beam near a
  Loader is kept out whatever the drift, so Q2's case A runs on an OWN red line carrying only the Loader spans (the
  draft's model): the draft widening (drift rate 0) SEATS wrong (> 0.5 in here; Q2's model said 1.57), and after 24 or
  48 in of travel the drift term refuses KEEP-OUT. Added: the same drift case beside a Toggle BRACKET on the season
  left wall (measured cone, `toggleChecked`; 55 in of travel), and "not over-refused right after the perpendicular
  axis was seated" (a 48-in leg refused, then a seat of x, then SEATED).
- **NEW test files beyond §1.9's five:** `test/wall_reset_rig.hpp` (the shared tank rig) and
  `test/wall_reset_example_test.cpp` (guide 10's compiled listing). `test/CMakeLists.txt` NOT touched (it globs
  `*_test.cpp`).
- 19:16 — guide 14's precision bullet re-checked against the library's OWN bound (a scratch probe,
  `scratch/l6-a1/probe/bound.cpp`, readings 4–10 in, the left wall, mount ±0.25 in / ±2°): f 0 — 0.862–0.877 square,
  0.900–0.936 at 3° skew, 0.923–0.974 at 5° before turning; after 180°: 0.929–0.982 square (PASSES), 1.002–1.096 at 3°,
  1.049–1.170 at 5° (refused); after 450°: 1.109–1.584 (refused); f 5 — 1.013–1.051 from the start (refused). The draft
  of the bullet said "refused after about 180° of turning" unconditionally — true only off square (the brief's Q1
  number is the worst corner over a believed skew ≤ 5°); the bullet now gives both.

### 19:33 — the src/ build gate
- `python3 tools/src_build_gate.py self-test` → "src build gate self-test: OK (235 detector cases, each against a real
  build)"; `check` → "src build gate: PASS (all 7 builds link; …)", every variant 11/11 TUs, 0 disallowed warnings.
  Afterwards `git status -- src/ Makefile tools/` is empty (no self-test plant left behind). (G3a Part 1's worktree ran
  its own self-test at the same time — a different directory, checked by `/proc/<pid>/cwd`.)

### 19:41 — the final gates, on a CLEAN rebuild (`cmake --build --target clean`, then 245 objects compiled)
- The binary was removed and the briefing regenerated against its absence BEFORE the build; every test object
  recompiled from the restored sources (the mutation driver checked each restore byte-for-byte by sha-256).
- `python3 tools/suite_run.py green` → **suite run: green — 2690 cases, 2806298 assertions** (the base: 2,626 cases;
  +64 new: 63 `L6 …` cases and `guide-10w`; `cookbook-05b` changed). `-tc='L6*,cookbook-05b*,guide-10w*'` → 65 / 65.
- Then `briefing_status.py generate` against that fresh binary → `check`: "generated block current" (2,690 cases,
  231 headers, 186 registered, next free HA-216, L6 an open chunk, 19 `[~]`).
- `api_doc_tool.py`: self-test OK; check-coverage "5167 public entities across 190 headers, all documented";
  check-fresh rc 0; check-examples "621 quoted lines, all verbatim (10 source files)"; check-removability "no public
  doc references docs/internal/". `doc_staleness_audit.py`: clean (31 live docs).
- The ARM header gate over all 231 headers in one TU: rc 0. The CI guards (no `pros/` outside `hal/pros`; no
  `shulib/sim/` from the core): empty. `src_build_gate.py check` re-run on the final tree: PASS (7 builds).
  `ctest -R compile_refusal`: "100% tests passed, 0 tests failed out of 251" (116.7 s).
- §7's brief-specific greps all as required (above). `git status -- src/ Makefile tools/`: empty. Nothing committed.

### The DoD (§2.3) for L6-A1, honestly
- [x] This log created first (14:39) and appended through the stop and the resumption.
- [x] Every new public member documented (the API doc gate); every belief cites HA-204..HA-209, HA-211, HA-212.
- [x] The suite green under the cap; all 36 of A1's mutations RUN and seen RED (a test assertion each), restored.
- [x] The ARM header gate, the CI guards, `src_build_gate.py` self-test and check.
- [x] The doc gates; the briefing regenerated against a fresh build only.
- [x] §3.1's documents in the same change; the prose review logged above.
- [x] The roadmap `[~]`, REMAINING-WORK and build-order: "host- and build-gated, NOT run on a robot".
- [ ] **Owed by the workflow, not this run:** three reviewers, their fixes, a fresh verifier, the coordinator's
  checks and the commit. **Nothing here has run on a robot.**

### L6-A1 review findings and their disposition

*Three reviewers (correctness, tests, docs) returned 30 findings on 2026-10-29. A first fix run worked through
them from ~20:05 and was STOPPED at ~21:10 (the account's 5-hour usage limit); the laptop was off overnight. That
run wrote NO entry here — its work is recorded below from the tree and its scratch (`scratch/l6-a1/fixmut/`,
`fixprobe/`, `sweep/`), and every result it reached is re-run on a fresh binary, not taken from its files.*

#### 08:33 (2026-10-30, `date`) — RESUMED: the audit, and a LIVE MUTANT found and restored
- **`include/shulib/localization/wall_reset.hpp` held a mutant** — the stopped run's mutation **S2** (the
  read-fault count's first read moved AFTER `distance()`, a two-line swap in `WallSampler::step`). Its driver
  was killed while S2's build was compiling (`fixmut/S2.build.log` ends "Terminated"), so the `finally` that
  restores never ran. The coordinator's post-restart audit did not catch it (the swap reads like an edit); this
  run found it by comparing the file against `fixmut/bak/S2__…` (identical to S1's backup, both taken from the
  20:21 file): the ONLY difference was S2's swap. Restored byte for byte from that backup, timestamp bumped
  (08:33:34), `cmp` clean.
- Every other mutation backup under the scratch folder compared with its live file (108 backups): the stopped
  run's R1/R2 backups are identical to the live `wall_reset_call.hpp`; every reviewer backup (19:52–20:01)
  differs from its live file only by the fix round's later edits (diffed by hand: the counters' steps, the
  sampler's displacement rule, the BadRequest window and evaluator check, the NaN guard, `faults moved`, doc
  rewordings) — no other mutant. `git status`/`git diff -- src/ Makefile tools/`: empty — no gate plant.
- `build/test/shulib_tests` removed, configured, full `cmake --build` (6 objects: the restored header's
  dependents), then `suite_run.py green`: **2708 cases** green. Every later result in this section is from a
  binary built after the restore.
- **The driver now marks a live mutant:** `fixmut/driver.py` writes `fixmut/APPLIED.json` (the mutation's id and
  backups) before it edits and deletes it only after the byte-checked restore — a resumer checks for that file
  first. It also records how many objects each mutant's build recompiled, and a mutant GREEN on the L6 filter is
  re-run on the FULL suite before it is called a survivor.

#### The findings (deduplicated: 30 → 24), each re-checked; the fix is the stopped run's unless marked "resumed"
1. **The turn counter added up IMU heading NOISE (correctness, major) — REAL, fixed at the root.** Re-checked: the
   counter summed every tick's |Δθ| (`localizer.hpp` :295-296 then), so HA-21's 0.05° white noise read ~5.6°/s on a
   robot that did not move. **The same flaw was in the travel counters** (found by the fix round, not the
   reviewers): on tracking wheels 3–4.5 in off the turning centre the noise moves the odometry a few thousandths of
   an inch a tick, back and forth, and a per-tick sum read ~25–50 in of "travel" a minute — a still robot's reset
   refused KEEP-OUT after a minute at 0.1°. **Fix:** both counted in STEPS from an anchor —
   `Localizer::kTurnCounterStepRadians` (1°: the raw heading's |wrapped| excursion is added, and the anchor moved,
   once it reaches a step) and `kTravelCounterStepInches` (0.5 in of the odometry's own summed path). A turn or a
   leg counts in full but for its last step (a reversal can lose up to two steps; a wobble that never strays a
   step from the anchor is not counted — the step is the noise floor: said in the accessor's doc, the last clause
   added at the resumption). The drift term (`distanceSinceCorrection()`) still sums every tick, unchanged.
   **Tests:** `wall_reset_plant_test.cpp` "a robot standing still under HA-21 heading noise: the counter stays ~0
   and it SEATS, at 0.3 s and 60 s" (the host plant's hostile IMU at σ 0.05° and 0.1°, HA-20's drift and HA-22's
   rate noise on, the rig's offset wheels; seeded as a FIELD start); `localizer_seat_axis_test.cpp` "the turn
   counter ignores jitter under its 1-deg step and counts a slow turn" (±0.4° for 20 s reads 0; 0.9° reads 0, 1.1°
   reads 1.1; a slow 90° turn reads 88.8–90°), "travelSinceSeat ignores back-and-forth jitter inside its 0.5-in
   step", and the ±10° wiggle still reads exactly 200° (M35's case, kept). Mutations N1, N1b, N1c, N1d, N2, N2b
   below. **A NEW HARDWARE BELIEF, for the coordinator:** the steps assume (a) two readings of a still IMU's heading
   never differ by a full degree (the step is 20 × HA-21's σ, 14 σ of the difference of two readings) and (b) a
   still robot's odometry path never strays 0.5 in from a point it passed. Recorded in **HA-211**'s Claim and Settle (an id §4 assigns to L6-A1: "a still-robot IMU and
   odometry capture, 60 s, the robot untouched — both counters must stay at a step or two") and one sentence in
   HA-21; NOT registered under a new id (the task registers only §4's ids) — **the coordinator decides whether it
   deserves its own (next free HA-216).** On the host test plant still nothing is counted at FOUR times HA-21's
   noise, and under 8° a minute with HA-20's drift added; whether real noise is white or integrated (HA-173) does
   not change that while its size is near HA-21's.
2. **Two of the call's gates untested through the call — NOT LIVE (S26 / R1) and the turn counter's wiring (W1 / R2)
   (correctness major; tests major ×2) — REAL (a coverage gap; the code was right).** Tests added in
   `wall_reset_gates_test.cpp`: "L6 T4 an estimate that is not live refuses NotLive: no time, no read, one line, no
   throw" (an IMU still calibrating: the clock unmoved, zero reads, one Warn line, the Localizer untouched) and "L6 T4
   the call's heading tolerance comes from the turn counter: turned 540 deg refuses imprecise" (with
   `rep.headingTolerance == headingTolerance(loc.turnedSinceHeadingSeed(), limits)` exactly; with the heading
   re-seeded, the same parking SEATS). R1 and R2 RED below.
3. **Stillness refused every reset as MOVING under heading noise on offset tracking wheels (correctness, minor) —
   REAL, fixed.** Rule 5's speed is now the estimate's DISPLACEMENT over the trailing `window` per that time
   (`WallSampler::step` takes the pose and the IMU's turn rate; the turn rate is still judged every tick). **A
   deviation from the brief's rule 5** ("|v| and |ω| within … on every tick"), kept for this reason: one tick's
   velocity is a finite difference of the fused pose, which noise about a fixed point shakes at 1.1–1.4 in/s while
   nothing moves; the displacement is the quantity the gate exists to bound (how far the robot moved while it
   sampled) and does not add noise up. Recorded in the header's rule list, `WallResetLimits::stillSpeed`'s doc,
   guide 10's limits paragraph and HA-211. Tests: "L6 T3 stillness: a pose jittering about a fixed point is still; a
   steady drift is not", "L6 T3 stillness: 0.99 / 1.01 in/s and 2.9 / 3.1 deg/s; a push late in the window is seen",
   and the plant's still-robot case (`speedSeen < 0.25`). Mutations N3, N4, N9, N10 (resumed), N12 (resumed), N13
   (resumed).
4. **Guide 14's "refused 3° or more off square" after 180° (correctness, minor) — REAL, fixed:** "refused
   `imprecise` from about 2° off square at a 7-in reading (from 0.5° at 10 in, 3° at 4 in)". Re-verified at the
   resumption with the library's own `axisErrorBound` (`fixprobe/bounds.cpp`, rebuilt on this tree): after 180°
   (tolerance 5.60°), 7 in: 1° 0.987, 2° 1.018 (refused); 10 in: 0.5° 1.001 (refused); 4 in: 2° 0.978, 3° 1.002
   (refused). Its other numbers too (`fixprobe/bounds3.cpp`, resumed): 0.862–0.974 before any turning (skew ≤ 5°),
   1.109–1.263 after 450° (refused).
5. **The median's ceiling check ignoring the bias (R5 / S1) and the skewed footprint's larger side (R4 / S4)
   (correctness minor; tests minor) — REAL (coverage).** Tests: "L6 T3 beam height with a bias: the median minus the
   bias is judged, at its exact edge" (rangeBias −0.25, exact binary values) and "L6 T3 keep-out: a skewed beam's
   footprint applies its LONGER side both ways" (4.9° skew, either side). S1, S4, R4 RED below.
6. **T16's "the counters change no existing observable" could not see a leak (correctness minor; tests minor) —
   REAL.** Renamed to what it checks — "reading the counters has no side effect, and the boot fold keeps them at 0"
   — and the claim's evidence recorded HERE (the test's comment points at this entry): a golden trace of the
   Localizer's EXISTING observables (x, y, heading, vx, vy, quality, quality class, the drift term, the heading
   bias; 17 digits) over one scripted 15-s drive with the hostile IMU (noise, drift, a 2-s calibration — the boot
   fold), a `setPose` and a `reseatHeading` mid-drive, over ComplementaryFusion (`fixprobe/trace.cpp`) and EkfFusion
   (`fixprobe/trace_ekf.cpp`). **Re-run at the resumption from a fresh `git archive d8fdb8b` and from this tree:
   1,500 ticks each, BIT-IDENTICAL for both fusions** (`cmp` clean). Plus the suite: every one of the base's 2,626
   cases passes unchanged.
7. **A `window` longer than the 64-sample buffer spans always ended TOO FEW SAMPLES (correctness, minor) — REAL,
   fixed:** `kMaxWallWindowSeconds` = 0.6 s; a longer `window` is BadRequest naming the field ("limits.window must
   be at most 0.6 s (the 64-sample buffer spans 0.63 s at 10 ms)"), and `window` 0 is now refused too (it was
   accepted). Rows in the BadRequest table (0.6 accepted, 0.8 refused, 0 refused). N7, N7b.
8. **`axisErrorBound` returned 0 for a NaN tolerance (correctness, minor) — REAL, fixed:** a non-finite corner gives
   +infinity (refused IMPRECISE). And the root of the reach: `evaluateWallReset` now refuses BadRequest itself (it
   used to clear `detail` and judge an unusable request). Tests: "L6 T2 a non-finite tolerance makes the error bound
   infinite, never 0"; every BadRequest row also runs the evaluator. N6, N5.
9. **T5 never faulted the FIRST device read (tests, major) — REAL (coverage; the code was right — and the stopped
   run's S2 mutant was this very swap).** Test: "L6 T5 the read-fault rule: a count that moves inside distance() of
   the last sample refuses" (sampleIndex 4). S2 RED below.
10. **A ReadFault line ended "| faults ok" (tests, major) — REAL, fixed:** it ends "| faults moved"; a per-sample
    refusal's tail is "ok" or "n/a" by the channel. Guide 11 says what the tail means per verdict. N8.
11. **T9's numbers, the KeepOut clean band and guide 11's six lines ungated (tests major; docs minor) — REAL.** NEW
    `test/wall_reset_line_test.cpp`: "guide 11's six WALL lines, byte for byte" (each from a report the library's own
    pure functions computed), "every verdict's line, byte for byte; the faults tail per verdict", and "guide 11
    quotes exactly the six pinned WALL lines" (reads the chapter: a format change turns red until it is re-quoted).
    Guide 11's section names the file. Clean band: "L6 T3 keep-out: the nearest clean band, between spans and past
    an own line's ends". S21, S21b, S24, S24b.
12. **Most BadRequest clauses untested; a REVERSED span accepted silently (tests, major) — REAL (coverage).** "L6 T3
    BadRequest: every clause refuses its own field by name, at its exact edge" — 57 rows, one bad field each, every
    clause, the angle edges typed exactly (compared in radians against `N * kWallDegToRad`, the value a robot file
    types: a degrees round trip gave 29.999999999999996 for 30 — fixed in `wallResetProblem`). S23a–S23e.
13. **T2's draws could not see the facing / forward-tolerance / bias-corrected-accuracy terms (tests, minor) — REAL.**
    "L6 T2 the error bound is attained at a corner, one tolerance at a time" (the plant's TRUE mount, heading or
    reading error at each corner; the bound attained to 1e-9). S7, S8, S11.
14. **The turn counter's boot boundary unpinned (tests, minor) — REAL.** "L6 T16 the turn counter never counts across
    the boot boundary, seeded while booting or not" (a 70° jump between the last closed and the first open tick must
    read exactly 0). L1, L2.
15. **`WallResetLog::record` and `WallSensors::side` untested (tests, minor) — REAL.** "L6 T10 WallResetLog counts and
    keeps the last; WallSensors::side picks the named side". S28, S29.
16. **Guide 10's "frozen" banner covered the wall reset (docs, major) — REAL, fixed:** the banner names three
    exceptions (the driver loop, the wall reset, the mechanisms); the section opens with "The banner at the top of
    this chapter does not cover this section" (in no Freeze Register row; grows additively through L6-A2); "Covers:"
    and the examples paragraph name it and `guide-10w` (that paragraph re-wrapped at the resumption).
17. **Stale IMPLAUSIBLE prose (docs, minor) — REAL, fixed:** `diag/plausibility_guard.hpp`'s banner (one deliberate
    exception: a wall reset's seat), HA-56's claim (the caller-made exception), guide 11's IMPLAUSIBLE row (one known
    benign cause). Grep 1 of §3.4 re-read at the resumption (below).
18. **Guide 10's "5 in forward is refused from the first reset" unqualified (docs, minor) — REAL, fixed:** "with the
    default ±0.25-in mount tolerance and VEX's published accuracy … (a mount measured to ±0.1 in and a measured
    accuracy of ±0.3 in would bring it back under the limit)". Re-verified (`fixprobe/bounds.cpp`): defaults
    1.013–1.148 in, measured 0.560–0.694 in (readings 4–10 in, skew 0/±5°).
19. **Guide 14 and HA-208's numbers (docs, minor) — REAL, fixed:** (a) "(3.0 in at 8 in)" dropped — the host plant
    pins 2.0 at 7 in only; (b) "HA-204 to HA-209, HA-211 and HA-212"; (c) HA-208's blast radius now cites T16's
    bracket case as what it is (a beam whose ESTIMATE was 5.5 in off along the wall seated > 0.5 in wrong at a
    bracket with the drift term off; refused KEEP-OUT with it on); (d) the Toggle body under a high beam added to
    guide 14's first bullet (no wrong-seat distance pinned for it, said so).
20. **Internal "Step 0.n" / "the L6 brief" in public API docs (docs, minor) — REAL, fixed:** `SideMount`'s fields say
    what to measure (taped readings at known distances, the beam's half-angle, a reading under a real Toggle);
    `WallResetRequest`'s and `WallSample::faultMoved`'s docs and the call's "frozen wait" heading lost their brief
    pointers. At the resumption: the banner's section heading "(the L6 brief's landmine 1 …)" → "(never claim
    otherwise — guide 14 lists it too)". KEPT, with reason: the file banner's provenance line ("the L6 brief, the
    team lead's answers of 2026-10-22") — the provenance form 58 generated pages already carry ("the COMP brief's
    …"); `sim/field_plant.hpp`'s two brief references (a test-plant header, not in the generated reference).
21. **Plan-commit items missing at the base (docs, minor) — REAL, the COORDINATOR's (pace rule 2), NOT edited here:**
    the master plan's Distance row (:412 "wall cross-check"); decision #4's status cell (:547 — add "amended
    2026-10-22 (L6, D2 (a)), see note"; the amendment itself is the blockquote at :550-554); the bench worksheet's
    Station H.4–H.6 (Step 0.3–0.6; Station H ends at H.3) — until then the register's "L6 Step 0.x" Settle lines and
    "Step 0.1–0.5 at the NEXT bench session" have no worksheet station to run from.
22. **Manual v2.1 recorded only in the header banner (docs, minor) — REAL, fixed:** HA-207, HA-208 and HA-212 say
    "manual v2.0's drawings; v2.1 (2026-10-08) exists and was not readable on 2026-10-29 — UNVERIFIED"; at the
    resumption also HA-206 (the code strip's place is A8's) and the changelog's `override_walls.hpp` line.
23. **The NotFieldStart line prints a fixed sentence, not the start mode (docs, minor) — REAL, an unlogged delta;
    HANDED TO L6-A2.** `WallResetReport` has no start-mode field and nothing in A1 can fill one (the verdict is the
    everyday form's): A2, which owns the verdict and has the context, adds the mode to the report and the line
    ("start RELATIVE" / "start unset" — T17 distinguishes them) and re-quotes guide 11's row (its "numbers" cell is
    "—" today). Not added now: a field no A1 path can set would be dead code.
24. **The citation re-check narrowed to a subset (docs, minor) — REAL, finished:** below.

#### The citation sweep — every file:line the brief cites, ceb17c0 → d8fdb8b (finding 24)
- **Method** (read-only; scripts kept): `scratch/l6-a1/sweep/sweep.py` (the stopped run's; re-run at the resumption,
  report identical) finds all **416** `file:line` citations in the brief and follows each through `git diff
  ceb17c0 d8fdb8b` (difflib over the cited file): 238 in a file unchanged since ceb17c0 and 24 in a changed file
  at unmoved lines — HOLD; **33 MOVED** (same text, new number); **7 CHANGED**; 80 outside the repository (the
  brief's probes, `v20.txt` — the manual's text, `imu_.h`, VIS1-A1's working tree, scratch); 41 it could not
  attribute (26 with no file named nearby, 7 past the end of the file it guessed, 1 ambiguous name, plus repo
  paths not yet at ceb17c0). `sweep/sweep2.py` (resumed) assigns those 40 their files BY HAND from the brief's own
  sentence and maps them the same way: 22 hold, 11 MOVED, 7 CHANGED.
- **MOVED** (the text is the same; only the number): REMAINING-WORK :107 → :122, :166-167 → :181-182, :169-170 →
  :188-189; build-order :1713 → :1823, :2224 → :2337, :2227 → :2340; HANDOFF :489 → :498 (its L6 entry :489-500 →
  :498-509), :632 → :641, :133-137 → :135-139; the roadmap :411 → :435, :1666 → :1690; changelog :1086-1090 →
  :1334-1338; the register :277 → :290, :959-990 → :973-1004, :3847-3852 → :3947-3952; the master plan :639 → :645;
  `match_runner.hpp` :147-149 → :158-160; `routine.hpp` :227-235 → :290-298, :241-256 → :304-319, :257-258 →
  :320-321, :426-462 → :489-750; `auton_registry.hpp` :266-275 → :274-283; `mechanism/service.hpp` :232 → :282,
  :382 → :449; guide 11 :331 → :385 (and the IMPLAUSIBLE row :342 → :396 — this tree's own additions move them
  again); guide 14 :362-367 → :380-385. None changes what the brief concluded.
- **CHANGED** (each read): REMAINING-WORK row 25 (:104/:107) and HANDOFF's L6 entry (:485) were REWORDED by the plan
  commit (row 25 is now :119, the side-sensor wording D12 asked for); build-order's phase table gained the L6 row
  and its count line — "69 chunks" at ceb17c0 (the brief's "70" counted L6 ahead) — is "**71 chunks**" at :855
  (L6 +1, ALLY1 +1), REMAINING-WORK :16 "of the phase table's 71" with it; build-order :632 (the
  first bold `Next:`, a shared line rewritten at every merge); `auton_registry.hpp` :215-229 — MECH-A4 reworded the
  `mechanisms` doc and APPENDED `AutonContext::mechanismService` (with a default, so the positional form still
  compiles) — **for L6-A2**, whose everyday form takes an `AutonContext`: the struct it reads has one more member
  than the brief saw; `routine.hpp` :5-22 — MECH-A4's: `RoutineResult` now has NINE fields (`mechanismVerdict`) and
  a delimited mechanism-step block — **for L6-C** (optional), which edits `routine.hpp`.
- **Off already at ceb17c0** (the brief's own slips, harmless for A1): the MECH brief's §2 table "(:2134)" is the
  MECH-A6 row — MECH-A5's is :2133; "the shadow context (:660-667) … (:668-671)" is `motion/motion_scheduler.hpp`
  (holds there), the brief's sentence reading as if it were `robot.hpp`; `robot.hpp` :646-696 (the mechanism
  accessors) and :740-806 / :744-750 (`describe()` at :753, "Truncates rather than overflowing" at :746) hold —
  VIS1-A1 (in review) edits `robot.hpp`, so **L6-A2 re-finds them after VIS1-A1 merges**; `device_provider.hpp`
  :217-222 is `hal/device_provider.hpp`'s `lastRefusal()` (holds).
- Nothing in this sweep touches A1's code or documents beyond what the 19:12 entry already re-checked.

#### 09:00 — the mutations: 76 (78 runs) on fresh builds — 75 RED, one equivalent; one test gap found and closed
- `fixmut/driver.py all` (the stopped run's 73 definitions — its own 37 for the review's fixes and the reviewers'
  survivors, and the build's 36 re-validated (none STALE; M35 is N1b, its line rewritten by the fix) — plus three
  added at the resumption: **N10** (stillness's reference tick never advances), **N12** (the call hands the sampler a
  constant pose), **N13** (the call hands it a zero turn rate)). Each: backup, `APPLIED.json` written, the edit
  applied exactly once, the binary REMOVED, the briefing regenerated against its absence, `cmake --build --target
  shulib_tests` (the build log's object count recorded: 3–118 objects, every dependent of the mutated file), the
  `L6*`, `cookbook-05b*`, `guide-10w*` cases under `ulimit -v 4 GiB` + `timeout 600`, a GREEN one re-run on the FULL
  suite, then the byte-checked restore (sha-256) with the timestamp bumped and the marker removed. Results:
  `fixmut/results2.jsonl`; after the run every mutated file equals its latest backup and no marker is left.
- **RED — 75**, every one by a TEST CASE'S ASSERTION (each build succeeded; the failing case named): R1 → "T4 an
  estimate that is not live refuses NotLive"; R2 → "T4 the call's heading tolerance comes from the turn counter";
  S1 → "T3 beam height with a bias"; **S2** → "T5 … inside distance() of the last sample"; S4, R4 → "T3 keep-out: a
  skewed beam's footprint applies its LONGER side both ways"; S7, S8, S11 → "T2 the error bound is attained at a
  corner"; S21, S21b → "T9 guide 11's six WALL lines"; S23a–S23e, N5, N7, N7b → "T3 BadRequest: every clause";
  S24, S24b → "T3 keep-out: the nearest clean band"; S28, S29 → "T10 WallResetLog … WallSensors::side"; S30 → "T16 a
  reset right after the perpendicular axis was seated"; L1, L2 → "T16 the turn counter never counts across the boot
  boundary"; N1, N1c → "T16 the turn counter ignores jitter …"; N1b → "T16 turnedSinceHeadingSeed sums |dtheta|
  through the wrap"; N2 → "T16 travelSinceSeat ignores back-and-forth jitter"; N2b → "T16 travelSinceSeat: grows by
  path length, setPose zeroes both"; N3, N4, N9 (+27), N10 → the stillness cases; N6 → "T2 a non-finite tolerance";
  N8 → "T9 every verdict's line"; N12 → "T8 S9 sampled while driving"; **N13** → "T8 S9b" (below); and the build's
  36 less M35 — M1–M24 (with M6b, M6c; M18b for M18), M29, M30, M32–M34, M36–M39 — each RED on its own case, as
  at 18:59.
- **N1d (the turn step 5° instead of 1°) — the build FAILED first:** the step is a documented constant, so the API
  doc gate (a dependency of the test target) said `docs/api` was stale. A gate, not a test assertion — NOT counted.
  Re-run with `fixmut/fast_one.py` (the test target's own rules, `shulib_tests/fast`: 115 objects recompiled from the
  compiler's depfiles, the doc-gate target skipped): **RED by the assertions** of "T16 the turn counter ignores jitter
  under its 1-deg step" — the constant's pin (0.0873 ≠ 0.0175) AND the behaviour (1.1° read 0; the slow 90° turn read
  86.7°, under 88.8°).
- **C3 (the sampling wait's own timeout = `windowMax`, no backstop) — GREEN on the full suite (2,708): EQUIVALENT,
  not a gap.** `MotionScheduler::waitUntil` (motion_scheduler.hpp :819-848) evaluates the predicate BEFORE the
  deadline on every pass, and the sampler's clock starts at the wait's first pass (the same `now()` the deadline is
  computed from), so the predicate always ends the window at `windowMax` first; the backstop (`kWallResetWaitBackstopSeconds`)
  is unreachable by construction, as the 19:12 entry says — kept as a code-level guard.
- **N13 SURVIVED the first pass (2,708 / 2,708) — a REAL gap the resumption closed:** nothing turned the robot while
  it sampled, and a robot turning IN PLACE moves its centre nowhere, so after the displacement rule only the turn rate
  can see it. NEW `wall_reset_plant_test.cpp` "L6 T8 S9b turning in place while sampled: REFUSED moving on the turn
  rate" (8°/s commanded: on the real code REFUSED moving, turn rate seen 8.00°/s, speed 0.000 in/s —
  `resume-probe/s9b_values.cpp`); N13 re-run on a fresh build: **RED** on S9b.

#### 09:05 — the reviewers' probes re-run on the fixed tree (`scratch/l6-a1/resume-probe/`, the sampler's new arguments)
- p4b (still, HA-21 noise only, σ 0.05 / 0.03, 0.3–60 s): the counter 0.0° throughout, tolerance 2.00°, every reset
  SEATED (was REFUSED imprecise from 45 s). p4c (σ 0.01–0.05, offset wheels): speed seen 0.01–0.06 in/s, SEATED
  (was REFUSED moving at 0.05). `fixprobe/noise.cpp` (σ 0.05 / 0.1 / 0.2, 15–60 s, with and without HA-20/22): the
  counter 0.00° and travel 0.000 in noise-only even at σ 0.2 (the per-tick sums would have read 1,341° and 127 in at
  60 s); with drift on at σ 0.2, 7.73° at 60 s — HA-211's "under 8° a minute" holds; every reset SEATED. The
  sampler's doc said the displacement reads "~0.04 in/s" — the probes read 0.05–0.09 (0.35 at σ 0.2): corrected.
- p6 (window 0.8): "limits.window must be at most 0.6 s …", the evaluator answers REFUSED bad request. s2 (the count
  moving inside `distance()`): REFUSED read fault, sampleIndex 4. line: the ReadFault line ends "| faults moved".
  p2 (bounds): NaN heading or position tolerance → bound `inf` (was 0.000).

#### 09:05 — what the resumption changed (beyond the stopped run's fixes), and the prose review of it
- `localizer.hpp`: the turn counter's doc says a wobble that never strays a full step is not counted (the step is
  the noise floor). `wall_reset.hpp`: the "What no gate here can catch" heading lost its brief pointer; the sampler's
  doc's speed number corrected. `test/wall_reset_plant_test.cpp`: S9b. `test/wall_reset_gates_test.cpp`: two unused
  locals dropped from the BadRequest table case. Guide 10: the examples paragraph re-wrapped; "a still robot's sensor
  jitter" → "IMU jitter, which shakes the estimate a few thousandths of an inch a tick" (it is the IMU, not the
  distance sensor). Guide 11: TooFewSamples' remedy also names limits that ask for more samples than `windowMax` holds
  at one a tick (that pairing is pace-dependent, so it stays a refusal at the window, not a BadRequest — the pure
  check cannot see the tick). Guide 12: the "estimate jumps" bullet says the one benign IMPLAUSIBLE after a seat.
  Guide 14: one long line re-wrapped. The register: HA-206 (manual v2.1 unread), HA-211's index row (the
  displacement rule, the counters' steps, the still-robot capture in Settle) and its claim made precise (two still
  readings never differ by a full degree; the path never strays 0.5 in). The changelog's `override_walls.hpp` line
  (v2.1). `docs/api` regenerated (`check-fresh` rc 0).
- **Prose review (rule 4), each grep run and its hits read at the resumption:** `turnedSinceHeadingSeed|travelSinceSeat|
  turn counter|travel counter` over the docs (guide 10 :521-528, the changelog, HA-211 — all say STEPS; nothing
  stale); `1 in/s|stillSpeed|stillness|REFUSED moving|moving` (guide 11's Moving row says the displacement; guide 10
  :483 and :509; the changelog; HA-211's row and claim — true); `faults ok|windowMax|too few samples|0.5 s` (guide 11
  :403-407 — the tails per verdict; guide 10 :482 names the 0.6-s cap; guide 14 :382; TooFewSamples fixed above);
  `IMPLAUSIBLE` over the guides, the register and `include/` (guide 06 :176 — the general description, true; guide 11
  :466 — the benign cause present; guide 12 :147 — the bullet fixed above; HA-56 — the exception present; the guard's
  banner and `wall_reset_call.hpp` :37-41 — present); `kTurnCounterStep|kTravelCounterStep|kMaxWallWindowSeconds` (the
  changelog and HA-211 name all three).

#### For the coordinator (the fix round's hand-offs)
- **A live mutant survived your post-restart audit** (S2 in `wall_reset.hpp`, a two-line swap — above). Found by
  `cmp` against the driver's backups; the driver now leaves `fixmut/APPLIED.json` while a mutant is in the tree.
- **A new hardware belief** (finding 1): the counters' steps — recorded in HA-211 (Claim, Settle, index row) and one
  sentence in HA-21, not under a new id; decide whether it gets its own (next free HA-216). Its settle is a 60-s
  still-robot IMU + odometry capture (both counters at a step or two) — worth adding to Step 0's bench list.
- **The plan commit's missing items** (finding 21): the master plan's Distance row (:412), decision #4's status cell
  (:547), the bench worksheet's Station H.4–H.6.
- **For L6-A2:** the NotFieldStart line's start mode (finding 23); `AutonContext` gained `mechanismService` at MECH-A4
  (the everyday form's context has one more member than the brief saw); `robot.hpp`'s cited lines re-found after
  VIS1-A1 merges. **For L6-C (optional):** `RoutineResult` has NINE fields since MECH-A4.
- **The base moved under this worktree:** `shulib-v2` now carries CAL1 Part A2a (`e5a69b5`) and IMU1 Part A
  (`6798aa5`) after `d8fdb8b`. Neither touches a file of this run's code; both edit the changelog's top (IMU1's "Fixed
  since 2.5" and CAL1's "Added since 2.5" sit where this run's "Added and fixed since 2.5" sits) and the register
  (IMU1 registered HA-213..HA-215 and amended HA-22 — no id collides with HA-204..209/211/212): whichever lands second
  rebases those two files, plus the briefing's generated block. IMU1 makes the Localizer's turn rate one value per
  tick — the rate this run's stillness gate reads every tick through `twist().omega()` — which only helps (brief §6).

#### The DoD for the fix round, honestly
- [x] All 24 deduplicated findings re-checked and dispositioned above: 22 resolved here — fixed at the root in code,
  tests or documents, finding 6 also with recorded evidence (the base-vs-tree trace), finding 24 by the finished
  sweep — and two handed on with reasons (21 to the coordinator, 23 to L6-A2).
- [x] Every fix's mutation RUN on a fresh build and seen RED by a test assertion (75), one equivalent (C3, argued from
  the source), the restore byte-checked each time; no mutant left (no marker; every mutated file equals its backup).
- [x] The reviewers' probes re-run on the fixed tree; the base-vs-tree trace re-run from a fresh `git archive`.
- [x] Documents moved with the code (guides 10, 11, 12, 14; the register; the changelog; `docs/api`); the prose
  review of the fix round logged above.
- [x] The final gates on a clean rebuild — below.
- [ ] **Owed by the workflow, not this run:** a fresh verifier, the coordinator's checks, the commit. **Nothing here
  has run on a robot.**

#### 09:52 (2026-10-30, `date`) — the final gates on the final tree, a CLEAN rebuild (`scratch/l6-a1/final3.log`)
- One comment fix first: `WallResetLimits::stillSpeed`'s doc had wrapped "PROVISIONAL (A4:" away from its "HA-211)"
  — the register's direction-1 grep (§7) flagged it on the 09:29 pass; re-wrapped onto one line, `docs/api`
  regenerated. Then everything again from scratch:
- No mutant marker; the binary removed, `--target clean`, the briefing regenerated against the absent binary, a full
  `cmake --build`: **246 objects** compiled, no warning. `suite_run.py green` → **2709 cases, 2806750 assertions**
  (the base: 2,626; this run's 82 `L6 …` cases and `guide-10w`; `cookbook-05b` changed). `-tc='L6*,cookbook-05b*,
  guide-10w*'` → **84 / 84**.
- `briefing_status.py generate` against that fresh binary, `self-test` OK (38 cases), `check`: generated block current.
  `api_doc_tool.py`: self-test OK; check-coverage "5170 public entities across 190 headers, all documented";
  check-fresh rc 0; check-examples "621 quoted lines, all verbatim"; check-removability "no public doc references
  docs/internal/". `doc_staleness_audit.py` self-test OK; the audit: clean (31 live docs).
- The ARM header gate (all **231** headers in one TU, arm-none-eabi-g++ `-Werror`): CLEAN. The CI guards: PROS-free,
  sim-free. `src_build_gate.py self-test`: OK (235 detector cases); `check`: PASS (all 7 builds link);
  `git status -- src/ Makefile tools/` empty afterwards. `ctest -R compile_refusal`: 251 / 251 (251 TUs, 251 CTests).
- §7's own greps (`scratch/l6-a1/sec7.sh`, `sec7_final3.log`): every "expect empty" empty — `hal/distance.hpp`
  differs only in comments; no `src/`/`tools/`/`Makefile` change; no scratch names; no `setPose`/`reseatHeading`/
  `headingBias` outside comments in the wall-reset headers; the plant names nothing in `localization/`; the pure
  headers include nothing from chassis/motion/sequence/competition; no untagged `PROVISIONAL (A4`; C7 clean; no RC tag
  in a guide, cookbook, header or listing; no "simulator" in guide 03 / cookbook 05. `maxCorrection`: the request's
  `{0.0}` UNSET default, the report's echo field and its copy — no other default. HA-204..209, 211, 212 each cited in
  code (HA-210 held for L6-B). The spot check: `seatAxis`'s body writes no drift term, no heading bias, no `odom_`.
- This log was appended after those doc gates; they were re-run once more after this entry (below). Nothing
  committed. **Nothing here has run on a robot.**
- 09:55 — after this entry: `briefing_status.py check` current, `doc_staleness_audit.py` clean, the four
  `api_doc_tool.py` checks rc 0. The audit noticed the binary was OLDER than headers the src gate's self-test had
  planted and restored (content identical — `git status` shows no change outside this run's files — but timestamps
  bumped), so the binary was removed, the briefing regenerated against its absence, the tree rebuilt (246 objects),
  and `suite_run.py green` re-run: **2709 cases, 2806750 assertions**; the briefing regenerated against that binary
  and `check` current; the audit clean with the binary trusted; no file newer than the binary. Nothing committed.

### Verification-round findings and their disposition

*A fresh verifier returned two minor findings on 2026-10-30. Each was re-checked on this tree, not taken from the
verifier's files. Scratch: `scratch/l6-a1/vround/` (the probe, the mutation driver and its definitions, the build
and test logs, `results.jsonl`, `final.log`, `sec7.log`). Started ~14:20; this entry was written at 15:14 (`date`).*

#### 1. The sampler's two non-finite guards were untested: REAL (a coverage gap; the code was right)
- **Re-checked.** `WallSampler::step` (`wall_reset.hpp` :1164-1169, at :1161-1166 before this round's doc lines)
  keeps the window's worst speed and turn rate with `std::max`, and `std::max(a, NaN)` returns `a`. Without the
  guards, a NaN tick is judged still, a sample is taken, and the window runs on to `windowMax`. Through
  `resetFromWall` this is close to unreachable: the Localizer turns a non-finite rate into 0 (`localizer.hpp` :416;
  I checked this line, not the verifier's claim that a NaN pose is refused in preCheck). But `WallSampler` and
  `evaluateWallReset` are public, and L6-A2's everyday form drives the sampler.
- **Test added** in `test/wall_reset_gates_test.cpp` :304: "L6 T3 stillness: a non-finite turn rate or pose ends
  the window AT that tick, judged Moving". It has nine sampler rows: NaN, +inf and −inf turn rates; NaN x, NaN y,
  +inf x and −inf y; and NaN on the FIRST tick, which is the window's reference, for both the turn rate and x. Each
  row must:
  - end the window AT the bad tick, with no sample taken there;
  - give a worst value of exactly +infinity, with the other channel exactly 0;
  - make `wallStill` false;
  - get `evaluateWallReset` to answer Moving, with `speedSeen` / `turnSeen` +infinity.

  The test also hands the evaluator a NaN twist directly (in vx, vy and omega) and expects Moving each time; the same
  window with a still twist SEATS. The test's comment says the ±inf rows pass with or without the guards, because
  `std::max` keeps an infinity. The sampler's doc gained one sentence: a NaN pose or turn rate counts as +infinity.
  The changelog's `wall_reset.hpp` entry gained one clause.
- **Mutations.** Each mutation:
  - got a FULL `cmake --build`, with the binary removed and the briefing regenerated against its absence first;
  - ran the `L6*`, `cookbook-05b*` and `guide-10w*` cases under `ulimit -v 4 GiB` + `timeout 600`;
  - was restored byte for byte, checked against its sha-256, with its timestamp bumped;
  - left no `APPLIED.json` marker behind.

  All four ran twice. The FINAL runs, on the final tree, each turned the new case RED by its own assertions:
  - **G1** (the verifier's V2: the speed guard dropped; 118 objects) RED on the NaN x, NaN y and NaN-x-on-the-first-tick
    rows. The window ran to tick 16 with 17 samples and the reset SEATED.
  - **G2** (V3: the turn guard dropped; 6 objects) RED on the two NaN turn-rate rows.
  - **G3** (`wallStill` rewritten NaN-blind, `!(x > limit)`; 6 objects) RED on the six evaluator-direct assertions
    only. The sampler rows still pass, because the guards turn the value into +infinity first.
  - **G4** (the speed guard sets `DBL_MAX`, not +infinity; 6 objects) RED on the exact-infinity assertions (:351,
    :359).

  The FIRST runs, before the last comment edits, were also RED ×4 on the same assertions. One re-run's builds failed
  at the API doc gate, because a `//` line I added in `localizer.hpp` shifted declaration lines and left `docs/api`
  stale. That failure was not counted: `docs/api` was regenerated and all four were re-run.

#### 2. The still-robot numbers were better than the host test plant shows: REAL, and wider than the verifier found
- **Re-measured across seeds.** The verifier ran the rig's one seed. I wrote a probe, `vround/still_numbers.cpp`,
  with a probe-only copy of the rig in `vround/inc/` whose only change is that the harness seed can be varied. Seed 1
  reproduces the verifier's numbers: at σ 0.3° for 120 s, 270.4° counted, refused imprecise, bound 1.030.
  - **(a) Rule 5's speed on a still robot** (600 resets: ten seeds, 60 resets each, 0.5 s apart). Noise-only and the
    full hostile IMU agree to three decimals. Values are min / mean / max in in/s:

    | σ (× HA-21) | min | mean | max |
    |---|---|---|---|
    | 0.05 (×1) | 0.036 | 0.082 | 0.172 |
    | ×2 | 0.072 | 0.164 | 0.344 |
    | ×3 | 0.108 | 0.246 | 0.516 |
    | ×4 | 0.145 | 0.328 | 0.687 |
    | ×5 | 0.181 | 0.410 | 0.859 |
    | ×6 | 0.217 | 0.490 | 1.031 |

    At ×6, 2 of the 600 resets were refused MOVING.
  - **(b) The turn counter after a still minute** (twenty seeds; degrees):

    | σ (× HA-21) | noise only | with HA-20's drift |
    |---|---|---|
    | ×1 | 0 | 0–1.02 |
    | ×2 | 0 | 0–1.02 |
    | ×3 | 0 | 0–3.17 |
    | ×4 | 0–4.13 | 0–31.0 |
    | ×5 | 0–99.6 | 0–95.9 |
    | ×6 | 19.6–243.8 | 49.0–210.4 |

    The travel counter read 0.000 in at every σ up to ×6. So the verifier's "holds to about ×4" was optimistic too.
    At ×4, one seed in twenty counted 4.13° from noise alone, which the old HA-21 sentence ("0 up to four times") got
    wrong. Another counted 31° with the drift, which HA-211's "under 8° a minute with drift" got wrong.
  - **(c) A reset after standing still** (full hostile IMU, ten seeds, a 7-in reading, parked square). Every reset
    SEATED at ×4 (worst bound 0.917 in), at ×5 (0.975) and at ×6 after 60 s (0.996). At ×6 after 120 s, 2 of 10
    SEATED; all 8 refusals were IMPRECISE (bounds 1.001–1.169).
  - **Why it is a cliff.** At ×4 the 1° step is 3.5 σ of the difference of two readings, and a minute gives it 6,000
    tries. With drift, the reading's mean walks to the middle of a step, and noise then crosses the step back and
    forth, costing two steps per round trip.
- **Fixed, all in documents.**
  - `wall_reset.hpp`: the sampler's numbers.
  - `localizer.hpp`:
    - `kTurnCounterStepRadians`' doc (:217-226): the cliff, both columns of the table, and that an over-count only
      refuses.
    - `kTravelCounterStepInches`' doc: "0 up to six times".
    - `turnedSinceHeadingSeed()`'s doc (:573) and the counter's implementation comment (:317): "~0" is qualified as
      holding at the believed noise.
  - HA-21's added sentence (:3530-3536).
  - HA-211's Claim: a new *Where it stops holding* paragraph (:3163-3176), with the measured numbers, the speed
    numbers and the refusal at ×6.
  - HA-211's Settle (:3180-3183): the still-robot capture reports the heading's σ against the ~0.15° clear margin;
    past that, the counter's input needs smoothing, which is a design change, not a constant.
  - Guide 10's counters paragraph (:528-530), guide 14's precision bullet (:378-383) and the changelog's counters
    line (:81-82).
  - `docs/api` regenerated.
- **NOT changed: the counter's design.** The reasons:
  - An over-count only widens the heading tolerance. The tolerance feeds only `axisErrorBound` (`headingTol`'s one
    use in the evaluator, `wall_reset.hpp` :1087), so an over-count refuses more resets as IMPRECISE and never
    seats a wrong reading.
  - HA-21 is invented, and the clean margin is three times it.
  - Smoothing the counter's input before stepping it (an average of N ticks moves the cliff out by about √N) would
    change behaviour that T16's cases pin exactly, and the reviewed fix with it. That is a decision for the
    coordinator after the Step 0 still capture, and it is recorded in HA-211's Settle.
- **Superseded log claims.** Two earlier claims in this log were wrong for one seed in twenty at ×4: finding 1's "nothing
  counted at FOUR times … under 8° a minute with HA-20's drift" (fix round) and the 09:05 probe line ("7.73° at 60 s —
  holds"). They are kept as history, and this entry supersedes them.
- **HA-216: unchanged, still the coordinator's call.** The belief stays recorded in HA-211 and HA-21. Its numbers are
  now the multi-seed ones above.

#### Prose review (rule 4): the greps were run, and their hits read, before this entry was written
- `four times|under 8°|0.05-0.09|~0.35|never adds up|jitter never|however long it stands|stays clear|three times` over
  `README.md`, `docs/`, `include/` and `test/`. Fixed: `localizer.hpp` :317, guide 10 :526-530, guide 14 :378-383,
  the changelog :79-82, HA-21 and HA-211. Kept:
  - `localizer.hpp` :439 (the travel counter's "never adds up"), which holds to ×6;
  - this log's history lines, superseded above;
  - the generated `docs/api`;
  - the other hits, which are unrelated: the master plan's HDrive ~0.35, ekf "four times the run length", cookbook
    04's strafe "three times", and others.
- `NaN|non-finite|infinit` over guides 10, 11 and 14 and the changelog, about the wall reset: only the changelog
  described the evaluator's non-finite handling, and it gained the sampler's clause. Guide 11's NaN line (:495) is
  the odometry-stuck fault's `why=` field (DIAG1), which is unrelated.
- `WallSampler` over the guides, the changelog and `test/README.md`: only the changelog's API list, which is true.
- The plant test's comment ("the counter stays at a few degrees … HA-20's drift is real turning of the reading")
  holds: it runs at ×1 and ×2, where the drift column reads ≤ 1.02°.

#### The gates on the final tree (`vround/final.log`, a CLEAN rebuild, 14:50–15:13)
- No mutant marker (`fixmut/`, `vround/` or `verify/v2/mut/`). `--target clean`; the briefing regenerated against
  the absent binary; a full `cmake --build` compiled **246 objects** with no warning.
- `suite_run.py green`: **2710 cases, 2806820 assertions** (2709 + the new case). The `L6*,cookbook-05b*,guide-10w*`
  filter: **85 / 85**.
- The briefing:
  - regenerated against that fresh binary;
  - self-test OK (38 cases);
  - `check`: current.
- `api_doc_tool.py`, every check rc 0:
  - self-test OK;
  - check-coverage: 5170 entities across 190 headers, all documented;
  - check-fresh;
  - check-examples: 621 lines, verbatim;
  - check-removability.
- `doc_staleness_audit.py`: self-test OK, and the audit is clean (31 live docs).
- The ARM header gate (231 headers in one TU, `-Werror`): CLEAN. The CI greps: PROS-free and sim-free.
- `src_build_gate.py`: self-test OK (235 cases); `check` PASS (7 builds link). `git status -- src/ Makefile tools/` is
  empty.
- `ctest -R compile_refusal`: **251 / 251**.
- §7's greps (`vround/sec7.log`): every "expect empty" is empty. `maxCorrection` has only the `{0.0}` UNSET default
  and the report's echo. HA-204..209, 211 and 212 are each cited in code. `seatAxis` writes no drift term, heading
  bias or `odom_`.
- After this entry, the changelog clause and this log were re-checked by the doc gates (below). Nothing committed.
  **Nothing here has run on a robot.**
- 15:20, after this entry: the doc gates (briefing `check`, the four `api_doc_tool.py` checks) were green. The
  staleness audit did NOT trust the binary, because the src gate's self-test had planted and restored `src/` files,
  `test/CMakeLists.txt` and two G3a-side test files after the build. Their content was identical (`git status` shows
  no change in `src/`, `tools/`, `Makefile` or `test/CMakeLists.txt`), but their timestamps were bumped. So I:
  - removed the binary and regenerated the briefing against its absence;
  - reconfigured and rebuilt (246 objects, no warning);
  - ran `suite_run.py green`: **2710 cases, 2806820 assertions**;
  - regenerated the briefing against that binary (`check`: current).

  The audit is now clean with the binary trusted, the four `api_doc_tool.py` checks are rc 0, and no source file is
  newer than the binary. Nothing committed.

## The coordinator's merge verification — L6-A1 (2026-11-03 14:00–14:45, main `9d77c9d` + the squash of `l6-a1`)
- **Audit before the merge:** `audit_wt.py l6-a1` (taught L6's `ID__path` backup names that day) — 8 backed-up files, 0
  suspicious; the one file newer than a backup (`localization/localizer.hpp` against the verifier's scratch-copy V1b) is the
  verification-fix round's later edit, no live mutant. No gate plant; no stray root file. The transport commit `d81d053`,
  then `git merge --squash l6-a1` (base `d8fdb8b`).
- **Seven conflicts:** `docs/api/README.md`, `all-entities.md` and the briefing (regenerated); the changelog — L6's entry
  at the top of `## API 2.5`, dated as landed ("2026-11-03; built 2026-10-29–30"), above VIS1-A1's, MECH-A5's, IMU1's and
  CAL1-A2a's; guide 10's opening sentence names the mechanisms, the camera AND the wall reset; the glossary — VIS1's "size
  range" entry kept, L6's rewritten "snap (and never-snap)" entry (the one caller-invoked exception) taken; the register —
  **L6-A1 added EIGHT, not nine** (HA-210, robot two's side-sensor ports and mounts, stays HELD for L6-B), so the status is
  **7 of 202** (main's 194 + 8; 202 index rows, sorted, none missing — checked against both sides' id sets), the index rows
  merged BY NUMBER (HA-204..209, 211, 212 between VIS1's 192..203 and IMU1's 213..215), the "added" paragraphs in landing
  order ending "With them, 202". The plan docs' "L6-A1 BUILT … reviews owed" phrases made LANDED (build-order, REMAINING-WORK
  row 25, the roadmap's checklist line) and a roadmap "Now" paragraph added.
- **Gates on the merged tree** (`.claude/coordinator/logs/l6_a1_merge_verify.out`): host build with the doc gates; suite
  **2,899 | 2,899 passed, 3 skipped**; `src_build_gate.py` self-test OK and check **PASS, all 7 builds**; ARM header gate over
  **240** headers clean; CI greps empty; api check-fresh, the briefing, removability and the staleness audit clean; the
  compile-refusal harness **255 of 255**; no plant, no stray root file.
- **The coordinator's hand mutations** (`muts_l6_a1_main.json`; each on a fresh link, the whole suite under a 4 GiB / 300 s
  cap, restored and hash-checked): **5 of 5 caught** — ML1 (an x seat moves y too), ML2 (the seat drops the heading — "never
  the heading"), ML3 (the seat keeps the EKF's last sample, no re-base — the LOAD-BEARING line), ML4 (the `maxCorrection` edge
  made strict), ML5 (the reset seats under an armed motion) RED. Restored tree green.
- **Next for L6:** L6-A2 (the robot's side sensors and the everyday call) — the team lead's "Autons first" (2026-11-03) puts
  it ahead of the panel; VIS1-A1 has merged.

---

## L6-A2

- **Started:** 2026-11-03 16:15 (`date`).
- **Tree at start:** `59d93bb` (main: L6-A1 and VIS1-A1 merged and verified), clean, in the git worktree
  `.claude/worktrees/l6-a2` (branch `l6-a2`) — NOT the main working tree.
- **Go:** the team lead, 2026-10-22: "Accept all (Recommended)" — D1–D13; D2 AMENDS locked decision #4 for a
  caller-invoked ONE-AXIS seat while no motion is active. His rulings of 2026-11-03 ("Autons first"): the wall
  reset must be callable from a code auton well before 2026-11-21 (first competition 2026-11-23). The
  coordinator's workflow run launched L6-A2 ONLY. Executor: opus. Do not commit; no `pros`; no `/dev/ttyACM*`.
  Mutation backups and probes under `.claude/coordinator/scratch/l6-a2/` (never `/tmp`, never the repository root).
- **Scope (§2.1's L6-A2 row, §1.4–§1.6, §1.12):** NEW `config/wall_sensor_config.hpp`,
  `competition/wall_reset.hpp`, three test files (`wall_sensor_config_test.cpp`, `robot_wall_sensors_test.cpp`,
  `competition_wall_reset_test.cpp`); CHANGED `config/robot_config.hpp`, `robot/robot.hpp`,
  `chassis/robot_context.hpp`, `motion/motion_scheduler.hpp` (the shadow context's one line), the boot-line pins
  (`boot_source_test`, `xdrive_config_test`, `robot_camera_test`). NOT: `version.hpp` (§1.8), L6-B (`src/`, HA-210
  stays held), L6-C. Public documents (§3.1, five): the changelog, guide 03 (NEW section), guide 10, guide 15,
  cookbook 05 (NEW recipe, compiled) — and `docs/api/`.

### 16:15 — start
- Read `docs/internal/RESUMING.md` (standing rules, pace rules 1–8) and, per the team lead's 2026-10-22 reading
  ruling, the brief's header and answers block, "## Re-check — amendments" (RC1–RC48) in full, the Decisions (D1–D13)
  in full, §1.2 (the listing, the rules, the verdict table, the sampling paragraph), §1.4–§1.6, §1.9, §1.12, §2.1–§2.3,
  §3.1–§3.4, §4, §5, §6 and §7–§8 in full; L6-A1's log above and the coordinator's merge record (every hand-on to A2:
  finding 23 — the NotFieldStart line's start mode; `AutonContext::mechanismService` since MECH-A4; `robot.hpp`'s
  lines re-found after VIS1-A1's merge). This entry written before any code.

### ~16:22 (approximate — no `date` was taken; corrected at 16:55) — the baseline, and what the tree says against the brief
- **Baseline** (no binary existed: the briefing regenerated against its absence first, then `cmake -S test -B
  build/test`, a full `cmake --build`): `suite_run.py green` → **2899 cases, 2953698 assertions** (the coordinator's
  merge count). Logs: `scratch/l6-a2/{cmake_config,baseline_build}.log`.
- **Re-read at the tip (landmine 18 — never a brief line number):** `chassis/wall_reset_call.hpp` — L6-A1 already
  built the ONE sampler as `chassis::detail::runWallReset<Wait>(chassis, sensor, request, limits, wait, log)`, its
  wait returning false for a CUT; `localization/wall_reset.hpp` — `WallSensors {left, right, limits, log}`,
  `WallSensor::absence` defaulting to "not declared", `WallResetLog::record()`, the verdicts `NoSensor`,
  `NotFieldStart`, `Cut` and their words already there (A1 appended them); `NotFieldStart`'s line prints a fixed
  sentence (finding 23, handed to A2). `competition/auton_registry.hpp` — `AutonContext` has SIX members since
  MECH-A4 (`mechanismService` appended); `StartPose::Mode {Unset, Field, Relative}`, `startModeWord()`,
  `AutonEntry::effectiveStart()` (Unset for a route whose Start is `Free` and no entry start).
  `sequence/run_guard.hpp` — `waitFor(pred, timeout)` → `GuardedWaitResult {Satisfied, TimedOut, RunExpired}`,
  PRECONDITION `running()`. `motion/motion_scheduler.hpp` — the shadow context at the scheduler's constructor
  (clock … vision; the motorGroups comment). `robot/robot.hpp` (VIS1-A1 merged) — `buildCamera(devices)` LAST in the
  body, `copyCameraReason`, the three `describe()` formats end `"GPS %s tags ABSENT %s"` with the camera's words.
  `config/robot_config.hpp` — `SensorsConfig {gps, camera}`, `PortUse` ends `Camera`, `portUseWord` the ONLY switch
  on `PortUse` (`grep -rn 'case PortUse::' include src test | wc -l` → **11** at the base), `valid()`'s camera block
  last in the first pass.
- **The boot-line pins at the base** (`grep -rn 'tags ABSENT'`): `robot_camera_test.cpp` :347, :369, :415 (head),
  :492, :497; `xdrive_config_test.cpp` :234; `boot_source_test.cpp` :393-394; `robot_builder_test.cpp` :883/:885/:888
  (survives: `find` and ends-with). Guide 10 :769-772 quotes four banner tails. The programs print the banner in
  `drive_program.cpp` (`%.220s`, then `logWrapped` — splits at 196) and `xdrive_program.cpp` (`%.180s`): robot two's
  banner grows 179 → 192 bytes with " walls ABSENT", 206 with its prefix — wrapped into two log lines by the drive
  program, nothing cut; the X-drive's 155 → 168, under its 180 and under TermSink's 200 with the prefix. No `src/`
  change is needed (§7: `git diff -- src/` empty for A2).

### ~16:50 (approximate — no `date` was taken; corrected at 16:55) — the code, the tests, and the first green suite
- **Code (each re-read at the tip before editing):**
  - NEW `config/wall_sensor_config.hpp`: `WallSensorConfig {port, mount}`, `WallSensorsConfig {left, right, limits,
    note, declared()}`, constexpr `wallMountProblem` (a MEASURED mount only) and `wallLimitsProblem`, the bounds as
    named constants.
  - `config/robot_config.hpp`: the include; `SensorsConfig::walls` APPENDED after `camera`; `PortUse::WallSensor`
    APPENDED after `Camera` and its `portUseWord` case ("a wall sensor"); `valid()`'s wall block AFTER the camera's
    (port range naming the side, the RIGHT on the LEFT's port with its own sentence, the claim, each declared side's
    measured mount, then the limits always); `valid()`'s doc.
  - `chassis/robot_context.hpp`: `RobotContextConfig::walls` APPENDED (`localization::WallSensors*`, non-const,
    nullptr default, a documented forward declaration — the context includes no localization header), the accessor
    `wallSensors()`, the banner's OPTIONAL-slot section; the constructor's checks unchanged.
  - `motion/motion_scheduler.hpp`: the shadow context's ONE line `.walls = deps.ctx->wallSensors()` and a comment
    sentence (worded without the string "walls", so §7's grep finds exactly the one line).
  - `robot/robot.hpp`: `walls_` declared BEFORE `ctx_`; `.walls = cfg_.sensors.walls.declared() ? &walls_ : nullptr`;
    `buildWallSensors(devices)` in the body AFTER `buildCamera`; `buildWallSide` (catches a throwing provider, copies
    the refusal into `wallAbsence_[2][160]`), `wallWords()` (`walls ABSENT` / `walls L4 R5` / `(ABSENT)` / `MOUNT
    UNSET`, ≤ 41 bytes), inserted as `GPS %s tags ABSENT %s %s` (walls, then camera) in all three `describe()`
    formats; the accessors `wallSensors()`, `wallSensor(side)` (both constnesses), `wallSensorAbsence(side)`; the
    file banner's SIDE DISTANCE SENSORS section; `validate()` NOT changed.
  - NEW `competition/auton_wall_reset.hpp` (**the brief's `competition/wall_reset.hpp` — renamed, a delta below**):
    `competition::resetFromWall(AutonContext&, const WallResetRequest&)` — a precondition that the guard runs; the
    start (`effectiveStart().mode() != Field` → `NotFieldStart`, `detail` = `startModeWord(mode)`); the slot (nullptr
    → `NoSensor`); then `chassis::detail::runWallReset` with the guard's `waitFor` as the wait (RunExpired → CUT), the
    slot's log, and `&ctx.telemetry` as the line's sink. Early refusals go through `detail::refuseWallReset` (the
    report, ONE Warn line, filed when a slot exists).
  - `chassis/wall_reset_call.hpp` (A1's): `runWallReset` gains a trailing optional `hal::ITelemetrySink* sink =
    nullptr` (null = the chassis context's telemetry, as before); comments that said "from L6-A2" now name the file.
  - `localization/wall_reset.hpp` (A1's): the `NotFieldStart` line prints `start <MODE> -- the season's walls are
    FIELD coordinates: declare a FIELD start` (finding 23) — "start not FIELD" when a hand-built report carries no
    mode; `detail`'s doc names the mode; the "from L6-A2" comments re-worded. `localization/override_walls.hpp`: one
    comment sentence.
- **Tests:** NEW `test/wall_sensor_config_test.cpp` (T12, 6 cases), `test/robot_wall_sensors_test.cpp` (T13 + S1, 7),
  `test/competition_wall_reset_test.cpp` (T17, 9), the shared rig `test/wall_robot_rig.hpp` (a Robot built from a
  robot file with walls over the fake provider and the field plant; the pacer copies each side plant's reading into
  the provider's own `fakeDistance(4/5)` and its fault state into `fakeReadFaults`); CHANGED `cookbook_examples_test.cpp`
  (NEW `cookbook-05c`, the recipe inside `namespace shulib::bench`, run through `runAuton` on the Robot, plus its
  RELATIVE twin), `wall_reset_example_test.cpp` (NEW `guide-10x` — guide 10's `walls` listing, constexpr, built; NEW
  `L6 guide-03 bands` — guide 03's table recomputed from `preCheck` on all four walls), `wall_reset_line_test.cpp`
  (the NotFieldStart line: RELATIVE / UNSET / no mode), the boot-line pins (`robot_camera_test.cpp` ×5,
  `xdrive_config_test.cpp` :234, `boot_source_test.cpp` :393 and its "193 bytes" comment → 206, wrapped).
- **Suite:** `suite_run.py green` → **2921 cases** before the cookbook/guide cases (2899 + 22); with them the
  `L6*,cookbook-05*,guide-10*` filter is **119 / 119**.
- **Probe** `scratch/l6-a2/probe/bands.cpp` (the library's own `preCheck`, all four walls, a centred left sensor
  10.5 in up, a 7-in reading): the clean stretches equal the brief's Q1b numbers — unmeasured cone 33.6–41.8 /
  34.8–40.6 / 36.0–39.4 / none / none after 0 / 12 / 24 / 48 / 96 in of travel; a measured 18° cone 18.3–41.8 /
  19.5–40.6 / 20.7–39.4 / 23.1–37.0 / 27.9–32.2 — the same on all four walls (`bands.txt`, `bands18.txt`).

### 16:55 (`date`) — the documents, and the prose review (each grep run and its hits read BEFORE this entry)
- **Public (§3.1's five):** `docs/changelog.md` — a NEW section at the top of `## API 2.5` ("Added and changed since
  2.5 … No version bump"; the boot line's `walls` words called out as the visible change; what to do). **Guide 03** —
  the NEW section "Re-seating one axis against a wall" (what a side sensor measures; the FIELD-start rule; the nine
  checks in plain words, the beam height among them; the everyday call quoted from `cookbook-05c`; where to park and
  the robot centre's arithmetic; the clean-stretch TABLE, recomputed from the library and pinned by the new
  `L6 guide-03 bands` case; the D10 mounting advice; clear panels and the code strip; what it cannot catch → guide
  14), its "Covers" line, one sentence in the distance-sensor bullet, the signatures line. **Guide 10** — the
  section note (the everyday call and the robot file's block are in the library, not frozen), "Two forms" rewritten,
  NEW "### The robot file's side sensors" (`guide-10x`'s constexpr listing, `valid()`'s refusals, the Robot's
  accessors, the reset log, the optional slot, the boot line's four `walls` forms), NEW "### The everyday call, from
  an auton", the explicit call's lateness sentence, the chapter's "Covers" line, the camera's four boot-line
  examples gain `walls ABSENT`. **Guide 15** — five glossary entries (footprint, keep-out, seat, side sensor, wall
  reset), alphabetical. **Cookbook 05** — NEW recipe "Re-seat x against a wall" (D4's sequence through `then()`,
  quoted from `cookbook-05c`; why, watch out for), the chapter's case list, the re-seed recipe's pointer; the cookbook
  README's row.
- **Prose-review fixes outside the five (rule 4 — A2's code made them stale; logged as deltas):** guide 11 (the
  WALL section's "(once it is in the library)", the `NoSensor` / `NotFieldStart` / `Cut` rows — `NotFieldStart`'s
  "numbers" cell now names the start mode, finding 23), guide 14 (":333 Only its explicit form is in the library"
  rewritten; the explicit call's lateness bullet gains the everyday call's cut; a NEW "Only a FIELD start" bullet),
  `test/README.md` (the new rig), the HA register (the Source lines of HA-204, HA-205, HA-208 and HA-211 gain
  `config/wall_sensor_config.hpp` — no new id: §4 assigns NONE to L6-A2; HA-210 stays held for L6-B).
- **Internal:** `docs/roadmap.md` (the `[~]` L6 item: "L6-A2 BUILT 2026-11-03 (reviews owed)"), `REMAINING-WORK.md`
  row 25's status cell, `build-order.md`'s L6 entry (a status line). NOT edited, for the coordinator: the roadmap's
  "Now" paragraph and "Next for the wall reset: L6-A2" (:441-446 — the merge's), `build-order.md`'s first bold
  `Next:`, the briefing's generated block beyond `generate`.
- **The prose review (§3.4), each grep run and its hits read:**
  1. `never snap|never-snap|nothing that reads a sensor|caller seed|relocalize` over `include/` and the public docs —
     68 hits: the correctors' never-snap (complementary/EKF fusion, gps corrector, correction.hpp, debug_record,
     plausibility guard), the Localizer's banner and `seatAxis` doc (A1's, still true — A2 adds no path to a seat
     other than `seatAxis` through the shared sampler), guide 03 :118 (correctors — and the new section says the wall
     reset is the one deliberate exception), guide 11's CLMP and heading audit, guide 12 :149 (A1's sentence, true),
     guide 14 :103 (yaw), the glossary's "snap", the master plan's amended decision #4, the changelog (history).
     Nothing stale.
  2. `wall reset|re-seed|reseat` (minus `reseatHeading`) — every hit in the guides, cookbooks, roadmap, master plan
     and `include/` read: the stale ones were guide 10's "is not in the library yet" / "will sample", guide 11's
     "(once it is in the library)", guide 14 :333 — all FIXED above; the "from L6-A2" comments in
     `chassis/wall_reset_call.hpp`, `localization/wall_reset.hpp` (five) and `override_walls.hpp` — FIXED (they now
     name `competition/auton_wall_reset.hpp`); guide 14 :464 ("only a GPS, a tracking wheel or a wall reset would
     see it" — true); the roadmap's "Now"/"Next" paragraph (the coordinator's at the merge).
  3. `distance sensor|side sensor|IDistance` — guide 03 :92 (true; gained a pointer to the new section), guide 06's
     in-band impostor, guide 13 :172, the faq's 393-inch entry, guide 10's piece-sensor lines, guide 11 :337, guide
     14 :239/:244 (piece sensors), the master plan's HAL list, F4's row — all true; `hal/distance.hpp`'s two
     sentences (A1's) still true.
  4. `about −70|70 in|140` — guide 02 :17-18 "about −70 to +70" (true, "about"); the new section says ±70.2.
  5. `HA-20[4-9]|HA-21[0-2]` both directions — code sites (include/src/test): HA-204 ×9, 205 ×11, 206 ×6, 207 ×6,
     208 ×10, 209 ×6, 211 ×31, 212 ×10; register rows HA-204..209, 211, 212 (:430-437); HA-210 in neither (held for
     L6-B). Every id has its row and every row a code site.

### 16:57 (`date`) — every delta from the brief (each kept, with its reason)
1. **The header is `competition/auton_wall_reset.hpp`, not the brief's `competition/wall_reset.hpp`.** The build's
   API doc gate refused the brief's name: "two headers want the same page file … both -> docs/api/wall_reset.md"
   (`api_doc_tool.py`'s `page_name()` drops the subsystem directory, so `localization/wall_reset.hpp` and
   `competition/wall_reset.hpp` collide). Extending `page_name()` is a `tools/` change, which §7 keeps out of A2 (`git
   diff --stat -- src/ tools/ Makefile` empty). The name sits beside `auton_registry.hpp` / `auton_menu.hpp`; the
   function is the brief's, `competition::resetFromWall(ctx, request)`, unchanged. §7's two greps that name the
   file were run against the new name.
2. **The Robot hands the slot only when its config declares a side** (`.walls = cfg_.sensors.walls.declared() ?
   &walls_ : nullptr`; §1.5 wrote `.walls = &walls_`). With the slot always set, a robot file with NO side declared
   would refuse every reset `sensor absent: not declared`, where the brief's own verdict table gives the true word
   for it: `NoSensor` — "this robot file declares no wall sensors: add the `walls` block". T13's "the context slot
   non-null" is pinned for a robot that declares sensors; the no-side case pins nullptr (mutation MA3).
3. **The line goes to `ctx.telemetry` through a new optional `sink` parameter of A1's shared sampler**
   (`chassis::detail::runWallReset(…, log, sink = nullptr)`; null = the chassis context's telemetry, as before).
   §1.12 item 4 asks for "ONE `[WALL]` line through `ctx.telemetry`", and the sampler wrote to `deps.ctx->telemetry()`
   only. A `detail::` template, defaulted — the explicit call and every A1 test unchanged.
4. **The wait's timeout is A1's `windowMax + kWallResetWaitBackstopSeconds` (1 s)**, not §1.12's "windowMax + two
   ticks": it is the shared sampler's, and unreachable by construction (the predicate ends the window at
   `windowMax` by the clock). A `TimedOut` is read like the explicit form's (evaluate the window); only `RunExpired`
   is a CUT.
5. **A PRECONDITION that the guard is running**, before anything else (the brief is silent). The guard's own
   `waitFor` would throw anyway — but only AFTER the pre-checks, so a RELATIVE-start call outside a run would have
   refused quietly and a FIELD-start one thrown: one program error, two behaviours. An `AutonContext` lives for one
   guarded run (`runAuton` builds it inside `guard.run`), so outside one is a program error, loud (T17 pins both).
6. **The start mode rides in the report's `detail`** ("RELATIVE" / "UNSET", `startModeWord`) — finding 23 asked A2 to
   add "the mode to the report and the line". A new field would be a type the PURE `wall_reset.hpp` cannot name
   (`StartPose::Mode` is `competition/`'s); `detail` is already the per-verdict field the line prints. The line:
   `start RELATIVE -- the season's walls are FIELD coordinates: declare a FIELD start`.
7. **`valid()` refuses a MEASURED mount with no provenance** (not in §1.4's list): D6 says each mount carries "a
   provenance string (who measured, how, when)", and the camera's placement check refuses the same. An UNMEASURED
   mount is never judged (M26).
8. **The limits are checked ALWAYS, and as a superset of `wallResetProblem`'s** (§1.4 listed only the new spans):
   every limit the reset itself would refuse as BAD REQUEST (squareWithin ≤ 20°, the cone 3–30°, spreadMax > 0,
   minConfidence 0–1, still speeds > 0, errorBoundMax > 0, keepOutMargin ≥ 0, `window` ≤ 0.6 s — A1's
   `kMaxWallWindowSeconds`) plus §1.4's tighter spans — so a valid robot file never meets BAD REQUEST from its own
   limits; T12 pins that INVARIANT over 20 000 seeded draws (mutation MA1). Checked whether or not a side is
   declared: a wrong block is wrong, and no older config writes one.
9. **Both sides on one port has its own sentence** ("the LEFT and RIGHT wall sensors are both on port 4 -- one sensor
   per side, each on its own port"): through the shared `claim` it would read "claimed by a wall sensor and by a wall
   sensor". Every other collision is the shared claim's "naming both".
10. **The walls words come from one helper that prints the whole phrase** (`walls ABSENT`, `walls L4 R5`, …), so the
    three formats read `GPS %s tags ABSENT %s %s` (walls, camera) rather than `walls %s `.
11. **A refused side's absence buffers are the Robot's** (`wallAbsence_[2][160]`, the camera's bound); a provider
    that THROWS from `distance()` is caught and named, like the camera's (the brief's §1.5 named only a nullptr).
12. **Tests beyond §1.9's three files:** the shared rig `test/wall_robot_rig.hpp` (a Robot over the fake provider and
    the field plant — the provider has no bind hook for a distance sensor, so the pacer copies the plant's reading
    into the provider's own fakes: no HAL change); `cookbook-05c` in `cookbook_examples_test.cpp` (as the brief says);
    `guide-10x` (guide 10's `walls` listing, constexpr) and `L6 guide-03 bands` (guide 03's table, recomputed from
    `preCheck`) in A1's `wall_reset_example_test.cpp`; the brief's §6.4 agreement test (VIS1's Toggle places and
    length against L6's walls and keep-outs) in A1's `wall_reset_geometry_test.cpp` — owed by "the second-merged
    run", which was L6-A1, built before VIS1-A1 merged; A2 is the first L6 run on a tree with both headers; and a
    DISABLE-mid-window cut beside the deadline one (the docs claim both).
13. **Public documents beyond §3.1's five** — guide 11, guide 14, the cookbook README, `test/README.md`, the HA
    register's Source lines: each a prose-review fix A2's code made necessary (logged in the 16:55 entry).
14. **HA: none registered.** §4's "Cited in code by" column assigns no id to L6-A2 (HA-204..209, 211, 212 are A1's;
    HA-210 is L6-B's). A2's new code cites HA-204, 205, 208 and 211, whose rows' Source lines now name the file.

### 2026-11-05 00:22 (`date`) — RESUMED after the stop (the team lead stopped all work at the 5-hour usage limit, 2026-11-03 ~17:30)
- **What the stopped run left:** the code, tests and documents logged above (16:15–16:57); the mutation set prepared
  (`scratch/l6-a2/mut/{make_muts.py,mutations.json,driver.py}` — 24 mutations, NONE run: no `results.jsonl`, `bak/`
  empty, no `APPLIED.json`); `srcgate_selftest.log` EMPTY (17:08 — the gate's self-test was running when the stop came).
- **A GATE SELF-TEST PLANT WAS LEFT IN `src/`** — the coordinator's audit missed it because it greps for "(self-test
  plant", and this plant has no such marker: `src/robots/chassis_table.hpp` ended
  `#ifdef SHULIB_ROBOT_TANK_2026 / static int self_test_directive_form; / #endif` (mtime 2026-11-03 17:08:46) —
  `src_build_gate.py`'s case-19 one-chain plant on `CHASSIS_TABLE_PATH` (its `plants` list, the `#ifdef` form), killed
  inside its `_plant` context. The file's WHOLE diff was the plant (A2 changes no `src/` file — §7). Copied to
  `scratch/l6-a2/PLANT_FOUND__src__robots__chassis_table.hpp`, restored with `git checkout`, timestamp bumped;
  `git diff --stat -- src/ tools/ Makefile` now EMPTY. Every other plant target the tool names (main.cpp,
  bench_r3a.cpp, mechanism_table.hpp, robot_config.hpp's tail, build_selection_test.cpp, doctest.h, CMakeLists.txt,
  boot_source.hpp, chassis_table_test.cpp, lv_obj_style.h, the Makefile) grepped for every plant string: none.
  **For the coordinator:** the audit's plant grep should also look for `self_test_directive_form`, `gate1_selftest`
  and `SelfTestUnhandled`.
- **Next:** a fresh build (the binary removed first) and the suite, then the mutations, the gates, the reviews.

### 2026-11-05 00:23–01:01 (`date` at each end) — the fresh build, the suite, the mutations
- **Fresh build:** the binary removed, the briefing regenerated against its absence, then `cmake --build --clean-first`
  (261 objects — nothing from the stopped run trusted). `suite_run.py green` → **2926 cases, 2954563 assertions**.
- **The mutation driver** (`scratch/l6-a2/mut/driver.py`): per mutation, exact-once replacements that keep the line
  count; the file backed up under `mut/bak/`, a marker `APPLIED.json` while it is live; the binary removed; `cmake --build
  build/test --target shulib_tests` (make's dependency scan recompiles EVERY object that includes the file — each run's
  object count recorded); the WHOLE suite under `ulimit -v 4194304` + `timeout 600`, its full output saved to
  `mut/out/<id>.txt`; then the file restored byte for byte (sha-256 checked), its mtime bumped — in a `finally`, and on
  SIGTERM / SIGHUP / SIGINT. **A driver defect found and fixed before any verdict was kept:** the first launch named
  "failing" cases by every `TEST CASE:` header in the output — passing cases that print warnings too — so it could not
  say WHICH test went red. Stopped by SIGTERM during M26 (its handler restored the file: sha equal to the backup, the
  marker gone; the orphan check found no suite process of this worktree); the driver now names a case only when its
  doctest block reports an `ERROR:`. That first partial run is `results.aborted.jsonl`, not counted.
- **Results** (`mut/results.jsonl`; every file afterwards byte-identical to its backup — `cmp` over all of `bak/`):

  | id | file | the mutation | verdict (objects rebuilt) | the case(s) that went red |
  |---|---|---|---|---|
  | M25 | robot_config.hpp | the wall port's claim removed | RED 1 (76) | T12 "a wall port claimed by anything else is refused NAMING BOTH" |
  | M26 | robot_config.hpp | an UNMEASURED mount refused at boot | RED 2 (76) | T12 "an UNMEASURED mount is VALID…"; T13 boot-line words |
  | M27 | robot.hpp | a refused side sensor refuses the robot | RED 3 (76) | T13 "a REFUSED side never refuses the robot…"; T13 "a provider that refuses with NO reason…" |
  | M28 | robot.hpp | the provider's absence pointer kept, not copied | RED 2 (21) | T13 "…COPIED"; T13 "…NO reason…" |
  | M31 | motion_scheduler.hpp | the shadow context's `.walls` line dropped | RED 12 (111) — *see below* | all of T17 that reach the sampler, cookbook-05c, T13's shadow / slot / S1 cases |
  | M40 | auton_wall_reset.hpp | the start check dropped | RED 5 (109) | T17 RELATIVE, UNSET, the start-before-slot case, D3, cookbook-05c's RELATIVE twin |
  | M41 | auton_wall_reset.hpp | sampling through the frozen `chassis.waitUntil` | RED 2 (3) | T17 deadline mid-window CUT; T17 DISABLE mid-window cut |
  | MA1 | wall_sensor_config.hpp | the robot file's 0.6-s window bound dropped | RED 2 (76) | T12 every LIMITS bound; T12 "valid() ⊆ the reset's accept set" (the 20 000-draw invariant) |
  | MA2 | robot_config.hpp | the both-sides-on-one-port check dropped | RED 1 (76) | T12 "…NAMING BOTH" |
  | MA3 | robot.hpp | the slot handed in with no side declared | RED 1 (76) | T13 "port 0 on both sides: no slot…" |
  | MA4 | robot.hpp | MOUNT UNSET read from the left side only | RED 1 (21) | T13 boot-line words |
  | MA5 | robot.hpp | a refused side not marked `(ABSENT)` | RED 3 (21) | T13 refused-side cases (banner pins) |
  | MA6 | robot.hpp | a throwing provider caught as `logic_error` only | RED 1 (21) | T13 "…one that THROWS…" |
  | MA7 | auton_wall_reset.hpp | the line sent to the chassis's sink | RED 3 (22) | T17 SEATS (the line to the auton's sink), T17 deadline CUT, T17 refused side |
  | MA8 | auton_wall_reset.hpp | the outcome not filed in the slot's log | RED 6 (3) | T17 SEATS, deadline CUT, DISABLE cut, refused side; cookbook-05c; T13/T17 S1 through the Robot |
  | MA9 | auton_wall_reset.hpp | the guard-running precondition dropped | RED 1 (3) | T17 "a call with the guard NOT running is a PRECONDITION…" |
  | MA10 | auton_wall_reset.hpp | the start mode's word not carried | RED 2 (3) | T17 RELATIVE line; T17 UNSET line |
  | MA11 | auton_wall_reset.hpp | `entry.start` read instead of `effectiveStart()` | RED 1 (3) | T17 UNSET — the route whose Start states (or not) a heading |
  | MA12 | auton_wall_reset.hpp | the slot judged before the start | RED 1 (3) | T17 "the start is judged BEFORE the slot" |
  | MA13 | wall_sensor_config.hpp | the 11.5-in height edge made strict | RED 1 (76) | T12 every MEASURED-mount bound (11.5 exactly accepted) |
  | MA14 | robot.hpp | the robot file's limits not carried | RED 1 (76) | T13 "…builds both side sensors…" (`rangeMax` 14 typed) |
  | MA15 | robot.hpp | the read-fault count not handed to the side | RED 1 (21) | T13 "…builds both side sensors…" |
  | MA16 | auton_wall_reset.hpp | a RunExpired read as an ended window | RED 2 (22) | T17 deadline CUT; T17 DISABLE cut |
  | MA17 | wall_sensor_config.hpp | an unmeasured mount judged anyway | **BUILD-FAIL — caught at COMPILE time** by the test's own `static_assert` (`wall_sensor_config_test.cpp:111`, `static_assert(cfgns::wallMountProblem(kConstexprBlock.right.mount) == nullptr);  // unmeasured: none`); NOT counted as a run verdict (the rule: a mutant whose build fails is never counted) — its run-time twin is M26 (RED above) |
  | MA18 | wall_sensor_config.hpp | the provenance check dropped | RED 1 (76) | T12 every MEASURED-mount bound |

  **24 of 24 run-time mutations RED** (MA17 caught at compile time, not counted). The brief's own list for A2 —
  M25, M26, M27, M28, M31, M40, M41 — all RED.
- **What the run found in the TESTS (fixed, then re-verified):**
  1. **M31 first went red by CRASHING the suite** (`rc -11`: SIGSEGV in `cookbook-05c` — the recipe's test hook read
     `ctx.chassis.deps().ctx->wallSensors()->log.last` with no null check, so under M31 it dereferenced the null slot;
     doctest died there, after 587 of 2926 cases — every later case unrun). A kill counts as caught, but a test that
     crashes on the very defect it guards hides the rest of the suite. Fixed in `test/cookbook_examples_test.cpp`: the
     hook resets `gCookbookWall` and reads the log only through a non-null slot (a default report's verdict is
     `NoSensor`, never `Seated`, so the case cannot pass vacuously). The listing the cookbook quotes is unchanged
     (`check-examples` matches line by line). **M31 re-run on a fresh link: RED 12, the whole suite ran (2926 cases,
     12 failed, no crash)**; the first run's output kept as `mut/out/M31.first-run-crash.txt`.
  2. **RC31 / landmine 23 had no pin through the EVERYDAY form.** T8's RC31 cases cover the explicit call; S1 through
     the Robot (T13/T17) seats 2.5 in — over the pose guard's 2.25 in per tick — and checked nothing about it. Added to
     that case: the applied correction is over 2.25 in (REQUIRE), and the Robot's fault latch raised NO `Implausible`
     and NO `OdoStuck` over the run. **Proved to bite (MA19, a PROBE on code that is not A2's):** `LoopMonitor::reset()`
     keeping `hasLast_` (every wait's first tick then has dt > 0) → RED 10 (111 objects), among them this case at
     `robot_wall_sensors_test.cpp:487` (`2 == 0`), restored.
- After the fixes, a fresh build: `suite_run.py green` → **2926 cases, 2954566 assertions** (+3 — the new checks).

### 2026-11-05 ~01:01–01:37 (`date` at the end) — every gate on the fixed tree (fresh build first; logs in `scratch/l6-a2/`)
- `suite_run.py green` → **2926 cases, 2954566 assertions**; the briefing regenerated after the binary existed.
- `api_doc_tool.py`: generate (no `///` changed this session — the pages unchanged from the stopped run's); self-test
  **OK**; check-coverage **5508 public entities across 199 headers, all documented**; check-fresh **rc 0**;
  check-examples **669 quoted lines, all verbatim (11 source files); 6 whole-run listings, each one unbroken run**;
  check-removability **no public doc references docs/internal/**.
- `doc_staleness_audit.py` **clean**; `briefing_status.py check` **generated block current**.
- `ctest -R compile_refusal` **255 of 255**.
- ARM header gate over **242** headers (L6-A2 adds two): **rc 0**.
- `src_build_gate.py self-test` **OK (269 detector cases, each against a real build)**; `check` **PASS (all 7 builds
  link, 0 disallowed warnings)**. Afterwards: no plant string anywhere (`(self-test plant`, `self_test_directive_form`,
  `gate1_selftest`, `SelfTestUnhandled`), `git diff --stat -- src/ tools/ Makefile` EMPTY.
- §7's greps, each run: F4's `hal/distance.hpp` comment-only (prints nothing); no probe leak; no `setPose` /
  `reseatHeading` / `headingBias` in the three calls' code (the brief's `competition/wall_reset.hpp` run as
  `competition/auton_wall_reset.hpp` — delta 1); the plant independent; the localization headers PURE;
  `grep -n walls motion_scheduler.hpp` → exactly ONE line (:668, the shadow's `.walls`); `case PortUse::` → **12** (11 at
  the base + `WallSensor`); no `PROVISIONAL (A4` without an id; C7 empty; no `RCn` tag in a guide, cookbook, header or
  compiled listing; no new "simulator"; the PROS-free and sim-layering guards empty.
- HA, both directions: code sites HA-204 ×9, 205 ×11, 206 ×6, 207 ×6, 208 ×10, 209 ×6, 211 ×31, 212 ×10; register rows
  HA-204..209, 211, 212; HA-210 in neither (L6-B's).
- **Prose review, re-run this session (rule 4):** this session changed NO prose — only two test files and this log —
  so the logged review of 16:55 stands; its greps re-run as a check: (1) `never snap|…|relocalize` over `include/` + the
  public docs → 58 hits (the 16:55 entry counted 68 over a wider scope; the set read then is the same files); (2) the
  stale-wording grep `from L6-A2|once it is in the library|is not in the library yet|everyday form (L6-A2)|later parts`
  → EMPTY; nothing in `include/`, `docs/` or `test/` names the brief's `competition/wall_reset.hpp`; (4) `about −70|70
  in|140` → only guide 02's "about −70 to +70" (true) and unrelated numbers.

### 2026-11-05 ~01:40–01:54 (`date` at the end) — the review (round 1), its findings verified and dispositioned, the fixes
- **Who reviewed:** this run has NO agent-spawning tool, so the independent review came through the `code-review`
  skill (effort "high", forked — its own reviewer agents, READ-ONLY over this worktree's diff against `59d93bb`).
  **The brief asks for THREE reviewers and a fresh verifier; this run could field one forked review plus this
  executor's own whole-brief reading (the header, answers, RC1–RC48, D1–D13, §1.2's listing and rules, §1.4–§1.6,
  §1.9, §1.12, §2.1–§2.3, §3, §4, §5, §6, §7, §8 in full). The other two reviewers and the fresh verifier are OWED —
  for the coordinator to launch.** Each finding below was re-checked against the code before acting.
- **Findings (10; the skill's summary said 9) and what was done:**
  1. **`facing` never bounded** (REAL): `90.0` typed alone is 90 RADIANS (~116.6° wrapped) and passed `valid()`;
     every reset would refuse NOT SQUARE with nothing at boot saying why. The camera's placement check bounds yaw to
     (−180, 180] — the same bound added to `wallMountProblem` ("facing outside (-180, 180] degrees -- type it as N *
     kWallDegToRad (90.0 alone is 90 RADIANS)"), edges typed as `N * kWallDegToRad` (180 accepted, −180 refused).
  2. **A mount copied from the other side passes** (REAL): a LEFT block with the right's −90° refuses every reset NOT
     SQUARE; with only the offset's sign wrong the answer is ~15 in off (DISAGREES at 3 in, SEATED 15 in wrong under a
     generous `maxCorrection`). NEW constexpr `config::wallSideProblem(mount, side)`, called by `valid()` after
     `wallMountProblem` for a MEASURED mount: the LEFT one's facing strictly inside (0°, 180°) and `left` > 0, the
     RIGHT one's mirrored. **A delta from §1.4** (logged below). Why strict and why the offset too: a lens on or past
     the centre line facing out a side looks through the robot's own body (D10: nothing of the robot's own in the cone).
  3. **`minSamples` above windowMax/tick can never be reached** (DECLINED): `valid()` cannot know the tick — the
     pacer's, not the config's (no tick constant exists in `include/`) — and the header's promise is "never BAD
     REQUEST from its own limits", which holds; such a block refuses at run time with its own word. For the
     coordinator: L6-B's robot file uses the defaults (5 of ≤ 50).
  4. **`refuseWallReset` left `faultChannel` at its default `true`** (REAL): a NO SENSOR report claimed a fault
     channel. Now set as the sampler sets it (`walls != nullptr && side.faults != nullptr`); T17 pins true for the
     RELATIVE refusal (the side has a channel) and false for NO SENSOR. The suggested merge of the two tails is
     declined: the early refusals have no `WallSensor` to hand the sampler, and the shared part is the line call.
  5. **A call made AFTER the deadline read "landed in the window"** (REAL): `guard.waitFor` returns RunExpired at once,
     no sample taken, and the line sent a student to debug a window that never ran. The CUT line now says, with
     `samples == 0`, "the run's deadline or a disable came before the window began; nothing changed" (the verdict stays
     CUT — nothing seated); `WallResetVerdict::Cut`'s doc and guide 11's row say so. NEW T17 case (the guard's wait run
     past the deadline, then the call: CUT, 0 samples, zero latency, nothing read). L6-A1's byte-for-byte line test
     (`wall_reset_line_test.cpp`) now pins BOTH forms (samples 7 / 0).
  6. **The shadow context is still built field by field** (DECLINED — out of A2's scope): the brief's §1.6 and §7 make
     A2's scheduler change exactly ONE carried line; rebuilding the shadow from a copy of the config (a
     `RobotContext::config()` accessor) is a scheduler refactor. **For the coordinator:** a candidate follow-up — the
     next optional slot would be dropped the same silent way; the new comment beside the line says so.
  7. **`detail::wallFinite` duplicated `config::finiteNumber`** (REAL): removed; `wall_sensor_config.hpp` includes
     `config/camera_config.hpp` for it (both in `shulib::config`; `robot_config.hpp` already includes both).
  8. **`buildWallSide` / `copyWallReason` parallel the camera's** (DECLINED): the camera's helper writes a fixed
     member and the words differ per device; merging would rewrite VIS1-A1's merged code for no behaviour change.
  9. **S1's comment said "further from" the wall** (REAL — the robot is 2.5 in NEARER): fixed.
  10. **`kWallWordsBytes`' comment said 41 bytes** (REAL — 41 characters, 42 with the NUL; the 48-byte buffer fits):
      fixed.
- **Documents moved with the fixes:** the header banner of `wall_sensor_config.hpp`, `RobotConfig::valid()`'s doc,
  guide 10's refusal list, guide 11's Cut row, the changelog (the two new refusals; the cut line's second form);
  `docs/api/` regenerated.
- **Tests added:** T12 the facing edges (180 / −179.99 accepted; −180, 180.01, `90.0`, `-90.0` refused); NEW T12 "a
  MEASURED mount must face out of, and sit on, its OWN side" (both sides' facing edges 0.01 / 179.99 / 0 / 180 and the
  offset edges 0.01 / 0 / ∓7.5, an unmeasured copy valid, and through `valid()` with the side and port); T17
  `faultChannel` checks; NEW T17 "a call made after the deadline has passed is CUT with no sample, at once"; the line
  test's two CUT forms. Suite on a fresh build: **2928 cases, 2954620 assertions**.
- **The fixes' mutations** (the same driver; each file byte-identical to its backup afterwards):

  | id | file | the mutation | verdict (objects) | red case |
  |---|---|---|---|---|
  | MA20 | wall_sensor_config.hpp | the facing bound dropped | RED 1 (76) | T12 every MEASURED-mount bound |
  | MA21 | wall_sensor_config.hpp | the LEFT side's facing check dropped | RED 1 (76) | T12 "…its OWN side…" |
  | MA22 | wall_sensor_config.hpp | the RIGHT offset check made non-strict (`left` 0 accepted) — the equality edge | RED 1 (76) | T12 "…its OWN side…" |
  | MA23 | robot_config.hpp | `valid()` never calls `wallSideProblem` | RED 1 (76) | T12 "…its OWN side…" (through `valid()`) |
  | MA24 | wall_reset.hpp | the cut line ignores whether a sample was taken | RED 2 (79) | T17 "a call made after the deadline…"; T9 the line byte for byte |
  | MA25 | auton_wall_reset.hpp | an early refusal's `faultChannel` left `true` | RED 1 (79) | T17 "no slot: 'no sensor'…" |

### 2026-11-05 15:37 (`date`) — RESUMED after the second stop (the team lead dropped to one job at 92% of the session limit on 2026-11-05 ~02:07, then paused everything; resumed the same afternoon)
- **What the stopped run left:** the round-1 fixes and their mutations MA20–MA25 (all RED, logged above, `results.jsonl`
  ends at MA25 02:00:43); then a gate pass was IN PROGRESS (`last_build.log` 02:05, the api logs 02:06, `staleness.log`
  02:07 EMPTY — killed mid-run). No entry was logged for that pass, so it is re-run whole below; nothing from it counted.
- **Audit before trusting anything:** no `APPLIED.json`; no plant string (`(self-test plant`, `self_test_directive_form`,
  `gate1_selftest`, `SelfTestUnhandled`) in `src/`, `include/`, `test/` or the Makefile; `git diff --stat -- src/ tools/
  Makefile` EMPTY; no process of this worktree alive. Every mutated file against its LATEST backup (`cmp`): six
  byte-identical; `robot/robot.hpp` differs from MA15's backup by ONE line, the logged round-1 fix 10 (the
  `kWallWordsBytes` comment, "41 characters, 42 bytes with its NUL") — no live mutant.
- **Next:** the binary removed, a fresh build, the suite, every gate; then the remaining reviews.

### 2026-11-05 15:36–16:04 (`date` at each end) — the fresh build, the suite, the gate pass begun; reviews 2 and 3, verified and dispositioned
- **Fresh build:** the binary removed, the briefing regenerated against its absence, `cmake -S test -B build/test`, then
  `cmake --build --clean-first` (261 objects). `suite_run.py green` → **2928 cases, 2954620 assertions** (the round-1
  count exactly). The gate script `scratch/l6-a2/gates.sh` (→ `gates.out`) started in the background at 15:41.
- **Reviewers 2 and 3** came through the `code-review` skill again (forked, READ-ONLY over the diff against `59d93bb`):
  reviewer 2 (effort "high") against the brief's A2 sections, test adversarialness and docs-versus-code; reviewer 3
  (effort "medium") the WHOLE-BRIEF reader (all 2288 lines), told to report only what a narrow reading misses. **The
  fresh verifier is still OWED — for the coordinator** (this run cannot spawn an agent of its own). Every finding below
  was re-checked against the code by this executor before acting.
- **Review 2 (9 findings):**
  1. **Cookbook 05 said "The auton, as `src/team/autons.cpp` holds it"** (REAL — `src/team/autons.cpp` holds no
     wall-reset auton; its one auton, `forty`, is `StartPose::relative()` at :80). Re-worded: "An auton as you would
     write it in your team's `src/team/autons.cpp` … (The shipped `autons.cpp` holds no wall-reset auton yet: its
     autons start RELATIVE, and this one needs a FIELD start.)"
  2. **T12's LIMITS case claimed every edge but missed several** (REAL): the cone's 30° upper edge, `minConfidence` 0 /
     −0.01, `beamMargin` 0, the still speeds' bounds, every "above 0" field's tiny-positive side, the infinities. Added
     (each typed with the header's own `kWallDegToRad` — the same constant, so 30° lands ON the edge): +27 checks.
     Mutation MB2 below.
  3. **The round-1 entry said "A delta from §1.4 (logged below)" and no delta was logged** (REAL — a log defect): deltas
     15 and 16 below. Read against §1.4: its list is "refuses, by name: …", not an exhaustive "only", and every added
     refusal is a TYPED error in a MEASURED mount — the class §1.4 already refuses (and the camera's placement check
     too); D5's "a refused sensor never refuses the robot" is about the DEVICE, which still never refuses. Flagged for
     the coordinator all the same: it widens what stops a robot file at `valid()`.
  4. **§1.5's "LAST, after the camera" had no pin** (REAL): every T13 case read results, never the request order. NEW
     T13 case with a recording provider (`RecordingProvider` in `robot_wall_sensors_test.cpp`): the same robot file
     with and without both sides, a mechanism (port 6) and a camera (port 3) declared — the requests with walls are
     EXACTLY the requests without them, in order, then `distance 4, readFaults 4, distance 5, readFaults 5` and
     nothing else. Mutation MB1 below.
  5. **T17's two mid-window cuts checked x within 0.01 in (and the X travel counter) where the brief's T17 says "the
     Localizer bit-identical"** (REAL). Both cases now snapshot the brief's T6 list (pose, heading bias, drift term, the
     odometry's pose) plus both travel counters and the turn since the heading seed, compared with `==` — the parked
     rig is bit-stable across a window (A1's T6 holds it). Mutation MB4 below.
  6. **The header rename** (delta 1) — DECLINED here: the root cause is `tools/api_doc_tool.py`'s `page_name()`, a
     `tools/` change §7 keeps out of A2. **For the coordinator** (with review 3's item 4): decide the name before the
     merge — once students include `competition/auton_wall_reset.hpp` it is permanent.
  7. **A CUT report's `before` is the estimate at the cut, not at the call** — DECLINED, no change: `before`'s doc
     (`wall_reset.hpp` :451) is "the estimate's coordinate on `axis` when the reset judged it"; every WINDOWED verdict
     (SEATED, DISAGREES, …) judges after the window too (`evaluateWallReset(…, loc.pose(), …)`), and the pre-check
     refusals judge at the call because no time passes. A CUT is judged at the cut — the same meaning. A1's code.
  8. **§6.4's agreement test took min/max over the Toggle spans** (REAL — a body shrunk inside its brackets passed). Now
     a sweep from the near end: each span must start at or before where the last reached (the brackets adjoin the body
     exactly — an equality the sweep accepts), and the sweep must reach the far end; exactly 3 spans. Mutation MB3.
  9. **Two test comments claimed what the code does not do** (REAL): `robot_wall_sensors_test.cpp` "the one the Auton
     Tester reads" (nothing reads it until L6-B) → "the one `Robot::wallSensors()` hands a program to read";
     `competition_wall_reset_test.cpp` "asking for it reads the right sensor" → it judges the RIGHT mount and refuses
     NOT SQUARE reading nothing — now pinned (`r.reads() == 0`; mutation MB5).
- **Review 3 (7 findings, the whole brief):**
  1. **Guide 10's `guide-10x` listing carried an INVENTED provenance** — `"tape, the build team, 2026-11-05"`, a
     measurement that never happened, on ports 4/5 with the tolerances left to their defaults (REAL — landmine 16, "no
     invented value, ever"; D6 (a): each number with its tolerance and a provenance). The listing (and its compiled
     twin, line for line) now says "EVERY NUMBER HERE IS AN EXAMPLE ROBOT'S: type your own, measured, …", types
     `positionTolerance` and `facingTolerance`, and its provenance is `"EXAMPLE: who measured, with what, on what
     date"`; `guide-10x` pins the tolerances carried and the EXAMPLE prefix. **The same defect in A1's merged
     `guide-10w` listing** (`"tape, 2026-10-29"`, guide 10 :440 and `wall_reset_example_test.cpp`) fixed the same way
     (delta 17). `grep -rn 'tape, 20' docs include test` → one hit left, `wall_sensor_config_test.cpp`'s private
     `goodMount()` fixture — a test value, never shown to a student.
  2. **The Makefile "plant"** — NO CHANGE: the reviewer saw the src gate's self-test RUNNING (its own transient plant,
     restored by the tool as each case ends); the coordinator's scope orders `self-test` on every build. Re-checked
     after the gate pass ends (below).
  3. **The briefing read "Suite: not built"** (REAL, a sequencing point): it had been regenerated before the build and
     not after. Regenerated after the final suite (below).
  4. **The roadmap's "Next for the wall reset: L6-A2" (:446) and build-order's "Next for L6: L6-A2"** — NO CHANGE, for
     the coordinator: the 16:55 entry left both for the merge (the coordinator's convention). **And the brief itself
     still names `competition/wall_reset.hpp`** (§1.12, §2.1, §7's greps, §9): an L6-B executor running §7's
     `setPose|reseatHeading` grep against that path greps nothing and passes VACUOUSLY. **For the coordinator:** an
     amendment to the brief (RC49) naming `competition/auton_wall_reset.hpp` — the brief is the coordinator's document.
  5. **Guide 14, the changelog and guide 10 said every reset on robot two refuses `no sensor`** (REAL — the start is
     judged first, and every shipped auton is RELATIVE: a student sees `not a field start`). All three now say a
     FIELD-start auton's reset refuses `no sensor` and a RELATIVE one `not a field start` first. Cookbook 05 :209
     (its own FIELD-start recipe) was already true.
  6. **Public documents beyond §3.1's five** (delta 13) — NO CHANGE, for the coordinator: each was a prose-review fix
     rule 4 required (A2's code made the old sentence false); pace rule 7 is a sizing rule, and this run cannot re-cut
     the chunk. Possible merge conflicts with in-flight runs in guide 11 / guide 14 (IMU1 Part B) noted.
  7. **Guide 03 never said to measure the mount from the same point the start poses name** (REAL — the brief's Step
     0.1 (a): "start poses and the mount must use the SAME point"; a frame-corner mount goes ~1:1 into every seated
     coordinate and inside `maxCorrection` nothing refuses it). Added to the mounting paragraph.
- **More deltas from the brief (continuing 1–14):**
  15. **`wallMountProblem` bounds `facing` to (−180°, 180°]** (round 1, finding 1) — not in §1.4's list; `90.0` typed
      alone is 90 RADIANS, and every reset would refuse NOT SQUARE with nothing at boot saying why.
  16. **`config::wallSideProblem(mount, side)`** (round 1, finding 2) — not in §1.4: a MEASURED mount must face out of
      its own side (LEFT strictly inside (0°, 180°), RIGHT mirrored) and sit on its own side of the centre line (LEFT
      `left` > 0, RIGHT < 0) — the sign of a block copied from the other side; D10: nothing of the robot's own in
      the cone.
  17. **A1's merged `guide-10w` listing changed** (review 3, finding 1): the invented provenance replaced, as in
      `guide-10x`. The listing and its compiled twin changed together (`check-examples` matches line by line).

### 2026-11-05 16:19–16:39 (`date` at each end) — the reviews' mutations, the final build and every gate
- **The first gate pass** (`gates.out`, 15:41–16:19, the round-1 tree; its doc gates ran before reviews 2–3's edits, so
  they were re-run below): api self-test OK, coverage 5509 entities, check-fresh rc 0, check-examples 669 lines,
  removability clean; staleness clean; briefing current; ARM 242 headers rc 0; compile-refusal 255/255; CI greps empty;
  `src_build_gate.py self-test` **OK (269 detector cases, each against a real build)**; `check` **PASS (all 7 builds)**;
  afterwards no plant string anywhere and `git diff --stat -- src/ tools/ Makefile` EMPTY (review 3's "Makefile plant"
  was this self-test's own, live at the time — restored by the tool).
- **The reviews' mutations** (the same driver, 16:19–16:26; every file afterwards byte-identical to its backup):

  | id | file | the mutation | verdict (objects) | red case |
  |---|---|---|---|---|
  | MB1 | robot.hpp | the side sensors requested BEFORE the camera | RED 1 (79) | T13 "the side sensors are requested LAST…" |
  | MB2 | wall_sensor_config.hpp | the limits' 30° cone edge made strict | RED 1 (76) | T12 every bound of the LIMITS |
  | MB3 | override_walls.hpp | the Toggle body keep-out shrunk to 0.8× inside its brackets | RED 1 (79) | §6.4 the two season headers agree |
  | MB4 | wall_reset_call.hpp | a CUT nudges the OTHER axis 0.001 in | RED 3 (10) | T17 deadline cut; T17 DISABLE cut; T17 after-the-deadline cut |
  | MB5 | wall_reset_call.hpp | the pre-check reads the sensor once | RED 3 (6) | T17 refused side (the RIGHT mount's `reads() == 0`); T17 after-the-deadline; A1's T4 |

  MB4's 10 and MB5's 6 objects: `wall_reset_call.hpp` has SIX dependents (`*.o.d`: competition_wall_reset,
  cookbook_examples, robot_wall_sensors, wall_reset_example, wall_reset_gates, wall_reset_plant) — all six rebuilt in
  each (object mtimes checked for MB5); MB4's other four were MB3's restored `override_walls.hpp` dependents.
  **With round 1: 35 of 35 run-time mutations RED** (MA17 caught at compile time, not counted; MA19 a probe).
- **Final:** a `--clean-first` build (261 objects — nothing from a mutation run trusted); `suite_run.py green` →
  **2929 cases, 2954674 assertions**; the briefing regenerated AFTER the binary existed ("2,929 cases / 2,954,674
  assertions, 3 skipped — green"; review 3's item 3).
- **The final gate pass** (`gates_final.out`, 16:30–16:38): api self-test **OK**; check-coverage **5509 public entities
  across 199 headers, all documented**; check-fresh **rc 0**; check-examples **676 quoted lines, all verbatim (11 source
  files); 6 whole-run listings, each one unbroken run**; check-removability **clean**; staleness **clean**; briefing
  **current**; ARM header gate **242 headers, rc 0**; compile-refusal **255 of 255**; CI greps (PROS-free, sim-layering)
  **empty**; `src_build_gate.py check` **PASS (all 7 builds link, 0 disallowed warnings)**; no plant;
  `git diff --stat -- src/ tools/ Makefile` EMPTY. The src gate's SELF-TEST was not re-run on this tree: since its pass
  at 16:1x only `test/` and `docs/` changed (every header byte-identical, `src/` and `tools/` untouched), and the
  self-test exercises the tool's detectors, which did not change.
- **§7's greps, re-run:** `grep -n walls motion_scheduler.hpp` → ONE line (:668); `case PortUse::` → 12; no
  `setPose` / `reseatHeading` / `headingBias` in the two calls' code (one A1 comment saying "never setPose"); no `RCn`
  tag in a guide, cookbook or header; no new "simulator" (the one added hit is this log's own sentence); HA-210 in no
  code site (L6-B's).
- **The prose review for THIS session's prose** (rule 4; each grep run, hits read): (1) `autons.cpp` near "holds" /
  "wall" over the guides, cookbooks, changelog, README → only the re-worded cookbook 05 :147-148; (2) `no sensor|NO
  SENSOR` over the public docs → guide 10 :501, :616 (fixed), :636 (lists the start check first — true), :802 (robot
  two's mechanisms, unrelated); guide 14 :335 (fixed), :357 (heading, unrelated); guide 11 :420/:429 (the start word
  listed first; the NoSensor row's remedy — true); guide 16/18 (claw sensors, unrelated); guide 03 :224 (unrelated); the
  changelog :28 (fixed), :42 (after the start check — true), :73 (the slot's description — true); cookbook 05 :209 (its
  FIELD-start recipe — true); (3) a public `provenance = "` not starting `EXAMPLE` → NONE; (4) `same point|start
  poses name` → guide 03 :369, the new sentence only.
- **For the coordinator:** (a) the FRESH VERIFIER is owed (this run cannot spawn one); (b) the header name (delta 1;
  review 2's item 6) — decide before students include it; the brief's §1.12/§2.1/§7/§9 still name
  `competition/wall_reset.hpp` — an RC49 so L6-B's §7 greps do not pass vacuously; (c) deltas 15–16 widen `valid()`
  past §1.4's list (typed errors in a MEASURED mount only) — the team lead may want to see them; (d) public documents
  beyond §3.1's five (delta 13; rule 7) and A1's `guide-10w` listing changed (delta 17); (e) the roadmap's "Now"/"Next"
  and build-order's "Next" left for the merge; (f) the audit's plant grep should also look for
  `self_test_directive_form`, `gate1_selftest` and `SelfTestUnhandled` (00:22 entry); (g) a follow-up candidate: the
  scheduler's shadow context built field by field (round 1, finding 6). Files other in-flight chunks own: NONE edited
  (no `calibration/*`, `vision/*`, `src/programs/bench_r3a.*`, `tools/`, `competition/auton_menu.hpp` or
  `alliance.hpp`, `chassis/routine.hpp`); shared files touched on A2's own lines only: `mkdocs.yml`, the changelog
  (top of `## API 2.5`), the HA register (four Source lines), `test/README.md`, the plan docs' L6 lines.
- **State at close:** the tree holds L6-A2 complete, uncommitted, on branch `l6-a2`; no mutant, no plant, no stray file;
  the suite binary is the clean build's.

### 2026-11-05 16:57 (`date`) — the coordinator's three reviewers (correctness, tests, docs): 15 findings to re-check and disposition
- **Audit before anything:** no `APPLIED.json` under `scratch/l6-a2/mut/`; no plant string (`self-test plant`,
  `self_test_directive_form`, `gate1_selftest`, `SelfTestUnhandled`) in `src/`, `include/`, `test/` or the Makefile;
  `git diff --stat -- src/ tools/ Makefile` EMPTY; no process of this worktree alive. The tests reviewer's mutations ran
  on a COPY (`scratch/l6-a2/review-tests/tree`, its driver says so) — this worktree was never mutated by it.
- Each finding is re-checked against the code before acting; the disposition entry follows when the fixes, their
  mutations and the gates are done.

### L6-A2 review findings and their disposition (the coordinator's three reviewers — correctness, tests, docs; 2026-11-05 16:57–18:06, `date` at each end)
- **Who:** three reviewers the coordinator launched (lenses: correctness, tests, docs), 15 findings after dedupe. Each
  was re-checked against the code by this executor before acting. The tests reviewer's 12 mutations ran on a COPY of
  this tree (`scratch/l6-a2/review-tests/tree`); its survivors were re-cut here as `RT*` on THIS tree with this run's
  driver (fresh link, the whole suite, every dependent recompiled, restored by sha).
- **Correctness 1 — MOUNT UNSET and MOTION ACTIVE never driven through the everyday form, nor through a real Robot**
  (REAL, a test gap; the behaviour was correct). NEW T17 cases on `wall_robot::WallWorld` (a `robot::Robot` built from
  a robot file, the auton run through `competition::runAuton`): (a) the robot file's LEFT mount `measured = false` →
  `mount unset`, no time, the Localizer bit-identical (the T6 list), the Robot's slot still says unmeasured, refused 1 /
  seated 0, one WALL line; not `required` → the chain completes 2 steps; `required` → stopped `ActionFailed`, 1
  skipped. (b) a `HoldPose` armed through `ctx.chassis.scheduler().async()` inside the routine → `motion active`, no
  time, bit-identical, the hold STILL armed (the reset cancels nothing). Mutations MC1–MC4 below, all RED.
- **Correctness 2 — deltas 7, 15 and 16 make a typo in a MEASURED mount refuse the whole robot at boot** (REAL as a
  consequence; NOT changed — a decision for the team lead). Verified: `valid()` false → `robot::validate` not ok →
  `competition_program.cpp:719-721` and `drive_program.cpp:646-647` log `the library graph REFUSED to build` and return
  (no driver control, no auton). Kept as built because §1.4 itself already refuses typed errors in a measured mount
  (height, offset, cone, tolerances, bias, accuracy) the same boot-fatal way, the camera's placement checks do the same
  (VIS1-A1, merged), and robot two's boot config is validated by a host test before upload (`boot_source_test.cpp:380`
  `validate(boot.config, …).ok` — L6-B's block, once it flows through `bootConfigFrom`, is covered there; L6-B should
  pin that). Delta 7 (no provenance) is the one whose reset never reads the field — it follows the camera's identical
  rule (D6: each mount carries its provenance). **The cost is now said to students:** guide 10's refusal list gains
  "Any of these … stops the WHOLE robot from building … boot the robot once after every edit to the block, never first
  at a match." **For the coordinator → the team lead (with the merge, or as RC49):** "a typo in a measured wall mount —
  including a missing provenance — refuses the robot at boot, as the camera's placement does"; the alternative is to
  move delta 7 (and optionally 15/16) from `valid()` to a boot-line warning and let the reset refuse BAD REQUEST.
- **Tests 1 — the robot's typed limits never checked on the everyday call (T1 survived)** (REAL). NEW T17 on CtxRig: the
  slot's `rangeMax` typed 6.5 in → the 7-in reading refuses `out of range` (the defaults SEAT the same rig — the SEATS
  case), bit-identical, the line saying the typed ceiling; NEW T17 on the Robot through `runAuton`: the robot file with
  the library's limits SEATS, the same file with `walls.limits.rangeMax = 6.5 in` refuses `out of range`, its line
  `(window 2.00..6.50)`. **RT1 RED 2** (both new cases).
- **Tests 2 + 3 — an early refusal's side, axis, before, fault channel, maxCorrection echo and Warn level pinned only at
  their defaults (T2–T7 survived)** (REAL). NEW T17 "an early refusal reports the REQUEST's …": the RED wall (axis y)
  with the RIGHT sensor, the right side's read-fault channel removed while the left keeps its own; two subcases (a
  RELATIVE start with the slot; no slot): `side == Right`, `axis == Y`, `before == pose.y()` exactly (x and y > 1 in
  apart), `faultChannel` false, `maxCorrection` 3, `required` false, level `Warn`, the filed copy's side/axis, and both
  lines byte for byte (`red wall y REFUSED not a field start (RIGHT sensor): start RELATIVE -- …`, `red wall y REFUSED
  no sensor (RIGHT sensor): this robot declares no wall sensors`). **RT2–RT7 each RED 1** (this case).
- **Tests — the reviewer's T9 and T12 also survived; not reported as findings; re-checked: EQUIVALENT, no test can see
  them.** T9 (a `TimedOut` read as a cut): unreachable — the predicate ends the window at `windowMax` by the clock,
  under the wait's `windowMax + 1 s` timeout (delta 4). T12 (an early refusal's line given no mount and the default
  limits): the line's `NoSensor` and `NotFieldStart` branches (`appendWallResetLine`) read neither the mount nor the
  limits. Code kept as is (the real mount and limits are the right arguments should those lines ever print them).
- **Tests 4 — the banner's walls buffer shrinkable below its longest form (T11 survived)** (REAL). NEW block in T13's
  boot-line case: wall sensors on ports 19 and 21 (20 is a drive motor in this robot file), both refused, the right
  mount unmeasured → the exact tail `walls L19(ABSENT) R21(ABSENT) MOUNT UNSET camera ABSENT` (41 characters, asserted).
  **RT11 RED 1** (22 objects = `robot.hpp`'s 22 dependents in the suite target).
- **Docs 1 — the header name differs from the accepted brief, and the brief's §7 grep names the old path** (REAL; NOT
  changed here — the brief is the coordinator's document and the root-cause alternative is a `tools/` change §7 keeps
  out of A2). **Recommendation for the coordinator, before the merge:** (a) keep `competition/auton_wall_reset.hpp`
  (beside `auton_registry.hpp` / `auton_menu.hpp`; no tool change) and write RC49 renaming the brief's references (D4,
  §1.2, §1.8, §1.12, §2.1, §7 — its `setPose|reseatHeading|headingBias` grep — and §9). §7's grep run against the new
  path today: `auton_wall_reset.hpp` no hit; `wall_reset_call.hpp` one hit, A1's comment "never setPose"; and
  `competition/wall_reset.hpp` does not exist (so the old grep WOULD pass vacuously — RC49 is needed).
- **Docs 2 — the header said the Auton Tester "shows" the log** (REAL): `auton_wall_reset.hpp` step 4 now "what the Auton
  Tester will read, from L6-B"; `wall_sensor_config.hpp`'s banner "G3a's robot panel is to write this block like every
  other (it writes none yet)". `docs/api/` regenerated.
- **Docs 3 — the recipe's includes never named** (REAL; verified: `auton_wall_reset.hpp` does not bring in
  `override_walls.hpp`, and `src/team/autons.cpp:34-41` includes neither). Cookbook 05's recipe, guide 03's quote of it
  and the changelog's "What you must do" now name both includes.
- **Docs 4 — guide 11's SensorAbsent / MountUnset rows short of the brief's table** (REAL): SensorAbsent gains "the boot
  line's `walls L4 R5(ABSENT)` says which side"; MountUnset "(the boot line says `MOUNT UNSET` until you do)".
- **Docs 5 — the prose review left out §3.4's `startAt` grep** (REAL, a log defect). Run now over the guides, the
  cookbooks, `docs/*.md` and `include/`: guide 16 :142-145 (no `startAt` in a registered auton — true, and the wall-reset
  recipe uses the entry's FIELD start), guide 17 :79 (the same), guide 09 :76/:90/:107/:355, cookbooks 01–05 (each
  `Routine::startAt` example; cookbook 05 :103-131 the D11 re-seed recipe and its "full seed" note — true, A2 adds
  only the pointer), the changelog :175 (A1's entry) and :1851 (history), roadmap :144/:594/:604 (F10), the HA register
  :535, `routine.hpp` :10/:66/:290. **None made stale by A2.**
- **Docs 6 — behaviour changes beyond §1.4 / §1.2's table; one missing from the numbered deltas** (REAL for the
  numbering): the CUT line's second form is now **delta 18** (below). Deltas 7, 8, 15, 16 and 18 are put to the
  coordinator for the team lead (with correctness 2). Guide 14's possible conflict with IMU1 Part B: for the merge.
- **Docs 7 — plan status lines and the changelog heading stale** (REAL): REMAINING-WORK row 25, the roadmap and
  build-order now read "L6-A2 BUILT 2026-11-03–05 … three reviews in, the fresh verifier owed"; the changelog heading
  "(built 2026-11-03–05)" — the landing date is the merge's to add, as for L6-A1.
- **Docs 8 — the glossary's "footprint" tagged chapters that never use the word** (REAL): guide 03's check 3 now says
  "widened by the beam's footprint (its spread at that range)"; the entry's tag is `[Ch. 3]` (guide 10 never used it).
- **Docs 9 — a review tag in a shipped header** (REAL): `wall_reset.hpp`'s comment now "The start mode's word, so
  RELATIVE and UNSET read apart."
- **More deltas from the brief (continuing 1–17):**
  18. **The CUT line has a second form** (round 1, finding 5): `samples == 0` → "the run's deadline or a disable came
      before the window began; nothing changed" — in A1's `localization/wall_reset.hpp` (not in §2.1's A2 file list),
      and the Cut row's remedy in guide 11 changed with it. The verdict stays `Cut`.
- **Mutations of this round** (`mut/results.jsonl`, 17:0x–17:17; each file `cmp`-identical to its backup afterwards; no
  `APPLIED.json` left):

  | id | file | the mutation | verdict (objects) | red case(s) |
  |---|---|---|---|---|
  | RT1 | auton_wall_reset.hpp | the library's limits, not the slot's | RED 2 (3) | T17 slot's own limits; T17 on the Robot: typed limits |
  | RT2 | auton_wall_reset.hpp | an early refusal's side not set | RED 1 (3) | T17 an early refusal reports the REQUEST's … |
  | RT3 | auton_wall_reset.hpp | its axis not set | RED 1 (3) | the same |
  | RT4 | auton_wall_reset.hpp | its maxCorrection not echoed | RED 1 (3) | the same |
  | RT5 | auton_wall_reset.hpp | its line at Info, not Warn | RED 1 (3) | the same |
  | RT6 | auton_wall_reset.hpp | its fault channel read from the LEFT side | RED 1 (3) | the same |
  | RT7 | auton_wall_reset.hpp | its `before` read on x whatever the wall | RED 1 (3) | the same |
  | RT11 | robot.hpp | the walls buffer 40 bytes | RED 1 (22) | T13 the boot line's walls words |
  | MC1 | robot.hpp | the Robot forces `measured = true` copying the mount | RED 1 (22) | T17 on the Robot: mount UNMEASURED |
  | MC2 | auton_wall_reset.hpp | the everyday form hands the sampler a side with `measured` forced | RED 1 (22*) | the same |
  | MC3 | auton_wall_reset.hpp | the everyday form cancels an armed motion first | RED 1 (3) | T17 on the Robot: a hold armed … |
  | MC4 | wall_reset_call.hpp | the sampler's motion check dropped | RED 2 (6) | T17 on the Robot: a hold armed …; A1's T4 |

  Dependents in the suite target (`CMakeFiles/shulib_tests.dir/**/*.o.d`): `auton_wall_reset.hpp` 3, `robot.hpp` 22,
  `wall_reset_call.hpp` 6 — every run rebuilt at least its file's dependents (*MC2's 22 = its 3 plus MC1's restored
  `robot.hpp` dependents). **12 of 12 RED.** With the earlier rounds: **47 of 47 run-time mutations RED** (MA17 caught at
  compile time, not counted; MA19 a probe).
- **Tests added this round:** 4 cases in `competition_wall_reset_test.cpp` (T17: early-refusal fields with 2
  subcases; the slot's limits; on the Robot: mount unmeasured ×2 runs, a hold armed, typed limits ×2 runs) + 1 block in
  T13's banner case; both files' banners updated. Suite on a `--clean-first` build (261 objects; the binary removed and
  the briefing regenerated against its absence first): **2934 cases, 2954791 assertions**; the briefing regenerated
  AFTER the binary existed ("2,934 cases / 2,954,791 assertions, 3 skipped — green").
- **The prose review for this round's prose** (each grep run, hits read): (1) `Auton Tester (shows|reads)` over
  `include/` and the public docs → one hit, A1's `WallResetLog` doc (`wall_reset.hpp` :531 and its API page: "the Auton
  Tester shows it …, from L6-B") — already qualified by "from L6-B", judged true, NOT changed (a `///` edit for no
  change of meaning); (2) `includes neither|auton_wall_reset.hpp` over the guides, cookbooks and changelog → cookbook
  05 :149-150, guide 03 :340-341, the changelog :94-95 (new, true), the changelog :35 and guide 10 :425 (true);
  (3) `footprint` → guide 03 (new), the glossary, `wall_reset.hpp` and its API page, the master plan (A1's era, true);
  (4) `REFUSED to build` → guide 10 :598 (new — the programs' exact string, `competition_program.cpp:720`); guide 17
  :350's "the graph refused to build" (true); (5) `finding [0-9]|review finding` in `include/` → `ekf_fusion.hpp:187`
  and `match_runner.hpp:30`, both older chunks' comments, none of A2's (the test files' "a review finding of
  2026-11-05" comments are test provenance, never shown to a student).
- **Every gate on the fixed tree** (`scratch/l6-a2/gates_r4.out`, 17:19:58–18:04:27; after the clean build and the
  suite above): api self-test **OK**; check-coverage **5509 public entities across 199 headers, all documented**;
  check-fresh **rc 0**; check-examples **676 quoted lines, all verbatim (11 source files); 6 whole-run listings**;
  check-removability **clean**; staleness **clean**; briefing **current**; ARM header gate **242 headers, rc 0**;
  compile-refusal **255 of 255**; CI greps (PROS-free, sim-layering) **empty**; `src_build_gate.py self-test` **OK (269
  detector cases, each against a real build)**; `check` **PASS (all 7 builds link)**; no plant string (`self-test
  plant`, `self_test_directive_form`, `gate1_selftest`, `SelfTestUnhandled`) in `src/`, `include/`, `test/` or the
  Makefile; `git diff --stat -- src/ tools/ Makefile` EMPTY. This entry was written after the doc gates ran, so they
  were re-run on it (next bullet).
- **The doc gates re-run on this entry** (18:05): check-fresh rc 0; check-examples 676 lines verbatim; removability
  clean; coverage 5509, all documented; staleness clean; briefing current.
- **For the coordinator (open, none decidable by this run):** (a) the FRESH VERIFIER is still owed; (b) the header name —
  RC49 renaming the brief's `competition/wall_reset.hpp` (docs 1; recommended: keep `auton_wall_reset.hpp`); (c) to the
  team lead: deltas 7, 8, 15, 16 and 18 — above all that a typo in a MEASURED wall mount (a missing provenance
  included) refuses the robot at boot (correctness 2); (d) guide 14 against IMU1 Part B at the merge; (e) the earlier
  entry's items (e)–(g) stand.
- **State at close:** L6-A2 complete and uncommitted on branch `l6-a2`; no mutant (`cmp` of every round-4 backup
  against its file: identical; no `APPLIED.json`), no plant, no stray file; the suite binary is the clean build's
  (2934 cases, green); `git diff --stat -- src/ tools/ Makefile` EMPTY.

### Verification-round findings and their disposition (the fresh verifier's one finding; 2026-11-10 21:16–, `date` at the start)

- **Before anything:** the verifier's probe and mutation left nothing behind. `localization/wall_reset.hpp` and
  `test/competition_wall_reset_test.cpp` are `cmp`-identical to its backups (`verify/probe1/wr.bak`,
  `orig_test.cpp`); there is no `APPLIED.json`; `git diff -- Makefile src/` holds no `self-test plant`.
- **V1 — the CUT line's two forms split between 0 and 1 sample, but no test sat ON a one-sample cut (V11 survived)**
  (minor). **REAL, re-checked.** Delta 18 picks the form with `rep.samples > 0` (`wall_reset.hpp`, the `Cut` case of
  the line writer). The existing cases sat at 0 samples (the call after the deadline) and at about 5 (the deadline
  0.05 s in; the disable 5 ticks in). T9's pure line check in `wall_reset_line_test.cpp` sat at **7**, so `samples > 1`
  passed everywhere. The root cause is in the tests: the equality edge was never landed on. That breaks the standing
  rule that a test claiming an edge lands on it exactly. The code is right and unchanged.
  - **The fix** has two parts, so the edge is pinned at both levels:
    1. A NEW T17 case in `competition_wall_reset_test.cpp`: "a DISABLE on the window's FIRST tick — exactly one
       sample — is a cut that LANDED in the window". It sets `pacer.abandonAt = paces + 1`, REQUIREs
       `rep.samples == 1`, and checks: verdict `Cut`; returned within one tick; `report.abandoned`; the Localizer
       BIT-IDENTICAL (`checkSame`); the log's `refused == 1` with `last.verdict == Cut` and `last.samples == 1`; and
       the line byte for byte ("left wall x REFUSED cut (LEFT sensor): the run's deadline or a disable landed in the
       window; nothing changed"). The file's banner says the two forms' edge is checked on each side.
    2. T9's line check moved from `samples = 7` to `samples = 1`, with a comment naming V11. Its 0-sample check
       stays.
  - **Proved RED** with the round-4 driver (`mut/driver.py`; restore in a finally and on SIGTERM/SIGHUP/SIGINT;
    sha-256 checked; mtime bumped). The binary was removed each time, and **79 objects** were rebuilt each time. That
    is every suite object whose `.o.d` names `localization/wall_reset.hpp`: 79 of 79.

    | id | the mutation | verdict | red case(s) |
    |---|---|---|---|
    | V11 (new case only, before the T9 change) | `rep.samples > 0` → `> 1` | RED 1 | T17 a DISABLE on the window's FIRST tick |
    | V11 (after the T9 change) | the same | RED 2 | that case; T9 every verdict's line, byte for byte |
    | V11b (the other side of the edge) | `rep.samples > 0` → `>= 0` | RED 2 | T17 a call made after the deadline has passed …; T9 |

    After each run, the file was `cmp`-identical to its backup, and no `APPLIED.json` was left. A `-Werror` build
    failure would have been re-cut; none happened. Each verdict comes from the suite's own failure count, run under
    `ulimit -v 4194304` and `timeout 600`.
- **The prose review for this round** (grep run before this entry; each hit read): `came before the window|landed in the
  window` over `docs/guide`, `docs/cookbook`, the changelog, `docs/api`, `include/` and `test/`. Hits: guide 11 :448
  (the Cut row: "landed in the window" by default, "came before the window began" when no sample was taken; true),
  `docs/api/wall_reset.md` :1361 and `wall_reset.hpp` :385 ("`samples` 0; the line says which"; true), the changelog
  :87 (true), and `wall_reset.hpp` :1435-1436 (the code). **No prose changed meaning and none was stale.** No `///`
  comment changed, so `docs/api` was not regenerated.
- **Every gate on the fixed tree** follows in the next bullet. A gate pass started at 21:20 was stopped at the
  staleness step, before the src gate's self-test could plant anything, so the T9 change could be made first. Its
  five api results (all OK) were superseded.
- **The fresh build and the suite** (21:24). The binary was removed, the briefing was regenerated against its absence,
  and then the build ran (79 objects). `suite_run.py green` was run under `ulimit -v 4 GiB` and `timeout 600`:
  **2935 cases, 2954809 assertions, green**. That is one case more than round 4 (the new T17 case). Moving T9's sample
  count does not add an assertion. The briefing was regenerated AFTER the binary existed: "2,935 cases / 2,954,809
  assertions, 3 skipped — green".
- **Every gate on the fixed tree** (`scratch/l6-a2/gates_v1.out`, 21:25:41–22:03:45):
  - api self-test **OK**
  - check-coverage **5509 public entities across 199 headers, all documented**
  - check-fresh **rc 0**
  - check-examples **676 quoted lines, all verbatim (11 source files); 6 whole-run listings**
  - check-removability **clean**
  - staleness **clean**
  - briefing **current**
  - ARM header gate **242 headers, rc 0**
  - compile-refusal **255 of 255**
  - CI greps (PROS-free, sim-layering) **empty**
  - `src_build_gate.py self-test` **OK (269 detector cases)**
  - `check` **PASS (all 7 builds link)**
  - no plant string in `src/`, `include/`, `test/` or the Makefile
  - `git diff --stat -- src/ tools/ Makefile` **EMPTY**

  This entry was written after the doc gates, so they were run again on it (next bullet).
- **The doc gates re-run on this entry** (22:04):
  - check-fresh rc 0
  - check-examples: 676 lines, all verbatim
  - removability: clean
  - coverage: 5509 entities, all documented
  - api self-test: OK
  - briefing: current
  - staleness: clean

  At 22:04 staleness also noted that the test binary was older than `src/common/boot_source.hpp` and 9 more. This was
  only a timestamp effect: the src gate's self-test plants files and restores them, and the restores were newer than
  the binary. `git diff --stat -- src/ Makefile tools/` stayed EMPTY. Nothing was untrusted, but the step was redone
  rather than explained away:
  - the binary was removed and the briefing regenerated;
  - a full rebuild ran (261 objects);
  - the suite was **green: 2935 cases, 2954809 assertions**;
  - the briefing was regenerated, and is current;
  - staleness came back **clean, with no note**, and check-fresh returned rc 0 (22:08).
- **State at close (verification round):**
  - L6-A2 is complete and uncommitted on branch `l6-a2`.
  - The one finding is fixed in the tests. The code is unchanged, so this round has no new delta from the brief.
  - V11 and V11b are both RED. Run-time mutations over all rounds: **49 of 49 RED**.
  - No mutant is left: every backup is `cmp`-identical to its file, and there is no `APPLIED.json`. No plant and no
    stray file remain.
  - The suite binary is the 22:08 clean build's.
  - For the coordinator, the round-4 entry's open items (b)–(e) stand. Item (a), the fresh verifier, is now done.

## L6-A2 — the coordinator's merge verification *(2026-11-10)*

- **Squash** of `l6-a2` (base `59d93bb`) onto `d684f34` (IMU1 complete). Conflicts resolved by hand: the changelog (L6-A2's
  entry at the top of `## API 2.5`, dated "2026-11-10 (built 2026-11-03–05)", above MECH-A6's), REMAINING-WORK rows 25–26
  (a word-level three-way merge against the base), and the generated `docs/api/` pages and briefing (regenerated). Status
  words moved to LANDED: the roadmap's L6 paragraph, build-order's L6 status line, REMAINING-WORK row 25; a roadmap "Now"
  paragraph added. **The header name (delta 1) is KEPT** — `competition/auton_wall_reset.hpp` — and the brief gains
  **RC49** (every reference to `competition/wall_reset.hpp` means the new path; §7's grep must name it). **Deltas 7, 15 and
  16 kept** (a typo in a MEASURED wall mount refuses the robot at boot, as the camera's placement does; the host test of
  the boot config catches it before an upload) — put to the team lead in plain words with the merge.
- **Gates on the merged tree** (`.claude/coordinator/scratch/l6_a2_merge_verify.sh`, every binary capped): host build
  clean; the suite green, 3,098 cases, 3,025,746 assertions; `src_build_gate.py` self-test OK and check PASS (all 7 builds
  link); the ARM header gate CLEAN (242 headers); the api doc gates, the staleness audit and the briefing check clean; the
  compile-refusal harness 255 of 255; no plant, no stray root file.
- **The coordinator's own mutations** (`.claude/coordinator/muts_l6_a2_main.json`): **8 of 8 RED** — ML1 the FIELD-start
  check dropped, ML2 a mid-window deadline not read as a cut, ML3 the `[WALL]` line sent nowhere, ML4 `facing` = 180°
  refused, ML5 an empty provenance accepted, ML6 a RIGHT mount copied from the LEFT block accepted, ML7 the 11.5 in
  height ceiling made exclusive, ML8 `declared()` blind to the right side. The restored tree relinked and ran green.
- **L6-A2 LANDED.** Nothing has run on a robot. Next for L6: L6-B (robot two's sensors and the WALL bench page) — and,
  since 2026-11-10, robot one's file (G1) declares its own side sensors.
