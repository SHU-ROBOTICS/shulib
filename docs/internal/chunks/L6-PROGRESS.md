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
