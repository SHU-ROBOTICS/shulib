# AF1-S — PROGRESS (live log)

<!-- release: between parts — AF1S-A1 and AF1S-A2 landed (the checked Toggle step, from an auton); robot one's Toggle block (its camera numbers, Step 0) and the driver-control auto-flipper come after (declared by the coordinator 2026-11-17) -->

Brief: [`AF1S-skills-toggle-check-flip.md`](AF1S-skills-toggle-check-flip.md). Executor model: `opus`. Nothing
committed by the executor. Host only: nothing here has run on a robot.

## AF1S-A1

**Base:** worktree `.claude/worktrees/af1s-a1`, branch `af1s-a1`, at `542b5f6` (main `e458dc6` + VIS1-A2's code as
built and reviewed — its fix round in flight). Started 2026-11-11.

### Log
- Created this log first. Read RESUMING.md's standing rules; the brief's header, answers block, Re-check R1–R12, D1–D11,
  §1, §2.0–2.3, §3, §4, §5, §6, §7, §8, §9 in full; §0 read; the coverage check (§0.6) skimmed by heading (A1 does not
  touch it).
- 2026-11-11: wrote `vision/override_elements.hpp`'s ToggleId / toggles:: / QuadrantColour / kToggleQuadrantColour /
  toggleIndex / quadrantColour / toggleName (HA-243); NEW `competition/toggle_visit.hpp` (the step, rules 1–7,
  lookAtToggle, worstCaseSeconds, toggleVisit consteval check); NEW `sim/toggle_plant.hpp`; NEW
  `test/toggle_visit_rig.hpp`. Manual re-check (brief header's ask): `<RSC3>`a-iii-1, `<SC4>`, `<SG6>`, `<VURS2>`,
  `<VURS3>` re-read in the local v2.0 text (`.claude/coordinator/scratch/autoflip/v20.txt`); v2.1 NOT re-fetched by
  this executor — the comments cite v2.0 and the brief writer's v2.1 reading of 2026-11-10 (no Toggle-rule change).
- First end-to-end smoke (arm, red-side Toggle Y->B->R, right wall): FLIPPED after 2 pushes, the plant scores RED.
  The placeholder bounds had to grow (flip/back 0.8 s -> 1.2 s): the host arm takes 0.87 s to reach 90 deg.
- Exploration on the host test plant (a temporary `test/zz_af1s_explore_test.cpp`, deleted before the report) — what
  the scenes give, read before the tests pin them:
  - all four flipper forms flip the red-side Toggle YELLOW -> RED in two pushes (arm 9.2 s, lift 7.3 s, piston 6.0 s,
    passive bar);
  - lens 10 level, look 16 +- 6: containment accepts a TRUE look distance of 14–17 in and refuses 18 in and farther
    (true 24 in: band 17.2–30.8 -> NOT AT THE SPOT — the overlap rule would have accepted it); turned 8 deg -> UNKNOWN
    BEFORE "another colour directly above" (P1's mount table, reproduced);
  - a camera FROZEN at the look spot, half-way in, at the push spot before or after the roll, or half-way out -> UNKNOWN
    AFTER every time (never a colour accepted);
  - lens 12 tilted 10 deg, look 16 (offBy 7: its band at the spot is 10.8–22.4 in), push 10 in, sticky -> NO CHANGE
    (R4's case reachable);
  - jammed part-way after a full stroke: 50 deg -> FLIPPED (the plant scores YELLOW), 70 deg -> UNKNOWN AFTER ("no band"),
    100 deg -> FLIPPED (YELLOW) — R1's honest limit, reproduced on this plant at two of three angles;
  - a lookalike while the true face is ours (the Toggle's own faces unseen, HA-199's miss): a BLUE plate on the face's
    height 4 in in front of the wall is pushed past (the Toggle rolled away from ours) WITH the position gate on as
    well as off (its 3-in tolerance plus the fix radius admits 4 in); the same plate 1.4 in lower -> NOT AT THE SPOT by
    containment (band 20.1–33.0), nothing pushed.
- DELTA (design, found by exploration): the first draft ran the retract-now check from the drive in until the drive
  out. With every stage within a tick of its bound (push 1 at need(2) + 1 ms) it fired at the END of push 2's retract
  and reported RETRACTED FOR TIME on a Toggle the plant showed flipped: at `back`'s command the time left is at least
  threshold + lookWithin + 2 ticks - backBound, which goes under the threshold during the retract when backBound >
  lookWithin. The brief's words are "while the flipper is out" — the check now runs only from the push's command until
  `back`'s (a passive bar: from the drive in's end until the drive out). Re-run: FLIPPED, 1.1 s to spare.
- Consequence, stated in the header: rule 4 makes the retract-now unreachable while every stage keeps to its bound; it
  is for time lost outside them. Its test models a brain loop that stalled (a clock jump while the flipper is out).
- Tests written (all green through `python3 tools/suite_run.py green`: 3315 cases at that point):
  `test/sim_toggle_plant_test.cpp` (T12, 8 cases), `test/toggle_visit_rules_test.cpp` (the table, the request's
  refusals with their equality edges, T13, T5's containment edges, R6's directions, T11's outcome map, the words),
  `test/toggle_visit_scenes_test.cpp` (T1–T11, T14, lookAtToggle — end to end on the plants; ONE [TGL] line,
  ASCII, <= 160 bytes, Info only on success and every success line ENDING `seating not checked` is checked after
  every scene), `test/compile_refusals/t01..t08_toggle_visit_*.cpp` (T10's build refusals, 8 TUs, all judged PASS by
  run_refusal.py), cookbook-08a in `test/cookbook_examples_test.cpp` (the route over the explicit form; its page is
  AF1S-A2's). `test/toggle_visit_rig.hpp` is the shared rig.
- Docs (same change): changelog (top of `## API 2.5`), guide 19 (a section "Checking and flipping a Toggle in a
  skills route"; "What none of this does" re-worded — it said the camera flips nothing), guide 14 (a section "The
  Toggle flip in a skills route: what it cannot catch", R1 first), guide 11 (a section "The Toggle visit (TGL)",
  each line and what to do), the HA register (rows + checklist entries HA-231/232/234/235/243/244/245; the Held
  line; the status total 208 -> 215). `docs/api/` regenerated (it also picks up VIS1-A2's three headers' pages,
  which the base's docs/api lacked — generated, never hand-written).
- Mutation driver: `.claude/coordinator/scratch/af1s-a1/mut.py` — each mutant on a fresh `shulib_tests/fast`
  build (the doc gates skipped: they would run the previous, possibly mutant, binary), every AF1S test object's
  mtime checked newer than the mutation (the first run found cookbook_examples_test.cpp.o NOT recompiled — `/fast`
  skips CMake's dependency consolidation and the cookbook test's new include of the rig was not yet in it; those
  five verdicts were DISCARDED, a full build consolidated the dependencies, the run restarted from MA1), the file
  restored in a finally and on SIGTERM/SIGHUP/SIGINT, its mtime bumped. Results: `scratch/af1s-a1/mutations.log`.
- MA16 as first cut (the stop moved to push 3, the loop bound kept) SURVIVED — an EQUIVALENT mutant: the loop's
  own bound still stopped at two pushes. Re-cut as MA16r (the loop AND its stop moved together).
- Mutation results (each on a fresh `/fast` build with every AF1S test object checked recompiled, observed, restored;
  full lines in `scratch/af1s-a1/mutations.log`, outputs `out_<id>.txt` for the re-runs). EVERY listed mutation
  CAUGHT except the first cut of MA16 (equivalent, re-cut) — details in the report. Notes from reading the failing
  assertions, said plainly:
  - MA12 (retract-now dropped) and MA12b (its threshold without driveBound) turn T6 (R7) red by its VERDICT (CUT,
    not RETRACTED FOR TIME; the step running into the deadline) — NOT by a flipper left out at 0:00: with every
    stage bounded, a stall large enough to cross the threshold also expires the push's own step deadline, the
    service ends it TIMED_OUT, and rule 5's "back on ANY verdict" retracts anyway. The retract-now is defence in
    depth here; the brief's "the flipper out at 0:00" failure is not reachable on this plant.
  - MA24 as first cut (a blocking moveTo, then the async motion) and as re-cut MA24f (ONLY the blocking verb) are
    both caught — but T3's "frozen at the look spot" case does NOT turn red under either: this step also reads the
    camera during the push, the retract and the settle at the PUSH spot, so a camera frozen at the look spot is
    seen frozen there (the lens moved 12 in). R3's own rule is pinned by a NEW test (T3 (R3): at least one read
    per 4 ticks while the step's motion is live — 2 reads against 124 needed under MA24f).
  - MX4 (the run-time check that `back` finds the flipper's own slot) SURVIVED the first run — no test reached it
    (two NAMES are refused by the request first). A test was added (another robot's same-named arm's `stow`), and
    the re-run CAUGHT it.
- Post-run additions (after the driver finished — the three mutated files are never edited while it runs): the HA
  citations in toggle_visit.hpp's request docs (HA-231, HA-234, HA-235, HA-244, HA-245), T2's "another colour beside
  it" case (ConflictingPartialBand — a blue band cut by the image's top edge; the first try, a tall strip, gave
  "another colour directly above" because its shape failed the aspect gate), the R3 reads-while-driving test, the
  MX4 test, and the plant's independence comment re-worded so the brief's §7 grep (`vision/` in toggle_plant.hpp)
  is empty.
- PROSE REVIEW (rule 4 — the greps run and their hits READ before this line was written):
  - `grep -rn -i 'override_elements|toggle_visit|checkAndFlip|toggle_plant|TogglePlant|kTogglePlaces'` over
    docs/*.md, docs/guide, docs/cookbook, README.md, test/README.md: roadmap.md :149 (F15 — names
    `override_elements.hpp` among the camera's unfrozen library code; still true, left), guide 19 (this run's own
    section). test/README.md had no entry for the new plant — ADDED (`sim::TogglePlant`, beside `sim::CameraPlant`).
  - `grep -rn -i 'auto-flip|flip.*toggle|toggle.*flip'` over the same: guide 18 :506 (a pneumatic `toggle()` binding
    — unrelated), roadmap.md :395-399, :1249 (the plan: the driver-control auto-flipper after the panel/21st — still
    true for AF1-A..D), master plan :81, :604-629 (the June strategy, marked history) — none stale. Guide 19's own
    "What none of this does" said the camera "flips nothing" — RE-WORDED (the step now flips, in a skills auton).
  - `simulator` in the new and changed lines: only the standard disclaimer (`Say "host test plant", never
    "simulator"`) in `sim/toggle_plant.hpp` and `test/toggle_visit_rig.hpp`, as `sim/camera_plant.hpp` has it.
  - Removability (`grep -rln 'internal/|chunks/|RESUMING|build-order'` over docs/*.md, docs/guide, docs/cookbook,
    README.md): empty.
  - Hardware claims in the new doc lines (`tested|measured|works on the robot|on a robot`): every hit says NOT run
    on a robot / None is measured, except the changelog's "each tested" — re-worded "each tested on the host test
    plant".
  - Guide 10 (the API page's list) and guide 15 (glossary) are AF1S-A2's by the brief's §3 — NOT touched; the
    `docs/api/` pages are generated.
- DELTAS from the brief (against base `542b5f6`), each with its reason:
  1. `ExplicitToggleCamera` = {feed*, model*, the tracker's CONFIG, poseTolerance, headingToleranceDeg} — not a tracker
     handle: the step builds a FRESH `vision::ToggleTracker` per visit, so "the hold that counts began inside this
     step" (rule 2) holds by construction (VIS1-A2's tracker has no reset). `poseTolerance` 0 = no bound stated: the
     tracker gets an infinite one, so its position gate is SKIPPED rather than run on a pose taken as exact.
  2. Verdicts APPENDED: `Unset` (= 0, a default report — a default `AlreadyOurs` would read as success), `DriveFailed`
     (the brief names no verdict for a drive in/out that does not arrive within driveBound: nothing is pushed after a
     drive in that did not arrive; the robot drives out), `SeenNotOurs` (`lookAtToggle`'s answer for a sure colour at
     the spot that is not ours).
  3. The retract-now rule runs only while the flipper is OUT (exploration: the first draft's wider window gave a wrong
     RETRACTED FOR TIME). Consequence documented: unreachable within the bounds; MA12/MA12b turn T6 red by the verdict,
     not by a flipper left out.
  4. The drive OUT targets the pose recorded at the look spot (not "the same distance reversed from wherever the drive
     in ended") — equal when the drive in arrives; lands back on the look spot when it did not.
  5. A PASSIVE flipper may leave flipBound / backBound at 0 (nothing is commanded); a set value still counts in need().
  6. Rule 6's "moved more than frozenMoveIn since the retract's end" is the LENS's displacement from where it was at the
     retract's end to where it was at the counted read (the tracker's own measure). "The retract's end" = the moment
     `back`'s verdict arrived (a passive bar: the drive in's end); settle follows it.
  7. A request's problem is a run-time PRECONDITION (thrown) and a BUILD refusal through `competition::toggleVisit({...})`
     (consteval); the excursion vs the tracker's frozenMoveIn is a precondition checked once the camera is found usable
     (it needs the tracker's config). 8 compile-refusal TUs (the brief named three).
  8. The tick: `kToggleVisitTickSeconds = 0.010` (HA-32's brain tick; the library has no constant outside hal/pros).
  9. T2: "camera frozen?" cannot be produced before a push (a parked look cannot see a frozen camera — the tracker's
     own rule) — it is T3's; T2 covers the other eleven reasons.
  10. T6's "every stroke at its bound": each bound set within a tick of what the host plant takes, push 1 at need(2) +
      1 ms. T6's retract-now scene is a loop STALL (a clock jump) with the host Toggle counting the robot touching until
      its front is 15 in out (a stand-in so the drive out's time decides).
  11. T1's "our other robot flipping it at a scripted time": before the visit in the seeded sweep (0 contacts while
      ours); during the drive IN it is a race the step cannot see — PINNED as a stated limit (guide 14).
  12. T1's lookalike (R5): the position gate did NOT turn the 4-in lookalike away (pinned, with the gate on and off);
      containment refused the lower plate. T14: jammed at 70 degrees this plant reads "no band" (UNKNOWN AFTER), at 50
      and 100 the destination (FLIPPED, scored YELLOW) — pinned per angle.
  13. T11 pins the DEFAULT stop (STEP1-A's onFail is not in the base) — as the brief allows.
  14. cookbook-08a uses the EXPLICIT form and the default stop, and has no cascade `startHome` (no cascade on the rig);
      AF1S-A2 rewrites it in the everyday form with onFail(skipTo(...)) and writes its page.
  15. HA register: AF1's HA-231/232/234/235 ROWS registered here (AF1 never registered them; the coordinator's plan
      commit, which was to add AF1's and ALLY1's numbers to the Held line, is not in this base) — the Held line amended
      in this change; the status total 208 -> 215.
- DEPENDENCIES on VIS1-A2's in-flight build (re-check at its merge): `vision::ToggleTracker` (ctor (config, model),
  copy, `update(frame, PoseBound)`, `view(now)`), `ToggleView::{colour, why, range, bearing}`, `ToggleColour` +
  `toggleColourWord`, `ToggleWhy` (Sure, NoRead, CameraSilent, ReadNotUsable, CameraFull, NoBand, CutByImage,
  MaybeBottomFace, TwoColours, NotHeldYet, CameraFrozen, ConflictingPartialBand, ColourAbove) + `toggleWhyWords`,
  `ToggleTrackerConfig::{ids, holdSeconds, bearingWindowDeg, frozenMoveIn, positionToleranceIn, maxPoseToleranceIn,
  maxFixRadiusIn}`, `toggleTrackerConfigProblem`, `CameraFeed::{tick, latest, stats}`, `CameraFeedConfig`,
  `vision::boundAtImage`, `PoseBound`, `Interval`, `kDegToRad`, `CameraModel::placement()`; the test rig uses
  `test/camera_scene_rig.hpp` (`camrig::truth/model/tcfg/face/kLensFwd`) and `sim::CameraPlant`'s addToggle /
  addRolledToggle / CameraState. Guide 19-the-camera.md is VIS1-A2's file: this run added a section to it.
- FINAL GATES (the tree as left, 2026-11-11): suite `green — 3318 cases, 3088711 assertions` (through
  `tools/suite_run.py`); `ctest -R compile_refusal`: 266/266 (the 8 new t0* TUs included); ARM header gate rc 0 (248
  headers in one TU); CI guards (pros/, sim/) empty; `src_build_gate.py self-test` OK (269 cases) and `check` PASS (7
  builds) — `git diff -- Makefile src/` empty afterwards, no self-test plant; api_doc_tool self-test / check-coverage
  (5835 entities, 204 headers) / check-fresh / check-examples (763 lines) / check-removability OK;
  doc_staleness_audit clean; briefing_status self-test OK and check current (215 registered, next free HA-246). The
  brief's §7 greps: every one as specified (the §1.6 files, src/, Makefile and tools/ unchanged; no AF1-A include;
  no competition/ include from vision/ or mechanism/; nothing of vision/ in the plant; no waitUntil and no blocking
  verb in the step; `seating not checked` present; every HA id cited has its row; no unlabelled PROVISIONAL).
- STOPPED HERE: AF1S-A1 built, NOT committed. For the reviewers and the coordinator — the report lists every file,
  delta and mutation.

### AF1S-A1 review findings and their disposition

Three reviewers (correctness, tests, docs), 2026-11-11. The fix round ran 2026-11-11 (~22:40–23:22), was cut by
the 5-hour usage limit and then by the team lead's pause; **resumed 2026-11-12 20:21 EST** (`date` read). The
cut run left NO entry in this log, so everything below was re-checked on the resumed tree.

**Resumption audit (2026-11-12 20:21).** `git status` and `git diff -- src/ Makefile`: no src/Makefile change, no
`(self-test plant`. Each file the mutation tooling backed up compared with its backup: `toggle_visit.hpp` ==
`fbackup_toggle_visit.hpp` (the cut run's last restore — pristine); `override_elements.hpp` ==
`backup_override_elements.hpp`; `toggle_plant.hpp` differs from `backup_toggle_plant.hpp` (08:35) only by the
08:58 post-run edit logged above (the plant's independence comment) — no mutant left anywhere. The test binary was
REMOVED and relinked by a full build (20:24) before any result was trusted; the briefing regenerated with the
binary absent and again after it existed. The cut run's only counted mutant verdict (R1 SURVIVED, 23:22) predates
the test that lands rule 4's edge and was discarded; every mutant was re-run (below).

**What the cut run had already done (found in the tree, re-verified, kept):** the frozen-on-the-drive-in stop
(FROZEN BEFORE PUSH), the pneumatic bounds refusal, the pneumatic/HA-201 limits in the banner, `positionChecked`,
the `cancelledPush_` rework (a replaced push is read as RUNNING then marked cancelled — never left RUNNING in the
report), the right wall's `-0` direction printed as `0`; NEW `test/toggle_visit_edges_test.cpp` and
`test/toggle_visit_line_test.cpp`; T11's stop-line check; the docs edits listed per finding. Its probes are in
`scratch/af1s-a1/fix-probes/`; its driver `mut_fix.py` finds each mutated header's dependents from the `.o.d`
depfiles and refuses to count a verdict when any dependent object or the binary was not rebuilt.

**Found on resumption — the frozen fix did NOT hold with the position gate on (fixed at the root).** A probe
(`fix-probes/r10_probe.cpp`, written to judge mutant R10) ran the correctness reviewer's case with a pose bound
stated: at gate tolerance 0.25 and 1.0 in, and 3.0 in with `maxImageAge` 0.3/1.0, the step still PUSHED a Toggle
that truly showed ours (UNKNOWN AFTER or NO CHANGE after 1 push). Cause: the tracker's position gate drops the
frozen picture's band as "no band" once the moved lens projects it off the Toggle's place — before the tracker
can say "camera frozen?" (its frozen check runs only on a band that passed the gate). Fix: with a pose bound
stated the step feeds an UNGATED twin tracker (`frozenWatch_`: the same reads, no bound — exactly the gate-off
path) and reads "camera frozen?" from it, on the drive in (FROZEN BEFORE PUSH) and at the look after (UNKNOWN
AFTER "camera frozen?" where the gated tracker would say "no band"). Re-probed: FROZEN BEFORE PUSH in all 18
gate-on frozen cases. Banner rule 5, guide 19, guide 14 and the changelog say so. Depends on VIS1-A2's tracker
API only as before (ctor, `update`, `view`) — re-check at its merge.

| # | Finding | Disposition |
|---|---|---|
| C1 | push on a Toggle already ours after the camera froze parked | **Fixed** (cut run + this run's gate-on root fix above). Tests: edges "camera FROZEN while parked BEFORE the look" (0 contacts while ours, scores RED), "frozen before push 2's drive in", the two gate-on cases (tol 0.25/1/3 × age 0.1/1; drive-in start / on the way out → `whyAfter` CameraFrozen), the hiccup case. Guide 14 bullet and banner amended. Mutants N1, N2, N7, N8, N9, N10, N11. |
| C2 | pneumatic: rule 4 counts flipBound/backBound, the slot waits its own move time | **Fixed** (refuse: FLIPPER NOT READY "takes longer than flipBound or backBound" when either is under `lastStepBound()`, before the camera is read). Test: edges "a PNEUMATIC's flipBound / backBound against its slot's own wait" (at the bound flips; under it refused). Mutants N3, N3b, N3c. |
| C3 | pneumatic "full stroke"/"back" are the move time, not sensed | **Fixed (documented + pinned)**: banner "What it cannot catch", guide 14 bullet, guide 19, HA-235's pneumatic line. Test: edges T7 PINNED "a PNEUMATIC's stroke is not SENSED". |
| C4 | FIELD-start distance measured to the wall's inner face, lookFrom is to the face | **Fixed (documented)**: banner rule 2, the request's `lookFrom` doc, guide 19 and the register cite HA-201 (face set-back unverified); guide 19's Step 0 list adds the face's set-back. The brief's own Step 0.4 list is the coordinator's copy of record — NOT edited here; for the coordinator. |
| T1 | DRIVE FAILED untested | **Fixed**: edges T7 drive IN blocked / drive OUT blocked. R18 R19 R20 R22 R57 CAUGHT. |
| T2 | MotionGuard cancel untested | **Fixed**: edges T6 "deadline MID-DRIVE-IN" asserts no active motion after the step. R55 CAUGHT — by a CRASH, not by the test's assertion, said plainly: rc -6, "terminate called without an active exception" (SIGABRT) in the first scene that stops a drive (the frozen-parked one), and the cut test ALONE (`-tc='*MID-DRIVE-IN*'`, a fresh `/fast` build, every AF1S object checked newer) crashes the same way — a motion the guard never cancelled is stranded when the step returns. |
| T3 | PASSIVE beyond the happy path | **Fixed**: edges passive retract-now (no service, no crash), passive short drive (never FLIPPED), T8 rows on PNEUMATIC and PASSIVE. R13 R15 (rc -11) R21 X1 CAUGHT. |
| T4 | position-gate wiring unpinned | **Fixed**: edges T1 (R5) — `positionChecked` true only with a bound; a lookalike refused BY THE GATE ("no band") with a bound, reaching containment without. R8, N4 CAUGHT. R10 see below. |
| T5 | retract-now pinned one-sided | **Fixed**: edges "threshold from BOTH sides" (±0.05 s: settle kept / no settle, measured from `back`'s end to the drive out's start) AND, this run, "retract-now landed EXACTLY" (bounds exact in binary, threshold 3.375 s, `remaining()` read back equal to the bit). R2 (`>`→`>=`) CAUGHT by the exact test; X11 R13 CAUGHT. **R12 is EQUIVALENT** (see below). |
| T6 | run-time limits not landed exactly | **Fixed**: rule 4 — this run's "rule 4 landed EXACTLY" (maxPushes 1 so the report keeps the check's own reading; `secondsLeftAtCheck == secondsNeeded` to the bit, proceeds and flips; the next deadline down refused); wall window 44.9/45.1°; field check offBy ∓ 0.05; the arm's at-back tolerance inside/outside. R1 R3 R6 R23 CAUGHT. |
| T7 | settle never observed | **Fixed**: edges HA-232 "settle is honoured" (drive out ≥ back's end + settle − 1 tick, both pushes). X8 CAUGHT. |
| T8 | maxPushes 1, pneumatic readiness | **Fixed**: edges maxPushes = 1 proceed case; T9 pneumatic left at flip / still running; arm still running. R24 R25 R41 CAUGHT. |
| T9 | rule 6 lens clause unreached | **Fixed (reached)**: edges T3 rule 6 — a settle tolerance of 11 in "arrives" at once; UNKNOWN AFTER "no picture after the retract". R4 R36 CAUGHT. Banner re-worded (it said "no host test reaches it"; now names the one that does). |
| T10 | T11 does not check CANCELLED/STALLED | **Fixed**: T11 reads the route's RTN stop line for "STALLED" (sticky) and "CANCELLED" (cut). |
| T11 | mut.py checked 3 objects only | **Fixed**: `mut_fix.py` finds dependents per header from the depfiles (5 for toggle_visit.hpp) and counts nothing with a stale dependent or binary (its first start, 23:19, counted nothing: a deleted probe's stale object was listed — restarted). The original run's verdicts on override_elements.hpp/toggle_plant.hpp were all CAUGHT, so they stand (a stale object cannot fake a red). |
| D1 | roadmap untouched; build-order Next silent | **Fixed**: build-order Next block (AF1S-A1 built, not committed; A2 after A1 and VIS1-A2; the blind flip the 21st's plan); roadmap "Now" note, F15 consumer (AF1S-A1 acts on the tracker), `[~]` line under the Toggle primitive. Briefing regenerated (now 20 `[~]`). |
| D2 | row 18e not split | **Fixed**: 18e says driver control, AF1-A..D after the 21st, no Optical (camera only — "just the ai vision sensor", 2026-11-03), the kind owed by the builder; 18e-S points at the brief (the brief's link resolves once the coordinator commits it). |
| D3 | "waits for L6-A2" untrue | **Fixed**: build-order, REMAINING-WORK 18e-S, roadmap say "after A1 and VIS1-A2 merge (L6-A2 merged, 16c01e0)". DELTAS 16 and 17 below. |
| D4 | manual re-check incomplete | **Done this run** — see "Manual re-check" below. |
| D5 | guide 19 banner stale | **Fixed**: the chapter summary names the skills step; "the automatic flip in driver control … none of them exists yet"; the new section and guide 14 say no team auton can call it until AF1S-A2. Extra prose grep (2026-11-12, run and read): `grep -rn -i 'automatic flip\|auto-flip'` over docs/guide, docs/*.md, docs/cookbook, README.md — guide 19 :15 and :408 (this change's rewording, both now "in driver control"); roadmap :149 (F15, updated), :395-399, :498, :1259 and the master plan :81, :619 (the plan and its history — true of AF1-A..D); changelog :414 (history); the register :39/:1919 (HA-236 held for AF1). None stale. |
| D6 | guide 11 TGL lines not pinned | **Fixed**: `test/toggle_visit_line_test.cpp` makes each of the twelve lines from a scene, byte for byte through the sink's `[t=…]` format, and fails if guide 11 quotes anything else; the chapter intro names it; a TGL bullet in "lines worth recognizing". N5 (the `-0` wall direction) CAUGHT by it. |
| D7 | prose slips | **Fixed**: (a) "`face(...)` or `turnTo(...)`"; (b) "within `offBy`"; (c) "at most mounts (a high, tilted lens can)"; (d) the 6/8/12-in figures labelled "an earlier build of the tracker — not re-run since, and not pinned" (the 10-in figures re-run and pinned); (e) HA-244's checklist no longer cites the internal probe P7b (grep empty). |
| D8 | merge-time items unlogged | **Logged**: "For the merge" below. |
| D9 | §2.2 expectation changes need the coordinator's ruling | **Logged for ruling**: "For the coordinator's ruling" below. (a)'s plant mode NOT built: a push deadline that outlasts the threshold is what rule 4 forbids — a stall large enough to cross the threshold also expires the push's own wait, so the plant cannot show a flipper out at 0:00 without a request rule 4 would refuse; the exact-edge test and the pair now pin the threshold itself. |

**Mutants that SURVIVED on the final tree, said plainly:**
- **R12** (settle kept after retract-now, `if (!retractForTime_)` → `if (true)`): EQUIVALENT. The settle wait's own
  predicate is `retractForTime_ || now() >= until`, and `RunGuard::waitFor` → `MotionScheduler::waitUntil`
  evaluates the predicate BEFORE the first tick (motion_scheduler.hpp :836-840), so with `retractForTime_` true the
  wait returns at once with zero ticks — the guard is redundant (kept: it says what happens). R13 (the predicate's
  `retractForTime_ ||` dropped) is the meaningful mutant, and is CAUGHT.
- **R10** (speed left out of `boundAtImage`'s widening): no step DECISION depends on it — the counted looks are
  parked (speed ~0), and the frozen check now reads the ungated twin. Probes (`fix-probes/r10*_probe.cpp`, the
  original against an overlay mutant): every verdict, colour and push identical across tol 0.25/1/3, ages
  0.1–1.0 and lags 0.1–0.6 s; the ONE difference is WHEN a look after counts with a 0.3-s camera lag and a 1-s
  hold (9.88 s correct vs 9.56 s mutant — the correct code is the slower, more careful one: with the bound widened
  the gate is skipped while moving, so the hold that counts is rebuilt once the gate can run). NOT pinned: it would
  pin the timing of VIS1-A2's in-flight tracker internals, not a rule of this step. For the coordinator.

**Manual re-check (D4, 2026-11-12).** Newest manual: **v2.1** (the manuals.vex.com landing page lists Override
v2.1 and nothing newer). Read on the web: v2.1 Section 3 (`/v5/override/v2.1/skills/`) — `<RSC2>` (start in the
Quadrant by the red station; drive team in the red station; loaders red) and `<RSC3>` a (yellow Pin HALVES are
Owned "if the Toggle in that Quadrant matches the color of its Quadrant") — the same in substance as v2.0 :2020-2049
(Pins → Pin Halves); v2.1 Section 6 `<VURS2>` (both robots in skills) and `<VURS3>` (both start in red legal
positions, each a red/yellow Preload); v2.1 `<SC4>` (set = fully seated, no robot contact) — unchanged. Figure
Q-1 and Appendix A pA15 were NOT reachable in v2.1 on the web (the glossary URL lands on the manuals index); read
in v2.0's drawings (`scratch/autoflip/img/gl-122.png`, `a-110.png`): Q-1 — red station on the left edge, Red 1
against it, Red 2 on the wall to the red station's right, Blue 1 opposite, Blue 2 on the far side wall; pA15 —
Red-Alliance-side Toggles red facing out, Blue-side blue facing out, yellow up and in. The table (red wall RED,
right wall RED, blue wall BLUE, left wall BLUE) agrees. The comments already name their version (toggle_visit.hpp
"v2.0 <RSC3>a-iii-1 … v2.1's change log keeps it in substance", override_elements.hpp "v2.0 Figure Q-1 … pA15");
no comment changed by this re-check.

**DELTAS added by the review round (continuing the list above):**
16. The brief's coordination facts have moved since it was written: L6-A2 is merged (16c01e0, in this base), MECH-S-A
    and CAL1 merged, STEP1-A merged to shulib-v2 after this base (API 2.6). The status lines say so.
17. HA-236 (AF1's flipper-stroke number) is NOT registered here though D10/§4 list it among the five to reuse: the
    register keeps it HELD for AF1 (its stroke belief is the driver-control flipper's; this step's stroke beliefs are
    HA-231 and HA-235). Four of AF1's five registered: 231, 232, 234, 235.
18. Verdict `FrozenBeforePush` appended (a fourth beyond the brief's list).
19. `ToggleVisitReport::positionChecked` added (whether the position gate ran on the look before).
20. A pneumatic flipper's `flipBound`/`backBound` under its slot's own wait (move time + confirm window) is FLIPPER
    NOT READY (the brief allowed any positive bound).
21. With a pose bound stated, an UNGATED twin tracker reads "camera frozen?" (this run's root fix above).

**For the merge (the coordinator's, at rebase onto shulib-v2):**
- The changelog entry sits at the top of `## API 2.5`; shulib-v2 now has `## API 2.6` (STEP1-A) — move it there.
- The register's status total says 215 here; shulib-v2 is at 209 after STEP1-A's HA-216, so the merged total is
  216 (7 registered here). The Held line conflicts with shulib-v2's reworded STEP1 clause ("HA-216 registered by
  STEP1-A; HA-217..HA-218 held for STEP1-B") — rebase it.
- STEP1-A has merged: brief §6.1's T11 onFail cases (carryOn after NoChange, skipTo landing, a Cut stopping whatever
  the choice) and cookbook-08a's `onFail(skipTo(...))` are owed once rebased (today both pin the default stop).
- The four links to `19-the-camera.md` anchors (guide 11 ×2, guide 14, the changelog) must be rewritten when
  chapter 19 is renumbered at VIS1-A2's merge.
- VIS1-A2's tracker API this step depends on (listed above) — plus, now, a second `ToggleTracker` per visit.

**For the coordinator's ruling (§2.2 expectations this change shows differently — each logged above):**
(a) MA12/MA12b turn T6 red by the verdict (CUT, not RETRACTED FOR TIME), not by a flipper out at 0:00 (unreachable
on this plant, see D9); (b) MA24 is caught by the reads-while-driving test, not by T3's "frozen at the look spot"
case (that case is now FROZEN BEFORE PUSH); (c) T1's "0 contacts while ours" holds for a flip BEFORE the look (now
also with the camera frozen over it, gate on or off); a flip during the drive in is PINNED as a stated limit (guide
14); (d) T14 at 70 degrees gives UNKNOWN AFTER, not FLIPPED (pinned per angle); (e) the position gate did not refuse
the lookalike 4 in in front of the face (pinned; the gate refuses one 6 in out with a 0.25-in tolerance); (f) R10
and R12 as above.

**Tests added on resumption (2026-11-12), all in `test/toggle_visit_edges_test.cpp`:** rule 4 landed EXACTLY
(maxPushes 1; `secondsLeftAtCheck == secondsNeeded` to the bit — the deadline walked by `nextafter` until it lands,
`look + need` landed at once — FLIPPED; the next deadline down refused); retract-now landed EXACTLY (bounds 1.25 /
0.5 / 1.5 / 0.125 s so the threshold is 3.375 s exactly; the clock jump solved so `remaining()` read back at the next
check IS 3.375 → RETRACTED FOR TIME); the camera frozen parked before the look WITH the gate on (tol 0.25/1/3 in ×
maxImageAge 0.1/1 s → FROZEN BEFORE PUSH, 0 contacts while ours, scores RED); the gate on with a freeze as the drive
in begins (FROZEN BEFORE PUSH) and on the way OUT to a 20-in look spot (UNKNOWN AFTER, `whyAfter` "camera frozen?" —
the gated tracker alone said "no band": probe `fix-probes/n10_probe.cpp`); a camera that HICCUPS (frozen 0.2/0.5 s at
each drive in's start, under maxImageAge 0.6) → FLIPPED (probe `fix-probes/n8b_probe.cpp`: the mutant N8 stops push
2 as FROZEN BEFORE PUSH). Suite 3340 → 3345 cases.

**Mutations on the final tree** (`scratch/af1s-a1/mut_fix.py`, results `mutations_fix.log` / `mut_fix_final.out`;
each on a fresh `shulib_tests/fast` build with all 5 dependents of `toggle_visit.hpp` (from the depfiles) and the
binary checked newer than the mutation, run as `AF1S*,cookbook*` under 4 GiB / 600 s, restored in a finally and on
SIGTERM/SIGHUP/SIGINT, the file compared with its pristine copy afterwards — identical): 43 mutants — the tests
reviewer's 25 (R1–R57, X1/X8/X11), the fix round's N1–N11 (N9–N11 this run's twin), the brief's MA6/MA9/MA11/MA23/
MA25 re-run. **41 CAUGHT** (R15 by SIGSEGV and R55 by SIGABRT, said above; every other by a failing assertion — the
red test named on each line), N10 caught after its scene moved to a 20-in look spot (re-run with N1, N9, N11 on
fresh builds: all CAUGHT). **2 SURVIVED: R12 (equivalent) and R10 (no decision depends on it)** — both above.
No mutant broke its own build.

**FINAL GATES (the tree as left, 2026-11-12 ~21:18):** test binary removed and relinked by a full build (20:43, 0
warnings); briefing regenerated before the build and after the binary existed; suite `green — 3345 cases, 3098063
assertions` (through `tools/suite_run.py`); `ctest -R compile_refusal`: 266/266 (266 TUs); ARM header gate rc 0 (248
headers in one TU); CI guards (pros/, sim/) empty; `src_build_gate.py self-test` OK (269 cases) and `check` PASS (7
builds) — `git diff -- Makefile src/` empty afterwards, no self-test plant; api_doc_tool self-test / check-coverage
(5837 entities, 204 headers) / check-fresh (rc 0) / check-examples (763 lines) / check-removability OK;
doc_staleness_audit self-test OK and clean; briefing_status self-test OK and check current. The brief's §7 greps:
every one as specified (§1.6 files, src/, Makefile, tools/ unchanged; no AF1-A include; layering; nothing of vision/
in the plant; no waitUntil and no blocking verb in the step; `seating not checked` present; HA-231/232/234/235/
243/244/245 each cited and each with its row, HA-236 held; no unlabelled PROVISIONAL; removability empty).
- STOPPED HERE (2026-11-12): AF1S-A1's review findings dispositioned, NOT committed. For the coordinator: the
  rulings and merge items above, and a fresh verifier.

### Verification-round findings and their disposition (2026-11-12, the builder again)

A fresh verifier returned three findings on the tree above. Each re-checked; all three real, fixed at the root.
The pre-fix header is kept as `scratch/af1s-a1/verify/pre_vfix_toggle_visit.hpp`.

| # | Finding | Disposition |
|---|---------|-------------|
| 1 (major) | C1 held only in part: with `maxImageAge` longer than the time from "camera frozen?" being raised to the drive in's end, the ARM pushed a Toggle already ours (the verifier's `verify/v1_probe.cpp`: ages 1.5/2.0 s, gate off and on → UNKNOWN AFTER, 1 contact while ours, scored BLUE on a red Quadrant); docs understated it | **Real, and WORSE for the passive bar — fixed at the root.** Re-checked with the verifier's probe on the old header: confirmed. A new probe (`verify/v3_probe.cpp`, the same scene with `passiveRequest()`, built against the PRE-fix header through `verify/old_inc/`) showed the PASSIVE bar already failing at age **1.0** s: the verdict said FROZEN BEFORE PUSH ("push 1 not made") while the bar had rolled the Toggle that was ours to BLUE (1 contact while ours) — the drive in IS a passive bar's stroke, so stopping on the stale evidence came after the contact; at 1.5/2.0 s UNKNOWN AFTER. **Fix (`toggle_visit.hpp`, new `driveIn()` / `holdOnFrozen()`, `drive()`'s `FrozenWatch`):** the step HOLDS before anything reaches the Toggle while "camera frozen?" is RAISED but not yet stale. An ACTIVE flipper: the drive in still stops on the stale evidence; when it ARRIVES with the question raised, nothing is pushed — it keeps reading (the camera, the retract-now check) up to ceil(maxImageAge): stale, or still open when that budget runs out → FROZEN BEFORE PUSH (drive out); the question cleared (a lagging live camera caught up) → push. A PASSIVE bar: the drive in stops as soon as the question is RAISED (cancelled through the same MotionGuard), holds the same way, and — the camera believed again — RESUMES with only what is left of driveBound (the moving time of both legs ≤ driveBound); out of drive time → TimedOut (no full stroke). The hold budget is per drive in, spent across its holds; rule 4's need(n) now adds ceil(maxImageAge) a push (the verifier's "count that wait in need()" — chosen over refusing a long `maxImageAge`, because the drive in's real time is not known to the request: only `driveBound`, its limit, is). Re-probed: `v1_probe` → FROZEN BEFORE PUSH, 0 contacts, ages 0.1–2.0, gate off and on (the arm at 1.5/2.0 s now arrives SETTLED and the hold catches it); `v3_probe` passive → FROZEN BEFORE PUSH, 0 contacts, every age, gate off and on; passive hiccup → FLIPPED with one resumed drive in. **Found while fixing (a stated limit, pinned):** a passive bar that reaches the Toggle before the lens has moved `frozenMoveIn` (before a frozen camera CAN look frozen) strokes it on the stale look and the verdict still says FROZEN BEFORE PUSH (`verify/v4_probe.cpp`: contact from 14–15 in on a 16-in look spot with `frozenMoveIn` 2 in → 1 contact while ours, both before and after this fix; from 12 in it is stopped in time). Not fixable inside the step (it does not know where a bar first touches); stated in the banner's "cannot catch" and guide 14, pinned by a test that a fix would turn red. **Docs:** the banner's rules 4 and 5 (the old "with a maxImageAge as long as the drive in this cannot fire" sentence replaced by the hold) and its "cannot catch"; the request's `maxImageAge` comment; `toggleVisitNeedSeconds`'s comment; guide 19 (the hold paragraph, "The time" names the hold, Step 0's `maxImageAge` now says "the longest a lagging picture trails the robot, not a guess 'to be safe': every push's time grows by it"); guide 14 (the hold; the old "never fires (the look after still catches it)" replaced by the passive early-contact limit); guide 11 (`not now` names what `n` sums; its example line `needs 12.9s` → `needs 13.1s`, the line test's copy with it — the rig's two pushes × 0.1 s); the changelog's Added bullet; docs/api regenerated. **Tests (edges):** the frozen-parked sweep ARM / PNEUMATIC / PASSIVE × gate off/on × ages 0.1/1.0/1.5/2.0 (0 contacts while ours, 2 motions, the active forms at ≥1.5 s `driveIn == Settled` — the hold, not the stop, caught it; passive `Cancelled`); the arm's hold with the camera LIVE again 0.1 s after arrival (age 1.5) and 1.75 s after (age 3 — past half the budget): the push only after the camera is live, within 0.2 s of it, FLIPPED; the passive pause → hold → resume (ages 0.6/1.5 × hiccups 0.2/0.5: FLIPPED, full strokes, no contact while frozen, 5 motions); the passive budget spent across TWO holds (re-frozen 0.05 s into the resumed leg → FROZEN BEFORE PUSH, the Toggle untouched, 3 motions); the passive resumed leg given only what is LEFT of driveBound (`frozenMoveIn` 5, driveBound 1.2: both legs' moving time ≤ 1.2 s + a tick, TimedOut, no full stroke — probes `verify/v5_probe.cpp`, `v6_probe.cpp` found the setting: the resumed leg needs 1.05–1.10 s, the real budget 1.0, a fresh one 1.2); the pinned passive early-contact limit (12/14/15 in × ages 0.1/1.5). Rules T13 now sums the hold (7.54 / 3.92 on the grid; 0.101 → 0.11 off it). |
| 2 (minor) | the changelog said a pneumatic is refused when a bound is "shorter than its move time"; the code refuses below move time + confirm window | **Real — fixed:** the changelog now says "shorter than its move time plus its confirm window (the most its slot waits)", matching the banner and guide 19 (`service.hpp` :1719-1721's `lastStepBound()`). |
| 3 (minor) | HA-245 still stated the 6/8/12-in lens figures without the D7(d) caveat | **Real — fixed:** the register's HA-245 row AND its checklist entry now say only the 10-in lens's figures are re-run and pinned by this release's tests; the 6-, 8- and 12-in ones come from an earlier build of the tracker, not re-run since — as guide 19 says. |

**Delta from the brief, logged:** rule 4's need(n) gains a term (ceil(maxImageAge) a push) the brief's §1.1 sum does
not have; the passive bar's drive in may now be TWO motions (stopped, resumed) where the brief has one. Both follow
from R3's own aim (nothing pushed on a look the camera itself doubts). **Depends on VIS1-A2's tracker** (for the
coordinator's merge re-check): the hold reads `ToggleWhy::CameraFrozen` from the tracker (or its ungated twin)
as RAISED — the tracker's frozen check must still raise it on the first read where the lens has moved past
`frozenMoveIn` with an unmoved box, and clear it on the first read whose box moves; and a passive resume relies on
the question not re-raising while the robot stands still.

**Mutations of this round** (`verify/vmut2.py`, the previous driver's machinery — a fresh `shulib_tests/fast` build,
every one of the 5 dependents of `toggle_visit.hpp` from the depfiles and the binary checked newer than the
mutation, `AF1S*,cookbook*` under 4 GiB / 600 s, restored in a finally and on SIGTERM/SIGHUP/SIGINT, the file
compared with its backup afterwards — identical; results `verify/vmut2.log`, `vmut2_run{1,2,3}.out`): **9 mutants,
9 CAUGHT.** V1 (no hold at arrival — the finding's code) by the frozen-parked sweep and the arm's live-again test;
V2 (passive stops only on stale evidence) by the sweep, the pause/resume test and the pinned limit; V4 (need()
without the hold) by T13 and guide 11's line test; V5 (passive never resumes) by the pause/resume test; V6 (the hold
not ended by the question clearing) by the arm's "within 0.2 s" check; V7 (the resumed leg given a fresh
driveBound — SURVIVED its first run, a test added: the resumed-budget test, then CAUGHT by it); V8 (the frozen
check on the drive in dropped, the old N1 re-cut for the new signature) by five tests; V9 (the hold budget halved)
by the arm's age-3 case, the pause/resume test and T3 (R8, MA6, MA25). **V3** (a hold that runs out with the
question still open believes the camera) is CAUGHT by a CRASH, not by an assertion — said plainly: rc -6,
`std::bad_alloc` under the 4 GiB cap, in the frozen-parked sweep's passive case: the mutant resumes, the resumed
drive pauses on its first read with no hold budget left, and the loop spins without the clock moving. The real
code cannot spin: a resume follows only a hold that CLEARED the question, and with the budget spent every further
pause ends FROZEN BEFORE PUSH (the clause V3 removes). That clause also covers the hold ending on the same read
the evidence turns stale (readAt - maxImageAge >= frozenSince_ can miss by an ulp at exactly ceil(maxImageAge)).
The previous round's N1 pattern no longer exists (the signature changed): re-cut as V8. No mutant broke its own build
(V7 re-cut BEFORE its run to `moveLeft -= 0.0 * (now() - t0);` — deleting the line would have left `t0` unused,
a -Werror build failure).

PROSE REVIEW (this round — the greps run and their hits READ before this line was written):
- `grep -rn -i 'maxImageAge'` over docs/*.md, docs/guide, docs/cookbook, README.md, test/README.md: the changelog
  :46-47, guide 19 :358-396, guide 14 :424-427 — every hit this round's text or still true.
- `grep -rn -i 'worstCaseSeconds\|need(n)\|needs [0-9.]*s'` over the same: guide 19 :379 (names the hold), the
  changelog :47/:52, guide 11 :499 (13.1 s, the line test's copy) and :529 — all current.
- `grep -rn -i 'as long as the drive in\|never fires\|cannot fire'` over docs, include, test: none about this step
  (the STEP1 brief's triggers, the register's other rows, history) — the old sentence is gone everywhere but the
  regenerated api page's history-free copy (regenerated).
- `grep -rn '6–12 in up\|12 in up'` over docs: guide 19 :339 (caveated) and the register's HA-245 row and checklist
  (now caveated alike).
- `grep -n 'move time'` in the changelog: :48 (fixed); the rest are other chunks' entries.

**GATES OF THIS ROUND (the tree as left, 2026-11-12 ~22:12):** test binary removed, briefing regenerated against
the unbuilt binary, docs/api regenerated, full build (0 warnings), briefing regenerated after the binary existed;
suite `green — 3351 cases, 3105975 assertions` (through `tools/suite_run.py`, 4 GiB / 900 s); `ctest -R
compile_refusal` 266/266; ARM header gate rc 0 (248 headers in one TU); CI guards (pros/, sim/) empty;
`src_build_gate.py self-test` OK (269 cases) and `check` PASS (7 builds) — `git diff -- Makefile src/` empty
afterwards, no self-test plant; api_doc_tool self-test / check-coverage (5837 entities, 204 headers) / check-fresh /
check-examples (763 lines) / check-removability OK; doc_staleness_audit self-test OK and clean; briefing_status
self-test OK and check current. (Logs: `scratch/af1s-a1/verify/g_*.log`; the script `verify/gates.sh`.)
- STOPPED HERE (2026-11-12): the verification round's three findings fixed, NOT committed. For the coordinator:
  the VIS1-A2 dependency named above (the hold reads "camera frozen?" RAISED), and the brief deltas (need(n)'s new
  term; a passive drive in may be two motions).

## AF1S-A1 — the coordinator's merge verification *(2026-11-13)*

- **Applied as its OWN diff** (`542b5f6..` its transport — its base was main e458dc6 plus VIS1-A2's code as an overlay) onto
  `2c3aa34` (VIS1-A2 landed), with every `19-the-camera.md` path rewritten to `20-the-camera.md` first (the camera chapter was
  renumbered at VIS1-A2's merge). Every code file applied cleanly. Documents resolved by hand: guide 14 (the camera's section,
  then the Toggle step's), test/README (the camera rig, then the Toggle plant), the roadmap's F15 row (AF1-S's step named as
  the first to ACT on the tracker, landed) and its "Now" paragraphs, REMAINING-WORK rows 18d–18e-S (a word-level three-way
  merge; AF1S-A1 LANDED), the register (Held line: AF1's HA-228..230, 233, 236 and ALLY1/L4's HA-237..242 held; 7 of **223**
  settled; AF1S-A1's "added SEVEN" paragraph — HA-231/232/234/235 reused, HA-243..245 new; the index's 223 rows sorted), the
  changelog (AF1S-A1's entry moved to the top of `## API 2.8`; no bump), guide 11 and the changelog's old camera links, the
  generated pages (regenerated). Status words: build-order, REMAINING-WORK, the roadmap's line and checklist entry.
- **Gates on the merged tree** (`.claude/coordinator/scratch/af1s_a1_merge_verify.sh`): the suite green (3,627 cases);
  `src_build_gate.py` self-test OK and check PASS (all 8 builds); the ARM header gate CLEAN; the doc gates and the briefing
  check clean; the compile-refusal harness 280 of 280; no plant, no stray root file.
- **The coordinator's own mutations** (`muts_af1s_a1_main.json`): **4 of 4 RED** — MT1 a Toggle already showing ours pushed
  anyway, MT2 a short stroke that then reads ours reported FLIPPED, MT3 a picture from before the retract's end counted as the
  after-look, MT4 the after-look accepted without backing off past the frozen move. The restored tree relinked and ran green.
- **AF1S-A1 LANDED.** Nothing has run on a robot; no Toggle, flipper or camera reading of a real Toggle exists. Next:
  AF1S-A2 (the camera reachable from an auton), then a team's skills route can call `checkAndFlipToggle`.

## AF1S-A2

**Base:** worktree `.claude/worktrees/af1s-a2`, branch `af1s-a2`, at `9befdb3` (main: API 2.8 — L6-A2, VIS1-A2, AF1S-A1,
STEP1-A/B, ROB1-A, the homing fix 49822e5, RPV1-A all landed). Started 2026-11-15 18:53 EST (`date` read). Executor
model: `opus`. Nothing committed by the executor. Host only: nothing here has run on a robot.

The between-parts release marker at the top of this file was REMOVED as this run started (the coordinator restores or
completes it at the merge).

### Log
- Created this section first.

- Read RESUMING.md's standing rules; the brief's header and answers block, "Re-check — amendments" R1–R12, D1–D11,
  §1.1/§1.3/§1.5/§1.6, §2.0–§2.3, §3, §4, §5, §6, §7, §8, §9 in full (§0 by headings — the ruling of 2026-10-22);
  AF1S-A1's record above in full. Read the code A2 touches: L6-A2's slot pattern in `chassis/robot_context.hpp`,
  `robot/robot.hpp`, `config/robot_config.hpp`, `motion/motion_scheduler.hpp`, `competition/auton_wall_reset.hpp`;
  A1's `competition/toggle_visit.hpp`, `test/toggle_visit_rig.hpp`, cookbook-08a; VIS1-A2's `vision/camera_feed.hpp`,
  `toggle_tracker.hpp`, `camera_geometry.hpp`. In-flight MIRROR1-A's diff to `toggle_visit.hpp` read (read only).
- Base build (`cmake -S test -B build/test`, full build, 0 warnings) and the suite through `tools/suite_run.py green`
  with the first code edits in place: `green — 3701 cases` (the base's own count — nothing broke).
- DELTA (found writing the code): **the context's slot cannot be "null on any refusal"** (the brief §1.3). The motion
  scheduler's shadow context copies every slot pointer when the CHASSIS is built — in robot::Robot's member
  initializers — and the camera is built in the Robot's BODY, last (VIS1-A1's order). So, exactly L6-A2's wall slot
  (present when DECLARED; a side ABSENT inside it): the context's pointer is set from the CONFIG (the Toggle block
  declared), the object (`vision::ToggleCamera`) is filled in the body, and `ready()` says whether it was built. The
  brief's "built, or null, with its sentence" is `robot::Robot::toggleCamera()` / `toggleCameraAbsence()`. The
  everyday step reads a present-but-not-ready slot as NO CAMERA with the slot's own sentence (better than "no camera":
  the [TGL] line says WHY).
- DELTA: **the Toggle block sits BESIDE the camera's block** (`config::SensorsConfig::toggleCamera`, type
  `vision::ToggleCameraConfig`), not inside `config::CameraConfig` (§1.3): `config/camera_config.hpp` is included by
  `vision/toggle_tracker.hpp` and `vision/camera_feed.hpp`, so CameraConfig cannot hold their types — a circular
  include (R9's lesson for the Quadrant colours). One NEW header, `vision/toggle_camera.hpp`: the block, its ONE check
  order (`toggleCameraCheck` / `toggleCameraProblem`, constexpr — a robot file may static_assert it), and the slot.
- DELTA: the slot holds the tracker's CONFIG, not a `ToggleTracker` (§1.3 named the tracker): A1 builds a FRESH
  tracker per visit (its delta 1), so a stored tracker would be unused.
- DELTA (added check): the tracker's three colour ids must be colours the camera's settings WRITE — an id the camera
  is never taught reads UNKNOWN for ever (a typo nothing else catches).
- DELTA (added): an optional third argument to the everyday form, `competition::TogglePoseBound` — without it the
  robot file's position gate (D6 (c), recommended after a wall reset) could never run from an auton (the explicit
  form's `poseTolerance` 0 = none stated). Default = none (the gate skipped), so the brief's two-argument form is
  exactly the brief's. Also `lookAtToggle(ctx, ...)` (the everyday look-only form). `ExplicitToggleCamera` gains an
  APPENDED `absence` (the NO CAMERA line's words).
- The slot asks the provider for NOTHING (it uses the context's clock and the camera built above): the boot's
  requests are unchanged request for request (a test pins it).
- LESSON (re-learned from A1's log): `make shulib_tests/fast` skips CMake's dependency consolidation, so a HEADER
  edit did not recompile the objects including it — an ODR mix of old and new `Robot::cameraWords` made a test fail
  for a reason that was not in the code. Every dev build since runs the `depend` step first
  (`scratch/af1s-a2/devbuild.sh`); every mutation verdict will check every dependent object was rebuilt.
- Code (2026-11-15): NEW `vision/toggle_camera.hpp` (`ToggleCameraConfig` + `declared()`, `ToggleCameraCheck`,
  `toggleIdsProblem`, `toggleCameraCheck` / `toggleCameraProblem` — ONE check order, constexpr —, `ToggleCamera` the
  slot, `kToggleCameraReasonBytes`); `chassis/robot_context.hpp` (the forward declaration, the appended
  `RobotContextConfig::toggleCamera`, `RobotContext::toggleCamera()`); `motion/motion_scheduler.hpp` (ONE line: the
  shadow context carries the slot — the comment beside it says "Toggle camera" so §7's `grep -n 'toggleCamera'` finds
  exactly that line); `config/robot_config.hpp` (`SensorsConfig::toggleCamera`, appended; `valid()`'s note: the block
  is NOT checked there); `robot/robot.hpp` (the banner's TOGGLE CAMERA section; `toggleCam_` declared before the
  context; the context's slot from the config; `toggleCam_.build(...)` LAST in the body with the CONTEXT's clock — no
  provider call; `toggleCamera()` / `toggleCameraAbsence()`; the boot line's `, toggles ready` / `, toggles OFF (why)`
  only with a block declared, appended BY HAND after the camera's own words, which stay bounded at their old 224
  bytes — `kCameraOwnWordsBytes`); `competition/toggle_visit.hpp` (the banner's "Two forms, ONE body";
  `ExplicitToggleCamera::absence` appended; rule 1's NO CAMERA takes it; `TogglePoseBound`; `detail::toggleCameraOf`;
  `checkAndFlipToggle(ctx, request, bound = {})` and `lookAtToggle(ctx, request, bound = {})` — both run the SAME
  `detail::ToggleVisit` as the explicit form).
- Tests: NEW `test/toggle_robot_rig.hpp` (a real Robot over the fake provider, the competition program's chain,
  the provider's fake camera fed the host camera model's frame from the TRUE pose every pace; the recipe robot's
  `toggle_robot::flipper` / `cascade`), `test/robot_toggle_camera_test.cpp` (10 cases), `test/toggle_visit_ctx_test.cpp`
  (9 cases); `test/cookbook_examples_test.cpp`'s cookbook-08a REWRITTEN in the everyday form (A1's version over the
  explicit form replaced, as A1's log said it would be). Suite: `green — 3720 cases, 3864009 assertions` (3701 + 19).
- DELTA (tests): `test/boot_source_test.cpp` and `test/robot_camera_test.cpp` (§1.5) are NOT changed: their pins are
  robots with no Toggle block, whose boot line is unchanged byte for byte; the Toggle words are pinned in
  `robot_toggle_camera_test.cpp`, which also pins that a block-less line equals the block's line less its words.
- DELTA (the recipe's robot): the cookbook's robot is the TEST's (`toggle_robot`, a motor-arm flipper and a stall-homed
  two-motor cascade with a measured shape — RPV1-B1's cascade-like declaration), not robot one's (robot one's camera
  block, tracker numbers and flipper are G1/ROB1-C's — the task's ruling). The recipe's cascade starts 3 in up at boot
  so its home (~2.3 s) outlasts the first leg (~1.6 s): without `waitUntilDone` the move would replace the home
  (M-RCP below proves the order).
- DELTA (docs): a NEW figure, `docs/assets/skills-toggle-look-spots.png`, generated by a NEW
  `tools/docs_figures/skills_toggle_route.py` over the manual's field (the team lead's 11-13 "docs need visuals" rule,
  newer than the brief) — so §7's `git diff --stat -- tools/` is NOT empty: exactly this one new docs-figure script.
- Docs (same change): changelog (top of `## API 2.8`); NEW `docs/cookbook/08-skills-toggles.md` (+ its README row, +
  mkdocs nav "Skills routes"); guide 10 (a section "The Toggle block, and the skills Toggle step from an auton";
  "its one OPTIONAL slot" → "first"; the Covers line); guide 15 (look spot, push spot, Quadrant colour, Toggle block);
  guide 20 (the banner's "What exists", the section's "No team auton can call it yet" replaced, a NEW subsection "The
  robot file's Toggle block", a pointer under "The team's numbers"); guide 11 (NO CAMERA's reasons); guide 14 (its
  "No team auton can call it yet" replaced); test/README.md (the new rig); the internal worksheet (NEW Station T —
  Step 0.1–0.7); roadmap ("Now" note, the `[~]` AF1S-A2 line, F15's file list); build-order Next block; REMAINING-WORK
  18e-S; `docs/api/` regenerated (NEW `toggle_camera.md`); PROJECT-BRIEFING regenerated.
- Public docs touched: SEVEN (changelog, cookbook 08 + its README row, guides 10, 11, 14, 15, 20) plus test/README and
  the roadmap — over rule 7's ~5: guides 11, 14 and 20 each said "no team auton can call it yet" / NO CAMERA's old
  reasons, which this run made false (rule 4: fix what is stale, in the same change). NOT touched, deliberately:
  guide 21 ("No camera ... nothing reads it" on robot one — still true: robot one's camera block is G1/ROB1-C's);
  `docs/hardware-assumptions.md` (no new belief: A2 adds no HA; the §4 rows are A1's).
- MUTATIONS (`scratch/af1s-a2/mut.py`; results `mut/mutations.log`, `mut/run1.out`, `mut/run2.out`, per-mutant
  outputs `mut/out_<id>.txt`, judged by `scratch/af1s-a2/analyze.py` — a case counts RED only by an ERROR/FATAL
  assertion under its own header). Each mutant: the file backed up in scratch, ONE exact replacement (matched once),
  mtime bumped, CMake's `depend` step THEN the build (never `/fast` alone — the ODR lesson above), every object whose
  depfile names the mutated file (nested src/ objects included: 145 for motion_scheduler.hpp, 111 for
  toggle_camera.hpp, 29 for robot.hpp, 7 for toggle_visit.hpp, 1 for the cookbook test) AND the binary checked newer
  than the mutation (none was stale; no verdict was discarded), the WHOLE suite binary run under 4 GiB / 600 s,
  restored in a finally and on SIGTERM/SIGHUP/SIGINT, mtime bumped, and compared byte for byte with its backup
  (identical, every time). No mutant broke its own build. **20 mutants, 20 CAUGHT by assertion:**
  - **M-A2-1** (the shadow context's line dropped): 10 cases red — cookbook-08a, the Robot's "slot READY" and "each
    refusal" (the chassis's slot is not the Robot's), every ctx case (the slot from inside the auton is nullptr; the
    RELATIVE-start skills auton does not flip — NO CAMERA). The FIRST run also ended in a SIGSEGV: the "TWO forms" case's
    explicit helper dereferenced the null slot — MY TEST's fragility, not a verdict. Fixed (the helper records the slot
    and never dereferences a null one; the case REQUIREs it non-null) and re-run on a fresh build: CAUGHT by assertion,
    no crash.
  - **M-A2-2a** (a refused block REFUSES THE ROBOT: the constructor throws) — 9 cases red (each refusal, the order,
    declared(), the build-time check's agreement, valid(), the boot line, the ctx MATCH / NO CAMERA / refused-camera).
  - **M-A2-2b** (valid() refuses a declared block with a problem) — 8 cases red (the same, by the config's door).
  - **M-A2-3** (the slot built on an UNMEASURED or past-the-limit mount) — 3 cases red (each refusal, the order, the
    boot line). The ctx tests do NOT catch it, and should not: the step's own rule 1 refuses an unmeasured mount with
    the same sentence (defence in depth) — the slot's readiness is the Robot's, pinned in robot_toggle_camera_test.
  - **M-RCP** (the recipe's `r.waitUntilDone(cascade, 4_s)` dropped) — cookbook-08a red: the route STOPS at step 4
    (`cascade`), the move sent while the cascade was still homing (the home ~2.3 s, the first leg ~1.6 s).
  - X1 (the colour-id cross-check dropped) — each refusal, the order. X2 (declared() misses the feed's period) — the
    declared() sweep. X3 (the everyday form ignores the pose bound) — the pose-bound case. X4 (the everyday NO CAMERA
    loses its reason) and X5 (rule 1 ignores the caller's words) — the NO CAMERA case and the refused-camera case.
    X6 (the Toggle words with no block) — 15 cases, robot_camera_test's / builder / L6 boot lines among them. X7 (the
    camera's own words unbounded) — the boot-line case. X8 (Robot::toggleCamera() hands out a slot not ready) — 3.
    X9 ("the camera was not built" before the file's checks) — each refusal, the order. X11 (the context's slot handed
    in with no block) — the no-block case. X12 (the everyday form decides NO CAMERA before rule 1) — the MATCH case and
    the NO CAMERA case. X13 (the slot without the block's tracker numbers) — 7. X14 (the slot fillable twice) — the
    object case. X15 (the everyday form drops the block's position gate) — the pose-bound case.
  - **X10** (the slot asks the provider for its clock — `devices.clock()` for `ctx_.clock()`) SURVIVED its first run:
    the provider-request test compared the requests WITH a block against those WITHOUT one, and the fill runs in
    both, so any request it makes appears in both lists alike. The test now also requires NOTHING after the camera's
    request in either list; re-run on a fresh build: CAUGHT (`with.back()` read `clock 0`, not `aiVision 3`).
- PROSE REVIEW (rule 4 — the greps run and their hits READ before this line was written):
  - `grep -rn -i 'robot_context|RobotContextConfig|optional slot|one optional|only optional|first optional'` over
    docs/*.md, docs/guide, docs/cookbook, README.md, test/README.md: guide 10 :662 said the wall slot was the
    context's "one OPTIONAL slot" — FIXED ("first"); roadmap :1339/:1340/:1749/:1760 (history of R3b and A-phase),
    :1877 (L6-A2's line) — true, left; the changelog's L6-A2 and VIS1 entries — history, left.
  - `grep -rn -i 'no team auton|explicit form|AF1S-A2|reaches an auton'` over the same and the register: guide 14
    :464-466 and guide 20 :320-322 said "No team auton can call it yet ... (AF1S-A2)" — REWRITTEN; guide 20's banner
    ("in its form over explicit handles — an auton reaches the camera in a later part") — REWRITTEN; roadmap :530,
    :546-550, :1978 — the A1 notes, true of their date, left beside this run's new "Now" note and `[~]` line; the
    changelog's A1 entry ("No team auton can call it yet") — history, left (this run's entry above it says what changed).
  - `grep -n -i 'no camera'` over guides 11, 14, 20: guide 11 :618 listed NO CAMERA's old reasons — REWRITTEN with the
    slot's (no block, no camera, mount, tracker or read period, an untaught id, not built).
  - `grep -rn -i 'toggles ready|toggles OFF|toggleCamera|Toggle block|TogglePoseBound|ExplicitToggleCamera|lookAtToggle'`
    over the docs: every hit this run's text or A1's still-true text.
  - `grep -rn 'camera port 3'` over docs: guide 10's boot-line examples — a block-less robot's line is unchanged, so
    they stand; this run's examples added beside them.
  - Guide 21 (`grep -n -i 'flipper|camera'`): ":294 No camera ... the camera's port is written down but nothing reads
    it" — still true of robot one (its camera block is G1/ROB1-C's); left.
  - `simulator` in the new and changed lines: only the standard disclaimer in the two new test files.
  - Removability (`grep -rln 'internal/|chunks/|RESUMING|build-order'` over docs/*.md, docs/guide, docs/cookbook,
    README.md): empty.
  - Hardware claims in the new doc lines (`tested|measured|works on the robot|on a robot`): every hit is "NOT run on a
    robot", or "measured" as a REQUIREMENT on the mount ("its mount MEASURED"), never a claim.

**The version history paragraph (for `include/shulib/version.hpp` — the coordinator numbers it at the merge; NOT
written into version.hpp here, as the task said; MIRROR1-A is also additive and in flight):**

> N = chunk AF1S-A2 (built 2026-11-15), the skills Toggle step from an auton, all additive and no frozen row touched
> (RobotContext is outside F4, the scheduler's shadow context outside F6's accessors, competition/ and vision/ open by
> design — F15): the new header vision/toggle_camera.hpp (`ToggleCameraConfig`, `toggleCameraCheck` /
> `toggleCameraProblem`, `ToggleCamera`); `config::SensorsConfig::toggleCamera` (appended, with a default);
> `chassis::RobotContextConfig::toggleCamera` / `RobotContext::toggleCamera()` (appended; the motion scheduler's shadow
> context carries it); `robot::Robot::toggleCamera()` / `toggleCameraAbsence()`; `competition::checkAndFlipToggle(ctx,
> request, bound)` / `lookAtToggle(ctx, request, bound)`, `competition::TogglePoseBound`, and the appended
> `ExplicitToggleCamera::absence`. A robot file with no Toggle block builds the same robot, its boot line byte for byte;
> every 2.8 program compiles and runs exactly as before. Nothing to do. docs/changelog.md, API 2.N, lists it.

**For the coordinator (the merge):**
- **API:** additive; the changelog entry sits at the top of `## API 2.8` — renumber with MIRROR1-A's at the merge. The
  task called it "F6/F10-additive": A2 touches NEITHER F6 nor F10 (no `Chassis` or `Routine` member changed; the
  signature pins are untouched).
- **MIRROR1-A (in flight) — ASSUMED, not tested here:** the everyday form passes `ctx.entry` to the SAME
  `detail::ToggleVisit` as the explicit form, and MIRROR1-A's `realRequest` (in that class's constructor) swaps the Toggle
  for a mirrored entry — so a mirrored route's everyday visit reads the real Toggle with no code of A2's. The camera slot
  is the robot's own (nothing to mirror). Owed by whichever lands second: one everyday-form visit under a mirrored entry
  (robot two at the right wall for robot one's red wall). Shared files to merge by hand: `toggle_visit.hpp` (its ctor /
  `req_` vs this run's banner, `ExplicitToggleCamera`, rule 1's NO CAMERA line, the new forms at the end);
  `test/cookbook_examples_test.cpp` (MIRROR1-A appends cookbook-09a after 08a and adds includes; this run REPLACED 08a's
  body and added `#include "toggle_robot_rig.hpp"`); `mkdocs.yml` and `docs/cookbook/README.md` (each adds its chapter —
  keep both, 8 before 9); guides 10, 14, 15, 20, the changelog, the roadmap, REMAINING-WORK, build-order (each touched
  minimally, logged above).
- **RMP1-A1 (the route builder):** its Toggle steps can generate `competition::checkAndFlipToggle(ctx, request)`
  (optionally `, {.poseTolerance = .., .headingToleranceDeg = ..}`) returning `.outcome()` from a `then()`; the request
  through `competition::toggleVisit({...})`; `worstCaseSeconds(request)` for the budget.
- **ROB1-BT / ROB1-C / G1:** robot one's (and robot two's) camera block and Toggle block are theirs; `src/` is untouched
  here. A follow-up worth having there: the programs print the banner at `%.220s`, so a long `toggles OFF (...)` reason
  is cut on the brain's log — print `robot.toggleCameraAbsence()` on its own line when a block is declared.
- **tools/:** ONE new file, `tools/docs_figures/skills_toggle_route.py` (the docs figure's generator) — §7's
  `git diff --stat -- tools/` is not empty for that reason only.
- **HA:** no new belief (A2 adds no HA number); the register untouched; the next free number unchanged.
- **The between-parts marker** at the top of this file was removed when this run started — restore or complete it.
- **Reviewers / verifier:** this executor has no way to launch agents; the three reviewers and the fresh verifier are
  the coordinator's to run on this tree.
- SRC GATE INCIDENT (2026-11-15 ~21:00, said plainly): the first `src_build_gate.py self-test` ran under `timeout 3000`
  (SIGTERM) while three other worktrees' gates shared the CPUs; it was killed at 50 min (exit 124) MID-CASE, and its
  `finally` did not run — it left `src/robots/robot_one_facts.hpp` with every `owed(...)` turned into a "gate copy"
  `typed(...)` (the self-test's own in-repo case). The `check` that followed was stopped (SIGINT) before it finished,
  so its result is DISCARDED. The ledger was restored with `git checkout -- src/robots/robot_one_facts.hpp` (this branch
  changes nothing in `src/`); `git diff -- Makefile src/` empty, no `build/src-gate-facts/`, no
  `build/ROBOT-ONE-GATE-COPY-ONLY`. Both re-run with `timeout -s INT` (so the gate's `finally` runs on a timeout).
- FINAL GATES (the tree as left, 2026-11-15 ~22:30): test binary REMOVED, `docs/api/` regenerated, the briefing
  regenerated with the binary absent, full build (`cmake --build build/test`, the doc gates inside it: api_doc_tool
  self-test OK, coverage 6422 entities / 212 headers, examples 926 lines verbatim, removability clean, suite_run and
  briefing self-tests OK; 0 warnings), the briefing regenerated after the binary existed; suite `green — 3720 cases,
  3864014 assertions` (through `tools/suite_run.py`, 4 GiB / 900 s); `ctest -R compile_refusal` 289/289; ARM header gate
  rc 0 (256 headers in one TU); CI guards (pros/, sim/) empty; `src_build_gate.py self-test` OK (377 detector cases)
  and `check` PASS (all 14 builds; robot one's 6 refuse naming exactly their owed facts and link in the typed copy) —
  `git diff -- Makefile src/` empty afterwards, no `(self-test plant`, no `build/src-gate-facts/`; api_doc_tool
  self-test / check-coverage / check-fresh (rc 0) / check-examples / check-removability OK; doc_staleness_audit clean;
  briefing_status check current. The brief's §7 greps: the §1.6 files unchanged; `src/` and `Makefile` unchanged,
  `tools/` = the one new figure script (delta above); no AF1-A include; no competition/ include from vision/ or
  mechanism/; nothing of vision/ in the plant; no waitUntil and no blocking verb in the step; `seating not checked`
  present (6); `grep -n 'toggleCamera' motion_scheduler.hpp` = exactly the shadow context's line (:669);
  HA-231/232/234/235/243/244/245 each cited and each with its row (unchanged); no unlabelled PROVISIONAL; removability
  empty.
- STOPPED HERE (2026-11-15): AF1S-A2 built, NOT committed. For the coordinator: the merge notes above (MIRROR1-A's
  shared files and the mirrored-visit test owed, the version paragraph, the one tools/ file, the restored marker), and
  the three reviewers and a fresh verifier.

### AF1S-A2 review findings and their disposition

- **RESUMED 2026-11-16 02:10 EST** (`date` read) — the builder again, after the first fix run was stopped by the
  usage limit. What the stopped run left, checked before anything else: (1) the worktree had been MOVED onto main's
  tip `28a7497` (MIRROR1-A, API 2.9, and the Tune label fix) at 2026-11-15 22:51 (`git reflog`: "reset: moving to
  28a7497"); the pre-move tree is saved in `scratch/af1s-a2/rebase/` (`a2-tracked.patch`, `a2-fulltree-changed.tgz`,
  `a2-untracked.tgz`). Checked by a three-way merge of every changed file (the saved A2 copy, base `9befdb3`,
  `28a7497`): 18 files equal a clean merge exactly; 6 had conflicts and were resolved by hand, each read — the
  cookbook README, `mkdocs.yml` and the cookbook test's index (both chapters kept, 8 before 9), `test/README.md`
  (both rigs' paragraphs), guide 20 (MIRROR1-A's "In a mirrored auton" paragraph kept, before A2's Toggle-block
  subsection), REMAINING-WORK's 18e-S row (A2's wording, a superset of MIRROR1-A's untouched row); the 8 new files
  are byte-identical to the saved copies. The generated files (`docs/api/`, the briefing) were NOT carried — they are
  regenerated below. (2) No mutation left applied: every file `mut.py` backed up equals the pre-move copy (one,
  `toggle_camera.hpp`, differs from its backup only by a later, logged comment — X2's fix), and the current files
  equal the clean merge; the reviewers mutated their own copies (`review-*/tree`), never this worktree.
  (3) `git diff -- Makefile src/` empty, no `(self-test plant`, no `build/src-gate-facts/`, no
  `build/ROBOT-ONE-GATE-COPY-ONLY`. (4) The test binary REMOVED and relinked from source by a full build before any
  suite result below is trusted. No disposition had been logged by the stopped run; every finding is taken below.
- **Baseline on the moved tree** (before any fix, binary relinked from source): `suite run: green — 3772 cases,
  3884248 assertions` (main's 28a7497 brings MIRROR1-A's cases; A2's 19 included).

| # | Finding (lens, severity) | Disposition |
|---|---|---|
| 1 | correctness, MAJOR — the excursion vs the robot file's `frozenMoveIn` is a precondition in the everyday form: a two-file mismatch THROWS out of `then()`/the Routine/`runAuton` (no onFail, no line, the run lost) while the slot says READY | **REAL — fixed at the root, a DELTA from the brief's §1.1 ("a run-time PRECONDITION otherwise"), FLAGGED for the coordinator / team lead.** The reviewer's probe's scene became a test (with the old code its `REQUIRE_NOTHROW(w.run(...))` fails — mutant F1-THROW below). The everyday form now answers a NEW appended verdict `ToggleVisitVerdict::SpotsTooClose` ("SPOTS TOO CLOSE") at rule 1's same place (after the camera's own checks, before FLIPPER NOT READY): nothing read or moved, `outcome()` UNCONFIRMED so the route's `onFail` applies, ONE Warn line naming both numbers (`spots too close -- lookFrom - pushFrom 12.0 in, needs more than 2 x frozenMoveIn 6.0 in -- nothing pushed`). The EXPLICIT form keeps A1's precondition (one caller holds both numbers — a programming error; A1's pinned case unchanged): ONE body, a `detail::VisitForm` the everyday calls pass. NEW `competition::toggleVisitBlockProblem(request, block)` (constexpr) checks the pair at build — shown in cookbook chapter 8 as a `static_assert`. Tests: the EQUALITY edge (frozenMoveIn 6.0, excursion 16 − 4 = 12) through a real Robot and `runAuton` with `onFail(skipTo("next"))` — refused, the skip applied, reads 0, strokes 0, one Warn line, for the flip AND the look; one ulp under (`nextafter(6, 0)`) it flips; the constexpr pair at the edge (static_assert and run time), an undeclared / unset / NaN block passing to the block's own check, a request's own problem first. A1's verdict-word test now runs to the last verdict. Docs: worksheet T.6, guide 20, guide 11 (the new line), guide 10, the changelog, cookbook 8 (a watch-out + the static_assert). Why not a refusal in the explicit form too: it would silently change A1's pinned programming-error rule for hand-wired callers; the everyday form is where the two numbers come from two files. |
| 2 | correctness, minor — a pose bound under a RELATIVE start runs the field-frame position gate on a robot-frame estimate: every visit "UNKNOWN before (no band)" | **REAL — fixed in the everyday form.** `detail::poseBoundDropped(ctx, bound)`: a stated bound under a start that is not FIELD is not handed to the tracker; the line says `wall and position not checked (relative start)`. Only the everyday form: there the library KNOWS runAuton seated the estimate in the robot's own frame; the explicit form's caller knows its own frame (A1's rig runs a field-frame estimate under a RELATIVE entry, and its gate tests rely on it — changing the body would have broken A1's pinned gate cases for no real caller's benefit). Tests: the reviewer's scene (the block's gate 3/4/8, the docs' literal `{2_in, 3}`) under RELATIVE — FLIPPED / SEEN NOT OURS, `positionChecked` false, the line's words, ≤ 160 bytes; with no bound — the old words only; `toggleCameraOf` FIELD vs RELATIVE. "FIELD start" said beside `TogglePoseBound` (header), and in guides 20, 14, 11, 10 and cookbook 8. |
| 3, 10, 20 | correctness / tests / docs, minor — the cookbook test's "12 in up" comment; "the wait mattered" never asserted (R8, home-first, caught only by the 2 % timing check) | **REAL — fixed.** The comment says 3 in. The overlap is now MEASURED: the clock at the first leg's last drive-motion boundary (`scheduler().completedCount()` changing before the first `[TGL]` line — the right wall's visit drives nowhere) and when the cascade slot first reports `homed()`; `CHECK(homeEnd > legEnd)` (observed 2.31 s against 1.64 s). R8 re-run below. |
| 4, 12 | correctness minor / docs MAJOR — MIRROR1-A LANDED first (e9bcee7, API 2.9): the changelog's section, the history number, the owed mirrored everyday visit, cookbook 09's stale line, the hand merges | **REAL — done.** The worktree was moved onto `28a7497` (the resumption note above: the moved tree checked file by file). The changelog entry now opens a NEW `## API 2.10` section above 2.9 (the coordinator confirms the number at the merge); the history paragraph's N is 2.10 (below). The owed test: an everyday visit under `mirrored(entry, kSkillsMirror)` through a real Robot and `runAuton` — robot two truly at the right wall's look spot, the route naming robot one's red wall: FLIPPED the right wall's Toggle, the wall checked, its line naming the right wall; the unmirrored control seats the estimate at robot one's start and names the red wall. (The reviewer's probe expected WRONG WALL for that control — not so: runAuton seats the entry's start, so the estimate faces the red wall; my first cut asserted WRONG WALL, went red, and was corrected to what the run truly does.) Re-read `toggle_visit.hpp` at MIRROR1-A's version: the everyday form passes `ctx.entry` to the same constructor, whose `realRequest` swaps; the new SPOTS check reads `req_`'s sizes (unchanged by the swap) and the bound drop reads `effectiveStart()` (a mirrored FIELD start stays FIELD). Cookbook 09's ":101-102" line reworded (the everyday form is here; chapter 8 linked). NOT done, said plainly: MIRROR1-A's log expected "AF1S-A2 adds the visit line to page 09's route" — cookbook-09a's robots have no camera or Toggle block, so a visit there would be a NO CAMERA step teaching nothing; the mirrored everyday visit is pinned by the new ctx case instead. For the coordinator. |
| 5 | tests, MAJOR — the bound's heading half never shown to reach the tracker (R1 survives); R10 (a tracker number changed after the copy) survives | **REAL — fixed.** A unit case on `detail::toggleCameraOf` with a READY slot, an ASYMMETRIC bound `{2 in, 7 deg}` and a block with every tracker number set (the gate's three too): the feed, the model, `poseTolerance` 2, heading 7, and all 17 tracker fields *(WRONG: 16 — 3 ids and 13 doubles, and their values repeated; corrected in the verification round below, finding V3)* equal the slot's, field by field — and under RELATIVE, 0 and 0 (finding 2). R1 and R10 re-run below. |
| 6 | tests, MAJOR — the block's read period never shown to reach the feed (R5/R6 survive) | **REAL — fixed.** The READY case drives the FakeClock: from the first read's start `t0`, a tick at the last double before a full period reads nothing; a tick ON the edge (the first instant `now - t0 >= 0.033` in the feed's own arithmetic — found with `nextafter`, not assumed) reads (reads 1 → 2). R5 / R6 re-run below. |
| 7 | tests, minor — `lookAtToggle(ctx, request, bound)` never given a bound (R3b) | **REAL — fixed.** The pose-bound case is a table over both forms (with a bound `positionChecked`, without not, a loose one not). R3b re-run below. |
| 8 | tests, minor — "no block declared" before "no camera" unpinned (R4) | **REAL — fixed.** `toggleCameraCheck(CameraConfig{}, ToggleCameraConfig{})` names the block (and no part), and a `const Robot` with neither a camera nor a block says the block's sentence. R4 re-run below. |
| 9 | tests, minor — the const `Robot::toggleCamera() const` never called (R11) | **REAL — fixed.** `std::as_const(robot).toggleCamera()` is nullptr in every refusal (`checkRefusedButDrives`) and the slot when READY; the `const Robot` above too. R11 re-run below. |
| 10 | tests, minor — `mut.py`'s verdict column labels survivors CAUGHT | **REAL — fixed in the tool** (`scratch/af1s-a2/mut.py`'s new `judge()`, reused by `mut2.py`): CAUGHT only with the doctest summary's `failed > 0` AND a case red BY AN ASSERTION (an ERROR/FATAL line under its own header — analyze.py's rule), or a killed run (it says which); otherwise INSPECT or SURVIVED. Checked on the reviewer's outputs: `out_R1/R4/R5.txt` now read SURVIVED, `mut/out_M-RCP.txt` CAUGHT. The first run's verdicts were never taken from that column (this log judged by analyze.py; X10's first run was logged as SURVIVED) — no claim changes. **For the coordinator's lessons:** never judge a mutant by a bare `TEST CASE:` header — passing cases print it with MESSAGE output. |
| 11 | tests, minor — the colour-id order unpinned (R7) | **REAL — fixed.** A subcase with RED and BLUE both untaught: the RED sentence. R7 re-run below. |
| 13 | docs, MAJOR — the recipe's FLIPPER STUCK recovery turns in place at the push spot (landmine 6) | **REAL — fixed, and the recipe's claim now tested.** Each leg after a visit stows, then `reverseTo(<a point BEHIND the robot on its line, a few inches past the look spot>)` — (40, 0) and (0, 40); the end's `reverseTo(0, -30)` already was one — THEN turns. Not the reviewer's `reverseTo(<this look spot>)`: after a good visit the robot is AT the look spot, and `reverseTo` aims at the bearing + π, so a target a fraction of an inch AHEAD would turn the robot round at the wall; a point clearly behind is a straight back-off from the look spot and from the push spot alike. NEW case `cookbook-08a (FLIPPER STUCK)`: the same route, the right wall's Toggle yellow, the flipper wedged on its first retract (freed once the step gives up): FLIPPER STUCK at the push spot (x > 56), the skip, the flipper stowed (< 5°) before the robot moved, the robot more than 10 in back before it turned more than 3°, the blue wall's Toggle still flipped. The run is now about 31.1 s (two back-offs; it was 29.1) — page and test. The figure shows the back-offs; a "Why" bullet says why. F13-NOBACK below. |
| 14 | docs, minor — build-order's AF1 entry not updated | **REAL — fixed:** a "Status 2026-11-16: AF1S-A2 BUILT" line; the old A1 status and VIS1's "Next: AF1S-A1 … then AF1S-A2" marked as history (the first bold `Next:` — the briefing's pointer — is the Next block, which this does not move). |
| 15 | docs, minor — the docs promise a boot line with `toggles OFF (why)` that no brain program shows whole | **REAL — fixed in the docs:** worksheet T.2/T.3, cookbook 8, guide 20, guide 11, the glossary, the changelog and the roadmap now send the team to the `[TGL]` NO CAMERA line and `robot.toggleCameraAbsence()`, and say the programs print the boot line cut to 220 characters and the competition program prints none. `src/` stays untouched (the brief's §7). **Proposed, not queued by me (rule 2: plan changes are proposed):** ROB1-C / VIS1-B print `robot.toggleCameraAbsence()` on its own line when a block is declared. |
| 16 | docs, minor — "the camera block you already have" (neither robot has one); Station T written for one robot | **REAL — fixed:** cookbook 8 ("the robot file's camera block … neither of the team's robot files has one yet … each robot … its own pair of blocks"), the changelog's "What you must do", guide 14 ("the camera block and the Toggle block"); Station T's heading says once per robot (both run skills — the answers block) and its table has a column per robot. |
| 17 | docs, minor — not every recipe number marked with its Step 0; "Step 0.x" resolves only in internal docs | **REAL — fixed:** every line of the Toggle block (Step 0.3), `lookWithin` (Step 0.3 — the request's own doc says so), `margin` ("your spare time … a choice, not measured") marked in the test and quoted verbatim; guide 20's paragraph is now a section, "Step 0: the numbers the team measures", items 0.1–0.7 numbered as the brief's §2.0; cookbook 8 links it. |
| 18 | docs, minor — the position gate presented as turning lookalikes away | **REAL — fixed** in the three places (cookbook 8, guide 20, `TogglePoseBound`'s doc): "MOST lookalikes — not one within its tolerance of where a Toggle's face is (Chapter 14)". |
| 19 | docs, minor — two places still say the wall sensors are filled "last" | **REAL — fixed:** robot.hpp's `walls_` doc and its banner, and guide 10 — "after the camera, the last provider requests; the Toggle camera, filled after them, asks for nothing". |
| 21 | docs, minor — the changelog's "the entry below"; the Added list incomplete | **REAL — fixed:** the A1 entry named ("API 2.8's entry of 2026-11-13, 'the Toggle flip for a SKILLS run, as one route step'" — no anchor: its heading's slug is fragile); `vision::toggleIdsProblem`, `vision::ToggleCameraCheck`, `vision::kToggleCameraReasonBytes`, and this round's `SpotsTooClose` and `toggleVisitBlockProblem`, listed. |
| 22 | docs, minor — front-door links | **REAL — fixed:** `toggle_camera` in guide 20's footer; docs/README's Toggle row links the recipe. |
| 23 | docs, minor — HA citations | **REAL — fixed:** `ToggleCameraConfig::feed`'s doc cites HA-192; T.3 cites HA-245, T.5 HA-244; the register's AF1S rows (HA-231/232/235/243/244/245 — this chunk's own) name "worksheet Station T, T.n" in their Settle cells and *Settle:* lines (the coordinator may reword at the merge); HA-245's *Source:* said "guide chapter 19" — the camera chapter is 20 — fixed. No new HA number. |
| 24 | docs, minor — the figure and caption state the Quadrant colours as fact | **REAL — fixed:** the figure's footer (regenerated by its script) and the page's alt text now say "a reading of the manual's drawings (HA-243), checked on the event's field at Step 0.7"; so does the "'Ours' is never typed" bullet. |

- **RESUMED AGAIN 2026-11-16 08:53 EST** (`date` read) — the builder, a third run of this fix round: the run that
  resumed at 02:10 stopped at about 02:57, after its fixes, the table above and its mutation run (`scratch/af1s-a2/
  mut2/`), before logging that run, its prose review and the final gates. Checked before anything else: (1) every
  file the second driver (`mut2.py`) backed up is byte-identical to the worktree's copy (`cmp`: cookbook test,
  `motion_scheduler.hpp`, `robot_config.hpp`, `robot.hpp`, `toggle_camera.hpp`, `toggle_visit.hpp`) — no mutation
  left applied; (2) `git diff -- Makefile src/` has no `(self-test plant`, no `build/src-gate-facts/`, no
  `build/ROBOT-ONE-GATE-COPY-ONLY`; (3) the test binary on disk is the LAST MUTANT's link (`M-A2-3`, 02:56) — removed
  and relinked from source before any suite result below; (4) `docs/api/` was generated at 2026-11-15 22:51, before
  this round's `///` changes — regenerated below. The table's dispositions were re-read against the tree (each fix
  present in the code, the tests and the docs it names). The second driver's verdicts are NOT taken as they stand:
  `test/robot_toggle_camera_test.cpp` was edited at 02:43, after R5 and R6 ran, and two mutants (F1-PAIR, F12-MIR)
  stopped their own builds — so every mutant is re-run below, on the final tree, on fresh builds.

- **The baseline of this run** (the binary relinked FROM SOURCE — the mutant link removed; `docs/api/` regenerated,
  the briefing regenerated with the binary absent, configure, the full build with its doc gates — api_doc_tool
  self-test OK, coverage 6473 entities / 215 headers, 988 quoted lines verbatim, removability clean, staleness audit
  clean, 0 warnings — then the briefing regenerated with the binary present): `suite run: green — 3778 cases,
  3884470 assertions` (the moved tree's 3772 + this round's 6 new cases).
- **Main moved again** while this round ran: `shulib-v2` is now `3c29b73` (ROB1-C2, 2026-11-16 01:19 — robot one's
  real mechanisms; NO version bump: its entry sits under `## API 2.9`). This tree stays on `28a7497` (the round's
  evidence is on one tree; the coordinator squashes onto the tip). Checked for the merge: `git diff 28a7497` (less the
  generated `docs/api/` and briefing) applied to an archive of `3c29b73` (`scratch/af1s-a2/rebase2/`) — EVERY hunk
  applies (`patch` rc 0; fuzz 1 on the changelog, fuzz 2 on the worksheet, offsets elsewhere), and each fuzzed hunk
  landed where it belongs: `## API 2.10` above ROB1-C2's 2.9 entry; Station T after Station R1's table. The files both
  change: the changelog, `docs/hardware-assumptions.md`, the R3a worksheet, REMAINING-WORK, the roadmap (and the
  generated `docs/api/README.md`, `all-entities.md`, the briefing — regenerate). ROB1-C2's `src/` sets
  `RobotConfig::sensors` only by designated initializers (`grep` of the tip's `src/`), so the appended
  `SensorsConfig::toggleCamera` breaks none — but `src/` changed a great deal there: the src build gate must run again
  on the merged tree. ROB1-C2 keeps robot one's camera (E18) owed, `Stage::Recorded`, read by no program, and records
  its flipper as a PNEUMATIC (D16: the front toggle, B) — so this run's "neither of the team's robot files has a
  camera block yet" still holds at the tip.
- **PROSE REVIEW of this round** (rule 4 — each grep RE-RUN by this run and its hits READ before this line was
  written; the 02:10 run had run them too, its notes in `scratch/af1s-a2/coordinator_block.md`, never logged):
  - `grep -rn -i "twice the tracker's\|or the step refuses"` over docs/guide, docs/cookbook, docs/README.md,
    test/README.md and the R3a worksheet: no hit (guide 20 and T.6 now say the step REFUSES, SPOTS TOO CLOSE).
  - `grep -rn -i "toggles OFF"` over the same plus the changelog and the roadmap: cookbook 8 :54, the worksheet :2766,
    the changelog :64, guide 20 :443 (its paragraph says at :447-448 where the line is printed), guide 11 :656 — each
    says where the boot line is printed and cut (220 characters; none in the competition program); guide 10 :988-989
    are `describe()`'s literal output, the paragraph above them (:983-985) saying the same. True; left.
  - `grep -rn -i "turns away\|turned away\|turns lookalikes\|lookalike"` over guide 20, cookbook 8, guides 10/11 and
    the two headers: this round's three places say MOST lookalikes (cookbook 8 :202, guide 20 :460,
    `toggle_visit.hpp` :706); guide 20 :143/:150/:205-206 are VIS1-A2's own (each gate's job, and its limits),
    `toggle_visit.hpp` :158 A1's cannot-catch list — true, left *(WRONG: it said the position gate "is what turns those away" — A1's own delta 12 pins the opposite; reworded in the verification round below, finding V2)*. No hit in this run's changelog entry.
  - `grep -rn -i "already have\|camera block you"` over docs/guide, docs/cookbook, docs/README.md, the changelog: only
    unrelated hits (guide 3 :30, guide 10 :136, the cookbook README :8/:139) and "the mechanism steps they already
    have" (cookbook 8 :207, guide 20 :469) — true.
  - `grep -rn "last, after the camera\|, last (build\|LAST, after the camera"` over docs/guide and robot.hpp: none.
  - `grep -n "29 s\|29\.1"` over cookbook 8 and its test: none (31 s / 31.1).
  - MIRROR1-A's own lines about the Toggle step (`git diff 9befdb3 28a7497 -U0 -- docs include test`, added lines
    matching "toggle step|toggle visit|AF1S|checkAndFlip|everyday form"): cookbook 09's "(A Toggle visit … arrives
    with the everyday form …)" — REWORDED by the 02:10 run; MIRROR1-PROGRESS's item 19 ("AF1S-A2 adds the visit line to
    page 09's route") — answered in the table (findings 4/12: not done, why); the rest (field_mirror.hpp's and
    toggle_visit.hpp's comments, the MIRROR1 brief, guide 11/17's mirrored-name lines) true, left.
  - `simulat` over this tree's added lines and the new files: only the two test files' "never 'simulator'" notes.
  - Removability (`grep -rln 'internal/\|chunks/\|RESUMING\|build-order'` over docs/*.md less the roadmap and the
    register, docs/guide, docs/cookbook, README.md): empty.
  - Hardware claims in the added public lines (`tested|measured|works on the robot|on a robot|on the robot`): every
    hit is "NOT run on a robot" or "measured" as a REQUIREMENT on a mount or a number, never a claim.
  - The figure (`docs/assets/skills-toggle-look-spots.png`, regenerated 02:30 with its script) re-read: the two
    hollow back-off dots, the footer's HA-243 caveat.
- **Public docs touched this round:** cookbook 8, cookbook 9 (one line), guides 10, 11, 14, 15, 20, docs/README (one
  link), the changelog; internal: the worksheet's Station T, the register's AF1S rows, the roadmap, build-order,
  REMAINING-WORK. Over rule 7's ~5 for the build round's reason: each fixes a sentence a finding showed false or
  missing.
- **MUTATIONS of this round — ALL re-run on the FINAL tree by this run** (`scratch/af1s-a2/mut3.py`, mut2.py's
  mutants and rules plus the build round's X1–X15 — X3 and X12 ported to the moved code, the same mutation — and one
  new, F1-OK; results `mut3/mutations.log`, `mut3/run.out`, per-mutant `mut3/out_<id>.txt`; 09:00–09:53). Each: the
  file backed up in scratch, ONE exact replacement (matched once — checked for all 37 before the run), mtime bumped,
  CMake's `depend` step THEN the build, every object whose depfile names the file AND the binary checked newer than
  the mutation (none stale), the WHOLE suite under 4 GiB / 600 s, judged by the FIXED `judge()` (finding 10: the
  summary's failed > 0 AND a case red by an ERROR/FATAL line under its own header), restored in a finally and on
  SIGTERM/SIGHUP/SIGINT, mtime bumped, compared byte for byte (37 of 37 identical; afterwards each of the six files
  `cmp`-equal to its last backup). **37 mutants, 37 CAUGHT — 36 by a run-time assertion, 1 AT BUILD by the test's
  own `static_assert`; none killed, none broke its own build:**
  - Finding 1: **F1-THROW** (the everyday form throws again) — the SPOTS case's `REQUIRE_NOTHROW` (FATAL, :427).
    **F1-EDGE** (`>=` for `>` in rule 1: the equality edge accepted) — the SPOTS case (:429, the flip and the look).
    **F1-PAIR** (`>=` for `>` in `toggleVisitBlockProblem`) — CAUGHT AT BUILD: the ONLY error in its build output
    (`mut3/build_F1-PAIR.txt`) is `toggle_visit_ctx_test.cpp:499:77: error: static assertion failed` — the test's
    own equality-edge `static_assert` (the cookbook's, at frozenMoveIn 2.0, rightly still passes). (The 02:10 run's
    driver had labelled the same result "BUILD-FAIL, not counted"; its cut was right, its verdict rule too blunt.)
    **F1-OK** (NEW: SPOTS TOO CLOSE answered SUCCEEDED) — the SPOTS case (the outcome, `failedOn`).
  - Finding 2: **F2-KEPT** (the bound kept under RELATIVE — the review's probe) — the RELATIVE case (:279) and the
    `toggleCameraOf` case (:546); **F2-LINE** (the drop unsaid) — the RELATIVE case's line (:285).
  - Findings 5/7: **R1** (the heading half dropped) and **R10** (a tracker number changed after the copy) — the
    `toggleCameraOf` case (:548, :556); **R3b** (`lookAtToggle(ctx)` drops the bound) — the pose-bound table (:249).
  - Finding 6: **R5** (period 0.001 s) — the READY case's tick just UNDER the period (:291); **R6** (0.05 s) — the
    tick ON the edge (:294). Re-run here because `robot_toggle_camera_test.cpp` was edited (02:43) after the 02:10
    run's R5/R6.
  - Finding 8: **R4** (the camera's check first) — the no-block case (:318). Finding 9: **R11** (the const door hands
    out a slot not ready) — the no-block case's `const Robot` (:326) and the refusals. Finding 11: **R7** (BLUE before
    RED) — the both-untaught subcase.
  - Findings 3/10/13: **M-RCP** (the wait dropped) — cookbook-08a and its FLIPPER STUCK twin (each route stops at its
    `cascade` step); **R8** (home FIRST, no overlap) — cookbook-08a's `CHECK(homeEnd > legEnd)` (:1149),
    no longer only the 2 % run-time check; **F13-NOBACK** (the first back-off dropped) — both cookbook-08a cases.
  - Findings 4/12: **F12-MIR**, RE-CUT (the 02:10 run's cut deleted the swap's line and left `entry` unused —
    `-Werror` stopped its own build, not counted; this cut puts `(void)entry;` in its place) — the A2 ctx "MIRRORED
    entry" case (verdict WRONG WALL, toggle `kRedWall`, 0 strokes — exactly what its comment says) and MIRROR1-A's T7.
  - The brief's own: **M-A2-1** (15 cases red), **M-A2-2a** (9), **M-A2-2b** (8), **M-A2-3** (3).
  - The build round's X1–X15 on the final tree — each CAUGHT by the case the build round named (X3 and X15 by the
    pose-bound table, now over both forms, and by the `toggleCameraOf` case; X6 by 15 cases; X13 by 12,
    cookbook-08a among them).

**The version history paragraph — REVISED 2026-11-16 (supersedes the one above: N is now 2.10, MIRROR1-A having
landed as 2.9 and ROB1-C2 having taken no number; the coordinator confirms it at the merge; NOT written into
version.hpp here):**

> 10 = chunk AF1S-A2 (built 2026-11-15, reviewed and fixed 2026-11-16), the skills Toggle step from an auton, all
> additive and no frozen row touched (RobotContext is outside F4, the scheduler's shadow context outside F6's
> accessors, competition/ and vision/ open by design — F15): the new header vision/toggle_camera.hpp
> (`ToggleCameraConfig`, `ToggleCameraCheck`, `toggleIdsProblem`, `toggleCameraCheck` / `toggleCameraProblem`,
> `ToggleCamera`, `kToggleCameraReasonBytes`); `config::SensorsConfig::toggleCamera` (appended, with a default);
> `chassis::RobotContextConfig::toggleCamera` / `RobotContext::toggleCamera()` (appended; the motion scheduler's
> shadow context carries it); `robot::Robot::toggleCamera()` / `toggleCameraAbsence()`;
> `competition::checkAndFlipToggle(ctx, request, bound)` / `lookAtToggle(ctx, request, bound)`,
> `competition::TogglePoseBound`, `competition::toggleVisitBlockProblem`, the appended
> `ExplicitToggleCamera::absence` and the appended verdict `ToggleVisitVerdict::SpotsTooClose` (the everyday form's
> only). A robot file with no Toggle block builds the same robot, its boot line byte for byte; every 2.9 program
> compiles and runs exactly as before. Nothing to do. docs/changelog.md, API 2.10, lists it.

**For the coordinator (the merge) — REVISED 2026-11-16; supersedes the build round's "API" and "MIRROR1-A (in
flight)" bullets, the rest stand:**
- **The base:** this tree is on `28a7497` (MIRROR1-A, API 2.9, and the Tune label fix); main's tip is now `3c29b73`
  (ROB1-C2) — the patch applies cleanly onto it (above). If main moves again (ROB1-BT, RMP1-A1 …), re-check the same
  shared files; run the src build gate on the merged tree (ROB1-C2 changed `src/` widely).
- **API:** additive — `## API 2.10` opens the changelog (its own section, above 2.9); the history paragraph above.
- **A delta for the team lead to see (finding 1):** the brief's §1.1 made the excursion check a run-time
  PRECONDITION; in the EVERYDAY form it is now a refusal verdict, SPOTS TOO CLOSE (two numbers from two files must
  not throw a skills run away); the explicit form keeps the precondition. If the ruling is "keep the precondition",
  revert `detail::VisitForm::everyday`'s use in rule 1 and the verdict, and the ctx case and docs with them.
- **MIRROR1-A — DONE here, no longer assumed:** the everyday-form visit under a mirrored entry is a test
  (`toggle_visit_ctx_test.cpp`, "a MIRRORED entry"), and the step's mirror swap dropped (F12-MIR, re-cut below) turns it
  red. Cookbook 09's line reworded. Not done: a Toggle visit in cookbook-09a's listing (its robots have no camera
  block — MIRROR1-A's log expected it of A2; the table, findings 4/12).
- **RMP1-A1 (the route builder):** the everyday call may also take `, {.poseTolerance = .., .headingToleranceDeg
  = ..}` — meaningful only under a FIELD start (the step drops it otherwise); a generated route can also emit
  `static_assert(competition::toggleVisitBlockProblem(request, block) == nullptr)` when it can see the robot file's
  block; the new verdict word is "SPOTS TOO CLOSE".
- **ROB1-C / VIS1-B (proposed, not queued — finding 15):** print `robot.toggleCameraAbsence()` on its own line when
  a Toggle block is declared (the programs print the boot line at `%.220s`; the competition program prints none).
- **The HA register:** AF1S's own rows (HA-231/232/235/243/244/245) now point their Settle at "worksheet Station T,
  T.n", and HA-245's source names guide chapter 20 (it said 19). No new number.
- **tools/:** still ONE new file, `tools/docs_figures/skills_toggle_route.py` (regenerated: the back-offs and the
  HA-243 caveat). The mutation drivers stay in the scratch directory (`mut.py` with its fixed `judge()`, `mut2.py`,
  `mut3.py`).
- **The coordinator's lessons (finding 10, for the list):** never judge a mutant by a bare `TEST CASE:` header —
  passing cases print it with MESSAGE output; judge by the summary's failed count AND an ERROR/FATAL line under a
  case's own header.

- **FINAL GATES of this round** (the tree as left; `scratch/af1s-a2/final_gates3.sh`, logs `gates3/`; 09:53–11:01,
  `date` read): no `(self-test plant` in `git diff -- Makefile src/`, no `build/src-gate-facts/`, no
  `build/ROBOT-ONE-GATE-COPY-ONLY`; the test binary REMOVED (the last mutant's link), `docs/api/` regenerated, the
  briefing regenerated with the binary absent, configure, the full build (the doc gates inside: api_doc_tool
  self-test OK, coverage 6473 entities / 215 headers, 988 quoted lines verbatim, removability clean, suite_run and
  briefing self-tests OK, staleness audit clean; 0 warnings, 0 errors), the briefing regenerated with the binary
  present; suite `green — 3778 cases, 3884470 assertions` (through `tools/suite_run.py`, 4 GiB / 900 s — the same
  count as this run's from-source baseline); `ctest -R compile_refusal` 289/289; ARM header gate rc 0 (259 headers in
  one TU, the CI's flags); CI guards (pros/ outside hal/pros, sim/ outside sim) empty; api_doc_tool self-test /
  check-coverage / check-fresh / check-examples / check-removability rc 0; doc_staleness_audit clean;
  briefing_status check current; `src_build_gate.py self-test` OK (377 detector cases, each against a real build;
  `timeout -s INT`) and `check` PASS (all 14 builds; robot one's 6 refuse naming exactly their owed facts and link in
  the typed copy); afterwards `git diff -- Makefile src/` EMPTY, no plant, no gate copy. The brief's §7 greps on the
  final tree: `src/` and `Makefile` unchanged; `tools/` = the one new figure script; `grep -n 'toggleCamera'
  motion_scheduler.hpp` = exactly the shadow context's line (:669); no competition/ include from vision/ or
  mechanism/; nothing of vision/ in sim/; no `waitUntil` or blocking mechanism verb in the step; `seating not
  checked` present (6).
- After the gates: the src gate's self-test and check bump `src/` files' mtimes (contents unchanged — `git diff --
  src/` empty), so the staleness audit then called the binary older than `src/team/autons.cpp` and 11 more and did
  not trust it. Rebuilt (`cmake --build build/test`, 315 objects, 0 warnings — `gates3/16_rebuild_after_srcgate.log`):
  suite `green — 3778 cases, 3884470 assertions` again, briefing_status check current, doc_staleness_audit clean with
  the binary trusted. The binary left on disk is this tree's own, built from source.
- **STOPPED HERE (2026-11-16 ~11:05): this fix round is COMPLETE** — every finding dispositioned (the table), every
  fix's mutation observed red on a fresh build of the final tree, every gate green on it. NOT committed. For the
  coordinator: the merge notes above (main's tip is now `3c29b73` — the patch applies; the src gate again on the merged
  tree), finding 1's delta for the team lead, a fresh verifier, and the between-parts marker to restore or complete.

### Verification-round findings and their disposition (2026-11-16, the builder again)

- **STARTED 2026-11-16 12:47 EST** (`date` read) — the builder, on a fresh verifier's four findings (all minor).
  Checked before anything else: `git diff -- Makefile src/` has no `(self-test plant`, no `build/src-gate-facts/`, no
  `build/ROBOT-ONE-GATE-COPY-ONLY`; the verifier mutated its OWN copy (`scratch/af1s-a2/verify/v2/tree/`) — this
  worktree's `toggle_visit.hpp`, `toggle_camera.hpp`, `robot.hpp` and the cookbook test are `cmp`-equal to that copy's
  restored files; the binary on disk is the 11:05 rebuild from source; the tree is still on `28a7497`.

| # | Finding (severity) | Disposition |
|---|---|---|
| V1 | minor — SPOTS TOO CLOSE's place in rule 1 tested on neither side (V-ORDER, V-ORDER2 survive); the banner's ordered rule list leaves it out | **REAL — fixed (tests and banner; the code's order is unchanged — it is A1's place for the precondition, kept by A2).** Re-checked first: the verifier's two survivors, and two more of mine cut for the EXPLICIT form, run on the tree BEFORE the fix — **V-ORDER2X** (the whole excursion check, precondition included, moved above NOT SKILLS: an explicit MATCH call would throw) and **V-ORDER3** (the check moved above the tracker's own check: an explicit call with a refused tracker would throw instead of NO CAMERA) — both SURVIVED the whole suite (`mut4/before.log`: 3778 passed, 0 failed, 7 dependents recompiled each). Banner: rule 1 now lists the excursion check in its place — after the camera's checks, before the flipper's — "else, in the EVERYDAY form, SPOTS TOO CLOSE; in the explicit form a precondition at this same place". NEW case `AF1S-A2 ctx: the excursion check's PLACE in rule 1` (`toggle_visit_ctx_test.cpp`), through a real Robot and `runAuton`, a new auton `flipOutThenVisit` (the flipper sent OUT to "flip" by the route first when the scene asks; then ONE visit, everyday or explicit over the slot's own handles, the explicit precondition caught and kept): each refusal ALONE first on the same scene (the flipper out with the spots fine: FLIPPER NOT READY "is not at its back position" in both forms, the flipper > 80°, reads 0 — so the flipper truly is away; the spots alone: SPOTS TOO CLOSE / the precondition; a MATCH alone: NOT SKILLS); then a MATCH auton at the EQUALITY edge (frozenMoveIn 6.0, excursion 12) — NOT SKILLS in both forms, no throw, reads 0, the one line `not a SKILLS auton (MATCH)`; the spots too close AND the flipper out — everyday SPOTS TOO CLOSE (the flipper still out, reads 0, strokes 0, the one line the exact spots line), explicit the precondition naming the excursion, never FLIPPER NOT READY, no line; the explicit form with a tracker the camera's check refuses (aliveSeconds 0) AND the spots too close — NO CAMERA naming the tracker, no throw. The everyday form's "after the camera's checks" side cannot be cut there: a slot that is not ready hands the body no tracker numbers (frozenMoveIn 0), so NO CAMERA answers first by construction — said in the case's comment. Changelog's SpotsTooClose bullet: "checked after NOT SKILLS and NO CAMERA and before FLIPPER NOT READY". Guide 11's line list was already in this order; guide 20 states no order — left. After the fix: V-ORDER (red on BOTH sides — the everyday :595/:601 and the explicit :605–609), V-ORDER2 (:585, :589), V-ORDER2X (the everyday :585 and the explicit :584), V-ORDER3 (:616–618) all CAUGHT by assertion. |
| V2 | minor — a fourth place in `toggle_visit.hpp` says the position gate "is what turns those [lookalikes] away"; the fix round's prose review called it true | **REAL — fixed.** The "What it cannot catch" banner now reads "(A4 register HA-199 — the position gate, with a pose the team trusts, turns MOST of those away; not one within its tolerance of where a Toggle's face is)" (the sentence reflowed onto three lines). The fix round's prose-review line in this log is marked WRONG in place, pointing here. Re-grepped the whole tree's public docs and headers (`is what turns those\|turns those away\|what turns those`): the only other hits are `toggle_tracker.hpp` :94 and guide 14 :431 — both name WHICH lookalikes ("those" = the 540 bands and the wall-top strip, 0 of 540) and state the limit in the next sentence (the strip ON a face line) — true, left. |
| V3 | minor — the `toggleCameraOf` case claims "EVERY tracker number set, each distinct" and "17 numbers": 16, and the values repeat (2.0 three times, 3.0 twice) | **REAL — fixed.** Re-checked first: two cross-wire mutants on EQUAL-valued fields, run BEFORE the fix — **XW1** (frozenTurnDeg taken from frozenMoveIn after the copy) and **XW2** (minRangeIn from frozenMoveIn) — both SURVIVED the whole suite (`mut4/before.log`). The case now sets all 13 doubles to pairwise-distinct values the tracker accepts (minAspect 3.25, minRangeIn 1.5, frozenMoveIn 2.25, frozenTurnDeg 2.5, positionToleranceIn 3.5; the rest as before; the ids stay 1, 2, 3 — the camera's taught colours, which the slot checks) and CHECKS its own premise: a `static_assert` that `ToggleTrackerConfig` is exactly 3 ids + 13 doubles (a field added later stops the build, naming this case) *(not a member small enough to hide in the ids' padding — `sizeof` cannot see one; the check now COUNTS members, the resumption below)*, and the 16 numbers pairwise distinct (120 checks). It compares all 16 field by field, the handles against the slot AND (new) the slot against the robot file (it compared two). Comments say 16. The log's row 5 "all 17 tracker fields" is marked WRONG in place. (`robot_toggle_camera_test.cpp`'s "the block's 17 numbers" is right: the BLOCK is the 16 tracker numbers plus the feed's period.) After the fix: XW1 (`a.frozenTurnDeg == b.frozenTurnDeg`) and XW2 (`a.minRangeIn == b.minRangeIn`) CAUGHT; R1, R10, X15 and F2-KEPT — the earlier mutants this rewritten case catches — re-run and still CAUGHT (R1 :758, R10 :730, X15 by the pose-bound case :256 and this one, F2-KEPT by the RELATIVE case :286). |
| V4 | minor — the figure uses a hollow dot for the push spot AND for each back-off point (and the end point) | **REAL — fixed** in `tools/docs_figures/skills_toggle_route.py`, the figure regenerated with it and re-read: every point the robot backs straight off to — (40, 0), (0, 40) and the end's (0, -30) — is now a grey **x** on a white halo, never a dot; the purple ring is the ONLY hollow marker (the push spot). Label 1 says "purple ring: push spot, 4 in;" and "grey x: back off to (40, 0)"; label 2 "grey x: back off to (0, 40)"; the end's label "grey x: back off to (0, -30)"; the footer, now five short lines (the longest 656 px of 760), says "Purple ring: the push spot. Grey x: where the robot backs straight off to before it turns." The script's docstring says why. The page's alt text names no marker shapes — left. |

- **MUTATIONS of this round** (`scratch/af1s-a2/mut4.py` — mut3.py's machinery and its fixed `judge()` unchanged: the
  file backed up in scratch, ONE exact replacement matched once (checked for all 10 before each run), mtime bumped,
  CMake's `depend` step THEN the build, every object whose depfile names the file (7) AND the binary checked newer than
  the mutation, the WHOLE suite under 4 GiB / 600 s, restored in a finally and on SIGTERM/SIGHUP/SIGINT, mtime bumped,
  compared byte for byte — identical every time; afterwards `toggle_visit.hpp` `cmp`-equal to its backup, no
  `MUTANT` left). **Before the fix** (`mut4/before.log`, `before_out_*.txt`; 12:51–12:55): V-ORDER2X, V-ORDER3, XW1,
  XW2 — all SURVIVED (3778 passed, 0 failed). (V-ORDER and V-ORDER2 survived the verifier's own run.) **After the
  fix** (`mut4/mutations.log`, `out_*.txt`; 12:57–13:08, on the binary that holds the new cases): **10 mutants, 10
  CAUGHT by run-time assertion, none killed, none broke its own build** — V-ORDER, V-ORDER2 (the verifier's exact
  cut), V-ORDER2X, V-ORDER3, XW1, XW2, R1, R10, X15, F2-KEPT (lines in the table). One edit came AFTER the run:
  `toggle_visit.hpp`'s V2 sentence reflowed (a comment, the same words); the final gates below rebuild everything
  from source.
- **PROSE REVIEW of this round** (rule 4 — each grep RUN by this run and its hits READ before this line was written):
  - `grep -rn -i "is what turns those\|turns those away\|what turns those"` over include/, docs/guide, docs/cookbook,
    the changelog, docs/README.md and the worksheet (docs/api excluded — generated): `toggle_tracker.hpp` :94 and
    guide 14 :431 only — qualified (which lookalikes, and the limit in the next sentence); true, left.
  - `grep -rn "17 numbers\|17 tracker\|each distinct"` over test/ and this log: `robot_toggle_camera_test.cpp` :17
    and :486 (the BLOCK's 17 — true); `toggle_visit_ctx_test.cpp` :685 (this round's own history note); the log's
    row 5 (marked WRONG); `calibrate_program_rig.hpp` :290 (unrelated).
  - `grep -n -i "hollow"` over the figure script, cookbook 8 and guide 20: only the script — the push spot's ring
    and the comments saying the x is never a dot.
  - The new banner bullet and rule 1's new words re-read against the code (`allowed()`): NOT SKILLS, the three NO
    CAMERA checks, the excursion check, FLIPPER NOT READY, WRONG WALL — in that order; "this same place" is the
    explicit form's precondition, in the same `if`.
  - The changelog's new sentence re-read against the new case: a MATCH auton with the spots too close says NOT
    SKILLS (pinned).
  - `simulat` over this round's added lines and its two changed files: only the ctx test's standing "never
    'simulator'" note.
  - No removability hit can come from this round: the only public-doc change is one changelog sentence (no internal
    path).
- **Public docs touched this round:** the changelog (one sentence) and the figure (`docs/assets/skills-toggle-look-
  spots.png`, regenerated by its script). Internal: this log. Headers: `toggle_visit.hpp` (banner comments only — no
  code); tests: `toggle_visit_ctx_test.cpp` (one new case, one rewritten); tools: the figure script.
- **RESUMED 2026-11-16 22:50 EST** (`date` read) — the builder again, on the same four findings. The 12:47 round had
  stopped inside its final gates (`scratch/af1s-a2/gates4/`, 13:08): steps 00–12 green on the final tree (suite
  `green — 3779 cases`, ctest compile refusals 289 of 289, the ARM gate over 259 headers, the CI greps empty, the five
  api_doc_tool checks, staleness clean, the briefing current) — then cut off INSIDE `src_build_gate.py self-test`
  (~13:53, the session's pause); `src_build_gate.py check` never ran, and none of this was logged.
  - **FOUND: a src-gate self-test PLANT left behind** — the gate's case 36, `void ser1SelfTestPrintf();` and
    `void ser1SelfTestPrintf() { std::printf("x\n"); }` appended to `src/programs/bench_r3a.cpp` (13:53:31): the
    process was KILLED, not interrupted, so its `with _plant` never restored. It carries no `(self-test plant`
    marker, so the check this run's task names — `git diff -- Makefile src/` grepped for `(self-test plant` — reads
    0 WITH it present; only `git diff --stat` shows it. Removed at 22:51: the exact two-line suffix, asserted to be
    the file's end and its only `ser1SelfTestPrintf`, then `git diff --quiet` on the file (= HEAD; A2 never edits
    it), mtime bumped; the planted copy and its diff kept in `scratch/af1s-a2/vround2/`. Every other file the
    self-test plants into — `Makefile`, `src/`, `test/CMakeLists.txt`, `test/build_selection_test.cpp`,
    `test/chassis_table_test.cpp`, `test/vendor/`, `include/liblvgl/` — = HEAD, nothing untracked; `robot_config.hpp`
    (A2 edits it; the gate's case 19 plants in it) holds A2's own hunks only, nothing appended; no
    `build/src-gate-facts/`, no `build/ROBOT-ONE-GATE-COPY-ONLY`. **For the coordinator:** a leftover-plant check
    must be `git diff --stat` (and untracked files) over EVERY path the self-test plants into, not the marker grep —
    case 36's plant and the other SER1 plants (`ser1Review…`) have no marker. The final gates below check that way
    (before, between the self-test and the check, after), and run the self-test and the check so a TERM, HUP or INT
    reaching their driver is passed on as INT (the `with _plant` restores on KeyboardInterrupt) and waited for.
  - **Re-checked before trusting the 12:47 record:** the banner's rule 1 (:69–73) against `allowed()`'s order (NOT
    SKILLS, the three NO CAMERA checks, the excursion check, FLIPPER NOT READY, WRONG WALL — the code unchanged by this
    round); the V2 sentence (:158–160); the new order case and the rewritten `toggleCameraOf` case, read whole; the
    figure, re-read (one purple ring; three grey x's, each labelled with its point); `mut4/before.log` and
    `mut4/mutations.log`, read (each verdict as the table says, 7 dependents recompiled each). Nothing in the mutated
    files changed after that run but the logged comment reflow (`toggle_visit.hpp`, 13:08:18).
  - **ONE more fix, V3's own premise check.** The 12:47 cut's `static_assert` —
    `sizeof(ToggleColourIds) == 3 && sizeof(ToggleTrackerConfig) == alignof(double) + 13 x sizeof(double)` — and its
    comment ("a field added to the tracker's config changes its size") overstated, as did row V3's "(a field added
    later stops the build, naming this case)": a member of 5 bytes or fewer placed straight after the 3-byte `ids`
    sits in their padding and leaves the size at 112. It now COUNTS MEMBERS: `membersExactly<T>(make_index_sequence<N>)`
    — T brace-initialises from N `AnyMember{}` (a type that converts to any member's type, never defined, used only
    unevaluated) but not from N + 1 — the ids block exactly 3, the config exactly 14 (the ids block and 13 more); the
    comment says so. Probe (`vround2/probe/count_probe.cpp`, `-Werror` with the build's warnings): on the real header
    3 and 14 hold, 2, 4, 13 and 15 are refused; with `bool paddingProbe` after `ids` in a COPY of the header, the new
    check fires and the old `sizeof` check compiles. **Mutation PAD** (`vround2/mut_pad.sh`, 22:56–22:57; the file
    backed up in scratch, restored in a trap on EXIT/TERM/HUP/INT, `cmp`-identical, mtime bumped):
    `bool paddingProbe = false;` straight after `ids` in `include/shulib/vision/toggle_tracker.hpp`. Its first cut
    broke its OWN build at the doc-coverage gate (an undocumented member) — re-cut, never counted: the member given a
    `///<`, and the suite target's own `build.make` driven (its `depend` step, then `make -k`) as `mut4.py` drives it,
    past the doc gates. The re-cut: 122 objects name the header in their depfiles; 121 recompiled CLEAN with the
    mutant (newer than it); the ONE failure is `toggle_visit_ctx_test.cpp.o`, by THIS case's `static_assert` (:728,
    "ToggleTrackerConfig gained or lost a member: add it to this case's numbers and to sameNumbers") — **CAUGHT by
    compile-time assertion, the test's own.** The two touched cases re-run alone first (2 of 2, 255 assertions).
    Row V3's sentence is marked in place.
  - **Seen, not done (for the coordinator, not a finding of this round):** `robot_toggle_camera_test.cpp`'s
    `declared()` case lists the block's 17 numbers by hand — a member added later reaches `declared()` and that case
    only by a person reading; the new `static_assert` above now stops the build on such a member, and its message
    names this case's lists, not that one.
  - **PROSE REVIEW of the resumption** (rule 4 — each grep RUN and its hits READ before this line was written):
    `grep -n "17 numbers\|17 tracker\|each distinct\|changes its size\|sizeof\|padding\|membersExactly\|AnyMember"`
    over `toggle_visit_ctx_test.cpp` and `robot_toggle_camera_test.cpp`: the ctx test's new helper (:683–693, its
    comment "compiles exactly when the aggregate T has n members or more" — true: fewer initialisers than members
    value-initialise the rest), the case's premise comment (:723–725) and its history note (:703); the camera test's
    "17 numbers" (:17, :486 — the block's 16 tracker numbers and the feed's period, `ToggleCameraConfig` = tracker +
    `CameraFeedConfig{minReadPeriodSeconds}` — true) and its `sizeof buf` lines (unrelated). `simulat` over the ctx
    test: only its standing "never 'simulator'" note. This entry re-read against `final_gates5.sh`, `mut_pad.out`
    and `mut_pad_build.log`. No public doc changed in the resumption (a test comment, a test `static_assert`, this
    log).
- **FINAL GATES of the resumption** (`scratch/af1s-a2/final_gates5.sh`, `gates5/`, 22:57–00:14, on the final tree;
  the test binary REMOVED first): the plant check (every plant path = HEAD, nothing untracked, no marker in the
  diff, no `build/src-gate-facts/`, no `build/ROBOT-ONE-GATE-COPY-ONLY`); `api_doc_tool generate`; the briefing
  regenerated with the binary absent; configure; the FULL build, doc gates inside (124 objects compiled — every
  dependent of the restored `toggle_tracker.hpp` among them; no warning); the briefing regenerated with the binary
  present; the suite `green — 3779 cases, 3884675 assertions`; ctest compile refusals 289 of 289; the ARM header gate
  over all 259 `include/shulib` headers (`-Werror`, rc 0); the CI greps (no `pros/` outside hal/pros, no `shulib/sim/`
  outside sim/); api_doc_tool self-test, check-coverage (6473 entities, all documented), check-fresh,
  check-examples (988 lines verbatim), check-removability — all rc 0; `doc_staleness_audit.py` clean;
  `briefing_status.py check` current; `src_build_gate.py self-test` OK (377 detector cases, each against a real
  build; 23:07–23:57); the plant check between — clean, `robot_config.hpp` = its start copy; `src_build_gate.py
  check` PASS (14 builds; robot one's 6 refuse naming exactly their owed facts and link in the gate's typed copy;
  23:57–00:14); the plant check after — clean. Then, as gates3 did, the host build re-run (the self-test's plants
  and restores bump `test/` files' mtimes: 315 objects, rc 0) and the suite again on it: `green — 3779 cases,
  3884675 assertions`. After this entry was written: `doc_staleness_audit.py` and `briefing_status.py check` re-run
  (below). The binary left on disk (00:16) is this tree's own, built from source.
- **After this entry was written** (00:18, `gates5/18_…`, `19_…`): `doc_staleness_audit.py` clean, `briefing_status.py check` current; all 122 objects that name `toggle_tracker.hpp` in their depfiles are newer
  than PAD's restore (none older). Final plant check: `git diff --stat` over every plant path empty.
- **STOPPED HERE (2026-11-17 ~00:20): the verification round is COMPLETE** — four findings, all REAL, all fixed
  (the table); one more fix to V3's own premise check (the resumption); a src-gate self-test plant left by the
  13:53 kill found and removed; every gate green on the final tree. NOT committed. For the coordinator: the merge
  notes in the fix round above (main's tip has moved again since `3c29b73`, to `c778c11` at 00:20 — re-apply, and the src gate again on the
  merged tree), the plant-check lesson above, finding 1's delta for the team lead, and the between-parts marker to
  restore or complete.

## AF1S-A2 — the coordinator's merge *(2026-11-17)*

- **Squash-merged from `af1s-a2`** (transport `feda811`) onto the integrate worktree after the Calibrate fix (main
  b3146d4 + 7867517). Conflicts with DOCS3-N1's new docs resolved in N1's shape: the front page and the cookbook index
  keep N1's layout, cookbook 08 joins the menu under Routes → Recipes and the cookbook index; the glossary keeps N1's
  anchored entries and adds A2's four terms (look spot, push spot, Quadrant colour, Toggle block — anchored and linked
  by `chapter_links.py`); the API pages regenerated (`toggle_camera.md` joins the generated menu block). Cookbook 08's
  opening lost its "Nothing here has run on a robot" phrase (the tone rule; the placeholder sentence stays).
- **API: no version bump** — the run's own history paragraph says no frozen row is touched (RobotContext outside F4,
  the shadow context outside F6's accessors, competition/ and vision/ open — F15); by the project's rule (AF1S-A1's
  precedent) that takes no number, so its entry sits at the top of `## API 2.9`.
- **Gated together with the Calibrate fix and MIRROR1-B** (one gate run on the final tree, `integrate_verify.sh`), then
  the coordinator's mutations; main fast-forwards only after.
- **Gates on the final tree** (Calibrate fix + AF1S-A2 + MIRROR1-B, `integrate_verify.sh freeze3`): suite green (**3,819 cases**); src gate self-test OK and check PASS (14 builds); ARM header gate; doc gates (check-nav, check-tone); briefing. **The coordinator's mutation MF-1** (the shadow context drops the Toggle camera): **RED, 16 cases**. AF1S-A2 LANDED.
