# AF1-S — PROGRESS (live log)

<!-- release: between parts — AF1S-A1 landed 2026-11-13 (5f6633b); AF1S-A2 (the camera reachable from an auton) is queued, not started (declared by the coordinator 2026-11-13 for the first release since August) -->

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
