# COMP — progress log (live)

- **Chunk:** COMP — the competition program. This log carries **COMP-a** (the LIBRARY half; no `src/`)
  first; COMP-b (the program, a seventh build) appends below it in its own run.
- **Brief:** `COMP-competition-program.md` (revised `8ac243f`; binding re-check B1-B27 at `ea58fbc`)
- **Started (COMP-a):** Sat Sep 26 09:35:32 AM EDT 2026
- **Tree at start:** `ea58fbc`, clean, in the git worktree `.claude/worktrees/comp-a` (branch `comp-a`) —
  NOT the main working tree. Parallel builders: T3 (`robot.hpp` mechanisms from config, a
  `mechanism(name)` lookup and a mechanisms span; `config/robot_config.hpp`; `include/shulib/teleop/`),
  DIAG1 (one forwarder in `localization/localizer.hpp` next to `lastOdomDeltaImplausible`), G1 Part 2c.
- **Executor model:** opus
- **Go:** the coordinator's workflow run, 2026-09-26, on the team lead's ruling (B-preamble) that COMP-a
  starts now in an isolated worktree. Do not commit; no robot upload; no `pros` commands; no
  `/dev/ttyACM*`. HA numbers reserved for this run: **HA-161, HA-162 only**.
- **COMP-a IN:** §2 items 1-7 as amended by B1, B9/B10, B12, B13, B14, B15, B16, B18, B20, B22, and
  C1's `BuildOptions::pacer`. **OUT (COMP-b):** everything under `src/`, the `CompetitionWatch` pacer
  (B7), the auton task (B19), the worksheet, the Timed Run HA (B5).

Appended in real time. **Nothing in this chunk has run on a robot.**

---

## Log

### 09:35 — start
- `date` → Sat Sep 26 09:35:32 AM EDT 2026; `git status` clean; HEAD `ea58fbc`.
- Read `docs/internal/RESUMING.md` (the chunk loop, the standards, pace rules 1-8) and the brief in
  full, B1-B27 first. This log created before any code.
- HA register's highest ENTRY at start: HA-146 (DEFECTS2's; the status line reads "7 of 146", and
  HA-147 appears only as a forward mention, "entries from HA-147 on" — *corrected at the review; this
  line first said HA-147*). Reserved elsewhere: T3 HA-147..159, DIAG1 HA-160. This run uses HA-161 for
  the task-kill belief (C5 / §2 item 7), and HA-162 is left unused unless needed.

### required reading (the tree, not the brief's line numbers) — *untimed; see the timing correction in the review subsection*
- Read in full: `sequence/run_guard.hpp` (F12, unfrozen), `localization/localizer.hpp`, `robot/robot.hpp`,
  `chassis/chassis.hpp` (F6), the scheduler's waits/cancel/tick body (`motion/motion_scheduler.hpp`
  ~:595-1250), `runner/path_runner.hpp` (F8), `config/route.hpp` (`Route::valid`, `Start` legal at 0 only),
  `calibration/calibrate_panel.hpp` (budgets :70-107, rects/hit boxes :109-142, `screenText` :329,
  `TouchSample`/`TouchGate` :913-1014), `test/sequence_run_guard_test.cpp`, `test/motion_test_rig.hpp`,
  `test/robot_builder_test.cpp`'s PlantWorld, the EKF's fuse() step A and resync (`ekf_fusion.hpp`
  ~:410-623), `drive_encoder_odometry.hpp` update/setPose, `diag/plausibility_guard.hpp`.
- Findings that shape the design (each re-read in the source, not taken from the brief):
  1. The published heading is `imu.heading() + headingBias_`; the odometry rotates its deltas by the RAW
     IMU heading and the Localizer re-rotates them by the bias (`localizer.hpp` :245-252). So a re-seat
     that sets the bias re-aims POSITION too, and the IMU stays the sole source of heading CHANGE.
  2. The EKF sizes heading process noise from the change in the handed heading (`ekf_fusion.hpp` :455-459);
     a 90 deg bias jump on an ordinary tick would be counted as 90 deg of physical rotation. What avoids it
     is the dt == 0 resync path (:439-441), which the Localizer produces only when `hasLast_` is cleared —
     B14's "re-baseline exactly as setPose does" is therefore load-bearing, and testable.
  3. The scheduler's PoseDeltaGuard (D-5) would raise IMPLAUSIBLE on a teleport, but `async()` and every
     blocking wait re-baseline the loop monitor, so the first tick after a re-seat has dt 0 and only
     re-baselines (`plausibility_guard.hpp` check(): dt <= 0). Same mechanism that keeps
     `Routine::startAt` quiet today; a test pins it for the re-seat.
  4. The OdoStallCheck baselines at each motion's start; a re-seat happens with no motion active, so it
     cannot seed an ODO_STUCK. Pinned by test (B14).
  5. `Routine::startAt` (F10, frozen) calls setPose and ignores heading — unchanged; runAuton re-seats.
  6. `PathRunner`'s `Start` leg calls setPose (position only) — the bias survives it (setPose keeps the
     bias, E3), so a route whose Start equals the entry's start pose keeps the re-seated heading.

### the library, built — *untimed*
- **`sequence/run_guard.hpp` (B1):** `requestAbandon() noexcept` (one atomic store), `abandonRequested()`,
  `RunGuardReport::abandoned`; pace()/expiredNow()/expired()/remaining() treat an abandon exactly as the
  deadline latch and exempt nothing (not even the end action); run() skips the end action when abandoned,
  voids its verdict, disarms at its normal exit; arm() clears a stale request FIRST. Banner section
  "ABANDON" carries the design and the rejected alternative (an abandon that disarms). Every new runtime
  string is ASCII (the file's older strings carry em dashes — untouched, not this chunk's).
- **`localization/localizer.hpp` (C2, B14):** `reseatHeading(math::Angle)` — bias = raw.errorTo(target), pose
  heading updated, twist cleared, `hasLast_ = false` (setPose's re-baseline). Precondition: the estimate is
  live. The banner's "no code path … ASSIGNS a heading" reworded to "no CORRECTOR and no fusion policy";
  the caller seed stated; decision-consistency check written into the banner (below).
- **`localization/complementary_fusion.hpp` (B14):** maxHeadingNudgeRate's doc: "no CORRECTOR can reset the yaw".
- **`robot/robot.hpp` (C1):** `BuildOptions::pacer` (nullptr = `devices.pacer()`), declared after
  `absentDrivePorts`; BuildOptions' and the constructor's docs rewritten to the two lifetimes; the one
  wiring line `chassis_{deps_, options.pacer != nullptr ? *options.pacer : devices.pacer(), …}`. Kept to
  that — T3 edits this file in parallel.
- **New:** `ui/touch_panel.hpp` (generic budgets, Rect/hitBox/contains/overlaps/within/onScreen, TextSize,
  Tone, screenText/copyScreenText, LayoutKey, ArmingClock, TouchSample, ReleaseEdge, hitTest);
  `competition/auton_registry.hpp` (AutonKind, AutonPeriods, StartPose, AutonContext, AutonEntry,
  checkAutonEntry, AutonRegistry); `competition/auton_menu.hpp` (AutonMenu, 2 autons per page + PREV / NONE /
  NEXT, one atomic state word); `competition/match_runner.hpp` (runAuton, AutonRunOutcome, AutonRunResult);
  `competition/acceptance_run.hpp` (B18).
- ARM check with the src/ warning policy (`-Wall -Wextra -Wformat=2`, which carries -Wformat-nonliteral and
  -Wformat-truncation=1) over a scratch TU that CALLS every new function (an unused inline emits no
  format-truncation diagnostics, so the header gate alone would not have seen them): clean.
- Doc-tool finding: its parser refused `std::int32_t{1} << 30` as an "unterminated declaration";
  rewritten as `0x40000000  // bit 30`.
- **Decision-consistency check (B14, logged here as required):** the sentence `localizer.hpp` carried —
  "there is no code path, here or anywhere below, that ASSIGNS a heading" — was grepped for across
  `include/` and `docs/` (`grep -rn -i "assign.\{0,20\}heading\|heading.\{0,30\}assign\|yaw reset\|
  no code path\|IMU-owned\|sole source"`). Echoes found and re-read: `apriltag_corrector.hpp:89-90` ("no code
  path HERE writes a pose or a heading" — the corrector; still true); `complementary_fusion.hpp:86-90`
  ("a yaw reset can never happen" — reworded to correctors); `docs/guide/03-knowing-where-you-are.md:217-218`
  ("There is no code path anywhere that assigns the robot a heading" — FALSE after this change; reworded,
  naming runAuton and reseatHeading). The "IMU-owned" sentences in odometry/arc_step/pilons/drive-encoder
  headers are about the ODOMETRY's heading (still the raw IMU — true). `chassis.hpp:448-450` (setPose: "heading
  stays IMU-owned") is still true of setPose; F6 surface, not edited. `cookbook/01-getting-there.md:30`
  ("Heading is not seeded here; the IMU owns it") is about `startAt` — still true; not edited. No test
  asserted the stronger reading. **INCOMPLETE, found at the review (finding 2):** this grep looked for
  the assignment SENTENCE and missed its CONSEQUENCES — six "the bias is exactly zero / the heading is
  the raw IMU without a heading corrector" claims. Closed in the review subsection below.
- Observation (*first left unfixed and raised with the team lead; FIXED at the review, finding 7 — the
  standard is that a chunk that finds a flaw in an earlier chunk fixes it there*): `Localizer::Quality::
  Uninitialized`'s doc says "update() has never run, or the boot settle window is still open", but
  refreshQuality() runs BEFORE `everUpdated_` is set, so the estimate is still Uninitialized after the FIRST
  update and live from the second. Harmless (runAuton's live-wait takes two ticks, 0.02 s); two of this
  chunk's tests first assumed the documented behaviour and failed, which is how it was found.

### tests — *untimed*
- New files: `test/run_guard_abandon_test.cpp` (6 cases: B1 x4 — mid-motion, the two waits, during the
  end action, the stale request — B10, B22; a seventh was added before the campaign, below. *Corrected at
  the review: this line first said "5 cases" over a list of six*),
  `test/localizer_reseat_test.cpp` (6: C2 x4, B12 precondition, B14 EKF), `test/auton_registry_test.cpp`
  (10), `test/auton_menu_test.cpp` (7), `test/ui_touch_panel_test.cpp` (5 + 11 static_asserts — B20),
  `test/competition_runner_test.cpp` (16 — *first written "17"; the list below sums to 16, and so does
  `grep -c '^TEST_CASE'`*: B13 field/relative, B12 calibrating/never-live, §2.6 x3, C5
  abandon/throw, B9, C6 refusals, B14 preconditions, B16 route-vs-routine bit identity, C1 x3 through the
  builder on robot two's shape incl. the B18 acceptance run).
- Suite: 1,464 → **1,514 cases**, 2,387,533 → **2,388,954 assertions**, 3 skipped, green.
- Two test bugs found by the first run and fixed in the TESTS (not the code): the "live from the second
  update" observation above; and the frozen-wait bound, which first ignored the live-wait's ticks (the
  wait starts at liveAfter, not T+0 — now measured from it).

### the mutation campaign (every one on a fresh binary) — *harness written 10:12, its log last written 10:26 (mtimes)*
Harness (scratchpad `mutate.py`, outside the repo): apply the edit (the old text must occur exactly once),
`touch test/*.cpp`, `cmake --build build/test --target shulib_tests/fast`, REQUIRE the line "Linking CXX
executable shulib_tests" (a mutation without it is reported NOT observed), run the named suites, restore
byte-exact and assert the restore. 31 mutations + 3 re-runs. One more case was added BEFORE the campaign,
when writing it showed a gap: "B1 an abandon voids even an end action that reports success" (the existing
end-action test's action returned Cancelled anyway, so voiding the verdict was unpinned).

| # | Mutation | File | Result (red cases) |
|---|---|---|---|
| M01 | skip the end action on a cut | run_guard.hpp | RED — §2.6 fires/frozen-wait/hard-stop, B9 (4) |
| M02 | arm an unset schedule (runAuton's check forced ok) | match_runner.hpp | RED — C6 refusals |
| M03 | drop the heading re-seat | match_runner.hpp | RED — B13 field, B13 relative, B12 calibrating, C1/B18 (4) |
| M04 | drop abandon's cancel (pace() notes, does not cut) | run_guard.hpp | RED — B1 mid-motion, B1 end action, B22, C5 abandon (4) |
| M05 | drop the default NONE (state starts at entry 0) | auton_menu.hpp | RED — NONE default, paging, arming (3) |
| M06 | a touch changes the choice after start (lock check dropped) | auton_menu.hpp | RED — C4 lock |
| M07 | drop the BuildOptions::pacer forwarding | robot.hpp | RED — C1 cut through the builder, C1/B18 (2) |
| M08 | disarm inside requestAbandon() (B1's listed mutation) | run_guard.hpp | RED — B1 mid-motion, B1 end action, B22 (3) |
| M09 | re-seat before the wait (B12's listed mutation) | match_runner.hpp | RED — 13 cases (the Localizer's precondition throws) |
| M09b | re-seat before the wait AND the Localizer precondition removed | match_runner.hpp + localizer.hpp | **first run: SURVIVED (all green)** — see below; after the test fix: RED — B12 calibrating |
| M10 | reseatHeading does not clear `hasLast_` (B14) | localizer.hpp | RED — B14 EKF no-spike |
| M11 | abandon checked AFTER the world advances (B22) | run_guard.hpp | RED — B1 mid-motion, B22 (2) |
| M12 | registry drops the hardStopAt-past-period check | auton_registry.hpp | RED — B15 schedule, C6 refusals (2) |
| M13 | a new button under the pressed one (PREV drawn in NEXT's slot) | auton_menu.hpp | RED — no-new-button, screen-safe (overlap), paging (3) |
| M14 | a route entry with a different start pose accepted (B16) | auton_registry.hpp | RED — B16 route entries |
| M15 | ui SMALL budget 50 -> 52 (B20) | touch_panel.hpp | RED at COMPILE time — `static assertion failed: B20: ui and calibration SMALL budgets differ` |
| M16 | a null mechanism not refused | match_runner.hpp | RED — C6 refusals |
| M17 | the end action runs although abandoned | run_guard.hpp | RED — B1 mid-motion, B1 waits (2) |
| M18 | arm() keeps a stale abandon | run_guard.hpp | RED — B1 stale request |
| M19 | the bias sign flipped (`heading.errorTo(raw)`) | localizer.hpp | RED — 9 cases (C2 position, B13 x2, B12, B14, C1/B18, …) |
| M20 | the routine runs on a dead estimate | match_runner.hpp | first form did not COMPILE (unused variable) — not an observation; re-run as `false && …`: RED — B12 never-live |
| M21 | hitTest ignores `armed` | touch_panel.hpp | RED — hitTest, lock, arming (3) |
| M22 | an error read moves the release count | touch_panel.hpp | RED — ReleaseEdge, menu release edge (2) |
| M23 | NONE delayed with the other buttons | auton_menu.hpp | RED — NONE default, arming (2) |
| M24 | paging skips one entry | auton_menu.hpp | RED — paging, arming (first run: the arming test SEGFAULTED on a null `find()`; REQUIREs added; re-run: RED cleanly) |
| M25 | registry uniqueness dropped | auton_registry.hpp | RED — B25 names |
| M26 | expired() ignores the abandon | run_guard.hpp | RED — B1 mid-motion |
| M27 | an abandoned end action's verdict not voided | run_guard.hpp | RED — B1 voids (the case added above) |
| M28 | the guard's waits ignore the abandon | run_guard.hpp | RED — B1 waits, B1 mid-motion, B1 voids (3) |
| M29 | the name budget unchecked | auton_registry.hpp | RED — B25 names |
| M30 | "NONE" accepted as an auton name | auton_registry.hpp | RED — B25 names |

**M09b was found GREEN, and the defect was in the TEST.** With the Localizer's precondition removed, a
re-seat made before the live-wait computed the bias from the IMU at T+0 — and the test's calibration
garbage, `2.5 sin(9 t)`, is exactly 0 at T+0, which is also what the IMU reads after calibration (truth
- 90 deg at a 90 deg start). The premature bias was therefore right by coincidence. The garbage's phase
is now `2.5 sin(9 t + 1)` (about 120 deg at T+0), the comment on `DisagreeingImu` says why, and M09b is
RED. The precondition itself (M09) was never the only line of defence the test relied on — now it isn't.
Final state after the campaign: every source byte-restored (the harness asserts it), a fresh
`touch test/*.cpp` rebuild, **1,515 cases / 2,388,967 assertions, 3 skipped, green.**

### the documentation pass and the prose review (greps RUN, hits READ, before this entry) — *untimed*
**Touched:** `docs/changelog.md` (a new "Added since 2.2 — the competition library" section, additive, no
version bump, with its reasoning); `docs/hardware-assumptions.md` (HA-05 AMENDED in the table and the entry —
the start heading's owner is the chosen auton's start pose, seated at autonomous start through the bias;
**HA-161 NEW**, reasoned: PROS deletes its competition tasks on a mode change and not the program's own — two
halves, both unobserved, the second the one COMP's own-task design rests on; status line "7 of 147" + a
COMP-a sentence); `docs/roadmap.md` (Freeze Register F12: the second consumer and the evidence stated, NOT
frozen — the team lead's call; M4 Sequencing: a `[~]` COMP-a row; the "Now" paragraph: one sentence);
`docs/internal/build-order.md` (the COMP entry's status line); `docs/guide/03-knowing-where-you-are.md`
(two sentences made false by `reseatHeading`: "no code path anywhere assigns the robot a heading", and "the
wiring code establishes the starting heading"); `docs/guide/13-extending-the-library.md` (the layer map gains
`competition/` and `ui/`); `docs/guide/15-glossary.md` ("heading bias" — the one seed); `README.md` (the tree
gains `competition/` and `ui/`, and `sequence/` mentions the abandon); `include/shulib/hal/imu_conversion.hpp`
and `include/shulib/hal/pros/device_provider.hpp` (their HA-05 comments: the competition program seats the
start heading at auton start over a boot heading of 0); `docs/api/` + `mkdocs.yml` regenerated.
**The greps (run, hits read):**
- `RunGuard|run_guard|run guard` over docs/README: legacy-command-vocabulary:105, diagnostics-plan:177-180,
  guide/README:135, cookbook/03 (8, 109-199), master-plan:631 — all still true (they describe the deadline,
  which did not change). Glossary:147-152 ("run guard") does not mention the abandon — not false; the chapter
  that explains the abandon is COMP-b's guide chapter, and the glossary line moves with it. roadmap F12 and
  guide/13:16, README:271 — updated.
- the heading-assignment greps — logged at 10:10-11:40 above.
- `construction only|BuildOptions`: robot.hpp (rewritten), motor_group.hpp:85 and device_provider.hpp:77
  (about absentDrivePorts — true), changelog:195 and roadmap:327 (history), HANDOFF:264 (true but now
  incomplete — COMP-b carries the held documentation pass, B21), `src/boot_source.hpp:47,121`
  (`buildOptions()`'s doc must say a pacer the caller adds is borrowed for the Robot's life — B8; `src/` is
  COMP-b's).
- `devices.pacer|provider's pacer|headingBias|heading bias|reseat|selector`: glossary "heading bias"
  (updated), roadmap:598 (E3 history), HA-82 (the fusion rate — still true, it is about correctors),
  vexbuilder-integration:191 (route names — true).
- `setPose|startAt` with "heading" in guide/cookbook/faq/README: guide/10:260 (setPose: position only — still
  true), cookbook/05:104 (startAt with the current heading — true), guide/03:221 (mine), guide/03:387-390
  (updated, above).
- HA-05's anchors in the tree: imu_conversion.hpp:20-26, pros/imu.hpp:10-15/92-109, device_provider.hpp:66-73
  — the first and last updated; pros/imu.hpp's ("bootHeading applied exactly once"; the owner "the robot's
  start pose") still true.
**Deliberately NOT touched, with why:** `chassis/chassis.hpp` (F6 — `setPose`'s doc is still true);
`chassis/routine.hpp` (F10 — `startAt` still seeds position only; B-ruling C2 keeps it); `calibration/
calibrate_panel.hpp` (ruling C4 — moved onto ui/ later, separately); `docs/cookbook/01-getting-there.md:30`
("Heading is not seeded here" — about startAt, true); guide chapter 6 and the glossary's "run guard" (the
abandon and the competition program are COMP-b's chapter "The competition program"); the worksheet, the
HANDOFF, `src/` (COMP-b); build-order's and roadmap's `Next:`/"You are here" POINTERS (several chunks merge in
parallel today and the coordinator orders the landings — the COMP entry's status line and one roadmap
sentence were written instead; the coordinator moves the pointers at merge).

### gates (every one run in the worktree, on the final tree) — *this log last written 10:40 (mtime)*
- Host suite (fresh `touch test/*.cpp` build): **1,515 cases / 2,388,967 assertions, 3 skipped — SUCCESS**
  (start: 1,464 / 2,387,533; +51 cases).
- Doc gates (inside the build and run explicitly): api_doc_tool self-test OK; coverage "2800 public
  entities across 153 headers, all documented"; check-fresh exit 0; examples "408 quoted lines, all
  verbatim"; removability clean; briefing "generated block current" (regenerated LAST); staleness audit
  self-test OK and "clean (26 live docs)".
- CI greps: "library is PROS-free (all of include/shulib except hal/pros/)"; "layering holds: core is
  sim-free".
- ARM header gate: 189 headers, exit 0 (the RESUMING flags). Plus a scratch TU calling every new function
  under the src/ warning policy (`-Wall -Wextra -Wformat=2 … -Werror`): exit 0.
- `tools/src_build_gate.py self-test`: "OK (33 detector cases, each against a real build)";
  `check`: "PASS (all 6 builds link …)", 0 disallowed warnings in each (src/ is untouched; robot.hpp,
  localizer.hpp and run_guard.hpp reach it through teleop_program.cpp).

### Deviations from the brief, each with its reason
1. **The selector shows TWO autons per page**, not "as many as fit": two full-width rows are what fits
   between the status lines and the bottom row with every vertical gap above 2 x the hit pad (a
   static_assert), and full width is what a 29-character name + its kind needs at MEDIUM (B25).
2. **B20's static_assert lives in the host suite** (`test/ui_touch_panel_test.cpp`), not a header — a
   header assert would make every competition header include the whole calibrator (reasoning in
   `ui/touch_panel.hpp`'s banner). It fails the host build and CI, naming the constant (M15, observed).
3. **`endActionRan` when the abandon lands DURING the end action is TRUE** (it WAS invoked — the field's
   documented meaning); B1's "(endActionRan false then)" is read as the case where the abandon arrives
   before scoring returns (the end action is then skipped and it IS false). `endActionSucceeded` is false
   in both cases.
4. **`StartPose` has a third state, UNSET**, so B16's "a route with no Start takes the entry's" and "the
   registry refuses a route entry that also declares a DIFFERENT StartPose" are expressible (unset = take
   the route's). A routine entry must not be unset.
5. **`AutonRegistry` is constructed with the program's two periods (`AutonPeriods`, required)** — B15's
   "the period follows from the program's two constants" needs the registry to know them, and the library
   must not state a match length (run_guard.hpp's banner). `runAuton` also takes `period`, as §2 item 5
   specifies, and re-checks.
6. **The mechanisms null-check in runAuton is a REFUSAL, not a filter** — filtering and logging an absent
   mechanism BY NAME (B9) needs the name, which the Robot/program has and a null pointer does not.
7. **HA-05's amendment and HA-161** were written; the Timed Run HA (B5) is COMP-b's. HA-162 unused.
8. **`AutonRunResult::armedAt`** added (not in the brief) so COMP-b can log B12's hand-off latency.
9. **The build-order/roadmap POINTERS were not moved** (above: parallel merges; the coordinator orders them).

### NOT done (COMP-b's, or the team lead's)
- Everything under `src/`: the program, its auton task (B19), the CompetitionWatch pacer (B7), the boot
  wiring (B8, incl. `BootConfig::buildOptions()`'s doc), the handshake (B2), the students' autons file, the
  seventh build, the worksheet (Station K), the Timed Run HA (B5), the guide chapter "The competition
  program" (with the glossary's "run guard" entry and the abandon), the held 2026-09-25 doc pass (B21).
- The F12 freeze decision (the team lead's; the evidence is in the register row).
- `Localizer::reseat(Pose2d)` (B14's "or one call does both") was not added: runAuton calls setPose then
  reseatHeading on the same tick, which B14 allows; one call is a small additive follow-up if wanted.
- ~~For the team lead: the `Quality::Uninitialized` doc/behaviour mismatch — not fixed here.~~ Fixed at the
  review (finding 7): the doc now states the behaviour.

**Tree left uncommitted, as instructed. Nothing in this chunk has run on a robot.**

---

### 11:05-11:41 — review findings and their disposition (the COMP-a review round, same worktree, same builder)
- `date` → Sat Sep 26 11:05:35 AM EDT 2026. Tree: the uncommitted COMP-a work on `ea58fbc`. Baseline rebuild
  and run BEFORE any change: **1,515 cases / 2,388,967 assertions, 3 skipped, green** (matches the gates
  entry above).
- **Timing correction (found while starting this entry, not by the review):** the first run's section
  headings carried clock times — 10:10, 11:40, 11:45-13:05, 13:05-13:30, 13:30-14:10 — that were NOT read
  from a clock. `date` read 09:35:32 at the start and 11:05:35 at the start of this round; the file mtimes
  put the whole first run inside 09:35-10:40 (the mutation harness written 10:12, its log last written
  10:26, `docs/changelog.md` 10:14, `docs/hardware-assumptions.md` 10:16, `docs/api/match_runner.md`
  10:38, this log 10:40). The headings above now carry no invented times; the evidence is here.

**Per finding (each re-checked in the tree before anything was changed):**

1. **B16 route re-seat untested at the runner level — REAL, fixed in the test.** Re-checked:
   `fourLegRoute()` left waypoint 0 at (0, 0, 0), the routine twin started at `Pose2d{}`, and the RecRig
   had no Degradation, so the IMU read 0 and the seat computed a zero bias — both listed mutations would
   have survived. The case is rewritten ("B16 a route entry is seated from its Start, and runs
   bit-identically to the same routine"): the route's Start is (-48, -24) facing 90 deg (`Fixed`), the
   plant stands there with `DisagreeingImu{-90}` (the IMU reads 0), the entry's own start is UNSET
   (`REQUIRE_FALSE`); asserted: `ra.seated` == the Start (x, y, heading), `ra.imuAtSeat` ~ 0, the seat
   line, ground truth at the field-frame end (-24, 0) facing 0, no ODO_STUCK / IMPLAUSIBLE, and the
   routine twin (seated from `StartPose::field(kRouteStart)` over the same disagreeing IMU) bit-identical
   by `memcmp` at every tick. **R1a** (`result.seated = entry.start.pose()`) and **R1b** (route entries
   skip the seat) both RED; R1a's detail: seated (0, 0, 0), truth 48.0 in and 90 deg from the end pose,
   the twins' tick counts 445 vs 439.
2. **reseatHeading falsified "the bias is exactly 0 without a heading corrector" — REAL, fixed.** The first
   decision-consistency grep looked for the ASSIGNMENT sentence and missed its CONSEQUENCES. Grep RUN before
   this claim: `grep -rn -i "heading-providing corrector\|no-heading-corrector\|no heading corrector\|EXACTLY
   0\.0\|bit-identical to the raw\|drift measure\|found to have drifted" include/ docs/*.md docs/guide
   docs/cookbook README.md` (docs/api excluded — generated). Hits read: the review's five (localizer.hpp
   position note, `headingBias()`'s doc, `biasedHeading()`'s doc, the `headingBias_` member comment,
   ekf_fusion.hpp's θ-gain sentence) plus a SIXTH the review did not list — STEP 2's early-out comment in
   `update()` ("a no-heading-corrector tree bit-identical"). All six reworded to "no heading-providing
   corrector AND no reseatHeading() seed"; `headingBias()`'s doc no longer calls the bias a drift measure
   without qualification (after a competition auton starts it carries the start seed — the review's +90 /
   RELATIVE examples are in the doc). The banner's decision-consistency record now says the first check was
   incomplete and lists all six with the grep. Left as history, deliberately: `docs/roadmap.md:605` (E3's
   delivery record — true of E3's tree), `build-order.md`:485 and the E3/E4 COMPLETED records. The teleop
   "EXACTLY 0.0" hits are about sticks, unrelated. Every test asserting a zero bias
   (`apriltag_corrector_blackbox_test` :232/:413, `apriltag_corrector_heading_test` :391,
   `ekf_fusion_seam_test` :298, `localizer_reseat_test`'s refusal case) builds a tree with no seed — still
   true. `docs/changelog.md` gains the headingBias() consequence. No runtime consumer of `headingBias()`
   exists (grep of include/ src/ test/: tests only) — a documentation-truth defect, as the review said.
3. **A NotStarted run still ran the entry's end action — REAL, fixed in the code.** `runAuton`'s end-action
   lambda now fills the slot with `chassis.brake()` when `result.outcome == NotStarted`, and Warn-logs
   "auton '<name>': never started -- its end action is NOT run (...); braking instead" when the entry had
   one. Documented at `AutonRunOutcome::NotStarted` and the banner's step 6; changelog. New case "B12 a run
   that never started brakes -- the entry's park is not driven in an un-seated frame": IMU ready at T+2.1,
   endActionAt T+2.0, hardStopAt T+6.0, a counting field-frame park. **R3** (the branch disabled) RED, and
   the detail shows the hazard was real, not hypothetical: the park ran (1 call) and drove the plant
   **57.5 in** from where the robot stood. *Decision, with the alternative:* documenting the hazard only
   (the review's second option) was rejected — the robot would still drive a park in the wrong frame; the
   substitution costs nothing a NotStarted run could use (it never moved, and its park was written for a
   pose that was never seated). `guard.endActionRan` stays true (the slot ran — as brake), which NotStarted's
   doc now states.
4. **Abandon during the live-wait untested; the cause sentence's mutation survived — REAL, fixed in the
   test.** New case "C5 an abandon during the live-wait: NotStarted, 'was abandoned', skipped, nothing
   seated": IMU ready at T+2.0, `watch.disableAt = 0.5`; asserted: NotStarted, the routine and the park never
   called, `why` has "was abandoned" and not "ran out of time", `guard.abandoned`, `!endActionRan`, not
   `scoringCut`, `liveAfter` in [0.5, 0.53], no seat line, `seated` untouched, `headingBias() == 0`, zero
   plant travel, guard disarmed, drive safe. **R4** (the ternary made the constant "ran out of time") RED.
5. **Smaller surviving mutations — all three REAL, fixed in the tests.**
   (a) `AbandonRig` gains `seqLineCount` and `abandonedLineAt(n)` (parses the T+ of the n-th "run ABANDONED
   at T+" line). B1 mid-motion now asserts exactly one such line at T+[1.0, 1.03]; the waitFor subcase
   asserts the line at T+[0.7, 0.73] (there the WAIT sees the abandon first, so the line must come from
   `expiredNow()`); new case "B1 each abandoned run logs its own 'run ABANDONED at T+' line, at its own
   time" — two runs on ONE guard, abandoned 0.5 s and 0.8 s into each, both routines drive only (no guard
   wait), two lines with their own T+. **R5a1** (pace-side note deleted), **R5a2** (wait-side note deleted),
   **R5a3** (both) and **R5a4** (`abandonNoted_ = false` deleted from `arm()`) all RED.
   (b) New case "C2 reseatHeading clears the twist, and the dt-0 tick after it keeps it clear": a MOVING,
   turning estimate (10 in/s along +x, 0.25 rad/s) re-seated to 90 deg; the twist is zero at once, the dt-0
   tick after it keeps linear velocity zero (that tick KEEPS the previous linear velocity by update()'s
   rule — which is why the clear matters: without it the tick publishes 10 in/s along the OLD +x) and
   refreshes omega, and the next ordinary tick reports 10 in/s along +y. **R5b** RED.
   (c) New case "C2 a position-only (GPS-like) fix after a seat moves the position, never the heading", on
   BOTH fusion tiers: after a 90 deg seat over an IMU at 5 deg, a `providesHeading = false` proposal 3 in
   east / 2 in south whose heading field says 93 deg (inside every heading gate, so code that wrongly read
   it would move the heading); position converges onto the fix, the heading is exactly the seed on all 600
   ticks and the bias is bit-unchanged. **R5c1** (ComplementaryFusion's `providesHeading` skip removed) and
   **R5c2** (EkfFusion's) both RED — the composition claim is now observable on each tier.
6. **A Free Start's heading was seated silently — REAL, fixed in the code (the decision logged here).**
   Re-checked: `effectiveStart()` read `waypoints[0].heading` regardless of `headingMode`, and route.hpp
   says `heading` is "Read only when headingMode == Fixed" and, for a Start, "neither mode is read".
   *Chosen:* a Start's heading is the start heading only when it is `Fixed`; a `Free` Start states no
   heading, so `effectiveStart()` returns UNSET for it, and `checkAutonEntry` refuses an unset entry over a
   Free Start ("its route's Start states no heading (Free) -- make it Fixed, or declare a FIELD start pose
   at the same (x, y)") and accepts a declared FIELD start at the Start's (x, y) with ANY heading (the Start
   states none to disagree with); a different place, or RELATIVE, is still refused as different.
   *Rejected:* refusing every route entry with a Free Start outright — simpler, but it would force a
   VexBuilder route whose start is exported `"free"` to be edited even when the entry states the heading;
   the chosen rule keeps route.hpp's "read only when Fixed" TRUE rather than making it a lie.
   *F8 consistency check:* `Route::valid()` is untouched (a Start still accepts either mode — asserted by
   the new test's `REQUIRE(freeStart.valid())`); PathRunner's Start leg still seeds position only; no F8
   signature, enumerator, capacity or rule moved. What changed is WORDING in two F8 documents, now true
   of the new reader: route.hpp's HeadingMode note for `Start` and `Waypoint::heading` (a competition
   entry reads a `Fixed` Start's heading as the start heading), and the VexBuilder verb table's `"start"`
   row ("Reads heading": was "yes"; now "when `"fixed"` — the heading a competition auton is seated to").
   Raised with the team lead below, since F8 is frozen. New case "B16 a Free Start states no heading: the
   entry must, or it is refused"; **R6a** (the refusal dropped), **R6b** (a Free Start's heading compared
   as if stated) and **R6c** (`effectiveStart` ignoring the mode — the pre-review behaviour) all RED.
7. **The `Quality::Uninitialized` doc flaw left unfixed — REAL, fixed.** The enum's doc now states the
   behaviour and its reason (the class is judged before the first call counts as an update, and that call
   has no earlier tick to take a dt against — so an IMU-ready-from-the-start Localizer is live from its
   SECOND update(); a not-ready boot is live once ready and settled, which can be its first ready tick).
   `qualityClass()`'s doc and `AutonRunResult::liveAfter`'s doc say the same. Behaviour unchanged. The
   deferral was not forced by DIAG1: the enum's doc is ~300 lines from DIAG1's forwarder site and
   `qualityClass()`'s edited lines sit more than ten unchanged lines above it (finding 2's `headingBias()` edit is the close one, 3
   unchanged lines below it), and a synthetic merge shows all of it merging cleanly (below).
8. **HA-161's citations — REAL, fixed.** Re-read: `drive_program.cpp` says "Stated so nobody believes the
   guard covers a kill" and `calibrate_program.cpp` says a deleted task runs no destructors — neither says
   "do not cover a kill". HA-161's Source now quotes each file verbatim (teleop_loop.hpp step 1, main.cpp's
   X-drive loop, drive_program.cpp, calibrate_program.cpp as "the same belief, nearly word for word",
   robot.hpp), names T3's static-state rule as a second consequence of half (a) (bindings, mechanisms and
   their `IMotor*` arrays in the teleop program's static state, never on the driver task's stack — cited
   by content, not by path: a public doc may not point into `docs/internal/`), and both the status line
   and the section intro now say "two parallel chunks — T3 HA-147..HA-159, DIAG1 HA-160".
9. **Glossary and chapter 12 — REAL, fixed with one clause each.** Glossary "run guard": "unless the
   program *abandons* the run (the field disabled the robot mid-auton), when the motion is cut at once and
   the end action is skipped". Chapter 12's SEQ walk-through gains one bullet for the two ABANDONED lines
   (the first's T+ says when). The full explanation stays COMP-b's chapter.
10. **Two log inaccuracies — REAL, corrected above in place** (each correction marked *in italics*): the
    highest HA ENTRY at start was HA-146, not HA-147; `run_guard_abandon_test.cpp` had 6 cases before the
    pre-campaign addition (the list said six, the number said 5) and 7 after; `competition_runner_test.cpp`
    had 16, not 17. The totals (51 new, 1,464 → 1,515) were right.

**Mutation campaign, this round** — harness `scratchpad/comp_a_r1/mutate_r1.py` (outside the repo): each
mutation alone; `touch test/*.cpp`; `cmake --build build/test --target shulib_tests/fast`; the line
"Linking CXX executable shulib_tests" REQUIRED (none was missing); the named test file run; the source
restored byte-exact and asserted; after the campaign, `sha256sum -c` of every competition/, run_guard and
localization header against a pre-campaign snapshot: all OK. **15 mutations, 15 RED, none survived:**

| # | Mutation | File | Red case(s) |
|---|---|---|---|
| R1a | seat a route from `entry.start`, not the effective start | match_runner.hpp | B16 seated from its Start |
| R1b | route entries skip the re-seat | match_runner.hpp | B16 seated from its Start |
| R3 | a NotStarted run still runs the entry's end action | match_runner.hpp | B12 never started brakes (park ran, plant moved 57.5 in) |
| R4 | the NotStarted cause always "ran out of time" | match_runner.hpp | C5 abandon during the live-wait |
| R5a1 | the pace side never notes the abandon | run_guard.hpp | B1 each abandoned run logs its own line |
| R5a2 | the wait side never notes the abandon | run_guard.hpp | B1 inside waitFor/pause |
| R5a3 | neither side notes it | run_guard.hpp | B1 waitFor/pause, B1 mid-motion, B1 each run (3) |
| R5a4 | `arm()` keeps the once-per-run note | run_guard.hpp | B1 each abandoned run logs its own line |
| R5b | reseatHeading keeps the old twist | localizer.hpp | C2 clears the twist |
| R5c1 | ComplementaryFusion reads a heading from a position-only proposal | complementary_fusion.hpp | C2 position-only fix |
| R5c2 | EkfFusion reads a heading from a position-only proposal | ekf_fusion.hpp | C2 position-only fix |
| R6a | the Free-Start refusal dropped | auton_registry.hpp | B16 a Free Start states no heading |
| R6b | a Free Start's heading compared as if stated | auton_registry.hpp | B16 a Free Start states no heading |
| R6c | `effectiveStart` seats a Free Start's heading (pre-review) | auton_registry.hpp | B16 a Free Start states no heading |

(R1a and R3 were re-run with assertion detail after the campaign, on fresh binaries, for the numbers above.)

**Merge with DIAG1 (checked, not assumed):** `scratchpad/comp_a_r1/diag1_merge_check.sh` — `git merge-file`
of this round's `localizer.hpp` against the `ea58fbc` base and a DIAG1-shaped edit (one forwarder inserted
right AFTER `lastOdomDeltaImplausible()`, and, variant B, right BEFORE its doc block): **CLEAN, 0 conflicts,
both variants**; the merged file carries both changes. `headingBias()`'s doc was edited from its third line
on for exactly this reason.

**Files touched this round:** `include/shulib/competition/match_runner.hpp`, `auton_registry.hpp`;
`include/shulib/localization/localizer.hpp`, `ekf_fusion.hpp` (docs only); `include/shulib/config/route.hpp`
(docs only — F8); `test/competition_runner_test.cpp`, `test/auton_registry_test.cpp`,
`test/localizer_reseat_test.cpp`, `test/run_guard_abandon_test.cpp`; `docs/hardware-assumptions.md`
(HA-161), `docs/changelog.md`, `docs/guide/15-glossary.md`, `docs/guide/12-when-things-go-wrong.md`,
`docs/vexbuilder-integration.md` (the start row — F8), `docs/api/` + `mkdocs.yml` (regenerated), this log;
`docs/internal/PROJECT-BRIEFING.md` (regenerated LAST).

**Tests, this round:** 6 new cases, 1 rewritten (the B16 differential). Per file now: run_guard_abandon 8,
localizer_reseat 8, auton_registry 11, auton_menu 7, ui_touch_panel 5, competition_runner 18 (57, from 51).

**Gates, this round (every one run in the worktree, on the final tree, after the campaign):**
- Host suite, fresh binary (`touch test/*.cpp`, `/fast` build, the link line present, 0 warnings):
  **1,521 cases / 2,389,066 assertions, 3 skipped — SUCCESS** (this round: +6 cases, +99 assertions).
  *A first "final" run read RED (1 case) and was NOT a result: the full build's doc gate stopped before the
  link (the briefing block was stale), so the suite ran the binary the last DETAIL mutation run (R3) had
  left. The fresh build above is the result; the order is now suite first, briefing last.*
- Doc gates, run explicitly, each exit 0: api_doc_tool self-test OK; coverage "2800 public entities across
  153 headers, all documented"; check-fresh; examples "408 quoted lines, all verbatim"; removability "no
  public doc references docs/internal/"; staleness audit self-test OK and "clean (26 live docs)".
  `api_doc_tool.py generate` re-run after the last header edit.
- CI greps: "library is PROS-free (all of include/shulib except hal/pros/)"; "layering holds: core is
  sim-free".
- ARM header gate: 189 headers, "ARM compile gate: CLEAN (all v2 headers)". The scratch TU that CALLS every
  new function (now also `effectiveStart()` and `Route::valid()`) under `-Wall -Wextra -Wformat=2
  -Wconversion -Wsign-conversion -Wshadow -Werror`: exit 0. (Run once by mistake at
  `-Wformat-truncation=2`, stricter than the policy: it flags ~38 pre-existing sites across the tree —
  route.hpp, plausibility_guard.hpp, motion_scheduler.hpp, run_guard.hpp's older lines and a few of
  COMP-a's — none from this round's one new snprintf. Not the policy; recorded, not chased.)
- `tools/src_build_gate.py self-test`: "OK (33 detector cases, each against a real build)"; `check`:
  "PASS (all 6 builds link …)", 0 disallowed warnings in each.
- Runtime strings: every new one ASCII (grep for non-ASCII inside string literals in competition/ and
  ui/: none); every screen-bound sentence still goes through `"%s"`.
- `briefing_status.py generate` LAST, then `check` and the full build (which runs every doc gate) — below.

**For the team lead (this round):**
1. F8 is frozen and this round changed WORDING in two of its documents (no signature, enumerator, rule or
   capacity): route.hpp's `HeadingMode`/`Waypoint::heading` notes and the VexBuilder verb table's `"start"`
   row now say a `Fixed` Start's heading is the start heading a competition auton is seated to, and a
   `Free` one states none. Veto or keep.
2. The Free-Start rule itself (finding 6) is a design choice with a rejected alternative, logged above —
   refuse-outright was the simpler one.
3. The NotStarted brake substitution (finding 3) changes what a student's end action can expect: it never
   runs in a run that never started.

- Briefing regenerated LAST (11:39): "briefing status: generated block current" (check exit 0); its suite line
  now reads 1,521 / 2,389,066, green. Then the full host build (every doc gate inside it) exit 0 and the
  suite once more: **1,521 / 2,389,066, 3 skipped, SUCCESS.**

**Tree left uncommitted, as instructed. Nothing in this chunk has run on a robot.**

---

## The coordinator's merge — COMP-a (2026-09-26, ~13:40–14:00)

Squash-merged from the transport commit `9e21412` (branch `comp-a`) onto `9654b56` (T3 Part 1, then
DIAG1 Part A, on top of `ea58fbc`). **One code conflict, `include/shulib/robot/robot.hpp`:** T3 had
made the constructor build the mechanisms last and extended its doc; COMP-a had made the Chassis take
`options.pacer` when set and reworded the same doc for the borrowed pacer. Resolved to both: the Chassis
is built on `options.pacer` or the provider's pacer, then `buildMechanisms(devices)`; the doc says
`absentDrivePorts` is read during construction only, the pacer is kept for the Robot's life, and the
mechanisms are built last. `include/shulib/localization/localizer.hpp` merged cleanly with DIAG1's
forwarder. Documentation: `docs/hardware-assumptions.md` — **7 of 156** (+HA-161); COMP-a's status
sentence kept without its "parallel chunks hold the numbers below it" clause (both have landed; its
subsection intro says so); HA-161's index row after HA-160; COMP-a's subsection after T3's.
`docs/changelog.md` — COMP-a's "Added since 2.2" above DIAG1's. `build-order.md` — the COMP status line
reads LANDED, and "57 new host cases" (the builder's "51" predates its review round's six); the `Next:`
block names both landings. `roadmap.md` — the COMP-a note reads LANDED. `docs/api/*` and the briefing
regenerated.

**Verification (the coordinator):** in the worktree before the merge — every gate green (1521 cases)
and eight hand mutations RED on fresh binaries (the end action running after an abandon; `pace()` /
expiry never seeing the abandon; a stale abandon surviving `arm()`; `runAuton` skipping the heading
re-seat; `reseatHeading`'s bias sign flipped; its dt re-baseline dropped; a never-started run driving
the entry's park; a route running with no command registry); `run_guard.hpp`'s latch, `runAuton`
and `Localizer::reseatHeading` read against B1, B9, B12–B16 (the wait inside the guarded run ticks the
scheduler, whose tick updates the localizer first, so the estimate can go live during it). On the
merged tree: see the commit.

**Not done:** COMP-b — the program (`src/`, the seventh build), its own auton task and watch pacer,
Station K, the Timed Run HA, the worksheet, the held documentation pass — after G1 Part 2c's final run.
**Nothing has run on a robot.**
