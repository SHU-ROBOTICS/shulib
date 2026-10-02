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

---

## Part b

- **Chunk:** COMP-b — "shulib Competition", the competition program: `PROGRAM=competition`, the EIGHTH
  build, slot 6 (slot 5 stays HELD for the team's old-code fallback).
- **Brief:** `COMP-competition-program.md` — the header (ND1-ND6), the 2026-09-28 re-check RC1-RC12
  (BINDING over B1-B30 and the body), B1-B29, the COMP-t amendments with B30, then §§0-5 as superseded.
- **Started (COMP-b):** Tue Sep 29 10:16:38 PM EDT 2026 (`date`).
- **Tree at start:** `0f56301` (COMP-t landed), clean, in the git worktree `.claude/worktrees/comp-b`
  (branch `comp-b`) — NOT the main working tree. Scratch and mutation backups:
  `.claude/coordinator/scratch/comp-b/` only.
- **Executor model:** opus.
- **Go:** the team lead's, recorded in the brief's header (2026-09-28, "i want to do all this"; ND1/ND2
  answered ~16:15; ND3-ND6 the coordinator's, as recommended), relayed by the coordinator's workflow run.
  Do not commit; no robot upload; no `pros` commands; no `/dev/ttyACM*`. HA: **HA-162 ONLY**.
- **In flight elsewhere at the same time (their files are not mine):** DIAG1 Part B, SAFE-SAVE Part A,
  L9 Part A1, H1 Part A1, R5a Part A, T4 Part 2a.

Appended in real time. **Nothing in this part has run on a robot; Station K is its first run.**

### 22:16 — start
- `date` → Tue Sep 29 10:16:38 PM EDT 2026; `git status` clean; HEAD `0f56301`.
- Read `docs/internal/RESUMING.md` (the chunk loop, the standards, pace rules 1-8) and the brief in full,
  the header and RC1-RC12 first. This section created before any code.

### 02:57 (2026-09-30) — RESUMED after the first run was stopped
- `date` → Wed Sep 30 02:57:12 AM EDT 2026. The first run was STOPPED (the session limit) after writing only
  the entry above.
- What the stopped run left, measured: `git status` shows ONE modified file, this log (+26 lines, the section
  header and the 22:16 entry); `git diff --stat -- src/ Makefile tools/ test/` is EMPTY — no gate self-test
  plant was left in `src/` or the `Makefile`, and no code was written. No `bin/`, `.d/` or `build/` exists in
  the worktree (nothing was built). The scratch directory `.claude/coordinator/scratch/comp-b/` is EMPTY (no
  mutation backup is outstanding).
- So nothing is to be kept or reverted: this run starts the work from the 22:16 entry, re-reading the brief
  (done again in full, header, RC1-RC12, ND1-ND6, B1-B30, §§0-5) before any code.

### 03:10 — the baseline, measured before any edit
- Host suite at `0f56301` (this worktree, `cmake -S test -B build/test` then `cmake --build build/test -j8`):
  **1,808 cases / 2,652,625 assertions, 3 skipped, 0 failed** — the number COMP-t's record gives for its
  merged tree. The briefing regenerated with the suite binary present is byte-identical to HEAD's
  (`briefing_status.py check`: current).
- `src/main.cpp` at start: **280 lines** (RC12/ND6's count, re-measured: `wc -l`).

### 03:15 — the tree since the brief's re-check (e124b88), every cited place re-anchored
The brief's citations were written against `b81a590`/`e124b88`. Two commits landed since: TANK1 Part B
(`a5bd83f`: the ` xt` column on translating tank result lines; `CompletedMotion::hasCrossTrack/crossTrack`;
no API bump — TANK1 complete, API 2.3) and COMP-t (`0f56301`). Each citation re-read in today's tree; the
deltas, and how I adapt (quoted text governs, per B23):
| Brief cites | Today | Adapted |
|---|---|---|
| RC4 Makefile :134 "Four programs" / :137 "only meaningful on robot two" / :142 "fourth, fifth and sixth" / :163 valid values | :142 "Five programs in five slots"; :145-150 "`drive`, `library`, `calibrate` and `autontest` are only meaningful…"; :151-153 "fourth to seventh entries"; :178 the valid-values `$(error)`; the autontest branch :172-176 | edit by quoted text: "Six programs in six slots", `competition` added to the list with its reason, "fourth to eighth", the new branch after autontest's |
| RC4 build_selection :8-12, :72-75, :82, :108-110, :141 | rule list :8-12; the one-PROGRAM guard :72-75; the needs-tank guards :76-87 (autontest's :85-87); the enum :92-99; the chain :119-147; beacon comment :110-118 ("the other six" :112, prefix list :113-114); robotBeacon's doc :149 "the five tank builds" | same edits, one more of each; "other seven", "six tank builds" |
| RC4 main.cpp :22-37, :33, :37, :49-51, :198-202, :230-237 | table :22-35; "Five programs in five slots" :36; "the five tank builds" :40; the WHAT RUNS bullet :52-55; `autonomous()`'s doc :209-219; `opcontrol()`'s doc :249-258 | COMP-t already reworded both autonomous sentences to "only in the competition program's autonomous() and, on the bench, the Auton Tester's RUN" — true once COMP-b lands; RC4's reword is applied to the parts that say "commands NO motion in ANY build", which go false |
| RC4 gate :52, :80, :173-174 | docstring builds table :21-58, "per build" :60-61, beacon list :89, **:189-191 "(the Auton Tester, COMP-t, was the sixth; COMP-b's competition program will be the seventh)"**, :124 "and for the seventh at COMP-t", :248 the self-test case list; tables :325-454 | every one edited; B29 as amended: the pair is **34/35** (case 33 is COMP-t's tables-agree check) |
| RC4 ci.yml :7-8, :110-113 | :7-8 "seven since COMP-t…", :116-120 | "eight" |
| RC4 RESUMING :116-117, :134-136 | :114-118 ("…since COMP-t — seven builds"), :133-137 | "eight builds", `PROGRAM=competition` |
| RC1 `run_guard.hpp` :459 | :480 (`RunUnwindGuard unwind{*this};  // throw ⇒ cancel-all + disarm + rethrow`) | cite by text |
| RC1 `motion_scheduler.hpp` WaitUnwindGuard :915 / FlagScope :933, waits :740-741, :772-773 | `WaitUnwindGuard` class :971, used :797, :829 | cite by text |
| RC3 `auton_menu.hpp` :190-191 (one task), :274-277 (unlock's doc) | :189-191 and :274-277 — unchanged | — |
| status block `match_runner.hpp` :160-166 (null refusal) | :161-169 | cite by text |
| RC8 `boot_source.hpp` :370 (`BootOutcome::built`), :50 | `built` :374; the door :47-56 | `st.boot.built.buildOptions()` as spelled |
| RC5 `teleop_program.cpp` :186 (`controlsMatchTable`), `teleop_claws.hpp` :218 / :297, pins `test/teleop_claws_test.cpp` :487, :490, :789 | unchanged: :186, :218, :297; the three pins :487, :490, :789 | the reword below |
| RC5 worksheet :1306-1307 (J.3.1's quotes) | :1308-1309 (the two quoted lines; J.3.1 itself :1306) | re-quoted |
| RC9 worksheet J.9.6 :1496-1503 | :1498-1506 | re-read |
| RC2 register :271 (HA-161), :84 ("HA-162..HA-178") | HA-161 and the reserved-numbers sentence moved with COMP-t's HA-177/HA-178 | cite by text |
| RC5 cookbook 06 :268, guide 07 :125-126, `src/team/README.md` :7-8, roadmap :353 | cookbook :268 "**No program runs this file yet.**…"; guide 07 :123-127; README :5-9; roadmap :351-356 (COMP-t's "Now"/Next) | edited by quoted text |
| RC5/RC10 build-order :632, :1479, :1501, :640 | :632 is the whole Next block (it names T4 1a "NO PROGRAM RUNS IT YET"); COMP's DoD is :1507; :640 (the "minimal auton hook" history block) unchanged and LEFT ALONE (RC10) | my own sentences only |
| RC11 guide README :49-51, `api_doc_tool.py` :1831, :1853 | README :48-55; the tool's globs moved with COMP-t | cite by text |
- Also re-read: COMP-t's pieces the move touches — `src/programs/auton_abort_watch.hpp` (`testerTouchFrom` :117-128),
  `src/programs/auton_tester_run.hpp` (`pacerChainProven` :176-183, `stopAfterProgramError` :188-204),
  `src/programs/auton_tester_program.cpp` (the refusal painter `paintRefusal` :367-397 and `refuse` :399-404;
  the PICK page's painter `paintLine`/`paintButton`/`paintFrame`'s Pick branch :244-270, `rgb`/`printSize`
  :223-242). **The CODE governs** (the review changed the brief's VERBATIM blocks: `runTesterOnce` takes
  `runPressCount`; `pacerChainProven` also requires `watch.attachedTo(guard)`).

### 03:20–03:45 — STEP 1: ND3 (b) — COMP-t's shared pieces MOVED into `src/common/`, the tester re-pointed
The coordinator's ruling ND3 (b) (2026-09-28): at the second consumer, as the first step, move the shared
halves into `src/common/`, PROS-free, and re-point COMP-t's call sites and pins, logged. What moved, where,
and what the tester calls now (the CODE governed, not the COMP-t brief's text — the review's `attachedTo`
term is in the moved proof):
| Piece (COMP-t) | New home | The tester now |
|---|---|---|
| `pacerChainProven(chassis, guard, const AbortWatch&)` (`auton_tester_run.hpp`) | `src/common/guarded_run.hpp`: `template <WatchLike Watch> pacerChainProven(...)` — the same body (one idle tick of `kGuardedTickSeconds` = 0.01 s; `pacesTotal` AND `paces` moved AND `attachedTo(guard)`); the concept names the two members it reads | calls it by the same name (template deduction); its own definition deleted; `static_assert(kTesterTickSeconds == kGuardedTickSeconds)` added |
| `stopAfterProgramError(chassis, present, AbortWatch&)` | `guarded_run.hpp`: `stopAfterProgramError(chassis, present) noexcept` — cancel, then every present mechanism, each alone | keeps a thin wrapper (same signature): the shared stop, then `watch.disarm()` — the order it had |
| `testerTouchFrom` (`auton_abort_watch.hpp`) | `src/common/screen_touch.hpp`: `ScreenTouch{ui::TouchSample, pressCount}` + `screenTouchFrom<ProsTouch>()` (the one place PROS's five touch fields are read) | `testerTouchFrom` = the shared reduction + its own `fingerDown` (same formula); `TesterTouch` stays the tester's |
| the status bits `kStatusDisabled/Autonomous/Connected` (`auton_tester_panel.hpp`) | `src/common/state_screens.hpp` (the static_assert against PROS moved to `state_screens.cpp`) | the panel includes `common/state_screens.hpp`; names unchanged |
| the refusal painter `paintRefusal` + `refuse` | `src/common/screen_paint.{hpp,cpp}` (PROS-free header, PROS-bound .cpp, as ND3 gives it): `paintRefusalScreen(RefusalScreen{tag, barWord, buildTime, headline, why, footer})`, `kRefusalMoreOnSerial`, `refusalBarFits()`/`refusalFooterFits()` | `refuse()` keeps its signature and storage and calls the shared painter |
| the PICK page's painters `rgb`, `printSize`, `paintLine`, `paintButton`, `paintFrame`'s Pick branch, `paintText`'s Pick branch, `kBg` | `screen_paint.{hpp,cpp}`: `toneRgb()`, `kPanelBg`, `lineDiffers()`, `clearPanel()`, `paintLine()`, `paintButton()`, `paintBar()`, `paintMenuFrame()`, `paintMenuText()` | `paintFrame`/`paintText` keep their names (the pinned `ScreenPacer::pace()` calls them) and call the shared ones in the same order |
- **A DELTA from ND3's list, forced by B11's "never copied":** ND3 names the refusal painter; the AUTON
  SELECTOR's painter (the tester's PICK page — `competition::AutonMenu`'s view) is the SAME code the
  competition program must paint, so it moved with the refusal painter into the same pair. Leaving it would
  have made COMP-b's menu painter a near-copy (ND3 (c), rejected). Teleop's own painter is NOT touched
  (ND3: its pinned file; it retires at T4 Part 3) — said in `screen_paint.hpp`'s banner.
- **Proof the tester's behaviour is unchanged:**
  1. The host suite, every tester test UNCHANGED (only a new pin case appended): 1,808 → 1,815 cases, all
     green (the assertion count moved −31 only because the source-pin loops iterate over the string literals
     of `auton_tester_program.cpp`, and the moved painter's literals left that file).
  2. **Seven mutations on the MOVED code, each on a fresh test binary, observed RED through the TESTER's own
     unchanged tests** (so those tests reach the shared code): MV1 the proof's `attachedTo` term dropped →
     tester (13b) and (20b) RED (+ G1); MV2 the stop's mechanism loop broken → tester (21c) RED (+ G2); MV3b
     `chassis.cancel()` dropped → (21c) RED (+ G2); MV4 release count read from `press_count` → tester (6b)
     RED (+ G3); MV5 press count from `release_count` → tester (6b) and (15b) RED (+ G3); MV6
     `kStatusAutonomous` = 1<<3 → tester panel (25) RED (+ G4); MV7b the wrapper not calling the shared stop →
     (21c) RED. (MV3 and MV7 as first written left a parameter unused and FAILED TO COMPILE under -Werror —
     not counted; re-run as MV3b/MV7b keeping the parameter used.) Every file restored byte-exact (asserted by
     the runner) and its timestamp bumped. Records: `scratch/comp-b/mut_move_results.jsonl`.
  3. **The V5 package, byte for byte in what it says:** `make ROBOT=tank PROGRAM=autontest` built from a
     `git archive 0f56301` snapshot and from a snapshot of the moved tree (both under `scratch/comp-b/`, both
     hash-less, same flags): the 717 read-only strings of `hot.package.elf` are IDENTICAL but for the two
     compile timestamps (`BUILD __TIME__`, `__DATE__ __TIME__`); the `shulib::bench` symbol diff is exactly
     the move (the tester's anonymous `paintLine`/`paintButton`/`paintRefusal`/`paintText` lambda out;
     `clearPanel`, `paintBar`, `paintLine`, `paintButton`, `paintMenuFrame`, `paintMenuText`,
     `paintRefusalScreen`, `lineDiffers` and the two-argument `stopAfterProgramError` in); `.text` +188 bytes.
  4. `src_build_gate.py check`: PASS, all 7 builds; the tank-autontest line reads "exit 0, **11/11** TUs
     compiled, warnings: 7 vendor-ignored, 0 disallowed, hash asserted, beacons (variant, robot) and program
     checked" — unchanged but for the TU count (10 → 11: `src/common/screen_paint.cpp` is a new TU in every
     build, as ND3's ".cpp half" requires); `[include-root] 35 source files` (was 31).
- **Tests added:** `test/guarded_run_test.cpp` — G1 (the chain proof over a watch that is NOT the tester's:
  right chain only, never attached / another guard / Chassis on the watch / on the plant all false, commands
  nothing, one tick), G2 (the stop with no watch: live motion and direct drive, a throwing and a null
  mechanism, an empty span, `noexcept`), G3 (the neutral reduction, and the tester's wrapper agreeing with
  it), G4 (the bits' literal values), G5 (the palette pinned, the refusal budget checks both ways,
  `lineDiffers` on tone and text), G6 (the shared headers PROS-free; the painters' .cpp the PROS half).
  `test/auton_tester_source_test.cpp` gains ONE case ("the tester calls the SHARED painters, chain proof,
  stop and touch reduction"): no palette, painter, `printText`, `screen_erase`, chain proof or touch-field
  read of its own is left — the "never copied" claim made mechanical. Every earlier pin is unchanged.

### 12:05 (2026-09-30) — RESUMED a second time (the second run was STOPPED at the session limit)
- `date` → Wed Sep 30 12:05:55 PM EDT 2026 (the first reading of this run was 12:01:19, at the rebuild).
- **The log above stops at STEP 1 (03:45); the tree does not.** The stopped run kept writing until 04:22 and
  logged none of it. What it left, read from the tree (`git status`, the files' mtimes, the scratch logs), in
  the order it was written: `src/programs/competition_watch.hpp` (03:36), `src/common/auton_run_log.hpp`
  (03:41; the run log's name and boot words, moved out of the tester — not in STEP 1's table), the tester's
  re-pointing to it (03:42), `src/programs/competition_run.hpp` (03:45), `src/common/driver_panel.hpp`
  (03:46) with Teleop's `refreshLcd()` re-pointed to its chooser (03:46) and the claw boot line reworded with
  its three pins (03:47), `src/programs/competition_screen.hpp` (03:50), `sameMenuButtons()` moved into
  `src/common/screen_paint.hpp` (03:52), `src/programs/competition_program.{hpp,cpp}` (03:52-03:57), the
  eighth build in `src/build_selection.hpp`, `src/main.cpp`, the `Makefile` and `tools/src_build_gate.py`
  (03:55-03:56, self-test cases 34/35 written), two V5 builds of `PROGRAM=competition` (03:57, scratch
  `make_comp1.log` / `make_comp2.log`: the first drew a DISALLOWED `-Wformat-truncation` in
  `idleProgramError()` — the program was edited at 03:57:34 and the second build is clean but for the vendor
  warnings), then `test/competition_rig.hpp` (03:59), `test/competition_watch_test.cpp` (04:19) and
  `test/competition_run_test.cpp` (04:21). The suite binary is from 04:22.
- **No gate self-test plant was left** (checked before anything else): `git diff -- Makefile
  src/build_selection.hpp src/main.cpp` holds only the eighth build's intended edits (the `PROGRAM=competition`
  append is present, not case 34's `# (self-test plant: ...)` comment); the gate's other plant targets —
  `src/programs/bench_r3a.cpp`, `test/build_selection_test.cpp`, `test/vendor/doctest.h`,
  `include/liblvgl/core/lv_obj_style.h` — are unmodified, and no untracked include-root plant exists. The
  scratch `backups/` hold only STEP 1's mutation copies, each restored byte-exact by the runner (asserted in
  its `finally`).
- **Measured at resume** (`cmake -S test -B build/test`, briefing regenerated, full build, suite): the build
  was current (the doc gates green); the suite **1,838 cases, 2 FAILED, 3 skipped** — `request (R8)` and
  `request (R10)`, both at `choose(): no button for entry 2/3 on the menu's first page`. **The defect was in
  the TEST RIG, not the program:** those two cases register three throwing autons before the team's two, so
  entries 2-4 are on pages 2-3 (the menu shows two per page, COMP-a's deviation 1), and the rig's `choose()`
  looked at page 1 only. Fixed in `test/competition_rig.hpp`: `choose()` now pages with real NEXT / PREV
  touches, each after the arming delay on a menu clock that only moves forward, then touches the entry — the
  way a person chooses. Re-run: the 27 `request`/`watch` cases green.
- **Kept, not redone:** STEP 1 (verified above by its own records) and everything listed. **Still owed** (read
  from the code's own banners, which cite files that do not exist yet): `test/competition_screen_test.cpp`
  (RC3's owner state machine and the pages), `test/competition_source_test.cpp` (B11's and B30's source pins,
  the PROS reads), `test/driver_panel_test.cpp` (the moved chooser); the mutation campaign for the program; the
  gates; the whole documentation list; `COMP-COMPLETED.md` is the coordinator's at merge.

### 12:10–16:50 — the program reviewed against the brief; four fixes; the tests the banners owed; the gate
*(Clock: `date` read 12:05:55 at the entry above and 16:48:38 at this one; the gate's `check` log closed
15:51:53 and ran 35 min of wall time for 2 min 37 s of CPU — the machine or the session was paused inside
it. No step below is timed more finely than that.)*
- **Read line by line against B2, B5(1), B7-B12, B19, B20, B30, RC1, RC3, RC7, ND1, ND2, ND4, ND5** (every
  library call re-checked in the tree: `AutonMenu` — `touch()` against an EMPTY view consumes the release edge
  and acts on nothing, as the menu phase relies on; `RobotTwoDriver::bind/run/unbind`, `runDriverControl`'s
  `perTick(command, connected, shaperState)`; `AutonRunResult`; `ArmingClock`; the tester's `initialize()` as
  B8's worked example). **Found and FIXED, each with its reason:**
  1. *A refusal arriving while the selector is up left the selector on the screen* (a program error caught
     on the idle loop latched "autons refused", but the refused page was painted only at the next TAKE of the
     screen — the selector kept inviting a choice that would be refused). The menu phase's decision moved
     into a PURE function, `menuPhaseStep(action, refused, menuUp)` in `competition_screen.hpp`, which
     replaces the selector at once, and the program follows it (host-tested, (S7) below).
  2. *The AUTON ENDING page froze a running number* ("%.1f s after driver control started", painted ONCE by
     `sayAutonEnding`, so it read 0.3 s for the rest of a frozen wait). Now "not handed back within 0.25 s" —
     a fact that stays true; the log line carries the time.
  3. *The controller's lines were first written 200 ms into driver control* (`tick_ % 20 == 0`), so an
     AUTON ENDING on the controller outlived the hand-back. Now written on the loop's first tick, then every
     20 (`== 1`).
  4. *The result page's second line overflowed the SMALL budget at the worst case* ("end action FAILED;
     scoring T+9999.99s, 2147483647 paces" — found by (S10), cut silently by the safety copy). The pace count
     left the screen (the log's line 2 keeps it): "end action <word>; scoring ended T+%.2fs".
- **Test defects found and fixed (in the TESTS):** the rig's `choose()` (above); and `faultsBefore =
  -2147483647` in the stopped run's (R13) — `faultsAfter - faultsBefore` then overflowed a signed int, which is
  undefined behaviour, not a worst case (a fault COUNT is never negative): now 0.
- **Kept as built, with the reason (for the deviations list):** a refused boot's `opcontrol()` RETURNS (with
  its B5(1) "normal return" line) instead of idling as C7's older wording says — it commands and paints nothing
  either way, and RC3 says "the phase lines remain"; the program's boot catch is `std::exception`, broader than
  the tester's `PreconditionError` (a `bad_alloc` from a constructor also refuses cleanly).
- **Checked, not changed:** the estimate is NOT ticked while the auton task idles (RC3 (iii)), so on the robot
  its only tick before a match is the boot's chain proof and the next is `runAuton`'s live-wait, possibly
  minutes later. The Localizer treats a dt outside `[minDt, maxDt]` as an unhealthy tick (zero velocity, a
  lower quality — no fault) and `runAuton` re-seats after the live-wait; (R17) proves it on the plant.
- **Tests added:** `test/competition_screen_test.cpp` (S1-S11: the owner — handTo always lands, handOver only
  from its `from`, a refused handOver changes nothing; a tracker paints None unless it holds, Whole on a take
  and once more 0.2 s later; a field match walked end to end with at most one painter per tick and the stale
  end-of-match hand-back refused; AUTON ENDING skipping the result paint; autonomous straight to driver
  control; the owner under 4-thread contention; `menuPhaseStep` over every input; every page for real plant
  records and at the worst case). `test/competition_source_test.cpp` (P1-P13: the program runs autons only
  through `runAutonRequest`; B8's boot order with the binding and the proof inside the ONE try and the latch
  before the task; the two PROS reads one line each; the auton task's body WHOLE; the present mechanisms; the
  phase functions' bodies WHOLE — they log and hand over, paint nothing; `opcontrol()`'s order; every paint
  decided by the owner; the SD log; no motion verb; the claw boot lines; `main.cpp`'s five forwards; no slot
  number; the pure headers PROS-free). `test/driver_panel_test.cpp` (D1-D4: the moved chooser equals
  Teleop's PRE-MOVE expression, restated as an oracle, byte for byte in 6,240 states; AUTON ENDING outranks
  every word; 19 characters at the worst case; the controller row). `test/source_pins.hpp`: COMP-t's pin
  helpers MOVED out of `test/auton_tester_source_test.cpp` (unchanged; that file now includes it) — one copy.
  `competition_run_test.cpp` gains (R17) and (R9) now waits after BOTH of RC1's throws.
- **Suite:** 1,867 cases / 2,701,247 assertions, 3 skipped — **green**.
- **`tools/src_build_gate.py check`: PASS, all 8 builds** — `[tank-competition] exit 0, 12/12 TUs compiled,
  warnings: 8 vendor-ignored, 0 disallowed, hash asserted, beacons (variant, robot) and program checked`;
  `[chain-table] 8 builds, 3 robots`; `[program-table] 7 programs, 19 entry names`; `[include-root] 42 source
  files`. (12 TUs per build: `screen_paint.cpp` and `competition_program.cpp` are compiled into every build.)
- **Next, in order:** the mutation campaign (the program's decisions, RC1's/ND5's listed mutations, the gate
  self-test pair 34/35), then the documentation list, then every gate once more on the final tree.

### the mutation campaign — the program's decisions, RC1's and ND5's listed mutations, the program's pins
*(Timing: harness written after the 16:50 entry; its records carry each mutation's `date` — scratch
`mut_b_results.jsonl`.)* Harness `scratch/comp-b/mut2.py` (COMP-b's second runner): each mutation alone; the
old text must occur exactly once; `cmake --build build/test --target shulib_tests`; it RECORDS whether the build
relinked the suite and, for a mutation of a file the suite compiles, a result without "Linking CXX executable
shulib_tests" is reported NOT OBSERVED (none was); the named cases run; the file restored byte-exact from the
scratch backup (ASSERTED) and its timestamp bumped; a final rebuild of the restored tree. A mutation of
`src/programs/competition_program.cpp` needs no relink: the host cannot compile that TU, and the source pins READ
it at run time. **Before the campaign one gap was closed:** MB10 (the choice read, never LOCKED) would have
survived — nothing observed the lock during a request — so the rig's observer records `menu.locked()` when the
status page is told, and (R1) checks it.

| # | Mutation | File | Red cases |
|---|---|---|---|
| MB01 | **RC1's listed:** FINISHED not published on the catch path | competition_run.hpp | R7, R8, R9 |
| MB02 | **ND5's listed:** the SD write moved before FINISHED (published only after the report) | competition_run.hpp | R11 |
| MB03 | the handler's program-error stop dropped | competition_run.hpp | R7 |
| MB04 | the program error not latched (ND1) | competition_run.hpp | R7, R8, R10 |
| MB05 | the menu never unlocked | competition_run.hpp | R1, R7 |
| MB06 | the watch not armed for the request | competition_run.hpp | R3, R4, R12 |
| MB07 | ND2: a typed-table boot runs the auton | competition_run.hpp | R6 |
| MB08 | ND1: a later auton runs after a program error | competition_run.hpp | R10 |
| MB09 | NONE not skipped | competition_run.hpp | R5 |
| MB10 | the choice read, never LOCKED | competition_run.hpp | R1 |
| MB11 | `isFinished` false for a collapsed request | competition_run.hpp | R14 |
| MB12 | `pending()` serves the OLDEST request | competition_run.hpp | R14 |
| MB13b | the driver loop not ending on a waiting request | competition_run.hpp | R15 |
| MB14 | the driver loop not ending on the autonomous bit | competition_run.hpp | R15 |
| MB15b | AUTON ENDING said every tick, not once | competition_run.hpp | R9 |
| MB16 | opcontrol()'s wait given an upper bound (B2: none) | competition_run.hpp | R9 |
| MB17 | an ignored abandon reads as a pass | competition_run.hpp | R13, S10 |
| MB18 | a std::exception's own words lost | competition_run.hpp | R7, R8 |
| MB19 | the END estimate not read before FINISHED | competition_run.hpp | S8 |
| MB20 | the watch never asks the guard to abandon | competition_watch.hpp | R3, W4, W7 |
| MB21b | **B30:** the request made only on the latch EDGE | competition_watch.hpp | W4 |
| MB22 | the end of the period not a cause | competition_watch.hpp | R4, W1 |
| MB23 | the precedence flipped | competition_watch.hpp | W1 |
| MB24 | `paces()` counts only while armed (the chain proof would refuse a good build) | competition_watch.hpp | W2, W6 |
| MB25 | `arm()` keeps an old latch (the next autonomous() abandoned at T+0) | competition_watch.hpp | R3, W3 |
| MB26 | the watch never paces its inner pacer | competition_watch.hpp | W2, W4, W5, W6, W7 |
| MB27b | `handOver` ignores `from` (a stale hand-back wins) | competition_screen.hpp | S1, S3, S5 |
| MB28 | `handTo` does not move the generation | competition_screen.hpp | S1, S2, S3, S6 |
| MB29 | no second whole paint after a take | competition_screen.hpp | S2 |
| MB30 | a touch acted on during a take | competition_screen.hpp | S7 |
| MB31 | a refusal arriving under the selector leaves it up (the PRE-FIX rule) | competition_screen.hpp | S7 |
| MB32 | another task's screen: the menu phase acts and paints anyway | competition_screen.hpp | S7 |
| MB33c | a line over budget cut SILENTLY | competition_screen.hpp | S12 |
| MB34 | NONE painted in the pass colour | competition_screen.hpp | S8 |
| MB35 | the driver page silent about ND1's refusal | competition_screen.hpp | S11 |
| MB36 | AUTON ENDING below a latched fault | driver_panel.hpp | D2 |
| MB37 | the move swaps Teleop's order (DEGRADED below CLAW ABSENT) | driver_panel.hpp | D1, D2 |
| MB38 | line 2 no longer cut at 19 | driver_panel.hpp | D1, D3 |
| MB39 | the chain proof's refusal made unreachable | competition_program.cpp | P2 |
| MB40 | RC3: the idle waits for ever (`TIMEOUT_MAX`) | competition_program.cpp | P4 |
| MB41 | a PROS phase task paints | competition_program.cpp | P6, P8 |
| MB42 | ND2 forged (`bootFromRecord = true`) | competition_program.cpp | P2 |
| MB43 | the driver loop's end drops the waiting request | competition_program.cpp | P7 |
| MB44 | the result painted without holding the screen | competition_program.cpp | P8 |
| MB45 | the present mechanisms' `++` dropped | competition_program.cpp | P5 |
| MB46 | a PROS read widened in the program (the touch count read here) | competition_program.cpp | P3 |
| MB47 | the fault latch never attached | competition_program.cpp | P2 |
| MB48 | opcontrol() touches the robot before the auton task's word | competition_program.cpp | P7, P10 |

**48 mutations, 48 RED.** Not counted, and why: MB13, MB15, MB27 and MB33 as first written did not COMPILE
(`-Werror`: an unused parameter or a set-but-unused variable) — re-run as MB13b/15b/27b/33b keeping each name
used. **MB21 as first written SURVIVED, and the defect was the MUTATION:** it added an edge request inside the
latch block but left the every-tick request after it, so it removed nothing; MB21b (the edge request only) is
RED. **MB33b SURVIVED, and the defect was a missing TEST:** since fix 4 no page line is over its budget, so
nothing ever exercised the `unsafeLines` counter; (S12) now hands the driver page a row over budget and one
with a '%', and MB33c is RED. Every file restored byte-exact (asserted) and re-timestamped; the final rebuild
of the restored tree is clean. The ND3 move's own seven (MV1-MV7b) are in the STEP 1 entry above.

### the gate's self-test, the first documents, and a stale binary caught — *`date` read Thu Oct 1 01:31 at the campaign's entry and 08:4x here; the session was paused between them*
- **`tools/src_build_gate.py self-test`: "OK (203 detector cases, each against a real build)"** (12 min 34 s;
  scratch `gate_selftest_1.log`). The count is the `expect()` evaluations: the tree has five more than at
  `0f56301` — case 34's four (the `PROGRAM=competition` append dropped ⇒ the FAIL names the define token, the
  tank-competition beacon's absence and the tank beacon's presence) and case 35's one (the real
  tank-competition build passes alone). Every plant target restored: `git diff` of `bench_r3a.cpp`,
  `test/build_selection_test.cpp`, `test/vendor/doctest.h` and `lv_obj_style.h` is empty, and the Makefile,
  `build_selection.hpp` and `main.cpp` carry only the eighth build's intended edits.
- **`src/` comments made true for the tree** (before the self-test, so it ran on the final `src/`):
  `src/team/robot_two_driver.hpp` :5-8 (B11/RC5: "The competition program's opcontrol() runs it …"),
  `src/team/README.md` (the driver file and the autons: "runs", chapter 17 named), `src/common/boot_source.hpp`
  :8-12 ("Two programs need it" was stale since COMP-t: every program that builds a graph, named),
  `src/programs/auton_abort_watch.hpp` :33-35 ("may share" → it shares, the watch's file named). No test pins
  any of these sentences (grepped `test/ tools/ docs/`).
- **Chapter 17's examples:** `test/guide_examples_test.cpp` gains `guide-17a` (a FIELD start), `guide-17b` (a
  schedule past its period refused BY NAME at registration — `check()`'s sentence, `add()` throws) and
  `guide-17c` (the selector: NONE by default, LOCKED by `lockChoice()`), in the file RC11 names (L9's landed
  commit `fa915b0` does not touch it — checked with `git show --stat`, so no merge collision there).
- **A stale binary, caught before it was believed:** the first suite run after the documents read 1,868 cases
  and NO `guide-17` case — the host build had stopped at the doc gates (the briefing's register count went
  stale with HA-162) BEFORE the link, and the suite ran the 01:29 binary. My filter (`error|warning:`) had
  hidden the gates' capitalised `Error 1`. Fixed by the order the brief gives — briefing regenerated, the full
  build (the link line present in its log), the suite — and from here every build is checked for its
  `Linking CXX executable shulib_tests` line. **Fresh binary: 1,871 cases / 2,701,264 assertions, 3 skipped,
  green; `guide-17*` 3 of 3.** The doc example scan: "520 quoted lines, all verbatim (6 source files); 6
  listings of whole-run files, each one unbroken run" (chapter 17's live quote of `autons.cpp` among them).
- **Documents done so far:** guide chapter 17 (NEW, `docs/guide/17-the-competition-program.md`), the guide's
  README (table row; the maintenance note; the quoted-files sentence), `mkdocs.yml` (nav), chapter 16 (three
  sentences made false: "not built yet", "once it is built", "will show"), chapter 7 :123-129 and chapter 14
  :55-63 (RC5; the program and HA-161/HA-162 named), cookbook 06 :268 (RC5), the repo README's `src/` tree,
  `docs/hardware-assumptions.md` (HA-162 NEW, unknown, settled by Station K's first step; HA-161's Source gains
  the competition program, and its Settle is reworded to the program's own lines with RC2's two limits; the
  status line "7 of 161" and one COMP-b clause in the reserved-numbers paragraph — L9's landed commit rewrote
  that paragraph: **the coordinator reconciles the count and the "next free" at merge**).
- **Still to write:** worksheet Station K and its anchors (A.1, the slot map, G.0.5, J.3.1, J.9.6, DO NOT,
  "Before the helper leaves"), HANDOFF (B21/RC10), build-order (RC5's T4 lines, COMP's entry and DoD, B21's
  "minimal auton hook"), roadmap (:353-359, M4), RESUMING and ci.yml (eight builds), the changelog, my lines
  in REMAINING-WORK/ORIENTATION; then every gate on the final tree.

### the documentation pass and the prose review — *greps RUN and hits READ before this entry; `date` above it*
**Written or changed (each re-read against the code it describes):**
- **NEW `docs/guide/17-the-competition-program.md`** (RC11): registering an auton — `autons.cpp`'s
  start/endActionAt/hardStopAt lines quoted LIVE as one unbroken run, linking to chapter 16 for the file; the two
  start modes, the schedule refused by name, the selector — `guide-17a/b/c`, compiled; one task on the robot (the
  auton task, the terminal word, the watch, the one wait an abandon cannot cut, AUTON ENDING after 0.25 s, the
  IMU's 2 s); every phase on the screen and on serial (the log lines quoted from the format strings); the result
  words in the program's own order; refused, not run (the boot, the typed table, a program error — idle or
  served); the Timed Run (HA-128 measured its start ONLY; HA-162); what it does not do yet (no protections until
  T4 Part 2; no route autons; B6). **Corrected while writing, against the register:** a first draft said a Timed
  Run "then runs autonomous, then driver control" — HA-128 measured only that it starts DISABLED; reworded.
- Guide README (table row 17; the maintenance note; the quoted-files sentence), `mkdocs.yml` (nav), chapter 16
  (three false sentences), chapter 7 :123-131 (RC5), chapter 14 :55-63, the glossary (auton task, competition
  program, terminal word, Timed Run, the watch; "run guard" points at ch. 17), cookbook 06 :268 (RC5), the repo
  README (the `src/` tree: `team/`, `programs/`, `common/`), the changelog (a dated "no API change" entry — the
  file's own precedent; HANDOFF L2's "Docs owed" lists the changelog).
- `docs/hardware-assumptions.md`: **HA-162 NEW** (row + entry; unknown; Station K's first step) — the ONLY new
  HA; HA-161's Source gains the competition program (both halves, how (a)'s failure is contained) and its Settle
  is the program's own lines with RC2's two limits; **HA-178's Source corrected** (the ND3 move made
  "`testerTouchFrom()` — the one place PROS's touch fields are read" false: it is `screenTouchFrom()` now).
  Reconciliation direction 2: `competition_program.cpp` now carries `A4 register HA-162` (it carried HA-161).
- The worksheet: **Station K NEW** (after M; K.0 the preconditions incl. F/J/M on the same build line and the claw
  left OPEN; K.1 slot 6, never 5; K.2 the brain start — boot lines, claw lines, phase lines, the driver page;
  **K.3 the Timed Run measured FIRST (HA-162)** with B5(4)'s Match fallback and B6's stop; K.4 B28's claw step;
  K.5 the practice match anchored on M.6's measured result, the host-test-plant numbers marked, TSTR and the
  `xt` column as `diag/motion_result.hpp` prints them; K.6 the mid-auton disable written from K.3's measurement
  and HA-161's settle with both limits; K.7 the SD log); its anchors: Station A's program list, the slot map,
  **A.1's one exception**, G.0.5, **J.3.1 re-quoted** (RC5), **J.9.6 — stays forbidden for slot 2** (RC9), Station
  M's preface, the DO NOT list's exception, "Before the helper leaves".
- HANDOFF (B21/RC10: the program list "eight builds" + shulib Competition; L2's status, "after G1 Part 2b" and
  "Left / Right / Skills / None" corrected, the `opcontrol()` line → the driver file, "state screens" → the
  program's own menu, "a seventh build" → the eighth; T4 1a's "no program runs it yet"; the shared chooser's
  path for SAFE-SAVE B2). **RC10's :617 sentence was already true** — COMP-t's merge reworded it ("whether its
  floor runs close R3b is a ruling against R3b's DoD … not claimed here"; S8 closes R3b and M1) — left as is.
  **"or a minimal auton hook":** the only hit left is build-order :640, inside the block kept as history — left
  alone (RC10).
- build-order (RC5: the one phrase in the Next block "NO PROGRAM RUNS IT YET"; COMP's description, "why here",
  and a COMP-b BUILT status line — the DoD :1507 was already corrected at COMP-t's merge), roadmap (RC5's
  :356-360; the COMP-a parenthetical; **M4 Sequencing: a `[~]` COMP-b row — "the guaranteed park now reaches a
  real program", UNRUN**; T4's driver-program row), RESUMING and `ci.yml` (eight builds, `PROGRAM=competition`),
  REMAINING-WORK (row 5's state), ORIENTATION (eight programs; never run).
- `src/` comments (above): the driver file's banner (B11/RC5), `src/team/README.md`, `boot_source.hpp`,
  `auton_abort_watch.hpp`; `teleop_claws.hpp`'s claw line and `kLcdClawAbsent`'s note (the stopped run, kept).
**The greps (run; hits read; each hit fixed above or judged true):**
- `competition program.{0,40}(not built|will |once it|is the next|yet)|not built yet|seven builds|seventh
  build|minimal auton hook|no program runs|NO PROGRAM RUNS|PROGRAM=competition|COMP-b` over README, docs/*.md,
  guide, cookbook, ci.yml, RESUMING, build-order, HANDOFF, REMAINING-WORK, ORIENTATION, PROJECT-BRIEFING — every
  hit read; fixed: guide 16 ×3, guide 07, cookbook 06, README, RESUMING, ci.yml ×2, roadmap :359/:382/:1767,
  build-order :632, HANDOFF :270/:348-367/:744, ORIENTATION :104, REMAINING-WORK :33; judged true or history:
  `docs/README.md:38` and `vexbuilder-integration.md:4` (other subjects), changelog :224/:289 (COMP-a's dated
  section, "written when it was built"), build-order :640 (history), :1245/:1954/:2033-2061 (other chunks'
  rows), REMAINING-WORK's other rows (other chunks', or decisions already answered).
- `Teleop only|pacerChainProven|stopAfterProgramError|testerTouchFrom|kStatusDisabled|TesterSdLog|refreshLcd|
  controllerScreenLines|sameButtons|paintRefusal|programs share` over README and docs — fixed: HA-178's Source,
  README's `common/`; the R3b/R3d chunk records' hits are history of their own trees.
- `competition_program|competition_run|competition_watch|competition_screen|driver_panel|guarded_run|
  screen_paint|screen_touch|auton_run_log|state_screens|slot 6 … reserved` — every hit is mine or still true
  (HA-128's Source names `state_screens` — still true).
- `No program runs this file|will run it|Two programs need it|it may share the layering|program will run the one`
  over test/, tools/, docs/ — no test pins any sentence I changed; cookbook 06 :268 fixed (above).
**Deliberately NOT touched, with why:** `docs/api/` (no `///` comment under `include/shulib/` changed —
`check-fresh` exit 0); the G1 brief :94 (ND6's note is for an overrun — `main.cpp` is 298, under 300); the
build-order `Next:` pointer and the roadmap's "You are here" (the coordinator moves them at merge — six chunks
merge in parallel); the T4 brief (not mine; its "At 1a: No program runs this file yet" is a quote of the file AS
IT WAS at 1a); `COMP-COMPLETED.md` is written below, from this log. *(Corrected after a re-check, the same
hour: this entry first said guide chapters 11 and 12 were deliberately not touched, "nothing in them made false".
The grep was then RUN on them (`competition|autonomous()|abandon|auton task`): chapter 11's "competition builds
route diagnostics to a NullSink" and chapter 12's "`n/a` … expected in competition builds" describe a
diagnostics-OFF build, and a reader would now take them to mean "shulib Competition", which logs everything to
serial — the provider's sink is a `TermSink` on USB (`hal/pros/device_provider.hpp` :373-376), and records flow
in a `robot::Robot` build (the path Station M's TSTR lines rely on). One clarifying sentence each, linking
chapter 17. Chapter 12's ABANDONED triage lines (:192-195) are COMP-a's and still true. L9 also edited both
chapters, on other lines.)*

### Deviations from the brief, each with its reason (Part b)
1. **ND3's list WIDENED (B11's "never copied"):** beyond the chain proof, the stop, the touch reduction and the
   refusal painter, the AUTON SELECTOR's painter (the tester's PICK page — `paintMenuFrame`/`paintMenuText`,
   `sameMenuButtons`), the tone palette, the status bits (`state_screens.hpp`, checked against PROS in its .cpp)
   and the SD run log's name and boot words (`auton_run_log.hpp`) moved too — the competition program paints the
   same selector and appends to the same file; leaving them would have made near-copies (ND3 (c), rejected).
   The chain proof is a template over a `WatchLike` concept (`paces()` AND `attachedTo()` — COMP-t's review's
   term, which ND3's "templated over any watch with paces()" predates).
2. **The controller chooser MOVED out of Teleop's `teleop_program.cpp`** (B11: "The controller screen's ONE
   chooser moves here … never copied") — ND3's "Teleop's pinned file is not touched" is about the REFUSAL
   painter, which stays. The pin `test/teleop_claws_test.cpp` (1l) was re-pointed WITH THIS LOG: it pinned the
   chooser's expression in Teleop; it now pins each fact Teleop hands the shared chooser and the one call, and
   `test/driver_panel_test.cpp` (D1) proves the moved chooser equals Teleop's pre-move expression byte for byte
   in 6,240 states. The claw boot line's three word pins (:487, :490, :789) changed with the reword (RC5, T4
   landmine 1), and J.3.1 was re-quoted.
3. **A refused boot's `opcontrol()` RETURNS** (after its B5(1) "normal return" line) rather than idling (C7's
   older word): it commands and paints nothing either way, and RC3 says the phase lines remain.
4. **The boot's catch is `std::exception`**, broader than the tester's `PreconditionError` (a `bad_alloc` or a
   registry throw refuses the boot cleanly, by its own words).
5. **Nothing paints during a run** (RC3 left "a painter in the pacer chain as COMP-t does" to the builder): no
   painter is in the chain; the status page is painted before `runAuton`, the result after the terminal word.
6. **A throw caught on the auton task's IDLE half** (the menu, the status log) also latches the program error —
   later autons refused (ND1's reason holds: the program's state is unproven) — and commands nothing (the robot
   was not the auton task's). Logged once.
7. **The driver loop's end is `driverControlMustEnd(status, handshake.busy())`** — a disable, OR the AUTONOMOUS
   bit, OR a waiting request — not the disable alone (B11's `disabled`): the auton task must never share the
   robot with the driver loop even if HA-161 (a) is wrong and PROS does not delete `opcontrol()` at a direct
   change to autonomous.
8. **Requests COLLAPSE:** two `autonomous()` calls during one serve run ONE auton (the newest request), and
   `isFinished(n)` is true for a request that collapsed into a later one (R14).
9. **The driver page has an AUTON row** (the last request's word, or why autons are refused) beside B11's
   header, controller, fault and claw rows — ND1's and ND2's "the screen … says why" during driver control.
10. **AUTON ENDING tops the controller's chooser** (D2): a state in force outranks a latched history.
11. **In a field match the result page shows only until the field's disable:** `disabled()` hands the screen to
    the menu (RC3's assignment); the log keeps the result. Said in chapter 17.
12. **Four of the stopped run's choices FIXED at review** (above): `menuPhaseStep` (a refusal under the
    selector), the AUTON ENDING page's frozen number, the controller's first-tick refresh, the pace count off the
    result screen. Test defects fixed: the rig's paging, the signed overflow in (R13).
13. **The test helpers MOVED to `test/source_pins.hpp`** (one copy) — the tester's source test includes it;
    its pins are unchanged.
14. **The changelog gained a dated "no API change" entry** (not on the brief's list; HANDOFF L2's "Docs owed"
    has the changelog, and the file has the precedent).
15. **`main.cpp` 280 → 298 lines** — inside ND6's 300; no overrun, so no note in the G1 brief.

### For the coordinator (Part b)
- **Merge conflicts to expect, all on shared documents** (my lines only): `docs/hardware-assumptions.md` — L9's
  `fa915b0` rewrote the reserved-numbers paragraph and moved "next free" to HA-187; mine adds HA-162's row and
  entry, edits HA-161's Source/Settle and HA-178's Source, sets the status to "7 of 161" and adds one clause:
  **reconcile the count and the reserved list** (HA-162 is no longer reserved — it is an entry). The changelog
  (my entry sits under "## API 2.3"; L9 added API 2.4 — place it by date). HANDOFF, build-order, roadmap,
  REMAINING-WORK, ORIENTATION (L9 and others edited them). `docs/guide/11`, `12` (L9 edited other lines).
- **Files another in-flight chunk may own, touched here:** `src/programs/teleop_program.cpp` (the chooser call,
  B11 — T4's Part 3 retires the file; T4 Part 2a is the library half and should not touch it — if it did, its
  LCD words go into `src/common/driver_panel.hpp`'s chooser, the T4 brief's D4); `src/programs/teleop_claws.hpp`
  (the claw boot line, RC5; T4 Part 3 moves the claw rows); `src/team/robot_two_driver.hpp` (its banner comment
  only — T4's file); `src/common/boot_source.hpp` (its banner comment only — SAFE-SAVE B2 edits the boot
  later); `test/guide_examples_test.cpp` (appended `guide-17a/b/c`); `test/teleop_claws_test.cpp` (pins).
- **T4 Part 2a:** if its protections land before this merges, `opcontrol()` gets them through
  `runDriverControl` with no wiring here, and these sentences become false: chapter 17's "What it does not do
  yet" first bullet, Station K's preface ("Driver control here has NO drive protections yet"), the roadmap's
  COMP-b row ("driver control has no drive protections until T4 Part 2"). Written for the tree I built on (B11).
- **Pointers not moved:** build-order's `Next:` and the roadmap's "You are here" (yours at merge). The
  briefing's `Next up` derives from them.
- **Not verifiable here:** everything on a robot — Station K is the program's first run.

### the final gates — every one on the final tree, in the brief's order (`date` read Thu Oct 1 09:34:11 as this entry was written)
- **Host:** briefing regenerated → `cmake -S test -B build/test` → the full build (log line `Linking CXX executable
  shulib_tests` PRESENT; the doc gates inside it green) → **1,871 cases / 2,701,264 assertions, 3 skipped — SUCCESS**
  (start of Part b: 1,808 / 2,652,625; +63 cases) → briefing regenerated again → `briefing_status.py check`:
  "generated block current". (One earlier run of the suite after the self-test showed the same counts; the final
  one above is on a FRESH link, because the self-test had re-timestamped the restored test files.)
- **Doc gates, run explicitly:** `api_doc_tool.py check-fresh` exit 0; `check-examples` "520 quoted lines, all
  verbatim (6 source files); 6 listings of whole-run files, each one unbroken run"; `self-test` OK;
  `check-coverage` "2956 public entities across 157 headers, all documented"; `check-removability` "no public doc
  references docs/internal/"; `doc_staleness_audit.py self-test` OK (10 cases) and the audit "clean (29 live
  docs …)"; `briefing_status.py self-test` "OK (17 cases …)". The relative links of every changed public doc and of
  `COMP-COMPLETED.md` resolve (a scratch link check: 0 missing).
- **CI greps:** "library is PROS-free (all of include/shulib except hal/pros/)"; "layering holds: core is
  sim-free".
- **ARM header gate** (RESUMING's command; the TU written to the scratch directory, never `/tmp`): 193 headers,
  exit 0.
- **`tools/src_build_gate.py check`: "src build gate: PASS (all 8 builds link; policy: …)"** —
  `[tank-competition] exit 0, 12/12 TUs compiled, warnings: 8 vendor-ignored, 0 disallowed, hash asserted
  (v0.1.1-356-g0f56301-dirty), beacons (variant, robot) and program checked`, the other seven the same;
  `[one-chain]` …; `[chain-table] 8 builds, 3 robots, matching src/build_selection.hpp's chain`; `[program-table] 7
  programs, 19 entry names, matching src/main.cpp`; `[include-root] 42 source files in src/ and 4 folders`;
  `[printed-path]` clean. (3 min 59 s.)
- **`tools/src_build_gate.py self-test`: "OK (203 detector cases, each against a real build)"** (17 min 2 s) —
  afterwards every plant target verified restored (`git diff` of `lv_obj_style.h`, `bench_r3a.cpp`,
  `test/build_selection_test.cpp`, `test/vendor/doctest.h` empty; the Makefile carries the real append and no
  plant comment).
- `src/main.cpp`: 280 → **298** lines (ND6's target 300 held); G1 item 5's grep (`MotorGroup|Localizer|
  RobotContext|MotionDeps|screen_print`) over it: 0 hits.
- `COMP-COMPLETED.md` written from this log (the brief's §3 item 4). **It closes COMP in the briefing's list
  (34 of 63; COMP no longer "open")** — that is what a COMP record means, since COMP-b is COMP's last part; the
  coordinator confirms it at merge, or renames it to a part record (`COMP-b-COMPLETED.md`) and regenerates.

**Tree left uncommitted, as instructed. Nothing in Part b has run on a robot — Station K is its first run.**

### COMP-b review findings and their disposition (the review round, same worktree, same builder) — *`date` read Thu Oct 1 10:35:58 at its start*
- **Three reviewers** (correctness, tests and gates, documents) returned 29 findings through the coordinator's
  workflow run; two pairs overlap (the driver loop's last tick: correctness #2 and tests #7; the ND3 proof's
  tree: correctness #6 and documents #10), so **27 distinct**. Each was re-checked in the tree before anything
  changed; the disposition of each is below, in the reviewers' order. Their probes and mutation specs live in
  `scratch/comp-b/review-correctness/`, `review-testgate/` and `review-docs/` (read, never run in the worktree);
  this round's own scratch is `scratch/comp-b/r2/`.
- **Baseline, measured before any edit:** no file in the worktree is newer than the 09:33 suite binary but this
  log and `COMP-COMPLETED.md` (`find -newer`); `cmake -S test -B build/test`, the full build (current — no
  relink), the suite: **1,871 cases / 2,701,264 assertions, 3 skipped — SUCCESS** (the last entry's numbers).
- **RESUMED 15:13 (`date`: Thu Oct 1 03:13:08 PM EDT 2026).** The round's first run stopped after the baseline above
  (the coordinator paused it for budget). Measured at resume: no file in the worktree is newer than the 09:33 suite
  binary but this log and `COMP-COMPLETED.md`; the scratch `r2/` holds only the baseline's cmake/build logs. Nothing
  of the round had been changed, so it starts here, from the findings, each re-read in the tree.
- **RESUMED a second time, 18:15 (`date`: Thu Oct 1 06:15:21 PM EDT 2026; the round's second run was STOPPED
  partway).** What it left, measured before anything else: the log stops at the 15:13 bullet above (last written
  15:36), but the tree does not — `src/` and `test/` edits from 15:36 to 16:52 and chapter 17 at 16:53, none of them
  logged. Its mutation campaign (`r2/mut3.py`, the Part b runner's logic) ran on a SCRATCH COPY of the worktree
  (`r2/mut_tree`, rsynced at 16:47) and FINISHED ("final rebuild ok", "runner exit 0"; 54 records in
  `r2/mut_r2_results.jsonl`). **No mutation and no plant is left:** each of the 54 backups in `r2/backups/` is
  byte-identical (`cmp`) to the worktree's file of that name; the copy differs from the worktree only in
  `test/competition_run_test.cpp` (R24, added at 16:52, after the copy) and chapter 17; the gate's plant targets
  are clean (`git diff` of `bench_r3a.cpp`, `test/vendor/doctest.h`, `lv_obj_style.h` empty; the Makefile carries
  no plant comment). **Relinked from source before any result was trusted:** briefing regenerated, `cmake -S test
  -B build/test`, the full build (`Linking CXX executable shulib_tests` present), the suite: **1,885 cases /
  2,701,678 assertions, 3 skipped — green** (R24 included).
- **What the stopped run changed, reconstructed** by diffing the reviewers' copy of the reviewed tree
  (`scratch/comp-b/review-docs/tree`, 09:58 — identical to the worktree at the review but for this log) against
  the worktree, file by file, and re-read line by line by me before it was kept. It is credited below, finding
  by finding; what THIS run added is marked *(this run)*. Nothing of it was reverted.

**The dispositions** (the reviewers' order; "C" correctness, "T" tests and gates, "D" documents):
- **C1 — the stop and the reads before FINISHED, unordered by any test: REAL, FIXED.** (R18) — a probe mechanism
  records `handshake.finished()` at every safe-state application: on both of RC1's throws and at a clean end every
  one comes before the word; (P15) pins `serveAutonRequest`'s tail (the stop, the END-estimate and fault reads,
  the unlock, `finish()` LAST) — a single-threaded host cannot watch the reads' order. The reviewer's RV2/RV3 and
  a harder RV3d (a SECOND stop after the word) are RED.
- **C2 = T7 — the driver loop could touch the robot for a tick after the auton task started (HA-161 (a) false):
  REAL, the window CLOSED (not reworded).** `DriverLoopPresence` (`competition_run.hpp`): `opcontrol()` CLAIMS the
  robot before the driver file's loop (`enter()` — the claim stored, then the request read, both sequentially
  consistent; refused while a request waits, and then it waits again), beats once per tick from the loop's end
  predicate, and `leave()`s after the loop — after its exit's `cancel()` and `makeSafe()`. The auton task, before
  it serves a request, `waitForDriverLoop()`s until no loop holds the robot or its heartbeat has stood still for
  `kDriverLoopStaleSeconds` = 0.1 s (REASONED: ten ticks; a loop PROS deleted mid-tick never leaves). The
  handshake's request/busy atomics became seq_cst (the other half of the store-then-load pair). When it waited,
  the request gets a fifth line ("a driver loop still held the robot … it LEFT / its heartbeat STOPPED"), which
  is also new evidence for HA-161 (a). (R21) models the review's probe PC through the same functions (the driver
  task half a tick out of step, still ticking during the run): FINISHED, the loop's exit before anything moved; a
  deleted loop is let go after 0.1 s; a loop dead long before costs nothing; `enter()` refuses while a request
  waits. The one margin left (a driver tick > 0.1 s with a request waiting) is in the register.
- **C3 — AUTON ENDING could stay on the controller (a three-line burst): REAL, FIXED at the root, for BOTH
  programs.** `ControllerLineWriter` (`src/common/driver_panel.hpp`): one line per write, at most one per 0.1 s
  (twice PROS's documented VEXnet figure), line 2 first whenever it changed, lines 0/1 in turn; `wrote()` keeps
  the gap after AUTON ENDING's direct write; `forget()` at each driver period. The competition program reads its
  facts every 5 ticks (was 20); **"shulib Teleop" writes through the same writer** (its burst was the same defect;
  B11's "never copied") — a Teleop behaviour change, logged as a deviation, its pin (1l) extended. (D5) tests it.
- **C4 — a request served after the period re-seated and commanded a tick: REAL, FIXED.** `CompetitionWatch::
  latchNow()` reads the status ONCE before anything moves; a cause already present skips the run as the new
  `CompetitionSkip::TooLate`, word **TOO LATE** (amber), two lines, nothing commanded, no re-seat, the guard never
  armed. (R20), (W8), (S14).
- **C5 — one bad auton entry refuses the whole boot: NOT CHANGED — the team lead's decision, raised.** It is the
  brief's B15/B18 rule, documented (chapter 17, `autons.hpp`); option one (registration that does not throw, the
  refused entry named and left off the menu, driver control kept — as ND2 treats a typed-table boot) is a change
  to a ruling, so it waits for his word (below, "For the coordinator").
- **C6 = D10 — the ND3 proof was taken on an in-between tree: REAL, re-measured on the FINAL tree *(this run)*.**
  `make ROBOT=tank PROGRAM=autontest` from a copy of the final worktree (`r2/pkg_final`), hash-less, `__DATE__`/
  `__TIME__` pinned (the reviewer's flags), against the reviewer's `git archive 0f56301` build: `.rodata`'s 766
  printable strings (`strings -a -n 4`, unique) are identical but for the PROS link timestamp and two 4-byte
  non-text runs (`r2/c3_autontest_rodata.diff`); function by function, addresses resolved
  (`r2/cmp_disasm.py`, the reviewer's method), the differences are exactly the move — the tester's own painters
  out, the shared ones in, the call sites of `ScreenPacer::pace`, `readTouch`, `refuse`, `stopTheTester`,
  `sameButtons` and `autonTesterInitialize` (its SD-log word chosen through `autonRunLogWords`, the same strings),
  and two linker veneers (`r2/c3_cmp_autontest_final.txt`); against the reviewer's build of the reviewed tree only
  `__printf_veneer` differs. STEP 1's 717-string figure stays as what it measured (the 03:35 snapshot);
  COMP-COMPLETED now cites this re-run.
- **C7 — a throw in a run's report left the refused page with no reason and a wrong headline: REAL, FIXED.**
  `reportAutonRun` writes its reason ("the report of request N threw: … -- the robot was not touched") and returns
  true; `runAutonRequest` writes a throw's ("request N threw: … -- the robot was stopped"); the idle half writes
  "the auton task threw outside a request: … -- the robot was not touched"; the headline is "AUTONS REFUSED: a
  program error on the auton task". (R22). *(This run)*, the same misstatement one level down: the request's
  refusal line and the page's line said "a program error stopped an earlier auton" — false after an idle or a
  report throw; now "a program error earlier on this boot (its own Error line says where)" / "a program error
  earlier on this boot", tests (R6b), (R10), (S-pages) updated.
- **C8 — the comment says --gc-sections drops the program, but its strings ride in other packages: REAL, the
  comments CORRECTED** (`competition_program.cpp`, `teleop_program.cpp`, `auton_tester_program.cpp`): the CODE is
  dropped; a file's string literals can ride where the linker keeps that file's copy of a shared inline function —
  words only, never code that runs; no beacon among them.
- **T1 — the program file's ND1/ND2/ND5/deviation-6 code unpinned (7 survivors): REAL, FIXED.** (P14) pins
  `logLine`, `logPhase`, `logStatusChange`, `idleProgramError`, `paintMenuPhase`, `paintAutonsRefused`,
  `DriverPanel`'s operator, view and `refreshLcd`, and `sayAutonEnding` WHOLE; (P2) gains `bootFromRecord` set
  before the task is created; ND1/ND2's decision became the pure `autonsRefused(flags)`, host-tested. The source
  test's header now says what is pinned whole and what by order. RV19, RV20, RV23, RV25, RV27, RV32, RV33 RED.
- **T2 — ND2's only test used an unreachable state: REAL, FIXED.** (R6b): a typed-table boot with NO choice gives
  BootFromTable and its lines; a program error with no choice gives AfterProgramError. RV04, RV13 RED.
- **T3 — "the status page before runAuton" untested: REAL, FIXED.** The rig's observer records the plant's
  distance, `guard.running()` and `watch.armed()` when told; (R1) checks nothing had moved or armed. RV01 RED.
- **T4 — "only this run's faults" untested: REAL, FIXED.** (R19) ports the tester's (3c): a fault before the
  request is not this run's (line 3 and the page say none); one during it is counted once, by name. RV02, RV11,
  RV12, RV15 RED.
- **T5 — log lines, levels, words, tones unpinned (10 survivors): REAL, FIXED.** (R23): NONE-with-a-throw is
  PROGRAM ERROR, NOT LIVE, every word's level, `competitionStopWord`, the boot lines whole at a real hash, a phase
  line whole; (R1) line 4's END estimate; (S13) every word's tone and the AUTON row's precedence. RV03, RV05-RV10,
  RV14, RV16 RED.
- **T6 — the program's log and controller functions unpinned, fix 3 included (8 survivors): REAL, FIXED** by
  (P14) and (P7) (opcontrol() whole: wait, claim, entry and return lines, leave, hand back). RV17, RV18, RV21,
  RV22, RV24, RV26, RV28, RV29 RED.
- **D1 — K.6.2 expected a result page the program never paints after a mid-auton change: REAL, FIXED *(this run)*.**
  K.6.2 now reads the result on serial (the run's first `[CMP]` line) and the driver page's auton row; the brain
  shows the menu after a disable; ABANDON IGNORED is a serial STOP; "What to send back" item 5 rewritten. Chapter
  17 gained "An auton ended early is read on serial" (the stopped run).
- **D2 — HA-161's AMBIGUOUS window anchored on a line logged only after the run: REAL, FIXED — the anchor moved
  *(this run)*.** The change's mark is now the NEXT phase function's `entry` line (PROS starts that function at
  the change; it logs that line first): a `normal return` after it refutes (a); one before it is AMBIGUOUS (the
  abandon finished before PROS acted); never confirms (a). (b) is read from the run's `request N:` lines appearing
  at all (the auton task writes them only after the run). Register (HA-161 Settle and Source), K.6.3 and chapter 17
  (the stopped run: the status-change line's real timing) agree. **RC2's stop condition does not fire:** with this
  anchor the lines DO tell the cases apart — but it departs from RC2's literal "within a tick of the status-change
  line", so it is raised for the coordinator below.
- **D3 — build-order's T4 line (brief :1479, today :1484) and HANDOFF L5b still said "no program runs the driver
  file": REAL, FIXED *(this run)*,** both reworded as :632 was (and Part 1b's fold into Part 3 stated). The prose
  grep's hit list in the documentation entry above missed :1484 — that list was incomplete; it is not rewritten
  (history), this line corrects it.
- **D4 — chapter 16 and Station M's preface: REAL, FIXED *(this run)*.** Chapter 16: neither program passes a
  command registry, so a route auton is refused by name in both. Station M: "makes", HA-162 (HA-128 measured only
  the start), the route auton is neither station's.
- **D5 — HANDOFF L2 and ORIENTATION: REAL, FIXED *(this run)*.** L2: routes refused until commands are registered;
  the wiring problem SOLVED at COMP-a (`BuildOptions::pacer`); "no program can command one today" → two can, none
  has run; "will make" → "makes". ORIENTATION item 2: the match program BUILT, not run (Station K).
- **D6 — RC3's refused-boot sentence: REAL, FIXED** — chapter 17 (the stopped run) and K.2.2 *(this run)*: no
  `competition status` line at all after a refused boot, only the four phase functions' two lines.
- **D7 — Station K lacked B17's stopped-at record and RC9's thresholds: REAL, FIXED.** (R24) *(the stopped run)*
  reproduces COMP-t's table through THIS program's request (the 40 in at 9 %: FINISHED T+1.89 s, END (39.64, 0.02,
  0.10°); the acceptance run FINISHED T+6.14 s, END (0.37, 0.06, 177.28°); ×0.723/×0.722 and ×0.724/×0.723, a stop
  read as line 3's `stopped at … (motion TIMEOUT)`). K.5.2 cites them as host-test-plant numbers and says what an
  `ODO_STUCK` abort prints (a fault abort ends the motion CANCELLED — `motion_scheduler.hpp`'s fault policy); K.5.3's
  table gains the line-3 row *(this run)*.
- **D8 — B5(4)'s "first step" reordered without a log entry; HA-162 pointed at the wrong step: REAL, FIXED *(this
  run)*.** Deviation 22 below; K.3 says why K.2 precedes it; HA-162's row and Settle name K.3.
- **D9 — HA-178's Source and two comments overclaimed the shared touch read; HA-162 missing from the UNKNOWN list:
  REAL, FIXED** — the comments (the stopped run) and the register *(this run)*: "for the Auton Tester and the
  competition program; shulib Calibrate and the bench tests read the status themselves"; HA-162 added to the
  register header's UNKNOWN parenthetical (the coordinator reconciles that header with `fa915b0` at merge).
- **D10** — with C6. Also: **deviation 13's "its pins are unchanged" was not exact** — one pin of
  `test/auton_tester_source_test.cpp` was REWRITTEN at STEP 1: `kSdRunLogFile = "shulib_auton_runs.txt"` became
  `kSdRunLogFile = kAutonRunLogFile` plus a pin that `src/common/auton_run_log.hpp` holds `kAutonRunLogFile =
  "shulib_auton_runs.txt"` — the same file name, pinned through the move. Logged here as ND3 requires.
- **D11 — `screen_paint.hpp`'s banner said Teleop's file is not touched: REAL, FIXED** (the stopped run): Teleop
  keeps its own refusal painter; only its controller lines moved (the chooser, and since this review the writer).
- **D12 — stale "seven builds" in `src/`, `test/` comments; `boot_source.hpp`'s overclaim: REAL, FIXED** (the
  stopped run: `teleop_program.cpp` ×3, `auton_tester_program.cpp`, `fault_latch_hook.hpp` ×2,
  `xdrive_program.cpp` ×3, `boot_source.cpp`, `test/build_selection_test.cpp` "other seven branches";
  `boot_source.hpp`: "every program that boots robot two's graph"; `main.cpp`'s handler comment no longer lists
  two programs). *(This run)* re-grepped `src test tools Makefile .github README RESUMING` for `seven builds|all
  seven|the seven|other (five|six)|six builds`: the hits left are history (`src_build_gate.py`'s dated case notes)
  or other subjects.
- **D13 — glossary out of order: REAL, FIXED *(this run)*** — `auton task` before `auton tester`, `terminal word`
  after `telemetry sink`, `Timed Run` after `tier` (letter by letter, the file's own convention: `autonomous`
  before `auton…`); the lines moved, none changed.
- **D14 — the master plan and the diagnostics plan still said a competition build routes to NullSink: REAL,
  FIXED *(this run)*** — the same one-line clarification as chapters 11 and 12 in both plans (a diagnostics-off
  build; "shulib Competition" is not one today).

**Also *(this run)*, found while re-reading the fixes:** chapter 17 said a full controller refresh takes up to
0.3 s for "both programs" — true here (facts every 50 ms); Teleop reads its facts every 0.2 s, so about 0.4 s
there — corrected. The changelog's COMP-b entry gained the review's three additions (the driver loop leaves
first, TOO LATE, the controller writer — Teleop included). The worksheet's Teleop controller paragraph says the
rows now appear one at a time.

**Mutations — this round:** the stopped run's campaign on the scratch copy, 54 records (`r2/mut_r2_results.jsonl`):
**every one of the reviewers' 31 survivors re-run on the fixed tree is RED** (RV01-RV29, RV32, RV33 — each by the
case the finding names: R1, R6b, R19, R20, R22, R23, S13, P2, P7, P14), the correctness reviewer's two as RV2c
(P15) and RV3c/RV3d (R18, R7) RED, and the fixes' own: NB01 (a late request run anyway: R20, S14), NB02/NB03
(`latchNow` keeps an old latch / arms the watch: W8), NB04 (TOO LATE reads REFUSED), NB05 (the auton task serves
while the loop is in: R21), NB07 (`enter()` claims while a request waits: R21), NB08-NB10 (the loop never leaves /
never beats / the auton task skips the wait: P7, P4), NB11-NB13 (the burst; line 2 not first; line 1 starves line
0: D5), NB15 (Teleop's burst back: 1l), NB17 (a request's throw not recorded: R22), NB18 (`autonsRefused` drops
the typed table: R6b), NB19 (the loop's line never said: R13, R21), NB20 (a line's seconds unbounded: R13) — all
RED. Three of its forms did not COMPILE (`-Werror`, a name left unused: NB06, NB14, NB16) — *(this run)* re-run as
**NB06b** (a deleted loop held ten times too long — NB06's "for ever" would also have hung the run), **NB14b** and
**NB16b**, each on a fresh link of a re-synced copy (`r2/c3_mut_results.jsonl`): **all three RED** (R21; D5; R16
and R22). Every file restored byte-exact (asserted) in the copy; the copy re-checked identical to the worktree.
**54 valid mutations this round (34 of the reviewers', 20 on the fixes), 54 RED.**

**Deviations added this round** (numbering continues the Part b list):
16. **The driver loop CLAIMS and LEAVES; the auton task waits for it** (C2/T7) — beyond B11/deviation 7's end
    condition. `kDriverLoopStaleSeconds` = 0.1 s is REASONED, not measured.
17. **TOO LATE** — a fifth skip and a new result word (C4); the status is read once per request before anything
    moves.
18. **The controller WRITER is shared, and Teleop writes through it** (C3) — a behaviour change in "shulib Teleop"
    (one line per refresh, line 2 first), outside COMP-b's program but the same defect in the same shared lines
    (B11). The competition program's facts are read every 5 ticks (was 20).
19. **The program-error reasons and words** (C7): the report's and the idle half's own reasons, the headline, and
    the request's refusal line no longer say an auton was stopped.
20. **`autonsRefused(flags)`** — ND1/ND2's menu decision moved into a pure, host-tested function (T1).
21. **HA-161's settle anchored on the next phase function's `entry` line**, not RC2's status-change line (D2).
22. **Station K's brain start (K.2) comes before the Timed Run (K.3)** — B5(4) made the Timed Run the FIRST step;
    K.2 only reads the boot and leaves the claw OPEN for K.4, which shares K.3's start (B28), so K.3 is the first
    MATCH step (D8).

**For the coordinator (this round):**
- **C5, the team lead's to decide:** one auton entry the registry refuses (e.g. a `hardStopAt` past the period)
  refuses the whole BOOT — driver control included — under B15/B18 as built. ND1's reason ("an auton bug must not
  cost the match's driver period") and ND2 ("refuse the auton, not the boot") point the other way. Option one:
  register through a check that does not throw, name each refused entry, keep it off the menu, keep driver control
  (a small change in the program and its tests; the students' file is untouched). Option two: keep the rule and
  say so beside ND1. Not changed here.
- **D2 / RC2:** the settle's anchor moved (deviation 21); confirm, or rule otherwise before Station K runs.
- **New HA evidence, no new HA:** the request's fifth line (a driver loop that LEFT) is evidence for HA-161 (a)'s
  `opcontrol()` half; it is in HA-161's Settle, not a new entry (HA-162 stays the only new one).
- **Merge notes:** `src/common/driver_panel.hpp` (the writer) and `src/programs/teleop_program.cpp` (its
  `refreshLcd`) changed again; if T4 Part 2a touched Teleop's LCD, its words go into the chooser and its writes
  through the writer. The changelog paragraph and the register's HA-161/HA-162/HA-178 and header edits sit on
  shared lines: reconcile them with `fa915b0` and the other in-flight chunks at merge.
- **Found while grepping, NOT fixed here (other programs' code):** "shulib Drive" (`src/programs/drive_program.cpp`,
  its controller refresh, a `for (row …)` over the three lines) and Bench Tests (`src/programs/bench_r3a.cpp`) write
  every changed controller line in one burst too — C3's defect, in programs COMP-b does not own (shulib Drive has
  a hardware history and T4 Part 3 replaces it; Bench Tests is R3a's). The shared `ControllerLineWriter` is there
  for either; scheduling it is the coordinator's.
- **Two more `src/` comments made true *(this run)*, found by the round's prose grep** (`status-change line|within a
  tick of the|stopped the auton task|result page reads|every changed line|lcdRow_|half \(a\).s failure is contained|
  never shares the robot|gc-sections drops|the one place .* touch fields are read` over `docs src test`, every hit
  read): `competition_program.cpp`'s `logPhase` doc and `competition_run.hpp`'s `competitionPhaseLine` doc said
  Station K reads the phase lines "against the status-change lines" — now "in serial order: the NEXT phase
  function's entry line marks a mode change (HA-161's settle)". Every other hit is true for the tree, history, or
  another program's (the Drive/Bench note above). The three b-mutations ran on a copy taken before this run's word
  changes (the refusal line, these two comments) — words and comments only; the suite below ran on the final tree.

### the final gates after the review round — every one on the final tree (`date` read Thu Oct 1 07:35:52 PM EDT 2026 at the first, 08:05:35 PM at the last)
- **Host:** briefing regenerated → `cmake -S test -B build/test` → the full build (`Linking CXX executable
  shulib_tests` present; the doc gates inside it green) → **1,885 cases / 2,701,678 assertions, 3 skipped —
  SUCCESS** (the reviewed tree: 1,871 / 2,701,264; +14 cases) → briefing regenerated → `briefing_status.py check`:
  "generated block current". Run again on a FRESH link after the gate's self-test had re-timestamped its plant
  targets: the same counts, SUCCESS.
- **Doc gates, run explicitly:** `api_doc_tool.py self-test` OK; `check-coverage` "2956 public entities across 157
  headers, all documented"; `check-fresh` exit 0 (no `///` comment under `include/shulib/` changed —
  `git diff 0f56301 -- include/` is empty, so `docs/api` needs no regeneration); `check-examples` "520 quoted
  lines, all verbatim (6 source files); 6 listings of whole-run files, each one unbroken run" (run again after
  chapter 17's last edit — the same); `check-removability` "no public doc references docs/internal/";
  `briefing_status.py self-test` "OK (17 cases …)"; `doc_staleness_audit.py self-test` "OK (10 detector/filter
  cases)" and the audit "clean (29 live docs …)". Relative links of every changed Markdown file, chapter 17 and
  `COMP-COMPLETED.md`: 0 missing (`r2/c3_links.py`).
- **CI greps:** "library is PROS-free (all of include/shulib except hal/pros/)"; "layering holds: core is sim-free".
- **ARM header gate** (RESUMING's command, the TU in the scratch directory, never `/tmp`): 193 headers, exit 0.
- **`tools/src_build_gate.py check`: "src build gate: PASS (all 8 builds link …)"** — every build `exit 0, 12/12 TUs
  compiled, warnings: 8 vendor-ignored, 0 disallowed, hash asserted (v0.1.1-356-g0f56301-dirty), beacons (variant,
  robot) and program checked`; `[chain-table] 8 builds, 3 robots`; `[program-table] 7 programs, 19 entry names`;
  `[include-root] 42 source files`; `[printed-path]` clean (`r2/c3_final_gate.log`).
- **`tools/src_build_gate.py self-test`: "OK (203 detector cases, each against a real build)"** (12 min); afterwards
  the plant targets verified restored (`git diff` of `bench_r3a.cpp`, `test/vendor/doctest.h`, `lv_obj_style.h`
  empty; `test/build_selection_test.cpp` carries only the "other seven branches" comment; the Makefile no plant
  comment). (An earlier pass of both, before the two comments above, gave the same lines: `r2/c3_gate_check.log`,
  `r2/c3_gate_selftest.log`.)
- `src/main.cpp`: **298** lines (unchanged this round; ND6's 300 held).
- `COMP-COMPLETED.md` updated from this log: the ND3 proof as re-measured on the final tree, the tests and the
  round's mutations, the gate numbers, the documents, and the open question C5.

**Tree left uncommitted, as instructed. Nothing in Part b has run on a robot — Station K is its first run.**

### Verification-round findings and their disposition (a fresh verifier, same worktree, same builder) — *`date` read Thu Oct 1 09:16:29 PM EDT 2026 at its start*
- **Three findings** (one major, two minor), each with mutations the verifier ran on its own copy
  (`scratch/comp-b/verify/tree`, the specs in `verify/spec_v.json`, the results in `verify/mut_v.jsonl`, its probe
  `verify/zz_verify_probe_test.cpp`; read, never run in the worktree). This round's own scratch is
  `scratch/comp-b/v3/`.
- **Baseline, measured before any edit:** the six files the findings cite (`competition_run.hpp`,
  `competition_program.cpp`, `driver_panel.hpp`, `competition_run_test.cpp`, `competition_source_test.cpp`,
  `driver_panel_test.cpp`) are byte-identical (`cmp`) to the verifier's copy, and `diff -rq` of `src/` and `test/`
  against it is empty, so its line numbers hold. Newer than the 20:05 suite binary: only this log and the
  regenerated `PROJECT-BRIEFING.md`.
- **Each finding re-checked on THIS tree before any test changed** (`v3/mutrun.py`, the verifier's runner with
  its paths moved to `v3/` and its failing-case parser narrowed to doctest's `: ERROR: ` lines — the verifier's
  listed every case that printed a message; each run a fresh link, the full suite, the file restored byte-exact
  (asserted) and its timestamp bumped): the verifier's V01–V05, and the two other survivors in its own record that
  it did not raise (V06, the stale threshold 0.1 → 0.05 s; V11, TOO LATE decided before a refusal), are all
  **GREEN on the full suite, 1,885/1,885** (`v3/mut_prefix.jsonl`). All three findings are REAL.

**The dispositions:**
- **Finding 1 (major) — R21 tested only a freshly seeded watcher; V01 reopened C2: REAL, FIXED.** R21's request
  now meets the watcher the PROGRAM has: half a second of the auton task's idle ticks with no loop (observed every
  tick, as (P4) pins), then `enter()` and a second of driver control (100 beats, the auton task observing every
  tick half a tick out of step), then the change to autonomous. Not deleted: the loop leaves at its next tick
  (`driver.ticks == ticksBefore + 1`, its exit at or after the request), the 40 in FINISHED, the exit before the
  plant moved (and the exit RAN: `plantAtExit >= 0`). Deleted: served `kDriverLoopStaleSeconds` after the LAST beat
  seen (the bound tightened from `stale - 0.005` to `stale`, since that beat was seen at the request). The verifier's
  PV1 is now R21's direct watcher block: two seconds of beats seen, then `mayServe` false at +0.05 s and at
  stale − 1 ms after the last beat, true at stale; one late beat restarts the count from it; a loop that LEFT is let
  go at once. The fresh-seed state was dropped, not kept beside it: under the real `observe()` it is the same state
  as the long history (`movedAt_` is the last change seen either way), so it adds no case — it only hid V01.
  **V01 RED** — on R21's main scenario: `w.waited` false, `ROUTINE STOPPED == FINISHED`, plant travel 0.6 at the
  driver loop's exit (`v3/spot_v01_v05.log`, the verifier's PV2 numbers exactly). Its first iteration's
  `REQUIRE(lines.count == 5)` ends the case there, so the watcher block's catch of V01 is REASONED (its first
  sighting at 0.0 would make `mayServe` true at +0.05 s), not observed separately.
- **Finding 2 (minor) — the handshake's store-then-load order unpinned; V02/V03/V05 survived: REAL, FIXED.** New
  **(P16)** in `test/competition_source_test.cpp` pins every member of the pair WHOLE in
  `src/programs/competition_run.hpp`: `enter()` (the beat, the claim STORED, then `busy()` READ, the refusal's undo),
  `beat()`, `leave()`, `active()`, `beats()`, `request()` (its `fetch_add` with no order), `pending()`, `busy()`,
  `finish()`; and `memory_order_relaxed` occurs exactly twice in the file (the request time's store and load,
  ordered by the seq_cst increment and read around them), `DriverLoopPresence` names no memory order,
  `requested_.fetch_add(` once, `active_.store(` three times. A pin, not a two-thread stress test: the race's window is a few
  instructions wide, so a stress test would catch a broken order only by chance — it could not guard it. V05 has a BEHAVIOURAL test too:
  R21's new block — an earlier driver period's loop that left at the disable, the watcher's last change long before,
  then `enter()` and a request before the loop's first tick: the auton task waits one tick and the loop LEAVES
  (V05: no wait, `w.waited` false — RED on that block alone, `v3/spot_v01_v05.log`). The source test's header and
  two `///` comments in `competition_run.hpp` (the handshake's class doc, `enter()`'s doc: it beats first, and P16
  pins it) now say so — `src/programs/`, not `include/shulib/`, so `docs/api` is not touched.
- **Finding 3 (minor) — D5 tested the 0/1 alternation one way; V04 survived: REAL, FIXED.** D5 gains the mirror:
  line 0 changes at every write (the battery under load) while line 1 changed once — line 1 is written at the
  write after line 0's (2.10 s), then line 0 alone while line 1 is current; D5's banner says "tested in BOTH
  directions". **V04 RED** (D5).
- **Also closed, from the verifier's own record (not raised by it):** **V11** — a late request on a REFUSED boot
  (a program error; a typed-table boot) is still REFUSED with its own reason, never TOO LATE (the refusal is what a
  person must act on, and it would have refused in time too): R20 gains both. My first form also asserted
  `rec.cause == None` and FAILED on the unmutated tree — wrong, not the code: `rec.cause` records the watch's latch,
  which `latchNow()` sets at every serve (the code's own comment: "its read is a skip's"); it now checks the word and
  that line 0 says REFUSED, not TOO LATE. **V06** — R21 now checks `kDriverLoopStaleSeconds >= 10 × 0.01 s`, the
  REASONED margin the register names under HA-161 ("a driver-loop tick that took longer than 0.1 s while a request
  waited would be taken for a deleted loop"): lowering it is a decision for the register, not an edit.
- **Mutations — this round, on the fixed tree** (`v3/mut_fixed.jsonl`, each a fresh link, the full suite, the file
  restored byte-exact and re-timestamped): **V01 RED (R21), V02 RED (P16), V03 RED (P16), V04 RED (D5), V05 RED
  (P16 and R21), V06 RED (R21), V11 RED (R20)**, and four more relaxed orders of the pair that only P16 can see —
  **X01** `active()`'s load, **X02** `busy()`'s read of the request, **X03** `enter()`'s claim stored relaxed,
  **X04** `pending()`'s read (`v3/spec_v3.json`) — **all RED (P16)**. **11 valid mutations, 11 RED**; plus the two
  spot runs above (V01, V05 against R21 alone, `v3/spot.py`). After the campaign: `cmake --build` relinked from the
  restored sources ("final rebuild ok", and a relink after the spot runs).
- **Prose review — greps RUN and hits READ before this entry:** `R21|(D5)|P15|single-threaded|neither starves|in
  turn|store-then-load|first sighting|movedAt|the one ORDER` over `docs src test` (`.md`, `.hpp`, `.cpp`), and
  `P1.P15|P1–P15|P15)|1,885|1885|2,701,678|seeded|fresh watcher` over `docs src test tools README.md`. Stale and
  fixed: `COMP-COMPLETED.md` :58 (R21's description), :62 (P1–P15 → P1–P16; D5 both directions), :72 (the suite),
  its mutation paragraph and its gates heading. True and left: `driver_panel.hpp:110` ("the other two in turn" —
  now tested both ways), `competition_run_test.cpp:900` (R18's pointer to P15). Every other hit is another
  subject (R5a, SAFE-SAVE, DIAG1, H1 "D5"s; worksheet "turn back"; "seeded" plant runs) or history.
- **Not changed:** no program behaviour (the source edits are two `///` comments); chapter 17, the worksheet and
  the register describe behaviour that did not change, so none of them moves. C5 and the D2/RC2 anchor stay the
  coordinator's (above).

### the final gates after the verification round — every one on the final tree (`date` read Thu Oct 1 10:01:52 PM EDT 2026 at the first, 11:26:19 PM at the self-test's end)
- **Host:** briefing regenerated → `cmake -S test -B build/test` → the full build (`Linking CXX executable
  shulib_tests` present; the doc gates inside it green) → **1,886 cases / 2,701,750 assertions, 3 skipped —
  SUCCESS** (after the review round: 1,885 / 2,701,678; +1 case, P16) → briefing regenerated →
  `briefing_status.py check`: "generated block current". Run again on a FRESH link after the gate's self-test had
  re-timestamped its plant targets: the same counts, SUCCESS (`v3/suite5.log`).
- **Doc gates, run explicitly:** `api_doc_tool.py self-test` OK; `check-coverage` "2956 public entities across 157
  headers, all documented"; `check-fresh` exit 0 (`git diff 0f56301 -- include/` empty: the two `///` edits are in
  `src/programs/`, so `docs/api` needs no regeneration); `check-examples` "520 quoted lines, all verbatim (6 source
  files); 6 listings of whole-run files, each one unbroken run"; `check-removability` "no public doc references
  docs/internal/"; `briefing_status.py self-test` "OK (17 cases …)"; `doc_staleness_audit.py self-test` "OK (10
  detector/filter cases)" and the audit "clean (29 live docs …)".
- **CI greps:** "library is PROS-free (all of include/shulib except hal/pros/)"; "layering holds: core is sim-free".
- **ARM header gate** (RESUMING's command, the TU at `v3/all.cpp`, never `/tmp`): 193 headers, exit 0.
- **`tools/src_build_gate.py check`: "src build gate: PASS (all 8 builds link …)"** — `[tank-competition] exit 0,
  12/12 TUs compiled, warnings: 8 vendor-ignored, 0 disallowed, hash asserted (v0.1.1-356-g0f56301-dirty), beacons
  (variant, robot) and program checked`; `[chain-table] 8 builds, 3 robots`; `[program-table] 7 programs, 19 entry
  names`; `[include-root] 42 source files`; `[printed-path]` clean (`v3/gate_check.log`).
- **`tools/src_build_gate.py self-test`: "OK (203 detector cases, each against a real build)"**
  (`v3/gate_selftest.log`); afterwards the plant targets verified restored (`git diff` of `bench_r3a.cpp`,
  `test/vendor/doctest.h`, `include/liblvgl/core/lv_obj_style.h` empty; `test/build_selection_test.cpp` carries
  only its "other seven branches" comment; the Makefile no plant comment).
- `src/main.cpp`: **298** lines (unchanged; ND6's 300 held).
- `COMP-COMPLETED.md` updated from this log: R21's form, P1–P16, D5 both ways, the round's 11 mutations, the suite.

**Tree left uncommitted, as instructed. Nothing in Part b has run on a robot — Station K is its first run.**

## The coordinator's merge — COMP-b (2026-10-02, 01:10–01:45 EDT; COMP COMPLETE)

- Transport commit `9468a12` (worktree `comp-b`), squash-merged onto `7daf809`. No code conflict; the document
  conflicts (the register, HANDOFF, the roadmap, build-order's Next line) resolved to the merged state, `docs/api` and
  the briefing regenerated; REMAINING-WORK (row 5, the position — 34 of 63 chunks — the totals, C5) updated.
- Gates on the merged tree: suite 2,036 | 2,036; src gate self-test (198) and check PASS on all eight builds; ARM
  header gate (197); CI greps; check-fresh, briefing, removability, staleness — all clean; no plant
  (`.claude/coordinator/logs/compb_merge_verify.log`). Six coordinator mutations (`muts_compb.json`), all RED — listed
  in `COMP-COMPLETED.md`, "The coordinator's merge verification".
- C5 answered by the team lead the same night: drop just that auton (built in T4 Part 3's late step).

