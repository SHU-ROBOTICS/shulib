# STEP1 — live progress log

The brief of record: [`STEP1-route-steps-library.md`](STEP1-route-steps-library.md) (its answers block of
2026-11-03: "Accept all, go" on D1–D8; the "## Re-check — amendments" override the body). Appended in real time.

## STEP1-A

**Run:** STEP1-A only (the arrival tolerance; stop / carry on / skip; labels; the cut rule; the not-started answer;
"DONE, n FAILED"). **Executor:** a subagent on Claude Opus 5.5, in the worktree `.claude/worktrees/step1-a` (branch
`step1-a`). **Base:** `ef6162e` (main, 2026-11-03 plan commit) — every diff below is against it, never `eab0f89`.
**Host only:** nothing here claims the robot. Never committed by the executor.

### Log

- 2026-11-05 — log created before any code. Read RESUMING's standing rules; the brief's header, answers block,
  Re-check amendments, D1–D8, §1.1, §1.3 A1–A7, §2 (STEP1-A rows), §3.1, §3.3, §4, §6, §7, §8 in full.
- 2026-11-05 — code, first pass: `chassis.hpp` (MotionOptions' two appended tolerances, validate,
  effectiveConfig), `motion_config.hpp` (`loosenedSettle`), `routine.hpp` (the STEP1-A blocks: OnFail and
  its three spellings, `MechanismNotStarted`, the six RoutineResult fields, `onFail`/`label`, the door
  `enterStep`, the policy in `recordStop`/`stopHere`/`carryOnFrom`/`stopPendingSkip`, the A6 mark, the
  guard from the context), `src/programs/` (the DONE words, the two-line form, the tones, the stop word).
  `-Wswitch` stopped the build at `routineStopWord` exactly as L14 predicted, until its case was added.
  Docs gate: three `using` declarations undocumented, fixed; `docs/api` regenerated. **Suite with no new
  tests: green, 2961 cases (the base count — ST1's "every existing test unchanged").**
- 2026-11-05 — **ST1's golden chains captured at the BASE**: `test/routine_golden_chains.hpp` (seven chains
  — corrected from "six" at the review: every chassis step form succeeding; five failure kinds; robot x's
  mechanism steps ending TIMED_OUT) was
  compiled against `git archive ef6162e` (`.claude/coordinator/scratch/step1-a/base/`) and against this
  worktree by `scratch/step1-a/golden_probe.cpp`; the two outputs (`golden_base.out`, `golden_new.out`,
  38 lines) are byte-identical (`cmp`), and those lines are `routine_on_fail_test.cpp`'s ST1 literals.
- 2026-11-05 — tests written (all green on the first full run after two test-side fixes, both test
  errors, not code: (1) ST15 (c)/(d) expected 0 motions started, but the guard's PARK is a motion — the
  base chain starts 1; now asserted equal to the default chain's and == 1; (2) the line-budget loops set
  `failedOn` to INT_MIN, which (correctly) writes no line 3b — failedOn now stays > 0 there).
  `test/routine_tolerance_test.cpp` (ST2, ST3 at the probe's EXACT times — 1.80/1.35/1.12 s,
  1.40/1.25/1.14 s, 1.18/0.82 s, 1.21/0.63 s — ST23), `test/routine_on_fail_test.cpp` (ST1, ST4, ST5,
  ST6, ST24, ST26, ST27, ST28, ST16's then() half, ST25's Warn half), `test/routine_cut_rule_test.cpp`
  (ST15 (a)–(d) through the real runAuton, every RunGuardReport field equal to the default chain's;
  ST16's latch and outside replacement; ST25 over an idle guard), `mechanism_auton_steps_test.cpp`
  (ST18 (a)–(e), every starting form, ST7), `auton_tester_run_test.cpp` (23c), `competition_run_test.cpp`
  (R25), `competition_screen_test.cpp` (S15) — ST17; the F10/F6 pins (ST8). **Suite: green, 2996 cases
  (2961 + 35).**
- 2026-11-05 — docs, in the same change: guide 09 (a new section, "Choosing what a failure does: `onFail` and
  `label`" — the three choices, `label`, forward-only skips, the two rules, the context form and the
  guard-less Warn, the tolerance in D2's words; the old "continue past a failure ... one tier down" bullet
  updated), cookbook 02 (two recipes, `cookbook-02c` "Carry on past a missed piece" and `cookbook-02d` "Give
  up on this piece, go for the next", context form; the intro and "no try-this-step mode" sentences made
  true again), cookbook README (two rows; where the recipes compile), guide 10 (MotionOptions' field list),
  guides 16 and 17 (the DONE word, its amber), the changelog (top of API 2.5), the roadmap's F6 and F10
  rows (F10's pin count 59 → 70), the HA register (HA-216 registered; HA-217..218 held for STEP1-B; 204
  rows), `docs/api` regenerated, the briefing regenerated (F6 48 → 50, F10 59 → 70 pins; 2999 cases).
  **Suite: green, 2999 cases (2961 + 38).**
- 2026-11-05 — **the prose review (RESUMING rule 4), run BEFORE this line was written.** §3.3's greps:
  `simulator` in guide/cookbook — 12 hits, all pre-existing, 0 in this change's added lines; `ROUTINE STOPPED`
  — guide 16 :91 (a code comment on an auton with no `onFail`: true), :187 (the new list), :238 (a weak-side
  run that stops: true), guide 17 :259 (reworded: "a step failed and stopped the routine (or a skip never
  found its step)"); `branches on the verbs directly\|continue past` — routine.hpp :145 (updated: "writes
  onFail(carryOn) ... or branches on the verbs directly"), guide 09 :213 (rewritten), check.hpp ×3 (another
  meaning); removability grep — empty. Rule 4's own greps (each changed file's name and concepts over
  `docs/*.md docs/guide docs/cookbook README.md`): `MotionOptions` (changelog, HA register, roadmap, guides
  09/10 — 10's field list updated), the settle names (guides 05/08/12/14 — each about the robot's own
  settle numbers: still true), `RoutineResult` / `RoutineStopCause` (cookbooks 01/02/03/05/07, guides
  09/10/16/18 — read; true of the default), "stops the chain / skip the rest / first failure" — four were
  absolute statements now true only by default and got one clause each: guide 15's "recipe" entry, cookbook
  README's rule 2, guide 10 :726 (the mechanism steps' policy), guide 18 :366-368; guide 08 :150 (Tier-3
  `if` ladder), cookbook 01 :38/:74, cookbook 03 :67, guide 11 :293 (then()'s verdict), guide 10 :468 — read,
  still true. Guide 11 lists the RTN lines a student reads: it gained one bullet for the new ones (each
  quoted line is asserted in a test: ST4/ST6/ST15/ST24/ST25, cookbook-02c/02d). The program headers' names
  (`auton_tester_run`, `competition_run`, `competition_screen`, `auton_tester_panel`) appear only in the
  HA register (other entries' sources): nothing to change.

### Deltas from the brief (against the base `ef6162e`), each with its reason

1. **No version bump; the changelog entry is at the top of `## API 2.5`** (the brief: `kApiMinor` 5 → 6,
   "API 2.6"). The coordinator's instruction for this run: changelog entries at the top of API 2.5, and
   `version.hpp` is shared — "the coordinator numbers at merge". `version.hpp` untouched; the F10 pin's
   `kApiMinor >= 5` floor untouched.
2. **The two new recipes compile in a NEW example test, `test/routine_on_fail_example_test.cpp`**, not
   `test/cookbook_examples_test.cpp` (§1.1/§3.1). They run robot x's declared mechanisms through the REAL
   `runAuton` on the MECH-A4 rig, in the context form the brief requires (re-check #8); the cookbook file
   has neither the rig nor the robot's namespace (its own `Intake` struct would collide with the robot's
   words), and `check-examples` reads every `test/*example*_test.cpp`. It also keeps this run off a file
   in-flight L6-A2 may touch. `cookbook_examples_test.cpp` and `test/CMakeLists.txt` (the glob) untouched.
   Guide 09's listing is case `guide-09e` in the same file.
3. **ST27's "the next step starts from rest (wheel speed 0 at its first tick)"** is asserted as: a Tier-3
   motion left running under the chain is CANCELLED by the carried-on failure (`motionsCancelled` + 1, no
   active motion), every drive motor at 0 V under Brake, and the robot does not move through the next
   step (a 0.3 s pause). The host test plant's velocity changes only on a pace, so the speed read at the
   next step's first tick is the previous pace's, not a statement about the cancel.
4. **A skip pending when the guard's time is up** (A5: "stops at the next step's door") — the brief does not
   say which step the stop is recorded at. Built: the skip's ORIGIN failure becomes the chain's STOP
   (`stoppedAt`/name/cause/exit/verdict of the failing step; it leaves `failedOn` and, if it was the first,
   the first-failed fields), exactly as if it had chosen stop — A5's "the choice is IGNORED" — so the
   result's stop fields equal the default chain's (ST15's invariant); the steps already jumped stay counted
   as jumped; the door's step and every later one are skipped. One Warn: "... STOPPED at step N (name):
   <cause> — the time ran out while skipping to 'X' — skipping the rest; drive safed".
5. **ST15 (d) compares the SKIP chain with the default only.** A carry-on chain whose failure came BEFORE
   the deadline legitimately goes on, and issues its next step after a frozen wait exactly as a default
   chain whose step had succeeded would (§0.3 item 8); (d) is about a pending skip.
6. **Two log sentences the brief did not spell:** an Info line where a skip LANDS ("the skip to 'X' landed
   at step N (name)"), so a transcript shows where the auton resumed; and, on a stop that OVERRULED a
   non-default choice, a clause "— the step was CANCELLED, so its on-failure choice is ignored" / "— the
   time is up, so its on-failure choice is ignored". Under the default rule no line changes (ST1).
7. **The "behind" Warn is also logged at `label()`**, when `onFail(skipTo("X"))` came first and
   `label("X")` after it on the same step (re-check #10 names the `onFail` call; that order would escape it).
8. **The guard is read from a context member `guard` convertible to `sequence::RunGuard&`** (a reference or
   an object; a pointer member is not read — no context in the tree has one).
9. **Documents beyond §3.1's seven** (pace rule 7's ~5 was already exceeded by the brief's recommendation):
   guide 11 (the new RTN lines), guide 15's glossary "recipe" entry, guide 18 and guide 10's mechanism
   paragraph (one clause each), cookbook README (two table rows, where the recipes compile, one clause in
   its rule 2 and its "untested" line, which had missed chapter 7's file too). Each was a prose-review hit
   that would be false, or incomplete, the day this lands.
10. **HA: only HA-216 registered** — §4 gives HA-217/218 to STEP1-B's code; the Held line now says so.
- 2026-11-05 — **gates on the clean tree** (before any mutation): the ARM header gate — 240 headers in one TU,
  `-Werror`, exit 0; the CI greps (pros/ outside hal/pros, shulib/sim outside sim) — both empty;
  `api_doc_tool` self-test OK, check-coverage "5524 public entities across 197 headers, all documented",
  check-fresh exit 0, check-examples "665 quoted lines, all verbatim (13 source files); 6 listings of
  whole-run files, each one unbroken run", check-removability clean; `briefing_status.py` self-test OK and
  check "generated block current"; `doc_staleness_audit.py` self-test OK and the audit clean;
  `src_build_gate.py self-test` "OK (269 detector cases, each against a real build)" (25 min) — then
  `git diff -- Makefile src/ | grep '(self-test plant'` empty; `src_build_gate.py check` "PASS (all 7 builds
  link ...)".
- 2026-11-05 — **ST8, the pins, each seen failing the build under its own mutation** (scratch
  `mutate_pins.py`; each compiles ONLY the pin's TU and counts only when the error is the pin's own
  static_assert): P1 `onFail(const OnFail&)`, P2 `label(const char*, int = 0)`, P3 `stopChain` = CarryOn, P4
  `carryOn` = Stop, P5 `skipTo(const char*, int = 0)`, P6 `MechanismNotStarted = 7`, P7 `long jumped`, P8
  `positionTolerance` a Velocity, P9 `headingTolerance` an AngleDim — **9 of 9 CAUGHT, each by its own pin's
  message and nothing else (1 error line each)**; restored and both TUs recompiled from the restored tree.
  Not mutation-proven: the two "non-const" pins (no mutation can make a modifier const without changing its
  signature pin first) — said, not claimed.
- 2026-11-05 15:37 (date reading) — **RESUMED** after the stop (the team lead paused every job). Found: the
  mutation run (scratch `mutate.py`) killed by SIGTERM while SM18 was building — its handler restored the file
  ("!! signal 15: restored and stopped"); every live file byte-equal to every mutation backup under
  `scratch/step1-a/bak/` (21 compared, `cmp`), `git diff -- Makefile src/ | grep '(self-test plant'` empty.
  Runtime mutations done before the stop (each red on a fresh build, restored): SM1, SM2, SM3, SM4, SM9, SM10,
  SM11a/b, SM12, SM16, SM17 (`scratch/step1-a/mutations.log`). Left: SM18 onward. The suite binary removed and
  rebuilt from source before any result below.
- 2026-11-05 — **§2.3's STEP1-A mutations, every one CAUGHT** (scratch `mutate.py`: one exact replacement
  asserted unique, the file touched, the suite binary rebuilt — `make shulib_tests/fast`, so every object
  depending on the file recompiles — run capped `ulimit -v 4194304; timeout 600`, restored in a `finally` and on
  SIGTERM/SIGHUP/SIGINT, the restored file touched; a mutation whose build fails is re-cut, never counted — none
  did in the counted runs; none was killed by the cap or the timeout; the full log `scratch/step1-a/mutations.log`).
  Before the stop: **SM1** (26 red, incl. ST4/ST6), **SM2** → ST28, **SM3** → ST4/ST5/ST6/cookbook-02c,
  **SM4** → ST3/ST23, **SM9** → ST16 ×2 (the brief named ST15 (a): ST15 (a)'s motion is cut by the DEADLINE, so
  `timeIsUp()` still stops it — the CANCELLED rule alone is what ST16's outside replacement and `then()` test),
  **SM10** → ST15 (c), **SM11a** (tester) → 23c, **SM11b** (competition) → R25 + S15, **SM12** → ST18 (a)/(b)/(d)+(e)
  /forms + ST25, **SM16** → ST3 ×2 + ST23, **SM17** → ST23. After the resume (fresh build first, green 2999):
  **SM18** → ST25 ×2, **SM19** → ST24, **SM20** → ST26 (+ ST15 (a)/(c), ST18 (d)+(e), ST25, ST28),
  **SM21a** (tester line 3b) → 23c, **SM21b** (competition line 3b) → R25, **SM22** → ST27, **SM27** → ST15 (d),
  **SM29** → ST18 (d)+(e), **SM30** (a jumped `stop` marks) and **SM30b** (a jumped `waitUntilDone` marks) →
  ST18 (c), **SM33** (the `onFail`-time behind check) and **SM33b** (its `label`-time twin, delta 7) → ST24,
  **SM34** → ST16 (`then()`), **SM35** → ST25 (the Warn), **SM36** → ST23's `e = 0` case. **26 of 26 runtime
  mutations red** (+ the 9 pin mutations above). After the run: the 26 mutated files `cmp`-equal to their
  backups, `git diff -- Makefile src/ | grep '(self-test plant'` empty, "final restore build: ok".
- 2026-11-05 ~16:50 — **final gates on the restored tree** (binary removed, briefing generated, full build,
  briefing generated again, build): the suite through `suite_run.py green` — "green — 2999 cases, 2956867
  assertions" (2961 at the base + 38); the ARM header gate — 240 headers in one TU, `-Werror`, exit 0; the two CI
  greps empty; `api_doc_tool` self-test OK, check-coverage "5524 public entities across 197 headers, all
  documented", check-fresh exit 0, check-examples "665 quoted lines, all verbatim (13 source files); 6 listings
  ...", check-removability clean; `doc_staleness_audit.py` clean (exit 0) and its self-test OK; `briefing_status.py`
  check "generated block current", self-test OK; `src_build_gate.py self-test` "OK (269 detector cases, each
  against a real build)", then `git diff -- Makefile src/ | grep -c '(self-test plant'` = 0, `check` "PASS (all
  7 builds link ...)". §7's own greps: route.hpp/path_runner.hpp diff 0 lines; service.hpp diff 0; no existing pin
  line removed; `timeIsUp()` reads `running()` first (routine.hpp :1270); `auton %.15s: %.27s` still at
  competition_screen.hpp :556; `src/` diff 4 files. §3.3's greps RE-RUN after the mutations (each hit as the
  prose-review entry above logs it: `simulator` 12, 0 added; `ROUTINE STOPPED` 4 hits, each true;
  `continue past` / `branches on the verbs` as logged; removability empty). **STEP1-A's build is complete;
  handed to the coordinator for its reviewers, the fresh verifier and its §7 re-runs. Not committed.** (The src gate's self-test touches `src/` files, so the binary was rebuilt after it and the suite re-run: green, 2999; the audit then derived its counts from a trusted binary.)

### STEP1-A review findings and their disposition (2026-11-05, evening)

Three reviewers (correctness, tests, docs) returned 20 findings; two pairs overlap (correctness #2 = tests'
"line 3b names a carried-on drive timeout"; the docs findings are all distinct). Each was re-checked against the
code before it was answered. **Every diff below is still against `ef6162e`.**

**Code**

1. **(correctness, minor) A start that `find(goal)` cannot place was never marked — REAL, FIXED at the root.** The
   mark was found by the step's GOAL (`find(goal)`: name + units + preset/position), the later wait by its
   DECLARATION (name + kind/units). A goal naming a preset this robot's same-named mechanism lacks found no slot, so
   (a) a carried-on start left the slot unmarked and the later wait THREW (the precondition) out of the auton, (b)
   after an earlier SUCCEEDED step the wait answered that older verdict, (c) a jumped start did the same. Fix: the
   mark is now by NAME (`markNotStarted(mechanismName, why)` over `mechs_->slotFor(name)`, unique on a robot):
   `startingStepPassed(step, mechanismName)` uses it for a jumped start (the `Find` lambda is gone), and the five
   starting steps' not-found path goes through a new `startNotOnThisRobot`, which marks the same-named slot "its
   start did nothing: another robot's declaration" and then calls `notOnThisRobot` unchanged. `stop()` and
   `waitUntilDone()`'s own not-found paths still call `notOnThisRobot` and mark nothing (A6: a stop or a wait never
   marks). Under the default the chain stops at that start, so the mark is never read (ST1 still byte-identical).
   **Delta 11 (below).** Test: "ST18 (review)" — the reviewer's three probe cases, through a real `Arm armB` with
   robot x's name and a `"reach"` preset robot x lacks (no throw; `MechanismNotStarted`; `completed` 1, never 2;
   nothing moved; both line texts). Docs: the `waitUntilDone` and `MechanismNotStarted` `///` comments, the banner's
   rule 2 and the MECH-A4 block's STEP1-A paragraph; guide 09's rule 2, guide 11's RTN bullet, the changelog's
   `MechanismNotStarted` bullet.
2. **(correctness, minor; = tests' last line-3b finding) Line 3b says "motion failed" for a carried-on drive
   TIMEOUT — REAL, NOT changed in the public type; pinned and flagged.** `RoutineResult` has no exit for its first
   failed step, so both programs print `routineStopWord(cause, Running)` → "motion failed" (true, less specific than
   the step's own RTN Warn, "motion TIMEOUT"). The fix the reviewer names — a seventh field `firstFailedExit` — is a
   public F10 field the brief does not have (§1.1: "APPENDS six fields", the team lead's "Accept all" of 2026-11-03),
   and adding public surface beyond an answered brief is the team lead's call (pace rule 2), not the builder's.
   Appending it later stays additive (an aggregate's trailing field). So: the current word is pinned (23c and R25 each
   gained a `MotionFailed` line-3b case: "... (motion failed); the routine went on"), and **FOR THE COORDINATOR / THE
   TEAM LEAD:** recommended — append `control::ExitReason firstFailedExit` (pinned by type) before F10's STEP1-A pins
   are relied on by the route maker, so line 3b can say "motion TIMEOUT".
3. **(tests, major) RM8 — a skip whose label is the first step after the deadline — REAL test gap, FIXED.** New
   "ST15 (d) (review): the skip's LABEL is the first step after the deadline": `r.label("back").driveTo(...)` right
   after the frozen wait; the guard's report equal to the default chain's, `motionsStarted == 1` (the park only),
   `stoppedAt == 1`, `skipped == 2`, the "time ran out while skipping to 'back'" line, and no "landed" line.
   Mutation **SM27b** (the reviewer's RM8: the label read before the time) — result below.
4. **(tests, major) RM4 — the reset at a deadline-stopped skip — REAL test gap, FIXED.** New "ST15 (d) (review): ...
   after an EARLIER carried-on failure": step 1 `onFail(carryOn)` false, step 2 `onFail(skipTo("back"))` returning
   `ExitReason::TimedOut`, then the deadline: `stoppedAt` 2 / `stoppedName` "second" / cause ActionFailed / `exit`
   TimedOut, `failedOn` 1, `firstFailedAt` 1 / "first" / ActionFailed, `skipped` 2, the stop line, the guard's report
   equal to the default chain's. The programs' line 3b for exactly that shape ("first failed: step 1 ...; the routine
   went on, then stopped") is 23c's and R25's existing "went on, then stopped" case — the library fields that feed it
   are what RM4 broke, and they are what this case pins.
5. **(tests, major) RM1 — the label memory's 32 — REAL test gap, FIXED.** ST24 gained "the label memory is the last
   32": k = 2, 32 and 33 labels passed, the skip naming the oldest — k ≤ 32 stops "behind" (`stoppedAt` k+1, two
   "behind" Warns), k = 33 reads as a target that never comes (`skipPendingTo` "L1", `jumped` 1, no "behind"). The
   32 is pinned at its edge both ways: **RM1a** (31) and **RM1b** (33) — results below. Guide 09 now says the routine
   remembers the last 32 labels (the banner already did).
6. **(tests, minor) RM2 — a label on a jumped step — REAL gap; the rule DECIDED as built and pinned.** A label on a
   step a skip jumped counts as passed: the chain runs its lines in order, so that label is behind any later step
   (forward-only, D3), and the other reading would turn a backward skip into one that passes over the rest of the
   auton — the very failure SM19 guards. ST24 gained "a label on a step a skip JUMPED counts as passed" (the
   `onFail`-time Warn, `stoppedAt` 4, `failedOn` 1, `jumped` 1). One clause each in guide 09 and the banner.
7. **(tests, minor) RM3/RM5/RM6 — the deadline-stop's copied fields — REAL gaps, FIXED.** ST15 (d) now also checks
   `firstFailedName` "" and `firstFailedCause` None (RM3); finding 4's case checks `exit` TimedOut (RM5); new "ST15 (d)
   (review): ... from a failed MECHANISM step" (a `run(intake, 12_volt, seesPiece, 0.3_s)` with no piece) checks
   `mechanismVerdict` TimedOut, equal to the default chain's (RM6).
8. **(tests, minor) RM9/RM10 — `competitionDoneWord`'s edge and table — REAL, FIXED.** R25's loop over -1..12 now
   checks the exact word for every n against the rule ("DONE, n FAILED" for 1..9, n < 1 read as 1, "DONE, 9+ FAILED"
   past 9) and `competitionResultWord` for each count; 23c's loop checks the tester's table against the SAME rule
   (the two headers stay separate — the COMP-t brief's §4 — so the shared rule is what keeps them equal). Mutations
   **RM9**, **RM10** (competition) and **RM10t** (the tester's twin) — results below.
9. **(tests, minor) RM11 — the tester's amber rule — REAL, FIXED.** 23c's RESULT-page colour check now loops
   `failedOn` 1..12 (amber, and the big line is that count's word). Mutation **RM11** — below.
10. **(tests, minor) the mutation driver and stale header dependencies — REAL as a process trap; the earlier
    verdicts stand.** The reviewer showed `make shulib_tests/fast` recompiles header dependents only once a full
    build has merged `compiler_depend.make`; my build directory had been fully built many times (the reviewer
    confirmed the merged list complete for all five mutated headers), so the 26 runtime verdicts above stand. The
    earlier sentence "`make shulib_tests/fast`, so every object depending on the file recompiles" was true here but
    stated without its precondition — read it with this one. `scratch/step1-a/mutate.py` now (a) runs one FULL
    `make shulib_tests` before the first mutant (it merges the dependency files) and (b) runs a CONTROL mutation
    (`CTRL`: `recordSuccess` counts a step twice) first, and stops with no verdict trusted unless it is red. The
    pre-review driver is kept as `mutate_v1_before_review.py`.
11. **(tests, minor) RM7 — `stopPendingSkip`'s `chassis_->cancel()` — REAL, and the code was the thing to change.**
    The reviewer found the cancel near-equivalent. Re-checked: it is not harmless — it breaks the invariant ST15
    exists for. The drive was already safed at the skip's own failure (`carryOnFrom` cancels exactly as `stopHere`
    would have for the default chain); the default chain's later doors cancel nothing; so a second cancel at the
    deadline door makes the skip chain do MORE than the default from the deadline on. A probe showed it only once the
    scenario paces after the door: a stray Tier-3 motion the auton starts after the buzzer is cut by the guard's
    latch in the default chain (`postExpiryCancels` 1) and by the Routine in the skip chain (0). **Fixed: the cancel
    is removed** (its doc says why); new "ST15 (d) (review): a stray Tier-3 motion after the deadline" asserts the
    two chains' reports equal and `postExpiryCancels == 1` in both. The mutation is now the reverse, **RM7r** (the
    cancel put back) — result below. The stop line still ends "drive safed": the routine safed it at the failure,
    and the guard owns every motion after the deadline. **Delta 12 (below).**

**Documents**

12. **(docs, major) REMAINING-WORK row 20c, build-order, HANDOFF untouched and unlogged — REAL, FIXED in part,
    the rest logged for the coordinator.** Row 20c: a status ("STEP1-A BUILT 2026-11-05 ... host- and build-gated,
    NOT run on a robot; in review. STEP1-B next", with the live log's link), the gate the brief's §6 corrects to
    ("STEP1-A before G3a Part R1's build; STEP1-B before R1-T") and the re-check's size (~6–8, was ~5–6½).
    **NOT edited, for the coordinator at merge:** `build-order.md` — it has no STEP1 entry (STEP1 appears only in
    G3a's dated status paragraph of 2026-10-30, whose "+ STEP1 ~5–6½" is history and is not rewritten, and in the
    phase table); a STEP1 status line needs a place the plan commit chooses, and its first bold `Next:` is the
    coordinator's at merge. `HANDOFF-remaining-work.md` — its only STEP1 mention (:684, the panel's scope) is still
    true; a STEP1-A line, if one is wanted, is the merge's. (Precedent: L6-A1 left build-order's `Next:` to the
    coordinator the same way, L6-PROGRESS.md :163.)
13. **(docs, minor) "Since API 2.5" in five public sentences — REAL, FIXED.** The version is removed from all of them
    (guide 09 ×3 — the history blockquote now reads "Then the on-failure choice arrived the same way", and its count
    of additive uses is "four times", which it now lists; guide 10's MotionOptions list; cookbook 02's intro), as
    L6-A1 and CAL1-A2b wrote theirs. The changelog entry stays at the top of "## API 2.5" by the coordinator's
    instruction; delta 1 stands (the number is the coordinator's at merge).
14. **(docs, minor) guide 17: the line count, the table's order, ROUTINE STOPPED — REAL, FIXED.** "one more — the
    first failed step — when the routine went on past a failed step or a skip never found its step"; the DONE row
    moved ABOVE FINISHED (`competitionResultWord` checks it first, `competition_run.hpp` :562-572); the ROUTINE
    STOPPED row now also names "the routine went on past its failed steps but the end action failed" (L12).
15. **(docs, minor) "all nine fields" and F6's "36 pins" — REAL, FIXED.** The roadmap's F10 row and routine.hpp's
    banner say fifteen (nine through MECH-A4, six at STEP1-A, each pinned by type); the F6 row says 50 pins "as
    `tools/briefing_status.py` counts them" (38 `SHULIB_F6_PIN` + 12 `SHULIB_F6_NOT_NOEXCEPT`; 48 before STEP1-A; the
    36 it said was the base's `SHULIB_F6_PIN` count alone — counted with `grep -o`, not assumed).
16. **(docs, minor) the banner's "in ONE delimited block" — REAL, FIXED.** It now says two delimited blocks (the
    types at namespace scope, the modifiers in the class), the state declared last, and the policy's edits inside the
    existing steps, each marked "STEP1-A". **Delta 13 (below).**
17. **(docs, minor) "six golden chains" / "four failure kinds" — REAL, FIXED.** Seven chains (G1, G2.0–G2.4, G6 — the
    probe's own labels) in the changelog, this log's entry above (corrected in place, marked so) and the golden
    header (comments only; the chains' code is untouched, so the base comparison stands).
18. **(docs, minor) HA-216 overstated ST3 — REAL, FIXED.** "the motor-lag plant (kA 0.02) ENDS the leg inside the
    tolerance (ST3, read at the verb's return — where that plant then comes to rest is not measured)".
19. **(docs, minor) guide 18's exception named only CANCELLED — REAL, FIXED.** "... except after a CANCELLED verdict
    or once the auton's time is up". The same omission was in routine.hpp's MECH-A4-block paragraph ("except that a
    CANCELLED verdict always stops"), found by the grep for this one and fixed alike.

**Deltas added by the review round** (continuing the list above):

11. **The A6 mark is SET by one more case than re-check #2 states:** a starting step that did nothing because its
    goal is another robot's (`startNotOnThisRobot`), marked "its start did nothing: another robot's declaration". The
    re-check's list was written against false marks from a stop or a wait (still: neither marks); this case is a
    starting step, and without the mark A7 ("nothing new throws on a run-dependent path") and D4's "never an older
    run's answer" were false under `carryOn`. CLEAR is unchanged (only `noteStarted`).
12. **`stopPendingSkip` does not cancel the drive** (A5's "stops at the next step's door"): the failure already safed
    it, and a second cancel made the skip chain differ from the default chain after the deadline (finding 11).
13. **routine.hpp's STEP1-A code is in two delimited blocks plus marked inline edits**, not §1.1's "one delimited
    block": the types must live at namespace scope and the modifiers in the class, and the policy had to change
    inside `then()`, `waitUntilDone`, `ok()`, `result()`, the door, `recordStop`/`stopHere`, `noteStarted` and
    `awaitMechanism`.

**The prose review for this round (RESUMING rule 4), run before this entry was written:** `grep` over
`docs/ include/ src/ test/ README.md` (not `docs/api/`, the brief, or other chunks' records) for "nine fields",
"NINE fields", "six golden", "six chains", "four failure kinds" — three hits left, each true (the nine FROZEN fields:
routine.hpp :357, the golden header :10/:48); "another robot's" in guide/cookbook — guide 09 :272, guide 11 :370/:380,
guide 18 :467, each true; `waitUntilDone` in guide/cookbook (guides 09/10/11/14/18, cookbook 02) — read: guide 18
:358/:380 and cookbook 02 :208 say a skipped start's wait fails counted, still true; nothing says a wait after a
start that did nothing throws; "except after a CANCELLED" / "except that a CANCELLED" — the two fixed in finding 19,
no third; "Since API 2.5" / "in 2.5" in this change's added guide/cookbook lines — none left (the one `2.5` match is a
`2.5_s` timeout); "time ran out while" — only routine.hpp, the cut-rule test and this log.

**The review round's mutations** (the verdicts this section's "below" promised; scratch `mutate.py`, which since the
review runs one FULL `make shulib_tests` first, so the merged header-dependency list is complete, then the CONTROL
mutant, which must be red before any verdict is trusted). First run 2026-11-05 17:50–18:32
(`scratch/step1-a/mutations_review.log`): CTRL red (56 cases), the 26 runtime SM mutations of §2.3 red again on the
review-round tree, and SM-R1a, SM27b, RM1a, RM1b, RM2–RM6, RM7r, RM9, RM10, RM10t, RM11, RM12t, RM12c all red.
That driver never wrote WHICH case went red (the lines that write them sat after the CTRL `return`, unreachable),
and SM-R1b was not in the run. Both are put right in the resumption below.

- 2026-11-10 20:55 EST (date reading) — **RESUMED the fix round** (stopped 2026-11-05 ~18:30 by the 5-hour usage
  limit; the laptop was off until today). Found: all 19 findings dispositioned above and their edits in the tree,
  the review mutation run complete ("final restore build: ok"). Checked before trusting anything: every file the
  mutation tooling backed up `cmp`-equal to its latest backup under `scratch/step1-a/bak/` (routine.hpp,
  chassis.hpp, motion_config.hpp, competition_run.hpp, auton_tester_run.hpp, auton_tester_panel.hpp);
  `git diff -- Makefile src/ | grep -c '(self-test plant'` = 0; no source or test file changed after the
  mutation run began (the newest test edit 17:41, this log 17:52; the headers' later times are the driver's
  restore touches). The test binary was removed and relinked from source (`cmake -S test -B build/test`, full
  build) before any result below.
- 2026-11-10 — **the driver's logging fixed** (the red-case lines moved before the CTRL check; the 2026-11-05
  driver kept as `mutate_v2_2026-11-05.py`) and **the review round's 18 mutants re-run on fresh builds**
  (`scratch/step1-a/mutations_review_1110.log`): pre-build ok; CTRL red (56); **18 of 18 CAUGHT, each by the
  test written for it** — SM-R1a (the not-found start's mark deleted) → ST18 (review); **SM-R1b** (the PRE-REVIEW
  rule put back: a jumped start marks only what `find(goal)` places) → ST18 (review); SM27b (RM8: the label read
  before the time) → ST15 (d) (review: the label first after the deadline); RM1a (memory 31) and RM1b (33) →
  ST24 (the 32-label edge, both ways); RM2 (a jumped step's label not remembered) → ST24; RM3 (name/cause not
  reset) → ST15 (d); RM4 (`if (true)`) and RM5 (the origin's exit not copied) → ST15 (d) (review: after an
  EARLIER carried-on failure); RM6 (the verdict not copied) → ST15 (d) (review: a failed MECHANISM step); RM7r
  (the cancel put back) → ST15 (d) (review: a stray Tier-3 motion); RM9 (`>= 9`) and RM10 (index 8 reads
  "8") → R25; RM10t (the tester's twin) and RM11 (the amber rule an exact match) → 23c; RM12t (the tester's
  line-3b word for a motion failure) → 23c; RM12c (the competition's) → R25 and R23. None broke its build,
  none was killed by the cap or the timeout. After the run: the mutated files `cmp`-equal to their backups,
  "final restore build: ok".
- 2026-11-10 ~22:00 EST — **every gate re-run on the final tree** (the binary removed; the briefing generated, a full
  build, the briefing generated again, a build): the suite through `suite_run.py green` — "green — 3004 cases,
  2957048 assertions" (2961 at the base + 43); the ARM header gate — 240 headers in one TU, `-Werror`, exit 0;
  the two CI greps (pros/ outside hal/pros, shulib/sim outside sim) empty; `api_doc_tool` self-test OK,
  check-fresh exit 0, check-examples "665 quoted lines, all verbatim (13 source files); 6 listings ...",
  check-coverage "5524 public entities across 197 headers, all documented", check-removability clean;
  `doc_staleness_audit.py` self-test OK and clean; `briefing_status.py` self-test OK and check "generated block
  current" (the briefing: Suite 3,004 / 2,957,048, HA 204 registered, F6 50 / F8 86 / F10 70); `src_build_gate.py
  self-test` "OK (269 detector cases, each against a real build)", then `git diff -- Makefile src/ | grep -c
  '(self-test plant'` = 0, `check` "PASS (all 7 builds link ...)"; the binary rebuilt after the self-test (it
  touches `src/`) and the suite re-run: green, 3004. §7's greps: route.hpp / path_runner.hpp / service.hpp diff 0
  lines; no `_PIN(` line removed from any test; `timeIsUp()` reads `running()` first (routine.hpp :1295);
  `auton %.15s: %.27s` still at competition_screen.hpp :556; `src/` diff 4 files.
  **The prose review for this resumption (RESUMING rule 4), run before this line:** no public document, header
  comment or test changed today — only this log and the scratch driver — so no reader-facing sentence can have
  gone stale; within this log, `grep -n "below"` finds the review section's five "result(s) below" promises
  (SM27b, RM1a/b, RM9/10/10t, RM11, RM7r), each now answered by the mutation entry above; the "simulator" grep
  of this log finds only the earlier entries quoting §3.3's grep. **Open for the coordinator / the team lead:**
  finding 2's recommended seventh field `firstFailedExit` (not built: public surface beyond the answered brief);
  finding 12's `build-order.md` STEP1 line and any HANDOFF line (the merge's). **The fix round is complete. Not
  committed.**

### Verification-round findings and their disposition (2026-11-10, late evening)

A fresh verifier returned two findings (both minor). Each was re-checked against the code before it was
answered. **Every diff is still against `ef6162e`.**

1. **The finding-1 fix was pinned for the arm move only — REAL test gap, FIXED.** Re-checked: the five starting
   forms each have their own not-found path (`set()`, `positionStep`, `homeStep`, `powerStep`, `speedStep`, each
   `return startNotOnThisRobot(...)` after its own `mechs().find(...)`), and ST18 (review) reached only
   `positionStep`. The verifier's N1–N4 (one form put back to `notOnThisRobot`) are exactly SM-R1a's deletion done
   one form at a time, so they survived. New **"ST18 (verify): EVERY starting form that did nothing — another
   robot's declaration — marks this robot's same-named mechanism"** (test/mechanism_auton_steps_test.cpp). Each
   form misses robot x's same-named mechanism its own way, because each `find` overload checks something
   different (mechanism/service.hpp :1767-1816): `set(hoodB["tilt"])` is a position name robot x's hood lacks
   (`find(PneumaticGoal)` checks the position's name); `startSpinUp(flywheelB["far"])` is a preset robot x's
   flywheel lacks (`find(SpeedGoal)` checks the preset); `startRun(rollerB, ...)` is a **Power named "flywheel"**,
   another kind. `find(const Power&)` matches by name among the power slots, so a Power named like robot x's
   intake WOULD find it, and the verifier's "a Power named like robot x's intake but declared differently" could
   not reach the not-found path. `startHome(liftAsArm)` is an **Arm named "lift"** (`find(const Arm&)` wants
   degrees and robot x's lift is in inches). The verifier's "another robot's same-named lift" would find robot x's
   lift for the same reason. Each form runs twice:
   - **Alone.** The pre-fix wait THREW on its precondition. The case checks: no throw, `MechanismNotStarted`,
     `stoppedAt` 2, `failedOn` 1, `firstFailedCause` MechanismStepFailed, `completed` 0, `mechanismVerdict` and
     `lastMechanism()` Running, the "another robot's declaration" MECH line once, nothing commanded (the hood's
     two lines, the flywheel's motor, both lift motors) and the full RTN stop line.
   - **After an older SUCCEEDED step on that mechanism.** The pre-fix wait answered that older verdict. The case
     checks: `completed` 1 (never 2), `stoppedAt` 3, `MechanismNotStarted`, `lastMechanism()` Running, the RTN
     line.

   Mutations **SM-R1c** (`set`), **SM-R1d** (`speedStep`), **SM-R1e** (`powerStep`) and **SM-R1f** (`homeStep`),
   the verifier's N1–N4 exactly, were added to `scratch/step1-a/mutate.py`. The driver before this round is kept
   as `mutate_v3_before_verify_2026-11-10.py`. Run `scratch/step1-a/mutations_verify_1110.log`: the pre-build
   (a full `make shulib_tests`, which merges header deps) was ok and CTRL was red (57 cases). **SM-R1c, SM-R1d,
   SM-R1e and SM-R1f were each CAUGHT by "ST18 (verify)" alone**, its assertions red (no build break, no cap, no
   timeout). SM-R1a and SM-R1b were re-run on this tree and are still caught: SM-R1a by ST18 (review) and
   ST18 (verify), SM-R1b by ST18 (review). "final restore build: ok". Suite: 3005 cases (3004 + this one).

2. **routine.hpp's STEP1-A banner still said "API 2.5 additive" — REAL, FIXED.** :153 now reads
   `(chunk STEP1-A, additive; pinned)`, and docs/api is regenerated (routine.md :1347). The finding-13 grep is
   widened to the WHOLE diff (`git diff ef6162e -- include src test docs README.md`, plus the untracked new test
   files). It finds three "API 2.5" lines. All three are MECH-A4's own, and true: the context constructor's
   `///` (:370 of the diff), the roadmap's F10 row "appended at MECH-A4 (2026-10-22, API 2.5)", and
   routine_signature_pin_test.cpp :353. No STEP1-A sentence names a version.

**Found while answering finding 1 (same root, a doc sentence), FIXED.** Five sentences said a start "did nothing
because it named another robot's preset or position" (or "a preset, units or position"). The new test shows two
more ways in: a different KIND (a Power named like a flywheel) and different UNITS (an Arm named like a lift). The
sentences now say "a different kind or units, or a preset or position this robot's same-named mechanism lacks".
They are: routine.hpp's banner rule 2 (:195-198), the `waitUntilDone` `///` (:863-864), the `markNotStarted`
comment (:1057), guide 09 :272-273, guide 11 :370-371 and the changelog's `MechanismNotStarted` bullet.
`notOnThisRobot`'s own MECH line already said "a different kind, units, preset or position". docs/api is
regenerated.

**A process note, for the record.** The three routine.hpp comment edits above were first made while the mutation
run was going. The driver backs up the file before each mutant and restores it after, so its last restore put the
pre-edit comments back. The SM verdicts are unaffected: they differ only in comments, and every one was read
against the code it names. The edits were re-applied after the run (checked: `grep -c 'return
startNotOnThisRobot('` = 5) and before every gate below. The banner's "additive" fix was made before the run
started and survived. The src files the verifier's own mutation run (`verify/vmut.py`) touched at 22:14-22:15 are
`cmp`-equal to the latest backups: competition_run.hpp, auton_tester_panel.hpp and auton_tester_run.hpp.

**The prose review for this round (RESUMING rule 4), run before this entry was written.** Over the whole diff and
the untracked tests, "API 2.5" / "Since 2.5" finds only the three MECH-A4 lines in finding 2. "preset or
position" / "preset, units or position" / "units or position" over guide, cookbook, the changelog,
include/shulib/chassis and docs/api/routine.md finds only the new wording and `notOnThisRobot`'s (true).
"did nothing because" finds guide 09 :272, routine.hpp :195, :328 and :744. :328 and :744 say only "its goal is
another robot's", which is true and needs no list. Guide 11's sentence wraps across lines; it was read and is the
new wording. This log's earlier entries (finding 1 of the review round: "a preset this robot's same-named mechanism
lacks") are history and are not rewritten.

**Gates on the final tree, all re-run after the routine.hpp re-edit.** Order: docs/api generated, the binary
removed, the briefing generated, a full build, the briefing generated again, a build.
- **The suite** through `suite_run.py green`: "green — 3005 cases, 2957132 assertions".
- **ARM header gate:** 240 headers in one TU (`scratch/step1-a/all_headers_verify.cpp`), `-Werror`, exit 0.
- **The two CI greps:** both empty.
- **`api_doc_tool`:**
  - self-test: OK
  - check-fresh: exit 0
  - check-examples: "665 quoted lines, all verbatim"
  - check-coverage: "5524 public entities across 197 headers, all documented"
  - check-removability: clean
- **`doc_staleness_audit.py`:** self-test OK, and clean.
- **`briefing_status.py`:** self-test OK, and check "generated block current".
- **src gate:** see the next entry.
- **`src_build_gate.py`:**
  - self-test: "OK (269 detector cases, each against a real build)".
  - Then `git diff -- Makefile src/ | grep -c '(self-test plant'` = 0.
  - check: "PASS (all 7 builds link ...)" (`scratch/step1-a/srcgate_{selftest,check}_verify.log`).
- **After the self-test** (it touches `src/`), the binary was rebuilt and the suite re-run: green, 3005 cases,
  2957132 assertions. The briefing check is still current.
- **Open for the coordinator / the team lead, unchanged:** the review round's finding 2 (`firstFailedExit`),
  and finding 12's build-order / HANDOFF lines.

**The verification round is complete. Not committed.**

## STEP1-A — the coordinator's merge verification *(2026-11-11)*

- **Squash** of `step1-a` (base `ef6162e`) onto `e458dc6` (CAL1 complete). Conflicts resolved by hand: the register's Held line
  (HA-216 registered by STEP1-A, HA-217..HA-218 held for STEP1-B; MECH-S-A's HA-219..HA-223 registered, HA-224..HA-227 held),
  its status (7 of 209 settled) and its "added" paragraphs (STEP1-A's after MECH-S-A's, "With it, 209"), the changelog
  (STEP1-A's entry at the top of `## API 2.5`, dated "2026-11-11 (built 2026-11-05)"), the generated `docs/api/` pages and
  briefing. REMAINING-WORK row 20c: STEP1-A LANDED 2026-11-11; a roadmap "Now" paragraph added.
- **Gates on the merged tree** (`.claude/coordinator/scratch/step1_a_merge_verify.sh`, every binary capped): the suite green;
  `src_build_gate.py` self-test OK and check PASS (all 7 builds link); the ARM header gate CLEAN (243); the doc gates and the
  briefing check clean; the compile-refusal harness 258 of 258; no plant, no stray root file.
- **The coordinator's own mutations** (`muts_step1_a_main.json`): **4 of 4 RED** — MR1 a loosened tolerance keeping the robot's
  settle speed, MR2 the deadline never a cut, MR3 a cancelled step obeying `carryOn`, MR4b a tolerance of 0 loosening to 0
  instead of meaning the robot's own (MR4, `>` → `>=` at `tolerance == maxError`, was dropped before running as EQUIVALENT:
  `rate·t/e == rate` there). The restored tree relinked and ran green.
- **STEP1-A LANDED.** Nothing has run on a robot. STEP1-B (building on a snapshot of STEP1-A) re-reads this tip at its merge;
  CURVE1-A builds on it.
- **Follow-up (2026-11-11, the coordinator's own miss):** the squash `196cdfc` landed WITHOUT the minor bump the brief (§1.1
  :434) assigns STEP1-A — the builder's delta 1 left the number to the merge, and the merge forgot it. STEP1-A appends to the
  FROZEN rows F6 and F10 (version.hpp's policy: additive growth of a frozen surface bumps the minor), so `kApiMinor` 5 → **6**,
  `kApiVersionString` "2.6", its history paragraph; the changelog's STEP1-A entry now sits under a new `## API 2.6` heading
  and says so; guide 09's history line names 2.6. STEP1-B (brief :552) and CURVE1-A take the next numbers at their merges.
