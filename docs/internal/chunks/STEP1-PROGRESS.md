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

## STEP1-B

**Run:** STEP1-B only — triggers partway through a drive (§1.2: `atPercent`, `afterDistance`, `afterTurning` and the
handles they return; D6 a trigger fires only if its mark is on the drive; D7 percent is distance along the straight
line from the drive's start, the angle turned on a turn; a refused start lets the drive finish and counts as failed;
D8 a trigger written wrong is caught on its own line, every run, by the mechanism's own checks; HA-217..HA-218).
**Executor:** a subagent on Claude Opus 5.5, in the worktree `.claude/worktrees/step1-b` (branch `step1-b`).
**Base:** `ddab08c` — a snapshot of STEP1-A's finished build on `ef6162e`; every diff below is against it.
**Host only:** nothing here claims the robot. Never committed by the executor. Scratch:
`.claude/coordinator/scratch/step1-b/`.

### Log

- 2026-11-11 02:00 — log created before any code. Read RESUMING's standing rules; the brief's header, answers block, the Re-check amendments, D1–D8, §1.2, §1.3 (B1–B9 and the A rules they touch), §2's STEP1-B rows, §3.2, §3.3, §4, §6, §7, §8 in full; the rest by heading. Read STEP1-A's log above (its deltas 1–13 are the base this run builds on).
- 2026-11-11 02:05 — **code, first pass.** `mechanism/service.hpp` (D8 (a)): the four start commands' precondition
  blocks moved, unchanged and in the same order, into `checkRun` (returns the sensor slot `run()` uses),
  `checkMoveTo`, `checkHome`, `checkSpinUp` — each `const`, each placed directly above its command, each command's
  first line now the call. Verified by script: the file's whole sequence of `SHULIB_PRECONDITION` statements and
  message lines is byte-identical to the base's, in order (`diff` of the extracted lines empty). Nothing else in
  the file changed. `motion/leg_progress.hpp` (NEW, pure). `chassis.hpp`: `LegObserver`, the two observer
  overloads, the private decorator `ObservedLeg`, a banner section. `routine.hpp`: the "STEP1-B: partway
  triggers" block (the two handles `TriggerMark<Kinds>` / `ArmedLeg<Kinds>`, the three arms), its machinery
  block, its state (declared last), the door's armed check (`enterStep(name, armedLeg)`), `onFail`/`label`'s armed
  check, the destructor Warn, and `notOnThisRobot` split into its logging half (`logNotOnThisRobot`) and its
  recording half. Host build green with no new test: **suite green, 3005 cases** (the base count).
- 2026-11-11 02:10 — probe (`scratch/step1-b/probes/p1.cpp`, run capped): robot x's 40 in leg is 180 Running
  ticks, settles at 1.80 s at 99.39 % (0.24 in short; the leg's live length exactly 40.0, tolerance 0.5); the
  90° turn settles at 99.16 %; the motor-lag plant (kA 0.02) carries the leg to 100.53 % and its progress does
  NOT fall back below 100 % before it settles (so ST31's "once through an overshoot" pins the in-tick fire plus
  the settle that must not fire it again — said in the test). The first tick of a fresh rig is not live.
- 2026-11-11 02:40 — tests: `test/leg_progress_test.cpp` (6 cases), `test/chassis_leg_observer_test.cpp` (5:
  ST19's bit-identity on both plants, the call count and the exit tick, the progress from the tick's own pose
  and the effective tolerances, the boot wait, the overloads' validate), `test/routine_trigger_test.cpp` (17:
  ST9+ST13, ST9's equality edge, ST10+ST11+ST20, ST31, ST12, ST21, ST22+ST36, ST36+B5, ST29, ST33, ST33 (b),
  ST30, ST32, ST34, ST35 ×2, the handle-once rule). All green after three test-side fixes (each a test error,
  not code: the first tick of a fresh rig is not live; the carried-on line's wording; the lift needs more than
  2 s for 22 in while the robot drives).
- 2026-11-11 02:40 — more tests: `test/mechanism_service_test.cpp` +2 cases (D8 (a): each check function throws
  exactly what its command throws, every refusal and every accepted edge, and checking commands nothing);
  `test/f6_signature_pin_test.cpp` +7 pins (the two overloads, `LegObserver::fn` / `::context`, the aggregate
  shape, the two overloads' not-noexcept — F6 50 → 57); 11 compile-refusal TUs `test/compile_refusals/t01..t11`
  (ST14: move / run / set / home / spinUp after a mark, a discarded armed handle; ST34: inches on a turn,
  degrees on a drive, the two marks on one leg both ways, faceAway after a narrowing arm) — `ctest -R
  compile_refusal` 266/266 passed; `test/routine_trigger_example_test.cpp` (guide-09f, cookbook-07b, through
  the real runAuton).
- 2026-11-11 02:45 — docs: guide 09 (a new section, "Partway through a drive: triggers", the Stability note and
  the case list), cookbook 07 (a new recipe, "Start the lift partway through a drive", measured: 5.06 s vs
  6.45 s lift-then-drive — `scratch/step1-b/probes/p3.cpp`), cookbook README (one row, where it compiles),
  guide 10 (the advanced seam: one paragraph on the observer overloads), guide 18 (one sentence after the
  START forms), the changelog (top of `## API 2.5`), the roadmap's F6 row (the overloads, `LegObserver`, 57
  pins) and F10 row (the arms added, OPEN), the HA register (HA-217, HA-218 registered — the summary, the
  Held line, the table, a detail section; 206 rows), `runner/mechanism_routes.hpp`'s comment, REMAINING-WORK
  row 20c's status; `docs/api` regenerated; the briefing regenerated (binary removed first). **Suite: green,
  3037 cases** (3005 + 32).
- 2026-11-11 02:50 — **the prose review (RESUMING rule 4), run BEFORE this line was written.** §3.3's greps:
  `simulator` in this change's added guide/cookbook lines — 0; `ROUTINE STOPPED` — guide 16 :91/:187/:238 and
  guide 17 :259, each still true (no trigger changes a word); `branches on the verbs directly\|continue past` —
  routine.hpp :154 and guide 09 :218 (true: STEP1-A's wording), check.hpp ×3 (another meaning); the removability
  grep — empty; `partway\|mid-drive\|mid-leg` over `include docs/guide docs/cookbook` — every hit in this
  change's own lines true; **one STALE hit found and fixed: `runner/command_registry.hpp` :41 said a step that
  starts mid-leg is "the mechanism core's later work"** (comment only; edited after the mutation run, so no
  header changed under a mutant); cookbook 02 :215 / README :68 ("stopped partway", a trajectory), cookbook 04
  :45 ("held in place mid-leg"), guide 12 :72, `diag/sd_sink.hpp` :23 — another meaning, true. Rule 4's own
  greps, each changed file's name and concepts over `docs/*.md docs/guide docs/cookbook README.md`: the
  observer overloads — guide 10's verb headings (`moveTo(target, options)`, `turnTo(heading, options)`) still
  true (the plain verbs are unchanged), and guide 10's advanced-seam section gained the observer paragraph;
  the START forms — guide 18 :357-364 (one sentence added), guide 09's Stability note (one sentence added);
  `MechanismNotStarted` / "NOT STARTED" — guide 09's rule 2 (true: it lists the skip's and another robot's
  causes; the trigger's cause is in the new section), guide 11's RTN bullet (true) **and guide 11 had no line
  for the trigger's RTN lines: one bullet added** (each quoted line asserted in `routine_trigger_test.cpp`);
  `checkRun`/the start commands' preconditions — guide 18's step table and refusal lists (true: the
  behaviour is unchanged); "route marker" / "markers stay blocking" — guide 18's route section, cookbook
  nothing: true (F8 unchanged). `docs/guide/14-what-it-cannot-do-yet.md` — no mid-drive / trigger / partway
  line (grepped `trigger`, `partway`, `mid-`, `while.*driv`): nothing to change.

### STEP1-B's deltas from the brief (against the base `ddab08c`), each with its reason

1. **No version bump; the changelog entry is at the top of `## API 2.5`** (the brief: `kApiMinor` 6 → 7, "API
   2.7"). The coordinator's instruction for this run, as for STEP1-A (its delta 1). `version.hpp` untouched.
2. **`motion::LegProgress` has NINE fields, not four:** the brief's `live`, `fraction`, `along`, `turnedRad`,
   then `hasDirection` (the brief's "flag" for a zero-size leg), `length`, `sizeRad`, `positionTolerance`,
   `headingToleranceRad`. B3's settle rule needs, AFTER the leg, the leg's size from its LIVE start and its
   EFFECTIVE settle `maxError`; the decorator inside the Chassis is the only place that has both (the start
   pose it captured, `effectiveConfig(options)`), so it hands them over with every call and the Routine keeps
   the last. A copy of `effectiveConfig`'s rule in `routine.hpp` was the alternative — two places for one
   formula. `LegProgress` stays open to new fields (no pin covers its list, D1).
3. **The examples compile in a NEW file, `test/routine_trigger_example_test.cpp`** (cases `guide-09f`,
   `cookbook-07b`), not `test/cookbook_examples_test.cpp` (§3.2) — STEP1-A's delta 2's reason: the recipes need
   the MECH-A4 rig and robot x's words, and it keeps this run off a file other chunks edit.
4. **The handle types are named `Routine::TriggerMark<Kinds>` and `Routine::ArmedLeg<Kinds>`** (the brief names
   no type), the kinds mask a template argument (`detail::kTriggerDriveLegs` / `kTriggerTurnLegs` /
   `kTriggerAnyLeg`). Both `[[nodiscard]]`; not pinned (D1).
5. **A JUMPED or STOPPED leg marks each trigger's mechanism "the trigger never fired: its drive was skipped"**
   — B4's sentence ("the robot did not reach the mark") is for a leg that ran; ST32 asks for the mark, not the
   words.
6. **A trigger whose mechanism `find()` could not place fires ABSENT and also marks the same-named slot "its
   start did nothing: another robot's declaration"** — STEP1-A's delta 11, applied to the fire, so a carried-on
   wait answers MechanismNotStarted rather than an older verdict.
7. **D8's checks run at the ARM even when the leg will be skipped** (a stopped chain, a pending skip): D8's
   "caught on its own line, every run" read strictly (a bench run whose chain stopped earlier still shows the
   mistake). A blocking step in a stopped chain checks nothing (it returns at its door) — the one place the two
   differ.
8. **A handle drives ONCE** (a kept `ArmedLeg` called for a second leg is a precondition, "this trigger
   handle's drive already ran") — not in the brief; without it a second call ran a plain, trigger-less drive
   the author did not write.
9. **`checkRun` returns the `const Slot*` whose piece sensor the condition reads** (the brief: "a `const`
   check function the command calls first"): `run()` used that local after its preconditions; returning it
   keeps `run()`'s remaining body byte-identical.
10. **A mark past a SETTLED leg is said in its own Warn line** ("routine 'x': step N (driveTo): trigger after
    50.0 in on 'lift' did not fire: the drive was 40.0 in") — B3's "the drive step's line says so once": a
    drive that succeeds has no line of its own.
11. **Documents beyond §3.2:** guide 11 (one bullet: the trigger's RTN lines — a guide that lists the RTN
    lines and missed these would be incomplete the day they land), guide 18 (one sentence after the START
    forms), cookbook README (one row; where the recipe compiles), REMAINING-WORK row 20c's status; and
    `runner/command_registry.hpp`'s stale comment (prose-review hit). Guide 14: not edited (nothing in it).
12. **ST31's "crosses 100 % then settles back"**: measured, the motor-lag plant's estimate does NOT fall back
    below 100 % before the leg settles (100.53 % at its settle). The test pins what that plant can show: the
    100 % mark fires on its crossing tick inside the leg, and the settle does not fire it again (SM7b).
13. **ST29 uses the scoring deadline at T+1.00 s** (the park), mid-leg, through the real `runAuton`; the marks
    are the deadline tick's own progress and the tick before's (read from a Tier-3 run of the same leg in the
    same auton schedule) — so the equality lands exactly.
14. **Two tests the brief does not list:** the room (a fifth trigger refused, on robot y's six motor
    mechanisms, so B8 cannot be what refuses it), and the handle-once rule; and ST30 gained the WHOLE line
    (drive included), so SM32's backstop path and SM26's scheduler path are observable, with captures.
15. **`turnProgress` is `startHeading.errorTo(heading)` with the turn's sign** (the brief's formula): honest
    while the robot has turned under 180° from its start either way; a turn near 180° that overshoots past
    180° wraps negative — said in the header (a mark at or below 100 % is crossed before the wrap).
- 2026-11-11 03:00–04:10 — **the mutations, every one CAUGHT** (scratch `mutate.py`: exact replacements, each
  asserted to occur the stated number of times; the file backed up under `scratch/step1-b/bak/`; ONE full
  build first so the merged header-dependency list is complete, then `make shulib_tests/fast` per mutant; the
  CONTROL mutant first; each run capped `ulimit -v 4194304; timeout 600`; the file restored in a `finally` and on
  SIGTERM/SIGHUP/SIGINT and touched; a mutant whose BUILD fails is re-cut, never counted). The first full run
  (`mutations_run1.out`) found all 34 red but its red-test list was polluted by doctest's MESSAGE blocks
  (driver kept as `mutate_v1_redlist_polluted.py`), and `B9door` broke its own build (`-Werror=unused-parameter`)
  — re-cut as `if (!armedLeg && false)`, never counted. **The run of record is the re-run with the parser fixed
  (`mutations_run2.out`, `mutations.log`):** CTRL red (64 cases) first; then, each with the test that went red:
  **SM5a** (`>` at a percent) → ST9 (edge); **SM5b** (`>` at a distance) → ST9 (edge), ST29; **SM6** (settle
  rule on any end) → ST10+ST11+ST20, ST21; **SM7a** (fires every tick past the mark) → ST9+ST13, guide-09f,
  cookbook-07b (+6); **SM7b** (the settle re-fires a fired trigger) → ST9+ST13, ST31, guide-09f (+4);
  **SM8** (a blocking move accepted after a mark) → `compile_refusal_t01` red, the judge's own verdict
  "COMPILED" (the TU's refusal is the test's assertion); **SM13** (an unfired trigger marks nothing) →
  ST10+ST11+ST20, ST21; **SM14** (observer on the exit tick) → ST19 (call count; progress) (+4); **SM15** (the
  settle-short rule inverted) → ST22+ST36, ST36+B5; **SM23** (the moveTo seam on waitUntil) → ST19 (bit-identity;
  call count); **SM24** (the guard not asked) → ST29; **SM26 on top of SM32 (run)** → ST30, ST35; **SM28** (the
  fire through `startedMechanism`/`recordStop`) → ST33, ST33 (b), ST9+ST13, ST10+ST11+ST20, ST12, ST22+ST36,
  guide-09f, cookbook-07b, the handle-once case; **SM31** (the settle fires every unfired trigger) → ST22+ST36,
  ST36+B5; **SM32r / SM32m / SM32h / SM32s** (the arm skips run's / moveTo's / home's / spinUp's checks) → ST30
  each (SM32r also ST35's D8 case); **SM37** (the kind mask ignored) → `compile_refusal_t07..t11` all five red,
  each "COMPILED"; **SM38** (an arm's throw leaves its triggers armed) → ST35, ST35 (room); **SM38b** (a refused
  percent leaves the modifiers) → ST35; **SM38c** (an armed leg's exit leaves its triggers armed) → ST35, ST35
  (B9 (4)), ST32, ST33 (+7); **B9door** (re-cut: a step while armed) → ST35; **B9onFail** → ST35; **B8** (two on
  one mechanism) → ST35; **ROOM** (`kMaxTriggers` 5) → ST35 (room); **B6** (a refused start on a settled leg
  ignored) → ST33; **PAST** (a mark past the leg not said) → ST22+ST36, ST36+B5; **DTOR** (the destructor Warn
  removed) → ST35 (B9 (4)); **JUMPMARK** (a jumped leg marks nothing) → ST32; **LP1** (percent as distance to
  go, D7 (b)) → leg progress ×2, ST31; **LP2** (the turn's sign ignored) → leg progress (turn), ST19 (progress);
  **LP3** (the zero-length edge `>` for `>=`) → leg progress (edge); **SVCDRIFT** (`stalls` checked by `run()`
  only — a check that drifted from its command) → D8 (a) checkRun, ST30. **35 of 35 counted mutants CAUGHT; none
  killed by the cap or the timeout; "final restore build: ok".**
- 2026-11-11 04:15 — **SM26 on top of SM32, observed the two ways the brief names** (`scratch/step1-b/
  observe_sm26.py` → `observe_sm26.log`; ST30's whole-line case, its CAPTUREs): **SM32r alone** — the line
  THROWS the service's own message, but only AFTER the leg: `motionsStarted` 1, the leg's exit 1 (Settled), 0
  PRECONDITION faults — B7's backstop. **SM26 + SM32r** — the line throws NOTHING; the leg's exit 3 (Cancelled),
  1 PRECONDITION fault, `steps` 2 — the scheduler's task-boundary catch made a fake cancelled drive. Restored,
  rebuilt ("final restore build: ok").
- 2026-11-11 04:20 — after the mutations: every mutated file `cmp`-equal to its latest backup (routine.hpp,
  chassis.hpp, service.hpp, leg_progress.hpp); `git diff -- Makefile src/ | grep -c '(self-test plant'` = 0.
  `runner/command_registry.hpp`'s stale comment fixed (the prose-review hit; comment only); `docs/api`
  regenerated; the binary removed, the briefing generated, a full build, the briefing generated again, a build.
  **Suite through `suite_run.py green`: "green — 3038 cases, 2960186 assertions"** (3005 at the base + 33).
- 2026-11-11 ~05:00 — **gates on the final tree** (after the mutations and the restores):
  - **The suite** through `suite_run.py green`: "green — 3038 cases, 2960186 assertions" (re-run after the src
    gate's self-test, which touches `src/`: the same).
  - **ARM header gate:** 241 headers in one TU (`scratch/step1-b/all.cpp`), `-Werror`, exit 0.
  - **The two CI greps** (pros/ outside hal/pros; shulib/sim outside sim): both empty.
  - **`api_doc_tool`:** self-test OK; check-coverage "5552 public entities across 198 headers, all documented";
    check-fresh exit 0; check-examples "683 quoted lines, all verbatim (14 source files); 6 listings of
    whole-run files, each one unbroken run"; check-removability clean.
  - **`doc_staleness_audit.py`:** clean; its self-test OK. **`briefing_status.py`:** check "generated block
    current"; self-test OK (F6 pins 57, F10 70).
  - **`ctest -R compile_refusal`:** 266/266 passed (255 + this run's 11).
  - **`src_build_gate.py`:** self-test "OK (269 detector cases, each against a real build)"; then `git diff --
    Makefile src/ | grep -c '(self-test plant'` = 0; check "PASS (all 7 builds link ...)".
  - **§7's own greps** (BASE = `ddab08c`): route.hpp / path_runner.hpp diff 0 lines; no existing pin line removed
    from either pin file; the F10 pin file not touched (no arm pinned); `git diff --stat $BASE -- src/` empty;
    the two observer overloads go through `runBlocking` (chassis.hpp :417, :429) and `sched_.waitUntil` stays in
    `wait()` and `waitUntil()` only (:516, :581); `service.hpp` read: only the four check functions extracted,
    the precondition statements and messages byte-identical in order (scripted diff, above).
  - **By hand (§7):** the decorator forwards `name()` and calls the observer only after an inner tick that
    returned Running (chassis.hpp :671-:706); `fire()` (routine.hpp :2115) reaches no `skipIfStopped`,
    `recordSuccess`, `recordStop`, `startedMechanism`, `awaitMechanism`, `notOnThisRobot`'s recording half or any
    `cancel()` — only the slot's command, `noteStarted`, `markNotStarted` and `logNotOnThisRobot` (SM28 is the
    mutation that proves the difference matters).
- **For the coordinator at merge:** `mechanism/service.hpp` — this run's edit is the four check functions only
  (main's MECH-S-A cap is elsewhere in the file; TUNE1-A's `rederive` lands beside them); a precondition a later
  chunk adds to `run` / `moveTo` / `home` / `spinUp` goes INSIDE its check function. `docs/hardware-assumptions.md`
  — this run's lines say "With them, 206" and "7 of 206 settled" against the base (main's MECH-S-A added
  HA-219..HA-223 since: the totals are the coordinator's to re-add at merge). `mkdocs.yml` — one nav line (Leg
  progress). The changelog entry sits at the top of `## API 2.5`. **STEP1-B's build is complete. Not committed.**
- 2026-11-11 07:45 — **RESUMED** after the 5-hour usage limit stopped the run (~04:30). Read this log. The work
  above is complete through "STEP1-B's build is complete"; the file times say the gates finished by ~04:26 (the
  "~05:00" stamp on the gates entry above is approximate — the last gate log, `srcgate_check.log`, is 04:25).
  Audit on resuming: `git diff -- Makefile src/` empty (no self-test plant; `grep -c '(self-test plant'` = 0);
  `git diff --stat ddab08c -- src/ Makefile` empty; the four mutated files `cmp`-equal to their latest backups
  (routine.hpp ← JUMPMARK, chassis.hpp ← SM23, leg_progress.hpp ← LP3, service.hpp ← SVCDRIFT); no mutation
  driver or test binary running. Nothing redone; nothing reverted. Next: a FRESH host build from source
  (`build/test` removed whole, so no object or link from the killed session survives), the suite, and every gate
  again on that build.
- 2026-11-11 07:46–08:30 — **the fresh build and every gate again, on the resumed tree** (no source, test or doc
  line changed on resuming; logs `scratch/step1-b/resume_*.log`): `build/test` removed whole, configured, the
  briefing generated (no binary present), a full build (0 warnings), the briefing generated again, a build.
  - **The suite** through `suite_run.py green`, capped: "green — 3038 cases, 2960186 assertions" — the same after
    the src gate (rebuilt, re-run).
  - **ARM header gate:** 241 headers, `-Werror`, exit 0. **The two CI greps:** both empty.
  - **`api_doc_tool`:** self-test OK; check-coverage "5552 public entities across 198 headers, all documented";
    check-fresh exit 0; check-examples "683 quoted lines, all verbatim (14 source files); 6 listings of
    whole-run files, each one unbroken run"; check-removability clean.
  - **`doc_staleness_audit.py`:** clean; self-test OK. **`briefing_status.py`:** check "generated block
    current"; self-test OK.
  - **`ctest -R compile_refusal`:** 266/266 passed (t01..t11 among them).
  - **`src_build_gate.py`:** self-test "OK (269 detector cases, each against a real build)"; `git diff --
    Makefile src/ | grep -c '(self-test plant'` = 0 and `git status -- Makefile src/` empty; check "PASS (all 7
    builds link ...)"; the plant count again 0.
  - The mutation run of record (03:20–03:54) is not repeated: every mutated file is `cmp`-equal to its backup,
    so the tree it judged is the tree built here.
  **STEP1-B's build is complete (re-confirmed on a fresh build). Not committed.**

### STEP1-B review findings and their disposition (2026-11-11)

Three reviewers (correctness, tests, docs) returned 22 findings. Each one was re-checked against this
worktree, fixed at the root or answered with evidence, and the fixes' mutations were observed on fresh
links. **History of the round:** a first fix run (~09:00–09:45) made every code, test and doc edit below. It
was stopped by the 5-hour usage limit at ~09:45, BEFORE its mutation driver (`scratch/step1-b/mutate_fix.py`)
ran: there is no `mutations_fix.log` from that run and no `bak/*.fix-<id>.orig`. It logged nothing. **Resumed
2026-11-11 22:39** (a `date` reading). Audit on resuming: `routine.hpp` and `chassis.hpp` are `cmp`-equal to
the run's own pre-mutation copies (`bak/*.fix-pre`, 09:45). `leg_progress.hpp` and `service.hpp` are
`cmp`-equal to their last backups (`LP3`, `SVCDRIFT`). `git diff -- Makefile src/` is empty, with 0
`(self-test plant` lines. No driver of mine was running (the `mutate.py` process seen running is the
coordinator's own, in the main tree). So no mutant or plant was left applied. I read the stopped run's edits
in full (the diffs against `bak/routine.hpp.JUMPMARK.orig` and `bak/chassis.hpp.SM23.orig`, the pre-round
sources) and found them correct. Nothing was reverted. **The binary was rebuilt from source:** `build/test`
was removed whole, reconfigured, the briefing generated with no binary, a full build (269 objects, 0
warnings), the briefing generated again, then a build.

**The findings, each with its disposition.** "RFx" is this round's mutant id (`mutations_fix_run1.out`,
`mutations_fix_run2.out`, `mutations_fix.log`). Each line names the test that went red and the number of
objects the mutant's build recompiled.

1. **(correctness) A fire on the tick the scheduler's FAULT abort ends.** REAL. Reproduced in the stopped
   run's test: an ODO_STUCK latched one pace before the 60 % tick. **Fixed at the root**, in the decorator,
   so a Tier-3 observer gets the same promise. The new read-only accessor
   `MotionScheduler::pendingAbortFault()` returns the scheduler's own `newlyRaisedAbortFault()` while a
   motion is active (the same test `tick()` applies right after a Running tick). `ObservedLeg::tick` calls
   nothing when it is set. Could anything raise an abort-mask fault between the check and the scheduler's
   own test? The only code between them is the fire's start command. Under D8 (a) that command cannot throw,
   and its only begin verdicts are ABSENT, REFUSED and NOT_HOMED; the service raises a fault only for
   STALLED (`service.hpp` `finishStep`). The docs (chassis.hpp banner and doc comments, routine.hpp,
   guides 09 and 10, the changelog) now say "nor a tick the fault abort ends". Test: "STEP1-B (review):
   NEVER on a tick the scheduler's FAULT ABORT ends" (it also checks that the accessor reads None with no
   motion). **RF1** (the check removed): CAUGHT, red in that case (107 objects).
2. **(correctness) A kept handle drives ANOTHER line's leg.** REAL. **Fixed at the root:** every trigger line
   gets a number (`armGen_`, from `newLine()`), and both handles carry it. `requireLine(gen)` refuses three
   cases: a handle whose leg already ran, a handle from an earlier line, and a handle whose newest mark has
   no start yet. Each refusal clears first (B9 (3)). `arm()` refuses a mark that was already started or comes
   from an earlier line. The kind mask can no longer be bypassed, so `warnMarkPastLeg`'s wording, which
   reads the mark's kind, now always matches the leg's kind (said in its comment). Test: "a handle acts for
   ITS line only". **RF2 / RF2b** broke their own build (`-Werror=unused-parameter`, 8 objects) and are
   **not counted**. Re-cut as **RF2r** (`other = ((void)gen, false)`) and **RF2br** (`((void)gen, true)`):
   both CAUGHT, red in that case (43 objects each). **RF2c** (a mark on the handle that was never started)
   was CAUGHT in the same case (103 objects, after RF2b's failed build).
3. **(correctness) With two failing triggers, the step named the one ARMED first.** REAL. **Fixed:** each
   fire takes a sequence number (`fireOrder`, from `fires_`). The failing trigger is now the FIRST TO FIRE
   of those that failed, and `lastFired` is the latest to fire, both by that number. This is the brief's
   "the first such verdict" read in time. Test: "two failing triggers": a 90 % REFUSED trigger armed first
   and a 20 % NOT_HOMED trigger; the step names NOT_HOMED at 20 %. **RF3** (arming order): CAUGHT (43).
4. **(correctness) `r.atPercent(60);` alone was silent at run time.** REAL. **Fixed at the root, not in the
   wording:** a written mark is PENDING (`pendingMark_`) until its start form arms it. `requireUnarmed()`
   refuses with its own sentence, so the next call on the Routine refuses. The destructor's Warn covers a
   mark still pending ("a trigger's mark was written but no start and no drive followed"). Guide 09's
   sentence is now true for both kinds of unfinished line, and guide 11 lists the new Warn. Test: "a line
   that ENDS AT ITS MARK". **RF4** (the next call is not refused): CAUGHT (43). **RF4b** (the destructor says
   nothing): CAUGHT (43).
5. **(tests) B3's "unless the guard has expired" on the settle fire was untested.** REAL. New test "NEVER at
   an arrival ON the deadline tick", through the real `runAuton`. A probe auton reads the 40 in leg's settle
   time. The mark auton runs `atPercent(100)` (which fires only at the settle, because the leg settles
   0.24 in short) with the scoring deadline EXACTLY on that settle tick, and then 5 ms later. The test
   asserts that the leg SETTLED at the probe's time, that the guard had expired at the leg's end only in
   the first case, and that the intake started 0 times and then 1 time. **RF5** (= the reviewer's R3,
   `if (settled)`): CAUGHT (43).
6. **(tests) Armed `reverseTo` / `faceAway` had no behavioural test.** REAL. New test: each armed leg and its
   plain twin, run on twin rigs, end bit-identical (pose, heading, record name, end time). The trigger fired
   once, and the absolute heading is checked: −90° facing away from (0, 40), and +x when backing to
   (−20, 0). **RF6a / RF6b** (= R11 / R12): CAUGHT (43 each).
7. **(tests) B9 (2) was tested only for `r.atPercent`.** REAL. ST35's `armThen` gained `r.afterDistance(5_in)`
   and `r.afterTurning(10_deg)`. **RF7a / RF7b** (= R8 / R9): CAUGHT, red in ST35 (43 each).
8. **(tests) The turn mark's equality edges were unpinned.** REAL. ST9 (edge) gained a DEGREES branch: a
   mark equal to `turnedRad` at tick k, a value an `Angle` stores unchanged (asserted), fires at tick k.
   ST22 gained the turn's bound EXACTLY, `size + tolerance` from a Tier-3 run of the same turn. That bound
   fires, and `nextafter` one ulp past it does not. The test also asserts that the turn never reaches the
   bound inside a tick, so only the settle path is judged. **RF8a** (`>` for `>=`, = R2): CAUGHT, red in
   ST9 (edge). **RF8b** (`<` for `<=`, = R1): CAUGHT, red in ST22 + ST36 (43 each).
9. **(tests) The report choice with more than one trigger was untested (R5, R14, R30).** REAL. The "two
   failing triggers" test covers three cases. (a) Two failures: the FIRST TO FIRE is named. (b) One REFUSED
   trigger, then one that starts: the verdict and `lastMechanism()` are the refusal. (c) A SUCCEEDED
   `r.move` earlier, then a trigger that starts: `lastMechanism()` is Running, not the older Succeeded.
   **RF3b** (the last failing trigger, = R14), **RF3c** (`lastFired`'s report although one failed, = R30)
   and **RF3d** (`lastFired` never set, = R5): all CAUGHT (43 each).
10. **(tests) The "did not fire" Warn was never shown silent on a leg that did not arrive.** REAL. ST10's
    timed-out leg also carries `afterDistance(50_in)` (past the 40 in leg), and the test asserts there are
    0 "did not fire" lines and that the arm never started. **RF10** (= R4, the `settled &&` guard removed):
    CAUGHT, red in ST10 + ST11 + ST20 (43).
11. **(tests) No kind refusal for an armed `moveTo`, `reverseTo` or `face`.** REAL. New refusal TUs:
    `t12_degrees_mark_on_move_to`, `t13_degrees_mark_on_reverse_to` and `t14_inches_mark_on_face`, each with
    its `// expect:` text. **RF11a/b/c** (= SA1/2/3, each `static_assert(true, ...)`): each CAUGHT by its
    OWN TU. The judge's verdict was "COMPILED", which is the refusal test's own assertion (43 each).
12. **(tests) Near-equivalent survivors R7, R10, R17, R18 and R39.** ACCEPTED as equivalent or defensive,
    with these reasons, and no test added. **R10** (the `legProgress_` reset before an armed leg) matters
    only if a leg SETTLES before any observed tick. The shipped settle configs hold the error for 0.1 s
    (`motion_config.hpp`, `settleTime = 0.1`; `MotionOptions` never changes it), which is about 10 Running
    ticks, and each of those calls the observer and overwrites `legProgress_`. The reset is a guard for a
    robot file that sets `settleTime` to 0. **R7** (`fire()`'s `markNotStarted` for a not-on-this-robot
    trigger) matters only when another robot's SAME-NAMED declaration of a different kind is found where
    this robot's is not (routine.hpp's ABSENT comment names this case). The suite has no case that
    builds it, which is why the reviewer's R7 survived. **R17** (the second `rethrowHeldError`) is unreachable under D8 (a), because arming ran the
    command's checks. **R18** (`sameLabel` vs pointer equality) is equivalent while one declaration object
    is used. **R39** (`ObservedLeg::cancel` not forwarded): the scheduler ends a cut leg itself (ST10's cut
    still ends Cancelled). Forwarding is the decorator's contract ("every member forwarded"), kept by
    reading. If the coordinator wants any of these pinned, R7 and R10 are the two with a reachable,
    test-shaped case.
13. **(tests) The F6 pins were never mutation-checked in the record, and no object counts were logged.**
    REAL. `mutate_fix.py` adds a `pin:` kind. Each pin mutant compiles ONLY the pin file's object (its stale
    object is removed first) and is counted as caught only when the error is that pin's OWN
    `static_assert` text. **P1** (observer `moveTo` takes `const LegObserver&`), **P2** (the same for
    `turnTo`), **P3** / **P4** (`noexcept` added; the `SHULIB_F6_NOT_NOEXCEPT` pin at :256), **P5**
    (`fn` takes the progress by value) and **P6** (`context` retyped): all CAUGHT, each by its own pin's
    message (pin file :371, :378, :256, :256, :385, :390). Every mutant line in this round records the
    objects its build recompiled: routine.hpp's dependents are 43 and chassis.hpp's are 107, which matches
    the reviewers' count.
14. **(docs) The briefing said 8 hardware assumptions were settled.** REAL. HA-218's index row now says
    "the leg ended within its tolerance", with no "settled". The register regex
    (`^\|\s*HA-\d+.*\bsettled\b`) now matches exactly HA-94/95/96/97/99/100/101. The briefing, regenerated
    on the fresh binary, reads "206 registered, **7 settled**". **For the coordinator:** the tool's regex
    (`tools/briefing_status.py:169`) counts any row that contains the word "settled". It is not this run's
    file, and it will miscount the next row that says "settled" in prose.
15. **(docs) A shape pin closed LegObserver's member set.** REAL: a freeze the team lead never ruled. The
    shape pin was **removed**. The `fn` / `context` type pins and the two overload pins stay, and the pin
    file says why the shape is not pinned (F6's banner: the member set stays additive-open). F6 has 56 pins
    (the roadmap's F6 row and the briefing read 56). The suite count is unchanged by this, because a pin is
    a `static_assert` and not a case.
16. **(docs) Guide 14 implied the trigger arms are locked.** REAL. One clause added: the three arms and their
    two handles are unpinned like the START forms they lead to, and they ride only the six drive and turn
    legs. This is guide 14's ONE change from STEP1-B. The 02:50 entry's "Guide 14 ... nothing to change"
    was wrong because its grep looked for trigger words and not for this list of pieces.
17. **(docs) "Any drive or turn" overstated where a trigger can go.** REAL. Guide 09, guide 14 and the
    changelog now name the six legs (`driveTo`, `reverseTo`, `moveTo`, `turnTo`, `face`, `faceAway`) and say
    that `strafeTo` and `followTrajectory` take no trigger yet. On the compiler's bare "no member named
    strafeTo": that is the error a handle with no such member gives. A friendly refusal would need a
    deleted member whose message names the reason. That is not done here (§5 rules such legs out until a
    later chunk), and the guide now says so in words.
18. **(docs) The start forms' `name` was silently ignored.** REAL. **Fixed in code, not documented away:**
    the name now names the trigger in every line that names it ("its trigger 'raise' at 60 % on 'lift'",
    "trigger 'feed' after 60.0 in on 'intake' did not fire"), through `describeTrigger`. The drive stays the
    step. This is documented on TriggerMark, in routine.hpp's banner, guide 09, guide 11 and the changelog.
    Test: "a start form's NAME names the trigger in its lines". **RF18** (`startMove`'s name dropped):
    CAUGHT (43).
19. **(docs) Pace rule 7 was not addressed.** Acknowledged here as an amendment to delta 11. STEP1-B's public
    documents are guide 09, guide 10, guide 11, guide 14, guide 18, cookbook 07, the cookbook README, the
    changelog, the roadmap and the HA register: TEN, against RESUMING rule 7's "about five". **Why they stay
    in this one change rather than splitting:** each one past §3.2's six is a one-line or one-bullet truth
    fix (guide 11's RTN bullet, guide 18's sentence, the README row, guide 14's clause). Splitting them off
    would leave a public page false on the day the triggers land, which rule 1 ("docs move in the same
    change") forbids. §3.1 reasoned STEP1-A's overage the same way. The team lead may still rule a split.
20. **(docs) HA-217 and HA-218 sat before HA-216.** REAL. Moved: the index now reads HA-215, HA-216, HA-217,
    HA-218.
21. **(docs) The log said 17 cases in `routine_trigger_test.cpp`; there were 18.** REAL. The 02:40 entry
    listed 17. The 18th, "ST35 (B9 (4)): a Routine destroyed with a trigger still ARMED logs one Warn", was
    added after 02:45 (the suite was 3037 then) and before the run of record (its DTOR mutant was caught by
    that case at 03:47), so that B9 (4)'s destructor Warn had a test. Its addition was never logged; it is
    logged here. **Now:** the file has 25 cases (18 + this round's 7). `routine_trigger_example_test.cpp`
    has 3 (guide-10i added, finding 22), so the suite has **3046 = 3005 + 41**.
22. **(docs) No compiled listing showed the Tier-3 observer spelling.** REAL. A compiled listing was added in
    **guide 10** (the API chapter, beside the plain verbs), not in a cookbook recipe: case `guide-10i` in
    `test/routine_trigger_example_test.cpp`, quoted verbatim (`check-examples` passes). **Delta from the
    brief's §1.2 "the cookbook shows":** the Tier-3 spelling lives in the API chapter, because it is a
    seam for writing a watcher, and no cookbook problem needs it on its own.

**New deltas from the brief (this round), each with its reason:**

16. **`motion::MotionScheduler::pendingAbortFault()`**, a new read-only member on `motion_scheduler.hpp`,
    outside every pinned row (no Freeze Register row pins MotionScheduler). Finding 1 needs it so the
    decorator applies the scheduler's own test rather than a copy. **For the coordinator at merge:** main's
    L6-A2 changed `motion_scheduler.hpp` (its shadow line). This run's edit is one member after
    `motionsAborted()`.
17. **A handle drives once, for ITS line only** (amends delta 8: a kept handle used after another line is
    armed is now refused, and a mark kept and started twice is refused).
18. **A mark written and not started is pending**, refused by the next call and said by the destructor
    (finding 4). The brief's B9 names only the armed handle.
19. **The failing trigger is the FIRST TO FIRE among the failures, not the first armed** (finding 3), and
    `lastMechanism()` is that trigger's report, else the latest fired trigger's.
20. **A start form's `name` names the trigger in its lines** (finding 18). The brief says "same parameters"
    and does not say what the name does.
21. **The changelog entry sits at the top of `## API 2.6`** (amends delta 1). Main is at API 2.6 since
    `1ed0e26`, and the coordinator's instruction for this round says so. In this worktree a `## API 2.6`
    heading was added above the entry, titled "Added since 2.6", with "Every 2.6 program". STEP1-A's entry
    stays under `## API 2.5` here, as at the base. Main moved it under 2.6 when it was numbered, and the
    coordinator merges the two and sets the minor (7).

**The record of mutations** (`scratch/step1-b/mutate_fix.py`). Each replacement is asserted to occur the
stated number of times, and the file is backed up under `bak/*.fix-<id>.orig`. The restore runs in a
`finally` and on SIGTERM, SIGHUP and SIGINT, and the file is touched. One full build comes first, then the
CONTROL mutant, then `make shulib_tests/fast` per mutant. Every subprocess runs under `ulimit -v 4194304`,
and the suite under `timeout 600`. A mutant whose build fails is re-cut and never counted. **The CONTROL
was red in 64 cases. Counted: RF1, RF2r, RF2br, RF2c, RF3, RF3b, RF3c, RF3d, RF4, RF4b, RF5, RF6a, RF6b,
RF7a, RF7b, RF8a, RF8b, RF10, RF11a, RF11b, RF11c, RF18, plus P1–P6: 28 of 28 CAUGHT.** None was killed by
the cap or the timeout. Not counted: RF2 and RF2b (they broke their own build and were re-cut).
**A slip in the driver, found and fixed:** run 1's "final restore build: FAILED (0 objects)". The cause was
the full target's doc gate (`briefing_status.py check`). It runs `build/test/shulib_tests` before it
compiles anything, and that binary was still **RF18's mutant link**: the pin mutants compile one object and
link nothing. The gate ran that link once under the driver's 4 GiB cap and once more uncapped, in my own
diagnosing `cmake --build`. Both runs ended normally, with one case red (RF18's). Nothing hung, and no
source file was mutated at the time: both files were `cmp`-equal to `bak/*.fix-pre`. Fixed: the driver now
removes the binary before its final build (run 2's final build: ok, 43 objects). I also removed the binary
by hand and relinked it from source (107 objects), and the suite is green on that link. **Lesson for the
coordinator's list:** after any pin-only (compile-one-object) mutant, the binary on disk is the last suite
mutant's link, and a full-target build runs it through the doc gate.

**Resumed again 2026-11-12 22:13** (a `date` reading), after the team lead's pause at ~14:00 on 2026-11-11.
The round above stopped at 23:11 on 2026-11-11 inside the src gate's self-test (`fix1/srcgate_selftest.log`
is empty). The coordinator's audit found and restored a self-test plant in the Makefile ("calibrate append
dropped"). **Audit on resuming:** `git diff -- Makefile src/` is empty and holds 0 `(self-test plant` lines.
Each file the mutation tooling backed up is `cmp`-equal to its newest backup: `routine.hpp` to
`bak/routine.hpp.fix-RF2br.orig`, `chassis.hpp` to `bak/chassis.hpp.fix-P6.orig`, `leg_progress.hpp` to
`bak/leg_progress.hpp.LP3.orig`, and `service.hpp` to `bak/service.hpp.SVCDRIFT.orig`. No driver of mine
was running. (The `src_build_gate.py self-test` process running at the time belonged to the `rob1-a`
worktree, not this one.) All 22 findings were already dispositioned above, and every RF/P mutant is
recorded as observed on its own fresh build, with its object count. So nothing was redone and nothing was
reverted. **I re-checked these by reading or grepping:** the fault-abort fix (`pendingAbortFault()` returns
the scheduler's own `newlyRaisedAbortFault()`, the test `tick()` applies after a Running tick, and the
decorator returns before calling the observer when it is set). The briefing reads "206 registered,
**7 settled**" and "F6 56". The HA index reads 215, 216, 217, 218. The changelog entry is at the top of
`## API 2.6`. Guide 14 has the arms clause. The case counts are 25 + 3 + 5 + 6 + 2 = 41, so 3005 + 41 =
3046. I also ran a prose-review grep over the round's new wording: "any/next drive or turn" has 0 hits
left. The "never on the tick that ends it" lines (guide 09 :338, guide 10 :433, the changelog :42,
routine.hpp :1026) all name the fault abort, which is now true. A stale "F6 57" outside the chunk logs has
0 hits. "simulator" in the change's added public lines has 0 hits.

- 2026-11-12 22:13–22:51: **the fresh link and every gate on the final tree.** Logs are in
  `scratch/step1-b/fix2/`. No source, test or doc line changed on resuming apart from this entry and the
  regenerated briefing.
  - **The binary was relinked from source:** it was removed, the build reconfigured, the briefing generated
    with no binary, a full build, the briefing generated again, then a build. After the src gate (which
    touches `src/`), the binary was removed again and the build rerun: **269 objects recompiled**, a
    wholly fresh link. Then the briefing was regenerated.
  - **The suite**, through `suite_run.py green` and capped: "green — 3046 cases, 2960308 assertions". The
    result was the same before and after the src gate.
  - **ARM header gate:** 241 headers in one TU (`fix2/all.cpp`), `-Werror`, exit 0. **The two CI greps**
    (pros/ outside hal/pros; shulib/sim outside sim): both empty.
  - **`api_doc_tool`:** self-test OK. check-coverage reports "5553 public entities across 198 headers, all
    documented". check-fresh exits 0. check-examples reports "695 quoted lines, all verbatim (14 source
    files); 6 listings of whole-run files, each one unbroken run". check-removability is clean.
  - **`doc_staleness_audit.py`:** clean, and its self-test OK. **`briefing_status.py`:** self-test OK
    (38 cases), and check reports "generated block current".
  - **`ctest -R compile_refusal`:** 269/269 passed (t01..t14 are among them).
  - **`src_build_gate.py`, run in full:** self-test "OK (269 detector cases, each against a real build)",
    then 0 plant lines. check reports "PASS (all 7 builds link ...)", then 0 plant lines.
    `git status -- Makefile src/` is empty, and `git diff --stat ddab08c -- Makefile src/` is empty.
  - **For the coordinator at merge:** main is at `c0a7c7e`, which has 8 builds (TUNE1-B1's Tune program). This
    worktree's base has 7, so the src gate's "all 7 builds" is this base's count. `service.hpp`'s edit is
    still the four check functions only (90 diff lines against the base). `motion_scheduler.hpp`'s edit is
    one member, `pendingAbortFault()`, after `motionsAborted()`. `mkdocs.yml` has one nav line.
  **STEP1-B's review fix round is complete; every gate is green on a fresh build. Not committed.**

### Verification-round findings and their disposition (2026-11-12, late evening → 2026-11-13 00:20)

A fresh verifier returned one finding (minor). It was re-checked against this worktree before it was
answered. **Every diff is still against `ddab08c`.** Logs are in `scratch/step1-b/fix2/vr_*`.

1. **Finding 2's fix was half untested — REAL test gap, FIXED (test only; no behaviour changed).** Re-checked:
   `arm()`'s guard (`routine.hpp` :1998, `SHULIB_PRECONDITION(pendingMark_ && gen == armGen_, ...)`) has two
   halves. The review test "a handle acts for ITS line only" reached only the started-twice case, where
   `pendingMark_` is false. My RF2br replaced the WHOLE condition with `true`, so it never measured the
   line-number half alone. The verifier's N1 (`((void)gen, pendingMark_)`) is that half alone, and it
   survived. With it, a kept DRIVE mark armed onto a later TURN line, silently consumed that line's pending
   mark, and the turn ran carrying a drive mark (the kind split bypassed, and a "the drive was 0.0 in" Warn
   on a turnTo).
   **Fixed:** a new case, "STEP1-B (verification): a MARK kept from an EARLIER line is refused while a LATER
   line's mark is pending -- a turn never carries a drive mark" (`test/routine_trigger_test.cpp` :1114). It
   runs the verifier's sequence: line 1 `afterDistance(5_in)` kept as `m1` and driven; line 2
   `afterTurning(30_deg).startSpinUp(...)` with `h2.atPercent(50)` pending; then `m1.startMove(lift...)`. It
   asserts:
   - the start is refused with "from an earlier trigger line";
   - line 2 was cleared with the refusal, so `h2.turnTo` is refused;
   - no motion started, and `steps` stayed 1;
   - neither the lift nor the flywheel began a step;
   - no "did not fire" Warn was written;
   - a following `pause` runs (steps 2, completed 2).
   The two cases share one message, so the STATE (a mark pending) pins this half, not the wording. The
   case's comment says so.
   **Delta from the verifier's suggestion, with its reason:** the verifier suggested appending the sequence
   to the existing review case. I did that first. On RF2d the case went red, but a probe showed the
   Routine had already STOPPED at that point: the earlier `m` line's `driveTo(20_in, 0_in)` from (30, 0)
   timed out (RTN: "STOPPED at step 5 (driveTo): motion TIMEOUT"). So the mutant's turn was only SKIPPED, and
   the motion and Warn checks passed under the mutant: vacuous. The sequence now runs in its own case, with
   a fresh `RobotX` and Routine. It REQUIREs that line 1 settled (`completed == 1`, `stoppedAt == 0`), so
   the turn really would run if the mark were accepted.
   **Comment updated:** `warnMarkPastLeg`'s comment (`routine.hpp` :2309) now names `arm()`'s line check
   beside `requireLine`, as the verifier noted. It is a private comment; `api_doc_tool generate` moved only
   a line number in `docs/api/routine.md`.
   **Mutations** (`mutate_fix.py`, run 4, `mutations_fix_run4.out`). Each was a fresh build of all 43
   dependent objects, run capped, and its file restored and touched. `routine.hpp` is `cmp`-equal to its
   pre-run copy afterwards.
   - **CTRL:** red in 65 cases.
   - **RF2d** (= N1, `SHULIB_PRECONDITION(((void)gen, pendingMark_), ...)`): **CAUGHT**, red in the new
     case only (3046 of 3047 passed). A show-run on its own fresh build (`fix2/vr_rf2d_show.log`) names
     the assertions: the refusal (:1136), `h2.turnTo` refused (:1139), motionsStarted 2 vs 1 (:1140), steps
     2 vs 1 (:1141), the flywheel's `stepsBegun` 1 vs 0 (:1143), one "did not fire" Warn (:1144), and
     steps/completed 3 vs 2 (:1146/:1147). So the turn RAN under the mutant, and the case is not vacuous.
     Each of these is the test's own CHECK; no compile-time check was involved.
   - **Re-runs on the changed test file:** RF2r CAUGHT (1 red); RF2br CAUGHT (2 red: the review case and
     the new one); RF2c CAUGHT (1 red).
   **A slip, found and fixed:** run 4's first attempt reported "pre-build: FAILED". My show-run had left
   its MUTANT link in `build/test/shulib_tests`: its trap restored the source, not the binary. The full
   target's doc gate (`briefing_status.py check`, through `suite_run.py`'s bounded run) then ran that link
   and found one case red. The run ended normally, under the bounded run's limits. No source was mutated
   at the time (`cmp`-equal). Fixed: the binary was removed, the briefing generated, a full build, the
   briefing generated again, then run 4 again. This is the lesson already in the list above ("after a
   pin-only mutant..."), in a new form: **any hand-run mutant must remove its binary in its trap, not only
   restore its source.**
   **Not changed (noted for the coordinator):** after the refusal, `h2.turnTo` is refused with
   requireLine's "this trigger handle's drive already ran". requireLine's comment covers a cleared line
   ("after its line was cleared"), but the wording says "drive" on a turn handle whose line was cleared
   by a refusal, not run. This is pre-existing, outside this finding, and not a correctness issue.

**The prose review for this round (RESUMING rule 4), run before this entry was written:**
- `simulator` in this round's diff: 0 hits.
- `earlier trigger line|kept handle|kept mark|already started` over `docs/guide`, `docs/cookbook`, the
  changelog and `routine.hpp`. Every hit is true: the behaviour is unchanged, and the only new wording is
  the private comment at :2309. Guide 16 :208 ("already started") has another meaning.
- `3046|3,046|41 cases` outside this log: no live STEP1-B claim. The hits are other chunks' history and
  `suite_run.py`'s 41 self-test cases.
- The briefing is regenerated: 3,047 cases.

- 2026-11-12 23:00 – 2026-11-13 00:20: **every gate on the final tree** (logs in `scratch/step1-b/fix2/vr_*`).
  - **The binary was relinked from source after the src gate:** it was removed, the briefing generated with
    no binary, a full build (**269 objects**), and the briefing generated again.
  - **The suite**, through `suite_run.py green` and capped: "green — 3047 cases, 2960320 assertions".
  - **ARM header gate:** 241 headers in one TU (`fix2/vr_all.cpp`), `-Werror`, exit 0. **The two CI greps:**
    both empty.
  - **`api_doc_tool`:** self-test OK. check-coverage reports "5553 public entities across 198 headers, all
    documented". check-fresh exits 0 (also re-run after the last build). check-examples reports "695 quoted
    lines, all verbatim". check-removability is clean.
  - **`doc_staleness_audit.py`:** clean, and its self-test OK (16 cases). **`briefing_status.py`:**
    self-test OK, and check reports "generated block current".
  - **`ctest -R compile_refusal`:** 269/269 passed.
  - **`src_build_gate.py`:** self-test "OK (269 detector cases, each against a real build)", then 0 plant
    lines. check reports "PASS (all 7 builds link ...)", then 0 plant lines. `git status -- Makefile src/`
    is empty, and `git diff --stat ddab08c -- Makefile src/` is empty.
  - `service.hpp` was not touched this round.
  **STEP1-B's verification round is complete; every gate is green on a fresh build. Not committed.**

## STEP1-B — the coordinator's merge verification *(2026-11-13)*

- **Applied as its OWN diff** (`ddab08c..` its transport — it was built on a snapshot of STEP1-A's build) onto `d33d25d`
  (CURVE1-A, API 2.7). Code conflicts, both comments/usings: routine.hpp's banner (CURVE1's curve-step paragraph, its "triggers
  will not cover curves at first" made "do not cover curves yet", then STEP1-B's trigger paragraph) and f6_signature_pin_test's
  usings (both `CurvePoint` and `LegObserver`). Documents: guide 09's history line (curves at 2.7, then triggers "at 2.8");
  the roadmap's F6 and F10 rows composed by hand (CURVE1's text kept, STEP1-B's observer appended to F6 with the pin count
  re-counted from the merged pin file — 67: 51 `SHULIB_F6_PIN` + 16 `SHULIB_F6_NOT_NOEXCEPT` — and F10's trigger arms named);
  REMAINING-WORK rows 20c–20f by a word-level three-way merge (STEP1-B LANDED); the register (Held line: STEP1's three all
  landed; 7 of 216 settled; STEP1-B's "added TWO" paragraph after CURVE1-A's; HA-217/218's rows) — and a PRE-EXISTING stray `>`
  line that split the register's index table (between HA-223 and HA-216, from an earlier merge) removed and the 216 rows sorted;
  the changelog (STEP1-B's entry under a NEW `## API 2.8`); this log (STEP1-A's merge record, then STEP1-B's sections); a
  roadmap "Now" paragraph. **Numbered API 2.8**: `kApiMinor` 7 → 8, "2.8", its history paragraph.
- **Gates on the merged tree** (`.claude/coordinator/scratch/step1_b_merge_verify.sh`): the suite green (3,414 cases);
  `src_build_gate.py` self-test OK (306 detector cases) and check PASS (all 8 builds); the ARM header gate CLEAN; the doc gates
  and the briefing check clean; the compile-refusal harness 272 of 272; no plant, no stray root file.
- **The coordinator's own mutations** (`muts_step1_b_main.json`): **4 of 4 RED** — MB1 a trigger firing again every tick after
  its mark, MB2 a trigger firing after the deadline cut, MB3 a distance mark past the leg's end (plus tolerance) counted on the
  leg, MB4 the same for a turn mark. The restored tree relinked and ran green.
- **STEP1-B LANDED — STEP1 (A + B) is complete before the 21st.** Nothing has run on a robot.
