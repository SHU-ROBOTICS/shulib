# F2 PROGRESS — sequence engine

- 05:44:01 START. Creating progress log. Next: read the brief twice, then RESUMING.md, then the scheduler header in full.
- 05:44:12 Brief read (pass 1 of 2). 15 measurements noted. Key overturns: M2 (waitUntil ignores cancel), M9 (cancel-at-deadline inert), M11 (post-deadline moveTo self-cancelled after 1 tick) -> latch, not cancel-storm. Now reading RESUMING.md + scheduler header in full.
- 05:44:23 Read RESUMING.md + motion_scheduler.hpp in full. Confirmed: cancel() guards are !inTick_ && !inBoundary_ only (lines 663-667) — inWait_ NOT checked, matching brief's T1 note. waitUntil loop (641-654) has no active_/deadline hook: measurement 2's 28s hole is structural. tick() is public and precondition-clean when not in wait — the supervisory loop can own it. Next: motion.hpp, chassis.hpp, routine.hpp.
- 05:44:38 Read motion.hpp, chassis.hpp, routine.hpp in full. Noted: Chassis::waitUntil (chassis.hpp:439-441) is indeed a bare pass-through, no DetachGuard (measurement 15's finding confirmed in source). runBlocking has the guard (chassis.hpp:526-532). MotionOptions.timeout=0 -> config default via MoveToPose ctor. Routine::waitFor maps Satisfied->recordSuccess (measurement 6 confirmed at routine.hpp:334-339). Next: diagnostics-plan 175-184, D1 §2.7, D3 §2.1, F1 §13, master plan §14, watchdog.hpp, mechanism headers.
- 05:45:14 Read watchdog.hpp, mechanism.hpp, mechanism_op.hpp, D-8 charter, D1 §2.7, D3 §2.1, F1 §13, master plan §14. Confirmed in source: RunUntilConfirmed::tick line 304 re-commands voltage every Running tick — applySafeState alone lasts until next tick (measurements 13/14). IMechanism has applySafeState+name+claim only — NO path to its op (T6 gap confirmed). Master plan §14 "extending the existing RobotCommands/Command queue" at line ~576 needs the Rule 4 fix. Now surveying roadmap/build-order/cookbook/guide targets.
DESIGN (timestamps below): the eight rulings, drafted after full required reading.
- T1: ONE new control point — a pacer decorator (the guard IS an ITickPacer wrapping the caller's).
  Deadline-aware waits reuse C2's waitUntil via a composite predicate (M5: 0.0000s latency) and
  disambiguate AFTER the wait (M6's fix). NO new supervisory tick() loop: the end action runs
  through the frozen facade's own runBlocking loop, so F2 adds ZERO loop owners. REJECTED: the
  brief's sketched scheduler.tick() supervisory loop — it would be a second loop owner
  duplicating what waitUntil already does. Rule 4: cancel-from-pace() gets added to C2's pinned
  re-entrancy list + tests. async-from-pace (M7/M8's silent hijack) REJECTED as a pattern.
- T2: two caller-supplied instants, no defaults: endActionAt (stop scoring, act) and hardStopAt
  (unconditional safe floor, fires even mid-end-action). REJECTED: a single instant / default lead.
- T3: end action = caller-supplied callable (void/bool/ExitReason/MechanismOutcome, then()'s
  exact convention). No pose, no parkAt(). REJECTED: parkAt(Pose2d) convenience.
- T4: guard.waitFor/guard.pause are deadline-aware (phase-aware: endActionAt during scoring,
  hardStopAt during the end action); frozen F10 waits CANNOT be — lateness formula documented
  (unexpired remainder summed over post-expiry wait steps until the first refused motion stops
  the chain). REJECTED: throwing pacer (M15) or predicate-Satisfied folding (M6).
- T5: mint minimal sequence-layer verdict: GuardedWaitResult{Satisfied, TimedOut, RunExpired}.
  Map in, never re-mean; RunExpired wins ties with Satisfied (pinned + mutated). REJECTED: a
  shared step-outcome vocabulary spanning all tiers (a fifth tier for one consumer).
- T6: fixed at F1's layer (F11 unfrozen, Rule 4): hal::IMechanism claim gains an optional
  claimant hook (tryClaim(ICancellable&)); both ops register themselves; guard walks
  span<IMechanism*>, cancels claimants, releases anonymous claims (Warn), applies safe state.
  BONUS F1 finding: an op destroyed mid-flight left its mechanism CLAIMED FOREVER AND ENERGIZED
  (no destructor; rule-of-three violation) — fixed with destructors + deleted copy/move.
- T7: latch = pacer-enforced refusal: post-expiry active motions cancelled at pace() entry,
  BEFORE the plant step (M10's 0.0000in ordering, pinned by mutation). End action exempt via an
  in-end-action window flag; hard floor exempt from the exemption. REJECTED: cancel-only (M9),
  refuse-by-throw.
- T8: MINIMAL CUT — no Sequence/Parallel/Race combinator types. Both public docs sanction it;
  the concurrency the combinators would express already exists (waitUntil predicate ticking ops
  while a motion runs — F1's own test); the async handle exists (scheduler()). Deadline ships as
  the guard, not a wrapper type. REJECTED: silently building four combinators against two docs.
- Shape: shulib::sequence::RunGuard final : ITickPacer. Ctor(inner pacer) — INERT pass-through
  until run() (D3 §2.1 opt-in instruction, tested). run(chassis, cfg, scoring, endAction) ->
  RunGuardReport. Exception unwind from scoring/endAction: cancel-all + safe + rethrow (no park
  on a broken program). Scoring returning early runs the end action immediately.
- Also Rule 4: MotionScheduler blocking waits get an unwind guard (M15's 11.4V-under-Coast:
  root cause is C2's loop stranding an active motion on throw; C4's DetachGuard was the symptom
  patch and stays). Chassis::waitUntil hole closes through it with no facade edit.
- Zero invented constants expected: both instants caller-supplied — the design's point. Pause
  backstop slack mirrors Chassis::kWaitBackstopSeconds (code-level, not HA, same reasoning).
- 05:56:24 Design drafted into log. Now checking implementation preconditions: existing tryClaim uses, op copies, stalled-pacer test expectations, exit_group, fake sink API, hostile mech fakes, CMake, api_doc_tool targets.
- 06:01:18 Pre-checks done: no op copies in tests; scoped op tests assert before destruction; degradation has slip/freeze but both corrupt the estimate (case-D cascade will use scripted raises, comment says why). CMake auto-globs *_test.cpp. check-examples requires new doc listings to live in a test/*example*_test.cpp — new file sequence_example_test.cpp. Doc tool TARGETS stays chassis+routine (F11 precedent: unfrozen headers stay out; record ruling). Starting edits: hal/mechanism.hpp claim hook.
- 06:03:55 Seam edits landed: hal claim hook (ICancellable + tryClaim(claimant) + claimant()), op destructors + deleted copy/move + registered claims, scheduler re-entrancy pin + WaitUnwindGuard in both blocking waits. FULL SUITE GREEN UNCHANGED: 974 cases / 1522018 assertions / 3 skipped — the unwind guard broke no pinned behavior. Now writing include/shulib/sequence/run_guard.hpp.
- 06:26:53 Suites landing green: claim-hook/destructor (7 cases), scheduler unwind+pin (4 new), run-guard unit (17 cases, 193 asserts total across F2-prefixed). Now writing the DoD end-to-end suite with the INDEPENDENT pace-tally script (the anti-shared-clock counter).
- 06:30:02 DoD first run: stalls 1-3 GREEN. Two RED: (a) cascade case — park missed by 29.58in, end action FAILED: the 80 aborted legs each drove ~10 ticks at full speed, robot zoomed ~300in downfield, park 4s timeout could not get back (test-design flaw: cap leg speed / earlier raise); (b) M10 exact-zero pin — 0.2249in over 40 refusals: SURPRISE, investigating whether the plant models braking decel (stopping distance from cut speed), which would make 'exactly zero' the WRONG claim and 'refusals add nothing beyond the physical stop' the honest pin.
- 06:31:32 PROBE RESULT: refusals add EXACTLY 0.000000in (5 probed) — engine correct. Both DoD reds were TEST bugs: the M10 storm's uncapped 300in leg SETTLES at ~5.3s (60in/s default) so no cut ever happened (0.2249in = settle chatter measured between legitimate motions); cascade legs at full speed drove ~300in downfield, park could not return in 4s. Fix: cap leg speeds so the stalls actually stall. The exact-zero pin STANDS.
- 06:32:11 DoD suite GREEN (7 cases, 71 asserts): four stall shapes each end parked+safe at the INDEPENDENT limit; M2/M3/M10 regressions pinned; cascade case now aborts >10 legs and still parks. Full suite next.
- 06:37:11 Roadmap updated: F12 register row (NOT FROZEN, out loud), F11 dependency cell fixed, WS8 block rewritten with the T8 ruling + F4 split + possession-phrase retired, you-are-here F2 entry (leads with what did NOT happen), WS-table renamed. Removability grep clean. Launching verify-f2.sh (14 mutations, build-gated) in background; writing guide/cookbook/master-plan/diagnostics docs meanwhile.
- 06:39:07 MUTATION CAMPAIGN run 1: 13 RED / 1 GREEN / 0 build-fail / 0 skipped. THE GREEN: M10 (composite predicate re-ordered pred-first) — my predCalls==0 test only covered waits STARTED after expiry, where the pre-check masks the order; a wait IN FLIGHT crossing the deadline calls pred one extra time at a post-deadline clock (one more 9V device event past the buzzer). Closing with a last-pred-call-time test. Note: M13 aborted (tightened precondition re-throws inside the unwind guard during unwind -> terminate) — counted RED, mechanism noted for the record.
- 06:41:55 M10 hole CLOSED: new in-flight-predicate test fails alone under the mutation (observed RED, 1 assert), restored byte-identical, suite green. Campaign now 14/14 RED. Example tests (sequence-01/02/03) green — two honest test-bugs found on the way: startAt(-48,-24) shifts estimate frame off plant truth (origin-start for truth-graded examples), and the chain-stop SKIPS the post-cut leg so only ONE post-expiry cancel exists (the formula's own point). Now: docs (guide 6/9/12/14, cookbook 03, master plan, diagnostics-plan, legacy vocab, accuracy naming, build-order, COMPLETED).
- 06:51:00 verify-f2.sh FINAL: exit 0, all clear — 14 RED / 0 GREEN / 0 build-fail / 0 skipped; register F1-F5 untouched, F12 added not-locked; scope gates clean; restores byte-identical.
- 06:51:00 Docs discharged: guide 6 (new failure class) / 9 (frozen-wait formula + guard idiom, compiled sequence-02) / 12 (SEQ triage) / 13 (claimant note, stale park-guard lines fixed) / 14 (honesty boundary VERBATIM) / README ledger; cookbook 03 reworked (guard primary, phased recipe kept for the conditional half, clockNow note updated with exactly which half is superseded); master plan §14 RobotCommands sentence retired; diagnostics D-8 DISCHARGED with remainder named + D3 miscite fixed; legacy vocab chunk attributions fixed; accuracy.hpp + test case names disambiguated (register row F2 vs chunk F2, brief-ordered); build-order Current position + F2 RULED block.
- 06:51:00 FINAL VERIFICATION: suite 1017 / 1,522,314 / 3 skipped GREEN; GUARD1 PASS; GUARD2 PASS; ARM 124 headers PASS; all four doc gates + self-test PASS; removability clean. F2-COMPLETED.md written (T1/T4/T7 first). Zero HA entries (nothing invented — the design's point); next free stays HA-94. NOTHING COMMITTED, NOTHING PUSHED — the tree above is the chunk.
- 06:51:00 DONE.
