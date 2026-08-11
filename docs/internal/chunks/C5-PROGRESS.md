# C5 PROGRESS — diagnostics: the run is legible in real time

05:48:13 START C5 session opened. First actions: read the brief (C5-results-and-summary.md), master plan §18.3, diagnostics-plan.md, A1/C2/C4 COMPLETED docs, then the diag/ code.
05:48:58 DONE Read brief, §18.3, diagnostics-plan, A1-COMPLETED, debug_record/term_sink/fault/telemetry_sink/char_sink headers, motion_scheduler, chassis facade. Next: C2/C4 COMPLETED, RESUMING, loop_monitor/finite_guard/health_monitor, motion.hpp, test rig.
05:56:53 DONE Required reading complete (brief, §18.3, D-plan, A1/C2/C4, RESUMING, build-order C5, ci.yml, diag+motion+chassis headers). Now reading motion record-emission internals (move_to_pose, exit_group, localizer) to design the result-line data path.
06:04:29 DECIDE Architecture settled after reading everything. The plan:
  - SCHEMA FIRST (the F9-sensitive act): DebugRecord += droppedRecords/droppedLines (u32, D-2) +
    tickPhase[8] (units::Time, D-3) with a wire-stable TickPhase enum beside GateReason (2 spare
    slots reserved). FaultCode += Implausible=10 (append-only, for D-5).
  - Result/summary vocabulary lives in diag/ as value types (SessionInfo, MotionResult+Outcome
    per §18.4's SETTLED/TIMEOUT/CANCELLED/FAULT_ABORT/SUPERSEDED list, RunSummary with bounded
    char arrays — no lifetime traps); ITelemetrySink grows an ADDITIVE summarize(RunSummary)
    no-op default (the A1 emit() pattern, forward-declared); TermSink renders the §18.3 block.
  - Result lines + session header ride the log() channel (the FaultLatch precedent: the owner
    formats structured text, the sink sanitizes/frames) so the [t=][MOT]/[SES] shape matches
    §18.3 exactly. Shared numeric formatting EXTRACTED from TermSink into diag/line_format.hpp
    verbatim (NaN/±Inf tokens + %.3g compaction everywhere; bit-identity gate = A1 goldens).
  - Scheduler: CompletedMotion += finalPose (always, localizer at boundary) + targetPose/
    overshoot/drift/hasPathData (record-stream-derived via a MotionStatsSink in the stamp
    chain) + preempted (SUPERSEDED). IMotionObserver boundary seam (re-entrancy guarded).
    TickAttribution wired behind an OPTIONAL attribution clock (null = zero cost, zero calls);
    overrun lines name the worst phase from the LAST COMPLETED tick. PoseDeltaGuard per tick.
  - D-2 RateLimitedSink decorator (token buckets; Error/Warn EXEMPT; drops counted, stamped
    into forwarded records, notice line per episode end, totals in summary). D-1
    LevelFilterSink decorator (explicit filtering is config, not degradation — not counted).
  - D-4: hal::ILineDisplay seam (3x19, HA register entry, PROS glue -> R1) + FakeLineDisplay +
    diag::ControllerFaultDisplay (writes only changed rows).
  - D-5: PoseDeltaGuard (scheduler, episode-gated) + command/wheel-volt audits in the shared
    pipeline (defense-in-depth self-check; injectable at the free functions).
  - Build hash: SHULIB_BUILD_HASH macro from the build system (test CMake: git describe
    --always --dirty at configure + CONFIGURE_DEPENDS on .git/HEAD); empty/missing => ERROR
    line + "build MISSING" in summary, never a plausible placeholder.
  - RunReporter (motion/, glue): sessionStart/onMotionComplete/finishRun; reads scheduler
    counters + latch + health + battery; diag/ cannot name CompletedMotion (dependency leaf),
    so glue sits with the scheduler, formatters sit in diag/.
06:04:29 START Schema work: debug_record.hpp fields + TickPhase + FaultCode::Implausible, then their test pins.
06:06:28 DONE Schema reserved AND green: DebugRecord += droppedRecords/droppedLines (u32) + tickPhase[8] (units::Time, TickPhase 0-5 defined, 6-7 spare); FaultCode += Implausible=10. Wire pins added (fault_test, debug_record_test incl. new TickPhase pin case). Suite 593/915,181 green.
06:06:28 START line_format extraction: TermSink's Line/appendNum/appendPadded/appendUnsigned/appendTimestamp -> diag/line_format.hpp VERBATIM (bit-identity gate = the A1 goldens staying green).
06:09:59 DONE line_format.hpp extracted VERBATIM + TermSink rewired via ADL (A1 goldens all green = bit-identity proof, 593/915,181). ITelemetrySink += summarize(RunSummary) (additive, A1 emit() recipe); diag/run_summary.hpp value type (bounded strings, no dangling views); TermSink renders the §18.3 six-line block. Next: Fake summary recording + build_info/session_info/motion_result.
06:17:30 DONE D-1 LevelFilterSink, D-2 RateLimitedSink (counted+stamped+announced drops; Error/Warn+summarize exempt), D-3 TickAttribution (+tickPhaseName), D-5 plausibility_guard (PoseDeltaGuard episode-gated + pipeline self-audit invariants 2/3 WIRED into command_pipeline). Suite green 593/915,181 — the audit is a pass-through on the entire existing corpus incl. hostile suites (zero false positives). build_info/session_info/motion_result headers in. Next: D-4 line display seam + controller content.
06:56:14 FOUND The first live judging of the estimate by PoseDeltaGuard tripped 3 boot-window suites (C2/C4 boot + never-live): while Uninitialized the published pose is not a physical trajectory, and the boot->live transition is a legitimate jump. ROOT-CAUSE fix: guard resets through the boot window, first live tick re-baselines (scheduler call site). Suite green again 593/915,181. This is exactly the false-positive class the guard had to be proven free of before it ships.
06:56:14 DONE Scheduler integration complete: StatsHolder+CommandIdStampSink chain (id+tickPhase stamps), MotionStatsSink aggregation, CompletedMotion +6 result fields, IMotionObserver boundary seam (inBoundary_ re-entrancy preconditions on all 5 verbs), AttributionTickGuard (exception-safe), overrun attribution line, run heading aggregates, SUPERSEDED preempt marking. Next: motion/run_reporter.hpp glue.
09:12:12 DONE RunReporter glue + CMake build-hash injection (git describe --always --dirty at configure, CONFIGURE_DEPENDS .git/HEAD). All C5 headers compile standalone under strict flags. Now writing the test files.
15:55:47 DONE All test files in: session_header (6), motion_result (6), run_summary (8), level_filter (5), rate_limit (7), tick_attribution (7), plausibility (7), controller_display (6), run_report_e2e (12). Suite 658 cases / 915,559 GREEN.
15:55:47 FOUND Ground-truth cross-check observed values: CLEAN plant reported-vs-true divergence ~1e-13 in (estimate == truth with perfect sensors) — bounds TIGHTENED to 1e-9 (razor pin: on a clean plant the report must EQUAL physics). But that made the in-vivo overshoot check 0-vs-0 (vacuous — the kA=0 plant is memoryless and CANNOT overshoot). Closed with an INERTIAL sub-case (kA=0.05, FF knows only kS/kV): robot really overshoots 3.79 in, reported == truth to 6 digits (3.78696 both). HOSTILE: settled 5/5, worst believed-vs-true 1.76 in (< the 5.0 C2/C3 hostile envelope).
15:55:47 FOUND Two e2e test bugs found+fixed during bring-up (both documented lessons biting): (1) BusyMotion emitted no records, so only UNSTAMPED plant records existed (C2 §4.3's two-producers note); (2) the overrun gap fired before the post-reset baseline tick, so LoopMonitor saw nothing. Neither was a scheduler defect.
15:55:47 START Verification gates: CI guards + ARM cross-compile; then the mutation campaign (build-gated).
15:56:35 DONE Gates: GUARD 1 (PROS-free) PASS incl. all new diag/hal headers; GUARD 2 (sim-free core) PASS; ARM cross-compile CLEAN over all 102 v2 headers — after catching ONE real portability bug (uint32_t is unsigned long on ARM; %u in the throttle notice would not compile on target; fixed with %lu+cast). Suite re-green 658/915,559.
15:56:35 START Mutation campaign. Protocol: snapshot pristine (cp), mutate, BUILD-GATE (make must succeed or the result is discarded — the C4 stale-binary lesson), run, OBSERVE, restore from snapshot, cmp-verify. Target: well past four; hunting green survivors deliberately.
16:08:25 MUTATE M23 GREEN SURVIVOR FOUND (deliberate hunt): severing BOTH D-5 audit calls in the command pipeline left all 915,559 assertions green — the audit is a pass-through on healthy input and the D-5 tests injected only at the free functions. The wiring itself was unpinned. Closing with a direct hostile-pipeline test (NaN speeds -> Implausible raised + 0V at the motors).
16:13:17 MUTATE M23 closure: first closure attempt used a combined raise-count and M23a-ONLY (command audit severed, volt recovery intact) STAYED GREEN behind invariant 3's four raises — one mutation hiding behind another. Strengthened to pin each invariant's wiring DISTINCTLY (structured-detail match + exact 1+wheelCount count). Re-run: M23a-only RED (2 asserts), M23b-only RED (case aborts — FakeMotor precondition-rejects the NaN volt that recovery no longer stops), both-severed RED. Suite re-green 659/915,570.
16:17:26 DONE Mutation campaign: 35 mutations / 37 executed runs (each: mutate -> BUILD-GATE -> run -> observe -> restore -> cmp-verify).
  RED (33): M1 marks-swap(15 asserts) M2 abort-code(2) M3 n/a-invert(5) M4 missing-error-line(2) M5 plausible-placeholder(3) M6 first-fault-none(1) M7 drops-hidden(1) M8 heading-fabricated(2) M9 filter-invert(5) M10 override-dead(3) M11 drop-uncounted(5) M12 stamp-removed(2) M13 exemption-gone(2) M14 notice-gone(1) M15 phase-overwrite(1) M16 worst-argmin(5) M17 phases-unstamped(2) M18 overrun-unattributed(1) M19 episode-spam(2) M20 guard-dead(12) M21 boot-gate-gone(3) M22 volt-recovery-dead(4) M24 stats-unread(4) M25 preempt-unmarked(2) M26 observer-severed(10) M27 waiting-aggregated(4) M28 hold-branch-gone(1) M29 last-as-first(1) M30 always-repaint(6) M31 finalPose-origin(6) M32 compaction-defeated(2) M33 battStart-unread(1) M34 stale-aggregates(2) M35 abort-collapsed(1).
  GREEN FOUND (2, both closed): M23 (pipeline D-5 wiring severed — invisible to 915k assertions; closed with the hostile-pipeline test) and M23a-hiding-behind-M23b (combined count let the command-audit half stay green behind the volt half — closed by pinning each invariant's wiring DISTINCTLY). Post-closure: M23a-only RED, M23b-only RED, both RED.
  BUILD-GATE trips (1): M10's first form (-Werror unused-parameter) — discarded, adjusted, re-run for the genuine red. The gate exists because of exactly this (C4 §4.6).
  Post-campaign: all 15 touched headers cmp-identical to pristine; marker grep clean; suite 659/915,570 green.
16:17:26 START Documentation: register entries (HA-56/57), diagnostics-plan.md D-1..D-5 status + reserved-fields record, roadmap touch, C5-COMPLETED.md.
16:20:32 DONE Register (HA-56/57 + counts), diagnostics-plan (D-1..D-5 delivered + F9 discharge table), roadmap (two WS13 boxes [x] with evidence + position note), build-order (position block). Now writing C5-COMPLETED.md.
16:25:18 DONE C5-COMPLETED.md written (633 lines, C1-C4 depth, with the "What we now know for certain, and what we do not" section).
16:25:18 DONE FINAL VERIFICATION (all actually run):
  - Host suite: 659 cases / 915,570 assertions, 0 failed, 3 pre-existing skips (baseline 592/915,157 -> +67 cases / +413 assertions)
  - GUARD 1 (PROS-free core) PASS · GUARD 2 (sim-free core) PASS
  - ARM cross-compile: CLEAN, all 102 v2 headers (89 -> 102; the gate caught + we fixed one real %u portability bug)
  - git: NOTHING COMMITTED — the full C5 working set (14 modified + 24 new files) left in the tree for review, per the brief
  - Register reconciliation grep clean; HA-56/57 added; diagnostics-plan carries the F9 discharge table; roadmap WS13 boxes flipped with evidence; build-order position updated
16:25:18 DONE C5 complete. Green survivors found+closed this campaign: M23 (D-5 pipeline wiring invisible to the whole suite) and M23a (command-audit half hiding behind the volt half). The transcript golden closes M2's "legible in real time on the terminal" clause.
