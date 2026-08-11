# D2 — F6 freeze — PROGRESS

[02:17:59] D2 session start. Step 0: PROGRESS log created. Beginning required reading: brief (D2-f6-freeze.md), D1-COMPLETED §2/§6/§11/§12, C4-COMPLETED §8/§2/§5, chassis.hpp, routine.hpp, roadmap Freeze Register, guide-maintenance.md, RESUMING.md.
[02:18:13] Read the D2 brief and D1-COMPLETED in full. The nine-item critique packet (§2.1–2.9) noted; D1 recommends: retype time (B1), lean-add wait (B2), adopt brake/hold (B3), close abortFaultMask deferral (C1), keep branch-on-fallback rejection (C2), sugar stays recipe-layer (C3), accept 3 result vocabularies + flag F2 (C4), four tensions no-change (C5). Next: C4-COMPLETED §8/§2/§5, then the two headers.
[02:18:23] Read C4-COMPLETED in full: 18-row ledger (§2), candidate surface + why-each-shape (§8), 14 decisions (§5). Noted C4 D9 (brake/hold proposed), D10 (MotionOptions additive), §2 C3-flag-4 (maxWheelSpeed additive path = MotionConfig passes whole — D2 must check structurally). Next: chassis.hpp + routine.hpp in full.
[02:18:38] Read chassis.hpp (528 lines) and routine.hpp (497 lines) in full. Surface tallied: 2 public types in chassis.hpp beyond Chassis (ChassisConfig, MotionOptions, TrajectoryResult = 3 types), Chassis members: ctor, moveTo, strafeTo, turnTo, followTrajectory x2, drive, brake, hold, cancel, waitUntil<Pred>, pose, setPose, strafeAuthority, lastExitReason, lastCompleted, motionConfig, deps, scheduler x2 (+non-copyable). Time is raw double in: MotionOptions.timeoutSeconds, hold(seconds), waitUntil(timeoutSeconds); Routine::pause/waitFor. Next: roadmap Freeze Register + guide-maintenance + RESUMING.
[02:21:33] Read roadmap Freeze Register (rows F1-F9, preamble lines 115-117: "changes only with a schemaVersion/API-version bump and a migration path"), guide-maintenance.md (D2 row: remove the not-frozen notice from the API chapter), RESUMING.md. Required reading COMPLETE. Kicking off baseline build+suite to confirm the 686/916,026 entry state.
[02:21:55] Baseline verified: suite 686 cases / 916,026 assertions green (3 skips) — matches D1 exit state exactly. Full output captured to scratchpad for post-retype bit-compare. Call-site survey: 101 timeoutSeconds occurrences (100 in test/). units::Time ctor confirmed EXPLICIT (quantity.hpp:33) — a bare double cannot silently become Time, which is what makes the B1 retype a compile-caught migration.

== THE DECISION DOCKET — rulings ==

[02:24:13] A1 RULED: the F6 row will ENUMERATE the full surface by group (construction / verbs / control / state / Tier-3 seam / three public types / documented semantics) and NAME the exclusions (Routine+RoutineResult+RoutineStopCause; the scheduler's own surface behind scheduler(); lower-layer config fields reached through ChassisConfig). Rejected: keep the five-verb row — fails "nobody can later say what was frozen".
[02:24:13] A2 RULED: Routine does NOT join F6 — accepted D1 §12's recommendation, having checked: (a) D3's cookbook is Routine's second consumer and may still reshape steps; (b) Routine delegates only, so freezing the facade fixes everything recipes rely on; (c) the B1 retype lands in Routine NOW anyway, so D3 inherits typed time. The row will say the exclusion out loud (silence in chassis/ reads as frozen). Rejected: freeze Routine too — repeats the mistake the C4->D1->D2 order was built to avoid (freezing before a second consumer).
[02:24:13] A3 RULED: ChassisConfig, MotionOptions, TrajectoryResult ARE frozen (they appear in frozen signatures — a frozen signature over an unfrozen type freezes nothing). Carve-outs stated: MotionOptions' EXISTING fields frozen, field SET additive-open (C4 D10's design); ChassisConfig's two members frozen, the member types' own fields belong to their layers; TrajectoryResult fully frozen incl. succeeded(). Rejected: signatures-only freeze.
[02:24:13] A4 RULED: deps()/scheduler() ARE inside F6 — accepted C4 §8's argument (freezing the facade without the seam freezes users OUT of the lower layers; the no-ceiling promise is load-bearing). Frozen: the accessors' existence and exact types. NOT frozen (named exclusion): MotionScheduler's and MotionDeps' own member surfaces — those are C1/C2's layers; claiming them for F6 would freeze two whole layers by the back door. Under-claim per constraint 1.
[02:24:47] B1 RULED: ADOPT the time retype. Time joins Length/Velocity/Angle as typed on the whole surface: MotionOptions.timeoutSeconds -> units::Time timeout (RENAMED so no call site survives unexamined), hold(units::Time), waitUntil(pred, units::Time); Routine pause/waitFor/hold retyped to match (unfrozen, but the carve-out dies everywhere at once). Facade converts at the motion boundary with .value() — F3 makes Time canonical seconds, so the conversion is the identity and bit-preservation is by construction (still to be PROVEN: staged suite runs, identical 686/916,026 + full-output diff before any new test lands). Rejected: keep raw doubles + document the carve-out — enshrines the one untyped dimension on a surface whose whole misuse story is "wrong units do not compile"; hold(500) is a match-loss that currently compiles.
[02:24:47] B2 RULED: ADOPT wait(units::Time) as an additive facade verb, VOID return. Reasons: universally needed beat; naive spelling Warn-spams (transcript lies about intent); working spelling needs Tier-3 plumbing + sacrificial timeout = a Tier-2 concept requiring Tier-3 plumbing, the no-cliff constraint violated. Void because a wait has no failure mode short of a stalled pacer (a loud precondition, not a verdict) — returning ExitReason/WaitResult would invite dead branches on an unreachable timeout. Routine::pause becomes a pure delegation (restoring "every step is exactly one Chassis call" — pause was the ONE step with Tier-3 plumbing). kPauseBackstopSeconds moves to chassis.hpp as kWaitBackstopSeconds, same value, same deliberate non-HA rationale. Precondition finite && > 0 (consistent with hold/pause; a computed possibly-zero wait belongs at waitUntil's tier where 0 is an honest poll). Rejected: name it sleep (implies thread semantics the single-task model does not have); decline + bless the predicate idiom (leaves every team rediscovering the Warn problem).
[02:24:47] B3 RULED: ADOPT brake/hold into F6 — accepted D1 §2.6, having checked: every complete D1 routine used both; parking via the Tier-3 seam would be a Tier-2 cliff (§17 forbids); C4 D9 built them precisely for this decision. hold retyped per B1. Rejected: delete before freeze (C4's stated fallback) — an auton surface that cannot park is incomplete.
[02:24:47] C1 RULED: CLOSE the runtime-abortFaultMask deferral — mask is construction-time only at F6. Accepted D1 §2.3, having checked: no D1 test or step design reached for it; the expert path (construction config + Tier-3 seam) exists; C2's Phase-R fault-statistics revisit stands. Closing does not foreclose: a setter would be ADDITIVE later. Rejected: add a setter now — speculative surface in a freeze chunk.
[02:24:47] C2 RULED: KEEP the branch-on-strafe-fallback rejection — accepted D1 §2.4, having checked the precise finding: a waitUntil predicate cannot see the fallback either (no accessor exists by design), so it is unreachable at EVERY tier, and strafeAuthority() covers authoring-time budgeting. Mid-leg reaction is re-planning = G2+. The rejection is additively reversible (a getter could be added later), which is exactly why it need not be reversed now.
[02:24:47] C3 RULED: tank field-vocabulary sugar STAYS in the recipe layer — accepted D1 §2.5, having checked: face/driveTo are argument sugar (bearing at step-run time), bit-identical to the hand idiom (D1 M4/M5/M19 pins), and the UNFROZEN recipe layer can still iterate on spellings before D3's cookbook. Rejected: facade turnToFace/moveToPoint — freezes sugar spellings before the cookbook has shaped them.
[02:24:47] C4 RULED: ACCEPT the three result vocabularies for F6 (ExitReason / TrajectoryResult / WaitResult); FLAG for F2 (its combinators span all three — the designer must read D1 §2.7 first). Checked: each is right locally, only spanning consumers pay, D1 absorbed it in ~30 lines. wait() returning void deliberately adds NO fourth vocabulary.
[02:24:47] C5 RULED: all four C4 §8 tensions CONFIRMED no-change, each with its additive path named: facade-level async -> behind scheduler() until F2 (D1 §3's eager analysis closed it); hold disturbance-radius -> additive MotionOptions field if R-phase shows need; per-leg trajectory results -> TrajectoryResult sufficient at Tier 2, detail is C5 record material; per-verb settle tolerance -> additive MotionOptions field. Every "no" here is safe BECAUSE its additive path exists.
[02:24:47] C6 RULED: maxWheelSpeed additive path CHECKED STRUCTURALLY: ChassisConfig.motion is the whole MotionConfig (chassis.hpp ~163); the facade stores it by whole-struct copy (ctor: cfg_{config.motion}) and effectiveConfig starts from a whole-struct copy (cfg = cfg_) — grep confirms NO field-by-field MotionConfig marshalling anywhere in chassis.hpp/routine.hpp, so a future per-wheel field flows through with zero facade reshape; a per-call override is additive via a new MotionOptions field + one effectiveConfig line. R5 measurement deferral unchanged (C3 D13).

[02:24:47] Docket complete: 9/9 D1 §2 items ruled (2.1 ADOPT, 2.2 ADOPT, 2.3 CLOSE, 2.4 KEEP, 2.5 ACCEPT, 2.6 ADOPT, 2.7 ACCEPT+FLAG, 2.8 CONFIRM x4, 2.9 noted as evidence), A1-A4 + B1-B3 + C1-C6 ruled. Beginning construction: stage 1 = the pure retype (chassis.hpp, routine.hpp, all call sites), verify identical numbers BEFORE anything else lands.
[02:28:18] STAGE 1 (the pure retype) VERIFIED: suite 686 / 916,026 green and the FULL suite output is byte-identical to the pre-retype baseline (diff clean, incl. the hostile-IMU drift MESSAGE digits). The retype moved no number — Time is canonical seconds (F3), so .value() at the motion boundary is the identity. Retyped: MotionOptions.timeout (units::Time, renamed from timeoutSeconds), Chassis::hold(units::Time), Chassis::waitUntil(pred, units::Time); Routine::hold/pause/waitFor to match. 100+ call sites converted (Time{x} in rig tests, _s/_ms literals in guide tests). run_report_e2e sched.waitUntil sites left as double DELIBERATELY — that is the scheduler's interior seconds signature (F3 internal-seconds), not the frozen facade. Next: stage 2 — wait(units::Time) + pause delegation.
[02:29:49] STAGE 2 (wait verb + pause delegation) VERIFIED: Chassis::wait(units::Time) added (void return, Warn-free by construction, backstop kWaitBackstopSeconds=1.0 moved from Routine with its non-HA rationale intact); Routine::pause is now a PURE delegation to Chassis::wait — the one Tier-3-plumbed step is gone, "every step is exactly one Chassis call" is true again. Suite 686 / 916,026 green, output STILL byte-identical to the pre-D2 baseline (pause rides the same sched_.waitUntil machinery one layer down). Next: stage 3 — version.hpp + policy, then the signature-pin test.
[02:35:00] PIN PROOF #1 (hold Time->double reshape): build FAILED (exit 2) — the freeze is enforced. BUT: found a pin-design defect worth recording. The member-exact pins were written as non-template requires-expressions; an invalid static_cast in a NON-dependent context is a HARD error, so the build broke with a raw "invalid static_cast" diagnostic at the pin site instead of the named F6 static_assert (only the terse-spellings CONCEPT produced the named message — concepts are dependent, so the failure stayed inside the immediate context). The brief's bar is "fails the build with a message NAMING F6". Redesigning: every member pin becomes a concept templated on the class, so any reshape is a substitution failure -> concept false -> the named F6 static_assert fires. Restored pristine header; suite re-green 687/916,035.

---

## ⏸ PAUSED 2026-08-11 02:35 — session usage exhausted, NOT a failure

Stopped deliberately, mid-stage-3, by the session owner. **The tree is in a coherent, GREEN
state** — this is a safe pause point, not a broken one.

### State at the pause

- **Suite: 687 cases / 916,035 assertions green**, 3 deliberate skips. Builds clean under
  `-Werror`. (Entry state was 686 / 916,026; the delta is the partial signature-pin test.)
- **Nothing committed.** HEAD is `02b8e2f` (the D2 brief). All D2 work is uncommitted in the
  working tree, which is the protocol's own signal for "a chunk was interrupted".
- **F6 is still 🎯 NOT frozen** — correct, because the pin and the sweep are incomplete.

### What is DONE (and verified by the agent as it went)

- **The decision docket is COMPLETE — 9/9 D1 §2 items plus A1–A4, B1–B3, C1–C6**, each ruled
  with its rejected alternative, all recorded above in this log. That is the hard thinking and
  it survived. Headlines: `Routine` does **not** join F6 (stated out loud in the row);
  time **retyped** to `units::Time`; `wait(units::Time)` **adopted** as an additive void verb;
  `brake`/`hold` **adopted**.
- **Stage 1 — the pure retype: VERIFIED.** `MotionOptions.timeoutSeconds` → `units::Time timeout`
  (deliberately renamed so no call site survives unexamined), `hold(units::Time)`,
  `waitUntil(pred, units::Time)`, plus `Routine::hold/pause/waitFor`. ~100 call sites converted.
  Full suite output **byte-identical** to the pre-retype baseline — the retype moved no number,
  which is the bar the brief set (constraint 3).
- **Stage 2 — `wait` verb: VERIFIED.** `Chassis::wait(units::Time)`, void return, Warn-free.
  `Routine::pause` is now a pure delegation, so "every step is exactly one Chassis call" is true
  again. Output still byte-identical.
- **Stage 3 — STARTED, incomplete:** `include/shulib/version.hpp` exists (57 lines: `kApiMajor=2`,
  `kApiMinor=0`, `kApiVersionString="2.0"`, and the written policy for what breaking vs additive
  means). `test/f6_signature_pin_test.cpp` exists (240 lines, **35 `SHULIB_F6_PIN` invocations**,
  a macro wrapping `static_assert` with an "F6 FREEZE VIOLATION" message).

### ⚠ WHAT IS NOT DONE — resume here, in this order

1. **THE GUIDE HAS DRIFTED — 21 lines. Fix before anything else.** The retype renamed
   `timeoutSeconds` → `timeout` in `test/guide_examples_test.cpp`, but chapters **08** and **09**
   still quote the OLD spelling. The chapters quote the test file **verbatim**; they now lie.
   Re-quote from the test file — never hand-edit the markdown into agreement.
   Check with: `python3` verbatim scan over ```cpp blocks vs the test file, or run
   `docs/internal/verify/verify-d2.sh` §9.
2. **Finish/verify the signature pin.** 35 pins exist; confirm every frozen member is covered,
   then **prove it works**: change one frozen signature, observe the build fail naming F6,
   restore. An unproven pin is decoration (brief §3).
3. **The notice sweep — NOT started.** Still 1 "not frozen" instance in each of `chassis.hpp`,
   guide ch. 09, ch. 10, ch. 14 (plus the guide README's maintenance list). **Atomic** — all
   together or none (brief constraint 5).
4. **Rewrite the F6 register row** to enumerate the surface per ruling A1, name the exclusions,
   and state `Routine`'s status explicitly. Only then flip it to LOCKED with the date.
5. **The mutation campaign** — not started. Break each frozen signature; if the `_ms` conversion
   is reachable, mutate it. Gate the runner on build success.
6. **`D2-COMPLETED.md`** — not started. 570–654 lines, `D1-COMPLETED.md` as the template, the
   decision docket as the centrepiece. Every ruling above is already written here; lift it.
7. **The six-part documentation contract** — roadmap checkbox, "you are here", design notes,
   test evidence, decisions, Freeze Register.

### How to resume

The rulings above are the chunk's real product and they are all here. A fresh agent can be handed
`docs/internal/chunks/D2-f6-freeze.md` plus this log and pick up at item 1 without re-deciding
anything. Re-verify the entry state first: build, suite (expect 687 / 916,035), both guards, ARM
gate — `docs/internal/verify/verify-d2.sh` runs the whole set.

**Do not start any other chunk while this tree is dirty.**
