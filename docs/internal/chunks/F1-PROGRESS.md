# F1 (mechanism seam) — live progress log

02:20:04 START. Creating progress log before reading anything, per chunk instructions.
02:20:15 Read brief (F1-mechanism-seam.md) and RESUMING.md in full. Key points captured: name collision (chunk F1 vs register row F1=coordinate frame), seven rulings T1-T7, scope creep into F2 is top risk, fake-must-lie trap, named mutations list. Now reading motion.hpp, hal/motor.hpp, fake_motor.hpp, routine.hpp.
02:20:27 Read motion.hpp (full 258), hal/motor.hpp, fake_motor.hpp, routine.hpp (full). Notes: cancel contract mirrors exactly the four bullets; then() accepts void/bool/ExitReason via static_assert on is_constructible_v<bool,R>; RoutineStopCause::ActionFailed already exists. Next: D1 §2.7, scheduler fault policy + waitUntil, fault.hpp, master plan, build-order, composed.hpp, E4 record.
02:20:41 Read fault.hpp (last code Implausible=10, kCodeSlots=32) and scheduler fault policy + waitUntil + re-entrancy guards. Key: waitUntil pred may call async/cancel but NOT blocking verbs; predicate runs before tickImpl each iteration. A mechanism ticked from the predicate is legal by construction. Next: master plan sections, build-order, composed.hpp, E4 record, exit_group, robot_context, debug_record.
02:21:06 Read master plan §6/§14/§16.3/§17, build-order Current position + §F1/§F2, composed.hpp (A3 chain pattern). Now reading E4-COMPLETED (tone), exit_group.hpp, robot_context.hpp, debug_record TickPhase, vision.hpp classId convention, cookbook stand-in Intake struct.
02:21:30 Read E4-COMPLETED in full — tone/rigor calibrated (measure before claim; retract when measured; green mutations recorded with measurements). Now surveying docs to be edited: guide ch9/13/14, cookbook, roadmap rows, api_doc_tool TARGETS, HA register tail.
02:22:06 Surveyed roadmap (F4 row, M4 WS7 line 1054, M7 1147/1153), guide ch9 the-then-section, ch14 placeholder sections, cookbook claims. Possible stale-prose finding to verify while editing: guide/14 line ~197 says "No recipe cookbook or generated API reference site yet (D3)" — cookbook and docs/api exist since D3; must check context when editing ch14. Now: ch13 structure, test conventions, fake_clock, verify-e4.sh harness, api_doc_tool TARGETS.
02:30:29 DESIGN DRAFT (pre-implementation) written after full required reading. T1: two layers — hal::IMechanism (minimal virtual surface: applySafeState+name+concrete claim token) + concrete MotorMechanism/PneumaticMechanism over IMotor*/IDigitalOut*; new hal::IDigitalOut; operation contract in manipulation/ (L2-peer of motion/, NOT above chassis — will argue vs master plan §6 placement). T2: separate MechanismOutcome enum {Running,Succeeded,Unconfirmed,TimedOut,Cancelled,Stalled}; then() gains a 4th accepted return type; RoutineStopCause appends MechanismFailed. T3: tick-only, no blocking helper (the blocking idiom is chassis.waitUntil ticking the op in the pred); one-op-per-mechanism structural via claim token + precondition. T4: per-mechanism declared safe state at construction, applied on EVERY exit path incl. success (an op is bounded; open-ended behavior = direct mechanism command). T5: leaning ruling (b) — fix the prose; intake.in as callable-data-member would shape team code around a marketing line; will confirm. T6: mint ONLY MechanismStalled=11 (stall/jam = pathology); op timeout with healthy device = strategy (Warn, no fault) — this DIVERGES from C1 MotionTimeout precedent, will argue; Unconfirmed = verdict only. T7: RobotContext does NOT grow; MechanismDeps{clock,faults,telemetry} mirrors MotionDeps; F7 schema gap found: mechanisms[] has no declared-safe-state field — named handoff. Freeze note: new register row F11 marked not-frozen. OPEN: TickPhase::User producer feasibility — checking attribution plumbing now.
02:34:38 RULINGS FINALIZED (full reasoning to follow in COMPLETED record):
  T1: hal::IMechanism = minimal virtual surface (applySafeState+name) + concrete non-virtual claim token; MotorMechanism + PneumaticMechanism ship beside it; new IDigitalOut. The interface earns its keep ONLY via the park-guard use case (heterogeneous mechanisms, one uniform force-safe verb) — a fatter interface would lie for solenoids. manipulation/ op layer sits BESIDE motion/ (L2), not above chassis — will argue vs master plan §6 skills-layer placement.
  T2: separate MechanismOutcome vocabulary {Running=0,Succeeded=1,Unconfirmed=2,TimedOut=3,Cancelled=4,Stalled=5}; then() gains 4th accepted return type; RoutineStopCause appends MechanismFailed; scoped enum cannot convert to bool so a truthy-Unconfirmed accident is a compile error.
  T3: tick-only ops; blocking idiom = chassis.waitUntil ticking op in pred (inherits every C2 guard; no second loop owner exists at all); one-op-per-mechanism structural via claim + loud precondition; pre-empt is F2 policy (cancel-then-start).
  T4: per-mechanism safe state declared at construction, applied on EVERY exit incl. success (an operation is bounded; open-ended spinning = direct mechanism command, not an op). HA-92 = Hold-holds-a-loaded-lift claim.
  T5: RULING (b) — fix the prose. intake.in-as-callable-member would shape team code around a marketing line; the honest idiom is then([&]{ intake.in(); }) / returning op outcome. All quoted places edited.
  T6: mint ONLY MechanismStalled=11 (jam/stall = pathology, stall detector proves it). Op timeout = strategy (Warn, no fault) — waitUntil precedent, NOT MotionTimeout precedent; a mechanism does not control whether the world cooperates. Unconfirmed = verdict only. Both new+old codes land continue-degraded (default abort mask unchanged) — deliberate, tested. TickPhase::User: NOT populated, reasoned — the only visible site (waitUntil pred) sits outside the D-3 attribution bracket; crediting it breaks the pinned sum contract; producer delegated by name to F2 sequencer loop / G2 marker dispatch.
  T7: RobotContext does not grow; MechanismDeps{clock,faults,telemetry} mirrors MotionDeps named-pointer+validate; register gains row F11 marked NOT frozen; F7 schema handoff: mechanisms[] needs a declared-safe-state field.
Starting implementation.
02:41:16 Headers written: hal/digital_out.hpp, hal/mechanism.hpp, hal/fake/fake_digital_out.hpp, hal/fake/fake_mechanism.hpp, manipulation/{mechanism_outcome,stall_detector,mechanism_op}.hpp, sim/hostile/mechanism_hostility.hpp; FaultCode::MechanismStalled=11 appended. Namespace of hostile models corrected to shulib::sim to match A3. Next: routine.hpp then() branch + RoutineStopCause append.
02:45:45 DESIGN CORRECTION (found by writing the expected timeline BY HAND before any test ran — the brief-mandated counter to the shared-model trap doing its job at design time): "every exit applies the safe state" is WRONG for a discrete actuator. A clamp whose declared safe command is OPEN would fling its goal the instant a grab SUCCEEDED. The motor rationale (stop the energy input) does not transfer: a solenoid consumes no energy holding state, and un-commanding a completed actuation would undo the very action. RULED SPLIT: motor-mechanism ops apply the declared safe state on EVERY exit (energy hazard); discrete-actuator ops leave the commanded state in place on Succeeded/Unconfirmed and apply the declared safe state ONLY on cancel (the outside hammer — which is precisely the park-guard path, where the DECLARED value expresses keep-the-goal vs open). Header being amended.
02:46:19 COORDINATOR CORRECTIONS RECEIVED (red-team of the brief; overrides brief where conflicting). Assessment against current state:
  (1) BLOCKING version-pin defect: I have already appended RoutineStopCause::MechanismFailed, so this bites NOW. Will follow version.hpp documented additive path (kApiMinor bump) and fix the D2/D3 pin conflation (pin major==2 + mechanism exists, not minor==0) as a Rule 4 fix, recorded as a D2/D3 defect found at F1.
  (2) T5 second break (chassis. receiver has no .then — ExitReason): my ruling was already fix-the-prose; the fix now must ALSO move the receiver to a Routine chain. No Chassis change will be made or proposed.
  (3a) Hostile injection level: already injecting at MOTOR level (velocity/current/position corrupted; detector must earn the verdict). No jammed-flag anywhere.
  (3b) then() one-shot: resolved explicitly — the action loops via chassis.waitUntil ticking the op in the predicate; and a Running outcome returned to then() stops the chain LOUDLY with RUNNING named (an un-driven op can read as neither success nor silence). Adding a test for exactly that.
  (3c) park-guard iteration: F2 guard takes a caller-supplied span<hal::IMechanism*>; the library never owns the list; IMechanism exists precisely to make that span heterogeneous. Will state in record + ch13.
  (3d) pneumatic safe state: generalized as declared bool safe command, asserted at FakeDigitalOut bottom; plus the safe-on-success split ruled earlier today.
  (3e) IMechanismOp does NOT inherit IMotion (separate hierarchy; async(op) is a compile error). Adding a static_assert pin expressing that intent.
02:54:15 First build+run: 944 cases, ONE red — my interrupted-stall-window timeline said tick 10, code said 11. The test was right and my expectation wrong: accumulated 0.01 clock steps carry decimal FP dust, so the ideal-arithmetic literal was off by one. Fixed the TEST (not the assertion bar): binary-exact dt=2^-7 and persistence=5dt make the hand timeline exactly the float timeline; expectation tick 10 now exact. Logged as a lesson: hand timelines must use binary-exact times or they pin lies.

=== RESUMPTION CHECKPOINT ===
Written at 02:58:44 on coordinator request (session may terminate). Audience: a fresh agent
with the brief + the coordinator's corrections but no memory of this session. The tree is DIRTY on
purpose; nothing is committed; do not revert anything.

--- 1. File state (all work is in the working tree) ---
NEW, complete, compiling, suite-green as of the 944-case run:
  include/shulib/hal/digital_out.hpp        — IDigitalOut (set/commanded; no-feedback rationale). Complete.
  include/shulib/hal/mechanism.hpp          — IMechanism (applySafeState+name virtual, concrete claim
                                              token tryClaim/releaseClaim/claimed), MotorMechanism
                                              (fan-out, device readback, maxCurrent/meanVelocity,
                                              declared BrakeMode safe state, HA-92 cited),
                                              PneumaticMechanism (declared bool safe). Complete.
  include/shulib/hal/fake/fake_digital_out.hpp — FakeDigitalOut (commanded + setCount). Complete.
  include/shulib/hal/fake/fake_mechanism.hpp   — FakeMechanism (counts applySafeState; the H2/no-device
                                              proof + F2 park-guard double). Complete.
  include/shulib/manipulation/mechanism_outcome.hpp — MechanismOutcome enum {Running=0,Succeeded=1,
                                              Unconfirmed=2,TimedOut=3,Cancelled=4,Stalled=5} +
                                              mechanismOutcomeName. Complete.
  include/shulib/manipulation/stall_detector.hpp — StallConfig (REQUIRED thresholds, no defaults) +
                                              StallDetector (persistence window, resets on healthy
                                              sample). Complete.
  include/shulib/manipulation/mechanism_op.hpp — MechanismDeps{clock,faults,telemetry}+validate;
                                              IMechanismOp (start/tick/cancel/outcome/started/name,
                                              concrete finished()); AlwaysConfirmed; RunUntilConfirmed
                                              (motor op: confirm-first order, stall→FaultCode::
                                              MechanismStalled raise, watchdog TimedOut Warn-no-fault,
                                              safe state on EVERY exit); ActuateAndConfirm (discrete
                                              op: deadline pair as watchdog, pred suppressed until
                                              actDeadline, Unconfirmed at confirmDeadline, commanded
                                              state PERSISTS on Succeeded/Unconfirmed, safe state only
                                              on cancel). Complete. Banner carries the full contract
                                              mirror + where mirroring stops.
  include/shulib/sim/hostile/mechanism_hostility.hpp — namespace shulib::sim (A3 convention):
                                              JammedMotorConfig/JammedMotor (IMotor decorator, window,
                                              stall current scales with |commanded|/12, creep, frozen
                                              position; defaults HA-93 PROVISIONAL), LyingSpinMotor
                                              (position=truth frozen, velocity/current lie healthy; NO
                                              defaults on purpose), NeverConfirm, ConfirmAfter. Complete.
  test/mechanism_test_rig.hpp               — RecordingMotor (ordered event list, everEnergized),
                                              OpRig (clock+sink+latch+deps), drive() helper (tick i at
                                              t=i*dt), countMechWarns. Complete.
  test/mechanism_test.cpp                   — device-seam suite (9 cases). Complete, green.
  test/mechanism_op_test.cpp                — operation-contract suite (~15 cases incl. hand
                                              timelines, cancel contract 4 clauses, adversarial clock,
                                              claim collision, static_asserts !is_base_of IMotion and
                                              !convertible-to-bool). Complete, green after the FP fix
                                              (see lesson at 04:0x entry: binary-exact dt).
  test/mechanism_routine_test.cpp           — JUST WRITTEN, NOT YET COMPILED. Concurrency proof
                                              (scheduler waitUntil, integer-exact 41-tick arithmetic,
                                              bit-identical pose twin), abort-mask consequence case,
                                              then() idiom cases (Succeeded continues; UNCONFIRMED
                                              stops+no fault; STALLED stops+fault; Running-returned
                                              stops loudly; legacy void/bool/ExitReason unchanged).
                                              NEXT ACTION = build this file and fix compile errors.
MODIFIED, complete:
  include/shulib/diag/fault.hpp             — appended MechanismStalled=11 + name "MECHANISM_STALLED".
  include/shulib/chassis/routine.hpp        — then() gains MechanismOutcome branch (only member
                                              changed besides private recordStop detail param +
                                              stopCauseText case + appended RoutineStopCause::
                                              MechanismFailed); banner updated (unfrozen note, example
                                              line, then() section with T5 honesty note). Includes
                                              manipulation/mechanism_outcome.hpp.
  include/shulib/diag/debug_record.hpp      — TickPhase::User comment now records the F1 ruling (no
                                              producer; why; F2/G2 named).
  include/shulib/version.hpp                — kApiMinor 0→1, string "2.1", reasoning comment (the F1
                                              additive set + the D2/D3 pin conflation note).
  test/f6_signature_pin_test.cpp            — version pin fixed: kApiMajor==2 && kApiMinor>=0, with
                                              HISTORY comment recording the D2/D3 conflation defect.
  test/routine_signature_pin_test.cpp       — same fix + HISTORY; added MechanismFailed==4 pin.
  test/fault_test.cpp                       — value pin 11 + name pin MECHANISM_STALLED.
  test/blackbox_format_test.cpp             — MechanismStalled added to the round-trip fault list.
  docs/api/chassis.md, routine.md, README.md — regenerated via `python3 tools/api_doc_tool.py generate`
                                              (check-fresh gate demanded it). Regenerate again if
                                              routine.hpp docs change further.
Suite state at last full run: 944 cases / 1,521,691 assertions, 0 failed, 3 deliberate skips.

--- 2. Exact next steps, in order ---
 a. `cmake --build build/test -j$(nproc)` — fix compile errors in test/mechanism_routine_test.cpp
    (not yet compiled; watch: WaitResult include comes from motion_scheduler via chassis.hpp;
    unused includes under -Werror; MotionOptions brace init field names).
 b. Write test/mechanism_hostile_test.cpp: (1) jam liveness pin (JammedMotor measurably alters
    velocity/current/position vs pass-through — the mutation "jam injection no-op" must go red
    here); (2) mid-run jam → Stalled at hand tick (use dt=2^-7, window start 32dt=0.25 exact,
    persistence 5dt, command 12V so jam current 2.5A clears the 2.0A threshold — at 6V the scaled
    current 1.25A would NOT trip, that is deliberate physics in JammedMotor); (3) stalled lift from
    t=0, declared Hold, assert INNER FakeMotor brakeMode==Hold through the wrapper; (4) LyingSpinMotor:
    stall cfg that would trip if honest; assert detector never trips, no fault, TimedOut at watchdog
    tick (timeout 0.5=64dt → tick 64), and position-frozen truth channel; (5) unconfirmed-grab world
    via NeverConfirm + ActuateAndConfirm; (6) ConfirmAfter within window → Succeeded at exact tick;
    (7) ConfirmAfter{clock,0} over a jam = the lying-true confirm → false Succeeded asserted AS the
    documented trust boundary; (8) composition/ablation: nested JammedMotor wrappers with two
    windows, removing one removes exactly that window.
 c. Full suite green; then the two CI guards + ARM gate (commands in the chunk prompt §5).
 d. Mutation campaign: write docs/internal/verify/verify-f1.sh CLONED FROM verify-e4.sh (same
    trap/backup/byte-compare/build-gate skeleton; never git checkout; never pipe to head). Mutations
    (targets): 1 watchdog disarm — in RunUntilConfirmed::tick remove/invert `watchdog_.expired()`;
    2 cancel skips safe state — delete `mech_->applySafeState();` in RunUntilConfirmed::cancel;
    3 cancel overwrites verdict — move outcome_=Cancelled outside the !finished_ guard;
    4a/4b safe-state swap — MotorMechanism::applySafeState `setBrakeMode(safe_)` → Hold and → Coast
    (two runs; lift case kills one, intake case the other);
    5 stall always healthy — StallDetector::update `return (now - windowStart_)...` → `return false`;
    6 unconfirmed reports success — ActuateAndConfirm Unconfirmed branch → finish(Succeeded);
    7 fault dropped, verdict kept — delete raiseStalled() call;
    8 jam no-op — JammedMotor::jammed() → return false;
    plus worth running: then() MechanismFailed branch mapping (succeeded = mo != Succeeded inverted),
    claim precondition removed (tryClaim result ignored), pred-suppression removed in ActuateAndConfirm
    (confirm consulted during actuation), RunUntilConfirmed confirm-first order swapped with stall.
    Gate each on build success; expect ALL red; any green = a hole to close with a new test and a
    prominent record entry. NONE EXECUTED YET.
 e. Documentation pass (NONE STARTED except docs/api regen): guide ch9 (rewrite the then() section
    — mechanisms exist now; spell the waitUntil idiom; T5 honesty incl. the SECOND break from the
    coordinator: `chassis.moveTo(p)` returns ExitReason so the flagship line needs a Routine
    receiver); ch13 (new section: writing a mechanism against the seam — team-owned structs, the
    park-guard span answer to correction 3(c)); ch14 (rewrite "The mechanism seam is a placeholder"
    §, rewrite the "No mechanisms for recipes to command" bullet in measured/measured-against/
    unmeasured order; VERIFY the suspicious line ~197 "No recipe cookbook or generated API reference
    site yet (D3)" — cookbook EXISTS since D3, likely stale prose defect, fix if so (Rule 4));
    cookbook README.md:101-104 bullet + 01-getting-there.md:65-67 (the `intake.release` claim is
    false as literal C++ — reword to the lambda idiom; do NOT change the quoted Intake struct, it is
    gate-bound to cookbook_examples_test.cpp); master plan §17 line 713 tier-table cell (fix BOTH
    breaks: Routine receiver + lambda spelling), §6 line ~214 module map (manipulation/ begins at F1
    with the op contract, architecturally beside motion/, below chassis — my T1 layering ruling
    argues master plan's skills-layer placement is about F3's concrete primitives, not the contract),
    F4 scope note lines 293-297 (mechanism abstraction no longer deferred — landed at F1 OUTSIDE the
    F4 freeze); roadmap.md M4 WS7 line 1054 checkbox (recommend [~] naming freeze-after-F3 + nothing
    on hardware; decide with evidence), M7 lines 1147/1153 (the [~] reason was ".then(intake.in)
    cannot exist" — resolve per T5: the honest spelling now EXISTS and runs; weigh flipping to [x]
    with the T5 correction named), Freeze Register: ADD ROW F11 🎯 (mechanism seam + op contract,
    NOT frozen, freezes after second consumer F3; do NOT touch rows F1-F5, do NOT edit F4's cell),
    "you are here" update; hardware-assumptions.md: HA-92 (Hold holds a loaded cascade lift — cited
    in hal/mechanism.hpp banner), HA-93 (jam signature defaults 2.5A@12V + 0.05 rad/s creep — cited
    in mechanism_hostility.hpp), HA-94 RESERVED for any actuation-time magnitude the docs invent
    (only register if a doc/test states one as physics); build-order.md "Current position" + §F1
    entry RULED block (record: T1-T7 rulings, the version-pin Rule 4 fix, the discrete-actuator
    safe-state split, TickPhase::User delegation to F2/G2); check guide-maintenance.md if ch13/14
    section lists are tracked there.
 f. api_doc_tool TARGETS ruling (T-docs): decide whether hal/mechanism.hpp + manipulation headers
    join TARGETS now or at freeze (F3). My leaning, not yet final: DEFER to the freeze chunk with
    the reasoning recorded loudly in build-order F3's entry AND in F1-COMPLETED (the D3 "exclusion
    lists are where holes live" counterargument must be addressed head-on; the counter-counter is
    that the surface is deliberately unfrozen and gate-churning generated docs on an unfrozen
    surface pins the wrong thing). If deferring, state it in the F11 register row too.
 g. F1-COMPLETED.md per the documentation contract: T1, T2, T4 get their own sections; separate
    section for the discrete-actuator safe-state split finding (coordinator agrees it merits one);
    the two brief defects (version-pin conflation = D2/D3 defect fixed at F1; T5's second break)
    each recorded; mutation table with real counts (PENDING MEASUREMENT until run); "what a
    mechanism can and cannot do now, what F2 may assume" section; named handoffs (F2: park guard
    takes caller-supplied span<IMechanism*>, pre-empt = cancel-then-start policy, TickPhase::User
    producer; F3: thresholds + freeze + confirm predicates on real sensors; G1: mechanisms[] schema
    needs a declared-safe-state field — F7 handoff; R1: IDigitalOut over ADI; R4: HA-92/93).
 h. Final verification: suite tail -6, GUARD1/GUARD2, ARM gate (expect header count to GROW ~8 from
    115), all four doc gates, register reconciliation greps (HA-92/93 cited in tree), removability
    check (no public doc references docs/internal).

--- 3. Mutation campaign status ---
NOT STARTED. No runner script exists yet. Planned mutations and their exact target lines are in
step (d) above. The named-eight from the brief all have designed killer tests already in the tree
(mechanism_op_test.cpp / mechanism_test.cpp / mechanism_routine_test.cpp; the jam-no-op killer will
be the hostile liveness pin, file not yet written).

--- 4. Documentation status ---
Only docs/api/{chassis,routine,README}.md regenerated (build gate forced it). Guide ch9/13/14,
cookbook, master plan, roadmap, HA register, build-order: ALL UNTOUCHED so far. The full edit list
with line numbers is step (e) above.

--- 5. Decisions not yet written anywhere else (beyond the 02:34:38 / 02:45:45 entries) ---
* Version bump executed: 2.0 → 2.1 per version.hpp's own additive policy; the F1 additive set is
  {FaultCode::MechanismStalled, RoutineStopCause::MechanismFailed, then() 4th return type}. The pin
  conflation fix wording lives in both pin files' HISTORY comments.
* T6 final shape: ONLY MechanismStalled minted. NO MechanismTimeout code exists — rejected
  alternative recorded in mechanism_op.hpp banner (a mechanism's success depends on the world;
  waitUntil's no-fault-timeout is the precedent, NOT MotionTimeout, because a motion has authority
  over its own success). Unconfirmed/TimedOut get one Warn line each on the "MECH" subsystem tag.
* Claim semantics: claim gates OPERATIONS only; direct device commands (mech.setVoltage) are NOT
  claim-gated — deliberate, mirrors C2's "direct Tier-3 IMotion use remains possible"; documented in
  mechanism.hpp banner. Pre-empt is F2 policy: cancel-then-start (cancel releases the claim).
* RunUntilConfirmed tick order pinned: confirm FIRST (success beats simultaneous stall/timeout,
  ExitGroup's rule), then stall, then watchdog, then command. True-on-entry = Succeeded with zero
  energizing (C2 pred-before-first-tick shape) — RecordingMotor.everEnergized() is the witness.
* ActuateAndConfirm: pred suppressed before actDeadline (pre-actuation world must not be believed —
  the previous-grab sensor trap); success priority at the exact window edge; deadline pair computed
  once in start() IS the watchdog (redundant second timer rejected — E4's "a defensive line no
  mutation can kill should not exist").
* The jam current SCALES with commanded voltage in JammedMotor (I = stall@12V * |V|/12): a 6V
  command under jam reads 1.25A, below a 2.0A threshold — tests that want a trip must command hard
  or set thresholds per mechanism. This is deliberate physics, not a bug; it is also exactly why
  StallConfig has no library defaults.
* FP lesson (the one red of the session): hand timelines must use binary-exact dt (2^-k) or the
  ideal-arithmetic tick literal can be off by one from the accumulated-float timeline. Recorded in
  the test comment at the fixed subcase.
* The static_assert pair in mechanism_op_test.cpp is the structural answer to correction 3(e)
  (IMechanismOp must not be an IMotion) and to T2's truthiness trap (no bool conversion).
* Layering ruling detail for the record: routine.hpp (L3) including manipulation/mechanism_outcome
  .hpp is the same direction as its existing motion/ include; manipulation/'s F1 content depends
  only on hal+control+diag. The master plan §6 "Skills layer" placement should be read as F3's
  concrete primitives; the §6 edit in step (e) says so.
* MechanismOutcome is NOT on the F9 wire (no DebugRecord slot; mechanisms emit no records — loop
  owner owns the record stream). Values still explicit+append-only for log-line stability.

--- 6. Open questions / known problems ---
* mechanism_routine_test.cpp is uncompiled; expect minor include/usage fixes.
* The bit-identity concurrency assertion (exact ==) SHOULD hold because the mech op touches nothing
  the motion reads; if it fails, investigate before weakening — a failure would mean the predicate
  path perturbs the world (a real finding, not a test bug).
* Roadmap 1147/1153 [~]→? decision deliberately deferred to the doc pass with evidence in hand.
* TARGETS ruling (step f) leaning DEFER but not final.
* Guide ch14 line ~197 possible stale-prose defect — verify against D3's record before editing.
* Nothing in the coordinator corrections is believed wrong; all five judgment items (3a-3e) are
  addressed in code/tests as described above.
=== END CHECKPOINT ===
03:01:22 === CHECKPOINT UPDATE === Hostile suite written and green first run after two pre-run timeline corrections (both caught by re-derivation BEFORE running: (1) jam current scales with commanded voltage so tick 0 reads 0 A and the stalled-lift trip lands at tick 6 not 5; (2) JammedMotor latches frozen position at first in-window READ, so the liveness pin reads once before moving the inner truth). Suite now 959 cases / 1,521,801 assertions, 0 failed. Next: CI guards + ARM gate, then the mutation campaign (verify-f1.sh).
03:03:51 === CHECKPOINT UPDATE === verify-f1.sh written (17 mutations: the 8 named + 9 beyond, incl. the T4-split regression #15 and claim-leak #16/17). Priority-order test added first (960 cases green) after mutation planning exposed that the confirm/stall order swap had no killer. Executing the campaign now — this takes a while (17 full rebuilds).
03:08:49 === CHECKPOINT UPDATE === MUTATION CAMPAIGN COMPLETE: 18 executed (the 8 named + 10 beyond, 4a/4b counting separately), 18 RED, 0 GREEN, 0 build-fail, 0 SKIPPED. Mutation 8 first BUILD-FAILED (unused var under -Werror) and was re-formed to compile (now<0.0) and re-run RED — a build-fail proves nothing (C4). Note for the record: guide-09d landed mid-campaign so later mutations ran against 961 cases, earlier against 960; every mutation ran gated on a successful fresh build either way. No suite hole found by the campaign itself; the one hole found was found EARLIER, by PLANNING mutation 12 (confirm/stall order swap had no killer) and was closed with the same-tick-success-wins test before the campaign ran. Log: scratchpad/f1-mutations-final.log. Docs progress: ch9 rewritten (compiled guide-09d listing, both T5 breaks stated), ch13 Extension 3 written (park-guard span answer included). Next: ch14.
03:14:11 === CHECKPOINT UPDATE === Docs done so far: guide ch9 (rewritten, compiled guide-09d listing, both T5 breaks stated), ch13 (Extension 3: building a mechanism; layer map updated; park-guard span answer), ch14 (placeholder section replaced with measured/measured-against/unmeasured; stale D3 cookbook line FIXED — confirmed stale-prose defect, corrected with a note), cookbook README + 01-getting-there (claims reworded, gate-bound struct untouched), master plan (§17 cell corrected with the double-break note; §6 module map status note; F4 scope note updated — landed as sibling, row F11), roadmap (F11 register row added AFTER F10, F1 you-are-here paragraph + Next:F2, M4 WS7 [~] with evidence, M7 recipe-API flipped [x] with the T5 correction named, D1 paragraph .then claim fixed; one removability slip — docs/internal referenced in roadmap — caught and fixed immediately), HA register (HA-92 R5 group, HA-93 R4 group + index rows), build-order (Current position F1 block + Next:F2; §F1 entry RULED block). Remaining: doc gates + final verification, exact suite numbers into roadmap/build-order (currently ~1.52M placeholder-ish — must replace with measured), TARGETS ruling finalization, F1-COMPLETED.md, final PROGRESS wrap.
03:20:41 === FINAL STATE — CHUNK COMPLETE, NOTHING COMMITTED ===
Suite: 961 cases / 1,521,812 assertions, 0 failed, 3 deliberate skips. GUARD1 PASS, GUARD2 PASS. ARM gate PASS (123 headers). Doc gates: coverage PASS, fresh PASS, examples PASS (361 verbatim lines), removability PASS, self-test OK. Mutations: 18 executed / 18 RED / 0 GREEN / 0 build-fail / 0 skipped (verify-f1.sh; the one suite hole was found while PLANNING mutation 12 and closed pre-campaign; mutation 8 first build-failed and was re-formed + full campaign re-run). HA-92/93 registered and reconciled. API 2.0->2.1 by the documented additive path; D2/D3 version-pin conflation fixed (Rule 4). F1-COMPLETED.md written with T1/T2/T4 own sections + the discrete-actuator safe-state split as its own section + both brief defects recorded + what-F2-may-assume.
NOTE for the verifier: HEAD moved mid-chunk to b80441b (a coordinator commit of an earlier snapshot of verify-f1.sh); my working-tree verify-f1.sh supersedes it (mutation 8 fixed to compile). I committed and pushed NOTHING; 35 dirty/untracked paths are the chunk.
Known honest partials, named: M4 WS7 checkbox kept [~] (unfrozen until F3; no hardware); TickPhase::User empty with producer delegated to F2/G2; TARGETS deferred to the F11 freeze chunk with the ruling recorded; F7 schema needs a declared-safe-state field (G1 handoff).
