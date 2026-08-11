# C4 Progress — Chassis facade

22:49:31 START C4: read brief + inherited-shape lists (C2 §11, C3 §11), then survey code before any edit
22:50:15 DONE reading brief + C1/C2/C3 §10-11 + master-plan §16-17 + roadmap F6 + build-order C/D. Inherited-shape list has 8 C2 flags + 5 C3 flags + C1's §11 shapes. Next: code survey.

23:00:42 DONE code survey: motion.hpp/motion_scheduler.hpp/move_to_pose/turn_to/strafe_to/hold_pose/drive_brake, kinematics factories, RobotContext, frame.hpp, test rig, C2 id/fault/boot tests, ci.yml guards + ARM gate

23:00:42 DECIDE facade composition: Chassis BORROWS (MotionDeps + ITickPacer), owns MotionScheduler + MotionConfig. Owning the Localizer rejected: would couple the facade to one odometry wiring and cap Tier 3 (custom estimators); building the pieces stays G1/RobotBuilder territory. This resolves C1 §11 item 1 (facade holds exactly the MotionDeps set).
23:00:42 DECIDE blocking verbs = async + waitUntilSettled (C2 §10 question answered YES). Bounded by the motion watchdog; no second timeout. Async/parallel composition stays at Tier 3 via scheduler() accessor — F2 combinators own richer composition later.
23:00:42 DECIDE no background task: ITickPacer stays the injected seam (C2 §11 flag 5 resolved BEFORE D2). A background task would be the tree's first two-task design, unbuildable PROS-free, and would kill host determinism. Robot-side pacer lands at R1/R3.
23:00:42 DECIDE drive(ChassisSpeeds, Frame): Frame is a NEW enum in math/frame.hpp (Field/Body), REQUIRED parameter, no default — silent frame confusion becomes a compile error. drive() pre-empts an active motion via scheduler cancel (C2 pre-empt semantics: a new order supersedes, safely), owns one loop iteration (localizer.update first — a teleop loop that never updates the estimate would rot the field-frame rotation = THE frame bug), field-frame during Uninitialized commands ZERO volts + one Warn (garbage heading must not reach the rotation).
23:00:42 DECIDE extract the saturation choreography into motion/command_pipeline.hpp (C1 §11 item 4: drive() must REUSE the pipeline, not re-derive). MoveToPose + TurnTo refactored onto it; bit-identity guarded by C2's scheduled==hand-chained tests. Health-observable tick also extracted (motion.hpp helper) — 3 copies existed, facade would have been the 4th.
23:00:42 DECIDE brake() + hold(s) INCLUDED as facade verbs beyond the roadmap's five, flagged as F6 CANDIDATE ADDITIONS for D2: an auton that cannot park/hold is incomplete, and DriveBrake/HoldPose otherwise become facade-unreachable (capability lost = Tier 2 cliff).
23:00:42 DECIDE followTrajectory returns TrajectoryResult{exit, completedLegs, totalLegs}; chains MoveToPose legs; STOPS at first non-Settled leg (chasing later waypoints while lost compounds blindly); per-LEG timeout (matches the watchdog structure); markers/ids/.vexbot = G2, stated in-header.
23:00:42 DECIDE MotionOptions{timeoutSeconds, maxLinearSpeed, maxAngularSpeed} per call (0 = config default) — per-leg speed caps are table stakes for real autons (slow approach) and options-struct keeps F6 signatures additive-extensible.
23:00:42 DECIDE runBlocking carries an exception DETACH GUARD: if waitUntilSettled throws (e.g. stalled pacer), the stack-owned motion would dangle in the scheduler's active slot — guard cancels (safe state + detach) on unwind. Found by construction; facade-level responsibility.
23:00:42 DECIDE C3 §11 flags: #1 ADOPTED (NO strafeFallbackActive() getter — record stream is the surface); #2 ADOPTED (chassis.strafeAuthority() read-only passthrough); #3 ADOPTED (turn-while-drive documented in chassis.hpp API docs); #4 named for D2 (MotionConfig flows whole through ChassisConfig — additive per-wheel budget path exists); #5 ADOPTED (kinematics injected as value-built configs).
23:00:42 START implementation: math/frame.hpp Frame enum -> motion/command_pipeline.hpp -> refactors -> chassis/chassis.hpp -> tests

23:05:49 DONE pipeline extraction (command_pipeline.hpp) + tickHealthObservables consolidation; MoveToPose/TurnTo/scheduler refactored. Suite: 556 cases / 913,561 assertions — IDENTICAL to the C3 baseline, so the C2 bit-identity suites confirm the extraction added zero physics. Now: chassis/chassis.hpp.

23:09:26 DONE chassis/chassis.hpp — facade with moveTo/strafeTo/turnTo/followTrajectory/drive(speeds,Frame) + candidate brake/hold + cancel/waitUntil/pose/setPose/strafeAuthority + Tier-3 seam (scheduler()/deps()). Strict-compile clean.
23:09:26 FOUND (by construction): a blocking verb's stack-owned motion DANGLES in the scheduler slot if waitUntilSettled throws (stalled pacer / unconverted estimator precondition) — added DetachGuard: cancel-on-unwind clears the slot + safes the drive BEFORE the motion object dies. Will mutation-check it.
23:09:26 START tests: chassis_facade_test.cpp (construction/stamping/misuse/options), chassis_drive_test.cpp (frame sweeps/clamps/pre-empt/boot), chassis_routine_test.cpp (autons x3 drivetrains, twin bit-identity, sweeps, guarantees)

01:07:02 DONE test suites green: 590 cases / 915,150 assertions (baseline was 556/913,561 → +34 cases). KEYSTONE RESULT: facade-vs-scheduler twin BIT-IDENTICAL, clean AND hostile — C1/C2/C3 baselines carry over verbatim through the facade.
01:07:02 FOUND while writing tests (3, each a contract clarification not a defect): (1) TimedOut stops motors at 0V but only CANCEL adds BrakeMode::Brake — two distinct contracts, my first assertion conflated them; (2) a tank auton MUST author-plan a bearing turn before an off-line leg (D12 honesty observed through the facade, exactly as documented); (3) followTrajectory per-leg budgets must cover the PID's exponential settle tail (~1.1s for a 6-in leg) — worth carrying into the F6 docs as guidance.
01:07:02 START verification gates: CI guard 1 (PROS-free), guard 2 (layering), ARM cross-compile of all headers

05:08:35 DONE CI guards re-verified after session interruption: GUARD 1 PASS (PROS-free incl. chassis/chassis.hpp + command_pipeline.hpp), GUARD 2 PASS (layering, core sim-free). Now the ARM gate.

05:10:28 DONE ARM gate: 89 headers, ONE TU, strict flags (-Wall -Wextra -Wconversion -Wsign-conversion -Wshadow -Werror, cortex-a9/softfp) — CLEAN. (87 at C3 → +2: chassis.hpp counts as new? No: chassis.hpp + command_pipeline.hpp are the two new headers.)
05:10:28 START mutation campaign — 18 planned, each break → build → run FULL suite → observe → restore; pristine copies snapshotted for post-campaign cmp. Deliberate green-hole probes included (norm-cap pin M16, drive-health M20).

05:12:24 MUTATE M1 (facade moveTo built from RAW deps — the forgotten-convention bug): RED — 1 case / 138 assertions: 'C4 stamping: every record of every facade verb carries its command id'. Restored, cmp-verified. NOTE process catch: first M1 attempt did not compile (-Werror reorder) and I nearly read a stale-binary green — every subsequent mutation run gates on build success explicitly.

05:12:51 MUTATE M2 (pipeline: fieldToRobot -> robotToField, the classic inverted frame rotation): RED — 41 cases / 51 assertions, spanning C1 frames+sweeps, C2 routines, C3 H-drive, AND C4 drive-frame sweeps + twin. One-pipeline-one-place demonstrated: a frame bug in the shared choreography cannot hide from either the motion tests or the facade tests. Body-frame sweep + heading-0 coincidence pin stayed green exactly as the frame math predicts. Restored.

05:13:33 MUTATE M3 (pipeline: Body input rotated as if Field): RED — 3 cases / 10 assertions, ALL in C4's drive suites (body sweep, lateral sign pin, record frame). STRUCTURAL NOTE (C3-class finding): no motion test can catch this — the only Body-frame motion caller is TurnTo with (0,0,w), which is rotation-invariant. The facade's body-frame sweep is the SOLE detector for this bug class; without C4's tests it lands on a robot. Restored.

05:13:47 MUTATE M4 (pipeline: strafe-authority clamp defeated, vyLimit=1e9): RED — 11 cases / 161 assertions across C1 authority pins, C3's full H-drive visibility suite, tank honesty, AND C4's drive-pipeline + trajectory-SFB cases. The clamp lives in ONE place and every consumer's tests see its removal. Restored.

05:14:03 MUTATE M5 (DetachGuard removed from runBlocking): RED — the unwind test fails 10 assertions and then the suite SEGFAULTS (core dumped, 545 cases skipped): the test's follow-up chassis.cancel() touches the destroyed stack motion still sitting in the scheduler slot. The hazard is a REAL use-after-free, observed as one — cancel-on-unwind is load-bearing, not defensive styling. Restored.

05:14:26 MUTATE M6 (drive() pre-empt removed — the double-commander bug): RED — 1 case / 6 assertions ('C4 drive pre-empt'). Restored.

05:14:54 MUTATE M7 (followTrajectory chases waypoints after a failed leg): RED — 1 case / 3 assertions ('a failing leg STOPS the chain'). Restored.

05:15:12 MUTATE M8 (followTrajectory drops the last waypoint): RED — 3 cases (corner test, H SFB trajectory, the full auton on all three drivetrains). Restored.

05:15:31 MUTATE M9 (effectiveConfig drops the maxLinearSpeed override): RED — 1 case / 1 assertion (the capped-vs-uncapped speed pin; the uncapped twin is what makes it non-vacuous). Restored.

05:16:13 MUTATE M10 (strafeTo + followTrajectory drop options.timeoutSeconds): RED — 2 cases / 6 assertions (tank budget pin catches strafeTo riding the 5s default; the failing-leg case catches the trajectory legs doing the same). Restored.

05:16:28 MUTATE M11 (drive() field-frame boot gate removed — commands rotate by a heading that does not exist yet): RED — 1 case / 7 assertions ('C4 drive boot': nonzero volts, robot moved during calibration, warn missing). Restored.

05:17:11 MUTATE M12 (facade cancel() delegation severed): RED — 2 cases / 15 assertions (panic stop + mid-motion cancel-through-facade). Neither cancel test is vacuous. Restored.

05:17:42 MUTATE M13 (hold() ignores its seconds parameter, hardwired 5.0): RED — 1 case / 2 assertions (window duration pin). Restored.

05:18:25 MUTATE M14 (brake() returns Settled without running DriveBrake): RED — 2 cases / 7 assertions (from-speed truth rest check + the auton counters). Restored.

05:18:58 MUTATE M15 (MoveToPose drops the stall verdict at the consolidated health tick — the refactor's own risk): RED — 6 cases / 14 assertions spanning C1's stall/watchdog pins, C2's fault policy, AND C4's ODO_STUCK-through-the-facade. The tickHealthObservables consolidation kept the wiring load-bearing at every layer. Restored.

05:19:32 MUTATE M16 (pipeline norm cap removed — deliberate green-hole probe): RED — 3 cases / 3 assertions, and the mechanism is informative: (a) C4's maxLinearSpeed option pin, (b) C3's 'X-bot NEVER falls back' invariant (which holds only BECAUSE the cap bounds |body vy| <= vyLimit — the cross-layer pin working exactly as C1 documented), (c) a C1 sentinel-timing case. Honest scope note for the record: the accuracy suites stay green without the cap (desaturate + battery ceiling bound raw physics) — the cap's tested role is direction preservation, the X invariant, and the per-leg budget, not gross accuracy. Restored.

05:19:53 MUTATE M17 (strafe fallback flag forced false — the SILENT fallback): RED — 8 cases / 8 assertions: C3's six-way visibility net PLUS C4's drive-pipeline and trajectory-SFB cases. The C3 'a silent fallback is a 6-way failing test' property is now an 8-way failing test through the facade. Restored.

05:20:13 MUTATE M18 (facade waitUntil stretches the caller's timeout by 3s): RED — 1 case / 1 assertion (the deadline-window pin: >= t0+0.3 AND < t0+0.5). Restored.

05:20:38 MUTATE M19 (drive() never advances the estimate — the stale-teleop bug): RED — 4 cases / 14 assertions (all drive-frame sweeps + boot: the estimate never leaves Uninitialized, so field driving zeroes forever). The 'drive() owns one loop iteration' decision is load-bearing. Restored.

05:25:04 MUTATE M20 (drive() health tick removed — GREEN-HOLE PROBE): stayed GREEN against the original suite (590/915,150 all passed under the mutation, build verified). THE HOLE: nothing pinned that fault observables stay live in a drive()-only teleop loop — IMU_LOST could go dark for a whole driver session. CLOSED with 'C4 drive health: fault observables stay live in a drive()-only teleop loop' (IMU dropout mid-teleop must latch ImuLost with zero motions started); re-run under the mutation: RED — 1 case / 1 assertion, exactly the new test. Restored.

05:26:07 MUTATE M21 (pipeline omega clamp defeated — SECOND GREEN HOLE): stayed GREEN against the original suite (591/915,467 all passed; desaturate keeps every turn CONVERGENT, so no accuracy/closed-loop test can see the yaw-rate budget — the C3-class structural lesson again). Also exposed that the maxAngularSpeed OPTION had no behavioral pin (M9's sibling would have been green too). CLOSED with 'C4 options: maxAngularSpeed caps the true yaw rate' (truth max yaw-step vs option budget + uncapped twin + config-budget bound); under the mutation: RED — 1 case / 2 assertions. Restored.

05:26:45 MUTATE M22 (pipeline battery ceiling dropped): RED — 1 case / 1 assertion (C1's weak-pack ceiling sweep — the pin migrated intact with the extraction). Restored.
05:26:45 DONE mutation campaign: 22 run (M1-M22), 20 RED as designed, 2 GREEN HOLES found AND closed with new tests (M20 teleop health dark, M21 yaw-rate budget invisible to closed-loop tests), 1 process catch (M1's first form did not compile — stale-binary green nearly misread; runner now gates on build). Now: post-campaign integrity + full re-green.

05:27:56 DONE final re-green: 592 cases / 915,157 assertions, 0 failed, 3 skipped (the pre-existing M3/R3 stubs). Post-campaign: all 6 mutated headers cmp-identical to pristine; only 'MUTATION' hit in include/ is truth_integrator's pre-existing doc comment.
05:27:56 START documentation contract: roadmap checkboxes + you-are-here, build-order position, register reconciliation, C4-COMPLETED.md

05:40:01 DONE documentation contract: roadmap facade checkbox [~] with evidence + you-are-here -> C4 done/C5 next + F6 register row annotated (still PENDING D2, deliberately); build-order current-position C4 paragraph + post-C4 verified numbers; register reconciliation clean (zero orphans, ZERO new HA entries — the facade invents no constants); C4-COMPLETED.md written at 600 lines (13 sections, §2 = the 18-item inherited-shape ledger, §8 = the F6 candidate surface for D1/D2, §9 = known/not-known).
05:40:01 DONE C4. Final: suite 592 cases / 915,157 assertions green (-Werror), 3 pre-existing skips; GUARD 1 + GUARD 2 PASS; ARM gate CLEAN (89 headers, one TU, strict flags). 22 mutations: 20 red / 2 green holes closed (M20 teleop health, M21 yaw budget) / 1 process catch. F6 NOT frozen. Nothing committed — working tree left for review at HEAD 38f880d.
