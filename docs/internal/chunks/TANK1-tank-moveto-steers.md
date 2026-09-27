# TANK1 — `moveTo` on a tank steers back onto its line: a bounded bend toward the target's line, and the yaw a weak side makes measured and cancelled; never a planned turn

> **THE TEAM LEAD'S GO (2026-09-27, ~08:17): "Go, all recommended".** **D1 (a)** a minor version bump — 2.3,
> since TANK1 lands before L9 Part A (L9 takes the next free minor); the changelog entry is a BEHAVIOUR CHANGE
> listing the three changes, `maxSteer = 0` the way back. **D2 (a)** only legs that start within 15° of their
> target heading are steered. **D3 (a)** two parts: **Part A** (the steering, the flag, the Warn, the tests, its
> documents) now — COMP-t waits only for it — and **Part B** (the ` xt` column) after it, alongside COMP-t.
> **D4 (a)** IMU1 is a separate small chunk (build-order entry), not TANK1's. Tree at launch: `c2473b1` (T4
> Part 1a and the order correction landed after this brief's re-check; neither changes a code file TANK1
> touches — re-read `build-order.md`, HANDOFF, the roadmap and the COMP-t brief at the tip).

> **Execution brief (DRAFT for the team lead's go).** Written 2026-09-27 (~05:00–07:30) from a read-only pass
> over `shulib-v2` at `1237c91`, re-checked at `9e86a3c` (the plan commit that added TANK1's build-order entry;
> it touches no CODE file TANK1 changes, but it does touch `docs/internal/build-order.md`,
> `HANDOFF-remaining-work.md`, `PROJECT-BRIEFING.md`, `docs/roadmap.md` and the COMP-t brief — every one of
> them is on TANK1's documentation list or cited by §6, so the executor re-reads them at the tip it starts
> from). **Probed on the host test plant, not argued:** three candidate laws and
> four variants were run through the real `Chassis`; the proposed code was written as an overlay of the real
> headers and compiled with the suite's flags; **the whole host suite was rebuilt against that overlay and
> passed, 1659 of 1659 cases** (one test file's size pins edited, §2.6); the ARM header gate passed on it; and
> COMP-t's own weak-side probe was re-run through it (§6). **Re-checked the same morning (~07:30–09:30)
> against a checker's eleven findings — see the next section; the amended overlay was rebuilt and re-run the
> same way (suite 1659/1659, 2 465 800 assertions, the ARM gate, COMP-t's probe byte-identical).** **Status:
> WAITS for the team lead's go and his answers to D1–D4. Nothing is built.**
>
> **The team lead's ruling (2026-09-27 ~04:55), which this brief holds:** "Fix the library first" — the
> coordinator's words he approved: *"give moveTo a tank mode that steers back onto the line while it drives,
> the way every tank controller does. It would be a small, bounded correction, and it would never plan a turn
> for you (face() stays your turn). It shows up in the log whenever it's steering. The holonomic drives stay
> byte-for-byte the same, and your auton line doesn't change."* Option text: *"moveTo on a tank steers back onto
> the line (bounded, logged). It builds in parallel with T4's driver file, and the auton tester builds on top
> of it, so its tests expect FINISHED even with a weak side. Adds about a day before the tester."*
>
> **Runs in an isolated worktree** (`.claude/worktrees/tank1`, branch `tank1`, from the tip at launch), in
> parallel with T4 Part 1a (shared files: §5). **Executor model: `opus`. Do not commit.** Live log:
> `docs/internal/chunks/TANK1-PROGRESS.md`, **created FIRST**, appended as you go, a heading per part.
> **Hardware Assumptions numbers: HA-179 and HA-180 ONLY** (T4 has 175/176, COMP-t 177/178); a third belief ⇒
> stop and tell the coordinator. **No `src/` file changes** (the evidence is `git diff --stat -- src/`, empty).
>
> **Probes** (evidence, never project code) — `.claude/coordinator/scratch/brief-TANK1/`:
> `probe_laws.cpp` + `cand/` (every candidate behind one probe switch, same engine otherwise);
> `probe_proposed.cpp` + `proposed/` (**the design exactly as §1 writes it**, compiled twice — against the tree
> and against the overlay — so the X/H hashes compare); `probe_window.cpp` (the engage window);
> `probe_extra.cpp`, `probe_hd0.cpp` (derate-0 H-drive, reset, IMU dropout); `nolatch/` + `probe_unfaced.cpp`
> (D2 (b), and the suite on it: `run_nolatch.txt`); `nofloor/` (C3's floor mutation with TANK1 in);
> `build_suite.sh` (the host suite
> rebuilt by hand against the overlay, into `suite_base/` and `suite_tank1/`; logs `run_base.txt`,
> `run_tank1_final.txt`); `boot_source_test_tank1.cpp` (the pin edit, §2.6); `comp_t_probe3_*` (COMP-t's probe 4
> through TANK1); `sweep*.txt` (the parameter sweeps). The re-check's probes are in `recheck/` (`b.sh` builds
> any of them against an overlay: `recheck/b.sh <out> <src.cpp> <overlay|-> [flags]`); the checker's in `check/`;
> the pre-re-check overlay is kept as `proposed_v1/` and `cand_v1/`. **Start from `proposed/` — it is the
> AMENDED design and it compiles.**

## Re-check 2026-09-27 — amendments

A checker returned eleven findings on this draft. Each was re-verified at the cited lines and on the host test
plant (probes in `recheck/`, named per item); each disposition below says what changed in this brief and where.

1. **BLOCKER — the yaw observer read the IMU's rate a SECOND time each tick. REAL; FIXED at the root.**
   `ProsImu`'s default rate (`DifferentiateRotation`, what `ProsDeviceProvider` builds for robot two —
   `device_provider.hpp` ~:252) differentiates `get_rotation()` between successive CALLS on a microsecond clock
   ("Call it once per tick", `hal/pros/imu.hpp` ~:127-133); the Localizer already made the tick's read
   (`localizer.hpp` ~:364, before the motion ticks, `motion_scheduler.hpp` ~:1016) on the same object
   (`robot.hpp` ~:519/543/549). So on the robot the observer's read was ~0, or a spike when the sensor updated
   in between. `FakeImu` is stateless, so no probe could see it. **Fix:** the observer reads the rate the
   Localizer read THIS tick — `loc.twist().omega().value()` (finite-guarded) — and never calls
   `ctx.imu().yawRate()` (§1.3 item 4, landmine 3). Measured with a ProsImu-shaped IMU wrapper
   (`recheck/probe_imu2.cpp`; first read of each clock instant true, every later one 0, optionally a 3 % ×50
   spike — the checker's assumed rate), 15 cases (the 40 in at R ×1.00–×0.75 and L ×0.91, lateral 2/6, ±8° starts
   at ×0.91, reverse, the acceptance run at ×0.95/×0.91): **the old read** — zeros: 15/15 arrive but every
   drift leg changes (×0.91 xt +0.21, −8° start xt +0.38 at 2.22 s); zeros + spikes: **2/15** (the 40 in times
   out from ×0.97, the acceptance run stops at ×0.95); **the new read — 15/15, per-tick truth and volts
   bit-identical to the plain run, 0 second reads.** On the plain host plant the new read is the same number,
   so every table, hash and suite count in this brief stands (the whole probe output is byte-identical,
   `recheck/pp_fix.txt` vs `pp_v1.txt`). New test **P14** and mutation **M28** (§2). **Pre-existing, handed over
   (D4):** every record's `r.imuYawRate = ctx.imu().yawRate()` is already a second read on the robot
   (`move_to_pose.hpp` ~:386, `turn_to.hpp` ~:238, `drive_brake.hpp` ~:210, `motion_scheduler.hpp` ~:1192,
   `chassis.hpp` ~:584), and a GPS or tag corrector reads before the Localizer's own read on the ticks it reaches
   its yaw-rate gate (`gps_corrector.hpp` ~:268, `apriltag_corrector.hpp` ~:326) — which then corrupts
   `loc.twist()` itself. Robot two has neither corrector. Modelled (a corrector reading first on every other
   tick): zeros only — 15/15 arrive; zeros + spikes — 8/15. HA-179's blast radius names it.
2. **MAJOR — the inertia and chatter evidence covered only the 26 core scenarios. REAL; claims qualified,
   tests extended, no gain change.** Re-run on the amended candidate (`recheck/laws_fix`, `LATCH=1 SWEEP=1`):
   stretch 14/14 at plant kA 0, **11/14 at kA 0.02 (τ 0.12 s: lateral 6 in, lateral 6 in at ×0.91 and reverse
   lateral 6 in TIME OUT at xt +0.56/+0.58/+0.56)**, 10/14 at τ 0.18 s (also R ×0.85, xt +0.60), core 22/26 at
   τ 0.24 s; near-target yaw sign changes over the sweep 6 / 63 / 141 / 190. The drift cases that matter hold
   better: R ×0.91 arrives at τ 0.12 s (1.45 s, 2 sign changes), 0.18 s (2.09 s, 3) and 0.24 s (3.23 s, xt 0.50 —
   the edge). **Is it a design error? No — measured:** the lateral-6 failures end BESIDE the target (along 0.00,
   cross +0.56): the bend overshot the line under lag and the fade then leaves nothing to steer with — a tank
   cannot close a sideways half-inch at the target. Alternatives probed (`recheck/sweep_k*.txt`,
   `sweep_look*.txt`): crossGain 0.08 passes stretch at τ 0.12 s but leaves the clean ±14° start legs 0.46 in off
   the line (0.04 in of margin) at τ 0; a look-ahead damping term passes the lag cases but fails lateral 6 in at
   τ 0. No setting wins everywhere, so the lag-free-plant setting stays (every other shulib default is tuned
   there) and R5 decides with the robot's measured kA (HA-179 settle clause now says so). Changed: §0.2, the
   plain words, §0.3 (inertia rows), P4, **P13 (lag cases with their measured verdicts)**, HA-179, and the docs
   list (no public document may promise a 6 in offset).
3. **MINOR — Part B's `xt` appeared on tank `turnTo` lines. REAL; FIXED.** `hadDirection()` compared the LATEST
   record target with the start, and `TurnTo`/`DriveBrake` publish a target that follows the robot, so a turn a
   weak side drags sideways read as a translation (`TurnTo#1 ... 1.49s xt 0.00"` at ×0.91 — reproduced). Now
   judged on the target the motion published on its FIRST live tick (`firstTarget_`): TurnTo/DriveBrake/HoldPose
   start "here", a translation's target is away. Re-probed: no `xt` on any turn line at ×0.91; every tank moveTo
   line unchanged. §1.5, test 2.4 (a ×0.91 tank turn), **M29**.
4. **MINOR — "yaw nobody commanded" / "cannot wind up on a commanded turn" were too strong. REAL; reworded and
   measured.** The reference is the body ω after the ω clamp but BEFORE desaturation (`command_pipeline.hpp`
   ~:139-140), and whenever the robot cannot follow (lag, desaturation, blocked) the observer integrates the
   heading error like an integral, to `yawCorrectionLimit`. Measured (`recheck/probe_pinned.cpp`, both sides'
   volts forced to 0 for a window): the ×0.91 40 in pinned 0.5 s still ARRIVES (2.40 s; peak correction 0.75 rad/s,
   peak heading error 2.7°) and pinned 1.0 s (3.28 s, 1.50 rad/s, 3.7°) — where TANK1-off and bend-only both
   time out; a lateral-6 leg pinned mid-bend overshoots after release (peak 17.4°) and times out at y +6.53,
   where bend-only arrives. Not gated: by yaw alone "pinned" and "compensating a weak side" look the same (the
   robot turns 0 while ω is commanded), and a translation gate needs a new threshold and fails when pinned
   wheels slip, which drive encoders cannot see (HA-180). Contained by the limit and a τ-length unwind; named
   in HA-179's blast radius; fenced by P8(e). The clamp itself: new P8(d) proves the post-clamp reference.
5. **MINOR — mutation table. REAL; FIXED.** M15 split: **M15a** (last tick's PRE-clamp ω) is equivalent on
   every leg at 6 rad/s — killed only by the new **P8(d)** (maxAngularSpeed 0.6 rad/s, clean lateral 6 in:
   |yawCorrection| peaks 0.028 rad/s, M15a 0.477); **M15b** (this tick's pre-pipeline ω) red on P1 (×0.91 TIMEOUT,
   xt −0.55 — reproduced). M18 is killed ONLY by P7's hostility clause (14 → 2 109 ticks over 10 seeds;
   reproduced) — the executor keeps that clause. M17: a HoldPose that holds "here" at rest is identical under
   M17 on this memoryless plant (probed, also after a coast); P9 now uses the explicit-pose HoldPose (3 in
   ahead, 1 in aside, 1 s) and StrafeTo at ×0.91 — both DIFFERENT under M17, IDENTICAL under the design
   (`recheck/probe_m17.cpp`). M11/M12 confirmed red by the checker.
6. **MINOR — P10(a) made a closed-loop, cross-toolchain golden hash a CI test. REAL; changed.** No closed-loop
   golden exists in `test/` (grep: no FNV); the cited precedent hashes only `forward()` arithmetic. P10(a) is now
   an executor pre/post check (captured on the pristine tree before any edit, re-run after, both logged in
   `TANK1-PROGRESS.md` with the commit) — not a CI test; CI keeps P10(b) (the within-build A/B) and the existing
   C2/C4 suites.
7. **MINOR — coordination under-stated. REAL; §5 rewritten.** R5a DOES write MotionConfig VALUES (`wheelFf` and
   the three speed budgets, `maxAngularSpeed` included — L7 brief ~:160-178, ~:744-756), so the TSTR floor in
   rad/s moves with it (P7's numbers are at 6 rad/s); R5a and SAFE-SAVE both edit guides 11, 12 and 14; DIAG1
   Part B edits `motion_scheduler.hpp` ~:1189, `chassis.hpp` ~:581 and `move_to_pose.hpp` ~:236. H1's lines
   corrected: the tick flags byte ~:357 ("bits 2–7 written 0 and ignored"), its forward-compatibility test
   ~:654; ~:293 is the ENVELOPE flags rule.
8. **MINOR — sentences TANK1 falsifies were missing, and an overlay comment was stale. REAL; FIXED.** Added to
   §3: cookbook 04 ~:33-35 ("never insert a turn on your behalf … silently turns your robot"), `move_to_pose.hpp`
   ~:13-21 (pipeline steps 2-3, "setpoint 0, measurement −eh"), guide 11 ~:159-160 (SFB "the H-drive" only),
   `test/robot_builder_test.cpp` ~:469-471 (a comment), `motion_config.hpp`'s `validate()` doc ("all FIVE"); four
   greps added. The overlay's `debug_record.hpp` comment now says "the bend's share of the heading loop plus the
   yaw correction".
9. **MINOR — "either side" untested on the acceptance run; P3's heading bound had 0.26° of margin. REAL;
   FIXED.** Re-probed both sides (`recheck/probe_accept.cpp`): L ×0.95 FIN 5.78 s, L ×0.91 FIN 6.15 s END
   (0.36, 0.05, −177.14°); L ×0.65 stops at step 1. P3 now runs both sides and grades the heading against the
   `driveTo` leg's OWN target heading (`scheduler().lastCompleted().targetPose`): |error| ≤ 0.02 rad (probe
   ≤ 0.23° at ×0.75–×1.00, both sides), |END| ≤ 0.5 in (≤ 0.38 at ×0.91).
10. **MINOR — §1.7 missed `HeadingMode::Free`, and one number is both cap and window. REAL; documented, not
    split.** A PathRunner Free leg's target heading is the robot's heading at leg start (`path_runner.hpp`
    ~:229-233), so it always engages and an off-line point is steered toward by ≤ 15° — exactly what a
    hand-written `moveTo({x, y, pose().heading()})` does (lateral 6 in: arrives 1.80 s; 20 in: times out, bounded).
    §1.7 and the docs say so. The window stays equal to the cap on purpose — a leg that starts within the cap is
    one whose heading error the bend itself could have commanded; beyond it the heading loop would be making
    the author's turn — and `maxSteer`'s doc and HA-179 now state the coupling and the measured cliff (un-faced
    legs arrive to ~30°, time out from ~40°).
11. **MINOR — D1(a) and the off switch were overstated; header and path errors. REAL; FIXED.** Behaviour
    changes now listed in D1 and the changelog entry: arrived legs take slightly different paths (the clean
    acceptance run END (0.24, 0.12, 180.00°) 5.42 s → (0.23, 0.05, −179.97°) 5.44 s — reproduced on the tree),
    un-faced tank legs gain one MOT Warn, and (Part B) translating tank result lines gain ` xt`. The off switch
    restores motion, records and truth bit for bit, and the log except the ` xt` column. Header note corrected
    (above); `roadmap.md` is `docs/roadmap.md`.

**What the amendments cost (D3):** P14 (a small rig with a ProsImu-shaped IMU), P8(d)/(e), P13's lag cases, P3's
left side and leg-heading grading, P9's explicit hold, a Part B test, five more document lines — Part A grows
from about one day to **about a day and a quarter**.

---

## Decisions for the team lead (a recommendation for each; nothing below decides them silently)

**D1 — The version. TANK1 rewrites a sentence a FROZEN header carries. RECOMMENDED (a): an additive (minor)
bump, with the behaviour change stated in the changelog and a one-line way back.**
- `chassis.hpp` ~:91-98 (inside F6's frozen "strafe-fallback visibility" semantics — `roadmap.md` F6 row ~:140,
  "plus the documented semantics the header carries") says: *"On tank, laterally-offset targets honestly exit
  TimedOut."* After TANK1 some of them arrive. `version.hpp` ~:14-22 calls "changing documented semantics a
  frozen surface carries" BREAKING, and ~:24-27 defines additive as "unchanged behaviour". Read strictly, TANK1
  is neither: nothing stops compiling, but tank programs move differently.
- **(a) RECOMMENDED: a minor bump** (2.3 if TANK1 lands before L9 Part A, which also claims 2.3; otherwise the
  next free minor — the coordinator numbers it at merge). The changed sentence was a statement of a LIMIT, and
  the change relaxes it — the same direction as G2's ruling R6, which the register (F8 row) calls additive
  ("relaxing a `Route::valid()` rule"). **It is not ONLY a relaxed limit, and the entry says so** (re-check
  item 11): every engaged tank leg with any cross-track error or yaw disturbance takes a slightly different
  path, including legs that already arrived (the clean acceptance run's END moves from (0.24, 0.12, 180.00°) at
  5.42 s to (0.23, 0.05, −179.97°) at 5.44 s); an un-faced tank leg gains one `MOT` Warn; and with Part B every
  translating tank result line gains ` xt`. The changelog entry is a **BEHAVIOUR CHANGE** listing those three,
  with its migration: `ChassisConfig.motion.tankSteer.maxSteer = 0` restores the pre-TANK1 tank's **motion,
  records and truth bit for bit** (probed: the ×0.95 40 in times out at y −1.56, exactly today's; `OFFSWITCH`
  hash equal to the pristine tree's) — and its log too, except the ` xt` column Part B adds to translating tank
  result lines whatever `maxSteer` is (a diagnostic, kept on purpose). No F6 signature moves.
- (b) No bump, as T3 Part 2's behaviour change had (changelog "Changed since 2.2"). **Cost:** T3's surface was
  not frozen; this one's sentence is, and a frozen promise changing with no version signal is what the policy
  exists to prevent.
- (c) A major bump (3.0) with a migration note — the strict reading. **Cost:** a major version for an
  improvement with a config opt-out; it would also force Q6 (the open `MotionDeps` major-bump question,
  `version.hpp` ~:71-74) to be settled now.

**D2 — Which tank legs TANK1 steers. RECOMMENDED (a): only a leg that STARTS within 15° of its own target
heading.** This is where "never plans a turn — `face()` stays your turn" becomes code.
- **(a) RECOMMENDED.** At its first live tick, a tank `moveTo` whose heading is within `maxSteer` (15°) of its
  target heading is steered for its whole run; one that starts farther off is **not steered at all** — it runs
  exactly as it does today (pinned bit for bit) and logs ONE Warn:
  `[WARN][MOT] tank moveTo not steered: it starts 26.6 deg off its target heading (steers within 15.0 deg) --
  turn first: face()`. Measured: every leg starting on its line up to ±14.9° off settles (clean and with a 9 %
  weak side, ≤ 2.04 s); at 15.1° it is today's timeout plus the Warn. Legs after `face()`, after `faceAway()`
  (L9), or after a previous leg that arrived, start within ~1° — all steered.
- (b) Steer every tank `moveTo`. **Cost — measured with this design and no window (`nolatch/`,
  `probe_unfaced.cpp`):** an un-faced 40 in `driveTo` then ARRIVES when its point is up to ~30° off the nose
  (20°: 1.82 s; 30°: 1.97 s) and times out from ~40° on — a cliff the author cannot see. C1's heading loop swings
  the robot toward the heading `driveTo` computed from the point, and TANK1 pulls it back onto the line: in
  effect the library making your turn for you, below 35° and not above. It also changes an existing test: the
  suite rebuilt on the no-window variant fails `path_runner_test.cpp` ~:368-370 ("tank, given the X-drive route:
  it fails HONESTLY" — `stoppedAt` 3, not 1; 1658/1659). Under (a) the whole existing suite passes (1659/1659,
  with §2.6's pin edit) and those legs behave exactly as today.

**D3 — One executor run or two, and the honest size. RECOMMENDED (a): two parts, A then B.**
- **Part A — the steering** (the library change, the record flag, the Warn, the tests, every document that
  would be false without it): **about a day and a quarter** (it was "about one day" before the re-check added
  P14's IMU rig, P8(d)/(e), P13's lag cases, P3's left side, P9's explicit hold and five document lines).
  COMP-t waits only for Part A. **Said plainly: the coordinator told the team lead "about a day before the
  tester"; it is about a day and a quarter.**
- **Part B — the result line's cross-track column** (` xt  -0.04"`, the coordinator's "the per-motion result
  reports the final cross-track error"): **about half a day**; it can run while COMP-t is being built (the
  tester prints END y itself; it does not need `xt`).
- **(b) One run, about a day and three quarters.** **Cost:** the tester starts half a day later, and one
  documentation pass of ~11 documents.
- **Said plainly:** even Part A's document list (~9 public documents, §3) is past pace rule 7's ~5, because
  the sentence "a tank cannot reach a sideways target" is in six places, and pace rule 3 forbids landing the
  code while they are false. COMP-b and COMP-t took the same overrun as one step.

**D4 — The IMU's yaw rate is read more than once per tick (PRE-EXISTING, found by the re-check; TANK1 no
longer does it). RECOMMENDED (a): a separate small chunk, IMU1, briefed by the coordinator — it does not block
TANK1 or COMP-t on robot two.**
- **What is wrong today:** `ProsImu`'s default yaw rate differentiates `get_rotation()` between successive
  CALLS (`hal/pros/imu.hpp` ~:127-133, "Call it once per tick"), but the tree calls it several times a tick: the
  Localizer (~:364), then every motion record (`move_to_pose.hpp` ~:386, `turn_to.hpp` ~:238, `drive_brake.hpp`
  ~:210, `motion_scheduler.hpp` ~:1192, `chassis.hpp` ~:584), and — before the Localizer's own read — a GPS or
  tag corrector at its yaw-rate gate (`gps_corrector.hpp` ~:268, `apriltag_corrector.hpp` ~:326). On a robot,
  every read after the first in a tick returns ~0 (or a spike). So **every `imuYawRate` in a record written on
  a robot is ~0 today** (the blackbox, TermSink, H1's wire all carry it), and on a robot with a GPS or tag
  corrector the Localizer's own rate — `loc.twist()`, which TANK1's observer and `DriveBrake` read — is ~0 on
  the ticks a fix reaches that gate. Robot two has neither corrector (IMU + drive encoders), so TANK1's observer
  sees a clean first read there. Modelled (`recheck/probe_imu2.cpp`, a corrector reading first on every other
  tick): zeros only — 15/15 TANK1 cases arrive (xt ≤ 0.27 in); zeros plus a 3 % ×50 spike — 8/15.
- **(a) RECOMMENDED:** IMU1 — make `ProsImu`'s differentiated rate answer every call within one tick with the
  first call's value (for example: a call within a fraction of a tick of the last sample returns the last
  rate), with a host test over the PROS shim on a microsecond-advancing clock. The root fix for all six
  readers at once. It must land before any GPS- or tag-corrected robot runs a tank `moveTo` (HA-179 names the
  hazard) and before anyone reads a robot record's `imuYawRate`.
- (b) Fold it into TANK1. **Cost:** it changes robot behaviour on every drivetrain (records, GPS gating, the
  Localizer's rate on correction ticks), which breaks the ruling's "the holonomic drives stay byte-for-byte the
  same" as the team lead heard it, and grows a chunk already past a day.
- (c) Leave it. **Cost:** the records lie on the robot, and TANK1 degrades on corrected robots.

*Not a decision, decided here with the numbers (§0.2): the arrival rule is NOT loosened — a tank meets 0.5 in
AND 0.02 rad after steering in every probed drift case; and no `MotionOptions` field is added (§1.8).*

---

## The design in plain words (for the team lead)

On a tank, `moveTo` now does two small things while it drives, and nothing else:

1. **It aims a little back toward its line.** The "line" is the one you drew: through the target point, along
   the target heading (for `driveTo`, that is the line from where the step started to the point). If the robot
   is off that line, the heading it steers to is bent toward the line — 0.1 rad (about 6°) per inch off,
   **never more than 15°**, and fading to nothing in the last 6 inches so it cannot wobble at the finish.
   Backing up, it bends the other way, which is what backing onto a line takes.
2. **It measures the turn it did not get, and cancels it.** Every tick it compares how fast the robot is
   turning (the IMU reading the position estimate already took that tick) with how fast it asked to turn a tick
   earlier. A weaker right side shows up as a steady turn to the right nobody asked for; TANK1 adds the opposite
   (never more than 1.5 rad/s). This is what actually fixes robot two's problem: the drift comes from the heading
   being pulled, and a plain heading-hold (today's) always leaves a few degrees of pull while driving — 3.5° with
   a 9 % weak side on the host test plant. The same comparison also sees anything else that stops the robot
   turning as asked — a robot pinned by a defender, slow motors — and pushes against it too, up to that same
   1.5 rad/s; when the robot is freed, the push dies away within about a tenth of a second.

What it never does: aim at the target POINT, or turn more than 15° off the heading you wrote. A leg that starts
more than 15° off its heading (you forgot `face()`) is not steered at all — it behaves exactly as today and the
log says `turn first: face()`. Whenever it is steering, the tick's log line carries ` TSTR`. The X-drive and
H-drive are untouched (byte-for-byte, checked). Your auton line does not change. Setting one config number to 0
turns all of it off.

On the host test plant the 40 in auton now **FINISHES** with either side up to **25 %** weak (robot two's is
~9 %), in 1.80–2.57 s, within 0.11 in of the line; the acceptance run finishes with either side 9 % weak; a
30 % weak side still stops honestly. That plant's motors respond instantly. Give them a lag the controller does
not know about (a motor time constant of 0.12 s) and the 40 in still finishes to 20 % weak; at 0.18 s a 15 %
weak side times out while 9 % still finishes (2.09 s). Small sideways offsets are a bonus, not a promise: a 2 in
offset along your heading is steered out even with lag; a 6 in one is on the instant plant but NOT with lag (it
stops half an inch beside the target). Face the point. **None of it has run on a robot** — COMP-t's floor run is
the first test, and R5's measured motor lag says which of these numbers applies.

---

## 0. Why — the evidence, re-verified at the lines cited

### 0.1 The mechanism (each line opened)
- **Arrival needs both criteria:** `move_to_pose.hpp` ~:176-177 (`settledTrans_.update(std::hypot(errX, errY))`,
  `settledHead_.update(errH)`), ~:188-189; thresholds `motion_config.hpp` ~:96-101 (0.5 in, 1.0 in/s, 0.1 s;
  0.02 rad, 0.30 rad/s, 0.1 s).
- **Nothing feeds a sideways error back on a tank:** the field-frame x/y PIDs ~:197-198 become a body command in
  `command_pipeline.hpp` ~:119-131, where body |vy| is clamped to `strafeAuthority()·maxLinearSpeed`;
  `TankKinematics::strafeAuthority()` returns 0.0 (`tank.hpp` ~:77), and `forward()` reports vy ≡ 0 (~:57-64).
  The heading loop is P only: `pidH_.update(0.0, -errH)` ~:199, kP 4 (`motion_config.hpp` ~:84), kI 0.
- **The facade says so:** `chassis.hpp` ~:98, "On tank, laterally-offset targets honestly exit TimedOut";
  `strafeTo` ~:271-273. `routine.hpp` ~:229-231 tells tank authors to `face()` first. C1's D12
  (`C1-COMPLETED.md` ~:267-273): *"Building nonholonomic turn-toward-target planning into C1 would smuggle the
  LemLib behaviour into the library's core; C3's H-drive fallback is the sanctioned, telemetry-visible
  exception."*
- **COMP-t's probe 4 reproduced exactly** (`probe_laws base`, and COMP-t's own `host_probe3.cpp` rebuilt
  against the tree): the 40 in, right side ×1.00 FINISHED at (39.77, 0.00); ×0.97 TIMEOUT y −0.93; ×0.95 −1.56;
  ×0.91 −2.86; the acceptance run stops at step 1 from ×0.95 at (23.93, −0.64). Robot two's sides differ by
  ~9 % and it veered 11° right over 48 in open-loop (`R3d-PROGRESS.md` ~:2507-2514).
- **Robot two's odometry is drive encoders + IMU.** A drive-encoder rig (`probe_laws` `drvenc`, the builder's
  wiring with `independentMotionSource = false`) gives the SAME numbers as the Pilons rig on the plant, because
  the plant's tank never skids — the drift is all heading. HA-180 registers that this is also true on the
  robot (§3).

### 0.2 The root causes, measured (not assumed)
- **(a) No lateral feedback on a tank — confirmed** (above).
- **(b) The P-only heading hold under a side imbalance has a steady heading error while driving — measured**,
  as the mean of (heading − setpoint) over the ticks at full forward speed (`probe_laws linep 0`): −1.12° (×0.97),
  −1.90° (×0.95), −3.50° (×0.91), −5.95° (×0.85). The yaw a weak side makes is proportional to the forward
  command (plus a constant from the weak side's static-friction volts, which dominates the last inches), and a
  P loop only holds against it by keeping an error — that error integrated over 40 in IS the drift.
- **Does lateral feedback alone suffice? No — measured.** The bend alone (candidate (i), best settings) leaves
  the 40 in timing out at ×0.95 (xt −0.61) and ×0.91 (xt −1.12), and the acceptance run stops at ×0.91. With a
  P heading loop and a P bend, the steady cross-track error is (heading error)/(bend gain); raising the bend
  gain to remove it under-damps the line loop (the heading loop's 0.25 s lag at 60 in/s). So **(b) needs its
  own fix.**
- **Which fix — an integral of the heading error, or measuring the unexplained yaw? The yaw observer, on the
  numbers:**
  - **A heading integral winds up on the leg's own opening turn.** A leg that starts 10° off its heading (still
    on its line) integrates that turn and overshoots the line at the end: TIMEOUT xt −0.51 (clean plant).
  - **Gating the integral until the heading first reaches its setpoint fails the other way:** with a 9 % weak
    side, a leg starting even 1.15° toward the weak side's pull never reaches the setpoint (the pull holds it
    off), never integrates, and times out at xt −0.87 to −1.01 — at every negative start offset probed.
  - **The observer does not accumulate a turn the robot FOLLOWS,** because it learns only the DIFFERENCE
    between the yaw rate the heading loop asked for (after the ω clamp, before desaturation) and the yaw rate
    the robot made: every start offset from −14.9° to +14.9° settles, clean and ×0.91 (`probe_window`).
    **It is not windup-free** (re-check item 4): while the robot CANNOT follow — motor lag, desaturation at full
    speed, rotation blocked — that difference is its own command, and it integrates the heading error like an
    integral, bounded by `yawCorrectionLimit` and decaying within ~`yawObserverTime` once the robot turns again.
    Measured pinned (both sides 0 V): the ×0.91 40 in still arrives after a 0.5 s pin (2.40 s) and a 1.0 s pin
    (3.28 s; the correction reaches its 1.5 rad/s limit, heading error peaks 3.7°) where TANK1-off and bend-only
    time out; a lateral-6 leg pinned mid-bend overshoots after release (heading error 17.4°) and times out at
    y +6.53 where bend-only arrives. Bounded and honest; HA-179's blast radius; P8(e).
  - **It degrades more gently when the robot has inertia the controller does not model** (the plant's `kA`
    raised; the controller's feedforward keeps `kA` 0, as R5a will — R5a records kA without applying it). Over
    the 26 core scenarios: at motor time constant 0.12 s the observer passes 26/26, the integral 17/26, the
    bend alone 10/26, today 5/26; at 0.18 s the observer still 26/26; at 0.24 s it falls to 22/26 (the weak-side
    start-offset legs), where the heading loop's own kP 4 is the limit (R5's to tune). **The 14 stretch
    scenarios do NOT hold under lag** (re-check item 2; `recheck/inertia_detail.txt`): 14/14 at τ 0, **11/14 at
    τ 0.12 s** (lateral 6 in, lateral 6 in at ×0.91, reverse lateral 6 in: TIMEOUT at xt +0.56 / +0.58 / +0.56 —
    each ends beside the target, along 0.00, having overshot the line), **10/14 at τ 0.18 s** (also R ×0.85, xt
    +0.60), 11/14 at τ 0.24 s. Robot two's case holds: R ×0.91 arrives at τ 0.12 s (1.45 s), 0.18 s (2.09 s),
    0.24 s (3.23 s, xt 0.50 — the edge). Gains re-probed (crossGain 0.08; a look-ahead damping term along the
    robot's own heading, 2–9 in): each fixes some lag cases and costs a τ-0 case (crossGain 0.08 leaves the
    clean ±14° starts 0.46 in off the line; look-ahead ≥ 2 in fails lateral 6 in at τ 0) — no setting passes
    everything, so the τ-0 setting stays, and R5's measured kA picks (HA-179's settle clause).
  - **Near the target it adds no chatter on the lag-free plant:** the commanded yaw rate changes sign 0 times
    inside the last 6 in on every drift case, 6 times over the whole 40-scenario sweep (lateral 2 in: 1;
    start-offset legs at ×0.91: ≤ 2) — a heading integral: ~1–2 per leg. **With lag it grows** — 63 / 141 / 190
    over the sweep at τ 0.12 / 0.18 / 0.24 s; the drift cases stay at ≤ 2 (τ 0.12) and ≤ 3 (τ 0.18) per leg,
    the lateral-6 legs reach 8–11. P13 fences both.
- **The arrival rule is NOT loosened.** With the recommended design every drift case up to a 25 % weak side
  settles inside 0.5 in / 0.02 rad (final heading error ≤ 0.12°); nothing in the probes needs a looser rule.

### 0.3 The candidates, probed (every cell is the host test plant, through the real `Chassis`, 5 s default budget)
Cell: `S`/`T` = SETTLED / TIMEOUT, seconds, final signed cross-track error (xt, in, + = left of the line), peak
steer (°: for (i) the largest bend; for (ii) the largest heading excursion from the target heading; for (iii) the
largest carrot-bearing offset). **PLAN** = the law aimed the robot at a bearing computed from the target POINT.
Settings: (i) bend 0.1 rad/in, cap 15°, fade 6 in; +integral kI 8/s², limit 1.5 rad/s, band 0.35 rad; +observer
τ 0.1 s, limit 1.5 rad/s — all engaged only within 15° (D2 (a)). (ii) Siegwart/Astolfi polar law
(kρ 3, kα 8, kβ −1.5, heading P below 1 in). (iii) a boomerang written from LemLib's published description
(carrot = target − 0.6·d along the target heading, the target itself inside 7.5 in) — not LemLib's code, and
judged by THIS engine's arrival rule.

| scenario | today | (i) bend only | (i) + heading integral | **(i) + yaw observer — RECOMMENDED** | (ii) unicycle | (iii) boomerang |
|---|---|---|---|---|---|---|
| 40 in, R ×1.00 | S 1.80 · 0.00 | S 1.80 · 0.00 | S 1.80 · 0.00 | **S 1.80 · 0.00 · 0°** | S 1.80 | S 2.55 |
| 40 in, R ×0.97 | T · −0.93 | S 1.85 · −0.36 · 3° | S 1.82 · −0.08 | **S 1.83 · 0.00 · 1°** | S 1.83 · −0.02 | S 2.58 · −0.07 |
| 40 in, R ×0.95 | T · −1.56 | T · −0.61 · 4° | S 1.83 · −0.13 | **S 1.85 · +0.01 · 1°** | S 1.84 · −0.04 | S 2.60 · −0.12 |
| 40 in, R ×0.91 | T · −2.86 | T · −1.12 · 8° | S 1.88 · −0.22 | **S 1.89 · +0.02 · 2°** | S 1.98 · −0.06 | S 2.68 · −0.22 |
| 40 in, **L** ×0.91 | T · +2.86 | T · +1.12 | S 1.88 · +0.22 | **S 1.89 · −0.02 · 2°** | S 1.98 · +0.06 | S 2.68 · +0.22 |
| 40 in, R ×0.85 | T · −4.87 | T · −1.91 | S 2.13 · −0.30 | **S 2.04 · +0.06 · 4°** | S 2.98 · −0.07 | T · −0.39 |
| 40 in, R ×0.75 / ×0.70 | T / T | — | — | **S 2.57 · 0.00 / T · −0.44** (along −0.50: the weak side cannot creep under its static friction) | — | — |
| acceptance run, R ×0.95 | STOP at step 1, (23.93, −0.64) | FIN 6.07 s | FIN 5.72 s | **FIN 5.78 s, (0.30, 0.07, 178.6°)** | FIN 5.78 s | FIN 6.96 s |
| acceptance run, R ×0.91 | STOP at step 1, (23.87, −1.17) | STOP at step 1 | FIN 6.43 s | **FIN 6.14 s, (0.37, 0.08, 177.3°)** | FIN 6.16 s | FIN 7.54 s |
| lateral 2 in at 40 in (faced along 0°) | T · −2.00 | S 1.76 · +0.25 | S 1.77 · +0.33 | **S 1.75 · +0.27 · 12°** | S 1.81 | S 2.54 |
| lateral 6 in at 40 in | T · −6.00 | S 1.78 · −0.22 | S 2.04 · +0.36 | **S 1.80 · −0.16 · 15°** | S 1.83 · 11° | S 2.53 · 22° PLAN |
| lateral 12 in at 40 in | T · −12.00 | T · −5.18 | T · −3.47 | **T · −5.10 · 15° (bounded, honest)** | S 2.04 · 22° | S 2.52 · 41° PLAN |
| lateral 6 in, R ×0.91 | T · −8.78 | T · −2.27 | S 1.92 · −0.25 | **S 1.93 · −0.26 · 15°** | S 2.43 | S 2.66 · 23° PLAN |
| lateral 0.1 in at 30 in (C3's floor case) | S 1.62 · −0.10 | S 1.64 | S 1.64 | **S 1.64 · 0.00 · 1° (SFB 0, TSTR 0)** | S 1.64 | S 2.31 |
| reverse −40 in, R ×0.91 | T · −2.86 | T · −1.12 | S 1.88 · −0.22 | **S 1.89 · +0.02 · 2°** | S 1.98 | S 2.68 |
| reverse −40 in, L ×0.91 | T · +2.86 | T · +1.12 | S 1.88 · +0.22 | **S 1.89 · −0.02** | S 1.98 | S 2.68 |
| L9's `reverseTo` shape (faced away: heading 180°, target (40, 0, 180°)), R ×0.91 | T · −2.86 | T · −1.12 | S 1.88 | **S 1.89 · +0.02** | S 1.98 | S 2.68 |
| reverse with 6 in lateral | T · −6.00 | S 1.78 | S 2.04 | **S 1.80 · −0.16 · 15°** | S 1.83 | S 2.53 · PLAN |
| a point BESIDE the robot: (0, 24) heading 0° | T · −24.00, no motion | same | same | **T · −24.00: engaged, nothing to steer along, no rotation** | S 2.69 · 148° **PLAN** | T · 121° **PLAN** |
| (0, 24) heading 90° (a 90° turn asked) | T · −9.06 | not engaged | not engaged | **T · −9.06 = today, bit for bit, + the Warn** | S 2.37 · 90° **PLAN** | S 2.34 · **PLAN** |
| (24, 0) heading 90° | T · +15.37 | not engaged | not engaged | **T · +15.37 = today, + the Warn** | S 2.16 · 101° **PLAN** | T · 121° **PLAN** |
| un-faced `driveTo`, (0,0,0°)→(40, 20) | T · −5.92 | = today | = today | **T · −5.92 = today, + Warn "26.6 deg"** | S 1.94 · **PLAN** | S 2.64 |
| un-faced `driveTo`, (−24,−12,90°)→(0, 24) (the runner test's leg) | T · +7.17 | = today | = today | **T · +7.17 = today, + Warn "33.7 deg"** | S · **PLAN** | S 2.61 |
| start ON the line, ±14.9° off heading, ×1.00 and ×0.91 | T | T at ×0.91 | T from ±8° at ×0.91 and at ±14° clean (windup) | **S ≤ 1.93 s all four; 15.1° = today + Warn** | S | S |
| composed hostility, even drive, 10 seeds | 10/10, xt ≤ 0.05 | 10/10 | 10/10 | **10/10, xt ≤ 0.05; TSTR on 0.7 % of ticks** | 10/10 | 10/10 |
| composed hostility, R ×0.91, 10 seeds | **0/10**, xt ≤ 2.83 | 0/10 | 10/10, ≤ 0.18 | **10/10, xt ≤ 0.08** | 10/10 | 10/10, ≤ 0.25 |
| inertia (motor τ 0.12 s), 26 core scenarios | 5/26 | 10/26 | 17/26 | **26/26** | 25/26 (the miss: it drove to the point beside it) | 26/26 (with PLANs) |
| *re-check:* inertia, the 14 STRETCH scenarios (lateral 2/6 in, ×0.85/×0.80, ±10°/±14° starts) at τ 0 / 0.12 / 0.18 / 0.24 s | — | — | — | **14 / 11 / 10 / 11 of 14** (lateral 6 in times out at xt ≈ +0.56 from τ 0.12 s; R ×0.85 from τ 0.18 s) | — | — |
| *re-check:* on a ProsImu-shaped IMU (2nd read in a tick = 0; + a 3 % ×50 spike), 15 cases | — | — | — | **v1 (read `ctx.imu()`): 15/15 but perturbed; spikes 2/15. Amended (read `loc.twist()`): 15/15, bit-identical to the plain IMU** | — | — |
| X-drive and H-drive, every verb, clean + hostile | — | — | — | **byte-identical** (records, log lines, truth: 4 hashes equal) | n/a | n/a |

**Recommendation: (i) + the yaw observer** — **unchanged by the re-check, with its observer reading the
Localizer's rate** (item 1). It is the only candidate that settles every drift case to a 25 % weak side, every
start offset in its window, both reverse cases, the acceptance run, and hostility on the lag-free plant, and
every core case to τ 0.18 s — and that never computes a heading from the target point. Its lateral-offset
bonus does not survive lag (the re-check row above); that is stated, fenced (P13) and not promised. (ii) and
(iii) arrive more often ONLY where they plan a turn (148° to reach a point beside the robot) — exactly C1's D12
line and the ruling's "never plans a turn".

### 0.4 Where this leaves shulib against LemLib on a tank
Measured against the boomerang column (a law written from LemLib's description, in this engine — not LemLib):
after TANK1, on a faced leg with a drifting side, shulib arrives where the boomerang arrives, ~0.75 s sooner on
the host test plant (1.80–1.89 s against 2.55–2.68 s: the carrot sits behind the target and the close zone
slows the finish), and to a weaker side (shulib to ×0.75; the boomerang to ×0.91, not ×0.85) — lag-free-plant
numbers; with lag (τ 0.12 s) both pass the 26 core cases, the boomerang only by planning turns. Where the boomerang turns FOR the author (any
off-line target: 22°–121° of unplanned heading change), shulib still refuses and says why (`face()` first). The
master plan's thesis (holonomic + fused localization) is unchanged; TANK1 closes the one place a tank-only
controller did better than shulib on a tank.

### 0.5 Found while probing (each handled in scope, or named as out)
1. **`test/boot_source_test.cpp` pins `MotionConfig`'s member count (12) and size (368 B), and `ChassisConfig`'s
   size (416 B)** (~:239-265), "compare it in requireDefaultTuning()". TANK1's new member trips them (13 members;
   408 B; 456 B). Part A extends `requireDefaultTuning()` (~:268-297) with the five new fields and updates the
   pins (§2.6). Probed: the patched copy compiles and passes.
2. **An H-drive with a dead strafe wheel (`strafeTractionDerate` 0 ⇒ authority 0, `h_drive.hpp` ~:52-53) IS
   eligible** — the honest signal — and steers lateral offsets on a clean plant. With a weak SIDE it does not
   settle on the plant (y ±1.58): the plant's least-squares forward kinematics turns the weak side's
   inconsistency into a phantom sideways creep (vy −0.25 in/s at rest) that no steering at rest can remove.
   A plant artefact of a 3-wheel model with one wheel declared dead, not a tank behaviour; recorded, OUT.
3. **At a stop, the observer can hold a small command a stalled weak side will not follow** (its static-friction
   deadband). On a tank the settle completes and the motors stop, so it is harmless there; on an unsettled leg it
   is part of why ×0.70 times out along-track. Recorded in HA-179's blast radius.
4. **The C3 floor test still detects its floor** (`motion_hdrive_test.cpp` ~:398-424): a 0.1 in offset at 30 in
   is now steered out (xt 0.00) and never lights SFB; with the floor removed (`nofloor/`) the same leg lights SFB
   on 163 ticks, so the test still goes red. Re-run that mutation in the tree (§2, M26) — the probe is not the
   evidence.

---

## 1. Scope — IN

### 1.1 `include/shulib/motion/tank_steer.hpp` — NEW, a leaf (Part A)
The pure geometry, unit-testable without a plant. As probed (`proposed/shulib/motion/tank_steer.hpp`); the
header comment explains the line, the formula, the fade, and the settle bound:

```cpp
struct TankLineError {
    double along = 0.0;  ///< along the target heading, from the target (− before it going forward)
    double cross = 0.0;  ///< off the line, + to the LEFT of the target heading
};

[[nodiscard]] inline TankLineError tankLineError(const math::Pose2d& pose,
                                                 const math::Pose2d& target) noexcept {
    const double th = target.heading().radians();
    const double c = std::cos(th);
    const double s = std::sin(th);
    const double dx = pose.x().value() - target.x().value();
    const double dy = pose.y().value() - target.y().value();
    return TankLineError{.along = dx * c + dy * s, .cross = -dx * s + dy * c};
}

/// delta = −clamp(−along/fade, −1, 1) · clamp(crossGain·cross, ±maxSteer): the first factor is the
/// fade AND the direction (+1 forward, −1 reverse, continuous through 0 at arrival); no speed in any
/// denominator, so the Stanley singularity (gain ∝ 1/v) cannot occur.
[[nodiscard]] inline double tankSteerAngle(const TankLineError& e,
                                           const TankSteerConfig& cfg) noexcept {
    const double direction = std::clamp(-e.along / cfg.fadeDistance, -1.0, 1.0);
    const double bend = std::clamp(cfg.crossGain * e.cross, -cfg.maxSteer, cfg.maxSteer);
    return -direction * bend;
}

/// Inclusive: a leg exactly at the window is steered.
[[nodiscard]] inline bool tankLegEngages(double startHeadingError,
                                         const TankSteerConfig& cfg) noexcept {
    return cfg.maxSteer > 0.0 && std::abs(startHeadingError) <= cfg.maxSteer;
}

struct TankSteerStatus {  // what MoveToPose::tankSteerStatus() returns — for tests and Tier-3 tools
    bool eligible = false; bool engaged = false;
    double cross = 0.0; double bend = 0.0; double yawCorrection = 0.0;
};

inline constexpr double kTankSteerNoiseFraction = 0.02;  // the TSTR legibility floor (below)
```
- **The settle bound, stated in the header and pinned (U5):** within the translation tolerance (|along| ≤ 0.5,
  |cross| ≤ 0.5), |delta| ≤ (0.5/fadeDistance)·crossGain·0.5 = **0.0042 rad** at the defaults — a fifth of the
  0.02 rad heading tolerance, so the bend can never hold a settle open.
- **`kTankSteerNoiseFraction` = 2 % of `maxAngularSpeed` (0.12 rad/s at HA-50's 6 rad/s).** Measured: composed
  hostility on an even drive lights the flag on ~0.7 % of a 40 in leg's ticks at 2 % (~6 % at 1 %, where IMU
  rate noise reaching the observer crosses it); a 3 % weak side lights it on ~57 of ~183 ticks. A
  telemetry-legibility constant like `kStrafeFallbackNoiseFraction` (`command_pipeline.hpp` ~:72-82) —
  host-decidable, **not** a register entry (A4 rule 1). Cite the test that measures it, never "the brief".
  **The floor in rad/s moves with `maxAngularSpeed`** (re-check item 7): R5a derives that budget from the
  measured gains, so on a characterized robot the floor is 2 % of the derived budget, not 0.12 rad/s; P7's
  lighting fractions are measured at 6 rad/s (the host rig's config, which R5a does not touch).

### 1.2 `MotionConfig::tankSteer` — `motion/motion_config.hpp` (Part A)
A new aggregate `TankSteerConfig`, **appended LAST** in `MotionConfig` (after `stall`, ~:123 — designated
initializers stay valid), validated in `validate()` (~:135-163) by `tankSteer.valid()`:

| field | default | meaning | register |
|---|---|---|---|
| `crossGain` | 0.1 rad/in | heading bend per inch off the line | HA-179 |
| `maxSteer` | 0.2617993877991494 (15°) | the bend's cap AND the engage window — ONE number on purpose (§1.7); its doc says that raising it widens which un-faced legs are steered, toward the measured ~30–40° cliff; **0 turns TANK1 off entirely** (pre-TANK1 tank motion, bit for bit); must be < π/2 | HA-179 |
| `fadeDistance` | 6 in | along-track distance over which the bend fades to 0 at arrival; > 0 | HA-179 |
| `yawObserverTime` | 0.1 s | low-pass time constant of "measured yaw rate (the Localizer's, this tick) − the yaw rate asked for last tick (post ω-clamp, pre-desaturation)"; > 0 | HA-179 |
| `yawCorrectionLimit` | 1.5 rad/s | the most yaw the correction may add, either way — also the bound on how far it integrates while the robot cannot follow (blocked, lag); 0 = bend only | HA-179 |

`valid()`: every field finite; `crossGain ≥ 0`, `yawCorrectionLimit ≥ 0`, `0 ≤ maxSteer < π/2`,
`fadeDistance > 0`, `yawObserverTime > 0`. Each field's doc says PROVISIONAL (A4: HA-179). The struct's comment
says what it does in one sentence per knob, and what it never does (probed text in `proposed/`). **Why these
numbers:** the sweep (`sweep_dob.txt`) passes every core scenario at every plant inertia to τ 0.18 s across
crossGain 0.1–0.125, cap 15°, fade 4–8, τ 0.05–0.1 — a plateau, not a knife edge; cap 15° beat 20° on the large
lateral offsets (less overshoot of the line), and τ 0.1 cuts the flag's noise lighting under composed hostility
from 78 to 14 ticks over ten seeds against τ 0.05.

### 1.3 `MoveToPose` — `motion/move_to_pose.hpp` (Part A)
As probed (`proposed/shulib/motion/move_to_pose.hpp`; diff against the tree in `TANK1-PROGRESS.md` §probe):
1. **`PoseMotionOptions` gains `bool tankSteer = false;` appended** (~:71-75). **Only** the public constructor
   passes `PoseMotionOptions{.tankSteer = true}` (~:90-92). `StrafeTo` (~:41-42 of `strafe_to.hpp`) and
   `HoldPose` (`hold_pose.hpp` ~:41, ~:49) keep the default false: unchanged on a tank.
2. **The gate, computed once in the protected constructor** (after `cfg_.validate()`, ~:322-331):
   `tankEligible_ = options.tankSteer && deps.kinematics->strafeAuthority() == 0.0 && cfg_.tankSteer.maxSteer > 0.0;`
   **Exactly `== 0.0`:** "no lateral authority at all" is the property that makes a sideways error
   uncorrectable by strafing; a kinematics-type check would miss a derate-0 H-drive and would bind the motion
   layer to a concrete class. A comment says so.
3. **The engage decision, once per run, at the first live tick** — right after `errH` is computed, before the
   exit verdict (~:173-175): `tankEngaged_ = tankEligible_ && tankLegEngages(errH, cfg_.tankSteer);` and, when
   eligible but not engaged, **one Warn** through `deps_.ctx->telemetry()`, tag `MOT`, built with
   `lineformat::Line`/`appendNum` (the ONE formatter — never `snprintf %f`), ASCII only:
   `tank moveTo not steered: it starts <n> deg off its target heading (steers within <n> deg) -- turn first: face()`
   (golden-pinned; wording not frozen). `start()` resets `tankDecided_`, `tankEngaged_` and the observer state.
4. **The heading command** (replacing ~:199's one line):
   ```cpp
   double w = 0.0;
   bool tankActive = false;
   if (tankEngaged_) {
       const TankLineError line = tankLineError(pose, target_);
       const double delta = tankSteerAngle(line, cfg_.tankSteer);
       const double errHs =
           pose.heading().errorTo(target_.heading() + math::Angle::radians(delta));
       // the yaw observer: (the yaw rate the robot made) − (the yaw rate asked for last tick,
       // post ω-clamp, pre-desaturation), low-passed and cancelled; bounded; held while the IMU is
       // not ready. The rate is the one the LOCALIZER read this tick — NEVER a second
       // ctx.imu().yawRate() (ProsImu differentiates between CALLS: a second read in a tick is ~0)
       if (hasOmegaCmd_ && dt.value() > 0.0 && ctx.imu().isReady()) {
           const double unexplained = loc.twist().omega().value() - lastOmegaCmd_;
           const double a = dt.value() / (cfg_.tankSteer.yawObserverTime + dt.value());
           yawDisturbance_ = std::clamp(yawDisturbance_ + a * (unexplained - yawDisturbance_),
                                        -cfg_.tankSteer.yawCorrectionLimit,
                                        cfg_.tankSteer.yawCorrectionLimit);
       }
       w = pidH_.update(0.0, -errHs) - yawDisturbance_;
       const double contribution =
           cfg_.heading.kP * math::Angle::radians(errHs - errH).radians() - yawDisturbance_;
       tankActive = std::abs(contribution) > kTankSteerNoiseFraction * cfg_.maxAngularSpeed.value();
       lastCross_ = line.cross;
       lastBend_ = delta;
   } else {
       w = pidH_.update(0.0, -errH);  // C1's line, unchanged — every holonomic and un-engaged motion
   }
   ```
   After `applyCommandPipeline` (~:216-220), **only when engaged**: `lastOmegaCmd_ = cmd.body.omega().value();
   hasOmegaCmd_ = true;` — the observer compares against the yaw rate the loop ASKED for after the ω clamp and
   BEFORE desaturation (`cmd.body` is `bodyClamped`, `command_pipeline.hpp` ~:125-140; desaturation runs after
   it). A desaturation shortfall at full speed therefore reads as unexplained and is compensated — bounded like
   everything else the observer does (§0.2). The ω clamp's own shortfall is NOT (P8(d) pins it).
   **Why `loc.twist()`** (re-check item 1): it is the ONE per-tick reading of the IMU's rate the library already
   consumes (`localizer.hpp` ~:363-365, non-finite ⇒ 0); on the host test plant it is the same number the IMU
   holds, so nothing measured here changes; on the robot it is the tick's first `yawRate()` call, the only one
   `ProsImu`'s default rate answers truly. The scheduler and the hand loops update the Localizer before the
   motion ticks (`motion_scheduler.hpp` ~:1016, `motion_test_rig.hpp` ~:124-126), so it is always this tick's.
5. **The translation command is untouched** (the field x/y PIDs, the pipeline, the SFB flag): TANK1 is a heading
   change only. On a tank SFB keeps C3's meaning (lateral demand the drive cannot deliver) and now lights LESS
   (×0.91: 41 ticks against 498 today), because the error it reports is being steered out.
6. **The record:** `r.tankSteerActive = tankActive;` beside the SFB line (~:231).
7. **`[[nodiscard]] TankSteerStatus tankSteerStatus() const noexcept`** — eligible, engaged, last cross, bend,
   yaw correction (all zero/false off a tank). Additive on an unfrozen class.
8. **The header's thesis block** (~:3-11, "C3's H-drive strafe FALLBACK is the only sanctioned exception"), the
   header's per-tick pipeline steps 2-3 (~:13-21: "heading PID is fed (setpoint 0, measurement −eh)" — on an
   engaged tank leg it is fed the error to the BENT setpoint, and the yaw correction is subtracted) and the
   pipeline comment ~:201-215 ("on tank it marks any real lateral demand as undeliverable") are rewritten:
   TANK1 is the second sanctioned, telemetry-visible exception, and what it is not.

### 1.4 The record flag (Part A)
- **`diag/debug_record.hpp`: `bool tankSteerActive = false;` APPENDED after `tickPhase`** (~:212) — the
  file's rule (~:10-17): "Later chunks POPULATE fields; they must never RESHAPE them. Additions are
  append-only." F9 is not frozen yet (`roadmap.md` F9 row: 🎯, H1 not run). Doc comment names the producer
  (TANK1), the floor, the TermSink token and the blackbox bit.
- **`diag/term_sink.hpp`:** ` TSTR` appended after ` SFB` (~:153-155) — flag order DR, SFB, TSTR, CLMP, flt;
  the header's list (~:19-20) and `emit()`'s doc (~:110) name it.
- **`diag/blackbox_format.hpp`: tick flags bit 2 = `tankSteerActive`** (encode ~:466-467, decode ~:549-552,
  layout comment ~:423-435). The width stays 428 bytes. **`kFormatVersion` stays 1** — the coordinator's call,
  recorded in the comment: every earlier writer wrote bit 2 as 0, an older file decodes it as false (true of
  it), and no robot program writes blackbox files today (`grep SdSink src/` is empty). If the coordinator rules
  "bump", it is v2 and `blackbox_reader.hpp` ~:129 accepts {1, 2}.

### 1.5 The result line's cross-track column (Part B)
As probed (`proposed/`):
- `MotionStatsSink` (`motion_scheduler.hpp` ~:406-525) keeps the last aggregated `measuredPose`
  (`lastMeasured_`) and the target the motion published on its FIRST live record (`firstTarget_`, captured
  beside `startPose_`), both reset in `beginMotion()`, and gains `hadDirection()` —
  `|firstTarget_ − startPose_| ≥ kHoldEpsilonIn` — and `crossTrack()` = `tankLineError(lastMeasured_,
  target_).cross`, computed at read time, no per-tick trig. **Why the FIRST target** (re-check item 3): `TurnTo`
  and `DriveBrake` publish a target that FOLLOWS the robot (`turn_to.hpp` ~:225, `drive_brake.hpp` ~:197), so the
  LATEST target minus the start is the distance a weak side dragged the turn — the draft's test printed
  `TurnTo#1 ✓SETTLED … 1.49s xt   0.00"` at ×0.91. The first live target is "here" for a turn, a brake and a
  capture-here hold, and the destination for a translation. `overshoot()` is left exactly as it is (its bytes
  are pinned; its own direction test has the same weak-side blind spot — "Found while reading").
- `CompletedMotion` (~:530-554) appends `bool hasCrossTrack = false; units::Length crossTrack{};`; `finalize()`
  (~:1094-1107) sets `hasCrossTrack = hasData && stats.hadDirection() && schedDeps_.kinematics->strafeAuthority() == 0.0`.
- `diag::MotionResult` (`motion_result.hpp` ~:87-104) appends the same two fields; `emitResultLine` (~:106-145)
  appends ` xt ` + `appendNum(crossTrack, 6, 2)` + `"` **after the duration** (~:139) — every existing byte of
  the §18.3 shape unchanged; `RunReporter::onMotionComplete` (~:136-148) copies them.
- Probed lines: `MoveToPose#1 ✓SETTLED final(  39.6,   0.0,   0.1°) over  0.00" drift  0.1°   1.89s xt   0.02"`;
  the un-faced leg: `... ✗TIMEOUT final(  42.6,  14.7,  26.6°) ... 5.01s xt  -5.92"`. X-drive and H-drive lines
  byte-identical (in the hashes). Tank `turnTo`/`hold`/`brake` lines carry no `xt` — **also with a ×0.91 side**
  (re-probed, `recheck/chk_xt2`: `TurnTo#1 ✓SETTLED final(  -0.4,  -0.7,  89.1°) … 1.49s` with no column,
  while the next `MoveToPose#2 … xt  -0.32"`). A tank `strafeTo` line does carry it (it set out to translate)
  — its final distance off its line is exactly what it failed at. An explicit-pose `HoldPose` whose target is
  away from the robot carries it too (it translates to its pose).

### 1.6 What TANK1 touches, verb by verb
| verb | TANK1? | why |
|---|---|---|
| `moveTo`, `followTrajectory` legs, `Routine::moveTo/driveTo`, `PathRunner` DriveTo/MoveToPose, L9's `reverseTo` | **yes**, per leg, engaged by D2's window | each is one plain `MoveToPose`; the route ≡ recipe (G2) and D1 twin/sugar bit-identities hold because both arms run the same engine (suite: 1659/1659) |
| `strafeTo` | **no** — honest TimedOut kept | its contract is "translate while HOLDING the heading" (`strafe_to.hpp` ~:3-16; C1 pins the heading within 2° throughout); a 15° bend would break its defining promise. A tank author who wants a line-following translate writes `moveTo` with the current heading. Pinned bit-identical (P9) |
| `hold` (and Tier-3 `HoldPose`, both constructors) | **no** | a stationary target has no along-track distance to steer over (the bend is 0 there anyway); an observer fighting a push during a hold is a change nobody asked for. Pinned (P9 — on the explicit-pose constructor with an offset target, the case that discriminates, re-check item 5) |
| `turnTo`, `face`, `faceAway` (L9) | **no** | a separate class (`turn_to.hpp`); rotation in place IS the author's turn. Pinned (P9) |
| `brake` | **no** | `DriveBrake` commands 0 V; nothing to steer |
| `drive()` / teleop | **no** | the manual verb (`chassis.hpp` ~:385-423); T4's territory |

### 1.7 "Never plans a turn" — what it means after TANK1 (the docs say exactly this)
1. **Nothing in TANK1 is computed from the bearing to the target POINT.** The line uses the target's position
   AND the target's heading; a point beside the robot on its own heading produces no bend and no rotation (P6).
2. **The heading setpoint never leaves the target heading by more than `maxSteer` (15°),** and the bend is
   zero at the target (U4, U5, P4).
3. **A leg that starts more than `maxSteer` off its heading is not steered:** it runs exactly as before TANK1,
   bit for bit, and says `turn first: face()` (P5).
4. **Where the target heading is not the author's** (re-check item 10): a `PathRunner` MoveToPose leg with
   `HeadingMode::Free` targets the robot's heading AT LEG START (`path_runner.hpp` ~:229-233), so it always
   engages, its line runs through the point along the way the robot already faces, and a point off that line is
   steered toward by at most 15° — exactly what a hand-written `moveTo({x, y, chassis.pose().heading()})` does.
   Measured: a point 6 in off at 40 in arrives (1.80 s); 20 in off times out, bounded (the lateral-12 row: xt
   −5.10, heading within 15°). That IS a bounded bend toward a point the author did not face; the docs say so
   in those words, and "face the point" stays the advice.
5. **The window is the cap, on purpose:** a leg that starts within 15° of its heading is one whose starting
   error the bend itself could have commanded; beyond that, steering it would mean the heading loop first making
   the author's turn (D2 (b)'s measured cliff: un-faced legs arrive to ~30°, time out from ~40°). So raising
   `maxSteer` to remove larger offsets also widens which un-faced legs are steered — `maxSteer`'s doc and
   HA-179 say it, and the default stays 15°.
What stays C1's, unchanged: `moveTo` servoes to the heading the author wrote, simultaneously with translation.
**C1's D12, amended** (append a dated note under D12 in `C1-COMPLETED.md`, do not rewrite it): "on a
strafeAuthority-0 drive, `moveTo` now removes a lateral offset from its target's LINE by bending its heading
setpoint toward the line (≤ 15°, faded to 0 at the target) and cancelling measured unexplained yaw — a second
sanctioned, telemetry-visible exception (TSTR) beside C3's fallback. Still no turn-then-drive planning anywhere:
nothing aims at the target point, and a leg that starts > 15° off its heading is not steered."

### 1.8 The A/B switch: config, not `MotionOptions`
`ChassisConfig.motion.tankSteer.maxSteer = 0` (the robot's `TuningConfig`, `robot_config.hpp` ~:489) turns
TANK1 off for a whole robot; a Tier-3 caller builds a `MoveToPose` with its own config per leg. **Rejected: a
`MotionOptions` field** — F6 grows (a minor bump of its own), and five of the eight verbs that take the struct
would silently ignore it (L9 D1 (d)'s argument, the same here). `moveTo`'s signature does not change.

### 1.9 Reverse, and L9 Part A
The direction factor's sign is what makes backing onto the line steer the right way: `reverse −40 in` and
L9's `reverseTo` shape (facing 180°, target heading 180°) settle with xt ±0.02 at ×0.91 (the unit test U3 and
plant test P2 each go red if the sign is dropped). **Interaction with L9 Part A** (routine.hpp's `faceAway` /
`reverseTo`, both one `Chassis` call):
- A1 (faceAway + reverseTo ≡ hand-written turnTo + moveTo) and A3 (route ≡ recipe): both arms run the same
  engine — unaffected.
- A2 (±10° heading band while reversing, clean tank): a faced reverse leg on a clean plant has ~0 cross, bend ~0
  — inside the band. **L9 sets A2's bounds "from a measurement": if TANK1 lands first, L9 measures with it.**
- A4 (`reverseTo` from a robot FACING the point: 180° to turn): not engaged (|errH| = π > 15°) — the leg is
  bit-identical to today **plus one `MOT` Warn**; L9's guide-09 paragraph (A4's measured outcome) should quote
  that Warn. A6's tests that read Warn lines must match the `RTN` tag, not "any Warn".
- **Who re-reads whom:** whichever lands second. If L9 Part A lands first, TANK1's executor also rewrites
  `reverseTo`'s doc comment in the TANK1 words, and adds P2's case through `Routine::reverseTo`. If TANK1 lands
  first, L9's executor re-reads `routine.hpp` ~:67-72 and ~:229-231 (TANK1 edits both; L9 edits ~:67-72 too —
  merged by hand) and the guide-09 Tank drives section. The version number: D1.

---

## 2. Tests (adversarial) and mutations

Grade on **ground truth** (`h.truePose()`), never on the estimate the motion read. New cases:
`test/tank_steer_test.cpp` (the pure functions, U*) and `test/motion_tank_steer_test.cpp` (the plant, P*);
edits in the files named in §2.5–2.6. Suite counts before and after in the PROGRESS log (baseline measured by
the probe: 1659 cases, 3 skipped).

### 2.1 Unit (`tank_steer_test.cpp`)
- **U1 line geometry:** `tankLineError` at target headings 0°, 90°, −90°, 179.9°, −179.9° (the seam), with the
  robot left/right/before/past — exact `along`/`cross` from hand-derived numbers.
- **U2 forward sign:** target ahead (along < 0), robot LEFT (cross > 0) ⇒ bend < 0 (turn right), and mirrored.
- **U3 reverse sign:** target behind (along > 0), robot LEFT ⇒ bend > 0.
- **U4 the cap:** over a grid |cross| ≤ 1000 in, |along| ≤ 1000 in: |bend| ≤ maxSteer, equality reached, never
  exceeded; the direction factor never exceeds 1 in magnitude.
- **U5 the fade and the settle bound:** bend == 0 at along == 0 exactly; linear in along inside the fade;
  |bend(+ε) − bend(−ε)| → 0 (continuity); over |along|, |cross| ≤ 0.5 in (a 101×101 grid), |bend| ≤ 0.0042 rad
  with the defaults, and < `headingSettle.maxError`.
- **U6 the engage predicate:** |e| == maxSteer ⇒ true; `std::nextafter(maxSteer, +inf)` ⇒ false; maxSteer 0 ⇒
  false for every e (including 0).
- **U7 `TankSteerConfig::valid()` / `MotionConfig::validate()`:** each field NaN, ±inf, negative; maxSteer = π/2;
  fadeDistance 0; yawObserverTime 0 — each refused (a `PreconditionError` from `validate()`); the defaults valid.

### 2.2 The plant (`motion_tank_steer_test.cpp`; TankKinematics 12 in; the COMP-t weak-side `DegradationModel`)
- **P1 the drift cases:** 40 in, right side ×0.97, ×0.95, ×0.91, ×0.85, ×0.75 and LEFT ×0.91: SETTLED within
  (measured + margin; probe: 1.83/1.85/1.89/2.04/2.57/1.89 s), truth |cross| ≤ 0.12 in, |heading error| ≤ 0.2°.
  **The same legs with `maxSteer = 0`: TimedOut with y −0.93/−1.56/−2.86**, and the ×0.95 one's hash over its
  records and truth (NOT its log lines — Part B adds ` xt` to every tank result line) equals the hash captured on
  the pristine tree — the off switch restores today's tank bit for bit (probed: `OFFSWITCH` hash equal). The
  Routine path: `Routine::moveTo` at ×0.95/×0.91 `r.ok()`.
- **P2 reverse:** −40 in at R and L ×0.91 SETTLED, xt ≤ 0.1; the `reverseTo` shape (start facing 180°, target
  (40, 0, 180°)) SETTLED; truth x decreases every tick of the forward case / increases for the reverse (it
  really backs up).
- **P3 the acceptance run** (`competition::acceptanceRun` through `Routine`), **the RIGHT and the LEFT side**
  (the DoD says "either side" — re-check item 9) at ×0.95 and ×0.91: FINISH; the true END within 0.5 in of
  (0, 0); the true heading within 0.02 rad of **the `driveTo` leg's own target heading** — read from
  `chassis.scheduler().lastCompleted().targetPose` with a record-wanting sink attached (the approach bearing
  computed when the step ran: 177.07° at R ×0.91, −177.05° at L ×0.91), NOT a fixed 180° (a weak side's turns
  translate the robot, so the bearing is not 180°; the draft's 3°-of-180° bound had 0.26° of margin). Probe
  (`recheck/probe_accept.cpp`): R ×0.95 5.78 s END (0.30, 0.07), R ×0.91 6.14 s (0.37, 0.08), L ×0.95 5.78 s
  (0.30, 0.05), L ×0.91 6.15 s (0.36, 0.05); heading vs the leg ≤ 0.23° at every scale ×0.75–×1.00, both sides.
  **×0.65 (either side) stops honestly at step 1** (TimedOut, one MOTION_TIMEOUT; END (23.37, ∓0.56)).
- **P4 bounded, and no chatter (the lag-free plant):** lateral 12 in at 40 in ⇒ TimedOut (xt ≈ −5.1), and on
  EVERY tick the true heading stays within `maxSteer + 3°` of the target heading; inside the last 6 in the
  commanded yaw rate changes sign ≤ 3 times on each drift case (probe: 0; fade removed: 25) and ≤ 3 on lateral
  2 in (probe: 1). Lateral 2 in and 6 in ⇒ SETTLED ≤ 2.2 s (probe: 1.75, 1.80; fade removed: 3.21 s). Its
  comment says these are lag-free numbers and points at P13.
- **P5 the engage window (D2):** starting ON the line at ±14.9° off heading, clean and ×0.91 ⇒ SETTLED (four
  cases); at 15.1° ⇒ the leg is **bit-identical** to the same leg with `maxSteer = 0` (memcmp per-tick truth +
  exact volts, the G2 `requireIdentical` shape) AND exactly ONE `MOT` Warn equal to the golden text, at the
  first live tick, and none on an engaged leg.
- **P6 never aims at the point:** target (0, 24, 0°) from the origin facing 0°: engaged, TimedOut, the true
  heading never leaves 0 by more than 1°, no motion beyond 0.5 in; target (0, 24, 90°): not engaged, Warn,
  bit-identical to `maxSteer = 0`.
- **P7 the flag:** ×0.95 leg ⇒ `tankSteerActive` on ≥ 40 records (probe: 82 of ~185); clean even drive ⇒ on 0
  records; composed hostility on an even drive, 10 seeds ⇒ on ≤ 2 % of the motion's records (probe 0.7 %); on
  X-drive and H-drive ⇒ never. TermSink renders ` TSTR` for a lit record (P7 reads the rendered line too).
  **The hostility clause is the ONLY one that kills M18** (the floor removed: the clean drive still lights 0
  records and the ×0.95 leg 183 ≥ 40; hostility 14 → 2 109 ticks over 10 seeds — re-check item 5): keep it, and
  say so in its comment.
- **P8 the observer:** (a) a ×0.5 side ⇒ `tankSteerStatus().yawCorrection` never exceeds 1.5 rad/s in magnitude
  and reaches it; (b) after a ×0.91 leg, `start()` ⇒ yawCorrection 0 and engaged false before the first live
  tick (probe: −0.067 → 0.000), and a second run's first record has the flag off on an even drive; (c) an IMU
  dropout (`imuReady` false for 0.30–0.60 s mid-leg) ⇒ the correction does not change during the dropout
  (probe: 0.000000); **(d) the ω clamp's shortfall is not a disturbance:** `maxAngularSpeed` lowered to 0.6 rad/s
  (so the clamp binds during the 15° bend), clean lateral 6 in ⇒ |yawCorrection| ≤ 0.1 rad/s on every tick
  (probe `recheck/probe_wclamp.cpp`: peak 0.028; with M15a 0.477 — at 6 rad/s the clamp never binds and M15a is
  equivalent, so this case is its only killer); **(e) a pinned robot (a fence for HA-179's blast radius):** the
  ×0.91 40 in with both sides' volts forced to 0 from 0.5 s to 1.0 s into the leg ⇒ |yawCorrection| ≤ 1.5 rad/s
  on every tick, true heading error ≤ 6° on every tick, and SETTLED (probe `recheck/probe_pinned.cpp`: 2.40 s,
  peak 0.75 rad/s, 2.7°); the same leg with `maxSteer = 0` still times out (y −2.86).
- **P9 the siblings are unchanged on a tank:** `StrafeTo` (an along-heading target and a lateral one, at ×1.00
  and ×0.91), **`HoldPose` built with the EXPLICIT-pose constructor on a target 3 in ahead and 1 in aside, held
  1 s, at ×1.00 and ×0.91**, `TurnTo`, `DriveBrake` — each bit-identical (per-tick truth + exact volts) with
  TANK1 on vs `maxSteer = 0`; the lateral StrafeTo still exits TimedOut with SFB lit (C3's existing case keeps
  passing unedited). **Why the explicit hold** (re-check item 5): a hold of "here" at rest computes the same
  doubles even when engaged (no error, no rate) — M17 survives it, also after a coast on this memoryless
  plant; the offset hold and the ×0.91 StrafeTo are DIFFERENT under M17 and IDENTICAL under the design
  (`recheck/probe_m17.cpp`).
- **P10 holonomic bit-identity:** (a) **an executor pre/post check, NOT a CI test** (re-check item 6): golden
  hashes captured on the PRISTINE tree before any TANK1 edit — FNV-1a over every record's `blackbox::encodeTick`
  bytes, every log line and the final truth, for an X-drive and an H-drive script of every verb (moveTo,
  strafeTo, turnTo, followTrajectory, a speed-capped moveTo, hold, brake) with a `RunReporter` attached, clean
  and under composed hostility (seed 7) — `probe_proposed.cpp`'s `holonomicScript` is the model — plus the tank
  ×0.95 40 in with `maxSteer = 0` (records + truth, for P1's off switch); run it again on the finished change;
  log both sets of five hashes in `TANK1-PROGRESS.md` with the base commit and the machine. It is not a CI test
  because it is a closed loop through libm (`sin`/`cos`/`hypot`/`atan2` and the hostility RNG's transforms): a
  runner-image libm change would turn CI red with no code change, and no closed-loop golden exists in `test/`
  (the `matrix_kinematics_test.cpp` ~:212-260 precedent hashes `forward()` arithmetic only). (b) **the CI test:**
  the same scripts with `tankSteer` defaults vs `maxSteer = 0` ⇒ identical within one build (libm-independent),
  beside the existing C2/C4 bit-identity suites.
- **P11 the honest tank signal:** an H-drive with `strafeTractionDerate = 0` (authority exactly 0) ⇒ eligible and
  engaged; a lateral 2 in target at 40 in on a clean plant SETTLES; the default H-bot (0.35) ⇒ not eligible.
- **P12 robot two's odometry shape:** the ×0.95 and ×0.91 40 in, both sides, on a DriveEncoderOdometry rig
  (`independentMotionSource` false, the builder's wiring) SETTLE (probe: 1.85 s, 1.89 s, xt ±0.02) — the drift is
  seen by drive encoders + IMU.
- **P13 the plant with inertia (a regression fence, not a claim about the robot):** plant `wheelFf.kA` 0.02 (τ
  0.12 s), controller kA 0 (probe: 26/26 core, 11/14 stretch — `recheck/inertia_detail.txt`):
  (a) the ×0.91 40 in, BOTH sides ⇒ SETTLES (probe 1.45 s, truth |cross| 0.22), and the commanded yaw rate
  changes sign ≤ 4 times inside the last 6 in (probe 2); (b) lateral 2 in ⇒ SETTLES (probe 1.33 s);
  (c) **lateral 6 in ⇒ its measured verdict, TimedOut**, ending beside the target (|along| ≤ 0.2, 0.5 < |xt| ≤
  1.0; probe along 0.00, xt +0.56), the true heading within `maxSteer + 3°` on every tick, and the yaw sign
  changes inside the last 6 in ≤ 12 (probe 8). The comment says: this pins today's limit so a regression is
  seen AND so an improvement is noticed — if (c) ever SETTLES, update guide 14, the cookbook and HA-179, then
  flip it. Plant kA 0.03 (τ 0.18 s): (d) the ×0.91 40 in ⇒ SETTLES (probe 2.09 s, 3 sign changes).
- **P14 the observer never re-reads the IMU** (re-check item 1; `recheck/probe_imu2.cpp` is the model): a small
  rig in the test file whose `RobotContext` AND `Localizer` share an IImu wrapper over the harness's `FakeImu` —
  the first `yawRate()` of each clock instant answers truly, every later one 0 ("zero" mode) or 0 with a 3 % ×50
  spike ("spike" mode), modelling `ProsImu`'s `DifferentiateRotation` on a microsecond clock; the wrapper counts
  its second reads. With a NullSink (no records, so no record read): the ×0.91 40 in (both sides) and the
  acceptance run at ×0.91 ⇒ per-tick truth + exact volts **bit-identical** to the same run on the plain
  `FakeImu`, in both modes, and the wrapper counts **0** second reads over every engaged tick. Probe: identical,
  0 reads; with M28 (the draft's `ctx.imu().yawRate()`) zero mode differs on every drift leg and spike mode
  times out the 40 in from ×0.97 and stops the acceptance run.

### 2.3 Record / diagnostics (Part A)
- `debug_record_test.cpp`: a default record has `tankSteerActive == false`; the field is the struct's last.
- `blackbox_format_test.cpp`: a per-field case `{"flags: tank steer", 123, 4ULL, 1, ...}` beside ~:561-563;
  round trip; a v1 file with bit 2 clear decodes false; bits 3–7 written 0.
- `term_sink_test.cpp`: a golden with DR SFB TSTR CLMP flt all set, in that order; TSTR alone.

### 2.4 The result line (Part B)
- `motion_result_test.cpp`: `hasCrossTrack` false ⇒ the EXISTING goldens byte-identical (unedited); true ⇒
  ` xt  -0.04"` appended after the duration; NaN ⇒ the deterministic token.
- Scheduler / reporter: a tank moveTo ⇒ `hasCrossTrack` and the value from the records (equal to
  `tankLineError(finalPose, target).cross` when the last record is the exit record); a tank turnTo ⇒ false **at
  ×1.00 AND at ×0.91** (the weak side drags the turn ~0.8 in; the draft's latest-target test printed `xt` there —
  re-check item 3; kills M29); a tank DriveBrake and a capture-here HoldPose ⇒ false; a tank StrafeTo ⇒ true;
  every X/H motion ⇒ false (the CI killer of M25 now that P10 (a) is not a CI test); a NullSink run ⇒ false (no
  data, never a made-up 0).

### 2.5 Mutations (a FRESH binary each — `motion_config.hpp` and `debug_record.hpp` reach 22+ test files and
`chassis.hpp`/`motion_test_rig.hpp` 39: use HANDOFF's stale-binary recipe, ~:211-217, and require `Linking CXX
executable shulib_tests`; log each RED or, if GREEN, why)
| # | mutation | must go red |
|---|---|---|
| 1 | bend sign flipped (`+direction * bend`) | U2, P1 |
| 2 | direction from `std::abs(along)` (no sign) | U3, P2 |
| 3 | bend clamp removed | U4, P4 (heading band) |
| 4 | fade removed (`direction = along < 0 ? 1 : -1`) | U5, P4 (lat 2 time; yaw flips) |
| 5 | direction factor unclamped (`-along / fade`) | U4 |
| 6 | eligibility `strafeAuthority() < 0.5` | P10 (b) — the H-bot engages (and the executor's P10 (a) check) |
| 7 | eligibility by `dynamic_cast<const TankKinematics*>` | P11 |
| 8 | engage window `<` instead of `<=` | U6 |
| 9 | latch removed (engage every tank leg) | P5 (bit-identity + Warn count), P6 |
| 10 | Warn emitted every tick, or never | P5 (exactly one) |
| 11 | observer sign flipped (`+ yawDisturbance_`) | P1 |
| 12 | observer limit clamp removed | P8 (a) |
| 13 | observer not reset in `start()` | P8 (b) |
| 14 | observer ignores `isReady()` | P8 (c) |
| 15a | observer compares against LAST tick's PRE-clamp ω (`lastOmegaCmd_ = w`) | P8 (d) ONLY — equivalent at 6 rad/s on every probed leg (the clamp never binds); probe: peak 0.477 vs 0.028 |
| 15b | observer compares against THIS tick's pre-pipeline ω | P1 (×0.91 TIMEOUT, xt −0.55 — probed) |
| 16 | `lastOmegaCmd_` written on holonomic motions too (harmless?) | P10 must stay GREEN — log it as an equivalent-behaviour mutant, and why |
| 17 | `PoseMotionOptions::tankSteer` default `true` | P9 — the explicit-pose HoldPose and the ×0.91 StrafeTo (a capture-here hold at rest survives it — probed) |
| 18 | flag floor removed (`> 0.0`) | P7's HOSTILITY clause only (14 → 2 109 ticks; the clean drive stays at 0 — probed) |
| 19 | flag never set | P7 |
| 20 | record field not written (`r.tankSteerActive` line dropped) | P7 |
| 21 | TermSink token missing / out of order | 2.3 goldens |
| 22 | blackbox bit 2 not encoded / not decoded | 2.3 per-field case, round trip |
| 23 | the bend aims at the POINT (`atan2(errY, errX) − θt` in place of the line) | P6 |
| 24 | `maxSteer = 0` does not disable the observer | P1's off-switch bit-identity |
| 25 | (Part B) `hasCrossTrack` without the authority check | 2.4 (every X/H motion ⇒ false); also the executor's P10 (a) check (X/H lines gain `xt`) |
| 26 | (existing) C3's SFB floor removed | `motion_hdrive_test.cpp` "sub-perceptible lateral demand" must STILL go red with TANK1 in (§0.5 item 4) |
| 27 | (Part B) cross-track sign flipped in the stats sink | 2.4 |
| 28 | the observer reads `ctx.imu().yawRate()` (the draft) instead of `loc.twist().omega()` | P14 (both modes; probed — the draft build IS this mutant) |
| 29 | (Part B) `hadDirection()` judged on the LATEST target (the draft) | 2.4's ×0.91 tank turn (probed: `TurnTo#1 … xt 0.00"`) |

### 2.6 The one existing test file that must change: `test/boot_source_test.cpp`
`static_assert(memberCount<MotionConfig>() == 13)`, a new `memberCount<TankSteerConfig>() == 5` pin,
`sizeof(MotionConfig) == 408`, `sizeof(ChassisConfig) == 456` (measured on the host, x86-64/LP64), and five
`REQUIRE`s in `requireDefaultTuning()` after `rotationRadius` (~:296). Probed (`boot_source_test_tank1.cpp`).
**No other existing test's CODE is edited** — the whole suite passed against the overlay unchanged (re-run on
the amended overlay: 1659/1659, 3 skipped, 2 465 800 assertions, `recheck/suite_run.txt`); if one fails for the
executor, stop and log it: that is a behaviour change this brief did not predict. **One test COMMENT is
corrected** (re-check item 8): `test/robot_builder_test.cpp` ~:469-471 ("A TANK has no lateral authority, so a
laterally-offset target exits TimedOut on purpose") — after TANK1 a small offset along the heading may arrive;
the fixture's reason (an along-heading leg) is unchanged, so only the sentence moves.

---

## 3. Documentation (same change — pace rule 3; prose review logged — rule 4)
Read `docs/internal/guide-maintenance.md` first. Public documents cite evidence by date, never a
`docs/internal` path (C7). "Host test plant", never "simulator". Say plainly: **nothing here has run on a robot.**

**Part A — public (~9):**
1. **`include/shulib/chassis/chassis.hpp`** — the TURN-WHILE-DRIVE item (~:91-98): replace "On tank,
   laterally-offset targets honestly exit TimedOut." with a TANK STEERING item (the line, the bend ≤ 15°, the
   observer ≤ 1.5 rad/s, never at the point, the 15° window and its Warn, TSTR, what still times out: a point
   beside the robot, an offset beyond what 15° over the leg removes — **no size of offset promised: it depends on
   motor lag (host test plant, 2026-09-27)** — `strafeTo` on a tank; a PathRunner `HeadingMode::Free` leg steers
   toward an off-line point by ≤ 15° (§1.7 item 4); `maxSteer = 0` restores the old tank's motion); `moveTo`'s
   doc (~:262-263) and `strafeTo`'s (~:271-273, unchanged behaviour, say why). This is the F6 sentence D1 is
   about.
2. **`include/shulib/chassis/routine.hpp`** — `driveTo`'s comment (~:229-231: `face()` first is still how a
   leg starts on its line — and now required for it to be steered); the field-vocabulary paragraph (~:67-72,
   "keeps C1's D12 intact"). Comments only; F10's pins untouched.
3. `strafe_to.hpp` (~:13-16, ~:29-33), `move_to_pose.hpp` (§1.3 item 8 — the thesis block, **the per-tick
   pipeline steps 2-3 ~:13-21**, and the pipeline comment), `command_pipeline.hpp` (~:37-47's SFB note is still
   true — read; the TANK1 cross-reference only if it helps), `motion_config.hpp` (the struct, as the overlay
   words it — the observer's reference, its bounded integrating while the robot cannot follow, the cap/window
   coupling; and **`validate()`'s doc ~:125-131**, "all FIVE speed / timeout / geometry scalars" — add that it
   also checks `tankSteer.valid()`, and that unlike SettleConfig/OdoStallCheckConfig it DOES descend into it).
4. **`docs/guide/04-drivetrains.md`** ~:91-94 (tank bullet).
5. **`docs/guide/09-the-recipe-api.md`** ~:111-117 (argument sugar: `face` is still your turn — and now the
   thing that makes the leg steerable) and "Tank drives" ~:194-222.
6. **`docs/guide/10-the-api.md`** ~:91-94 (moveTo's fine print), ~:116-117 (strafeTo on tank: unchanged), gotcha
   ~:332. `guide-10b`'s listing (`guide_examples_test.cpp` ~:602-628) is still true — keep it.
7. **`docs/guide/11-reading-the-diagnostics.md`** — the flags list (~:155-167: ` TSTR`, what it means, that it
   lights only on a tank, with the floor); **the `SFB` line ~:159-160** ("the H-drive is running a
   sideways-limited leg") — SFB also lights on a tank whenever a leg asks for sideways motion it cannot deliver,
   and on tank logs it appears beside TSTR (41 SFB ticks on the ×0.91 40 in); the new Warn line and what to do
   about it.
8. **`docs/guide/12-when-things-go-wrong.md`** ~:74-75 ("Unreachable by drivetrain") — the differential now
   reads: a tank leg that timed out with the not-steered Warn ⇒ face first; with TSTR lit to the end ⇒ an
   imbalance or offset beyond the bound; `strafeTo` on a tank ⇒ unchanged.
9. **`docs/guide/14-what-it-cannot-do-yet.md`** — Motion-quality boundaries (~:321-364): tank steering exists
   and is host-proven only; its bounds (15°, 1.5 rad/s, the window); a sideways skid drive encoders cannot see is
   invisible to it (HA-180); **the host test plant's lag margin, in numbers** (re-check item 2): with a motor
   time constant of 0.12 s the 40 in finishes to a 20 % weak side, at 0.18 s a 15 % side times out; a 2 in
   offset along the heading is removed with lag, a 6 in one only without it; a pinned robot's correction winds
   to its limit and can overshoot on release. No sentence here may promise a 6 in offset.
10. **`docs/cookbook/04-drivetrains.md`** "A tank routine" (~:13-50): why `face` then `driveTo` still (the
    window); **the "You are still writing the turn" bullet ~:33-35** ("shulib's motion verbs never insert a turn
    on your behalf … a library that silently turns your robot") — still true of TURNS, but now say it exactly:
    no verb plans a turn for you; on a tank, `moveTo` may bend its heading up to 15° toward the line you drew
    while it drives, visibly (TSTR), and never more; "Watch out for" `moveTo to a pose that is sideways of you`
    rewritten — **not** "a small offset is now steered out" as a promise: "on the host test plant a couple of
    inches off your heading are steered out; motor lag shrinks that, and a point beside you is not — face the
    point." The listing is unchanged.
11. **`docs/changelog.md`** — under the heading D1 sets: **"Changed — `moveTo` on a tank steers back onto its
    line (TANK1)"**: BEHAVIOUR CHANGE (tank only; X/H byte-identical) **listing the three visible changes**
    (re-check item 11): legs that already arrived take slightly different paths; an un-faced tank leg logs one
    `MOT` Warn; (Part B) translating tank result lines gain ` xt`; ADDED (`TankSteerConfig`,
    `MotionConfig::tankSteer`, `tank_steer.hpp`, `DebugRecord::tankSteerActive`, `MoveToPose::tankSteerStatus()`,
    the ` TSTR` token, the Warn), Migration (`maxSteer = 0` — restores motion, records and truth exactly, and
    the log except ` xt`), "host test plant only".
12. **`docs/hardware-assumptions.md`** — HA-179, HA-180 (below): index rows in numeric order (175–178 may land
    first), a "TANK1 (2026-09-27) added TWO" paragraph in the style of COMP-t's, the header's status count.
13. `docs/api` regenerated (a new `tank_steer.md` page; `motion_config`, `move_to_pose`, `debug_record`,
    `term_sink`, `blackbox_format` pages; `all-entities.md`, `README.md`) — never hand-edited; `check-fresh`.

**Part B — public:** guide 11's "Per-motion result lines" (~:60-80: the ` xt` column, tank only);
`motion_result.hpp`'s header shape comment; `docs/api` regenerated; the changelog entry extended (ADDED).

**Also public (Part A), not internal:** `docs/roadmap.md` (it lives in `docs/`, not `docs/internal/` — re-check
item 11) F6 row: one parenthetical "(TANK1, <date>: the tank sentence of the strafe-fallback semantics
rewritten, API <D1>)" — no `docs/internal` path in it (C7).

**Test comment (Part A):** `test/robot_builder_test.cpp` ~:469-471 (§2.6).

**Internal (both parts):** `TANK1-PROGRESS.md` (first) and `TANK1-COMPLETED.md`; `build-order.md` TANK1 status
line (~:1177-1182 — the coordinator wrote the entry, the executor writes its status); HANDOFF L2u (~:516-520)
one appended sentence; `C1-COMPLETED.md` D12's dated amendment note (§1.7) — the coordinator reconciles "you
are here"; the briefing regenerated LAST. (The plan commit `9e86a3c` edited `build-order.md`, HANDOFF,
`PROJECT-BRIEFING.md` and `docs/roadmap.md` after the draft's first read — re-read each at the tip.)

**Prose review greps (log each and its hits):** `laterally-offset`, `honestly exit`, `sideways target`,
`cannot strafe`, `turn-then-drive`, `face(`, `TimedOut`, `SFB`, `D12`, `plans turns`, `plan a turn`,
`never plans`, `tank`, `strafeAuthority`, `simulator`, `MotionConfig`, `368`, `416`, **`insert a turn`,
`silently turn`, `the H-drive is running`, `setpoint 0`, `yawRate()`, `6 in`, `sideways of you`** (the last
three: the observer's source, and no promise of a 6 in offset).

**HA-179 — TANK1's steering constants hold on robot two.**
- *Claim:* with crossGain 0.1 rad/in, a 15° cap, a 6 in fade, a 0.1 s yaw observer and a 1.5 rad/s limit,
  robot two's tank `moveTo` removes the drift of a side up to ~10 % weaker than the other before arrival, within
  the unchanged 0.5 in / 0.02 rad rule, without heading oscillation; specifically, the drivetrain follows a yaw
  command much faster than 0.1 s (its motor time constant kA/kV well under ~0.15 s).
- *Source:* `motion/motion_config.hpp` `TankSteerConfig` (PROVISIONAL (A4: HA-179)); `motion/move_to_pose.hpp`.
  Host test plant: settles to a 25 % weak side at τ 0 and every core case to τ 0.18 s; degrades at τ 0.24 s.
  **The stretch cases do not hold under lag:** a 6 in lateral offset is removed at τ 0 but ends ~0.56 in beside
  the target (TimedOut) from τ 0.12 s; a 15 % weak side times out from τ 0.18 s; near-target yaw sign changes
  grow with lag (6 → 63 → 141 → 190 over the probe sweep at τ 0 / 0.12 / 0.18 / 0.24 s; ≤ 3 per drift leg to
  τ 0.18 s).
- *Confidence:* **invented** — tuned on the host test plant's placeholder dynamics (HA-45/HA-50 co-dependency).
- *Settle:* COMP-t's Station M.6 (the 40 in on the tape: RESULT, END y, the TSTR count, the tape's sideways
  offset) — and R5, whose characterization records kA (R5a) and so τ = kA/kV. **When τ is known, re-run the
  probe grid** (`probe_laws` SWEEP with the measured kA — the brief's `recheck/sweep_k*.txt`/`sweep_look*.txt`
  are the starting point): at τ ≥ 0.12 s crossGain 0.08 was measured more lag-tolerant (stretch 14/14 at 0.12 s,
  13/14 at 0.18 s) at the cost of the clean ±14° starts ending 0.46 in off the line at τ 0; that choice is R5's.
  The same grid re-derives the TSTR floor's lighting if R5a moves `maxAngularSpeed`.
- *Blast radius if wrong:* too weak ⇒ the drift is not removed ⇒ today's honest MOTION_TIMEOUT (bounded,
  visible). Too strong / too slow a drivetrain ⇒ heading wobble on engaged tank legs, bounded by the 15° cap and
  the 1.5 rad/s limit, visible as TSTR; at a stop, a weak side's static friction can leave a small commanded yaw
  (§0.5 item 3). **While the robot cannot follow a yaw command** (pinned by a defender, wedged on a wall, motor
  lag, desaturation) the correction integrates the heading error up to 1.5 rad/s and unwinds within ~0.1 s once
  the robot turns again: measured, a 0.5–1.0 s pin on the ×0.91 40 in still arrives (2.40 / 3.28 s), a pin during
  a 15° bend overshoots on release (heading error 17.4°) and times out. **The cap is also the window:** raising
  `maxSteer` to remove larger offsets also steers un-faced legs further off (the measured cliff: arrive to ~30°,
  time out from ~40°) — keep it well under 30°. **The IMU read:** the observer uses the Localizer's rate
  (`loc.twist()`), which is the tick's first `ProsImu::yawRate()` on robot two; on a robot with a GPS or tag
  corrector, until D4's IMU1 lands, the corrector reads first on the ticks it reaches its yaw-rate gate and the
  Localizer's rate is ~0 (or a spike) there — modelled on every other tick: zeros 15/15 arrive, zeros + a 3 %
  ×50 spike 8/15. **Contained:** `maxSteer = 0` restores the pre-TANK1 tank's motion exactly; every number is
  config.

**HA-180 — robot two's sideways error on a straight `moveTo` comes from its heading, not a sideways skid.**
- *Claim:* the drift of a tank leg is the heading being pulled (which the IMU sees and the drive encoders
  integrate into the pose), not the chassis sliding sideways (which `TankKinematics::forward()` reports as zero,
  `tank.hpp` ~:52-64, and no sensor on robot two sees).
- *Source:* TANK1 steers on the ESTIMATE's cross-track error; robot two's odometry is its drive encoders alone
  (HA-131; the fall robots' sensor set, master plan ~:92) and it has no GPS on the bench.
- *Confidence:* **reasoned** — the 2026-09-25 veer (11° over 48 in open-loop) was a heading change, read on the
  IMU; a skid component was never measured.
- *Settle:* M.6 — END y by the estimate against the tape's sideways offset from the line, three runs. Agreement
  within 0.5 in ⇒ settled.
- *Blast radius if wrong:* FINISHED on the screen while the robot is off its line on the tape by the skid; every
  later leg starts from that error. Not contained by config — only GPS, a tracking wheel or a wall reset sees it.

---

## 4. OUT, and explicitly rejected
- **OUT:** a sideways skid the drive encoders cannot see (HA-180); the derate-0 H-drive under a weak side on the
  plant (§0.5 item 2); `strafeTo`/`hold` on a tank (§1.6); per-direction or per-side gains; re-tuning heading kP
  (R5); a tank arrival rule or a looser tolerance (not needed — §0.2); anything in `src/`; COMP-t's
  re-predictions (the coordinator's — §6); any hardware claim.
- **Rejected:** (ii) the unicycle pose stabilizer and (iii) the boomerang (they plan turns — §0.3); a heading
  integral (winds up on the leg's own opening turn; its acquisition gate fails under the very pull it is for —
  §0.2); steering every tank leg (D2 (b)); a `MotionOptions` switch (§1.8); zeroing the tank's lateral demand
  before the pipeline to keep SFB dark (it changes the norm-cap arithmetic and C3's flag meaning for no gain —
  SFB already lights less with TANK1); a `kFormatVersion` bump for bit 2 (§1.4 — the coordinator may overrule);
  a speed term in the bend (Stanley's 1/v — the singularity §1.1 avoids).

## 5. Coordination — shared files, and who re-reads whom
- **T4 Part 1a (in flight, `t4-1a`):** teleop only; `Chassis::drive()` untouched. Shared: `test/boot_source_test.cpp`
  (T4 edits hand-built tables; TANK1 ~:239-297), the changelog, the HA register (175/176 vs 179/180), and the
  status documents. Whoever lands second re-reads and merges; self-contained blocks only.
- **L9 Part A:** §1.9 (routine.hpp ~:67-72 both; guide 09; the version number, D1; A2/A4/A6 notes).
- **R5a** (brief file `next-wave/L7-gains-and-noise-on-the-sd-card.md`): does not change `MotionConfig`'s SHAPE
  or the heading loop, **but it does write MotionConfig VALUES** (re-check item 7): its Part B makes the
  characterization own `tuning.motion.wheelFf` and derives the three speed budgets — `maxAngularSpeed`
  included — from the measured gains (~:160-178, ~:744-756), in `effectiveTuning()` (`robot.hpp` ~:1033-1041).
  Consequences for TANK1: `tankSteer` passes through unchanged (geometric, not a volts budget), but **the TSTR
  floor in rad/s is 2 % of the DERIVED `maxAngularSpeed`**, and the cruise speed TANK1's numbers were measured at
  moves with the derived `maxLinearSpeed` — P7's lighting fractions and every probe here are at 6 rad/s / 60 in/s
  (the host rig's config, which R5a does not touch). Shared files: `boot_source_test.cpp` (R5a adds gains rows;
  `requireDefaultTuning()` is on both lists — merge the comparisons) and **guides 11, 12 and 14** (R5a's doc list
  ~:1168-1206: the gains line, its refusals, guide 14 ~:56). Whoever lands second re-reads those three guides.
  R5's measured kA settles HA-179's lag claim and picks the gain (HA-179's settle clause).
- **DIAG1 Part B** (`next-wave/DIAG1-partB-median-nonfinite.md` ~:233): edits the records' `wheelCurrent` reads
  in **`move_to_pose.hpp` ~:236, `chassis.hpp` ~:581, `motion_scheduler.hpp` ~:1189**, `turn_to.hpp` ~:235 and
  `drive_brake.hpp` ~:207, and guide 11. TANK1 edits `move_to_pose.hpp` (many lines), `chassis.hpp` (comments
  ~:91-98, ~:262-273), `motion_scheduler.hpp` (Part B: ~:406-554, ~:1094-1107) and guide 11 — same files,
  different lines; the second to land merges by hand and re-runs both suites' cases.
- **H1 (not started):** its brief says the tick flags byte carries "bit 0 `clampedThisTick`, bit 1
  `strafeFallbackActive`, bits 2–7 written 0 and ignored" (`next-wave/H1-shul2-wire.md` **~:357**), and its
  forward-compatibility test decodes "Tick flag bits 2–7 set … with bits 0–1 unchanged" (**~:654**). (~:293 is
  the ENVELOPE flags rule, not the tick's — the draft cited it wrongly.) **The coordinator amends H1 before its
  go:** bit 2 = `tankSteerActive` — H1's decoder should expose it rather than ignore it; an older reader that
  ignores it loses only the flag; H1's goldens and its 1.0 width table (428) are unaffected. Note also D4: the
  records' `imuYawRate` H1 would carry is ~0 on the robot until IMU1.
- **SAFE-SAVE:** copies `boot_source_test.cpp`'s fixture (~:111-148), not the pins — no conflict there; **but it
  edits guides 11, 12 and 14** (`next-wave/SAFE-SAVE-calibrate-save-and-teleop-label.md` ~:974-984: boot lines,
  SAVE UNCONFIRMED / BACKUP rows, HA-166) — the second to land re-reads them.
- **COMP-b:** runs autons through the same `runAuton`; its log now carries TSTR/Warn lines; no prediction of its
  depends on a tank timeout (grepped).
- **COMP-t:** §6.

## 6. The hand-off to COMP-t — every place the coordinator re-predicts after TANK1 lands
Numbers from COMP-t's own probe (`host_probe3.cpp`, rebuilt against TANK1's overlay: `comp_t_probe3_tank1`,
`comp_t_probe3_weaker`), through `runTesterOnce` exactly as COMP-t runs it. TANK1's P1/P3 pin the same legs
through the Routine; re-derive from THOSE tests' final numbers before COMP-t's builder starts. **Re-run on the
AMENDED overlay** (`recheck/ct3`, `recheck/ct3w`, with the COMP-t probe's `src1sim/` include root and its
`team/autons.cpp`): byte-identical to the table below. **The left side** (P3, re-check item 9): the acceptance
run FINISHES at L ×0.95 (5.78 s, END (0.30, 0.05, −178.47°)) and L ×0.91 (6.15 s, (0.36, 0.05, −177.14°)), and
stops at step 1 at L ×0.65 ((23.37, +0.56)) — mirror images, so COMP-t's re-predictions hold for either side.
These are lag-free-plant numbers (HA-179): COMP-t's floor run is where the robot's lag shows.

| right side | 40 in forward (was) | 40 in forward, after TANK1 | acceptance run, after TANK1 |
|---|---|---|---|
| ×1.00 | FINISHED (39.77, 0.00, 0.00°) T+1.80 | **unchanged** | FINISHED (0.23, 0.05, −179.97°) T+5.44 (was (0.24, 0.12, 180.0°), 5.42) |
| ×0.97 | STOPPED, y −0.93 | **FINISHED (39.73, 0.00, 0.04°) T+1.83** | FINISHED (0.28, 0.07, 179.18°) T+5.61 |
| ×0.95 | STOPPED, y −1.56 | **FINISHED (39.70, 0.01, 0.07°) T+1.85, faults this run: none** | FINISHED (0.30, 0.07, 178.57°) T+5.78 |
| ×0.91 | STOPPED, y −2.86 | **FINISHED (39.64, 0.02, 0.11°) T+1.89** | FINISHED (0.37, 0.08, 177.26°) T+6.14 |
| ×0.75 | — | FINISHED (39.53, 0.00, 0.09°) T+2.56 | FINISHED (0.47, 0.08, 171.51°) T+8.46 |
| ×0.72 / ×0.70 | — | STOPPED, MOTION_TIMEOUT, END y −0.25 / −0.44 (along-limited) | STOPPED at step 1 |
| **×0.65** | — | **STOPPED at moveTo, MOTION_TIMEOUT, END (39.38, −1.01, 0.60°)** — the new honest-stop case | STOPPED at step 1, (23.37, −0.56) |

Places (in `docs/internal/chunks/COMP-t-auton-tester.md` as committed at `9e86a3c`):
- header ~:5-13 (already says D10 is superseded — date the re-prediction there);
- re-check finding 3, ~:103-119, and ~:160 ("the ×0.95 run reports … MOTION_TIMEOUT" → the ×0.65 run);
- **D3** ~:257-271 (~:267-268 "Whatever the speed, the likely first floor RESULT is ROUTINE STOPPED" → now
  FINISHED on the plant; the robot is HA-179/180's question);
- **D10** ~:347-366 (superseded; keep the record, point at TANK1);
- **§0.2** ~:412-425 ("the likely RESULT word is ROUTINE STOPPED" → FINISHED expected if the drift is
  heading-driven; ROUTINE STOPPED still possible and the Warn/TSTR/xt say which);
- **§0.4 probe 4** ~:482-493 (keep the table as the pre-TANK1 evidence; add the after-TANK1 table above) and
  probe 5's "×0.95: ROUTINE STOPPED" ~:502;
- **§3.1 the students' file**: the tank comment ~:861-864 ("the drift cannot be driven out: the step ends on its
  time limit…") becomes true to TANK1 — e.g. "A tank cannot drive sideways. If one side is weaker the robot is
  pulled off its line; the library steers it back while it drives (on the brain's log: TSTR). If it still cannot
  get there, the step ends on its time limit and the result reads ROUTINE STOPPED" — and the note ~:906-910;
  **the `moveTo` line itself does not change**;
- **§4** ~:1654-1655 (the OUT row "a tank arrival rule … (D10)" → built as TANK1) and ~:1672-1673 (the rejected
  settle-tolerance row stays rejected — TANK1 did not need it);
- **§5 tests:** (3b) ~:1693-1697 → at ×0.95 it now expects **FINISHED, faults this run none**; add **(3b′)** at
  **×0.65**: ROUTINE STOPPED at step 1, `faultsAfter − faultsBefore == 1`, `lastFault == MotionTimeout`, END
  |y| > 0.5 (probe: −1.01), end action succeeded — the tester still reports a real failure; (3c) unchanged (a
  clean ×1.00 run); (4) unchanged within tolerance; **(4b)** ~:1701 → at ×0.95 it FINISHES; the honest stop moves
  to ×0.65 (step 1, END (23.37, −0.56)); the golden list ~:1759-1760 → the ×0.65 lines;
- **Station M:** M.5 ~:1935-1937 (the wheels-up reasoning still holds; drop "no sideways error can build (D10)");
  **M.6** ~:1938-1948 (expected: **FINISHED**, END |y| within 0.5 in, faults none; record the TSTR count and, if
  STOPPED, whether the Warn appeared; the tape's sideways offset against END y settles HA-180); M.8 ~:1953-1954;
- **guide 16** (~:1967-1972): "what a tank does with a sideways drift" → what TANK1 does, and when it still stops;
- §6.7's FOUND row ~:2022-2025 (→ "fixed by TANK1") and **landmine 17** ~:2106-2107 (→ "expected FINISHED; if
  red, read the Warn/TSTR/xt before touching anything").

## 7. Checks
- Host suite green, counts before/after (probe baseline 1659 cases, 3 skipped; with TANK1 + the §2.6 pins
  1659/1659 before the new tests); every mutation RED on a fresh binary (log the `Linking` line).
- P10 (a)'s golden hashes captured on the pristine tree FIRST and re-run on the finished change, both logged
  with the commit and the machine (an executor check, not a CI test — §2.2).
- `python3 tools/src_build_gate.py self-test && check` (all builds; `src/` untouched: `git diff --stat -- src/` empty).
- The ARM header gate (RESUMING ~:108-112) — it now includes `motion/tank_steer.hpp` (probed: PASS); every
  changed header compiled alone for the V5 (probed: PASS).
- The CI greps; `api_doc_tool` self-test / check-coverage / check-fresh / check-examples / check-removability;
  `doc_staleness_audit.py`; the briefing regenerated LAST and checked.

## 8. Landmines
1. **`PoseMotionOptions::tankSteer` defaults to FALSE.** `StrafeTo`/`HoldPose` build it with designated
   initializers; a `true` default silently steers them (M17).
2. **Decide the engage once, at the first LIVE tick** — never at `start()` (the boot window has no heading:
   the wait-for-live contract, `motion.hpp`), never per tick (that re-opens D2 (b) mid-leg).
3. **The observer reads the rate the LOCALIZER read this tick — `loc.twist().omega()` — and NEVER calls
   `ctx.imu().yawRate()`** (M28, P14): `ProsImu`'s default rate differentiates between CALLS, so a second read
   in a tick is ~0 or a spike on the robot, and no host test with `FakeImu` can see it. It compares against LAST
   tick's body ω after the ω clamp and before desaturation (`cmd.body`), holds while `!imu.isReady()`, and is
   reset in `start()`.
4. **Nothing in TANK1 may compute a bearing to the target point** (M23). The line needs the target heading.
5. **`strafeAuthority() == 0.0` exactly.** Not `< ε`; not a type check (M6, M7).
6. **Append-only:** `DebugRecord::tankSteerActive` after `tickPhase`; `TankSteerConfig` after `stall`;
   `CompletedMotion`/`MotionResult` fields at their ends; the result-line `xt` after the duration.
7. **`boot_source_test.cpp`'s pins are a shared file** (T4, R5a, SAFE-SAVE, L9 B) — self-contained edits only.
8. **The stale-binary trap** (HANDOFF ~:211-217): `motion_config.hpp` and `debug_record.hpp` fan out widely.
9. **Capture P10's golden BEFORE editing anything** — a golden captured after the change proves nothing.
10. **The Warn uses `lineformat`** (the ONE formatter), ASCII, tag `MOT`, once per run.
11. **Say "host test plant", never "simulator"**, and no hardware claim anywhere.
12. **HA-179 and HA-180 only**; no `docs/internal` path in a public document (C7).
13. **The flag floor is a legibility constant**, not a register entry; cite the test that measures it.
14. **`hadDirection()` uses the FIRST live record's target** (M29): the latest target of a turn or a brake
    follows the robot.
15. **No public document promises a 6 in offset**, and every performance number says "host test plant" and,
    where lag changes it, says so (re-check item 2).

## Found while reading (not TANK1's to fix)
- `version.hpp`'s policy (~:14-27) defines additive as "unchanged behaviour", which no library that fixes a
  control law can meet for the drivetrains it fixes — D1 is the third time this has needed a ruling (L9 D3,
  H1 D7). A sentence in the policy for "a frozen LIMIT relaxed" would end the case-by-case question; the team
  lead's to reword, if ever.
- The COMP-t brief's probe copies live under `.claude/coordinator/scratch/brief-COMP-t/`; its `host_probe3.cpp`
  rebuilt unchanged against this overlay — the coordinator can re-run it after TANK1 merges, from the tree.
- **(re-check) The IMU's yaw rate is read several times per tick → D4.** Every `imuYawRate` a robot record
  carries is ~0 today; a GPS/tag corrector's gate read makes the Localizer's own rate ~0 on those ticks.
- **(re-check) `MotionStatsSink::overshoot()` has the same weak-side blind spot Part B fixed for `xt`:** its
  direction test uses the LATEST target, so a `TurnTo` a weak side drags ≥ 0.1 in is scored as a translation's
  projection past the target (it printed `over 0.00"` in the probe, so harmless there). Its bytes are pinned by
  the §18.3 goldens; changing it is a result-line behaviour change for a later chunk, if ever.
