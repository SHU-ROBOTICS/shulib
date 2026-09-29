# TANK1 — COMPLETED (Part A landed 2026-09-28, Part B 2026-09-29; host- and build-gated, NOT run on a robot)

> A tank's `moveTo` steers back onto its line. The team lead's ruling of 2026-09-27, choosing "Fix the library
> first" over building the auton tester to expect a timeout; his go ~08:17 with D1–D4 as recommended. Written
> FROM [`TANK1-PROGRESS.md`](TANK1-PROGRESS.md) (Part A: the build, cut off once by a session end and resumed;
> its review round, whose blocker added the lead; Part B: its build, a stopped run resumed, its review round) and
> the coordinator's two merge verifications. Brief: [`TANK1-tank-moveto-steers.md`](TANK1-tank-moveto-steers.md)
> (its header note says where the lead supersedes its formula, P-numbers and §6 table). Executor model: `opus`, in
> the isolated worktrees `.claude/worktrees/tank1` and `tank1b`.

## The one-sentence honest status

On a tank (strafe authority exactly 0), a `moveTo` leg that starts within 15° of its target heading bends its
heading setpoint back toward the target's line — on the cross-track error predicted 0.06 s ahead, capped at 15°,
faded over the last 6 in — and cancels the yaw the loop did not command (a weak side's, capped at 1.5 rad/s); a
leg that starts farther off runs exactly as before plus one Warn; the X-drive and H-drive are byte-for-byte
unchanged; and every translating tank result line now ends with ` xt`, the final cross-track error — all proven on
the host test plant, none of it yet on robot two (COMP-t's Station M is the first).

## Built

- **Part A — `3965791` (2026-09-28):** `include/shulib/motion/tank_steer.hpp` (the line error, the lead, the
  bend, the engage window); `MoveToPose`'s tank path (decided once at the first live tick; the observer reads the
  Localizer's one IMU read of the tick, never a second `imuYawRate()`); `MotionConfig::tankSteer` (six
  PROVISIONAL fields, HA-179; `maxSteer = 0` turns it off); the record flag `TSTR`; `TankSteerStatus` (Tier 3);
  API 2.3 with a BEHAVIOUR CHANGE entry; HA-179 and HA-180; 40 new test cases.
- **Part B — 2026-09-29:** the ` xt` column on every translating tank result line (moveTo,
  strafeTo, a Tier-3 `HoldPose` of a spot away; never a turn, a hold of here, a brake, or an X/H line — except the
  derate-0 H-drive, which Part A already steers like a tank), judged on the FIRST live record's target;
  `CompletedMotion::hasCrossTrack` / `crossTrack`; guides 11, 12 and 15, the changelog's 2.3 entry made true;
  12 new test cases. No API bump.

## What the reviews changed

- **Part A:** one blocker — legs starting 10–15° off their heading and off their line, which settled before
  TANK1, timed out beside the target; the bend reacted to the cross-track error NOW and over-shot the line.
  Fixed at the root with the lead (the review's 24 563-leg grid: 178 regressions → 0 up to ~0.18 s of motor lag;
  24 remain at ~0.24 s, disclosed). Two majors (the observer's cap tested one way; the bend's share of `TSTR`
  untested) and seven minors, all fixed. The team lead was told of the design change the same morning;
  everything he ruled is unchanged.
- **Part B:** twelve minors (the sign rule in guide 12 stated as measured; `xt` on cancelled and fault-abort
  lines pinned; the formatter's true worst case, 197 bytes; the derate-0 H-drive exception stated), all fixed,
  plus five further sentences the fixer's own prose review found.

## The coordinator's merge verifications

- **Part A** (2026-09-28, 11:31–11:42): see `TANK1-PROGRESS.md`, "Part A — the coordinator's merge and
  verification" — suite 1725/1725, six builds, the X/H and `maxSteer = 0` hashes identical to the pristine
  tree (17/17 and 12/12), the review's blocker legs settling, seven coordinator mutations all red.
- **Part B** (2026-09-29, 10:50–11:01): the tank1b worktree (the build; a stopped run resumed; the review round,
  whose fixer found and removed a gate self-test plant a killed run had left in `src/build_selection.hpp`) as the
  transport commit `920c704`, squash-merged onto `ee2e399` (one conflict, HANDOFF L2u's status, resolved to the
  merged state). Suite **1737 | 1737 passed**, 2,650,734 assertions; `src_build_gate.py` self-test OK (189) and
  check PASS on all six builds; the ARM header gate, both CI greps, `check-fresh`, removability, the staleness
  audit and the briefing all clean. **Identity, re-run by the coordinator** (the review's `rv_hash` against the
  pristine `a62067b` golden): the six X/H lines identical; the eleven tank lines differ ONLY in their log-text hash
  (the ` xt` column) — record counts, pace hashes and end poses identical. **Four coordinator mutations, each on a
  fresh binary, all RED:** TB1 every drivetrain gets `xt` (the strafe-authority test dropped); TB2 turns and holds
  of here get `xt` (`hadDirection()` dropped); TB3 the cross-track sign flipped; TB4 the formatter prints the
  column without path data.

## What is NOT done

- Nothing of TANK1 has run on a robot. HA-179 (every constant, provisional) and HA-180 (robot two's drift is a
  heading pull, not a sideways skid the encoders cannot see) settle at COMP-t's Station M: a FINISHED whose tape
  offset from the line differs from END y by more than 0.5 in is HA-180 failing, not a pass.
- IMU1 (the IMU's yaw rate read once per tick) is its own planned chunk; TANK1 is immune (it reads the
  Localizer's rate), but other readers are not.
