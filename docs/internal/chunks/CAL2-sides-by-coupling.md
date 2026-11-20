# CAL2 — Calibrate finds the drive's sides by COUPLING (and a steady DONE)

> Brief by the coordinator, 2026-11-17 ~08:55. **The team lead's go, 2026-11-17: "YES BE QUICK"** (after: "we need to
> just rethink it because it should beyond the push fully calibrate itself"). One run, **CAL2-A**. Library code
> (`include/shulib/calibration/`) — lands before the Wed Nov 18 freeze. Host- and build-gated; the coordinator
> uploads and runs it on robot one with the team.

## 1. Why (robot one, 2026-11-16/17 — logs `.claude/coordinator/robot_logs/robot_one_2026-11-1{6,7}.log`)

Robot one: a TANK, 10 drive motors (ports 11–20, 5 per side, each side's motors GEARED TOGETHER and facing each other,
so their push signs alternate + − + − +), BLUE, heavy (a cascade), at least one high-traction wheel. Four Calibrate runs
STOPPED in phase 3 (the PULSES): one motor of ten cannot break the robot loose. Pulsing port 11/12/14 alone gave a robot
yaw of −0.1 … −0.75 deg whatever the voltage (port 12: −0.34 / −0.33 / −0.35 deg at 6 / 9 / 12 V; its own shaft 8–14 deg
at every level) — chassis flex, not a turn. More volts or longer pulses do not help (`PULSE_STILL`, `PULSE_ROLLED`).

**The proof the new method works (diagnostic build, 2026-11-17 08:50, every other drive port's sign-corrected travel
logged per pulse):** pulsing port 11 —

| level | 12 | 13 | 14 | 15 | 16 | 17 | 18 | 19 | 20 |
|---|---|---|---|---|---|---|---|---|---|
| 3 V | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| 6 V | +6 | +6 | +6 | +5 | 0 | 0 | 0 | 0 | 0 |
| 9 V | +5 | +6 | +4 | +5 | 0 | 0 | 0 | 0 | 0 |
| 12 V | +5 | +4 | +5 | +2 | 0 | 0 | 0 | 0 | 0 |

A pulsed motor's SIDE-MATES turn with it (the shared gear train), the OTHER side never does: the sides are visible
with the robot standing still. (3 V: the backlash swallows ~13 deg of the pulsed shaft — nothing follows.)

**The DONE flicker (same runs):** DONE could not be pressed: `pushReady()` needs `atRest()` THIS tick, and robot one's
IMU reads an instantaneous yaw rate of ~0.9–1.0 deg/s at rest (logged: −0.94 deg/s with every motor at 0.000 rad/s)
against its rest floor of 1.0 deg/s, so READY flipped every update and the drawn button lagged one update — every
release landed on a not-ready tick. The heading itself was steady (−0.15 deg over a whole push).

## 2. The design (CAL2-A)

- **D1 — sides by coupling (phase 3).** After the push, pulse a drive port (one at a time, the existing ramp; add a
  **12 V** fourth level to the default ramp). At each level record every OTHER drive port's sign-corrected travel. A
  port **follows** the pulsed one when its sign-corrected travel ≥ a new `CalibratorConfig::followThreshold`
  (default **2.0 deg**, PROVISIONAL HA-136 family) in the pulsed port's direction. Stop ramping that port at the first
  level with ≥ 1 follower. The pulsed port and its followers are ONE group. Pulse next a port in no group yet; repeat
  until every drive port is in a group, or no unpulsed ungrouped port remains. A port that follows from BOTH groups, a
  third group, or a follower against its sign → refuse with a named reason (a new `CalibrateRefusal`/`StopReason` or the
  existing family — say which, plainly, with the port). **Expected on a tank: exactly two groups.** Few pulses (robot
  one: two — port 11 → {11–15}, then port 16 → {16–20}).
- **D2 — which group is LEFT (phase 4, the spin).** Spin group A forward and group B backward at the spin level; the
  IMU's yaw sign over the first `identifyYaw` (new config, default **5 deg**) decides: counter-clockwise (yaw +) ⇒ A is
  the RIGHT side. Then the spin continues as today (both directions, the effective track width, the agreement check) —
  with the assignment known, never refused as "the wrong way" for the identification turn itself. Log a line naming it.
- **D3 — fallback (uncoupled drives).** A port that reaches the top level with NO follower: use today's yaw verdict for
  it (LEFT / RIGHT by the yaw's sign when |yaw| ≥ pulseYawFloor; DECOUPLED / STILL / ROLLED as today). A run where EVERY
  port is decided by yaw (a drive whose motors are not geared together) behaves exactly as today. Mixed runs: a
  yaw-decided port joins the group whose side the spin later names — state the rule; if inconsistent, refuse by name.
- **D4 — steady rest.** `atRest()` judges the IMU by the HEADING's change over the last `restHold` window
  (|Δheading| / restHold ≤ restYawRateFloor — the same number, now an AVERAGE rate), not the instantaneous rate; the
  motors' check unchanged. A rest needs a full window of samples. The precondition
  `restYawRateFloor × pulseTime < pulseYawFloor` keeps its meaning (an average drift over a pulse). DONE then stays drawn
  while the robot is still. Keep the panel's rule (DONE drawn only when `pushReady()`, accepted only when ready at the
  release) — it is now steady.
- **D5 — the serial record.** Per pulse, a second line with the followers:
  `pulse port 11 6.0 V followers 12 13 14 15 (+6 +6 +6 +5 deg)` (or `no followers`). One line naming the groups after
  phase 3, one naming LEFT/RIGHT after the identification. Fix the PULSE_STILL sentence: it says "no other drive port
  moved" while `others` can be non-zero — say what was measured.
- **D6 — robot one's facts.** Its two typed Calibrate thresholds (`kCalibratePulseYawFloorDeg` 0.6,
  `kCalibrateRestYawRateDegPerS` 1.0 — ledger, chassis table, robot two's twin copies) stay valid under D4; keep them
  unless a test proves otherwise (log the decision). Robot two (the twin) gets the same behaviour.
- **D7 — nothing else changes:** the push, the front, the scale, the record format and its file, the save path, the
  panel's screens' layout (new words only where D5 needs them, within the existing line budgets), the robot programs.

## 3. Tests (ADVERSARIAL — each must be seen RED against a mutant)

The host test plant must model a **side's gear train with BACKLASH**: N motors per side coupled through a shared train
with a dead band (≈10 deg at the motor), plus a heavy chassis that one motor alone cannot turn (static friction), so the
OLD algorithm fails on it exactly as on robot one (PULSE_STILL) and the NEW one succeeds. Cases at least:
1. Robot one's geometry (5 + 5, alternating signs, heavy): two groups, correct sides, track width within today's bound.
2. Robot two's old robot / the 34 recorded golden runs (`test/calibrate_golden_test.cpp`): every run still passes;
   where the new phase changes lines, re-record ONLY with each diff explained in the log (expected: the pulse lines).
3. An uncoupled drive (no shared train): yaw fallback ≡ today's result.
4. Backlash wider than the 3 V pulse: no follower at 3 V, followers at 6 V (the ramp stops there).
5. A port that follows both groups / three groups / a reversed follower → the named refusal.
6. The spin identification both ways (A left, A right), and a spin that does not reach `identifyYaw` → today's
   spin refusal.
7. D4: an IMU whose instantaneous rate alternates ±1.2 deg/s around zero with a steady heading → `pushReady()` stays
   true through every tick once still; a real drift of 2 deg/s → not at rest. The panel test that DONE is pressable
   (the release lands on a ready tick) under that noise.
8. `factsProblem()` / precondition unchanged in meaning; robot one's and robot two's facts still accepted.
Mutations (each RED, restored): the follower threshold's comparison; the group union; the LEFT/RIGHT identification
sign; the fallback switch; D4's window (instant rate restored); the 12 V level; the refusal on a both-groups follower.

## 4. Docs (short, plain — the team, 2026-11-15: "soooooo many comments")

Guide 19 (Calibrate's phases: "the pulses" becomes "finding the sides" — what the robot does and the new STOPPED words);
the changelog (top of `## API 2.9`, additive: new config fields with defaults; behaviour change of phase 3 — say it);
the HA register (the new thresholds are invented — the HA-136 family); `docs/api` regenerated. Log every delta in
`docs/internal/chunks/CAL2-PROGRESS.md` (create it FIRST, `## CAL2-A`).

## 5. Landmines

- `test/calibrate_golden_test.cpp` compares 34 runs line for line — a moved line is a finding until explained.
- `calibrate_panel_test.cpp` P1 walks every reachable state of seven runs; new phases/words must keep every line inside
  its box.
- The calibrator object lives on the opcontrol task's STACK (`std::optional<DrivetrainCalibrator> cal` in
  calibrate_program.hpp): a diagnostic that added 10 KB to `CalibrationReport` froze the robot at boot (2026-11-17).
  Keep any new per-pulse storage SMALL (no per-record char arrays); state its size delta in the log.
- `src/robots/` and the facts ledgers: touch only if D6 needs it.
- Never run pros or touch /dev/ttyACM* (the coordinator uploads).

## 6. Added by the coordinator, 2026-11-17 09:10 (after a real run on robot one — read this)

- **A real run of the coupling idea COMPLETED on robot one** (a measurement build in `.claude/worktrees/calfix2`,
  never to be merged — its diff is the coordinator's quick version of D1: the yaw's SIGN names a coupled group even
  under the floor, and the spin checked it): pulse 11 at 6 V → 12–15 followed (+16…+23 deg), yaw −1.95 deg → LEFT;
  pulse 16 at 6 V → 17–20 followed (+10…+11 deg), yaw +0.84 deg → RIGHT; the spins: CCW 13.2268 in, CW 13.2341 in
  (0.05 %). MEASURED: LEFT +11 −12 +13 −14 +15, RIGHT −16 +17 −18 +19 −20, ipr L 1.320179 R 1.332460, W 13.2305 in.
  (An earlier run's spins disagreed 8 % — the USB cable caught on the robot; not a calibrator fault.)
- **LANDMINE — STACK OVERFLOW at the END of a run:** right after `front check: the card holds no record yet` (no SD
  card in the brain) the brain printed `FATAL ERROR!! Task User Operator Control (PROS)` — the opcontrol task died. The
  measurement build had added ~140 bytes to the calibrator (on the opcontrol STACK). The release Calibrate may sit just
  under the same limit on the save path. **CAL2 must move the calibrator OFF the opcontrol stack** (static storage —
  e.g. the `std::optional<DrivetrainCalibrator>` and the program run state at namespace/function-static scope, one
  instance) and state the stack use of the deepest path (the save with no card, the save with a card, the HELD screen)
  in the log. Test what can be tested on the host (the object is not constructed on the task's stack).
