# R3d Part 2F — "shulib Calibrate" made to work on robot two: one push, a pulse that ramps, a screen a person can follow

> **Execution brief.** Written 2026-09-24 at tree `9fe155f`, from the first bench session's evidence
> (`R3d-PROGRESS.md` "Part 3"; raw captures `docs/internal/evidence/r3d-session-2026-09-24-*.log`)
> and two read-only investigations of that evidence against the code (physics; screen). A chunk that
> finds a flaw in an earlier chunk fixes it there (RESUMING): R3d Part 1 built the calibrator, Part 2
> built the program, and Part 3 — the bench session — found both unable to finish on robot two.
> **Two parts, each its own executor run, verification and commit, each on the team lead's go:**
> **F1** the library (the calibrator, the host test plant, the panel's pure model), **F2** the program
> (`src/calibrate_program.cpp` as a renderer) and the runbook. **Executor model: `opus`. Do not
> commit.** Live log: append "Part 2F-1" / "Part 2F-2" sections to [`R3d-PROGRESS.md`](R3d-PROGRESS.md), FIRST.
>
> **The team lead's decisions (2026-09-24, asked with the alternatives and their costs):**
> 1. **Sides from STRONGER MOTOR PULSES** — not the unpowered hand twist the physics investigation
>    recommended, and not a wheels-up partition. The pulse RAMPS (3 → 6 → 9 V) and stops at the first
>    level that turns the robot enough. Recorded pushback, so the choice is informed: a single member
>    of a five-motor coupled side may pivot the robot in one run and not the next (the evidence shows
>    both), and a member whose gear slips can never be labelled by a pulse — so this design must
>    DIAGNOSE those cases in plain words rather than stop blind (§2.3).
> 2. **ONE push, 48 in, along a tape** — it gives the drive ports, their signs AND the scale. No
>    separate tape push; no distance question (only 48 in exists, so a wrong distance cannot be picked).
> 3. **A RUN AGAIN button** on the finished / stopped screen.
> And his words on the screen: *"THE UI ON THE ROBOT IS TERRIBLE WHILE RUNNING CALIBRATE I CANT READ IT
> AND THE BUTTONS DONT WORK I DONT KNOW AFTER I PUSH IT WHAT IM SUPPOSED TO DO"* — then *"make it nice
> to read on the screen with the buttons working then ill run it"*.

---

## 0. What the evidence says (read before designing anything)

**Five Calibrate starts** are in `r3d-session-2026-09-24-calibrate.log` (the progress log first recorded
three — corrected):

| Start | What happened | What it proves |
|---|---|---|
| A | push → FRONT 1st → pulses → `STOPPED: port 11: the robot turned only 0.14 deg (floor 2.00 deg)` | the 3 V / 0.4 s single-member pulse can fail on the first port |
| B | push (the robot yawed +10.5 deg during it) → FRONT 1st → pulses → **`STOPPED: port 18 ... 0.09 deg`** | ports 11–17 each CLEARED the 2 deg floor (the only way to reach port 18) — the pulse CAN work; it failed on port 18, the member that travels 22–33 % short (HA-130) |
| C, D | DONE touched while the screen read `0 of 9 motor(s) have moved` → `the push moved 0 port(s)` | the screen let DONE be touched before the push (a UI defect, not an operator error) |
| E | as A, 0.17 deg | as A |

**Why the host tests passed and the robot did not** (physics investigation, every line cited there):
the calibrator's test rig runs the plant with **kS = 0 on purpose** (`test/drivetrain_calibrator_test.cpp`
~:33-38, :95-101), because the plant gives a coupled wheel the MEAN of its members' volts (HA-137):
one member of five delivers a fifth of the pulse. At the kS = 1.2 V most other suites use, the default
3 V pulse gives the wheel 0.6 V and the plant turns **exactly 0°** — tonight's refusal; the only full
run at kS 1.2 (test 19) had to raise the pulse to 9 V. The plant also pins any unpowered wheel at zero
speed (an implicit perfect brake) and models no scrub, mass, rolling, or coast-vs-brake difference, so it
can only ever PIVOT about the idle side. On the robot the idle side COASTS, so a one-sided push can ROLL
the robot forward instead of turning it. **A fix tested against that plant proves nothing.**

**Defects found on the way that every design needs fixed:**
- **No wrong-way spin guard** (`tickSpinTest`, ~:670-688): sides swapped CONSISTENTLY cause no fight,
  so the robot would spin the wrong way under power for up to 20 s. A live safety defect.
- **No spin-start check:** a spin that cannot turn the robot stalls for the whole 20 s timeout.
- **`atRest()` checks motor velocities only** (~:986-993) though the R3d brief defines rest as "yaw
  rate and every velocity under a floor" — hand motion leaks into a pulse's yaw (start B's +10.5° push
  yaw is exactly the kind of motion that can).
- **The push has no yaw bound:** a twist during the SIGN push can invert one side's captured signs.
- **The program printed no per-pulse numbers**, so start B cannot be fully interpreted.

**The screen** (screen investigation, every line cited there): the touches WORKED — all eight releases
hit a button and reached `cal.answer()`; none fell between buttons. What failed:
the instruction is cut at 44 characters (`src/calibrate_program.cpp` ~:430) so "PUSH THE WHOLE ROBOT
along its length, then touch DONE." shows without "touch DONE"; the only live number (the moved
count) is small dim grey (~:432); DONE is live while 0 motors have moved and pressing it ends the run;
after STOPPED the last buttons stay painted but nothing reads them; the header runs into the build
stamp, the ninth port row prints inside the button row, and numbers split across lines ("R1.3|7500");
the button set is rebuilt only at the 10 Hz repaint and **DONE sits exactly where FRONT 1st and the
distance answers appear next, so a reflexive second tap silently answers the next question** (a wrong
front or distance can end SAVED). PROS 4.2.2 draws text through VEXos's own display routines (not LVGL)
in three sizes (SMALL, MEDIUM, LARGE); `print_at` ignores the centre formats; **the rendered string is
re-read as a format** (a `%` on screen is a hazard) and **leaks its buffer on every call** (redraw only
on change); character widths have NEVER been measured (Bench Tests has a SCREEN RULER, button 9).

---

## 1. Scope — Part F1 (the library; no `src/` change; host-proven)

### F1.1 One push, 48 in, forward, straight (decision 2)
Replace `PushCapture` + `ScalePush` with ONE unpowered push: the operator pushes the robot FORWARD
(front leading) `CalibratorConfig::pushDistance` (48 in, fixed, stated) along a tape, straight. From it:
- the DRIVE ports (moved ≥ `movedThreshold`) and each one's SIGN (direction, inverted by BACK 1st),
  exactly as today;
- the straightness (accumulated IMU yaw over the push; ≤ `straightBound` 3° else refuse
  "the push curved by X deg -- push it again, straight" — this is also the push-yaw bound the sign
  capture was missing);
- each candidate's sign-corrected TRAVEL, kept. The per-side SCALE (`pushDistance` ÷ the side's
  MEDIAN travel, rule 4 unchanged) is computed only after the sides are known (§F1.2), and the minimum
  travel (`minPushRad`, one wheel turn) is checked per side then, as today.
`Answer::Distance24/48/72` and the distance question go; `Answer::Done` ends the push. The rule that
**DONE is only accepted once the push is plausibly complete** is a readiness query (F1.5), not a new
refusal. Keep rule 5: the push is powerless.

### F1.2 Sides from a RAMPED single-member pulse (decision 1)
For each drive port in turn, with every other candidate at 0 V + Coast: pulse `sign × V` for
`pulseTime` at `pulseLevels = {3.0, 6.0, 9.0}` V, **stopping at the first level whose accumulated yaw
reaches `pulseYawFloor` (2.0°)**; between levels and between ports wait for REST — **motor velocities
AND the IMU's yaw rate** under their floors (new `restYawRateFloor`; fixes the `atRest()` divergence).
The side is the yaw's sign exactly as today (HA-134: LEFT forward ⇒ clockwise ⇒ negative yaw). The
own-encoder confirmation stays (a contradiction refuses naming the port). Record per port: the level
used (V), the yaw (deg), the port's own travel, and the LARGEST travel any OTHER drive port showed.
**When no level reaches the floor, classify instead of refusing blind:**
- **DECOUPLED** — the port's own shaft moved ≥ `decoupledOwnMin` (say 20° of shaft) while NO other
  drive port moved ≥ `movedThreshold`: its motor turns but does not drive its gear train (a slipping or
  stripped gear — port 18's likely fault). The port is **LEFT OUT** of the description with a WARNING
  naming it ("port 18 turns but does not move the robot -- its gear is slipping; it is left out"), and
  the run continues. It is never commanded again; the DONE screen and the report list it.
- **ROLLED** — other drive ports moved (the train and the robot moved) but the yaw stayed under the
  floor: the robot rolled instead of turning. REFUSE naming the port and saying so ("port 11 pushed
  the robot forward instead of turning it, even at 9 V -- ...").
- **STILL** — nothing moved even at 9 V: REFUSE ("port N could not move the robot at 9 V -- on the
  floor? stuck?").
After all ports: each side must keep ≥ 1 member (the existing one-sided refusal), and the result must
still pass `DrivetrainDescription::valid()`.
Safety: 9 V for 0.4 s on ONE motor is a visible jerk, not a drive; STAND CLEAR is on screen; ABORT is
always live; a missed tick is still a stop (rule 7).

### F1.3 The spin: two guards and a ramp (safety — every design needs these)
- **Wrong-way guard:** refuse IMMEDIATELY when `spinDirection × yaw < −spinWrongWay` (15°): "the
  robot spun the wrong way -- the sides are swapped".
- **Start check + ramp:** if `|yaw| < spinStartYaw` (5°) after `spinStartTime` (2 s) at `spinLevels[0]`
  (3 V), step to `spinLevels[1]` (6 V) and check again; still not turning ⇒ refuse "the drive cannot
  turn the robot at 6 V" — never a 20 s stall.
- The width math, the agreement check and the fight monitor are unchanged.

### F1.4 The host test plant made able to FAIL like the robot (the proof)
In `include/shulib/sim/drive_plant.hpp`, ADDITIVE, with every existing suite bit-identical by default:
- **Coast-follow:** a wheel whose members are ALL at 0 V + Coast follows the body's motion by a
  configured fraction κ (0 = today's pinned wheel, the default; 1 = free rolling) instead of being
  pinned — so a one-sided push can roll the robot rather than pivot it. The plant must read the
  members' BRAKE MODE to know "coast" (today it reads only `commandedVoltage()`).
- **`MemberFault::Decoupled`:** the member's command does not reach its train and its encoder reports
  its own free spin — port 18's case.
- The calibrator's test rig moves to **kS = 1.2** (the tree's usual value) and states κ per test.
Required cases (each must FAIL on the old code or a named mutant): the old 3 V single pulse at kS 1.2
turns 0.00° (tonight, as a regression that documents the finding); the ramp reaches the floor at 6 V
or 9 V on a plant where 3 V fails; a Decoupled member is LEFT OUT with the warning and the run SAVES a
description without it; κ = 1 at 9 V is the ROLLED refusal naming the port; hand motion during rest
never leaks into a pulse (the yaw-rate rest); a wrong-way spin refuses within 15° (a hook that inverts
the IMU during the spin); a spin that cannot start refuses in ≤ 2 s at each level; a curved push
refuses; the single push produces the same scales as today's two pushes did on a straight 48 in push.
**The plant's numbers are invented** (κ, kS, the fault) — the tests prove the LOGIC's branches against
physics that can fail, not the robot's physics. Say so in the test banner.

### F1.5 What the screen needs from the calibrator (for F2's panel)
- `CalibratorStatus` gains numbers, not prose: moved-now of candidates; push travel so far vs the
  distance's expected travel (a "far enough" flag); live drift (deg); pulse k of N, current port, the
  level (V), the live yaw; per-port side as it is decided; spin direction and turns; the excluded ports.
- Readiness queries built from the SAME predicates the finish functions use — `pushReady()` (≥ 2
  moved AND far enough AND at rest) — so the gate cannot drift from the finish (as test 21 pins
  `accepts()`).
- A `Refusal` code (+ `refusalPort()`) for every refusal, so the STOPPED screen can say it in plain words
  keyed on a code, never on prose. **No `%` in any reason** (the spin-disagreement reason has one today —
  use "pct").
- A per-step serial record: the push table at DONE (every candidate: travel, moved?), one line per
  pulse (port, level V, yaw, own travel, others' max, verdict), the spin per direction.

### F1.6 `include/shulib/calibration/calibrate_panel.hpp` — the screen as a PURE model
PROS-free, allocation-free, ASCII-only, host-tested — the part of the UI that decides WHAT the screen
says and WHICH touches mean something, so it is unit-tested rather than eyeballed:
- `PanelView viewFor(const DrivetrainCalibrator&, double secondsSinceLayoutChange)` — per state: a
  header colour/word, ONE instruction (≤ 2 LARGE lines), ONE big live number, a NEXT line, and the
  buttons that are LIVE (each with its rectangle), per the layout in §2.
- `int hitTest(const PanelView&, x, y)` with padded boxes; greyed buttons never hit.
- `TouchGate`: release-edge only, ignores error reads, an ARMING DELAY (~0.5 s, buttons shown greyed
  meanwhile) after every layout change, and re-checks `accepts()` before an answer.
- `wrapWords(text, cols)` — never splits a word or a number, never drops text.
- The STOPPED wording per `Refusal` code: a LARGE headline, a WHY line, a DO line.
- `constexpr` checks: every literal is ASCII, contains no `%`, and fits its line budget (budgets are
  constants — LARGE ≤ 18, MEDIUM ≤ 36, SMALL ≤ 50 characters — ESTIMATES until the SCREEN RULER is
  read; changing one constant must be all it takes).
Required host cases (adversarial): every drawn live button is accepted by the calibrator in every
reachable state; DONE is not drawn at 0 or 1 moved, nor before far-enough; ABORT has the identical
rectangle in every state; **no answer button of a new layout overlaps the button that caused the
transition** (the double-tap hazard); the arming delay makes a second tap within 0.5 s land on nothing;
wrapping; the ASCII/`%`/budget checks.

### F1 — OUT
`src/` (F2), the worksheet (F2), the record format (`calibration_record.hpp` — same fields,
`kRecordVersion` stays 1; an excluded port is simply absent from the description), `boot_check.hpp`
(an extra motor the record does not name is already a warning, never a refusal).

---

## 2. Scope — Part F2 (the program and the runbook)

### F2.1 `src/calibrate_program.cpp` becomes a RENDERER of `viewFor()`
It redraws only the zones whose text or colour changed (the PROS print leak; no flicker), recomputes
the view right after every answer and every step change (never waits for the 10 Hz repaint), feeds
touches through `TouchGate`, writes F1.5's serial lines, and keeps everything the program does today
that is right (the census, the IMU discovery, the cartridge belief, AllStopGuard, the per-tick
`competition_is_disabled()` exit, write → flush → re-read → compare → SAVED).
**RUN AGAIN (decision 3):** on the DONE and STOPPED screens, a big RUN AGAIN button rebuilds the
calibrator (a new `DrivetrainCalibrator` + `start()`; the IMU is already calibrated) with no state
carried between runs.

### F2.2 The layout (480 × 240 usable; every pixel at y < 236)
- **Header bar** y 0–26, its colour the state: BLUE waiting/unpowered, AMBER powered, GREEN saved,
  RED stopped. The step word left; the build time right, SMALL.
- **Instruction:** LARGE, two lines. **Live number:** LARGE, green/amber/red. **NEXT:** MEDIUM, one line.
- **Button row** y 180–234: the primary slot left; **ABORT at the same rectangle in every state**
  (right). In the two POWERED phases ABORT grows to the whole row.
- **Questions** (FRONT / BACK) are big boxes in the MIDDLE of the screen, never in the primary slot, so
  a repeated DONE tap cannot answer them.

| State | Instruction | Live number | Buttons | NEXT |
|---|---|---|---|---|
| IMU wait | HANDS OFF / IMU WARMING UP | seconds | ABORT | push it 48 in along the tape |
| Push | PUSH IT FORWARD / 48 IN ALONG THE TAPE | `7 of 9 turned` + drift; green when ready | DONE, shown only when `pushReady()` | let go, then DONE |
| Which end | WHICH END / WENT FIRST? | — | FRONT, BACK (middle) | it twitches each motor -- stand clear |
| Pulses (AMBER) | STAND CLEAR / TWITCHING MOTORS | `port 12  3 of 9  6 V` + the live yaw vs 2 deg; L/R chips as decided | ABORT (whole row) | it spins -- stand clear |
| Spin (AMBER) | STAND CLEAR / SPINNING | `left 0.8 of 2 turns` | ABORT (whole row) | done |
| Saved | SAVED (green) | `LEFT 5  RIGHT 4`, `ipr L 1.37500 R 1.37500`, `track 13.700 in`, LEFT OUT ports if any | RUN AGAIN | copy these -- run again to check |
| Stopped | STOPPED -- MOTORS OFF (red) | the headline per code, WHY, DO | RUN AGAIN | — |

### F2.3 The runbook, rewritten from the code
`docs/internal/chunks/R3a-BENCH-WORKSHEET.md` Station G: every quoted screen string copied from the
new source; the one-push flow; the DONE rule; the pulse levels and what a jerk looks like (the
2-degree floor ≈ the front corner moving a quarter to a third of an inch); LEFT OUT and what it means;
RUN AGAIN; the wheels-up run (G.3) made OPTIONAL (it still refuses by design; the team lead skipped
it); "how to know it calibrated correctly" (SAVED; the MEASURED line — ports per side, `ipr` near 1.375
for 2.75 in wheels driven 1:1 and L ≈ R, the track width about the tape-measured centre-to-centre or
somewhat larger; a second run reproducing within 2 % / 3 %; then Teleop's `est x` ≈ 48 after a 48 in
drive).

---

## 3. Rulings

**F-R1 — Decisions 1–3 are the team lead's** (above), made with the alternatives and their costs in
front of him; the recommended hand twist is recorded as the rejected alternative, with its reasoning,
in the deviations table.
**F-R2 — A decoupled member is LEFT OUT, not refused.** Refusing would mean a robot with one slipping
gear can never be calibrated; guessing its side would be inventing. Leaving it out is the truth: its
motor cannot drive the robot. It stays in the gear train, unpowered (drag); the boot check already
treats a present motor the record does not name as a warning. The report and the DONE screen name it.
**F-R3 — The gate lives in readiness queries, not in `answer()`.** `answer()`'s contract ("no silent
touch") is unchanged; the panel simply never draws DONE before `pushReady()`.
**F-R4 — Every threshold stays INVENTED** (HA-136: pulse levels, `restYawRateFloor`, `decoupledOwnMin`,
the spin levels and guards, `pushDistance` is a choice not a belief) until this robot's runs settle them;
the serial record is what will settle them.

---

## 4. DoD
- [ ] **F1:** §1 in full; every public member documented; ARM gate; PROS-free/sim-free guards
      (`calibrate_panel.hpp` includes no `pros/`); every required case in F1.4 and F1.6 with its
      mutation RUN on a fresh binary and observed red (flip the side rule; drop the yaw-rate rest; drop
      the decoupled classification; drop the wrong-way guard; drop the spin-start check; draw DONE before
      ready; move ABORT; let a new answer button overlap the old DONE; drop the arming delay); every
      pre-existing suite bit-identical except the calibrator's (whose rewritten cases are listed with
      why); `src/` byte-identical; the documentation below; the log.
- [ ] **F2:** §2 in full; six builds link, zero disallowed warnings (`-Wformat-nonliteral`,
      `-Wformat-truncation` — only a literal `"%s"` ever reaches `print_at`), beacons unchanged; the
      worksheet rewritten from the code; a list of every on-screen string with its size and budget in the
      log; no hardware claim.

## 5. Documentation, in each part's commit
- **F1:** `docs/api/` (regenerated: `drivetrain_calibrator.md`, new `calibrate_panel.md`,
  `drive_plant.md`); `docs/hardware-assumptions.md` — HA-134 (the ramp), HA-136 (fields added and
  removed), HA-137 (the coast-follow rule and the calibrator no longer resting on the pinned wheel);
  `docs/changelog.md` (the calibrator's public surface changed — unfrozen, no bump); the R3d brief's T3
  amendment note; `build-order.md` R3d status + Next; `roadmap.md` pointer.
- **F2:** the worksheet's Station G; `build-order.md` + `roadmap.md` pointers; `R3d-PROGRESS.md`.
Prose review per part (grep docs/, README.md, include/ and src/ comments for `SideAssign`, `ScalePush`,
`Distance24`, `pulseVolts`, `tape push`, `twitch`, `STAND CLEAR`, `44`) — run BEFORE claiming it.

## 6. Landmines
1. `-Wformat-nonliteral` / `-Wformat-truncation` are errors in `src/`; only a literal `"%s"` reaches
   `print_at`; size every buffer from its budget.
2. **No `%` on screen, ever** (PROS re-reads the rendered string as a format). **ASCII only.**
3. The screen is RESISTIVE and reads on RELEASE; x/y are stale when idle; error reads return −1.
4. Usable height is 240; keep every pixel at y < 236.
5. Rule 1 (one place for a sign) and rule 6 (coast on every stop) are unchanged — the ramp and the spin
   ramp re-command every tick and stop on a missed tick.
6. Removing `Phase::SideAssign`/`ScalePush` fields fails the doc freshness gate until `docs/api` is
   regenerated; tests 6–9, 17 and 21 reference them.
7. The plant change must keep every other suite bit-identical by default (κ = 0, no fault).
8. **No hardware claim.** Calibrate has never saved a record; say so where the program already does.

## 7. Before F2's layout is final — one minute on the robot
Bench Tests → **9 SCREEN RULER**: read the last visible digit on the SMALL, MEDIUM and LARGE rows and
whether the frame is whole. The budgets in F1.6 become measured numbers (one constant each).

## 8. Verification (the coordinator, per part)
The handoff §3 command set; read the ramp and classification (F1.2), the spin guards (F1.3), the
plant's coast-follow (F1.4) and `viewFor`/`TouchGate` (F1.6) in full; re-run by hand: flip the side
rule; drop the wrong-way guard; draw DONE before `pushReady()`; after F2, rebuild the calibrate package
from the committed tree, confirm the beacon, and read every screen string against the worksheet.
