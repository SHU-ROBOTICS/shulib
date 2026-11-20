# CAL2 — COMPLETED (CAL2-A built 2026-11-17, resumed and finished 2026-11-19, LANDED 2026-11-19; host- and build-gated, NOT run on a robot in this form)

> Calibrate finds the drive's sides by COUPLING: a pulsed motor's side-mates turn with it through their shared gears, so
> the sides are found with the robot standing still; a short turn names LEFT; rest is judged by the heading's average
> drift (a steady DONE); the calibrator lives off the opcontrol stack. The team lead's go 2026-11-17 ("YES BE QUICK");
> landed after the Nov 18 library freeze on his word of 2026-11-19 ("fix it because i have to calibrate the other bot
> too"). Written FROM [`CAL2-PROGRESS.md`](CAL2-PROGRESS.md); brief [`CAL2-sides-by-coupling.md`](CAL2-sides-by-coupling.md).

## What landed

- **D1–D3** sides by coupling (followers at ≥ 2.0 deg, a 12 V fourth pulse level, named refusals for a port in both
  groups / a third group / a reversed follower), LEFT by the short turn, the yaw fallback for an uncoupled drive. Its
  main delta: every drive port is still pulsed once to confirm its group, so a slipping gear is still LEFT OUT (12 pulses
  on robot one, not the brief's 2).
- **D4** steady rest (the heading's change over `restHold`); **D5** the serial record's follower lines.
- **§6 the stack:** `detail::programStorage()` (`static constinit`, trivially destructible) holds the calibrator and the
  run record. GCC's deepest path at the save, robot one's build: 31,824 B → **23,552 B** of 32,768 (newlib's and VEXos's
  frames now have 9,216 B). T-STACK runs three whole runs on a painted thread stack.

## Rounds

Build (2026-11-17, cut twice by the weekly limit; finished 2026-11-19 with §6) → three reviewers → fix → a fresh
verifier (3 minor findings: V-F1 KEPT's own-shaft floor, V-F2 the drag rule, V-F3 the run histories) → fix2, all green.

## The coordinator's merge (2026-11-19, `integrate`)

Merged onto main `8b2d99e` (robot one's measured setup, the docs moves, the short-move test). Conflicts: the changelog
(both entries kept, CAL2's first) and the generated briefing (regenerated). Gates on the merged tree: see the merge
commit's message.

## Owed

- **On robot one: a run of this Calibrate** to the save (the stack's real test; GCC's measure is not VEXos's).
- **Robot two:** its Stage A facts read off the robot (the builders), then this Calibrate on it.
- The next stack item: `ProsDeviceProvider` is 14,696 B on `runCalibrate`'s frame (not moved).
