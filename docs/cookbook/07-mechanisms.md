# 7 — Mechanisms

A robot's mechanisms are declared once in its robot file ([chapter 10's mechanism
section](../guide/10-the-api.md#mechanisms--declared-once-wired-separately)) and then run from ONE line
of an auton and from a driver file's controls ([chapter 18](../guide/18-mechanisms.md)). This chapter is
the recipes. **Nothing here has run on a robot** — every number below is the host test plant's
(invented models, HA-183). Robot two's programs run on the mechanism service since the core's fifth part
(2026-10-29): its claws are declarations, and its driver file is [chapter 6](06-driver-control.md)'s last
section.

Listings are compiled and run in
[`test/mechanism_auton_example_test.cpp`](../../test/mechanism_auton_example_test.cpp), case
`cookbook-07a`, and [`test/routine_trigger_example_test.cpp`](../../test/routine_trigger_example_test.cpp),
case `cookbook-07b`, over robot x (an invented robot with one of each kind of mechanism).

---

## An arm with presets, held while the robot drives

**Use this when:** an arm (or a lift) must get to a height and STAY there while the robot drives — to
carry a piece, to clear a wall, to score at the end of the drive.

The auton moves the arm to its preset, then drives. Nothing more is needed to keep it up: after a
`move` step the arm HOLDS, every tick, inside the drive's own wait.

```cpp
RoutineResult scoreHigh(AutonContext& ctx) {
    Routine r{ctx, "score-high"};
    r.move(arm["score"], 1.5_s);  // the arm moves, then HOLDS 95 degrees ...
    r.driveTo(40_in, 0_in);       // ... while the robot drives
    r.move(arm["high"], 1.5_s);   // then on up, and it holds there
    return r.result();
}
```

and the same presets on the driver's buttons, beside a hold-to-run that jogs it in between:

```cpp
whileHeld(arm, Button::R1, Button::R2, 6_volt),  // up / down while held; let go: it holds there
preset(arm["stow"], Button::Down, 1.5_s),
preset(arm["score"], Button::Right, 1.5_s),      // the most recent button wins
preset(arm["high"], Button::Up, 1.5_s),
```

**Why it is written this way.**

- **The hold is the mechanism service's, not your loop's.** Every shulib wait — a drive, a turn, a
  pause — ticks the service, so the arm's hold loop (P + I + D and its gravity term) runs through the
  whole drive. On the host test plant the arm held 95° within 1.16° through a 1.8 s, 40 in drive,
  with 2.0 V of gravity where its declaration says 1.6 (the hold's integral covers the difference).
- **A preset, not a number.** `arm["score"]` is checked AT BUILD against the arm's presets and limits:
  a misspelt preset or one past a limit does not compile. `arm.to(95)` works too, also checked.
- **Each step states its own time limit.** 1.5 s is the budget for THIS move; out of time it says
  TIMED_OUT, the routine stops (the drive is safed, the rest skipped), and the arm holds where it got
  to — it never keeps pushing toward a target it could not reach.
- **On the buttons, the most recent input wins.** A preset pressed while R1 is still held takes over,
  and R1 then does nothing until you let it go — your thumb never fights the preset back.

**Watch out for:**

- **A busy loop of your own starves the hold.** `pros::delay` in an auton (or a `then()` action that
  blocks) never ticks the service: the arm is left at its last command — not held, not limited. Wait
  with `r.pause(…)` or `r.waitFor(…)`.
- **The end of the auton lets it down.** The run guard's end of run puts every mechanism in its
  `.safe` state — an arm with `.safe = Hold` then rests on the motor's own Hold (no gravity term),
  and whether that keeps a LOADED arm up is unmeasured. Driver control starts the same way.
- **An arm homed by a switch or a stall needs a way to home in driver control** (`homeAtDriverStart` or
  `homeOnPress`), or the build refuses its buttons: after a program restart off its home it is not
  homed, and its presets would end NOT_HOMED for the whole match. (`.unhomedVolts` is enough only for
  an arm that is HELD and never preset — a hold moves it unhomed but never homes it.) Robot x's arm
  rests on its stop at boot, so it needs none.
- **A 6 V hold-to-run into something may never trip the jam signature.** The recipe's 2.0 A is a
  full-power stall; on the host test plant a 6 V push stopped by an obstacle draws about 1.25 A, so
  only letting go of R1 ends it. State the arm's `.amps` below what its hold draws when stopped, or
  accept that the driver's thumb ends that push
  ([chapter 18's jam section](../guide/18-mechanisms.md#jam-signatures--what-a-stall-looks-like-and-the-recipe)).

---

## Start the lift partway through a drive

**Use this when:** a lift (or an arm, an intake, a flywheel) should be on its way up while the robot is
still driving — but not from the very start of the drive: raised too early it tips the robot, blocks
the camera, or hits a field element on the way.

One line holds the mark, the START and the drive it rides. The lift starts the tick the robot is 60 %
of the way along the drive; the routine collects it afterwards, like any start form's step.

```cpp
RoutineResult liftOnTheWay(AutonContext& ctx) {
    Routine r{ctx, "lift-on-the-way"};
    r.atPercent(60).startMove(lift["high"], 5_s).driveTo(40_in, 0_in);  // at 24 in, the lift starts up
    r.waitUntilDone(lift, 3_s);      // up by now (or the routine says why it is not)
    r.run(intake, -12_volt, 0.5_s);  // score
    return r.result();
}
```

**Why it is written this way.**

- **It saves the lift's whole climb from the drive's time.** On the host test plant the lift needs
  4.1 s for its 22 in. Written lift-THEN-drive, this auton ends at 6.45 s; with the trigger it ends
  at 5.06 s — the lift rises while the robot covers the last 16 in, and keeps rising after.
- **"60 %" is distance along the line from where the drive started to its target** — 24 in of this
  40 in drive — not time, and not the distance still to go. `afterDistance(24_in)` says the same in
  inches; `afterTurning(45_deg)` is the mark for a turn
  ([Chapter 9](../guide/09-the-recipe-api.md#partway-through-a-drive-triggers)).
- **The bound is the lift's own, counted from when the trigger fires.** `5_s` is the move's time limit,
  as on any `startMove`; `waitUntilDone(lift, 3_s)` is how much longer the routine will wait for it
  once the drive is done.
- **The trigger is part of the drive step.** If the lift's start is refused when the robot reaches the
  mark (not homed, not plugged in), the drive still finishes — then that step fails, saying why, and
  the routine stops (or carries on, under `r.onFail(carryOn)` written before the mark).

**Watch out for:**

- **A drive that ends early fires nothing.** If the drive times out at 50 %, the lift never starts, and
  the `waitUntilDone` fails — counted, "the trigger never fired" — rather than waiting for a lift that
  is not moving. Give the drive a timeout it can meet, or put `r.onFail(carryOn)` in front of the wait.
- **A mark past the end never fires.** A drive that arrives just short of its target (0.24 in short
  here) still fires a mark at 100 % or at its full length — but not "after 50 in" on a 40 in drive;
  the routine logs that it did not fire.
- **Write the drive on the SAME line.** A trigger line that ends without its drive is a compiler
  warning (an error in this project's own builds); the routine's next step refuses, and a routine that
  ends with one still waiting logs that it never fired.
- **Nothing here has run on a robot.** How soon after the mark the real lift starts to move, and how
  close the robot's estimate of "60 %" is to the tape, are unmeasured.
