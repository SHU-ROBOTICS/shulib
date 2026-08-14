# 4 — Drivetrains

shulib never pretends a drivetrain can do something it cannot. These recipes are how you write
routines for the two cases where that honesty changes your code:  a drivetrain that cannot slide
sideways at all, and one that can but slowly.

Listings are compiled and run in
[`test/cookbook_examples_test.cpp`](../../test/cookbook_examples_test.cpp), cases `cookbook-04a`
and `cookbook-04b`.

---

## A tank routine

**Use this when:** your drivetrain is a tank (two sides, no lateral wheels). It cannot strafe, so
every leg is turn-then-drive.

```cpp
Routine r{c.chassis, "tank-side"};
r.startAt(Pose2d{-24_in, -24_in, 0_deg})
    .face(-24_in, 12_in, {.timeout = 3_s})
    .driveTo(-24_in, 12_in, {.timeout = 8_s})
    .face(12_in, 12_in, {.timeout = 3_s})
    .driveTo(12_in, 12_in, {.timeout = 8_s})
    .brake({.timeout = 1.5_s});
```

**Why it is written this way.**

- **`face` then `driveTo`, always, in that order.** `face(x, y)` turns the robot to point at a
  field location; `driveTo(x, y)` drives there with that same bearing as the target heading, so
  the approach is a straight line the drivetrain can actually follow.
- **You are still writing the turn.** shulib's motion verbs never insert a turn on your behalf —
  that is a deliberate rule, because a library that silently turns your robot is a library you
  cannot predict. `face(x, y)` *is* your turn, written in field words instead of `atan2`.
  ([Chapter 10](../guide/10-the-api.md) shows the hand-written version; they produce identical
  motion.)
- **The bearing is computed when the step runs**, not when you write it. If the previous leg
  ended a little short, `face` still points at the real target from where the robot really is.

**Watch out for:**

- **`strafeTo` on a tank.** It will not error at the call — it runs, fails to move sideways, and
  honestly returns `TimedOut` at whatever budget you gave it. That is the library refusing to
  pretend, but it costs you the whole timeout, and in a chain it stops the routine.
- **`moveTo` to a pose that is sideways of you.** Same outcome, same reason. On tank, treat
  `moveTo` as "drive to a point I am already pointed at, and finish at this heading".
- **Turning costs time you have to budget.** The routine above spends about six seconds of
  simulated time on two turns and two drives. On an X-drive the same route needs no turns at all.

---

## Budget a sideways leg

**Use this when:** your drivetrain strafes, but not at full speed — an H-drive, where a single
lateral wheel does all the sideways work.

The cost of getting this wrong is a leg that times out for no reason you can see. A sideways leg
on a drivetrain with 35% lateral authority takes roughly three times as long as the same distance
forward, so a timeout computed from distance and top speed is about three times too small.

```cpp
Time lateralBudget(Chassis& chassis, Length distance, double safety) {
    const double reach =
        chassis.motionConfig().maxLinearSpeed.value() * chassis.strafeAuthority();
    return Time{safety * std::abs(distance.value()) / reach};
}
```

Used:

```cpp
const Time budget = lateralBudget(c.chassis, 18_in, 3.0);
```

```cpp
Routine r{c.chassis, "h-lateral"};
```

```cpp
r.turnTo(0_deg, {.timeout = 3_s}).strafeTo(0_in, 18_in, {.timeout = budget});
```

**Why it is written this way.**

- **`strafeAuthority()` is the fraction of the linear speed budget the drivetrain can sustain
  sideways** — 1.0 on an X-drive, about 0.35 on the H-bot in these examples, 0 on a tank. It is a
  read-only query, available at authoring time. **But read what it is honest about:** on an
  H-drive it returns `strafeSpeedRatio × strafeTractionDerate`, and both are values *you*
  configured. The library is not measuring your robot's real sideways grip — the shipped derate
  (0.35) is an invented placeholder, registered as such, and nobody has measured one on a
  physical H-drive. Treat the budget it produces as an estimate built on your own number.
- **`motionConfig().maxLinearSpeed` is the speed the leg will actually run at**, so the two
  together give the achievable sideways speed — and **dividing the distance by it** gives travel
  time, which is what the helper above does. (This bullet said *multiplying* until DOCS1. Speed
  times distance is in²/s, not seconds, and a reader who followed the prose instead of the code
  would compute a budget wrong by a factor of the speed squared.)
- **`safety` covers what the arithmetic does not model**: acceleration, deceleration, and the
  settle at the end. A factor of 3 is generous on purpose — a timeout is a bound, not a schedule,
  and a leg that finishes early costs nothing.
- The compiled test asserts both halves: the naive budget really does time out, and the
  authority-aware one really does settle. A recipe that only proved the happy half would not be
  telling you anything.

**Watch out for:**

- **Aim before you strafe.** `strafeTo` holds whatever heading it finds when it starts. The
  `turnTo` above is not decoration — it fixes the heading the strafe will preserve.
- **Do not try to detect the slow mode at runtime.** When an H-drive runs a lateral-heavy leg
  under reduced authority it shows up in the diagnostics (an `SFB` marker), and there is
  deliberately no way to poll for it in code. Budget for it at authoring time with
  `strafeAuthority()`, which is what this recipe does.
- **`strafeAuthority()` is 0 on a tank**, and dividing by it would give you infinity. If your
  code runs on more than one drivetrain, check for zero and use
  [the tank recipe](#a-tank-routine) instead.

---

*Next: [5 — Mixing tiers](05-mixing-tiers.md).*
