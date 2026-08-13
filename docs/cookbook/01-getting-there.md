# 1 — Getting there

Recipes for the shape of a routine: starting, visiting places, doing something at each, parking.

Every listing here is compiled and run in
[`test/cookbook_examples_test.cpp`](../../test/cookbook_examples_test.cpp), cases `cookbook-01a`
through `cookbook-01c`.

---

## The skeleton

**Use this when:** you are starting a new routine and want the smallest thing that is honest.

```cpp
RoutineResult skeleton(Chassis& chassis) {
    Routine r{chassis, "skeleton"};
    r.startAt(Pose2d{-48_in, -24_in, 90_deg})
        .moveTo(Pose2d{-24_in, -24_in, 90_deg}, {.timeout = 4_s})
        .brake({.timeout = 2_s});
    return r.result();
}
```

**Why it is written this way.**

- `startAt` is the first line of every auton. It tells the pose estimator where you physically
  placed the robot. Skip it and every field coordinate afterwards is measured from wherever the
  estimator happened to start, which is the origin — so the robot drives to the wrong place with
  complete confidence. (Heading is not seeded here; the IMU owns it. See
  [Chapter 3](../guide/03-knowing-where-you-are.md).)
- `brake` at the end is not decoration. Without it the routine returns while the last motion's
  residual velocity is still bleeding off, and "where the robot ended up" becomes a matter of
  luck. `brake` blocks until the estimate says the robot is genuinely at rest.
- The name `"skeleton"` appears in the log if anything stops the chain, so a transcript says
  *which* routine broke. Use a name you will recognise at a competition.
- Returning `r.result()` gives the caller the whole verdict — how many steps ran, which one
  stopped it, why. Returning nothing throws that away.

**Watch out for:** a timeout that does not cover the first motion's sensor-boot wait. The first
verb in a routine may spend up to about two seconds waiting for the IMU to finish calibrating,
and that wait comes out of the timeout you gave it.

---

## A reusable scoring step

**Use this when:** the same short sequence — drive, stop, do a thing, wait a beat — happens more
than once in a routine.

```cpp
Routine& scoreAt(Routine& r, Length x, Length y, Intake& intake) {
    return r.driveTo(x, y, {.timeout = 5_s, .maxLinearSpeed = Velocity{30.0}})
        .brake({.timeout = 1.5_s})
        .then([&intake] { intake.release(); }, "release")
        .pause(200_ms);
}
```

**Why it is written this way.**

- It **takes and returns `Routine&`**, so it chains exactly like a built-in step. This is the
  supported way to add vocabulary to the recipe layer: you never subclass `Routine`, you write
  free functions that take it.
- `Intake` here is a struct written by hand, not a shulib type — the library has no mechanisms
  yet. `then()` accepts any callable, so when mechanisms do arrive, `intake.release` slots into
  exactly this position:

```cpp
struct Intake {
    int grabs = 0;
    int releases = 0;
    bool nextGrabSucceeds = true;
    bool grab() {
        ++grabs;
        return nextGrabSucceeds;
    }
    void release() { ++releases; }
};
```

- The approach is capped at 30 in/s. A scoring approach that arrives fast arrives *approximately*;
  the cap costs a few tenths of a second and buys a repeatable stopping point.
- `pause(200_ms)` lets the mechanism finish. `pause` leaves the motors idle — it is not `hold`,
  which actively fights to keep the pose (and which you want instead if someone might shove you).

**Watch out for:** naming the action. `then(action)` with no name labels the step `"action"` in
the log, so a routine with four unnamed actions produces four indistinguishable lines exactly
when you need to tell them apart. Always pass a name.

---

## A two-goal side run

**Use this when:** your auton visits several places and does something at each — the ordinary
case.

```cpp
RoutineResult twoGoalSideRun(Chassis& chassis, Intake& intake) {
    Routine r{chassis, "left-two-goal"};
    r.startAt(Pose2d{-48_in, -24_in, 0_deg});
    scoreAt(r, -24_in, -24_in, intake);
    scoreAt(r, -24_in, 24_in, intake);
    r.moveTo(Pose2d{-48_in, -48_in, 0_deg}, {.timeout = 6_s}).brake({.timeout = 1.5_s});
    return r.result();
}
```

The test that compiles this listing also holds what the prose claims: eleven steps, six of them
motions, two releases, the robot genuinely within an inch and a half of the parking pose on the
simulator's ground truth — and the whole run finishing in about 7.5 seconds of simulated time,
inside a fifteen-second autonomous window with room to spare.

**Why it is written this way.**

- `driveTo(x, y)` (inside `scoreAt`) drives to a field *point* and arrives facing it. The heading
  is computed when the step runs, from wherever the robot actually is, so it stays correct even
  if an earlier leg ended slightly off.
- The chain is broken into statements rather than one long expression because `scoreAt` is a
  function call, not a member. This reads fine and costs nothing: steps run as they are chained,
  so the statement boundaries are invisible to the robot.
- Parking is a real `moveTo` to a chosen pose, not just a `brake` wherever the last goal was.
  Where you end an auton is worth points or is in someone's way; decide it.

**Watch out for:** assuming the whole run has a time budget. It does not — see
[Fit the match window](03-timing-and-partners.md#fit-the-match-window). Each step has its own
timeout, and eleven steps with generous timeouts can add up to far more than an autonomous
period.

---

## A waypoint sweep

**Use this when:** you want one call to drive a path, and you want to know how far it got if it
breaks.

```cpp
Routine good{c.chassis, "sweep"};
good.followTrajectory({Pose2d{12_in, 0_in, 0_deg}, Pose2d{24_in, 12_in, 45_deg},
                       Pose2d{24_in, 24_in, 90_deg}},
                      {.timeout = 8_s})
    .brake({.timeout = 1.5_s});
CHECK(good.ok());
CHECK(good.lastTrajectory().succeeded());
CHECK(good.lastTrajectory().completedLegs == 3);
```

**Why it is written this way.**

- `followTrajectory` is a chain of `moveTo` legs that **settles at every waypoint**. That is the
  documented motion model today: there is no blending through corners yet
  ([Chapter 14](../guide/14-what-it-cannot-do-yet.md)). So five close waypoints cost five settle
  times; prefer fewer, farther apart.
- The options — including the timeout — apply **per leg**, not to the whole path. `{.timeout =
  8_s}` over three waypoints permits twenty-four seconds, not eight.
- `lastTrajectory()` is the only place the leg count survives. The chain flattens a trajectory
  into pass/fail like any other step; `lastTrajectory()` is what keeps "it completed two of
  four", which is the fact a recovery plan needs.

**Watch out for:** a non-finite waypoint. The whole call is rejected before the robot moves at
all, rather than driving three legs and then throwing — which is what you want, but it means a
typo in waypoint five stops waypoint one.

---

*Next: [2 — When a step fails](02-when-a-step-fails.md).*
