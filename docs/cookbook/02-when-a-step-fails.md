# 2 — When a step fails

The recipe layer has one built-in answer to failure: **stop, park, skip the rest, say so.** These
recipes are about the decisions that answer leaves to you.

Listings are compiled and run in
[`test/cookbook_examples_test.cpp`](../../test/cookbook_examples_test.cpp), cases `cookbook-02a`
and `cookbook-02b` (plus `cookbook-01c` for the sweep).

---

## Bail out and park somewhere useful

**Use this when:** a step can fail in a way that makes the rest of the plan pointless — the grab
came up empty, so driving to the goal and releasing nothing is worse than useless.

```cpp
RoutineResult grabOrBailOut(Chassis& chassis, Intake& intake) {
    Routine r{chassis, "grab-or-bail"};
    r.startAt(Pose2d{-48_in, -24_in, 0_deg})
        .driveTo(-24_in, -24_in, {.timeout = 5_s})
        .brake({.timeout = 1.5_s})
        .then([&intake] { return intake.grab(); }, "grab")
        .driveTo(0_in, 0_in, {.timeout = 6_s})
        .then([&intake] { intake.release(); }, "score")
        .brake({.timeout = 1.5_s});
    if (r.ok()) {
        return r.result();
    }

    // Save the verdict BEFORE the fallback: a stopped Routine never runs
    // another step, so the fallback is a new chain with its own counters, and
    // `r` is the only record of what actually went wrong.
    const RoutineResult failure = r.result();
    Routine bail{chassis, "grab-or-bail/fallback"};
    bail.driveTo(-48_in, -48_in, {.timeout = 6_s}).brake({.timeout = 1.5_s});
    return failure;
}
```

**Why it is written this way.**

- **The grab is an ordinary step.** `then()` with an action returning `bool` makes `false` a step
  failure, so a missed grab stops the chain exactly the way a timed-out move would. You do not
  write any of the stopping logic.
- **You do not have to check anything for the robot to be safe.** By the time `r.ok()` is read,
  the chain has already put the drive at zero volts under brake and skipped the remaining steps.
  The `if` is where you decide to do something *different*, not where you prevent a crash.
- **The fallback is a second `Routine`.** A stopped chain stays stopped — every further step is
  counted as skipped and never runs — so recovery cannot live in the same chain. This is
  deliberate (a chain that resumed after a failure would be a chain you could not read top to
  bottom), but it is worth stating plainly: *there is no `.finally()`*.
- **Save `r.result()` first.** The fallback chain has its own counters. If you return
  `bail.result()` you have thrown away the only record of what actually failed.

**What the log says.** One `Warn` and nothing else invented:

```text
routine 'grab-or-bail' STOPPED at step 4 (grab): action FAILED — skipping the rest; drive safed
routine 'grab-or-bail': step 5 (driveTo) skipped — stopped at step 4
```

**Watch out for two things.**

1. **`result().exit` is `Running` after a non-motion stop.** That reads like "still going", and
   it is not: it means "there is no motion verdict here, because what failed was an action or a
   wait". Branch on `result().cause` first (`ActionFailed`, `WaitTimedOut`, `MotionFailed`), and
   read `exit` only when the cause is `MotionFailed`.
2. **Step numbers count `startAt`.** In the routine above the grab is step 4, not step 3.

---

## Attempt something and keep going

**Use this when:** one failure should *not* end the routine — a sweep across three goals where
missing the middle one is disappointing but not fatal.

The chain has no "try this step" mode: `then()` stops on `false`. So the attempt returns `void`
and puts its outcome somewhere you own.

```cpp
Routine& attemptGrab(Routine& r, Intake& intake, std::vector<bool>& outcomes) {
    return r.then([&intake, &outcomes] { outcomes.push_back(intake.grab()); },
                  "attempt-grab");
}
```

Used in a sweep:

```cpp
Routine r{c.chassis, "sweep-three"};
r.startAt(Pose2d{0_in, 0_in, 0_deg});
r.driveTo(24_in, 0_in, {.timeout = 5_s});
attemptGrab(r, intake, outcomes);
intake.nextGrabSucceeds = false;  // the middle stop comes up empty
r.driveTo(24_in, 24_in, {.timeout = 5_s});
attemptGrab(r, intake, outcomes);
intake.nextGrabSucceeds = true;
r.driveTo(0_in, 24_in, {.timeout = 5_s});
attemptGrab(r, intake, outcomes);
r.brake({.timeout = 1.5_s});
```

**Why it is written this way.** A `void` action always succeeds as far as the chain is concerned,
which is precisely the semantics you want here: the *routine* did not fail, the *grab* did. The
outcome goes into `outcomes` so the code after the routine can decide what two-out-of-three
means.

**Watch out for the price, which is real.** Swallowing the failure this way makes it **invisible
in the transcript** — the routine layer logs nothing, because as far as it knows nothing went
wrong. The test for this recipe asserts that silence deliberately, so nobody can claim otherwise
later. If the miss matters, log it yourself inside the action.

There is a second, sharper trap here: this idiom looks almost identical to the failing one. The
only difference between "attempt and continue" and "attempt and bail" is whether the lambda
`return`s the bool. Changing

```
{ outcomes.push_back(intake.grab()); }
```

to

```
{ return intake.grab(); }
```

silently converts a tolerant sweep into a routine that stops at the first miss. Name your action
steps so the log tells you which one you wrote.

---

## Recover from a broken sweep

**Use this when:** a `followTrajectory` stopped partway and you want to do something sensible
about it.

```cpp
// Starve the PER-LEG budget (options are per leg, never per chain): the
// trajectory stops at the first leg that misses, and so does the routine.
Routine broken{c.chassis, "sweep-starved"};
broken.followTrajectory({Pose2d{36_in, 24_in, 0_deg}, Pose2d{-48_in, -24_in, 0_deg}},
                        {.timeout = 0.6_s});
CHECK_FALSE(broken.ok());
CHECK(broken.result().cause == RoutineStopCause::MotionFailed);
CHECK(broken.result().exit == ExitReason::TimedOut);

// How far it got is still readable — that is what a recovery plan needs.
const TrajectoryResult partial = broken.lastTrajectory();
CHECK(partial.completedLegs < partial.totalLegs);
CHECK(partial.totalLegs == 2);
```

**Why it is written this way.** The chain reduces every step to succeeded-or-not, and for a
trajectory that would lose the one fact you need: *how far*. `lastTrajectory()` keeps the full
result, so a recovery plan can say "we made it through two of four waypoints, resume from there"
instead of restarting a path from a position the robot is not at.

**Watch out for:** the robot's position after a mid-trajectory failure. It is not at a waypoint —
it is wherever the timeout caught it. Re-plan from `chassis.pose()`, and treat that pose as an
estimate ([Chapter 3](../guide/03-knowing-where-you-are.md)), not as a fact.

---

*Next: [3 — Timing and teammates](03-timing-and-partners.md).*
