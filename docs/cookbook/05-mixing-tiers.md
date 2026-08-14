# 5 — Mixing tiers

A recipe is a convenience layer over the full API, not a walled garden. When a step cannot say
what you need, you call the `Chassis` verb directly — mid-routine, for one leg, and then carry on
chaining. That is the intended shape, and it has exactly one sharp edge, which this chapter is
mostly about.

Listings are compiled and run in
[`test/cookbook_examples_test.cpp`](../../test/cookbook_examples_test.cpp), cases `cookbook-05a`
and `cookbook-05b`.

---

## Call the full API mid-routine

**Use this when:** one leg needs something no step offers — direct velocity control, a branch on
the exit reason, a wait you want to inspect.

Because steps run as they are chained, a direct call between two steps simply happens between
them. There is nothing to configure and no mode to enter:

```cpp
Routine r{c.chassis, "unguarded"};
r.startAt(Pose2d{0_in, 0_in, 0_deg});
// Not a step. 0.3 s cannot cross 60 inches.
const ExitReason direct = c.chassis.moveTo(Pose2d{60_in, 0_in, 0_deg},
                                           {.timeout = 0.3_s});
```

**The sharp edge.** A direct call is *not a step*, so the chain never learns how it went:

```cpp
CHECK(direct == ExitReason::TimedOut);
r.brake({.timeout = 1.5_s});
CHECK(r.ok());              // the chain is cheerful…
CHECK(r.result().steps == 2);  // …and never counted the failed move
```

The routine reports success. Its step count does not include the move. And — the part that
actually loses matches — every subsequent step runs, from a position the robot never reached.
Everything the chain does for you about failure (stop, park, skip, report) applies only to steps.

---

## Make a direct call count as a step

**Use this when:** you dropped a tier for one leg and you still want the chain's failure handling.

Wrap it in `then()`. An action that returns an `ExitReason` has that verdict honored, so the leg
becomes a step in every way that matters:

```cpp
Routine r{c.chassis, "guarded"};
r.startAt(Pose2d{0_in, 0_in, 0_deg})
    .then([&c] {
        return c.chassis.moveTo(Pose2d{60_in, 0_in, 0_deg}, {.timeout = 0.3_s});
    }, "long-approach")
    .driveTo(60_in, 24_in, {.timeout = 5_s})
    .brake({.timeout = 1.5_s});
```

Now the failure behaves like any other:

```cpp
CHECK_FALSE(r.ok());
CHECK(r.result().cause == RoutineStopCause::ActionFailed);
CHECK(r.result().exit == ExitReason::TimedOut);  // the facade's verdict, kept
CHECK(std::string{r.result().stoppedName} == "long-approach");
CHECK(r.result().skipped == 2);
```

**Why it is written this way.** `then()` accepts actions returning `void`, `bool`, `ExitReason`,
or — since the mechanism seam landed — a `MechanismOutcome`. The `ExitReason` form exists for
exactly this: gluing a full-API call into a chain without translating its verdict into something
coarser. The cause is reported as `ActionFailed` rather than `MotionFailed` — the chain is telling
the truth about *what it ran*, which was your action — but the motion's own `TimedOut` survives in
`exit`.

The fourth form is worth knowing even if you are not using mechanisms yet: for a
`MechanismOutcome`, **only `Succeeded` continues the chain**, so an *unconfirmed* grab can never
read as success. That distinction is the whole reason the mechanism layer reports an outcome
rather than a bool ([guide Chapter 13](../guide/13-extending-the-library.md)).

**When you want the unguarded form instead:** when the whole point is to branch yourself. "Try
the fast route; if it times out, take the slow one" is a decision the chain cannot make, so make
it in plain C++ and keep the chain for the parts that are linear.

**Watch out for:** deciding this per-leg by accident. Both forms are one line and they look
similar. A useful habit: if the next step would be wrong when this leg fails, it belongs in
`then()`.

---

## Re-seed the estimate mid-routine

**Use this when:** the robot has just done something that tells you where it really is — squared
itself against a wall, driven into a fixed structure, or passed a sensor you trust more than
odometry.

```cpp
// Pretend the robot has just squared itself against the wall at x = 36 in:
// the measurement is better than the estimate, so overwrite the estimate.
// (The step is called startAt even here — see the chapter's note.)
r.startAt(Pose2d{36_in, 0_in, c.chassis.pose().heading()});
```

**Why it is written this way.**

- **Seeding changes belief, not position.** The robot does not move; the estimator's idea of
  where it is does. The compiled test asserts exactly that: the estimate jumps to 36 inches and
  the simulator's ground truth does not move at all.
- **Keep the current heading.** Position is what a wall measurement gives you. Heading belongs to
  the IMU, which does not drift the way odometry position does
  ([Chapter 3](../guide/03-knowing-where-you-are.md)) — passing `chassis.pose().heading()` says
  "correct where I am, not which way I face".
- **It is a step**, so it participates in the chain: if the routine has already stopped, the
  re-seed is skipped like anything else, rather than quietly rewriting the estimate of a routine
  that is no longer running.

**Watch out for the name.** The step is called `startAt`, and here it is not the start of
anything. The recipe layer has one spelling for "seed the estimate" and it was named for its
first and most common use. Read it as "the robot is at", not "the robot begins at".

**And watch out for the temptation.** Re-seeding is a correction, not a fix for a routine that
keeps missing. If the estimate is wrong enough to need correcting mid-auton, the interesting
question is why ([Chapter 12](../guide/12-when-things-go-wrong.md)).

---

*Back to the [index](README.md), or on to [Chapter 10 — the API, as prose](../guide/10-the-api.md)
for the full verb behind each step.*
