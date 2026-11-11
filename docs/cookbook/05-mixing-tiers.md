# 5 — Mixing tiers

A recipe is a convenience layer over the full API, not a walled garden. When a step cannot say
what you need, you call the `Chassis` verb directly — mid-routine, for one leg, and then carry on
chaining. That is the intended shape, and it has exactly one sharp edge, which this chapter is
mostly about.

Listings are compiled and run in
[`test/cookbook_examples_test.cpp`](../../test/cookbook_examples_test.cpp), cases `cookbook-05a`,
`cookbook-05b` and `cookbook-05c`.

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
// Pretend the robot has just squared itself against a wall at x = 36 in:
// a wall measures ONE axis, so overwrite x and keep y and the heading.
// (The step is called startAt even here — see the chapter's note.)
r.startAt(Pose2d{36_in, c.chassis.pose().y(), c.chassis.pose().heading()});
```

**Why it is written this way.**

- **Seeding changes belief, not position.** The robot does not move; the estimator's idea of
  where it is does. The compiled test asserts exactly that: the estimate jumps to 36 inches and
  the host test plant's ground truth does not move at all.
- **Keep what the wall did not measure.** A wall at x = 36 in says how far the robot is along x
  and nothing about y — so the recipe passes the current y (`c.chassis.pose().y()`). An earlier
  version of this recipe wrote `0_in` there, which is right only for a robot that happens to stand
  on y = 0 (its test passed because its robot did); the compiled test now drives along y = 12 in,
  so a recipe that overwrote y would turn it red.
- **Keep the current heading.** Position is what a wall measurement gives you. Heading belongs to
  the IMU, which does not drift the way odometry position does
  ([Chapter 3](../guide/03-knowing-where-you-are.md)) — passing `chassis.pose().heading()` says
  "correct where I am, not which way I face".
- **`startAt` is a full seed.** It also clears the estimate's drift term for BOTH axes, so the
  estimate then trusts its y as if y had been measured too. That is right after a placement you
  trust in both axes; after a wall, it overclaims. For an axis a side distance sensor measured,
  use the wall reset instead ([the next recipe](#re-seat-x-against-a-wall);
  [Chapter 10](../guide/10-the-api.md#the-wall-reset--one-axis-against-a-field-wall)):
  it moves that ONE axis and leaves the drift term alone.
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

## Re-seat x against a wall

**Use this when:** your robot has a side distance sensor (declared in its robot file, its mount
measured — [Chapter 10](../guide/10-the-api.md#the-robot-files-side-sensors)) and your auton parks it
beside a field wall: the wall can put one coordinate of the estimate back where the robot really is.

An auton as you would write it in your team's `src/team/autons.cpp` — a leg along the left wall, a
square-up, then the reset, through `then()`. (The shipped `autons.cpp` holds no wall-reset auton yet:
its autons start RELATIVE, and this one needs a FIELD start.) The auton file needs two includes for it — `shulib/competition/auton_wall_reset.hpp` (the call) and
`shulib/localization/override_walls.hpp` (the season's walls); the shipped `autons.cpp` includes neither.

```cpp
// A FIELD-start auton: placed at (-55.7, -50) facing +y, its left
// side sensor about 7 in from the left wall.
chassis::RoutineResult leftWallReset(competition::AutonContext& ctx) {
    chassis::Routine r{ctx, "left-wall"};
    r.face(-55.7_in, -38_in);      // a tank turns first
    r.driveTo(-55.7_in, -38_in);   // a leg ALONG the left wall
    r.turnTo(90_deg);              // square to the wall: a tank turns in place
    r.then([&] { return competition::resetFromWall(ctx,
                     {.wall = localization::override2026::kLeftWall,
                      .sensor = localization::WallSide::Left,
                      .maxCorrection = 3_in}).continues(); }, "left wall x");
    // ...the rest of the auton drives from the re-seated x.
    return r.result();
}
```

and its entry, with a FIELD start:

```cpp
    e.routine = &leftWallReset;
    e.start = competition::StartPose::field(-55.7_in, -50_in, 90_deg);  // a FIELD start
```

The compiled test places the robot 2.5 in nearer the wall than the start says, runs the auton through
`competition::runAuton` — the competition program's own call — on the host test plant, and the reset
puts x back within 0.1 in of where the robot really is.

**Why it is written this way.**

- **A FIELD start.** The season's walls (`localization::override2026::kLeftWall`, …) are field
  coordinates. Registered with a RELATIVE start, the same auton is refused `not a field start` — the
  compiled test checks that too — because a relative origin is wherever the robot was placed, and "the
  left wall" names nothing in it.
- **`face`, then the leg, then `turnTo`.** A tank turns before it drives, and `driveTo` arrives facing the
  way it travelled, which may not be square to the wall; the reset refuses anything more than 5° off
  (`not square`). `turnTo(90_deg)` squares it, turning in place, so x does not move. A leg ALONG the
  wall — not into it — is how a side sensor comes to face it.
- **About 7 in from the wall.** The robot's centre at x = −55.7 puts a sensor 7.5 in out from the
  centre about 7 in from the left wall (at −70.2): short enough for the beam's lower edge to clear a
  standing game piece, with room for the 3-in `maxCorrection` either way. And y = −38 puts the beam on
  the wall's clean stretch — a stretch that narrows as the robot drives
  ([Chapter 3's table](../guide/03-knowing-where-you-are.md#re-seating-one-axis-against-a-wall)).
- **`maxCorrection = 3_in`** is the auton's answer to "how far could x have drifted by here?" — run the
  auton without the reset a few times, tape the error, take the largest plus about an inch. It has no
  default on purpose ([Chapter 11](../guide/11-reading-the-diagnostics.md#the-wall-reset-wall)).
- **`.continues()`** is what `then()` should return: true when it SEATED, and true when it refused —
  unless the request says `.required = true`, when a refusal stops the chain (`ActionFailed`). Not
  required, a refused reset leaves the estimate exactly as it was and the auton goes on.

**Watch out for:**

- **The `WALL` line.** Every call prints one — `SEATED`, or `REFUSED` and the check that failed, with
  its numbers. A reset that keeps refusing is telling you where to park
  ([Chapter 11](../guide/11-reading-the-diagnostics.md#the-wall-reset-wall) says what to do for each).
- **Stopping first.** The reset never moves the robot; it reads it. A robot still coasting is refused
  `moving` — after a `driveTo` or `turnTo` step the robot has settled, but after a direct call add
  `r.brake()`.
- **On the host test plant's tester rig, it refuses `no sensor`** — that rig builds no robot file — and,
  not required, the auton goes on. Only the robot tests the reset.

---

*Back to the [index](README.md), or on to [Chapter 10 — the API, as prose](../guide/10-the-api.md)
for the full verb behind each step.*
