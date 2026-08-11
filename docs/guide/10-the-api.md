# 10 — The API, as prose

> **Covers:** every public operation on `Chassis` — what it does, when to use it, what it does
> when things go wrong, and the gotchas. Plus typed units, per-call options, and the escape
> hatch for advanced use.
> **Read this if:** you've done the tutorial ([Chapter 8](08-your-first-routine.md)) and are
> writing real routines.
> **Assumes:** Chapters 2–8.

> **⚠ Stability notice.** This API is built and heavily tested, but **deliberately not frozen
> yet**. The project's rule is that an interface freezes only after a second, independent
> consumer has used it (the upcoming recipe API — chunk D1 — is that consumer; the freeze
> happens at D2). Until then, signatures may still change, and this chapter is written to be
> easy to revise. The current status lives in the
> [roadmap's Freeze Register](../roadmap.md#freeze-register), row F6. The *concepts* here are
> stable; hold the exact spellings loosely.

The code examples in this chapter are compiled and run in
[`test/guide_examples_test.cpp`](../../test/guide_examples_test.cpp), cases `guide-10a` through
`guide-10e`. The authoritative fine print for everything here is the header itself —
[`include/shulib/chassis/chassis.hpp`](../../include/shulib/chassis/chassis.hpp) opens with a
long design commentary that is meant to be read, not skipped.

## Typed units — why the API won't take a plain number

Every physical quantity in the API has a type: `Length`, `Velocity`, `Angle`, `Time`. You write
them with literal suffixes — `24_in`, `1_tile` (= 24 inches), `90_deg`, `2_s` — after a
`using namespace shulib::units::literals;`. Passing a bare `double` where a length or angle
belongs **does not compile**.

Why so strict? Because the alternative is the most expensive bug class in robotics: a number
that means the wrong thing. Degrees fed into math that expects radians (off by 57×, robot spins
wildly); a millisecond timeout read as seconds; an x swapped with a y. None of these are typos a
reviewer reliably catches — all of them are type errors a compiler catches instantly. Converting
"runtime disaster at the competition" into "red squiggle while typing" is the entire point, and
it costs you five characters per number. (The library also converts *sensor* quirks — the IMU's
compass-style angles, milliseconds from clocks — to canonical units at exactly one boundary, so
no conversion ever happens twice or never. That discipline is
[locked as F3 in the Freeze Register](../roadmap.md#freeze-register).)

The one place you'll see plain doubles: genuinely dimensionless things, and `timeoutSeconds`
in the options struct (a documented, named exception — the field name carries the unit).

## The motion verbs

All five verbs share a contract worth internalizing once (the tutorial demonstrated it; the
scheduler beneath enforces it):

- **They block** until the motion ends, and **cannot hang** — the watchdog bounds every path,
  including the wait-for-sensors boot window.
- **They return an honest `ExitReason`**: `Settled`, `TimedOut`, or `Cancelled`. You may ignore
  the return (that's deliberate — a fire-and-forget auton style is legitimate, and the
  diagnostics record everything anyway), but you may never be lied to.
- **Targets are field-frame** ([Chapter 2](02-the-field-and-coordinates.md)), in inches and
  field headings.
- **One motion at a time, structurally.** Starting a new motion while one is active (possible
  via the advanced seam) safely cancels the old one first. There is never a tick where two
  motions fight over the motors.
- **A faulting estimate aborts the motion.** If the fault system decides the pose estimate is
  lying (`ODO_STUCK`, [Chapter 6](06-how-things-fail.md)), the active motion is cancelled into
  a safe stop and the verb returns `Cancelled`; `lastCompleted().abortFault` names the cause.
  The *run* continues — what happens next is your routine's decision.

### `moveTo(target, options)` — drive to a pose

The workhorse: translate to (x, y) *and* rotate to the target heading, simultaneously and
independently. Use it for essentially every "go there" in a routine.

Fine print: on drivetrains with limited sideways ability, `moveTo` degrades exactly as
[Chapter 4](04-drivetrains.md) described — the H-drive runs sideways-heavy legs slower (with the
`SFB` flag in the diagnostics), and a tank drive simply cannot reach a target that requires
net sideways motion, so it honestly times out. The tank idiom is `guide-10b`:

```cpp
// On a tank drive, a sideways target is physically unreachable — the library
// says TimedOut rather than pretending. The idiom: turn to the bearing of the
// target, then drive to it WITH that bearing as the target heading, so the
// approach is a straight line the drivetrain can actually follow.
ExitReason tankGoTo(Chassis& chassis, shulib::units::Length x, shulib::units::Length y) {
    const Pose2d here = chassis.pose();
    const Angle bearing = Angle::radians(
        std::atan2((y - here.y()).value(), (x - here.x()).value()));
    const ExitReason turn = chassis.turnTo(bearing, {.timeoutSeconds = 3.0});
    if (turn != ExitReason::Settled) { return turn; }
    return chassis.moveTo(Pose2d{x, y, bearing}, {.timeoutSeconds = 8.0});
}
```

### `strafeTo(x, y, options)` — translate, guarding the heading

Go to a field (x, y) while *actively holding* whatever heading the robot has when the motion
starts — disturbances get driven back, not just ignored. Use it when the aim matters more than
the path: sliding along a wall of goals while pointed at them. Gotcha: it holds the heading it
*finds*, so settle your heading (with `moveTo`/`turnTo`) before strafing, not after. On tank:
honest `TimedOut`, as above.

### `turnTo(heading, options)` — rotate in place

Face a field heading, always by the shortest rotation (an exact 180° tie resolves
counterclockwise, deterministically). Note it's "face 135°," never "turn by 90°" — absolute
targets don't accumulate error the way relative ones do. If you want "turn by," compute it:
`chassis.pose().heading()` plus your delta.

### `followTrajectory(waypoints, options)` — a chain of moveTos

Drives through a list of poses, settling at each, and **stops at the first leg that doesn't
settle** — a robot that timed out mid-chain is *lost*, and blindly chasing waypoint 5 from
nowhere-in-particular compounds the loss. It returns a `TrajectoryResult`, not a bare
`ExitReason`, because "how far did it get" is exactly what your recovery logic needs
(`guide-10c`):

```cpp
const TrajectoryResult ok = c.chassis.followTrajectory(
    {Pose2d{12_in, 0_in, 0_deg}, Pose2d{24_in, 12_in, 45_deg},
     Pose2d{24_in, 24_in, 90_deg}},
    {.timeoutSeconds = 8.0});
CHECK(ok.succeeded());
CHECK(ok.completedLegs == 3);
```

Gotchas: options (including the timeout) apply **per leg**, not to the whole chain. The robot
*stops at every waypoint* — this is the documented v1 motion model (smooth, non-stop blending
through waypoints is future work; see [Chapter 14](14-what-it-cannot-do-yet.md)) — so a
five-waypoint trajectory pays five settle times. For now, fewer+farther waypoints beat
many+close ones. Waypoints are validated up front: one bad (non-finite) pose anywhere rejects
the whole call *before the robot moves an inch*, rather than driving three legs and then
throwing.

### `drive(speeds, frame)` — the manual verb

Direct velocity control: "move with this vx, vy, and rotation rate, *now*." No target, no
settling, no blocking — call it every iteration of a loop, the way a driver-control loop feeds
joystick values. It's also the auton escape hatch for "just push forward for a beat" moments.
The `frame` parameter is **mandatory** — `Frame::Field` or `Frame::Body` — and this is
[Chapter 2](02-the-field-and-coordinates.md)'s whole lesson cast in a signature: there is no
default to silently assume wrong. From `guide-10d`:

```cpp
// A teleop-shaped loop: command, then let the world advance one tick.
// Frame::Body: "+x" means the robot's OWN forward, wherever it faces.
for (int i = 0; i < 100; ++i) {
    c.chassis.drive(ChassisSpeeds{Velocity{20.0}, Velocity{0.0},
                                  AngularVelocity{0.0}},
                    Frame::Body);
    c.pacer.pace();
}
```

Fine print: `drive()` pre-empts (safely cancels) any active motion — a manual command
supersedes. A `Frame::Field` command during sensor boot commands *zero* and warns once — a
field-relative command needs a heading, and the boot estimate doesn't have one yet
(`Frame::Body` works fine during boot). Each call runs exactly one loop iteration — sensor
update, command, diagnostics record — so your loop's own timing sets the cadence.

### `brake(options)` and `hold(seconds, options)` — stopping, actively

`brake()` commands a stop and blocks until the estimate *certifies* the robot at rest (settling
logic on speed instead of position). Use before actions that need a genuinely still robot.
`hold(seconds)` actively holds the current pose for a duration, driving back anything that
shoves the robot — the "someone will bump me while I score" verb, holonomic authority as a
parking brake. These two are marked *candidate* verbs — most likely of anything here to be
reshaped at the freeze review.

## Options: per-call knobs

Every verb takes a `MotionOptions` struct: `timeoutSeconds`, `maxLinearSpeed` (in/s),
`maxAngularSpeed` (rad/s). Unset fields (0) mean "use the chassis-wide config." They affect
**one call** (`guide-10a` pins that the chassis config is untouched afterwards). Nonsense values
— NaN, negatives — are rejected loudly at the call, before anything moves; so are non-finite
targets. The library's philosophy is that misuse fails *at the door*, never as a mystery
mid-run. (Why a struct instead of extra parameters? So future knobs can be added without
breaking every call site — an intentional piece of freeze-proofing.)

The chassis-wide defaults live in `ChassisConfig` / `MotionConfig`
([`include/shulib/motion/motion_config.hpp`](../../include/shulib/motion/motion_config.hpp)) —
gains, speed budgets, settle tolerances, the default timeout. Read that header before touching
any of it, and remember: **every number in it is a provisional stand-in** until a real robot is
measured ([Chapter 14](14-what-it-cannot-do-yet.md)).

## Control and state

**`cancel()`** — stop the active motion *right now*, into the defined safe state: zero volts,
brake mode, synchronously. With no motion active it's the panic stop and still safes the
drivetrain. This is what you wire to an emergency condition.

**`waitUntil(predicate, timeoutSeconds)`** — block until a condition of your choosing becomes
true, or the timeout passes; returns which one happened (`Satisfied` / `TimedOut`, and you must
look at the answer — the compiler warns if you discard it). The active motion, if any, keeps
running while you wait. Crucially, a timed-out wait raises **no fault** — waiting for something
that didn't happen is a *strategy branch*, not an emergency (`guide-10e`):

```cpp
// Wait up to 0.5 s for a condition that never comes true (say, a game
// piece a sensor never sees). The result is a value you must look at —
// and no fault is raised: a timed-out wait is a strategy branch, not an
// emergency.
const WaitResult seen = c.chassis.waitUntil([] { return false; }, 0.5);
CHECK(seen == WaitResult::TimedOut);
CHECK_FALSE(c.rig.latch.hasFault());
```

The timeout is required and must be finite — an unbounded wait is a hang wearing a costume, and
this library doesn't sell costumes.

**Reading state:** `pose()` — the current estimate (an estimate! [Chapter 3](03-knowing-where-you-are.md));
`setPose(p)` — seed/teleport the estimated *position* (heading stays IMU-owned);
`strafeAuthority()` — the drivetrain's sideways-speed fraction ([Chapter 4](04-drivetrains.md)),
for budgeting lateral legs; `lastExitReason()` and `lastCompleted()` — how the previous motion
ended, the latter with full detail (name, id, timing, and the `abortFault` that names a
fault-policy cancel).

## The advanced seam — when the verbs aren't enough

`chassis.scheduler()` and `chassis.deps()` expose the machinery the verbs are built from: the
motion scheduler (for non-blocking `async()` composition) and the dependency bundle (for writing
your own motion types). This is "Tier 3" in the project's
[accessibility model](../shulib-v2-master-plan.md#17-accessibility--progressive-disclosure-for-teams-that-cant-code-yet)
— the no-ceiling tier — and it's how [Chapter 13](13-extending-the-library.md)'s extensions plug
in. Two things to know even if you never use it: it's the *same* one-motion-at-a-time slot the
verbs use (an async motion and a verb pre-empt each other — that's a feature), and motions built
from raw materials rather than `chassis.deps()` lose their id stamp in the diagnostics (records
show `cmd#0`). Until you need it, you don't need it.

## Gotchas, collected

- **The first motion waits for sensors.** Budget its timeout for ~2 s of IMU calibration.
- **`strafeTo` holds the heading it finds** — aim first, then strafe.
- **Trajectory options are per leg**, and every waypoint costs a settle.
- **Tank + sideways target = honest timeout.** Use the turn-then-drive idiom.
- **`pose()` is an estimate.** Grading your routine by `pose()` alone proves the robot agrees
  with itself, not that it's right. (In simulation, tests grade against the sim's ground truth
  — which the estimator provably cannot see.)
- **Exit reasons are advice you're free to ignore, once.** A routine that ignores a `TimedOut`
  and fires the next leg *from the wrong place* is how one failure becomes five. Check the ones
  that matter.
- **Don't re-tune config numbers to fix a logic problem.** Every default is a documented
  placeholder; if a motion misbehaves in simulation, the cause is almost never the gains
  ([Chapter 12](12-when-things-go-wrong.md) first).

---

*Next: [Chapter 11 — Reading the diagnostics](11-reading-the-diagnostics.md)*
