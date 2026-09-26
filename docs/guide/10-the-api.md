# 10 — The API, as prose

> **Covers:** the public operations on `Chassis` you write routines with — what each does, when
> to use it, what it does when things go wrong, and the gotchas. It is deliberately selective
> rather than exhaustive: construction and the advanced accessors are covered where they matter
> and skipped where they do not. The [generated API reference](../api/README.md) is the
> exhaustive one, and it cannot disagree with the code. Plus typed units, per-call options, and the escape
> hatch for advanced use.
> **Read this if:** you've done the tutorial ([Chapter 8](08-your-first-routine.md)) and are
> writing real routines.
> **Assumes:** Chapters 2–8.

> **Stability.** This API is **frozen** — the
> [roadmap's Freeze Register](../roadmap.md#freeze-register), row F6, locked 2026-08-12.
> Routines you write against it will not need rewriting: every signature and documented
> behavior in this chapter changes only with a major API-version bump plus a migration note
> ([`include/shulib/version.hpp`](../../include/shulib/version.hpp) states the policy), and
> the freeze is enforced by a compile-time signature pin
> ([`test/f6_signature_pin_test.cpp`](../../test/f6_signature_pin_test.cpp)) that fails the
> build if a frozen signature drifts. New capability arrives *additively* — new verbs, new
> options fields — never as reshapes. The freeze waited, deliberately, until a second
> independent consumer ([Chapter 9](09-the-recipe-api.md)'s recipe layer) had used the
> surface in anger. The [recipe layer](09-the-recipe-api.md) froze the same way one chunk later
> (row F10, after the cookbook became *its* second consumer), so both tiers are now stable.

The code examples in this chapter are compiled and run in
[`test/guide_examples_test.cpp`](../../test/guide_examples_test.cpp), cases `guide-10a` through
`guide-10e`. The authoritative fine print for everything here is the header itself —
[`include/shulib/chassis/chassis.hpp`](../../include/shulib/chassis/chassis.hpp) opens with a
long design commentary that is meant to be read, not skipped.

> **This chapter and the [generated API reference](../api/chassis.md) are different documents,
> on purpose.** This one is *how to think about the API* — when to reach for a verb, what it
> does when things go wrong, the gotchas, worked idioms. The reference is *exactly what exists*:
> every member, its precise signature, its documentation, extracted from the header by a tool.
> **The division is a rule, not a habit:** this chapter names verbs and their arguments
> conversationally and never restates a signature, because a signature copied by hand is a fact
> that will go stale. When you need the exact type of an argument, a return, or a default, go to
> the reference — and if the two ever disagree, the reference is right, because nobody typed it.

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

The only plain doubles left are genuinely dimensionless things (a fraction like
`strafeAuthority()`, feedforward gains). Time is typed like everything else: `{.timeout = 5_s}`,
`hold(500_ms)`. It wasn't always — the options timeout began life as a plain `timeoutSeconds`
double, a documented exception where the field name carried the unit. The freeze review (D2)
retyped it before locking the surface, precisely because `hold(500)` written by someone thinking
in milliseconds would have *compiled* — and held pose for 500 seconds of a 15-second match.
Now it doesn't compile.

## The motion verbs

All the blocking verbs share a contract worth internalizing once (the tutorial demonstrated it;
the scheduler beneath enforces it):

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
    const ExitReason turn = chassis.turnTo(bearing, {.timeout = 3_s});
    if (turn != ExitReason::Settled) { return turn; }
    return chassis.moveTo(Pose2d{x, y, bearing}, {.timeout = 8_s});
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
    {.timeout = 8_s});
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

That loop is the shape of driver control, and you do not have to write the real one yourself:
`teleop::runDriverControl` ([`teleop/teleop_loop.hpp`](../api/teleop_loop.md)) is it, finished.
Each tick it checks whether the field has disabled the robot (and if so stops the drive into the
safe state and returns), reads the controller's sticks — all four axes, and only while the
controller is actually connected — passes them through the driver's feel settings (the deadband,
the curve, the mode: arcade, tank sticks, or field-centric with its honest fallback), scales them
to the chassis's speed budgets, calls `drive(…, Frame::Body)`, lets your code paint a panel, and
waits for the next tick. The things it cannot know — how long a tick is, whether the robot is
disabled, what the heading is and whether to trust it — are arguments you hand it, which is also
what lets the test suite run the whole loop on the host test plant. Honest status: "shulib Teleop"
(`src/teleop_program.cpp`) now drives through it, but that version of the program has not run on a
robot yet — so the loop is host-tested only, and no driver has felt it.

#### Buttons for mechanisms

A mechanism the driver works *while* driving — a claw, an arm, an intake — is a
`teleop::MechanismBinding` ([`teleop/mechanism_binding.hpp`](../api/mechanism_binding.md)) handed
to the same loop: a forward button and a reverse button over one motor mechanism, at a voltage
you state (there is no default). While one button is held the mechanism is driven at that voltage
every tick; the tick neither is held — or both are — it goes to its declared safe state once, and
then nothing is commanded until a button is pressed again. Letting go *is* the stop; the sticks
never interrupt a mechanism, because the driver's thumbs are on them the whole time. The loop makes
every binding safe before its first tick and again on a field disable (after the drive's own
stop), and reads the buttons only while the controller is connected. Two rules keep it honest:
a stop made through the binding while a finger is still on the button (`makeSafe()`) is not undone
by that finger — the binding waits until it has seen both buttons up — and a mechanism an operation
has claimed is never touched, even when that operation is one the end of autonomous cut off (its
claim outlives it, so release it before driver control). `teleop::validBindings()` checks a whole
set before driver control starts: one button, one job, one binding per mechanism, and no binding on
the loop's re-zero button — which defaults to **L1**, so name another one if a mechanism wants L1.
With a `robot::Robot`, the mechanisms come from the config: each `config::MechanismConfig` with
motors states its own cartridge and safe brake mode (neither has a default),
`Robot::mechanism("…")` finds it by name, and one whose motor is unplugged is reported absent
rather than stopping the robot. The [cookbook](../cookbook/06-driver-control.md) has the recipe.
Honest status: host-tested only; no driver has used a binding, and whether Hold keeps a loaded
mechanism up is unmeasured.

A pneumatic claw is the other shape: **one** button that opens it on one press and closes it on
the next. That is a `teleop::ToggleBinding` (same header) over one `hal::PneumaticMechanism`,
handed to the loop in a second list after the hold bindings. Each press flips the level the valve
was last told, once — holding the button does nothing more. A press has to be *seen*: a button that
is already down when driver control starts, or when a dropped controller comes back, is not a
press, so a press made while the controller was gone is not replayed — provided the controller's
"connected" signal and its buttons come back together, which is not measured yet. You state the
claw's starting level (there is no default — which level is "closed" depends on the plumbing), and
the binding sends it once, the first time driver control starts. And on a field disable the loop
makes every hold binding safe but leaves the claw **exactly where it is**, so it never drops what it
holds; a controller drop does not touch it either. `validBindings(holds, toggles, …)` checks both
lists as one set. From a config, a mechanism with 3-wire lines (the brain's ports A–H, stored as
1–8) states a `pneumaticStart` and a `pneumaticSafe` level, and `Robot::pneumaticMechanism("…")`
finds it — making the line sets it to the start level on the spot. The name and index lookups are
typed, so `mechanismAt(i)` is null for a built claw; to ask "is the `i`-th mechanism built?" of
either kind, use `Robot::builtMechanismAt(i)` (and `mechanismAbsence(i)` for why not). Honest
status: host-tested only; no solenoid has fired, and whether the brain holds a 3-wire output's
level while it is disabled — or acts on a level set while it is disabled, which is when a robot
built in `initialize()` on a field sets the claw's start level — is unknown (the register's
HA-157).

### `brake(options)` and `hold(duration, options)` — stopping, actively

`brake()` commands a stop and blocks until the estimate *certifies* the robot at rest (settling
logic on speed instead of position). Use before actions that need a genuinely still robot.
`hold(duration)` actively holds the current pose for a duration — `hold(500_ms)` — driving back
anything that shoves the robot: the "someone will bump me while I score" verb, holonomic
authority as a parking brake. These two began as *candidate* verbs; the freeze review adopted
them, because every complete routine written against the API used both — an auton surface that
cannot park would have sent everyday code to the advanced seam.

### `wait(duration)` — do nothing, on purpose

`wait(2_s)` returns after two seconds of robot time, commanding nothing. The world keeps
advancing (sensors, health checks, any active motion keep ticking), the drive keeps whatever
state the last verb left it in — after a settled motion, stopped — and nothing is logged.
This is the "sit still while your alliance partner clears the lane" beat that every real
routine has. Deliberately distinct from `hold()`: `wait()` never energizes the drive, so it
cannot fight a defender — it just lets time pass. And unlike `waitUntil`, there is no result
to check: a wait has no failure mode, so it returns nothing. The duration must be finite and
greater than zero.

## Options: per-call knobs

Every **blocking** verb takes a `MotionOptions` struct: `timeout` (typed time — `5_s`,
`500_ms`), `maxLinearSpeed` (in/s), `maxAngularSpeed` (rad/s). The two non-blocking verbs do
not — `drive(speeds, frame)` commands velocities directly and returns immediately, and
`wait(duration)` has nothing to bound but the duration you already gave it. Unset fields (0) mean "use the
chassis-wide config." They affect
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

**`waitUntil(predicate, timeout)`** — block until a condition of your choosing becomes
true, or the timeout passes; returns which one happened (`Satisfied` / `TimedOut`, and you must
look at the answer — the compiler warns if you discard it). The active motion, if any, keeps
running while you wait. Crucially, a timed-out wait raises **no fault** — waiting for something
that didn't happen is a *strategy branch*, not an emergency (`guide-10e`):

```cpp
// Wait up to 0.5 s for a condition that never comes true (say, a game
// piece a sensor never sees). The result is a value you must look at —
// and no fault is raised: a timed-out wait is a strategy branch, not an
// emergency.
const WaitResult seen = c.chassis.waitUntil([] { return false; }, 0.5_s);
CHECK(seen == WaitResult::TimedOut);
CHECK_FALSE(c.rig.latch.hasFault());
```

The timeout is required and must be finite — an unbounded wait is a hang wearing a costume, and
this library doesn't sell costumes.

(Every member named in this chapter, and several that are not — the reference is complete and
this chapter is selective — is listed with its exact signature in
[the generated reference](../api/chassis.md).)

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

*Next: [Chapter 11 — Reading the diagnostics](11-reading-the-diagnostics.md). For exact
signatures: [the generated API reference](../api/README.md). For "how do I write the routine I
am writing right now": [the cookbook](../cookbook/README.md).*
