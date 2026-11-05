# 10 — The API, as prose

> **Covers:** the public operations on `Chassis` you write routines with — what each does, when
> to use it, what it does when things go wrong, and the gotchas — and, since the mechanism core's
> first part, how a robot's mechanisms are declared and wired, since the camera's first part, how the AI Vision
> camera is declared and read, and since the wall reset's library core (L6-A1), how one coordinate is re-seated
> against a field wall from a side distance sensor (the last sections before the gotchas). It is deliberately selective
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
> Three parts of this chapter are outside this banner, and each says so: the driver loop (under
> `drive()`), the wall reset and the mechanisms (the auton steps among them).

The code examples in this chapter are compiled and run in
[`test/guide_examples_test.cpp`](../../test/guide_examples_test.cpp), cases `guide-10a` through
`guide-10e`, the mechanism section's in
[`test/mechanism_guide_example_test.cpp`](../../test/mechanism_guide_example_test.cpp), cases
`guide-10f` through `guide-10h`, and the wall reset's in
[`test/wall_reset_example_test.cpp`](../../test/wall_reset_example_test.cpp), case `guide-10w`.
The authoritative fine print for everything here is the header itself —
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
`SFB` flag in the diagnostics), and a tank drive cannot move sideways at all, so a target it
cannot reach by driving along its heading — a point beside it, or one farther off its line than
the steering below removes over the leg — honestly times out. What a tank `moveTo` does do
(since API 2.3) is stay on its *line* — the line through the target point along the target
heading: a leg that starts within 15° of its target heading bends its heading back toward that
line when a weaker side pulls it off (by at most 15°, visible as `TSTR` in the diagnostics),
which is what lets a leg you aimed finish on a robot whose sides are not perfectly matched. It never turns you toward
the target point. So the idiom is unchanged — aim first, then drive along the aim — and it is
`guide-10b`:

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
honest `TimedOut`, as above — `strafeTo` never steers the way a tank `moveTo` does, because
holding the heading it found *is* its promise, and steering means leaving it. On a tank, the
translate that steers is a `moveTo` with the heading you already have: it bends toward the line
through its target by at most 15°. On the host test plant that removes a target a few inches off
your heading over a 40 in leg — more over a longer leg, about an inch over a 20 in one; on a robot
it depends on motor lag nobody has measured yet, and nothing here has run on one — and never a
point beside you: face that one.

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
what lets the test suite run the whole loop on the host test plant. Honest status: robot two's
driver file (`src/team/robot_two_driver.hpp` — the [cookbook](../cookbook/06-driver-control.md)
quotes it whole) drives through it — the sticks, the feel, and robot two's claws on the buttons
below, all stated in that one file — and two programs run that file: "shulib Drive"
(`src/programs/drive_program.cpp`) on its own, and the competition program's driver control
([Chapter 17](17-the-competition-program.md)). Neither has run on a robot yet, so the loop is
host-tested only, and no driver has felt it.

The banner at the top of this chapter does not cover the loop: `teleop::runDriverControl`, like the
rest of `teleop/`, is in no row of the Freeze Register, so it is not part of the frozen surface. It
changes without a major version bump — and with a changelog entry every time, as it did when the
drive protections below arrived (the changelog's "the drive protections, inside the driver loop"):
a program built against an earlier version keeps compiling unchanged, and now cuts and refuses.

#### The drive protections — in the loop, for every program

The loop also protects the drive, with no wiring of your own (`teleop::DriveProtection`,
[`teleop/drive_protection.hpp`](../api/drive_protection.md); its numbers are
`DriverLoopConfig::protection`, and a program that leaves the field out gets the defaults below).
They are the old "shulib Drive"'s three rules, moved into the library so every program that drives
through `runDriverControl` has them:

- **A drive motor that stops answering is dropped.** Through the motor seam a dead port does not
  say so — its adapter keeps repeating its last good position, velocity and current. So the loop
  watches for exactly that: a member whose readings repeat bit for bit for 5 reads while it, or a
  side-mate whose encoder is still moving, says the train is turning, with at least as many of its
  side answering as not. That member is FROZEN — kept out of the fight check at once — and after
  250 ms of it, marked ABSENT for the rest of the run: out of every reading, still commanded, and
  said in one `[WARN][TEL]` line
  (`WARNING: port 18 (LEFT) went ABSENT at runtime: reads frozen for 250 ms
  while its side answered …`). The robot drives on, degraded. At a wall beside a slipping drive
  gear it can mark a motor that is NOT dead — [Chapter 12](12-when-things-go-wrong.md)'s "A motor
  reported dead that is not".
- **More than two dead drive motors, in total, stops the drive for the rest of the run** — 0 V, one
  `[ERROR][TEL]` line (`REFUSED: more than 2 motors dead in total -- 0 V for the rest of the run,
  every later start of driver control included …`), and every later start of driver control is
  refused too until the program is started again (the marks live in the robot's motor groups). One
  or two dead, in any arrangement, drive on with a warning — the team lead's rule ("1-2 dead ports
  shouldn't stop driving"). At boot the stricter rule still applies (a side with fewer than three
  answering is refused before the match starts).
- **A fight, or too much current, cuts the drive for 1 s.** A drive motor running against its side
  (opposite to the command, or under a quarter of its fastest mate, while the side is driven above
  1 V and turning), or one drawing more than 2.4 A while its side is driven above 1 V, for 250 ms,
  sends BOTH sides to 0 V for 1 s — in that same tick — with one cut line and an `at the cut:` line
  per motor; then driving resumes on its own (`re-armed after cut #1 (1 s at 0 V)`).
  `protection.overCurrentCuts = false` turns the over-current cut into a logged, counted warning
  (a robot that must push); the fight cut has no switch.

After each driven stretch the loop logs one line if that stretch's highest motor current is a new
high for this start of driver control (`drive: LEFT peak 2.31 A (port 17) over a 3.1 s drive, no
cut`) — the record a push test reads. A `perTick` that takes a fourth argument,
`const teleop::DriveProtectionState&`, is handed what the protections are doing (driving, degraded,
cut with its ticks left, or refused; the dead ports; the cut count) for a panel. **None of it has run
on a robot**: every number is invented (register entries HA-132, HA-175, HA-176), and it is proven
on the host test plant and on hand-made readings only.

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
has claimed *during* driver control is never touched. A claim still there when driver control
*starts* is different: an operation the end of autonomous cut off never releases its own claim (PROS
deletes that task wherever it is), so the loop releases it at its start, before anything is made
safe — without calling the dead operation — and says so, one `[WARN][TEL]` line per claim, with
`releasedLeftoverClaim()` true on the binding for your panel. `teleop::validBindings()` checks a whole
set before driver control starts: one button, one job, one binding per mechanism, and no binding on
the loop's re-zero button — which defaults to **L1**, so name another one if a mechanism wants L1.
With a `robot::Robot`, the mechanisms come from the config: each `config::MechanismConfig` with
motors states its own cartridge and safe brake mode (neither has a default),
`Robot::mechanism("…")` finds it by name, and one whose motor is unplugged is reported absent
rather than stopping the robot. (Since the mechanism core's first part the Robot also builds each
mechanism's SENSOR roles — [the section below](#mechanisms--declared-once-wired-separately).) The
[cookbook](../cookbook/06-driver-control.md) has the recipe.
Honest status: host-tested only; no driver has used a binding, and whether Hold keeps a loaded
mechanism up is unmeasured. **Superseded, not removed (since the mechanism core's fourth part):** a
robot whose mechanisms are DECLARED ([below](#mechanisms--declared-once-wired-separately)) runs them
from its driver file's controls — `whileHeld`, `toggle`, `preset`, the homing controls — through
`runDriverControl`'s mechanism-service overload, the same service its autons use
([chapter 18](18-mechanisms.md#on-the-drivers-buttons--the-driver-files-controls)); these bindings
stay, unchanged, for a program with no service, and the loop refuses the two paths together.

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
gains, speed budgets, settle tolerances, the default timeout, and (since API 2.3) the tank
steering's `tankSteer` knobs, whose `maxSteer = 0` turns it off. Read that header before touching
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

## The wall reset — one axis against a field wall

> **The banner at the top of this chapter does not cover this section.** The wall reset is in no
> row of the [Freeze Register](../roadmap.md#freeze-register): `chassis::resetFromWall`, the request,
> mount, limits and report types, the season's walls, and the Localizer's `seatAxis` and two counters
> are not frozen and not signature-pinned, and they grow additively as the rest of the wall reset
> lands (the everyday form and the robot file's side sensors).

*Host- and build-gated, NOT run on a robot.* When the robot has **parked beside a field wall, nose
along it and square to it**, the side distance sensor facing that wall measures how far the wall is;
from where that sensor sits on the robot, that is **the robot's distance from the wall — ONE
coordinate**: x against the left and right walls, y against the red and blue walls. The wall reset
replaces that one coordinate of the estimate and nothing else — not the other coordinate, not the
heading. It is a step you call, at a moment you chose; there is no background mode.

Two forms. The **explicit** call, `chassis::resetFromWall(chassis, sensor, request, limits)`
(`shulib/chassis/wall_reset_call.hpp`), takes the sensor handles you build — the form for a
hand-wired Tier-3 robot and for the host tests. The everyday form an auton writes,
`competition::resetFromWall(ctx, request)`, which finds the robot file's side sensors through the
auton's context and refuses any start that is not a FIELD start, is **not in the library yet** (it
lands with the robot file's side sensors). The explicit call (`guide-10w`):

```cpp
// A hand-wired robot's left side sensor, as the wall reset reads it: the
// sensor, its read-fault count, and the mount a student MEASURED.
localization::SideMount mount{};
mount.measured = true;
mount.left = 7.5_in;     // out from the robot's centre
mount.height = 10.5_in;  // the lens above the floor
mount.facing = units::AngleDim{90.0 * localization::kWallDegToRad};  // straight out the left
mount.provenance = "tape, 2026-10-29";
const localization::WallSensor left{&sensor, &faults, mount, "not declared"};

// Re-seat x against the left wall: the beam square to it, on a clean
// stretch of it, and x may move by at most 3 inches.
const localization::WallResetReport report = chassis::resetFromWall(
    c.chassis, left,
    {.wall = localization::override2026::kLeftWall,
     .sensor = localization::WallSide::Left,
     .maxCorrection = 3_in},
    localization::WallResetLimits{});
CHECK(report.seated());
```

**The request** (`localization::WallResetRequest`): the `wall` — one of the season's four,
`localization::override2026::kLeftWall` / `kRightWall` / `kRedWall` / `kBlueWall`
(`shulib/localization/override_walls.hpp`: inner faces at ±70.205 in, each carrying the stretches a
beam must not land on — the Toggle, its brackets, the Loaders, the corners, the VEX U Load Zones and,
until the beam's spread is measured, the start pieces), which are FIELD coordinates and mean nothing
under a relative start; or `FieldWall::line(name, axis, coord, inward, keepOuts)`, your own line in your
own frame with your own keep-outs (none by default — then nothing tells a Loader from the wall). The
`sensor` side. **`maxCorrection` — how far this wall may move the coordinate — has no default:** 0 is
UNSET and refused by name (`maxCorrection UNSET: say how far this wall may move x`); choose it as the
largest drift you have taped at this point plus ~1 in ([Chapter 11](11-reading-the-diagnostics.md#the-wall-reset-wall)).
`required` (false by default): a refusal leaves the estimate exactly as it was and the auton goes on —
unless the request says `required = true`, when `continues()` is false and a `then()` returning it
stops the chain `ActionFailed`. `name`, when set, leads the log line.

**The mount** (`localization::SideMount`) is MEASURED, never guessed: `forward`, `left` (from the
robot's centre — the turning centre on a tank), `height` (the lens above the floor), `facing` (CCW
from forward: `90.0 * kWallDegToRad` is straight out the left side — written that way because the
`_deg` literal is not `constexpr`), each with a tolerance, and a `provenance` string. `measured =
false` — the default — makes every reset with that sensor refuse `MOUNT UNSET`. Four optional
measured fields stay 0 / false until their measurement exists: `rangeBias`, `rangeAccuracy`,
`coneHalfAngle` (a measured beam spread also lifts the start-piece keep-outs) and `toggleChecked`
(lifts the Toggle BODY's keep-out for that sensor — never its brackets). The sensor's place matters:
mounted at the robot's fore-aft centre, a heading error barely moves the answer; 5 in forward of it —
with the default ±0.25-in mount tolerance and VEX's published accuracy (no measured `rangeAccuracy`)
— the precision check refuses from the first reset (a mount measured to ±0.1 in and a measured
accuracy of ±0.3 in would bring it back under the limit).

**The limits** (`localization::WallResetLimits`) are the sensor-physics and geometry gates — library
defaults that are PROVISIONAL (the [hardware-assumptions register](../hardware-assumptions.md),
HA-211): a reading inside 2–16 in, the beam within 5° of square, at least 5 samples over at least
0.15 s (at most 0.5 s; `window` may not exceed 0.6 s — the 64-sample buffer could not span it)
spreading no more than 0.5 in, confidence at least 0.5, the robot still (≤ 1 in/s — measured as the
estimate's displacement over the last `window`, so a still robot's IMU jitter, which shakes the
estimate a few thousandths of an inch a tick, is not read as motion — and ≤ 3°/s on every tick), the
answer's worst-case error at most 1.0 in, and the beam-height ceiling: the reading short enough that
the beam's lower edge stays above a standing game piece (6.5 in) — about 10.8 in of reading for a
lens 10.5 in up, 3 in for one 8 in up.

**The report** (`localization::WallResetReport`) is a value, never an exception: the `verdict`, the
coordinate `before`, the wall's answer (`measured`), what was `applied` (0 unless SEATED), the window's
`median`, `spread` and `samples`, the `errorBound`, and every number its log line prints. `seated()` is
true when the coordinate moved; `continues()` is what a `then()` should return —
`r.then([&] { return chassis::resetFromWall(…).continues(); }, "left wall x")`. The verdict's word is
`localization::wallResetVerdictWord(v)`: `SEATED`, or `REFUSED` and the first check that failed —
`sensor absent`, `mount unset`, `not live`, `motion active`, `bad request`, `not square`, `keep-out`,
`beam low`, `moving`, `read fault`, `no return`, `low confidence`, `out of range`, `too few samples`,
`spread`, `imprecise`, `disagrees` (and, for the everyday form, `no sensor`, `not a field start`,
`cut`). [Chapter 11](11-reading-the-diagnostics.md#the-wall-reset-wall) says what each means and
what to do about it.

**What one call does.** It refuses, reading nothing and taking no time, when the sensor is absent,
the mount unmeasured, the estimate not live, **a motion still armed** (a Tier-3 `async()` — the
estimate is never moved under a running motion), the request unusable, the beam not square, the beam
on a keep-out, or the expected reading past the beam-height ceiling. Otherwise it samples inside
`chassis.waitUntil` — the Localizer and every health check keep ticking; the window ends itself, so
no "timed out" line — then judges the samples and, when every check passes, SEATS. ONE log line
either way (subsystem `WALL`). It **commands nothing to the drive**: stop first (`brake()`, a short
pause) — a robot still coasting is refused `MOVING`. Blocking, at most `windowMax` (0.5 s) plus a
tick — and that window is a wait the run guard cannot cut, so a deadline landing in it is acted on
up to 0.5 s late (the everyday form will sample through the guard's own wait instead).

**`Localizer::seatAxis(axis, value)`** is what SEATED calls — and why it is not `setPose`. It moves
that one axis and leaves everything else exactly as it was: the other axis, the heading, the heading
bias, the drift term (`distanceSinceCorrection()`), the quality, and the odometry. `setPose` with the
other axis copied would clear the drift term, and from the second tick after it the estimate would
read DeadReckon (quality 1.0) where one measured axis should leave it Degraded — a one-axis
measurement passing as a two-axis one. The next tick after a seat is a re-base (its dt is 0), which
an EKF-tier estimator needs so that it does not read the seat as one tick of travel. Preconditions:
the estimate is live, the value finite, and — by contract, since the Localizer cannot see the
scheduler — no motion active. Its two read-only counters feed the gates: `travelSinceSeat(axis)`
(the path length since that axis was last set — the keep-outs widen by 10 % of it along the wall,
because along the wall is the one direction the reset does not measure) and
`turnedSinceHeadingSeed()` (the rotation since the heading was set — the precision check's heading
tolerance grows by 2 % of it). Both count in STEPS — 0.5 in of the odometry's path, 1° of the IMU's
heading — so a robot standing still reads ~0 however long it stands: summed tick by tick, a still
IMU's heading jitter would add up to hundreds of degrees of "turning" a minute, and a long skills run
would be refused. A real leg or turn counts in full but for its last step. "~0" holds at the jitter
the library believes a still IMU has (the register's HA-21); an IMU more than about three times that
noisy starts counting it, which only refuses more resets (HA-211 has the numbers).

**A seat is a deliberate jump.** The scheduler's pose guard raises an advisory `IMPLAUSIBLE` when one
tick's pose change is bigger than 150 in/s × 1.5 × the tick (2.25 in at 10 ms), and it forgives a
jump only on a tick whose dt is 0 — which every verb and wait gives on its first tick. So a seat
followed by a Routine step raises nothing; **a Tier-3 loop that ticks the scheduler itself
(`scheduler().tick()`) sees one advisory `IMPLAUSIBLE` after a seat over ~2.25 in.** It is advice,
not an abort; the report says what was applied.

No sensor ever seats the heading — here or anywhere. And no reading check can tell a tall object at
a plausible distance from the wall: [Chapter 14](14-what-it-cannot-do-yet.md#the-wall-reset-what-it-cannot-catch)
says what the reset cannot catch.

## Mechanisms — declared once, wired separately

> **What exists, and what does not yet.** The mechanism core's first part (MECH-A1, 2026-10-08) is
> the VOCABULARY and the BUILD CHECKS: how a robot file says what each mechanism is and where it is
> plugged in, what the build refuses, the route ids every mechanism publishes, and the devices the
> robot builds for every role. Since its second part (MECH-A2, 2026-10-15) a mechanism SERVICE runs
> POWER and PNEUMATIC mechanisms through it, ticked once per control tick, and since its third (MECH-A3,
> 2026-10-22) the POSITION and SPEED behaviours too — an `Arm` or a `Lift` moves, holds and finds its
> zero, a `Flywheel` spins up and stays at speed ([chapter 18](18-mechanisms.md)). Since its fourth
> (MECH-A4, 2026-10-29) everything that calls the service exists: the auton STEPS
> (`r.move(arm["score"], 1.5_s)`), the driver CONTROLS, the ROUTE BINDER that runs a marker's id, and
> the direction check's judgement ([below](#in-an-auton-on-the-buttons-on-a-route)). Since its fifth
> (MECH-A5, 2026-10-29) robot two's programs run on it: each binds one service over robot two's
> declared claws, prints the per-role boot lines below, and publishes the claws' route ids; robot two's
> driver file states its buttons as the core's controls. **Nothing here has run on a robot.**
>
> **The banner at the top of this chapter does not cover this section**, as it does not cover the
> driver loop: the mechanism words, the service, the driver controls, the route binder and the
> direction check are in no row of the [Freeze Register](../roadmap.md#freeze-register) (the
> mechanism seam beneath them is row F11's, NOT frozen), and whether the auton steps, the second
> `Routine` constructor and `lastMechanism()` join the recipe layer's freeze (row F10) is not yet
> ruled — they are not pinned meanwhile. The two pieces the route binder appended to the FROZEN
> command vocabulary (`CommandEntry::doc` and the four-argument `CommandRegistry::on`) are pinned
> with it (row F8).

A mechanism is written ONCE, as **what it is** — its kind, its motors and which way each is mounted,
its cartridge, its gearing as tooth counts, what it does when stopped and what a jam looks like, and
the numbers only you know. There is no port in it. The five kinds are `Arm` (an angle in degrees,
held), `Lift` (a height in inches, held — a spool, a linkage or a measured table), `Flywheel` (an
output rpm, "at speed"), `Power` (run at a voltage: an intake, a roller, a claw pivot) and
`Pneumatic` (two or three named positions, each the line levels you state). One
`using namespace shulib::mechanism::words;` gives every word (`guide-10f`):

```cpp
inline constexpr Arm arm{
    .name = "arm",
    .motors = {Forward},
    .cartridge = Green,
    .gearing = gears(12, 60),         // a 12-tooth gear on the motor drives a 60 on the arm
    .safe = Hold,
    .stall = {.amps = 2.0, .belowMotorRpm = 10, .lasting = 0.5_s},
    .range = {0, 160},                // degrees: down on its stop .. just past vertical
    .gravity = cosine(1.6_volt, 45),  // 1.6 V holds it out level; it is level at 45 degrees
    .home = restsAtStop(0),           // it rests on its stop at 0 when the program starts
    .presets = {{"stow", 0}, {"score", 95}, {"high", 140}},
};
```

**The members go in ONE order per kind — C++ requires it** (a designated initializer out of order is
a plain C++ error): what it IS (name, motors, cartridge, gearing, a lift's shape, a sensor), what it
does STOPPED (`.safe`, `.stall`), where it GOES (`.range`, `.gravity`, `.home`, `.presets`), then the
extras (`.unhomedVolts`, `.tuning`). The [generated reference](../api/kinds.md) lists each kind's
members in that order. The rules that matter most:

- **The gearing is tooth counts, the motor's gear first:** `gears(12, 60)` is five motor turns per
  output turn; stages multiply (`gears(12, 36) * gears(12, 60)`); `direct` is none. A bare number
  does not compile — the drive's own geometry states its ratio the other way round, and a bare `5`
  or `1.0/5` would compile either way and be wrong by 25 times.
- **An arm's angle is a plain number** (`arm.to(90)`); `90_deg` is a WRAPPING heading (270 reads
  −90) and does not compile there. A lift's height is inches (`lift.to(18_in)`).
- **A jam signature is REQUIRED on an `Arm`, a `Lift` and a `Flywheel`** — they keep holding or
  spinning after their step, and must never sit at stall current — and every signature you write must
  be able to trip: `.amps` below the motor's 2.5 A current limit (a limited motor never reads more),
  `.belowMotorRpm` above 0 (a stalled motor still reads a little creep) and `.lasting` above 0 (a motor
  starting from rest draws stall-grade current with its shaft still). Optional on a `Power`.
- **Every number you own is yours to state.** A field left out is UNSET and refused at build, by
  name. The only numbers the library supplies PER MECHANISM are its STARTING gains, smooth-move speed
  and "reached" tolerance — derived from your cartridge and gearing, provisional until tuned, and
  replaced one by one with `.tuning` ([chapter 18](18-mechanisms.md#tuning--the-starting-values-and-how-to-change-them)
  lists them and says how). Three loop constants are the library's own, the same for every mechanism
  and not in `.tuning`: the hold's integral clamp and the flywheel integral's window and clamp (the same
  chapter).

**Where each one is plugged in** is a separate block, one `wire(...)` line per mechanism, in boot
order — re-cabling changes only these lines. A smart port is a NUMBER and a 3-wire port a LETTER in
quotes, so the two cannot be confused:

```cpp
inline constexpr MechanismTable kGuideMechanisms = mechanisms(
    wire(arm, motors(1)),
    wire(intake, motors(5), pieceSensor(9)),
    wire(claw, lines('A')));
```

Name a robot's list `k<Robot>Mechanisms`; `MechanismTable` is the library's one type for it. (The
intake and the claw are declared the way the arm is — a `Power` with a `distanceSensor(2_in)`, a
`Pneumatic` with `{{"closed", {Low}, grabs}, {"open", {High}}}` — and `guide-10f` compiles all three.)

### What the build refuses, and how to read a refusal

The build refuses — each by NAME — a port or a 3-wire port used twice (`smart_port_4_is_used_twice`),
a number where a letter belongs and the reverse, a motor count that is not the declaration's (and
more than four motors or two lines), a sensor the declaration has but the wiring leaves unplugged
(or the reverse), one sensor written twice in a `wire(...)` line, a pneumatic position that states a
different number of levels than its wiring has lines, a preset past a travel limit, a homing push
that points away from its home end, a voltage past the motor's 12 V (a homing push, a gravity hold,
`.unhomedVolts`), a jam signature that could never trip, absolute homing over a travel of a whole
turn of its angle sensor or more, a missing number, a misspelt preset, and every route id the list
could not publish (below). The reason is the name of a function GCC refuses
to call, so a refusal reads like this:

```text
your_robot.hpp:12:47:   in 'constexpr' expansion of 'shulib::mechanism::mechanisms<...>(...)'
.../shulib/mechanism/wiring.hpp:...:   in 'constexpr' expansion of '...refuseSmartPortTwice(...)'
.../shulib/mechanism/wiring.hpp:...: error: call to non-'constexpr' function
    'void shulib::mechanism::refused::smart_port_4_is_used_twice()'
```

**Your line is in the first "in 'constexpr' expansion of" note just above it** (for a mistake between
two `wire(...)` lines — a port twice — that is the list's first line; for a declaration's own mistake,
its `wire(...)` line). A few slips are refused ON your line (`intake["fast"]` — a power mechanism has
no presets —, `lift.to(18)`, `.gearing = 5`); a misspelt preset is not one of them: its error is in the
library, and your line is the expansion note's, as above. Three stay plain C++ errors: `arm.to(90_deg)` ("call to non-'constexpr' function
operator""_deg"), a run-time value where a constant is checked ("is not a constant expression" — a
computed target is `arm.clampedTo(x)`, clamped to the limits when it runs), and a misspelt mechanism
("was not declared in this scope"). Every refusal your robot file can reach has a test that fails to compile on purpose
(`test/compile_refusals/`).

### Route ids — the actions every mechanism publishes, automatically

Every mechanism's actions are ROUTE IDS a route marker can name, derived from its declaration —
nobody types one (`guide-10g`): an arm's or a lift's `<name>_<preset>` and `<name>_home`; a
flywheel's `<name>_<preset>` and `<name>_stop`; a power mechanism's `<name>_run`, `<name>_until_piece`
and `<name>_until_gone` (with a piece sensor) and `<name>_stop`; a pneumatic's `<name>_<position>`.
The name is lower-cased and every run of other characters becomes one `_` ("claw pivot 6" →
`claw_pivot_6`). The list above publishes `arm_stow`, `arm_score`, `arm_high`, `arm_home`,
`intake_run`, `intake_until_piece`, `intake_until_gone`, `intake_stop`, `claw_closed` and `claw_open`.
An id longer than 23 characters, two ids alike (a preset named `home` beside a homing method), an id
that is one of the library's own (`wait_ms`, `brake`, `hold_ms`, `log`) and more than 32 ids are each
refused at the list. A program registers every one of a robot's ids in ONE call
(`runner::MechanismRoutes`, [below](#in-an-auton-on-the-buttons-on-a-route)), and a marker then runs
exactly the step an auton's line runs.

### At boot — every role built, one line each

The robot builds every ROLE a mechanism declares through its device provider (the provider is the
kind check: a port holding the wrong device is refused, naming what it found), and
`mechanism::roleLine()` words one boot line per role (`guide-10h`). Robot two's programs build their
claws this way (the Robot does it for them) and print these lines at boot, tag `MECH`, since the
core's fifth part:

```text
arm: motor 1 -> port 1 ok
intake: piece sensor -> port 9 MISSING, it runs WITHOUT it: port 9 has no distance sensor
claw: line 1 -> 3-wire A not checkable (the brain cannot see a 3-wire device)
```

A missing or wrong-kind MOTOR, line or ANGLE sensor makes that mechanism ABSENT — nothing commands
it, and the robot still drives. A missing PIECE sensor leaves the mechanism running WITHOUT one. A
3-wire role is always "not checkable": the brain cannot see a switch or a solenoid. Two cables of the
SAME kind swapped between mechanisms look fine at boot — that is the direction check's to find (its
judgement, `mechanism::judgeDirection`, exists since the core's fourth part; the bench page that runs it
on the robot — Bench Tests' station 13 — since its sixth and last, built and not yet run on a robot, [chapter 18](18-mechanisms.md#the-bench-page--bench-tests-station-13)). Beside each piece sensor and home switch the provider also answers its
READ-FAULT count (`hal::IReadFaults`, through `readFaults(port)` / `lineReadFaults(line)`): a sensor
whose read fails is answered with its last good value, and a distance sensor held from a far reading
is, byte for byte, the "no object" reading — only the count tells them apart. The count is there so
the core's service never takes a held value for evidence: since its second part the service reads a
PIECE sensor's count, and since its third a HOME switch's (a read that failed is never a home) — and
since its fifth, robot two's programs build the service (robot two declares no sensor yet).

At run time `config::RobotConfig::valid()` owns every port collision — now including each
mechanism's sensor ports and switch lines — against the drivetrain the boot actually chose, so a
calibration record that drives a mechanism sensor's port is refused naming both
("port 9 is claimed by the LEFT drive and by a mechanism sensor").

**Hardware beliefs this rests on, unmeasured:** that a limit switch reads 1 when pressed and an
unplugged switch line reads released (the Hardware Assumptions Register's HA-184), and that a V5
rotation sensor starts at its shaft's absolute angle at power-up (HA-186). Both are settled at the
bench page (Bench Tests' station 13, built at the core's sixth part — not yet run on a robot).

### In an auton, on the buttons, on a route

What calls the service, each taught in [chapter 18](18-mechanisms.md) with its rules and its measured
numbers — this is the map:

- **An auton's steps** — `Routine r{ctx, "name"}` (the second `Routine` constructor, taking the auton's
  `competition::AutonContext`, whose appended `mechanismService` carries the robot's service), then one
  line per step: `move`, `home`, `run`, `set`, `spinUp`, `stop`, and the START forms `startMove`,
  `startHome`, `startRun`, `startSpinUp` with `waitUntilDone`. SUCCEEDED and an UNCONFIRMED counted done
  continue; every other verdict stops the chain as `RoutineStopCause::MechanismStepFailed`, the verdict
  in `RoutineResult::mechanismVerdict` and `lastMechanism()`. `competition::runAuton`'s mechanism-service
  overload runs such an auton — the service's `enterAutonomous()` first, so whatever a deleted driver
  task left running stops before the auton.
- **The driver's buttons** — `mechanism::controls(kRobotXMechanisms, whileHeld(...), toggle(...),
  preset(...), homeOnPress(...), homeAtDriverStart(...), autonOnly(...))`, checked at build;
  `mechanism::DriverInputs` binds them to the service at boot; `teleop::runDriverControl`'s
  mechanism-service overload runs them, the service ticked once per tick before your panel.
- **A route's markers** — `runner::MechanismRoutes` registers a robot's ids in a `CommandRegistry` in one
  call, each with a one-line doc the manifest publishes (`CommandEntry::doc`, the four-argument
  `CommandRegistry::on`); a marker blocks at its waypoint until its step's verdict, and one whose args are
  wrong is REFUSED.
- **The direction check's judgement** — `mechanism::judgeDirection` / `judgePneumatic`: a jog's
  readings and the person's three answers in, the robot-file line to change out.

## The camera — a frame at a time

The V5 AI Vision camera is declared in the robot file like any device — its port, what the library writes
to it, and where it sits — and the Robot builds it at boot through the device provider. Nothing in the
library READS it yet: the camera's consumers (a Toggle colour tracker, a piece finder that takes one named
piece, and a feed that reads once per period for both) are the camera's next part. What exists now is the
device, the frame, and the geometry that turns a box into directions and distances. **No AI Vision sensor
has been read by this code yet; everything below has run on the host test plant only.**

**The robot file's camera block** (`config::CameraConfig`, in `sensors.camera`): the port (0, the
default, means no camera); the SETTINGS the library writes at every boot — the colour descriptors the
team measures with VEXcode's AI Vision Utility (each with its name and who measured it, under what
light), whether the AI model's OBJECTS are on, and the class names that model must report; and the
MOUNT, measured from the robot's tracking centre with how well each number was measured. A mount nobody
has measured stays UNSET — that is not an error: the camera is built and read, and only the geometry
refuses, naming why. `RobotConfig::valid()` claims the camera's port like any other ("port 3 is claimed
by the LEFT drive and by the AI Vision camera").

**At boot the library writes the camera, it never trusts it.** Making the camera writes its colours and
its detection modes (exactly the ones asked for — AprilTags OFF) and reads each back, because PROS reports
success whatever the camera did. It writes them again after a read the camera does not answer, and
whenever its once-a-second check — the modes and every colour read back, the model's names asked again —
finds one changed. And it checks the camera's AI MODEL by NAME before an object's class number means
anything — VEX's Override model calls class 3 a yellow Pin; its Classroom model calls class 3 a blue ring,
and a route would chase it. Until every expected name matches, every object is WITHHELD (the colours still
come through), and since the names are asked again every second, a match is never more than a second old.
A colour blob the camera reports for a colour this program did not write (one another program left on it)
is withheld too. A camera the provider will not make — nothing plugged in, the
OLDER V5 Vision sensor on the port, a cable out — leaves the camera ABSENT, and **the robot still builds
and drives**: a dead camera must never cost a match. The boot line's last words say which:

```text
... GPS ABSENT tags ABSENT camera ABSENT
... GPS ABSENT tags ABSENT camera port 3 set up, model OK
... GPS ABSENT tags ABSENT camera port 3 set up, model DIFFERENT (class 0 is 'blueBall', the robot file expects 'redBluePin')
... GPS ABSENT tags ABSENT camera port 3 ABSENT (port 3: a vision sensor (expected an AI Vision sensor))
```

(Since the camera's first part a census names the AI Vision sensor "AI-VISION" — assuming the brain
reports it as one, which no census has shown yet; "VISION" now means the older sensor only.)

**A frame, and what it can and cannot tell you.** `camera()->read()` returns one `hal::VisionFrame`:
up to 24 detections — colour blobs and model objects, each with its box in the picture — the time the
read FINISHED, and how the read went. **Only a frame whose status is `Ok` is evidence of anything.** A
read that failed is a frame that says so, with NO detections — never the last good picture held over,
which would read as a fresh "nothing changed". A read that reports 24 or more is "possibly full": the
camera sends at most 24 and may hold more, so what matters may be the one it never sent. And PROS gives no frame number
and no camera timestamp: the picture is OLDER than the time the read finished, by an amount nobody has
measured yet. A camera that freezes looks exactly like a still scene.

**Pixels into directions and distances, each with its error.** `vision::CameraModel`, built from the
camera's optics (VEX's 74 by 63 degrees — two different focal lengths, never square pixels, each with
its own tolerance) and the measured mount, turns a box into a bearing, an elevation, a range from where
its base meets the floor, a range from its known height, and a field position with a radius — every
answer an INTERVAL, the exact range over every combination of the stated tolerances, never one number.
(A Pin's box takes in its top face as well as its side, so the range from its height reads a little
NEARER than its near rim.) A pose bound that is not a bound — a negative or non-finite tolerance — gives
an INFINITE radius, never a tight one. Near the horizon a floor range is refused rather than reported as
infinite, and a mount whose pitch with its tolerance reaches past 55 degrees is refused by the geometry
(the camera is still read; only its directions and distances are not computed). Two quantities from
ONE camera (a band's two edges, a Pin's base and its height) are checked one camera at a time — is
there one camera inside the tolerances that sees this box as that? — never by overlapping two widened
intervals. And a robot that moved since the picture was taken is not
where its pose says: `vision::boundAtImage()` widens the pose's bound by the speed and turn rate times the
image's MEASURED age; until that age is measured on the bench, use the camera's field positions only with
the robot stopped.

**Hardware beliefs this rests on, unmeasured:** HA-192 to HA-203 in the Hardware Assumptions Register,
all new with the camera — among them the frame's freshness and the image's age (HA-192), the optics
(HA-194), the AI model kept on the sensor and named as VEX spells it (HA-197), and the Toggle's shape
(HA-201). The camera's bench page settles them.

## Gotchas, collected

- **The first motion waits for sensors.** Budget its timeout for ~2 s of IMU calibration.
- **`strafeTo` holds the heading it finds** — aim first, then strafe.
- **Trajectory options are per leg**, and every waypoint costs a settle.
- **Tank + sideways target = honest timeout.** Use the turn-then-drive idiom. A tank `moveTo`
  steers back onto its line only when the leg starts within 15° of its target heading; the log
  says `turn first: face()` when it does not.
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
