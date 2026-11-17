# 5 — Getting there: control

> **Covers:** how the robot actually arrives at a target — feedback control and PID (without
> math anxiety), feedforward, settling ("how does it know it has arrived?"), and curves (flowing
> through points instead of stopping at each).
> **Read this if:** you want to understand what happens between "I called `moveTo`" and "the
> robot stopped," or you've heard "PID" used like a magic word and want it demystified.
> **Assumes:** [Chapters 2](02-the-field-and-coordinates.md)–[4](04-drivetrains.md).

## The shape of the problem

You've asked the robot to go to (24, 36) facing 90°. The localizer ([Chapter 3](03-knowing-where-you-are.md)) says it's
currently at (0, 0) facing 0°. The kinematics ([Chapter 4](04-drivetrains.md)) can turn any desired velocity into
wheel speeds. The missing piece is the decision: *at this instant, how fast should the robot be
trying to move, and in which direction?*

That decision is remade about a hundred times per second, and the machinery that makes it is
called a **control loop**: measure where you are, compare to where you want to be, compute a
command, apply it, repeat. The gap between want and have is called the **error** — "24 inches to
go in x, 36 in y, 90° in heading" is an error. All of control is strategies for turning error
into commands.

## PID, without the anxiety

**PID** is the classic strategy. The letters stand for Proportional, Integral, Derivative, which
sounds like a calculus exam, but each is an ordinary idea you already use when you drive or ride
a bike:

**P — Proportional: push harder when you're farther away.** Command = error × some constant. Far
from the target → drive fast. Close → slow down. This one term does most of the work, and it's
genuinely how you reach for a cup: fast motion that tapers as your hand closes in. The constant
(called a **gain**, written `kP`) sets how aggressive the taper is. Too small: the robot crawls,
dawdling toward the target. Too large: it charges, can't shed speed in time, shoots past,
charges back — **overshoot**, and in bad cases a sustained wobble around the target called
**oscillation**. Almost all tuning is finding the aggression level between "sluggish" and
"wobbly."

**I — Integral: notice you've been stopping short.** Pure P has a subtle flaw: near the target,
the error is small, so the command is small — sometimes too small to overcome friction. The
robot parks an inch short, pushing gently forever. The integral term watches error *accumulate
over time* and slowly adds push until the leftover error finally closes. It's the "we've been
stuck here a while, lean on it" term. Its danger is over-accumulating (wind-up) and causing lazy
overshoot, which is why it's used sparingly.

**D — Derivative: ease off when closing fast.** The derivative term watches how *quickly* the
error is shrinking and pushes back against fast approaches — a gentle brake that starts before
the target rather than at it. It damps the overshoot that an aggressive P causes. Its danger:
it reacts to *change*, and sensor noise is nothing but change, so too much D makes the robot
buzz nervously.

That's the whole idea. PID is not magic and not deep — it's "push proportionally, notice being
stuck, brake early," summed. What makes it feel like dark arts is **tuning**: choosing the three
gains for a specific robot, which is plain trial-and-error guided by the symptoms above
(sluggish → more P; stops short → a touch of I; overshoots → more D or less P).

Two shulib specifics worth knowing:

- There are separate controllers for position and for heading, running simultaneously — that's
  what makes "drive there while rotating to face this way" one motion instead of two
  ([Chapter 4](04-drivetrains.md)'s holonomic point, realized in control).
- **Every gain in the library today is a placeholder.** They're tuned to the *simulated* robot,
  and they will all be re-tuned when a real robot exists. Each is registered as an assumption
  (HA-50 and neighbors in the [Hardware Assumptions Register](../hardware-assumptions.md)). Do
  not read the numbers in `motion_config.hpp` as measured truth — the file says so itself, loudly.

## Feedforward: predict first, correct second

PID is purely reactive — it only pushes *after* error appears. But we know things in advance:
motors need a minimum voltage before anything moves at all (static friction), and holding a
given speed needs a roughly proportional voltage. Why wait for error to reveal what physics
already told us?

**Feedforward** is the predictive half: given the speed we're asking for, compute the voltage
that *should* produce it, from a simple model of the motor ("this much to break friction, plus
this much per unit of speed"). Feedforward gets the command roughly right by prediction, and PID
only cleans up the leftover — disturbance, model error, battery sag. The division of labor makes
both parts better: feedforward does the bulk lifting so PID can stay gentle, and a gentle PID
doesn't oscillate.

The model's constants (you'll see `kS`, `kV`, `kA` in configs — friction, speed, acceleration
terms) are measured from the actual robot by driving it through a calibration script… on a robot
we don't have yet. Today's values match the host test plant's physics by construction. This is the
circularity of pre-hardware work: the control *logic* is proven, the *constants* are
stand-ins.

## Settling: how the robot knows it has arrived

"Drive to (24, 36)" has a surprisingly tricky ending. When is the robot *there*? This matters
practically: your routine is a sequence, and step 7 (drop the ring) must not start until step 6
(reach the goal) is genuinely finished. Call it done too early and you're dropping game pieces
while still moving.

The naive answer — "when error is zero" — never happens; a real pose estimate flickers by
hundredths of an inch forever. The next answer — "when error is small" — has a subtler flaw:
a robot *flying past* the target is momentarily "close." If closeness alone counted, a
40-inch-per-second drive-through would count as arrival.

So settling requires **three conditions at once** (this is the `SettleConfig` you'll meet in the
configuration):

1. **Close enough** — position error below a tolerance (e.g. half an inch).
2. **Slow enough** — error barely changing (the robot has actually stopped, not passing through).
3. **For long enough** — both of the above sustained for a short hold time (e.g. a tenth of a
   second), so one lucky flicker of sensor noise can't declare victory.

When all three hold, the motion reports **`Settled`** — "I have arrived." Heading has
its own tolerance trio (currently about a degree — and note the settle tolerance can't be tighter
than sensor noise, or the robot would never be "still enough"; the config file documents this
trade).

## The watchdog: refusing to hang

One more piece completes the picture. What if the robot *can't* arrive — a wall in the way, a
tank drive asked to strafe, a mechanism jammed against the field? Without protection, "block
until settled" would block forever, and the routine would die there, silently, mid-run.

Every motion therefore carries a **watchdog**: a per-motion time budget you set (e.g. "this leg
gets 3 seconds"). If the budget expires before settling, the motion stops the motors and returns
**`TimedOut`** instead of `Settled`. This is a designed-in guarantee, not a convention: *no shulib
motion can hang*, and this is one of the most heavily tested properties in the library. Your
routine always gets control back, always knows whether the leg succeeded, and can decide what to
do next — retry, skip, or head for the next scoring opportunity. The third possible answer,
**`Cancelled`**, means something outside the motion stopped it (your code, or a fault — [Chapter 6](06-how-things-fail.md)).

These three words — `Settled`, `TimedOut`, `Cancelled` — are the vocabulary every motion ends
with. You'll see them as return values in code and as `SETTLED` / `TIMEOUT` / `CANCELLED` in the
diagnostics, and the first skill of reading a run log is scanning for the leg that stopped
settling.


## Flowing through points: curves

Everything above ends a motion by STOPPING at its target. On a tank that means a route through
several points stops and turns at each one: `face` the next point, `driveTo` it, stop, turn again.
Since chunk CURVE1 the library also has a **curve**: one step that drives a smooth path through the
points without stopping at them.

```cpp
r.curveTo({{24_in, 12_in}, {48_in, 24_in, 0_deg}});         // through (24, 12), end at (48, 24) facing 0°
r.reverseCurveTo({{-12_in, 30_in}, {-36_in, 36_in, 0_deg}});  // backing, still FACING 0° at the end
```

**What it does.** The library joins where the robot IS (and the way it faces) and your points into
one smooth path, up to 16 points. A point `{x, y}` lets the library pick the smooth direction
there; `{x, y, heading}` says which way the robot points as it passes. Every 10 ms the robot finds
the point on that path 8 in ahead of itself (the **look-ahead**) and steers along the arc that
carries it there — **pure pursuit**. It slows BEFORE each bend, by one number, the **bend grip**
(`bendAccel`, how hard the robot may be pushed sideways), and slows to the end.

**How it ends.** The last few inches are an ordinary `moveTo`: when the robot is near the end, lined
up and on the end's line, the curve hands over, so a curve stops, is judged and is logged exactly
like any drive — the same arrival check, the same `{.positionTolerance = …}` /
`{.headingTolerance = …}`. A curve stops at its end; two steps in a row still stop between them.

**Two rules to remember.** A heading is ALWAYS the robot's NOSE — also on `reverseCurveTo`, where the
robot travels the other way. And do not list the robot's own spot as the first point: the curve
already starts there.

**When it refuses, and when it stops.** Before the robot moves, a curve is refused — the step fails,
one Warn names why — if its first point is more than 90° off the robot's nose ("turn first:
face()": the library still plans no turn for you), or if the path doubles back (two points on top
of each other, or a heading pointing back the way the path came). Backwards, it is the robot's BACK
that must point within 90° of the first point: a `reverseCurveTo` whose first point is in front is
refused with "... off the robot's back ... turn first: faceAway()". A bend tighter than half the
look-ahead is driven, with one Warn: expect the robot to cut that corner. While it drives, the curve
stops and fails the step if the robot has LOST the path — pointing more than 60° off it for a
quarter second (checked right up to the hand-off to the end `moveTo`), or making less than an inch
of progress in three quarters of a second (a dead side; not checked in the last 12 in, where the end
is slow on purpose). Once the end `moveTo` has taken over, only the step's `.timeout` stops it. A
side that dies EARLY on a gentle bend can keep the robot creeping along while it pivots; on the
host test plant a guard still stopped it within about 2.7 s and 10 in. A robot pushing a
wall with its wheels spinning also looks like progress on a robot whose position comes from its
drive encoders, so give every curve a `.timeout` near its real time.

**What it never does.** Every other verb keeps the library's rule that it never aims at a point. A
curve is the one exception, and only toward a point on the path YOU drew (or that path's end line,
past the end) — never at the end point itself, and never a turn the path does not contain.

**Tuning.** The two numbers that matter, `MotionConfig::curve.lookAhead` (8 in) and `bendAccel`
(60 in/s²), are PROVISIONAL: chosen on the host test plant to hold up under a weaker side and
slow-reacting motors, not for speed. The team tunes them on the robot in one session — look-ahead
first (shorter tracks tighter and wobbles more under lag; longer is steadier and cuts tight bends),
then the bend grip. A curve also uses the tank steering's yaw correction and its 15° end window
([Chapter 4](04-drivetrains.md)): switching that steering off (`tankSteer.maxSteer = 0`) switches both off on
curves too — the end then takes over without waiting for a window, and the curve no longer corrects
for a weaker side. Curves have been proven on the host test plant only: on the same five test
routes they took about half to two-thirds of the time of stop-and-turn legs.

### A sideways miss: what a curve fixes, and from how far

A tank cannot slide sideways. If the robot is a few inches to the side of where it must end, a
`moveTo` will not fix it; a one-point `curveTo` to the end pose will — **if it starts far enough
back**, because it needs room to arc onto the end's line before its last few inches, which are a
`moveTo`. With the shipped settings (look-ahead 8 in), the robot pointing within 20° of the end
heading: **a miss up to 6 in needs 2½ ft of run-up along the end's line; up to 1 ft, 3 ft.** This
holds with motors that react slowly (up to 0.18 s of lag on the host test plant) and a side up to 9 %
weaker; it leaves only an inch or two to spare, so add a few inches where there is room. A
noticeably weaker side (12–15 %) needs about 2¾ ft and 3⅓ ft. `{.positionTolerance = 1_in}` (when
1 in is good enough) brings it to 1¾ ft and 2 ft. If the robot file's look-ahead was changed at the
robot session, the distances change with it: look-ahead 10 in needs about 3¼ ft and 3⅔ ft;
look-ahead 6 in needs only about 1⅔ ft but does not settle at all on slow-reacting motors.

Closer than the rule, the curve gets close but not within half an inch: it runs until its
`.timeout` is used up, then stops — up to 3½ in off from 6 in or more back, and from closer about as
far to the side as it started — and the result line's ` xt` shows by how much
([Chapter 11](11-reading-the-diagnostics.md)). **Closer than the rule, write today's
turn-drive-turn: `face(x, y)`, `driveTo(x, y)`, `turnTo(heading)`** — every step said it arrived in
every host test, in about 3 s (worst about 4½ s). One thing to know: on a robot with a weaker side,
its last turn in place can shift the robot up to about 1 in (1½ in on a much weaker side) — about 1
run in 8 ended more than half an inch from the exact spot.

The library never backs up and tries again on its own: it cannot see behind the robot. If YOU know
the space behind is clear and the pose must be exact, write it: **`faceAway(x0, y0)` FIRST** —
without it this fails about half the time on the host test plant, because the backward drive is not
steered unless the robot already faces away — then `reverseTo(x0, y0)` to a spot 2½ ft back on the
end's line, then the `curveTo`. It is slower (about 4 s), and it arrived in all but 4 of 3,240 host
tests with a side up to 9 % weaker (about 95 % with a 15 % weaker side). All of these are host test
plant numbers, not robot numbers.

The rule is pinned by a test (`test/motion_curve_test.cpp`, CV23): if a change to the library moves
it, the rule here is re-measured and rewritten in the same change.

## The exact signatures

The motion layer's exact surface: the [`IMotion` contract](../api/motion.md), the primitives [`MoveToPose`](../api/move_to_pose.md), [`TurnTo`](../api/turn_to.md), [`StrafeTo`](../api/strafe_to.md), [`HoldPose`](../api/hold_pose.md) and [`DriveBrake`](../api/drive_brake.md), the curve follower [`CurveFollow`](../api/curve_follow.md) and its pure path [`CurvePath`](../api/curve_path.md), the knobs in [`MotionConfig`](../api/motion_config.md), the [`MotionScheduler`](../api/motion_scheduler.md) that runs them, and the one command path they all share, [`applyCommandPipeline`](../api/command_pipeline.md).

---

*Next: [Chapter 6 — How things fail](06-how-things-fail.md)*
