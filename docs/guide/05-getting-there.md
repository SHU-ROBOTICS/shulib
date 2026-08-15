# 5 — Getting there: control

> **Covers:** how the robot actually arrives at a target — feedback control and PID (without
> math anxiety), feedforward, and settling ("how does it know it has arrived?").
> **Read this if:** you want to understand what happens between "I called `moveTo`" and "the
> robot stopped," or you've heard "PID" used like a magic word and want it demystified.
> **Assumes:** [Chapters 2](02-the-field-and-coordinates.md)–[4](04-drivetrains.md).

## The shape of the problem

You've asked the robot to go to (24, 36) facing 90°. The localizer (Chapter 3) says it's
currently at (0, 0) facing 0°. The kinematics (Chapter 4) can turn any desired velocity into
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
gains for a specific robot, which is honest trial-and-error guided by the symptoms above
(sluggish → more P; stops short → a touch of I; overshoots → more D or less P).

Two shulib specifics worth knowing:

- There are separate controllers for position and for heading, running simultaneously — that's
  what makes "drive there while rotating to face this way" one motion instead of two
  (Chapter 4's holonomic point, realized in control).
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
we don't have yet. Today's values match the simulator's physics by construction. This is the
honest circularity of pre-hardware work: the control *logic* is proven, the *constants* are
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

When all three hold, the motion reports **`Settled`** — the honest "I have arrived." Heading has
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
**`Cancelled`**, means something outside the motion stopped it (your code, or a fault — Chapter 6).

These three words — `Settled`, `TimedOut`, `Cancelled` — are the vocabulary every motion ends
with. You'll see them as return values in code and as `SETTLED` / `TIMEOUT` / `CANCELLED` in the
diagnostics, and the first skill of reading a run log is scanning for the leg that stopped
settling.


## The exact signatures

The motion layer's exact surface: the [`IMotion` contract](../api/motion.md), the primitives [`MoveToPose`](../api/move_to_pose.md), [`TurnTo`](../api/turn_to.md), [`StrafeTo`](../api/strafe_to.md), [`HoldPose`](../api/hold_pose.md) and [`DriveBrake`](../api/drive_brake.md), the knobs in [`MotionConfig`](../api/motion_config.md), the [`MotionScheduler`](../api/motion_scheduler.md) that runs them, and the one command path they all share, [`applyCommandPipeline`](../api/command_pipeline.md).

---

*Next: [Chapter 6 — How things fail](06-how-things-fail.md)*
