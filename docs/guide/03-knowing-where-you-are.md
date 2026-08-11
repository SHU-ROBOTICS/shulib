# 3 — Knowing where you are

> **Covers:** odometry (how a robot tracks its own position), why the estimate drifts, what each
> sensor can and cannot tell you, and how the library combines them.
> **Read this if:** you want to understand why the robot's idea of "where am I" is a guess, and
> what makes the guess good or bad.
> **Assumes:** [Chapter 2](02-the-field-and-coordinates.md) — pose, heading, coordinates.

## The core idea: count your own steps

Close your eyes and walk across a room you know. You can do it, roughly, because you're doing
what robots do: you know where you started, you count your steps, and you keep a running total.
"I started at the door, I've taken six steps forward and turned a bit left, so I must be near the
table."

That — start from a known pose and keep adding up your own measured movement — is called
**odometry** (also "dead reckoning"). It is the backbone of robot position tracking, and it is
why every autonomous routine begins by telling the robot its starting pose: the running total has
to start from somewhere, and the robot has no way to know where it was placed.

## The hardware: tracking wheels and the IMU

The robot measures its own movement with two kinds of sensor.

**Tracking wheels** are small, unpowered wheels that press against the floor and spin freely as
the robot moves. Each has a rotation sensor that reports how far the wheel has turned, which
converts directly to inches rolled. We use two, mounted perpendicular to each other: one measures
forward/backward travel, one measures sideways travel. Why unpowered wheels, when the drive
wheels already have encoders? Because drive wheels *slip* — they spin against the floor when the
robot accelerates hard or pushes against something, and a slipping wheel counts distance the
robot didn't actually travel. A free-spinning wheel has no torque on it, so it mostly just rolls.
(Mostly. Bumps and collisions can still lift or skid it.)

**The IMU** (inertial measurement unit — think "the gyroscope") senses rotation directly. It
answers "how fast am I turning?" and, by accumulation, "which way am I facing?"

Roughly a hundred times a second, the library reads all three sensors and does a small piece of
geometry: "the forward wheel rolled 0.11 inches, the sideways wheel rolled 0.02, and the IMU says
we rotated 0.4° — so the robot moved *this* much, in *that* direction, and the pose estimate
updates from (24.00, 36.00, 45.0°) to (24.08, 36.08, 45.4°)." Each little update is tiny and very
accurate. The pose is the sum of tens of thousands of them.

Two details of that geometry are worth knowing exist, even though you'll never touch them:

- The math handles *curved* movement exactly (the update treats each tick's motion as a small
  arc, not a straight line — the difference matters when driving and turning at once).
- **Heading comes from the IMU, never from the wheels.** You can in principle infer rotation by
  comparing left and right wheel distances, but wheel-derived heading inherits every wheel
  problem. The IMU measures rotation directly and is much better at it, so the library trusts
  the IMU for heading, always, and uses wheels only for distance. This "IMU-owned heading" rule
  comes up again and again in the codebase.

## Why it drifts

Here is the uncomfortable truth: **every tick's tiny error stays in the total forever.** The
sum-of-small-updates design means odometry has no way to notice, let alone remove, an error it
absorbed a thousand ticks ago. Error only accumulates. This slow accumulation is called
**drift**.

Where does per-tick error come from?

- **Wheel slip and scrub.** A tracking wheel skids through a collision, or lifts for a moment
  going over a field element joint. Distance is counted that didn't happen (or missed that did).
- **Real geometry vs. assumed geometry.** The math needs each wheel's diameter and mounting
  position. If a wheel is actually 2.01 inches instead of 2.00, every measured inch is 0.5% off —
  a half inch per hundred inches driven, silently.
- **IMU gyro drift.** The IMU's rotation sensing has a small bias that changes every time it
  powers on; the heading estimate slowly rotates even with the robot bolted to a table. A
  typical V5 IMU drifts on the order of a degree per minute — and recall from Chapter 2 that
  heading error converts into position error with every inch driven. Over a 60-second run, IMU
  drift alone can eat the entire heading accuracy budget. This single fact drives a lot of the
  library's design.

The consequence: odometry is *locally excellent and globally rotten*. Over two seconds it is
nearly perfect. Over sixty seconds, on a real robot, it can be off by inches — and it will report
that wrong pose with total confidence, because nothing inside it can know better.

## Correction: sensors that measure the field, not the robot

The only cure for accumulated error is a sensor that measures where the robot *actually is*,
against the field itself, rather than accumulating movement. Those measurements can pull the
estimate back toward truth. Each such sensor knows one kind of thing and fails one kind of way:

- **The V5 GPS sensor** is a camera that reads a barcode-like strip mounted on the field
  perimeter and reports the robot's absolute field position — no accumulation, no drift. Its
  weaknesses: it needs line-of-sight to the strip (robots block it; some events' fields don't
  have it), it's noisy (an inch or so of scatter), and it lags slightly behind reality. Great
  anchor, poor navigator.
- **AI Vision / camera + AprilTags** can recognize known visual markers and compute the robot's
  position relative to them — potentially very accurate, especially close-up for goal alignment.
  Needs the marker in view and good lighting; more moving parts.
- **Distance sensors** (a laser rangefinder, roughly) tell you how far the nearest surface is
  along one line. Pressed near a wall you know, that's a one-dimensional position fix. It cannot
  tell a wall from a robot passing by.
- **Optical sensors** see color up close — "there is something orange right here." Useful for
  game-piece handling; useless for position.

Notice the shape of the situation: odometry is smooth, fast, always available, and slowly wrong;
absolute sensors are jumpy, occasional, sometimes absent, and anchored to truth. Neither is
enough. You want both — smooth short-term motion from odometry, long-term anchoring from
absolute references.

## Fusion: one estimate from many witnesses

Combining sensors into a single best estimate is called **sensor fusion**. The library's fusion
layer (the `Localizer`) owns the one official pose estimate, built on odometry-plus-IMU, with a
"correction seam": a plug-in point where absolute sensors (GPS today; vision later) can nudge the
estimate toward what they observed.

Two principles govern it, and they explain behavior you will see in the logs:

**Corrections are gated.** A correction that disagrees wildly with the current estimate is
*rejected*, on the logic that a sensor reporting the robot teleported eight feet is more likely a
bad reading than a real event. You'll see this in diagnostics as gate accept/reject decisions.
The dark side: if the estimate is *badly* wrong, truthful corrections can look outrageous and get
rejected too. Gates trade rare catastrophic acceptance for rare catastrophic stubbornness.

**Corrections never snap.** Accepted corrections are applied gradually, a little per tick,
because motion control is running off this estimate in real time — teleporting the estimate
mid-move would make the robot visibly twitch.

The estimate also carries an honest quality label at all times — the library distinguishes
*"booting, no estimate yet"* / *"dead-reckoning"* (odometry only, drifting) / *"corrected"*
(absolute reference active) / *"degraded"* (something is wrong with a sensor). Motion code and
diagnostics both read this label; among other things, the robot refuses to act on
field-relative commands while the estimate is still booting.

One current-status note, so this chapter doesn't oversell: **the correction seam exists and is
tested, but no real corrector is finished** — today's estimate is odometry + IMU
(dead-reckoning). GPS and vision correctors are planned work; see
[Chapter 14](14-what-it-cannot-do-yet.md). On top of that, everything here has run only against
simulated sensors. The simulation is deliberately hostile — sensors that lie during startup,
freeze, drop out, and drift — but real hardware will have its own opinions. The library's
assumptions about real sensor behavior are cataloged, each with its planned measurement, in the
[Hardware Assumptions Register](../hardware-assumptions.md).

## What this means for your routines

- **Always set the starting pose first.** Odometry is a running total; `setPose` is where the
  total starts. Place the robot carefully to match — every inch of placement error is error the
  run starts with. (Heading is special: it's owned by the IMU, which calibrates at power-on. The
  wiring code establishes the starting heading; your `setPose` sets position.)
- **Expect drift, and design around it.** Short routines can live on pure odometry. A 60-second
  skills run cannot — plan legs so that accuracy-critical actions happen early, or near future
  correction opportunities (once correctors exist).
- **The estimate is the robot's only reality.** When a log says the robot "reached (24, 36)," it
  means *the estimate* reached (24, 36). If the estimate had drifted an inch, the robot is an
  inch from where it thinks — and neither it nor the log can tell you that from inside. This is
  the single most useful mental habit for debugging autonomous runs: always ask, "is this a
  motion problem, or an estimate problem?" [Chapter 12](12-when-things-go-wrong.md) is organized
  around exactly that question.

---

*Next: [Chapter 4 — Drivetrains](04-drivetrains.md)*
