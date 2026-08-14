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

## The GPS corrector: what changes when a real one exists

The first real corrector is finished: it reads the V5 GPS and feeds the fusion layer. This is
worth pausing on, because it changes the mental model this chapter has been teaching, not just
the feature list.

Up to this point the story was: *error only accumulates.* Odometry cannot notice an error it
absorbed, so every mistake stays in the total forever, and the only defence is to finish before
the total gets large. With a working corrector the story becomes: **error accumulates between
fixes and is removed at each one.** Drift stops being a quantity that grows with the length of
your routine and becomes a quantity that grows with the time since the last accepted fix. That
is a different kind of number to design around. A sixty-second run is no longer inherently
worse than a ten-second one — provided the strip is visible.

Six things the corrector does before it will move your estimate an inch, each of which is a way
GPS data can be wrong:

- **No fix, no proposal.** If the sensor cannot see the strip, the corrector contributes
  *nothing* — not a weak pull, nothing. Off the strip the device still returns *some* position,
  and quietly trusting it is how a routine gets dragged toward a place the robot has never been.
- **One measurement, folded once.** The GPS camera produces a new reading about every 50 ms
  while the control loop runs at 100 Hz, so the same reading is handed out about five times. The
  corrector uses it once and declines the repeats. Counting one observation five times would
  make the correction strength depend on your loop rate, which is not a thing you should have to
  think about.
- **It compensates for lag.** A fix describes where the robot *was* when the camera looked,
  roughly 50 ms ago. Applied as though it described *now*, it drags the estimate backwards along
  your direction of travel — at 40 in/s that is a systematic two inches, always in the same
  direction, so it never averages out. The corrector carries the fix forward by the odometry
  travelled since it was taken.
- **It ignores fixes taken mid-spin.** During a fast rotation the geometry that converts a
  camera reading into a robot position is at its least reliable, so those fixes are dropped.
- **It gates on the sensor's own uncertainty, not on a fixed distance.** The GPS reports how
  well it thinks it is doing; a fix that disagrees with the estimate by more than a few times
  that figure is rejected as a lie. Crucially the bound *widens* the longer the robot has been
  navigating blind — after twenty feet of dead reckoning, a large disagreement is believable,
  and a fixed bound would reject the truth exactly when it was needed most.
- **It never touches heading.** The GPS reports one; the library does not use it. Heading
  belongs to the IMU, which is far better at it, and the fusion layer is built so that no
  corrector *can* rotate the robot.

Every one of those decisions is written into the diagnostics, per tick, with the numbers it was
made from — see [Chapter 11](11-reading-the-diagnostics.md). If your GPS is doing nothing, the
log will tell you which of the six reasons is why.

**And the honest limits, because they matter more than the feature:**

- **It has never seen a GPS.** Everything above was proven against a simulated sensor whose
  noise, timing and failure behaviour are *guesses* — reasoned guesses, written down and
  scheduled for measurement, but guesses. See the
  [Hardware Assumptions Register](../hardware-assumptions.md).
- **How much it helps depends on numbers nobody has measured yet.** A corrector is worth folding
  in when the sensor is more accurate than your accumulated drift. In simulation those two are
  currently about the same size, so the measured gain is real but modest. On a real field the
  balance could be much better or much worse, and it will not be known until both are measured.
- **It corrects position, not a lost robot.** Corrections are deliberately capped in size (next
  section), and a correction larger than a foot is rejected outright. If the estimate is badly
  wrong — a hard collision, a wheel that slipped through a whole turn — the corrector will not
  rescue it. Bounded drift is not the same promise as recovery.
- **Driving Skills has no GPS strip.** In that event the corrector reports "no fix" for the
  whole run and the estimate is pure dead reckoning, exactly as it was before. Everything this
  section describes applies to Autonomous, not to Skills. Plan routines accordingly.

On top of all that, everything here has run only against simulated sensors. The simulation is
deliberately hostile — sensors that lie during startup, freeze, drop out, and drift — but real
hardware will have its own opinions. See [Chapter 14](14-what-it-cannot-do-yet.md).

## The tag corrector: the first thing that can tell you which way you are facing

This is the second change to the mental model, and it is a bigger one than the first.

Everything in this chapter so far has treated **heading as something only the IMU knows**. The
odometry asks the IMU which way the robot is pointing and believes the answer; the GPS reports a
heading and the library deliberately throws it away; and no corrector was allowed to rotate the
robot. That was correct while it lasted, and it had one consequence nobody could argue with:
**heading error only ever grew.** A gyro drifts, slowly and in one direction, and nothing could
notice.

An AprilTag is different in kind from a GPS strip. What a tag gives you is not a position — it is
a *relative pose*: how far away the tag is **and which way it is turned relative to you**. If you
also know where that tag sits on the field, the two together say where the robot is *and which
way it is pointing*, in absolute field terms. That is the only absolute heading available to this
library, and correcting heading is what the team's accuracy spec actually turns on.

**The IMU still owns rotation.** This is worth being precise about, because it sounds like the
previous rule was thrown away. It was not. Every degree the robot actually turns still comes from
the IMU, tick by tick, exactly as before. What the tag corrector learns is a slowly-moving
**bias** — "the IMU reads about three degrees low today" — which is added to the IMU's answer.
The robot's reported heading is *the IMU's reading plus a learned correction*, and that
correction is allowed to move only a fraction of a degree per tick. There is no code path
anywhere that assigns the robot a heading. A yaw reset in the middle of a match would be worse
than a position jump, because every field-relative command issued afterwards would inherit it.

The checks it runs before it will move anything, each one a way tag data can be wrong:

- **A tag it has never heard of is not a fix, it is a configuration error.** shulib ships **no
  built-in map of where the tags are** — that is your input, and the library will not guess. A
  tag whose id is not in your map produces a distinct, loud diagnostic rather than silence.
- **Too close or too far, and it is not used.** Very near, the tag overfills the frame; far away,
  the maths that recovers a tag's *angle* becomes unreliable much sooner than the maths that
  recovers its *distance*. There is a trusted range band, and outside it the observation is
  dropped.
- **A poorly-detected tag is declined.** The detector reports how sure it is; below a floor the
  fix is not worth folding. Without that floor, a barely-seen tag still produces a microscopic
  pull — and the run then *reports itself as corrected* while having no usable anchor, which is
  worse than reporting the truth.
- **It ignores frames taken mid-spin.** A spinning robot smears the tag across the image, and a
  rolling shutter bends it into a shape that the solver will happily interpret as a different,
  confidently wrong pose.
- **It compensates for lag — in heading as well as position.** A tag fix describes where the
  robot was, and *which way it was facing*, about 80 ms ago. At a brisk turning speed that is
  fourteen degrees, which is fourteen times the entire heading error budget.
- **Vision runs on its own clock, and you drive it.** The corrector has two methods: one you call
  from a vision-rate task, and one the control loop calls every tick. Only the first one talks to
  the camera. That split exists because reading a camera allocates memory, and the 10 ms control
  loop must not. **If nothing calls the vision method, the corrector says so on every tick** — a
  wiring mistake shows up as a specific diagnostic rather than as a feature that quietly does
  nothing.

**And the honest limits, which are larger here than for the GPS:**

- **No camera has ever been pointed at a tag by this project.** The corner-to-pose maths is
  proven against synthetic images computed from geometry, and the corrector against a simulated
  camera with invented noise. It has never seen a lens.
- **The tag map is the most dangerous input in the library.** Sensor noise averages out; a wrong
  map does not. A tag entered two inches off yields a corrector that is *confidently* two inches
  wrong every time it sees that tag, with a small residual and a high confidence — which is to
  say, it looks exactly like a healthy fix. Measure your tags, and record how you measured them:
  the library makes you state where each number came from and refuses an entry that does not.
- **Two correctors that disagree used to be bounded, not resolved** — the library limited how
  far either could pull and let the estimate settle between them. There is now a filter that can
  actually weigh them against each other; the next section is about it, and about what it costs.
- **But it works where the GPS cannot.** The field's tags do not depend on the GPS strip, so in
  **Driving Skills** — where there is no strip at all — the tag corrector is the only absolute
  source of anything. That is the event where this feature is worth the most, and also the one
  where nothing else can catch its mistakes.

## Knowing how wrong you might be

Everything above shares one blind spot, and it is worth seeing clearly before reading about the
thing that fixes it.

When a sensor says "you are eight inches from where you think you are", there are two completely
different situations behind that sentence. Maybe the estimate is fresh and trustworthy, and the
sensor glitched — in which case you should ignore it. Or maybe the robot has been dead-reckoning
across the field for twenty seconds and really *is* eight inches out — in which case that reading
is the most valuable thing that has happened all match. **The two look identical from the
outside.** Everything described so far answers with the same fixed rule either way: reject
anything past a set distance, and pull gently on anything closer.

To tell them apart, the estimator has to know something it has never known: **how wrong it might
be right now.**

### What a covariance is, in plain words

Suppose you are asked where the robot is and you answer "(24, 36)". A **covariance** is the
answer to the follow-up question: *how sure are you?* — written as a number rather than a
feeling.

At its simplest it is a distance: "I am at (24, 36), give or take about half an inch." That
"give or take" is a standard deviation, usually written σ. A covariance is the same idea done
properly for more than one number at once, so it can also express things a single distance
cannot — for example *"give or take half an inch along the direction I am driving, but three
inches sideways"*, which is exactly what happens to a robot that has been driving straight for a
while with a slightly uncertain heading. It is stored as a small table of numbers, one for each
pair of quantities the estimator tracks, and the entries off the diagonal say how the errors are
*linked*: if the estimator is wrong about its heading, it is probably wrong about its sideways
position too, and in a predictable way.

The two things that happen to it are the whole story:

- **It grows while you dead-reckon**, in proportion to how far you have actually driven. Sitting
  still barely costs anything; crossing the field blind costs a lot.
- **It shrinks whenever a fix is folded in**, in proportion to how good that fix claimed to be.

That is all a covariance is: a running answer to "how sure am I?", inflated by driving and
deflated by looking.

### What having one buys

Four things, and they are the reason the filter exists.

**The gate stops being a fixed distance.** Instead of "reject anything more than twelve inches
away", the test becomes "reject anything more than three times as far away as I could plausibly
be wrong". The same fix, twenty inches out, is now *rejected* when the estimate is fresh — no
sensor is that trustworthy against a good estimate — and *accepted* after a long blind stretch,
because by then twenty inches is well within what the estimator admits it might be off by. The
old fixed gate had a real consequence recorded during development: an estimate twenty-nine inches
from truth **never recovered**, with a perfectly good GPS in view the entire time, because
twenty-nine is more than twelve. That specific failure is gone.

**Two disagreeing sources can finally be resolved.** If the GPS says one thing with a σ of two
inches and the tags say another with a σ of half an inch, the filter does not pick one and does
not split the difference — it lands about sixteen times closer to the tags, because a σ four
times smaller is sixteen times more informative. That weighting is not a preference anyone typed
in; it falls out of the arithmetic.

**"How sure am I?" becomes something you can read.** The blackbox now carries the estimator's own
uncertainty on every tick, which turns "the robot ended up in the wrong place" into a question
with an answer in the file.

**The estimator can notice that it is lost.** Which brings up the one behaviour worth
understanding before you see it in a log.

### When the estimator gives up on itself — and why it still does not teleport

A confident filter that is *wrong* is a trap, and it has a very real cause in VEX: an opponent
shoves your robot. The wheels do not turn, so the odometry reports no travel and the estimate
stays exactly where it was — and stays **certain**, because as far as the wheels are concerned
nothing happened. The robot is now thirty inches from where it believes it is, and every honest
GPS fix looks like an outrageous lie. The gate does what it is built to do and rejects all of
them. Forever.

So there is an escape hatch, and its design is the interesting part. After a long run of
consecutive rejections *and* a persistently large disagreement, the estimator declares that it is
lost — and what it does then is **throw away its confidence, not its position.** The covariance
is reset to "I could be anywhere within a tile". The estimate itself does not move by so much as
a thousandth of an inch.

That distinction is the whole design. With the confidence gone the gate is wide open, so the next
fixes are accepted and the estimate walks home at the usual bounded rate — about two and a half
seconds to cover thirty inches. **It never jumps.** An estimator that quietly teleports is worse
than one that stays wrong: a wrong-but-continuous estimate still produces sane motion, while a
teleport hands the motion controller a target that moved eight feet between ticks, and the robot
lurches. The event is also written into the log as its own word, because an estimator that
changes its mind about how much to trust the world without saying so is the hardest kind of run
to debug.

### Two filters, and why the simpler one is still the default

The library ships **both**, behind the same seam, and choosing between them is one argument where
you build the estimator.

- The **complementary filter** is the default. It has no covariance. It rejects anything past a
  fixed distance and pulls a fixed fraction of the way toward whatever it accepts.
- The **extended Kalman filter** is the one this section has been describing.

Keeping the simpler one as the default is a deliberate choice, and the honest reasons are worth
stating. Measured over eight seeded sixty-second runs against the simulated robot, the two finish
within about an eighth of an inch of each other — and the *complementary* filter is very slightly
ahead. That is not a defect in the Kalman filter; it is what happens when the simulated GPS is
noisier than the drift it is correcting, so the best available move is mostly to ignore it, and a
blunt fixed gain ignores it slightly harder. On top of that, **every noise number the Kalman
filter uses is currently a guess**, because no part of this project has met a real robot, and a
filter whose behaviour a team member can explain out loud is worth something real on competition
day.

So: use the Kalman tier when you need what only it can do — recovering from a large displacement,
or resolving two sources that disagree — and expect the accuracy difference between them to be
smaller than the things you have not measured yet.

## What this means for your routines

- **Always set the starting pose first.** Odometry is a running total; `setPose` is where the
  total starts. Place the robot carefully to match — every inch of placement error is error the
  run starts with. (Heading is special: it's owned by the IMU, which calibrates at power-on. The
  wiring code establishes the starting heading; your `setPose` sets position. If a tag corrector
  has already learned that the IMU reads a little low, `setPose` keeps that correction — moving
  the robot says nothing about which way the gyro is wrong.)
- **Expect drift, and design around it — but know which run you are in, and what you have
  wired.** In Autonomous, with the strip visible, position drift is bounded by the GPS corrector
  and a long routine is not inherently worse than a short one. In **Driving Skills there is no
  strip** — but the field's AprilTags are still there, so a camera plus a tag map is the only
  thing that bounds anything in that event. With neither, the old rule stands unchanged: plan
  legs so that accuracy-critical actions happen early, while the accumulated error is still
  small.
- **The estimate is the robot's only reality.** When a log says the robot "reached (24, 36)," it
  means *the estimate* reached (24, 36). If the estimate had drifted an inch, the robot is an
  inch from where it thinks — and neither it nor the log can tell you that from inside. This is
  the single most useful mental habit for debugging autonomous runs: always ask, "is this a
  motion problem, or an estimate problem?" [Chapter 12](12-when-things-go-wrong.md) is organized
  around exactly that question.

---

*Next: [Chapter 4 — Drivetrains](04-drivetrains.md)*
