# 6 — How things fail

> **Covers:** the ways autonomous runs actually go wrong — drift, wheel slip, dead sensors,
> brownout, jams — and the library's philosophy for surviving them.
> **Read this if:** you want to understand faults before you meet one in a log, or you're
> wondering why so much of this library is about things going wrong.
> **Assumes:** [Chapters 3](03-knowing-where-you-are.md)–[5](05-getting-there.md).

## Why a whole chapter on failure

A routine that works when everything works is the easy 80%. Competition robots operate in the
other 20%: batteries sag, robots collide, sensors power up confused, wheels slip on a scuffed
tile. The difference between a good autonomous and a great one is rarely the happy path — it's
what happens in the three seconds after something goes wrong. shulib was built around a specific
stance on this, and knowing the stance makes everything else in the library make sense:

**Faults log and recover. They never crash.** A mid-run problem gets detected, named, recorded,
and *survived* — the run continues with whatever capability remains. A robot that freezes on the
field scores zero; a robot that abandons one bad motion and drives on can still score everything
else. The corollary: **failures must be loud.** Every detected problem gets a named code in the
log, because a failure you can't see in the log is one you'll re-live at the next event.

Here are the actual failure modes, worst-first.

## Drift — the failure that's always happening

Chapter 3 covered the mechanism; here's the operational reality. Drift isn't an *event* — it's a
constant background process. There is no moment when it "goes wrong"; the estimate is simply a
little worse every second, and the run is a race between your routine and your error budget.

What it looks like: everything *reports* success — motions settle, the log is green — but the
robot is physically a bit off target, and more so late in the run. Nothing inside the robot can
directly see drift (that's what makes it drift), which is why the library's diagnostics record
whether the estimate is running on dead reckoning alone (the `DR` flag) versus being corrected
by an absolute sensor — the honest label for "my error is currently growing."

## Wheel slip and collisions — sudden position lies

A collision, or a wheel-spinning shove against a wall, feeds the odometry motion that didn't
happen (or hides motion that did). Unlike drift's slow creep, this is a step: the estimate can go
from an inch off to eight inches off in half a second, and it *stays* wrong — odometry never
forgets (Chapter 3).

The extreme case is a **stuck or dead encoder** — a tracking wheel that stops reporting entirely.
Undetected, this is catastrophic in a specific, nasty way: the robot drives, the estimate doesn't
move, so the controller sees unchanging error and pushes *harder*, at full authority, into
whatever it's touching — a runaway powered by its own blindness. shulib runs a cross-check
against exactly this: if the motors are demonstrably spinning but the odometry reports no motion,
it raises the fault **`ODO_STUCK`** — "the estimate is lying." This is the *one* fault that
aborts the current motion by default (into a safe stop), precisely because continuing to steer by
a lying estimate at full power is the worst available option. In simulation, the abort turns a
42-inch runaway into a 4-inch one.

## Sensors that fail by lying, not by erroring

Boot is not the only time a sensor tells you something false with a straight face. This is the
single most important thing to know about V5 hardware, and it is why the adapter layer exists at
all: **a failed V5 sensor read does not come back as an error. It comes back as a plausible
number.**

Three shapes of it, all of which the adapters now defuse before the value reaches anything:

- **The sentinel.** PROS reports a failed read by returning a special value — a huge integer, or
  literally infinity. Convert one of those without checking and you get a pose in the next
  galaxy. Every adapter screens the raw value *before* converting it, and on a bad read **holds
  its last good value** rather than substituting zero. Zero is the dangerous choice: a zeroed
  encoder reads as "the robot stopped", which is exactly the lie that makes a dead-encoder
  runaway invisible. A frozen value, by contrast, is what the wheels-spin-but-nothing-moves
  cross-check is built to catch. The screened reads are counted, so the log can show you a sensor
  that is quietly failing.
- **The in-band impostor.** The distance sensor's way of saying "nothing in view" is to report
  **9999 mm** — not an error, just a number, which converts to a perfectly believable 393 inches
  of wall. A capture-confirm that thresholds on distance alone reads that phantom wall as a real
  object. The adapter maps it to **zero confidence** instead, which is why the rule for that
  sensor is *always threshold `confidence()` before trusting `distance()`*.
- **The dead port that reads "pressed".** A digital input on a dead ADI port returns the error
  sentinel, and that sentinel is not zero — so an unscreened limit switch on a broken port reads
  as **permanently pressed**. On a homing routine that is a lift driving into its own hard stop
  and staying there.

The through-line: **the adapters convert exactly once, at the edge, after screening.** Everything
above them gets a value that is either good or visibly held, never a sentinel wearing a unit.
None of this has been exercised on a robot that was actually driving, so treat the *thresholds*
as provisional — but the shapes are real, and they are what a V5 log will show you.

## Sensors that boot up lying

Real V5 sensors are at their least trustworthy right after power-on. The IMU runs a calibration
for a couple of seconds after boot, and *during* calibration it reports garbage that looks
plausible. Software that reads it a moment too early bakes a garbage heading into everything
downstream — a classic, maddening field bug (works on the bench, fails when you power-cycle at
the field).

shulib's defense is structural: the estimate reports itself as *uninitialized* until sensors are
genuinely live, and motions **wait, motionless**, until it is (with their watchdog still running,
so a sensor that never comes up gives you an honest `TimedOut` rather than a hang). You'll notice
this as: the first motion of a run doesn't move for a beat. That pause is the library refusing to
act on garbage — budget your first motion's timeout to allow for it (about 2 seconds of IMU
calibration).

A sensor can also die *mid-run* (a cable works loose). The health monitor notices a
formerly-alive IMU going quiet and raises **`IMU_LOST`**. The run deliberately continues — a
degraded estimate still beats a parked robot — but the log tells you exactly when it happened,
which is usually the moment the run's pose story starts diverging from reality.

## Brownout — the whole robot dims

Motors draw enormous current under load. A hard push with a weakening battery can drag the
system voltage low enough that everything droops — motors lose torque and, if it gets bad
enough, electronics reset. That's a **brownout**. Before the cliff, there's a milder constant
version: as battery voltage sags over a match, the same commanded voltage produces less speed.

The library monitors battery voltage continuously and raises **`BROWNOUT`** when it crosses the
danger line. It does **not** try to cancel the sag, and that is deliberate: because it commands
*actual volts* rather than a percentage of the pack, the only battery correction it makes is a
**ceiling** — each wheel command is clamped to what the pack can still deliver, and a clamped
command is flagged so the motion layer knows it is voltage-starved. The
run continues (the estimate isn't compromised — the robot is just weaker), but the log records
it: a run full of timeouts *plus* a brownout flag usually means "battery," not "code."
Prevention is operational, and it's on you: fresh batteries for scored runs.

Related: a motor worked hard gets hot, and V5 motors *throttle themselves* at temperature —
suddenly your carefully tuned control is commanding a weaker motor. That surfaces as
**`MOTOR_OVER_TEMP`**.

## Jams and unreachable targets — honest timeouts

A robot pinned by a defender, a mechanism jammed on a game piece, a target that physics won't
allow (tank asked to strafe): the motion does its best until its watchdog expires, stops the
motors, and returns `TimedOut` (Chapter 5). The failure is contained to one leg, and your routine
decides what's next. When you read a log, remember that **`TIMEOUT` is a symptom with many
diseases** — jammed, blocked, unreachable, undertuned, or just an over-tight time budget.
[Chapter 12](12-when-things-go-wrong.md) walks the differential diagnosis.

## Running out of match — the failure the clock causes

Every bound above is scoped to one motion, one wait, one mechanism operation. There is a whole
class of failure none of them can see: the *run* going long. Every step behaves exactly as
designed — and the sum doesn't fit the match. A leg that fights a defender for six seconds
steals those seconds from every leg after it; a mechanism that waits its full budget for a ring
that never comes spends time no later step gets back. The measured worst case is instructive:
a stalled routine keeps the motors safe the whole time (each motion's own watchdog stops it) —
**safe and parked are different things, and only one of them scores.** A run that ends
mid-field with the motors politely braked is a *lost-points* failure, not a runaway, and it is
invisible to every per-step bound because no step misbehaved.

Since the sequence layer landed, that failure has an owner: a **run-scoped guard** you wrap
your whole auton in. You give it two instants — when to stop scoring and go do your final act,
and when everything must simply be *safe* — and the action to perform. Both numbers and the
action are yours: the library has no idea how long your match is or what your endgame is worth,
and it refuses to guess. [Chapter 14](14-what-it-cannot-do-yet.md) states exactly what the
guard's guarantee covers and what it cannot; [Chapter 9](09-the-recipe-api.md) covers how it
interacts with a recipe chain; the [cookbook's match-window recipe](../cookbook/03-timing-and-partners.md)
is the worked example.

## Software's own failures — contained, not trusted

The library doesn't exempt itself from suspicion. Two more fault families exist to catch *its
own* potential bugs, and knowing they exist helps you trust the rest:

- **Numeric corruption.** A divide-by-zero somewhere producing NaN ("not a number" — the value
  that poisons every calculation it touches) could silently rot the pose estimate. Guards check
  for non-finite values at the boundaries and replace them with safe fallbacks, raising
  **`NAN_POSE`** — the estimate freezes rather than going insane. Similarly, **`IMPLAUSIBLE`**
  fires when the estimate claims something physics forbids (e.g. the pose jumped faster than the
  robot can move).
- **Timing failures.** The control loop assumes it runs every 10 ms. If a tick takes much longer
  (**`LOOP_OVERRUN`**), every rate-based calculation in that tick — PID's D term, odometry's
  integration — quietly degrades. The loop monitor measures every tick and names the overrun,
  including *which phase* of the loop ate the budget.

## The pattern to internalize

Notice the design running through all of these:

1. **Detect** — a dedicated check per failure mode, running continuously.
2. **Name** — a specific fault code (`ODO_STUCK`, `IMU_LOST`, `BROWNOUT`, ...), never a vague
   error.
3. **Contain** — the blast radius is one motion at most; almost all faults don't even cost that.
4. **Record** — the first fault of a run is latched specially (one root cause usually triggers a
   cascade of secondary faults; the *first* one is the disease, the rest are symptoms), and the
   end-of-run summary reports it.
5. **Continue** — the run goes on, degraded but alive.

Every fault code, its exact meaning, and what to do when you see it is cataloged in
[Chapter 11](11-reading-the-diagnostics.md). The honest caveat, as always: every threshold in
this chapter (what voltage counts as brownout, how long a stuck encoder takes to detect) is a
provisional number verified against simulated hardware, and the whole detect-name-contain
machinery has yet to meet a real robot. The machinery is real and tested; the calibration awaits
hardware.


## The exact signatures

The exact surface behind this chapter: the [fault vocabulary](../api/fault.md) (every raisable code and the latch), [`HealthMonitor`](../api/health_monitor.md), [`LoopMonitor`](../api/loop_monitor.md), the [plausibility guard](../api/plausibility_guard.md), [`Watchdog`](../api/watchdog.md), [`ExitGroup`](../api/exit_group.md) and [`SettledUtil`](../api/settled_util.md), and the [odometry stall cross-check](../api/odo_stall_check.md).

---

*Next: [Chapter 7 — Getting set up](07-getting-set-up.md)*
