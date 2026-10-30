# 18 — Mechanisms: arms, lifts, flywheels — in autons, on buttons, on routes

> **Read [chapter 10's "Mechanisms" section](10-the-api.md#mechanisms--declared-once-wired-separately)
> first** — it shows how a robot file declares a mechanism and wires it, and how to read a refusal.
> This chapter is what happens AFTER the declaration: how an `Arm`, a `Lift` and a `Flywheel` move,
> hold, find their zero, stay inside their limits and stop when something jams, and how to tune
> them — then how an AUTON runs a mechanism (one line per step), how the DRIVER's buttons do
> (presets, holds, toggles), how a ROUTE's markers do (every mechanism's ids, published), and what
> the direction check judges.
>
> **What exists, and what does not yet.** The mechanism core's first five parts are in (MECH-A1 and
> A2, 2026-10-15; A3, 2026-10-22; A4 and A5, 2026-10-29): the vocabulary and its build checks, the mechanism SERVICE
> (one object, ticked once per control tick, that runs every mechanism's behaviour), the behaviours of
> all four kinds of mechanism — power, pneumatic, POSITION (an arm, a lift) and SPEED (a flywheel) —
> and everything that calls them: the auton STEPS (`r.move(arm["score"], 1.5_s)`), the driver
> CONTROLS (presets on buttons), the ROUTE BINDER that runs a marker's id, and the direction check's
> JUDGEMENT — and, since the fifth part, robot two on all of it: its claws are declarations, every
> program that boots it binds one service, prints each mechanism's boot lines and publishes its route
> ids, and its driver file is three lines of controls ([the cookbook](../cookbook/06-driver-control.md)).
> What is still to come: the bench page that runs the direction check (the core's sixth part). Everything below is measured on the **host test plant** — invented models of an
> arm, a lift and a flywheel ([HA-183](../hardware-assumptions.md)) — and the library's own numbers in
> it are invented starting values ([HA-182](../hardware-assumptions.md)). **Nothing here has run on a
> robot.**

## The five kinds, and what each one does

| Kind | Its units | What a step does | It goes on after the step | Required besides the common fields |
|---|---|---|---|---|
| `Arm` | degrees of the arm, plain numbers | move to an angle, or home | it HOLDS where it got to | `.gearing`, `.stall`, `.range`, `.gravity`, `.home` |
| `Lift` | inches (`_in`) | move to a height, or home | it HOLDS | the same, and `.shape` |
| `Flywheel` | rpm of its output | spin up to a speed | it KEEPS SPINNING | `.gearing`, `.stall`, `.atSpeed` |
| `Power` | volts | run until a condition, within a time | nothing: it stops | — (`.stall` optional) |
| `Pneumatic` | named positions | go to a position | its lines keep their levels | `.positions`, `.start`, `.safe`, `.moveTime` |

The common fields are `.name`, `.motors` (or `.positions` for a pneumatic), `.cartridge` (every kind
with motors) and `.safe`.
A field you leave out is UNSET and the build refuses it by name — the library never invents a number
you own (a limit, a preset, the gravity hold, how it homes, what a jam looks like, a valve's move
time).

**The members go in one order per kind** (C++ requires it): what it IS, what it does STOPPED, where
it GOES, the extras.

| Kind | The order |
|---|---|
| `Arm` | `.name`, `.motors`, `.cartridge`, `.gearing`, `.angleSensor`, `.safe`, `.stall`, `.range`, `.gravity`, `.home`, `.presets`, `.unhomedVolts`, `.tuning` |
| `Lift` | `.name`, `.motors`, `.cartridge`, `.gearing`, `.shape`, `.angleSensor`, `.safe`, `.stall`, `.range`, `.gravity`, `.home`, `.presets`, `.unhomedVolts`, `.tuning` |
| `Flywheel` | `.name`, `.motors`, `.cartridge`, `.gearing`, `.angleSensor`, `.safe`, `.stall`, `.atSpeed`, `.presets`, `.tuning` |
| `Power` | `.name`, `.motors`, `.cartridge`, `.pieceSensor`, `.safe`, `.stall`, `.maxVolts` |
| `Pneumatic` | `.name`, `.positions`, `.start`, `.safe`, `.moveTime`, `.pieceSensor`, `.confirmWithin` |

A lift and a flywheel, as a team writes them (`guide-18a`):

```cpp
inline constexpr Lift lift{
    .name = "lift",
    .motors = {Forward, Reversed},        // the right-hand motor is mounted mirrored
    .cartridge = Red,
    .gearing = direct,
    .shape = spool(1.5_in),
    .safe = Hold,
    .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s},
    .range = {0_in, 24_in},
    .gravity = constantGravity(1.0_volt),
    .home = homeBySwitch(0_in, -3_volt),  // down at 3 V until its switch is pressed: that is 0 in
    .presets = {{"floor", 0_in}, {"low", 8_in}, {"high", 22_in}},
};
inline constexpr Flywheel flywheel{
    .name = "flywheel",
    .motors = {Forward},
    .cartridge = Blue,
    .gearing = gears(72, 12),             // geared UP 6:1, so 3600 rpm free
    .safe = Coast,
    .stall = {.amps = 2.0, .belowMotorRpm = 30, .lasting = 0.5_s},
    .atSpeed = within(60, 0.1_s),         // "at speed": within 60 rpm for 0.1 s
    .presets = {{"shoot", 3000}},
};
```

## Units and gearing

- **The gearing is tooth counts, the motor's gear first.** `gears(12, 60)` is a 5:1 reduction — the
  arm turns once for five motor turns; `gears(72, 12)` is geared UP six times; stages multiply
  (`gears(12, 36) * gears(12, 60)`); `direct` is none. The library uses it for two things: to turn the
  motors' encoders into the OUTPUT angle, and to work out the output's FREE SPEED (the cartridge's
  100 / 200 / 600 rpm through the gearing) — the basis of every starting value below and of the
  fastest speed a flywheel accepts (`flywheel.at(4000)` past its free speed does not compile).
- **An arm is in degrees of the arm** (a plain number); **a lift is in inches**, converted to the angle
  of its output shaft by its `.shape`: a `spool(diameter)` (exact), a `linkage(armLength, pivotHeight,
  stages)` (a four-bar or a DR4B: height from the sine of the angle above level), or a
  `measured({{0_in, 0}, {6_in, 30}, …})` table you read off the robot. **A flywheel is in rpm of its
  output.**
- **The loops run on the output angle in degrees** — where gravity's cosine lives — and every gain is
  per OUTPUT degree, a lift's too (per degree of its spool). The limits, presets and "reached" stay in
  the mechanism's own units. A lift's speed and acceleration (inches per second) become degrees by ONE
  scale, its STEEPEST inches-per-degree: exact for a spool; for a linkage its slope at level, for a
  measured table its steepest segment. So a move is never faster in inches than you said anywhere on
  its travel — and slower where the lift is flatter (a linkage near its top): a plan that asked the
  motor for more than it has would be a full-power push, not a smooth move.
- **An angle sensor** (`.angleSensor = rotationSensor()` — `rotationSensor(Reversed)` when it turns the
  other way) replaces the motor encoders as the output angle; on a flywheel it is the speed.

## Limits, and the soft stop

The travel limits (`.range`) are enforced two ways:

1. **A constant target past a limit never compiles** — a preset, `arm.to(170)` on a 0..160 arm.
2. **A target COMPUTED while the auton runs is CLAMPED** to the limit, and said once:

```text
'arm': target 175.00 clamped to the travel limit 160.00 (soft stop)
```

And while a button pushes the mechanism (the driver's hold-to-run, `whileHeld` —
[below](#on-the-drivers-buttons--the-driver-files-controls)), the push is
the driver's own until the mechanism could no longer stop before a limit at the smooth move's own
acceleration. There the **soft stop takes the push over** for as long as it lasts: the move's own loop
brings the mechanism to the limit, and a fresh hold (the same closed loop that holds it anywhere else,
gravity and all) **HOLDS IT AT THE LIMIT**. It never pushes toward the limit harder than the button
does; pushed the other way, it is the driver's again at once; let go, it holds at the limit. One push
is one soft stop, however long the button is held.

**How far past the limit it can go** — measured on the host test plant with the starting values, an
arm with limits 20..140 on a slow, a medium and a fast cartridge (Red, Green and Blue through 5:1:
120, 240 and 720 °/s free), pushed at 4, 8 and 12 V, the plant's arm 1×, 1.5× and 2× as heavy as its
`.gravity` says:

| The arm | Its `.gravity` right | 1.5× as heavy | 2× as heavy |
|---|---|---|---|
| slow (Red 5:1) | 1.08° | 2.08° | 3.50° |
| medium (Green 5:1) | 0.02° | 1.50° | 3.68° |
| fast (Blue 5:1) | 2.50° | 5.41° | **8.32°** |

The worst in every row is a push DOWN onto the bottom limit, and worst of all a heavy arm's: the hold believes it lighter than it
is, so it sags until its integral catches up — and a fast arm arrives with more to stop. It grows with
how fast the mechanism arrives and how wrong `.gravity` is, so **get `.gravity` right first**, then **set
your limits short of the hard stops by more than these** — and by more than a move's own overshoot
(on the host test plant with the starting values: up to 7.22° over 144 moves on Red and Green arms,
3.59° with a plant no faster than its cartridge; 10.27° on the fast Blue arm twice as heavy as its
`.gravity` says, moving down). A hard stop hit at speed is a bent arm.

**At a home end the limit IS the stop.** A mechanism that `restsAtStop(x)` or `homeByStall(x, …)` has
its stop at `x`, and the build refuses a home outside the travel — so that end's limit cannot sit short
of the stop. There the soft stop is what keeps the arrival gentle: on the host test plant (the Green
arm, its stop AT its bottom limit, pushed down at 6, 8 and 12 V — 146 to 252 °/s on the way) it met its
stop at 0.2 °/s with its `.gravity` right, 13.5 °/s 1.5× as heavy, 35 °/s 2× as heavy.

No HOLD ever aims past a limit: a release, a move that ran out of time and a push at a limit are all
clamped to the travel.

## Homing — every moment, and which one to choose

A position mechanism has to know where its zero is before it can go anywhere. **Before it does, a
move is NOT_HOMED** (nothing moves) and a held button pushes it at most `.unhomedVolts` (0, the
default: nothing moves it — no limits are known yet, and the library invents no power for it).

| `.home` | When it finds its zero | What it TRUSTS | Use it when |
|---|---|---|---|
| `restsAtStop(0)` | at program start, no motion | that it is ON its stop when the program starts | it falls onto a hard stop by itself (a dropped arm) |
| `absoluteSensor(0, 180)` | at program start, no motion | that the rotation sensor reads its absolute angle at power-up ([HA-186](../hardware-assumptions.md)) | it has a rotation sensor on its output and moves less than one turn |
| `homeBySwitch(0, -3_volt)` | at program start if the switch is already pressed; otherwise a homing STEP pushes until it is | that the switch reads pressed when pressed ([HA-184](../hardware-assumptions.md)) | there is a limit switch at one end |
| `homeByStall(0, -3_volt, {…})` | a homing STEP pushes until it stalls against its stop | that the FIRST stall is the stop | there is a solid stop and no switch |
| `neverHomed` | never | — | it is only ever pushed by a button (`.unhomedVolts`) |

The `at` is where zero puts it, and a homing push must point at that end (a home at the bottom limit
pushes down — the build refuses the other way). Each one's honest weakness:

- **`restsAtStop` trusts the boot pose.** If the arm was up when the program started — a restart in
  the middle of a match, a hand that lifted it — everything is that far off, and the soft stop holds
  it at a "zero" that is really in the air. A homing step does not fix it (it keeps the boot zero);
  restart with the arm down.
- **`absoluteSensor(at, reads)`** means "the sensor reads `reads` when the mechanism is at `at`" — read
  it off the bench page. The sensor's reading at power-up is within one turn, so the travel must be
  less than one turn of it (a spool lift's sensor turns several times — the build refuses that). The
  library picks the turn that puts the mechanism nearest the middle of its travel, so **the travel plus
  twice the farthest it can ever rest outside a limit must stay under one turn**: an arm with a 160°
  travel is read right from 100° below its bottom limit to 100° above its top; one with a 340° travel,
  only 10° either way — an arm resting 12° past its limit at power-up is believed a whole turn away.
  The build checks the travel, not where the arm rests: that part is yours (the bench page shows the
  reading). Mount it the other way round and say so (`rotationSensor(Reversed)`), or it reads the arm
  on the wrong side of its zero and drives it into its stop — where its jam signature stops it.
- **A home switch that FAILS is never read as pressed** (a read that failed holds its last level; the
  service ignores every read taken while the switch's read-fault count moved). An UNPLUGGED switch is
  different: it reads a level, and whether that level says "pressed" is HA-184 — the bench page reads
  it pressed, released and unplugged.
- **`homeByStall` trusts the first stall.** A piece wedged under the arm is a stop too. That is why
  **every home logs where its zero went**, and a re-home that moves the zero by more than the
  "reached" tolerance is a warning:

```text
'arm': homed -- +25.0 deg from where its encoders read zero at boot
'arm': homed again -- its zero moved -27.0 deg (past its tolerance: was something in the way?)
'wrist': homed -- its angle sensor reads 170.0 deg, so it is at 10.00 deg
```

- **A homing push is gentle, so its stall is SMALL — `homeByStall`'s own signature is not the jam
  recipe below.** A motor pushing at 3 V against its stop draws far less than one jammed at full power:
  on the host test plant about 3/12 of its 2.5 A limit, 0.62 A (the shape of an invented belief about
  V5 motors, [HA-93](../hardware-assumptions.md)). A homing signature of 2.0 A could never trip there —
  every homing step would push for its whole time limit and end TIMED_OUT, never homed. **State the
  homing signature's `.amps` below what the push draws when it stalls** — 0.45 A for a 3 V push to
  start — and check it on the bench. A stall-home that never trips says the most current its push drew
  **when stopped** — no faster than the signature's own speed, the only samples its current could have
  tripped on — so the number is the one to size the signature by. It never counts the moment before the
  push (on the host test plant a jog pinned at 8 V draws 1.67 A there), nor a swing the push brakes
  (1.29 A, at speed):

```text
'arm': TIMED_OUT in 3.01s (it never stalled against its stop: it drew at most 0.62 A when stopped, its homing signature needs 2.00 A)
```

A push that never got that slow says so instead — its time limit ran out before it reached its stop (or
something kept it moving): `(it never stalled against its stop: it never slowed to its homing
signature's speed -- did it reach its stop?)`. Give it longer, or start it closer.

## What a move, a hold and a spin do

**A move** is a smooth trapezoid (speed up, cruise, slow down) to the target — in output degrees, at
the stated speed turned into degrees by the steepest slope (above) — with a P+D loop keeping it on the
plan and feedforward doing most of the work: the volts the speed needs, and the volts
gravity needs (`.gravity`: `cosine(holdLevel, levelAt)` for an arm — the hold is largest when the arm
is level —, `constantGravity(volts)` for a lift). When the plan ends a fresh HOLD loop (P+I+D) takes
over; its integral corrects a gravity number that is a little wrong (on the host test plant, an arm 25 %
lighter than its `.gravity` says held level within 0.001° — 3.20° off without the integral).

**REACHED** = within the tolerance, in the mechanism's own units (1.5° or 0.25 in to start), for the
settle time (0.1 s) without leaving it — an arm that rings in and out starts that clock again each time
it comes back: **SUCCEEDED, and it goes on HOLDING there.** Out of time, the step says
**TIMED_OUT** and the mechanism **HOLDS WHERE IT STOPPED** — it never keeps pushing toward a target it
could not reach, because a blocked arm would then sit at stall current until something else stopped
it. Before homing: **NOT_HOMED**. A jam: **STALLED** (below). The verdicts are the ones every
mechanism step uses ([chapter 11](11-reading-the-diagnostics.md)).

**What "held" costs.** A hold is the motor working: it draws what the load needs, every tick, for as
long as the service runs. Driver control starting, an auton starting and the end of a guarded run each
drop it to the mechanism's `.safe` state — an arm with `.safe = Hold` then rests on the motor's own
Hold, with no gravity term and no integral (whether that alone keeps a LOADED arm up is
[HA-92](../hardware-assumptions.md), unmeasured).

**A spin** speeds the flywheel to the target with feedforward (most of the volts) and P on the error;
an integral that acts only near the target (within 10 %) trims the last few rpm — from the start it
would wind up and overshoot (on the host test plant: 233 rpm with it from the start, 31 with the
window). **"At speed"** (within the `.atSpeed` band for its time) is **SUCCEEDED, and it keeps
spinning** until something stops it. Out of time it is **TIMED_OUT and still spins** — a wheel that
cannot reach the speed is not a fault; the auton decides what to do about it.

## Jam signatures — what a stall looks like, and the recipe

An `Arm`, a `Lift` and a `Flywheel` keep working after their step, so each must say what a jam looks
like: `.stall = {.amps = …, .belowMotorRpm = …, .lasting = …}` — at least that current while the MOTOR
shaft turns slower than that, for that long. **The service watches it on everything the mechanism
does**: the move, the hold after it, a button's push, a homing push by switch, a spin before and after
"at speed". A trip is **STALLED**, a `MECHANISM_STALLED` fault, the `.safe` state and the motor let go —
and a driver's button that was held is ignored until it is released ([below](#on-the-drivers-buttons--the-driver-files-controls)).

**The recipe, before you measure one:** a bit below the motor's 2.5 A limit (2.0 A), a LOW motor speed
(5 rpm for an arm or a lift; a flywheel's 30), about half a second. (Not for a homing push: its own
signature is far smaller — the homing section above.) Then look at what your mechanism
really draws: a hold that squeezes on purpose (an arm pressing a goal down) trips it too — state a
higher `.amps` or a longer `.lasting` for that one. The build refuses a signature that could never
trip: 2.5 A or more (a limited motor never reads more), 0 rpm, or no `.lasting`.

On the host test plant: a move blocked at 60° ended STALLED 1.00 s after it began (the recipe's
numbers); a hold loaded far past what it can carry stopped once the load had dragged it to its stop
(1.66 s at stall current of a 30 s test — without the watch it sat there for all 30); a jammed flywheel
stopped 0.51 s in; a button pushing an arm into an obstacle at **12 V**, 0.95 s in.

**A gentler push may never trip it.** That last number is a full-power push. A hold-to-run at less
power stalls at less current: on the host test plant a stalled motor draws its volts' share of 2.5 A,
so a **6 V** push into something draws about **1.25 A** — below the recipe's 2.0 A. It never trips, and
only letting go of the button ends it. If your driver file holds a mechanism at less than full power,
either state its `.amps` below what that hold draws when it is stopped (measure it on the bench), or
accept that the driver's thumb is what ends a push into an obstacle.

## Tuning — the starting values, and how to change them

Three kinds of number are not yours to know before the robot runs: the loops' gains, the smooth
move's speed and acceleration, and "reached". The library starts them from your cartridge and gearing
— **invented and provisional** (HA-182). `startingValuesLine()` words them for a boot line, so nobody
takes them for measurements; every program that boots a robot's list prints it, one line per position
or speed mechanism (since the core's fifth part — robot two declares neither, so its boot prints none):

| Number | Starting value | What it is |
|---|---|---|
| `kP` | 0.25 V per degree | the push per degree of error |
| `kI` | 0.3 V per degree-second | the hold's correction for a wrong gravity number |
| `kD` | 0.004 V per degree/s | damping |
| `kV` | 12 V over the output's free speed | the volts a speed needs |
| `kS`, `kA` | 0 | friction and acceleration feedforward |
| `maxSpeed` | 75 % of the free speed | the smooth move's cruise |
| `maxAcceleration` | reaching the cruise in 0.25 s | its ramp |
| `within`, `settle` | 1.5° (0.25 in), 0.1 s | "reached" |
| a flywheel's `kV`, `kP` | 12 V over its free speed, 0.05 | per output rad/s |
| a flywheel's `kI` | 0.2 (acting near the target only) | per output rad (the speed error's integral) |
| a flywheel's `kS` | 0 | volts |

```text
arm: starting values kP 0.250 kI 0.300 kD 0.0040 kV 0.0500, cruise 180.0 deg/s in 0.25 s, reached within 1.50 deg for 0.10 s -- PROVISIONAL (HA-182); .tuning overrides
```

`.tuning` replaces any of them, one by one — whatever it leaves out keeps its starting value (and the
boot line marks the ones you stated "(tuned)"; a stated `maxSpeed` alone keeps the starting 0.25 s
ramp, so the ramp is not marked):

```cpp
    .tuning = {.kP = 0.3, .within = 2.0},
```

Three more of the library's numbers are NOT in that table and NOT in `.tuning`: the hold's integral
clamp (3 V), and the flywheel integral's window (within 10 % of the target) and clamp (3 V). They are
the library's constants for every mechanism (HA-182 too) and are not on the boot line; if one turns out
wrong on a real robot, the fix is a library change, not your declaration's.

The order that works: **get `.gravity` right first** (with the arm level, the volts that hold it still
— the integral covers a little error, not a wrong number); then the speed (`maxSpeed`, lower it if a
move overshoots past what your limits allow); then `kP` (more if it stops short, less if it rings);
then `within` (a mechanism that cannot settle inside 1.5° needs a wider tolerance or a better hold).
On the host test plant the starting values reached 143 of 144 deliberately mismatched moves — the one
miss was a plant slower than its own cartridge, reported TIMED_OUT, as it should be. A real arm is not
the host test plant: tune on the robot.

## In an auton — one line per step

An auton builds its routine from its context — `Routine r{ctx, "name"}` — and every mechanism step is
then ONE line, naming the mechanism by its declaration and stating its time limit. The auton below is
robot x's (`guide-18c`, in
[`test/mechanism_auton_example_test.cpp`](../../test/mechanism_auton_example_test.cpp); the robot's own
namespace gives every word — robot two's claws in an auton are
[Chapter 16](16-testing-an-auton-on-the-bench.md#the-claws-in-an-auton)'s):

```cpp
RoutineResult scoreThree(AutonContext& ctx) {
    Routine r{ctx, "score-three"};
    r.startSpinUp(flywheel["shoot"], 2_s);         // the flywheel spins up while the robot drives
    r.startRun(intake, 12_volt, seesPiece.orAfter(2.5_s), 3_s);  // until it sees a piece (or 2.5 s)
    r.driveTo(24_in, 0_in);
    r.waitUntilDone(intake, 1_s);
    r.move(arm["score"], 1.5_s);                    // the arm HOLDS 95 degrees from here on
    r.move(lift.to(18_in), 4_s);
    r.set(hood["up"]);
    r.waitUntilDone(flywheel, 1_s);
    r.run(intake, 12_volt, 0.5_s);                  // feed the piece into the flywheel
    r.stop(flywheel);
    return r.result();
}
```

| Step | What it does | Its verdict |
|---|---|---|
| `r.move(arm["score"], 1.5_s)` | an arm or a lift to a preset, a constant (`arm.to(120)`, `lift.to(18_in)` — past a limit, no build) or a computed target (`arm.clampedTo(x)` — clamped, logged) | REACHED → SUCCEEDED, and it holds there |
| `r.home(lift, 1.5_s)` | an arm or a lift finds its zero by its declaration's method | SUCCEEDED, holding there |
| `r.run(intake, 12_volt, seesPiece, 3_s)` | a power mechanism at a voltage until a condition — `seesPiece`, `stopsSeeingPiece`, `seesPiece(grip)` (ANOTHER mechanism's sensor), `stalls`, `seesPiece.orAfter(2.5_s)` — or for a time: `r.run(clawPivot6, 12_volt, 0.4_s)` | the condition → SUCCEEDED; then its `.safe` state |
| `r.set(claw["closed"])` | a pneumatic to a named position — no time limit: its `.moveTime` is the robot file's | SUCCEEDED; with no sensor to confirm a `grabs` position, UNCONFIRMED, counted done |
| `r.spinUp(flywheel["shoot"], 2_s)` | a flywheel to a named speed or `flywheel.at(3000)` | AT SPEED → SUCCEEDED, and it keeps spinning |
| `r.stop(intake)` / `r.stop(flywheel)` | its `.safe` state, now | SUCCEEDED (ABSENT or REFUSED when it could not) |

**The START forms** — `startMove`, `startHome`, `startRun`, `startSpinUp` — command and return at once, so
the next line (a drive) runs WHILE the mechanism works; `r.waitUntilDone(intake, 1_s)` collects that
step's verdict later (at once if it is already over). On the host test plant robot x's arm held 95° within
1.16° through a 1.8 s, 40 in drive, its intake — started before the drive — stopped on its sensor at
T+2.00 s while the drive was still running, its lift and flywheel reached 9.7 in and 3000 rpm during
the same drive, and the service ticked exactly once per pace (180 paces, 180 ticks).

**The verdicts, and what each does to the chain:** SUCCEEDED goes on; **UNCONFIRMED counted done** — no
sensor exists to confirm it, like robot two's sensorless claw — goes on (the team lead's ruling);
**every other verdict STOPS the chain** — TIMED_OUT, STALLED, NOT_HOMED, ABSENT, REFUSED, CANCELLED, or
an UNCONFIRMED NOT counted (a piece sensor that stopped answering may have missed the very piece the
step waited for). Stopping is the routine's usual policy: the drive safed, every later step skipped,
one line naming the verdict —

```text
routine 'score-three' STOPPED at step 4 (intake): mechanism step FAILED (TIMED_OUT) — skipping the rest; drive safed
```

— and the verdict is also `r.result().mechanismVerdict` (cause `MechanismStepFailed`) and
`r.lastMechanism()`. A START form stops the chain only when its START failed (ABSENT, REFUSED,
NOT_HOMED); its later verdict is `waitUntilDone`'s. A step on a mechanism this robot does not have is
ABSENT and says so (`'claw' is not on this robot (its wiring list does not have it): the step did
nothing`) — autons are per robot. After its step a mechanism KEEPS its behaviour (an arm holds, a
flywheel spins) until another command, the end of the run, or driver control.

**Mistakes, and where they are caught.** The wrong step for a kind fails the build at your line, with a
sentence: `r.run(arm, …)` ("r.run(...) is for a POWER mechanism -- move an arm or a lift with
r.move(...)…"), `r.run(intake, 12, 1_s)` ("volts are written with their unit: 12_volt, not 12"),
`r.move(intake, …)` or `r.move(arm, …)` ("r.move(...) takes a TARGET…"), `r.stop(arm)` ("r.stop(...) is
for a power mechanism or a flywheel…"). A condition that can never be met — `seesPiece` on a mechanism
that declares no piece sensor, `stalls` with no `.stall` — throws at the step, before anything moves;
the host test of every auton runs each step against the robot's real list, so it fails the suite
before an upload. A routine built the old way, `Routine r{ctx.chassis, …}`, runs its drive steps as
ever, and its first mechanism step throws, naming the fix (`Routine r{ctx, "name"}`). If the program
forgot to put the service in the chassis's pacer chain, the first blocking mechanism step stops the
chain saying `the mechanism service never ticked -- it is not in the chassis pacer's chain`.

## On the driver's buttons — the driver file's controls

A driver file states its mechanism controls as ONE list, checked at build against the robot's list
(`guide-18d`):

```cpp
inline constexpr DriverControls kControls = controls(kRobotXMechanisms,
    whileHeld(arm, Button::R1, Button::R2, 6_volt),  // up / down while held; let go: it holds there
    preset(arm["stow"], Button::Down, 1.5_s),
    preset(arm["score"], Button::Right, 1.5_s),      // the most recent button wins
    preset(arm["high"], Button::Up, 1.5_s),
    whileHeld(lift, Button::L1, Button::L2, 12_volt),
    homeAtDriverStart(lift, 1.5_s),                  // after a restart it may not be on its switch
    toggle(flywheel["shoot"], Button::X),            // on at "shoot" / off
    whileHeld(intake, Button::A, Button::B, 12_volt),
    autonOnly(hood));                                // the hood is the autons' alone
```

- **`whileHeld(m, forward, back, volts)`** — a power mechanism runs while a button is held and goes to its
  `.safe` state when let go; an arm or a lift JOGS while held (inside its limits once homed; at most
  its `.unhomedVolts` before) and HOLDS WHERE IT IS when let go. Both buttons at once = neither.
- **`toggle(claw, Button::A)`** — each press flips a two-position pneumatic; **`toggle(flywheel["shoot"],
  Button::X)`** spins a flywheel up to that NAMED speed or stops it (`toggle(flywheel, …)` is refused:
  name the speed). A toggle's press, a preset's and a homing press are each a STEP, as an auton's is, so
  each logs its verdict once it ends — robot two's claw, 0.5 s after each press of A: `'claw': SUCCEEDED
  -- time in 0.50s` for an open, `[WARN]` `'claw': UNCONFIRMED -- time in 0.50s (no piece sensor to
  confirm it; counted done)` for a close (nothing senses the grab), and `CANCELLED` when a newer press or
  the end of driver control came first. Those lines are expected; a hold-to-run (`whileHeld`) logs
  none.
- **`preset(arm["score"], Button::Right, 1.5_s)`** — a press sends it there within the time limit, then it
  holds (or keeps spinning); a pneumatic position needs no time limit: `preset(hood["up"], Button::Up)`.
- **`homeOnPress(lift, Button::Left, 1.5_s)`**, **`homeAtDriverStart(lift, 1.5_s)`** (once, at the start of
  driver control, when nothing has homed it — skipped when the auton did; its mechanism's own buttons wait
  while it runs — "the one exception", below), and **`autonOnly(hood)`** (no
  control, on purpose).

**The most recent input wins.** A preset pressed while a hold-to-run is still held takes over, and the
held button then commands NOTHING until you let it go — it never fights the preset back. The same after
a stall: a held button whose mechanism tripped its jam signature is ignored until released (a push
gentler than full power may never trip it — [the jam section](#jam-signatures--what-a-stall-looks-like-and-the-recipe)).
**A press must be SEEN** — for a toggle, a preset and `homeOnPress`: its button already down when driver
control starts, or still down when a dropped controller comes back, is not a press, and **a drop itself
never presses anything** (robot two's claw stays as it is) — provided the controller's "connected"
signal and its buttons come back together, which nobody has measured
([HA-147](../hardware-assumptions.md)); until it is, a driver whose controller drops lets go before
pressing again. A hold-to-run is a level, not a press: its
button already down when driver control starts drives at once, and a controller drop releases every
hold-to-run that tick.

**The one exception — the driver-start home.** While a `homeAtDriverStart` home is running (at most its
time limit), that mechanism's OWN buttons wait: a hold held during it does nothing until you let go
after the home, and a preset or `homeOnPress` press during it is dropped. Without that, a driver
touching the lift in the first second and a half would cancel the very home that makes its buttons
work. Every other mechanism's buttons work as usual meanwhile.

**What the build refuses, by name:** a button doing two jobs (`button_Right_does_two_jobs`); a control
on a mechanism the robot does not have; a hold's volts outside (0, 12]; a preset or a homing with no
time limit; two controls of one kind on a mechanism (a hold beside presets is fine); a toggle on
anything but a two-position pneumatic or a flywheel's named speed; a homing control on a never-homed
mechanism; a mechanism with no control and not marked `autonOnly`, or marked and controlled; more than 24
controls; and — the rule that saves a match — **an arm or a lift homed by its switch or by a stall,
held or preset, with no way to home it in driver control**
(`a_mechanism_that_must_be_homed_has_no_way_to_home_in_driver_control`): after a program restart that
leaves it off its home it is not homed, its holds would command its `.unhomedVolts` (0 = nothing) and
its presets end NOT_HOMED — dead buttons for the whole match. Add `homeAtDriverStart` or `homeOnPress`.
The declaration's `.unhomedVolts` is enough only for a mechanism that is HELD and never preset: its
holds then move it, unhomed, at those volts — but a hold never homes it (nothing zeroes a mechanism
but its boot and a homing step), so a preset beside it would end NOT_HOMED all match, and needs one of
the two homing controls. (A never-homed mechanism held with no `.unhomedVolts`, or preset at all, is
refused the same way.) The program checks the list once more against the robot its service is
bound to — a driver file built for another robot's same-named mechanism never commands a stranger.

**What the loop does with them.** `teleop::runDriverControl`'s mechanism-service overload runs them:
at the start of driver control the service drops whatever autonomous left (a leftover claim released
and said, motor mechanisms to their `.safe` state, a pneumatic sent its `.start` on the program's FIRST
entry only), and each homeAtDriverStart homes; every tick, after the drive, your buttons command the
service and the service ticks ONCE — before your panel's `perTick`, so a panel sees this tick's
command; on a field disable every motor mechanism goes to its `.safe` state and a pneumatic keeps its
level ([HA-157](../hardware-assumptions.md): whether the brain keeps a 3-wire output's level while
disabled is unmeasured). The drive protections are unchanged. One driver path per mechanism: the loop refuses T3's older
bindings beside the service, and a control on its re-zero button.

## As route markers — every mechanism's ids, run

Every mechanism's actions are route ids a route marker names — derived from its declaration, never
typed ([chapter 10](10-the-api.md#route-ids--the-actions-every-mechanism-publishes-automatically)). A
program registers ALL of a robot's ids in one call (`guide-18e`):

```cpp
static runner::CommandRegistry registry;
static runner::MechanismRoutes routes{chassis, service, kRobotXMechanisms, telemetry};
routes.registerInto(registry);
```

A marker runs EXACTLY the step an auton's line runs — `arm_score [1500]` is `r.move(arm["score"],
1.5_s)`, `intake_until_piece [12, 3000]` is `r.run(intake, 12_volt, seesPiece, 3_s)`, `claw_closed` is
`r.set(claw["closed"])` — and **blocks at its waypoint until the step's verdict** (markers fire only
after a leg settles, so a marker never runs while driving — a long time limit is a long stop there).
SUCCEEDED and a counted UNCONFIRMED let the route go on; any other verdict stops it. The args are the
route author's: an arm's, a lift's or a flywheel's id carries its **time limit in ms**; a power
mechanism's `_run`, `_until_piece` and `_until_gone` carry **volts (−12..12, not 0) and a time in
ms**; a `_stop` and a pneumatic's position carry none. A marker missing one is REFUSED and the route
stops, nothing commanded:

```text
route marker 'arm_score' REFUSED: it needs its time limit in ms, above 0 (args[0])
```

Each id is published with a one-line doc for a marker picker — `"claw pivot 6: run; args[0] = volts
(-12..12), args[1] = how long, ms"` (the manifest:
[the VexBuilder contract §4.2.3](../vexbuilder-integration.md#423-the-manifest--ask-4-and-it-now-exists)).
On the host test plant robot x's route of seven generated markers ran every one to SUCCEEDED inside a
real auton, and robot two's sensorless grab went UNCONFIRMED, counted done, with its route going on.

## The direction check — what it judges

Two cables of the SAME kind swapped between mechanisms, a motor declared the wrong way, or an angle
sensor mounted backwards all look fine at boot. The direction check (its bench page is the core's last
part) jogs ONE mechanism a little and asks you ONE question — "ONLY 'arm' should have moved: did it go
its + way, the OTHER way, or did it NOT move (or something ELSE moved)?" — and
`mechanism::judgeDirection` (or `judgePneumatic`) turns the encoders' readings and your answer into the
line to change: the declaration's for a direction (`'arm' moved the OTHER way: in the declaration at
…:14, .motors: swap EVERY Forward and Reversed (and the angle sensor's), then rebuild and upload`), the
`wire(...)` line for a cable on another mechanism's port, a generated robot file's JSON path when it
states one (and then the value to write there, and "regenerate, rebuild and upload"). What the
encoders see on their own: a motor turning AGAINST its command while a mate turns WITH its own (the
mate drags it — that motor's word is wrong), every motor still while every one draws (a fight of
equals, or a jam), a motor that neither moved nor drew (its cable), an angle sensor reading against its
motors — and every motor that moved going AGAINST its command, one motor or all of them alike: the
mechanism was **back-driven** by something stronger than the jog (gravity, a load), no declaration is
wrong, and none is named — jog it harder or from another pose. A V5 motor reverses its encoder with its
command, so its own encoder cannot see it declared the wrong way — your answer does. It REPORTS; it
saves nothing: you edit the robot file, rebuild and upload.

## The honest limit — the service runs only when something ticks it

The service runs ONLY inside shulib's own waits, motions and pauses — and every tick of the driver loop.
**A busy loop of your own starves it** — a `pros::delay` in an auton, a `then()` action that blocks — and
a starved service leaves every mechanism at its LAST command: not held, not limited, not watched for a
jam (on the host test plant, an arm left at 11.74 V for 1.5 s sat against its hard stop at 2.45 A). In
an auton, wait with `r.pause(…)` or `r.waitFor(…)`, never with `pros::delay`, and never let a `then()`
action block.

## Reading a refusal

Chapter 10's [section](10-the-api.md#what-the-build-refuses-and-how-to-read-a-refusal) is the
lesson: the reason is the name of a function GCC refuses to call, and your line is the first "in
'constexpr' expansion of" note above it. A wrist homed by an absolute sensor over a whole turn
(`.range = {-180, 180}`) reads like this (GCC 13, trimmed):

```text
your_robot.hpp:7:9:   in 'constexpr' expansion of 'shulib::mechanism::wire<...>(wrist, ...)'
.../shulib/mechanism/wiring.hpp:...:   in 'constexpr' expansion of '...refuse(...specRefusal(...))'
.../shulib/mechanism/spec.hpp:...: error: call to non-'constexpr' function
    'void shulib::mechanism::refused::position_absolute_homing_needs_a_travel_shorter_than_one_turn_of_its_angle_sensor()'
```

Line 7 is the `wire(wrist, …)` line — a declaration's own mistake points at its `wire(...)` line; the
reason names the field to look at (`.range` here: a whole turn). The refusals that belong to this
chapter:
`position_a_preset_is_outside_the_travel_limits`, `position_the_homing_push_points_away_from_its_home_end`,
`position_the_stall_signature_is_unset_a_held_mechanism_must_never_sit_at_stall_current`,
`speed_the_stall_signature_is_unset_a_jammed_wheel_must_never_sit_at_stall_current`,
`a_stall_signatures_amps_must_be_below_the_motors_2_5_A_current_limit`, `speed_is_past_the_free_speed`,
`target_is_outside_the_travel_limits` and — new with the position behaviours —
`position_absolute_homing_needs_a_travel_shorter_than_one_turn_of_its_angle_sensor`. The driver
file's (above) point at its `controls(` line — a button twice names the button — and the auton's wrong
steps are `static_assert` sentences ("required from here" names your line).
