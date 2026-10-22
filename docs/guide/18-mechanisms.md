# 18 — Mechanisms: arms, lifts and flywheels

> **Read [chapter 10's "Mechanisms" section](10-the-api.md#mechanisms--declared-once-wired-separately)
> first** — it shows how a robot file declares a mechanism and wires it, and how to read a refusal.
> This chapter is what happens AFTER the declaration: how an `Arm`, a `Lift` and a `Flywheel` move,
> hold, find their zero, stay inside their limits and stop when something jams — and how to tune
> them.
>
> **What exists, and what does not yet.** The mechanism core's first three parts are in (MECH-A1 and
> A2, 2026-10-15; A3, 2026-10-22): the vocabulary and its build checks, the mechanism SERVICE (one
> object, ticked once per control tick, that runs every mechanism's behaviour), and the behaviours of all four kinds
> of mechanism — power, pneumatic, POSITION (an arm, a lift) and SPEED (a flywheel). The auton steps
> that call them (`r.move(arm["score"], 1.5_s)`), the driver controls (presets on buttons), the
> route markers and the bench page are the core's next parts, and **no program builds a service
> yet**. Everything below is measured on the **host test plant** — invented models of an arm, a lift
> and a flywheel ([HA-183](../hardware-assumptions.md)) — and the library's own numbers in it are
> invented starting values ([HA-182](../hardware-assumptions.md)). **Nothing here has run on a
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

And while a button pushes the mechanism (the driver's hold-to-run, the core's next part), the push is
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
and, from the core's driver controls on (its fourth part), a button that was held is ignored until it
is released.

**The recipe, before you measure one:** a bit below the motor's 2.5 A limit (2.0 A), a LOW motor speed
(5 rpm for an arm or a lift; a flywheel's 30), about half a second. (Not for a homing push: its own
signature is far smaller — the homing section above.) Then look at what your mechanism
really draws: a hold that squeezes on purpose (an arm pressing a goal down) trips it too — state a
higher `.amps` or a longer `.lasting` for that one. The build refuses a signature that could never
trip: 2.5 A or more (a limited motor never reads more), 0 rpm, or no `.lasting`.

On the host test plant: a move blocked at 60° ended STALLED 1.00 s after it began (the recipe's
numbers); a hold loaded far past what it can carry stopped once the load had dragged it to its stop
(1.66 s at stall current of a 30 s test — without the watch it sat there for all 30); a jammed flywheel
stopped 0.51 s in; a button pushing an arm into an obstacle, 0.95 s in.

## Tuning — the starting values, and how to change them

Three kinds of number are not yours to know before the robot runs: the loops' gains, the smooth
move's speed and acceleration, and "reached". The library starts them from your cartridge and gearing
— **invented and provisional** (HA-182). `startingValuesLine()` words them for a boot line, so nobody
takes them for measurements; the programs print it from the core's fifth part on — none does yet:

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

## The honest limit — the service runs only when something ticks it

The service runs ONLY inside shulib's own waits, motions and pauses — and, from the core's driver
controls on (its fourth part), every tick of the driver loop. **A busy loop of your own starves it** — a `pros::delay` in an auton, an action that blocks — and
a starved service leaves every mechanism at its LAST command: not held, not limited, not watched for a
jam. In an auton, wait with the routine's own pause; never with `pros::delay`.

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
`position_absolute_homing_needs_a_travel_shorter_than_one_turn_of_its_angle_sensor`.
