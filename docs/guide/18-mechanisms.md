# 18 — Mechanisms: arms, lifts, flywheels — in autons, on buttons, on routes

> **Read [chapter 10's "Mechanisms" section](10-the-api.md#mechanisms--declared-once-wired-separately)
> first** — it shows how a robot file declares a mechanism and wires it, and how to read a refusal.
> This chapter is what happens AFTER the declaration: how an `Arm`, a `Lift` and a `Flywheel` move,
> hold, find their zero, stay inside their limits and stop when something jams, and how to tune
> them — then how an AUTON runs a mechanism (one line per step), how the DRIVER's buttons do
> (presets, holds, toggles), how a ROUTE's markers do (every mechanism's ids, published), and what
> the direction check judges — and the BENCH PAGE that runs it on the robot.
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
> And since the sixth and last part (MECH-A6, built 2026-10-30), Bench Tests' **station 13 MECHANISMS**
> — the bench page ([below](#the-bench-page--bench-tests-station-13)) — shows every declared mechanism in
> its declared units and runs the direction check on the robot. Since then, **current caps** (`.currentCap` —
> [below](#current-caps--how-hard-a-string-is-pulled)): the most each motor of a mechanism may draw, set in the
> motor itself. And **live tuning's library** ([below](#or-on-the-robot-live-tuning)): a mechanism's numbers changed
> while it runs, inside the build's own rules, undone, and copied back out as the lines its robot file needs —
> on the brain screen through **"shulib Tune"** ([below](#the-tune-program--tuning-on-the-brain-screen)). Still to come: a
> watch for a snapped string. Everything below is
> measured on the **host test plant** — invented models of an
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
with motors) and `.safe`. Every kind with motors may also state a `.currentCap` — the most each motor
may draw ([current caps](#current-caps--how-hard-a-string-is-pulled), below); it is never required.
A field you leave out is UNSET and the build refuses it by name — the library never invents a number
you own (a limit, a preset, the gravity hold, how it homes, what a jam looks like, a valve's move
time).

**The members go in one order per kind** (C++ requires it): what it IS, what it does STOPPED, where
it GOES, the extras.

| Kind | The order |
|---|---|
| `Arm` | `.name`, `.motors`, `.cartridge`, `.gearing`, `.angleSensor`, `.safe`, `.stall`, `.currentCap`, `.range`, `.gravity`, `.home`, `.presets`, `.unhomedVolts`, `.tuning` |
| `Lift` | `.name`, `.motors`, `.cartridge`, `.gearing`, `.shape`, `.angleSensor`, `.safe`, `.stall`, `.currentCap`, `.range`, `.gravity`, `.home`, `.presets`, `.unhomedVolts`, `.tuning` |
| `Flywheel` | `.name`, `.motors`, `.cartridge`, `.gearing`, `.angleSensor`, `.safe`, `.stall`, `.currentCap`, `.atSpeed`, `.presets`, `.tuning` |
| `Power` | `.name`, `.motors`, `.cartridge`, `.pieceSensor`, `.safe`, `.stall`, `.currentCap`, `.maxVolts` |
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
signature is far smaller — the homing section above.) **Under a `.currentCap` the 2.0 A is wrong:** a
capped motor never draws more than its cap, so the signature must sit below the cap — MORE than 0.2 A
below it ([the next section](#current-caps--how-hard-a-string-is-pulled) says why and how). Then look at
what your mechanism really draws: a hold that squeezes on purpose (an arm pressing a goal down) trips
it too — state a higher `.amps` or a longer `.lasting` for that one. The build refuses a signature that
could never trip: 2.5 A or more (a limited motor never reads more), at or above the mechanism's
`.currentCap`, 0 rpm, or no `.lasting`.

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

## Current caps — how hard a string is pulled

A motor's current IS its pull: the motor turns current into torque, and a spool turns that torque into
string tension — so "how many amps" is "how hard the string is pulled". A string pulled too hard snaps.
`.currentCap` tells each of the mechanism's motors the most it may draw (`guide-18g`):

```cpp
inline constexpr Lift stringLift{
    .name = "string lift",
    .motors = {Forward, Reversed},
    .cartridge = Red,
    .gearing = direct,
    .shape = spool(1.5_in),
    .safe = Hold,
    .stall = {.amps = 0.9, .belowMotorRpm = 5, .lasting = 0.5_s},  // below the cap, with room
    .currentCap = {.amps = 1.2},                                    // each motor: at most 1.2 A
    .range = {0_in, 24_in},
    .gravity = constantGravity(1.0_volt),
    .home = restsAtStop(0_in),
};
```

**The motor enforces it, not the library.** At boot the library writes the cap into every motor of the
mechanism and reads it back; from then on the MOTOR holds it — every command, the Hold brake, a homing
push up or down, and even while a blocking loop of yours stops the library from running. (That a real
motor holds its limit this way, and how fast, is a belief nobody has measured yet — Hardware Assumption
HA-219.) The boot says it once:

```
'string lift': current cap 1.20 A per motor (2 motors: 2.40 A in all) -- set and read back
```

**It is PER MOTOR.** Two motors on one spool pull with twice the cap — the line says the total. **No cap
written means the motor's own 2.5 A**; the library never invents one. A cap written as `0` means the same:
no cap. The build refuses a cap below 0, above 2.5 A, or under 0.001 A (a motor holds its limit in whole
milliamps, so a smaller one would be sent as 0 mA — no torque at all).

**Four currents must stay in ORDER**, or the cap and the jam detector fight each other:
the heaviest **hold** (with a game piece) < the stall signature < the cap − 0.2 A; the heaviest
**move's peak** < the cap; and the cap ≤ what the string can safely take.

- **The signature below the cap.** A capped motor never draws more than its cap, so a signature at or
  above it could never fire — the build refuses it. A signature just under the cap may never fire
  either: a real capped motor may read a little under its cap (believed within about 0.1 A —
  HA-223, not measured), so the boot warns when one is not MORE than 0.2 A below it (exactly 0.2 A
  below warns too). It judges as the motor reads, in whole milliamps: a signature trips at the first
  whole milliamp at or above it, so under a 1.2 A cap a 0.9994 A signature counts as 1.000 A and warns,
  and 0.999 A does not. Type a signature in whole milliamps (three decimals) and the arithmetic is the
  plain one.
- **The hold below the signature.** The jam detector watches the hold too. On the host test plant a
  lift capped at 1.2 A with a 1.0 A signature (the boot warned: exactly 0.2 A under the cap) held a 3 V
  lift for 12 s at 0.56 A without a stall — then a 2.5 V game piece raised its hold to the cap's 1.2 A,
  above the signature, and the hold ended **STALLED**, the piece and all. Measure the hold WITH your
  heaviest piece before you choose the signature.
- **The move's peak below the cap.** Too low a cap also limits how hard the motor can accelerate:
  moves slow down, run out of time, or stall at their start.
- **The cap below the string's limit.** The string's safe pull divided by the pull per amp — about
  10 lbf per amp per motor for a red cartridge on a 1.5 in spool, by the published motor numbers
  (HA-222, not checked on this robot) — divided by your safety margin.

**Where the hold and the peak come from.** The Tune program's TEST report
([below](#the-tune-program--tuning-on-the-brain-screen)): each move's **peak** and its **hold** — the mean
current over the second after it settled — with a WARNING when the stall signature is not above that
hold or the cap not above the peak. Run each TEST with and without the heaviest piece the lift carries,
and choose the numbers from the four currents in the order above. Until you have run it, treat your
numbers as unmeasured.

**What it cannot limit:** the carriage's and the motor's momentum when the lift hits a stop pulls on the
string with no current at all (HA-222). Push into stops gently. And it does not see a string that has
already snapped — today a snapped string is invisible to the library (the lift reports its moves done
with the carriage on the floor); a watch for that is a later part.

**If a motor will not take its cap,** the boot warns, naming the port and both numbers
(`'lift': motor 2 (port 12) did not take its current cap: asked 1.20 A, the write was refused, reads back
2.50 A -- it runs; that motor is not capped as stated`), and the lift still runs. **Once a second** the
library reads each capped motor's limit again: one that reads 2.5 A again (it lost power and came back
at its default — HA-220) is written again, with one warning; one that reads any other number was set by
something else and is warned about once and left alone. **A mechanism with no cap** is only read at
boot, never written: if its motor's limit reads anything but 2.5 A — a cap an earlier program left
(HA-220), or VEXos sharing current when more than 8 motors are plugged in (HA-221) — one line says so,
and the library changes nothing. [Chapter 11](11-reading-the-diagnostics.md) lists every one of these
lines.

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

### Or on the robot: live tuning

The numbers above are found on the ROBOT, not at a desk — so a mechanism's numbers can also be changed
**while the robot runs**, its response watched, and the result copied back into the robot file. The
library half is `mechanism/live_tuning.hpp`; the program that puts it on the brain screen is **"shulib
Tune"** ([the next section](#the-tune-program--tuning-on-the-brain-screen)) — what follows is what it
does underneath, and how a program of your own would call it. Nothing here has run on a robot.

**How it is wired.** A program keeps a `mechanism::LiveTuning` — a COPY of the robot file's table, in
static storage beside its service — and runs the mechanism service over the copy:

```cpp
mechanism::LiveTuning tuning{kRobotTwoMechanisms};  // a COPY of the robot file's list
service.bind(tuning.table(), devices, clock, faults, sink);  // the service runs the copy
tuning.attach(service);  // checked: the service runs THIS copy
```

`tuning.step(i, Tunable::KP, +1)` is one [+] press on mechanism `i`'s kP; `tuning.apply(i, t, value)` sets
a number outright. Either one is checked, applied, and the mechanism's slot re-reads what it copied (its
loops' gains, its stall detectors, its limits in degrees, its current cap), with one line:
`'lift': retuned live -- kP 0.25 -> 0.3125`. **The robot file is never written:** leaving the program or
a reboot runs the robot file's numbers again, and every program but the one that tunes always does —
with **one exception, the current cap:** a cap changed live is written into the motors, and a motor
keeps a limit after the program that set it ends (believed — the team has not measured it), so the
next program reports the limit it finds but, if its robot file states no cap, does not undo it.
**Before leaving, `tuning.reset(i)` every mechanism whose cap you changed** (or undo back to the file's
cap): that writes the robot file's cap back — or the motor's 2.5 A default when the file states none.

**What is live.** An Arm or a Lift: its ten `.tuning` numbers, its gravity hold, its stall signature, its
homing push and homing signature (when it homes by switch or by stall), its current cap, and its travel
limits — **narrowed only**. A Flywheel: kS, kV, kP, kI, its at-speed band and time, its stall signature,
its cap (no kA — the robot file cannot say a flywheel's). A Power mechanism: its maxVolts, its stall
signature (if it states one) and its cap. **Not live:** presets (an auton finds a preset by its VALUE —
change "high" live and `lift["high"]` would find nothing; jog the lift to the height you want and read
the number off), the gearing, the shape, how it homes, and a pneumatic's anything.

**What a press does:** a gain ×1.25 or ÷1.25 (FINE: 1.05); volts and amps ±0.05; a speed, an
acceleration or a stall's rpm ×1.05 or ÷1.05 (about 5 %); "within" ±0.05 in or ±0.5°; a time ±0.05 s; a travel limit ±0.25 in
or ±1°. A number that starts at **0** cannot leave it by a factor: kA's first [+] is kV × 0.01 s (the
feedforward of a 10 ms lag), then ×1.25; a [−] below that first step goes back to 0. A press never
removes a current cap (0 A would be NO cap — the motor's full 2.5 A).

**What it refuses** — one sentence each, and nothing changes:
- anything **the build would refuse** — the build's own sentence (a stall signature at or above the cap,
  a travel limit past a preset or the home position, a settle time below 0 …);
- **a number the robot file cannot say.** In `.tuning`, 0 means "the starting value", so kP, kI, kD, kV
  and the settle time are never set to 0 (kD at 0 would come back as 0.004 from the file); kS and kA are
  never negative; nothing that is not a finite number;
- **a kV past twice its starting value** (12 V over the output's free speed — the bound is the
  library's, invented) and **a top speed past the output's free speed** (the build accepts a 500 in/s
  top speed on a lift that turns at most 7.85 in/s; live tuning does not);
- **a travel limit wider than the robot file's**, a homing push turned around, and a stated cap removed;
- **while a step runs** on that mechanism — a change waits until the step ends — and from inside the
  service's tick;
- **a narrowed limit that leaves out where the mechanism is HOLDING** ("it holds at 20.00 in, outside the
  new limits: move it inside first"), any limit change while a held button moves it, and — when it holds
  nothing (after a STOP, say) — **a narrowed limit that leaves out where it IS** ("it is at 19.95 in,
  outside the new limits: move it inside first"). A hold is never moved by a change, now or at the next
  hold: a HOME of a lift that rests at its stop, or letting go of a held button, starts a hold where it
  is, kept inside the limits — past a new limit, that would jump the target with no smooth move (8 in on
  a 1.5 in spool, at the starting kP, asks 153 V; the motor gives its −11 V at once). Where it IS is
  read again by the service as the change is applied; a lift still coasting or sagging across the new
  limit between the two reads gets the same refusal, and nothing is changed or recorded.

A rule above judges only a number that DIFFERS from the robot file's: a number as your file states it
is the build's to judge, so a file the build accepted can always be tuned. And a number your file
states PAST a bound — a top speed of 10 in/s on a lift that turns at most 7.85 — can always be stepped
back TOWARD it: a change is refused only when it is further past than your file's own number.

**Undo.** `tuning.undo()` takes back the last change — whichever mechanism it was on — up to 16 back;
`tuning.reset(i)` puts mechanism `i` back to the robot file's numbers, bit for bit (and can itself be
undone). Both wait while that mechanism's step runs, and an undo that would narrow a limit past where
it holds, or is, waits too (move it inside first).

**Copying the numbers back: the COPY block.** `tuning.copyBlock(i, sink)` logs a block of lines (tag
`TUNE`) to write into the mechanism's declaration — the SMALLEST set that builds to exactly what ran:

```text
'lift': copy begins -- write these into its declaration (tuned live): src/robots/mechanism_table.hpp:41
.tuning.kP = 0.3125
.tuning.kD = 0.01
.tuning.maxSpeed = 2
.tuning.maxAcceleration = 23.561944901923447
'lift': copy ends -- 4 lines
```

(This lift's file says `.tuning = {.kD = 0.01}`; kP and the top speed were changed live. The begin
line names the file and line of the mechanism's own declaration — here, an example line.)

Each member it names — and only a member changed live — is written WHOLE, the way the declaration
writes it (`.stall = {...}`, `.home = homeByStall(...)`): replace that member in your declaration and
leave every other one as it is. Its `.tuning` lines (when a `.tuning` number changed) are the whole
`.tuning = {...}` your declaration then needs, in its member order — every one changed, and every one
the library would otherwise work out DIFFERENTLY: a number your file states that the library would not
work out the same (the 0.01 kD above), and a changed top speed's acceleration as it ran (the file would
otherwise make it the new top speed in 0.25 s), while a kV nobody touched stays unstated. One catch: a
number your file states at EXACTLY the value the library would work out anyway cannot be told from one
it works out, so it is left out — your declaration builds to the same numbers today, but after a
cartridge or gearing change that number would follow the library's working-out. **Write them in by
hand**, replacing your `.tuning = {...}` with the block's lines. **If the cartridge, the gearing or the
spool changes later, RESET and tune again:** a number a COPY states stays exactly as it was tuned. Each number is printed in the
shortest form that reads back as the same number, and every line fits the terminal's 200 characters.

### The Tune program — tuning on the brain screen

**"shulib Tune"** is a program of its own — never a page of the driver's or the match program, which
always run exactly your robot file's numbers. It builds **only your robot's mechanisms** (never the
drive), for **any robot whose mechanism table is not empty**:

```sh
rm -rf bin .d && make ROBOT=tank PROGRAM=tune
pros upload --slot 7 --name "shulib Tune"
```

(`ROBOT=` is your robot's; a robot with no mechanisms is refused by `make`, naming why. The
`rm -rf bin .d` matters for the same reason as the Auton Tester's: `make` does not see that `PROGRAM=`
changed.) Start it from the **brain's** screen with the controller paired; nothing in it moves with a
field or a competition switch plugged in.

**The safety rules, before anything else:**
- **Nothing is made until READY.** The first screen says what READY does — making a pneumatic's line
  drives it to its start, so a claw can move — and waits for you to say the robot is clear (on blocks,
  hands away). READY is greyed, and says why in red, until the controller is connected, no field or
  switch is plugged in and no controller button is held.
- **Each mechanism's FIRST motion asks again** (GO or BACK): its first TEST, HOME or stick push.
- **ANY controller button stops every mechanism** — each one to its declared safe state — and so do the
  controller disconnecting, a field or switch being plugged in, VEXos disabling the robot, and the
  screen's STOP (never greyed). While a button is held, nothing starts.
- **TEST, HOME, STRING CHECK and the stick are refused**, with a sentence on the screen, with no
  controller connected, with a field or switch plugged in, while a button is held, on a mechanism that
  is ABSENT — and a position TEST before the mechanism has found its zero ("press HOME first").
- **The stick moves something only on a fresh push.** A stick still held when you change the mechanism,
  after a stop, after GO or on another page moves nothing until you let it centre and push it again.

**The page.** The top line is the mechanism (‹ › picks another), what it is and whether it is homed; the
next, where it is, what it aims at and its last verdict; the third, why nothing may move, when
something forbids it. Below them, the **selected number** — ^ v pick it, − + change it, FINE makes a
gain's step ×1.05 — with the robot file's value and what one press does. A `*` marks a number you
changed. The two status lines say what the last press did, or why it was refused, without the shown
mechanism's name (the title has it; the terminal has the whole sentence). The buttons: **TEST A** / **TEST B** (labelled with their targets), **HOME**, **UNDO**,
**RESET**, **COPY**, **STRING**, **STOP** and **EXIT**.

**TEST A and TEST B** move the mechanism and report the move. A lift or an arm goes to its first and its
last preset (with fewer than two presets, to a quarter and three quarters of its range); a flywheel
spins up to its first and last preset (or half and nine tenths of its top speed) and is stopped after
its report; a power mechanism runs at + and − its most volts for half a second. Each move's time limit is
its plan's own time at the live speed and acceleration, twice over, plus its settle time and half a
second — at most 10 s; past it the move is TIMED_OUT. A move that SUCCEEDS is watched for one more
second (its hold), then the **REPORT** page shows: the verdict and the time; the **overshoot** — the
farthest past the target; how long it took to **settle** after first reaching it; the **final error**;
the **peak** current; the **hold** current (the mean over that second); and two warnings —
`WARNING: stall … not above the hold` (an ordinary hold can trip the stall signature: raise it) and
`WARNING: cap … not above the peak` (the cap limited the move). When the signature trips on the hold
itself, the hold ENDS inside its second: the page says `hold ENDED (the stall tripped on it)` and
`WARNING: stall … trips on the hold: raise it` — even when you press STOP after it ended; a stop
before the hold ends says the hold is n/a. A power
mechanism's report has no settle and no hold. AGAIN repeats the test; BACK returns; the report's **STOP**
is where the tuning page's STOP is, and works from its first moment.

**The stick.** The left stick's up/down moves the shown lift, arm or power mechanism by hand (at most
6 V — a library choice), and holds it where it is when let go: jog the lift to the height a preset
should be, and read the number off the state line. (Presets are not live — an auton finds a preset by
its value.) A lift that has not found its zero moves only by its `.unhomedVolts`.

**HOME** runs the mechanism's own homing step (a homing push until its switch or its stall signature;
at most 10 s) and says the verdict, the time and the push's peak current — the number a homing
signature is sized by.

**COPY** writes a block of `[TUNE]` lines (the block above) for **every mechanism you changed** to the
terminal **and** to the SD card, `/usd/tune_log.txt`, kept across boots — a reboot back into Tune does
not erase the last session's COPY. **EXIT copies for you** every mechanism changed since its last COPY,
so the numbers are in the log before you stop the program. **Copy the lines into your robot's mechanism table by hand** (robot two's is
`src/robots/mechanism_table.hpp`), with a comment saying they were tuned live and when; the panel's
import is for after the first competition. Nothing persists: leaving the program, or a reboot, runs the
robot file's numbers again. **After a cartridge, gearing or spool change, RESET and tune again** — a
number a COPY states stays exactly as it was tuned.

**The current cap stays in the motor.** A cap you change is written into the motors, and a motor keeps
a limit after the program that set it ends (believed — HA-220). So Tune writes the robot file's cap
back — or the motor's own 2.5 A when the file states none — on **RESET**, when you **leave that
mechanism's page** (‹ ›), and on **EXIT**. Stopping the program from the brain's screen runs no code:
**press EXIT before you stop it.** **COPY before you leave a page whose cap you tuned:** the tuned cap
leaves the live numbers as the page closes, so Tune COPIES that mechanism's numbers to the log first
when you have not (the status line says so) — write the cap from that block, not from a later COPY. Only
the page you are on can hold a changed cap, so UNDO — which takes back the last change on ANY mechanism —
**waits** when its next undo would change another mechanism's cap (the cap put back as you left its
page, or a RESET of a raised cap): nothing changes, and the page says
`UNDO waits: next is '…''s cap -- UNDO it on its page`. Go to that mechanism's page (‹ ›) and UNDO
there; then come back to UNDO the changes before it. A cap that cannot go back by itself — a live
stall or homing signature at or above the file's cap — keeps you on the page and says so: lower
that signature, or RESET (its numbers are COPIED). EXIT RESETS such a mechanism instead, after
copying it, and the EXITED page says how many. **To start a cap where the file states none, press
−**: the first press sets 2.45 A (the motor's
own 2.5 A less one step); + from none would ask 2.55 A, above the motor's most, and is refused (the page
says `file: none; - starts a cap at 2.45 A`). EXIT also releases every mechanism; TUNE AGAIN returns to
the page.

**STRING CHECK** is empty in this version: the snapped-string watch and its bench page come after the
first competition. Today a snapped string is **not** detected.

Everything here is tested on the **host test plant**; nothing has run on a robot.

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
step waited for). Stopping is the routine's usual policy (a step written after `r.onFail(carryOn)` or
`r.onFail(skipTo("..."))` goes on instead, except after a CANCELLED verdict or once the auton's time
is up —
[Ch. 9](09-the-recipe-api.md#choosing-what-a-failure-does-onfail-and-label)): the drive safed, every later step skipped,
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

## The bench page — Bench Tests station 13

**Bench Tests** (the bench program, slot 3) has a thirteenth station, **13 MECHANISMS**: it builds the
robot's mechanisms from its robot file exactly as every program does — no drivetrain, no mechanism service,
nothing homed — and shows ONE at a time, with three jobs:

- **Every reading, live, in the mechanism's own units.** Each motor's angle since power-up, speed,
  current and temperature (`unknown` when the read failed — never a guessed "cool"); an arm's or a lift's
  position **in its declared units**, through its gearing and its shape, taking the power-up pose as its
  declared home (`ARM +95.0 deg (power-up = home 0.0; 5.00:1)`) — so you read `.range`, the presets and
  the level angle off the robot by moving the arm by hand, and **a gearing typed wrong is a wrong angle on
  the screen** (a 12:60 arm turned one turn reads 360°; typed `gears(12, 36)` it reads 600°); a
  flywheel's output rpm; the angle sensor raw and as mounted, and on its own line **the number
  `absoluteSensor(at, reads)` takes — the RAW reading, in [0, 360)** (`absoluteSensor(at, reads): reads =
  180.0`; the mechanism service applies the mounting to both sides, so "as mounted" is the joint's
  direction, never what you type — for a `rotationSensor(Reversed)` it is the wrong number); a limit
  switch's level — a home switch or a piece switch alike — and the page asks for it **pressed, released
  and with its cable out** (HA-184: an unplugged 3-wire line reads a level, so only the three readings side
  by side say what it does); the piece sensor's reading and whether it SEES a piece, by the same
  positive-evidence rule a step uses (`NOT ANSWERING` while its read-fault count moves). A mechanism with
  more readings than the panel holds (four motors, an angle sensor and a home switch) gets a second page:
  **tap the readings** for it. **The current you see while you hold a jog
  against something is the number a jam signature is sized by** — a held button draws less than a jam at
  full power (on the host test plant a 6 V push into an obstacle drew 1.25 A, under a 2.0 A signature).
- **Power, behind six gates.** Only a mechanism the robot built; a CLEAR question before anything is
  made (making a pneumatic's line drives it to its start level), then a confirm before a motor
  mechanism's first volt (`POWER 'arm'?`) and before EVERY pneumatic fire (`FIRE 'claw' -> 'open'?` — a
  pneumatic's position is read back from its lines at each entry, so the fire names the OTHER one even
  after an earlier visit left it in its safe position); **L1 held** the whole time — letting go, or a
  controller that drops, puts whatever the page powered in its declared safe state that tick (a confirm
  prompt CLOSES the moment L1 is let go, granting nothing), nothing it never powered is touched, and
  nothing restarts by itself (a button still held when the controller comes back, or through NEXT, is not
  a press); a jog runs only while its arrow is held; a ceiling starting at 3 V — again for each mechanism
  NEXT shows — that R1 raises to 6, 9, 12, **never past a power mechanism's declared `maxVolts`**; a cut
  that stops THAT mechanism, names why and keeps it stopped until the station is re-entered — two of its motors fighting for 250 ms, any motor above 2.4 A for 250 ms, its
  own stall signature, a jog held for 3 s; and leaving the station makes everything safe. **Every one of
  those numbers is invented for a first run** ([HA-182](../hardware-assumptions.md)'s page entries) and
  the page says so.
- **The direction check.** Hold L1 and press A with the mechanism at rest (A is refused while a jog runs —
  let go of the arrow first): the page says `ONLY 'arm' should move -- watch it` and drives NOTHING for
  1 s — a mechanism still turning in that second's last 0.4 s (a coast, a sag) abandons the check, since
  a jog from a moving start would judge the coast — then jogs it "+" at the ceiling for 0.4 s and hands
  what the encoders saw to `judgeDirection` (above). A fight, a motor that never answered, a jam, a
  back-drive or a sensor against its motors is reported at once; otherwise it asks its question with three
  buttons — its + way / the other way / it did NOT move or something ELSE moved — and shows the verdict,
  naming the robot-file line. A pneumatic's check is a fire and "which position did it go to?" (its
  positions as buttons, and "nothing / something else moved"). **RECORD** writes every reading and EACH
  mechanism's latest verdict this visit to the serial log and the SD card as `[R3A] mech` lines — each
  line one serial write, a longer one in pieces that each start `[R3A] mech ... (cont.)`, so a grep of the
  tag keeps the whole sentence and the path stays whole on its line.

It saves nothing: edit the robot file at the line the verdict names, rebuild, upload, check again. The
page's logic is host-tested (the gates, the check's flow, the readings, the RECORD lines —
`test/bench_mech_station_test.cpp`, over the library's own mechanism parts on a fake provider), and the
order of the steps only the bench program runs (the CLEAR before anything is made, the dead-man watched
inside each confirm) is pinned by reading its source; the page itself has **never run on a robot** — the
bench worksheet's station for it is its first run.

## The honest limit — the service runs only when something ticks it

The service runs ONLY inside shulib's own waits, motions and pauses — and every tick of the driver loop.
**A busy loop of your own starves it** — a `pros::delay` in an auton, a `then()` action that blocks — and
a starved service leaves every mechanism at its LAST command: not held, not limited, not watched for a
jam (on the host test plant, an arm left at 11.74 V for 1.5 s sat against its hard stop at 2.45 A) —
except a stated [`.currentCap`](#current-caps--how-hard-a-string-is-pulled), which the motor itself keeps
(believed, not measured — HA-219). In
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
`target_is_outside_the_travel_limits`, new with the position behaviours
`position_absolute_homing_needs_a_travel_shorter_than_one_turn_of_its_angle_sensor`, and new with
current caps `a_current_cap_must_be_above_0_and_at_most_2_5_A`,
`a_current_cap_must_be_at_least_0_001_A_a_motor_holds_whole_milliamps` and
`a_stall_signatures_amps_must_be_below_the_current_cap_it_could_never_trip`. The driver
file's (above) point at its `controls(` line — a button twice names the button — and the auton's wrong
steps are `static_assert` sentences ("required from here" names your line).
