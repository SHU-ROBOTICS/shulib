# 21 — Adding your robot by hand

> **Covers:** how a second robot — robot one, the 2026 cascade robot — was added to this repository by
> hand: every file a robot lives in, its **facts file** (every number nobody has measured yet is written
> there once, marked **OWED**, and the build of every program that needs it **refuses**, naming it), its
> six programs and their slots, the order things happen on the robot, its six mechanisms and the builder's
> button map (yours to change), the flipper's three forms, and robot two — robot one's TWIN: the same build,
> its own facts file, the team's routes mirrored where they are marked so (the skills route always).
> **Read this if:** robot one or robot two is on your bench, the build just printed `robot one: OWED -- …` or
> `robot two: OWED -- …`, or you are adding a robot of your own.
> **Assumes:** [Chapter 7](07-getting-set-up.md) (building and uploading), [Chapter 18](18-mechanisms.md)
> (declaring mechanisms) and [Chapter 19](19-calibrating-your-robot.md) (Calibrate).

Robot one's mechanisms, its controls and its autons run in the host test suite on the host test plant (the
suite's model of a robot) on the typed values and, where a fact is still owed, its **placeholder**, and its six
builds are checked by the src build gate. A placeholder can never reach the robot: until a fact is typed, every
build of a program that needs it stops with an error naming it. **Robot two** was rebuilt as robot one's
twin; its facts are owed until its builder reads them off it ([below](#robot-two-robot-ones-twin)).

## Where a robot lives in the code

Adding robot two touched these places one at a time over several weeks; robot one touched all of them at
once. Each line is what a robot needs — copy the pattern when you add yours.

| File | What the robot needs there |
|---|---|
| `src/build_selection.hpp` | its `RobotId` (`RobotOne`), one branch of THE ONE CHAIN per program (each sets the program, the robot and the build's beacon together), its robot beacon (`shulib-robot=robot-one`), and its define (`SHULIB_ROBOT_ONE_2026`) in each `PROGRAM=` guard above the chain — and, because it has mechanisms, in the Tune guard — without it that build stops at an `#error` naming why |
| `Makefile` | its `ROBOT=` word (`ROBOT=one`) and its define; the four `PROGRAM=` lines accept it; and — because it has mechanisms — its word in `MECHANISM_ROBOTS`, the list "shulib Tune" (`PROGRAM=tune`) accepts |
| `src/robots/chassis_table.hpp` | its drive table (`kRobotOneChassis`) — what Calibrate measured, and the autons' drive settings (speed, turn rate, heading), read from its facts file. Calibrate discovers the drive's ports, directions, sides, front, scale and track width, and finds the **IMU** on the brain's own device list; the one drive fact it cannot discover is the drive **cartridge** (the form's A2) — typed here before Calibrate runs (robot one's: BLUE, 2026-11-15) |
| `src/robots/robot_one_facts.hpp` | **its facts file** — every number about its mechanisms, OWED or typed (below) |
| `src/robots/mechanism_table.hpp` | its mechanisms, in its own `namespace robot_one`, between its `[robot one mechanisms]` markers, every number read from the facts file; its line in `mechanismTableFor()` |
| `src/team/robot_one_driver.hpp` | its driver file — which button does what (the builder's map, below) |
| `src/team/robot_one_autons.cpp` | its autons — the 40 in and the library's acceptance run as SKILLS autons, and the team's routes (one line, `addTeamRoutes`) |
| `src/team/skills_route.cpp` | **the team's routes — the skills route and every route you name — written once for both robots** (robot two runs the ones marked mirrored, mirrored). **They are yours to write** — in the route maker, or by hand |
| `src/common/this_robot.hpp`, `src/common/robot_words.hpp` | nothing to type: the programs run THIS build's driver file and autons (`ThisRobotDriver`, `registerThisRobotsAutons()`) and print THIS robot's words — both robots' screens say `MECH`, and Calibrate writes THIS robot's name into its record ([Chapter 19](19-calibrating-your-robot.md#two-robots-one-card-each-the-record-names-its-robot)) |
| `test/CMakeLists.txt` | its autons file, so the host suite runs every auton in it |
| `tools/src_build_gate.py` | its six builds |

Robot two has the same files with its own names — `src/robots/robot_two_facts.hpp`, its region in the
mechanism table, `src/team/robot_two_driver.hpp`, `src/team/autons.cpp` — copied from robot one's when it
became the twin ([below](#robot-two-robot-ones-twin)).

## The facts file: OWED until typed

Every number about robot one is written once, in `src/robots/robot_one_facts.hpp`. One nobody has
measured yet is OWED, like this (the cascade's gear teeth, before the builder counted them):

```text
inline constexpr auto kCascadeTeeth = owed(Teeth{12, 60}, Stage::A, "the cascade's gear teeth, motor then spool");
ROBOT_ONE_CHECK(kCascadeTeeth, "the cascade's gear teeth, motor then spool");
```

`Teeth{12, 60}` is a **placeholder** — a number chosen only so that every rule the library checks
passes, so the host suite can build and run robot one's cascade before the robot exists. It is not a
measurement. The line below it is the fact's **check**: while the fact is OWED, robot one's build of
every program that needs it stops:

```text
make ROBOT=one
...
src/robots/robot_one_facts.hpp: error: static assertion failed: robot one: OWED -- the cascade's gear teeth, motor then spool (src/robots/robot_one_facts.hpp)
src/robots/robot_one_facts.hpp: error: static assertion failed: robot one: OWED -- the cascade's motor ports (src/robots/robot_one_facts.hpp)
...
```

— one line per owed fact the program needs, and **no other robot's or program's package is left in
`bin/` to upload**: `make` notices a changed `ROBOT=` or `PROGRAM=` and starts fresh by itself, so a
refused build can never leave the last robot's or program's package behind. (One exception: if this
same robot-and-program build once linked — every fact it needs typed — and is refused now, its last good
package is still in `bin/`. It carries no placeholder.) Every other build — the other robot's, the bench
bot's, the host suite — is untouched by robot one's owed facts.

### Which program needs which fact

| Stage | The facts | Needed by |
|---|---|---|
| **A** — what is plugged where | the cascade's motor ports, their directions (as the builder sees them), the cartridge, the gear teeth, the **starting** stall signature and current cap (chosen, not measured), every motor on the Devices screen that is not a drive motor, the flipper's kind; the side toggles' ports (and which is the right one), cartridge and starting cap; the three pistons' 3-wire letters | **all six programs** — Bench Tests included, because it powers what is declared |
| **B** — how it moves | the cascade's measured rows (bottom stop, every level, top stop), its levels, its hold volts, its homing push and the homing's own stall signature, its volts before it is homed; which way each side toggle runs; which line level is each piston's open (or down) position, and each stroke time | every program **except** Bench Tests and Calibrate — today Drive, the Auton Tester, Competition and shulib Tune |
| Recorded | the AI Vision camera's port and where it sits | none — written down for the camera's own chunk |

Stage B is not needed by Bench Tests or Calibrate on purpose: Bench Tests is the tool that *measures*
Stage B, so it must build on Stage A alone. Every other program needs it — the facts file lists every
program by name (`needsStageB()`), so a program added to the repository later does not build until
someone says whether it needs Stage B.

### Typing a fact

The builder fills in the [robot facts form](../robot-facts-form.md); the coordinator types each answer.
`owed(` becomes `typed(`, the placeholder **kept**, the value in its own place, and who measured it, how
and when:

```text
inline constexpr auto kCascadeTeeth = typed(Teeth{12, 60}, Teeth{1, 1}, Stage::A, "the cascade's gear teeth, motor then spool", "the team's builder, 2026-11-15, read off the robot: DIRECT, no gears -- the motor turns the spool (1:1)");
```

The build refuses three mistakes, each by name:

- **a typed fact with no source** — `robot one: a typed fact needs a real source …`;
- **a typed value still equal to its placeholder** (`owed(` flipped to `typed(` without writing the
  value) — `robot one: TYPED but still the placeholder …`, unless the source says
  `same as the placeholder, checked` because the builder really measured that number;
- **the src build gate's own marker** as a source (`gate copy`) — the gate proves robot one's builds
  with every fact typed in a throwaway copy of the tree, and a file copied back from there cannot link.

A number chosen at the bench (the starting cap, say) is typed with the source `chosen, not measured`.
A fact is never "corrected" to match a belief, and never invented to get a build through. The host suite
reads every typed fact's source too: it must say **when** (a date, like `2026-11-17`) and never call itself
a guess — any wording otherwise (`station 13 RECORD, the team's builder, 2026-11-16` is fine).

**After each typing, run the host suite and the src build gate.** Both robots' mechanisms compile in
*every* build, so a typed value that breaks one of the library's rules — a level above the top stop, rows
out of order, a preset in the driver file bound to a level that has no name — stops every build, both
robots' and the host suite's (loudly, by name). The gate runs all fourteen builds, and builds each robot's
six again in a throwaway copy with every owed fact typed — so it checks a robot whose own builds still
refuse. Run it before uploading anything.

**Not a facts-file edit:** a third cascade motor, or a new mechanism's block — those are declarations, in
`src/robots/mechanism_table.hpp` (the number of motors is part of the declaration's shape). A new
mechanism's *numbers* are facts first: each is added to the facts file with its check and its stage, then
the block reads them as `facts::…` (that is how the side toggles and the three pistons were added).

## Six programs, six slots

| Program | Build | Slot | Needs |
|---|---|---|---|
| Bench Tests | `make ROBOT=one` | 3 | Stage A |
| shulib Calibrate | `make ROBOT=one PROGRAM=calibrate` | 4 | Stage A |
| shulib Drive | `make ROBOT=one PROGRAM=drive` | 1 | Stage A and B |
| shulib Auton Tester | `make ROBOT=one PROGRAM=autontest` | 2 | Stage A and B |
| shulib Competition | `make ROBOT=one PROGRAM=competition` | 6 | Stage A and B |
| shulib Tune | `make ROBOT=one PROGRAM=tune` | 7 | Stage A and B |

Each is uploaded like robot two's (`pros upload --slot 3 --name "Bench Tests"`, and so on —
[Chapter 7](07-getting-set-up.md)). Robot two's six are the same programs with `ROBOT=tank` (robot two kept
its name). **shulib Tune** tunes the cascade's
numbers live on the brain screen ([Chapter 18](18-mechanisms.md#the-tune-program--tuning-on-the-brain-screen)); it
homes the cascade, jogs it and moves it to its first and last level — with fewer than two levels named (robot
one's today: manual, no levels), to a quarter and three quarters of its range — so, like Drive, it needs Stage B:
type a hold voltage and a homing push first (measured, or "chosen, not measured"), then tune them with it. Each
TEST stops at 10 s ([Chapter 18](18-mechanisms.md#the-tune-program--tuning-on-the-brain-screen)); a TEST that
runs out of time ends **TIMED_OUT** holding where it stopped, and a second TEST finishes the move. (On ROB1-A's
placeholder gearing — a red cartridge through 12:60 — that happened on every long move; the builder's blue
cartridge driving the spool directly turns it thirty times faster.)

### Tune's COPY on robot one: each number goes to its fact

A robot with no facts file has its COPY lines written into its declaration by hand
([Chapter 18](18-mechanisms.md#the-tune-program--tuning-on-the-brain-screen)). **Robot one's and robot two's
are not**: each robot's declaration reads every number from its own facts file, and the host suite fails a
number typed straight into robot one's region ("a literal number in robot one's mechanism region"; robot
two's region is robot one's, token for token). So each number a COPY prints is
typed into its FACT, like any other answer — by the coordinator, with a source such as
`shulib Tune COPY, 2026-11-17`:

| The COPY's line | The fact it is typed into |
|---|---|
| `.gravity = constantGravity(…_volt)` | `kCascadeHoldVolts` |
| `.home = homeByStall(…, …_volt, {…})` — its push, the volts | `kCascadeHomingPushVolts` |
| `.home = homeByStall(…, …, {.amps = …})` — its signature's amps | `kCascadeHomingStallAmps` |
| `.stall = {.amps = …, .belowMotorRpm = …, .lasting = …_s}` | `kCascadeStallAmps`, `kCascadeStallRpm`, `kCascadeStallTime` |
| `.currentCap = {.amps = …}` | `kCascadeCapAmps` |

**HOME has its own amps; its speed and time are `.stall`'s.** A homing push is gentle, so its stall is
small ([Chapter 18](18-mechanisms.md#homing--every-moment-and-which-one-to-choose)): robot one's homing
signature has its own amps fact, `kCascadeHomingStallAmps` (0.45 A, chosen, not measured — the 1.5 A jam
signature could never end a gentle home), and shares `kCascadeStallRpm` and `kCascadeStallTime` with
`.stall`. So a `.stall` line whose rpm or time a session changed changes what HOME waits for too, and a COPY
prints a `.home` line only when the homing push or the homing signature changed. After typing either, jog the
cascade partway up in Tune and run **HOME** once; rely on it only when HOME ends SUCCEEDED with the cascade
resting on its bottom stop. A homing `.amps` above what the push draws at the stop ends TIMED_OUT, never
homed, and a stall-home trusts the first stall it sees. If the session tuned HOME's rpm or time to numbers
different from `.stall`'s, give both lines to the coordinator (below).

**No fact holds these yet:** a `.tuning` block (kP, kI, kD, kS, kV, kA, the top speed, the acceleration,
"within", the settle time), a narrowed `.range`, or a `.home` signature whose speed or time differs from
`.stall`'s (robot one's homing signature has its own amps only). Keep those lines — they are in `/usd/tune_log.txt` —
and give them to the coordinator: where they go is a decision not made yet (new facts, or a marked block
the host suite allows). Until then robot one runs what its declaration says today: the library's starting
gains and profile, and the range from the bottom row to the top row.

## The order on the robot

1. **Read the brain's own Devices screen** (no program needed): every port 1–21 and every 3-wire port —
   the form's section B. The coordinator types Stage A into the facts file, and the drive **cartridge**
   (the form's A2) into `kRobotOneChassis`, with who read it and when in its `provenance` — Calibrate
   refuses an unset cartridge. The **IMU** is not typed: Calibrate finds it on the brain's device list
   and records its port (an IMU must be on the robot — nothing drives a tank without one). Robot one's
   Stage A was typed on 2026-11-15 (below) except one answer: which side toggle port is the right one.
2. **Bench Tests** (slot 3): the census, then station 13 jogs each declared mechanism once to check which
   way it moves. **Calibrate** (slot 4): it discovers the rest of the drive by itself.
3. **Measure at station 13** — the cascade's page lists these steps and marks the one you are on; the line
   under its name always says what to do now. Hold **L1** the whole time (let go and it stops); **UP**
   raises, **DOWN** lowers. With the cascade on its bottom stop, RECORD; then (once there are levels) jog
   it up to each level, tape the height and RECORD each; then the top stop: **UP at 3–6 V (R1 raises the
   3 V start to 6 V), stop when it stops rising**, RECORD — never keep pushing into it. If the line says
   `It STALLED` at a stop, that IS the stop: **RECORD first, then DONE and open 13 again** — the page keeps
   your place in the steps while Bench Tests runs. A RECORD with nothing jogged since the last one counts as
   that step again; to start the steps over, stop the program and run Bench Tests again. Two rows, the
   bottom stop and the top stop, are the least the library takes. After each RECORD the page shows, large,
   what it wrote — `REC 2: cascade output +3759.04 deg`: **that `output … deg` is the number the
   coordinator types, not the inches** (until Stage B is typed, station 13's inches come from the
   placeholder rows). A jog stops by itself after 3 s: let go of L1 and hold it again to go on. Run each
   side toggle once — briefly, never held against its stop (it declares no stall signature) — and say
   which way is "run"; fire each piston once with **UP** (the ramp too: DOWN never fires a piston) and say
   whether it went open (down, for the ramp) or closed.
   The coordinator types Stage B.
4. **Drive, the Auton Tester, Competition, shulib Tune** — and your skills route.

## Robot one's mechanisms (typed 2026-11-15)

The builder's answers, as the facts file holds them — every line `typed(…, "the team's builder, 2026-11-15,
read off the robot")`, or a starting value `"chosen, not measured -- …"`:

| Mechanism | What it is | Typed | Still owed |
|---|---|---|---|
| **cascade** | a `Lift`: two **blue** motors on ports **10 and 1**, facing each other, **direct** to the spool (typed as the 1:1 tooth count `gears(1, 1)`); a 4-stage string cascade, homed by stalling gently down | ports, directions (the builder: they face each other; port 10 taken as Forward, station 13 confirms), cartridge, gearing; the coordinator's starting cap **2.0 A** per motor and stall signature **1.5 A below 5 rpm for 0.3 s**; its own homing signature **0.45 A** (chosen, not measured — a gentle push's stall); its levels: **none yet** ("for now it will be manual") | its two rows (station 13: the bottom stop and the top stop), its hold volts, its homing push, its volts before it is homed |
| **right toggle**, **left toggle** | two `Power` mechanisms — each its own arm on its own **green** motor, direct — that RUN one way while their button is held and stop on release; no angles, no homing | their ports **9 and 2**; cartridge; a starting cap of **1.5 A** per motor (chosen, not measured, to bound the push held against a stop; written into the motors by Drive, Competition, the Auton Tester and station 13) | **which port is the right one** (the left is the other); which way each runs |
| **claw** | a `Pneumatic` on 3-wire **A** — one piston on the cascade's top stage; it picks up Pins and Cups | its letter; a starting stroke time of 0.5 s (chosen) | which line level opens it |
| **flipper** — the front toggle | a `Pneumatic` on 3-wire **B** — two forward pistons on one solenoid, pushing where the camera looks; robot one's flipper for the autons' Toggle step | its letter; the flipper's kind = pneumatic (the coordinator's assignment); a starting stroke time of 0.5 s (chosen) | which line level pushes it out |
| **ramp** | a `Pneumatic` on 3-wire **C** — one piston on the back; it comes down to push other robots | its letter; a starting stroke time of 0.5 s (chosen) | which line level puts it down |

Each piston **starts, and is made safe, in the position its solenoid holds unpowered** — so nothing moves
when the robot is switched on, whichever way its levels turn out. A wrong guess about the levels only swaps
the two names (`claw["open"]` would close it), which is why autons wait for station 13's answer.

So on the real tree: **Bench Tests and Calibrate refuse naming exactly one fact** — "the right side toggle's
port" — and build as soon as it is typed; **Drive, the Auton Tester, Competition and shulib Tune** also wait
for station 13's answers above and the cascade's three voltages.

## Robot one's controls — the builder's map, yours

The library's builder wrote a starting map when robot one first got a build (no robot-one program builds
without a driver file); on 2026-11-15 the builder gave the real one, and it is typed exactly. **It is
yours**: change any line in `src/team/robot_one_driver.hpp` and be able to say why — by hand, or by clicking on
the route builder's picture of the controller ([chapter 22](22-making-your-first-route.md#the-drivers-buttons-and-the-driving-feel)).

| Input | Does |
|---|---|
| left stick | drive forward / back |
| right stick | turn |
| R1 (held) | the cascade **up**; let go and it stops and holds where it is |
| R2 (held) | the cascade **down**; let go and it stops and holds |
| A | the claw: press = open, press again = close |
| B | the front toggle: press = open (out), press again = close |
| X | the ramp: press = down, press again = up |
| L1 (held) | the **right** side toggle runs; let go and it stops |
| L2 (held) | the **left** side toggle runs; let go and it stops |
| Y | the drive loop's re-zero (field-centric is off) |
| d-pad | free — the cascade's preset heights go here later |
| (driver start) | the first time driver control starts, the cascade homes |

**The first press moves each piston from where it rests with its solenoid unpowered** — so A, B and X do
what the table says only if station 13 finds the claw and the front toggle rest closed and the ramp up. If
one rests open (or down), tell the coordinator: its first press would then close it, and starting it powered
instead would move it at power-up.

Before the cascade is homed, R1 and R2 give it at most its unhomed volts and it has no soft stops; once
homed, the library stops it short of its bottom and top. **At driver start the cascade moves down on its own**
until it stalls gently on its bottom stop (at most 3 s), and R1 / R2 wait until it has — a button held
through it does nothing until let go. That is the one line marked `[delete-able]`: delete it and the cascade
is never homed in driver control. A side toggle held on L1 or L2 runs ONE way — the library's one-button
hold, `whileHeld(mechanism, button, volts)` (a power mechanism only): its direction is the facts file's,
never a second button. As the builder gave it, the statement reads:

```cpp
inline constexpr DriverControls kControls = controls(kRobotOneMechanisms,
    whileHeld(cascade, Button::R1, Button::R2, 12_volt),   // R1 up / R2 down while held; let go: it holds
    toggle(claw, Button::A),                               // each press: open / close
    toggle(flipper, Button::B),                            // the front toggle -- each press: open / close
    toggle(ramp, Button::X),                               // each press: down / up
    whileHeld(rightToggle, Button::L1, 12_volt),           // held: runs (one way); let go: stops
    whileHeld(leftToggle, Button::L2, 12_volt),            // held: runs (one way); let go: stops
    homeAtDriverStart(cascade, 3_s));                      // [delete-able] home it at the first driver start
```

Three numbers there were nobody's answer, and the file marks each **chosen, not measured**: the cascade's
12 V (the fastest jog a blue motor driving the spool directly has — lower it if the cascade is hard to stop
where you want), the side toggles' 12 V (each motor's chosen 1.5 A cap bounds the push against a stop), and
the driver-start home's 3 s.

**The Drive screen's mechanism row** is compact on both robots — six mechanisms do not fit the screen's
50-character row in the long form a robot with fewer mechanisms gets:

```text
MECH 10H34 9B31 2B30 CLAW:C FLIP:O RAMP:U
```

Each motor mechanism by its first motor's port, then a letter for what it was last told — `F` forward, `R`
reverse, or its stop: `H` hold, `B` brake, `C` coast — and its temperature in °C (`10X`: not found at boot).
Then each piston by the first four letters of its name and the first letter of its position (`C`losed,
`O`pen, `U`p, `D`own; `?` its lines match neither, `X` not found). **`FLIP` is the front toggle** — its
declared name is `flipper`, the name the flipper check looks for. ` FREED` at the end means what it means
on the long row.

**When presets come back**, each level is typed into the facts file first — a name and a height, with a row
AT each level (the cascade's string winds in layers, so the height per turn changes as it climbs) — and then
its line goes on the d-pad, `preset(cascade[facts::lv(0).name], Button::Down, 2_s)`. A preset bound to a level
that has no name yet does not build.

Homing for a skills run is the route's to say: a route's first line `r.startHome(cascade, 2_s)` homes it
while the first leg drives. **Wait for the home to finish before the cascade's first move** —
`r.waitUntilDone(cascade, 2_s)`, or home with `r.home(cascade, 3_s)`, which waits by itself. A move (or a
trigger's move) sent while the cascade is still homing replaces the home, and that move then stops with
NOT_HOMED.

## The flipper: three forms

The flipper is the mechanism the autons' Toggle step pushes with ([Chapter 18](18-mechanisms.md)). Robot
one's is the **front toggle** — the builder named its pistons on 2026-11-15 and the coordinator made them the
flipper — declared as a `Pneumatic` named
`"flipper"` (the name robot one's flipper check looks for; the team calls it the front toggle) on 3-wire B,
and the facts file's `kFlipperKind` is typed `Pneumatic`; the build checks the two agree. The Toggle step
takes its two positions as the push and the retract: `.flipper = flipper["open"], .back = flipper["closed"]`.
The side toggles are not the flipper.

All three forms were written and tested before robot one's flipper was known (each beside the real cascade, bound
and flipped on the host test plant), and they stay as the forms for a robot built later. For any of them:

1. **Each of the block's numbers becomes a fact first** — a motor arm's port, cartridge, gear teeth, stall
   signature, cap, range and two angles, or a pneumatic's 3-wire letter, its levels and its stroke time —
   each added to the facts file with its check, then typed. A motor arm's are Stage A (station 13 powers
   it); its angles, and a pneumatic's levels and stroke time, are found at station 13, so Stage B.
2. **Then the block is written** in the robot's region, every number written as `facts::…` like the
   cascade's — never the literal numbers below. The host suite fails on any literal number or bare value
   word left in robot one's region; **`make` alone does not run that check**, so run the suite.
3. Its control goes into the driver file, and the flipper's kind is typed. The build then checks that the
   typed kind and the list agree.

**A motor arm** (a motor on its own port — that port is also a non-drive motor on the Devices screen):

```cpp
inline constexpr Arm flipperArm{.name = "flipper", .motors = {Forward}, .cartridge = Green, .gearing = gears(12, 36),
                                .safe = Hold, .stall = {.amps = 1.0, .belowMotorRpm = 5, .lasting = 0.3_s},
                                .currentCap = {.amps = 1.5}, .range = {0, 120}, .gravity = noGravity,
                                .home = restsAtStop(0), .presets = {{"stow", 0}, {"flip", 95}}};
```

its wiring `wire(flipperArm, motors(11)),` and its control
`whileHeld(flipperArm, Button::Up, Button::Down, 8_volt));  // the flipper: Up out / Down back while held`.

**A pneumatic** (a valve on a 3-wire port; its stroke time measured at station 13) — robot one's front toggle
is this form:

```cpp
inline constexpr Pneumatic flipperAir{.name = "flipper", .positions = {{"stow", {Low}}, {"flip", {High}}},
                                      .start = "stow", .safe = "stow", .moveTime = 0.4_s};
```

its wiring `wire(flipperAir, lines('D')));` and its control `toggle(flipperAir, Button::B));`. Leave out
`.moveTime` and the build refuses it by name.

**A passive bar**: nothing to declare — type the kind and nothing else changes.

(The numbers in these blocks — a port, a gearing, a stall signature and cap, two angles, a stroke time
— are placeholders of the test that proves the forms, not robot one's. Step 1 above replaces every one.)

## Robot two: robot one's twin

Robot two was rebuilt as robot one's twin (the team lead, 2026-11-13: "IT IS LITERALLY THE SAME ROBOT
BUILD JUST A MIRRORED ROUTINE"). The two robots are one design in the code; only two things are each
robot's own — **its facts** (each robot is wired and measured on its own) and **one auton line**: robot
two runs the team's skills route MIRRORED across the field's skills diagonal, so in a skills run it works
the right wall's Quadrant while robot one works the red wall's
([cookbook chapter 9](../cookbook/09-one-route-two-robots.md)).

![The real Override field from above, red station at the bottom, with the skills diagonal y = -x drawn from the red/right corner to the blue/left corner. An example leg: robot one at (-36, -60) facing 90 drives up; robot two runs the same function mirrored, from (60, 36) facing 180. At the leg's end, what robot one reaches with its RIGHT side toggle is on robot two's LEFT](../assets/twin-mirror.png)

| | Robot one | Robot two |
|---|---|---|
| Mechanisms | its region of `src/robots/mechanism_table.hpp` | robot one's region, word for word, `robot_one` → `robot_two` |
| Facts | `src/robots/robot_one_facts.hpp` | `src/robots/robot_two_facts.hpp`: the same facts, names and stages — **every one OWED** until its builder reads it off robot two |
| Drive cartridge | BLUE (typed) | UNSET until read off robot two |
| Driver file | the builder's map | the same map |
| Autons | the 40 in, the acceptance run, the team's routes | the same — the routes marked mirrored **mirrored** (one line) |
| Calibration record | names `robot-one` | names `robot-two` |

Each robot-two placeholder is robot one's own value for that fact, so the host suite runs robot two
exactly as robot one. Until a fact is typed, every robot-two build that needs it refuses:
`robot two: OWED -- the cascade's motor ports (src/robots/robot_two_facts.hpp)`. Bench Tests and Calibrate
wait on its 17 Stage A facts, the other four programs on all 31 of Stage A and B. A robot-two fact is typed
like robot one's, keeping robot one's own PLACEHOLDER (`.placeholder` — it never changes, so typing robot
one later cannot upset robot two) beside robot two's value:

```text
inline constexpr auto kCascadePorts = typed(one::kCascadePorts.placeholder, Pair<int>{10, 1}, Stage::A,
    "the cascade's motor ports", "the team's builder, <date>, read off robot two");
```

**A change to one robot is a change to both.** `test/robot_twin_test.cpp` fails the host suite when the two
mechanism regions, the two facts files' names and stages, the two driver files (their controls, their feel
and their header maps) or the two autons lists differ anywhere but where the table above says. Change a
button in both driver files; add a mechanism to both regions.

Robot two's old build — two claw pivots on ports 6 and 7, a pneumatic claw on 3-wire A, and its measured
drive table — is retired. The host suite keeps it as its example robot with claws
([Chapter 16](16-testing-an-auton-on-the-bench.md#the-claws-in-an-auton)).

## What robot one does not do yet

- **Robot one's drive was measured by a Calibrate build that is not the standard one yet** (2026-11-17): one
  motor of its ten could not turn the robot, so the sides were found by which motors move together (the standard
  Calibrate's version of that is still being built). Its result is typed into `kRobotOneChassis` (the drive's
  ports, signs, scale and an effective track width of 13.23 in), and the autons drive slower and softer than the
  library's defaults (40 in/s, a heading arrival of 4 degrees — its ten geared motors rock about 2 degrees).
- **Station 13 does not know where the cascade's top stop is** (it has no soft stops: nothing is homed there).
  It applies each mechanism's current cap (since 2026-11-19): the cascade's 2.0 A and the side toggles' 1.5 A
  are written into their motors when you confirm each one and at every jog, and read back — the page's bottom
  line says `cap 2.0 A`. The motor holds its cap ([HA-219](../hardware-assumptions.md)), so R1 is safe on the
  cascade under it; still **never power it into its top stop**: approach it at 3–6 V and stop when it stops
  rising. If the bottom line says `cap 2.0 A NOT SET: keep 3 V`, a motor did not take it: keep the cascade at
  3 V and tell the coordinator. Station
  13's own cut-outs (two motors fighting, 2.4 A for 250 ms, the stall signature, a 3 s jog) stay on top of
  the cap. The side toggles declare no stall signature: run each only briefly, never held against its
  stop.
- **No camera, tracking wheels or wall reset**: the camera's port is written down but nothing reads it;
  the boot builds drive-encoder odometry only; robot one has no distance sensors.
- **No preset levels yet** — the builder's cascade is manual for now; until levels are typed, an auton
  moves it to a height with a constant target (`cascade.to(...)`), and more than six named levels is never
  possible (a level past six is a constant target too).
