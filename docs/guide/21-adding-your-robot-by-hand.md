# 21 — Adding your robot by hand

> **Covers:** how a second robot — robot one, the 2026 cascade robot — was added to this repository by
> hand: every file a robot lives in, its **facts file** (every number nobody has measured yet is written
> there once, marked **OWED**, and the build of every program that needs it **refuses**, naming it), its
> six programs and their slots, the order things happen on the robot, its six mechanisms and the builder's
> button map (yours to change), the flipper's three forms, and why the two robots differ only in their facts.
> **Read this if:** robot one is on your bench, the build just printed `robot one: OWED -- …`, or you are
> adding a robot of your own.
> **Assumes:** [Chapter 7](07-getting-set-up.md) (building and uploading), [Chapter 18](18-mechanisms.md)
> (declaring mechanisms) and [Chapter 19](19-calibrating-your-robot.md) (Calibrate).

**Nothing in this chapter has run on robot one yet.** The robot was built by 2026-11-15, and that night
its builder read its ports, cartridges, gearing and 3-wire letters off it and gave its button map — those
answers are typed into its facts file. No census, reading or photo of it is on file. Everything below is
what the code is written and tested to do: robot one's mechanisms, its controls and its autons run in the
host test suite on the host test plant (the suite's model of a robot) on the typed values and, where a
fact is still owed, its **placeholder**, and its six builds are checked by the src build gate. A
placeholder can never reach the robot: until a fact is typed, every build of a program that needs it
stops with an error naming it.

(Chapter 20 is held for "Your robot file", the robot panel's chapter, when it lands — this one is the
hand-written path the panel will later write for you.)

## Where a robot lives in the code

Adding robot two touched these places one at a time over several weeks; robot one touched all of them at
once. Each line is what a robot needs — copy the pattern when you add yours.

| File | What the robot needs there |
|---|---|
| `src/build_selection.hpp` | its `RobotId` (`RobotOne`), one branch of THE ONE CHAIN per program (each sets the program, the robot and the build's beacon together), its robot beacon (`shulib-robot=robot-one`), and its define (`SHULIB_ROBOT_ONE_2026`) in each `PROGRAM=` guard above the chain — and, because it has mechanisms, in the Tune guard — without it that build stops at an `#error` naming why |
| `Makefile` | its `ROBOT=` word (`ROBOT=one`) and its define; the four `PROGRAM=` lines accept it; and — because it has mechanisms — its word in `MECHANISM_ROBOTS`, the list "shulib Tune" (`PROGRAM=tune`) accepts |
| `src/robots/chassis_table.hpp` | its drive table (`kRobotOneChassis`) — every unknown field left UNSET, and every station that needs one refuses by name. Calibrate discovers the drive's ports, directions, sides, front, scale and track width, and finds the **IMU** on the brain's own device list; the one drive fact it cannot discover is the drive **cartridge** (the form's A2) — typed here before Calibrate runs (robot one's: BLUE, 2026-11-15) |
| `src/robots/robot_one_facts.hpp` | **its facts file** — every number about its mechanisms, OWED or typed (below) |
| `src/robots/mechanism_table.hpp` | its mechanisms, in its own `namespace robot_one`, between its `[robot one mechanisms]` markers, every number read from the facts file; its line in `mechanismTableFor()` |
| `src/team/robot_one_driver.hpp` | its driver file — which button does what (the builder's map, below) |
| `src/team/robot_one_autons.cpp` | its autons — a starter file: the 40 in as a SKILLS auton and the library's acceptance run. **The skills route is yours to write.** |
| `src/common/this_robot.hpp`, `src/common/robot_words.hpp` | nothing to type: the programs run THIS build's driver file and autons (`ThisRobotDriver`, `registerThisRobotsAutons()`) and print THIS robot's words — robot one's screens say `MECH`, never `CLAW` |
| `test/CMakeLists.txt` | its autons file, so the host suite runs every auton in it |
| `tools/src_build_gate.py` | its six builds |

**Robot two is the worked example and was not changed.** Its tables, driver file and autons are byte for
byte what they were, and the eight robot-two, bench and X-drive packages carry the same machine code and
the same constant data as before (compared function by function and object by object) — and every robot-one line of
`src/robots/mechanism_table.hpp` sits
*below* robot two's region, because each of robot two's declarations records the line it is written on
and a line added above it would change every robot-two package.

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
package is still in `bin/`. It carries no placeholder.) Every other build — robot two's, the bench
bot's, the host suite — is untouched by an owed fact.

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

**After each typing, build robot two too.** Robot one's mechanisms compile in *every* build, so a typed
value that breaks one of the library's rules — a level above the top stop, rows out of order, a preset in
the driver file bound to a level that has no name — stops robot two's builds as well as robot one's
(loudly, by name). The src build gate
runs all fourteen builds; run it, or at least `make ROBOT=tank`, before uploading anything.

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
[Chapter 7](07-getting-set-up.md)). Robot two's builds keep `ROBOT=tank`. **shulib Tune** tunes the cascade's
numbers live on the brain screen ([Chapter 18](18-mechanisms.md#the-tune-program--tuning-on-the-brain-screen)); it
homes the cascade, jogs it and moves it to its first and last level — with fewer than two levels named (robot
one's today: manual, no levels), to a quarter and three quarters of its range — so, like Drive, it needs Stage B:
type a hold voltage and a homing push first (measured, or "chosen, not measured"), then tune them with it. Each
TEST stops at 10 s ([Chapter 18](18-mechanisms.md#the-tune-program--tuning-on-the-brain-screen)); a TEST that
runs out of time ends **TIMED_OUT** holding where it stopped, and a second TEST finishes the move. (On ROB1-A's
placeholder gearing — a red cartridge through 12:60 — that happened on every long move; the builder's blue
cartridge driving the spool directly turns it thirty times faster.)

### Tune's COPY on robot one: each number goes to its fact

Robot two's COPY lines are written into its declaration by hand
([Chapter 18](18-mechanisms.md#the-tune-program--tuning-on-the-brain-screen)). **Robot one's are not**: its
declaration reads every number from the facts file, and the host suite fails a number typed straight into
robot one's region ("a literal number in robot one's mechanism region"). So each number a COPY prints is
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
3. **Measure at station 13**: with the cascade on its bottom stop, RECORD; then (once there are levels)
   jog it up to each level, tape the height and RECORD each; then the top stop, **approached as the last
   section of this chapter says, never jogged into** — two rows, the bottom stop and the top stop, are
   the least the library takes. **Read the RECORD's `output … deg`, not its inches** — until Stage B is
   typed, station 13's inches come from the placeholder rows. Run each side toggle once — briefly, and
   never held against its stop: station 13 runs it **uncapped** (below) — and say which way is "run"; fire
   each piston once and say whether it went open (down, for the ramp) or closed. The coordinator types
   Stage B.
4. **Drive, the Auton Tester, Competition, shulib Tune** — and your skills route.

## Robot one's mechanisms (typed 2026-11-15)

The builder's answers, as the facts file holds them — every line `typed(…, "the team's builder, 2026-11-15,
read off the robot")`, or a starting value `"chosen, not measured -- …"`:

| Mechanism | What it is | Typed | Still owed |
|---|---|---|---|
| **cascade** | a `Lift`: two **blue** motors on ports **10 and 1**, facing each other, **direct** to the spool (typed as the 1:1 tooth count `gears(1, 1)`); a 4-stage string cascade, homed by stalling gently down | ports, directions (the builder: they face each other; port 10 taken as Forward, station 13 confirms), cartridge, gearing; the coordinator's starting cap **2.0 A** per motor and stall signature **1.5 A below 5 rpm for 0.3 s**; its own homing signature **0.45 A** (chosen, not measured — a gentle push's stall); its levels: **none yet** ("for now it will be manual") | its two rows (station 13: the bottom stop and the top stop), its hold volts, its homing push, its volts before it is homed |
| **right toggle**, **left toggle** | two `Power` mechanisms — each its own arm on its own **green** motor, direct — that RUN one way while their button is held and stop on release; no angles, no homing | their ports **9 and 2**; cartridge; a starting cap of **1.5 A** per motor (chosen, not measured, to bound the push held against a stop; written into the motors by Drive, Competition and the Auton Tester — not by station 13) | **which port is the right one** (the left is the other); which way each runs |
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
yours**: change any line in `src/team/robot_one_driver.hpp` and be able to say why.

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
never a second button.

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

**The Drive screen's mechanism row** is compact on robot one — six mechanisms do not fit the screen's
50-character row in the long form robot two's two claws use:

```text
MECH 10H34 9B31 2B30 CLAW:C FLIP:O RAMP:U
```

Each motor mechanism by its first motor's port, then a letter for what it was last told — `F` forward, `R`
reverse, or its stop: `H` hold, `B` brake, `C` coast — and its temperature in °C (`10X`: not found at boot).
Then each piston by the first four letters of its name and the first letter of its position (`C`losed,
`O`pen, `U`p, `D`own; `?` its lines match neither, `X` not found). **`FLIP` is the front toggle** — its
declared name is `flipper`, the name the flipper check looks for. ` FREED` at the end means what it means
on robot two's row.

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

## Two robots, one design

The builder says both robots are built alike. Each robot's mechanisms are written the same way from its
own facts file, so when robot two is rebuilt as robot one's twin, its region becomes robot one's copied
with its own facts file, and a test will fail if the two ever differ anywhere but their facts. Until
then robot two stays exactly as it is.

## What robot one does not do yet

- **A calibration record names no robot** — both robots read `shulib_drivetrain.bin` from their own SD
  card, and the boot recognises a card moved between them only by its ports. Keep one card per robot,
  labelled, until the record carries the robot's name.
- **Station 13 does not apply the cascade's current cap yet.** Until it does, keep its ceiling at 3 V on
  the cascade (never press R1 to raise it) and **never power the cascade into its top stop**: jog up at
  3 V to just short of it and stop; close the last bit by hand only if the cascade moves freely by hand
  (its motors turn with the spool: no gears). If it does not, tell the coordinator — the top-stop row then
  waits for the capped station. Station 13's
  own cut-outs (two motors fighting, 2.4 A for 250 ms, the stall signature, a 3 s jog) are the only limit
  until then. **The side toggles are uncapped there too** (and declare no stall signature): run each only
  briefly to see which way it goes, never held against its stop.
- **No camera, tracking wheels or wall reset**: the camera's port is written down but nothing reads it;
  the boot builds drive-encoder odometry only; robot one has no distance sensors.
- **No preset levels yet** — the builder's cascade is manual for now; until levels are typed, an auton
  moves it to a height with a constant target (`cascade.to(...)`), and more than six named levels is never
  possible (a level past six is a constant target too).
