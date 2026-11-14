# 21 — Adding your robot by hand

> **Covers:** how a second robot — robot one, the 2026 cascade robot — was added to this repository by
> hand: every file a robot lives in, its **facts file** (every number nobody has measured yet is written
> there once, marked **OWED**, and the build of every program that needs it **refuses**, naming it), its
> six programs and their slots, the order things happen on the robot, its default controls (yours to
> change), the flipper's three forms ready to paste, and why the two robots differ only in their facts.
> **Read this if:** robot one is on your bench, the build just printed `robot one: OWED -- …`, or you are
> adding a robot of your own.
> **Assumes:** [Chapter 7](07-getting-set-up.md) (building and uploading), [Chapter 18](18-mechanisms.md)
> (declaring mechanisms) and [Chapter 19](19-calibrating-your-robot.md) (Calibrate).

**Robot one does not exist in this project's records yet — nothing in this chapter has run on it.** No
census has seen it, no reading has been taken from it, no photo is on file. Everything below is what the
code is written and tested to do: robot one's cascade, its controls and its autons run in the host test
suite on the host test plant (the suite's model of a robot) on **placeholder** numbers, and its six
builds are checked by the src build gate. A placeholder can never reach the robot: until a fact is typed,
every build of a program that needs it stops with an error naming it.

(Chapter 20 is held for "Your robot file", the robot panel's chapter, when it lands — this one is the
hand-written path the panel will later write for you.)

## Where a robot lives in the code

Adding robot two touched these places one at a time over several weeks; robot one touched all of them at
once. Each line is what a robot needs — copy the pattern when you add yours.

| File | What the robot needs there |
|---|---|
| `src/build_selection.hpp` | its `RobotId` (`RobotOne`), one branch of THE ONE CHAIN per program (each sets the program, the robot and the build's beacon together), its robot beacon (`shulib-robot=robot-one`), and its define (`SHULIB_ROBOT_ONE_2026`) in each `PROGRAM=` guard above the chain — and, because it has mechanisms, in the Tune guard — without it that build stops at an `#error` naming why |
| `Makefile` | its `ROBOT=` word (`ROBOT=one`) and its define; the four `PROGRAM=` lines accept it; and — because it has mechanisms — its word in `MECHANISM_ROBOTS`, the list "shulib Tune" (`PROGRAM=tune`) accepts |
| `src/robots/chassis_table.hpp` | its drive table (`kRobotOneChassis`) — every unknown field left UNSET, and every station that needs one refuses by name. Calibrate discovers the drive's ports, directions, sides, scale and track width; it cannot discover the drive **cartridge** or the **IMU's port** — those two are typed here (the form's A2 and A4) before Calibrate runs |
| `src/robots/robot_one_facts.hpp` | **its facts file** — every number about its mechanisms, OWED or typed (below) |
| `src/robots/mechanism_table.hpp` | its mechanisms, in its own `namespace robot_one`, between its `[robot one mechanisms]` markers, every number read from the facts file; its line in `mechanismTableFor()` |
| `src/team/robot_one_driver.hpp` | its driver file — which button does what (a default, below) |
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

Every number about robot one that nobody has measured is written once, in
`src/robots/robot_one_facts.hpp`, like this:

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
| **A** — what is plugged where | the cascade's motor ports, their directions (as the builder sees them), the cartridge, the gear teeth, the **starting** stall signature and current cap (chosen, not measured), every motor on the Devices screen that is not a drive motor, the flipper's kind | **all six programs** — Bench Tests included, because it powers what is declared |
| **B** — how it moves | the cascade's measured rows (bottom stop, every level, top stop), its levels, its hold volts, its homing push, its volts before it is homed | every program **except** Bench Tests and Calibrate — today Drive, the Auton Tester, Competition and shulib Tune |
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
inline constexpr auto kCascadeTeeth = typed(Teeth{12, 60}, Teeth{12, 84}, Stage::A, "the cascade's gear teeth, motor then spool", "the builder, counted, 2026-11-14");
```

The build refuses three mistakes, each by name:

- **a typed fact with no source** — `robot one: a typed fact needs a real source …`;
- **a typed value still equal to its placeholder** (`owed(` flipped to `typed(` without writing the
  value) — `robot one: TYPED but still the placeholder …`, unless the source says
  `same as the placeholder, checked` because the builder really measured that number;
- **the src build gate's own marker** as a source (`gate copy`) — the gate proves robot one's builds
  with every fact typed in a throwaway copy of the tree, and a file copied back from there cannot link.

A number chosen at the bench (the starting cap, say) is typed with the source `chosen, not measured`.
A fact is never "corrected" to match a belief, and never invented to get a build through.

**After each typing, build robot two too.** Robot one's mechanisms compile in *every* build, so a typed
value that breaks one of the library's rules — a level above the top stop, rows out of order, fewer than
four named levels — stops robot two's builds as well as robot one's (loudly, by name). The src build gate
runs all fourteen builds; run it, or at least `make ROBOT=tank`, before uploading anything.

**Not a facts-file edit:** a third cascade motor, or the flipper's block — those are declarations, in
`src/robots/mechanism_table.hpp` (the number of motors is part of the declaration's shape).

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
homes the cascade, jogs it and moves it to its first and last level, so — like Drive — it needs Stage B: type a
hold voltage and a homing push first (measured, or "chosen, not measured"), then tune them with it. Each TEST
stops at 10 s ([Chapter 18](18-mechanisms.md#the-tune-program--tuning-on-the-brain-screen)): on the facts file's
placeholder gearing (a red cartridge through 12:60, 120° a second at the output) TEST B from the bottom to the
last level needs about 9 s of travel alone, so it ends **TIMED_OUT** short of the level, holding where it
stopped — a second TEST B from there finishes the move. With the real gearing typed, check it again.

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
| `.stall = {.amps = …, .belowMotorRpm = …, .lasting = …_s}` | `kCascadeStallAmps`, `kCascadeStallRpm`, `kCascadeStallTime` |
| `.currentCap = {.amps = …}` | `kCascadeCapAmps` |

**The three stall facts are HOME's signature too.** Robot one's declaration builds `.stall` AND its homing
signature from the same three facts, but Tune tunes them as two signatures, and a COPY prints a `.home` line
only when the homing push or the homing signature changed. So a `.stall` a session changed on its own comes
with no `.home` line — and typing it changes what HOME waits for, to a signature HOME was never run at.
After typing one, jog the cascade partway up in Tune and run **HOME** once; rely on it only when HOME ends
SUCCEEDED with the cascade resting on its bottom stop. An `.amps` above what the push draws at the stop ends
TIMED_OUT, never homed, and a stall-home trusts the first stall it sees
([Chapter 18](18-mechanisms.md#homing--every-moment-and-which-one-to-choose)). If the session tuned `.stall`
and HOME's signature to different numbers, give both lines to the coordinator (below).

**No fact holds these yet:** a `.tuning` block (kP, kI, kD, kS, kV, kA, the top speed, the acceleration,
"within", the settle time), a narrowed `.range`, or a `.home` whose stall signature differs from `.stall`
(robot one's declaration uses ONE signature for both). Keep those lines — they are in `/usd/tune_log.txt` —
and give them to the coordinator: where they go is a decision not made yet (new facts, or a marked block
the host suite allows). Until then robot one runs what its declaration says today: the library's starting
gains and profile, and the range from the bottom row to the top row.

## The order on the robot

1. **Read the brain's own Devices screen** (no program needed): every port 1–21 and every 3-wire port —
   the form's section B. The coordinator types Stage A into the facts file, and the drive **cartridge**
   and the **IMU's port** (the form's A2 and A4) into `kRobotOneChassis`, with who read them and when in
   its `provenance` — Calibrate refuses an unset cartridge, and nothing drives a tank without its IMU.
   **Type the IMU's port in the same edit as the motor ports (the form's B6 and C1)**: until they are typed,
   the facts file holds placeholder motor ports (8 and 9), and a real IMU on 8 or 9 beside them fails
   every build — robot two's too — as "a mechanism wired to the IMU port". Loud, never unsafe; typing the
   real ports clears it.
2. **Bench Tests** (slot 3): the census, then station 13 jogs each declared mechanism once to check which
   way it moves. **Calibrate** (slot 4): it discovers the rest of the drive by itself.
3. **Measure the cascade at station 13**: with it on its bottom stop, RECORD; then jog it up to each level,
   tape the height and RECORD each; then the top stop, **approached as the last section of this chapter
   says, never jogged into**. **Read the RECORD's `output … deg`, not its inches** — until Stage B is
   typed, station 13's inches come from the placeholder rows. The coordinator types Stage B.
4. **Drive, the Auton Tester, Competition, shulib Tune** — and your skills route.

## Robot one's controls — a default, yours to change

No robot-one program builds without a driver file (the library refuses a declared mechanism with no
control), so the library's builder wrote a starting map, robot two's feel. **From the day it landed it is
yours**: change any line in `src/team/robot_one_driver.hpp` and be able to say why.

| Input | Does |
|---|---|
| left stick | drive forward / back |
| right stick | turn |
| L1 / L2 | the cascade up / down at full power while held (before it is homed, at most its unhomed volts) |
| d-pad Down / Left / Up / Right | the cascade to its first / second / third / fourth level |
| B | home the cascade — it stalls gently down into its bottom stop |
| (driver start) | the first time driver control starts, the cascade homes |
| R1 / R2 | free, for the flipper (below) |
| Y | the drive loop's re-zero (field-centric is off) |

The levels are named by the facts file (`facts::lv(0)` is its first level), so renaming a level is a
facts edit, never a driver-file edit:

```cpp
inline constexpr DriverControls kControls = controls(kRobotOneMechanisms,
    whileHeld(cascade, Button::L1, Button::L2, 12_volt),        // up / down, full power
    preset(cascade[facts::lv(0).name], Button::Down, 2_s),      // the first level
    preset(cascade[facts::lv(1).name], Button::Left, 2_s),      // the second level
    preset(cascade[facts::lv(2).name], Button::Up, 2_s),        // the third level
    preset(cascade[facts::lv(3).name], Button::Right, 2_s),     // the fourth level
    homeOnPress(cascade, Button::B, 3_s),                       // home it now
    homeAtDriverStart(cascade, 3_s));                           // home it at the first driver start
```

Homing for a skills run is the route's to say: a route's first line `r.startHome(cascade, 2_s)` homes it
while the first leg drives.

## The flipper: three forms, ready

The builder has not said yet what the flipper is. Robot one's file declares **no flipper** until he
does — a wrong declaration is worse than none (Calibrate and the boot would read a motor arm's port on a
robot whose flipper is a pneumatic). All three forms are written and tested (each beside the real
cascade, bound and flipped on the host test plant). The day the kind is known:

1. **Each of the block's numbers becomes a fact first.** The facts file holds **no flipper number yet**
   — only its kind. A motor arm's port, cartridge, gear teeth, stall signature, cap, range and two angles
   (or a pneumatic's 3-wire letter and stroke time) are each added to the facts file as an OWED fact with
   its check, then typed from the form's item 17. A motor arm's are Stage A (station 13 powers it); its
   angles and a pneumatic's stroke time are measured at station 13, so Stage B.
2. **Then the block is pasted** under `cascade` in robot one's region, every number written as
   `facts::…` like the cascade's — never the literal numbers below. The host suite fails on any literal
   number or bare value word left in robot one's region; **`make` alone does not run that check**, so run
   the suite.
3. Its control goes into the driver file, and the flipper's kind is typed. The build then checks that the
   typed kind and the list agree.

**A motor arm** (a motor on its own port — that port is also a non-drive motor on the Devices screen):

```cpp
inline constexpr Arm flipperArm{.name = "flipper", .motors = {Forward}, .cartridge = Green, .gearing = gears(12, 36),
                                .safe = Hold, .stall = {.amps = 1.0, .belowMotorRpm = 5, .lasting = 0.3_s},
                                .currentCap = {.amps = 1.5}, .range = {0, 120}, .gravity = noGravity,
                                .home = restsAtStop(0), .presets = {{"stow", 0}, {"flip", 95}}};
```

its wiring `wire(flipperArm, motors(10)),` and its control
`whileHeld(flipperArm, Button::R1, Button::R2, 8_volt));  // the flipper: R1 out / R2 back while held`.

**A pneumatic** (a valve on a 3-wire port; its stroke time measured at station 13):

```cpp
inline constexpr Pneumatic flipperAir{.name = "flipper", .positions = {{"stow", {Low}}, {"flip", {High}}},
                                      .start = "stow", .safe = "stow", .moveTime = 0.4_s};
```

its wiring `wire(flipperAir, lines('A')));` and its control `toggle(flipperAir, Button::R1));`. Leave out
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
  3 V to just short of it and stop; close the last bit by hand only if the gearing turns freely by hand.
  If it does not, tell the coordinator — the top-stop row then waits for the capped station. Station 13's
  own cut-outs (two motors fighting, 2.4 A for 250 ms, the stall signature, a 3 s jog) are the only limit
  until then.
- **No camera, tracking wheels or wall reset**: the camera's port is written down but nothing reads it;
  the boot builds drive-encoder odometry only; robot one has no distance sensors.
- **More than six named levels** — a level past six is a constant target (`cascade.to(...)`) in an auton.
