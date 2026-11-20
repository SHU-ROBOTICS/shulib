# 17 — The competition program

> **Covers:** "shulib Competition" — the program you upload for a match: choosing an auton on the
> brain's menu, how `autonomous()` runs it on the program's own task, what ends it early (a disable,
> or the end of the autonomous period), the hand-over to driver control, what the screen and the
> serial log say in each phase, the drive protections' row and words, the three ways an auton is
> refused (and the entry that is dropped instead), and how to start a pretend match on the bench with
> no field.
> **Read this if:** you are uploading a program for a match, or running the bench worksheet's
> practice-match station.
> **Assumes:** [Chapter 16](16-testing-an-auton-on-the-bench.md) (the team's auton file and the
> tester, which runs the same call), [Chapter 9](09-the-recipe-api.md) (a routine as a chain of
> steps) and [Chapter 6](06-how-things-fail.md)'s "Running out of match".

## What it is

The competition program is the one you upload for a match. Build and upload it like the other
programs — robot two's build below; robot one's is `make ROBOT=one PROGRAM=competition`, the same slot
(each refuses, naming every owed fact, until its robot's facts are typed —
[Chapter 21](21-adding-your-robot-by-hand.md)):

```sh
rm -rf bin .d && make ROBOT=tank PROGRAM=competition
pros upload --slot 2 --name "shulib Competition" --description "$(git describe --always --dirty)"
```

The `rm -rf bin .d` is harmless, for the reason [Chapter 16](16-testing-an-auton-on-the-bench.md)
gives: `make` notices a changed `ROBOT=` or `PROGRAM=` word and starts fresh by itself (until it did,
it would link the last program you built under this name). **Slot 2.** Slot 8 holds the team's own old code, as a
fallback: never upload a shulib program over it. In VS Code, **Upload a program to a robot** → Competition does both
steps and says plainly whether it worked.

What happens in a match, in one paragraph: the robot boots from its calibration record; before the
match you choose an auton on the brain's screen (**NONE**, the default, runs nothing); when the field
starts the autonomous period, the chosen auton runs on the program's **own task** through
`competition::runAuton` — the same call the Auton Tester makes from its RUN — under a run guard
with a guaranteed end action; a **disable**, or the end of the autonomous period, ends it at once;
then driver control runs the robot's driver file — `src/team/robot_one_driver.hpp` or
`src/team/robot_two_driver.hpp`, one map on both robots ([cookbook chapter 6](../cookbook/06-driver-control.md)).

## Registering an auton

The competition program has no autons of its own. It registers the same autons the tester does — the
build's robot's list: robot two's `src/team/autons.cpp` (`registerAutons()`), robot one's
`src/team/robot_one_autons.cpp` — [Chapter 16](16-testing-an-auton-on-the-bench.md) walks through
them, and everything it says about an entry holds here unchanged. The three lines that decide when and
where an auton starts and stops, quoted from robot two's file (a SKILLS auton — the only competition on
November 21 is skills, a 60 s run):

```cpp
    forty.start = competition::StartPose::relative(); // where we place it is (0, 0), facing 0
    forty.endActionAt = 57_s;  // still running at 57 s? it is stopped, and the end action starts
    forty.hardStopAt = 59_s;   // everything stops: 2 s after 57 s, 1 s before the run's 60 s
```

### The two start modes

Every auton states where it starts, and the program seats both the position **and** the heading
there at the moment the auton starts — never at boot, because the choice is made after the robot
exists, and so the drift the IMU picks up while the robot waits on the field is thrown away.

- **RELATIVE** (the 40 in auton's): wherever you placed the robot is (0, 0), facing 0°.
- **FIELD**: the robot was placed at a known spot on the field, facing a known way, and its estimate
  starts there — so a later GPS or AprilTag fix agrees with it. An entry declares it like this:

```cpp
    competition::AutonEntry left{};
    left.name = "left side";
    left.kind = competition::AutonKind::Match;
    left.routine = &leftSide;
    left.start = competition::StartPose::field(-48_in, -24_in, 90_deg);
    left.endActionAt = 27_s;
    left.hardStopAt = 29_s;
    registry.add(left);
```

The coordinates are [Chapter 2](02-the-field-and-coordinates.md)'s field frame. Do not call
`startAt` inside a registered auton: its entry's `start` is the start, and `runAuton` seats it before
the routine's first line runs ([Chapter 16](16-testing-an-auton-on-the-bench.md) says why).

### The schedule, refused by name

At `endActionAt` the routine is stopped wherever it is and the **end action** runs (the brake, unless
the entry names another); at `hardStopAt` everything stops. A schedule that does not fit its period —
a MATCH auton's is 30 s, a SKILLS run's 60 s, the game manual's numbers, declared once in
`src/team/autons.hpp` — is refused **when it is registered**, by name:

```cpp
    late.endActionAt = 27_s;
    late.hardStopAt = 31_s;  // a MATCH auton's period is 30 s

    const competition::AutonVerdict verdict = registry.check(late);
```

`verdict.why` begins `auton 'left side late': ` and says what is wrong. The competition program (and
the tester) build their registry so that `registry.add()` of such an entry **drops that entry and
nothing else**: it is never on the menu, the menu names it in red while you choose (`REFUSED auton
'left side late': ...`), the boot logs its whole sentence (`DROPPED: auton 'left side late': ...`),
and every other auton and driver control still run — an auton bug must not cost the match's driver
period (the team lead's ruling, 2026-10-02). Fix the entry in the robot's autons file and upload again
— or, for the routes both robots share (the route builder's), in `src/team/skills_route.cpp`.
A registry built without asking for that (`competition::AutonRegistry registry{periods};`) still
throws the sentence at the `add()`. Leave a margin inside the period: a hard stop exactly on the
field's 30 s never gets a tick to act.

## Two robots, one route: a mirrored auton

In a VEX U skills run both robots run at once, both from red's side: one robot in each of red's two
Quadrants — here robot one in the **red wall's** and robot two in the **right wall's**, though the other way
round works the same, the mirror is its own inverse. Those two Quadrants are mirror images of each other across
the field's diagonal ([Chapter 2](02-the-field-and-coordinates.md#the-fields-mirror-two-robots-one-route)
draws it): `(x, y)` facing `h` is `(-y, -x)` facing `-90° - h`. So the students write the skills route
**once**, in robot one's numbers, as a function returning its entry, and each robot's autons file
registers it — robot one as written, robot two **mirrored**, in one line:

```cpp
// Robot one's autons file:
void registerRobotOneSkills(competition::AutonRegistry& registry) {
    registry.add(skillsRoute());
}

// Robot two's autons file -- the one line:
void registerRobotTwoSkills(competition::AutonRegistry& registry) {
    registry.add(competition::mirrored(skillsRoute(), competition::override2026::kSkillsMirror));
}
```

`skillsRoute()` is the students' function (its name, kind, routine, start and schedule) —
[cookbook chapter 9](../cookbook/09-one-route-two-robots.md) shows a whole one. The route is theirs;
the one line only says "run it mirrored". `mirrored()` copies the entry and marks it; mirroring it
twice gives it back as written.

The team's own files do this for every route the [route builder](22-making-your-first-route.md#9-more-routes)
writes: each autons file has one line, `addTeamRoutes(registry, TeamRobot::One);` in robot one's and
`addTeamRoutes(registry, TeamRobot::Two);` in robot two's, which mirrors each route marked mirrored.

**What the program does with a mirrored entry.** It seats robot two at the **reflected** start — its
REAL start, where the drivers must place it — and runs the routine AND its end action in a *mirrored
run*: every field number the route writes is reflected before the robot drives to it, and every
position the route reads back (`ctx.chassis.pose()`, a `face()`'s bearing, a `waitFor` on the pose)
comes back in robot one's numbers. So the route always sees robot one's field — and **everything the
robot REPORTS is robot two's real field**: the log, the screens, the localizer
(`ctx.chassis.deps().localizer->pose()`) and `lastCompleted()`. Only what the route reads through the
Chassis is robot one's ([below](#below-the-mirror-what-reads-the-real-field)).

### What the mirror changes, and what it does not

| The route writes | On robot two |
|---|---|
| the entry's FIELD start | seated at the reflection; the screen's `START MIR` line shows it |
| `startAt(pose)` / `setPose(pose)` | the position reflected (the heading stays the IMU's, as always) |
| `driveTo` / `reverseTo` / `face` / `faceAway` / `strafeTo` points, `moveTo` / `followTrajectory` poses, `turnTo` headings, `curveTo` / `reverseCurveTo` points and headings | reflected; every turn sweeps the OTHER way |
| `drive(speeds, Field)` / `drive(speeds, Body)` in a `then()` | the field vector reflected / the sideways speed negated; the turn reversed |
| the end action (the park) | reflected too — it runs inside the mirrored run |
| an entry that runs a `config::Route` (its `route`, not a `routine` — run by the path runner) | mirrored the same way — the path runner drives only through the Chassis |
| a wall reset (`competition::resetFromWall(ctx, ...)`) | the IMAGE wall and the OTHER side's sensor: robot one's red wall with its RIGHT sensor is robot two's right wall with its LEFT sensor |
| a Toggle step (it is handed `ctx.entry`) | the IMAGE Toggle — robot one's red wall Toggle is robot two's right wall Toggle, the same Quadrant colour, so "ours" does not change |
| timeouts, speeds, arrival tolerances, `hold`, `pause`, `waitFor` timeouts, a trigger's percent / inches / degrees, every mechanism step | unchanged — sizes, not field numbers |
| a RELATIVE start | mirrored in the robot's own frame instead, `(x, y)` facing `h` to `(x, -y)` facing `-h` — every left a right; right only if the drivers place robot two at the mirror image of robot one's spot |

**The one exception to "every turn the other way":** an EXACT half turn. The library always takes an
exact 180° turn counterclockwise ([Chapter 10](10-the-api.md)) — on both robots — so a mirrored `turnTo`,
`moveTo` or trajectory leg (a `driveTo` or `reverseTo` is a `moveTo`) whose heading changes by 180° (within
1°) may sweep the same way as robot one, on ANY drivetrain. It says so once, a `[MIR]` Warn
([Chapter 11](11-reading-the-diagnostics.md)). How much it matters, on the host test plant:

- a **turn** in place: the paths differ by 0.15 in, the ends by under 0.1 in — where the sweep matters,
  write it as **two 90° turns**;
- a **leg on a tank**: a tank only moves along its heading, so it slides sideways while it sweeps and ends
  BESIDE its target — and robot two, sweeping the same way, ends beside its own target on the side
  opposite robot one's image. The two misses add, and they grow with the leg: a `moveTo` 16 in ahead,
  facing back (180° from robot one's start), ended **5.7 in** from the mirror; the same leg 36 in long,
  **14.2 in** — roughly a third of the leg. A `driveTo` to a point straight behind is the same leg
  (**5.0 in** on 14 in). (A tank leg that starts that far off its heading is already the one the library
  warns about: `turn first: face()`.) **On a tank the pair check refuses that leg**
  ([below](#both-robots-at-once)): turn first, then drive;
- a leg on an X-drive: 0.31 in along the way, the ends within 0.02 in.

On a real robot an exact tie never happens — but the IMU's noise makes a leg that close to 180° a coin toss
for EACH robot, and on a tank the toss is as large as the tie: robot one's own end moves **5.8 in** between
a target 0.5° either side of 180° on the 16 in leg, **14.5 in** on the 36 in leg. So on a tank the pair
check ([below](#both-robots-at-once)) refuses any leg within 1° of 180°, not only the exact one. Turn first.

### What the screens and the log say

- The menu's button reads **`skills SKILLS MIR`** — so a mirrored auton's name is at most **25**
  characters (a longer one is refused when it is registered, by name).
- The result screen's START line reads **`START MIR (55.7, 36.0) -90.0 deg`** — robot two's REAL start;
  END, and every other number, is robot two's real field too.
- The log's `COMP` lines say `re-seated FIELD MIRRORED (55.70, 36.00)` and, on a second line,
  `MIRRORED across the skills diagonal -- robot one's start (-36.00, -55.70) facing 0.0 deg`; a `WALL`
  line says `(LEFT sensor, mirrored from red wall RIGHT)`; a `TGL` line names the real Toggle
  ([Chapter 11](11-reading-the-diagnostics.md) reads each).

### Below the mirror: what reads the REAL field

Inside a mirrored run, `ctx.chassis.pose()` is robot one's numbers — and these are robot two's real
field, NOT reflected: `ctx.chassis.deps()` (the localizer: `deps().localizer->pose()`),
`ctx.chassis.scheduler()`, `ctx.chassis.lastCompleted()`'s poses, the records and every log line. The
**camera's** field inputs are real-field numbers too: hand the piece finder or the Toggle tracker the
LOCALIZER's pose, never `ctx.chassis.pose()`; a real-field fix sent to a Chassis verb goes through
`ctx.chassis.reflection().apply(...)` first (a reflection is its own inverse), or the verb reflects it a
second time.

### Refused, by name

- the **explicit** wall reset (`chassis::resetFromWall(chassis, sensor, ...)`) inside a mirrored run — it
  is handed ONE sensor, and robot two needs the other side's: use the everyday call over `ctx`;
- an entry mirrored by two DIFFERENT mirrors (mirrored twice by the same one is the route as written);
- a mirrored name over 25 characters.

Because ONE function serves both robots, a refusal can only happen when its line runs — so the host
suite runs every auton robot two registers on the host test plant, mirrored ones included.

### Both robots at once

**The mirror's line is the seam between the two robots.** Whenever robot one's robot reaches the
diagonal, robot two's reaches the **same spot at the same moment** — and in a pure mirror the gap between
the two robots is exactly twice robot one's distance from the line, however their timing drifts. So:

1. **Keep robot one's WHOLE robot at least 3 in off the diagonal.** The robots then stay at least 6 in
   apart. For a 15 in robot that is its centre at least 10.5 in from the line when it is square to it,
   13.6 in when a corner points at it.
2. **What lies ON the diagonal is for ONE robot** — the centre goal, the two yellow Pins on it, the
   red/right corner's Load Zone. The shared route says which, with `isMirrored(ctx)` (true in robot two's
   mirrored run):

   ```cpp
       // The centre goal sits ON the mirror line: ONE robot goes there.
       if (!competition::isMirrored(ctx)) {
           r.face(-6_in, -6_in);
           r.driveTo(-6_in, -6_in);
           r.reverseTo(-12_in, -30_in);
       } else {
           r.pause(4_s);  // robot two waits on its own side meanwhile
       }
   ```

3. **The suite checks the pair.** For every mirrored route the team says both robots run at the SAME time
   (the route builder asks; `Together::Yes` in the routes' list), the host suite runs robot one's run and
   robot two's TOGETHER on the host test plant and fails — naming the moment, both positions and the motion
   each robot was in — when their 15 in squares come within 6 in:

   ```text
   auton 'P1 route': the two robots come within 6.0 in (5.76 in apart) at 10.02 s -- robot one at (-5.9, -12.7) in motion 8 (CurveTo ending at (12.0, -18.0)), robot two at (12.7, 5.9) in motion 8 (CurveTo). Keep robot one's whole robot 3.0 in off the mirror line, or send ONE robot there with isMirrored(ctx).
   ```

   Read it as: 10 s into the run, in its 8th drive motion — the `curveTo` whose last point is
   (12, -18), in robot one's numbers, the ones your route writes — robot one's robot reached within 3 in of
   the diagonal. Move that step's points onto robot one's side, or wrap it in
   `if (!competition::isMirrored(ctx))`. **The motion number counts DRIVE motions only, not your route's
   lines:** each `driveTo`, `reverseTo`, `face`, `faceAway`, `turnTo`, `moveTo`, `strafeTo`, curve, `hold`
   and brake is one, and so is each leg of a `followTrajectory`; a `then()` or a Toggle step counts the
   drives it runs; a wall reset, a `pause`, a `waitFor` or a mechanism step is none. So it is not the
   `step N` the routine's own lines print — find the step by its target. A RELATIVE mirrored entry's gap
   is skipped, by name — its field positions are wherever the drivers place the robots — and so is a
   route the robots run one at a time (`Together::No`). A mirrored route with a field start that nobody
   has answered yet fails the suite until it is answered.
4. **On a tank, the check also refuses one step that drives AND turns around** (within 1° of 180°),
   RELATIVE entries too. Each robot ends beside its target, on opposite sides of the mirror. The check
   names the step and the steps to write instead:

   ```text
   auton 'one step': motion 2 (MoveToPose to (-36.0, -55.7)) at 1.54 s turns 180.0 deg while it drives, which a tank cannot mirror -- turn first, then drive: face(-36.0, -55.7), then driveTo(-36.0, -55.7).
   ```

   ```cpp
       r.face(-36_in, -55.7_in);     // turn first,
       r.driveTo(-36_in, -55.7_in);  // then drive
   ```

   A step that ends facing the way it came gets `faceAway`, then `reverseTo`. The `face` is a half turn too, so
   robot two logs a `[MIR]` line for it — that one is fine. Not refused: a `turnTo`, a `moveTo` that
   only turns, a drive that can strafe.

**After an `isMirrored` branch the two runs differ in time**, and the check compares them at the
PLANNED timing only. A real robot's timing drifts (on the host test plant a 9 % weaker side moved one
robot 0.44 s over 21 s), so keep the other robot well away from the spot while one robot takes it — a
`pause` that ends with time to spare, not one that ends as the other robot leaves. **Before the event, run
the two robots TOGETHER on a field at least once**, the drive speed turned down first and someone at each
robot's stop, watching the diagonal (A4 register HA-260: the 3 in a side is a chosen margin, never
measured).

### What it cannot carry

The Loaders (they are only on the red and blue walls — robot two's mirrored spot has none), the red/left
corner's Load Zone (its image is the blue one), and **the robot itself**: a part mounted off the robot's
centre line meets the field on the other side of the reflected path (A4: HA-259), and each robot's own
left/right drive differences are its own (HA-260). The mirror swaps the robot's SIDES, not its mechanisms:
a step on robot one's right side toggle runs robot two's RIGHT toggle, which faces the other way on the
reflected path. So the shared route picks the side for the cascade robots' toggle pair —
`competition::isMirrored(ctx) ? leftToggle : rightToggle` — and the host suite proves that line reaches
robot one's right toggle and robot two's left one. Nothing here has run on a robot.

![The real Override field from above, red station at the bottom, the skills diagonal y = -x drawn. Robot one drives an example leg up the left side and reaches a target with its RIGHT side toggle; robot two runs the same function mirrored across the diagonal and meets the mirrored target on its LEFT side](../assets/twin-mirror.png)


## The selector

The menu on the brain's screen is the one the tester's PICK page shows: up to two autons per page
with `< PREV` and `NEXT >` below them, and **NONE** always in the middle of the bottom row, always in
the same place. Touch a name to choose it; the bar says `AUTON: <name>`. Two rules decide what runs:

```cpp
    competition::AutonMenu menu{registry, 0};
    CHECK(menu.choice() == -1);  // NONE: nobody touched the screen, so no auton runs
    const int running = menu.lockChoice();
```

- **NONE is the default.** A robot nobody chose an auton for sits still in autonomous. That is the
  safe failure; a robot that drives off because nobody chose is the dangerous one.
- **The choice is LOCKED when the auton starts**, in the same step that reads it: a touch that races
  the start cannot change the auton that runs, and no touch changes it until the run has ended.

The menu is up after the boot, while the robot is disabled, and when a field or a competition switch
connects. It is **not** up during driver control: driver control shows the driver page, and an auton
is never started from driver control.

## One task on the robot at a time

PROS runs `autonomous()` and `opcontrol()` on tasks it **deletes** when the match changes mode — a
deleted task stops mid-statement and runs no clean-up (the
[Hardware Assumptions Register](../hardware-assumptions.md), HA-161, records this belief; the
practice-match station checks it). An auton cut off that way would leave a motion half-finished and
the drive in whatever state it was last given. So the competition program never runs the auton on
PROS's task:

- `initialize()` builds everything once and starts **the auton task** — a task the program created,
  which PROS never deletes.
- `autonomous()` only **asks**: it hands the auton task a request and waits, checking every 10 ms
  tick.
- The auton task **serves** the request: it locks the choice, runs the auton through
  `competition::runAuton`, and then — on **every** path: an auton that finished, NONE, a refusal, an
  abandon, even a program error — publishes the request's **terminal word**, FINISHED. Until it
  does, no other task touches the robot.
- `opcontrol()` waits for that word with **no upper bound**, then runs the driver file.
- The driver file's loop ends on a disable, on the autonomous bit, and on a new auton request — and
  the auton task does not serve that request until the loop has **left** (or, if PROS deleted the
  task it ran on, until its once-a-tick heartbeat has stopped for 0.1 s). So the two never share the
  robot even if PROS did **not** delete `opcontrol()` at a change straight from driver control to
  autonomous; when the auton task had to wait for the loop, the run's lines say so.

**What ends an auton early.** Between the run guard and the brain's 10 ms tick sit the robot's
**mechanism service** — one object that ticks every mechanism's step once per tick, since the mechanism
core's fifth part ([Chapter 18](18-mechanisms.md)) — and, below it, the program's **watch**. Every tick it reads the competition status; the moment VEXos reports the robot
**disabled**, or the status leaves **autonomous** (the period ended, or the match moved straight to
driver control), it asks the run guard to **abandon** the run. The guard cuts the motion before the
world advances again — at most **one more commanded 10 ms tick** after the watch saw the change —
and in the same tick cancels every mechanism operation still running that registered its claim (as
the library's own do, since the mechanism core's second part), skips the end action (a disabled robot's motors are off; the brake cannot run), puts the drive at
0 V under Brake and every present mechanism in its safe state, and lets the routine unwind on
the auton task's own stack. Nothing is left dangling. A driver's bumped button does **not** end an
auton here (that is the tester's abort, never the match's).

**The one wait an abandon cannot cut.** The guard can only cut waits it owns: `guard.waitFor()` and
`guard.pause()` end the moment it abandons. `Chassis::wait()` and `Routine::pause()` run to their own
end. So a routine stopped inside a long `Routine::pause()` when the period ends holds the robot for the
rest of that pause, and driver control waits for it. **In an auton, pause with `guard.pause()`** —
[Chapter 9](09-the-recipe-api.md)'s match-clock section shows the idiom.

If driver control has to wait more than **0.25 s**, the program says so: the brain shows **AUTON
ENDING**, the controller's third line reads `AUTON ENDING`, and one log line names the auton that
has not handed the robot back. The wait still has no upper bound — a robot whose auton is still
running must not be driven by two tasks at once — and the 0.25 s only decides when it is said.

**The first auton after a boot is slow, not wrong.** The IMU calibrates for about 2 s after power-on.
An auton the field starts before that waits for a live estimate before it seats the start pose and
moves; the wait comes out of its own schedule.

## Every phase, on the screen and on serial

The program's serial lines carry `[CMP]`; the library's own lines keep theirs (`runAuton`'s `[COMP]`,
the run guard's `[SEQ]`, the motions' `[MOT]`). Only the auton task and the driver-control task paint,
one at a time: whichever holds the screen paints it, and PROS's phase tasks paint nothing.

| Phase | What the program does | The brain's screen |
|---|---|---|
| boot (`initialize()`) | builds the robot from the calibration record, binds its mechanism service and the driver's controls, registers the autons and the mechanisms' route ids, **proves its tick chain**, starts the auton task | the menu (or a refusal) |
| field connects (`competition_initialize()`) | one line on entry, one on return; hands the screen to the menu | the menu |
| disabled (`disabled()`) | the same | the menu |
| autonomous (`autonomous()`) | asks the auton task for a run, waits for its terminal word | the auton's **status**, then its **result** |
| driver control (`opcontrol()`) | waits for the terminal word, then the driver file until a disable, the autonomous bit or a new auton request | the **driver page** (or AUTON ENDING) |

**At boot**, serial carries a banner, the boot's decision lines, the session header, and three boot
lines — the build, how it booted, and the SD run log:

```text
boot: shulib COMPETITION [shulib-robot-variant=tank-competition] hash ...
boot source RECORD, 3 autons, 6 of 6 mechanisms present; autons ALLOWED
SD run log ON, APPENDED to /usd/shulib_auton_runs.txt
```

then one line per mechanism the robot could not build (`mechanism '...' is ABSENT -- the run guard
cannot safe it: ...`), the autons the registry dropped (`N auton(s) REFUSED at registration and
DROPPED -- NOT on the menu (each named below); every other auton and driver control still run -- fix
src/team/autons.cpp` — robot one's names `src/team/robot_one_autons.cpp` — then `(the route
builder's routes are fixed in src/team/skills_route.cpp)`, then `DROPPED: auton '...': ...` for each —
past eight, the line says `(the first 8 named below)` and a last line counts the rest), where the log copy is (`log
copy: /usd/comp_log.txt (APPENDED, kept across boots) -- ...`, or that it did not open and why: `NO
SD CARD`, or a card that is in and would not take the file), the mechanisms' lines — tag `MECH`, in
the words "shulib Drive" and the tester print too: for each mechanism what the robot file declares it
to be, where each part is plugged in and whether the brain found it, what the driver file's control
does with it, and anything it lacks — and one line for the route ids the autons' markers can fire:

```text
cascade: LIFT -- BLUE, motor 1 forward, motor 2 reversed; made safe: HOLD
cascade: motor 1 -> port 10 ok
cascade: motor 2 -> port 1 ok
cascade: hold-to-run R1 / R2, 12 V
cascade: homed when driver control starts, if nothing has homed it (3.00 s)
cascade: NOT HOMED -- a hold moves it at most 4.0 V with no soft stops; a preset or a move ends NOT_HOMED until it is homed (homeAtDriverStart, homeOnPress, or an auton's r.home)
right toggle: hold-to-run L1 (one way), 12 V
claw: toggle on A ('closed' / 'open')
flipper: toggle on B ('closed' / 'open')
ramp: toggle on X ('up' / 'down')
ROUTES: 11 mechanism route ids published (cascade_home .. ramp_down); 21 of the registry's 32 left for the team's own
```

(Robot two's, printed by the same functions on the host over its real declarations and driver file — on
its facts file's placeholders, robot one's numbers; robot one's read the same. Each mechanism's other
lines are the tester's, [Chapter 16](16-testing-an-auton-on-the-bench.md#what-the-boot-says-about-them).
The buttons are the driver file's, whichever program binds them.)

**Every phase function** logs one line when it starts and one when it returns normally, with the
brain's clock in milliseconds and the competition status in words:

```text
autonomous() entry at 61234 ms, status 0x06 field=yes disabled=no auton=yes
```

and the auton task logs every **change** of the competition status
(`competition status 0x05 field=yes disabled=yes auton=no at 76240 ms`) — within one tick while it
idles. It cannot log while it serves a run: a change during a run is logged after the run (after its
lines), and only if the status then differs from the last one it logged; the run's own first line
says which status ended it. So the **timely** record of a change is the phase functions' entry
lines — PROS starts the next phase function the moment the mode changes, and its first line carries
the milliseconds and the new status; the status that starts autonomous is in `autonomous()`'s.

**The status page** (before the auton moves): `AUTON: <name>` in an amber bar, **AUTON RUNNING**, the
entry's kind, schedule and start mode, and two reminders — the robot is the auton's until it ends,
and a disable or the period's end stops it at once. Nothing is painted while the auton runs.

**The result page** (after the terminal word): one word, big, and what happened. The words, in the
order the program checks them:

| Word | Meaning |
|---|---|
| `PROGRAM ERROR` | something threw — see "Refused, not run" below |
| `NONE` | no auton was chosen; nothing moved |
| `TOO LATE` | autonomous was already over when the auton task reached the request (a run held past its period went first); the status is read once before anything moves, and nothing moved |
| `REFUSED` | the auton was refused before anything moved (below), or `runAuton` refused it — the page says why |
| `ABANDONED` | a disable or the end of the period ended it; the run's first line says which, and the status byte |
| `ABANDON IGNORED` | the watch saw a disable but the run was not abandoned — a wiring fault the boot's chain proof exists to catch; **stop and report the run's lines** |
| `NOT STARTED` | the estimate never went live before the schedule ran out |
| `HARD STOP` | the hard stop fired (the end action did not finish in time) |
| `CUT BY DEADLINE` | the routine was still running at `endActionAt`; the end action ran |
| `DONE, 2 FAILED` | (amber) the routine ran to its end and the end action succeeded, but two steps written `onFail(carryOn)` or `onFail(skipTo(...))` failed on the way ([Chapter 9](09-the-recipe-api.md#choosing-what-a-failure-does-onfail-and-label)); `DONE, 9+ FAILED` for ten or more; the log's extra line names the first failed step |
| `FINISHED` | the routine (or route) ran to its end and the end action succeeded |
| `ROUTINE STOPPED` | a step failed and stopped the routine, a skip never found its step, or the routine went on past its failed steps but the end action failed; the routine's own lines say which |

Below the word: how it ended, the end action (`succeeded`, `FAILED`, `skipped`, or `CUT` when an
abandon landed during it — nothing is wrong with the brake then), where it was seated (`START`), the
**END** estimate, this run's faults (never faults from before the run), and that driver control
starts when the field says. In a real match the field disables the robot right after the period,
and the menu comes back over the result; the serial log keeps it.

**An auton ended early is read on serial.** The disable or the change of mode that ends a run also
starts the next phase function, which hands the screen over at once — to the menu, or to the driver
page — while the auton task is still unwinding, so the result page of an `ABANDONED` run (or of a red
`ABANDON IGNORED` one) is not painted, or shows only for a moment. Its word is in the run's first `[CMP]` line, and on the driver
page's auton row when driver control follows.

**The driver page** (driver control): the program's name and build time, then five rows — the
**drive protections** first (what explains a drive that stopped), the last auton's word (or why autons
are refused), the controller (`controller: CONNECTED`, or red `NO CONTROLLER -- the drive gets 0 V`),
the faults, and the mechanisms (the row begins `MECH`). The protections row (the library's protections run under the driver file in
every program — [Chapter 12](12-when-things-go-wrong.md) says what each does):

| Row | Means |
|---|---|
| `drive OK` (green) | every drive motor answers, nothing is cut |
| `drive OK -- frozen 13` (amber) | port 13's reading froze this tick — dropped 250 ms later if it stays frozen, or a train stopping |
| `DEGRADED dead 18 13` (amber) | those drive ports are dead (at boot, or dropped mid-run); the stick still drives |
| `CUT 1 s #2: FIGHT on LEFT -- port(s) 18` (red) | a protection cut: both sides at 0 V for 1 s, then driving resumes by itself |
| `REFUSED: 3 drive motors dead -- 0 V for the run` (red) | more than two dead: the drive stays at 0 V until the program is started again |

The controller shows the same three lines "shulib Drive" shows: `LIB 12V B12.9V`, then the dead drive
ports (`dead 18 13`) or else the heading, then the most urgent word — `AUTON ENDING`, `REFUSED see
brain`, `CUT see brain`, the last fault, `no controller 0V`, `DEGRADED see brain`, `MECH ABSENT` (a mechanism not built), or
`ok`, the first that holds. Both programs write those lines **one at a time**, at most one every
0.1 s, the word line first — PROS's own documentation says faster controller text updates are not
applied — so a full refresh takes up to 0.3 s. When driver control ends on a disable, the screen goes
back to the menu.

**Each auton run's lines.** Four `[CMP]` lines per run (two for NONE, TOO LATE, a refusal or a program
error; one more when the auton task first waited for the driver loop to leave, and one more — the
first failed step — when the routine went on past a failed step or a skip never found its step):

```text
request 1: '40in forward' FINISHED -- ROUTINE_RAN; in time, end action succeeded
request 1: scoring ended T+1.89s, end action ended T+2.25s, 225 paces, 0 cancels after the cut; hand-off 0.012s after autonomous(), live after 0.00s
request 1: routine OK, 1 of 1 steps done; faults this run: none
request 1: seated (0.00, 0.00) facing 0.0 deg (the IMU read 3.2); END estimate (39.64, 0.02) facing 0.1 deg LIVE
```

(The numbers are illustrative.) The **hand-off** is how long after `autonomous()` asked the run guard
armed — the auton's clock starts there. The `END estimate` is the robot's own belief, measured before
the terminal word; put it against the tape, as [Chapter 16](16-testing-an-auton-on-the-bench.md)
explains.

**The SD card** holds two files, each **appended** across reboots. The run log: the boot lines and
every run's lines go to the same file the tester writes, `shulib_auton_runs.txt` at the card's root —
one record across both programs — and a run's lines are written only **after** its terminal word. The
log copy, `comp_log.txt`: a copy of this program's log lines and summaries from the moment the brain's
devices are set up (the banner before that is on serial only) — the dead ports, every cut, the peak
currents, and why a boot was refused once the devices existed — but never the per-tick records serial
carries, so a practice match with no laptop still leaves its story behind. Each line of the copy is
written when it is logged, so a line logged during an auton (the auton's own event lines, a fault) or
during driver control (a protection's line) costs one card write in that tick; what that costs on the
brain is not measured. A loop that runs slow says so **once** for each slow stretch (`fault=LOOP_OVERRUN`
on its first slow tick), never once per tick, so a slow stretch never adds a card write to every tick
of it. When two parts of the program log at the same instant, the card takes one of the lines and
leaves the other off (serial keeps both), and the card's next line says so where it happened: `copy: <n>
line(s) left off here while another task was writing it -- serial has them`. One case ends the card's
copy instead: the program believes (not yet seen on a robot) that when the field disables the robot,
PROS stops the driver-control task wherever it is — and this program logs two lines just as its driver
loop sees that disable. If PROS stops the task in the middle of writing one of them to the card, the
card's copy **ends there for the rest of that boot** — no later line reaches the card, and no `copy:`
line says so — while serial keeps every line. The program does this on purpose: a card write PROS
stopped half-way may leave the card's file in a state nobody has measured, and the next task to touch
it could hang; a copy that stops never makes the driver loop wait. So a `comp_log.txt` whose boot ends
at a disable, while the robot ran on, lost the rest of that boot — read serial. How often it happens is
not measured. No card: serial only, never a refusal.

Serial keeps every line whole, even when two parts of the program print at the same instant: each line
goes out as ONE write to the USB, which PROS keeps in one piece (since 2026-10-08 — before, two lines
printed at once could mix; reasoned from PROS's C library, not yet seen on a robot either way). Serial
has one case of its own that nothing in shulib can cover, also believed and not seen: if PROS stops the
driver-control task at that same disable **in the middle of writing a line to serial** (a window of
microseconds per line), PROS's own serial lock is never released, and every later serial line of that
boot waits for ever — the program's first line in the next phase would wait with it, so the next driver
control's loop would never start: the robot would not answer the sticks. The card's copy stops at the same
line (it is written after serial, so it waits too): both logs end there. A capture that ends at a change
of mode while the brain still shows the program running is that case: restart the program. (The A4
register's HA-190 has the reasoning; the fix would be PROS's.)

## Refused, not run

There are three refusals, and each says why on the screen and on serial.

- **A refused boot.** When the robot cannot be described — no calibration record and no usable typed
  table, the card's record names another robot or none ([chapter 19](19-calibrating-your-robot.md)),
  the graph refused to build, the driver's controls do not fit the robot, or the boot's
  **tick-chain proof** failed (one idle tick must reach the run guard, the mechanism service and the
  watch, and the watch must be attached to the guard; otherwise no disable could end an auton, or no
  mechanism step would ever end — the refusal names which link is missing) — the program paints the red refusal screen, powers nothing and starts **no auton task**.
  Nothing paints over the refusal afterwards; each phase still logs its two lines, and
  `autonomous()` logs `the boot was REFUSED: no auton runs`. With no auton task there is no
  `competition status … at <ms>` line at all: serial carries only the four phase functions' entry and
  normal-return lines.
- **A boot from the typed table.** The typed chassis table's front may be the **other** end from the
  calibration record's, so an auton could drive the wrong way. Such a boot **refuses every auton** —
  the boot line says `autons REFUSED (a typed-table boot: run shulib Calibrate)`, the menu phase shows
  why instead of the menu, and an `autonomous()` runs nothing — but **driver control still runs**.
- **After a program error.** If anything throws while an auton is being served — a broken auton, a
  failed precondition, a log that throws — the auton task catches it, puts the drive at 0 V under
  Brake and every present mechanism in its safe state (each step on its own), logs one
  `PROGRAM ERROR` line in the throw's own words, and publishes the terminal word anyway. **Driver
  control still runs.** Every later auton on this boot is **refused** — the program's state is
  unproven after an unknown throw — until the program is restarted. The same refusal follows a throw
  caught while the auton task idles (painting the menu, logging a status change), or while it writes
  a run's lines after the terminal word; the robot was not the auton task's then, so nothing is
  commanded. The menu phase's page gives the reason the auton task recorded — what threw, and whether
  the robot was stopped or never touched — and the driver page's auton row says `restart`.

One more case is **not** a refusal of the boot: an auton entry the registry refuses at registration is
**dropped** — that entry alone. It is never on the menu, the menu names it in red, the boot logs its
sentence, and every other auton and driver control run (the schedule section above).

## Starting a pretend match on the bench

The team owns no competition switch. On the bench, the controller's **Timed Run** (from the
controller's own program menu) is the way to run a pretend match. What is **measured** is its start:
a program started from the controller's menu comes up **disabled**, in a pretend field-control match
(the [Hardware Assumptions Register](../hardware-assumptions.md), HA-128). Whether a Timed Run then
**enables autonomous**, after how long a disabled window (the window in which to choose an auton),
for how long, and what follows it, has **not been measured** — that measurement is the first step of
the bench worksheet's practice-match station (the same register, HA-162). If it never enters
`autonomous()`, the station records that and tries the controller's **Match** instead.

That makes the competition program the **one exception** to the rule that a program is never
started from the controller's menu: this program, in slot 2, at that station. Afterwards, turn the
controller off, wait ten seconds, and see the brain's menu return before starting any other program.

## What it does not do yet

- **The drive protections have never run on a robot.** Driver control runs them (the driver file, on
  the library's loop), but every number they use is a guess: whether a full-stick push against a wall
  CUTS the drive (a V5 motor pushing is believed to sit near its 2.5 A limit, above the 2.4 A cut —
  [HA-154](../hardware-assumptions.md), not measured) is what the bench
  worksheet's practice-match station's **push check** measures, and it is **required before this
  program drives a match**.
- **Route autons fire only the mechanisms' ids.** An auton written as a route (the path editor's
  kind) runs, and its markers can fire the robot's eleven mechanism route ids, as in the tester
  ([chapter 16](16-testing-an-auton-on-the-bench.md#the-claws-in-an-auton) shows one on the host
  suite's example robot with claws; the host suite runs it through this program's own request path
  too); a marker
  naming any other id is counted unknown and runs nothing — the team's own commands have no place to
  be registered yet. The team's file holds no route auton today.
- **The auton pick under a Timed Run** waits on the measurement above: if the Timed Run gives no
  disabled window before autonomous, how an auton is chosen on the bench is decided after it.
