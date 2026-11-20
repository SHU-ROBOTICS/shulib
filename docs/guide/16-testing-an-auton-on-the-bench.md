# 16 — Testing an auton on the bench

> **Covers:** "shulib Auton Tester" — the program that runs one of the team's autons on the bench,
> from the brain's screen, with no competition switch; each robot's autons file
> (`src/team/robot_one_autons.cpp`, `src/team/autons.cpp`) and the routes both share
> (`src/team/skills_route.cpp`), which it and the competition program both read; what to expect from the
> first run on a tank, and how to read the result against a tape measure; and previewing a whole
> route on the laptop before it goes on the robot (`tools/route_preview.py`).
> **Read this if:** you are about to run an auton on either robot, or you are writing the next one.
> **Assumes:** [Chapter 9](09-the-recipe-api.md) (a routine as a chain of steps) and
> [Chapter 4](04-drivetrains.md)'s section on what a tank can and cannot do.

## What the tester is, and what it is not

An autonomous routine ("auton") runs in a match when the field starts the autonomous period. The
team's rule, from the start of this season, is that autons run **only** through the competition
program's real `autonomous()` — never through a shortcut hidden in some other program, because a
shortcut is a second path, and a second path is the one that differs on match day.

The tester keeps that rule by running the auton through **exactly the same library call** the
competition program makes — `competition::runAuton`, on the same registered entry, with the same
schedule, the same start-pose re-seat and the same guaranteed end action. Only the **trigger**
differs:

- in a match, the field (or the controller's Timed Run) starts the autonomous period;
- on the bench, you touch **RUN** on the brain's screen, and a 3 second countdown gives you time
  to step back.

So what the tester shows you is what the library will do in the match, from the moment the
auton starts to the moment the end action finishes. What it **cannot** show you — and a practice
match on a real field (or the controller's Timed Run) still must — is everything around that: a
field disabling the robot mid-auton, the hand-over from autonomous to driver control, whether
the controller's Timed Run really enters `autonomous()`, and the first moments after a mode
change. Both programs also hand the call the same two things an auton needs for the robot's
**mechanisms** (since the mechanism core's fifth part, 2026-10-29): the robot's **mechanism
service**, so a mechanism step in an auton (the cascade, a piston) runs and is judged exactly as it will be
in the match, and a **command registry** holding the robot's mechanism **route ids** — so an auton written
as a *route* (the path editor's kind, with markers) can fire the robot's mechanisms from its markers. A marker naming
any other id is counted as unknown and runs nothing: the team's own commands have no place to be
registered yet.

**The competition program ("shulib Competition") calls the same `registerAutons()` and the same
`competition::runAuton`** — [Chapter 17](17-the-competition-program.md) describes it.

For a route from the route builder, the easiest is its **Send to robot** button (opened from VS Code: [chapter 22,
step 7](22-making-your-first-route.md#7-send-it-to-the-robot)): it runs `tools/upload_auton.py`, below, for you.

Build and upload it like the other programs — robot two's build below; robot one's is
`make ROBOT=one PROGRAM=autontest`. Each robot's build refuses, naming every fact of its facts file it
still owes, until those facts are typed ([Chapter 21](21-adding-your-robot-by-hand.md)):

```sh
rm -rf bin .d && make ROBOT=tank PROGRAM=autontest
pros upload --slot 6 --name "shulib Auton Tester"
```

Or both in one command that says plainly whether it worked — `python3 tools/upload_auton.py two` (`one` for
robot one; `python` on Windows), in the PROS terminal (VS Code: PROS, then Integrated Terminal) or any VS Code
terminal: when `make` or `pros` is not on the PATH, it takes them from the PROS toolchain's own folder, as the PROS
extension's build does. It needs Python 3 ([python.org](https://www.python.org/downloads/)); without it, use the two
commands above. It prints **✓ BUILT** and
**✓ UPLOADED to slot 6** with the route names to pick on the brain, or **✗** and the first real error in plain words:
the facts still OWED, a compile error's file and line (inside the route builder's lines: re-export), or "plug the
brain in with USB (not the controller)". Everything make and pros printed is in `build/upload_auton.log`. It has run
against stand-ins for `make` and `pros` only: not yet against the real `pros`, or on Windows.

`make` notices a changed `ROBOT=` or `PROGRAM=` word and starts fresh by itself: it deletes `bin/` and
`.d/` and recompiles everything (it prints `shulib: the build changed …` when it does). Until it did,
after building another program it recompiled nothing and linked **that** program again — and the
upload put it in the tester's slot under the tester's name. The `rm -rf bin .d` is now harmless, and it stays in
the line above for anyone on an older copy of this repository.

(Slot 2, since the new "shulib Drive" retired "shulib Teleop" from it; until then the tester was
in slot 7, so a brain uploaded before that may still hold it there — slot 7 is "shulib Tune"'s now,
and uploading Tune replaces it.) Start it from the **brain's**
screen, with the controller paired and on its home screen — started from the controller's menu it
would be a pretend match, and the tester refuses to run an auton in any competition mode.

## The team's files: the autons

Every auton a robot can run lives in a file the team owns, and one function at its bottom lists them in
the order the brain's menu shows them: **robot two's** is `src/team/autons.cpp` (`registerAutons()`),
**robot one's** `src/team/robot_one_autons.cpp` (`registerRobotOneAutons()`). Both programs — the tester
and the competition program — call the build's robot's list; neither has an auton of its own. The two
robots are one build (robot two is robot one's twin), so the two files hold the same autons — with ONE
difference, the team lead's: robot two runs each of the team's routes marked mirrored — the **skills route**
among them — mirrored across the field's diagonal ([cookbook chapter 9](../cookbook/09-one-route-two-robots.md)).
The routes themselves live once, in `src/team/skills_route.cpp`, shared by both. The host test suite compiles all three files and runs every
auton in them on the host test plant through the tester's own run sequence, so a change to an auton is
tested before it is uploaded.

The first hand-written auton drives 40 inches straight ahead. Here it is, quoted from robot two's file (the
build fails if this listing and the file ever disagree):

```cpp
// ── THE 40 IN AUTON (robot one's, copied) ──────────────────────────────────────────────────
// Drive straight forward 40 inches and stop, facing the way the robot started.
chassis::RoutineResult fortyInchesForward(competition::AutonContext& ctx) {
    chassis::Routine r{ctx, "40in-forward"};
    r.moveTo(math::Pose2d{40_in, 0_in, 0_deg});
    return r.result();
}
```

It is one [Routine](09-the-recipe-api.md) with one step: ONE `moveTo` — the library drives to the POSE
40 in ahead, 0 in to the side, facing 0 degrees, and the step ends when the robot is there and still, or
on the motion's own time limit. The robot's coordinates are the ones from
[Chapter 2](02-the-field-and-coordinates.md): x straight ahead, y to the left, angles counterclockwise.
And the list at the bottom of the file:

```cpp
void registerAutons(competition::AutonRegistry& registry) {
    // 1. THE 40 IN AUTON, as a SKILLS auton (the 60 s autonomous coding skills run).
    competition::AutonEntry forty{};
    forty.name = "40in forward";                      // what the brain's menu shows
    forty.kind = competition::AutonKind::Skills;      // runs in the 60 s skills period
    forty.routine = &fortyInchesForward;              // the function above
    forty.start = competition::StartPose::relative(); // where we place it is (0, 0), facing 0
    forty.endActionAt = 57_s;  // still running at 57 s? it is stopped, and the end action starts
    forty.hardStopAt = 59_s;   // everything stops: 2 s after 57 s, 1 s before the run's 60 s
    // endAction is left out: the end action is chassis.brake() -- 0 V under Brake until still.
    registry.add(forty);

    // 2. THE LIBRARY'S ACCEPTANCE RUN (drive 24 in, turn to 90 degrees, face the start, drive back): the
    //    test that the whole library works on this robot, on the skills schedule.
    registry.add(competition::acceptanceRunEntry(competition::AutonKind::Skills, 57_s, 59_s));

    // 3. THE TEAM'S ROUTES (src/team/skills_route.cpp, named in the route builder): each route marked mirrored for
    //    robot two MIRRORED across the skills diagonal, the others as written.
    addTeamRoutes(registry, TeamRobot::Two);
}
```

Robot one's list is the same with `addTeamRoutes(registry, TeamRobot::One);` — every route as written — on its last
line. The routes are the ones the team named in the route builder ([Chapter 22](22-making-your-first-route.md)), the
skills route among them.
Every entry is a SKILLS entry: the only competition before the 21st is skills (the team lead's ruling).

Each entry is **data**: a name (at most 29 characters, shown as `AUTON: <name>`; 25 for a mirrored
entry — [Chapter 17](17-the-competition-program.md#two-robots-one-route-a-mirrored-auton)), a kind (a 30 s
match period or a 60 s skills run — the periods are the game manual's, declared once in
`src/team/autons.hpp`), the routine, where it starts, and its schedule. The schedule is the part a
new author gets wrong most often, so read it slowly: at `endActionAt` the routine is stopped
wherever it is and the **end action** runs (here the brake); at `hardStopAt` everything stops,
end action included. A hard stop that landed exactly on the run's 60 s would never get a tick
to act, which is why 59 s. An entry whose hard stop does not fit its period is **refused by name
when it is registered** — and **dropped, that entry alone**: it is not on the menu (the menu names it
in red, `REFUSED auton '<name>': ...`), the boot logs its whole sentence (`DROPPED: auton '<name>':
...`), and every other auton still runs. The route builder's routes are not in the autons file: both
robots take them from `src/team/skills_route.cpp`, and the boot's line after the count says so (`(the
route builder's routes are fixed in src/team/skills_route.cpp)`). That is the team lead's ruling for the competition program
(2026-10-02: one bad auton must never cost the rest, or a match's driver period); the tester does the
same, so a student fixing one auton can still test the others.

**RELATIVE start** means: wherever you place the robot is (0, 0), facing 0°, at the moment the
auton starts — the library re-seats both the position and the heading there, so any drift the
IMU picked up while the robot sat on the READY page does not leak into the auton.

That is also why the 40 in auton has no `startAt` line, although [Chapter 9](09-the-recipe-api.md)
calls `startAt` an auton's first line: a **registered** auton's start is its entry's `start`, and
`runAuton` seats it before the routine's first line runs ([Chapter 3](03-knowing-where-you-are.md)
explains the seat). Do not call `startAt` in a registered auton; set its entry's `start` instead.

**"Ahead" is the robot's FRONT as the calibration record defines it**: the end that *led the
push* when "shulib Calibrate" ran. On robot two today that is the camera end — and it is **not**
the end the typed chassis table calls the front. That is why the tester refuses RUN unless the
robot booted from the calibration record, and why the bench worksheet's first step is a push
test: push the robot a few inches toward the end you believe is forward and watch the estimate's
x **rise** on the READY page. If it falls, stop — the auton would drive toward you.

## The pages

**PICK** is the same selector the competition program shows (`NONE` in the middle of the
bottom row, the autons above it). Touch an auton's name.

**READY** says everything you need before the robot moves, one line each:

```text
MATCH park T+27.0s stop T+29.0s RELATIVE
BOOT RECORD: forward = the end that led Calibrate
EST x 0.0 y 0.0 in, facing 0.0 deg LIVE
READY: touch RUN, then step back
RUN counts 3 s, then it moves. Step back.
ANY BUTTON or a touch of the screen aborts.
SD: runs appended to shulib_auton_runs.txt
faults: none  BUILD 14:05:31
```

`RUN` (bottom right) is grey until the page has been up half a second **and** all of these hold:
the estimate is live, a controller is connected, the robot is not disabled and nothing
competition-like (a field, a switch, a Timed Run) is connected, and the robot booted from the
calibration record. When one fails, the fourth line says which — and RUN is checked again at the
moment your finger lifts, not only when the page was drawn. `< BACK`
(bottom left) goes back to PICK. The `BUILD` time is when the program was compiled — the quickest
way to see that your upload actually landed.

**The countdown** shows STEP BACK with a count of 3, 2, 1. Nothing is commanded during it.
**Then it runs.**

**RESULT** shows one word — `FINISHED`, `DONE, 2 FAILED` (amber: the routine ran to its end,
carrying on past two failed steps it marked `onFail(carryOn)` or `onFail(skipTo(...))`, [Chapter
9](09-the-recipe-api.md#choosing-what-a-failure-does-onfail-and-label); ten or more read
`DONE, 9+ FAILED`, and the log names the first failed step on a line of its own),
`ROUTINE STOPPED`, `ABORTED`, `ABORT IGNORED`,
`CUT BY DEADLINE`, `HARD STOP`, `NOT STARTED`, `REFUSED` or `PROGRAM ERROR` — and, for a run, six
lines: how it ended, the end action and the guard's clock, where it was seated, the **END**
estimate, the faults raised during this run (never faults from before RUN), and a reminder to
measure. `< PICK` (bottom left) and `AGAIN` (bottom centre — never where RUN was, so a double tap
cannot start a second run) go back.

`ABORT IGNORED`, in red, should never appear: it means a button or a touch was seen but the run
was not stopped for it — the abort's wiring is broken, which is a build error, not a robot fault.
The tester checks that wiring when it boots and refuses to start without it; if you ever see this
word, stop using the tester and report the run's lines.

## Aborting

**Any controller button**, or **a touch of the screen**, aborts — during the countdown (then
nothing ever moves) or during the run. So does the controller switching off or losing its link,
and VEXos disabling the robot. The sticks do not abort: a thumb resting on a stick must not end a
run; a button press is deliberate. RUN is refused outright without a connected controller, because
without one the abort has no hands.

An abort during the run stops the motion **within one more 10 ms tick** of the abort being seen,
skips the end action — or cuts it short, if the brake had already started (RESULT then says
`end action CUT`; nothing is wrong with the brake) — and leaves the drive at 0 V under Brake, with
every mechanism in its safe state. The abort is read when a finger **lands** on the screen,
not when it lifts, and any finger that lands after your RUN touch lifted counts, however soon.
Touch the **top half** of the screen to abort: no button on the countdown, running or RESULT page
is there, so the finger that aborted cannot also press something on the RESULT page. (If you keep
holding, RESULT keeps its buttons grey until you lift, then arms them half a second later.)

## Reading the result against the tape

The 40 in auton drives 40 inches **by the robot's own estimate** — its drive encoders and its IMU.
The tape measure is how you find out whether the estimate is right. If the tape says more or less
than 40 in, the calibration record's wheel scale is off by that ratio (a record that reads 44 in for
a taped 48 in is about 8 % short, and its 40 in auton measures about **40 × 48 / 44 ≈ 43.6 in**).
That is the calibration record being wrong, not the auton: the fix is a careful Calibrate rerun on
that robot, **never** changing the 40 in the file to make the tape agree. Every auton the team
writes afterwards would inherit the fudge.

**What to expect on a tank.** A tank cannot drive sideways, and no robot's two sides are exactly
equally strong (the old robot two veered right over 48 in with its motors driven open-loop). Since
API 2.3, `moveTo` on a tank steers back onto its line while it drives — the file's own comment
above says how, and [Chapter 4](04-drivetrains.md#limits--what-limited-strafe-does-to-your-commands)
explains it in full. So the result to expect is **`FINISHED`**, with `END` x near 40 and `END` y
within half an inch, and `TSTR` on many of the tick lines in the serial log (the library was
steering). On the host test plant the 40 in finishes with one side up to about a quarter weaker —
the old robot two's was nearer a tenth — but that is the host test plant's number, not a promise for
the robot, whose motors respond with a lag the plant does not have.

If it **cannot** arrive within half an inch and about a degree, the step ends after its time limit
— 5 s, the robot's configured default, itself still a provisional number — and the result reads
**`ROUTINE STOPPED`** with `faults this run: 1, last MOTION_TIMEOUT`, and the brake still runs. That
is still the library telling the truth about the robot. Read `END` x **and** y first — how far
short and how far beside it stopped — and look at the `TSTR` lines, before touching the auton,
the tester or any tolerance.

**And a `FINISHED` is checked against the tape, not taken as a pass.** If the robot's wheels skid
sideways, its encoders and IMU cannot see it — so the estimate, the steering and the result word
never see it either. A skid shows up only as a `FINISHED` whose sideways offset on the tape
disagrees with `END` y. That is why the worksheet has you measure both.

## The SD card

Every run's summary lines — the same words the serial log's `[ATS]` lines carry — are **appended**
to one text file on the card, `shulib_auton_runs.txt`, at the card's root (open it on any computer).
It is one file across reboots, so a session that spans a re-upload keeps every run. The third of
the serial boot lines says `SD run log ON, CREATED /usd/shulib_auton_runs.txt` the first time the
tester boots on a card without the file, `ON, APPENDED to /usd/shulib_auton_runs.txt` when the file
was already there, or `OFF` with no card — and with no card the tester still runs; it just logs to
serial only. Never format the card to start a fresh log: the calibration record is on it too, and
without the record the tester refuses to run. To start fresh, copy the file off and delete only
that one file. Nothing is written to the card during a run. Whether the brain appends the way this
expects is itself one of the beliefs the bench worksheet checks (the
[Hardware Assumptions Register](../hardware-assumptions.md), HA-177).

## Mechanisms in an auton

Both robots have the same mechanisms — the cascade, the right and left side toggles, and three pistons:
the claw, the front toggle (declared as `flipper`) and the ramp — declared in
`src/robots/mechanism_table.hpp` inside each robot's namespace (`robot_one`, `robot_two`). An autons
file says `using namespace robot_one;` (or `robot_two`), and the route builder's routes
`using namespace this_robot;` — the BUILDING robot's names — so an auton names them directly:
`r.set(claw["open"])`, `r.run(rightToggle, 12_volt, 0.5_s)`, `r.move(cascade.to(18_in), 2_s)`. (In a
route robot two runs MIRRORED, a side toggle is picked by side —
`competition::isMirrored(ctx) ? leftToggle : rightToggle` — because a mirror swaps the robot's sides;
[chapter 17](17-the-competition-program.md#what-it-cannot-carry).)
The cascade has no limit switch: it learns where its bottom is by stalling gently down into its stop,
so a route homes it first — `r.startHome(cascade, 2_s)` while the first leg drives — and **waits for the
home to finish** (`r.waitUntilDone(cascade, 2_s)`) before its first move; a move sent while it is still
homing replaces the home and stops `NOT_HOMED`. Which mechanism does what in an auton is the team's to
write ([Chapter 21](21-adding-your-robot-by-hand.md) has robot one's facts;
[Chapter 18](18-mechanisms.md) every step form).

### The claws in an auton

How a mechanism step runs and is judged, shown on the host suite's example robot with claws — the OLD
robot two's two claw pivots and pneumatic claw, retired in the code (2026-11-16) after the team lead's
ruling of 2026-11-13 that robot two is robot one's build, and kept as the suite's example of two motor mechanisms and a piston. Quoted from the host
test that runs it (the build fails if this listing and that test ever disagree):

```cpp
RoutineResult grabAndPivot(AutonContext& ctx) {
    Routine r{ctx, "grab-and-pivot"};
    r.set(claw["open"]);
    r.driveTo(20_in, 0_in);
    r.set(claw["closed"]);                     // no sensor: UNCONFIRMED, counted done
    r.run(clawPivot6, 12_volt, 0.4_s);         // pivot 6 forward for 0.4 s
    r.run(clawPivot7, -12_volt, 0.4_s);        // pivot 7 back for 0.4 s
    return r.result();
}
```

Two things in it are easy to get wrong. The routine is built on the **context**,
`Routine r{ctx, "grab-and-pivot"}` — the form the 40 in auton uses too. Built on the chassis
alone (`Routine r{ctx.chassis, ...}`) it has no mechanisms: its drive steps run, and its first
mechanism step throws, naming the fix. And every mechanism step is **judged**, one serial line each (tag
`MECH`), as it ends: on the host test plant, the run above logs `'claw': SUCCEEDED -- time in 0.5…`
for the open, `'claw': UNCONFIRMED -- time in 0.5…` for the grab, then `'claw pivot 6': SUCCEEDED`
and `'claw pivot 7': SUCCEEDED`, each `-- time in 0.4…`, and the result reads `FINISHED`. Nothing
senses a piston, so a piston step ends when its declared move time has passed — on the robots, its
stroke time from the facts file — and a grab is UNCONFIRMED, **counted done**: the auton goes on (the
team lead's ruling). At the end of every run, and on an abort, the run guard puts every mechanism in
its safe state.

**On a route, the same steps are markers.** Each mechanism's actions are published as route ids,
derived from its name: here `claw_open` and `claw_closed`; `claw_pivot_6_run` and `claw_pivot_6_stop`;
the same two for pivot 7 (on the robots: `cascade_home`, `claw_open`, `ramp_down` and the rest). A run
marker takes its volts and a time limit in milliseconds (`claw_pivot_6_run` with 12 and 400 is the
auton's pivot step above); one without its time limit is refused by name, nothing moves, and the route
stops there. Here is the auton above as a route — a start that opens the claw, then a drive 20 inches
out whose end grabs and pivots — quoted from the same host test, which runs it through the tester and
checks each marker's verdict (a route is normally drawn in a route editor, not typed; this is what one
holds):

```cpp
route.waypoints[0].markers[0] = marker("claw_open");
route.waypoints[0].markerCount = 1;
route.waypoints[1] = {.x = Length{20.0}, .y = Length{0.0}, .headingMode = shulib::config::HeadingMode::Free,
                      .motion = shulib::config::WaypointMotion::DriveTo,
                      .constraints = {.timeout = Time{5.0}}};
route.waypoints[1].markers[0] = marker("claw_closed");
route.waypoints[1].markers[1] = marker("claw_pivot_6_run", 12, 400, 2);
route.waypoints[1].markers[2] = marker("claw_pivot_7_run", -12, 400, 2);
route.waypoints[1].markerCount = 3;
```

Its four markers fire in that order and are judged as the auton's steps are — the open SUCCEEDED, the
grab UNCONFIRMED and counted done, each pivot SUCCEEDED — and the result reads `FINISHED`.
[Chapter 18](18-mechanisms.md#as-route-markers--every-mechanisms-ids-run) covers the ids in full.

### What the boot says about them

The tester's serial log names each mechanism as it boots: what the robot file declares it to be, where
each part is plugged in, and whether the brain found it there; then the route ids it published (the
competition program prints the same, plus a line saying what each driver button does). Robot two's,
printed by the same functions on the host over its real declarations — on its facts file's placeholders,
robot one's numbers, which a robot-two build refuses until they are typed:

```text
cascade: LIFT -- BLUE, motor 1 forward, motor 2 reversed; made safe: HOLD
cascade: motor 1 -> port 10 ok
cascade: motor 2 -> port 1 ok
cascade: NOT HOMED -- a hold moves it at most 4.0 V with no soft stops; a preset or a move ends NOT_HOMED until it is homed (homeAtDriverStart, homeOnPress, or an auton's r.home)
cascade: starting values kP 0.250 kI 0.300 kD 0.0040 kV 0.0033, cruise 60.0 in/s in 0.25 s, reached within 0.25 in for 0.10 s -- PROVISIONAL (HA-182); .tuning overrides
right toggle: POWER -- GREEN, motor 1 forward; let go: BRAKE
right toggle: motor 1 -> port 9 ok
'right toggle': no stall signature -- held against its stop it is cut only by the driver letting go (HA-154)
claw: PNEUMATIC -- starts 'closed', safe 'closed'; closed = A low, open = A high (levels UNVERIFIED until a bench station sees them); a disable leaves it where it is
claw: line 1 -> 3-wire A not checkable (the brain cannot see a 3-wire device)
ROUTES: 11 mechanism route ids published (cascade_home .. ramp_down); 21 of the registry's 32 left for the team's own
```

— the left toggle's lines between the right toggle's and the claw's, the flipper's and the ramp's after
the claw's, each like its twin's. A motor that is not plugged in says so on its second line, as a
warning naming what the brain found on that port, and the tester still runs: the run guard cannot make
an absent mechanism safe, and the drive does not need it.

## Writing the next auton

Add a function beside `fortyInchesForward`, and an entry for it in BOTH robots' lists — robot one's
`src/team/robot_one_autons.cpp` and robot two's `src/team/autons.cpp`: the host suite fails while the twins'
lists differ (a route made in the route builder goes in `src/team/skills_route.cpp`, once, for both robots). Keep the
schedule a real match schedule — the students will copy it — and remember a tank turns first
(`face()`) and then drives. Build, run the host tests, upload the tester, and try it on the bench,
wheels up first. When you change a line this chapter quotes, the build tells you which listing to
update.

What the host tests do with your new entry: one test runs **every** registered auton, whatever the
file holds, through the tester's own run sequence on the host test plant — each robot's mechanisms
included, through the same mechanism service the programs use — and fails if any of them is
refused, throws (a mechanism step that can never be met throws at the step), never starts, hits its hard
stop, or does not end with its end action run, the drive stopped under Brake and every mechanism in its
safe state. Another reads the file itself and fails if a routine is built on `ctx.chassis` instead
of `ctx`. A second test pins the first two autons by name and schedule, and
counts the entries: it fails on your new one with a message asking you to pin its name, kind and
schedule there too. Neither checks that your auton goes where you meant — that is the bench's job,
and the tape's.

## Before the bench: previewing a route

Robot time is the scarcest thing in the week before a competition. Before a route goes on the
robot, **see the whole of it on the laptop**: one command runs one of your autons, from your own
`src/team/autons.cpp` (robot one's `src/team/robot_one_autons.cpp` with `--robot one`), through the
same library code the robot runs, on the host test plant, and draws it. A route still being built in the
[route builder](22-making-your-first-route.md#6-preview-watch-the-library-drive-it) gets the same run from its
**PREVIEW** button, in the browser on any laptop, before it is code.

```bash
python3 tools/route_preview.py --list                              # every auton's name
python3 tools/route_preview.py "40in forward" --place=-48,-48,90   # a RELATIVE auton, placed on the field
python3 tools/route_preview.py "TEST 24in turn back"
python3 tools/route_preview.py "40in forward" --robot one --place=-48,-48,90   # robot one's, on its own mechanisms
```

The name is the auton's name exactly as the brain's menu shows it — spaces and capitals included,
in quotes. The command builds a small program on the laptop (a few seconds the first time), runs
your auton the way "shulib Auton Tester" runs it — the same `competition::runAuton` call, the same
deadline guard, the same park at your park time — prints a table of the steps in the terminal, and
says where it wrote the page: `build/route_preview/<name>.html` (robot one's in
`build/route_preview/one/`). Open that file in any browser; it needs no internet. It never writes
or changes your code: it only runs it and draws it.

**What the page shows.** The field to scale, red station at the bottom: the walls, the Toggles,
the Loaders, the Load Zones and the start pieces — the field parts the host test plant's field model
has; anything else on the field, the goals included, is not drawn or checked here (the route builder
checks the goals, [chapter 22](22-making-your-first-route.md#solid-structures-on-the-field)). On it, the path the host test plant's robot really drove, step by step,
each step numbered where it started (hover a number for each step's times and result, or a step's
end outline for where it ended), its outline (a square, front edge in blue) at the start and end of
every drive and turn, an × where each drive was aimed, and a diamond where each mechanism step ran.
A timeline under the field runs from 0 to the period (60 s for skills, 30 s for a match) with your
park and stop times marked. Then the table (the same as the terminal's, with every step's start and
end time), every line the library said about the run, and the caption: *IDEAL host test plant, not
the robot.*

**Reading the colours.**

- **Green** — the step did what it was asked.
- **Red** — it failed or was refused, and the library's own sentence is beside it, word for word
  (for example `routine '40in-forward' STOPPED at step 1 (moveTo): motion TIMEOUT — skipping the
  rest; drive safed`). Steps after a stop never ran: they are listed in grey under it, and never
  drawn on the field.
- **Amber** — cut short: by the deadline (your park time arrived first), or by a throw (the
  PROGRAM ERROR line says where it happened); or a wall reset that was REFUSED while the route
  went on. On the laptop every wall reset is refused `no sensor` — the preview's robot has no side
  sensors, as robot two's program has none today — so a wall reset is never green here.
- **Teal** — a grab that nothing could confirm (UNCONFIRMED, counted done).
- **Dashed** (a dashed diamond, `[PLACEHOLDER]` in the table) — a step whose time comes from a
  number nobody has measured yet: a lift or arm still on the library's starting numbers (HA-182) —
  the cascade, until "shulib Tune" sets its own — moves in the time its declaration predicts, not its
  real one; a piston whose stroke time is still OWED in its robot's facts file (all three of robot
  two's: robot one's chosen 0.5 s until the builder times robot two's) moves in that time; a
  flywheel spins up on an ASSUMED 0.2 s time
  constant (the host test plant has to give the wheel some weight, and a flywheel's declaration has
  no number for it). On robot one, every fact its facts file (`src/robots/robot_one_facts.hpp`)
  still OWES is a placeholder (the preview uses it wherever it needs that number): the page and the
  terminal list each one under PLACEHOLDER at the top, and a fact leaves the list once it is typed.
  A value typed as "chosen, not measured" (a starting value picked at the bench) counts as typed:
  the page does not mark it, though nobody has measured it yet.

A mechanism started in the background (`startRun`, `startMove`, `startSpinUp`, `startHome`) is drawn
as its own thick stretch along the path — its diamond where it started, a dot where its answer
arrived — red on *its* stretch if it failed, while the drive it ran during keeps the drive's own
colour. (The `waitUntilDone` that collects it is not drawn: from outside, the preview cannot see it.)

**THROUGH and TOUCH.** The host test plant has no walls: a drive aimed past a wall drives straight
through it. So the preview checks the robot's outline — a 15 in square centred on the point the
robot tracks — against the field on every 10 ms step:

- **THROUGH** (red) — a corner more than 1 in past a wall's face, or more than 1 in into a Loader.
  On the robot it hits the wall there. Everything after that point is drawn faded (the robot would
  not get there), and the command fails (exit 1).
- **TOUCH** (amber) — the outline reaches a wall or a Loader but not 1 in past it, or overlaps a
  Toggle by any amount (a Toggle sits on top of the wall, its brackets hanging from 10.6 in up, so
  the preview never calls it THROUGH). Squaring on a wall and driving up to a Loader are normal, so
  a TOUCH is a note, never a failure.
- Driving over a start piece is a grey note: the plant does not push pieces.

The square is a drawing assumption, said on every page: the real frame may be smaller, the tracked
point may not be its middle, and a claw or lift reaching past the square is not checked.

**Why the numbers on the page are not your code's.** Today the library says nothing when a step
*succeeds*; from outside, the preview can see every motion (drives, curves, strafes, turns, brakes and holds)
and every mechanism step, but not a `pause`, a wait (`waitFor`, `waitUntilDone`), a `then(...)` step
(a wall reset is one) or a `startAt`. So the page counts the motions and mechanism steps in order.
Two started at the same instant (`startRun` twice in a row) are numbered in the order of the
robot's mechanism list, which may not be your code's. The library's own lines —
`STOPPED at step 12`, `step 13 skipped` — carry your code's real step numbers, and the page quotes
them word for word.

**A RELATIVE auton** starts wherever you put the robot, so give that place with `--place=X,Y,DEG`
(inches, inches, degrees, in the field frame; `--place X,Y,DEG` works too). Without it the robot is
drawn starting at the field's centre facing 0°, and the page says so in red. A FIELD auton is drawn
at its declared start.

**Which robot.** The preview runs robot two (`--robot two`, the default) on the test suite's
**stand-in drive** — a 12 in tank with made-up gains, not robot two's drive (Calibrate discovers that
on the robot). Robot two's mechanisms — robot one's twin's cascade, side toggles and pistons — run on
fakes built from their declarations' own numbers, every one of them still a placeholder in robot
two's facts file, so its cascade and piston steps are dashed. Where a declaration does not say how quickly
its mechanism gets up to speed (an arm's or a lift's `kA` in its `.tuning`; a flywheel's declaration
has no such number), the fake assumes it (0.05 s for an arm or lift, 0.2 s for a flywheel). A
flywheel's spin-up time is mostly that number, so its steps are dashed and the page names it; an arm
or a lift moves mostly in the time its declared speeds predict, so its assumed number is not marked.
Robot one (`--robot one`) runs its own autons file on its own mechanisms — each on a fake built
from its own declaration, the cascade included — on the same stand-in drive. A step on a mechanism
the robot does not have stops the route there, as it would on that robot, and the page's top line
says why.

**What it does NOT tell you.** The robot's real times: wheel slip, a weak side, the battery, a lift
slower than its numbers, a claw that sticks — the plant has none of them, and its times are neither
a floor nor a ceiling (a sluggish motor can settle a drive *sooner* on it). Anything on the field
shulib does not draw. Pieces the robot pushes. Whether a wall reset will seat. Whether the start is
legal, or how many points the route scores. **The preview is a plan; the robot is the test.**

**If it never finishes.** A route that loops forever after a stop (`for (;;) { ... }`) is stopped by
the preview itself: the page is still written, saying after which step the auton keeps looping
without letting time pass — on the robot it would sit there until the field disables it, and its
park would never run — and the command fails. A loop that never logs anything is stopped at 300 s
of laptop time, and the command says so: no page is written for that run (the command removes the
auton's last page before it starts, so an old page is never mistaken for this one).

**If your code throws.** The page says PROGRAM ERROR with the exception's own words, draws the path
up to the throw, and says where it happened: *inside* a step (the drive, turn or mechanism step
that was running when it threw, cut short there — but not a mechanism the robot drove during: that
one was started in the background, and your code had moved past it), *during a wait*, or *after*
the last step that began.

**Exit codes:** 0 drawn and nothing wrong (TOUCH notes allowed); 1 drawn, and the auton was
refused, threw, went THROUGH a wall or a Loader, or looped — or the command's limits stopped it
(300 s, or the program was killed; nothing drawn then); 2 a name, robot or flag the command does not
know — an unknown name lists the real names, an unknown robot lists the robots, and nothing is
drawn; 3 the command itself failed — its program did not build, the page could not be written, or
the preview broke (its words are printed). It never guesses a name: `"40IN FORWARD"` is not
`"40in forward"`, and an auton the program itself refuses (a stop time past 60 s, a missing start)
is found by its name and drawn as REFUSED, with the program's own sentence.

```bash
python3 tools/route_preview.py self-test    # proves the command's own limits and these lines
```
