# 16 — Testing an auton on the bench

> **Covers:** "shulib Auton Tester" — the program that runs one of the team's autons on the bench,
> from the brain's screen, with no competition switch; the team's auton file,
> `src/team/autons.cpp`, which it and the competition program both read; what to expect from the
> first run on a tank, and how to read the result against a tape measure.
> **Read this if:** you are about to run an auton on robot two, or you are writing the next one.
> **Assumes:** [Chapter 9](09-the-recipe-api.md) (a routine as a chain of steps) and
> [Chapter 4](04-drivetrains.md)'s section on what a tank can and cannot do.

**Nothing in this chapter has run on a robot yet.** Every number below is from the host test
plant — the suite's model of a robot, with motors that respond instantly and wheels that never
skid — and it is marked where it appears. The robot's own first results will be written into the
team's bench worksheet; until then, treat each number here as a prediction to check, not a promise.

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
service**, so a claw step in an auton runs and is judged exactly as it will be in the match, and a
**command registry** holding the robot's mechanism **route ids** — so an auton written as a *route*
(the path editor's kind, with markers) can fire robot two's claws from its markers. A marker naming
any other id is counted as unknown and runs nothing: the team's own commands have no place to be
registered yet.

**The competition program ("shulib Competition") calls the same `registerAutons()` and the same
`competition::runAuton`** — [Chapter 17](17-the-competition-program.md) describes it. Like the
tester, it has not run on a robot yet.

Build and upload it like the other robot-two programs:

```sh
rm -rf bin .d && make ROBOT=tank PROGRAM=autontest
pros upload --slot 2 --name "shulib Auton Tester"
```

The `rm -rf bin .d` is not optional. `make` rebuilds only the files that changed, and it does not
notice that the `PROGRAM=` word changed: after building another program it recompiles nothing and
links **that** program again — and the upload would put it in slot 2 under the tester's name. The
same holds whenever you switch between any two of the `make` lines in the README.

(Slot 2, since the new "shulib Drive" retired "shulib Teleop" from it; until then the tester was
in slot 7, so a brain uploaded before that may still hold it there — slot 7 is "shulib Tune"'s now,
and uploading Tune replaces it.) Start it from the **brain's**
screen, with the controller paired and on its home screen — started from the controller's menu it
would be a pretend match, and the tester refuses to run an auton in any competition mode.

## The team's file: `src/team/autons.cpp`

Every auton the robot can run lives in one file the team owns, `src/team/autons.cpp`, and one
function at its bottom, `registerAutons()`, lists them in the order the brain's menu shows them.
Both programs — the tester and the competition program — call that function;
neither has an auton of its own. The host test suite compiles the same file and runs every auton
in it on the host test plant through the tester's own run sequence, so a change to an auton is
tested before it is uploaded.

The first auton drives 40 inches straight ahead. Here it is, quoted from the file (the build
fails if this listing and the file ever disagree):

```cpp
// ── THE 40 IN AUTON ─────────────────────────────────────────────────────────────────────
// Drive straight forward 40 inches and stop, facing the way the robot started.
// ONE moveTo: the library drives to the POSE -- 40 in ahead, 0 in to the side, facing 0 degrees --
// and the step ends when the robot is there and still, or when the motion's own time limit ends
// it. Nothing is tuned here: the motion uses the robot's configured defaults.
// A TANK CANNOT DRIVE SIDEWAYS. If one side of the drive is weaker than the other, it turns the
// robot toward that side and the robot drifts off its straight line. While it drives, the library
// steers it back: it measures the turn it did not ask for and cancels it, and it aims a little
// back toward the line, never more than 15 degrees off the heading written here. (It steers only
// a step that starts within 15 degrees of that heading, as this one does; otherwise turn first
// with face().) On the serial log, TSTR marks each tick's line where it is steering noticeably.
// If the robot still cannot finish within half an inch of the spot and about a degree of the
// heading, the step ends on its time limit: the result reads ROUTINE STOPPED with the fault
// MOTION_TIMEOUT, and the brake at the end still runs. That is the library telling the truth
// about this robot, not a broken auton: read END x and y (how far short, how far beside) and the
// TSTR lines before changing anything.
chassis::RoutineResult fortyInchesForward(competition::AutonContext& ctx) {
    chassis::Routine r{ctx, "40in-forward"};
    r.moveTo(math::Pose2d{40_in, 0_in, 0_deg});
    return r.result();
}
```

It is one [Routine](09-the-recipe-api.md) with one step. The robot's coordinates are the ones
from [Chapter 2](02-the-field-and-coordinates.md): x straight ahead, y to the left, angles
counterclockwise. And the list at the bottom of the file:

```cpp
void registerAutons(competition::AutonRegistry& registry) {
    // 1. THE 40 IN AUTON.
    competition::AutonEntry forty{};
    forty.name = "40in forward";                      // what the brain's menu shows
    forty.kind = competition::AutonKind::Match;       // runs in the 30 s autonomous period
    forty.routine = &fortyInchesForward;              // the function above
    forty.start = competition::StartPose::relative(); // where we place it is (0, 0), facing 0
    forty.endActionAt = 27_s;  // still running at 27 s? it is stopped, and the end action starts
    forty.hardStopAt = 29_s;   // everything stops: 2 s after 27 s, 1 s before the field's 30 s
    // endAction is left out: the end action is chassis.brake() -- 0 V under Brake until still.
    registry.add(forty);

    // 2. THE LIBRARY'S ACCEPTANCE RUN (drive 24 in, turn to 90 degrees, face the start, drive
    //    back): the test that the whole library works on this robot, on the same schedule.
    registry.add(competition::acceptanceRunEntry(competition::AutonKind::Match, 27_s, 29_s));
}
```

Each entry is **data**: a name (at most 29 characters, shown as `AUTON: <name>`), a kind (a 30 s
match period or a 60 s skills run — the periods are the game manual's, declared once in
`src/team/autons.hpp`), the routine, where it starts, and its schedule. The schedule is the part a
new author gets wrong most often, so read it slowly: at `endActionAt` the routine is stopped
wherever it is and the **end action** runs (here the brake); at `hardStopAt` everything stops,
end action included. A hard stop that landed exactly on the field's 30 s would never get a tick
to act, which is why 29 s. An entry whose hard stop does not fit its period is **refused by name
when it is registered** — and **dropped, that entry alone**: it is not on the menu (the menu names it
in red, `REFUSED auton '<name>': ...`), the boot logs its whole sentence (`DROPPED: auton '<name>':
...`), and every other auton still runs. That is the team lead's ruling for the competition program
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
every claw and pivot in its safe state. The abort is read when a finger **lands** on the screen,
not when it lifts, and any finger that lands after your RUN touch lifted counts, however soon.
Touch the **top half** of the screen to abort: no button on the countdown, running or RESULT page
is there, so the finger that aborted cannot also press something on the RESULT page. (If you keep
holding, RESULT keeps its buttons grey until you lift, then arms them half a second later.)

## Reading the result against the tape

The 40 in auton drives 40 inches **by the robot's own estimate** — its drive encoders and its IMU.
The tape measure is how you find out whether the estimate is right. With robot two's calibration
record as it stood on 2026-09-25 (it read 44 in for a taped 48 in — about 8 % short), a
40 in auton should measure about **40 × 48 / 44 ≈ 43.6 in** on the tape. That is the calibration
record being wrong, not the auton: the fix is a careful Calibrate rerun, **never** changing the
40 in the file to make the tape agree. Every auton the team writes afterwards would inherit the
fudge.

**What to expect on a tank.** A tank cannot drive sideways, and robot two's two sides are not
equally strong (on 2026-09-25 it veered right over 48 in with the motors driven open-loop). Since
API 2.3, `moveTo` on a tank steers back onto its line while it drives — the file's own comment
above says how, and [Chapter 4](04-drivetrains.md#honesty-about-limits--what-limited-strafe-does-to-your-commands)
explains it in full. So the result to expect is **`FINISHED`**, with `END` x near 40 and `END` y
within half an inch, and `TSTR` on many of the tick lines in the serial log (the library was
steering). On the host test plant the 40 in finishes with one side up to about a quarter weaker —
robot two's is nearer a tenth — but that is the host test plant's number, not a promise for the
robot, whose motors respond with a lag the plant does not have.

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

## The claws in an auton

Robot two's mechanisms — the two claw pivots and the pneumatic claw — are declared in its robot
file, `src/robots/mechanism_table.hpp`, each by name inside the namespace `robot_two`. The team's
auton file says `using namespace robot_two;` at its top, so an auton names them directly:
`clawPivot6`, `clawPivot7` and `claw`. No auton in the team's file steps a claw yet — which claw
does what in an auton is the team's to write — so here is one, quoted from a host test that runs it
on robot two's real declarations (the build fails if this listing and that test ever disagree):

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
alone (`Routine r{ctx.chassis, ...}`) it has no mechanisms: its drive steps run, and its first claw
step throws, naming the fix. And every step on a claw is **judged**, one serial line each (tag
`MECH`), as it ends: on the host test plant, the run above logs `'claw': SUCCEEDED -- time in 0.5…`
for the open, `'claw': UNCONFIRMED -- time in 0.5…` for the grab, then `'claw pivot 6': SUCCEEDED`
and `'claw pivot 7': SUCCEEDED`, each `-- time in 0.4…`, and the result reads `FINISHED`. Nothing
senses the claw, so a claw step ends when its declared move time has passed — 0.5 s, a guess
until the bench worksheet times it ([HA-185](../hardware-assumptions.md)) — and the grab is
UNCONFIRMED, **counted done**: the auton goes on (the team lead's ruling). Which level opens the
claw is a guess too ([HA-159](../hardware-assumptions.md)). At the end of every run, and on an
abort, the run guard puts every claw in its safe state: the pivots at 0 V in Hold, the claw closed.

**On a route, the same claws are markers.** Each mechanism's actions are published as route ids,
derived from its name: `claw_open` and `claw_closed`; `claw_pivot_6_run` and `claw_pivot_6_stop`;
the same two for pivot 7. A run marker takes its volts and a time limit in milliseconds
(`claw_pivot_6_run` with 12 and 400 is the auton's pivot step above); one without its time limit is
refused by name, nothing moves, and the route stops there. Here is the auton above as a route — a
start that opens the claw, then a drive 20 inches out whose end grabs and pivots — quoted from the
same host test, which runs it through the tester and checks each marker's verdict (a route is
normally drawn in a route editor, not typed; this is what one holds):

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

**What the boot says about them.** The tester's serial log names each mechanism as it boots: what
the robot file declares it to be, where each part is plugged in, and whether the brain found it
there; then the route ids it published (the competition program prints the same, plus a line saying
what each driver button does) —

```text
claw pivot 6: POWER -- RED, motor 1 forward; let go: HOLD
claw pivot 6: motor 1 -> port 6 ok
'claw pivot 6': no stall signature -- held against its stop it is cut only by the driver letting go (HA-154)
claw: PNEUMATIC -- starts 'closed', safe 'closed'; closed = A low, open = A high (levels UNVERIFIED until a bench station sees them); a disable leaves it where it is
claw: line 1 -> 3-wire A not checkable (the brain cannot see a 3-wire device)
ROUTES: 6 mechanism route ids published (claw_pivot_6_run .. claw_open); 26 of the registry's 32 left for the team's own
```

— pivot 7's lines between them, like pivot 6's. A pivot that is not plugged in says so on its
second line, as a warning naming what the brain found on that port, and the tester still runs:
the run guard cannot make an absent claw safe, and the drive does not need it.

## Writing the next auton

Add a function beside `fortyInchesForward`, and an entry for it in `registerAutons()`. Keep the
schedule a real match schedule — the students will copy it — and remember a tank turns first
(`face()`) and then drives. Build, run the host tests, upload the tester, and try it on the bench,
wheels up first. When you change a line this chapter quotes, the build tells you which listing to
update.

What the host tests do with your new entry: one test runs **every** registered auton, whatever the
file holds, through the tester's own run sequence on the host test plant — robot two's claws
included, through the same mechanism service the programs use — and fails if any of them is
refused, throws (a claw step that can never be met throws at the step), never starts, hits its hard
stop, or does not end with its end action run, the drive stopped under Brake and every claw in its
safe state. Another reads the file itself and fails if a routine is built on `ctx.chassis` instead
of `ctx`. A second test pins the first two autons by name and schedule, and
counts the entries: it fails on your new one with a message asking you to pin its name, kind and
schedule there too. Neither checks that your auton goes where you meant — that is the bench's job,
and the tape's.
