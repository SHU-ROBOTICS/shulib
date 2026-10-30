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

**Nothing in this chapter has run on a robot yet.** The program is host-tested — its decisions run
in the host test suite on the host test plant, the suite's model of a robot (motors that respond
instantly, wheels that never skid) — and every robot build compiles and links it, but no brain has
started it. Its first run is the bench worksheet's practice-match station. Treat every behaviour
below as what the code is written and tested to do, not as something a robot has shown.

## What it is

The competition program is the one you upload for a match. Build and upload it like the other
robot-two programs:

```sh
rm -rf bin .d && make ROBOT=tank PROGRAM=competition
pros upload --slot 6 --name "shulib Competition"
```

The `rm -rf bin .d` matters for the reason [Chapter 16](16-testing-an-auton-on-the-bench.md) gives:
`make` does not notice that the `PROGRAM=` word changed, and would link the last program you built
under this name. **Slot 6.** Slot 5 is held for the team's own old code, as a fallback: never upload
a shulib program over it.

What happens in a match, in one paragraph: the robot boots from its calibration record; before the
match you choose an auton on the brain's screen (**NONE**, the default, runs nothing); when the field
starts the autonomous period, the chosen auton runs on the program's **own task** through
`competition::runAuton` — the same call the Auton Tester makes from its RUN — under a run guard
with a guaranteed end action; a **disable**, or the end of the autonomous period, ends it at once;
then driver control runs the team's driver file, `src/team/robot_two_driver.hpp`
([cookbook chapter 6](../cookbook/06-driver-control.md)).

## Registering an auton

The competition program has no autons of its own. It calls the same `registerAutons()` the tester
calls, from the team's file `src/team/autons.cpp` — [Chapter 16](16-testing-an-auton-on-the-bench.md)
walks through that file, and everything it says about an entry holds here unchanged. The three
lines that decide when and where an auton starts and stops, quoted from the file:

```cpp
    forty.start = competition::StartPose::relative(); // where we place it is (0, 0), facing 0
    forty.endActionAt = 27_s;  // still running at 27 s? it is stopped, and the end action starts
    forty.hardStopAt = 29_s;   // everything stops: 2 s after 27 s, 1 s before the field's 30 s
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
period (the team lead's ruling, 2026-10-02). Fix the entry in `src/team/autons.cpp` and upload again.
A registry built without asking for that (`competition::AutonRegistry registry{periods};`) still
throws the sentence at the `add()`. Leave a margin inside the period: a hard stop exactly on the
field's 30 s never gets a tick to act.

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
**mechanism service** — one object that ticks every claw's step once per tick, since the mechanism
core's fifth part ([Chapter 18](18-mechanisms.md)) — and, below it, the program's **watch**. Every tick it reads the competition status; the moment VEXos reports the robot
**disabled**, or the status leaves **autonomous** (the period ended, or the match moved straight to
driver control), it asks the run guard to **abandon** the run. The guard cuts the motion before the
world advances again — at most **one more commanded 10 ms tick** after the watch saw the change —
and in the same tick cancels every mechanism operation still running that registered its claim (as
the library's own do, since the mechanism core's second part), skips the end action (a disabled robot's motors are off; the brake cannot run), puts the drive at
0 V under Brake and every present claw and pivot in its safe state, and lets the routine unwind on
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
boot source RECORD, 2 autons, 3 of 3 mechanisms present; autons ALLOWED
SD run log ON, APPENDED to /usd/shulib_auton_runs.txt
```

then one line per mechanism the robot could not build (`mechanism '...' is ABSENT -- the run guard
cannot safe it: ...`), the autons the registry dropped (`N auton(s) REFUSED at registration and
DROPPED -- the dropped ones are NOT on the menu (each named below); every other auton and driver
control still run -- ...`, then `DROPPED: auton '...': ...` for each — past eight, the line says `(the
first 8 named below)` and a last line counts the rest), where the log copy is (`log
copy: /usd/comp_log.txt (APPENDED, kept across boots) -- ...`, or that it did not open and why: `NO
SD CARD`, or a card that is in and would not take the file), the mechanisms' lines — tag `MECH`, in
the words "shulib Drive" and the tester print too: for each mechanism what the robot file declares it
to be, where each part is plugged in and whether the brain found it, what the driver file's control
does with it, and anything it lacks — and one line for the route ids the autons' markers can fire:

```text
claw pivot 6: POWER -- RED, motor 1 forward; let go: HOLD
claw pivot 6: motor 1 -> port 6 ok
claw pivot 6: hold-to-run R1 / R2, 12 V
'claw pivot 6': no stall signature -- held against its stop it is cut only by the driver letting go (HA-154)
claw: toggle on A ('closed' / 'open')
ROUTES: 6 mechanism route ids published (claw_pivot_6_run .. claw_open); 26 of the registry's 32 left for the team's own
```

(pivot 7's four lines and the claw's first two between them; [Chapter 16](16-testing-an-auton-on-the-bench.md#the-claws-in-an-auton)
shows the claw's. The buttons are the driver file's, whichever program binds them.)

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
| `FINISHED` | the routine (or route) ran to its end and the end action succeeded |
| `ROUTINE STOPPED` | a step failed; the routine's own stop line says which |

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
the faults, and the claws. The protections row (the library's protections run under the driver file in
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
brain`, `CUT see brain`, the last fault, `no controller 0V`, `DEGRADED see brain`, `CLAW ABSENT`, or
`ok`, the first that holds. Both programs write those lines **one at a time**, at most one every
0.1 s, the word line first — PROS's own documentation says faster controller text updates are not
applied — so a full refresh takes up to 0.3 s. When driver control ends on a disable, the screen goes
back to the menu.

**Each auton run's lines.** Four `[CMP]` lines per run (two for NONE, TOO LATE, a refusal or a program
error; one more when the auton task first waited for the driver loop to leave):

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
  table, the graph refused to build, the driver's controls do not fit the robot, or the boot's
  **tick-chain proof** failed (one idle tick must reach the run guard, the mechanism service and the
  watch, and the watch must be attached to the guard; otherwise no disable could end an auton, or no
  claw step would ever end — the refusal names which link is missing) — the program paints the red refusal screen, powers nothing and starts **no auton task**.
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
  Brake and every present claw and pivot in its safe state (each step on its own), logs one
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
started from the controller's menu: this program, in slot 6, at that station. Afterwards, turn the
controller off, wait ten seconds, and see the brain's menu return before starting any other program.

## What it does not do yet

- **The drive protections have never run on a robot.** Driver control runs them (the driver file, on
  the library's loop), but every number they use is a guess: whether a full-stick push against a wall
  CUTS the drive (a V5 motor pushing is believed to sit near its 2.5 A limit, above the 2.4 A cut —
  [HA-154](../hardware-assumptions.md), not measured) is what the bench
  worksheet's practice-match station's **push check** measures, and it is **required before this
  program drives a match**.
- **Route autons fire only the mechanisms' ids.** An auton written as a route (the path editor's
  kind) runs, and its markers can fire robot two's six mechanism route ids, as in the tester
  ([chapter 16](16-testing-an-auton-on-the-bench.md#the-claws-in-an-auton) shows one, the claws
  auton as a route; the host suite runs it through this program's own request path too); a marker
  naming any other id is counted unknown and runs nothing — the team's own commands have no place to
  be registered yet. The team's file holds no route auton today.
- **The auton pick under a Timed Run** waits on the measurement above: if the Timed Run gives no
  disabled window before autonomous, how an auton is chosen on the bench is decided after it.
