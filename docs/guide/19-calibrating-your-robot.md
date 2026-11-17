# 19 — Calibrating your robot

> **Covers:** the standard "shulib Calibrate", which ships with the library: what it measures and the
> three facts you state instead, your Calibrate file (21 lines, shown whole), one run from the IMU
> to SAVED, what each ending means, the two screens where it waits for you before saving
> (**FRONT FLIPPED** and **MECHANISM PORT TURNED**), why every motor that is not a drive motor must be
> listed, and the optional typed drive ports.
> **Read this if:** your team is setting up a tank drivetrain for shulib, or Calibrate just showed
> you a screen this chapter names.
> **Assumes:** [Chapter 4](04-drivetrains.md) (tank drivetrains) and
> [Chapter 7](07-getting-set-up.md) (building and uploading).

The standard Calibrate runs whole in the host test suite on the host test plant (the suite's model
of a robot), both through the library's own test devices and through its real PROS adapters over the
suite's stand-in for PROS, and 34 runs of it are compared line for line with recorded ones. The
measurement inside it — the calibrator — ran on robot two in an earlier form on 2026-09-24. Every
threshold in it is invented for a first run (register entry HA-136). Since 2026-11-05 this
repository's own "shulib Calibrate" (robot two, slot 4) IS the standard Calibrate: its facts are
derived from robot two's tables (`src/robots/calibrate_facts.hpp`) instead of typed in a file like
the one below — and deriving them changed none of the 34 recorded runs the library's tests hold.

## What it measures, and what you state

Calibrate finds out, on the robot, everything about the drivetrain that can be measured — and you
state only what cannot be:

| Fact | Who | How |
|---|---|---|
| the drive kind | **you** | `config::DrivetrainKind` — only a **tank** is measured; anything else is refused by name |
| the drive motors' cartridge (insert colour) | **you** | no reading can tell it: every insert turns the motor the same, and the firmware divides by what it was told |
| every motor that is **not** a drive motor (your mechanisms) | **you** | the brain knows a motor is plugged in, never what it is for |
| which ports drive, which way each is wired, which side | measured | the push and the pulses |
| the **front** | measured | the end that **leads the push** is the front |
| the distance scale of each side, the track width | measured | the push along a tape, then the spin |
| the IMU's port | measured | the brain's device list |
| *(optional)* the drive ports your drive program already types | **you, only if you have them** | never used to calibrate — see [Typed drive ports](#typed-drive-ports-optional) |

The result is the **calibration record**: a small file on the SD card that your drive program reads
at boot. Calibrate writes it, with a backup copy beside it, each read back to check it (below).

## Your file

A team's Calibrate is one file — a PROS program's whole source:

```cpp
// Our robot's Calibrate. We state only what the robot cannot measure about itself;
// shulib Calibrate finds everything else (which ports drive, their directions, the sides,
// the FRONT -- the end you push first -- the track width and the wheel scale).
#include "main.h"
#include "shulib/hal/pros/calibrate.hpp"

using namespace shulib;

constexpr calibration::CalibrateFacts kOurRobot{
    .drive = config::DrivetrainKind::Tank,
    .cartridge = hal::MotorCartridge::Blue,  // the insert colour of the DRIVE motors
    // EVERY motor that is not a drive motor. A motor left out is a drive candidate,
    // and one the push turns gets powered (3, 6, 9 V) like a drive motor.
    .mechanisms = {{6, "left claw"}, {7, "right claw"}},
};

void initialize() { hal::pros::calibrateInitialize(); }
void disabled() { hal::pros::calibrateDisabled(); }
void competition_initialize() { hal::pros::calibrateConnected(); }
void autonomous() { hal::pros::calibrateAutonomous(); }
void opcontrol() { hal::pros::runCalibrate(kOurRobot); }
```

That is the whole program. The three facts are the only things you state; the five lines at the end
are PROS's five entry points, each one call. Change the cartridge to your drive motors' insert
colour (`Red`, `Green` or `Blue`) and list your own mechanism motors, each with the name the log and
the screen will use. (These 21 lines are compiled and run by the library's host test suite, in
[`test/calibrate_team_file_example_test.cpp`](../../test/calibrate_team_file_example_test.cpp) —
so this listing cannot drift from code that builds.)

**What you get wrong, and what it says.** Every one of these is refused on the brain's screen
(**CANNOT START**, in red) and on the serial log *before anything is made or powered*:

- the kind or the cartridge left out → **DRIVE KIND UNSET** or **CARTRIDGE UNSET**, with the fix
  ("state it in the robot's facts, rebuild, upload") — Calibrate never guesses a cartridge;
- a kind other than `Tank` → **NOT A TANK** ("the facts say x-drive; this procedure measures a TANK
  drivetrain only"). **Do not run Calibrate on an X-drive or an H-drive**: a holonomic drive's
  calibration is not built yet;
- a mechanism named with port 0 (a placeholder), a port outside 1..21, or a port listed twice →
  **FACTS REFUSED**, naming the entry, e.g. `mechanisms[2] = {0, "intake"}`;
- [typed drive ports](#typed-drive-ports-optional) (optional) with a port outside 1..21, a port
  listed twice — on one side, or once on each side (`-11` and `11` are the same port) — or only one
  side typed → **FACTS REFUSED**, naming the entry and its value, e.g. `typedDrive.right[4] = 11`;
- so many motors declared as mechanisms that fewer than two drive motors are left →
  **TOO FEW MOTORS**, naming the ports you declared ("is any a DRIVE motor? else re-seat the drive
  cables").

A declared port with no motor on it is only a line in the log (`the census found NO MOTOR there`),
and the run goes on.

## List every motor that is not a drive motor

The brain reports *that* a motor is on a port, never *what* it drives. So Calibrate treats every
motor you did not declare as a possible drive motor — a **candidate**. One the push turns is then
**powered** like a drive motor: pulsed at 3, then 6, then 9 V, one at a time. An intake roller the
push drags along will spin; a claw that cannot move will strain. The record stays safe (such a
motor ends up `LEFT OUT`, or the run is refused), but a mechanism was powered next to a person.

So: **list every motor that is not a drive motor** in `.mechanisms` — a 3-wire solenoid is not a
motor and never needs listing. A declared motor is never powered, never configured, never braked;
Calibrate only *reads* it (the watch, below). With nothing declared, the log says so at the start
(`no mechanism port declared -- ... EVERY motor is a drive candidate`): either declare them or
**unplug every mechanism motor before you run Calibrate**. When a candidate turns out not to be a
drive motor, the log says so — `port 8 did not turn with the push: not a drive motor, never powered
-- if it is a mechanism, declare it in the robot's facts`; a `LEFT OUT` line asks "(a slipping gear?
or a MECHANISM motor -- declare it in the robot's facts)"; and a refused pulse is followed by `if
port N is a MECHANISM motor, declare it in the robot's facts: a motor that is not declared is a
drive candidate`.

## One run

Build it as a PROS project and upload it like any program. Then, on the brain, with the robot on
the floor at one end of a tape at least 48 inches long:

1. **1/4 IMU — HANDS OFF.** The IMU warms up for about 2 seconds (register entry HA-23). The screen
   says the rule you are about to use: **the end that leads the push is FRONT**.
2. **2/4 PUSH — PUSH FRONT FIRST, 48 IN ALONG TAPE.** Push the robot by hand, straight along the
   tape, with the end you want to be the **front** leading. The screen counts the motors that
   turned. When it has seen enough, a green **DONE** appears: touch it, then stand clear. (A curved
   push, or one too short, is refused here — push again.)
3. **3/4 PULSES — STAND CLEAR.** One motor at a time is pulsed at 3, 6 and 9 V, which tells the
   sides apart from how the robot turns.
4. **4/4 SPIN — STAND CLEAR.** The robot spins both ways to measure the track width.
5. **SAVED** — the result is on the screen and the serial log. Copy the `MEASURED tank ...` line
   down, then touch **RUN AGAIN** and push once more: two runs that agree are a measurement; two
   that do not are not.

**ABORT** stops a run at any step; every motor is left at 0 V and coasting. It is the red button at
the bottom right; once a motor is powered — from the first pulse on — it covers the whole bottom row,
so a touch anywhere there stops the robot. Until then a second tap where DONE was does nothing: the
run goes on to the pulses. A field disable, or
the start of autonomous, ends the program the same way (Calibrate runs only in driver control,
enabled).

**This program, in this form, has not yet run on a robot.** Its calibrator ran on robot two in an earlier
form on 2026-09-24 ([What it cannot do yet](14-what-it-cannot-do-yet.md#it-has-never-driven-a-robot-under-its-own-control)).

## What the ending means

Calibrate saves with the library's **safe save**, in three steps, each file read back and compared
byte for byte: it copies the record that was on the card to the backup
(`shulib_drivetrain.bak`), then writes the new record (`shulib_drivetrain.bin`), and only then
**refreshes the backup with the new record**. So after **SAVED** the backup holds the new record too
(the log says `The backup ... re-reads identical`) — not the earlier one: copying it back cannot undo
a record saved by mistake (a USE NEW after a wrong push: see FRONT FLIPPED below). Only a step that
fails leaves the earlier record in one of the files, and the log then says which file holds what
(after a SAVED whose last step failed: `The backup ... does NOT hold this record -- it holds ...`).
The save is designed so that one failure part-way never erases the good record; that rests on
beliefs about the card no robot has tested yet (register entry HA-166). The finished screen's header
says what the card measurably holds:

- **SAVED** — written and re-read. The next boot reads the new record.
- **NOT SAVED** — the new record did not stick; the log says which file and what the card now holds
  (the earlier record kept, or written back). If it says the record file is **damaged** and the
  earlier record is safe in the backup, the drive program in this build does not read the backup
  yet: rerun Calibrate, or on a laptop copy `shulib_drivetrain.bak` over `shulib_drivetrain.bin`.
- **SAVE UNCONFIRMED** (in the log; the screen says NOT SAVED) — the card stopped answering
  mid-save, so this run cannot say which record the next boot will use. Put the card back if it was
  pulled, copy both files off it on a laptop, and start the drive program to read its calibration
  line.

To start again from nothing, delete **both** files on a laptop. Under NOT SAVED the result is still
on the screen and the log: copy it down.

## The front, and FRONT FLIPPED

The record has no "front" field: every drive port's side and sign *are* the front, measured from
the end that led the push. Push from the other end and every port changes side and sign — and
every auton tuned on the old record would now drive backwards.

So before saving, Calibrate compares the new record with the one already on the card, port by port.
**SAME FRONT** saves. **CANNOT TELL** (no drive port in common, or no majority) saves with an amber
note, **FRONT NOT CHECKED -- see the log**: check the robot drives the right way before an auton.
And when the card's record calls the end that led this push its **back**, Calibrate does not save.
It holds, on an amber screen:

> **FRONT FLIPPED** · NOTHING SAVED YET · *the card calls the end you led BACK*
> RUN AGAIN: you led with the BACK by mistake
> USE NEW: you led with the FRONT (cables swapped?)
> *USE NEW after a wrong push reverses every auton*

The records cannot tell two things apart, so the screen never guesses:

- **You pushed with the back leading by mistake** → touch **RUN AGAIN**. Nothing is saved; push
  again, front first.
- **You pushed front first, and the LEFT and RIGHT drive cables were swapped between the sides**
  (on a tank whose two sides are mirror images, as most are, moving every drive cable to the other
  side's ports gives exactly the same signed record as a push from the other end), **or you changed
  the front on purpose** → touch **USE NEW**. The run is
  saved as pushed, the header says **SAVED -- NEW FRONT** in amber, and every saved front is still
  the end that led its push.

A field disable, or the brain turned off, while it waits saves nothing — the card keeps its record.
On a card with no record yet (a new card, or both files deleted), there is nothing to compare: the
run saves with the note **FIRST RECORD -- the end you pushed first is FRONT**. Push your first
Calibrate with the end your autons call the front. If the other end led that first push, the record
just saved is reversed: touch **RUN AGAIN** and push with the right end leading. Calibrate then shows
**FRONT FLIPPED** against the record just saved — touch **USE NEW** (you are changing the front back
on purpose), not RUN AGAIN, which would throw away the right push and keep the reversed record.

## MECHANISM PORT TURNED

During the push, Calibrate **reads** every declared mechanism motor — never commands it. A declared
motor that turned with the push like a drive motor (as far as the push's own 5° test for a drive
motor) holds the save:

> **MECHANISM PORT TURNED** · NOTHING SAVED YET · *port 12: 1692 deg, drive 1692 deg*
> the card's record DRIVES with port 12 *(when it does)*
> SAVE: it is a mechanism the push turns -- keep it
> *RUN AGAIN = save nothing: a drive motor? fix facts*

- **It is a mechanism the push turns** (an intake that touches the floor) → **SAVE**. It stays
  declared, the record has no such port, and the finished screen says **PORT 12 KEPT AS A
  MECHANISM**.
- **It is a drive motor you listed by mistake** (a typo in `.mechanisms`, or a mechanism's cable in
  a drive port) → **RUN AGAIN**, then stop: fix the facts or the wiring, rebuild, upload. Nothing
  is saved, and the card keeps what it had. Saving would have made a record one drive motor short.

The second line is the evidence: the motor's turn beside the drive's. Each declared port that turned
is its own question, one screen each, and nothing is saved until every one is answered SAVE. After
a USE NEW on FRONT FLIPPED, this screen comes next if a mechanism also turned.

## Typed drive ports (optional)

If your **drive program already types its drive ports** — and drives from them when the card holds
no calibration record — you may state them too, in the same spelling, after `.mechanisms`:

```cpp
    .typedDrive = {.left = {-11, 12, -13, 14, -15}, .right = {20, -19, 18, -17, 16}},
```

(signed ports per side, as a chassis table types them: `-11` is port 11 reversed). Or, if your
drive program is a `config::RobotConfig`:

```cpp
    .typedDrive = calibration::typedDrivePortsFrom(kRobot.drivetrain.tank.description),
```

(the `kRobot` must be a `constexpr` config, because the facts are). Both spellings are compiled and
checked by the same example test. Leave them out and nothing changes.

**Calibrate never calibrates from them.** They matter only when the card holds **no usable record**
(no file, or files that do not decode — a drive program treats both alike and drives from its typed
ports). Then the front is checked against **your typed drive ports** instead of nothing, in their
own words:

- **every port agrees** (same side, same sign, none in one list only) → saved, with the note
  **FRONT MATCHES YOUR TYPED DRIVE PORTS**;
- **the front agrees, but some port does not** → saved, with the note **FRONT MATCHES, SOME TYPED
  PORTS DIFFER -- see log**; the log names each port: a wrong sign, the other side, a typed port
  not in this push (LEFT OUT, declared, or did not turn), a pushed port you did not type. A typed
  port with a wrong sign fights its side-mates in anything that still drives from the typed ports:
  fix each one the log names;
- **they call the end that led this push its back** → held on **FRONT FLIPPED**, in the typed
  ports' words (*typed ports: the end you led is BACK*): **RUN AGAIN** if the back led by mistake;
  **USE NEW** if the front led — the typed ports are then wrong, or the cables moved since they were
  typed — and then **fix your typed ports**: the note says **FIX YOUR TYPED DRIVE PORTS**, because
  anything that still drives from them drives the robot backwards, and **USE NEW reverses every
  auton tuned on the typed ports**;
- **CANNOT TELL** (a tie, or no port in common) → saved with **FRONT NOT CHECKED**.

With a record on the card, the typed ports change nothing at all — the card's record decides. With
no SD card, nothing is checked (nothing can be saved). A robot with neither a record nor typed
ports gets only the **FIRST RECORD** note: push its first Calibrate with the end its autons call the
front.

## The serial log

Everything the screen shows, and more, goes to the serial log, one line at a time, tagged `[R3D]`:
the device census, the push table, every pulse and spin, the `MEASURED` line, the front check with
every count, the watch's verdict on each declared port, and the save's own words. When a screen
says "see the log", this is the log it means. [Chapter 12](12-when-things-go-wrong.md) lists each
held screen and note, and what to do about it.

## The exact signatures

The facts, the program and its words: [`calibrate_program`](../api/calibrate_program.md). The five
entry points: [`hal/pros/calibrate`](../api/pros-calibrate.md). The screens:
[`calibrate_panel`](../api/calibrate_panel.md). The safe save:
[`record_store`](../api/record_store.md). The belief that the watch's reads change nothing: register
entry [HA-187](../hardware-assumptions.md).

