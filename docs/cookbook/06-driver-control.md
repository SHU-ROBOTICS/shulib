# 6 — Driver control

The other five chapters are about autonomous routines. This one is about the two minutes after
them: the driver's own buttons. The drive itself needs no recipe — `teleop::runDriverControl` is
the whole loop ([Chapter 10](../guide/10-the-api.md) walks through it) — so this chapter is about
the thing a team adds to it: a mechanism the driver works while driving. It ends with a whole
driver file — one robot's buttons, feel and loop in one place.

The first two recipes' listings are compiled and run in
[`test/cookbook_examples_test.cpp`](../../test/cookbook_examples_test.cpp), cases `cookbook-06a`
and `cookbook-06b`, on the host test plant. Beyond the names every recipe assumes, they have the
teleop and hardware names in scope —
`MechanismBinding`, `ToggleBinding`, `MotorMechanism`, `PneumaticMechanism`, `ControllerButton`,
`BrakeMode`, `DriverLoopConfig`, `validBindings`, `runDriverControl` — plus the shaper, the
controller, the loop's three callbacks and a `showOnScreen` stand-in for the program's own screen
line, which each test declares just above. The last section quotes a real file from the project,
[`src/team/robot_one_driver.hpp`](../../src/team/robot_one_driver.hpp), which every robot build
compiles and [`test/robot_one_driver_test.cpp`](../../test/robot_one_driver_test.cpp) runs through
the host's stand-in for PROS.

> **Since the mechanism core's fourth part (2026-10-29) these two bindings are SUPERSEDED, not removed.** A
> robot whose mechanisms are declared in its robot file states its buttons as one checked list of driver
> controls — `whileHeld`, `toggle`, `preset` (presets on buttons), the homing controls — run through the same
> mechanism service as its autons ([chapter 18](../guide/18-mechanisms.md#on-the-drivers-buttons--the-driver-files-controls);
> [chapter 7](07-mechanisms.md) has the recipe). The two below stay, unchanged, for a program with no mechanism
> service. The robots' driver files — the last section — use the controls.

---

## Drive a mechanism from two buttons

**Use this when:** a claw, an arm or an intake should move one way while one button is held, the
other way while another is held, and stop the moment the driver lets go.

Declare the mechanism, then bind two buttons to it at a voltage you state — there is no default
voltage, because there is no voltage that is right for every mechanism:

```cpp
// Two claws that move separately are TWO mechanisms, one motor each.
shulib::hal::IMotor* clawMotors[1] = {&clawMotor};
MotorMechanism claw{clawMotors, BrakeMode::Hold, "claw"};
MechanismBinding clawButtons{claw, ControllerButton::R1, ControllerButton::R2, 12_volt};
MechanismBinding* bindings[1] = {&clawButtons};
```

Then check the set while you can still say what is wrong, and hand it to the loop:

```cpp
// L1 is the loop's default re-zero button: state one the claws do not use.
const DriverLoopConfig loop{.tickPeriod = 10_ms, .rezeroButton = ControllerButton::A};
char why[160] = "";
if (!validBindings(bindings, loop.rezeroButton, why, sizeof why)) {
    showOnScreen(why);  // say what is wrong while the program still can...
    return;             // ...and never start the loop, which would refuse it by throwing
}
runDriverControl(c.chassis, controller, c.pacer, shaper, loop, bindings,
                 disabled, heading, panel);
```

The refusal branch must stop the program before `runDriverControl`, not just report: the loop
checks the same set again before its first tick, and there a refusal is an exception, thrown from
inside driver control — the one place a program cannot afford one.

While R1 is held the claw gets +12 V every tick; while R2 is held, -12 V. The tick neither is
held — or both are — it goes to its declared safe state (here Hold: the brake mode first, then
0 V) exactly once, and then nothing at all is commanded until a button is pressed again. The
test drives this for ten ticks and checks the claw's volts tick by tick: 0, 0, 12, 12, 12, 12,
0, 0, 0, 0, with the motor left in Hold.

The loop also protects the drive, and there is nothing to write for it: a drive motor whose
readings freeze while its side turns is dropped after 250 ms, more than two dead drive motors in
total stop the drive for the rest of the run, and a motor fighting its side — or drawing more than
2.4 A while driven — for 250 ms sends both sides to 0 V for 1 s, then driving resumes
([Chapter 10](../guide/10-the-api.md) has the rules; the numbers are `DriverLoopConfig::protection`,
every one unmeasured). `panel` above is the loop's `perTick`, called once a tick after the drive
with `(const ChassisSpeeds& command, bool connected, const DriveShaperState& state)` — the driver's
request, whether the controller is there, and the feel layer's state. To show the protections on
a screen, give it a fourth parameter, `const DriveProtectionState&`: the loop hands the
protections' state (driving, degraded, cut or refused; the dead ports; the cut count) to a
`perTick` that takes it, and one with the three parameters works exactly as before.

**Why it is written this way.**

- **Letting go is the stop.** A mechanism the driver uses *while driving* cannot be interrupted by
  the sticks the way a driving macro can — the driver's thumbs are on the sticks the whole time.
  So the release is the stop, and it lands in the tick the release is read.
- **The safe state is applied once, not every tick.** Re-applying a hold every tick would fight
  anything else that later wants the mechanism, and on a V5 a repeated hold can re-capture its
  hold point wherever the load has sagged to.
- **The loop owns the edges of driver control.** It releases any claim an operation left on a
  bound mechanism, makes every binding safe before its first tick and again on a field disable
  (after stopping the drive), and it reads the buttons only while the controller is connected — a
  dropped controller is a release.
- **`MotorMechanism` takes the safe brake mode as an argument, with no default**, for the reason
  [Chapter 13](../guide/13-extending-the-library.md) gives: at 0 V a loaded claw in Hold stays up,
  and in Coast it drops — no single mode is safe for every mechanism.

**Watch out for:**

- **The re-zero button defaults to L1.** `DriverLoopConfig::rezeroButton` is the field-centric
  re-zero, and its default is L1 — a button teams very often want for a mechanism. Name another
  one. `validBindings()` refuses the clash by name, and so does the loop, before its first tick.
- **Two claws are two mechanisms.** A `MotorMechanism` sends one voltage to all of its motors, so
  two claws in one mechanism would move together on every button.
- **One button, one job.** A button used by two bindings, or one mechanism bound twice, is refused
  — two bindings on one mechanism would fight over it.
- **On a V5 brain, keep the mechanism, its motor array and the binding in static storage**, never
  on `opcontrol()`'s stack: a field disable can delete the driver-control task mid-tick and runs
  no destructors, so anything on that stack is simply gone — and anything else that still points
  at it (an end-of-run guard's mechanism list, a panel) is left pointing at nothing.
- **A robot built from a config gets its mechanisms from it.** With a `config::RobotConfig`, state
  each mechanism's ports, cartridge and safe mode in its `mechanisms` list and bind to
  `robot.mechanism("claw")` — the robot builds each one with its own cartridge, and one whose
  motor is unplugged is reported absent (`mechanismAbsence()`) instead of stopping the robot.
- **An outside stop wins until the driver lets go.** If your program stops the mechanism through
  its binding (`makeSafe()`) while a button is still held, the binding does nothing until both of
  its buttons have been seen up — the next tick of a finger that never left the button cannot
  undo the stop. A stop that goes around the binding does not stick: something that holds only
  the mechanism (an end-of-run guard's list) can make it safe, but a held button drives it again
  on the next tick.
- **A claim left over when driver control starts is released — and said.** While an operation
  has the mechanism claimed, its buttons do nothing. An operation your autonomous routine was
  running when the field ended autonomous never releases its own claim — PROS deletes the
  autonomous task wherever it is, and nothing after that runs — so `runDriverControl` does it for
  you: at its start, before it makes anything safe, it releases any claim still on a bound
  mechanism (it never calls the dead operation's `cancel()`), writes one `[WARN][TEL] driver
  control: RELEASED a claim left on mechanism '...' at entry` line for each, and the binding's
  `releasedLeftoverClaim()` stays true for your panel. The price: an operation your program starts
  on a bound mechanism *before* the loop loses its claim at the loop's start — start it from the
  loop's per-tick hook instead. A claim taken *during* driver control is still obeyed: the
  buttons do nothing until that operation lets go. For a toggle the release comes first, so the
  first time driver control starts, its initial level goes out even if an operation had the claw
  claimed: a claw your autonomous left open is closed (for a claw that starts closed) the moment
  driver control begins — the same as when nothing had claimed it.
- **Hold is only as good as the motor.** Whether a V5 motor at 0 V in Hold actually keeps a loaded
  claw where it stopped has not been measured; test it on your robot before you trust it.

---

## Open and close a pneumatic claw with one button

**Use this when:** a claw (or a clamp, or a latch) runs on air, and the driver wants ONE button —
press it and the claw opens, press it again and it closes.

Declare the claw's line and one named constant for which level opens it — nobody can know that
until the valve is plumbed, so it lives in exactly one place — then bind the button and state the
level the claw starts at:

```cpp
// ONE constant says which level opens the claw: if A closes it instead, flip it here.
constexpr bool kClawOpen = true;
shulib::hal::IDigitalOut* clawLines[1] = {&clawSolenoid};
PneumaticMechanism claw{clawLines, !kClawOpen, "claw"};  // its safe level: closed
ToggleBinding clawButton{claw, ControllerButton::A, !kClawOpen};  // it starts closed
ToggleBinding* toggles[1] = {&clawButton};
```

Hand the toggles to the loop BESIDE the hold bindings from the recipe above (here, one claw pivot
on R1/R2), and check both lists as one set:

```cpp
// A is the claw now, so the field-centric re-zero moves to another button.
const DriverLoopConfig loop{.tickPeriod = 10_ms, .rezeroButton = ControllerButton::Y};
char why[160] = "";
if (!validBindings(bindings, toggles, loop.rezeroButton, why, sizeof why)) {
    showOnScreen(why);
    return;
}
runDriverControl(c.chassis, controller, c.pacer, shaper, loop, bindings, toggles,
                 disabled, heading, panel);
```

The claw is set closed once, when driver control first starts. The press flips it open in the tick
it is read; holding A down does nothing more; the next press closes it. The test holds A for four
ticks and checks the claw tick by tick — closed, closed, then open for the other eight ticks, the
valve told exactly twice in all (closed at the start, open on the press) — and then the field
disable ends driver control and the claw is **still open**.

**Why it is written this way.**

- **A disable leaves the claw alone.** The loop stops the drive and makes every hold binding safe,
  but it commands the claw nothing: a claw that closed (or opened) on every field disable would
  drop what it holds. So does a dropped controller — nothing is commanded.
- **A press has to be seen.** A button that is already down when driver control starts, or when a
  dropped controller comes back, is treated as held, not pressed: the driver lets go and presses
  again. So a press made while the controller was gone is not replayed when it comes back —
  *provided* the controller's "connected" signal and its buttons come back together. Nobody has
  measured that yet: if the first reading after a cable blip said "not pressed" while the driver
  was still holding A, the next real reading would look like a new press, and the claw would flip.
  Until it is measured, a driver whose controller drops should let go of A before pressing again.
- **Each press flips what the valve was last told**, even if something else set it in between
  (an end-of-run stop, say), so a press always does the visible thing.
- **The starting level is stated, never defaulted**, for the same reason the safe level is: which
  level is "closed" depends on how the valve is plumbed.

**Watch out for:**

- **One button, one job — the re-zero button included.** `DriverLoopConfig::rezeroButton` is the
  field-centric re-zero; if it is the claw's button, `validBindings()` refuses the set by name, and
  so does the loop before its first tick. The test checks exactly that: with A as the re-zero, the
  same set is refused.
- **Which level opens the claw is a guess until you press the button.** If the claw closes on the
  first press, the constant is backwards — flip `kClawOpen`, and nothing else.
- **Two valves in one mechanism move together.** The toggle sends one level to all of a
  `PneumaticMechanism`'s lines — right for two cylinders that must fire together, wrong for the two
  coils of a double solenoid, which need opposite levels. (Since the mechanism core's first part a
  mechanism can DESCRIBE such positions — each position states every line's level, and
  [`config::MechanismConfig`](../api/robot_config.md#struct-mechanismconfig) carries per-line
  levels — and since its fourth part the core's driver controls drive such positions, `toggle` and
  `preset` ([chapter 7](07-mechanisms.md), [chapter 18](../guide/18-mechanisms.md#on-the-drivers-buttons--the-driver-files-controls));
  both robots' driver files use `toggle` for their claw, front toggle and ramp — one line each.) A single valve on ONE
  3-wire port is one line, whatever cylinder it drives.
- **From a config, the robot builds the claw.** State the mechanism's 3-wire port (A..H, stored as
  1..8), its `pneumaticStart` and its `pneumaticSafe` level, and bind to
  `robot.pneumaticMechanism("claw")`. Making the line sets it to the start level at once, so a claw
  that starts closed is told to close the moment the robot is built. Give the toggle the same
  starting level, or the claw moves when driver control first starts. (To report whether each
  mechanism was built, ask `robot.builtMechanismAt(i)` — `mechanismAt(i)` is the motor-only
  lookup and returns null for a claw that works.)
- **What the brain does with a 3-wire output while it is disabled has not been measured** — whether
  it keeps the level through a disable, and whether it acts on a level set while it is disabled.
  Driver control commands nothing on a disable (the one write on a disable is the competition
  program's: a disable DURING a guarded auton puts every claw in its safe state —
  [Chapter 17](../guide/17-the-competition-program.md)); if the brain itself drops the line, the claw moves
  anyway. And on a field the robot is usually built in `initialize()`, while the brain is disabled,
  so the start level is set then: if the brain ignores it, the claw sits wherever its valve rests
  until the robot is enabled — or until driver control starts and the toggle sets its level. Check
  both on your robot before a match depends on them.

---

## A whole driver file: robot one's (and its twin's)

**Use this when:** you want to see everything a driver does to one robot — every stick and button,
and the feel — in one file a student can read top to bottom and defend (the call into the loop is
`src/common/driver_file.hpp`'s, shared by every driver file).

A team's own robot puts its driver's decisions in ONE file:
[`src/team/robot_one_driver.hpp`](../../src/team/robot_one_driver.hpp) is robot one's — the builder's
button map of 2026-11-15. Robot two is robot one's twin, and
[`src/team/robot_two_driver.hpp`](../../src/team/robot_two_driver.hpp) started as the same file with robot
two's name; its buttons, feel and map are robot two's own, to change for robot two alone.

![The builder's controller map, on both robots: R1 / R2 hold the cascade up / down; A the claw, B the front toggle, X the ramp (each press flips it); L1 runs the right side toggle and L2 the left while held; the left stick drives and the right stick turns; Y is the re-zero (field-centric is off); the d-pad is free](../assets/cascade-controls.svg)

The file's header comment lists what every stick and button does in plain words; below it, the code is
the team's decisions and nothing else. **Which button does what** — one checked list, each control
naming its mechanism by the declaration in the robot file
([`src/robots/mechanism_table.hpp`](../../src/robots/mechanism_table.hpp)):

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

**The feel** — the library's default arcade feel with field-centric off, the re-zero button, and
what the drive does when the sticks are let go:

```cpp
inline constexpr DriverProfile kFeel{.mode = ArcadeBody};
inline constexpr DriverLoopConfig kLoop{
    .rezeroButton = Button::Y,     // field-centric's "away from me"
    .releaseBrakeMode = Coast,     // let go of the sticks: the drive coasts
};
```

**Its name** — the one line that makes the three constants a driver the programs can hold:

```cpp
using RobotOneDriver = DriverFile<robot_one::kControls, robot_one::kFeel, robot_one::kLoop>;
```

These listings are the file as the builder gave it on 2026-11-15, quoted from the route builder's compiled
copy of that map
([`test/goldens/route_maker/controls_one_builders_map.hpp`](../../test/goldens/route_maker/controls_one_builders_map.hpp)):
the team's own changes to the file — by hand or from the route builder's Controls tab — never fail the
documentation build. The file itself is always the robot's real map.

**Two programs run this file**: "shulib Drive" on its own, and the competition program's driver
control ("shulib Competition" — [guide chapter 17](../guide/17-the-competition-program.md)). Under
it, in both, the library protects the drive: a drive motor that stops answering is dropped and the
robot drives on; more than two dead stop the drive for the rest of the run; a fight, or more than
2.4 A while driven, for 250 ms stops both sides for 1 s — the loop's numbers, every one unmeasured.
**Nothing of it has run on a robot**: it is tested on the host, through a stand-in for PROS, against
each robot's real declarations.

**Why it is written this way.**

- **The file holds only the team's decisions.** How a driver file RUNS — binding the controls to
  the robot's mechanism service, the controller, the call into the library's driver loop, the one
  rule for trusting the heading — is written once, in
  [`src/common/driver_file.hpp`](../../src/common/driver_file.hpp)'s `DriverFile`, for every robot.
  A second robot's driver file copies none of it: its controls, its feel, its loop and one `using`
  line.
- **A control names its mechanism by its declaration**, so a misspelt or removed mechanism is a
  compile error at the control's line, and `controls(...)` checks the whole list against the
  robot's list at build — each refusal BY NAME: a button doing two jobs, a hold's volts outside
  (0, 12], a control on a mechanism this robot does not have, a mechanism with no control (mark one
  the driver never works `autonOnly(...)`). `DriverFile` adds the last three checks: no control on
  the loop's re-zero button, no tick but the brain's, and a feel in range — each dead zone at least 0
  and below 1, each curve from 0 to 1.
- **The cascade homes at the first driver start.** `homeAtDriverStart` moves it DOWN on its own until it
  stalls gently on its bottom stop (at most 3 s); R1 / R2 wait until it has. Before it is homed a held
  button gives it at most its unhomed volts (the robot file's), with no soft stops.
- **A side toggle runs ONE way** (`whileHeld(rightToggle, Button::L1, 12_volt)`, one button): held, it runs
  its declaration's direction; let go, it stops. Which way that is lives in the robot's facts file — a
  toggle that runs the wrong way is fixed there, never by its button.
- **The pistons start where they rest.** Each piston starts (and is made safe) in the position its
  solenoid holds UNPOWERED, so nothing moves at power-up; the first press moves it to the other. A press
  must be SEEN — up, then down, on two connected ticks — so a controller that drops out never flips one.
- **Each robot's map is its own.** The marked regions — `[driver file …]` (the controls), `[control words
  …]` (the header's map) and `[feel …]` — are the team's on each robot, and each robot's test checks its
  own: every button with a job has one row in the map, the re-zero button stays free, each button runs only
  its own mechanism. [`test/robot_twin_test.cpp`](../../test/robot_twin_test.cpp) holds only the code
  OUTSIDE those regions equal (but for the robot's name).

**Watch out for:**

- **Keep the driver in static storage.** The program holds ONE driver for the whole run, never on
  `opcontrol()`'s stack: a field disable deletes the driver-control task, and the loop holds the
  driver's inputs by address.
- **Bind it once, at boot, inside the boot's `try`, AFTER the mechanism service is bound** — a
  control naming a mechanism the robot does not have, or of another kind, throws there, where the
  program can still refuse the boot and say why.
- **Change a button in the driver files and nowhere else.** The buttons — the re-zero button
  included — are stated only there; the robot file says what each mechanism IS (its ports, its
  stop), never which button works it, and the programs' boot lines read the buttons from the driver
  file's controls.
