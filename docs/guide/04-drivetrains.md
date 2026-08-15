# 4 — Drivetrains: tank, X-drive, H-drive

> **Covers:** what "holonomic" means, why our two robots move differently, what strafing is, and
> what kinematics does.
> **Read this if:** you want to know why the same command behaves differently on our two robots,
> or what "strafe authority" means in the logs.
> **Assumes:** [Chapter 2](02-the-field-and-coordinates.md) — frames and headings.

## Three ways to build a robot that moves

A **drivetrain** is the set of wheels and motors that moves the robot. The design choice that
matters most for autonomous programming is: *which directions can it move in?*

**Tank drive** is the familiar one — left wheels, right wheels, like a car or an excavator. It
can drive forward and backward, and it can turn. What it cannot do is move *sideways*. If a tank
robot needs to be 12 inches to its left, it has to turn, drive, and turn back — like parallel
parking. Its position and its heading are coupled: you cannot change one without disturbing the
other.

**Holonomic** drivetrains break that coupling. The word just means: the robot can move in any
direction, *regardless of which way it's facing*, and can rotate while doing it. Moving sideways
without turning is called **strafing** (a word borrowed from video games). Think of an office
chair on casters versus a car: the chair can slide any direction and spin at the same time; the
car has to steer through arcs.

Our 24-inch robot is an **X-drive**: four omniwheels (wheels with rollers around the rim, so they
can slide along their axle direction) mounted at 45° angles, forming an X. Push all four one way
and it drives forward; push them in the right combination and it slides sideways, or rotates, or
does all of it at once. An X-drive strafes exactly as well as it drives forward — full holonomic.

Our 15-inch robot is an **H-drive**: a normal tank layout plus one extra sideways-mounted wheel
in the middle (the layout draws an H). The tank wheels handle forward/back and turning; the
middle wheel pushes sideways. It *can* strafe — but with one wheel's worth of grip and power
instead of four, so it strafes distinctly slower than it drives. It sits genuinely in between:
more capable than tank, weaker sideways than an X-drive.

Why does holonomic matter enough to design a whole library around it? Time and simplicity. A
holonomic robot approaching a goal can slide sideways to line up *while already facing the goal*
— one smooth motion. A tank robot does a three-move shuffle. Over a 60-second run with many
scoring cycles, those saved seconds are points. And for the code, holonomic motion is *simpler*
to reason about: "go to (x, y, heading)" is one command with three independent knobs, not a
sequence of arcs. Most VEX libraries are built for tank drives and treat sideways motion as an
afterthought; shulib is built the other way around.

## Kinematics: from "which way" to "how fast each wheel"

Your code says things like "move toward the far goal at 20 in/s while rotating slowly
counterclockwise." The motors understand none of that — each motor just spins one wheel at some
speed. Something has to translate. That translator is called **kinematics**: the geometry that
converts a desired *chassis velocity* (forward speed, sideways speed, rotation speed) into
individual wheel speeds, and back.

For an X-drive, "slide left" translates to "front-left and back-right wheels one way, front-right
and back-left the other way" — four numbers, every hundredth of a second. You will never do this
math; the point of this section is that you know it *has* a home. Each drivetrain is described in
one place (`include/shulib/kinematics/` — `x_drive.hpp`, `tank.hpp`, `h_drive.hpp`), and
everything above it is drivetrain-agnostic. The same autonomous routine, unchanged, runs on all
three drivetrains — this is tested, not aspirational. When the team builds a different drivetrain
someday, the change is a new kinematics description, not a rewrite of every routine
([Chapter 13](13-extending-the-library.md) shows how).

Two practical concepts from this layer will show up in your debugging life:

**Desaturation.** Each wheel has a top speed. Ask for a fast diagonal strafe *plus* a fast spin,
and the math may demand more from one wheel than it can give. The library then scales the whole
command down uniformly until every wheel is achievable — so the robot moves in the *right
direction, slower*, rather than in a wrong direction at full speed (which is what happens if you
just cap each wheel independently — capping one wheel of an X-drive bends the direction of
travel).

**Strafe authority.** Each drivetrain reports one number: how fast it can sustainably move
sideways, as a fraction of its forward speed. X-drive: 1.0 (sideways = forward). Tank: 0.0
(cannot strafe at all). H-drive: somewhere in between — currently a placeholder of 0.35, meaning
"sideways at roughly a third of forward speed," because the real number depends on wheel grip
that can only be measured on a robot that doesn't exist yet (it's a registered assumption —
HA-54 in the [Hardware Assumptions Register](../hardware-assumptions.md)).

## Honesty about limits — what limited strafe does to your commands

What should happen when a routine written for an X-drive asks an H-drive for a fast sideways
move? shulib's answer is a principle worth knowing because you'll see it in the logs:
**degrade predictably, never silently.**

- On the **H-drive**, a sideways-dominant move runs *authority-limited*: the sideways part
  proceeds at the speed the drivetrain can actually sustain, while forward motion and rotation
  keep their full speed. The move completes correctly — it just takes longer. Measured in
  simulation, the H-drive matches the X-drive's *accuracy* on identical routines and pays only a
  few percent in *time*. When this limited mode is active, the diagnostic stream marks every
  affected line with the flag `SFB` ("strafe fallback") — the robot never quietly does something
  different from what you asked without telling you.
- On **tank** (authority 0.0), a sideways target is physically unreachable, and the library does
  not pretend otherwise: the motion runs until its time limit and honestly reports `TimedOut`,
  rather than faking success or crashing. Routines for a tank robot simply don't ask for pure
  sideways moves — they turn first, like a driver would (the API chapter shows the idiom).

If you're budgeting time for a routine on the H-drive, the `strafeAuthority()` query on the
chassis tells you the number the library is using, so you can decide whether a leg should strafe
or turn-and-drive.


## The exact signatures

The kinematics surface: the [`IKinematics` contract](../api/kinematics.md), the presets [`xDrive`](../api/x_drive.md), [`hDrive`](../api/h_drive.md) and [`TankKinematics`](../api/tank.md), the general [`MatrixKinematics`](../api/matrix_kinematics.md) behind them, [`WheelSpeeds`](../api/wheel_speeds.md), and [`desaturateUniform`](../api/desaturate.md).

---

*Next: [Chapter 5 — Getting there: control](05-getting-there.md)*
