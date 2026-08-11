# 1 — What is this?

> **Covers:** what VEX U is, what happens in a match, what "the autonomous problem" actually is,
> and what shulib does about it.
> **Read this if:** you are new to the team, or you're deciding whether to join.
> **Assumes:** nothing. No robotics, no C++, no VEX.

## The competition

[VEX U](https://www.vexrobotics.com/v5/competition/vex-u) is the university division of VEX
robotics. Each team builds **two robots** — ours are a 24-inch one and a 15-inch one — and they
compete together on a 12-foot-square field, scoring points by picking up game objects and placing
them on goals. The game changes every year; this season's game is called Override.

Each robot is controlled by a [V5 "brain"](https://www.vexrobotics.com/v5-architecture) — a small
computer that runs the code we write, reads the sensors, and powers the motors.

Matches have two phases. In the **driver control** phase, a human drives the robot with a
controller. In the **autonomous** phase, nobody touches anything: the robot runs entirely on its
own code and sensors. There is also a separate event called **Autonomous Coding Skills**, where a
robot is alone on the field for a full **60 seconds**, fully autonomous, trying to score as much
as possible. That 60-second run is the main thing this library exists to win.

## The autonomous problem

Here is the problem in one sentence: *a robot is alone on a field, nobody is steering, and it has
to know where it is and get where it's going.*

That sounds simple, and it is genuinely hard. Two things make it hard:

**1. The robot doesn't know where it is.** There is no little map dot. The robot has sensors —
wheels that count their own rotation, a gyroscope that senses turning, a camera-like GPS sensor
that reads a patterned strip on the field wall — and every one of them is imperfect. Wheels slip.
Gyroscopes drift. The GPS sensor drops out or reports a position a few inches off. The robot's
belief about where it is, is a *guess* assembled from imperfect witnesses, and the guess gets
worse over time unless something corrects it. (Chapter 3 explains how the guessing works and why
it degrades.)

**2. Small errors compound.** Suppose the robot thinks it is at position (24, 36) on the field
but it is actually at (25, 35) — one inch off in each direction. Every move it makes from now on
starts from a wrong assumption. Drive "forward 48 inches" from the wrong spot and you arrive at a
wrong spot. Worse: if the robot's sense of *which way it is facing* is off by even a few degrees,
every long drive smears that angle into inches of position error. A 5° heading error over a
48-inch drive puts you about 4 inches sideways of your target — easily the difference between
scoring and ramming a goal.

So an autonomous routine is not "a list of moves." It is a list of moves *plus* a running,
self-correcting estimate of where the robot actually is, *plus* a plan for what to do when a move
doesn't go as intended. Most teams get the first part working. The second and third parts are
where matches are won, and they are what this library is for.

## What shulib is

shulib is the C++ library our autonomous code is built on. When you eventually write a routine,
your code will read like this — "drive to this spot, turn to face that way" — and the library
handles everything underneath:

- **Knowing where the robot is** ("localization"): it combines the wheel sensors and the
  gyroscope into one continuously-updated position estimate, and it is built to distrust sensors
  that misbehave rather than believe them blindly.
- **Getting where it's going** (motion control): you name a target position and direction, and
  the library computes motor commands, every hundredth of a second, until the robot is there —
  or until it can honestly say it isn't going to get there, and tells you so instead of hanging.
- **Telling you what happened** (diagnostics): every motion logs what it did, how close it got,
  how long it took, and why it stopped. When a run goes wrong, the terminal output is designed to
  tell you where and why. This sounds mundane; it is one of the best things about the library,
  and it has its own chapter ([Chapter 11](11-reading-the-diagnostics.md)).

The [README](../../README.md) at the top of the repository is the summary version of all this,
with current test counts and build instructions.

## The honest part

**This library has never run on a physical robot.** Everything it does has been verified on a
simulated robot — a software model of motors, sensors, and physics that was deliberately built to
misbehave the way real hardware misbehaves (sensors that lie during startup, wheels that slip,
batteries that sag). That is real verification and we trust it, but simulation is not a robot,
and we do not pretend otherwise. The pieces of software that would connect the library to real
motors and sensors are the next phase of work. [Chapter 14](14-what-it-cannot-do-yet.md) lists
everything the library cannot do yet, with links to the plans for each.

Why build it this way? Because the robot doesn't exist yet either — the team builds robots during
the season, and the software had to be ready before the hardware. Building against a hostile
simulation was the only way to make real progress without a robot, and it means that on the day
the robot exists, we will be debugging *hardware surprises*, not logic bugs.

## Where to go next

Read the chapters in order — each assumes the ones before it. Chapters 2 through 6 are concepts,
with no code: they give you the vocabulary everything else uses. Chapter 7 gets the code building
on your laptop. Chapter 8 walks you through writing your first routine, line by line.

If you hit a word you don't know, it should be defined where it first appears; the
[glossary](15-glossary.md) has every term in one place. If you find a term that *isn't* explained,
that's a bug in this guide — tell whoever maintains it (see the
[guide README](README.md)).

---

*Next: [Chapter 2 — The field and coordinates](02-the-field-and-coordinates.md)*
