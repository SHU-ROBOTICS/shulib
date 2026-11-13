# shulib documentation

shulib is the autonomous and driver-control library for Seton Hall University's VEX U robots. It
knows where the robot is on the field, drives it there, runs its mechanisms, and tells you plainly
what happened — and why, when something goes wrong.

---

## What do you want to do?

### Drive the robot

- **Set up the programs and upload them** — [Getting set up](guide/07-getting-set-up.md) lists every
  program, its slot on the brain, and the one command that builds it.
- **Put your mechanisms on the controller's buttons** — [Driver control](cookbook/06-driver-control.md):
  a mechanism on two buttons, a pneumatic claw on one, and a whole driver file to copy from.
- **On competition day** — [The competition program](guide/17-the-competition-program.md): choosing
  the auton on the brain's screen, what the screen tells you, and what happens when a match starts.

### Write an autonomous route

- **Your first route** — [Your first routine](guide/08-your-first-routine.md), then
  [the recipe API](guide/09-the-recipe-api.md) for every step a route can take.
- **When a step fails** — [carry on, skip ahead, or stop](guide/09-the-recipe-api.md#choosing-what-a-failure-does-onfail-and-label),
  and the [recipe for it](cookbook/02-when-a-step-fails.md).
- **Smooth curves instead of stop-and-turn** — [Flowing through points](guide/05-getting-there.md#flowing-through-points-curves),
  and [cookbook 1](cookbook/01-getting-there.md) for the recipe and its fallback.
- **Start a mechanism partway through a drive** ("at 60 % of this drive, raise the cascade") —
  [Triggers](guide/09-the-recipe-api.md#partway-through-a-drive-triggers).
- **Re-zero the robot's position against a wall** — [Re-seating one axis against a wall](guide/03-knowing-where-you-are.md#re-seating-one-axis-against-a-wall).
- **Check and flip a Toggle in a skills run** — [the Toggle step](guide/20-the-camera.md#checking-and-flipping-a-toggle-in-a-skills-route).
- **Try a route on the bench before the field** — [Testing an auton on the bench](guide/16-testing-an-auton-on-the-bench.md).

### Add, tune or test a mechanism

- **Declare a lift, arm, claw, roller or piston, put it on buttons and use it in a route** —
  [Mechanisms](guide/18-mechanisms.md), with the [recipe](cookbook/07-mechanisms.md).
- **Protect a string lift with a current cap** — [Current caps](guide/18-mechanisms.md#current-caps--how-hard-a-string-is-pulled).
- **Tune it on the robot, no laptop code** — [the Tune program](guide/18-mechanisms.md#the-tune-program--tuning-on-the-brain-screen):
  change the numbers on the brain's screen, watch the result, copy the lines back into your robot file.

### Set up or check a robot

- **Calibrate the drivetrain** — [Calibrating your robot](guide/19-calibrating-your-robot.md): one run
  that measures the drive and saves it to the SD card.
- **Use the camera** — [The camera](guide/20-the-camera.md): what it sees, how a Toggle's colour is
  judged, and what it cannot tell.
- **Understand a log line, an error or a refusal** — [Reading the diagnostics](guide/11-reading-the-diagnostics.md),
  [When things go wrong](guide/12-when-things-go-wrong.md), and the [FAQ](faq.md).

### Learn how it works

The [user guide](guide/README.md) read in order assumes no robotics background: the field and its
coordinates, how a robot knows where it is, the drivetrains, and how it gets from one place to
another — before any code appears. The [glossary](guide/15-glossary.md) defines every term.

### Look something up

- **What is not built yet** — [What it cannot do yet](guide/14-what-it-cannot-do-yet.md).
- **The exact signature of anything** — the [API reference](api/README.md), generated from the code
  so it can never disagree with it, with an [A–Z index](api/all-entities.md). For routes you mostly
  need two pages: [`Chassis`](api/chassis.md) and [`Routine`](api/routine.md).
- **What changed, and whether it affects you** — the [changelog](changelog.md).

---

## Why this library exists

Most VEX teams use LemLib, which drives only tank robots. Our robots have been X-drive and H-drive,
which LemLib cannot drive at all — that is why this exists. This season both of our robots are
tanks, and shulib drives them as well.

It is built around three questions a robot answers all the time: **where am I** (odometry fused with
the inertial sensor, corrected against field walls and, when available, a GPS or AprilTags), **how do
I get over there** (motion that can translate and turn at the same time — an X-drive or H-drive robot
can slide sideways, so the library is built around that rather than treating it as an afterthought;
it drives tank robots too), and **what just happened** (a plain record of every step, plus a blackbox
log that survives without a laptop).

---

## How these documents stay true

Documentation that drifts is worse than none, because people trust it. These are enforced by the
build itself, not by anyone remembering:

- **Every code example compiles and runs.** Examples live in the test suite, and the pages quote them
  word for word; the build fails if a listing drifts from the code that compiles it.
- **The API reference is regenerated from the code,** and the build fails if the published copy is
  out of date — and if any public part of the library is left undocumented.
- **The parts teams build on are frozen,** and changing one fails the build, naming the rule it broke.

---

## Reference material

- [**Roadmap**](roadmap.md) — every milestone and task, and where the project is now
- [**Architecture and design decisions**](shulib-v2-master-plan.md) — why the library is shaped the
  way it is, including the alternatives that were rejected
- [**Hardware assumptions register**](hardware-assumptions.md) — every physical number that is still
  an estimate, how it will be measured, and what breaks if it is wrong
- [**Diagnostics plan**](diagnostics-plan.md) — the telemetry and logging contract
- [**VEXBuilder integration**](vexbuilder-integration.md) — how routes drawn in VEXBuilder map onto the library
- [**Legacy command vocabulary**](legacy-command-vocabulary.md) — what the previous codebase's
  routine format meant

---

*Source: [github.com/SHU-ROBOTICS/shulib](https://github.com/SHU-ROBOTICS/shulib)*
