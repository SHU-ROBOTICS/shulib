# shulib, from scratch — the plain-English orientation

> **Who this is for:** a new team member, or anyone who opens this repo and wonders what it is.
> No jargon, no chunk codes, nothing assumed. Written 2026-08-14 because the team lead asked for
> something they could hand to a teammate — which was itself the finding: fifteen guide chapters and
> a hundred thousand words existed, and none of them answered *"what is this project and where is
> it?"* in one page.

---

## The problem

In a VEX match the first 15 seconds are **autonomous** — no driver, the robot acts alone. Doing that
well means the robot must know where it is on the field, decide where to go, and get there
accurately. That is hard, and it is what this library does.

## Why build our own

Most teams use **LemLib**. It supports only *tank* drives — robots that turn like a car. Our robots
are **X-drive and H-drive**: they slide sideways. LemLib cannot drive them at all.

So shulib exists to do three things LemLib does not:

- **Move sideways and turn at the same time**, as independent motions
- **Combine several sensors** into one confident estimate of where the robot is
- **Be usable by someone who cannot write C++** — *this part is not built yet*

## The four jobs

1. **Talk to hardware.** Read an encoder, command a voltage, read the gyro.
2. **Know where it is.** Fuse wheel movement, gyro, GPS and camera into one position.
3. **Get somewhere.** "Drive to that spot facing that way" — and actually arrive.
4. **Run a routine.** A sequence of moves, with a guarantee it stops safely before time runs out.

Jobs 2, 3 and 4 are **done and heavily tested**. Job 1 was built on 2026-08-13/14.

## The trick that made it possible without a robot

There was no robot for most of this project. So instead of guessing, we built **a simulated robot in
software** — voltage in, motion out, sensors reading back — and then made that simulator *lie* the
way real hardware lies: drifting gyros, dropouts, sagging batteries, wheels that slip.

Everything above the hardware layer was developed and proven against it. That is why there are over
a million test assertions, and why the library works at all when no robot existed.

**What a simulator cannot tell you is whether your numbers are right.** Every constant — degrees per
encoder tick, what a millivolt means — was a written-down *guess*, tracked in a register. A real
robot is the only thing that settles them.

## Why the strange names

Work happens in **chunks**: one focused piece at a time, each with a written plan, adversarial tests,
and a record of what it actually proved. They are lettered by phase — **A** foundations, **C** making
it move, **D** making it usable, **E** position accuracy, **F** sequencing, **R** the robot, **T**
driver control, **G** the no-code tool, **H** ecosystem.

`R1a` means "robot phase, first chunk, first half." The letters are bookkeeping and nothing more.

**`HA-` numbers** are hardware assumptions — the list of every guess, each with what it would break
if wrong and the measurement that settles it.

**Freeze rows** (`F1`…`F14`) are contracts promised not to change without a version bump. Confusingly
they share letters with chunk names and are *not* the same thing.

## Three different kinds of "proven" — read this before anything else

**Almost every confusing sentence in this project is one of these three being mistaken for another.**
"The library is nearly finished" and "the library has never driven a robot" are both true, and they
are true at different tiers.

| Tier | What it proves | How much we have |
|---|---|---|
| **1. Host simulation** | the **logic** is right — the maths, the control loops, the decisions | **Enormous.** 1,151 tests / 1.5M assertions, graded against exact ground truth |
| **2. ARM compile + boot** | it will **build and run** on a V5 brain | **Complete.** All 149 headers cross-compile; it booted on a brain 2026-08-12 |
| **3. Physical hardware** | the **numbers** are right — what a millivolt is, which way is clockwise | **Almost nothing.** 8 motors spun at 2 V once; 7 conversions confirmed; one robot, one day |

The library is roughly **95% done at tier 1 and 5% done at tier 3.** That single sentence explains
more of this repo's apparent contradictions than anything else here.

## Where it stands (2026-08-17)

- **Done, and proven at tier 1:** the thinking half, completely — three drivetrain types, sensor
  fusion, motion, sequencing, two ways to write a routine, diagnostics, and the documentation.
- **Done, and proven at tier 3 in part:** the hardware layer — fourteen adapters. Motors, battery
  and clock have met real devices. The other eleven have not.
- **Published:** everything through 2026-08-15 is live at docs.shurobotics.com.
- **Not built:** the no-code authoring tool (the entire "usable without C++" promise), driver-control
  feel, real tuning constants, the students' own scoring routines — **and three small pieces of the
  drivetrain layer nobody knew were missing until a real robot was held up against the code** (see
  below).

**An auton already works in simulation.** A six-step routine — move, move, strafe, turn, hold, brake
— settles **0.228 in** from its target in 9.56 s. That is a measured result from a passing test, not
a goal.

## The sentence that governs everything

**The library has never driven a robot.**

It has booted on a brain. It has spun a motor. Every conversion it performs is verified. But nothing
has ever closed a control loop — no wheel has ever turned under the library's own steering.

That distinction is defended in about six places across the documentation **on purpose**, because it
is the easiest thing in this project to start quietly lying about.

## What stands between here and a robot that drives

> **CORRECTED 2026-08-17, and the correction is the point.** This list previously said the first item
> was a *"small fix, no hardware needed"* — telling the library which ports the motors are on. That
> was wrong, and it was wrong in the most expensive way a document can be: it was the one actionable
> list in the one document written to orient a confused reader, and it sent them in the wrong
> direction. **Measured, not argued:** the port map is the easy part; the library is missing two
> capabilities underneath it. Written up in `build-order.md`'s R3b entry with the measurements.

**Step 1 — measure the robot.** Wheel diameter, gear ratio, track width, and which direction the
gyro calls positive. Mostly a ruler and a protractor; half a morning. **Nothing later is correct
until this is done**, because two of the pieces below take these numbers as inputs.

**Step 2 — build three small missing pieces.** Roughly 150 lines of library code between them:

1. **Command more than one motor per wheel.** The library sends one voltage to one motor per wheel.
   A real VEX drivetrain has 2–4 motors per side; the available robot has seven. Handing it seven is
   *accepted* today and **two get driven while five sit dead, silently.** The kinematics file says
   this aggregation is "the HAL's business" — and that facility was never built.
2. **Read distance from a drive motor's encoder.** Odometry (knowing where you are) currently
   requires two dedicated *Rotation Sensors* on unpowered tracking wheels. The available robot has
   none. Reading the drive motors instead is how most teams do it — LemLib does — and shulib cannot
   yet. This is also the only honest home for a **gear ratio**, a concept the library does not have
   anywhere.
3. **Say "this chassis cannot see sideways."** A tank robot cannot measure lateral movement, and
   odometry currently insists on a sensor for it. The kinematics layer already states exactly this in
   shipped code; the localization layer has not caught up.

**Step 3 — drive it**, measure how far off it is, and fix that.

### Two different autons, two different distances away

| Goal | What it needs | Realistic distance |
|---|---|---|
| **Heading-based auton** — turn to a heading under control, drive forward for a measured time | steps 1–3 above. **No odometry at all**: turning reads only the gyro | days |
| **Position-based auton** — "drive to that spot on the field" | the above, **plus two Rotation Sensors** to buy and mount, plus measured sensor noise and real tuning constants | weeks, and gated on parts |
| **The sideways-motion thesis** that justifies the whole project | a competition robot with an X or H drive, a GPS and a camera | gated on a robot that does not exist yet |

Everything else — the camera, the no-code tool, driver-control feel — is real work, and **none of it
is on that path.**
