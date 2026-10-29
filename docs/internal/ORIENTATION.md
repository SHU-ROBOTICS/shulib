# shulib, from scratch — the plain-English orientation

> **Who this is for:** a new team member, or anyone who opens this repo and wonders what it is.
> No jargon, no chunk codes, nothing assumed. First written 2026-08-14 because the team lead asked for
> something they could hand to a teammate — fifteen guide chapters and a hundred thousand words
> existed, and none of them answered *"what is this project and where is it?"* in one page.
> **Rewritten 2026-09-24:** the August version said our robots were X-drive and H-drive (both of this
> fall's robots are tank drives) and listed three missing pieces that have since been built. A page
> written to orient a confused reader is the worst page to leave stale.
> **Brought up to date 2026-09-29,** when the bench auton program landed: the counts, what has run on
> robot two since, and the next steps.

---

## The problem

In a VEX match the first 15 seconds are **autonomous** — no driver, the robot acts alone. Doing that
well means the robot must know where it is on the field, decide where to go, and get there
accurately. That is hard, and it is what this library does.

## Why build our own

Most teams use **LemLib**. It supports only *tank* drives — robots that turn like a car. shulib was
started so that X-drive and H-drive robots — which can slide sideways — could be driven properly, and
it does three things LemLib does not:

- **Move sideways and turn at the same time**, as independent motions
- **Combine several sensors** into one confident estimate of where the robot is
- **Be usable by someone who cannot write C++** — *half built: a routine can be data, but nothing
  loads one from a file yet*

**This season is different from the plan.** Both of our fall 2026 robots are **tank drives** (the
team lead's call, 2026-09-14); the X-drive comes in spring 2027. So this fall shulib drives tanks —
which it does fine — and the sideways-motion half waits for the spring robot to be proven.

## The four jobs

1. **Talk to hardware.** Read an encoder, command a voltage, read the gyro.
2. **Know where it is.** Fuse wheel movement, gyro, GPS and camera into one position.
3. **Get somewhere.** "Drive to that spot facing that way" — and actually arrive.
4. **Run a routine.** A sequence of moves, with a guarantee it stops safely before time runs out.

All four are **built and heavily tested on the laptop.** Only the first has been tried on a robot.

## The trick that made it possible without a robot

There was no robot for most of this project. So instead of guessing, we built a **host test plant**:
a laptop-only model of a robot — voltage in, motion out, sensors reading back — that knows exactly
where the robot *really* is, so a test can grade a routine by how close it ended up. Then we made it
*lie* the way real hardware lies: drifting gyros, dropouts, sagging batteries, wheels that slip.
(It is not a visual simulator you can watch; it is a grading tool inside the tests.)

Everything above the hardware layer was developed and proven against it. That is why there are over
a million and a half test assertions, and why the library works at all when no robot existed.

**What the host test plant cannot tell you is whether your numbers are right.** Every constant —
degrees per encoder tick, what a millivolt means, how fast the robot accelerates — is a written-down
*guess* until a real robot measures it, and every guess is tracked in a register.

## Why the strange names

Work happens in **chunks**: one focused piece at a time, each with a written plan, adversarial tests,
and a record of what it actually proved. They are lettered by phase — **A** foundations, **C** making
it move, **D** making it usable, **E** position accuracy, **F** sequencing, **R** the robot, **T**
driver control, **G** the no-code path, **H** ecosystem.

`R1a` means "robot phase, first chunk, first half." The letters are bookkeeping and nothing more.

**`HA-` numbers** are hardware assumptions — the list of every guess, each with what it would break
if wrong and the measurement that settles it.

**Freeze rows** (`F1`…`F14`) are contracts promised not to change without a version bump. Confusingly
they share letters with chunk names and are *not* the same thing.

## Three different kinds of "proven" — read this before anything else

**Almost every confusing sentence in this project is one of these three being mistaken for another.**
"The library is nearly finished" and "the library's motion has never driven a robot" are both true,
and they are true at different tiers.

| Tier | What it proves | How much we have (2026-09-29) |
|---|---|---|
| **1. The host test plant** | the **logic** is right — the maths, the control loops, the decisions | **Enormous.** 1,808 tests / 2.65M assertions, graded against exact ground truth |
| **2. ARM compile + boot** | it will **build and run** on a V5 brain | **Complete for what exists.** All 193 headers cross-compile; seven robot programs build and link every commit; the core booted on a brain 2026-08-12 |
| **3. Physical hardware** | the **numbers** are right — what a millivolt is, which way is clockwise | **A little.** Motors spun at 2 V and seven conversions confirmed (2026-08-13); the gyro's direction and the SD card confirmed (2026-08-18/19); robot two's ports and motor directions measured (2026-09-10); robot two driven by hand through the library's hardware layer once (2026-09-17, no log); shulib Calibrate run on robot two (first 2026-09-24), and robot two driven from its calibration record by the earlier, hand-wired version of shulib Teleop (2026-09-25). **The library's own steering has never moved a wheel.** |

## Where it stands (2026-09-29)

- **Built and proven on the host test plant:** the whole thinking half — three drivetrain types,
  sensor fusion, motion (a tank that drifts off its line now steers back onto it), sequencing, two
  ways to write a routine in code, a way to write a routine as **data**, a robot described as data
  and built from it, the drivetrain teaching itself its own facts (which motors, which way, how far
  apart the wheels are), driver-control feel, the claws' buttons, diagnostics, and the documentation.
- **Built for the robot:** the V5 hardware layer (twenty-one files) and seven programs — a measuring
  tool ("Bench Tests"), the driving program ("shulib Drive": the team's driver file — which button does
  what — on the library, with the library's drive protections and the claws; since T4 Part 3 it
  replaces both the old plain "shulib Drive" and "shulib Teleop"), a calibration program ("shulib
  Calibrate"), a bench auton program ("shulib Auton Tester": pick one of the team's autons on the
  brain's screen, press RUN, and it runs exactly as it will in a match — any button stops it) and the
  program for a match ("shulib Competition": the auton chosen on the brain's screen runs in
  `autonomous()`, then driver control runs the same driver file — built 2026-09-30) for robot two,
  plus two others.
- **Run on a robot:** Bench Tests (in several sessions), the OLD shulib Drive (once, 2026-09-17),
  shulib Calibrate (from 2026-09-24) and an earlier, hand-wired version of shulib Teleop
  (2026-09-25). **Today's shulib Drive, shulib Auton Tester and shulib Competition have never run.**
- **Published:** everything through 2026-08-15 is live at docs.shurobotics.com.
- **Not built yet:** the camera adapter,
  mechanisms in general (any mechanism a robot has, plugged into whatever ports it uses that day, and
  usable inside an auton — planned next), the scoring moves, real tuning numbers, and the loading of
  routines from a file. **And the students' own routines are the students' to write** — the library
  supplies the moves, not the strategy.

**An auton already works on the host test plant.** A six-step routine — move, move, strafe, turn,
hold, brake — settles **0.228 in** from its target in 9.56 s, and the team's 40 in auton, run through
the Auton Tester's exact path, finishes at 39.77 in after 1.80 s. Those are measured results from
passing tests, not goals — and they are not robot results.

## The sentence that governs everything

**The library's motion has never driven a robot.**

It has booted on a brain. Its hardware layer has spun motors and read sensors, and a driver has
driven robot two through it. But nothing has ever closed a control loop — no wheel has ever turned
under the library's own steering.

That distinction is defended in about six places across the documentation **on purpose**, because it
is the easiest thing in this project to start quietly lying about.

## What stands between here and a robot that drives itself

1. **The first auton on robot two** — the Auton Tester runs the team's 40 in auton from its RUN
   button, wheels on the floor, with a tape measure beside it (the bench worksheet's Station M). It
   needs only the robot, its calibration record and the USB cable. That run is the moment "the library
   has driven a robot" becomes true — or the first measurement of why not.
2. **The match program on the robot, and the driver program** — the program uploaded for a
   competition ("shulib Competition": an auton picker, the field's autonomous period, then driver
   control) is BUILT, not run: its first run is the bench worksheet's Station K, a practice match
   through the controller's Timed Run. The driver file's own program, the new "shulib Drive", is built
   too (T4 Part 3) and not run: its first run is Station F.
3. **Real numbers** — tuning measured on the robot instead of guessed, and position corrections
   (a wall reset, GPS or the camera) so the estimate does not drift over a routine.

The full list, with who decides what, is `docs/internal/HANDOFF-remaining-work.md`.
