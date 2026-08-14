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

## Where it stands (2026-08-14)

- **Done:** the thinking half, completely — three drivetrain types, sensor fusion, motion,
  sequencing, two ways to write a routine, and the documentation.
- **Just finished:** the hardware layer — fourteen adapters translating between the library and real
  V5 devices.
- **Proven on a real robot:** it commanded a physical motor and every conversion it performed was
  correct. Seven guesses became measurements.
- **Not built:** the no-code authoring tool (the entire "usable without C++" promise), driver-control
  feel, real tuning constants, and the students' own scoring routines.

## The sentence that governs everything

**The library has never driven a robot.**

It has booted on a brain. It has spun a motor. Every conversion it performs is verified. But nothing
has ever closed a control loop — no wheel has ever turned under the library's own steering.

That distinction is defended in about six places across the documentation **on purpose**, because it
is the easiest thing in this project to start quietly lying about.

## What stands between here and a robot that drives

1. **Tell it what the robot is.** It currently believes it has four sliding wheels and two measuring
   wheels. The available robot has seven drive motors, no measuring wheels and no GPS. Small fix, no
   hardware needed.
2. **Check the compass.** The gyro works, but nobody has confirmed which direction it calls positive.
   Needs a person, the robot and a protractor.
3. **Drive it**, measure how far off it is, and fix that.

Everything else — the camera, the no-code tool, driver controls — is real work, and **none of it is
on that path.**
