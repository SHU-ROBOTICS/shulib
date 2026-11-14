# 7 — Getting set up

> **Covers:** getting the code onto your machine, building it, running the test suite, reading
> the test output, and how the repository is laid out.
> **Read this if:** you're ready to touch the code. This is the first hands-on chapter.
> **Assumes:** [Chapter 1](01-what-is-this.md). You can open a terminal and type commands into
> it; everything else is explained.

## The counterintuitive part first

You do not need a robot to work on this library — and you won't touch one for a while even if
you have one. Almost all development happens **on your own computer** ("host-side"): the library
is tested against a simulated robot, and your first autonomous routine (next chapter) will run
in that simulation too. This is a feature, not a workaround — a test that runs in seconds on a
laptop gets run a thousand times more often than one that needs a charged battery and a
practice field.

## Prerequisites

You need three tools. On Ubuntu/Debian Linux:

```sh
sudo apt-get install git cmake g++ python3
```

- **git** — version control; how you get and contribute code.
- **cmake** (version 3.20 or newer) — the build configurator: it works out how to compile the
  project on your machine.
- **g++** — the C++ compiler itself (any compiler with C++20 support works; clang is fine too).
- **python3** — not for the library, which is pure C++ and depends on nothing. The test build
  runs the documentation gates with it: a public member that ships with no documentation fails
  the build by name, the generated [API reference](../api/README.md) is checked up to date, and
  every code example in the documentation is checked against the compiled test that proves it.
  There is deliberately no way to switch those off, so python3 is required rather than optional.
  Ubuntu, macOS and WSL all ship it; `python3 --version` tells you.

macOS: install the Xcode command-line tools (`xcode-select --install`) and cmake
(`brew install cmake`). Windows: WSL (Windows Subsystem for Linux) with the Ubuntu instructions
above is the path of least resistance.

That's all for library work. Two more tools exist that you do **not** need yet:
`arm-none-eabi-g++` (the cross-compiler that builds for the V5's processor) and
[pros-cli](https://pros.cs.purdue.edu/v5/getting-started/) (the tool that uploads to a V5
brain). They only matter for robot-package work — and remember, the library still runs on no
robot: a robot exists for the hardware phase's bench work — its adapters have been checked
against real motors and sensors — but nothing has yet driven under motion control, and everything
you will do from this guide is host-side.

## Clone and build

```sh
git clone https://github.com/SHU-ROBOTICS/shulib.git
cd shulib
cmake -S test -B build/test        # configure (first time, or after adding files)
cmake --build build/test -j        # compile everything
./build/test/shulib_tests          # run the whole test suite
```

The first build takes a few minutes (it is compiling the whole suite); later builds
only recompile what changed. The [README](../../README.md) carries the same commands and is the
canonical copy if these ever drift.

## Reading the test output

The suite prints a burst of text and ends with something shaped like this. **Match the shape, not
the numbers** — the counts grow every time anyone adds a test, and the only current figures are
the ones your own run just printed:

```text
[doctest] test cases:    ... |    ... passed | 0 failed | 3 skipped
[doctest] assertions: ...... | ...... passed | 0 failed |
[doctest] Status: SUCCESS!
```

*(This block used to carry real counts. They went stale — by hundreds of cases and hundreds of
thousands of assertions — and no gate noticed, because a `text` block is checked by none of
them. That is why the shape is shown here and the numbers are not.)*

How to read it:

- **doctest** is the test framework we use — each `*_test.cpp` file in `test/` declares test
  cases, and this runner executes them all.
- A **test case** is one named scenario ("a stuck encoder raises ODO_STUCK within 0.3 s"); an
  **assertion** is one checked claim inside a case. The assertion count is huge because many
  tests sweep hundreds of situations in a loop, asserting at each step.
- The **3 skipped** are deliberate placeholders, and they are not all the same kind. Two are
  acceptance stubs for accuracy targets whose features do not exist yet; the third needs a GPS
  in front of a field strip. They are marked skipped rather than deleted so the suite itself
  remembers what is owed.
- **Status: SUCCESS!** with `0 failed` is the only acceptable end state. If you see failures on
  a fresh clone, something is wrong with the environment, not (probably) the code — ask before
  digging alone.

When a test fails, doctest prints the file, line, the failed claim, and the actual values —
e.g. `test/pid_test.cpp:74: CHECK( output < 12.0 ) is NOT correct! (values: 14.3 < 12.0)`. Read
it top-down: the *first* failure is usually the real one, later ones often cascade.

Worth knowing early: this team's tests are written to *attack* the code — deliberately feeding
it lying sensors, sagging batteries, and impossible targets — not to confirm it works on sunny
days. `test/README.md` explains the testing philosophy and its rules; you'll be held to them
when you contribute.

## The layout, and why it's split this way

The [README](../../README.md#how-the-tree-is-laid-out) has the full annotated tree; here's the
mental model, which matters more:

- **`include/shulib/`** — the library itself. Everything under here is plain C++20 with **zero
  dependency on VEX's software** (PROS, the V5 runtime). That rule is what makes the entire
  library testable on your laptop, and it's enforced by an automated check on every commit, not
  by good intentions.
- **`test/`** — the host test suite, including the simulated robot it runs against
  (`include/shulib/sim/` holds the simulation; a second automated check ensures the library
  never peeks at the simulator's ground truth — code under test can't cheat).
- **`include/shulib/hal/pros/`** — the one deliberate exception to the zero-dependency rule,
  and the newest part of the tree: the adapters that implement the library's hardware
  interfaces over the real PROS SDK (`ProsMotor`, `ProsImu`, `ProsGps`, …). The automated
  check exempts exactly this directory and nothing else. Each adapter is thin glue: it reads
  the raw device, applies the unit conversion **exactly once**, and hands the core a canonical
  value — every unit belief it rests on is catalogued in the
  [Hardware Assumptions Register](../hardware-assumptions.md) (HA-94 onward).
- **`src/`** — the programs that run on a real V5 brain, one folder per job: **`src/robots/`** is
  what each robot IS (its ports, motor directions and mechanisms), **`src/team/`** is the team's
  own code — where students edit: robot two's driver file, `src/team/robot_two_driver.hpp` (which
  button does what and the driving feel, in one file; "shulib Drive" runs it on its own, and the
  competition program's driver control runs it, [Chapter 17](17-the-competition-program.md)), and
  the team's autons,
  `src/team/autons.cpp` (every autonomous routine the robot can run, listed by `registerAutons()` —
  the one file the host suite also compiles, and runs every auton in; the auton tester runs them on
  the bench, [Chapter 16](16-testing-an-auton-on-the-bench.md), and the competition program in a
  match, [Chapter 17](17-the-competition-program.md)) —
  **`src/programs/`** holds one program each, and **`src/common/`** what the programs share.
  Every include of one of these files is written from `src/` —
  `#include "robots/chassis_table.hpp"` — the same in every folder. `src/build_selection.hpp` is
  where a build becomes one program on one robot: the `make ROBOT=… PROGRAM=…` switches are read
  there and nowhere else, by one chain that picks the program and the robot's tables together.
  `src/main.cpp` is the PROS entry point, and all it does is hand each PROS entry point to that
  program. "shulib Drive" (`src/programs/drive_program.cpp`) is the worked example of building a
  whole robot — `robot::Robot` from a config over the real adapters, then robot two's driver file
  bound to it — and
  `src/robots/xdrive_config.hpp` shows a robot typed by hand as a `constexpr RobotConfig` (the
  invented X-drive, built the same way by `src/programs/xdrive_program.cpp`). The other programs are
  the bench tester ("Bench Tests"), "shulib Calibrate" (the library's standard Calibrate, with robot
  two's facts derived from its tables in `src/robots/calibrate_facts.hpp` — a team's own is one file,
  [chapter 19](19-calibrating-your-robot.md)), "shulib Auton Tester"
  (`src/programs/auton_tester_program.cpp`, `make ROBOT=tank PROGRAM=autontest`), "shulib
  Competition" (`src/programs/competition_program.cpp`, `make ROBOT=tank PROGRAM=competition`) and
  "shulib Tune" (`src/programs/tune_program.cpp`, `make ROBOT=tank PROGRAM=tune` — any robot with
  mechanisms; slot 7 — live tuning of the mechanisms on the brain screen,
  [Chapter 18](18-mechanisms.md#the-tune-program--tuning-on-the-brain-screen)) —
  robot two's builds; robot one, the cascade robot, takes the same six with `make ROBOT=one` and
  `make ROBOT=one PROGRAM=…`, each REFUSING, by name, every still-owed fact of its facts file that its
  program needs ([chapter 21](21-adding-your-robot-by-hand.md), and the
  [robot facts form](../robot-facts-form.md));
  `src/common/` holds what those programs share — the boot source, the competition-state screens, the
  painters, every driver file's plumbing (`src/common/driver_file.hpp`), and the driver-control
  screen's words (`src/common/driver_panel.hpp`: the drive protections' row, the controller's lines,
  and the mechanisms' boot lines and the claw row, whose buttons are the driver file's); and the
  per-robot tables in `src/robots/` carry every robot's ports, and robot two's claws, declared in the
  mechanism core's words (`src/robots/mechanism_table.hpp`). (Wiring a robot by hand, object by
  object, is still possible — the test suite's standalone case does it — but no program here does
  any more.) Which robot a build targets is
  chosen at `make` time: the X-drive build's port map is an
  explicitly labelled guess (no such robot exists yet), while the tank builds use port tables
  measured on the old bench robot and on robot two — and robot two's graph is built from the calibration record the
  robot writes to its own SD card.
- **`test/pros_shim/`** — a hand-written, programmable stand-in for the PROS SDK, used only by
  the host test build so the adapters can be compiled and *run* on your laptop. It is
  structurally unable to reach a robot build (its headers refuse to compile outside the test
  suite), and its limit is stated in its own headers: it tests the adapters against our
  beliefs about PROS — only hardware tests the beliefs.
- **`docs/`** — you are here. Plans, the assumptions register, and this guide.
- **`firmware/`, `Makefile`, …** — the vendored V5 build machinery (used by `make`, which
  builds the robot package; not needed for library work).

## What about putting code on a robot?

For completeness, the path that exists today: `make` at the repo root cross-compiles a real,
uploadable V5 package, and `pros upload` puts it on a brain. Booting has been done (2026-08-12:
uploaded, booted, printed its banner over USB), and since the hardware-adapter work landed the
package wires **real device adapters** — motors, IMU, battery, the controller, and on the
X-drive build GPS and rotation sensors — rather than in-memory fakes, including a first
driver-control loop.

What that does *not* yet mean, stated as plainly as the robot itself states it over serial:
**the library has still never driven a robot under its own control** — no control loop has ever closed, and no wheel
has ever turned under the library's own steering. The adapters are host-tested against a
programmable stand-in for PROS, which proves the glue is faithful to our *beliefs* about PROS
— units, signs, error values — and cannot prove the beliefs themselves
(the [FAQ](../faq.md) unpacks that sentence). A few bench sessions (2026-08-13 onward) have since
checked a handful of those beliefs against real devices and found them right, and a driver has
driven robot two with the sticks through the adapters once — a real result and a narrow one: it
validated the *platform layer*, open-loop, not the library's steering.

Only the X-drive build's port map is a labelled guess; the tank builds use measured port tables
(robot two's is owed a re-measure since its drivetrain was rebuilt). A wrong **motor** port does refuse loudly at boot — `ProsMotor` configures the device and reads
the configuration back — but do not generalise that to every device: the sensor adapters screen
bad reads and hold their last good value rather than refusing, so a mis-mapped sensor degrades
quietly instead of stopping you.
First motion, sign checks, and validation are the hardware phase's, walked as a prepared
checklist. The [roadmap](../roadmap.md#milestones-at-a-glance) tracks exactly where that
stands (the "you are here" note is kept current). Until then, everything in this guide runs
host-side, and nothing you'll learn is wasted — the code you write against the simulator is
the same code that will run on the robot.

---

*Next: [Chapter 8 — Your first autonomous routine](08-your-first-routine.md)*
