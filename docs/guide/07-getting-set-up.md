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
brain). They only matter for robot-package work — and remember, there is no robot yet.

## Clone and build

```sh
git clone https://github.com/SHU-ROBOTICS/shulib.git
cd shulib
cmake -S test -B build/test        # configure (first time, or after adding files)
cmake --build build/test -j        # compile everything
./build/test/shulib_tests          # run the whole test suite
```

The first build takes a few minutes (it's compiling several hundred test cases); later builds
only recompile what changed. The [README](../../README.md) carries the same commands and is the
canonical copy if these ever drift.

## Reading the test output

The suite prints a burst of text and ends with something shaped like this (the exact counts grow
over time — the [README](../../README.md#how-verified-is-it-honestly) tracks the current
numbers):

```text
[doctest] test cases:    659 |    659 passed | 0 failed | 3 skipped
[doctest] assertions: 915570 | 915570 passed | 0 failed |
[doctest] Status: SUCCESS!
```

How to read it:

- **doctest** is the test framework we use — each `*_test.cpp` file in `test/` declares test
  cases, and this runner executes them all.
- A **test case** is one named scenario ("a stuck encoder raises ODO_STUCK within 0.3 s"); an
  **assertion** is one checked claim inside a case. The assertion count is huge because many
  tests sweep hundreds of situations in a loop, asserting at each step.
- The **3 skipped** are deliberate: placeholder cases for measurements that can only be made on
  a real robot. They're marked skipped rather than deleted so the suite itself remembers what's
  owed.
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
- **`src/main.cpp`** — the one and only file that touches both worlds: the program that runs on
  a real V5 brain and wires the library to real hardware. Today its hardware connections are
  explicitly marked `TODO(R1)` — stand-ins awaiting the hardware-adapter phase.
- **`docs/`** — you are here. Plans, the assumptions register, and this guide.
- **`firmware/`, `Makefile`, …** — the vendored V5 build machinery (used by `make`, which
  builds the robot package; not needed for library work).

## What about putting code on a robot?

For completeness, the path that exists today: `make` at the repo root cross-compiles a real,
uploadable V5 package, and `pros upload` would put it on a brain. It boots and prints a
diagnostics banner — **and drives nothing**, because the adapters connecting the library's
motor/sensor interfaces to real V5 devices don't exist yet. The
[roadmap](../roadmap.md#milestones-at-a-glance) tracks exactly what stands between here and a
driving robot (the "you are here" note is kept current). Until then, everything in this guide
runs host-side, and nothing you'll learn is wasted — the code you write against the simulator is
the same code that will run on the robot.

---

*Next: [Chapter 8 — Your first autonomous routine](08-your-first-routine.md)*
