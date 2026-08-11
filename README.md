<img src="docs/assets/banner.png" alt="shulib">

**shulib** — the Seton Hall University VEX U team's autonomous-robotics library, rebuilt from
the ground up for holonomic robots.

---

## What is this?

[VEX U](https://www.vexrobotics.com/v5/competition/vex-u) is a university robotics competition:
two robots per team, driven by a [V5 "brain"](https://www.vexrobotics.com/v5-architecture)
(an ARM Cortex-A9 controller), scoring points in one-minute matches that begin with a fully
**autonomous** period — the robot acts entirely on its own code and sensors.

shulib is the C++ library our autonomous code is built on. It answers three questions a robot
has to answer continuously, sixty times a second or more:

1. **Where am I?** — position tracking ("localization") that fuses wheel encoders and an
   inertial sensor into one trustworthy field position, and refuses to be fooled by a stuck
   wheel or a sensor that boots up lying.
2. **How do I get over there?** — motion control that drives to a target position and heading
   **simultaneously and independently**. Our robots are *holonomic* (their drivetrains can
   translate in any direction while rotating — think of an office chair, not a car), and the
   library is designed around that ability rather than treating it as an afterthought.
3. **What just happened?** — diagnostics on every tick of every motion: structured logs,
   per-motion result lines, fault codes, health monitoring. When a run goes wrong, the terminal
   output is designed to tell you exactly where and why.

It runs on [PROS](https://pros.cs.purdue.edu/), the open-source V5 runtime — but the library
core never touches PROS directly (that separation is load-bearing; see the layout section).

## What it does today

- **Motion**: `moveTo` / `strafeTo` / `turnTo` / `followTrajectory` / `drive`, plus `brake` and
  `hold` — blocking verbs on a `Chassis` facade. Every motion is watchdog-bounded (it can
  *never* hang), reports an honest exit reason (`Settled` / `TimedOut` / `Cancelled`), and can
  be cancelled into a defined safe state at any instant.
- **Three drivetrains** behind one kinematics contract: X-drive, tank, and H-drive — the same
  autonomous routine drives all three, and drivetrains with limited sideways authority degrade
  predictably instead of silently.
- **Localization**: arc-based odometry (exact SE(2) integration) with IMU-owned heading, fused
  behind a correction seam that GPS- and vision-based correctors plug into later.
- **Diagnostics**: per-tick structured records, leveled logs, first-fault latching, brownout and
  over-temperature monitoring, per-motion result lines, and an end-of-run summary — all through
  one telemetry seam.
- **A simulated proving ground**: a host-side plant (voltage in, motion out) plus *hostile*
  sensor models that lie the way real V5 hardware lies — IMU calibration garbage, GPS dropouts,
  encoder freezes, battery sag, wheel slip, sensor latency. The entire motion stack is tested
  against all of it, off-robot.

## What it is NOT yet

**This library has never run on a physical robot.** That is the single most important fact
about its current state, so here it is, third heading from the top:

- Everything verified so far is verified **off-robot**, against a simulated plant and simulated
  sensors. The simulation was built to be hostile, not flattering — but it is still simulation.
- The hardware adapters that would connect the library's motor/sensor interfaces to real V5
  devices **do not exist yet** (they are the next phase of work; the seams they fill are marked
  `TODO(R1)` in `src/main.cpp`). `make` produces a real, uploadable V5 package that boots and
  prints a diagnostics banner — and drives nothing.
- Every physical constant in the tree (gains, geometry, sensor noise) is a labeled stand-in.
  [`docs/hardware-assumptions.md`](docs/hardware-assumptions.md) inventories all 49 of them as
  falsifiable claims, each with its blast radius if wrong and the measurement that settles it.
- No vision-based position correction yet (the seam exists; the correctors are planned work),
  and no no-code routine authoring — routines are written in C++ today.

## How verified is it, honestly?

**659 test cases, 915,570 assertions, all green, all off-robot.** That includes: closed-loop
motion on three drivetrains graded against ground truth the estimator cannot see; a 9-attack
survival matrix (every simulated hardware pathology must degrade to a fault code, never a
crash or a NaN pose); byte-pinned diagnostic output; and a measured heading-accuracy test
(worst end-of-60-seconds error 0.912° across 10 seeded boots under full sensor hostility,
against a 1.0° requirement — with the caveat that the drift magnitudes are provisional until
measured on real sensors). Load-bearing logic is mutation-tested: break the code deliberately,
watch the test go red, restore it. CI additionally cross-compiles every library header for the
V5's Cortex-A9 at strict warning levels, so no host-only assumption can enter the tree
unnoticed — compiling for the target is verified; *running* on it is not.

## Build and test

Prerequisites: `cmake` (≥ 3.20) and a C++20 host compiler for the tests;
`arm-none-eabi-g++` for the V5 package.

Host test suite (no robot, no PROS — this is where the library actually gets verified):

```sh
cmake -S test -B build/test
cmake --build build/test -j
./build/test/shulib_tests
```

Expected tail of the output:

```text
[doctest] test cases:    659 |    659 passed | 0 failed | 3 skipped
[doctest] assertions: 915570 | 915570 passed | 0 failed |
[doctest] Status: SUCCESS!
```

(The 3 skips are deliberate placeholders for on-robot acceptance numbers.)

The V5 package (compiles and links the real ARM binary; uploading needs a V5 brain and
[pros-cli](https://pros.cs.purdue.edu/v5/getting-started/), and remember — it boots, it does
not drive):

```sh
make            # produces bin/hot.package.bin + bin/cold.package.bin
pros upload     # only with a V5 connected
```

## What using it looks like

An autonomous routine is plain, sequential code against the `Chassis` facade. Each verb blocks
until the robot settles — or honestly reports why it stopped — and cannot hang:

```cpp
#include "shulib/chassis/chassis.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/units/literals.hpp"

using namespace shulib;
using namespace shulib::units::literals;

control::ExitReason firstScore(chassis::Chassis& chassis) {
    // Tell the localizer where the robot starts (heading comes from the IMU).
    chassis.setPose(math::Pose2d{-48_in, -24_in, 90_deg});

    // Drive to a field position AND rotate to a heading, simultaneously —
    // translation and rotation are independent on a holonomic drive.
    chassis.moveTo(math::Pose2d{-24_in, 0_in, 45_deg}, {.timeoutSeconds = 3.0});

    // A slow, precise approach: per-call options cap this leg's speed.
    chassis.moveTo(math::Pose2d{-12_in, 12_in, 45_deg},
                   {.timeoutSeconds = 2.0, .maxLinearSpeed = units::Velocity{20.0}});

    // Sideways to the goal, actively holding the current heading.
    chassis.strafeTo(-12_in, 24_in, {.timeoutSeconds = 2.0});

    // Face the corner — always the short way around.
    const control::ExitReason last = chassis.turnTo(135_deg, {.timeoutSeconds = 1.5});
    return last;  // Settled, TimedOut, or Cancelled — never a lie, never a hang
}
```

Units are typed (`-48_in`, `90_deg` — a bare `double` will not compile where a length or angle
belongs), every motion carries its own timeout, and if a fault fires mid-motion (say, the
odometry catches a wheel lying) the motion aborts into a safe state and the exit reason says so.

Wiring a `Chassis` is ordinary C++ too — construct a kinematics preset, the hardware interfaces,
the localizer, and hand them over; no config file or code generator is ever required.
`src/main.cpp` is the complete, commented wiring example (with its hardware seams honestly
marked `TODO(R1)`), and the test suite builds the same stack in one place against simulated
hardware (`test/chassis_facade_test.cpp`, the "standalone" case).

## How the tree is laid out

```text
include/shulib/     the library — pure C++20, PROS-free by CI guard
  units/ math/      typed quantities (inches/radians/seconds), Pose2d, frames, wrapped angles
  spec/             frozen accuracy targets
  hal/              the 10 hardware interfaces (motor, IMU, GPS, clock, …) + in-memory fakes
  kinematics/       drivetrain math: X-drive, tank, H-drive behind one contract
  control/          PID, feedforward, motion profiles, settling, watchdogs
  localization/     odometry, tracking wheels, the fused localizer + correction seam
  motion/           motion primitives, the one-active-motion scheduler, stall cross-check
  chassis/          the public facade (Chassis) + the composition root (RobotContext)
  diag/             structured records, sinks, fault latch, health/loop monitors
  sim/              the host plant + hostile sensor models (tests only — see rule 2)
src/main.cpp        the PROS entry point — the ONLY file that sees both PROS and shulib
test/               the host suite (doctest): 659 cases against plant + hostile sims
docs/               documentation (roadmap, architecture, hardware assumptions)
firmware/, include/pros|liblvgl, Makefile, common.mk    vendored PROS kernel + V5 build
```

Two structural rules, both enforced by CI grep-guards, keep the design honest:

1. **The library never includes PROS.** Everything under `include/shulib/` compiles on any
   C++20 compiler — which is exactly what makes the 915k-assertion host suite possible. Only
   `src/main.cpp` touches the PROS runtime.
2. **The core never includes the simulator.** Code under test can only see sensor interfaces,
   never the simulated ground truth — so a test can grade the estimator against a truth the
   estimator provably cannot peek at.

## Where to go next

- **[`docs/guide/`](docs/guide/README.md) — the user guide.** Start here if you're new: what
  the autonomous problem is, every concept from pose to PID in plain language, a full
  first-routine tutorial (every code example compiles and runs in the test suite), the API as
  prose, and how to read the diagnostics line by line. Written for a reader with no robotics
  background.
- [`docs/roadmap.md`](docs/roadmap.md) — everything remaining, by milestone, with an honest
  "you are here" (including what is deliberately *not* claimed yet).
- [`docs/shulib-v2-master-plan.md`](docs/shulib-v2-master-plan.md) — the architecture and the
  reasoning behind every locked decision.
- [`docs/hardware-assumptions.md`](docs/hardware-assumptions.md) — all 49 hardware claims the
  no-robot build rests on, and the plan for settling each one.
- [`docs/diagnostics-plan.md`](docs/diagnostics-plan.md) — the observability design.
- The headers themselves are written to be read — each one opens with *why it exists*, not
  just what it does. The guide points into them; going deeper than the guide means opening
  them.

## Acknowledgments

- **[@n0es](https://github.com/n0es)** — wrote the original shulib: the architecture, the odometry
  system, and the multi-robot config setup.
- **[LemLib](https://github.com/LemLib/LemLib)** — inspired the original library's design.
- **[PROS](https://pros.cs.purdue.edu/)** — the V5 runtime this library runs on.
- **Purdue ACM SIGBots** — for building PROS and keeping it open.

---

## Contributing

The repo is public because we'd like other teams to actually use this. Fork it, take what's useful.
Questions and bug reports go in [Issues](https://github.com/SHU-ROBOTICS/shulib/issues).

If you're sending code, two rules — the same ones we hold ourselves to:

1. **Write tests that try to break the code**, not ones that confirm it works. If you can't say
   what bug a test would catch, it isn't earning its place.
2. **Be honest about status.** Something is done when there's evidence it's done — a passing test,
   a real measurement. Half-finished stays half-finished. Most of this repo is only useful because
   that rule held.

## License

shulib is released under the [MIT License](LICENSE) — use it, modify it, ship it.

---

<p align="center">
  <b>Built by Seton Hall University Robotics</b><br>
  <i>For the next generation of builders</i>
</p>
