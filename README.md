<img src="docs/assets/banner.png" alt="shulib">

**shulib** — the Seton Hall University VEX U team's autonomous-robotics library, rebuilt from
the ground up for holonomic robots.

---

## What is this?

[VEX U](https://www.vexrobotics.com/v5/competition/vex-u) is a university robotics competition:
two robots per team, driven by a [V5 "brain"](https://www.vexrobotics.com/v5-architecture)
(an ARM Cortex-A9 controller). A match pairs a driver-controlled phase with a fully
**autonomous** one, where nobody touches anything and the robot acts entirely on its own code
and sensors — and a separate event, Autonomous Coding Skills, is a full minute of nothing else.
That minute is what this library is built to win.

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
  *never* hang), reports why it stopped — its exit reason (`Settled` / `TimedOut` / `Cancelled`), and can
  be cancelled into a defined safe state at any instant.
- **Three drivetrains** behind one kinematics contract: X-drive, tank, and H-drive — the same
  autonomous routine drives all three, and drivetrains with limited sideways authority degrade
  predictably instead of silently.
- **Localization**: arc-based odometry (exact SE(2) integration) with IMU-owned heading, fused
  behind a correction seam with two correctors built on it — the V5 GPS (bounded position drift)
  and AprilTags (the only absolute *heading* source in the library) — and **two fusion tiers**
  behind that same seam: a bounded-nudge complementary filter (the default) and a 5-state SE(2)
  extended Kalman filter that can weigh two disagreeing correctors against each other by their
  stated accuracy, recover from a displacement the simpler filter refuses, and state how
  uncertain it is. All of it proven against simulated sensors only, and the EKF's noise
  parameters are guesses until a robot exists; see "How verified is it?" below. When the two
  tiers were raced head to head the Kalman filter came out **less** accurate (0.351″ vs 0.225″
  mean final error, losing 7 of 8 seeds), which is why the simpler one is still the default —
  the EKF is there for recovery, arbitration and stated uncertainty, not for accuracy.
- **Mechanisms**: one device seam (`IMechanism`) and two season-free bounded operations — run a
  motor mechanism until something confirms, and fire a discrete actuator then check it happened.
  Every operation is time-bounded and reports an outcome, and each mechanism declares its own
  safe state: motor mechanisms go safe on every exit, while a discrete actuator keeps a state it
  successfully commanded and goes safe only on cancel — a clamp whose safe state is "open" would
  fling its game piece the instant a grab succeeded. The library ships the grammar, not the
  nouns; there is no `shulib::Intake` and never will be. On top of that grammar sits the
  **mechanism core**: a team DECLARES each mechanism once in its robot file — an arm, a lift, a
  flywheel, a power roller, a pneumatic — and wires it separately, the build refusing each mistake
  by name; one mechanism service runs them all each tick (a move and its hold, homing, a spin to
  speed, a run until a piece sensor sees a piece, a valve's position), every behaviour watched for a
  jam; an auton runs a mechanism on one line (`r.move(arm["score"], 1.5_s)`), a driver file puts
  them on buttons (hold-to-run, toggles, presets), and a route's markers fire each mechanism's
  published ids. Both robots' programs run their mechanisms this way — the cascade, two side
  toggles and three pistons; robot two is robot one's twin (guide chapters 16–18 and 21; host-tested
  only — none of it has run on a robot yet).
- **A guaranteed end of run**: wrap the tick pacer and a routine gets a hard deadline — the
  active motion — and every mechanism operation still running that registered its claim, as the
  library's own do — is cut at the instant you name, every later motion is *refused* (a latch,
  not a pause), your end action runs strictly after cancel-all, and an unconditional floor fires
  regardless. The library refuses to know your strategy: no field coordinate, no park pose, no
  default match length, no default lead time. Both instants and the action are yours.
- **Diagnostics**: per-tick structured records, leveled logs, first-fault latching, brownout and
  over-temperature monitoring, per-motion result lines, and an end-of-run summary — all through
  one telemetry seam — plus an SD-card blackbox with a RAM flight recorder that costs the card
  nothing until a fault fires, and a decoder that ships with it.
- **A simulated proving ground**: a host-side plant (voltage in, motion out) plus *hostile*
  sensor models that lie the way real V5 hardware lies — IMU calibration garbage, GPS dropouts,
  encoder freezes, battery sag, wheel slip, sensor latency. The entire motion stack is tested
  against all of it, off-robot.

## What it is NOT yet

**This library has never driven a robot under its own control.** That is the single most important fact about its
current state, so here it is, third heading from the top:

- **Everything about motion and control is verified off-robot**, against a simulated plant and
  simulated sensors. The simulation was built to be hostile, not flattering — but it is still
  simulation, and **not one control loop has ever closed on hardware.**
- Eighteen hardware adapters exist (`include/shulib/hal/pros/` — motors, IMU, GPS, battery,
  encoders, controller, distance, optical, ADI digital in and out, USB serial, the controller
  LCD, the brain's own touch screen, the SD card (write and read — and, built over those two, the
  store the calibration record's safe save uses, which is not counted as an adapter of its own), the
  brain's device list, the loop pacer, and real time).
  The motor, IMU, battery, controller and clock adapters and the SD-card writer have met real
  devices on the bench. **Distance** is constructed by Bench Tests (its distance station) and **ADI
  digital out** by the programs that boot the robots' mechanisms (their pistons, 3-wire lines A to C);
  this README claims neither confirmed on a device. **Optical and ADI digital in are constructed by no
  program yet** — no V5 build compiles them into a package — and **the brain's touch screen adapter is built only into the robots' "shulib Calibrate"** (the
  library's standard Calibrate since 2026-11-05), which ran on robot one in the form before this one
  (2026-11-16 and 11-17) — and GPS
  and rotation appear only in the X-drive build, which has never run. All
  eighteen are host-tested against a hand-written stand-in for PROS, which proves the glue is
  faithful to *our beliefs* about PROS — units, signs, error values — and cannot prove the
  beliefs themselves; every such belief is a labelled entry in the assumptions register (HA-94
  onward) — except two the touch-screen adapter shares with the robot programs' own screen path
  (`src/common/screen_text.hpp`), read from the SDK and not yet entered: that PROS re-reads printed text as a format string, and that each
  print leaks its buffer (the adapter sanitizes and counts its prints either way). Only the
  X-drive build (`make ROBOT=xdrive`) carries an invented, labelled port map; the default build's
  table was measured on the old bench robot, and the two cascade robots' tables type only the drive
  cartridge — shulib Calibrate discovers each one's drive and writes it to that robot's SD card.
- **What has been confirmed on hardware — and only this.** On 2026-08-12 the package was built,
  uploaded, and booted on a V5 brain, where it constructed its whole object graph and printed
  its diagnostics banner over USB serial. Every motor and sensor it spoke to was still a fake.
  On 2026-08-13 that changed at the seam and nowhere else: the adapters commanded **eight real
  motors at 2.0 V** and read a real IMU, battery and clock, settling seven unit-conversion
  assumptions against a turning wheel — on one robot, once, which is an observation and not
  proof of portability. What those two runs establish is narrow and real: the build-and-upload
  path works, the library makes no host-only assumption that breaks on ARM, and the adapters'
  conversions are measured rather than believed. Later bench sessions added a little more: the
  SD card written and read back and the IMU's heading direction confirmed (2026-08-18/19, the
  old robot); the old robot two's ports and every drive motor's direction measured (2026-09-10); and
  a driver drove it with the sticks through the library's hardware adapters — open-loop, no
  library steering — once (2026-09-17, the team lead's report; no log was captured). **Nothing
  was ever steered.** None of it establishes anything about motion, accuracy, or control.
- Every physical constant in the tree (gains, geometry, sensor noise) is a labeled stand-in.
  [`docs/hardware-assumptions.md`](docs/hardware-assumptions.md) inventories them as falsifiable
  claims, each with its blast radius if wrong and the measurement that settles it.
- **A program now exists in which the robot measures its own drivetrain** — which ports drive,
  which way each motor is wired, which side each is on, the per-side inches per radian and the
  effective track width — and writes the answer to the SD card for the teleop program to boot
  from. **It has run on robot two (2026-09-24): one run stopped on its own consistency check and
  one finished with no log captured, so its recorded reproduction run is still owed.** Every
  threshold in it is an invented first guess, proven against the host test plant.
- The vision/AprilTag corrector exists, but **no camera has ever been pointed at a tag by this
  project**, and the library deliberately ships **no map of where the tags are** — that is your
  input, and nobody here can cite one. Its accuracy claims are simulation claims
  ([chapter 14](docs/guide/14-what-it-cannot-do-yet.md) states exactly what was measured and on
  what). No no-code routine authoring either — routines are written in C++ today.

## How verified is it?

**Over a thousand test cases and more than a million assertions, all green, all off-robot.** For the
exact figures, run them — `./build/test/shulib_tests` prints the count as the last thing it does.

That is deliberately not a number on this page. It used to be, and it went stale twice in a single
day: once when a chunk added tests, and once when the reviewer of that chunk added eight more. A
count written by hand is wrong the moment anyone does the work the count is supposed to describe,
and a stale figure carrying a fresh date reads as freshly verified — which is worse than no figure
at all. Anything this page can only keep true by hand belongs in a command instead.

A caveat about the assertion count wherever you read it, because it flatters: most of those
assertions come from parameter sweeps — one test case running thousands of seeds — so the count says
more about how many scenarios were swept than about how many independent things are checked. **The
measure we actually trust is mutation testing** (below): break the code deliberately and see whether
a test notices. A test that cannot fail when the code is wrong is worse than no test, because it
reads as coverage. Every chunk in this project has found at least one.

What the suite includes: closed-loop
motion on three drivetrains graded against ground truth the estimator cannot see; a survival
matrix of simulated hardware pathologies (every one must degrade to a fault code, never a
crash or a NaN pose); byte-pinned diagnostic output; and a measured heading-accuracy test
(worst end-of-60-seconds error 0.912° across 10 seeded boots under full sensor hostility,
against a 1.0° requirement — with the caveat that the drift magnitudes are provisional until
measured on real sensors). Load-bearing logic is mutation-tested: break the code deliberately,
watch the test go red, restore it. CI additionally cross-compiles every library header for the
V5's Cortex-A9 at strict warning levels, so no host-only assumption can enter the tree
unnoticed. Since 2026-08-12 that is no longer only a compile-time claim: the package has been
observed booting on a real V5 brain and constructing its full object graph there. Since the
adapters landed, the suite also compiles and RUNS the hardware glue itself, against a
programmable stand-in for PROS — with the limit that a stand-in can only check the glue
against our beliefs about PROS, never the beliefs. A few bench sessions (2026-08-13 onward)
have since checked a handful of those beliefs against real devices and found them right.
*Steering* a robot remains unverified, and will be until the hardware-validation phase walks the rest of its
prepared bench checklist.

## Build and test

Prerequisites: `cmake` (≥ 3.20) and a C++20 host compiler for the tests;
`arm-none-eabi-g++` for the V5 package.

Host test suite (no robot, no PROS — this is where the library actually gets verified):

```sh
cmake -S test -B build/test
cmake --build build/test -j
./build/test/shulib_tests
```

The tail of the output looks like this — the counts grow as the library does, so match the
*shape*, not the numbers:

```text
[doctest] test cases:    ... |    ... passed | 0 failed | 3 skipped
[doctest] assertions: ...... | ...... passed | 0 failed |
[doctest] Status: SUCCESS!
```

What matters is **0 failed** and **3 skipped**. The 3 skips are deliberate placeholders for
on-robot numbers — two accuracy targets that need a control loop closed on a field, and one GPS
axis oracle that needs a GPS in front of a field strip — and they stay skipped until hardware
validation measures them.

The V5 package (compiles and links the real ARM binary; uploading needs a V5 brain and
[pros-cli](https://pros.cs.purdue.edu/v5/getting-started/)). The default build is the bench
tester for the measured bench robot; `make ROBOT=tank PROGRAM=…` builds robot two's programs, and
`make ROBOT=… PROGRAM=tune` "shulib Tune" (slot 7) for any robot with mechanisms.
Remember that no program here has steered a robot yet:

```sh
make            # produces bin/hot.package.bin + bin/cold.package.bin
pros upload     # only with a V5 connected
```

## What using it looks like

An autonomous routine is plain, sequential code against the `Chassis` facade. Each verb blocks
until the robot settles — or reports why it stopped — and cannot hang:

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
    chassis.moveTo(math::Pose2d{-24_in, 0_in, 45_deg}, {.timeout = 3_s});

    // A slow, precise approach: per-call options cap this leg's speed.
    chassis.moveTo(math::Pose2d{-12_in, 12_in, 45_deg},
                   {.timeout = 2_s, .maxLinearSpeed = units::Velocity{20.0}});

    // Sideways to the goal, actively holding the current heading.
    chassis.strafeTo(-12_in, 24_in, {.timeout = 2_s});

    // Face the corner — always the short way around.
    const control::ExitReason last = chassis.turnTo(135_deg, {.timeout = 1.5_s});
    return last;  // Settled, TimedOut, or Cancelled — never a lie, never a hang
}
```

Units are typed (`-48_in`, `90_deg`, `3_s` — a bare `double` will not compile where a length,
angle, or duration belongs), every motion carries its own timeout, and if a fault fires
mid-motion (say, the odometry catches a wheel lying) the motion aborts into a safe state and the
exit reason says so.

Wiring a `Chassis` is ordinary C++ too, and there are two ways to do it. The on-ramp is the
builder: describe the robot as a `config::RobotConfig`, and `robot::Robot` builds the whole graph
from it over a device provider — `robot::validate()` names the port or the field that is wrong
before anything is constructed. `src/programs/drive_program.cpp` ("shulib Drive") is the worked example
on a real robot: robot two's config comes from the calibration record on its SD card, and
`hal::pros::ProsDeviceProvider` supplies the real adapters. (A typed port table is the fallback in
general, but robot two's cannot describe its drivetrain — its track width and gearing are unset —
so without a record "shulib Drive" refuses to build.) The longhand
— construct a kinematics preset, the hardware interfaces, the localizer, and hand them over — is
always available (Tier 3), and no config file or code generator is ever required: the test suite
builds the same stack by hand in one place against the host test plant
(`test/chassis_facade_test.cpp`, the "standalone" case). No program in `src/` wires a graph by hand
any more: which program a build is, and which robot's tables it carries, are chosen together by one
`#if` chain in `src/build_selection.hpp` (the only place the `make ROBOT=… PROGRAM=…` switches are
read), `src/main.cpp` only hands the PROS entry points to that program, and the one other robot
described there — an X-drive whose port map is labelled as the invention it is — is a
`constexpr RobotConfig` (`src/robots/xdrive_config.hpp`) built the same way as robot two's.

## How the tree is laid out

```text
include/shulib/     the library — pure C++20, PROS-free by CI guard
  units/ math/      typed quantities (inches/radians/seconds), Pose2d, frames, wrapped angles
  spec/             frozen accuracy targets
  hal/              the hardware interfaces (motor, motor groups, IMU, GPS, clock, distance,
                    optical, digital in/out, controller, mechanism, device provider, …) + fakes
    pros/           the real V5 adapters over the PROS SDK — the ONE exempted path (see rule 1)
  calibration/      the drivetrain self-calibrator, its screen model and the painter that draws
                    it, its SD-card record and its safe save and load (a verified backup copy),
                    the boot check, the front check, and the standard Calibrate program itself
                    (a team states its facts; its robot shell is hal/pros/calibrate.hpp)
  config/           a robot and a route described as DATA (RobotConfig, Route)
  robot/            robot::Robot — the whole library graph built from a RobotConfig (its
                    mechanisms' motors, lines and sensor roles built by MechanismParts)
  mechanism/        a mechanism declared once (what it is: kind, gearing, homing, presets)
                    and wired separately (where it is plugged in) — each mistake refused at
                    build by name; the route ids derived from the declarations; the
                    mechanism SERVICE that runs every one each tick (power, pneumatic,
                    position — arms, lifts — and speed — flywheels; host-only so far); the
                    driver file's controls (presets on buttons) and the direction check's
                    judgement (which Bench Tests' station 13 MECHANISMS runs on the robot —
                    built, not yet run on one)
  runner/           runs a Route: the path runner, the command-id registry, the manifest,
                    and every mechanism's route ids bound in one call
  teleop/           driver control: stick mapping, input shaping, drive modes, field-centric,
                    mechanism buttons (and the mechanism service's driver controls)
  kinematics/       drivetrain math: X-drive, tank, H-drive behind one contract
  control/          PID, feedforward, motion profiles, settling, watchdogs
  localization/     odometry, tracking wheels, the fused localizer + correction seam
  motion/           motion primitives, the one-active-motion scheduler, stall cross-check
  manipulation/     bounded mechanism operations (run-until-confirmed, actuate-and-confirm)
  sequence/         the end-of-run guard — cuts and latches motion on a hard schedule, or on an
                    abandon (a mid-auton disable)
  competition/      the competition program's library half: the auton registry, the selector
                    model, the auton runner (start-pose re-seat + guard), the acceptance run
  ui/               the generic brain-screen panel pieces (budgets, hit boxes, the touch edge)
  core/             the precondition policy (one place decides what a broken contract does)
  chassis/          the public facade (Chassis) + the composition root (RobotContext)
  diag/             structured records, sinks, fault latch, health/loop monitors
  sim/              the host plant + hostile sensor models (tests only — see rule 2)
src/                the robot programs, one folder per job (every include written from src/:
                    #include "robots/chassis_table.hpp"):
  main.cpp          the PROS entry point — it hands each entry point to the build's program
  build_selection.hpp  the one place a build's switches are read — it picks the program and
                    the robot together
  robots/           what each robot IS: the per-robot tables (every robot's ports; each robot's
                    mechanisms, declared in the mechanism core's words, their numbers from its
                    facts file), the invented X-drive (a
                    RobotConfig constant), and the standard Calibrate's facts, derived from the
                    tables (calibrate_facts.hpp)
  team/             the team's own code — where students edit: each robot's driver file (which
                    button does what, the feel; shulib Drive runs it on its own, and the
                    competition program's driver control runs it), each robot's autons
                    (autons.cpp, robot_one_autons.cpp — one list each, both auton programs
                    read it; the host suite compiles them and runs every auton in them) and
                    the skills route both robots share (skills_route.cpp)
  programs/         one program each: the bench tester, shulib Drive (the robot's driver file on
                    robot::Robot, with its MAIN / DETAILS / MORE pages), shulib Calibrate, the
                    X-drive's program, shulib Auton Tester (runs one of the team's autons on the
                    bench from the brain's screen, through the competition program's own call —
                    guide chapter 16), shulib Competition (THE program for a match: the auton
                    chosen on the brain's menu runs in autonomous() on the program's own task,
                    then driver control runs the team's driver file — guide chapter 17) and
                    shulib Tune (slot 7: live tuning of the mechanisms on the brain screen, for
                    any robot with mechanisms — guide chapter 18); this shulib Drive, the Auton
                    Tester, Competition and Tune have not run on a robot yet, and no X-drive
                    robot exists
  common/           what the programs share: the boot source, the state screens, the screen
                    text path and the painters (the selector, the refusal screen), the
                    fault-latch hook, every driver file's plumbing (driver_file.hpp) and the
                    driver-control heading, the driver-control screen's words (the drive
                    protections' row, the controller's lines, the mechanisms' boot lines and
                    the mechanism row), the log copy's and the dropped autons' boot lines, and an
                    auton run's boot chain proof and program-error stop
test/               the host suite (doctest) against plant + hostile sims + a PROS shim
                    (test/pros_shim/ — the programmable SDK stand-in the adapter tests run on;
                    its headers refuse to compile outside the host test build, on purpose)
docs/               documentation (roadmap, architecture, hardware assumptions)
firmware/, include/pros|liblvgl, Makefile, common.mk    vendored PROS kernel + V5 build
```

Two structural rules, both enforced by CI grep-guards, keep the layers apart:

1. **The library never includes PROS** — except `include/shulib/hal/pros/`, the adapter
   directory built to, which the guard exempts by exact path and nothing else. Everything else
   under `include/shulib/` compiles on any C++20 compiler — which is exactly what makes a host
   suite of this size possible. `src/` — the programs, outside the library tree — is the other
   deliberate PROS toucher.
2. **The core never includes the simulator.** Code under test can only see sensor interfaces,
   never the simulated ground truth — so a test can grade the estimator against a truth the
   estimator provably cannot peek at.

## Where to go next

- **[`docs/guide/`](docs/guide/README.md) — the user guide.** Start here if you're new: what
  the autonomous problem is, every concept from pose to PID in plain language, a full
  first-routine tutorial (every code example compiles and runs in the test suite), the API as
  prose, and how to read the diagnostics line by line. Written for a reader with no robotics
  background.
- **[`docs/cookbook/`](docs/cookbook/README.md) — the recipe cookbook.** Read out of order, mid-task:
  complete answers to "how do I write the routine I am writing right now" — a multi-goal side run,
  a bail-out when a grab fails, an alliance-partner wait, a tank routine, mixing tiers. Every recipe
  is compiled and run against the simulator on every build.
- **[`docs/api/`](docs/api/README.md) — the API reference.** Generated from the headers, so it cannot
  disagree with the code: one page per shipped header, plus an
  [A–Z index](docs/api/all-entities.md) of every public type, member, nested type, free function,
  constant and type alias. A public entity that ships undocumented fails the build, by name and
  line. If you are writing a routine you need two of those pages —
  [`Chassis`](docs/api/chassis.md) and [`Routine`](docs/api/routine.md); the rest is the machinery
  underneath.
- [`docs/roadmap.md`](docs/roadmap.md) — everything remaining, by milestone, with a
  "you are here" (including what is deliberately *not* claimed yet).
- [`docs/shulib-v2-master-plan.md`](docs/shulib-v2-master-plan.md) — the architecture and the
  reasoning behind every locked decision.
- [`docs/hardware-assumptions.md`](docs/hardware-assumptions.md) — every hardware claim this
  build rests on, which of them the bench has settled, and the measurement that settles each of
  the rest. (The register states its own count; this page deliberately does not carry one.)
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
2. **Claim only what you can show.** Something is done when there's evidence it's done — a passing test,
   a real measurement. Half-finished stays half-finished. Most of this repo is only useful because
   that rule held.

## License

shulib is released under the [MIT License](LICENSE) — use it, modify it, ship it.

---

<p align="center">
  <b>Built by Seton Hall University Robotics</b><br>
  <i>For the next generation of builders</i>
</p>
