# 13 — Extending the library

> **Covers:** how the layers fit together, the testing discipline and why it's strict, and the
> two most likely extensions — adding a drivetrain and adding a motion.
> **Read this if:** you're comfortable using the library and want to change or grow it.
> **Assumes:** all previous chapters, and working C++ (classes, virtual functions).

## The layer map

Everything in `include/shulib/` sits in a deliberate stack — each layer only knows about the
ones below it:

```text
chassis/        the facade (Chassis) — what routines talk to            [Ch. 10]
motion/         motion primitives + the one-at-a-time scheduler         [Ch. 5]
control/        PID, feedforward, settling, watchdogs                   [Ch. 5]
localization/   odometry, fusion, the Localizer                         [Ch. 3]
kinematics/     drivetrain geometry (X, tank, H)                        [Ch. 4]
diag/           records, fault latch, monitors, formatters              [Ch. 11]
hal/            the 10 hardware interfaces (+ in-memory fakes)
units/ math/    typed quantities, Angle, Pose2d, the frame transforms   [Ch. 2]
sim/            the simulated robot (tests only!)
```

Two structural rules are enforced by CI (automated checks that run on every commit — see
`.github/workflows/ci.yml`), and understanding *why* tells you most of what matters here:

1. **The library never includes PROS** (the V5 runtime). Everything under `include/shulib/`
   compiles on any C++20 compiler — which is precisely what makes the massive host-side test
   suite possible. Exactly one file in the repo sees both worlds: `src/main.cpp`. If your
   change needs something from PROS inside the library, the design is wrong — the need belongs
   behind a `hal/` interface.
2. **The core never includes the simulator.** Code under test can only see sensor interfaces —
   never the sim's ground truth — so a test can grade the estimator against a truth the
   estimator *provably cannot peek at*. Don't "just include" something from `sim/` in library
   code; the guard will fail your build, on purpose.

A third, softer convention: **headers open with *why they exist***, not just what they do. When
you're about to change a file, read its header comment first — the reasoning that isn't
recoverable from the code is written there, including decisions that were considered and
rejected. When you *write* one, do the same; it's the house style, and it's load-bearing
(several chapters of this guide are distilled from those headers).

## The testing discipline, and why it's strict

The short version of the rules you'll be held to (the long version:
[`test/README.md`](../../test/README.md)):

- **Tests must try to break the code, not confirm it works.** Every test should be able to name
  the bug it would catch. Sweeps and invariants beat hand-picked happy values; NaN, wrap-around,
  boundary, and hostile-sensor cases are the point, not the garnish.
- **Load-bearing logic gets mutation checks**: deliberately break the implementation, run the
  suite, *watch it go red*, restore it. A test that stays green while the code is wrong is
  theater — and this project has caught real green holes this way (cases where every existing
  test stayed green against a genuinely broken behavior; some of the oddest-looking tests in
  the suite exist to plug exactly those).
- **Evidence, not vibes.** A thing is done when a passing test proves it. Status claims cite
  the test.

Why so strict? Because there is no robot. Off-robot verification is the *only* verification
this project has, so its standard has to be brutal or it's nothing. This is also why the
simulated sensors are hostile (they lie during boot, freeze, drop out, sag) — the suite's job
is to be a worse day than the field will be, within the limits of what simulation can honestly
claim.

**The workflow for any change:** build and run the suite before you start (Chapter 7 — it must
be green); make the change with its new attacking tests; run everything again; run the two CI
guard checks and the ARM cross-compile if you touched library headers (the commands are in
`.github/workflows/ci.yml`). If your change alters behavior this guide documents, update the
guide — the [maintenance README](README.md) lists what to check.

## Extension 1: adding a drivetrain

Drivetrains are data, not code ([Chapter 4](04-drivetrains.md)) — the library's
`MatrixKinematics` takes a list of wheels (position, rolling direction, gearing) and derives
everything else. Look at how the presets are built in
[`x_drive.hpp`](../../include/shulib/kinematics/x_drive.hpp) and
[`h_drive.hpp`](../../include/shulib/kinematics/h_drive.hpp): each is a short function
assembling wheel descriptions and a strafe-authority number. A new symmetric drivetrain
(mecanum, a different X geometry) is a new preset function in the same shape, and every motion,
verb, and diagnostic works with it immediately — that's the point of the kinematics contract
(frozen as F5 in the [Freeze Register](../roadmap.md#freeze-register)).

The part that is genuinely hard, learned at cost during the H-drive work: **closed-loop tests
cannot catch a wrong geometry table.** Plant and controller share the kinematics, so a
sign-flipped wheel direction cancels itself out — the robot converges beautifully in simulation
and would drive garbage in reality. Every preset therefore needs **from-scratch oracle tests**:
hand-derived rigid-body checks ("all wheels forward at speed v ⇒ chassis moves forward at v,
zero rotation") computed independently of the code under test. Write the oracles *before*
trusting anything the closed loop tells you, and add a mutation check that flips a sign in your
table and watches the oracle go red. The existing kinematics tests
(`test/x_drive_test.cpp`, `test/h_drive_test.cpp`, `test/kinematics_contract_test.cpp`) are the
worked examples, and the contract test suite runs against *any* `IKinematics` — point it at
yours.

## Extension 2: adding a motion

A motion (a new primitive — say, a drive-until-distance-sensor-triggers) is a class
implementing `IMotion` ([`include/shulib/motion/motion.hpp`](../../include/shulib/motion/motion.hpp)
— read its header contract first): essentially `start()`, a per-tick `tick()` returning
`Running`/`Settled`/`TimedOut`, and `cancel()` into the safe state. Study
[`turn_to.hpp`](../../include/shulib/motion/turn_to.hpp) as the smallest complete example —
note everything it handles beyond the obvious: the wait-for-live-estimate boot window, the
watchdog, settle logic, emitting its per-tick record, leaving the motors safe on every exit
path.

The non-negotiables your motion inherits from the contract: **it must be watchdog-bounded** (no
path may run forever), **cancel() must land in the safe state synchronously**, and **post-exit
ticks must be harmless no-ops**. The suite has generic tests for these properties; wire your
motion into them rather than hand-rolling.

You can *use* a custom motion without touching the library at all, via the Tier-3 seam
([Chapter 10](10-the-api.md)): build it from `chassis.deps()` (so its diagnostic records carry
a command id, like the built-in verbs) and run it through `chassis.scheduler().async(...)` /
`waitUntilSettled()`. Promote it into the library proper when it's earned its tests.

## Extension 3 and beyond

The seams for bigger work already exist, with their owners named on the
[roadmap](../roadmap.md): position correctors (GPS/vision) plug into the localizer's correction
seam; real-hardware adapters implement the `hal/` interfaces (phase R1); new telemetry sinks
implement `ITelemetrySink`. Before starting anything sized like these, read the roadmap entry
and the relevant header — and talk to the team. Big pieces here get designed in writing first;
that's how the library got this far without a robot.

---

*Next: [Chapter 14 — What it can't do yet](14-what-it-cannot-do-yet.md)*
