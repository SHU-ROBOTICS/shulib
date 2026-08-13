# 14 — What it can't do yet

> **Covers:** the honest list — everything this library cannot do today, with where each gap is
> tracked and what closes it.
> **Read this if:** you're about to rely on the library for something, or you're evaluating
> the project from outside.
> **Assumes:** nothing beyond [Chapter 1](01-what-is-this.md). This chapter is deliberately
> readable standalone.

This project's culture takes honest status seriously — "we under-claim before we over-claim"
is written into the [roadmap's rules](../roadmap.md#how-to-read-this-roadmap), and this chapter
is that rule applied to itself. The links matter more than the prose: statuses change, and the
linked documents are maintained as the truth. Where this page and the roadmap disagree, the
roadmap is right.

## It has never run on a robot

The headline limitation, stated as many times as it takes:

- **No shulib code has ever executed on a physical robot.** All verification is host-side,
  against a simulated plant and deliberately hostile simulated sensors
  ([Chapter 7](07-getting-set-up.md) explains the approach;
  the [README](../../README.md#what-it-is-not-yet) states it third-heading-from-the-top).
- **The hardware adapters don't exist.** The library defines the interfaces real V5 motors and
  sensors will plug into; the implementations are phase R1 on the [roadmap](../roadmap.md).
  `make` produces a real V5 package that boots, prints a banner — and drives nothing
  (`src/main.cpp`'s seams are marked `TODO(R1)`).
- **Every physical constant is a labeled guess.** Control gains, settle tolerances, sensor
  noise levels, drivetrain geometry, fault thresholds — all provisional, and cataloged as
  falsifiable claims in the [Hardware Assumptions Register](../hardware-assumptions.md)
  (57 of them as of 2026-08-10, zero settled; each names its blast radius if wrong and the
  measurement that settles it). Two examples to convey the range: the H-drive's sideways
  authority (0.35) is pure invention that could plausibly be anywhere from ~0.15 to ~0.8
  (HA-54); and the entire heading-accuracy story leans on an assumed IMU drift bound (HA-20)
  that is community folklore until measured.
- **Consequently, the accuracy numbers are simulation numbers.** The measured sub-degree
  heading performance ([README](../../README.md#how-verified-is-it-honestly)) is real
  measurement — of a simulated robot under modeled hostility. It is evidence of sound logic,
  not a promise about the field.

The team's phrase for the plan here is worth quoting: *the first hardware contact is a prepared
sequence, not a surprise* — phases R1–R6 on the roadmap are exactly that sequence (adapters,
day-one validation, sensor characterization, gain measurement, sim back-fit).

## No absolute position correction yet

The estimate is pure dead reckoning today (odometry + IMU,
[Chapter 3](03-knowing-where-you-are.md)). The correction *seam* exists and is tested — but the
GPS corrector and the vision/AprilTag corrector that would plug into it are planned work
(roadmap milestone M3). Until at least one exists, drift over a 60-second run is uncorrected,
and the team's < 1° end-of-run heading requirement is, by the master plan's own analysis,
**not reliably achievable on a real robot** — absolute yaw correction is load-bearing for the
spec, not a nice-to-have.

## The mechanism seam is a placeholder

Both API tiers are now frozen: the `Chassis` API ([Chapter 10](10-the-api.md)) at D2 and the
recipe layer ([Chapter 9](09-the-recipe-api.md)) at D3, Freeze Register rows F6 and F10, both
2026-08-12. Signatures and documented behavior change only with a major version bump and a
migration note, and compile-time pins fail the build if one drifts — routines written today
will not need rewriting. One piece is deliberately left open:

- **`then()`** ([Chapter 9](09-the-recipe-api.md)) is the seam mechanisms will plug into, and
  **mechanisms do not exist yet** (see the next section). Its accepted return types and its
  step-name default were chosen before there was anything real to plug in, so freezing them
  would have committed the library to a guess about code nobody has written. `then()` works and
  is tested; treat its exact shape as provisional.

The [Freeze Register](../roadmap.md#freeze-register) remains the authority on what is and isn't
stable (the coordinate conventions, units, accuracy targets, hardware interfaces, kinematics
contract, and now both API tiers *are* locked).

## No easier tiers yet

The project's [accessibility model](../shulib-v2-master-plan.md#17-accessibility--progressive-disclosure-for-teams-that-cant-code-yet)
promises four tiers of use. Today Tiers 2 and 3 exist — the recipe layer
([Chapter 9](09-the-recipe-api.md)) and the full C++ API — but the easier tiers don't:

- **No zero-code authoring** (Tiers 0–1: build the robot and drag waypoints in the VexBuilder
  app, run the saved file with no C++). The `.vexbot` file formats and the path-runner are
  roadmap milestone M5, coordinated with the separate VexBuilder project.
- **No mechanisms for recipes to command** — `then()` in a recipe is a labeled placeholder
  seam until the mechanism layer exists (roadmap items F1/F3); today it runs only code you
  write yourself.
- **No recipe cookbook or generated API reference site yet** (D3).

## Motion-quality boundaries

Deliberate v1 boundaries, documented where they bind (each is on the roadmap or the
[master plan's frontier list](../shulib-v2-master-plan.md#15-the-one-stop-shop-capability-catalog-past--present--future)):

- **Stop-and-settle only**: trajectories settle at every waypoint
  ([Chapter 10](10-the-api.md)); no blended, non-stop waypoint traversal, no curved profiled
  segments. Measured cost: about 1.2 s per motion.
- **No motion profiles on the main verbs yet** (trapezoidal velocity planning exists in
  `control/` but isn't wired into `moveTo`).
- **`drive()` is a primitive, not a driver-control product** — no joystick shaping, slew-rate
  limits, or driver-preference curves yet.
- **No mechanism/scoring layer** — lifts, intakes, pneumatics, match-load sequences (roadmap
  M4). shulib moves the chassis; mechanisms are hand-rolled today.
- **Braking physics are unverified**: brake mode is commanded correctly, but what a real V5
  brake does to a moving robot is a registered assumption (HA-53). Plan conservatively around
  stopping distances.

## Diagnostics boundaries

The terminal transcript ([Chapter 11](11-reading-the-diagnostics.md)) is real and byte-pinned.
Not built yet, per the [diagnostics plan](../diagnostics-plan.md): the SD-card blackbox (logs
without a laptop), the live telemetry wire to VexBuilder, record/replay, and the on-brain HUD.
Today's transcript needs a USB tether (or the simulator).

## And the meta-limitation

This guide itself has never been used by its intended reader. It was written alongside the
library, checked against the code (every example compiles and runs in
[`test/guide_examples_test.cpp`](../../test/guide_examples_test.cpp)), but no first-year has yet
learned from it. When you — the actual new member reading this — find the sentence that stopped
you, that's a guide bug: report it ([maintenance README](README.md)).

---

*Next: [Chapter 15 — Glossary](15-glossary.md)*
