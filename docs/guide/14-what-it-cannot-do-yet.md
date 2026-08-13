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

## It has never driven a robot

The headline limitation, stated as many times as it takes:

- **No shulib code has ever controlled a motor or read a real sensor.** All verification is
  host-side, against a simulated plant and deliberately hostile simulated sensors
  ([Chapter 7](07-getting-set-up.md) explains the approach;
  the [README](../../README.md#what-it-is-not-yet) states it third-heading-from-the-top).
- **It has booted on a brain, and that proves less than it sounds like.** On 2026-08-12 the
  package was uploaded to a V5 brain, where it started, built its entire object graph, and
  printed its diagnostics banner over USB. Worth knowing, because it means the build path works
  and nothing in the library depends on being on a laptop. But every motor and sensor in that
  run was a fake, and the robot said so itself in the second line it printed. Booting is not
  driving, and nothing about motion or accuracy was tested that day.
- **The hardware adapters don't exist.** The library defines the interfaces real V5 motors and
  sensors will plug into; the implementations are phase R1 on the [roadmap](../roadmap.md).
  `make` produces a real V5 package that boots, prints a banner — and drives nothing
  (`src/main.cpp`'s seams are marked `TODO(R1)`).
- **Every physical constant is a labeled guess.** Control gains, settle tolerances, sensor
  noise levels, drivetrain geometry, fault thresholds — all provisional, and cataloged as
  falsifiable claims in the [Hardware Assumptions Register](../hardware-assumptions.md)
  (every one still unsettled; each names its blast radius if wrong and the measurement that
  settles it — the register itself is the count, so this page does not carry a number that
  would go stale). Two examples to convey the range: the H-drive's sideways
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

## Position correction exists; yaw correction does not

**This limitation has partly fallen.** The GPS corrector is finished and tested
([Chapter 3](03-knowing-where-you-are.md)): with the strip in view, position drift is bounded
rather than accumulating for the whole run. What is still missing is the half that the accuracy
spec actually turns on.

- **No absolute HEADING correction.** Heading comes from the IMU alone and its drift is
  uncorrected. The GPS reports a heading; the library deliberately does not use it, because it
  is worse than the IMU's. The team's < 1° end-of-run heading requirement therefore remains, by
  the master plan's own analysis, **not reliably achievable on a real robot** — absolute yaw
  correction is load-bearing for that spec, and it needs the vision/AprilTag corrector (roadmap
  milestone M3), which is planned work.
- **No vision/AprilTag corrector**, so there is exactly one absolute position source. If the
  strip is not visible there is no second opinion.
- **No Kalman filter.** Fusion is a bounded-nudge complementary filter. It has no covariance, so
  it cannot weigh two disagreeing sources against each other properly, and the diagnostics field
  reserved for a Mahalanobis distance is honestly left empty rather than filled with a
  look-alike.
- **Correction does not rescue a badly wrong estimate.** Fixes more than a foot from the
  estimate are rejected outright. If the estimate is grossly wrong, the corrector will not pull
  it back — bounded drift is a weaker promise than recovery, and the difference matters after a
  hard collision.
- **None of it has seen a GPS.** The corrector's behaviour was proven against a simulated sensor
  built from written-down guesses about noise, timing and failure modes
  ([Hardware Assumptions Register](../hardware-assumptions.md)). How much a real GPS helps
  depends on how its accuracy compares to real dead-reckoning drift, and neither has been
  measured.
- **Driving Skills has no strip at all**, so in that event the estimate is exactly as
  dead-reckoned as it was before this feature existed.

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
