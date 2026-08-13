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

## Correction now exists for position *and* heading — and what it is worth is unmeasured

Two correctors are finished and tested ([Chapter 3](03-knowing-where-you-are.md)): the GPS
corrector bounds position drift when the strip is in view, and the AprilTag corrector adds
absolute **heading** — a tag whose field position is known tells the robot which way it is
actually pointing, which no other source in this library can do.

This section used to say, in plain words, that the team's `< 1°` end-of-run heading requirement
was **not reliably achievable on a real robot** because nothing in the library could correct
heading at all. That sentence has to change, and **the honest change is narrow**. Here is what
was measured, what it was measured on, and what is still unknown — in that order, because the
order is the point.

**What was measured.** A robot whose IMU is 4° wrong recovers to within 0.5° in about three
seconds of tag sightings, and to about a ten-thousandth of a degree in fifteen. It moved *toward*
the truth on every one of 1500 consecutive ticks and never past it. Separately, with a 12° error
and a maximally confident tag, **no tick ever changed the robot's idea of its heading by more
than a tenth of a degree** — the documented per-tick bound — across 2000 consecutive ticks, and
the same bound holds when re-read from a decoded blackbox file rather than from live state. Yaw
is nudged, never snapped, and the correction accumulates instead of evaporating.

**What it was measured *on*.** Simulated truth. A simulated camera. A tag map invented for the
test. Noise, latency, detection confidence and range magnitudes that were **made up** and are
catalogued as guesses in the [Hardware Assumptions Register](../hardware-assumptions.md)
(HA-68…HA-82). Every one of those numbers could be wrong by a factor of several.

**What remains unmeasured, and is load-bearing for the claim:**

- **No camera has ever been pointed at a tag by this project.** The corners-to-pose maths is
  proven against synthetic images computed from geometry; it has never seen a lens, a shutter, a
  gymnasium light, or a printed tag.
- **shulib ships no tag map, deliberately.** Where the tags are on the field is *your* input, and
  nobody here has a citable table of tag positions. A map that is two inches off produces a
  corrector that is *confidently* two inches wrong — and unlike noise, that error does not
  average out (HA-68).
- **The detector's corner ordering is unverified, and one way of getting it wrong is silent.** A
  reversed corner winding mirrors the tag, puts the recovered heading 180° out, and leaves the
  solver's own error check reading zero. Nothing in software can detect it; only a physical tag
  can (HA-69).
- **Real IMU drift is unknown.** The whole reason yaw correction is load-bearing is an assumed
  drift of about 1°/min, which is community folklore until somebody measures our units (HA-20).
- **How accurate a real tag fix is, at what range, is unknown.** The library handles this with a
  trusted-range band — a blunt instrument, chosen because the alternative was inventing a second
  noise model (HA-73).

**So, precisely:** this page does **not** claim the `< 1°` requirement is met, and does not claim
it is likely to be met. It claims exactly one thing — *the specific reason this requirement was
previously listed as unachievable, namely that nothing in the library could correct heading at
all, no longer applies.* Everything else about the claim is unchanged and unmeasured. If that
reads as a smaller change than the feature sounds like, that is the honest size of it.

**Still missing, and unrelated to any of the above:**

- **The heading correction has its own version of that limit.** A heading disagreement larger
  than 15° is rejected rather than folded, on the reasoning that it is far more likely to be a
  misread tag than real drift. If a real IMU ever drifts further than that, the correction will
  refuse to fix it (HA-80).
- **None of it has seen real hardware.** Both correctors were proven against simulated sensors
  built from written-down guesses about noise, timing and failure modes. How much either helps
  depends on how its accuracy compares to real dead-reckoning drift, and none of those numbers
  has been measured.
- **Driving Skills has no GPS strip**, so in that event the GPS corrector reports "no fix" for
  the whole run. The **tag** corrector does not depend on the strip — but it does depend on you
  having a camera, a tag map, and a task polling it, none of which exist yet either.

## A Kalman filter now exists — and it is not the default

This page used to carry two limitations together: *no Kalman filter*, and *two disagreeing
correctors are bounded rather than resolved*. Both have changed, and as with the heading section
above the honest change is narrower than it sounds. Same order: what was measured, what it was
measured on, what is still unknown.

**What was measured.** A 5-state extended Kalman filter now exists behind the same fusion seam
([Chapter 3](03-knowing-where-you-are.md)), and three things it can do were measured directly.
*One:* handed two disagreeing fixes with stated accuracies of 1 inch and 5 inches, it settles on
the inverse-variance weighted point between them — the number a person computes from the two σ
values on paper, matched to better than a fifth of a percent. It does not pick one, does not
average blindly, and is not swayed by which source claims to be more confident. *Two:* an
estimate wounded by 20 inches — further than the default filter's fixed gate will ever accept —
returns to within 2 inches on all six seeds tested, while the default filter's estimate is still
20 inches out at the end of every one of them. *Three:* 2000 consecutive confident lies, each 50
inches from the truth, moved the estimate by **zero inches**, and no tick under any of these
conditions moved the estimate further than the documented per-tick bound.

**What it was measured *on*.** The same simulated robot everything else in this library was
measured on, and — new here — **noise parameters that are entirely invented**. A Kalman filter is
only as good as its description of how wrong its sensors are, and every one of those numbers
(HA-83…HA-91 in the [Hardware Assumptions Register](../hardware-assumptions.md)) is a guess
written down by somebody who has not measured a robot. The filter's *structure* is what was
proven here; its *numbers* are fitted on hardware that does not exist yet.

**What remains unmeasured, and what did not change:**

- **The Kalman tier is not the shipped default, and the measurement is why.** Over eight seeded
  60-second runs, the two filters finish within about an eighth of an inch of each other and the
  **simpler one is slightly ahead**. In this simulation dead-reckoning is already sub-inch over a
  minute, so the modelled GPS is noisier than the drift it corrects, and against a sensor like
  that the best move is mostly to ignore it — which a blunt fixed gain does slightly harder. The
  Kalman filter is not there for accuracy on a clean run; it is there for recovery, arbitration,
  and knowing how wrong it might be.
- **Recovery from a large displacement needs both gates to agree, and only one of them was
  fixed.** The fusion layer's fixed 12-inch ceiling is gone under the Kalman tier. Each corrector
  *also* has its own gate, which E4 did not touch — so a real recovery from a 20-inch error
  depends on the corrector's own widening rule as well, and that rule's constant (HA-67) is
  another guess.
- **The uncertainty the filter reports is only as honest as its noise model.** `covarianceTrace`
  is a real number computed from a real filter, and it is computed from invented inputs. Treat it
  as a *relative* signal — it grew, it shrank — rather than as a calibrated distance, until R4.
- **Nothing here has seen a robot.** Same as everything else on this page.

**So, precisely:** the library can now weigh two disagreeing sources against each other by their
stated accuracy, recover from a displacement that used to be permanent, and state its own
uncertainty as a number. It is **not** claimed to be more accurate than what it replaces on an
ordinary run, and on the evidence available it is very slightly less so.

## The mechanism layer exists — on a host, against fakes, with no season in it

Both API tiers are frozen: the `Chassis` API ([Chapter 10](10-the-api.md)) at D2 and the
recipe layer ([Chapter 9](09-the-recipe-api.md)) at D3, Freeze Register rows F6 and F10, both
2026-08-12. Signatures and documented behavior change only with a major version bump and a
migration note, and compile-time pins fail the build if one drifts — routines written today
will not need rewriting.

The mechanism seam that used to be a placeholder here is now real code
([Chapter 13](13-extending-the-library.md) is the how-to). What is proven, and against what:

- **Proven, on a host, against fakes:** motor groups and pneumatic lines can be commanded
  through the device seam; a bounded operation runs, is ticked by the chassis's own wait
  (including *while a motion drives*, with the motion's result proven bit-identical to a run
  with no mechanism at all), detects a jam from current + shaft speed, times out under an
  adversarial clock rather than hanging, cancels into a per-mechanism declared safe state,
  and hands `then()` a verdict where a completed-but-unconfirmed action can never read as
  success. All of it against in-memory fakes and hostile fakes that jam, stall and lie.
- **Deliberately absent:** any concrete mechanism. There is no `shulib::Intake` and never
  will be — mechanism sets change every season, so the library ships the grammar and your
  team writes the nouns (Chapter 13 says how, and why the safe state is declared per
  mechanism).
- **Not proven, stated plainly:** anything on hardware. No solenoid has fired, no motor has
  jammed for real, and whether `Hold` truly holds a *loaded* lift is a registered assumption
  (HA-92), not a fact. The stall thresholds an operation needs are required parameters
  precisely because no honest default exists before hardware characterization (phase R4).
  And the scoring verbs themselves — `intakeUntilCapture`, `liftToLevel`,
  `setQuadrantToggle` — remain future work (roadmap item F3) that needs both hardware and
  the build team's final mechanism decisions.

One related piece stays deliberately open: **`then()`** and the mechanism seam are still
**unfrozen** — the seam gets its second real consumer at F3 (the concrete scoring verbs),
and this project freezes surfaces after a second consumer has stressed them, not before.
The [Freeze Register](../roadmap.md#freeze-register) remains the authority on what is and
isn't stable (the coordinate conventions, units, accuracy targets, hardware interfaces,
kinematics contract, and both API tiers *are* locked; the mechanism seam is listed there as
explicitly not-yet-frozen).

## A run-scoped deadline now exists — and "guaranteed" is a narrower word than it sounds

The sequence layer's guard ([Chapter 6](06-how-things-fail.md)) is the library's
highest-stakes claim, so its boundary is stated here in full, exactly as the project's own
records word it:

> F2 proves a **scheduling property** — a stalled loop still ends with the end action,
> against the plant, with the clock driven to the limit. It **cannot** claim the timing
> margin is right on a real brain: real loop rate under load and PROS call latency are
> invented register entries until R4. **And nothing preempts pure user code that never calls
> into shulib** — there are no background tasks.

Unpacked, that is three separate limits:

- **Proven on the host plant, four ways:** a motion that never settles, a mechanism that
  never confirms, a wait whose condition never comes true, and a fault-abort cascade each
  end with the caller's end action performed, judged against the simulator's ground truth
  with the match limit driven by a clock the guard never touches. That is a real property
  of the *scheduling* — it is not a claim that a real robot's 6-second park fits a
  6-second runway. Until phase R4 measures real loop rates and call latencies, your lead
  time is your own engineering margin. Budget generously and measure at the field.
- **Nothing preempts your code.** There are no background tasks anywhere in this library —
  a deliberate, standing decision. The guard works by making every shulib call after the
  deadline finish quickly (cut) and stay finished (refused). Code that never lets a
  finished call end its loop — `while (true) { chassis.moveTo(goal); }` with no exit —
  keeps the CPU forever, and the end action runs only when that loop returns. The
  transcript will show the refusals; the guard cannot show you the way out. Write retry
  loops against `guard.expired()`.
- **Frozen waits pay their remainder.** `pause`/`waitFor`/`waitUntil` on the frozen
  surfaces cannot see the deadline ([Chapter 9](09-the-recipe-api.md) has the exact
  formula and the deadline-aware alternative).
- **"Zero travel after the deadline" is a simulator result, not a robot one.** The host
  measurement is exactly 0.0 inches, and it is real — but it holds because the guard
  refuses the next command *before* the world advances **and** because the simulated
  drivetrain is memoryless: it stops the instant the voltage does. A real robot has mass.
  The honest on-robot claim is **"no new commanded motion after the deadline"** — the
  coasting and braking distance already in the wheels is physics, and this plant cannot
  produce it. Budget your lead time for a robot that is still moving when the command
  stops.
  There is a second difference that matters more on a field than it does here: **on a real
  brain, time passes whether or not your code calls into shulib.** In the simulator the
  clock only advances when the pacer steps it, so the deadline is never "missed" while
  something else runs. On hardware, both the cut and the hard floor can only fire when your
  code next re-enters the library — which is the same limit as the bullet above, seen from
  the other side.

And its deliberate refusals, which are design, not gaps: no default match length, no
default lead time, no park pose, no field coordinate — a guard with a built-in notion of
"the endgame" would be one team's strategy frozen into everyone's library. Both instants
and the action are yours.

## No easier tiers yet

The project's [accessibility model](../shulib-v2-master-plan.md#17-accessibility--progressive-disclosure-for-teams-that-cant-code-yet)
promises four tiers of use. Today Tiers 2 and 3 exist — the recipe layer
([Chapter 9](09-the-recipe-api.md)) and the full C++ API — but the easier tiers don't:

- **No zero-code authoring** (Tiers 0–1: build the robot and drag waypoints in the VexBuilder
  app, run the saved file with no C++). The `.vexbot` file formats and the path-runner are
  roadmap milestone M5, coordinated with the separate VexBuilder project.
- **No ready-made scoring verbs for recipes to command** — the mechanism layer exists (see
  above) and `then()` runs its operations, but the mechanisms themselves are structs your
  team writes ([Chapter 13](13-extending-the-library.md)), and the season's concrete
  primitives (`intakeUntilCapture`, `liftToLevel`, …) are roadmap item F3.
- **No published API reference site.** The [cookbook](../cookbook/README.md) and the
  generated API reference (`docs/api/`) both exist and are build-checked — but nothing is
  hosted anywhere yet, so "the reference is live on the team website" remains open.
  *(This bullet used to say no cookbook existed at all; that was written before D3 shipped
  it and went stale — corrected at F1.)*

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
