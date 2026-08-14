# 13 — Extending the library

> **Covers:** how the layers fit together, the testing discipline and why it's strict, and four
> extensions — adding a drivetrain, adding a motion, building a mechanism, and porting to
> different hardware behind the adapter seam.
> **Read this if:** you're comfortable using the library and want to change or grow it.
> **Assumes:** all previous chapters, and working C++ (classes, virtual functions).

## The layer map

Everything in `include/shulib/` sits in a deliberate stack — each layer only knows about the
ones below it:

```text
chassis/        the facade (Chassis) — what routines talk to            [Ch. 10]
sequence/       the run guard — a whole-run deadline + guaranteed end   [Ch. 6]
motion/         motion primitives + the one-at-a-time scheduler         [Ch. 5]
manipulation/   bounded mechanism operations (spin-until, actuate)      [Ch. 9]
control/        PID, feedforward, settling, watchdogs                   [Ch. 5]
localization/   odometry, fusion, the Localizer                         [Ch. 3]
kinematics/     drivetrain geometry (X, tank, H)                        [Ch. 4]
diag/           records, fault latch, monitors, formatters              [Ch. 11]
hal/            the hardware interfaces — motors, sensors, the mechanism
                device seam, the controller, digital in/out, byte sinks
                (+ in-memory fakes for every one)
hal/pros/       the real-hardware adapters over the PROS SDK — the ONE
                place the library touches VEX's software
units/ math/    typed quantities, Angle, Pose2d, the frame transforms   [Ch. 2]
sim/            the simulated robot (tests only!)
```

Two structural rules are enforced by CI (automated checks that run on every commit — see
`.github/workflows/ci.yml`), and understanding *why* tells you most of what matters here:

1. **The library never includes PROS** (the V5 runtime) — except the adapter directory built
   to, `include/shulib/hal/pros/`, which the check exempts by exact path and nothing else.
   Everything else under `include/shulib/` compiles on any C++20 compiler — which is precisely
   what makes the massive host-side test suite possible (`src/main.cpp`, outside the library
   tree, is the other deliberate PROS toucher). If your change needs something from PROS
   anywhere else in the library, the design is wrong — the need belongs behind a `hal/`
   interface, with the PROS call living in a `hal/pros/` adapter.
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

Why so strict? Because off-robot verification is still the *only* verification this project has
for anything above the hardware seam — a robot has been on a bench, but no control loop has ever
closed on it — so the standard has to be brutal or it's nothing. This is also why the
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

## Extension 3: building a mechanism

An intake, a lift, a clamp — the library deliberately does not know any of them by name.
What it gives you is a grammar with two levels, and your mechanism is a small struct you
write from it. (Why no `shulib::Intake`? Because mechanism sets change every season, and a
library that hard-codes this year's robot is wrong next year — a failure mode this project
found fully formed in its own predecessor.)

**Level 1 — the devices** ([`hal/mechanism.hpp`](../../include/shulib/hal/mechanism.hpp)):
`MotorMechanism` is a group of motors on one shaft, commanded as one; `PneumaticMechanism`
is one air circuit behind one or more digital lines. Both are built from the same hardware
interfaces the rest of the library uses, so they run identically on fakes (tests), the
simulator, and — through the `hal/pros/` adapters — the robot. The one decision you MUST make
at construction is the **declared safe state**, and it is per-mechanism because no single
answer is safe for all of them:

- a **loaded lift** at zero volts, coasting, drops its stack — its safe state is `Hold`;
- a **jammed intake** told to Hold sits at stall current until the motor cooks — its safe
  state is `Coast` (or `Brake`);
- a **clamp**'s safe line state is a strategy fact — "stay closed at the buzzer and keep
  the goal" and "retract inside the expansion limit" are both legitimate declarations.

Everything that ever stops your mechanism — an operation finishing, a cancel, the run
guard's end-of-match cancel-all ([Chapter 6](06-how-things-fail.md)) — applies the state
*you declared*, so the decision is made once,
where you know the physics, instead of being re-made (wrongly) by generic code at the worst
moment. Whether `Hold` truly holds *your* loaded lift is a hardware question nobody can
answer until there is a robot; the assumptions register tracks it (HA-92).

On the robot, one construction fact matters enough to state here: **building the pneumatic
line adapter actuates the port** — the V5 drives a digital output the moment the object is
constructed, so `ProsDigitalOut` makes its initial state a *required* argument with no
default. Pass it the same value you declare as the mechanism's safe state and boot drives
the line straight to safe, once, with no glitch through the wrong state; mismatch them and
the cylinder physically moves at power-on (the [FAQ](../faq.md) entry "Why did my pneumatic
fire the moment the robot booted?" is this exact story). The confirm sensors have their
adapters too — `ProsDistance`, `ProsOptical`, and `ProsDigitalIn` for a limit switch or bumper,
which is the obvious confirmation for a clamp or a homed lift — and each carries a trap note
worth reading before your predicate thresholds on it. A distance sensor with nothing in view
reports a plausible-looking 393-inch reading, which the adapter maps to zero confidence, so
threshold `confidence()` first, always. And a digital input is screened for the same reason in
the other direction: an unscreened dead ADI port reads as **permanently pressed**, which on a
homing routine is a lift climbing into its own hard stop.

**Level 2 — the operations** ([`manipulation/mechanism_op.hpp`](../../include/shulib/manipulation/mechanism_op.hpp)
— read its header contract the way you'd read `motion.hpp`'s): a bounded, tickable,
cancellable action over a device. Two season-free shapes ship, and every scoring verb is one
of them with real sensors:

- `RunUntilConfirmed` — spin a motor mechanism until *your* confirmation says the task
  happened, with a jam detector (high current + stopped shaft) and a watchdog. Verdicts:
  `Succeeded`, `Stalled` (raises the `MECHANISM_STALLED` fault), `TimedOut` (no fault — a
  healthy intake that never saw a ring is strategy, not a robot problem), `Cancelled`.
- `ActuateAndConfirm` — fire a solenoid, wait out the physical actuation time, then require
  confirmation within a window. This is where `Unconfirmed` lives: the command completed,
  the mechanism is healthy, and the world says the thing did not happen. A solenoid has no
  feedback of its own, so confirmation **must** come from a separate sensor — current,
  distance, color — and *what confirms is a predicate you supply*, because "captured" means
  something different every season.

The rules an operation lives by are the motion layer's, mirrored: it never owns a loop (the
chassis's `waitUntil` ticks it — the Chapter 9 listing is the idiom, and it runs happily
*while a motion drives*); it can never hang (watchdog armed at `start()`, nothing disarms
it); `cancel()` is idempotent, lands the device in its declared safe state synchronously,
and never rewrites a finished verdict. One operation per mechanism at a time is enforced
structurally — a second `start()` on a busy mechanism fails loudly instead of silently
double-driving it.

Three practical notes. First, keep a list of your mechanisms as `hal::IMechanism*` — the
run guard's cancel-all ([Chapter 6](06-how-things-fail.md)) takes exactly that list and
forces every one of them safe at the buzzer, which is the whole reason the base interface
exists. Second, if you write your OWN operation type (not the two shipped shapes), claim
the mechanism with the **registering** overload — `tryClaim(*this)`, implementing
`hal::ICancellable`, exactly as the shipped operations do — or the guard cannot reach your
operation at the deadline: it can repaint the device safe, but a live operation re-commands
its voltage on its very next tick, and the guard will only be able to force-release your
claim and warn (`anonymous claim force-released` in the transcript is this). Third, your
confirmation predicate is *trusted*: an operation cannot second-guess its only eye on the
task, so confirm on a real sensor reading, not on hope — the test suite demonstrates,
deliberately, that a predicate that lies produces a false success.

## Extension 4 — porting to different hardware: the adapter pattern

The `hal/pros/` tree is now the worked example of the library's biggest promise: the core
never knew what hardware it was on, so putting it on hardware meant writing thin adapters —
zero core changes. If you are porting shulib to a different SDK (VEXcode, a custom firmware,
a different robot platform entirely), copy the pattern, which has four parts:

1. **One adapter class per HAL interface**, thin glue only: read the raw device, convert,
   return canonical. Every adapter's header states which SDK call it binds and *why that
   one*, which conversion it applies, and what it deliberately does not do. Read
   `hal/pros/motor.hpp` first — it carries the pattern's hardest lessons (device state that
   survives across programs, and error sentinels on interfaces that have no validity
   channel: screen to the last good value, never zero, never a NaN).
2. **Pure conversion functions, SDK-free**, one per unit belief
   (`hal/motor_conversion.hpp` and friends). The adapter *calls* them; it never re-derives
   the arithmetic inline. This is what lets the unit math be tested and mutation-attacked on
   a laptop with no SDK anywhere in sight — and reused verbatim by your port if your SDK
   speaks the same units.
3. **A register of beliefs.** Every claim about your SDK — units, sign conventions, error
   values, device-state semantics — written down as a falsifiable entry with the bench
   measurement that settles it. Ours is the
   [Hardware Assumptions Register](../hardware-assumptions.md) (the PROS set is HA-94
   onward); the [FAQ](../faq.md) explains why no host test can substitute for that bench.
4. **A programmable SDK stand-in for the host tests** (ours is `test/pros_shim/`), so the
   adapter *glue* is testable: that the conversion is actually called, that error values are
   screened, that configuration is written and read back. Make its defaults adversarial —
   ours boots every fake motor in the wrong encoder units on purpose, because that is what a
   real motor a previous program touched looks like. And make it structurally unable to
   reach a robot build: every header of ours refuses to compile unless the host test build
   says so.

The other seams for bigger work also exist, with their owners named on the
[roadmap](../roadmap.md): position correctors (GPS/vision) plug into the localizer's correction
seam; new telemetry sinks implement `ITelemetrySink`. Before starting anything sized like
these, read the roadmap entry and the relevant header — and talk to the team. Big pieces here
get designed in writing first; that's how the library got this far without a robot.

---

*Next: [Chapter 14 — What it can't do yet](14-what-it-cannot-do-yet.md)*
