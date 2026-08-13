# 9 — The recipe API

> **Covers:** `Routine` — the simpler way to write an autonomous routine: a chain of named
> steps, ~10 readable lines for a complete auton, with a built-in answer to "what happens when
> a step fails".
> **Read this if:** you've done the tutorial ([Chapter 8](08-your-first-routine.md)) and want
> the shortest honest path to a working routine — or you'll be helping newer members write one.
> **Assumes:** [Chapters 2](02-the-field-and-coordinates.md)–[8](08-your-first-routine.md).

> **Stability.** Both tiers are now **frozen**: the `Chassis` API underneath
> ([Freeze Register](../roadmap.md#freeze-register) row F6) and `Routine` itself (row F10),
> both locked 2026-08-12. Every step spelling, every observer, `RoutineResult` and
> `RoutineStopCause` change only with a major API-version bump plus a migration note
> ([`include/shulib/version.hpp`](../../include/shulib/version.hpp)), and the freeze is enforced
> by a compile-time signature pin
> ([`test/routine_signature_pin_test.cpp`](../../test/routine_signature_pin_test.cpp)) that
> fails the build if a spelling drifts. New steps arrive *additively*.
> **One exception, deliberately:** `then()` is **not** frozen. It is the seam that mechanisms
> will plug into, and mechanisms do not exist yet — freezing the shape of a placeholder would
> commit the library to a guess. Expect `then()` to keep working; do not assume its exact
> spelling is permanent.

Once you can write a routine, the [cookbook](../cookbook/README.md) is where to go next: it
answers "how do I write the routine I am writing right now" with complete, compiled recipes —
a two-goal side run, a bail-out on a failed grab, a partner wait, a tank routine. This chapter
teaches the layer; the cookbook uses it.

The code in this chapter is compiled and run in
[`test/guide_examples_test.cpp`](../../test/guide_examples_test.cpp), cases `guide-09a`
through `guide-09c`. The fine print lives in the header,
[`include/shulib/chassis/routine.hpp`](../../include/shulib/chassis/routine.hpp), whose
opening commentary — like `chassis.hpp`'s — is meant to be read.

## What a recipe is

Chapter 8's routine was a sequence of `chassis.` calls with an `ExitReason` to check after
each one. That is the full API, and it never goes away. A **recipe** is the same routine
written as a chain of steps on a `Routine` object:

- **Each step runs the moment it is chained.** There is no "build now, run later": the
  routine executes in exactly the order it reads, top to bottom. You can put a breakpoint
  between two steps, print the pose between two steps, or call any `chassis.` verb between
  two steps, and everything happens in program order.
- **Each step delegates to one `Chassis` verb.** The recipe layer contains no motion logic of
  its own — same gains, same watchdogs, same guarantees, same accuracy. (The test suite holds
  this as a bit-for-bit identity: the same routine driven through `Routine` steps and through
  direct `chassis.` calls produces *identical* runs.)
- **Failure is handled for you, loudly.** If a step fails, the chain stops, parks the robot,
  skips everything after it, and tells you what happened. More on this below — it is the main
  thing the recipe layer adds.

## The first recipe

This is chapter 8's `firstRoutine`, one tier up (from `guide-09a`):

```cpp
// The same routine as chapter 8's firstRoutine, written as a recipe: each
// step runs (and blocks) the moment it is chained, so the routine reads in
// exactly the order the robot acts. If any step fails, the chain stops, parks
// the robot, and skips the rest — r.ok() tells you which world you are in.
RoutineResult firstRecipe(Chassis& chassis) {
    Routine r{chassis, "first-recipe"};
    r.startAt(Pose2d{-48_in, -24_in, 90_deg})
        .moveTo(Pose2d{-24_in, 0_in, 45_deg}, {.timeout = 5_s})
        .moveTo(Pose2d{-12_in, 12_in, 45_deg},
                {.timeout = 4_s, .maxLinearSpeed = Velocity{20.0}})
        .strafeTo(-12_in, 24_in, {.timeout = 3_s})
        .turnTo(135_deg, {.timeout = 2_s})
        .hold(300_ms)  // stand your ground for 0.3 s…
        .brake();    // …then park, braked
    return r.result();
}
```

Read it out loud: start here, drive there, approach slowly, slide sideways, face the corner,
hold, park. The test that compiles this listing also asserts what the prose claims: all seven
steps succeed, six of them are motions (`startAt` only seeds the pose estimate), and the
robot genuinely ends within an inch of the last target — measured on the simulator's ground
truth, which the robot's own estimate never sees.

Everything you learned in chapter 8 still applies underneath: the per-call options struct
(`{.timeout = …}`) is [Chapter 10's](10-the-api.md) `MotionOptions`, unchanged; typed
units still refuse bare numbers at compile time (`.strafeTo(-12, 24, …)` does not build, and
neither does `.hold(0.3)` — durations are typed, `300_ms`); and
the run report reads exactly as before, because the same motions run.

## The steps

Every step delegates to the `Chassis` verb named here — [Chapter 10](10-the-api.md) has each
verb's full behavior and gotchas, and they all apply verbatim.

| Step | What the robot does | Delegates to |
|---|---|---|
| `startAt(pose)` | Seeds the pose estimate with the measured starting pose — every auton's first line | `setPose` |
| `moveTo(pose, opts)` | Drives to a field pose, translating and rotating at once | `moveTo` |
| `driveTo(x, y, opts)` | Drives to a field *point*, arriving facing it (heading = the bearing, computed when the step runs) | `moveTo` |
| `strafeTo(x, y, opts)` | Translates to a field point while holding the current heading | `strafeTo` |
| `turnTo(heading, opts)` | Rotates in place to a field heading, the short way | `turnTo` |
| `face(x, y, opts)` | Rotates in place to *face* a field point (the bearing, computed when the step runs) | `turnTo` |
| `followTrajectory({…}, opts)` | Drives waypoints as chained moves; a leg that fails stops it | `followTrajectory` |
| `brake(opts)` | Controlled stop: 0 V under brake until the estimate certifies rest | `brake` |
| `hold(duration, opts)` | Actively holds the current pose against disturbance | `hold` |
| `pause(duration)` | Waits, motors idle — the "wait for your partner" beat | `wait` |
| `waitFor(pred, timeout)` | Waits for a condition; if the deadline passes first, **the chain stops** | `waitUntil` |
| `then(action, name)` | Runs your code between motions (see the mechanism seam, below) | — |

`face` and `driveTo` deserve one sentence of honesty: they are *argument sugar*, not new
motion. Each computes a single number — the bearing from the current pose estimate to the
point you named — and hands it to `turnTo` / `moveTo`. That matters most on a tank drive,
where [Chapter 4](04-drivetrains.md) explained the drivetrain cannot slide sideways and
[Chapter 10](10-the-api.md) showed you the turn-then-drive idiom with `atan2`. `face(x, y)`
**is** that idiom's turn, written in field words — you are still the one deciding to turn,
which is exactly the honesty rule tank verbs follow everywhere in shulib.

## When a step fails

This is the recipe layer's real contribution. A chain of calls makes it easy to *ignore*
failures — and a routine that keeps driving after a failed move is acting out a plan from a
position it is not at, compounding the miss with every step. So the chain's policy, built in
and tested:

1. **Stop.** The first failed step ends the routine's forward progress.
2. **Park.** The drive goes to the defined safe state (0 V, brake mode) immediately.
3. **Skip.** Every later step is counted and logged as skipped, and does not run.
4. **Report.** One `Warn` line names the routine, the step, and the reason; `r.ok()` goes
   false; `r.result()` carries the step index, its name, the cause, and the motion's honest
   exit reason.

A step "fails" when its motion exits non-`Settled`, a trajectory doesn't complete every leg,
a `waitFor` deadline passes with the condition still false, or a `then` action reports
failure.

**Read `result().cause` before `result().exit`.** `cause` says what *kind* of thing stopped the
chain (`MotionFailed`, `WaitTimedOut`, `ActionFailed`); `exit` carries a motion's verdict and is
only meaningful when the cause is `MotionFailed`. After a failed wait or a failed action, `exit`
reads `Running`, which means "no motion verdict here" — not "still going". It is the same
"nothing yet" convention `lastCompleted()` uses, and it surprises everyone once.

From `guide-09b`:

```cpp
const auto kin = shulib::kinematics::xDrive(7_in);
shulib::hal::fake::FakeTelemetrySink log;  // on the robot: the terminal
motion_rig::ChassisRig c{kin, motion_rig::plantConfig(), &log};

// 0.5 s is not enough to cross half the field, so step 2 times out. The
// chain then STOPS: the drive is put in the safe state (0 V + brake) and
// step 3 is skipped — a routine that kept driving from a position it is
// not at would compound the miss blindly.
Routine r{c.chassis, "starved"};
r.moveTo(Pose2d{12_in, 0_in, 0_deg}, {.timeout = 5_s})
    .moveTo(Pose2d{60_in, 40_in, 0_deg}, {.timeout = 0.5_s})
    .turnTo(90_deg, {.timeout = 2_s});

// The result says WHERE it stopped and WHY — a strategy branch, not a mystery.
CHECK_FALSE(r.ok());
const RoutineResult res = r.result();
CHECK(res.stoppedAt == 2);                // which step failed…
CHECK(res.exit == ExitReason::TimedOut);  // …and the motion's honest verdict
CHECK(res.completed == 1);
CHECK(res.skipped == 1);                  // the turn never ran
```

When this happens the routine layer adds two kinds of line to the transcript (subsystem
`RTN`; captured from this exact run):

```text
routine 'starved' STOPPED at step 2 (moveTo): motion TIMEOUT — skipping the rest; drive safed
routine 'starved': step 3 (turnTo) skipped — stopped at step 2
```

— one `Warn` for the stop, one `Info` per skipped step, so a run that stopped early is
legible at a glance in the same terminal you learned to read in
[Chapter 11](11-reading-the-diagnostics.md). The
layers below behave exactly as chapter 8 showed — the result line still reads `✗TIMEOUT`, the
`MOTION_TIMEOUT` fault still latches. Nothing is masked; the chain only adds the stopping.

Two deliberate edges of the policy:

- **A nonsense argument is not a "failed step" — it throws.** A NaN pose or a negative
  timeout is a programming error, and it stays as loud through the recipe layer as through
  the facade ([Chapter 10](10-the-api.md)'s misuse rules). The chain's counters don't move;
  the bad call simply never ran.
- **Wanting to continue past a failure is legitimate — and it is one tier down, not a
  rewrite.** Call `chassis.moveTo(...)` directly for the legs where you want to branch on the
  `ExitReason` yourself, and use chain steps for the rest. Mixing is fully supported; that's
  next.

## Tank drives, and the no-cliff rule

From `guide-09c` — a tank recipe, plus the point this guide keeps making about tiers:

```cpp
const shulib::kinematics::TankKinematics kin{12_in};
motion_rig::ChassisRig c{kin};

// A tank drive cannot slide sideways, and shulib never pretends it can
// (chapter 4). In a recipe YOU still write the turn — in field words:
// face the point, then drive to it.
Routine r{c.chassis, "tank-recipe"};
r.face(0_in, 24_in, {.timeout = 3_s})
    .driveTo(0_in, 24_in, {.timeout = 8_s});
CHECK(r.ok());
CHECK(motion_rig::posErr(c.rig.h.truePose(), Pose2d{0_in, 24_in, 90_deg}) < 1.0);

// No cliff between tiers: the full API is the same chassis, mid-routine.
// Here the direct turnTo IS this leg's "face", done one tier down…
REQUIRE(c.chassis.turnTo(0_deg, {.timeout = 3_s}) == ExitReason::Settled);
// …and the same chain object carries on afterwards, unconfused.
r.driveTo(24_in, 24_in, {.timeout = 8_s}).brake();
CHECK(r.ok());
```

Because steps run eagerly, recipe steps and direct `chassis.` calls interleave in plain
program order — the chain object keeps counting only its own steps, and the scheduler
underneath sees one honest sequence of motions. Outgrowing recipes never means rewriting a
routine; it means replacing the steps you want more control over, one at a time.

**One sharp edge worth knowing before you use this.** A direct `chassis.` call is *not a step*,
so the chain never learns how it went. If a direct `moveTo` times out, `r.ok()` stays **true**,
the step count does not include it, and every following step runs — from a position the robot
never reached. Everything the recipe layer does about failure applies to steps only. When you
want a dropped-tier leg to count, wrap it: `then()` accepts an action returning an `ExitReason`
and honors that verdict, so the leg becomes a step in every way that matters. The
[cookbook](../cookbook/05-mixing-tiers.md) shows both forms side by side, with a compiled test
holding each of them true.

## `then()` — the mechanism seam

`then(action, name)` runs any callable between motions, strictly after the previous step
finishes. An action returning `void` always succeeds; returning `bool`, `false` stops the
chain (`ActionFailed`); returning an `ExitReason` — say, from a direct `chassis.` call
inside the action — has that verdict honored. And since the mechanism layer landed
([Chapter 13](13-extending-the-library.md) shows how to build one), an action may return a
mechanism operation's `MechanismOutcome`: **only `Succeeded` continues the chain.** A grab
that completed but was never confirmed reports `Unconfirmed`, stops the chain as
`MechanismFailed`, and the transcript names the exact verdict — a failed grab cannot be
mistaken for a successful one, and there is no way to spell it that would.

Here is the whole idiom, compiled and run by the test suite. The operation never owns a
loop: you start it, the chassis's own `waitUntil` ticks it once per control tick (this also
works *while a motion is driving* — that is how "intake while moving" is written), and its
verdict goes to `then()`:

```cpp
RoutineResult scoreOne(Chassis& chassis, shulib::manipulation::IMechanismOp& grab) {
    Routine r{chassis, "score-one"};
    r.moveTo(Pose2d{18_in, 0_in, 0_deg}, {.timeout = 5_s})
        .then(
            [&] {
                grab.start();
                (void)chassis.waitUntil(
                    [&] {
                        return grab.tick() !=
                               shulib::manipulation::MechanismOutcome::Running;
                    },
                    2_s);
                return grab.outcome();  // only Succeeded continues the chain
            },
            "grab")
        .moveTo(Pose2d{0_in, 24_in, 90_deg}, {.timeout = 5_s});
    return r.result();
}
```

**Return the outcome.** A `void` lambda that runs an operation and drops its verdict
"succeeds" no matter what happened — the same sharp edge as the dropped direct-call
`ExitReason` above, owned the same way. And if an action returns `Running`, the chain stops
loudly too: an operation nobody drove to completion is not a success either.

One honest spelling note, because older drafts of the plan wrote it differently: the
flagship used to be quoted as `chassis.moveTo(p).then(intake.in)`, and that line was never
real C++ twice over — `Chassis::moveTo` returns an `ExitReason` (which has no `.then()`;
chains belong to `Routine`), and `intake.in` only names a function, it does not call one.
The honest spelling is the one above: a `Routine` chain, and a lambda (or the operation
idiom) inside `then()`.

## The match clock, and what a chain honestly cannot see

A `Routine` has per-step timeouts and **no whole-run deadline** — that is a frozen fact of this
API, not an oversight, and pretending otherwise would be worse than saying it plainly. When a
run-scoped deadline (the sequence layer's guard, [Chapter 6](06-how-things-fail.md)) fires
while a chain is executing:

- **A motion step is cut immediately.** The verb returns `Cancelled` — its honest verdict —
  the chain stops on it like any failed motion, and every later step is skipped. Skips are
  instant, so a chain in a motion pays essentially nothing past the deadline.
- **`pause()` and `waitFor()` cannot be cut.** A wait checks its condition and its *own*
  timeout, and nothing else — the deadline is invisible to it. The lateness bound is exact:
  **the unexpired remainder of the wait's own timeout at the instant the deadline fires,
  summed over every wait step that runs after that instant.** In practice a chain pays one
  term — the wait it was standing in — because its next motion step is refused and stops the
  chain. Consecutive `pause()` steps each pay in full (a pause never fails, so it never stops
  the chain). Budget waits tightly.

When the wait itself is the thing that must respect the deadline — "wait for the partner, but
never past our budget" — use the guard's own wait through `then()`. Only `Satisfied` continues
the chain; both the wait's own timeout *and* the run expiring stop it:

```cpp
Routine r{chassis, "with-partner"};
r.startAt(Pose2d{0_in, 0_in, 0_deg})
    .then([&] { return guard.waitFor([&] { return partnerSignal; }, 30_s)
                    == GuardedWaitResult::Satisfied; },
          "wait-partner")
    .then([&] { ++stepsAfterWait; }, "score");
```

One trap this spelling exists to prevent, because it was measured: folding a deadline into an
ordinary `waitFor` predicate makes the deadline *look* like the condition arriving — the wait
returns satisfied, the chain records success, **and keeps scoring past the buzzer.** The
guard's wait returns its own verdict (`RunExpired`) precisely so time running out can never
read as the thing you were waiting for.

## What a recipe deliberately can't do

- **`drive(speeds, frame)`** — a recipe is a *sequence*; `drive` is a per-loop-iteration
  primitive for driver control and experts ([Chapter 10](10-the-api.md)). Call it directly if
  a routine truly needs it.
- **`cancel()`** — the panic stop belongs to whatever supervises the routine, not to a step
  inside it. (The chain does use it internally when it stops on a failure.)
- **Branching on the pose** — recipes don't look at where the robot is between steps; when
  your strategy does, read `chassis.pose()` between steps, exactly as `guide-09c` reads the
  world mid-routine. That's not a workaround; it's the intended shape of mixed-tier code.

## Where you are now

You can write a complete, honest autonomous routine in about ten lines, know exactly what it
does when a step fails, and drop to the full API mid-routine whenever your strategy outgrows
a step. The full verb fine print is [Chapter 10](10-the-api.md); the exact signature of any
step is in the [generated API reference](../api/routine.md); reading a stopped run's transcript
is [Chapter 11](11-reading-the-diagnostics.md); and worked recipes for real situations are in
the [cookbook](../cookbook/README.md).

---

*Next: [Chapter 10 — The API, as prose](10-the-api.md).*
