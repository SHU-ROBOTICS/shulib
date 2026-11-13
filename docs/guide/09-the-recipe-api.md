# 9 — The recipe API

> **Covers:** `Routine` — the simpler way to write an autonomous routine: a chain of named
> steps, ~10 readable lines for a complete auton, with a built-in answer to "what happens when
> a step fails".
> **Read this if:** you've done the tutorial ([Chapter 8](08-your-first-routine.md)) and want
> the shortest honest path to a working routine — or you'll be helping newer members write one.
> **Assumes:** [Chapters 2](02-the-field-and-coordinates.md)–[8](08-your-first-routine.md).

> **Stability.** Both tiers are now **frozen**: the `Chassis` API underneath
> ([Freeze Register](../roadmap.md#freeze-register) row F6) and `Routine` itself (row F10),
> both locked 2026-08-12. Every step spelling and every observer (but the ones listed as not
> frozen below), `RoutineResult` and `RoutineStopCause` change only with a major API-version bump
> plus a migration note
> ([`include/shulib/version.hpp`](../../include/shulib/version.hpp)), and the freeze is enforced
> by a compile-time signature pin
> ([`test/routine_signature_pin_test.cpp`](../../test/routine_signature_pin_test.cpp)) that
> fails the build if a spelling drifts. New steps arrive *additively* — and that path has since
> been used for real, five times. First `RoutineStopCause` gained an **appended** `MechanismFailed`,
> which is a minor version bump (2.0 → 2.1), not a breaking change: existing enumerator *values*
> are pinned and only the end of the list grows, so if you `switch` exhaustively, add the new
> case. Then, at 2.4, two new **steps** were appended, `reverseTo` and `faceAway` (below), with
> nothing that already existed changing shape. And at 2.5 the mechanism core added a second
> constructor (`Routine r{ctx, "name"}`), the mechanism steps (`move`, `run`, `set`, `spinUp`,
> `home`, `stop`, the START forms, `waitUntilDone` — [Chapter 18](18-mechanisms.md#in-an-auton--one-line-per-step)),
> an appended `MechanismStepFailed` (a `switch` over every `RoutineStopCause` needs its case) and
> an appended `RoutineResult::mechanismVerdict`. Then, at 2.6, the on-failure choice arrived the same
> way — `onFail(...)` and `label(...)`, an appended `MechanismNotStarted` (one more `switch` case) and
> six appended `RoutineResult` fields ([below](#choosing-what-a-failure-does-onfail-and-label)) —
> and it was **pinned the day it landed**, because the route maker generates code against it.
> And with chunk CURVE1 (after 2.6), two more **steps** were appended in one block, `curveTo` and
> `reverseCurveTo` ([below](#the-steps)), pinned the day they landed, nothing else changing shape.
> **Not frozen:** `then()`, deliberately — and, until a decision still open, the mechanism steps,
> the second constructor and `lastMechanism()` (the Freeze Register's F10 row gives both readings;
> they are not pinned meanwhile). `then()` is the seam mechanisms plug
> into. Mechanisms now *exist* — the seam was filled, and `then()` accepts a mechanism
> operation's `MechanismOutcome` as a fourth return type — but the surface stays unfrozen until
> a second real consumer has stressed it, which is this project's standing rule for freezing.
> Expect `then()` to keep working; do not assume its exact spelling is permanent.

Once you can write a routine, the [cookbook](../cookbook/README.md) is where to go next: it
answers "how do I write the routine I am writing right now" with complete, compiled recipes —
a two-goal side run, a bail-out on a failed grab, a partner wait, a tank routine. This chapter
teaches the layer; the cookbook uses it.

The code in this chapter is compiled and run in
[`test/guide_examples_test.cpp`](../../test/guide_examples_test.cpp), cases `guide-09a`
through `guide-09d`, and the on-failure section's in
[`test/routine_on_fail_example_test.cpp`](../../test/routine_on_fail_example_test.cpp), case
`guide-09e`. The fine print lives in the header,
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
robot genuinely ends within an inch of the last target — measured on the host test plant's ground
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
| `startAt(pose)` | Seeds the pose estimate with the measured starting pose — the first line of every auton you run yourself (one registered for `competition::runAuton` has none: its entry's `start` is seated for it — [Chapter 16](16-testing-an-auton-on-the-bench.md)) | `setPose` |
| `moveTo(pose, opts)` | Drives to a field pose, translating and rotating at once | `moveTo` |
| `driveTo(x, y, opts)` | Drives to a field *point*, arriving facing it (heading = the bearing, computed when the step runs) | `moveTo` |
| `reverseTo(x, y, opts)` | Drives to a field point and arrives with its *back* to it (heading = the bearing + 180°, computed when the step runs) — backwards the whole way after a `faceAway` | `moveTo` |
| `strafeTo(x, y, opts)` | Translates to a field point while holding the current heading | `strafeTo` |
| `turnTo(heading, opts)` | Rotates in place to a field heading, the short way | `turnTo` |
| `face(x, y, opts)` | Rotates in place to *face* a field point (the bearing, computed when the step runs) | `turnTo` |
| `faceAway(x, y, opts)` | Rotates in place to turn its *back* to a field point (the bearing + 180°) — the turn before a `reverseTo` | `turnTo` |
| `followTrajectory({…}, opts)` | Drives waypoints as chained moves; a leg that fails stops it | `followTrajectory` |
| `curveTo({…}, opts)` | Drives ONE smooth path through up to 16 points without stopping at them, and ends like a `moveTo` — refused before it moves if its first point is more than 90° off the nose ([Chapter 5](05-getting-there.md#flowing-through-points-curves)) | `curveTo` |
| `reverseCurveTo({…}, opts)` | The same, backing: the robot's BACK leads; every heading is still the NOSE's | `reverseCurveTo` |
| `brake(opts)` | Controlled stop: 0 V under brake until the estimate certifies rest | `brake` |
| `hold(duration, opts)` | Actively holds the current pose against disturbance | `hold` |
| `pause(duration)` | Waits, motors idle — the "wait for your partner" beat | `wait` |
| `waitFor(pred, timeout)` | Waits for a condition; if the deadline passes first, **the chain stops** | `waitUntil` |
| `then(action, name)` | Runs your code between motions (see the mechanism seam, below) | — |

`face` and `driveTo` — and their turned-around twins `faceAway` and `reverseTo` — deserve one
sentence of honesty: they are *argument sugar*, not new motion. Each computes a single number —
the bearing from the current pose estimate to the point you named, plus 180° for the two
turned-around twins — and hands it to `turnTo` / `moveTo`. That matters most on a tank drive,
where [Chapter 4](04-drivetrains.md) explained the drivetrain cannot slide sideways and
[Chapter 10](10-the-api.md) showed you the turn-then-drive idiom with `atan2`. `face(x, y)`
**is** that idiom's turn, written in field words — you are still the one deciding to turn,
which is exactly the honesty rule tank verbs follow everywhere in shulib. On a tank it is also
what makes the next leg *steerable*: since API 2.3 a tank `moveTo` (and so `driveTo`) steers
back onto its line when a weaker side pulls it off, but only for a leg that starts within 15° of
its target heading — which a leg right after `face()` does. A `driveTo` with no `face()` before
it starts as far off its heading as the point is off your nose; past 15° it is not steered and
the log says `turn first: face()` ([Chapter 11](11-reading-the-diagnostics.md)).

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
chain (`MotionFailed`, `WaitTimedOut`, `ActionFailed`, `MechanismFailed` when a `then()` action's
mechanism operation returned any verdict other than `Succeeded`, or `MechanismStepFailed` when one of
the mechanism core's steps did — its verdict then in `result().mechanismVerdict`,
[Chapter 18](18-mechanisms.md#in-an-auton--one-line-per-step)); `exit` carries a motion's verdict and is
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
- **Wanting to continue past a failure is legitimate.** The chain can do it for
  you, one step at a time: write `onFail(carryOn)` or `onFail(skipTo("..."))` in front of that
  step (the next section). And it is still one tier down, not a rewrite: call
  `chassis.moveTo(...)` directly for the legs where you want to branch on the `ExitReason`
  yourself, and use chain steps for the rest. Mixing is fully supported.

## Choosing what a failure does: `onFail` and `label`

The policy above is the **default**, and it does not change. You can change it for
**one step at a time**, by writing a modifier in front of that step. From `guide-09e`:

```cpp
RoutineResult threeChoices(AutonContext& ctx) {
    Routine r{ctx, "three-choices"};
    r.onFail(carryOn).driveTo(24_in, 0_in, {.timeout = 2.5_s, .positionTolerance = 1_in});
    r.onFail(skipTo("third piece")).run(intake, 12_volt, seesPiece, 2_s);
    r.move(arm["score"], 1.5_s);
    r.label("third piece").face(0_in, 48_in, {.headingTolerance = 3_deg});
    return r.result();
}
```

- **`onFail(carryOn)`** — if this step fails, the failure is **counted** and said (one `Warn`:
  `... FAILED: ... — carrying on; drive safed`), the drive is parked exactly as a stop parks it,
  and the next step runs from where the robot is. `r.ok()` goes false, `result().failedOn`
  counts the failures, and `firstFailedAt` / `firstFailedName` / `firstFailedCause` name the
  first one. The stop fields stay clean: `stoppedAt` is still 0, because nothing stopped.
- **`onFail(skipTo("third piece"))`** — as `carryOn`, then every step until the one
  `label("third piece")` names is **passed over** — counted in `steps` and in `jumped`, one
  `Info` line each, never run — and the auton goes on **at** that step. In the listing above a
  missed piece skips the score, not the rest of the auton.
- **`onFail(stopChain)`** is the default rule, spelled out. No `onFail` at all means the same.
- **`label("...")`** names the next step. It is not a step itself, so the step numbers on the
  brain's screen do not move. Names are matched whole and case-sensitively (`"B2"`, `"b"` and
  `"B "` are not `"B"`), and they are stored, not copied: write string literals.

Each modifier binds to **exactly one step** — the next one — and is used up there, whatever
happens to that step (it runs, it is skipped after a stop, a skip jumps it, or it throws). Two
`onFail`s (or two `label`s) in a row, and an empty name, are programming errors: they throw.

**A skip only goes forward.** The chain runs its lines as it reaches them; it never stores them,
so it cannot go back. A skip whose target is a label the routine has **already passed** — a label
on a step an earlier skip jumped counts as passed — or the failing step's own label stops the
chain right there, and its stop line says the target is
behind it. That mistake is also logged where it is **written**, as a `Warn` on the `onFail` line
on every run, so a bench run whose step succeeds still shows it. A target that **never comes** (a
misspelling) jumps every remaining step; `result().skipPendingTo` names it. The routine remembers
the last 32 labels it passed; a target further back than that reads as one that never comes.

**Two rules sit above every choice.**

1. **The deadline always stops.** A step whose verdict is **CANCELLED** — the run guard cut it at
   the deadline, an abandon, the fault policy, a step replaced from outside, or a `then()` action
   that returned `Cancelled` — stops the chain whatever it chose. So does any failure once the
   auton's time is up, and a skip still looking for its step when the time runs out. A carry-on
   auton must never keep issuing steps past the buzzer.
2. **Nothing new throws.** A `waitUntilDone(intake, ...)` whose `startRun(intake, ...)` a skip
   jumped does not crash the auton and does not answer an older run of the intake: it **fails,
   counted**, as `MechanismNotStarted` ("its start was skipped"), under the wait's own choice. So
   does a wait whose start did nothing because it named another robot's mechanism (a different kind
   or units, or a preset or position this robot's mechanism of that name lacks), when
   the chain carried on past that start ("its start did nothing: another robot's declaration").

The deadline rule needs the auton's run guard, and the Routine gets it from the **context**:
write `Routine r{ctx, "name"}` in an auton that takes `AutonContext& ctx`, as every listing here
does. A Routine built from a bare chassis (`Routine r{chassis, "name"}`) has only the CANCELLED
half of the rule, and says so once, with a `Warn`, at its first `carryOn` or `skipTo`.

**An arrival tolerance, per step.** `{.positionTolerance = 1_in}` means "arrive within 1 in";
`{.headingTolerance = 3_deg}` means "within 3°". A leg ends when the robot is close enough **and**
slow enough, and on a drive the speed is what decides — so a tolerance looser than the robot's own
loosens **both, in proportion**: 2 in means "within 2 in and slower than 4 in/s". On the host
test plant the 40 in drive takes 1.80 s with the robot's own numbers and 1.35 s within 2 in. A
tolerance tighter than the robot's own tightens only the distance. The leg can end with the robot
still rolling a little; how far a real robot coasts after that has not been measured yet.

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

**Backing into a point** is the same idiom turned around: `faceAway(x, y)` turns the robot's
*back* to the point, then `reverseTo(x, y)` drives backwards onto it and arrives with its back to
it. On a tank the turn is still yours to write, for the same two reasons as `face()`: a tank
cannot slide onto the line, and only a leg that starts within 15° of its heading is steered. A
`reverseTo` leg starts as far off its heading as the robot's *back* is turned from the point — so
right after `faceAway()` it starts within about a degree, and a robot that just drove straight
out from the point already has its back to it and can back straight back without one. From a
robot *facing* the point, though, a `reverseTo` with no `faceAway()` starts 180° off: it is not
steered, the log says `tank moveTo not steered: it starts 180.0 deg off its target heading …
turn first: face()` (for a backwards leg, that turn is `faceAway()`), and on the host test plant
the robot swings round through the half turn as it drives and the step times out beside the
point — almost 8 in beside it at the end of a 43 in leg, and still about an inch at the end of a
6 in one. A holonomic drive arrives turned round without `faceAway()`, because it rotates in
flight — which means its nose leads for the early part of the leg; write `faceAway()` first if
the robot's back must lead the whole way (a mechanism on the back, say). Nothing here has run on
a robot yet: whether a tank backs up exactly as far as its calibration says is an open entry,
HA-167, in the [Hardware Assumptions Register](../hardware-assumptions.md).

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
mistaken for a successful one, and there is no way to spell it that would. (A mechanism DECLARED in
the robot file needs no `then()`: it has its own one-line steps, run by the mechanism service, with
their own verdict policy — [Chapter 18](18-mechanisms.md#in-an-auton--one-line-per-step). `then()` is
the seam for anything else.)

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
- **A mechanism operation is cut too** (since the mechanism core's second part). An operation still
  running at the deadline — the grab in the listing above, ticked inside the chassis's
  `waitUntil`, or a mechanism-service step — is cancelled into its declared safe state at the
  next pace, so the wait standing on it sees it end one pace after the deadline and its verdict
  reads `Cancelled` (which stops the chain). The guard reaches it through the claim it
  registered; an operation of your own that claims ANONYMOUSLY is not reached until the guard's
  cancel-all ([Chapter 13](13-extending-the-library.md) says how to register). The wait's own
  lateness bound above still applies to a wait on anything else.

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
