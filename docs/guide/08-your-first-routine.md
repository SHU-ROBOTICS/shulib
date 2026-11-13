# 8 — Your first autonomous routine

> **Covers:** writing and running a complete autonomous routine, from an empty file to a
> readable run report — every line explained.
> **Read this if:** you've built the project ([Chapter 7](07-getting-set-up.md)) and want to
> make a (simulated) robot move.
> **Assumes:** [Chapters 2](02-the-field-and-coordinates.md)–[7](07-getting-set-up.md). C++
> knowledge needed: variables, functions, and the idea that `foo.bar()` calls `bar` on `foo`.
> Everything else is explained as it appears.

## Before you type anything

**Every line of code in this chapter is real, compiled, and runs in the test suite** — the file
[`test/guide_examples_test.cpp`](../../test/guide_examples_test.cpp) contains exactly the code
you see here (test cases `guide-08a`, `guide-08b`, `guide-08c`, `guide-08d`), and CI runs it on
every commit.
You have two ways to follow along:

1. **Run the finished thing first** (recommended — you see the destination before the journey):

   ```sh
   SHULIB_GUIDE_PRINT=1 ./build/test/shulib_tests -tc='guide-08*'
   ```

   That runs just this chapter's cases and prints each run's full diagnostic transcript to your
   terminal. All the output shown below is from a real run of exactly this command (2026-08-11;
   your durations and build hash will differ slightly).

2. **Type it yourself**: create `test/my_first_auton_test.cpp`, build it up as the chapter goes,
   and re-run `cmake --build build/test -j && ./build/test/shulib_tests -tc='my*'` at each step.
   (Any `*_test.cpp` file in `test/` is discovered automatically on the next build; if yours
   somehow isn't, re-run the configure step from Chapter 7. Wrap your code in
   `TEST_CASE("my first auton") { … }` with the includes from the top of
   `guide_examples_test.cpp`.)

One honest note before we start: this routine runs against the **simulated** robot, because
that is where all *motion* code still runs ([Chapter 7](07-getting-set-up.md) explains why).
The hardware adapters underneath do now exist and have read and commanded real devices at a
bench — but no control loop has ever closed on a robot, so a routine like this one has never
driven anything physical. The
punchline of the design is that the routine itself — the `firstRoutine` function below — doesn't
know that. It talks to a `Chassis`, and whether the chassis is simulated or real is decided
entirely by the wiring around it. This exact function is the kind that will run on the real
robot, unchanged.

## The shape of what we're building

Three parts, and keeping them separate in your head is most of the learning:

1. **The wiring** — build a `Chassis` out of parts: a drivetrain description, hardware,
   localization, diagnostics. Done once, at startup. (On a real robot the same parts are built
   from the real `hal/pros` adapters, plugged into the slots your test wiring fills with fakes —
   `src/programs/drive_program.cpp` does it for robot two through `robot::Robot`, the two-line form at
   the end of Step 2. `src/build_selection.hpp` chooses which program runs, on which robot, and
   `src/main.cpp` only hands it the PROS entry points.)
2. **The routine** — a plain function that tells the chassis what to do. This is the part
   you'll write many of.
3. **The report** — reading what actually happened.

We'll write the routine first, because it's the point of everything else.

## Step 1 — the routine

```cpp
// The first routine: leave the start tile, make a slow precise approach, slide
// sideways along the goal wall, and turn to face the corner. Every call blocks
// until the motion settles or honestly gives up — the return value says which.
ExitReason firstRoutine(Chassis& chassis) {
    // Tell the localizer where the robot starts. Position comes from you;
    // heading is owned by the IMU (chapter 3).
    chassis.setPose(Pose2d{-48_in, -24_in, 90_deg});

    // Drive to a field position AND rotate to a heading, at the same time.
    ExitReason leg1 = chassis.moveTo(Pose2d{-24_in, 0_in, 45_deg},
                                     {.timeout = 5_s});

    // A slow, precise approach: this leg's speed is capped at 20 in/s.
    ExitReason leg2 = chassis.moveTo(Pose2d{-12_in, 12_in, 45_deg},
                                     {.timeout = 4_s,
                                      .maxLinearSpeed = Velocity{20.0}});

    // Slide sideways while actively holding the current heading.
    ExitReason leg3 = chassis.strafeTo(-12_in, 24_in, {.timeout = 3_s});

    // Face the corner — always the short way around.
    ExitReason leg4 = chassis.turnTo(135_deg, {.timeout = 2_s});

    // Real routines branch on these; here we just report the worst one.
    if (leg1 != ExitReason::Settled) { return leg1; }
    if (leg2 != ExitReason::Settled) { return leg2; }
    if (leg3 != ExitReason::Settled) { return leg3; }
    return leg4;
}
```

Now every line, slowly.

**`ExitReason firstRoutine(Chassis& chassis)`** — a function that takes a chassis (the `&` means
"the actual chassis, not a copy") and returns an `ExitReason` — the `Settled` / `TimedOut` /
`Cancelled` vocabulary from [Chapter 5](05-getting-there.md).

**`chassis.setPose(Pose2d{-48_in, -24_in, 90_deg});`** — the first line of essentially every
routine. A `Pose2d` is the position-plus-heading triple from
[Chapter 2](02-the-field-and-coordinates.md), and this one says: the robot starts 48 inches left
of center, 24 inches toward the near side, facing 90° (toward the far side). Odometry is a
running total ([Chapter 3](03-knowing-where-you-are.md)); this is where the total starts. On a
real field, you measure where you place the robot and write *that* here — every inch of
placement error is error the run begins with.

The `_in`, `_deg`, and `_s` suffixes are shulib's **typed units**: `-48_in` is *a length*,
`90_deg` is *an angle*, `5_s` is *a duration* (there's a `_ms` for milliseconds, too), and they
are not interchangeable with each other or with bare numbers. If you pass a
plain `24.0` where a length belongs, the code does not compile. That's deliberate — entire
seasons have been lost to a degrees-vs-radians mix-up that a type system would have caught (the
API chapter has more on this).

**`chassis.moveTo(Pose2d{-24_in, 0_in, 45_deg}, {.timeout = 5_s});`** — the workhorse.
"Drive so that the robot ends at field position (−24, 0), facing 45°." Because this robot is
holonomic ([Chapter 4](04-drivetrains.md)), the translation and the rotation happen *at the same
time* — it slides toward the target while turning, one smooth motion.

The second argument is the per-motion **options**, and `.timeout = 5_s` is the watchdog
from [Chapter 5](05-getting-there.md): this leg gets five seconds, then it stops and reports
`TimedOut`. Two budgeting rules worth learning now: the budget is a *ceiling*, not a target (a
leg that settles in 1.7 s returns in 1.7 s — generous budgets cost nothing when things go well);
and the *first* motion's budget must also cover sensor boot — motions politely wait, motionless,
until the estimate is live, and on a real robot the IMU takes about 2 seconds to calibrate after
power-on.

The call **blocks**: your function stops on this line until the motion is over, which is what
makes a routine readable top-to-bottom, like a to-do list. And it cannot block forever — the
watchdog guarantees it.

**`.maxLinearSpeed = Velocity{20.0}`** (leg 2) — a per-motion speed cap, in inches per second.
The default budget is fast; this leg approaches at a third of it. You'll use this constantly:
fast across open field, slow and precise near scoring. Options apply to *one* motion — the next
call is back to full speed. (A value of `0`, the default, means "use the configured default" —
that's why you only write the fields you want to change.)

**`chassis.strafeTo(-12_in, 24_in, …)`** — pure sideways translation: go to field position
(−12, 24) while *actively holding* the current heading (45°, where leg 2 left it). Use it when
the whole point of the move is not disturbing your aim — sliding along a goal wall while facing
the goals. Note it takes an (x, y), not a full pose: the heading is "whatever you have now, keep
it."

**`chassis.turnTo(135_deg, …)`** — rotate in place to face 135°. Field heading, not "turn by
90°" — and always the short way around, per Chapter 2's angle-wrapping discussion.

**The `if` ladder** — each leg's `ExitReason` is checked. This routine just gives up and reports
the first failure; a competition routine might retry, or skip to the next scoring opportunity.
The library's job is to make sure you always *get* that honest answer; what to do with it is
strategy, and it's yours.

## Step 2 — the wiring

Now the part that builds the `Chassis` the routine drives. It's longer, but it's written once,
and every piece is a concept you've already met. This is test case `guide-08a`, and it is also,
piece for piece, the graph `robot::Robot` builds for you (the end of this step) — the structure
`src/main.cpp` used to spell out by hand:

```cpp
namespace k = shulib::kinematics;
namespace loc = shulib::localization;

// Step 1 — the drivetrain, described as data (an X-drive, wheels 7 in
// from the center).
const k::MatrixKinematics kin = k::xDrive(7_in);

// Step 2 — the robot. On a real V5 the hal/pros adapters go here (they
// exist; a device provider supplies them); here it is the host test plant the
// whole library is tested against. The feedforward constants describe
// the robot being driven — they MUST match between plant and
// controller (chapter 5).
shulib::sim::SimHarnessConfig simCfg;
simCfg.plant.wheelFf = {.kS = 1.2, .kV = 0.17, .kA = 0.0};
simCfg.plant.initialPose = Pose2d{-48_in, -24_in, 90_deg};  // where it's placed
shulib::sim::SimHarness robot{kin, simCfg};

// Step 3 — localization: odometry (two tracking wheels + IMU) fused into
// the one official pose estimate.
loc::PilonsOdometry odom{robot.imu(), robot.makeForwardTrackingWheel(),
                         robot.makeLateralTrackingWheel()};
loc::ComplementaryFusion fusion{};
loc::Localizer localizer{robot.clock(), robot.imu(), odom, fusion};

// Step 4 — diagnostics: a terminal formatter, the fault latch that
// records what goes wrong, and the health monitor that watches for it.
FakeCharSink capture;  // on the robot: the USB serial port; here: a string
shulib::diag::TermSink term{robot.clock(), capture};
shulib::diag::FaultLatch faults{term, robot.clock()};
shulib::diag::HealthMonitor health{faults};

// Step 5 — one dependencies bundle, one pacer, one Chassis.
const shulib::motion::MotionDeps deps{.ctx = &robot.context(),
                                      .localizer = &localizer,
                                      .kinematics = &kin,
                                      .faults = &faults,
                                      .health = &health};
SimPacer pacer{robot};
shulib::chassis::ChassisConfig cfg;
cfg.motion.wheelFf = {.kS = 1.2, .kV = 0.17, .kA = 0.0};  // matches step 2
Chassis chassis{deps, pacer, cfg};
```

Walking through it:

**Step 1: the drivetrain is data.** `xDrive(7_in)` builds the kinematics
([Chapter 4](04-drivetrains.md)) for an X-drive whose wheels sit 7 inches from the robot's
center. That one value-construction is the *entire* description of the drivetrain. Building the
H-drive robot instead means calling `hDrive(…)` with its geometry — nothing else in this file
would change. No configuration file, no code generator: a working chassis is plain C++, by
promise (there's a test that holds the library to that).

**Step 2: the robot itself.** `SimHarness` is the simulated robot: simulated motors, IMU,
tracking-wheel sensors, battery, clock, and the physics connecting them. We tell it where the
robot is physically placed (`initialPose`) — the simulation's equivalent of putting the robot on
the field — and we give it feedforward constants (`kS`, `kV`, `kA`,
[Chapter 5](05-getting-there.md)) describing its motors. On a real robot, this step becomes
"construct the objects that talk to real hardware" — and **those adapters now exist**
(`include/shulib/hal/pros/`, built in the R1 chunks; `src/programs/drive_program.cpp` is the worked
example). This
harness implements the *same interfaces* they do, which is exactly why the routine below does
not change between them.

**Step 3: localization.** Reads exactly as [Chapter 3](03-knowing-where-you-are.md) described:
`PilonsOdometry` does the per-tick geometry from the two tracking wheels, with heading from the
IMU ("Pilons" is the team whose write-up popularized the method); `ComplementaryFusion` is the
current fusion policy; `Localizer` owns the one official estimate everything else reads.

**Step 4: diagnostics.** `TermSink` formats diagnostic events as human-readable terminal lines;
it writes bytes somewhere via a small "character sink" — on a robot, the USB serial port; here, a
string we can print and assert on. `FaultLatch` records faults (remembering the *first* one
specially — [Chapter 6](06-how-things-fail.md)); `HealthMonitor` watches battery, IMU liveness,
and temperature, and raises those faults.

**Step 5: assemble.** `MotionDeps` is the bundle of everything a motion needs, handed over as
one struct. The **pacer** deserves its one paragraph, because it answers a question you should
be asking: *if `moveTo` blocks, what keeps the world moving while it waits?* Answer: the chassis
repeatedly calls the pacer's `pace()` — "let the world advance one tick" — between control
updates. In simulation, that steps the physics by 10 ms:

```cpp
struct SimPacer final : shulib::motion::ITickPacer {
    explicit SimPacer(shulib::sim::SimHarness& h) : harness{&h} {}
    void pace() override { harness->plant().step(0.01_s); }
    shulib::sim::SimHarness* harness;
};
```

On the real robot, `pace()` is simply "sleep until the next 10 ms boundary" — and that adapter
is built (`ProsTickPacer`). This one little seam is precisely where "simulated time" and "real
time" swap, and it's why the routine can't tell the difference.

Finally `Chassis chassis{deps, pacer, cfg};` — the object the routine drives.

### The same wiring, in two lines

Steps 3 to 5 are the same every time: the drivetrain decides the kinematics, the sensors decide
the odometry, and everything after that is plumbing. So the library can do that plumbing for you,
from a description of the robot written as **data**. This is test case `guide-08d` — the same
simulated X-drive as above, the same routine, built a second way:

```cpp
// The same X-drive, described as DATA: which port drives which wheel, and
// where the IMU and the two tracking wheels are plugged in.
shulib::config::RobotConfig config{
    .drivetrain = {.kind = shulib::config::DrivetrainKind::XDrive,
                   .xdrive = {.ports = {1, 2, 3, 4},  // FL, BL, BR, FR
                              .cartridge = shulib::hal::MotorCartridge::Green,
                              .imuPort = 10,
                              .driveRadius = 7_in}},
    .odometry = {.kind = shulib::config::OdometryKind::TrackingWheels,
                 .wheels = {.forward = {.port = 6, .wheelDiameter = 2_in, .offset = -3_in},
                            .lateral = {.port = 7, .wheelDiameter = 2_in,
                                        .offset = -4.5_in}}}};
config.tuning.motion.wheelFf = {.kS = 1.2, .kV = 0.17, .kA = 0.0};  // matches step 2

// Line one: ask. The verdict names the port or the field that is wrong,
// before a single object exists.
const shulib::robot::BuildVerdict verdict = shulib::robot::validate(config, devices);
REQUIRE(verdict.ok);

// Line two: build. Steps 3 to 5 above, wired, in one line.
shulib::robot::Robot built{config, devices};

// The routine does not know, or care, how its chassis was built.
const ExitReason outcome = firstRoutine(built.chassis());
```

**`RobotConfig`** is the whole robot as one value — which ports drive which wheels (with their
signs, when a motor is mounted backwards), where the IMU is, which odometry it runs, what the
motion layer is tuned to. A person can type one, and a calibration or a file can produce one.

**`devices`** is the one new idea: a *device provider*, the object that knows what a port is. The
config says "the front-left motor is on port 1"; the provider is what turns "port 1" into a motor
you can command. In this test it is the host's provider, bound to the simulated robot's motors and
sensors (the test file shows the binding). On a V5 brain it is
`shulib::hal::pros::ProsDeviceProvider`, which opens the real hardware port by port — and before
it opens anything on a smart port it asks the brain what is actually plugged into that port, so an
empty or wrong port becomes a sentence naming it (`port 13: nothing plugged in (expected a motor)`)
instead of a crash or a sensor that silently reads zero. (The brain's 3-wire ports are the one
exception: nothing can see a pneumatic valve on one, so a line is opened on trust.)

**`validate`** is the question "can this robot be built, right now?", answered with a yes or a
sentence naming the port or the field that is wrong. **`Robot`** is the answer being yes: it builds
the same objects the longhand wiring above builds, in the same order — the test suite drives a
builder-made robot and a hand-wired one side by side on the host test plant and requires them to
agree to the last bit. `built.chassis()` is the same `Chassis` your routine already takes.

The longhand stays, and it stays worth reading: it is exactly what `Robot` does for you, and it is
the path you take (Tier 3 in the project's
[accessibility model](../shulib-v2-master-plan.md#17-accessibility--progressive-disclosure-for-teams-that-cant-code-yet))
when you need something the builder does not build. One honest note: every program in `src/` that
builds a robot builds it this way, and no program wires a graph by hand any more. "shulib Drive", in
`src/programs/drive_program.cpp`, builds its `robot::Robot` from robot two's calibration record (the
auton tester builds robot two the same way) — and it has not run on a robot yet (a program before
it, "shulib Teleop", drove robot two open-loop under a driver on 2026-09-25, in a version wired by
hand). The other kind is an X-drive typed as a `constexpr RobotConfig`
in `src/robots/xdrive_config.hpp` — worth reading as the shape of a robot described by hand — whose ports
are invented, so on every robot that exists it refuses at boot, naming the first port that is not
what it says. Nothing built this way has run on a robot, and the on-robot provider has only been
tested against a host stand-in for the PROS headers.

## Step 3 — the report, and running it

Three more lines surround the routine call — a `RunReporter`, which turns the run into the
readable transcript this library is a little bit famous for (on this team, anyway):

```cpp
// Step 6 — the run reporter: session header now, one result line per
// motion as it ends, a summary when we say the run is over.
shulib::motion::RunReporter report{term, chassis.scheduler()};
report.sessionStart({.buildHash = shulib::diag::compiledBuildHash(),
                     .routineId = "first-auton",
                     .alliance = "red",
                     .side = "left",
                     .portMap = "sim"});

// Step 7 — run the routine.
const ExitReason outcome = firstRoutine(chassis);

// Step 8 — end the run.
report.finishRun();
```

Run it (`SHULIB_GUIDE_PRINT=1 ./build/test/shulib_tests -tc='guide-08a*'`), and this comes out
— the whole thing, unabridged:

```text
[t=   0.00] [SES] run start · build v0.1.1-162-g562f8fe · routine "first-auton"
[t=   0.00] [SES] alliance red · side left · batt 12.60V
[t=   0.00] [SES] ports sim
[t=   1.75] [MOT] MoveToPose#1 ✓SETTLED final( -24.2,  -0.2,  45.0°) over   n/a  drift  n/a    1.75s
[t=   3.39] [MOT] MoveToPose#2 ✓SETTLED final( -12.2,  11.8,  45.0°) over   n/a  drift  n/a    1.64s
[t=   4.70] [MOT] StrafeTo#3 ✓SETTLED final( -12.0,  23.8,  45.0°) over   n/a  drift  n/a    1.31s
[t=   5.88] [MOT] TurnTo#4 ✓SETTLED final( -12.0,  23.8, 134.3°) over   n/a  drift  n/a    1.18s
── RUN SUMMARY ───────────────────────────────────────────
 motions 4 · settled 4 · timeout 0 · cancelled 0 · aborted 0
 heading max  n/a  final  n/a  · gating rejects 0 · brownout no
 worst loop dt   10.0ms · first fault none · dropped 0 rec 0 ln
 build v0.1.1-162-g562f8fe · routine "first-auton" · batt 12.6→12.6V
──────────────────────────────────────────────────────────
```

Read it with what you know: the three `[SES]` lines are the **session header** — which build,
which routine, what battery, so a log is never mysterious about where it came from. Then one
result line per motion: `MoveToPose#1 ✓SETTLED … 1.75s` — motion #1 settled at position
(−24.2, −0.2) facing 45.0°, in 1.75 seconds. That's within half an inch of the (−24, 0, 45°) we
asked for — the settle tolerance doing its job. Then the summary: four motions, four settled,
no faults, no brownout. A clean run. ([Chapter 11](11-reading-the-diagnostics.md) decodes every
field of every line, including the ones that only appear when things go wrong.)

One thing should bother you: `over n/a drift n/a`. Overshoot and heading drift are computed
from the high-rate diagnostic record stream, and we haven't connected it — so the library says
**"no data"** rather than printing a plausible `0.00`. Remember that move: shulib never invents
a number. Where you see `n/a`, you're seeing honesty, not breakage.

## Step 4 — turn on the full stream

Test case `guide-08b` is the same run with one addition: the per-tick record stream routed to
the terminal. (The wiring needs a two-line forwarding helper because the simulated hardware and
the terminal formatter each want the other constructed first — a sim-only wart, explained in the
test file, that real-robot wiring won't have.) Run it:

```sh
SHULIB_GUIDE_PRINT=1 ./build/test/shulib_tests -tc='guide-08b*'
```

Now, between the landmarks, the transcript carries the robot's every 10 ms heartbeat:

```text
[t=   0.01] [MOT] cmd#1▸2 tgt( -24.0,   0.0,  45.0°) err( 24.00", 24.00",-45.0°) v(  42.4,  42.4,-3.14) q=1.00 DR
[t=   0.02] [MOT] cmd#1▸2 tgt( -24.0,   0.0,  45.0°) err( 23.69", 23.69",-43.7°) v(  42.4,  42.4,-3.05) q=0.96 DR
[t=   0.03] [MOT] cmd#1▸2 tgt( -24.0,   0.0,  45.0°) err( 23.37", 23.38",-42.4°) v(  42.4,  42.4,-2.96) q=0.93 DR
```

Squint and you can *watch the control loop think*: motion `cmd#1` in state `▸2` (Running),
target (−24, 0, 45°), error shrinking tick by tick — 24.00" to 23.69" to 23.37" — while it
commands velocity `v(42.4, 42.4, −3.14)` (field-frame vx, vy in in/s; rotation in rad/s). The
`DR` flag says the estimate is dead-reckoning (no absolute correction — [Chapter 3](03-knowing-where-you-are.md)),
which is true on every tick of *this* run because it has no corrector wired in. The library does
ship correctors now, and with one attached the flag clears on the ticks where a fix is applied. And with the stream connected, the result lines now
carry real measurements — `over 0.00" drift 0.0°` — instead of `n/a`. (You'll also see
interleaved unstamped `[LOC] idle` lines: the *simulator's* own ground-truth records riding the
same channel. Only the sim produces those.)

Scrolling hundreds of tick lines is normal at first. Chapter 11 teaches the disciplined version:
landmarks first, then drill into ticks only around the moment that interests you.

## Step 5 — change something, and watch the library tell the truth

The fastest way to trust a tool is to watch it handle failure. Test case `guide-08c` starves a
motion: drive 30 inches with a 0.5-second budget — physically not enough time.

```cpp
const ExitReason r = c.chassis.moveTo(Pose2d{30_in, 0_in, 0_deg},
                                      {.timeout = 0.5_s});
```

What comes back: `r == ExitReason::TimedOut`, the motors are stopped (commanded to zero volts —
verified in the test), and the transcript says, at exactly t = 0.50:

```text
[t=   0.50] [MOT] MoveToPose#1 ✗TIMEOUT final(  22.5,   0.0,   0.0°) over   n/a  drift  n/a    0.50s
```

It got 22.5 inches of the way there, ran out of budget, stopped, and told you — no hang, no
exception, no pretending. This is the failure model from [Chapter 6](06-how-things-fail.md) in
one line.

Now experiment on your own copy. Good ones to try: give leg 1 an unreachable target (behind a
"wall" doesn't exist in the empty sim — but 500 inches away does); drop `.maxLinearSpeed` to
`5.0` and watch durations stretch; ask `turnTo` for `-135_deg` instead of `135_deg` and check
which way it turns (shortest path — from 45°, both are a 180° turn; Chapter 2's tie-break says
counterclockwise). Every experiment ends with a transcript that tells you what happened; getting
fluent at reading it is the actual skill this chapter is building.

## Where you are now

You can wire a chassis, command it, and read the report — which is genuinely most of what
writing autonomous routines is. What's left is vocabulary and judgment: the full set of verbs
and their fine print ([Chapter 10](10-the-api.md)), reading diagnostics like a pro
([Chapter 11](11-reading-the-diagnostics.md)), and debugging when reality disagrees with you
([Chapter 12](12-when-things-go-wrong.md)).

---

*Next: [Chapter 9 — The recipe API](09-the-recipe-api.md), where this routine becomes a
chain of steps with failure handling built in — or skip ahead to
[Chapter 10 — The API, as prose](10-the-api.md) for the full verb set.*
