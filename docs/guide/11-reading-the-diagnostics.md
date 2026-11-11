# 11 — Reading the diagnostics

> **Covers:** the terminal output, line by line and field by field — the session header, result
> lines, the run summary, per-tick lines, warnings — **every fault code: what it means,
> what causes it, what to do about it** — and how your own code prints lines into the same log.
> **Read this if:** you have a transcript in front of you, or you want to be the person the
> team hands transcripts to.
> **Assumes:** [Chapters 6](06-how-things-fail.md) and [8](08-your-first-routine.md). This
> chapter is written to be *used* — skim it once, then come back with real output.

Nobody outside this team knows how to read this output. After this chapter, you will — and
that's a genuine skill: the transcript is designed so that "what went wrong at t=34?" is
answerable from a text file, at the field, without a debugger. Everything below is the *exact*
format the code produces today — every example is copied from output that the test suite pins
byte-for-byte (`test/term_sink_test.cpp`, `test/run_summary_test.cpp`,
`test/motion_result_test.cpp`, `test/session_header_test.cpp`, for the `ODO_STUCK` line
`test/odom_tick_audit_test.cpp` and, for the wall reset's `WALL` lines,
`test/wall_reset_line_test.cpp` hold the golden strings). If the
format ever changes, those tests change with it — trust them over this page and tell the guide
maintainer.

## How to read a transcript: landmarks first

A full transcript has two kinds of content: a handful of **landmarks** (header, one result line
per motion, warnings/errors, the summary) and a flood of **per-tick lines** (the robot's 100 Hz
heartbeat). The discipline that makes transcripts fast to read:

1. Jump to the **RUN SUMMARY** at the bottom. Is the ledger clean? What's the first fault?
2. Scan the **result lines** top to bottom. Find the first ✗.
3. Only then read **tick lines**, and only around the timestamp that interests you.

Never start at line one and read forward — that's how you drown.

## The session header

The first three lines of every run:

```text
[t=   0.00] [SES] run start · build a1b2c3d · routine "redLeftTall"
[t=   0.00] [SES] alliance red · side left · batt 12.40V
[t=   0.00] [SES] ports L1,2,3 R4,5,6 IMU10
```

`[SES]` is the session subsystem tag (every line carries its source's tag). The header answers
the questions you'll ask *days later* when comparing logs: **which code** (`build a1b2c3d` is
the git commit hash the binary was compiled from — plus `-dirty` if the tree had uncommitted
changes), **which routine**, **which robot config** (the port map), and **what battery it
started with** (read live from the battery at that moment, not typed in by anyone — 12.40 V is
a healthy start; compare [Chapter 6](06-how-things-fail.md) on brownout).

If you ever see this instead:

```text
[t=   0.00] [ERROR][SES] build hash MISSING — define SHULIB_BUILD_HASH at build time; a wrong hash is worse than none, so nothing is invented
```

the binary was built without its identity stamp. Nothing else is wrong — but that log can no
longer prove which code produced it, which is exactly the thing you'll wish you knew later. Fix
the build setup rather than living with it.

## Per-motion result lines

One line per finished motion, appearing in the stream at the moment the motion ended:

```text
[t=  12.50] [MOT] MoveToPose#7 ✓SETTLED final(  24.1,  36.0,  90.1°) over  0.20" drift  0.1°   1.16s
```

Field by field:

- **`MoveToPose#7`** — which primitive ran, and its **command id**: motions are numbered 1, 2,
  3… in start order for the run. The id is the thread that ties everything together — this
  motion's tick lines all say `cmd#7`.
- **`✓SETTLED`** — the outcome, with a glanceable pass/fail mark. The five outcomes:

  | Mark | Meaning |
  |---|---|
  | `✓SETTLED` | arrived within tolerance — success |
  | `✗TIMEOUT` | the watchdog fired first ([Ch. 5](05-getting-there.md)) |
  | `✗CANCELLED` | stopped by code — `cancel()`, or a panic stop |
  | `✗FAULT_ABORT=ODO_STUCK` | the fault policy stopped it; the `=CODE` names the cause |
  | `✗SUPERSEDED` | pre-empted — a newer motion took over |

- **`final( 24.1, 36.0, 90.1°)`** — the pose *estimate* at the boundary. Compare mentally to
  the target: this motion wanted (24, 36, 90°) and landed a tenth of an inch off. This field is
  always real, even when the next two aren't.
- **`over 0.20"`** — **overshoot**: how far past the target the robot pushed at its worst,
  measured along the direction of travel. Persistent overshoot is the aggressive-tuning
  signature ([Ch. 5](05-getting-there.md)).
- **`drift 0.1°`** — the final heading error, absolute.
- **`1.16s`** — duration. Watch this across a run: durations creeping toward their timeouts
  are legs asking for trouble.

When `over` and `drift` read **`n/a`**, the high-rate record stream wasn't flowing (e.g. a
competition build with diagnostics off), so there was no path data to compute them from. The
library prints n/a instead of a fabricated 0.00 — no data is never dressed up as good news.

On a **tank** (since API 2.3), the result line of a motion that set out to *go somewhere* — a
`moveTo` or `driveTo` leg, each leg of `followTrajectory`, a `strafeTo`, and (Tier 3) a
`HoldPose` built with a spot away from the robot — ends with one more column, after the
duration, whatever the outcome:

```text
[t=  12.50] [MOT] MoveToPose#7 ✓SETTLED final(  24.1,  36.0,  90.1°) over  0.20" drift  0.1°   1.16s xt  -0.10"
```

- **`xt -0.10"`** — the **cross-track error**: how far the robot finished off its *line*, in
  inches. The line is the one [Chapter 4](04-drivetrains.md) describes for tank steering: through
  the target point, along the target heading. The sign says which side: **+ is left** of the
  target heading, − is right — for a leg that backed up too, left and right of the target
  *heading*, not of the way the robot was moving. This leg wanted (24, 36) facing 90° and ended
  0.1 in to the right of that line. Like `final(…)`, it is the estimate's story, not a tape
  measure.
- **Why a tank gets it:** it is the one error a tank cannot drive out sideways, so it tells two
  kinds of stopped leg apart. A `✗TIMEOUT` with a large `xt` ended *beside* its line — a weaker
  side the steering could not fully cancel, or a point too far off the line
  ([Chapter 12](12-when-things-go-wrong.md) has the differential). With a small `xt`, being
  beside the line is not the story: compare `final(…)` with the target — short of it *along* the
  line, or at it but never settled (the heading still off — `drift` — or hovering just outside
  the tolerance: Chapter 12's *almost settled, never quite*). A `✓SETTLED` leg's `xt` is always
  inside the arrival tolerance (half an inch by default), because arriving means being that
  close to the point.
- **Where it never appears:** turns (`turnTo`, `face`), `brake`, and the `hold` verb (it holds
  the spot the robot is already on) — they did not set out to go anywhere. X-drive and H-drive
  lines never carry it either (they can strafe a sideways error out), except an H-drive
  configured with no strafe authority at all, which the library treats as a tank. The column is
  there whether or not the steering is switched on (`maxSteer = 0`), and like `over` and `drift`
  it comes from the high-rate record stream — with that stream off, the column is left out rather
  than filled with a made-up number.

## The run summary

The one-screen verdict, printed when the run reporter is told the run is over:

```text
── RUN SUMMARY ───────────────────────────────────────────
 motions 7 · settled 6 · timeout 1 · cancelled 0 · aborted 1
 heading max  0.7° final  0.3° · gating rejects 4 · brownout YES
 worst loop dt   11.2ms · first fault ODO_STUCK@  4.2s · dropped 3 rec 47 ln
 build a1b2c3d · routine "redLeftTall" · batt 12.4→11.6V
──────────────────────────────────────────────────────────
```

- **The motion ledger** — started / settled / timed out / cancelled (by code) / aborted (by the
  fault policy). The first health question: does `settled` equal `motions`?
- **`heading max 0.7° final 0.3°`** — the worst per-motion final heading error, and the last
  motion's. The team's accuracy spec makes heading the hard requirement (< 1°), so this pair is
  the score that matters most.
- **`gating rejects 4`** — how many times the fusion gate rejected an absolute-sensor fix
  ([Ch. 3](03-knowing-where-you-are.md)). Occasional rejects mean the gate is working; *many*
  mean the estimate and the sensor disagree persistently — one of them is wrong.
- **`brownout YES`** — the battery hit the danger line at some point ([Ch. 6](06-how-things-fail.md)).
  A YES here reframes every timeout in the run.
- **`worst loop dt 11.2ms`** — the slowest control tick (budget: the tick should be ~10 ms;
  the monitor flags overruns past 15 ms). Chronic overruns corrupt everything rate-based.
- **`first fault ODO_STUCK@ 4.2s`** — **the most valuable field in the whole output**: the
  *first* fault of the run and when it fired. One root cause typically triggers a cascade of
  secondary faults; this field is the root ([Ch. 6](06-how-things-fail.md)). Start every
  post-mortem here.
- **`dropped 3 rec 47 ln`** — output-throttling losses (see "rate limiting" below). `dropped 0
  rec 0 ln` is a positive claim — nothing was lost — which is why it prints even when zero.
  A run with a blackbox attached can add `· blackbox dropped 12` here; that one appears **only**
  when it is non-zero, because most runs have no blackbox and "blackbox dropped 0" would be a
  claim about something that never ran (see "The blackbox" below).
- **`batt 12.4→11.6V`** — battery start → end. A big drop means hard motor work (or a tired
  battery).

## Per-tick lines

The 100 Hz heartbeat ([Chapter 8](08-your-first-routine.md) showed a live one). During a
motion:

```text
[t=  12.34] [MOT] cmd#7▸1 tgt(  24.0,  36.0,  90.0°) err(  0.40",  0.20",  0.3°) v(  18.0,   4.0, 0.10) q=0.91
```

- **`[t= 12.34]`** — seconds since the run started.
- **`cmd#7▸1`** — command id 7, in state 1. The states: 0 idle, **1 waiting for the estimate**
  (the boot wait — a motion in ▸1 is deliberately motionless), **2 running**, 3 settled,
  4 timed out, 5 cancelled. A motion stuck in ▸1 for its whole budget is a sensor that never
  came up.
- **`tgt(…)`** — the target pose. **`err(…)`** — target minus estimate: x error, y error,
  heading error. This triple shrinking toward zero *is* the control loop working.
- **`v( 18.0, 4.0, 0.10)`** — the **commanded chassis velocity** (field-frame vx and vy in
  in/s, rotation in rad/s). Note: what was *asked for*, not what the wheels did, and not
  voltages.
- **`q=0.91`** — the estimate's 0-to-1 quality/trust scalar.
- **Flags**, appended only when true, in this order:
  - `DR` — dead reckoning: no absolute correction active; error is accumulating silently
    ([Ch. 3](03-knowing-where-you-are.md)). It is on for every tick of a run with no corrector
    wired in; with a GPS or AprilTag corrector attached it clears on the ticks where a fix is
    actually applied.
  - `SFB` — strafe fallback: the leg is asking for more sideways speed than the drivetrain can
    deliver, and the sideways part is being cut to what it can do ([Ch. 4](04-drivetrains.md)).
    On an H-drive that is a sideways-limited leg running at reduced lateral speed. On a tank,
    which cannot move sideways at all, it lights whenever the target point is not straight ahead
    of the robot by more than a hair — a leg off its line, but also a leg that starts turned off
    its target heading (even one that starts ON its line), or the robot's own steering turning it
    for a moment — so on a tank log it usually appears beside `TSTR` (next), which is the part
    doing something about it (on the host test plant, a 40 in leg with one side 9 % weak shows
    `SFB` on 39 ticks, down from about 500 before tank steering existed).
  - `TSTR` — tank steering (a tank only, since API 2.3): `moveTo` changed this tick's turn
    command to steer the robot back onto its line — the heading it aims for bent toward the line,
    and the turn a weaker side keeps adding cancelled ([Ch. 4](04-drivetrains.md)). It lights
    only when that change is larger than 2 % of the robot's turn-speed budget, so sensor noise
    does not keep it on; on an even drive it stays dark, and on a drive with one side 5 % weak it
    is on for a good part of every leg. `TSTR` on to the very end of a leg that then timed out
    means the drift or the offset was bigger than the steering is allowed to remove.
  - `CLMP` — a fusion correction was clamped this tick (the never-snap rule at work).
  - `flt=CODE` — a fault was raised *this tick*: this is where you find the exact moment
    something happened.

Between motions the line starts `[LOC] idle` instead of `[MOT] cmd#…` — same fields, nothing
commanded.

## Warnings, errors, and other lines you'll meet

Leveled messages share the stream. `Info` lines carry no level tag (the common case stays
quiet); others are tagged:

```text
[t=  12.51] [WARN][SEQ] intakeUntilCapture retry 1/3 (optical=none)
[t= 100.00] [ERROR][IMU] lost mid-run
```

The levels, most to least severe: `ERROR`, `WARN`, `INFO`, `DEBUG`, `TRACE`. Specific lines
worth recognizing on sight:

- **Fault lines** (Error level, from the fault latch):
  `fault=NAN_POSE n=1 FIRST pose.x` — the code, the run's running fault count, `FIRST` if this
  is the run's first fault (the root-cause marker), then detail (here: which quantity went bad;
  for a brownout you'd see `battery=10.50V`, for a loop overrun `dt=0.7500 budget=0.5000`, and
  for an odometry tick the odometry refused, the check and its numbers —
  [below](#reading-an-odo_stuck-line)). A loop that stays slow is said **once per slow stretch**:
  its first slow tick logs the line, and every later slow tick of the stretch is counted (in `n=`
  of the next fault line, and in the run's fault count) without a line of its own — the per-tick
  `dt` is on every record. A tick back under the budget ends the stretch; the next slow one is
  said again.
- **A wall reset** (tag `WALL`, Info when it seated, Warn when it refused) — one line per call;
  [its own section below](#the-wall-reset-wall) reads it word by word.
- **Fault-policy aborts** (Warn, from the scheduler):
  `[SCH] fault abort: ODO_STUCK — cancelling MoveToPose` — pairs with a `✗FAULT_ABORT` result
  line moments later.
- **Overrun attribution** (Warn, the tick after a slow stretch begins — once per stretch, beside its
  `LOOP_OVERRUN` line):
  `[SCH] overrun attribution: loc 0.5ms · mot 5.5ms · other 0.2ms (worst mot)` — *which phase*
  of the control tick ate the budget (every later slow tick's phases ride its records).
- **Throttle notices** (Warn, tag `DIA`): `throttled MOT: dropped 47 lines` — the rate limiter
  (below) dropped output and is telling you exactly how much.
- **A tank leg that will not be steered** (Warn, tag `MOT`, once, at the leg's first live tick):
  `tank moveTo not steered: it starts 26.6 deg off its target heading (steers within 15.0 deg)
  -- turn first: face()` — the leg starts too far off its target heading for tank steering to
  take it on (steering it would mean the library making your turn), so it runs exactly as a tank
  `moveTo` did before API 2.3 and will most likely time out. Fix: `face()` the point (or
  `turnTo` the heading) before this leg — for a `reverseTo` that backs in, `faceAway()` it,
  since the turn it needs is the one that puts the robot's back to the point ([Ch. 9](09-the-recipe-api.md)).
- **Boot-window guard** (Warn, tag `CHS`): `field-frame drive() before the estimate is live:
  commanding zero until it is` — a field-relative manual command arrived before the sensors
  were up ([Ch. 10](10-the-api.md)).
- **`waitUntil timed out after 3.00s`** (Warn, tag `SCH`) — informational; deliberately not a
  fault.
- **A released leftover claim** (Warn, tag `TEL`, once per mechanism at the start of driver
  control): `driver control: RELEASED a claim left on mechanism 'claw' at entry (an operation that
  outlived its task) -- its buttons work from the first tick` — an operation autonomous was
  running when the field ended it still held that mechanism; driver control freed it so its
  buttons work ([Ch. 10](10-the-api.md), "Buttons for mechanisms"). Expected after an autonomous
  cut off mid-operation; anywhere else, find out who took the claim. (A program whose mechanisms run
  on the mechanism service — robot two's, since the mechanism core's fifth part — says it with the
  service's `MECH` line below instead.)
- **A mechanism step's verdict** (tag `MECH`; `Info` when it SUCCEEDED, `Warn` for every other
  verdict) — ONE line per step the mechanism service ran, written the moment the step ended. In
  driver control a toggle's press, a preset and a homing press are steps too, so each driver press of
  robot two's claw logs one — a close as the `Warn` `UNCONFIRMED ... counted done` below (nothing
  senses the grab), a press within the claw's move time of the last as `CANCELLED ... (replaced by a
  newer command)`; expected, not a fault ([Ch. 18](18-mechanisms.md#on-the-drivers-buttons--the-driver-files-controls)):

  ```text
  'intake': SUCCEEDED -- sees a piece in 0.97s
  'claw': UNCONFIRMED -- time in 0.50s (no piece sensor to confirm it; counted done)
  ```

  Field by field: the mechanism's name as the robot file spells it; the verdict; after `--`, WHAT
  ended it — `time` (its time condition, or a pneumatic's move time), `sees a piece`,
  `stopped seeing a piece`, `stalled`, `reached` (a position move within its tolerance, a home found,
  a flywheel at speed) — or nothing, when no condition ended it (its time limit, a refusal, a
  cancel); `in` the seconds since the step was commanded; and, in brackets, the reason when there is
  one. The verdicts: `SUCCEEDED`; `UNCONFIRMED` — done, but nothing CONFIRMED it, and
  the bracket says whether it was `counted done` (`StepReport::countedDone`: no sensor exists to
  ask — a sensorless grab, or a declared piece sensor missing this boot, named with its port) or
  not (the sensor was there and said no, or did not answer). An auton's mechanism STEP
  ([Ch. 18](18-mechanisms.md#in-an-auton--one-line-per-step)) goes on after a counted-done step and stops
  on any other `UNCONFIRMED`; a `then()` action that returns an operation's verdict continues on
  `SUCCEEDED` only ([Ch. 9](09-the-recipe-api.md)), so there a counted-done verdict stops the chain.
  `TIMED_OUT` (its time limit came first and no condition it asked for was met — a piece sensor
  that answered on every read but never as asked, a stall that never came, or no condition at all
  but the limit, `Until::bound()`; since the core's third part also a position move that never got
  within its tolerance — `it holds where it stopped`: it never pushes on toward a target it could
  not reach —, a flywheel that never reached speed — `it keeps spinning` —, and a homing push —
  `its home switch never read pressed`, or `it never stalled against its stop: it drew at most 0.62
  A when stopped, its homing signature needs 2.00 A`: the most current the push drew when stopped (no
  faster than its homing signature's speed) beside what that signature asks — a homing signature above
  what the push can draw never trips —, or `it never stalled against its stop: it never slowed to its
  homing signature's speed -- did it reach its stop?` (its time ran out first), [Ch. 18](18-mechanisms.md));
  `STALLED` (a stall
  signature the robot file states tripped — the same tick also raises `MECHANISM_STALLED`, once; the
  bracket says during what: `a stated stall signature tripped` during the step, and after it `...
  while holding`, `... while held to move` (a button pushing an arm or a lift), `... while held to
  run` (a power mechanism's), `... while spinning`, or `... while homing: its home switch never read
  pressed`); `NOT_HOMED` (a position move before the mechanism found its zero — nothing moved: `it
  has not found its zero yet: home it first (homeBySwitch)` or `(homeByStall)`, or `it is declared
  neverHomed: a move needs a zero`); `CANCELLED` (stopped from outside — the bracket
  names by what: `replaced by a newer command`, `stopped`, `released`, `driver control began`, `autonomous
  began`, `driver control ended`, `its claim was taken by another owner`, `its claim was released
  outside the service`, `its service was destroyed`; NO bracket means it was cancelled through its
  claim — by the run guard (its deadline or abandon cut, whose own `SEQ` line is below, or its
  end-of-run cancel-all) or by a program's own `cancel()`); `REFUSED` (`another owner holds its
  claim` — nothing moved; or, from a route, a MARKER whose args are wrong — its own line, below);
  `ABSENT` (`not built at boot: ...`, the boot's own sentence for the port — nothing moved). A homing step on a mechanism that needs no motion SUCCEEDS at once, its bracket
  saying why: `it rests at its stop at boot: its zero is kept`, `its angle sensor read again`. Lines
  you will meet:

  ```text
  'intake': UNCONFIRMED -- time in 1.00s (its piece sensor is missing this boot, port 9; counted done)
  'intake': UNCONFIRMED in 1.00s (its piece sensor stopped answering: no evidence on 101 of 101 reads)
  'grip': UNCONFIRMED in 0.45s (its piece sensor did not see a piece)
  'roller': TIMED_OUT in 1.00s
  'catapult': STALLED -- stalled in 0.71s (a stated stall signature tripped)
  'intake': REFUSED in 0.00s (another owner holds its claim)
  'arm': SUCCEEDED -- reached in 0.88s
  'arm': TIMED_OUT in 3.00s (it holds where it stopped)
  'arm': STALLED -- stalled in 3.15s (a stated stall signature tripped while holding)
  'arm': NOT_HOMED in 0.00s (it is declared neverHomed: a move needs a zero)
  'flywheel': TIMED_OUT in 2.00s (it keeps spinning)
  ```

  **"Stopped answering" is not "saw nothing".** A sensor cable that comes out does not read as an
  error: the adapter answers with its last good value, and a distance sensor's "far" is exactly its
  "no object". So a reading taken while the sensor's read-fault count moved is NO evidence either
  way, and a step that had any such reading cannot say the piece never came — `no evidence on N of
  M reads` counts them. Check the sensor's cable; a step that keeps saying it needs its sensor
  looked at, not a longer time limit.
- **Where a mechanism's zero went** (tag `MECH`, at every home — at boot for one that needs no motion,
  and at the end of a homing step): `'arm': homed -- +0.0 deg from where its encoders read zero at
  boot` (the first time: where its zero sits against the motor encoders' — or its angle sensor's —
  reading at boot, in degrees of its output) or `'wrist': homed -- its angle sensor reads 170.0 deg, so
  it is at 10.00 deg` (an absolute sensor); `'arm': homed again -- its zero moved -27.0 deg (past its
  tolerance: was something in the way?)` (a re-home). Info, but a Warn when a re-home moves the zero by
  more than the mechanism's "reached" tolerance: a stall-home trusts the FIRST stall, and a piece wedged
  under the arm is a stall too ([Ch. 18](18-mechanisms.md)).
- **A mechanism's current cap** (tag `MECH`, at boot, for a mechanism whose robot file states
  `.currentCap` — [Ch. 18](18-mechanisms.md#current-caps--how-hard-a-string-is-pulled)): Info
  `'lift': current cap 1.20 A per motor (2 motors: 2.40 A in all) -- set and read back` when every
  motor took it. Otherwise a Warn per motor that did not: `'lift': motor 2 (port 12) did not take its
  current cap: asked 1.20 A, the write was refused, reads back 2.50 A -- it runs; that motor is not
  capped as stated` (or `the write was accepted, reads back 2.00 A` — the motor kept another number),
  or `'lift': motor 2 (port 12): current cap 1.20 A stated, but this provider cannot cap a motor -- the
  motor's own 2.50 A applies`; the mechanism runs either way. And a Warn for each stall or homing
  signature not MORE than 0.2 A below the cap (exactly 0.2 A below included; judged as the motor reads, at the first whole milliamp that trips the signature — chapter 18): `'lift': its stall signature (1.05 A) is within 0.2 A of its
  current cap (1.20 A): a capped motor may read a little under its cap, so it may never trip (HA-223)` —
  lower the signature, or raise the cap. **Once a second** afterwards (inside the mechanism service's
  tick — a program stuck in its own loop checks nothing, and the motor is believed to keep its cap
  meanwhile, HA-219): a Warn
  `'lift': motor 1 (port 11): its current limit reads 2.50 A, the motor's default (it lost power?
  HA-220) -- its cap 1.20 A written again`, once per loss; or, once, `'lift': motor 2 (port 12): its
  current limit reads 1.80 A, not its cap 1.20 A: something else set it (another program, or VEXos's
  sharing -- HA-221) -- not written again`. A loss over and over is most likely a loose cable or a
  brown-out (believed: that a power loss is what resets a limit is HA-220, unmeasured).
- **A motor's current limit that is not the default** (Info, tag `MECH`, at boot, for a mechanism with
  NO cap — its motors are read, never written): `'claw pivot 6': motor 1 (port 6): its current limit
  reads 1.80 A: a cap left by another program (HA-220), or VEXos's sharing with many motors plugged in
  (HA-221) -- nothing written`. That motor is believed to push with less than its full 2.5 A this run
  (HA-219). Power the
  robot off and on: a cap another program left is believed to go with the power (HA-220); one that
  comes back with every motor plugged in is VEXos's own.
- **A computed target clamped** (Warn, tag `MECH`, once per step): `'arm': target 175.00 clamped to the
  travel limit 160.00 (soft stop)`, or a flywheel's `'flywheel': speed 4000 rpm clamped to its free
  speed 3600 rpm` — a target computed while the auton ran was past what the robot file allows; the step
  went to the limit instead. Expected only if your code computes targets; check its arithmetic.
- **A leftover claim released by the mechanism service** (Warn, tag `MECH`, at a mode change — once
  per mechanism): `'claw pivot 7': a claim left by another owner RELEASED as driver control began
  (its operation was not cancelled -- it may have lived on a deleted task)` — the mechanism
  service's line for the same kind of event as the `TEL` line above (`as autonomous began` when it is
  an auton that starts with a claim driver control left).
- **A step's on-failure choice** (tag `RTN`, [Ch. 9](09-the-recipe-api.md#choosing-what-a-failure-does-onfail-and-label)):
  `routine 'two-pieces': step 2 (intake) FAILED: mechanism step FAILED (TIMED_OUT) — carrying on; drive
  safed` (Warn: a step written after `onFail(carryOn)` failed, and the routine went on), the same ending
  `— skipping to 'second piece'; drive safed` for `onFail(skipTo(...))`, then one Info line per step the
  skip passes over (`step 3 (arm) jumped — skipping to 'second piece'`) and one where it lands (`the skip
  to 'second piece' landed at step 5 (driveTo)`). A stop line can end `— the step was CANCELLED, so its
  on-failure choice is ignored` or `— the time is up, so its on-failure choice is ignored` (the deadline
  always stops), or `— its skip target 'X' is behind it: a skip only goes forward`; a Warn
  `step N: its skip target 'X' is behind it` is the same mistake caught where it is written, and
  `on-failure choices with no run guard` means the Routine was built from a bare chassis. A wait whose
  start a skip passed over reads `mechanism NOT STARTED (its start was skipped)`; one whose start did
  nothing because it named another robot's mechanism (a different kind or units, or a preset or
  position this robot's mechanism of that name lacks), and the routine carried on past it,
  reads `mechanism NOT STARTED (its start did nothing: another robot's declaration)`.
- **A routine stopped by a mechanism step** (Warn, tag `RTN`): `routine 'score-three' STOPPED at step 4
  (intake): mechanism step FAILED (TIMED_OUT) — skipping the rest; drive safed` — the step's verdict in
  the brackets (its own `MECH` line above says why), the rest of the auton skipped
  ([Ch. 18](18-mechanisms.md#in-an-auton--one-line-per-step)). Two brackets are the routine's own:
  `(TIMED_OUT: the mechanism service never ticked -- it is not in the chassis pacer's chain)` — the
  PROGRAM's wiring: nothing ticked the service during the step's wait — and `(TIMED_OUT: no verdict
  within the wait's bound; the step runs on to its own)` — a `waitUntilDone` given less time than the
  step has left. A step on a mechanism the robot does not have says so first, tag `MECH`: `'claw' is
  not on this robot (its wiring list does not have it): the step did nothing` (another robot's
  same-named mechanism: `'arm' on this robot is not the mechanism this step names (...)`). The third
  one, `'arm': the mechanism service is not bound (the program bound no robot list): the step did
  nothing`, is the PROGRAM's: its boot never bound the mechanism service. The auton still ran — its
  drive steps before that line drove — and stopped at its first mechanism step, which commanded
  nothing (every mechanism step would say the same). Look for the boot's own line about the service:
  it must be bound before any auton can run.
- **A route marker REFUSED** (Warn, tag `MECH`): `route marker 'arm_score' REFUSED: it needs its time
  limit in ms, above 0 (args[0])` (or `its volts (args[0], -12..12, not 0) and its time in ms, above 0
  (args[1])`) — a marker of a mechanism's route id written without the args its step needs
  ([Ch. 18](18-mechanisms.md#as-route-markers--every-mechanisms-ids-run)); the route stops there,
  nothing commanded, and `RUN`'s stop line names the marker `returned REFUSED`. Fix the route's marker.
- **The run guard cut a mechanism** (Warn, tag `SEQ`, once per run — later cuts are counted, not
  logged): `mechanism 'intake' cut at the deadline -- its operation cancelled into its safe state
  (later cuts are counted, not logged)` (or `after the abandon`, when the field disabled the robot) —
  a mechanism step was still running when the guard's deadline arrived (or the abandon came), and the
  guard cancelled it: the step's own `MECH` line reads `CANCELLED` with no bracket. Expected when an
  auton runs long; the guard's report carries the run's total in `RunGuardReport::mechanismCuts`.

## The wall reset (WALL)

A wall reset ([Chapter 10](10-the-api.md#the-wall-reset--one-axis-against-a-field-wall)) writes
**exactly one line per call**, tag `WALL` — `Info` (no level tag) when it SEATED, `Warn` when it
REFUSED — and nothing else: no line per sample, and no scheduler "timed out" line (its window ends
itself). Each line below is pinned byte for byte (after its `[t=…]` prefix) by
`test/wall_reset_line_test.cpp`, which computes each from the library's own checks on the season's
left wall — and fails if this page quotes anything else; nothing here has run on a robot:

```text
[t=  14.20] [WALL] left wall x SEATED (LEFT sensor): -53.30 -> -55.71 (moved -2.41, limit 3.00) | median 6.99 in spread 0.25 n 16 | bound 0.87 | faults ok
[t=  14.20] [WARN][WALL] left wall x REFUSED disagrees (LEFT sensor): wall says -55.72, estimate -52.20, limit 3.00 | median 6.98 in spread 0.25 n 16
[t=  14.20] [WARN][WALL] left wall x REFUSED keep-out (LEFT sensor): Load Zone at along -46.0 (widened 5.3 = beam 2.3 + margin 3.0 + drift 0.0) | clean -41.8..-33.6
[t=  14.20] [WARN][WALL] left wall x REFUSED not square (LEFT sensor): beam 7.0 deg off square (limit 5.0)
[t=  14.20] [WARN][WALL] left wall x REFUSED beam low (LEFT sensor): expected 11.00 in past the ceiling 10.77 in (lens 10.50 in up, cone 18.0 deg)
[t=  14.20] [WARN][WALL] left wall x REFUSED bad request (LEFT sensor): maxCorrection UNSET: say how far this wall may move x
```

Word by word: the request's `name`, when it set one, then a colon; the wall's name (`left wall`); the
axis it measures (`x` or `y`); the verdict's word (`SEATED`, or `REFUSED` and the first check that
failed); which side's sensor was read — `(LEFT sensor)`, a sensor, not a direction; then that
verdict's numbers. On a SEATED line: the coordinate before -> after, how far it moved and the limit
(`maxCorrection`), the window's median reading, spread and sample count, the answer's worst-case
error (`bound`), and `faults ok` — or `faults n/a` when the sensor has no read-fault count to check (a
hand-wired handle without one: accepted, but never called "ok"). The same tail ends the per-sample
refusals (`no return`, `low confidence`, `out of range`): `faults ok` there means the refusing sample
was read cleanly — the port was fine, the reading itself was the problem. A `read fault` line ends
`faults moved` instead: that sample's read-fault count moved, so its numbers may be the sensor's last
good value held over, not a reading. A keep-out line names the stretch
the beam would land on, the beam centre's coordinate ALONG the wall, how far either side of it had to
be clear (the beam's spread, the margin, and the drift allowance — 10 % of the travel since that
along-wall coordinate was last set), and the nearest clean stretch for the beam's centre — park so
the beam centre lands inside it. The same line comes from the everyday call an auton writes
(`competition::resetFromWall` — [Chapter 10](10-the-api.md#the-everyday-call-from-an-auton)), which adds
three words of its own: `no sensor`, `not a field start` (naming the start: `start RELATIVE -- the season's
walls are FIELD coordinates: declare a FIELD start`, or `start UNSET`) and `cut`; its line goes to the
auton's own log.

**Every verdict, its word, what the line also prints, and what to do:**

| Verdict | Word | The line also prints | What to do |
|---|---|---|---|
| Seated | `SEATED` | before -> after, moved, the limit, median, spread, n, the bound, faults | nothing; now and then compare the next leg's end with the tape |
| NoSensor | `REFUSED no sensor` | — | the everyday call only: this robot file declares no wall sensors — add the `walls` block ([Chapter 10](10-the-api.md#the-robot-files-side-sensors)). On the host test plant's tester rig it is expected — that rig has none |
| SensorAbsent | `REFUSED sensor absent` | why the sensor is absent (the handle's `absence`, or the provider's reason) | check the cable and the port; the boot line's `walls L4 R5(ABSENT)` says which side |
| MountUnset | `REFUSED mount unset` | — | measure the mount (tape: forward, left, height, facing) and type it with `measured = true` (the boot line says `MOUNT UNSET` until you do) |
| NotLive | `REFUSED not live` | — | the estimate never went live (the IMU still calibrating?) — read the boot lines; a program error |
| MotionActive | `REFUSED motion active` | — | a motion is still armed (a Tier-3 `async()`): let it finish or cancel it first |
| BadRequest | `REFUSED bad request` | the field, e.g. `maxCorrection UNSET: say how far this wall may move x`, or `limits.window must be at most 0.6 s …` | fix that field in the auton (or the robot's limits) |
| NotFieldStart | `REFUSED not a field start` | the start mode: `start RELATIVE`, or `start UNSET` (a route whose Start states no heading) | the everyday call only: declare a FIELD start (`StartPose::field(…)`, or a route Start that states its heading); the season's walls are field coordinates |
| NotSquare | `REFUSED not square` | the beam's angle off the wall's normal | 5°–20°: `turnTo` square before the reset. Near 90° or 180°: this beam faces away from that wall — the wrong side named (LEFT/RIGHT), the wrong wall, or the nose not along the wall |
| KeepOut | `REFUSED keep-out` | the stretch's name (Toggle, Toggle bracket, Loader, Load Zone, start pieces, corner), the beam's along-wall coordinate, the widening, the nearest clean stretch | park so the beam lands in the wall's clean stretch; after a long drive it narrows — reset earlier in the leg, or the other axis first |
| BeamLow | `REFUSED beam low` | the expected range or the median, the ceiling, the lens height and the beam's spread | park nearer the wall, under the ceiling; a ceiling near zero means the sensor is mounted too low or its beam spread is unmeasured |
| Moving | `REFUSED moving` | the speed (the estimate's displacement over the last `window`, per second) and the turn rate seen | brake and let the leg settle first (`r.brake()`, a short `pause`) |
| ReadFault | `REFUSED read fault` | the sample number, then `faults moved` | the port faulted mid-window — check the cable |
| NoReturn | `REFUSED no return` | the sample, its confidence and distance | nothing within ~78 in — or a CLEAR wall panel: see what the beam faces |
| LowConfidence | `REFUSED low confidence` | the sample, its confidence and distance | a dark, shiny or slanted surface, or the GPS code strip — measure the sensor against it |
| OutOfRange | `REFUSED out of range` | the reading and the window | park so the reading is ~6–8 in; never flush (mount the lens at least 2.5 in inboard) |
| TooFewSamples | `REFUSED too few samples` | n and the window | a pacing problem in the program (the loop not ticking every 10 ms) — report it; or limits that ask for more samples (`minSamples`) than `windowMax` holds at one a tick — fix the limits |
| Spread | `REFUSED spread` | max − min | the beam is on an edge, a seam or something moving — move along the wall |
| Imprecise | `REFUSED imprecise` | the bound, its limit and the heading tolerance | the sensor is far from the fore-aft centre, the robot has turned a lot since its heading was set, or the mount and the sensor's accuracy are unmeasured — measure them and type them |
| Disagrees | `REFUSED disagrees` | the wall's answer, the estimate, the limit | EITHER the robot drifted more than `maxCorrection` OR something other than the wall is in the beam (a robot, a stack): look first; raise the limit only from taped legs |
| Cut | `REFUSED cut` | — | the everyday call only: the run's deadline or a disable landed in the window; nothing changed. When the line says it `came before the window began` (no sample was taken), the call itself came after the deadline: a step before it ran long — move the reset earlier in the auton |

**Choosing `maxCorrection`.** Run the auton up to the reset WITHOUT it, several times, and tape the
error on that axis at the reset point; the largest seen plus ~1 in is the drift this wall may
correct. Never set it near the gap to the nearest thing a wrong reading could mistake for the wall
— a tight limit is the one defence the reading checks cannot give
([Chapter 14](14-what-it-cannot-do-yet.md#the-wall-reset-what-it-cannot-catch)).

## Every fault code

The complete vocabulary. [Chapter 6](06-how-things-fail.md) tells the story behind most of
them — not all; `MECHANISM_STALLED`, `GPS_GATE_REJECT`, `PRECONDITION` and `MOTOR_GROUP_DISAGREE` are catalogued here
but not narrated there. The authoritative list is always
[`include/shulib/diag/fault.hpp`](../../include/shulib/diag/fault.hpp).
"Aborts?" = does it cancel the active motion under the default fault policy.

| Code | Meaning | Typical cause | Aborts? | What to do |
|---|---|---|---|---|
| `PRECONDITION` | code broke a library contract mid-run | a bug in routine or library code (NaN input, bad config) | ends that motion | read the message; fix the calling code — this is a programming error surfaced politely |
| `NAN_POSE` | a not-a-number reached the data path; a safe fallback was substituted | divide-by-zero upstream, corrupt sensor math | no | the estimate froze rather than went insane — treat the pose as suspect from this moment; find the named quantity in the detail |
| `LOOP_OVERRUN` | a control tick blew its time budget | too much work on the control task, a blocking call where none belongs | no | check the attribution line for *which phase*; recurring overruns degrade control quality everywhere |
| `ODO_STUCK` | **the estimate is lying**, for one of two reasons the line names: `wheels spin but odometry reports no motion` (the stall cross-check), or `implausible odometry tick why=…` (the odometry's own trust gate refused one tick — [how to read it](#reading-an-odo_stuck-line)) | stall: a dead/unplugged rotation sensor, or the robot pinned against something. Trust gate: a heading that jumped more than 90° in one tick (`ROT`), one side or pod that moved more than 36 in in one tick (`TRAVEL` — a pod's or single-motor side's port waking up late, a corrupt reading; on a side driven by a motor group the odometry reads the members' median, so one waking member cannot trip it — a drive-side `TRAVEL` there means several members jumped together, or that members whose readings were not numbers were left out of the median of a side whose members had drifted far apart (the membership step, under `L=`/`R=` below), and the group-level check for one bad member is `MOTOR_GROUP_DISAGREE`, which compares member velocities — whether it catches a member whose velocity is held too is not established), or a reading that came back NaN/Inf (`NONFINITE` — on a side driven by a motor group only when NO member of that side gave a number: one member's bad reading is left out of the median, like a missing member) | **yes** (the only default abort) | this is the serious one: after it, the pose can't be trusted. Read the `why=` first — it says where to look. A robot with no tracking wheels (odometry from the drive motors) cannot run the stall check, so on it this code always comes from the trust gate, and "check the tracking wheels" does not apply |
| `IMU_LOST` | the IMU was alive, then stopped responding | cable/port failure mid-run | no | heading is now coasting on the last good data; expect growing heading error after this timestamp. Check the cable |
| `GPS_GATE_REJECT` | an absolute fix arrived and the fusion gate refused it | sensor glitch — or an estimate so far gone the truth looks crazy | no | a few: the gate doing its job. Many in a row: estimate and sensor disagree badly — cross-check the pose story around that time |
| `BROWNOUT` | battery at/below the danger threshold | tired battery + heavy motor load | no | swap the battery; re-read the rest of the log knowing the motors were weak |
| `MOTION_TIMEOUT` | a motion's watchdog fired | blocked, jammed, unreachable, or under-budgeted ([Ch. 12](12-when-things-go-wrong.md) has the differential) | n/a (already ended) | informational — the result line's `✗TIMEOUT` is the same event |
| `MOTOR_OVER_TEMP` | a drive motor crossed the thermal throttle step (55 °C) | sustained load; too much current for too long | no | the motor is now weaker than the control model thinks; rest it. In a match: expect sluggishness |
| `IMPLAUSIBLE` | the estimate or a command violated a physical sanity bound (e.g. pose jumped impossibly fast) | usually a symptom of another problem; occasionally a library bug being caught in the act. One known benign cause: a wall reset that SEATED a jump over ~2.25 in, followed by a Tier-3 loop that ticks the scheduler itself — one advisory line, the estimate correct ([the wall reset](#the-wall-reset-wall)) | no | advisory — look at what *else* happened that tick (a `WALL ... SEATED` line just before it?); report persistent ones |
| `MOTOR_GROUP_DISAGREE` | one motor of a drive side disagreed with its mates for 250 ms: running opposite to the side's command, or under a quarter of its fastest mate's speed, while the side was driven above 1 V and turning above 1 rad/s | a wrong sign for that port, a fight on the gear train, a slipping gear, a stalled or frozen encoder | no | the drive continues in a motion. In DRIVER CONTROL the same verdict also CUTS the drive for 1 s (the library's drive protections, [Chapter 12](12-when-things-go-wrong.md)): read the `TEL` cut line for the port |
| `MECHANISM_STALLED` | a mechanism's stall detector tripped: stall-grade current with the shaft not turning — an operation's, or a stall signature the robot file states (`.stall`), watched by the mechanism service through a step and through what goes on after it — a driver's hold-to-run, a position mechanism's hold and a button's push, a homing push by switch, a flywheel's spin; once per stall, beside the step's `MECH` line (`STALLED -- stalled`) | a jam on a game piece, an over-travelled lift against its stop, a seized gearbox | no (the *operation* or the *step* ends with a stalled verdict and the mechanism is put in its safe state; a driver's hold-to-run ends too and its command number moves — the held button then does nothing more until it is let go; the drive keeps going) | the mechanism stopped itself before it cooked a motor — clear the jam. A stall on every attempt is a mechanical problem, not a tuning one. (A step that ASKED to end on a stall — `stalls`, a squeeze — SUCCEEDS instead and raises nothing) |

Notes on the machinery: fault raising is **edge-triggered** (a problem persisting 500 ticks is
*one* fault episode, not 500 lines), the **first** fault of a run is latched immutably (that's
the `FIRST` marker and the summary field — the root cause is unerasable by later noise), and
per-code counts survive across motions within a run. The default abort policy —
`ODO_STUCK` aborts, everything else logs and continues — is configurable per run, but change it
only with a reason: the default encodes "abort only when the estimate itself is lying"
(everything else is better survived than halted). The reasoning is spelled out in
[`include/shulib/motion/motion_scheduler.hpp`](../../include/shulib/motion/motion_scheduler.hpp).

### Reading an `ODO_STUCK` line

Because `ODO_STUCK` ends whatever motion is running when it fires (the default policy above),
its line carries enough to explain itself. When the odometry's trust gate is the cause, it reads:

```text
fault=ODO_STUCK n=1 FIRST implausible odometry tick why=ROT dth=1.745rad h0=0.012 h1=1.757 L=0.000in R=0.000in
```

- **`why=`** — which check refused the tick, one or more of `NONFINITE`, `ROT`, `TRAVEL`
  (always in that order, joined by `+`):
  - `ROT` — the heading changed by more than the rotation bound in one tick. The default bound
    is π/2 = **1.571 rad (90°)**.
  - `TRAVEL` — one side (or one tracking pod) moved more than the travel bound in one tick. The
    default is **36 in**.
  - `NONFINITE` — the tick's position change came out NaN or infinite, so position was held for
    that tick. A NaN side shows as `NONFINITE+TRAVEL` together: a NaN is not "within the bound"
    either. On a side driven by a motor group, a side reads `NaN` only when **no** member of it
    gave a finite reading: a member whose reading is NaN or infinite is left out of that tick's
    median, exactly like a member that is not there, and the side reads the others. (On a V5 this
    is not expected at all — the motor adapter replaces a failed read with the last good one
    before the group sees it — HA-08 in the
    [Hardware Assumptions Register](../hardware-assumptions.md) — and nothing here has been run
    on a robot.) `NONFINITE` *alone* means every reading was within its bound but the arithmetic
    overflowed — possible only when the travel bound or a tracking wheel's offset is configured
    near the largest number a `double` holds, so check the configuration, not the robot.
- **`dth=`** — the heading change of that tick, in **radians** (1.571 = 90°, 3.142 = 180°).
- **`h0=` / `h1=`** — the IMU heading at the start and the end of the tick, also in radians.
  A `ROT` with the wheels quiet (`L`/`R` near zero) means the *heading* jumped while nothing
  rolled — look at the IMU (its cable, its mounting, a bump) before the wheels.
- **`L=` / `R=`** — each drive side's travel that tick, in inches, on a robot whose odometry
  reads the drive motors. On a robot with tracking wheels the same two slots read **`fwd=` /
  `lat=`** (the forward pod's raw reading and the lateral pod's). A `TRAVEL` names the side or
  pod with the big number — start with its ports and cables. On a side driven by a motor
  group the number is the members' **median**, so it moved because several members jumped
  together — or because members with no reading were left out of it (next sentence); a single
  bad member is absorbed by the median and shows up, if at all, as `MOTOR_GROUP_DISAGREE`. A
  member whose reading is not a number at all is left out of the median for that tick, which can
  shift the side's number and shift it back on the tick the member returns: the median of the
  others is not exactly the median of all. While coupled members' positions agree that is a
  fraction of the gap between them; with several left out it can be as large as their whole
  spread — the member farthest from the median, read alone (HA-165 in the
  [Hardware Assumptions Register](../hardware-assumptions.md), invented).
  Under the 36 in bound such a shift is not reported; past it, the tick reads `TRAVEL`.
  `L=NaN` means no member of that side gave a number.
  (In driver control a member whose readings are held — position AND velocity repeating while its
  side turns — is caught by the library's drive protections instead: marked ABSENT after 250 ms with a
  `TEL` line naming the port, [Chapter 12](12-when-things-go-wrong.md).)

Numbers print as `NaN`, `+Inf` or `-Inf` when they are not finite, and a huge value prints
compactly (`1e+09in`) rather than as a wall of digits. Two more endings you may see:
**`why=UNREPORTED`** means the odometry said "implausible" without saying why (a custom
odometry that does not fill in its account), and **`+STALL`** at the end means the stall
cross-check tripped on the same tick. One *episode* is one line, carrying the values of its
**first** tick: if the heading keeps swinging for half a second, you get the first swing, not
fifty lines.

## Volume control: filtering, throttling, and the competition build

Three independent mechanisms manage output volume, and they're worth knowing so "where did my
lines go?" is never a mystery:

- **Level filtering** (`diag/level_filter_sink.hpp`) — explicit config: "from subsystem `LOC`,
  show me Warn and up." Filtered lines are simply not printed and *not* counted as drops —
  you asked.
- **Rate limiting** (`diag/rate_limit_sink.hpp`) — protection: per-tick records and each
  subsystem's chatter are token-bucketed (defaults: 50 records/s, 20 lines/s per subsystem).
  **Errors and warnings are never throttled.** Every drop is counted, announced on resume, and
  totaled in the summary — involuntary loss is never silent.
- **The zero-cost off switch** — competition builds route diagnostics to a `NullSink`, which
  costs approximately nothing (records aren't even assembled). This is why result lines can
  read `n/a` for a competition run, and why leaving diagnostics code in place for matches is
  fine. (A *competition build* here means one built with diagnostics off. The team's match
  program, "shulib Competition" — [Chapter 17](17-the-competition-program.md) — is not one today:
  it logs to the USB serial line like every robot-two program, so a practice match leaves a full
  transcript.)

There's also a three-line **controller LCD display** (first fault, fault count, battery, and a
deliberately ticking clock — so a frozen screen is distinguishable from a crashed program) for
reading health at the field without a laptop; see `diag/controller_display.hpp`.

## Printing your own lines: one line, one call

Your own code — an auton, a driver file, a task of your own — can add lines to the same serial log.
Do it one way (`#include "shulib/hal/pros/char_sink.hpp"`):

```cpp
void sayArm(int angle, const char* reason) {
    shulib::hal::pros::serialLine("[ARM] at %d deg", angle);
    shulib::hal::pros::serialWrapped("ARM", reason);
}
```

- **A line — `serialLine`:** printf-style, one line per call. It adds the newline if you left it off
  (and never doubles one), and sends the whole line to the USB in ONE write.
- **A long text — `serialWrapped`:** any length, sent as `[ARM] ` lines of at most 256 bytes, broken at
  spaces, the later pieces beginning `[ARM] ... `. Nothing is cut. `serialLine` stops at 256 bytes (its
  newline included) and ends a longer line with `...[cut]` — so a text that can be long belongs to
  `serialWrapped`. (The limits as of 2026-10-08; the [reference](../api/pros-char_sink.md) has the exact
  names.)

**Never `printf`, `puts`, `putchar` or `std::cout << ... << ...`** in code that runs while another task
prints — and in a competition program something always might (the auton runs on its own task, and PROS
runs each phase function on another). Every task on the V5 shares ONE stdout buffer with no lock on it,
so two tasks inside `printf` at once can mix their bytes or lose a line; and `printf("text\n")` is not
even one write — the compiler turns it into `puts`, which sends the text and the newline separately.
(That is reasoned from PROS's C library, not yet seen on a robot — the A4 register's HA-189. The two
functions above are how shulib's own programs print since 2026-10-08, and the project's src build gate
fails a build whose code in `src/` names stdout or stderr, or calls any printer that writes them — every
one in PROS's C library, `std::cout` and its family.) Don't mix the two on one line either:
a `serialLine` sent while `printf` text still waits in stdout's buffer goes out first.

Three limits worth knowing:

- **Keep one write under about 2 KB.** PROS's serial output holds 2047 bytes, and a larger single write
  is believed to wait for ever (HA-191 — reasoned from PROS's library, not yet seen). shulib never sends
  more than 512 bytes in one write: the two functions stay under 256, and a longer text through a sink
  goes out in 512-byte pieces — another task's line can land between two of them, so a line that must
  stay whole stays under 512. Only a `write()` of your own on the serial can reach the limit.
- **The stack.** A `serialLine` takes about 0.9 KB of the calling task's stack, and about 1.25 KB with
  `%f`, `%e` or `%g` — which also allocate memory, so keep them out of a fast loop (upper bounds read from
  the compiled code, 2026-10-08 — not a run on a robot; the header's banner has the numbers). A task you
  create with PROS's default stack (32 KB) has plenty; one created with the smallest
  (`TASK_STACK_DEPTH_MIN`, 2 KB) does not.
- **One case nothing on our side can fix:** if PROS stops a task in the middle of one of these writes,
  PROS's own serial lock is believed never to be released, so every later line of that boot would wait
  for ever (HA-190 — reasoned from PROS's library, not yet seen; [Chapter 17](17-the-competition-program.md)
  says when the competition program can meet it).

## The blackbox: what you read when there was no laptop

Everything above assumes somebody was watching a terminal. At a competition nobody is, and that
is the whole problem the **blackbox** solves: a binary record of the run, written to the brain's
SD card, that you open afterwards.

It does **not** write continuously. A competition build cannot afford to, so the default posture
is a **flight recorder**: every per-tick record goes into a fixed ring in RAM (200 ticks, about
two seconds), the ring quietly overwrites its oldest entry, and *nothing at all reaches the card*
— not one byte — until a fault fires. Then it writes:

```text
[ 256-byte header ]   which build, which routine, alliance/side, port map, when the run started
[ triage frame    ]   which fault, at what time, on which tick, and the FULL record of that tick
[ tick ][ tick ]…     the ticks that came BEFORE the fault, oldest first
[ summary frame   ]   the same numbers as the RUN SUMMARY block
[ end frame       ]   the run closed cleanly
```

The order is deliberate. The fault that triggers a dump might be a brownout, which is the worst
possible moment to start a long write, so the most valuable thing — *what broke, when, and in
what state* — goes first. If the write is cut short, everything already written still decodes,
and the reader tells you the file stops there. **A file with no end frame ended abruptly**; that
absence is information, not corruption.

Two things follow from that design that are worth knowing before you go looking for data:

- **A clean run leaves almost nothing.** No fault, no history: you get the header, the summary
  and the end frame, a few hundred bytes. That is not a bug — if you want a full trace, turn
  streaming on for a bench session.
- **Nothing is lost silently.** The RAM budget is fixed, so if a run generates more than fits
  before you flush, whole frames are dropped and *counted* — never half-written, and never at
  the cost of stalling the control loop. The count is written into the file and, when it is
  non-zero, appears on the run summary line as `· blackbox dropped 12`.

When a dump happens, the same triage information also prints at the end of the run, after the
summary — so the last thing on the screen is why it broke:

```text
[t=   4.25] [ERROR][TRI] fault ODO_STUCK @  4.25 tick 421 preceding 200 brownout no
[t=   4.25] [ERROR][TRI] state pos(  24.0,  36.0) hdg  90.0° q=0.91 DR cmd#7▸1 batt 11.9V
```

**Reading a blackbox file** needs the decoder that ships with it (`diag/blackbox_reader.hpp`).
It refuses a file whose format version it does not recognise rather than guessing at it — a
wrong number read confidently is worse than no number — and it will not crash on a damaged file,
which is exactly the file you are most likely to be holding. The format has grown once without a new
version number, on purpose: since API 2.3 each tick's flags byte also carries the tank-steering
flag (`TSTR` above), in a bit every earlier writer wrote as 0 — so a file from before simply reads
it as "not steering", which is true of it.

**Honest status:** the format, the sink and the decoder exist and are tested — **and so does the
piece that writes to `/usd/` on a real brain** (`ProsBlockSink`, which arrived with the hardware
bridge; it is deliberately kept out of the core). What is still true is the limit that matters:
none of it has ever written to a physical SD card. A missing card is handled rather than fatal —
the sink constructs, refuses every write, and says so. See
[Chapter 14](14-what-it-cannot-do-yet.md).

## Why the estimator trusted (or ignored) a sensor

Every tick, the record carries the fusion layer's verdict on the fixes it was offered: one
**reason**, the **residual** it was decided on (how far the fix disagreed with the estimate — in
x, in y, and now in heading), and the scale it was judged against. This is the part of the file
you read when a routine ended in the wrong place and you need to know whether the estimate was
wrong or the motion was.

| Reason | What happened |
|---|---|
| `None` | Nobody offered a fix this tick. With no correctors wired up, this is every tick. |
| `Accepted` | A fix passed the gate and was folded in as a bounded nudge. |
| `RejectedInnovation` | The fix disagreed with the estimate by more than the fusion layer's hard limit (a foot). The last line of defence — a fix this far out is treated as a misread whatever the sensor claims. |
| `RejectedNoFix` | The source had nothing usable: the GPS is off the strip, covered, or disconnected. **In Driving Skills this is the whole run**, and seeing it is how you tell "no strip, as expected" from "the GPS was never wired up". |
| `RejectedHighYawRate` | The robot was spinning too fast for the fix to be trusted. |
| `RejectedNormalizedInnovation` | The fix disagreed by more than a few times its own claimed accuracy. This is the everyday gate — it adapts to how confident the sensor says it is and to how long the robot has been navigating blind. |
| `RejectedStaleFix` | The sensor re-reported a reading already used. Expect a lot of these: the GPS updates about five times slower than the control loop, so most ticks are legitimately stale, and the corrector folds each reading exactly once. |
| `RejectedSensorQuality` | The sensor claimed a fix but reported so much error that folding it was not worth doing. For a tag, this is a detection the camera itself was not confident about. |
| `RejectedNoTagMapEntry` | A tag was **seen**, and your map does not say where it is. This one is a configuration error you can fix, not the field being the field — check that the tag's id is in your map. An empty map produces this on every tag. |
| `RejectedTagRange` | Every visible tag was too close or too far to be trusted. Far away, a tag's recovered *angle* goes bad long before its distance does, which is why the band exists. |
| `RejectedObservationAge` | The vision task has stopped feeding the corrector — it stalled, died, or was never started. Different from `RejectedStaleFix`, which is the normal state between camera frames. |
| `RejectedMahalanobis` | The **Kalman tier** refused a fix: it disagreed by more than three times what the estimator admits it could plausibly be wrong by. Unlike `RejectedInnovation` this threshold is not a fixed distance — the same fix can be refused when the estimate is fresh and accepted after a long blind stretch. Only the Kalman tier writes this. |
| `CovarianceReinit` | The **Kalman tier declared itself lost** and reset its own confidence — after a long run of consecutive rejections with a persistently large disagreement. **The estimate was not moved**; only the uncertainty was thrown away, so the next fixes can get through. See "Reading a re-init" below. Only the Kalman tier writes this. |

Two habits worth forming:

- **A run full of `RejectedNoFix` in Autonomous means the strip is not being seen.** Check
  mounting height and line of sight before you touch any gains.
- **A run full of `RejectedNormalizedInnovation` means the estimate and the GPS have stopped
  agreeing** — either the sensor is lying, or the estimate drifted far enough that truthful
  corrections now look outrageous. The residual in the record tells you how far apart they were.
- **A run full of `RejectedNoTagMapEntry` means the robot can see tags and you have not told it
  where they are.** This is the one on this list that is definitely your bug and definitely
  fixable.
- **Even one `CovarianceReinit` in a match is worth reading the surrounding ticks for.** The
  estimator only says that after it has been arguing with a sensor for seconds. Something
  displaced the robot without the wheels turning — a shove, a wall, a pinned drivetrain — or a
  sensor started lying convincingly.

### Reading the heading correction

Two more fields carry the yaw story, and they are read together:

- **`correctionDTheta`** — how far the estimator's idea of its own heading moved on this tick.
  This is the field that audits *never-snap for heading*: it can never exceed the documented
  per-tick bound, and if it ever does, that is a library bug, visible after the fact from the
  file alone.
- **`gateResidualHeading`** — how far the tag's heading disagreed with the estimate.

The combination is what tells you what happened:

| `gateResidualHeading` | `correctionDTheta` | What it means |
|---|---|---|
| small | small and non-zero | Normal. The estimator is trimming a small IMU bias, a fraction of a degree at a time. |
| **large** | **zero** | A heading fix was **rejected**. A disagreement this size is far more likely to be a misread tag, a wrong map entry, or a mirrored detection than real gyro drift, so it was refused. |
| anything | zero, every tick | Nothing is correcting heading at all — no tag corrector wired, or none of its fixes are getting through. Read the `reason` column to find out which. |

There is only one `reason` slot and it reports the **position** verdict, so a heading-only
rejection reads as "a large heading residual beside a zero heading correction" rather than having
a word of its own. Position and heading are gated separately: a fix can be good enough to move
your position and not good enough to move your heading, and that is a normal thing to see.

### The two fields that only mean something under the Kalman tier

For most of this library's life two slots in the record sat deliberately empty, because the
default fusion layer has no covariance and a plausible-looking number in either would have been
impossible to tell from a real one later. Under the **Kalman tier** they are real, and they are
the two most informative numbers in the file when an estimate goes wrong.

- **`covarianceTrace`** — how uncertain the estimator currently is about its own position, in
  **square inches**. It is not a distance: to get one, take `sqrt(trace / 2)`, which is roughly
  the 1σ radius. So a trace of 0.5 means "give or take about half an inch"; a trace of 1152 means
  "I could be anywhere within a tile". Watch it **grow** while the robot dead-reckons and **drop**
  each time a fix lands. Under the default complementary tier this same slot carries that tier's
  scalar trust weight instead — the `reason` column tells you which filter wrote it.
- **`gateMahalanobis`** — how far the fix disagreed, measured in units of *how wrong the estimator
  thought it might be*. Under three, it was accepted; over three, refused. A value of 20 does not
  mean twenty inches — it means the fix was twenty times further out than the estimator's own
  uncertainty could account for. Zero on every record written by the default tier, which has no
  covariance to normalise by.

### Reading a re-init

A `CovarianceReinit` tick is worth knowing how to read, because the interesting part is what
*didn't* happen.

1. `gateReason` reads `CovarianceReinit`.
2. `covarianceTrace` **jumps** on that same tick, typically by a factor of hundreds — that is the
   estimator throwing away its confidence, and it is the independent numeric witness that the
   word is telling the truth.
3. `correctionDx` / `correctionDy` on that tick are **inside the normal per-tick bound**, exactly
   like every other tick. Nothing teleported.
4. Over the following second or two, `covarianceTrace` falls again while `correctionDx`/`Dy` run
   at the per-tick limit — that is the estimate walking back to where the sensors say it is.

If you ever see step 3 broken — a correction larger than the documented bound — that is a library
bug and the file is the proof.

**One caveat specific to the Kalman tier.** Under the default complementary tier,
`correctionDx/Dy` is exactly the correction and nothing else. Under the Kalman tier it also
carries a small amount of the filter's own smoothing of the wheel readings, which is not a
correction at all — it is the estimate tracking real motion through a filter. Measured against
the simulated robot, that pushes the number up to about 9% over the per-tick budget during the
hardest direction changes. Read the bound with that allowance under this tier; the correction
itself never exceeds it.

For the deeper design — what's planned beyond the terminal (live telemetry, replay) — see the
[diagnostics plan](../diagnostics-plan.md).


## The exact signatures

Every field of every record in this chapter has an exact spelling in the reference: [`DebugRecord`](../api/debug_record.md), [`MotionResult`](../api/motion_result.md), [`RunSummary`](../api/run_summary.md), [`SessionInfo`](../api/session_info.md) and the [fault codes](../api/fault.md). The serial's own writer — `ProsCharSink`, `serialLine` and `serialWrapped` — is [`char_sink`](../api/pros-char_sink.md). The sinks are there too — [`TermSink`](../api/term_sink.md), [`SdSink`](../api/sd_sink.md), [`NullSink`](../api/null_sink.md), [`LevelFilterSink`](../api/level_filter_sink.md), [`RateLimitedSink`](../api/rate_limit_sink.md) — along with the [blackbox format](../api/blackbox_format.md) and its [reader](../api/blackbox_reader.md), and [`TickAttribution`](../api/tick_attribution.md), which answers *which phase* spent the tick.

The SHUL/2 telemetry wire — the binary stream a laptop tool such as VexBuilder will read — has its
format in [`shul2_format`](../api/shul2_format.md), its reader in
[`shul2_reader`](../api/shul2_reader.md), and the checksum it shares with the calibration record in
[`crc32`](../api/crc32.md). Nothing writes the wire yet: its sink is still to come, so today these
read only bytes the host test suite built.

---

*Next: [Chapter 12 — When things go wrong](12-when-things-go-wrong.md)*
