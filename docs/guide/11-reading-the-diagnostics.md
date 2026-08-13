# 11 — Reading the diagnostics

> **Covers:** the terminal output, line by line and field by field — the session header, result
> lines, the run summary, per-tick lines, warnings — and **every fault code: what it means,
> what causes it, what to do about it.**
> **Read this if:** you have a transcript in front of you, or you want to be the person the
> team hands transcripts to.
> **Assumes:** [Chapters 6](06-how-things-fail.md) and [8](08-your-first-routine.md). This
> chapter is written to be *used* — skim it once, then come back with real output.

Nobody outside this team knows how to read this output. After this chapter, you will — and
that's a genuine skill: the transcript is designed so that "what went wrong at t=34?" is
answerable from a text file, at the field, without a debugger. Everything below is the *exact*
format the code produces today — every example is copied from output that the test suite pins
byte-for-byte (`test/term_sink_test.cpp`, `test/run_summary_test.cpp`,
`test/motion_result_test.cpp`, `test/session_header_test.cpp` hold the golden strings). If the
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
    ([Ch. 3](03-knowing-where-you-are.md)). Today (no correctors built yet) this is always on.
  - `SFB` — strafe fallback: the H-drive is running a sideways-limited leg at reduced lateral
    speed ([Ch. 4](04-drivetrains.md)).
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
  for a brownout you'd see `battery=10.50V`, for a loop overrun `dt=0.7500 budget=0.5000`).
- **Fault-policy aborts** (Warn, from the scheduler):
  `[SCH] fault abort: ODO_STUCK — cancelling MoveToPose` — pairs with a `✗FAULT_ABORT` result
  line moments later.
- **Overrun attribution** (Warn, tick after an overrun):
  `[SCH] overrun attribution: loc 0.5ms · mot 5.5ms · other 0.2ms (worst mot)` — *which phase*
  of the control tick ate the budget.
- **Throttle notices** (Warn, tag `DIA`): `throttled MOT: dropped 47 lines` — the rate limiter
  (below) dropped output and is telling you exactly how much.
- **Boot-window guard** (Warn, tag `CHS`): `field-frame drive() before the estimate is live:
  commanding zero until it is` — a field-relative manual command arrived before the sensors
  were up ([Ch. 10](10-the-api.md)).
- **`waitUntil timed out after 3.00s`** (Warn, tag `SCH`) — informational; deliberately not a
  fault.

## Every fault code

The complete vocabulary ([Chapter 6](06-how-things-fail.md) gives each one's story; the
authoritative list is [`include/shulib/diag/fault.hpp`](../../include/shulib/diag/fault.hpp)).
"Aborts?" = does it cancel the active motion under the default fault policy.

| Code | Meaning | Typical cause | Aborts? | What to do |
|---|---|---|---|---|
| `PRECONDITION` | code broke a library contract mid-run | a bug in routine or library code (NaN input, bad config) | ends that motion | read the message; fix the calling code — this is a programming error surfaced politely |
| `NAN_POSE` | a not-a-number reached the data path; a safe fallback was substituted | divide-by-zero upstream, corrupt sensor math | no | the estimate froze rather than went insane — treat the pose as suspect from this moment; find the named quantity in the detail |
| `LOOP_OVERRUN` | a control tick blew its time budget | too much work on the control task, a blocking call where none belongs | no | check the attribution line for *which phase*; recurring overruns degrade control quality everywhere |
| `ODO_STUCK` | wheels demonstrably spinning, odometry reporting no motion — **the estimate is lying** | dead/unplugged rotation sensor, robot physically stalled against something | **yes** (the only default abort) | this is the serious one: after it, the pose can't be trusted. Physically check the tracking wheels and their cables first |
| `IMU_LOST` | the IMU was alive, then stopped responding | cable/port failure mid-run | no | heading is now coasting on the last good data; expect growing heading error after this timestamp. Check the cable |
| `GPS_GATE_REJECT` | an absolute fix arrived and the fusion gate refused it | sensor glitch — or an estimate so far gone the truth looks crazy | no | a few: the gate doing its job. Many in a row: estimate and sensor disagree badly — cross-check the pose story around that time |
| `BROWNOUT` | battery at/below the danger threshold | tired battery + heavy motor load | no | swap the battery; re-read the rest of the log knowing the motors were weak |
| `MOTION_TIMEOUT` | a motion's watchdog fired | blocked, jammed, unreachable, or under-budgeted ([Ch. 12](12-when-things-go-wrong.md) has the differential) | n/a (already ended) | informational — the result line's `✗TIMEOUT` is the same event |
| `MOTOR_OVER_TEMP` | a drive motor crossed the thermal throttle step (55 °C) | sustained load; too much current for too long | no | the motor is now weaker than the control model thinks; rest it. In a match: expect sluggishness |
| `IMPLAUSIBLE` | the estimate or a command violated a physical sanity bound (e.g. pose jumped impossibly fast) | usually a symptom of another problem; occasionally a library bug being caught in the act | no | advisory — look at what *else* happened that tick; report persistent ones |

Notes on the machinery: fault raising is **edge-triggered** (a problem persisting 500 ticks is
*one* fault episode, not 500 lines), the **first** fault of a run is latched immutably (that's
the `FIRST` marker and the summary field — the root cause is unerasable by later noise), and
per-code counts survive across motions within a run. The default abort policy —
`ODO_STUCK` aborts, everything else logs and continues — is configurable per run, but change it
only with a reason: the default encodes "abort only when the estimate itself is lying"
(everything else is better survived than halted). The reasoning is spelled out in
[`include/shulib/motion/motion_scheduler.hpp`](../../include/shulib/motion/motion_scheduler.hpp).

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
  fine.

There's also a three-line **controller LCD display** (first fault, fault count, battery, and a
deliberately ticking clock — so a frozen screen is distinguishable from a crashed program) for
reading health at the field without a laptop; see `diag/controller_display.hpp`.

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
which is exactly the file you are most likely to be holding.

**Honest status:** the format, the sink and the decoder exist and are tested; the piece that
actually writes to `/usd/` on a real brain does not (it is deliberately kept out of the core, and
lands with the rest of the hardware bridge). See [Chapter 14](14-what-it-cannot-do-yet.md).

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

---

*Next: [Chapter 12 — When things go wrong](12-when-things-go-wrong.md)*
