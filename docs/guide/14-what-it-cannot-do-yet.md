# 14 — What it can't do yet

> **Covers:** everything this library cannot do today, with where each gap is
> tracked and what closes it.
> **Read this if:** you're about to rely on the library for something, or you're evaluating
> the project from outside.
> **Assumes:** nothing beyond [Chapter 1](01-what-is-this.md). This chapter is deliberately
> readable standalone.

"We under-claim before we over-claim" is written into the
[roadmap's rules](../roadmap.md#how-to-read-this-roadmap), and this chapter is that rule applied to
itself. The links matter more than the prose: statuses change, and the
linked documents are maintained as the truth. Where this page and the roadmap disagree, the
roadmap is right.

## It has never driven a robot under its own control

The headline limitation, stated as many times as it takes:

- **No shulib code has ever closed a control loop or followed a path on a robot.** Bench
  sessions (2026-08-13 onward) commanded real motors open-loop and read real sensors through
  the adapters, and on 2026-09-17 a driver drove robot two with the sticks through them
  (open-loop, the team lead's report, no log) — that validated the *platform layer*, nothing
  more: no loop closed, no wheel turned under motion control. Everything above the platform layer is verified host-side
  only, against a simulated plant and deliberately hostile simulated sensors
  ([Chapter 7](07-getting-set-up.md) explains the approach;
  the [README](../../README.md#what-it-is-not-yet) states it third-heading-from-the-top).
- **It has booted on a brain, and that proves less than it sounds like.** On 2026-08-12 the
  package was uploaded to a V5 brain, where it started, built its entire object graph, and
  printed its diagnostics banner over USB. Worth knowing, because it means the build path works
  and nothing in the library depends on being on a laptop. But every motor and sensor in that
  run was a fake, and the robot said so itself in the second line it printed. Booting is not
  driving, and nothing about motion or accuracy was tested that day.
- **The hardware adapters now exist — and that closes less of the gap than it sounds like.**
  The `hal/pros/` adapters implement every hardware interface over the real PROS SDK — the
  drivetrain-and-driver set and, as of 2026-08-14, the mechanism-sensor set (distance,
  optical, digital lines, the SD-card blackbox device) — and `make` produces a package wired
  to real devices, including a first driver-control loop. Three limits, in decreasing order
  of comfort. *First*, the adapters are host-tested — compiled, run, and mutation-attacked on
  a laptop — against a hand-written stand-in for PROS, which proves the glue faithfully
  implements **our beliefs about PROS** and cannot prove the beliefs: if we are wrong about a
  unit or a sign, the stand-in and the adapter are wrong together and every test stays green
  ([FAQ](../faq.md) has the full version). Every such belief is a labelled register entry
  (HA-94 onward). *Second*, what the bench sessions have actually settled is narrow: seven
  unit-scale beliefs on the drivetrain set (2026-08-13, one robot, once), the IMU's heading
  direction (HA-02, confirmed 2026-08-19) and part of the SD card's behaviour (HA-122,
  2026-08-18). The other heading and sign conventions remain open; only the X-drive build's
  port map is a labelled guess (the tank builds use port tables measured on those robots) —
  and **of the mechanism-sensor adapters only the SD-card writer has touched a physical
  device**: distance, optical and the digital lines have not, and their beliefs (HA-113
  onward, including two the vendored SDK sources do not state) wait on the extended bench
  checklist. *Third*, the
  headline unchanged: a bench reading sensors is not a robot driving. Until a loop closes on
  hardware, "the adapters work" means "the adapters agree with our model of PROS, and that
  model has been spot-checked once."
- **The program that will close the first loop exists — and has not run.** "shulib Auton Tester"
  ([Chapter 16](16-testing-an-auton-on-the-bench.md), 2026-09-28) runs one of the team's autons —
  a `moveTo` under the library's own control, through the competition program's own call — from
  the brain's screen. On the host test plant it finishes the 40 in auton; it has never run on a
  brain, and the beliefs its abort and its SD log rest on (HA-177, HA-178) are unchecked. The
  competition program itself, "shulib Competition" ([Chapter 17](17-the-competition-program.md),
  2026-09-30), runs the same autons in `autonomous()`; it has not run on a brain either, and the
  beliefs a match rests on — that PROS deletes its phase tasks on a mode change (HA-161), and what
  the controller's Timed Run does (HA-162) — are unchecked.
- **The route preview draws a perfect robot, on a stand-in drive, in an empty field.**
  `tools/route_preview.py` ([Chapter 16](16-testing-an-auton-on-the-bench.md#before-the-bench-previewing-a-route))
  runs an auton on the host test plant and draws it, but its drive is the test suite's stand-in (a 12 in tank
  with made-up gains, not robot two's — whose track width is not in the code yet), its plant has no walls (the
  preview checks a 15 in square against the field itself and says THROUGH or TOUCH, but nothing stops the
  plant's robot), it has no side sensors (every wall reset is refused), no pieces and no camera (a step that
  needs to see something cannot). Its times are not the robot's either way. Robot one previews on its own
  mechanisms, but every number its facts file still owes is a placeholder (the page lists them), and a value
  typed there as "chosen, not measured" is not marked at all.
- **The standard Calibrate has not run on a robot in its current form.** It runs whole on the host test
  plant, and 34 of its runs are compared line for line with recorded ones; the calibrator inside it ran on
  robot two in an earlier form on 2026-09-24 ([Chapter 19](19-calibrating-your-robot.md)). Its thresholds
  are invented for a first run (HA-136).
- **Nothing has run on robot one.** No census has seen it, no reading has been taken from it and no photo
  is on file. Its programs and autons run on the host test plant on placeholder numbers, and every build
  that needs a fact nobody has typed refuses
  ([Chapter 21](21-adding-your-robot-by-hand.md#what-robot-one-does-not-do-yet)).
- **Every physical constant is a labeled guess.** Control gains, settle tolerances, sensor
  noise levels, drivetrain geometry, fault thresholds — all provisional, and cataloged as
  falsifiable claims in the [Hardware Assumptions Register](../hardware-assumptions.md)
  (all but the handful the bench settled on 2026-08-13 are still open; each names its blast
  radius if wrong and the measurement that settles it — the register itself is the count, so
  this page does not carry a number that would go stale). Two examples to convey the range:
  the H-drive's sideways
  authority (0.35) is pure invention that could plausibly be anywhere from ~0.15 to ~0.8
  (HA-54); and the entire heading-accuracy story leans on an assumed IMU drift bound (HA-20)
  that is community folklore until measured.
- **Consequently, the accuracy numbers are simulation numbers.** The measured sub-degree
  heading performance ([README](../../README.md#how-verified-is-it)) is real
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
heading at all. That sentence has to change, and **the change is narrow**. Here is what
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
reads as a smaller change than the feature sounds like, that is the true size of it.

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
  having a camera that reads TAGS, a tag map, and a task polling it, none of which exist yet either
  (since the camera's first part the library opens the AI Vision camera, but for its colours and
  objects only — its tags stay off).

## A Kalman filter now exists — and it is not the default

This page used to carry two limitations together: *no Kalman filter*, and *two disagreeing
correctors are bounded rather than resolved*. Both have changed, and as with the heading section
above the change is narrower than it sounds. Same order: what was measured, what it was
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
- **The uncertainty the filter reports is only as good as its noise model.** `covarianceTrace`
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
2026-08-12 — all but the pieces the register leaves outside them: for the recipe layer,
`then()` and the exact wording of its log lines, and — not yet ruled — its mechanism steps (the
end of this section has `then()` and the steps). Frozen signatures and documented behavior change
only with a major version bump and a migration note, and compile-time pins fail the build if one
drifts — routines written today against the frozen surfaces will not need rewriting.

The mechanism seam that used to be a placeholder here is now real code
([Chapter 13](13-extending-the-library.md) is the how-to). What is proven, and against what:

- **Proven, on a host, against fakes:** motor groups and pneumatic lines can be commanded
  through the device seam; a bounded operation runs, is ticked by the chassis's own wait
  (including *while a motion drives*, with the motion's result proven bit-identical to a run
  with no mechanism at all), detects a jam from current + shaft speed, times out under an
  adversarial clock rather than hanging, cancels into a per-mechanism declared safe state,
  and hands `then()` a verdict where a completed-but-unconfirmed action can never read as
  success. All of it against in-memory fakes and hostile fakes that jam, stall and lie.
- **The mechanism core, on a host only (2026-10-15 to 2026-10-29, five of its six parts):** a robot file
  DECLARES each mechanism (an `Arm`, a `Lift`, a `Flywheel`, a `Power` roller, a `Pneumatic`) and
  wires it separately, the build refusing each mistake by name ([Chapter 10](10-the-api.md)); one
  mechanism service runs all five kinds — a move and its hold, homing, the soft stop, a spin to "at
  speed", a run until a piece sensor sees a piece, a valve's position — every behaviour watched for a
  jam ([Chapter 18](18-mechanisms.md)); an auton runs a mechanism on ONE line (`r.move(arm["score"],
  1.5_s)`), a driver file's controls put presets on buttons, a route's markers run every mechanism's
  ids, and the direction check judges a jog. Proven only on the host test plant's invented models, with
  invented starting gains; robot two's programs run on the service since its fifth part (2026-10-29),
  its claws as declarations — never on a robot — and the bench page that runs the direction check,
  Bench Tests' station 13, since its sixth and last (built 2026-10-30; its logic host-tested, the page
  itself never run on a robot).
- **Deliberately absent:** any concrete mechanism. There is no `shulib::Intake` and never
  will be — mechanism sets change every season, so the library ships the grammar and your
  team writes the nouns ([Chapter 13](13-extending-the-library.md) says how, and why the safe state is declared per
  mechanism).
- **Now real at the device edge, still host-proven only:** the sensors that *confirm* a
  mechanism action — distance, color/proximity, limit switches — and the solenoid lines that
  act, all have PROS adapters as of 2026-08-14, each converting exactly once at the edge and
  each documenting the trap it defuses (a distance sensor reports "no object" as an in-band
  plausible-looking number; constructing a digital-out physically drives the line — the FAQ
  covers both). Proven against the programmable PROS stand-in; never against a physical
  sensor.
- **Not proven, stated plainly:** anything on hardware. No solenoid has fired, no motor has
  jammed for real, no distance sensor has seen a real game piece, and whether `Hold` truly
  holds a *loaded* lift is a registered assumption (HA-92), not a fact. Nor is it known whether
  the brain keeps a 3-wire output's level while it is disabled, or acts on a level set while it
  is disabled (HA-157) — a pneumatic claw on a driver's toggle button relies on the first,
  because driver control commands the claw nothing on a disable, and its start level on a field
  relies on the second (so does the safe level the competition program's run guard writes when a
  disable lands DURING a guarded auton — [Chapter 17](17-the-competition-program.md)) — and nothing the program can read is believed to see a valve on a
  3-wire port at all (HA-158, reasoned), so an unplugged solenoid is invisible to every check.
  The stall thresholds
  an operation needs are required parameters
  precisely because no sound default exists before hardware characterization (phase R4).
  And the scoring verbs themselves — `intakeUntilCapture`, `liftToLevel`,
  `setQuadrantToggle` — remain future work (roadmap item F3) that needs both hardware and
  the build team's final mechanism decisions.

What stays open in the recipe layer: **`then()`** and the mechanism seam are still
**unfrozen**, deliberately — the seam gets its second real consumer at F3 (the concrete scoring
verbs), and this project freezes surfaces after a second consumer has stressed them, not before.
And, since the mechanism core's fourth part, whether **the mechanism steps** (`r.move`, `r.run`,
`r.set`, `r.spinUp`, `r.home`, `r.stop`, the START forms and `waitUntilDone`), **the second
constructor** `Routine r{ctx, "name"}` and **`lastMechanism()`** join F10's freeze is **not yet
ruled** — the register's F10 row gives both readings, and they are not pinned meanwhile. The
partway triggers that lead to the START forms — **`atPercent`, `afterDistance`, `afterTurning`** and
the two handles they return — are unpinned in the same way (and ride only the six drive and turn
legs; `strafeTo` and `followTrajectory` take no trigger yet). An auton that uses any of them may need
an edit if their spelling moves before that ruling.
The [Freeze Register](../roadmap.md#freeze-register) remains the authority on what is and
isn't stable (the coordinate conventions, units, accuracy targets, hardware interfaces,
kinematics contract, both API tiers but for the pieces named above and — since 2026-09-18 —
the route data schema and command-id vocabulary *are* locked; the mechanism seam is listed there
as explicitly not-yet-frozen).

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
  end with the caller's end action performed, judged against the host test plant's ground truth
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
  formula and the deadline-aware alternative). One exception, since the mechanism core's
  second part: a wait on a mechanism operation that registered its claim ends one pace after
  the deadline, because the guard cuts the OPERATION it waits on ([Chapter 9](09-the-recipe-api.md), the same
  section).
- **"Zero travel after the deadline" is a host-test-plant result, not a robot one.** The host
  measurement is exactly 0.0 inches, and it is real — but it holds because the guard
  refuses the next command *before* the world advances **and** because the simulated
  drivetrain is memoryless: it stops the instant the voltage does. A real robot has mass.
  The on-robot claim is **"no new commanded motion after the deadline"** — the
  coasting and braking distance already in the wheels is physics, and this plant cannot
  produce it. Budget your lead time for a robot that is still moving when the command
  stops.
  There is a second difference that matters more on a field than it does here: **on a real
  brain, time passes whether or not your code calls into shulib.** On the host test plant the
  clock only advances when the pacer steps it, so the deadline is never "missed" while
  something else runs. On hardware, both the cut and the hard floor can only fire when your
  code next re-enters the library — which is the same limit as the bullet above, seen from
  the other side.

And its deliberate refusals, which are design, not gaps: no default match length, no
default lead time, no park pose, no field coordinate — a guard with a built-in notion of
"the endgame" would be one team's strategy frozen into everyone's library. Both instants
and the action are yours.

## The wall reset: what it cannot catch

The wall reset ([Chapter 10](10-the-api.md#the-wall-reset--one-axis-against-a-field-wall)) re-seats
ONE coordinate from a side distance sensor parked beside a field wall. It is **host- and build-gated,
NOT run on a robot**: every number below comes from the host test plant or from paper, and no
distance-sensor reading of the team's robot exists yet. The library has the explicit call, the robot
file's side sensors and the everyday call an auton writes
([Chapter 3](03-knowing-where-you-are.md#re-seating-one-axis-against-a-wall)); robot two's robot file
does not declare its side sensors yet, so on robot two a FIELD-start auton's reset refuses `no sensor`
until it does (and a RELATIVE-start one refuses `not a field start` first, as it does on every robot).

- **Something taller than a game piece, at a plausible distance, passes every check.** No reading
  check can tell it from the wall, and no document should say one can. On the host test plant: a
  Loader read as the wall with the keep-outs switched off seated y **3.7 in wrong**; a sensor mounted
  (or wired) to face the other way, with a robot 5 in to that side, seated x **2.0 in wrong** at a 7-in
  parking (the parking distance less the 5 in); an object 1 or 2 in behind a CLEAR wall panel seated
  ~1 / ~2 in wrong. A nested stack against the wall would do the same — and so would the Toggle's
  body, where it overhangs the wall's inner face above the wall top, for a lens high enough that its
  beam's upper edge reaches it: which is why that body's keep-out lifts only for a sensor SEEN to read
  the wall beneath a real Toggle (`toggleChecked`), and never by a typed height (the overhang's size
  is read off a drawing by eye; no wrong-seat distance for it is pinned on the host test plant). The
  defences are yours: a **tight `maxCorrection`** (how far the robot can really have drifted since its
  last reset — never near the gap to anything that could be mistaken for the wall), parking where the
  route says, the season's keep-outs, the sensor's height, and checking each side's port with a board
  held to ONE side.
- **A single standing Pin or Cup is caught — if the typed height and beam spread are true.** The
  beam-height check refuses any reading the beam's lower edge could take from a 6.5-in piece; it
  rests on the mount's typed height and on a beam spread of ±18° that is unofficial until it is
  measured. Never type a height "so the reset works".
- **No heading.** One sensor per side gives ONE range — the axis and the angle are two unknowns.
  The IMU owns the heading; no sensor ever seats it.
- **No background mode.** The reset runs only where an auton asks for it. A "correct whenever a wall
  is in view" mode would run where nobody chose — beside a partner robot, a piece against the wall —
  and every wrong seat the host test plant produced passed every reading check.
- **Clear wall panels.** A VEX Forum report (seen only through search summaries) says the sensor
  sees through the metal perimeter's clear panels, or drops out beyond ~700 mm. Then the reset
  usually REFUSES — but it **seats wrong by up to `maxCorrection`** when something opaque stands
  within that distance behind the panel: the wall's own frame, a drive-team member's hand. Check
  the perimeter at your event.
- **The GPS code strip.** VEX U and skills fields must carry it: black and white squares on the wall
  at about 9.1–11.4 in — exactly where a useful sensor's beam lands. Whether a black square reads like a
  white one is not published; measure your sensor against both.
- **VEX U Load Zones are always kept out.** Match loads come in through the Loaders during the
  autonomous period too, so a robot taking loads, or a stack it drops, may be in a Load Zone (23.11 in
  from each corner, along both walls) at any time — taller than the beam-height check assumes. With
  the beam's spread unmeasured the start pieces are kept out as well, so the clean stretch is narrow:
  about 2 × 8 in per wall for the beam's centre at a 7-in reading right after a seed, and none after
  ~48 in of driving (the keep-outs widen by 10 % of the travel since that along-wall coordinate was
  set); with a measured spread, about 2 × 23.6 in. Reset early in a leg, or reset the perpendicular
  axis first.
- **The precision is just inside its own limit.** With VEX's published accuracy (the larger of 15 mm
  and 5 %) and a mount measured to ±0.25 in, a sensor at the robot's fore-aft centre answers to
  0.86–0.97 in at 4–10 in of reading before any turning (square to the wall, up to 5° off) — inside the
  1.0-in limit. After 180° of turning it still passes parked square (0.93–0.98 in), but the margin is
  gone: it is refused `imprecise` from about 2° off square at a 7-in reading (from 0.5° at 10 in, 3° at
  4 in); after 450°, refused. Mounted 5 in forward of the centre, it is refused from the first reset
  (1.01–1.15 in); a mount measured to ±0.1 in and a measured accuracy of ±0.3 in bring that sensor to
  0.56–0.69 in. (Computed with the library's own error bound; nothing measured.) Standing still does
  not use the margin up: the rotation counted is the IMU's heading in 1° steps, so its jitter does not
  add up — at the jitter the library believes (HA-21). An IMU more than about three times that noisy
  starts counting it, steeply (up to ~100° a still minute at five times, on the host test plant —
  HA-211); that only refuses more resets, never seats a wrong one, and nothing has measured a real
  IMU's still jitter yet.
- **The explicit call's window cannot be cut.** It samples inside a wait the run guard cannot
  interrupt, so a deadline or a disable that lands in the window is acted on up to `windowMax`
  (0.5 s) late. The everyday call an auton writes samples through the run guard's own wait instead, and
  is cut at once (`cut`, nothing seated).
- **Only a FIELD start.** The everyday call refuses an auton that starts RELATIVE (or with no start
  stated): the season's walls are field coordinates. The explicit call takes a `Chassis` and cannot see
  the start, so under a relative estimate its author must use an own wall line with its own keep-outs —
  or none.
- **Every number is a belief** until measured — the sensor's accuracy, its beam's spread and how it
  answers two surfaces at once, whether the wall answers at all at that height, its update rate, the
  gate defaults (and the steps the Localizer counts turning and travel in), the field's dimensions
  (the two legal perimeters differ by 0.045 in a wall): the
  [hardware-assumptions register](../hardware-assumptions.md), HA-204 to HA-209, HA-211 and HA-212.

## The camera reads colours and pieces — on a host model, and with limits it states

Since the camera's second part the library turns the AI Vision camera's frames into a Toggle's colour and
one named piece ([chapter 20](20-the-camera.md)). What that is worth, and what it cannot do:

- **No AI Vision sensor has run this code.** Every number in [chapter 20](20-the-camera.md) is the host test plant's camera
  model looking through a true camera set inside the stated tolerances; the optics, the frame rate, the
  picture's age, the read's cost, the model's class names, the colours under event light and the
  Toggle's real shape are all unmeasured beliefs (HA-192 to HA-203).
- **A colour is the face the camera SEES, never a "set" Toggle.** A Toggle stuck mid-roll can show the
  colour it is rolling toward, and nothing the camera measures tells seated from rolled or touched from
  untouched. Anything that pushes a Toggle needs the colour confirmed at the push point by a sensor that
  sees colour there.
- **Lookalikes.** With the position gate off, a red band of the wrong size near a wall can read as a face
  (70 of 540 on the host plant), and so can a red LED strip on the wall top from 20 in and closer — the
  position gate, with a pose the team trusts, turns those away. It cannot turn away a red strip about a
  Toggle's length lying ON a Toggle's face line: that reads as the face from about 22 in out, gate or no
  gate.
- **The bottom face is told apart only if the face above it is SEEN.** If the field face's detection
  vanished on every read (glare) while its bottom face's fired, the bottom face's colour could read as sure;
  the bench checks the real camera never misses the face that way.
- **A frozen camera is caught only while the camera moves.** Parked, a frozen camera and a still scene
  look the same.
- **Close in, only square.** At 14–20 in, turned even 5 degrees, the colour is UNKNOWN (an 11-in lens
  keeps it at 5 degrees one way at 20 in); at 30 in an 8-in lens still needs square; nowhere past 15
  degrees — never a wrong colour, but no colour either.
- **Where the camera sits decides what it sees.** A lens about level with the face's lower edge (11¾–12¾
  in) is blind to the colour at most distances.
- **The picture's age is unmeasured**, so a field position from the camera is a bound only with the robot
  stopped until the bench measures it.
- **A piece is FOUND only close in.** With a mount measured to ±1 degree, the middle of three Pins a foot
  apart is FOUND (within 6 in) only from about 22–23 in; from a high lens far out, a Pin on a Goal
  cannot be told from one on the floor, and close in a Pin lying toward the camera cannot be told from a
  standing one, so both are UNSURE (never taken for a floor piece).
- **A field position near the picture's side can be a hair short.** For a Pin about 30 degrees off the
  axis and close in, from a near-exactly measured camera, the host plant showed the position missing the
  Pin by 0.02 in past its stated radius (the box's centre column belongs to the Pin's top). The finder's
  arithmetic for it is not fixed yet; nothing uses the finder before drive-to-piece.
- **A field position is only as good as a box's edges.** It assumes each reported edge is within 2 px of
  the piece's true edge, rounding included (HA-202); edges farther off than that (on the host plant, 2 px
  of jitter plus rounding) made standing Pins' positions miss by up to 1.8 in.
- **No tags.** The camera's AprilTags stay off; no camera has been pointed at a tag by this project.

## The Toggle flip in a skills route: what it cannot catch

The Toggle step ([Chapter 20](20-the-camera.md#checking-and-flipping-a-toggle-in-a-skills-route)) looks at a
Toggle with the front camera, flips it only when it is not its Quadrant's colour, and looks again. It is
**host- and build-gated, NOT run on a robot**: no Toggle, no flipper and no camera reading of a real Toggle
exist in this project yet, and every number below is the host test plant's. A team auton calls it through the
auton's context, with the camera the robot file's camera block and Toggle block describe — neither of the team's
robot files has a camera block yet; the plan for the first competition is still the team's own blind two-push
flip, until this step has flipped a real Toggle.

- **"FLIPPED" is not "set".** The camera cannot see whether a Toggle has SETTLED flat. One stuck part-way
  through its roll shows the face it is rolling TO, and reads that colour sure — on the host test plant a
  Toggle jammed 50 or 100 degrees into its roll toward ours read ours, the step said FLIPPED, and the game
  counted it yellow (the test pins this). So every success line ends `seating not checked`, and a stroke that
  stopped short and then reads ours is a failure. Whether every full stroke seats the Toggle is checked by eye
  at the bench (A4 register HA-232) — nothing on the field can.
- **A frozen camera, parked.** A camera that keeps answering with its last picture reads exactly like a still
  scene while the robot stands still — so the look BEFORE, parked at the look spot, can read a stale picture
  as sure. The step's own driving is what shows it: on the drive in the lens moves and the picture does not, so
  the step stops there and pushes nothing (`FROZEN BEFORE PUSH`), once the unmoved picture is newer than the
  camera's lag (`maxImageAge`) can explain — with the camera's position check on as well (that check alone
  would drop the frozen picture's band as "no band"; the frozen check reads the camera without it). If the
  drive in would reach the Toggle first (a `maxImageAge` longer than the rest of the drive), the step holds
  before anything touches the Toggle until it can tell (at most `maxImageAge`, counted in the time check). And it never accepts the look after from where it pushed: it backs
  out first and counts only pictures from after the retract, once the camera has moved. Never remove the
  back-off "to save time". What it still cannot catch: a stale picture that says the Toggle is ALREADY ours —
  nothing is driven, so nothing shows the camera frozen (the Toggle is left as it truly is, unflipped); and,
  for a PASSIVE bar, a Toggle the bar reaches before the camera has moved `frozenMoveIn` on the drive in
  (before a frozen camera CAN look frozen): the bar strokes it on the stale look, and the line still says
  `FROZEN BEFORE PUSH ... push 1 not made` (on the host test plant, a 16-in look spot and a 2-in
  `frozenMoveIn`: a bar touching from 12 in is stopped in time, one touching from 14 in is not). Keep a
  passive bar's look spot well back of where it first touches the Toggle.
- **A lookalike.** A coloured band the camera's checks cannot tell from a Toggle's face is read as one (A4
  register HA-199). On the host test plant, with the Toggle's own faces unseen, a blue plate at the face's
  height 4 in in front of the wall made the step push a Toggle that already showed ours — with the camera's
  position check on as well as off (its tolerance admitted 4 in); the same plate 1.4 in lower was refused as
  not at the spot. The position check, with a pose the team trusts (a FIELD start, right after a wall reset),
  turns most lookalikes away; none that sits where a Toggle's face is. Under a RELATIVE start it cannot run at
  all: the step drops the bound (the estimate is not in the field's frame) and says so in its line.
- **Our other robot, mid-visit.** Between the look and the push the camera, at most mounts, cannot name a
  colour (it is too close; a high, tilted lens can). If our other robot flips the Toggle in that moment, the step
  pushes it anyway — the route keeps the other robot away from a Toggle being visited.
- **A pneumatic flipper is not sensed.** A cylinder's step reports success once its valve has held the
  commanded position for its move time — nothing measures where the rod went. So for a pneumatic flipper "a
  full stroke" and "back" mean "the move time passed": a cylinder blocked half-way by a jammed Toggle still
  counts as a full stroke, and one caught on the Toggle on the way back counts as back, and the robot drives
  out with it (on the host test plant both are pinned). `OURS AFTER A SHORT STROKE` and `FLIPPER STUCK` come
  only from a motor flipper. Watch a pneumatic flipper's strokes at the bench (A4 register HA-235).
- **The spot check is coarse.** The camera catches only gross arrival errors (a 10-in lens 16 in out: refused
  at 2 in too far, accepted at 1 in; a lens near the Toggle's height hardly ranges at all). Where the flipper
  lands is the route's driving (HA-244).
- **A robot touching the Toggle** after the step ends, and **the pieces it holds** (`<SG6>`: holding more than
  one Pin or one Cup, a robot may not touch a Toggle) are the route's — the library cannot see either.
- **A wrong side wall.** Which side wall's Toggle is a red Quadrant's is a reading of the manual's drawings
  (HA-243). Under a RELATIVE start a route that names the wrong Toggle flips it to the wrong colour, said only by
  `wall not checked (relative start)` in its line; a FIELD start checks the robot faces the named wall.

## A mirrored auton: what the mirror cannot carry

Robot two can run robot one's skills route mirrored across the field's diagonal
([Chapter 17](17-the-competition-program.md#two-robots-one-route-a-mirrored-auton)). Proven on the host test
plant only — robot two as robot one's twin has never run it — and three things it cannot do:

- **The field's Loaders and the robot's own sides do not mirror.** The Loaders sit only on the red and blue
  walls, so a step at a Loader has no mirror (and the red/left Load Zone's image is the blue one); and the
  ROBOT is one physical build — a part off its centre line lands on the other side of the reflected path
  (A4 register HA-259). The library cannot see either; the bench shows which steps need robot two's own
  numbers.
- **An exact half turn is not mirrored.** The library always takes an exact 180° turn counterclockwise, on
  both robots and any drivetrain — robot two sweeps the same way as robot one there, in a `turnTo` and in a
  `moveTo` / `driveTo` / trajectory leg whose heading changes by 180° (the log says so once, `[MIR]`). On the
  host test plant a turn's path differs by 0.15 in — write it as two 90° turns where the sweep matters; a TANK
  leg ENDS off the mirror by more the longer the leg — 5.7 in on a 16 in leg, 14.2 in on a 36 in leg (a tank
  moves only along its heading) — turn first, in two 90° turns, then drive.
- **A pure mirror cannot take what lies ON the diagonal.** The centre goal, two yellow Pins and the red/right
  Load Zone are their own mirror images: both robots would reach them at the same moment. One robot takes
  them (`isMirrored(ctx)`), and the suite's pair check holds the robots 6 in apart only at the PLANNED timing —
  a real robot's timing drifts (A4 register HA-260).

## No easier tiers yet

The project's [accessibility model](../shulib-v2-master-plan.md#17-accessibility--progressive-disclosure-for-teams-that-cant-code-yet)
promises four tiers of use. Today Tiers 2 and 3 exist — the recipe layer
([Chapter 9](09-the-recipe-api.md)) and the full C++ API — but the easier tiers don't:

- **No zero-code authoring yet** (Tiers 0–1: build the robot and drag waypoints in the
  VexBuilder app, run the saved file with no C++). Half of it exists: a routine can be DATA — the
  path runner, the command-id registry and the manifest a path editor reads shipped on
  2026-09-18, with the route schema frozen — but nothing loads a route or a robot from a
  `.vexbot` file yet, so today a route is still typed in C++. The file reader is the rest of
  roadmap milestone M5, coordinated with the separate VexBuilder project.
  The [route builder](22-making-your-first-route.md) (2026-11) is the first piece a non-coder can use: it builds the
  skills route by clicking on the field and WRITES the commented C++ into the skills-route file — the team still
  builds and uploads it. What it does not do yet: **PREVIEW** (playing the route from the real library on the
  host test plant), triggers partway through a drive, wall resets, the mirrored robot drawn on the field, a time
  bar, the Toggle steps, strafing, moving to a pose, driving through points, sensor waits, flywheel steps, a step of
  your own C++, relative starts, 30 s match routes, and exporting a route to VexBuilder. Until they come, they are
  typed by hand inside the route's region; the route builder never overwrites a hand edit there without asking.
- **No ready-made scoring verbs for recipes to command** — the mechanism layer exists (see
  above) and `then()` runs its operations, but the mechanisms themselves are structs your
  team writes ([Chapter 13](13-extending-the-library.md)), and the season's concrete
  primitives (`intakeUntilCapture`, `liftToLevel`, …) are roadmap item F3.
- **The published reference lags the code, by construction.** The
  [cookbook](../cookbook/README.md) and the generated API reference (`docs/api/`) both exist,
  are build-checked, and *are* now published to the team site. But the site publishes from the
  release branch, not the working one, so it shows the last release rather than the current
  tree — deliberately, because that is what keeps the development log off the public site. Read
  the in-repo copies if you need today's truth.
  *(This bullet used to say no cookbook existed at all; that was written before D3 shipped
  it and went stale — corrected at F1. It then said nothing was hosted anywhere, which went
  stale when the site went up — corrected then. It said "HTTPS is still pending a
  certificate" until 2026-09-24; the certificate was issued and HTTPS enforced on 2026-08-14.)*

## Motion-quality boundaries

Deliberate v1 boundaries, documented where they bind (each is on the roadmap or the
[master plan's frontier list](../shulib-v2-master-plan.md#15-the-one-stop-shop-capability-catalog-past--present--future)):

- **Curves exist; rolling through steps does not.** Since chunk CURVE1, `curveTo` /
  `reverseCurveTo` drive one smooth path through up to 16 points without stopping at them
  ([Chapter 5](05-getting-there.md#flowing-through-points-curves)) — pure pursuit, proven on the
  host test plant only. A curve still STOPS at its end, and two steps in a row (two curves, a curve
  then a `driveTo`) stop between them: rolling from one step into the next at speed ("motion
  chaining") is not offered. `followTrajectory` still settles at every waypoint
  ([Chapter 10](10-the-api.md)); measured cost of a stop: about 1.2 s per motion. No timed,
  profiled trajectory (RAMSETE and the like) either — it needs the drive's measured speed constants.
- **A tank cannot remove a sideways miss in a short distance** — not with `moveTo`, and not with a
  curve started too close: a curve needs about 2½ ft of run-up along the end's line for a miss up
  to 6 in, 3 ft up to 1 ft ([Chapter 5's run-up rule](05-getting-there.md#a-sideways-miss-what-a-curve-fixes-and-from-how-far)).
  The library does not back up and retry by itself: it cannot see behind the robot.
- **No motion profiles on the main verbs yet** (trapezoidal velocity planning exists in
  `control/` but isn't wired into the drive's `moveTo`; a mechanism's moves use it since the
  mechanism core's third part — [chapter 18](18-mechanisms.md)).
- **Tank steering exists, and has only been proven on the host test plant.** Since API 2.3 a
  tank `moveTo` steers back onto its line ([Chapter 4](04-drivetrains.md)): its heading bends at
  most 15° toward the line, the turn a weaker side adds is cancelled at most 1.5 rad/s, and only
  a leg that starts within 15° of its target heading is steered. Its limits, as measured on the
  host test plant (2026-09-27, re-measured 2026-09-28 after a review changed how the bend is
  computed):
  - **Motor lag moves it.** The plant's motors respond instantly; given a lag the controller
    does not know about, the margins move. With a motor time constant of about 0.12 s or 0.18 s
    the 40 in leg still finished with one side up to 25 % weak (30 % timed out with and without
    lag). A target 6 in off your heading over a 40 in leg was steered out with and without that
    lag; 10 in was not (it stopped 1.6 in off the line with 0.12 s of lag, 3.1 in without), and
    a short leg removes much less (over 20 in, about an inch with no lag). **Do not plan on a
    sideways offset being removed** — face the point. The robot's real lag is measured at R5.
  - **With a long lag, a few legs that arrived without it time out with it.** Up to about
    0.18 s of lag, no leg on the grid measured (24 563 legs starting up to 14° off their target
    heading) that arrives with steering switched off times out with it on. At about 0.24 s, 24 of
    the 1 867 that arrive do — legs 48 in long starting 10–14° off their heading — because at
    that lag the heading loop itself is at its limit; R5's measured tuning is for that.
  - **A skid it cannot see.** It steers on the pose *estimate*. On a robot whose position comes
    from its drive motors and IMU, a sideways slide of the chassis is invisible (the IMU sees
    heading, the wheels see rolling), so the screen can say FINISHED while the robot is off its
    line by the slide. Registered as HA-180 in the
    [Hardware Assumptions Register](../hardware-assumptions.md); only a GPS, a tracking wheel or
    a wall reset would see it.
  - **A pinned robot winds it up.** While something stops the robot turning — a defender, a wall
    — the correction builds up to its 1.5 rad/s limit, and when the robot is freed it can
    overshoot: a 6 in offset leg pinned mid-bend swung 16–19° off its heading on release (on the
    host test plant it still arrived). Bounded, visible (`TSTR`), and registered with its
    constants as HA-179.
  - **It does nothing for a point beside you**, for `strafeTo` (which holds its heading by
    promise), or for a leg that starts more than 15° off its heading (that one logs
    `turn first: face()`).
  - Setting `tankSteer.maxSteer` to 0 in the motion config restores the old tank exactly — its
    motion, its records, its log — except the result line's `xt` column, which stays
    ([Chapter 11](11-reading-the-diagnostics.md#per-motion-result-lines)).
- **`drive()` is a primitive, and the driver-control layer above it has never been driven.**
  The shaping now exists: `shulib/teleop/` carries a continuous deadband, a monotonic expo
  curve, an optional rate limit that is **off** by default, arcade / tank / field-centric
  modes, and a `DriveShaper` that composes them in one call per tick — all host-tested,
  including a test that pins a stick step reaching the commanded motor volts in the same tick.
  Two things are still missing, and the second is the one that matters. **First**, nothing that
  uses it has run on a robot: robot two's driver file (`src/team/robot_two_driver.hpp`) drives
  through `teleop::runDriverControl` with the default profile, and "shulib Drive"
  (`src/programs/drive_program.cpp`) runs that file — a program that has not run on a robot yet. The
  bench tester's drive station keeps the older raw mapping (`teleop/stick_mapping.hpp`) — raw volts,
  the baseline a driver can compare the library's feel against, wheels up; nobody has yet. The old
  "shulib Drive"'s protections are no longer that program's alone: since T4 Part 2 the loop drops a
  drive motor that stops answering mid-run, stops the drive once more than two are dead, and cuts it
  for 1 s on a fight or an over-current ([Chapter 10](10-the-api.md)) — host-tested only, every
  number invented (HA-132, HA-175, HA-176), and no push, full-stick reversal or launch has ever been
  measured against the 2.4 A limit. One case it gets wrong is known: at a wall beside a drive motor
  whose gear slips (robot two's port 18), live motors can be marked dead, and a second push can then
  stop the drive; at any other wall, a motor whose reported speed settles well before its mates'
  costs one 1 s cut instead (HA-175's known gaps; [Chapter 12](12-when-things-go-wrong.md)).
  **Second, no driver has felt any of it.** Every default in the feel layer is invented and registered as a guess
  (HA-112) — which deadband, which curve, which slew rate a driver actually prefers is a
  question only a person on a robot can answer, and nobody has been asked. Treat the defaults
  as a starting point, not a recommendation. Mechanism BUTTONS exist since chunk T3
  (`teleop::MechanismBinding`: hold a button to drive a mechanism, let go and it goes to its safe
  state; `teleop::ToggleBinding`: press a button to open a pneumatic claw, press again to close it
  — [Chapter 10](10-the-api.md)), and robot two's driver file gave robot two's claws to the
  driver with them (the claws since T3's second part, in the driver file since T4, until the
  mechanism core's fifth part moved them onto the core's controls — below; "shulib Drive" runs
  that file): R1/R2 move the claw pivot on port 6, L1/L2 the one on port 7
  (full power while held, holding on release), A opens and closes the pneumatic claw, and the
  field-centric re-zero moved to Y. **None of it has run on a robot**: no claw has moved, whether a
  released pivot really holds its load, which way each one turns and which line level opens the
  claw are all unmeasured, and no driver has used a claw button. Since the mechanism core's fourth
  part (2026-10-29) a robot's DECLARED mechanisms have their own driver controls — `whileHeld`,
  `toggle`, `preset` (an arm to its "score" on one button), the homing controls, "the most recent input
  wins" ([Chapter 18](18-mechanisms.md#on-the-drivers-buttons--the-driver-files-controls)) — host-tested
  only; robot two's driver file runs its claws on them since the core's fifth part (2026-10-29),
  behaving exactly as before. Driver MACROS (a
  button that runs a chassis motion the sticks can interrupt) are still future work (phase T on the
  roadmap).
  *(This bullet used to say joystick shaping and slew limits did not exist at all; that went
  stale when the layer landed, and it is corrected here rather than deleted, because "it
  exists" and "it is known to feel right" are very different claims.)*
- **No *scoring* layer** — the concrete season verbs (`intakeUntilCapture`, `liftToLevel`,
  match-load sequences) are roadmap item F3/M4 and do not exist. The mechanism *seam* they would
  be written against does exist and is described above; what you write today is your own
  mechanism structs against a real seam, not from nothing. (This bullet used to say the
  mechanism layer itself was missing, which contradicted the section three headings up.)
- **Braking physics are unverified**: brake mode is commanded correctly, but what a real V5
  brake does to a moving robot is a registered assumption (HA-53). Plan conservatively around
  stopping distances.

## Diagnostics boundaries

The terminal transcript ([Chapter 11](11-reading-the-diagnostics.md)) is real and byte-pinned.

**The SD-card blackbox is built** — this bullet used to list it as missing and was wrong. A run
records to the card without a laptop: a RAM flight recorder that costs the card nothing until a
fault fires, then writes the triage block first and the preceding ticks after it (in that order,
because the fault may be the brownout that cuts the write short), and a decoder that ships in the
same chunk and is held to the format by byte-exact goldens rather than by agreeing with the
encoder. The controller-screen fault display is built too. **Both are host-proven only:** the
V5 card adapter under the blackbox HAS written a file to a real SD card at the bench (2026-08-18;
the file was read back on a laptop), but the blackbox itself has never recorded a run on a robot, so "it records without
a laptop" is a property of the design and the host tests, not something any robot has yet
demonstrated.

Still genuinely not built, per the [diagnostics plan](../diagnostics-plan.md): the live telemetry
wire to VexBuilder, record/replay-as-regression-test, and the on-brain status screen (the
*brain's* screen — distinct from the controller screen, which exists). Live watching still needs
a USB tether or the host test plant.

## And the meta-limitation

This guide itself has never been used by its intended reader. It was written alongside the
library, checked against the code (every example compiles and runs in
[`test/guide_examples_test.cpp`](../../test/guide_examples_test.cpp)), but no first-year has yet
learned from it. When you — the actual new member reading this — find the sentence that stopped
you, that's a guide bug: report it ([maintenance README](README.md)).

---

*Next: [Chapter 15 — Glossary](15-glossary.md)*
