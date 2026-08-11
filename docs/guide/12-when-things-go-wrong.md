# 12 — When things go wrong

> **Covers:** symptom-first troubleshooting — "the robot did X, why?" — with, for each symptom:
> the likely causes, how to confirm from the diagnostics, and what to change.
> **Read this if:** something just went wrong. That's what it's for.
> **Assumes:** [Chapter 11](11-reading-the-diagnostics.md) — you'll be asked to read
> transcripts. Concepts from Chapters 2–6 are referenced by name.

Before the symptoms, the one master question. Almost every motion mystery reduces to:

> **Is this a *motion* problem (the robot can't get where it thinks it should go) or an
> *estimate* problem (the robot doesn't know where it is)?**

The single most reliable discriminator: **does the transcript's story match what your eyes
saw?** If the log says `✓SETTLED final( 24.1, 36.0, …)` and the robot is visibly a foot from
there — the estimate is wrong (the robot faithfully drove to a place that wasn't where it
thought). If the log itself shows trouble — timeouts, oscillating error — the motion layer is
struggling honestly. Estimate problems: Chapters [2](02-the-field-and-coordinates.md)/[3](03-knowing-where-you-are.md)
territory. Motion problems: [Chapter 5](05-getting-there.md) territory. Route yourself first;
half of all debugging time is wasted on the wrong half of this fork.

A scoping note, honestly: nobody has field-debugged this library yet — there is no robot. The
simulation symptoms below are exercised constantly by the test suite; the hardware symptoms are
*designed for* and reasoned, but the specific advice will get sharper once reality has had its
say. Where a diagnosis depends on an unverified assumption, the
[Hardware Assumptions Register](../hardware-assumptions.md) is cited.

---

## "The robot doesn't go where I told it"

*It moves confidently, arrives somewhere — the wrong somewhere.*

**Likely, in order:**

1. **Wrong starting pose.** The `setPose` at routine start doesn't match where the robot was
   physically placed. Every subsequent target is offset by exactly the placement error.
   *Confirm:* the error is roughly **constant** across the whole run (every leg lands offset by
   the same amount and direction). *Fix:* measure the start placement carefully; make the
   `setPose` value and the field setup procedure agree — write both down.
2. **A frame or heading confusion** ([Chapter 2](02-the-field-and-coordinates.md)). *Confirm:*
   the miss **rotates with the robot** — legs driven at heading 0° land fine, legs at 90° miss
   sideways; or all motion is consistently rotated by some angle. Check any `drive()` calls'
   `Frame::` argument against your intent. *Fix:* it's a thinking error, not a tuning error —
   re-read the field-vs-robot section and re-derive what you meant.
3. **The estimate drifted or was knocked** ([Chapter 3](03-knowing-where-you-are.md)).
   *Confirm:* early legs land true, later legs miss progressively (drift); or the story is fine
   until one violent moment — look for a collision around a specific timestamp, `flt=` flags,
   or an `ODO_STUCK`/`IMU_LOST` in the ledger. *Fix (today, without correctors):* shorten the
   error chain — put accuracy-critical actions earlier; recheck tracking-wheel hardware
   (spinning freely? spring-loaded against the floor? cable seated?).
4. **You and the robot disagree about which point "position" means** — the tracking center vs.
   the intake ([Chapter 2](02-the-field-and-coordinates.md)'s last section). *Confirm:* misses
   are consistent and roughly half a robot in size. *Fix:* adjust targets to be tracking-center
   targets.

## "It stops early / gives up"

*A leg ends before reaching the target.*

**Read the result line first — it names the ending:**

- **`✗TIMEOUT`** — the watchdog fired. Now, the differential (this is Chapter 11's "a symptom
  with many diseases"):
  - *Budget simply too small:* the `final(…)` pose is well on the way and error was still
    shrinking. Fix: raise `timeoutSeconds` — and remember every leg pays ~a second of settle
    time on top of travel; a 2.0 s budget for a 1.9 s leg is a coin flip.
  - *Physically blocked / jammed:* error stopped shrinking at a constant value partway. In sim:
    an unreachable target. On a robot: a wall, a defender, a game piece under the chassis.
  - *Unreachable by drivetrain:* a sideways target on tank ([Chapter 4](04-drivetrains.md)).
    Fix: the turn-then-drive idiom ([Chapter 10](10-the-api.md), `guide-10b`).
  - *Waiting for sensors the whole time:* tick lines show `▸1` (waiting-for-estimate) for the
    entire duration — the estimate never went live. A sensor didn't boot; on hardware, check
    the IMU.
  - *Almost settled, never quite:* error hovers just outside tolerance, or bounces through
    zero. That's a tuning/settling story — see "it oscillates" below.
- **`✗FAULT_ABORT=CODE`** — the fault policy stopped it deliberately. The code (almost always
  `ODO_STUCK`) is the story; see its row in [Chapter 11](11-reading-the-diagnostics.md) and
  check the physical sensor.
- **`✗SUPERSEDED` / `✗CANCELLED`** — *your code* ended it: something called the next verb (or
  `cancel()`) early. Look at your routine's logic — a mis-nested `if`, a `waitUntil` that
  returned sooner than you assumed.

## "It oscillates / wobbles / vibrates"

*At the end of motions it rocks back and forth instead of stopping; or it buzzes continuously.*

This is [Chapter 5](05-getting-there.md)'s control-tuning material made audible:

- **Rocking around the target (overshoot each way):** control too aggressive for the robot —
  in PID terms, too much P (or too little D). *Confirm:* result lines show recurring `over`
  values; tick error crosses zero and comes back repeatedly. *Fix:* in sim, this points at a
  config regression (the shipped gains settle cleanly on the shipped plant — suspect a changed
  number; `git diff` the config). On future hardware: the gains were never tuned for reality —
  that's expected, and re-tuning is a planned hardware-phase activity, not a bug you patch
  blind. Either way, change one number at a time and re-run the same leg.
- **Constant fine buzzing:** the derivative term amplifying sensor noise, or a settle-rate
  tolerance set tighter than the noise floor (the config header documents this exact trap —
  see the settle notes in
  [`motion_config.hpp`](../../include/shulib/motion/motion_config.hpp)).
- **Never *quite* settles (then times out):** tolerance tighter than the estimate's noise:
  the robot is done, but "done" flickers. Loosen the settle tolerance a notch before touching
  gains.

## "The pose looks wrong" (and the robot may even know it)

*Printed poses / tick lines disagree with reality, or jump.*

- **Heading right, position wrong:** tracking-wheel path — slip, a lifted wheel, wrong wheel
  geometry constants. On hardware, roll the robot a measured 24 inches by hand and compare the
  estimate's delta; a percentage miss is a diameter/geometry constant, a gross miss is a dead
  sensor.
- **Position right-ish, heading wrong:** the IMU path — and heading error *becomes* position
  error as you drive ([Chapter 2](02-the-field-and-coordinates.md)), so catch it early. Look
  for `IMU_LOST`, or on hardware suspect drift/calibration
  (the register's HA-20 is the relevant open assumption — a raw V5 IMU is expected to drift
  toward the spec limit over a 60 s run until correction exists).
- **The estimate jumps:** a fusion correction landed (look for `CLMP`) — or something worse
  (`flt=NAN_POSE`, `IMPLAUSIBLE`). The never-snap rule means visible teleports should not
  happen; a real jump with no flags is a bug — capture the transcript and report it.
- **The pose froze:** `NAN_POSE`'s designed behavior is freezing position rather than
  corrupting it, and `ODO_STUCK` means it *should* have frozen. The transcript will say.

## "It won't build" (your code, not the robot)

- **"No viable conversion" / "no matching function" at a verb call:** you passed a bare number
  where a typed unit belongs. Write `24_in`, `90_deg`, `Velocity{20.0}` — and check you have
  `using namespace shulib::units::literals;`. This error is the type system doing its job
  ([Chapter 10](10-the-api.md)).
- **`drive()` "too few arguments":** the `Frame::` argument is mandatory, by design.
- **A wall of template errors mentioning `Quantity`:** usually an arithmetic mix of
  incompatible dimensions (adding a length to an angle). Find the line, ask what unit each
  operand is.
- **A new test file that the build ignores:** the build normally discovers `*_test.cpp` files
  automatically on the next compile; if yours isn't picked up, re-run the configure step
  (`cmake -S test -B build/test`, [Chapter 7](07-getting-set-up.md)).
- **A `PreconditionError` at *runtime*, immediately:** not a build problem — the library
  rejected nonsense input loudly (NaN target, negative timeout, empty waypoint list). The
  message names the rule; fix the caller.

## "The transcript itself looks wrong"

- **Result lines say `n/a`:** record stream not connected — expected in competition builds and
  in the tutorial's first wiring ([Chapter 8](08-your-first-routine.md)).
- **Lines missing:** check for `throttled …: dropped N lines` notices and the summary's
  `dropped` field (rate limiting), or a level filter you configured and forgot.
- **`build MISSING`:** the binary lost its identity stamp — Chapter 11's session-header note.

## When you're stuck

Capture the transcript (the whole thing), note what you *saw* the robot do, and bring both to
whoever maintains the motion stack. A transcript plus an eyewitness account is almost always
enough to find it — that's what all this diagnostic machinery is *for*. And if the investigation
uncovers a library bug: it gets a test that would have caught it, then the fix — in that order.
That's the house discipline ([Chapter 13](13-extending-the-library.md)), and it's why the bug
count stays down.

---

*Next: [Chapter 13 — Extending the library](13-extending-the-library.md)*
