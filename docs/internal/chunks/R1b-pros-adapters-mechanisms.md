# Chunk R1b — `hal/pros` adapters: the mechanism seams

> **The other half of R1.** Four adapters, one new seam, and the SD blackbox that E1 shipped an
> interface for and no implementation. R1a put the drivetrain on hardware; R1b puts the *mechanisms*
> there — which is what F3 needs and what R3 cannot walk the register without.
>
> **Status:** brief. Not started.
> **Predecessor:** R1a (built, verified, committed) + its bench session.
> **Live log:** `R1b-PROGRESS.md` — create it FIRST, append continuously.

---

## 1. Why this chunk is here

**R3 is blocked on it.** R3 walks the Hardware Assumptions Register top to bottom, and the register
contains entries only a distance sensor or an optical sensor can settle. R1a's bench session already
hit this: five runbook steps were unrunnable, two of them because *the hardware isn't on the robot*
and the rest because the seams don't exist.

**F3 is blocked on it.** Every scoring primitive in F3 — `intakeUntilCapture`, `clampConfirm`,
`setQuadrantToggle` — rests on the master plan's one non-negotiable rule: **task-sensor confirmation
on every grab and place, never advance on an unconfirmed action.** The sensors that confirm are
`IDistance` and `IOptical`, and the actuator that grabs is `IDigitalOut`. None has a PROS-backed
implementation.

And R1b is **mechanical repetition of a pattern R1a proved**: the two-flag diagnostic fence, the
path-anchored guard, the host shim, the pure-conversion split. R1a paid the structural cost; R1b
should be the cheap half. If it turns out not to be, that is a finding about R1a's pattern.

---

## 2. Scope

**In:**

| Seam | Backing | Notes |
|---|---|---|
| `IDistance` | `pros::Distance` | mm → inches; confidence 0–63 → [0,1]; **the 9999 trap, §5 T4** |
| `IOptical` | `pros::Optical` | hue and sat/bri pass through; proximity 0–255 → [0,1] |
| `IDigitalOut` | `pros::adi::DigitalOut` | F1's seam; **construction actuates, §5 T3** |
| **`IDigitalIn` — NEW SEAM** | `pros::adi::DigitalIn` | + fake; register row; **built on an open question, §5 T1** |
| `IBlockSink` | PROS `FILE*` on `/usd/` | E1 shipped the interface and named R1b as the owner |

Plus: new pure conversions (`distance_conversion.hpp`, `optical_conversion.hpp`), shim extensions
(`pros/distance.hpp`, `pros/optical.hpp`, `pros/adi.hpp`, `usd`), tests, the mutation campaign, and
the documentation contract including a **changelog entry** (deliverable #7).

**Out:**
- `IVision` / `ITagSource` — **R2's**, explicitly.
- Any homing routine, capture routine or scoring primitive — **F3's**, and season content the
  students author (the §16 guardrail). R1b ships seams and adapters and stops.
- Wiring these into `src/main.cpp` — `main.cpp` still carries R1a's invented X-drive-with-tracking-
  wheels configuration, which the bench session proved matches no robot we own. Re-wiring it is a
  chunk of its own and should not be smuggled in here.

---

## 3. What R1a established that this chunk inherits

Read these before writing a line; they are the pattern, and deviating from them needs a reason.

- **Adapters live at `include/shulib/hal/pros/`**, header-only.
- **The diagnostic fence is exactly two flags** — `-Wshadow`, `-Wsign-conversion` — pushed around the
  PROS include block and popped **before** any shulib code. `-Wconversion` stays live. R1a's negative
  control proved the fence does not protect our own code.
- **Include PROS with QUOTED form: `#include "pros/adi.hpp"`.** Not angle brackets. PROS's
  `common.mk` is `-iquote`-only (`common.mk:150`), so `<pros/...>` **cannot compile in the robot
  build** — invisible to `-I`-flag probes, found only by running `make`. R1a's own brief got this
  wrong.
- **The PROS-free guard is path-anchored** and already exempts `hal/pros/`. It needs no further
  amendment — but re-prove it bites, with a plant.
- **The ARM gate needs no amendment.** Its generated glob picks new headers up automatically.
- **The shim `#error`s without `SHULIB_HOST_PROS_SHIM`** and is `BEFORE`-shadowed in the host build
  only. Every new shim header must do the same. A shim that reaches a robot build produces a binary
  that boots, prints a healthy banner and drives nothing.
- **The shim's defaults are adversarial where the trap is real.** Follow that: a distance shim should
  default to returning 9999 (§5 T4), a DigitalIn shim's `get_new_press()` must really consume.

**And the seven measurements R1a's bench session settled** (HA-94…97, 99, 100, 101) are now
observations, not guesses. R1b's beliefs get the same treatment: **register every one as `HA-113`
onward**, and add a bench-runbook step for each.

---

## 4. The PROS surface, read from the vendored headers

| shulib wants | PROS gives | R1b owns |
|---|---|---|
| `IDistance::distance()` inches | `get_distance()` int32 **mm**, **9999 = "no object"** | ×0.0393701, **and the 9999 rule** |
| `IDistance::confidence()` [0,1] | `get_confidence()` int32 **0–63**, *"only available when distance > 200 mm"* | ÷63, **and the <200 mm rule** |
| `IOptical::hue()` [0,360) | `get_hue()` double, *"range of 0 to 359.999"* | identity |
| `IOptical::saturation()` [0,1] | `get_saturation()` double, *"range of 0 to 1.0"* | identity |
| `IOptical::brightness()` [0,1] | `get_brightness()` double, *"range of 0 to 1.0"* | identity |
| `IOptical::proximity()` [0,1] | `get_proximity()` int32, *"range of 0 to 255"* | ÷255 |
| `IDigitalOut::set(bool)` | `adi::DigitalOut::set_value(int32)` | bool → 1/0 |
| `IDigitalIn::state()` (new) | `adi::DigitalIn::get_value()` int32 | ≠0 → true |
| `IBlockSink::write(bytes)` | `std::fwrite` to a `FILE*` on `/usd/` | + `usd_is_installed()` |

**ADI addressing has two forms**, and both must be supported:
- `DigitalOut(std::uint8_t adi_port, bool init_state = LOW)` — the brain's own 8 ports, addressed
  `1–8` **or** `'a'–'h'` **or** `'A'–'H'`
- `DigitalOut(ext_adi_port_pair_t, bool)` — an expander, addressed as `{smart_port, adi_port}`

**Sentinels:** `PROS_ERR` = `INT32_MAX`, `PROS_ERR_F` = `INFINITY`, exactly as R1a.

---

## 5. The tensions to rule

### T1 — `IDigitalIn` is being built on an open question, not a consumer

**The lift-homing question is still unanswered** (asked 2026-08-13; the answer was *"not decided
yet"*). The ruling was: build the seam anyway — cheap now, expensive to discover at R3 with the robot
on the bench.

**This departs from F1's standard** and the brief says so rather than hiding it: `IMechanism` earned
its two virtual members on one verb a real consumer needed. `IDigitalIn` has **no consumer at all**.

Two things keep it defensible, and both are binding:

1. **The shape is derived, not guessed.** A digital input has one degree of freedom — `state()`.
   Every other question is already ruled by `IDigitalOut`'s header (`digital_out.hpp:37-44`,
   `:18-20`): what "pressed" means physically belongs to the mechanism that owns the line, and there
   is **no validity channel** — a dead ADI port is indistinguishable from a working one.
2. **R1b ships the seam, the adapter and the fake — and NO homing routine.** Homing is F3's.

**If the answer comes back "stall", this is a small unused sibling.** That cost is accepted and
stated. **Register row: NOT FROZEN**, said out loud (D2's lesson).

**Rejected:** waiting for the answer. R3 is the expensive place to discover a missing seam, and R3 is
two chunks away.

### T2 — Debounce belongs ABOVE the seam, not in the adapter

**Proposed ruling: the adapter reports the raw level; debouncing is the consumer's.**

Reasoning: a limit switch used for homing wants a different filter from a bumper used for collision
detection, and the adapter cannot know which it is. Baking a time constant into the seam invents a
constant nobody measured (and it would need an `HA-nn` immediately).

**But note the asymmetry with `IController`**, which *does* carry `ButtonEdge` above the seam — that
is edge *detection*, not debouncing, and it is stateless per consumer. Do the same here: if
new-press semantics are wanted, reuse `ButtonEdge` rather than minting a second mechanism.

**Rejected:** debouncing in the adapter with a configurable window — it puts an unmeasured constant
in the one layer that has no idea what the switch is for.

### T3 — Constructing a `DigitalOut` ACTUATES the line

`explicit DigitalOut(std::uint8_t adi_port, bool init_state = LOW)` — construction drives the
output. For a pneumatic clamp this means **the cylinder moves when the object is built.**

This collides with F1's discrete-actuator ruling, which was that a clamp's safe state is *not*
"open", because un-actuating on success would fling the game piece. If the adapter's construction
forces LOW and the mechanism's declared safe state is HIGH, there is a window at boot where the line
is wrong — and on a pneumatic, "wrong" means physically moving.

**Proposed ruling — confirm while writing the code:** make the initial state an **explicit, required
constructor argument** with no default, so the author must state what the line should be at boot,
and document that constructing the adapter is a physical action. `PneumaticMechanism` already
declares a safe state; the two must agree, and a test should pin that they do.

**Rejected:** defaulting to LOW and saying nothing — that is exactly the "a safety step a caller can
forget is a safety step that WILL be forgotten" lesson from the legacy `escapeJSONString` defect.

### T4 — The 9999 trap: an in-band magic number, not a sentinel

`get_distance()` **"will return 9999 if the sensor can not detect an object"**
(`include/pros/distance.hpp:98-99`). Not `PROS_ERR`. A plain integer that converts to **393.66
inches** — a perfectly plausible-looking reading.

And `get_confidence()` is **"only available when distance is > 200mm"**, so confidence does not
reliably report "no object" either.

**Proposed ruling:** the adapter maps 9999 to **`confidence() == 0`**, which is `IDistance`'s
existing contract for "no usable return" (`distance.hpp:28`) — callers already threshold on it. The
distance value itself must stay finite (the F4 finiteness contract). **This is the single most
important line in the chunk**: without it, dock-confirm logic gets a wall 33 feet away and calls it
real.

**Also rule** what confidence means below 200 mm, where PROS says it is meaningless. An object at
100 mm is *close*, which is exactly when a mechanism cares most. Do not silently report a confidence
PROS says is not available.

**Rejected:** treating 9999 as a fault. It is the sensor working correctly and reporting nothing in
range — that is a normal state, not an error, and raising a fault on it would cry wolf every tick the
intake is empty.

### T5 — `IBlockSink` on `/usd/`: what happens with no card

E1 built the blackbox around the fact that **an SD card that fills up, is yanked, or dies mid-write
is the NORMAL failure of a blackbox** — which is why `write()` returns `bool` and is `[[nodiscard]]`.

R1b must decide: **no card at boot** — does construction fail loudly, or does the sink accept-and-
discard? `usd_is_installed()` exists to tell you.

**Proposed ruling:** construction succeeds, `write()` returns **false** from the first call, and the
fact is visible once at construction through the diagnostics layer. Reasoning: a missing SD card
must not stop a robot from driving, and E1's drop-and-count design already handles a sink that
refuses. A precondition throw here would turn a missing card into a dead robot.

**Rejected:** silently succeeding — that reproduces the exact "nothing looks wrong" failure the whole
blackbox exists to avoid.

**Note the path quirk:** `usd_list_files()` documents *"DO NOT PREPEND YOUR PATHS WITH /usd/"* while
`fopen` requires the `/usd/` prefix. Two conventions in one API, like the port indexing R1a found.
Register it and write it in the FAQ.

### T6 — One adapter or two for built-in vs expander ADI?

Two constructors on one class, or two classes? **Proposed: one class, two constructors** — the seam
is identical and a caller should not pick a type based on where the wire goes.

**Verify at the bench**: the R1a session reported an ADI expander from an **out-of-range registry
index**, so whether this robot even has one is **unknown**. R1b must not assume it does. Add a
runbook step to settle it.

---

## 6. Test requirements

**Every test names, in a comment, the bug it would catch.** Mutations must be **executed** — build,
run, observe red, restore. **Gate the runner on build success** (C4). **Trap `PIPE`** (E2). And
**bump mtime on restore** — `cp -a` preserves it, so make skips the rebuild and the next run reads a
stale binary (found during R1a's review).

### Mutations, all mandatory

| # | Mutation | Must go |
|---|---|---|
| 1 | Drop the mm→inch scale in the distance adapter | RED |
| 2 | **Pass 9999 through as a real reading with nonzero confidence** (T4) | RED |
| 3 | Drop the confidence ÷63 | RED |
| 4 | Drop the proximity ÷255 | RED |
| 5 | Adapter reads the conversion but returns the raw value (the C5 D-5 / E1 wiring hole) | RED |
| 6 | Remove sentinel screening on one reader | RED |
| 7 | Invert `IDigitalOut::set()` | RED |
| 8 | Make `commanded()` report the world instead of the command (`digital_out.hpp:44`) | RED |
| 9 | Bind `IDigitalIn` to `get_new_press()` instead of `get_value()` — **needs two consumers to be visible** | RED |
| 10 | `IBlockSink::write()` returns true when the device refused | RED |
| 11 | Remove the `usd_is_installed()` check | RED |
| 12 | Remove the `#error` from a new shim header | RED |
| 13 | Widen the PROS-free guard and plant a violation outside `hal/pros/` | RED |
| 14 | Move a `#pragma GCC diagnostic pop` past the adapter code | RED |

**A mutation that stays GREEN is a hole and the most valuable thing this chunk can find.** Record
every green with its measurement; do not paper one over (E3's standard).

### Conversion tests
Hand-computed literals, **never importing the constant under test** — E2 found a shipped test
asserting `kMetersToInches == kMetersToInches`. 200 mm is 7.874015748 inches; write that out.

---

## 7. Definition of Done

- [ ] `IDistance`, `IOptical`, `IDigitalOut`, `IDigitalIn`, `IBlockSink` all have PROS-backed
      implementations under `include/shulib/hal/pros/`
- [ ] `hal/digital_in.hpp` + fake exist; register row records **NOT FROZEN**
- [ ] New pure conversions, host-tested with hand-computed literals
- [ ] Shim extended; every new shim header `#error`s outside the host test build
- [ ] The 9999 rule (T4) is implemented, tested, and mutation-proven
- [ ] PROS-free guard re-proven to bite; ARM gate green with the new headers in the glob
- [ ] `make` still produces an uploadable package
- [ ] Full suite green; all 14 mutations executed and recorded
- [ ] `HA-113…` registered for every new belief, each with the bench step that settles it
- [ ] Bench runbook **extended** with the new steps (including: does this robot have an ADI expander?)
- [ ] Documentation contract §8 discharged, **changelog entry included**
- [ ] **The governing constraint survives:** nothing here lets "the adapters read a sensor" drift
      into "it works on a robot"

---

## 8. Documentation scope

**Every change gets documented, not only the interesting ones.** Five seams is five new surfaces.

**Public:** `docs/changelog.md` (deliverable #7 — what changed, breaking or additive, what a user
must do); `docs/faq.md` (the 9999 trap is a *perfect* FAQ entry — "why does my distance sensor read
393 inches?" — plus the `/usd/` path quirk); guide **ch. 13 "Extending the library"**; guide
**ch. 14 "What it cannot do yet"**, edited with more care than anything else here;
`docs/hardware-assumptions.md`; `docs/roadmap.md` with cited evidence and the new register row.

**Internal:** `R1b-PROGRESS.md` (live, first), `R1b-COMPLETED.md` (written *from* the log).

**Header design notes are a deliverable:** every adapter says which PROS call it binds and why,
which conversion it applies, which `HA-nn` its beliefs are registered under, and what it refuses to
do.

---

## 9. Landmines

**L1 — The 9999 rule is the whole chunk.** If one thing here is right, make it that.

**L2 — Frozen means frozen.** F4 is LOCKED; `IDigitalIn` is an **additive sibling outside it**, like
`IDigitalOut` and `IController` before it. If a signature pin fires, **stop and report**. Remember
the freshness gate fires *before* the pins and names the wrong problem.

**L3 — Do not wire `src/main.cpp`.** It carries an X-drive-with-tracking-wheels configuration that
matches no robot we own. Fixing it is not this chunk.

**L4 — The reviewer's harness at `docs/internal/verify/verify-r1b.sh` is the REVIEWER's.** Do not
create, edit or overwrite it. F1's agent destroyed the independence of its own audit this way.

**L5 — Do not commit, do not push.**

**L6 — Assertion counts flatter.** Mutation results are the measure. A chunk that reports only wins
has not looked hard enough.

**L7 — Quoted PROS includes, not angle brackets** (§3). The robot build is `-iquote`-only and host
probes cannot see the difference.

---

## 10. What R1b hands forward

**To R3:** every `HA-113+` with the bench measurement that settles it, and an extended runbook. R3's
promise is that first contact is a checklist rather than an exploration — R1a's session showed what
happens when the checklist assumes hardware that isn't there.

**To F3:** the sensor-confirm seams. F3's non-negotiable rule — never advance on an unconfirmed grab
— becomes implementable the moment `IDistance` and `IOptical` can see a real object.

**Still owed, and not R1b's to answer:** does the lift home on a switch or a stall?
