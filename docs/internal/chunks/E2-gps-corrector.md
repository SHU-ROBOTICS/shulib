# Chunk E2 — `GpsCorrector`

> **Phase E, chunk 2 of 4.** Predecessor: E1 (the blackbox + the introspection path).
> **The first REAL corrector.** Everything before this dead-reckoned; this is the first code that
> can tell the estimator it is wrong.

**Workstream:** WS5 (localization) · **Milestone:** M3 · **Freezes:** none

---

## Why this chunk exists, and why it is *here*

E1 built the path that records *why* the estimator trusted or rejected a fix, and proved it with a
synthetic corrector — deliberately, because no real one existed. **E2 is what makes those numbers
real.** It is also the first chunk where the `< 1.0°` / ~1.0″ accuracy story stops being a
simulation of drift and starts being a measurement of drift *bounded by something*.

It comes before the EKF (E4) for the same reason A1 came before A2: the simplest real corrector
against the existing complementary tier exposes every framing, units, latency and gating problem
while the math is still small enough to debug. Those problems do not get easier inside a Kalman
filter — they get *hidden* inside one.

---

## What already exists — do not rebuild any of it

| Thing | Where | Note |
|---|---|---|
| **`ICorrector` — the exact signature to implement** | `localization/i_corrector.hpp` | PULL, not push: `propose(predicted, dt)`. Non-throwing. Returns `{valid=false}` when it has no usable fix — **never a zero-confidence pull.** |
| **`CorrectionProposal` — already has every field you need** | `localization/correction.hpp:49` | `valid`, `fieldPose`, `confidence`, `positionStdDev`, and `providesHeading` |
| `FusionResult` incl. **`GateAudit audit`** (appended at E1) | `localization/correction.hpp:63` | `applied` / `gated` / `clamped` / `appliedConfidence` |
| `GateReason` — the accept/reject vocabulary | `diag/debug_record.hpp:45` | Explicit-valued, append-only |
| The record fields you finally populate | `diag/debug_record.hpp` | `gateResidualX/Y`, `gateMahalanobis`, `gateReason` |
| **GPS frame + unit conversion** | `hal/gps_conversion.hpp` | Read it before writing any arithmetic |
| **A3's hostile GPS**: no-fix (off-strip), bad-fix, error | `sim/degradation.hpp`, `hal/fake/fake_gps.hpp` | `FakeGps` defaults to **no fix** — "safe default off-strip" |
| `NullCorrector` — the placeholder you replace | `localization/i_corrector.hpp` | The seam is already wired for telemetry |

**Read first:** `i_corrector.hpp` and `correction.hpp` **in full**; `hal/gps_conversion.hpp`;
`E1-COMPLETED.md`'s introspection section (you are closing the other half of its T3);
`hardware-assumptions.md` entries **HA-01, HA-07, HA-10, HA-26…HA-31**.

---

## Two traps that are already written down, and will be silent if you miss them

Both are register entries, which is exactly what the register is for.

### HA-07 — `pros::Gps::get_error()` returns **METERS**
Every length in this library is **inches**. A missed conversion is a factor of **39.37**, and it
would not look like a crash — it would look like a corrector that gates everything (error read as
absurdly large) or trusts everything (absurdly small). **Pin the conversion against a hand-computed
absolute, not against another constant from the same file.**

### HA-01 — GPS position axes are **+X = East, +Y = North**
F1's frozen field frame is origin = field centre, +X right, +Y away from red. **These are not
automatically the same frame.** `gps_conversion.hpp` exists for this; use it, and pin the transform
with an **independent from-scratch oracle**, not by round-tripping through the same helper. A frame
error here cancels perfectly in any test that uses the same conversion on both sides — this project
has been bitten by exactly that three times (C1, C3, C4).

---

## Three things to rule on explicitly

### T1 — is it really a Mahalanobis distance at E2?
`build-order.md` says "Mahalanobis gating". But `GateReason::RejectedMahalanobis` is tagged
**"(EKF tier, E4)"**, and a true Mahalanobis distance needs a covariance — which E4's EKF provides
and the complementary tier does not.

Decide what E2's gate actually computes (most likely a **normalized innovation**: residual over the
proposal's `positionStdDev`, possibly inflated by accumulated dead-reckoning uncertainty), and
**name it honestly.** If it is not a Mahalanobis distance, do not populate `gateMahalanobis` with it
and do not raise `RejectedMahalanobis` — add the honest `GateReason` (the enum is append-only by
design) and leave the Mahalanobis field for E4. **A field labelled `gateMahalanobis` holding
something that is not one is worse than a zero**, because a zero is visibly absent and a wrong
number is invisibly wrong.

### T2 — what does "never increases error" actually mean?
The DoD says the corrector "reduces pose error versus dead-reckoning alone and **never increases
it**." Taken tick-by-tick that is unachievable: any corrector fed a noisy fix will occasionally make
a single tick worse, and a test asserting otherwise is either vacuous or unfairly specified.

Define the metric before you write the test. The defensible claims are shaped like: *over a run,
final and worst-case error are lower than dead-reckoning alone across N seeds*; *error stays bounded
rather than growing without limit*; *no single accepted correction moves the estimate more than the
per-tick budget*. Say which one you are proving, and state plainly what you are **not**.

### T3 — heading stays IMU-owned
GPS reports a heading. `CorrectionProposal::providesHeading` is **RESERVED and ignored at M2** —
`correction.hpp` says so, and `FusionResult` deliberately has no heading field because *the Localizer
re-stamps heading from the IMU as the final write*. **Do not start using it.** If GPS heading is
worth having, that is the documented additive path (a `headingNudge` on `FusionResult`) and it
belongs to E3/E4 with a deliberate decision, not to E2 as a side effect.

---

## Scope

### In
1. **`GpsCorrector`** implementing `ICorrector` — HAL access, frame + unit reduction, lever-arm
   compensation, latency compensation, staleness, gating. The corrector owns all of its own mess by
   design; the Localizer stays geometry-free.
2. **Adaptive R** from `get_error()` → `positionStdDev` and `confidence`.
3. **The gate** (T1), with every decision surfacing through E1's audit path into the record.
4. **High-yaw-rate rejection** — during a fast turn the lever-arm correction is at its most wrong.
5. **The off-strip dead-reckon-only flag** — see below; this is the competition-critical path.
6. **Accuracy evidence** against the A2 plant with A3's hostile GPS (T2's metric).

### Out
- The EKF and true Mahalanobis gating → **E4**
- AprilTags → **E3**
- GPS *heading* → deliberately not used (T3)
- The real `pros::Gps` adapter → **R1**; you build against the HAL seam and `FakeGps`
- Real noise magnitudes → **R4**. Every hostile GPS number is invented; E2 proves *logic*, not
  constants.

### Explicitly rejected
- **A hard pose reset on a fix.** §13 #4's never-snap invariant is not negotiable: corrections are
  bounded nudges, always. A snap is how a routine teleports mid-match.
- **Trusting a fix because it exists.** `FakeGps` defaults to no-fix for a reason.

---

## The off-strip case is the one that loses matches

**Driving Skills has no GPS strip.** A corrector that quietly trusts garbage when the strip is
absent will not fail loudly in testing — it will produce a routine that works in Quals and
disintegrates in Skills, which is the worst possible failure schedule.

Required behaviour: no fix ⇒ `{valid=false}`, **never** a low-confidence pull. Off-strip must be
*visible* — a named `GateReason`, a per-source dead-reckon accounting entry, something a person
reading the blackbox can see. And it must be tested as its own case, with the estimator proven to
degrade to dead-reckoning rather than drifting toward a phantom fix.

---

## Test requirements

Every test names, in a comment, the bug it would catch.

- **Units:** a `get_error()` value in metres produces the right `positionStdDev` in inches, pinned
  against a hand-computed absolute (HA-07).
- **Frame:** GPS (East/North) → field frame, pinned by an **independent from-scratch oracle**
  (HA-01). Not a round trip through the same helper.
- **Lever arm:** a GPS mounted off-centre, robot rotated — the reduced pose is correct. Pin at
  several headings, because a lever-arm sign error is invisible at heading 0.
- **Latency:** a fix that describes where the robot *was* is not applied as where it *is*.
- **Gating:** a deliberately bad fix is rejected, and the reason is recorded (T1's honest name).
- **High yaw rate:** fixes during a fast turn are rejected.
- **Off-strip:** degrades to dead-reckon-only, visibly.
- **Accuracy (T2's stated metric):** across N seeds on the A2 plant, versus dead-reckoning alone.
- **Never-snap:** no accepted correction exceeds the per-tick budget; `clamped` is honest.
- **Through the blackbox:** every gating decision is reconstructable from the file alone — this is
  the clause E1 could only half-close, and E2 is where it becomes true.

### Mutations
- Flip a lever-arm sign; swap the frame axes; drop the metres→inches conversion; invert the gate;
  make off-strip return a low-confidence fix instead of `{valid=false}`. **Each must go red.**
- **A mutation that stays GREEN is a hole — log it, close it with a test that fails alone, and give
  it a prominent place in the record.** Every chunk so far found one; D3 found four, E1 two.
- Gate the runner on build success.

**The warning that has come true three times:** if the oracle shares the conversion with the code,
a sign or axis error cancels on both sides and the test passes. At least one frame test and one
lever-arm test must be written from the geometry, by hand, independently.

---

## Definition of Done

- [ ] `GpsCorrector` implements `ICorrector` behind the existing signature, no caller change
- [ ] Adaptive R from `get_error()`; units and frame pinned by independent oracles
- [ ] Lever-arm + latency compensation, pinned at several headings
- [ ] The gate rules per T1, **honestly named**; decisions reach the record
- [ ] High-yaw-rate rejection tested
- [ ] **Off-strip degrades to dead-reckon-only, visibly, as its own test**
- [ ] Accuracy evidence per T2's stated metric, with the claim scoped honestly
- [ ] Never-snap preserved; no hard reset anywhere
- [ ] Every gating decision reconstructable from the blackbox alone (closes E1's T3)
- [ ] T1/T2/T3 ruled with rejected alternatives; invented constants registered `HA-nn`
- [ ] Suite green; both guards; ARM gate; all four doc gates

---

## Live progress log — required

`docs/internal/chunks/E2-PROGRESS.md`, appended as work happens (`date +%H:%M:%S`).

---

## Documentation contract

All six, plus **`E2-COMPLETED.md`** at the depth of C1–C5 / D1–D3 / E1.

**Documentation scope, named explicitly** (per the standing instruction that docs are never the
thing compressed to save a chunk): guide **chapter 3** (knowing where you are) gains the corrector
story — drift becomes *bounded* rather than unbounded, which is a change to the mental model, not
just a new feature; **chapter 11** gains the new `GateReason` spellings; **chapter 14** loses a
limitation if one genuinely falls. Any new public member needs a `///` or the build fails.

**Do not commit. Do not push.**

---

## Landmines

- **Metres.** HA-07. A factor of 39.37, silent.
- **Frames.** HA-01 vs F1. Cancels in any shared-conversion test.
- **Don't call it Mahalanobis if it isn't** (T1).
- **Don't claim "never increases error"** without defining the metric (T2).
- **Don't touch heading** (T3).
- **Don't snap.** §13 #4.
- **Don't let off-strip be a quiet low-confidence pull.** It is the Skills-losing bug.
- **Don't tune to the hostile model's invented magnitudes.** They are guesses until R4; E2 proves
  logic, not constants.
