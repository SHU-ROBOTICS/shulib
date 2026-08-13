# E2 — `GpsCorrector` — completion record

> Completion record for [`E2-gps-corrector.md`](E2-gps-corrector.md). Live log:
> [`E2-PROGRESS.md`](E2-PROGRESS.md). Closes chunk 17 of 40; **Phase E, chunk 2 of 4.**
>
> **Do not commit** was the instruction; everything below is in the working tree.

---

## 1. What this chunk actually did

Everything in the library before this chunk dead-reckoned. `GpsCorrector` is the first code that
can tell the estimator it is wrong, and the first thing to put decisions a real gate made into
E1's introspection path.

It implements `ICorrector` behind the exact signature the seam has carried since M2 — no caller
changed. Per tick, in this order: record the predicted position for latency compensation; decline
if there is no fix; decline if the read is non-finite; decline if the sample has not changed since
it was last folded; decline if the device's own claimed error is too large; decline if the robot
is spinning too fast; carry the fix forward by the odometry travelled since it was captured; run
the normalized-innovation gate; otherwise propose an absolute pose with an adaptive σ and a
confidence derived from it.

**Every decline carries its reason out**, which is the other half of what this chunk is for.

| | |
|---|---|
| Suite before | 752 cases / 936,895 assertions |
| **Suite after** | **794 cases / 1,081,382 assertions**, 3 skipped (unchanged), all green |
| Mutations | **20 executed, 20 red, 1 hole found and closed** |
| CI guards | GUARD1 (PROS-free) PASS · GUARD2 (no `sim/` in core) PASS |
| ARM gate | PASS, 111 headers (was 110) |
| Doc gates | self-test · check-coverage · check-fresh · check-examples · check-removability — all PASS |
| New invented constants | **HA-61 … HA-67**, all registered |
| Freezes | **none** |

### Files

| File | What |
|---|---|
| `include/shulib/localization/gps_corrector.hpp` | **new** — the corrector |
| `include/shulib/hal/gps_conversion.hpp` | *modified* — `gpsRmsErrorToCanonical()` (the HA-07 hole) |
| `include/shulib/diag/debug_record.hpp` | *modified* — three appended `GateReason` values |
| `include/shulib/localization/correction.hpp` | *modified* — `CorrectionProposal::selfAudit` (appended) |
| `include/shulib/localization/localizer.hpp` | *modified* — the substitution rule (~15 lines) |
| `test/gps_corrector_test.cpp` | **new** — 22 cases, the corrector's decisions in isolation |
| `test/gps_corrector_blackbox_test.cpp` | **new** — 8 cases, through the Localizer to a decoded file |
| `test/gps_corrector_accuracy_test.cpp` | **new** — 7 cases, the A2 plant under A3 hostility |
| `test/gps_conversion_test.cpp` | *modified* — 7 `[oracle]` cases (the two register traps) |
| `test/debug_record_test.cpp`, `test/blackbox_format_test.cpp` | *modified* — wire pins for the new reasons |
| `docs/internal/verify/verify-e2.sh` | **new** — the mutation harness |

---

## 2. The tensions, ruled — each with the alternative that was rejected

### T1 — it is a NORMALIZED INNOVATION, and it is called that

`build-order.md` said "Mahalanobis gating". The gate computes

```text
ν = |z − predicted| / σ_eff        σ_eff = hypot(σ_meas, σ_dr)
```

and declines when `ν > gateSigma`.

**RULING: this is not a Mahalanobis distance and must not be labelled one.** A Mahalanobis
distance normalises by the innovation covariance `S = H·P·Hᵀ + R` where `P` is **estimated by a
filter**. The complementary tier estimates no `P`; `σ_dr` here is a hand-written heuristic over an
invented growth rate. So `gateMahalanobis` stays **0** on every E2 path, `RejectedMahalanobis` is
never raised, and the append-only enum gained honest spellings:

```text
RejectedNormalizedInnovation = 6
RejectedStaleFix             = 7
RejectedSensorQuality        = 8
```

**Rejected alternative — call it Mahalanobis anyway.** It is defensible on paper: with an assumed
isotropic `S = σ_eff²·I`, the scalar ratio *is* the Mahalanobis distance. Rejected because the
assumption is the entire content. E4's number will be earned from a filter; E2's is asserted by a
constant. A single field holding both makes the difference between them invisible, and the brief's
formulation is exactly right — *a wrong number is worse than a zero, because a zero is visibly
absent.*

**Rejected alternative — a fixed threshold in inches.** Simpler, and fully reconstructable from
the residual alone with no new constants. Rejected because it cannot widen after a long blind
stretch, which is the one moment a correction is worth the most. See §4.

### T2 — what "never increases error" was allowed to mean

The DoD said the corrector "reduces pose error versus dead-reckoning alone and never increases
it." Tick-by-tick that is unachievable, so the metric was defined **before** the test was written
(logged at 22:26 in the progress file, before any measurement).

**What is claimed:**

1. **Recovery.** Give both estimators the same 6-inch position error and drive. The corrected one
   converges back; the dead-reckoned one carries the error to the end. **Asserted per seed, all 8.**
2. **Aggregate accuracy.** Over 8 seeds of a 60-second run under full hostility, mean final and
   mean worst-case error are lower with the corrector.
3. **Budget honesty.** No tick moves the fused position further from the odometry-only prediction
   than `maxNudgeRate · dt`. Asserted on **every tick of every seed** — about 48,000 assertions.
4. **Degradation.** Off-strip the corrector contributes bit-identically nothing; a confident lie
   mid-run does bounded damage.

**What is NOT claimed, and the measurement that forced the honesty:**

| seed | corrected final / worst | dead-reckoned final / worst |
|---|---|---|
| 1 | 0.043 / 0.594 | 0.314 / 0.546 |
| 2 | 0.174 / 0.674 | 0.360 / 0.618 |
| 3 | **0.771** / 0.771 | **0.560** / 1.607 |
| 4 | 0.164 / 0.478 | 0.205 / 0.605 |
| 5 | 0.095 / 0.385 | 0.211 / 0.707 |
| 6 | 0.310 / 0.751 | 0.590 / 1.013 |
| 7 | 0.122 / 0.477 | 0.205 / 0.702 |
| 8 | 0.118 / 0.563 | 0.404 / 0.668 |

Inches, 60 s, 1246 inches of path, 963 fixes folded. **Final error is better on 7 of 8; worst-case
on 6 of 8.** On seed 3 the corrected final error is *worse*.

The reason is worth understanding rather than tuning away. **Dead-reckoning in this simulation is
already sub-two-inch over a minute**, because A3's slip model degrades the *driven* wheels while
the unpowered tracking wheels read true body travel — leaving IMU heading drift (HA-20) and
encoder quantization as the only error sources. Meanwhile the modeled GPS noise is 0.7″/axis
(HA-26), about 1″ radial. **The sensor's noise is the same order as the drift it is correcting**,
so folding it injects roughly as much noise as it removes.

**Rejected alternative — pick the 30-second closed-loop scenario, which passes 8 of 8 per seed.**
That was the first scenario written and it did pass. Choosing it *because* it passed is scenario
shopping; the outbound 60-second path is the honest one (a closed loop lets heading-drift errors
partly cancel, flattering dead-reckoning's worst case and the corrector's alike). The claim shrank
instead, and the per-seed table is in the test file's own header so nobody has to find this
document to learn it.

**Also not claimed:** any absolute accuracy number; anything about heading; anything about real
hardware; and **not recovery from a grossly wrong estimate** (§4, finding 2).

### T3 — heading stays IMU-owned, and E2 makes it provable

`providesHeading` is never set. More than that: the proposal's `fieldPose` carries the
**predicted (IMU) heading**, not the GPS's, so even a future policy that read
`fieldPose.heading()` would read the IMU's answer. Tested directly with a GPS that is confidently
95° wrong about which way the robot faces, and mutation-proven.

**Rejected alternative — pass the GPS heading through since nothing reads it.** "Nothing reads it
today" is how a field becomes load-bearing by accident.

### T4 — a ruling the brief did not anticipate: the corrector does NOT own frame or lever arm

The brief handed the corrector "frame/lever-arm reduction". Three existing headers say the
opposite, in writing:

- `hal/gps.hpp:26` — `IGps::pose()` is "the robot-CENTER pose (lever-arm corrected)".
- `gps_conversion.hpp:3-5` — "the ONE place the VEX GPS frame becomes shulib's canonical frame
  (§7: convert exactly once, at the edge)".
- `gps_conversion.hpp:29-34` — "Lever arm removed HERE, **ONE owner** = robot config", with
  double-subtraction named as the failure ("inches of silent bias").
- `fake_gps.hpp:3-5` — "the VEX-frame conversion is the adapter's job".

**RULING: the HAL edge keeps both.** A corrector that re-did either would be *correct* against
`FakeGps` (which stores a centre pose) and *silently wrong* against the R1 adapter, which is
contractually obliged to have already done it. That is a bug host tests could never see, which is
the worst kind.

So E2's obligation to the two register traps is **proof, not code** — and the proof turned out to
be the more valuable half (§4, finding 1).

---

## 3. Closing the other half of E1's T3

E1 built the whole introspection path and proved it with a deliberately synthetic corrector,
reporting the DoD clause `[~]`. The reason it could only be half-closed is structural, and E2 had
to build the missing wire.

**The gap.** `Localizer::update()` step 3 keeps only *valid* proposals. A corrector that returns
`{valid=false}` is dropped, so its verdict never reaches a fusion policy, so it never reaches
`FusionResult::audit`, so it never reaches the record. **An off-strip GPS and an empty corrector
list both produced `GateReason::None`** — "no correction proposal this tick". Since **Driving
Skills has no GPS strip**, that is the difference between a diagnosable Skills run and a mystery.
`RejectedNoFix` and `RejectedHighYawRate` had sat in the enum since A1, documented as
corrector-side verdicts, with no way to be produced.

**The wire.** `CorrectionProposal` gained one appended, defaulted, trailing field:

```text
GateAudit selfAudit{};   // the corrector's own account, for the tick nothing was proposed
```

Same discipline E1 used when it appended `GateAudit` to `FusionResult`: every existing positional
construction still compiles and means the same thing. The `Localizer` substitutes it **only when
the policy returned no verdict of its own**, and names the corrector in
`AppliedCorrection::source` so per-source accounting says *which* source went quiet.

**Rejected alternative — a new virtual `lastVerdict()` on `ICorrector`.** It adds temporal
coupling (valid only immediately after `propose()`) to a seam Phases E–I depend on, for data the
existing call already returns.

**Rejected alternative — let the Localizer infer `RejectedNoFix` from "correctors registered, zero
valid proposals".** No new field at all, but it cannot distinguish off-strip from a yaw-rate
rejection from a stale sample, and with two correctors (E3) it cannot say which source was silent.

**A corrector never fills `selfAudit` on an accepted proposal.** If it did, and the Localizer's
screening then rejected that proposal (non-finite confidence, say), the substitution rule would
stamp `Accepted` on a tick where nothing was applied. Pinned by its own test.

### What is now reconstructable from the file alone, precisely

`test/gps_corrector_blackbox_test.cpp` drives real conditions — no strip, a re-read, a bad claim,
a spin, a lie — through a real `Localizer`, `ComplementaryFusion` and `SdSink`, then **decodes the
bytes** and re-derives the verdicts.

- **`RejectedNoFix`** — the reason is the whole decision; residual and trace are asserted **zero**,
  because there was no fix to difference and a leftover number would be the invisible-wrong T1
  refuses. Pinned by a test that first folds a real fix so there *are* numbers to go stale.
- **`RejectedHighYawRate`** — fully checkable: the yaw rate is already on the record
  (`imuYawRate`), so a reader compares it to the documented threshold.
- **`RejectedNormalizedInnovation`** — fully recomputable. `gateResidualX/Y` carry the numerator
  and `covarianceTrace` carries the σ the gate normalized by, so `ν = |residual| / σ` and the
  verdict is `ν > gateSigma`. The test does exactly this from the decoded record and nothing else,
  and independently confirms the numbers are the ones a person would compute from the scenario
  (30″ residual, σ_eff = √5, ν = 13.42).
- **`RejectedStaleFix`, `RejectedSensorQuality`** — the reason is the whole decision.
- **`Accepted`** — verdict, residual, applied nudge and the tier's trust weight, per E1's
  semantics.

**Honest limit, stated rather than glossed:** on an *accepted* tick the exact ν is **not** on the
wire. There is one scalar slot (`covarianceTrace`), E1 assigned it to the trust weight, and E2 did
not reassign it — on a *declined* tick that slot is genuinely vacant (nothing was trusted), which
is why σ can occupy it there unambiguously, with `reason` naming the producer. Reassigning the
slot on accepted ticks would have broken E1's documented meaning to buy a number that only
confirms an inequality the verdict already asserts.

The DoD line is therefore reported **closed for corrector-side decisions** and **scoped** for the
accept path — a real improvement on E1's `[~]`, not a claim of completeness.

---

## 4. Two findings that predate this chunk

### Finding 1 — HA-07 had no code and no test, and the existing tests could not have caught it

The register's most dangerous entry — `get_error()` returns **metres**, everything here is
**inches**, factor 39.37, silent in both directions — existed only as **prose** in
`gps_conversion.hpp`, addressed to an adapter author who does not exist yet. No function performed
it. Nothing tested it.

Worse, the pre-existing conversion tests pinned the *position* scale like this:

```text
CHECK(east.x().value() == doctest::Approx(kMetersToInches));
```

with `kMetersToInches` imported from the header under test. **If the constant were 3.937, every
one of those assertions would still pass** — the same wrong number on both sides of the `==`. That
is precisely the shared-oracle blindness the brief warned about, sitting in the file most exposed
to it.

Fixed **in the layer that owns it** (Rule 4): `gpsRmsErrorToCanonical()` now exists with a
fail-loud sentinel screen, and the new `[oracle]` tests assert against the **definition** of the
inch — 0.0254 m *is* 1.0 inch, 0.3048 m *is* 12.0 — arithmetic a person checks on paper without
opening the header. Mutation-proven both ways (dropping the multiply on rms and on position each
go red).

### Finding 2 — a hard 12-inch ceiling on what any corrector can repair

Observed live while debugging the first accuracy run: with the estimate 29 inches from truth and a
perfectly good GPS in view, **nothing ever recovered**. The corrector's anti-lockout widening was
working correctly; the ceiling is one layer down.

`ComplementaryFusion::innovationGate` is a **fixed 12 inches**, applied after the corrector's own
gate. E2's gate can open as wide as it likes; a residual above 12 inches is rejected by the policy
regardless.

**Not fixed here, deliberately.** The constant belongs to the fusion policy; E4 replaces that
policy with an EKF whose gate is a real Mahalanobis test; and reaching up from inside a corrector
to retune another layer's threshold is exactly the two-owner change Rule 4 exists to prevent.
Recorded with its evidence, and the guide now says it in plain English: *bounded drift is not the
same promise as recovery.*

---

## 5. The decision docket

**D1 — the confidence is the scalar Kalman gain, `σ_dr²/(σ_dr² + σ_meas²)`.** It has the right
shape for a complementary tier's pull weight: trust the fix more when the estimate is uncertain,
less when the sensor says it is. *Rejected: a constant confidence* — it cannot be adaptive R in
any meaningful sense. *Rejected: `1/σ_meas` normalised* — monotone in the right direction but with
no interpretation, and it never rises when the robot is lost.

**D2 — σ_dr grows with distance travelled since this source's last fix.** Without it, after twenty
feet of dead-reckoning a truthful fix looks outrageous and is rejected — and so is every fix after
it, because nothing else can repair the estimate. **Gate lockout: the GPS dies exactly when it is
worth the most.** Pinned by a test where two rigs differ *only* in `driftStdDevPerInch`, and by
its companion proving the widening RESETS on an accepted fix (otherwise the gate stays open after
one blind stretch and a lie is accepted because the corrector still believes it is lost).

**D3 — one measurement is folded once.** A read whose position and reported error are unchanged is
declined as `RejectedStaleFix`. *Rejected: fold every read* — at 100 Hz against a ~50 ms cadence
that treats one observation as five independent ones, and re-applies a measurement describing an
ever-older moment while the robot moves. It also makes correction strength depend on the ratio of
loop rate to camera cadence, which is a hidden coupling nobody should have to reason about.
Measured: 2520 stale ticks against ~570 fresh in 30 s, i.e. ~19 fresh fixes/s against HA-28's
modeled 20/s. **Documented limitation:** with a noiseless GPS (the identity degradation model, or
a `FakeGps` set once) every read after the first is byte-identical, so correction happens once and
stops. That is the contract behaving as written; a real device jitters.

**D4 — latency compensation bridges with odometry, not with a velocity estimate.** `z + (P(now) −
P(capture))`, read from a 64-entry ring, rather than `z + v·L`. Odometry is excellent over 50 ms
even when it is poor over 50 seconds, which is exactly what this needs; the velocity form carries
an `a·L²/2` error through every acceleration. Fixed capacity, no allocation.

**D5 — the corrector times from the injected clock, not from the `dt` parameter.** The standing
injected-clock contract, and `now()` is authoritative where a per-tick `dt` is a difference the
Localizer already took. `dt` is unused and says so.

**D6 — yaw rate comes from an injected `IImu`, not from differencing predicted headings.** The
corrector *could* difference `predicted.heading()` tick to tick and avoid a second HAL dependency
— but at 0.05° heading noise and dt = 0.01 that is 5°/s of noise on the quantity a threshold is
being applied to. The IMU reports the rate directly; the Localizer's own twist already uses it.

**D7 — a sensor-quality ceiling exists.** Without it, a fix claiming 99″ of error is still folded:
the gate widens to accept it and the confidence shrinks to ~0.0001, so the estimate barely moves —
but the Localizer sees `applied`, reports quality class **Corrected**, and a run with no usable
anchor looks anchored. That last consequence is what earns the third enum value.

**D8 — a rejection consumes the sample.** A fix declined for high yaw rate is not folded later
once the spin ends: it describes a moment already judged untrustworthy. Pinned by its own test.

**D9 — `propose()` never throws and never allocates.** Non-finite reads are screened before any
`Pose2d` is constructed, so the F4 finiteness contract is honoured without relying on a
precondition throw inside the control loop.

---

## 6. Test evidence

**44 new/changed cases.** Every one names, in a comment, the bug it would catch.

**`gps_conversion_test.cpp` — 7 new `[oracle]` cases (the two register traps).** Independent by
construction: the inch scale from the definition, the frame rotation re-derived from the compass
convention (North along θ_N, East 90° clockwise of it), the lever arm re-derived from "left is 90°
CCW of forward". Seven headings × five arms with **both components non-zero** — the pre-E2 cases
used one component at a time at headings 0 and 90 only, where a cross-term error has somewhere to
hide. One case is worked entirely by hand with the arithmetic in the comment.

**`gps_corrector_test.cpp` — 22 cases.** Adaptive R (σ and confidence hand-computed to 16
digits; the floor; the monotone response to a worse claim) · no fix (never a low-confidence pull;
the unconfigured-GPS safe default; non-finite reads as no-fix and **not a throw**) · staleness
(folded once, then declined; freshness watches the reported error too) · sensor quality · high yaw
rate (both signs; and a mid-spin sample not folded after the spin) · latency (the exact case: 1
inch/tick, a fix describing five ticks ago, compensated back to the prediction; the sign; a
stationary robot unmoved) · the gate (boundary at the hand-computed 8.944…; the numbers a
rejection carries; anti-lockout; the reset) · T3 · never-snap through the real fusion policy ·
`selfAudit` empty on accept.

**`gps_corrector_blackbox_test.cpp` — 8 cases.** Off-strip visibility; **bit-identical to no
corrector** (a stronger statement than "small effect"); the substitution rule in both directions;
all six verdicts arriving in the *decoded* file; a gate rejection re-derived from the decoded
record alone; a no-fix tick carrying no stale residual.

**`gps_corrector_accuracy_test.cpp` — 7 cases**, ~144,000 assertions. Both estimators run on **one
plant reading one sensor stream**, tick for tick — two `PilonsOdometry` over the same tracking
wheels, two `Localizer`s, one with the corrector — so there is no run-to-run variance to argue
about and the only difference between the two numbers is the corrector.

### Mutations — 20 executed, 20 red, 1 hole

`docs/internal/verify/verify-e2.sh`, gated on build success (a mutation that will not compile is
reported BUILD-FAIL, never as red).

Red: lever-arm sign (offY) · lever-arm cross term (offX) · frame axes swapped · metres→inches
dropped on rms · metres→inches dropped on position · gate inverted · **off-strip returns a
low-confidence fix** · latency dropped · latency backwards · staleness guard removed · anti-lockout
widening removed · rmsTrustFactor dropped · σ floor removed · yaw-rate rejection removed ·
sensor-quality ceiling removed · **GPS heading leaks into the proposal** · corrector verdict never
reaches the record · confidence inverted · travel accumulator never resets · substitution
overwrites a real verdict *(after the fix)*.

### THE HOLE — a mutation that stayed green

```text
localizer.hpp:  if (tickAudit.reason == None && selfAuditSource != nullptr)
            →   if (selfAuditSource != nullptr)
```

**Why the suite could not see it.** With **one** corrector the two versions are equivalent:
`selfAuditSource` is only set when a proposal is *declined*, and a declined proposal means the
policy saw nothing, which means its reason is already `None`. The guard is dead code until a
**second** corrector exists — and then it is load-bearing: corrector A proposes a fix the policy
ACCEPTS while corrector B is off-strip, and the mutated code stamps B's `RejectedNoFix` over A's
`Accepted`. **The record would say the estimator dead-reckoned on a tick where it applied a
correction** — the same class of invisible-wrong the T1 ruling refuses.

E3 adds the second corrector, so this would have shipped as a latent bug appearing one chunk
later, in telemetry, where nobody looks. Every Localizer-level test in the chunk wired exactly one
corrector: the blind spot was **structural, not accidental**.

Closed with a two-corrector case. Verified to fail **alone**: under the mutation it is the only
failing case in the whole 794-case suite (3 assertions), and green with the guard restored.

---

## 7. A process failure of mine, recorded because the recovery is the point

Two harness faults, both mine, caught by reading `git diff` rather than trusting the mutation
report:

1. **I ran `git checkout -- localizer.hpp` to undo a mutation.** That file held uncommitted E2
   work, so the checkout discarded the chunk's change, not the mutation. The brief forbids
   `git checkout` for exactly this reason.
2. **I piped the mutation script into `head`.** SIGPIPE killed it mid-mutation and left
   `gps_conversion.hpp` on disk with the metres→inches multiply **deleted** — the very trap the
   chunk exists to guard. Three mutations then reported `SKIP — pattern not found`, which I had
   written as a quiet one-line shrug.

**The tell was the totals: 17 RED where the previous run had 19 + 1 GREEN.** A drop in the count
of mutations that *ran* is the signal; the report itself looked fine.

`verify-e2.sh` is hardened: restore is always from the byte copy taken before the edit (never
`git checkout`, with the reason in the script); a `trap` on INT/TERM/PIPE restores the file in
flight; and **a SKIP is now loud and exits non-zero**, because an unfound pattern means a required
mutation never ran — "a mutation you reasoned about but did not execute does not count" applies to
the harness too. State verified clean afterwards and all 20 re-run red.

---

## 8. What we know for certain, and what we do not

**KNOWN**

- The corrector implements `ICorrector` behind the unchanged signature; no caller changed.
- Off-strip, it contributes **bit-identically nothing** — the estimate matches a Localizer with no
  correctors at all, digit for digit — while the record names both the reason and the source.
- Every corrector-side gating decision reaches a **decoded blackbox file**, and rejections are
  numerically re-derivable from it.
- A known 6-inch position error is healed on all 8 seeds; dead-reckoning carries it to the end.
- No tick ever moved the fused position more than `maxNudgeRate · dt` from the odometry-only
  prediction, across ~48,000 asserted ticks.
- The GPS's heading never leaves the corrector (mutation-proven).
- The frame and lever-arm conversions are pinned by oracles that share nothing with them.

**NOT KNOWN, stated plainly**

- **Nothing here has seen a GPS.** Every behaviour was proven against a simulated sensor whose
  noise, cadence, reported error and failure modes are invented (HA-26…HA-31).
- **Whether folding GPS is worth doing at all is unmeasured.** It depends on the ratio of sensor
  noise to dead-reckoning drift; both numbers are invented, and in simulation they are the same
  order — which is why the gain is real but modest, and why the corrector loses on one seed of
  eight.
- **Seven new constants are guesses** (HA-61…HA-67). Every test asserts a *shape* — "a worse claim
  widens σ", "the gate widens with travel", "a spin rejects the fix" — never that a constant is
  right.
- **E2 bounds drift; it does not recover a grossly wrong estimate** (§4, finding 2).
- **The `< 1°` heading claim is exactly where A3 left it.** Heading is untouched by design.
- **`gateMahalanobis` is still 0 on every real path**, and will be until E4.
- **The simulation's dead-reckoning is better than a real robot's**, because A3 models slip on the
  driven wheels and the tracking wheels read true body travel. Real tracking wheels skid and lift.

---

## 9. Documentation contract — discharged

1. **Roadmap checkbox** — WS5 tier 2 `GpsCorrector` flipped `[x]` with file, test files, case and
   assertion counts, and the three scope caveats attached to the checkbox itself.
2. **"You are here"** — E2 recorded with what it did *and* what it did not; next pointer moved to
   E3.
3. **Design notes in the headers** — `gps_corrector.hpp` carries the order of operations, the
   deliberate non-responsibilities, the T1 reasoning, why σ_dr exists, the double-count guard with
   its honest limitation, and the latency reasoning. `correction.hpp` explains why `selfAudit`
   exists and its contract. `localizer.hpp` explains the substitution rule *and* why its guard is
   dead code today.
4. **Test evidence** — §6, with every mutation named and the hole given its own section.
5. **Decisions** — §2 (T1–T4) and §5 (D1–D9), each with its rejected alternative.
6. **Freeze Register** — **E2 freezes nothing.** It appends to two wire-stable vocabularies
   (`GateReason`) and one value type (`CorrectionProposal`), both append-only by their own
   documented rules, both re-pinned by test. F9 remains H1's.

**Guide, per the named scope:**

- **Chapter 3** gains "The GPS corrector: what changes when a real one exists" — written as a
  change to the *mental model* (error accumulates between fixes and is removed at each one, rather
  than accumulating forever), the six checks the corrector runs, and four honest limits including
  the Skills case. The "design around drift" advice now distinguishes Autonomous from Skills.
- **Chapter 11** gains "Why the estimator trusted (or ignored) a sensor" — all nine `GateReason`
  spellings in a table, two reading habits, and why the Mahalanobis slot is deliberately empty.
- **Chapter 14**'s "No absolute position correction yet" **partly falls**, rewritten as "Position
  correction exists; yaw correction does not" — with the five things that remain missing, because
  the half that the `< 1°` spec actually turns on is the half still outstanding.

No guide code block changed, so no example re-quoting was needed; `check-examples` passes.

---

## 10. Verification (run, with output as observed)

```text
cmake --build build/test -j$(nproc) && ./build/test/shulib_tests | tail -6
  [doctest] test cases:     794 |     794 passed | 0 failed | 3 skipped
  [doctest] assertions: 1081382 | 1081382 passed | 0 failed |
  [doctest] Status: SUCCESS!

GUARD1 PASS          (no pros/ include anywhere in include/shulib)
GUARD2 PASS          (no shulib/sim/ include outside sim/)
ARM GATE PASS        (111 headers, arm-none-eabi-g++ -std=gnu++20 -Werror …)

doc gates:  self-test PASS · check-coverage PASS · check-fresh PASS
            check-examples PASS · check-removability PASS

docs/internal/verify/verify-e2.sh
  E2 mutations: 20 RED (good), 0 GREEN (HOLES), 0 build-fail, 0 SKIPPED

register reconciliation, direction 1 (must print nothing):
  grep -rn "PROVISIONAL (A4" include/ test/ | grep -v "HA-[0-9]"   → empty
```

---

## 11. Named handoffs

**→ E3 (`AprilTagCorrector`)**

- **The substitution guard becomes live code the moment you add a second corrector.** The
  two-corrector test exists; read it before touching `localizer.hpp`.
- **The record has ONE gating slot.** With two correctors, only the first silent source's verdict
  survives. That was acceptable with one corrector and is a real design question with two — decide
  it deliberately rather than inheriting it.
- **The heading nudge is yours to decide.** T3 kept the documented additive path open
  (`headingNudge` on `FusionResult` + the Localizer applying it before the IMU re-stamp).
  `providesHeading` is still RESERVED and still ignored.

**→ E4 (the EKF)**

- **`gateMahalanobis` and `RejectedMahalanobis` are yours**, unused and unfaked. When the filter
  has a real `P`, both become meaningful — and E2's `RejectedNormalizedInnovation` stays as the
  honest name for what the complementary tier does, so old blackboxes keep their meaning.
- **`ComplementaryFusion`'s fixed 12-inch gate** is the ceiling on what any corrector can repair
  (§4, finding 2). A covariance-driven gate is the principled replacement.
- **HA-66 (`postFixStdDev`) is a gain knob wearing the clothes of a covariance.** An EKF estimates
  it instead of asserting it.

**→ R3 / R4**

- **HA-01's field-cal oracle is still skipped**, and still the oldest entry in the register. The
  new `[oracle]` cases prove the conversion matches its documented convention; they cannot prove
  the convention matches the device.
- **HA-07's hole is closed but the assumption is not.** Read `get_error()` on strip: a healthy fix
  should report ~0.01–0.05 (metres), not ~1–2.
- **HA-26 and HA-20 together decide whether this chunk was worth building.** Measure both.
