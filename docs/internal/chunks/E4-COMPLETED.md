# E4 — the 5-state SE(2) EKF — completion record

> Completion record for [`E4-ekf.md`](E4-ekf.md). Live log: [`E4-PROGRESS.md`](E4-PROGRESS.md).
> Closes chunk 19 of 40; **Phase E, chunk 4 of 4 — the phase is complete.**
>
> **Do not commit** was the instruction; everything below is in the working tree.

---

## 1. What this chunk actually did

Before this chunk the library could *bound* the damage when two correctors disagreed. It could not
*resolve* it, and it could not say how wrong it might be. `EkfFusion` is a 5-state SE(2) extended
Kalman filter behind the `IFusionPolicy` seam that has been carrying that promise since M2. The
seam did not move, no caller changed, and `ComplementaryFusion` is untouched and still the default.

| | |
|---|---|
| Suite before | 867 cases / 1,091,167 assertions |
| **Suite after** | **915 cases / 1,521,419 assertions**, 3 skipped (unchanged), all green |
| New cases | **48** |
| Mutations | **36 executed, 34 red, 2 recorded green with their measurements**, 0 build-fail, 0 skipped |
| CI guards | GUARD1 (PROS-free) PASS · GUARD2 (no `sim/` in core) PASS |
| ARM gate | PASS, **115 headers** (was 114) |
| Doc gates | self-test · check-coverage · check-fresh · check-examples · check-removability — all PASS |
| New invented constants | **HA-83 … HA-91** (9), all registered, reconciliation clean both ways |
| Freezes | **none** |

### Files

| File | What |
|---|---|
| `include/shulib/localization/ekf_fusion.hpp` | **new** — the filter |
| `include/shulib/diag/debug_record.hpp` | *modified* — one appended `GateReason` (`CovarianceReinit = 12`) |
| `test/ekf_fusion_test.cpp` | **new** — 33 cases: the invariants, the gate, arbitration, re-init, never-snap, T1, and a from-scratch algebra oracle |
| `test/ekf_fusion_seam_test.cpp` | **new** — 8 cases: the swap through a real `Localizer`, out to decoded blackbox bytes |
| `test/ekf_fusion_accuracy_test.cpp` | **new** — 4 cases: EKF vs complementary on the A2 plant, identical seeds |
| `test/ekf_fusion_cost_test.cpp` | **new** — 3 cases: the allocation pin |
| `test/debug_record_test.cpp`, `test/blackbox_format_test.cpp` | *modified* — wire pins for the new reason |
| `docs/internal/verify/verify-e4.sh` | **new** — the mutation harness |

**What this chunk did NOT touch, deliberately:** `IFusionPolicy`, `ICorrector`, `IPoseSource`,
`CorrectionProposal`, `FusionResult`, `AppliedCorrection`, `Localizer`, `ComplementaryFusion`,
`GpsCorrector`, `AprilTagCorrector`. The seam was built EKF-ready at M2 and it was filled, not
reshaped. **No existing test changed except the two wire pins**, and no existing never-snap test
changed at all.

---

## 2. T1 — the state has θ, and the policy still does not own heading

`IFusionPolicy`'s header says a policy *"returns the corrected POSITION only — heading is
re-stamped from the IMU by the Localizer afterward, so a policy can never own heading."* E3 then
opened the one sanctioned heading path: a bounded `headingNudge` folded into a persistent bias.
The state is `[px, py, θ, vx, vy]`, so θ is in it.

**RULING: the EKF tracks θ, and every word of both of those sentences stays true.** Three
mechanisms, and each is enforced structurally rather than by convention:

1. **The θ time update IS the IMU's, by assignment**: `θ⁻ := predicted.heading()` at the top of
   every tick. The filter holds no rival heading and integrates no heading of its own. What it
   estimates is the **error** in the IMU's answer, and that estimate leaves as a bounded increment
   — exactly the shape E3 designed.
2. **That assignment is also what makes the feedback loop safe.** `predicted.heading()` already
   contains the bias built out of this filter's own past nudges. A filter that integrated
   `Δθ = predicted.heading() − previousPredictedHeading` would count its own correction a second
   time and overshoot, with the overshoot growing with the gain. Re-basing makes the double count
   **structurally impossible** rather than arithmetically avoided.
3. **Only a proposal that sets `providesHeading` may move θ**; for every other update the θ row of
   the Kalman gain is zeroed. So a GPS cannot rotate the robot's idea of the field through a
   cross-covariance term, and E2's T3 ruling survives the swap. Proven by a test that compares
   `pose().heading().radians()` with `==` on doubles across 300 ticks with position fixes landing
   throughout, and again through the real `Localizer`.

θ is genuinely load-bearing rather than decorative: the velocity states are in the **body** frame,
so the motion model `p⁻ = p + R(θ)·v·dt` cannot be evaluated without it, and the θ column of `F` is
where the "extended" in EKF actually lives.

**Rejected — the EKF owns the published heading and the re-stamp is deleted.** Tidier algebra, and
a structural change to both M2's decision #4 and E3's design. A policy that can return an absolute
heading can snap one; there is nowhere on `FusionResult` to put an absolute heading without
reshaping it; and E3's whole argument is that an increment is *structurally incapable* of snapping
whatever a corrector claims.

**Rejected — drop θ and run a 4-state filter with field-frame velocity.** Simpler, and it removes
the double-count hazard entirely. Rejected because without θ in the state there is **no θ
variance**, so a heading fix cannot be weighed against the filter's own uncertainty — heading
arbitration becomes impossible, which is half of what the chunk exists for — and the θ↔position
cross-covariance does not exist to be had.

**Rejected — track θ internally but emit nothing**, leaving heading to the complementary tier.
Then swapping tiers would *silently delete* E3's heading correction. A swap that removes a
capability is a regression wearing a feature's clothes.

---

## 3. T2 — "consecutive-reject re-init" vs §13 #4 "never snap"

**This was a genuine conflict between two documented requirements, latent in `build-order.md`
since the roadmap was drafted.** Re-init exists to recover from a hijacked estimate; that is
precisely a discontinuous jump; never-snap is one of the project's oldest invariants. Both cannot
hold unconditionally.

**RULING: "re-init" means re-initialising the belief's UNCERTAINTY, never its VALUE.** On the
trigger the filter resets its position and velocity covariance to the initial prior and **does not
move the estimate by so much as a thousandth of an inch.**

This is not a compromise between the two requirements. It satisfies both exactly:

- **Never-snap holds bit-for-bit.** The per-tick rate clamp is untouched and still binds on the
  re-init tick and every tick after it. **No existing never-snap test changed, and none needed
  to.** Asserted directly: across the 30-inch shove scenario the worst per-tick correction is
  0.12 in against a 0.12 in budget, on the declaring tick as on all others, and re-read from
  decoded blackbox bytes.
- **The estimator nevertheless recovers**, which is the whole reason re-init was asked for. With a
  large `P` the Mahalanobis gate opens, the next fixes are accepted with a large gain, and the
  estimate walks home at the ordinary bounded rate. Measured: 30 inches recovered in **295 ticks
  (2.95 s)** — and the fact that it took 295 ticks rather than one is itself the numeric signature
  of a nudge rather than a teleport, and is asserted.
- **It is declared.** `GateReason::CovarianceReinit = 12`, appended to a wire-stable, append-only
  vocabulary and re-pinned by test. The event is a **word** in the decoded blackbox, not an
  inference from a discontinuity. `covarianceTrace` jumping on the same tick (measured:
  0.026 → 1152 in², a factor of 45,000) is the independent numeric witness.
- **It is latched and counted** (`reinitCount()`, `everReinit()`, which never clears), and
  **rate-limited** by a cooldown, and it takes a high bar: N consecutive Mahalanobis rejections
  **and** a mean rejected innovation above a floor. Ticks on which nothing was proposed neither
  count nor reset. Measured: 30 seconds of alternating hijacks produced 5 declarations with every
  gap at or above the 5-second cooldown; an ordinary 60-second hostile run over 8 seeds produced
  **zero**.

**Rejected — teleport the state onto the rejected fix** (the literal reading). That is the snap
§13 #4 forbids, and the brief's argument holds: a wrong-but-continuous estimate still produces
sane motion, while a teleport produces a violent correction the moment the next motion runs. There
is also a plain self-contradiction in it — **the trigger is the filter saying fifty times that it
does not trust this fix, so jumping onto that fix trusts it completely on the strength of having
distrusted it.**

**Rejected — no re-init at all.** That preserves E2's finding 2 forever. Proven as a negative
control: with re-init disabled, a 30-inch displacement is still 30 inches wrong thirty seconds
later, with a perfect fix in view the entire time.

**Rejected — permanently widen the gate after N rejections.** The outlier protection is then spent
once and gone. Covariance inflation self-heals as fixes are folded.

**The scenario this exists for is not academic in VEX.** An opponent shoves the robot. The wheels
do not turn, so the odometry reports no travel, the process noise stays tiny, and the estimate is
thirty inches wrong **and certain** — the one state from which a covariance gate cannot recover on
its own. That is the test.

---

## 4. T3 — which tier ships as the default, decided on the measurement

**RULING: `ComplementaryFusion` stays the shipped default. Both tiers remain selectable and both
stay tested.** The choice is one constructor argument at the call site.

The evidence, measured before the ruling and reported whichever way it fell. Eight seeds of a
60-second run under full A3 hostility, both estimators on **one plant reading one sensor stream,
tick for tick**, differing in exactly one constructor argument (inches):

| seed | EKF final / worst | complementary final / worst |
|---|---|---|
| 1 | 0.371 / 0.792 | 0.043 / 0.594 |
| 2 | 0.182 / 0.772 | 0.174 / 0.674 |
| 3 | 0.842 / 0.864 | 0.771 / 0.771 |
| 4 | 0.189 / 0.670 | 0.164 / 0.478 |
| 5 | 0.356 / 0.802 | 0.095 / 0.385 |
| 6 | 0.150 / 0.716 | 0.310 / 0.751 |
| 7 | 0.337 / 0.668 | 0.122 / 0.477 |
| 8 | 0.380 / 0.707 | 0.118 / 0.563 |
| **mean** | **0.351 / 0.749** | **0.225 / 0.587** |

**The EKF does not beat the complementary filter. It loses, on 7 of 8 seeds.** `build-order.md`'s
DoD asked for the opposite. **It was not tuned toward** — these are the first numbers measured,
with the defaults exactly as designed, and the brief is explicit that fitting Q and R until the
sweep passes is fitting the test rather than the filter.

The reason is worth understanding rather than tuning away, and it is E2's reason one layer up.
**Dead-reckoning in this simulation is already sub-inch over a minute**, because A3's slip model
degrades the *driven* wheels while the unpowered tracking wheels read true body travel. The
modelled GPS noise (0.7″/axis, HA-26) is therefore **larger than the drift it is correcting**.
Against a sensor noisier than the error it removes, the right move is mostly to ignore it — and a
blunt fixed gain of 0.15 ignores it slightly harder than a filter that, correctly given the noise
model it was handed, trusts the sensor about as much as that model says it deserves. Both tiers
finish inside four tenths of an inch; **the gap between them is smaller than the noise on either.**
Tellingly, the EKF's Mahalanobis gate rejected **zero** fixes across all eight runs: nothing in an
ordinary run is anomalous, which is correct behaviour and is exactly why this metric cannot
separate the tiers.

Two further arguments for the same conclusion, both from the brief and both still true:

- **Every noise number the EKF uses is a guess** (HA-83…HA-91) until R4 measures hardware. A
  filter whose optimality argument rests on nine invented constants is not obviously safer than a
  filter that makes no optimality claim at all.
- **A filter a student can explain out loud is worth something real on a competition team.** The
  complementary tier is a gate and a gain. The EKF is a covariance propagation.

**So what is the EKF for?** Capability the other tier does not have at all, and the record should
say this plainly rather than leaving "it lost" as the headline:

1. **Recovery from a displacement past the fixed 12-inch ceiling.** 20-inch wound, six seeds: EKF
   back under 2 inches on all six, complementary still over 15 inches on all six.
2. **Resolving two disagreeing sources by their stated σ** — see §5.
3. **Saying how wrong it might be**, which is what makes both of the above possible and what fills
   `covarianceTrace` and `gateMahalanobis`.

And it costs nothing measurable where it does not help: an eighth of an inch on a clean run, and
**nothing at all in dead-reckoning** — off-strip over 1246 inches of path the two tiers' estimates
differ by at most **0.141 inch**, and the EKF is in fact slightly *better* on 3 of 4 seeds because
the velocity filter smooths encoder quantization. Not claimed as a win; reported.

---

## 5. T4 and T5 — the two debts E2 left, paid

### T4 — the 12-inch ceiling is REPLACED on the EKF tier and KEPT on the complementary tier

E2 recorded live that an estimate 29 inches from truth **never recovered** with a perfectly good
GPS in view, and named the cause: `ComplementaryFusion::innovationGate`, a fixed 12 inches applied
after the corrector's own gate.

**RULING: replaced on the EKF tier by `ν = √(rᵀ S⁻¹ r) > gateSigma` with `S = H P Hᵀ + R`; kept
unchanged on the complementary tier.** A fixed distance is exactly what a covariance replaces.
Measured, same fix, same distance, one filter:

- estimate fresh: **ν = 19.74 → rejected** (no sensor is that trustworthy against a good estimate)
- after 360 inches of dead-reckoning: **ν = 2.70 → accepted** (by then 20 inches is well inside
  what the estimator admits it might be off by)

The complementary tier keeps its 12-inch gate because it has **no `P` to normalise by**, and E2's
T1 already ruled that a distance normalised by an assumed constant must not be called a Mahalanobis
distance. Pinned by a test that drives the complementary tier through both a 20-inch rejection and
a 2-inch acceptance and asserts `RejectedInnovation` and a zero Mahalanobis on both.

**Stated honestly, because the claim is narrower than it sounds:** there are **two** gates between
a GPS reading and the fused pose, and E4 replaced one of them. Run with both in place, a 20-inch
wound is refused at the *corrector* as well — measured, 19 fixes reached the policy in 25 seconds
— so the accuracy test opens the corrector's gate to isolate the fusion layer, and says so in its
own header. **A real recovery from a large error needs both layers to agree**, and the corrector's
half is E2's `driftStdDevPerInch` (HA-67), which is another guess, and R4's to settle.

### T5 — `gateMahalanobis` and `covarianceTrace` become real

**RULING: both are populated only from a real innovation covariance, never from a plausible-looking
scalar.**

- **`gateMahalanobis`** is `√(rᵀ S⁻¹ r)` for the fix the verdict was rendered on — the most trusted
  accepted proposal if any passed, else the first rejected one. Re-derived in the test from the
  filter's own exposed covariance and the proposal's own σ, by an implementation of the formula
  written in the test file, and separately asserted **not** to equal `|r|` or `|r|/σ` — the two
  look-alikes E2 refused to write.
- **`covarianceTrace`** is `P[px][px] + P[py][py]`, in **square inches** — the position block only.
  A trace over a state vector mixing inches, radians and inches-per-second is a number with no unit
  and no meaning; a reader wanting a 1σ radius takes `√(trace/2)`, and chapter 11 says so. The
  complementary tier's documented use of the same slot (its scalar trust weight) is unchanged, so
  old blackboxes keep their meaning; the `reason` column says which filter wrote it.
- **`RejectedMahalanobis`** is raised for the first time since A1 reserved it, and only by a tier
  that estimates a `P`.

Both survive the wire: **69 accepted and 49 Mahalanobis-rejected ticks decoded from real blackbox
bytes**, with accepted ν asserted under the gate and rejected ν over it, worst ν = 49.02.

---

## 6. The decision docket

**D1 — the odometry increment is a measurement of BODY-frame velocity, not of position, and its
gain's position and heading rows are zeroed.** The wheels measure how far the wheels turned; they
say nothing directly about where the robot is. Folding a relative measurement as though it were an
absolute one shrinks the position covariance every tick, and **a position covariance that shrinks
while dead-reckoning is a filter that becomes certain as it becomes wrong** — after which no
absolute fix can ever pass the gate. That is E2's D2 gate-lockout failure arrived at from the other
side. *Rejected: fold `previousPosterior + u` as a pseudo-absolute position measurement* — no lag,
and it manufactures confidence the filter has not earned. Mutation-proven: written that way, 20
tests go red.

**The frame is not cosmetic, and getting it wrong cost 86 inches.** Written in the *field* frame
(`z = u/dt` against `h = R(θ)·v`) the innovation covariance picks up `|v|²·P_θθ`, which at the 30°
prior and 30 in/s is ≈246 (in/s)² — it swamps the measurement and drives the velocity gain to
nearly zero. **The filter then refuses to believe the wheels because it is unsure which way it is
facing**, and dead-reckons on a velocity it never updated: measured as an 86-inch gap against raw
odometry over 30 seconds. The sensitivity is fictitious — `PilonsOdometry` produced `u` by rotating
the wheels' body displacement through the same IMU heading θ is re-based to, so rotating it back is
exact. The real cost of a wrong heading is that the whole increment is rotated wrongly, and that is
accounted for where it belongs: the θ column of `F`.

**D2 — the velocity update happens BEFORE the position propagation.** `u/dt` is the *average*
velocity over the interval just ended, so it is the correct velocity to carry the position across
that same interval. *Rejected: the textbook predict-then-update order*, which lags the odometry by
a full tick at every change of speed. Consequence, measured: the EKF's dead-reckoning differs from
raw odometry by **0.69 inch worst-case over 30 seconds and 0.69 inch over 120 seconds** — the same
number, which is the non-cumulative property stated as a measurement rather than an argument.

**D3 — process noise on position is SYSTEMATIC (linear in distance), not a random walk.** Adding
`(k·Δtravel)²` per tick makes σ grow as `k·√(Σ Δtravel²)`, i.e. as the square root of distance.
Real odometry error does not cancel: a wheel diameter measured 1% small is 1% small on every tick
in the same direction. Measured with the random-walk form in place: **after 360 inches of
dead-reckoning the filter believed it was within half an inch**, and a truthful fix 20 inches away
was still rejected (ν = 10.4). Fixed by adding the increment of `(k·travelSinceFix)²`, which is the
same model E2's `driftStdDevPerInch` already uses one layer up. The gyro's drift-per-minute gets
the same treatment for the same reason.

**D4 — the standing-still floor is a random walk, and the distinction matters.** It stands for
small unmodelled disturbances with no preferred direction, so it is genuinely a random walk. Made
systematic, a robot standing perfectly still accumulates 30 inches of position doubt over a match
and then accepts a 30-inch lie as though it had earned it — which would have made the shove test
pass for entirely the wrong reason.

**D5 — the odometry channel is NOT gated.** The Mahalanobis gate exists to refuse an absolute fix
that disagrees with the filter; the odometry is the dead-reckoning input, not a fix. Gated, a
40 in/s launch against a converged velocity covariance is a Mahalanobis distance of about 13, so
**the filter rejected the wheels every time the robot changed speed hard**. *A gate on the
prediction channel is a filter that rejects reality for disagreeing with its model.*

**D6 — never-snap is enforced as a GAIN REDUCTION, not as a clip on the state.** The obvious
implementation — take the Kalman step, then clip the move — makes the filter lie: `P` shrinks as
though the full correction had been applied while the state sits where the clip left it, i.e. it
becomes confident *because* it was prevented from correcting. Scaling `K` instead and updating the
covariance with that gain is exactly consistent, because **the Joseph form is correct for any
gain, optimal or not** — which is its actual virtue and the reason it is used here rather than the
shorter `(I − K H) P`. That property is load-bearing three separate times: the rate clamp, the
zeroed θ row, and the velocity-only odometry update.

**D7 — the per-tick budget is charged for the whole tick's departure from the prediction**, not
only for the corrections. A position fix teaches the velocity states through the p–v
cross-covariance and that returns as movement on the next tick's propagation: measured, a
persistent correction stream inflated the published per-tick move to **0.16 inch against a 0.12
inch budget**. Charging the residual first makes the total `max(budget, residual)` rather than
`budget + residual`.

**The honest limit of that, corrected from an earlier claim of mine.** It does **not** make
`AppliedCorrection::dx/dy` obey the same bound under both tiers, and the live progress log records
me claiming it did and then striking it. Under the complementary tier that slot is the correction
and nothing else. Under the EKF it is the correction **or** the filtering residual, whichever is
larger, and the residual was measured reaching **0.1307 inch against a 0.12 inch budget — 9% over**
— during the hardest direction changes of the plant script. That excess is the estimate tracking
real motion through a one-tick filter, not a snap; the **correction** never exceeds the budget
(worst measured 0.119995). Chapter 11 tells a reader to apply that allowance under this tier, and
the test asserts a 25% allowance so a regression still shows.

**D8 — proposals are folded as sequential updates in ascending σ, most trusted first.** For
independent measurements sequential updates are equivalent to a batch update, and unlike a batch
update they let each proposal be gated on its own merits. Most-trusted-first is deliberate: a good
fix tightens `P` before a doubtful one is tested against it. *Rejected: averaging the proposals* —
E3's D1 already ruled that averaging **hides disagreement**.

**D9 — `confidence` is NOT a fusion weight.** E2 *derives* its confidence from σ (it is the scalar
Kalman gain `σ_dr²/(σ_dr² + σ_meas²)`), so weighting by both counts the same information twice.
This tier weights by the stated σ and by nothing else — **which is precisely the difference between
a covariance and the gain knob that HA-66 and HA-78 have been wearing a covariance's clothes as.**
`confidence` survives for exactly one purpose: it is passed back as `appliedConfidence`, which is
Localizer bookkeeping (how much of the drift accumulator an applied fix clears), not fusion
weighting.

**D10 — no `headingStdDev` appended to `CorrectionProposal`**, which E3's handoff suggested E4
would do. **No corrector in the tree can state one**, and "nothing reads it today" is exactly how a
field becomes load-bearing by accident — E2's T3, in reverse. Heading measurements use one
configured σ (HA-88, and the register says plainly that it is the weakest entry in the group). The
field stays a named handoff for whoever adds a corrector that can measure it. *Rejected: scale the
heading σ by `confidence`* — that invents a relationship between a [0,1] trust scalar and a
variance, and it would be inconsistent with the position channel, which ignores confidence.

**D11 — the policy takes no clock.** `dt` already comes from the Localizer's injected clock, so a
second time source could only disagree with the first. Elapsed time for the re-init cooldown is
accumulated from `dt`. (E2's D5 in reverse: there the corrector needed `now()` because `dt` was a
difference someone else had already taken; here `dt` is the whole quantity.)

**D12 — `fuse()` never throws and never allocates.** All preconditions are in the constructor;
every runtime pathology is screened and counted (`numericGuardTrips()`, `resyncCount()`) rather
than raised, because a fusion policy runs inside the control loop and the estimate must degrade
rather than die (F4's posture).

---

## 7. Test evidence

**48 new cases.** Every one names, in a comment, the bug it would catch.

### The trap, in its E4 form

The recurring failure — a truth model that shares a motion model with the code under test, so an
error cancels and the sweep proves nothing — has bitten C1, C3, C4, E2 and E3. Here the truth
generator integrates the body twist **exactly, in closed form, as a circular arc**:

```text
Δp_body = [ sin(ωh)/ω·vx − (1−cos(ωh))/ω·vy ,  (1−cos(ωh))/ω·vx + sin(ωh)/ω·vy ]   then R(θ₀)·
```

while the filter takes a **first-order Euler step at the end heading**. They agree only in the
limit; at ω = 4 rad/s and h = 10 ms they differ in the fourth decimal every tick. Nothing in the
test files computes a truth pose using anything the filter uses.

### `ekf_fusion_test.cpp` — 33 cases

**The covariance invariant sweep** — 3 trajectories × 4 seeds × 3 noise levels × 600 ticks =
**21,600 ticks**, each checked for exact symmetry (`==` on doubles, which the filter's explicit
symmetrization makes the correct bar — the Joseph products are *not* bit-symmetric on their own,
which is what lets this assertion see the symmetrization disappear), positive-definiteness via an
**independent Cholesky written in the test**, and a finite bounded trace.

**The gate** — the same 20-inch fix rejected at ν = 19.74 when fresh and accepted at ν = 2.70 after
360 inches blind; process noise proven to scale with travel rather than time, and to scale
**linearly** (doubling the distance more than triples the variance growth, where a random walk
would only double it).

**Outlier rejection** — 2000 consecutive confident 50-inch lies moved the estimate **0 inches**,
with the covariance still bounded afterwards; non-finite and zero-σ proposals rejected without
entering the state.

**Arbitration, the capability that justifies the chunk** — with an uninformative prior (the regime
where the algebra is exact), two fixes 20 inches apart with σ = 1 and σ = 5 fuse to **10.7692**
against an inverse-variance weighted mean of **10.7692** computed in the test from the σ values
alone; asserted *not* to be the first proposal, not the more confident one, and not the midpoint.
Plus the exact symmetry under swapping the σ values, the exact midpoint at equal σ, monotonicity
across six σ ratios, and a case proving a gated proposal contributes nothing.

**Re-init** — the shove scenario; the negative control with re-init disabled (still 30 inches out
after 30 seconds); ordinary noise over 6000 ticks producing zero declarations; and the rate limit
asserted as the property itself (30 seconds of alternating hijacks, every gap ≥ the cooldown).

**Never-snap** — 4000 ticks with a fix always far enough that the optimal step exceeds the budget;
four simultaneous proposals unable to out-vote the budget; three simultaneous heading proposals
unable to out-vote the heading budget.

**T1** — heading bit-identity across 1500 ticks with position fixes landing throughout;
accumulation and convergence from a 4° error (3.99981° after 15 s, monotone, no overshoot); and
E3's own trap re-run — a 5° lie kept **inside** the gate, where only `providesHeading` can stop it.

**A from-scratch algebra oracle** — the whole documented tick (Q, the odometry update, the `F`
propagation, the fix, both Joseph updates, the gain clamp) re-implemented in the test file with
plain loops and no shared code, compared entry by entry against all 5 states and all 25 covariance
entries, in two subcases: unclamped, and with the rate clamp binding so the **reduced-gain** path
is what is being compared. Its scope is stated in the file: it catches transcription and structural
error, and it cannot validate the equations, which is the invariants' and the plant comparison's
job.

### `ekf_fusion_seam_test.cpp` — 8 cases

One rig driven twice with **exactly one constructor argument different**: quality class and
dead-reckoning flag asserted identical on all 400 ticks (and proven non-vacuous — the script really
visits more than one state); the A3 boot guard and settle window; `setPose`; heading bit-identity
through the real re-stamp; `AppliedCorrection` inside the budget on 1500 ticks; both new numeric
slots decoded from real bytes; the re-init declared exactly once in a 700-tick file with the trace
jumping beside it; and the complementary tier proven **unchanged** — still a 12-inch gate, still
`RejectedInnovation`, still a zero Mahalanobis.

### `ekf_fusion_accuracy_test.cpp` — 4 cases

The comparison, with the metric written into `E4-PROGRESS.md` at 03:26 **before any number
existed** and the results block left as `PENDING MEASUREMENT — DO NOT FILL IN BY HAND` until the
run. §4 has the table. ~279,000 assertions, most of them the per-tick never-snap bound on both
tiers across every seed.

### `ekf_fusion_cost_test.cpp` — 3 cases: the cost is PINNED, not asserted

**0 allocations across 20,000 `fuse()` ticks** with every branch live (applied on 19,999, gated on
19,997, heading applied on 19,999 — so this is not measuring an early-out), and **0 across 5,000
full `Localizer::update()` ticks** with two correctors and the heading path live. The counter is
borrowed from E3's cost file rather than re-replacing the global allocator (one replacement per
binary), and a positive control proves the instrument is alive first — without it, a silently
broken counter would make every assertion here vacuously true.

---

## 8. Mutations

`docs/internal/verify/verify-e4.sh`, gated on build success, hardened with E2's and E3's lessons
carried over from the start (never `git checkout`; a `trap` on INT/TERM/PIPE; a byte-compare after
the edit so a mutation that changed nothing cannot be scored; a loud non-zero exit on any SKIP).

**36 mutations executed: 34 RED, 2 GREEN (both recorded below with their measurements), 0
build-fail, 0 SKIPPED.** The six the brief requires are first and labelled: covariance symmetry broken · Joseph form dropped · Mahalanobis comparison inverted ·
process noise independent of travel · re-init fires silently · multi-proposal weighting broken.
All six red.

**The five defects this chunk actually found are re-armed as mutations**, so that if any of them
ever goes green again the test that caught it during development has left the suite.

### The two recorded GREEN, with their measurements

**GREEN 1 — the position rows of the VELOCITY update unblocked.** I first wrote this up as the most
dangerous hole in the chunk. **Then I measured it, and I was wrong about its severity.** Unblocking
those rows leaks only through the p–v cross-covariance, and the leak is tiny — over three blind
trajectories the position trace moves 223878 → 223877, 6765.24 → 6764.94 and 2469.77 → 2469.41,
i.e. **between 4e-6 and 1.5e-4 relative.** Any test tight enough to separate 2469.77 from 2469.41
would be pinning an invented constant, which this chunk explicitly refuses to do. So it is recorded
GREEN with those numbers, and **the harness gains the dangerous form the block actually guards
against** — the odometry folded as an absolute position measurement — which collapses the
covariance and turns **20 tests red**. The header now states the measured difference between the
two rather than implying the mild form is the dangerous one.

**GREEN 2 — the posterior finiteness guard removed.** I could not construct a route to a non-finite
posterior that is not caught earlier, either by the screen on the proposal or by the determinant
test inside the update. The guard stays as defence in depth, the mutation stays recorded green with
that argument, and a **hostile 16-config × 200-tick fuzz** pins the *system* property it stands for
(the state and covariance never go non-finite under any combination of extreme config and extreme
proposal) even though it cannot reach that particular branch.

### One line deleted rather than tested, and one mutation deleted with it

A guard zeroing `headingNudge` when nothing was applied **could never fire** — `headingSum`
accumulates only inside the branch that sets `headingApplied`, and the Localizer ignores the nudge
unless that flag is set. A defensive line no mutation can kill is a line that should not be there.
Removed, with the invariant stated in a comment instead — and the mutation targeting it removed
from the harness, with the reasoning left there as a comment so nobody re-adds the guard. (On the
run before the deletion it showed as a SKIP, which the harness exits non-zero for: a pattern that
is not found means a mutation did not run, and that is never allowed to pass quietly.)

### The five real holes, closed

1. **The heading budget is not spent across proposals.** Every heading case in the chunk used ONE
   heading-providing proposal, so the per-tick accounting was invisible. E3 explicitly anticipates
   two tag sources. Closed with three sources pulling the same way, every tick asserted.
2. **An accepted fix does not reset the consecutive-rejection run.** Nothing fed an accept and a
   reject on the same tick for long enough to accumulate 50. Closed with a full simulated minute of
   one honest source and one 49-inch liar, `consecutiveRejects()` asserted zero on all 6000 ticks —
   which is also the realistic two-corrector case and must not read as "the estimator is lost".
3. **Proposals folded in arrival order rather than ascending σ**, and *why* the strongest
   arbitration test cannot see it is the instructive part: it uses a deliberately uninformative
   prior, and **with an uninformative prior sequential Kalman updates are exactly order-independent
   — they are the information form, which commutes.** Order only matters once gating is in play.
   Closed with hand-checkable arithmetic where the **verdict flips**: prior P = 4 in², an honest
   σ = 0.2 fix folded first shrinks P to 0.0396, after which an 8-inch fix with σ = 2 has ν = 3.98
   and is refused; in arrival order the same fix sees S = 8, ν = 2.83, and walks in.
4. **The mean-innovation bar dropped from the re-init trigger** — a hole that only appeared on the
   re-run, because on the first pass that mutation had failed to compile and a build-fail proves
   nothing. Without the bar, fifty consecutive rejections of *any* size declare the estimator lost.
   The realistic case is a corrector with a small systematic bias — a lever arm half an inch off, a
   tag map entry two inches out — sitting just outside the gate forever. **That is a calibration
   problem, not a lost robot**, and an estimator that responded by throwing away its confidence
   would turn a small persistent error into a large intermittent one.
   Writing the test taught me something about the filter I had not predicted, and the test now
   asserts the real behaviour rather than the one I guessed: after 1326 consecutive rejections the
   estimator's own uncertainty had grown enough (the random-walk time floor) that two inches was no
   longer an outlier, so it **accepted the fix and converged on it**, gradually, at the per-tick
   rate. A small persistent disagreement resolves itself by the estimator becoming appropriately
   less sure. No re-init was needed and none happened — which is exactly why the size bar belongs
   in the trigger beside the count.
5. **A "the position covariance never shrinks while dead-reckoning" invariant that I wrote and that
   turned out to be false.** It failed on the *correct* code, with drops of up to 12 in² on the
   spinning and stop-start scripts — legitimately, because `F P Fᵀ` carries a position–heading
   cross-covariance that can be negative, so driving a curve can genuinely cancel uncertainty a
   previous curve created. The test now states the true pair: monotone on a straight blind drive,
   and a universal process-noise floor on any trajectory. **Writing the invariant down is what
   found out that it was wrong.**

---

## 9. What we know for certain, and what we do not

**Phase E closes here, and F1/F2 and Phase R will be read against this section.**

**KNOWN**

- `EkfFusion` implements `IFusionPolicy` behind the unchanged signature. `IFusionPolicy`,
  `ICorrector`, `IPoseSource`, `CorrectionProposal`, `FusionResult`, `AppliedCorrection`,
  `Localizer`, `ComplementaryFusion` and both correctors are **unmodified**. No caller changed.
  The only shared-file change in the whole chunk is one appended enum value.
- The `Localizer`'s quality class and dead-reckoning flag are **identical tick for tick** under
  both tiers across a 400-tick script that visits more than one state.
- With no heading-providing corrector the published heading is the raw IMU reading **bit for bit**,
  under the EKF as under the complementary tier, through the real `Localizer`.
- Two disagreeing fixes are combined at the **inverse-variance weighted mean**, matched to better
  than 0.2% against a value computed from the σ values alone.
- The gate scales with the filter's own uncertainty: the same 20-inch fix is refused at ν = 19.74
  when fresh and accepted at ν = 2.70 after 360 inches blind.
- 2000 consecutive confident 50-inch lies move the estimate **zero inches**.
- A 30-inch displacement with the wheels stationary is recovered in **2.95 seconds**, with **no
  tick exceeding the per-tick correction budget**, and the event is a declared, latched,
  rate-limited word in a **decoded blackbox file**.
- The covariance stays exactly symmetric and positive-definite across **21,600 swept ticks**.
- `fuse()` allocates **zero** times across 20,000 ticks, measured by replacing the global allocator
  with the instrument proved by a positive control.
- The EKF's dead-reckoning differs from raw odometry by **≤ 0.69 inch, and that number does not
  grow with run length.**

**NOT KNOWN, stated plainly**

- **The EKF is not more accurate than what it replaces.** 0.351″ vs 0.225″ mean final error over 8
  seeded 60-second runs; it loses on 7 of 8 seeds. It is not the default. Anyone quoting this
  chunk as an accuracy improvement is quoting it wrong.
- **Nine new constants are guesses** (HA-83…HA-91), and a Kalman filter is *only as good as its
  description of how wrong its sensors are*. This is a sharper dependence on invented numbers than
  any previous chunk has had: E2's and E3's constants set thresholds, and these set the filter's
  entire notion of optimality. **The structure is proven; the numbers are R4's.**
- **`covarianceTrace` is a real number computed from invented inputs.** Treat it as a *relative*
  signal — it grew, it shrank — not as a calibrated distance, until R4.
- **HA-88 (a flat 2° heading σ) is the weakest entry in the group.** It stands in for every
  heading-providing source at every range, where E3 used a trusted-range band instead.
- **Recovery from a large error needs both gates, and only one was fixed.** The corrector's own
  gate is untouched and its widening constant (HA-67) is another guess.
- **Nothing here has seen a robot, a GPS, or a camera.** Every behaviour was proven against
  simulated sensors whose noise, cadence and failure modes are invented.
- **The `< 1°` heading claim is exactly where A3 and E3 left it.** Nothing about HA-20 changed, and
  no sentence about it was edited.
- **`AppliedCorrection::dx/dy` does not mean quite the same thing under the two tiers** — measured
  9% over budget under the EKF during hard direction changes, from the filtering residual rather
  than from any correction.
- **No multi-tag triangulation.** E3 handed it here; the filter arbitrates *proposals*, and a
  corrector still emits one proposal.

---

## 10. Documentation contract — discharged

1. **Roadmap checkboxes** — the EKF line flipped `[x]` with **four caveats attached to the
   checkbox itself** (not the default; parameters invented; the T2 ruling; no hardware); the
   `gateMahalanobis`/`covarianceTrace` line added as `[x]`; the "innovation-bounded,
   covariance-weighted gated nudge" line moved `[~]` → `[x]` with its scope stated.
2. **"You are here"** — rewritten to lead with what the EKF **cannot** do, then what it can.
3. **`build-order.md`** — the E4 entry gains a RULED block recording the T2 resolution of the
   latent never-snap conflict *and* the fact that its own DoD line was not met and not tuned
   toward, so the next reader does not re-derive either.
4. **Design notes in the header** — `ekf_fusion.hpp` carries what a covariance buys in plain words,
   the state and why each piece is there, T1/T2/T4/T5 each with its rejected alternatives, the tick
   in order, why the Joseph form is load-bearing, the frame ruling with its measured cost, the
   never-snap accounting with its measured limit, and every invented constant tagged.
5. **Test evidence** — §7, with the independent-truth-model argument and the oracle's scope stated.
6. **Decisions** — §2–§5 (T1–T5) and §6 (D1–D12), each with its rejected alternative.
7. **Freeze Register** — **E4 freezes nothing.** It appends one value to one wire-stable vocabulary
   (`GateReason`, re-pinned by test) and adds one header. F9 remains H1's; F6 and F10 untouched.

**Guide, per the named scope:**

- **Chapter 3** gains *"Knowing how wrong you might be"* — the blind spot the covariance fills,
  **a covariance explained from scratch** (including why it is a table and not one number), the
  four things having one buys, the give-up-on-yourself behaviour and why it still does not
  teleport, and the two tiers with the honest reason the simpler one is still the default.
- **Chapter 11** — `RejectedMahalanobis` rewritten from "reserved, nothing writes it" to what it
  now means; `CovarianceReinit` added; a new reading habit; a section on the two formerly-empty
  numeric slots including how to turn `covarianceTrace` into a 1σ radius; a four-step *"Reading a
  re-init"*; and the measured `correctionDx/Dy` caveat.
- **Chapter 14** — the *"No Kalman filter"* bullet and the *"two disagreeing correctors are
  bounded, not resolved"* bullet both **deleted**, replaced by a section in E3's
  measured → measured-on-what → still-unmeasured order that **leads with the fact that the EKF is
  not the default and is slightly less accurate**. The *"correction does not rescue a badly wrong
  estimate"* bullet also went, folded in with its remaining half named.
- **Chapter 15** — covariance, Kalman filter, Mahalanobis distance, process noise.
- **README** — the localization bullet now names both tiers and says the EKF's noise parameters are
  guesses; suite counts refreshed.

No guide code block changed, so no example re-quoting was needed; `check-examples` passes.
**`check-removability` caught a real violation I had just created** — two sentences in
`docs/roadmap.md` referenced `build-order.md`, which is dropped on squash-merge. Rephrased.

### On the sentence that was hard to write

E3 recorded that everything true about it pulled toward optimism. E4's difficulty was the opposite
and, oddly, harder: everything true about the *feature* is genuine, and the headline number says it
lost. The temptation was to lead with the capabilities and let the accuracy result appear as a
caveat four paragraphs down, where a skimming reader would take "we built a Kalman filter" and stop.

The resolution was to put the losing number **first**, in the guide, in the roadmap's "you are
here", in the test file's own header, and in this document's §4 — and then explain why it loses,
because the explanation is genuinely interesting and does not weaken the case for shipping the
thing. A filter that is *not* the default, does *not* improve accuracy, and exists for three
specific capabilities is a smaller claim than "we built an EKF". It is also the true one.

---

## 11. Verification (run, with output as observed)

```text
cmake --build build/test -j$(nproc) && ./build/test/shulib_tests | tail -6
  [doctest] test cases:     915 |     915 passed | 0 failed | 3 skipped
  [doctest] assertions: 1521419 | 1521419 passed | 0 failed |
  [doctest] Status: SUCCESS!

GUARD1 PASS          (no pros/ include anywhere in include/shulib)
GUARD2 PASS          (no shulib/sim/ include outside sim/)
ARM GATE PASS        (115 headers, arm-none-eabi-g++ -std=gnu++20 -Werror …)

doc gates:  self-test PASS · check-coverage PASS · check-fresh PASS
            check-examples PASS · check-removability PASS

docs/internal/verify/verify-e4.sh
  E4 mutations: 34 RED (good), 2 GREEN (HOLES), 0 build-fail, 0 SKIPPED
  (36 executed; both green are recorded in §8 with the measurements that justify them)

register reconciliation, direction 1 (must print nothing):
  grep -rn "PROVISIONAL (A4" include/ test/ | grep -v "HA-[0-9]"   → empty
register reconciliation, direction 2 (every HA-83…HA-91 cited in ekf_fusion.hpp) → all found
```

---

## 12. Named handoffs

**→ F1 / F2 (sequencing, and the accuracy spec)**

- **The default estimator is unchanged.** Everything F1 and F2 build on behaves exactly as it did
  before this chunk unless someone passes `EkfFusion` explicitly.
- **`qualityClass()` means the same thing under both tiers** — pinned tick-for-tick by test. Gate
  motion on it as before.
- **If a routine wants the EKF, it wants it for recovery or arbitration**, not for accuracy. Say so
  where the choice is made.

**→ R2 (the vision adapter)**

- **A corrector that can state a heading σ makes HA-88 unnecessary.** `CorrectionProposal` has room
  for one appended trailing field, and E4 deliberately did not add it because nothing could fill
  it. If R2's adapter can characterise heading error against range, add the field and the filter's
  weighting improves the day it lands.

**→ R4 (characterization) — this is where the chunk's value is decided**

- **HA-83…HA-91 are the filter's entire notion of optimality.** Measure them in this order, because
  each one gates the next: HA-86 (odometry σ, from a hand-pushed known distance) → HA-85 (peak
  acceleration, which falls straight out of the R5 sysid ramp) → HA-83 (the *shape* of the
  dead-reckoning error against distance — linear or √d — which is a modelling question the plot
  answers, not just a constant) → HA-20/HA-84 (real IMU drift) → HA-87 (the gate width, chosen from
  the measured innovation distribution rather than from a table).
- **Re-run the eight-seed comparison with real numbers.** The result in §4 is a statement about a
  simulation whose GPS is noisier than its own dead-reckoning. On a real robot that ratio may
  invert, and if it does the T3 default should be revisited — **the ruling is on the evidence, not
  on a preference, so new evidence reopens it.**
- **HA-90/HA-91 need one cheap experiment**: count how often a healthy run produces consecutive
  rejections at all, and set the bar above the observed maximum.

**→ whoever writes the estimator's calibration routine (WS5, still open)**

- `EkfFusion` exposes `covariance(i,j)`, `positionCovarianceTrace()`, `reinitCount()`,
  `resyncCount()` and `numericGuardTrips()`. `numericGuardTrips()` should be **zero on every run**;
  a non-zero value means the filter caught something it could not reason about, and it is the first
  number to look at when an estimate goes strange.
