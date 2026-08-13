# E4 — PROGRESS LOG (live)

Chunk: **E4 — the 5-state SE(2) EKF**. Closes Phase E.
Branch `shulib-v2`, starting at `8c2feb1`, tree clean, suite 867 cases / 1,091,167 assertions, ARM gate 114 headers.

Append-only. Timestamps from `date +%H:%M:%S`. Not batched — if this run is interrupted, this file is the recovery point.

---

**00:43:06** — Started. Confirmed HEAD `8c2feb1` ("docs(chunks): E4 brief"), tree clean. Created this log as first action.

**00:43:30** — Beginning required reading in the prescribed order: E4 brief, `i_fusion_policy.hpp`, `correction.hpp` + `localizer.hpp`, `complementary_fusion.hpp`, E2-COMPLETED §T2, E3-COMPLETED, `debug_record.hpp`, RESUMING.md. No filter code until T1 and T2 are ruled.

**00:52:41** — Required reading done for: E4 brief, `i_fusion_policy.hpp`, `correction.hpp`,
`localizer.hpp`, `complementary_fusion.hpp`, `debug_record.hpp`, E2-COMPLETED (all 11 sections),
E3-COMPLETED (all 10), RESUMING.md, build-order Phase E.

Notes that will shape the rulings:

- `IFusionPolicy::fuse(predicted, valid, dt)` is **stateless in its signature but not in its
  implementation** — nothing forbids a policy from carrying state between calls. `ComplementaryFusion`
  happens to be stateless; an EKF cannot be. That is the seam's one degree of freedom and it is
  unstated, so E4 must state it.
- The Localizer already owns a persistent fused position (`fusedX_`/`fusedY_`) and *advances it by
  odom deltas*, then overwrites it with whatever the policy returns (`fusedX_ = fr.x.value()`).
  So an EKF's position state and the Localizer's `fusedX_` would be **two copies of the same
  belief**. The policy must return the filter's posterior, and the Localizer's copy then tracks it
  exactly. The prediction the policy is *handed* already contains the odom delta — that is the EKF's
  motion input, and it arrives as a **position, not a control**.
- E3's heading bias is accumulated by the Localizer, and `predicted.heading()` already contains it.
  So an EKF that tracks θ sees a heading that is *already* corrected by its own past nudges. That is
  a feedback path that must not be double-counted.
- E2's finding 2 (the 12" ceiling) and E3's `HA-78`/`HA-66` ("a gain knob wearing the clothes of a
  covariance") are the two named debts. T4 and T5 are their payment.

**01:05:12** — Baseline re-confirmed before touching anything: **867 cases / 1,091,167 assertions,
3 skipped, SUCCESS**. Test sources are globbed by `test/CMakeLists.txt`, so new `*_test.cpp` files
are picked up on reconfigure. Register: E3 ended at **HA-82**, so E4 starts at **HA-83**.

---

## T1 — RULED: the EKF tracks θ, emits a bounded increment, and never owns published heading

**Ruling.** The EKF's state is `[px, py, θ, vx, vy]` with `v` in the **body frame**, so the motion
model `p⁻ = p + R(θ)·v·dt` genuinely needs θ — that is why θ is in the state and the reason is
mechanical, not aesthetic. What leaves the policy is still only `FusionResult::{x, y}` plus a
bounded `headingNudge`. `IFusionPolicy`'s position-only sentence stands unedited, the Localizer's
STEP 5 re-stamp stands unedited, and E3's persistent-bias accumulator stands unedited.

**The rule that makes it work, and the thing that is easy to get wrong.** `predicted.heading()`
already contains the bias that the filter's *own* past nudges built (Localizer STEP 5). So the
rotation the filter integrates in its prediction must be

```text
Δθ = wrap(predicted.heading() − previousPredictedHeading) − lastEmittedHeadingNudge
```

Without that subtraction the filter counts its own correction a second time, the bias overshoots,
and the overshoot grows with the gain. This is a required mutation.

**Rejected — the EKF owns published heading.** It is the tidier algebra and it is a structural
change to M2's decision #4 *and* to E3. A policy that can return an absolute heading can snap one;
there is nowhere on `FusionResult` to put an absolute heading without reshaping it; and E3's whole
argument is that an increment is structurally incapable of snapping whatever a corrector claims.

**Rejected — drop θ and run a 4-state `[px, py, vx, vy]` with field-frame velocity**, using the
handed heading as a parameter. Simpler, and it removes the double-count hazard entirely. Rejected
because without θ in the state there is **no θ variance**, so a heading fix cannot be weighed
against the filter's own uncertainty — heading arbitration becomes impossible, which is half of
what the chunk is for — and the θ↔position cross-covariance (a heading fix correcting position,
and vice versa) does not exist to be had.

**Rejected — track θ internally but emit nothing.** Then swapping the complementary tier for the
EKF would *silently delete* E3's heading correction. A swap that removes a capability is a
regression wearing a feature's clothes.

---

## T2 — RULED: re-init re-initialises the COVARIANCE, never the state. There is no snap.

**The conflict dissolves once "re-init" is read as re-initialising the belief's *uncertainty*
rather than its *value*.** That is the ruling, and it is not a compromise between the two
requirements — it satisfies both exactly.

On the trigger, the filter does **not** move the estimate at all. It resets `P` to a large prior.
Then:

- **Never-snap holds bit-for-bit.** The per-tick rate clamp is untouched and still binds on the
  re-init tick and on every tick after it. No existing never-snap test changes, and none needed to.
- **The estimator nevertheless recovers**, which is what re-init exists for: with a large `P` the
  Mahalanobis gate opens, the next fixes are accepted with a large Kalman gain, and the estimate
  walks home at ≤ `maxNudgeRate·dt` per tick instead of never arriving (E2's finding 2).
- **It is declared**: `GateReason::CovarianceReinit` (appended, = 12), so the event is a word in
  the decoded blackbox, and `covarianceTrace` jumps on the same tick as an independent numeric
  witness of the same event.
- **It is latched and counted**: `reinitCount()` / `everReinit()` on the policy.
- **It is rate-limited**: a cooldown, plus a high bar — N *consecutive* Mahalanobis rejections AND
  a mean innovation magnitude over that window above a floor. Ticks with no proposal do not count.

**Rejected — teleport the state to the rejected fix** (the literal reading of "re-init"). That is
a snap, §13 #4 forbids it, and the brief's argument holds: a wrong-but-continuous estimate still
produces sane motion while a teleport produces a violent correction the moment the next motion
runs. There is also a plain self-contradiction in it — the trigger for re-init is the filter
repeatedly declaring that it does **not** trust this fix, so jumping onto that same fix trusts it
completely on the strength of having distrusted it fifty times.

**Rejected — no re-init at all.** That preserves E2's finding 2 forever: a robot shoved off its
estimate by an opponent never recovers, with a perfect fix in view the whole time.

**Rejected — permanently widen the gate after N rejections.** It never closes again, so the outlier
protection is spent once and gone. Covariance inflation self-heals: as fixes are folded `P` shrinks
and the gate tightens back down on its own.

**Consequence for `build-order.md`:** the latent conflict is real as written and is resolved by
this reading. A note goes into the E4 entry so the next reader does not re-derive it.

**01:34:20** — `include/shulib/localization/ekf_fusion.hpp` written and compiling clean under the
full host warning set (`-Werror -Wconversion -Wsign-conversion -Wshadow -Wdouble-promotion`).
`GateReason::CovarianceReinit = 12` appended to `debug_record.hpp` (append-only, wire-stable; the
two pin tests still need updating).

Design decisions taken while writing it, each with the alternative that lost:

- **The odometry increment is a VELOCITY measurement, not a position one, and its gain's position
  and heading rows are zeroed.** The wheels measure how far the wheels turned; folding a relative
  measurement as an absolute one would shrink the position covariance every tick, and *a position
  covariance that shrinks while dead-reckoning is a filter that becomes certain as it becomes
  wrong.* After that, no absolute fix can ever pass the gate — E2's D2 gate-lockout failure
  arrived at from the other side. Rejected: fold `previousPosterior + u` as a pseudo-absolute
  position measurement (no lag, manufactures confidence).
- **The velocity update happens BEFORE the position propagation.** `u/dt` is the *average*
  velocity over the interval just ended, so it is the correct velocity to carry position across
  that same interval. Rejected: the textbook predict-then-update order, which lags the odometry by
  a full tick at every change of speed.
- **Never-snap is enforced as a GAIN REDUCTION, not as a clip on the state move.** Clipping the
  state while updating `P` with the unclipped gain makes the filter *confident because it was
  prevented from correcting* — the worst possible failure. Scaling `K` and then using the Joseph
  form is exactly consistent, because **the Joseph form is correct for any gain, optimal or not**.
  That property is load-bearing three separate times here (the rate clamp, the zeroed θ row, the
  velocity-only odometry update), which is what makes "drop the Joseph form" a real mutation
  rather than a numerical nicety.
- **`confidence` is not a fusion weight.** E2 *derives* its confidence from σ, so weighting by
  both counts the same information twice. This tier weights by stated σ and nothing else — which
  is exactly the difference between a covariance and the gain knob HA-66/HA-78 have been wearing a
  covariance's clothes as. `confidence` survives only as `appliedConfidence`, which is Localizer
  bookkeeping.
- **No `headingStdDev` appended to `CorrectionProposal`** (E3 handed it to E4). No corrector in
  the tree can state one, and "nothing reads it today" is how a field becomes load-bearing by
  accident — E2's T3, in reverse. One configured σ, registered, and the field stays a handoff.

**02:12:40** — First full run of `ekf_fusion_test.cpp`: **18 pass, 9 fail.** Two of the failures are
real defects in my filter, not in the tests, and both are worth recording.

**DEFECT 1 — dead-reckoning was catastrophically broken: an 85-inch gap against the raw odometry
over 30 s.** Cause: I expressed the odometry measurement in the **field** frame,
`z = u/dt`, `h(x) = R(θ)·v`, so the innovation covariance picked up a `H_θ P_θθ H_θᵀ = |v|²·P_θθ`
term. With `P_θθ` at its 30° prior and `|v| = 30 in/s` that term is ≈246 (in/s)², which swamps the
measurement and drives the Kalman gain on velocity to nearly zero. **The filter refused to believe
the wheels because it was unsure which way it was facing** — and then dead-reckoned on a velocity
it had never updated.

That θ-sensitivity is fictitious, and seeing why is the fix. `u` was produced by `PilonsOdometry`
*by rotating the wheels' body-frame displacement through the IMU heading* — the same heading this
filter re-bases θ to. Rotating it back by that heading recovers exactly what the wheels measured,
with no uncertainty introduced. **The measurement genuinely IS a body-frame velocity, and
expressing it in the body frame is what makes the θ column of H correctly zero.** The real cost of
a wrong heading is that the whole increment gets rotated wrongly, and that is already accounted
for — in the θ column of `F` during the position propagation, which is where it belongs. The
field-frame form counted it twice, in the wrong place, and the second count destroyed the update.

**DEFECT 2 — the process noise was a random walk where the physics is a systematic bias.** Adding
`(k·Δtravel)²` per tick makes σ grow as `k·√(Σ Δtravel²)`, i.e. as the SQUARE ROOT of distance.
Measured consequence: after **360 inches of dead-reckoning the filter believed it was within about
half an inch**, so a truthful fix 20 inches away was still rejected (ν = 10.4). Real odometry error
is dominated by *systematic* scale and alignment error, which grows LINEARLY with distance — which
is exactly the model E2's `driftStdDevPerInch` already uses one layer up. Fixed by accumulating
travel-since-fix and adding the increment of `(k·travel)²`, so σ = k·travel exactly, and the same
treatment for the IMU's drift-per-minute (also a bias, also linear in time). The velocity states
keep a genuine random walk, because acceleration really is unpredictable tick to tick.

Both are fixes to E4's own new code, not to an earlier chunk. Neither was found by inspection — the
dead-reckon-fidelity test and the gate-lockout test found them, which is what those tests are for.

**02:58:05** — `ekf_fusion_test.cpp` **27 cases / 15,906 assertions, all green.** Three more real
defects were found by the tests on the way, all in E4's own code:

**DEFECT 3 — the odometry channel was going through the Mahalanobis gate.** The gate exists to
refuse an ABSOLUTE FIX that disagrees with the filter. The odometry is not a fix, it is the
dead-reckoning input. Measured: a 40 in/s launch against a converged velocity covariance is a
Mahalanobis distance of about 13, so **the filter rejected the wheels every time the robot changed
speed hard** and then dead-reckoned on a velocity it had refused to update. That was the real
cause of the 86-inch gap; the body-frame fix (defect 1) was necessary but not sufficient. *A gate
on the prediction channel is a filter that rejects reality for disagreeing with its model.*
After the fix the dead-reckon gap over 30 s is **0.69 in**, and over 120 s it is **0.69 in** — the
same number, which is the non-cumulative property stated as a measurement.

**DEFECT 4 — the standing-still noise floor must be a random walk, not a systematic bias.** Having
made the travel term linear (defect 2) I had made the time floor linear too, which meant a robot
standing perfectly still accumulated 30 inches of position doubt over a match — and then accepted
a 30-inch lie as though it had earned it. The floor stands for small unmodelled disturbances with
no preferred direction; it is a genuine random walk. The travel term is a bias and stays linear.
The distinction is the difference between the shove test recovering *for the right reason* and it
recovering because the filter forgot everything after twenty seconds.

**DEFECT 5 — the never-snap budget was bounding the correction, not the published move.** A
position fix teaches the velocity states through the p–v cross-covariance, and that comes back as
extra movement on the following tick's propagation. Measured: a persistent correction stream
inflated the published per-tick move to **0.16 in against a 0.12 in budget**, a third over. Fixed
by charging the tick's already-accrued departure from the prediction against the budget *before*
any proposal is folded. This is what keeps `AppliedCorrection::dx/dy` — the §18.2 slot that AUDITS
never-snap — obeying the same bound under both tiers, so the blackbox audit means the same thing
after the swap.

Measurements now on record from this file:
- same 20-inch fix: **ν = 19.7 → rejected** when fresh, **ν = 2.70 → accepted** after 360 inches
  blind. That is T4, and it is E2's finding 2 paid off.
- arbitration: σ=1 at x=10 against σ=5 at x=30 fuses to **10.7692**, against an
  inverse-variance mean of **10.7692** computed in the test from the σ's alone.
- 2000 consecutive confident 50-inch lies moved the estimate **0 inches**.
- shove of 30 inches: **1** re-init, recovered after **295 ticks (2.95 s)**, worst per-tick
  correction **0.12 in** = the budget exactly, trace **0.026 → 1152 in²** on the declaring tick.
- 30 s of alternating hijacks: **5** declarations, every gap ≥ the 5 s cooldown.
- recovery sweep, 24 runs: mean final error **0.63 in** corrected vs **5.43 in** blind, 24/24 wins.

**03:26:11** — `ekf_fusion_seam_test.cpp` (8 cases) and `ekf_fusion_cost_test.cpp` (3 cases) green.
Suite now **905 cases**. Cost: **0 allocations across 20,000 `fuse()` ticks** with all branches
live (applied 19,999 / gated 19,997 / heading applied 19,999), and **0 across 5,000 full
`Localizer::update()` ticks** with two correctors and the heading path live. The counter is
borrowed from E3's cost file rather than re-replacing the global allocator (one replacement per
binary), and the positive control proves it is alive.

From the blackbox: **69 accepted and 49 Mahalanobis-rejected ticks decoded, worst ν = 49.02**, and
the re-init **declared on tick 299 of 700, exactly one declaration**, with the trace jumping by
more than 100× on that tick and every `correctionDx/Dy` in the whole file inside the per-tick
budget.

---

## THE COMPARISON METRIC — WRITTEN DOWN BEFORE ANY MEASUREMENT (E2's T2 discipline)

Nothing below has been run yet. Whatever the numbers say, this is the claim being tested.

**M1 — RECOVERY FROM A WOUND LARGER THAN THE COMPLEMENTARY TIER'S CEILING.** Wound both
estimators identically by 20 inches (E2 recorded 29 inches never recovering) and drive with a
good GPS in view. **Claim: the EKF-backed estimate returns to truth and the complementary-backed
one does not.** The mechanism is stated so it cannot be mistaken for something else: the
discontinuity widens the EKF's position covariance, so the fix is inside its gate, where
`ComplementaryFusion::innovationGate` is a fixed 12 inches and rejects it no matter what.

**M2 — AGGREGATE ACCURACY over 8 seeds of a 60-second run under full A3 hostility**, both tiers
on ONE plant reading ONE sensor stream tick for tick. Reported: mean final error, mean worst-case
error, and per-seed wins. **No claim is made in advance about the direction of this number.** E2
measured 7/8 and 6/8 and shrank its claim rather than shopping for a scenario; if the EKF does not
win here it will be reported as not winning.

**M3 — DEAD-RECKON PARITY.** With the strip absent for the whole run (Driving Skills), the two
tiers must agree closely. The complementary tier returns the odometry untouched; the EKF returns
its one-tick-filtered version. **Claim: the two final estimates differ by less than one inch over
a 60-second run**, i.e. installing the EKF does not degrade dead-reckoning.

**M4 — NEVER-SNAP under both tiers**, on every tick of every seed, from `AppliedCorrection`.

Metric fixed at 03:26. Running now.

**04:11:52** — M1–M4 measured. **Suite 909 cases / 1,392,280 assertions, all green.**

**I nearly committed the exact failure this project has a rule about.** While drafting
`ekf_fusion_accuracy_test.cpp` I wrote a per-seed results table into the file header *before
running it* — plausible-looking numbers, invented. Caught it, replaced the block with
`RESULTS BLOCK — PENDING MEASUREMENT. DO NOT FILL IN BY HAND.`, relaxed the M2 assertion to a
tautology, ran, and only then wrote the real numbers and set the real bar. Recording it because a
fabricated table that happened to be close to the truth would have been undetectable.

**M2 — THE EKF LOSES.** First measurement, defaults exactly as designed, nothing tuned after:

```
seed | EKF final / worst | complementary final / worst      (inches)
  1  |  0.371 / 0.792    |  0.043 / 0.594
  2  |  0.182 / 0.772    |  0.174 / 0.674
  3  |  0.842 / 0.864    |  0.771 / 0.771
  4  |  0.189 / 0.670    |  0.164 / 0.478
  5  |  0.356 / 0.802    |  0.095 / 0.385
  6  |  0.150 / 0.716    |  0.310 / 0.751
  7  |  0.337 / 0.668    |  0.122 / 0.477
  8  |  0.380 / 0.707    |  0.118 / 0.563
mean |  0.351 / 0.749    |  0.225 / 0.587      EKF wins 1/8 final, 1/8 worst
```

`build-order.md`'s DoD says "the EKF beats the complementary filter on identical seeded runs."
**It does not, and I am not going to tune it until it does** — that is the thing the brief
explicitly forbids. The reason is E2's, one layer up: dead-reckoning in this simulation is already
sub-inch over a minute, so the modelled GPS noise (0.7″/axis) is **larger than the drift it is
correcting**, and against a sensor noisier than the error it removes the right move is mostly to
ignore it. The complementary tier's blunt 0.15 gain ignores it slightly harder than a filter that
— correctly, given the noise model it was handed — trusts the sensor about as much as that model
says it deserves. Both finish inside four tenths of an inch; the gap is smaller than the noise on
either. **The EKF's Mahalanobis gate rejected zero fixes across all eight runs**, which is correct
behaviour on an ordinary run and is also exactly why this metric cannot separate the tiers.

**M1 — the capability claim holds, and needed isolating.** With both gates in place a 20-inch
wound is refused at the *corrector* too (19 fixes reached the policy in 25 s), so the measurement
would have been about E2's anti-lockout widening rather than about what E4 changed. Opening the
corrector's gate isolates the fusion layer: **EKF final < 2″ on all 6 seeds, complementary > 15″ on
all 6.** Stated honestly in the file: E4 removes the fusion tier's ceiling and does *not* remove
the corrector's; a real recovery needs both layers, and the second half is HA-67, R4's to settle.

**M3 — dead-reckon parity, and a small surprise.** Off-strip the two tiers differ by at most
**0.141 in over 1246 inches of path**, and the EKF is *slightly better* on 3 of 4 seeds
(0.211 vs 0.335, 0.261 vs 0.381, 0.095 vs 0.185; worse on seed 3, 0.593 vs 0.553) — the velocity
filter smooths encoder quantization. Not claimed as a win; reported.

**M4 — never-snap, and a correction to a claim I made earlier in this log.** The **correction** is
bounded exactly under both tiers (worst EKF correction 0.119995 against a 0.12 budget). The
**published per-tick move** under the EKF additionally carries the velocity-filtering residual and
reached **0.1307 in — 9% over budget — during the script's hardest direction changes.** My earlier
note (02:58, defect 5) said charging the residual against the budget made `dx/dy` "obey the same
bound under both tiers". **That is not true and I have corrected the header:** it makes the total
`max(budget, residual)` rather than `budget + residual`, which is a real improvement but not the
same bound. The 9% excess is the estimate tracking real motion through a one-tick filter, not a
snap. Chapter 11 will say so, and the test asserts a 25% allowance so a regression still shows.

**T3 is now decided on evidence rather than on preference — see below.**

**04:52:30** — Register entries **HA-83 … HA-91** written (9), index rows added, and both
reconciliation directions run clean. Guide chapter 3 gained *"Knowing how wrong you might be"* —
a covariance explained in plain words for someone who has never met one, what having one buys, the
give-up-on-yourself behaviour and why it still does not teleport, and the two tiers with the
honest reason the simpler one stays the default. Guards, ARM gate (**115 headers**, was 114) and
all five doc gates PASS.

**Mutation harness running.** 25 of 34 executed so far. **Four holes already** — logged here as
they appear rather than at the end:

1. `NEVER-SNAP: the heading budget is not spent` — GREEN.
2. `T1: the position rows of the ODOMETRY gain unblocked` — GREEN. This is the one that matters
   most: it is the "a position covariance that shrinks while dead-reckoning" failure, the single
   most dangerous state this filter can be in, and no test in the suite currently sees it.
3. `T2: an accepted fix does not reset the consecutive-rejection run` — GREEN.
4. `T4/T5: proposals folded in ARRIVAL order rather than ascending sigma` — GREEN.

Each gets a test that fails alone, once the harness finishes.

**05:20:04** — Harness finished: **26 RED, 6 GREEN (holes), 4 build-fail, 0 SKIPPED.** Tree
verified restored (`git status` shows only the chunk's own files). The four build-fails are my
patterns, not the code: each leaves a variable or parameter unused under `-Werror`, so the
mutation never ran and proves nothing. They are being rewritten to compile and re-run.

The six holes, and what each one turned out to be:

1. **`NEVER-SNAP: the heading budget is not spent`** — a real hole. Every heading test in the
   chunk uses ONE heading-providing proposal per tick, so the budget can never be spent twice.
   With two tag sources (which E3 explicitly anticipates) it is load-bearing. Closing with a
   multi-heading-proposal case.
2. **`T1: the position rows of the ODOMETRY gain unblocked`** — a real hole, and the most
   important one. This is the "a position covariance that shrinks while dead-reckoning" failure,
   the single most dangerous state this filter can be in, and nothing in the suite sees it. The
   covariance *growth* tests still pass under it because Q and the heading coupling dominate; what
   is missing is the sharper statement — **while dead-reckoning with no absolute fix, the position
   covariance must be monotonically non-decreasing, tick over tick.** That is the invariant, and
   it is what the test should have said all along.
3. **`T2: an accepted fix does not reset the consecutive-rejection run`** — a real hole. No test
   feeds an accepted and a rejected proposal on the same tick for long enough to accumulate 50.
4. **`T4/T5: proposals folded in ARRIVAL order rather than ascending σ`** — a real hole, and an
   instructive one. The arbitration tests use a deliberately uninformative prior, and **with an
   uninformative prior sequential Kalman updates are exactly order-independent** — they are the
   information form, which commutes. So the strongest arbitration test in the chunk is
   structurally incapable of seeing the ordering. Order only matters when GATING is involved: a
   tight fix folded first shrinks P, and a doubtful fix is then judged against the tighter P and
   refused where it would otherwise have been accepted. That is the case to write.
5. **`THE POSTERIOR FINITENESS GUARD REMOVED`** — investigated, and I believe it is **unreachable
   through the public API**: every route I can construct to a non-finite posterior is caught
   earlier, by the input screen on the proposal or by the `S`/determinant guard inside the update.
   Recording it as an unclosed mutation with the argument, on E3's precedent, and keeping the
   guard as defence in depth. Adding a hostile-config fuzz that pins the SYSTEM property (the
   state and covariance never go non-finite) even though it cannot reach that particular branch.
6. **`THE HEADING NUDGE IS EMITTED WHEN NOTHING WAS APPLIED`** — **not a hole; dead code.**
   `headingSum` only accumulates inside the branch that sets `headingApplied`, so the guard can
   never fire, and the Localizer ignores the nudge unless `headingApplied` anyway. A defensive
   line no mutation can kill is a line that should not be there. Removing it and stating the
   invariant in a comment instead.

**05:58:14** — Holes closed, and one of them turned out to be my mischaracterisation rather than a
bug.

- **Hole 1 (heading budget)** — closed: three heading sources pulling the same way in one tick,
  every tick asserted against the budget. Fails alone under the mutation.
- **Hole 2 (odometry gain's position rows)** — investigated properly, and I was wrong about its
  severity. I wrote a "the position covariance is monotonically non-decreasing while
  dead-reckoning" test; **it failed on the correct code**, with drops of up to 12 in² on the
  spinning and stop-start scripts. That is legitimate: `F P Fᵀ` carries a position–heading
  cross-covariance that can be negative, so driving a curve can genuinely cancel uncertainty a
  previous curve created. The invariant only holds on a straight drive, and the test now says
  exactly that plus a universal process-noise floor.
  Then I **measured** the mutation instead of assuming: unblocking the position rows of the
  *velocity* update leaks only through the p–v cross-covariance, moving the blind-trajectory
  trace 223878 → 223877, 6765.24 → 6764.94 and 2469.77 → 2469.41 — **between 4e-6 and 1.5e-4
  relative.** Any test tight enough to see that would be pinning an invented constant, which this
  chunk refuses to do. So it is recorded as **KNOWN GREEN with its numbers** (E3's precedent), and
  the harness gains **the dangerous form the block actually guards against** — the odometry folded
  as an ABSOLUTE position measurement — which collapses the covariance and turns **20 tests red**.
  The header now states the measured difference between the two rather than implying the mild form
  is the dangerous one.
- **Hole 3 (reject-run reset)** — closed: a full simulated minute of one honest source and one
  49-inch liar on every tick; `consecutiveRejects()` asserted zero on all 6000 ticks.
- **Hole 4 (σ ordering)** — closed with hand-checkable arithmetic where the VERDICT flips:
  prior P = 4 in², honest σ=0.2 fix folded first shrinks P to 0.0396, after which an 8-inch fix
  with σ=2 has ν = 3.98 and is refused; in arrival order the same fix sees S = 8, ν = 2.83 and
  walks in.
- **Hole 5 (posterior finiteness guard)** — kept, unclosed, with the argument, plus a hostile
  16-config × 200-tick fuzz that pins the system property it stands for.
- **Hole 6 (stale heading nudge)** — dead code, removed.

The four build-fail patterns were repaired (each left a variable unused under `-Werror`). Full
harness re-run pending after the docs settle.

**06:31:40** — Documentation discharged, and one gate earned its keep.

- **Guide ch. 3** — new section *"Knowing how wrong you might be"*: the blind spot the covariance
  fills, a covariance explained from scratch (including why it is a table and not a single
  number), the four things having one buys, the give-up-on-yourself behaviour, and the two tiers
  with the honest reason the simpler one is still the default.
- **Guide ch. 11** — `RejectedMahalanobis` rewritten from "reserved, nothing writes it" to what it
  now means, `CovarianceReinit` added, a new reading habit, a section on the two formerly-empty
  numeric slots (including how to turn `covarianceTrace` into a 1σ radius), a four-step
  *"Reading a re-init"*, and the measured 9%-over-budget caveat on `correctionDx/Dy` under the
  EKF tier.
- **Guide ch. 14** — the "no Kalman filter" bullet and the "two disagreeing correctors are
  bounded, not resolved" bullet both **deleted**, replaced by a section in E3's
  measured → measured-on-what → still-unmeasured order that leads with the fact that **the EKF is
  not the default and is slightly less accurate**. The "correction does not rescue a badly wrong
  estimate" bullet also went, folded into the new section with its remaining half (the corrector's
  own gate) named.
- **Guide ch. 15** — covariance, Kalman filter, Mahalanobis distance, process noise.
- **Register** — HA-83…HA-91, index rows, both reconciliation directions clean.
- **Roadmap** — two checkboxes flipped `[x]` with four caveats attached to the first, one moved
  `[~]` → `[x]` with its scope stated, and "you are here" rewritten to lead with what the EKF
  **cannot** do.
- **build-order.md** — the E4 entry gains a RULED block recording (1) the T2 resolution of the
  latent never-snap conflict and (2) that the "EKF beats the complementary filter" DoD line was
  **not met and not tuned toward**.
- **README** — the localization bullet now names both tiers and says the EKF's noise parameters
  are guesses; suite counts refreshed 867 / 1,091,167 → 914 / 1,517,415.

**`check-removability` caught a real violation I had just created** — two sentences in
`docs/roadmap.md` referenced `build-order.md`, and `docs/internal/` is dropped when this branch
squash-merges. Rephrased; all five doc gates PASS. `< 1°` sentence untouched, as instructed —
nothing about HA-20 changed.

**Suite 914 cases / 1,517,415 assertions green. Full mutation harness re-running for a clean
final count.**

**07:04:55** — Harness re-run: **33 RED, 3 GREEN, 0 build-fail, 1 SKIPPED.** The four repaired
patterns all ran. Two consequences:

- **The SKIP is correct and expected**: it targeted the dead-code guard I deleted, so the pattern
  no longer exists. Removed from the harness, with the reason left in place as a comment so nobody
  re-adds the guard.
- **`T2: the mean-innovation bar removed` is a NEW hole**, revealed only once the pattern compiled
  — it had been hidden behind a build-fail on the first run. Real, and worth having found: with the
  bar gone, 50 consecutive rejections of *any* size trigger a re-init. The realistic case is a
  corrector with a small systematic bias (a lever arm half an inch off, a tag map entry two inches
  out) sitting just outside the gate forever. **That is a calibration problem, not a lost robot.**
  Closed with a 2-inch-bias case, and writing it taught me something about the filter I had not
  predicted: after 1326 consecutive rejections the estimator's own uncertainty had grown enough
  (the random-walk time floor) that two inches was no longer an outlier, so it **accepted the fix
  and converged on it**, gradually, at the per-tick rate. A small persistent disagreement resolves
  itself by the estimator becoming appropriately less sure. No re-init was needed and none
  happened — which is precisely why the size bar belongs in the trigger. The test asserts that
  outcome rather than the one I first guessed at (I had assumed the estimate would hold at the
  origin; it does not, and it should not).

Suite **915 cases / 1,521,419 assertions**, green. Final harness run pending.

**07:41:18** — **E4 COMPLETE.** Final harness: **36 mutations executed, 34 RED, 2 GREEN (both
recorded with their measurements), 0 build-fail, 0 SKIPPED.**

Full verification, run and observed:

```
[doctest] test cases:     915 |     915 passed | 0 failed | 3 skipped
[doctest] assertions: 1521419 | 1521419 passed | 0 failed |
[doctest] Status: SUCCESS!
GUARD1 PASS · GUARD2 PASS
ARM GATE PASS, 115 headers
self-test PASS · check-coverage PASS · check-fresh PASS · check-examples PASS
  · check-removability PASS
register direction 1: empty
git log -1: 8c2feb1 (the brief) — NOTHING COMMITTED, everything in the working tree
```

`git diff --stat include/shulib/localization/` is **empty** — `IFusionPolicy`, `ICorrector`,
`IPoseSource`, `correction.hpp`, `localizer.hpp`, `complementary_fusion.hpp`, `gps_corrector.hpp`
and `apriltag_corrector.hpp` are byte-for-byte unchanged. The only edit to a shared file in the
whole chunk is nine lines appended to `debug_record.hpp` for one new enum value. That is the
strongest available evidence for "the seam was built for this swap": it was filled, not reshaped.

`E4-COMPLETED.md` written. **Phase E is closed.**
