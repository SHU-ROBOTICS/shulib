# Chunk E4 — the 5-state SE(2) EKF

> **Phase E, chunk 4 of 4 — closes the phase.** Predecessors: E2 (`GpsCorrector`), E3
> (`AprilTagCorrector`).
> **The hardest remaining host chunk, and the one that inherits three named debts.**

**Workstream:** WS5 (localization) · **Milestone:** M3 · **Freezes:** none

---

## Why this chunk exists, and why it is *here*

Two correctors now exist, and the library **cannot weigh them against each other.** When the GPS and
the tags disagree, the complementary filter bounds the damage instead of resolving the
disagreement — chapter 14 says exactly that, in those words, as a live limitation. Resolving it
needs a covariance, and a covariance means a Kalman filter.

E4 is ordered last in the phase because *the EKF's job is to arbitrate between correctors, so it
needs correctors to exist.* It also inherits three debts its predecessors deliberately refused to
pay from inside a corrector:

| Debt | Left by | What E4 owes it |
|---|---|---|
| The **12″ innovation ceiling** in `ComplementaryFusion` | E2 (observed live: an estimate 29″ out never recovered with a good GPS in view) | A gate that scales with *uncertainty* rather than a fixed distance |
| **`gateMahalanobis` is 0** and `RejectedMahalanobis` is never raised | E2 (honestly, for want of a covariance) | The covariance that finally makes both real |
| Two correctors that **cannot be arbitrated** | E3 | The whole point of the filter |

---

## What already exists — the seam was built for exactly this

| Thing | Where | The part that constrains you |
|---|---|---|
| **`IFusionPolicy` — the swap point** | `localization/i_fusion_policy.hpp` | `fuse(predicted, valid, dt) → FusionResult`. Its header says a policy returns **corrected POSITION only**, because heading is re-stamped by the Localizer. |
| **`FusionResult`, incl. E3's `headingNudge`** | `localization/correction.hpp` | The additive heading path E3 established. Read how the Localizer accumulates it into a persistent bias. |
| `ComplementaryFusion` — **the fallback tier, which stays** | `localization/complementary_fusion.hpp` | build-order: *"the simpler filter is easier to get right and to explain, so it stays as the fallback."* **Do not delete it.** |
| `GateReason::RejectedMahalanobis` (= 3) | `diag/debug_record.hpp` | Reserved since A1, never yet raised. E4 is what earns it. |
| `DebugRecord::covarianceTrace`, `gateMahalanobis` | `diag/debug_record.hpp` | Declared and unpopulated, tagged `— E4`. **You are not adding fields.** |
| E2's and E3's rulings | `E2-COMPLETED.md`, `E3-COMPLETED.md` | E2's honest-naming and scoped-claim discipline; E3's persistent-bias structure |

**Read first:** `i_fusion_policy.hpp` **in full** (the position-only sentence is T1);
`correction.hpp` and `localizer.hpp` (E3's heading accumulator — T1 again);
`complementary_fusion.hpp` (what you must not break); `E2-COMPLETED.md` §T2 (how to size an
accuracy claim to the evidence — **the model for this chunk's DoD**).

---

## Five rulings

### T1 — the state has θ in it; the seam says a policy cannot own heading

`[px, py, θ, vx, vy]` includes heading. `IFusionPolicy`'s header says, flatly, that a policy
*"returns the corrected POSITION only — heading is re-stamped from the IMU by the Localizer
afterward, so a policy can never own heading."* E3 then added the **one** sanctioned heading path: a
bounded `headingNudge` accumulated into a persistent bias.

These can be reconciled, and the reconciliation is the chunk's most important design decision.
**The likely correct shape:** the EKF *tracks* θ internally because its motion model needs it (a
body-frame velocity cannot be integrated into a field-frame position without a heading), but it
**emits a bounded `headingNudge`, never an absolute heading**, and the Localizer's re-stamp stays
exactly where it is.

Rule it explicitly. If you conclude the EKF should own published heading, that is a **structural
change to M2's design and to E3's**, and it needs to be argued as such — not arrived at because the
matrix algebra was tidier that way.

### T2 — "consecutive-reject re-init" is a snap, and §13 #4 forbids snapping

This is a genuine conflict between two documented requirements, and it has been sitting in
`build-order.md` unnoticed since the roadmap was written.

Re-init exists to recover from a hijacked estimate — that is *precisely* a discontinuous jump, and
never-snap (§13 #4) is one of the project's oldest invariants. Both cannot hold unconditionally.

Decide the rule and write it down. The shape that seems defensible: a re-init is **not** a silent
recovery but a **declared, latched, telemetry-visible event** — its own `FaultCode` or
`GateReason`, visible in the blackbox, rate-limited, and requiring a high bar (N consecutive
rejections *and* a persistently high innovation). **An estimator that quietly teleports is worse
than one that stays wrong**, because a wrong-but-continuous estimate still produces sane motion
while a teleport produces a violent correction the moment the next motion runs.

Whatever you rule, the never-snap tests that exist must still pass, or you must explain precisely
which one you changed and why.

### T3 — which tier ships as the default?

The DoD says the EKF must beat the complementary filter. If it does, is it the default? Every
existing test in the tree currently runs the complementary filter, so this decision has a blast
radius far beyond this chunk.

Decide, and be conservative about it: the argument for keeping the *simpler, explainable* filter as
the shipped default until hardware exists is strong — its noise parameters are guesses (R4), and a
filter whose behaviour a student can explain is worth something real on a competition team. Whatever
you choose, **both tiers must remain selectable and both must stay tested.**

### T4 — does the 12″ ceiling survive?

E2 recorded it deliberately for you. A fixed-distance bound is exactly the thing a covariance
replaces: a fix 29″ away should be *acceptable* when the filter knows it is badly lost, and
*rejected* when the filter is confident. Rule on whether the EKF path keeps, replaces, or scales the
ceiling — and if the complementary tier keeps it, say so.

### T5 — `gateMahalanobis` finally becomes real

E2 left it 0 and refused to raise `RejectedMahalanobis`, because the assumption *is* the content.
E4 has a covariance, so both become honest. **Populate them — and only from a real innovation
covariance**, not from a plausible-looking scalar.

---

## Scope

### In
1. **`EkfFusion` implementing `IFusionPolicy`** — 5-state SE(2), Mahalanobis gating, process noise
   ∝ travel, the arbitration of multiple simultaneous proposals.
2. **Consecutive-reject re-init** per T2's ruling.
3. **`covarianceTrace` and `gateMahalanobis` populated**; `RejectedMahalanobis` raisable (T5).
4. **The structural invariants**, proven: covariance stays positive-definite and symmetric across a
   parameter sweep; gating rejects outliers without diverging; re-init recovers a hijacked estimate.
5. **The comparison** — EKF vs complementary on **identical seeded runs**, with the metric defined
   before it is measured (E2's T2 discipline).
6. **The swap changes nothing above the seam** — pin it.

### Out
- Real noise parameters → **R4**. E4 proves *structure*; the numbers are fitted on hardware.
- Deleting or degrading `ComplementaryFusion` → it stays, by decision.
- New correctors → E2/E3 are the set.
- Owning published heading → T1, unless explicitly argued.

### Explicitly rejected
- **A silent re-init.** T2.
- **Filling `gateMahalanobis` with a look-alike.** T5, and E2 already set this precedent.
- **Tuning Q and R until the sweep passes.** That is fitting the test, not the filter. The
  parameters are guesses; say so and register them.

---

## Test requirements

Every test names, in a comment, the bug it would catch.

- **Covariance invariants across a sweep**: symmetric, positive-definite, trace finite and bounded —
  over noise levels, seeds, and trajectories. A filter that loses positive-definiteness is broken
  even when its output looks fine.
- **Recovery of a known trajectory** from corrupted synthetic streams, within bounds, swept.
- **Outlier rejection without divergence** — repeated bad fixes must not inflate the state.
- **Re-init from a hijacked estimate** (T2), and proof it is visible, latched and rate-limited.
- **Arbitration**: GPS and tag proposals that *disagree* — the filter must weight them by their
  stated σ, not pick one or average blindly. **This is the capability that justifies the chunk;
  test it directly.**
- **The comparison** on identical seeds, per T3/T5's metric.
- **The swap is invisible above the seam** — the Localizer, correctors and every caller unchanged.
- **Never-snap** still holds for the non-re-init path, position and heading.
- **Cost** — the hot path is a fixed-size matrix update, no allocation. Pin it.

### Mutations
Break symmetry in the covariance update; drop the Joseph form (or whatever keeps it stable); invert
the Mahalanobis comparison; make process noise independent of travel; make re-init fire silently;
break the multi-proposal weighting so the second proposal is ignored.

**A mutation that stays GREEN is a hole — log it, close it with a test that fails alone, and give it
a prominent place in the record.** D3 found four, E1 two, E2 one, E3 three. Gate the runner on build
success, and **never pipe the runner into `head`** (E2 lost a header to SIGPIPE that way).

**The recurring trap, in its E4 form:** if the filter's prediction step and the test's truth model
share a motion model, an error in that model cancels and the sweep proves nothing. **Generate the
truth trajectory independently of the filter's prediction.** This has bitten C1, C3, C4, E2 and E3.

---

## Documentation

**Chapter 3** (knowing where you are) — the mental model changes again: from "bounded nudges" to
"weighted fusion with an uncertainty estimate". Explain a covariance in plain words, for someone who
has not met one. **Chapter 14** — the "no Kalman filter" limitation falls, and the "cannot resolve
disagreement" limitation with it; both must be replaced by what is *now* true and still unmeasured,
in E3's measured → measured-on-what → unmeasured order. **Chapter 11** — `gateMahalanobis` and the
re-init event become readable. **Glossary** — covariance, Mahalanobis distance, process noise.

Accuracy claims stay scoped to simulation with invented noise. **Do not let the EKF's arrival
upgrade the `< 1°` sentence** — nothing about HA-20 changed.

---

## Definition of Done

- [ ] `EkfFusion` implements `IFusionPolicy`; the swap is invisible above the seam
- [ ] T1–T5 ruled explicitly, each with its rejected alternative
- [ ] Covariance invariants hold across a parameter sweep
- [ ] Outlier rejection without divergence; re-init recovers a hijacked estimate, visibly
- [ ] **Multi-corrector arbitration tested directly** — disagreeing sources weighted by σ
- [ ] EKF vs complementary on identical seeds, metric defined *before* measurement
- [ ] `covarianceTrace` / `gateMahalanobis` populated from a real covariance
- [ ] `ComplementaryFusion` intact, selectable and still tested
- [ ] Truth model independent of the filter's prediction model
- [ ] No allocation on the hot path; cost pinned
- [ ] Guide updated; accuracy claims scoped to simulation; `< 1°` sentence unchanged
- [ ] Invented parameters registered `HA-nn`
- [ ] Suite green; both guards; ARM gate; all doc gates

---

## Live progress log — required

`docs/internal/chunks/E4-PROGRESS.md`, appended as work happens (`date +%H:%M:%S`).

---

## Documentation contract

All six, plus **`E4-COMPLETED.md`**. Give **T1 and T2 their own sections** — T1 is a structural
decision three chunks depend on, and T2 resolves a conflict between two documented invariants that
has been latent in the roadmap since it was written.

**This chunk closes Phase E. The completion record should state plainly what the estimator can and
cannot do now, because F1/F2 and Phase R will both be read against it.**

**Do not commit. Do not push.**

---

## Landmines

- **Don't let the EKF own published heading** without arguing it (T1).
- **Don't let re-init be silent** (T2). An estimator that quietly teleports is worse than one that
  stays wrong.
- **Don't delete the complementary tier.** It is the fallback by decision.
- **Don't fill `gateMahalanobis` with a look-alike** (T5).
- **Don't tune Q/R until the sweep passes.**
- **Don't share a motion model between the truth generator and the filter.**
- **Don't upgrade the `< 1°` claim.** Nothing measured changed.
