<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/localization/complementary_fusion.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `complementary_fusion.hpp`

ComplementaryFusion — the M2 fusion policy.

This header declares **2** types (8 members).

Extracted from [`include/shulib/localization/complementary_fusion.hpp`](../../include/shulib/localization/complementary_fusion.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`struct ComplementaryFusionConfig`](#struct-complementaryfusionconfig)
  - [`maxNudgeRate`](#complementaryfusionconfig-maxnudgerate)
  - [`innovationGate`](#complementaryfusionconfig-innovationgate)
  - [`maxGain`](#complementaryfusionconfig-maxgain)
  - [`headingGate`](#complementaryfusionconfig-headinggate)
  - [`maxHeadingGain`](#complementaryfusionconfig-maxheadinggain)
  - [`maxHeadingNudgeRate`](#complementaryfusionconfig-maxheadingnudgerate)
- [`class ComplementaryFusion`](#class-complementaryfusion)
  - [`ComplementaryFusion`](#complementaryfusion-complementaryfusion)
  - [`fuse`](#complementaryfusion-fuse)

<a id="struct-complementaryfusionconfig"></a>

## `struct ComplementaryFusionConfig`

```cpp
struct ComplementaryFusionConfig
```

The six numbers that bound how hard an absolute fix may pull the estimate. Position and heading each get three of the same kind: a GATE (reject an innovation larger than this outright), a GAIN (the fraction of a surviving innovation taken per tick at confidence 1), and a per-tick budget expressed as a RATE, so the never-snap bound does not move when the loop rate does. Position and heading are configured separately because they are different measurements with different failure modes — a mirrored tag ruins the heading while leaving the position plausible. Every default is a conservative PROVISIONAL placeholder awaiting M3 tuning; the ctor rejects an out-of-range one loudly rather than clamping it.

*struct, declared at [`include/shulib/localization/complementary_fusion.hpp:66`](../../include/shulib/localization/complementary_fusion.hpp#L66).*

<a id="complementaryfusionconfig-maxnudgerate"></a>

### `ComplementaryFusionConfig::maxNudgeRate`

```cpp
units::Velocity maxNudgeRate{12.0}
```

Per-tick nudge budget as a RATE: the max position correction applied in a tick is `maxNudgeRate · dt`. Loop-rate-independent. (M3-tuned; conservative placeholder.)

*field, declared at [`include/shulib/localization/complementary_fusion.hpp:69`](../../include/shulib/localization/complementary_fusion.hpp#L69).*

<a id="complementaryfusionconfig-innovationgate"></a>

### `ComplementaryFusionConfig::innovationGate`

```cpp
units::Length innovationGate{12.0}
```

Reject a proposal whose |innovation| exceeds this — the never-snap gate. (M3-tuned.)

*field, declared at [`include/shulib/localization/complementary_fusion.hpp:71`](../../include/shulib/localization/complementary_fusion.hpp#L71).*

<a id="complementaryfusionconfig-maxgain"></a>

### `ComplementaryFusionConfig::maxGain`

```cpp
double maxGain = 0.15
```

Fraction of the innovation pulled per tick at confidence == 1, in (0,1]. (M3-tuned.)

*field, declared at [`include/shulib/localization/complementary_fusion.hpp:73`](../../include/shulib/localization/complementary_fusion.hpp#L73).*

<a id="complementaryfusionconfig-headinggate"></a>

### `ComplementaryFusionConfig::headingGate`

```cpp
units::AngleDim headingGate{15.0 * math::Angle::kPi / 180.0}
```

Reject a heading proposal whose |innovation| exceeds this — the never-snap gate for yaw. 15 degrees is ~15x the heading drift a 60-second match is expected to accumulate (the master plan's ~1 deg/min IMU figure, HA-20), so an innovation this large is far more likely to be a mirrored tag winding, a wrong tag-map entry or a misidentified id than real drift — and folding it would be worse than folding nothing. PROVISIONAL (A4: HA-80).

*field, declared at [`include/shulib/localization/complementary_fusion.hpp:82`](../../include/shulib/localization/complementary_fusion.hpp#L82).*

<a id="complementaryfusionconfig-maxheadinggain"></a>

### `ComplementaryFusionConfig::maxHeadingGain`

```cpp
double maxHeadingGain = 0.15
```

Fraction of the heading innovation pulled per tick at confidence == 1, in (0,1]. The regulator near convergence. PROVISIONAL (A4: HA-81).

*field, declared at [`include/shulib/localization/complementary_fusion.hpp:85`](../../include/shulib/localization/complementary_fusion.hpp#L85).*

<a id="complementaryfusionconfig-maxheadingnudgerate"></a>

### `ComplementaryFusionConfig::maxHeadingNudgeRate`

```cpp
units::AngularVelocity maxHeadingNudgeRate{10.0 * math::Angle::kPi / 180.0}
```

Per-tick heading budget as a RATE: at most `maxHeadingNudgeRate · dt` of bias change in one tick, loop-rate-independent, exactly as maxNudgeRate bounds position. This is the never-snap bound for yaw — the number that makes "a yaw reset can never happen" a property of the code rather than a promise. 10 deg/s. PROVISIONAL (A4: HA-82).

*field, declared at [`include/shulib/localization/complementary_fusion.hpp:90`](../../include/shulib/localization/complementary_fusion.hpp#L90).*

<a id="class-complementaryfusion"></a>

## `class ComplementaryFusion`

```cpp
class ComplementaryFusion final : public IFusionPolicy
```

The M2 fusion policy: a gated, rate-limited NUDGE toward absolute fixes, never a snap. It is structurally incapable of snapping, and that is the point rather than a tuning achievement — position moves by at most `maxNudgeRate · dt` in a tick, and heading leaves as a bounded INCREMENT instead of an absolute value, so no corrector can reset the estimate however confident it claims to be. Proposals sum and the sum is clamped again, so N correctors cannot out-vote one tick's budget either. Holds no state between calls: the same prediction, proposals and dt always give the same answer. EkfFusion replaces it behind IFusionPolicy at M3 without touching a caller.

*class, declared at [`include/shulib/localization/complementary_fusion.hpp:101`](../../include/shulib/localization/complementary_fusion.hpp#L101).*

<a id="complementaryfusion-complementaryfusion"></a>

### `ComplementaryFusion::ComplementaryFusion`

```cpp
explicit ComplementaryFusion(const ComplementaryFusionConfig& config = {})
```

Copies `config`; nothing is referenced after construction, so the argument may be a temporary. Each field is a LOUD precondition rather than a silent clamp: rates ≥ 0, gates > 0, gains in (0, 1]. A zero gain is excluded on purpose — it is a policy that accepts every fix and then corrects by nothing, which looks like working fusion in every audit flag while the estimate dead-reckons.

*function, declared at [`include/shulib/localization/complementary_fusion.hpp:108`](../../include/shulib/localization/complementary_fusion.hpp#L108).*

<a id="complementaryfusion-fuse"></a>

### `ComplementaryFusion::fuse`

```cpp
[[nodiscard]] FusionResult fuse(const math::Pose2d& predicted, std::span<const CorrectionProposal> valid, units::Time dt) override
```

Fold `valid` into `predicted` and return the corrected absolute POSITION together with a bounded heading INCREMENT — never an absolute heading, which is what makes snapping impossible rather than merely unlikely.  Position and heading are gated INDEPENDENTLY, so a fix may pass one and fail the other. Position: reject |measured − predicted| > innovationGate, else pull maxGain·confidence of it, clamped per proposal and once more on the sum to maxNudgeRate·dt. Heading: the same recipe over `predicted.heading().errorTo(measured)` (shortest signed, so the ±π seam costs nothing), but ONLY for proposals with `providesHeading` — everything else carries a pass-through of the prediction whose innovation is zero by construction. A non-finite innovation or confidence is rejected exactly like an out-of-gate one, and a confidence outside [0, 1] is clamped, so a corrector cannot amplify its own gain.  Empty `valid` returns the predicted position unchanged (dead-reckoning). dt == 0 makes both per-tick budgets zero: the position comes back unchanged and `applied` / `headingApplied` are false, but the audit still reports the GATE's verdict rather than pretending no proposal arrived. `positionStdDev` is not read here — it is carried for the M3 EKF's measurement noise. Holds no state, so this is safe to call out of order.

*function, declared at [`include/shulib/localization/complementary_fusion.hpp:141`](../../include/shulib/localization/complementary_fusion.hpp#L141).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 42 lines</summary>

```text

 ComplementaryFusion — the M2 fusion policy (master plan §8 "complementary → EKF"; decision #4
 "innovation-bounded gated nudge, never snap"). It folds absolute proposals into the predicted
 POSITION as a bounded incremental pull, NEVER an assignment to the measured pose, so it is
 structurally incapable of snapping. An EkfFusion drops in behind the same IFusionPolicy at M3.

 Per valid proposal, given the odom-predicted position:
   innovation = measured.position − predicted.position
   • REJECT it if |innovation| > innovationGate  → a wild GPS/tag fix can never yank the pose.
   • else nudge = (maxGain · confidence) · innovation, then CLAMP |nudge| ≤ maxNudgeRate · dt
     (a RATE-based per-tick budget, so the never-snap bound is invariant to loop rate).
 Proposals sum, and the SUM is clamped once more to the same budget, so N correctors can never
 out-vote the per-tick limit. Empty proposals → the position is returned unchanged (dead-reckon).

 `confidence` (∈[0,1]) is the complementary-tier gain; `positionStdDev` is carried on the proposal
 for the M3 EKF's measurement noise R and is unused here.

 ── HEADING, ADDED AT E3 — AND WHY THIS STILL CANNOT SNAP ──────────────────────────────────
 Until E3 this policy could not touch heading at all, because there was no absolute heading in
 the tree to touch. `AprilTagCorrector` produces one, marks it with
 `CorrectionProposal::providesHeading`, and this policy folds it by exactly the same recipe it
 has always used for position:
   headingInnovation = predicted.heading().errorTo(measured.heading())   (shortest signed)
   • REJECT it if |innovation| > headingGate  → a mirrored tag or a wrong map entry can never
     yank the robot's idea of which way it faces.
   • else nudge = (maxHeadingGain · confidence) · innovation, CLAMPED to maxHeadingNudgeRate·dt.
 Nudges sum and the sum is clamped once more, so N sources cannot out-vote the budget.

 THE STRUCTURAL POINT: what leaves here is an INCREMENT (`FusionResult::headingNudge`), never an
 absolute heading. A policy that could return an absolute heading could snap; a policy that can
 only return a bounded increment cannot, whatever a corrector claims. The Localizer folds the
 increment into a persistent bias and composes the published heading from the IMU as the last
 write of the tick, so the IMU stays the sole source of heading CHANGE (decision #4) and this
 policy can only ever move a slow bias. The two regimes are both live and both tested: for
 innovations under about a degree the GAIN binds (so the bias settles smoothly instead of
 chattering), and above that the RATE CLAMP binds (so a large innovation can never arrive fast).

 Position and heading are gated INDEPENDENTLY. A fix may pass one and fail the other — they are
 different measurements with different failure modes — so `applied`/`gated` continue to describe
 POSITION exactly as they always did, and `headingApplied`/`headingGated` describe heading.
 `GateAudit::reason` likewise stays position-primary; a heading rejection is read off the record
 as a large `gateResidualHeading` with a zero `correctionDTheta`.
```

</details>
