<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/localization/correction.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `correction.hpp`

correction.hpp — the value types the localization fusion seam exchanges.

This header declares **4** types (30 members).

Extracted from [`include/shulib/localization/correction.hpp`](../../include/shulib/localization/correction.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`struct GateAudit`](#struct-gateaudit)
  - [`residualX`](#gateaudit-residualx)
  - [`residualY`](#gateaudit-residualy)
  - [`residualHeading`](#gateaudit-residualheading)
  - [`mahalanobis`](#gateaudit-mahalanobis)
  - [`covarianceTrace`](#gateaudit-covariancetrace)
  - [`reason`](#gateaudit-reason)
- [`struct CorrectionProposal`](#struct-correctionproposal)
  - [`valid`](#correctionproposal-valid)
  - [`fieldPose`](#correctionproposal-fieldpose)
  - [`confidence`](#correctionproposal-confidence)
  - [`positionStdDev`](#correctionproposal-positionstddev)
  - [`providesHeading`](#correctionproposal-providesheading)
  - [`selfAudit`](#correctionproposal-selfaudit)
- [`struct FusionResult`](#struct-fusionresult)
  - [`x`](#fusionresult-x)
  - [`y`](#fusionresult-y)
  - [`applied`](#fusionresult-applied)
  - [`gated`](#fusionresult-gated)
  - [`clamped`](#fusionresult-clamped)
  - [`appliedConfidence`](#fusionresult-appliedconfidence)
  - [`audit`](#fusionresult-audit)
  - [`headingNudge`](#fusionresult-headingnudge)
  - [`headingApplied`](#fusionresult-headingapplied)
  - [`headingGated`](#fusionresult-headinggated)
  - [`headingClamped`](#fusionresult-headingclamped)
- [`struct AppliedCorrection`](#struct-appliedcorrection)
  - [`dx`](#appliedcorrection-dx)
  - [`dy`](#appliedcorrection-dy)
  - [`gated`](#appliedcorrection-gated)
  - [`clamped`](#appliedcorrection-clamped)
  - [`source`](#appliedcorrection-source)
  - [`audit`](#appliedcorrection-audit)
  - [`dtheta`](#appliedcorrection-dtheta)

<a id="struct-gateaudit"></a>

## `struct GateAudit`

```cpp
struct GateAudit
```

WHY the gate decided what it decided, as data (WS13/E1's estimator introspection).  The §18.2 record already has slots for these quantities — `gateResidualX/Y/Heading`, `gateMahalanobis`, `gateReason`, `covarianceTrace` — declared at A1 and unpopulated by design until a real gate exists. This struct is the CARRIER that lets a fusion policy fill them: it rides out on FusionResult, the Localizer keeps it on AppliedCorrection, and the record producer stamps it. Nothing about the frozen IPoseSource / ICorrector / IFusionPolicy signatures changes — the seam was built EKF-ready at M2 and stays exactly as shaped.  Why it matters: every tick after E2 makes a DECISION about whether to trust a sensor fix, and those decisions are where fusion goes wrong. They are invisible unless something writes them down, and the < 1° accuracy claim is certified by exactly these numbers — residual, Mahalanobis distance, accept/reject reason — rather than asserted.  HONEST SCOPE AT E1: the complementary tier fills `reason` (None / Accepted / RejectedInnovation), the residual of the fix it acted on, and `covarianceTrace` as the tier's scalar TRUST WEIGHT (which is what debug_record.hpp's own note reserves that slot for until an EKF exists). `mahalanobis` stays 0 until E4 — a complementary filter has no covariance to normalise by, and a fabricated distance would be worse than an absent one. RejectedNoFix / RejectedHighYawRate are CORRECTOR-side verdicts that E2 fills in.

*struct, declared at [`include/shulib/localization/correction.hpp:37`](../../include/shulib/localization/correction.hpp#L37).*

<a id="gateaudit-residualx"></a>

### `GateAudit::residualX`

```cpp
units::Length residualX{}
```

innovation (measured − predicted), field x

*field, declared at [`include/shulib/localization/correction.hpp:38`](../../include/shulib/localization/correction.hpp#L38).*

<a id="gateaudit-residualy"></a>

### `GateAudit::residualY`

```cpp
units::Length residualY{}
```

innovation, field y

*field, declared at [`include/shulib/localization/correction.hpp:39`](../../include/shulib/localization/correction.hpp#L39).*

<a id="gateaudit-residualheading"></a>

### `GateAudit::residualHeading`

```cpp
units::AngleDim residualHeading{}
```

innovation, heading (radians) — E3 fills it

*field, declared at [`include/shulib/localization/correction.hpp:40`](../../include/shulib/localization/correction.hpp#L40).*

<a id="gateaudit-mahalanobis"></a>

### `GateAudit::mahalanobis`

```cpp
double mahalanobis = 0.0
```

Mahalanobis distance of the fix — E4 fills it

*field, declared at [`include/shulib/localization/correction.hpp:41`](../../include/shulib/localization/correction.hpp#L41).*

<a id="gateaudit-covariancetrace"></a>

### `GateAudit::covarianceTrace`

```cpp
double covarianceTrace = 0.0
```

EKF trace (E4), or the tier's trust weight today

*field, declared at [`include/shulib/localization/correction.hpp:42`](../../include/shulib/localization/correction.hpp#L42).*

<a id="gateaudit-reason"></a>

### `GateAudit::reason`

```cpp
diag::GateReason reason = diag::GateReason::None
```

why accepted/rejected

*field, declared at [`include/shulib/localization/correction.hpp:43`](../../include/shulib/localization/correction.hpp#L43).*

<a id="struct-correctionproposal"></a>

## `struct CorrectionProposal`

```cpp
struct CorrectionProposal
```

What ONE corrector offers this tick. An ABSOLUTE field pose + how much to trust it — never a delta and never a "set". `valid == false` means "I have nothing usable this tick" (off-strip GPS, no tag) and the proposal is ignored entirely (NOT a zero-confidence pull toward (0,0)).

*struct, declared at [`include/shulib/localization/correction.hpp:49`](../../include/shulib/localization/correction.hpp#L49).*

<a id="correctionproposal-valid"></a>

### `CorrectionProposal::valid`

```cpp
bool valid = false
```

false ⇒ skip entirely (dead-reckon w.r.t. this source)

*field, declared at [`include/shulib/localization/correction.hpp:50`](../../include/shulib/localization/correction.hpp#L50).*

<a id="correctionproposal-fieldpose"></a>

### `CorrectionProposal::fieldPose`

```cpp
math::Pose2d fieldPose{}
```

absolute field pose the source believes the robot is at

*field, declared at [`include/shulib/localization/correction.hpp:51`](../../include/shulib/localization/correction.hpp#L51).*

<a id="correctionproposal-confidence"></a>

### `CorrectionProposal::confidence`

```cpp
double confidence = 0.0
```

[0,1] peak trust; 0 ⇒ no pull even if valid

*field, declared at [`include/shulib/localization/correction.hpp:52`](../../include/shulib/localization/correction.hpp#L52).*

<a id="correctionproposal-positionstddev"></a>

### `CorrectionProposal::positionStdDev`

```cpp
units::Length positionStdDev{}
```

1σ position noise (R for an EKF / nudge weight); > 0 when valid

*field, declared at [`include/shulib/localization/correction.hpp:53`](../../include/shulib/localization/correction.hpp#L53).*

<a id="correctionproposal-providesheading"></a>

### `CorrectionProposal::providesHeading`

```cpp
bool providesHeading = false
```

LIVE SINCE E3 (was RESERVED at M2). `true` means `fieldPose.heading()` is an ABSOLUTE measured heading and the fusion policy may nudge toward it; `false` means the heading field is a pass-through of the prediction and carries no information (E2's GpsCorrector sets it false and passes the PREDICTED heading, deliberately, so that even a policy that read it would read the estimator's own answer).  This is the additive path M2 reserved, taken exactly as written: a `headingNudge` on FusionResult which the Localizer folds into a persistent heading BIAS before composing the final heading from the IMU. The frozen IPoseSource / ICorrector / IFusionPolicy signatures did not move, and no existing construction of this struct changed meaning.

*field, declared at [`include/shulib/localization/correction.hpp:64`](../../include/shulib/localization/correction.hpp#L64).*

<a id="correctionproposal-selfaudit"></a>

### `CorrectionProposal::selfAudit`

```cpp
GateAudit selfAudit{}
```

The corrector's OWN account of this tick — APPENDED at E2, trailing and defaulted, so every existing construction of this struct still compiles and means the same thing (the same discipline E1 used to add `GateAudit` to `FusionResult`).  WHY IT EXISTS. A corrector that returns `valid == false` is dropped by the Localizer and never reaches a fusion policy, so before E2 a corrector-side verdict had NO channel to the record: an off-strip GPS and an empty corrector list produced the same `GateReason::None`, and "the estimator is dead-reckoning because the strip is missing" was indistinguishable from "nobody asked". Driving Skills has no GPS strip, which makes that the difference between a diagnosable run and a mystery. `RejectedNoFix` and `RejectedHighYawRate` were reserved at A1 as corrector-side verdicts; this is the wire that carries them.  CONTRACT. Set `selfAudit.reason` on every tick the corrector declines to propose, and leave it `None` when it does propose — the fusion policy owns the audit for proposals that reach it, and a corrector claiming `Accepted` here could otherwise be substituted into the record on a tick where the Localizer screened the proposal out and nothing was applied. The Localizer substitutes this audit ONLY when the policy returned no verdict of its own (see localizer.hpp, STEP 4).

*field, declared at [`include/shulib/localization/correction.hpp:84`](../../include/shulib/localization/correction.hpp#L84).*

<a id="struct-fusionresult"></a>

## `struct FusionResult`

```cpp
struct FusionResult
```

What a fusion policy did this tick.  x/y are an ABSOLUTE fused position: predicted + a bounded nudge. `headingNudge` is NOT — it is a bounded INCREMENT, and the difference is the whole safety argument. A policy that returned an absolute heading could snap; a policy that can only return an increment cannot, no matter what a corrector proposes or how confident it claims to be. The Localizer accumulates the increment into a persistent heading bias and composes the published heading from the IMU as the final write of the tick, so the IMU remains the sole source of heading CHANGE and the corrector can only ever learn a slowly-moving BIAS (localizer.hpp, STEP 5).

*struct, declared at [`include/shulib/localization/correction.hpp:96`](../../include/shulib/localization/correction.hpp#L96).*

<a id="fusionresult-x"></a>

### `FusionResult::x`

```cpp
units::Length x{}
```

fused field x (predicted + bounded nudge)

*field, declared at [`include/shulib/localization/correction.hpp:97`](../../include/shulib/localization/correction.hpp#L97).*

<a id="fusionresult-y"></a>

### `FusionResult::y`

```cpp
units::Length y{}
```

fused field y

*field, declared at [`include/shulib/localization/correction.hpp:98`](../../include/shulib/localization/correction.hpp#L98).*

<a id="fusionresult-applied"></a>

### `FusionResult::applied`

```cpp
bool applied = false
```

≥1 proposal passed the gate and was incorporated

*field, declared at [`include/shulib/localization/correction.hpp:99`](../../include/shulib/localization/correction.hpp#L99).*

<a id="fusionresult-gated"></a>

### `FusionResult::gated`

```cpp
bool gated = false
```

a proposal was rejected by the innovation bound

*field, declared at [`include/shulib/localization/correction.hpp:100`](../../include/shulib/localization/correction.hpp#L100).*

<a id="fusionresult-clamped"></a>

### `FusionResult::clamped`

```cpp
bool clamped = false
```

the per-tick budget bound the applied nudge

*field, declared at [`include/shulib/localization/correction.hpp:101`](../../include/shulib/localization/correction.hpp#L101).*

<a id="fusionresult-appliedconfidence"></a>

### `FusionResult::appliedConfidence`

```cpp
double appliedConfidence = 0.0
```

[0,1] confidence of the strongest applied fix (0 if none); drives how much the drift accumulator is cleared.

*field, declared at [`include/shulib/localization/correction.hpp:102`](../../include/shulib/localization/correction.hpp#L102).*

<a id="fusionresult-audit"></a>

### `FusionResult::audit`

```cpp
GateAudit audit{}
```

WHY this tick decided as it did (E1) — APPENDED, so every existing positional construction of this struct still compiles and means the same thing.

*field, declared at [`include/shulib/localization/correction.hpp:104`](../../include/shulib/localization/correction.hpp#L104).*

<a id="fusionresult-headingnudge"></a>

### `FusionResult::headingNudge`

```cpp
units::AngleDim headingNudge{}
```

The bounded heading INCREMENT to fold into the estimator's heading bias this tick, in radians. APPENDED at E3, trailing and defaulted, on the same discipline E1 and E2 used: every existing construction of this struct still compiles and still means exactly what it meant, because a policy that does not set these leaves heading untouched.

*field, declared at [`include/shulib/localization/correction.hpp:111`](../../include/shulib/localization/correction.hpp#L111).*

<a id="fusionresult-headingapplied"></a>

### `FusionResult::headingApplied`

```cpp
bool headingApplied = false
```

a proposal supplying an absolute heading was folded

*field, declared at [`include/shulib/localization/correction.hpp:112`](../../include/shulib/localization/correction.hpp#L112).*

<a id="fusionresult-headinggated"></a>

### `FusionResult::headingGated`

```cpp
bool headingGated = false
```

a heading proposal was rejected by the heading bound

*field, declared at [`include/shulib/localization/correction.hpp:113`](../../include/shulib/localization/correction.hpp#L113).*

<a id="fusionresult-headingclamped"></a>

### `FusionResult::headingClamped`

```cpp
bool headingClamped = false
```

the per-tick heading budget bound the nudge

*field, declared at [`include/shulib/localization/correction.hpp:114`](../../include/shulib/localization/correction.hpp#L114).*

<a id="struct-appliedcorrection"></a>

## `struct AppliedCorrection`

```cpp
struct AppliedCorrection
```

The per-tick audit record the Localizer exposes via lastCorrection() — maps onto the §18.2 DebugRecord "applied-correction (dx,dy) + clamped + gating reason" so the never-snap guarantee is observable in telemetry. dx/dy are the NET position change applied this tick.

*struct, declared at [`include/shulib/localization/correction.hpp:120`](../../include/shulib/localization/correction.hpp#L120).*

<a id="appliedcorrection-dx"></a>

### `AppliedCorrection::dx`

```cpp
units::Length dx{}
```

inches the estimate moved in field +X (fused − predicted)

*field, declared at [`include/shulib/localization/correction.hpp:121`](../../include/shulib/localization/correction.hpp#L121).*

<a id="appliedcorrection-dy"></a>

### `AppliedCorrection::dy`

```cpp
units::Length dy{}
```

inches in field +Y; both zero when nothing was applied

*field, declared at [`include/shulib/localization/correction.hpp:122`](../../include/shulib/localization/correction.hpp#L122).*

<a id="appliedcorrection-gated"></a>

### `AppliedCorrection::gated`

```cpp
bool gated = false
```

any proposal rejected as too far (innovation gate)

*field, declared at [`include/shulib/localization/correction.hpp:123`](../../include/shulib/localization/correction.hpp#L123).*

<a id="appliedcorrection-clamped"></a>

### `AppliedCorrection::clamped`

```cpp
bool clamped = false
```

the per-tick nudge budget was hit

*field, declared at [`include/shulib/localization/correction.hpp:124`](../../include/shulib/localization/correction.hpp#L124).*

<a id="appliedcorrection-source"></a>

### `AppliedCorrection::source`

```cpp
const char* source = "none"
```

name() of the corrector applied, or "none"

*field, declared at [`include/shulib/localization/correction.hpp:125`](../../include/shulib/localization/correction.hpp#L125).*

<a id="appliedcorrection-audit"></a>

### `AppliedCorrection::audit`

```cpp
GateAudit audit{}
```

the gate's own account of this tick (E1) — this is the value the record producer stamps into the §18.2 slots

*field, declared at [`include/shulib/localization/correction.hpp:126`](../../include/shulib/localization/correction.hpp#L126).*

<a id="appliedcorrection-dtheta"></a>

### `AppliedCorrection::dtheta`

```cpp
units::AngleDim dtheta{}
```

The NET heading change applied this tick, in radians — the §18.2 `correctionDTheta` slot, declared at A1 as "heading nudge (0 at M2: heading is IMU-owned) — E3" and filled here. APPENDED, trailing and defaulted, so every existing construction still compiles. This is what audits never-snap for HEADING the way dx/dy audit it for position.

*field, declared at [`include/shulib/localization/correction.hpp:132`](../../include/shulib/localization/correction.hpp#L132).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 5 lines</summary>

```text

 correction.hpp — the value types the localization fusion seam exchanges (master plan §8; WS5).
 These are the mechanism-agnostic currency that lets a complementary filter today and a 5-state
 SE(2) EKF later share ONE seam: a corrector PROPOSES an absolute fix, a fusion policy decides
 how hard to move toward it, and the Localizer records what was applied for telemetry.
```

</details>
