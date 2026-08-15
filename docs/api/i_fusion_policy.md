<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/localization/i_fusion_policy.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `i_fusion_policy.hpp`

IFusionPolicy — the swap point that lets a complementary filter ship NOW and a 5-state SE(2) EKF drop in LATER behind the same seam.

This header declares **1** type (7 members).

Extracted from [`include/shulib/localization/i_fusion_policy.hpp`](../../include/shulib/localization/i_fusion_policy.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class IFusionPolicy`](#class-ifusionpolicy)
  - [`~IFusionPolicy`](#ifusionpolicy-destructor-ifusionpolicy)
  - [`IFusionPolicy`](#ifusionpolicy-ifusionpolicy)
  - [`IFusionPolicy (overload 2)`](#ifusionpolicy-ifusionpolicy-2)
  - [`IFusionPolicy (overload 3)`](#ifusionpolicy-ifusionpolicy-3)
  - [`operator=`](#ifusionpolicy-operator-eq)
  - [`operator= (overload 2)`](#ifusionpolicy-operator-eq-2)
  - [`fuse`](#ifusionpolicy-fuse)

<a id="class-ifusionpolicy"></a>

## `class IFusionPolicy`

```cpp
class IFusionPolicy
```

The seam a fusion MECHANISM plugs into: how hard to move the estimate toward the absolute fixes that arrived this tick. TWO tiers ship behind it: ComplementaryFusion (the gated, rate-limited nudge, still the default) and EkfFusion (a 5-state SE(2) Kalman update with Mahalanobis gating). Picking one is a single constructor argument to the Localizer, with ICorrector, the Localizer API and every caller unchanged across the swap — which is the only reason this is an interface and not a function. An implementation MAY be stateful and may equally be pure, so callers must assume the worse of the two: EkfFusion carries the filter state (x, P) across ticks, while ComplementaryFusion holds nothing but its config and that tier's cross-tick memory lives in the Localizer instead (fusedX_/fusedY_ and the heading bias). So fuse() is not guaranteed to be a function of its arguments alone, and one policy object must not be shared between two Localizers. The Localizer calls fuse() exactly once per update(), UNCONDITIONALLY — during the boot/settle window it is still called, with an empty span of proposals.

*class, declared at [`include/shulib/localization/i_fusion_policy.hpp:31`](../../include/shulib/localization/i_fusion_policy.hpp#L31).*

<a id="ifusionpolicy-destructor-ifusionpolicy"></a>

### `IFusionPolicy::~IFusionPolicy`

```cpp
virtual ~IFusionPolicy() = default
```

Virtual so an owner holding a policy polymorphically can destroy it — the Localizer is not that owner: it takes `IFusionPolicy&` and never deletes it, so the policy must outlive the Localizer that was handed it. The copy/move members are re-defaulted (a user-declared destructor suppresses the implicit MOVEs and deprecates the implicit copies) only so this base imposes no policy of its own. Copying a real one is tier-dependent and rarely what you want: ComplementaryFusion is pure config, but an EkfFusion copy clones a live BELIEF (state and covariance), which then ages independently of the original.

*function, declared at [`include/shulib/localization/i_fusion_policy.hpp:40`](../../include/shulib/localization/i_fusion_policy.hpp#L40).*

<a id="ifusionpolicy-ifusionpolicy"></a>

### `IFusionPolicy::IFusionPolicy`

```cpp
IFusionPolicy() = default
```

*Covered by the comment on [`~IFusionPolicy`](#ifusionpolicy-destructor-ifusionpolicy) — one comment documents this run of special members.*

*function, declared at [`include/shulib/localization/i_fusion_policy.hpp:41`](../../include/shulib/localization/i_fusion_policy.hpp#L41).*

<a id="ifusionpolicy-ifusionpolicy-2"></a>

### `IFusionPolicy::IFusionPolicy (overload 2)`

```cpp
IFusionPolicy(const IFusionPolicy&) = default
```

*Covered by the comment on [`~IFusionPolicy`](#ifusionpolicy-destructor-ifusionpolicy) — one comment documents this run of special members.*

*function, declared at [`include/shulib/localization/i_fusion_policy.hpp:42`](../../include/shulib/localization/i_fusion_policy.hpp#L42).*

<a id="ifusionpolicy-ifusionpolicy-3"></a>

### `IFusionPolicy::IFusionPolicy (overload 3)`

```cpp
IFusionPolicy(IFusionPolicy&&) = default
```

*Covered by the comment on [`~IFusionPolicy`](#ifusionpolicy-destructor-ifusionpolicy) — one comment documents this run of special members.*

*function, declared at [`include/shulib/localization/i_fusion_policy.hpp:43`](../../include/shulib/localization/i_fusion_policy.hpp#L43).*

<a id="ifusionpolicy-operator-eq"></a>

### `IFusionPolicy::operator=`

```cpp
IFusionPolicy& operator=(const IFusionPolicy&) = default
```

*Covered by the comment on [`~IFusionPolicy`](#ifusionpolicy-destructor-ifusionpolicy) — one comment documents this run of special members.*

*function, declared at [`include/shulib/localization/i_fusion_policy.hpp:44`](../../include/shulib/localization/i_fusion_policy.hpp#L44).*

<a id="ifusionpolicy-operator-eq-2"></a>

### `IFusionPolicy::operator= (overload 2)`

```cpp
IFusionPolicy& operator=(IFusionPolicy&&) = default
```

*Covered by the comment on [`~IFusionPolicy`](#ifusionpolicy-destructor-ifusionpolicy) — one comment documents this run of special members.*

*function, declared at [`include/shulib/localization/i_fusion_policy.hpp:45`](../../include/shulib/localization/i_fusion_policy.hpp#L45).*

<a id="ifusionpolicy-fuse"></a>

### `IFusionPolicy::fuse`

```cpp
[[nodiscard]] virtual FusionResult fuse(const math::Pose2d& predicted, std::span<const CorrectionProposal> valid, units::Time dt) = 0
```

Fold the valid proposals into the predicted position and return the corrected (x, y) plus the audit flags. `valid` holds only already-screened proposals (valid && confidence>0 && positionStdDev>0 && finite). `dt` is the tick duration (for a rate-based per-tick clamp).

*function, declared at [`include/shulib/localization/i_fusion_policy.hpp:50`](../../include/shulib/localization/i_fusion_policy.hpp#L50).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 7 lines</summary>

```text

 IFusionPolicy — the swap point that lets a complementary filter ship NOW and a 5-state SE(2) EKF
 drop in LATER behind the same seam (master plan §8: "complementary → EKF"). Given the predicted
 pose and the valid proposals, it returns the corrected POSITION only — heading is re-stamped from
 the IMU by the Localizer afterward, so a policy can never own heading at M2. ComplementaryFusion
 (the gated nudge) implements this now; EkfFusion (Kalman update + Mahalanobis gating) drops in at
 M3+ without changing ICorrector, the Localizer API, or a single caller.
```

</details>
