<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/localization/i_corrector.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `i_corrector.hpp`

ICorrector — the WRITE seam: one source of ABSOLUTE position fixes (V5 GPS, AprilTag PnP, LIDAR scan-match).

This header declares **2** types (10 members).

Extracted from [`include/shulib/localization/i_corrector.hpp`](../../include/shulib/localization/i_corrector.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class ICorrector`](#class-icorrector)
  - [`~ICorrector`](#icorrector-destructor-icorrector)
  - [`ICorrector`](#icorrector-icorrector)
  - [`ICorrector (overload 2)`](#icorrector-icorrector-2)
  - [`ICorrector (overload 3)`](#icorrector-icorrector-3)
  - [`operator=`](#icorrector-operator-eq)
  - [`operator= (overload 2)`](#icorrector-operator-eq-2)
  - [`propose`](#icorrector-propose)
  - [`name`](#icorrector-name)
- [`class NullCorrector`](#class-nullcorrector)
  - [`propose`](#nullcorrector-propose)
  - [`name`](#nullcorrector-name)

<a id="class-icorrector"></a>

## `class ICorrector`

```cpp
class ICorrector
```

One source of ABSOLUTE field-pose fixes — V5 GPS, AprilTag PnP, LIDAR scan-match. PULL, not push: the Localizer calls propose() once per tick with its odom-predicted pose, and nothing here ever writes into the estimator. An implementation owns ALL of its own mess — HAL access, frame/lever-arm/PnP reduction, latency, staleness, gating — so the Localizer stays geometry-free and the trust math stays in one place. Pure with respect to its injected HAL handle, which is what makes a corrector host-testable against a fake.

*class, declared at [`include/shulib/localization/i_corrector.hpp:25`](../../include/shulib/localization/i_corrector.hpp#L25).*

<a id="icorrector-destructor-icorrector"></a>

### `ICorrector::~ICorrector`

```cpp
virtual ~ICorrector() = default
```

Polymorphic-base boilerplate: the destructor is virtual so a concrete corrector held as `ICorrector&`/`ICorrector*` destroys correctly, and DECLARING it is what suppresses the implicit copy/move, which are re-defaulted below. The base carries no state of its own. Ownership stays with the CALLER either way: the Localizer takes a NON-OWNING `span<ICorrector* const>` (at most kMaxCorrectors, each checked non-null at construction), so every corrector must outlive the Localizer it was handed to.

*function, declared at [`include/shulib/localization/i_corrector.hpp:33`](../../include/shulib/localization/i_corrector.hpp#L33).*

<a id="icorrector-icorrector"></a>

### `ICorrector::ICorrector`

```cpp
ICorrector() = default
```

*Covered by the comment on [`~ICorrector`](#icorrector-destructor-icorrector) — one comment documents this run of special members.*

*function, declared at [`include/shulib/localization/i_corrector.hpp:34`](../../include/shulib/localization/i_corrector.hpp#L34).*

<a id="icorrector-icorrector-2"></a>

### `ICorrector::ICorrector (overload 2)`

```cpp
ICorrector(const ICorrector&) = default
```

*Covered by the comment on [`~ICorrector`](#icorrector-destructor-icorrector) — one comment documents this run of special members.*

*function, declared at [`include/shulib/localization/i_corrector.hpp:35`](../../include/shulib/localization/i_corrector.hpp#L35).*

<a id="icorrector-icorrector-3"></a>

### `ICorrector::ICorrector (overload 3)`

```cpp
ICorrector(ICorrector&&) = default
```

*Covered by the comment on [`~ICorrector`](#icorrector-destructor-icorrector) — one comment documents this run of special members.*

*function, declared at [`include/shulib/localization/i_corrector.hpp:36`](../../include/shulib/localization/i_corrector.hpp#L36).*

<a id="icorrector-operator-eq"></a>

### `ICorrector::operator=`

```cpp
ICorrector& operator=(const ICorrector&) = default
```

*Covered by the comment on [`~ICorrector`](#icorrector-destructor-icorrector) — one comment documents this run of special members.*

*function, declared at [`include/shulib/localization/i_corrector.hpp:37`](../../include/shulib/localization/i_corrector.hpp#L37).*

<a id="icorrector-operator-eq-2"></a>

### `ICorrector::operator= (overload 2)`

```cpp
ICorrector& operator=(ICorrector&&) = default
```

*Covered by the comment on [`~ICorrector`](#icorrector-destructor-icorrector) — one comment documents this run of special members.*

*function, declared at [`include/shulib/localization/i_corrector.hpp:38`](../../include/shulib/localization/i_corrector.hpp#L38).*

<a id="icorrector-propose"></a>

### `ICorrector::propose`

```cpp
[[nodiscard]] virtual CorrectionProposal propose(const math::Pose2d& predicted, units::Time dt) = 0
```

Propose an absolute fix given the odom-predicted pose this tick. MUST be non-throwing and MUST return {valid=false} when it has no usable fix (off-strip GPS, no tag) — never a zero-confidence pull. `dt` is the tick duration (seconds).

*function, declared at [`include/shulib/localization/i_corrector.hpp:43`](../../include/shulib/localization/i_corrector.hpp#L43).*

<a id="icorrector-name"></a>

### `ICorrector::name`

```cpp
[[nodiscard]] virtual const char* name() const noexcept = 0
```

Stable id for telemetry / per-source dead-reckon accounting.

*function, declared at [`include/shulib/localization/i_corrector.hpp:47`](../../include/shulib/localization/i_corrector.hpp#L47).*

<a id="class-nullcorrector"></a>

## `class NullCorrector`

```cpp
class NullCorrector final : public ICorrector
```

The M2 placeholder: a registered source that never has a fix. Lets the fusion pipeline run and be tested end-to-end (it just always dead-reckons) before any real corrector exists, and keeps the seam visibly wired for telemetry. M3 replaces it with GpsCorrector/AprilTagCorrector.

*class, declared at [`include/shulib/localization/i_corrector.hpp:53`](../../include/shulib/localization/i_corrector.hpp#L53).*

<a id="nullcorrector-propose"></a>

### `NullCorrector::propose`

```cpp
[[nodiscard]] CorrectionProposal propose(const math::Pose2d& /*predicted*/, units::Time /*dt*/) override
```

Always declines — a default-constructed proposal, so `valid == false` and `selfAudit.reason == None`. Both arguments are ignored, and the estimator dead-reckons this tick exactly as it would with no corrector registered at all.

*function, declared at [`include/shulib/localization/i_corrector.hpp:58`](../../include/shulib/localization/i_corrector.hpp#L58).*

<a id="nullcorrector-name"></a>

### `NullCorrector::name`

```cpp
[[nodiscard]] const char* name() const noexcept override
```

`"null"`. Because this corrector never proposes and never self-audits, the Localizer never reads it — the id exists so the seam is visibly wired, not to label a record.

*function, declared at [`include/shulib/localization/i_corrector.hpp:64`](../../include/shulib/localization/i_corrector.hpp#L64).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 10 lines</summary>

```text

 ICorrector — the WRITE seam: one source of ABSOLUTE position fixes (V5 GPS, AprilTag PnP, LIDAR
 scan-match) (master plan §6/§8). It is PULL, not push: the Localizer calls propose() each tick
 with the odom-PREDICTED pose and the tick dt, and the corrector returns a CorrectionProposal
 (or {valid=false}). The corrector owns ALL of its mess — HAL access, frame/lever-arm/PnP
 reduction, latency, staleness, gating — so the Localizer stays geometry-free and the trust math
 stays in one place. Pure w.r.t. its injected HAL handle; host-testable with a fake.

 At M2 the registered list is empty or holds only NullCorrector (below); GpsCorrector /
 AprilTagCorrector are concrete at M3 behind THIS exact signature, with no caller change.
```

</details>
