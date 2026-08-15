<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/localization/i_pose_source.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `i_pose_source.hpp`

IPoseSource — the READ seam every pose consumer (motion, alignment, telemetry, skills) depends on.

This header declares **1** type (10 members).

Extracted from [`include/shulib/localization/i_pose_source.hpp`](../../include/shulib/localization/i_pose_source.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class IPoseSource`](#class-iposesource)
  - [`~IPoseSource`](#iposesource-destructor-iposesource)
  - [`IPoseSource`](#iposesource-iposesource)
  - [`IPoseSource (overload 2)`](#iposesource-iposesource-2)
  - [`IPoseSource (overload 3)`](#iposesource-iposesource-3)
  - [`operator=`](#iposesource-operator-eq)
  - [`operator= (overload 2)`](#iposesource-operator-eq-2)
  - [`pose`](#iposesource-pose)
  - [`twist`](#iposesource-twist)
  - [`quality`](#iposesource-quality)
  - [`isDeadReckoning`](#iposesource-isdeadreckoning)

<a id="class-iposesource"></a>

## `class IPoseSource`

```cpp
class IPoseSource
```

The READ seam every pose consumer (motion, alignment, telemetry, skills) depends on. `Localizer` implements it today; a future EKF-backed localizer, a log-replay source or a test fake implement the SAME four accessors, so swapping the fusion tier never touches a caller. All four are `const noexcept` BY CONTRACT, which makes them pure reads of a published snapshot: an implementation must have folded its sensors in its own update step, so calling these repeatedly within one tick costs nothing and cannot change the answer. Heading is IMU-owned; twist() is the matching field-frame derivative. PROS-free (L2).

*class, declared at [`include/shulib/localization/i_pose_source.hpp:21`](../../include/shulib/localization/i_pose_source.hpp#L21).*

<a id="iposesource-destructor-iposesource"></a>

### `IPoseSource::~IPoseSource`

```cpp
virtual ~IPoseSource() = default
```

Abstract base, held and destroyed through IPoseSource*; the implementation must outlive every consumer holding it. Copy/move are defaulted because the interface carries no state of its own, but copying THROUGH this base slices a concrete localizer — and its odometry, correctors and fused belief — away, so consumers take a reference.

*function, declared at [`include/shulib/localization/i_pose_source.hpp:27`](../../include/shulib/localization/i_pose_source.hpp#L27).*

<a id="iposesource-iposesource"></a>

### `IPoseSource::IPoseSource`

```cpp
IPoseSource() = default
```

*Covered by the comment on [`~IPoseSource`](#iposesource-destructor-iposesource) — one comment documents this run of special members.*

*function, declared at [`include/shulib/localization/i_pose_source.hpp:28`](../../include/shulib/localization/i_pose_source.hpp#L28).*

<a id="iposesource-iposesource-2"></a>

### `IPoseSource::IPoseSource (overload 2)`

```cpp
IPoseSource(const IPoseSource&) = default
```

*Covered by the comment on [`~IPoseSource`](#iposesource-destructor-iposesource) — one comment documents this run of special members.*

*function, declared at [`include/shulib/localization/i_pose_source.hpp:29`](../../include/shulib/localization/i_pose_source.hpp#L29).*

<a id="iposesource-iposesource-3"></a>

### `IPoseSource::IPoseSource (overload 3)`

```cpp
IPoseSource(IPoseSource&&) = default
```

*Covered by the comment on [`~IPoseSource`](#iposesource-destructor-iposesource) — one comment documents this run of special members.*

*function, declared at [`include/shulib/localization/i_pose_source.hpp:30`](../../include/shulib/localization/i_pose_source.hpp#L30).*

<a id="iposesource-operator-eq"></a>

### `IPoseSource::operator=`

```cpp
IPoseSource& operator=(const IPoseSource&) = default
```

*Covered by the comment on [`~IPoseSource`](#iposesource-destructor-iposesource) — one comment documents this run of special members.*

*function, declared at [`include/shulib/localization/i_pose_source.hpp:31`](../../include/shulib/localization/i_pose_source.hpp#L31).*

<a id="iposesource-operator-eq-2"></a>

### `IPoseSource::operator= (overload 2)`

```cpp
IPoseSource& operator=(IPoseSource&&) = default
```

*Covered by the comment on [`~IPoseSource`](#iposesource-destructor-iposesource) — one comment documents this run of special members.*

*function, declared at [`include/shulib/localization/i_pose_source.hpp:32`](../../include/shulib/localization/i_pose_source.hpp#L32).*

<a id="iposesource-pose"></a>

### `IPoseSource::pose`

```cpp
[[nodiscard]] virtual math::Pose2d pose() const noexcept = 0
```

Best current field-frame pose (heading == the IMU heading).

*function, declared at [`include/shulib/localization/i_pose_source.hpp:35`](../../include/shulib/localization/i_pose_source.hpp#L35).*

<a id="iposesource-twist"></a>

### `IPoseSource::twist`

```cpp
[[nodiscard]] virtual math::Twist2d twist() const noexcept = 0
```

Field-frame velocity estimate (the derivative of the published pose).

*function, declared at [`include/shulib/localization/i_pose_source.hpp:38`](../../include/shulib/localization/i_pose_source.hpp#L38).*

<a id="iposesource-quality"></a>

### `IPoseSource::quality`

```cpp
[[nodiscard]] virtual double quality() const noexcept = 0
```

Graded trust in the current estimate, in [0,1] — a measurable number, not a vibe.

*function, declared at [`include/shulib/localization/i_pose_source.hpp:41`](../../include/shulib/localization/i_pose_source.hpp#L41).*

<a id="iposesource-isdeadreckoning"></a>

### `IPoseSource::isDeadReckoning`

```cpp
[[nodiscard]] virtual bool isDeadReckoning() const noexcept = 0
```

True when no absolute corrector contributed this tick (running on odom + IMU alone).

*function, declared at [`include/shulib/localization/i_pose_source.hpp:44`](../../include/shulib/localization/i_pose_source.hpp#L44).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 6 lines</summary>

```text

 IPoseSource — the READ seam every pose consumer (motion, alignment, telemetry, skills) depends
 on (master plan §6: "IPoseSource"). The concrete `Localizer` implements it; a future EKF-backed
 localizer, a log-replay source, or a test fake implement the SAME interface, so swapping the
 fusion tier never touches a caller. PROS-free (L2). Heading is IMU-owned; twist() is the matching
 field-frame pose derivative.
```

</details>
