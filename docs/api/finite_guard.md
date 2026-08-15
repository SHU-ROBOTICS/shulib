<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/diag/finite_guard.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `finite_guard.hpp`

Finite-value invariant guards (master plan §18.4) — the LOG-AND-RECOVER counterpart to SHULIB_PRECONDITION's throw.

This header declares **3** free functions.

Extracted from [`include/shulib/diag/finite_guard.hpp`](../../include/shulib/diag/finite_guard.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`isFinitePose`](#isfinitepose) — *free function*
- [`recoverFinite`](#recoverfinite) — *free function*
- [`recoverFinitePose`](#recoverfinitepose) — *free function*

<a id="isfinitepose"></a>

## `isFinitePose`

```cpp
[[nodiscard]] inline bool isFinitePose(const math::Pose2d& p) noexcept
```

True iff the pose is finite. Heading is finite by construction (header note).

*free function, declared at [`include/shulib/diag/finite_guard.hpp:32`](../../include/shulib/diag/finite_guard.hpp#L32).*

<a id="recoverfinite"></a>

## `recoverFinite`

```cpp
[[nodiscard]] inline double recoverFinite(double candidate, double fallback, FaultLatch& faults, FaultCode code, std::string_view subsystem, std::string_view what) noexcept
```

`candidate` if finite; otherwise raise `code` on the latch and return `fallback` (degraded to 0.0 if the fallback is itself non-finite — the guarantee above). `what` names the quantity for the fault line (e.g. "fused vx").

*free function, declared at [`include/shulib/diag/finite_guard.hpp:39`](../../include/shulib/diag/finite_guard.hpp#L39).*

<a id="recoverfinitepose"></a>

## `recoverFinitePose`

```cpp
[[nodiscard]] inline math::Pose2d recoverFinitePose(const math::Pose2d& candidate, const math::Pose2d& fallback, FaultLatch& faults, std::string_view subsystem) noexcept
```

Pose overload: raises NAN_POSE. `fallback` should be the last-known-good pose; a non-finite fallback degrades to the origin pose (finite, unconditionally).

*free function, declared at [`include/shulib/diag/finite_guard.hpp:51`](../../include/shulib/diag/finite_guard.hpp#L51).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 20 lines</summary>

```text

 Finite-value invariant guards (master plan §18.4) — the LOG-AND-RECOVER counterpart to
 SHULIB_PRECONDITION's throw. A precondition guards a CALLER contract (a violation is a
 bug; host tests must go red). These guards protect the RUNTIME DATA PATH: a NaN/Inf
 that appears mid-run (sensor pathology, a division that went bad) must be caught,
 logged as a fault, and replaced with a safe fallback — never propagated into the pose
 estimate, and never allowed to crash or abort the auton.

 The one absolute guarantee, pinned by test/finite_guard_test.cpp:

     recoverFinite* ALWAYS returns a finite value — unconditionally.

 Even a non-finite FALLBACK (a broken caller) degrades to zero rather than letting a
 NaN through: the whole point is that a non-finite value cannot pass this line, so the
 guard cannot have a bypass path, not even a caller-error one. (In correct use the
 fallback is the last-known-good value, which is finite by induction.)

 Heading note: math::Angle cannot HOLD a non-finite value (its factories reject them),
 so a Pose2d's finiteness reduces to its x/y — which is why isFinitePose checks two
 fields, not three. That is a load-bearing property of Angle, not an omission here.
```

</details>
