<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/version.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `version.hpp`

The shulib API version — the mechanism behind the Freeze Register's promise.

This header declares **3** constants.

Extracted from [`include/shulib/version.hpp`](../../include/shulib/version.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`kApiMajor`](#kapimajor) — *constant*
- [`kApiMinor`](#kapiminor) — *constant*
- [`kApiVersionString`](#kapiversionstring) — *constant*

<a id="kapimajor"></a>

## `kApiMajor`

```cpp
inline constexpr int kApiMajor = 2
```

Bumped ONLY for breaking changes to a frozen public surface, always with a migration note (policy above). 2 = the shulib v2 rebuild; the surface frozen at D2 (2026-08-12, register row F6) is API 2.0.

*constant, declared at [`include/shulib/version.hpp:48`](../../include/shulib/version.hpp#L48).*

<a id="kapiminor"></a>

## `kApiMinor`

```cpp
inline constexpr int kApiMinor = 1
```

Bumped for additive extensions of a frozen surface (new verbs, new options fields, appended enumerators). Reset to 0 on a major bump. 1 = chunk F1's additive growth (2026-08-13): RoutineStopCause gains the appended MechanismFailed enumerator, then() accepts a fourth return type (manipulation::MechanismOutcome), and FaultCode appends MechanismStalled — every one the documented additive path, no frozen member changed shape. (F1 is also the change that PROVED this path works: the D2/D3 pin tests had hard-asserted `kApiMinor == 0`, fencing off the growth this header calls "the intended path" — a conflation fixed in those pins at F1.)

*constant, declared at [`include/shulib/version.hpp:59`](../../include/shulib/version.hpp#L59).*

<a id="kapiversionstring"></a>

## `kApiVersionString`

```cpp
inline constexpr const char* kApiVersionString = "2.1"
```

"major.minor", for session headers / logs that want one printable token.

*constant, declared at [`include/shulib/version.hpp:62`](../../include/shulib/version.hpp#L62).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 40 lines</summary>

```text

 The shulib API version — the mechanism behind the Freeze Register's promise.

 The register (docs/roadmap.md, "Freeze Register") says a LOCKED contract
 changes "only with a schemaVersion/API-version bump and a migration path".
 Until D2 (2026-08-12) no such version existed anywhere in the tree; this
 header is that mechanism, made inspectable in code instead of prose.

 ═══ The policy: what a version bump concretely IS ═══════════════════════════════
 shulib is a header-only C++ library, so its API contract is SOURCE
 compatibility — what user programs compile to and mean:

   * BREAKING (bump kApiMajor): any change that can make a previously-
     compiling user program fail to compile or silently change meaning —
     removing or renaming a frozen member or type; changing a frozen
     signature's parameter, return, const, ref or noexcept shape; changing
     documented semantics a frozen surface carries (pre-empt, cancel's safe
     state, wait-for-live, the fault policy...); re-meaning or renumbering
     an existing enumerator. A major bump REQUIRES a migration note next to
     the Freeze Register row: the old spelling, the new spelling, the
     mechanical rewrite, and why the break was worth it.

   * ADDITIVE (bump kApiMinor): every previously-valid program compiles
     unchanged with unchanged behaviour — new members, new overloads, new
     options-struct fields whose default preserves the old behaviour,
     appended enumerators. This is the intended growth path of every frozen
     surface; freezing well means never needing a major bump.

 Frozen C++ surfaces are additionally enforced STRUCTURALLY: F6 carries a
 compile-time signature pin (test/f6_signature_pin_test.cpp) that fails the
 build, naming F6, if a frozen signature changes shape. If that pin sent
 you here and the change is intended: you are making a breaking change —
 update the pin, bump kApiMajor, write the migration note, and update the
 register row. If the change can be re-expressed additively instead, do
 that; it is almost always cheaper for everyone downstream.

 The data-schema freezes (F7/F8: .vexbot sub-schemas; F9: the SHUL/2 wire
 protocol) freeze DATA, not C++ source. They carry their own schemaVersion
 fields inside the artifacts (designed at Phase G/H, not here) and are
 governed by this same breaking-vs-additive policy.
```

</details>
