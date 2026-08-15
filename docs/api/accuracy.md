<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/spec/accuracy.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `accuracy.hpp`

accuracy.hpp — the accuracy targets of Freeze Register ROW F2, the LOCKED spec the autonomous is measured against.

This header declares **5** constants.

Extracted from [`include/shulib/spec/accuracy.hpp`](../../include/shulib/spec/accuracy.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`kHeadingErrorMaxDeg`](#kheadingerrormaxdeg) — *constant*
- [`kDockedHeadingTypicalDeg`](#kdockedheadingtypicaldeg) — *constant*
- [`kPositionErrorEndOfRun`](#kpositionerrorendofrun) — *constant*
- [`kRepeatability`](#krepeatability) — *constant*
- [`kDockedPositionError`](#kdockedpositionerror) — *constant*

<a id="kheadingerrormaxdeg"></a>

## `kHeadingErrorMaxDeg`

```cpp
inline constexpr double kHeadingErrorMaxDeg = 1.0
```

The one HARD target: |heading error| must ALWAYS be strictly below this. A bare magnitude in DEGREES — not radians, not a units:: quantity — so comparing against it means converting the estimate's radians first, as the acceptance test does. The position targets flex around this one; it never flexes around them.

*constant, declared at [`include/shulib/spec/accuracy.hpp:24`](../../include/shulib/spec/accuracy.hpp#L24).*

<a id="kdockedheadingtypicaldeg"></a>

## `kDockedHeadingTypicalDeg`

```cpp
inline constexpr double kDockedHeadingTypicalDeg = 0.5
```

What closed-loop docking on a tag is expected to reach (degrees), stated beside the cap so the two are read together. An ASPIRATION, not a gate: nothing asserts a run against it; the only test that mentions it checks that it is tighter than kHeadingErrorMaxDeg.

*constant, declared at [`include/shulib/spec/accuracy.hpp:29`](../../include/shulib/spec/accuracy.hpp#L29).*

<a id="kpositionerrorendofrun"></a>

## `kPositionErrorEndOfRun`

```cpp
inline constexpr units::Length kPositionErrorEndOfRun{1.0}
```

Absolute position error, in canonical inches, at the END of a 60 s autonomous on the v1 estimator (GPS + tracking wheels + IMU) — the loosest of the three, because it is the one that accumulates for a whole run.

*constant, declared at [`include/shulib/spec/accuracy.hpp:35`](../../include/shulib/spec/accuracy.hpp#L35).*

<a id="krepeatability"></a>

## `kRepeatability`

```cpp
inline constexpr units::Length kRepeatability{0.75}
```

Run-to-run SPREAD (canonical inches), not error against truth: how far apart the same routine's end poses may land across repeats. Deliberately tighter than the absolute target — a repeatable robot can be trimmed, a scattered one cannot.

*constant, declared at [`include/shulib/spec/accuracy.hpp:40`](../../include/shulib/spec/accuracy.hpp#L40).*

<a id="kdockedpositionerror"></a>

## `kDockedPositionError`

```cpp
inline constexpr units::Length kDockedPositionError{0.25}
```

Final alignment error (canonical inches) for closed-loop vision docking on a tag (M3). The tightest number here because the loop closes on the thing being docked to, so it never pays the dead-reckoning accumulation the other two do.

*constant, declared at [`include/shulib/spec/accuracy.hpp:44`](../../include/shulib/spec/accuracy.hpp#L44).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 12 lines</summary>

```text

 accuracy.hpp — the accuracy targets of Freeze Register ROW F2, the LOCKED
 spec the autonomous is measured against (master plan §7, ratified
 2026-06-08). "F2" here is the REGISTER ROW, not chunk F2 (the sequence
 engine, 2026-08-13) — the name collision is real and this line is the
 disambiguation.

 This is the SINGLE SOURCE OF TRUTH for the numbers. The system-level
 acceptance tests at M2/M3 assert against these constants, and a guard test
 pins them so they cannot drift without a deliberate Freeze change.

 HEADING < 1.0 deg is a HARD requirement; position targets flex around it.
```

</details>
