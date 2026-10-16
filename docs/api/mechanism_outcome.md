<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/manipulation/mechanism_outcome.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `mechanism_outcome.hpp`

MechanismOutcome — the verdict vocabulary of a bounded mechanism operation.

This header declares **1** type (9 members) and **1** free function.

Extracted from [`include/shulib/manipulation/mechanism_outcome.hpp`](../../include/shulib/manipulation/mechanism_outcome.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`enum class MechanismOutcome`](#enum-class-mechanismoutcome)
  - [`Running`](#mechanismoutcome-running)
  - [`Succeeded`](#mechanismoutcome-succeeded)
  - [`Unconfirmed`](#mechanismoutcome-unconfirmed)
  - [`TimedOut`](#mechanismoutcome-timedout)
  - [`Cancelled`](#mechanismoutcome-cancelled)
  - [`Stalled`](#mechanismoutcome-stalled)
  - [`Absent`](#mechanismoutcome-absent)
  - [`NotHomed`](#mechanismoutcome-nothomed)
  - [`Refused`](#mechanismoutcome-refused)
- [`mechanismOutcomeName`](#mechanismoutcomename) — *free function*

<a id="enum-class-mechanismoutcome"></a>

## `enum class MechanismOutcome`

```cpp
enum class MechanismOutcome : std::uint8_t
```

The verdict of a bounded mechanism operation, and deliberately NOT control::ExitReason: a mechanism can finish healthy and still fail its task (`Unconfirmed`), which no motion can do. Where the two vocabularies meet — Routine::then() — only `Succeeded` maps to success. Scoped with no conversion to bool on purpose: `if (outcome)` does not compile, so a failed grab can never read as truthy. Explicit values, append-only; log lines carry the spellings below, so re-meaning one breaks a grep of an old transcript.

*enum class, declared at [`include/shulib/manipulation/mechanism_outcome.hpp:58`](../../include/shulib/manipulation/mechanism_outcome.hpp#L58).*

<a id="mechanismoutcome-running"></a>

### `MechanismOutcome::Running`

```cpp
Running = 0
```

still working; tick again next loop iteration

*enumerator, declared at [`include/shulib/manipulation/mechanism_outcome.hpp:59`](../../include/shulib/manipulation/mechanism_outcome.hpp#L59).*

<a id="mechanismoutcome-succeeded"></a>

### `MechanismOutcome::Succeeded`

```cpp
Succeeded = 1
```

completed AND confirmed (where the operation defines a confirmation; completed, where it does not)

*enumerator, declared at [`include/shulib/manipulation/mechanism_outcome.hpp:60`](../../include/shulib/manipulation/mechanism_outcome.hpp#L60).*

<a id="mechanismoutcome-unconfirmed"></a>

### `MechanismOutcome::Unconfirmed`

```cpp
Unconfirmed = 2
```

the operation ran to completion and the confirmation said the world did not change — healthy mechanism, failed task. Strategy, not pathology: NO fault.

*enumerator, declared at [`include/shulib/manipulation/mechanism_outcome.hpp:62`](../../include/shulib/manipulation/mechanism_outcome.hpp#L62).*

<a id="mechanismoutcome-timedout"></a>

### `MechanismOutcome::TimedOut`

```cpp
TimedOut = 3
```

the watchdog fired before the operation completed

*enumerator, declared at [`include/shulib/manipulation/mechanism_outcome.hpp:65`](../../include/shulib/manipulation/mechanism_outcome.hpp#L65).*

<a id="mechanismoutcome-cancelled"></a>

### `MechanismOutcome::Cancelled`

```cpp
Cancelled = 4
```

stopped from outside via cancel()

*enumerator, declared at [`include/shulib/manipulation/mechanism_outcome.hpp:66`](../../include/shulib/manipulation/mechanism_outcome.hpp#L66).*

<a id="mechanismoutcome-stalled"></a>

### `MechanismOutcome::Stalled`

```cpp
Stalled = 5
```

the stall detector tripped (jam / mechanical bind) — FaultCode::MechanismStalled raised

*enumerator, declared at [`include/shulib/manipulation/mechanism_outcome.hpp:67`](../../include/shulib/manipulation/mechanism_outcome.hpp#L67).*

<a id="mechanismoutcome-absent"></a>

### `MechanismOutcome::Absent`

```cpp
Absent = 6
```

the mechanism was not built (a device missing at boot): nothing moved

*enumerator, declared at [`include/shulib/manipulation/mechanism_outcome.hpp:70`](../../include/shulib/manipulation/mechanism_outcome.hpp#L70).*

<a id="mechanismoutcome-nothomed"></a>

### `MechanismOutcome::NotHomed`

```cpp
NotHomed = 7
```

a position move asked of a mechanism that has not found its zero: nothing moved

*enumerator, declared at [`include/shulib/manipulation/mechanism_outcome.hpp:72`](../../include/shulib/manipulation/mechanism_outcome.hpp#L72).*

<a id="mechanismoutcome-refused"></a>

### `MechanismOutcome::Refused`

```cpp
Refused = 8
```

another owner holds the mechanism's claim: nothing moved

*enumerator, declared at [`include/shulib/manipulation/mechanism_outcome.hpp:74`](../../include/shulib/manipulation/mechanism_outcome.hpp#L74).*

<a id="mechanismoutcomename"></a>

## `mechanismOutcomeName`

```cpp
[[nodiscard]] constexpr const char* mechanismOutcomeName(MechanismOutcome o) noexcept
```

Stable spelling for log lines. Never returns null; an out-of-range cast renders as "UNKNOWN" (never a crash) — faultCodeName's rule.

*free function, declared at [`include/shulib/manipulation/mechanism_outcome.hpp:79`](../../include/shulib/manipulation/mechanism_outcome.hpp#L79).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 44 lines</summary>

```text

 MechanismOutcome — the verdict vocabulary of a bounded mechanism operation
 (chunk F1, WS7/M4). A SEPARATE vocabulary from control::ExitReason, on
 purpose, and the reasoning deserves its length because D1 §2.7 warned that
 every new result vocabulary costs something:

 A mechanism has an outcome no motion can have: THE OPERATION COMPLETED AND
 THE THING DID NOT HAPPEN. The jaws closed on schedule, on a healthy
 mechanism, on nothing — no watchdog fired (so it is not TimedOut) and the
 task certainly did not succeed (so it is emphatically not Settled). That is
 `Unconfirmed`, and ExitReason has no honest place for it:
   * Appending Unconfirmed to ExitReason would let a Chassis::moveTo
     SYNTACTICALLY return it, which it never can — a vocabulary that admits
     impossible values makes every exhaustive switch over it a lie.
   * Reusing ExitReason plus a separate confirmed() flag is the cheapest shape
     and the most dangerous: a caller that reads the verdict and not the flag
     sees "Settled" on a failed grab — precisely the silent success the whole
     error policy exists to prevent.
 So mechanisms get their own verdict, ExitReason stays exactly the four values
 a motion can produce, and the one place the two vocabularies meet —
 Routine::then() — maps them under test, inside the library, with only
 Succeeded mapping to success. This is a scoped enum with NO conversion to
 bool: "if (outcome)" does not compile, so Unconfirmed cannot be truthy by
 accident.

 `Stalled` is a verdict AND a pathology: the operation reports Stalled (so a
 sequencing layer can choose a jam-specific response — back off and retry is
 sensible for a jam and useless for a timeout) and the operation raises
 FaultCode::MechanismStalled (so triage sees the robot is unwell). `TimedOut`
 deliberately raises NO fault — see mechanism_op.hpp for that ruling (T6).

 Explicit values, append-only, like every verdict vocabulary in this project.
 Not currently on the F9 wire (DebugRecord has no mechanism slot — mechanisms
 do not emit per-tick records at F1), but log lines carry the spellings below,
 so the values are treated as stable anyway: re-meaning a value someone
 grepped a transcript for is the same bug class at human scale.

 Three verdicts APPENDED by the mechanism core (MECH-A2, 2026-10-15), each a
 step that moved NOTHING: `Absent` (the mechanism was not built at boot — a
 device missing), `NotHomed` (a position move asked of a mechanism that has not
 found its zero — produced from MECH-A3 on) and `Refused` (another owner holds
 the mechanism's claim). Never "a target the build could not check": a COMPUTED
 target past a travel limit is CLAMPED to it (the soft stop), not refused.
 Like every non-Succeeded verdict, each is a failure where the vocabularies meet.
```

</details>
