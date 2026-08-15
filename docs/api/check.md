<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/core/check.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `check.hpp`

Precondition checking for shulib core.

This header declares **1** type (1 member), **4** free functions, and **1** type alias.

Extracted from [`include/shulib/core/check.hpp`](../../include/shulib/core/check.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`struct PreconditionError`](#struct-preconditionerror)
  - [`logic_error`](#preconditionerror-logic_error)
- [`PreconditionHandler`](#preconditionhandler) — *type alias*
- [`throwingPreconditionHandler`](#throwingpreconditionhandler) — *free function*
- [`setPreconditionHandler`](#setpreconditionhandler) — *free function*
- [`preconditionHandler (overload 2)`](#preconditionhandler-2) — *free function*
- [`precondition_failed`](#precondition_failed) — *free function*

<a id="struct-preconditionerror"></a>

## `struct PreconditionError`

```cpp
struct PreconditionError : std::logic_error
```

What a violated precondition throws under BOTH shipped policies. A logic_error because a breach is a CALLER bug, not a runtime condition to retry: nothing catches it at the call site, because the call sites guard invariants (bounds, non-null, finite) past which continuing is undefined behavior. The motion scheduler catches it at the task boundary and turns it into a FAULT_ABORT exit plus a safe drivetrain, so one bad reading costs one motion — never the auton.

*struct, declared at [`include/shulib/core/check.hpp:49`](../../include/shulib/core/check.hpp#L49).*

<a id="preconditionerror-logic_error"></a>

### `PreconditionError::logic_error`

```cpp
using std::logic_error::logic_error
```

Inherits logic_error's constructors: the message is the literal handed to SHULIB_PRECONDITION, readable through what().

*alias, declared at [`include/shulib/core/check.hpp:52`](../../include/shulib/core/check.hpp#L52).*

<a id="preconditionhandler"></a>

## `PreconditionHandler`

```cpp
using PreconditionHandler = void (*)(const char* message)
```

The policy hook type. MUST NOT RETURN (see header contract).

*type alias, declared at [`include/shulib/core/check.hpp:56`](../../include/shulib/core/check.hpp#L56).*

<a id="throwingpreconditionhandler"></a>

## `throwingPreconditionHandler`

```cpp
[[noreturn]] inline void throwingPreconditionHandler(const char* message)
```

The host/test default policy: throw, so breaches turn tests red.

*free function, declared at [`include/shulib/core/check.hpp:59`](../../include/shulib/core/check.hpp#L59).*

<a id="setpreconditionhandler"></a>

## `setPreconditionHandler`

```cpp
inline PreconditionHandler setPreconditionHandler(PreconditionHandler handler) noexcept
```

Install a policy handler; returns the PREVIOUS handler so a caller (e.g. a test) can restore it. Passing nullptr restores the default throwing policy — the seam is never left empty.

*free function, declared at [`include/shulib/core/check.hpp:73`](../../include/shulib/core/check.hpp#L73).*

<a id="preconditionhandler-2"></a>

## `preconditionHandler (overload 2)`

```cpp
[[nodiscard]] inline PreconditionHandler preconditionHandler() noexcept
```

The currently installed policy (introspection; used by tests to restore).

*free function, declared at [`include/shulib/core/check.hpp:81`](../../include/shulib/core/check.hpp#L81).*

<a id="precondition_failed"></a>

## `precondition_failed`

```cpp
[[noreturn]] inline void precondition_failed(const char* message)
```

Report a violated precondition through whichever policy is installed. Reach it through SHULIB_PRECONDITION rather than calling it directly — the macro is what keeps the call sites identical on every target. [[noreturn]] is load-bearing: a handler must not return, and if a broken one does, this calls std::terminate() rather than let execution continue past a violated invariant. That terminate is unreachable through either shipped policy; only a handler that breaks the seam's contract can get there.

*free function, declared at [`include/shulib/core/check.hpp:91`](../../include/shulib/core/check.hpp#L91).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 35 lines</summary>

```text

 Precondition checking for shulib core.

 SHULIB_PRECONDITION(cond, msg) guards a CALLER contract. On a violation it calls
 precondition_failed(), which routes through an installable POLICY HANDLER — the
 §18.4 policy seam (resolved at chunk A1; this replaced the former TODO here).

 The two policies, and why the seam exists:
   * HOST/TEST (the DEFAULT handler, installed at startup automatically): THROW a
     catchable PreconditionError, so a contract breach turns a test RED instead of
     silently corrupting state (e.g. a NaN flowing into the pose estimate).
   * ON-ROBOT (installed once at init by R1's hal/pros bootstrap): raise
     diag::FaultCode::Precondition on the fault latch, then THROW the same
     PreconditionError — which the motion scheduler catches at the task boundary and
     converts to a FAULT_ABORT exit + a safe drivetrain state. Recovery happens at
     the MOTION boundary, not the call site: the call sites guard invariants
     (bounds, non-null, finite) past which continuing would be undefined behavior,
     so "log and continue right here" is not a safe fallback — unwinding to the
     nearest boundary that CAN recover is. One bad reading degrades one motion to a
     fault code; it never aborts the auton.

 The call sites never change — SHULIB_PRECONDITION(cond, msg) everywhere, on every
 target — only the installed policy differs. That is the whole design.

 HANDLER CONTRACT (load-bearing): a handler must NOT return — it throws (both shipped
 policies do) or otherwise diverts control. precondition_failed() is [[noreturn]];
 if a broken handler does return, std::terminate() fires rather than letting
 execution continue past a violated invariant into undefined behavior. (This
 terminate is reachable ONLY via a handler that violates the seam's contract — the
 shipped policies can never hit it.)

 Concurrency contract: the handler is installed ONCE, at startup, before any other
 task exists (host: never re-installed outside tests; robot: in R1's init). The slot
 is a plain pointer read on the hot path — no atomics, because installation is not
 concurrent with use by contract.
```

</details>
