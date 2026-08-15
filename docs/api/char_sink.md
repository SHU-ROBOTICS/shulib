<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/hal/char_sink.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `char_sink.hpp`

ICharSink — where formatted diagnostic BYTES physically go (a terminal, a captured string in a test, later a serial port).

This header declares **1** type (7 members).

Extracted from [`include/shulib/hal/char_sink.hpp`](../../include/shulib/hal/char_sink.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`class ICharSink`](#class-icharsink)
  - [`~ICharSink`](#icharsink-destructor-icharsink)
  - [`ICharSink`](#icharsink-icharsink)
  - [`ICharSink (overload 2)`](#icharsink-icharsink-2)
  - [`ICharSink (overload 3)`](#icharsink-icharsink-3)
  - [`operator=`](#icharsink-operator-eq)
  - [`operator= (overload 2)`](#icharsink-operator-eq-2)
  - [`write`](#icharsink-write)

<a id="class-icharsink"></a>

## `class ICharSink`

```cpp
class ICharSink
```

Where formatted diagnostic BYTES physically go — a terminal, a captured string in a test, later a serial port. Injecting the character device is what makes TermSink's output a testable claim: a host test asserts its exact bytes against a golden string, which hard-coding std::cout would have made impossible. Sanitization is the FORMATTER's job, not a sink's; a sink takes the bytes verbatim. NOT one of the frozen ten runtime robot-HAL interfaces — this is an additive diagnostics-output seam.

*class, declared at [`include/shulib/hal/char_sink.hpp:29`](../../include/shulib/hal/char_sink.hpp#L29).*

<a id="icharsink-destructor-icharsink"></a>

### `ICharSink::~ICharSink`

```cpp
virtual ~ICharSink() = default
```

Polymorphic-base plumbing: the destructor is virtual so a sink may be owned and destroyed through an `ICharSink*`, and declaring it suppresses the implicit copy/move, which are re-defaulted here. Defaulted rather than deleted because this seam holds no state — but the concrete sinks DO (a FILE* on the robot, a captured std::string in tests), so copying THROUGH this base slices one down to the empty interface. Nothing in the library owns a sink: TermSink keeps an `ICharSink&`, so the sink must outlive every TermSink pointed at it, and opening/closing the device stays the caller's job.

*function, declared at [`include/shulib/hal/char_sink.hpp:39`](../../include/shulib/hal/char_sink.hpp#L39).*

<a id="icharsink-icharsink"></a>

### `ICharSink::ICharSink`

```cpp
ICharSink() = default
```

*Covered by the comment on [`~ICharSink`](#icharsink-destructor-icharsink) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/char_sink.hpp:40`](../../include/shulib/hal/char_sink.hpp#L40).*

<a id="icharsink-icharsink-2"></a>

### `ICharSink::ICharSink (overload 2)`

```cpp
ICharSink(const ICharSink&) = default
```

*Covered by the comment on [`~ICharSink`](#icharsink-destructor-icharsink) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/char_sink.hpp:41`](../../include/shulib/hal/char_sink.hpp#L41).*

<a id="icharsink-icharsink-3"></a>

### `ICharSink::ICharSink (overload 3)`

```cpp
ICharSink(ICharSink&&) = default
```

*Covered by the comment on [`~ICharSink`](#icharsink-destructor-icharsink) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/char_sink.hpp:42`](../../include/shulib/hal/char_sink.hpp#L42).*

<a id="icharsink-operator-eq"></a>

### `ICharSink::operator=`

```cpp
ICharSink& operator=(const ICharSink&) = default
```

*Covered by the comment on [`~ICharSink`](#icharsink-destructor-icharsink) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/char_sink.hpp:43`](../../include/shulib/hal/char_sink.hpp#L43).*

<a id="icharsink-operator-eq-2"></a>

### `ICharSink::operator= (overload 2)`

```cpp
ICharSink& operator=(ICharSink&&) = default
```

*Covered by the comment on [`~ICharSink`](#icharsink-destructor-icharsink) — one comment documents this run of special members.*

*function, declared at [`include/shulib/hal/char_sink.hpp:44`](../../include/shulib/hal/char_sink.hpp#L44).*

<a id="icharsink-write"></a>

### `ICharSink::write`

```cpp
virtual void write(std::string_view text) = 0
```

Write `text` verbatim to the device. MUST NOT throw.

*function, declared at [`include/shulib/hal/char_sink.hpp:47`](../../include/shulib/hal/char_sink.hpp#L47).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 15 lines</summary>

```text

 ICharSink — where formatted diagnostic BYTES physically go (a terminal, a captured
 string in a test, later a serial port). It exists so TermSink's output is a TESTABLE
 claim: with the character device injected (mirroring the injected-clock pattern in
 control/), a host test asserts TermSink's exact bytes against a golden string —
 hard-coding std::cout would have made "readable, column-aligned" unfalsifiable.

 NOT part of the frozen F4 ten (that freeze covers the 10 runtime robot-HAL
 interfaces); this is an ADDITIVE diagnostics-output seam introduced at chunk A1.
 R1 adds the on-robot stdout adapter; test/ uses hal::fake::FakeCharSink.

 Contract: write() takes the bytes verbatim (the FORMATTER owns sanitization — see
 TermSink), is called synchronously on the caller's task, and MUST NOT throw. One
 write() call carries one complete line, so an implementation that is atomic per call
 never interleaves lines.
```

</details>
