<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/diag/line_format.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `line_format.hpp`

line_format — the ONE set of §18.3 text-formatting primitives.

This header declares **1** type (7 members), **4** free functions, and **1** constant.

Extracted from [`include/shulib/diag/line_format.hpp`](../../include/shulib/diag/line_format.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`kCompactThresholdBytes`](#kcompactthresholdbytes) — *constant*
- [`struct Line`](#struct-line)
  - [`kCapacity`](#line-kcapacity)
  - [`appendLiteral`](#line-appendliteral)
  - [`appendRaw`](#line-appendraw)
  - [`appendSanitized`](#line-appendsanitized)
  - [`view`](#line-view)
  - [`buf`](#line-buf)
  - [`n`](#line-n)
- [`appendPadded`](#appendpadded) — *free function*
- [`appendNum`](#appendnum) — *free function*
- [`appendUnsigned`](#appendunsigned) — *free function*
- [`appendTimestamp`](#appendtimestamp) — *free function*

<a id="kcompactthresholdbytes"></a>

## `kCompactThresholdBytes`

```cpp
inline constexpr int kCompactThresholdBytes = 10
```

A plain %f rendering longer than this is pathological → compact %.3g re-render. 10 comfortably admits every sane field value (±144.00 coords, ±9999.99 t).

*constant, declared at [`include/shulib/diag/line_format.hpp:40`](../../include/shulib/diag/line_format.hpp#L40).*

<a id="struct-line"></a>

## `struct Line`

```cpp
struct Line
```

One output line: a bounded stack buffer (no heap, hot-path safe). Appends that would overflow truncate silently — unreachable with the fixed widths the §18.3 renderers use, but the bound is enforced, not assumed.

*struct, declared at [`include/shulib/diag/line_format.hpp:45`](../../include/shulib/diag/line_format.hpp#L45).*

<a id="line-kcapacity"></a>

### `Line::kCapacity`

```cpp
static constexpr std::size_t kCapacity = 384
```

Bytes of stack storage per line. Sized far above what the fixed-width renderers can produce, so truncation is a backstop rather than a working mode — and there is no heap anywhere on this path, which is why a line can be built inside the control loop.

*field, declared at [`include/shulib/diag/line_format.hpp:49`](../../include/shulib/diag/line_format.hpp#L49).*

<a id="line-appendliteral"></a>

### `Line::appendLiteral`

```cpp
void appendLiteral(const char* s)
```

Append a NUL-terminated literal verbatim. Renderer-owned text only: nothing here sanitizes, so anything a caller supplied must go through appendSanitized() instead.

*function, declared at [`include/shulib/diag/line_format.hpp:53`](../../include/shulib/diag/line_format.hpp#L53).*

<a id="line-appendraw"></a>

### `Line::appendRaw`

```cpp
void appendRaw(const char* s, std::size_t len)
```

Append exactly `len` bytes verbatim, stopping at kCapacity — an overflowing append is truncated silently rather than reported. Does not sanitize; same rule as appendLiteral.

*function, declared at [`include/shulib/diag/line_format.hpp:57`](../../include/shulib/diag/line_format.hpp#L57).*

<a id="line-appendsanitized"></a>

### `Line::appendSanitized`

```cpp
void appendSanitized(std::string_view text, std::size_t cap)
```

The ONLY entry point for caller-controlled text (header note): sanitizes control bytes to '?', truncates at `cap` with '…' on a UTF-8 boundary.

*function, declared at [`include/shulib/diag/line_format.hpp:66`](../../include/shulib/diag/line_format.hpp#L66).*

<a id="line-view"></a>

### `Line::view`

```cpp
[[nodiscard]] std::string_view view() const noexcept
```

The bytes written so far, as a view INTO this Line's own buffer — a SNAPSHOT of `n` taken at the call. Every append writes at or after the cursor, so the bytes an already-returned view spans are never rewritten: it stays readable for the whole life of the Line and only goes STALE, missing what was appended after it. No flush-before-append discipline and no defensive copy is needed; the one real hazard is LIFETIME, since it dangles the moment the Line leaves scope. Not NUL-terminated — nothing here ever writes a terminator.

*function, declared at [`include/shulib/diag/line_format.hpp:96`](../../include/shulib/diag/line_format.hpp#L96).*

<a id="line-buf"></a>

### `Line::buf`

```cpp
char buf[kCapacity]
```

Raw storage, deliberately left UNINITIALIZED (a Line costs nothing to declare). Only the first `n` bytes have ever been written; read them through view(), never directly.

*field, declared at [`include/shulib/diag/line_format.hpp:100`](../../include/shulib/diag/line_format.hpp#L100).*

<a id="line-n"></a>

### `Line::n`

```cpp
std::size_t n = 0
```

Bytes written so far, and the append cursor. Public because Line is a plain aggregate on the caller's stack, not an encapsulated type; there is no clear(), so reuse means declaring a fresh Line.

*field, declared at [`include/shulib/diag/line_format.hpp:104`](../../include/shulib/diag/line_format.hpp#L104).*

<a id="appendpadded"></a>

## `appendPadded`

```cpp
inline void appendPadded(Line& line, const char* s, int width)
```

Right-pad-to-width helper for the non-finite tokens (and any literal that must occupy a numeric column).

*free function, declared at [`include/shulib/diag/line_format.hpp:109`](../../include/shulib/diag/line_format.hpp#L109).*

<a id="appendnum"></a>

## `appendNum`

```cpp
inline void appendNum(Line& line, double v, int width, int prec)
```

Fixed-width numeric column (header contract): finite values via %*.*f; non-finite as deterministic right-aligned tokens; pathologically wide values compacted to %.3g.

*free function, declared at [`include/shulib/diag/line_format.hpp:119`](../../include/shulib/diag/line_format.hpp#L119).*

<a id="appendunsigned"></a>

## `appendUnsigned`

```cpp
inline void appendUnsigned(Line& line, unsigned long v)
```

Plain decimal, UNPADDED — no column width and no compaction path, unlike appendNum. For the counted quantities in a line (tick numbers, fault counts) whose width is unbounded in principle but never pathological in practice, so no column can be reserved for them anyway.

*free function, declared at [`include/shulib/diag/line_format.hpp:148`](../../include/shulib/diag/line_format.hpp#L148).*

<a id="appendtimestamp"></a>

## `appendTimestamp`

```cpp
inline void appendTimestamp(Line& line, double tSeconds)
```

The §18.3 "[t=%7.2f] " stamp every timestamped line opens with.

*free function, declared at [`include/shulib/diag/line_format.hpp:157`](../../include/shulib/diag/line_format.hpp#L157).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 28 lines</summary>

```text

 line_format — the ONE set of §18.3 text-formatting primitives (WS13, chunk C5).

 Extracted VERBATIM from TermSink (chunk A1) when C5 grew two more §18.3 renderers
 (the per-motion result line and the run-summary block): three renderers each
 hand-rolling NaN/±Inf tokens and pathological-magnitude compaction is exactly how
 one of them drifts — a result line printing libc's "-nan(0x400000)" while the tick
 stream prints "NaN" would break the column discipline §18.3 exists for. One
 definition; every §18.3 line is built from these.

 The formatting CONTRACT (each clause pinned by the A1 golden tests, which survived
 this extraction bit-identically — that is the refactor's proof):
   * appendNum: finite values render fixed-width %*.*f; NaN/±Inf render as
     deterministic right-aligned tokens ("NaN", "+Inf", "-Inf") — never libc's
     locale/sign-varying spellings; a rendering longer than kCompactThresholdBytes
     re-renders compactly as %.3g (the column widens slightly rather than exploding
     to 300+ digits).
   * Line: a bounded stack buffer (no heap, hot-path safe); appends that would
     overflow truncate silently — the bound is enforced, not assumed.
   * appendSanitized: the ONLY entry point for caller-controlled text — control
     bytes (< 0x20, 0x7F) become '?' so a stray '\n'/ESC can never break the
     one-line-per-write framing; truncation backs off UTF-8 continuation bytes and
     marks itself with '…'. (The legacy escapeJSONString lesson: a sanitization
     step callers must remember is a step that gets skipped — so there is no
     unsanitized route into a Line.)

 Concurrency: everything here is stateless free functions plus a caller-owned
 stack value (Line). Nothing allocates; nothing is shared.
```

</details>
