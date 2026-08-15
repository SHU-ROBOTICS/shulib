<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/diag/build_info.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `build_info.hpp`

build_info — the git build hash plumbing for the §18.5 session header.

This header declares **1** free function.

Extracted from [`include/shulib/diag/build_info.hpp`](../../include/shulib/diag/build_info.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`compiledBuildHash`](#compiledbuildhash) — *free function*

<a id="compiledbuildhash"></a>

## `compiledBuildHash`

```cpp
[[nodiscard]] constexpr std::string_view compiledBuildHash() noexcept
```

The build hash the build system injected, or EMPTY if it injected none. Empty means MISSING and must be rendered loudly (header note).

*free function, declared at [`include/shulib/diag/build_info.hpp:36`](../../include/shulib/diag/build_info.hpp#L36).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 27 lines</summary>

```text

 build_info — the git build hash plumbing for the §18.5 session header (WS13, C5).

 THE POINT: six months after a run, the log must say WHICH BINARY produced it.
 The hash therefore comes from the BUILD SYSTEM, never from code — code cannot
 know its own commit. The consuming build defines

     -DSHULIB_BUILD_HASH="\"<git describe --always --dirty>\""

 (test/CMakeLists.txt does exactly this for the host suite; the PROS Makefile
 gains the same line at R1). The --dirty suffix is NOT optional politeness: a
 clean-looking hash from a modified tree IS a wrong hash, and a wrong hash is
 worse than an absent one — it sends the 2am investigation to the wrong commit
 with full confidence.

 THE LOUDNESS CONTRACT (§18.5, the brief's landmine): when the macro is absent,
 compiledBuildHash() returns EMPTY, and every renderer treats empty as MISSING —
 an [ERROR] line in the session header, the literal token "MISSING" in the run
 summary. Nothing anywhere invents a plausible-looking placeholder. There is no
 "unknown"/"0000000" fallback BY DESIGN; grep for one before adding it, then
 don't.

 Header-only nuance, stated honestly: the macro is evaluated PER TRANSLATION
 UNIT. The value that reaches a log is the one seen by the TU that populated the
 SessionInfo (in practice: the one place a program builds its session header).
 A program that never defines the macro gets the loud MISSING path — which is
 correct, not a bug.
```

</details>
