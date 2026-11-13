<!-- GENERATED FILE — DO NOT EDIT BY HAND.
     Source: include/shulib/runner/manifest.hpp
     Regenerate: python3 tools/api_doc_tool.py generate
     The host test build fails if this file is out of date, so an edit here
     is reverted by the next build rather than reviewed. Edit the header. -->

# `manifest.hpp`

writeManifest — the command-id vocabulary, published.

This header declares **2** free functions and **1** constant.

Extracted from [`include/shulib/runner/manifest.hpp`](../../include/shulib/runner/manifest.hpp) — this page **is** that header's documentation, reformatted, so it cannot disagree with the code. Prose about *how to think about* the API lives in the [user guide](../guide/README.md); worked recipes live in the [cookbook](../cookbook/README.md); this page is the complete, mechanical list of what exists.

## Contents

- [`kManifestSchemaVersion`](#kmanifestschemaversion) — *constant*
- [`writeManifestEntry`](#writemanifestentry) — *free function*
- [`writeManifest`](#writemanifest) — *free function*

<a id="kmanifestschemaversion"></a>

## `kManifestSchemaVersion`

```cpp
inline constexpr int kManifestSchemaVersion = 1
```

The manifest format's own version, carried on the file's first line. Bumped only when a line's FIELDS change in a way a reader must know about; adding a field is additive and does not bump it (F8's breaking-vs-additive policy, applied to this artifact).

*constant, declared at [`include/shulib/runner/manifest.hpp:58`](../../include/shulib/runner/manifest.hpp#L58).*

<a id="writemanifestentry"></a>

## `writeManifestEntry`

```cpp
inline void writeManifestEntry(hal::ICharSink& sink, std::string_view id, bool builtin, std::uint8_t minArgs, std::uint8_t maxArgs, std::string_view doc)
```

One manifest record, as its line. Not itself the published contract — `writeManifest` is; this is where the five fields are spelled exactly once, so the two kinds of record cannot drift apart.  Precondition: `doc` is free of `"` and `\`. Docs are library-owned literals (`config::kBuiltinCommands`), so this is a self-check on the library rather than on a caller — and it is here because the day someone writes a built-in whose doc says `the "grab" id` is the day a picker silently stops parsing the file.

*free function, declared at [`include/shulib/runner/manifest.hpp:68`](../../include/shulib/runner/manifest.hpp#L68).*

<a id="writemanifest"></a>

## `writeManifest`

```cpp
inline void writeManifest(const CommandRegistry& registry, hal::ICharSink& sink)
```

Write the command-id manifest for `registry` to `sink` — one JSON object per line, the schema line first, then the built-ins in their fixed order, then the registered ids in registration order (file banner). Exactly one `write()` call per line, terminator included.

*free function, declared at [`include/shulib/runner/manifest.hpp:94`](../../include/shulib/runner/manifest.hpp#L94).*

## Design commentary, from the header

The header opens with the reasoning behind these shapes. It is reproduced here in full because a reference that only lists signatures teaches nobody *why*.

<details markdown="1" open>
<summary>The header’s own reasoning — 41 lines</summary>

```text

 writeManifest — the command-id vocabulary, published (chunk G2, 2026-09-18;
 docs/vexbuilder-integration.md §4.2, cross-team ask #4).

 ── Why the library publishes a file at all ─────────────────────────────────────────
 The no-code story only works if the person drawing a path in VexBuilder can PICK a command
 rather than type one. "Ids are not free-form strings the user types" is the promise made to
 VexBuilder in writing, and a promise like that needs an artifact: this writer emits the list
 of ids this robot's program actually understands — the four library built-ins, then every id
 the team registered — so a picker can render buttons for exactly them, with the right number
 of argument fields, and no shared list has to be kept in sync by hand.

 ── One JSON object per LINE, not one JSON document (ruling R4) ─────────────────────
 `hal::ICharSink`'s contract is "one write() call carries one complete line", so a
 line-oriented format is the one that rides the seam as it is, without buffering a document
 and without an implementation that can interleave. It also keeps the picker's parser
 trivial (read a line, parse an object) and the file diffable, which matters the first time
 someone asks "what changed in the vocabulary between these two builds?".

 The alternative — a single JSON document — was rejected because it needs a serializer with
 string escaping, and escaping is exactly what the id vocabulary rule
 (`config::commandIdValid`: `[a-z0-9_]`) exists to make unnecessary. The one free text in the
 file is the `doc` strings — the built-ins' own library-owned literals, and since chunk MECH-A4
 a registered id's optional one; a precondition below refuses a `"` or a `\` in either rather
 than emitting a file no parser can read.

 ── The ORDER is part of the contract ───────────────────────────────────────────────
 Schema line, then the built-ins in `config::kBuiltinCommands` order, then the registered ids
 in REGISTRATION order. The built-in order is APPEND-ONLY: a golden test pins these bytes, and
 a reordering would be a silent change to an artifact another team's tool parses.

 Every line carries the SAME five fields, including `"doc":""` for a registered id whose
 meaning the library does not know. A regular shape costs a few bytes and buys a picker that
 needs no special cases. Since chunk MECH-A4 a registered id MAY carry its doc
 (CommandEntry::doc — a robot's derived mechanism ids each do, runner/mechanism_routes.hpp);
 one registered without one still writes `""`, so a registry with no docs writes exactly the
 bytes it wrote before (the golden test pins them).

 FROZEN: F8 (docs/roadmap.md) — the field names and the order. New FIELDS may be appended.

 PROS-free, sim-free, allocation-free; no std::string, no heap.
```

</details>
