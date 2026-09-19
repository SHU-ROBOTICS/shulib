#pragma once
//
// writeManifest — the command-id vocabulary, published (chunk G2, 2026-09-18;
// docs/vexbuilder-integration.md §4.2, cross-team ask #4).
//
// ── Why the library publishes a file at all ─────────────────────────────────────────
// The no-code story only works if the person drawing a path in VexBuilder can PICK a command
// rather than type one. "Ids are not free-form strings the user types" is the promise made to
// VexBuilder in writing, and a promise like that needs an artifact: this writer emits the list
// of ids this robot's program actually understands — the four library built-ins, then every id
// the team registered — so a picker can render buttons for exactly them, with the right number
// of argument fields, and no shared list has to be kept in sync by hand.
//
// ── One JSON object per LINE, not one JSON document (ruling R4) ─────────────────────
// `hal::ICharSink`'s contract is "one write() call carries one complete line", so a
// line-oriented format is the one that rides the seam as it is, without buffering a document
// and without an implementation that can interleave. It also keeps the picker's parser
// trivial (read a line, parse an object) and the file diffable, which matters the first time
// someone asks "what changed in the vocabulary between these two builds?".
//
// The alternative — a single JSON document — was rejected because it needs a serializer with
// string escaping, and escaping is exactly what the id vocabulary rule
// (`config::commandIdValid`: `[a-z0-9_]`) exists to make unnecessary. The one free text in the
// file is the built-ins' own `doc` strings, which are library-owned literals; a precondition
// below refuses a `"` or a `\` in one rather than emitting a file no parser can read.
//
// ── The ORDER is part of the contract ───────────────────────────────────────────────
// Schema line, then the built-ins in `config::kBuiltinCommands` order, then the registered ids
// in REGISTRATION order. The built-in order is APPEND-ONLY: a golden test pins these bytes, and
// a reordering would be a silent change to an artifact another team's tool parses.
//
// Every line carries the SAME five fields, including `"doc":""` for a registered id whose
// meaning the library does not know. A regular shape costs a few bytes and buys a picker that
// needs no special cases.
//
// FROZEN: F8 (docs/roadmap.md) — the field names and the order. New FIELDS may be appended.
//
// PROS-free, sim-free, allocation-free; no std::string, no heap.

#include <cstdint>
#include <string_view>

#include "shulib/config/route.hpp"
#include "shulib/core/check.hpp"
#include "shulib/diag/line_format.hpp"
#include "shulib/hal/char_sink.hpp"
#include "shulib/runner/command_registry.hpp"

namespace shulib::runner {

/// The manifest format's own version, carried on the file's first line. Bumped only when a line's
/// FIELDS change in a way a reader must know about; adding a field is additive and does not bump
/// it (F8's breaking-vs-additive policy, applied to this artifact).
inline constexpr int kManifestSchemaVersion = 1;

/// One manifest record, as its line. Not itself the published contract — `writeManifest` is;
/// this is where the five fields are spelled exactly once, so the two kinds of record cannot
/// drift apart.
///
/// Precondition: `doc` is free of `"` and `\`. Docs are library-owned literals
/// (`config::kBuiltinCommands`), so this is a self-check on the library rather than on a caller —
/// and it is here because the day someone writes a built-in whose doc says `the "grab" id` is the
/// day a picker silently stops parsing the file.
inline void writeManifestEntry(hal::ICharSink& sink, std::string_view id, bool builtin,
                               std::uint8_t minArgs, std::uint8_t maxArgs, std::string_view doc) {
    SHULIB_PRECONDITION(doc.find('"') == std::string_view::npos
                            && doc.find('\\') == std::string_view::npos,
                        "writeManifest: a command's doc string contains a quote or a backslash, "
                        "which this deliberately escape-free format cannot carry");
    diag::lineformat::Line line;
    line.appendLiteral("{\"id\":\"");
    line.appendRaw(id.data(), id.size());  // commandIdValid: [a-z0-9_], nothing to escape
    line.appendLiteral("\",\"builtin\":");
    line.appendLiteral(builtin ? "true" : "false");
    line.appendLiteral(",\"minArgs\":");
    diag::lineformat::appendUnsigned(line, static_cast<unsigned long>(minArgs));
    line.appendLiteral(",\"maxArgs\":");
    diag::lineformat::appendUnsigned(line, static_cast<unsigned long>(maxArgs));
    line.appendLiteral(",\"doc\":\"");
    if (!doc.empty()) {
        line.appendRaw(doc.data(), doc.size());
    }
    line.appendLiteral("\"}\n");
    sink.write(line.view());
}

/// Write the command-id manifest for `registry` to `sink` — one JSON object per line, the schema
/// line first, then the built-ins in their fixed order, then the registered ids in registration
/// order (file banner). Exactly one `write()` call per line, terminator included.
inline void writeManifest(const CommandRegistry& registry, hal::ICharSink& sink) {
    diag::lineformat::Line header;
    header.appendLiteral("{\"manifestSchemaVersion\":");
    diag::lineformat::appendUnsigned(header, static_cast<unsigned long>(kManifestSchemaVersion));
    header.appendLiteral(",\"builtins\":");
    diag::lineformat::appendUnsigned(header,
                                     static_cast<unsigned long>(config::kBuiltinCommandCount));
    header.appendLiteral(",\"commands\":");
    diag::lineformat::appendUnsigned(header, static_cast<unsigned long>(registry.count()));
    header.appendLiteral("}\n");
    sink.write(header.view());

    for (const config::BuiltinCommand& b : config::kBuiltinCommands) {
        writeManifestEntry(sink, std::string_view{b.id}, true, b.minArgs, b.maxArgs,
                           std::string_view{b.doc});
    }
    for (const CommandEntry& e : registry.ids()) {
        writeManifestEntry(sink, e.idView(), false, e.argCount, e.argCount, std::string_view{});
    }
}

}  // namespace shulib::runner
