// runner::writeManifest — the published command-id vocabulary (chunk G2, brief §4 test 11).
//
// This file is a GOLDEN test, byte for byte, and that is the point. The manifest is an artifact
// another team's tool parses: VexBuilder's picker reads it to decide which buttons exist and how
// many argument fields each one gets. A change to a field name, to the order of the built-ins,
// or to the framing of a line is a change to someone else's parser, and the only way to make
// that visible at the moment it happens is to assert the bytes.
//
// The built-in ORDER is part of the contract (G2 landmine 5). It is append-only: a new built-in
// goes at the END of config::kBuiltinCommands, never in the middle, and this golden is where
// that shows up if anyone forgets.

#include "doctest.h"

#include <string>
#include <vector>

#include "shulib/config/route.hpp"
#include "shulib/hal/char_sink.hpp"
#include "shulib/hal/fake/fake_char_sink.hpp"
#include "shulib/runner/command_registry.hpp"
#include "shulib/runner/manifest.hpp"

namespace cfgns = shulib::config;
using shulib::hal::fake::FakeCharSink;
using shulib::runner::CommandRegistry;
using shulib::runner::writeManifest;

namespace {

/// A sink that keeps each write() SEPARATE, so the "one write carries one complete line"
/// contract is a testable claim rather than an assertion about the concatenation.
class LineCountingSink final : public shulib::hal::ICharSink {
public:
    void write(std::string_view text) override { writes.emplace_back(text); }
    std::vector<std::string> writes;
};

}  // namespace

// ═══ 11. The golden ══════════════════════════════════════════════════════════════════
// Bug caught: any silent change to the artifact — a renamed field, a reordered built-in, a
// registered id published out of registration order, a missing newline that would glue two
// records together in the picker's reader.
TEST_CASE("G2 manifest 11: the exact bytes, built-ins first, registrations in order") {
    auto intakeIn = [] {};
    auto liftTo = [](const cfgns::Marker&) {};

    CommandRegistry registry;
    registry.on("intake_in", intakeIn);
    registry.on("lift_to_level", liftTo, 1);

    FakeCharSink sink;
    writeManifest(registry, sink);

    const std::string expected =
        "{\"manifestSchemaVersion\":1,\"builtins\":4,\"commands\":2}\n"
        "{\"id\":\"wait_ms\",\"builtin\":true,\"minArgs\":1,\"maxArgs\":1,\"doc\":\"wait, "
        "commanding nothing, for args[0] milliseconds (the drive keeps its last state)\"}\n"
        "{\"id\":\"brake\",\"builtin\":true,\"minArgs\":0,\"maxArgs\":0,\"doc\":\"stop the drive "
        "(0 V + Brake) and block until the estimate certifies rest\"}\n"
        "{\"id\":\"hold_ms\",\"builtin\":true,\"minArgs\":1,\"maxArgs\":1,\"doc\":\"actively hold "
        "the current pose for args[0] milliseconds, driving back disturbance\"}\n"
        "{\"id\":\"log\",\"builtin\":true,\"minArgs\":0,\"maxArgs\":2,\"doc\":\"write one Info "
        "line naming this marker and its args\"}\n"
        "{\"id\":\"intake_in\",\"builtin\":false,\"minArgs\":0,\"maxArgs\":0,\"doc\":\"\"}\n"
        "{\"id\":\"lift_to_level\",\"builtin\":false,\"minArgs\":1,\"maxArgs\":1,\"doc\":\"\"}\n";

    CHECK(sink.text() == expected);
}

// ═══ 11b. One write per line, and an empty registry is still a valid manifest ════════
// Bug caught: a writer that buffers a document and emits it in one write (breaking ICharSink's
// framing contract, which is what lets a terminal sink be atomic per line), or one that emits
// nothing at all when no team has registered anything — the built-ins are always available, so
// a picker must always see them.
TEST_CASE("G2 manifest 11b: exactly one write() per record, terminator included") {
    auto fn = [] {};
    CommandRegistry registry;
    registry.on("clamp", fn);

    LineCountingSink sink;
    writeManifest(registry, sink);

    REQUIRE(sink.writes.size() == 1 + cfgns::kBuiltinCommandCount + 1);
    for (const std::string& w : sink.writes) {
        CAPTURE(w);
        CHECK(w.back() == '\n');
        CHECK(w.find('\n') == w.size() - 1);  // exactly one, and it is the terminator
        CHECK(w.front() == '{');
    }

    LineCountingSink empty;
    CommandRegistry none;
    writeManifest(none, empty);
    REQUIRE(empty.writes.size() == 1 + cfgns::kBuiltinCommandCount);
    CHECK(empty.writes[0] == "{\"manifestSchemaVersion\":1,\"builtins\":4,\"commands\":0}\n");
}

// ═══ 11c. Nothing in the published vocabulary needs escaping ═════════════════════════
// Bug caught: a built-in added later whose id or doc carries a quote or a backslash, which this
// deliberately escape-free format cannot represent. The id side is structural (the charset rule
// forbids both characters); the doc side is a library-owned literal and therefore a self-check.
TEST_CASE("G2 manifest 11c: every published id is in the vocabulary and no doc needs escaping") {
    for (const cfgns::BuiltinCommand& b : cfgns::kBuiltinCommands) {
        CAPTURE(b.id);
        CHECK(cfgns::commandIdValid(std::string_view{b.id}));
        const std::string_view doc{b.doc};
        CHECK(doc.find('"') == std::string_view::npos);
        CHECK(doc.find('\\') == std::string_view::npos);
        CHECK(doc.find('\n') == std::string_view::npos);
        CHECK(b.maxArgs <= cfgns::kMaxMarkerArgs);
        CHECK(b.minArgs <= b.maxArgs);
    }
    CHECK(cfgns::kBuiltinCommandCount == 4);
}
