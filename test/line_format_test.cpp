// Tests for diag/line_format.hpp — the bounded line builder every §18.4 renderer writes
// through. NEW AT DEFECTS1: this header had no direct test file at all, which is how both
// defects below survived. Its behaviour was covered only incidentally, through the renderers
// that happen to call it with the shapes they happen to use — and both defects live exactly
// where no shipped renderer goes yet.
//
// What each case targets:
//  * A5 — appendSanitized's truncation marker could be emitted PARTIALLY when fewer than
//    three bytes remained, leaving an invalid multi-byte UTF-8 tail: the one path in this
//    file that could emit malformed UTF-8, in the very function whose job is to make
//    caller-controlled text safe to frame.
//  * A6 — appendNum measured "pathological" against a fixed 10 bytes instead of against the
//    column the caller asked for, so every width above 10 destroyed the column it reserved.

#include "doctest.h"

#include <string>
#include <string_view>

#include "shulib/diag/line_format.hpp"

using shulib::diag::lineformat::appendNum;
using shulib::diag::lineformat::Line;

// Bug caught (DEFECTS1 item A6): the compaction trigger was `len > kCompactThresholdBytes`
// with the threshold a fixed 10. A `%*.*f` rendering is AT LEAST `width` bytes, so any column
// wider than 10 tripped it for perfectly ordinary values: appendNum(line, 1.0, 12, 2)
// rendered "        1.00" and then threw it away for "1", destroying the column the caller
// asked for. Latent only because the widest live caller is appendTimestamp's 7 — the guard
// the banner describes ("a rendering longer than kCompactThresholdBytes is pathological") is
// only true while width stays well under 10.
TEST_CASE("A6: a wide column is honoured, not compacted away") {
    Line line;
    appendNum(line, 1.0, 12, 2);
    CHECK(line.view() == "        1.00");   // 12 bytes, as asked; was "1"

    Line narrow;
    appendNum(narrow, 1.0, 6, 2);
    CHECK(narrow.view() == "  1.00");

    // NEGATIVE CONTROL: compaction must STILL fire for a genuinely pathological rendering —
    // one longer than both the requested width and the threshold — or the fix would simply
    // have disabled the guard rather than corrected its comparison.
    Line huge;
    appendNum(huge, 1.0e30, 4, 2);
    CHECK(huge.view() == "1e+30");
    CHECK(huge.view().size() < 20);         // not the 30-odd digits of %f
}

// Bug caught (DEFECTS1 item A5): appendSanitized appended the 3-byte "…" unconditionally on
// truncation, and appendRaw silently clips to the room left — so with 1 or 2 bytes free it
// memcpy'd a PARTIAL "…" (0xE2, or 0xE2 0x80), leaving a truncated multi-byte sequence at the
// end of the line. That is exactly the breakage the UTF-8 continuation back-off two lines
// above exists to prevent, reintroduced by the marker meant to announce it.
TEST_CASE("A5: a truncation marker is never emitted in pieces") {
    // The window has to be chosen exactly, and getting it wrong is how a first draft of this
    // test passed against the UNFIXED code: leave ZERO bytes free and appendRaw writes nothing
    // at all, so no partial marker appears and the mutation survives. The bug needs 1 or 2
    // bytes free AFTER the sanitized text lands. Fill to kCapacity - 4, then truncate a
    // 2-character window: the copy takes the line to kCapacity - 2, leaving exactly 2 bytes —
    // enough for appendRaw to clip "…" (0xE2 0x80 0xA6) to its first two bytes.
    Line line;
    const std::string filler(Line::kCapacity - 4, 'x');
    line.appendRaw(filler.data(), filler.size());
    REQUIRE(line.view().size() == Line::kCapacity - 4);

    line.appendSanitized("abcdef", 2);   // truncates: would append "ab" + "…"
    REQUIRE(line.view().size() <= Line::kCapacity);

    const std::string_view out = line.view();
    // Every byte on the line is ASCII: no lone 0xE2 / 0x80 tail survived.
    for (const char c : out) {
        CHECK(static_cast<unsigned char>(c) < 0x80U);
    }

    // NEGATIVE CONTROL: with room, the marker IS still appended whole — the fix must not have
    // simply stopped marking truncation.
    Line roomy;
    roomy.appendSanitized("abcdef", 3);
    CHECK(roomy.view() == "abc…");
}

// Bug caught: the sanitizer letting a control byte through would break the one-line-per-write
// framing every sink depends on. Pinned here because this file had no test of its own.
TEST_CASE("appendSanitized: control bytes become '?', framing survives caller text") {
    Line line;
    line.appendSanitized("a\nb\tc\x1b" "d\x7f", 64);
    CHECK(line.view() == "a?b?c?d?");
}
