// block_char_sink_test.cpp — hal::BlockCharSink (include/shulib/hal/block_char_sink.hpp; chunk
// COMP-t, 2026-09-28; the brief's test (32)): an ICharSink over an IBlockSink, one block write and
// one flush per line, a refusal dropped and COUNTED, never thrown. The Auton Tester's SD run log is
// a diag::TermSink over this over the card's adapter; the driver program's SD copy of serial will
// reuse it. Every case names the bug it would catch.

#include "doctest.h"

#include <cstddef>
#include <string>
#include <string_view>

#include "shulib/diag/term_sink.hpp"
#include "shulib/hal/block_char_sink.hpp"
#include "shulib/hal/fake/fake_block_sink.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/telemetry_sink.hpp"
#include "shulib/units/quantity.hpp"

using shulib::hal::BlockCharSink;
using shulib::hal::fake::FakeBlockSink;

namespace {

std::string text(const FakeBlockSink& blk) {
    std::string out;
    for (const std::byte b : blk.bytes()) {
        out += static_cast<char>(b);
    }
    return out;
}

}  // namespace

// Bug caught: bytes changed on the way (a newline translated, a line merged with the next), a line
// split over several block writes (a card yanked between them leaves half a line), or no flush (the
// line sits in a buffer when the power goes).
TEST_CASE("BlockCharSink: each line verbatim, as ONE block write and ONE flush") {
    FakeBlockSink blk;
    BlockCharSink chars{blk};
    chars.write("[t=  1.00][INFO][ATS] run 1: '40in forward' FINISHED\n");
    CHECK(blk.writeCalls() == 1);
    CHECK(blk.flushCalls() == 1);
    chars.write(std::string_view{"a\0b\n", 4});  // bytes, not a C string
    CHECK(blk.writeCalls() == 2);
    CHECK(blk.flushCalls() == 2);
    const std::string expected{"[t=  1.00][INFO][ATS] run 1: '40in forward' FINISHED\na\0b\n", 57};
    CHECK(text(blk) == expected);
    CHECK(chars.droppedWrites() == 0);
    chars.write("");  // an empty line is still one write and one flush, and not a refusal
    CHECK(blk.writeCalls() == 3);
    CHECK(chars.droppedWrites() == 0);
}

// Bug caught (mutation 36): a refused FLUSH ignored (the card lost the line and the count says
// none were lost), a refused write not counted, or either one THROWN (a full card must never stop
// a robot) — and a flush skipped after a refused write (what the card did take is pushed out).
TEST_CASE("BlockCharSink: a refused write or flush is dropped and counted, never thrown") {
    FakeBlockSink blk;
    BlockCharSink chars{blk};
    blk.setFlushSucceeds(false);
    CHECK_NOTHROW(chars.write("lost at the flush\n"));
    CHECK(chars.droppedWrites() == 1);
    blk.setFlushSucceeds(true);
    chars.write("kept\n");
    CHECK(chars.droppedWrites() == 1);
    blk.setCapacity(blk.size() + 3);  // the card fills three bytes into the next line
    CHECK_NOTHROW(chars.write("cut short\n"));
    CHECK(chars.droppedWrites() == 2);
    CHECK(blk.flushCalls() == 3);  // flushed even after the refused write
    CHECK(blk.shortWrites() == 1);
}

// Bug caught: the SD run log's second TermSink not writing TermSink's line — the file would hold
// something other than what serial shows. Through a real diag::TermSink, as the program uses it.
TEST_CASE("BlockCharSink: a diag::TermSink over it puts serial's exact line on the block device") {
    FakeBlockSink blk;
    BlockCharSink chars{blk};
    shulib::hal::fake::FakeClock clock;
    clock.set(shulib::units::Time{12.5});
    shulib::diag::TermSink log{clock, chars};
    log.log(shulib::hal::LogLevel::Info, "ATS", "boot: shulib AUTON TESTER");
    const std::string line = text(blk);
    CHECK(line.find("ATS") != std::string::npos);
    CHECK(line.find("boot: shulib AUTON TESTER") != std::string::npos);
    CHECK(line.back() == '\n');
    CHECK(blk.writeCalls() == 1);
}
