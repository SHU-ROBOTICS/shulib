// Adapter tests for ProsBlockSink THROUGH THE HOST SHIM + a real filesystem
// (chunk R1b). The usd_is_installed belief runs through the shim (HA-122);
// the FILE* path runs against REAL files — the same join+open+write code the
// robot runs against /usd/, pointed at a temp directory through the
// injectable mount root. What only these can prove: the /usd/ prefix logic,
// the no-card refusal, byte-verbatim writes, and that write()/flush() report
// device refusal honestly (a real refusing device: /dev/full).

#include "doctest.h"

#include <array>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "pros/shim_control.hpp"

#include "shulib/core/check.hpp"
#include "shulib/hal/pros/block_sink.hpp"

using shulib::PreconditionError;
using shulib::hal::pros::ProsBlockSink;

namespace {

namespace fs = std::filesystem;

/// A fresh scratch directory (with the trailing '/' the adapter demands),
/// unique per test run so parallel/stale runs cannot collide.
std::string scratchRoot() {
    const fs::path dir = fs::temp_directory_path() / "shulib_r1b_blocksink";
    fs::create_directories(dir);
    return dir.string() + "/";
}

std::vector<std::byte> readAll(const fs::path& p) {
    std::ifstream in(p, std::ios::binary);
    REQUIRE_MESSAGE(in.good(), "cannot open ", p.string());
    std::vector<char> raw((std::istreambuf_iterator<char>(in)),
                          std::istreambuf_iterator<char>());
    std::vector<std::byte> out;
    out.reserve(raw.size());
    for (const char c : raw) {
        out.push_back(static_cast<std::byte>(c));
    }
    return out;
}

}  // namespace

TEST_CASE("ProsBlockSink: bytes land VERBATIM at <mountRoot><fileName> (the one prefix join)") {
    // BUG CAUGHT (mutation 5's shape for the sink): framing/escaping snuck
    // into a VERBATIM device (0x00 or 0x0A translated — the blackbox decoder
    // reads a corrupt stream), or the path join broken (file lands somewhere
    // else; on the robot that is a blackbox that silently never exists).
    pros::shim::resetAll();
    pros::shim::usdState().installed = true;  // opt IN — the default is no card
    const std::string root = scratchRoot();
    const fs::path expected = fs::path(root) / "bb_verbatim.bin";
    fs::remove(expected);

    const std::array<std::byte, 6> payload{
        std::byte{0x00}, std::byte{0x0A}, std::byte{0xFF},
        std::byte{0x53}, std::byte{0x00}, std::byte{0xC5}};
    {
        ProsBlockSink sink{"bb_verbatim.bin", root.c_str()};
        REQUIRE(sink.isOpen());
        CHECK(std::string{sink.path()} == expected.string());
        CHECK(sink.write(payload) == true);
        CHECK(sink.flush() == true);
    }  // destruction closes the FILE*

    const auto onDisk = readAll(expected);
    REQUIRE(onDisk.size() == payload.size());
    for (std::size_t i = 0; i < payload.size(); ++i) {
        CHECK(onDisk[i] == payload[i]);
    }
}

TEST_CASE("ProsBlockSink: NO CARD — construction succeeds, everything refuses, nothing is created (T5)") {
    // BUG CAUGHT (mutation 11): the usd_is_installed() check removed — on
    // the host the fopen would succeed (the temp dir exists) and the sink
    // would report healthy writes with no card in the robot's slot: the
    // exact 'nothing looks wrong' failure the blackbox exists to avoid. The
    // shim's ADVERSARIAL default is already no-card; this test never opts in.
    pros::shim::resetAll();
    const std::string root = scratchRoot();
    const fs::path expected = fs::path(root) / "bb_nocard.bin";
    fs::remove(expected);

    ProsBlockSink sink{"bb_nocard.bin", root.c_str()};
    CHECK_FALSE(sink.isOpen());
    const std::array<std::byte, 2> payload{std::byte{0x01}, std::byte{0x02}};
    CHECK(sink.write(payload) == false);   // false from the FIRST call
    CHECK(sink.flush() == false);
    CHECK_FALSE(fs::exists(expected));     // the check gates the open itself
}

TEST_CASE("ProsBlockSink: card present but the open fails — a refusing sink, NEVER a throw (T5)") {
    // BUG CAUGHT: an open failure escalated to a throw — a missing/corrupt
    // SD filesystem would kill the robot at construction, in initialize(),
    // before anything drives. The ruling: a dead blackbox must never stop a
    // match.
    pros::shim::resetAll();
    pros::shim::usdState().installed = true;
    ProsBlockSink sink{"bb.bin", "/nonexistent_shulib_dir/"};
    CHECK_FALSE(sink.isOpen());
    const std::array<std::byte, 1> payload{std::byte{0x00}};
    CHECK(sink.write(payload) == false);
    CHECK(sink.flush() == false);
}

TEST_CASE("ProsBlockSink: a REAL refusing device — short write reports false (the /usd/-full shape)") {
    // BUG CAUGHT (mutation 10): write() returning true when the device
    // refused. /dev/full accepts the open and refuses every actual write —
    // the same shape as a full SD card mid-match. 64 KiB defeats stdio
    // buffering, so the refusal happens INSIDE this write() call.
    pros::shim::resetAll();
    pros::shim::usdState().installed = true;
    ProsBlockSink sink{"full", "/dev/"};
    REQUIRE_MESSAGE(sink.isOpen(), "/dev/full unavailable — this host cannot run the "
                                   "refusing-device test, and a skip would be decoration");
    const std::vector<std::byte> big(65536, std::byte{0xAB});
    CHECK(sink.write(big) == false);
}

TEST_CASE("ProsBlockSink: a buffered small write can say true — flush() is where the card's refusal lands") {
    // BUG CAUGHT: flush() ignoring fflush's result (or a flush() that lies
    // true while refusing). Small writes sit in newlib's buffer and report
    // accepted; the device refuses at flush — exactly why IBlockSink::flush
    // is bool and why E1's format tolerates a truncated tail. This is also
    // the honest limit of write()==true, recorded as behaviour.
    pros::shim::resetAll();
    pros::shim::usdState().installed = true;
    ProsBlockSink sink{"full", "/dev/"};
    REQUIRE(sink.isOpen());
    const std::array<std::byte, 8> small{};
    (void)sink.write(small);        // may report true — buffered
    CHECK(sink.flush() == false);   // the refusal must land HERE, visibly
}

TEST_CASE("ProsBlockSink: the /usd/ double-prefix mistake is a LOUD precondition, not a wrong path") {
    // BUG CAUGHT: a caller pre-prefixing "/usd/log.bin" (the vendored API
    // itself teaches two path conventions — HA-122): silently joined it
    // becomes "/usd//usd/log.bin" and the blackbox writes nowhere, wearing a
    // healthy construction. The adapter owns the prefix; a leading '/' must
    // refuse at the call site, loudly.
    pros::shim::resetAll();
    pros::shim::usdState().installed = true;
    CHECK_THROWS_AS(ProsBlockSink("/usd/log.bin"), PreconditionError);
    CHECK_THROWS_AS(ProsBlockSink(""), PreconditionError);
    CHECK_THROWS_AS(ProsBlockSink("log.bin", "/tmp"), PreconditionError);  // no trailing '/'
}
