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
#include <system_error>
#include <vector>

#include "pros/shim_control.hpp"

#include "shulib/core/check.hpp"
#include "shulib/hal/block_char_sink.hpp"
#include "shulib/hal/pros/block_sink.hpp"

using shulib::PreconditionError;
using shulib::hal::pros::AppendOpen;
using shulib::hal::pros::openForAppend;
using shulib::hal::pros::ProsBlockSink;
using shulib::hal::pros::ProsFileMode;

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

// ═══ THE APPEND MODE (chunk COMP-t, 2026-09-28; the brief's test (31)) ═══════════════════
// The Auton Tester keeps ONE run log on the card across reboots. What must never happen is an
// EXISTING log truncated: the append open is decided in one function, openForAppend(), probed here
// branch by branch with a scripted opener (VEXos's answer to an append on a missing file is
// unknown — A4 register HA-177 — so both answers are scripted), then through the shim on a real
// directory, boot after boot.

namespace {

/// A scripted fopen: which modes succeed, and the modes asked for, in order.
struct ScriptedOpen {
    bool exists = false;
    bool appendOk = true;
    bool createOk = true;
    std::string calls;
    int closes = 0;
    int opens = 0;
    char token = 0;  // its address stands in for a FILE*
};

ScriptedOpen script(bool exists, bool appendOk, bool createOk = true) {
    ScriptedOpen sc;
    sc.exists = exists;
    sc.appendOk = appendOk;
    sc.createOk = createOk;
    return sc;
}

std::FILE* decide(ScriptedOpen& sc, AppendOpen& how) {
    return openForAppend(
        "log.txt",
        [&sc](const char* path, const char* mode) -> std::FILE* {
            CHECK(std::string{path} == "log.txt");
            // never read-write: the brain refuses a read-write open (the vendored libpros's
            // disassembly of 2026-09-26 -- SAFE-SAVE's belief, not yet in the register)
            CHECK(std::string{mode}.find('+') == std::string::npos);
            sc.calls += mode;
            sc.calls += ' ';
            const std::string m{mode};
            const bool ok = m == "rb" ? sc.exists : m == "ab" ? sc.appendOk : sc.createOk;
            if (ok) {
                ++sc.opens;
            }
            return ok ? reinterpret_cast<std::FILE*>(&sc.token) : nullptr;
        },
        [&sc](std::FILE*) { ++sc.closes; }, how);
}

/// How many file descriptors this process holds open right now (Linux: /proc/self/fd), or -1 where
/// that cannot be read. The directory listing's own descriptor is in every count alike.
int openDescriptors() {
    const fs::path fds{"/proc/self/fd"};
    std::error_code ec;
    if (!fs::is_directory(fds, ec)) {
        return -1;
    }
    int n = 0;
    for (auto it = fs::directory_iterator(fds, ec); !ec && it != fs::directory_iterator();
         it.increment(ec)) {
        ++n;
    }
    return ec ? -1 : n;
}

std::string readText(const fs::path& p) {
    std::ifstream in(p, std::ios::binary);
    return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

}  // namespace

TEST_CASE("openForAppend: the four branches — and an existing log is NEVER truncated") {
    // BUG CAUGHT (mutations 34 and 35): a fallback to "wb" when the file EXISTED but the append
    // open failed (the whole log emptied — the one outcome this mode exists to prevent), or no "rb"
    // probe at all (Appended and Created indistinguishable, and the existed branch truncates).
    AppendOpen how = AppendOpen::Refused;

    ScriptedOpen a = script(true, true);
    CHECK(decide(a, how) != nullptr);
    CHECK(how == AppendOpen::Appended);
    CHECK(a.calls == "rb ab ");
    CHECK(a.closes == 1);  // the probe is closed

    ScriptedOpen b = script(false, true);
    CHECK(decide(b, how) != nullptr);
    CHECK(how == AppendOpen::Created);
    CHECK(b.calls == "rb ab ");
    CHECK(b.closes == 0);

    ScriptedOpen c = script(false, false, true);
    CHECK(decide(c, how) != nullptr);
    CHECK(how == AppendOpen::Created);
    CHECK(c.calls == "rb ab wb ");  // "ab" refused an absent file: create it

    ScriptedOpen d = script(true, false, true);
    CHECK(decide(d, how) == nullptr);
    CHECK(how == AppendOpen::Refused);
    CHECK(d.calls == "rb ab ");  // NO "wb": the existing log is left as it is
    CHECK(d.calls.find("wb") == std::string::npos);

    ScriptedOpen e = script(false, false, false);
    CHECK(decide(e, how) == nullptr);
    CHECK(how == AppendOpen::Refused);
}

TEST_CASE("ProsBlockSink Append: two boots leave both lines, in order (CREATED, then APPENDED)") {
    // BUG CAUGHT: the Append constructor still opening "wb" (every reboot would erase the earlier
    // boots' runs), the path join broken in the new constructor, or appendOpen() misreporting —
    // the boot line would tell the operator CREATED on a card that already held a session.
    pros::shim::resetAll();
    pros::shim::usdState().installed = true;
    const std::string root = scratchRoot();
    const fs::path file = fs::path(root) / "runs_append.txt";
    fs::remove(file);
    {
        ProsBlockSink sd{"runs_append.txt", ProsFileMode::Append, root.c_str()};
        REQUIRE(sd.isOpen());
        CHECK(std::string{sd.path()} == file.string());
        CHECK(sd.appendOpen() == AppendOpen::Created);
        shulib::hal::BlockCharSink chars{sd};
        chars.write("boot 1 run 1\n");
        CHECK(chars.droppedWrites() == 0);
    }
    {
        ProsBlockSink sd{"runs_append.txt", ProsFileMode::Append, root.c_str()};
        REQUIRE(sd.isOpen());
        CHECK(sd.appendOpen() == AppendOpen::Appended);
        shulib::hal::BlockCharSink chars{sd};
        chars.write("boot 2 run 1\n");
    }
    CHECK(readText(file) == "boot 1 run 1\nboot 2 run 1\n");

    // THE ADAPTER'S OWN CLOSE (COMP-t's review, 2026-09-29: H13 — the real adapter's close callback
    // made a no-op SURVIVED; the scripted opener above counts closes on the TEMPLATE only). On an
    // existing file the probe "rb" is opened and must be closed before this constructor returns:
    // exactly ONE descriptor more while the sink lives (its "ab" handle), none after it. A leaked
    // "rb" handle on the same file during the "ab" open is exactly the untested VEXos case HA-177 is
    // about — never left to chance on the brain.
    const int fdBefore = openDescriptors();
    if (fdBefore < 0) {
        WARN_MESSAGE(false, "no /proc/self/fd on this host: the adapter's close is not observed");
    } else {
        {
            ProsBlockSink sd{"runs_append.txt", ProsFileMode::Append, root.c_str()};
            REQUIRE(sd.appendOpen() == AppendOpen::Appended);  // the probe WAS opened
            CHECK(openDescriptors() == fdBefore + 1);  // the "ab" handle, and no leaked probe
        }
        CHECK(openDescriptors() == fdBefore);  // and the destructor closed that one
    }

    // No card: Refused, nothing created, nothing written.
    pros::shim::resetAll();
    const fs::path none = fs::path(root) / "runs_nocard.txt";
    fs::remove(none);
    {
        ProsBlockSink sd{"runs_nocard.txt", ProsFileMode::Append, root.c_str()};
        CHECK_FALSE(sd.isOpen());
        CHECK(sd.appendOpen() == AppendOpen::Refused);
    }
    CHECK_FALSE(fs::exists(none));
    // The same preconditions as the two-argument constructor.
    CHECK_THROWS_AS(ProsBlockSink("/usd/x.txt", ProsFileMode::Append), PreconditionError);
}

TEST_CASE("ProsBlockSink: the two-argument constructor still TRUNCATES (every existing caller)") {
    // BUG CAUGHT: the delegation changing the old constructor's meaning — the blackbox appending to
    // last boot's file (its decoder would read two sessions as one) — or Truncate reporting an
    // append it never made.
    pros::shim::resetAll();
    pros::shim::usdState().installed = true;
    const std::string root = scratchRoot();
    const fs::path file = fs::path(root) / "bb_truncate.bin";
    {
        std::ofstream old(file, std::ios::binary);
        old << "last boot's bytes";
    }
    {
        ProsBlockSink sink{"bb_truncate.bin", root.c_str()};
        REQUIRE(sink.isOpen());
        CHECK(sink.appendOpen() == AppendOpen::Refused);  // Truncate never asks
        const std::array<std::byte, 2> payload{std::byte{'o'}, std::byte{'k'}};
        CHECK(sink.write(payload));
        CHECK(sink.flush());
    }
    CHECK(readText(file) == "ok");
    {
        ProsBlockSink sink{"bb_truncate.bin", ProsFileMode::Truncate, root.c_str()};
        REQUIRE(sink.isOpen());
    }
    CHECK(readText(file).empty());
}
