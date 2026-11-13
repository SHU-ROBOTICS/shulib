// serial_line_test.cpp — ONE LINE, ONE write() CALL (chunk SER1, 2026-10-08; A4 register HA-189):
// hal::pros::ProsCharSink, serialLine(), serialWrapped() and writeWhole()
// (include/shulib/hal/pros/char_sink.hpp), and the source pins that keep every src/ line on that path.
//
// WHAT THESE TESTS PROVE, AND WHAT THEY CANNOT. On the brain every task shares ONE stdout FILE,
// line-buffered, and the vendored newlib's per-FILE locks are no-op stubs — two tasks inside
// printf/fwrite on stdout at once can mix or lose bytes; ONE write() call is kept whole by PROS's
// serial write mutex (HA-189, reasoned from the vendored libc.a/libpros.a). The host runs glibc,
// not newlib, so these tests prove shulib's CALLS — how many write()s a line takes, that none of
// them goes through a FILE's buffer, that a short write sends the rest and a failed one stops —
// and never newlib's or PROS's behaviour (HA-189's bench settle, worksheet Station K, does that).
//
// HOW A CALL IS SEEN. A SOCK_DGRAM socketpair dup'd onto fd 1 (StdoutCapture, RAII — every test
// shares one binary, and doctest itself prints on fd 1): each write() call arrives as exactly ONE
// datagram, so "one line, one call" is counted from the outside, with no seam in the code under
// test. The only seam is writeWhole()'s injectable write, used ONLY for the short-write and
// failure fakes, which no real descriptor produces on demand (the brief's amendment A6).

#include "doctest.h"

#include <fcntl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <atomic>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <set>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

#include "source_pins.hpp"

#include "shulib/core/check.hpp"
#include "shulib/diag/line_format.hpp"
#include "shulib/hal/pros/char_sink.hpp"

using shulib::hal::pros::kSerialCutMarker;
using shulib::hal::pros::kSerialLineBytes;
using shulib::hal::pros::kSerialTagBytes;
using shulib::hal::pros::kSerialWriteMaxBytes;
using shulib::hal::pros::ProsCharSink;
using shulib::hal::pros::serialLine;
using shulib::hal::pros::serialWrapped;
using shulib::hal::pros::writeWhole;

namespace {

/// fd 1 pointed at a SOCK_DGRAM socket for the object's life, restored by the destructor (or
/// restore()). Each write() on fd 1 becomes one datagram; datagrams() reads them back in order.
/// A test can never hang on a full queue: by default the sending end is NON-BLOCKING (an
/// over-full queue makes write() fail, which a test sees as a missing line); a test with a reader
/// thread asks for a BLOCKING end with a 5 s send timeout instead (a stalled reader fails it, never
/// hangs it). ASSERT NOTHING while it is live — doctest reports on fd 1, into the socket; collect,
/// restore, then CHECK.
class StdoutCapture {
public:
    explicit StdoutCapture(bool blocking = false) {
        std::cout.flush();
        (void)std::fflush(stdout);  // nothing of the suite's own left in stdout's buffer
        if (::socketpair(AF_UNIX, SOCK_DGRAM, 0, sv_) != 0) {
            return;
        }
        if (blocking) {
            timeval tv{5, 0};
            if (::setsockopt(sv_[0], SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof tv) != 0) {
                return;
            }
        } else {
            const int flags = ::fcntl(sv_[0], F_GETFL);
            if (flags < 0 || ::fcntl(sv_[0], F_SETFL, flags | O_NONBLOCK) != 0) {
                return;
            }
        }
        saved_ = ::dup(STDOUT_FILENO);
        ok_ = saved_ >= 0 && ::dup2(sv_[0], STDOUT_FILENO) == STDOUT_FILENO;
    }
    StdoutCapture(const StdoutCapture&) = delete;
    StdoutCapture& operator=(const StdoutCapture&) = delete;
    ~StdoutCapture() {
        restore();
        for (const int fd : sv_) {
            if (fd >= 0) {
                ::close(fd);
            }
        }
    }

    [[nodiscard]] bool ok() const { return ok_; }

    /// fd 1 back to what it was.
    void restore() {
        if (saved_ >= 0) {
            (void)::dup2(saved_, STDOUT_FILENO);
            ::close(saved_);
            saved_ = -1;
        }
    }

    /// Every datagram queued so far, in order (each one write() call).
    [[nodiscard]] std::vector<std::string> datagrams() const {
        std::vector<std::string> out;
        std::vector<char> buf(65536);
        for (;;) {
            const ssize_t n = ::recv(sv_[1], buf.data(), buf.size(), MSG_DONTWAIT);
            if (n < 0) {
                break;
            }
            out.emplace_back(buf.data(), static_cast<std::size_t>(n));
        }
        return out;
    }

    /// The receiving end, for a reader thread (the two-task test).
    [[nodiscard]] int readEnd() const { return sv_[1]; }

private:
    int sv_[2] = {-1, -1};
    int saved_ = -1;
    bool ok_ = false;
};

/// What `fn` wrote on fd 1, one string per write() call. `ok` is false if fd 1 could not be
/// captured (the case then FAILS: a capture that saw nothing proves nothing).
struct Captured {
    bool ok = false;
    std::vector<std::string> calls;
};

template <class Fn>
Captured capture(Fn&& fn) {
    Captured c;
    StdoutCapture cap;
    c.ok = cap.ok();
    if (c.ok) {
        fn();
        c.calls = cap.datagrams();
    }
    return c;
}

/// The bytes at the start of `f`'s DESCRIPTOR — what reached the file, not what waits in the
/// FILE's buffer.
std::string fileBytes(std::FILE* f) {
    char buf[256] = {};
    const ssize_t n = ::pread(::fileno(f), buf, sizeof buf, 0);
    return std::string(buf, n > 0 ? static_cast<std::size_t>(n) : 0);
}

/// A string of `n` copies of `c`.
std::string run(std::size_t n, char c = 'x') { return std::string(n, c); }

/// serialWrapped()'s pieces read back into the text they carry: each piece must end in its one
/// '\n', begin `[tag] ` (the first) or `[tag] ... ` (every later one), and fit kSerialLineBytes.
/// Returns false (and says why in `why`) when one does not.
bool piecesText(const std::vector<std::string>& pieces, const std::string& prefix,
                std::vector<std::string>& texts, std::string& why) {
    texts.clear();
    for (std::size_t i = 0; i < pieces.size(); ++i) {
        const std::string& p = pieces[i];
        const std::string head = i == 0 ? prefix : prefix + "... ";
        if (p.size() > kSerialLineBytes) {
            why = "piece " + std::to_string(i) + " is " + std::to_string(p.size()) + " bytes";
            return false;
        }
        if (p.empty() || p.back() != '\n' || p.find('\n') != p.size() - 1) {
            why = "piece " + std::to_string(i) + " does not end in exactly one newline";
            return false;
        }
        if (p.compare(0, head.size(), head) != 0) {
            why = "piece " + std::to_string(i) + " does not begin `" + head + "`: " + p;
            return false;
        }
        texts.push_back(p.substr(head.size(), p.size() - head.size() - 1));
    }
    return true;
}

/// True when `texts`, the pieces of ONE line, are exactly `original` broken at spaces (the ONE
/// space at the break dropped) or hard-broken (nothing dropped): no character lost, none doubled,
/// none reordered. A break at a space is the only place a character may go missing, it must be a
/// space, and it is exactly one: a hard break never lands before a space (a space there is a break
/// AT it), so a space after a piece's text is always the break's. (Until 2026-10-15 this let a piece
/// that BEGAN with a space keep the break's space — the old code's doubled space; that is now a
/// failure.)
bool rejoins(const std::vector<std::string>& texts, const std::string& original) {
    std::size_t pos = 0;
    for (std::size_t i = 0; i < texts.size(); ++i) {
        if (original.compare(pos, texts[i].size(), texts[i]) != 0) {
            return false;
        }
        pos += texts[i].size();
        const bool last = i + 1 == texts.size();
        if (!last && pos < original.size() && original[pos] == ' ') {
            ++pos;  // the space this break consumed
        }
    }
    return pos == original.size();
}

/// An INDEPENDENT greedy word wrap — the oracle for serialWrapped()'s rule (it shares no code with
/// it): `text`, words separated by single spaces and each word shorter than a piece, as the lines
/// a greedy fill makes — each as many whole words as fit `firstRoom` characters (the first line) or
/// `room` (every later one), joined by one space, the space at each break dropped.
std::vector<std::string> greedyWrap(const std::string& text, std::size_t firstRoom,
                                    std::size_t room) {
    std::vector<std::string> lines;
    std::string cur;
    std::size_t cap = firstRoom;
    std::size_t pos = 0;
    for (;;) {
        std::size_t sp = text.find(' ', pos);
        if (sp == std::string::npos) {
            sp = text.size();
        }
        const std::string word = text.substr(pos, sp - pos);
        if (cur.empty() || cur.size() + 1 + word.size() <= cap) {
            cur += cur.empty() ? word : " " + word;
        } else {
            lines.push_back(cur);
            cap = room;
            cur = word;
        }
        if (sp == text.size()) {
            break;
        }
        pos = sp + 1;
    }
    lines.push_back(cur);
    return lines;
}

/// The pieces serialWrapped() must send for `text` under `tag`, from greedyWrap(): the first
/// `[tag] `, every later one `[tag] ... `, each ending in its one '\n'. The content of a piece ends
/// at kSerialLineBytes - 1 (its '\n' takes the last byte).
std::vector<std::string> greedyPieces(const std::string& tag, const std::string& text) {
    const std::string head = "[" + tag + "] ";
    const std::size_t kLast = kSerialLineBytes - 1;
    std::vector<std::string> out;
    const std::vector<std::string> lines =
        greedyWrap(text, kLast - head.size(), kLast - head.size() - 4);
    for (std::size_t i = 0; i < lines.size(); ++i) {
        out.push_back((i == 0 ? head : head + "... ") + lines[i] + "\n");
    }
    return out;
}

/// `text` with every string and character literal's CONTENTS removed (the quotes kept), for a pin
/// on CODE: a printed sentence that says "printf(" is not a call.
std::string withoutLiterals(const std::string& code) {
    std::string out;
    char quote = 0;
    for (std::size_t i = 0; i < code.size(); ++i) {
        const char c = code[i];
        if (quote != 0) {
            if (c == '\\' && i + 1 < code.size()) {
                ++i;
            } else if (c == quote) {
                quote = 0;
                out += c;
            }
            continue;
        }
        if (c == '\'' && source_pins::digitSeparatorAt(code, i)) {
            out += c;
            continue;
        }
        if (c == '"' || c == '\'') {
            quote = c;
        }
        out += c;
    }
    return out;
}

/// Every call of `name` in `code`: `name` not inside a longer identifier (so snprintf is not
/// printf), followed — after spaces — by `(`.
std::size_t callsOf(const std::string& code, const std::string& name) {
    std::size_t n = 0;
    for (std::size_t at = code.find(name); at != std::string::npos;
         at = code.find(name, at + 1)) {
        const bool startOk = at == 0 || !(std::isalnum(static_cast<unsigned char>(code[at - 1]))
                                          || code[at - 1] == '_');
        std::size_t after = at + name.size();
        while (after < code.size() && (code[after] == ' ' || code[after] == '\n')) {
            ++after;
        }
        if (startOk && after < code.size() && code[after] == '(') {
            ++n;
        }
    }
    return n;
}

/// Every use of the identifier `name` in `code` (not inside a longer identifier).
std::size_t usesOf(const std::string& code, const std::string& name) {
    std::size_t n = 0;
    for (std::size_t at = code.find(name); at != std::string::npos;
         at = code.find(name, at + 1)) {
        const auto ident = [](char c) {
            return std::isalnum(static_cast<unsigned char>(c)) != 0 || c == '_';
        };
        const bool startOk = at == 0 || !ident(code[at - 1]);
        const std::size_t end = at + name.size();
        const bool endOk = end >= code.size() || !ident(code[end]);
        if (startOk && endOk) {
            ++n;
        }
    }
    return n;
}

}  // namespace

// ═══ ProsCharSink ═══════════════════════════════════════════════════════════════════════

TEST_CASE("ProsCharSink(): the robot's sink writes fd 1, ONE write() per line, the text verbatim") {
    // BUG CAUGHT: the sink back to fwrite+fflush on stdout (each fwrite a separate trip through a
    // buffer every task shares, HA-189), or a line split into two calls — the text, then "\n" —
    // which two tasks interleave between. Both are MUTATIONS run against this case (SER1-PROGRESS):
    // each shows as a datagram count other than one per line.
    const std::string line300 = "[t=  1.00] [SES] " + run(282) + "\n";
    REQUIRE(line300.size() == 300);
    const Captured c = capture([&] {
        ProsCharSink sink;
        sink.write(line300);
        sink.write("[t=  1.01] [SES] second\n");
        sink.write("no newline: verbatim, the FORMATTER frames");
    });
    REQUIRE(c.ok);
    REQUIRE(c.calls.size() == 3);
    CHECK(c.calls[0] == line300);  // the whole 300-byte line in ONE call
    CHECK(c.calls[1] == "[t=  1.01] [SES] second\n");
    CHECK(c.calls[2] == "no newline: verbatim, the FORMATTER frames");  // nothing added
}

TEST_CASE("ProsCharSink(): a text longer than kSerialWriteMaxBytes goes out in pieces of at most that, in order") {
    // BUG CAUGHT (MUTATIONS, the review of 2026-10-15): the sink offering a text of ANY length to
    // ONE write(). On the brain a frame waits until it fits PROS's 2047-byte stream WHOLE, holding
    // PROS's write mutex, so a write of more than ~2 KB waits for ever and every later serial line
    // with it (A4 register HA-191); and PROS copies each frame onto the writer's stack. writeWhole()
    // caps each call at kSerialWriteMaxBytes; every line shulib prints (Line::kCapacity) still fits
    // ONE call — pinned here at compile time and by the 384-byte line below.
    static_assert(shulib::diag::lineformat::Line::kCapacity <= kSerialWriteMaxBytes,
                  "a diagnostic line must stay ONE write() call");
    CHECK(kSerialWriteMaxBytes == 512);
    std::string big;
    for (std::size_t i = 0; i < 1500; ++i) {
        big += static_cast<char>('a' + (i * 7) % 26);  // patterned: a reordered piece shows
    }
    const std::string line384 = run(shulib::diag::lineformat::Line::kCapacity - 1) + "\n";
    const Captured c = capture([&] {
        ProsCharSink sink;
        sink.write(line384);
        sink.write(run(kSerialWriteMaxBytes, 'm'));      // exactly the cap: ONE call
        sink.write(run(kSerialWriteMaxBytes + 1, 'n'));  // one over: two calls
        sink.write(big);
    });
    REQUIRE(c.ok);
    REQUIRE(c.calls.size() == 1 + 1 + 2 + 3);
    CHECK(c.calls[0] == line384);
    CHECK(c.calls[1] == run(kSerialWriteMaxBytes, 'm'));
    CHECK(c.calls[2] == run(kSerialWriteMaxBytes, 'n'));
    CHECK(c.calls[3] == "n");
    CHECK(c.calls[4] == big.substr(0, 512));
    CHECK(c.calls[5] == big.substr(512, 512));
    CHECK(c.calls[6] == big.substr(1024));
}

TEST_CASE("ProsCharSink(): default-constructible, cannot throw, holds no buffer (the provider's usb_{})") {
    // BUG CAUGHT: a line built in a MEMBER buffer of the sink — two tasks logging through one sink
    // would share it (the brief's landmine); and A3's default constructor lost, which
    // hal/pros/device_provider.hpp's `ProsCharSink usb_{};` needs unchanged (MECH-A1 builds beside).
    static_assert(std::is_nothrow_default_constructible_v<ProsCharSink>);
    static_assert(sizeof(ProsCharSink) <= 2 * sizeof(void*),
                  "a vtable pointer and a descriptor: no room for a line buffer");
    struct Holder {
        ProsCharSink usb_{};
    };
    Holder h;
    (void)h;
    const std::string provider = source_pins::readFile("include/shulib/hal/pros/device_provider.hpp");
    CHECK(source_pins::count(source_pins::stripComments(provider), "ProsCharSink usb_{};") == 1);
}

TEST_CASE("ProsCharSink(FILE*): a byte left in the FILE's buffer is NOT sent ahead of the line") {
    // BUG CAUGHT (the brief's amendment A6): the sink back to fwrite+fflush — it would push the
    // FILE's pending "B" out in front of its own line ("BC\n"); one direct write() leaves the
    // FILE's buffer alone and sends exactly "C\n". MUTATION: fwrite+fflush -> "BC\n", RED.
    std::FILE* f = std::tmpfile();
    REQUIRE(f != nullptr);
    std::string got;
    {
        ProsCharSink sink{f};
        REQUIRE(std::fwrite("B", 1, 1, f) == 1);  // a regular file: fully buffered, stays pending
        sink.write("C\n");
        got = fileBytes(f);
    }
    CHECK(got == "C\n");
    std::fclose(f);
}

TEST_CASE("ProsCharSink(FILE*): flushes the FILE ONCE at construction, so earlier text goes first") {
    // BUG CAUGHT: the constructor's one fflush removed (A3) — text printed into the FILE before the
    // sink existed would be overtaken by the sink's first direct write ("C\nA" instead of "AC\n").
    // MUTATION: the constructor's fflush removed -> "C\n", RED.
    std::FILE* f = std::tmpfile();
    REQUIRE(f != nullptr);
    REQUIRE(std::fwrite("A", 1, 1, f) == 1);  // pending BEFORE the sink is built
    std::string got;
    {
        ProsCharSink sink{f};
        sink.write("C\n");
        got = fileBytes(f);
    }
    CHECK(got == "AC\n");
    std::fclose(f);
}

TEST_CASE("ProsCharSink(FILE*): a null FILE, or one with no descriptor, is a loud contract breach") {
    // BUG CAUGHT: fileno(nullptr) is undefined behaviour, and a FILE with no descriptor (fmemopen)
    // would make every write fail SILENTLY — both refused at construction instead.
    CHECK_THROWS_AS(ProsCharSink{static_cast<std::FILE*>(nullptr)}, shulib::PreconditionError);
    char mem[16] = {};
    std::FILE* m = ::fmemopen(mem, sizeof mem, "w");
    REQUIRE(m != nullptr);
    REQUIRE(::fileno(m) < 0);  // the premise: glibc's memory stream has no descriptor
    CHECK_THROWS_AS(ProsCharSink{m}, shulib::PreconditionError);
    std::fclose(m);
}

// ═══ writeWhole(): the loop, through the ONE seam ════════════════════════════════════════

namespace {

/// A write fake: records each call (fd, the bytes it was offered), takes at most `perCall` bytes,
/// and returns `failWith` from call `failAt` on (0-based; -1 = never). It CAPS its calls: past
/// kCap it returns the whole size, so a loop that spins on 0 or -1 ENDS, and fails on the count.
struct WriteFake {
    static constexpr int kCap = 50;
    std::size_t perCall = static_cast<std::size_t>(-1);
    int failAt = -1;
    ssize_t failWith = 0;
    ssize_t claim = -1;  ///< >= 0: return this instead of the bytes taken (a lying device)
    std::vector<int> fds;
    std::vector<std::string> offered;
    std::string taken;

    ssize_t operator()(int fd, const char* data, std::size_t size) {
        fds.push_back(fd);
        offered.emplace_back(data, size);
        const int call = static_cast<int>(offered.size()) - 1;
        if (call >= kCap) {
            return static_cast<ssize_t>(size);  // the cap: a spinning loop ends here
        }
        if (failAt >= 0 && call >= failAt) {
            return failWith;
        }
        const std::size_t n = size < perCall ? size : perCall;
        taken.append(data, n);
        return claim >= 0 ? claim : static_cast<ssize_t>(n);
    }
};

}  // namespace

TEST_CASE("writeWhole(): a whole line in ONE call when the device takes it") {
    WriteFake fake;
    const std::string line = run(300) + "\n";
    CHECK(writeWhole(7, line, fake) == line.size());
    REQUIRE(fake.offered.size() == 1);
    CHECK(fake.offered[0] == line);
    CHECK(fake.fds == std::vector<int>{7});  // the descriptor passed through
    WriteFake none;
    CHECK(writeWhole(7, std::string_view{}, none) == 0);
    CHECK(none.offered.empty());  // nothing to send: no call at all
}

TEST_CASE("writeWhole(): a SHORT write sends the REMAINDER, never the start again") {
    // BUG CAUGHT (MUTATION, the brief's §1.4): the loop re-sending from the start (`bytes.data()`
    // for `bytes.data() + sent`) — a 300-byte line arriving as its first 100 bytes three times.
    WriteFake fake;
    fake.perCall = 100;
    const std::string line = run(100, 'a') + run(100, 'b') + run(99, 'c') + "\n";
    CHECK(writeWhole(1, line, fake) == 300);
    REQUIRE(fake.offered.size() == 3);
    CHECK(fake.offered[0] == line);              // the whole line offered first
    CHECK(fake.offered[1] == line.substr(100));  // then exactly what was left
    CHECK(fake.offered[2] == line.substr(200));
    CHECK(fake.taken == line);                   // every byte once, in order
}

TEST_CASE("writeWhole(): a write that returns 0 or -1 STOPS the loop — it can never spin") {
    // BUG CAUGHT (MUTATIONS, the brief's amendment A6): the loop continuing on 0 (`wrote < 0`) or on
    // -1 (`wrote == 0`) — on the brain a refused write would retry for ever inside a task that
    // logs. The fake caps its calls, so such a mutant FAILS on the count instead of hanging.
    for (const ssize_t bad : {ssize_t{0}, ssize_t{-1}}) {
        CAPTURE(bad);
        WriteFake fake;
        fake.failAt = 0;
        fake.failWith = bad;
        CHECK(writeWhole(1, "a whole line\n", fake) == 0);
        CHECK(fake.offered.size() == 1);
        WriteFake later;  // a short write, THEN the failure: what was sent is counted, no more
        later.perCall = 4;
        later.failAt = 1;
        later.failWith = bad;
        CHECK(writeWhole(1, "a whole line\n", later) == 4);
        CHECK(later.offered.size() == 2);
        CHECK(later.taken == "a wh");
    }
}

TEST_CASE("writeWhole(): a device claiming MORE than it was offered took what it was offered — no read past the text") {
    // BUG CAUGHT (MUTATION): the claim counted as the WHOLE text — once a long text goes in pieces
    // (the ceiling, below), a lying first call would end the loop with the later pieces never offered
    // and the count saying they were sent.
    WriteFake fake;
    fake.claim = 500;
    CHECK(writeWhole(1, "short\n", fake) == 6);
    CHECK(fake.offered.size() == 1);
    WriteFake liar;
    liar.claim = 600;  // more than any piece it is offered
    const std::string text = run(1200);
    CHECK(writeWhole(1, text, liar) == 1200);
    REQUIRE(liar.offered.size() == 3);
    CHECK(liar.offered[2] == text.substr(1024));
}

TEST_CASE("writeWhole(): no call is offered more than kSerialWriteMaxBytes — a long text goes in pieces") {
    // BUG CAUGHT (MUTATIONS, the review of 2026-10-15): the ceiling removed (one call offered the
    // whole text: on the brain a write past ~2 KB waits for ever holding PROS's mutex, HA-191), or
    // off by one. And with a short-writing device the pieces still carry every byte once, in order.
    WriteFake fake;
    std::string text;
    for (std::size_t i = 0; i < 1200; ++i) {
        text += static_cast<char>('a' + (i * 5) % 26);
    }
    CHECK(writeWhole(3, text, fake) == 1200);
    REQUIRE(fake.offered.size() == 3);
    CHECK(fake.offered[0] == text.substr(0, 512));
    CHECK(fake.offered[1] == text.substr(512, 512));
    CHECK(fake.offered[2] == text.substr(1024));
    CHECK(fake.taken == text);
    WriteFake exact;
    CHECK(writeWhole(3, run(kSerialWriteMaxBytes), exact) == kSerialWriteMaxBytes);
    CHECK(exact.offered.size() == 1);  // exactly the ceiling: still ONE call
    WriteFake shortish;
    shortish.perCall = 100;
    CHECK(writeWhole(3, text, shortish) == 1200);
    CHECK(shortish.taken == text);
    CHECK(shortish.offered.size() == 12);
    for (const std::string& o : shortish.offered) {
        CHECK(o.size() <= kSerialWriteMaxBytes);
    }
}

// ═══ serialLine() ══════════════════════════════════════════════════════════════════════

TEST_CASE("serialLine(): ONE write() per line, and exactly one trailing newline") {
    // BUG CAUGHT: `printf("…\n")` (the compiler's puts: two writes in newlib); a line without its
    // newline (MUTATION: the guarantee removed — the next line would join it); and a doubled one
    // for a format that already ends in '\n'.
    // The "%s", "" call is also the n == 0 guard's case: without it serialLine reads buf[-1] — a
    // stack read the suite cannot see, but -fsanitize=address does (the coordinator runs this WHOLE
    // file under it; SER1-PROGRESS, the review of 2026-10-15).
    const char* volatile nullFormat = nullptr;  // not a constant: no compile-time format check
    const Captured c = capture([&] {
        serialLine("[T] %d %s", 42, "words");
        serialLine("[T] already ended\n");
        serialLine("%s", "");
        // A null format on purpose (the header's contract: an empty line). The host compiler's
        // -Wformat-security refuses a non-literal format with no arguments — here, deliberately.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-security"
        serialLine(nullFormat);
#pragma GCC diagnostic pop
        serialLine("\n[T] booted");  // a banner's blank line rides in the same call
        serialLine("[T] a\nb");      // an inner newline is the caller's: still ONE call
    });
    REQUIRE(c.ok);
    REQUIRE(c.calls.size() == 6);
    CHECK(c.calls[0] == "[T] 42 words\n");
    CHECK(c.calls[1] == "[T] already ended\n");
    CHECK(c.calls[2] == "\n");
    CHECK(c.calls[3] == "\n");
    CHECK(c.calls[4] == "\n[T] booted\n");
    CHECK(c.calls[5] == "[T] a\nb\n");
}

TEST_CASE("serialLine(): kSerialLineBytes is 256 — the boundary sends WHOLE, one past it is CUT, visibly") {
    // BUG CAUGHT: the buffer sized kSerialLineBytes instead of + 1 (a 255-character text from a
    // format ending "\n" would be cut while the same text without the "\n" was not); an off-by-one
    // that sends 257 bytes or drops the newline at the boundary; a silent cut.
    CHECK(kSerialLineBytes == 256);
    const std::string t255 = run(255);
    const std::string t256 = run(256);
    const Captured c = capture([&] {
        serialLine("%s", t255.c_str());     // 255 + the added '\n' = 256: whole
        serialLine("%s\n", t255.c_str());   // 255 + its own '\n' = 256: whole
        serialLine("%s", t256.c_str());     // 256 and no room for '\n': CUT
        serialLine("%s\n", t256.c_str());   // 257: CUT
    });
    REQUIRE(c.ok);
    REQUIRE(c.calls.size() == 4);
    CHECK(c.calls[0] == t255 + "\n");
    CHECK(c.calls[1] == t255 + "\n");
    const std::string cut =
        run(kSerialLineBytes - kSerialCutMarker.size()) + std::string(kSerialCutMarker);
    CHECK(c.calls[2] == cut);
    CHECK(c.calls[3] == cut);
    CHECK(c.calls[2].size() == kSerialLineBytes);
}

TEST_CASE("serialLine(): a format vsnprintf REFUSES sends an empty line — never the buffer's stale bytes") {
    // BUG CAUGHT (MUTATION, the review of 2026-10-15): the `formatted < 0` guard removed — vsnprintf's
    // −1 becomes SIZE_MAX, the cut branch fires, and 247 bytes of uninitialised stack go to the
    // serial ahead of the cut marker. glibc (and newlib) refuse a `%ls` character the C locale
    // cannot encode; nothing in the suite sets a locale.
    const wchar_t accented[] = {static_cast<wchar_t>(0x00e9), 0};
    const Captured c = capture([&] { serialLine("[T] %ls", accented); });
    REQUIRE(c.ok);
    REQUIRE(c.calls.size() == 1);
    CHECK(c.calls[0] == "\n");
}

TEST_CASE("serialLine(): a 2,000-byte %s sends exactly kSerialLineBytes, the text's start intact") {
    // BUG CAUGHT: a buffer overrun on a long argument (vsnprintf bounded by `sizeof buf`, pinned
    // below) — no sanitizer job exists (the brief's amendment A6), so the coordinator runs this
    // WHOLE file under -fsanitize=address (not this case alone: the empty format's guard is another
    // stack read only ASan sees); here, the bytes sent are exactly the buffer's.
    std::string big;
    for (int i = 0; big.size() < 2000; ++i) {
        big += std::to_string(i % 10);
    }
    const Captured c = capture([&] { serialLine("[BIG] %s", big.c_str()); });
    REQUIRE(c.ok);
    REQUIRE(c.calls.size() == 1);
    CHECK(c.calls[0].size() == kSerialLineBytes);
    CHECK(c.calls[0].compare(0, 6, "[BIG] ") == 0);
    CHECK(c.calls[0].compare(6, 100, big, 0, 100) == 0);
    CHECK(c.calls[0].substr(kSerialLineBytes - kSerialCutMarker.size()) == kSerialCutMarker);
}

TEST_CASE("serialLine(): two tasks printing at once — every line arrives whole, each exactly once") {
    // BUG CAUGHT: the line built in a STATIC buffer (the brief's landmine) — two threads formatting
    // into one buffer send each other's text, torn or doubled. The host's glibc keeps each write()
    // whole on a datagram socket, so only shulib's own sharing can tear a line here. (A race shows
    // with a probability, not a certainty: the mutation's observed rate is in SER1-PROGRESS.)
    constexpr int kLines = 400;
    StdoutCapture cap{true};  // blocking, with a send timeout: the reader keeps up, nothing lost
    std::vector<std::string> got;
    std::atomic<bool> readerDone{false};
    bool ok = cap.ok();
    if (ok) {
        timeval tv{2, 0};  // a reader that waits at most 2 s for a datagram: never a hang
        ok = ::setsockopt(cap.readEnd(), SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv) == 0;
    }
    if (ok) {
        // The reader runs beside the writers and reads until "END".
        std::thread reader([&] {
            std::vector<char> buf(4096);
            for (;;) {
                const ssize_t n = ::recv(cap.readEnd(), buf.data(), buf.size(), 0);
                if (n <= 0) {
                    break;
                }
                std::string s(buf.data(), static_cast<std::size_t>(n));
                if (s == "END\n") {
                    break;
                }
                got.push_back(std::move(s));
            }
            readerDone = true;
        });
        const auto writer = [](char id) {
            const std::string pad = run(150, id);
            for (int i = 0; i < kLines; ++i) {
                serialLine("[T%c] line %04d %s", id, i, pad.c_str());
                if (i % 32 == 0) {
                    std::this_thread::yield();
                }
            }
        };
        std::thread a(writer, 'a');
        std::thread b(writer, 'b');
        a.join();
        b.join();
        serialLine("END");
        reader.join();
    }
    cap.restore();
    REQUIRE(ok);
    CHECK(readerDone);
    std::set<std::string> seen;
    std::size_t torn = 0;
    for (const std::string& s : got) {
        // whole: "[Tx] line NNNN " + 150 x + "\n", one id throughout
        const bool shape = s.size() == 15 + 150 + 1 && s.compare(0, 2, "[T") == 0 && s[3] == ']'
                           && s.find_first_not_of(s[2], 15) == 165 && s.back() == '\n';
        if (!shape) {
            ++torn;
            continue;
        }
        seen.insert(s.substr(0, 14));
    }
    CHECK(torn == 0);
    CHECK(got.size() == 2 * kLines);
    CHECK(seen.size() == 2 * kLines);  // each line exactly once: none lost, none doubled
}

// ═══ serialWrapped() ═══════════════════════════════════════════════════════════════════

TEST_CASE("serialWrapped(): a short text is one `[tag] ` line in one call; parts join as one text") {
    const Captured c = capture([&] {
        serialWrapped("TAG", "hello");
        serialWrapped("TAG", {"REFUSED: ", "headline", " -- ", "why"});
        serialWrapped("TAG", "");
        serialWrapped(nullptr, "no tag: the line carries its own");
        serialWrapped("ABCDEFGHIJKLMNOPQRSTUVWXYZ", "a long tag");
        serialWrapped("", "an empty tag");
    });
    REQUIRE(c.ok);
    REQUIRE(c.calls.size() == 6);
    CHECK(c.calls[0] == "[TAG] hello\n");
    CHECK(c.calls[1] == "[TAG] REFUSED: headline -- why\n");
    CHECK(c.calls[2] == "[TAG] \n");  // an empty text still says something
    CHECK(c.calls[3] == "no tag: the line carries its own\n");
    CHECK(c.calls[4] == "[" + std::string("ABCDEFGHIJKLMNOPQRSTUVWXYZ").substr(0, kSerialTagBytes)
                            + "] a long tag\n");
    CHECK(c.calls[5] == "[] an empty tag\n");
}

TEST_CASE("serialWrapped(): a newline in the text ends a line there; a blank line stays blank") {
    const Captured c = capture([&] {
        serialWrapped("T", "one\n\ntwo");
        serialWrapped("T", "ends\n");
    });
    REQUIRE(c.ok);
    REQUIRE(c.calls.size() == 4);
    CHECK(c.calls[0] == "[T] one\n");
    CHECK(c.calls[1] == "[T] \n");
    CHECK(c.calls[2] == "[T] two\n");
    CHECK(c.calls[3] == "[T] ends\n");  // no empty line after a text that ends in '\n'
}

TEST_CASE("serialWrapped(): a newline AFTER a wrapped piece starts a fresh line — a short one is still sent") {
    // BUG CAUGHT (MUTATION, the review of 2026-10-15): the newline resetting where the piece ENDS
    // but not where its text STARTS — a continuation's start (`[T] ... `) survives the newline, and
    // a following line of four characters or fewer is never sent.
    const Captured c = capture([&] {
        serialWrapped("T", run(300) + "\nab");
        serialWrapped("T", run(300) + "\nabcd\nlast");
    });
    REQUIRE(c.ok);
    REQUIRE(c.calls.size() == 3 + 4);
    CHECK(c.calls[0] == "[T] " + run(251) + "\n");
    CHECK(c.calls[1] == "[T] ... " + run(49) + "\n");
    CHECK(c.calls[2] == "[T] ab\n");
    CHECK(c.calls[5] == "[T] abcd\n");
    CHECK(c.calls[6] == "[T] last\n");
}

TEST_CASE("serialWrapped(): the boundary — 256 bytes is one piece, one character more is two") {
    // BUG CAUGHT: an off-by-one in the "full" test (`len >= kLast`) — a 257-byte piece, or a
    // needless break of a line that fits exactly.
    const std::string fits = run(kSerialLineBytes - 4 - 1);  // "[T] " + text + '\n' = 256
    const std::string over = fits + "y";
    const Captured c = capture([&] {
        serialWrapped("T", fits);
        serialWrapped("T", over);
        serialWrapped("T", fits + "\nz");  // full exactly, THEN a newline: still one piece
    });
    REQUIRE(c.ok);
    REQUIRE(c.calls.size() == 5);
    CHECK(c.calls[0] == "[T] " + fits + "\n");
    CHECK(c.calls[0].size() == kSerialLineBytes);
    CHECK(c.calls[1] == "[T] " + fits + "\n");  // hard break: no space to break at
    CHECK(c.calls[2] == "[T] ... y\n");
    CHECK(c.calls[3] == "[T] " + fits + "\n");  // the newline lands in the full piece's last byte
    CHECK(c.calls[4] == "[T] z\n");
}

TEST_CASE("serialWrapped(): a long text of words WRAPS at spaces — never cut, nothing lost or doubled") {
    // BUG CAUGHT: the carry starting AT the space instead of after it (a doubled space, or the
    // space leading the next piece); the carried words copied over themselves (memcpy where the
    // ranges overlap); a break that cut mid-word with a space available; a piece past 256 bytes.
    std::string text;
    const char* words[] = {"a", "brief", "wrap", "of", "words", "that", "runs", "long", "enough",
                           "to", "need", "several", "pieces", "--", "punctuation,", "too."};
    for (int i = 0; text.size() < 1500; ++i) {
        if (!text.empty()) {
            text += ' ';
        }
        text += words[(i * 7) % 16];
    }
    const Captured c = capture([&] { serialWrapped("DRV", text); });
    REQUIRE(c.ok);
    std::vector<std::string> texts;
    std::string why;
    REQUIRE_MESSAGE(piecesText(c.calls, "[DRV] ", texts, why), why);
    CHECK(c.calls == greedyPieces("DRV", text));  // FILLED: as many whole words as fit, each piece
    CHECK(rejoins(texts, text));
    // every break but the last is at a space (the text has one every few characters)
    std::size_t pos = 0;
    for (std::size_t i = 0; i + 1 < texts.size(); ++i) {
        pos += texts[i].size();
        CHECK(text[pos] == ' ');
        ++pos;
    }
}

TEST_CASE("serialWrapped(): every piece holds as many whole words as fit — the greedy fill, against an independent wrap") {
    // BUG CAUGHT (MUTATION, the review of 2026-10-15): the scan for a break going on past the LAST
    // space that fits and settling on the earliest — still valid output, but nearly one word per
    // piece ("shulib Drive"'s 408-character run status, printed on every boot, went from 2 pieces
    // to 31). The "words" case above only rejoined the pieces; this one compares every piece with
    // greedyWrap(), which shares no code with serialWrapped(), over texts of varied word lengths
    // (a fixed generator, so a failure repeats) under tags of every length class.
    std::uint32_t seed = 12345;
    const auto next = [&seed](std::uint32_t mod) {
        seed = seed * 1103515245u + 12345u;
        return (seed >> 16) % mod;
    };
    std::size_t texts = 0;
    for (const std::string tag : {"", "T", "DRV", "ABCDEFGHIJKLMNOP"}) {
        for (int round = 0; round < 25; ++round) {
            std::string text;
            const std::size_t target = 200 + next(1400);
            while (text.size() < target) {
                if (!text.empty()) {
                    text += ' ';
                }
                const std::size_t len = 1 + next(round % 5 == 0 ? 60 : 14);
                for (std::size_t i = 0; i < len; ++i) {
                    text += static_cast<char>('a' + next(26));
                }
            }
            const Captured c = capture([&] { serialWrapped(tag.c_str(), text); });
            REQUIRE(c.ok);
            CHECK_MESSAGE(c.calls == greedyPieces(tag, text), "tag `", tag, "`, text: ", text);
            ++texts;
        }
    }
    CHECK(texts == 100);
    // And one sentence a reader would recognise: "shulib Drive"'s run status (408 characters) is
    // TWO pieces under its tag, the first nearly full.
    const std::string status =
        "THIS VERSION of \"shulib Drive\" (T4 Part 3: robot two's driver file on robot::Robot) has not "
        "run on hardware. The program it replaced in slot 1 drove robot two open-loop on 2026-09-17, "
        "and the library program it retired drove it from the calibration record on 2026-09-25. No "
        "library-commanded motion has run (R3b open). Its claws (the pivots on 6 and 7, the pneumatic "
        "claw on 3-wire A) have never been driven.";
    const Captured s = capture([&] { serialWrapped("DRV", status); });
    REQUIRE(s.ok);
    REQUIRE(s.calls.size() == 2);
    CHECK(s.calls[0].size() > 240);
    CHECK(s.calls == greedyPieces("DRV", status));
}

TEST_CASE("serialWrapped(): a space right after a FULL piece is the break — no leading space, no empty piece, no word moved") {
    // BUG CAUGHT (MUTATION, the review of 2026-10-15): a full piece whose last word ends exactly at
    // its last byte, followed by a space. Before the fix the space started the next piece (`[T] ...
    // ` + " next": a doubled space), or the scan carried that last word — which FIT — to the next
    // piece; and a scan allowed to break AT a continuation's first character (`k >= start`) could
    // then send an empty `[T] ... ` piece. Now the piece breaks at that space and drops it.
    const Captured c = capture([&] {
        serialWrapped("T", run(251) + " next");                       // no earlier space
        serialWrapped("T", run(245) + " abcde next");                 // an earlier space
        serialWrapped("T", run(251) + " " + run(246, 'y') + " z");    // the review's empty-piece shape
    });
    REQUIRE(c.ok);
    REQUIRE(c.calls.size() == 2 + 2 + 3);
    CHECK(c.calls[0] == "[T] " + run(251) + "\n");
    CHECK(c.calls[1] == "[T] ... next\n");
    CHECK(c.calls[2] == "[T] " + run(245) + " abcde\n");  // the word that fit stays
    CHECK(c.calls[3] == "[T] ... next\n");
    CHECK(c.calls[4] == "[T] " + run(251) + "\n");
    CHECK(c.calls[5] == "[T] ... " + run(246, 'y') + "\n");
    CHECK(c.calls[6] == "[T] ... z\n");
}

TEST_CASE("serialWrapped(): after a FULL piece, a SECOND space is the text's — kept, never an empty piece; a newline sends no empty piece") {
    // BUG CAUGHT (MUTATION H5, SER1's verification of 2026-10-15): the break after a full piece
    // drops ONE space; a double space leaves the second at the continuation's first character. A
    // scan allowed to break AT that character (`k >= start`, not `k > start`) then sends an EMPTY
    // `[T] ... ` piece and moves the rest to a third — the fix round had called H5 equivalent with a
    // tag; this shape is the case its proof missed.
    // And the same break followed by a NEWLINE: the line's continuation holds nothing, so nothing is
    // sent for it (before this fix the newline sent `[T] ... ` alone, 9 bytes, where the end of the
    // text sent nothing). A newline on a FIRST piece still sends it, blank: the blank-line case.
    const Captured c = capture([&] {
        serialWrapped("T", run(251) + "  " + run(260, 'y'));   // the verifier's shape
        serialWrapped("T", run(251) + " \nab");                // a full piece, a space, a newline
        serialWrapped("T", run(251) + "  \nab");               // two spaces, then the newline
        serialWrapped(nullptr, run(255) + "  " + run(260, 'y'));  // no tag: the same, from index 0
        serialWrapped("DRV", run(247) + "  " + run(260, 'y'));   // the double space INSIDE a piece
    });
    REQUIRE(c.ok);
    REQUIRE(c.calls.size() == 3 + 2 + 3 + 3 + 3);
    CHECK(c.calls[0] == "[T] " + run(251) + "\n");
    CHECK(c.calls[1] == "[T] ...  " + run(246, 'y') + "\n");  // the second space kept, then 246 y
    CHECK(c.calls[2] == "[T] ... " + run(14, 'y') + "\n");
    CHECK(c.calls[3] == "[T] " + run(251) + "\n");
    CHECK(c.calls[4] == "[T] ab\n");                           // no `[T] ... ` between them
    CHECK(c.calls[5] == "[T] " + run(251) + "\n");
    CHECK(c.calls[6] == "[T] ...  \n");                         // a space is text: sent, as the end of
    CHECK(c.calls[7] == "[T] ab\n");                           // the text would send it
    CHECK(c.calls[8] == run(255) + "\n");
    CHECK(c.calls[9] == "...  " + run(250, 'y') + "\n");
    CHECK(c.calls[10] == "... " + run(10, 'y') + "\n");
    // The scan breaks at the LAST space, so here the OTHER space ends the first piece instead.
    CHECK(c.calls[11] == "[DRV] " + run(247) + " \n");
    CHECK(c.calls[12] == "[DRV] ... " + run(245, 'y') + "\n");
    CHECK(c.calls[13] == "[DRV] ... " + run(15, 'y') + "\n");
    for (std::size_t i = 0; i < c.calls.size(); ++i) {
        CHECK_MESSAGE(c.calls[i].size() <= kSerialLineBytes, "call ", i);
        // every text here is x, y and "ab": a call ENDING `... \n` is an empty continuation, any tag
        const std::string& call = c.calls[i];
        CHECK_MESSAGE((call.size() < 5 || call.compare(call.size() - 5, 5, "... \n") != 0), "call ", i,
                      " is an empty continuation: ", call);
    }
    // Nothing lost: each line's pieces rejoin to its text, the ONE space at each break dropped.
    std::vector<std::string> texts;
    std::string why;
    REQUIRE_MESSAGE(piecesText({c.calls[0], c.calls[1], c.calls[2]}, "[T] ", texts, why), why);
    CHECK(rejoins(texts, run(251) + "  " + run(260, 'y')));
    REQUIRE_MESSAGE(piecesText({c.calls[11], c.calls[12], c.calls[13]}, "[DRV] ", texts, why), why);
    CHECK(rejoins(texts, run(247) + "  " + run(260, 'y')));
}

TEST_CASE("serialWrapped(): a word longer than a piece starts a piece of its own, then breaks hard") {
    // The rule: break at the LAST space that leaves the rest room in the next piece — so the long
    // word moves whole to a fresh piece (as a word processor would), and only a word longer than a
    // whole piece is broken mid-word. Every character kept, in order.
    // The word is PATTERNED, never one repeated letter: moving 245 carried characters two places
    // LEFT overlaps itself, and a copy in the wrong order (MUTATION: back to front, where memmove
    // copies front to back) garbles them — invisibly, if every character were the same.
    std::string word;
    for (std::size_t i = 0; i < 600; ++i) {
        word += static_cast<char>('a' + (i * 7) % 26);
    }
    const Captured c = capture([&] { serialWrapped("T", {"start ", word, " end"}); });
    REQUIRE(c.ok);
    std::vector<std::string> texts;
    std::string why;
    REQUIRE_MESSAGE(piecesText(c.calls, "[T] ", texts, why), why);
    CHECK(rejoins(texts, "start " + word + " end"));
    REQUIRE(c.calls.size() == 4);
    CHECK(c.calls[0] == "[T] start\n");                          // the word moved to a fresh piece
    CHECK(c.calls[1] == "[T] ... " + word.substr(0, 247) + "\n");  // full, broken hard
    CHECK(c.calls[2] == "[T] ... " + word.substr(247, 247) + "\n");
    CHECK(c.calls[3] == "[T] ... " + word.substr(494) + " end\n");
}

TEST_CASE("serialWrapped(): the carry's room is STRICT — a space leaving exactly no room breaks hard") {
    // BUG CAUGHT (MUTATION): `len - (k + 1) < kLast - next` loosened to `<=`. With the tag "T" the
    // continuation's text starts at 8 and the content ends at 255; a space at index 7 leaves 247
    // characters to carry — exactly the room, with none for the character that forced the break.
    // The loosened test carries them, writes that character at index 255 and the next break's
    // newline at 256: one byte PAST the piece (a 257-byte line, and a stack write out of bounds).
    const std::string text = "abc " + run(260);
    const Captured c = capture([&] { serialWrapped("T", text); });
    REQUIRE(c.ok);
    std::vector<std::string> texts;
    std::string why;
    REQUIRE_MESSAGE(piecesText(c.calls, "[T] ", texts, why), why);
    CHECK(rejoins(texts, text));
    REQUIRE(c.calls.size() == 2);
    CHECK(c.calls[0].size() == kSerialLineBytes);  // the hard break: a full first piece
}

TEST_CASE("serialWrapped(): with no tag, a continuation still says `... `") {
    const std::string text = run(300, 'q');
    const Captured c = capture([&] { serialWrapped(nullptr, text); });
    REQUIRE(c.ok);
    REQUIRE(c.calls.size() == 2);
    CHECK(c.calls[0] == run(255, 'q') + "\n");
    CHECK(c.calls[1] == "... " + run(45, 'q') + "\n");
}

// ═══ Source pins: what keeps every src/ line on the one-call path ═══════════════════════

TEST_CASE("pin: char_sink.hpp is PROS-free — the CI guard SKIPS hal/pros/, so this is the only check") {
    const std::string src = source_pins::readFile("include/shulib/hal/pros/char_sink.hpp");
    const std::string code = source_pins::stripComments(src);
    CHECK(code.find("\"pros/") == std::string::npos);
    CHECK(code.find("<pros/") == std::string::npos);
    CHECK(code.find("pros::c::") == std::string::npos);
}

TEST_CASE("pin: serialLine()'s buffer is on the caller's stack, kSerialLineBytes + 1, and vsnprintf is bounded by it") {
    // BUG CAUGHT: a `static` buffer (two tasks share it — the brief's landmine; the threaded case
    // above catches it only with a probability), and vsnprintf given a size other than the
    // buffer's own (the 2,000-byte case's overrun).
    const std::string code =
        source_pins::stripComments(source_pins::readFile("include/shulib/hal/pros/char_sink.hpp"));
    const std::string line = source_pins::squeeze(source_pins::bodyOf(code, "void serialLine("));
    REQUIRE_FALSE(line.empty());
    CHECK(source_pins::count(line, "char buf[kSerialLineBytes + 1];") == 1);
    CHECK(source_pins::count(line, "std::vsnprintf(buf, sizeof buf, format, args)") == 1);
    CHECK(usesOf(withoutLiterals(line), "static") == 0);
    CHECK(usesOf(withoutLiterals(line), "thread_local") == 0);
    const std::string wrapped = source_pins::squeeze(
        source_pins::bodyOf(code, "void serialWrapped(const char* tag, std::initializer_list"));
    REQUIRE_FALSE(wrapped.empty());
    CHECK(source_pins::count(wrapped, "char piece[kSerialLineBytes];") == 1);
    CHECK(usesOf(withoutLiterals(wrapped), "static") == 0);
    CHECK(usesOf(withoutLiterals(wrapped), "thread_local") == 0);
}

TEST_CASE("pin: no src/ code prints through stdout's FILE — every line is ONE write() (SER1, HA-189)") {
    // BUG CAUGHT: a printf/puts/putchar/fputs/std::cout line, or an fflush(stdout), back in a
    // program — the census SER1 converted (20 printf sites, 14 fflush(stdout) lines). The src build
    // gate's step 9 checks the OBJECTS (it also sees what the compiler rewrites); this pin names
    // the file and the call on the host, before any ARM build. Comments and string literals are
    // stripped: a sentence that says "printf" is not a call. snprintf/vsnprintf into a buffer and
    // the SD card's fwrite/fflush on its own FILE stay allowed.
    std::size_t files = 0;
    for (const std::string& rel : source_pins::sourcesUnder("src")) {
        ++files;
        const std::string code =
            withoutLiterals(source_pins::stripComments(source_pins::readFile(rel)));
        for (const char* call :
             {"printf", "puts", "putchar", "fputs", "vprintf", "fprintf", "vfprintf", "perror", "putc",
              "fputc", "iprintf", "viprintf", "fiprintf", "putchar_unlocked", "wprintf", "vwprintf",
              "putwchar", "putwchar_unlocked", "psignal"}) {
            CHECK_MESSAGE(callsOf(code, call) == 0, rel, " calls ", call);
        }
        for (const char* name :
             {"stdin", "stdout", "stderr", "cout", "cerr", "clog", "wcout", "wcerr", "wclog", "__sf"}) {
            CHECK_MESSAGE(usesOf(code, name) == 0, rel, " names ", name);
        }
    }
    CHECK(files >= 20);  // the walk read the tree (src/ has 20+ sources)
}

TEST_CASE("pin: the bench tester's emit() — A2's named example — sends its line through serialWrapped") {
    // BUG CAUGHT (MUTATION, the brief's §1.4): bench_r3a.cpp's emit() back to
    // `std::printf("%s\n", line)`, which the compiler rewrites to puts — TWO writes in newlib.
    const std::string code = source_pins::stripComments(
        source_pins::readFile("src/programs/bench_r3a.cpp"));
    const std::string body = source_pins::squeeze(source_pins::bodyOf(code, "void emit(const char* line)"));
    REQUIRE_FALSE(body.empty());
    CHECK(source_pins::count(body, "hal::pros::serialWrapped(nullptr, line);") == 1);
    CHECK(callsOf(withoutLiterals(body), "printf") == 0);
}
